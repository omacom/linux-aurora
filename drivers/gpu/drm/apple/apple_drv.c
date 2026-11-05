// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */
/* Based on meson driver which is
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 * Copyright (C) 2014 Endless Mobile
 */

#include <linux/soc/apple/j613-display.h>
#include <linux/aperture.h>
#include <linux/component.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/ratelimit.h>
#include <linux/workqueue.h>
#include <linux/soc/apple/dp-tunnel.h>
#include <linux/soc/apple/display.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_mode.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_module.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_fixed.h>
#include <linux/pm.h>
#include <linux/suspend.h>

#include "apple_boot_display.h"
#include "apple_resume.h"
#include "dcp.h"
#include "plane.h"
#include "iomfb_m4.h"
#include "t8132_ext_drm.h"

#define DRIVER_NAME     "apple"
#define DRIVER_DESC     "Apple display controller DRM driver"

#define MAX_COPROCESSORS 8

struct apple_drm_private {
	struct drm_device drm;
	/*
	 * A T6041 panel refused the takeover trial: the boot framebuffer keeps
	 * the screen, the DRM device stays unregistered and its DCPs running.
	 */
	bool aborted;
	/* This suspend found every firmware already stopped. */
	bool s2r_skipped;
	/* A dark wake left the outputs as the suspend did, for the next resume. */
	bool dark_held;
	struct notifier_block pm_nb;
};

static int apple_drm_pm_notify(struct notifier_block *nb, unsigned long action, void *data);

/* Component readiness is later than platform probe/bind. */
static DEFINE_MUTEX(apple_g15_display_lock);
static struct device *apple_t6030_display;
static struct device *apple_t6031_display;
static struct device *apple_t8132_display;
static struct device *apple_t6034_display;

/* The publication slot of a G15/G16 SoC's internal display subsystem, or NULL. */
static struct device **apple_g15_display_slot(struct device *dev)
{
	if (of_device_is_compatible(dev->of_node, "apple,t6030-display-subsystem"))
		return &apple_t6030_display;
	if (of_device_is_compatible(dev->of_node, "apple,t6031-display-subsystem"))
		return &apple_t6031_display;
	if (of_device_is_compatible(dev->of_node, "apple,t8132-display-subsystem"))
		return &apple_t8132_display;
	if (of_device_is_compatible(dev->of_node, "apple,t6034-display-subsystem"))
		return &apple_t6034_display;
	return NULL;
}

static int apple_g15_display_publish(struct device *dev)
{
	struct device **slot = apple_g15_display_slot(dev);

	if (!slot)
		return 0;
	guard(mutex)(&apple_g15_display_lock);
	if (*slot)
		return *slot == dev ? 0 : -EBUSY;
	*slot = get_device(dev);
	return 0;
}

static void apple_g15_display_withdraw(struct device *dev)
{
	struct device **slot = apple_g15_display_slot(dev);
	struct device *old = NULL;

	if (!slot)
		return;
	scoped_guard(mutex, &apple_g15_display_lock) {
		if (*slot == dev) {
			old = *slot;
			*slot = NULL;
		}
	}
	put_device(old);
}

/* Initial G15/G16 firmware startup must follow successful native display setup.
 * The component's device lock does not serialize component bind/unbind, so
 * publish separately. This lock protects only publication and a device link;
 * no display calls or firmware RPC run while it is held.
 */
static int apple_g15_link_device(struct device *consumer, const char *machine,
				 const char *gpu, struct device **display)
{
	struct device_link *link;

	if (!of_machine_is_compatible(machine) || !consumer->of_node ||
	    !of_device_is_compatible(consumer->of_node, gpu))
		return -ENODEV;
	guard(mutex)(&apple_g15_display_lock);
	if (!*display)
		return -EPROBE_DEFER;
	link = device_link_add(consumer, *display, DL_FLAG_AUTOREMOVE_CONSUMER);
	return link ? 0 : -EINVAL;
}

int apple_drm_t6030_link_device(struct device *consumer)
{
	return apple_g15_link_device(consumer, "apple,t6030", "apple,agx-t6030",
				     &apple_t6030_display);
}
EXPORT_SYMBOL_GPL(apple_drm_t6030_link_device);

int apple_drm_t6031_link_device(struct device *consumer)
{
	return apple_g15_link_device(consumer, "apple,t6031", "apple,agx-t6031",
				     &apple_t6031_display);
}
EXPORT_SYMBOL_GPL(apple_drm_t6031_link_device);

int apple_drm_t8132_link_device(struct device *consumer)
{
	return apple_g15_link_device(consumer, "apple,t8132", "apple,agx-t8132",
				     &apple_t8132_display);
}
EXPORT_SYMBOL_GPL(apple_drm_t8132_link_device);

int apple_drm_t6034_link_device(struct device *consumer)
{
	if (!of_machine_is_compatible("apple,j514m"))
		return -ENODEV;
	return apple_g15_link_device(consumer, "apple,t6034", "apple,agx-t6034",
				     &apple_t6034_display);
}
EXPORT_SYMBOL_GPL(apple_drm_t6034_link_device);

DEFINE_DRM_GEM_DMA_FOPS(apple_fops);

#define DART_PAGE_SIZE 16384

static int apple_drm_gem_dumb_create(struct drm_file *file_priv,
                            struct drm_device *drm,
                            struct drm_mode_create_dumb *args)
{
        args->pitch = ALIGN(DIV_ROUND_UP(args->width * args->bpp, 8), 64);
        args->size = round_up(args->pitch * args->height, DART_PAGE_SIZE);

	return drm_gem_dma_dumb_create_internal(file_priv, drm, args);
}

#ifdef CONFIG_DRM_FBDEV_EMULATION
static int apple_drm_fbdev_probe(struct drm_fb_helper *helper,
				 struct drm_fb_helper_surface_size *sizes)
{
	struct drm_crtc *crtc;

	drm_for_each_crtc(crtc, helper->dev) {
		struct apple_dcp *dcp;

		if (t8132_ext_crtc(crtc))
			continue;
		dcp = platform_get_drvdata(to_apple_crtc(crtc)->dcp);
		if (dcp->fw_compat == DCP_FIRMWARE_H17P ||
		    dcp->fw_compat == DCP_FIRMWARE_V_27_0) {
			/*
			 * H17P and firmware 27 apply the stored fourth byte as
			 * straight alpha: an XRGB console with X = 0 stays black.
			 * ARGB makes fbcon write it opaque.
			 */
			sizes->surface_bpp = 32;
			sizes->surface_depth = 32;
			break;
		}
	}

	return drm_fbdev_dma_driver_fbdev_probe(helper, sizes);
}
#endif

/*
 * HDR is asked for by the compositor that set it, but connector properties
 * outlive their client.  When it exits, the next client (Plymouth at
 * shutdown, the fbdev console) would inherit HDR_OUTPUT_METADATA and BT.2020
 * and have its SDR frames shown as PQ, at the panel's peak.  So when a
 * former master closes and no client has taken over, take its outputs back
 * to SDR the way an HDR-off commit from it would.  A master that only drops
 * master for a VT switch keeps its state: it comes back without resending
 * it.
 */
static void apple_drm_reset_hdr(struct drm_device *dev)
{
	struct drm_modeset_acquire_ctx ctx;
	struct drm_connector_list_iter iter;
	struct drm_atomic_commit *state;
	struct drm_connector *conn;
	bool changed;
	int ret;

	drm_modeset_acquire_init(&ctx, 0);
	state = drm_atomic_commit_alloc(dev);
	if (!state)
		goto out_ctx;
	state->acquire_ctx = &ctx;
retry:
	changed = false;
	ret = drm_modeset_lock(&dev->mode_config.connection_mutex, &ctx);
	if (ret)
		goto out;
	drm_connector_list_iter_begin(dev, &iter);
	drm_for_each_connector_iter(conn, &iter) {
		struct drm_connector_state *cs = conn->state;

		if (!cs->crtc || (!cs->hdr_output_metadata &&
				  cs->colorspace == DRM_MODE_COLORIMETRY_DEFAULT))
			continue;
		cs = drm_atomic_get_connector_state(state, conn);
		if (IS_ERR(cs)) {
			ret = PTR_ERR(cs);
			break;
		}
		drm_property_blob_put(cs->hdr_output_metadata);
		cs->hdr_output_metadata = NULL;
		cs->colorspace = DRM_MODE_COLORIMETRY_DEFAULT;
		changed = true;
	}
	drm_connector_list_iter_end(&iter);
	if (!ret && changed)
		ret = drm_atomic_commit(state);
out:
	if (ret == -EDEADLK) {
		drm_atomic_commit_clear(state);
		drm_modeset_backoff(&ctx);
		goto retry;
	}
	if (ret)
		drm_warn(dev, "HDR output state kept after the DRM master left (%d)\n",
			 ret);
	drm_atomic_commit_put(state);
out_ctx:
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);
}

/*
 * A compositor or boot splash is closing the device.  drm_release() has
 * already taken its file off the file list, so the Type-C fabric can tell
 * whether any other one still has the device open.
 */
static void apple_drm_postclose(struct drm_device *drm, struct drm_file *file)
{
	bool unowned;

	if (!file->was_master)
		return;
	dcp_typec_reorder();
	mutex_lock(&drm->master_mutex);
	unowned = !drm->master;
	mutex_unlock(&drm->master_mutex);
	if (unowned)
		apple_drm_reset_hdr(drm);
}

static const struct drm_driver apple_drm_driver = {
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(apple_drm_gem_dumb_create),
#ifdef CONFIG_DRM_FBDEV_EMULATION
	.fbdev_probe		= apple_drm_fbdev_probe,
#else
	DRM_FBDEV_DMA_DRIVER_OPS,
#endif
	.postclose		= apple_drm_postclose,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.major			= 1,
	.minor			= 0,
	.driver_features	= DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC | DRIVER_SYNCOBJ | DRIVER_SYNCOBJ_TIMELINE,
	.fops			= &apple_fops,
};

enum drm_connector_status
apple_connector_detect(struct drm_connector *connector, bool force)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);

	if (connector->force == DRM_FORCE_ON || connector->force == DRM_FORCE_ON_DIGITAL)
		return connector_status_connected;
	if (connector->force == DRM_FORCE_OFF)
		return connector_status_disconnected;
	return dcp_connector_connected(apple_connector) ?
		connector_status_connected : connector_status_disconnected;
}

static void apple_connector_oob_hotplug(struct drm_connector *connector,
					enum drm_connector_status status)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);

	if (status == connector_status_connected)
		dcp_dptx_connect_oob(apple_connector->dcp, 0);
	else if (status == connector_status_disconnected)
		dcp_dptx_disconnect_oob(apple_connector->dcp, 0);
	else if (status == connector_status_unknown)
		dcp_retrain_oob(apple_connector);
	else
		dev_err(&apple_connector->dcp->dev, "unexpected connector status"
			":0x%x in oob_hotplug event\n", (u32)status);
}

/* The fabric hold is optional at link time, see dcp.h. */
static int apple_drm_fabric_hold(void)
{
	typeof(&apple_pmgr_fabric_hold) hold = symbol_get(apple_pmgr_fabric_hold);
	int ret;

	if (!hold)
		return 0;
	ret = hold();
	symbol_put(apple_pmgr_fabric_hold);
	return ret;
}

static void apple_drm_fabric_release(void)
{
	typeof(&apple_pmgr_fabric_release) release =
		symbol_get(apple_pmgr_fabric_release);

	if (!release)
		return;
	release();
	symbol_put(apple_pmgr_fabric_release);
}

static void apple_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_atomic_commit *commit)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct apple_dcp *dcp = platform_get_drvdata(apple_crtc->dcp);
	struct drm_crtc_state *crtc_state;
	bool locked;
	int ret;

	crtc_state = drm_atomic_get_new_crtc_state(commit, crtc);
	locked = false;
	ret = to_apple_atomic_state(commit)->failed_routes & drm_crtc_mask(crtc) ?
		-EIO : dcp_typec_follow_crtc(crtc, commit, &locked);
	/* A failed move must never power or program another sink's pipeline. */
	if (ret || !dcp_crtc_route_ready(crtc, commit, false))
		goto failed;

	if (crtc_state->active && dcp_crtc_needs_route_start(dcp)) {
		/* Establish the link/catalog before powering the assigned IOMFB. */
		ret = dcp_typec_prepare_route(dcp);
		if (ret)
			goto failed;
	}
	if (crtc_state->active && !dcp_crtc_route_ready(crtc, commit, true))
		goto failed;
	if (crtc_state->active && !dcp->main_display && !apple_crtc->fabric_held) {
		/* Raise the fabric before external scanout starts. */
		if (apple_drm_fabric_hold())
			dev_warn(dcp->dev, "fabric raise not confirmed before scanout\n");
		apple_crtc->fabric_held = true;
	}
	if (crtc_state->active &&
	    (crtc_state->active_changed || dcp_crtc_needs_route_start(dcp))) {
		/* The HPD of the link dcp_poweron() reconnects precedes our mode set. */
		WRITE_ONCE(dcp->typec_enable_pending, true);
		dcp_poweron(apple_crtc->dcp);
		crtc_state->color_mgmt_changed = true;
	} else if (crtc_state->active && drm_atomic_crtc_needs_modeset(crtc_state)) {
		/* A cable generation can change while the CRTC stays active. */
		dcp_external_recover(to_apple_crtc(crtc)->dcp);
	}
	if (crtc_state->active) {
		ret = dcp_crtc_atomic_modeset(crtc, commit);
		if (ret) {
			if (dcp_is_ext26(dcp))
				goto failed;
			dcp_drm_crtc_error(apple_crtc, crtc_state, ret);
		}
	}
	/* The mode set of the enable has run: a sink still without one has a bad link. */
	WRITE_ONCE(dcp->typec_enable_pending, false);
	dcp_typec_follow_done(locked);
	return;

failed:
	if (!ret)
		ret = -EAGAIN;
	dcp_mode_invalidate(&dcp->mode_state);
	dev_err(dcp->dev, "CRTC route unavailable (%d); modeset deferred\n", ret);
	/* Flush completes any pending flip without issuing a firmware swap. */
	schedule_work(&dcp->vblank_wq);
	WRITE_ONCE(dcp->typec_enable_pending, false);
	if (dcp->connector)
		dcp_route_failure_notify(dcp->connector);
	dcp_typec_follow_done(locked);
}

static void apple_crtc_atomic_disable(struct drm_crtc *crtc,
				      struct drm_atomic_commit *commit)
{
	struct drm_crtc_state *crtc_state;
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);

	crtc_state = drm_atomic_get_new_crtc_state(commit, crtc);

	if (crtc_state->active_changed && !crtc_state->active) {
		dcp_poweroff(apple_crtc->dcp);
		if (apple_crtc->fabric_held) {
			apple_crtc->fabric_held = false;
			apple_drm_fabric_release();
		}
	}

	if (iomfb_is_m4(platform_get_drvdata(apple_crtc->dcp)) &&
	    iomfb_m4_error(platform_get_drvdata(apple_crtc->dcp))) {
		dcp_drm_crtc_error(apple_crtc, crtc_state, -EIO);
		return;
	}

	if (crtc_state->event && !crtc_state->active) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc_state->event);
		spin_unlock_irq(&crtc->dev->event_lock);

		crtc_state->event = NULL;
	}
}

static void apple_crtc_atomic_begin(struct drm_crtc *crtc,
				    struct drm_atomic_commit *commit)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct drm_crtc_state *new = drm_atomic_get_new_crtc_state(commit, crtc);
	unsigned long flags;

	if (new->event) {
		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		apple_crtc->event = new->event;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
		new->event = NULL;
	}
}

static void apple_crtc_cleanup(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
	kfree(to_apple_crtc(crtc));
}

static int apple_crtc_parse_crc_source(const char *source, bool *enabled)
{
	int ret = 0;

	if (!source) {
		*enabled = false;
	} else if (strcmp(source, "auto") == 0) {
		*enabled = true;
	} else {
		*enabled = false;
		ret = -EINVAL;
	}

	return ret;
}

static int apple_crtc_set_crc_source(struct drm_crtc *crtc, const char *source)
{
	bool enabled = false;

	int ret = apple_crtc_parse_crc_source(source, &enabled);

	if (!ret)
		ret = dcp_set_crc(crtc, enabled);

	return ret;
}

static int apple_crtc_verify_crc_source(struct drm_crtc *crtc,
					const char *source,
					size_t *values_cnt)
{
	bool enabled;
	struct apple_dcp *dcp = platform_get_drvdata(to_apple_crtc(crtc)->dcp);

	if (iomfb_is_m4(dcp)) {
		*values_cnt = 0;
		return source ? -EOPNOTSUPP : 0;
	}

	if (apple_crtc_parse_crc_source(source, &enabled) < 0) {
		pr_warn("dcp: Invalid CRC source name %s\n", source);
		return -EINVAL;
	}

	*values_cnt = 1;

	return 0;
}

static const char * const apple_crtc_crc_sources[] = {"auto"};

static const char *const * apple_crtc_get_crc_sources(struct drm_crtc *crtc,
						      size_t *count)
{
	struct apple_dcp *dcp = platform_get_drvdata(to_apple_crtc(crtc)->dcp);

	if (iomfb_is_m4(dcp)) {
		*count = 0;
		return NULL;
	}
	*count = ARRAY_SIZE(apple_crtc_crc_sources);
	return apple_crtc_crc_sources;
}

static const struct drm_crtc_funcs apple_crtc_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.destroy		= apple_crtc_cleanup,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.set_config             = drm_atomic_helper_set_config,
	.set_crc_source		= apple_crtc_set_crc_source,
	.verify_crc_source	= apple_crtc_verify_crc_source,
	.get_crc_sources	= apple_crtc_get_crc_sources,

};

static struct drm_atomic_commit *apple_atomic_state_alloc(struct drm_device *dev)
{
	struct apple_atomic_state *state = kzalloc_obj(*state);

	if (!state)
		return NULL;
	if (drm_atomic_commit_init(dev, &state->base)) {
		kfree(state);
		return NULL;
	}
	return &state->base;
}

static void apple_atomic_state_clear(struct drm_atomic_commit *state)
{
	drm_atomic_commit_default_clear(state);
	to_apple_atomic_state(state)->failed_routes = 0;
}

static void apple_atomic_state_free(struct drm_atomic_commit *state)
{
	drm_atomic_commit_default_release(state);
	kfree(to_apple_atomic_state(state));
}

/*
 * A plane's in-fence (its buffer's implicit write fence, or the compositor's
 * IN_FENCE_FD) is waited for at most this long per commit. The helpers wait
 * for it without a timeout: a GPU fence that never signals, after a GPU
 * recovery for example, leaves the commit worker asleep for good. No later
 * frame reaches the display, the commit's flip event never comes, and every
 * later nonblocking commit fails with -EBUSY. One second is many frames even
 * at the panel's slowest refresh; past it the buffer is scanned out as it is,
 * so a GPU bug costs a stale or torn frame instead of the panel.
 */
#define APPLE_PLANE_FENCE_TIMEOUT_MS	1000

static void apple_warn_fence_timeout(struct drm_device *dev,
				     struct drm_plane *plane,
				     struct dma_fence *fence)
{
	static DEFINE_RATELIMIT_STATE(rs, 60 * HZ, 1);
	const char __rcu *driver;
	const char __rcu *timeline;

	if (!__ratelimit(&rs))
		return;
	rcu_read_lock();
	driver = dma_fence_driver_name(fence);
	timeline = dma_fence_timeline_name(fence);
	drm_warn(dev,
		 "[PLANE:%d:%s] in-fence %s:%s seqno %llu not signalled after %u ms, scanning out without it\n",
		 plane->base.id, plane->name, rcu_dereference(driver),
		 rcu_dereference(timeline), fence->seqno,
		 APPLE_PLANE_FENCE_TIMEOUT_MS);
	rcu_read_unlock();
}

/* As the helpers do: the GPU may finish the frame sooner knowing the vblank. */
static void apple_set_fence_deadline(struct drm_atomic_commit *state)
{
	struct drm_crtc_state *new_crtc_state;
	struct drm_plane_state *new_plane_state;
	struct drm_plane *plane;
	struct drm_crtc *crtc;
	ktime_t vbltime = 0;
	int i;

	for_each_new_crtc_in_state(state, crtc, new_crtc_state, i) {
		ktime_t v;

		if (drm_atomic_crtc_needs_modeset(new_crtc_state) ||
		    !new_crtc_state->active ||
		    drm_crtc_next_vblank_start(crtc, &v))
			continue;
		if (!vbltime || ktime_before(v, vbltime))
			vbltime = v;
	}
	if (!vbltime)
		return;
	for_each_new_plane_in_state(state, plane, new_plane_state, i)
		if (new_plane_state->fence)
			dma_fence_set_deadline(new_plane_state->fence, vbltime);
}

/*
 * drm_atomic_helper_wait_for_fences() with one bound for all of the commit's
 * planes (APPLE_PLANE_FENCE_TIMEOUT_MS). A fence that does not signal in time
 * is dropped. Before the swap (blocking commits) the wait is interruptible,
 * as the helper's is.
 */
static int apple_wait_for_fences(struct drm_device *dev,
				 struct drm_atomic_commit *state,
				 bool pre_swap)
{
	unsigned long deadline =
		jiffies + msecs_to_jiffies(APPLE_PLANE_FENCE_TIMEOUT_MS);
	struct drm_plane_state *new_plane_state;
	struct drm_plane *plane;
	int i;

	apple_set_fence_deadline(state);

	for_each_new_plane_in_state(state, plane, new_plane_state, i) {
		struct dma_fence *fence = new_plane_state->fence;
		long left, ret;

		if (!fence)
			continue;

		WARN_ON(!new_plane_state->fb);

		left = max_t(long, (long)(deadline - jiffies), 0);
		ret = dma_fence_wait_timeout(fence, pre_swap, left);
		if (ret < 0)
			return ret;
		if (!ret)
			apple_warn_fence_timeout(dev, plane, fence);

		dma_fence_put(fence);
		new_plane_state->fence = NULL;
	}

	return 0;
}

/*
 * A flip DCP did not complete within the helper's 10 s
 * (drm_atomic_helper_wait_for_flip_done() gave up) leaves its event pending.
 * A compositor waits for that event before it commits again, so send it now,
 * with -ETIMEDOUT on its out-fence, and the next commit can go ahead. A swap
 * completion that still comes later finds no event.
 */
static void apple_fail_lost_flips(struct drm_atomic_commit *state)
{
	for (int i = 0; i < state->dev->mode_config.num_crtc; i++) {
		struct drm_crtc_commit *commit = state->crtcs[i].commit;
		struct drm_crtc *crtc = state->crtcs[i].ptr;

		if (!crtc || !commit || t8132_ext_crtc(crtc) ||
		    completion_done(&commit->flip_done))
			continue;
		dcp_drm_crtc_flip_lost(to_apple_crtc(crtc), &commit->flip_done,
				       -ETIMEDOUT);
	}
}

static void apple_atomic_commit_tail(struct drm_atomic_commit *state);

/* The helper's commit_tail() with the bounded fence wait. */
static void apple_commit_tail(struct drm_atomic_commit *state)
{
	apple_wait_for_fences(state->dev, state, false);
	drm_atomic_helper_wait_for_dependencies(state);
	apple_atomic_commit_tail(state);
	drm_atomic_helper_commit_cleanup_done(state);
	drm_atomic_commit_put(state);
}

static void apple_commit_work(struct work_struct *work)
{
	apple_commit_tail(container_of(work, struct drm_atomic_commit,
				       commit_work));
}

/*
 * drm_atomic_helper_commit() with apple_wait_for_fences() in place of the
 * helper's unbounded wait. The driver keeps no self-refresh state, so the
 * helper's self-refresh accounting is left out.
 */
static int apple_atomic_commit(struct drm_device *dev,
			       struct drm_atomic_commit *state, bool nonblock)
{
	int ret;

	if (state->async_update) {
		ret = drm_atomic_helper_prepare_planes(dev, state);
		if (ret)
			return ret;

		drm_atomic_helper_async_commit(dev, state);
		drm_atomic_helper_unprepare_planes(dev, state);

		return 0;
	}

	ret = drm_atomic_helper_setup_commit(state, nonblock);
	if (ret)
		return ret;

	INIT_WORK(&state->commit_work, apple_commit_work);

	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret)
		return ret;

	if (!nonblock) {
		ret = apple_wait_for_fences(dev, state, true);
		if (ret)
			goto err;
	}

	ret = drm_atomic_helper_swap_state(state, true);
	if (ret)
		goto err;

	drm_atomic_commit_get(state);
	if (nonblock)
		queue_work(system_dfl_wq, &state->commit_work);
	else
		apple_commit_tail(state);

	return 0;

err:
	drm_atomic_helper_unprepare_planes(dev, state);
	return ret;
}

static const struct drm_mode_config_funcs apple_mode_config_funcs = {
	.atomic_state_alloc	= apple_atomic_state_alloc,
	.atomic_state_clear	= apple_atomic_state_clear,
	.atomic_state_free	= apple_atomic_state_free,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= apple_atomic_commit,
	.fb_create		= drm_gem_fb_create,
};

static void apple_atomic_commit_tail(struct drm_atomic_commit *state)
{
	struct drm_plane *plane;
	struct drm_plane_state *old, *new;
	int i;

	/* Retain displaced scanout even when the CRTC stays inactive. */
	for_each_oldnew_plane_in_state(state, plane, old, new, i) {
		struct apple_plane_state *apple_state;
		struct dcp_fb_reference *entry;

		if (t8132_ext_plane(plane))
			continue;
		apple_state = to_apple_plane_state(new);
		entry = apple_state->retirement;
		if (!entry)
			continue;

		apple_state->retirement = NULL;
		dcp_retain_framebuffer(to_apple_crtc(old->crtc)->dcp, entry);
	}

	drm_atomic_helper_commit_modeset_disables(state->dev, state);
	drm_atomic_helper_commit_modeset_enables(state->dev, state);
	drm_atomic_helper_commit_planes(state->dev, state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);
	drm_atomic_helper_fake_vblank(state);
	drm_atomic_helper_commit_hw_done(state);
	drm_atomic_helper_wait_for_flip_done(state->dev, state);
	apple_fail_lost_flips(state);
	drm_atomic_helper_cleanup_planes(state->dev, state);
}

/* apple_atomic_commit() calls the tail itself; this serves other helper users. */
static const struct drm_mode_config_helper_funcs apple_mode_config_helpers = {
	.atomic_commit_tail	= apple_atomic_commit_tail,
};

static void appledrm_connector_cleanup(struct drm_connector *connector)
{
	struct apple_connector *apple_connector = to_apple_connector(connector);

	/* Queued by the last disconnect at shutdown; they use the connector. */
	cancel_work_sync(&apple_connector->hotplug_wq);
	cancel_work_sync(&apple_connector->bl_sync_wq);
	cancel_work_sync(&apple_connector->bl_commit_wq);
	mutex_destroy(&apple_connector->bl_lock);
	drm_connector_cleanup(connector);
	kvfree(apple_connector->color_elements.data);
	kvfree(apple_connector->timing_elements.data);
	kvfree(apple_connector->display_attributes.data);
	kvfree(apple_connector->transport.data);
	kfree(apple_connector);
}

static int apple_connector_late_register(struct drm_connector *connector)
{
	return apple_connector_backlight_late_register(to_apple_connector(connector));
}

static void apple_connector_early_unregister(struct drm_connector *connector)
{
	apple_connector_backlight_early_unregister(to_apple_connector(connector));
}

static const struct drm_connector_funcs apple_connector_funcs = {
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= appledrm_connector_cleanup,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
	.detect			= apple_connector_detect,
	.debugfs_init		= apple_connector_debugfs_init,
	.oob_hotplug_event	= apple_connector_oob_hotplug,
	.late_register		= apple_connector_late_register,
	.early_unregister	= apple_connector_early_unregister,
};

/*
 * A Type-C connector is attached to every pipeline that can drive its port, so
 * the atomic helper cannot pick one on its own.  Follow the fabric's choice.
 */
static struct drm_encoder *
apple_connector_atomic_best_encoder(struct drm_connector *conn,
				    struct drm_atomic_commit *state)
{
	struct apple_connector *apple_connector = to_apple_connector(conn);
	struct drm_connector_state *conn_state;
	struct drm_encoder *encoder;

	/*
	 * Type-C tunnels are routed before the modeset and cannot move here.
	 * Reject stale topology and explicit assignments to another pipeline,
	 * except for a direct route that follows its CRTC (12.3/13.5, see
	 * dcp_typec_follow_crtc()): dcp_crtc_atomic_check() decides there.
	 */
	if (apple_connector->port_encoder) {
		struct platform_device *pdev = READ_ONCE(apple_connector->dcp);

		conn_state = drm_atomic_get_new_connector_state(state, conn);
		if (!conn_state || !conn_state->crtc || !pdev)
			return NULL;
		if (pdev != to_apple_crtc(conn_state->crtc)->dcp &&
		    !dcp_typec_follows_crtc(platform_get_drvdata(pdev)))
			return NULL;
		return apple_connector->port_encoder;
	}

	drm_connector_for_each_possible_encoder(conn, encoder)
		return encoder;

	return NULL;
}

/* The EOTF a connector state asks for, an HDMI_EOTF_* value. */
static u8 apple_connector_eotf(const struct drm_connector_state *state)
{
	const struct hdr_output_metadata *hdr;

	if (!state->hdr_output_metadata ||
	    state->hdr_output_metadata->length != sizeof(*hdr))
		return HDMI_EOTF_TRADITIONAL_GAMMA_SDR;
	hdr = state->hdr_output_metadata->data;
	return hdr->hdmi_metadata_type1.eotf;
}

static int apple_connector_atomic_check(struct drm_connector *conn,
					struct drm_atomic_commit *state)
{
	struct drm_connector_state *old = drm_atomic_get_old_connector_state(state, conn);
	struct drm_connector_state *new = drm_atomic_get_new_connector_state(state, conn);
	struct drm_crtc_state *crtc_state;

	if (!new->crtc || (old->colorspace == new->colorspace &&
			   drm_connector_atomic_hdr_metadata_equal(old, new)))
		return 0;
	crtc_state = drm_atomic_get_crtc_state(state, new->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);
	/*
	 * The output colour element follows the EOTF and colorspace, so a
	 * change of either is a mode set.  Where the firmware takes HDR10
	 * static metadata on a live link, a new block alone is sent with this
	 * commit's flush (dcp_hdr_metadata_sync()) and the colour mode stays.
	 */
	if (old->colorspace == new->colorspace && old->crtc == new->crtc &&
	    apple_connector_eotf(old) == apple_connector_eotf(new) &&
	    to_apple_connector(conn)->dcp &&
	    dcp_hdr_metadata_live(to_apple_connector(conn)->dcp))
		return 0;
	/*
	 * A panel that shows PQ as EDR keeps its SDR link: SDR and PQ switch
	 * with the next swap, so a compositor turning HDR on at startup does
	 * not blank it.  Other EOTFs still go through the mode set's check.
	 */
	if (old->crtc == new->crtc && to_apple_connector(conn)->dcp &&
	    dcp_edr_live(to_apple_connector(conn)->dcp) &&
	    (apple_connector_eotf(new) == HDMI_EOTF_TRADITIONAL_GAMMA_SDR ||
	     apple_connector_eotf(new) == HDMI_EOTF_SMPTE_ST2084))
		return 0;
	crtc_state->mode_changed = true;
	return 0;
}

static const struct drm_connector_helper_funcs apple_connector_helper_funcs = {
	.get_modes		= dcp_get_modes,
	.mode_valid		= dcp_mode_valid,
	.atomic_best_encoder	= apple_connector_atomic_best_encoder,
	.atomic_check		= apple_connector_atomic_check,
};

/*
 * A desktop 26.x panel shows PQ as EDR (see dcp_edr_prepare()): PQ 100 cd/m²
 * lands on the white the backlight sets, and brighter levels use the
 * headroom above it, up to the panel's peak. macOS uses the panel the same
 * way. PQ_REFERENCE_LUMINANCE tells a compositor where that white is in PQ,
 * so it can put its reference white there and keep using the backlight as
 * the brightness control in PQ.
 */
#define DCP_EDR_PQ_REFERENCE_NITS	100

static int apple_connector_attach_pq_reference(struct drm_connector *connector)
{
	struct drm_property *prop;

	prop = drm_property_create_range(connector->dev, DRM_MODE_PROP_IMMUTABLE,
					 "PQ_REFERENCE_LUMINANCE", 0, 10000);
	if (!prop)
		return -ENOMEM;
	drm_object_attach_property(&connector->base, prop,
				   DCP_EDR_PQ_REFERENCE_NITS);
	return 0;
}

/*
 * External displays can take a PQ BT.2020 signal where the firmware
 * offers one, and a desktop 26.x panel PQ content it shows as EDR;
 * dcp_crtc_atomic_check() refuses it elsewhere.
 */
static int apple_connector_attach_hdr(struct drm_connector *connector,
				      bool panel_edr)
{
	const u32 colorspaces = BIT(DRM_MODE_COLORIMETRY_BT2020_RGB);
	int ret;

	switch (connector->connector_type) {
	case DRM_MODE_CONNECTOR_eDP:
		if (!panel_edr)
			return 0;
		fallthrough;
	/* A desktop 26.x Type-C route is a pipeline output carrying DisplayPort. */
	case DRM_MODE_CONNECTOR_USB:
	case DRM_MODE_CONNECTOR_DisplayPort:
		ret = drm_mode_create_dp_colorspace_property(connector, colorspaces);
		break;
	case DRM_MODE_CONNECTOR_HDMIA:
		ret = drm_mode_create_hdmi_colorspace_property(connector, colorspaces);
		break;
	default:
		return 0;
	}
	if (ret)
		return ret;
	ret = drm_connector_attach_colorspace_property(connector);
	if (ret)
		return ret;
	drm_connector_attach_hdr_output_metadata_property(connector);
	if (connector->connector_type == DRM_MODE_CONNECTOR_eDP)
		return apple_connector_attach_pq_reference(connector);
	return 0;
}

static const struct drm_crtc_helper_funcs apple_crtc_helper_funcs = {
	.atomic_begin		= apple_crtc_atomic_begin,
	.atomic_check		= dcp_crtc_atomic_check,
	.atomic_flush		= dcp_flush,
	.atomic_enable		= apple_crtc_atomic_enable,
	.atomic_disable		= apple_crtc_atomic_disable,
	.mode_fixup		= dcp_crtc_mode_fixup,
};

static int apple_connector_create(struct drm_device *drm,
				  struct platform_device *dcp,
				  struct apple_crtc *crtc,
				  struct apple_encoder *encoder,
				  int connector_type, bool attach_fwnode)
{
	struct apple_connector *connector;
	int ret;

	connector = kzalloc_obj(*connector);
	if (!connector)
		return -ENOMEM;

	mutex_init(&connector->chunk_lock);
	apple_connector_edid_init(connector);
	apple_connector_backlight_init(connector);
	/* before anything can fail: cleanup cancels it */
	INIT_WORK(&connector->hotplug_wq, dcp_hotplug);
	drm_connector_helper_add(&connector->base,
				 &apple_connector_helper_funcs);
	if (attach_fwnode)
		connector->base.fwnode = fwnode_handle_get(dcp->dev.fwnode);

	ret = drm_connector_init(drm, &connector->base, &apple_connector_funcs,
				 connector_type);
	if (ret)
		goto err_free;
	ret = drm_connector_attach_vrr_capable_property(&connector->base);
	if (ret)
		goto err_cleanup;
	ret = apple_connector_attach_hdr(&connector->base,
					 dcp_fw_compat_panel_edr(dcp));
	if (ret)
		goto err_cleanup;

	connector->base.polled = DRM_CONNECTOR_POLL_HPD;
	connector->connected = false;
	apple_connector_set_pipeline(connector, dcp);

	ret = drm_connector_attach_encoder(&connector->base, &encoder->base);
	if (ret)
		goto err_cleanup;
	dcp_link(dcp, crtc, connector);

	return 0;

err_cleanup:
	drm_connector_cleanup(&connector->base);
	kfree(connector);
	return ret;
err_free:
	fwnode_handle_put(connector->base.fwnode);
	kfree(connector);
	return ret;
}

static int apple_probe_per_dcp(struct device *dev,
			       struct drm_device *drm,
			       struct platform_device *dcp,
			       int num, bool dcp_ext,
			       struct drm_encoder **encoder,
			       u32 *crtc_mask)
{
	struct apple_crtc *crtc;
	struct apple_encoder *enc;
	struct drm_plane *planes[DCP_MAX_PLANES];
	unsigned long *iomfb_surfaces = dcp_get_iomfb_surfaces(dcp);
	unsigned long top_surf = find_last_bit(iomfb_surfaces, DCP_MAX_PLANES);
	int ret;
	int connector_type;
	u32 surf;
	int zpos = 0;
	bool supports_l10r = !dcp_fw_compat_is_12_x(dcp);
	struct apple_dcp *dcp_data = platform_get_drvdata(dcp);
	bool supports_xrgb2101010;
	bool supports_premultiplied;
	enum drm_plane_type plane_type;
	bool interchange;
	bool supports_interchange;

	supports_xrgb2101010 = dcp_data->fw_compat != DCP_FIRMWARE_H17P ||
		dcp_data->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G;
	/*
	 * Interchange scanout has not been qualified with the H17P firmware
	 * (T8140 and T8122): advertise linear framebuffers only there.
	 */
	supports_interchange = dcp_data->fw_compat != DCP_FIRMWARE_H17P;
	/* The H17P swap surface has no is_premultiplied flag. */
	supports_premultiplied = dcp_data->fw_compat != DCP_FIRMWARE_H17P;
	if (bitmap_empty(iomfb_surfaces, DCP_MAX_PLANES))
		return dev_err_probe(dev, -EINVAL, "No usable display surfaces\n");

	if (dcp_fw_compat_is_14_7(dcp)) {
		planes[0] = apple_plane_init_v14_7(drm, 1U << num, 0,
						    DRM_PLANE_TYPE_PRIMARY);
		if (IS_ERR(planes[0]))
			return PTR_ERR(planes[0]);
		ret = drm_plane_create_zpos_immutable_property(planes[0], 0);
		if (ret)
			return ret;
		zpos = 1;
	}

	/*
	 * BUG:
	 * Some DCPs support more than two surfaces. For some reason, these
	 * will crash if the topmost layer is Interchange *and* the surface
	 * is cleared *while content is being scanned out on the middle one*.
	 * They do not do this with linear framebuffers, nor do they do this
	 * when the middle surface is not in use. We therefore hide Interchange
	 * from the surface.
	 */
	for_each_set_bit(surf, iomfb_surfaces, DCP_MAX_PLANES) {
		if (dcp_fw_compat_is_14_7(dcp))
			break;
		plane_type = (zpos == 0) ? DRM_PLANE_TYPE_PRIMARY : DRM_PLANE_TYPE_OVERLAY;
		if (iomfb_is_m4(dcp_data)) {
			if (zpos || surf)
				return dev_err_probe(dev, -EINVAL,
						     "T8132 panel needs surface 0 only\n");
			planes[zpos] = apple_plane_init_v26_6(drm, 1U << num, surf);
		} else {
			interchange = supports_interchange &&
				      (zpos <= 1 || surf != top_surf);

			/*
			 * The T8142 runs macOS 27.0's DCP firmware as firmware
			 * 27, outside the H17G profile, and blends XRGB's X
			 * byte as alpha all the same (GPU-rendered alacritty
			 * showed wrong colours on the panel).
			 */
			bool xrgb_is_alpha =
				dcp_data->hw.iomfb_method_profile ==
					DCP_IOMFB_METHODS_H17G ||
				(dcp_data->fw_compat == DCP_FIRMWARE_V_27_0 &&
				 of_device_is_compatible(dcp_data->dev->of_node,
							 "apple,t8142-dcp"));

			planes[zpos] = apple_plane_init(drm, 1U << num, surf,
							supports_l10r,
							supports_xrgb2101010,
							xrgb_is_alpha,
							plane_type, interchange,
							supports_premultiplied, dcp_is_ext26(dcp_data));
		}
		if (IS_ERR(planes[zpos]))
			return PTR_ERR(planes[zpos]);

		ret = drm_plane_create_zpos_immutable_property(planes[zpos], zpos);
		if (ret)
			return ret;

		zpos++;
	}

	/*
	 * Even though we have an overlay plane, we cannot expose it to legacy
	 * userspace for cursors as we cannot make the same guarantees as ye olde
	 * hardware cursor planes such userspace would expect us to. Modern userspace
	 * knows what to do with overlays.
	 */
	crtc = kzalloc(sizeof(*crtc), GFP_KERNEL);
	if (!crtc)
		return -ENOMEM;
	ret = drm_crtc_init_with_planes(drm, &crtc->base, planes[0], NULL,
					&apple_crtc_funcs, NULL);
	if (ret)
		return ret;

	drm_crtc_helper_add(&crtc->base, &apple_crtc_helper_funcs);
	/* T8132 has no qualified CTM transaction. */
	if (!iomfb_is_m4(dcp_data)) {
		unsigned int gamma_size = dcp_gamma_lut_size(dcp);

		drm_crtc_enable_color_mgmt(&crtc->base, 0, true, gamma_size);
		if (gamma_size)
			drm_mode_crtc_set_gamma_size(&crtc->base, gamma_size);
	}

	crtc->dcp = dcp;
	*crtc_mask = drm_crtc_mask(&crtc->base);
	connector_type = dcp_get_connector_type(dcp);

	/*
	 * Type-C outputs are physical ports, not pipelines: several DCPs may be
	 * able to drive one port, and which one does can change at runtime.
	 * Those connectors are created per port once every pipeline is known,
	 * so a pipeline whose only output is Type-C gets no connector here --
	 * and no encoder either, because the port's encoder spans every
	 * pipeline that can drive it.
	 */
	/* Desktop 26.x drives its Type-C route itself, as a pipeline output. */
	if (connector_type == DRM_MODE_CONNECTOR_USB &&
	    !dcp_fw_compat_is_desktop(dcp)) {
		dcp_link(dcp, crtc, NULL);
		*encoder = NULL;
		return 0;
	}

	enc = drmm_simple_encoder_alloc(drm, struct apple_encoder, base,
					DRM_MODE_ENCODER_TMDS);
	if (IS_ERR(enc))
		return PTR_ERR(enc);
	enc->base.possible_crtcs = *crtc_mask;
	*encoder = &enc->base;

	return apple_connector_create(drm, dcp, crtc, enc, connector_type,
				      dcp_ext);
}

/*
 * Create a connector per physical Type-C port and attach it to every
 * pipeline that can drive it. The M2 Pro/Max laptops append a second
 * connector per port for a dock's second DP tunnel, preserving primary
 * connector names. T6050 instead exposes one connector per port/pipeline
 * pair: tunnel allocation chooses which connector is connected, without
 * changing any encoder's advertised CRTC mask after registration. Pairs
 * whose route carries DP tunnels only come last, so adding such a route
 * does not renumber the connectors that existed before it.
 */
static int apple_probe_typec_ports(struct drm_device *drm,
				   struct platform_device **dcp,
				   u32 *crtc_mask, int num_dcp)
{
	unsigned int n, nr_ports = dcp_typec_nr_ports();
	bool fixed = dcp_typec_fixed_crtcs();
	unsigned int slots = fixed ? num_dcp : (dcp_typec_dual_stream() ? 2 : 1);
	/* T6050 creates its tunnel-only port/pipeline pairs in a second pass. */
	unsigned int passes = fixed ? 2 : 1;
	int i, ret;

	/* Keep the existing physical-port connector numbers stable. On the
	 * M2 Pro/Max laptops, append one more connector per port for a second
	 * USB4 DP tunnel.
	 */
	for (n = 0; n < passes * nr_ports * slots; n++) {
		unsigned int pass = n / (nr_ports * slots);
		unsigned int idx = n % (nr_ports * slots);
		struct apple_connector *connector;
		struct apple_encoder *enc;
		struct platform_device *pipeline = fixed ? dcp[idx / nr_ports] : NULL;
		unsigned int port_idx = idx % nr_ports;
		bool secondary = idx >= nr_ports;
		u32 mask = 0;

		/* Only a failed T8122 dcpext could drive it: no connector. */
		if (dcp_typec_port_unusable(port_idx)) {
			if (!pass)
				drm_warn(drm, "Type-C port %u: its display pipeline is unusable\n",
					 port_idx);
			continue;
		}

		if (fixed) {
			if (!dcp_typec_port_has_candidate(port_idx, pipeline))
				continue;
			/* T6050: tunnel-only pairs in the second pass */
			if (dcp_typec_port_tunnel_only(port_idx, pipeline) != pass)
				continue;
			mask = crtc_mask[idx / nr_ports];
			if (!mask)
				continue;
		} else {
			for (i = 0; i < num_dcp; i++) {
				if (!dcp_typec_port_has_candidate(port_idx, dcp[i]))
					continue;
				/* Ordered dual-stream docks reserve the Type-C-only pipe. */
				mask = dcp_fabric_port_crtc_mask(secondary,
								 dcp_typec_ordered_dual_stream(),
							       dcp_is_typec_only(dcp[i]),
							       mask, crtc_mask[i]);
			}
			if (!mask) {
				drm_warn(drm, "Type-C port %u has no display pipeline\n",
					 port_idx);
				return -ENODEV;
			}
		}

		connector = kzalloc_obj(*connector);
		if (!connector)
			return -ENOMEM;

		mutex_init(&connector->chunk_lock);
		apple_connector_edid_init(connector);
		apple_connector_backlight_init(connector);
		/* before anything can fail: cleanup cancels it */
		INIT_WORK(&connector->hotplug_wq, dcp_hotplug);
		drm_connector_helper_add(&connector->base,
					 &apple_connector_helper_funcs);

		ret = drm_connector_init(drm, &connector->base,
					 &apple_connector_funcs,
					 DRM_MODE_CONNECTOR_DisplayPort);
		if (ret) {
			kfree(connector);
			return ret;
		}

		ret = drm_connector_attach_vrr_capable_property(&connector->base);
		if (ret)
			return ret;
		/*
		 * T8122's retained external scanout accepts only 8-bit RGB.
		 * Do not advertise the BT.2020/PQ output contract until this
		 * route has a qualified 10-bit primary-plane format. The sink
		 * EDID remains intact; its capabilities are not the pipeline's.
		 */
		if (!of_machine_is_compatible("apple,t8122")) {
			ret = apple_connector_attach_hdr(&connector->base, false);
			if (ret)
				return ret;
		}

		connector->base.polled = DRM_CONNECTOR_POLL_HPD;
		connector->connected = false;
		connector->dcp = NULL;

		/*
		 * One encoder per connector, covering its allowed pipelines.
		 * T6050 connectors each cover exactly one. Attaching several
		 * encoders to the same connector would describe
		 * the same hardware, but userspace takes the CRTCs a connector
		 * can use to be what all of its encoders have in common, and
		 * single-pipeline encoders have nothing in common -- the port
		 * ends up with no usable CRTC and the monitor is left dark.
		 */
		enc = drmm_simple_encoder_alloc(drm, struct apple_encoder, base,
						DRM_MODE_ENCODER_TMDS);
		if (IS_ERR(enc))
			return PTR_ERR(enc);
		enc->base.possible_crtcs = mask;

		ret = drm_connector_attach_encoder(&connector->base, &enc->base);
		if (ret)
			return ret;
		connector->port_encoder = &enc->base;
		connector->candidate_crtcs = mask;

		dcp_typec_port_set_connector(port_idx, secondary, connector, pipeline);
	}

	return 0;
}

static int apple_get_fb_resource(struct device *dev, const char *name,
				 struct resource *fb_r)
{
	int idx, ret = -ENODEV;
	struct device_node *node;

	idx = of_property_match_string(dev->of_node, "memory-region-names", name);

	node = of_parse_phandle(dev->of_node, "memory-region", idx);
	if (!node) {
		dev_err(dev, "reserved-memory node '%s' not found\n", name);
		return -ENODEV;
	}

	if (!of_device_is_available(node)) {
		dev_err(dev, "reserved-memory node '%s' is unavailable\n", name);
		goto err;
	}

	if (!of_device_is_compatible(node, "framebuffer")) {
		dev_err(dev, "reserved-memory node '%s' is incompatible\n",
			node->full_name);
		goto err;
	}

	ret = of_address_to_resource(node, 0, fb_r);

err:
	of_node_put(node);
	return ret;
}

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T8132)
/*
 * The J713 loader describes the boot framebuffer only as a simple-framebuffer
 * node; its display subsystem has no "framebuffer" memory region.  Take the
 * aperture from the single available simple-framebuffer.
 */
static bool apple_boot_fb_only(struct device *dev)
{
	return (of_device_is_compatible(dev->of_node, "apple,t8132-display-subsystem") ||
		of_device_is_compatible(dev->of_node, "apple,t8132-display-diagnostics")) &&
	       of_property_match_string(dev->of_node, "memory-region-names",
					"framebuffer") < 0;
}

static int apple_get_boot_fb_resource(struct device *dev, struct resource *fb_r)
{
	struct device_node *np;
	int found = 0, ret = -ENODEV;

	for_each_compatible_node(np, NULL, "simple-framebuffer") {
		if (!of_device_is_available(np))
			continue;
		if (++found > 1) {
			of_node_put(np);
			dev_err(dev, "more than one boot framebuffer\n");
			return -EINVAL;
		}
		ret = of_address_to_resource(np, 0, fb_r);
	}
	if (ret)
		dev_err(dev, "no boot framebuffer aperture: %d\n", ret);
	return ret;
}
#endif

static const struct of_device_id apple_dcp_id_tbl[] = {
	{ .compatible = "apple,dcp" },
	{ .compatible = "apple,dcpext" },
	/* T6030: apple_dcp_usable() keeps its dcpext outside the DRM device. */
	{ .compatible = "apple,t6030-dcp" },
	{ .compatible = "apple,t8122-dcp" },
	{ .compatible = "apple,t6031-dcp" },
	{ .compatible = "apple,t6034-dcp" },
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T8132)
	{ .compatible = "apple,t8132-dcp" },
#endif
	{},
};

/*
 * A supported firmware-described external provider participates in component
 * matching. A T6030 external display processor is a pipe of this DRM device
 * once the display gate gave it its display DART; on the manual diagnostic
 * path it runs a separate session and never joins this DRM device.
 */
static bool apple_dcp_usable(struct device_node *np)
{
	return of_device_is_available(np) &&
	       (!of_device_is_compatible(np, "apple,t6030-dcpext") ||
		dcp_t6030_ext_native(np)) &&
	       (!of_device_is_compatible(np, "apple,t8140-dcpext") ||
		dcp_h17p_external_supported(np)) &&
	       /* t6041_dcpext=0 keeps the T6041 external processors out. */
	       (!of_device_is_compatible(np, "apple,t6041-dcpext") ||
		dcp_t6041_ext_enabled());
}

static void apple_drm_quiesce_connectors(struct drm_device *drm)
{
	struct drm_connector_list_iter iter;
	struct drm_connector *connector;

	drm_connector_list_iter_begin(drm, &iter);
	drm_for_each_connector_iter(connector, &iter) {
		struct apple_connector *apple_connector = to_apple_connector(connector);

		/* A T8132 external output quiesces on its own. */
		if (t8132_ext_connector(connector))
			continue;
		/* DCP callbacks can still enqueue these until component unbind
		 * has drained RTKit. Stop their users before releasing endpoints,
		 * clocks and IOMMUs, and reject enqueues during that release.
		 */
		disable_work_sync(&apple_connector->hotplug_wq);
		disable_work_sync(&apple_connector->bl_sync_wq);
		disable_work_sync(&apple_connector->bl_commit_wq);
	}
	drm_connector_list_iter_end(&iter);
}

/*
 * Each display processor runs its own RTKit firmware, and starting one waits
 * for its firmware: a T6041 external processor takes 0.6 to 1.6 s to bring up
 * its endpoints. Started one after another, the internal panel waited for
 * every external processor before the DRM device registered. Start them all
 * at once and wait for each; shared routes and crossbars are claimed through
 * the display fabric's locking, as they are when processors hotplug.
 */
struct apple_dcp_start_work {
	struct work_struct work;
	struct platform_device *dcp;
	int ret;
};

static void apple_dcp_start_fn(struct work_struct *work)
{
	struct apple_dcp_start_work *w =
		container_of(work, struct apple_dcp_start_work, work);

	w->ret = dcp_start(w->dcp);
}

static int apple_drm_init_dcp(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);
	struct platform_device *dcp[MAX_COPROCESSORS];
	struct drm_encoder *encoder[MAX_COPROCESSORS];
	u32 crtc_mask[MAX_COPROCESSORS] = {};
	int connector_type[MAX_COPROCESSORS];
	bool started[MAX_COPROCESSORS] = {};
	struct apple_dcp_start_work start[MAX_COPROCESSORS];
	bool ready[MAX_COPROCESSORS] = {};
	struct device_node *np;
	u64 timeout;
	int i, ret, num_dcp = 0;

	for_each_matching_node(np, apple_dcp_id_tbl) {
		bool dcp_ext;
		if (!apple_dcp_usable(np))
			continue;
		dcp_ext = of_device_is_compatible(np, "apple,dcpext") ||
		          of_device_is_compatible(np, "apple,t8140-dcpext") ||
		          of_property_present(np, "phys");

		/* component_bind_all() established the matched provider lifetime. */
		if (of_device_is_compatible(np, "apple,t8140-dcpext") &&
		    !dcp_h17p_external_ready(np)) {
			of_node_put(np);
			return -ENODEV;
		}

		dcp[num_dcp] = of_find_device_by_node(np);
		if (!dcp[num_dcp])
			continue;
		/* A failed T8122 dcpext: bound, but no pipeline. */
		if (dcp_ext_unusable(dcp[num_dcp])) {
			dev_warn(dev, "%s: external display processor unusable, skipped\n",
				 dev_name(&dcp[num_dcp]->dev));
			put_device(&dcp[num_dcp]->dev);
			continue;
		}

		device_link_add(dev, &dcp[num_dcp]->dev, DL_FLAG_AUTOREMOVE_SUPPLIER);

		ret = apple_probe_per_dcp(dev, &apple->drm, dcp[num_dcp],
					  num_dcp, dcp_ext, &encoder[num_dcp],
					  &crtc_mask[num_dcp]);
		if (ret) {
			if (of_device_is_compatible(np, "apple,t8140-dcpext")) {
				put_device(&dcp[num_dcp]->dev);
				of_node_put(np);
				return ret;
			}
			continue;
		}

		num_dcp++;
	}

	if (num_dcp < 1)
		return -ENODEV;

	/*
	 * Build the Type-C port connectors before starting any pipeline: a
	 * pipeline can be handed a route as soon as the Type-C mux notifies,
	 * and it needs somewhere to report the display.
	 */
	ret = apple_probe_typec_ports(&apple->drm, dcp, crtc_mask, num_dcp);
	if (ret)
		return ret;

	/* The J713 external outputs; the panel goes on without them. */
	ret = t8132_ext_drm_init(&apple->drm);
	if (ret)
		dev_warn(dev, "T8132 external outputs not all registered: %d\n", ret);

	for (i = 0; i < num_dcp; i++) {
		connector_type[i] = dcp_get_connector_type(dcp[i]);
		start[i].dcp = dcp[i];
		INIT_WORK_ONSTACK(&start[i].work, apple_dcp_start_fn);
		queue_work(system_dfl_wq, &start[i].work);
	}
	for (i = 0; i < num_dcp; i++) {
		flush_work(&start[i].work);
		destroy_work_on_stack(&start[i].work);
		ret = start[i].ret;
		started[i] = !ret;
		if (ret)
			dev_warn(dev, "DCP[%d] failed to start: %d\n", i, ret);
	}

	/*
	 * Starting DPTX might take some time.  The T6041 panel runs its
	 * client-open, main-display and power chain inside the start: give it
	 * as long as that takes.
	 */
	timeout = get_jiffies_64() + msecs_to_jiffies(3000);
	for (i = 0; i < num_dcp; i++)
		if (of_device_is_compatible(dcp[i]->dev.of_node, "apple,t6041-dcp"))
			timeout = get_jiffies_64() + msecs_to_jiffies(10000);

	for (i = 0; i < num_dcp; ++i) {
		u64 jiffies = get_jiffies_64();
		u64 wait = time_after_eq64(jiffies, timeout) ?
				   0 :
				   timeout - jiffies;

		/* A processor that did not start cannot become ready. */
		if (!started[i])
			continue;
		ret = dcp_wait_ready(dcp[i], wait);
		ready[i] = !ret;
		/*
		 * Any other processor that does not come up is left out: the
		 * driver goes on without it.
		 */
		if (ret)
			dev_warn(dev, "DCP[%d] not ready: %d\n", i, ret);
		/*
		 * The J713 panel is only taken over from a started session:
		 * without it the boot framebuffer (simpledrm) stays.
		 */
		if (ret && iomfb_is_m4(platform_get_drvdata(dcp[i])))
			return ret;
	}

	/*
	 * The caller removes the boot framebuffer next. If none of the display
	 * processors that can be showing it came up, that would leave the
	 * screen black with nothing to replace it: fail instead, so that the
	 * boot framebuffer stays.
	 */
	if (!apple_boot_display_up(connector_type, ready, num_dcp)) {
		dev_err(dev, "no display processor that can show the boot framebuffer came up; keeping the boot framebuffer\n");
		return -ENODEV;
	}

	/* HACK: Wait for dcp* to settle before a modeset */
	msleep(100);

	return 0;
}

static int apple_drm_init(struct device *dev)
{
	struct apple_drm_private *apple;
	struct resource fb_r;
	struct drm_crtc *crtc;
	resource_size_t fb_size;
	bool retained_modeset = false;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T8132)
	if (apple_boot_fb_only(dev))
		ret = apple_get_boot_fb_resource(dev, &fb_r);
	else
#endif
		ret = apple_get_fb_resource(dev, "framebuffer", &fb_r);
	if (ret)
		return ret;

	apple = devm_drm_dev_alloc(dev, &apple_drm_driver,
				   struct apple_drm_private, drm);
	if (IS_ERR(apple))
		return PTR_ERR(apple);

	dev_set_drvdata(dev, apple);

	ret = drmm_mode_config_init(&apple->drm);
	if (ret)
		return ret;

	/*
	 * IOMFB::UPPipeDCP_H13P::verify_surfaces produces the error "plane
	 * requires a minimum of 32x32 for the source buffer" if smaller
	 */
	apple->drm.mode_config.min_width = 32;
	apple->drm.mode_config.min_height = 32;

	/*
	 * TODO: this is the max framebuffer size not the maximal supported
	 * output resolution. DCP reports the maximal framebuffer size take it
	 * from there.
	 * Hardcode it for now to the M1 Max DCP reported 'MaxSrcBufferWidth'
	 * and 'MaxSrcBufferHeight' of 16384.
	 */
	apple->drm.mode_config.max_width = 16384;
	apple->drm.mode_config.max_height = 16384;

	apple->drm.mode_config.funcs = &apple_mode_config_funcs;
	apple->drm.mode_config.helper_private = &apple_mode_config_helpers;

	ret = component_bind_all(dev, apple);
	if (ret)
		return ret;

	ret = apple_drm_init_dcp(dev);
	if (ret)
		goto err_unbind;

	drm_mode_config_reset(&apple->drm);
	if (IS_ENABLED(CONFIG_DRM_APPLE_DCP_T8132)) {
		drm_for_each_crtc(crtc, &apple->drm) {
			if (!t8132_ext_crtc(crtc) &&
			    iomfb_is_m4(platform_get_drvdata(to_apple_crtc(crtc)->dcp))) {
				retained_modeset = true;
				break;
			}
		}
	}

	/* Registration makes KMS visible before the first destructive T8132
	 * swap. Hold the modeset locks across activation so an early userspace
	 * commit cannot race the retained startup surface.
	 */
	/*
	 * T6041 panel: everything that can still be refused is tried while
	 * the boot framebuffer owns the screen; past the eviction there is no
	 * way back.  On refusal leave the DCP running (its shutdown sequence
	 * can crash it and blank the panel), pin the module and keep the boot
	 * framebuffer. The pin is taken before the trials, while a refusal can
	 * still unwind; it is given back if every trial passes.
	 */
	if (!try_module_get(THIS_MODULE)) {
		ret = -ENODEV;
		goto err_unbind;
	}
	drm_for_each_crtc(crtc, &apple->drm) {
		const char *reason = "none";

		if (t8132_ext_crtc(crtc))
			continue;
		ret = dcp_takeover_trial(to_apple_crtc(crtc)->dcp, &reason);
		if (!ret)
			continue;
		dev_err(dev, "display takeover refused (%s: %d); the boot framebuffer keeps the screen\n",
			reason, ret);
		/*
		 * The DCPs keep running and keep their pipelines: pin the
		 * module and the DRM device their CRTCs and connectors live
		 * in, and keep them out of shutdown and sleep. The module
		 * reference taken before the trials is kept.
		 */
		drm_dev_get(&apple->drm);
		drm_for_each_crtc(crtc, &apple->drm)
			if (!t8132_ext_crtc(crtc))
				dcp_takeover_aborted(to_apple_crtc(crtc)->dcp);
		apple->aborted = true;
		/* No sleep handling for outputs of a device never registered. */
		t8132_ext_drm_fini(&apple->drm);
		return 0;
	}
	module_put(THIS_MODULE);

	if (retained_modeset)
		drm_modeset_lock_all(&apple->drm);
	fb_size = fb_r.end - fb_r.start + 1;
	ret = aperture_remove_conflicting_devices(fb_r.start, fb_size,
						  apple_drm_driver.name);
	if (ret) {
		dev_err(dev, "Failed remove fb: %d\n", ret);
		goto err_unlock;
	}

	ret = drm_dev_register(&apple->drm, 0);
	if (ret)
		goto err_unlock;
	if (retained_modeset) {
		drm_for_each_crtc(crtc, &apple->drm) {
			if (t8132_ext_crtc(crtc))
				continue;
			ret = dcp_activate(to_apple_crtc(crtc)->dcp);
			if (ret) {
				drm_modeset_unlock_all(&apple->drm);
				t8132_ext_drm_fini(&apple->drm);
				drm_dev_unregister(&apple->drm);
				goto err_unbind;
			}
		}
	}
	if (retained_modeset)
		drm_modeset_unlock_all(&apple->drm);

	/* the fabric keeps order only once registered: catch up on probe */
	dcp_typec_reorder();

	/* A panel that publishes no level registers its backlight now. */
	drm_for_each_crtc(crtc, &apple->drm)
		if (!t8132_ext_crtc(crtc))
			dcp_backlight_takeover_registered(to_apple_crtc(crtc)->dcp);

	drm_client_setup_with_fourcc(&apple->drm, DRM_FORMAT_XRGB8888);

	/* A subsequent successful platform probe retries deferred GPU probes. */
	ret = apple_g15_display_publish(dev);
	if (ret) {
		t8132_ext_drm_fini(&apple->drm);
		drm_dev_unregister(&apple->drm);
		goto err_unbind;
	}

	apple->pm_nb.notifier_call = apple_drm_pm_notify;
	register_pm_notifier(&apple->pm_nb);

	return 0;

err_unlock:
	if (retained_modeset)
		drm_modeset_unlock_all(&apple->drm);
err_unbind:
	t8132_ext_drm_fini(&apple->drm);
	apple_drm_quiesce_connectors(&apple->drm);
	component_unbind_all(dev, NULL);
	dcp_unlink(&apple->drm);
	return ret;
}

static void apple_drm_uninit(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);

	apple_g15_display_withdraw(dev);

	/* An aborted takeover leaves its DCPs running until the reboot. */
	if (apple->aborted)
		return;

	unregister_pm_notifier(&apple->pm_nb);

	/* Before anything of the DRM device is torn down. */
	t8132_ext_drm_fini(&apple->drm);
	apple_drm_quiesce_connectors(&apple->drm);
	drm_dev_unplug(&apple->drm);
	drm_atomic_helper_shutdown(&apple->drm);

	component_unbind_all(dev, NULL);

	dcp_unlink(&apple->drm);
	dev_set_drvdata(dev, NULL);
}

static int apple_drm_bind(struct device *dev)
{
	return apple_drm_init(dev);
}

static void apple_drm_unbind(struct device *dev)
{
	apple_drm_uninit(dev);
}

const struct component_master_ops apple_drm_ops = {
	.bind	= apple_drm_bind,
	.unbind	= apple_drm_unbind,
};

static int add_dcp_components(struct device *dev,
			      struct component_match **matchptr)
{
	struct device_node *np, *endpoint, *port;
	int num = 0;

	for_each_matching_node(np, apple_dcp_id_tbl) {
		if (apple_dcp_usable(np)) {
			drm_of_component_match_add(dev, matchptr,
						   component_compare_of, np);
			num++;
			for_each_endpoint_of_node(np, endpoint) {
				port = of_graph_get_remote_port_parent(endpoint);
				if (!port)
					continue;

#if !IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
				if (of_device_is_compatible(port, "apple,dpaudio")) {
					of_node_put(port);
					continue;
				}
#endif

				/*
				 * The ATC phy driver is not part of the component
				 * collection for the Apple display-subsystem so
				 * ignore it here.
				 */
				if (of_device_is_compatible(port, "apple,t8103-atcphy") ||
				    of_device_is_compatible(port, "apple,t8122-atcphy")) {
					of_node_put(port);
					continue;
				}

				if (of_device_is_available(port))
					drm_of_component_match_add(dev, matchptr,
							   component_compare_of,
							   port);
				of_node_put(port);
			}
		}
	}

	return num;
}

static int apple_platform_probe(struct platform_device *pdev)
{
	struct device *mdev = &pdev->dev;
	struct component_match *match = NULL;
	int num_dcp, ret;

	/* add DCP components, handle less than 1 as probe error */
	num_dcp = add_dcp_components(mdev, &match);
	if (num_dcp < 1)
		return -ENODEV;

	ret = component_master_add_with_match(mdev, &apple_drm_ops, match);
	if (ret)
		return ret;

	/*
	 * The display restores its own state across system sleep, so the PM
	 * core need not switch to the text console first (which flashes the
	 * kernel log on screen on every suspend and resume).
	 */
	pm_vt_switch_required(mdev, false);
	return 0;
}

static void apple_platform_remove(struct platform_device *pdev)
{
	pm_vt_switch_unregister(&pdev->dev);
	component_master_del(&pdev->dev, &apple_drm_ops);
}

static const struct of_device_id of_match[] = {
	{ .compatible = "apple,display-subsystem" },
	{ .compatible = "apple,t6030-display-subsystem" },
	{ .compatible = "apple,t8122-display-subsystem" },
	{ .compatible = "apple,t6031-display-subsystem" },
	{ .compatible = "apple,t6034-display-subsystem" },
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T8132)
	{ .compatible = "apple,t8132-display-subsystem" },
	/* Accept the qualified handoff emitted by the existing boot loader. */
	{ .compatible = "apple,t8132-display-diagnostics" },
#endif
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

#ifdef CONFIG_PM_SLEEP
/*
 * Left stopped by a suspend that never reached the commit. Nothing can commit to
 * them, and a refused sleep has to be able to try again.
 */
static bool apple_drm_firmware_stopped(struct apple_drm_private *apple)
{
	struct drm_crtc *crtc;
	bool stopped = false;

	drm_for_each_crtc(crtc, &apple->drm) {
		if (t8132_ext_crtc(crtc) ||
		    !dcp_s2r_firmware_stopped(to_apple_crtc(crtc)->dcp))
			return false;
		stopped = true;
	}

	return stopped;
}

static int apple_platform_suspend(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);

	/* Still suspended from before a dark wake. */
	if (apple && apple->dark_held)
		return 0;

	if (apple && !apple->aborted) {
		apple->s2r_skipped = apple_drm_firmware_stopped(apple);
		if (!apple->s2r_skipped)
			return drm_mode_config_helper_suspend(&apple->drm);
	}

	return 0;
}

/*
 * The DCP of an external display sends the hotplug event for its CRTC
 * itself, once the modes of the new link are in; see dcp_hold_resume_event().
 * Returns false if any of @crtcs needs the event now.
 */
static bool apple_drm_hold_resume_events(struct apple_drm_private *apple, u32 crtcs)
{
	struct drm_crtc *crtc;
	bool held = true;

	drm_for_each_crtc(crtc, &apple->drm) {
		if ((crtcs & drm_crtc_mask(crtc)) && !t8132_ext_crtc(crtc))
			held &= dcp_hold_resume_event(to_apple_crtc(crtc)->dcp);
	}
	return held;
}

static void apple_drm_resume_outputs(struct apple_drm_private *apple)
{
	u32 dropped = 0;

	apple->dark_held = false;
	if (apple->drm.mode_config.suspend_state)
		dropped = apple_drm_drop_stale_crtcs(apple->drm.mode_config.suspend_state);
	/* Have userspace set the mode of a CRTC resumed disabled again. */
	if (dropped && apple_drm_hold_resume_events(apple, dropped))
		dropped = 0;
	drm_mode_config_helper_resume(&apple->drm);
	if (dropped)
		drm_kms_helper_hotplug_event(&apple->drm);
}

static int apple_platform_resume(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);

	if (!apple || apple->aborted || apple->s2r_skipped)
		return 0;

	/* A dark wake goes back to sleep with user space frozen: nothing is lit for it. */
	if (pm_dark_wake()) {
		apple->dark_held = true;
		dev_info(dev, "dark wake: outputs stay off\n");
		return 0;
	}

	apple_drm_resume_outputs(apple);
	return 0;
}

/* The suspend after a dark wake failed before it suspended this device. */
static void apple_platform_complete(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);

	if (apple && apple->dark_held && !pm_dark_wake())
		apple_drm_resume_outputs(apple);
}

/* It failed before ->prepare() reached this device, so ->complete() didn't run either. */
static int apple_drm_pm_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	struct apple_drm_private *apple = container_of(nb, struct apple_drm_private, pm_nb);

	if (action == PM_POST_SUSPEND && apple->dark_held) {
		dev_warn(apple->drm.dev, "dark wake over without a resume: outputs back now\n");
		apple_drm_resume_outputs(apple);
	}

	return NOTIFY_DONE;
}

static const struct dev_pm_ops apple_platform_pm_ops = {
	.suspend	= apple_platform_suspend,
	.resume		= apple_platform_resume,
	.complete	= apple_platform_complete,
};
#else
static int apple_drm_pm_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	return NOTIFY_DONE;
}
#endif

static struct platform_driver apple_platform_driver = {
	.driver	= {
		.name = "apple-drm",
		.of_match_table	= of_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &apple_platform_pm_ops,
#endif
	},
	.probe		= apple_platform_probe,
	.remove		= apple_platform_remove,
};


static int __init appledrm_register(void)
{
	int ret;
	if (drm_firmware_drivers_only()) {
		apple_j613_display_fail();
		return -ENODEV;
	}

	/* A refused experimental profile keeps the boot framebuffer available. */
	ret = apple_j613_25g83_coldplug();
	if (ret)
		pr_info("J613/25G83 display handoff refused: %d; keeping boot framebuffer\n", ret);

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	ret = dcp_audio_register();
	if (ret) {
		apple_j613_display_fail();
		return ret;
	}
#endif
	ret = dcp_register();
	if (ret)
		goto unregister_audio;
	ret = platform_driver_register(&apple_platform_driver);
	if (!ret)
		return 0;
	dcp_unregister();
unregister_audio:
	apple_j613_display_fail();
#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	dcp_audio_unregister();
#endif
	return ret;
}

static void __exit appledrm_unregister(void)
{
	apple_j613_display_fail();
#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	dcp_audio_unregister();
#endif
	dcp_unregister();
	platform_driver_unregister(&apple_platform_driver);
}

module_init(appledrm_register);
module_exit(appledrm_unregister);

MODULE_AUTHOR("Asahi Linux contributors");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("Dual MIT/GPL");
/*
 * The DCPs take their HDMI and Type-C route mux controls from the display
 * crossbar. Without this, initramfs generators leave it out, it loads only
 * from the root filesystem, and a DCP probe that waits past the deferred
 * probe timeout (an encrypted root's passphrase prompt) gives up its routes.
 * The same holds for the routes' PHYs: on T6050 they are the combo PHYs of
 * atc-t6050-dp, which appledrm does not otherwise depend on.
 * DCP also asks for the battery temperature once while it starts (see
 * dcp_pmu_temperature()), so have the SMC battery registered by then.
 */
MODULE_SOFTDEP("pre: mux-apple-display-crossbar atc-t6050-dp macsmc-power");
