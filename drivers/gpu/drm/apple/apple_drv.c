// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2021 Alyssa Rosenzweig */
/* Based on meson driver which is
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 * Copyright (C) 2014 Endless Mobile
 */

#include <linux/aperture.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/soc/apple/dp-tunnel.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/clients/drm_client_setup.h>
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

#include <linux/soc/apple/pmgr-misc.h>

#include "apple_resume.h"
#include "dcp.h"
#include "plane.h"

#define DRIVER_NAME     "apple"
#define DRIVER_DESC     "Apple display controller DRM driver"

#define MAX_COPROCESSORS 8

struct apple_drm_private {
	struct drm_device drm;
};

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

		dcp = platform_get_drvdata(to_apple_crtc(crtc)->dcp);
		if (dcp->fw_compat == DCP_FIRMWARE_H17P &&
		    dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17G) {
			/* H17P applies the stored fourth byte as straight alpha. */
			sizes->surface_bpp = 32;
			sizes->surface_depth = 32;
			break;
		}
	}

	return drm_fbdev_dma_driver_fbdev_probe(helper, sizes);
}
#endif

/*
 * A compositor or boot splash is closing the device.  drm_release() has
 * already taken its file off the file list, so the Type-C fabric can tell
 * whether any other one still has the device open.
 */
static void apple_drm_postclose(struct drm_device *drm, struct drm_file *file)
{
	if (file->was_master)
		dcp_typec_reorder();
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
	return READ_ONCE(apple_connector->connected) ? connector_status_connected :
						    connector_status_disconnected;
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

static void apple_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	struct apple_dcp *dcp = platform_get_drvdata(apple_crtc->dcp);
	struct drm_crtc_state *crtc_state;
	bool locked;
	int ret;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	locked = false;
	ret = to_apple_atomic_state(state)->failed_routes & drm_crtc_mask(crtc) ?
		-EIO : dcp_typec_follow_crtc(crtc, state, &locked);
	/* A failed move must never power or program another sink's pipeline. */
	if (ret || !dcp_crtc_route_ready(crtc, state, false))
		goto failed;

	if (crtc_state->active && dcp_crtc_needs_route_start(dcp)) {
		/* Establish the link/catalog before powering the assigned IOMFB. */
		ret = dcp_typec_prepare_route(dcp);
		if (ret)
			goto failed;
	}
	if (crtc_state->active && !dcp_crtc_route_ready(crtc, state, true))
		goto failed;
	if (crtc_state->active && !dcp->main_display && !apple_crtc->fabric_held &&
	    dcp->fw_compat != DCP_FIRMWARE_H17P) {
		/* Raise the fabric before external scanout starts. */
		if (apple_pmgr_fabric_hold())
			dev_warn(dcp->dev, "fabric raise not confirmed before scanout\n");
		apple_crtc->fabric_held = true;
	}
	if (crtc_state->active &&
	    (crtc_state->active_changed || dcp_crtc_needs_route_start(dcp))) {
		dcp_poweron(apple_crtc->dcp);
		crtc_state->color_mgmt_changed = true;
	} else if (crtc_state->active && drm_atomic_crtc_needs_modeset(crtc_state)) {
		/* A cable generation can change while the CRTC stays active. */
		dcp_external_recover(to_apple_crtc(crtc)->dcp);
	}
	if (crtc_state->active)
		dcp_crtc_atomic_modeset(crtc, state);
	dcp_typec_follow_done(locked);
	return;

failed:
	if (!ret)
		ret = -EAGAIN;
	dcp_mode_invalidate(&dcp->mode_state);
	dev_err(dcp->dev, "CRTC route unavailable (%d); modeset deferred\n", ret);
	/* Flush completes any pending flip without issuing a firmware swap. */
	schedule_work(&dcp->vblank_wq);
	if (dcp->connector)
		dcp_route_failure_notify(dcp->connector);
	dcp_typec_follow_done(locked);
}

static void apple_crtc_atomic_disable(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state;
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	if (crtc_state->active_changed && !crtc_state->active) {
		dcp_poweroff(apple_crtc->dcp);
		if (apple_crtc->fabric_held) {
			apple_crtc->fabric_held = false;
			apple_pmgr_fabric_release();
		}
	}

	if (crtc->state->event && !crtc->state->active) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);

		crtc->state->event = NULL;
	}
}

static void apple_crtc_atomic_begin(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct apple_crtc *apple_crtc = to_apple_crtc(crtc);
	unsigned long flags;

	if (crtc->state->event) {
		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		apple_crtc->event = crtc->state->event;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
		crtc->state->event = NULL;
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
		dcp_set_crc(crtc, enabled);

	return ret;
}

static int apple_crtc_verify_crc_source(struct drm_crtc *crtc,
					const char *source,
					size_t *values_cnt)
{
	bool enabled;

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

static struct drm_atomic_state *apple_atomic_state_alloc(struct drm_device *dev)
{
	struct apple_atomic_state *state = kzalloc_obj(*state);

	if (!state)
		return NULL;
	if (drm_atomic_state_init(dev, &state->base)) {
		kfree(state);
		return NULL;
	}
	return &state->base;
}

static void apple_atomic_state_clear(struct drm_atomic_state *state)
{
	drm_atomic_state_default_clear(state);
	to_apple_atomic_state(state)->failed_routes = 0;
}

static void apple_atomic_state_free(struct drm_atomic_state *state)
{
	drm_atomic_state_default_release(state);
	kfree(to_apple_atomic_state(state));
}

static const struct drm_mode_config_funcs apple_mode_config_funcs = {
	.atomic_state_alloc	= apple_atomic_state_alloc,
	.atomic_state_clear	= apple_atomic_state_clear,
	.atomic_state_free	= apple_atomic_state_free,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
	.fb_create		= drm_gem_fb_create,
};

static void apple_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_plane *plane;
	struct drm_plane_state *old, *new;
	int i;

	/* Retain displaced scanout even when the CRTC stays inactive. */
	for_each_oldnew_plane_in_state(state, plane, old, new, i) {
		struct apple_plane_state *apple_state = to_apple_plane_state(new);
		struct dcp_fb_reference *entry = apple_state->retirement;

		if (!entry)
			continue;

		apple_state->retirement = NULL;
		dcp_retain_framebuffer(to_apple_crtc(old->crtc)->dcp, entry);
	}

	drm_atomic_helper_commit_tail_rpm(state);
}

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
	apple_connector_invalidate_edid(apple_connector);
	mutex_destroy(&apple_connector->edid_lock);
	drm_connector_cleanup(connector);
	kfree(apple_connector->color_elements.data);
	kfree(apple_connector->timing_elements.data);
	kfree(apple_connector->display_attributes.data);
	kfree(apple_connector->transport.data);
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
				    struct drm_atomic_state *state)
{
	struct apple_connector *apple_connector = to_apple_connector(conn);
	struct drm_connector_state *conn_state;
	struct drm_encoder *encoder;

	/*
	 * A Type-C port has one encoder, whose possible_crtcs either already
	 * names the pipeline the fabric routed the port to or spans the set
	 * the fabric routes within: on dual-stream machines, and where the
	 * route follows the CRTC (see dcp_typec_follow_crtc()).  So it is the
	 * only answer there is.
	 *
	 * H17P (T8140, T8142): Type-C tunnels are routed before the modeset
	 * and cannot move here. Reject stale topology and explicit assignments
	 * to another pipeline.
	 */
	if (apple_connector->port_encoder) {
		struct apple_dcp *target;

		conn_state = drm_atomic_get_new_connector_state(state, conn);
		target = conn_state && conn_state->crtc ?
			platform_get_drvdata(to_apple_crtc(conn_state->crtc)->dcp) : NULL;
		if (target && target->fw_compat == DCP_FIRMWARE_H17P &&
		    READ_ONCE(apple_connector->dcp) !=
		    to_apple_crtc(conn_state->crtc)->dcp)
			return NULL;
		return apple_connector->port_encoder;
	}

	drm_connector_for_each_possible_encoder(conn, encoder)
		return encoder;

	return NULL;
}

static const struct drm_connector_helper_funcs apple_connector_helper_funcs = {
	.get_modes		= dcp_get_modes,
	.mode_valid		= dcp_mode_valid,
	.atomic_best_encoder	= apple_connector_atomic_best_encoder,
};

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
	int ret;
	int connector_type;
	u32 surf;
	int zpos = 0;
	bool supports_l10r = !dcp_fw_compat_is_12_x(dcp);
	struct apple_dcp *dcp_data = platform_get_drvdata(dcp);
	bool supports_xrgb2101010;
	enum drm_plane_type plane_type;

	supports_xrgb2101010 = dcp_data->fw_compat != DCP_FIRMWARE_H17P ||
		dcp_data->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17G;
	if (dcp_data->fw_compat == DCP_FIRMWARE_H17P &&
	    bitmap_empty(iomfb_surfaces, DCP_MAX_PLANES))
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

	for_each_set_bit(surf, iomfb_surfaces, DCP_MAX_PLANES) {
		if (dcp_fw_compat_is_14_7(dcp))
			break;
		plane_type = (zpos == 0) ? DRM_PLANE_TYPE_PRIMARY : DRM_PLANE_TYPE_OVERLAY;
		planes[zpos] = apple_plane_init(drm, 1U << num, surf,
						supports_l10r,
						supports_xrgb2101010,
						plane_type);
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
	/* No matrix property on a pipe whose firmware is never sent one. */
	if (dcp_ctm_supported(dcp))
		drm_crtc_enable_color_mgmt(&crtc->base, 0, true, 0);

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
	if (connector_type == DRM_MODE_CONNECTOR_USB) {
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
 * connector names.
 */
static int apple_probe_typec_ports(struct drm_device *drm,
				   struct platform_device **dcp,
				   u32 *crtc_mask, int num_dcp)
{
	unsigned int idx, nr_ports = dcp_typec_nr_ports();
	int i, ret;

	/* Keep the existing physical-port connector numbers stable. On the
	 * M2 Pro/Max laptops, append one more connector per port for a second
	 * USB4 DP tunnel.
	 */
	for (idx = 0; idx < nr_ports * (dcp_typec_dual_stream() ? 2 : 1);
	     idx++) {
		struct apple_connector *connector;
		struct apple_encoder *enc;
		unsigned int port_idx = idx % nr_ports;
		bool secondary = idx >= nr_ports;
		u32 mask = 0;

		/* Only a failed T8122 dcpext could drive it: no connector. */
		if (dcp_typec_port_unusable(port_idx)) {
			drm_warn(drm, "Type-C port %u: its display pipeline is unusable\n",
				 port_idx);
			continue;
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

		connector->base.polled = DRM_CONNECTOR_POLL_HPD;
		connector->connected = false;
		connector->dcp = NULL;

		for (i = 0; i < num_dcp; i++) {
			if (!dcp_typec_port_has_candidate(port_idx, dcp[i]))
				continue;
			/*
			 * A dock's second stream (DPIN1) always runs on the
			 * lowest-indexed Type-C-only pipeline (dcpext1). Fix the
			 * connector to it: compositors read possible_crtcs once
			 * and would otherwise pair it with a pipeline the stream
			 * can never be routed to.
			 */
			if (secondary) {
				if (dcp_is_typec_only(dcp[i]) &&
				    (!mask || crtc_mask[i] < mask))
					mask = crtc_mask[i];
				continue;
			}
			mask |= crtc_mask[i];
		}

		if (!mask) {
			drm_warn(drm, "Type-C port %u has no display pipeline\n",
					 port_idx);
			return -ENODEV;
		}

		/*
		 * One encoder for the port, spanning every pipeline that can
		 * drive it.  Attaching one encoder per pipeline would describe
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

		dcp_typec_port_set_connector(port_idx, secondary, connector);
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

static const struct of_device_id apple_dcp_id_tbl[] = {
	{ .compatible = "apple,dcp" },
	{ .compatible = "apple,dcpext" },
	{ .compatible = "apple,t6030-dcp" },
	{ .compatible = "apple,t8122-dcp" },
	{ .compatible = "apple,t6031-dcp" },
	{},
};

/*
 * A T6030 external display processor is a pipe of this DRM device once the
 * display gate gave it its display DART; on the manual diagnostic path it
 * runs a separate session instead.
 */
static bool apple_dcp_in_drm(const struct device_node *np)
{
	if (!of_device_is_compatible(np, "apple,t6030-dcpext"))
		return true;
	return dcp_t6030_ext_native(np);
}

static void apple_drm_quiesce_connectors(struct drm_device *drm)
{
	struct drm_connector_list_iter iter;
	struct drm_connector *connector;

	drm_connector_list_iter_begin(drm, &iter);
	drm_for_each_connector_iter(connector, &iter) {
		struct apple_connector *apple_connector = to_apple_connector(connector);

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

/* Does any pipeline of @drm run H17-generation firmware (T8140, T8142)? */
static bool apple_drm_has_h17p(struct drm_device *drm)
{
	struct drm_crtc *crtc;

	drm_for_each_crtc(crtc, drm) {
		struct apple_dcp *dcp = platform_get_drvdata(to_apple_crtc(crtc)->dcp);

		if (dcp && dcp->fw_compat == DCP_FIRMWARE_H17P)
			return true;
	}
	return false;
}

/* A supported firmware-described external provider participates in component matching. */
static bool apple_dcp_usable(struct device_node *np)
{
	return of_device_is_available(np) &&
	       (!of_device_is_compatible(np, "apple,t8140-dcpext") ||
		dcp_h17p_external_supported(np));
}

static int apple_drm_init_dcp(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);
	struct platform_device *dcp[MAX_COPROCESSORS];
	struct drm_encoder *encoder[MAX_COPROCESSORS];
	u32 crtc_mask[MAX_COPROCESSORS] = {};
	struct device_node *np;
	u64 timeout;
	int i, ret, num_dcp = 0;

	for_each_matching_node(np, apple_dcp_id_tbl) {
		bool dcp_ext;
		if (!apple_dcp_in_drm(np) || !apple_dcp_usable(np))
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

	for (i = 0; i < num_dcp; i++) {
		ret = dcp_start(dcp[i]);
		if (ret)
			dev_warn(dev, "DCP[%d] failed to start: %d\n", i, ret);
	}

	/*
	 * Starting DPTX might take some time.
	 */
	timeout = get_jiffies_64() + msecs_to_jiffies(3000);

	for (i = 0; i < num_dcp; ++i) {
		u64 jiffies = get_jiffies_64();
		u64 wait = time_after_eq64(jiffies, timeout) ?
				   0 :
				   timeout - jiffies;
		ret = dcp_wait_ready(dcp[i], wait);
		/* There is nothing we can do if a dcp/dcpext does not boot
		 * (successfully). Ignoring it should not do any harm now.
		 * Needs to reevaluated when adding dcpext support.
		 */
		if (ret)
			dev_warn(dev, "DCP[%d] not ready: %d\n", i, ret);
	}
	/* HACK: Wait for dcp* to settle before a modeset */
	msleep(100);

	return 0;
}

static int apple_drm_init(struct device *dev)
{
	struct apple_drm_private *apple;
	struct resource fb_r;
	resource_size_t fb_size;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;

	ret = apple_get_fb_resource(dev, "framebuffer", &fb_r);
	if (ret)
		return ret;

	apple = devm_drm_dev_alloc(dev, &apple_drm_driver,
				   struct apple_drm_private, drm);
	if (IS_ERR(apple))
		return PTR_ERR(apple);

	dev_set_drvdata(dev, apple);

	ret = component_bind_all(dev, apple);
	if (ret)
		return ret;

	ret = drmm_mode_config_init(&apple->drm);
	if (ret)
		goto err_components;

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

	ret = apple_drm_init_dcp(dev);
	if (ret)
		goto err_unbind;

	drm_mode_config_reset(&apple->drm);

	fb_size = fb_r.end - fb_r.start + 1;
	ret = aperture_remove_conflicting_devices(fb_r.start, fb_size,
						  apple_drm_driver.name);
	if (ret) {
		dev_err(dev, "Failed remove fb: %d\n", ret);
		goto err_unbind;
	}

	ret = drm_dev_register(&apple->drm, 0);
	if (ret)
		goto err_unbind;

	/* the fabric keeps order only once registered: catch up on probe */
	dcp_typec_reorder();

	drm_client_setup_with_fourcc(&apple->drm, DRM_FORMAT_XRGB8888);

	return 0;

err_unbind:
	apple_drm_quiesce_connectors(&apple->drm);
	component_unbind_all(dev, NULL);
	dcp_unlink(&apple->drm);
	return ret;
err_components:
	component_unbind_all(dev, NULL);
	return ret;
}

static void apple_drm_uninit(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);

	apple_drm_quiesce_connectors(&apple->drm);
	/* H17P keeps its live scanout across unbind: unplug, as aurora-wip. */
	if (apple_drm_has_h17p(&apple->drm))
		drm_dev_unplug(&apple->drm);
	else
		drm_dev_unregister(&apple->drm);
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
		if (apple_dcp_in_drm(np) && apple_dcp_usable(np)) {
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
	int num_dcp;

	/* add DCP components, handle less than 1 as probe error */
	num_dcp = add_dcp_components(mdev, &match);
	if (num_dcp < 1)
		return -ENODEV;

	return component_master_add_with_match(mdev, &apple_drm_ops, match);
}

static void apple_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &apple_drm_ops);
}

static const struct of_device_id of_match[] = {
	{ .compatible = "apple,display-subsystem" },
	{ .compatible = "apple,t6030-display-subsystem" },
	{ .compatible = "apple,t8122-display-subsystem" },
	{ .compatible = "apple,t6031-display-subsystem" },
	{ .compatible = "apple,t8140-display-subsystem" },
	{ .compatible = "apple,t8142-display-subsystem" },
	{}
};
MODULE_DEVICE_TABLE(of, of_match);

#ifdef CONFIG_PM_SLEEP
static int apple_platform_suspend(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);

	if (apple)
		return drm_mode_config_helper_suspend(&apple->drm);

	return 0;
}

static int apple_platform_resume(struct device *dev)
{
	struct apple_drm_private *apple = dev_get_drvdata(dev);
	bool dropped = false;

	if (apple) {
		/* H17P (T8140, T8142) only, as on aurora-wip. */
		if (apple->drm.mode_config.suspend_state &&
		    apple_drm_has_h17p(&apple->drm))
			dropped = apple_drm_drop_stale_crtcs(apple->drm.mode_config.suspend_state);
		drm_mode_config_helper_resume(&apple->drm);
		/* Have userspace set the mode of a CRTC resumed disabled again. */
		if (dropped)
			drm_kms_helper_hotplug_event(&apple->drm);
	}

	return 0;
}

static const struct dev_pm_ops apple_platform_pm_ops = {
	.suspend	= apple_platform_suspend,
	.resume		= apple_platform_resume,
};
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
	if (drm_firmware_drivers_only())
		return -ENODEV;

#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	ret = dcp_audio_register();
	if (ret)
		return ret;
#endif
	ret = dcp_register();
	if (ret)
		goto unregister_audio;
	ret = platform_driver_register(&apple_platform_driver);
	if (!ret)
		return 0;
	dcp_unregister();
unregister_audio:
#if IS_ENABLED(CONFIG_DRM_APPLE_AUDIO)
	dcp_audio_unregister();
#endif
	return ret;
}

static void __exit appledrm_unregister(void)
{
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
