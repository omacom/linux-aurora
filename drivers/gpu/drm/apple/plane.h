// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright (C) The Asahi Linux Contributors
 */

#ifndef __APPLE_PLANE_H__
#define __APPLE_PLANE_H__

#include <drm/drm_plane.h>

#include <linux/types.h>

#include "iomfb_plane.h"

struct apple_plane {
	struct drm_plane base;
	u32 iomfb_surf;
	/* Reserved for the kernel camera indicator; userspace may not use it. */
	bool indicator;
	/* The kernel commit in flight, compared by pointer only. */
	const struct drm_atomic_commit *indicator_commit;
};

#define to_apple_plane(x) container_of(x, struct apple_plane, base)

struct dcp_fb_reference;

struct apple_plane_state {
	struct drm_plane_state base;
	struct dcp_surface surf;
	struct dcp_rect src_rect;
	struct dcp_rect dst_rect;
	u64 iova;
	struct dcp_fb_reference *retirement;
};

#define to_apple_plane_state(x) container_of(x, struct apple_plane_state, base)

struct drm_plane *apple_plane_init(struct drm_device *dev,
				   unsigned long possible_crtcs,
				   u32 iomfb_surf,
				   bool supports_l10r,
				   bool supports_xrgb2101010,
				   bool xrgb_is_alpha,
				   enum drm_plane_type type,
				   bool interchange,
				   bool premultiplied, bool native26);
struct drm_plane *apple_plane_init_v14_7(struct drm_device *dev,
					 unsigned long possible_crtcs,
					 u32 iomfb_surf,
					 enum drm_plane_type type);

/* T8132 internal panel: the qualified single-surface firmware profile. */
struct drm_plane *apple_plane_init_v26_6(struct drm_device *dev,
				      unsigned long possible_crtcs, u32 iomfb_surf);

#endif /* __APPLE_PLANE_H__ */
