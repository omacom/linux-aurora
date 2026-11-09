/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#ifndef __APPLE_IOMFB_V14_7_PLANE_H__
#define __APPLE_IOMFB_V14_7_PLANE_H__

#include <linux/types.h>

#include <drm/drm_framebuffer.h>
#include <drm/drm_plane.h>
#include <drm/drm_rect.h>

/*
 * The 14.7 presenter scans out one unscaled image of the mode's size from
 * the top-left corner of the framebuffer. Judge a plane by its clipped
 * rectangles, which drm_atomic_helper_check_plane_state() has set by the
 * time the CRTC is checked: a plane that reaches past the right or bottom
 * edge of the mode is clipped there, and what remains is exactly what the
 * presenter scans out. Compositors put up such planes, for example when they
 * centre a mirrored output and round its borders up on both sides.
 *
 * A plane that starts left of or above the mode, or that is scaled, starts
 * its clipped source inside the framebuffer or differs in size from the
 * mode, which the presenter cannot show. Clipping rounds the source toward
 * 1:1, so a plane that reaches past the edge with a source less than a pixel
 * off unscaled may clip to exactly the mode's size; it is then taken and
 * shown unscaled.
 *
 * An external pipe may show the top-left of a larger framebuffer; the panel
 * takes only a framebuffer of its mode's size.
 */
static inline bool dcp_v14_plane_fits_mode(const struct drm_plane_state *p,
					   u32 width, u32 height, bool external)
{
	const struct drm_framebuffer *fb = p->fb;

	if (p->src.x1 || p->src.y1 || p->dst.x1 || p->dst.y1)
		return false;
	if (drm_rect_width(&p->src) != (int)(width << 16) ||
	    drm_rect_height(&p->src) != (int)(height << 16) ||
	    drm_rect_width(&p->dst) != (int)width ||
	    drm_rect_height(&p->dst) != (int)height)
		return false;
	if (external)
		return fb->width >= width && fb->height >= height;
	return fb->width == width && fb->height == height;
}

/*
 * The size of the image the presenter scans out for a visible plane that
 * dcp_v14_plane_fits_mode() took: its clipped source on an external pipe,
 * where the framebuffer may be larger, and the framebuffer on the panel.
 */
static inline void dcp_v14_plane_scanout_size(const struct drm_plane_state *p,
					      bool external, u32 *width, u32 *height)
{
	if (external) {
		*width = drm_rect_width(&p->src) >> 16;
		*height = drm_rect_height(&p->src) >> 16;
	} else {
		*width = p->fb->width;
		*height = p->fb->height;
	}
}

#endif /* __APPLE_IOMFB_V14_7_PLANE_H__ */
