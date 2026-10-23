// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Copyright (C) The Asahi Linux Contributors
 */

#include "plane.h"

#include "dcp-internal.h"
#include "iomfb_internal.h"

#include "iomfb_plane.h"
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_plane.h>

#include <linux/math.h>
#include <linux/overflow.h>
#include <linux/types.h>

#define APPLE_GPU_CACHELINE	16
#define FRAC_16_16(mult, div)    (((mult) << 16) / (div))

struct apple_plane_layout {
	struct dcp_image_layout image;
	u32 width;
	u32 height;
	u32 offset;
	u32 stride;
	u32 size;
	u32 tile_size;
};

static int apple_plane_ix_bpp(const struct drm_format_info *fmt, u32 plane,
			      u32 *bpp)
{
	u32 bw = drm_format_info_block_width(fmt, plane);
	u32 bh = drm_format_info_block_height(fmt, plane);
	u32 bits = fmt->char_per_block[plane] * 8;
	u32 block_pixels;

	/*
	 * P030/P230 pack three pixels per block. Neither an integer bpp
	 * (which would be zero) nor a truncated bpp describes their tiles.
	 * Keep these formats linear until their interchange packing is known.
	 */
	if (!bw || !bh || check_mul_overflow(bw, bh, &block_pixels) || !bits ||
	    bits % block_pixels)
		return -EINVAL;
	*bpp = bits / block_pixels;

	return 0;
}

static bool apple_plane_format_mod_supported(struct drm_plane *plane,
					     u32 format, u64 modifier)
{
	const struct drm_format_info *fmt = drm_format_info(format);
	u32 bpp;
	unsigned int i;

	if (!fmt)
		return false;

	/* The callback replaces DRM's generic modifier-list validation. */
	for (i = 0; i < plane->modifier_count; i++)
		if (modifier == plane->modifiers[i])
			break;
	if (i == plane->modifier_count)
		return false;

	switch (modifier) {
	case DRM_FORMAT_MOD_LINEAR:
	case DRM_FORMAT_MOD_INVALID:
		return true;
	case DRM_FORMAT_MOD_APPLE_INTERCHANGE:
		for (u32 i = 0; i < fmt->num_planes; i++)
			if (apple_plane_ix_bpp(fmt, i, &bpp))
				return false;
		return true;
	default:
		return false;
	}
}

static int apple_get_ix_layout(u32 width, u32 height,
			       const struct drm_format_info *fmt, u32 plane,
			       struct apple_plane_layout *out)
{
	struct dcp_image_layout *layout = &out->image;
	u32 bpp, meta_tile_w, meta_tile_h, tiles, bytes;
	int ret;

	ret = apple_plane_ix_bpp(fmt, plane, &bpp);
	if (ret)
		return ret;

	/*
	 * Compression subtiles are always 16x16 with 8 bytes of metadata,
	 * but IOMFB requires values relative to the image tile dimensions.
	 */
	if (fmt->is_yuv && plane == 0) {
		layout->tile_w = 32;
		layout->tile_h = 32;
		layout->meta_tile_bytes = 32;
	} else {
		layout->tile_w = 16;
		layout->tile_h = 16;
		layout->meta_tile_bytes = 8;
	}

	layout->width_tiles = (width - 1) / layout->tile_w + 1;
	layout->height_tiles = (height - 1) / layout->tile_h + 1;
	layout->tile_bytes = layout->tile_w * layout->tile_h * bpp / 8;
	if (layout->tile_bytes > U16_MAX ||
	    check_mul_overflow(layout->width_tiles, layout->tile_bytes,
			       &out->stride) ||
	    check_mul_overflow(out->stride, layout->height_tiles, &bytes) ||
	    check_add_overflow(bytes, APPLE_GPU_CACHELINE - 1, &bytes))
		return -EINVAL;
	layout->meta_offset = bytes & ~(APPLE_GPU_CACHELINE - 1);

	/* The rounded dimensions and all wire-format sizes must fit u32. */
	if (width > BIT(31) || height > BIT(31))
		return -EINVAL;
	meta_tile_w = DIV_ROUND_UP(roundup_pow_of_two(width), layout->tile_w);
	meta_tile_h = DIV_ROUND_UP(roundup_pow_of_two(height), layout->tile_h);
	if (check_mul_overflow(meta_tile_w, meta_tile_h, &tiles) ||
	    check_mul_overflow(tiles, layout->meta_tile_bytes, &bytes) ||
	    check_add_overflow(bytes, APPLE_GPU_CACHELINE - 1, &bytes))
		return -EINVAL;
	layout->meta_size = bytes & ~(APPLE_GPU_CACHELINE - 1);
	if (check_add_overflow(layout->meta_offset, layout->meta_size,
			       &out->size))
		return -EINVAL;
	out->tile_size = layout->tile_bytes;

	return 0;
}

/* Use exactly the same geometry for admission and surface construction. */
static int apple_plane_get_fb_layout(struct drm_framebuffer *fb,
				     struct apple_plane_layout *planes,
				     u32 *buf_size)
{
	const struct drm_format_info *fmt = fb->format;
	struct drm_gem_dma_object *first;
	u32 end;
	int ret;

	if (!fb->width || !fb->height || !fmt->hsub || !fmt->vsub ||
	    !fmt->num_planes || fmt->num_planes > DCP_SURF_MAX_PLANES)
		return -EINVAL;
	first = drm_fb_dma_get_gem_obj(fb, 0);
	if (!first)
		return -EINVAL;

	*buf_size = 0;
	for (u32 i = 0; i < fmt->num_planes; i++) {
		struct apple_plane_layout *p = &planes[i];
		struct drm_gem_dma_object *obj = drm_fb_dma_get_gem_obj(fb, i);

		/* DCP currently uses one GEM object for the whole surface. */
		if (obj != first || fb->offsets[i] < fb->offsets[0])
			return -EINVAL;
		*p = (struct apple_plane_layout){
			.width = i ? DIV_ROUND_UP_ULL(fb->width, fmt->hsub) :
				     fb->width,
			.height = i ? DIV_ROUND_UP_ULL(fb->height, fmt->vsub) :
				      fb->height,
			.offset = fb->offsets[i] - fb->offsets[0],
		};
		if (!p->width || !p->height)
			return -EINVAL;

		switch (fb->modifier) {
		case DRM_FORMAT_MOD_APPLE_INTERCHANGE:
			ret = apple_get_ix_layout(p->width, p->height, fmt, i,
						  p);
			if (ret)
				return ret;
			break;
		case DRM_FORMAT_MOD_INVALID:
		case DRM_FORMAT_MOD_LINEAR:
			if (!fb->pitches[i] || fb->pitches[i] & 63 ||
			    check_mul_overflow(p->height, fb->pitches[i],
					       &p->size))
				return -EINVAL;
			p->image.tile_w = drm_format_info_block_width(fmt, i);
			p->image.tile_h = drm_format_info_block_height(fmt, i);
			p->tile_size = p->image.tile_w * p->image.tile_h;
			p->stride = fb->pitches[i];
			break;
		default:
			return -EINVAL;
		}

		if (check_add_overflow(p->offset, p->size, &end) ||
		    (u64)fb->offsets[i] + p->size > obj->base.size ||
		    check_add_overflow(*buf_size, p->size, buf_size))
			return -EINVAL;
	}
	if ((u64)fb->offsets[0] + *buf_size > first->base.size)
		return -EINVAL;

	return 0;
}

static int apple_plane_atomic_check(struct drm_plane *plane,
				    struct drm_atomic_commit *commit)
{
	struct drm_plane_state *new_plane_state;
	struct drm_crtc_state *crtc_state;
	struct drm_rect *dst;
	struct apple_plane_layout layouts[DCP_SURF_MAX_PLANES];
	u32 buf_size;
	int ret;

	new_plane_state = drm_atomic_get_new_plane_state(commit, plane);

	if (!new_plane_state->crtc)
		return 0;

	crtc_state = drm_atomic_get_crtc_state(commit, new_plane_state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	/*
	 * DCP limits downscaling to 2x and upscaling to 4x. Attempting to
	 * scale outside these bounds errors out when swapping.
	 *
	 * This function also takes care of clipping the src/dest rectangles,
	 * which is required for correct operation. Partially off-screen
	 * surfaces may appear corrupted.
	 *
	 * DCP does not distinguish plane types in the hardware, so we set
	 * can_position. If the primary plane does not fill the screen, the
	 * hardware will fill in zeroes (black).
	 */
	ret = drm_atomic_helper_check_plane_state(new_plane_state, crtc_state,
						  FRAC_16_16(1, 2),
						  FRAC_16_16(4, 1),
						  true, true);
	if (ret < 0)
		return ret;

	ret = apple_plane_get_fb_layout(new_plane_state->fb, layouts, &buf_size);
	if (ret)
		return ret;

	if (!new_plane_state->visible)
		return 0;

	/*
	 * DCP does not allow a surface to clip off the screen, and will crash
	 * if any blended surface is smaller than 32x32. Reject the atomic op
	 * if the plane will crash DCP.
	 *
	 * This is most pertinent to cursors. Userspace should fall back to
	 * software cursors if the plane check is rejected.
	 */
	dst = &new_plane_state->dst;
	if (drm_rect_width(dst) < 32 || drm_rect_height(dst) < 32) {
		dev_err_once(commit->dev->dev,
			"Plane operation would have crashed DCP! Rejected!\n\
			DCP requires 32x32 of every plane to be within screen space.\n\
			Your compositor asked to overlay [%dx%d, %dx%d] on %dx%d.\n\
			This is not supported, and your compositor should have\n\
			switched to software compositing when this operation failed.\n\
			You should not have noticed this at all. If your screen\n\
			froze/hitched, or your compositor crashed, please report\n\
			this to the your compositor's developers. We will not\n\
			throw this error again until you next reboot.\n",
			dst->x1, dst->y1, dst->x2, dst->y2,
			crtc_state->mode.hdisplay, crtc_state->mode.vdisplay);
		return -EINVAL;
	}

	return 0;
}

/*
 * DRM specifies rectangles as start and end coordinates.  DCP specifies
 * rectangles as a start coordinate and a width/height. Convert a DRM rectangle
 * to a DCP rectangle.
 */
static struct dcp_rect drm_to_dcp_rect(const struct drm_rect *rect)
{
	return (struct dcp_rect){ .x = rect->x1,
				  .y = rect->y1,
				  .w = drm_rect_width(rect),
				  .h = drm_rect_height(rect),
	};
}

static struct dcp_rect drm_to_dcp_rect_fp(const struct drm_rect *fp_rect)
{
	struct drm_rect rect;
	drm_rect_fp_to_int(&rect, fp_rect);
	return drm_to_dcp_rect(&rect);
}

static u32 drm_format_to_dcp(u32 drm, enum drm_color_range range)
{
	bool fr = range == DRM_COLOR_YCBCR_FULL_RANGE;
	switch (drm) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return DCP_FORMAT_BGRA;

	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		return DCP_FORMAT_RGBA;

	case DRM_FORMAT_XRGB2101010:
	case DRM_FORMAT_ARGB2101010:
		return DCP_FORMAT_L10R;

	case DRM_FORMAT_ABGR16161616F:
		return DCP_FORMAT_AhGR;

	/* semi planar YCbCr formats, limited and full range */
	case DRM_FORMAT_NV12:
		return fr ? DCP_FORMAT_420F : DCP_FORMAT_420V;
	case DRM_FORMAT_NV16:
		return fr ? DCP_FORMAT_422F : DCP_FORMAT_422V;
	case DRM_FORMAT_NV24:
		return fr ? DCP_FORMAT_444F : DCP_FORMAT_444V;

	/* semi planar 10-bit YCbCr formats, limited and full range */
	case DRM_FORMAT_P010:
		return fr ? DCP_FORMAT_XF20 : DCP_FORMAT_X420;
	case DRM_FORMAT_P210:
		return fr ? DCP_FORMAT_XF22 : DCP_FORMAT_X422;
	/* semi planar 10-bit YCbCr formats, 3x1 blocks, limited and full range */
	case DRM_FORMAT_P030:
		return fr ? DCP_FORMAT_PF20 : DCP_FORMAT_P420;
	case DRM_FORMAT_P230:
		return fr ? DCP_FORMAT_PF22 : DCP_FORMAT_P422;
	/*
	 * TODO: missing DRM fourccs for P410 / P430
	 */
#if defined(DRM_FORMAT_P410)
	case DRM_FORMAT_P410:
		return fr ? DCP_FORMAT_XF44 : DCP_FORMAT_X444;
#endif
#if defined(DRM_FORMAT_P430)
	case DRM_FORMAT_P430:
		return fr ? DCP_FORMAT_PF44 : DCP_FORMAT_P444;
#endif
	}

	pr_warn("DRM format %X not supported in DCP\n", drm);
	return 0;
}

static enum dcp_xfer_func get_xfer_func(bool is_yuv, enum drm_color_encoding enc)
{
	if (!is_yuv)
		return DCP_XFER_FUNC_SDR;

	switch (enc) {
	case DRM_COLOR_YCBCR_BT601:
		return DCP_XFER_FUNC_BT601;
	case DRM_COLOR_YCBCR_BT709:
	case DRM_COLOR_YCBCR_BT2020:
		return DCP_XFER_FUNC_BT1886;
	default:
		return DCP_XFER_FUNC_SDR;
	}
}

static enum dcp_colorspace get_colorspace(bool is_yuv,
					  enum drm_color_encoding enc)
{
	if (!is_yuv)
		return DCP_COLORSPACE_NATIVE;

	switch (enc) {
	case DRM_COLOR_YCBCR_BT601:
		return DCP_COLORSPACE_BT601;
	case DRM_COLOR_YCBCR_BT709:
		return DCP_COLORSPACE_BT709;
	case DRM_COLOR_YCBCR_BT2020:
		return DCP_COLORSPACE_BG_BT2020;
	default:
		return DCP_COLORSPACE_NATIVE;
	}
}

static void apple_plane_atomic_update(struct drm_plane *plane,
				      struct drm_atomic_commit *commit)
{
	struct drm_plane_state *base = drm_atomic_get_new_plane_state(commit, plane);
	struct apple_plane_state *new_state;
	struct drm_gem_dma_object *obj;
	struct apple_plane_layout layouts[DCP_SURF_MAX_PLANES];
	u32 buf_size;
	bool is_premultiplied = false;

	if (!base)
		return;

	new_state = to_apple_plane_state(base);

	if (!base->fb) {
		memset(&new_state->surf, 0, sizeof(new_state->surf));
		return;
	}

	struct drm_framebuffer *fb = base->fb;
	const struct drm_format_info *fmt = fb->format;

	if (WARN_ON_ONCE(apple_plane_get_fb_layout(fb, layouts, &buf_size))) {
		memset(&new_state->surf, 0, sizeof(new_state->surf));
		return;
	}

	/*
	 * DCP doesn't support XBGR8 / XRGB8 / XRGB2101010 / XBGR2101010
	 * natively (XRGB2101010 is scanned out as "l10r", an alpha format,
	 * so its two padding bits are read as alpha). Blending as
	 * pre-multiplied alpha with a black background can be used as
	 * workaround for the bottommost plane.
	 */
	if (fmt->format == DRM_FORMAT_XRGB8888 ||
	    fmt->format == DRM_FORMAT_XBGR8888 ||
	    fmt->format == DRM_FORMAT_XRGB2101010 ||
	    fmt->format == DRM_FORMAT_XBGR2101010)
		is_premultiplied = true;

	new_state->src_rect = drm_to_dcp_rect_fp(&base->src);
	new_state->dst_rect = drm_to_dcp_rect(&base->dst);

	new_state->surf = (struct dcp_surface){
		.is_premultiplied = is_premultiplied,
		.plane_cnt = fb->format->num_planes,
		.plane_cnt2 = fb->format->num_planes,
		.format = drm_format_to_dcp(fmt->format, base->color_range),
		.xfer_func = get_xfer_func(fmt->is_yuv, base->color_encoding),
		.colorspace = get_colorspace(fmt->is_yuv, base->color_encoding),
		.stride = fb->pitches[0],
		.width = fb->width,
		.height = fb->height,
		.surface_id = plane->base.id,
		.buf_size = buf_size,

		/* Only used for compressed or multiplanar surfaces */
		.pix_size = 1,
		.pel_w = 1,
		.pel_h = 1,
		.has_comp = 1,
		.has_planes = 1,
		.has_compr_info = 1,
	};

	/* Populate plane information for planar formats */
	struct dcp_surface *surf = &new_state->surf;
	for (int i = 0; fb->format->num_planes && i < fb->format->num_planes; i++) {
		const struct apple_plane_layout *p = &layouts[i];
		const struct dcp_image_layout *layout = &p->image;

		surf->planes[i] = (struct dcp_plane_info){
			.width = p->width,
			.height = p->height,
			.base = p->offset,
			.offset = p->offset,
			.tile_w = layout->tile_w,
			.tile_h = layout->tile_h,
			.stride = p->stride,
			.size = p->size,
			.tile_size = p->tile_size,
		};

		switch (fb->modifier) {
		case DRM_FORMAT_MOD_APPLE_INTERCHANGE:
			surf->planes[i].address_fmt = DCP_ADDR_FMT_INTERCHANGE_TILED;

			surf->compression_info[i] = (struct dcp_compression_info) {
				.tile_w = layout->tile_w,
				.tile_h = layout->tile_h,
				.data_offset = surf->planes[i].offset,
				.metadata_offset = surf->planes[i].offset + layout->meta_offset,
				.meta_bytes = layout->meta_tile_bytes,
				.tiles_w = layout->width_tiles,
				.tiles_h = layout->height_tiles,
				.tile_bytes = layout->tile_bytes,
				.row_stride = p->stride,
				.compression_type = DCP_COMPRESSION_INTERCHANGE,
			};
			break;
		case DRM_FORMAT_MOD_INVALID:
		case DRM_FORMAT_MOD_LINEAR:
			surf->planes[i].address_fmt = DCP_ADDR_FMT_LINEAR;
			break;
		default:
			break;
		}
	}

	/* the obvious helper call drm_fb_dma_get_gem_addr() adjusts
	 * the address for source x/y offsets. Since IOMFB has a direct
	 * support source position prefer that.
	 */
	obj = drm_fb_dma_get_gem_obj(base->fb, 0);
	if (obj)
		new_state->iova = obj->dma_addr + base->fb->offsets[0];
}

static void apple_plane_cleanup_fb(struct drm_plane *plane,
				   struct drm_plane_state *state)
{
	struct apple_plane_state *apple_state = to_apple_plane_state(state);
	struct dcp_fb_reference *entry = apple_state->retirement;

	if (!entry)
		return;

	apple_state->retirement = NULL;
	drm_framebuffer_put(entry->fb);
	kfree(entry);
}

static int apple_plane_prepare_fb(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	struct drm_plane_state *old;
	struct dcp_fb_reference *entry;
	int ret;

	ret = drm_gem_plane_helper_prepare_fb(plane, state);
	if (ret)
		return ret;

	old = drm_atomic_get_old_plane_state(state->state, plane);
	if (!old || !old->crtc || !old->fb || old->fb == state->fb)
		return 0;

	entry = kzalloc_obj(*entry);
	if (!entry)
		return -ENOMEM;

	drm_framebuffer_get(old->fb);
	entry->fb = old->fb;
	to_apple_plane_state(state)->retirement = entry;
	return 0;
}

static const struct drm_plane_helper_funcs apple_primary_plane_helper_funcs = {
	.prepare_fb	= apple_plane_prepare_fb,
	.cleanup_fb	= apple_plane_cleanup_fb,
	.atomic_check	= apple_plane_atomic_check,
	.atomic_update	= apple_plane_atomic_update,
	.get_scanout_buffer = drm_fb_dma_get_scanout_buffer,
};

static const struct drm_plane_helper_funcs apple_plane_helper_funcs = {
	.prepare_fb	= apple_plane_prepare_fb,
	.cleanup_fb	= apple_plane_cleanup_fb,
	.atomic_check	= apple_plane_atomic_check,
	.atomic_update	= apple_plane_atomic_update,
};

// Duplicate drm_atomic_helper_plane_reset but allocate struct apple_plane_state
static void apple_plane_reset(struct drm_plane *plane)
{
        struct apple_plane_state *state = to_apple_plane_state(plane->state);
	if (state) {
		apple_plane_cleanup_fb(plane, &state->base);
		__drm_atomic_helper_plane_destroy_state(&state->base);
	}

	kfree(state);
	plane->state = NULL;
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state)
		__drm_atomic_helper_plane_reset(plane, &state->base);
}

static struct drm_plane_state *
apple_plane_duplicate_state(struct drm_plane *plane)
{
        struct apple_plane_state *apple_plane_state, *old_apple_plane_state;

	if (!plane->state)
		return NULL;

        old_apple_plane_state = to_apple_plane_state(plane->state);
        apple_plane_state = kzalloc(sizeof(*apple_plane_state), GFP_KERNEL);
        if (!apple_plane_state)
                return NULL;

        __drm_atomic_helper_plane_duplicate_state(plane, &apple_plane_state->base);

	apple_plane_state->surf = old_apple_plane_state->surf;
	apple_plane_state->src_rect = old_apple_plane_state->src_rect;
	apple_plane_state->dst_rect = old_apple_plane_state->dst_rect;
	apple_plane_state->iova = old_apple_plane_state->iova;

	return &apple_plane_state->base;
}

static void apple_plane_destroy_state(struct drm_plane *plane,
				      struct drm_plane_state *state)
{
	apple_plane_cleanup_fb(plane, state);
	drm_atomic_helper_plane_destroy_state(plane, state);
}

static const struct drm_plane_funcs apple_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.reset			= apple_plane_reset,
	.atomic_duplicate_state = apple_plane_duplicate_state,
	.atomic_destroy_state	= apple_plane_destroy_state,
	.format_mod_supported = apple_plane_format_mod_supported,
};

/*
 * Table of supported formats, mapping from DRM fourccs to DCP fourccs.
 *
 * For future work, DCP supports more formats not listed, including YUV
 * formats, an extra RGBA format, and a biplanar RGB10_A8 format (fourcc b3a8)
 * used for HDR.
 *
 * Note: we don't have non-alpha formats but userspace breaks without XRGB. It
 * doesn't matter for the primary plane, but cursors/overlays must not
 * advertise formats without alpha.
 */
static const u32 dcp_primary_formats[] = {
	DRM_FORMAT_ABGR16161616F,
	DRM_FORMAT_XRGB2101010,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
	DRM_FORMAT_P010,
	DRM_FORMAT_P210,
#if defined(DRM_FORMAT_P410)
	DRM_FORMAT_P410,
#endif
	DRM_FORMAT_P030,
	DRM_FORMAT_P230,
#if defined(DRM_FORMAT_P430)
	DRM_FORMAT_P430,
#endif
};

/* The H17P alpha semantics of 10-bit XRGB are unknown. */
static const u32 dcp_primary_formats_h17p[] = {
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
	DRM_FORMAT_P010,
	DRM_FORMAT_P210,
#if defined(DRM_FORMAT_P410)
	DRM_FORMAT_P410,
#endif
};

static const u32 dcp_overlay_formats[] = {
	DRM_FORMAT_ABGR16161616F,
	DRM_FORMAT_ARGB2101010,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
	DRM_FORMAT_P010,
	DRM_FORMAT_P210,
#if defined(DRM_FORMAT_P410)
	DRM_FORMAT_P410,
#endif
	DRM_FORMAT_P030,
	DRM_FORMAT_P230,
#if defined(DRM_FORMAT_P430)
	DRM_FORMAT_P430,
#endif
};

/*
 * Formats for the 12.x firmware which does not support "l10r" / ARGB2101010
 */
static const u32 dcp_primary_formats_12_x[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
	DRM_FORMAT_P010,
	DRM_FORMAT_P210,
#if defined(DRM_FORMAT_P410)
	DRM_FORMAT_P410,
#endif
};

static const u32 dcp_overlay_formats_12_x[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
	DRM_FORMAT_P010,
	DRM_FORMAT_P210,
#if defined(DRM_FORMAT_P410)
	DRM_FORMAT_P410,
#endif
};

static u64 format_modifiers[] = {
	DRM_FORMAT_MOD_APPLE_INTERCHANGE,
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static u64 format_modifiers_no_ix[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

struct drm_plane *apple_plane_init(struct drm_device *dev,
				   unsigned long possible_crtcs,
				   u32 iomfb_surf,
				   bool supports_l10r,
				   bool supports_xrgb2101010,
				   enum drm_plane_type type,
				   bool interchange)
{
	struct apple_plane *plane;
	const u32 *fmts;
	u32 num_fmts;

	switch (type) {
	case DRM_PLANE_TYPE_PRIMARY:
		if (supports_l10r && supports_xrgb2101010) {
			fmts = dcp_primary_formats;
			num_fmts = ARRAY_SIZE(dcp_primary_formats);
		} else if (supports_l10r) {
			fmts = dcp_primary_formats_h17p;
			num_fmts = ARRAY_SIZE(dcp_primary_formats_h17p);
		} else {
			fmts = dcp_primary_formats_12_x;
			num_fmts = ARRAY_SIZE(dcp_primary_formats_12_x);
		}
		plane = drmm_universal_plane_alloc(dev, struct apple_plane, base, possible_crtcs,
				       &apple_plane_funcs, fmts, num_fmts,
				       interchange ? format_modifiers : format_modifiers_no_ix,
				       type, NULL);
		break;
	case DRM_PLANE_TYPE_OVERLAY:
	case DRM_PLANE_TYPE_CURSOR:
		if (supports_l10r) {
			fmts = dcp_overlay_formats;
			num_fmts = ARRAY_SIZE(dcp_overlay_formats);
		} else {
			fmts = dcp_overlay_formats_12_x;
			num_fmts = ARRAY_SIZE(dcp_overlay_formats_12_x);
		}
		plane = drmm_universal_plane_alloc(dev, struct apple_plane, base, possible_crtcs,
				       &apple_plane_funcs, fmts, num_fmts,
				       interchange ? format_modifiers : format_modifiers_no_ix,
				       type, NULL);
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	if (IS_ERR(plane))
		return ERR_PTR(PTR_ERR(plane));

	drm_plane_create_color_properties(&plane->base,
					  (1 << DRM_COLOR_ENCODING_MAX) - 1,
					  (1 << DRM_COLOR_RANGE_MAX) - 1,
					  DRM_COLOR_YCBCR_BT709,
					  DRM_COLOR_YCBCR_LIMITED_RANGE);

	if (type == DRM_PLANE_TYPE_PRIMARY)
		drm_plane_helper_add(&plane->base, &apple_primary_plane_helper_funcs);
	else
		drm_plane_helper_add(&plane->base, &apple_plane_helper_funcs);

	plane->iomfb_surf = iomfb_surf;

	return &plane->base;
}

/* The formats the T6030 (14.x firmware) scanout was qualified with. */
static const u32 dcp_primary_formats_14_7[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

struct drm_plane *apple_plane_init_v14_7(struct drm_device *dev,
					 unsigned long possible_crtcs,
					 u32 iomfb_surf,
					 enum drm_plane_type type)
{
	struct apple_plane *plane;

	if (type != DRM_PLANE_TYPE_PRIMARY)
		return ERR_PTR(-EINVAL);

	plane = drmm_universal_plane_alloc(dev, struct apple_plane, base, possible_crtcs,
					   &apple_plane_funcs, dcp_primary_formats_14_7,
					   ARRAY_SIZE(dcp_primary_formats_14_7),
					   format_modifiers_no_ix, type, NULL);
	if (IS_ERR(plane))
		return ERR_CAST(plane);

	drm_plane_helper_add(&plane->base, &apple_primary_plane_helper_funcs);
	plane->iomfb_surf = iomfb_surf;

	return &plane->base;
}
