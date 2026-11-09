// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Which primary planes the 14.7 presenter takes: one unscaled image of the
 * mode's size from the top-left of the framebuffer, judged on the rectangles
 * drm_atomic_helper_check_plane_state() clips to the mode, with the scaling
 * limits apple_plane_atomic_check() passes it. A plane that reaches past the
 * right or bottom edge of the mode is clipped there and taken; one that is
 * offset into the mode, starts left of or above it, or is scaled is not. For
 * a plane it takes, check the size iomfb_v14_7_flush() hands the presenter.
 */
#include <kunit/test.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_crtc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_modes.h>
#include <drm/drm_plane.h>
#include <drm/drm_rect.h>

#include "iomfb_v14_7_plane.h"

struct v14_plane_case {
	const char *name;
	u16 hdisplay, vdisplay;		/* the mode */
	u32 fb_w, fb_h;
	int crtc_x, crtc_y;
	u32 crtc_w, crtc_h;
	u32 src_x, src_y, src_w, src_h;	/* whole pixels */
	bool external;
	bool fits;
	u32 shown_w, shown_h;		/* clipped source, when it fits */
	u32 src_w_frac;			/* 16.16 fraction added to src_w */
};

static const struct v14_plane_case v14_plane_cases[] = {
	{ "external, exact", 1920, 1080, 1920, 1080, 0, 0, 1920, 1080,
	  0, 0, 1920, 1080, true, true, 1920, 1080 },
	/*
	 * KWin mirroring a 3024x1964 panel at scale 1.7 onto a 1920x1080
	 * display: 1663 columns of picture and 129 of border on each side.
	 */
	{ "external, one column past the right edge", 1920, 1080, 1921, 1080,
	  0, 0, 1921, 1080, 0, 0, 1921, 1080, true, true, 1920, 1080 },
	{ "external, one row past the bottom edge", 1920, 1080, 1920, 1081,
	  0, 0, 1920, 1081, 0, 0, 1920, 1081, true, true, 1920, 1080 },
	{ "external, past the right and bottom edges", 1920, 1080, 1921, 1081,
	  0, 0, 1921, 1081, 0, 0, 1921, 1081, true, true, 1920, 1080 },
	/*
	 * Clipping rounds the source toward 1:1: 1921 + 1/65536 columns on
	 * 1921 clip to exactly 1920 << 16, while 1921.5 on 1921 clip to
	 * (1920 << 16) + 32750, which is scaled.
	 */
	{ "external, past the edge, source a 65536th over 1:1", 1920, 1080,
	  1922, 1080, 0, 0, 1921, 1080, 0, 0, 1921, 1080, true, true,
	  1920, 1080, .src_w_frac = 1 },
	{ "external, past the edge, source half a pixel over 1:1", 1920, 1080,
	  1922, 1080, 0, 0, 1921, 1080, 0, 0, 1921, 1080, true, false,
	  .src_w_frac = 0x8000 },
	{ "external, top-left of a larger framebuffer", 1920, 1080, 2560, 1440,
	  0, 0, 1920, 1080, 0, 0, 1920, 1080, true, true, 1920, 1080 },
	{ "external, starts left of the mode", 1920, 1080, 1921, 1080,
	  -1, 0, 1921, 1080, 0, 0, 1921, 1080, true, false },
	{ "external, starts above the mode", 1920, 1080, 1920, 1081,
	  0, -1, 1920, 1081, 0, 0, 1920, 1081, true, false },
	{ "external, offset into the framebuffer", 1920, 1080, 1921, 1080,
	  0, 0, 1920, 1080, 1, 0, 1920, 1080, true, false },
	{ "external, scaled to the mode", 1920, 1080, 1921, 1080,
	  0, 0, 1920, 1080, 0, 0, 1921, 1080, true, false },
	{ "external, narrower than the mode", 1920, 1080, 1919, 1080,
	  0, 0, 1919, 1080, 0, 0, 1919, 1080, true, false },
	{ "external, positioned inside the mode", 1920, 1080, 1920, 1080,
	  1, 0, 1919, 1080, 0, 0, 1919, 1080, true, false },
	{ "panel, exact", 3024, 1964, 3024, 1964, 0, 0, 3024, 1964,
	  0, 0, 3024, 1964, false, true, 3024, 1964 },
	{ "panel, larger framebuffer", 3024, 1964, 3025, 1964, 0, 0, 3025, 1964,
	  0, 0, 3025, 1964, false, false },
};

static void v14_plane_case_desc(const struct v14_plane_case *c, char *desc)
{
	strscpy(desc, c->name, KUNIT_PARAM_DESC_SIZE);
}

KUNIT_ARRAY_PARAM(v14_plane, v14_plane_cases, v14_plane_case_desc);

static void v14_plane_fits_mode(struct kunit *test)
{
	const struct v14_plane_case *c = test->param_value;
	struct drm_plane *plane = kunit_kzalloc(test, sizeof(*plane), GFP_KERNEL);
	struct drm_crtc *crtc = kunit_kzalloc(test, sizeof(*crtc), GFP_KERNEL);
	struct drm_framebuffer *fb = kunit_kzalloc(test, sizeof(*fb), GFP_KERNEL);
	struct drm_crtc_state *crtc_state =
		kunit_kzalloc(test, sizeof(*crtc_state), GFP_KERNEL);
	struct drm_plane_state p;
	u32 shown_w, shown_h;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, plane);
	KUNIT_ASSERT_NOT_NULL(test, crtc);
	KUNIT_ASSERT_NOT_NULL(test, fb);
	KUNIT_ASSERT_NOT_NULL(test, crtc_state);
	fb->width = c->fb_w;
	fb->height = c->fb_h;
	crtc_state->crtc = crtc;
	crtc_state->enable = true;
	crtc_state->mode.hdisplay = c->hdisplay;
	crtc_state->mode.vdisplay = c->vdisplay;
	p = (struct drm_plane_state) {
		.plane = plane, .crtc = crtc, .fb = fb,
		.rotation = DRM_MODE_ROTATE_0,
		.crtc_x = c->crtc_x, .crtc_y = c->crtc_y,
		.crtc_w = c->crtc_w, .crtc_h = c->crtc_h,
		.src_x = c->src_x << 16, .src_y = c->src_y << 16,
		.src_w = (c->src_w << 16) + c->src_w_frac,
		.src_h = c->src_h << 16,
	};
	/* As apple_plane_atomic_check() runs it, before the CRTC is checked. */
	ret = drm_atomic_helper_check_plane_state(&p, crtc_state,
						  DRM_PLANE_NO_SCALING / 2,
						  DRM_PLANE_NO_SCALING * 4,
						  true, true);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_TRUE(test, p.visible);

	KUNIT_EXPECT_EQ(test, dcp_v14_plane_fits_mode(&p, c->hdisplay, c->vdisplay,
						      c->external), c->fits);
	if (!c->fits)
		return;
	/* What iomfb_v14_7_flush() hands the presenter. */
	dcp_v14_plane_scanout_size(&p, c->external, &shown_w, &shown_h);
	KUNIT_EXPECT_EQ(test, shown_w, c->shown_w);
	KUNIT_EXPECT_EQ(test, shown_h, c->shown_h);
}

static struct kunit_case v14_plane_test_cases[] = {
	KUNIT_CASE_PARAM(v14_plane_fits_mode, v14_plane_gen_params),
	{}
};

static struct kunit_suite v14_plane_test_suite = {
	.name = "apple-dcp-v14-plane",
	.test_cases = v14_plane_test_cases,
};

kunit_test_suite(v14_plane_test_suite);
