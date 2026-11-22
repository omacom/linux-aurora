/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Just enough of KUnit to build dcp-indicator-test.c as a plain program
 * (-DDCP_IND_USERSPACE), for a host with no kernel tree to run the suite in.
 */
#ifndef DCP_KUNIT_SHIM_H
#define DCP_KUNIT_SHIM_H

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* The fields of the DRM structs that dcp_ind_plane_check() reads. */
struct drm_crtc;
struct drm_framebuffer_funcs { int unused; };
struct drm_framebuffer { const struct drm_framebuffer_funcs *funcs; };
struct drm_plane_state {
	struct drm_framebuffer *fb;
	struct drm_crtc *crtc;
	int crtc_x, crtc_y;
	unsigned int crtc_w, crtc_h;
	uint32_t src_x, src_y, src_w, src_h;
	uint16_t alpha;
	uint16_t pixel_blend_mode;
	unsigned int rotation;
};
struct drm_crtc_state { bool active; };

struct kunit { int fail; };
struct kunit_case { void (*run)(struct kunit *test); const char *name; };
struct kunit_suite { const char *name; struct kunit_case *test_cases; };

#define KUNIT_CASE(f) { f, #f }
#define KUNIT_EXPECT_EQ(t, a, b) do {					\
	if ((long)(a) != (long)(b)) {					\
		printf("FAIL %s:%d\n", __FILE__, __LINE__);		\
		(t)->fail = 1;						\
	}								\
} while (0)
#define KUNIT_ASSERT_EQ(t, a, b) KUNIT_EXPECT_EQ(t, a, b)
#define KUNIT_EXPECT_TRUE(t, a) KUNIT_EXPECT_EQ(t, !!(a), 1)
#define KUNIT_EXPECT_FALSE(t, a) KUNIT_EXPECT_EQ(t, !!(a), 0)
#define KUNIT_SHIM_MAIN(s) int main(void)				\
{									\
	int bad = 0;							\
	for (struct kunit_case *c = (s).test_cases; c->run; c++) {	\
		struct kunit t = { 0 };					\
		c->run(&t);						\
		printf("%s %s\n", t.fail ? "not ok" : "ok", c->name);	\
		bad |= t.fail;						\
	}								\
	return bad;							\
}

#endif /* DCP_KUNIT_SHIM_H */
