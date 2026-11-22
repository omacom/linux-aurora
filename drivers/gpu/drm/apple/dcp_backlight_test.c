// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2026 Ryan Murray */

#include <kunit/test.h>
#include <linux/module.h>

#include "dcp_backlight.h"
#include "dcp-edr.h"

static void backlight_millinits_takeover_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	u32 nits = 123;

	KUNIT_EXPECT_EQ(test, dcp_bl_takeover_nits(0, 100000, &nits), -EINVAL);
	KUNIT_EXPECT_EQ(test, dcp_bl_takeover_nits(U32_MAX, 100000, &nits), -EINVAL);
	KUNIT_EXPECT_EQ(test, nits, 123U);
	/* Reports above the panel ceiling clamp to it. */
	KUNIT_ASSERT_EQ(test, dcp_bl_takeover_nits(525, 525999, &nits), 0);
	KUNIT_EXPECT_EQ(test, nits, 525U);
	KUNIT_ASSERT_EQ(test, dcp_bl_takeover_nits(509, 509001, &nits), 0);
	KUNIT_EXPECT_EQ(test, nits, 509U);
	KUNIT_ASSERT_EQ(test, dcp_bl_takeover_nits(509, 1018999, &nits), 0);
	KUNIT_EXPECT_EQ(test, nits, 509U);
	/* More than twice the ceiling is not a panel level. */
	nits = 123;
	KUNIT_EXPECT_EQ(test, dcp_bl_takeover_nits(509, 1019000, &nits), -ERANGE);
	KUNIT_EXPECT_EQ(test, dcp_bl_takeover_nits(509, U32_MAX, &nits), -ERANGE);
	KUNIT_EXPECT_EQ(test, nits, 123U);
	KUNIT_ASSERT_EQ(test, dcp_bl_takeover_nits(509, 100999, &nits), 0);
	KUNIT_EXPECT_EQ(test, nits, 100U);
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 509, true, nits, false, 0), 0);
	KUNIT_EXPECT_EQ(test, state.actual, 100U);
	KUNIT_EXPECT_FALSE(test, state.dirty);
	/* Registration freezes the hint before exposing the controls. */
	state.controlled = true;
	KUNIT_EXPECT_FALSE(test, dcp_bl_seed(&state, 400));
	KUNIT_EXPECT_FALSE(test, dcp_bl_seed(&state, 0));
	KUNIT_EXPECT_EQ(test, state.target, 100U);
	KUNIT_ASSERT_EQ(test, dcp_bl_takeover_nits(509, 0, &nits), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 509, true, nits, false, 0), 0);
	KUNIT_EXPECT_EQ(test, state.target, 0U);
	KUNIT_EXPECT_FALSE(test, state.dirty);
	KUNIT_ASSERT_EQ(test, dcp_bl_takeover_nits(509, 509000, &nits), 0);
	KUNIT_EXPECT_EQ(test, nits, 509U);
}

static void backlight_takeover_test(struct kunit *test)
{
	struct dcp_backlight_state state = {};

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 140, true, 36), 0);
	KUNIT_EXPECT_EQ(test, state.target, 140U);
	KUNIT_EXPECT_EQ(test, state.actual, 140U);
	KUNIT_EXPECT_FALSE(test, state.dirty);
	KUNIT_EXPECT_TRUE(test, dcp_bl_seed(&state, 400));
	KUNIT_EXPECT_EQ(test, state.target, 400U);
	KUNIT_EXPECT_FALSE(test, dcp_bl_seed(&state, 601));
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 400, false, false), 0);
	KUNIT_EXPECT_FALSE(test, dcp_bl_seed(&state, 0));
	KUNIT_EXPECT_EQ(test, state.target, 400U);
	KUNIT_EXPECT_EQ(test, state.actual, 400U);
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 0, true, 36), 0);
	KUNIT_EXPECT_EQ(test, state.target, 0U);
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, false, 0, true, 36), 0);
	KUNIT_EXPECT_EQ(test, state.target, 36U);
}

static void backlight_range_test(struct kunit *test)
{
	struct dcp_backlight_state state = {};

	KUNIT_EXPECT_EQ(test, dcp_bl_init(&state, 600, false, 0, false, 0), -ENODATA);
	KUNIT_EXPECT_FALSE(test, state.ready);
	KUNIT_EXPECT_EQ(test, dcp_bl_request(&state, 140, false, false), -ENODATA);
	KUNIT_EXPECT_EQ(test, dcp_bl_dpms(&state, false), -ENODATA);
	KUNIT_EXPECT_EQ(test, dcp_bl_init(&state, 0, true, 0, false, 0), -EINVAL);
	KUNIT_EXPECT_EQ(test, dcp_bl_init(&state, U32_MAX, true, 0, false, 0), -EINVAL);
	KUNIT_EXPECT_EQ(test, dcp_bl_init(&state, 600, true, 601, false, 0), -ERANGE);
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 140, false, 0), 0);
	KUNIT_EXPECT_EQ(test, dcp_bl_request(&state, 601, false, false), -ERANGE);
	KUNIT_EXPECT_EQ(test, state.target, 140U);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 600, false, false), 0);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 600U);
}

static void backlight_dpms_cycles_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;
	int i;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 400, false, 0), 0);
	for (i = 0; i < 20; i++) {
		KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
		KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
		KUNIT_EXPECT_EQ(test, present.nits, 0U);
		KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
		KUNIT_EXPECT_EQ(test, state.actual, 0U);
		KUNIT_EXPECT_EQ(test, state.target, 400U);
		KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, true), 0);
		KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
		KUNIT_EXPECT_EQ(test, present.nits, 400U);
		KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
		KUNIT_EXPECT_EQ(test, state.actual, 400U);
	}
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 0U);
	KUNIT_EXPECT_EQ(test, state.target, 400U);
}

static void backlight_blanked_target_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 400, false, 0), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 140, false, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 0U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_EQ(test, state.target, 140U);
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, true), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 140U);
	KUNIT_EXPECT_EQ(test, state.actual, 0U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_EQ(test, state.actual, 140U);
}

static void backlight_suspend_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	int off_first;

	for (off_first = 0; off_first < 2; off_first++) {
		KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 140, false, 0), 0);
		if (off_first)
			KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
		KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 140, true, true), 0);
		if (!off_first)
			KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
		KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 36, true, true), 0);
		KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 0U);
		KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, true), 0);
		KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 0U);
		KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
		KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 36, false, false), 0);
		KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 0U);
		KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, true), 0);
		KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 36U);
		KUNIT_EXPECT_EQ(test, state.target, 36U);
	}
	/* A core blank also keeps the panel dark independently of DRM DPMS. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 400, true, false), 0);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 0U);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 400, false, false), 0);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 400U);
}

static void backlight_surface_and_busy_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present first, second;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 140, false, 0), 0);
	KUNIT_EXPECT_EQ(test, dcp_bl_prepare(&state, true, &first), -EALREADY);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 36, false, false), 0);
	KUNIT_EXPECT_EQ(test, dcp_bl_prepare(&state, false, &first), -ENODATA);
	KUNIT_EXPECT_TRUE(test, state.dirty);
	KUNIT_EXPECT_FALSE(test, state.in_flight);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &first), 0);
	KUNIT_EXPECT_EQ(test, dcp_bl_prepare(&state, true, &second), -EBUSY);
	KUNIT_EXPECT_EQ(test, state.actual, 140U);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 400, false, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, first.sequence, true));
	KUNIT_EXPECT_EQ(test, state.actual, 36U);
	KUNIT_EXPECT_EQ(test, state.target, 400U);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &second), 0);
	KUNIT_EXPECT_EQ(test, second.nits, 400U);
	KUNIT_EXPECT_FALSE(test, dcp_bl_complete(&state, first.sequence, true));
	KUNIT_EXPECT_TRUE(test, state.in_flight);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, second.sequence, true));
	KUNIT_EXPECT_EQ(test, state.actual, 400U);
}

static void backlight_reject_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 140, false, 0), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 400, false, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, false));
	KUNIT_EXPECT_EQ(test, state.actual, 140U);
	KUNIT_EXPECT_TRUE(test, state.dirty);
	KUNIT_EXPECT_FALSE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 0U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_EQ(test, state.target, 400U);
	KUNIT_EXPECT_EQ(test, state.actual, 0U);
}

static void backlight_inflight_dpms_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 140, false, 0), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, true), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_TRUE(test, state.dirty);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 0U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, true), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 140U);
}

static void backlight_resend_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	/* Nothing is re-sent before Linux controls the level. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 525, true, 140, false, 0), 0);
	KUNIT_EXPECT_FALSE(test, dcp_bl_resend(&state));
	KUNIT_EXPECT_FALSE(test, state.dirty);

	state.controlled = true;
	KUNIT_EXPECT_TRUE(test, dcp_bl_resend(&state));
	KUNIT_EXPECT_FALSE(test, dcp_bl_resend(&state));
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 140U);
	/* The present in flight already carries the level. */
	KUNIT_EXPECT_FALSE(test, dcp_bl_resend(&state));
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_FALSE(test, state.dirty);

	/* A blanked panel is re-sent dark, keeping the restore target. */
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_TRUE(test, dcp_bl_resend(&state));
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 0U);
	KUNIT_EXPECT_EQ(test, state.target, 140U);
}

static void backlight_fallback_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	/* A registration default is reported, never presented on its own. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 525, false, 0, true, 262), 0);
	KUNIT_EXPECT_FALSE(test, state.level_known);
	state.controlled = true;
	KUNIT_EXPECT_FALSE(test, dcp_bl_resend(&state));
	KUNIT_EXPECT_FALSE(test, state.dirty);
	KUNIT_EXPECT_EQ(test, dcp_bl_prepare(&state, true, &present), -EALREADY);

	/* A rejected userspace level does not become known. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 300, false, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 300U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, false));
	KUNIT_EXPECT_FALSE(test, state.level_known);

	/* Once a present carrying it completes, the level is re-sendable. */
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_TRUE(test, state.level_known);
	KUNIT_EXPECT_TRUE(test, dcp_bl_resend(&state));

	/* DPMS has to light the panel at some level: wake uses the default. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 525, false, 0, true, 262), 0);
	state.controlled = true;
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 0U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, true), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 262U);
}

static void backlight_unknown_takeover_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, false, 0, true, 300), 0);
	state.preserve_inherited = true;
	KUNIT_EXPECT_EQ(test, dcp_bl_actual(&state), 300);
	/* Core suspend/blank and resume do not turn the guess into a write. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 300, true, true), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 300, false, false), 0);
	KUNIT_EXPECT_FALSE(test, state.controlled);
	KUNIT_EXPECT_FALSE(test, state.dirty);
	KUNIT_EXPECT_EQ(test, dcp_bl_prepare(&state, true, &present), -EALREADY);
	KUNIT_EXPECT_EQ(test, dcp_bl_actual(&state), 300);

	/* A late loader hint supplies the inherited level without a present. */
	KUNIT_EXPECT_TRUE(test, dcp_bl_seed(&state, 250));
	KUNIT_EXPECT_EQ(test, dcp_bl_actual(&state), 250);
	KUNIT_EXPECT_FALSE(test, state.dirty);

	/* Echoing the unknown registration value leaves the panel alone. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, false, 0, true, 300), 0);
	state.preserve_inherited = true;
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 300, false, false), 0);
	KUNIT_EXPECT_FALSE(test, state.controlled);
	KUNIT_EXPECT_EQ(test, dcp_bl_prepare(&state, true, &present), -EALREADY);
	/* A new target is reported even before its present completes. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 250, false, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, false));
	KUNIT_EXPECT_EQ(test, dcp_bl_actual(&state), 250);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_EQ(test, dcp_bl_actual(&state), 250);
}

static void backlight_first_change_fades_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, false, 0, true, 300), 0);
	state.preserve_inherited = true;
	state.fade = true;
	KUNIT_EXPECT_EQ(test, dcp_bl_actual(&state), 300);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 300, false, false), 0);
	KUNIT_EXPECT_FALSE(test, state.dirty);
	KUNIT_EXPECT_FALSE(test, state.controlled);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 100, false, false), 0);
	KUNIT_EXPECT_TRUE(test, state.fading);
	KUNIT_EXPECT_FALSE(test, state.level_known);
	KUNIT_EXPECT_FALSE(test, state.dirty);
	KUNIT_EXPECT_EQ(test, state.fade_from, 300U);
	KUNIT_EXPECT_EQ(test, dcp_bl_actual(&state), 100);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 125, 250));
	KUNIT_EXPECT_EQ(test, state.level, 200U);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_EQ(test, dcp_bl_actual(&state), 200);
	KUNIT_EXPECT_FALSE(test, dcp_bl_fade_step(&state, 250, 250));
	KUNIT_EXPECT_EQ(test, state.level, 100U);
}

static void backlight_fallback_same_level_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	/* A userspace write of the reported default is still presented. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 525, false, 0, true, 262), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 262, false, false), 0);
	KUNIT_EXPECT_TRUE(test, state.dirty);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 262U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_TRUE(test, state.level_known);

	/* Once the level is known, repeating it presents nothing. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 262, false, false), 0);
	KUNIT_EXPECT_FALSE(test, state.dirty);
	KUNIT_EXPECT_EQ(test, dcp_bl_prepare(&state, true, &present), -EALREADY);

	/* A rejected present leaves it unknown, so the same write is retried. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 525, false, 0, true, 262), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 262, false, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, false));
	KUNIT_EXPECT_FALSE(test, state.level_known);
	state.dirty = false;
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 262, false, false), 0);
	KUNIT_EXPECT_TRUE(test, state.dirty);

	/* An inherited level is known from the start. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 525, true, 180, false, 0), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 180, false, false), 0);
	KUNIT_EXPECT_FALSE(test, state.dirty);
}

static void backlight_seed_known_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	/* A loader sample seeded over a registration default is known. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 525, false, 0, true, 262), 0);
	KUNIT_EXPECT_FALSE(test, state.level_known);
	KUNIT_EXPECT_TRUE(test, dcp_bl_seed(&state, 180));
	KUNIT_EXPECT_TRUE(test, state.level_known);
	state.controlled = true;
	KUNIT_EXPECT_TRUE(test, dcp_bl_resend(&state));
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 180U);

	/* A refused sample leaves the default unknown. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 525, false, 0, true, 262), 0);
	KUNIT_EXPECT_FALSE(test, dcp_bl_seed(&state, 526));
	KUNIT_EXPECT_FALSE(test, state.level_known);
	state.controlled = true;
	KUNIT_EXPECT_FALSE(test, dcp_bl_seed(&state, 180));
	KUNIT_EXPECT_FALSE(test, state.level_known);
	KUNIT_EXPECT_FALSE(test, dcp_bl_resend(&state));
}

static void backlight_fade_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 100, false, 0), 0);
	state.fade = true;
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 400, false, false), 0);
	KUNIT_EXPECT_TRUE(test, state.fading);
	KUNIT_EXPECT_FALSE(test, state.dirty);
	KUNIT_EXPECT_EQ(test, state.target, 400U);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 100U);

	/* The start of the fade is the level already shown. */
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 0, 250));
	KUNIT_EXPECT_FALSE(test, state.dirty);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 125, 250));
	KUNIT_EXPECT_TRUE(test, state.dirty);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 250U);

	/* Steps during a present merge into the next one. */
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 150, 250));
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 200, 250));
	KUNIT_EXPECT_EQ(test, dcp_bl_prepare(&state, true, &present), -EBUSY);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_EQ(test, state.actual, 250U);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 340U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));

	/* The last step is exactly the target, however late it runs. */
	KUNIT_EXPECT_FALSE(test, dcp_bl_fade_step(&state, 1000, 250));
	KUNIT_EXPECT_FALSE(test, state.fading);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 400U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_EQ(test, state.actual, 400U);
	KUNIT_EXPECT_FALSE(test, dcp_bl_fade_step(&state, 250, 250));
	KUNIT_EXPECT_FALSE(test, state.dirty);

	/* Fading down reaches the target too, including zero. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 0, false, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 249, 250));
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 2U);
	KUNIT_EXPECT_FALSE(test, dcp_bl_fade_step(&state, 250, 250));
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 0U);
}

static void backlight_fade_restart_test(struct kunit *test)
{
	struct dcp_backlight_state state;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 100, false, 0), 0);
	state.fade = true;
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 500, false, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 100, 250));
	KUNIT_EXPECT_EQ(test, state.level, 260U);

	/* Repeating the target keeps the running fade. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 500, false, false), 0);
	KUNIT_EXPECT_TRUE(test, state.fading);
	KUNIT_EXPECT_EQ(test, state.fade_from, 100U);

	/* A new level replaces it, starting from the level presented last. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 60, false, false), 0);
	KUNIT_EXPECT_TRUE(test, state.fading);
	KUNIT_EXPECT_EQ(test, state.fade_from, 260U);
	KUNIT_EXPECT_EQ(test, state.level, 260U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 125, 250));
	KUNIT_EXPECT_EQ(test, state.level, 160U);
	KUNIT_EXPECT_FALSE(test, dcp_bl_fade_step(&state, 250, 250));
	KUNIT_EXPECT_EQ(test, state.level, 60U);

	/* A new level equal to the one presented ends the fade there. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 300, false, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 125, 250));
	KUNIT_EXPECT_EQ(test, state.level, 180U);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 180, false, false), 0);
	KUNIT_EXPECT_FALSE(test, state.fading);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 180U);

	/* No overflow over the whole range a panel ceiling may have. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, INT_MAX, true, 0, false, 0), 0);
	state.fade = true;
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, INT_MAX, false, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 249, 250));
	KUNIT_EXPECT_EQ(test, state.level, (u32)div_u64((u64)INT_MAX * 249, 250));
	KUNIT_EXPECT_FALSE(test, dcp_bl_fade_step(&state, 250, 250));
	KUNIT_EXPECT_EQ(test, state.level, (u32)INT_MAX);
}

static void backlight_fade_blank_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	/* DPMS off and on during a fade jump, as without fades. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 100, false, 0), 0);
	state.fade = true;
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 400, false, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 125, 250));
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, false), 0);
	KUNIT_EXPECT_FALSE(test, state.fading);
	KUNIT_EXPECT_TRUE(test, state.dirty);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 0U);
	KUNIT_EXPECT_FALSE(test, dcp_bl_fade_step(&state, 200, 250));
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 0U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&state, true), 0);
	KUNIT_EXPECT_FALSE(test, state.fading);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 400U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));

	/* A level set while blanked is shown at once on unblank. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 400, true, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 200, true, false), 0);
	KUNIT_EXPECT_FALSE(test, state.fading);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 0U);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 200, false, false), 0);
	KUNIT_EXPECT_FALSE(test, state.fading);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 200U);

	/* Suspend during a fade blanks at once; resume shows the target. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 500, false, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 50, 250));
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 500, true, true), 0);
	KUNIT_EXPECT_FALSE(test, state.fading);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 0U);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 500, false, false), 0);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 500U);

	/* A level the panel may not show yet is presented directly. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 525, false, 0, true, 262), 0);
	state.fade = true;
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 100, false, false), 0);
	KUNIT_EXPECT_FALSE(test, state.fading);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_EQ(test, present.nits, 100U);

	/* Without fades a new level is presented directly. */
	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 100, false, 0), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 400, false, false), 0);
	KUNIT_EXPECT_FALSE(test, state.fading);
	KUNIT_EXPECT_EQ(test, dcp_bl_effective(&state), 400U);
}

static void backlight_desktop_swap_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_swap swap = {};

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, true, 300, false, 0), 0);
	state.fade = true;
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 600, false, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 125, 250));
	KUNIT_EXPECT_EQ(test, state.level, 450U);
	KUNIT_ASSERT_EQ(test, dcp_bl_swap_prepare(&state, &swap, 42, true), 0);
	/* A completion alone must not publish an unaccepted panel level. */
	KUNIT_EXPECT_FALSE(test, dcp_bl_swap_finish(&state, &swap, 42, false, true));
	KUNIT_EXPECT_EQ(test, state.actual, 300U);
	KUNIT_EXPECT_FALSE(test, dcp_bl_swap_finish(&state, &swap, 41, true, true));
	KUNIT_EXPECT_TRUE(test, dcp_bl_swap_finish(&state, &swap, 42, true, true));
	KUNIT_EXPECT_EQ(test, state.actual, 450U);
	KUNIT_EXPECT_FALSE(test, dcp_bl_swap_finish(&state, &swap, 42, true, false));
	KUNIT_EXPECT_FALSE(test, dcp_bl_fade_step(&state, 250, 250));
	KUNIT_ASSERT_EQ(test, dcp_bl_swap_prepare(&state, &swap, 43, true), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_swap_finish(&state, &swap, 43, true, false));
	KUNIT_EXPECT_EQ(test, state.actual, 450U);
	KUNIT_EXPECT_EQ(test, dcp_bl_retry_delay(&state), 100U);
	KUNIT_ASSERT_EQ(test, dcp_bl_swap_prepare(&state, &swap, 44, true), 0);
	KUNIT_EXPECT_FALSE(test, dcp_bl_swap_finish(&state, &swap, 44, true, true));
	KUNIT_EXPECT_TRUE(test, dcp_bl_swap_finish(&state, &swap, 44, false, true));
	KUNIT_EXPECT_EQ(test, state.actual, 600U);
	/* A lit zero is a valid request, with the same fade as the Neo. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 0, false, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 125, 250));
	KUNIT_EXPECT_EQ(test, state.level, 300U);
	KUNIT_EXPECT_FALSE(test, dcp_bl_fade_step(&state, 250, 250));
	KUNIT_EXPECT_EQ(test, state.level, 0U);
}

static void backlight_reports_keep_target_test(struct kunit *test)
{
	struct dcp_backlight_state state;
	struct dcp_backlight_present present;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&state, 600, false, 0, true, 300), 0);
	state.fade = true;
	KUNIT_EXPECT_TRUE(test, dcp_bl_report(&state, 100));
	KUNIT_EXPECT_EQ(test, state.actual, 100U);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 600, false, false), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 125, 250));
	KUNIT_EXPECT_FALSE(test, dcp_bl_report(&state, 50));
	KUNIT_EXPECT_EQ(test, state.target, 600U);
	KUNIT_EXPECT_EQ(test, state.level, 350U);
	KUNIT_EXPECT_EQ(test, state.actual, 100U);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_FALSE(test, dcp_bl_report(&state, 600));
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_EQ(test, state.actual, 350U);
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&state, 500, false, false), 0);
	KUNIT_EXPECT_FALSE(test, dcp_bl_report(&state, 100));
	KUNIT_EXPECT_TRUE(test, dcp_bl_fade_step(&state, 125, 250));
	KUNIT_EXPECT_EQ(test, state.level, 425U);
	KUNIT_EXPECT_FALSE(test, dcp_bl_fade_step(&state, 250, 250));
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&state, true, &present), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&state, present.sequence, true));
	KUNIT_EXPECT_FALSE(test, dcp_bl_report(&state, 200));
	KUNIT_EXPECT_EQ(test, state.reported, 200U);
	KUNIT_EXPECT_EQ(test, state.target, 500U);
	KUNIT_EXPECT_EQ(test, state.actual, 500U);
	KUNIT_EXPECT_FALSE(test, state.dirty);
}


/* Synthetic panel: a 1200 nit peak and a potential headroom of 8.0. */
#define EDR_TEST_PEAK		1200U
#define EDR_TEST_CEILING	0x80000U

static void edr_ceiling_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, dcp_edr_ceiling(0), 0U);
	KUNIT_EXPECT_EQ(test, dcp_edr_ceiling(0x8000), 0U);
	KUNIT_EXPECT_EQ(test, dcp_edr_ceiling(0x10000), 0U);
	KUNIT_EXPECT_EQ(test, dcp_edr_ceiling(0x18000), 1500U);
	KUNIT_EXPECT_EQ(test, dcp_edr_ceiling(EDR_TEST_CEILING), 8000U);
	KUNIT_EXPECT_EQ(test, dcp_edr_ceiling(U32_MAX), 65535999U);
}

static void edr_target_test(struct kunit *test)
{
	u32 ceiling = dcp_edr_ceiling(EDR_TEST_CEILING);

	/* Bright SDR white: the peak sets the headroom. */
	KUNIT_EXPECT_EQ(test, dcp_edr_target(EDR_TEST_PEAK, 400, ceiling), 3000U);
	KUNIT_EXPECT_EQ(test, dcp_edr_target(EDR_TEST_PEAK, 150, ceiling), 8000U);
	/* Dim SDR white: the ceiling does, never peak / sdr. */
	KUNIT_EXPECT_EQ(test, dcp_edr_target(EDR_TEST_PEAK, 6, ceiling), 8000U);
	KUNIT_EXPECT_EQ(test, dcp_edr_target(EDR_TEST_PEAK, 1, ceiling), 8000U);
	KUNIT_EXPECT_EQ(test, dcp_edr_target(EDR_TEST_PEAK, 6, 0), 200000U);
	/* No headroom above the peak, and none to give with no SDR white. */
	KUNIT_EXPECT_EQ(test, dcp_edr_target(EDR_TEST_PEAK, 1200, ceiling), 1000U);
	KUNIT_EXPECT_EQ(test, dcp_edr_target(EDR_TEST_PEAK, 1500, ceiling), 1000U);
	KUNIT_EXPECT_EQ(test, dcp_edr_target(EDR_TEST_PEAK, 0, ceiling), 1000U);
	KUNIT_EXPECT_EQ(test, dcp_edr_target(10000, 1, 0), 10000000U);
}

/*
 * Present a fade of SDR white from @from to @to in @steps swaps, as the
 * backlight fade does, then @extra swaps at the new level, and return the
 * number of swaps whose panel level moved against the fade.
 */
static unsigned int edr_fade(struct kunit *test, u32 *hm, u32 ceiling,
			     u32 from, u32 to, unsigned int steps,
			     unsigned int extra, u32 *last_bri)
{
	u32 prev = from, bri = (u64)from * *hm / DCP_EDR_UNITY;
	unsigned int reversals = 0;

	for (unsigned int i = 1; i <= steps + extra; i++) {
		u32 sdr = i >= steps ? to :
			(from > to ? from - (from - to) * i / steps :
				     from + (to - from) * i / steps);
		u32 target = dcp_edr_target(EDR_TEST_PEAK, sdr, ceiling);
		u32 next = dcp_edr_next(*hm, target, prev, sdr);
		u32 level = (u64)sdr * next / DCP_EDR_UNITY;

		KUNIT_EXPECT_LE(test, next, max(target, *hm));
		/* Allow the rounding of the headroom and of the level. */
		u32 slack = max(1U, bri / 200);

		if ((from > to && level > bri + slack) ||
		    (from < to && level + slack < bri))
			reversals++;
		*hm = next;
		prev = sdr;
		bri = level;
	}
	*last_bri = bri;
	return reversals;
}

static void edr_low_end_steady_test(struct kunit *test)
{
	u32 ceiling = dcp_edr_ceiling(EDR_TEST_CEILING);
	u32 hm = dcp_edr_target(EDR_TEST_PEAK, 35, ceiling), bri;

	/* At the ceiling below 150 nits: a fade only moves the panel level. */
	KUNIT_EXPECT_EQ(test, edr_fade(test, &hm, ceiling, 35, 6, 15, 0, &bri), 0U);
	KUNIT_EXPECT_EQ(test, hm, 8000U);
	KUNIT_EXPECT_EQ(test, bri, 48U);
	/* Swaps at the floor, e.g. for an OSD, change nothing. */
	for (unsigned int i = 0; i < 200; i++)
		KUNIT_EXPECT_EQ(test, dcp_edr_next(hm, dcp_edr_target(EDR_TEST_PEAK, 6, ceiling),
						   6, 6), hm);
}

static void edr_fade_monotonic_test(struct kunit *test)
{
	u32 ceilings[] = { dcp_edr_ceiling(EDR_TEST_CEILING), 0 };

	for (unsigned int c = 0; c < ARRAY_SIZE(ceilings); c++) {
		u32 ceiling = ceilings[c];
		u32 hm = dcp_edr_target(EDR_TEST_PEAK, 400, ceiling), bri;

		/* Down: the panel level holds, then follows SDR white down. */
		KUNIT_EXPECT_EQ(test, edr_fade(test, &hm, ceiling, 400, 6, 15, 300, &bri), 0U);
		KUNIT_EXPECT_EQ(test, hm, dcp_edr_target(EDR_TEST_PEAK, 6, ceiling));
		KUNIT_EXPECT_EQ(test, bri, (u32)((u64)6 * hm / DCP_EDR_UNITY));
		/* Up: the headroom drops at once and the level only rises. */
		KUNIT_EXPECT_EQ(test, edr_fade(test, &hm, ceiling, 6, 400, 15, 10, &bri), 0U);
		KUNIT_EXPECT_EQ(test, hm, 3000U);
		KUNIT_EXPECT_EQ(test, bri, EDR_TEST_PEAK);
		/* One step at a time between near levels. */
		KUNIT_EXPECT_EQ(test, edr_fade(test, &hm, ceiling, 400, 300, 15, 50, &bri), 0U);
		KUNIT_EXPECT_EQ(test, hm, 4000U);
	}
}

static void edr_start_ramp_test(struct kunit *test)
{
	u32 ceiling = dcp_edr_ceiling(EDR_TEST_CEILING);
	u32 target = dcp_edr_target(EDR_TEST_PEAK, 6, ceiling);
	u32 hm = DCP_EDR_UNITY;
	unsigned int swaps = 0;

	/* EDR starting at a steady level rises one step per swap. */
	while (hm < target) {
		u32 next = dcp_edr_next(hm, target, 6, 6);

		KUNIT_ASSERT_GT(test, next, hm);
		KUNIT_EXPECT_LE(test, next, hm * DCP_EDR_STEP_UP / DCP_EDR_UNITY + 1);
		hm = next;
		KUNIT_ASSERT_LT(test, ++swaps, 100U);
	}
	KUNIT_EXPECT_EQ(test, hm, target);
	/* Ending EDR returns to 1.0 in one swap. */
	KUNIT_EXPECT_EQ(test, dcp_edr_next(hm, DCP_EDR_UNITY, 6, 6), DCP_EDR_UNITY);
}

/* Swaps @us apart from @from to @to; returns their number, each step checked. */
static unsigned int edr_timed_ramp(struct kunit *test, u32 from, u32 to, u32 us,
				   u32 lo, u32 hi)
{
	u32 cap = max(from, to), hm = from;
	unsigned int swaps = 0;

	while (hm != to) {
		u32 next = dcp_edr_next_timed(hm, to, cap, 60, 60, us);
		u32 ratio = div_u64((u64)next * 100000, hm);

		/* Each step's ratio x100000, but for the last one. */
		if (next != to) {
			KUNIT_EXPECT_GE(test, ratio, lo);
			KUNIT_EXPECT_LE(test, ratio, hi);
		}
		hm = next;
		KUNIT_ASSERT_LT(test, ++swaps, 1000U);
	}
	return swaps;
}

static void edr_timed_ramp_test(struct kunit *test)
{
	/* macOS's ramp up: x1.0555 per 24 Hz frame, 1.2 to 16 in 2.0 s. */
	KUNIT_EXPECT_EQ(test, edr_timed_ramp(test, 1200, 16000, 41667, 105450, 105650), 49U);
	/* Down: x0.9893 per 120 Hz refresh, 16 to 1.2 in 2.0 s. */
	KUNIT_EXPECT_EQ(test, edr_timed_ramp(test, 16000, 1200, 8333, 98800, 98980), 237U);
	/* The rate is in time: 1.0 to 16 in about 2.15 s at any cadence. */
	KUNIT_EXPECT_EQ(test, edr_timed_ramp(test, 1000, 16000, 12000, 101400, 101700), 181U);
	KUNIT_EXPECT_EQ(test, edr_timed_ramp(test, 1000, 16000, DCP_EDR_HOLDOFF_US,
					     106600, 106700), 43U);
	/* No time yet: still one step. */
	KUNIT_EXPECT_EQ(test, dcp_edr_next_timed(1000, 16000, 16000, 60, 60, 0), 1001U);
	KUNIT_EXPECT_EQ(test, dcp_edr_next_timed(16000, 1000, 16000, 60, 60, 0), 15999U);
	KUNIT_EXPECT_EQ(test, dcp_edr_next_timed(16000, 16000, 16000, 60, 60, 8333), 16000U);
}

static void edr_timed_limits_test(struct kunit *test)
{
	/* Above what the peak leaves, at once: SDR white rose to 200 nits. */
	KUNIT_EXPECT_EQ(test, dcp_edr_next_timed(16000, 8000, 8000, 60, 200, 8333), 8000U);
	KUNIT_EXPECT_EQ(test, dcp_edr_next_timed(12000, 1000, 8000, 60, 200, 8333), 8000U);
	/* SDR white falling mid-ramp holds the panel level, up to the target. */
	KUNIT_EXPECT_EQ(test, dcp_edr_next_timed(4000, 16000, 16000, 60, 50, 8333), 4800U);
	KUNIT_EXPECT_EQ(test, dcp_edr_next_timed(15000, 16000, 16000, 60, 30, 8333), 16000U);
}

/*
 * DAC firmware after a boot with display power cycles (J414s, 13.5) and
 * after a system sleep.  The report goes to 0 with the power-off present
 * and no report follows the one that lights the panel again; both
 * sequences must end at the level the panel shows.
 */
static void backlight_presented_boot_resume_test(struct kunit *test)
{
	struct dcp_bl_presented boot = {}, resume = {};
	int reported;

	/* Fresh boot: only the loader's level is known, from its report. */
	reported = 140;
	KUNIT_EXPECT_EQ(test, dcp_bl_presented_actual(&boot, reported), 140);
	/* systemd-backlight restores 500, and a present carries it. */
	dcp_bl_presented_set(&boot, 500);
	KUNIT_EXPECT_EQ(test, dcp_bl_presented_actual(&boot, reported), 500);
	/* DPMS off: the clear swap takes the panel to 0, and the firmware says so. */
	dcp_bl_presented_set(&boot, 0);
	reported = 0;
	KUNIT_EXPECT_EQ(test, dcp_bl_presented_actual(&boot, reported), 0);
	/* DPMS on: the first present carries 500 again; no report follows. */
	dcp_bl_presented_set(&boot, 500);
	KUNIT_EXPECT_EQ(test, dcp_bl_presented_actual(&boot, reported), 500);

	/* System sleep: the backlight core presents 0, then the clear swap. */
	dcp_bl_presented_set(&resume, 500);
	dcp_bl_presented_set(&resume, 0);
	reported = 0;
	dcp_bl_presented_set(&resume, 0);
	/* Power-on carries the core's 0; the core's resume then presents 500. */
	dcp_bl_presented_set(&resume, 0);
	KUNIT_EXPECT_EQ(test, dcp_bl_presented_actual(&resume, reported), 0);
	dcp_bl_presented_set(&resume, 500);
	reported = 500;
	KUNIT_EXPECT_EQ(test, dcp_bl_presented_actual(&resume, reported), 500);

	/* Boot ends in the state resume does, even with a stale 0 report. */
	KUNIT_EXPECT_EQ(test, dcp_bl_presented_actual(&boot, 0),
			dcp_bl_presented_actual(&resume, 500));
}

static struct kunit_case backlight_cases[] = {
	KUNIT_CASE(backlight_presented_boot_resume_test),
	KUNIT_CASE(backlight_reports_keep_target_test),
	KUNIT_CASE(backlight_millinits_takeover_test),
	KUNIT_CASE(backlight_takeover_test),
	KUNIT_CASE(backlight_range_test),
	KUNIT_CASE(backlight_dpms_cycles_test),
	KUNIT_CASE(backlight_blanked_target_test),
	KUNIT_CASE(backlight_suspend_test),
	KUNIT_CASE(backlight_surface_and_busy_test),
	KUNIT_CASE(backlight_reject_test),
	KUNIT_CASE(backlight_inflight_dpms_test),
	KUNIT_CASE(backlight_resend_test),
	KUNIT_CASE(backlight_fallback_test),
	KUNIT_CASE(backlight_unknown_takeover_test),
	KUNIT_CASE(backlight_first_change_fades_test),
	KUNIT_CASE(backlight_fallback_same_level_test),
	KUNIT_CASE(backlight_seed_known_test),
	KUNIT_CASE(backlight_desktop_swap_test),
	KUNIT_CASE(backlight_fade_test),
	KUNIT_CASE(backlight_fade_restart_test),
	KUNIT_CASE(backlight_fade_blank_test),
	KUNIT_CASE(edr_ceiling_test),
	KUNIT_CASE(edr_target_test),
	KUNIT_CASE(edr_low_end_steady_test),
	KUNIT_CASE(edr_fade_monotonic_test),
	KUNIT_CASE(edr_start_ramp_test),
	KUNIT_CASE(edr_timed_ramp_test),
	KUNIT_CASE(edr_timed_limits_test),
	{}
};

static struct kunit_suite backlight_suite = {
	.name = "apple-dcp-backlight",
	.test_cases = backlight_cases,
};

kunit_test_suite(backlight_suite);

MODULE_DESCRIPTION("KUnit tests for the Apple DCP backlight");
MODULE_LICENSE("Dual MIT/GPL");
