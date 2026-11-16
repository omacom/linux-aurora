// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * The DCP calls of an external mode set, included at the end of
 * iomfb_v14_7.c so that the production dcp_v14_external_modeset() runs.
 * Every call is answered as accepted; the audio service is stubbed.
 */
#include <kunit/device.h>
#include <kunit/static_stub.h>
#include <kunit/test.h>

#define V14_EXT_MODESET_MAX_CALLS 8

struct v14_ext_modeset {
	struct apple_dcp *dcp;
	struct apple_dcp_v14 *v14;
	struct dcp_display_mode mode;
	u32 tags[V14_EXT_MODESET_MAX_CALLS];
	bool power[V14_EXT_MODESET_MAX_CALLS];
	unsigned int calls;
};

static struct v14_ext_modeset *v14_ext_modeset_current;

static int v14_ext_modeset_call(struct apple_dcp_v14 *v14, u32 tag, const void *in, u32 in_size,
				void *out, u32 out_size, u32 completion, dcp_v14_admit_fn admit,
				void *admit_cookie)
{
	struct v14_ext_modeset *s = v14_ext_modeset_current;

	if (s->calls < V14_EXT_MODESET_MAX_CALLS) {
		s->tags[s->calls] = tag;
		s->power[s->calls] = tag == A(472) && in_size >= 8 && get_unaligned_le64(in);
	}
	s->calls++;
	if (out)
		memset(out, 0, out_size);
	return 0;
}

static void v14_ext_modeset_audio(struct apple_dcp *dcp)
{
}

static void v14_ext_modeset_retry(struct work_struct *work)
{
}

static void v14_ext_modeset_stop(void *data)
{
	struct apple_dcp *dcp = data;

	cancel_delayed_work_sync(&dcp->external_retry_wq);
}

/* A started, described native external session with one mode, its display @powered. */
static struct v14_ext_modeset *v14_ext_modeset_setup(struct kunit *test, bool powered)
{
	struct v14_ext_modeset *s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	struct device *dev;

	KUNIT_ASSERT_NOT_NULL(test, s);
	dev = kunit_device_register(test, "dcp-v14-ext-modeset");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	s->dcp = devm_kzalloc(dev, sizeof(*s->dcp), GFP_KERNEL);
	s->v14 = devm_kzalloc(dev, sizeof(*s->v14), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s->dcp);
	KUNIT_ASSERT_NOT_NULL(test, s->v14);
	s->dcp->dev = dev;
	s->dcp->fw_compat = DCP_FIRMWARE_V_14_7;
	s->dcp->external = true;
	s->dcp->external_native = true;
	/* Never dereferenced: the audio service is stubbed. */
	s->dcp->avep = (struct apple_dcp_afkep *)s;
	mutex_init(&s->dcp->modes_lock);
	mutex_init(&s->dcp->hpd_mutex);
	spin_lock_init(&s->dcp->metadata.lock);
	spin_lock_init(&s->dcp->mode_state.lock);
	INIT_DELAYED_WORK(&s->dcp->external_retry_wq, v14_ext_modeset_retry);
	/* 3840x2160 at 100 Hz, timing 167 and color 219 as a J514S TV reports it. */
	s->mode.mode = (struct drm_display_mode){
		DRM_MODE("3840x2160", DRM_MODE_TYPE_DRIVER, 1188000, 3840, 4896, 4984, 5280, 0,
			 2160, 2168, 2178, 2250, 0, DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC)
	};
	s->mode.timing_mode_id = 167;
	s->mode.color_mode_id = 219;
	s->dcp->modes = &s->mode;
	s->dcp->nr_modes = 1;
	s->dcp->modes_admitted = true;
	s->dcp->v14 = s->v14;
	s->v14->dev = dev;
	s->v14->dcp = s->dcp;
	s->v14->external = true;
	s->v14->started = true;
	s->v14->opened = true;
	s->v14->powered = powered;
	s->v14->described = DCP_V14_DESC_ALL;
	mutex_init(&s->v14->lock);
	init_waitqueue_head(&s->v14->described_wait);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, v14_ext_modeset_stop, s->dcp), 0);
	v14_ext_modeset_current = s;
	kunit_activate_static_stub(test, dcp_v14_call_guarded, v14_ext_modeset_call);
	kunit_activate_static_stub(test, dcp_v14_audio_off, v14_ext_modeset_audio);
	kunit_activate_static_stub(test, dcp_v14_audio_on, v14_ext_modeset_audio);
	return s;
}

static int v14_ext_modeset_run(struct kunit *test, struct v14_ext_modeset *s)
{
	struct drm_crtc_state *state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, state);
	drm_mode_copy(&state->mode, &s->mode.mode);
	return dcp_v14_external_modeset(s->dcp, state);
}

/*
 * A display that was off, as on attach: the mode is set, the display powered
 * on, and the mode set once more with the pipe on, before any swap. A DSC
 * mode set with the pipe off only takes effect then.
 */
static void v14_ext_modeset_sets_mode_again_after_power_on(struct kunit *test)
{
	struct v14_ext_modeset *s = v14_ext_modeset_setup(test, false);

	KUNIT_EXPECT_EQ(test, v14_ext_modeset_run(test, s), 0);
	KUNIT_ASSERT_EQ(test, s->calls, 3U);
	KUNIT_EXPECT_EQ(test, s->tags[0], A(411));
	KUNIT_EXPECT_EQ(test, s->tags[1], A(472));
	KUNIT_EXPECT_TRUE(test, s->power[1]);
	KUNIT_EXPECT_EQ(test, s->tags[2], A(411));
	KUNIT_EXPECT_TRUE(test, s->v14->powered);
	KUNIT_EXPECT_TRUE(test, s->v14->mode_live);
	KUNIT_EXPECT_EQ(test, s->v14->mode_timing, 167U);
	KUNIT_EXPECT_EQ(test, s->v14->mode_color, 219U);
	KUNIT_EXPECT_FALSE(test, mutex_is_locked(&s->v14->lock));
}

/* A mode change on a display that is on: power down, set, power up, set again. */
static void v14_ext_modeset_mode_change(struct kunit *test)
{
	struct v14_ext_modeset *s = v14_ext_modeset_setup(test, true);

	KUNIT_EXPECT_EQ(test, v14_ext_modeset_run(test, s), 0);
	KUNIT_ASSERT_EQ(test, s->calls, 4U);
	KUNIT_EXPECT_EQ(test, s->tags[0], A(472));
	KUNIT_EXPECT_FALSE(test, s->power[0]);
	KUNIT_EXPECT_EQ(test, s->tags[1], A(411));
	KUNIT_EXPECT_EQ(test, s->tags[2], A(472));
	KUNIT_EXPECT_TRUE(test, s->power[2]);
	KUNIT_EXPECT_EQ(test, s->tags[3], A(411));
	KUNIT_EXPECT_TRUE(test, s->v14->powered);
}

static struct kunit_case v14_ext_modeset_cases[] = {
	KUNIT_CASE(v14_ext_modeset_sets_mode_again_after_power_on),
	KUNIT_CASE(v14_ext_modeset_mode_change),
	{}
};

static struct kunit_suite v14_ext_modeset_suite = {
	.name = "dcp_v14_external_modeset",
	.test_cases = v14_ext_modeset_cases,
};

kunit_test_suite(v14_ext_modeset_suite);
