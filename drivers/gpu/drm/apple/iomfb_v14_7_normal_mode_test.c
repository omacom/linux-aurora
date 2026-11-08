// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * The wait after an external display's power-on for its timings to run,
 * included at the end of iomfb_v14_7.c so that the production
 * dcp_v14_external_wait_normal() and D104 handler run. Only the link pump
 * is stubbed: it delivers the callbacks a case scripts, as the firmware
 * queues them after A472 replied.
 */
#include <kunit/device.h>
#include <kunit/static_stub.h>
#include <kunit/test.h>

/* What the stubbed pump does on each call, in order. */
enum v14_nm_step {
	V14_NM_OTHER,		/* an unrelated callback */
	V14_NM_NORMAL_0,	/* D104 NormalModeActive = 0 */
	V14_NM_NORMAL_1,	/* D104 NormalModeActive = 1 */
	V14_NM_TIMEOUT,		/* nothing arrived in time */
	V14_NM_ERROR,		/* a callback that failed */
};

struct v14_nm {
	struct apple_dcp_v14 *v14;
	const enum v14_nm_step *steps;
	unsigned int count, pumps;
};

static struct v14_nm *v14_nm_current;

/* D104: a 64-byte key, then the boolean. */
static int v14_nm_d104(struct apple_dcp_v14 *v14, bool value)
{
	u8 in[68] = {};

	strscpy((char *)in, "NormalModeActive", 64);
	in[64] = value;
	return dcp_v14_callback(v14, D(104), in, sizeof(in), NULL, 0);
}

static int v14_nm_pump(struct apple_dcp_v14 *v14, unsigned long timeout)
{
	struct v14_nm *s = v14_nm_current;

	if (s->pumps == s->count)
		return -ETIMEDOUT;
	switch (s->steps[s->pumps++]) {
	case V14_NM_OTHER:
		return 0;
	case V14_NM_NORMAL_0:
		return v14_nm_d104(v14, false);
	case V14_NM_NORMAL_1:
		return v14_nm_d104(v14, true);
	case V14_NM_TIMEOUT:
		return -ETIMEDOUT;
	case V14_NM_ERROR:
		return -EPROTO;
	}
	return -EINVAL;
}

static void v14_nm_stopped(struct work_struct *work)
{
}

static void v14_nm_stop(void *data)
{
	struct apple_dcp_v14 *v14 = data;

	cancel_work_sync(&v14->stopped_work);
}

/* A powered-on external session; the pump plays @steps. */
static struct v14_nm *v14_nm_setup(struct kunit *test, const enum v14_nm_step *steps,
				   unsigned int count)
{
	struct v14_nm *s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	struct device *dev;

	KUNIT_ASSERT_NOT_NULL(test, s);
	dev = kunit_device_register(test, "dcp-v14-normal-mode");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	s->v14 = devm_kzalloc(dev, sizeof(*s->v14), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s->v14);
	s->v14->dev = dev;
	s->v14->external = true;
	s->v14->started = true;
	s->v14->opened = true;
	s->v14->powered = true;
	mutex_init(&s->v14->lock);
	INIT_WORK(&s->v14->stopped_work, v14_nm_stopped);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, v14_nm_stop, s->v14), 0);
	s->steps = steps;
	s->count = count;
	v14_nm_current = s;
	kunit_activate_static_stub(test, dcp_v14_pump, v14_nm_pump);
	return s;
}

static int v14_nm_wait(struct v14_nm *s)
{
	int ret;

	mutex_lock(&s->v14->lock);
	ret = dcp_v14_external_wait_normal(s->v14);
	mutex_unlock(&s->v14->lock);
	return ret;
}

/* Firmware that never reported NormalModeActive: no wait. */
static void v14_nm_not_reported(struct kunit *test)
{
	struct v14_nm *s = v14_nm_setup(test, NULL, 0);

	KUNIT_EXPECT_EQ(test, v14_nm_wait(s), 0);
	KUNIT_EXPECT_EQ(test, s->pumps, 0U);
}

/* The run mode change ended within A472 (a slow mode): no wait. */
static void v14_nm_already_running(struct kunit *test)
{
	struct v14_nm *s = v14_nm_setup(test, NULL, 0);

	KUNIT_ASSERT_EQ(test, v14_nm_d104(s->v14, false), 0);
	KUNIT_ASSERT_EQ(test, v14_nm_d104(s->v14, true), 0);
	KUNIT_EXPECT_EQ(test, v14_nm_wait(s), 0);
	KUNIT_EXPECT_EQ(test, s->pumps, 0U);
}

/*
 * The J516C 3840x2560 DSC sequence: A472 replied with NormalModeActive 0,
 * other callbacks follow, then 1. The wait ends on it, not before.
 */
static void v14_nm_deferred(struct kunit *test)
{
	static const enum v14_nm_step steps[] = {
		V14_NM_OTHER, V14_NM_OTHER, V14_NM_NORMAL_1, V14_NM_OTHER,
	};
	struct v14_nm *s = v14_nm_setup(test, steps, ARRAY_SIZE(steps));

	KUNIT_ASSERT_EQ(test, v14_nm_d104(s->v14, false), 0);
	KUNIT_EXPECT_EQ(test, v14_nm_wait(s), 0);
	KUNIT_EXPECT_EQ(test, s->pumps, 3U);
	KUNIT_EXPECT_FALSE(test, s->v14->failed);
}

/* A power-on after the first sets 0 again: it is waited for alike. */
static void v14_nm_every_power_on(struct kunit *test)
{
	static const enum v14_nm_step steps[] = { V14_NM_NORMAL_1, V14_NM_NORMAL_1 };
	struct v14_nm *s = v14_nm_setup(test, steps, ARRAY_SIZE(steps));

	KUNIT_ASSERT_EQ(test, v14_nm_d104(s->v14, false), 0);
	KUNIT_EXPECT_EQ(test, v14_nm_wait(s), 0);
	KUNIT_ASSERT_EQ(test, v14_nm_d104(s->v14, false), 0);
	KUNIT_EXPECT_EQ(test, v14_nm_wait(s), 0);
	KUNIT_EXPECT_EQ(test, s->pumps, 2U);
}

/* Only service 0 counts: the IOMF service's NormalModeActive (D415) does not. */
static void v14_nm_other_service(struct kunit *test)
{
	struct v14_nm *s = v14_nm_setup(test, NULL, 0);
	struct dcp_v14_property *p;
	static const u8 key[64] = "NormalModeActive";

	p = dcp_v14_property(s->v14, 1229933894, key, true);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, p);
	p->value = 0;
	KUNIT_EXPECT_EQ(test, v14_nm_wait(s), 0);
	KUNIT_EXPECT_EQ(test, s->pumps, 0U);
}

/* Timings that never start fail the mode set; the session goes on. */
static void v14_nm_timeout(struct kunit *test)
{
	static const enum v14_nm_step steps[] = { V14_NM_OTHER, V14_NM_TIMEOUT };
	struct v14_nm *s = v14_nm_setup(test, steps, ARRAY_SIZE(steps));

	KUNIT_ASSERT_EQ(test, v14_nm_d104(s->v14, false), 0);
	KUNIT_EXPECT_EQ(test, v14_nm_wait(s), -ETIMEDOUT);
	KUNIT_EXPECT_FALSE(test, s->v14->failed);
}

/* A callback that fails stops the session, as in dcp_v14_idle(). */
static void v14_nm_callback_failed(struct kunit *test)
{
	static const enum v14_nm_step steps[] = { V14_NM_ERROR, V14_NM_NORMAL_1 };
	struct v14_nm *s = v14_nm_setup(test, steps, ARRAY_SIZE(steps));

	KUNIT_ASSERT_EQ(test, v14_nm_d104(s->v14, false), 0);
	KUNIT_EXPECT_EQ(test, v14_nm_wait(s), -EPROTO);
	KUNIT_EXPECT_TRUE(test, s->v14->failed);
	KUNIT_EXPECT_EQ(test, s->pumps, 1U);
}

/* A session that failed meanwhile is not waited for. */
static void v14_nm_session_failed(struct kunit *test)
{
	struct v14_nm *s = v14_nm_setup(test, NULL, 0);

	KUNIT_ASSERT_EQ(test, v14_nm_d104(s->v14, false), 0);
	s->v14->failed = true;
	KUNIT_EXPECT_EQ(test, v14_nm_wait(s), -EIO);
	KUNIT_EXPECT_EQ(test, s->pumps, 0U);
}

static struct kunit_case v14_nm_cases[] = {
	KUNIT_CASE(v14_nm_not_reported),
	KUNIT_CASE(v14_nm_already_running),
	KUNIT_CASE(v14_nm_deferred),
	KUNIT_CASE(v14_nm_every_power_on),
	KUNIT_CASE(v14_nm_other_service),
	KUNIT_CASE(v14_nm_timeout),
	KUNIT_CASE(v14_nm_callback_failed),
	KUNIT_CASE(v14_nm_session_failed),
	{}
};

static struct kunit_suite v14_nm_suite = {
	.name = "dcp_v14_normal_mode",
	.test_cases = v14_nm_cases,
};

kunit_test_suite(v14_nm_suite);
