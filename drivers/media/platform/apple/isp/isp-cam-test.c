// SPDX-License-Identifier: GPL-2.0-only
/* Setfile setup boundary tests; no firmware data or hardware access. */
#include <kunit/test.h>
#include <kunit/static_stub.h>
#include <kunit/device.h>
#include "isp-ipc.h"

struct isp_test_command_trace {
	unsigned int count;
	unsigned int fail_at;
	int setfile_error;
	unsigned int setfile_calls;
};

static int isp_test_send(struct apple_isp *isp, struct isp_channel *chan,
			 unsigned long timeout)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_command_trace *trace = test->priv;
	u32 *words = isp->cmd_virt;

	trace->count++;
	KUNIT_EXPECT_EQ(test, words[1], trace->count == 1 ?
			 CISP_CMD_FLICKER_SENSOR_SET : CISP_CMD_CH_SBS_ENABLE);
	return trace->count == trace->fail_at ? -ETIME : 0;
}

static struct apple_isp *isp_test_capture_alloc(struct kunit *test)
{
	static const struct apple_isp_hw hw = { };
	struct apple_isp *isp = kunit_kzalloc(test, sizeof(*isp), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, isp);
	isp->dev = kunit_device_register(test, "apple-isp-setfile-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, isp->dev);
	isp->hw = &hw;
	isp->chan_io = kunit_kzalloc(test, sizeof(*isp->chan_io), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, isp->chan_io);
	isp->chan_io->name = "test";
	isp->cmd_virt = kunit_kzalloc(test, 256, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, isp->cmd_virt);
	kunit_activate_static_stub(test, ipc_chan_send, isp_test_send);
	return isp;
}

static int isp_test_load_setfile(struct apple_isp *isp, u32 ch)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_command_trace *trace = test->priv;

	trace->setfile_calls++;
	return trace->setfile_error;
}

static void isp_test_setfile_failure_stops_setup(struct kunit *test)
{
	static const int errors[] = { -EINVAL, -EIO, -ETIME, -EINTR, -ENOMEM };
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_test_command_trace trace;

	kunit_activate_static_stub(test, isp_ch_load_setfile, isp_test_load_setfile);
	test->priv = &trace;
	for (unsigned int i = 0; i < ARRAY_SIZE(errors); i++) {
		trace = (struct isp_test_command_trace){ .setfile_error = errors[i] };
		KUNIT_EXPECT_EQ(test, isp_ch_configure_capture(isp, 0), errors[i]);
		/* Only the flicker command preceded the rejected tuning. */
		KUNIT_EXPECT_EQ(test, trace.count, 1);
		KUNIT_EXPECT_EQ(test, trace.setfile_calls, 1);
	}
}

static void isp_test_optional_setfile_continues_setup(struct kunit *test)
{
	static const int results[] = { 0, -ENOENT };
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_test_command_trace trace;

	kunit_activate_static_stub(test, isp_ch_load_setfile, isp_test_load_setfile);
	test->priv = &trace;
	for (unsigned int i = 0; i < ARRAY_SIZE(results); i++) {
		trace = (struct isp_test_command_trace){
			.setfile_error = results[i], .fail_at = 2,
		};
		/* The SBS command after tuning, rather than tuning, fails. */
		KUNIT_EXPECT_EQ(test, isp_ch_configure_capture(isp, 0), -ETIME);
		KUNIT_EXPECT_EQ(test, trace.count, 2);
		KUNIT_EXPECT_EQ(test, trace.setfile_calls, 1);
	}
}

static struct kunit_case isp_cam_test_cases[] = {
	KUNIT_CASE(isp_test_setfile_failure_stops_setup),
	KUNIT_CASE(isp_test_optional_setfile_continues_setup),
	{}
};

static struct kunit_suite isp_cam_test_suite = {
	.name = "apple-isp-cam",
	.test_cases = isp_cam_test_cases,
};

kunit_test_suite(isp_cam_test_suite);
