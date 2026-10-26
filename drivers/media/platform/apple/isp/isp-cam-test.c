// SPDX-License-Identifier: GPL-2.0-only
/* Synthetic geometry checks: no camera firmware or setfile data. */
#include <kunit/test.h>
#include <kunit/static_stub.h>
#include <kunit/device.h>
#include "isp-ipc.h"

static void isp_test_sensor_geometry(struct kunit *test)
{
	struct cmd_ch_camera_config config = {
		.in_width = 1920, .in_height = 2160,
		.out_width = 1920, .out_height = 1080,
		.in_width2 = 1920, .in_height2 = 2160,
		.out_width2 = 1920, .out_height2 = 1080,
	};

	KUNIT_EXPECT_TRUE(test, isp_sensor_config_usable(&config));
	/* A future larger output is accepted only when both descriptions agree. */
	config.in_width = 4096;
	config.in_width2 = 4096;
	config.in_height = 3072;
	config.in_height2 = 3072;
	config.out_width = 4096;
	config.out_width2 = 4096;
	config.out_height = 3072;
	config.out_height2 = 3072;
	KUNIT_EXPECT_TRUE(test, isp_sensor_config_usable(&config));
	config.out_height2 = 2160;
	KUNIT_EXPECT_FALSE(test, isp_sensor_config_usable(&config));
	config.out_height2 = 3072;
	config.in_height2 = 2160;
	KUNIT_EXPECT_FALSE(test, isp_sensor_config_usable(&config));
	config.in_height2 = 3072;
	config.out_width = 4097;
	config.out_width2 = 4097;
	KUNIT_EXPECT_FALSE(test, isp_sensor_config_usable(&config));
	config.out_width = 0;
	config.out_width2 = 0;
	KUNIT_EXPECT_FALSE(test, isp_sensor_config_usable(&config));
	config.out_width = 8192;
	config.out_width2 = 8192;
	KUNIT_EXPECT_FALSE(test, isp_sensor_config_usable(&config));
}

struct isp_test_command {
	u32 words[39];
	u32 size;
};

/* Derived from the proxy command fixture, with contiguous V4L2 plane sizes. */
static const struct isp_test_command isp_test_preview_commands[] = {
	{{0, 0x24, 0}, 0xc},
	{{0, 0x13b, 0, 1}, 0x10},
	{{0, 0x107, 0, 5}, 0x10},
	{{0, 0x10e, 0, 1}, 0x10},
	{{0, 0x10f, 0}, 0xc},
	{{0, 0x80c, 0, 0, 0, 1280, 720}, 0x1c},
	{{0, 0xb09, 0, 1280, 720, 1, 0x12, 2560, 2560, 0, 0, 720, 0, 1280}, 0x38},
	{{0, 0x208, 0, 7680}, 0x10},
	{{0, 0x20a, 0, 3840}, 0x10},
	{{[1] = 0x117, [3] = 8 << 16, [4] = 0x8000, [5] = 0x8000, [37] = 1}, 0x9c},
	{{[1] = 0x117, [3] = 9 | 2 << 16, [4] = 1843200, [5] = 2560,
	  [12] = 921600, [13] = 2560, [37] = 2}, 0x9c},
	{{0, 0x125, 0, 1}, 0x10},
	{{0, 0xb0d, 0, 1}, 0x10},
	{{0, 0x138, 0, 0}, 0x10},
};

struct isp_test_command_trace {
	unsigned int count;
	unsigned int fail_at;
};

static int isp_test_send(struct apple_isp *isp, void *args, u32 insize,
			 u32 outsize, int timeout)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_command_trace *trace = test->priv;
	const struct isp_test_command *expected;

	if (trace->count >= ARRAY_SIZE(isp_test_preview_commands)) {
		KUNIT_FAIL(test, "unexpected extra command");
		return -EIO;
	}
	expected = &isp_test_preview_commands[trace->count++];
	KUNIT_EXPECT_EQ(test, insize, expected->size);
	KUNIT_EXPECT_EQ(test, outsize, expected->words[1] == 0x117 ? insize : 0);
	if (insize == expected->size)
		KUNIT_EXPECT_MEMEQ(test, args, expected->words, insize);
	if (trace->count == trace->fail_at)
		return -ETIME;
	return 0;
}

static void isp_test_free_controls(void *ptr)
{
	v4l2_ctrl_handler_free(ptr);
}

static struct apple_isp *isp_test_capture_alloc(struct kunit *test)
{
	static const struct apple_isp_hw hw = {
		.gen = ISP_GEN_T6040, .fw_abi = ISP_FW_ABI_H17,
		.scl1 = true, .meta_size = ISP_META_SIZE_T8140,
	};
	static struct isp_preset preset = {
		.index = 5, .input_dim = {1920, 2160},
		.output_dim = {1280, 720}, .crop_size = {1280, 720},
	};
	struct apple_isp *isp = kunit_kzalloc(test, sizeof(*isp), GFP_KERNEL);
	struct isp_format *fmt;
	int err;

	KUNIT_ASSERT_NOT_NULL(test, isp);
	isp->dev = kunit_device_register(test, "apple-isp-camera-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, isp->dev);
	isp->hw = &hw;
	isp->frame_rate = 30;
	fmt = isp_get_current_format(isp);
	fmt->version = 0x958;
	fmt->pixelformat = V4L2_PIX_FMT_P010;
	fmt->preset = &preset;
	fmt->strides[0] = 2560;
	fmt->strides[1] = 2560;
	fmt->plane_size[0] = 1843200;
	fmt->plane_size[1] = 921600;
	v4l2_ctrl_handler_init(&isp->ctrl_handler, 1);
	err = kunit_add_action_or_reset(test, isp_test_free_controls, &isp->ctrl_handler);
	KUNIT_ASSERT_EQ(test, err, 0);
	isp->exposure_priority =
		v4l2_ctrl_new_std(&isp->ctrl_handler, NULL,
				  V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0, 1, 1, 1);
	KUNIT_ASSERT_EQ(test, isp->ctrl_handler.error, 0);
	kunit_activate_static_stub(test, isp_cmd_send, isp_test_send);
	return isp;
}

static void isp_test_preview_stream(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_test_command_trace trace = {};

	test->priv = &trace;
	KUNIT_EXPECT_EQ(test, isp_ch_configure_capture(isp, 0), 0);
	KUNIT_EXPECT_EQ(test, trace.count, ARRAY_SIZE(isp_test_preview_commands));
}

static void isp_test_preview_command_failures(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_test_command_trace trace;

	test->priv = &trace;
	for (unsigned int i = 1; i <= ARRAY_SIZE(isp_test_preview_commands); i++) {
		trace = (struct isp_test_command_trace){ .fail_at = i };
		KUNIT_EXPECT_EQ(test, isp_ch_configure_capture(isp, 0), -ETIME);
		KUNIT_EXPECT_EQ(test, trace.count, i);
	}
}

static int isp_test_config_reply(struct apple_isp *isp, void *args, u32 insize,
				 u32 outsize, int timeout)
{
	struct kunit *test = kunit_get_current_test();
	u32 *count = test->priv;
	struct cmd_ch_camera_config_h17 *reply = isp->cmd_virt;
	struct cmd_ch_camera_config *config = &reply->config;

	KUNIT_EXPECT_EQ(test, insize, 0x120);
	KUNIT_EXPECT_EQ(test, outsize, 0x120);
	memcpy(reply, args, sizeof(*reply));
	KUNIT_EXPECT_EQ(test, config->preset, (*count)++);
	config->in_width = 1920;
	config->in_height = 2160;
	config->out_width = 1920;
	config->out_height = 1080;
	if (config->preset == 1) {
		config->in_width = 4096;
		config->in_height = 3072;
		config->out_width = 4096;
		config->out_height = 3072;
	}
	config->in_width2 = config->in_width;
	config->in_height2 = config->in_height;
	config->out_width2 = config->out_width;
	config->out_height2 = config->out_height;
	if (config->preset == 2)
		config->out_height2 = 2160;
	reply->unknown[sizeof(reply->unknown) - 1] = 0x5a;
	return 0;
}

static void isp_test_free_configs(void *ptr)
{
	kfree(ptr);
}

static void isp_test_sensor_config_list(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_preset presets[] = {
		{ .index = 5, .input_dim = {1920, 2160},
		  .output_dim = {1920, 1080}, .crop_size = {1920, 1080} },
		{ .index = 5, .input_dim = {1920, 2160},
		  .output_dim = {1280, 720}, .crop_size = {1280, 720} },
	};
	u32 count = 0;
	int err;

	isp->cmd_virt = kunit_kzalloc(test, 0x120, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, isp->cmd_virt);
	isp->presets = presets;
	isp->num_presets = ARRAY_SIZE(presets);
	test->priv = &count;
	kunit_activate_static_stub(test, isp_cmd_send, isp_test_config_reply);
	KUNIT_ASSERT_EQ(test, isp_ch_cache_sensor_configs(isp, 0, 6), 0);
	err = kunit_add_action_or_reset(test, isp_test_free_configs, isp->sensor_configs);
	KUNIT_ASSERT_EQ(test, err, 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(isp->sensor_configs[5].reply + 0xc), 5);
	KUNIT_EXPECT_EQ(test, isp->sensor_configs[5].reply[ISP_SENSOR_CONFIG_SIZE - 1], 0x5a);
	KUNIT_EXPECT_EQ(test, count, 6);
	KUNIT_EXPECT_EQ(test, isp->num_sensor_configs, 6);
	KUNIT_EXPECT_EQ(test, isp->num_presets, 3);
	KUNIT_EXPECT_EQ(test, isp->presets[0].index, 5);
	KUNIT_EXPECT_EQ(test, isp->presets[0].output_dim.y, 1080);
	KUNIT_EXPECT_TRUE(test, isp->presets[1].p010);
	KUNIT_EXPECT_TRUE(test, isp->presets[2].p010);
	KUNIT_EXPECT_EQ(test, isp->presets[2].index, 1);
	KUNIT_EXPECT_EQ(test, isp->presets[2].output_dim.x, 4096);
	KUNIT_EXPECT_EQ(test, isp->presets[2].crop_size.y, 3072);
	KUNIT_EXPECT_FALSE(test, isp->sensor_configs[2].usable);
	KUNIT_EXPECT_EQ(test, isp_ch_cache_sensor_configs(isp, 0, 0), -EINVAL);
	KUNIT_EXPECT_EQ(test, isp_ch_cache_sensor_configs(isp, 0, 65), -EINVAL);
	KUNIT_EXPECT_EQ(test, count, 6);
}

static int isp_test_bad_config_reply(struct apple_isp *isp, void *args, u32 insize,
				     u32 outsize, int timeout)
{
	struct cmd_ch_camera_config_h17 *reply = isp->cmd_virt;

	isp_test_config_reply(isp, args, insize, outsize, timeout);
	reply->config.preset = 65;
	return 0;
}

static void isp_test_sensor_config_failure(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_preset preset = {};
	u32 count = 0;

	isp->cmd_virt = kunit_kzalloc(test, ISP_SENSOR_CONFIG_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, isp->cmd_virt);
	isp->presets = &preset;
	isp->num_presets = 1;
	test->priv = &count;
	kunit_activate_static_stub(test, isp_cmd_send, isp_test_bad_config_reply);
	KUNIT_EXPECT_EQ(test, isp_ch_cache_sensor_configs(isp, 0, 6), -EIO);
	KUNIT_EXPECT_EQ(test, count, 1);
	KUNIT_EXPECT_EQ(test, isp->num_sensor_configs, 0);
	KUNIT_EXPECT_PTR_EQ(test, isp->sensor_configs, NULL);
	KUNIT_EXPECT_PTR_EQ(test, isp->presets, &preset);
}

struct isp_test_ack_state {
	unsigned int fault, calls;
};

static int isp_test_ack_reply(struct apple_isp *isp, struct isp_channel *chan,
			      unsigned long timeout)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_ack_state *state = test->priv;

	state->calls++;
	chan->rsp = (struct isp_message){ .arg0 = chan->req.arg0 | ISP_IPC_FLAG_ACK,
					  .arg1 = chan->req.arg2 };
	switch (state->fault) {
	case 1:
		chan->rsp.arg2 = 1;
		break;
	case 2:
		put_unaligned_le32(1, isp->cmd_virt);
		break;
	case 3:
		chan->rsp.arg1 += 4;
		break;
	case 4:
		return -ETIME;
	}
	return 0;
}

static void isp_test_command_ack_failure_latches(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_test_ack_state state = {};

	isp->cmd_virt = kunit_kzalloc(test, 0x400, GFP_KERNEL);
	isp->chan_io = kunit_kzalloc(test, sizeof(*isp->chan_io), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, isp->cmd_virt);
	KUNIT_ASSERT_NOT_NULL(test, isp->chan_io);
	isp->chan_io->name = "IO";
	test->priv = &state;
	kunit_deactivate_static_stub(test, isp_cmd_send);
	kunit_activate_static_stub(test, ipc_chan_send, isp_test_ack_reply);
	KUNIT_EXPECT_EQ(test, isp_cmd_ch_camera_config_select(isp, 0, 5), 0);
	for (state.fault = 1; state.fault <= 4; state.fault++) {
		isp->ipc_failed = false;
		state.calls = 0;
		KUNIT_EXPECT_EQ(test, isp_cmd_ch_camera_config_select(isp, 0, 5),
				state.fault == 4 ? -ETIME : -EIO);
		KUNIT_EXPECT_TRUE(test, isp->ipc_failed);
		/* A late ACK cannot admit another command on an uncertain slot. */
		KUNIT_EXPECT_EQ(test, isp_cmd_ch_camera_config_select(isp, 0, 5), -EIO);
		KUNIT_EXPECT_EQ(test, state.calls, 1);
	}
}

static struct kunit_case isp_cam_test_cases[] = {
	KUNIT_CASE(isp_test_command_ack_failure_latches),
	KUNIT_CASE(isp_test_sensor_geometry),
	KUNIT_CASE(isp_test_sensor_config_list),
	KUNIT_CASE(isp_test_sensor_config_failure),
	KUNIT_CASE(isp_test_preview_stream),
	KUNIT_CASE(isp_test_preview_command_failures),
	{}
};

static struct kunit_suite isp_cam_test_suite = {
	.name = "apple-isp-camera",
	.test_cases = isp_cam_test_cases,
};

kunit_test_suite(isp_cam_test_suite);
