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
	int setfile_error;
	unsigned int setfile_calls;
	u32 last_opcode;
};

static int isp_test_send(struct apple_isp *isp, void *args, u32 insize,
			 u32 outsize, int timeout)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_command_trace *trace = test->priv;
	const struct isp_test_command *expected;
	u32 expected_outsize;

	if (trace->count >= ARRAY_SIZE(isp_test_preview_commands)) {
		KUNIT_FAIL(test, "unexpected extra command");
		return -EIO;
	}
	expected = &isp_test_preview_commands[trace->count++];
	KUNIT_EXPECT_EQ(test, insize, expected->size);
	expected_outsize = expected->words[1] == 0x117 ||
		(expected->words[1] == 0x24 && !isp_is_p010(isp)) ? insize : 0;
	KUNIT_EXPECT_EQ(test, outsize, expected_outsize);
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

	isp_get_current_format(isp)->pixelformat = V4L2_PIX_FMT_NV12;
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

/* Count NV12 setup commands and fail the chosen one, without a fixture. */
static int isp_test_count_send(struct apple_isp *isp, void *args, u32 insize,
			       u32 outsize, int timeout)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_command_trace *trace = test->priv;

	trace->last_opcode = ((const u32 *)args)[1];
	if (++trace->count == trace->fail_at)
		return -ETIME;
	return 0;
}

static void isp_test_optional_setfile_continues_setup(struct kunit *test)
{
	static const int results[] = { 0, -ENOENT };
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_test_command_trace trace;

	isp_get_current_format(isp)->pixelformat = V4L2_PIX_FMT_NV12;
	kunit_activate_static_stub(test, isp_ch_load_setfile, isp_test_load_setfile);
	kunit_activate_static_stub(test, isp_cmd_send, isp_test_count_send);
	test->priv = &trace;
	for (unsigned int i = 0; i < ARRAY_SIZE(results); i++) {
		trace = (struct isp_test_command_trace){
			.setfile_error = results[i], .fail_at = 2,
		};
		/* The mains frequency after tuning, rather than tuning, fails. */
		KUNIT_EXPECT_EQ(test, isp_ch_configure_capture(isp, 0), -ETIME);
		KUNIT_EXPECT_EQ(test, trace.count, 2);
		KUNIT_EXPECT_EQ(test, trace.last_opcode, 0x8212);
		KUNIT_EXPECT_EQ(test, trace.setfile_calls, 1);
	}
}

struct isp_test_nv12_trace {
	unsigned int count;
	u32 opcode[64];
	u32 size[64];
	u32 word[64][3];
};

/* Accept every NV12 setup command and record its opcode and first fields. */
static int isp_test_record(struct apple_isp *isp, void *args, u32 insize,
			   u32 outsize, int timeout)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_nv12_trace *trace = test->priv;
	const u32 *words = args;
	unsigned int i = trace->count;

	if (i >= ARRAY_SIZE(trace->opcode)) {
		KUNIT_FAIL(test, "unexpected extra command");
		return -EIO;
	}
	trace->opcode[i] = words[1];
	trace->size[i] = insize;
	for (unsigned int j = 0; j < 3 && 12 + 4 * j < insize; j++)
		trace->word[i][j] = words[3 + j];
	trace->count++;
	return 0;
}

static int isp_test_find(const struct isp_test_nv12_trace *trace, u32 opcode)
{
	for (unsigned int i = 0; i < trace->count; i++)
		if (trace->opcode[i] == opcode)
			return i;
	return -1;
}

static int isp_test_no_setfile(struct apple_isp *isp, u32 ch)
{
	return 0;
}

static struct apple_isp *isp_test_nv12_alloc(struct kunit *test,
					     enum isp_sensor_id id)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_format *fmt = isp_get_current_format(isp);

	fmt->pixelformat = V4L2_PIX_FMT_NV12;
	fmt->id = id;
	fmt->version = isp_setfiles[id].version;
	isp->temporal_filter = 1;
	kunit_activate_static_stub(test, isp_ch_load_setfile, isp_test_no_setfile);
	kunit_activate_static_stub(test, isp_cmd_send, isp_test_record);
	return isp;
}

/* Expect exactly @n commands, in this order, from the first of them on. */
static void isp_test_expect_run(struct kunit *test,
				const struct isp_test_nv12_trace *trace,
				const u32 (*run)[2], unsigned int n)
{
	int first = isp_test_find(trace, run[0][0]);

	KUNIT_ASSERT_GE(test, first, 0);
	KUNIT_ASSERT_LE(test, first + n, trace->count);
	for (unsigned int i = 0; i < n; i++) {
		KUNIT_EXPECT_EQ(test, trace->opcode[first + i], run[i][0]);
		KUNIT_EXPECT_EQ(test, trace->size[first + i], run[i][1]);
	}
}

static void isp_test_temporal_filter_band0(struct kunit *test)
{
	static const u32 run[][2] = {
		{ 0xc100, 0x18 }, { 0xc120, 0x10 }, { 0xc11c, 0x10 },
		{ 0xc11d, 0x14 }, { 0xc102, 0x0c }, { 0xc113, 0x0c },
	};
	struct apple_isp *isp = isp_test_nv12_alloc(test, ISP_IMX958_2224_01);
	struct isp_test_nv12_trace *trace;
	int i;

	trace = kunit_kzalloc(test, sizeof(*trace), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, trace);
	test->priv = trace;
	KUNIT_EXPECT_EQ(test, isp_ch_configure_capture(isp, 0), 0);
	isp_test_expect_run(test, trace, run, ARRAY_SIZE(run));
	/* The start record is (1, 0, 0) whatever the DT property says. */
	i = isp_test_find(trace, 0xc100);
	KUNIT_EXPECT_EQ(test, trace->word[i][0], 1);
	KUNIT_EXPECT_EQ(test, trace->word[i][1], 0);
	KUNIT_EXPECT_EQ(test, trace->word[i][2], 0);
	i = isp_test_find(trace, 0xc11d);
	KUNIT_EXPECT_EQ(test, trace->word[i][0], 0x7800);
	KUNIT_EXPECT_EQ(test, trace->word[i][1], 0x6400);
	KUNIT_EXPECT_LT(test, isp_test_find(trace, 0xc114), 0);
}

static void isp_test_temporal_filter_off(struct kunit *test)
{
	static const u32 run[][2] = {
		{ 0xc114, 0x0c }, { 0xc103, 0x0c }, { 0xc101, 0x0c },
	};
	struct apple_isp *isp = isp_test_nv12_alloc(test, ISP_IMX558_1925_03);
	struct isp_test_nv12_trace *trace;

	trace = kunit_kzalloc(test, sizeof(*trace), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, trace);
	test->priv = trace;
	KUNIT_EXPECT_EQ(test, isp_ch_configure_capture(isp, 0), 0);
	isp_test_expect_run(test, trace, run, ARRAY_SIZE(run));
	KUNIT_EXPECT_LT(test, isp_test_find(trace, 0xc100), 0);
	KUNIT_EXPECT_LT(test, isp_test_find(trace, 0xc102), 0);
	KUNIT_EXPECT_LT(test, isp_test_find(trace, 0xc113), 0);
}

static void isp_test_power_line_frequency(struct kunit *test)
{
	struct apple_isp *isp = isp_test_nv12_alloc(test, ISP_IMX958_2224_01);
	struct isp_test_nv12_trace *trace;
	int i;

	trace = kunit_kzalloc(test, sizeof(*trace), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, trace);
	test->priv = trace;

	/* No control: 50 Hz, in Hz 16.16 */
	KUNIT_EXPECT_EQ(test, isp_ch_configure_capture(isp, 0), 0);
	i = isp_test_find(trace, 0x8212);
	KUNIT_ASSERT_GE(test, i, 0);
	KUNIT_EXPECT_EQ(test, trace->size[i], 0x10);
	KUNIT_EXPECT_EQ(test, trace->word[i][0], 0x00320000);
	/* After the tuning, before the sensor configuration is selected */
	KUNIT_EXPECT_LT(test, i, isp_test_find(trace, 0x107));

	isp->power_line_frequency =
		v4l2_ctrl_new_std_menu(&isp->ctrl_handler, NULL,
				       V4L2_CID_POWER_LINE_FREQUENCY,
				       V4L2_CID_POWER_LINE_FREQUENCY_60HZ,
				       BIT(V4L2_CID_POWER_LINE_FREQUENCY_DISABLED),
				       V4L2_CID_POWER_LINE_FREQUENCY_50HZ);
	KUNIT_ASSERT_NOT_NULL(test, isp->power_line_frequency);
	KUNIT_ASSERT_EQ(test, v4l2_ctrl_s_ctrl(isp->power_line_frequency,
					       V4L2_CID_POWER_LINE_FREQUENCY_60HZ), 0);
	memset(trace, 0, sizeof(*trace));
	KUNIT_EXPECT_EQ(test, isp_ch_configure_capture(isp, 0), 0);
	i = isp_test_find(trace, 0x8212);
	KUNIT_ASSERT_GE(test, i, 0);
	KUNIT_EXPECT_EQ(test, trace->word[i][0], 0x003c0000);
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
	/* Firmware configs only qualify P010: the uncropped 4096x3072 one is not offered. */
	KUNIT_ASSERT_EQ(test, isp->num_presets, 2);
	KUNIT_EXPECT_EQ(test, isp->presets[0].index, 5);
	KUNIT_EXPECT_EQ(test, isp->presets[0].output_dim.y, 1080);
	KUNIT_EXPECT_TRUE(test, isp->presets[1].p010);
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

static void isp_test_command_ack_failures(struct kunit *test)
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
		unsigned int fault = state.fault;

		state.calls = 0;
		KUNIT_EXPECT_EQ(test, isp_cmd_ch_camera_config_select(isp, 0, 5),
				fault == 4 ? -ETIME : -EIO);
		/* A failed command does not take the next one with it. */
		KUNIT_EXPECT_FALSE(test, isp->ipc_failed);
		state.fault = 0;
		KUNIT_EXPECT_EQ(test, isp_cmd_ch_camera_config_select(isp, 0, 5), 0);
		KUNIT_EXPECT_EQ(test, state.calls, 2);
		state.fault = fault;
	}

	/* Firmware that never answered gets no further command. */
	isp->ipc_failed = true;
	state.calls = 0;
	KUNIT_EXPECT_EQ(test, isp_cmd_ch_camera_config_select(isp, 0, 5), -EIO);
	KUNIT_EXPECT_EQ(test, state.calls, 0);
}

/* The firmware's answer to the stop key request, as macOS reads it */
static int isp_test_stop_key_reply(struct apple_isp *isp,
				   struct isp_channel *chan,
				   unsigned long timeout)
{
	struct kunit *test = kunit_get_current_test();
	unsigned int *calls = test->priv;
	u8 *cmd = isp->cmd_virt;

	(*calls)++;
	KUNIT_EXPECT_PTR_EQ(test, chan, isp->chan_dg);
	KUNIT_EXPECT_EQ(test, chan->req.arg1, 0x20);
	KUNIT_EXPECT_EQ(test, chan->req.arg2, 0x20);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(cmd + 4), 0x1d);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(cmd + 8), 1);
	KUNIT_EXPECT_TRUE(test, timeout);
	put_unaligned_le64(0x10001e00010ULL, cmd + 0x10);
	put_unaligned_le64(0x8042006, cmd + 0x18);
	chan->rsp = (struct isp_message){ .arg0 = chan->req.arg0 | ISP_IPC_FLAG_ACK,
					  .arg1 = chan->req.arg2 };
	return 0;
}

static void isp_test_stop_key_on_debug(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct cmd_stop_key key;
	unsigned int calls = 0;

	isp->cmd_virt = kunit_kzalloc(test, 0x400, GFP_KERNEL);
	isp->chan_io = kunit_kzalloc(test, sizeof(*isp->chan_io), GFP_KERNEL);
	isp->chan_dg = kunit_kzalloc(test, sizeof(*isp->chan_dg), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, isp->cmd_virt);
	KUNIT_ASSERT_NOT_NULL(test, isp->chan_io);
	KUNIT_ASSERT_NOT_NULL(test, isp->chan_dg);
	isp->chan_io->name = "IO";
	isp->chan_dg->name = "DEBUG";
	isp->chan_dg->type = ISP_IPC_CHAN_TYPE_COMMAND;
	test->priv = &calls;
	kunit_deactivate_static_stub(test, isp_cmd_send);
	kunit_activate_static_stub(test, ipc_chan_send, isp_test_stop_key_reply);

	KUNIT_EXPECT_EQ(test, isp_cmd_stop_key_get(isp, &key), 0);
	KUNIT_EXPECT_EQ(test, calls, 1);
	KUNIT_EXPECT_EQ(test, key.addr, 0x10001e00010ULL);
	KUNIT_EXPECT_EQ(test, key.key, 0x8042006ULL);

	/* No DEBUG command ring: nothing is sent, on IO either. */
	isp->chan_dg->type = ISP_IPC_CHAN_TYPE_REPLY;
	KUNIT_EXPECT_EQ(test, isp_cmd_stop_key_get(isp, &key), -ENODEV);
	isp->chan_dg = NULL;
	KUNIT_EXPECT_EQ(test, isp_cmd_stop_key_get(isp, &key), -ENODEV);
	KUNIT_EXPECT_EQ(test, calls, 1);
}

static void isp_test_call_crop(struct kunit *test)
{
	struct isp_preset source = {
		.index = 7, .input_dim = { 1936, 1096 },
		.output_dim = { 1920, 1080 }, .crop_offset = { 8, 8 },
		.crop_size = { 1920, 1080 }, .p010 = true,
	};
	struct isp_preset result;

	KUNIT_ASSERT_TRUE(test,
			  isp_size_from_crop(&source, (struct coord){ 640, 480 }, &result));
	KUNIT_EXPECT_EQ(test, result.index, 7);
	KUNIT_EXPECT_TRUE(test, result.p010);
	KUNIT_EXPECT_EQ(test, result.crop_offset.x, 248);
	KUNIT_EXPECT_EQ(test, result.crop_offset.y, 8);
	KUNIT_EXPECT_EQ(test, result.crop_size.x, 1440);
	KUNIT_EXPECT_EQ(test, result.crop_size.y, 1080);
	KUNIT_EXPECT_EQ(test, result.output_dim.x, 640);
	KUNIT_EXPECT_EQ(test, result.output_dim.y, 480);
}

static void isp_test_call_crop_bounds(struct kunit *test)
{
	struct isp_preset source = {
		.input_dim = { 1920, 2160 }, .output_dim = { 1920, 1080 },
		.crop_size = { 1920, 1080 },
	};
	struct isp_preset result;
	static const struct coord rejected[] = {
		{ 1080, 1920 }, { 1760, 1328 }, { 1552, 1552 },
		{ 1922, 1080 }, { 0, 480 }, { 641, 480 },
	};

	/* Stacked input rows do not make a portrait or a larger square. */
	for (unsigned int i = 0; i < ARRAY_SIZE(rejected); i++)
		KUNIT_EXPECT_FALSE(test, isp_size_from_crop(&source, rejected[i], &result));
	source.crop_offset.x = 2;
	KUNIT_EXPECT_FALSE(test,
			   isp_size_from_crop(&source, (struct coord){ 640, 480 }, &result));
	source.crop_offset.x = U32_MAX;
	KUNIT_EXPECT_FALSE(test,
			   isp_size_from_crop(&source, (struct coord){ 640, 480 }, &result));
}

static void isp_test_call_size_table(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_preset source = {
		.index = 7, .input_dim = { 1920, 1920 },
		.output_dim = { 1920, 1920 }, .crop_size = { 1920, 1920 },
	};

	isp->presets = &source;
	isp->num_presets = 1;
	KUNIT_ASSERT_EQ(test, isp_add_call_sizes(isp), 0);
	KUNIT_EXPECT_EQ(test, isp->num_presets, ARRAY_SIZE(isp_call_sizes) + 1);
	KUNIT_EXPECT_EQ(test, isp->presets[0].output_dim.x, 1920);
	KUNIT_EXPECT_EQ(test, isp->presets[0].output_dim.y, 1920);
	for (unsigned int i = 0; i < ARRAY_SIZE(isp_call_sizes); i++) {
		struct isp_preset *preset = &isp->presets[i + 1];

		KUNIT_EXPECT_EQ(test, preset->output_dim.x, isp_call_sizes[i].x);
		KUNIT_EXPECT_EQ(test, preset->output_dim.y, isp_call_sizes[i].y);
		KUNIT_EXPECT_EQ(test, preset->index, 7);
		KUNIT_EXPECT_FALSE(test, preset->p010);
		KUNIT_EXPECT_EQ(test, preset->crop_size.x * preset->output_dim.y,
				preset->crop_size.y * preset->output_dim.x);
		KUNIT_EXPECT_EQ(test, (preset->crop_offset.x | preset->crop_offset.y |
				       preset->crop_size.x | preset->crop_size.y) & 1, 0);
		KUNIT_EXPECT_LE(test, preset->crop_offset.x + preset->crop_size.x, 1920);
		KUNIT_EXPECT_LE(test, preset->crop_offset.y + preset->crop_size.y, 1920);
	}
	/* Rebuilding preserves order and never duplicates an existing size. */
	KUNIT_ASSERT_EQ(test, isp_add_call_sizes(isp), 0);
	KUNIT_EXPECT_EQ(test, isp->num_presets, ARRAY_SIZE(isp_call_sizes) + 1);
}

static void isp_test_call_size_capabilities(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_preset sources[] = {
		{ .index = 5, .input_dim = { 1920, 2160 },
		  .output_dim = { 1920, 1080 }, .crop_size = { 1920, 1080 }, .p010 = true },
		{ .index = 5, .input_dim = { 1920, 2160 },
		  .output_dim = { 1280, 720 }, .crop_size = { 1280, 720 }, .p010 = true },
		{ .index = 9, .input_dim = { 1536, 3024 },
		  .output_dim = { 1536, 3024 }, .crop_size = { 1536, 3024 }, .p010 = true },
	};

	isp->presets = sources;
	isp->num_presets = ARRAY_SIZE(sources);
	KUNIT_ASSERT_EQ(test, isp_add_call_sizes(isp), 0);
	KUNIT_EXPECT_EQ(test, isp->num_presets, 9);
	for (unsigned int i = 0; i < ARRAY_SIZE(sources); i++)
		KUNIT_EXPECT_MEMEQ(test, &isp->presets[i], &sources[i], sizeof(sources[i]));
	for (unsigned int i = ARRAY_SIZE(sources); i < isp->num_presets; i++) {
		struct isp_preset *preset = &isp->presets[i];

		KUNIT_EXPECT_TRUE(test, preset->p010);
		KUNIT_EXPECT_FALSE(test, preset->output_dim.x == 1760 &&
				   preset->output_dim.y == 1328);
		KUNIT_EXPECT_FALSE(test, preset->output_dim.x == 1552 &&
				   preset->output_dim.y == 1552);
	}
	KUNIT_EXPECT_EQ(test, isp->presets[8].output_dim.x, 1536);
	KUNIT_EXPECT_EQ(test, isp->presets[8].output_dim.y, 1536);
}

static void isp_test_sensor_names(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_format *fmt = isp_get_current_format(isp);

	mutex_init(&isp->video_lock);
	for (unsigned int i = 0; i < ARRAY_SIZE(isp_setfiles); i++) {
		KUNIT_ASSERT_NOT_NULL_MSG(test, isp_setfiles[i].sensor,
					  "setfile %u", i);
		KUNIT_EXPECT_NE_MSG(test, isp_setfiles[i].sensor[0], '\0',
				    "setfile %u", i);
	}
	KUNIT_EXPECT_STREQ(test, isp_setfiles[ISP_IMX248_1820_01].sensor,
			   "imx248");
	KUNIT_EXPECT_STREQ(test, isp_setfiles[ISP_IMX558_1925_03].sensor,
			   "imx558");
	KUNIT_EXPECT_STREQ(test, isp_setfiles[ISP_IMX958_2224_01].sensor,
			   "imx958-uw");
	KUNIT_EXPECT_STREQ(test, isp_setfiles[ISP_VD56G0_6221_01].sensor,
			   "vd56g0");

	KUNIT_EXPECT_NULL(test, apple_isp_sensor_name(isp));
	fmt->id = ISP_IMX958_2224_01;
	isp->camera_detected = true;
	KUNIT_EXPECT_STREQ(test, apple_isp_sensor_name(isp), "imx958-uw");
}

static void isp_test_snf_flatten(u8 *snf)
{
	int k;

	for (k = 0; k < SNF_POINTS; k++)
		put_unaligned_le16(SNF_ONE, snf + 6 + 2 * k);
}

/*
 * A synthetic setfile: a 1024x1024 shading frame of four 3x3 tables at
 * 256-pixel steps, table t at (t + 1) x unity and twice that at the
 * centre point, then the SNF section. Expected gains from a port of the
 * computation: 16 / sum of the 16 interpolated gains, in Q12.
 */
static void isp_test_snf_radial(struct kunit *test)
{
	enum { GW = 3, GH = 3, LSC = 0x54 + 4 * GW * GH * 2,
	       SNF = 6 + 2 * SNF_POINTS };
	struct apple_isp *isp = isp_test_capture_alloc(test);
	size_t len = 0x14 + 12 + LSC + 12 + SNF;
	u8 *data = kunit_kzalloc(test, len, GFP_KERNEL);
	u8 *lsc, *snf;
	int t, k;

	KUNIT_ASSERT_NOT_NULL(test, data);
	lsc = data + 0x14 + 12;
	snf = lsc + LSC + 12;
	put_unaligned_le32(8 + LSC, lsc - 12);
	put_unaligned_le32(0xc003, lsc - 4);
	put_unaligned_le32(8 + SNF, snf - 12);
	put_unaligned_le32(0xc404, snf - 4);
	put_unaligned_le16(1024, lsc + 6);
	put_unaligned_le16(1024, lsc + 8);
	for (t = 0; t < 4; t++) {
		u8 *rec = lsc + 0xa + 18 * t;

		put_unaligned_le16(256, rec + 2);
		put_unaligned_le16(256, rec + 4);
		put_unaligned_le16(t * GW * GH, rec + 6);
		put_unaligned_le16(GW, rec + 14);
		put_unaligned_le16(GH, rec + 16);
		for (k = 0; k < GW * GH; k++)
			put_unaligned_le16(SNF_ONE * (t + 1) * (k == GW + 1 ? 2 : 1),
					   lsc + 0x54 + 2 * (t * GW * GH + k));
	}

	/* The corners are 724 pixels out: 4-pixel radius steps reach them. */
	put_unaligned_le16(7, snf + 4);
	isp_test_snf_flatten(snf);
	isp_setfile_snf_radial(isp, data, len, 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(snf + 4), 2);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(snf + 6), 819);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(snf + 6 + 2 * 128), 1509);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(snf + 6 + 2 * 256), 1638);
	for (k = 1; k < SNF_POINTS; k++)
		KUNIT_EXPECT_LE(test, get_unaligned_le16(snf + 6 + 2 * (k - 1)),
				get_unaligned_le16(snf + 6 + 2 * k));

	/* A table that does not ship flat is the setfile's own: kept. */
	isp_setfile_snf_radial(isp, data, len, -128);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(snf + 6), 819);

	/* A centre 128 pixels up moves the peak off the frame's centre. */
	isp_test_snf_flatten(snf);
	isp_setfile_snf_radial(isp, data, len, -128);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(snf + 6), 936);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(snf + 6 + 2 * 256), 1591);

	/*
	 * Tables whose size wraps 32 bits to 4 bytes (2 x 33025 x 65026),
	 * which fits the section, or that do not cover the frame, keep the
	 * gains there.
	 */
	isp_test_snf_flatten(snf);
	put_unaligned_le16(33025, lsc + 0xa + 18 * 3 + 14);
	put_unaligned_le16(65026, lsc + 0xa + 18 * 3 + 16);
	isp_setfile_snf_radial(isp, data, len, 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(snf + 6), SNF_ONE);
	put_unaligned_le16(GW, lsc + 0xa + 18 * 3 + 14);
	put_unaligned_le16(GH, lsc + 0xa + 18 * 3 + 16);
	put_unaligned_le16(128, lsc + 0xa + 18 * 3 + 2);
	isp_setfile_snf_radial(isp, data, len, 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(snf + 6), SNF_ONE);
}

/*
 * Camera indicator, best-effort: the DCP set(true) is requested before the
 * firmware stream, set(false) follows the stream and the watchdog, and the
 * camera streams whether or not the indicator came up. The DCP set wrapper
 * and the channel commands are stubbed; the real ones are never called.
 */
struct isp_test_indicator_trace {
	unsigned int order;
	unsigned int fw_start_at;
	unsigned int fw_stop_at;
	unsigned int set_on_at;
	unsigned int set_off_at;
	int fw_start_result;
	int set_on_result;
	struct platform_device *dcp_pdev;
	unsigned int lookup_calls;
};

static int isp_test_indicator_set(struct device *dcp, bool on)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_indicator_trace *trace = test->priv;

	if (on) {
		trace->set_on_at = ++trace->order;
		return trace->set_on_result;
	}
	trace->set_off_at = ++trace->order;
	return 0;
}

static struct platform_device *isp_test_dcp_lookup(struct device_node *np)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_indicator_trace *trace = test->priv;

	trace->lookup_calls++;
	return trace->dcp_pdev;
}

static int isp_test_fw_start(struct apple_isp *isp, u32 chan)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_indicator_trace *trace = test->priv;

	trace->fw_start_at = ++trace->order;
	return trace->fw_start_result;
}

static int isp_test_fw_stop(struct apple_isp *isp, u32 chan)
{
	struct kunit *test = kunit_get_current_test();
	struct isp_test_indicator_trace *trace = test->priv;

	trace->fw_stop_at = ++trace->order;
	return 0;
}

static int isp_test_fw_bufret(struct apple_isp *isp, u32 chan)
{
	return 0;
}

static struct apple_isp *isp_test_indicator_alloc(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);

	isp->dcp = isp->dev;
	return isp;
}

static void isp_test_indicator_set_fails_still_streams(struct kunit *test)
{
	struct apple_isp *isp = isp_test_indicator_alloc(test);
	struct isp_test_indicator_trace trace = { .set_on_result = -EIO };

	test->priv = &trace;
	kunit_activate_static_stub(test, apple_isp_dcp_indicator_set,
				   isp_test_indicator_set);
	kunit_activate_static_stub(test, isp_cmd_ch_start, isp_test_fw_start);
	KUNIT_EXPECT_EQ(test, apple_isp_start_capture(isp), 0);
	KUNIT_EXPECT_EQ(test, trace.set_on_at, 1);
	KUNIT_EXPECT_EQ(test, trace.fw_start_at, 2);
	KUNIT_EXPECT_EQ(test, trace.set_off_at, 0);
}

/*
 * The DCP may probe after the ISP: it is looked up when a stream starts. A DCP
 * that is not there yet means no indicator for that stream, and the next stream
 * finds it. A bound DCP is not looked up again.
 */
static void isp_test_dcp_bound_at_stream_start(struct kunit *test)
{
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_test_indicator_trace trace = {};
	struct device_node np = {};

	isp->dcp = NULL;
	isp->dcp_np = &np;
	test->priv = &trace;
	kunit_activate_static_stub(test, apple_isp_dcp_lookup, isp_test_dcp_lookup);
	kunit_activate_static_stub(test, apple_isp_dcp_indicator_set,
				   isp_test_indicator_set);
	kunit_activate_static_stub(test, isp_cmd_ch_start, isp_test_fw_start);

	/* The DCP has not probed: the stream runs without a dot. */
	KUNIT_EXPECT_EQ(test, apple_isp_start_capture(isp), 0);
	KUNIT_EXPECT_EQ(test, trace.lookup_calls, 1);
	KUNIT_EXPECT_NULL(test, isp->dcp);
	KUNIT_EXPECT_EQ(test, trace.set_on_at, 0);
	KUNIT_EXPECT_EQ(test, trace.fw_start_at, 1);

	/* It probed since: the next stream binds it and asks for the dot first. */
	trace.dcp_pdev = kunit_kzalloc(test, sizeof(*trace.dcp_pdev), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, trace.dcp_pdev);
	KUNIT_EXPECT_EQ(test, apple_isp_start_capture(isp), 0);
	KUNIT_EXPECT_EQ(test, trace.lookup_calls, 2);
	KUNIT_EXPECT_PTR_EQ(test, isp->dcp, &trace.dcp_pdev->dev);
	KUNIT_EXPECT_EQ(test, trace.set_on_at, 2);
	KUNIT_EXPECT_EQ(test, trace.fw_start_at, 3);

	/* Bound: no further lookup. */
	KUNIT_EXPECT_EQ(test, apple_isp_start_capture(isp), 0);
	KUNIT_EXPECT_EQ(test, trace.lookup_calls, 2);
}

static void isp_test_no_indicator_still_streams(struct kunit *test)
{
	struct apple_isp *isp = isp_test_indicator_alloc(test);
	struct isp_test_indicator_trace trace = {};

	isp->dcp = NULL;
	test->priv = &trace;
	kunit_activate_static_stub(test, isp_cmd_ch_start, isp_test_fw_start);
	KUNIT_EXPECT_EQ(test, apple_isp_start_capture(isp), 0);
	KUNIT_EXPECT_EQ(test, trace.fw_start_at, 1);
	KUNIT_EXPECT_EQ(test, trace.set_on_at, 0);
}

static void isp_test_indicator_on_precedes_stream(struct kunit *test)
{
	struct apple_isp *isp = isp_test_indicator_alloc(test);
	struct isp_test_indicator_trace trace = {};

	test->priv = &trace;
	kunit_activate_static_stub(test, apple_isp_dcp_indicator_set,
				   isp_test_indicator_set);
	kunit_activate_static_stub(test, isp_cmd_ch_start, isp_test_fw_start);
	KUNIT_EXPECT_EQ(test, apple_isp_start_capture(isp), 0);
	KUNIT_EXPECT_EQ(test, trace.set_on_at, 1);
	KUNIT_EXPECT_EQ(test, trace.fw_start_at, 2);

	kunit_activate_static_stub(test, isp_cmd_ch_stop, isp_test_fw_stop);
	kunit_activate_static_stub(test, isp_cmd_ch_buffer_return,
				   isp_test_fw_bufret);
	isp->fw_state = ISP_FW_RUNNING;
	apple_isp_stop_capture(isp);
	KUNIT_EXPECT_EQ(test, trace.fw_stop_at, 3);
	KUNIT_EXPECT_EQ(test, trace.set_off_at, 4);
}

static void isp_test_stream_failure_lights_off(struct kunit *test)
{
	struct apple_isp *isp = isp_test_indicator_alloc(test);
	struct isp_test_indicator_trace trace = { .fw_start_result = -ETIME };

	test->priv = &trace;
	kunit_activate_static_stub(test, apple_isp_dcp_indicator_set,
				   isp_test_indicator_set);
	kunit_activate_static_stub(test, isp_cmd_ch_start, isp_test_fw_start);
	KUNIT_EXPECT_EQ(test, apple_isp_start_capture(isp), -ETIME);
	KUNIT_EXPECT_EQ(test, trace.set_on_at, 1);
	KUNIT_EXPECT_EQ(test, trace.fw_start_at, 2);
	KUNIT_EXPECT_EQ(test, trace.set_off_at, 3);
}

static void isp_test_failed_fw_stop_lights_off(struct kunit *test)
{
	struct apple_isp *isp = isp_test_indicator_alloc(test);
	struct isp_test_indicator_trace trace = {};

	isp->ipc_failed = true;
	test->priv = &trace;
	kunit_activate_static_stub(test, apple_isp_dcp_indicator_set,
				   isp_test_indicator_set);
	kunit_activate_static_stub(test, isp_cmd_ch_stop, isp_test_fw_stop);
	apple_isp_stop_capture(isp);
	KUNIT_EXPECT_EQ(test, trace.fw_stop_at, 0);
	KUNIT_EXPECT_EQ(test, trace.set_off_at, 1);
}

static struct kunit_case isp_cam_test_cases[] = {
	KUNIT_CASE(isp_test_call_crop),
	KUNIT_CASE(isp_test_call_crop_bounds),
	KUNIT_CASE(isp_test_call_size_table),
	KUNIT_CASE(isp_test_call_size_capabilities),
	KUNIT_CASE(isp_test_setfile_failure_stops_setup),
	KUNIT_CASE(isp_test_optional_setfile_continues_setup),
	KUNIT_CASE(isp_test_command_ack_failures),
	KUNIT_CASE(isp_test_stop_key_on_debug),
	KUNIT_CASE(isp_test_sensor_geometry),
	KUNIT_CASE(isp_test_sensor_names),
	KUNIT_CASE(isp_test_sensor_config_list),
	KUNIT_CASE(isp_test_sensor_config_failure),
	KUNIT_CASE(isp_test_preview_stream),
	KUNIT_CASE(isp_test_preview_command_failures),
	KUNIT_CASE(isp_test_temporal_filter_band0),
	KUNIT_CASE(isp_test_temporal_filter_off),
	KUNIT_CASE(isp_test_power_line_frequency),
	KUNIT_CASE(isp_test_snf_radial),
	KUNIT_CASE(isp_test_indicator_set_fails_still_streams),
	KUNIT_CASE(isp_test_dcp_bound_at_stream_start),
	KUNIT_CASE(isp_test_no_indicator_still_streams),
	KUNIT_CASE(isp_test_indicator_on_precedes_stream),
	KUNIT_CASE(isp_test_stream_failure_lights_off),
	KUNIT_CASE(isp_test_failed_fw_stop_lights_off),
	{}
};

static struct kunit_suite isp_cam_test_suite = {
	.name = "apple-isp-camera",
	.test_cases = isp_cam_test_cases,
};

kunit_test_suite(isp_cam_test_suite);
