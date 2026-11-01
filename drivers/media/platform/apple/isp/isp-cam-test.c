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

static void isp_test_optional_setfile_continues_setup(struct kunit *test)
{
	static const int results[] = { 0, -ENOENT };
	struct apple_isp *isp = isp_test_capture_alloc(test);
	struct isp_test_command_trace trace;

	isp_get_current_format(isp)->pixelformat = V4L2_PIX_FMT_NV12;
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

static struct kunit_case isp_cam_test_cases[] = {
	KUNIT_CASE(isp_test_call_crop),
	KUNIT_CASE(isp_test_call_crop_bounds),
	KUNIT_CASE(isp_test_call_size_table),
	KUNIT_CASE(isp_test_call_size_capabilities),
	KUNIT_CASE(isp_test_setfile_failure_stops_setup),
	KUNIT_CASE(isp_test_optional_setfile_continues_setup),
	KUNIT_CASE(isp_test_command_ack_failures),
	KUNIT_CASE(isp_test_sensor_geometry),
	KUNIT_CASE(isp_test_sensor_config_list),
	KUNIT_CASE(isp_test_sensor_config_failure),
	KUNIT_CASE(isp_test_preview_stream),
	KUNIT_CASE(isp_test_preview_command_failures),
	KUNIT_CASE(isp_test_temporal_filter_band0),
	KUNIT_CASE(isp_test_temporal_filter_off),
	{}
};

static struct kunit_suite isp_cam_test_suite = {
	.name = "apple-isp-camera",
	.test_cases = isp_cam_test_cases,
};

kunit_test_suite(isp_cam_test_suite);
