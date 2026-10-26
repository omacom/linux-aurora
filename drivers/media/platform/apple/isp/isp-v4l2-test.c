// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the videobuf2 queue of the Apple ISP driver, built into
 * isp-v4l2.c. They check the queue parameters vb2 accepts and the buffer
 * limit queue_setup() enforces, without any hardware.
 */

#include <kunit/device.h>
#include <kunit/test.h>
#include <kunit/static_stub.h>
#include <linux/bitmap.h>

#define ISP_TEST_FRAME_SIZE 0x1000

static struct apple_isp *isp_test_alloc(struct kunit *test)
{
	struct apple_isp *isp;

	isp = kunit_kzalloc(test, sizeof(*isp), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, isp);

	mutex_init(&isp->video_lock);
	spin_lock_init(&isp->buf_lock);
	INIT_LIST_HEAD(&isp->bufs_returned);
	isp_get_current_format(isp)->total_size = ISP_TEST_FRAME_SIZE;

	return isp;
}

/* The queue must initialise with the parameters the driver probes with. */
static void isp_test_queue_init(struct kunit *test)
{
	struct apple_isp *isp = isp_test_alloc(test);

	isp->dev = kunit_device_register(test, "apple-isp-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, isp->dev);

	KUNIT_ASSERT_EQ(test, isp_vb2_queue_init(isp), 0);
	KUNIT_EXPECT_GE(test, isp->vbq.max_num_buffers, VB2_MAX_FRAME);
	KUNIT_EXPECT_EQ(test, isp->vbq.type, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	KUNIT_EXPECT_FALSE(test, isp->vbq.is_multiplanar);
	vb2_queue_release(&isp->vbq);
}

/* Pretend the queue already has @num buffers, as vb2_get_num_buffers() sees. */
static void isp_test_set_num_buffers(struct kunit *test, struct vb2_queue *q,
				     unsigned int num)
{
	q->max_num_buffers = VB2_MAX_FRAME;
	q->bufs_bitmap = kunit_kzalloc(test,
				       BITS_TO_LONGS(VB2_MAX_FRAME) *
					       sizeof(unsigned long),
				       GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, q->bufs_bitmap);
	bitmap_set(q->bufs_bitmap, 0, num);
}

static int isp_test_setup(struct kunit *test, unsigned int existing,
			  unsigned int *nbuffers, unsigned int *num_planes,
			  unsigned int sizes[])
{
	struct apple_isp *isp = isp_test_alloc(test);

	isp->vbq.drv_priv = isp;
	isp_test_set_num_buffers(test, &isp->vbq, existing);

	return isp_vb2_queue_setup(&isp->vbq, nbuffers, num_planes, sizes,
				   NULL);
}

/* REQBUFS: an empty queue gets at most one pool of buffers. */
static void isp_test_reqbufs_limit(struct kunit *test)
{
	unsigned int sizes[VB2_MAX_PLANES] = {};
	unsigned int nbuffers = VB2_MAX_FRAME, num_planes = 0;

	KUNIT_ASSERT_EQ(test, isp_test_setup(test, 0, &nbuffers, &num_planes,
					     sizes), 0);
	KUNIT_EXPECT_EQ(test, nbuffers, ISP_MAX_BUFFERS);
	KUNIT_EXPECT_EQ(test, num_planes, 1);
	KUNIT_EXPECT_EQ(test, sizes[0], ISP_TEST_FRAME_SIZE);

	nbuffers = 4;
	num_planes = 0;
	KUNIT_ASSERT_EQ(test, isp_test_setup(test, 0, &nbuffers, &num_planes,
					     sizes), 0);
	KUNIT_EXPECT_EQ(test, nbuffers, 4);
}

/* CREATE_BUFS: the buffers the queue has count against the pool. */
static void isp_test_create_bufs_limit(struct kunit *test)
{
	unsigned int sizes[VB2_MAX_PLANES] = { ISP_TEST_FRAME_SIZE };
	unsigned int nbuffers, num_planes;

	nbuffers = 10;
	num_planes = 1;
	KUNIT_ASSERT_EQ(test, isp_test_setup(test, 10, &nbuffers, &num_planes,
					     sizes), 0);
	KUNIT_EXPECT_EQ(test, nbuffers, ISP_MAX_BUFFERS - 10);

	nbuffers = 1;
	num_planes = 1;
	KUNIT_EXPECT_EQ(test, isp_test_setup(test, ISP_MAX_BUFFERS, &nbuffers,
					     &num_planes, sizes), -ENOBUFS);

	/* The contiguous API must refuse a second independent memory plane. */
	nbuffers = 1;
	num_planes = 2;
	KUNIT_EXPECT_EQ(test, isp_test_setup(test, 0, &nbuffers, &num_planes,
					     sizes), -EINVAL);

	/* A plane smaller than a frame is still refused. */
	nbuffers = 1;
	num_planes = 1;
	sizes[0] = ISP_TEST_FRAME_SIZE - 1;
	KUNIT_EXPECT_EQ(test, isp_test_setup(test, 0, &nbuffers, &num_planes,
					     sizes), -EINVAL);
}

static const struct apple_isp_hw isp_test_t6040 = {
	.gen = ISP_GEN_T6040,
	.fw_abi = ISP_FW_ABI_H17,
};

static void isp_test_p010_format(struct kunit *test)
{
	struct apple_isp *isp = isp_test_alloc(test);
	struct isp_format *fmt = isp_get_current_format(isp);
	struct isp_preset preset = { .output_dim = { 1920, 1080 }, .p010 = true };
	struct v4l2_format f = {};
	unsigned int sizes[VB2_MAX_PLANES] = {};
	unsigned int nbuffers = 16, num_planes = 0;

	isp->hw = &isp_test_t6040;
	fmt->version = 0x958;
	KUNIT_ASSERT_EQ(test, isp_set_preset(isp, fmt, &preset, V4L2_PIX_FMT_P010), 0);
	isp_get_sp_pix_format(isp, &f, fmt);
	KUNIT_EXPECT_EQ(test, f.fmt.pix.pixelformat, V4L2_PIX_FMT_P010);
	KUNIT_EXPECT_EQ(test, f.fmt.pix.bytesperline, 3840);
	KUNIT_EXPECT_EQ(test, f.fmt.pix.sizeimage, 6220800);
	KUNIT_EXPECT_EQ(test, fmt->plane_size[0], (size_t)4147200);
	KUNIT_EXPECT_EQ(test, fmt->plane_size[1], (size_t)2073600);
	KUNIT_EXPECT_EQ(test, f.fmt.pix.quantization, V4L2_QUANTIZATION_LIM_RANGE);

	isp->vbq.drv_priv = isp;
	isp_test_set_num_buffers(test, &isp->vbq, 0);
	KUNIT_ASSERT_EQ(test,
			isp_vb2_queue_setup(&isp->vbq, &nbuffers, &num_planes, sizes, NULL), 0);
	KUNIT_EXPECT_EQ(test, nbuffers, 2);
	KUNIT_EXPECT_EQ(test, num_planes, 1);
	KUNIT_EXPECT_EQ(test, sizes[0], 6220800);
	KUNIT_EXPECT_EQ(test, isp_num_meta(isp), 8);

	preset.output_dim = (struct coord){1280, 720};
	KUNIT_ASSERT_EQ(test, isp_set_preset(isp, fmt, &preset, V4L2_PIX_FMT_P010), 0);
	KUNIT_EXPECT_EQ(test, fmt->strides[0], 2560);
	KUNIT_EXPECT_EQ(test, fmt->total_size, (size_t)2764800);
	/* NV12 remains the default and restores the existing pool capacity. */
	KUNIT_ASSERT_EQ(test, isp_set_preset(isp, fmt, &preset, V4L2_PIX_FMT_NV12), 0);
	KUNIT_EXPECT_EQ(test, fmt->pixelformat, V4L2_PIX_FMT_NV12);
	KUNIT_EXPECT_EQ(test, fmt->total_size, (size_t)1382400);
	KUNIT_EXPECT_EQ(test, isp_num_rendered(isp), ISP_MAX_BUFFERS);
	KUNIT_EXPECT_EQ(test, isp_num_meta(isp), ISP_MAX_BUFFERS);

}

static void isp_test_p010_admission(struct kunit *test)
{
	static const struct apple_isp_hw neo = { .gen = ISP_GEN_T8140 };
	struct apple_isp *isp = isp_test_alloc(test);
	struct isp_format *fmt = isp_get_current_format(isp);
	struct isp_preset preset = { .output_dim = {1280, 720}, .p010 = true };
	struct v4l2_fmtdesc desc = { .index = 0 };

	isp->presets = &preset;
	isp->num_presets = 1;
	isp->hw = &isp_test_t6040;
	fmt->version = 0x958;
	KUNIT_ASSERT_EQ(test, isp_enum_pixelformat(isp, &desc), 0);
	KUNIT_EXPECT_EQ(test, desc.pixelformat, V4L2_PIX_FMT_NV12);
	desc.index = 1;
	KUNIT_ASSERT_EQ(test, isp_enum_pixelformat(isp, &desc), 0);
	KUNIT_EXPECT_EQ(test, desc.pixelformat, V4L2_PIX_FMT_P010);
	desc.index = 2;
	KUNIT_EXPECT_EQ(test, isp_enum_pixelformat(isp, &desc), -EINVAL);
	desc.index = 1;
	isp->hw = &neo;
	KUNIT_EXPECT_EQ(test, isp_enum_pixelformat(isp, &desc), -EINVAL);
	KUNIT_ASSERT_EQ(test, isp_set_preset(isp, fmt, &preset, V4L2_PIX_FMT_P010), 0);
	KUNIT_EXPECT_EQ(test, fmt->pixelformat, V4L2_PIX_FMT_NV12);
	KUNIT_EXPECT_EQ(test, fmt->strides[0], 1280);
}

static void isp_test_p010_completion_identity(struct kunit *test)
{
	struct isp_format fmt = { .plane_size = {4147200, 2073600} };
	struct isp_buffer buf = { .surfs = {{ .iova = 0x10020000000ULL }}, .tag = 42 };
	struct isp_buflist_buffer desc = {
		.iovas = {0x20000000, 0x203f4800},
		.num_planes = 2, .pool_type = CISP_POOL_TYPE_RENDERED_SCL1, .tag = 42,
	};

	KUNIT_EXPECT_TRUE(test, isp_p010_buffer_matches(&fmt, &buf, &desc));
	desc.tag--;
	KUNIT_EXPECT_FALSE(test, isp_p010_buffer_matches(&fmt, &buf, &desc));
	desc.tag++;
	desc.iovas[1]++;
	KUNIT_EXPECT_FALSE(test, isp_p010_buffer_matches(&fmt, &buf, &desc));
	desc.iovas[1]--;
	desc.iovas[0] |= BIT_ULL(40);
	/* Reject mixed encodings, accept both exact full submitted addresses. */
	KUNIT_EXPECT_FALSE(test, isp_p010_buffer_matches(&fmt, &buf, &desc));
	desc.iovas[1] |= BIT_ULL(40);
	KUNIT_EXPECT_TRUE(test, isp_p010_buffer_matches(&fmt, &buf, &desc));
	desc.iovas[0] |= BIT_ULL(41);
	desc.iovas[1] |= BIT_ULL(41);
	KUNIT_EXPECT_FALSE(test, isp_p010_buffer_matches(&fmt, &buf, &desc));
	desc.iovas[0] = (u32)desc.iovas[0];
	desc.iovas[1] = (u32)desc.iovas[1];
	desc.pool_type = CISP_POOL_TYPE_RENDERED;
	KUNIT_EXPECT_FALSE(test, isp_p010_buffer_matches(&fmt, &buf, &desc));
	desc.pool_type = CISP_POOL_TYPE_RENDERED_SCL1;
	desc.num_planes = 1;
	KUNIT_EXPECT_FALSE(test, isp_p010_buffer_matches(&fmt, &buf, &desc));
}

static void isp_test_p010_report_defers_completion(struct kunit *test)
{
	struct apple_isp *isp = isp_test_alloc(test);
	struct isp_format *fmt = isp_get_current_format(isp);
	struct isp_buffer *buf = kunit_kzalloc(test, sizeof(*buf), GFP_KERNEL);
	struct {
		u64 type, count;
		struct isp_buflist_buffer desc;
	} report = {
		.type = 2, .count = 1,
		.desc = { .iovas = {0x20000000, 0x203f4800},
			  .num_planes = 2, .pool_type = 9, .tag = 42 },
	};
	struct isp_surf report_surf = {
		.iova = 0x30000000, .size = sizeof(report), .virt = &report,
	};
	struct isp_channel chan = {
		.req = { .arg0 = report_surf.iova, .arg1 = sizeof(report),
			 .arg2 = ISP_IPC_BUFEXC_FLAG_RENDER },
	};

	KUNIT_ASSERT_NOT_NULL(test, buf);
	fmt->pixelformat = V4L2_PIX_FMT_P010;
	fmt->plane_size[0] = 4147200;
	isp->fw_iova_mask = U64_MAX;
	isp->bt_surf = &report_surf;
	buf->surfs[0].iova = 0x10020000000ULL;
	buf->tag = 42;
	INIT_LIST_HEAD(&isp->bufs_submitted);
	list_add(&buf->link, &isp->bufs_submitted);
	/* There is no vb2 queue: completing it before the ACK would fault. */
	KUNIT_ASSERT_EQ(test, ipc_bt_handle(isp, &chan), 0);
	KUNIT_EXPECT_TRUE(test, list_empty(&isp->bufs_submitted));
	KUNIT_EXPECT_PTR_EQ(test,
			    list_first_entry(&isp->bufs_returned, struct isp_buffer, link), buf);
	KUNIT_EXPECT_EQ(test, buf->return_state, VB2_BUF_STATE_DONE);
	KUNIT_EXPECT_EQ(test, chan.rsp.arg0, report_surf.iova | ISP_IPC_FLAG_ACK);
}

static void isp_test_p010_mode_selection(struct kunit *test)
{
	struct apple_isp *isp = isp_test_alloc(test);
	struct isp_preset presets[] = {
		{ .index = 5, .output_dim = {1920, 1080} },
		{ .index = 5, .output_dim = {1280, 720}, .p010 = true },
		{ .index = 1, .output_dim = {4096, 3072} },
	};

	isp->hw = &isp_test_t6040;
	isp_get_current_format(isp)->version = 0x958;
	isp->presets = presets;
	isp->num_presets = ARRAY_SIZE(presets);
	KUNIT_EXPECT_PTR_EQ(test, isp_select_preset(isp, 0, 0, V4L2_PIX_FMT_P010), &presets[1]);
	KUNIT_EXPECT_PTR_EQ(test, isp_select_preset(isp, 1920, 1080, V4L2_PIX_FMT_P010),
			    &presets[1]);
	KUNIT_EXPECT_PTR_EQ(test, isp_select_preset(isp, 4096, 3072, V4L2_PIX_FMT_P010),
			    &presets[1]);
	KUNIT_EXPECT_PTR_EQ(test, isp_select_preset(isp, 1920, 1080, V4L2_PIX_FMT_NV12),
			    &presets[0]);
	/* Only a matching firmware output enables the larger P010 size. */
	presets[2].p010 = true;
	KUNIT_EXPECT_PTR_EQ(test, isp_select_preset(isp, 4096, 3072, V4L2_PIX_FMT_P010),
			    &presets[2]);
}

static int isp_test_buffer_ack(struct apple_isp *isp, struct isp_channel *chan,
			       unsigned long timeout)
{
	unsigned int *calls = kunit_get_current_test()->priv;

	(*calls)++;
	return 0;
}

static void isp_test_imx958_last_lease_requeue(struct kunit *test)
{
	struct apple_isp *isp = isp_test_alloc(test);
	struct isp_format *fmt = isp_get_current_format(isp);
	struct isp_buffer *buf = kunit_kzalloc(test, sizeof(*buf), GFP_KERNEL);
	struct isp_buffer *submitted;
	unsigned int calls = 0;

	KUNIT_ASSERT_NOT_NULL(test, buf);
	isp->hw = &isp_test_t6040;
	fmt->version = 0x958;
	fmt->pixelformat = V4L2_PIX_FMT_P010;
	fmt->num_planes = 2;
	fmt->plane_size[0] = 1843200;
	fmt->plane_size[1] = 921600;
	INIT_LIST_HEAD(&isp->bufs_pending);
	INIT_LIST_HEAD(&isp->bufs_submitted);
	isp->cmd_virt = kunit_kzalloc(test, ISP_CMD_AREA_SIZE(0), GFP_KERNEL);
	isp->chan_bh = kunit_kzalloc(test, sizeof(*isp->chan_bh), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, isp->cmd_virt);
	KUNIT_ASSERT_NOT_NULL(test, isp->chan_bh);
	for (unsigned int i = 0; i < ISP_MAX_BUFFERS; i++) {
		isp->meta_surfs[i] = kunit_kzalloc(test, sizeof(struct isp_surf), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, isp->meta_surfs[i]);
		isp->meta_surfs[i]->submitted = true;
	}
	isp->vbq.drv_priv = isp;
	buf->vb.vb2_buf.vb2_queue = &isp->vbq;
	buf->surfs[0].iova = 0x20000000;
	set_bit(ISP_STATE_STREAMING, &isp->state);
	test->priv = &calls;
	kunit_activate_static_stub(test, ipc_chan_send, isp_test_buffer_ack);
	for (unsigned int format = 0; format < 2; format++) {
		fmt->pixelformat = format ? V4L2_PIX_FMT_NV12 : V4L2_PIX_FMT_P010;
		calls = 0;
		for (unsigned int i = 1; i <= 10000; i++) {
			/* Both lists empty, as after the last completion was dequeued. */
			isp_vb2_buf_queue(&buf->vb.vb2_buf);
			KUNIT_ASSERT_EQ(test, calls, i);
			if (!format)
				KUNIT_ASSERT_EQ(test, buf->tag, (u64)i);
			KUNIT_ASSERT_TRUE(test, list_empty(&isp->bufs_pending));
			submitted = list_first_entry(&isp->bufs_submitted, struct isp_buffer, link);
			KUNIT_ASSERT_PTR_EQ(test, submitted, buf);
			list_del(&buf->link);
		}
	}
}

static struct kunit_case isp_v4l2_test_cases[] = {
	KUNIT_CASE(isp_test_imx958_last_lease_requeue),
	KUNIT_CASE(isp_test_p010_report_defers_completion),
	KUNIT_CASE(isp_test_p010_mode_selection),
	KUNIT_CASE(isp_test_p010_format),
	KUNIT_CASE(isp_test_p010_admission),
	KUNIT_CASE(isp_test_p010_completion_identity),
	KUNIT_CASE(isp_test_queue_init),
	KUNIT_CASE(isp_test_reqbufs_limit),
	KUNIT_CASE(isp_test_create_bufs_limit),
	{}
};

static struct kunit_suite isp_v4l2_test_suite = {
	.name = "apple-isp-v4l2",
	.test_cases = isp_v4l2_test_cases,
};

kunit_test_suite(isp_v4l2_test_suite);
