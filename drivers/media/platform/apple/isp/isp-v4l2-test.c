// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the videobuf2 queue of the Apple ISP driver, built into
 * isp-v4l2.c. They check queue parameters, buffer limits and suspended
 * return ownership and error wakeups without any hardware.
 */

#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/bitmap.h>
#include <linux/completion.h>
#include <linux/kthread.h>

#define ISP_TEST_FRAME_SIZE 0x1000

static struct apple_isp *isp_test_alloc(struct kunit *test)
{
	struct apple_isp *isp;

	isp = kunit_kzalloc(test, sizeof(*isp), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, isp);

	mutex_init(&isp->video_lock);
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

	/* A plane smaller than a frame is still refused. */
	nbuffers = 1;
	num_planes = 1;
	sizes[0] = ISP_TEST_FRAME_SIZE - 1;
	KUNIT_EXPECT_EQ(test, isp_test_setup(test, 0, &nbuffers, &num_planes,
					     sizes), -EINVAL);
}

/* Returned suspended buffers remain queued, but no longer outstanding. */
static void isp_test_suspended_return(struct kunit *test)
{
	struct apple_isp *isp = isp_test_alloc(test);
	struct apple_isp_hw hw = {};
	struct isp_surf surface = {};
	struct isp_channel chan = {};
	struct isp_buffer *buf;
	struct isp_buflist *bl;

	isp->hw = &hw;
	isp->fw_iova_mask = U64_MAX;
	spin_lock_init(&isp->buf_lock);
	INIT_LIST_HEAD(&isp->bufs_pending);
	INIT_LIST_HEAD(&isp->bufs_submitted);
	buf = kunit_kzalloc(test, sizeof(*buf), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, buf);
	buf->surfs[0].iova = 0x300000;
	list_add_tail(&buf->link, &isp->bufs_submitted);
	bl = kunit_kzalloc(test, sizeof(*bl) + sizeof(bl->buffers[0]), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, bl);
	bl->num_buffers = 1;
	bl->buffers[0].iovas[0] = buf->surfs[0].iova;
	bl->buffers[0].pool_type = CISP_POOL_TYPE_RENDERED;
	surface.iova = 0x1000;
	surface.virt = bl;
	surface.size = sizeof(*bl) + sizeof(bl->buffers[0]);
	isp->bt_surf = &surface;
	chan.req.arg0 = surface.iova;
	chan.req.arg1 = surface.size;
	set_bit(ISP_STATE_SLEEPING, &isp->state);
	KUNIT_EXPECT_EQ(test, ipc_bt_handle(isp, &chan), 0);
	KUNIT_EXPECT_TRUE(test, list_empty(&isp->bufs_submitted));
	KUNIT_EXPECT_PTR_EQ(test, isp->bufs_pending.next, &buf->link);
	KUNIT_EXPECT_EQ(test, isp->sequence, 0);
}

struct isp_test_wait {
	struct vb2_queue *queue;
	struct completion entered;
	struct completion exited;
	long remaining;
};

static int isp_test_queue_waiter(void *arg)
{
	struct isp_test_wait *w = arg;
	DEFINE_WAIT(wait);

	prepare_to_wait(&w->queue->done_wq, &wait, TASK_UNINTERRUPTIBLE);
	complete(&w->entered);
	w->remaining = schedule_timeout(5 * HZ);
	finish_wait(&w->queue->done_wq, &wait);
	complete(&w->exited);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(HZ);
	return 0;
}

/* Use the real vb2 error wakeup, waitqueue and startup notification lock. */
static void isp_test_error_wakeup(struct kunit *test)
{
	struct apple_isp *isp = isp_test_alloc(test);
	struct isp_test_wait w = { .queue = &isp->vbq };
	struct task_struct *task;

	spin_lock_init(&isp->buf_lock);
	init_waitqueue_head(&isp->vbq.done_wq);
	isp->ipc_error = -EIO;
	apple_isp_video_error(isp);
	KUNIT_EXPECT_FALSE(test, isp->vbq.error);
	set_bit(ISP_STATE_STREAMING, &isp->state);
	isp->ipc_error = 0;
	apple_isp_video_error(isp);
	KUNIT_EXPECT_FALSE(test, isp->vbq.error);
	init_completion(&w.entered);
	init_completion(&w.exited);
	task = kthread_run(isp_test_queue_waiter, &w, "isp-test-wait");
	KUNIT_ASSERT_FALSE(test, IS_ERR(task));
	wait_for_completion(&w.entered);
	isp->ipc_error = -EIO;
	apple_isp_video_error(isp);
	wait_for_completion(&w.exited);
	kthread_stop(task);
	KUNIT_EXPECT_TRUE(test, isp->vbq.error);
	KUNIT_EXPECT_GT(test, w.remaining, 0L);
}

static struct kunit_case isp_v4l2_test_cases[] = {
	KUNIT_CASE(isp_test_suspended_return),
	KUNIT_CASE(isp_test_error_wakeup),
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
