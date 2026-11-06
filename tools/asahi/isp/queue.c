// SPDX-License-Identifier: GPL-2.0-only
/* BUFFER FIXTURE */
enum vb2_buffer_state { VB2_BUF_STATE_ERROR, VB2_BUF_STATE_DONE };
#define ISP_STATE_STREAMING 0

static unsigned int completed, stopped;

static struct apple_isp *vb2_get_drv_priv(struct vb2_queue *q)
{
	return q->priv;
}

static void vb2_queue_error(struct vb2_queue *q)
{
	q->error = true;
}

static void vb2_buffer_done(struct vb2_buffer *vb, enum vb2_buffer_state state)
{
	CHECK(quiescent && state == VB2_BUF_STATE_ERROR && !vb->done);
	vb->done++;
	completed++;
}

static void apple_isp_stop_capture(struct apple_isp *isp)
{
	stopped++;
}

static void v4l2_ctrl_grab(void *ctrl, bool grabbed)
{
	CHECK(!grabbed);
}

/* DRIVER FUNCTIONS */
static void receive_error(struct apple_isp *isp)
{
	isp->ipc_error = -EIO;
	apple_isp_video_error(isp);
}

int main(void)
{
	struct hardware hw = { 0 };
	struct isp_channel chan = { 0 };
	struct isp_surf metadata[ISP_MAX_BUFFERS];
	struct isp_buffer rendered[3] = { 0 };
	unsigned char area[ISP_CMD_AREA_SIZE(0)];
	struct apple_isp isp = {
		.hw = &hw,
		.chan_bh = &chan,
		.cmd_virt = area,
		.fmt = { .num_planes = 2, .plane_size = { 4096, 2048 } },
		.state = 1UL << ISP_STATE_STREAMING,
	};

	isp.vbq.priv = &isp;
	INIT_LIST_HEAD(&isp.bufs_pending);
	INIT_LIST_HEAD(&isp.bufs_submitted);
	for (unsigned int i = 0; i < ARRAY_SIZE(metadata); i++) {
		metadata[i] =
			(struct isp_surf){ .iova = 0x100000000ULL + i * 4096 };
		isp.meta_surfs[i] = &metadata[i];
	}
	for (unsigned int i = 0; i < ARRAY_SIZE(rendered); i++) {
		rendered[i].surfs[0].iova = 0x300000000ULL + i * 8192;
		rendered[i].vb.vb2_buf.vb2_queue = &isp.vbq;
	}
	list_add_tail(&rendered[0].link, &isp.bufs_submitted);
	send_error = -ETIME;
	mutate = true;
	isp_vb2_buf_queue(&rendered[1].vb.vb2_buf);
	CHECK(isp.vbq.error && !completed && sends == 1);
	isp_vb2_buf_queue(&rendered[2].vb.vb2_buf);
	CHECK(isp.vbq.error && !completed && sends == 1);
	isp_vb2_stop_streaming(&isp.vbq);
	CHECK(quiescent && stopped == 1 && completed == 3);
	CHECK(list_empty(&isp.bufs_pending) && list_empty(&isp.bufs_submitted));
	isp_vb2_stop_streaming(&isp.vbq);
	CHECK(completed == 3 && stopped == 1);
	/* A receive error before or during startup must not poison vb2. */
	for (unsigned int when = 0; when < 3; when++) {
		isp.state = 0;
		isp.vbq.error = false;
		isp.ipc_error = 0;
		transition_error = when == 1 ? receive_error : NULL;
		send_error = 0;
		mutate = false;
		if (!when) {
			receive_error(&isp);
			CHECK(!isp.vbq.error);
			quiescent = true;
		}
		int err = apple_isp_start_streaming(&isp);

		CHECK(err == (when == 1 ? -EIO : 0));
		CHECK(!isp.vbq.error);
		CHECK(test_bit(ISP_STATE_STREAMING, &isp.state) == (when != 1));
		if (when == 2) {
			receive_error(&isp);
			CHECK(isp.vbq.error);
		}
		apple_isp_stop_camera(&isp);
	}
	puts("PASS: queue error wakeup and shutdown before error completion on close");
	return 0;
}
