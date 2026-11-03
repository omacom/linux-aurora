// SPDX-License-Identifier: GPL-2.0-only
/* BUFFER FIXTURE */
enum vb2_buffer_state { VB2_BUF_STATE_ERROR, VB2_BUF_STATE_DONE };
#define V4L2_FIELD_NONE 0

static bool allocation_failure, change_after_copy;
static unsigned int completions;
static enum vb2_buffer_state expected_state = VB2_BUF_STATE_DONE;
static struct isp_buflist *shared;

static void *kmemdup(const void *p, size_t size, int flags)
{
	void *copy;

	if (allocation_failure)
		return NULL;
	copy = malloc(size ? size : 1);
	CHECK(copy);
	memcpy(copy, p, size);
	if (change_after_copy)
		memset(shared, 0xff, 16 + size);
	return copy;
}

static void *apple_isp_translate(struct apple_isp *isp, struct isp_surf *surf,
				 u64 addr, u64 size)
{
	if (addr < surf->iova || addr - surf->iova > surf->size ||
	    size > surf->size - (addr - surf->iova))
		return NULL;
	return (char *)surf->virt + (addr - surf->iova);
}

static u64 ktime_get_ns(void)
{
	return 1234;
}

static void vb2_buffer_done(struct vb2_buffer *buf, enum vb2_buffer_state state)
{
	CHECK(state == expected_state);
	CHECK(!buf->done);
	buf->done++;
	completions++;
}

static void vb2_queue_error(struct vb2_queue *q)
{
	q->error = true;
}

/* DRIVER FUNCTIONS */
static struct isp_channel *return_channel;

static int suspend_return(struct apple_isp *isp)
{
	int err = ipc_bt_handle(isp, return_channel);

	if (err)
		isp->ipc_error = err;
	return err;
}

int main(void)
{
	unsigned char memory[8192] = { 0 };
	struct isp_surf surface = { .iova = 0x1000,
				    .size = sizeof(memory),
				    .virt = memory };
	struct isp_surf metadata[ISP_MAX_BUFFERS], capmeta[ISP_CAPMETA_BUFFERS];
	struct isp_buffer rendered[ISP_MAX_BUFFERS] = { 0 };
	struct hardware hw = { .capture_meta_size = 1 };
	struct isp_channel chan = { 0 };
	struct apple_isp isp = { .hw = &hw,
				 .bt_surf = &surface,
				 .fw_iova_mask = UINT64_MAX };
	struct isp_buflist *bl = (void *)memory;

	shared = bl;
	for (unsigned int test = 0; test < 27; test++) {
		bool valid = test == 0 || test == 11 || test == 12 ||
			     test == 13 || test == 16 || test == 1 || test == 2 ||
			     test == 5 || test == 18 || test == 19 || test == 20 ||
			     test == 21 || test == 22 || test == 24;
		unsigned int before = completions;
		u64 count = 1;

		INIT_LIST_HEAD(&isp.bufs_pending);
		INIT_LIST_HEAD(&isp.bufs_submitted);
		memset(memory, 0, sizeof(memory));
		isp.ipc_error = 0;
		isp.sequence = 0;
		hw.scl1 = false;
		hw.capture_meta_size = 1;
		isp.fw_iova_mask = UINT64_MAX;
		allocation_failure = test == 10;
		change_after_copy = test == 13;
		for (unsigned int i = 0; i < ARRAY_SIZE(rendered); i++) {
			metadata[i] =
				(struct isp_surf){ .iova = 0x100100000ULL +
							   i * 4096,
						   .submitted = true };
			isp.meta_surfs[i] = &metadata[i];
			rendered[i].surfs[0].iova = 0x300300000ULL + i * 8192;
			rendered[i].vb.vb2_buf.done = 0;
			list_add_tail(&rendered[i].link, &isp.bufs_submitted);
		}
		for (unsigned int i = 0; i < ARRAY_SIZE(capmeta); i++) {
			capmeta[i] = (struct isp_surf){ .iova = 0x200200000ULL +
								i * 4096,
							.submitted = true };
			isp.capmeta_surfs[i] = &capmeta[i];
		}
		bl->buffers[0].pool_type = CISP_POOL_TYPE_RENDERED;
		bl->buffers[0].iovas[0] = rendered[0].surfs[0].iova;
		chan.req = (struct isp_message){
			.arg0 = surface.iova,
			.arg1 = 80,
			.arg2 = ISP_IPC_BUFEXC_FLAG_RENDER
		};
		if (test == 1)
			bl->buffers[0].iovas[0] += 1ULL << 32;
		if (test == 2)
			bl->buffers[0].pool_type = 99;
		if (test == 3) {
			count = 2;
			chan.req.arg1 += 64;
			bl->buffers[1] = bl->buffers[0];
		}
		if (test == 4) {
			list_del(&rendered[0].link);
			list_add_tail(&rendered[0].link, &isp.bufs_pending);
		}
		if (test == 5)
			bl->buffers[0].pool_type = CISP_POOL_TYPE_RENDERED_SCL1;
		if (test == 6)
			chan.req.arg1 = 15;
		if (test == 7)
			count = UINT64_MAX;
		if (test == 8)
			chan.req.arg0 += sizeof(memory) - 1;
		if (test == 9)
			isp.ipc_error = -ETIME;
		if (test == 11) {
			isp.fw_iova_mask = (1ULL << 36) - 1;
			bl->buffers[0].iovas[0] |= 1ULL << 40;
		}
		if (test == 12) {
			count = 0;
			chan.req.arg1 = 16;
		}
		if (test == 14 || test == 15) {
			bl->buffers[0].pool_type =
				test == 14 ? CISP_POOL_TYPE_META :
					     ISP_BUFLIST_POOL_CAPTURE_META;
			bl->buffers[0].iovas[0] =
				test == 14 ? metadata[0].iova : capmeta[0].iova;
			metadata[0].submitted = false;
			hw.capture_meta_size = 0;
		}
		if (test == 16 || test == 17) {
			count = test == 17 ? 41 : 40;
			chan.req.arg1 = 16 + count * 64;
			for (unsigned int i = 0; i < 40; i++) {
				bl->buffers[i].pool_type = i < 16 ? 0 :
							   i < 24 ? 2 :
								    1;
				bl->buffers[i].iovas[0] =
					i < 16 ? metadata[i].iova :
					i < 24 ? capmeta[i - 16].iova :
						 rendered[i - 24].surfs[0].iova;
			}
		}
		if (test == 18)
			chan.req.arg2 = 0;
		if (test >= 19 && test <= 22) {
			hw.resident_fw = test >= 21;
			isp.state = 1UL << ISP_STATE_STREAMING;
			bl->buffers[1].pool_type = CISP_POOL_TYPE_META;
			bl->buffers[1].iovas[0] = metadata[0].iova;
			if (!(test & 1)) {
				count = 2;
				chan.req.arg1 += 64;
			}
			bl->num_buffers = count;
			return_hook = suspend_return;
			return_channel = &chan;
			unsigned int completed_before = completions;

			CHECK(!apple_isp_video_suspend(&isp));
			CHECK(!isp.ipc_error && completions == completed_before);
			CHECK(list_empty(&isp.bufs_submitted));
			CHECK(!list_empty(&isp.bufs_pending));
			return_hook = NULL;
			isp.cmd_virt = memory;
			isp.chan_bh = &chan;
			CHECK(!apple_isp_video_resume(&isp));
			CHECK(test_bit(ISP_STATE_STREAMING, &isp.state));
			CHECK(!list_empty(&isp.bufs_submitted));
			CHECK(!isp.vbq.error && completions == completed_before);
			isp.state = 0;
			continue;
		}
		if (test == 24) {
			hw.capture_meta_size = 0;
			bl->buffers[0].pool_type = ISP_BUFLIST_POOL_CAPTURE_META;
		}
		if (test == 23)
			bl->buffers[0].iovas[0] += 123;
		/* Reject the entire list even when its prefix is outstanding. */
		if (test >= 25) {
			count = test == 25 ? 2 : 4;
			chan.req.arg1 = 16 + count * 64;
			bl->buffers[count - 1] = bl->buffers[0];
			bl->buffers[count - 1].iovas[0] += 123;
			if (test == 26) {
				bl->buffers[1].pool_type = CISP_POOL_TYPE_META;
				bl->buffers[1].iovas[0] = metadata[0].iova;
				bl->buffers[2].pool_type = ISP_BUFLIST_POOL_CAPTURE_META;
				bl->buffers[2].iovas[0] = capmeta[0].iova;
			}
		}
		expected_state = test == 18 ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE;
		bl->num_buffers = count;
		CHECK((ipc_bt_handle(&isp, &chan) == 0) == valid);
		if (valid && count) {
			CHECK(completions == before + (test == 16 ? 16 : 1));
			CHECK(rendered[0].vb.vb2_buf.done == 1);
			if (test == 16) {
				for (unsigned int i = 0;
				     i < ARRAY_SIZE(metadata); i++)
					CHECK(!metadata[i].submitted);
				for (unsigned int i = 0;
				     i < ARRAY_SIZE(capmeta); i++)
					CHECK(!capmeta[i].submitted);
			}
			/* A stale return with no outstanding identity must not complete twice. */
			CHECK(ipc_bt_handle(&isp, &chan) != 0);
		} else {
			CHECK(completions == before &&
			      !rendered[0].vb.vb2_buf.done);
			CHECK(metadata[1].submitted && capmeta[1].submitted);
			if (test >= 25) {
				struct list_head *node = isp.bufs_submitted.next;

				CHECK(list_empty(&isp.bufs_pending));
				CHECK(!isp.sequence);
				for (unsigned int i = 0; i < ARRAY_SIZE(rendered); i++) {
					CHECK(node == &rendered[i].link);
					CHECK(!rendered[i].vb.vb2_buf.done);
					CHECK(metadata[i].submitted);
					node = node->next;
				}
				CHECK(node == &isp.bufs_submitted);
				for (unsigned int i = 0; i < ARRAY_SIZE(capmeta); i++)
					CHECK(capmeta[i].submitted);
			}
		}
	}
	puts("PASS: 27 return/lifecycle cases, full pools and mutable records");
	return 0;
}
