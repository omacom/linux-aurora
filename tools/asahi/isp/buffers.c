// SPDX-License-Identifier: GPL-2.0-only
/* Host-only boundaries for the real batch producer and return handler. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

typedef uint64_t u64;
typedef uint32_t u32;
typedef u64 dma_addr_t;
#define __packed
#define ISP_MAX_PLANES 4
#define ISP_MAX_BUFFERS 16
#define ISP_CAPMETA_BUFFERS 8
#define ISP_CMD_AREA_SIZE(n) (16 + 64 * (32 + (n)))
#define ISP_IPC_BUFEXC_STAT_SIZE 0x280
#define ISP_IPC_BUFEXC_FLAG_COMMAND 0x30000000
#define ISP_IPC_BUFEXC_FLAG_RENDER 0x10000000
#define ISP_IPC_BUFEXC_FLAG_ACK 0x80000000
#define ISP_IPC_FLAG_ACK 1
#define ISP_BUFFER_TIMEOUT 1500
#define CISP_POOL_TYPE_META 0
#define CISP_POOL_TYPE_RENDERED 1
#define CISP_POOL_TYPE_RENDERED_SCL1 9
#define ISP_BUFLIST_POOL_CAPTURE_META 2
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define max_t(t, a, b) ((t)max_u64(a, b))
#define READ_ONCE(x) (x)
#define isp_num_capmeta(i) \
	((i)->hw->capture_meta_size ? ISP_CAPMETA_BUFFERS : 0)
#define isp_get_current_format(i) (&(i)->fmt)
#define isp_fw_iova(i, x) ((x) & (i)->fw_iova_mask)
static pthread_mutex_t buffer_mutex = PTHREAD_MUTEX_INITIALIZER;
static unsigned int acquisitions;
#define spin_lock_irqsave(l, f) \
	((void)(l), (f) = 0, acquisitions++, pthread_mutex_lock(&buffer_mutex))
#define spin_unlock_irqrestore(l, f) \
	((void)(l), (void)(f), pthread_mutex_unlock(&buffer_mutex))
#define spin_lock(l) ((void)(l), acquisitions++, pthread_mutex_lock(&buffer_mutex))
#define spin_unlock(l) ((void)(l), pthread_mutex_unlock(&buffer_mutex))
static bool test_bit(unsigned int bit, const unsigned long *state)
{
	return *state & (1UL << bit);
}

static inline bool test_and_clear_bit(unsigned int bit, unsigned long *state)
{
	bool previous = test_bit(bit, state);

	*state &= ~(1UL << bit);
	return previous;
}

#define ISP_STATE_STREAMING 0
#define ISP_STATE_SLEEPING 2

static void set_bit(unsigned int bit, unsigned long *state)
{
	*state |= 1UL << bit;
}

static void clear_bit(unsigned int bit, unsigned long *state)
{
	*state &= ~(1UL << bit);
}

#define dev_err(...) ((void)0)
#define dev_err_ratelimited(...) ((void)0)
#define dev_warn_ratelimited(...) ((void)0)
#define GFP_KERNEL 0
#define array_size(a, b) ((a) * (b))
#define kfree(p) free(p)

static void check(bool ok, int line)
{
	if (!ok) {
		fprintf(stderr, "FAIL line %d\n", line);
		exit(1);
	}
}

#define CHECK(x) check((x), __LINE__)

struct list_head {
	struct list_head *next, *prev;
};

static void INIT_LIST_HEAD(struct list_head *h)
{
	h->next = h;
	h->prev = h;
}

static void list_add_tail(struct list_head *e, struct list_head *h)
{
	e->prev = h->prev;
	e->next = h;
	h->prev->next = e;
	h->prev = e;
}

static void list_del(struct list_head *e)
{
	e->prev->next = e->next;
	e->next->prev = e->prev;
}

static void list_move_tail(struct list_head *e, struct list_head *h)
{
	list_del(e);
	list_add_tail(e, h);
}

static void list_splice_init(struct list_head *from, struct list_head *to)
{
	if (from->next == from)
		return;
	from->prev->next = to->next;
	to->next->prev = from->prev;
	to->next = from->next;
	from->next->prev = to;
	INIT_LIST_HEAD(from);
}

static bool list_empty(struct list_head *h)
{
	return h->next == h;
}

#define container_of(p, type, m)                                   \
	({                                                         \
		type *__ptr;                                       \
		__ptr = (void *)((char *)(p) -                     \
				 offsetof(__typeof__(*__ptr), m)); \
		__ptr;                                             \
	})
#define list_first_entry_or_null(h, t, m)                                     \
	({                                                                    \
		struct list_head *__head = (h);                               \
		list_empty(__head) ? NULL : container_of(__head->next, t, m); \
	})
#define list_for_each_entry(p, h, m)                                 \
	for (struct list_head *node = (h)->next, *head = node->prev; \
	     node != head &&                                         \
	     ((p) = container_of(node, struct isp_buffer, m), true); \
	     node = node->next)
#define list_for_each_entry_safe_reverse(p, t, h, m)                         \
	for (struct list_head *node = (h)->prev, *head = node->next, *prev;  \
	     node != head && (prev = node->prev,                             \
			     (p) = container_of(node, struct isp_buffer, m), \
			     (t) = NULL, true);                              \
	     node = prev)

static u64 max_u64(u64 a, u64 b)
{
	return a > b ? a : b;
}

struct isp_surf {
	u64 iova, size;
	bool submitted;
	void *virt;
};

struct apple_isp;
struct vb2_queue {
	struct apple_isp *priv;
	bool error;
};

struct vb2_buffer {
	struct vb2_queue *vb2_queue;
	u64 timestamp;
	unsigned int done;
};

struct isp_buffer {
	struct {
		struct vb2_buffer vb2_buf;
		unsigned int sequence, field;
	} vb;
	struct list_head link;
	struct isp_surf surfs[4];
	u64 tag;
	int return_state;
};

struct isp_message {
	u64 arg0, arg1, arg2;
};

struct isp_channel {
	struct isp_message req, rsp;
};

struct isp_format {
	unsigned int num_planes;
	size_t plane_size[4];
};

struct hardware {
	bool scl1, resident_fw;
	unsigned int capture_meta_size;
};

struct apple_isp {
	void *dcp;
	struct isp_format fmt;
	struct isp_channel *chan_bh;
	const struct hardware *hw;
	struct isp_surf *meta_surfs[ISP_MAX_BUFFERS];
	struct isp_surf *capmeta_surfs[ISP_CAPMETA_BUFFERS];
	struct isp_surf *bt_surf;
	struct list_head bufs_pending, bufs_submitted, bufs_returned;
	void *cmd_virt;
	u64 cmd_iova, fw_iova_mask;
	int ipc_error, buf_lock, video_lock;
	unsigned int current_ch;
	struct vb2_queue vbq;
	unsigned long state;
	void *exposure_priority, *power_line_frequency;
	bool ipc_failed;
	int fw_state;
	u64 buffer_tag;
	unsigned long meta_returned;
	unsigned int sequence;
};

/* DRIVER STRUCTURES */
static unsigned int sends, fail_at, sent_meta, sent_capmeta;
static int (*return_hook)(struct apple_isp *isp);
static pthread_mutex_t *video_lock(void)
{
	static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

	return &lock;
}

#define mutex_lock(l) ((void)(l), pthread_mutex_lock(video_lock()))
#define mutex_unlock(l) ((void)(l), pthread_mutex_unlock(video_lock()))
static bool quiescent;
static void (*transition_error)(struct apple_isp *isp);
static int send_error;
static bool mutate;

static int ipc_chan_send(struct apple_isp *isp, struct isp_channel *chan,
			 unsigned long timeout)
{
	struct isp_buflist *bl = isp->cmd_virt;

	sends++;
	for (unsigned int i = 0; i < bl->num_buffers; i++) {
		sent_meta += bl->buffers[i].pool_type == CISP_POOL_TYPE_META;
		sent_capmeta += bl->buffers[i].pool_type == ISP_BUFLIST_POOL_CAPTURE_META;
	}
	if (mutate) {
		bl->num_buffers = UINT64_MAX;
		memset(bl->buffers, 0xff,
		       ISP_CMD_AREA_SIZE(isp_num_capmeta(isp)) - 16);
	}
	int err = fail_at ? (sends == fail_at ? -ETIME : 0) : send_error;

	if (err)
		isp->ipc_error = err;
	return err;
}

static int apple_isp_start_camera(struct apple_isp *isp)
{
	CHECK(quiescent || !isp->ipc_error);
	isp->ipc_error = 0;
	quiescent = false;
	return 0;
}

static void apple_isp_stop_camera(struct apple_isp *isp)
{
	quiescent = true;
}

static int apple_isp_start_capture(struct apple_isp *isp)
{
	if (transition_error)
		transition_error(isp);
	return 0;
}

static int isp_cmd_ch_stop(struct apple_isp *isp, int channel)
{
	return isp->ipc_error;
}

static int isp_cmd_ch_buffer_return(struct apple_isp *isp, int channel)
{
	return return_hook ? return_hook(isp) : 0;
}

static void apple_isp_wdt_stop(struct apple_isp *isp)
{
}

/* The camera indicator is not under test here: no DCP, so no call. */
static bool apple_isp_camera_indicator(const struct apple_isp *isp)
{
	return isp->dcp;
}

static int apple_isp_dcp_indicator_set(void *dcp, bool on)
{
	return 0;
}

static void apple_isp_stop_capture(struct apple_isp *isp);

/* These fixtures exercise the non-P010/J613 path. P010 has kernel KUnit coverage. */
#define ISP_FW_DEAD 3
/* The zero-initialized fixtures run the firmware (#1445's stop checks for it). */
#define ISP_FW_RUNNING 0
#define __set_bit set_bit
static bool isp_is_p010(struct apple_isp *isp) { return false; }
static bool isp_supports_p010(struct apple_isp *isp) { return false; }
static unsigned int isp_num_meta(struct apple_isp *isp) { return ISP_MAX_BUFFERS; }
static unsigned int isp_num_rendered(struct apple_isp *isp) { return ISP_MAX_BUFFERS; }
static int isp_ipc_resync(struct apple_isp *isp) { return 0; }
static void isp_meta_read_exposure(struct apple_isp *isp, struct isp_surf *surf) { }
static bool isp_p010_buffer_matches(struct isp_format *fmt, struct isp_buffer *buf, const struct isp_buflist_buffer *desc) { return false; }
static void apple_isp_firmware_halt(struct apple_isp *isp) { quiescent = true; }
/* DRIVER FUNCTIONS */
int main(void)
{
	struct hardware hw = { 0 };
	struct isp_channel chan = { 0 };
	struct isp_surf metadata[ISP_MAX_BUFFERS], capmeta[ISP_CAPMETA_BUFFERS];
	struct isp_buffer rendered[ISP_MAX_BUFFERS + 1] = { 0 };
	unsigned char area[ISP_CMD_AREA_SIZE(ISP_CAPMETA_BUFFERS)];
	struct apple_isp isp = {
		.hw = &hw,
		.chan_bh = &chan,
		.cmd_virt = area,
		.fmt = { .num_planes = 2, .plane_size = { 4096, 2048 } },
	};

	for (unsigned int mode = 0; mode < 4; mode++) {
		unsigned int caps = mode & 1;
		bool pre_start = mode & 2;

		for (unsigned int fail = 0; fail < 2; fail++) {
			for (unsigned int change = 0; change < 2; change++) {
				INIT_LIST_HEAD(&isp.bufs_pending);
				INIT_LIST_HEAD(&isp.bufs_submitted);
				isp.ipc_error = 0;
				hw.capture_meta_size = caps;
				for (unsigned int i = 0;
				     i < ARRAY_SIZE(metadata); i++) {
					metadata[i] = (struct isp_surf){
						.iova = 0x100000000ULL +
							i * 4096
					};
					isp.meta_surfs[i] = &metadata[i];
				}
				for (unsigned int i = 0;
				     i < ARRAY_SIZE(capmeta); i++) {
					capmeta[i] = (struct isp_surf){
						.iova = 0x200000000ULL +
							i * 4096
					};
					isp.capmeta_surfs[i] = &capmeta[i];
				}
				for (unsigned int i = 0;
				     i < ARRAY_SIZE(rendered); i++) {
					rendered[i].surfs[0].iova =
						0x300000000ULL + i * 8192;
					list_add_tail(&rendered[i].link,
						      &isp.bufs_pending);
				}
				send_error = fail ? -ETIME : 0;
				mutate = change;
				unsigned int locks_before = acquisitions;

				CHECK(isp_submit_buffers(&isp, pre_start) ==
				      send_error);
				CHECK(acquisitions == locks_before + 1);
				CHECK(isp.bufs_pending.next ==
				      &rendered[caps && pre_start ?
							0 :
							ISP_MAX_BUFFERS]
					       .link);
				CHECK(isp.bufs_pending.prev ==
				      &rendered[ISP_MAX_BUFFERS].link);
				CHECK(isp.bufs_submitted.next ==
				      (caps && pre_start ? &isp.bufs_submitted :
							   &rendered[0].link));
				CHECK(isp.bufs_submitted.prev ==
				      (caps && pre_start ?
					       &isp.bufs_submitted :
					       &rendered[ISP_MAX_BUFFERS - 1]
							.link));
				for (unsigned int i = 0;
				     i < ARRAY_SIZE(metadata); i++)
					CHECK(metadata[i].submitted);
				for (unsigned int i = 0;
				     i < ARRAY_SIZE(capmeta); i++)
					CHECK(capmeta[i].submitted ==
					      !!(caps && !pre_start));
				if (fail) {
					unsigned int before = sends;
					unsigned char saved[sizeof(area)];

					memcpy(saved, area, sizeof(area));
					CHECK(isp_submit_buffers(&isp,
								 pre_start) ==
					      -ETIME);
					CHECK(sends == before &&
					      !memcmp(saved, area,
						      sizeof(area)));
				}
			}
		}
	}
	/* Reuse the same metadata objects after each failed, retired session. */
	mutate = false;
	for (unsigned int caps = 0; caps < 2; caps++) {
		hw.capture_meta_size = caps;
		for (unsigned int phase = 1; phase <= (caps ? 2 : 1); phase++) {
			INIT_LIST_HEAD(&isp.bufs_pending);
			INIT_LIST_HEAD(&isp.bufs_submitted);
			isp.ipc_error = 0;
			isp.state = 0;
			fail_at = phase;
			sends = 0;
			CHECK(apple_isp_start_streaming(&isp) == -ETIME);
			CHECK(quiescent && metadata[0].submitted);
			if (phase == 2)
				CHECK(capmeta[0].submitted);
			fail_at = 0;
			send_error = 0;
			sent_meta = 0;
			sent_capmeta = 0;
			CHECK(!apple_isp_start_streaming(&isp));
			CHECK(sent_meta == ISP_MAX_BUFFERS);
			CHECK(sent_capmeta == (caps ? ISP_CAPMETA_BUFFERS : 0));
			apple_isp_stop_camera(&isp);
		}
	}
	puts("PASS: immutable ownership with mutable batches, timeout and full pools");
	return 0;
}
