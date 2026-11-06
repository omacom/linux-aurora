// SPDX-License-Identifier: GPL-2.0-only
/* Fake producer, with the actual drain and IRQ-thread scheduling functions. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

typedef uint32_t u32;
#define irqreturn_t int
#define IRQ_HANDLED 1
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, v) ((x) = (v))
#define cmpxchg(p, old, v) __sync_val_compare_and_swap(p, old, v)
#define spin_lock_irqsave(l, f) ((void)(l), (f) = 0)
#define spin_unlock_irqrestore(l, f) ((void)(l), (void)(f))
#define mutex_lock(l) ((void)(l))
#define mutex_unlock(l) ((void)(l))
#define wake_up_all(w) ((void)(w))
#define ISP_IPC_FLAG_ACK 1
#define ISP_IPC_FLAG_TERMINAL_ACK 3

struct isp_message {
	unsigned long arg0;
};

struct isp_channel {
	unsigned int num, cursor, remaining, handled;
	int lock, error;
	struct isp_message req;
};

struct apple_isp {
	struct isp_channel *chan_sm, *chan_tm, *chan_bt;
	int ipc_error, wait, vbq, buf_lock;
	bool ipc_quiescing;
	unsigned long state;
};

#define ISP_STATE_STREAMING 0
static bool test_bit(unsigned int bit, const unsigned long *state)
{
	return *state & (1UL << bit);
}

static void vb2_queue_error(int *queue)
{
	(*queue)++;
}

static unsigned int schedules;
static bool refill;
static void check(bool ok, int line)
{
	if (!ok) {
		fprintf(stderr, "FAIL line %d\n", line);
		exit(1);
	}
}

#define CHECK(x) check((x), __LINE__)

static void cond_resched(void)
{
	CHECK(++schedules < 10000);
}

static void chan_read_msg(struct apple_isp *isp, struct isp_channel *chan,
			  struct isp_message *req)
{
	req->arg0 = chan->remaining ? 0 : ISP_IPC_FLAG_ACK;
}

static int chan_handle_once(struct apple_isp *isp, struct isp_channel *chan)
{
	if (chan->error)
		return chan->error;
	chan->handled++;
	if (!refill)
		chan->remaining--;
	chan->cursor = (chan->cursor + 1) % chan->num;
	return 0;
}

/* DRIVER FUNCTIONS */
int main(void)
{
	struct isp_channel channels[3] = {
		{ .num = 2, .remaining = 11 },
		{ .num = 3, .remaining = 8 },
		{ .num = 1, .remaining = 7 },
	};
	struct apple_isp isp = {
		.chan_sm = &channels[0],
		.chan_tm = &channels[1],
		.chan_bt = &channels[2],
	};

	CHECK(ipc_chan_handle(&isp, &channels[0]) == -EAGAIN);
	CHECK(channels[0].handled == 2 && channels[0].remaining == 9);
	CHECK(apple_isp_isr_thread(0, &isp) == IRQ_HANDLED);
	CHECK(channels[0].handled == 11 && channels[1].handled == 8 &&
	      channels[2].handled == 7);
	CHECK(!channels[0].remaining && !channels[1].remaining &&
	      !channels[2].remaining);
	CHECK(schedules >= 7 && schedules <= 8);
	/* Healthy streaming IRQs must leave the queue usable. */
	isp.state = 1UL << ISP_STATE_STREAMING;
	CHECK(apple_isp_isr_thread(0, &isp) == IRQ_HANDLED);
	CHECK(!isp.ipc_error && !isp.vbq);
	channels[0].remaining = 1;
	CHECK(apple_isp_isr_thread(0, &isp) == IRQ_HANDLED);
	CHECK(!channels[0].remaining && channels[0].handled == 12);
	CHECK(!isp.ipc_error && !isp.vbq);
	isp.state = 0;
	/* A continuous producer must let a quiescing invocation finish. */
	refill = true;
	channels[0].remaining = 2;
	isp.ipc_quiescing = true;
	unsigned int before = schedules;

	CHECK(apple_isp_isr_thread(0, &isp) == IRQ_HANDLED);
	CHECK(schedules == before + 1 && !isp.ipc_error);
	refill = false;
	isp.ipc_quiescing = false;
	channels[0].remaining = 2;
	channels[0].error = -EIO;
	CHECK(ipc_chan_handle(&isp, &channels[0]) == -EIO &&
	      isp.ipc_error == -EIO);
	channels[0].error = 0;
	CHECK(ipc_chan_handle(&isp, &channels[0]) == -EIO &&
	      channels[0].remaining == 2);
	CHECK(apple_isp_isr_thread(0, &isp) == IRQ_HANDLED && isp.vbq == 0);
	isp.state = 1UL << ISP_STATE_STREAMING;
	CHECK(apple_isp_isr_thread(0, &isp) == IRQ_HANDLED && isp.vbq == 1);
	puts("PASS: bounded turns, burst continuation, channel fairness and error containment");
	return 0;
}
