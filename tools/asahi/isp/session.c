// SPDX-License-Identifier: GPL-2.0-only
/* Host fixture: the functions under test are extracted without rewriting. */
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>

typedef uint64_t u64;
typedef uint32_t u32;
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, v) ((x) = (v))
#define cmpxchg(p, old, v) __sync_val_compare_and_swap(p, old, v)
#define dma_wmb() ((void)0)
#define dev_err(...) ((void)0)
#define dev_warn(...) ((void)0)
#define dev_err_ratelimited(...) ((void)0)
#define isp_err(...) ((void)0)
#define isp_dbg(...) ((void)0)
#define CISP_OPCODE_GET(x) ((x) >> 32)
#define ISP_MBOX2_IRQ_DOORBELL 0
#define irqreturn_t int
#define IRQ_HANDLED 1
#define ISP_IPC_FLAG_ACK 1
#define ISP_IPC_FLAG_TERMINAL_ACK 3
#define mutex_lock(l) ((void)(l))
#define mutex_unlock(l) ((void)(l))
#define ISP_FW_OFF 0
#define ISP_FW_RUNNING 1
#define ISP_FW_DEAD 2
static void check(bool ok, int line)
{
	if (!ok) {
		fprintf(stderr, "FAIL line %d\n", line);
		exit(1);
	}
}

#define CHECK(x) check((x), __LINE__)

struct isp_message {
	u64 arg0, arg1, arg2;
};

struct isp_channel {
	struct isp_message req;
	unsigned int doorbell, num, cursor, remaining;
	int lock;
};

struct hardware {
	bool resident_fw;
};

struct apple_isp {
	int ipc_error, wait, fw_state, irq, vbq, buf_lock;
	unsigned long state;
	bool ipc_quiescing;
	struct isp_channel *chan_io, *chan_sm, *chan_tm, *chan_bt;
	const struct hardware *hw;
	void *dev, *wq, *cmd_virt;
	u64 cmd_iova;
	unsigned int current_ch;
};

#define ISP_STATE_STREAMING 0
#define spin_lock_irqsave(l, f) ((void)(l), (f) = 0)
#define spin_unlock_irqrestore(l, f) ((void)(l), (void)(f))
static bool test_bit(unsigned int bit, const unsigned long *state)
{
	return *state & (1UL << bit);
}

static void vb2_queue_error(int *queue)
{
	(*queue)++;
}

static unsigned int writes, bells, cursors, stops, returns, watchdog_stops;
static unsigned int stop_order, free_order, order, halts, freed_released;
static int stage_error, fail_stage, stop_error;
static bool ack;
static int entry_error, stage3_error;
static void chan_write_msg(struct apple_isp *isp, struct isp_channel *ch,
			   struct isp_message *msg)
{
	writes++;
}

static void isp_mbox2_write32(struct apple_isp *isp, int reg, unsigned int v)
{
	bells++;
}

static bool chan_tx_done(struct apple_isp *isp, struct isp_channel *ch)
{
	if (ack)
		cursors++;
	return ack;
}

#define wait_event_timeout(w, condition, timeout) \
	((void)(w), (void)(timeout), (condition) ? 1L : 0L)
static pthread_mutex_t wait_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t wait_cond = PTHREAD_COND_INITIALIZER;
static bool waiting, notified;

static void wake_up_all(int *wait)
{
	pthread_mutex_lock(&wait_lock);
	notified = true;
	pthread_cond_broadcast(&wait_cond);
	pthread_mutex_unlock(&wait_lock);
}

static void *waiter(void *arg)
{
	struct timespec deadline;

	clock_gettime(CLOCK_REALTIME, &deadline);
	deadline.tv_sec++;
	pthread_mutex_lock(&wait_lock);
	waiting = true;
	pthread_cond_broadcast(&wait_cond);
	while (!notified) {
		int err = pthread_cond_timedwait(&wait_cond, &wait_lock, &deadline);

		CHECK(err != ETIMEDOUT);
		CHECK(!err);
	}
	pthread_mutex_unlock(&wait_lock);
	return NULL;
}

static int isp_cmd_ch_stop(struct apple_isp *isp, int ch)
{
	stops++;
	return stop_error;
}

static int isp_cmd_ch_buffer_return(struct apple_isp *isp, int ch)
{
	returns++;
	return 0;
}

static void apple_isp_wdt_stop(struct apple_isp *isp)
{
	watchdog_stops++;
}

static void isp_firmware_shutdown_stage1(struct apple_isp *isp)
{
	stop_order = ++order;
}

static void isp_firmware_shutdown_stage3(struct apple_isp *isp)
{
	free_order = ++order;
}

static void isp_firmware_shutdown_stage2(struct apple_isp *isp)
{
	++order;
}

static void isp_collect_gc_surface(struct apple_isp *isp)
{
	++order;
}

static int stage(struct apple_isp *isp, int n)
{
	if (n == 1)
		entry_error = isp->ipc_error;
	if (n == 3)
		stage3_error = isp->ipc_error;
	return fail_stage == n ? stage_error : 0;
}

static int isp_firmware_boot_stage1(struct apple_isp *isp)
{
	return stage(isp, 1);
}

static int isp_firmware_boot_stage2(struct apple_isp *isp)
{
	return stage(isp, 2);
}

static int isp_firmware_boot_stage3(struct apple_isp *isp)
{
	return stage(isp, 3);
}

static int isp_enable_irq(struct apple_isp *isp)
{
	return stage(isp, 4);
}

static int isp_start_command_processor(struct apple_isp *isp)
{
	if (isp->ipc_error)
		return isp->ipc_error;
	return stage(isp, 5);
}

static irqreturn_t apple_isp_isr_thread(int irq, void *dev);
static unsigned int receive_rounds;
static bool refill;

static void cond_resched(void)
{
	CHECK(++receive_rounds < 10000);
}

static void chan_read_msg(struct apple_isp *isp, struct isp_channel *chan,
			  struct isp_message *req)
{
	req->arg0 = chan->remaining ? 0 : ISP_IPC_FLAG_ACK;
}

static int chan_handle_once(struct apple_isp *isp, struct isp_channel *chan)
{
	if (!refill)
		chan->remaining--;
	return 0;
}

static bool shutdown_called, flood;
static struct apple_isp *synchronizing;
static int sync_error;
static bool irq_disabled;
static int isp_stop_command_processor(struct apple_isp *isp)
{
	shutdown_called = true;
	return 0;
}

static void isp_disable_irq(struct apple_isp *isp)
{
	if (flood)
		apple_isp_isr_thread(isp->irq, isp);
}

static void flush_workqueue(void *wq)
{
}

static int pm_runtime_resume_and_get(void *dev)
{
	return 0;
}

static void pm_runtime_put_sync(void *dev)
{
	halts++;
}

static void apple_isp_firmware_halt(struct apple_isp *isp);

static void disable_irq(int irq)
{
	CHECK(synchronizing->ipc_quiescing);
	irq_disabled = true;
	apple_isp_isr_thread(irq, synchronizing);
	if (sync_error)
		synchronizing->ipc_error = sync_error;
}

static void enable_irq(int irq)
{
	CHECK(irq_disabled && !synchronizing->ipc_quiescing);
	irq_disabled = false;
}

static void isp_free_released_surfaces(struct apple_isp *isp)
{
	CHECK(irq_disabled);
	freed_released++;
}

/* DRIVER FUNCTIONS */
int main(void)
{
	struct isp_channel ch = { 0 };
	struct isp_channel incoming = { .num = 2, .remaining = 2 };
	const struct hardware hw = { 0 }, resident = { .resident_fw = true };
	u64 area[4] = { 1, 2, 3, 4 }, args[4] = { 5, 6, 7, 8 };
	struct apple_isp isp = { .hw = &hw, .chan_io = &ch, .cmd_virt = area,
				 .chan_sm = &incoming, .chan_tm = &incoming,
				 .chan_bt = &incoming };

	pthread_t thread;

	CHECK(!pthread_create(&thread, NULL, waiter, NULL));
	pthread_mutex_lock(&wait_lock);
	while (!waiting)
		pthread_cond_wait(&wait_cond, &wait_lock);
	pthread_mutex_unlock(&wait_lock);
	CHECK(ipc_chan_send(&isp, &ch, 1) == -ETIME);
	CHECK(!pthread_join(thread, NULL));
	CHECK(isp.ipc_error == -ETIME && writes == 1 && bells == 1 &&
	      cursors == 0);
	ack = true;
	for (unsigned int i = 0; i < 3; i++) {
		CHECK(ipc_chan_send(&isp, &ch, 1) == -ETIME);
		CHECK(ipc_chan_send(&isp, &ch, 0) == -ETIME);
		CHECK(cisp_send(&isp, args, sizeof(args), 0, 1) == -ETIME);
	}
	CHECK(writes == 1 && bells == 1 && cursors == 0);
	CHECK(area[0] == 1 && area[1] == 2 && area[2] == 3 && area[3] == 4);
	isp.ipc_error = -EIO;
	CHECK(ipc_chan_send(&isp, &ch, 1) == -EIO);
	CHECK(writes == 1 && cursors == 0);
	for (unsigned int fail = 0; fail < 2; fail++) {
		stop_error = fail ? -ETIME : 0;
		stops = 0;
		returns = 0;
		watchdog_stops = 0;
		apple_isp_stop_capture(&isp);
		CHECK(stops == 1 && returns == !fail && watchdog_stops == 1);
	}
	for (fail_stage = 1; fail_stage <= 5; fail_stage++) {
		isp.ipc_error = -ETIME;
		stage_error = -ENOMEM;
		order = 0;
		stop_order = 0;
		free_order = 0;
		CHECK(isp_firmware_boot(&isp) == -ENOMEM);
		CHECK(entry_error == -ETIME);
		if (fail_stage >= 3)
			CHECK(stage3_error == -ETIME);
		if (fail_stage >= 2)
			CHECK(stop_order && free_order > stop_order);
		CHECK(isp.ipc_error == (fail_stage <= 3 ? -ETIME : -ENOMEM));
	}
	fail_stage = 0;
	isp.ipc_error = -ETIME;
	CHECK(isp_firmware_boot(&isp) == 0 && isp.ipc_error == 0);
	CHECK(ipc_chan_send(&isp, &ch, 1) == 0 && cursors == 1);
	CHECK(ipc_chan_send(&isp, &ch, 0) == 0 && cursors == 1);
	isp.hw = &resident;
	isp.fw_state = ISP_FW_RUNNING;
	isp.ipc_error = -ETIME;
	CHECK(apple_isp_firmware_boot(&isp) == -ETIME &&
	      isp.ipc_error == -ETIME);
	synchronizing = &isp;
	apple_isp_firmware_shutdown(&isp);
	CHECK(halts == 1 && freed_released == 0 && isp.fw_state == ISP_FW_DEAD);
	CHECK(stop_order < free_order);
	CHECK(apple_isp_firmware_boot(&isp) == -EIO);
	/* Healthy resident close is temporary; a late receive failure halts. */
	isp.fw_state = ISP_FW_RUNNING;
	isp.ipc_error = 0;
	isp.chan_sm = &incoming;
	isp.chan_tm = &incoming;
	isp.chan_bt = &incoming;
	refill = true;
	receive_rounds = 0;
	synchronizing = &isp;
	apple_isp_firmware_shutdown(&isp);
	CHECK(!irq_disabled && !isp.ipc_quiescing && freed_released == 1);
	CHECK(receive_rounds == 1);
	CHECK(isp.fw_state == ISP_FW_RUNNING && !isp.ipc_error);
	sync_error = -EIO;
	apple_isp_firmware_shutdown(&isp);
	CHECK(isp.fw_state == ISP_FW_DEAD && freed_released == 1);
	CHECK(halts == 2 && isp.ipc_error == -EIO);
	isp.ipc_error = 0;
	isp.ipc_quiescing = false;
	flood = true;
	isp_firmware_shutdown(&isp);
	CHECK(isp.ipc_error == -ESHUTDOWN);
	puts("PASS: timeout, late ACK, shared writes, stop, boot and resident lifetime");
	return 0;
}
