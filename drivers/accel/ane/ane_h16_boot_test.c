// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <kunit/test.h>
#include <linux/module.h>

#include "ane_h16_boot_data.h"

struct ane_h16_sim {
	struct kunit *test;
	struct ane_h16_boot boot;
	struct ane_h16_build build;
	struct ane_h16_mmio mmio;
	struct ane_fw_platform soc;
	u32 scratch[8];
	u32 irq;
	u32 dapf[SZ_16K / 4];
	u32 dart_params;
	u32 dapf_reads;
	u32 dapf_order[8];
	int dapf_stuck;
	u64 now;
	u64 ce_ack;
	u32 cpu;
	u32 command_count;
	u32 scratch_writes;
	u32 tuning_writes;
	/* Tuning writes expected before the first scratch write; XL writes seen. */
	u32 expect_tuning;
	u32 xl_writes;
	/* Engine registers below 0x5000 keep what is written, like the AXI block. */
	u32 engine[0x5000 / 4];
	u32 dapf_writes;
	u32 mailbox_polls;
	u32 maps;
	u32 pins;
	u32 quarantines;
	u32 allocations;
	u32 timeout_slots;
	u32 timeout_bodies;
	u32 timeout_scratch;
	u64 timeout_head;
	u32 lock;
	u32 protect[3];
	u32 protect_reads;
	u32 fault;
	u32 fault_address_reads;
	u32 channel_request;
	u32 ipc_request;
	u32 heap_request;
	u32 tick;
	struct ane_fw_rtk rtk;
	u32 mailbox_tx;
	bool offered_mailbox;
	bool fault_after_run;
	u32 lifecycle_mode;
	bool lifecycle_requested;
	u16 refuse_id;
	bool no_ready;
	bool wrong_table;
	bool stale_table;
	bool no_clear;
	bool no_startup_reply;
	bool shared_pending;
	bool shared_replied;
	bool ce_timeout;
	bool bad_power;
	bool bad_rvbar;
	bool fail_map;
	/* Restart op: startups still to stall, cycles seen, result, reset latch. */
	struct ane_h16_boot_ops ops;
	u32 stall_startups;
	u32 power_cycles;
	int restart_error;
	bool reset;
	bool unpowered;
	u32 quarantine_reads;
	/* The stopped op, and the CPU control word the stop evidence logged. */
	struct ane_fw_transport_ops transport;
	u64 stop_control;
	u32 stopped_checks;
};

static const u32 sim_power_words[] = { 0x3c0, 0xc008, 0xc000, 0xc010, 0xc018 };

/* Only the PARAMS3 word and the offsets of the selected table are accessed. */
static void sim_check_dart(struct ane_h16_sim *s, enum ane_h16_window w, u32 off)
{
	const struct ane_h16_mmio *m = s->boot.mmio;

	if (off == s->soc.dart_params)
		return;
	KUNIT_ASSERT_NOT_NULL(s->test, m);
	if (off == m->dart_fault || off == m->dart_address_lo || off == m->dart_address_hi)
		return;
	if (off == m->dart_protect)
		return;
	KUNIT_EXPECT_EQ(s->test, w, ANE_H16_DART0 + s->soc.dapf_dart);
	KUNIT_EXPECT_EQ(s->test, off, m->dart_lock);
}

static void sim_check_dapf(struct ane_h16_sim *s, u32 off, u32 width)
{
	const struct ane_h16_mmio *m = s->boot.mmio;

	KUNIT_ASSERT_NOT_NULL(s->test, m);
	KUNIT_EXPECT_GE(s->test, off, m->dapf_records);
	KUNIT_EXPECT_LE(s->test, off + width,
			m->dapf_records + s->soc.dapf_count * m->dapf_stride);
	KUNIT_EXPECT_LE(s->test, off + width, s->soc.dapf_size);
}

static u32 sim_read32(void *ctx, enum ane_h16_window w, u32 off)
{
	struct ane_h16_sim *s = ctx;

	if (w == ANE_H16_PMGR)
		return s->bad_power ? 0xf : 0xff;
	if (w >= ANE_H16_DART0 && w <= ANE_H16_DART2) {
		sim_check_dart(s, w, off);
		if (off == s->soc.dart_params)
			return s->dart_params;
		if (off == s->mmio.dart_protect) {
			s->protect_reads |= BIT(w - ANE_H16_DART0);
			return s->protect[w - ANE_H16_DART0];
		}
		if (off == s->mmio.dart_lock)
			return s->lock;
		if (off == s->mmio.dart_fault)
			return s->fault;
		if (off == s->mmio.dart_address_lo || off == s->mmio.dart_address_hi)
			s->fault_address_reads++;
		return 0;
	}
	if (w == ANE_H16_DAPF) {
		sim_check_dapf(s, off, 4);
		s->dapf_reads++;
		return s->dapf[off / 4];
	}
	if (w == ANE_H16_ENGINE) {
		if (off == s->build.cpu_control)
			return s->cpu;
		if (off == 0x197fffc)
			return 0;
		if (off >= s->soc.scratch && off < s->soc.scratch + 32)
			return s->scratch[(off - s->soc.scratch) / 4];
		if (off == s->soc.irq_status)
			return s->irq;
		if (off < sizeof(s->engine))
			return s->engine[off / 4];
	}
	return 0xa5a5a5a5;
}

static u64 sim_read64(void *ctx, enum ane_h16_window w, u32 off)
{
	struct ane_h16_sim *s = ctx;

	if (w == ANE_H16_DAPF) {
		sim_check_dapf(s, off, 8);
		s->dapf_reads++;
		return s->dapf[off / 4] | (u64)s->dapf[off / 4 + 1] << 32;
	}
	KUNIT_EXPECT_EQ(s->test, w, ANE_H16_ENGINE);
	if (off == s->build.rvbar)
		return s->bad_rvbar ? 0 : s->build.expected_rvbar;
	if (off == s->build.ce + 8)
		return s->ce_ack;
	return 0;
}

static void sim_reply(struct ane_h16_sim *s)
{
	struct ane_fw_transport *t = &s->boot.transport;
	struct ane_fw_transaction *tx = &t->pending[ANE_FW_IO];
	u8 *slot = ane_fw_transport_slot(t, ANE_FW_IO);
	u8 *command = (u8 *)t->pool.cpu + (tx->dva - t->pool.dva);

	if (tx->id == 3)
		put_unaligned_le32(s->tick, command + 8);
	if (tx->id == 0x22)
		memset(command + 8, 0x5a, 92);
	if (tx->id == s->refuse_id)
		put_unaligned_le16(7, command + 6);
	put_unaligned_le64(0, slot + 16);
	put_unaligned_le64(tx->dva | 1, slot);
}

/* The firmware's channel table, as it publishes it in the IPC surface. */
static void sim_publish_table(void *cpu, u64 dva)
{
	static const char * const names[] = {
		"TERMINAL", "IO", "DEBUG", "BUF_H2T", "BUF_T2H",
		"SHAREDMALLOC", "IO_T2H", "DATA_CHAIN_H2T",
	};
	unsigned int i;

	for (i = 0; i < 8; i++) {
		u8 *p = (u8 *)cpu + i * ANE_FW_DESCRIPTOR_SIZE;

		strscpy(p, names[i], 64);
		put_unaligned_le32(i == 0 ? 2 : i == 4 || i == 5 || i == 6, p + 0x40);
		put_unaligned_le32(i, p + 0x44);
		put_unaligned_le64(1, p + 0x48);
		put_unaligned_le64(dva + ANE_FW_TABLE_SIZE + i * 64, p + 0x50);
	}
}

static void sim_write32(void *ctx, enum ane_h16_window w, u32 off, u32 value)
{
	struct ane_h16_sim *s = ctx;
	const struct ane_h16_build *f = &s->build;

	if (w == ANE_H16_DAPF) {
		sim_check_dapf(s, off, 4);
		if (s->dapf_writes < ARRAY_SIZE(s->dapf_order))
			s->dapf_order[s->dapf_writes] = off;
		s->dapf_writes++;
		/* Like the hardware, r0 keeps only its low byte. */
		if (off % s->mmio.dapf_stride == 0)
			value &= 0xff;
		if (off % s->mmio.dapf_stride == 0x20 &&
		    (int)(off / s->mmio.dapf_stride) == s->dapf_stuck)
			return;
		s->dapf[off / 4] = value;
		return;
	}
	KUNIT_EXPECT_TRUE(s->test, w == ANE_H16_ENGINE || w == ANE_H16_XL);
	if (w == ANE_H16_ENGINE && off == s->soc.irq_ack) {
		s->irq &= ~value;
		return;
	}
	if (w == ANE_H16_ENGINE && off == f->cpu_control) {
		s->cpu = value;
		if (value == f->run)
			s->reset = false;
		if (value == f->run && s->fault_after_run)
			s->fault = BIT(31);
		if (value == f->run && !s->no_ready) {
			s->scratch[0] = s->channel_request;
			s->scratch[1] = s->ipc_request;
			s->scratch[3] = s->heap_request;
			s->scratch[4] = 0;
			s->scratch[7] = f->ready;
		}
		return;
	}
	if (w == ANE_H16_ENGINE && off >= s->soc.scratch && off < s->soc.scratch + 32) {
		u32 cell = (off - s->soc.scratch) / 4;

		if (s->scratch_writes < 8) {
			KUNIT_EXPECT_EQ(s->test, cell, s->scratch_writes);
			KUNIT_EXPECT_EQ(s->test, value, 0U);
			KUNIT_EXPECT_EQ(s->test, s->tuning_writes, s->expect_tuning);
		}
		s->scratch_writes++;
		s->scratch[cell] = value;
		if (cell == 7 && value == ~f->ready) {
			u64 address = f->ipc_dva + (s->wrong_table ? SZ_16K : 0);

			/* A booted-again engine publishes into the cleared, reused IPC. */
			if (s->power_cycles && s->boot.ipc.cpu)
				sim_publish_table(s->boot.ipc.cpu, s->boot.ipc.dva);

			if (!s->stale_table) {
				s->scratch[0] = lower_32_bits(address);
				s->scratch[1] = upper_32_bits(address);
			}
			s->scratch[7] = f->ready;
		}
		if (cell == 3 && value == f->ready && !s->no_clear)
			s->scratch[3] = 0;
		return;
	}
	if (w == ANE_H16_ENGINE && off == s->soc.doorbell) {
		struct ane_fw_transport *t = &s->boot.transport;

		if (value == BIT(1)) {
			struct ane_fw_transaction *tx = &t->pending[ANE_FW_IO];

			KUNIT_EXPECT_LT(s->test, s->command_count, 4U);
			if (s->command_count < 4)
				KUNIT_EXPECT_EQ(s->test, tx->id, f->startup[s->command_count].id);
			s->command_count++;
			if (s->no_startup_reply)
				return;
			if (tx->id == 3 && s->lifecycle_mode) {
				u8 *slot = ane_fw_transport_slot(t, ANE_FW_SHAREDMALLOC);
				u64 dva = s->lifecycle_mode == 2 ? 0 : s->build.shared_dva;

				if (s->lifecycle_mode == 3)
					dva += SZ_16K;

				put_unaligned_le64(dva, slot);
				put_unaligned_le64(0, slot + 8);
				put_unaligned_le64(s->lifecycle_mode == 2 ? 1 : 0, slot + 16);
				s->lifecycle_requested = true;
				s->irq |= BIT(5);
			} else if (!tx->id) {
				u8 *slot = ane_fw_transport_slot(t, ANE_FW_SHAREDMALLOC);

				put_unaligned_le64(0, slot);
				put_unaligned_le64(SZ_16K, slot + 8);
				put_unaligned_le64(0, slot + 16);
				s->shared_pending = true;
				s->irq |= BIT(5);
			} else {
				sim_reply(s);
			}
		}
		if (value == BIT(5)) {
			s->shared_replied = true;
			if (s->lifecycle_requested) {
				u8 *slot = ane_fw_transport_slot(t, ANE_FW_SHAREDMALLOC);

				KUNIT_EXPECT_EQ(s->test, s->lifecycle_mode, 1U);
				KUNIT_EXPECT_EQ(s->test, get_unaligned_le64(slot), 1ULL);
				KUNIT_EXPECT_EQ(s->test, get_unaligned_le64(slot + 8), 0ULL);
				KUNIT_EXPECT_EQ(s->test, get_unaligned_le64(slot + 16), 0ULL);
			}
			sim_reply(s);
		}
		return;
	}
	/* The selected AXI and XL tables must finish before scratch writes. */
	KUNIT_EXPECT_TRUE(s->test, !s->boot.ran || s->reset);
	s->tuning_writes++;
	if (w == ANE_H16_XL)
		s->xl_writes++;
	else if (off < sizeof(s->engine))
		s->engine[off / 4] = value;
}

static void sim_write64(void *ctx, enum ane_h16_window w, u32 off, u64 value)
{
	struct ane_h16_sim *s = ctx;

	if (w == ANE_H16_DAPF) {
		sim_check_dapf(s, off, 8);
		/* The hardware drops the low two bits of an end. */
		if (off % s->mmio.dapf_stride == 0x10)
			value &= s->mmio.dapf_end_mask;
		s->dapf[off / 4] = lower_32_bits(value);
		s->dapf[off / 4 + 1] = upper_32_bits(value);
		if (s->dapf_writes < ARRAY_SIZE(s->dapf_order))
			s->dapf_order[s->dapf_writes] = off;
		s->dapf_writes++;
		return;
	}
	KUNIT_EXPECT_EQ(s->test, w, ANE_H16_ENGINE);
	if (off == s->build.ce && !s->ce_timeout)
		s->ce_ack = value;
}

static void sim_delay(void *ctx, unsigned int us)
{
	((struct ane_h16_sim *)ctx)->now += (u64)us * 1000;
}

static int sim_map(void *ctx)
{
	struct ane_h16_sim *s = ctx;

	s->maps++;
	return s->fail_map ? -ENOMEM : 0;
}

static int sim_alloc_at(void *ctx, u64 dva, size_t size, struct ane_fw_surface *out)
{
	struct ane_h16_sim *s = ctx;

	/* Sparse simulated heap. The engine only accesses pool and IPC bytes. */
	out->cpu = kunit_kzalloc(s->test, min_t(size_t, size, SZ_256K), GFP_KERNEL);
	if (!out->cpu)
		return -ENOMEM;
	out->dva = dva;
	out->size = size;
	s->allocations++;
	if (dva == s->build.ipc_dva)
		sim_publish_table(out->cpu, dva);
	return 0;
}

static int sim_pin(void *ctx)
{
	((struct ane_h16_sim *)ctx)->pins++;
	return 0;
}

static int sim_mailbox_send(void *ctx, u64 word0, u64 word1)
{
	struct ane_h16_sim *s = ctx;
	static const u8 types[] = { 2, 8, 8, 5, 6, 0xb, 5 };

	KUNIT_EXPECT_LT(s->test, s->mailbox_tx, ARRAY_SIZE(types));
	if (s->mailbox_tx < ARRAY_SIZE(types))
		KUNIT_EXPECT_EQ(s->test, (word0 >> 52) & 0xff, (u64)types[s->mailbox_tx]);
	KUNIT_EXPECT_EQ(s->test, word1, 0ULL);
	s->mailbox_tx++;
	return 0;
}

static void sim_mailbox(void *ctx)
{
	struct ane_h16_sim *s = ctx;
	static const u64 offered[] = {
		ANE_FW_RTK_TYPE(1) | (12 << 16) | 12,
		ANE_FW_RTK_TYPE(8) | BIT(0) | BIT(1),
		ANE_FW_RTK_TYPE(8) | BIT_ULL(32) | ANE_FW_RTK_LAST | BIT(0),
		ANE_FW_RTK_TYPE(7) | 0x20,
		ANE_FW_RTK_TYPE(0xb) | 0x20,
	};
	unsigned int i;

	s->mailbox_polls++;
	if (!s->offered_mailbox || !s->boot.ran || s->mailbox_tx)
		return;
	for (i = 0; i < ARRAY_SIZE(offered); i++) {
		int ret = ane_fw_rtk_receive(&s->rtk, offered[i], 0, s, sim_mailbox_send);

		KUNIT_EXPECT_EQ(s->test, ret, 0);
	}
}

static void sim_log(void *ctx, const char *item, u64 a, u64 b)
{
	struct ane_h16_sim *s = ctx;

	if (!strcmp(item, "startup SHAREDMALLOC head/slots"))
		s->timeout_head = a;
	if (!strcmp(item, "startup SHAREDMALLOC slot/word0"))
		s->timeout_slots++;
	if (!strcmp(item, "startup SHAREDMALLOC word1/word2"))
		s->timeout_bodies++;
	if (!strcmp(item, "startup timeout scratch"))
		s->timeout_scratch++;
	if (!strcmp(item, "quarantine scratch"))
		s->quarantine_reads++;
	if (!strcmp(item, "startup CPU control/status"))
		s->stop_control = a;
}

static bool sim_powered(void *ctx)
{
	return !((struct ane_h16_sim *)ctx)->unpowered;
}

/* A verified power cycle: the CPU is held in reset until the next RUN. */
static int sim_restart(void *ctx)
{
	struct ane_h16_sim *s = ctx;

	s->power_cycles++;
	if (s->restart_error)
		return s->restart_error;
	s->reset = true;
	s->cpu = 0;
	s->scratch_writes = 0;
	s->tuning_writes = 0;
	s->command_count = 0;
	memset(s->scratch, 0, sizeof(s->scratch));
	if (s->stall_startups)
		s->stall_startups--;
	s->no_startup_reply = s->stall_startups > 0;
	return 0;
}

static void sim_quarantine(void *ctx, int error)
{
	((struct ane_h16_sim *)ctx)->quarantines++;
}

static u64 sim_load64(void *ctx, const void *p)
{
	return get_unaligned_le64(p);
}

static void sim_store64(void *ctx, void *p, u64 value)
{
	put_unaligned_le64(value, p);
}

static void sim_barrier(void *ctx)
{
}

static u32 sim_engine_read(void *ctx, u32 off)
{
	return sim_read32(ctx, ANE_H16_ENGINE, off);
}

static void sim_engine_write(void *ctx, u32 off, u32 value)
{
	sim_write32(ctx, ANE_H16_ENGINE, off, value);
}

static u64 sim_now(void *ctx)
{
	return ((struct ane_h16_sim *)ctx)->now;
}

static void sim_relax(void *ctx, u64 waited_ns)
{
	sim_delay(ctx, 1000);
}

static int sim_alloc_shared(void *ctx, size_t size, u32 tag, struct ane_fw_surface *out)
{
	struct ane_h16_sim *s = ctx;

	return sim_alloc_at(ctx, s->build.shared_dva, size, out);
}

static void sim_refused(void *ctx, u64 w0, u64 w1, u64 w2, int error)
{
}

static const struct ane_h16_boot_ops sim_ops = {
	.read32 = sim_read32,
	.read64 = sim_read64,
	.write32 = sim_write32,
	.write64 = sim_write64,
	.delay_us = sim_delay,
	.map = sim_map,
	.alloc_at = sim_alloc_at,
	.pin = sim_pin,
	.mailbox = sim_mailbox,
	.log = sim_log,
	.quarantine = sim_quarantine,
};

static const struct ane_fw_transport_ops sim_transport_ops = {
	.load64 = sim_load64,
	.store64 = sim_store64,
	.read_barrier = sim_barrier,
	.write_barrier = sim_barrier,
	.read32 = sim_engine_read,
	.write32 = sim_engine_write,
	.now_ns = sim_now,
	.relax = sim_relax,
	.alloc = sim_alloc_shared,
	.refused = sim_refused,
};

/* As the backend: only while the startup commands are out. */
static bool sim_stopped(void *ctx)
{
	struct ane_h16_sim *s = ctx;

	s->stopped_checks++;
	return s->boot.starting && !(s->cpu & s->build.run);
}

static void sim_use_stopped(struct ane_h16_sim *s)
{
	s->transport = sim_transport_ops;
	s->transport.stopped = sim_stopped;
	s->boot.transport_ops = &s->transport;
}

/* A programmed bank, ends without their low two bits. */
static void sim_program_dapf(struct ane_h16_sim *s)
{
	unsigned int i;

	for (i = 0; i < s->soc.dapf_count; i++) {
		u32 *p = &s->dapf[(s->mmio.dapf_records + i * s->mmio.dapf_stride) / 4];
		u64 end = s->boot.ranges[i][1] & s->mmio.dapf_end_mask;

		p[0] = 0x31;
		p[1] = 0;
		p[2] = lower_32_bits(s->boot.ranges[i][0]);
		p[3] = upper_32_bits(s->boot.ranges[i][0]);
		p[4] = lower_32_bits(end);
		p[5] = upper_32_bits(end);
		p[8] = 0xabcd0001;
	}
}

static int sim_init(struct kunit *test)
{
	struct ane_h16_sim *s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	unsigned int i;

	if (!s)
		return -ENOMEM;
	test->priv = s;
	s->test = test;
	s->build = ane_h16_build_26a428;
	s->mmio = ane_h16_mmio_t6041[0];
	s->dart_params = 0x2a2a0202;
	s->soc.h16_mmio = &s->mmio;
	s->soc.h16_mmio_count = 1;
	/* Separate DART header and DAPF record windows. */
	s->soc.dart_params = 8;
	s->soc.dapf_dart = 0;
	s->soc.dapf_offset = 0x10000;
	s->soc.dapf_size = SZ_16K;
	s->soc.dapf_count = 4;
	s->soc.has_xl = true;
	s->soc.scratch = 0x1880020;
	s->soc.doorbell = 0x1884000;
	s->soc.irq_status = 0x188c000;
	s->soc.irq_ack = 0x1890000;
	s->soc.power_words = sim_power_words;
	s->soc.power_count = ARRAY_SIZE(sim_power_words);
	s->boot.soc = &s->soc;
	s->boot.build = &s->build;
	s->boot.ops = &sim_ops;
	s->boot.transport_ops = &sim_transport_ops;
	s->boot.ctx = s;
	for (i = 0; i < ANE_FW_DAPF_MAX; i++) {
		s->boot.ranges[i][0] = 0x500000000ULL + i * SZ_16K;
		s->boot.ranges[i][1] = s->boot.ranges[i][0] + 3;
	}
	s->expect_tuning = 103;
	/* The J616s bank after iBoot: power-on values, flags 0. */
	for (i = 0; i < ARRAY_SIZE(s->dapf); i++)
		s->dapf[i] = i % 16 ? 0x5a5a0000 | i : 0;
	s->dapf_stuck = -1;
	s->channel_request = 8;
	s->ipc_request = 0xc440;
	s->heap_request = 0xf00000;
	s->tick = 25000000;
	s->refuse_id = 0xffff;
	return 0;
}

static void ane_h16_happy_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	u8 *p;

	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), 0);
	KUNIT_EXPECT_TRUE(test, s->boot.ran);
	KUNIT_EXPECT_FALSE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->pins, 1U);
	KUNIT_EXPECT_EQ(test, s->command_count, 4U);
	KUNIT_EXPECT_TRUE(test, s->shared_replied);
	KUNIT_EXPECT_GT(test, s->mailbox_polls, 0U);
	KUNIT_EXPECT_EQ(test, s->boot.tick, 25000000U);
	KUNIT_EXPECT_EQ(test, s->boot.resource_info[99], (u8)0x5a);
	p = (u8 *)s->boot.pool.cpu + s->build.init_offset;
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(p), 0ULL);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(p + 8), s->build.ipc_dva);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(p + 0x10), 0x380000ULL);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(p + 0x18), 0xfc80000ULL);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(p + 0x28), 0xf00000ULL);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(p + 0x50), 0xc441ULL);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(p + 0x12c), 4U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(p + 0x38), 0ULL);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -EBUSY);
}

/*
 * A cold boot after sleep runs the boot again on the surfaces it mapped
 * before: the same DVA and size is reused, cleared, with no new mapping;
 * anything else is allocated.
 */
static void ane_h16_alloc_reuse_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	struct ane_fw_surface surface = {};
	u64 dva = s->build.pool_dva;
	void *cpu;
	u32 allocations;

	KUNIT_ASSERT_EQ(test, ane_h16_alloc(&s->boot, dva, SZ_16K, &surface), 0);
	cpu = surface.cpu;
	memset(cpu, 0x5a, SZ_16K);
	allocations = s->allocations;
	KUNIT_ASSERT_EQ(test, ane_h16_alloc(&s->boot, dva, SZ_16K, &surface), 0);
	KUNIT_EXPECT_EQ(test, s->allocations, allocations);
	KUNIT_EXPECT_PTR_EQ(test, surface.cpu, cpu);
	KUNIT_EXPECT_FALSE(test, memchr_inv(cpu, 0, SZ_16K));
	surface.size = SZ_32K;
	KUNIT_ASSERT_EQ(test, ane_h16_alloc(&s->boot, dva, SZ_16K, &surface), 0);
	KUNIT_EXPECT_EQ(test, s->allocations, allocations + 1);
}

static void ane_h16_no_ready_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->no_ready = true;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -ETIMEDOUT);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->quarantines, 1U);
	KUNIT_EXPECT_EQ(test, s->allocations, 1U);
	KUNIT_EXPECT_EQ(test, s->now, 1000000000ULL);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -EBUSY);
}

static void ane_h16_wrong_table_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->wrong_table = true;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -EPROTO);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_FALSE(test, s->boot.transport.armed);
}

static void ane_h16_stale_table_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->stale_table = true;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -ETIMEDOUT);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_FALSE(test, s->boot.transport.armed);
}

static void ane_h16_oversized_ipc_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->ipc_request = U32_MAX;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -E2BIG);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->allocations, 1U);
}

static void ane_h16_oversized_heap_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->heap_request = U32_MAX;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -E2BIG);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
}

static void ane_h16_channel_limit_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->channel_request = 0x21;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -E2BIG);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
}

static void ane_h16_locked_dapf_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->lock = BIT(2);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_MAP), -EPERM);
	KUNIT_EXPECT_FALSE(test, s->boot.ran);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 0U);
	KUNIT_EXPECT_EQ(test, s->pins, 0U);
}

static void ane_h16_nonzero_ack_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->refuse_id = 0;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -EREMOTEIO);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->command_count, 2U);
	KUNIT_EXPECT_EQ(test, s->pins, 1U);
}

static void ane_h16_preflight_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_PREFLIGHT), 0);
	KUNIT_EXPECT_PTR_EQ(test, s->boot.mmio, &s->mmio);
	KUNIT_EXPECT_EQ(test, s->maps, 0U);
	KUNIT_EXPECT_EQ(test, s->dapf_reads, 20U);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 0U);
	KUNIT_EXPECT_EQ(test, s->tuning_writes, 0U);
	KUNIT_EXPECT_EQ(test, s->scratch_writes, 0U);
}

static void ane_h16_default_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	KUNIT_EXPECT_EQ(test, ane_h16_stage_parse(NULL), ANE_H16_REFUSE);
	KUNIT_EXPECT_EQ(test, ane_h16_stage_parse("unknown"), -EINVAL);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_REFUSE), -EOPNOTSUPP);
	KUNIT_EXPECT_FALSE(test, s->boot.attempted);
	KUNIT_EXPECT_EQ(test, s->scratch_writes, 0U);
}

static void ane_h16_ce_timeout_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->ce_timeout = true;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -ETIMEDOUT);
	KUNIT_EXPECT_FALSE(test, s->boot.ran);
	KUNIT_EXPECT_EQ(test, s->scratch_writes, 0U);
	KUNIT_EXPECT_EQ(test, s->now, 50010000ULL);
}

static void ane_h16_zero_tick_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->tick = 0;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -EPROTO);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
}

static void ane_h16_scratch3_timeout_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->no_clear = true;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -ETIMEDOUT);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_TRUE(test, s->boot.transport.armed);
	KUNIT_EXPECT_EQ(test, s->command_count, 0U);
}

/* A timeout snapshots the ring without answering, moving it or trying again. */
static void ane_h16_startup_timeout_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	struct ane_fw_transport *t = &s->boot.transport;
	u8 before[ANE_FW_SLOT_SIZE];
	u8 *slot;

	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), 0);
	slot = ane_fw_transport_slot(t, ANE_FW_SHAREDMALLOC);
	/* An earlier reply still belongs to firmware; the host must leave it. */
	put_unaligned_le64(s->build.shared_dva | 1, slot);
	put_unaligned_le64(0, slot + 8);
	put_unaligned_le64(1, slot + 16);
	memcpy(before, slot, sizeof(before));
	s->no_startup_reply = true;
	s->build.command_timeout_ns = 2000000;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -ETIMEDOUT);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->command_count, 1U);
	KUNIT_EXPECT_TRUE(test, t->pending[ANE_FW_IO].active);
	KUNIT_EXPECT_EQ(test, t->head[ANE_FW_SHAREDMALLOC], 0U);
	KUNIT_EXPECT_EQ(test, s->timeout_head, 0ULL);
	KUNIT_EXPECT_EQ(test, s->timeout_slots, t->channels[ANE_FW_SHAREDMALLOC].slots);
	KUNIT_EXPECT_EQ(test, s->timeout_bodies, s->timeout_slots);
	KUNIT_EXPECT_EQ(test, s->timeout_scratch, 8U);
	KUNIT_EXPECT_MEMEQ(test, before, slot, sizeof(before));
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -EBUSY);
	KUNIT_EXPECT_EQ(test, s->command_count, 1U);
}

static void sim_use_restart(struct ane_h16_sim *s, u32 stalls, unsigned int budget)
{
	s->ops = sim_ops;
	s->ops.restart = sim_restart;
	s->ops.powered = sim_powered;
	/* Reuse clears the surfaces; keep them inside the sparse simulated heap. */
	s->heap_request = SZ_64K;
	s->boot.ops = &s->ops;
	s->boot.max_restarts = budget;
	s->stall_startups = stalls;
	s->no_startup_reply = stalls > 0;
	s->build.command_timeout_ns = 2000000;
}

/* A stalled startup is power cycled and booted again; the second one completes. */
static void ane_h16_startup_restart_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	u32 allocations;

	sim_use_restart(s, 1, 2);
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), 0);
	allocations = s->allocations;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), 0);
	KUNIT_EXPECT_EQ(test, s->boot.stage, ANE_H16_STARTUP);
	KUNIT_EXPECT_FALSE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->quarantines, 0U);
	KUNIT_EXPECT_EQ(test, s->boot.restarts, 1U);
	KUNIT_EXPECT_EQ(test, s->power_cycles, 1U);
	KUNIT_EXPECT_EQ(test, s->command_count, 4U);
	KUNIT_EXPECT_EQ(test, s->boot.tick, 25000000U);
	/*
	 * Pool, IPC and heap are reused: the only new mapping is the shared
	 * allocation the stalled first boot never got as far as requesting.
	 */
	KUNIT_EXPECT_EQ(test, s->allocations, allocations + 1);
	KUNIT_EXPECT_EQ(test, s->timeout_scratch, 8U);
}

/* The budget bounds the cycles; the last timeout quarantines as before. */
static void ane_h16_startup_restart_budget_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	sim_use_restart(s, 4, 2);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, s->boot.restarts, 2U);
	KUNIT_EXPECT_EQ(test, s->power_cycles, 2U);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->quarantines, 1U);
	KUNIT_EXPECT_EQ(test, s->timeout_scratch, 24U);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -EBUSY);
}

/*
 * A CPU that loses RUN during startup can answer nothing: the wait ends at
 * once with -ESHUTDOWN and the evidence names the control word, instead of
 * waiting out the command deadline.
 */
static void ane_h16_startup_stopped_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	u64 start;

	sim_use_stopped(s);
	s->build.command_timeout_ns = 3ULL * NSEC_PER_SEC;
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), 0);
	/* Running and answering: never reported stopped. */
	KUNIT_EXPECT_FALSE(test, sim_stopped(s));
	s->no_startup_reply = true;
	s->cpu = 0;
	start = s->now;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -ESHUTDOWN);
	KUNIT_EXPECT_LT(test, s->now - start, 2ULL * NSEC_PER_MSEC);
	KUNIT_EXPECT_EQ(test, s->stop_control, 0ULL);
	KUNIT_EXPECT_EQ(test, s->timeout_scratch, 8U);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_FALSE(test, s->boot.starting);
}

/* A stopped CPU gets the same verified restart as a timed-out one. */
static void ane_h16_startup_stopped_restart_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	u64 start;

	sim_use_restart(s, 1, 2);
	sim_use_stopped(s);
	s->build.command_timeout_ns = 3ULL * NSEC_PER_SEC;
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), 0);
	s->cpu = 0;
	start = s->now;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), 0);
	KUNIT_EXPECT_LT(test, s->now - start, 100ULL * NSEC_PER_MSEC);
	KUNIT_EXPECT_EQ(test, s->boot.restarts, 1U);
	KUNIT_EXPECT_EQ(test, s->power_cycles, 1U);
	KUNIT_EXPECT_FALSE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->boot.stage, ANE_H16_STARTUP);
	KUNIT_EXPECT_FALSE(test, s->boot.starting);
}

/* An unverified power cycle is not followed by another release of the CPU. */
static void ane_h16_startup_restart_refused_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	sim_use_restart(s, 1, 2);
	s->restart_error = -EIO;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -EIO);
	KUNIT_EXPECT_EQ(test, s->power_cycles, 1U);
	KUNIT_EXPECT_EQ(test, s->command_count, 1U);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->quarantines, 1U);
	/* Still powered: the quarantine reads its evidence. */
	KUNIT_EXPECT_EQ(test, s->quarantine_reads, 8U);
}

/* A restart that left the engine off quarantines without touching it. */
static void ane_h16_startup_restart_unpowered_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	sim_use_restart(s, 1, 2);
	s->restart_error = -EIO;
	s->unpowered = true;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -EIO);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->quarantines, 1U);
	KUNIT_EXPECT_EQ(test, s->quarantine_reads, 0U);
}

/*
 * The boot after system sleep stalls too: its restarts come from a budget of
 * their own, so a restart spent at bring-up does not use them up.
 */
static void ane_h16_resume_restart_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	unsigned int used = 0;

	sim_use_restart(s, 1, 2);
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), 0);
	KUNIT_ASSERT_EQ(test, s->boot.restarts, 1U);
	/* Cold boot after sleep (power cycle, scratch boot), then two stalls. */
	s->stall_startups = 3;
	KUNIT_ASSERT_EQ(test, ane_h16_restart(&s->boot), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_startup_restarting(&s->boot, &used, "resume"), 0);
	KUNIT_EXPECT_EQ(test, used, 2U);
	KUNIT_EXPECT_EQ(test, s->boot.restarts, 1U);
	KUNIT_EXPECT_EQ(test, s->power_cycles, 4U);
	KUNIT_EXPECT_EQ(test, s->command_count, 4U);
	KUNIT_EXPECT_EQ(test, s->quarantines, 0U);
}

/* Past its budget, a stall after sleep is returned to the caller unquarantined. */
static void ane_h16_resume_restart_budget_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	unsigned int used = 0;

	sim_use_restart(s, 0, 2);
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), 0);
	s->stall_startups = 5;
	KUNIT_ASSERT_EQ(test, ane_h16_restart(&s->boot), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_startup_restarting(&s->boot, &used, "resume"),
			-ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, used, 2U);
	KUNIT_EXPECT_EQ(test, s->power_cycles, 3U);
	/* The backend's resume path quarantines; the helper leaves that to it. */
	KUNIT_EXPECT_EQ(test, s->quarantines, 0U);
	KUNIT_EXPECT_FALSE(test, s->boot.quarantined);
}

/* Without the op (or with no budget) a timeout ends the attempt, as before. */
static void ane_h16_startup_no_budget_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	sim_use_restart(s, 1, 0);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, s->power_cycles, 0U);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
}

/* Stages advance one write at a time, each at most once, in one firmware session. */
static void ane_h16_incremental_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_PREFLIGHT), 0);
	KUNIT_EXPECT_EQ(test, s->maps, 0U);
	KUNIT_EXPECT_EQ(test, s->scratch_writes, 0U);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_PREFLIGHT), -EBUSY);
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_MAP), 0);
	KUNIT_EXPECT_EQ(test, s->maps, 1U);
	KUNIT_EXPECT_EQ(test, s->scratch_writes, 0U);
	KUNIT_EXPECT_FALSE(test, s->boot.ran);
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), 0);
	KUNIT_EXPECT_TRUE(test, s->boot.ran);
	KUNIT_EXPECT_EQ(test, s->boot.stage, ANE_H16_BOOT);
	KUNIT_EXPECT_EQ(test, s->command_count, 0U);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_MAP), -EBUSY);
	/* The program stage belongs to the backend; here it means startup. */
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_PROGRAM), 0);
	KUNIT_EXPECT_EQ(test, s->boot.stage, ANE_H16_STARTUP);
	KUNIT_EXPECT_EQ(test, s->command_count, 4U);
	KUNIT_EXPECT_EQ(test, s->maps, 1U);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -EBUSY);
	KUNIT_EXPECT_STREQ(test, ane_h16_stage_name(s->boot.stage), "startup");
}

static void ane_h16_mailbox_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->offered_mailbox = true;
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), 0);
	KUNIT_EXPECT_EQ(test, s->rtk.phase, ANE_FW_RTK_READY);
	KUNIT_EXPECT_EQ(test, s->mailbox_tx, 7U);
}

static void ane_h16_no_heap_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	u8 *record;

	s->heap_request = 0;
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), 0);
	KUNIT_EXPECT_EQ(test, s->allocations, 2U);
	record = (u8 *)s->boot.pool.cpu + s->build.init_offset;
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(record + 0x20), 0ULL);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(record + 0x28), 0ULL);
}

static void ane_h16_power_actual_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->bad_power = true;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_PREFLIGHT), -EHOSTDOWN);
	KUNIT_EXPECT_EQ(test, s->scratch_writes, 0U);
	KUNIT_EXPECT_EQ(test, s->pins, 0U);
}

static void ane_h16_rvbar_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->bad_rvbar = true;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -EBUSY);
	KUNIT_EXPECT_EQ(test, s->scratch_writes, 0U);
	KUNIT_EXPECT_EQ(test, s->maps, 0U);
}

static void ane_h16_dart_fault_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->fault_after_run = true;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -EIO);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->pins, 1U);
	KUNIT_EXPECT_EQ(test, s->quarantines, 1U);
}

static void ane_h16_fault_address_on_demand_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->boot.mmio = &s->mmio;
	KUNIT_EXPECT_EQ(test, ane_h16_faults(&s->boot, false), 0);
	KUNIT_EXPECT_EQ(test, s->fault_address_reads, 0U);
	s->fault = BIT(31);
	KUNIT_EXPECT_EQ(test, ane_h16_faults(&s->boot, false), -EIO);
	KUNIT_EXPECT_EQ(test, s->fault_address_reads, 6U);
	s->fault = 0;
	KUNIT_EXPECT_EQ(test, ane_h16_faults(&s->boot, true), 0);
	KUNIT_EXPECT_EQ(test, s->fault_address_reads, 12U);
}

static void ane_h16_map_failure_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->fail_map = true;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -ENOMEM);
	KUNIT_EXPECT_FALSE(test, s->boot.ran);
	KUNIT_EXPECT_FALSE(test, s->boot.quarantined);
	KUNIT_EXPECT_EQ(test, s->pins, 0U);
}

/* Every DART is observed; only bit 0 prevents fresh translation ownership. */
static void ane_h16_dart_protection_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	unsigned int i;

	KUNIT_EXPECT_EQ(test, ane_h16_dart_protection(&s->boot, true), -ENODEV);
	KUNIT_EXPECT_EQ(test, s->protect_reads, 0U);
	KUNIT_ASSERT_EQ(test, ane_h16_select_mmio(&s->boot), 0);
	for (i = 0; i < 3; i++) {
		s->protect[i] = BIT(0);
		s->protect_reads = 0;
		KUNIT_EXPECT_EQ(test, ane_h16_dart_protection(&s->boot, true), -EPERM);
		KUNIT_EXPECT_EQ(test, s->protect_reads, 7U);
		KUNIT_EXPECT_EQ(test, ane_h16_dart_protection(&s->boot, false), 0);
		s->protect[i] = BIT(1) | BIT(2);
	}
	KUNIT_EXPECT_EQ(test, ane_h16_dart_protection(&s->boot, true), 0);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 0U);
	KUNIT_EXPECT_EQ(test, s->maps, 0U);
	KUNIT_EXPECT_EQ(test, s->scratch_writes, 0U);
}

static void ane_h16_locked_matching_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	sim_program_dapf(s);
	s->lock = BIT(2);
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_MAP), 0);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 0U);
}

/* Preflight only reads the unprogrammed bank; map writes it in t8110 order. */
static void ane_h16_dapf_program_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	static const u32 order[] = { 4, 8, 0x10, 0, 0x20 };
	unsigned int i;

	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_PREFLIGHT), 0);
	KUNIT_EXPECT_EQ(test, s->protect_reads, 7U);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 0U);
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_MAP), 0);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 20U);
	for (i = 0; i < ARRAY_SIZE(order); i++)
		KUNIT_EXPECT_EQ(test, s->dapf_order[i], order[i]);
	for (i = 0; i < 4; i++) {
		u32 *p = &s->dapf[(s->mmio.dapf_records + i * s->mmio.dapf_stride) / 4];

		KUNIT_EXPECT_TRUE(test, ane_h16_dapf_record(&s->boot, i));
		KUNIT_EXPECT_EQ(test, p[0], 0x31U);
		KUNIT_EXPECT_EQ(test, p[1], 0U);
		KUNIT_EXPECT_EQ(test, p[8], 1U);
	}
	KUNIT_EXPECT_FALSE(test, s->boot.ran);
}

/* Only records that differ are written. */
static void ane_h16_dapf_partial_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	sim_program_dapf(s);
	s->dapf[(s->mmio.dapf_records + 2 * s->mmio.dapf_stride + 8) / 4] ^= 0x4000;
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_MAP), 0);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 5U);
	KUNIT_EXPECT_EQ(test, s->dapf_order[1], s->mmio.dapf_records + 2 * s->mmio.dapf_stride + 8);
}

/* A record that does not read back fails map before RUN. */
static void ane_h16_dapf_readback_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->dapf_stuck = 1;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_MAP), -EIO);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 10U);
	KUNIT_EXPECT_FALSE(test, s->boot.ran);
	KUNIT_EXPECT_EQ(test, s->pins, 0U);
}

/* A DART revision without a table stops after the PARAMS3 reads. */
static void ane_h16_unknown_revision_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->dart_params = 0x2a2a0203;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_PREFLIGHT), -ENODEV);
	KUNIT_EXPECT_NULL(test, s->boot.mmio);
	KUNIT_EXPECT_EQ(test, s->dapf_reads, 0U);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 0U);
	KUNIT_EXPECT_EQ(test, s->maps, 0U);
}

static void ane_h16_heap_overlap_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->heap_request = SZ_16M;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_BOOT), -E2BIG);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
}

static void ane_h16_native_free_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->lifecycle_mode = 1;
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), 0);
	KUNIT_EXPECT_TRUE(test, s->lifecycle_requested);
	KUNIT_EXPECT_TRUE(test, s->boot.transport.shared_retired[0]);
	KUNIT_EXPECT_TRUE(test, s->boot.transport.shared[0].cpu);
}

static void ane_h16_control_refusal_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->lifecycle_mode = 2;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -EOPNOTSUPP);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_FALSE(test, s->boot.transport.shared_retired[0]);
}

static void ane_h16_foreign_free_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->lifecycle_mode = 3;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), -ENOENT);
	KUNIT_EXPECT_TRUE(test, s->boot.quarantined);
	KUNIT_EXPECT_FALSE(test, s->boot.transport.shared_retired[0]);
}

/* The T8132: five DAPF records, no XL window, its own engine writes. */
static void sim_use_t8132(struct ane_h16_sim *s)
{
	s->build = ane_h16_build_t8132_25g83;
	s->soc.dapf_count = 5;
	s->soc.has_xl = false;
	s->expect_tuning = ARRAY_SIZE(ane_t8132_axi) + ARRAY_SIZE(ane_t8132_axi_wd) +
			   ARRAY_SIZE(ane_t8132_axi_lock);
}

/* Map programs and reads back all five T8132 records. */
static void ane_h16_dapf_five_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	unsigned int i;

	sim_use_t8132(s);
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_MAP), 0);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 25U);
	for (i = 0; i < 5; i++)
		KUNIT_EXPECT_TRUE(test, ane_h16_dapf_record(&s->boot, i));
}

/* A fifth record that does not read back fails map before RUN. */
static void ane_h16_dapf_five_readback_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	sim_use_t8132(s);
	s->dapf_stuck = 4;
	KUNIT_EXPECT_EQ(test, ane_h16_run(&s->boot, ANE_H16_MAP), -EIO);
	KUNIT_EXPECT_EQ(test, s->dapf_writes, 25U);
	KUNIT_EXPECT_FALSE(test, s->boot.ran);
}

/* A platform without the XL window skips the build's XL tunables. */
static void ane_h16_no_xl_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	s->soc.has_xl = false;
	s->expect_tuning = 103 - ARRAY_SIZE(ane_h16_xl);
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), 0);
	KUNIT_EXPECT_EQ(test, s->xl_writes, 0U);
}

/* The T8132 boots with its own engine writes only: no XL or C1PPT. */
static void ane_h16_t8132_tunables_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;

	sim_use_t8132(s);
	KUNIT_ASSERT_EQ(test, ane_h16_run(&s->boot, ANE_H16_STARTUP), 0);
	KUNIT_EXPECT_EQ(test, s->xl_writes, 0U);
	KUNIT_EXPECT_EQ(test, s->engine[0], 0x8000001bU);
	KUNIT_EXPECT_EQ(test, s->engine[0xdfc / 4], 1U);
}

/*
 * Within one power cycle the T8132 engine set stays locked: a second
 * preparation writes nothing when the values match and refuses a changed one.
 */
static void ane_h16_t8132_locked_test(struct kunit *test)
{
	struct ane_h16_sim *s = test->priv;
	u32 writes;

	sim_use_t8132(s);
	KUNIT_ASSERT_EQ(test, ane_h16_prepare(&s->boot), 0);
	writes = s->tuning_writes;
	KUNIT_EXPECT_EQ(test, writes, s->expect_tuning);
	KUNIT_EXPECT_EQ(test, ane_h16_prepare(&s->boot), 0);
	KUNIT_EXPECT_EQ(test, s->tuning_writes, writes);
	s->engine[0x4 / 4] ^= 0x10000;
	KUNIT_EXPECT_EQ(test, ane_h16_prepare(&s->boot), -EPERM);
}

static struct kunit_case ane_h16_boot_cases[] = {
	KUNIT_CASE(ane_h16_alloc_reuse_test),
	KUNIT_CASE(ane_h16_happy_test),
	KUNIT_CASE(ane_h16_dart_protection_test),
	KUNIT_CASE(ane_h16_locked_matching_test),
	KUNIT_CASE(ane_h16_dapf_program_test),
	KUNIT_CASE(ane_h16_dapf_partial_test),
	KUNIT_CASE(ane_h16_dapf_readback_test),
	KUNIT_CASE(ane_h16_dapf_five_test),
	KUNIT_CASE(ane_h16_dapf_five_readback_test),
	KUNIT_CASE(ane_h16_no_xl_test),
	KUNIT_CASE(ane_h16_t8132_tunables_test),
	KUNIT_CASE(ane_h16_t8132_locked_test),
	KUNIT_CASE(ane_h16_unknown_revision_test),
	KUNIT_CASE(ane_h16_heap_overlap_test),
	KUNIT_CASE(ane_h16_native_free_test),
	KUNIT_CASE(ane_h16_control_refusal_test),
	KUNIT_CASE(ane_h16_foreign_free_test),
	KUNIT_CASE(ane_h16_mailbox_test),
	KUNIT_CASE(ane_h16_incremental_test),
	KUNIT_CASE(ane_h16_no_heap_test),
	KUNIT_CASE(ane_h16_power_actual_test),
	KUNIT_CASE(ane_h16_rvbar_test),
	KUNIT_CASE(ane_h16_dart_fault_test),
	KUNIT_CASE(ane_h16_fault_address_on_demand_test),
	KUNIT_CASE(ane_h16_map_failure_test),
	KUNIT_CASE(ane_h16_no_ready_test),
	KUNIT_CASE(ane_h16_wrong_table_test),
	KUNIT_CASE(ane_h16_stale_table_test),
	KUNIT_CASE(ane_h16_oversized_ipc_test),
	KUNIT_CASE(ane_h16_oversized_heap_test),
	KUNIT_CASE(ane_h16_channel_limit_test),
	KUNIT_CASE(ane_h16_locked_dapf_test),
	KUNIT_CASE(ane_h16_nonzero_ack_test),
	KUNIT_CASE(ane_h16_preflight_test),
	KUNIT_CASE(ane_h16_default_test),
	KUNIT_CASE(ane_h16_ce_timeout_test),
	KUNIT_CASE(ane_h16_zero_tick_test),
	KUNIT_CASE(ane_h16_scratch3_timeout_test),
	KUNIT_CASE(ane_h16_startup_timeout_test),
	KUNIT_CASE(ane_h16_startup_restart_test),
	KUNIT_CASE(ane_h16_startup_restart_budget_test),
	KUNIT_CASE(ane_h16_startup_stopped_test),
	KUNIT_CASE(ane_h16_startup_stopped_restart_test),
	KUNIT_CASE(ane_h16_startup_restart_refused_test),
	KUNIT_CASE(ane_h16_startup_restart_unpowered_test),
	KUNIT_CASE(ane_h16_startup_no_budget_test),
	KUNIT_CASE(ane_h16_resume_restart_test),
	KUNIT_CASE(ane_h16_resume_restart_budget_test),
	{}
};

static struct kunit_suite ane_h16_boot_suite = {
	.name = "ane-h16-boot",
	.init = sim_init,
	.test_cases = ane_h16_boot_cases,
};

kunit_test_suite(ane_h16_boot_suite);

MODULE_DESCRIPTION("KUnit simulated H16 cold boot engine");
MODULE_LICENSE("Dual MIT/GPL");
