// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <kunit/test.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/mutex.h>

#include "ane_h16_boot.h"
#include "ane_h16_program.h"

#include "ane_h16_hwx_test_fixture.h"
#include "ane_h16_add128_fixture.h"
#include "ane_h17_mobilenet_fixture.h"

#define FW_IPC_DVA 0x100003d0000ULL
#define FW_POOL_DVA 0x10000384000ULL
#define FW_MAP_DVA 0x10001f00000ULL
#define FW_RING_SLOTS 8
#define FW_REPORTS 0x8000

enum fw_mode {
	FW_OK,
	FW_LOAD_ID_UNCHANGED,
	FW_LOAD_NAK,
	FW_CREATE_BODY,
	FW_REPORT_FOREIGN_UUID,
	FW_REPORT_WRONG_PROCESS,
	FW_REPORT_FAILURE,
	FW_REPORT_STALL,
	/* State 0 at once, then states 3 and 1 after late_polls sleeps. */
	FW_REPORT_LATE,
	/* Reports and the clock are driven by the test under the session mutex. */
	FW_REPORT_PIPELINE,
};

/* A synchronous fake firmware that answers on each doorbell. */
struct prog_fw {
	struct kunit *test;
	struct ane_fw_platform soc;
	struct ane_fw_surface ipc, pool;
	struct ane_fw_transport t;
	struct ane_h16_session s;
	enum fw_mode mode;
	u64 now;
	/* Time each doorbell takes to answer. */
	u64 doorbell_ns;
	u64 late_uuid;
	u32 late_program, late_process;
	unsigned int late_polls;
	u64 next_dva;
	u32 head[ANE_FW_CHANNELS];
	u32 reports;
	/* First IDs; the firmware hands out the lowest free one, reused after teardown. */
	u32 program, process;
	u32 programs_live, processes_live;
	u16 ids[32];
	u32 values[32];
	unsigned int count;
	u8 exec[0xd00];
	size_t exec_length;
	u32 exec_flags[32];
	struct ane_h16_request_key exec_keys[32];
	wait_queue_head_t pipeline_wake;
	unsigned int execs;
	u8 load[ANE_H16_LOAD_SIZE];
	unsigned int maps, writable_maps;
	/* busy(true) and busy(false) calls, and the deepest nesting seen. */
	unsigned int busy_on, busy_off;
	int busy_depth, busy_max;
	/* Cache maintenance calls, when the fake hands out cached surfaces. */
	bool cached;
	unsigned int syncs_device, syncs_cpu;
	u64 last_sync_dva;
	/* The EXECUTE failure trace lines, by step. */
	u64 fail_step, fail_flags, fail_reports, fail_state, fail_other;
	/* EXECUTE of a program or process this firmware never created fails. */
	bool strict;
	unsigned int replay_failed;
	/* ops->hung calls and the error of the last one. */
	unsigned int hung;
	int hung_error;
	/*
	 * Bytes mapped and not unmapped, ops->unmap calls, the most bytes a
	 * nonzero window lets be mapped at once (-ENOSPC beyond), and the
	 * number of the map call that fails with -ENOMEM (0: none).
	 */
	u64 live, window;
	unsigned int unmaps, map_fail_at;
};

static u64 fw_load(void *ctx, const void *p)
{
	return get_unaligned_le64(p);
}

static void fw_store(void *ctx, void *p, u64 value)
{
	put_unaligned_le64(value, p);
}

static void fw_barrier(void *ctx)
{
}

static u32 fw_read32(void *ctx, u32 reg)
{
	return 0;
}

static u8 *fw_slot(struct prog_fw *f, unsigned int ch)
{
	return (u8 *)f->ipc.cpu + f->t.channels[ch].offset + f->head[ch] * ANE_FW_SLOT_SIZE;
}

static void fw_advance(struct prog_fw *f, unsigned int ch)
{
	f->head[ch] = (f->head[ch] + 1) % f->t.channels[ch].slots;
}

static void fw_report(struct prog_fw *f, u64 uuid, u32 program, u32 process, u32 state)
{
	u32 offset = FW_REPORTS + (f->reports++ % 16) * 0x60;
	u8 *r = (u8 *)f->ipc.cpu + offset, *slot = fw_slot(f, ANE_FW_IO_T2H);

	memset(r, 0, 0x60);
	put_unaligned_le32(0xfe00 + (state == 1), r);
	put_unaligned_le16(0x300, r + 4);
	put_unaligned_le64(uuid, r + 8);
	put_unaligned_le32(program, r + 0x10);
	put_unaligned_le32(process, r + 0x14);
	put_unaligned_le32(state, r + 0x1c);
	put_unaligned_le64(0x60, slot + 8);
	put_unaligned_le64(0, slot + 16);
	put_unaligned_le64(f->ipc.dva + offset, slot);
	fw_advance(f, ANE_FW_IO_T2H);
}

/* Reports carry the IDs of the EXECUTE they answer. */
static void fw_execute(struct prog_fw *f, const u8 *cmd)
{
	u64 uuid = get_unaligned_le64(cmd + 0x20);
	u32 program = get_unaligned_le32(cmd + 8), process = get_unaligned_le32(cmd + 0xc);

	if (f->execs < ARRAY_SIZE(f->exec_flags)) {
		f->exec_flags[f->execs] = get_unaligned_le32(cmd + ANE_H16_EXECUTE_FLAGS);
		f->exec_keys[f->execs] = (struct ane_h16_request_key) {
			.uuid = uuid, .program = program, .process = process,
		};
	}
	f->execs++;
	wake_up_all(&f->pipeline_wake);
	switch (f->mode) {
	case FW_REPORT_PIPELINE:
		break;
	case FW_REPORT_FOREIGN_UUID:
		fw_report(f, uuid + 100, program, process, 0);
		fw_report(f, uuid + 100, program, process, 1);
		break;
	case FW_REPORT_WRONG_PROCESS:
		fw_report(f, uuid, program, process, 0);
		fw_report(f, uuid, program, process + 1, 1);
		break;
	case FW_REPORT_FAILURE:
		fw_report(f, uuid, program, process, 0);
		fw_report(f, uuid, program, process, 2);
		break;
	case FW_REPORT_LATE:
		fw_report(f, uuid, program, process, 0);
		f->late_uuid = uuid;
		f->late_program = program;
		f->late_process = process;
		break;
	case FW_REPORT_STALL:
		/* Started on the engine, then never a terminal report. */
		fw_report(f, uuid, program, process, 0);
		fw_report(f, uuid, program, process, 3);
		break;
	default:
		if (f->strict && (program - f->program >= 32 || process - f->process >= 32 ||
				  !(f->programs_live & BIT(program - f->program)) ||
				  !(f->processes_live & BIT(process - f->process)))) {
			fw_report(f, uuid, program, process, 2);
			break;
		}
		fw_report(f, uuid, program, process, 0);
		fw_report(f, uuid, program, process, 3);
		fw_report(f, uuid, program, process, 1);
		break;
	}
}

/* A firmware booted again from the preload: no program or process, new IDs. */
static void fw_cold_boot(struct prog_fw *f)
{
	f->programs_live = 0;
	f->processes_live = 0;
	f->program += 0x10;
	f->process += 0x10;
}

static u32 fw_take_id(u32 *live, u32 base)
{
	unsigned int i = ffz(*live);

	*live |= BIT(i);
	return base + i;
}

static void fw_command(struct prog_fw *f, unsigned int ch)
{
	u8 *slot = fw_slot(f, ch), *cmd;
	u64 dva = get_unaligned_le64(slot), length = get_unaligned_le64(slot + 8);
	u16 id;

	if (dva & 1)
		return;
	cmd = (u8 *)f->pool.cpu + (dva - f->pool.dva);
	id = get_unaligned_le16(cmd + 4);
	if (f->count < ARRAY_SIZE(f->ids)) {
		f->ids[f->count] = id;
		f->values[f->count] = length >= 12 ? get_unaligned_le32(cmd + 8) : 0;
		f->count++;
	}
	switch (id) {
	case ANE_H16_LOAD_PROGRAM:
		memcpy(f->load, cmd, sizeof(f->load));
		if (f->mode == FW_LOAD_NAK)
			put_unaligned_le16(1, cmd + 6);
		else if (f->mode != FW_LOAD_ID_UNCHANGED)
			put_unaligned_le32(fw_take_id(&f->programs_live, f->program),
					   cmd + ANE_H16_LOAD_ID_OFFSET);
		break;
	case ANE_H16_UNLOAD_PROGRAM:
		if (get_unaligned_le32(cmd + 8) - f->program < 32)
			f->programs_live &= ~BIT(get_unaligned_le32(cmd + 8) - f->program);
		break;
	case ANE_H16_TERMINATE_PROCESS:
		if (get_unaligned_le32(cmd + 0xc) - f->process < 32)
			f->processes_live &= ~BIT(get_unaligned_le32(cmd + 0xc) - f->process);
		break;
	case ANE_H16_CREATE_PROCESS:
		KUNIT_EXPECT_FALSE(f->test, memchr_inv(cmd + 0xc, 0, ANE_H16_CREATE_SIZE - 0xc));
		put_unaligned_le32(fw_take_id(&f->processes_live, f->process), cmd + 0xc);
		if (f->mode == FW_CREATE_BODY)
			cmd[0x100] = 1;
		break;
	case ANE_H16_EXECUTE:
		f->exec_length = min_t(size_t, length, sizeof(f->exec));
		memcpy(f->exec, cmd, f->exec_length);
		break;
	}
	put_unaligned_le64(length, slot + 8);
	put_unaligned_le64(0, slot + 16);
	put_unaligned_le64(dva | 1, slot);
	fw_advance(f, ch);
	if (id == ANE_H16_EXECUTE)
		fw_execute(f, cmd);
}

static void fw_write32(void *ctx, u32 reg, u32 value)
{
	struct prog_fw *f = ctx;

	if (reg != f->soc.doorbell)
		return;
	/* Commands take doorbell_ns to answer; returning report slots is free. */
	if (value & (BIT(ANE_FW_IO) | BIT(ANE_FW_DATA_CHAIN)))
		f->now += f->doorbell_ns;
	if (value & BIT(ANE_FW_IO))
		fw_command(f, ANE_FW_IO);
	if (value & BIT(ANE_FW_DATA_CHAIN))
		fw_command(f, ANE_FW_DATA_CHAIN);
}

static u64 fw_now(void *ctx)
{
	return ((struct prog_fw *)ctx)->now;
}

static void fw_relax(void *ctx, u64 waited_ns)
{
	struct prog_fw *f = ctx;

	if (f->mode == FW_REPORT_PIPELINE) {
		usleep_range(50, 100);
		return;
	}
	f->now += 10;
	if (f->late_uuid && !--f->late_polls) {
		fw_report(f, f->late_uuid, f->late_program, f->late_process, 3);
		fw_report(f, f->late_uuid, f->late_program, f->late_process, 1);
		f->late_uuid = 0;
	}
}

static int fw_alloc(void *ctx, size_t size, u32 tag, struct ane_fw_surface *s)
{
	return -ENOMEM;
}

static void fw_refused(void *ctx, u64 w0, u64 w1, u64 w2, int error)
{
}

static const struct ane_fw_transport_ops fw_ops = {
	.load64 = fw_load,
	.store64 = fw_store,
	.read_barrier = fw_barrier,
	.write_barrier = fw_barrier,
	.read32 = fw_read32,
	.write32 = fw_write32,
	.now_ns = fw_now,
	.relax = fw_relax,
	.alloc = fw_alloc,
	.refused = fw_refused,
};

static void fw_kvfree(void *p)
{
	kvfree(p);
}

/* Tests own the memory; a MobileNet kernel region exceeds kmalloc's limit. */
static void *fw_zalloc(struct kunit *test, size_t size)
{
	void *p = kvzalloc(size, GFP_KERNEL);

	if (p && kunit_add_action_or_reset(test, fw_kvfree, p))
		return NULL;
	return p;
}

static int fw_map(void *ctx, size_t size, bool write, struct ane_fw_surface *s)
{
	struct prog_fw *f = ctx;

	if (f->window && size > f->window - f->live)
		return -ENOSPC;
	if (f->map_fail_at && f->maps + 1 == f->map_fail_at) {
		f->map_fail_at = 0;
		return -ENOMEM;
	}
	*s = (struct ane_fw_surface) {
		.cpu = fw_zalloc(f->test, size),
		.dva = f->next_dva,
		.size = size,
		.handle = f->cached ? f->next_dva : 0,
	};
	if (!s->cpu)
		return -ENOMEM;
	f->next_dva += size;
	f->maps++;
	f->writable_maps += write;
	f->live += size;
	return 0;
}

static void fw_unmap(void *ctx, const struct ane_fw_surface *s)
{
	struct prog_fw *f = ctx;

	KUNIT_EXPECT_FALSE(f->test, f->s.asleep || f->s.parked);
	KUNIT_EXPECT_GE(f->test, f->live, (u64)s->size);
	f->live -= s->size;
	f->unmaps++;
	kunit_release_action(f->test, fw_kvfree, s->cpu);
}

static void fw_log(void *ctx, const char *item, u64 a, u64 b)
{
	struct prog_fw *f = ctx;

	if (!strcmp(item, "EXECUTE step/flags")) {
		f->fail_step = a;
		f->fail_flags = b;
	} else if (!strcmp(item, "EXECUTE reports/last state")) {
		f->fail_reports = a;
		f->fail_state = b;
	} else if (!strcmp(item, "EXECUTE other reports")) {
		f->fail_other = a;
	} else if (!strcmp(item, "PROGRAM REPLAY FAILED")) {
		f->replay_failed++;
	}
}

static void fw_sync(void *ctx, const struct ane_fw_surface *s, bool for_device)
{
	struct prog_fw *f = ctx;

	KUNIT_EXPECT_NE(f->test, s->handle, 0ULL);
	if (for_device)
		f->syncs_device++;
	else
		f->syncs_cpu++;
	f->last_sync_dva = s->dva;
}

static void fw_busy(void *ctx, bool busy)
{
	struct prog_fw *f = ctx;

	if (busy) {
		f->busy_on++;
		f->busy_max = max(f->busy_max, ++f->busy_depth);
	} else {
		f->busy_off++;
		f->busy_depth--;
	}
}

static const struct ane_h16_session_ops fw_session_ops = {
	.map = fw_map,
	.log = fw_log,
	.busy = fw_busy,
	.sync = fw_sync,
};

static void fw_hung(void *ctx, int error)
{
	struct prog_fw *f = ctx;

	f->hung++;
	f->hung_error = error;
}

/* As fw_session_ops, with a backend that frees what the session gives back. */
static const struct ane_h16_session_ops fw_session_ops_unmap = {
	.map = fw_map,
	.unmap = fw_unmap,
	.log = fw_log,
	.busy = fw_busy,
	.sync = fw_sync,
	.hung = fw_hung,
};

/* As fw_session_ops, with a backend that recovers a quarantined session. */
static const struct ane_h16_session_ops fw_session_ops_hung = {
	.map = fw_map,
	.log = fw_log,
	.busy = fw_busy,
	.sync = fw_sync,
	.hung = fw_hung,
};

static struct prog_fw *fw_create_family(struct kunit *test, unsigned int family)
{
	static const char * const names[] = {
		"TERMINAL", "IO", "DEBUG", "BUF_H2T", "BUF_T2H",
		"SHAREDMALLOC", "IO_T2H", "DATA_CHAIN_H2T",
	};
	struct prog_fw *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	unsigned int i;

	if (!f)
		return NULL;
	f->test = test;
	init_waitqueue_head(&f->pipeline_wake);
	f->soc.chip_family = family;
	f->soc.hwx_subtype = family == 17 ? ANE_HWX_SUBTYPE_H17 : ANE_HWX_SUBTYPE_H16;
	f->soc.doorbell = 0x1884000;
	f->soc.irq_status = 0x188c000;
	f->soc.irq_ack = 0x1890000;
	f->ipc = (struct ane_fw_surface) {
		.cpu = kunit_kzalloc(test, SZ_64K, GFP_KERNEL),
		.dva = FW_IPC_DVA, .size = SZ_64K,
	};
	f->pool = (struct ane_fw_surface) {
		.cpu = kunit_kzalloc(test, SZ_256K, GFP_KERNEL),
		.dva = FW_POOL_DVA, .size = SZ_256K,
	};
	if (!f->ipc.cpu || !f->pool.cpu)
		return NULL;
	for (i = 0; i < ANE_FW_CHANNELS; i++) {
		u8 *p = (u8 *)f->ipc.cpu + i * ANE_FW_DESCRIPTOR_SIZE;

		strscpy((char *)p, names[i], 64);
		put_unaligned_le32(i == 0 ? 2 : (i >= 4 && i <= 6), p + 0x40);
		put_unaligned_le32(i, p + 0x44);
		put_unaligned_le64(FW_RING_SLOTS, p + 0x48);
		put_unaligned_le64(f->ipc.dva + 0x800 + i * 0x200, p + 0x50);
	}
	f->next_dva = FW_MAP_DVA;
	f->program = 0x11;
	f->process = 0;
	if (ane_fw_transport_init(&f->t, &f->soc, &fw_ops, f, f->ipc, f->pool, SZ_16K,
				  f->ipc.dva, ANE_FW_TABLE_SIZE) ||
	    ane_fw_transport_arm(&f->t) ||
	    ane_h16_session_init(&f->s, &f->t, &fw_session_ops, f, 1000))
		return NULL;
	/* The fake clock moves 10 ns per poll; keep the post-UNLOAD settle short. */
	f->s.settle_ns = 100;
	return f;
}

static struct prog_fw *fw_create(struct kunit *test)
{
	return fw_create_family(test, 16);
}

static void snapshot_free(void *snapshot)
{
	ane_h16_hwx_snapshot_destroy(snapshot);
}

static struct ane_h16_hwx_snapshot *fw_snapshot(struct kunit *test, bool mixed, bool kernel)
{
	struct hwx_fixture *x = fixture_create(test, mixed, kernel);
	struct ane_h16_hwx_snapshot *snapshot;

	if (!x)
		return NULL;
	snapshot = kunit_kzalloc(test, sizeof(*snapshot), GFP_KERNEL);
	if (!snapshot || ane_h16_hwx_snapshot_create(x->bytes, x->size, snapshot) ||
	    kunit_add_action_or_reset(test, snapshot_free, snapshot))
		return NULL;
	return snapshot;
}

/*
 * Generic section with ID base + k for surface k, then base + count for the
 * scratch, as a host builder writes it.
 */
static size_t generic_fill(u8 *g, const struct ane_h16_hwx *hwx, u32 base)
{
	unsigned int i, entries = hwx->surface_count + !!hwx->scratch.size;

	memset(g, 0, ANE_H16_GENERIC_HEADER + entries * ANE_H16_GENERIC_ENTRY);
	put_unaligned_le32(ANE_H16_GENERIC_MAGIC, g);
	put_unaligned_le32(ANE_H16_GENERIC_VERSION, g + 4);
	put_unaligned_le32(entries, g + ANE_H16_GENERIC_COUNT);
	for (i = 0; i < entries; i++) {
		u8 *e = g + ANE_H16_GENERIC_HEADER + i * ANE_H16_GENERIC_ENTRY;
		bool scratch = i == hwx->surface_count;
		u32 direction = scratch ? 1 : hwx->surfaces[i].direction;

		put_unaligned_le32(1, e);
		put_unaligned_le32(base + i, e + 4);
		put_unaligned_le32(direction - 1, e + 8);
		put_unaligned_le32(direction, e + 0x10);
		put_unaligned_le64(scratch ? hwx->scratch.size : hwx->surfaces[i].span, e + 0x20);
		put_unaligned_le32(0xffff, e + 0x28);
	}
	return ANE_H16_GENERIC_HEADER + entries * ANE_H16_GENERIC_ENTRY;
}

/*
 * Operation section with one record that binds every program-descriptor
 * slot whose base is a surface, the scratch or the kernel, with the IDs that
 * generic_fill() gives.
 */
static size_t operation_fill(u8 *o, const struct ane_h16_hwx_snapshot *snapshot,
			     const struct ane_h16_hwx *hwx, u32 base)
{
	const u8 *c = (const u8 *)snapshot->bytes + hwx->program_offset;
	unsigned int slot, i, refs = 0;

	memset(o, 0, 4 + ANE_H16_OPERATION_RECORD);
	put_unaligned_le32(1, o);
	for (slot = 0; slot < 64; slot++) {
		u64 vm = get_unaligned_le64(c + 0x10 * (slot + 1));
		u32 tag = 0;

		if (!vm)
			continue;
		if (hwx->kernel.size && vm == hwx->kernel.vm)
			tag = ANE_H16_RESOURCE_KERNEL;
		if (hwx->scratch.size && vm == hwx->scratch.vm)
			tag = base + hwx->surface_count;
		for (i = 0; i < hwx->surface_count; i++)
			if (vm == hwx->surfaces[i].section.vm)
				tag = base + i;
		if (!tag)
			continue;
		put_unaligned_le32(slot, o + 4 + 0xc + refs * 8);
		put_unaligned_le32(tag, o + 4 + 0x10 + refs * 8);
		refs++;
	}
	put_unaligned_le32(refs, o + 4 + 8);
	return 4 + ANE_H16_OPERATION_RECORD;
}

static u8 test_aux[ANE_H16_AUX_COUNT][0x410];
static const void *const test_aux_ptr[ANE_H16_AUX_COUNT] = {
	test_aux[0], test_aux[1], test_aux[2], test_aux[3],
};

static size_t test_aux_size[ANE_H16_AUX_COUNT] = { 0x298, 0x410, 0x38, 0x28 };

#define TEST_GENERIC_BASE 8

static int fw_load_snapshot(struct prog_fw *f, struct ane_h16_hwx_snapshot *snapshot,
			    struct ane_h16_program **p)
{
	struct ane_h16_hwx hwx;

	if (!snapshot || ane_h16_hwx_validate(snapshot->bytes, snapshot->size, &hwx))
		return -ENOMEM;
	test_aux_size[ANE_H16_AUX_GENERIC] =
		generic_fill(test_aux[ANE_H16_AUX_GENERIC], &hwx, TEST_GENERIC_BASE);
	test_aux_size[ANE_H16_AUX_OPERATION] =
		operation_fill(test_aux[ANE_H16_AUX_OPERATION], snapshot, &hwx,
			       TEST_GENERIC_BASE);
	return ane_h16_session_load(&f->s, snapshot, test_aux_ptr, test_aux_size, p);
}

static int fw_load_program(struct prog_fw *f, struct ane_h16_program **p)
{
	return fw_load_snapshot(f, fw_snapshot(f->test, false, false), p);
}

static void expect_slot(struct kunit *test, const u8 *load, unsigned int slot, u32 resource,
			u32 class, u64 dva, u64 size)
{
	const u8 *d = load + ANE_H16_LOAD_SLOT_OFFSET + slot * ANE_H16_LOAD_SLOT_SIZE;

	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d), 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 4), resource);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 8), class);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0xc), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x10), 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x14), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x18), dva);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x20), size);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x28), 0);
}

static void expect_empty(struct kunit *test, const u8 *load, unsigned int slot)
{
	const u8 *d = load + ANE_H16_LOAD_SLOT_OFFSET + slot * ANE_H16_LOAD_SLOT_SIZE;

	KUNIT_EXPECT_FALSE(test, memchr_inv(d, 0, ANE_H16_LOAD_SLOT_SIZE));
}

/* add-128 shape: input, output, input surfaces of 32 KiB, no kernel section. */
static void program_add128_slots_test(struct kunit *test)
{
	static const u32 direction[3] = { 1, 2, 1 };
	struct ane_h16_process_binding bindings[ANE_H16_SURFACES], scratch;
	struct ane_h16_load_slot slots[ANE_H16_LOAD_SLOTS];
	struct ane_h16_load_layout layout = {};
	struct ane_h16_load_plan *plan = kunit_kzalloc(test, sizeof(*plan), GFP_KERNEL);
	struct ane_h16_hwx *hwx = kunit_kzalloc(test, sizeof(*hwx), GFP_KERNEL);
	u8 load[ANE_H16_LOAD_SIZE];
	u8 *generic = kunit_kzalloc(test, ANE_H16_GENERIC_HEADER + 3 * ANE_H16_GENERIC_ENTRY,
				    GFP_KERNEL);
	size_t generic_size;
	unsigned int i, count;

	KUNIT_ASSERT_NOT_NULL(test, plan);
	KUNIT_ASSERT_NOT_NULL(test, hwx);
	KUNIT_ASSERT_NOT_NULL(test, generic);

	for (i = 0; i < 3; i++) {
		hwx->surfaces[i].section.vm = 0x30000000 + i * SZ_32K;
		hwx->surfaces[i].direction = direction[i];
		hwx->surfaces[i].span = SZ_32K;
		hwx->surfaces[i].allocation = SZ_32K;
		plan->regions[i] = (struct ane_h16_hwx_region) {
			.vm = hwx->surfaces[i].section.vm, .size = SZ_32K,
			.permissions = direction[i], .kind = ANE_H16_REGION_SURFACE,
		};
		plan->allocation[i] = SZ_32K;
	}
	hwx->surface_count = 3;
	hwx->text = (struct ane_h16_hwx_section) { .vm = 0x30018000, .size = 0x140 };
	hwx->constants = (struct ane_h16_hwx_section) { .vm = 0x30018140, .size = 0x4000 };
	plan->regions[3] = (struct ane_h16_hwx_region) {
		.vm = 0x30018000, .size = 0x4140, .file_size = 0x4140,
		.permissions = 5, .kind = ANE_H16_REGION_TEXT,
	};
	plan->allocation[3] = SZ_32K;
	plan->count = 4;
	for (i = 0; i < plan->count; i++)
		layout.regions[i] = (struct ane_h16_load_mapping) {
			.dva = 0x10002000000ULL + i * SZ_1M, .size = plan->allocation[i],
			.permissions = plan->regions[i].permissions,
		};
	layout.region_count = plan->count;
	for (i = 0; i < ANE_H16_AUX_COUNT; i++) {
		layout.aux_dva[i] = 0x10003000000ULL + i * SZ_16K;
		layout.aux_size[i] = test_aux_size[i];
	}

	/* IDs from the generic section: x = 5, y = 4, z = 6 as the add-128 builder writes. */
	generic_size = generic_fill(generic, hwx, 0);
	put_unaligned_le32(5, generic + ANE_H16_GENERIC_HEADER + 4);
	put_unaligned_le32(4, generic + ANE_H16_GENERIC_HEADER + ANE_H16_GENERIC_ENTRY + 4);
	put_unaligned_le32(6, generic + ANE_H16_GENERIC_HEADER + 2 * ANE_H16_GENERIC_ENTRY + 4);
	KUNIT_ASSERT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size, bindings,
						       &count, NULL), 0);
	KUNIT_EXPECT_EQ(test, count, 3);
	KUNIT_EXPECT_EQ(test, bindings[0].index, 5);
	KUNIT_EXPECT_EQ(test, bindings[0].class, 0);
	KUNIT_EXPECT_EQ(test, bindings[1].index, 4);
	KUNIT_EXPECT_EQ(test, bindings[1].class, 1);
	KUNIT_EXPECT_EQ(test, bindings[2].index, 6);
	KUNIT_EXPECT_EQ(test, bindings[2].class, 0);
	/* No scratch segment, no scratch binding. */
	KUNIT_EXPECT_EQ(test, scratch.index, 0);

	/* A generic section that disagrees with the program refuses. */
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size - 4,
						       bindings, &count, NULL), -ENOEXEC);
	put_unaligned_le32(2, generic + ANE_H16_GENERIC_HEADER + 4);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size, bindings,
						       &count, NULL), -ENOEXEC);
	put_unaligned_le32(4, generic + ANE_H16_GENERIC_HEADER + 4);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size, bindings,
						       &count, NULL), -ENOEXEC);
	put_unaligned_le32(5, generic + ANE_H16_GENERIC_HEADER + 4);
	put_unaligned_le32(1, generic + ANE_H16_GENERIC_HEADER + 8);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size, bindings,
						       &count, NULL), -ENOEXEC);
	put_unaligned_le32(0, generic + ANE_H16_GENERIC_HEADER + 8);
	put_unaligned_le64(SZ_32K - 2, generic + ANE_H16_GENERIC_HEADER + 0x20);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size, bindings,
						       &count, NULL), -ENOEXEC);
	put_unaligned_le64(SZ_32K, generic + ANE_H16_GENERIC_HEADER + 0x20);
	put_unaligned_le32(2, generic + ANE_H16_GENERIC_COUNT);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size, bindings,
						       &count, NULL), -ENOEXEC);
	put_unaligned_le32(3, generic + ANE_H16_GENERIC_COUNT);
	put_unaligned_le32(0x11, generic + 4);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size, bindings,
						       &count, NULL), -ENOEXEC);
	put_unaligned_le32(ANE_H16_GENERIC_VERSION, generic + 4);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size, bindings,
						       &count, NULL), 0);

	KUNIT_ASSERT_EQ(test, ane_h16_load_slots(hwx, plan, &layout, slots), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_load_encode(load, sizeof(load), slots), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(load), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(load + 4), ANE_H16_LOAD_PROGRAM);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(load + 6), 0);
	expect_slot(test, load, 0, 1002, 4, layout.aux_dva[0], layout.aux_size[0]);
	/* No kernel section: slot 1 carries the constants at their sub-page DVA. */
	expect_slot(test, load, 1, 1007, 3, layout.regions[3].dva + 0x140, 0x4000);
	expect_slot(test, load, 2, 1006, 4, layout.regions[3].dva, 0x140);
	expect_slot(test, load, 3, 1003, 4, layout.aux_dva[1], test_aux_size[1]);
	expect_slot(test, load, 4, 1004, 4, layout.aux_dva[2], 0x38);
	expect_empty(test, load, 5);
	expect_slot(test, load, 6, 1005, 4, layout.aux_dva[3], 0x28);
	expect_empty(test, load, 7);
	expect_empty(test, load, 8);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(load + 0x1b8), 0xffffffff);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(load + 0x1bc), 0);

	/* Missing aux, short aux, mismatched layout and wrong length refuse. */
	layout.aux_dva[2] = 0;
	KUNIT_EXPECT_EQ(test, ane_h16_load_slots(hwx, plan, &layout, slots), -EINVAL);
	layout.aux_dva[2] = 0x10003008000ULL;
	layout.aux_size[3] = ANE_H16_AUX_MIN - 4;
	KUNIT_EXPECT_EQ(test, ane_h16_load_slots(hwx, plan, &layout, slots), -EINVAL);
	layout.aux_size[3] = 0x28;
	layout.regions[1].size = SZ_16K;
	KUNIT_EXPECT_EQ(test, ane_h16_load_slots(hwx, plan, &layout, slots), -EINVAL);
	layout.regions[1].size = SZ_32K;
	layout.region_count = 3;
	KUNIT_EXPECT_EQ(test, ane_h16_load_slots(hwx, plan, &layout, slots), -EINVAL);
	KUNIT_EXPECT_EQ(test, ane_h16_load_encode(load, sizeof(load) - 8, slots), -EINVAL);

	/* A program needs an input and exactly one output. */
	hwx->surfaces[0].direction = 2;
	generic_size = generic_fill(generic, hwx, 4);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size, bindings,
						       &count, NULL), -ENOEXEC);
	hwx->surfaces[0].direction = 3;
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, generic, generic_size, bindings,
						       &count, NULL), -ENOEXEC);
}

static u8 *sparse_bytes(struct kunit *test, const struct ane_h16_sparse_run *runs,
			unsigned int count, size_t size)
{
	u8 *bytes = kunit_kzalloc(test, size, GFP_KERNEL);
	unsigned int i;

	if (!bytes)
		return NULL;
	for (i = 0; i < count; i++) {
		if (runs[i].offset + runs[i].length > size)
			return NULL;
		memcpy(bytes + runs[i].offset, runs[i].bytes, runs[i].length);
	}
	return bytes;
}

/*
 * The pinned add-128 program and the sections h16-sections.py builds for it
 * load and run against the fake firmware. Segments are x (in), y (out), z (in)
 * while the tensors list x, z, y; EXECUTE names x = 5, y = 4, z = 6.
 */
static void program_add128_hwx_test(struct kunit *test)
{
	static const u32 ids[3] = { 5, 4, 6 }, classes[3] = { 0, 1, 0 };
	struct ane_h16_hwx_snapshot *snapshot;
	struct ane_h16_program *p = NULL;
	const void *aux[ANE_H16_AUX_COUNT];
	size_t aux_size[ANE_H16_AUX_COUNT] = {
		ADD128_GENERIC_SIZE, ADD128_OPERATION_SIZE,
		ADD128_PROCEDURE_SIZE, ADD128_PROPERTIES_SIZE,
	};
	struct prog_fw *f = fw_create(test);
	u8 *hwx;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	hwx = sparse_bytes(test, add128_hwx_runs, ARRAY_SIZE(add128_hwx_runs), ADD128_HWX_SIZE);
	aux[0] = sparse_bytes(test, add128_generic_runs, ARRAY_SIZE(add128_generic_runs),
			      ADD128_GENERIC_SIZE);
	aux[1] = sparse_bytes(test, add128_operation_runs, ARRAY_SIZE(add128_operation_runs),
			      ADD128_OPERATION_SIZE);
	aux[2] = sparse_bytes(test, add128_procedure_runs, ARRAY_SIZE(add128_procedure_runs),
			      ADD128_PROCEDURE_SIZE);
	aux[3] = sparse_bytes(test, add128_properties_runs, ARRAY_SIZE(add128_properties_runs),
			      ADD128_PROPERTIES_SIZE);
	KUNIT_ASSERT_NOT_NULL(test, hwx);
	for (i = 0; i < ANE_H16_AUX_COUNT; i++)
		KUNIT_ASSERT_NOT_NULL(test, aux[i]);
	snapshot = kunit_kzalloc(test, sizeof(*snapshot), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	KUNIT_ASSERT_EQ(test, ane_h16_hwx_snapshot_create(hwx, ADD128_HWX_SIZE, snapshot), 0);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, snapshot_free, snapshot), 0);
	KUNIT_ASSERT_MEMEQ(test, snapshot->digest, add128_hwx_sha256, SHA256_DIGEST_SIZE);

	KUNIT_ASSERT_EQ(test, ane_h16_session_load(&f->s, snapshot, aux, aux_size, &p), 0);
	KUNIT_ASSERT_NOT_NULL(test, p);
	KUNIT_EXPECT_EQ(test, p->hwx.surface_count, 3);
	KUNIT_EXPECT_EQ(test, p->hwx.surfaces[1].direction, 2);
	KUNIT_EXPECT_EQ(test, p->resource_count, 3);
	expect_slot(test, f->load, 0, 1002, 4, p->aux[0].dva, ADD128_GENERIC_SIZE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->load + ANE_H16_LOAD_SLOT_OFFSET +
						 ANE_H16_LOAD_SLOT_SIZE + 8), 3);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(f->load + ANE_H16_LOAD_SLOT_OFFSET +
						 ANE_H16_LOAD_SLOT_SIZE + 0x20), 0x4000);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(f->load + ANE_H16_LOAD_SLOT_OFFSET +
						 2 * ANE_H16_LOAD_SLOT_SIZE + 0x20), 0x130);
	expect_slot(test, f->load, 3, 1003, 4, p->aux[1].dva, ADD128_OPERATION_SIZE);
	expect_slot(test, f->load, 4, 1004, 4, p->aux[2].dva, ADD128_PROCEDURE_SIZE);
	expect_empty(test, f->load, 5);
	expect_slot(test, f->load, 6, 1005, 4, p->aux[3].dva, ADD128_PROPERTIES_SIZE);
	expect_empty(test, f->load, 7);
	expect_empty(test, f->load, 8);
	/* Only the output is device-writable. */
	KUNIT_EXPECT_EQ(test, f->writable_maps, 1);

	for (i = 0; i < 2; i++)
		KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x28), 3);
	for (i = 0; i < 3; i++) {
		const u8 *d = f->exec + 0x60 + i * 0x30;

		KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 4), ids[i]);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 8), classes[i]);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 0x10), 3);
		KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x18), p->surfaces[i]->dva);
		KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x20), SZ_32K);
	}
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
}

/* Synthetic HWX with a kernel section: slot 1 is class 3 with index 1007. */
static void program_kernel_slots_test(struct kunit *test)
{
	struct ane_h16_hwx_snapshot *snapshot = fw_snapshot(test, false, true);
	struct ane_h16_load_slot slots[ANE_H16_LOAD_SLOTS];
	struct ane_h16_load_layout layout = {};
	struct ane_h16_load_plan *plan = kunit_kzalloc(test, sizeof(*plan), GFP_KERNEL);
	struct ane_h16_hwx *hwx = kunit_kzalloc(test, sizeof(*hwx), GFP_KERNEL);
	unsigned int i, kernel = ANE_H16_REGIONS, text = ANE_H16_REGIONS;

	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	KUNIT_ASSERT_NOT_NULL(test, plan);
	KUNIT_ASSERT_NOT_NULL(test, hwx);
	KUNIT_ASSERT_EQ(test, ane_h16_load_plan_create(snapshot, plan), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_hwx_validate(snapshot->bytes, snapshot->size, hwx), 0);
	for (i = 0; i < plan->count; i++) {
		layout.regions[i] = (struct ane_h16_load_mapping) {
			.dva = 0x10002000000ULL + i * SZ_1M, .size = plan->allocation[i],
			.permissions = plan->regions[i].permissions,
		};
		if (plan->regions[i].kind == ANE_H16_REGION_KERNEL)
			kernel = i;
		if (plan->regions[i].kind == ANE_H16_REGION_TEXT)
			text = i;
	}
	KUNIT_ASSERT_LT(test, kernel, ANE_H16_REGIONS);
	KUNIT_ASSERT_LT(test, text, ANE_H16_REGIONS);
	layout.region_count = plan->count;
	for (i = 0; i < ANE_H16_AUX_COUNT; i++) {
		layout.aux_dva[i] = 0x10003000000ULL + i * SZ_16K;
		layout.aux_size[i] = test_aux_size[i];
	}
	KUNIT_ASSERT_EQ(test, ane_h16_load_slots(hwx, plan, &layout, slots), 0);
	KUNIT_EXPECT_TRUE(test, slots[1].present);
	KUNIT_EXPECT_EQ(test, slots[1].resource, 1007);
	KUNIT_EXPECT_EQ(test, slots[1].class, 3);
	KUNIT_EXPECT_EQ(test, slots[1].dva, layout.regions[kernel].dva);
	/* The kernel slot spans the section's whole 16 KiB pages. */
	KUNIT_EXPECT_EQ(test, slots[1].size,
			min_t(u64, ALIGN(hwx->kernel.size, SZ_16K), plan->allocation[kernel]));
	KUNIT_EXPECT_EQ(test, hwx->kernel.vm, plan->regions[kernel].vm);
	KUNIT_EXPECT_EQ(test, slots[2].resource, slots[1].resource - 1);
	KUNIT_EXPECT_EQ(test, slots[2].dva,
			layout.regions[text].dva + (hwx->text.vm - plan->regions[text].vm));
	KUNIT_EXPECT_EQ(test, slots[2].size, hwx->text.size);
	for (i = 0; i < ANE_H16_LOAD_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, slots[i].present, i != 5 && i < 7);
}

static void program_replies_test(struct kunit *test)
{
	struct ane_h16_load_slot slots[ANE_H16_LOAD_SLOTS] = {};
	u8 request[ANE_H16_CREATE_SIZE], reply[ANE_H16_CREATE_SIZE];
	u32 id;

	slots[0] = (struct ane_h16_load_slot) {
		.present = true, .resource = 1002, .class = 4,
		.dva = 0x10002000000ULL, .size = 0x298,
	};
	KUNIT_ASSERT_EQ(test, ane_h16_load_encode(request, ANE_H16_LOAD_SIZE, slots), 0);
	memcpy(reply, request, ANE_H16_LOAD_SIZE);
	/* ID 0 is valid. */
	put_unaligned_le32(0, reply + 0x1b8);
	KUNIT_EXPECT_EQ(test, ane_h16_load_reply(reply, ANE_H16_LOAD_SIZE, request, &id), 0);
	KUNIT_EXPECT_EQ(test, id, 0);
	/* Request ID left unchanged. */
	put_unaligned_le32(0xffffffff, reply + 0x1b8);
	KUNIT_EXPECT_EQ(test, ane_h16_load_reply(reply, ANE_H16_LOAD_SIZE, request, &id), -EPROTO);
	put_unaligned_le32(0x61, reply + 0x1b8);
	/* Nonzero acknowledgment, wrong length, wrong command, altered slot echo. */
	put_unaligned_le16(1, reply + 6);
	KUNIT_EXPECT_EQ(test, ane_h16_load_reply(reply, ANE_H16_LOAD_SIZE, request, &id),
			-EREMOTEIO);
	put_unaligned_le16(0, reply + 6);
	KUNIT_EXPECT_EQ(test, ane_h16_load_reply(reply, ANE_H16_LOAD_SIZE - 8, request, &id),
			-EPROTO);
	put_unaligned_le16(ANE_H16_CREATE_PROCESS, reply + 4);
	KUNIT_EXPECT_EQ(test, ane_h16_load_reply(reply, ANE_H16_LOAD_SIZE, request, &id), -EPROTO);
	put_unaligned_le16(ANE_H16_LOAD_PROGRAM, reply + 4);
	reply[0x20] ^= 1;
	KUNIT_EXPECT_EQ(test, ane_h16_load_reply(reply, ANE_H16_LOAD_SIZE, request, &id), -EPROTO);
	reply[0x20] ^= 1;
	KUNIT_EXPECT_EQ(test, ane_h16_load_reply(reply, ANE_H16_LOAD_SIZE, request, &id), 0);
	KUNIT_EXPECT_EQ(test, id, 0x61);

	/* CREATE: zero body, process ID 0 valid, any nonzero body refused. */
	KUNIT_ASSERT_EQ(test, ane_h16_create_encode(request, ANE_H16_CREATE_SIZE, 0x61), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(request + 4), ANE_H16_CREATE_PROCESS);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 8), 0x61);
	KUNIT_EXPECT_FALSE(test, memchr_inv(request + 0xc, 0, ANE_H16_CREATE_SIZE - 0xc));
	memcpy(reply, request, ANE_H16_CREATE_SIZE);
	KUNIT_EXPECT_EQ(test, ane_h16_create_reply(reply, ANE_H16_CREATE_SIZE, 0x61, &id), 0);
	KUNIT_EXPECT_EQ(test, id, 0);
	reply[0x257] = 1;
	KUNIT_EXPECT_EQ(test, ane_h16_create_reply(reply, ANE_H16_CREATE_SIZE, 0x61, &id),
			-EPROTO);
	reply[0x257] = 0;
	KUNIT_EXPECT_EQ(test, ane_h16_create_reply(reply, ANE_H16_CREATE_SIZE, 0x60, &id),
			-EPROTO);
	KUNIT_EXPECT_EQ(test, ane_h16_create_reply(reply, 0x10, 0x61, &id), -EPROTO);
	put_unaligned_le16(2, reply + 6);
	KUNIT_EXPECT_EQ(test, ane_h16_create_reply(reply, ANE_H16_CREATE_SIZE, 0x61, &id),
			-EREMOTEIO);
	KUNIT_EXPECT_EQ(test, ane_h16_create_encode(request, 0x10, 0x61), -EINVAL);

	/* TERMINATE 0x10 and UNLOAD 0xc carry IDs that the reply must echo. */
	KUNIT_ASSERT_EQ(test, ane_h16_terminate_encode(request, 0x10, 0x61, 0), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(request + 4), ANE_H16_TERMINATE_PROCESS);
	memcpy(reply, request, 0x10);
	KUNIT_EXPECT_EQ(test, ane_h16_id_reply(reply, 0x10, request), 0);
	put_unaligned_le16(1, reply + 6);
	KUNIT_EXPECT_EQ(test, ane_h16_id_reply(reply, 0x10, request), -EREMOTEIO);
	put_unaligned_le16(0, reply + 6);
	put_unaligned_le16(ANE_H16_UNLOAD_PROGRAM, reply + 4);
	KUNIT_EXPECT_EQ(test, ane_h16_id_reply(reply, 0x10, request), -EPROTO);
	KUNIT_ASSERT_EQ(test, ane_h16_unload_encode(request, 0xc, 0x61), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(request + 4), ANE_H16_UNLOAD_PROGRAM);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(request + 8), 0x61);
	KUNIT_EXPECT_EQ(test, ane_h16_unload_encode(request, 0x10, 0x61), -EINVAL);
	KUNIT_EXPECT_EQ(test, ane_h16_gate_encode(request, 0xc, 2), -EINVAL);
}

static void program_deny_list_test(struct kunit *test)
{
	static const u16 denied[] = { 0x27, 0x28, 0x30, 0x31, 0x34 };
	static const u16 allowed[] = { 0x0, 0x3, 0x4, 0x22, 0x2d, 0x2f, 0x200, 0x20c };
	struct prog_fw *f = fw_create(test);
	u8 command[16] = {};
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	for (i = 0; i < ARRAY_SIZE(denied); i++) {
		KUNIT_EXPECT_TRUE(test, ane_fw_command_denied(denied[i]));
		put_unaligned_le16(denied[i], command + 4);
		KUNIT_EXPECT_EQ(test, ane_fw_transport_publish(&f->t, ANE_FW_IO, command,
							       sizeof(command), 100), -EPERM);
		KUNIT_EXPECT_EQ(test, ane_fw_transport_publish(&f->t, ANE_FW_DATA_CHAIN,
							       command, sizeof(command), 100),
				-EPERM);
	}
	for (i = 0; i < ARRAY_SIZE(allowed); i++)
		KUNIT_EXPECT_FALSE(test, ane_fw_command_denied(allowed[i]));
	/* Nothing reached a ring and the transport stays usable. */
	KUNIT_EXPECT_EQ(test, f->count, 0);
	KUNIT_EXPECT_EQ(test, f->t.error, 0);
	KUNIT_EXPECT_EQ(test, ane_h16_stage_parse("program"), ANE_H16_PROGRAM);
}

static void program_custom_bar_test(struct kunit *test)
{
	struct ane_h16_hwx_snapshot *linear = fw_snapshot(test, false, false);
	struct ane_h16_hwx_snapshot *mixed = fw_snapshot(test, true, false);
	struct ane_h16_custom_bar bars[ANE_H16_MAX_CUSTOM_BARS];
	unsigned int count = 9;

	KUNIT_ASSERT_NOT_NULL(test, linear);
	KUNIT_ASSERT_NOT_NULL(test, mixed);
	KUNIT_ASSERT_EQ(test, ane_h16_custom_bars(linear, &linear->info, 36, bars, &count), 0);
	KUNIT_ASSERT_EQ(test, count, 1);
	KUNIT_EXPECT_EQ(test, bars[0].configuration, 1);
	KUNIT_EXPECT_EQ(test, bars[0].value_type, 0);
	KUNIT_EXPECT_EQ(test, bars[0].bar, 4);
	KUNIT_EXPECT_EQ(test, bars[0].bit_offset, 7);
	KUNIT_EXPECT_EQ(test, bars[0].bit_width, 8);
	KUNIT_EXPECT_EQ(test, bars[0].flags, 0);
	KUNIT_EXPECT_EQ(test, bars[0].value, 36);
	/* The DSID must fit the field. */
	KUNIT_EXPECT_EQ(test, ane_h16_custom_bars(linear, &linear->info, 256, bars, &count),
			-ENOEXEC);
	/* Scratch form: prefetch and intermediate-spill mcache records. */
	KUNIT_ASSERT_EQ(test, ane_h16_custom_bars(mixed, &mixed->info, 36, bars, &count), 0);
	KUNIT_ASSERT_EQ(test, count, 2);
	KUNIT_EXPECT_EQ(test, bars[0].configuration, 1);
	KUNIT_EXPECT_EQ(test, bars[0].value_type, 0);
	KUNIT_EXPECT_EQ(test, bars[0].bar, 4);
	KUNIT_EXPECT_EQ(test, bars[0].bit_offset, 2);
	KUNIT_EXPECT_EQ(test, bars[0].bit_width, 8);
	KUNIT_EXPECT_EQ(test, bars[0].flags, 0);
	KUNIT_EXPECT_EQ(test, bars[0].value, 36);
	KUNIT_EXPECT_EQ(test, bars[1].configuration, 1);
	KUNIT_EXPECT_EQ(test, bars[1].value_type, 1);
	KUNIT_EXPECT_EQ(test, bars[1].bar, 4);
	KUNIT_EXPECT_EQ(test, bars[1].bit_offset, 8);
	KUNIT_EXPECT_EQ(test, bars[1].bit_width, 8);
	KUNIT_EXPECT_EQ(test, bars[1].flags, 0);
	KUNIT_EXPECT_EQ(test, bars[1].value, 36);
	/* Usage type to descriptor class, round 5 section 7. */
	KUNIT_EXPECT_EQ(test, ane_h16_usage_class(1), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_usage_class(2), 1);
	KUNIT_EXPECT_EQ(test, ane_h16_usage_class(5), 6);
	KUNIT_EXPECT_EQ(test, ane_h16_usage_class(7), 2);
	KUNIT_EXPECT_EQ(test, ane_h16_usage_class(10), 4);
	KUNIT_EXPECT_EQ(test, ane_h16_usage_class(13), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_usage_class(0), 8);
	KUNIT_EXPECT_EQ(test, ane_h16_usage_class(99), 8);
}

static void program_session_test(struct kunit *test)
{
	static const u16 expected[] = {
		0x200, 0x202, 0x2d, 0x20c, 0x2d, 0x2d, 0x20c, 0x2d, 0x203, 0x201,
	};
	static const u32 gates[] = { 0, 0, 0, 0, 1, 0, 0, 1 };
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL, *second = NULL;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_ASSERT_NOT_NULL(test, p);
	KUNIT_EXPECT_EQ(test, p->key.program, 0x11);
	KUNIT_EXPECT_EQ(test, p->key.process, 0);
	KUNIT_EXPECT_EQ(test, p->resource_count, 2);
	/* Firmware saw the request ID and the synthetic slots. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->load + 0x1b8), 0xffffffff);
	expect_slot(test, f->load, 0, 1002, 4, p->aux[0].dva,
		    test_aux_size[ANE_H16_AUX_GENERIC]);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->load + ANE_H16_LOAD_SLOT_OFFSET +
						 ANE_H16_LOAD_SLOT_SIZE + 4), 1007);
	expect_slot(test, f->load, 6, 1005, 4, p->aux[3].dva, 0x28);
	/* Only the output surface is device-writable. */
	KUNIT_EXPECT_EQ(test, f->writable_maps, 1);

	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	/* The first request after boot sets up EXELOOP state: queued. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x30), ANE_H16_EXECUTE_FLAG_QUEUED);
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	/* EXECUTE: IDs, priority 5, UUID 2, two descriptors and one mcache record. */
	KUNIT_EXPECT_EQ(test, f->exec_length, 0xcc0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 8), 0x11);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xc), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x18), 5);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(f->exec + 0x20), 2);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x28), 2);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x60 + 8), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x90 + 8), 1);
	/* Descriptor IDs are the generic-section IDs. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x60 + 4), TEST_GENERIC_BASE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x90 + 4), TEST_GENERIC_BASE + 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(f->exec + 0x60 + 0x18),
			p->surfaces[0]->dva);
	/* H16: each descriptor covers its whole HWX section. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(f->exec + 0x60 + 0x20),
			p->hwx.surfaces[0].section.size);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xca0), 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xca8 + 8), 4);
	/* Later requests of the same program take the direct path. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x30), 0);


	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	KUNIT_EXPECT_TRUE(test, p->closed);
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), -EINVAL);

	KUNIT_ASSERT_EQ(test, f->count, ARRAY_SIZE(expected));
	for (i = 0; i < ARRAY_SIZE(expected); i++)
		KUNIT_EXPECT_EQ(test, f->ids[i], expected[i]);
	for (i = 0; i < ARRAY_SIZE(gates); i++)
		if (f->ids[i] == 0x2d)
			KUNIT_EXPECT_EQ(test, f->values[i], gates[i]);

	/* After TERMINATE and UNLOAD a new program loads in the same session. */
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &second), 0);
	KUNIT_ASSERT_NOT_NULL(test, second);
	KUNIT_EXPECT_EQ(test, second->generation, 2);
	/* The firmware reuses the program ID; the session tells them apart. */
	KUNIT_EXPECT_EQ(test, second->key.program, p->key.program);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, second), 0);
	/* The engine last ran the first program: this request is queued. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x30), ANE_H16_EXECUTE_FLAG_QUEUED);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, second), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x30), 0);

	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, second), 0);
	/* Closing the first program's file late is harmless. */
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/*
 * Gating held off across back-to-back requests: one 0x002d 0 for a run of
 * EXECUTEs, 0x002d 1 once the hold expired, again before TERMINATE, and
 * before system sleep. Without a hold every EXECUTE gates itself.
 */
static void program_gate_hold_test(struct kunit *test)
{
	static const u16 expected[] = {
		0x200, 0x202, 0x2d, 0x20c, 0x20c, 0x20c, 0x2d,
		0x2d, 0x20c, 0x2d, 0x203, 0x201,
		0x200, 0x202, 0x2d, 0x20c, 0x2d,
	};
	static const u32 gates[] = { 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL, *q = NULL;
	/* Stands in for the accel node lock the sleep gate takes. */
	struct mutex lock;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->s.gate_hold_ns = 1000;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	for (i = 0; i < 3; i++)
		KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_TRUE(test, f->s.gate_held);
	/* Held requests report no gate phases. */
	KUNIT_EXPECT_EQ(test, f->s.timing.ns[ANE_H16_T_GATE_OPEN], 0);
	KUNIT_EXPECT_EQ(test, f->s.timing.ns[ANE_H16_T_GATE_CLOSE], 0);
	/* Too soon after the last request: still held, no traffic. */
	i = f->count;
	KUNIT_EXPECT_EQ(test, ane_h16_session_gate_expire(&f->s), -EAGAIN);
	KUNIT_EXPECT_EQ(test, f->count, i);
	f->now += 1000;
	KUNIT_EXPECT_EQ(test, ane_h16_session_gate_expire(&f->s), 0);
	KUNIT_EXPECT_FALSE(test, f->s.gate_held);
	KUNIT_EXPECT_EQ(test, ane_h16_session_gate_expire(&f->s), 0);
	/* The next request opens it again; close turns gating on first. */
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	KUNIT_EXPECT_FALSE(test, f->s.gate_held);
	/* System sleep turns gating on before the firmware is suspended. */
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &q), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, q), 0);
	mutex_init(&lock);
	f->s.lock = &lock;
	ane_h16_session_sleep(&f->s, true);
	KUNIT_EXPECT_FALSE(test, f->s.gate_held);
	ane_h16_session_sleep(&f->s, false);
	f->s.lock = NULL;
	KUNIT_ASSERT_EQ(test, f->count, ARRAY_SIZE(expected));
	for (i = 0; i < ARRAY_SIZE(expected); i++) {
		KUNIT_EXPECT_EQ(test, f->ids[i], expected[i]);
		if (f->ids[i] == 0x2d)
			KUNIT_EXPECT_EQ(test, f->values[i], gates[i]);
	}
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, q), 0);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/* A linked program: the HWX scratch is a kernel buffer sent with every EXECUTE. */
static void program_scratch_test(struct kunit *test)
{
	struct ane_h16_process_binding bindings[ANE_H16_SURFACES], scratch;
	struct ane_h16_hwx_snapshot *snapshot = fw_snapshot(test, true, false);
	struct ane_h16_hwx *hwx = kunit_kzalloc(test, sizeof(*hwx), GFP_KERNEL);
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	u8 *g = test_aux[ANE_H16_AUX_GENERIC], *e;
	unsigned int count;
	size_t size;
	const u8 *d;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, hwx);
	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	KUNIT_ASSERT_EQ(test, ane_h16_hwx_validate(snapshot->bytes, snapshot->size, hwx), 0);
	KUNIT_ASSERT_EQ(test, hwx->scratch.size, SZ_16K);
	size = generic_fill(g, hwx, TEST_GENERIC_BASE);
	e = g + ANE_H16_GENERIC_HEADER + 2 * ANE_H16_GENERIC_ENTRY;
	KUNIT_ASSERT_EQ(test, ane_h16_process_bindings(hwx, g, size, bindings, &count,
						       &scratch), 0);
	KUNIT_EXPECT_EQ(test, count, 2);
	KUNIT_EXPECT_EQ(test, scratch.index, TEST_GENERIC_BASE + 2);
	KUNIT_EXPECT_EQ(test, scratch.class, 0);
	KUNIT_EXPECT_EQ(test, scratch.hwx_vm, hwx->scratch.vm);
	/* No scratch entry, an output-like or short one, or a reused ID refuse. */
	put_unaligned_le32(2, g + ANE_H16_GENERIC_COUNT);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, g, size - ANE_H16_GENERIC_ENTRY,
						       bindings, &count, &scratch), -ENOEXEC);
	put_unaligned_le32(3, g + ANE_H16_GENERIC_COUNT);
	put_unaligned_le32(2, e + 0x10);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, g, size, bindings, &count,
						       &scratch), -ENOEXEC);
	put_unaligned_le32(1, e + 0x10);
	put_unaligned_le64(SZ_16K - 4, e + 0x20);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, g, size, bindings, &count,
						       &scratch), -ENOEXEC);
	put_unaligned_le64(SZ_16K, e + 0x20);
	put_unaligned_le32(TEST_GENERIC_BASE, e + 4);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, g, size, bindings, &count,
						       &scratch), -ENOEXEC);
	put_unaligned_le32(TEST_GENERIC_BASE + 2, e + 4);

	test_aux_size[ANE_H16_AUX_GENERIC] = size;
	KUNIT_ASSERT_EQ(test, ane_h16_session_load(&f->s, snapshot, test_aux_ptr, test_aux_size,
						   &p), 0);
	KUNIT_ASSERT_TRUE(test, p->has_scratch);
	/* Userspace still sees two surfaces. */
	KUNIT_EXPECT_EQ(test, p->resource_count, 2);
	/* The output and the scratch are the device-writable mappings. */
	KUNIT_EXPECT_EQ(test, f->writable_maps, 2);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + ANE_H16_EXECUTE_FLAGS),
			ANE_H16_EXECUTE_FLAG_QUEUED);
	/* The complete tuple lets the second execution use the direct path. */
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + ANE_H16_EXECUTE_FLAGS), 0);
	KUNIT_EXPECT_EQ(test, f->exec_length,
			ANE_H16_EXECUTE_BASE_SIZE + 2 * ANE_H16_CUSTOM_BAR_STRIDE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xca0), 2);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xca8), 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xcac), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xcb0), 4);
	KUNIT_EXPECT_EQ(test, f->exec[0xcb4], 2);
	KUNIT_EXPECT_EQ(test, f->exec[0xcb5], 8);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(f->exec + 0xcb8), 36);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xcc0), 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xcc4), 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xcc8), 4);
	KUNIT_EXPECT_EQ(test, f->exec[0xccc], 8);
	KUNIT_EXPECT_EQ(test, f->exec[0xccd], 8);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(f->exec + 0xcd0), 36);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x28), 3);
	d = f->exec + 0x60 + 2 * 0x30;
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d), 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 4), TEST_GENERIC_BASE + 2);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 8), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x18), p->scratch.mapping_dva);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x20), SZ_16K);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

static void program_bad_load_test(struct kunit *test)
{
	static const enum fw_mode modes[] = { FW_LOAD_ID_UNCHANGED, FW_LOAD_NAK, FW_CREATE_BODY };
	static const int errors[] = { -EPROTO, -EREMOTEIO, -EPROTO };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		struct prog_fw *f = fw_create(test);
		struct ane_h16_program *p = NULL;

		KUNIT_ASSERT_NOT_NULL(test, f);
		f->mode = modes[i];
		KUNIT_EXPECT_EQ(test, fw_load_program(f, &p), errors[i]);
		KUNIT_EXPECT_PTR_EQ(test, p, NULL);
		KUNIT_EXPECT_TRUE(test, f->s.quarantined);
		/* No further firmware traffic after quarantine. */
		KUNIT_EXPECT_EQ(test, f->count, modes[i] == FW_CREATE_BODY ? 2 : 1);
		KUNIT_EXPECT_EQ(test, fw_load_program(f, &p), -EIO);
	}
}

/*
 * A LOAD whose process descriptors do not validate is refused before any
 * mapping or firmware traffic, and it must not block the next LOAD.
 */
static void program_bad_operation_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_hwx_snapshot *snapshot;
	struct ane_h16_program *p = NULL;
	struct ane_h16_hwx hwx;

	KUNIT_ASSERT_NOT_NULL(test, f);
	snapshot = fw_snapshot(test, false, false);
	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	KUNIT_ASSERT_EQ(test, ane_h16_hwx_validate(snapshot->bytes, snapshot->size, &hwx), 0);
	test_aux_size[ANE_H16_AUX_GENERIC] =
		generic_fill(test_aux[ANE_H16_AUX_GENERIC], &hwx, TEST_GENERIC_BASE);
	test_aux_size[ANE_H16_AUX_OPERATION] =
		operation_fill(test_aux[ANE_H16_AUX_OPERATION], snapshot, &hwx,
			       TEST_GENERIC_BASE);
	/* One record more than the section carries. */
	put_unaligned_le32(get_unaligned_le32(test_aux[ANE_H16_AUX_OPERATION]) + 1,
			   test_aux[ANE_H16_AUX_OPERATION]);
	KUNIT_EXPECT_EQ(test, ane_h16_session_load(&f->s, snapshot, test_aux_ptr,
						   test_aux_size, &p), -ENOEXEC);
	KUNIT_EXPECT_PTR_EQ(test, p, NULL);
	KUNIT_EXPECT_PTR_EQ(test, f->s.program, NULL);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	KUNIT_EXPECT_EQ(test, f->count, 0);
	/* The refused LOAD left nothing behind: a valid one still works. */
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
}

static void program_bad_report_test(struct kunit *test)
{
	static const enum fw_mode modes[] = {
		FW_REPORT_FOREIGN_UUID, FW_REPORT_WRONG_PROCESS, FW_REPORT_FAILURE,
	};
	static const int errors[] = { -ETIMEDOUT, -EPROTO, -EIO };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(modes); i++) {
		struct prog_fw *f = fw_create(test);
		struct ane_h16_program *p = NULL;
		unsigned int sent;

		KUNIT_ASSERT_NOT_NULL(test, f);
		KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
		f->mode = modes[i];
		KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), errors[i]);
		/* No closing 0x002d after an uncertain or failed request. */
		KUNIT_EXPECT_EQ(test, f->ids[f->count - 1], ANE_H16_EXECUTE);
		KUNIT_EXPECT_TRUE(test, f->s.quarantined);
		KUNIT_EXPECT_TRUE(test, f->s.pending.quarantined);
		sent = f->count;
		KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), -EIO);
		/* Teardown is not attempted while quarantined; mappings stay. */
		KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), -EIO);
		KUNIT_EXPECT_EQ(test, f->count, sent);
	}
}

/* A request that starts and never ends names the wait step and the last state. */
static void program_stall_trace_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	f->mode = FW_REPORT_STALL;
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), -ETIMEDOUT);
	/* The fake firmware asked for no log buffer: nothing to print. */
	KUNIT_EXPECT_EQ(test, f->s.fwlog_lines, 0);
	KUNIT_EXPECT_TRUE(test, f->s.quarantined);
	KUNIT_EXPECT_EQ(test, f->fail_step, 3);
	KUNIT_EXPECT_EQ(test, f->fail_flags, ANE_H16_EXECUTE_FLAG_QUEUED);
	KUNIT_EXPECT_EQ(test, f->fail_reports, 2);
	KUNIT_EXPECT_EQ(test, f->fail_state, 3);
	KUNIT_EXPECT_EQ(test, f->fail_other, 0);

	/* Reports for another UUID only: none seen for this request. */
	f = fw_create(test);
	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	f->mode = FW_REPORT_FOREIGN_UUID;
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, f->fail_step, 3);
	KUNIT_EXPECT_EQ(test, f->fail_reports, 0);
	KUNIT_EXPECT_EQ(test, f->fail_state, U32_MAX);
	KUNIT_EXPECT_EQ(test, f->fail_other, 2);
}

/* The phases of one EXECUTE add up, and only successful ones are accounted. */
static void program_timing_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	const struct ane_h16_exec_sample *x;
	struct ane_h16_exec_stats st;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	f->doorbell_ns = 100;
	f->mode = FW_REPORT_LATE;
	f->late_polls = 7;
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	x = &f->s.timing;
	KUNIT_EXPECT_EQ(test, x->ns[ANE_H16_T_GATE_OPEN], 100ULL);
	KUNIT_EXPECT_EQ(test, x->ns[ANE_H16_T_SUBMIT_ACK], 100ULL);
	/* The state-0 report was there with the ack. */
	KUNIT_EXPECT_EQ(test, x->ns[ANE_H16_T_ACK_REPORT], 0ULL);
	/* Seven sleeps of 10 ns until the completion. */
	KUNIT_EXPECT_EQ(test, x->ns[ANE_H16_T_REPORT_DONE], 70ULL);
	KUNIT_EXPECT_EQ(test, x->ns[ANE_H16_T_GATE_CLOSE], 100ULL);
	KUNIT_EXPECT_EQ(test, x->ns[ANE_H16_T_FIRMWARE], 370ULL);
	KUNIT_EXPECT_EQ(test, x->polls, 7ULL);
	/* Nothing outside the firmware phases takes time in the fake. */
	KUNIT_EXPECT_EQ(test, x->ns[ANE_H16_T_PREPARE], 0ULL);
	KUNIT_EXPECT_EQ(test, x->ns[ANE_H16_T_FINISH], 0ULL);

	ane_h16_session_account(&f->s, 1, 5, 6, 400);
	f->mode = FW_OK;
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	/* All three reports came with the ack: nothing between them. */
	KUNIT_EXPECT_EQ(test, f->s.timing.ns[ANE_H16_T_REPORT_DONE], 0ULL);
	KUNIT_EXPECT_EQ(test, f->s.timing.polls, 0ULL);
	ane_h16_session_account(&f->s, 1, 15, 8, 340);
	ane_h16_session_stats(&f->s, &st, false);
	KUNIT_EXPECT_EQ(test, st.count, 2ULL);
	KUNIT_EXPECT_EQ(test, st.sum.ns[ANE_H16_T_LOCK], 2ULL);
	KUNIT_EXPECT_EQ(test, st.last.ns[ANE_H16_T_COPY_IN], 15ULL);
	KUNIT_EXPECT_EQ(test, st.sum.ns[ANE_H16_T_COPY_IN], 20ULL);
	KUNIT_EXPECT_EQ(test, st.min.ns[ANE_H16_T_REPORT_DONE], 0ULL);
	KUNIT_EXPECT_EQ(test, st.max.ns[ANE_H16_T_REPORT_DONE], 70ULL);
	KUNIT_EXPECT_EQ(test, st.min.ns[ANE_H16_T_IOCTL], 340ULL);
	KUNIT_EXPECT_EQ(test, st.max.ns[ANE_H16_T_IOCTL], 400ULL);
	KUNIT_EXPECT_EQ(test, st.sum.polls, 7ULL);
	KUNIT_EXPECT_EQ(test, st.min.polls, 0ULL);
	KUNIT_EXPECT_EQ(test, st.max.polls, 7ULL);

	/* A reset starts over: the next sample sets min and max alone. */
	ane_h16_session_stats(&f->s, NULL, true);
	ane_h16_session_account(&f->s, 1, 9, 9, 500);
	ane_h16_session_stats(&f->s, &st, false);
	KUNIT_EXPECT_EQ(test, st.count, 1ULL);
	KUNIT_EXPECT_EQ(test, st.min.ns[ANE_H16_T_IOCTL], 500ULL);
	KUNIT_EXPECT_EQ(test, st.sum.ns[ANE_H16_T_COPY_OUT], 9ULL);

	/* A stalled request times out and leaves the statistics alone. */
	f->mode = FW_REPORT_STALL;
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), -ETIMEDOUT);
	ane_h16_session_stats(&f->s, &st, false);
	KUNIT_EXPECT_EQ(test, st.count, 1ULL);
}

static void program_fwlog_line_test(struct kunit *test)
{
	static const char text[] = "R: [0:1]: CMD = 0x020c\tok\x01  ";
	u8 *r = kunit_kzalloc(test, ANE_H16_FWLOG_RECORD, GFP_KERNEL);
	char line[ANE_H16_FWLOG_RECORD];

	KUNIT_ASSERT_NOT_NULL(test, r);
	KUNIT_EXPECT_EQ(test, ane_h16_fwlog_line(r, line), 0);
	r[0] = 0x12;
	memcpy(r + 2, text, sizeof(text));
	KUNIT_EXPECT_EQ(test, ane_h16_fwlog_line(r, line), 26);
	KUNIT_EXPECT_STREQ(test, line, "R: [0:1]: CMD = 0x020c ok.");
	/* An unterminated record stops at its end. */
	memset(r + 2, 'a', ANE_H16_FWLOG_RECORD - 2);
	KUNIT_EXPECT_EQ(test, ane_h16_fwlog_line(r, line), ANE_H16_FWLOG_RECORD - 2);
	KUNIT_EXPECT_EQ(test, line[ANE_H16_FWLOG_RECORD - 2], 0);
}

/* The transport finds the log allocation by its tag. */
static void program_shared_tag_test(struct kunit *test)
{
	struct ane_fw_transport *t = kunit_kzalloc(test, sizeof(*t), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, t);
	t->shared_count = 2;
	t->shared_tags[0] = ANE_FW_SHARED_LOG;
	t->shared_tags[1] = ANE_FW_SHARED_EXPP;
	t->shared[0].dva = 0x10001300000ULL;
	t->shared[1].dva = 0x10001358000ULL;
	KUNIT_EXPECT_PTR_EQ(test, ane_fw_transport_shared_tagged(t, ANE_FW_SHARED_LOG),
			    &t->shared[0]);
	KUNIT_EXPECT_PTR_EQ(test, ane_fw_transport_shared_tagged(t, ANE_FW_SHARED_EXPP),
			    &t->shared[1]);
	t->shared_retired[0] = true;
	KUNIT_EXPECT_NULL(test, ane_fw_transport_shared_tagged(t, ANE_FW_SHARED_LOG));
	KUNIT_EXPECT_NULL(test, ane_fw_transport_shared_tagged(t, 0x43434d44));
}

/*
 * Each completed EXECUTE gives its arena back, so a long run never fills
 * the pool; LOAD and CREATE keep theirs.
 */
static void program_pool_recycle_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	size_t mark;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	mark = ane_fw_transport_mark(&f->t);
	KUNIT_EXPECT_GT(test, mark, 0);
	/* Far more requests than one pass through the 256 KiB arena holds. */
	for (i = 0; i < 1000; i++) {
		KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
		KUNIT_ASSERT_EQ(test, ane_fw_transport_mark(&f->t), mark);
	}
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	/* A failed request keeps its space: nothing is reused after a fault. */
	f->mode = FW_REPORT_FAILURE;
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), -EIO);
	KUNIT_EXPECT_GT(test, f->t.pool_used, mark);
	/* Release refuses while a command is in flight or below the arena use. */
	f->t.pending[ANE_FW_IO].active = true;
	KUNIT_EXPECT_EQ(test, ane_fw_transport_release(&f->t, mark), -EBUSY);
	f->t.pending[ANE_FW_IO].active = false;
	KUNIT_EXPECT_EQ(test, ane_fw_transport_release(&f->t, f->t.pool_used + 1), -EINVAL);
}

/*
 * A surface mapped into a process outlives its program's teardown: it goes
 * back to the spares only after the last mapping reference is dropped, so a
 * later program never gets pages a process still maps.
 */
static void program_export_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	struct ane_h16_program *p = NULL, *second = NULL;
	struct ane_h16_export *e = NULL, *again = NULL;
	struct ane_fw_surface exported;
	unsigned int spares, i;
	bool reused = false;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.lock = lock;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	/* Bad indices refuse; two mappings of one surface share its export. */
	KUNIT_EXPECT_EQ(test, ane_h16_session_export(&f->s, p, p->resource_count, &e), -EINVAL);
	KUNIT_ASSERT_EQ(test, ane_h16_session_export(&f->s, p, 1, &e), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_export(&f->s, p, 1, &again), 0);
	KUNIT_EXPECT_PTR_EQ(test, e, again);
	KUNIT_EXPECT_EQ(test, refcount_read(&e->ref), 3);
	KUNIT_EXPECT_TRUE(test, e->write);
	exported = e->surface;
	KUNIT_EXPECT_EQ(test, exported.dva, p->surfaces[1]->dva);
	ane_h16_export_put(again);

	spares = f->s.spare_count;
	KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	/* Every region and section went back except the mapped surface. */
	KUNIT_EXPECT_EQ(test, f->s.spare_count, spares + p->plan.count + ANE_H16_AUX_COUNT - 1);
	KUNIT_EXPECT_TRUE(test, e->released);
	for (i = 0; i < f->s.spare_count; i++)
		KUNIT_EXPECT_NE(test, f->s.spares[i].surface.dva, exported.dva);
	KUNIT_EXPECT_EQ(test, ane_h16_session_export(&f->s, p, 1, &again), -EINVAL);
	ane_h16_session_put(&f->s, p);

	/* A second program cannot get the mapped pages. */
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &second), 0);
	for (i = 0; i < second->plan.count; i++)
		KUNIT_EXPECT_NE(test, second->regions[i].dva, exported.dva);
	KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, second), 0);
	ane_h16_session_put(&f->s, second);

	/* The last unmap returns it. */
	spares = f->s.spare_count;
	ane_h16_export_put(e);
	ane_h16_session_flush_exports(&f->s);
	KUNIT_EXPECT_EQ(test, f->s.spare_count, spares + 1);
	for (i = 0; i < f->s.spare_count; i++)
		reused |= f->s.spares[i].surface.dva == exported.dva;
	KUNIT_EXPECT_TRUE(test, reused);
	f->s.lock = NULL;
}

/* Running out of arena refuses cleanly, before any traffic. */
static void program_pool_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	unsigned int sent;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	sent = f->count;
	f->t.pool_used = f->t.pool.size - 0x200;
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), -ENOSPC);
	KUNIT_EXPECT_EQ(test, f->count, sent);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, f->count, sent + 2);
}

static u8 *fw_sparse_large(struct kunit *test, const struct ane_h16_sparse_run *runs,
			   unsigned int count, size_t size)
{
	u8 *bytes = fw_zalloc(test, size);
	unsigned int i;

	if (!bytes)
		return NULL;
	for (i = 0; i < count; i++) {
		if (runs[i].offset + runs[i].length > size)
			return NULL;
		memcpy(bytes + runs[i].offset, runs[i].bytes, runs[i].length);
	}
	return bytes;
}

static struct ane_h16_hwx_snapshot *mobilenet_snapshot(struct kunit *test)
{
	struct ane_h16_hwx_snapshot *snapshot = kunit_kzalloc(test, sizeof(*snapshot),
							      GFP_KERNEL);
	u8 *hwx = fw_sparse_large(test, mobilenet_hwx_runs, ARRAY_SIZE(mobilenet_hwx_runs),
				  MOBILENET_HWX_SIZE);

	if (!snapshot || !hwx ||
	    ane_h16_hwx_snapshot_create(hwx, MOBILENET_HWX_SIZE, snapshot) ||
	    kunit_add_action_or_reset(test, snapshot_free, snapshot))
		return NULL;
	return snapshot;
}

static void mobilenet_sections(struct kunit *test, const void *aux[ANE_H16_AUX_COUNT],
			       size_t aux_size[ANE_H16_AUX_COUNT])
{
	aux_size[0] = MOBILENET_GENERIC_SIZE;
	aux_size[1] = MOBILENET_OPERATION_SIZE;
	aux_size[2] = MOBILENET_PROCEDURE_SIZE;
	aux_size[3] = MOBILENET_PROPERTIES_SIZE;
	aux[0] = sparse_bytes(test, mobilenet_generic_runs, ARRAY_SIZE(mobilenet_generic_runs),
			      aux_size[0]);
	aux[1] = sparse_bytes(test, mobilenet_operation_runs,
			      ARRAY_SIZE(mobilenet_operation_runs), aux_size[1]);
	aux[2] = sparse_bytes(test, mobilenet_procedure_runs,
			      ARRAY_SIZE(mobilenet_procedure_runs), aux_size[2]);
	aux[3] = sparse_bytes(test, mobilenet_properties_runs,
			      ARRAY_SIZE(mobilenet_properties_runs), aux_size[3]);
}

/*
 * The H17 MobileNetV1 HWX is one linked program: 28 tasks, input x and
 * output y as surfaces, the 3.5 MiB __DATA scratch on BAR slot 3 and the
 * kernel table on slot 8. The H16 rules hold except the subtype and the stream
 * prologue, which is 0 or 1.
 */
static void program_h17_hwx_test(struct kunit *test)
{
	struct ane_h16_hwx_snapshot *snapshot = mobilenet_snapshot(test);
	struct ane_h16_hwx *hwx = kunit_kzalloc(test, sizeof(*hwx), GFP_KERNEL);
	u8 *bytes;

	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	KUNIT_ASSERT_NOT_NULL(test, hwx);
	bytes = snapshot->bytes;
	*hwx = snapshot->info;
	KUNIT_EXPECT_EQ(test, hwx->subtype, ANE_HWX_SUBTYPE_H17);
	KUNIT_EXPECT_EQ(test, hwx->tasks, 28);
	KUNIT_EXPECT_EQ(test, hwx->surface_count, 2);
	KUNIT_EXPECT_EQ(test, hwx->surfaces[0].direction, 1);
	KUNIT_EXPECT_EQ(test, hwx->surfaces[0].span, 301056);
	KUNIT_EXPECT_EQ(test, hwx->surfaces[1].direction, 2);
	KUNIT_EXPECT_EQ(test, hwx->surfaces[1].span, 458752);
	KUNIT_EXPECT_EQ(test, hwx->scratch.vm, 0x30000000);
	KUNIT_EXPECT_EQ(test, hwx->scratch.size, 0x380000);
	KUNIT_EXPECT_EQ(test, hwx->kernel.size, 0x81cb40);
	KUNIT_EXPECT_EQ(test, hwx->text.size, 0x3110);

	/* Subtype 9 opens its stream with 0 or 1; a subtype 7 file must open with 1. */
	put_unaligned_le32(ANE_HWX_SUBTYPE_H16, bytes + 8);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(bytes, snapshot->size, hwx), -ENOEXEC);
	put_unaligned_le32(1, bytes + hwx->text.offset);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(bytes, snapshot->size, hwx), 0);
	KUNIT_EXPECT_EQ(test, hwx->subtype, ANE_HWX_SUBTYPE_H16);
	/* Apple's matmul and add programs for H17 open with 1; other words stay refused. */
	put_unaligned_le32(ANE_HWX_SUBTYPE_H17, bytes + 8);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(bytes, snapshot->size, hwx), 0);
	put_unaligned_le32(2, bytes + hwx->text.offset);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(bytes, snapshot->size, hwx), -ENOEXEC);
	put_unaligned_le32(0, bytes + hwx->text.offset);
	put_unaligned_le32(8, bytes + 8);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(bytes, snapshot->size, hwx), -ENOEXEC);
	put_unaligned_le32(ANE_HWX_SUBTYPE_H17, bytes + 8);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(bytes, snapshot->size, hwx), 0);
}

/* Generic and operation sections of the MobileNet program, and their faults. */
static void program_h17_sections_test(struct kunit *test)
{
	struct ane_h16_hwx_snapshot *snapshot = mobilenet_snapshot(test);
	struct ane_h16_process_binding bindings[ANE_H16_SURFACES], scratch = {};
	const struct ane_h16_hwx *hwx;
	const void *aux[ANE_H16_AUX_COUNT];
	size_t aux_size[ANE_H16_AUX_COUNT];
	unsigned int count;
	u8 *g, *o;

	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	hwx = &snapshot->info;
	mobilenet_sections(test, aux, aux_size);
	g = (u8 *)aux[ANE_H16_AUX_GENERIC];
	o = (u8 *)aux[ANE_H16_AUX_OPERATION];
	KUNIT_ASSERT_NOT_NULL(test, g);
	KUNIT_ASSERT_NOT_NULL(test, o);

	KUNIT_ASSERT_EQ(test, ane_h16_process_bindings(hwx, g, aux_size[0], bindings, &count,
						       &scratch), 0);
	KUNIT_EXPECT_EQ(test, count, 2);
	KUNIT_EXPECT_EQ(test, bindings[0].index, 5);
	KUNIT_EXPECT_EQ(test, bindings[0].class, 0);
	KUNIT_EXPECT_EQ(test, bindings[1].index, 4);
	KUNIT_EXPECT_EQ(test, bindings[1].class, 1);
	KUNIT_EXPECT_EQ(test, scratch.index, 6);
	KUNIT_EXPECT_EQ(test, scratch.class, 0);
	KUNIT_EXPECT_EQ(test, scratch.hwx_vm, 0x30000000);
	/* A scratch program needs somewhere to put the scratch binding. */
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, g, aux_size[0], bindings, &count,
						       NULL), -EINVAL);
	/* The scratch entry: input type and direction, exact __bss size, own ID. */
	g[ANE_H16_GENERIC_HEADER + 2 * ANE_H16_GENERIC_ENTRY + 8] = 1;
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, g, aux_size[0], bindings, &count,
						       &scratch), -ENOEXEC);
	g[ANE_H16_GENERIC_HEADER + 2 * ANE_H16_GENERIC_ENTRY + 8] = 0;
	put_unaligned_le64(0x400000, g + ANE_H16_GENERIC_HEADER + 2 * ANE_H16_GENERIC_ENTRY +
			   0x20);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, g, aux_size[0], bindings, &count,
						       &scratch), -ENOEXEC);
	put_unaligned_le64(0x380000, g + ANE_H16_GENERIC_HEADER + 2 * ANE_H16_GENERIC_ENTRY +
			   0x20);
	put_unaligned_le32(5, g + ANE_H16_GENERIC_HEADER + 2 * ANE_H16_GENERIC_ENTRY + 4);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, g, aux_size[0], bindings, &count,
						       &scratch), -ENOEXEC);
	put_unaligned_le32(6, g + ANE_H16_GENERIC_HEADER + 2 * ANE_H16_GENERIC_ENTRY + 4);
	/* Without its scratch entry the section no longer describes the program. */
	put_unaligned_le32(2, g + ANE_H16_GENERIC_COUNT);
	KUNIT_EXPECT_EQ(test, ane_h16_process_bindings(hwx, g, aux_size[0] -
						       ANE_H16_GENERIC_ENTRY, bindings,
						       &count, &scratch), -ENOEXEC);
	put_unaligned_le32(3, g + ANE_H16_GENERIC_COUNT);
	KUNIT_ASSERT_EQ(test, ane_h16_process_bindings(hwx, g, aux_size[0], bindings, &count,
						       &scratch), 0);

	/* Operation refs {3: scratch 6, 6: x 5, 7: y 4, 8: kernel 1007}. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(o + 4 + 8), 4);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(o + 4 + 0xc), 3);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(o + 4 + 0x10), 6);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(o + 4 + 0x24), 8);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(o + 4 + 0x28), 1007);
	KUNIT_EXPECT_EQ(test, ane_h16_operation_validate(snapshot, hwx, o, aux_size[1],
							 bindings, count, &scratch), 0);
	/* The legacy kernel tag 2 asserts the H17 firmware. */
	put_unaligned_le32(2, o + 4 + 0x28);
	KUNIT_EXPECT_EQ(test, ane_h16_operation_validate(snapshot, hwx, o, aux_size[1],
							 bindings, count, &scratch), -ENOEXEC);
	/* A kernel program must bind its kernel. */
	put_unaligned_le32(3, o + 4 + 8);
	KUNIT_EXPECT_EQ(test, ane_h16_operation_validate(snapshot, hwx, o, aux_size[1],
							 bindings, count, &scratch), -ENOEXEC);
	put_unaligned_le32(4, o + 4 + 8);
	put_unaligned_le32(1007, o + 4 + 0x28);
	/* Each tag must name the resource at its slot's descriptor base. */
	put_unaligned_le32(5, o + 4 + 0x10);
	KUNIT_EXPECT_EQ(test, ane_h16_operation_validate(snapshot, hwx, o, aux_size[1],
							 bindings, count, &scratch), -ENOEXEC);
	put_unaligned_le32(6, o + 4 + 0x10);
	put_unaligned_le32(4, o + 4 + 0xc);
	KUNIT_EXPECT_EQ(test, ane_h16_operation_validate(snapshot, hwx, o, aux_size[1],
							 bindings, count, &scratch), -ENOEXEC);
	put_unaligned_le32(6, o + 4 + 0xc);
	KUNIT_EXPECT_EQ(test, ane_h16_operation_validate(snapshot, hwx, o, aux_size[1],
							 bindings, count, &scratch), -ENOEXEC);
	put_unaligned_le32(3, o + 4 + 0xc);
	/* Sizes follow the record count. */
	KUNIT_EXPECT_EQ(test, ane_h16_operation_validate(snapshot, hwx, o, aux_size[1] - 4,
							 bindings, count, &scratch), -ENOEXEC);
	put_unaligned_le32(0, o);
	KUNIT_EXPECT_EQ(test, ane_h16_operation_validate(snapshot, hwx, o, aux_size[1],
							 bindings, count, &scratch), -ENOEXEC);
	put_unaligned_le32(1, o);
	KUNIT_EXPECT_EQ(test, ane_h16_operation_validate(snapshot, hwx, o, aux_size[1],
							 bindings, count, &scratch), 0);
}

/*
 * The MobileNet program through an H17 session: LOAD places the whole
 * kernel pages, EXECUTE names x, y and the kernel-owned scratch, and only
 * the first EXECUTE after the program change is queued.
 */
static void program_h17_mobilenet_test(struct kunit *test)
{
	static const u32 ids[3] = { 5, 4, 6 }, classes[3] = { 0, 1, 0 };
	/* Whole HWX sections: the input is 0x49800 of tensor in a 0x4c000 section. */
	static const u64 sizes[3] = { 0x4c000, 458752, 0x380000 };
	struct ane_h16_hwx_snapshot *snapshot = mobilenet_snapshot(test);
	struct prog_fw *f = fw_create_family(test, 17);
	struct ane_h16_program *p = NULL;
	const void *aux[ANE_H16_AUX_COUNT];
	size_t aux_size[ANE_H16_AUX_COUNT];
	const u8 *slot;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	mobilenet_sections(test, aux, aux_size);
	for (i = 0; i < ANE_H16_AUX_COUNT; i++)
		KUNIT_ASSERT_NOT_NULL(test, aux[i]);
	KUNIT_ASSERT_EQ(test, ane_h16_session_load(&f->s, snapshot, aux, aux_size, &p), 0);
	KUNIT_ASSERT_NOT_NULL(test, p);
	KUNIT_EXPECT_EQ(test, p->resource_count, 2);
	KUNIT_EXPECT_TRUE(test, p->has_scratch);
	KUNIT_EXPECT_EQ(test, p->scratch.index, 6);
	KUNIT_EXPECT_EQ(test, p->scratch.size, 0x380000);
	KUNIT_EXPECT_EQ(test, p->bar_count, 0);
	/* Output and scratch are the only device-writable mappings. */
	KUNIT_EXPECT_EQ(test, f->writable_maps, 2);
	slot = f->load + ANE_H16_LOAD_SLOT_OFFSET;
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(slot + ANE_H16_LOAD_SLOT_SIZE + 4), 1007);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(slot + ANE_H16_LOAD_SLOT_SIZE + 8), 3);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(slot + ANE_H16_LOAD_SLOT_SIZE + 0x20), 0x820000);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(slot + 2 * ANE_H16_LOAD_SLOT_SIZE + 0x20),
			0x3110);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32((const u8 *)p->aux[3].cpu + 0xc), 28);

	for (i = 0; i < 3; i++)
		KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, f->exec_length, ANE_H16_EXECUTE_BASE_SIZE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x28), 3);
	for (i = 0; i < 3; i++) {
		const u8 *d = f->exec + 0x60 + i * 0x30;

		KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 4), ids[i]);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(d + 8), classes[i]);
		KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x18),
				i < 2 ? p->surfaces[i]->dva : p->scratch.mapping_dva);
		KUNIT_EXPECT_EQ(test, get_unaligned_le64(d + 0x20), sizes[i]);
	}
	KUNIT_ASSERT_EQ(test, f->execs, 3);
	KUNIT_EXPECT_EQ(test, f->exec_flags[0], ANE_H16_EXECUTE_FLAG_QUEUED);
	KUNIT_EXPECT_EQ(test, f->exec_flags[1], 0);
	KUNIT_EXPECT_EQ(test, f->exec_flags[2], 0);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
}

/*
 * An H17 session switches between programs: a second LOAD succeeds, and
 * every EXECUTE that follows a different program, a LOAD or a CREATE is
 * queued. A program built for the other
 * engine is refused before any traffic.
 */
static void program_h17_switch_test(struct kunit *test)
{
	static const u32 queued[] = { 1, 0, 1, 1, 0, 1 };
	struct prog_fw *f = fw_create_family(test, 17), *h16 = fw_create(test);
	struct ane_h16_program *a = NULL, *b = NULL, *c = NULL;
	struct ane_h16_hwx_snapshot *snapshot;
	const void *aux[ANE_H16_AUX_COUNT];
	size_t aux_size[ANE_H16_AUX_COUNT];
	unsigned int i, sent;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, h16);
	/* Two H17 programs: two loads of the MobileNet program. */
	snapshot = mobilenet_snapshot(test);
	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	mobilenet_sections(test, aux, aux_size);
	KUNIT_ASSERT_EQ(test, ane_h16_session_load(&f->s, snapshot, aux, aux_size, &a), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, a), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, a), 0);
	snapshot = mobilenet_snapshot(test);
	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	KUNIT_ASSERT_EQ(test, ane_h16_session_load(&f->s, snapshot, aux, aux_size, &b), 0);
	KUNIT_EXPECT_NE(test, a->key.program, b->key.program);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, b), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, a), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, a), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, a), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, b), 0);
	KUNIT_ASSERT_EQ(test, f->execs, ARRAY_SIZE(queued));
	for (i = 0; i < ARRAY_SIZE(queued); i++)
		KUNIT_EXPECT_EQ(test, f->exec_flags[i], queued[i]);
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, a), -EINVAL);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, b), 0);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);

	/* An H17 program on the H16 session, and an H16 one on the H17 session. */
	sent = h16->count;
	snapshot = mobilenet_snapshot(test);
	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	KUNIT_EXPECT_EQ(test, ane_h16_session_load(&h16->s, snapshot, aux, aux_size, &c),
			-ENOEXEC);
	KUNIT_EXPECT_EQ(test, h16->count, sent);
	sent = f->count;
	KUNIT_EXPECT_EQ(test, fw_load_program(f, &c), -ENOEXEC);
	KUNIT_EXPECT_EQ(test, f->count, sent);
	KUNIT_EXPECT_PTR_EQ(test, c, NULL);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/*
 * H17 sends no custom BAR record, even for a linear program whose descriptor
 * names the mcache field: the H17 firmware asserts on the record.
 */
static void program_h17_no_custom_bar_test(struct kunit *test)
{
	struct prog_fw *f = fw_create_family(test, 17);
	struct ane_h16_program *p = NULL;

	KUNIT_ASSERT_NOT_NULL(test, f);
	/* The linear add128 fixture is an H16 HWX; let this H17 session take it. */
	f->soc.hwx_subtype = ANE_HWX_SUBTYPE_H16;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_ASSERT_NOT_NULL(test, p);
	KUNIT_EXPECT_EQ(test, p->bar_count, 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, f->exec_length, ANE_H16_EXECUTE_BASE_SIZE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xca0), 0);
	/*
	 * H17 descriptors carry the size the generic section declared (this
	 * fixture declares the span), not the larger HWX section.
	 */
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(f->exec + 0x60 + 0x20), p->hwx.surfaces[0].span);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(f->exec + 0x90 + 0x20), p->hwx.surfaces[1].span);
	KUNIT_EXPECT_NE(test, p->hwx.surfaces[0].span, p->hwx.surfaces[0].section.size);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x30), ANE_H16_EXECUTE_FLAG_QUEUED);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/*
 * Cached program surfaces: LOAD writes back every mapping it filled, and
 * the per-request sync writes back every surface before the engine and
 * drops the output lines after it. Coherent surfaces (no handle) are never
 * synced.
 */
static void program_cached_sync_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test), *g = fw_create(test);
	struct ane_h16_program *p = NULL, *q = NULL;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, g);
	f->cached = true;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_ASSERT_NOT_NULL(test, p);
	/* Every HWX region and every aux section, once. */
	KUNIT_EXPECT_EQ(test, f->syncs_device, p->plan.count + ANE_H16_AUX_COUNT);
	KUNIT_EXPECT_EQ(test, f->syncs_cpu, 0);
	f->syncs_device = 0;
	ane_h16_session_sync(&f->s, p, true);
	KUNIT_EXPECT_EQ(test, f->syncs_device, p->resource_count);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	ane_h16_session_sync(&f->s, p, false);
	/* One output: only its lines are dropped. */
	KUNIT_EXPECT_EQ(test, f->syncs_cpu, 1);
	KUNIT_EXPECT_EQ(test, f->last_sync_dva, p->surfaces[1]->dva);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);

	/* Coherent memory: nothing to maintain. */
	KUNIT_ASSERT_EQ(test, fw_load_program(g, &q), 0);
	ane_h16_session_sync(&g->s, q, true);
	ane_h16_session_sync(&g->s, q, false);
	KUNIT_EXPECT_EQ(test, g->syncs_device + g->syncs_cpu, 0);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&g->s, q), 0);
}

/*
 * Fifty programs loaded, run and closed one after another (each close is
 * what the last close of the accel file does) map memory only for the
 * first: every later LOAD is served from the spares the previous program
 * gave back, zeroed, and each closed program is freed.
 */
static void program_cycle_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p;
	unsigned int i, maps = 0, spares = 0;

	KUNIT_ASSERT_NOT_NULL(test, f);
	for (i = 0; i < 50; i++) {
		p = NULL;
		KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
		KUNIT_ASSERT_NOT_NULL(test, p);
		if (!i)
			maps = f->maps;
		/* Reused memory comes back zeroed before LOAD fills it. */
		KUNIT_EXPECT_FALSE(test, memchr_inv(p->surfaces[1]->cpu, 0,
						    p->surfaces[1]->size));
		KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
		memset(p->surfaces[1]->cpu, 0x5a, p->surfaces[1]->size);
		KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, p), 0);
		if (!i)
			spares = f->s.spare_count;
		ane_h16_session_put(&f->s, p);
	}
	KUNIT_EXPECT_EQ(test, f->maps, maps);
	KUNIT_EXPECT_EQ(test, f->s.spare_count, spares);
	KUNIT_EXPECT_GT(test, spares, 0U);
	KUNIT_EXPECT_PTR_EQ(test, f->s.program, NULL);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/* The same for H17 MobileNet, with its kernel-owned scratch. */
static void program_h17_cycle_test(struct kunit *test)
{
	struct prog_fw *f = fw_create_family(test, 17);
	struct ane_h16_hwx_snapshot *snapshot;
	const void *aux[ANE_H16_AUX_COUNT];
	size_t aux_size[ANE_H16_AUX_COUNT];
	struct ane_h16_program *p;
	unsigned int i, maps = 0;

	KUNIT_ASSERT_NOT_NULL(test, f);
	mobilenet_sections(test, aux, aux_size);
	for (i = 0; i < 50; i++) {
		snapshot = mobilenet_snapshot(test);
		KUNIT_ASSERT_NOT_NULL(test, snapshot);
		p = NULL;
		KUNIT_ASSERT_EQ(test, ane_h16_session_load(&f->s, snapshot, aux, aux_size, &p), 0);
		if (!i)
			maps = f->maps;
		KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
		/* Every program's first request is queued, reused memory or not. */
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x30),
				ANE_H16_EXECUTE_FLAG_QUEUED);
		KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, p), 0);
		ane_h16_session_put(&f->s, p);
	}
	KUNIT_EXPECT_EQ(test, f->maps, maps);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/* A program whose teardown failed keeps its memory and is never freed. */
static void program_failed_close_keeps_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	f->s.quarantined = true;
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), -EIO);
	KUNIT_EXPECT_FALSE(test, p->released);
	KUNIT_EXPECT_EQ(test, f->s.spare_count, 0U);
	ane_h16_session_put(&f->s, p);
	/* Still there: put refused a program that was not released. */
	KUNIT_EXPECT_TRUE(test, p->closed);
	KUNIT_EXPECT_NOT_NULL(test, p->surfaces[0]);
}

/*
 * The busy hook brackets every EXECUTE that reaches the firmware, pass or
 * fail, exactly once and never nested; a refused call never votes.
 */
static void program_busy_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test), *g = fw_create(test);
	struct ane_h16_program *p = NULL, *q = NULL;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, g);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_EXPECT_EQ(test, f->busy_on, 0);
	for (i = 0; i < 5; i++)
		KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, f->busy_on, 5);
	KUNIT_EXPECT_EQ(test, f->busy_off, 5);
	KUNIT_EXPECT_EQ(test, f->busy_max, 1);
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, NULL), -EINVAL);
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&g->s, p), -EINVAL);
	KUNIT_EXPECT_EQ(test, f->busy_on + g->busy_on, 5);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);

	/* A stalled request still releases the vote. */
	KUNIT_ASSERT_EQ(test, fw_load_program(g, &q), 0);
	g->mode = FW_REPORT_STALL;
	KUNIT_EXPECT_NE(test, ane_h16_session_execute(&g->s, q), 0);
	KUNIT_EXPECT_EQ(test, g->busy_on, 1);
	KUNIT_EXPECT_EQ(test, g->busy_off, 1);
	KUNIT_EXPECT_EQ(test, g->busy_depth, 0);
}

/*
 * While the firmware is suspended for system sleep, requests wait at the
 * gate and a close is deferred: no firmware traffic until the session
 * reopens, then the normal TERMINATE and UNLOAD, and the program survives
 * the sleep untouched until then (no LOAD or CREATE replay).
 */
static void program_sleep_gate_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	unsigned int sent, spares;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.lock = lock;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);

	ane_h16_session_sleep(&f->s, true);
	KUNIT_EXPECT_TRUE(test, f->s.asleep);
	sent = f->count;
	spares = f->s.spare_count;
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), -EINPROGRESS);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), -EINPROGRESS);
	KUNIT_EXPECT_EQ(test, f->count, sent);
	KUNIT_EXPECT_FALSE(test, p->closed);

	/* Reopen: the deferred close runs once, with its firmware traffic. */
	ane_h16_session_sleep(&f->s, false);
	KUNIT_EXPECT_FALSE(test, f->s.asleep);
	KUNIT_EXPECT_GT(test, f->count, sent);
	KUNIT_EXPECT_GT(test, f->s.spare_count, spares);
	KUNIT_EXPECT_TRUE(test, list_empty(&f->s.deferred));
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);

	/* The gate lets requests in again. */
	KUNIT_ASSERT_EQ(test, ane_h16_session_enter(&f->s, false), 0);
	mutex_unlock(lock);
}

/* A program survives a sleep: the same handle executes afterwards. */
static void program_sleep_keeps_program_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	unsigned int loads;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.lock = lock;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	loads = f->programs_live;
	ane_h16_session_sleep(&f->s, true);
	ane_h16_session_sleep(&f->s, false);
	KUNIT_EXPECT_EQ(test, f->programs_live, loads);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	/* Same program on the engine: the direct path, no requeue. */
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x30), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
}

/*
 * A cold restart across system sleep: a program from before is lost
 * (EXECUTE refuses it with no firmware traffic, close only releases its
 * memory) and a new program loads into the spares it gave back, its first
 * EXECUTE queued.
 */
static void program_sleep_restart_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL, *q = NULL;
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	unsigned int maps, sent;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.lock = lock;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	maps = f->maps;

	ane_h16_session_sleep(&f->s, true);
	ane_h16_session_restart(&f->s);
	ane_h16_session_sleep(&f->s, false);
	KUNIT_EXPECT_FALSE(test, f->s.asleep);
	KUNIT_EXPECT_TRUE(test, ane_h16_program_lost(&f->s, p));
	KUNIT_EXPECT_PTR_EQ(test, f->s.program, NULL);

	sent = f->count;
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), -ENODEV);
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), -ENODEV);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, f->count, sent);
	KUNIT_EXPECT_TRUE(test, p->released);
	KUNIT_EXPECT_GT(test, f->s.spare_count, 0U);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	ane_h16_session_put(&f->s, p);

	/* The gate lets requests in again; the new program reuses memory. */
	KUNIT_ASSERT_EQ(test, ane_h16_session_enter(&f->s, false), 0);
	mutex_unlock(lock);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &q), 0);
	KUNIT_EXPECT_EQ(test, f->maps, maps);
	KUNIT_EXPECT_FALSE(test, ane_h16_program_lost(&f->s, q));
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, q), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x30), ANE_H16_EXECUTE_FLAG_QUEUED);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, q), 0);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/*
 * A close deferred while the engine is powered off for a cold cycle runs
 * at the reopen as a release only: the new firmware never knew the program.
 */
static void program_sleep_restart_deferred_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	unsigned int sent, spares;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.lock = lock;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);

	ane_h16_session_sleep(&f->s, true);
	sent = f->count;
	spares = f->s.spare_count;
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), -EINPROGRESS);
	ane_h16_session_restart(&f->s);
	/* The reopen releases and frees it, with no TERMINATE or UNLOAD. */
	ane_h16_session_sleep(&f->s, false);
	KUNIT_EXPECT_EQ(test, f->count, sent);
	KUNIT_EXPECT_GT(test, f->s.spare_count, spares);
	KUNIT_EXPECT_TRUE(test, list_empty(&f->s.deferred));
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/*
 * A cold cycle with replay is invisible to the application: the same handle
 * runs after it, on new firmware IDs, the firmware got the identical program
 * image (LOAD bytes, regions rewritten from the HWX) and the EXECUTE carries
 * the identical buffer table, so the outputs are the same.
 */
static void program_sleep_replay_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	u8 *load = kunit_kzalloc(test, ANE_H16_LOAD_SIZE, GFP_KERNEL);
	u8 *exec = kunit_kzalloc(test, sizeof(f->exec), GFP_KERNEL);
	unsigned int replayed, i;
	size_t exec_length;
	u32 program, process;
	u64 first;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	KUNIT_ASSERT_NOT_NULL(test, load);
	KUNIT_ASSERT_NOT_NULL(test, exec);
	mutex_init(lock);
	f->s.lock = lock;
	f->strict = true;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	memcpy(load, f->load, ANE_H16_LOAD_SIZE);
	memcpy(exec, f->exec, sizeof(f->exec));
	exec_length = f->exec_length;
	program = p->key.program;
	process = p->key.process;
	/* The engine wrote into the program's memory while it ran. */
	for (i = 0; i < p->plan.count; i++)
		memset(p->regions[i].cpu, 0xa5, p->regions[i].size);

	ane_h16_session_sleep(&f->s, true);
	fw_cold_boot(f);
	ane_h16_session_restart(&f->s);
	KUNIT_EXPECT_TRUE(test, ane_h16_program_lost(&f->s, p));
	KUNIT_ASSERT_EQ(test, ane_h16_session_replay(&f->s, &replayed, &first), 0);
	ane_h16_session_sleep(&f->s, false);
	KUNIT_EXPECT_EQ(test, replayed, 1U);
	KUNIT_EXPECT_FALSE(test, ane_h16_program_lost(&f->s, p));
	KUNIT_EXPECT_EQ(test, p->key.program, program + 0x10);
	KUNIT_EXPECT_EQ(test, p->key.process, process + 0x10);
	KUNIT_EXPECT_EQ(test, memcmp(f->load + 8, load + 8, ANE_H16_LOAD_SIZE - 8), 0);
	for (i = 0; i < p->plan.count; i++) {
		const struct ane_h16_hwx_region *r = &p->plan.regions[i];

		KUNIT_EXPECT_EQ(test, memcmp(p->regions[i].cpu,
					     (const u8 *)p->snapshot.bytes + r->file_offset,
					     r->file_size), 0);
		KUNIT_EXPECT_FALSE(test, memchr_inv((u8 *)p->regions[i].cpu + r->file_size, 0,
						    p->regions[i].size - r->file_size));
	}

	/* The same handle runs on the new firmware IDs, queued first. */
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 8), program + 0x10);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0xc), process + 0x10);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->exec + 0x30), ANE_H16_EXECUTE_FLAG_QUEUED);
	KUNIT_EXPECT_EQ(test, f->exec_length, exec_length);
	KUNIT_EXPECT_EQ(test, memcmp(f->exec + 0x34, exec + 0x34, exec_length - 0x34), 0);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	ane_h16_session_put(&f->s, p);
}

/* Requests during the cycle wait at the gate; O_NONBLOCK gets -EAGAIN. */
static void program_sleep_nonblock_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.lock = lock;
	ane_h16_session_sleep(&f->s, true);
	KUNIT_EXPECT_EQ(test, ane_h16_session_enter(&f->s, true), -EAGAIN);
	ane_h16_session_sleep(&f->s, false);
	KUNIT_ASSERT_EQ(test, ane_h16_session_enter(&f->s, true), 0);
	mutex_unlock(lock);
	KUNIT_ASSERT_EQ(test, ane_h16_session_enter(&f->s, false), 0);
	mutex_unlock(lock);
}

/* A replay the new firmware refuses is reported loudly and quarantines. */
static void program_sleep_replay_failure_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	unsigned int replayed;
	u64 first;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.lock = lock;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	ane_h16_session_sleep(&f->s, true);
	fw_cold_boot(f);
	f->mode = FW_LOAD_NAK;
	ane_h16_session_restart(&f->s);
	KUNIT_EXPECT_NE(test, ane_h16_session_replay(&f->s, &replayed, &first), 0);
	ane_h16_session_sleep(&f->s, false);
	KUNIT_EXPECT_EQ(test, replayed, 0U);
	KUNIT_EXPECT_EQ(test, f->replay_failed, 1U);
	KUNIT_EXPECT_TRUE(test, f->s.quarantined);
	KUNIT_EXPECT_TRUE(test, ane_h16_program_lost(&f->s, p));
}

/* A program closed while asleep is not replayed, only released. */
static void program_sleep_replay_deferred_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	unsigned int replayed, sent;
	u64 first;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.lock = lock;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	ane_h16_session_sleep(&f->s, true);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), -EINPROGRESS);
	fw_cold_boot(f);
	ane_h16_session_restart(&f->s);
	sent = f->count;
	KUNIT_ASSERT_EQ(test, ane_h16_session_replay(&f->s, &replayed, &first), 0);
	KUNIT_EXPECT_EQ(test, replayed, 0U);
	ane_h16_session_sleep(&f->s, false);
	KUNIT_EXPECT_EQ(test, f->count, sent);
	KUNIT_EXPECT_TRUE(test, list_empty(&f->s.programs));
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/* Real concurrent callers; the fake firmware alone decides completion order. */
struct pipeline_thread {
	struct prog_fw *fw;
	struct ane_h16_program *program;
	struct task_struct *task;
	struct completion entered;
	struct completion done;
	bool sleep;
	int result;
};

static int pipeline_thread_run(void *arg)
{
	struct pipeline_thread *t = arg;
	struct ane_h16_session *s = &t->fw->s;

	complete(&t->entered);
	if (t->sleep) {
		ane_h16_session_sleep(s, true);
	} else {
		mutex_lock(s->lock);
		t->result = ane_h16_session_execute(s, t->program);
		mutex_unlock(s->lock);
	}
	complete(&t->done);
	/* Keep the task alive until the cleanup action calls kthread_stop(). */
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	__set_current_state(TASK_RUNNING);
	return 0;
}

/* Also recovers a failed assertion, before KUnit releases the fake's memory. */
static void pipeline_thread_stop(void *arg)
{
	struct pipeline_thread *t = arg;
	struct ane_h16_session *s = &t->fw->s;

	mutex_lock(s->lock);
	if (s->active) {
		s->quarantined = true;
		s->error = -ECANCELED;
		ane_h16_pending_fail_all(&s->pending, -ECANCELED);
	}
	s->asleep = false;
	wake_up_all(&s->wake);
	mutex_unlock(s->lock);
	kthread_stop(t->task);
	cancel_delayed_work_sync(&s->gate_work);
}

static struct pipeline_thread *pipeline_start(struct kunit *test, struct prog_fw *f,
					      struct ane_h16_program *p)
{
	struct pipeline_thread *t = kunit_kzalloc(test, sizeof(*t), GFP_KERNEL);

	if (!t)
		return NULL;
	t->fw = f;
	t->program = p;
	t->sleep = !p;
	init_completion(&t->entered);
	init_completion(&t->done);
	t->task = kthread_run(pipeline_thread_run, t, "ane-pipeline-test");
	if (IS_ERR(t->task))
		return NULL;
	if (kunit_add_action_or_reset(test, pipeline_thread_stop, t))
		return NULL;
	if (!wait_for_completion_timeout(&t->entered, HZ))
		return NULL;
	return t;
}

static struct prog_fw *pipeline_create(struct kunit *test, unsigned int family,
				       struct ane_h16_program **a,
				       struct ane_h16_program **b)
{
	struct prog_fw *f = fw_create_family(test, family);
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	const void *aux[ANE_H16_AUX_COUNT];
	size_t sizes[ANE_H16_AUX_COUNT];
	int ret;

	if (!f || !lock)
		return NULL;
	if (family == 16) {
		ret = fw_load_program(f, a);
		if (!ret)
			ret = fw_load_program(f, b);
	} else {
		mobilenet_sections(test, aux, sizes);
		ret = ane_h16_session_load(&f->s, mobilenet_snapshot(test), aux, sizes, a);
		if (!ret)
			ret = ane_h16_session_load(&f->s, mobilenet_snapshot(test), aux, sizes, b);
	}
	if (ret)
		return NULL;
	mutex_init(lock);
	f->s.lock = lock;
	f->s.timeout_ns = NSEC_PER_SEC;
	f->mode = FW_REPORT_PIPELINE;
	return f;
}

static bool pipeline_submitted(struct prog_fw *f, unsigned int count)
{
	return wait_event_timeout(f->pipeline_wake, READ_ONCE(f->execs) >= count, HZ);
}

static void pipeline_complete(struct prog_fw *f, unsigned int index, u32 state)
{
	const struct ane_h16_request_key *key = &f->exec_keys[index];

	mutex_lock(f->s.lock);
	fw_report(f, key->uuid, key->program, key->process, state);
	mutex_unlock(f->s.lock);
}

static void pipeline_expect_empty(struct kunit *test, struct prog_fw *f)
{
	unsigned int i;

	KUNIT_EXPECT_EQ(test, f->s.active, 0U);
	KUNIT_EXPECT_EQ(test, f->s.running, 0U);
	KUNIT_EXPECT_EQ(test, f->busy_on, 1U);
	KUNIT_EXPECT_EQ(test, f->busy_off, 1U);
	KUNIT_EXPECT_EQ(test, f->busy_depth, 0);
	for (i = 0; i < ANE_H16_PENDING_MAX; i++)
		KUNIT_EXPECT_PTR_EQ(test, f->s.pending.slots[i], NULL);
}

/* B completes while A remains pending; neither its arena nor gate is released. */
static void program_pipeline_out_of_order_test(struct kunit *test)
{
	unsigned int family;

	for (family = 16; family <= 17; family++) {
		struct ane_h16_program *a, *b;
		struct prog_fw *f = pipeline_create(test, family, &a, &b);
		struct pipeline_thread *ta, *tb;
		size_t mark, used;

		KUNIT_ASSERT_NOT_NULL(test, f);
		mark = f->t.pool_used;
		ta = pipeline_start(test, f, a);
		KUNIT_ASSERT_NOT_NULL(test, ta);
		KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 1));
		tb = pipeline_start(test, f, b);
		KUNIT_ASSERT_NOT_NULL(test, tb);
		KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 2));
		mutex_lock(f->s.lock);
		used = f->t.pool_used;
		KUNIT_EXPECT_EQ(test, f->s.active, 2U);
		KUNIT_EXPECT_EQ(test, f->busy_on, 1U);
		KUNIT_EXPECT_EQ(test, f->busy_off, 0U);
		KUNIT_EXPECT_EQ(test, ane_h16_session_gate_expire(&f->s), -EAGAIN);
		mutex_unlock(f->s.lock);
		KUNIT_EXPECT_NE(test, f->exec_keys[0].uuid, f->exec_keys[1].uuid);
		KUNIT_EXPECT_EQ(test, f->exec_flags[1], ANE_H16_EXECUTE_FLAG_QUEUED);
		pipeline_complete(f, 1, 1);
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&tb->done, HZ), 0UL);
		KUNIT_EXPECT_EQ(test, tb->result, 0);
		KUNIT_EXPECT_FALSE(test, completion_done(&ta->done));
		mutex_lock(f->s.lock);
		KUNIT_EXPECT_TRUE(test, f->s.gate_held);
		KUNIT_EXPECT_EQ(test, f->t.pool_used, used);
		KUNIT_EXPECT_EQ(test, f->busy_off, 0U);
		mutex_unlock(f->s.lock);
		pipeline_complete(f, 0, 1);
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&ta->done, HZ), 0UL);
		KUNIT_EXPECT_EQ(test, ta->result, 0);
		pipeline_expect_empty(test, f);
		KUNIT_EXPECT_EQ(test, f->t.pool_used, mark);
		KUNIT_EXPECT_FALSE(test, f->s.gate_held);
		/* Completion order invalidates the speculative direct BAR context. */
		KUNIT_EXPECT_FALSE(test, f->t.bar_known);
	}
}

/* A second user of the same surfaces cannot publish until the first returns. */
static void program_pipeline_same_program_test(struct kunit *test)
{
	struct ane_h16_program *a, *b;
	struct prog_fw *f = pipeline_create(test, 16, &a, &b);
	struct pipeline_thread *ta, *tb;

	KUNIT_ASSERT_NOT_NULL(test, f);
	ta = pipeline_start(test, f, a);
	KUNIT_ASSERT_NOT_NULL(test, ta);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 1));
	tb = pipeline_start(test, f, a);
	KUNIT_ASSERT_NOT_NULL(test, tb);
	KUNIT_EXPECT_EQ(test, wait_event_timeout(f->pipeline_wake,
						 READ_ONCE(f->execs) >= 2,
						 msecs_to_jiffies(20)), 0L);
	KUNIT_EXPECT_FALSE(test, completion_done(&tb->done));
	pipeline_complete(f, 0, 1);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&ta->done, HZ), 0UL);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 2));
	pipeline_complete(f, 1, 1);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&tb->done, HZ), 0UL);
	KUNIT_EXPECT_EQ(test, ta->result, 0);
	KUNIT_EXPECT_EQ(test, tb->result, 0);
	KUNIT_EXPECT_EQ(test, f->busy_max, 1);
	KUNIT_EXPECT_EQ(test, f->s.active, 0U);
}

/* A third, distinct program waits for a slot, then overlaps the survivor. */
static void program_pipeline_bound_test(struct kunit *test)
{
	struct ane_h16_program *a, *b, *c = NULL;
	struct prog_fw *f = pipeline_create(test, 16, &a, &b);
	struct pipeline_thread *ta, *tb, *tc;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, f);
	mutex_lock(f->s.lock);
	ret = fw_load_program(f, &c);
	mutex_unlock(f->s.lock);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ta = pipeline_start(test, f, a);
	KUNIT_ASSERT_NOT_NULL(test, ta);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 1));
	tb = pipeline_start(test, f, b);
	KUNIT_ASSERT_NOT_NULL(test, tb);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 2));
	tc = pipeline_start(test, f, c);
	KUNIT_ASSERT_NOT_NULL(test, tc);
	KUNIT_EXPECT_EQ(test, wait_event_timeout(f->pipeline_wake,
						 READ_ONCE(f->execs) >= 3,
						 msecs_to_jiffies(20)), 0L);
	KUNIT_EXPECT_FALSE(test, completion_done(&tc->done));
	pipeline_complete(f, 1, 1);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&tb->done, HZ), 0UL);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 3));
	KUNIT_EXPECT_FALSE(test, completion_done(&ta->done));
	pipeline_complete(f, 2, 1);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&tc->done, HZ), 0UL);
	pipeline_complete(f, 0, 1);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&ta->done, HZ), 0UL);
	KUNIT_EXPECT_EQ(test, ta->result, 0);
	KUNIT_EXPECT_EQ(test, tb->result, 0);
	KUNIT_EXPECT_EQ(test, tc->result, 0);
	pipeline_expect_empty(test, f);
}

/* Arena pressure drains a batch rather than failing a long-running pipeline. */
static void program_pipeline_arena_pressure_test(struct kunit *test)
{
	struct ane_h16_program *a, *b;
	struct prog_fw *f = pipeline_create(test, 16, &a, &b);
	struct pipeline_thread *ta, *tb;
	size_t mark;

	KUNIT_ASSERT_NOT_NULL(test, f);
	mark = f->t.pool_used;
	/* Room for one sequence and its teardown, but not a second command. */
	f->t.pool.size = mark + 0xe00;
	ta = pipeline_start(test, f, a);
	KUNIT_ASSERT_NOT_NULL(test, ta);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 1));
	tb = pipeline_start(test, f, b);
	KUNIT_ASSERT_NOT_NULL(test, tb);
	KUNIT_EXPECT_EQ(test, wait_event_timeout(f->pipeline_wake,
						 READ_ONCE(f->execs) >= 2,
						 msecs_to_jiffies(20)), 0L);
	pipeline_complete(f, 0, 1);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&ta->done, HZ), 0UL);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 2));
	pipeline_complete(f, 1, 1);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&tb->done, HZ), 0UL);
	KUNIT_EXPECT_EQ(test, ta->result, 0);
	KUNIT_EXPECT_EQ(test, tb->result, 0);
	KUNIT_EXPECT_EQ(test, f->s.active, 0U);
	KUNIT_EXPECT_EQ(test, f->t.pool_used, mark);
	KUNIT_EXPECT_EQ(test, f->busy_on, 2U);
	KUNIT_EXPECT_EQ(test, f->busy_off, 2U);
}

/* Copy-out retains ownership after the terminal report and busy vote end. */
static void program_pipeline_surface_owner_test(struct kunit *test)
{
	struct ane_h16_program *a, *b;
	struct prog_fw *f = pipeline_create(test, 16, &a, &b);
	struct pipeline_thread *t;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, f);
	/* Emulate the ioctl between its acquire and run, before copy-in. */
	mutex_lock(f->s.lock);
	ret = ane_h16_session_acquire(&f->s, a);
	mutex_unlock(f->s.lock);
	KUNIT_ASSERT_EQ(test, ret, 0);
	t = pipeline_start(test, f, a);
	KUNIT_ASSERT_NOT_NULL(test, t);
	KUNIT_EXPECT_EQ(test, wait_event_timeout(f->pipeline_wake,
						 READ_ONCE(f->execs) >= 1,
						 msecs_to_jiffies(20)), 0L);
	/* Copy-out uses the same ownership; no firmware need be running. */
	mutex_lock(f->s.lock);
	KUNIT_EXPECT_EQ(test, f->s.running, 0U);
	ane_h16_session_release(&f->s, a);
	mutex_unlock(f->s.lock);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 1));
	pipeline_complete(f, 0, 1);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&t->done, HZ), 0UL);
	KUNIT_EXPECT_EQ(test, t->result, 0);
	pipeline_expect_empty(test, f);
}

/* A timeout or malformed report fails both waiters and retains the whole arena. */
static void program_pipeline_fault(struct kunit *test, bool timeout)
{
	struct ane_h16_program *a, *b;
	struct prog_fw *f = pipeline_create(test, 16, &a, &b);
	struct pipeline_thread *ta, *tb;
	size_t mark;
	int error = timeout ? -ETIMEDOUT : -EPROTO;

	KUNIT_ASSERT_NOT_NULL(test, f);
	mark = f->t.pool_used;
	ta = pipeline_start(test, f, a);
	KUNIT_ASSERT_NOT_NULL(test, ta);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 1));
	/* Only A expires; B's deadline remains a full second later. */
	mutex_lock(f->s.lock);
	f->now += NSEC_PER_MSEC;
	mutex_unlock(f->s.lock);
	tb = pipeline_start(test, f, b);
	KUNIT_ASSERT_NOT_NULL(test, tb);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 2));
	mutex_lock(f->s.lock);
	if (timeout) {
		f->now = NSEC_PER_SEC;
	} else {
		const struct ane_h16_request_key *key = &f->exec_keys[0];

		fw_report(f, key->uuid, key->program, key->process + 1, 1);
	}
	mutex_unlock(f->s.lock);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&ta->done, HZ), 0UL);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&tb->done, HZ), 0UL);
	KUNIT_EXPECT_EQ(test, ta->result, error);
	KUNIT_EXPECT_EQ(test, tb->result, error);
	KUNIT_EXPECT_TRUE(test, f->s.quarantined);
	KUNIT_EXPECT_GT(test, f->t.pool_used, mark);
	pipeline_expect_empty(test, f);
}

static void program_pipeline_timeout_test(struct kunit *test)
{
	program_pipeline_fault(test, true);
}

static void program_pipeline_quarantine_test(struct kunit *test)
{
	program_pipeline_fault(test, false);
}

/* Sleep closes admission, then waits for both callers before closing the gate. */
static void program_pipeline_sleep_test(struct kunit *test)
{
	struct ane_h16_program *a, *b;
	struct prog_fw *f = pipeline_create(test, 16, &a, &b);
	struct pipeline_thread *ta, *tb, *sleep;
	unsigned int tries;

	KUNIT_ASSERT_NOT_NULL(test, f);
	ta = pipeline_start(test, f, a);
	KUNIT_ASSERT_NOT_NULL(test, ta);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 1));
	tb = pipeline_start(test, f, b);
	KUNIT_ASSERT_NOT_NULL(test, tb);
	KUNIT_ASSERT_TRUE(test, pipeline_submitted(f, 2));
	sleep = pipeline_start(test, f, NULL);
	KUNIT_ASSERT_NOT_NULL(test, sleep);
	for (tries = 0; tries < 100 && !READ_ONCE(f->s.asleep); tries++)
		usleep_range(100, 200);
	KUNIT_ASSERT_TRUE(test, READ_ONCE(f->s.asleep));
	KUNIT_EXPECT_FALSE(test, completion_done(&sleep->done));
	pipeline_complete(f, 0, 1);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&ta->done, HZ), 0UL);
	KUNIT_EXPECT_FALSE(test, completion_done(&sleep->done));
	pipeline_complete(f, 1, 1);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&tb->done, HZ), 0UL);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&sleep->done, HZ), 0UL);
	KUNIT_EXPECT_EQ(test, ta->result, 0);
	KUNIT_EXPECT_EQ(test, tb->result, 0);
	KUNIT_EXPECT_TRUE(test, f->s.asleep);
	KUNIT_EXPECT_FALSE(test, f->s.gate_held);
	pipeline_expect_empty(test, f);
	ane_h16_session_sleep(&f->s, false);
}

/*
 * A hung request quarantines the session and tells the backend once. A
 * program closed while quarantined is kept, memory and all, until the
 * backend booted a new firmware (ane_h16_session_recovered()), which ends
 * the quarantine and gives that memory back; an open program is replayed and
 * runs again behind its handle.
 */
static void program_hung_recovery_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL, *q = NULL;
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	unsigned int replayed, spares;
	u64 first;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.ops = &fw_session_ops_hung;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &q), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p), 0);

	f->mode = FW_REPORT_STALL;
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, q), -ETIMEDOUT);
	KUNIT_EXPECT_TRUE(test, f->s.quarantined);
	KUNIT_EXPECT_EQ(test, f->hung, 1U);
	KUNIT_EXPECT_EQ(test, f->hung_error, -ETIMEDOUT);
	/* Later failures while quarantined do not ask again. */
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), -EIO);
	KUNIT_EXPECT_EQ(test, f->hung, 1U);

	/* Closed while quarantined: kept on the held list, not released. */
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, q), -EIO);
	KUNIT_EXPECT_FALSE(test, q->released);
	KUNIT_EXPECT_FALSE(test, list_empty(&f->s.held));
	spares = f->s.spare_count;

	/* The backend powered the engine off and booted a new firmware. */
	f->s.lock = lock;
	ane_h16_session_sleep(&f->s, true);
	fw_cold_boot(f);
	f->mode = FW_OK;
	ane_h16_session_restart(&f->s);
	ane_h16_session_recovered(&f->s);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	KUNIT_EXPECT_FALSE(test, f->s.pending.quarantined);
	KUNIT_EXPECT_TRUE(test, list_empty(&f->s.held));
	KUNIT_EXPECT_GT(test, f->s.spare_count, spares);
	KUNIT_ASSERT_EQ(test, ane_h16_session_replay(&f->s, &replayed, &first), 0);
	KUNIT_EXPECT_EQ(test, replayed, 1U);
	ane_h16_session_sleep(&f->s, false);

	KUNIT_EXPECT_FALSE(test, ane_h16_program_lost(&f->s, p));
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	KUNIT_EXPECT_EQ(test, f->hung, 1U);
	ane_h16_session_put(&f->s, p);
}

/* Load, run, close and free one program. */
static int fw_cycle(struct prog_fw *f)
{
	struct ane_h16_program *p = NULL;
	int ret = fw_load_program(f, &p);

	if (ret)
		return ret;
	ret = ane_h16_session_execute(&f->s, p);
	if (!ret)
		ret = ane_h16_session_close(&f->s, p);
	if (!ret)
		ane_h16_session_put(&f->s, p);
	return ret;
}

/* Records the session holds in its placed range. */
static unsigned int fw_records(struct prog_fw *f)
{
	return bitmap_weight(f->s.records, ANE_H16_RECORDS);
}

/* The LOAD and CREATE bytes of one program's record, as the firmware sees them. */
static const u8 *fw_record(struct prog_fw *f, const struct ane_h16_program *p)
{
	return (const u8 *)f->pool.cpu + f->t.placed + p->record * ANE_H16_RECORD_SIZE;
}

/*
 * One client's load/close cycles give back all of their arena: LOAD and
 * CREATE through their record, the rest at the arena's end. A quarantined
 * session keeps both.
 */
static void program_pool_rewind_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	size_t base, kept;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	base = ane_fw_transport_mark(&f->t);
	KUNIT_ASSERT_EQ(test, fw_cycle(f), 0);
	/* The first LOAD of a firmware boot takes the records, once. */
	KUNIT_EXPECT_EQ(test, f->t.placed, base);
	KUNIT_EXPECT_EQ(test, f->t.placed_size, (size_t)ANE_H16_RECORDS * ANE_H16_RECORD_SIZE);
	base = ane_fw_transport_mark(&f->t);
	KUNIT_EXPECT_EQ(test, base, f->t.placed + f->t.placed_size);
	for (i = 0; i < 1000; i++) {
		KUNIT_ASSERT_EQ(test, fw_cycle(f), 0);
		KUNIT_ASSERT_EQ(test, ane_fw_transport_mark(&f->t), base);
		KUNIT_ASSERT_EQ(test, fw_records(f), 0U);
	}

	/* A quarantined session keeps the arena and the record as they are. */
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_EXPECT_EQ(test, p->record, 0);
	kept = ane_fw_transport_mark(&f->t);
	f->s.quarantined = true;
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, p), -EIO);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_mark(&f->t), kept);
	KUNIT_EXPECT_EQ(test, fw_records(f), 1U);
	KUNIT_EXPECT_FALSE(test, list_empty(&f->s.held));
}

/*
 * One client keeps a program loaded and runs it while another loads, runs
 * and closes far more programs than the arena held before records (about
 * 210): no cycle leaves anything behind, the resident program's LOAD and
 * CREATE stay untouched, and its EXECUTEs keep working throughout.
 */
static void program_pool_resident_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *a = NULL, *b;
	u8 *record = kunit_kzalloc(test, ANE_H16_RECORD_SIZE, GFP_KERNEL);
	size_t base;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, record);
	f->strict = true;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &a), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, a), 0);
	memcpy(record, fw_record(f, a), ANE_H16_RECORD_SIZE);
	base = ane_fw_transport_mark(&f->t);
	for (i = 0; i < 1000; i++) {
		b = NULL;
		KUNIT_ASSERT_EQ(test, fw_load_program(f, &b), 0);
		/* The lowest free record: the resident program holds record 0. */
		KUNIT_ASSERT_EQ(test, b->record, 1);
		KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, b), 0);
		KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, a), 0);
		KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, b), 0);
		ane_h16_session_put(&f->s, b);
		KUNIT_ASSERT_EQ(test, ane_fw_transport_mark(&f->t), base);
		KUNIT_ASSERT_EQ(test, fw_records(f), 1U);
	}
	KUNIT_EXPECT_EQ(test, memcmp(fw_record(f, a), record, ANE_H16_RECORD_SIZE), 0);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, a), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, a), 0);
	ane_h16_session_put(&f->s, a);
	KUNIT_EXPECT_EQ(test, fw_records(f), 0U);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_mark(&f->t), base);
}

/*
 * Programs closed out of order free their records for the next LOADs, lowest
 * first; the programs still loaded keep theirs and keep running.
 */
static void program_pool_records_reuse_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p[24] = {};
	size_t base = 0;
	unsigned int i, round;

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->strict = true;
	for (round = 0; round < 50; round++) {
		for (i = 0; i < ARRAY_SIZE(p); i++) {
			if (p[i])
				continue;
			KUNIT_ASSERT_EQ(test, fw_load_program(f, &p[i]), 0);
			if (!base)
				base = ane_fw_transport_mark(&f->t);
		}
		for (i = 0; i < ARRAY_SIZE(p); i++)
			KUNIT_ASSERT_EQ(test, p[i]->record, (int)i);
		KUNIT_ASSERT_EQ(test, fw_records(f), (unsigned int)ARRAY_SIZE(p));
		/* Close a different pattern each round: every 2nd, 3rd, ... program. */
		for (i = round % 5; i < ARRAY_SIZE(p); i += 2 + round % 4) {
			KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, p[i]), 0);
			ane_h16_session_put(&f->s, p[i]);
			p[i] = NULL;
		}
		for (i = 0; i < ARRAY_SIZE(p); i++)
			if (p[i])
				KUNIT_ASSERT_EQ(test, ane_h16_session_execute(&f->s, p[i]), 0);
		KUNIT_ASSERT_EQ(test, ane_fw_transport_mark(&f->t), base);
	}
	for (i = 0; i < ARRAY_SIZE(p); i++) {
		if (!p[i])
			continue;
		KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, p[i]), 0);
		ane_h16_session_put(&f->s, p[i]);
	}
	KUNIT_EXPECT_EQ(test, fw_records(f), 0U);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_mark(&f->t), base);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/*
 * With every record in use a LOAD is a clean -ENOSPC: nothing is sent or
 * mapped and the session stays usable. The records taken by other programs
 * are faked: the fake firmware holds 32 programs at most.
 */
static void program_pool_records_full_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL, *q = NULL;
	unsigned int sent, maps;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, fw_cycle(f), 0);
	bitmap_set(f->s.records, 0, ANE_H16_RECORDS - 1);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_EXPECT_EQ(test, p->record, ANE_H16_RECORDS - 1);
	KUNIT_EXPECT_TRUE(test, bitmap_full(f->s.records, ANE_H16_RECORDS));
	sent = f->count;
	maps = f->maps;
	KUNIT_EXPECT_EQ(test, fw_load_program(f, &q), -ENOSPC);
	KUNIT_EXPECT_NULL(test, q);
	KUNIT_EXPECT_EQ(test, f->count, sent);
	KUNIT_EXPECT_EQ(test, f->maps, maps);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	/* EXECUTE does not use records: it still runs. */
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, p), 0);

	/* A freed record in the middle serves the next LOAD, in place. */
	__clear_bit(40, f->s.records);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &q), 0);
	KUNIT_EXPECT_EQ(test, q->record, 40);
	/* The firmware read this LOAD from the record. */
	KUNIT_EXPECT_EQ(test, memcmp(f->load + 8, fw_record(f, q) + 8,
				     ANE_H16_LOAD_ID_OFFSET - 8), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, q), 0);
	KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, q), 0);
	ane_h16_session_put(&f->s, q);
	KUNIT_EXPECT_FALSE(test, test_bit(40, f->s.records));
	KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	ane_h16_session_put(&f->s, p);
	KUNIT_EXPECT_EQ(test, fw_records(f), (unsigned int)ANE_H16_RECORDS - 2);
}

/* No room for the records at the first LOAD: a clean -ENOSPC, nothing sent. */
static void program_pool_records_no_room_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->t.pool_used = f->t.pool.size - ANE_H16_RECORDS * ANE_H16_RECORD_SIZE + 1;
	KUNIT_EXPECT_EQ(test, fw_load_program(f, &p), -ENOSPC);
	KUNIT_EXPECT_EQ(test, f->count, 0U);
	KUNIT_EXPECT_EQ(test, f->t.placed_size, (size_t)0);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	/* Once the arena's end came back, the records fit. */
	f->t.pool_used = SZ_16K;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_EXPECT_EQ(test, f->t.placed, (size_t)SZ_16K);
	KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	ane_h16_session_put(&f->s, p);
}

/* Placed commands stay inside the placed range; releases stay above it. */
static void program_pool_placed_bounds_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	u8 command[ANE_H16_UNLOAD_SIZE] = {};
	size_t end;

	KUNIT_ASSERT_NOT_NULL(test, f);
	put_unaligned_le16(ANE_H16_UNLOAD_PROGRAM, command + 4);
	/* Nothing is placed before the range exists. */
	KUNIT_EXPECT_EQ(test, ane_fw_transport_publish_at(&f->t, ANE_FW_IO, command,
							  sizeof(command), U64_MAX, SZ_16K),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_reserve(&f->t, 100), -EINVAL);
	KUNIT_ASSERT_EQ(test, ane_fw_transport_reserve(&f->t, SZ_4K), 0);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_reserve(&f->t, SZ_4K), -EINVAL);
	end = f->t.placed + f->t.placed_size;
	KUNIT_EXPECT_EQ(test, ane_fw_transport_mark(&f->t), end);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_publish_at(&f->t, ANE_FW_IO, command,
							  sizeof(command), U64_MAX,
							  f->t.placed + 8),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_publish_at(&f->t, ANE_FW_IO, command,
							  sizeof(command), U64_MAX,
							  f->t.placed - ANE_FW_SLOT_SIZE),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_publish_at(&f->t, ANE_FW_IO, command,
							  sizeof(command), U64_MAX, end),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_publish_at(&f->t, ANE_FW_IO, command,
							  ANE_FW_SLOT_SIZE + 8, U64_MAX,
							  end - ANE_FW_SLOT_SIZE),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, f->count, 0U);
	/* The last slot of the range is fine, and leaves the arena's end alone. */
	KUNIT_EXPECT_EQ(test, ane_fw_transport_publish_at(&f->t, ANE_FW_IO, command,
							  sizeof(command), U64_MAX,
							  end - ANE_FW_SLOT_SIZE),
			0);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_wait(&f->t, ANE_FW_IO), 0);
	KUNIT_EXPECT_EQ(test, f->count, 1U);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_mark(&f->t), end);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_release(&f->t, end - 1), -EINVAL);
	KUNIT_EXPECT_EQ(test, ane_fw_transport_release(&f->t, end), 0);
}

/*
 * After a cold boot the replayed programs take records again from the
 * start; a program closed while asleep, lost with the old firmware, does
 * not free the record a replayed program now holds.
 */
static void program_pool_records_restart_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *a = NULL, *b = NULL;
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);
	unsigned int replayed;
	u64 first;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.lock = lock;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &a), 0);
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &b), 0);
	KUNIT_EXPECT_EQ(test, a->record, 0);
	KUNIT_EXPECT_EQ(test, b->record, 1);
	ane_h16_session_sleep(&f->s, true);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, a), -EINPROGRESS);
	fw_cold_boot(f);
	ane_h16_session_restart(&f->s);
	KUNIT_EXPECT_EQ(test, fw_records(f), 0U);
	KUNIT_ASSERT_EQ(test, ane_h16_session_replay(&f->s, &replayed, &first), 0);
	KUNIT_EXPECT_EQ(test, replayed, 1U);
	KUNIT_EXPECT_EQ(test, b->record, 0);
	/* The deferred close of the lost program runs here. */
	ane_h16_session_sleep(&f->s, false);
	KUNIT_EXPECT_TRUE(test, test_bit(0, f->s.records));
	KUNIT_EXPECT_EQ(test, fw_records(f), 1U);
	mutex_lock(lock);
	KUNIT_EXPECT_EQ(test, ane_h16_session_execute(&f->s, b), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_session_close(&f->s, b), 0);
	mutex_unlock(lock);
	ane_h16_session_put(&f->s, b);
	KUNIT_EXPECT_EQ(test, fw_records(f), 0U);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

/*
 * Memory a program of another shape gave back: no later program of these
 * tests maps a surface of this size, so only freeing it makes room.
 */
static bool fw_foreign_spare(struct prog_fw *f, size_t size)
{
	struct ane_fw_surface surface;

	if (f->s.spare_count == ANE_H16_SPARES || fw_map(f, size, false, &surface))
		return false;
	f->s.spares[f->s.spare_count++] = (struct ane_h16_spare) { .surface = surface };
	f->s.spare_bytes += size;
	return true;
}

/* Larger than any region or section of the fixture programs. */
#define FW_FOREIGN(i) (SZ_16K * (9 + (i) % 3))
#define FW_WINDOW SZ_16M

/*
 * Programs of ever new shapes leave spares that no later program can use.
 * Without ops->unmap a spare table full of them makes every close leave its
 * memory mapped and unused, the DVA window fills, and every LOAD fails with
 * -ENOSPC from then on, until reset. With it, the session frees the oldest
 * spares beyond its limits: thousands of programs load into a window that
 * holds a few dozen, with a new shape left behind before each.
 */
static void program_unmap_window_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test), *g = fw_create(test);
	unsigned int i;
	int ret = 0;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, g);
	f->window = FW_WINDOW;
	for (i = 0; i < ANE_H16_SPARES; i++)
		KUNIT_ASSERT_TRUE(test, fw_foreign_spare(f, FW_FOREIGN(i)));
	for (i = 0; i < 2000 && !ret; i++)
		ret = fw_cycle(f);
	KUNIT_EXPECT_EQ(test, ret, -ENOSPC);
	KUNIT_EXPECT_LT(test, i, 2000U);
	KUNIT_EXPECT_EQ(test, f->unmaps, 0U);

	g->s.ops = &fw_session_ops_unmap;
	g->window = FW_WINDOW;
	for (i = 0; i < ANE_H16_SPARES; i++)
		KUNIT_ASSERT_TRUE(test, fw_foreign_spare(g, FW_FOREIGN(i)));
	for (i = 0; i < 2000; i++) {
		fw_foreign_spare(g, FW_FOREIGN(i));
		KUNIT_ASSERT_EQ(test, fw_cycle(g), 0);
		KUNIT_ASSERT_LE(test, g->s.spare_count, ANE_H16_SPARES_KEEP);
	}
	KUNIT_EXPECT_GE(test, g->unmaps, 2000U);
	KUNIT_EXPECT_FALSE(test, g->s.quarantined);
	KUNIT_EXPECT_PTR_EQ(test, g->s.program, NULL);
	/* What the spares hold is all that stays mapped. */
	KUNIT_EXPECT_EQ(test, g->live, (u64)g->s.spare_bytes);
}

/* A mapping that finds no room frees every spare and tries again. */
static void program_unmap_pressure_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->s.ops = &fw_session_ops_unmap;
	f->window = SZ_2M;
	for (i = 0; fw_foreign_spare(f, FW_FOREIGN(i)); i++)
		;
	/* Not even the smallest mapping fits beside them. */
	f->window = f->live;
	KUNIT_ASSERT_GT(test, i, 0U);
	KUNIT_ASSERT_LT(test, i, ANE_H16_SPARES_KEEP);
	KUNIT_ASSERT_EQ(test, fw_cycle(f), 0);
	KUNIT_EXPECT_EQ(test, f->unmaps, i);
	KUNIT_EXPECT_EQ(test, f->live, (u64)f->s.spare_bytes);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
}

static u8 fw_dummy[SZ_16K];

/* The spares stay within their byte limit too, freeing the oldest first. */
static void program_unmap_bytes_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	const size_t half = ANE_H16_SPARE_BYTES / 2;
	u64 newest = 0;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->s.ops = &fw_session_ops_unmap;
	/* Large spares with no memory behind them; never taken, only freed. */
	for (i = 0; i < 3; i++) {
		newest = f->next_dva;
		f->s.spares[f->s.spare_count++] = (struct ane_h16_spare) {
			.surface = { .cpu = fw_dummy, .dva = newest, .size = half },
		};
		f->s.spare_bytes += half;
		f->next_dva += half;
		f->live += half;
	}
	KUNIT_ASSERT_EQ(test, fw_cycle(f), 0);
	KUNIT_EXPECT_LE(test, f->s.spare_bytes, (size_t)ANE_H16_SPARE_BYTES);
	KUNIT_EXPECT_EQ(test, f->live, (u64)f->s.spare_bytes);
	KUNIT_EXPECT_EQ(test, f->unmaps, 2U);
	for (i = 0; i < f->s.spare_count; i++)
		if (f->s.spares[i].surface.size == half)
			KUNIT_EXPECT_EQ(test, f->s.spares[i].surface.dva, newest);
}

/*
 * Nothing is freed while the engine is parked: the DART invalidation of an
 * unmap could not reach it. The spares keep the memory until the next LOAD,
 * which runs with the engine up.
 */
static void program_unmap_parked_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->s.ops = &fw_session_ops_unmap;
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	for (i = 0; i < ANE_H16_SPARES_KEEP + 8; i++)
		KUNIT_ASSERT_TRUE(test, fw_foreign_spare(f, SZ_16K * 3));
	f->s.parked = true;
	KUNIT_ASSERT_EQ(test, ane_h16_session_close(&f->s, p), 0);
	ane_h16_session_put(&f->s, p);
	KUNIT_EXPECT_EQ(test, f->unmaps, 0U);
	KUNIT_EXPECT_GT(test, f->s.spare_count, ANE_H16_SPARES_KEEP);
	f->s.parked = false;
	KUNIT_ASSERT_EQ(test, fw_cycle(f), 0);
	KUNIT_EXPECT_GT(test, f->unmaps, 0U);
	KUNIT_EXPECT_LE(test, f->s.spare_count, ANE_H16_SPARES_KEEP);
}

/*
 * A LOAD that fails before the firmware saw the program (here, no memory
 * for its third mapping) gives back what it mapped and frees the program.
 */
static void program_unmap_failed_map_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->s.ops = &fw_session_ops_unmap;
	f->map_fail_at = 3;
	KUNIT_EXPECT_EQ(test, fw_load_program(f, &p), -ENOMEM);
	KUNIT_EXPECT_PTR_EQ(test, p, NULL);
	KUNIT_EXPECT_PTR_EQ(test, f->s.program, NULL);
	KUNIT_EXPECT_EQ(test, f->count, 0U);
	KUNIT_EXPECT_FALSE(test, f->s.quarantined);
	KUNIT_EXPECT_EQ(test, f->s.spare_count, 2U);
	KUNIT_EXPECT_EQ(test, f->live, (u64)f->s.spare_bytes);
	/* The next LOAD takes them. */
	KUNIT_ASSERT_EQ(test, fw_load_program(f, &p), 0);
	KUNIT_EXPECT_EQ(test, f->s.spare_count, 0U);
}

/*
 * A LOAD the firmware refused quarantines the session; the program is held
 * with those closed while quarantined, and the verified power cycle of the
 * recovery gives its memory back.
 */
static void program_unmap_refused_load_test(struct kunit *test)
{
	struct prog_fw *f = fw_create(test);
	struct ane_h16_program *p = NULL;
	struct mutex *lock = kunit_kzalloc(test, sizeof(*lock), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, lock);
	mutex_init(lock);
	f->s.ops = &fw_session_ops_unmap;
	f->mode = FW_LOAD_NAK;
	KUNIT_EXPECT_NE(test, fw_load_program(f, &p), 0);
	KUNIT_EXPECT_PTR_EQ(test, p, NULL);
	KUNIT_EXPECT_TRUE(test, f->s.quarantined);
	KUNIT_EXPECT_EQ(test, f->hung, 1U);
	KUNIT_EXPECT_FALSE(test, list_empty(&f->s.held));
	KUNIT_EXPECT_EQ(test, f->s.spare_count, 0U);

	f->s.lock = lock;
	ane_h16_session_sleep(&f->s, true);
	fw_cold_boot(f);
	/* The backend's cold boot starts a new transport. */
	memset(f->head, 0, sizeof(f->head));
	KUNIT_ASSERT_EQ(test, ane_fw_transport_init(&f->t, &f->soc, &fw_ops, f, f->ipc, f->pool,
						    SZ_16K, f->ipc.dva, ANE_FW_TABLE_SIZE), 0);
	KUNIT_ASSERT_EQ(test, ane_fw_transport_arm(&f->t), 0);
	f->mode = FW_OK;
	ane_h16_session_restart(&f->s);
	ane_h16_session_recovered(&f->s);
	ane_h16_session_sleep(&f->s, false);
	KUNIT_EXPECT_TRUE(test, list_empty(&f->s.held));
	KUNIT_EXPECT_PTR_EQ(test, f->s.program, NULL);
	KUNIT_EXPECT_GT(test, f->s.spare_count, 0U);
	KUNIT_EXPECT_EQ(test, f->live, (u64)f->s.spare_bytes);
	f->s.lock = NULL;
	KUNIT_EXPECT_EQ(test, fw_cycle(f), 0);
}

static struct kunit_case program_cases[] = {
	KUNIT_CASE_SLOW(program_pool_rewind_test),
	KUNIT_CASE_SLOW(program_pool_resident_test),
	KUNIT_CASE_SLOW(program_pool_records_reuse_test),
	KUNIT_CASE(program_pool_records_full_test),
	KUNIT_CASE(program_pool_records_no_room_test),
	KUNIT_CASE(program_pool_placed_bounds_test),
	KUNIT_CASE(program_pool_records_restart_test),
	KUNIT_CASE_SLOW(program_unmap_window_test),
	KUNIT_CASE(program_unmap_pressure_test),
	KUNIT_CASE(program_unmap_bytes_test),
	KUNIT_CASE(program_unmap_parked_test),
	KUNIT_CASE(program_unmap_failed_map_test),
	KUNIT_CASE(program_unmap_refused_load_test),
	KUNIT_CASE(program_hung_recovery_test),
	KUNIT_CASE(program_pipeline_out_of_order_test),
	KUNIT_CASE(program_pipeline_same_program_test),
	KUNIT_CASE(program_pipeline_bound_test),
	KUNIT_CASE(program_pipeline_arena_pressure_test),
	KUNIT_CASE(program_pipeline_surface_owner_test),
	KUNIT_CASE(program_pipeline_timeout_test),
	KUNIT_CASE(program_pipeline_quarantine_test),
	KUNIT_CASE(program_pipeline_sleep_test),
	KUNIT_CASE(program_sleep_replay_test),
	KUNIT_CASE(program_sleep_nonblock_test),
	KUNIT_CASE(program_sleep_replay_failure_test),
	KUNIT_CASE(program_sleep_replay_deferred_test),
	KUNIT_CASE(program_sleep_gate_test),
	KUNIT_CASE(program_sleep_keeps_program_test),
	KUNIT_CASE(program_sleep_restart_test),
	KUNIT_CASE(program_sleep_restart_deferred_test),
	KUNIT_CASE(program_cycle_test),
	KUNIT_CASE(program_h17_cycle_test),
	KUNIT_CASE(program_failed_close_keeps_test),
	KUNIT_CASE(program_busy_test),
	KUNIT_CASE(program_cached_sync_test),
	KUNIT_CASE(program_h17_no_custom_bar_test),
	KUNIT_CASE(program_add128_slots_test),
	KUNIT_CASE(program_kernel_slots_test),
	KUNIT_CASE(program_add128_hwx_test),
	KUNIT_CASE(program_replies_test),
	KUNIT_CASE(program_deny_list_test),
	KUNIT_CASE(program_custom_bar_test),
	KUNIT_CASE(program_session_test),
	KUNIT_CASE(program_gate_hold_test),
	KUNIT_CASE(program_scratch_test),
	KUNIT_CASE(program_bad_load_test),
	KUNIT_CASE(program_bad_operation_test),
	KUNIT_CASE(program_bad_report_test),
	KUNIT_CASE(program_stall_trace_test),
	KUNIT_CASE(program_timing_test),
	KUNIT_CASE(program_fwlog_line_test),
	KUNIT_CASE(program_shared_tag_test),
	KUNIT_CASE(program_pool_test),
	KUNIT_CASE(program_pool_recycle_test),
	KUNIT_CASE(program_export_test),
	KUNIT_CASE(program_h17_hwx_test),
	KUNIT_CASE(program_h17_sections_test),
	KUNIT_CASE(program_h17_mobilenet_test),
	KUNIT_CASE(program_h17_switch_test),
	{}
};

static struct kunit_suite program_suite = {
	.name = "ane-h16-program",
	.test_cases = program_cases,
};

kunit_test_suite(program_suite);
MODULE_DESCRIPTION("KUnit tests for the H16 program lifecycle");
MODULE_LICENSE("Dual MIT/GPL");
