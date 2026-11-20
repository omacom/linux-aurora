/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __ANE_H16_BOOT_H__
#define __ANE_H16_BOOT_H__

#include <linux/slab.h>

#include "ane_fw_platform.h"
#include "ane_fw_transport.h"

enum ane_h16_stage {
	ANE_H16_REFUSE,
	ANE_H16_PREFLIGHT,
	ANE_H16_MAP,
	ANE_H16_BOOT,
	ANE_H16_STARTUP,
	ANE_H16_PROGRAM,
};

enum ane_h16_window {
	ANE_H16_ENGINE,
	ANE_H16_PMGR,
	ANE_H16_XL,
	ANE_H16_DART0,
	ANE_H16_DART1,
	ANE_H16_DART2,
	/* The one DAPF bank: the DART reg entry the ADT tags FPAD. */
	ANE_H16_DAPF,
	ANE_H16_WINDOWS,
};

/*
 * Every DART and DAPF register offset the stages read or write, for one DART
 * revision: PARAMS3 bits 15:0 of the instance that owns the DAPF bank. The
 * platform lists one entry per supported revision. PARAMS3 is read before an
 * entry is chosen; with no entry nothing else in the DARTs or the bank is
 * touched, and an entry names every other offset that is.
 */
struct ane_h16_mmio {
	u16 dart_revision;
	u32 dart_fault;
	u32 dart_address_lo;
	u32 dart_address_hi;
	u32 dart_protect;
	u32 dart_lock;
	u32 dapf_records;
	u32 dapf_stride;
	/* Record end bits that read back; the bounds are inclusive. */
	u64 dapf_end_mask;
	/* Record words other than the bounds, as the ADT dapf-instance gives them. */
	u8 dapf_r0;
	u32 dapf_r4;
	u16 dapf_r20;
};

struct ane_h16_tunable {
	u32 offset;
	u32 mask;
	u32 value;
};

struct ane_h16_tunables {
	enum ane_h16_window window;
	u32 base;
	const struct ane_h16_tunable *values;
	size_t count;
	bool c1ppt;
	u32 lock_offset;
	u32 lock_mask;
};

/* Selected only after the immutable resident firmware identity passes. */
struct ane_h16_build {
	u32 cpu_control;
	u32 rvbar;
	u32 ce;
	u32 ce_delays;
	u32 ce_delay_us;
	u32 ready;
	u32 run;
	u32 scratch_polls;
	u32 scratch_delay_us;
	u64 command_timeout_ns;
	u64 pool_dva;
	u64 ipc_dva;
	u64 heap_dva;
	u64 shared_dva;
	u64 expected_rvbar;
	u32 pool_size;
	u32 init_offset;
	u32 init_size;
	u32 config_span;
	u32 config_window;
	u32 template_count;
	u32 template_word;
	u32 template_value;
	u32 max_channels;
	u32 max_heap;
	bool native_free;
	bool c1ppt;
	const struct ane_h16_tunables *tunables;
	size_t tunable_count;
	struct {
		u16 id;
		u16 size;
	} startup[4];
};

/*
 * One supported firmware build: how to recognize its iBoot preload and the
 * values to boot it with. Builds that differ only in their preload share one
 * ane_h16_build. The platform lists every profile; the first whose preload
 * rule matches the live preload is used.
 */
struct ane_h16_preload_rule;
struct ane_h16_profile {
	const char *identity;
	const struct ane_h16_preload_rule *preload;
	const struct ane_h16_build *build;
};

struct ane_h16_boot;
struct ane_h16_boot_ops {
	u32 (*read32)(void *ctx, enum ane_h16_window window, u32 offset);
	u64 (*read64)(void *ctx, enum ane_h16_window window, u32 offset);
	void (*write32)(void *ctx, enum ane_h16_window window, u32 offset, u32 value);
	void (*write64)(void *ctx, enum ane_h16_window window, u32 offset, u64 value);
	void (*delay_us)(void *ctx, unsigned int us);
	int (*map)(void *ctx);
	int (*alloc_at)(void *ctx, u64 dva, size_t size, struct ane_fw_surface *s);
	int (*pin)(void *ctx);
	void (*mailbox)(void *ctx);
	void (*log)(void *ctx, const char *item, u64 a, u64 b);
	void (*quarantine)(void *ctx, int error);
	/*
	 * Optional. After a startup timeout: power the engine off and on again,
	 * verify that every domain went off, and restore the preload so the CPU
	 * can be released from the same entry. 0 only for a verified reset.
	 */
	int (*restart)(void *ctx);
	/* Optional. False while the engine's domains are off: no MMIO then. */
	bool (*powered)(void *ctx);
};

/*
 * One caller. Stages advance in order, each at most once, and a failure ends
 * the attempt. No mapping is released here, even on success.
 */
struct ane_h16_boot {
	const struct ane_fw_platform *soc;
	const struct ane_h16_build *build;
	/* Chosen from the DART revision in preflight. */
	const struct ane_h16_mmio *mmio;
	const struct ane_h16_boot_ops *ops;
	const struct ane_fw_transport_ops *transport_ops;
	void *ctx;
	struct ane_fw_transport transport;
	struct ane_fw_surface pool;
	struct ane_fw_surface ipc;
	struct ane_fw_surface heap;
	u64 ranges[ANE_FW_DAPF_MAX][2];
	u32 requests[5];
	u32 tick;
	/* The PMU aperture base for SET_SNE_PMU_BASE2 at startup; 0: not sent. */
	u64 pmu_base;
	bool pmu_base_set;
	u8 resource_info[100];
	int error;
	enum ane_h16_stage stage;
	bool attempted;
	bool failed;
	bool ran;
	bool quarantined;
	/* Inside ane_h16_startup(): the CPU must keep running until it answers. */
	bool starting;
	/* When the CPU was last released, for the startup evidence. */
	u64 run_ns;
	/* Startup timeouts answered with a verified restart, and the budget. */
	unsigned int restarts;
	unsigned int max_restarts;
};

static inline int ane_h16_stage_parse(const char *name)
{
	if (!name || !*name)
		return ANE_H16_REFUSE;
	if (!strcmp(name, "preflight"))
		return ANE_H16_PREFLIGHT;
	if (!strcmp(name, "map"))
		return ANE_H16_MAP;
	if (!strcmp(name, "boot"))
		return ANE_H16_BOOT;
	if (!strcmp(name, "startup"))
		return ANE_H16_STARTUP;
	if (!strcmp(name, "program"))
		return ANE_H16_PROGRAM;
	return -EINVAL;
}

static inline u32 ane_h16_read(struct ane_h16_boot *b, u32 cell)
{
	return b->ops->read32(b->ctx, ANE_H16_ENGINE, b->soc->scratch + cell * 4);
}

static inline void ane_h16_write(struct ane_h16_boot *b, u32 cell, u32 value)
{
	b->ops->write32(b->ctx, ANE_H16_ENGINE, b->soc->scratch + cell * 4, value);
}

static inline int ane_h16_faults(struct ane_h16_boot *b, bool log)
{
	const struct ane_h16_mmio *m = b->mmio;
	unsigned int i;
	int ret = 0;

	if (!m)
		return 0;
	for (i = 0; i < 3; i++) {
		enum ane_h16_window w = ANE_H16_DART0 + i;
		u32 status = b->ops->read32(b->ctx, w, m->dart_fault);

		if (log || (status & BIT(31))) {
			u64 address = b->ops->read32(b->ctx, w, m->dart_address_lo);

			address |= (u64)b->ops->read32(b->ctx, w, m->dart_address_hi) << 32;
			b->ops->log(b->ctx, "DART status", i, status);
			b->ops->log(b->ctx, "DART address", i, address);
		}
		if (status & BIT(31))
			ret = -EIO;
	}
	return ret;
}

static inline int ane_h16_fail(struct ane_h16_boot *b, int error)
{
	unsigned int i;

	if (!b->error)
		b->error = error;
	if (b->ran && !b->quarantined) {
		b->quarantined = true;
		/* A failed restart can leave the engine off: then no register reads. */
		if (!b->ops->powered || b->ops->powered(b->ctx)) {
			for (i = 0; i < 8; i++)
				b->ops->log(b->ctx, "quarantine scratch", i, ane_h16_read(b, i));
			ane_h16_faults(b, true);
		}
		b->ops->quarantine(b->ctx, b->error);
	}
	return b->error;
}

/*
 * PARAMS3 of the three instances, logged; the DAPF owner's revision selects
 * the register table. The other two are only reported.
 */
static inline int ane_h16_select_mmio(struct ane_h16_boot *b)
{
	const struct ane_fw_platform *soc = b->soc;
	u32 params[3];
	unsigned int i;

	for (i = 0; i < 3; i++) {
		params[i] = b->ops->read32(b->ctx, ANE_H16_DART0 + i, soc->dart_params);
		b->ops->log(b->ctx, "DART PARAMS3", i, params[i]);
	}
	for (i = 0; i < soc->h16_mmio_count; i++) {
		if (soc->h16_mmio[i].dart_revision == (params[soc->dapf_dart] & 0xffff)) {
			b->mmio = &soc->h16_mmio[i];
			return 0;
		}
	}
	b->ops->log(b->ctx, "DART revision has no register table", params[soc->dapf_dart] & 0xffff,
		    soc->h16_mmio_count);
	return -ENODEV;
}

/* TTBR/TCR protection is bit 0, separate from the DAPF lock at +0x208. */
static inline int ane_h16_dart_protection(struct ane_h16_boot *b, bool require_unlocked)
{
	unsigned int i;
	u32 protected = 0;

	if (!b->mmio)
		return -ENODEV;
	for (i = 0; i < 3; i++) {
		u32 value = b->ops->read32(b->ctx, ANE_H16_DART0 + i,
					 b->mmio->dart_protect);

		b->ops->log(b->ctx, require_unlocked ? "DART map protect" :
			   "DART preflight protect", i, value);
		protected |= value & BIT(0);
	}
	return require_unlocked && protected ? -EPERM : 0;
}

static inline int ane_h16_preflight(struct ane_h16_boot *b)
{
	const struct ane_h16_build *f = b->build;
	unsigned int pass, i;
	u64 rvbar;
	u32 control;
	int ret;

	/* All suppliers are raised by genpd before entry. Never write PMGR. */
	for (pass = 0; pass < 2; pass++) {
		for (i = 0; i < b->soc->power_count; i++) {
			u32 word = b->ops->read32(b->ctx, ANE_H16_PMGR,
						b->soc->power_words[i]);

			b->ops->log(b->ctx, "power ACTUAL", i, word);
			if ((word & 0xff) != 0xff)
				return -EHOSTDOWN;
		}
	}
	control = b->ops->read32(b->ctx, ANE_H16_ENGINE, f->cpu_control);
	rvbar = b->ops->read64(b->ctx, ANE_H16_ENGINE, f->rvbar);
	b->ops->log(b->ctx, "CPU control/RVBAR", control, rvbar);
	if (control || rvbar != f->expected_rvbar)
		return -EBUSY;
	ret = ane_h16_select_mmio(b);
	if (ret)
		return ret;
	ret = ane_h16_dart_protection(b, false);
	if (ret)
		return ret;
	ret = ane_h16_faults(b, true);
	if (ret)
		return ret;
	return 0;
}

/*
 * One record of the DAPF bank, read only. Record words as the ADT
 * dapf-instance: r0 (+0x00, only its low byte reads back), r4, the inclusive
 * start and end, r20 (+0x20, its low half is the enable).
 */
static inline bool ane_h16_dapf_record(struct ane_h16_boot *b, unsigned int record)
{
	const struct ane_h16_mmio *m = b->mmio;
	u32 off = m->dapf_records + record * m->dapf_stride;
	u32 flags = b->ops->read32(b->ctx, ANE_H16_DAPF, off);
	u32 r4 = b->ops->read32(b->ctx, ANE_H16_DAPF, off + 4);
	u64 start = b->ops->read64(b->ctx, ANE_H16_DAPF, off + 8);
	u64 end = b->ops->read64(b->ctx, ANE_H16_DAPF, off + 0x10);
	u32 enabled = b->ops->read32(b->ctx, ANE_H16_DAPF, off + 0x20);

	b->ops->log(b->ctx, "DAPF record/r4", record, r4);
	b->ops->log(b->ctx, "DAPF flags/enable", flags, enabled);
	b->ops->log(b->ctx, "DAPF start/end", start, end);
	return (flags & 0xff) == m->dapf_r0 && r4 == m->dapf_r4 &&
	       (enabled & 0xffff) == m->dapf_r20 && start == b->ranges[record][0] &&
	       (end & m->dapf_end_mask) == (b->ranges[record][1] & m->dapf_end_mask);
}

/*
 * Compare the platform's records with apple,dapf-ranges. Preflight only
 * reports.
 * The bank loses its records whenever its power domain does, so map writes
 * every record that differs, in the order the t8110 bank takes them, and
 * reads it back. A bank whose lock word has bit 2 set and that differs is
 * refused before any write.
 */
static inline int ane_h16_dapf(struct ane_h16_boot *b, bool program)
{
	const struct ane_h16_mmio *m = b->mmio;
	enum ane_h16_window owner = ANE_H16_DART0 + b->soc->dapf_dart;
	u32 protect = b->ops->read32(b->ctx, owner, m->dart_protect);
	u32 lock = b->ops->read32(b->ctx, owner, m->dart_lock);
	unsigned int record, matched = 0, count = b->soc->dapf_count;
	bool match[ANE_FW_DAPF_MAX];

	b->ops->log(b->ctx, "DAPF protect/lock", protect, lock);
	for (record = 0; record < count; record++) {
		match[record] = ane_h16_dapf_record(b, record);
		matched += match[record];
	}
	b->ops->log(b->ctx, "DAPF records matching", matched, count);
	if (!program || matched == count)
		return 0;
	if (lock & BIT(2))
		return -EPERM;
	for (record = 0; record < count; record++) {
		u32 off = m->dapf_records + record * m->dapf_stride;

		if (match[record])
			continue;
		b->ops->write32(b->ctx, ANE_H16_DAPF, off + 4, m->dapf_r4);
		b->ops->write64(b->ctx, ANE_H16_DAPF, off + 8, b->ranges[record][0]);
		b->ops->write64(b->ctx, ANE_H16_DAPF, off + 0x10, b->ranges[record][1]);
		b->ops->write32(b->ctx, ANE_H16_DAPF, off, m->dapf_r0);
		b->ops->write32(b->ctx, ANE_H16_DAPF, off + 0x20, m->dapf_r20);
		if (!ane_h16_dapf_record(b, record))
			return -EIO;
	}
	b->ops->log(b->ctx, "DAPF records written/read back", count - matched, count);
	return 0;
}

/* r5 section 1.1. H16 has CE, with no H17 PWGATE pair. */
static inline int ane_h16_ce(struct ane_h16_boot *b)
{
	const struct ane_h16_build *f = b->build;
	u64 state = b->ops->read64(b->ctx, ANE_H16_ENGINE, f->ce + 0x10);
	u64 error = b->ops->read64(b->ctx, ANE_H16_ENGINE, f->ce + 0x18);
	u64 wanted = (state & 3) ^ 3;
	unsigned int i;

	b->ops->log(b->ctx, "CE state/error", state, error);
	if (error & wanted)
		return -EIO;
	if ((state & 3) == 3)
		return 0;
	b->ops->write64(b->ctx, ANE_H16_ENGINE, f->ce, wanted);
	for (i = 0; i <= f->ce_delays; i++) {
		if ((b->ops->read64(b->ctx, ANE_H16_ENGINE, f->ce + 8) & 3) == wanted)
			break;
		if (i == f->ce_delays)
			return -ETIMEDOUT;
		b->ops->delay_us(b->ctx, f->ce_delay_us);
	}
	error = b->ops->read64(b->ctx, ANE_H16_ENGINE, f->ce + 0x18);
	if (error & wanted)
		return -EIO;
	b->ops->write64(b->ctx, ANE_H16_ENGINE, f->ce + 8, wanted);
	b->ops->log(b->ctx, "CE ack bits/delays", wanted, i);
	return 0;
}

static inline int ane_h16_prepare(struct ane_h16_boot *b)
{
	const struct ane_h16_build *f = b->build;
	unsigned int i, j;
	int ret = ane_h16_ce(b);

	if (ret)
		return ret;
	for (i = 0; i < f->tunable_count; i++) {
		const struct ane_h16_tunables *group = &f->tunables[i];

		if (group->c1ppt && !f->c1ppt)
			continue;
		if (group->window == ANE_H16_XL && !b->soc->has_xl) {
			b->ops->log(b->ctx, "tunables skipped: no XL window", group->window,
				    group->count);
			continue;
		}
		if (group->lock_mask &&
		    (b->ops->read32(b->ctx, group->window, group->base +
				    group->lock_offset) & group->lock_mask)) {
			for (j = 0; j < group->count; j++) {
				const struct ane_h16_tunable *v = &group->values[j];
				u32 value = b->ops->read32(b->ctx, group->window,
							group->base + v->offset);

				if ((value & v->mask) != (v->value & v->mask)) {
					b->ops->log(b->ctx, "locked tunable offset/value",
						    group->base + v->offset, value);
					return -EPERM;
				}
			}
			b->ops->log(b->ctx, "tunables locked match", group->window, group->count);
			continue;
		}
		for (j = 0; j < group->count; j++) {
			const struct ane_h16_tunable *v = &group->values[j];
			u32 off = group->base + v->offset;
			u32 old = b->ops->read32(b->ctx, group->window, off);

			b->ops->write32(b->ctx, group->window, off,
					(old & ~v->mask) | (v->value & v->mask));
		}
		b->ops->log(b->ctx, "tunables applied", group->window, group->count);
	}
	return 0;
}

static inline int ane_h16_wait(struct ane_h16_boot *b, u32 cell, u32 value,
			       bool table)
{
	const struct ane_h16_build *f = b->build;
	unsigned int i;
	int ret;
	u64 deadline = b->transport_ops->now_ns(b->ctx) +
		       (u64)f->scratch_polls * f->scratch_delay_us * 1000;

	for (i = 0; i < f->scratch_polls; i++) {
		if (b->transport_ops->now_ns(b->ctx) >= deadline)
			break;
		b->ops->mailbox(b->ctx);
		ret = ane_h16_faults(b, false);
		if (ret)
			return ret;
		if (ane_h16_read(b, cell) == value) {
			u64 address;

			if (!table)
				return 0;
			address = ane_h16_read(b, 0);
			address |= (u64)ane_h16_read(b, 1) << 32;
			if (address == f->ipc_dva)
				return 0;
			/* READY can pre-exist publication. Never accept the init DVA. */
			if (address != f->pool_dva + f->init_offset)
				return -EPROTO;
		}
		b->ops->delay_us(b->ctx, f->scratch_delay_us);
	}
	return -ETIMEDOUT;
}

static inline int ane_h16_alloc(struct ane_h16_boot *b, u64 dva, size_t size,
				struct ane_fw_surface *s)
{
	int ret;

	/* A cold boot after sleep reuses the surface it mapped before, cleared. */
	if (s->cpu && s->dva == dva && s->size == size) {
		memset(s->cpu, 0, size);
		return 0;
	}
	ret = b->ops->alloc_at(b->ctx, dva, size, s);

	if (ret)
		return ret;
	if (!ane_fw_surface_valid(s) || s->dva != dva || s->size != size)
		return -EPROTO;
	return 0;
}

static inline int ane_h16_scratch_boot(struct ane_h16_boot *b)
{
	const struct ane_h16_build *f = b->build;
	u64 ipc_bytes, heap_bytes;
	u8 *p;
	unsigned int i;
	int ret;

	ret = ane_h16_alloc(b, f->pool_dva, f->pool_size, &b->pool);
	if (ret)
		return ret;
	ret = ane_h16_prepare(b);
	if (ret)
		return ret;
	for (i = 0; i < 8; i++)
		ane_h16_write(b, i, 0);
	/* Endpoint-1 management client is installed, never awaited. */
	b->ops->mailbox(b->ctx);
	ane_h16_write(b, 6, 1);
	ane_h16_write(b, 7, 0);
	if (b->ops->read64(b->ctx, ANE_H16_ENGINE, f->rvbar) != f->expected_rvbar)
		return -EBUSY;
	ret = b->ops->pin(b->ctx);
	if (ret)
		return ret;
	b->ops->write32(b->ctx, ANE_H16_ENGINE, f->cpu_control, 0);
	/* Latch before the first RUN store, including a failure at that store. */
	b->ran = true;
	b->ops->write32(b->ctx, ANE_H16_ENGINE, f->cpu_control, f->run);
	b->run_ns = b->transport_ops->now_ns(b->ctx);
	ret = ane_h16_wait(b, 7, f->ready, false);
	if (ret)
		return ret;
	b->ops->log(b->ctx, "READY", f->ready, 0);
	for (i = 0; i < ARRAY_SIZE(b->requests); i++)
		b->requests[i] = ane_h16_read(b, i);
	b->ops->log(b->ctx, "requests channels/IPC-minus-one", b->requests[0], b->requests[1]);
	b->ops->log(b->ctx, "requests heap/mode", b->requests[3], b->requests[4]);
	ipc_bytes = (u64)b->requests[1] + 1;
	heap_bytes = b->requests[3];
	/* Host policy. IPC cannot reach the fixed heap; no guessed firmware ceiling. */
	if (!b->requests[0] || b->requests[0] >= f->max_channels ||
	    ipc_bytes < ANE_FW_TABLE_SIZE || ipc_bytes > f->heap_dva - f->ipc_dva ||
	    heap_bytes > f->max_heap ||
	    ALIGN(heap_bytes, SZ_16K) > f->shared_dva - f->heap_dva ||
	    b->requests[4] > 1)
		return -E2BIG;
	ret = ane_h16_alloc(b, f->ipc_dva, ALIGN(ipc_bytes, SZ_16K), &b->ipc);
	if (ret)
		return ret;
	if (heap_bytes) {
		ret = ane_h16_alloc(b, f->heap_dva, ALIGN(heap_bytes, SZ_16K), &b->heap);
		if (ret)
			return ret;
	}
	p = (u8 *)b->pool.cpu + f->init_offset;
	memset(p, 0, f->init_size);
	put_unaligned_le64(f->ipc_dva, p + 8);
	put_unaligned_le64(f->config_span, p + 0x10);
	put_unaligned_le64(f->config_window - f->config_span, p + 0x18);
	put_unaligned_le64(heap_bytes ? f->heap_dva : 0, p + 0x20);
	put_unaligned_le64(heap_bytes, p + 0x28);
	put_unaligned_le64(ipc_bytes, p + 0x50);
	put_unaligned_le64(f->pool_dva, p + 0x58);
	put_unaligned_le64(f->pool_size, p + 0x60);
	put_unaligned_le32(f->template_count, p + 0x68);
	put_unaligned_le32(f->template_value, p + f->template_word);
	b->transport_ops->write_barrier(b->ctx);
	ane_h16_write(b, 0, lower_32_bits(f->pool_dva + f->init_offset));
	ane_h16_write(b, 1, upper_32_bits(f->pool_dva + f->init_offset));
	ane_h16_write(b, 7, ~f->ready);
	ret = ane_h16_wait(b, 7, f->ready, true);
	if (ret)
		return ret;
	ret = ane_fw_transport_init(&b->transport, b->soc, b->transport_ops, b->ctx,
				    b->ipc, b->pool, f->init_offset + f->init_size,
				    f->ipc_dva, ANE_FW_TABLE_SIZE);
	if (ret)
		return ret;
	b->transport.native_free = f->native_free;
	b->ops->log(b->ctx, "table validated", f->ipc_dva, ANE_FW_CHANNELS);
	ret = ane_fw_transport_arm(&b->transport);
	if (ret)
		return ret;
	ane_h16_write(b, 3, f->ready);
	ret = ane_h16_wait(b, 3, 0, false);
	if (!ret)
		b->ops->log(b->ctx, "scratch 3 cleared", 0, 0);
	return ret;
}

/*
 * Read-only evidence: never acknowledge an IRQ or change a cursor on timeout
 * or when the CPU stopped.
 */
static inline void ane_h16_startup_timeout(struct ane_h16_boot *b)
{
	struct ane_fw_transport *t = &b->transport;
	const struct ane_fw_channel *c = &t->channels[ANE_FW_SHAREDMALLOC];
	const struct ane_h16_build *f = b->build;
	unsigned int i;
	u8 *p;

	b->ops->log(b->ctx, "startup CPU control/status",
		    b->ops->read32(b->ctx, ANE_H16_ENGINE, f->cpu_control),
		    b->ops->read32(b->ctx, ANE_H16_ENGINE, f->cpu_control + 4));
	b->ops->log(b->ctx, "startup us since RUN/error",
		    div_u64(t->ops->now_ns(t->ctx) - b->run_ns, NSEC_PER_USEC), -t->error);
	b->ops->log(b->ctx, "startup timeout IRQ/polls",
		    t->ops->read32(t->ctx, t->soc->irq_status), t->polls);
	b->ops->log(b->ctx, "startup SHAREDMALLOC head/slots",
		    t->head[ANE_FW_SHAREDMALLOC], c->slots);
	b->ops->log(b->ctx, "startup SHAREDMALLOC ready/allocations",
		    t->shared_ready, t->shared_count);
	/* Geometry was validated before startup; include slots behind the cursor. */
	for (i = 0; i < c->slots; i++) {
		p = (u8 *)t->ipc.cpu + c->offset + i * ANE_FW_SLOT_SIZE;
		t->ops->read_barrier(t->ctx);
		b->ops->log(b->ctx, "startup SHAREDMALLOC slot/word0", i,
			    t->ops->load64(t->ctx, p));
		b->ops->log(b->ctx, "startup SHAREDMALLOC word1/word2",
			    t->ops->load64(t->ctx, p + 8),
			    t->ops->load64(t->ctx, p + 16));
	}
	p = ane_fw_transport_slot(t, ANE_FW_IO);
	t->ops->read_barrier(t->ctx);
	b->ops->log(b->ctx, "startup IO head/word0", t->head[ANE_FW_IO],
		    t->ops->load64(t->ctx, p));
	b->ops->log(b->ctx, "startup IO word1/word2", t->ops->load64(t->ctx, p + 8),
		    t->ops->load64(t->ctx, p + 16));
	for (i = 0; i < 8; i++)
		b->ops->log(b->ctx, "startup timeout scratch", i, ane_h16_read(b, i));
}

static inline int ane_h16_startup_commands(struct ane_h16_boot *b)
{
	const struct ane_h16_build *f = b->build;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(f->startup); i++) {
		u8 command[100] = {};
		u8 *reply;
		u64 deadline = b->transport_ops->now_ns(b->ctx) + f->command_timeout_ns;

		ane_fw_command_header(command, f->startup[i].id, true);
		ret = ane_fw_transport_publish(&b->transport, ANE_FW_IO, command,
					       f->startup[i].size, deadline);
		if (ret)
			return ret;
		ret = ane_fw_transport_wait(&b->transport, ANE_FW_IO);
		reply = (u8 *)b->pool.cpu +
			(b->transport.pending[ANE_FW_IO].dva - b->pool.dva);
		if (ret) {
			if (ret == -ETIMEDOUT || ret == -ESHUTDOWN)
				ane_h16_startup_timeout(b);
			if (ret == -EREMOTEIO)
				b->ops->log(b->ctx, "startup refusal ID/ack",
					    f->startup[i].id, get_unaligned_le16(reply + 6));
			return ret;
		}
		b->ops->log(b->ctx, "startup ack", f->startup[i].id, get_unaligned_le16(reply + 6));
		if (f->startup[i].id == 3) {
			b->tick = get_unaligned_le32(reply + 8);
			b->ops->log(b->ctx, "CONFIG_GET tick", b->tick, 0);
			if (!b->tick)
				return -EPROTO;
		}
		if (f->startup[i].id == 0x22)
			memcpy(b->resource_info, reply, sizeof(b->resource_info));
		ret = ane_h16_faults(b, false);
		if (ret)
			return ret;
		/* macOS order: START, then SET_SNE_PMU_BASE2 on the control ring. */
		if (f->startup[i].id == 0 && b->pmu_base) {
			b->pmu_base_set = false;
			ret = ane_fw_transport_publish_pmu_base(&b->transport, b->pmu_base,
								deadline);
			if (!ret)
				ret = ane_fw_transport_wait(&b->transport, b->transport.control_ch);
			b->ops->log(b->ctx, "SET_SNE_PMU_BASE2 base/result", b->pmu_base, (u64)ret);
			if (ret)
				return ret;
			b->pmu_base_set = true;
		}
	}
	/* How often a request body was seen before its notification and held. */
	b->ops->log(b->ctx, "startup SHAREDMALLOC allocations/early polls held",
		    b->transport.shared_count, b->transport.shared_early);
	return 0;
}

/*
 * While the startup commands are out the CPU is watched for a stop (its RUN
 * bit cleared, see ane_fw_transport_ops.stopped): a stopped CPU answers
 * nothing, so the startup ends at once with -ESHUTDOWN and gets the same
 * evidence and verified restart as a timeout.
 */
static inline int ane_h16_startup(struct ane_h16_boot *b)
{
	int ret;

	b->starting = true;
	ret = ane_h16_startup_commands(b);
	b->starting = false;
	return ret;
}

/*
 * A startup that timed out left the firmware stopped part way, with requests
 * of its own outstanding. Its rings cannot be resumed, so the engine is
 * power cycled (the backend verifies the reset) and booted again from the
 * preload, as after a cold system sleep: the pool, IPC and heap are reused
 * and cleared, and the shared allocations are carried over, retired, so the
 * new firmware's requests of the same tag and size get the same memory.
 */
static inline int ane_h16_restart(struct ane_h16_boot *b)
{
	struct ane_fw_transport *old;
	int ret = b->ops->restart(b->ctx);

	if (ret)
		return ret;
	old = kmemdup(&b->transport, sizeof(*old), GFP_KERNEL);
	if (!old)
		return -ENOMEM;
	ret = ane_h16_scratch_boot(b);
	if (!ret)
		ane_fw_transport_carry_shared(&b->transport, old);
	kfree(old);
	return ret;
}

/*
 * Run startup; after a timeout or a stop, restart the engine and run it again, at most
 * b->max_restarts times in all against @used. The caller owns @used: one
 * budget for bring-up, a fresh one for each boot after system sleep. Logs
 * @what with the restart count and the budget.
 */
static inline int ane_h16_startup_restarting(struct ane_h16_boot *b,
					     unsigned int *used,
					     const char *what)
{
	int ret = ane_h16_startup(b);

	while ((ret == -ETIMEDOUT || ret == -ESHUTDOWN) && b->ops->restart &&
	       *used < b->max_restarts) {
		(*used)++;
		b->ops->log(b->ctx, what, *used, b->max_restarts);
		ret = ane_h16_restart(b);
		if (!ret)
			ret = ane_h16_startup(b);
	}
	return ret;
}

/*
 * Advance from the last completed stage to @stage. The program stage belongs
 * to the caller; here it runs startup. A stage at or below the completed one,
 * or any call after a failure, is refused.
 */
static inline int ane_h16_run(struct ane_h16_boot *b, enum ane_h16_stage stage)
{
	int ret = 0;

	if (stage == ANE_H16_REFUSE)
		return -EOPNOTSUPP;
	if (stage < ANE_H16_PREFLIGHT || stage > ANE_H16_PROGRAM)
		return -EINVAL;
	if (stage > ANE_H16_STARTUP)
		stage = ANE_H16_STARTUP;
	if (b->failed || b->quarantined || stage <= b->stage)
		return -EBUSY;
	b->attempted = true;
	if (b->stage < ANE_H16_PREFLIGHT) {
		ret = ane_h16_preflight(b);
		if (!ret)
			ret = ane_h16_dapf(b, false);
		if (!ret)
			b->stage = ANE_H16_PREFLIGHT;
	}
	if (!ret && stage >= ANE_H16_MAP && b->stage < ANE_H16_MAP) {
		ret = b->ops->map(b->ctx);
		if (!ret)
			ret = ane_h16_dapf(b, true);
		if (!ret)
			ret = ane_h16_faults(b, true);
		if (!ret)
			b->stage = ANE_H16_MAP;
	}
	if (!ret && stage >= ANE_H16_BOOT && b->stage < ANE_H16_BOOT) {
		ret = ane_h16_scratch_boot(b);
		if (!ret)
			b->stage = ANE_H16_BOOT;
	}
	if (!ret && stage >= ANE_H16_STARTUP && b->stage < ANE_H16_STARTUP) {
		ret = ane_h16_startup_restarting(b, &b->restarts,
						 "startup restart/budget");
		if (!ret)
			b->stage = ANE_H16_STARTUP;
	}
	if (!ret)
		return 0;
	b->failed = true;
	return ane_h16_fail(b, ret);
}

static inline const char *ane_h16_stage_name(enum ane_h16_stage stage)
{
	static const char * const names[] = {
		"none", "preflight", "map", "boot", "startup", "program",
	};

	return stage <= ANE_H16_PROGRAM ? names[stage] : "invalid";
}

#endif /* __ANE_H16_BOOT_H__ */
