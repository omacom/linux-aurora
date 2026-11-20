// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* H16 program lifecycle on the shared firmware transport. */
#include <linux/ctype.h>
#include <linux/math64.h>
#include <linux/freezer.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include "ane_h16_program.h"

/*
 * The one LOAD slot table. Slot 1 carries the HWX kernel section, or the
 * __TEXT,__const constants when the program has no kernel section, and slot 2
 * the task stream. macOS populated slot 1 in every captured load, with at
 * least 0x4000 bytes at a sub-page DVA, which is the constants section of a
 * program without a kernel. Slots 1 and 2 carry a per-program pair with
 * slot 1 = slot 2 + 1. A later correction to any slot is one line here.
 */
static const struct ane_h16_slot_rule ane_h16_slot_rules[ANE_H16_LOAD_SLOTS] = {
	{ ANE_H16_SLOT_AUX_GENERIC, ANE_H16_RESOURCE_GENERIC, 4 },
	{ ANE_H16_SLOT_KERNEL, ANE_H16_RESOURCE_KERNEL, 3 },
	{ ANE_H16_SLOT_TEXT, ANE_H16_RESOURCE_TEXT, 4 },
	{ ANE_H16_SLOT_AUX_OPERATION, ANE_H16_RESOURCE_OPERATION, 4 },
	{ ANE_H16_SLOT_AUX_PROCEDURE, ANE_H16_RESOURCE_PROCEDURE, 4 },
	{ ANE_H16_SLOT_EMPTY, 0, 0 },
	{ ANE_H16_SLOT_AUX_TEXT_PROPERTIES, ANE_H16_RESOURCE_TEXT_PROPERTIES, 4 },
	{ ANE_H16_SLOT_EMPTY, 0, 0 },
	{ ANE_H16_SLOT_EMPTY, 0, 0 },
};

const struct ane_h16_slot_rule *ane_h16_slot_rule(unsigned int slot)
{
	return slot < ANE_H16_LOAD_SLOTS ? &ane_h16_slot_rules[slot] : NULL;
}
EXPORT_SYMBOL_GPL(ane_h16_slot_rule);

/*
 * Place a section that lies wholly inside one mapped region of that kind.
 * With @pages the slot extends to the end of the section's last 16 KiB page
 * of the region, the form every measured H17 kernel-table LOAD used; the
 * tail is the zero padding of the mapped segment.
 */
static int h16_section_dva(const struct ane_h16_load_plan *plan,
			   const struct ane_h16_load_layout *layout,
			   enum ane_h16_region_kind kind,
			   const struct ane_h16_hwx_section *section,
			   struct ane_h16_load_slot *slot, bool pages)
{
	unsigned int i;

	if (!section->size)
		return -ENOEXEC;
	for (i = 0; i < plan->count; i++) {
		const struct ane_h16_hwx_region *r = &plan->regions[i];
		u64 offset = section->vm - r->vm;

		if (r->kind != kind || section->vm < r->vm || offset >= r->size ||
		    section->size > r->size - offset)
			continue;
		if (offset > layout->regions[i].size ||
		    section->size > layout->regions[i].size - offset)
			return -EINVAL;
		slot->dva = layout->regions[i].dva + offset;
		slot->size = section->size;
		if (pages)
			slot->size = min_t(u64, ALIGN(offset + section->size, SZ_16K),
					   layout->regions[i].size) - offset;
		return 0;
	}
	return -ENOEXEC;
}

int ane_h16_load_slots(const struct ane_h16_hwx *hwx,
		       const struct ane_h16_load_plan *plan,
		       const struct ane_h16_load_layout *layout,
		       struct ane_h16_load_slot slots[ANE_H16_LOAD_SLOTS])
{
	struct ane_h16_load_slot next[ANE_H16_LOAD_SLOTS] = {};
	unsigned int i, aux;
	int ret;

	if (!hwx || !plan || !layout || !slots || !plan->count ||
	    layout->region_count != plan->count)
		return -EINVAL;
	for (i = 0; i < plan->count; i++)
		if (!layout->regions[i].dva || layout->regions[i].size != plan->allocation[i])
			return -EINVAL;
	for (i = 0; i < ANE_H16_LOAD_SLOTS; i++) {
		const struct ane_h16_slot_rule *rule = &ane_h16_slot_rules[i];
		struct ane_h16_load_slot *slot = &next[i];

		switch (rule->source) {
		case ANE_H16_SLOT_EMPTY:
			continue;
		case ANE_H16_SLOT_KERNEL:
			if (hwx->kernel.size)
				ret = h16_section_dva(plan, layout, ANE_H16_REGION_KERNEL,
						      &hwx->kernel, slot, true);
			else
				ret = h16_section_dva(plan, layout, ANE_H16_REGION_TEXT,
						      &hwx->constants, slot, false);
			break;
		case ANE_H16_SLOT_TEXT:
			ret = h16_section_dva(plan, layout, ANE_H16_REGION_TEXT,
					      &hwx->text, slot, false);
			break;
		default:
			aux = rule->source == ANE_H16_SLOT_AUX_GENERIC ? ANE_H16_AUX_GENERIC :
			      rule->source == ANE_H16_SLOT_AUX_OPERATION ? ANE_H16_AUX_OPERATION :
			      rule->source == ANE_H16_SLOT_AUX_PROCEDURE ? ANE_H16_AUX_PROCEDURE :
			      ANE_H16_AUX_TEXT_PROPERTIES;
			ret = 0;
			slot->dva = layout->aux_dva[aux];
			slot->size = layout->aux_size[aux];
			if (!slot->dva || slot->size < ANE_H16_AUX_MIN ||
			    slot->size > ANE_H16_AUX_MAX)
				ret = -EINVAL;
			break;
		}
		if (ret)
			return ret;
		slot->resource = rule->resource;
		slot->class = rule->class;
		slot->present = true;
	}
	memcpy(slots, next, sizeof(next));
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_load_slots);

int ane_h16_load_encode(void *command, size_t length,
			const struct ane_h16_load_slot slots[ANE_H16_LOAD_SLOTS])
{
	u8 *p = command;
	unsigned int i;

	if (!p || !slots || length != ANE_H16_LOAD_SIZE)
		return -EINVAL;
	for (i = 0; i < ANE_H16_LOAD_SLOTS; i++)
		if (slots[i].present &&
		    (!slots[i].size || slots[i].size > U64_MAX - slots[i].dva))
			return -EINVAL;
	memset(p, 0, length);
	for (i = 0; i < ANE_H16_LOAD_SLOTS; i++) {
		u8 *d = p + ANE_H16_LOAD_SLOT_OFFSET + i * ANE_H16_LOAD_SLOT_SIZE;

		if (!slots[i].present)
			continue;
		put_unaligned_le32(1, d);
		put_unaligned_le32(slots[i].resource, d + 4);
		put_unaligned_le32(slots[i].class, d + 8);
		put_unaligned_le32(1, d + 0x10);
		put_unaligned_le64(slots[i].dva, d + 0x18);
		put_unaligned_le64(slots[i].size, d + 0x20);
	}
	put_unaligned_le32(ANE_H16_LOAD_ID_REQUEST, p + ANE_H16_LOAD_ID_OFFSET);
	return ane_h16_command_prepare(p, length, ANE_H16_LOAD_PROGRAM, ANE_FW_IO);
}
EXPORT_SYMBOL_GPL(ane_h16_load_encode);

static int h16_reply_header(const u8 *reply, size_t length, size_t expected, u16 id)
{
	if (!reply || length != expected || get_unaligned_le32(reply) ||
	    get_unaligned_le16(reply + 4) != id)
		return -EPROTO;
	return get_unaligned_le16(reply + 6) ? -EREMOTEIO : 0;
}

/* The reply echoes every slot and replaces the request ID with the program ID. */
int ane_h16_load_reply(const void *reply, size_t length, const void *request,
		       u32 *program)
{
	const u8 *p = reply, *q = request;
	u32 id;
	int ret;

	if (!program || !q)
		return -EINVAL;
	ret = h16_reply_header(p, length, ANE_H16_LOAD_SIZE, ANE_H16_LOAD_PROGRAM);
	if (ret)
		return ret;
	id = get_unaligned_le32(p + ANE_H16_LOAD_ID_OFFSET);
	if (id == ANE_H16_LOAD_ID_REQUEST || get_unaligned_le32(p + 0x1bc) ||
	    memcmp(p + ANE_H16_LOAD_SLOT_OFFSET, q + ANE_H16_LOAD_SLOT_OFFSET,
		   ANE_H16_LOAD_ID_OFFSET - ANE_H16_LOAD_SLOT_OFFSET))
		return -EPROTO;
	/* ID 0 is a valid program ID. */
	*program = id;
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_load_reply);

int ane_h16_create_encode(void *command, size_t length, u32 program)
{
	u8 *p = command;

	if (!p || length != ANE_H16_CREATE_SIZE)
		return -EINVAL;
	/* No process resources travel in CREATE on 27.0; the body stays zero. */
	memset(p, 0, length);
	put_unaligned_le32(program, p + 8);
	return ane_h16_command_prepare(p, length, ANE_H16_CREATE_PROCESS, ANE_FW_IO);
}
EXPORT_SYMBOL_GPL(ane_h16_create_encode);

int ane_h16_create_reply(const void *reply, size_t length, u32 program, u32 *process)
{
	const u8 *p = reply;
	int ret;

	if (!process)
		return -EINVAL;
	ret = h16_reply_header(p, length, ANE_H16_CREATE_SIZE, ANE_H16_CREATE_PROCESS);
	if (ret)
		return ret;
	if (get_unaligned_le32(p + 8) != program ||
	    memchr_inv(p + 0x10, 0, ANE_H16_CREATE_SIZE - 0x10))
		return -EPROTO;
	/* ID 0 is a valid process ID. */
	*process = get_unaligned_le32(p + 0xc);
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_create_reply);

int ane_h16_terminate_encode(void *command, size_t length, u32 program, u32 process)
{
	u8 *p = command;

	if (!p || length != ANE_H16_TERMINATE_SIZE)
		return -EINVAL;
	memset(p, 0, length);
	put_unaligned_le32(program, p + 8);
	put_unaligned_le32(process, p + 0xc);
	return ane_h16_command_prepare(p, length, ANE_H16_TERMINATE_PROCESS, ANE_FW_IO);
}
EXPORT_SYMBOL_GPL(ane_h16_terminate_encode);

int ane_h16_unload_encode(void *command, size_t length, u32 program)
{
	u8 *p = command;

	if (!p || length != ANE_H16_UNLOAD_SIZE)
		return -EINVAL;
	memset(p, 0, length);
	put_unaligned_le32(program, p + 8);
	return ane_h16_command_prepare(p, length, ANE_H16_UNLOAD_PROGRAM, ANE_FW_IO);
}
EXPORT_SYMBOL_GPL(ane_h16_unload_encode);

int ane_h16_gate_encode(void *command, size_t length, u32 value)
{
	u8 *p = command;

	if (!p || length != ANE_H16_GATE_SIZE || value > 1)
		return -EINVAL;
	memset(p, 0, length);
	put_unaligned_le32(value, p + 8);
	return ane_h16_command_prepare(p, length, ANE_H16_POWER_GATE, ANE_FW_DATA_CHAIN);
}
EXPORT_SYMBOL_GPL(ane_h16_gate_encode);

/*
 * Short commands (0x002d, TERMINATE, UNLOAD) have no recorded reply body, so
 * only the transport reply shape is a predicate: same length and command,
 * zero first word and a zero acknowledgment.
 */
int ane_h16_id_reply(const void *reply, size_t length, const void *request)
{
	const u8 *q = request;

	if (!q || length < 8 || length > ANE_H16_TERMINATE_SIZE)
		return -EINVAL;
	return h16_reply_header(reply, length, length, get_unaligned_le16(q + 4));
}
EXPORT_SYMBOL_GPL(ane_h16_id_reply);

/*
 * Kernel-owned EXECUTE resource map. EXECUTE names each buffer by its ID in
 * the generic section that LOAD installs, so the ID comes from that section:
 * entry k describes surface k in HWX surface order. The entry must agree with
 * the surface direction and size. IDs are unique and never the reserved
 * kernel IDs 2 and 3. The class follows the round 5 usage mapping (input
 * usage 1, output usage 2).
 *
 * A program with a __DATA scratch has one more entry, last: the scratch,
 * typed as an input (type 0, direction 1) of exactly the __bss size. Linked
 * programs keep the DRAM edges between their tasks there; the measured H17
 * MobileNet run bound it this way.
 */
int ane_h16_process_bindings(const struct ane_h16_hwx *hwx, const void *generic,
			     size_t generic_size,
			     struct ane_h16_process_binding *bindings,
			     unsigned int *count,
			     struct ane_h16_process_binding *scratch)
{
	const u8 *g = generic;
	unsigned int i, j, entries;
	u32 seen[2] = {};

	if (!hwx || !g || !bindings || !count || hwx->surface_count < 2 ||
	    hwx->surface_count > ANE_H16_SURFACES || (hwx->scratch.size && !scratch))
		return -EINVAL;
	entries = hwx->surface_count + !!hwx->scratch.size;
	if (generic_size != ANE_H16_GENERIC_HEADER + entries * ANE_H16_GENERIC_ENTRY ||
	    get_unaligned_le32(g) != ANE_H16_GENERIC_MAGIC ||
	    !get_unaligned_le32(g + 4) ||
	    get_unaligned_le32(g + 4) > ANE_H16_GENERIC_VERSION ||
	    get_unaligned_le32(g + ANE_H16_GENERIC_COUNT) != entries)
		return -ENOEXEC;
	for (i = 0; i < entries; i++) {
		const u8 *e = g + ANE_H16_GENERIC_HEADER + i * ANE_H16_GENERIC_ENTRY;
		bool is_scratch = i == hwx->surface_count;
		u32 direction = is_scratch ? 1 : hwx->surfaces[i].direction;
		u32 id = get_unaligned_le32(e + 4);
		u64 bytes = get_unaligned_le64(e + 0x20);

		if (direction < 1 || direction > 2)
			return -ENOEXEC;
		if (get_unaligned_le32(e) != 1 || id == 2 || id == 3 ||
		    id > ANE_H16_GENERIC_ID_MAX ||
		    get_unaligned_le32(e + 8) != direction - 1 ||
		    get_unaligned_le32(e + 0x10) != direction)
			return -ENOEXEC;
		/* The scratch entry may round __bss up to its 16 KiB mapping. */
		if (is_scratch ? bytes < hwx->scratch.size ||
				 bytes > ALIGN(hwx->scratch.size, SZ_16K) :
		    bytes < hwx->surfaces[i].span || bytes > hwx->surfaces[i].allocation)
			return -ENOEXEC;
		for (j = 0; j < i && j < hwx->surface_count; j++)
			if (bindings[j].index == id)
				return -ENOEXEC;
		if (is_scratch) {
			*scratch = (struct ane_h16_process_binding) {
				.hwx_vm = hwx->scratch.vm,
				.index = id,
				.class = ane_h16_usage_class(direction),
			};
			continue;
		}
		bindings[i] = (struct ane_h16_process_binding) {
			.hwx_vm = hwx->surfaces[i].section.vm,
			.index = id,
			.class = ane_h16_usage_class(direction),
			.bytes = bytes,
		};
		seen[direction - 1]++;
	}
	if (!seen[0] || seen[1] != 1)
		return -ENOEXEC;
	if (scratch && !hwx->scratch.size)
		*scratch = (struct ane_h16_process_binding) {};
	*count = hwx->surface_count;
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_process_bindings);

/*
 * The operation section binds BAR slots to resources: per record, a count at
 * +8 and (slot, tag) pairs from +0xc. A tag is a generic-section ID, or the
 * kernel resource index for a __KERN_0 program. BAR slot s takes its base
 * from the program descriptor qword at +0x10 * (s + 1), so every pair must
 * name the resource whose HWX address is that base. Kernel programs must bind
 * the kernel: the legacy tag 2 instead makes the H17 firmware assert and stop
 * answering (H17 harness facts, kernel-table programs).
 */
int ane_h16_operation_validate(const struct ane_h16_hwx_snapshot *snapshot,
			       const struct ane_h16_hwx *hwx, const void *operation,
			       size_t operation_size,
			       const struct ane_h16_process_binding *bindings,
			       unsigned int count,
			       const struct ane_h16_process_binding *scratch)
{
	const u8 *o = operation, *c;
	unsigned int r, records, i, j;
	bool kernel_bound = false;
	int ret;

	if (!hwx || !o || !bindings || count != hwx->surface_count ||
	    (hwx->scratch.size && !scratch))
		return -EINVAL;
	ret = ane_h16_hwx_snapshot_check(snapshot);
	if (ret)
		return ret;
	if (!hwx->program_offset || snapshot->size < 0x800 ||
	    hwx->program_offset > snapshot->size - 0x800)
		return -ENOEXEC;
	c = (const u8 *)snapshot->bytes + hwx->program_offset;
	if (operation_size < 4)
		return -ENOEXEC;
	records = get_unaligned_le32(o);
	if (!records || records > ANE_H16_OPERATION_RECORDS_MAX ||
	    operation_size != 4 + (size_t)records * ANE_H16_OPERATION_RECORD)
		return -ENOEXEC;
	for (r = 0; r < records; r++) {
		const u8 *rec = o + 4 + r * ANE_H16_OPERATION_RECORD;
		u32 refs = get_unaligned_le32(rec + 8);
		u64 seen = 0;

		if (get_unaligned_le32(rec) || get_unaligned_le32(rec + 4) ||
		    refs > ANE_H16_OPERATION_REFS_MAX)
			return -ENOEXEC;
		for (i = 0; i < refs; i++) {
			u32 slot = get_unaligned_le32(rec + 0xc + i * 8);
			u32 tag = get_unaligned_le32(rec + 0x10 + i * 8);
			u64 base, want = 0;

			if (slot >= 64 || (seen & BIT_ULL(slot)))
				return -ENOEXEC;
			seen |= BIT_ULL(slot);
			base = get_unaligned_le64(c + 0x10 * (slot + 1));
			if (tag == ANE_H16_RESOURCE_KERNEL && hwx->kernel.size) {
				want = hwx->kernel.vm;
				kernel_bound = true;
			} else if (scratch && hwx->scratch.size && tag == scratch->index) {
				want = scratch->hwx_vm;
			} else {
				for (j = 0; j < count; j++)
					if (tag == bindings[j].index)
						want = bindings[j].hwx_vm;
			}
			if (!want || base != want)
				return -ENOEXEC;
		}
	}
	return hwx->kernel.size && !kernel_bound ? -ENOEXEC : 0;
}
EXPORT_SYMBOL_GPL(ane_h16_operation_validate);

/*
 * The mcache custom-BAR records come from the program descriptor relocation
 * tuples pinned by the HWX validator.  The linear 0x8a0 form has one prefetch
 * tuple (BAR ID, field, bit offset, width at +0x868, +0x870, +0x878, +0x87c).
 * The scratch-backed 0x8c0 form has that tuple plus one intermediate-spill
 * tuple in the kind-3 payload at +0x890.  The two records use independent
 * mcache value types even though both name BAR 4 and their bit ranges overlap.
 */
int ane_h16_custom_bars(const struct ane_h16_hwx_snapshot *snapshot,
			const struct ane_h16_hwx *hwx, u32 dsid,
			struct ane_h16_custom_bar *bars, unsigned int *count)
{
	const u8 *c;
	u32 bar, offset, width, length;
	int ret;

	if (!hwx || !bars || !count)
		return -EINVAL;
	ret = ane_h16_hwx_snapshot_check(snapshot);
	if (ret)
		return ret;
	if (!hwx->program_offset || snapshot->size < 0x8a0 ||
	    hwx->program_offset > snapshot->size - 0x8a0)
		return -ENOEXEC;
	c = (const u8 *)snapshot->bytes + hwx->program_offset;
	*count = 0;
	length = get_unaligned_le32(c + 4);
	bar = get_unaligned_le32(c + 0x868);
	offset = get_unaligned_le32(c + 0x878);
	width = get_unaligned_le32(c + 0x87c);
	if (!width || width > 16 || offset > 64 - width || dsid >= BIT(width))
		return -ENOEXEC;
	bars[0] = (struct ane_h16_custom_bar) {
		.configuration = 1,
		.value_type = 0,
		.bar = bar,
		.bit_offset = offset,
		.bit_width = width,
		.value = dsid,
	};
	*count = 1;
	if (length == 0x8a0)
		return 0;
	if (length != 0x8c0 || snapshot->size < 0x8c0 ||
	    hwx->program_offset > snapshot->size - 0x8c0)
		return -ENOEXEC;
	bar = get_unaligned_le16(c + 0x890);
	offset = c[0x893];
	width = c[0x894];
	if (!width || width > 16 || offset > 64 - width || dsid >= BIT(width))
		return -ENOEXEC;
	bars[1] = (struct ane_h16_custom_bar) {
		.configuration = 1,
		.value_type = 1,
		.bar = bar,
		.bit_offset = offset,
		.bit_width = width,
		.value = dsid,
	};
	*count = 2;
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_custom_bars);

/* Read-only after module load: use depth 1 for the throughput A/B baseline. */
static unsigned int execute_depth = 2;
module_param(execute_depth, uint, 0444);
MODULE_PARM_DESC(execute_depth, "Maximum concurrent EXECUTEs (1-4, default 2)");

static void h16_gate_work(struct work_struct *work);
static void h16_export_work(struct work_struct *work);

/* Lifecycle traffic must not interleave with a batch's command arena. */
static void h16_drain(struct ane_h16_session *s)
{
	if (!s->active)
		return;
	s->draining++;
	mutex_unlock(s->lock);
	wait_event(s->wake, !READ_ONCE(s->active));
	mutex_lock(s->lock);
	s->draining--;
	wake_up_all(&s->wake);
}

static int h16_session_init(struct ane_h16_session *s, struct ane_fw_transport *transport,
			    const struct ane_h16_session_ops *ops, void *ctx,
			    u64 timeout_ns, bool parked)
{
	if (!s || !transport || !transport->soc || (!parked && !transport->armed) || !ops ||
	    !ops->map || !ops->log || !timeout_ns || (parked && !ops->wake))
		return -EINVAL;
	memset(s, 0, sizeof(*s));
	s->transport = transport;
	s->ops = ops;
	s->ctx = ctx;
	s->timeout_ns = timeout_ns;
	s->dsid = ANE_H16_MCACHE_DSID;
	s->settle_ns = ANE_H16_SETTLE_QUIET_NS;
	s->depth = clamp(execute_depth, 1U, 4U);
	init_waitqueue_head(&s->wake);
	INIT_LIST_HEAD(&s->deferred);
	INIT_LIST_HEAD(&s->programs);
	INIT_LIST_HEAD(&s->held);
	spin_lock_init(&s->stats_lock);
	INIT_DELAYED_WORK(&s->gate_work, h16_gate_work);
	spin_lock_init(&s->export_lock);
	INIT_LIST_HEAD(&s->exports_done);
	INIT_WORK(&s->export_work, h16_export_work);
	s->asleep = parked;
	s->parked = parked;
	return 0;
}

int ane_h16_session_init(struct ane_h16_session *s, struct ane_fw_transport *transport,
			 const struct ane_h16_session_ops *ops, void *ctx,
			 u64 timeout_ns)
{
	return h16_session_init(s, transport, ops, ctx, timeout_ns, false);
}
EXPORT_SYMBOL_GPL(ane_h16_session_init);

/* Closed and parked, its transport armed only by ops->wake on the first request. */
int ane_h16_session_init_parked(struct ane_h16_session *s, struct ane_fw_transport *transport,
				const struct ane_h16_session_ops *ops, void *ctx,
				u64 timeout_ns)
{
	return h16_session_init(s, transport, ops, ctx, timeout_ns, true);
}
EXPORT_SYMBOL_GPL(ane_h16_session_init_parked);

static int h16_fail(struct ane_h16_session *s, int error)
{
	if (!s->error)
		s->error = error;
	if (!s->quarantined) {
		s->quarantined = true;
		ane_h16_pending_fail_all(&s->pending, s->error);
		wake_up_all(&s->wake);
		s->ops->log(s->ctx, "program quarantine", (u64)-s->error, 0);
		if (s->ops->hung)
			s->ops->hung(s->ctx, s->error);
	}
	return s->error;
}

static u64 h16_now(struct ane_h16_session *s)
{
	return s->transport->ops->now_ns(s->transport->ctx);
}

/*
 * Publish one command at arena offset at (ANE_FW_ARENA: the arena's end) and
 * wait for its transport reply, kept in the pool.
 */
static int h16_send_at(struct ane_h16_session *s, unsigned int ch, const void *command,
		       size_t length, size_t at, const u8 **reply)
{
	struct ane_fw_transport *t = s->transport;
	int ret;

	if (ane_fw_command_denied(get_unaligned_le16((const u8 *)command + 4)))
		return -EPERM;
	ret = ane_fw_transport_publish_at(t, ch, command, length,
					  h16_now(s) + s->timeout_ns, at);
	if (ret)
		return ret;
	ret = ane_fw_transport_wait(t, ch);
	*reply = (const u8 *)t->pool.cpu + (t->pending[ch].dva - t->pool.dva);
	return ret;
}

static int h16_send(struct ane_h16_session *s, unsigned int ch, const void *command,
		    size_t length, const u8 **reply)
{
	return h16_send_at(s, ch, command, length, ANE_FW_ARENA, reply);
}

/*
 * The transport copies commands into a pool arena. A program's LOAD and
 * CREATE stay in its own record (h16_record_get()) until the firmware
 * acknowledged its UNLOAD. Everything else goes to the end of the arena and
 * is given back as soon as the firmware is done with it: each EXECUTE
 * sequence once the firmware reported the request complete, a gate or a
 * TERMINATE and UNLOAD outside a request once acknowledged. Refuse a
 * sequence up front when it cannot fit, so running out of arena is a clean
 * -ENOSPC and never a half-sent sequence.
 */
static int h16_pool_room(struct ane_h16_session *s, const size_t *lengths,
			 unsigned int count)
{
	const struct ane_fw_transport *t = s->transport;
	size_t used = t->pool_used;
	unsigned int i;

	for (i = 0; i < count; i++) {
		used = ALIGN(used, ANE_FW_SLOT_SIZE);
		if (used > t->pool.size || lengths[i] > t->pool.size - used)
			return -ENOSPC;
		used += lengths[i];
	}
	return 0;
}

/*
 * Outside a request batch, the commands published since mark were all
 * acknowledged and the firmware keeps no pointer into them: give their
 * arena back. A batch gives back its own commands, and those published
 * while it ran, when it ends; a quarantined session keeps everything.
 */
static void h16_pool_release(struct ane_h16_session *s, size_t mark)
{
	int ret;

	if (s->running || s->quarantined)
		return;
	ret = ane_fw_transport_release(s->transport, mark);
	if (ret)
		s->ops->log(s->ctx, "command arena kept", (u64)-ret, mark);
}

/*
 * Each program's LOAD and CREATE get a record of their own, freed at its
 * UNLOAD, so a program that stays loaded never keeps another program's
 * commands in the arena, and EXECUTE batches never wait for program
 * lifetimes. The records are taken at the first LOAD of a firmware boot,
 * before any EXECUTE batch.
 */
static int h16_records_ready(struct ane_h16_session *s)
{
	if (s->transport->placed_size)
		return 0;
	/* A new placed range: no record of it is in use. */
	bitmap_zero(s->records, ANE_H16_RECORDS);
	return ane_fw_transport_reserve(s->transport, ANE_H16_RECORDS * ANE_H16_RECORD_SIZE);
}

/* Room for one more program: a free record, and its teardown at the arena's end. */
static int h16_load_room(struct ane_h16_session *s)
{
	static const size_t teardown[] = {
		ANE_H16_GATE_SIZE, ANE_H16_TERMINATE_SIZE, ANE_H16_UNLOAD_SIZE,
	};
	int ret = h16_records_ready(s);

	if (ret)
		return ret;
	if (bitmap_full(s->records, ANE_H16_RECORDS))
		return -ENOSPC;
	return h16_pool_room(s, teardown, ARRAY_SIZE(teardown));
}

static int h16_record_get(struct ane_h16_session *s, struct ane_h16_program *p)
{
	unsigned int i;
	int ret = h16_records_ready(s);

	if (ret)
		return ret;
	i = find_first_zero_bit(s->records, ANE_H16_RECORDS);
	if (i >= ANE_H16_RECORDS)
		return -ENOSPC;
	__set_bit(i, s->records);
	p->record = i;
	return 0;
}

/*
 * The firmware acknowledged the program's UNLOAD, after which it keeps no
 * pointer into its LOAD and CREATE: the record serves the next program.
 */
static void h16_record_put(struct ane_h16_session *s, struct ane_h16_program *p)
{
	if (p->record >= 0 && p->epoch == s->epoch)
		__clear_bit(p->record, s->records);
	p->record = -1;
}

/* Arena offset of the program's LOAD, or with create of its CREATE. */
static size_t h16_record_at(const struct ane_h16_session *s, const struct ane_h16_program *p,
			    bool create)
{
	return s->transport->placed + p->record * ANE_H16_RECORD_SIZE +
	       (create ? ALIGN(ANE_H16_LOAD_SIZE, ANE_FW_SLOT_SIZE) : 0);
}

static void h16_sync(struct ane_h16_session *s, const struct ane_fw_surface *surface,
		     bool for_device)
{
	if (s->ops->sync && surface->handle)
		s->ops->sync(s->ctx, surface, for_device);
}

/*
 * Around each EXECUTE: inputs and the poisoned outputs reach memory before
 * the doorbell (for_device), and the outputs are read fresh after the
 * completion report (!for_device). Caller holds the node lock.
 */
void ane_h16_session_sync(struct ane_h16_session *s, struct ane_h16_program *p,
			  bool for_device)
{
	unsigned int i;

	for (i = 0; i < p->resource_count; i++)
		if (for_device || p->resources[i].permissions == 2)
			h16_sync(s, p->surfaces[i], for_device);
}
EXPORT_SYMBOL_GPL(ane_h16_session_sync);

/*
 * Spare memory can go back to the backend: it has ops->unmap and the engine
 * is up, so the DART invalidation that comes with the unmap reaches it.
 */
static bool h16_can_unmap(const struct ane_h16_session *s)
{
	return s->ops->unmap && !s->asleep && !s->parked;
}

static struct ane_fw_surface h16_spare_remove(struct ane_h16_session *s, unsigned int i)
{
	struct ane_fw_surface surface = s->spares[i].surface;

	s->spare_bytes -= surface.size;
	s->spare_count--;
	memmove(&s->spares[i], &s->spares[i + 1], (s->spare_count - i) * sizeof(s->spares[0]));
	return surface;
}

/* Free the oldest spares until at most @count entries and @bytes remain. */
static void h16_spares_trim(struct ane_h16_session *s, unsigned int count, size_t bytes)
{
	if (!h16_can_unmap(s))
		return;
	while (s->spare_count && (s->spare_count > count || s->spare_bytes > bytes)) {
		struct ane_fw_surface surface = h16_spare_remove(s, 0);

		s->ops->unmap(s->ctx, &surface);
	}
}

static bool h16_take_spare(struct ane_h16_session *s, size_t size, bool write,
			   struct ane_fw_surface *out)
{
	unsigned int i;

	for (i = 0; i < s->spare_count; i++) {
		if (s->spares[i].surface.size != size || s->spares[i].write != write)
			continue;
		*out = h16_spare_remove(s, i);
		/* Zeroed like new memory; LOAD writes it back after filling it. */
		memset(out->cpu, 0, size);
		return true;
	}
	return false;
}

static void h16_give_spare(struct ane_h16_session *s, struct ane_fw_surface *surface,
			   bool write)
{
	if (!surface->cpu) {
		memset(surface, 0, sizeof(*surface));
		return;
	}
	if (s->spare_count < ANE_H16_SPARES) {
		s->spares[s->spare_count++] = (struct ane_h16_spare) {
			.surface = *surface,
			.write = write,
		};
		s->spare_bytes += surface->size;
	} else if (h16_can_unmap(s)) {
		s->ops->unmap(s->ctx, surface);
	}
	/* Otherwise a full table keeps the surface mapped and unused until reset. */
	memset(surface, 0, sizeof(*surface));
}

static void h16_export_work(struct work_struct *work)
{
	struct ane_h16_session *s = container_of(work, struct ane_h16_session, export_work);
	struct ane_h16_export *e, *next;
	LIST_HEAD(done);

	spin_lock(&s->export_lock);
	list_splice_init(&s->exports_done, &done);
	spin_unlock(&s->export_lock);
	mutex_lock(s->lock);
	list_for_each_entry_safe(e, next, &done, node) {
		list_del(&e->node);
		h16_give_spare(s, &e->surface, e->write);
		kfree(e);
	}
	h16_spares_trim(s, ANE_H16_SPARES_KEEP, ANE_H16_SPARE_BYTES);
	mutex_unlock(s->lock);
}

/* Return released exports now. Caller does not hold the node lock. */
void ane_h16_session_flush_exports(struct ane_h16_session *s)
{
	flush_work(&s->export_work);
}
EXPORT_SYMBOL_GPL(ane_h16_session_flush_exports);

/*
 * A process maps surface @index of @p. The first mapping of a surface
 * creates its export, held by the program; each call returns one more
 * reference for the caller's mapping. Caller holds the node lock.
 */
int ane_h16_session_export(struct ane_h16_session *s, struct ane_h16_program *p,
			   unsigned int index, struct ane_h16_export **out)
{
	struct ane_h16_export *e;
	unsigned int j;

	if (!s || !p || p->session != s || p->closed || p->released ||
	    index >= p->resource_count || !p->surfaces[index])
		return -EINVAL;
	j = p->surfaces[index] - p->regions;
	if (j >= p->plan.count)
		return -EINVAL;
	e = p->exports[j];
	if (!e) {
		e = kzalloc_obj(*e);
		if (!e)
			return -ENOMEM;
		refcount_set(&e->ref, 1);
		e->session = s;
		e->surface = p->regions[j];
		e->write = p->plan.regions[j].permissions & 2;
		INIT_LIST_HEAD(&e->node);
		p->exports[j] = e;
	}
	refcount_inc(&e->ref);
	*out = e;
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_session_export);

void ane_h16_export_get(struct ane_h16_export *e)
{
	refcount_inc(&e->ref);
}
EXPORT_SYMBOL_GPL(ane_h16_export_get);

/*
 * Drop a reference. The last one returns a released surface to the spares
 * from a work item: a mapping's last put runs under the process mmap lock,
 * which an EXECUTE may wait for while holding the node lock.
 */
void ane_h16_export_put(struct ane_h16_export *e)
{
	struct ane_h16_session *s;

	if (!e || !refcount_dec_and_test(&e->ref))
		return;
	s = e->session;
	spin_lock(&s->export_lock);
	list_add_tail(&e->node, &s->exports_done);
	spin_unlock(&s->export_lock);
	schedule_work(&s->export_work);
}
EXPORT_SYMBOL_GPL(ane_h16_export_put);

static void h16_give_back(struct ane_h16_session *s, struct ane_h16_program *p)
{
	unsigned int i;

	for (i = 0; i < p->plan.count; i++) {
		struct ane_h16_export *e = p->exports[i];

		if (e) {
			/* Mapped by a process: the export returns it after the last unmap. */
			e->released = true;
			p->exports[i] = NULL;
			memset(&p->regions[i], 0, sizeof(p->regions[i]));
			ane_h16_export_put(e);
			continue;
		}
		h16_give_spare(s, &p->regions[i], p->plan.regions[i].permissions & 2);
	}
	for (i = 0; i < ANE_H16_AUX_COUNT; i++)
		h16_give_spare(s, &p->aux[i], false);
	memset(p->surfaces, 0, sizeof(p->surfaces));
	p->released = true;
	h16_spares_trim(s, ANE_H16_SPARES_KEEP, ANE_H16_SPARE_BYTES);
}

static int h16_map(struct ane_h16_session *s, size_t size, bool write,
		   struct ane_fw_surface *out)
{
	int ret = 0;

	if (!h16_take_spare(s, size, write, out)) {
		ret = s->ops->map(s->ctx, size, write, out);
		/* No DVA range or memory left: free every spare and try once more. */
		if ((ret == -ENOSPC || ret == -ENOMEM) && s->spare_count && h16_can_unmap(s)) {
			h16_spares_trim(s, 0, 0);
			ret = s->ops->map(s->ctx, size, write, out);
		}
	}
	if (ret)
		return ret;
	if (!ane_fw_surface_valid(out) || out->size != size || !IS_ALIGNED(out->dva, SZ_16K))
		return -EPROTO;
	return 0;
}

struct h16_program_binds {
	struct ane_h16_process_binding bindings[ANE_H16_SURFACES];
	struct ane_h16_process_binding scratch;
	unsigned int count;
};

/* Pure checks of the program and its process descriptors; nothing is mapped. */
static int h16_program_bind(const struct ane_h16_program *p,
			    const void *const aux[ANE_H16_AUX_COUNT],
			    const size_t aux_size[ANE_H16_AUX_COUNT],
			    struct h16_program_binds *b)
{
	int ret;

	ret = ane_h16_process_bindings(&p->hwx, aux[ANE_H16_AUX_GENERIC],
				       aux_size[ANE_H16_AUX_GENERIC], b->bindings,
				       &b->count, &b->scratch);
	if (!ret)
		ret = ane_h16_operation_validate(&p->snapshot, &p->hwx,
						 aux[ANE_H16_AUX_OPERATION],
						 aux_size[ANE_H16_AUX_OPERATION],
						 b->bindings, b->count, &b->scratch);
	return ret;
}

static int h16_program_map(struct ane_h16_session *s, struct ane_h16_program *p,
			   const void *const aux[ANE_H16_AUX_COUNT],
			   const size_t aux_size[ANE_H16_AUX_COUNT],
			   struct h16_program_binds *b,
			   struct ane_h16_load_layout *layout)
{
	struct ane_h16_process_binding *bindings = b->bindings, scratch = b->scratch;
	const struct ane_h16_hwx *hwx = &p->hwx;
	unsigned int i, j, count = b->count;
	int ret;

	for (i = 0; i < p->plan.count; i++) {
		const struct ane_h16_hwx_region *r = &p->plan.regions[i];

		if (r->file_size > p->plan.allocation[i] ||
		    r->file_offset > p->snapshot.size ||
		    r->file_size > p->snapshot.size - r->file_offset)
			return -ENOEXEC;
		ret = h16_map(s, p->plan.allocation[i], r->permissions & 2, &p->regions[i]);
		if (ret)
			return ret;
		memcpy(p->regions[i].cpu, (const u8 *)p->snapshot.bytes + r->file_offset,
		       r->file_size);
		/* Weights, text and the zeroed scratch reach memory before LOAD. */
		h16_sync(s, &p->regions[i], true);
		layout->regions[i] = (struct ane_h16_load_mapping) {
			.dva = p->regions[i].dva,
			.size = p->regions[i].size,
			.permissions = r->permissions,
		};
	}
	layout->region_count = p->plan.count;
	for (i = 0; i < ANE_H16_AUX_COUNT; i++) {
		ret = h16_map(s, ALIGN(aux_size[i], SZ_16K), false, &p->aux[i]);
		if (ret)
			return ret;
		memcpy(p->aux[i].cpu, aux[i], aux_size[i]);
		h16_sync(s, &p->aux[i], true);
		layout->aux_dva[i] = p->aux[i].dva;
		layout->aux_size[i] = aux_size[i];
	}
	ret = ane_h16_load_resources_create(&p->snapshot, layout->regions,
					    layout->region_count, bindings, count,
					    p->resources);
	if (ret)
		return ret;
	p->resource_count = count;
	for (i = 0; i < count; i++)
		p->declared[i] = bindings[i].bytes;
	for (i = 0; i < count; i++) {
		for (j = 0; j < p->plan.count; j++)
			if (p->regions[j].dva == p->resources[i].buffer.mapping_dva)
				p->surfaces[i] = &p->regions[j];
		if (!p->surfaces[i])
			return -EINVAL;
	}
	if (hwx->scratch.size) {
		for (j = 0; j < p->plan.count; j++)
			if (p->plan.regions[j].kind == ANE_H16_REGION_SCRATCH)
				break;
		/* The validator puts __bss at the start of its writable segment. */
		if (j == p->plan.count || p->plan.regions[j].vm != hwx->scratch.vm ||
		    p->plan.regions[j].permissions != 3 ||
		    hwx->scratch.size > p->regions[j].size)
			return -ENOEXEC;
		p->scratch = (struct ane_h16_execute_buffer) {
			.index = scratch.index,
			.class = scratch.class,
			.mapping_dva = p->regions[j].dva,
			.mapping_size = p->regions[j].size,
			.size = hwx->scratch.size,
		};
		p->has_scratch = true;
	}
	ret = ane_h16_load_slots(hwx, &p->plan, layout, p->slots);
	if (ret)
		return ret;
	/*
	 * H17 (T8140, 26.6.2 firmware) asserts in EXELOOP right after logging a
	 * custom BAR record (CAneProgramManagerHx.cpp:416) and never reports the
	 * request: the mcache record of a linear relu stopped the Neo there.
	 * Every H17 request that ran sent no record, so H17 programs carry none
	 * and keep the firmware's default mcache DSID.
	 */
	p->bar_count = 0;
	if (s->transport->soc->chip_family >= 17)
		return 0;
	return ane_h16_custom_bars(&p->snapshot, hwx, s->dsid, p->bars, &p->bar_count);
}

/*
 * LOAD or CREATE failed with the firmware holding the program, maybe: the
 * session is quarantined and the program is kept, memory and all, with
 * those closed while quarantined, until a verified power cycle gives them
 * back (ane_h16_session_recovered()).
 */
static int h16_failed_load(struct ane_h16_session *s, struct ane_h16_program *p, int error)
{
	p->closed = true;
	list_add_tail(&p->node, &s->held);
	return h16_fail(s, error);
}

/*
 * On this firmware family a direct EXECUTE reuses the BAR context of the
 * program the engine ran last, so the first EXECUTE of a program writes into
 * the previous program's output buffer unless it is queued. The session
 * queues the first EXECUTE whenever the engine last ran another program, the
 * first one after boot included (see ane_h16_session_execute()). A LOAD is
 * serialized with in-flight EXECUTEs. The transport also requeues on every
 * program or process change, so a session may hold several programs. A
 * program's firmware-visible memory goes back to the spares once the
 * firmware no longer uses it (see h16_give_back()).
 */
int ane_h16_session_load(struct ane_h16_session *s,
			 struct ane_h16_hwx_snapshot *snapshot,
			 const void *const aux[ANE_H16_AUX_COUNT],
			 const size_t aux_size[ANE_H16_AUX_COUNT],
			 struct ane_h16_program **program)
{
	struct ane_h16_load_layout layout = {};
	struct h16_program_binds binds = {};
	struct ane_h16_program *p;
	u8 *command;
	const u8 *reply;
	unsigned int i;
	int ret;

	if (!s || !snapshot || !aux || !aux_size || !program)
		return -EINVAL;
	h16_drain(s);
	if (s->asleep)
		return -EAGAIN;
	if (s->quarantined)
		return -EIO;
	for (i = 0; i < ANE_H16_AUX_COUNT; i++)
		if (!aux[i] || aux_size[i] < ANE_H16_AUX_MIN ||
		    aux_size[i] > ANE_H16_AUX_MAX || !IS_ALIGNED(aux_size[i], 4))
			return -EINVAL;
	ret = ane_h16_hwx_snapshot_check(snapshot);
	if (ret)
		return ret;
	ret = h16_load_room(s);
	if (ret)
		return ret;
	p = kzalloc_obj(*p);
	command = kzalloc(ANE_H16_CREATE_SIZE, GFP_KERNEL);
	if (!p || !command) {
		ret = -ENOMEM;
		goto free;
	}
	ret = ane_h16_load_plan_create(snapshot, &p->plan);
	if (!ret)
		ret = ane_h16_hwx_validate(snapshot->bytes, snapshot->size, &p->hwx);
	/* An HWX runs only on the engine generation it was compiled for. */
	if (!ret && p->hwx.subtype != s->transport->soc->hwx_subtype)
		ret = -ENOEXEC;
	if (ret)
		goto free;
	p->snapshot = *snapshot;
	/*
	 * Refuse a program whose process descriptors do not validate before the
	 * session owns it: nothing is mapped yet, so it can still be freed and
	 * the next LOAD is not blocked by it.
	 */
	ret = h16_program_bind(p, aux, aux_size, &binds);
	if (ret)
		goto free;
	p->record = -1;
	/*
	 * From the first mapping on, firmware-visible memory exists, so the
	 * program becomes the session's here; it is freed only once that memory
	 * went back (ane_h16_session_put()).
	 */
	p->generation = ++s->generation;
	p->epoch = s->epoch;
	INIT_LIST_HEAD(&p->deferred);
	INIT_LIST_HEAD(&p->node);
	memset(snapshot, 0, sizeof(*snapshot));
	p->session = s;
	s->program = p;
	ret = h16_program_map(s, p, aux, aux_size, &binds, &layout);
	if (!ret)
		ret = ane_h16_load_encode(command, ANE_H16_LOAD_SIZE, p->slots);
	if (!ret)
		ret = h16_record_get(s, p);
	if (ret) {
		/* Firmware never saw this program: its memory goes back now. */
		p->closed = true;
		h16_give_back(s, p);
		ane_h16_session_put(s, p);
		goto out;
	}
	/* A refused LOAD or CREATE keeps the record, like the rest, until reset. */
	ret = h16_send_at(s, ANE_FW_IO, command, ANE_H16_LOAD_SIZE, h16_record_at(s, p, false),
			  &reply);
	if (!ret)
		ret = ane_h16_load_reply(reply, ANE_H16_LOAD_SIZE, command, &p->key.program);
	if (ret) {
		s->ops->log(s->ctx, "LOAD refused", (u64)-ret, 0);
		ret = h16_failed_load(s, p, ret);
		goto out;
	}
	s->ops->log(s->ctx, "LOAD program", p->key.program, p->bar_count);
	ret = ane_h16_create_encode(command, ANE_H16_CREATE_SIZE, p->key.program);
	if (!ret)
		ret = h16_send_at(s, ANE_FW_IO, command, ANE_H16_CREATE_SIZE,
				  h16_record_at(s, p, true), &reply);
	if (!ret)
		ret = ane_h16_create_reply(reply, ANE_H16_CREATE_SIZE, p->key.program,
					   &p->key.process);
	if (ret) {
		s->ops->log(s->ctx, "CREATE refused", (u64)-ret, 0);
		ret = h16_failed_load(s, p, ret);
		goto out;
	}
	/* Each validated program has exactly one procedure. */
	p->key.procedure = 0;
	p->created = true;
	list_add_tail(&p->node, &s->programs);
	s->ops->log(s->ctx, "CREATE process", p->key.program, p->key.process);
	*program = p;
out:
	h16_spares_trim(s, ANE_H16_SPARES_KEEP, ANE_H16_SPARE_BYTES);
	kfree(command);
	return ret;
free:
	kfree(command);
	kfree(p);
	return ret;
}
EXPORT_SYMBOL_GPL(ane_h16_session_load);

static int h16_gate(struct ane_h16_session *s, u32 value)
{
	size_t mark = ane_fw_transport_mark(s->transport);
	u8 command[ANE_H16_GATE_SIZE];
	const u8 *reply;
	int ret = ane_h16_gate_encode(command, sizeof(command), value);

	if (!ret)
		ret = h16_send(s, ANE_FW_DATA_CHAIN, command, sizeof(command), &reply);
	if (!ret)
		ret = ane_h16_id_reply(reply, sizeof(command), command);
	if (!ret)
		h16_pool_release(s, mark);
	return ret;
}

/* Gating back on now if it is held off. Caller holds the session lock. */
static int h16_gate_release(struct ane_h16_session *s)
{
	int ret;

	if (!s->gate_held)
		return 0;
	s->gate_held = false;
	if (s->quarantined)
		return -EIO;
	ret = h16_gate(s, 1);
	return ret ? h16_fail(s, ret) : 0;
}

/*
 * The idle end of the hold: gating goes back on once gate_hold_ns passed
 * since the last EXECUTE ended. Caller holds the session lock. Returns 0
 * when gating is on (or was never held off), -EAGAIN while a newer request
 * keeps it off, or the error of the closing gate.
 */
int ane_h16_session_gate_expire(struct ane_h16_session *s)
{
	if (!s->gate_held)
		return 0;
	if (s->running || h16_now(s) - s->gate_last_ns < s->gate_hold_ns)
		return -EAGAIN;
	return h16_gate_release(s);
}
EXPORT_SYMBOL_GPL(ane_h16_session_gate_expire);

static void h16_gate_work(struct work_struct *work)
{
	struct ane_h16_session *s = container_of(to_delayed_work(work),
						 struct ane_h16_session, gate_work);

	mutex_lock(s->lock);
	if (!s->asleep && ane_h16_session_gate_expire(s) == -EAGAIN)
		mod_delayed_work(system_dfl_wq, &s->gate_work, nsecs_to_jiffies(s->gate_hold_ns));
	mutex_unlock(s->lock);
}

/*
 * One firmware log record: a 2-byte header, then NUL-terminated text in the
 * rest of the 0x80 bytes. Copies the printable text (other bytes become '.')
 * into line, which holds ANE_H16_FWLOG_RECORD - 1 bytes, and returns its
 * length; 0 for an empty record.
 */
size_t ane_h16_fwlog_line(const u8 *record, char *line)
{
	size_t i, n = 0;

	for (i = 2; i < ANE_H16_FWLOG_RECORD && record[i]; i++)
		line[n++] = isprint(record[i]) ? record[i] : record[i] == '\t' ? ' ' : '.';
	while (n && line[n - 1] == ' ')
		n--;
	line[n] = 0;
	return n;
}
EXPORT_SYMBOL_GPL(ane_h16_fwlog_line);

/*
 * After a failed EXECUTE the session is quarantined, so this runs at most
 * once per program: print every non-empty record of the firmware log in
 * buffer order (at most 512 lines for the 64 KiB log).
 */
static unsigned int h16_dump_fwlog(struct ane_h16_session *s)
{
	const struct ane_fw_surface *log =
		ane_fw_transport_shared_tagged(s->transport, ANE_FW_SHARED_LOG);
	char line[ANE_H16_FWLOG_RECORD];
	unsigned int i, lines = 0;

	if (!log || !log->cpu) {
		s->ops->log(s->ctx, "fwlog absent", 0, 0);
		return 0;
	}
	s->transport->ops->read_barrier(s->transport->ctx);
	for (i = 0; i < log->size / ANE_H16_FWLOG_RECORD; i++) {
		const u8 *r = (const u8 *)log->cpu + i * ANE_H16_FWLOG_RECORD;

		if (!ane_h16_fwlog_line(r, line))
			continue;
		pr_info("H16 fwlog[%03x] %04x %s\n", i, get_unaligned_le16(r), line);
		lines++;
	}
	s->ops->log(s->ctx, "fwlog records printed", lines, log->dva);
	return lines;
}

/* The firmware text log, for a backend diagnosing a command it sent. */
unsigned int ane_h16_session_dump_fwlog(struct ane_h16_session *s)
{
	return h16_dump_fwlog(s);
}
EXPORT_SYMBOL_GPL(ane_h16_session_dump_fwlog);

/* Non-zero 16-byte rows of a command or reply, as two little-endian words. */
static void h16_dump_rows(const char *what, const u8 *p, size_t length)
{
	size_t off;

	for (off = 0; off + 16 <= length; off += 16) {
		u64 lo = get_unaligned_le64(p + off), hi = get_unaligned_le64(p + off + 8);

		if (lo || hi)
			pr_info("H16 %s +%04zx: %016llx %016llx\n", what, off, lo, hi);
	}
}

/* What one EXECUTE saw, logged only when it fails. */
struct h16_execute_trace {
	u32 step;
	u32 reports;
	u32 last_state;
	u32 other;
	/* When the first report and the completion of this UUID were seen. */
	u64 first_ns;
	u64 done_ns;
};

struct h16_request {
	struct ane_h16_pending pending;
	struct h16_execute_trace trace;
};

enum {
	H16_STEP_GATE_OPEN = 1,
	H16_STEP_SEND,
	H16_STEP_REPORT,
	H16_STEP_GATE_CLOSE,
};

static void h16_trace_report(struct h16_execute_trace *trace, const u8 *report,
			     size_t size, const struct ane_h16_pending *request, u64 now)
{
	struct ane_h16_event e;

	if (!ane_h16_event_decode(report, size, ANE_FW_IO_T2H, &e) &&
	    e.key.uuid == request->key.uuid) {
		if (!trace->reports++)
			trace->first_ns = now;
		trace->last_state = e.state;
	} else {
		trace->other++;
	}
}

/* Dispatch by key, including timing, even when another caller polls the ring. */
static int h16_report(struct ane_h16_session *s, const u8 *report, size_t size)
{
	struct ane_h16_event event;
	unsigned int i;
	u64 now = h16_now(s);

	if (!ane_h16_event_decode(report, size, ANE_FW_IO_T2H, &event)) {
		for (i = 0; i < ANE_H16_PENDING_MAX; i++) {
			struct ane_h16_pending *p = s->pending.slots[i];
			struct h16_request *r;

			if (!p || p->key.uuid != event.key.uuid)
				continue;
			r = container_of(p, struct h16_request, pending);
			h16_trace_report(&r->trace, report, size, p, now);
			if (event.state == 1)
				r->trace.done_ns = now;
			break;
		}
	}
	return ane_h16_pending_dispatch(&s->pending, report, size, ANE_FW_IO_T2H,
					(unsigned long)div_u64(now, NSEC_PER_USEC));
}

/* Wait for this request, serving all keys under the session mutex. */
static int h16_wait_report(struct ane_h16_session *s, struct ane_h16_pending *request,
			   u64 deadline, struct h16_execute_trace *trace)
{
	struct ane_fw_transport *t = s->transport;
	u64 start = h16_now(s), now;
	u8 *report;
	int ret;

	report = kmalloc(ANE_H16_REPORT_MAX, GFP_KERNEL);
	if (!report)
		return -ENOMEM;
	for (;;) {
		size_t size;
		u32 pending;

		/* A request that already ended keeps its own result. */
		if (!request->active) {
			ret = request->status;
			break;
		}
		if (s->quarantined || s->pending.quarantined) {
			ret = s->error ?: s->pending.error ?: -EIO;
			break;
		}
		/* Other keys' reports must not postpone this request's deadline. */
		if (h16_now(s) >= deadline) {
			ret = -ETIMEDOUT;
			break;
		}
		ret = ane_fw_transport_consume(t, report, ANE_H16_REPORT_MAX, &size);
		if (!ret) {
			ret = h16_report(s, report, size);
			if (ret == -ENOENT || ret == -ENOMSG)
				trace->other++;
			else if (ret < 0)
				break;
			continue;
		}
		if (ret != -EAGAIN)
			break;
		ret = ane_fw_transport_irq(t, &pending);
		if (!ret) {
			ret = ane_fw_transport_shared(t);
			if (ret == -EAGAIN)
				ret = 0;
		}
		if (!ret)
			ret = ane_fw_transport_terminal(t);
		if (ret)
			break;
		now = h16_now(s);
		if (now >= deadline) {
			ret = -ETIMEDOUT;
			break;
		}
		t->polls++;
		/*
		 * Publish/ack and dispatch stay serialized; engine time can
		 * overlap. Only the pause runs unlocked: servicing the mailbox
		 * and fault state touches shared transport state.
		 */
		if (s->lock && t->ops->service) {
			t->ops->service(t->ctx);
			mutex_unlock(s->lock);
			if (t->ops->pause)
				t->ops->pause(t->ctx, now - start);
			else
				ane_fw_poll_pause(now - start);
			mutex_lock(s->lock);
		} else if (s->lock) {
			mutex_unlock(s->lock);
			t->ops->relax(t->ctx, now - start);
			mutex_lock(s->lock);
		} else {
			t->ops->relax(t->ctx, now - start);
		}
	}
	kfree(report);
	return ret;
}

/* Include the final gate and teardown pair before admitting another command. */
static int h16_execute_room(struct ane_h16_session *s, struct ane_h16_program *p)
{
	const size_t sequence[] = {
		ANE_H16_GATE_SIZE,
		ANE_H16_EXECUTE_BASE_SIZE + p->bar_count * ANE_H16_CUSTOM_BAR_STRIDE,
		ANE_H16_GATE_SIZE, ANE_H16_TERMINATE_SIZE, ANE_H16_UNLOAD_SIZE,
	};

	return h16_pool_room(s, sequence, ARRAY_SIZE(sequence));
}

/*
 * 0x002d with 0, EXECUTE on channel 7 with its UUID already pending, the
 * matching state-1 report on channel 6, 0x002d with 1, then a DMA read
 * barrier before the caller reads outputs. Any uncertain outcome quarantines
 * the session and keeps every mapping.
 */
static int h16_execute(struct ane_h16_session *s, struct ane_h16_program *p)
{
	struct ane_h16_execute_buffer buffers[ANE_H16_SURFACES + 1];
	struct h16_request r = { .trace.last_state = U32_MAX };
	struct ane_h16_pending *request = &r.pending;
	struct h16_execute_trace *trace = &r.trace;
	struct ane_h16_execute call = {};
	struct ane_h16_exec_sample *x = &s->timing;
	size_t length;
	u8 *command;
	const u8 *reply = NULL;
	unsigned int i;
	u64 deadline, polls, t0, gate_open, t1 = 0, t2 = 0, t3, end;
	u64 entry = h16_now(s);
	int ret;

	if (!s || !p || p->session != s)
		return -EINVAL;
	if (s->quarantined)
		return -EIO;
	if (!p->created || p->closed)
		return -EINVAL;
	/*
	 * The snapshot was hashed and parsed at LOAD, everything the firmware
	 * reads was copied out of it then, and it stays private and unchanged
	 * until the program is freed. Hashing it again here (8.4 MB for
	 * MobileNet, about 2.9 ms) protected nothing per request.
	 */
	ret = ane_h16_resources_validate(&p->hwx, p->resources, p->resource_count);
	if (ret)
		return ret;
	/*
	 * H17 checks each descriptor against the size the generic section
	 * declared for its buffer at LOAD and asserts in its program manager
	 * when it is smaller (relu64: section 0x1000 of a declared 0x4000
	 * surface), so H17 descriptors carry the declared size. H16 keeps the
	 * whole HWX section, which is what ran MobileNet on the J616s; the two
	 * agree there (0x4c000). The declared size lies between the tensor span
	 * and the mapped allocation. Userspace still sees and copies the span.
	 */
	for (i = 0; i < p->resource_count; i++) {
		buffers[i] = p->resources[i].buffer;
		buffers[i].size = s->transport->soc->chip_family >= 17 ? p->declared[i] :
				  p->hwx.surfaces[i].section.size;
	}
	if (p->has_scratch)
		buffers[i++] = p->scratch;
	length = ANE_H16_EXECUTE_BASE_SIZE + p->bar_count * ANE_H16_CUSTOM_BAR_STRIDE;
	ret = h16_execute_room(s, p);
	if (ret)
		return ret;
	command = kzalloc(length, GFP_KERNEL);
	if (!command)
		return -ENOMEM;
	/* Inputs and poisoned outputs must reach memory before any doorbell. */
	s->transport->ops->write_barrier(s->transport->ctx);
	polls = s->transport->polls;
	trace->step = H16_STEP_GATE_OPEN;
	t0 = h16_now(s);
	if (!s->gate_held) {
		ret = h16_gate(s, 0);
		if (ret)
			goto fail;
		s->gate_held = true;
	}
	gate_open = h16_now(s) - t0;
	deadline = t0 + gate_open + s->timeout_ns;
	request->key = p->key;
	request->deadline = (unsigned long)div_u64(deadline, NSEC_PER_USEC);
	ret = ane_h16_pending_insert(&s->pending, request,
				     (unsigned long)div_u64(h16_now(s), NSEC_PER_USEC));
	if (ret)
		goto fail;
	call.key = request->key;
	call.buffers = buffers;
	call.buffer_count = p->resource_count + (p->has_scratch ? 1 : 0);
	/*
	 * The first request of a program on the engine, the first one after boot
	 * included, takes the queued path once: only EXELOOP sets up the BAR
	 * context and the L2-spill mapping that later direct requests reuse.  The
	 * request carries every descriptor-declared custom BAR, including both
	 * mixed-descriptor mcache streams, on the queued and direct paths.
	 */
	if (s->running > 1 || s->executed != p->generation)
		call.flags = ANE_H16_EXECUTE_FLAG_QUEUED;
	s->executed = p->generation;
	call.bars = p->bars;
	call.bar_count = p->bar_count;
	ret = ane_h16_execute_encode(command, length, &call);
	if (!ret)
		ret = ane_h16_command_prepare(command, length, ANE_H16_EXECUTE,
					      ANE_FW_DATA_CHAIN);
	if (!ret) {
		trace->step = H16_STEP_SEND;
		t1 = h16_now(s);
		ret = h16_send(s, ANE_FW_DATA_CHAIN, command, length, &reply);
	}
	/* The channel-7 reply acknowledges transport only, never completion. */
	if (!ret) {
		trace->step = H16_STEP_REPORT;
		t2 = h16_now(s);
		ret = h16_wait_report(s, request, deadline, trace);
	}
	if (ret) {
		if (request->active)
			ane_h16_pending_abort(&s->pending, request, ret);
		goto fail;
	}
	trace->step = H16_STEP_GATE_CLOSE;
	t3 = h16_now(s);
	if (s->running == 1 && !s->gate_hold_ns) {
		ret = h16_gate_release(s);
		if (ret)
			goto fail;
	}
	end = h16_now(s);
	if (s->gate_held && s->running == 1) {
		/* Back-to-back requests find gating off; it goes on when idle. */
		s->gate_last_ns = end;
		if (s->lock)
			mod_delayed_work(system_dfl_wq, &s->gate_work,
					 nsecs_to_jiffies(s->gate_hold_ns));
	}
	s->transport->ops->read_barrier(s->transport->ctx);
	memset(x, 0, sizeof(*x));
	x->ns[ANE_H16_T_PREPARE] = t0 - entry;
	x->ns[ANE_H16_T_GATE_OPEN] = gate_open;
	x->ns[ANE_H16_T_SUBMIT_ACK] = t2 - t1;
	x->ns[ANE_H16_T_ACK_REPORT] = trace->first_ns - t2;
	x->ns[ANE_H16_T_REPORT_DONE] = trace->done_ns - trace->first_ns;
	x->ns[ANE_H16_T_GATE_CLOSE] = end - t3;
	x->ns[ANE_H16_T_FIRMWARE] = end - t0;
	x->polls = s->transport->polls - polls;
	kfree(command);
	x->ns[ANE_H16_T_FINISH] = h16_now(s) - end;
	return 0;
fail:
	s->ops->log(s->ctx, "EXECUTE failed", (u64)-ret, request->key.uuid);
	/*
	 * Step 1 opening gate, 2 channel-7 ack, 3 report wait, 4 closing gate.
	 * Reports for this UUID and the last state (0 start, 3 engine start,
	 * ffffffff none), then the count of other reports seen while waiting.
	 */
	s->ops->log(s->ctx, "EXECUTE step/flags", trace->step, call.flags);
	s->ops->log(s->ctx, "EXECUTE reports/last state", trace->reports, trace->last_state);
	s->ops->log(s->ctx, "EXECUTE other reports", trace->other, 0);
	if (trace->step >= H16_STEP_SEND) {
		h16_dump_rows("EXECUTE sent", command, length);
		if (reply)
			h16_dump_rows("EXECUTE ack", reply, 0x40);
	}
	s->fwlog_lines = h16_dump_fwlog(s);
	kfree(command);
	return h16_fail(s, ret);
}

/* Loaded before the firmware was booted again across system sleep. */
bool ane_h16_program_lost(const struct ane_h16_session *s, const struct ane_h16_program *p)
{
	return p->epoch != s->epoch;
}
EXPORT_SYMBOL_GPL(ane_h16_program_lost);

/* Read without the mutex in wait_event(), then rechecked under it. */
static bool h16_available(struct ane_h16_session *s, struct ane_h16_program *p)
{
	return !READ_ONCE(s->asleep) && !READ_ONCE(s->draining) &&
	       !READ_ONCE(p->executing) && READ_ONCE(s->active) < s->depth;
}

/* Own one program's surfaces, and one bounded session slot, through copy-out. */
int ane_h16_session_acquire(struct ane_h16_session *s, struct ane_h16_program *p)
{
	int ret;

	if (!s || !p || p->session != s)
		return -EINVAL;
	while (!s->quarantined && !h16_available(s, p)) {
		mutex_unlock(s->lock);
		ret = wait_event_freezable(s->wake,
					   READ_ONCE(s->quarantined) || h16_available(s, p));
		mutex_lock(s->lock);
		if (ret)
			return ret;
	}
	if (s->quarantined)
		return -EIO;
	if (ane_h16_program_lost(s, p))
		return -ENODEV;
	if (!p->created || p->closed)
		return -EINVAL;
	p->executing = true;
	s->active++;
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_session_acquire);

void ane_h16_session_release(struct ane_h16_session *s, struct ane_h16_program *p)
{
	p->executing = false;
	s->active--;
	wake_up_all(&s->wake);
}
EXPORT_SYMBOL_GPL(ane_h16_session_release);

/* Surface ownership was acquired before copy-in. Called with s->lock held. */
int ane_h16_session_run(struct ane_h16_session *s, struct ane_h16_program *p)
{
	int ret;

	if (!s || !p || p->session != s || !p->executing)
		return -EINVAL;
	/*
	 * A continuous pipeline can fill the monotonic command arena. Stop
	 * admission and let the current batch drain and reclaim it, without
	 * counting this not-yet-published request among the running commands.
	 */
	if (s->running && h16_execute_room(s, p)) {
		s->draining++;
		do {
			mutex_unlock(s->lock);
			wait_event(s->wake, !READ_ONCE(s->running));
			mutex_lock(s->lock);
		} while (s->running);
		s->draining--;
		wake_up_all(&s->wake);
		if (s->quarantined)
			return s->error;
	}
	if (s->running)
		s->pipelined = true;
	if (!s->running++) {
		s->execute_mark = ane_fw_transport_mark(s->transport);
		if (s->ops->busy)
			s->ops->busy(s->ctx, true);
	}
	ret = h16_execute(s, p);
	if (!--s->running) {
		/* Completion order need not match submission order. */
		if (s->pipelined) {
			s->executed = 0;
			s->transport->bar_known = false;
			s->pipelined = false;
		}
		/* An uncertain outcome pins the entire batch, including its commands. */
		if (!s->quarantined) {
			int err = ane_fw_transport_release(s->transport, s->execute_mark);

			if (err)
				ret = h16_fail(s, err);
		}
		if (s->ops->busy)
			s->ops->busy(s->ctx, false);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ane_h16_session_run);

int ane_h16_session_execute(struct ane_h16_session *s, struct ane_h16_program *p)
{
	int ret = ane_h16_session_acquire(s, p);

	if (ret)
		return ret;
	ret = ane_h16_session_run(s, p);
	ane_h16_session_release(s, p);
	return ret;
}
EXPORT_SYMBOL_GPL(ane_h16_session_execute);

const char *const ane_h16_exec_phase_names[ANE_H16_T_PHASES] = {
	[ANE_H16_T_LOCK] = "lock",
	[ANE_H16_T_COPY_IN] = "copy_in",
	[ANE_H16_T_PREPARE] = "prepare",
	[ANE_H16_T_GATE_OPEN] = "gate_open",
	[ANE_H16_T_SUBMIT_ACK] = "submit_ack",
	[ANE_H16_T_ACK_REPORT] = "ack_report",
	[ANE_H16_T_REPORT_DONE] = "report_done",
	[ANE_H16_T_GATE_CLOSE] = "gate_close",
	[ANE_H16_T_FINISH] = "finish",
	[ANE_H16_T_COPY_OUT] = "copy_out",
	[ANE_H16_T_FIRMWARE] = "firmware",
	[ANE_H16_T_IOCTL] = "ioctl",
};
EXPORT_SYMBOL_GPL(ane_h16_exec_phase_names);

/* Fold one sample into last, sum, min and max. */
void ane_h16_exec_stats_add(struct ane_h16_exec_stats *st,
			    const struct ane_h16_exec_sample *x)
{
	unsigned int i;

	if (!st->count++) {
		st->min = *x;
		st->max = *x;
		memset(&st->sum, 0, sizeof(st->sum));
	}
	for (i = 0; i < ANE_H16_T_PHASES; i++) {
		st->sum.ns[i] += x->ns[i];
		st->min.ns[i] = min(st->min.ns[i], x->ns[i]);
		st->max.ns[i] = max(st->max.ns[i], x->ns[i]);
	}
	st->sum.polls += x->polls;
	st->min.polls = min(st->min.polls, x->polls);
	st->max.polls = max(st->max.polls, x->polls);
	st->last = *x;
}
EXPORT_SYMBOL_GPL(ane_h16_exec_stats_add);

/*
 * Account the last successful EXECUTE with the caller's copy and whole-request
 * times. The caller serializes this with ane_h16_session_execute().
 */
void ane_h16_session_account(struct ane_h16_session *s, u64 lock_ns, u64 copy_in_ns,
			     u64 copy_out_ns, u64 ioctl_ns)
{
	struct ane_h16_exec_sample x = s->timing;

	x.ns[ANE_H16_T_LOCK] = lock_ns;
	x.ns[ANE_H16_T_COPY_IN] = copy_in_ns;
	x.ns[ANE_H16_T_COPY_OUT] = copy_out_ns;
	x.ns[ANE_H16_T_IOCTL] = ioctl_ns;
	spin_lock(&s->stats_lock);
	ane_h16_exec_stats_add(&s->stats, &x);
	spin_unlock(&s->stats_lock);
}
EXPORT_SYMBOL_GPL(ane_h16_session_account);

/* A consistent copy of the statistics, optionally starting them over. */
void ane_h16_session_stats(struct ane_h16_session *s, struct ane_h16_exec_stats *out,
			   bool reset)
{
	spin_lock(&s->stats_lock);
	if (out)
		*out = s->stats;
	if (reset)
		memset(&s->stats, 0, sizeof(s->stats));
	spin_unlock(&s->stats_lock);
}
EXPORT_SYMBOL_GPL(ane_h16_session_stats);

static void h16_us(struct seq_file *m, u64 ns)
{
	seq_printf(m, " %9llu.%u", div_u64(ns, 1000), (unsigned int)(div_u64(ns, 100) % 10));
}

/*
 * Where the time of the successful EXECUTEs since the last reset went, per
 * phase in microseconds (last, mean, min, max), and the poll sleeps per
 * request. Backends expose it as <debugfs>/ane-h1x/<dev>/execute.
 */
int ane_h16_session_stats_show(struct seq_file *m, struct ane_h16_session *s)
{
	struct ane_h16_exec_stats *st;
	unsigned int i;

	st = kmalloc_obj(*st);
	if (!st)
		return -ENOMEM;
	ane_h16_session_stats(s, st, false);
	seq_printf(m, "executes %llu\n", st->count);
	if (st->count) {
		seq_printf(m, "%-12s %11s %11s %11s %11s\n", "phase", "last_us", "mean_us",
			   "min_us", "max_us");
		for (i = 0; i < ANE_H16_T_PHASES; i++) {
			seq_printf(m, "%-12s", ane_h16_exec_phase_names[i]);
			h16_us(m, st->last.ns[i]);
			h16_us(m, div64_u64(st->sum.ns[i], st->count));
			h16_us(m, st->min.ns[i]);
			h16_us(m, st->max.ns[i]);
			seq_putc(m, '\n');
		}
		seq_printf(m, "%-12s %11llu %9llu.%u %11llu %11llu\n", "polls", st->last.polls,
			   div64_u64(st->sum.polls, st->count),
			   (unsigned int)(div64_u64(st->sum.polls * 10, st->count) % 10),
			   st->min.polls, st->max.polls);
	}
	kfree(st);
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_session_stats_show);

/*
 * TERMINATE then UNLOAD; once acknowledged, the program's memory goes back
 * to the spares and its commands give their arena back.
 */
int ane_h16_session_close(struct ane_h16_session *s, struct ane_h16_program *p)
{
	u8 command[ANE_H16_TERMINATE_SIZE];
	const u8 *reply;
	size_t mark;
	int ret;

	if (!s || !p || p->session != s)
		return -EINVAL;
	h16_drain(s);
	if (p->closed || !p->created)
		return 0;
	/*
	 * The firmware is suspended for system sleep and keeps this program:
	 * tear it down when the session reopens. The session owns it until.
	 */
	if (s->asleep) {
		if (list_empty(&p->deferred))
			list_add_tail(&p->deferred, &s->deferred);
		return -EINPROGRESS;
	}
	p->closed = true;
	list_del_init(&p->node);
	/*
	 * The firmware that knew this program was powered off and booted again
	 * from the preload: there is nothing to tear down, and nothing reads
	 * its memory any more.
	 */
	if (ane_h16_program_lost(s, p)) {
		h16_give_back(s, p);
		s->ops->log(s->ctx, "program lost across sleep, released", p->key.program,
			    p->key.process);
		return 0;
	}
	/* Kept, memory and all, until a verified power cycle (held list). */
	if (s->quarantined) {
		list_add_tail(&p->node, &s->held);
		return -EIO;
	}
	/* Nothing runs after the drain: every command from here on is ours. */
	mark = ane_fw_transport_mark(s->transport);
	/* TERMINATE and UNLOAD run with gating on, as without the hold. */
	ret = h16_gate_release(s);
	if (ret) {
		list_add_tail(&p->node, &s->held);
		return ret;
	}
	ret = ane_h16_terminate_encode(command, ANE_H16_TERMINATE_SIZE, p->key.program,
				       p->key.process);
	if (!ret)
		ret = h16_send(s, ANE_FW_IO, command, ANE_H16_TERMINATE_SIZE, &reply);
	if (!ret)
		ret = ane_h16_id_reply(reply, ANE_H16_TERMINATE_SIZE, command);
	if (!ret)
		ret = ane_h16_unload_encode(command, ANE_H16_UNLOAD_SIZE, p->key.program);
	if (!ret)
		ret = h16_send(s, ANE_FW_IO, command, ANE_H16_UNLOAD_SIZE, &reply);
	if (!ret)
		ret = ane_h16_id_reply(reply, ANE_H16_UNLOAD_SIZE, command);
	if (ret) {
		list_add_tail(&p->node, &s->held);
		return h16_fail(s, ret);
	}
	s->ops->log(s->ctx, "TERMINATE and UNLOAD", p->key.program, p->key.process);
	/* Answer the FREE of the process pool while the firmware waits for it. */
	ret = ane_fw_transport_settle(s->transport, s->settle_ns,
				      max(s->settle_ns, ANE_H16_SETTLE_LIMIT_NS));
	if (ret < 0) {
		list_add_tail(&p->node, &s->held);
		return h16_fail(s, ret);
	}
	/*
	 * The firmware acknowledged TERMINATE and UNLOAD, so it no longer
	 * reads or writes this program's memory or its commands: later
	 * programs may reuse them.
	 */
	h16_give_back(s, p);
	h16_record_put(s, p);
	h16_pool_release(s, mark);
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_session_close);

/*
 * Requests (LOAD, EXECUTE) wait here while the engine is cycled across
 * system sleep, freezably so the freezer can stop their task, and enter with
 * the session's serializing lock held.
 */
int ane_h16_session_enter(struct ane_h16_session *s, bool nonblock)
{
	int ret;

	for (;;) {
		/* A parked engine comes up for its first request. */
		if (READ_ONCE(s->parked) && s->ops->wake) {
			ret = s->ops->wake(s->ctx);
			if (ret && ret != -EAGAIN)
				return ret;
		}
		/* O_NONBLOCK: a clean -EAGAIN while the engine is cycled. */
		if (nonblock && READ_ONCE(s->asleep))
			return -EAGAIN;
		ret = wait_event_freezable(s->wake, !READ_ONCE(s->asleep) ||
					   (READ_ONCE(s->parked) && s->ops->wake &&
					    !READ_ONCE(s->wake_blocked)));
		if (ret)
			return ret;
		mutex_lock(s->lock);
		if (!s->asleep)
			return 0;
		mutex_unlock(s->lock);
	}
}
EXPORT_SYMBOL_GPL(ane_h16_session_enter);

/*
 * Close the session to new requests and wait for every surface owner
 * (asleep), or reopen it. The caller cycles the firmware in between.
 */
void ane_h16_session_sleep(struct ane_h16_session *s, bool asleep)
{
	struct ane_h16_program *p, *n;

	mutex_lock(s->lock);
	/* Close admission before dropping the lock to drain every owner. */
	WRITE_ONCE(s->asleep, asleep);
	if (asleep)
		h16_drain(s);
	/* The firmware is suspended or powered off with gating on. */
	if (asleep && s->gate_held && h16_gate_release(s))
		s->ops->log(s->ctx, "gate release before sleep failed", (u64)-s->error, 0);
	WRITE_ONCE(s->asleep, asleep);
	/* Closes that came in while asleep, now that the firmware is back. */
	if (!asleep) {
		list_for_each_entry_safe(p, n, &s->deferred, deferred) {
			list_del_init(&p->deferred);
			if (!ane_h16_session_close(s, p))
				ane_h16_session_put(s, p);
		}
	}
	mutex_unlock(s->lock);
	if (!asleep)
		wake_up_all(&s->wake);
}
EXPORT_SYMBOL_GPL(ane_h16_session_sleep);

/*
 * The firmware was booted again from the preload (system sleep without
 * firmware cooperation): every program loaded before is lost (EXECUTE
 * refuses it with -ENODEV, close only releases its memory), no request is
 * pending, and the next program starts a new BAR context. Called with the
 * session asleep, before it reopens.
 */
void ane_h16_session_restart(struct ane_h16_session *s)
{
	s->epoch++;
	s->program = NULL;
	/* The new firmware knows no command of the old one. */
	bitmap_zero(s->records, ANE_H16_RECORDS);
	/* A firmware booted again from the preload starts with gating on. */
	s->gate_held = false;
	s->executed = 0;
	memset(s->pending.slots, 0, sizeof(s->pending.slots));
	s->pending.error = 0;
	s->error = 0;
}
EXPORT_SYMBOL_GPL(ane_h16_session_restart);

/*
 * After ane_h16_session_restart() on a quarantined session: the backend
 * powered the engine off (every domain verified off) and booted a new
 * firmware from the preload, so nothing reads the old firmware's memory any
 * more. End the quarantine, and give back the memory of every program that
 * was closed while it lasted. Open programs are replayed by the caller.
 * Called with the session asleep.
 */
void ane_h16_session_recovered(struct ane_h16_session *s)
{
	struct ane_h16_program *p, *n;

	s->quarantined = false;
	s->pending.quarantined = false;
	s->pending.error = 0;
	s->error = 0;
	list_for_each_entry_safe(p, n, &s->held, node) {
		list_del_init(&p->node);
		h16_give_back(s, p);
		s->ops->log(s->ctx, "program closed while quarantined, released",
			    p->key.program, p->key.process);
		ane_h16_session_put(s, p);
	}
}
EXPORT_SYMBOL_GPL(ane_h16_session_recovered);

/*
 * Put one program back on a firmware booted again from the preload, behind
 * its existing handle: its regions are rewritten from the HWX snapshot as at
 * its first LOAD (the aux sections and every mapping kept their DVAs), then
 * LOAD and CREATE are sent again and the firmware's new program and process
 * IDs replace the old ones. The firmware sees exactly the image it saw at
 * the first LOAD, so the program computes the same outputs.
 */
static int h16_replay(struct ane_h16_session *s, struct ane_h16_program *p)
{
	u8 *command;
	const u8 *reply;
	unsigned int i;
	int ret;

	command = kzalloc(ANE_H16_CREATE_SIZE, GFP_KERNEL);
	if (!command)
		return -ENOMEM;
	/* The old firmware's record went with it (ane_h16_session_restart()). */
	p->record = -1;
	ret = h16_record_get(s, p);
	if (ret) {
		kfree(command);
		return ret;
	}
	for (i = 0; i < p->plan.count; i++) {
		const struct ane_h16_hwx_region *r = &p->plan.regions[i];

		memset(p->regions[i].cpu, 0, p->regions[i].size);
		memcpy(p->regions[i].cpu, (const u8 *)p->snapshot.bytes + r->file_offset,
		       r->file_size);
		h16_sync(s, &p->regions[i], true);
	}
	ret = ane_h16_load_encode(command, ANE_H16_LOAD_SIZE, p->slots);
	if (!ret)
		ret = h16_send_at(s, ANE_FW_IO, command, ANE_H16_LOAD_SIZE,
				  h16_record_at(s, p, false), &reply);
	if (!ret)
		ret = ane_h16_load_reply(reply, ANE_H16_LOAD_SIZE, command, &p->key.program);
	if (!ret) {
		memset(command, 0, ANE_H16_CREATE_SIZE);
		ret = ane_h16_create_encode(command, ANE_H16_CREATE_SIZE, p->key.program);
	}
	if (!ret)
		ret = h16_send_at(s, ANE_FW_IO, command, ANE_H16_CREATE_SIZE,
				  h16_record_at(s, p, true), &reply);
	if (!ret)
		ret = ane_h16_create_reply(reply, ANE_H16_CREATE_SIZE, p->key.program,
					   &p->key.process);
	kfree(command);
	if (ret)
		return h16_fail(s, ret);
	p->epoch = s->epoch;
	s->ops->log(s->ctx, "program replayed", p->key.program, p->key.process);
	return 0;
}

/*
 * After ane_h16_session_restart(): replay every program that was loaded and
 * not closed, in load order, so that its handle runs again unchanged. A
 * program closed while asleep is only released (it stays lost). A failed
 * replay is a bug: it is logged loudly, quarantines the session (the
 * firmware state is uncertain) and returns the error.
 */
int ane_h16_session_replay(struct ane_h16_session *s, unsigned int *replayed,
			   u64 *first_ns)
{
	struct ane_h16_program *p;
	int ret;

	*replayed = 0;
	*first_ns = 0;
	list_for_each_entry(p, &s->programs, node) {
		if (!list_empty(&p->deferred))
			continue;
		ret = h16_replay(s, p);
		if (ret) {
			s->ops->log(s->ctx, "PROGRAM REPLAY FAILED", p->key.program, (u64)-ret);
			return ret;
		}
		if (!(*replayed)++)
			*first_ns = h16_now(s);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_session_replay);

/*
 * The caller's last reference: free a closed program and its HWX snapshot.
 * A program whose teardown failed keeps everything until reset.
 */
void ane_h16_session_put(struct ane_h16_session *s, struct ane_h16_program *p)
{
	if (!s || !p || p->session != s || !p->released)
		return;
	if (s->program == p)
		s->program = NULL;
	list_del_init(&p->node);
	ane_h16_hwx_snapshot_destroy(&p->snapshot);
	kfree(p);
}
EXPORT_SYMBOL_GPL(ane_h16_session_put);

MODULE_DESCRIPTION("H16 program load, execute and teardown on the shared transport");
MODULE_LICENSE("Dual MIT/GPL");
