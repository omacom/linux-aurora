// SPDX-License-Identifier: GPL-2.0-only
/* Host boundaries for the extracted T6030 PMP selection and power bridge. */
#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define __init
#define IS_ENABLED(x) (x)
#define EPROBE_DEFER 517
#define PMP_REPORT_READY 1
#define DL_FLAG_AUTOREMOVE_CONSUMER 1
#define BIT_ULL(n) (1ULL << (n))
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
struct device_node {
	const char *compatible;
	bool available, has_reg, has_pmp, has_report, no_ack;
	int reg_cells, pmp_cells, report_cells, refs;
	u32 reg, phandle;
	struct device_node *pmp, *report, *child, *sibling, *parent;
};
struct device {
	struct device_node *of_node;
	int refs;
	bool bound;
};
struct platform_device {
	struct device dev;
};
struct device_link {
	int unused;
};
struct of_changeset {
	int unused;
};
struct gate_soc {
	int id;
};
struct apple_pmp_report_offsets {
	unsigned int status, actual, tgt_read, tgt_write;
	bool starts_pmp, devices_ordered;
	const char *name, *fallback, *pwrstate;
};
struct apple_pmp_report {
	void *base;
	const struct apple_pmp_report_offsets *offsets;
	struct device *dev;
	void *start;
	struct apple_pmp_report *next;
};
struct completion {
	bool done;
};
struct apple_pmp_start {
	struct apple_pmp_report *rep;
	struct apple_pmp_start *next;
	bool stopping;
	unsigned int waiters;
	bool queued;
	int result, work;
	u64 ack;
	struct completion started, waiters_done;
};
struct host_list {
	void *first;
};
static struct host_list apple_pmp_starts, apple_pmp_reports;
static int apple_pmp_start_mutex;
#define list_for_each_entry(it, head, member) \
	for ((it) = (head)->first; (it); (it) = (it)->next)
#define list_for_each_entry_rcu(it, head, member) \
	list_for_each_entry(it, head, member)
static int rcu_depth;
static void rcu_read_lock(void)
{
	assert(!rcu_depth++);
}
static void rcu_read_unlock(void)
{
	assert(rcu_depth-- == 1);
}
static int apple_pmp_report_bit(struct apple_pmp_start *st,
				struct device_node *entry, u32 *id,
				bool *record)
{
	(void)st;
	*id = entry->reg;
	*record = false;
	return 0;
}
static struct apple_pmp_start managed_start;
static int ready_result, start_calls;
#define msecs_to_jiffies(n) (n)
#define system_unbound_wq NULL
static bool wait_for_completion_timeout(struct completion *c,
					unsigned long timeout)
{
	assert(timeout > 0);
	return c->done;
}
static void complete(struct completion *c)
{
	c->done = true;
}
static void queue_work(void *wq, int *work);
static const struct gate_soc gate_t6030 = { 6030 }, gate_t8122 = { 8122 },
			     other_soc = { 0 };
static const struct gate_soc *gate_soc;
static struct device_node supplier_node, other_node, gpu, extra_gpu;
static struct device_node report14, extra_report, report25, entry7, entry17;
static struct device_node *nodes[9];
static struct device supplier, consumer;
static struct platform_device native_report, passive_report;
static struct device_link link;
static struct apple_pmp_report_offsets native_offsets;
static u64 status_word;
static struct apple_pmp_report native_rep, passive_rep;
static u64 passive_words[0x14000 / sizeof(u64)];
static bool passive_present, passive_lockable, passive_data;

static const char *machine;
static const void *pmp_data;
static struct device *pmp_device;
static int pmp_lock, ready_calls, add_calls, add_result;
static int poll_calls, device_locked, power_calls, power_result, link_calls;
static int find_calls, lockable, present_device, link_ok;
static int case_count;
/* The report transaction itself is covered by the G15 PMP-vote selftest. */
static int gfx_check_result, gfx_check_calls, gfx_set_calls;

static struct device_node *of_node_get(struct device_node *node)
{
	if (node)
		node->refs++;
	return node;
}
static void of_node_put(struct device_node *node)
{
	if (node) {
		assert(node->refs > 0);
		node->refs--;
	}
}
static bool of_device_is_compatible(const struct device_node *node,
				    const char *name)
{
	return node && node->compatible && !strcmp(node->compatible, name);
}
static bool of_device_is_available(const struct device_node *node)
{
	return node && node->available;
}
static bool of_machine_is_compatible(const char *name)
{
	return !strcmp(machine, name);
}
static struct device_node *of_find_compatible_node(struct device_node *from,
						   const char *type,
						   const char *compat)
{
	size_t i = 0;

	assert(!type);
	find_calls++;
	if (from) {
		for (; i < sizeof(nodes) / sizeof(nodes[0]); i++)
			if (nodes[i] == from) {
				i++;
				break;
			}
		of_node_put(from);
	}
	for (; i < sizeof(nodes) / sizeof(nodes[0]); i++)
		if (of_device_is_compatible(nodes[i], compat))
			return of_node_get(nodes[i]);
	return NULL;
}
#define for_each_compatible_node(np, type, compat)                     \
	for ((np) = of_find_compatible_node(NULL, type, compat); (np); \
	     (np) = of_find_compatible_node(np, type, compat))
static struct device_node *of_get_next_child(struct device_node *parent,
					     struct device_node *old)
{
	struct device_node *next = old ? old->sibling : parent->child;

	of_node_put(old);
	return of_node_get(next);
}
#define for_each_child_of_node(parent, child)                    \
	for ((child) = of_get_next_child(parent, NULL); (child); \
	     (child) = of_get_next_child(parent, child))
static bool of_property_present(const struct device_node *node,
				const char *prop)
{
	if (!strcmp(prop, "apple,pmp"))
		return node->has_pmp;
	assert(!strcmp(prop, "apple,pmp-report"));
	return node->has_report;
}
static bool of_property_read_bool(const struct device_node *node,
				  const char *prop)
{
	assert(!strcmp(prop, "apple,no-ack"));
	return node->no_ack;
}
static int of_property_count_u32_elems(const struct device_node *node,
				       const char *prop)
{
	if (!strcmp(prop, "reg"))
		return node->has_reg ? node->reg_cells : -EINVAL;
	assert(!strcmp(prop, "apple,pmp"));
	return node->has_pmp ? node->pmp_cells : -EINVAL;
}
static int of_property_read_u32(const struct device_node *node,
				const char *prop, u32 *value)
{
	assert(!strcmp(prop, "reg"));
	if (!node->has_reg || node->reg_cells < 1)
		return -EINVAL;
	*value = node->reg;
	return 0;
}
static struct device_node *of_parse_phandle(struct device_node *node,
					    const char *prop, int index)
{
	assert(!index);
	if (!strcmp(prop, "apple,pmp"))
		return node->has_pmp && node->pmp_cells >= 1 ?
			       of_node_get(node->pmp) :
			       NULL;
	assert(!strcmp(prop, "apple,pmp-report"));
	return node->has_report && node->report_cells >= 1 ?
		       of_node_get(node->report) :
		       NULL;
}
static int of_changeset_add_prop_u32(struct of_changeset *cs,
				     struct device_node *node, const char *prop,
				     u32 value)
{
	assert(cs && node == &gpu && !strcmp(prop, "apple,pmp"));
	assert(value == supplier_node.phandle);
	add_calls++;
	return add_result;
}
static struct device_node *of_get_parent(struct device_node *node)
{
	return of_node_get(node->parent);
}
static struct platform_device *of_find_device_by_node(struct device_node *node)
{
	struct platform_device *pdev;

	assert(node == &report25 || node == &report14);
	if (node == &report25) {
		if (!present_device)
			return NULL;
		pdev = &native_report;
	} else {
		if (!passive_present)
			return NULL;
		pdev = &passive_report;
	}
	pdev->dev.refs++;
	return pdev;
}
static bool device_trylock(struct device *dev)
{
	assert(!device_locked);
	assert(dev == &native_report.dev || dev == &passive_report.dev);
	if (dev == &native_report.dev ? !lockable : !passive_lockable)
		return false;
	device_locked = dev == &native_report.dev ? 1 : 2;
	return true;
}
static void device_unlock(struct device *dev)
{
	assert(device_locked == (dev == &native_report.dev ? 1 : 2));
	device_locked = 0;
}
static bool device_is_bound(const struct device *dev)
{
	return dev->bound;
}
static struct apple_pmp_report *
platform_get_drvdata(struct platform_device *pdev)
{
	assert(device_locked == (pdev == &native_report ? 1 : 2));
	return pdev == &native_report ? &native_rep :
	       passive_data	      ? &passive_rep :
					NULL;
}
static u64 readq(const void *address)
{
	if (!device_locked) {
		assert(apple_pmp_start_mutex && managed_start.waiters);
	} else {
		assert(device_locked == 1 && native_report.dev.refs > 0);
	}
	return *(const u64 *)address;
}
#define readq_poll_timeout(address, value, condition, delay, timeout) \
	({                                                            \
		int poll_ret = -ETIMEDOUT;                            \
		assert(device_locked && (delay) == 1000 &&            \
		       (timeout) == 5000000);                         \
		if (device_locked == 2)                               \
			ready_calls++;                                \
		else                                                  \
			poll_calls++;                                 \
		for (int poll_i = 0; poll_i < 4; poll_i++) {          \
			(value) = readq(address);                     \
			if (condition) {                              \
				poll_ret = 0;                         \
				break;                                \
			}                                             \
		}                                                     \
		poll_ret;                                             \
	})
static struct device *get_device(struct device *dev)
{
	dev->refs++;
	return dev;
}
static void put_device(struct device *dev)
{
	assert(dev->refs > 0);
	dev->refs--;
}
static struct device_link *
device_link_add(struct device *con, struct device *sup, unsigned int flags)
{
	assert(con == &consumer && sup == &supplier && sup->refs > 0);
	assert(flags == DL_FLAG_AUTOREMOVE_CONSUMER && !pmp_lock);
	link_calls++;
	return link_ok ? &link : NULL;
}
struct lock_guard {
	int *lock;
	bool once;
};
static struct lock_guard lock_take(int *lock)
{
	assert(!*lock);
	*lock = 1;
	return (struct lock_guard){ lock, true };
}
static void lock_drop(struct lock_guard *guard)
{
	assert(*guard->lock == 1);
	*guard->lock = 0;
}
#define guard(kind)                   \
	struct lock_guard local_guard \
		__attribute__((cleanup(lock_drop))) = lock_take
#define scoped_guard(kind, lock)                                    \
	for (struct lock_guard scoped                               \
	     __attribute__((cleanup(lock_drop))) = lock_take(lock); \
	     scoped.once; scoped.once = false)
static int apple_pmp_send_power_command(const void *data, u8 command, u16 id,
					u32 enabled)
{
	assert(data == pmp_data && command == 0x0f && id == 5 && enabled == 1);
	assert(pmp_lock == 1 && (start_calls == 1 || poll_calls == 1));
	power_calls++;
	return power_result;
}
static void queue_work(void *wq, int *work)
{
	assert(!wq && work == &managed_start.work && apple_pmp_start_mutex);
	/* Firmware boundary: no READY is fabricated by the readiness reader. */
	start_calls++;
	managed_start.result = ready_result;
	managed_start.started.done = true;
}
static int apple_pmp_report_check_t6030_gfx(struct device *dev)
{
	assert(dev == pmp_device && pmp_lock == 1);
	gfx_check_calls++;
	return gfx_check_result;
}
static int apple_pmp_report_set_t6030_gfx(struct device *dev, bool enabled)
{
	assert(dev == pmp_device && pmp_lock == 1);
	(void)enabled;
	gfx_set_calls++;
	return 0;
}
#include "t6030-pmp-production.h"

static void balanced(void)
{
	for (size_t i = 0; i < sizeof(nodes) / sizeof(nodes[0]); i++)
		assert(!nodes[i]->refs);
	assert(!managed_start.waiters);
	assert(!supplier.refs && !consumer.refs && !native_report.dev.refs &&
	       !passive_report.dev.refs);
	assert(!apple_pmp_start_mutex && !rcu_depth);
	assert(!pmp_lock && !device_locked);
}
static void reset(void)
{
	balanced();
	supplier_node =
		(struct device_node){ .compatible = "apple,t6030-pmp-v2",
				      .phandle = 10 };
	other_node = (struct device_node){ .compatible = "apple,t6030-pmp-v2",
					   .phandle = 11 };
	gpu = (struct device_node){ .compatible = "apple,agx-t6030",
				    .available = true };
	extra_gpu = (struct device_node){ 0 };
	report14 =
		(struct device_node){ .compatible = "apple,t6030-pmp-v2-report",
				      .available = true,
				      .has_pmp = true,
				      .pmp_cells = 1,
				      .pmp = &supplier_node,
				      .child = &entry17 };
	extra_report = (struct device_node){ 0 };
	report25 = (struct device_node){
		.compatible = "apple,j613-25g83-pmp-report", .available = true
	};
	entry7 =
		(struct device_node){ .compatible =
					      "apple,t6000-pmp-v2-report-entry",
				      .available = true,
				      .has_reg = true,
				      .reg_cells = 1,
				      .reg = 7,
				      .parent = &report14 };
	entry17 =
		(struct device_node){ .compatible =
					      "apple,t6000-pmp-v2-report-entry",
				      .available = true,
				      .has_reg = true,
				      .reg_cells = 1,
				      .reg = 17,
				      .sibling = &entry7,
				      .parent = &report14 };
	supplier = (struct device){ .of_node = &supplier_node, .bound = true };
	consumer = (struct device){ .of_node = &gpu };
	native_report = (struct platform_device){ .dev = { .of_node = &report25,
							   .bound = true } };
	native_rep = (struct apple_pmp_report){ .base = &status_word,
						.offsets = &native_offsets,
						.dev = &native_report.dev };
	passive_report = (struct platform_device){
		.dev = { .of_node = &report14, .bound = true }
	};
	passive_rep =
		(struct apple_pmp_report){ .base = passive_words,
					   .offsets = &apple_pmp_offsets_t6030,
					   .dev = &passive_report.dev };
	memset(passive_words, 0, sizeof(passive_words));
	passive_words[apple_pmp_offsets_t6030.status / 8] = PMP_REPORT_READY;
	passive_words[apple_pmp_offsets_t6030.actual / 8] = BIT_ULL(7);
	passive_present = passive_lockable = passive_data = true;
	managed_start = (struct apple_pmp_start){ .rep = &passive_rep,
						  .ack = BIT_ULL(7) };
	passive_rep.start = &managed_start;
	apple_pmp_starts.first =
		CONFIG_APPLE_T6030_GPU_PMP_LINK ? &managed_start : NULL;
	ready_result = start_calls = 0;
	apple_pmp_reports.first = &passive_rep;
	machine = "apple,t6030";
	gate_soc = &gate_t6030;
	pmp_device = &supplier;
	pmp_data = (void *)1;
	ready_calls = add_calls = add_result = poll_calls = 0;
	power_calls = power_result = link_calls = find_calls = 0;
	gfx_check_result = gfx_check_calls = gfx_set_calls = 0;
	lockable = present_device = link_ok = 1;
	status_word = PMP_REPORT_READY;
}
/* The T8122 (M3 Air) 14.x graph: the same report shape, with T8122 compatibles. */
static void t8122(void)
{
	machine = "apple,t8122";
	gate_soc = &gate_t8122;
	supplier_node.compatible = "apple,t8122-pmp-v2";
	other_node.compatible = "apple,t8122-pmp-v2";
	gpu.compatible = "apple,agx-t8122";
	report14.compatible = "apple,t8122-pmp-v2-report";
	apple_pmp_starts.first = &managed_start;
}
static int power(void)
{
	return apple_pmp_set_device_power(0x0f, 5, 1);
}
#define RUN(change, result, ready, sent)                                 \
	do {                                                             \
		reset();                                                 \
		change;                                                  \
		assert(power() == (result));                             \
		assert(ready_calls == (ready) && power_calls == (sent)); \
		balanced();                                              \
		case_count++;                                            \
	} while (0)
int main(void)
{
	struct device_node *initial[] = {
		&supplier_node, &other_node, &gpu,    &extra_gpu, &report14,
		&extra_report,	&report25,   &entry7, &entry17
	};
	memcpy(nodes, initial, sizeof(nodes));
	/* Keep host boundary functions available when a mutant removes a call. */
	(void)apple_pmp_report_wait_ready;
	(void)complete;
	(void)of_machine_is_compatible;
	(void)of_property_read_bool;
	(void)of_property_present;
	(void)gate_find_one;
	(void)apple_pmp_report_t6030_entry;
	(void)apple_pmp_report_wait_t6030_14;
	(void)apple_pmp_report_wait_t8122_14;
	reset();
	assert(apple_pmp_offsets_t6030.starts_pmp ==
	       !!CONFIG_APPLE_T6030_GPU_PMP_LINK);
#if CONFIG_APPLE_T6030_GPU_PMP_LINK
	assert(apple_pmp_report_wait_ready(&entry7, 10000) == 0 &&
	       start_calls == 1);
	balanced();
	case_count++;
	RUN((void)0, 0, 0, 1);
	assert(gfx_check_calls == 1 && gfx_set_calls == 0);
	RUN(gfx_check_result = -EINVAL, -EINVAL, 0, 0);
	assert(gfx_check_calls == 1 && gfx_set_calls == 0);
	RUN(report14.child = &entry7; entry7.sibling = &entry17;
	    entry17.sibling = NULL, 0, 0, 1);
	RUN(apple_pmp_starts.first = NULL, -ENODEV, 0, 0);
	RUN(apple_pmp_starts.first = NULL;
	    apple_pmp_reports.first = NULL, -EPROBE_DEFER, 0, 0);
	RUN(managed_start.stopping = true, -ENODEV, 0, 0);
	RUN(ready_result = -ETIMEDOUT, -ETIMEDOUT, 0, 0);
	RUN(ready_result = -EINVAL, -EINVAL, 0, 0);
	RUN(passive_words[apple_pmp_offsets_t6030.status / 8] = 0, -EIO, 0, 0);
	RUN(passive_words[apple_pmp_offsets_t6030.status / 8] = 3, -EIO, 0, 0);
	RUN(passive_words[apple_pmp_offsets_t6030.actual / 8] = 0, -EIO, 0, 0);
	RUN(passive_words[apple_pmp_offsets_t6030.actual / 8] = BIT_ULL(8),
	    -EIO, 0, 0);
	RUN(passive_words[apple_pmp_offsets_t6030.actual / 8] |= BIT_ULL(9), 0,
	    0, 1);
	RUN(power_result = -EIO, -EIO, 0, 1);
	RUN(machine = "apple,t8122", -ENODEV, 0, 0);
	RUN(machine = "apple,t6031", -ENODEV, 0, 0);
	RUN(machine = "apple,t8140", -ENODEV, 0, 0);
	RUN(supplier_node.compatible = "apple,t8122-pmp-v2", -ENODEV, 0, 0);
	RUN(report14.compatible = NULL, -ENODEV, 0, 0);
	RUN(extra_report = report14, -ENODEV, 0, 0);
	RUN(extra_report = report14;
	    extra_report.available = false, -ENODEV, 0, 0);
	RUN(report14.available = false, -ENODEV, 0, 0);
	RUN(report14.has_pmp = false, -ENODEV, 0, 0);
	RUN(report14.pmp_cells = 0, -ENODEV, 0, 0);
	RUN(report14.pmp_cells = 2, -ENODEV, 0, 0);
	RUN(report14.pmp = NULL, -ENODEV, 0, 0);
	RUN(report14.pmp = &other_node, -ENODEV, 0, 0);
	RUN(entry17.sibling = NULL, -ENODEV, 0, 0);
	RUN(entry7.has_reg = false, -ENODEV, 0, 0);
	RUN(entry7.reg_cells = 0, -ENODEV, 0, 0);
	RUN(entry7.reg_cells = 2, -ENODEV, 0, 0);
	RUN(entry7.reg = 8, -ENODEV, 0, 0);
	RUN(entry7.available = false, -ENODEV, 0, 0);
	RUN(entry7.compatible = "apple,other", -ENODEV, 0, 0);
	RUN(entry7.no_ack = true, -ENODEV, 0, 0);
	RUN(entry17.reg = 7, -ENODEV, 0, 0);
	RUN(entry17.reg = 7; entry17.available = false, -ENODEV, 0, 0);
#else
	RUN((void)0, -ENODEV, 0, 0);
	assert(!find_calls);
#endif
	/* A T8122 supplier never takes the T6030 report. */
	RUN(machine = "apple,t8122";
	    supplier_node.compatible = "apple,t8122-pmp-v2", -ENODEV, 0, 0);
#if CONFIG_APPLE_T8122_GPU_PMP_LINK
	/* The T8122 14.x wait: one available report linked to this PMP, DISP (reg 7) acked. */
	RUN(t8122(), 0, 0, 1);
	RUN(t8122(); report14.child = &entry7; entry7.sibling = &entry17;
	    entry17.sibling = NULL, 0, 0, 1);
	RUN(t8122(); apple_pmp_starts.first = NULL, -ENODEV, 0, 0);
	RUN(t8122(); apple_pmp_starts.first = NULL;
	    apple_pmp_reports.first = NULL, -EPROBE_DEFER, 0, 0);
	RUN(t8122(); managed_start.stopping = true, -ENODEV, 0, 0);
	RUN(t8122(); ready_result = -ETIMEDOUT, -ETIMEDOUT, 0, 0);
	RUN(t8122(); passive_words[apple_pmp_offsets_t6030.status / 8] = 0, -EIO,
	    0, 0);
	RUN(t8122(); passive_words[apple_pmp_offsets_t6030.actual / 8] = 0, -EIO,
	    0, 0);
	RUN(t8122(); power_result = -EIO, -EIO, 0, 1);
	RUN(t8122(); machine = "apple,t6030", -ENODEV, 0, 0);
	RUN(t8122(); machine = "apple,t8112", -ENODEV, 0, 0);
	RUN(t8122(); supplier_node.compatible = "apple,t6030-pmp-v2", -ENODEV, 0,
	    0);
	RUN(t8122(); report14.compatible = NULL, -ENODEV, 0, 0);
	RUN(t8122(); report14.compatible = "apple,t6030-pmp-v2-report", -ENODEV,
	    0, 0);
	RUN(t8122(); extra_report = report14, -ENODEV, 0, 0);
	RUN(t8122(); extra_report = report14; extra_report.available = false,
	    -ENODEV, 0, 0);
	RUN(t8122(); report14.available = false, -ENODEV, 0, 0);
	RUN(t8122(); report14.has_pmp = false, -ENODEV, 0, 0);
	RUN(t8122(); report14.pmp_cells = 0, -ENODEV, 0, 0);
	RUN(t8122(); report14.pmp_cells = 2, -ENODEV, 0, 0);
	RUN(t8122(); report14.pmp = NULL, -ENODEV, 0, 0);
	RUN(t8122(); report14.pmp = &other_node, -ENODEV, 0, 0);
	RUN(t8122(); entry17.sibling = NULL, -ENODEV, 0, 0);
	RUN(t8122(); entry7.reg = 8, -ENODEV, 0, 0);
	RUN(t8122(); entry7.available = false, -ENODEV, 0, 0);
	RUN(t8122(); entry7.compatible = "apple,other", -ENODEV, 0, 0);
	RUN(t8122(); entry7.no_ack = true, -ENODEV, 0, 0);
	RUN(t8122(); entry17.reg = 7, -ENODEV, 0, 0);
#else
	/* Without APPLE_T8122_GPU_PMP_LINK, even a well-formed T8122 graph is refused. */
	RUN(t8122(), -ENODEV, 0, 0);
#endif
	/* A declared report, including malformed/unresolved data, never falls back. */
	RUN(supplier_node.has_report = true;
	    supplier_node.report_cells = 0, -ENODEV, 0, 0);
	RUN(supplier_node.has_report = true;
	    supplier_node.report_cells = 1, -ENODEV, 0, 0);
	RUN(supplier_node.has_report = true; supplier_node.report_cells = 1;
	    supplier_node.report = &report14, -ENODEV, 0, 0);
	/* Native25 dispatch is untouched, even on another machine/config. */
	RUN(supplier_node.has_report = true; supplier_node.report_cells = 1;
	    supplier_node.report = &report25; machine = "apple,t8122", 0, 0, 1);
	assert(poll_calls == 1);
	/* Preserve the existing native25 first-phandle behaviour as well. */
	RUN(supplier_node.has_report = true; supplier_node.report_cells = 2;
	    supplier_node.report = &report25, 0, 0, 1);
	RUN(supplier_node.has_report = true; supplier_node.report_cells = 1;
	    supplier_node.report = &report25;
	    present_device = 0, -EPROBE_DEFER, 0, 0);
	RUN(supplier_node.has_report = true; supplier_node.report_cells = 1;
	    supplier_node.report = &report25;
	    lockable = 0, -EPROBE_DEFER, 0, 0);
	RUN(supplier_node.has_report = true; supplier_node.report_cells = 1;
	    supplier_node.report = &report25;
	    native_report.dev.bound = false, -EPROBE_DEFER, 0, 0);
	RUN(supplier_node.has_report = true; supplier_node.report_cells = 1;
	    supplier_node.report = &report25;
	    status_word = 0, -ETIMEDOUT, 0, 0);
	RUN(pmp_data = NULL, -ENODEV, 0, 0);
	/* GPU phandle changes are scoped, preserve existing good links, and fail closed. */
	for (int soc = 0; soc < 3; soc++) {
		struct of_changeset cs;
		reset();
		gate_soc = soc == 0 ? &gate_t8122 :
			   soc == 1 ? &gate_t6030 :
				      &other_soc;
		gpu.compatible = soc == 0 ? "apple,agx-t8122" :
					    "apple,agx-t6030";
		int active = soc == 0 ||
			     (soc == 1 && CONFIG_APPLE_T6030_GPU_PMP_LINK);
		assert(gate_gpu_pmp_link(&cs, &supplier_node) == 0 &&
		       add_calls == active);
		balanced();
		case_count++;
		add_calls = 0;
		gpu.has_pmp = true;
		gpu.pmp_cells = 1;
		gpu.pmp = &supplier_node;
		assert(gate_gpu_pmp_link(&cs, &supplier_node) == 0 &&
		       !add_calls);
		balanced();
		case_count++;
		gpu.pmp = &other_node;
		assert(gate_gpu_pmp_link(&cs, &supplier_node) ==
		       (active ? -EINVAL : 0));
		balanced();
		case_count++;
	}
	reset();
	struct of_changeset cs;
#if CONFIG_APPLE_T6030_GPU_PMP_LINK
	gpu.has_pmp = true;
	gpu.pmp_cells = 2;
	gpu.pmp = &supplier_node;
	assert(gate_gpu_pmp_link(&cs, &supplier_node) == -EINVAL && !add_calls);
	balanced();
	case_count++;
	reset();
	gpu.has_pmp = true;
	gpu.pmp_cells = 1;
	assert(gate_gpu_pmp_link(&cs, &supplier_node) == -EINVAL && !add_calls);
	balanced();
	case_count++;
	reset();
	extra_gpu = gpu;
	assert(gate_gpu_pmp_link(&cs, &supplier_node) == 0 && !add_calls);
	balanced();
	case_count++;
	reset();
	supplier_node.phandle = 0;
	assert(gate_gpu_pmp_link(&cs, &supplier_node) == -EINVAL && !add_calls);
	balanced();
	case_count++;
	reset();
	assert(gate_gpu_pmp_link(&cs, NULL) == -EINVAL && !add_calls);
	balanced();
	case_count++;
	reset();
	add_result = -ENOMEM;
	assert(gate_gpu_pmp_link(&cs, &supplier_node) == -ENOMEM &&
	       add_calls == 1);
	balanced();
	case_count++;
	reset();
	gpu.available = false;
	assert(gate_gpu_pmp_link(&cs, &supplier_node) == 0 && !add_calls);
	balanced();
	case_count++;
	reset();
	gpu.compatible = NULL;
	assert(gate_gpu_pmp_link(&cs, &supplier_node) == 0 && !add_calls);
	balanced();
	case_count++;
#else
	assert(gate_gpu_pmp_link(&cs, &supplier_node) == 0 && !find_calls);
	balanced();
	case_count++;
#endif
	/* Existing T8122 linking keeps its original first-phandle semantics. */
	reset();
	gate_soc = &gate_t8122;
	gpu.compatible = "apple,agx-t8122";
	gpu.has_pmp = true;
	gpu.pmp_cells = 2;
	gpu.pmp = &supplier_node;
	assert(gate_gpu_pmp_link(&cs, &supplier_node) == 0 && !add_calls);
	balanced();
	case_count++;
	/* Actual existing consumer link uses the exact supplier and balances refs. */
	reset();
	gpu.has_pmp = true;
	gpu.pmp_cells = 1;
	gpu.pmp = &supplier_node;
	assert(apple_pmp_link_device(&consumer) == 0 && link_calls == 1);
	balanced();
	case_count++;
	gpu.pmp = &other_node;
	assert(apple_pmp_link_device(&consumer) == -EPROBE_DEFER &&
	       link_calls == 1);
	balanced();
	case_count++;
	gpu.pmp = &supplier_node;
	supplier.bound = false;
	assert(apple_pmp_link_device(&consumer) == -EPROBE_DEFER);
	balanced();
	case_count++;
	supplier.bound = true;
	link_ok = 0;
	assert(apple_pmp_link_device(&consumer) == -EINVAL);
	balanced();
	case_count++;
	printf("PASS: %d T6030/T8122 PMP cases, config=%d/%d; reference, lock and command boundaries checked\n",
	       case_count, CONFIG_APPLE_T6030_GPU_PMP_LINK,
	       CONFIG_APPLE_T8122_GPU_PMP_LINK);
	return 0;
}
