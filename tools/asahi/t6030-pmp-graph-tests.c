// SPDX-License-Identifier: GPL-2.0-only
/* Typed host boundaries for the actual T6030 PMP probe/start/consumer graph. */
#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
#define BIT_ULL(n) (1ULL << (n))
#define __ffs64(v) __builtin_ctzll(v)
#define BIT(n) (1U << (n))
#define IS_ENABLED(x) (x)
#define CONFIG_APPLE_T6030_GPU_PMP_LINK 1
#define CONFIG_APPLE_T8122_GPU_PMP_LINK 0
#define PMP_REPORT_READY 1
#define GFP_KERNEL 0
#define GENPD_FLAG_ACTIVE_WAKEUP 1
#define MAX_PHANDLE_ARGS 4
#define EPROBE_DEFER 517
#define SZ_4K 4096
#define __iomem
#define __force
#define container_of(p, t, m) ((t *)((char *)(p) - offsetof(t, m)))
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, v) ((x) = (v))
#define EXPORT_SYMBOL_GPL(x)
#define IS_ERR(p) false
#define PTR_ERR(p) (-ENOMEM)
static void host_log(const char *format, ...)
{
	(void)format;
}
#define dev_err(dev, ...) ((void)(dev), host_log(__VA_ARGS__))
#define dev_info dev_err
#define dev_warn dev_err
#define dev_err_probe(dev, err, ...) ((void)(dev), (err))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
struct device_node {
	const char *name, *compatible;
	bool available, always_on, no_ack, deferred, uuid, soc_record, release_sleep;
	u32 reg;
	int refs;
	struct device_node *parent, *child, *sibling, *pmp;
};
struct device {
	struct device_node *of_node;
	struct device *parent;
	void *data;
};
struct platform_device {
	struct device dev;
};
struct generic_pm_domain {
	char *name;
	int flags;
	int (*power_on)(struct generic_pm_domain *),
		(*power_off)(struct generic_pm_domain *);
};
struct apple_pmp_report_offsets {
	u32 tgt_read, tgt_write, actual, status, fast_die, fast_die_lanes;
	bool starts_pmp;
	const char *name, *fallback, *pwrstate;
	bool devices_ordered;
};
struct list_head {
	struct list_head *next, *prev;
};
#define INIT_LIST_HEAD(h) ((h)->next = (h), (h)->prev = (h))
static void list_add_tail(struct list_head *n, struct list_head *h)
{
	n->prev = h->prev;
	n->next = h;
	h->prev->next = n;
	h->prev = n;
}
static void list_del(struct list_head *n)
{
	n->prev->next = n->next;
	n->next->prev = n->prev;
	INIT_LIST_HEAD(n);
}
#define list_add_tail_rcu list_add_tail
#define list_for_each_entry(p, h, m)                               \
	for (struct list_head *cursor = (h)->next;                 \
	     cursor != (h) &&                                      \
	     ((p) = container_of(cursor, __typeof__(*(p)), m), 1); \
	     cursor = cursor->next)
#define list_for_each_entry_rcu list_for_each_entry
struct apple_pmp_start;
struct apple_pmp_report {
	struct device *dev;
	const struct apple_pmp_report_offsets *offsets;
	void *base;
	int lock;
	struct list_head list;
	void *debugfs;
	struct apple_pmp_start *start;
};
struct completion {
	bool done;
};
struct work_struct {
	void (*func)(struct work_struct *);
	bool pending;
};
typedef struct {
	u8 b[16];
} uuid_t;
struct apple_pmp_start_ops {
	int (*check)(struct apple_pmp_start *, uuid_t *);
	int (*enable)(struct apple_pmp_start *);
	bool (*bound)(struct apple_pmp_start *);
};
struct apple_pmp_start_timing {
	unsigned int bind, quiet, ready, ack;
};
struct apple_pmp_start {
	struct apple_pmp_report *rep;
	struct list_head list;
	struct device_node *pmp;
	u64 seed, ack, startup_ack, record_seed;
	const struct apple_pmp_start_ops *ops;
	const struct apple_pmp_start_timing *timing;
	struct work_struct work;
	bool queued, enabled;
	struct completion started;
	int result;
	bool stopping;
	unsigned int waiters;
	struct completion waiters_done;
};
struct apple_pmp_report_entry {
	struct generic_pm_domain genpd;
	struct device_node *node, *report_node;
	u32 id;
	bool no_ack, removed;
	bool sleep_release, released;
};
#define genpd_to_apple_pmp_report_entry(p) \
	container_of(p, struct apple_pmp_report_entry, genpd)
struct of_phandle_iterator {
	struct device_node *node;
};
struct of_phandle_args {
	struct device_node *np;
	int args_count, args[MAX_PHANDLE_ARGS];
};
#define of_for_each_phandle(it, ret, node, ...) for ((ret) = 0; false;)
static int of_phandle_iterator_args(struct of_phandle_iterator *it, int *a,
				    int max)
{
	(void)it;
	(void)a;
	(void)max;
	return 0;
}
static int of_genpd_add_subdomain(struct of_phandle_args *p,
				  struct of_phandle_args *c)
{
	(void)p;
	(void)c;
	return 0;
}
static int pm_genpd_init(struct generic_pm_domain *pd, void *gov, bool off)
{
	(void)pd;
	(void)gov;
	(void)off;
	return 0;
}
static int pm_genpd_remove(struct generic_pm_domain *pd)
{
	(void)pd;
	return 0;
}
static void pm_genpd_remove_device(struct device *d)
{
	(void)d;
}
static int of_genpd_add_provider_simple(struct device_node *n,
					struct generic_pm_domain *pd)
{
	(void)n;
	(void)pd;
	return 0;
}
static void of_genpd_del_provider(struct device_node *n)
{
	(void)n;
}
static int apple_pmp_start_mutex, apple_pmp_reports_lock;
struct lock_guard {
	int *lock;
	bool once;
};
static struct lock_guard lock_take(int *l)
{
	assert(!*l);
	*l = 1;
	return (struct lock_guard){ l, true };
}
static void lock_drop(struct lock_guard *l)
{
	assert(*l->lock);
	*l->lock = 0;
}
#define guard(kind)                   \
	struct lock_guard local_guard \
		__attribute__((cleanup(lock_drop))) = lock_take
#define scoped_guard(kind, lock)                                    \
	for (struct lock_guard scoped                               \
	     __attribute__((cleanup(lock_drop))) = lock_take(lock); \
	     scoped.once; scoped.once = false)
static void mutex_lock(int *l)
{
	assert(!*l);
	*l = 1;
}
static void mutex_unlock(int *l)
{
	assert(*l);
	*l = 0;
}
#define spin_lock_init(l) (*(l) = 0)
#define spin_lock_irqsave(l, f) ((f) = 0, mutex_lock(l))
#define spin_unlock_irqrestore(l, f) ((void)(f), mutex_unlock(l))
static void rcu_read_lock(void)
{
}
static void rcu_read_unlock(void)
{
}
static struct list_head apple_pmp_starts, apple_pmp_reports;
static struct device_node report_node, pmp_node, entries[4];
static struct platform_device report_dev, entry_devs[4];
static u64 regs[0x14000 / 8];
static u8 ranges[64], devices[17 * 0x7c];
static struct work_struct *pending;
static unsigned long jiffies;
static unsigned int ack_polls;
#define msecs_to_jiffies(v) (v)
#define time_before(a, b) ((long)((a) - (b)) < 0)
#define time_after(a, b) time_before(b, a)
static void msleep(unsigned int n)
{
	jiffies += n ? n : 1;
}
static void init_completion(struct completion *c)
{
	c->done = false;
}
static void complete_all(struct completion *c)
{
	c->done = true;
}
#define complete complete_all
#define INIT_WORK(w, f) ((w)->func = (f), (w)->pending = false)
#define system_unbound_wq NULL
static bool queue_work(void *q, struct work_struct *w)
{
	assert(!q && !pending);
	pending = w;
	w->pending = true;
	return true;
}
static unsigned long wait_for_completion_timeout(struct completion *c,
						 unsigned long n)
{
	assert(!apple_pmp_start_mutex);
	if (pending) {
		struct work_struct *w = pending;
		pending = NULL;
		w->pending = false;
		w->func(w);
	}
	jiffies++;
	return c->done ? n : 0;
}
static void wait_for_completion(struct completion *c)
{
	assert(c->done);
}
static void cancel_work_sync(struct work_struct *w)
{
	if (pending == w)
		pending = NULL;
	w->pending = false;
}
static u64 readq(const void *a)
{
	return *(const u64 *)a;
}
/* The sleep cases: the PMP acknowledges the requests it is given, ack_late writes late. */
static bool ack_follows;
static unsigned int ack_late;
static void writeq(u64 v, void *a)
{
	assert(a == (char *)regs + 0x108c0);
	*(u64 *)a = v;
	regs[0x1180 / 8] = v;
	if (ack_follows && ack_late)
		ack_late--;
	else if (ack_follows)
		regs[0x11c0 / 8] = v;
}
#define readq_poll_timeout(address, value, condition, delay, timeout) \
	({                                                            \
		int r = -ETIMEDOUT;                                   \
		ack_polls++;                                          \
		for (int i = 0; i < 4; i++) {                         \
			(value) = readq(address);                     \
			if (condition) {                              \
				r = 0;                                \
				break;                                \
			}                                             \
		}                                                     \
		(void)(delay);                                        \
		(void)(timeout);                                      \
		r;                                                    \
	})
#define readq_poll_timeout_atomic readq_poll_timeout
static u32 get_unaligned_le32(const void *p)
{
	u32 v;
	memcpy(&v, p, 4);
	return v;
}
static void put32(u8 *p, u32 v)
{
	memcpy(p, &v, 4);
}
static struct device_node *of_node_get(struct device_node *n)
{
	if (n)
		n->refs++;
	return n;
}
static void of_node_put(struct device_node *n)
{
	if (n) {
		assert(n->refs > 0);
		n->refs--;
	}
}
static bool of_device_is_available(const struct device_node *n)
{
	return n && n->available;
}
static bool of_device_is_compatible(const struct device_node *n, const char *s)
{
	return n && n->compatible && !strcmp(n->compatible, s);
}
static bool of_machine_is_compatible(const char *s)
{
	return !strcmp(s, "apple,t6030");
}
static bool of_property_read_bool(const struct device_node *n, const char *p)
{
	if (!strcmp(p, "apple,always-on"))
		return n->always_on;
	if (!strcmp(p, "apple,no-ack"))
		return n->no_ack;
	if (!strcmp(p, "apple,defer-startup-ack"))
		return n->deferred;
	if (!strcmp(p, "apple,soc-device-record"))
		return n->soc_record;
	if (!strcmp(p, "apple,release-for-sleep"))
		return n->release_sleep;
	return false;
}
static bool of_property_present(const struct device_node *n, const char *p)
{
	if (!strcmp(p, "apple,tunable-uuid"))
		return n->uuid;
	if (!strcmp(p, "apple,pmp"))
		return !!n->pmp;
	return false;
}
static int of_property_read_u32(const struct device_node *n, const char *p,
				u32 *v)
{
	assert(!strcmp(p, "reg"));
	*v = n->reg;
	return 0;
}
static int of_property_count_u32_elems(const struct device_node *n,
				       const char *p)
{
	if (!strcmp(p, "reg"))
		return 1;
	assert(!strcmp(p, "apple,pmp"));
	return n->pmp ? 1 : -EINVAL;
}
static int of_property_read_string(const struct device_node *n, const char *p,
				   const char **v)
{
	assert(!strcmp(p, "label"));
	*v = n->name;
	return 0;
}
static const void *of_get_property(const struct device_node *n, const char *p,
				   int *len)
{
	assert(n == &pmp_node);
	if (!strcmp(p, "apple,tunable-ptd-range")) {
		*len = sizeof(ranges);
		return ranges;
	}
	assert(!strcmp(p, "apple,tunable-soc-device"));
	*len = sizeof(devices);
	return devices;
}
static struct device_node *of_parse_phandle(const struct device_node *n,
					    const char *p, int i)
{
	assert(!i);
	if (!strcmp(p, "apple,pmp"))
		return of_node_get(n->pmp);
	assert(!strcmp(p, "apple,pmp-report"));
	return NULL;
}
static struct device_node *of_get_parent(struct device_node *n)
{
	return of_node_get(n->parent);
}
static struct device_node *next_child(struct device_node *p,
				      struct device_node *old, bool available)
{
	struct device_node *n = old ? old->sibling : p->child;
	of_node_put(old);
	while (n && available && !n->available)
		n = n->sibling;
	return of_node_get(n);
}
#define for_each_child_of_node(p, n)                \
	for ((n) = next_child(p, NULL, false); (n); \
	     (n) = next_child(p, n, false))
#define for_each_available_child_of_node(p, n) \
	for ((n) = next_child(p, NULL, true); (n); (n) = next_child(p, n, true))
static struct device_node *find_report(struct device_node *old, const char *c)
{
	of_node_put(old);
	return !old && of_device_is_compatible(&report_node, c) ?
		       of_node_get(&report_node) :
		       NULL;
}
#define for_each_compatible_node(n, t, c) \
	for ((n) = find_report(NULL, c); (n); (n) = find_report(n, c))
static void *dev_get_drvdata(struct device *d)
{
	return d->data;
}
static void platform_set_drvdata(struct platform_device *p, void *v)
{
	p->dev.data = v;
}
static void *platform_get_drvdata(struct platform_device *p)
{
	return p->dev.data;
}
static bool device_is_bound(struct device *d)
{
	(void)d;
	return false;
}
static bool device_trylock(struct device *d)
{
	(void)d;
	return false;
}
static void device_unlock(struct device *d)
{
	(void)d;
}
static void put_device(struct device *d)
{
	(void)d;
}
static struct platform_device *of_find_device_by_node(struct device_node *n)
{
	(void)n;
	return NULL;
}
struct action {
	void (*f)(void *);
	void *p;
};
static struct action actions[8];
static unsigned int action_count;
static void *allocations[8];
static unsigned int allocation_count;
static void *devm_kzalloc(struct device *d, size_t n, int g)
{
	(void)d;
	(void)g;
	void *v = calloc(1, n);
	assert(v);
	allocations[allocation_count++] = v;
	return v;
}
#define kzalloc_obj(v) calloc(1, sizeof(v))
#define kfree free
static char *kstrdup(const char *s, int g)
{
	(void)g;
	return strdup(s);
}
static int devm_add_action_or_reset(struct device *d, void (*f)(void *),
				    void *p)
{
	(void)d;
	assert(action_count < 8);
	actions[action_count++] = (struct action){ f, p };
	return 0;
}
static void *devm_platform_ioremap_resource(struct platform_device *p, int i)
{
	assert(p == &report_dev && !i);
	return regs;
}
static const struct apple_pmp_report_offsets *
of_device_get_match_data(struct device *d);
static void apple_pmp_report_release(void *p)
{
	struct apple_pmp_report *r = p;
	list_del(&r->list);
}
static void *apple_pmp_report_debugfs;
static int apple_pmp_report_status_fops;
static const char *dev_name(struct device *d)
{
	(void)d;
	return "host-report";
}
static void *debugfs_create_file(const char *n, int mode, void *dir, void *data,
				 void *ops)
{
	(void)n;
	(void)mode;
	(void)dir;
	(void)data;
	(void)ops;
	return NULL;
}
static int of_platform_populate(struct device_node *n, void *a, void *b,
				struct device *d);
static int check_error, enable_error;
static bool hw_binds, hw_ready;
static u64 hw_ack;
static unsigned int enables, commands;
static int hw_check(struct apple_pmp_start *s, uuid_t *u)
{
	(void)s;
	memset(u, 0, sizeof(*u));
	return check_error;
}
static int hw_enable(struct apple_pmp_start *s)
{
	assert(!pmp_node.available);
	enables++;
	if (!enable_error)
		pmp_node.available = true;
	(void)s;
	return enable_error;
}
static bool hw_bound(struct apple_pmp_start *s)
{
	if (!hw_binds)
		return false;
	regs[s->rep->offsets->status / 8] = hw_ready ? 1 : 0;
	regs[s->rep->offsets->actual / 8] = hw_ack;
	return true;
}
static const struct apple_pmp_start_ops apple_pmp_start_hw_ops = { hw_check,
								   hw_enable,
								   hw_bound };
static const struct apple_pmp_start_timing apple_pmp_start_timing = { 2, 0, 2,
								      2 };
#include "production.h"
static const struct apple_pmp_report_offsets *
of_device_get_match_data(struct device *d)
{
	assert(d == &report_dev.dev);
	return &apple_pmp_offsets_t6030;
}
static int of_platform_populate(struct device_node *n, void *a, void *b,
				struct device *d)
{
	assert(n == &report_node && !a && !b && d == &report_dev.dev);
	/* Actual child probe seeds each request while PMP status is still halted. */
	for (unsigned int i = 0; i < 4; i++) {
		int r = apple_pmp_report_entry_probe(&entry_devs[i]);
		if (r)
			return r;
	}
	return 0;
}
struct overlay_entry {
	u32 reg;
	bool always_on, no_ack, deferred;
};
#include "overlay.h"
static void cleanup(void)
{
	for (unsigned int i = 0; i < 4; i++) {
		struct apple_pmp_report_entry *e = entry_devs[i].dev.data;
		if (e)
			apple_pmp_report_entry_free(e);
		entry_devs[i].dev.data = NULL;
	}
	while (action_count) {
		struct action a = actions[--action_count];
		a.f(a.p);
	}
	for (unsigned int i = 0; i < allocation_count; i++)
		free(allocations[i]);
	allocation_count = 0;
	assert(!report_node.refs && !pmp_node.refs);
	for (unsigned int i = 0; i < 4; i++)
		assert(!entries[i].refs);
	assert(!apple_pmp_start_mutex && !apple_pmp_reports_lock && !pending);
}
static struct apple_pmp_report *setup(void)
{
	INIT_LIST_HEAD(&apple_pmp_starts);
	INIT_LIST_HEAD(&apple_pmp_reports);
	memset(regs, 0, sizeof(regs));
	memset(ranges, 0, sizeof(ranges));
	memset(devices, 0, sizeof(devices));
	report_node =
		(struct device_node){ .name = "report",
				      .compatible = "apple,t6030-pmp-v2-report",
				      .available = true,
				      .child = &entries[0],
				      .pmp = &pmp_node };
	pmp_node = (struct device_node){ .name = "PMP",
					 .compatible = "apple,t6000-pmp-v2",
					 .uuid = true };
	for (unsigned int i = 0; i < 4; i++) {
		const struct overlay_entry *o = &overlay_entries[i];
		entries[i] = (struct device_node){
			.name = "entry",
			.compatible = "apple,t6000-pmp-v2-report-entry",
			.available = true,
			.reg = o->reg,
			.always_on = o->always_on,
			.no_ack = o->no_ack,
			.deferred = o->deferred,
			.parent = &report_node,
			.sibling = i < 3 ? &entries[i + 1] : NULL
		};
		entry_devs[i] = (struct platform_device){
			.dev = { .of_node = &entries[i],
				 .parent = &report_dev.dev }
		};
	}
	report_dev =
		(struct platform_device){ .dev = { .of_node = &report_node } };
	put32(ranges, 10);
	put32(ranges + 4, 0x118);
	put32(ranges + 8, 4);
	put32(ranges + 32, 11);
	put32(ranges + 36, 0x11c);
	put32(ranges + 40, 4);
	for (unsigned int i = 0; i < 17; i++) {
		put32(devices + i * 0x7c, i + 1);
		if (i >= 7 && i <= 9)
			put32(devices + i * 0x7c + 8, 2);
	}
	ack_polls = 0;
	ack_follows = false;
	ack_late = 0;
	check_error = enable_error = 0;
	hw_binds = hw_ready = true;
	hw_ack = BIT_ULL(7);
	enables = commands = 0;
	return NULL;
}
static int g15_vote(void)
{
	struct device supplier = { .of_node = &pmp_node };
	/* Scoped supplier has its first, more specific compatible as on the real DT. */
	pmp_node.compatible = "apple,t6030-pmp-v2";
	int ret = apple_pmp_report_wait_supplier_ready(&supplier, 5000);
	if (!ret)
		commands++;
	return ret;
}
int main(void)
{
	unsigned int cases = 0;
	(void)apple_pmp_start_init;
	(void)apple_pmp_report_wait_t6030_14;
	setup();
	assert(apple_pmp_report_probe(&report_dev) == 0);
	struct apple_pmp_report *r = report_dev.dev.data;
	/* The old passive descriptor fails the real DCP entry waiter here. */
	if (!r->start)
		assert(apple_pmp_report_wait_ready(&entries[0], 30000) == 0);
	assert(r->start);
	assert(r->start->seed ==
	       (BIT_ULL(7) | BIT_ULL(8) | BIT_ULL(9) | BIT_ULL(16)));
	assert(r->start->ack == (BIT_ULL(7) | BIT_ULL(8) | BIT_ULL(9)) &&
	       r->start->startup_ack == BIT_ULL(7));
	assert(!pmp_node.available && regs[0x10 / 8] == 0 && enables == 0);
	assert(regs[0x1180 / 8] == r->start->seed);
	assert(((struct apple_pmp_report_entry *)entry_devs[3].dev.data)
		       ->no_ack);
	assert(apple_pmp_report_wait_ready(&entries[0], 30000) == 0 &&
	       enables == 1 && pmp_node.available);
	assert(g15_vote() == 0 && commands == 1 && enables == 1);
	cases++;
	assert(apple_pmp_report_wait_ready(&entries[1], 3000) == -EIO);
	cases++;
	regs[0x11c0 / 8] |= BIT_ULL(8);
	assert(apple_pmp_report_wait_ready(&entries[1], 3000) == 0);
	cases++;
	assert(apple_pmp_report_wait_ready(&entries[3], 3000) == -EINVAL);
	cases++;
	unsigned int polls = ack_polls;
	struct apple_pmp_report_entry *storage = entry_devs[3].dev.data;
	assert(apple_pmp_report_entry_power_on(&storage->genpd) == 0 &&
	       ack_polls == polls);
	cases++;
	regs[0x11c0 / 8] &= ~BIT_ULL(7);
	assert(g15_vote() == -EIO && commands == 1);
	cases++;
	cleanup();
	/* Missing display acknowledgement cannot release G15, even if bound+READY. */
	setup();
	assert(apple_pmp_report_probe(&report_dev) == 0);
	hw_ack = BIT_ULL(8);
	assert(apple_pmp_report_wait_ready(&entries[1], 3000) == -EIO);
	cases++;
	assert(g15_vote() == -EIO && commands == 0 && enables == 1);
	cases++;
	cleanup();
	setup();
	assert(apple_pmp_report_probe(&report_dev) == 0);
	hw_ready = false;
	assert(g15_vote() == -ETIMEDOUT && commands == 0);
	cases++;
	cleanup();
	setup();
	assert(apple_pmp_report_probe(&report_dev) == 0);
	hw_binds = false;
	assert(g15_vote() == -ETIMEDOUT && commands == 0);
	cases++;
	cleanup();
	setup();
	assert(apple_pmp_report_probe(&report_dev) == 0);
	check_error = -EIO;
	assert(g15_vote() == -EIO && !enables && !pmp_node.available);
	cases++;
	cleanup();
	setup();
	assert(apple_pmp_report_probe(&report_dev) == 0);
	enable_error = -EINVAL;
	assert(g15_vote() == -EINVAL && enables == 1 && !pmp_node.available);
	cases++;
	cleanup();
	setup();
	assert(apple_pmp_report_probe(&report_dev) == 0);
	regs[0x1180 / 8] = 0;
	assert(g15_vote() == -EBUSY && !enables);
	cases++;
	cleanup();
	setup();
	assert(apple_pmp_report_probe(&report_dev) == 0);
	regs[0x10 / 8] = 1;
	assert(g15_vote() == -EBUSY && !enables);
	cases++;
	cleanup();
	/* No valid handoff means no startup owner; a bound report isn't enough. */
	setup();
	pmp_node.uuid = false;
	assert(apple_pmp_report_probe(&report_dev) == 0);
	assert(!((struct apple_pmp_report *)report_dev.dev.data)->start);
	assert(g15_vote() == -ENODEV && !enables);
	cases++;
	cleanup();
	/* A drained managed owner is unavailable even while the report stays bound. */
	setup();
	assert(apple_pmp_report_probe(&report_dev) == 0);
	r = report_dev.dev.data;
	assert(action_count == 2 && actions[1].p == r->start);
	action_count--;
	apple_pmp_start_drain(r->start);
	assert(g15_vote() == -ENODEV && commands == 0 && !enables);
	cases++;
	cleanup();
	/* Actual describe rejects malformed handoff layouts before any enable. */
	setup();
	put32(ranges + 4, 0x100);
	assert(apple_pmp_report_probe(&report_dev) == 0);
	assert(g15_vote() == -EINVAL && commands == 0 && !enables);
	cases++;
	cleanup();
	setup();
	put32(devices + 16 * 0x7c + 8, 2);
	assert(apple_pmp_report_probe(&report_dev) == 0);
	assert(g15_vote() == -EINVAL && commands == 0 && !enables);
	cases++;
	cleanup();
#ifdef HAVE_SLEEP_RELEASE
	/* System sleep withdraws an acknowledged request marked for it, then asserts it again. */
	setup();
	entries[1].release_sleep = true;
	assert(apple_pmp_report_probe(&report_dev) == 0);
	assert(apple_pmp_report_wait_ready(&entries[0], 30000) == 0);
	struct apple_pmp_report_entry *disp = entry_devs[0].dev.data;
	struct apple_pmp_report_entry *ext = entry_devs[1].dev.data;

	assert(ext->sleep_release && !disp->sleep_release);
	ack_follows = true;
	regs[0x11c0 / 8] = regs[0x1180 / 8];
	u64 target = regs[0x1180 / 8];

	assert(target & BIT_ULL(8));
	assert(apple_pmp_report_entry_suspend_noirq(&entry_devs[0].dev) == 0 &&
	       !disp->released && regs[0x1180 / 8] == target);
	assert(apple_pmp_report_entry_suspend_noirq(&entry_devs[1].dev) == 0);
	assert(ext->released && regs[0x1180 / 8] == (target & ~BIT_ULL(8)));
	assert(apple_pmp_report_entry_resume_noirq(&entry_devs[1].dev) == 0);
	assert(!ext->released && regs[0x1180 / 8] == target);
	cases++;
	/* A request the PMP has not acknowledged is left as it is. */
	regs[0x11c0 / 8] &= ~BIT_ULL(8);
	assert(apple_pmp_report_entry_suspend_noirq(&entry_devs[1].dev) == 0);
	assert(!ext->released && regs[0x1180 / 8] == target);
	assert(apple_pmp_report_entry_resume_noirq(&entry_devs[1].dev) == 0 &&
	       regs[0x1180 / 8] == target);
	cases++;
	/* A re-assert the PMP acknowledges late is written again, three times at most. */
	regs[0x11c0 / 8] = target;
	assert(apple_pmp_report_entry_suspend_noirq(&entry_devs[1].dev) == 0 && ext->released);
	ack_late = 2;
	assert(apple_pmp_report_entry_resume_noirq(&entry_devs[1].dev) == 0 &&
	       !ack_late && regs[0x11c0 / 8] == target);
	assert(apple_pmp_report_entry_suspend_noirq(&entry_devs[1].dev) == 0 && ext->released);
	ack_late = 5;
	assert(apple_pmp_report_entry_resume_noirq(&entry_devs[1].dev) == -ETIMEDOUT &&
	       ack_late == 2 && regs[0x1180 / 8] == target);
	cases++;
	cleanup();
#endif
	printf("PASS: %u actual overlay/probe/entry/DCP7/managed-start/supplier graph cases\n",
	       cases);
	return 0;
}
