// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMP power state reporting driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/rculist.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/apple/pmp-report.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/uuid.h>
#include <linux/workqueue.h>

#include "pmp-report-validation.h"

#define PMP_REPORT_READY 0x1

/**
 * struct apple_pmp_report_offsets - per-SoC layout of the report region
 * @tgt_read: power state request, read side
 * @tgt_write: power state request, write side
 * @actual: power state acknowledgment
 * @status: PMP status; bit 0 is set while the PMP is running
 * @fast_die: 32-bit word holding the fast die-temperature effort of each
 *	CPU cluster, one byte per lane; 0 if the SoC has none
 * @fast_die_lanes: number of lanes at @fast_die, at most 4
 * @starts_pmp: this driver starts a halted PMP that a display gate hands
 *	over to the report (see apple_pmp_start_init())
 * @name: with @starts_pmp, the SoC in messages
 * @fallback: with @starts_pmp, how to boot without starting the PMP
 * @pwrstate: with @starts_pmp, the compatible of the SoC's PMGR power states
 * @devices_ordered: with @starts_pmp, soc-device record i holds device ID
 *	i + 1
 */
struct apple_pmp_report_offsets {
	u32 tgt_read;
	u32 tgt_write;
	u32 actual;
	u32 status;
	u32 fast_die;
	u32 fast_die_lanes;
	bool starts_pmp;
	const char *name;
	const char *fallback;
	const char *pwrstate;
	bool devices_ordered;
};

struct apple_pmp_start;

struct apple_pmp_report {
	struct device *dev;
	const struct apple_pmp_report_offsets *offsets;
	void __iomem *base;
	spinlock_t lock;
	struct list_head list;
	struct dentry *debugfs;
	/* Set only for a report whose halted PMP this driver starts. */
	struct apple_pmp_start *start;
};

static const struct apple_pmp_report_offsets apple_pmp_offsets_t6030;

/* Bound report regions, protected by RCU until managed teardown drains readers. */
static LIST_HEAD(apple_pmp_reports);
static DEFINE_MUTEX(apple_pmp_reports_lock);
static struct dentry *apple_pmp_report_debugfs;

/**
 * apple_pmp_report_fast_die_effort() - read the fast die-temperature effort
 *	of a CPU cluster
 * @np: device tree node of the report region
 * @lane: lane of the cluster in the fast die-temperature report
 * @effort: set to the effort on success
 *
 * The PMP of some SoCs runs a control loop on its own die-temperature sensors
 * and publishes an 8-bit effort for each CPU cluster: 0 requests no limit,
 * and the effort rises towards 255 as the die exceeds the firmware's target.
 * The effort is only valid while the PMP reports that it is running; a
 * stopped PMP reads 0.
 *
 * Context: Any context.
 * Return: 0 on success, -EPROBE_DEFER if the region is not bound (yet),
 * -EOPNOTSUPP if the SoC's PMP publishes no such effort, -EINVAL if @lane is
 * out of range, or -EAGAIN if the PMP does not report that it is running.
 */
int apple_pmp_report_fast_die_effort(const struct device_node *np,
				     unsigned int lane, u8 *effort)
{
	const struct apple_pmp_report_offsets *offs;
	const struct apple_pmp_report *rep;
	int ret = -EPROBE_DEFER;

	rcu_read_lock();
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->dev->of_node != np)
			continue;

		offs = rep->offsets;
		if (!offs->fast_die) {
			ret = -EOPNOTSUPP;
		} else if (lane >= offs->fast_die_lanes) {
			ret = -EINVAL;
		} else if (!(readl(rep->base + offs->status) & PMP_REPORT_READY)) {
			ret = -EAGAIN;
		} else {
			*effort = readl(rep->base + offs->fast_die) >>
				  (lane * BITS_PER_BYTE);
			ret = 0;
		}
		break;
	}
	rcu_read_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_fast_die_effort);

static void apple_pmp_start_show(struct seq_file *s, struct apple_pmp_start *st);

/* Read only the documented status and fast-die entries, never the aperture. */
static int apple_pmp_report_status_show(struct seq_file *s, void *unused)
{
	struct apple_pmp_report *rep = s->private;
	const struct apple_pmp_report_offsets *offs = rep->offsets;
	u32 status = readl(rep->base + offs->status);
	unsigned int lane;
	u32 effort;

	seq_printf(s, "running: %u\n", !!(status & PMP_REPORT_READY));
	if (rep->start)
		apple_pmp_start_show(s, rep->start);
	if (!(status & PMP_REPORT_READY) || !offs->fast_die)
		return 0;

	effort = readl(rep->base + offs->fast_die);
	for (lane = 0; lane < offs->fast_die_lanes; lane++)
		seq_printf(s, "lane%u: %u\n", lane,
			   (effort >> (lane * BITS_PER_BYTE)) & 0xff);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_pmp_report_status);

static void apple_pmp_report_release(void *data)
{
	struct apple_pmp_report *rep = data;

	/* Withdraw child providers while their power callbacks can still use MMIO. */
	of_platform_depopulate(rep->dev);
	mutex_lock(&apple_pmp_reports_lock);
	if (!list_empty(&rep->list))
		list_del_rcu(&rep->list);
	mutex_unlock(&apple_pmp_reports_lock);
	synchronize_rcu();
	/* The debugfs proxy drains active reads and rejects subsequent accesses. */
	debugfs_remove(rep->debugfs);
}

/* The unique, enabled and acknowledged T6030 display request (DISP, reg 7). */
static struct device_node *apple_pmp_report_t6030_entry(struct device_node *report)
{
	struct device_node *child, *entry = NULL;
	u32 reg;

	for_each_child_of_node(report, child) {
		if (of_property_read_u32(child, "reg", &reg) || reg != 7)
			continue;
		if (entry || of_property_count_u32_elems(child, "reg") != 1 ||
		    !of_device_is_available(child) ||
		    !of_device_is_compatible(child, "apple,t6000-pmp-v2-report-entry") ||
		    of_property_read_bool(child, "apple,no-ack")) {
			of_node_put(child);
			of_node_put(entry);
			return NULL;
		}
		entry = of_node_get(child);
	}
	return entry;
}

/* The existing display domain seeds DISP; require its READY and ACK before AGX. */
static int apple_pmp_report_wait_t6030_14(struct device *supplier, unsigned int timeout_ms)
{
	struct device_node *np, *report = NULL, *pmp, *entry;
	int ret;

	if (!IS_ENABLED(CONFIG_APPLE_T6030_GPU_PMP_LINK) ||
	    !of_machine_is_compatible("apple,t6030") ||
	    !of_device_is_compatible(supplier->of_node, "apple,t6030-pmp-v2"))
		return -ENODEV;
	for_each_compatible_node(np, NULL, "apple,t6030-pmp-v2-report") {
		if (report) {
			of_node_put(np);
			of_node_put(report);
			return -ENODEV;
		}
		report = of_node_get(np);
	}
	if (!report || !of_device_is_available(report) ||
	    of_property_count_u32_elems(report, "apple,pmp") != 1) {
		of_node_put(report);
		return -ENODEV;
	}
	pmp = of_parse_phandle(report, "apple,pmp", 0);
	entry = apple_pmp_report_t6030_entry(report);
	ret = pmp == supplier->of_node && entry ?
		apple_pmp_report_wait_ready(entry, msecs_to_jiffies(timeout_ms)) : -ENODEV;
	of_node_put(entry);
	of_node_put(pmp);
	of_node_put(report);
	return ret;
}

/*
 * The T8122 (M3 MacBook Air) GPU on its 14.8.3 firmware, for the M3 runtime
 * (APPLE_T8122_GPU_PMP_LINK): the unique T8122 report linked to this PMP, and
 * its display request (DISP, reg 7) READY and acknowledged before AGX, as on
 * the 7.1.12 kernel that runtime was qualified with.
 */
static int apple_pmp_report_wait_t8122_14(struct device *supplier, unsigned int timeout_ms)
{
	struct device_node *np, *report = NULL, *pmp, *entry;
	int ret;

	if (!IS_ENABLED(CONFIG_APPLE_T8122_GPU_PMP_LINK) ||
	    !of_machine_is_compatible("apple,t8122") ||
	    !of_device_is_compatible(supplier->of_node, "apple,t8122-pmp-v2"))
		return -ENODEV;
	for_each_compatible_node(np, NULL, "apple,t8122-pmp-v2-report") {
		if (report) {
			of_node_put(np);
			of_node_put(report);
			return -ENODEV;
		}
		report = of_node_get(np);
	}
	if (!report || !of_device_is_available(report) ||
	    of_property_count_u32_elems(report, "apple,pmp") != 1) {
		of_node_put(report);
		return -ENODEV;
	}
	pmp = of_parse_phandle(report, "apple,pmp", 0);
	entry = apple_pmp_report_t6030_entry(report);
	ret = pmp == supplier->of_node && entry ?
		apple_pmp_report_wait_ready(entry, msecs_to_jiffies(timeout_ms)) : -ENODEV;
	of_node_put(entry);
	of_node_put(pmp);
	of_node_put(report);
	return ret;
}

/* The native 25G83 PMP starts through its supplier topology, after requests
 * are seeded. Its report is pinned against removal while readiness is read.
 */
int apple_pmp_report_wait_supplier_ready(struct device *supplier, unsigned int timeout_ms)
{
	struct device_node *np = of_parse_phandle(supplier->of_node, "apple,pmp-report", 0);
	struct platform_device *pdev;
	struct apple_pmp_report *rep;
	u64 status;
	int ret;

	/* A declared report always takes the existing native25 path below. */
	if (!np && !of_property_present(supplier->of_node, "apple,pmp-report")) {
		if (of_machine_is_compatible("apple,t8122"))
			return apple_pmp_report_wait_t8122_14(supplier, timeout_ms);
		return apple_pmp_report_wait_t6030_14(supplier, timeout_ms);
	}
	if (!np || !of_device_is_compatible(np, "apple,j613-25g83-pmp-report")) {
		of_node_put(np);
		return -ENODEV;
	}
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -EPROBE_DEFER;
	/* Never wait behind supplier removal while the command bridge is locked. */
	if (!device_trylock(&pdev->dev)) {
		put_device(&pdev->dev);
		return -EPROBE_DEFER;
	}
	rep = device_is_bound(&pdev->dev) ? platform_get_drvdata(pdev) : NULL;
	if (!rep)
		ret = -EPROBE_DEFER;
	else
		ret = readq_poll_timeout(rep->base + rep->offsets->status, status,
					status == PMP_REPORT_READY, 1000, timeout_ms * 1000);
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_wait_supplier_ready);

/*
 * Starting a halted PMP (T8122, T6031, opted-in T6030)
 *
 * On these SoCs iBoot leaves the PMP loaded but halted, and the DCP firmware
 * of the 14.x system needs it running. A display gate that admits the boot
 * loader's display handoff completes the device tree's PMP topology before
 * any device is created: it copies this Mac's PMP values onto the disabled
 * PMP node (which this report names in apple,pmp), makes the display and
 * storage requests always on, and marks requests that the PTD indexes by
 * soc-device record. Only then does this driver take the start over; on any
 * other boot it behaves exactly as before.
 *
 * The start is the source design of the T6030 bring-up, on this driver's
 * shared T8140 structure:
 *  1. at probe, the requests are taken from the always-on entries and
 *     checked against the PMP tunables (request and acknowledge apertures,
 *     soc-device table);
 *  2. the first apple_pmp_report_wait_ready() caller queues the start;
 *  3. the start waits for the entries to have seeded their requests, then
 *     checks that the display and PMP power states are up, the display CPU
 *     has its floor, the PMP CPU is halted, its SRAM holds the image the
 *     tunables name with valid boot arguments, and the PTD holds exactly the
 *     seeded requests;
 *  4. it enables the PMP node, and its DART and mailbox if they are disabled;
 *     the PMP driver then patches the boot arguments and runs the firmware;
 *  5. it waits for the PMP driver to bind, the PTD to report ready and the
 *     startup requests to be acknowledged.
 * From step 4 on the PMP runs, and only a reboot stops it. That stays so
 * when step 5 fails: nothing stops the PMP, and the display stays on the boot
 * framebuffer. There is no safe stop to take instead. The PMP driver has no
 * shutdown handshake for it, a PMP that did not bind or report ready would
 * not answer one, and one that runs owns the SoC power states: halting its
 * CPU would freeze whatever it set, with nothing left to manage them. A
 * reboot is the recovery, and the SoC's fallback option (@fallback) or
 * nomodeset boots without starting it. See APPLE_PMP_REPORT in Kconfig.
 */
#define PMP_ASC_CPU_CONTROL		0x44
#define PMP_ASC_CPU_RUN			BIT(4)
#define PMP_SRAM_UUID			0x214
#define PMP_SRAM_BOOTARGS_OFFSET	0x22c
#define PMP_SRAM_BOOTARGS_SIZE		0x230

#define APPLE_PMGR_PS_MIN		GENMASK(19, 16)
#define APPLE_PMGR_PS_ACTUAL		GENMASK(7, 4)
#define APPLE_PMGR_PS_ACTIVE		0xf

#define PMP_START_SEED_TIMEOUT_MS	2000
#define PMP_START_BIND_TIMEOUT_MS	20000
#define PMP_START_QUIET_MS		1000
#define PMP_START_READY_TIMEOUT_MS	3000
#define PMP_START_ACK_TIMEOUT_MS	1000

/*
 * The steps of the start that reach beyond the report region: the checks
 * before the PMP may run (step 3), enabling its nodes (step 4) and whether
 * its driver has bound (step 5). The KUnit tests replace them to inject
 * faults; the start reads the report region itself.
 */
struct apple_pmp_start;

struct apple_pmp_start_ops {
	int (*check)(struct apple_pmp_start *st, uuid_t *image);
	int (*enable)(struct apple_pmp_start *st);
	bool (*bound)(struct apple_pmp_start *st);
};

/* Step 5's waits, in milliseconds. */
struct apple_pmp_start_timing {
	unsigned int bind;
	unsigned int quiet;
	unsigned int ready;
	unsigned int ack;
};

static const struct apple_pmp_start_timing apple_pmp_start_timing = {
	.bind = PMP_START_BIND_TIMEOUT_MS,
	.quiet = PMP_START_QUIET_MS,
	.ready = PMP_START_READY_TIMEOUT_MS,
	.ack = PMP_START_ACK_TIMEOUT_MS,
};

struct apple_pmp_start {
	struct apple_pmp_report *rep;
	struct list_head list;
	/* The halted PMP this report starts. */
	struct device_node *pmp;
	/* Requests seeded by always-on entries, and those acknowledged. */
	u64 seed;
	u64 ack;
	/* Requests that must be acknowledged before any consumer can proceed. */
	u64 startup_ack;
	/* Seeded bits that are soc-device record indices. */
	u64 record_seed;
	struct of_changeset pmp_cs;
	const struct apple_pmp_start_ops *ops;
	const struct apple_pmp_start_timing *timing;
	struct work_struct work;
	bool queued;
	/* The PMP nodes were enabled: the PMP may run, until the next reboot. */
	bool enabled;
	struct completion started;
	int result;
	bool stopping;
	unsigned int waiters;
	struct completion waiters_done;
};

/* Reports that start their PMP; serializes startup requests and waiters. */
static LIST_HEAD(apple_pmp_starts);
static DEFINE_MUTEX(apple_pmp_start_mutex);

static const struct {
	const char *label;
	/* The minimum state must already be "active". */
	bool floor;
} apple_pmp_start_domains[] = {
	{ "pms_sram", false },
	{ "pmp", false },
	/*
	 * DISP_SYS and DISP_FE only need to be on: their minimum-state field
	 * does not hold a value (it reads back 0 after "active" is written),
	 * and the PMP leaves them on once the DCP CPU domain has its floor.
	 */
	{ "disp_sys", false },
	{ "disp_fe", false },
	/* Without this floor the PMP stops the running DCP CPU when it starts. */
	{ "disp_cpu", true },
};

/*
 * The PTD request/acknowledge bit of a report entry. An entry's reg is its
 * soc-device ID - 1. An entry marked apple,soc-device-record is requested by
 * soc-device record instead: on T8122 the PTD request and acknowledge bits
 * are indexed by soc-device record (apple,tunable-soc-device), and the
 * records are not in ID order, so the bit is the index of the record that
 * holds that ID. Only a report that starts its PMP has that table.
 */
static int apple_pmp_report_bit(const struct apple_pmp_start *st,
				struct device_node *entry, u32 *bit, bool *record)
{
	const u8 *table;
	int len = 0, index;
	u32 reg;

	if (of_property_read_u32(entry, "reg", &reg) || reg > 63)
		return -EINVAL;
	*record = of_property_read_bool(entry, "apple,soc-device-record");
	if (!*record) {
		*bit = reg;
		return 0;
	}
	if (!st)
		return -EINVAL;
	table = of_get_property(st->pmp, "apple,tunable-soc-device", &len);
	index = apple_pmp_record_of_id(table, len > 0 ? len : 0, reg + 1);
	if (index < 0 || index > 63)
		return -ENOENT;
	*bit = index;
	return 0;
}

/* The single PMGR power state of this SoC with @label. */
static struct device_node *apple_pmp_start_domain(struct apple_pmp_report *rep,
						  const char *label)
{
	struct device_node *np, *found = NULL;
	const char *name;

	for_each_compatible_node(np, NULL, rep->offsets->pwrstate) {
		if (of_property_read_string(np, "label", &name) || strcmp(name, label))
			continue;
		if (found) {
			of_node_put(np);
			of_node_put(found);
			return NULL;
		}
		found = of_node_get(np);
	}

	return found;
}

static int apple_pmp_start_check_power(struct apple_pmp_report *rep)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(apple_pmp_start_domains); i++) {
		const char *label = apple_pmp_start_domains[i].label;
		struct device_node *np = apple_pmp_start_domain(rep, label);
		struct regmap *regmap;
		u32 offset, val;
		int ret;

		if (!np) {
			dev_err(rep->dev, "%s PMP not started: no single %s power state\n",
				rep->offsets->name, label);
			return -ENODEV;
		}
		ret = of_property_read_u32(np, "reg", &offset);
		regmap = syscon_node_to_regmap(np->parent);
		of_node_put(np);
		if (ret)
			return ret;
		if (IS_ERR(regmap))
			return PTR_ERR(regmap);
		ret = regmap_read(regmap, offset, &val);
		if (ret)
			return ret;

		if (FIELD_GET(APPLE_PMGR_PS_ACTUAL, val) != APPLE_PMGR_PS_ACTIVE) {
			dev_err(rep->dev, "%s PMP not started: %s is not powered (%#x)\n",
				rep->offsets->name, label, val);
			return -EIO;
		}
		/* The PMGR power state driver applies the floor when it probes. */
		if (apple_pmp_start_domains[i].floor &&
		    FIELD_GET(APPLE_PMGR_PS_MIN, val) != APPLE_PMGR_PS_ACTIVE) {
			dev_err(rep->dev, "%s PMP not started: the %s minimum state is not active (%#x)\n",
				rep->offsets->name, label, val);
			return -EIO;
		}
	}

	return 0;
}

static void __iomem *apple_pmp_start_map(struct apple_pmp_start *st, const char *name,
					 resource_size_t *size)
{
	struct resource res;
	void __iomem *base;
	int idx;

	idx = of_property_match_string(st->pmp, "reg-names", name);
	if (idx < 0 || of_address_to_resource(st->pmp, idx, &res))
		return NULL;
	/* The PMP driver claims these registers; only read them here. */
	base = ioremap_np(res.start, resource_size(&res));
	if (base && size)
		*size = resource_size(&res);
	return base;
}

/* The PMP CPU, which must still be halted. */
static int apple_pmp_start_check_halted(struct apple_pmp_start *st)
{
	void __iomem *asc = apple_pmp_start_map(st, "asc", NULL);
	u32 control;

	if (!asc)
		return -ENOMEM;
	control = readl(asc + PMP_ASC_CPU_CONTROL);
	iounmap(asc);
	if (control & PMP_ASC_CPU_RUN) {
		dev_err(st->rep->dev, "%s PMP not started: its CPU is already running (%#x)\n",
			st->rep->offsets->name, control);
		return -EBUSY;
	}
	return 0;
}

/* The SRAM must hold the image named by the node's tunables, and its boot arguments. */
static int apple_pmp_start_check_sram(struct apple_pmp_start *st, uuid_t *image)
{
	void __iomem *sram;
	resource_size_t sram_size;
	const char *expected;
	u32 offset, size, pos;
	u8 *args;
	uuid_t want;
	int ret = 0;

	if (of_property_read_string(st->pmp, "apple,tunable-uuid", &expected) ||
	    uuid_parse(expected, &want))
		return -EINVAL;
	sram = apple_pmp_start_map(st, "pmp", &sram_size);
	if (!sram)
		return -ENOMEM;
	if (sram_size < PMP_SRAM_BOOTARGS_SIZE + 4) {
		ret = -EINVAL;
		goto out;
	}
	/* Aligned 32-bit reads only: the SRAM is device memory. */
	for (pos = 0; pos < sizeof(image->b); pos += 4)
		put_unaligned_le32(readl(sram + PMP_SRAM_UUID + pos), image->b + pos);
	offset = readl(sram + PMP_SRAM_BOOTARGS_OFFSET);
	size = readl(sram + PMP_SRAM_BOOTARGS_SIZE);
	if (!uuid_equal(image, &want) || size < 8 || size > SZ_4K ||
	    size > sram_size || offset > sram_size - size || offset & 3) {
		ret = -EINVAL;
		goto out;
	}

	/* The boot-argument entries are packed: parse a copy. */
	args = kmalloc(size, GFP_KERNEL);
	if (!args) {
		ret = -ENOMEM;
		goto out;
	}
	memcpy_fromio(args, sram + offset, size);
	if (!apple_pmp_bootargs_valid(args, size))
		ret = -EINVAL;
	kfree(args);
out:
	iounmap(sram);
	return ret;
}

/*
 * The PMP tunables describe the PTD: the request and acknowledge ranges
 * must be where the offsets say, and each seeded device must be the one its
 * index names, acknowledged exactly when the node expects an acknowledgment.
 */
static int apple_pmp_start_check_layout(struct apple_pmp_report *rep,
					struct apple_pmp_start *st)
{
	const u8 *table;
	int len = 0;

	table = of_get_property(st->pmp, "apple,tunable-ptd-range", &len);
	if (!apple_pmp_ranges_valid(table, len > 0 ? len : 0, rep->offsets->tgt_read,
				    rep->offsets->tgt_write, rep->offsets->actual))
		return -EINVAL;

	len = 0;
	table = of_get_property(st->pmp, "apple,tunable-soc-device", &len);
	if (!apple_pmp_devices_valid(table, len > 0 ? len : 0, st->seed & ~st->record_seed,
				     st->ack & ~st->record_seed,
				     rep->offsets->devices_ordered))
		return -EINVAL;
	if (st->record_seed &&
	    !apple_pmp_records_valid(table, len > 0 ? len : 0, st->record_seed,
				     st->ack & st->record_seed))
		return -EINVAL;
	return 0;
}

/* The requests this driver seeds, from the report's always-on entries. */
static int apple_pmp_start_describe(struct apple_pmp_report *rep,
				    struct apple_pmp_start *st)
{
	struct device_node *child;
	bool record;
	u32 id;
	int ret;

	for_each_available_child_of_node(rep->dev->of_node, child) {
		if (!of_property_read_bool(child, "apple,always-on"))
			continue;
		ret = apple_pmp_report_bit(st, child, &id, &record);
		if (ret == -EINVAL)
			continue;
		if (ret || (st->seed & BIT_ULL(id))) {
			dev_err(rep->dev, "%s PMP not started: no unique request bit for %pOF\n",
				rep->offsets->name, child);
			of_node_put(child);
			return -EINVAL;
		}
		if (record)
			st->record_seed |= BIT_ULL(id);
		st->seed |= BIT_ULL(id);
		if (!of_property_read_bool(child, "apple,no-ack")) {
			st->ack |= BIT_ULL(id);
			if (!of_property_read_bool(child, "apple,defer-startup-ack"))
				st->startup_ack |= BIT_ULL(id);
		}
	}
	if (!st->seed || !st->startup_ack) {
		dev_err(rep->dev, "%s PMP not started: no acknowledged always-on report\n",
			rep->offsets->name);
		return -EINVAL;
	}
	if (apple_pmp_start_check_layout(rep, st)) {
		dev_err(rep->dev, "%s PMP not started: its tunables do not describe this PTD layout\n",
			rep->offsets->name);
		return -EINVAL;
	}
	return 0;
}

static void apple_pmp_start_finish(struct apple_pmp_start *st, int result)
{
	scoped_guard(mutex, &apple_pmp_start_mutex) {
		if (!st->stopping)
			st->result = result;
		complete_all(&st->started);
	}
	if (result && READ_ONCE(st->enabled))
		dev_err(st->rep->dev,
			"%s PMP start failed (%d) after the PMP was enabled: it may keep running until the next reboot. Reboot to stop it; to boot without starting it: %s, or nomodeset\n",
			st->rep->offsets->name, result, st->rep->offsets->fallback);
}

/* Teardown has begun: a start still in progress gives up early. */
static bool apple_pmp_start_stopping(struct apple_pmp_start *st)
{
	return READ_ONCE(st->stopping);
}

static bool apple_pmp_start_bound(struct apple_pmp_start *st)
{
	struct platform_device *pdev = of_find_device_by_node(st->pmp);
	bool bound = false;

	if (!pdev)
		return false;
	/* The device lock is held for a whole probe: never wait on it. */
	if (device_trylock(&pdev->dev)) {
		bound = device_is_bound(&pdev->dev);
		device_unlock(&pdev->dev);
	}
	put_device(&pdev->dev);
	return bound;
}

/* The PMP, and its DART and mailbox where they are still disabled. */
static int apple_pmp_start_enable(struct apple_pmp_start *st)
{
	struct device_node *nodes[3];
	unsigned int i;
	int ret = 0;

	nodes[0] = of_parse_phandle(st->pmp, "iommus", 0);
	nodes[1] = of_parse_phandle(st->pmp, "mboxes", 0);
	nodes[2] = of_node_get(st->pmp);

	of_changeset_init(&st->pmp_cs);
	for (i = 0; i < ARRAY_SIZE(nodes) && !ret; i++) {
		if (!nodes[i])
			ret = -EINVAL;
		else if (!of_device_is_available(nodes[i]))
			ret = of_changeset_update_prop_string(&st->pmp_cs, nodes[i],
							      "status", "okay");
		/* The PMP itself must not have been enabled behind this driver. */
		else if (nodes[i] == st->pmp)
			ret = -EBUSY;
	}
	/* Platform devices are created, and probed, in this order. */
	if (!ret)
		ret = of_changeset_apply(&st->pmp_cs);
	if (ret)
		of_changeset_destroy(&st->pmp_cs);

	for (i = 0; i < ARRAY_SIZE(nodes); i++)
		of_node_put(nodes[i]);
	return ret;
}

/* Every always-on entry has set its request, and nothing else is requested. */
static int apple_pmp_start_wait_seeded(struct apple_pmp_report *rep,
				       struct apple_pmp_start *st)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(PMP_START_SEED_TIMEOUT_MS);
	u64 request;

	for (;;) {
		request = readq(rep->base + rep->offsets->tgt_read);
		if ((request & st->seed) == st->seed)
			return 0;
		if (time_after(jiffies, deadline))
			break;
		msleep(20);
	}
	dev_err(rep->dev, "%s PMP not started: request %#llx, expected %#llx\n",
		rep->offsets->name, request, st->seed);
	return -ETIMEDOUT;
}

/* Step 3: everything that must hold before the PMP may run. */
static int apple_pmp_start_check(struct apple_pmp_start *st, uuid_t *image)
{
	struct apple_pmp_report *rep = st->rep;
	int ret;

	/* The always-on entries probe after the report, once their domains do. */
	ret = apple_pmp_start_wait_seeded(rep, st);
	if (!ret)
		ret = apple_pmp_start_check_power(rep);
	if (!ret)
		ret = apple_pmp_start_check_halted(st);
	if (!ret) {
		ret = apple_pmp_start_check_sram(st, image);
		if (ret)
			dev_err(rep->dev, "%s PMP not started: its SRAM does not hold the described firmware\n",
				rep->offsets->name);
	}
	return ret;
}

static const struct apple_pmp_start_ops apple_pmp_start_hw_ops = {
	.check = apple_pmp_start_check,
	.enable = apple_pmp_start_enable,
	.bound = apple_pmp_start_bound,
};

static void apple_pmp_start_work(struct work_struct *work)
{
	struct apple_pmp_start *st = container_of(work, struct apple_pmp_start, work);
	const struct apple_pmp_start_timing *timing = st->timing;
	struct apple_pmp_report *rep = st->rep;
	u64 status = 0, request, ack = 0;
	unsigned long deadline;
	uuid_t image;
	int ret;

	ret = st->ops->check(st, &image);
	if (ret) {
		apple_pmp_start_finish(st, ret);
		return;
	}
	status = readq(rep->base + rep->offsets->status);
	request = readq(rep->base + rep->offsets->tgt_read);
	ack = readq(rep->base + rep->offsets->actual);
	if (status || request != st->seed) {
		dev_err(rep->dev, "%s PMP not started: PTD status %#llx request %#llx before start, expected request %#llx\n",
			rep->offsets->name, status, request, st->seed);
		apple_pmp_start_finish(st, -EBUSY);
		return;
	}
	dev_info(rep->dev, "PMP halted, firmware %pUb; PTD status %#llx request %#llx ack %#llx\n",
		 &image, status, request, ack);
	if (st->record_seed)
		dev_info(rep->dev, "request bits %#llx are soc-device records\n",
			 st->record_seed);

	/* From here the PMP may run, and only a reboot stops it. */
	dev_info(rep->dev, "starting the PMP with request %#llx\n", st->seed);
	ret = st->ops->enable(st);
	if (ret) {
		dev_err(rep->dev, "%s PMP not started: enabling its nodes failed: %d\n",
			rep->offsets->name, ret);
		apple_pmp_start_finish(st, ret);
		return;
	}
	WRITE_ONCE(st->enabled, true);

	/* Bound means the firmware and its endpoint are up. */
	deadline = jiffies + msecs_to_jiffies(timing->bind);
	while (!st->ops->bound(st) && time_before(jiffies, deadline) &&
	       !apple_pmp_start_stopping(st))
		msleep(50);
	if (!st->ops->bound(st)) {
		dev_err(rep->dev, "PMP driver did not bind within %u ms; display power not acknowledged\n",
			timing->bind);
		apple_pmp_start_finish(st, -ETIMEDOUT);
		return;
	}
	msleep(timing->quiet);

	deadline = jiffies + msecs_to_jiffies(timing->ready);
	do {
		status = readq(rep->base + rep->offsets->status);
		if (status == PMP_REPORT_READY || apple_pmp_start_stopping(st))
			break;
		msleep(50);
	} while (time_before(jiffies, deadline));
	if (status == PMP_REPORT_READY) {
		deadline = jiffies + msecs_to_jiffies(timing->ack);
		do {
			ack = readq(rep->base + rep->offsets->actual);
			if ((ack & st->startup_ack) == st->startup_ack ||
			    apple_pmp_start_stopping(st))
				break;
			msleep(20);
		} while (time_before(jiffies, deadline));
	}
	request = readq(rep->base + rep->offsets->tgt_read);

	if (status != PMP_REPORT_READY) {
		dev_err(rep->dev, "PMP started but not ready: PTD status %#llx request %#llx\n",
			status, request);
		ret = -ETIMEDOUT;
	} else if ((ack & st->startup_ack) != st->startup_ack) {
		dev_err(rep->dev, "PMP ready but did not acknowledge %#llx: request %#llx ack %#llx\n",
			st->startup_ack, request, ack);
		ret = -EIO;
	} else {
		dev_info(rep->dev, "PMP ready: PTD status %#llx request %#llx ack %#llx\n",
			 status, request, ack);
		ret = 0;
	}
	apple_pmp_start_finish(st, ret);
}

/*
 * For the debugfs status of a report that starts its PMP: "start" is idle
 * until a consumer asks for the PMP, then starting, ready or failed, and
 * "enabled" says whether the PMP nodes were enabled (the PMP may then run
 * until the next reboot, whatever the result).
 */
static void apple_pmp_start_show(struct seq_file *s, struct apple_pmp_start *st)
{
	const char *state;
	int result;

	scoped_guard(mutex, &apple_pmp_start_mutex) {
		result = st->result;
		if (result)
			state = "failed";
		else if (completion_done(&st->started))
			state = "ready";
		else if (st->queued)
			state = "starting";
		else
			state = "idle";
	}
	seq_printf(s, "start: %s\nstart_result: %d\nenabled: %u\n", state, result,
		   READ_ONCE(st->enabled));
}

/* No work or waiter may use the start state once the report is torn down. */
static void apple_pmp_start_drain(void *data)
{
	struct apple_pmp_start *st = data;
	unsigned int waiters;

	scoped_guard(mutex, &apple_pmp_start_mutex) {
		st->stopping = true;
		list_del(&st->list);
		st->result = -ENODEV;
		complete_all(&st->started);
		waiters = st->waiters;
	}
	/* A start in progress gives up at its next poll. */
	cancel_work_sync(&st->work);
	if (waiters)
		wait_for_completion(&st->waiters_done);
	/*
	 * A started PMP keeps running and its nodes stay enabled: there is no
	 * safe stop (see "Starting a halted PMP" above).
	 */
	of_node_put(st->pmp);
}

/*
 * A report whose PMP a display gate handed over: apple,pmp names a disabled
 * PMP node that carries this Mac's values. Any other report is left exactly
 * as it was.
 */
static int apple_pmp_start_init(struct apple_pmp_report *rep)
{
	struct device *dev = rep->dev;
	struct apple_pmp_start *st;
	struct device_node *pmp;
	int ret;

	pmp = of_parse_phandle(dev->of_node, "apple,pmp", 0);
	if (!pmp || !of_device_is_compatible(pmp, "apple,t6000-pmp-v2") ||
	    of_device_is_available(pmp) || !of_property_present(pmp, "apple,tunable-uuid")) {
		of_node_put(pmp);
		return 0;
	}

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		of_node_put(pmp);
		return -ENOMEM;
	}
	st->rep = rep;
	st->pmp = pmp;
	st->ops = &apple_pmp_start_hw_ops;
	st->timing = &apple_pmp_start_timing;
	INIT_WORK(&st->work, apple_pmp_start_work);
	init_completion(&st->started);
	init_completion(&st->waiters_done);
	/* Stay bound if it does not check out, so consumers learn why. */
	ret = apple_pmp_start_describe(rep, st);
	if (ret) {
		st->result = ret;
		complete_all(&st->started);
	}
	scoped_guard(mutex, &apple_pmp_start_mutex)
		list_add_tail(&st->list, &apple_pmp_starts);
	ret = devm_add_action_or_reset(dev, apple_pmp_start_drain, st);
	if (ret)
		return ret;
	rep->start = st;
	if (!st->result)
		dev_info(dev, "%s PMP handed over: requests %#llx; it starts when a consumer asks for it\n",
			 rep->offsets->name, st->seed);
	return 0;
}

/*
 * Starts the PMP if no one has yet, and waits for request bit @id to be
 * acknowledged. The caller holds a waiter on @st (st->waiters), which this
 * gives back: teardown waits for it, so @st stays valid until then.
 */
static int apple_pmp_start_wait(struct apple_pmp_start *st, u32 id,
				unsigned long timeout)
{
	u64 status, ack;
	int ret;

	scoped_guard(mutex, &apple_pmp_start_mutex) {
		if (!st->stopping && !st->result && !st->queued) {
			st->queued = true;
			queue_work(system_unbound_wq, &st->work);
		}
	}
	ret = wait_for_completion_timeout(&st->started, timeout) ? 0 : -ETIMEDOUT;
	scoped_guard(mutex, &apple_pmp_start_mutex) {
		if (st->stopping) {
			ret = -ENODEV;
		} else if (!ret) {
			if (st->result) {
				ret = st->result;
			} else if (!(st->ack & BIT_ULL(id))) {
				ret = -EINVAL;
			} else {
				status = readq(st->rep->base + st->rep->offsets->status);
				ack = readq(st->rep->base + st->rep->offsets->actual);
				ret = status == PMP_REPORT_READY && (ack & BIT_ULL(id)) ? 0 : -EIO;
			}
		}
		if (!--st->waiters && st->stopping)
			complete(&st->waiters_done);
	}
	return ret;
}

/**
 * apple_pmp_report_wait_ready() - wait until a PMP report entry is acknowledged
 * @entry: report entry node whose request must be acknowledged
 * @timeout: in jiffies
 *
 * For a report whose halted PMP this driver starts (see above): the first
 * call starts it.
 *
 * Context: Process context; sleeps.
 * Return: 0 once the PMP runs and has acknowledged @entry, -EPROBE_DEFER
 * while the report driver has not probed yet, -ENODEV if no PMP was handed
 * over to this report, -EINVAL if @entry is not one of its acknowledged
 * requests, or the reason the PMP was not started or is not ready.
 */
int apple_pmp_report_wait_ready(struct device_node *entry, unsigned long timeout)
{
	struct device_node *parent = of_get_parent(entry);
	const struct apple_pmp_report *rep;
	struct apple_pmp_start *st = NULL, *it;
	bool record, bound = false;
	u32 id;
	int ret;

	if (!parent || of_property_read_u32(entry, "reg", &id) || id > 63) {
		of_node_put(parent);
		return -EINVAL;
	}

	scoped_guard(mutex, &apple_pmp_start_mutex) {
		list_for_each_entry(it, &apple_pmp_starts, list) {
			if (it->rep->dev->of_node == parent) {
				st = it;
				break;
			}
		}
		if (st && !st->stopping &&
		    !apple_pmp_report_bit(st, entry, &id, &record)) {
			st->waiters++;
		} else if (st) {
			ret = st->stopping ? -ENODEV : -EINVAL;
			st = NULL;
			goto out;
		}
	}
	if (!st) {
		/* A bound report that was handed no PMP will never start one. */
		rcu_read_lock();
		list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
			if (rep->dev->of_node == parent) {
				bound = true;
				break;
			}
		}
		rcu_read_unlock();
		ret = bound ? -ENODEV : -EPROBE_DEFER;
		goto out;
	}

	ret = apple_pmp_start_wait(st, id, timeout);
out:
	of_node_put(parent);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_wait_ready);

static int apple_pmp_report_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct apple_pmp_report *rep;
	int ret;

	rep = devm_kzalloc(dev, sizeof(*rep), GFP_KERNEL);
	if (!rep)
		return -ENOMEM;

	spin_lock_init(&rep->lock);
	rep->dev = dev;
	rep->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rep->base))
		return PTR_ERR(rep->base);
	rep->offsets = of_device_get_match_data(dev);
	platform_set_drvdata(pdev, rep);

	INIT_LIST_HEAD(&rep->list);
	ret = devm_add_action_or_reset(dev, apple_pmp_report_release, rep);
	if (ret)
		return ret;

	/* Before the entries exist: they request by soc-device record through it. */
	if (rep->offsets->starts_pmp) {
		ret = apple_pmp_start_init(rep);
		if (ret)
			return ret;
	}

	mutex_lock(&apple_pmp_reports_lock);
	list_add_tail_rcu(&rep->list, &apple_pmp_reports);
	mutex_unlock(&apple_pmp_reports_lock);

	if (rep->offsets->fast_die || rep->start)
		rep->debugfs = debugfs_create_file(dev_name(dev), 0400,
						   apple_pmp_report_debugfs, rep,
						   &apple_pmp_report_status_fops);

	/* Publish the region before child power callbacks start looking it up. */
	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create child devices\n");

	return 0;
}

static const struct apple_pmp_report_offsets apple_pmp_offsets_t600x = {
	.tgt_read = 0xf80,
	.tgt_write = 0x107c0,
	.actual = 0x1000,
	.status = 0x10,
};

static const struct apple_pmp_report_offsets apple_pmp_offsets_t602x = {
	.tgt_read = 0x2000,
	.tgt_write = 0x11000,
	.actual = 0x2080,
	.status = 0x10,
};

static const struct apple_pmp_report_offsets apple_pmp_offsets_t6030 = {
	.tgt_read = 0x1180,
	.tgt_write = 0x108c0,
	.actual = 0x11c0,
	.status = 0x10,
	/* Native G15 explicitly opts into the gate's halted-PMP handoff. */
	.starts_pmp = IS_ENABLED(CONFIG_APPLE_T6030_GPU_PMP_LINK),
	.name = "T6030",
	.fallback = "apple_t6030_display.enable=0",
	.pwrstate = "apple,t6030-pmgr-pwrstate",
	.devices_ordered = true,
};

static const struct apple_pmp_report_offsets apple_pmp_offsets_t8112 = {
	.tgt_read = 0xa00,
	.tgt_write = 0x10500,
	.actual = 0xa40,
	.status = 0x10,
};

/*
 * T8122, from its ADT ptd-range: SOC-DEV-PS-REQ at PTD index 0x100 (read
 * 0x1000, update 0x10000 + 0x800), SOC-DEV-PS-ACK at 0x108 (0x1080),
 * PMP-STATUS at 0x1 (0x10). Its soc-device records are not in ID order.
 */
static const struct apple_pmp_report_offsets apple_pmp_offsets_t8122 = {
	.tgt_read = 0x1000,
	.tgt_write = 0x10800,
	.actual = 0x1080,
	.status = 0x10,
	.starts_pmp = true,
	.name = "T8122",
	.fallback = "apple_t8122_display.enable=0",
	.pwrstate = "apple,t8122-pmgr-pwrstate",
};

/* The same PTD apertures; its startup belongs to the native PMP supplier. */
static const struct apple_pmp_report_offsets apple_pmp_offsets_j613_25g83 = {
	.tgt_read = 0x1000,
	.tgt_write = 0x10800,
	.actual = 0x1080,
	.status = 0x10,
};

/*
 * T6031, from the J516C ADT ptd-range: SOC-DEV-PS-REQ at PTD index 0x200
 * (read 0x2000, update 0x10000 + 0x1000), SOC-DEV-PS-ACK at 0x208 (0x2080),
 * PMP-STATUS at 0x1 (0x10): the T6020 apertures. Its soc-device records are
 * in ID order, as on T6030.
 */
static const struct apple_pmp_report_offsets apple_pmp_offsets_t6031 = {
	.tgt_read = 0x2000,
	.tgt_write = 0x11000,
	.actual = 0x2080,
	.status = 0x10,
	.starts_pmp = true,
	.name = "T6031",
	.fallback = "leave out apple_t6031_display.enable=1",
	.pwrstate = "apple,t6031-pmgr-pwrstate",
	.devices_ordered = true,
};

/* The fast die-temperature report is the low two bytes of entry 0x1c1. */
static const struct apple_pmp_report_offsets apple_pmp_offsets_t8140 = {
	.tgt_read = 0x1880,
	.tgt_write = 0x10c40,
	.actual = 0x18c0,
	.status = 0x10,
	.fast_die = 0x1c10,
	.fast_die_lanes = 2,
};

static const struct of_device_id apple_pmp_report_of_match[] = {
	{ .compatible = "apple,j613-25g83-pmp-report", .data = &apple_pmp_offsets_j613_25g83 },
	{ .compatible = "apple,t6000-pmp-v2-report", .data = &apple_pmp_offsets_t600x },
	{ .compatible = "apple,t6020-pmp-v2-report", .data = &apple_pmp_offsets_t602x },
	{ .compatible = "apple,t6030-pmp-v2-report", .data = &apple_pmp_offsets_t6030 },
	{ .compatible = "apple,t6031-pmp-v2-report", .data = &apple_pmp_offsets_t6031 },
	{ .compatible = "apple,t8112-pmp-v2-report", .data = &apple_pmp_offsets_t8112 },
	{ .compatible = "apple,t8122-pmp-v2-report", .data = &apple_pmp_offsets_t8122 },
	{ .compatible = "apple,t8140-pmp-v2-report", .data = &apple_pmp_offsets_t8140 },
	{}
};

static struct platform_driver apple_pmp_report_driver = {
	.probe = apple_pmp_report_probe,
	.driver = {
		.name = "apple-pmp-report",
		.of_match_table = apple_pmp_report_of_match,
		/* Power-domain providers are not intended for manual unbinding. */
		.suppress_bind_attrs = true,
	},
};

struct apple_pmp_report_entry {
	struct device_node *node;
	struct device_node *report_node;
	struct generic_pm_domain genpd;
	u32 id;
	/* The PMP does not acknowledge this request (a started PMP only). */
	bool no_ack;
	bool removed;
};

#define genpd_to_apple_pmp_report_entry(_genpd) \
	container_of(_genpd, struct apple_pmp_report_entry, genpd)

static int apple_pmp_report_set_state(struct apple_pmp_report *rep, u32 id,
				      bool enable, bool no_ack)
{
	u64 bit_val = BIT_ULL(id);
	u64 val;
	unsigned long flags;

	spin_lock_irqsave(&rep->lock, flags);
	val = readq(rep->base + rep->offsets->tgt_read);
	val &= ~bit_val;
	if (enable)
		val |= bit_val;
	writeq(val, rep->base + rep->offsets->tgt_write);
	spin_unlock_irqrestore(&rep->lock, flags);
	val = readq(rep->base + rep->offsets->status);
	if ((val & PMP_REPORT_READY) == 0 || no_ack)
		return 0;
	return readq_poll_timeout_atomic(
		rep->base + rep->offsets->actual,
		val,
		!!(val & bit_val) == !!enable,
		100,
		50000);
}

/* Find and check the report before taking the GPU vote; optionally update it. */
static int apple_pmp_report_t6030_gfx(struct device *supplier, const bool *enable)
{
	struct apple_pmp_report *rep, *found = NULL;
	struct device_node *pmp;
	const u8 *table;
	int len = 0, ret = -ENODEV;

	if (!IS_ENABLED(CONFIG_APPLE_T6030_GPU_PMP_LINK) ||
	    !of_machine_is_compatible("apple,t6030") ||
	    !of_device_is_compatible(supplier->of_node, "apple,t6030-pmp-v2"))
		return -ENODEV;

	/* T6030's ordered soc-device ID 5 is the acknowledged AGX request. */
	table = of_get_property(supplier->of_node, "apple,tunable-soc-device", &len);
	if (!apple_pmp_devices_valid(table, len, BIT_ULL(4), BIT_ULL(4), true) ||
	    memcmp(table + 4 * PMP_SOC_DEVICE_SIZE + 116, "AGX\0\0\0\0\0", 8))
		return -EINVAL;

	rcu_read_lock();
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->offsets != &apple_pmp_offsets_t6030)
			continue;
		pmp = of_parse_phandle(rep->dev->of_node, "apple,pmp", 0);
		if (pmp == supplier->of_node) {
			of_node_put(pmp);
			if (found)
				goto out;
			found = rep;
		} else {
			of_node_put(pmp);
		}
	}
	if (found) {
		if (readq(found->base + found->offsets->status) != PMP_REPORT_READY)
			ret = -EAGAIN;
		else
			ret = enable ? apple_pmp_report_set_state(found, 4, *enable, false) : 0;
	}
out:
	rcu_read_unlock();
	return ret;
}
int apple_pmp_report_check_t6030_gfx(struct device *supplier)
{
	return apple_pmp_report_t6030_gfx(supplier, NULL);
}
EXPORT_SYMBOL_GPL(apple_pmp_report_check_t6030_gfx);

/* The RTKit AGX vote alone does not enable PMP's GPU bandwidth controllers. */
int apple_pmp_report_set_t6030_gfx(struct device *supplier, bool enable)
{
	return apple_pmp_report_t6030_gfx(supplier, &enable);
}
EXPORT_SYMBOL_GPL(apple_pmp_report_set_t6030_gfx);

/*
 * T6031 (M3 Max): the GPU is powered by its PMGR domain alone, with no RTKit
 * vote, and nothing sets its PTD request, so PMP ignores the GPU's bandwidth
 * demand and memory stays at its idle floor. AGX is the acknowledged
 * soc-device ID 14 (record 13) of the J516C table. As on T6030, a request
 * made before the firmware runs is acknowledged without enabling bandwidth
 * scaling: enabling clears and sets it again. On a failed enable the request
 * is cleared, which is how the GPU ran before.
 */
int apple_pmp_report_t6031_gfx(bool enable)
{
	struct apple_pmp_report *rep, *found = NULL;
	struct device_node *pmp = NULL;
	const u8 *table = NULL;
	int len = 0, ret = -ENODEV;

	if (!IS_ENABLED(CONFIG_APPLE_T6031_GPU_PMP_REPORT) ||
	    !of_machine_is_compatible("apple,t6031"))
		return -ENODEV;

	rcu_read_lock();
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->offsets != &apple_pmp_offsets_t6031)
			continue;
		if (found)
			goto out;
		found = rep;
	}
	if (!found)
		goto out;
	pmp = of_parse_phandle(found->dev->of_node, "apple,pmp", 0);
	if (pmp && of_device_is_compatible(pmp, "apple,t6031-pmp-v2"))
		table = of_get_property(pmp, "apple,tunable-soc-device", &len);
	if (!apple_pmp_devices_valid(table, len > 0 ? len : 0, BIT_ULL(13), BIT_ULL(13), true) ||
	    memcmp(table + 13 * PMP_SOC_DEVICE_SIZE + 116, "AGX\0\0\0\0\0", 8)) {
		ret = -EINVAL;
	} else if (readq(found->base + found->offsets->status) != PMP_REPORT_READY) {
		ret = -EAGAIN;
	} else {
		ret = apple_pmp_report_set_state(found, 13, false, false);
		if (!ret && enable)
			ret = apple_pmp_report_set_state(found, 13, true, false);
		if (ret && enable)
			apple_pmp_report_set_state(found, 13, false, false);
	}
	of_node_put(pmp);
out:
	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_t6031_gfx);

static int apple_pmp_report_entry_set_state(struct generic_pm_domain *genpd, bool enable)
{
	struct apple_pmp_report_entry *ent = genpd_to_apple_pmp_report_entry(genpd);
	struct apple_pmp_report *rep;
	int ret = -ENODEV;

	if (READ_ONCE(ent->removed))
		return ret;

	rcu_read_lock();
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->dev->of_node == ent->report_node) {
			ret = apple_pmp_report_set_state(rep, ent->id, enable, ent->no_ack);
			break;
		}
	}
	rcu_read_unlock();
	return ret;
}

static int apple_pmp_report_entry_power_on(struct generic_pm_domain *genpd)
{
	return apple_pmp_report_entry_set_state(genpd, true);
}

static int apple_pmp_report_entry_power_off(struct generic_pm_domain *genpd)
{
	return apple_pmp_report_entry_set_state(genpd, false);
}

static void apple_pmp_report_entry_free(struct apple_pmp_report_entry *ent)
{
	of_node_put(ent->report_node);
	of_node_put(ent->node);
	kfree(ent->genpd.name);
	kfree(ent);
}

static void apple_pmp_report_entry_remove(struct platform_device *pdev)
{
	struct apple_pmp_report_entry *ent = platform_get_drvdata(pdev);

	WRITE_ONCE(ent->removed, true);
	of_genpd_del_provider(ent->node);
	if (pm_genpd_remove(&ent->genpd)) {
		/* The core still owns this domain. Keep its now inert storage alive. */
		dev_warn(&pdev->dev, "retaining busy power domain %s\n", ent->genpd.name);
		return;
	}
	apple_pmp_report_entry_free(ent);
}

static int apple_pmp_report_entry_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct apple_pmp_report_entry *ent;
	const struct apple_pmp_report *rep;
	struct apple_pmp_start *start;
	int ret;
	const char *name;
	struct of_phandle_iterator it;

	ent = kzalloc_obj(*ent);
	if (!ent)
		return -ENOMEM;

	ent->node = of_node_get(node);
	ent->report_node = of_node_get(dev->parent->of_node);

	ret = of_property_read_u32(node, "reg", &ent->id);
	if (ret) {
		ret = dev_err_probe(dev, ret, "missing reg property\n");
		goto err_free;
	}
	if (ent->id >= 64) {
		ret = -EINVAL;
		goto err_free;
	}
	/* Only a report that starts its PMP requests by record, or without ack. */
	rep = dev->parent ? dev_get_drvdata(dev->parent) : NULL;
	start = rep ? rep->start : NULL;
	if (start) {
		bool record;

		ret = apple_pmp_report_bit(start, node, &ent->id, &record);
		if (ret) {
			ret = dev_err_probe(dev, ret, "no request bit (reg, soc-device record)\n");
			goto err_free;
		}
		ent->no_ack = of_property_read_bool(node, "apple,no-ack");
	}

	ret = of_property_read_string(node, "label", &name);
	if (ret < 0) {
		ret = dev_err_probe(dev, ret, "missing label property\n");
		goto err_free;
	}
	ent->genpd.name = kstrdup(name, GFP_KERNEL);
	if (!ent->genpd.name) {
		ret = -ENOMEM;
		goto err_free;
	}

	if (of_property_read_bool(node, "apple,always-on")) {
		ent->genpd.flags |= GENPD_FLAG_ACTIVE_WAKEUP;
		apple_pmp_report_entry_set_state(&ent->genpd, true);
	}

	ent->genpd.power_on = apple_pmp_report_entry_power_on;
	ent->genpd.power_off = apple_pmp_report_entry_power_off;

	ret = pm_genpd_init(&ent->genpd, NULL, true);
	if (ret) {
		ret = dev_err_probe(dev, ret, "pm_genpd_init failed\n");
		goto err_free;
	}

	ret = of_genpd_add_provider_simple(node, &ent->genpd);
	if (ret) {
		ret = dev_err_probe(dev, ret, "of_genpd_add_provider_simple failed\n");
		goto err_remove_domain;
	}

	of_for_each_phandle(&it, ret, node, "power-domains", "#power-domain-cells", -1) {
		struct of_phandle_args parent, child;

		parent.np = it.node;
		parent.args_count = of_phandle_iterator_args(&it, parent.args, MAX_PHANDLE_ARGS);
		child.np = node;
		child.args_count = 0;
		ret = of_genpd_add_subdomain(&parent, &child);

		if (ret == -EPROBE_DEFER) {
			of_node_put(parent.np);
			goto err_remove;
		} else if (ret < 0) {
			dev_err(dev, "failed to add to parent domain: %d (%s -> %s)\n",
				ret, it.node->name, node->name);
			of_node_put(parent.np);
			goto err_remove;
		}
	}

	pm_genpd_remove_device(dev);
	platform_set_drvdata(pdev, ent);

	return 0;
err_remove:
	of_genpd_del_provider(node);
err_remove_domain:
	WRITE_ONCE(ent->removed, true);
	if (pm_genpd_remove(&ent->genpd)) {
		dev_warn(dev, "retaining busy power domain %s\n", ent->genpd.name);
		return ret;
	}
err_free:
	apple_pmp_report_entry_free(ent);
	return ret;
}

static const struct of_device_id apple_pmp_report_entry_of_match[] = {
	{ .compatible = "apple,t6000-pmp-v2-report-entry" },
	{ .compatible = "apple,t8140-pmp-v2-report-entry" },
	{}
};

static struct platform_driver apple_pmp_report_entry_driver = {
	.probe = apple_pmp_report_entry_probe,
	.remove = apple_pmp_report_entry_remove,
	.driver = {
		.name = "apple-pmp-report-entry",
		.of_match_table = apple_pmp_report_entry_of_match,
		/* Power-domain providers are not intended for manual unbinding. */
		.suppress_bind_attrs = true,
	},
};

MODULE_DEVICE_TABLE(of, apple_pmp_report_of_match);
MODULE_DEVICE_TABLE(of, apple_pmp_report_entry_of_match);

static int __init apple_pmp_report_init(void)
{
	apple_pmp_report_debugfs = debugfs_create_dir("apple-pmp-report", NULL);
	platform_driver_register(&apple_pmp_report_entry_driver);
	platform_driver_register(&apple_pmp_report_driver);
	return 0;
}

static void __exit apple_pmp_report_exit(void)
{
	debugfs_remove_recursive(apple_pmp_report_debugfs);
	platform_driver_unregister(&apple_pmp_report_entry_driver);
	platform_driver_unregister(&apple_pmp_report_driver);
}

module_init(apple_pmp_report_init);
module_exit(apple_pmp_report_exit);

MODULE_DESCRIPTION("PMP power state reporting driver for Apple SoCs");
MODULE_LICENSE("Dual MIT/GPL");

#if IS_ENABLED(CONFIG_APPLE_PMP_REPORT_START_KUNIT_TEST)
#include "pmp-report-start-test.c"
#endif
