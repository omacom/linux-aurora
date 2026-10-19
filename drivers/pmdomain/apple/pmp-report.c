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
#include <linux/hwmon.h>
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
#include <linux/slab.h>
#include <linux/soc/apple/pmp-report.h>
#include <linux/soc/apple/pmp-temps.h>
#include <linux/string.h>
#include <linux/thermal.h>
#include <linux/unaligned.h>
#include <linux/uuid.h>
#include <linux/workqueue.h>

#include "pmp-report-validation.h"

#define PMP_REPORT_READY 0x1

struct apple_pmp_report_offsets {
	u32 tgt_read;
	u32 tgt_write;
	u32 actual;
	u32 status;
	/* The PMP is started by this driver, once its requests are seeded. */
	bool starts_pmp;
	/* With starts_pmp: the SoC in messages, and its PMGR power states. */
	const char *name;
	const char *pwrstate;
	/* With starts_pmp: the PMP image has the T6030 temperature records. */
	bool temps;
	/* With starts_pmp: soc-device record i holds device id i + 1. */
	bool devices_ordered;
	/*
	 * With starts_pmp: request the CPU clients' DRAM and fabric floor
	 * once the PMP runs, see apple_pmp_t6030_floor_work().
	 */
	bool memory_floor;
	/*
	 * The report is published in apple_pmp_reports, where the fast
	 * die-temperature reader and the power callbacks of its entries look
	 * it up by node, and its teardown withdraws the entries first (T8140).
	 */
	bool published;
	/*
	 * With published: 32-bit word holding the fast die-temperature effort
	 * of each CPU cluster, one byte per lane; 0 if the SoC has none.
	 */
	u32 fast_die;
	/* With fast_die: number of lanes at fast_die, at most 4. */
	u32 fast_die_lanes;
};

struct apple_pmp_report {
	struct device *dev;
	const struct apple_pmp_report_offsets *offsets;
	void __iomem *base;
	void __iomem *temp_sram;
	spinlock_t lock;

	/* Only with offsets->published */
	struct list_head list;
	struct dentry *debugfs;

	/* Only with offsets->starts_pmp */
	struct device_node *pmp;
	u64 seed;
	u64 ack;
	/* Requests that must be acknowledged before any consumer can proceed. */
	u64 startup_ack;
	/* Seeded bits that are soc-device record indices (apple,soc-device-record). */
	u64 record_seed;
	struct of_changeset pmp_cs;
	struct work_struct start_work;
	/* The requests are in place; the first consumer starts the PMP. */
	bool seeded;
	atomic_t start_requested;
	atomic_t start_queued;
	struct completion started;
	int start_result;
	bool stopping;
	unsigned int waiters;
	struct completion waiters_done;
	/* Only with offsets->memory_floor: the floor request, once the PMP runs. */
	resource_size_t size;
	struct delayed_work floor_work;
	unsigned int floor_try;
	bool floor_valid;
	u32 floor_req[2];
	u32 floor_state[2];
	u64 floor_prior[2];
};

/* Serializes M3 publication, startup requests and waiter references. */
static DEFINE_MUTEX(apple_pmp_report_mutex);

/*
 * The PTD request/acknowledge bit of a report entry. An entry's reg is its
 * soc-device ID - 1. An entry marked apple,soc-device-record is requested by
 * soc-device record instead: on T8122 the PTD request and acknowledge bits
 * are indexed by soc-device record (apple,tunable-soc-device), and the
 * records are not in ID order, so the bit is the index of the record that
 * holds that ID.
 */
static int apple_pmp_report_bit(const struct apple_pmp_report *rep,
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
	if (!rep->offsets->starts_pmp || !rep->pmp)
		return -EINVAL;
	table = of_get_property(rep->pmp, "apple,tunable-soc-device", &len);
	index = apple_pmp_record_of_id(table, len > 0 ? len : 0, reg + 1);
	if (index < 0 || index > 63)
		return -ENOENT;
	*bit = index;
	return 0;
}


static umode_t apple_pmp_temp_is_visible(const void *data,
					 enum hwmon_sensor_types type, u32 attr,
					 int channel)
{
	if (channel < 0 || channel >= PMP_TEMP_CHANNELS || type != hwmon_temp)
		return 0;
	if (attr == hwmon_temp_input || attr == hwmon_temp_label)
		return 0444;
	return 0;
}

static int apple_pmp_temp_read(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, long *value)
{
	struct apple_pmp_report *rep = dev_get_drvdata(dev);

	if (channel < 0 || channel >= PMP_TEMP_CHANNELS)
		return -EINVAL;
	if (type != hwmon_temp || attr != hwmon_temp_input)
		return -EOPNOTSUPP;
	if (!channel)
		return apple_pmp_temp_hotspot(rep->temp_sram, value);
	return apple_pmp_temp_sample(rep->temp_sram, channel - 1, value);
}

static int apple_pmp_temp_read_string(struct device *dev, enum hwmon_sensor_types type,
				      u32 attr, int channel, const char **str)
{
	if (channel < 0 || channel >= PMP_TEMP_CHANNELS)
		return -EINVAL;
	if (type != hwmon_temp || attr != hwmon_temp_label)
		return -EOPNOTSUPP;

	*str = channel ? apple_pmp_temp_records[channel - 1].label :
			 "PMP aggregate die hotspot";
	return 0;
}

static const struct hwmon_ops apple_pmp_temp_ops = {
	.is_visible = apple_pmp_temp_is_visible,
	.read = apple_pmp_temp_read,
	.read_string = apple_pmp_temp_read_string,
};

static const struct hwmon_channel_info * const apple_pmp_temp_info[] = {
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	NULL,
};

static const struct hwmon_chip_info apple_pmp_temp_chip = {
	.ops = &apple_pmp_temp_ops,
	.info = apple_pmp_temp_info,
};

static int apple_pmp_temp_register(struct apple_pmp_report *rep)
{
	struct device *hwmon;

	/* Read-only telemetry from running firmware; never start or stop PMP. */
	rep->temp_sram = devm_ioremap(rep->dev, PMP_TEMP_SRAM_BASE, PMP_TEMP_SRAM_SIZE);
	if (!rep->temp_sram)
		return dev_err_probe(rep->dev, -ENOMEM, "cannot map PMP temperature SRAM\n");

	hwmon = devm_hwmon_device_register_with_info(rep->dev, "apple_pmp", rep,
						     &apple_pmp_temp_chip, NULL);
	if (IS_ERR(hwmon))
		return dev_err_probe(rep->dev, PTR_ERR(hwmon), "cannot register PMP hwmon\n");
	return 0;
}

/*
 * T6030: the PMP is described in the device tree, disabled, and this driver
 * starts it. iBoot leaves the PMP loaded but halted. Once the PMP runs it
 * owns the SoC power states, and it keeps a block powered only if that
 * block's request bit is set in the PTD when it starts. So, in order:
 *
 *  1. the display power domains must be on, the DCP CPU domain must already
 *     have "active" as its minimum state (the PMP must not be able to power
 *     the running DCP down), and the PTD SRAM and the PMP must be powered;
 *  2. the PMP CPU must be halted, and its SRAM must hold the firmware image
 *     the device tree describes, with the boot arguments the PMP driver
 *     fills in;
 *  3. the PTD must report the PMP as not ready, with no request set except
 *     the ones this driver seeds;
 *  4. the always-on report entries (display and storage) set their request
 *     bits while the PMP is still halted, and the result is read back;
 *  5. the PMP is started only when a consumer first waits for it through
 *     apple_pmp_report_wait_ready(), after the rest of the SoC has been
 *     brought up: the halted PMP and the PTD are checked again, then the
 *     PMP and its DART and mailbox are enabled, which starts the PMP
 *     through its own driver;
 *  6. once that driver has bound, and after one second in which the PTD is
 *     not touched, the PMP must report ready and acknowledge the requests
 *     required at startup. Entries marked apple,defer-startup-ack still
 *     require their own acknowledgment before their consumer can proceed.
 *
 * A refusal before step 5 leaves the PMP halted, and so does a boot on which
 * no consumer asks for it. Once the PMP has been started it can only be
 * stopped by a reboot.
 */
#define PMP_ASC_CPU_CONTROL		0x44
#define PMP_ASC_CPU_RUN			BIT(4)
#define PMP_SRAM_UUID			0x214
#define PMP_SRAM_BOOTARGS_OFFSET	0x22c
#define PMP_SRAM_BOOTARGS_SIZE		0x230

#define APPLE_PMGR_PS_MIN		GENMASK(19, 16)
#define APPLE_PMGR_PS_ACTUAL		GENMASK(7, 4)
#define APPLE_PMGR_PS_ACTIVE		0xf

#define PMP_START_BIND_TIMEOUT_MS	20000
#define PMP_START_QUIET_MS		1000
#define PMP_START_READY_TIMEOUT_MS	3000
#define PMP_START_ACK_TIMEOUT_MS	1000

static const struct {
	const char *label;
	/* The minimum state must already be "active". */
	bool floor;
} apple_pmp_t6030_domains[] = {
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

/* Finds the single PMGR power state of this SoC with @label. */
static struct device_node *apple_pmp_t6030_domain(struct apple_pmp_report *rep,
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

static int apple_pmp_t6030_check_power(struct apple_pmp_report *rep)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(apple_pmp_t6030_domains); i++) {
		const char *label = apple_pmp_t6030_domains[i].label;
		struct device_node *np = apple_pmp_t6030_domain(rep, label);
		struct regmap *regmap;
		u32 offset, val;
		int ret;

		if (!np)
			return dev_err_probe(rep->dev, -ENODEV,
					     "%s PMP not started: no single %s power state\n",
					     rep->offsets->name, label);
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

		if (FIELD_GET(APPLE_PMGR_PS_ACTUAL, val) != APPLE_PMGR_PS_ACTIVE)
			return dev_err_probe(rep->dev, -EIO,
					     "%s PMP not started: %s is not powered (%#x)\n",
					     rep->offsets->name, label, val);
		/* The PMGR power state driver applies the floor when it probes. */
		if (apple_pmp_t6030_domains[i].floor &&
		    FIELD_GET(APPLE_PMGR_PS_MIN, val) != APPLE_PMGR_PS_ACTIVE)
			return dev_err_probe(rep->dev, -EPROBE_DEFER,
					     "waiting for the %s minimum state to be active (%#x)\n",
					     label, val);
	}

	return 0;
}

static void __iomem *apple_pmp_t6030_map(struct apple_pmp_report *rep, const char *name,
					 resource_size_t *size)
{
	struct resource res;
	void __iomem *base;
	int idx;

	idx = of_property_match_string(rep->pmp, "reg-names", name);
	if (idx < 0 || of_address_to_resource(rep->pmp, idx, &res))
		return NULL;
	/* The PMP driver claims these registers; only read them here. */
	base = ioremap_np(res.start, resource_size(&res));
	if (base && size)
		*size = resource_size(&res);
	return base;
}

/* The SRAM must hold the image named by the node's tunables, and its boot arguments. */
static int apple_pmp_t6030_check_sram(struct apple_pmp_report *rep, uuid_t *image)
{
	void __iomem *sram;
	resource_size_t sram_size;
	const char *expected;
	u32 offset, size, pos;
	u8 *args;
	uuid_t want;
	int ret = 0;

	if (of_property_read_string(rep->pmp, "apple,tunable-uuid", &expected) ||
	    uuid_parse(expected, &want))
		return -EINVAL;
	sram = apple_pmp_t6030_map(rep, "pmp", &sram_size);
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
static int apple_pmp_t6030_check_layout(struct apple_pmp_report *rep)
{
	const u8 *table;
	int len = 0;

	table = of_get_property(rep->pmp, "apple,tunable-ptd-range", &len);
	if (!apple_pmp_ranges_valid(table, len, rep->offsets->tgt_read,
				    rep->offsets->tgt_write, rep->offsets->actual))
		return -EINVAL;

	table = of_get_property(rep->pmp, "apple,tunable-soc-device", &len);
	if (!apple_pmp_devices_valid(table, len, rep->seed & ~rep->record_seed,
				     rep->ack & ~rep->record_seed,
				     rep->offsets->devices_ordered))
		return -EINVAL;
	if (rep->record_seed &&
	    !apple_pmp_records_valid(table, len, rep->record_seed,
				     rep->ack & rep->record_seed))
		return -EINVAL;
	return 0;
}

/*
 * T6030 DRAM and fabric floor. Once the PMP runs it owns the DRAM controller
 * (DCS) and SoC fabric (FAB) performance states, and nothing asks it for more
 * than their lowest states: an 8-thread CPU copy measures 28 GB/s and the GPU
 * is starved. The PMP's CPU clients, EACC0 and PACC0, are the first two
 * entries of the SOC-DEV-DVFS range of its device-state table (PTD indices
 * 288 and 289 on the J514S); a request there with the valid bit set asks for
 * a DCS and a FAB state, and the DVFS-STATE range (indices 8 and 10)
 * publishes the firmware's response.
 *
 * The PMP applies a CPU-client request on its change notification, and a
 * read of the request entry may consume that notification: a floor written
 * and read back at once stayed latched without effect. So, from a delayed
 * work once the PMP has reported ready: read the two entries once, clear the
 * floor fields (the floor is always a fresh transition), write both floors
 * back to back through the request-set aperture and confirm on the
 * DVFS-STATE entries without reading the requests again; retry if the
 * firmware did not respond. With the floor the DVFS-STATE entries go from
 * 0x100 to 0x2104, the CPU copy bandwidth from 28 to 123 GB/s.
 *
 * The entry indices are taken from the PMP's own apple,tunable-ptd-range,
 * not hard-coded; the floor values are the J514S measurement. Known on T6030
 * only: the T6031 table's CPU-client entries have not been read.
 */
#define PMP_T6030_FLOOR_VALID		BIT_ULL(61)
#define PMP_T6030_FLOOR_DCS		GENMASK_ULL(51, 48)
#define PMP_T6030_FLOOR_FAB		GENMASK_ULL(35, 32)
#define PMP_T6030_FLOOR_MASK		(PMP_T6030_FLOOR_VALID | PMP_T6030_FLOOR_DCS | \
					 PMP_T6030_FLOOR_FAB)
#define PMP_T6030_FLOOR			(PMP_T6030_FLOOR_VALID | \
					 FIELD_PREP(PMP_T6030_FLOOR_DCS, 4) | \
					 FIELD_PREP(PMP_T6030_FLOOR_FAB, 2))
#define PMP_T6030_FLOOR_CLIENTS		2
#define PMP_T6030_FLOOR_SETTLE_MS	100
#define PMP_T6030_FLOOR_TRIES		10
#define PMP_PTD_ENTRY(i)		((i) * 16)
#define PMP_PTD_SET(i)			(0x10000 + (i) * 8)

/* The entries of the floor, from the PMP's PTD ranges; not fatal to the start. */
static void apple_pmp_t6030_floor_setup(struct apple_pmp_report *rep)
{
	const u8 *table;
	u32 req, state, count;
	unsigned int i;
	int len = 0;

	if (!rep->offsets->memory_floor)
		return;
	table = of_get_property(rep->pmp, "apple,tunable-ptd-range", &len);
	if (!apple_pmp_range_find(table, len > 0 ? len : 0, PMP_PTD_RANGE_SOC_DEV_DVFS,
				  "SOC-DEV-DVFS", &req, &count) ||
	    count < PMP_T6030_FLOOR_CLIENTS ||
	    !apple_pmp_range_find(table, len > 0 ? len : 0, PMP_PTD_RANGE_DVFS_STATE,
				  "DVFS-STATE", &state, &count) ||
	    count < 2 * PMP_T6030_FLOOR_CLIENTS) {
		dev_warn(rep->dev, "no memory floor: the PTD ranges do not describe the CPU clients\n");
		return;
	}
	for (i = 0; i < PMP_T6030_FLOOR_CLIENTS; i++) {
		rep->floor_req[i] = req + i;
		rep->floor_state[i] = state + 2 * i;
		if (PMP_PTD_SET(rep->floor_req[i]) + 8 > rep->size ||
		    PMP_PTD_ENTRY(rep->floor_state[i]) + 16 > rep->size) {
			dev_warn(rep->dev, "no memory floor: PTD entries %u/%u outside the aperture\n",
				 rep->floor_req[i], rep->floor_state[i]);
			return;
		}
	}
	rep->floor_valid = true;
}

static void apple_pmp_t6030_floor_work(struct work_struct *work)
{
	struct apple_pmp_report *rep = container_of(work, struct apple_pmp_report,
						    floor_work.work);
	u64 before[PMP_T6030_FLOOR_CLIENTS], after[PMP_T6030_FLOOR_CLIENTS];
	bool confirmed = false;
	unsigned int i;

	if (readq(rep->base + rep->offsets->status) != PMP_REPORT_READY)
		goto retry;
	/* The first try reads the requests while no change of ours is pending. */
	if (!rep->floor_try)
		for (i = 0; i < PMP_T6030_FLOOR_CLIENTS; i++)
			rep->floor_prior[i] = readq(rep->base + PMP_PTD_ENTRY(rep->floor_req[i])) &
					      ~PMP_T6030_FLOOR_MASK;
	for (i = 0; i < PMP_T6030_FLOOR_CLIENTS; i++)
		writeq(rep->floor_prior[i], rep->base + PMP_PTD_SET(rep->floor_req[i]));
	msleep(PMP_T6030_FLOOR_SETTLE_MS);
	for (i = 0; i < PMP_T6030_FLOOR_CLIENTS; i++)
		before[i] = readq(rep->base + PMP_PTD_ENTRY(rep->floor_state[i]));
	for (i = 0; i < PMP_T6030_FLOOR_CLIENTS; i++)
		writeq(rep->floor_prior[i] | PMP_T6030_FLOOR,
		       rep->base + PMP_PTD_SET(rep->floor_req[i]));
	msleep(PMP_T6030_FLOOR_SETTLE_MS);
	for (i = 0; i < PMP_T6030_FLOOR_CLIENTS; i++)
		after[i] = readq(rep->base + PMP_PTD_ENTRY(rep->floor_state[i]));
	confirmed = before[0] != after[0] || before[1] != after[1];
	dev_info(rep->dev, "memory floor DCS%llu/FAB%llu try %u: DVFS state %#llx/%#llx -> %#llx/%#llx%s\n",
		 (unsigned long long)FIELD_GET(PMP_T6030_FLOOR_DCS, PMP_T6030_FLOOR),
		 (unsigned long long)FIELD_GET(PMP_T6030_FLOOR_FAB, PMP_T6030_FLOOR),
		 rep->floor_try + 1, before[0], before[1], after[0], after[1],
		 confirmed ? "" : ", no response");
retry:
	if (!confirmed && ++rep->floor_try < PMP_T6030_FLOOR_TRIES) {
		schedule_delayed_work(&rep->floor_work, HZ);
		return;
	}
	if (!confirmed)
		dev_warn(rep->dev, "memory floor not confirmed by the PMP\n");
}

static int apple_pmp_t6030_check(struct apple_pmp_report *rep)
{
	struct device_node *np = rep->dev->of_node;
	struct device_node *child;
	void __iomem *asc;
	uuid_t image;
	u64 status, request, ack;
	u32 control, id;
	int ret;

	/* 1: power */
	ret = apple_pmp_t6030_check_power(rep);
	if (ret)
		return ret;

	/* 2: a halted PMP holding the described firmware image */
	rep->pmp = of_parse_phandle(np, "apple,pmp", 0);
	if (!rep->pmp || !of_device_is_compatible(rep->pmp, "apple,t6000-pmp-v2") ||
	    of_device_is_available(rep->pmp))
		return dev_err_probe(rep->dev, -EINVAL,
				     "%s PMP not started: apple,pmp is not a disabled PMP node\n",
				     rep->offsets->name);
	asc = apple_pmp_t6030_map(rep, "asc", NULL);
	if (!asc)
		return -ENOMEM;
	control = readl(asc + PMP_ASC_CPU_CONTROL);
	iounmap(asc);
	if (control & PMP_ASC_CPU_RUN)
		return dev_err_probe(rep->dev, -EBUSY,
				     "%s PMP not started: its CPU is already running (%#x)\n",
				     rep->offsets->name, control);
	ret = apple_pmp_t6030_check_sram(rep, &image);
	if (ret)
		return dev_err_probe(rep->dev, ret,
				     "%s PMP not started: its SRAM does not hold the described firmware\n",
				     rep->offsets->name);

	/* 3: the requests this driver seeds, and a PTD with nothing else in it */
	for_each_available_child_of_node(np, child) {
		bool record;

		if (!of_property_read_bool(child, "apple,always-on"))
			continue;
		ret = apple_pmp_report_bit(rep, child, &id, &record);
		if (ret == -EINVAL)
			continue;
		if (ret || (rep->seed & BIT_ULL(id))) {
			of_node_put(child);
			return dev_err_probe(rep->dev, -EINVAL,
					     "%s PMP not started: no unique request bit for %pOF\n",
					     rep->offsets->name, child);
		}
		if (record)
			rep->record_seed |= BIT_ULL(id);
		rep->seed |= BIT_ULL(id);
		if (!of_property_read_bool(child, "apple,no-ack")) {
			rep->ack |= BIT_ULL(id);
			if (!of_property_read_bool(child, "apple,defer-startup-ack"))
				rep->startup_ack |= BIT_ULL(id);
		}
	}
	if (!rep->seed || !rep->startup_ack)
		return dev_err_probe(rep->dev, -EINVAL,
				     "%s PMP not started: no acknowledged always-on report\n",
				     rep->offsets->name);
	if (apple_pmp_t6030_check_layout(rep))
		return dev_err_probe(rep->dev, -EINVAL,
				     "%s PMP not started: its tunables do not describe this PTD layout\n",
				     rep->offsets->name);
	apple_pmp_t6030_floor_setup(rep);

	status = readq(rep->base + rep->offsets->status);
	request = readq(rep->base + rep->offsets->tgt_read);
	ack = readq(rep->base + rep->offsets->actual);
	if (status || (request & ~rep->seed))
		return dev_err_probe(rep->dev, -EBUSY,
				     "%s PMP not started: PTD status %#llx request %#llx before start\n",
				     rep->offsets->name, status, request);

	dev_info(rep->dev,
		 "PMP halted, firmware %pUb; PTD status %#llx request %#llx ack %#llx; requesting %#llx\n",
		 &image, status, request, ack, rep->seed);
	if (rep->record_seed)
		dev_info(rep->dev, "request bits %#llx are soc-device records\n",
			 rep->record_seed);
	return 0;
}

static void apple_pmp_t6030_finish(struct apple_pmp_report *rep, int result)
{
	guard(mutex)(&apple_pmp_report_mutex);
	if (!rep->stopping)
		rep->start_result = result;
	complete_all(&rep->started);
}

static bool apple_pmp_t6030_bound(struct apple_pmp_report *rep)
{
	struct platform_device *pdev = of_find_device_by_node(rep->pmp);
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

/* Just before the start: the PMP still halted, and the PTD as seeded. */
static int apple_pmp_t6030_recheck(struct apple_pmp_report *rep)
{
	void __iomem *asc = apple_pmp_t6030_map(rep, "asc", NULL);
	u64 status, request;
	u32 control;

	if (!asc)
		return -ENOMEM;
	control = readl(asc + PMP_ASC_CPU_CONTROL);
	iounmap(asc);
	status = readq(rep->base + rep->offsets->status);
	request = readq(rep->base + rep->offsets->tgt_read);
	if ((control & PMP_ASC_CPU_RUN) || status || request != rep->seed) {
		dev_err(rep->dev,
			"%s PMP not started: CPU control %#x, PTD status %#llx request %#llx changed since seeding\n",
			rep->offsets->name, control, status, request);
		return -EBUSY;
	}
	return 0;
}

/* The PMP is enabled with its DART and mailbox, which must still be disabled. */
static int apple_pmp_t6030_enable(struct apple_pmp_report *rep)
{
	struct device_node *nodes[3];
	unsigned int i;
	int ret = 0;

	nodes[0] = of_parse_phandle(rep->pmp, "iommus", 0);
	nodes[1] = of_parse_phandle(rep->pmp, "mboxes", 0);
	nodes[2] = of_node_get(rep->pmp);

	of_changeset_init(&rep->pmp_cs);
	for (i = 0; i < ARRAY_SIZE(nodes) && !ret; i++) {
		if (!nodes[i] || of_device_is_available(nodes[i]))
			ret = -EINVAL;
		else
			ret = of_changeset_update_prop_string(&rep->pmp_cs, nodes[i],
							      "status", "okay");
	}
	/* Platform devices are created, and probed, in this order. */
	if (!ret)
		ret = of_changeset_apply(&rep->pmp_cs);
	if (ret)
		of_changeset_destroy(&rep->pmp_cs);

	for (i = 0; i < ARRAY_SIZE(nodes); i++)
		of_node_put(nodes[i]);
	return ret;
}

/*
 * T6030 PMP temperatures, read from the SRAM of the PMP image the PMP node
 * names in apple,tunable-uuid (the report driver starts no other image, and
 * the display gate gives the node only the image it knows): 0xa4-byte records
 * with the sensor name at +0x94, the last valid sample (1/64 degree Celsius)
 * at +0x3c and its valid flag at +0x48. The current sample is not used: it is
 * invalid while a CPU cluster is idle, so the last valid sample of an idle
 * cluster is its temperature when it was last active. Te* and Tp* are the CPU
 * cluster sensors; Ta000m is often without a valid sample. None of them is a
 * GPU sensor. Registered once the PMP runs: before that the records hold
 * whatever the halted image left.
 */
#define T6030_PMP_TEMP_RECORD_SIZE	0xa4
#define T6030_PMP_TEMP_NAME		0x94
#define T6030_PMP_TEMP_SAMPLE		0x3c
#define T6030_PMP_TEMP_VALID		0x48
#define T6030_PMP_TEMP_READ_ATTEMPTS	20
#define T6030_PMP_TEMP_ZONE		"apple_pmp_hotspot"

static const struct {
	const char *name;
	const char *label;
	u32 offset;
	/* May have no valid sample; the hotspot then leaves it out. */
	bool optional;
} apple_pmp_t6030_temps[] = {
	{ "ta000m", "Ta000m", 0x69190, true },
	{ "te000m", "Te000m", 0x69234 },
	{ "te001m", "Te001m", 0x692d8 },
	{ "tp000m", "Tp000m", 0x6937c },
	{ "tp001m", "Tp001m", 0x69420 },
	{ "tp002m", "Tp002m", 0x694c4 },
};

#define T6030_PMP_TEMP_CHANNELS		(1 + ARRAY_SIZE(apple_pmp_t6030_temps))

static bool temps = true;
module_param(temps, bool, 0444);
MODULE_PARM_DESC(temps, "Register the T6030 PMP temperatures once the PMP runs (default: on)");

static int apple_pmp_t6030_temp_sample(struct apple_pmp_report *rep, unsigned int index,
				       long *value)
{
	const char *name = apple_pmp_t6030_temps[index].name;
	void __iomem *record = rep->temp_sram + apple_pmp_t6030_temps[index].offset;
	u8 first[T6030_PMP_TEMP_RECORD_SIZE], second[T6030_PMP_TEMP_RECORD_SIZE];
	unsigned int attempt;
	s32 sample;

	/* The PMP updates records while they are read: use a stable copy. */
	for (attempt = 0; attempt < T6030_PMP_TEMP_READ_ATTEMPTS; attempt++) {
		memcpy_fromio(first, record, sizeof(first));
		memcpy_fromio(second, record, sizeof(second));
		if (memcmp(first, second, sizeof(first)))
			continue;
		if (memcmp(first + T6030_PMP_TEMP_NAME, name, strlen(name) + 1))
			return -EIO;
		if (get_unaligned_le32(first + T6030_PMP_TEMP_VALID) != 1)
			return -ENODATA;
		sample = (s32)get_unaligned_le32(first + T6030_PMP_TEMP_SAMPLE);
		if (sample < -2560 || sample > 9600)
			return -ERANGE;
		*value = DIV_ROUND_CLOSEST((s64)sample * 1000, 64);
		return 0;
	}
	return -EAGAIN;
}

/* The hottest record; fails if a record that is not optional cannot be read. */
static int apple_pmp_t6030_temp_hotspot(struct apple_pmp_report *rep, long *value)
{
	bool found = false;
	long hottest = 0, sample;
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(apple_pmp_t6030_temps); i++) {
		ret = apple_pmp_t6030_temp_sample(rep, i, &sample);
		if (ret == -ENODATA && apple_pmp_t6030_temps[i].optional)
			continue;
		if (ret)
			return ret;
		if (!found || sample > hottest)
			hottest = sample;
		found = true;
	}
	if (!found)
		return -ENODATA;
	*value = hottest;
	return 0;
}

static umode_t apple_pmp_t6030_temp_is_visible(const void *data,
					       enum hwmon_sensor_types type, u32 attr,
					       int channel)
{
	if (type != hwmon_temp || channel < 0 || channel >= T6030_PMP_TEMP_CHANNELS)
		return 0;
	if (attr == hwmon_temp_input || attr == hwmon_temp_label)
		return 0444;
	return 0;
}

static int apple_pmp_t6030_temp_read(struct device *dev, enum hwmon_sensor_types type,
				     u32 attr, int channel, long *value)
{
	struct apple_pmp_report *rep = dev_get_drvdata(dev);

	if (type != hwmon_temp || attr != hwmon_temp_input)
		return -EOPNOTSUPP;
	if (channel < 0 || channel >= T6030_PMP_TEMP_CHANNELS)
		return -EINVAL;
	if (!channel)
		return apple_pmp_t6030_temp_hotspot(rep, value);
	return apple_pmp_t6030_temp_sample(rep, channel - 1, value);
}

static int apple_pmp_t6030_temp_read_string(struct device *dev, enum hwmon_sensor_types type,
					    u32 attr, int channel, const char **str)
{
	if (type != hwmon_temp || attr != hwmon_temp_label)
		return -EOPNOTSUPP;
	if (channel < 0 || channel >= T6030_PMP_TEMP_CHANNELS)
		return -EINVAL;
	*str = channel ? apple_pmp_t6030_temps[channel - 1].label : "PMP CPU hotspot";
	return 0;
}

static const struct hwmon_ops apple_pmp_t6030_temp_ops = {
	.is_visible = apple_pmp_t6030_temp_is_visible,
	.read = apple_pmp_t6030_temp_read,
	.read_string = apple_pmp_t6030_temp_read_string,
};

static const struct hwmon_channel_info * const apple_pmp_t6030_temp_info[] = {
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	NULL,
};

static const struct hwmon_chip_info apple_pmp_t6030_temp_chip = {
	.ops = &apple_pmp_t6030_temp_ops,
	.info = apple_pmp_t6030_temp_info,
};

#if IS_ENABLED(CONFIG_THERMAL)
static int apple_pmp_t6030_zone_get_temp(struct thermal_zone_device *tz, int *temp)
{
	long value;
	int ret;

	ret = apple_pmp_t6030_temp_hotspot(thermal_zone_device_priv(tz), &value);
	if (!ret)
		*temp = value;
	return ret;
}

static const struct thermal_zone_device_ops apple_pmp_t6030_zone_ops = {
	.get_temp = apple_pmp_t6030_zone_get_temp,
};

static void apple_pmp_t6030_zone_unregister(void *data)
{
	thermal_zone_device_unregister(data);
}

/*
 * A thermal zone with the hotspot, for in-kernel users that limit SoC power
 * by temperature (they look it up by name). No trip points, not polled.
 */
static void apple_pmp_t6030_zone_register(struct apple_pmp_report *rep)
{
	struct thermal_zone_device *tz;

	tz = thermal_tripless_zone_device_register(T6030_PMP_TEMP_ZONE, rep,
						   &apple_pmp_t6030_zone_ops, NULL);
	if (IS_ERR(tz)) {
		dev_warn(rep->dev, "cannot register the PMP temperature zone: %pe\n", tz);
		return;
	}
	devm_add_action_or_reset(rep->dev, apple_pmp_t6030_zone_unregister, tz);
}
#else
static void apple_pmp_t6030_zone_register(struct apple_pmp_report *rep)
{
}
#endif

static void apple_pmp_t6030_temp_unmap(void *data)
{
	iounmap(data);
}

/* Once the PMP runs: its temperatures as a hwmon device and a thermal zone. Never fatal. */
static void apple_pmp_t6030_temps_register(struct apple_pmp_report *rep)
{
	resource_size_t size;
	struct device *hwmon;
	unsigned int i;
	long value;
	int ret;

	if (!temps) {
		dev_info(rep->dev, "PMP temperatures not registered (pmp_report.temps=0)\n");
		return;
	}
	rep->temp_sram = apple_pmp_t6030_map(rep, "pmp", &size);
	if (!rep->temp_sram) {
		dev_warn(rep->dev, "cannot map the PMP SRAM for its temperatures\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(apple_pmp_t6030_temps); i++) {
		if (apple_pmp_t6030_temps[i].offset + T6030_PMP_TEMP_RECORD_SIZE > size) {
			iounmap(rep->temp_sram);
			rep->temp_sram = NULL;
			dev_warn(rep->dev, "PMP SRAM too small for its temperature records\n");
			return;
		}
	}
	if (devm_add_action_or_reset(rep->dev, apple_pmp_t6030_temp_unmap, rep->temp_sram)) {
		rep->temp_sram = NULL;
		return;
	}

	hwmon = devm_hwmon_device_register_with_info(rep->dev, "apple_pmp", rep,
						     &apple_pmp_t6030_temp_chip, NULL);
	if (IS_ERR(hwmon))
		dev_warn(rep->dev, "cannot register the PMP temperatures: %pe\n", hwmon);
	apple_pmp_t6030_zone_register(rep);

	ret = apple_pmp_t6030_temp_hotspot(rep, &value);
	if (ret)
		dev_info(rep->dev, "PMP temperatures registered; hotspot not readable yet (%d)\n", ret);
	else
		dev_info(rep->dev, "PMP temperatures registered; CPU hotspot %ld mC (zone %s)\n",
			 value, T6030_PMP_TEMP_ZONE);
}

static void apple_pmp_t6030_start(struct work_struct *work)
{
	struct apple_pmp_report *rep = container_of(work, struct apple_pmp_report, start_work);
	u64 status = 0, request, ack = 0;
	unsigned long deadline;
	int ret;

	/* 5: nothing may have changed since the requests were seeded */
	ret = apple_pmp_t6030_recheck(rep);
	if (ret) {
		apple_pmp_t6030_finish(rep, ret);
		return;
	}
	/* From here the PMP may run, and only a reboot stops it. */
	dev_info(rep->dev, "starting the PMP with request %#llx\n", rep->seed);
	ret = apple_pmp_t6030_enable(rep);
	if (ret) {
		dev_err(rep->dev, "%s PMP not started: enabling its nodes failed: %d\n",
			rep->offsets->name, ret);
		apple_pmp_t6030_finish(rep, ret);
		return;
	}

	/* 6: bound means the firmware and its endpoint are up */
	deadline = jiffies + msecs_to_jiffies(PMP_START_BIND_TIMEOUT_MS);
	while (!apple_pmp_t6030_bound(rep) && time_before(jiffies, deadline))
		msleep(50);
	if (!apple_pmp_t6030_bound(rep)) {
		dev_err(rep->dev, "PMP driver did not bind within %d ms; display power not acknowledged\n",
			PMP_START_BIND_TIMEOUT_MS);
		apple_pmp_t6030_finish(rep, -ETIMEDOUT);
		return;
	}
	msleep(PMP_START_QUIET_MS);

	deadline = jiffies + msecs_to_jiffies(PMP_START_READY_TIMEOUT_MS);
	do {
		status = readq(rep->base + rep->offsets->status);
		if (status == PMP_REPORT_READY)
			break;
		msleep(50);
	} while (time_before(jiffies, deadline));
	if (status == PMP_REPORT_READY) {
		deadline = jiffies + msecs_to_jiffies(PMP_START_ACK_TIMEOUT_MS);
		do {
			ack = readq(rep->base + rep->offsets->actual);
			if ((ack & rep->startup_ack) == rep->startup_ack)
				break;
			msleep(20);
		} while (time_before(jiffies, deadline));
	}
	request = readq(rep->base + rep->offsets->tgt_read);

	if (status != PMP_REPORT_READY) {
		dev_err(rep->dev, "PMP started but not ready: PTD status %#llx request %#llx\n",
			status, request);
		ret = -ETIMEDOUT;
	} else if ((ack & rep->startup_ack) != rep->startup_ack) {
		dev_err(rep->dev, "PMP ready but did not acknowledge %#llx: request %#llx ack %#llx\n",
			rep->startup_ack, request, ack);
		ret = -EIO;
	} else {
		dev_info(rep->dev, "PMP ready: PTD status %#llx request %#llx ack %#llx\n",
			 status, request, ack);
		ret = 0;
		if (rep->offsets->temps)
			apple_pmp_t6030_temps_register(rep);
		/* The PMP has settled: ask it for the CPU clients' memory floor. */
		if (rep->floor_valid)
			schedule_delayed_work(&rep->floor_work, HZ);
	}
	apple_pmp_t6030_finish(rep, ret);
}

/* Queues the start once both the requests are seeded and a consumer asked. */
static void apple_pmp_t6030_request_start(struct apple_pmp_report *rep)
{
	guard(mutex)(&apple_pmp_report_mutex);
	if (!rep->stopping && READ_ONCE(rep->seeded) && atomic_read(&rep->start_requested) &&
	    !atomic_xchg(&rep->start_queued, 1))
		queue_work(system_unbound_wq, &rep->start_work);
}

static void apple_pmp_report_remove(struct platform_device *pdev)
{
	struct apple_pmp_report *rep = platform_get_drvdata(pdev);
	bool waiters;

	if (!rep->offsets->starts_pmp)
		return;

	scoped_guard(mutex, &apple_pmp_report_mutex) {
		rep->stopping = true;
		platform_set_drvdata(pdev, NULL);
		rep->start_result = -ENODEV;
		complete_all(&rep->started);
		waiters = rep->waiters;
	}
	/* No work or waiter may use the devres state after remove returns. */
	cancel_work_sync(&rep->start_work);
	cancel_delayed_work_sync(&rep->floor_work);
	if (waiters)
		wait_for_completion(&rep->waiters_done);
}

/*
 * apple_pmp_report_wait_ready() - wait until a PMP report entry is acknowledged
 * @entry: report entry node whose request must be acknowledged
 * @timeout: in jiffies
 *
 * For a report driver that starts the PMP itself: the first call starts it.
 * Returns 0 once the PMP runs and has acknowledged @entry, -EPROBE_DEFER
 * while the report driver has not probed yet, or the reason the PMP was not
 * started or is not ready.
 */
int apple_pmp_report_wait_ready(struct device_node *entry, unsigned long timeout)
{
	struct device_node *parent = of_get_parent(entry);
	struct platform_device *pdev;
	struct apple_pmp_report *rep;
	u64 status, ack;
	bool record;
	u32 id;
	int ret;

	if (!parent || of_property_read_u32(entry, "reg", &id) || id > 63) {
		of_node_put(parent);
		return -EINVAL;
	}
	/* Only M3 reports publish state under the startup lifetime lock. */
	if (!of_device_is_compatible(parent, "apple,t6030-pmp-v2-report") &&
	    !of_device_is_compatible(parent, "apple,t8122-pmp-v2-report") &&
	    !of_device_is_compatible(parent, "apple,t6031-pmp-v2-report")) {
		of_node_put(parent);
		return -EINVAL;
	}
	pdev = of_find_device_by_node(parent);
	of_node_put(parent);
	if (!pdev)
		return -EPROBE_DEFER;

	/* A device reference alone does not pin its driver's devres. */
	scoped_guard(mutex, &apple_pmp_report_mutex) {
		rep = platform_get_drvdata(pdev);
		if (!rep) {
			ret = -EPROBE_DEFER;
			goto out_put;
		}
		if (rep->stopping) {
			ret = -ENODEV;
			goto out_put;
		}
		if (!rep->offsets->starts_pmp ||
		    apple_pmp_report_bit(rep, entry, &id, &record)) {
			ret = -EINVAL;
			goto out_put;
		}
		rep->waiters++;
	}

	atomic_set(&rep->start_requested, 1);
	/* Pairs with the barrier after seeding: one side sees the other's flag. */
	smp_mb__after_atomic();
	apple_pmp_t6030_request_start(rep);
	ret = wait_for_completion_timeout(&rep->started, timeout) ? 0 : -ETIMEDOUT;
	scoped_guard(mutex, &apple_pmp_report_mutex) {
		if (rep->stopping) {
			ret = -ENODEV;
		} else if (!ret) {
			if (rep->start_result) {
				ret = rep->start_result;
			} else if (!(rep->ack & BIT_ULL(id))) {
				ret = -EINVAL;
			} else {
				status = readq(rep->base + rep->offsets->status);
				ack = readq(rep->base + rep->offsets->actual);
				ret = status == PMP_REPORT_READY && (ack & BIT_ULL(id)) ? 0 : -EIO;
			}
		}
		if (!--rep->waiters && rep->stopping)
			complete(&rep->waiters_done);
	}

out_put:
	put_device(&pdev->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_wait_ready);

/*
 * apple_pmp_report_check_ready() - whether a PMP report entry is acknowledged now
 * @entry: report entry node whose request must be acknowledged
 *
 * The non-blocking form of apple_pmp_report_wait_ready(): it never starts the
 * PMP and never waits. Returns 0 once the PMP runs and has acknowledged
 * @entry, -EAGAIN while it has not (the PMP not started yet, starting, or
 * the entry not acknowledged yet), -EPROBE_DEFER while the report driver
 * has not probed, -EINVAL for an entry this report cannot acknowledge, or
 * the reason the PMP was not started.
 */
int apple_pmp_report_check_ready(struct device_node *entry)
{
	struct device_node *parent = of_get_parent(entry);
	struct platform_device *pdev;
	struct apple_pmp_report *rep;
	u64 status, ack;
	bool record;
	u32 id;
	int ret;

	if (!parent || of_property_read_u32(entry, "reg", &id) || id > 63) {
		of_node_put(parent);
		return -EINVAL;
	}
	if (!of_device_is_compatible(parent, "apple,t6030-pmp-v2-report") &&
	    !of_device_is_compatible(parent, "apple,t8122-pmp-v2-report") &&
	    !of_device_is_compatible(parent, "apple,t6031-pmp-v2-report")) {
		of_node_put(parent);
		return -EINVAL;
	}
	pdev = of_find_device_by_node(parent);
	of_node_put(parent);
	if (!pdev)
		return -EPROBE_DEFER;

	scoped_guard(mutex, &apple_pmp_report_mutex) {
		rep = platform_get_drvdata(pdev);
		if (!rep) {
			ret = -EPROBE_DEFER;
		} else if (rep->stopping) {
			ret = -ENODEV;
		} else if (!rep->offsets->starts_pmp ||
			   apple_pmp_report_bit(rep, entry, &id, &record) ||
			   !(rep->ack & BIT_ULL(id))) {
			ret = -EINVAL;
		} else if (completion_done(&rep->started) && rep->start_result) {
			ret = rep->start_result;
		} else {
			status = readq(rep->base + rep->offsets->status);
			ack = readq(rep->base + rep->offsets->actual);
			ret = status == PMP_REPORT_READY && (ack & BIT_ULL(id)) ? 0 : -EAGAIN;
		}
	}
	put_device(&pdev->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_check_ready);

/* Select one available report entry by its reg (soc-device ID - 1). */
static struct device_node *apple_pmp_report_entry(struct device_node *report, u32 id)
{
	struct device_node *child, *entry = NULL;
	u32 reg;

	for_each_available_child_of_node(report, child) {
		if (!of_device_is_compatible(child, "apple,t6000-pmp-v2-report-entry") ||
		    of_property_count_u32_elems(child, "reg") != 1 ||
		    of_property_read_u32(child, "reg", &reg) || reg != id)
			continue;
		if (entry) {
			of_node_put(entry);
			of_node_put(child);
			return NULL;
		}
		entry = of_node_get(child);
	}
	return entry;
}

/* A current14 T8122 supplier starts through its own display report entry.
 * The entry consumer pins startup/removal and verifies both READY and DISP ACK.
 */
static int apple_pmp_report_wait_t8122_14(struct device *supplier, unsigned int timeout_ms)
{
	struct device_node *report, *pmp, *entry;
	int ret;

	if (!of_machine_is_compatible("apple,t8122") ||
	    !of_device_is_compatible(supplier->of_node, "apple,t8122-pmp-v2"))
		return -ENODEV;
	report = of_find_compatible_node(NULL, NULL, "apple,t8122-pmp-v2-report");
	if (!report)
		return -ENODEV;
	pmp = of_parse_phandle(report, "apple,pmp", 0);
	/*
	 * The display request entry, reg 7 (DISP). Its request bit comes from
	 * apple_pmp_report_bit(): the DISP record's index, 11 on J613.
	 */
	entry = apple_pmp_report_entry(report, 7);
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

	/* A declared native report always takes the unchanged native25 path below. */
	if (!np)
		return apple_pmp_report_wait_t8122_14(supplier, timeout_ms);
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

/* Bound published report regions, protected by RCU until managed teardown drains readers. */
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
 * Return: 0 on success, -EPROBE_DEFER if the region is not bound (yet) or
 * is not a published region (only T8140 regions are), -EOPNOTSUPP if the
 * SoC's PMP publishes no such effort, -EINVAL if @lane is out of range, or
 * -EAGAIN if the PMP does not report that it is running.
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

/* Read only the documented status and fast-die entries, never the aperture. */
static int apple_pmp_report_status_show(struct seq_file *s, void *unused)
{
	struct apple_pmp_report *rep = s->private;
	const struct apple_pmp_report_offsets *offs = rep->offsets;
	u32 status = readl(rep->base + offs->status);
	unsigned int lane;
	u32 effort;

	seq_printf(s, "running: %u\n", !!(status & PMP_REPORT_READY));
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

/*
 * With offsets->published: publish the region before its children are
 * populated, and withdraw it again on teardown, children first.
 */
static int apple_pmp_report_publish(struct apple_pmp_report *rep)
{
	struct device *dev = rep->dev;
	int ret;

	INIT_LIST_HEAD(&rep->list);
	ret = devm_add_action_or_reset(dev, apple_pmp_report_release, rep);
	if (ret)
		return ret;

	mutex_lock(&apple_pmp_reports_lock);
	list_add_tail_rcu(&rep->list, &apple_pmp_reports);
	/* Only SoCs with a fast die-temperature report have the directory. */
	if (rep->offsets->fast_die && !apple_pmp_report_debugfs)
		apple_pmp_report_debugfs = debugfs_create_dir("apple-pmp-report", NULL);
	mutex_unlock(&apple_pmp_reports_lock);

	if (rep->offsets->fast_die)
		rep->debugfs = debugfs_create_file(dev_name(dev), 0400,
						   apple_pmp_report_debugfs, rep,
						   &apple_pmp_report_status_fops);

	return 0;
}

static int apple_pmp_report_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct apple_pmp_report *rep;
	struct resource *res;
	u64 request;
	int ret;

	rep = devm_kzalloc(dev, sizeof(*rep), GFP_KERNEL);
	if (!rep)
		return -ENOMEM;

	rep->dev = dev;
	spin_lock_init(&rep->lock);
	rep->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rep->base))
		return PTR_ERR(rep->base);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rep->size = res ? resource_size(res) : 0;
	rep->offsets = of_device_get_match_data(dev);
	if (rep->offsets->starts_pmp) {
		init_completion(&rep->started);
		init_completion(&rep->waiters_done);
		INIT_WORK(&rep->start_work, apple_pmp_t6030_start);
		INIT_DELAYED_WORK(&rep->floor_work, apple_pmp_t6030_floor_work);
		ret = apple_pmp_t6030_check(rep);
		if (ret == -EPROBE_DEFER) {
			of_node_put(rep->pmp);
			return ret;
		}
		if (ret) {
			/* Stay bound, so consumers learn why and do not wait. */
			of_node_put(rep->pmp);
			rep->pmp = NULL;
			scoped_guard(mutex, &apple_pmp_report_mutex)
				dev_set_drvdata(dev, rep);
			apple_pmp_t6030_finish(rep, ret);
			return 0;
		}
	}
	if (rep->offsets->starts_pmp) {
		guard(mutex)(&apple_pmp_report_mutex);

		dev_set_drvdata(dev, rep);
	} else {
		dev_set_drvdata(dev, rep);
	}
	if (of_device_is_compatible(np, "apple,t8132-pmp-v2-report")) {
		ret = apple_pmp_temp_register(rep);
		if (ret)
			return ret;
	}
	if (rep->offsets->published) {
		/* Publish the region before child power callbacks start looking it up. */
		ret = apple_pmp_report_publish(rep);
		if (ret)
			return ret;
	}
	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret && !rep->offsets->starts_pmp)
		return dev_err_probe(dev, ret, "failed to create child devices\n");

	if (rep->offsets->starts_pmp) {
		/* 4: the always-on entries have seeded their requests */
		request = readq(rep->base + rep->offsets->tgt_read);
		if (ret || request != rep->seed) {
			dev_err(dev, "%s PMP not started: request read back %#llx, expected %#llx (%d)\n",
				rep->offsets->name, request, rep->seed, ret);
			apple_pmp_t6030_finish(rep, -EIO);
			return 0;
		}
		dev_info(dev, "request %#llx set; the PMP starts when a consumer asks for it\n",
			 request);
		WRITE_ONCE(rep->seeded, true);
		/* Pairs with the barrier in apple_pmp_report_wait_ready(). */
		smp_mb();
		apple_pmp_t6030_request_start(rep);
	}

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

static const struct apple_pmp_report_offsets apple_pmp_offsets_t8112 = {
	.tgt_read = 0xa00,
	.tgt_write = 0x10500,
	.actual = 0xa40,
	.status = 0x10,
};

static const struct apple_pmp_report_offsets apple_pmp_offsets_t8132 = {
	.tgt_read = 0x1880,
	.tgt_write = 0x10c40,
	.actual = 0x1900,
	.status = 0x10,
};

static const struct apple_pmp_report_offsets apple_pmp_offsets_t6030 = {
	.tgt_read = 0x1180,
	.tgt_write = 0x108c0,
	.actual = 0x11c0,
	.status = 0x10,
	.starts_pmp = true,
	.name = "T6030",
	.pwrstate = "apple,t6030-pmgr-pwrstate",
	.temps = true,
	.devices_ordered = true,
	.memory_floor = true,
};

/*
 * T8122, from its ADT ptd-range: SOC-DEV-PS-REQ at PTD index 0x100 (read
 * 0x1000, update 0x10000 + 0x800), SOC-DEV-PS-ACK at 0x108 (0x1080),
 * PMP-STATUS at 0x1 (0x10). Its soc-device records are not in id order.
 * The positions of its temperature records are not known, so none are
 * registered.
 */
static const struct apple_pmp_report_offsets apple_pmp_offsets_t8122 = {
	.tgt_read = 0x1000,
	.tgt_write = 0x10800,
	.actual = 0x1080,
	.status = 0x10,
	.starts_pmp = true,
	.name = "T8122",
	.pwrstate = "apple,t8122-pmgr-pwrstate",
};

/*
 * T6031, from the J516C ADT ptd-range: SOC-DEV-PS-REQ at PTD index 0x200
 * (read 0x2000, update 0x10000 + 0x1000), SOC-DEV-PS-ACK at 0x208 (0x2080),
 * PMP-STATUS at 0x1 (0x10): the T6020 apertures. Its soc-device records are
 * in id order, as on T6030. The positions of its temperature records are
 * not known, so none are registered.
 */
static const struct apple_pmp_report_offsets apple_pmp_offsets_t6031 = {
	.tgt_read = 0x2000,
	.tgt_write = 0x11000,
	.actual = 0x2080,
	.status = 0x10,
	.starts_pmp = true,
	.name = "T6031",
	.pwrstate = "apple,t6031-pmgr-pwrstate",
	.devices_ordered = true,
};

/* The fast die-temperature report is the low two bytes of entry 0x1c1. */
static const struct apple_pmp_report_offsets apple_pmp_offsets_t8140 = {
	.tgt_read = 0x1880,
	.tgt_write = 0x10c40,
	.actual = 0x18c0,
	.status = 0x10,
	.published = true,
	.fast_die = 0x1c10,
	.fast_die_lanes = 2,
};

/* Same PTD apertures; startup belongs to the native PMP supplier. */
static const struct apple_pmp_report_offsets apple_pmp_offsets_j613_25g83 = {
	.tgt_read = 0x1000,
	.tgt_write = 0x10800,
	.actual = 0x1080,
	.status = 0x10,
};

static const struct of_device_id apple_pmp_report_of_match[] = {
	{ .compatible = "apple,j613-25g83-pmp-report", .data = &apple_pmp_offsets_j613_25g83 },
	{ .compatible = "apple,t6000-pmp-v2-report", .data = &apple_pmp_offsets_t600x },
	{ .compatible = "apple,t6020-pmp-v2-report", .data = &apple_pmp_offsets_t602x },
	{ .compatible = "apple,t8112-pmp-v2-report", .data = &apple_pmp_offsets_t8112 },
	{ .compatible = "apple,t8132-pmp-v2-report", .data = &apple_pmp_offsets_t8132 },
	{ .compatible = "apple,t6030-pmp-v2-report", .data = &apple_pmp_offsets_t6030 },
	{ .compatible = "apple,t8122-pmp-v2-report", .data = &apple_pmp_offsets_t8122 },
	{ .compatible = "apple,t6031-pmp-v2-report", .data = &apple_pmp_offsets_t6031 },
	{ .compatible = "apple,t8140-pmp-v2-report", .data = &apple_pmp_offsets_t8140 },
	{}
};

static struct platform_driver apple_pmp_report_driver = {
	.probe = apple_pmp_report_probe,
	.remove = apple_pmp_report_remove,
	.driver = {
		.name = "apple-pmp-report",
		.of_match_table = apple_pmp_report_of_match,
	},
};

struct apple_pmp_report_entry {
	struct device *dev;
	struct generic_pm_domain genpd;
	u32 id;
	/* The PMP does not acknowledge this request. */
	bool no_ack;

	/*
	 * Only for an entry of a published report (T8140), which owns this
	 * storage and looks its report up by node: see apple_pmp_t8140_entry_probe().
	 */
	struct device_node *node;
	struct device_node *report_node;
	bool removed;
};

#define genpd_to_apple_pmp_report_entry(_genpd) \
	container_of(_genpd, struct apple_pmp_report_entry, genpd)

static int apple_pmp_report_request(struct apple_pmp_report *rep, u32 id,
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

static int apple_pmp_report_set_state(struct generic_pm_domain *genpd, bool enable)
{
	struct apple_pmp_report_entry *ent = genpd_to_apple_pmp_report_entry(genpd);
	struct apple_pmp_report *rep = dev_get_drvdata(ent->dev->parent);

	return apple_pmp_report_request(rep, ent->id, enable, ent->no_ack);
}

static int apple_pmp_report_entry_power_on(struct generic_pm_domain *genpd)
{
	return apple_pmp_report_set_state(genpd, true);
}

static int apple_pmp_report_entry_power_off(struct generic_pm_domain *genpd)
{
	return apple_pmp_report_set_state(genpd, false);
}

static int apple_pmp_t8140_entry_set_state(struct generic_pm_domain *genpd, bool enable)
{
	struct apple_pmp_report_entry *ent = genpd_to_apple_pmp_report_entry(genpd);
	struct apple_pmp_report *rep;
	int ret = -ENODEV;

	if (READ_ONCE(ent->removed))
		return ret;

	rcu_read_lock();
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->dev->of_node == ent->report_node) {
			ret = apple_pmp_report_request(rep, ent->id, enable, false);
			break;
		}
	}
	rcu_read_unlock();
	return ret;
}

static int apple_pmp_t8140_entry_power_on(struct generic_pm_domain *genpd)
{
	return apple_pmp_t8140_entry_set_state(genpd, true);
}

static int apple_pmp_t8140_entry_power_off(struct generic_pm_domain *genpd)
{
	return apple_pmp_t8140_entry_set_state(genpd, false);
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

	/* Only entries of a published report (T8140) are withdrawn. */
	if (!ent)
		return;

	WRITE_ONCE(ent->removed, true);
	of_genpd_del_provider(ent->node);
	if (pm_genpd_remove(&ent->genpd)) {
		/* The core still owns this domain. Keep its now inert storage alive. */
		dev_warn(&pdev->dev, "retaining busy power domain %s\n", ent->genpd.name);
		return;
	}
	apple_pmp_report_entry_free(ent);
}

/*
 * An entry of a published report (T8140). Its storage, name and node
 * references are its own, not the device's: if genpd refuses to remove a
 * busy domain, they stay alive with callbacks that return -ENODEV.
 */
static int apple_pmp_t8140_entry_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct apple_pmp_report_entry *ent;
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
		apple_pmp_t8140_entry_set_state(&ent->genpd, true);
	}

	ent->genpd.power_on = apple_pmp_t8140_entry_power_on;
	ent->genpd.power_off = apple_pmp_t8140_entry_power_off;

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

static int apple_pmp_report_entry_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	const struct apple_pmp_report *rep = dev_get_drvdata(dev->parent);
	struct apple_pmp_report_entry *ent;
	int ret;
	bool record;
	const char *name;
	struct of_phandle_iterator it;

	/* Entries of a published report (T8140) look it up by node. */
	if (rep->offsets->published)
		return apple_pmp_t8140_entry_probe(pdev);

	ent = devm_kzalloc(dev, sizeof(*ent), GFP_KERNEL);
	if (!ent)
		return -ENOMEM;

	ent->dev = dev;

	ret = apple_pmp_report_bit(dev_get_drvdata(dev->parent), node, &ent->id, &record);
	if (ret)
		return dev_err_probe(dev, ret, "no request bit (reg, soc-device record)\n");

	ret = of_property_read_string(node, "label", &name);
	if (ret < 0)
		return dev_err_probe(dev, ret, "missing label property\n");

	ent->no_ack = ((struct apple_pmp_report *)dev_get_drvdata(dev->parent))->offsets->starts_pmp &&
		of_property_read_bool(node, "apple,no-ack");
	if (of_property_read_bool(node, "apple,always-on")) {
		ent->genpd.flags |= GENPD_FLAG_ACTIVE_WAKEUP;
		apple_pmp_report_set_state(&ent->genpd, true);
	}

	ent->genpd.name = name;
	ent->genpd.power_on = apple_pmp_report_entry_power_on;
	ent->genpd.power_off = apple_pmp_report_entry_power_off;

	ret = pm_genpd_init(&ent->genpd, NULL, true);
	if (ret)
		return dev_err_probe(dev, ret, "pm_genpd_init failed\n");

	ret = of_genpd_add_provider_simple(node, &ent->genpd);
	if (ret)
		return dev_err_probe(dev, ret, "of_genpd_add_provider_simple failed\n");

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

	return 0;
err_remove:
	of_genpd_del_provider(node);
	pm_genpd_remove(&ent->genpd);
	return ret;
}

static const struct of_device_id apple_pmp_report_entry_of_match[] = {
	{ .compatible = "apple,t6000-pmp-v2-report-entry" },
	/* Probed through apple_pmp_t8140_entry_probe() by its report. */
	{ .compatible = "apple,t8140-pmp-v2-report-entry" },
	{}
};

static struct platform_driver apple_pmp_report_entry_driver = {
	.probe = apple_pmp_report_entry_probe,
	.remove = apple_pmp_report_entry_remove,
	.driver = {
		.name = "apple-pmp-report-entry",
		.of_match_table = apple_pmp_report_entry_of_match,
	},
};

MODULE_DEVICE_TABLE(of, apple_pmp_report_of_match);
MODULE_DEVICE_TABLE(of, apple_pmp_report_entry_of_match);

static int __init apple_pmp_report_init(void)
{
	platform_driver_register(&apple_pmp_report_entry_driver);
	platform_driver_register(&apple_pmp_report_driver);
	return 0;
}

static void __exit apple_pmp_report_exit(void)
{
	platform_driver_unregister(&apple_pmp_report_entry_driver);
	platform_driver_unregister(&apple_pmp_report_driver);
	debugfs_remove_recursive(apple_pmp_report_debugfs);
}

module_init(apple_pmp_report_init);
module_exit(apple_pmp_report_exit);

MODULE_DESCRIPTION("PMP power state reporting driver for Apple SoCs");
MODULE_LICENSE("Dual MIT/GPL");
