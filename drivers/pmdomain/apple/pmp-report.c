// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMP power state reporting driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/kstrtox.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_domain.h>
#include <linux/ratelimit.h>
#include <linux/reboot.h>
#include <linux/rculist.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/apple/pmp-report.h>
#include <linux/soc/apple/t8132.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/unaligned.h>
#include <linux/uuid.h>
#include <linux/workqueue.h>

#include "pmp-agx-vote.h"
#include "pmp-report-validation.h"
#include "pmp-dram.h"
#include "pmp-ane.h"
#include "pmp-j613-floor-gov.h"

#define PMP_REPORT_READY 0x1

/* How often a report that defers requests checks whether the PMP runs. */
#define PMP_REPORT_POLL_MS	100
#define PMP_REPORT_POLL_SLOW_MS	1000
#define PMP_REPORT_FAST_POLLS	300

/**
 * struct apple_pmp_report_offsets - per-SoC layout of the report region
 * @tgt_read: power state request, read side
 * @tgt_write: power state request, write side
 * @actual: power state acknowledgment
 * @status: PMP status; bit 0 is set while the PMP is running
 * @fast_die: 32-bit word holding the fast die-temperature effort of each
 *	CPU cluster, one byte per lane; 0 if the SoC has none
 * @fast_die_lanes: number of lanes at @fast_die, at most 4
 * @running_any: the PMP runs while any bit of the 64-bit status word is set,
 *	not only bit 0
 * @defer_requests: keep power state requests in memory until the PMP runs,
 *	then write them all at once; nothing is written to a stopped PMP
 * @starts_pmp: this driver starts a halted PMP that a display gate hands
 *	over to the report (see apple_pmp_start_init())
 * @name: with @starts_pmp, the SoC in messages
 * @fallback: with @starts_pmp, how to boot without starting the PMP
 * @pwrstate: with @starts_pmp, the compatible of the SoC's PMGR power states
 * @j613_floor_default: enable the demand-driven floor on the native J613 report
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
	/* Update aperture of every entry (8 bytes each) and SOC-DEV-DVFS range. */
	u32 update;
	u32 dvfs_first, dvfs_count;
	bool running_any;
	bool defer_requests;
	bool starts_pmp;
	const char *name;
	const char *fallback;
	const char *pwrstate;
	bool devices_ordered;
	bool j613_floor_default;
};

struct apple_pmp_start;
struct j613_floor;

struct apple_pmp_report {
	struct device *dev;
	struct apple_pmp_dram dram;
	struct apple_pmp_ane ane;
	struct device *ane_owner;
	const struct apple_pmp_report_offsets *offsets;
	void __iomem *base;
	/* Protects the request word, @shadow, @known and @defer. */
	spinlock_t lock;
	struct list_head list;
	struct dentry *debugfs;
	/* Set only for a report whose halted PMP this driver starts. */
	struct apple_pmp_start *start;
	/* Requests made through entries: bits requested on, all bits. */
	u64 shadow;
	u64 known;
	/* Bits whose drivers withdraw them at the sleep's commit, with the PMP still running. */
	u64 commit_owned;
	/*
	 * Requests are held in memory until the PMP runs (@defer_requests), and
	 * while it's stopped for a system sleep.
	 */
	bool defer;
	/* The request word when a system sleep stopped the PMP. */
	u64 sleep_request;
	/* The last SOC-DEV-DVFS word written to each entry, 0 if none. */
	u64 *dvfs_shadow;
	unsigned int polls;
	struct delayed_work ready_work;
	struct device_node *native_pmp;
	u64 native_claimed;
	/* J613 floor owner, absent on all other report instances. */
	struct j613_floor *j613;
	/* T6050: the boot-time PTD entries (apple,ptd-boot) are written */
	bool t6050_boot_done;
};

static bool apple_pmp_report_running(const struct apple_pmp_report *rep)
{
	u64 status = readq(rep->base + rep->offsets->status);

	return rep->offsets->running_any ? status : status & PMP_REPORT_READY;
}

static ssize_t type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct apple_pmp_report *rep = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", rep->dram.type);
}
static DEVICE_ATTR_RO(type);

#define APPLE_PMP_DRAM_ATTR(_name, _field, _format) \
static ssize_t _name##_show(struct device *dev, struct device_attribute *attr, char *buf) \
{ \
	struct apple_pmp_report *rep = dev_get_drvdata(dev); \
	return sysfs_emit(buf, _format "\n", rep->dram._field); \
} \
static DEVICE_ATTR_RO(_name)

APPLE_PMP_DRAM_ATTR(data_bus_width, width, "%u");
APPLE_PMP_DRAM_ATTR(state_count, state_count, "%u");
APPLE_PMP_DRAM_ATTR(max_data_rate, max_rate, "%llu");
APPLE_PMP_DRAM_ATTR(peak_bandwidth, peak_bandwidth, "%llu");
APPLE_PMP_DRAM_ATTR(bandwidth_class, bandwidth_class, "%llu");

static struct attribute *apple_pmp_dram_attrs[] = {
	&dev_attr_type.attr,
	&dev_attr_data_bus_width.attr,
	&dev_attr_state_count.attr,
	&dev_attr_max_data_rate.attr,
	&dev_attr_peak_bandwidth.attr,
	&dev_attr_bandwidth_class.attr,
	NULL,
};

static const struct attribute_group apple_pmp_dram_group = {
	.name = "dram",
	.attrs = apple_pmp_dram_attrs,
};

static const struct apple_pmp_report_offsets apple_pmp_offsets_t6030;
static const struct apple_pmp_report_offsets apple_pmp_offsets_t600x;

/* T6001 AGX request (see the engine below set_state): AGX is soc-device record 9 on the T6001. */
#define APPLE_PMP_T6001_AGX_ID 9
static bool agx_vote = true;
static struct agx_vote apple_pmp_agx_vote_state = {.idle_ms = AGX_VOTE_IDLE_MS_DEFAULT};
static DEFINE_SPINLOCK(apple_pmp_agx_vote_lock);
/* Serializes the vote work, the unbind release and the report's removal. */
static DEFINE_MUTEX(apple_pmp_agx_vote_mutex);
static void apple_pmp_agx_vote_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(apple_pmp_agx_vote_work, apple_pmp_agx_vote_fn);
static void apple_pmp_agx_vote_stop(void);
static void apple_pmp_agx_vote_stop_new(void);

/* Bound report regions, protected by RCU until managed teardown drains readers. */
static LIST_HEAD(apple_pmp_reports);
static DEFINE_MUTEX(apple_pmp_reports_lock);

/* The report lock also protects ANE_SYS against a genpd owner appearing. */
static int apple_pmp_ane_request(void *ctx, bool enable)
{
	struct apple_pmp_report *rep = ctx;
	struct apple_pmp_ane *ane = &rep->ane;
	const u64 bit = BIT_ULL(PMP_ANE_ID);
	unsigned long flags;
	u64 val;

	spin_lock_irqsave(&rep->lock, flags);
	val = readq(rep->base + rep->offsets->tgt_read);
	if (enable) {
		if ((rep->known | val) & bit) {
			spin_unlock_irqrestore(&rep->lock, flags);
			return -EBUSY;
		}
		ane->owned = true;
		val |= bit;
	} else {
		val &= ~bit;
	}
	writeq(val, rep->base + rep->offsets->tgt_write);
	spin_unlock_irqrestore(&rep->lock, flags);
	if (!enable) {
		int ret = readq_poll_timeout(rep->base + rep->offsets->actual, val,
					    !(val & bit), 200, 50000);

		if (!ret) {
			spin_lock_irqsave(&rep->lock, flags);
			ane->owned = false;
			spin_unlock_irqrestore(&rep->lock, flags);
		}
		return ret;
	}
	return readq_poll_timeout(rep->base + rep->offsets->actual, val,
				 val & bit, 200, 50000);
}

/* Probe-time availability check: deliberately no MMIO while the ANE is off. */
int apple_pmp_report_ane_available(const struct device_node *np)
{
	struct apple_pmp_report *rep;
	int ret = -EPROBE_DEFER;

	mutex_lock(&apple_pmp_reports_lock);
	list_for_each_entry(rep, &apple_pmp_reports, list) {
		if (rep->dev->of_node != np)
			continue;
		ret = !rep->ane.dvfs ? -EOPNOTSUPP :
		      (rep->ane_owner || rep->ane.reserved) ? -EBUSY : 0;
		break;
	}
	mutex_unlock(&apple_pmp_reports_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_ane_available);

/**
 * apple_pmp_report_ane_perf() - request a T600x ANE performance level
 * @np: report provider node
 * @owner: ANE device holding the request
 * @level: 0 to release, 1 through 5 to request a level
 * @held: whether the caller still owns a request, including failed rollback
 *
 * The caller must keep its engine and the report SRAM powered throughout
 * the request, and link itself to the report supplier. Other generations
 * and reports without an ANE DVFS resource are unsupported. The returned
 * level is a firmware request, not a measured frequency.
 *
 * Context: Process context. May sleep.
 * Return: 0 on success, or a negative errno. Ownership and ACK failures
 * are returned to the submitter; no CPU frequency request is substituted.
 */
int apple_pmp_report_ane_perf(const struct device_node *np,
			      struct device *owner, unsigned int level, bool *held)
{
	struct apple_pmp_report *rep;
	int ret = -EPROBE_DEFER;

	if (!np || !owner || !held)
		return -EINVAL;
	*held = false;
	mutex_lock(&apple_pmp_reports_lock);
	list_for_each_entry(rep, &apple_pmp_reports, list) {
		if (rep->dev->of_node != np)
			continue;
		if (!rep->ane.dvfs) {
			ret = -EOPNOTSUPP;
			break;
		}
		if (rep->ane_owner && rep->ane_owner != owner) {
			ret = -EBUSY;
			break;
		}
		if (level && !apple_pmp_report_running(rep)) {
			ret = -EAGAIN;
			break;
		}
		/* Claim before touching DVFS, and exclude genpd for the whole cycle. */
		if (level && !rep->ane.reserved) {
			unsigned long flags;
			u64 req;

			spin_lock_irqsave(&rep->lock, flags);
			req = readq(rep->base + rep->offsets->tgt_read);
			if ((rep->known | req) & BIT_ULL(PMP_ANE_ID)) {
				spin_unlock_irqrestore(&rep->lock, flags);
				ret = -EBUSY;
				break;
			}
			rep->ane.reserved = true;
			spin_unlock_irqrestore(&rep->lock, flags);
		}
		ret = apple_pmp_ane_set(&rep->ane, level, apple_pmp_ane_request, rep);
		if (!rep->ane.level && !rep->ane.owned) {
			unsigned long flags;

			spin_lock_irqsave(&rep->lock, flags);
			rep->ane.reserved = false;
			spin_unlock_irqrestore(&rep->lock, flags);
		}
		/* Failed rollback retains ownership until a successful retry. */
		if (rep->ane.level || rep->ane.owned)
			rep->ane_owner = owner;
		else
			rep->ane_owner = NULL;
		*held = rep->ane_owner == owner;
		break;
	}
	mutex_unlock(&apple_pmp_reports_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_ane_perf);

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
		} else if (!apple_pmp_report_running(rep)) {
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
	bool running = apple_pmp_report_running(rep);
	unsigned int lane;
	u32 effort;

	seq_printf(s, "running: %u\n", running);
	if (rep->start)
		apple_pmp_start_show(s, rep->start);
	if (!running || !offs->fast_die)
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

	cancel_delayed_work_sync(&rep->ready_work);
	/* Withdraw child providers while their power callbacks can still use MMIO. */
	of_platform_depopulate(rep->dev);
	if (rep->offsets == &apple_pmp_offsets_t600x) {
		apple_pmp_agx_vote_stop_new();
		cancel_delayed_work_sync(&apple_pmp_agx_vote_work);
		guard(mutex)(&apple_pmp_agx_vote_mutex);
		apple_pmp_agx_vote_stop();
	}
	mutex_lock(&apple_pmp_reports_lock);
	if (!list_empty(&rep->list))
		list_del_rcu(&rep->list);
	mutex_unlock(&apple_pmp_reports_lock);
	synchronize_rcu();
	/* The debugfs proxy drains active reads and rejects subsequent accesses. */
	debugfs_remove(rep->debugfs);
	of_node_put(rep->native_pmp);
}

/*
 * T6050: the PTD entries the PMP's memory-cache channel and PPM-TO-SMC need at boot, which no
 * Linux driver otherwise writes, from the DT (apple,ptd-boot: entry and value pairs, where the
 * boot loader puts them). An entry is written at the update page (+0x10000, 8 bytes each) and
 * read at the read aperture (16 bytes each). Entry 0xfff waits for the channel's echo of the
 * value in the entry apple,ptd-echo-entry names; entry 0xffe reads the entry the value names.
 */
#define PTD_W(e)		(0x10000 + 8 * (e))
#define PTD_R(e)		(16 * (e))
#define T6050_PTD_WAIT		0xfff
#define T6050_PTD_READ		0xffe
#define T6050_PTD_ENTRIES	0x400
#define T6050_MC_FIRST_MS	3000
#define T6050_MC_STEP_MS	100

static void apple_pmp_report_t6050_boot(struct apple_pmp_report *rep)
{
	struct device_node *np = rep->dev->of_node;
	int n = of_property_count_u64_elems(np, "apple,ptd-boot");
	unsigned int i, waited = 0;
	u64 *seq, val = 0;
	u32 echo = T6050_PTD_ENTRIES;
	int ret = 0;

	if (rep->t6050_boot_done || !of_device_is_compatible(np, "apple,t6050-pmp-v2-report"))
		return;
	rep->t6050_boot_done = true;
	if (n <= 0)
		return;
	if (n % 2) {
		dev_err(rep->dev, "apple,ptd-boot is not in pairs\n");
		return;
	}
	seq = kcalloc(n, sizeof(*seq), GFP_KERNEL);
	if (!seq)
		return;
	if (of_property_read_u64_array(np, "apple,ptd-boot", seq, n))
		goto out;
	/* A sequence that waits needs its echo entry: without it, none of it is written. */
	of_property_read_u32(np, "apple,ptd-echo-entry", &echo);
	for (i = 0; echo >= T6050_PTD_ENTRIES && i < n; i += 2) {
		if (seq[i] == T6050_PTD_WAIT) {
			dev_warn(rep->dev, "PTD boot entries wait with no echo entry: skipped\n");
			goto out;
		}
	}
	for (i = 0; i < n; i += 2) {
		u64 e = seq[i], want = seq[i + 1];

		if (e == T6050_PTD_WAIT) {
			ret = readq_poll_timeout(rep->base + PTD_R(echo), val, val == want,
						 1000, (waited++ ? T6050_MC_STEP_MS :
							T6050_MC_FIRST_MS) * 1000);
			if (ret)
				break;
		} else if (e == T6050_PTD_READ && want < T6050_PTD_ENTRIES) {
			readq(rep->base + PTD_R(want));
		} else if (e < T6050_PTD_ENTRIES) {
			writeq(want, rep->base + PTD_W(e));
		}
	}
	if (ret)
		dev_err(rep->dev, "PTD boot entries: no echo of %llu at step %u (echo %#llx)\n",
			seq[i + 1], i / 2, val);
	else
		dev_info(rep->dev, "PTD boot entries written (%d steps)\n", n / 2);
out:
	kfree(seq);
}

/*
 * Wait for the PMP to run, then hand it every request made so far, as macOS
 * does for each PMP-notified device that is already powered on.
 */
static void apple_pmp_report_ready_work(struct work_struct *work)
{
	struct apple_pmp_report *rep = container_of(to_delayed_work(work),
						    struct apple_pmp_report,
						    ready_work);
	const struct apple_pmp_report_offsets *offs = rep->offsets;
	unsigned long flags;
	u64 val, want;

	if (!apple_pmp_report_running(rep)) {
		unsigned int ms = rep->polls++ < PMP_REPORT_FAST_POLLS ?
				  PMP_REPORT_POLL_MS : PMP_REPORT_POLL_SLOW_MS;

		queue_delayed_work(system_freezable_power_efficient_wq,
				   &rep->ready_work, msecs_to_jiffies(ms));
		return;
	}

	spin_lock_irqsave(&rep->lock, flags);
	val = readq(rep->base + offs->tgt_read);
	val = apple_pmp_request_word(val, rep->known, rep->shadow);
	writeq(val, rep->base + offs->tgt_write);
	want = rep->shadow;
	rep->defer = false;
	spin_unlock_irqrestore(&rep->lock, flags);

	if (want && readq_poll_timeout(rep->base + offs->actual, val,
				       (val & want) == want, 1000, 1000000))
		dev_warn(rep->dev, "PMP did not acknowledge requests %#llx (ack %#llx)\n",
			 want, val);
	dev_info(rep->dev, "PMP running (status %#llx), requests %#llx\n",
		 readq(rep->base + offs->status), want);
	apple_pmp_report_t6050_boot(rep);
}

/* A report names its PMP where a SoC has more than one kind of link to it. */
static bool apple_pmp_report_of(const struct apple_pmp_report *rep, const struct device *pmp)
{
	struct device_node *np = of_parse_phandle(rep->dev->of_node, "apple,pmp", 0);
	bool match = !np || np == pmp->of_node;

	of_node_put(np);
	return match;
}

/**
 * apple_pmp_report_sleep() - withdraw and hold requests before a system sleep stops the PMP
 * @pmp: the PMP device, still running
 * @at_commit: the PMP stops at the sleep's commit, so requests whose drivers withdraw them
 *	there are left set
 *
 * The devices behind these requests have stopped by now, so the PMP is told before
 * it goes. Requests made from here on only update the books, so nothing waits on an
 * acknowledgment from a stopped PMP. Bits no entry owns are left alone: nothing here
 * knows whether their devices have stopped.
 */
void apple_pmp_report_sleep(struct device *pmp, bool at_commit)
{
	struct apple_pmp_report *rep;
	unsigned long flags;
	u64 val, withdrawn;

	guard(mutex)(&apple_pmp_reports_lock);
	list_for_each_entry(rep, &apple_pmp_reports, list) {
		const struct apple_pmp_report_offsets *offs = rep->offsets;

		if (!apple_pmp_report_of(rep, pmp))
			continue;
		spin_lock_irqsave(&rep->lock, flags);
		rep->sleep_request = readq(rep->base + offs->tgt_read);
		withdrawn = rep->sleep_request & rep->known & ~(at_commit ? rep->commit_owned : 0);
		if (withdrawn)
			writeq(rep->sleep_request & ~withdrawn, rep->base + offs->tgt_write);
		rep->defer = true;
		spin_unlock_irqrestore(&rep->lock, flags);

		if (withdrawn && readq_poll_timeout(rep->base + offs->actual, val,
						    !(val & withdrawn), 100, 50000))
			dev_warn(rep->dev, "PMP still acknowledges withdrawn requests %#llx (ack %#llx)\n",
				 withdrawn, val);
	}
}
EXPORT_SYMBOL_GPL(apple_pmp_report_sleep);

/**
 * apple_pmp_report_wake() - hand the PMP every request after a system sleep
 * @pmp: the PMP device, running again
 * @lost_power: the sleep reached the hardware, so the report may have lost its
 *	contents: bits no entry owns come from the word kept at the sleep
 *
 * The power requests go first and the performance requests after them, the
 * order an entry uses when it powers on.
 */
void apple_pmp_report_wake(struct device *pmp, bool lost_power)
{
	struct apple_pmp_report *rep;
	unsigned long flags;
	unsigned int i;
	u64 val, want;

	guard(mutex)(&apple_pmp_reports_lock);
	list_for_each_entry(rep, &apple_pmp_reports, list) {
		const struct apple_pmp_report_offsets *offs = rep->offsets;

		if (!apple_pmp_report_of(rep, pmp))
			continue;
		spin_lock_irqsave(&rep->lock, flags);
		val = lost_power ? rep->sleep_request : readq(rep->base + offs->tgt_read);
		writeq(apple_pmp_request_word(val, rep->known, rep->shadow),
		       rep->base + offs->tgt_write);
		for (i = 0; rep->dvfs_shadow && i < offs->dvfs_count; i++)
			if (rep->dvfs_shadow[i])
				writeq(rep->dvfs_shadow[i],
				       rep->base + offs->update + 8 * (offs->dvfs_first + i));
		want = rep->shadow;
		rep->defer = false;
		spin_unlock_irqrestore(&rep->lock, flags);

		if (want && readq_poll_timeout(rep->base + offs->actual, val,
					       (val & want) == want, 1000, 1000000))
			dev_warn(rep->dev, "PMP did not acknowledge requests %#llx (ack %#llx)\n",
				 want, val);
	}
}
EXPORT_SYMBOL_GPL(apple_pmp_report_wake);

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
	bool t8132 = apple_t8132_firmware_path() &&
		of_device_is_compatible(supplier->of_node, "apple,t8132-pmp-v2");
	int ret;

	/* A declared report always takes the existing native25 path below. */
	if (!np && !of_property_present(supplier->of_node, "apple,pmp-report")) {
		if (of_machine_is_compatible("apple,t8122"))
			return apple_pmp_report_wait_t8122_14(supplier, timeout_ms);
		return apple_pmp_report_wait_t6030_14(supplier, timeout_ms);
	}
	if (!np || !of_device_is_compatible(np, t8132 ?
		"apple,t8132-pmp-v2-report" : "apple,j613-25g83-pmp-report")) {
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
					t8132 ? (status & PMP_REPORT_READY) : status == PMP_REPORT_READY,
					1000, timeout_ms * 1000);
	device_unlock(&pdev->dev);
	put_device(&pdev->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_wait_supplier_ready);

/**
 * apple_pmp_report_wait_running() - wait until the PMP behind a report runs
 * @report_np: report region node
 * @timeout_ms: how long to wait
 *
 * While its firmware starts, the PMP takes over the power management of the
 * SoC's coprocessors, and a coprocessor CPU released before then can be
 * stopped by it. A driver that releases such a CPU at boot waits here first.
 *
 * Context: process context; sleeps.
 * Return: 0 once the PMP runs, -ENODEV for a report that is not available,
 * -EPROBE_DEFER while the report is not bound, -ETIMEDOUT otherwise.
 */
int apple_pmp_report_wait_running(struct device_node *report_np, unsigned int timeout_ms)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	struct platform_device *pdev;
	int ret;

	if (!report_np || !of_device_is_available(report_np))
		return -ENODEV;
	pdev = of_find_device_by_node(report_np);
	if (!pdev)
		return -EPROBE_DEFER;
	for (;;) {
		struct apple_pmp_report *rep;

		ret = -EPROBE_DEFER;
		/* Never wait behind the report's removal. */
		if (device_trylock(&pdev->dev)) {
			rep = device_is_bound(&pdev->dev) ? platform_get_drvdata(pdev) : NULL;
			if (rep)
				ret = apple_pmp_report_running(rep) ? 0 : -ETIMEDOUT;
			device_unlock(&pdev->dev);
		}
		if (!ret || time_after(jiffies, deadline))
			break;
		usleep_range(10000, 20000);
	}
	put_device(&pdev->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_wait_running);

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
static int apple_pmp_report_record(struct device_node *pmp,
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
	if (!pmp)
		return -EINVAL;
	table = of_get_property(pmp, "apple,tunable-soc-device", &len);
	index = apple_pmp_record_of_id(table, len > 0 ? len : 0, reg + 1);
	if (index < 0 || index > 63)
		return -ENOENT;
	*bit = index;
	return 0;
}

static int apple_pmp_report_bit(const struct apple_pmp_start *st,
				struct device_node *entry, u32 *bit, bool *record)
{
	return apple_pmp_report_record(st ? st->pmp : NULL, entry, bit, record);
}

/* The native PMP owns startup; this report still exclusively owns PTD votes. */
static int apple_pmp_native_records(struct apple_pmp_report *rep)
{
	struct device_node *pmp, *link;
	const u8 *table, *group, *ranges;
	const char *uuid;
	int ntable, ngroup, nranges;

	{
		struct device_node *chosen = of_find_node_by_path("/chosen");
		const char *opt = NULL;
		int len = 0;
		bool selected;

		if (chosen)
			opt = of_get_property(chosen, "asahi,j613-25g83-dcpext", &len);
		selected = opt && len == 2 && opt[0] == '1' && !opt[1] &&
			of_device_is_compatible(rep->dev->of_node, "apple,j613-25g83-pmp-report");
		of_node_put(chosen);
		if (!selected)
			return 0;
	}
	if (!of_machine_is_compatible("apple,j613") ||
	    !of_device_is_compatible(rep->dev->of_node, "apple,j613-25g83-pmp-report"))
		return -ENODEV;
	for_each_compatible_node(pmp, NULL, "apple,t8122-pmp-v2") {
		link = of_parse_phandle(pmp, "apple,pmp-report", 0);
		if (link != rep->dev->of_node || !of_device_is_available(pmp)) {
			of_node_put(link);
			continue;
		}
		of_node_put(link);
		if (rep->native_pmp) {
			of_node_put(pmp);
			return -EINVAL;
		}
		rep->native_pmp = of_node_get(pmp);
	}
	pmp = rep->native_pmp;
	if (!pmp || of_property_read_string(pmp, "apple,tunable-uuid", &uuid) ||
	    strcmp(uuid, "EFD60284-7C58-33A9-8522-CB3366AF6040"))
		return -EINVAL;
	table = of_get_property(pmp, "apple,tunable-soc-device", &ntable);
	group = of_get_property(pmp, "apple,tunable-soc-device-ps-group", &ngroup);
	ranges = of_get_property(pmp, "apple,tunable-ptd-range", &nranges);
	if (!table || ntable != 20 * PMP_SOC_DEVICE_SIZE || !group || ngroup != 8 ||
	    get_unaligned_le64(group) != 0x3fff0 ||
	    !apple_pmp_ranges_valid(ranges, nranges, 0x1000, 0x10800, 0x1080) ||
	    apple_pmp_record_of_id(table, ntable, 8) != 11 ||
	    apple_pmp_record_of_id(table, ntable, 9) != 12 ||
	    apple_pmp_record_of_id(table, ntable, 18) != 19 ||
	    memcmp(table + 11 * PMP_SOC_DEVICE_SIZE + 0x74, "DISP\0\0\0\0", 8) ||
	    memcmp(table + 12 * PMP_SOC_DEVICE_SIZE + 0x74, "DISPEXT\0", 8) ||
	    memcmp(table + 19 * PMP_SOC_DEVICE_SIZE + 0x74, "ANS\0\0\0\0\0", 8) ||
	    !apple_pmp_records_valid(table, ntable, BIT_ULL(11) | BIT_ULL(12) | BIT_ULL(19),
				     BIT_ULL(11) | BIT_ULL(12)))
		return -EINVAL;
	dev_dbg(rep->dev, "native26 PTD admitted: DISP11 DISPEXT12 ANS19 (no ANS ack)\n");
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
			queue_work(system_dfl_wq, &st->work);
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
				if (rep->native_pmp) {
					ret = apple_pmp_report_record(rep->native_pmp, entry, &id, &record);
					if (!ret && (!record || id != 12))
						ret = -EINVAL;
					if (!ret && (!(readq(rep->base + rep->offsets->status) & 1) ||
						    !(readq(rep->base + rep->offsets->actual) & BIT_ULL(id)) ||
						    !(readq(rep->base + rep->offsets->tgt_read) & BIT_ULL(id))))
						ret = -EIO;
					rcu_read_unlock();
					goto out;
				}
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

/*
 * J613 (T8122, 25G83 firmware) demand-driven CPU memory floor.
 *
 * Under Linux nothing asks the J613 PMP for more than the lowest DRAM and
 * fabric states, and a CPU or GPU copy measures about 20 GB/s. Like the
 * T6030 CPU clients' floor, a request on the second CPU-client entry of
 * SOC-DEV-DVFS (PACC0) raises it. Measured on J613: with a DCS 4 floor a
 * 4-thread CPU copy runs at 67 GB/s and a GPU copy at 62 GB/s, and with a
 * fabric 2 floor as well at 92 and 76 GB/s, against 21 and 19 without.
 *
 * By default the native J613 report owns that request entry. A
 * governor samples CPU and GPU busy time; on demand it writes the valid bit
 * with the DCS and fabric floors, and after a hold time with no demand it
 * writes level 0 with the valid bit. Every exit path (disable, fault,
 * suspend, remove, reboot/shutdown) writes that level 0. Nothing else is
 * written. The request entry is read once, before the first write, to refuse
 * another producer: a read of a request entry after a store may retire the
 * pending update before the PMP handles it. Acceptance is judged only from
 * the DVFS-STATE responses.
 */
static bool j613_floor = true;
module_param(j613_floor, bool, 0444);
MODULE_PARM_DESC(j613_floor, "J613: own the CPU memory-floor request and run its governor (default: on for the native J613 report; 0 disables)");
static unsigned int j613_floor_dcs = 4;
module_param(j613_floor_dcs, uint, 0444);
/* Fabric 2: 4-thread CPU copy 67 -> 92 GB/s, GPU copy 62 -> 76 GB/s over fabric 0. */
static unsigned int j613_floor_soc = 2;
module_param(j613_floor_soc, uint, 0444);
MODULE_PARM_DESC(j613_floor_soc, "J613 floor: fabric level while raised, 0-2 (default: 2)");
static unsigned int j613_floor_cpu_pct = 80;
module_param(j613_floor_cpu_pct, uint, 0444);
static unsigned int j613_floor_cpu_sum_pct = 90;
module_param(j613_floor_cpu_sum_pct, uint, 0444);
MODULE_PARM_DESC(j613_floor_cpu_sum_pct, "J613 floor: a cluster busy for this % of one CPU is demand (0: off, default: 90)");
static unsigned int j613_floor_gpu_pct = 40;
module_param(j613_floor_gpu_pct, uint, 0444);
static unsigned int j613_floor_raise_samples = 2;
module_param(j613_floor_raise_samples, uint, 0444);
static unsigned int j613_floor_hold_ms = 500;
module_param(j613_floor_hold_ms, uint, 0444);
static unsigned int j613_floor_min_interval_ms = 100;
module_param(j613_floor_min_interval_ms, uint, 0444);
static unsigned int j613_floor_sample_ms = 30;
module_param(j613_floor_sample_ms, uint, 0444);
static unsigned int j613_floor_confirm_ms = 1000;
module_param(j613_floor_confirm_ms, uint, 0444);
static unsigned int j613_floor_start_delay_ms = 10000;
module_param(j613_floor_start_delay_ms, uint, 0444);
static bool j613_floor_idle_defer;
module_param(j613_floor_idle_defer, bool, 0444);

#define J613_FLOOR_START_POLLS	600	/* 1 s each, for the PMP to report ready */

struct j613_floor {
	struct apple_pmp_report *rep;
	struct device *dev;
	struct device_node *pmp;
	void __iomem *update;	/* PACC0 update alias; the only address ever written */
	void __iomem *state;	/* DVFS-STATE word 0; words 1-3 follow at 16 bytes */
	/* Serializes every request write and all state below. */
	struct mutex lock;
	struct delayed_work start_work, work, idle_work;
	struct notifier_block reboot_nb;
	struct ratelimit_state rs;
	struct pmp_floor_gov gov;
	struct pmp_floor_gov_cfg cfg;
	struct pmp_floor_layout layout;
	unsigned int dcs, soc, sample_ms, confirm_ms;
	bool idle_defer;
	bool admitted, enabled, stopped, suspended, faulted;
	const char *why;	/* refusal or fault reason */
	unsigned int start_polls;
	u64 initial, word, writes;
	/* The transition awaiting its response, and its target. */
	bool confirm_raise, confirm_drop, held_valid;
	unsigned int tdcs, tsoc;
	u64 confirm_since, held_resp, resp[4], last_confirm_us;
	/* Sampling state. */
	bool primed;
	u64 *cpu_idle, *cpu_wall;
	unsigned int *cluster_sum;	/* per sample, indexed by the cluster's first CPU */
	u64 gpu_prev, last_ns;
	unsigned int cpu_pct, cpu_sum, gpu_pct, reason, raise_reason;
	/* Counters. */
	u64 samples, raises, drops, relevels, raise_ok, raise_unconfirmed;
	u64 drop_ok, drop_unconfirmed, superseded, held_ns, raised_at;
};

/* GPU busy state, pushed by the GPU driver on idle/busy edges. */
static struct {
	spinlock_t lock;
	bool busy;
	u64 since, busy_ns, edges;
	struct j613_floor *wake;
} j613_floor_gpu = { .lock = __SPIN_LOCK_UNLOCKED(j613_floor_gpu.lock) };

static void j613_floor_queue(struct j613_floor *f)
{
	unsigned long delay = msecs_to_jiffies(f->sample_ms);

	if (f->stopped || f->suspended || f->faulted || !f->enabled)
		return;
	/*
	 * While the floor is held, sample on time so that it drops on time.
	 * With idle_defer and nothing held, sampling is deferrable and waits
	 * for a CPU that wakes anyway; a GPU busy edge moves it back.
	 */
	if (f->idle_defer && !f->gov.raised) {
		cancel_delayed_work(&f->work);
		queue_delayed_work(system_freezable_power_efficient_wq, &f->idle_work, delay);
	} else {
		cancel_delayed_work(&f->idle_work);
		queue_delayed_work(system_freezable_power_efficient_wq, &f->work, delay);
	}
}

/**
 * apple_pmp_j613_floor_gpu_activity() - GPU work outstanding or retired
 * @busy: true once work may reach the GPU, false once all of it retired
 *
 * Called by the GPU driver on each idle/busy edge, in any context but NMI.
 * Records busy time for the J613 floor governor; a no-op without an owner.
 */
void apple_pmp_j613_floor_gpu_activity(bool busy)
{
	struct j613_floor *f;
	unsigned long flags;
	u64 now = ktime_get_ns();

	spin_lock_irqsave(&j613_floor_gpu.lock, flags);
	if (busy != j613_floor_gpu.busy) {
		if (busy)
			j613_floor_gpu.since = now;
		else
			j613_floor_gpu.busy_ns += now - j613_floor_gpu.since;
		j613_floor_gpu.busy = busy;
		j613_floor_gpu.edges++;
		f = j613_floor_gpu.wake;
		/* Not cancel_delayed_work_sync: this may run in atomic context. */
		if (busy && f && READ_ONCE(f->idle_defer) && cancel_delayed_work(&f->idle_work))
			queue_delayed_work(system_freezable_power_efficient_wq, &f->work,
					   msecs_to_jiffies(READ_ONCE(f->sample_ms)));
	}
	spin_unlock_irqrestore(&j613_floor_gpu.lock, flags);
}
EXPORT_SYMBOL_GPL(apple_pmp_j613_floor_gpu_activity);

static u64 j613_floor_gpu_busy_ns(u64 now, u64 *edges)
{
	unsigned long flags;
	u64 ns;

	spin_lock_irqsave(&j613_floor_gpu.lock, flags);
	ns = j613_floor_gpu.busy_ns;
	if (j613_floor_gpu.busy && now > j613_floor_gpu.since)
		ns += now - j613_floor_gpu.since;
	if (edges)
		*edges = j613_floor_gpu.edges;
	spin_unlock_irqrestore(&j613_floor_gpu.lock, flags);
	return ns;
}

/* DVFS-STATE reads have no side effect; only request reads do. */
static void j613_floor_read_state(struct j613_floor *f)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(f->resp); i++)
		f->resp[i] = readq(f->state + PMP_FLOOR_READ(i));
}

/* The only request store. Never followed by a read of the request word. */
static void j613_floor_write(struct j613_floor *f, u64 word)
{
	lockdep_assert_held(&f->lock);
	writeq(word, f->update);
	f->word = word;
	f->writes++;
}

static void j613_floor_expect(struct j613_floor *f, bool raise, u64 now)
{
	f->confirm_raise = raise;
	f->confirm_drop = !raise;
	f->confirm_since = now;
}

static void j613_floor_raise(struct j613_floor *f, u64 now, bool relevel)
{
	j613_floor_read_state(f);
	f->tdcs = f->dcs;
	f->tsoc = f->soc;
	f->held_valid = false;
	if (f->confirm_drop)
		f->superseded++;
	j613_floor_write(f, pmp_floor_word(f->dcs, f->soc));
	j613_floor_expect(f, true, now);
	if (relevel) {
		f->relevels++;
	} else {
		f->raises++;
		f->raised_at = now;
		f->raise_reason = f->reason;
	}
	if (__ratelimit(&f->rs))
		dev_info(f->dev, "J613 floor %s DCS%u SOC%u (%s%s cpu=%u%% sum=%u%% gpu=%u%%) word %#llx DVFS-STATE before %#llx\n",
			 relevel ? "relevel" : "raise", f->dcs, f->soc,
			 f->reason & 1 ? "cpu" : "", f->reason & 2 ? "gpu" : "",
			 f->cpu_pct, f->cpu_sum, f->gpu_pct, f->word, f->resp[0]);
}

/* Publish the explicit valid floor 0 if anything above it was written. */
static void j613_floor_release(struct j613_floor *f, u64 now, const char *why)
{
	bool was = f->gov.raised;

	lockdep_assert_held(&f->lock);
	if (was) {
		f->held_ns += now - f->raised_at;
		f->drops++;
	}
	pmp_floor_gov_reset(&f->gov, now);
	if (f->word == PMP_FLOOR_VALID || (!f->writes && pmp_floor_initial_ok(f->word)))
		return;
	if (f->confirm_raise)
		f->superseded++;
	j613_floor_write(f, PMP_FLOOR_VALID);
	j613_floor_expect(f, false, now);
	if (was && __ratelimit(&f->rs))
		dev_info(f->dev, "J613 floor release (%s) after %llu ms held, valid floor 0 published\n",
			 why, div_u64(now - f->raised_at, NSEC_PER_MSEC));
	else if (!was)
		dev_info(f->dev, "J613 floor release (%s), valid floor 0 published\n", why);
}

static void j613_floor_fault(struct j613_floor *f, u64 now, const char *why)
{
	f->faulted = true;
	f->why = why;
	j613_floor_release(f, now, why);
	dev_warn(f->dev, "J613 floor fault: %s; governor stopped, valid floor 0 published (DVFS-STATE %#llx/%#llx/%#llx/%#llx)\n",
		 why, f->resp[0], f->resp[1], f->resp[2], f->resp[3]);
}

/* Judge the last transition from the responses only. */
static void j613_floor_check(struct j613_floor *f, u64 now)
{
	bool timeout = now - f->confirm_since >= (u64)f->confirm_ms * NSEC_PER_MSEC;
	u64 r;

	if (!f->confirm_raise && !f->confirm_drop)
		return;
	j613_floor_read_state(f);
	r = f->resp[0];
	if (f->confirm_raise) {
		if (pmp_floor_raise_confirmed(r, f->tdcs, f->tsoc)) {
			f->confirm_raise = false;
			f->held_valid = true;
			f->held_resp = r;
			f->raise_ok++;
			f->last_confirm_us = div_u64(now - f->confirm_since, NSEC_PER_USEC);
			if (__ratelimit(&f->rs))
				dev_info(f->dev, "J613 floor DCS%u SOC%u confirmed in %llu us: DVFS-STATE %#llx/%#llx/%#llx/%#llx\n",
					 f->tdcs, f->tsoc, f->last_confirm_us,
					 f->resp[0], f->resp[1], f->resp[2], f->resp[3]);
		} else if (timeout) {
			f->confirm_raise = false;
			f->raise_unconfirmed++;
			j613_floor_fault(f, now,
					 "raise unconfirmed: DVFS-STATE did not reach the floor");
		}
		return;
	}
	if (f->held_valid ? pmp_floor_drop_confirmed(r, f->held_resp) :
			    !pmp_floor_raise_confirmed(r, f->tdcs, f->tsoc)) {
		f->confirm_drop = false;
		f->drop_ok++;
		f->last_confirm_us = div_u64(now - f->confirm_since, NSEC_PER_USEC);
		if (__ratelimit(&f->rs))
			dev_info(f->dev, "J613 floor release confirmed in %llu us: DVFS-STATE %#llx/%#llx/%#llx/%#llx\n",
				 f->last_confirm_us, f->resp[0], f->resp[1],
				 f->resp[2], f->resp[3]);
	} else if (pmp_floor_drop_timeout(now, f->confirm_since, 0, false,
					  f->confirm_ms)) {
		f->confirm_drop = false;
		f->drop_unconfirmed++;
		/* Floor 0 is already published; nothing further is written. */
		f->faulted = true;
		f->why = "release unconfirmed: DVFS-STATE did not change after floor 0";
		dev_warn(f->dev, "J613 floor fault: %s (%#llx/%#llx/%#llx/%#llx); governor stopped\n",
			 f->why, f->resp[0], f->resp[1], f->resp[2], f->resp[3]);
	}
}

static void j613_floor_prime(struct j613_floor *f)
{
	f->primed = false;
	f->gov.streak = 0;
}

static void j613_floor_sample_cpus(struct j613_floor *f, u64 period,
				   unsigned int *cpct, unsigned int *csum)
{
	unsigned int cpu;

	memset(f->cluster_sum, 0, nr_cpu_ids * sizeof(*f->cluster_sum));
	/* Sleepable worker only; exclude hotplug while reading idle counters.
	 * A policy reference keeps related_cpus alive even if it is unpublished
	 * during this sample. No policy pointer escapes the mask lookup.
	 */
	cpus_read_lock();
	for_each_possible_cpu(cpu)
		if (!cpu_online(cpu))
			f->cpu_wall[cpu] = 0;
	for_each_online_cpu(cpu) {
		u64 wall, idle = get_cpu_idle_time_us(cpu, NULL);

		/* Match CPUFreq io_busy=0 accounting without requiring CPU_FREQ. */
		idle += get_cpu_iowait_time_us(cpu, &wall);

		/* A CPU unseen for many periods (hotplug, long delay) is skipped once. */
		if (f->primed && f->cpu_wall[cpu] && wall > f->cpu_wall[cpu] &&
		    (wall - f->cpu_wall[cpu]) * NSEC_PER_USEC <= 10 * period) {
			unsigned int pct = pmp_floor_cpu_pct(f->cpu_idle[cpu], f->cpu_wall[cpu],
							     idle, wall);
			struct cpufreq_policy *pol = cpufreq_cpu_get(cpu);
			/* A cluster is a cpufreq policy; without one, all CPUs are one. */
			unsigned int key = pol ? cpumask_first(pol->related_cpus) : 0;

			if (pol)
				cpufreq_cpu_put(pol);

			*cpct = max(*cpct, pct);
			if (key < nr_cpu_ids) {
				f->cluster_sum[key] += pct;
				*csum = max(*csum, f->cluster_sum[key]);
			}
		}
		f->cpu_idle[cpu] = idle;
		f->cpu_wall[cpu] = wall;
	}
	cpus_read_unlock();
}

static void j613_floor_sample(struct j613_floor *f)
{
	u64 now = ktime_get_ns(), gpu, period = (u64)f->sample_ms * NSEC_PER_MSEC;
	unsigned int cpct = 0, csum = 0;
	enum pmp_floor_action act;

	f->samples++;
	j613_floor_sample_cpus(f, period, &cpct, &csum);
	gpu = j613_floor_gpu_busy_ns(now, NULL);
	f->cpu_pct = cpct;
	f->cpu_sum = csum;
	f->gpu_pct = f->primed ? pmp_floor_busy_pct(gpu - f->gpu_prev, now - f->last_ns) : 0;
	f->gpu_prev = gpu;
	f->last_ns = now;
	f->reason = (f->primed && pmp_floor_cpu_demand(&f->cfg, cpct, csum)) |
		    (f->primed && f->gpu_pct >= f->cfg.gpu_busy_pct) << 1;
	f->primed = true;

	j613_floor_check(f, now);
	if (f->faulted)
		return;
	act = pmp_floor_gov_step(&f->gov, &f->cfg, now, f->reason);
	if (act == PMP_FLOOR_RAISE) {
		j613_floor_raise(f, now, false);
	} else if (act == PMP_FLOOR_DROP) {
		/* The governor already left the raised state; account it here. */
		f->gov.raised = true;
		j613_floor_release(f, now, "idle");
	}
}

static void j613_floor_work(struct work_struct *work)
{
	struct j613_floor *f = container_of(to_delayed_work(work), struct j613_floor, work);

	mutex_lock(&f->lock);
	if (!f->stopped && !f->suspended && !f->faulted && f->enabled) {
		j613_floor_sample(f);
		j613_floor_queue(f);
	}
	mutex_unlock(&f->lock);
}

static void j613_floor_idle_work(struct work_struct *work)
{
	struct j613_floor *f = container_of(to_delayed_work(work), struct j613_floor, idle_work);

	mutex_lock(&f->lock);
	if (!f->stopped && !f->suspended && !f->faulted && f->enabled) {
		j613_floor_sample(f);
		j613_floor_queue(f);
	}
	mutex_unlock(&f->lock);
}

/* Validate this boot's tables and registers, then the single request read. */
static int j613_floor_admit(struct j613_floor *f)
{
	struct apple_pmp_report *rep = f->rep;
	struct resource *res;
	const u8 *dev, *rng;
	int dlen = 0, rlen = 0;
	u64 status;

	lockdep_assert_held(&f->lock);
	dev = of_get_property(f->pmp, "apple,tunable-soc-device", &dlen);
	rng = of_get_property(f->pmp, "apple,tunable-ptd-range", &rlen);
	f->why = pmp_floor_tables_check(dev, dlen > 0 ? dlen : 0, rng, rlen > 0 ? rlen : 0,
					&f->layout);
	if (f->why)
		return -EINVAL;
	res = platform_get_resource(to_platform_device(f->dev), IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < pmp_floor_layout_end(&f->layout)) {
		f->why = "report resource too small";
		return -EINVAL;
	}
	status = readq(rep->base + rep->offsets->status);
	if (status != PMP_REPORT_READY) {
		f->why = "PMP status not READY";
		return -EIO;
	}
	f->update = rep->base + pmp_floor_req_update(&f->layout);
	f->state = rep->base + pmp_floor_state_read(&f->layout, 0);
	/* The one read of the request entry, before any write. */
	f->initial = readq(rep->base + pmp_floor_req_read(&f->layout));
	f->word = f->initial;
	if (!pmp_floor_initial_ok(f->initial)) {
		f->why = "PACC0 request held by another producer";
		return -EBUSY;
	}
	j613_floor_read_state(f);
	return 0;
}

static void j613_floor_start_work(struct work_struct *work)
{
	struct j613_floor *f = container_of(to_delayed_work(work), struct j613_floor, start_work);
	u64 status;
	int ret;

	mutex_lock(&f->lock);
	if (f->stopped || f->suspended)
		goto out;
	status = readq(f->rep->base + f->rep->offsets->status);
	if (status != PMP_REPORT_READY) {
		if (++f->start_polls >= J613_FLOOR_START_POLLS) {
			f->why = "PMP never reported READY";
			dev_warn(f->dev, "J613 floor refused: %s (status %#llx)\n", f->why, status);
		} else {
			queue_delayed_work(system_freezable_power_efficient_wq, &f->start_work, HZ);
		}
		goto out;
	}
	if (f->start_polls != ~0U) {
		/* Ready: let the rest of the platform settle before owning the floor. */
		f->start_polls = ~0U;
		queue_delayed_work(system_freezable_power_efficient_wq, &f->start_work,
				   msecs_to_jiffies(j613_floor_start_delay_ms));
		goto out;
	}
	ret = j613_floor_admit(f);
	if (ret) {
		dev_warn(f->dev, "J613 floor refused: %s (%d); nothing written\n", f->why, ret);
		goto out;
	}
	f->admitted = true;
	f->why = NULL;
	f->enabled = true;
	dev_info(f->dev, "J613 floor owner admitted: PACC0 update +%#llx, initial %#llx, DVFS-STATE %#llx/%#llx/%#llx/%#llx; governor DCS%u SOC%u cpu>=%u%% gpu>=%u%% x%u hold %u ms\n",
		 pmp_floor_req_update(&f->layout), f->initial,
		 f->resp[0], f->resp[1], f->resp[2], f->resp[3], f->dcs, f->soc,
		 f->cfg.cpu_busy_pct, f->cfg.gpu_busy_pct, f->cfg.raise_samples, f->cfg.hold_ms);
	j613_floor_prime(f);
	j613_floor_queue(f);
out:
	mutex_unlock(&f->lock);
}

static void j613_floor_cancel(struct j613_floor *f)
{
	cancel_delayed_work_sync(&f->start_work);
	cancel_delayed_work_sync(&f->work);
	cancel_delayed_work_sync(&f->idle_work);
}

/* Stop for good (remove, reboot, shutdown): floor 0, no further sampling. */
static void j613_floor_stop(struct j613_floor *f, const char *why)
{
	unsigned long flags;

	spin_lock_irqsave(&j613_floor_gpu.lock, flags);
	if (j613_floor_gpu.wake == f)
		j613_floor_gpu.wake = NULL;
	spin_unlock_irqrestore(&j613_floor_gpu.lock, flags);
	mutex_lock(&f->lock);
	if (!f->stopped) {
		f->stopped = true;
		if (f->admitted)
			j613_floor_release(f, ktime_get_ns(), why);
	}
	mutex_unlock(&f->lock);
	j613_floor_cancel(f);
}

static void j613_floor_remove(void *data)
{
	j613_floor_stop(data, "remove");
}

static int j613_floor_reboot(struct notifier_block *nb, unsigned long action, void *data)
{
	j613_floor_stop(container_of(nb, struct j613_floor, reboot_nb), "reboot");
	return NOTIFY_DONE;
}

static struct j613_floor *j613_floor_of(struct device *dev)
{
	struct apple_pmp_report *rep = dev_get_drvdata(dev);

	return rep->j613;
}

/* Values are clamped to the range; a busy threshold of 101 turns that input off. */
#define J613_FLOOR_ATTR(_name, _field, _lo, _hi, _apply)				\
static ssize_t _name##_show(struct device *dev, struct device_attribute *a, char *buf)	\
{											\
	struct j613_floor *f = j613_floor_of(dev);					\
	return sysfs_emit(buf, "%u\n", READ_ONCE(f->_field));				\
}											\
static ssize_t _name##_store(struct device *dev, struct device_attribute *a,		\
			     const char *buf, size_t count)				\
{											\
	struct j613_floor *f = j613_floor_of(dev);					\
	unsigned int v;									\
	int ret = kstrtouint(buf, 0, &v);						\
	if (ret)									\
		return ret;								\
	mutex_lock(&f->lock);								\
	WRITE_ONCE(f->_field, pmp_floor_clamp(v, _lo, _hi));				\
	_apply;										\
	mutex_unlock(&f->lock);								\
	return count;									\
}											\
static DEVICE_ATTR_RW(_name)

/* A level change while the floor is held is written at once, serialized. */
#define J613_FLOOR_RELEVEL do {								\
	if (f->admitted && f->gov.raised && !f->faulted && !f->stopped &&		\
	    !f->suspended && (f->dcs != f->tdcs || f->soc != f->tsoc))			\
		j613_floor_raise(f, ktime_get_ns(), true);				\
} while (0)

J613_FLOOR_ATTR(dcs, dcs, PMP_FLOOR_DCS_MIN, PMP_FLOOR_DCS_MAX, J613_FLOOR_RELEVEL);
J613_FLOOR_ATTR(soc, soc, 0, PMP_FLOOR_SOC_MAX, J613_FLOOR_RELEVEL);
J613_FLOOR_ATTR(cpu_busy_pct, cfg.cpu_busy_pct, 1, 101, (void)0);
J613_FLOOR_ATTR(cpu_sum_pct, cfg.cpu_sum_pct, 0, 1600, (void)0);
J613_FLOOR_ATTR(gpu_busy_pct, cfg.gpu_busy_pct, 1, 101, (void)0);
J613_FLOOR_ATTR(raise_samples, cfg.raise_samples, 1, 20, (void)0);
J613_FLOOR_ATTR(hold_ms, cfg.hold_ms, 100, 10000, (void)0);
J613_FLOOR_ATTR(min_interval_ms, cfg.min_interval_ms, 0, 5000, (void)0);
J613_FLOOR_ATTR(sample_ms, sample_ms, 10, 1000, (void)0);
J613_FLOOR_ATTR(confirm_ms, confirm_ms, 100, 5000, (void)0);

static ssize_t enable_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct j613_floor *f = j613_floor_of(dev);

	return sysfs_emit(buf, "%u\n", READ_ONCE(f->enabled));
}

static ssize_t enable_store(struct device *dev, struct device_attribute *a,
			    const char *buf, size_t count)
{
	struct j613_floor *f = j613_floor_of(dev);
	bool on;
	int ret = kstrtobool(buf, &on);

	if (ret)
		return ret;
	mutex_lock(&f->lock);
	if (!f->admitted || f->faulted || f->stopped) {
		ret = -EPERM;
	} else if (on != f->enabled) {
		f->enabled = on;
		if (on) {
			j613_floor_prime(f);
			j613_floor_queue(f);
		} else {
			j613_floor_release(f, ktime_get_ns(), "disabled");
		}
	}
	mutex_unlock(&f->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(enable);

static ssize_t status_show(struct device *dev, struct device_attribute *a, char *buf)
{
	struct j613_floor *f = j613_floor_of(dev);
	u64 now = ktime_get_ns(), edges, gpu = j613_floor_gpu_busy_ns(now, &edges);
	u64 held;
	int n;

	mutex_lock(&f->lock);
	if (f->admitted && !f->stopped)
		j613_floor_read_state(f);
	held = f->held_ns + (f->gov.raised ? now - f->raised_at : 0);
	n = sysfs_emit(buf,
		"admitted=%u enabled=%u faulted=%u stopped=%u suspended=%u why=%s\n"
		"raised=%u dcs=%u soc=%u word=%#llx initial=%#llx writes=%llu\n"
		"dvfs_state=%#llx/%#llx/%#llx/%#llx\n"
		"raises=%llu drops=%llu relevels=%llu rate_limited=%llu superseded=%llu\n"
		"raise_confirmed=%llu raise_unconfirmed=%llu drop_confirmed=%llu drop_unconfirmed=%llu last_confirm_us=%llu\n"
		"held_ms_total=%llu held_ms_current=%llu last_raise_reason=%u\n"
		"samples=%llu cpu_pct=%u cpu_sum=%u gpu_pct=%u reason=%u gpu_busy_ms=%llu gpu_edges=%llu\n",
		f->admitted, f->enabled, f->faulted, f->stopped, f->suspended, f->why ?: "-",
		f->gov.raised, f->dcs, f->soc, f->word, f->initial, f->writes,
		f->resp[0], f->resp[1], f->resp[2], f->resp[3],
		f->raises, f->drops, f->relevels, f->gov.rate_limited, f->superseded,
		f->raise_ok, f->raise_unconfirmed, f->drop_ok, f->drop_unconfirmed,
		f->last_confirm_us,
		div_u64(held, NSEC_PER_MSEC),
		f->gov.raised ? div_u64(now - f->raised_at, NSEC_PER_MSEC) : 0, f->raise_reason,
		f->samples, f->cpu_pct, f->cpu_sum, f->gpu_pct, f->reason,
		div_u64(gpu, NSEC_PER_MSEC), edges);
	mutex_unlock(&f->lock);
	return n;
}
static DEVICE_ATTR_RO(status);

static struct attribute *j613_floor_attrs[] = {
	&dev_attr_enable.attr, &dev_attr_dcs.attr, &dev_attr_soc.attr,
	&dev_attr_cpu_busy_pct.attr, &dev_attr_cpu_sum_pct.attr, &dev_attr_gpu_busy_pct.attr,
	&dev_attr_raise_samples.attr, &dev_attr_hold_ms.attr,
	&dev_attr_min_interval_ms.attr, &dev_attr_sample_ms.attr,
	&dev_attr_confirm_ms.attr, &dev_attr_status.attr, NULL,
};

static const struct attribute_group j613_floor_group = {
	.name = "j613_floor",
	.attrs = j613_floor_attrs,
};

/*
 * Probe-time setup. The report driver data stays the report; the attribute
 * group reads the floor through the report's j613 pointer.
 */
static void j613_floor_put_pmp(void *pmp)
{
	of_node_put(pmp);
}

static int j613_floor_probe(struct apple_pmp_report *rep)
{
	struct device *dev = rep->dev;
	struct j613_floor *f;
	struct device_node *pmp, *link;
	const char *uuid;
	unsigned long flags;
	int ret;

	if (!pmp_floor_enabled(of_machine_is_compatible("apple,j613"),
			       rep->offsets->j613_floor_default, j613_floor))
		return 0;
	f = devm_kzalloc(dev, sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;
	/* Native PMP startup remains owned by its existing supplier driver. */
	for_each_compatible_node(pmp, NULL, "apple,t8122-pmp-v2") {
		link = of_parse_phandle(pmp, "apple,pmp-report", 0);
		if (link != dev->of_node || !of_device_is_available(pmp)) {
			of_node_put(link);
			continue;
		}
		of_node_put(link);
		if (f->pmp) {
			of_node_put(pmp);
			return -EINVAL;
		}
		f->pmp = of_node_get(pmp);
		ret = devm_add_action_or_reset(dev, j613_floor_put_pmp, f->pmp);
		if (ret) {
			of_node_put(pmp);
			return ret;
		}
	}
	if (!f->pmp || of_property_read_string(f->pmp, "apple,tunable-uuid", &uuid) ||
	    strcmp(uuid, PMP_FLOOR_J613_UUID))
		return dev_err_probe(dev, -EINVAL, "J613 floor PMP identity refused\n");
	f->cpu_idle = devm_kcalloc(dev, nr_cpu_ids, sizeof(u64), GFP_KERNEL);
	f->cpu_wall = devm_kcalloc(dev, nr_cpu_ids, sizeof(u64), GFP_KERNEL);
	f->cluster_sum = devm_kcalloc(dev, nr_cpu_ids, sizeof(unsigned int), GFP_KERNEL);
	if (!f->cpu_idle || !f->cpu_wall || !f->cluster_sum)
		return -ENOMEM;
	f->rep = rep;
	f->dev = dev;
	mutex_init(&f->lock);
	ratelimit_state_init(&f->rs, 10 * HZ, 20);
	ratelimit_set_flags(&f->rs, RATELIMIT_MSG_ON_RELEASE);
	INIT_DELAYED_WORK(&f->start_work, j613_floor_start_work);
	INIT_DELAYED_WORK(&f->work, j613_floor_work);
	INIT_DEFERRABLE_WORK(&f->idle_work, j613_floor_idle_work);
	f->dcs = pmp_floor_clamp(j613_floor_dcs, PMP_FLOOR_DCS_MIN, PMP_FLOOR_DCS_MAX);
	f->soc = pmp_floor_clamp(j613_floor_soc, 0, PMP_FLOOR_SOC_MAX);
	f->cfg.cpu_busy_pct = pmp_floor_clamp(j613_floor_cpu_pct, 1, 101);
	f->cfg.cpu_sum_pct = pmp_floor_clamp(j613_floor_cpu_sum_pct, 0, 1600);
	f->cfg.gpu_busy_pct = pmp_floor_clamp(j613_floor_gpu_pct, 1, 101);
	f->cfg.raise_samples = pmp_floor_clamp(j613_floor_raise_samples, 1, 20);
	f->cfg.hold_ms = pmp_floor_clamp(j613_floor_hold_ms, 100, 10000);
	f->cfg.min_interval_ms = pmp_floor_clamp(j613_floor_min_interval_ms, 0, 5000);
	f->sample_ms = pmp_floor_clamp(j613_floor_sample_ms, 10, 1000);
	f->confirm_ms = pmp_floor_clamp(j613_floor_confirm_ms, 100, 5000);
	f->idle_defer = j613_floor_idle_defer;
	rep->j613 = f;

	/* Removal order (reverse): stop the governor before the group goes. */
	ret = devm_device_add_group(dev, &j613_floor_group);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, j613_floor_remove, f);
	if (ret)
		return ret;
	f->reboot_nb.notifier_call = j613_floor_reboot;
	/* Before device shutdown: the PMP is still running then. */
	f->reboot_nb.priority = INT_MAX;
	ret = devm_register_reboot_notifier(dev, &f->reboot_nb);
	if (ret)
		return ret;
	spin_lock_irqsave(&j613_floor_gpu.lock, flags);
	j613_floor_gpu.wake = f;
	spin_unlock_irqrestore(&j613_floor_gpu.lock, flags);
	queue_delayed_work(system_freezable_power_efficient_wq, &f->start_work, HZ);
	dev_info(dev, "J613 floor: waiting for the PMP before admission\n");
	return 0;
}

static void apple_pmp_report_shutdown(struct platform_device *pdev)
{
	struct apple_pmp_report *rep = platform_get_drvdata(pdev);

	if (rep && rep->j613)
		j613_floor_stop(rep->j613, "shutdown");
}

/* System sleep: publish level 0 first, and sample again after resume. */
static int apple_pmp_report_suspend(struct device *dev)
{
	struct apple_pmp_report *rep = dev_get_drvdata(dev);
	struct j613_floor *f = rep ? rep->j613 : NULL;

	if (!f)
		return 0;
	mutex_lock(&f->lock);
	f->suspended = true;
	if (f->admitted && !f->stopped)
		j613_floor_release(f, ktime_get_ns(), "suspend");
	mutex_unlock(&f->lock);
	j613_floor_cancel(f);
	return 0;
}

static int apple_pmp_report_resume(struct device *dev)
{
	struct apple_pmp_report *rep = dev_get_drvdata(dev);
	struct j613_floor *f = rep ? rep->j613 : NULL;

	if (!f)
		return 0;
	mutex_lock(&f->lock);
	f->suspended = false;
	if (!f->stopped && !f->admitted && !f->why)
		queue_delayed_work(system_freezable_power_efficient_wq, &f->start_work, HZ);
	j613_floor_prime(f);
	j613_floor_queue(f);
	mutex_unlock(&f->lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_pmp_report_pm_ops, apple_pmp_report_suspend,
				apple_pmp_report_resume);

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
	if (of_property_match_string(np, "reg-names", "ane-dvfs") >= 0) {
		struct resource *res;

		if (!of_device_is_compatible(np, "apple,t6000-pmp-v2-report"))
			return -EINVAL;
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res || resource_size(res) != SZ_128K)
			return -EINVAL;
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ane-dvfs");
		if (!res || resource_size(res) != SZ_16K)
			return -EINVAL;
		rep->ane.dvfs = devm_ioremap_resource(dev, res);
		if (IS_ERR(rep->ane.dvfs))
			return PTR_ERR(rep->ane.dvfs);
		rep->ane.report = rep->base;
	}

	rep->defer = rep->offsets->defer_requests;
	if (rep->offsets->dvfs_count) {
		rep->dvfs_shadow = devm_kcalloc(dev, rep->offsets->dvfs_count,
						sizeof(*rep->dvfs_shadow), GFP_KERNEL);
		if (!rep->dvfs_shadow)
			return -ENOMEM;
	}
	INIT_DELAYED_WORK(&rep->ready_work, apple_pmp_report_ready_work);
	platform_set_drvdata(pdev, rep);

	ret = apple_pmp_dram_parse(dev_fwnode(dev), &rep->dram);
	if (ret && ret != -ENODATA)
		return dev_err_probe(dev, ret, "invalid DRAM description\n");
	if (!ret) {
		ret = devm_device_add_group(dev, &apple_pmp_dram_group);
		if (ret)
			return ret;
	}

	INIT_LIST_HEAD(&rep->list);
	ret = devm_add_action_or_reset(dev, apple_pmp_report_release, rep);
	if (ret)
		return ret;

	ret = apple_pmp_native_records(rep);
	if (ret)
		return dev_err_probe(dev, ret, "native26 PTD table refused\n");

	ret = j613_floor_probe(rep);
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

	/* A report that comes back (rebind) takes votes again; its removal refused them. */
	if (rep->offsets == &apple_pmp_offsets_t600x)
		apple_pmp_agx_activity_resume();

	if (rep->offsets->fast_die || rep->start)
		rep->debugfs = debugfs_create_file(dev_name(dev), 0400,
						   apple_pmp_report_debugfs, rep,
						   &apple_pmp_report_status_fops);

	/* Publish the region before child power callbacks start looking it up. */
	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create child devices\n");

	if (rep->defer)
		queue_delayed_work(system_freezable_power_efficient_wq,
				   &rep->ready_work, 0);

	return 0;
}

static const struct apple_pmp_report_offsets apple_pmp_offsets_t600x = {
	.tgt_read = 0xf80,
	.tgt_write = 0x107c0,
	.update = 0x10000,
	.dvfs_first = 0x108,
	.dvfs_count = 0x28,
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
	.j613_floor_default = true,
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
	.fallback = "apple_t6031_display.enable=0",
	.pwrstate = "apple,t6031-pmgr-pwrstate",
	.devices_ordered = true,
};

/*
 * J514M/T6034's selected ADT ptd-range has the same request (index 0x200),
 * acknowledge (0x208) and status (0x1) apertures. Its 35 soc-device records
 * are in ID order; DISP is ID 17 with ACK and ANS is ID 34 without ACK.
 * The exact T6034 display gate supplies the halted PMP and validated values.
 * The generic T6020 fallback only observes an already-running PMP.
 */
static const struct apple_pmp_report_offsets apple_pmp_offsets_t6034 = {
	.tgt_read = 0x2000,
	.tgt_write = 0x11000,
	.actual = 0x2080,
	.status = 0x10,
	.starts_pmp = true,
	.name = "T6034",
	.fallback = "apple_t6034_display.enable=0",
	.pwrstate = "apple,t6031-pmgr-pwrstate",
	.devices_ordered = true,
};

/* The fast die-temperature report is the low two bytes of entry 0x1c1. */
static const struct apple_pmp_report_offsets apple_pmp_offsets_t8140 = {
	.tgt_read = 0x1880,
	.tgt_write = 0x10c40,
	.update = 0x10000,
	.dvfs_first = 0x190,
	.dvfs_count = 16,
	.actual = 0x18c0,
	.status = 0x10,
	.fast_die = 0x1c10,
	.fast_die_lanes = 2,
};

/*
 * T6040 (M4 Pro, "t6041" die): PTD entries 1 (PMP-STATUS), 480
 * (SOC-DEV-PS-REQ), 488 (SOC-DEV-PS-ACK) and 600 (fast die-temperature
 * effort: E, P0, P1). Reads are 16 bytes per entry from the base, writes 8
 * bytes per entry from base + 0x10000. macOS treats any set bit of
 * PMP-STATUS as running, and writes no request before the PMP starts.
 */
static const struct apple_pmp_report_offsets apple_pmp_offsets_t6041 = {
	.tgt_read = 0x1e00,
	.tgt_write = 0x10f00,
	.actual = 0x1e80,
	.status = 0x10,
	.fast_die = 0x2580,
	.fast_die_lanes = 3,
	.running_any = true,
	.defer_requests = true,
};

static const struct apple_pmp_report_offsets apple_pmp_offsets_t8132 = {
	.tgt_read = 0x1880,
	.tgt_write = 0x10c40,
	.actual = 0x1900,
	.status = 0x10,
	/* No qualified fast-die effort on this generation. */
};

static const struct of_device_id apple_pmp_report_of_match[] = {
	{ .compatible = "apple,j613-25g83-pmp-report", .data = &apple_pmp_offsets_j613_25g83 },
	{ .compatible = "apple,t6000-pmp-v2-report", .data = &apple_pmp_offsets_t600x },
	{ .compatible = "apple,t6020-pmp-v2-report", .data = &apple_pmp_offsets_t602x },
	{ .compatible = "apple,t6030-pmp-v2-report", .data = &apple_pmp_offsets_t6030 },
	{ .compatible = "apple,t6031-pmp-v2-report", .data = &apple_pmp_offsets_t6031 },
	{ .compatible = "apple,t6041-pmp-v2-report", .data = &apple_pmp_offsets_t6041 },
	/* T6050: the ADT's ptd-range lists T6041's entries at the same places. */
	{ .compatible = "apple,t6050-pmp-v2-report", .data = &apple_pmp_offsets_t6041 },
	{ .compatible = "apple,t6034-pmp-v2-report", .data = &apple_pmp_offsets_t6034 },
	{ .compatible = "apple,t8112-pmp-v2-report", .data = &apple_pmp_offsets_t8112 },
	{ .compatible = "apple,t8122-pmp-v2-report", .data = &apple_pmp_offsets_t8122 },
	{ .compatible = "apple,t8132-pmp-v2-report", .data = &apple_pmp_offsets_t8132 },
	{ .compatible = "apple,t8140-pmp-v2-report", .data = &apple_pmp_offsets_t8140 },
	{}
};

static struct platform_driver apple_pmp_report_driver = {
	.probe = apple_pmp_report_probe,
	.shutdown = apple_pmp_report_shutdown,
	.driver = {
		.name = "apple-pmp-report",
		.of_match_table = apple_pmp_report_of_match,
		.pm = pm_sleep_ptr(&apple_pmp_report_pm_ops),
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
	/* Withdrawn for system sleep (apple,release-for-sleep), and now so. */
	bool sleep_release;
	bool released;
	/*
	 * With @dvfs, the SOC-DEV-DVFS entry this soc-device's performance
	 * request lives in, and the request word written while powered.
	 */
	bool dvfs;
	u32 dvfs_entry;
	u64 dvfs_off;
	u64 dvfs_on;
	bool removed;
	struct apple_pmp_report *native_owner;
};

#define genpd_to_apple_pmp_report_entry(_genpd) \
	container_of(_genpd, struct apple_pmp_report_entry, genpd)

static int apple_pmp_report_set_state(struct apple_pmp_report *rep, u32 id,
				      bool enable, bool no_ack)
{
	u64 bit_val = BIT_ULL(id);
	u64 val;
	bool latched = true;
	unsigned long flags;
	bool ready;

	spin_lock_irqsave(&rep->lock, flags);
	if (id == PMP_ANE_ID && rep->ane.reserved) {
		spin_unlock_irqrestore(&rep->lock, flags);
		return -EBUSY;
	}
	rep->known |= bit_val;
	if (enable)
		rep->shadow |= bit_val;
	else
		rep->shadow &= ~bit_val;
	ready = !rep->defer;
	if (ready) {
		val = readq(rep->base + rep->offsets->tgt_read);
		val &= ~bit_val;
		if (enable)
			val |= bit_val;
		writeq(val, rep->base + rep->offsets->tgt_write);
		if (rep->native_pmp)
			latched = !!(readq(rep->base + rep->offsets->tgt_read) & bit_val) == enable;
	}
	spin_unlock_irqrestore(&rep->lock, flags);
	if (!latched)
		return -EIO;
	if (!ready || !apple_pmp_report_running(rep) || no_ack)
		return 0;
	return readq_poll_timeout_atomic(
		rep->base + rep->offsets->actual,
		val,
		!!(val & bit_val) == !!enable,
		100,
		50000);
}

/*
 * T6001 AGX request. T2 showed that setting PS-REQ bit 9 (AGX, soc-device record 9) lifts the GPU read roof from 241 to 368 GB/s: the PMP
 * leaves memory at its idle floor for a GPU that is busy but has not asked. The GPU driver reports operations in flight
 * (apple_pmp_agx_activity_begin/end); the request is set while any operation runs and cleared agx_vote_idle_ms after the last one ends, at
 * GPU unbind and when this report goes away. The decisions are in pmp-agx-vote.h (unit tested); this is the glue.
 * On by default (pmp_report.agx_vote): it only writes while the PMP runs, so a board that leaves the PMP off is unchanged; with it off
 * nothing is written. T6000 has the same record and bit (soc-device 10, PS-REQ bit 9) as T6001.
 */
static unsigned int agx_vote_idle_ms = AGX_VOTE_IDLE_MS_DEFAULT;

static agx_ms_t apple_pmp_agx_now_ms(void)
{
	return jiffies_to_msecs(jiffies);
}

/*
 * Queue the reconcile step. agx_vote=1 on the kernel command line runs the parameter setter from parse_args(), before workqueue_init_early() has
 * created system_dfl_wq; queueing on the NULL workqueue would oops in start_kernel(). Nothing is owed then: begin() and the work function read the
 * parameter themselves, so the first GPU operation kicks.
 */
static void apple_pmp_agx_vote_kick(unsigned long delay_ms)
{
	struct workqueue_struct *wq = READ_ONCE(system_dfl_wq);

	if (!wq)
		return;
	mod_delayed_work(wq, &apple_pmp_agx_vote_work, msecs_to_jiffies(delay_ms));
}

static int apple_pmp_agx_vote_param_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (!ret)
		apple_pmp_agx_vote_kick(0);	/* switching off clears at once, switching on votes if the GPU is busy */
	return ret;
}

static const struct kernel_param_ops apple_pmp_agx_vote_ops = {
	.set = apple_pmp_agx_vote_param_set,
	.get = param_get_bool,
};
module_param_cb(agx_vote, &apple_pmp_agx_vote_ops, &agx_vote, 0644);
MODULE_PARM_DESC(agx_vote, "T6000/T6001: set the AGX power request in the PMP report while the GPU has work in flight (default: on)");
module_param(agx_vote_idle_ms, uint, 0644);
MODULE_PARM_DESC(agx_vote_idle_ms, "Keep the AGX request this many ms after the last GPU operation ends (default: 250)");

/* Write the AGX request of the bound T6000/T6001 report: -ENODEV without one, -EAGAIN (nothing written) when the PMP does not run. */
static int apple_pmp_agx_write(bool enable, bool no_ack)
{
	struct apple_pmp_report *rep;
	int ret = -ENODEV;

	if (!of_machine_is_compatible("apple,t6000") && !of_machine_is_compatible("apple,t6001"))
		return ret;
	rcu_read_lock();
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->offsets != &apple_pmp_offsets_t600x)
			continue;
		if (enable && !(readq(rep->base + rep->offsets->status) & PMP_REPORT_READY))
			ret = -EAGAIN;
		else
			ret = apple_pmp_report_set_state(rep, APPLE_PMP_T6001_AGX_ID, enable, no_ack);
		break;
	}
	rcu_read_unlock();
	return ret;
}

/* Refuse new votes (the stopped flag), without writing anything. */
static void apple_pmp_agx_vote_stop_new(void)
{
	unsigned long flags;

	spin_lock_irqsave(&apple_pmp_agx_vote_lock, flags);
	agx_vote_refuse(&apple_pmp_agx_vote_state);
	spin_unlock_irqrestore(&apple_pmp_agx_vote_lock, flags);
}

/* Clear a request that is still set. Caller holds apple_pmp_agx_vote_mutex. */
static void apple_pmp_agx_vote_stop(void)
{
	unsigned long flags;
	bool voted;

	spin_lock_irqsave(&apple_pmp_agx_vote_lock, flags);
	voted = agx_vote_release(&apple_pmp_agx_vote_state);
	spin_unlock_irqrestore(&apple_pmp_agx_vote_lock, flags);
	if (voted)
		apple_pmp_agx_write(false, true);
}

static void apple_pmp_agx_vote_fn(struct work_struct *work)
{
	struct agx_vote *v = &apple_pmp_agx_vote_state;
	enum agx_vote_action act;
	unsigned long flags;
	agx_ms_t now, next = 0;
	int ret = 0;

	guard(mutex)(&apple_pmp_agx_vote_mutex);
	spin_lock_irqsave(&apple_pmp_agx_vote_lock, flags);
	v->enabled = READ_ONCE(agx_vote);
	v->idle_ms = READ_ONCE(agx_vote_idle_ms);
	now = apple_pmp_agx_now_ms();
	act = agx_vote_decide(v, now);
	if (act == AGX_VOTE_NONE && v->failures && !v->users && !v->voted &&
	    agx_vote_retry_reached(v, now))
		v->failures = 0;	/* the backoff fired with nobody waiting: it is spent */
	spin_unlock_irqrestore(&apple_pmp_agx_vote_lock, flags);

	if (act != AGX_VOTE_NONE) {
		const bool set = act == AGX_VOTE_SET;

		ret = apple_pmp_agx_write(set, false);
		if (ret == -ETIMEDOUT) {
			pr_err("pmp_report: AGX request %s not acknowledged, clearing\n", set ? "set" : "clear");
			apple_pmp_agx_write(false, true);
		}
		if (!set)
			ret = 0;	/* a clear is final: the state is "not voted" whatever happened */
		spin_lock_irqsave(&apple_pmp_agx_vote_lock, flags);
		agx_vote_applied(v, act, ret, now);
		next = agx_vote_next(v, apple_pmp_agx_now_ms());
		spin_unlock_irqrestore(&apple_pmp_agx_vote_lock, flags);
	} else {
		spin_lock_irqsave(&apple_pmp_agx_vote_lock, flags);
		next = agx_vote_next(v, now);
		spin_unlock_irqrestore(&apple_pmp_agx_vote_lock, flags);
	}
	if (next)
		apple_pmp_agx_vote_kick(next);
}

void apple_pmp_agx_activity_begin(void)
{
	struct agx_vote *v = &apple_pmp_agx_vote_state;
	unsigned long flags;
	agx_ms_t delay;

	spin_lock_irqsave(&apple_pmp_agx_vote_lock, flags);
	v->enabled = READ_ONCE(agx_vote);
	delay = agx_vote_begin(v, apple_pmp_agx_now_ms());
	spin_unlock_irqrestore(&apple_pmp_agx_vote_lock, flags);
	if (delay)
		apple_pmp_agx_vote_kick(delay);
}
EXPORT_SYMBOL_GPL(apple_pmp_agx_activity_begin);

void apple_pmp_agx_activity_end(void)
{
	struct agx_vote *v = &apple_pmp_agx_vote_state;
	unsigned long flags;
	agx_ms_t delay;

	spin_lock_irqsave(&apple_pmp_agx_vote_lock, flags);
	v->enabled = READ_ONCE(agx_vote);
	v->idle_ms = READ_ONCE(agx_vote_idle_ms);
	delay = agx_vote_end(v, apple_pmp_agx_now_ms());
	spin_unlock_irqrestore(&apple_pmp_agx_vote_lock, flags);
	if (delay)
		apple_pmp_agx_vote_kick(delay);
}
EXPORT_SYMBOL_GPL(apple_pmp_agx_activity_end);

/*
 * The GPU driver unbinds: clear the request now instead of waiting out the idle time, so it is never left set past the GPU's
 * power-domain release (the T6030 fix clears before it releases its vote). Process context.
 */
void apple_pmp_agx_activity_release(void)
{
	/* Stop first so that a begin() from now on cannot queue a set, then wait out a work item that already decided, then clear what it set. */
	apple_pmp_agx_vote_stop_new();
	cancel_delayed_work_sync(&apple_pmp_agx_vote_work);
	guard(mutex)(&apple_pmp_agx_vote_mutex);
	apple_pmp_agx_vote_stop();
}
EXPORT_SYMBOL_GPL(apple_pmp_agx_activity_release);

/* The GPU probes again, the report comes back or the system resumes: allow votes, and vote now if operations are already in flight. Process context. */
void apple_pmp_agx_activity_resume(void)
{
	struct agx_vote *v = &apple_pmp_agx_vote_state;
	unsigned long flags;
	bool kick;

	spin_lock_irqsave(&apple_pmp_agx_vote_lock, flags);
	v->enabled = READ_ONCE(agx_vote);
	kick = agx_vote_resume(v);
	spin_unlock_irqrestore(&apple_pmp_agx_vote_lock, flags);
	if (kick)
		apple_pmp_agx_vote_kick(0);
}
EXPORT_SYMBOL_GPL(apple_pmp_agx_activity_resume);

/*
 * System sleep: withdraw the request before the devices suspend and decide again after resume. Whether bit 9 held through s2idle costs power is not
 * measured (a held display request cost about 0.25 W on J514S), and the PMP may drop the word across sleep, which would leave voted true with nothing
 * on the wire. Clearing and re-voting covers both.
 */
static int apple_pmp_agx_pm_notify(struct notifier_block *nb, unsigned long action, void *data)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		apple_pmp_agx_activity_release();
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
		apple_pmp_agx_activity_resume();
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block apple_pmp_agx_pm_nb = {
	.notifier_call = apple_pmp_agx_pm_notify,
};

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
 * One 64-bit store of the whole request word. Never read a request entry
 * back after storing it: the read retires the pending PTD update before the
 * PMP handles it, and the request is lost.
 */
static void apple_pmp_report_dvfs_write(struct apple_pmp_report *rep, u32 entry, u64 val)
{
	unsigned long flags;

	spin_lock_irqsave(&rep->lock, flags);
	rep->dvfs_shadow[entry - rep->offsets->dvfs_first] = val;
	if (!rep->defer)
		writeq(val, rep->base + rep->offsets->update + 8 * entry);
	spin_unlock_irqrestore(&rep->lock, flags);
}

/**
 * apple_pmp_report_set_request() - set or clear one bit of the PMP power request word of a bound report
 * @report_np: device tree node of the report region (compatible apple,t6000-pmp-v2-report and its
 *	siblings)
 * @id: bit number of the request, the index of the soc-device record, 0 to 63 (ANE0 is 10 and AGX
 *	is 9 on the T6001)
 * @enable: set or clear the bit
 * @no_ack: do not wait for the acknowledge
 *
 * One read-modify-write of the word, made under the report's lock: the same lock every report
 * entry and the AGX vote take, so requests for different bits cannot lose each other. Unless
 * @no_ack, waits (atomic poll, at most 50 ms) until the acknowledge word shows the bit as
 * requested. A PMP that does not run has nothing to acknowledge: the request is written and 0 is
 * returned, as for a report entry.
 *
 * The caller owns the bit it requests; this function does not arbitrate between callers of the
 * same bit. On the T6001 bit 9 belongs to the AGX vote (pmp_report.agx_vote) and is refused here
 * with -EBUSY. After -ETIMEDOUT the request was written but not acknowledged and stays in the
 * word: the caller must clear it (@enable false, @no_ack true) before giving up.
 *
 * Context: process context (the acknowledge wait is a busy poll of up to 50 ms).
 * Return: 0, -ENODEV if no report is bound to @report_np, -EINVAL if @id is out of range,
 * -EBUSY for the T6001 AGX bit, -ETIMEDOUT if the acknowledge did not follow.
 */
int apple_pmp_report_set_request(const struct device_node *report_np, unsigned int id,
				bool enable, bool no_ack)
{
	struct apple_pmp_report *rep;
	int ret = -ENODEV;

	if (id > 63)
		return -EINVAL;
	rcu_read_lock();
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->dev->of_node == report_np) {
			if (id == APPLE_PMP_T6001_AGX_ID && rep->offsets == &apple_pmp_offsets_t600x)
				ret = -EBUSY;	/* the AGX vote owns this bit */
			else
				ret = apple_pmp_report_set_state(rep, id, enable, no_ack);
			break;
		}
	}
	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_set_request);

static int apple_pmp_report_entry_set_state(struct generic_pm_domain *genpd, bool enable)
{
	struct apple_pmp_report_entry *ent = genpd_to_apple_pmp_report_entry(genpd);
	struct apple_pmp_report *rep;
	int ret = -ENODEV;

	if (READ_ONCE(ent->removed))
		return ret;

	rcu_read_lock();
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->dev->of_node != ent->report_node)
			continue;
		/* The device request is admitted first and the level dropped first. */
		if (ent->dvfs && !enable)
			apple_pmp_report_dvfs_write(rep, ent->dvfs_entry, ent->dvfs_off);
		ret = apple_pmp_report_set_state(rep, ent->id, enable, ent->no_ack);
		if (ent->dvfs && enable && !ret)
			apple_pmp_report_dvfs_write(rep, ent->dvfs_entry, ent->dvfs_on);
		break;
	}
	rcu_read_unlock();
	return ret;
}

/**
 * apple_pmp_report_entry_request() - set or withdraw one entry's request
 * @entry: the report entry's node
 * @enable: make the request, or withdraw it
 *
 * For a driver that switches its device's power itself around a system sleep. The
 * PMP may use a device whose request is set, so that driver withdraws the request
 * before its power goes and makes it again after its power is back. A withdrawn
 * request isn't replayed after the PMP restarts.
 *
 * Return: 0, also with no report bound or the PMP not running, or the error from
 * waiting for its acknowledgment.
 */
int apple_pmp_report_entry_request(const struct device_node *entry, bool enable)
{
	struct device_node *report;
	struct apple_pmp_report *rep;
	u32 id;
	int ret = 0;

	if (of_property_read_u32(entry, "reg", &id))
		return -EINVAL;

	report = of_get_parent(entry);
	rcu_read_lock();
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->dev->of_node != report)
			continue;
		ret = apple_pmp_report_set_state(rep, id, enable, false);
		break;
	}
	rcu_read_unlock();
	of_node_put(report);

	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_entry_request);

static struct apple_pmp_report *apple_pmp_report_of_entry(const struct device_node *entry,
							  u32 *id)
{
	struct device_node *report;
	struct apple_pmp_report *rep, *found = NULL;

	if (of_property_read_u32(entry, "reg", id) || *id >= 64)
		return NULL;
	report = of_get_parent(entry);
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->dev->of_node == report) {
			found = rep;
			break;
		}
	}
	of_node_put(report);

	return found;
}

/**
 * apple_pmp_report_entry_commit_owned() - mark an entry withdrawn by its driver at the commit
 * @entry: the report entry's node
 * @owned: its driver withdraws it at the sleep's commit, or no longer does
 *
 * For a device whose stop moves to the sleep's commit: a PMP that stops there too keeps the
 * request through apple_pmp_report_sleep(), until apple_pmp_report_entry_withdraw_polled().
 *
 * Return: 0, or -ENODEV with no report bound for the entry.
 */
int apple_pmp_report_entry_commit_owned(const struct device_node *entry, bool owned)
{
	struct apple_pmp_report *rep;
	unsigned long flags;
	int ret = -ENODEV;
	u32 id;

	rcu_read_lock();
	rep = apple_pmp_report_of_entry(entry, &id);
	if (rep) {
		spin_lock_irqsave(&rep->lock, flags);
		if (owned)
			rep->commit_owned |= BIT_ULL(id);
		else
			rep->commit_owned &= ~BIT_ULL(id);
		spin_unlock_irqrestore(&rep->lock, flags);
		ret = 0;
	}
	rcu_read_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_entry_commit_owned);

/**
 * apple_pmp_report_entry_withdraw_polled() - withdraw an entry's request at the sleep's commit
 * @entry: the report entry's node
 *
 * Interrupts off, one CPU: the request goes even with requests held for the sleep, and the
 * PMP's acknowledgment is polled for while it runs. One already withdrawn returns at once.
 *
 * Return: 0, also with no report bound, or -ETIMEDOUT with no acknowledgment.
 */
int apple_pmp_report_entry_withdraw_polled(const struct device_node *entry)
{
	struct apple_pmp_report *rep;
	unsigned long flags;
	u64 bit, val;
	int ret = 0;
	u32 id;

	rcu_read_lock();
	rep = apple_pmp_report_of_entry(entry, &id);
	if (rep) {
		bit = BIT_ULL(id);
		spin_lock_irqsave(&rep->lock, flags);
		rep->known |= bit;
		rep->shadow &= ~bit;
		val = readq(rep->base + rep->offsets->tgt_read);
		if (val & bit)
			writeq(val & ~bit, rep->base + rep->offsets->tgt_write);
		spin_unlock_irqrestore(&rep->lock, flags);
		if (apple_pmp_report_running(rep))
			ret = readq_poll_timeout_atomic(rep->base + rep->offsets->actual, val,
							!(val & bit), 10, 50000);
	}
	rcu_read_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_report_entry_withdraw_polled);

/*
 * System sleep. An external display processor's request keeps the PMP's
 * external display state up while nothing is shown, about 0.25 W in
 * suspend-to-idle on J514S. By the noirq phase every display processor has
 * suspended, its displays off and its links released or idle: withdraw such a
 * request if it is acknowledged, with one replacement write as macOS releases
 * its display requests, and assert it again before the processors resume.
 */
static int apple_pmp_report_entry_sleep(struct apple_pmp_report_entry *ent, bool release)
{
	struct apple_pmp_report *rep;
	int ret = -ENODEV;

	rcu_read_lock();
	list_for_each_entry_rcu(rep, &apple_pmp_reports, list) {
		if (rep->dev->of_node != ent->report_node)
			continue;
		if (release && (!(readq(rep->base + rep->offsets->status) & PMP_REPORT_READY) ||
				!(readq(rep->base + rep->offsets->actual) & BIT_ULL(ent->id))))
			ret = -EALREADY;
		else
			ret = apple_pmp_report_set_state(rep, ent->id, !release, ent->no_ack);
		break;
	}
	rcu_read_unlock();
	return ret;
}

static int apple_pmp_report_entry_suspend_noirq(struct device *dev)
{
	struct apple_pmp_report_entry *ent = dev_get_drvdata(dev);
	int ret;

	if (!ent || !ent->sleep_release || READ_ONCE(ent->removed))
		return 0;
	ret = apple_pmp_report_entry_sleep(ent, true);
	if (ret == -EALREADY)
		return 0;
	/* A withdrawal that was not acknowledged is asserted again all the same. */
	if (ret)
		dev_warn(dev, "request %u not withdrawn for sleep: %d\n", ent->id, ret);
	ent->released = true;
	return 0;
}

static int apple_pmp_report_entry_resume_noirq(struct device *dev)
{
	struct apple_pmp_report_entry *ent = dev_get_drvdata(dev);
	int ret, tries = 0;

	if (!ent || !ent->released)
		return 0;
	ent->released = false;
	/*
	 * Without the request the external display stays dark until reboot:
	 * write it again, three times at most, while the PMP does not
	 * acknowledge it in time.
	 */
	do {
		ret = apple_pmp_report_entry_sleep(ent, false);
	} while (ret == -ETIMEDOUT && ++tries < 3);
	if (ret)
		dev_err(dev, "request %u not asserted after sleep: %d\n", ent->id, ret);
	else if (tries)
		dev_warn(dev, "request %u asserted after sleep on write %d\n", ent->id,
			 tries + 1);
	return ret;
}

static const struct dev_pm_ops apple_pmp_report_entry_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(apple_pmp_report_entry_suspend_noirq,
				  apple_pmp_report_entry_resume_noirq)
};

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
	if (ent->native_owner) {
		unsigned long flags;

		spin_lock_irqsave(&ent->native_owner->lock, flags);
		ent->native_owner->native_claimed &= ~BIT_ULL(ent->id);
		spin_unlock_irqrestore(&ent->native_owner->lock, flags);
	}

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
	struct apple_pmp_report *rep;
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
	if (rep && rep->native_pmp) {
		unsigned long flags;
		bool record;

		ret = apple_pmp_report_record(rep->native_pmp, node, &ent->id, &record);
		if (ret || !record || (ent->id != 11 && ent->id != 12 && ent->id != 19)) {
			ret = -EINVAL;
			goto err_free;
		}
		ent->no_ack = of_property_read_bool(node, "apple,no-ack");
		if (ent->no_ack != (ent->id == 19)) {
			ret = -EINVAL;
			goto err_free;
		}
		spin_lock_irqsave(&rep->lock, flags);
		ret = rep->native_claimed & BIT_ULL(ent->id) ? -EBUSY : 0;
		if (!ret) {
			rep->native_claimed |= BIT_ULL(ent->id);
			ent->native_owner = rep;
		}
		spin_unlock_irqrestore(&rep->lock, flags);
		if (ret)
			goto err_free;
	} else if (start) {
		bool record;

		ret = apple_pmp_report_bit(start, node, &ent->id, &record);
		if (ret) {
			ret = dev_err_probe(dev, ret, "no request bit (reg, soc-device record)\n");
			goto err_free;
		}
		ent->no_ack = of_property_read_bool(node, "apple,no-ack");
	}
	ent->sleep_release = of_property_read_bool(node, "apple,release-for-sleep");
	if (of_property_present(node, "apple,dvfs")) {
		u32 class_levels[PMP_DVFS_CLASS_CELLS_MAX];
		u32 off_levels[PMP_DVFS_CLASS_CELLS_MAX];
		u32 request_class;
		u32 dvfs[2];
		int cells = 0, i;

		ret = of_property_read_u32_array(node, "apple,dvfs", dvfs, 2);
		if (ret || of_property_count_u32_elems(node, "apple,dvfs") != 2) {
			ret = dev_err_probe(dev, -EINVAL, "invalid apple,dvfs\n");
			goto err_free;
		}
		if (of_property_present(node, "apple,dvfs-request-levels")) {
			if (of_property_present(node, "apple,dvfs-request-class")) {
				ret = dev_err_probe(dev, -EINVAL,
						    "both DVFS request-class properties present\n");
				goto err_free;
			}
			cells = of_property_count_u32_elems(node,
						     "apple,dvfs-request-levels");
			if (cells <= 0 || cells > (int)ARRAY_SIZE(class_levels) ||
			    of_property_read_u32_array(node, "apple,dvfs-request-levels",
						       class_levels, cells)) {
				ret = dev_err_probe(dev, -EINVAL,
						    "invalid apple,dvfs-request-levels\n");
				goto err_free;
			}
		} else if (of_property_present(node, "apple,dvfs-request-class")) {
			if (of_property_read_u32(node, "apple,dvfs-request-class",
						 &request_class)) {
				ret = dev_err_probe(dev, -EINVAL,
						    "invalid apple,dvfs-request-class\n");
				goto err_free;
			}
			/* Legacy DTs put their only class level in apple,dvfs[1]. */
			class_levels[0] = request_class;
			class_levels[1] = dvfs[1];
			cells = 2;
			dvfs[1] = 0;
		}
		/* Releasing the request writes the same classes at level 0. */
		for (i = 0; i < cells; i += 2) {
			off_levels[i] = class_levels[i];
			off_levels[i + 1] = 0;
		}
		if (!rep || !rep->offsets->update ||
		    dvfs[0] < rep->offsets->dvfs_first ||
		    dvfs[0] >= rep->offsets->dvfs_first + rep->offsets->dvfs_count ||
		    !apple_pmp_dvfs_word(0, off_levels, cells, &ent->dvfs_off) ||
		    !apple_pmp_dvfs_word(dvfs[1], class_levels, cells, &ent->dvfs_on)) {
			ret = dev_err_probe(dev, -EINVAL, "invalid apple,dvfs\n");
			goto err_free;
		}
		ent->dvfs = true;
		ent->dvfs_entry = dvfs[0];
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
		ret = apple_pmp_report_entry_set_state(&ent->genpd, true);
		if (ret && ent->native_owner)
			goto err_free;
		/* Native DCPEXT keeps its resident producer and request through s2idle. */
		if (ent->native_owner && ent->id == 12) {
			ent->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
			ent->sleep_release = false;
		}
	}

	ent->genpd.power_on = apple_pmp_report_entry_power_on;
	ent->genpd.power_off = apple_pmp_report_entry_power_off;
	/*
	 * An entry that is the parent of PMGR power states (which are IRQ safe)
	 * must be IRQ safe too. Setting the state takes a spinlock and polls
	 * the acknowledgment atomically, so it may be called with IRQs off.
	 */
	if (of_property_read_bool(node, "apple,irq-safe"))
		ent->genpd.flags |= GENPD_FLAG_IRQ_SAFE;

	ret = pm_genpd_init(&ent->genpd, NULL,
			    !(ent->genpd.flags & GENPD_FLAG_ALWAYS_ON));
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
		.pm = pm_sleep_ptr(&apple_pmp_report_entry_pm_ops),
		/* Power-domain providers are not intended for manual unbinding. */
		.suppress_bind_attrs = true,
	},
};

MODULE_DEVICE_TABLE(of, apple_pmp_report_of_match);
MODULE_DEVICE_TABLE(of, apple_pmp_report_entry_of_match);

static int __init apple_pmp_report_init(void)
{
	register_pm_notifier(&apple_pmp_agx_pm_nb);
	apple_pmp_report_debugfs = debugfs_create_dir("apple-pmp-report", NULL);
	platform_driver_register(&apple_pmp_report_entry_driver);
	platform_driver_register(&apple_pmp_report_driver);
	return 0;
}

static void __exit apple_pmp_report_exit(void)
{
	unregister_pm_notifier(&apple_pmp_agx_pm_nb);
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
