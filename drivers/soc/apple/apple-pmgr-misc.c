// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMGR device power state driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/soc/apple/pmgr-misc.h>
#include <linux/workqueue.h>

#define APPLE_CLKGEN_PSTATE 0
#define APPLE_CLKGEN_PSTATE_DESIRED GENMASK(3, 0)
#define APPLE_CLKGEN_PSTATE_ACTUAL GENMASK(7, 4)

#define SYS_DEV_PSTATE_SUSPEND 1

#define APPLE_PMGR_IDLE_POLL_MS 250
#define APPLE_PMGR_RAISE_TIMEOUT_US 1000
#define APPLE_PMGR_PS_ACTUAL GENMASK(7, 4)
#define APPLE_PMGR_PS_ACTIVE 0xf

/*
 * Bandwidth users that keep the fabric up whenever their power domain is
 * active. External display and ANE system domains stay on even when idle,
 * so external displays vote through apple_pmgr_fabric_hold() instead, and
 * the ANE is not covered.
 */
static const char * const apple_pmgr_fabric_voters[] = {
	"gfx", "avd_sys", "isp_sys",
};

enum sys_device {
	DEV_FABRIC,
	DEV_DCS,
	DEV_MAX,
};

struct apple_pmgr_sys_device {
	void __iomem *base;
	u32 active_state;
	u32 suspend_state;
};

struct apple_pmgr_misc {
	struct device *dev;
	struct apple_pmgr_sys_device devices[DEV_MAX];

	/* Idle fabric governor; lock serializes fabric writes while awake. */
	struct mutex lock;
	struct delayed_work idle_work;		/* deferrable: fabric high */
	struct delayed_work idle_work_prompt;	/* bounded raise: fabric low */
	u32 fabric_now;				/* protected by lock */
	unsigned int holds;			/* protected by lock */
	bool sleeping;				/* protected by lock */
	bool voters_ok;
	bool pmp_owned; /* PMP is described: no governor, holds don't raise */
	bool pmp_sleep; /* PMP runs: this sleep leaves DCS to it (see prepare) */
	u32 pmp_fabric; /* PMP's desired fabric p-state, saved across suspend */
	struct {
		struct regmap *regmap;
		u32 reg;
	} votes[ARRAY_SIZE(apple_pmgr_fabric_voters)];
};

static struct apple_pmgr_misc *apple_pmgr_misc_inst;
static unsigned int idle_fabric_pstate;

static void apple_pmgr_sys_dev_set_pstate(struct apple_pmgr_misc *misc,
					  enum sys_device dev, bool active)
{
	u32 pstate;
	u32 val;

	if (!misc->devices[dev].base)
		return;

	if (active)
		pstate = misc->devices[dev].active_state;
	else
		pstate = misc->devices[dev].suspend_state;

	printk("set %d ps to pstate %d\n", dev, pstate);

	val = readl_relaxed(misc->devices[dev].base + APPLE_CLKGEN_PSTATE);
	val &= ~APPLE_CLKGEN_PSTATE_DESIRED;
	val |= FIELD_PREP(APPLE_CLKGEN_PSTATE_DESIRED, pstate);
	writel_relaxed(val, misc->devices[dev].base);
}

static void apple_pmgr_write_fabric(struct apple_pmgr_misc *misc, u32 pstate)
{
	void __iomem *reg = misc->devices[DEV_FABRIC].base + APPLE_CLKGEN_PSTATE;
	u32 val;

	lockdep_assert_held(&misc->lock);
	val = readl_relaxed(reg);
	val &= ~APPLE_CLKGEN_PSTATE_DESIRED;
	val |= FIELD_PREP(APPLE_CLKGEN_PSTATE_DESIRED, pstate);
	writel_relaxed(val, reg);
	misc->fabric_now = pstate;
}

/* Raise to the boot p-state and wait until the hardware reports it. */
static int apple_pmgr_raise_fabric(struct apple_pmgr_misc *misc)
{
	u32 active = misc->devices[DEV_FABRIC].active_state;
	u32 val;
	int ret;

	lockdep_assert_held(&misc->lock);
	/* PMP firmware owns the fabric; holds delegate to it. */
	if (misc->pmp_owned || misc->fabric_now == active)
		return 0;
	apple_pmgr_write_fabric(misc, active);
	ret = readl_poll_timeout(misc->devices[DEV_FABRIC].base + APPLE_CLKGEN_PSTATE,
				 val, FIELD_GET(APPLE_CLKGEN_PSTATE_ACTUAL, val) == active,
				 10, APPLE_PMGR_RAISE_TIMEOUT_US);
	/* Not confirmed: make the next caller try again. */
	if (ret)
		misc->fabric_now = U32_MAX;
	return ret;
}

/**
 * apple_pmgr_fabric_hold() - keep the fabric at its boot p-state
 *
 * For bandwidth users the idle governor cannot see, such as an external
 * display. Returns once the fabric is confirmed up, or an error if it did
 * not get there in time; the hold stands either way and must be released.
 * With PMP owning the fabric no vote is sent: the firmware decides, and
 * this returns success.
 */
int apple_pmgr_fabric_hold(void)
{
	/* Pairs with the release in probe: the instance is set up before it is seen. */
	struct apple_pmgr_misc *misc = smp_load_acquire(&apple_pmgr_misc_inst);

	if (!misc)
		return 0;
	guard(mutex)(&misc->lock);
	misc->holds++;
	return apple_pmgr_raise_fabric(misc);
}
EXPORT_SYMBOL_GPL(apple_pmgr_fabric_hold);

void apple_pmgr_fabric_release(void)
{
	/* Pairs with the release in probe: the instance is set up before it is seen. */
	struct apple_pmgr_misc *misc = smp_load_acquire(&apple_pmgr_misc_inst);

	if (!misc)
		return;
	guard(mutex)(&misc->lock);
	if (!WARN_ON_ONCE(!misc->holds))
		misc->holds--;
}
EXPORT_SYMBOL_GPL(apple_pmgr_fabric_release);

static bool apple_pmgr_voters_active(struct apple_pmgr_misc *misc)
{
	unsigned int i;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(misc->votes); i++) {
		/*
		 * Only the actual state shows activity: the GPU domain keeps its
		 * target on with auto power gating and the hardware gates it
		 * when idle. Unreadable counts as busy.
		 */
		if (regmap_read(misc->votes[i].regmap, misc->votes[i].reg, &val) ||
		    FIELD_GET(APPLE_PMGR_PS_ACTUAL, val) == APPLE_PMGR_PS_ACTIVE)
			return true;
	}
	return false;
}

/* Every expected voter must be found, or the governor never lowers. */
static void apple_pmgr_find_voters(struct apple_pmgr_misc *misc)
{
	unsigned int found = 0, i;
	struct device_node *np;
	const char *label;
	u32 reg;

	for_each_compatible_node(np, NULL, "apple,pmgr-pwrstate") {
		if (!of_device_is_available(np) ||
		    of_property_read_string(np, "label", &label) ||
		    of_property_read_u32(np, "reg", &reg))
			continue;
		for (i = 0; i < ARRAY_SIZE(apple_pmgr_fabric_voters); i++) {
			struct regmap *map;

			if (strcmp(label, apple_pmgr_fabric_voters[i]) ||
			    misc->votes[i].regmap)
				continue;
			map = syscon_node_to_regmap(np->parent);
			if (IS_ERR(map))
				break;
			misc->votes[i].regmap = map;
			misc->votes[i].reg = reg;
			found++;
			break;
		}
	}
	misc->voters_ok = found == ARRAY_SIZE(apple_pmgr_fabric_voters);
	/* Desktops have no ISP: say so only where the governor is wanted. */
	if (!misc->voters_ok && READ_ONCE(idle_fabric_pstate))
		dev_warn(misc->dev, "idle fabric: found %u of %zu bandwidth voters, staying high\n",
			 found, ARRAY_SIZE(apple_pmgr_fabric_voters));
	else if (!misc->voters_ok)
		dev_dbg(misc->dev, "idle fabric: found %u of %zu bandwidth voters\n",
			found, ARRAY_SIZE(apple_pmgr_fabric_voters));
}

/* True while every cpufreq policy runs at its lowest frequency. */
static bool apple_pmgr_cpus_at_min(void)
{
	struct cpufreq_policy *policy;
	unsigned int cpu;
	bool at_min = true;

	for_each_online_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			return false;
		if (READ_ONCE(policy->cur) > policy->cpuinfo.min_freq)
			at_min = false;
		cpufreq_cpu_put(policy);
		if (!at_min)
			break;
	}
	return at_min;
}

static void apple_pmgr_idle_eval(struct apple_pmgr_misc *misc)
{
	unsigned int idle = READ_ONCE(idle_fabric_pstate);
	u32 active = misc->devices[DEV_FABRIC].active_state;
	bool low = false;

	scoped_guard(mutex, &misc->lock) {
		if (misc->sleeping)
			return;
		low = idle && idle < active && misc->voters_ok && !misc->pmp_owned &&
		      !misc->holds && !apple_pmgr_voters_active(misc) &&
		      apple_pmgr_cpus_at_min();
		if (low && misc->fabric_now != idle)
			apple_pmgr_write_fabric(misc, idle);
		else if (!low)
			apple_pmgr_raise_fabric(misc);
		if (!idle)
			return;
	}

	/*
	 * Freezable, so it stops before devices suspend. While the fabric is
	 * high a deferrable timer never wakes an idle CPU; while it is low a
	 * normal timer limits how long new load runs at reduced bandwidth.
	 */
	queue_delayed_work(system_freezable_power_efficient_wq,
			   low ? &misc->idle_work_prompt : &misc->idle_work,
			   msecs_to_jiffies(APPLE_PMGR_IDLE_POLL_MS));
}

static void apple_pmgr_idle_work(struct work_struct *work)
{
	apple_pmgr_idle_eval(container_of(to_delayed_work(work),
					  struct apple_pmgr_misc, idle_work));
}

static void apple_pmgr_idle_work_prompt(struct work_struct *work)
{
	apple_pmgr_idle_eval(container_of(to_delayed_work(work),
					  struct apple_pmgr_misc, idle_work_prompt));
}

static void apple_pmgr_idle_stop(struct apple_pmgr_misc *misc)
{
	cancel_delayed_work_sync(&misc->idle_work);
	cancel_delayed_work_sync(&misc->idle_work_prompt);
	guard(mutex)(&misc->lock);
	apple_pmgr_raise_fabric(misc);
}

static int idle_fabric_pstate_set(const char *val, const struct kernel_param *kp)
{
	/* Pairs with the release in probe: the instance is set up before it is seen. */
	struct apple_pmgr_misc *misc = smp_load_acquire(&apple_pmgr_misc_inst);
	unsigned int ps;
	int ret;

	ret = kstrtouint(val, 0, &ps);
	if (ret)
		return ret;
	if (ps > FIELD_MAX(APPLE_CLKGEN_PSTATE_DESIRED))
		return -EINVAL;
	if (ps && misc && misc->pmp_owned)
		return -EBUSY;
	if (ps && misc && !misc->voters_ok)
		dev_warn(misc->dev, "idle fabric: not every bandwidth voter was found, staying high\n");
	WRITE_ONCE(idle_fabric_pstate, ps);
	if (!misc)
		return 0;
	if (!ps)
		apple_pmgr_idle_stop(misc);
	else
		mod_delayed_work(system_freezable_power_efficient_wq, &misc->idle_work, 0);
	return 0;
}

static const struct kernel_param_ops idle_fabric_pstate_ops = {
	.set = idle_fabric_pstate_set,
	.get = param_get_uint,
};

/*
 * Boot leaves the fabric at its high p-state and nothing lowers it while the
 * system is awake; on t6000 a fabric p-state of 1 saves about 0.5 W at idle.
 */
module_param_cb(idle_fabric_pstate, &idle_fabric_pstate_ops, &idle_fabric_pstate, 0644);
MODULE_PARM_DESC(idle_fabric_pstate,
		 "Fabric p-state while all CPUs run at their lowest frequency and no bandwidth user is active (0 = keep the boot state)");

/*
 * Whether PMP runs: its node is available and its driver is bound. With the
 * node described but no driver (not loaded, or no values from the boot
 * loader), nothing manages DCS, and sleep takes the path without PMP.
 */
static bool apple_pmgr_pmp_running(void)
{
	struct device_node *np __free(device_node) =
		of_find_compatible_node(NULL, NULL, "apple,t6000-pmp-v2");
	struct platform_device *pdev;
	bool bound;

	if (!np || !of_device_is_available(np))
		return false;
	pdev = of_find_device_by_node(np);
	if (!pdev)
		return false;
	bound = device_is_bound(&pdev->dev);
	put_device(&pdev->dev);
	return bound;
}

/* Raise before any device suspends; noirq then applies the suspend p-state. */
static int __maybe_unused apple_pmgr_misc_prepare(struct device *dev)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);

	misc->pmp_sleep = misc->pmp_owned && apple_pmgr_pmp_running();
	if (misc->pmp_owned && !misc->pmp_sleep)
		dev_warn_once(dev, "PMP is described but not running: sleeping without it\n");
	if (!misc->devices[DEV_FABRIC].base)
		return 0;
	scoped_guard(mutex, &misc->lock)
		misc->sleeping = true;
	apple_pmgr_idle_stop(misc);
	return 0;
}

static void __maybe_unused apple_pmgr_misc_complete(struct device *dev)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);

	if (!misc->devices[DEV_FABRIC].base)
		return;
	scoped_guard(mutex, &misc->lock)
		misc->sleeping = false;
	if (READ_ONCE(idle_fabric_pstate))
		mod_delayed_work(system_freezable_power_efficient_wq, &misc->idle_work, 0);
}

static int __maybe_unused apple_pmgr_misc_suspend_noirq(struct device *dev)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);
	int i;

	if (misc->pmp_sleep) {
		/*
		 * Lowering DCS under PMP leaves s2idle unable to resume; the
		 * fabric alone is safe and saves ~0.35 W in s2idle on t6000.
		 * Keep PMP's choice to hand back on resume.
		 */
		void __iomem *base = misc->devices[DEV_FABRIC].base;

		if (base) {
			misc->pmp_fabric = FIELD_GET(APPLE_CLKGEN_PSTATE_DESIRED,
						     readl_relaxed(base + APPLE_CLKGEN_PSTATE));
			apple_pmgr_sys_dev_set_pstate(misc, DEV_FABRIC, false);
		}
		return 0;
	}
	for (i = 0; i < DEV_MAX; i++)
		apple_pmgr_sys_dev_set_pstate(misc, i, false);

	return 0;
}

static int __maybe_unused apple_pmgr_misc_resume_noirq(struct device *dev)
{
	struct apple_pmgr_misc *misc = dev_get_drvdata(dev);
	int i;

	misc->fabric_now = misc->devices[DEV_FABRIC].active_state;
	if (misc->pmp_sleep) {
		if (misc->devices[DEV_FABRIC].base) {
			void __iomem *reg = misc->devices[DEV_FABRIC].base + APPLE_CLKGEN_PSTATE;
			u32 val = readl_relaxed(reg);

			/* PMP keeps running: never overwrite a choice it made since. */
			if (FIELD_GET(APPLE_CLKGEN_PSTATE_DESIRED, val) ==
			    misc->devices[DEV_FABRIC].suspend_state) {
				val &= ~APPLE_CLKGEN_PSTATE_DESIRED;
				val |= FIELD_PREP(APPLE_CLKGEN_PSTATE_DESIRED, misc->pmp_fabric);
				writel_relaxed(val, reg);
			}
		}
		return 0;
	}
	for (i = 0; i < DEV_MAX; i++)
		apple_pmgr_sys_dev_set_pstate(misc, i, true);

	return 0;
}

static bool apple_pmgr_init_device(struct apple_pmgr_misc *misc,
				   enum sys_device dev, const char *device_name)
{
	void __iomem *base;
	char name[32];
	u32 val;

	snprintf(name, sizeof(name), "%s-ps", device_name);

	base = devm_platform_ioremap_resource_byname(
		to_platform_device(misc->dev), name);
	if (IS_ERR(base))
		return false;

	val = readl_relaxed(base + APPLE_CLKGEN_PSTATE);

	misc->devices[dev].base = base;
	misc->devices[dev].active_state =
		FIELD_GET(APPLE_CLKGEN_PSTATE_DESIRED, val);
	misc->devices[dev].suspend_state = SYS_DEV_PSTATE_SUSPEND;

	snprintf(name, sizeof(name), "apple,%s-min-ps", device_name);
	of_property_read_u32(misc->dev->of_node, name,
			     &misc->devices[dev].suspend_state);

	return true;
}

static int apple_pmgr_misc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_pmgr_misc *misc;
	struct device_node *np;
	int ret = -ENODEV;

	misc = devm_kzalloc(dev, sizeof(*misc), GFP_KERNEL);
	if (!misc)
		return -ENOMEM;

	misc->dev = dev;

	if (apple_pmgr_init_device(misc, DEV_FABRIC, "fabric"))
		ret = 0;

	if (apple_pmgr_init_device(misc, DEV_DCS, "dcs"))
		ret = 0;

	platform_set_drvdata(pdev, misc);

	/*
	 * With PMP enabled its firmware owns the fabric and DCS p-states.
	 * Writing them for suspend as well leaves the system unable to resume
	 * from s2idle on t6000, and the idle governor would fight PMP.
	 */
	np = of_find_compatible_node(NULL, NULL, "apple,t6000-pmp-v2");
	misc->pmp_owned = np && of_device_is_available(np);
	of_node_put(np);
	if (misc->pmp_owned)
		dev_info(dev, "PMP manages fabric and DCS p-states\n");

	if (misc->devices[DEV_FABRIC].base) {
		mutex_init(&misc->lock);
		misc->fabric_now = misc->devices[DEV_FABRIC].active_state;
		INIT_DEFERRABLE_WORK(&misc->idle_work, apple_pmgr_idle_work);
		INIT_DELAYED_WORK(&misc->idle_work_prompt, apple_pmgr_idle_work_prompt);
		apple_pmgr_find_voters(misc);
		/* Publish only a set-up instance; pairs with the readers' acquire. */
		smp_store_release(&apple_pmgr_misc_inst, misc);
		if (READ_ONCE(idle_fabric_pstate) && misc->pmp_owned) {
			dev_warn(dev, "idle fabric governor disabled: PMP manages the fabric\n");
			WRITE_ONCE(idle_fabric_pstate, 0);
		} else if (READ_ONCE(idle_fabric_pstate)) {
			queue_delayed_work(system_freezable_power_efficient_wq,
					   &misc->idle_work, 0);
		}
	}

	return ret;
}

static const struct of_device_id apple_pmgr_misc_of_match[] = {
	{ .compatible = "apple,t6000-pmgr-misc" },
	{}
};

MODULE_DEVICE_TABLE(of, apple_pmgr_misc_of_match);

static const struct dev_pm_ops apple_pmgr_misc_pm_ops = {
	.prepare = apple_pmgr_misc_prepare,
	.complete = apple_pmgr_misc_complete,
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(apple_pmgr_misc_suspend_noirq,
				      apple_pmgr_misc_resume_noirq)
};

static struct platform_driver apple_pmgr_misc_driver = {
	.probe = apple_pmgr_misc_probe,
	.driver = {
		.name = "apple-pmgr-misc",
		.of_match_table = apple_pmgr_misc_of_match,
		/* The governor's workers and holders would outlive an unbind. */
		.suppress_bind_attrs = true,
		.pm = pm_ptr(&apple_pmgr_misc_pm_ops),
	},
};

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("PMGR misc driver for Apple SoCs");
MODULE_LICENSE("GPL v2");

module_platform_driver(apple_pmgr_misc_driver);
