// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMGR device power state driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/reset-controller.h>
#include <linux/module.h>
#include <linux/soc/apple/pmgr-pwrstate.h>
#include <linux/soc/apple/sleep.h>
#include <linux/soc/apple/t8132.h>

#define APPLE_PMGR_RESET        BIT(31)
#define APPLE_PMGR_AUTO_ENABLE  BIT(28)
#define APPLE_PMGR_PS_AUTO      GENMASK(27, 24)
#define APPLE_PMGR_PS_MIN       GENMASK(19, 16)
#define APPLE_PMGR_PS_RESET     BIT(12)
#define APPLE_PMGR_BUSY         BIT(11)
#define APPLE_PMGR_DEV_DISABLE  BIT(10)
#define APPLE_PMGR_WAS_CLKGATED BIT(9)
#define APPLE_PMGR_WAS_PWRGATED BIT(8)
#define APPLE_PMGR_PS_ACTUAL    GENMASK(7, 4)
#define APPLE_PMGR_PS_TARGET    GENMASK(3, 0)

#define APPLE_PMGR_FLAGS        (APPLE_PMGR_WAS_CLKGATED | APPLE_PMGR_WAS_PWRGATED)

#define APPLE_PMGR_PS_ACTIVE    0xf
#define APPLE_PMGR_PS_CLKGATE   0x4
#define APPLE_PMGR_PS_PWRGATE   0x0

#define APPLE_PMGR_PS_SET_TIMEOUT 100
/* How long a lowered domain may report BUSY while its switch finishes */
#define APPLE_PMGR_BUSY_SETTLE_US 1000
#define APPLE_PMGR_PS_CONSUMER_TIMEOUT 1000
#define APPLE_PMGR_PS_MAX_RETENTION 4
#define APPLE_PMGR_PS_MAX_CHAIN 4

#define APPLE_PMGR_PS_LEVELS	(APPLE_PMGR_PS_TARGET | APPLE_PMGR_PS_ACTUAL)
/* Target and actual state both on (or both power gated). */
#define APPLE_PMGR_PS_ALL_ON	(FIELD_PREP(APPLE_PMGR_PS_TARGET, APPLE_PMGR_PS_ACTIVE) | \
				 FIELD_PREP(APPLE_PMGR_PS_ACTUAL, APPLE_PMGR_PS_ACTIVE))
#define APPLE_PMGR_PS_PARENT_TIMEOUT 10000
#define APPLE_PMGR_T8122_PMS_C1PPT 0x160
#define APPLE_PMGR_RESET_TIME 1
#define APPLE_PMGR_T8132_RESET_TIME 10

struct apple_pmgr_ps {
	struct device *dev;
	struct generic_pm_domain genpd;
	struct reset_controller_dev rcdev;
	struct regmap *regmap;
	u32 offset;
	u32 min_state;
	bool force_disable;
	bool force_reset;
	bool externally_clocked;
	bool system_sleep;
	/* Lowered as macOS lowers it, and unlatched even in a system sleep. */
	bool macos_lower;
	bool firmware_owned;
	/* apple,sleep-retained: kept on while the system sleeps */
	bool sleep_retained;
	/* apple,sleep-driver-owned: the driver behind it brings it back after a sleep */
	bool sleep_driver_owned;
	bool sleep_wakeup_path;
	/* Never enable automatic power management (apple,no-auto-pm). */
	bool no_auto_pm;
	bool inherited_on;
	bool no_auto_enable;
	bool needs_reset_idle_wait;
	bool strict_state;
	bool wait_parent;
	u32 reset_time_us;
	/* Off in genpd but on in hardware as the system slept: someone else raised it. */
	bool sleep_raised_by_other;
	/* That domain's target as the system slept. */
	u32 sleep_target;
	/*
	 * apple,consumer-switched: firmware-owned, but powered off and on by its
	 * consumer (apple_pmgr_ps_consumer_off()). The state word before
	 * power-off, and the retention workaround registers with their values
	 * before power-off. A failed transition leaves it failed until reboot.
	 */
	bool consumer_switched;
	bool consumer_off;
	bool consumer_failed;
	u32 consumer_saved;
	unsigned int nr_retention;
	struct {
		u32 offset, mask, value, saved;
	} retention[APPLE_PMGR_PS_MAX_RETENTION];
	/* Two bridge link words of the block behind this state, usable while it is on. */
	void __iomem *bridge;
};

#define genpd_to_apple_pmgr_ps(_genpd) container_of(_genpd, struct apple_pmgr_ps, genpd)
#define rcdev_to_apple_pmgr_ps(_rcdev) container_of(_rcdev, struct apple_pmgr_ps, rcdev)

static int apple_pmgr_ps_wait_parent(struct apple_pmgr_ps *ps, u32 *cur)
{
	int ret, err;

	/* Bit 11 reports a missing upstream power/clock on T6000 GFX. */
	ret = read_poll_timeout_atomic(regmap_read, err,
				       err || !(*cur & APPLE_PMGR_BUSY),
				       1, APPLE_PMGR_PS_PARENT_TIMEOUT, false,
				       ps->regmap, ps->offset, cur);
	return err ?: ret;
}

/*
 * A power-on can stop with the actual state short of the target and bit 11
 * latched: on the T8140 ANE_SYS after system sleep the word reads 0x2b4f
 * (target 0xf, actual 0x4) and stays so. That is not a parent that comes up
 * late: gated again, the domain reaches actual 0 at once, but bit 11 stays
 * set and every later power-on stops the same way. Only a PS reset clears it.
 * Power the domain off, pulse the PS reset while it is off (its devices lose
 * their state there anyway), then request @reg, the on state, again.
 * Domains whose bit 11 means a parent that is not up yet (wait_parent) and
 * those with their own state contract are left alone.
 *
 * Never during a system sleep transition: there the power-on comes from the
 * genpd noirq resume, with the DARTs and the engine of the domain still in
 * their noirq phase. On the T8140 a recovery at that point hangs the machine:
 * in a suspend loop on the J700, kernels with it hung on 3 of 3 boots and came
 * back only through the watchdog, while kernels without it hung on none of 6,
 * two of which latched ANE_SYS and carried on with the ANE quarantined. The
 * power-on then fails as it did before, and genpd, which ignores errors in
 * that phase, keeps the domain requested on.
 */
/* Pulse the PS reset of a domain that is off with BUSY latched; it stays off. */
static void apple_pmgr_ps_unlatch_off(struct apple_pmgr_ps *ps, u32 *cur)
{
	u32 off = *cur & ~(APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_FLAGS | APPLE_PMGR_BUSY |
			   APPLE_PMGR_PS_RESET | APPLE_PMGR_DEV_DISABLE | APPLE_PMGR_PS_TARGET);

	regmap_write(ps->regmap, ps->offset, off | APPLE_PMGR_PS_RESET);
	regmap_read_poll_timeout_atomic(ps->regmap, ps->offset, *cur,
					!(*cur & APPLE_PMGR_BUSY), 1, APPLE_PMGR_PS_SET_TIMEOUT);
	regmap_write(ps->regmap, ps->offset, off);
	regmap_read(ps->regmap, ps->offset, cur);
	if (*cur & APPLE_PMGR_BUSY)
		dev_warn_ratelimited(ps->dev, "PS %s: off with BUSY latched: %#x\n",
				     ps->genpd.name, *cur);
}

static int apple_pmgr_ps_clear_latch(struct apple_pmgr_ps *ps, u32 reg, u32 *cur)
{
	u32 off = (reg & ~APPLE_PMGR_PS_TARGET) | APPLE_PMGR_PS_PWRGATE;
	int ret;

	if (ps->system_sleep || ps->wait_parent || ps->strict_state ||
	    ps->externally_clocked || !(*cur & APPLE_PMGR_BUSY) ||
	    FIELD_GET(APPLE_PMGR_PS_ACTUAL, *cur) == APPLE_PMGR_PS_ACTIVE)
		return -ETIMEDOUT;

	regmap_write(ps->regmap, ps->offset, off);
	ret = regmap_read_poll_timeout_atomic(ps->regmap, ps->offset, *cur,
					      !FIELD_GET(APPLE_PMGR_PS_ACTUAL, *cur), 1,
					      APPLE_PMGR_PS_SET_TIMEOUT);
	if (ret)
		return ret;
	regmap_write(ps->regmap, ps->offset, off | APPLE_PMGR_PS_RESET);
	ret = regmap_read_poll_timeout_atomic(ps->regmap, ps->offset, *cur,
					      !(*cur & APPLE_PMGR_BUSY), 1,
					      APPLE_PMGR_PS_SET_TIMEOUT);
	regmap_write(ps->regmap, ps->offset, off);
	if (ret)
		return ret;
	regmap_write(ps->regmap, ps->offset, reg);
	return regmap_read_poll_timeout_atomic(ps->regmap, ps->offset, *cur,
					       FIELD_GET(APPLE_PMGR_PS_ACTUAL, *cur) ==
					       APPLE_PMGR_PS_ACTIVE &&
					       !(*cur & APPLE_PMGR_BUSY), 1,
					       APPLE_PMGR_PS_SET_TIMEOUT);
}

static int apple_pmgr_ps_set(struct generic_pm_domain *genpd, u32 pstate, bool auto_enable)
{
	int ret;
	struct apple_pmgr_ps *ps = genpd_to_apple_pmgr_ps(genpd);
	u32 reg, cur;
	u32 timeout = ps->strict_state ? 10000 : APPLE_PMGR_PS_SET_TIMEOUT;

	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret < 0)
		return ret;

	/* Resets are synchronous, and only work if the device is powered and clocked. */
	if (reg & APPLE_PMGR_RESET && pstate != APPLE_PMGR_PS_ACTIVE)
		dev_err_ratelimited(ps->dev, "PS %s: powering off with RESET active\n",
				    genpd->name);

	if (pstate != APPLE_PMGR_PS_ACTIVE && (ps->force_disable || ps->force_reset)) {
		u32 reg_pre = reg & ~(APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_FLAGS |
				      APPLE_PMGR_BUSY);

		if (ps->force_disable)
			reg_pre |= APPLE_PMGR_DEV_DISABLE;
		if (ps->force_reset)
			reg_pre |= APPLE_PMGR_PS_RESET;

		regmap_write(ps->regmap, ps->offset, reg_pre);

		ret = regmap_read_poll_timeout_atomic(
			ps->regmap, ps->offset, cur,
			(cur & (APPLE_PMGR_DEV_DISABLE | APPLE_PMGR_PS_RESET)) ==
			(reg_pre & (APPLE_PMGR_DEV_DISABLE | APPLE_PMGR_PS_RESET)), 1,
			APPLE_PMGR_PS_SET_TIMEOUT);

		if (ret < 0)
			dev_err_ratelimited(ps->dev,
					    "PS %s: Failed to set reset/disable bits (now: 0x%x)\n",
					    genpd->name, cur);
	}

	/*
	 * As macOS lowers a domain: automatic power management off first, back at full
	 * power, and only then the target. Taking both at once from an auto-gated state
	 * can leave BUSY latched, and a latch stalls the sleep engine's wake.
	 */
	if (ps->macos_lower && pstate != APPLE_PMGR_PS_ACTIVE && (reg & APPLE_PMGR_AUTO_ENABLE)) {
		regmap_write(ps->regmap, ps->offset,
			     reg & ~(APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_FLAGS | APPLE_PMGR_BUSY));
		regmap_read_poll_timeout_atomic(ps->regmap, ps->offset, cur,
						FIELD_GET(APPLE_PMGR_PS_ACTUAL, cur) ==
						APPLE_PMGR_PS_ACTIVE, 1, timeout);
	}

	/* Do not replay the hardware-owned BUSY bit in this whole-word write. */
	reg &= ~(APPLE_PMGR_DEV_DISABLE | APPLE_PMGR_PS_RESET |
		 APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_FLAGS | APPLE_PMGR_BUSY |
		 APPLE_PMGR_PS_TARGET);
	reg |= FIELD_PREP(APPLE_PMGR_PS_TARGET, pstate);
	/* Writing the history bits clears them. */
	if (ps->macos_lower && pstate != APPLE_PMGR_PS_ACTIVE)
		reg |= APPLE_PMGR_FLAGS;

	dev_dbg(ps->dev, "PS %s: pwrstate = 0x%x: 0x%x\n", genpd->name, pstate, reg);

	ret = regmap_write(ps->regmap, ps->offset, reg);
	if (ret && ps->strict_state)
		return ret;
	cur = reg;

	if (ps->wait_parent && pstate == APPLE_PMGR_PS_ACTIVE) {
		/*
		 * GFX can remain clock-gated with bit 11 set on noirq resume,
		 * even after genpd has powered its AFR parent on. Wait for that
		 * hardware dependency separately, then allow the usual 100 us
		 * for the actual state. A missing parent must still fail.
		 * Use delay-based polling so the wait is bounded even while
		 * timekeeping is suspended.
		 */
		ret = apple_pmgr_ps_wait_parent(ps, &cur);
		if (ret)
			goto report;
	}

	if (ps->externally_clocked && pstate == APPLE_PMGR_PS_ACTIVE) {
		/*
		 * If this clock domain requires an external clock, then
		 * consider the "clock gated" state to be good enough.
		 */
		ret = regmap_read_poll_timeout_atomic(
			ps->regmap, ps->offset, cur,
			FIELD_GET(APPLE_PMGR_PS_ACTUAL, cur) >= APPLE_PMGR_PS_CLKGATE, 1,
			timeout);
	} else {
		ret = regmap_read_poll_timeout_atomic(
			ps->regmap, ps->offset, cur,
			FIELD_GET(APPLE_PMGR_PS_ACTUAL, cur) == pstate &&
			(!ps->wait_parent || pstate != APPLE_PMGR_PS_ACTIVE ||
			 !(cur & APPLE_PMGR_BUSY)), 1,
			timeout);
	}
	if (ps->strict_state) {
		dev_info(ps->dev, "GFX target=%#x written=%#x actual=%#x error=%d\n",
			 pstate, reg, cur, ret);
		if (ret)
			return ret;
	}

	if (ret < 0 && pstate == APPLE_PMGR_PS_ACTIVE) {
		u32 stalled = cur;

		if (!apple_pmgr_ps_clear_latch(ps, reg, &cur)) {
			dev_info_ratelimited(ps->dev,
					     "PS %s: power-on stopped at 0x%x with bit 11 latched; cleared by a PS reset\n",
					     genpd->name, stalled);
			ret = 0;
		}
	}

	/*
	 * A domain can reach off with BUSY latched: on T8140 both ISP CPU cores do,
	 * every time they go down under Linux. macOS leaves them off with it clear,
	 * and the sleep engine, which powers them up itself in a wake that keeps DRAM
	 * up, stalls on a latched one until its watchdog resets the SoC. Only a PS
	 * reset clears it, and the domain is off, so pulse one as on the power-on
	 * path.
	 *
	 * T8140 only. Other SoCs do not need it: on T8112 the gfx domain reaches
	 * off with BUSY set before every system sleep and is fine left that way,
	 * while a PS reset of a domain whose block is still busy can reset the
	 * whole SoC (a hung ANE does on T8112).
	 */
	/*
	 * Off only once BUSY clears too: macOS waits for (word & 0x8ff) == 0. The
	 * actual state reads 0 a moment before the switch is done. BUSY still set
	 * after this is a latch.
	 */
	if (!ret && ps->macos_lower && pstate != APPLE_PMGR_PS_ACTIVE && (cur & APPLE_PMGR_BUSY))
		regmap_read_poll_timeout_atomic(ps->regmap, ps->offset, cur,
						!(cur & APPLE_PMGR_BUSY), 1,
						APPLE_PMGR_BUSY_SETTLE_US);

	if (!ret && pstate != APPLE_PMGR_PS_ACTIVE && (cur & APPLE_PMGR_BUSY) &&
	    !FIELD_GET(APPLE_PMGR_PS_ACTUAL, cur) && !ps->wait_parent &&
	    !ps->strict_state && !ps->externally_clocked) {
		if (ps->macos_lower) {
			/* macOS never needs this: say so when it does. */
			dev_info_ratelimited(ps->dev,
					     "PS %s: off with BUSY%s: %#x; cleared by a PS reset\n",
					     genpd->name, ps->system_sleep ? " in a system sleep" : "",
					     cur);
			apple_pmgr_ps_unlatch_off(ps, &cur);
		} else {
			dev_dbg(ps->dev, "PS %s: off with BUSY: %#x; left as it is\n",
				genpd->name, cur);
		}
	}

report:
	if (ret < 0)
		dev_err_ratelimited(ps->dev,
				    "PS %s: Failed to reach power state 0x%x (requested 0x%x, now: 0x%x)\n",
				    genpd->name, pstate, reg, cur);

	if (auto_enable) {
		/* Not all devices implement this; this is a no-op where not implemented. */
		reg |= APPLE_PMGR_AUTO_ENABLE;
		if (ps->strict_state)
			return regmap_write(ps->regmap, ps->offset, reg);
		regmap_write(ps->regmap, ps->offset, reg);
	}

	return ret;
}

static int apple_pmgr_ps_is_active(struct apple_pmgr_ps *ps)
{
	u32 reg = 0;
	int ret;

	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret && ps->needs_reset_idle_wait)
		return ret;
	/* The J713 GFX admission needs an actual-state ACK, not target/auto. */
	if (ps->strict_state)
		return FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == APPLE_PMGR_PS_ACTIVE;
	/*
	 * We consider domains as active if they are actually on, or if they have auto-PM
	 * enabled and the intended target is on.
	 */
	return (FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) == APPLE_PMGR_PS_ACTIVE ||
		(FIELD_GET(APPLE_PMGR_PS_TARGET, reg) == APPLE_PMGR_PS_ACTIVE &&
		 reg & APPLE_PMGR_AUTO_ENABLE));
}

/* Verify a retained domain without requesting any hardware transition. */
static int apple_pmgr_ps_check_inherited(struct apple_pmgr_ps *ps)
{
	u32 reg;
	int ret;

	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret)
		return ret;
	if (FIELD_GET(APPLE_PMGR_PS_TARGET, reg) != APPLE_PMGR_PS_ACTIVE ||
	    FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) != APPLE_PMGR_PS_ACTIVE ||
	    (reg & (APPLE_PMGR_RESET | APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_PS_RESET |
		    APPLE_PMGR_BUSY | APPLE_PMGR_DEV_DISABLE)))
		return -EBUSY;

	return 0;
}

/*
 * genpd keeps a domain off when ->power_on() fails and on when ->power_off()
 * fails. Follow a failed transition with the opposite one, so the domain is
 * not left requesting a state genpd does not know it is in.
 *
 * The exception is a failed power-on in the noirq and syscore phases of a
 * system sleep transition: genpd ignores the error there and records the
 * domain as on. The PM core runs those phases after the suspend_late callback
 * of every device and before any resume_early callback, so keep requesting
 * the domain on between this device's two callbacks.
 */
static int apple_pmgr_ps_power_on(struct generic_pm_domain *genpd)
{
	struct apple_pmgr_ps *ps = genpd_to_apple_pmgr_ps(genpd);
	int ret;

	/* Its coprocessor's firmware owns the state; Linux only holds a vote. */
	if (ps->firmware_owned)
		return 0;
	if (ps->inherited_on)
		return apple_pmgr_ps_check_inherited(ps);

	ret = apple_pmgr_ps_set(genpd, APPLE_PMGR_PS_ACTIVE,
				!ps->no_auto_enable && !ps->no_auto_pm);
	if (ret && !ps->system_sleep)
		apple_pmgr_ps_set(genpd, APPLE_PMGR_PS_PWRGATE, false);

	return ret;
}

static int apple_pmgr_ps_power_off(struct generic_pm_domain *genpd)
{
	struct apple_pmgr_ps *ps = genpd_to_apple_pmgr_ps(genpd);
	int ret;

	if (ps->firmware_owned)
		return 0;
	if (ps->inherited_on)
		return -EBUSY;
	/* A live coprocessor keeps this domain through system sleep. */
	if (ps->sleep_retained && ps->system_sleep)
		return -EBUSY;
	/*
	 * Its driver took it down for the sleep and raises it in its own order. Gating it
	 * here would take its parents too, and genpd would raise them all first at the wake.
	 */
	if (ps->system_sleep && apple_sleep_domain_is_down(ps->dev->of_node))
		return -EBUSY;

	ret = apple_pmgr_ps_set(genpd, APPLE_PMGR_PS_PWRGATE, false);
	if (ret)
		apple_pmgr_ps_set(genpd, APPLE_PMGR_PS_ACTIVE,
				 !ps->no_auto_enable && !ps->no_auto_pm);

	return ret;
}

/* genpd's runtime suspend callback, as it installs it in every genpd's dev_pm_domain. */
static int (*apple_pmgr_genpd_runtime_suspend)(struct device *dev);

/* Check the physical result of genpd's void syscore suspend/resume helpers. */
int apple_pmgr_ps_verify_state(struct device *domain_dev, bool on)
{
	int (*genpd_suspend)(struct device *dev) = READ_ONCE(apple_pmgr_genpd_runtime_suspend);
	struct generic_pm_domain *genpd;
	struct apple_pmgr_ps *ps;
	u32 value, mask = APPLE_PMGR_PS_LEVELS | APPLE_PMGR_BUSY |
			  APPLE_PMGR_RESET | APPLE_PMGR_PS_RESET | APPLE_PMGR_DEV_DISABLE;

	/* Only a genpd's dev_pm_domain is embedded in one: anything else is refused. */
	if (!domain_dev->pm_domain || !genpd_suspend ||
	    domain_dev->pm_domain->ops.runtime_suspend != genpd_suspend)
		return -EINVAL;
	genpd = pd_to_genpd(domain_dev->pm_domain);
	if (genpd->power_on != apple_pmgr_ps_power_on ||
	    !(genpd->flags & GENPD_FLAG_IRQ_SAFE))
		return -EOPNOTSUPP;
	ps = genpd_to_apple_pmgr_ps(genpd);
	if (ps->firmware_owned || ps->inherited_on)
		return -EPERM;
	if (!on)
		mask |= APPLE_PMGR_AUTO_ENABLE;
	return regmap_read_poll_timeout_atomic(ps->regmap, ps->offset, value,
					      (value & mask) == (on ? APPLE_PMGR_PS_ALL_ON : 0),
					      1, APPLE_PMGR_PS_PARENT_TIMEOUT);
}
EXPORT_SYMBOL_GPL(apple_pmgr_ps_verify_state);

/**
 * apple_pmgr_ps_cycle() - power gate a domain and turn it on again
 * @np: the power state node, as named by a consumer's power-domains
 *
 * For a coprocessor that can stop in a state only a power-off of this domain
 * clears, while other devices in the domain (its DARTs) hold it on at
 * runtime. The caller owns every device in the domain and keeps them idle;
 * every subdomain must already be off. genpd's view does not change: the
 * domain is on before and after, as is its automatic power management.
 *
 * Return: 0, -EBUSY if a subdomain is on, -EPERM for a domain Linux does not
 * own, -EINVAL if the domain is off, or the error of a failed transition.
 */
int apple_pmgr_ps_cycle(struct device_node *np)
{
	struct platform_device *pdev = of_find_device_by_node(np);
	struct apple_pmgr_ps *ps;
	struct gpd_link *link;
	unsigned long flags;
	int ret, on;

	if (!pdev)
		return -ENODEV;
	ps = platform_get_drvdata(pdev);
	if (!ps) {
		ret = -EPROBE_DEFER;
		goto out;
	}
	if (ps->firmware_owned || ps->inherited_on) {
		ret = -EPERM;
		goto out;
	}

	spin_lock_irqsave(&ps->genpd.slock, flags);
	if (ps->genpd.status != GENPD_STATE_ON) {
		ret = -EINVAL;
		goto unlock;
	}
	list_for_each_entry(link, &ps->genpd.parent_links, parent_node) {
		if (link->child->status == GENPD_STATE_ON) {
			ret = -EBUSY;
			goto unlock;
		}
	}
	ret = apple_pmgr_ps_set(&ps->genpd, APPLE_PMGR_PS_PWRGATE, false);
	on = apple_pmgr_ps_set(&ps->genpd, APPLE_PMGR_PS_ACTIVE,
			       !ps->no_auto_enable && !ps->no_auto_pm);
	ret = ret ?: on;
unlock:
	spin_unlock_irqrestore(&ps->genpd.slock, flags);
out:
	put_device(&pdev->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmgr_ps_cycle);

static void apple_pmgr_ps_set_system_sleep(struct device *dev, bool sleep)
{
	struct apple_pmgr_ps *ps = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&ps->genpd.slock, flags);
	ps->system_sleep = sleep;
	spin_unlock_irqrestore(&ps->genpd.slock, flags);
}

static int apple_pmgr_ps_suspend_late(struct device *dev)
{
	apple_pmgr_ps_set_system_sleep(dev, true);

	return 0;
}

static int apple_pmgr_ps_resume_early(struct device *dev)
{
	apple_pmgr_ps_set_system_sleep(dev, false);

	return 0;
}

/* T8140's always-on PMGR, the block the AOP shares */
#define APPLE_PMGR_T8140_AON_BASE 0x308280000ULL

static bool apple_pmgr_ps_in_aon(struct device_node *node)
{
	struct resource res;

	return node->parent && !of_address_to_resource(node->parent, 0, &res) &&
	       res.start == APPLE_PMGR_T8140_AON_BASE;
}

/*
 * By the noirq phase genpd has powered off what it is going to. A domain it has off
 * whose target is still on was raised by a coprocessor's firmware: on T8140 the AOP
 * raises audio domains of its own in the always-on PMGR, and the sleep keeps them.
 */
static int apple_pmgr_ps_suspend_noirq(struct device *dev)
{
	struct apple_pmgr_ps *ps = dev_get_drvdata(dev);
	unsigned long flags;
	u32 reg;

	ps->sleep_raised_by_other = false;
	if (ps->firmware_owned || ps->inherited_on)
		return 0;

	spin_lock_irqsave(&ps->genpd.slock, flags);
	if (!regmap_read(ps->regmap, ps->offset, &reg)) {
		/*
		 * Off for genpd but still powered, as the loader left it (T8140
		 * isp_dprx): finish the power-off now. Otherwise the sleep lowers it
		 * at the commit, raises it again at the wake, and the resync lowers
		 * it a second time with BUSY left. macOS has it off at the commit and
		 * doesn't touch it after the wake. Not in the always-on PMGR, whose
		 * words the AOP raises for itself.
		 */
		if (ps->macos_lower && ps->genpd.status == GENPD_STATE_OFF &&
		    FIELD_GET(APPLE_PMGR_PS_TARGET, reg) && FIELD_GET(APPLE_PMGR_PS_ACTUAL, reg) &&
		    !apple_pmgr_ps_in_aon(dev->of_node)) {
			apple_pmgr_ps_set(&ps->genpd, APPLE_PMGR_PS_PWRGATE, false);
			regmap_read(ps->regmap, ps->offset, &reg);
		}
		ps->sleep_target = FIELD_GET(APPLE_PMGR_PS_TARGET, reg);
		ps->sleep_raised_by_other = ps->genpd.status == GENPD_STATE_OFF &&
					    ps->sleep_target != APPLE_PMGR_PS_PWRGATE;
	}
	spin_unlock_irqrestore(&ps->genpd.slock, flags);

	return 0;
}

/*
 * A firmware-driven suspend powers the SoC down and its sleep engine leaves the
 * domains in states of its own: AUTO_ENABLE cleared, some at clock gating. genpd
 * only powers a domain back on if it turned it off itself, so its idea of a
 * domain it left on is now wrong. Put each domain back where genpd thinks it is.
 *
 * On T8140 macOS re-raises 22 domains in the first 5 ms after a wake and lowers
 * 30 others (measured under aura). Which of them Linux needs isn't established,
 * so this follows genpd's books rather than a list. A domain whose driver took it
 * down for the sleep, as the USB host's PHY does, is that driver's to raise, in
 * its own order. A domain someone else raised before the sleep, and still at that
 * target, stays theirs.
 */
static int apple_pmgr_ps_resume_noirq(struct device *dev)
{
	struct apple_pmgr_ps *ps = dev_get_drvdata(dev);
	unsigned long flags;
	u32 reg;
	int ret;

	if (!pm_resume_via_firmware() || ps->firmware_owned || ps->inherited_on ||
	    ps->sleep_driver_owned || apple_sleep_domain_is_down(dev->of_node))
		return 0;

	spin_lock_irqsave(&ps->genpd.slock, flags);
	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (!ret) {
		if (ps->genpd.status == GENPD_STATE_OFF) {
			u32 target = FIELD_GET(APPLE_PMGR_PS_TARGET, reg);

			/*
			 * Lowering an AOP-raised audio domain here leaves the internal
			 * microphone array's data path dark: the AOP never raises it again.
			 */
			if (target != APPLE_PMGR_PS_PWRGATE &&
			    !(ps->sleep_raised_by_other && target == ps->sleep_target))
				ret = apple_pmgr_ps_set(&ps->genpd, APPLE_PMGR_PS_PWRGATE,
							false);
		} else {
			bool auto_enable = !ps->no_auto_enable && !ps->no_auto_pm;

			if (FIELD_GET(APPLE_PMGR_PS_TARGET, reg) != APPLE_PMGR_PS_ACTIVE ||
			    (auto_enable && !(reg & APPLE_PMGR_AUTO_ENABLE)))
				ret = apple_pmgr_ps_set(&ps->genpd, APPLE_PMGR_PS_ACTIVE,
							auto_enable);
		}
	}
	spin_unlock_irqrestore(&ps->genpd.slock, flags);
	if (ret)
		dev_err(dev, "PS %s: resync after sleep failed: %d\n", ps->genpd.name, ret);

	return 0;
}

static int apple_pmgr_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	unsigned long flags;
	int ret;
	u32 reg;

	if (ps->firmware_owned || ps->inherited_on)
		return -EPERM;

	spin_lock_irqsave(&ps->genpd.slock, flags);

	if (ps->genpd.status == GENPD_STATE_OFF)
		dev_err_ratelimited(ps->dev,
				    "PS 0x%x: asserting RESET while powered down\n",
				    ps->offset);

	dev_dbg(ps->dev, "PS 0x%x: assert reset\n", ps->offset);
	/* Quiesce device before asserting reset */
	regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_DEV_DISABLE,
			   APPLE_PMGR_DEV_DISABLE);
	if (ps->needs_reset_idle_wait) {
		ret = regmap_read_poll_timeout_atomic(ps->regmap, ps->offset,
						      reg, !(reg & APPLE_PMGR_BUSY),
						      1, APPLE_PMGR_PS_SET_TIMEOUT);
		if (ret) {
			dev_err(ps->dev, "reset quiesce timed out offset=%#x reg=%#x\n",
				ps->offset, reg);
			spin_unlock_irqrestore(&ps->genpd.slock, flags);
			return ret;
		}
	}
	regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_RESET,
			   APPLE_PMGR_RESET);

	spin_unlock_irqrestore(&ps->genpd.slock, flags);

	return 0;
}

static int apple_pmgr_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	unsigned long flags;

	if (ps->firmware_owned || ps->inherited_on)
		return -EPERM;

	if (ps->reset_time_us == APPLE_PMGR_T8132_RESET_TIME)
		usleep_range(ps->reset_time_us, 2 * ps->reset_time_us);

	spin_lock_irqsave(&ps->genpd.slock, flags);

	dev_dbg(ps->dev, "PS 0x%x: deassert reset\n", ps->offset);
	regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_RESET, 0);
	regmap_update_bits(ps->regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_DEV_DISABLE, 0);

	if (ps->genpd.status == GENPD_STATE_OFF)
		dev_err_ratelimited(ps->dev,
				    "PS 0x%x: RESET was deasserted while powered down\n",
				    ps->offset);

	spin_unlock_irqrestore(&ps->genpd.slock, flags);

	return 0;
}

static int apple_pmgr_reset_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	int ret;

	ret = apple_pmgr_reset_assert(rcdev, id);
	if (ret)
		return ret;

	if (ps->reset_time_us != APPLE_PMGR_T8132_RESET_TIME)
		usleep_range(APPLE_PMGR_RESET_TIME, 2 * APPLE_PMGR_RESET_TIME);

	return apple_pmgr_reset_deassert(rcdev, id);
}

static int apple_pmgr_reset_status(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct apple_pmgr_ps *ps = rcdev_to_apple_pmgr_ps(rcdev);
	u32 reg = 0;

	if (ps->firmware_owned || ps->inherited_on)
		return -EPERM;
	regmap_read(ps->regmap, ps->offset, &reg);

	return !!(reg & APPLE_PMGR_RESET);
}

static const struct reset_control_ops apple_pmgr_reset_ops = {
	.assert		= apple_pmgr_reset_assert,
	.deassert	= apple_pmgr_reset_deassert,
	.reset		= apple_pmgr_reset_reset,
	.status		= apple_pmgr_reset_status,
};

static int apple_pmgr_reset_xlate(struct reset_controller_dev *rcdev,
				  const struct of_phandle_args *reset_spec)
{
	return 0;
}

/*
 * genpd adds a power state that is on only below parents that are on. With a
 * single parent the platform bus has already attached this device to it and
 * powered it on before probe; with more than one it attaches to none, so a
 * domain the boot loader left on (DISPEXT1_SYS on the J616s, below FAB7_SOC
 * and its PMP request) could not be added. Power every parent on through a
 * virtual device until the subdomain links hold them.
 */
static int apple_pmgr_ps_hold_parents(struct apple_pmgr_ps *ps, bool active,
				      struct dev_pm_domain_list **parents)
{
	static const struct dev_pm_domain_attach_data data = {
		.pd_flags = PD_FLAG_NO_DEV_LINK,
	};
	struct dev_pm_domain_list *list = NULL;
	unsigned int i;
	int ret;

	*parents = NULL;
	if (!active || of_count_phandle_with_args(ps->dev->of_node, "power-domains",
						  "#power-domain-cells") < 2)
		return 0;

	ret = dev_pm_domain_attach_list(ps->dev, &data, &list);
	if (ret <= 0)
		return ret;
	for (i = 0; i < list->num_pds; i++) {
		ret = pm_runtime_resume_and_get(list->pd_devs[i]);
		if (ret < 0) {
			while (i--)
				pm_runtime_put(list->pd_devs[i]);
			dev_pm_domain_detach_list(list);
			return dev_err_probe(ps->dev, ret, "cannot power parent domain %u\n", i);
		}
	}
	*parents = list;
	return 0;
}

static void apple_pmgr_ps_release_parents(struct dev_pm_domain_list *parents)
{
	unsigned int i;

	if (!parents)
		return;
	for (i = 0; i < parents->num_pds; i++)
		pm_runtime_put(parents->pd_devs[i]);
	dev_pm_domain_detach_list(parents);
}

/* Explicit DT policy wins; only the T8122 PPT needs the legacy fallback. */
static bool apple_pmgr_ps_always_on(struct device_node *node, u32 offset,
				   const char *name)
{
	return of_property_read_bool(node, "apple,always-on") ||
	       (of_device_is_compatible(node, "apple,t8122-pmgr-pwrstate") &&
		offset == APPLE_PMGR_T8122_PMS_C1PPT &&
		!strcmp(name, "pms_c1ppt"));
}

static int apple_pmgr_ps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct apple_pmgr_ps *ps;
	struct regmap *regmap;
	struct dev_pm_domain_list *parents = NULL;
	struct of_phandle_iterator it;
	int ret;
	const char *name;
	bool active, t8132, strict_state = false;
	struct device_node *tvm;

	t8132 = of_device_is_compatible(node, "apple,t8132-pmgr-pwrstate");
	/*
	 * On the M4 firmware path some domains are the firmware's, which Linux
	 * cannot access. The early compatibility adapter supplies the explicit
	 * status and ownership before population; fail closed if it was not
	 * enabled. Other firmware keeps the tree's own domain descriptions.
	 */
	if (t8132 && apple_t8132_firmware_path() && !of_property_present(node, "status"))
		return dev_err_probe(dev, -ENODEV, "missing M4 power-domain policy\n");

	tvm = of_parse_phandle(node, "power-domains", 0);
	if (t8132 && tvm)
		strict_state = of_device_is_compatible(tvm, "apple,t8132-gfx-tvm");
	of_node_put(tvm);

	regmap = syscon_node_to_regmap(node->parent);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ps = devm_kzalloc(dev, sizeof(*ps), GFP_KERNEL);
	if (!ps)
		return -ENOMEM;

	ps->dev = dev;
	ps->regmap = regmap;
	platform_set_drvdata(pdev, ps);
	ps->reset_time_us = t8132 ? APPLE_PMGR_T8132_RESET_TIME : APPLE_PMGR_RESET_TIME;
	ps->needs_reset_idle_wait = t8132;
	ps->macos_lower = of_device_is_compatible(node, "apple,t8140-pmgr-pwrstate");
	ps->no_auto_enable = t8132 && !strict_state;
	ps->strict_state = strict_state;

	ret = of_property_read_string(node, "label", &name);
	if (ret < 0) {
		dev_err(dev, "missing label property\n");
		return ret;
	}

	ret = of_property_read_u32(node, "reg", &ps->offset);
	if (ret < 0) {
		dev_err(dev, "missing reg property\n");
		return ret;
	}

	ps->wait_parent = of_device_is_compatible(node, "apple,t6000-pmgr-pwrstate") &&
			  ps->offset == 0x3c0;

	ps->genpd.flags |= GENPD_FLAG_IRQ_SAFE;
	ps->genpd.name = name;
	ps->genpd.power_on = apple_pmgr_ps_power_on;
	ps->genpd.power_off = apple_pmgr_ps_power_off;

	/*
	 * Firmware retains control of this domain across the handoff. Its state
	 * register may not be accessible to the CPU, so represent only an always-on
	 * vote and never read, reset or change the hardware state.
	 */
	ps->firmware_owned = of_property_read_bool(node, "apple,firmware-owned");
	ps->consumer_switched = of_property_read_bool(node, "apple,consumer-switched");
	if (ps->consumer_switched) {
		int n = of_property_count_u32_elems(node, "apple,retention-workaround");

		if (!ps->firmware_owned ||
		    (n > 0 && (n % 3 || n / 3 > APPLE_PMGR_PS_MAX_RETENTION)))
			return dev_err_probe(dev, -EINVAL, "bad consumer-switched policy\n");
		for (int i = 0; n > 0 && i < n / 3; i++) {
			of_property_read_u32_index(node, "apple,retention-workaround", 3 * i,
						   &ps->retention[i].offset);
			of_property_read_u32_index(node, "apple,retention-workaround", 3 * i + 1,
						   &ps->retention[i].mask);
			of_property_read_u32_index(node, "apple,retention-workaround", 3 * i + 2,
						   &ps->retention[i].value);
			if (ps->retention[i].value & ~ps->retention[i].mask)
				return dev_err_probe(dev, -EINVAL, "bad retention workaround\n");
		}
		ps->nr_retention = n > 0 ? n / 3 : 0;
	} else if (of_property_present(node, "apple,retention-workaround")) {
		return dev_err_probe(dev, -EINVAL,
				     "retention workaround needs a consumer switch\n");
	}
	if (of_property_present(node, "apple,bridge-link")) {
		struct of_phandle_args link;
		struct resource res;

		if (!ps->consumer_switched ||
		    of_parse_phandle_with_fixed_args(node, "apple,bridge-link", 2, 0, &link))
			return dev_err_probe(dev, -EINVAL, "bad bridge link\n");
		ret = of_address_to_resource(link.np, link.args[0], &res);
		of_node_put(link.np);
		if (ret)
			return dev_err_probe(dev, ret, "bad bridge link block\n");
		ps->bridge = devm_ioremap(dev, res.start + link.args[1], 8);
		if (!ps->bridge)
			return -ENOMEM;
	}
	ps->sleep_retained = of_property_read_bool(node, "apple,sleep-retained");
	ps->sleep_driver_owned = of_property_read_bool(node, "apple,sleep-driver-owned");
	ps->sleep_wakeup_path = of_property_read_bool(node, "apple,sleep-wakeup-path");
	/*
	 * Only T8132 hands over inherited-on domains with this contract. Older
	 * M3 device trees carry the property with another meaning; keep them
	 * as they were.
	 */
	ps->inherited_on = t8132 && of_property_read_bool(node, "apple,inherited-on");
	if (ps->inherited_on &&
	    (ps->firmware_owned || ps->sleep_retained ||
	     of_property_present(node, "apple,min-state") ||
	     of_property_present(node, "apple,always-on") ||
	     of_property_present(node, "apple,force-disable") ||
	     of_property_present(node, "apple,force-reset") ||
	     of_property_present(node, "apple,externally-clocked")))
		return dev_err_probe(dev, -EINVAL, "conflicting inherited power policy\n");

	ret = of_property_read_u32(node, "apple,min-state", &ps->min_state);
	if (ret == 0 && ps->min_state <= APPLE_PMGR_PS_ACTIVE && !ps->firmware_owned)
		regmap_update_bits(regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_PS_MIN,
				   FIELD_PREP(APPLE_PMGR_PS_MIN, ps->min_state));

	if (of_property_read_bool(node, "apple,force-disable"))
		ps->force_disable = true;

	if (of_property_read_bool(node, "apple,force-reset"))
		ps->force_reset = true;

	if (of_property_read_bool(node, "apple,externally-clocked"))
		ps->externally_clocked = true;

	/* A cold-started coprocessor CPU must not be gated behind its firmware. */
	ps->no_auto_pm = of_property_read_bool(node, "apple,no-auto-pm");

	/*
	 * An inherited-on domain has readable state but no transition ownership.
	 * A firmware-owned domain need not have a CPU-accessible state register;
	 * its availability is supplied by the handoff rather than an MMIO probe.
	 */
	if (ps->inherited_on) {
		ret = apple_pmgr_ps_check_inherited(ps);
		if (ret)
			return dev_err_probe(dev, ret, "domain was not handed over active\n");
		ps->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
		active = true;
	} else if (ps->firmware_owned) {
		ps->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
		active = true;
	} else {
		ret = apple_pmgr_ps_is_active(ps);
		if (ret < 0)
			return dev_err_probe(dev, ret, "cannot read power state\n");
		active = ret;
	}
	/* Legacy handoff DTs must also keep the throttle controller powered. */
	if (!ps->firmware_owned && !ps->inherited_on &&
	    apple_pmgr_ps_always_on(node, ps->offset, name)) {
		ps->genpd.flags |= GENPD_FLAG_ALWAYS_ON;
		if (!active) {
			/*
			 * The boot loader need not leave an always-on domain on,
			 * so this is not an error: apple_pmgr_ps_set() reports a
			 * domain that cannot be turned on.
			 */
			dev_info(dev, "always-on domain %s is off at boot, turning it on\n",
				 name);
			/*
			 * Turn it on so pm_genpd_init does not fail. genpd never
			 * powers an always-on domain, so keep requesting it on
			 * even if it does not get there in time.
			 */
			active = apple_pmgr_ps_set(&ps->genpd, APPLE_PMGR_PS_ACTIVE,
						   !ps->no_auto_enable && !ps->no_auto_pm) == 0;
		}
	} else if (active && !ps->firmware_owned && !ps->inherited_on) {
		ps->genpd.flags |= GENPD_FLAG_DEFER_OFF | GENPD_FLAG_ACTIVE_WAKEUP;
	} else if (ps->sleep_wakeup_path && !ps->firmware_owned && !ps->inherited_on) {
		/*
		 * Off at probe, powered up later by its consumer, which the sleep's
		 * commit stops after the noirq phase: hold it while that consumer is
		 * on the wakeup path.
		 */
		ps->genpd.flags |= GENPD_FLAG_ACTIVE_WAKEUP;
	}

	/* Turn on auto-PM if the domain is already on, unless it must stay off */
	if (active && !ps->firmware_owned && !ps->inherited_on && !ps->no_auto_enable)
		regmap_update_bits(regmap, ps->offset, APPLE_PMGR_FLAGS | APPLE_PMGR_AUTO_ENABLE,
				   ps->no_auto_pm ? 0 : APPLE_PMGR_AUTO_ENABLE);

	ret = pm_genpd_init(&ps->genpd, NULL, !active);
	if (ret < 0) {
		dev_err(dev, "pm_genpd_init failed\n");
		return ret;
	}
	WRITE_ONCE(apple_pmgr_genpd_runtime_suspend, ps->genpd.domain.ops.runtime_suspend);

	ret = of_genpd_add_provider_simple(node, &ps->genpd);
	if (ret < 0) {
		dev_err(dev, "of_genpd_add_provider_simple failed\n");
		return ret;
	}

	ret = apple_pmgr_ps_hold_parents(ps, active, &parents);
	if (ret < 0)
		goto err_remove;

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

	/* The subdomain links hold the parents now. */
	apple_pmgr_ps_release_parents(parents);

	/*
	 * Do not participate in regular PM; parent power domains are handled via the
	 * genpd hierarchy.
	 */
	pm_genpd_remove_device(dev);

	ps->rcdev.owner = THIS_MODULE;
	ps->rcdev.nr_resets = 1;
	ps->rcdev.ops = &apple_pmgr_reset_ops;
	ps->rcdev.of_node = dev->of_node;
	ps->rcdev.of_reset_n_cells = 0;
	ps->rcdev.of_xlate = apple_pmgr_reset_xlate;

	ret = devm_reset_controller_register(dev, &ps->rcdev);
	if (ret < 0)
		goto err_remove;

	return 0;
err_remove:
	apple_pmgr_ps_release_parents(parents);
	of_genpd_del_provider(node);
	pm_genpd_remove(&ps->genpd);
	return ret;
}

static struct platform_driver apple_pmgr_ps_driver;

/* Wait for the masked state word to read @want; @cur gets the last value. */
static int apple_pmgr_ps_wait(struct apple_pmgr_ps *ps, u32 mask, u32 want, u32 *cur)
{
	return regmap_read_poll_timeout_atomic(ps->regmap, ps->offset, *cur,
					       (*cur & mask) == want, 1,
					       APPLE_PMGR_PS_CONSUMER_TIMEOUT);
}

/* Write a retention register and read it back. */
static int apple_pmgr_ps_retention_set(struct apple_pmgr_ps *ps, unsigned int i, u32 value)
{
	u32 now;
	int ret;

	ret = regmap_write(ps->regmap, ps->retention[i].offset, value);
	if (!ret)
		ret = regmap_read(ps->regmap, ps->retention[i].offset, &now);
	return ret ?: now == value ? 0 : -EIO;
}

/* Fully on (target and actual), and not in reset, busy or disabled. */
static bool apple_pmgr_ps_consumer_ready(u32 reg)
{
	return (reg & APPLE_PMGR_PS_LEVELS) == APPLE_PMGR_PS_ALL_ON &&
	       !(reg & (APPLE_PMGR_RESET | APPLE_PMGR_PS_RESET | APPLE_PMGR_BUSY |
			APPLE_PMGR_DEV_DISABLE));
}

/*
 * The native power-off of a consumer-switched state: from fully on, disable
 * automatic power management while staying on, set the retention workaround,
 * then power off (clearing the history bits) and wait for it.
 */
/* Only with the state on: the link down before its power-off, up after its power-on. */
static void apple_pmgr_ps_bridge(struct apple_pmgr_ps *ps, bool up)
{
	if (!ps->bridge)
		return;
	for (unsigned int i = 0; i < 2; i++) {
		u32 value = readl(ps->bridge + 4 * i);

		writel(up ? value | BIT(0) : value & ~BIT(0), ps->bridge + 4 * i);
	}
	/* Complete the stores before the state word changes or its block is used. */
	mb();
	readl(ps->bridge);
	dev_dbg(ps->dev, "PS %s: bridge link %s\n", ps->genpd.name, up ? "up" : "down");
}

static int apple_pmgr_ps_consumer_state_off(struct apple_pmgr_ps *ps)
{
	u32 reg, cur;
	int ret;

	if (ps->consumer_failed)
		return -EIO;
	if (ps->consumer_off)
		return -EALREADY;
	ret = regmap_read(ps->regmap, ps->offset, &reg);
	if (ret)
		goto fail;
	if (!apple_pmgr_ps_consumer_ready(reg))
		return -EBUSY;	/* Not touched: not a failure of this state. */
	for (unsigned int i = 0; i < ps->nr_retention; i++) {
		ret = regmap_read(ps->regmap, ps->retention[i].offset, &ps->retention[i].saved);
		if (ret)
			goto fail;
	}
	ps->consumer_saved = reg;
	reg &= ~(APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_FLAGS);
	if (ps->consumer_saved & APPLE_PMGR_AUTO_ENABLE) {
		ret = regmap_write(ps->regmap, ps->offset, reg);
		if (!ret)
			ret = apple_pmgr_ps_wait(ps, APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_PS_LEVELS,
						 APPLE_PMGR_PS_ALL_ON, &cur);
		if (ret)
			goto fail;
	}
	for (unsigned int i = 0; i < ps->nr_retention; i++) {
		ret = apple_pmgr_ps_retention_set(ps, i, (ps->retention[i].saved &
							   ~ps->retention[i].mask) |
							  ps->retention[i].value);
		if (ret)
			goto fail;
	}
	apple_pmgr_ps_bridge(ps, false);
	ps->consumer_off = true;
	reg = (reg & ~APPLE_PMGR_PS_TARGET) | APPLE_PMGR_FLAGS |
	      FIELD_PREP(APPLE_PMGR_PS_TARGET, APPLE_PMGR_PS_PWRGATE);
	ret = regmap_write(ps->regmap, ps->offset, reg);
	if (!ret)
		ret = apple_pmgr_ps_wait(ps, APPLE_PMGR_PS_LEVELS | APPLE_PMGR_AUTO_ENABLE, 0,
					 &cur);
	if (ret)
		goto fail;
	dev_dbg(ps->dev, "PS %s: consumer power-off: %#x -> %#x\n", ps->genpd.name,
		ps->consumer_saved, cur);
	return 0;
fail:
	ps->consumer_failed = true;
	dev_err(ps->dev, "PS %s: consumer power-off failed: %d\n", ps->genpd.name, ret);
	return ret;
}

/*
 * The native power-on: power on with automatic power management disabled,
 * give the retention registers' workaround bits back the values they had
 * (only those: power-off changed no other), then automatic power management
 * as it was.
 */
static int apple_pmgr_ps_consumer_state_on(struct apple_pmgr_ps *ps)
{
	u32 reg, cur, now;
	int ret;

	if (ps->consumer_failed)
		return -EIO;
	if (!ps->consumer_off)
		return -EALREADY;
	reg = (ps->consumer_saved & ~(APPLE_PMGR_AUTO_ENABLE | APPLE_PMGR_FLAGS |
				      APPLE_PMGR_BUSY | APPLE_PMGR_PS_ACTUAL)) |
	      FIELD_PREP(APPLE_PMGR_PS_TARGET, APPLE_PMGR_PS_ACTIVE);
	ret = regmap_write(ps->regmap, ps->offset, reg);
	if (!ret)
		ret = apple_pmgr_ps_wait(ps, APPLE_PMGR_PS_LEVELS, APPLE_PMGR_PS_ALL_ON, &cur);
	if (ret)
		goto fail;
	apple_pmgr_ps_bridge(ps, true);
	for (unsigned int i = 0; i < ps->nr_retention; i++) {
		u32 mask = ps->retention[i].mask;

		ret = regmap_read(ps->regmap, ps->retention[i].offset, &now);
		if (!ret)
			ret = apple_pmgr_ps_retention_set(ps, i, (now & ~mask) |
							  (ps->retention[i].saved & mask));
		if (ret)
			goto fail;
	}
	if (ps->consumer_saved & APPLE_PMGR_AUTO_ENABLE) {
		ret = regmap_write(ps->regmap, ps->offset, reg | APPLE_PMGR_AUTO_ENABLE);
		if (!ret)
			ret = apple_pmgr_ps_wait(ps, APPLE_PMGR_AUTO_ENABLE, APPLE_PMGR_AUTO_ENABLE,
						 &cur);
		if (ret)
			goto fail;
	}
	ps->consumer_off = false;
	dev_dbg(ps->dev, "PS %s: consumer power-on: %#x\n", ps->genpd.name, cur);
	return 0;
fail:
	ps->consumer_failed = true;
	dev_err(ps->dev, "PS %s: consumer power-on failed: %d\n", ps->genpd.name, ret);
	return ret;
}

/* The consumer-switched states from @consumer's domain up, child first. */
static int apple_pmgr_ps_consumer_chain(struct device *consumer,
					struct apple_pmgr_ps **chain)
{
	struct device_node *np = of_parse_phandle(consumer->of_node, "power-domains", 0);
	int n = 0;

	while (np) {
		struct platform_device *pdev = of_find_device_by_node(np);
		struct apple_pmgr_ps *ps = NULL;
		struct device_node *parent;

		if (pdev && pdev->dev.driver == &apple_pmgr_ps_driver.driver)
			ps = platform_get_drvdata(pdev);
		if (pdev)
			put_device(&pdev->dev);
		if (!ps || !ps->consumer_switched || n == APPLE_PMGR_PS_MAX_CHAIN) {
			of_node_put(np);
			break;
		}
		chain[n++] = ps;
		parent = of_parse_phandle(np, "power-domains", 0);
		of_node_put(np);
		np = parent;
	}
	return n ?: -ENODEV;
}

static int apple_pmgr_ps_consumer_switch(struct apple_pmgr_ps *ps, bool on)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ps->genpd.slock, flags);
	ret = on ? apple_pmgr_ps_consumer_state_on(ps) : apple_pmgr_ps_consumer_state_off(ps);
	spin_unlock_irqrestore(&ps->genpd.slock, flags);
	return ret;
}

/*
 * Whether apple_pmgr_ps_consumer_off() can run for @consumer, changing
 * nothing: the whole chain consumer-switched, not failed and not off. Not
 * whether each state is fully on: under automatic power management the
 * J713 DCP CPU state reads level 0 while its CPU idles, and is fully on
 * again once the CPU has been stopped, which the power-off then requires.
 */
int apple_pmgr_ps_consumer_check(struct device *consumer)
{
	struct apple_pmgr_ps *chain[APPLE_PMGR_PS_MAX_CHAIN];
	int n = apple_pmgr_ps_consumer_chain(consumer, chain), ret = 0;

	if (n < 0)
		return n;
	for (int i = 0; i < n && !ret; i++) {
		struct apple_pmgr_ps *ps = chain[i];
		unsigned long flags;

		spin_lock_irqsave(&ps->genpd.slock, flags);
		if (ps->consumer_failed)
			ret = -EIO;
		else if (ps->consumer_off)
			ret = -EALREADY;
		spin_unlock_irqrestore(&ps->genpd.slock, flags);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmgr_ps_consumer_check);

int apple_pmgr_ps_consumer_off(struct device *consumer)
{
	struct apple_pmgr_ps *chain[APPLE_PMGR_PS_MAX_CHAIN];
	int n = apple_pmgr_ps_consumer_chain(consumer, chain), ret;

	if (n < 0)
		return n;
	for (int i = 0; i < n; i++) {
		ret = apple_pmgr_ps_consumer_switch(chain[i], false);
		if (ret)
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(apple_pmgr_ps_consumer_off);

int apple_pmgr_ps_consumer_on(struct device *consumer)
{
	struct apple_pmgr_ps *chain[APPLE_PMGR_PS_MAX_CHAIN];
	int n = apple_pmgr_ps_consumer_chain(consumer, chain), ret;

	if (n < 0)
		return n;
	for (int i = n - 1; i >= 0; i--) {
		ret = apple_pmgr_ps_consumer_switch(chain[i], true);
		/* Also a rollback after a power-off that stopped partway: skip what stayed on. */
		if (ret && ret != -EALREADY)
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(apple_pmgr_ps_consumer_on);

/*
 * Whether @consumer's own state, the first of its chain, is powered off: after
 * a power-off that stopped partway, whether its block went through reset.
 */
bool apple_pmgr_ps_consumer_is_off(struct device *consumer)
{
	struct apple_pmgr_ps *chain[APPLE_PMGR_PS_MAX_CHAIN];
	int n = apple_pmgr_ps_consumer_chain(consumer, chain);
	unsigned long flags;
	bool off;

	if (n < 0)
		return false;
	spin_lock_irqsave(&chain[0]->genpd.slock, flags);
	off = chain[0]->consumer_off;
	spin_unlock_irqrestore(&chain[0]->genpd.slock, flags);
	return off;
}
EXPORT_SYMBOL_GPL(apple_pmgr_ps_consumer_is_off);

static const struct of_device_id apple_pmgr_ps_of_match[] = {
	{ .compatible = "apple,t8103-pmgr-pwrstate" },
	{ .compatible = "apple,pmgr-pwrstate" },
	{}
};

MODULE_DEVICE_TABLE(of, apple_pmgr_ps_of_match);

static const struct dev_pm_ops apple_pmgr_ps_pm_ops = {
	LATE_SYSTEM_SLEEP_PM_OPS(apple_pmgr_ps_suspend_late, apple_pmgr_ps_resume_early)
	NOIRQ_SYSTEM_SLEEP_PM_OPS(apple_pmgr_ps_suspend_noirq, apple_pmgr_ps_resume_noirq)
};

static struct platform_driver apple_pmgr_ps_driver = {
	.probe = apple_pmgr_ps_probe,
	.driver = {
		.name = "apple-pmgr-pwrstate",
		.of_match_table = apple_pmgr_ps_of_match,
		.pm = pm_sleep_ptr(&apple_pmgr_ps_pm_ops),
	},
};

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("PMGR power state driver for Apple SoCs");

module_platform_driver(apple_pmgr_ps_driver);

#if IS_ENABLED(CONFIG_APPLE_PMGR_PWRSTATE_KUNIT_TEST)
#include "pmgr-pwrstate-test.c"
#endif
