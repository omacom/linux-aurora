// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SoC PMC voter interface driver
 *
 * The PMC arbitrates the performance state of shared rails, such as the
 * DRAM controller (DCS) and the SoC fabric, between voter agents: the PMP,
 * the host performance controller (CLPC), the GPU and the application
 * processor. Each agent owns a perf-state floor word holding one 4-bit
 * state per rail, and each rail has an interface enable word with one bit
 * per agent. The PMC only honours the floor of an agent whose interface is
 * enabled on that rail.
 *
 * At startup macOS enables the static agents (PMP, CLPC and AP) on their
 * rails, and enables a dynamic agent such as the GPU only while the PMGR
 * device it belongs to is powered on: after the device has been powered
 * on, and before it is powered off. This driver does the former at probe
 * and models each dynamic agent as a power domain that sits between the
 * device and its PMGR power state, which gives the same ordering.
 *
 * Under thermal pressure the driver can also limit the DRAM controller
 * (DCS) rail: it stops honouring the other agents on that rail and votes
 * on their behalf through the AP's own agent, capped. This bounds the heat
 * of the PMP's DCS vote, which follows GPU load.
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/soc/apple/pmc.h>
#include <linux/spinlock.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

#if IS_ENABLED(CONFIG_APPLE_PMC_KUNIT_TEST)
#include <kunit/static_stub.h>
#endif

#define APPLE_PMC_MAX_RAILS	8
#define APPLE_PMC_MAX_AGENTS	32
#define APPLE_PMC_MAX_STATES	16

/* How often a DCS limit re-reads the votes it stands in for. */
#define APPLE_PMC_DCS_POLL_MS	20

/**
 * struct apple_pmc_hw - per-SoC layout of the PMC voter interface
 * @agent_stride: distance between the floor words of consecutive agents
 *	in the "voters" region
 * @agents: number of voter agents
 * @rails: number of rails
 * @agent_state: offset of the per-agent state words in the "rails" region;
 *	each holds the agent's floor as the PMC honours it: masked by the
 *	agent's interface enables and clamped to each rail's highest state
 * @rail_enable: offset of the per-rail interface enable words in the
 *	"rails" region
 * @dcs_rail: rail of the DRAM controller (DCS)
 * @dcs_max: highest DCS performance state
 * @ap_agent: voter agent of the application processor, which a DCS limit
 *	votes through
 * @has_pending: the word after each agent's floor word holds one pending
 *	bit per agent, set until the PMC has taken up that agent's last
 *	floor
 */
struct apple_pmc_hw {
	u32 agent_stride;
	u32 agents;
	u32 rails;
	u32 agent_state;
	u32 rail_enable;
	u32 dcs_rail;
	u32 dcs_max;
	u32 ap_agent;
	bool has_pending;
};

struct apple_pmc {
	struct device *dev;
	const struct apple_pmc_hw *hw;
	void __iomem *voters;
	void __iomem *rails;
	/*
	 * Serializes read-modify-write cycles on enable and floor words, and
	 * protects @enable and @dcs_state.
	 */
	raw_spinlock_t lock;
	u32 saved_enable[APPLE_PMC_MAX_RAILS];
	/* Agents enabled on each rail, before any DCS limit is applied. */
	u32 enable[APPLE_PMC_MAX_RAILS];
	u32 static_agents;
	struct list_head domains;
	struct dentry *debugfs;
	/* Cooling state of the DCS limit: 0 is none, dcs_max holds DCS at 0. */
	unsigned long dcs_state;
	/* DCS state the AP's agent voted for before probe, kept without a limit. */
	unsigned int ap_dcs_vote;
	/* Power cost of each DCS state in mW, if a model is described. */
	u32 dcs_power[APPLE_PMC_MAX_STATES];
	struct delayed_work dcs_work;
};

struct apple_pmc_domain {
	struct generic_pm_domain genpd;
	struct apple_pmc *pmc;
	struct device_node *np;
	struct list_head list;
	unsigned int agent;
	bool provider;
};

#define genpd_to_apple_pmc_domain(_genpd) \
	container_of(_genpd, struct apple_pmc_domain, genpd)

static struct dentry *apple_pmc_debugfs_root;

static u32 apple_pmc_enable_read(struct apple_pmc *pmc, unsigned int rail)
{
#if IS_ENABLED(CONFIG_APPLE_PMC_KUNIT_TEST)
	KUNIT_STATIC_STUB_REDIRECT(apple_pmc_enable_read, pmc, rail);
#endif
	return readl(pmc->rails + pmc->hw->rail_enable + 4 * rail);
}

/*
 * Caller holds pmc->lock. While the DCS rail is limited, only the AP's agent
 * is enabled on it. Every write is read back, as macOS does.
 */
static int apple_pmc_enable_write(struct apple_pmc *pmc, unsigned int rail)
{
	void __iomem *reg = pmc->rails + pmc->hw->rail_enable + 4 * rail;
	u32 old = apple_pmc_enable_read(pmc, rail);
	u32 val = pmc->enable[rail];

	if (pmc->dcs_state && rail == pmc->hw->dcs_rail)
		val = BIT(pmc->hw->ap_agent);
	if (val == old)
		return 0;

	writel(val, reg);
	if (apple_pmc_enable_read(pmc, rail) != val) {
		dev_err_ratelimited(pmc->dev,
				    "rail %u enable %#x -> %#x did not stick\n",
				    rail, old, val);
		return -EIO;
	}

	return 0;
}

/* Caller holds pmc->lock. */
static int apple_pmc_enable_update(struct apple_pmc *pmc, unsigned int rail,
				   u32 set, u32 clear)
{
	pmc->enable[rail] = (pmc->enable[rail] | set) & ~clear;

	return apple_pmc_enable_write(pmc, rail);
}

/*
 * Caller holds pmc->lock. Put the rails up to and including @last back to
 * the enables in @prev, after a change failed on @last.
 */
static void apple_pmc_enable_restore(struct apple_pmc *pmc, const u32 *prev,
				     unsigned int last)
{
	unsigned int rail;

	for (rail = 0; rail <= last; rail++) {
		pmc->enable[rail] = prev[rail];
		apple_pmc_enable_write(pmc, rail);
	}
}

/*
 * Enable or disable @agent on every rail. If a rail does not take the
 * change, undo it on the rails changed so far: genpd keeps a domain whose
 * power_on() fails off, and never calls its power_off(), so an agent left
 * enabled on some rails would stay enabled for good.
 */
static int apple_pmc_agent_enable(struct apple_pmc *pmc, unsigned int agent,
				  bool enable)
{
	u32 prev[APPLE_PMC_MAX_RAILS];
	unsigned long flags;
	unsigned int rail;
	int ret = 0;

	raw_spin_lock_irqsave(&pmc->lock, flags);
	memcpy(prev, pmc->enable, sizeof(prev));
	for (rail = 0; rail < pmc->hw->rails; rail++) {
		ret = apple_pmc_enable_update(pmc, rail,
					      enable ? BIT(agent) : 0,
					      enable ? 0 : BIT(agent));
		if (ret) {
			apple_pmc_enable_restore(pmc, prev, rail);
			break;
		}
	}
	raw_spin_unlock_irqrestore(&pmc->lock, flags);

	return ret;
}

static struct apple_pmc *apple_pmc_from_dev(struct device *dev)
{
	return dev ? dev_get_drvdata(dev) : NULL;
}

/**
 * apple_pmc_rail_count() - number of rails of a PMC
 * @dev: the PMC device
 *
 * Return: the number of rails, or 0 if @dev is not a bound PMC.
 */
unsigned int apple_pmc_rail_count(struct device *dev)
{
	struct apple_pmc *pmc = apple_pmc_from_dev(dev);

	return pmc ? pmc->hw->rails : 0;
}
EXPORT_SYMBOL_GPL(apple_pmc_rail_count);

/**
 * apple_pmc_voter_enabled() - whether the PMC honours an agent on a rail
 * @dev: the PMC device
 * @agent: voter agent
 * @rail: rail
 *
 * Context: Any context.
 */
bool apple_pmc_voter_enabled(struct device *dev, unsigned int agent,
			     unsigned int rail)
{
	struct apple_pmc *pmc = apple_pmc_from_dev(dev);

	if (!pmc || agent >= pmc->hw->agents || rail >= pmc->hw->rails)
		return false;

	return apple_pmc_enable_read(pmc, rail) & BIT(agent);
}
EXPORT_SYMBOL_GPL(apple_pmc_voter_enabled);

/**
 * apple_pmc_floor_read() - read the perf-state floor word of an agent
 * @dev: the PMC device
 * @agent: voter agent
 *
 * Context: Any context.
 * Return: the floor word, or 0 if @agent is out of range.
 */
u32 apple_pmc_floor_read(struct device *dev, unsigned int agent)
{
	struct apple_pmc *pmc = apple_pmc_from_dev(dev);

	if (!pmc || agent >= pmc->hw->agents)
		return 0;

	return readl(pmc->voters + agent * pmc->hw->agent_stride);
}
EXPORT_SYMBOL_GPL(apple_pmc_floor_read);

/**
 * apple_pmc_floor_update() - update the perf-state floor of an agent
 * @dev: the PMC device
 * @agent: voter agent
 * @mask: bits of the floor word to change, see APPLE_PMC_RAIL_MASK()
 * @value: new value of the bits in @mask
 *
 * The floor only takes effect on rails where the agent's interface is
 * enabled.
 *
 * On SoCs with a pending bit, wait until the PMC has taken up the
 * previous floor before writing, and until it has taken up the new one.
 *
 * Context: Any context.
 * Return: 0 on success, -EINVAL if @agent is out of range, -EIO if the PMC
 * did not accept the new floor, or -ETIMEDOUT if it did not take up a floor
 * within 1 ms.
 */
int apple_pmc_floor_update(struct device *dev, unsigned int agent, u32 mask,
			   u32 value)
{
	struct apple_pmc *pmc = apple_pmc_from_dev(dev);
	void __iomem *reg;
	unsigned long flags;
	u32 old, val;
	int ret = 0;

	if (!pmc || agent >= pmc->hw->agents)
		return -EINVAL;

	reg = pmc->voters + agent * pmc->hw->agent_stride;
	raw_spin_lock_irqsave(&pmc->lock, flags);
	if (pmc->hw->has_pending) {
		ret = readl_poll_timeout_atomic(reg + 4, old, !(old & BIT(agent)),
						1, 1000);
		if (ret)
			goto unlock;
	}
	old = readl(reg);
	val = (old & ~mask) | (value & mask);
	if (val != old) {
		writel(val, reg);
		if ((readl(reg) & mask) != (value & mask))
			ret = -EIO;
	}
	if (!ret && pmc->hw->has_pending)
		ret = readl_poll_timeout_atomic(reg + 4, old, !(old & BIT(agent)),
						1, 1000);
unlock:
	raw_spin_unlock_irqrestore(&pmc->lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmc_floor_update);

/*
 * Caller holds pmc->lock. The highest DCS state that the agents enabled on
 * the DCS rail ask for, the AP's own agent excepted.
 */
static unsigned int apple_pmc_dcs_demand(struct apple_pmc *pmc)
{
	const struct apple_pmc_hw *hw = pmc->hw;
	unsigned long agents = pmc->enable[hw->dcs_rail] & ~BIT(hw->ap_agent);
	unsigned int agent, demand = 0;
	u32 floor;

	for_each_set_bit(agent, &agents, hw->agents) {
		floor = readl(pmc->voters + agent * hw->agent_stride);
		demand = max(demand, (floor & APPLE_PMC_RAIL_MASK(hw->dcs_rail)) >>
				     APPLE_PMC_RAIL_SHIFT(hw->dcs_rail));
	}

	return min(demand, hw->dcs_max);
}

/* The DCS state the AP's agent votes for. */
static unsigned int apple_pmc_ap_dcs_read(struct apple_pmc *pmc)
{
	const struct apple_pmc_hw *hw = pmc->hw;
	u32 floor = readl(pmc->voters + hw->ap_agent * hw->agent_stride);

	return (floor & APPLE_PMC_RAIL_MASK(hw->dcs_rail)) >>
	       APPLE_PMC_RAIL_SHIFT(hw->dcs_rail);
}

/*
 * Caller holds pmc->lock. Vote through the AP's agent for what the other
 * agents ask for, capped by the limit; without a limit, put back the vote
 * the AP's agent had before probe.
 */
static void apple_pmc_dcs_vote(struct apple_pmc *pmc)
{
	const struct apple_pmc_hw *hw = pmc->hw;
	void __iomem *reg = pmc->voters + hw->ap_agent * hw->agent_stride;
	u32 mask = APPLE_PMC_RAIL_MASK(hw->dcs_rail);
	unsigned int state = pmc->ap_dcs_vote;
	u32 old, val;

	if (pmc->dcs_state)
		state = min_t(unsigned int, apple_pmc_dcs_demand(pmc),
			      hw->dcs_max - pmc->dcs_state);

	old = readl(reg);
	val = (old & ~mask) | (state << APPLE_PMC_RAIL_SHIFT(hw->dcs_rail));
	if (val != old)
		writel(val, reg);
}

static void apple_pmc_dcs_work(struct work_struct *work)
{
	struct apple_pmc *pmc = container_of(to_delayed_work(work),
					     struct apple_pmc, dcs_work);
	unsigned long flags;
	bool limited;

	raw_spin_lock_irqsave(&pmc->lock, flags);
	apple_pmc_dcs_vote(pmc);
	limited = pmc->dcs_state;
	raw_spin_unlock_irqrestore(&pmc->lock, flags);

	if (limited)
		queue_delayed_work(system_freezable_power_efficient_wq,
				   &pmc->dcs_work,
				   msecs_to_jiffies(APPLE_PMC_DCS_POLL_MS));
}

static int apple_pmc_dcs_get_max_state(struct thermal_cooling_device *cdev,
				       unsigned long *state)
{
	struct apple_pmc *pmc = cdev->devdata;

	*state = pmc->hw->dcs_max;
	return 0;
}

static int apple_pmc_dcs_get_cur_state(struct thermal_cooling_device *cdev,
				       unsigned long *state)
{
	struct apple_pmc *pmc = cdev->devdata;

	*state = READ_ONCE(pmc->dcs_state);
	return 0;
}

/*
 * Caller holds pmc->lock. Vote the capped demand before the other agents
 * stop being honoured, and drop that vote only once they are honoured
 * again, so that the rail never dips below either.
 */
static int apple_pmc_dcs_limit(struct apple_pmc *pmc, unsigned long state)
{
	unsigned int rail = pmc->hw->dcs_rail;
	int ret;

	WRITE_ONCE(pmc->dcs_state, state);
	if (state) {
		apple_pmc_dcs_vote(pmc);
		return apple_pmc_enable_write(pmc, rail);
	}

	ret = apple_pmc_enable_write(pmc, rail);
	if (!ret)
		apple_pmc_dcs_vote(pmc);

	return ret;
}

static int apple_pmc_dcs_set_cur_state(struct thermal_cooling_device *cdev,
				       unsigned long state)
{
	struct apple_pmc *pmc = cdev->devdata;
	unsigned long flags, old;
	bool limited;
	int ret = 0;

	if (state > pmc->hw->dcs_max)
		return -EINVAL;

	raw_spin_lock_irqsave(&pmc->lock, flags);
	old = pmc->dcs_state;
	if (state != old) {
		ret = apple_pmc_dcs_limit(pmc, state);
		/*
		 * The rail enable did not change, so neither did the limit:
		 * go back to the old state, which the next call then retries
		 * to leave.
		 */
		if (ret)
			apple_pmc_dcs_limit(pmc, old);
	}
	limited = pmc->dcs_state;
	raw_spin_unlock_irqrestore(&pmc->lock, flags);

	if (limited)
		queue_delayed_work(system_freezable_power_efficient_wq,
				   &pmc->dcs_work,
				   msecs_to_jiffies(APPLE_PMC_DCS_POLL_MS));
	else
		cancel_delayed_work(&pmc->dcs_work);

	return ret;
}

static int apple_pmc_dcs_get_requested_power(struct thermal_cooling_device *cdev,
					     u32 *power)
{
	struct apple_pmc *pmc = cdev->devdata;
	unsigned long flags;
	unsigned int demand;

	raw_spin_lock_irqsave(&pmc->lock, flags);
	demand = apple_pmc_dcs_demand(pmc);
	raw_spin_unlock_irqrestore(&pmc->lock, flags);

	*power = pmc->dcs_power[demand];
	return 0;
}

static int apple_pmc_dcs_state2power(struct thermal_cooling_device *cdev,
				     unsigned long state, u32 *power)
{
	struct apple_pmc *pmc = cdev->devdata;

	if (state > pmc->hw->dcs_max)
		return -EINVAL;

	*power = pmc->dcs_power[pmc->hw->dcs_max - state];
	return 0;
}

static int apple_pmc_dcs_power2state(struct thermal_cooling_device *cdev,
				     u32 power, unsigned long *state)
{
	struct apple_pmc *pmc = cdev->devdata;
	unsigned long s;

	for (s = 0; s < pmc->hw->dcs_max; s++)
		if (pmc->dcs_power[pmc->hw->dcs_max - s] <= power)
			break;

	*state = s;
	return 0;
}

static const struct thermal_cooling_device_ops apple_pmc_dcs_ops = {
	.get_max_state = apple_pmc_dcs_get_max_state,
	.get_cur_state = apple_pmc_dcs_get_cur_state,
	.set_cur_state = apple_pmc_dcs_set_cur_state,
};

static const struct thermal_cooling_device_ops apple_pmc_dcs_power_ops = {
	.get_max_state = apple_pmc_dcs_get_max_state,
	.get_cur_state = apple_pmc_dcs_get_cur_state,
	.set_cur_state = apple_pmc_dcs_set_cur_state,
	.get_requested_power = apple_pmc_dcs_get_requested_power,
	.state2power = apple_pmc_dcs_state2power,
	.power2state = apple_pmc_dcs_power2state,
};

/*
 * Register the DCS limit as a cooling device. With a power model it is a
 * power actor that the power allocator governor can drive.
 */
static int apple_pmc_dcs_init(struct apple_pmc *pmc)
{
	const struct thermal_cooling_device_ops *ops = &apple_pmc_dcs_ops;
	const struct apple_pmc_hw *hw = pmc->hw;
	struct device *dev = pmc->dev;
	struct device_node *np = dev->of_node;
	struct thermal_cooling_device *cdev;
	u32 uw[APPLE_PMC_MAX_STATES];
	unsigned int i;
	int count, ret;

	if (!IS_ENABLED(CONFIG_THERMAL) ||
	    !of_property_present(np, "#cooling-cells"))
		return 0;

	count = of_property_count_u32_elems(np, "apple,dcs-power-microwatt");
	if (count > 0) {
		if (count != hw->dcs_max + 1)
			return dev_err_probe(dev, -EINVAL,
					     "need %u DCS power values\n",
					     hw->dcs_max + 1);
		ret = of_property_read_u32_array(np, "apple,dcs-power-microwatt",
						 uw, count);
		if (ret)
			return ret;
		for (i = 0; i < count; i++) {
			pmc->dcs_power[i] = uw[i] / 1000;
			if (i && pmc->dcs_power[i] < pmc->dcs_power[i - 1])
				return dev_err_probe(dev, -EINVAL,
						     "DCS power must not decrease\n");
		}
		ops = &apple_pmc_dcs_power_ops;
	}

	cdev = devm_thermal_of_cooling_device_register(dev, 0, "pmc-dcs", pmc,
						       ops);
	if (IS_ERR(cdev))
		return dev_err_probe(dev, PTR_ERR(cdev),
				     "failed to register the DCS limit\n");

	return 0;
}

static int apple_pmc_domain_power_on(struct generic_pm_domain *genpd)
{
	struct apple_pmc_domain *dom = genpd_to_apple_pmc_domain(genpd);

	return apple_pmc_agent_enable(dom->pmc, dom->agent, true);
}

static int apple_pmc_domain_power_off(struct generic_pm_domain *genpd)
{
	struct apple_pmc_domain *dom = genpd_to_apple_pmc_domain(genpd);

	return apple_pmc_agent_enable(dom->pmc, dom->agent, false);
}

static void apple_pmc_domain_free(struct apple_pmc_domain *dom)
{
	if (dom->provider)
		of_genpd_del_provider(dom->np);
	if (pm_genpd_remove(&dom->genpd)) {
		/* The core still owns this domain. Keep its storage alive. */
		dev_warn(dom->pmc->dev, "retaining busy power domain %s\n",
			 dom->genpd.name);
		return;
	}
	of_node_put(dom->np);
	kfree(dom->genpd.name);
	kfree(dom);
}

static int apple_pmc_domain_add(struct apple_pmc *pmc, struct device_node *np)
{
	struct device *dev = pmc->dev;
	struct apple_pmc_domain *dom;
	struct of_phandle_args parent;
	const char *label;
	u32 agent;
	int i, ret;

	ret = of_property_read_u32(np, "reg", &agent);
	if (ret)
		return dev_err_probe(dev, ret, "%pOF: missing reg\n", np);
	if (agent >= pmc->hw->agents)
		return dev_err_probe(dev, -EINVAL, "%pOF: invalid agent %u\n",
				     np, agent);
	if (pmc->static_agents & BIT(agent))
		return dev_err_probe(dev, -EINVAL,
				     "%pOF: agent %u is also static\n", np,
				     agent);

	dom = kzalloc_obj(*dom);
	if (!dom)
		return -ENOMEM;

	dom->pmc = pmc;
	dom->np = of_node_get(np);
	dom->agent = agent;
	if (of_property_read_string(np, "label", &label))
		label = np->name;
	dom->genpd.name = kasprintf(GFP_KERNEL, "pmc-%s", label);
	if (!dom->genpd.name) {
		of_node_put(dom->np);
		kfree(dom);
		return -ENOMEM;
	}
	dom->genpd.power_on = apple_pmc_domain_power_on;
	dom->genpd.power_off = apple_pmc_domain_power_off;

	/*
	 * Start from the hardware state, so that an agent enabled by the
	 * bootloader is not disabled under a device that is still running.
	 */
	ret = pm_genpd_init(&dom->genpd, NULL,
			    !(apple_pmc_enable_read(pmc, 0) & BIT(agent)));
	if (ret) {
		of_node_put(dom->np);
		kfree(dom->genpd.name);
		kfree(dom);
		return dev_err_probe(dev, ret, "%pOF: genpd init failed\n", np);
	}
	list_add_tail(&dom->list, &pmc->domains);

	ret = of_genpd_add_provider_simple(np, &dom->genpd);
	if (ret)
		return dev_err_probe(dev, ret, "%pOF: provider failed\n", np);
	dom->provider = true;

	for (i = 0; !of_parse_phandle_with_args(np, "power-domains",
						"#power-domain-cells", i,
						&parent); i++) {
		struct of_phandle_args child = { .np = np };

		ret = of_genpd_add_subdomain(&parent, &child);
		of_node_put(parent.np);
		if (ret)
			return dev_err_probe(dev, ret,
					     "%pOF: failed to join parent domain\n",
					     np);
	}

	return 0;
}

static int apple_pmc_status_show(struct seq_file *s, void *unused)
{
	struct apple_pmc *pmc = s->private;
	const struct apple_pmc_hw *hw = pmc->hw;
	unsigned int i;

	seq_printf(s, "static agents: %#x\n", pmc->static_agents);
	for (i = 0; i < hw->agents; i++)
		seq_printf(s, "agent%u floor: %#010x honoured: %#010x\n", i,
			   readl(pmc->voters + i * hw->agent_stride),
			   readl(pmc->rails + hw->agent_state + 4 * i));
	for (i = 0; i < hw->rails; i++)
		seq_printf(s, "rail%u enable: %#010x (saved %#010x)\n",
			   i, apple_pmc_enable_read(pmc, i), pmc->saved_enable[i]);
	seq_printf(s, "dcs limit: state %lu of %u\n", READ_ONCE(pmc->dcs_state),
		   hw->dcs_max);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_pmc_status);

static void apple_pmc_release(void *data)
{
	struct apple_pmc *pmc = data;
	struct apple_pmc_domain *dom, *tmp;
	unsigned long flags;
	unsigned int rail;

	of_platform_depopulate(pmc->dev);
	debugfs_remove_recursive(pmc->debugfs);
	list_for_each_entry_safe_reverse(dom, tmp, &pmc->domains, list) {
		list_del(&dom->list);
		apple_pmc_domain_free(dom);
	}

	/* The cooling device is gone by now; drop any limit it left. */
	cancel_delayed_work_sync(&pmc->dcs_work);
	raw_spin_lock_irqsave(&pmc->lock, flags);
	for (rail = 0; rail < pmc->hw->rails; rail++)
		writel(pmc->saved_enable[rail],
		       pmc->rails + pmc->hw->rail_enable + 4 * rail);
	pmc->dcs_state = 0;
	apple_pmc_dcs_vote(pmc);
	raw_spin_unlock_irqrestore(&pmc->lock, flags);
}

static int apple_pmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	u32 agents[APPLE_PMC_MAX_AGENTS];
	struct device_node *child;
	struct apple_pmc *pmc;
	unsigned int rail;
	int count, i, ret;

	pmc = devm_kzalloc(dev, sizeof(*pmc), GFP_KERNEL);
	if (!pmc)
		return -ENOMEM;

	pmc->dev = dev;
	pmc->hw = of_device_get_match_data(dev);
	raw_spin_lock_init(&pmc->lock);
	INIT_LIST_HEAD(&pmc->domains);

	if (pmc->hw->rails > APPLE_PMC_MAX_RAILS ||
	    pmc->hw->agents > APPLE_PMC_MAX_AGENTS ||
	    pmc->hw->dcs_max >= APPLE_PMC_MAX_STATES)
		return -EINVAL;
	INIT_DELAYED_WORK(&pmc->dcs_work, apple_pmc_dcs_work);

	pmc->voters = devm_platform_ioremap_resource_byname(pdev, "voters");
	if (IS_ERR(pmc->voters))
		return PTR_ERR(pmc->voters);
	pmc->rails = devm_platform_ioremap_resource_byname(pdev, "rails");
	if (IS_ERR(pmc->rails))
		return PTR_ERR(pmc->rails);

	/* An unpowered PMC reads all-ones; do not write to it. */
	for (rail = 0; rail < pmc->hw->rails; rail++) {
		pmc->saved_enable[rail] = apple_pmc_enable_read(pmc, rail);
		if (pmc->saved_enable[rail] == ~0U)
			return dev_err_probe(dev, -ENODEV,
					     "PMC is not responding\n");
		pmc->enable[rail] = pmc->saved_enable[rail];
	}
	pmc->ap_dcs_vote = apple_pmc_ap_dcs_read(pmc);

	count = of_property_count_u32_elems(np, "apple,static-voters");
	if (count > 0) {
		if (count > ARRAY_SIZE(agents))
			return -EINVAL;
		ret = of_property_read_u32_array(np, "apple,static-voters",
						 agents, count);
		if (ret)
			return ret;
		for (i = 0; i < count; i++) {
			if (agents[i] >= pmc->hw->agents)
				return dev_err_probe(dev, -EINVAL,
						     "invalid static voter %u\n",
						     agents[i]);
			pmc->static_agents |= BIT(agents[i]);
		}
	}

	platform_set_drvdata(pdev, pmc);
	ret = devm_add_action_or_reset(dev, apple_pmc_release, pmc);
	if (ret)
		return ret;

	for (i = 0; i < pmc->hw->agents; i++) {
		if (!(pmc->static_agents & BIT(i)))
			continue;
		ret = apple_pmc_agent_enable(pmc, i, true);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to enable static voter %d\n",
					     i);
	}

	for_each_available_child_of_node(np, child) {
		if (!of_property_present(child, "#power-domain-cells"))
			continue;
		ret = apple_pmc_domain_add(pmc, child);
		if (ret) {
			of_node_put(child);
			return ret;
		}
	}

	ret = apple_pmc_dcs_init(pmc);
	if (ret)
		return ret;

	pmc->debugfs = debugfs_create_dir(dev_name(dev), apple_pmc_debugfs_root);
	debugfs_create_file("status", 0400, pmc->debugfs, pmc,
			    &apple_pmc_status_fops);

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to create child devices\n");

	return 0;
}

/*
 * T8132: DCS states 0-5 on rail 0, fabric states on rail 1. A floor write
 * sets the agent's pending bit until the PMC has taken it up.
 */
static const struct apple_pmc_hw apple_pmc_hw_t8132 = {
	.agent_stride = 0x4000,
	.agents = 4,
	.rails = 2,
	.agent_state = 0x0,
	.rail_enable = 0x2000,
	.dcs_rail = 0,
	.dcs_max = 5,
	.ap_agent = 3,
	.has_pending = true,
};

static const struct apple_pmc_hw apple_pmc_hw_t8140 = {
	.agent_stride = 0x4000,
	.agents = 4,
	.rails = 4,
	.agent_state = 0x0,
	.rail_enable = 0x2000,
	.dcs_rail = 0,
	.dcs_max = 4,
	.ap_agent = 3,
};

static const struct of_device_id apple_pmc_of_match[] = {
	{ .compatible = "apple,t8132-pmc", .data = &apple_pmc_hw_t8132 },
	{ .compatible = "apple,t8140-pmc", .data = &apple_pmc_hw_t8140 },
	{}
};
MODULE_DEVICE_TABLE(of, apple_pmc_of_match);

static struct platform_driver apple_pmc_driver = {
	.probe = apple_pmc_probe,
	.driver = {
		.name = "apple-pmc",
		.of_match_table = apple_pmc_of_match,
		/* Power-domain providers are not intended for manual unbinding. */
		.suppress_bind_attrs = true,
	},
};

static int __init apple_pmc_init(void)
{
	apple_pmc_debugfs_root = debugfs_create_dir("apple-pmc", NULL);
	return platform_driver_register(&apple_pmc_driver);
}
module_init(apple_pmc_init);

MODULE_DESCRIPTION("Apple SoC PMC voter interface driver");
MODULE_LICENSE("Dual MIT/GPL");

#if IS_ENABLED(CONFIG_APPLE_PMC_KUNIT_TEST)
#include "pmc-test.c"
#endif
