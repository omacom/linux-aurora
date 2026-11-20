// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * The GPU's power state word across a system sleep. The domain is always on as far
 * as genpd knows, so nothing else lowers it. The word and its parent come from the
 * device tree.
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/soc/apple/dcp-v27-sleep.h>
#include <linux/soc/apple/mailbox.h>
#include <linux/soc/apple/pmp-report.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/sleep.h>

#include "sleep.h"

#define PS_AUTO_ENABLE		BIT(28)
#define PS_AUTO			GENMASK(27, 24)
#define PS_MIN			GENMASK(19, 16)
#define PS_BUSY			BIT(11)
#define PS_FLAGS		GENMASK(9, 8)
#define PS_ACTUAL		GENMASK(7, 4)
#define PS_TARGET		GENMASK(3, 0)
#define PS_ACTIVE		0xf
#define PS_POLL_US		100
#define PS_TIMEOUT_US		50000

/* In the "sgx" range, as regs.rs has them. */
#define SGX_GATING		0xd06030
#define SGX_GATING_HOLD		BIT(2)
#define SGX_BRIDGE		0x1000104
#define SGX_BRIDGE_WORDS	2
#define SGX_BRIDGE_ENABLE	BIT(0)

#define ASC_CPU_STATUS		0x4
/* Where the ASC's own registers hold the CPU's control: G16. */
#define ASC_CPU_CONTROL		0x44
#define ASC_CPU_CONTROL_RUN	BIT(4)

/* Only the M5 boards with a deep sleep need a wrapped provider translation here: the J714s
 * and the J716c. Other G17 clients retain their established direct-domain methods below.
 */
static bool asahi_g17_m5_provider(struct device *dev)
{
	return (of_machine_is_compatible("apple,j714s") ||
		of_machine_is_compatible("apple,j716c")) &&
	       of_machine_is_compatible("apple,t6050") &&
	       of_device_is_compatible(dev->of_node, "apple,agx-g17s");
}

static bool asahi_g17_m5_zero_cells(struct device_node *np)
{
	u32 cells;

	return of_device_is_available(np) &&
	       of_property_count_u32_elems(np, "#power-domain-cells") == 1 &&
	       !of_property_read_u32(np, "#power-domain-cells", &cells) && !cells;
}

static int asahi_g17_m5_dependency(struct device_node *np, struct device_node **out)
{
	struct of_phandle_args args;
	int ret;

	if (of_count_phandle_with_args(np, "power-domains", "#power-domain-cells") != 1)
		return -EINVAL;
	ret = of_parse_phandle_with_args(np, "power-domains", "#power-domain-cells", 0, &args);
	if (ret)
		return ret;
	if (args.args_count || !asahi_g17_m5_zero_cells(args.np)) {
		of_node_put(args.np);
		return -EINVAL;
	}
	*out = args.np;
	return 0;
}

/* Report reg is an entry identity. Its ONLY dependency supplies the power word. */
static int asahi_g17_m5_report_power(struct device_node *np, struct device_node **power)
{
	struct device_node *parent;
	u32 id;
	int ret = -EINVAL;

	if (!of_device_is_compatible(np, "apple,t6050-pmp-v2-report-entry") ||
	    !asahi_g17_m5_zero_cells(np) ||
	    of_property_count_u32_elems(np, "reg") != 1 ||
	    of_property_read_u32(np, "reg", &id) || id >= 64)
		return -EINVAL;
	parent = of_get_parent(np);
	if (parent && of_device_is_available(parent) &&
	    of_device_is_compatible(parent, "apple,t6050-pmp-v2-report"))
		ret = asahi_g17_m5_dependency(np, power);
	of_node_put(parent);
	return ret;
}

/* Referenced output nodes; metadata only, no request or power transition. */
static int asahi_g17_m5_nodes(struct device *dev, struct device_node **power,
			    struct device_node **request, u32 *offset)
{
	struct device_node *np = NULL, *report = NULL, *parent = NULL;
	struct device_node *explicit = NULL, *dep = NULL;
	struct resource res;
	u32 reg[2];
	int ret;

	*power = *request = NULL;
	ret = asahi_g17_m5_dependency(dev->of_node, &np);
	if (ret)
		return ret;
	if (of_device_is_compatible(np, "apple,t6050-pmp-v2-report-entry")) {
		report = np;
		np = NULL;
		ret = asahi_g17_m5_report_power(report, &np);
		if (ret)
			goto out;
	}
	ret = -EINVAL;
	if (!of_device_is_compatible(np, "apple,t6050-pmgr-pwrstate") ||
	    !asahi_g17_m5_zero_cells(np) ||
	    of_property_count_u32_elems(np, "reg") != 2 ||
	    of_property_read_u32_array(np, "reg", reg, 2) ||
	    reg[1] != sizeof(u32) || (reg[0] & 3))
		goto out;
	parent = of_get_parent(np);
	if (!parent || !of_device_is_available(parent) ||
	    !of_device_is_compatible(parent, "apple,t6050-pmgr") ||
	    !of_device_is_compatible(parent, "syscon") ||
	    of_address_to_resource(parent, 0, &res) || !(res.flags & IORESOURCE_MEM) ||
	    res.end < res.start || resource_size(&res) < sizeof(u32) ||
	    reg[0] > resource_size(&res) - sizeof(u32))
		goto out;
	if (of_property_present(dev->of_node, "apple,pmp-report-entry")) {
		if (of_property_count_u32_elems(dev->of_node, "apple,pmp-report-entry") != 1)
			goto out;
		explicit = of_parse_phandle(dev->of_node, "apple,pmp-report-entry", 0);
		if (!explicit || (report && explicit != report))
			goto out;
		ret = asahi_g17_m5_report_power(explicit, &dep);
		if (ret)
			goto out;
		ret = -EINVAL;
		if (dep != np)
			goto out;
		if (!report)
			report = of_node_get(explicit);
	}
	*offset = reg[0];
	*power = np;
	*request = report;
	np = report = NULL;
	ret = 0;
out:
	of_node_put(np);
	of_node_put(report);
	of_node_put(parent);
	of_node_put(explicit);
	of_node_put(dep);
	return ret;
}

/* The PMP's request for the GPU, where the device tree names one. */
int asahi_g17_pmp_request(struct device *dev, bool enable)
{
	struct device_node *np;
	int ret;

	if (asahi_g17_m5_provider(dev)) {
		struct device_node *power;
		u32 offset;

		ret = asahi_g17_m5_nodes(dev, &power, &np, &offset);
		if (ret)
			return ret;
		of_node_put(power);
	} else {
		np = of_parse_phandle(dev->of_node, "apple,pmp-report-entry", 0);
	}
	if (!np)
		return 0;
	ret = apple_pmp_report_entry_request(np, enable);
	of_node_put(np);

	return ret;
}

static int asahi_g17_ps(struct device *dev, struct regmap **regmap, u32 *offset)
{
	struct device_node *np;
	int ret;

	if (asahi_g17_m5_provider(dev)) {
		struct device_node *request, *parent;

		ret = asahi_g17_m5_nodes(dev, &np, &request, offset);
		if (ret)
			return ret;
		of_node_put(request);
		parent = of_get_parent(np);
		*regmap = parent ? syscon_node_to_regmap(parent) : ERR_PTR(-EINVAL);
		ret = PTR_ERR_OR_ZERO(*regmap);
		of_node_put(parent);
		of_node_put(np);
		return ret;
	}
	np = of_parse_phandle(dev->of_node, "power-domains", 0);
	if (!np)
		return -EINVAL;
	ret = of_property_read_u32(np, "reg", offset);
	if (!ret) {
		*regmap = np->parent ? syscon_node_to_regmap(np->parent) : ERR_PTR(-EINVAL);
		ret = PTR_ERR_OR_ZERO(*regmap);
	}
	of_node_put(np);

	return ret;
}

/* Automatic power management off. Returns the word as it was in @saved. */
int asahi_g17_power_auto_off(struct device *dev, u32 *saved)
{
	struct regmap *regmap;
	u32 offset;
	int ret;

	ret = asahi_g17_ps(dev, &regmap, &offset);
	if (ret)
		return ret;
	ret = regmap_read(regmap, offset, saved);
	if (ret)
		return ret;

	return regmap_write(regmap, offset, *saved & ~(PS_AUTO_ENABLE | PS_BUSY | PS_FLAGS));
}

/* The target off, then wait for the domain to finish going off. */
int asahi_g17_power_lower(struct device *dev)
{
	struct regmap *regmap;
	u32 offset, val, cur;
	int ret;

	ret = asahi_g17_ps(dev, &regmap, &offset);
	if (ret)
		return ret;
	ret = regmap_read(regmap, offset, &val);
	if (ret)
		return ret;

	regmap_write(regmap, offset,
		     (val & ~(PS_AUTO_ENABLE | PS_BUSY | PS_FLAGS | PS_TARGET)) | PS_FLAGS);

	ret = regmap_read_poll_timeout(regmap, offset, cur,
				       !FIELD_GET(PS_ACTUAL, cur) && !(cur & PS_BUSY),
				       PS_POLL_US, PS_TIMEOUT_US);
	dev_dbg(dev, "GPU domain %#x, now %#x\n", val, cur);

	return ret;
}

/*
 * The automatic state to raise with: apple,sleep-ps-auto if the DT has it, else the saved one.
 *
 * The floor (bits 19:16) as well, from apple,sleep-ps-min if the DT has it. The sleep
 * controller's wake replay writes the GFX word whole, with a zero floor, and the raise keeps
 * whatever floor it finds. The boot's word has floor 4, and the domain idles at state 4
 * between jobs. With floor 0 it goes fully off between jobs after every wake instead, and a
 * long-lived client's compute kick then occasionally never starts. Without the property the
 * floor stays as found.
 */
static u32 asahi_g17_ps_on(struct device *dev, u32 val, u32 saved)
{
	u32 ps_auto, ps_min;

	val &= ~(PS_AUTO_ENABLE | PS_AUTO | PS_BUSY | PS_FLAGS | PS_ACTUAL | PS_TARGET);
	if (!of_property_read_u32(dev->of_node, "apple,sleep-ps-min", &ps_min))
		val = (val & ~PS_MIN) | FIELD_PREP(PS_MIN, ps_min & 0xf);

	if (!of_property_read_u32(dev->of_node, "apple,sleep-ps-auto", &ps_auto))
		return val | FIELD_PREP(PS_AUTO, ps_auto) | FIELD_PREP(PS_TARGET, PS_ACTIVE);
	return val | (saved & PS_AUTO) | FIELD_PREP(PS_TARGET, PS_ACTIVE);
}

/* The target on, and wait for it. */
int asahi_g17_power_raise(struct device *dev, u32 saved)
{
	struct regmap *regmap;
	u32 offset, val, cur;
	int ret;

	ret = asahi_g17_ps(dev, &regmap, &offset);
	if (ret)
		return ret;
	ret = regmap_read(regmap, offset, &val);
	if (ret)
		return ret;
	dev_dbg(dev, "GPU domain before raising it: %#x\n", val);

	regmap_write(regmap, offset, asahi_g17_ps_on(dev, val, saved));

	return regmap_read_poll_timeout(regmap, offset, cur,
					FIELD_GET(PS_ACTUAL, cur) == PS_ACTIVE,
					PS_POLL_US, PS_TIMEOUT_US);
}

/* Automatic power management as it was. */
int asahi_g17_power_auto_on(struct device *dev, u32 saved)
{
	struct regmap *regmap;
	u32 offset, val;
	int ret;

	ret = asahi_g17_ps(dev, &regmap, &offset);
	if (ret)
		return ret;
	ret = regmap_read(regmap, offset, &val);
	if (ret)
		return ret;

	ret = regmap_write(regmap, offset,
			   asahi_g17_ps_on(dev, val, saved) | (saved & PS_AUTO_ENABLE));
	/* The word as the wake leaves it, its floor included (bits 19:16). */
	if (!ret && !regmap_read(regmap, offset, &val))
		dev_info(dev, "GPU domain after the wake: %#x (floor %lu, saved %#x)\n", val,
			 FIELD_GET(PS_MIN, val), saved);
	return ret;
}

/*
 * What the sleep's commit needs to stop the GPU with interrupts off, taken in .prepare, and
 * what it saw, for the log after the sleep.
 */
struct asahi_g17_commit {
	struct device *dev;
	unsigned int count;
	struct apple_rtkit *rtk[2];
	struct apple_mbox *mbox[2];
	void __iomem *cpu[2];
	void __iomem *gating;
	void __iomem *bridge;
	void __iomem *ps;
	struct device_node *request;

	bool ran, request_owned;
	int idle[2];
	unsigned int idle_us[2];
	u32 status[2];
	u32 words[SGX_BRIDGE_WORDS + 1];
	int request_ret;
	u32 saved;
	u32 lowered;
	int lower_ret;
	/* The domain at the start of the stop, and the raise to full power before any ASC access */
	u32 entry;
	int raise_ret;
	/* Whether the stop raises the domain first (asahi_g17_commit_raises()) */
	bool raise;
};

/*
 * Whether the commit's stop raises the GPU's domain before it touches the coprocessor: on the
 * SoCs the raise was made for, the T8142 (J815: the GPU gates itself between frames and its
 * registers may not answer below full power) and the T6050. Elsewhere the stop goes straight
 * to the idle poll, as before the raise. On the T8140 (J700) the domain reads its target on
 * and nothing powered under automatic power at the commit (0x1000230f); a raise from there
 * times out with BUSY latched, the PMP's AP IDLE then goes unanswered, the sleep takes the
 * deep path and the PMP and GPU firmware don't come back from it.
 */
static bool asahi_g17_commit_raises(struct device *dev)
{
	return of_device_is_compatible(dev->of_node, "apple,agx-t8142") ||
	       of_device_is_compatible(dev->of_node, "apple,agx-g17s");
}

/*
 * down() at the commit, every refusal behind it: both instances idled by polling, mailboxes
 * and run bits off, the gating hold, the PMP request withdrawn while the PMP still runs, then
 * the bridge and the domain. Told to idle, the firmware can't carry on, so the sleep goes on
 * whatever it answers.
 */
static int asahi_g17_commit_stop(void *data)
{
	struct asahi_g17_commit *c = data;
	unsigned int i;
	u32 val;

	c->ran = true;
	apple_sleep_note_sub(1);
	/*
	 * The GPU gates itself between frames (~95 times a second on J815), and at the commit its
	 * domain read 0x244 (actual 4, J815 boot 12505746): its coprocessor's and mailbox's
	 * registers may not answer there, and with interrupts off a stalled read ends at WD2 (J815
	 * boot 3c93d5a2: PMU note 0x44, the first commit stop). Bring it to full power first, as
	 * asahi_g17_power_raise() does, and keep the word from before for the resume. Only where
	 * asahi_g17_commit_raises() says so.
	 */
	if (c->raise) {
		c->entry = readl(c->ps);
		c->saved = c->entry;
		if (FIELD_GET(PS_ACTUAL, c->entry) != PS_ACTIVE) {
			writel(asahi_g17_ps_on(c->dev, c->entry, c->entry), c->ps);
			c->raise_ret = readl_poll_timeout_atomic(c->ps, val,
								 FIELD_GET(PS_ACTUAL, val) ==
								 PS_ACTIVE, 10, PS_TIMEOUT_US);
		}
	}
	apple_sleep_note_sub(2);
	if (c->raise_ret) {
		/* Its registers can't be trusted to answer: lower the domain only. */
		c->idle[0] = c->raise_ret;
		goto lower;
	}
	for (i = 0; i < c->count; i++)
		c->idle[i] = apple_rtkit_sleep_idle_polled(c->rtk[i], c->cpu[i], &c->idle_us[i]);
	apple_sleep_note_sub(3);
	for (i = 0; i < c->count; i++) {
		c->status[i] = readl(c->cpu[i] + ASC_CPU_STATUS);
		apple_rtkit_mailbox_recv_off(c->rtk[i]);
		if (c->mbox[i])
			apple_mbox_ascwrap_v6_stop_cpu_atomic(c->mbox[i]);
		else
			writel(readl(c->cpu[i]) & ~ASC_CPU_CONTROL_RUN, c->cpu[i]);
		if (!i && c->bridge)
			writel(readl(c->gating) | SGX_GATING_HOLD, c->gating);
	}
	apple_sleep_note_sub(4);
	c->request_ret = c->request ? apple_pmp_report_entry_withdraw_polled(c->request) : 0;
	apple_sleep_note_sub(5);

	/*
	 * Off automatic management, for the bridge (still at full power after a raise). Without
	 * the raise, the word for the resume is the one found here, after the idle.
	 */
	val = readl(c->ps);
	if (!c->raise)
		c->saved = val;
	writel(val & ~(PS_AUTO_ENABLE | PS_BUSY | PS_FLAGS), c->ps);
	if (c->bridge) {
		for (i = 0; i < SGX_BRIDGE_WORDS; i++)
			c->words[i] = readl(c->bridge + 4 * i);
		c->words[i] = readl(c->gating);
		for (i = 0; i < SGX_BRIDGE_WORDS; i++)
			writel(c->words[i] & ~SGX_BRIDGE_ENABLE, c->bridge + 4 * i);
	}
	apple_sleep_note_sub(6);
lower:
	val = readl(c->ps);
	writel((val & ~(PS_AUTO_ENABLE | PS_BUSY | PS_FLAGS | PS_TARGET)) | PS_FLAGS, c->ps);
	c->lower_ret = readl_poll_timeout_atomic(c->ps, c->lowered,
						 !FIELD_GET(PS_ACTUAL, c->lowered) &&
						 !(c->lowered & PS_BUSY),
						 10, PS_TIMEOUT_US);
	apple_sleep_note_sub(7);

	return (c->idle[0] || c->idle[1] || c->request_ret || c->lower_ret) ? 1 : 0;
}

/*
 * The M5 Pro's stop, made in the sleep's last steps, before the commit, and nothing else. GFX
 * then GFX1, each whole: AP IDLE, IOP IDLE and its ack, receiving off, the CPU's run bit off.
 * Then GFX's power word down. No PMP request, gating or bridge write.
 *
 * After a first sleep the domain runs under automatic power as it does under macOS: between
 * jobs its word has its target on, automatic power enabled and nothing powered, where the
 * boot's word never reads that low. That is an idle GPU whose firmware still runs, not a stopped
 * one: a stop that sends it no IDLE leaves firmware the wake cannot restart. macOS stops a
 * domain that is up, so the domain is taken out of automatic power and waited on first, and
 * the stop is the same from there.
 */
static int asahi_g17_m5_commit_stop(void *data)
{
	struct asahi_g17_commit *c = data;
	unsigned int i;
	u32 val, automatic;

	c->ran = true;
	/*
	 * With its domain off the first mailbox read here takes a synchronous external abort.
	 * With GFX off there is nothing to idle: report it and only lower the word.
	 */
	c->saved = readl(c->ps);
	dev_info(c->dev, "commit stop: GFX %#x\n", c->saved);
	automatic = c->saved & PS_AUTO_ENABLE;
	if (!FIELD_GET(PS_ACTUAL, c->saved) && automatic && FIELD_GET(PS_TARGET, c->saved)) {
		int ret;

		writel((c->saved & ~(PS_AUTO_ENABLE | PS_AUTO | PS_BUSY | PS_FLAGS | PS_ACTUAL |
				     PS_TARGET)) |
		       FIELD_PREP(PS_AUTO, PS_ACTIVE) | FIELD_PREP(PS_TARGET, PS_ACTIVE), c->ps);
		ret = readl_poll_timeout_atomic(c->ps, val,
						FIELD_GET(PS_ACTUAL, val) == PS_ACTIVE,
						10, PS_TIMEOUT_US);
		dev_info(c->dev, "commit stop: GFX idle under automatic power, raised for the stop: %#x (%d)\n",
			 val, ret);
		c->saved = val;
	}
	if (FIELD_GET(PS_ACTUAL, c->saved) != PS_ACTIVE &&
	    (!FIELD_GET(PS_ACTUAL, c->saved) || !automatic)) {
		/* Off, or not raised: its mailboxes can't be read. The wake will not restart it. */
		dev_warn(c->dev, "commit stop: GFX off (%#x), no idle sent: the GPU's firmware is not stopped for this sleep\n",
			 c->saved);
		c->idle[0] = c->idle[1] = -ENODEV;
	} else {
		for (i = 0; i < c->count; i++) {
			c->idle[i] = apple_rtkit_sleep_idle_polled(c->rtk[i], c->cpu[i],
								   &c->idle_us[i]);
			c->status[i] = readl(c->cpu[i] + ASC_CPU_STATUS);
			apple_rtkit_mailbox_recv_off(c->rtk[i]);
			apple_mbox_ascwrap_v6_stop_cpu_atomic(c->mbox[i]);
		}
		c->saved = readl(c->ps);
	}
	/* The wake puts automatic power back as the stop found it. */
	c->saved = (c->saved & ~PS_AUTO_ENABLE) | automatic;
	/*
	 * The GPU's PMP request (AGX, PTD request bit 15) goes before GFX does, as in the Neo's stop
	 * above: before the sleep the only PTD request left is ANS's. Left on here, the PMP's
	 * firmware crashes before its own stop (AP IDLE never echoed, a crashlog message instead).
	 */
	c->request_ret = c->request ? apple_pmp_report_entry_withdraw_polled(c->request) : 0;
	dev_info(c->dev, "commit stop: PMP request withdrawn: %d\n", c->request_ret);
	writel(c->saved & ~(PS_AUTO_ENABLE | PS_BUSY | PS_FLAGS), c->ps);
	val = readl(c->ps);
	writel((val & ~(PS_AUTO_ENABLE | PS_BUSY | PS_FLAGS | PS_TARGET)) | PS_FLAGS, c->ps);
	c->lower_ret = readl_poll_timeout_atomic(c->ps, c->lowered,
						 !FIELD_GET(PS_ACTUAL, c->lowered) &&
						 !(c->lowered & PS_BUSY),
						 10, PS_TIMEOUT_US);

	return (c->idle[0] || c->idle[1] || c->request_ret || c->lower_ret) ? 1 : 0;
}

static void __iomem *asahi_g17_map(struct device_node *np, const char *name, int index,
				   u32 offset, u32 size)
{
	struct resource res;

	if (name)
		index = of_property_match_string(np, "reg-names", name);
	if (index < 0 || of_address_to_resource(np, index, &res) ||
	    offset + size > resource_size(&res))
		return NULL;

	return ioremap(res.start + offset, size);
}

static void asahi_g17_commit_free(struct asahi_g17_commit *c)
{
	if (c->request) {
		if (c->request_owned)
			apple_pmp_report_entry_commit_owned(c->request, false);
		of_node_put(c->request);
	}
	/* The mapping is only this structure's on G16: G17's belongs to its mailbox. */
	if (!c->mbox[0] && c->cpu[0])
		iounmap(c->cpu[0]);
	if (c->ps)
		iounmap(c->ps);
	if (c->bridge)
		iounmap(c->bridge);
	if (c->gating)
		iounmap(c->gating);
	kfree(c);
}

/*
 * Leaves the GPU's whole stop to the sleep's commit: maps what the stop touches, keeps the
 * PMP request for it, and registers it with the sleep driver, ready. Nothing is stopped
 * here. @secondary_rtk is NULL on a single instance, and @bridge false on a target that runs
 * without the bridge.
 */
struct asahi_g17_commit *asahi_g17_commit_prepare(struct device *dev,
						  struct apple_rtkit *primary_rtk,
						  struct apple_mbox *primary_mbox,
						  struct apple_rtkit *secondary_rtk,
						  struct apple_mbox *secondary_mbox, bool bridge)
{
	struct asahi_g17_commit *c;
	struct device_node *np;
	unsigned int i;
	u32 offset;
	int ret;

	c = kzalloc_obj(*c);
	if (!c)
		return ERR_PTR(-ENOMEM);
	c->dev = dev;
	c->rtk[0] = primary_rtk;
	c->mbox[0] = primary_mbox;
	c->rtk[1] = secondary_rtk;
	c->mbox[1] = secondary_mbox;
	c->count = secondary_rtk ? 2 : 1;
	c->raise = asahi_g17_commit_raises(dev);

	ret = -EOPNOTSUPP;
	for (i = 0; i < c->count; i++) {
		c->cpu[i] = apple_mbox_ascwrap_v6_cpu_control(c->mbox[i]);
		if (!c->rtk[i] || !c->cpu[i])
			goto err;
	}

	ret = -ENODEV;
	if (asahi_g17_m5_provider(dev)) {
		struct device_node *parent;

		ret = asahi_g17_m5_nodes(dev, &np, &c->request, &offset);
		if (ret)
			goto err;
		/* A failed mapping must remain an error, not ERR_PTR(0). */
		ret = -ENODEV;
		parent = of_get_parent(np);
		if (parent)
			c->ps = asahi_g17_map(parent, NULL, 0, offset, 4);
		of_node_put(parent);
		of_node_put(np);
	} else {
		np = of_parse_phandle(dev->of_node, "power-domains", 0);
		if (!np)
			goto err;
		if (!of_property_read_u32(np, "reg", &offset) && np->parent)
			c->ps = asahi_g17_map(np->parent, NULL, 0, offset, 4);
		of_node_put(np);
	}
	c->gating = asahi_g17_map(dev->of_node, "sgx", 0, SGX_GATING, 4);
	if (bridge)
		c->bridge = asahi_g17_map(dev->of_node, "sgx", 0, SGX_BRIDGE,
					  4 * SGX_BRIDGE_WORDS);
	if (!c->ps || !c->gating || (bridge && !c->bridge))
		goto err;

	if (!asahi_g17_m5_provider(dev))
		c->request = of_parse_phandle(dev->of_node, "apple,pmp-report-entry", 0);
	if (c->request) {
		ret = apple_pmp_report_entry_commit_owned(c->request, true);
		if (ret && asahi_g17_m5_provider(dev))
			goto err;
		/* Legacy release semantics are retained, even for its ignored error. */
		c->request_owned = true;
	}

	ret = apple_sleep_coproc_register_first(dev, asahi_g17_m5_provider(dev) ?
						asahi_g17_m5_commit_stop : asahi_g17_commit_stop, c);
	if (ret)
		goto err;
	apple_sleep_coproc_set_ready(dev, true);

	return c;

err:
	asahi_g17_commit_free(c);
	return ERR_PTR(ret);
}

/* The M5 Pro: its GPU firmware is stopped by the sleep's last steps, not by .suspend. */
bool asahi_g17_commit_stops_firmware(void)
{
	return apple_sleep_v27_late();
}

/* Whether only the sleep's commit may stop the GPU: then nothing else may. */
bool asahi_g17_commit_required(void)
{
	return apple_sleep_commit_stops_coprocs();
}

/*
 * The same stop for G16: one firmware instance, whose control registers are the ASC's own, no
 * bridge and no PMP request to withdraw, as its vote stays through the sleep.
 */
struct asahi_g17_commit *asahi_g16_commit_prepare(struct device *dev, struct apple_rtkit *rtk)
{
	struct asahi_g17_commit *c;
	struct device_node *np;
	u32 offset;
	int ret;

	c = kzalloc_obj(*c);
	if (!c)
		return ERR_PTR(-ENOMEM);
	c->dev = dev;
	c->rtk[0] = rtk;
	c->count = 1;
	c->raise = asahi_g17_commit_raises(dev);

	ret = -ENODEV;
	c->cpu[0] = asahi_g17_map(dev->of_node, "asc", 0, ASC_CPU_CONTROL, 8);
	np = of_parse_phandle(dev->of_node, "power-domains", 0);
	if (!np)
		goto err;
	if (!of_property_read_u32(np, "reg", &offset) && np->parent)
		c->ps = asahi_g17_map(np->parent, NULL, 0, offset, 4);
	of_node_put(np);
	if (!c->cpu[0] || !c->ps)
		goto err;

	ret = apple_sleep_coproc_register_first(dev, asahi_g17_commit_stop, c);
	if (ret)
		goto err;
	apple_sleep_coproc_set_ready(dev, true);

	return c;

err:
	asahi_g17_commit_free(c);
	return ERR_PTR(ret);
}

/*
 * After the sleep, refused or not: whether the commit stopped the GPU, and in @saved the
 * domain's word from before, if it did. Frees @c.
 */
bool asahi_g17_commit_finish(struct asahi_g17_commit *c, u32 *saved)
{
	bool stopped = apple_sleep_coproc_stopped(c->dev) || c->ran;

	apple_sleep_coproc_unregister(c->dev);
	if (c->ran) {
		if (c->idle[0] || c->idle[1] || c->request_ret || c->lower_ret)
			dev_warn(c->dev, "GPU stop at the commit: idle %d %d, PMP request %d, domain %#x (%d)\n",
				 c->idle[0], c->idle[1], c->request_ret, c->lowered, c->lower_ret);
		dev_dbg(c->dev, "GPU stopped at the commit: idle in %u %u us, status %#x %#x\n",
			c->idle_us[0], c->idle_us[1], c->status[0], c->status[1]);
		dev_dbg(c->dev, "GPU at the commit: bridge %#x %#x gating %#x, domain %#x now %#x\n",
			c->words[0], c->words[1], c->words[2], c->saved, c->lowered);
		if (c->raise)
			dev_info(c->dev, "GPU domain at the commit stop: %#x, raise to full %d\n",
				 c->entry, c->raise_ret);
		*saved = c->saved;
	}
	asahi_g17_commit_free(c);

	return stopped;
}
