// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple T8152 PSD (power state director) service slot power domains
 *
 * On the M6 the USB blocks are not usable (even their DART registers hang the bus)
 * until two things have happened: the PMGR power states of the block are active, and the
 * PSD service slot for the port has been requested to run (state 0) and has
 * acknowledged. This driver models the second step as a genpd whose parents are the
 * PMGR domains: genpd powers the parents first, then this domain's power_on writes the
 * slot request and waits for the ack.
 *
 * Per-slot window (reg, 0x100 bytes at base + (slot << 8)):
 *   0x80 status  bits 2:0 = current PSD state
 *   0x90 request (write a PSD state)
 *   0xa0 diag
 *   0xbc valid   bit 0 = status is valid
 * PSD states: this driver accepts states 0 (running) to 6 (off).
 *
 * Ack: the valid bit 0 is set and (status & 7) equals the requested state.
 *
 * Slot groups: a block that needs several slots (the GPU uses several) must bring them up
 * together, in a fixed order, with possibly different active states. A node with compatible
 * "apple,t8152-psd-group" lists one reg window per slot and apple,psd-states with each slot's
 * active state; power_on requests the slots in reg order, power_off requests state 6 in reverse.
 *
 * Reading STATUS clears the acknowledge, so a request polls VALID first and reads
 * STATUS only once VALID is set.
 *
 * Lifetime: this domain sleeps in power_on (usleep_range polling), so it is not IRQ-safe.
 * The platform bus powers a consumer's single power domain on at attach time, before its
 * probe runs, which issues the request in process context. A consumer that marks its
 * runtime PM irq_safe (the Apple DART does) keeps this domain powered: genpd never powers off
 * a sleepable domain holding an irq-safe device and skips the (atomic-context) power_on on
 * its runtime resume (drivers/pmdomain/core.c irq_safe_dev_in_sleep_domain users). Other
 * consumers may let it power off to state 6.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>

#define APPLE_PSD_STATUS	0x80
#define APPLE_PSD_REQUEST	0x90
#define APPLE_PSD_DIAG		0xa0
#define APPLE_PSD_VALID		0xbc
#define APPLE_PSD_STATE		GENMASK(2, 0)
#define APPLE_PSD_VALID_BIT	BIT(0)

#define APPLE_PSD_STATE_RUNNING	0
#define APPLE_PSD_STATE_OFF	6

#define APPLE_PSD_DEFAULT_TIMEOUT_MS	1000	/* default ack timeout, apple,psd-timeout-ms overrides it */
#define APPLE_PSD_MAX_SLOTS		3

struct apple_psd {
	struct device *dev;
	void __iomem *regs[APPLE_PSD_MAX_SLOTS];
	u32 active[APPLE_PSD_MAX_SLOTS];
	unsigned int nslots;
	struct generic_pm_domain genpd;
	unsigned int timeout_ms;
};

#define to_apple_psd(d) container_of(d, struct apple_psd, genpd)

static int apple_psd_request(struct apple_psd *psd, unsigned int slot, u32 state)
{
	void __iomem *regs = psd->regs[slot];
	unsigned long timeout = jiffies + msecs_to_jiffies(psd->timeout_ms);
	u32 valid = 0, status = 0;

	writel(state, regs + APPLE_PSD_REQUEST);

	for (;;) {
		valid = readl(regs + APPLE_PSD_VALID);
		if (valid & APPLE_PSD_VALID_BIT) {
			/* Reading STATUS clears the acknowledge, so only after VALID. */
			status = readl(regs + APPLE_PSD_STATUS);
			if (FIELD_GET(APPLE_PSD_STATE, status) == state)
				return 0;
		}
		if (time_after(jiffies, timeout))
			break;
		usleep_range(50, 200);
	}

	dev_err(psd->dev,
		"PSD slot %u request %u not acknowledged: valid %#x status %#x diag %#x\n", slot,
		state, valid, status, readl(regs + APPLE_PSD_DIAG));
	return -ETIMEDOUT;
}

static int apple_psd_power_on(struct generic_pm_domain *genpd)
{
	struct apple_psd *psd = to_apple_psd(genpd);
	int i, ret;

	for (i = 0; i < psd->nslots; i++) {
		ret = apple_psd_request(psd, i, psd->active[i]);
		if (ret)
			goto undo;
	}
	return 0;

undo:
	/*
	 * Best effort: put the slots back to idle, newest first. The slot whose request timed
	 * out is included, because its acknowledge was not seen but it may still start.
	 */
	for (; i >= 0; i--)
		apple_psd_request(psd, i, APPLE_PSD_STATE_OFF);
	return ret;
}

static int apple_psd_power_off(struct generic_pm_domain *genpd)
{
	struct apple_psd *psd = to_apple_psd(genpd);
	int i, ret, first = 0;

	for (i = psd->nslots - 1; i >= 0; i--) {
		ret = apple_psd_request(psd, i, APPLE_PSD_STATE_OFF);
		if (ret && !first)
			first = ret;
	}
	if (!first)
		return 0;

	/*
	 * genpd keeps the domain marked on after a failed power-off, so the next resume does not
	 * call power_on. Put every slot back to its active state, best effort: the slots that
	 * did stop, and the one whose off request timed out and may still finish stopping.
	 */
	for (i = 0; i < psd->nslots; i++)
		apple_psd_request(psd, i, psd->active[i]);
	return first;
}

static int apple_psd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct apple_psd *psd;
	const char *name;
	u32 status, valid;
	unsigned int i;
	int ret;

	psd = devm_kzalloc(dev, sizeof(*psd), GFP_KERNEL);
	if (!psd)
		return -ENOMEM;
	psd->dev = dev;
	psd->timeout_ms = APPLE_PSD_DEFAULT_TIMEOUT_MS;
	of_property_read_u32(node, "apple,psd-timeout-ms", &psd->timeout_ms);

	for (i = 0; i < APPLE_PSD_MAX_SLOTS; i++) {
		struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, i);

		if (!res)
			break;
		psd->regs[i] = devm_ioremap_resource(dev, res);
		if (IS_ERR(psd->regs[i]))
			return PTR_ERR(psd->regs[i]);
		psd->nslots++;
	}
	if (!psd->nslots)
		return dev_err_probe(dev, -EINVAL, "no register window\n");

	/* Per-slot active state, in reg order; default 0 (running). */
	if (of_property_present(node, "apple,psd-states")) {
		ret = of_property_read_u32_array(node, "apple,psd-states", psd->active,
						 psd->nslots);
		if (ret)
			return dev_err_probe(dev, ret, "apple,psd-states needs one cell per reg\n");
		for (i = 0; i < psd->nslots; i++)
			if (psd->active[i] >= APPLE_PSD_STATE_OFF)
				return dev_err_probe(dev, -EINVAL, "bad PSD active state %u\n",
						     psd->active[i]);
	}

	ret = of_property_read_string(node, "label", &name);
	if (ret)
		name = node->name;
	psd->genpd.name = name;
	psd->genpd.power_on = apple_psd_power_on;
	psd->genpd.power_off = apple_psd_power_off;

	/*
	 * Always start OFF: an inherited status of state 0 without the valid bit is not an
	 * acknowledged P0, and the parents' state is not known here. The first consumer's
	 * power_on then issues the request (idempotent if the slot already runs P0).
	 * A status read is safe (proven on hardware); no write happens at probe.
	 */
	status = readl(psd->regs[0] + APPLE_PSD_STATUS);
	valid = readl(psd->regs[0] + APPLE_PSD_VALID);

	ret = pm_genpd_init(&psd->genpd, NULL, true);
	if (ret)
		return dev_err_probe(dev, ret, "pm_genpd_init failed\n");

	ret = of_genpd_add_provider_simple(node, &psd->genpd);
	if (ret) {
		dev_err(dev, "of_genpd_add_provider_simple failed: %d\n", ret);
		goto err_remove;
	}

	/* The PMGR parents are brought up before power_on runs. */
	for (i = 0; ; i++) {
		struct of_phandle_args parent, child;

		ret = of_parse_phandle_with_args(node, "power-domains", "#power-domain-cells", i,
						 &parent);
		if (ret == -ENOENT)
			break;	/* no (more) parents */
		if (ret)
			goto err_unlink;
		child.np = node;
		child.args_count = 0;
		ret = of_genpd_add_subdomain(&parent, &child);
		if (ret < 0 && ret != -EPROBE_DEFER)
			dev_err(dev, "failed to add parent domain %pOFn: %d\n", parent.np, ret);
		of_node_put(parent.np);
		if (ret < 0)
			goto err_unlink;
	}
	ret = 0;

	/* Parent domains are handled through the genpd hierarchy, not as a device. */
	pm_genpd_remove_device(dev);

	dev_info(dev, "PSD slot domain %s, %u slot(s) (status %#x valid %#x at probe, starting off)\n",
		 name, psd->nslots, status, valid);
	return 0;

err_unlink:
	/* pm_genpd_remove() drops any parent links added so far. */
	of_genpd_del_provider(node);
err_remove:
	pm_genpd_remove(&psd->genpd);
	return ret;
}

static const struct of_device_id apple_psd_of_match[] = {
	{ .compatible = "apple,t8152-psd" },
	{ .compatible = "apple,t8152-psd-group" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_psd_of_match);

static struct platform_driver apple_psd_driver = {
	.probe = apple_psd_probe,
	.driver = {
		.name = "apple-pmgr-psd",
		.of_match_table = apple_psd_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(apple_psd_driver);

MODULE_DESCRIPTION("Apple T8152 PSD service slot power domains");
MODULE_LICENSE("Dual MIT/GPL");
