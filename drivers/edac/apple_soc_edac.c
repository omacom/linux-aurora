// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple SoC platform error reporting
 *
 * Copyright The Aurora Silicon Contributors
 */

#include <linux/bitops.h>
#include <linux/edac.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include "edac_module.h"

#define APPLE_SOC_EDAC_NAME	"apple_soc_edac"

/*
 * The sources' status registers have no established acknowledge contract,
 * so a line is masked when it fires and unmasked again after a backoff.
 * A line still asserted after APPLE_SOC_EDAC_MAX_RETRIES backoffs stays
 * masked; its count keeps the record.
 */
#define APPLE_SOC_EDAC_BACKOFF_MS	1000
#define APPLE_SOC_EDAC_BACKOFF_MAX_MS	60000
#define APPLE_SOC_EDAC_MAX_RETRIES	8

#define T6041_NI_REGS		25
#define T6041_NI_STATUS		0x416c

struct apple_soc_edac_range {
	u8 first;
	u8 count;
	const char *name;
};

/*
 * A fabric NI whose error status sits in its aperture: a block of status words,
 * one per error type, whose set bits name the port. They're acknowledged by
 * writing the set bits back.
 */
struct apple_soc_edac_ni {
	u8 index;	/* interrupt index */
	u8 reg;		/* reg index of the NI's aperture */
	u16 status;	/* offset of the first status word */
	u32 words;	/* bit n: a status word at status + 4 * n */
};

struct apple_soc_edac_hw {
	const struct apple_soc_edac_range *ranges;
	size_t nranges;
	const struct apple_soc_edac_ni *ni;
	size_t nni;
	unsigned int num_irqs;
	unsigned int num_status_regs;
	bool event_log;
};

struct apple_soc_edac;

struct apple_soc_edac_irq {
	struct apple_soc_edac *edac;
	const char *source;
	unsigned int index;
	int irq;
	const struct apple_soc_edac_ni *ni;
	void __iomem *ni_base;
	unsigned int retries;
	unsigned int backoff_ms;
	unsigned long armed_jiffies;
	struct delayed_work rearm;
};

struct apple_soc_edac {
	struct device *dev;
	struct edac_device_ctl_info *dci;
	const struct apple_soc_edac_hw *hw;
	struct apple_soc_edac_irq *irqs;
	void __iomem **status_regs;
};

/*
 * DCS IRQs drive a separate thermal loop and are intentionally omitted.
 * NSC/NSI errors are fatal on the reference implementation; no returning
 * acknowledge path is established for them, so leave those lines unclaimed.
 * The AMCC lines stay unclaimed too: they are asserted from boot on the
 * J616s, and a read of the AMCC status apertures from the AP resets it.
 */
static const struct apple_soc_edac_range t6041_error_ranges[] = {
	{ 0, 9, "sep" },
	{ 9, 1, "pmgr" },
	{ 26, 3, "sram" },
	{ 45, 3, "ni0" },
	{ 48, 4, "ni1" },
	{ 52, 4, "ni2" },
	{ 56, 8, "ni3" },
	{ 64, 2, "ni4" },
	{ 66, 2, "ni5" },
	{ 68, 2, "ni6" },
};

/*
 * T6000 follows the T6041 policy: the AMCC, DCS, NSC and NSI lines stay
 * unclaimed. PMGR has no interrupt line on this SoC.
 */
static const struct apple_soc_edac_range t6000_error_ranges[] = {
	{ 0, 8, "sep" },
	{ 12, 4, "ioa" },
	{ 32, 4, "ni0" },
	{ 36, 3, "ni1" },
	{ 39, 3, "ni2" },
	{ 42, 2, "ni3" },
	{ 44, 2, "ni4" },
	{ 46, 4, "ni5" },
	{ 50, 4, "ni6" },
	{ 54, 4, "ni7" },
	{ 58, 4, "ni8" },
	{ 72, 4, "nsr" },
};

/*
 * T8112: one AMCC aggregate line (0), two SEP lines (1-2) and sixteen DCS
 * lines (3-18, eight channels plus their calibration partners). AMCC and
 * DCS stay unclaimed, as on T6000 and T6041.
 */
static const struct apple_soc_edac_range t8112_error_ranges[] = {
	{ 1, 2, "sep" },
};

static const struct apple_soc_edac_range t8140_error_ranges[] = {
	{ 4, 1, "pmgr" },
	{ 5, 9, "sep" },
	{ 18, 1, "ni0" },
	{ 19, 1, "ni1" },
	{ 20, 1, "ni2" },
	{ 21, 1, "ni3" },
	{ 22, 1, "ni4" },
	{ 23, 1, "ni5" },
	{ 24, 1, "nsi" },
	{ 25, 1, "pio" },
};

/* From the ADT's niN-metadata and macOS's NI decoder tables */
static const struct apple_soc_edac_ni t8140_ni[] = {
	{ 18,  9, 0x1900, 0x1ffd7ff },	/* SoC NI 0 */
	{ 19, 10, 0x1b00, 0x1ffd7ff },	/* SoC NI 1 */
	{ 20, 11, 0x1500, 0x1ffd7ff },	/* SoC NI 2 */
	{ 21, 12, 0x2500, 0x1ffd7ff },	/* MCC NI */
	{ 22, 13, 0x2100, 0x7fefff },	/* AFC MCC NI */
	{ 23, 14, 0x1d00, 0x3ff7ff },	/* GFX NI */
};

static const struct apple_soc_edac_hw t6000_error_hw = {
	.ranges = t6000_error_ranges,
	.nranges = ARRAY_SIZE(t6000_error_ranges),
	.num_irqs = 76,
};

static const struct apple_soc_edac_hw t6041_error_hw = {
	.ranges = t6041_error_ranges,
	.nranges = ARRAY_SIZE(t6041_error_ranges),
	.num_irqs = 89,
	.num_status_regs = T6041_NI_REGS,
};

static const struct apple_soc_edac_hw t8112_error_hw = {
	.ranges = t8112_error_ranges,
	.nranges = ARRAY_SIZE(t8112_error_ranges),
	.num_irqs = 19,
};

static const struct apple_soc_edac_hw t8140_error_hw = {
	.ranges = t8140_error_ranges,
	.nranges = ARRAY_SIZE(t8140_error_ranges),
	.ni = t8140_ni,
	.nni = ARRAY_SIZE(t8140_ni),
	.num_irqs = 26,
};

static const struct apple_soc_edac_hw t6000_event_hw = {
	.num_irqs = 28,
	.event_log = true,
};

static const struct apple_soc_edac_hw t8112_event_hw = {
	.num_irqs = 16,
	.event_log = true,
};

static const struct apple_soc_edac_hw t6041_event_hw = {
	.num_irqs = 21,
	.event_log = true,
};

static const struct apple_soc_edac_hw t8140_event_hw = {
	.num_irqs = 9,
	.event_log = true,
};

static const char *apple_soc_edac_source(const struct apple_soc_edac_hw *hw,
					 unsigned int index)
{
	size_t i;

	if (hw->event_log)
		return "event-log";

	for (i = 0; i < hw->nranges; i++) {
		const struct apple_soc_edac_range *range = &hw->ranges[i];

		if (index >= range->first && index < range->first + range->count)
			return range->name;
	}

	return NULL;
}

/*
 * Read and acknowledge a T6041 NI status word. Returns whether it was set;
 * @error says whether a bit in the platform error policy was set and
 * @cleared whether the word reads back clear after the acknowledge.
 */
static bool apple_soc_edac_t6041_status(struct apple_soc_edac_irq *irq_data,
					bool *error, bool *cleared)
{
	struct apple_soc_edac *edac = irq_data->edac;
	u32 status, left;

	*error = false;
	*cleared = true;

	if (irq_data->index >= 45 && irq_data->index < 70) {
		void __iomem *reg = edac->status_regs[irq_data->index - 45] +
				    T6041_NI_STATUS;

		status = readl(reg);
		if (!status)
			return false;

		/* Bit 3 is not part of the platform error policy. */
		*error = status & 0xf7;
		writel(status, reg);
		left = readl(reg);
		if (left) {
			*cleared = false;
			dev_info_ratelimited(edac->dev,
					     "%s source %u: status %#x, %#x after the acknowledge\n",
					     irq_data->source, irq_data->index,
					     status, left);
		}
		return true;
	}

	return false;
}

/*
 * Log and acknowledge an NI's set status words. Returns whether any was set;
 * @cleared says whether all of them read back clear.
 */
static bool apple_soc_edac_ni_status(struct apple_soc_edac_irq *irq_data,
				     bool *cleared)
{
	const struct apple_soc_edac_ni *ni = irq_data->ni;
	unsigned long words = ni->words;
	unsigned int bit, offset;
	bool found = false;
	u32 status, left;

	*cleared = true;
	for_each_set_bit(bit, &words, 32) {
		offset = ni->status + 4 * bit;
		status = readl(irq_data->ni_base + offset);
		if (!status)
			continue;
		writel(status, irq_data->ni_base + offset);
		left = readl(irq_data->ni_base + offset);
		dev_err_ratelimited(irq_data->edac->dev,
				    "%s: status %#x at %#x, %#x after the acknowledge\n",
				    irq_data->source, status, offset, left);
		found = true;
		if (left)
			*cleared = false;
	}

	return found;
}

static void apple_soc_edac_rearm(struct work_struct *work)
{
	struct apple_soc_edac_irq *irq_data =
		container_of(to_delayed_work(work), struct apple_soc_edac_irq,
			     rearm);

	irq_data->armed_jiffies = jiffies;
	enable_irq(irq_data->irq);
}

static irqreturn_t apple_soc_edac_irq_thread(int irq, void *data)
{
	struct apple_soc_edac_irq *irq_data = data;
	struct apple_soc_edac *edac = irq_data->edac;
	bool error, cleared;
	char msg[64];

	snprintf(msg, sizeof(msg), "%s source %u", irq_data->source,
		 irq_data->index);

	if (edac->hw->num_status_regs &&
	    apple_soc_edac_t6041_status(irq_data, &error, &cleared)) {
		if (error) {
			edac_device_handle_ue(edac->dci, 0, 0, msg);
			dev_err_ratelimited(edac->dev,
					    "hardware error: %s (IRQ %d)\n",
					    msg, irq);
		}
		if (cleared)
			return IRQ_HANDLED;
		/*
		 * The acknowledge did not take, so the level line would
		 * fire again at once: mask it and back off instead.
		 */
	}

	if (irq_data->ni) {
		if (apple_soc_edac_ni_status(irq_data, &cleared)) {
			edac_device_handle_ue(edac->dci, 0, 0, msg);
			dev_err_ratelimited(edac->dev, "hardware error: %s (IRQ %d)\n",
					    msg, irq);
			if (cleared)
				return IRQ_HANDLED;
		} else {
			dev_err_ratelimited(edac->dev, "%s asserted with no status set\n",
					    msg);
		}
	}

	/* No status was pending, or this source has no clear contract. */
	disable_irq_nosync(irq);

	if (!irq_data->ni && (!edac->hw->num_status_regs || irq_data->index < 45)) {
		edac_device_handle_ue(edac->dci, 0, 0, msg);
		dev_err_ratelimited(edac->dev,
				    "hardware error: %s (IRQ %d)\n", msg, irq);
	}

	/*
	 * A line that stayed quiet for a minute after its last unmask had
	 * cleared: start the backoff again. One that fires straight after the
	 * unmask is still asserted and keeps counting towards the limit.
	 */
	if (time_after(jiffies, irq_data->armed_jiffies +
				msecs_to_jiffies(APPLE_SOC_EDAC_BACKOFF_MAX_MS))) {
		irq_data->retries = 0;
		irq_data->backoff_ms = APPLE_SOC_EDAC_BACKOFF_MS;
	}

	if (irq_data->retries++ >= APPLE_SOC_EDAC_MAX_RETRIES) {
		dev_info(edac->dev, "%s stays asserted, leaving IRQ %d masked\n",
			 msg, irq);
		return IRQ_HANDLED;
	}

	schedule_delayed_work(&irq_data->rearm,
			      msecs_to_jiffies(irq_data->backoff_ms));
	irq_data->backoff_ms = min(irq_data->backoff_ms * 2,
				   APPLE_SOC_EDAC_BACKOFF_MAX_MS);

	return IRQ_HANDLED;
}

static int apple_soc_edac_map_ni(struct platform_device *pdev,
				 struct apple_soc_edac_irq *irq_data)
{
	const struct apple_soc_edac_hw *hw = irq_data->edac->hw;
	size_t i;

	for (i = 0; i < hw->nni; i++) {
		if (hw->ni[i].index != irq_data->index)
			continue;
		irq_data->ni_base = devm_platform_ioremap_resource(pdev, hw->ni[i].reg);
		if (IS_ERR(irq_data->ni_base))
			return PTR_ERR(irq_data->ni_base);
		irq_data->ni = &hw->ni[i];
	}

	return 0;
}

static int apple_soc_edac_probe(struct platform_device *pdev)
{
	const struct apple_soc_edac_hw *hw;
	struct apple_soc_edac *edac;
	unsigned int requested = 0;
	unsigned int i;
	int ret;

	hw = device_get_match_data(&pdev->dev);
	if (!hw)
		return -EINVAL;

	if (platform_irq_count(pdev) != hw->num_irqs)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "expected %u interrupts\n", hw->num_irqs);

	edac = devm_kzalloc(&pdev->dev, sizeof(*edac), GFP_KERNEL);
	if (!edac)
		return -ENOMEM;

	edac->irqs = devm_kcalloc(&pdev->dev, hw->num_irqs,
				   sizeof(*edac->irqs), GFP_KERNEL);
	if (!edac->irqs)
		return -ENOMEM;

	edac->dev = &pdev->dev;
	edac->hw = hw;
	if (hw->num_status_regs) {
		edac->status_regs = devm_kcalloc(&pdev->dev, hw->num_status_regs,
						 sizeof(*edac->status_regs),
						 GFP_KERNEL);
		if (!edac->status_regs)
			return -ENOMEM;

		for (i = 0; i < hw->num_status_regs; i++) {
			edac->status_regs[i] = devm_platform_ioremap_resource(pdev, i);
			if (IS_ERR(edac->status_regs[i]))
				return PTR_ERR(edac->status_regs[i]);
		}
	}
	edac->dci = edac_device_alloc_ctl_info(0, (char *)dev_name(&pdev->dev), 1,
					       "platform", 1, 0,
					       edac_device_alloc_index());
	if (!edac->dci)
		return -ENOMEM;

	edac->dci->dev = &pdev->dev;
	edac->dci->mod_name = KBUILD_MODNAME;
	edac->dci->ctl_name = APPLE_SOC_EDAC_NAME;
	edac->dci->dev_name = dev_name(&pdev->dev);
	platform_set_drvdata(pdev, edac);

	ret = edac_device_add_device(edac->dci);
	if (ret)
		goto err_free_dci;

	for (i = 0; i < hw->num_irqs; i++) {
		struct apple_soc_edac_irq *irq_data = &edac->irqs[i];
		const char *source = apple_soc_edac_source(hw, i);
		int irq;

		if (!source)
			continue;

		irq = platform_get_irq(pdev, i);
		if (irq < 0) {
			ret = irq;
			goto err_del_device;
		}

		irq_data->edac = edac;
		irq_data->source = source;
		irq_data->index = i;
		irq_data->irq = irq;
		irq_data->backoff_ms = APPLE_SOC_EDAC_BACKOFF_MS;
		irq_data->armed_jiffies = jiffies;
		ret = apple_soc_edac_map_ni(pdev, irq_data);
		if (ret)
			goto err_del_device;
		INIT_DELAYED_WORK(&irq_data->rearm, apple_soc_edac_rearm);

		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
						 apple_soc_edac_irq_thread,
						 IRQF_ONESHOT | IRQF_NO_AUTOEN,
						 dev_name(&pdev->dev), irq_data);
		if (ret)
			goto err_del_device;

		requested++;
	}

	for (i = 0; i < hw->num_irqs; i++) {
		if (edac->irqs[i].source)
			enable_irq(edac->irqs[i].irq);
	}

	dev_info(&pdev->dev, "monitoring %u error sources\n", requested);

	return 0;

err_del_device:
	edac_device_del_device(&pdev->dev);
err_free_dci:
	edac_device_free_ctl_info(edac->dci);

	return ret;
}

static void apple_soc_edac_remove(struct platform_device *pdev)
{
	struct apple_soc_edac *edac = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < edac->hw->num_irqs; i++) {
		if (!edac->irqs[i].source)
			continue;
		/* A pending rearm would unbalance the disable below. */
		if (!cancel_delayed_work_sync(&edac->irqs[i].rearm))
			disable_irq(edac->irqs[i].irq);
	}

	edac_device_del_device(&pdev->dev);
	edac_device_free_ctl_info(edac->dci);
}

static const struct of_device_id apple_soc_edac_of_match[] = {
	{ .compatible = "apple,t6000-error-handler", .data = &t6000_error_hw },
	{ .compatible = "apple,t6041-error-handler", .data = &t6041_error_hw },
	{ .compatible = "apple,t8112-error-handler", .data = &t8112_error_hw },
	{ .compatible = "apple,t8140-error-handler", .data = &t8140_error_hw },
	{ .compatible = "apple,t6000-event-log", .data = &t6000_event_hw },
	{ .compatible = "apple,t6041-event-log", .data = &t6041_event_hw },
	{ .compatible = "apple,t8112-event-log", .data = &t8112_event_hw },
	{ .compatible = "apple,t8140-event-log", .data = &t8140_event_hw },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_soc_edac_of_match);

static struct platform_driver apple_soc_edac_driver = {
	.probe = apple_soc_edac_probe,
	.remove = apple_soc_edac_remove,
	.driver = {
		.name = APPLE_SOC_EDAC_NAME,
		.of_match_table = apple_soc_edac_of_match,
	},
};
module_platform_driver(apple_soc_edac_driver);

MODULE_AUTHOR("The Aurora Silicon Contributors");
MODULE_DESCRIPTION("Apple SoC platform error reporting driver");
MODULE_LICENSE("GPL");
