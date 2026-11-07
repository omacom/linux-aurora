// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple Generic RTKit helper coprocessor
 * Copyright The Asahi Linux Contributors
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/string.h>

#define APPLE_ASC_CPU_CONTROL		0x44
#define APPLE_ASC_CPU_CONTROL_RUN	BIT(4)

/* A running T8140 MTP session described by the boot stage that started it. */
#define APPLE_RTKIT_HELPER_HANDOFF	"mtp-live-v1"

struct apple_rtkit_helper {
	struct device *dev;
	struct apple_rtkit *rtk;
	/* Inherited from the boot stage; it cannot be restarted. */
	bool adopted;

	void __iomem *asc_base;

	struct resource *sram;
	void __iomem *sram_base;
};

static int apple_rtkit_helper_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_rtkit_helper *helper = cookie;
	struct resource res = {
		.start = bfr->iova,
		.end = bfr->iova + bfr->size - 1,
		.name = "rtkit_map",
	};

	if (!bfr->iova) {
		bfr->buffer = dma_alloc_coherent(helper->dev, bfr->size,
						    &bfr->iova, GFP_KERNEL);
		if (!bfr->buffer)
			return -ENOMEM;
		return 0;
	}

	if (!helper->sram) {
		dev_err(helper->dev,
			"RTKit buffer request with no SRAM region: %pR\n", &res);
		return -EFAULT;
	}

	res.flags = helper->sram->flags;

	if (res.end < res.start || !resource_contains(helper->sram, &res)) {
		dev_err(helper->dev,
			"RTKit buffer request outside SRAM region: %pR\n", &res);
		return -EFAULT;
	}

	bfr->iomem = helper->sram_base + (res.start - helper->sram->start);
	bfr->is_mapped = true;

	return 0;
}

static void apple_rtkit_helper_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	// no-op
}

static const struct apple_rtkit_ops apple_rtkit_helper_ops = {
	.shmem_setup = apple_rtkit_helper_shmem_setup,
	.shmem_destroy = apple_rtkit_helper_shmem_destroy,
};

static void apple_rtkit_helper_release(void *rtk)
{
	apple_rtkit_free_retaining_buffers(rtk);
}

/*
 * T8140 MTP firmware cannot be restarted once it has been taken to QUIESCED
 * or SLEEP, so a boot stage that used it hands the session over running. It
 * reserves the buffers the firmware holds and maps them 1:1 through the
 * DART. Only adopt a session it described whose CPU still runs.
 */
static bool apple_rtkit_helper_live(struct apple_rtkit_helper *helper)
{
	const char *handoff;

	if (of_property_read_string(helper->dev->of_node, "apple,rtkit-handoff",
				    &handoff))
		return false;

	if (strcmp(handoff, APPLE_RTKIT_HELPER_HANDOFF)) {
		dev_warn(helper->dev, "unknown RTKit handoff \"%s\"\n", handoff);
		return false;
	}

	if (!(readl_relaxed(helper->asc_base + APPLE_ASC_CPU_CONTROL) &
	      APPLE_ASC_CPU_CONTROL_RUN)) {
		dev_warn(helper->dev, "RTKit handoff without a running CPU\n");
		return false;
	}

	return true;
}

static int apple_rtkit_helper_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_rtkit_helper *helper;
	int ret;

	/* 44 bits for addresses in standard RTKit requests */
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(44));
	if (ret)
		return ret;

	helper = devm_kzalloc(dev, sizeof(*helper), GFP_KERNEL);
	if (!helper)
		return -ENOMEM;

	helper->dev = dev;
	platform_set_drvdata(pdev, helper);

	helper->asc_base = devm_platform_ioremap_resource_byname(pdev, "asc");
	if (IS_ERR(helper->asc_base))
		return PTR_ERR(helper->asc_base);

	helper->sram = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (helper->sram) {
		helper->sram_base = devm_ioremap_resource(dev, helper->sram);
		if (IS_ERR(helper->sram_base))
			return dev_err_probe(dev, PTR_ERR(helper->sram_base),
					"Failed to map SRAM region\n");
	}

	if (apple_rtkit_helper_live(helper)) {
		helper->rtk = apple_rtkit_init_adopted(dev, helper, NULL, 0,
						       &apple_rtkit_helper_ops);
		if (IS_ERR(helper->rtk))
			return dev_err_probe(dev, PTR_ERR(helper->rtk),
					     "Failed to adopt RTKit\n");

		ret = devm_add_action_or_reset(dev, apple_rtkit_helper_release,
					       helper->rtk);
		if (ret)
			return ret;

		helper->adopted = true;
		dev_info(dev, "adopted the running RTKit session\n");
		return 0;
	}

	helper->rtk =
		devm_apple_rtkit_init(dev, helper, NULL, 0, &apple_rtkit_helper_ops);
	if (IS_ERR(helper->rtk))
		return dev_err_probe(dev, PTR_ERR(helper->rtk),
				     "Failed to initialize RTKit\n");

	writel_relaxed(APPLE_ASC_CPU_CONTROL_RUN,
		       helper->asc_base + APPLE_ASC_CPU_CONTROL);

	/* Works for both wake and boot */
	if (of_device_is_compatible(dev->of_node, "apple,t8142-mtp") ||
	    of_device_is_compatible(dev->of_node, "apple,t6050-mtp"))
		ret = apple_rtkit_wake_early_ap(helper->rtk);
	else
		ret = apple_rtkit_wake(helper->rtk);
	if (ret != 0)
		return dev_err_probe(dev, ret, "Failed to wake up coprocessor\n");

	return 0;
}

static void apple_rtkit_helper_remove(struct platform_device *pdev)
{
	struct apple_rtkit_helper *helper = platform_get_drvdata(pdev);

	/* An inherited session would not come back from QUIESCED. */
	if (helper->adopted)
		return;

	if (apple_rtkit_is_running(helper->rtk))
		apple_rtkit_quiesce(helper->rtk);

	writel_relaxed(0, helper->asc_base + APPLE_ASC_CPU_CONTROL);
}

static const struct of_device_id apple_rtkit_helper_of_match[] = {
	{ .compatible = "apple,rtk-helper-asc4" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_rtkit_helper_of_match);

static struct platform_driver apple_rtkit_helper_driver = {
	.driver = {
		.name = "rtkit-helper",
		.of_match_table = apple_rtkit_helper_of_match,
	},
	.probe = apple_rtkit_helper_probe,
	.remove = apple_rtkit_helper_remove,
};
module_platform_driver(apple_rtkit_helper_driver);

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple RTKit helper driver");
