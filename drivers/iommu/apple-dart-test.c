// SPDX-License-Identifier: GPL-2.0-only
/* Included by apple-dart.c to exercise domain and command lifecycles. */
#include <kunit/device.h>
#include <kunit/static_stub.h>
#include <kunit/test.h>
#include <linux/vmalloc.h>

struct apple_dart_test_window {
	const char *soc;
	u32 ias;
	u32 sid;
	u64 base;
	u64 size;
};

static void apple_dart_test_iommu_specifiers(struct kunit *test)
{
	static const struct apple_dart_test_window windows[] = {
		{ "t6000", 32, 0, 0, 0xfc000000 },
		{ "t6020", 42, 5, 0x10000000000, 0x1000000000 },
		{ "t8112", 36, 5, 0x800000000, 0x7ffff0000 },
	};
	struct device *dev;
	int i;

	dev = kunit_device_register(test, "apple-dart-window-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);

	for (i = 0; i < ARRAY_SIZE(windows); i++) {
		const struct apple_dart_test_window *window = &windows[i];
		struct of_phandle_args args = { .args_count = 1 };
		struct apple_dart *dart;
		dma_addr_t dma_min;
		dma_addr_t dma_max;
		int ret;

		dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, dart);
		dart->ias = window->ias;
		dart->pgsize = SZ_16K;
		dart->dma_min = window->base;
		dart->dma_max = window->base + window->size - 1;

		args.args[0] = window->sid;
		ret = apple_dart_parse_dma_window(dev, dart, &args,
						  &dma_min, &dma_max);
		KUNIT_ASSERT_EQ_MSG(test, ret, 0, "%s legacy specifier",
				    window->soc);
		KUNIT_EXPECT_EQ_MSG(test, dma_min, (dma_addr_t)window->base,
				    "%s legacy window start", window->soc);
		KUNIT_EXPECT_EQ_MSG(test, dma_max,
				    (dma_addr_t)(window->base + window->size - 1),
				    "%s legacy window end", window->soc);

		dart->dma_min = 0;
		dart->dma_max = DMA_BIT_MASK(dart->ias);
		args.args_count = 5;
		args.args[1] = upper_32_bits(window->base);
		args.args[2] = lower_32_bits(window->base);
		args.args[3] = upper_32_bits(window->size);
		args.args[4] = lower_32_bits(window->size);
		ret = apple_dart_parse_dma_window(dev, dart, &args,
						  &dma_min, &dma_max);
		KUNIT_ASSERT_EQ_MSG(test, ret, 0, "%s 5-cell specifier",
				    window->soc);
		KUNIT_EXPECT_EQ_MSG(test, dma_min, (dma_addr_t)window->base,
				    "%s 5-cell window start", window->soc);
		KUNIT_EXPECT_EQ_MSG(test, dma_max,
				    (dma_addr_t)(window->base + window->size - 1),
				    "%s 5-cell window end", window->soc);
	}
}

static void apple_dart_test_consumer_window(struct kunit *test)
{
	__be32 value[] = { cpu_to_be32(0x100), 0, cpu_to_be32(0x10), 0 };
	struct property property = {
		.name = "apple,dma-range",
		.length = sizeof(value),
		.value = value,
	};
	struct device_node node = { .properties = &property };
	struct of_phandle_args args = { .args_count = 1 };
	struct apple_dart *dart;
	struct device *dev;
	dma_addr_t dma_min, dma_max;
	int i, ret;

	dev = kunit_device_register(test, "apple-dart-consumer-window-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	dart->ias = 36;
	dart->pgsize = SZ_16K;
	dev->of_node = &node;

	/* J613's common scanout aperture must survive both DART defaults. */
	for (i = 0; i < 2; i++) {
		dart->dma_min = i ? 0 : 0x10000000000;
		dart->dma_max = i ? 0xffffeffff : 0x10fffffffff;
		ret = apple_dart_parse_dma_window(dev, dart, &args,
						  &dma_min, &dma_max);
		KUNIT_EXPECT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, dma_min, (dma_addr_t)0x10000000000);
		KUNIT_EXPECT_EQ(test, dma_max, (dma_addr_t)0x10fffffffff);
	}

	/* Domain allocation gives the consumer priority over five-cell args too. */
	args.args_count = 5;
	args.args[3] = 0xf;
	args.args[4] = 0xffff0000;
	ret = apple_dart_parse_dma_window(dev, dart, &args, &dma_min, &dma_max);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, dma_min, (dma_addr_t)0x10000000000);
	KUNIT_EXPECT_EQ(test, dma_max, (dma_addr_t)0x10fffffffff);

	/* An override must not make a malformed stream specifier acceptable. */
	args.args[3] = 0;
	args.args[4] = 0;
	KUNIT_EXPECT_EQ(test, apple_dart_parse_dma_window(dev, dart, &args,
							 &dma_min, &dma_max), -EINVAL);

	/* PIODMA is a different consumer of the same external display DART. */
	dev->of_node = NULL;
	args.args_count = 1;
	args.args[0] = 4;
	ret = apple_dart_parse_dma_window(dev, dart, &args, &dma_min, &dma_max);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, dma_min, (dma_addr_t)0);
	KUNIT_EXPECT_EQ(test, dma_max, (dma_addr_t)0xffffeffff);
}

static void apple_dart_test_invalid_consumer_window(struct kunit *test)
{
	static const struct {
		u64 base, size;
	} windows[] = {
		{ 0, 0 },
		{ U64_MAX - SZ_16K + 1, 2 * SZ_16K },
		{ 1, SZ_16K },
		{ 0, SZ_16K + 1 },
		{ 0, BIT_ULL(37) },
	};
	__be64 value[2];
	struct property property = {
		.name = "apple,dma-range",
		.length = sizeof(value),
		.value = value,
	};
	struct device_node node = { .properties = &property };
	struct of_phandle_args args = { .args_count = 1 };
	struct apple_dart *dart;
	struct device *dev;
	dma_addr_t dma_min, dma_max;
	int i;

	dev = kunit_device_register(test, "apple-dart-invalid-window-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	dart->ias = 36;
	dart->pgsize = SZ_16K;
	dev->of_node = &node;
	for (i = 0; i < ARRAY_SIZE(windows); i++) {
		value[0] = cpu_to_be64(windows[i].base);
		value[1] = cpu_to_be64(windows[i].size);
		KUNIT_EXPECT_EQ(test, apple_dart_parse_dma_window(dev, dart, &args,
							 &dma_min, &dma_max), -EINVAL);
	}
	property.length = sizeof(__be32);
	KUNIT_EXPECT_LT(test, apple_dart_parse_dma_window(dev, dart, &args,
							 &dma_min, &dma_max), 0);
	dev->of_node = NULL;
}

static void apple_dart_test_before_attach(struct kunit *test)
{
	struct apple_dart_master_cfg master = {};
	struct apple_dart_domain *domain;
	struct apple_dart *dart;
	struct io_pgtable_ops *ops;
	size_t mapped = 0;
	int ret;

	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	domain = kunit_kzalloc(test, sizeof(*domain), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, domain);
	dart->dev = kunit_device_register(test, "apple-dart-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dart->dev);
	dart->hw = &apple_dart_hw_t8103;
	dart->pgsize = PAGE_SIZE;
	dart->ias = 32;
	dart->oas = 36;
	dart->dma_max = DMA_BIT_MASK(32);
	dart->num_streams = 16;
	dart->locked = true;
	dart->regs = (__force void __iomem *)kunit_kzalloc(test, SZ_16K, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart->regs);
	spin_lock_init(&dart->lock);
	mutex_init(&domain->init_lock);
	master.stream_maps[0].dart = dart;
	__set_bit(0, master.stream_maps[0].sidmap);
	/* of_xlate() starts each master with the window of its DART. */
	master.dma_min = dart->dma_min;
	master.dma_max = dart->dma_max;
	writel(dart->hw->ttbr_valid, dart->regs + DART_TTBR(dart, 0, 0));
	KUNIT_ASSERT_EQ(test, apple_dart_finalize_domain(domain, &master), 0);
	ops = domain->pgtbl_ops;
	pm_runtime_set_active(dart->dev);
	pm_runtime_enable(dart->dev);

	/* The IOMMU core installs firmware reservations before attach_dev(). */
	ret = apple_dart_map_pages(&domain->domain, 0, SZ_1M, PAGE_SIZE, 1,
				   IOMMU_READ | IOMMU_WRITE, GFP_KERNEL, &mapped);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, mapped, (size_t)PAGE_SIZE);
	KUNIT_EXPECT_EQ(test, apple_dart_iotlb_sync_map(&domain->domain, 0, PAGE_SIZE), 0);
	KUNIT_EXPECT_EQ(test, ops->iova_to_phys(ops, 0), (phys_addr_t)SZ_1M);
	KUNIT_EXPECT_EQ(test, atomic_long_read(&domain->stream_maps[0].sidmap[0]), 0L);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8020_STREAM_COMMAND), 0U);

	/*
	 * A stream whose locked roots are not mapped yet is left to attach,
	 * which publishes the complete page table.
	 */
	KUNIT_EXPECT_EQ(test, apple_dart_domain_add_streams(domain, &master), 0);
	KUNIT_EXPECT_EQ(test, apple_dart_iotlb_sync_map(&domain->domain, 0, PAGE_SIZE), 0);

	pm_runtime_disable(dart->dev);
	free_io_pgtable_ops(ops);
}

static void apple_dart_test_locked_handoff(struct kunit *test)
{
	struct apple_dart_stream_map stream = {};
	struct io_pgtable_cfg cfg = {};
	struct apple_dart *dart;
	u64 live[] = { 0x1001, 0, 0x3001, 0 };
	u64 owned[ARRAY_SIZE(live)] = {};
	u64 ours[] = { 0x4001, 0x5001, 0, 0 };

	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	dart->hw = &apple_dart_hw_t8103;
	dart->num_streams = 1;
	dart->pgsize = sizeof(live);
	spin_lock_init(&dart->lock);
	dart->locked_ttbr[0][0] = live;
	dart->locked_owned[0][0] = owned;
	stream.dart = dart;
	__set_bit(0, stream.sidmap);
	cfg.apple_dart_cfg.n_ttbrs = 1;
	cfg.apple_dart_cfg.ttbr[0] = ours;

	/* Legacy firmware mappings are rebuilt by the IOMMU core at handoff. */
	KUNIT_EXPECT_EQ(test, apple_dart_hw_sync_locked(&cfg, &stream, false), 0);
	KUNIT_EXPECT_MEMEQ(test, live, ours, sizeof(live));
	KUNIT_EXPECT_MEMEQ(test, owned, ours, sizeof(owned));
	apple_dart_retire_root(live, owned, ARRAY_SIZE(live));
	KUNIT_EXPECT_EQ(test, live[0], 0ULL);
	KUNIT_EXPECT_EQ(test, live[1], 0ULL);

	/* Older T8110 instances also use the legacy display handoff. */
	dart->hw = &apple_dart_hw_t8110;
	dart->version = 0x0200;
	live[0] = 0x1001;
	KUNIT_EXPECT_EQ(test, apple_dart_hw_sync_locked(&cfg, &stream, false), 0);
	KUNIT_EXPECT_MEMEQ(test, live, ours, sizeof(live));
}

static void apple_dart_test_sync_unknown_roots(struct kunit *test)
{
	struct apple_dart_stream_map stream = {};
	struct io_pgtable_cfg cfg = {};
	struct apple_dart *dart;
	u64 live[] = { 0x1001, 0, 0, 0 };
	u64 ours[] = { 0, 0x2001, 0, 0 };

	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	dart->hw = &apple_dart_hw_t8110;
	dart->version = 0x0202;
	dart->num_streams = 1;
	dart->pgsize = sizeof(live);
	spin_lock_init(&dart->lock);
	/* The live root is mapped, its ownership shadow is not yet. */
	dart->locked_ttbr[0][0] = live;
	stream.dart = dart;
	__set_bit(0, stream.sidmap);
	cfg.apple_dart_cfg.n_ttbrs = 1;
	cfg.apple_dart_cfg.ttbr[0] = ours;

	/* An early TLB sync skips the stream and leaves the root alone... */
	KUNIT_EXPECT_EQ(test, apple_dart_hw_sync_locked(&cfg, &stream, true), 0);
	KUNIT_EXPECT_EQ(test, live[0], 0x1001ULL);
	KUNIT_EXPECT_EQ(test, live[1], 0ULL);
	/* ...but publishing at attach still requires both. */
	KUNIT_EXPECT_EQ(test, apple_dart_hw_sync_locked(&cfg, &stream, false), -EIO);
	KUNIT_EXPECT_EQ(test, live[1], 0ULL);
}

static void apple_dart_test_firmware_roots(struct kunit *test)
{
	struct apple_dart_stream_map stream = {};
	struct io_pgtable_cfg cfg = {};
	struct apple_dart *dart;
	u64 live[] = { 0x1001, 0, 0, 0 };
	u64 owned[ARRAY_SIZE(live)] = {};
	u64 ours[] = { 0, 0x2001, 0, 0 };

	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	dart->hw = &apple_dart_hw_t8110;
	dart->version = 0x0202;
	dart->num_streams = 1;
	dart->pgsize = sizeof(live);
	spin_lock_init(&dart->lock);
	dart->locked_ttbr[0][0] = live;
	dart->locked_owned[0][0] = owned;
	stream.dart = dart;
	__set_bit(0, stream.sidmap);
	cfg.apple_dart_cfg.n_ttbrs = 1;
	cfg.apple_dart_cfg.ttbr[0] = ours;

	KUNIT_ASSERT_EQ(test, apple_dart_hw_sync_locked(&cfg, &stream, false), 0);
	KUNIT_EXPECT_EQ(test, live[0], 0x1001ULL);
	KUNIT_EXPECT_EQ(test, live[1], 0x2001ULL);
	ours[0] = 0x3001;
	KUNIT_EXPECT_EQ(test, apple_dart_hw_sync_locked(&cfg, &stream, false), -EBUSY);
	dart->version = 0x0203;
	KUNIT_EXPECT_EQ(test, apple_dart_hw_sync_locked(&cfg, &stream, false), -EBUSY);
	KUNIT_EXPECT_EQ(test, live[0], 0x1001ULL);
	/* Teardown must preserve a root which firmware has since replaced. */
	live[1] = 0x4001;
	apple_dart_retire_root(live, owned, ARRAY_SIZE(live));
	KUNIT_EXPECT_EQ(test, live[0], 0x1001ULL);
	KUNIT_EXPECT_EQ(test, live[1], 0x4001ULL);
}

static struct apple_dart *apple_dart_test_t8110(struct kunit *test, const char *name)
{
	struct apple_dart *dart;

	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	dart->dev = kunit_device_register(test, name);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dart->dev);
	dart->regs = (__force void __iomem *)kunit_kzalloc(test, SZ_16K, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart->regs);
	dart->hw = &apple_dart_hw_t8110;
	dart->version = 0x0202;
	dart->num_streams = 1;
	spin_lock_init(&dart->lock);
	pm_runtime_set_active(dart->dev);
	pm_runtime_enable(dart->dev);
	return dart;
}

static void apple_dart_test_unmap_flush_after_publish_failure(struct kunit *test)
{
	struct iommu_iotlb_gather gather = { .start = 0, .end = SZ_16K - 1 };
	struct apple_dart_stream_map stream = {};
	struct apple_dart_domain *domain;
	struct apple_dart *locked, *other;
	struct io_pgtable *iop;
	u64 live[] = { 0x1001, 0, 0, 0 };
	u64 owned[ARRAY_SIZE(live)] = {};
	u64 ours[] = { 0x3001, 0, 0, 0 };
	u32 sentinel = 0x40;

	/* A locked stream whose root slot firmware holds: publishing fails. */
	locked = apple_dart_test_t8110(test, "apple-dart-flush-locked");
	locked->locked = true;
	locked->pgsize = sizeof(live);
	locked->locked_ttbr[0][0] = live;
	locked->locked_owned[0][0] = owned;
	other = apple_dart_test_t8110(test, "apple-dart-flush-other");
	other->pgsize = SZ_16K;

	iop = kunit_kzalloc(test, sizeof(*iop), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, iop);
	iop->cfg.apple_dart_cfg.n_ttbrs = 1;
	iop->cfg.apple_dart_cfg.ttbr[0] = ours;
	domain = kunit_kzalloc(test, sizeof(*domain), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, domain);
	domain->pgtbl_ops = &iop->ops;
	domain->stream_maps[0].dart = locked;
	atomic_long_set(&domain->stream_maps[0].sidmap[0], 1);
	domain->stream_maps[1].dart = other;
	atomic_long_set(&domain->stream_maps[1].sidmap[0], 1);

	writel(sentinel, locked->regs + DART_T8110_TLB_CMD);
	writel(sentinel, other->regs + DART_T8110_TLB_CMD);
	stream.dart = locked;
	__set_bit(0, stream.sidmap);
	KUNIT_ASSERT_EQ(test, apple_dart_hw_sync_locked(&iop->cfg, &stream, true),
			-EBUSY);

	/* The pages are freed after this: both DARTs must drop them. */
	apple_dart_iotlb_sync(&domain->domain, &gather);
	KUNIT_EXPECT_NE(test, readl(locked->regs + DART_T8110_TLB_CMD), sentinel);
	KUNIT_EXPECT_NE(test, readl(other->regs + DART_T8110_TLB_CMD), sentinel);
	KUNIT_EXPECT_EQ(test, live[0], 0x1001ULL);
	KUNIT_EXPECT_EQ(test, atomic_read(&locked->dev->power.usage_count), 0);
	KUNIT_EXPECT_EQ(test, atomic_read(&other->dev->power.usage_count), 0);
	pm_runtime_disable(locked->dev);
	pm_runtime_disable(other->dev);
}

static void apple_dart_test_gated_commands(struct kunit *test)
{
	struct apple_dart_stream_map stream = {};
	struct apple_dart *dart;
	u32 sentinel = 0x40;
	int ret;

	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	dart->dev = kunit_device_register(test, "apple-dart-gated-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dart->dev);
	dev_set_drvdata(dart->dev, dart);
	dart->regs = (__force void __iomem *)kunit_kzalloc(test, SZ_16K, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart->regs);
	dart->hw = &apple_dart_hw_t8110;
	dart->num_streams = 1;
	dart->pgsize = SZ_16K;
	dart->version = 0x0202;
	spin_lock_init(&dart->lock);
	stream.dart = dart;
	__set_bit(0, stream.sidmap);
	pm_runtime_set_active(dart->dev);
	pm_runtime_enable(dart->dev);

	writel(sentinel, dart->regs + DART_T8020_STREAM_SELECT);
	writel(sentinel, dart->regs + DART_T8020_STREAM_COMMAND);
	writel(sentinel, dart->regs + DART_T8110_TLB_CMD);
	writel(sentinel, dart->regs + DART_T8110_TLB_START);
	writel(sentinel, dart->regs + DART_T8110_TLB_END);
	apple_dart_quiesce_commands(dart->dev);

	/* Neither engine may touch its command registers after quiesce. */
	KUNIT_EXPECT_EQ(test, apple_dart_t8020_hw_invalidate_tlb(&stream), -EHOSTDOWN);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8020_STREAM_SELECT), sentinel);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8020_STREAM_COMMAND), sentinel);
	ret = apple_dart_t8110_hw_tlb_command_range(&stream,
						    DART_T8110_TLB_CMD_OP_FLUSH_SID,
						    true, 0, SZ_16K - 1);
	KUNIT_EXPECT_EQ(test, ret, -EHOSTDOWN);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8110_TLB_CMD), sentinel);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8110_TLB_START), sentinel);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8110_TLB_END), sentinel);

	/* A restored port may submit commands again. */
	apple_dart_resume_commands(dart->dev);
	KUNIT_EXPECT_EQ(test, apple_dart_t8020_hw_invalidate_tlb(&stream), 0);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8020_STREAM_COMMAND),
			(u32)DART_T8020_STREAM_COMMAND_INVALIDATE);
	KUNIT_EXPECT_EQ(test, apple_dart_t8110_hw_invalidate_tlb(&stream), 0);
	KUNIT_EXPECT_NE(test, readl(dart->regs + DART_T8110_TLB_CMD), sentinel);
	pm_runtime_disable(dart->dev);
}

static void apple_dart_test_resume_commands_failure(struct kunit *test)
{
	struct apple_dart *dart;

	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	dart->dev = kunit_device_register(test, "apple-dart-resume-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dart->dev);
	dev_set_drvdata(dart->dev, dart);
	spin_lock_init(&dart->lock);
	pm_runtime_set_active(dart->dev);
	pm_runtime_enable(dart->dev);
	apple_dart_quiesce_commands(dart->dev);

	/* A prior runtime-PM error must leave commands gated and report failure. */
	scoped_guard(spinlock_irqsave, &dart->dev->power.lock)
		dart->dev->power.runtime_error = -EIO;
	KUNIT_EXPECT_EQ(test, apple_dart_resume_commands(dart->dev), -EINVAL);
	KUNIT_EXPECT_TRUE(test, dart->commands_gated);
	KUNIT_EXPECT_EQ(test, atomic_read(&dart->dev->power.usage_count), 0);

	/* Clearing the PM error permits a later retry without leaking a reference. */
	KUNIT_EXPECT_EQ(test, pm_runtime_set_active(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, apple_dart_resume_commands(dart->dev), 0);
	KUNIT_EXPECT_FALSE(test, dart->commands_gated);
	KUNIT_EXPECT_EQ(test, atomic_read(&dart->dev->power.usage_count), 0);
	pm_runtime_disable(dart->dev);
}

/*
 * Large enough for the T8110 layout, whose exception records reach up to SID
 * 255 below 0xc000. The page after it is vmalloc's guard, so an access beyond
 * any layout faults instead of reaching other memory.
 */
#define APPLE_DART_TEST_REGS_SIZE SZ_64K

KUNIT_DEFINE_ACTION_WRAPPER(apple_dart_test_vfree, vfree, const void *);

static struct apple_dart *apple_dart_test_tunnel(struct kunit *test)
{
	struct apple_dart *dart;
	void *regs;

	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	dart->dev = kunit_device_register(test, "apple-dart-tunnel-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dart->dev);
	regs = vzalloc(APPLE_DART_TEST_REGS_SIZE);
	KUNIT_ASSERT_NOT_NULL(test, regs);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, apple_dart_test_vfree, regs), 0);
	dart->regs = (__force void __iomem *)regs;
	dart->hw = &apple_dart_hw_t8103_usb4;
	dart->num_streams = dart->hw->max_sid_count;
	dart->tunneled = true;
	spin_lock_init(&dart->lock);
	dev_set_drvdata(dart->dev, dart);
	return dart;
}

static void apple_dart_test_tunnel_restore(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_tunnel(test);
	unsigned int sid, idx;
	u32 sentinel = 0x40;

	/* M1 tunnel DART nodes have no retained power-domain attachment. */
	KUNIT_ASSERT_FALSE(test, dart->power_retained);
	for (sid = 0; sid < dart->num_streams; sid++) {
		writel(0x80 + sid, dart->regs + DART_TCR(dart, sid));
		for (idx = 0; idx < dart->hw->ttbr_count; idx++)
			writel(0x1000 + 0x10 * sid + idx,
			       dart->regs + DART_TTBR(dart, sid, idx));
	}
	KUNIT_ASSERT_EQ(test, apple_dart_save_tunnel_state(dart->dev), 0);
	memset_io(dart->regs, 0, APPLE_DART_TEST_REGS_SIZE);
	KUNIT_ASSERT_EQ(test, apple_dart_restore_tunnel_state(dart->dev), 0);
	for (sid = 0; sid < dart->num_streams; sid++) {
		KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, sid)),
				0x80 + sid);
		for (idx = 0; idx < dart->hw->ttbr_count; idx++)
			KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TTBR(dart, sid, idx)),
					0x1000 + 0x10 * sid + idx);
	}
	KUNIT_EXPECT_FALSE(test, dart->tunnel_state_saved);
	KUNIT_EXPECT_TRUE(test, dart->tunnel_state_restored);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8020_STREAM_SELECT), U32_MAX);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8020_STREAM_SELECT + 4), U32_MAX);

	/* Normal resume must not reset translations after the link is live. */
	writel(sentinel, dart->regs + DART_T8020_STREAM_COMMAND);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8020_STREAM_COMMAND), sentinel);
	KUNIT_EXPECT_FALSE(test, dart->tunnel_state_restored);
	KUNIT_EXPECT_EQ(test, apple_dart_restore_tunnel_state(dart->dev), -EINVAL);
}

static void apple_dart_test_tunnel_reject_unsafe(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_tunnel(test);
	u32 sentinel = 0x40;

	writel(sentinel, dart->regs + DART_T8020_STREAM_COMMAND);
	KUNIT_EXPECT_EQ(test, apple_dart_restore_tunnel_state(dart->dev), -EINVAL);
	dart->locked = true;
	KUNIT_EXPECT_EQ(test, apple_dart_save_tunnel_state(dart->dev), -EOPNOTSUPP);
	dart->locked = false;
	dart->commands_gated = true;
	KUNIT_EXPECT_EQ(test, apple_dart_save_tunnel_state(dart->dev), -EBUSY);
	KUNIT_EXPECT_FALSE(test, dart->tunnel_state_saved);
	dart->commands_gated = false;
	dart->hw = &apple_dart_hw_t8110;
	KUNIT_EXPECT_EQ(test, apple_dart_save_tunnel_state(dart->dev), -EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8020_STREAM_COMMAND), sentinel);
}

static int apple_dart_test_tlb_failure(struct apple_dart_stream_map *stream_map)
{
	return -EIO;
}

static void apple_dart_test_tunnel_reset_failure(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_tunnel(test);
	struct apple_dart_hw hw = apple_dart_hw_t8103_usb4;

	hw.invalidate_tlb = apple_dart_test_tlb_failure;
	dart->hw = &hw;
	KUNIT_ASSERT_EQ(test, apple_dart_save_tunnel_state(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, apple_dart_restore_tunnel_state(dart->dev), -EIO);
	KUNIT_EXPECT_FALSE(test, dart->tunnel_state_restored);
	KUNIT_EXPECT_TRUE(test, dart->tunnel_state_saved);

	/* A later successful reset can still restore the saved mappings. */
	dart->hw = &apple_dart_hw_t8103_usb4;
	KUNIT_EXPECT_EQ(test, apple_dart_restore_tunnel_state(dart->dev), 0);
	KUNIT_EXPECT_TRUE(test, dart->tunnel_state_restored);
	KUNIT_EXPECT_EQ(test, apple_dart_suspend(dart->dev), 0);
	KUNIT_EXPECT_FALSE(test, dart->tunnel_state_restored);
}

static void apple_dart_test_failed_host_pm(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_tunnel(test);
	u32 sentinel = 0x40;

	writel(0x80, dart->regs + DART_TCR(dart, 0));
	KUNIT_ASSERT_EQ(test, apple_dart_save_tunnel_state(dart->dev), 0);
	apple_dart_quiesce_commands(dart->dev);
	writel(sentinel, dart->regs + DART_T8020_STREAM_COMMAND);
	writel(sentinel, dart->regs + DART_TCR(dart, 0));

	/* PM still resumes dependents after the host's noirq callback fails. */
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8020_STREAM_COMMAND), sentinel);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 0)), sentinel);
	KUNIT_EXPECT_TRUE(test, dart->commands_gated);

	/* Another sleep must neither access the failed port nor lose its snapshot. */
	KUNIT_EXPECT_EQ(test, apple_dart_suspend(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, dart->save_tcr[0], 0x80U);
	KUNIT_EXPECT_EQ(test, apple_dart_save_tunnel_state(dart->dev), -EBUSY);
	KUNIT_EXPECT_TRUE(test, dart->tunnel_state_saved);
	KUNIT_EXPECT_EQ(test, dart->save_tcr[0], 0x80U);
}

static irqreturn_t apple_dart_test_irq_handler(int irq, void *data)
{
	struct apple_dart *dart = data;

	writel(0x1234, dart->regs + DART_TCR(dart, 0));
	return IRQ_HANDLED;
}

static void apple_dart_test_gated_irq(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_tunnel(test);
	struct apple_dart_hw hw = apple_dart_hw_t8103_usb4;

	hw.irq_handler = apple_dart_test_irq_handler;
	dart->hw = &hw;
	pm_runtime_set_active(dart->dev);
	pm_runtime_enable(dart->dev);
	apple_dart_quiesce_commands(dart->dev);
	KUNIT_EXPECT_EQ(test, apple_dart_irq(0, dart), IRQ_NONE);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 0)), 0U);
	KUNIT_EXPECT_EQ(test, atomic_read(&dart->dev->power.usage_count), 0);

	KUNIT_ASSERT_EQ(test, apple_dart_resume_commands(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, apple_dart_irq(0, dart), IRQ_HANDLED);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 0)), 0x1234U);
	KUNIT_EXPECT_EQ(test, atomic_read(&dart->dev->power.usage_count), 0);

	/* Drain the work queued by the successful IRQ's pm_runtime_put(). */
	pm_runtime_barrier(dart->dev);

	/* Failed runtime resume must not run the handler or leak its reference. */
	writel(0, dart->regs + DART_TCR(dart, 0));
	scoped_guard(spinlock_irqsave, &dart->dev->power.lock)
		dart->dev->power.runtime_error = -EIO;
	KUNIT_EXPECT_EQ(test, apple_dart_irq(0, dart), IRQ_NONE);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 0)), 0U);
	KUNIT_EXPECT_EQ(test, atomic_read(&dart->dev->power.usage_count), 0);
	KUNIT_EXPECT_EQ(test, pm_runtime_set_active(dart->dev), 0);
	pm_runtime_disable(dart->dev);
}

struct apple_dart_test_commands {
	DECLARE_BITMAP(sids, DART_MAX_STREAMS);
	unsigned int count;
};

static void apple_dart_test_record_write(struct apple_dart *dart, u32 value,
					 u32 offset)
{
	struct kunit *test = kunit_get_current_test();
	struct apple_dart_test_commands *commands = test->priv;

	if (offset == DART_T8110_TLB_CMD) {
		__set_bit(FIELD_GET(DART_T8110_TLB_CMD_STREAM, value), commands->sids);
		commands->count++;
	}
	writel(value, dart->regs + offset);
}

/* Separate masters on one DART need not share a remap target. */
static void apple_dart_test_independent_remap_roots(struct kunit *test)
{
	struct apple_dart_master_cfg masters[2] = {};
	struct apple_dart_test_commands commands = {};
	struct iommu_iotlb_gather gather;
	struct apple_dart_domain *domain;
	struct apple_dart *dart;
	size_t mapped = 0;
	unsigned int i;

	/* T8110 table pointers must be aligned to its 16 KiB granule. */
	if (PAGE_SIZE < SZ_16K) {
		kunit_skip(test, "T8110 needs kernel pages of at least 16 KiB");
		return;
	}

	dart = kunit_kzalloc(test, sizeof(*dart), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart);
	domain = kunit_kzalloc(test, sizeof(*domain), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, domain);
	dart->dev = kunit_device_register(test, "apple-dart-remap-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dart->dev);
	dart->hw = &apple_dart_hw_t8110;
	dart->num_streams = 16;
	dart->pgsize = SZ_16K;
	dart->ias = 32;
	dart->oas = 42;
	dart->regs = (__force void __iomem *)kunit_kzalloc(test, SZ_16K, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dart->regs);
	spin_lock_init(&dart->lock);
	mutex_init(&domain->init_lock);
	domain->domain.type = IOMMU_DOMAIN_UNMANAGED;
	test->priv = &commands;
	kunit_activate_static_stub(test, apple_dart_writel, apple_dart_test_record_write);
	pm_runtime_set_active(dart->dev);
	pm_runtime_enable(dart->dev);

	for (i = 0; i < ARRAY_SIZE(masters); i++) {
		struct device *dev;
		unsigned int sid = 3 + 2 * i;
		int ret;

		dev = kunit_device_register(test, i ? "dart-master-5" : "dart-master-3");
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
		dev->iommu = kunit_kzalloc(test, sizeof(*dev->iommu), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, dev->iommu);
		masters[i].dma_max = DMA_BIT_MASK(32);
		masters[i].stream_maps[0].dart = dart;
		__set_bit(sid, masters[i].stream_maps[0].sidmap);
		dev->iommu->priv = &masters[i];
		ret = apple_dart_attach_dev_paging(&domain->domain, dev, NULL);
		dev->iommu = NULL;
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, sid)),
				dart->hw->tcr_enabled);
	}

	KUNIT_ASSERT_EQ(test, apple_dart_map_pages(&domain->domain, 0, SZ_1M,
						   SZ_16K, 1, IOMMU_READ | IOMMU_WRITE,
						   GFP_KERNEL, &mapped), 0);
	KUNIT_ASSERT_EQ(test, mapped, (size_t)SZ_16K);
	KUNIT_ASSERT_EQ(test, apple_dart_iotlb_sync_map(&domain->domain, 0, SZ_16K), 0);
	KUNIT_EXPECT_EQ(test, apple_dart_iova_to_phys(&domain->domain, 0), (phys_addr_t)SZ_1M);

	bitmap_zero(commands.sids, DART_MAX_STREAMS);
	commands.count = 0;
	iommu_iotlb_gather_init(&gather);
	KUNIT_EXPECT_EQ(test, apple_dart_unmap_pages(&domain->domain, 0, SZ_16K, 1,
						     &gather), (size_t)SZ_16K);
	apple_dart_iotlb_sync(&domain->domain, &gather);
	KUNIT_EXPECT_EQ(test, apple_dart_iova_to_phys(&domain->domain, 0), (phys_addr_t)0);
	KUNIT_EXPECT_EQ(test, commands.count, 2U);
	KUNIT_EXPECT_EQ(test, commands.sids[0], BIT(3) | BIT(5));

	pm_runtime_disable(dart->dev);
	free_io_pgtable_ops(domain->pgtbl_ops);
	kunit_deactivate_static_stub(test, apple_dart_writel);
	test->priv = NULL;
}

static struct apple_dart *apple_dart_test_sleeping(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_tunnel(test);
	unsigned int sid;

	dart->tunneled = false;
	dart->hw = &apple_dart_hw_t8103;
	dart->num_streams = 16;
	for (sid = 0; sid < dart->num_streams; sid++)
		writel(0x80 + sid, dart->regs + DART_TCR(dart, sid));
	writel(dart->hw->ttbr_valid | 0x1000, dart->regs + DART_TTBR(dart, 2, 0));
	KUNIT_ASSERT_EQ(test, apple_dart_suspend(dart->dev), 0);
	return dart;
}

static void apple_dart_test_resume_retained(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_sleeping(test);
	u32 enable = dart->hw->enable_streams;

	/* The domain stayed on and a consumer may still be doing DMA: no reset. */
	writel(0, dart->regs + enable);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + enable), 0U);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 2)), 0x82U);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TTBR(dart, 2, 0)),
			dart->hw->ttbr_valid | 0x1000);
}

static void apple_dart_test_resume_lost(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_sleeping(test);

	/* Power was cut: reset and restore the saved translations. */
	memset_io(dart->regs, 0, APPLE_DART_TEST_REGS_SIZE);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + dart->hw->enable_streams), U32_MAX);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 2)), 0x82U);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TTBR(dart, 2, 0)),
			dart->hw->ttbr_valid | 0x1000);
}

static void apple_dart_test_resume_unused(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_tunnel(test);

	/* Matching registers without any page table prove nothing: reset. */
	dart->tunneled = false;
	dart->hw = &apple_dart_hw_t8103;
	dart->num_streams = 16;
	KUNIT_ASSERT_EQ(test, apple_dart_suspend(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + dart->hw->enable_streams), U32_MAX);
}

/*
 * A DART that kept its state is flushed stream by stream. Where the streams of
 * a map are remapped onto its first stream, as on DARTs that program only that
 * stream, a single flush of all streams would reach stream 0 and nothing else.
 */
static void apple_dart_test_resume_retained_each_stream(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_tunnel(test);
	unsigned int sid;
	u32 cmd;

	dart->tunneled = false;
	dart->hw = &apple_dart_hw_t8110;
	dart->num_streams = 16;
	for (sid = 0; sid < dart->num_streams; sid++)
		writel(dart->hw->tcr_enabled, dart->regs + DART_TCR(dart, sid));
	writel(dart->hw->ttbr_valid | 0x1000, dart->regs + DART_TTBR(dart, 0, 0));
	writel(dart->hw->ttbr_valid | 0x2000, dart->regs + DART_TTBR(dart, 5, 0));
	KUNIT_ASSERT_EQ(test, apple_dart_suspend(dart->dev), 0);

	writel(0, dart->regs + DART_T8110_TLB_CMD);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), 0);
	/* Commands go out in stream order: the last one names the last stream. */
	cmd = readl(dart->regs + DART_T8110_TLB_CMD);
	KUNIT_EXPECT_EQ(test, FIELD_GET(DART_T8110_TLB_CMD_STREAM, cmd), dart->num_streams - 1);
	KUNIT_EXPECT_EQ(test, FIELD_GET(DART_T8110_TLB_CMD_OP, cmd),
			(u32)DART_T8110_TLB_CMD_OP_FLUSH_SID);
	/* Nothing was reset. */
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TTBR(dart, 5, 0)),
			dart->hw->ttbr_valid | 0x2000);
}

/*
 * A stream of an identity domain bypasses the DART without a page table. Its
 * consumer may still be doing DMA, so a DART that kept that state is not reset.
 */
static void apple_dart_test_resume_retained_bypass_hw(struct kunit *test,
						      const struct apple_dart_hw *hw)
{
	struct apple_dart *dart = apple_dart_test_tunnel(test);
	u32 enable;

	dart->tunneled = false;
	dart->hw = hw;
	dart->num_streams = 16;
	dart->supports_bypass = true;
	enable = dart->hw->enable_streams;
	writel(dart->hw->tcr_bypass, dart->regs + DART_TCR(dart, 3));
	KUNIT_ASSERT_EQ(test, apple_dart_suspend(dart->dev), 0);

	writel(0, dart->regs + enable);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), 0);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + enable), 0U);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 3)), dart->hw->tcr_bypass);
}

static void apple_dart_test_resume_retained_bypass_t8103(struct kunit *test)
{
	apple_dart_test_resume_retained_bypass_hw(test, &apple_dart_hw_t8103);
}

static void apple_dart_test_resume_retained_bypass_t8110(struct kunit *test)
{
	apple_dart_test_resume_retained_bypass_hw(test, &apple_dart_hw_t8110);
}

/*
 * The PCIe DART of the J700 radio: two streams translating for a function
 * that keeps running in suspend-to-idle.
 */
static struct apple_dart *apple_dart_test_radio(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_tunnel(test);
	unsigned int sid;

	dart->tunneled = false;
	dart->hw = &apple_dart_hw_t8110;
	dart->num_streams = 16;
	dart->version = 0x0202;
	for (sid = 1; sid <= 2; sid++) {
		writel(dart->hw->tcr_enabled, dart->regs + DART_TCR(dart, sid));
		writel(dart->hw->ttbr_valid | (0x1000 * sid),
		       dart->regs + DART_TTBR(dart, sid, 0));
	}
	KUNIT_ASSERT_EQ(test, apple_dart_suspend(dart->dev), 0);
	return dart;
}

static void apple_dart_test_radio_flush_timeout(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_radio(test);
	u32 busy = DART_T8110_TLB_CMD_BUSY |
		   FIELD_PREP(DART_T8110_TLB_CMD_OP, DART_T8110_TLB_CMD_OP_FLUSH_SID);

	/* A flush that never completes must not cost the streams their tables. */
	writel(busy, dart->regs + DART_T8110_TLB_CMD);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 1)), dart->hw->tcr_enabled);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TTBR(dart, 2, 0)),
			dart->hw->ttbr_valid | 0x2000);

	/* The same after a power loss, where the DART is reset first. */
	memset_io(dart->regs, 0, APPLE_DART_TEST_REGS_SIZE);
	writel(busy, dart->regs + DART_T8110_TLB_CMD);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 2)), dart->hw->tcr_enabled);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TTBR(dart, 1, 0)),
			dart->hw->ttbr_valid | 0x1000);
}

/* The radio DART after a power loss, set to lock on errors. */
static struct apple_dart *apple_dart_test_radio_lost(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_radio(test);

	memset_io(dart->regs, 0, APPLE_DART_TEST_REGS_SIZE);
	writel(DART_T8110_DIAG_LOCK_ON_ERR, dart->regs + DART_T8110_DIAG_LOCK);
	/* The reset unmasks the error interrupt only after the unlock. */
	writel(0x55, dart->regs + DART_T8110_ERROR_MASK);
	return dart;
}

/* The last command flushed the last stream, and nothing is pending. */
static void apple_dart_test_expect_flushed(struct kunit *test, struct apple_dart *dart)
{
	u32 cmd = readl(dart->regs + DART_T8110_TLB_CMD);

	KUNIT_EXPECT_FALSE(test, cmd & DART_T8110_TLB_CMD_BUSY);
	KUNIT_EXPECT_EQ(test, FIELD_GET(DART_T8110_TLB_CMD_OP, cmd),
			(u32)DART_T8110_TLB_CMD_OP_FLUSH_SID);
	KUNIT_EXPECT_EQ(test, FIELD_GET(DART_T8110_TLB_CMD_STREAM, cmd), dart->num_streams - 1);
}

static int apple_dart_test_unlocks;

/*
 * The command engine is still busy when the reset unlocks the DART, and goes
 * idle just after the unlock gave up waiting for it.
 */
static int apple_dart_test_unlock_late(struct apple_dart *dart)
{
	struct kunit *test = kunit_get_current_test();
	bool first = !apple_dart_test_unlocks++;
	int ret;

	if (first)
		writel(DART_T8110_TLB_CMD_BUSY, dart->regs + DART_T8110_TLB_CMD);
	kunit_deactivate_static_stub(test, apple_dart_t8110_v2_unlock);
	ret = apple_dart_t8110_v2_unlock(dart);
	kunit_activate_static_stub(test, apple_dart_t8110_v2_unlock,
				   apple_dart_test_unlock_late);
	if (first)
		writel(0, dart->regs + DART_T8110_TLB_CMD);
	return ret;
}

static void apple_dart_test_radio_unlock_timeout(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_radio_lost(test);

	apple_dart_test_unlocks = 0;
	kunit_activate_static_stub(test, apple_dart_t8110_v2_unlock,
				   apple_dart_test_unlock_late);

	/* The stream flushes after the failed unlock complete... */
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), 0);
	apple_dart_test_expect_flushed(test, dart);
	/* ...but the reset only ends once the unlock and the error mask are done. */
	KUNIT_EXPECT_EQ(test, apple_dart_test_unlocks, 2);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8110_ERROR_MASK), 0U);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 1)), dart->hw->tcr_enabled);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TTBR(dart, 2, 0)),
			dart->hw->ttbr_valid | 0x2000);
}

static void apple_dart_test_radio_unlock_stuck(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_radio_lost(test);

	/* An unlock that never gets through fails resume, with the tables back. */
	writel(DART_T8110_TLB_CMD_BUSY, dart->regs + DART_T8110_TLB_CMD);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8110_ERROR_MASK), 0x55U);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 2)), dart->hw->tcr_enabled);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TTBR(dart, 1, 0)),
			dart->hw->ttbr_valid | 0x1000);

	/*
	 * The next resume finds the tables as they were saved, and must still
	 * unlock the DART and unmask its errors, not only flush the TLBs.
	 */
	writel(0, dart->regs + DART_T8110_TLB_CMD);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), 0);
	apple_dart_test_expect_flushed(test, dart);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_T8110_ERROR_MASK), 0U);
}

static int apple_dart_test_flush_calls;

static int apple_dart_test_flush_once_late(struct apple_dart_stream_map *stream_map)
{
	return apple_dart_test_flush_calls++ ? 0 : -ETIMEDOUT;
}

static void apple_dart_test_resume_reset_retry(struct kunit *test)
{
	struct apple_dart *dart = apple_dart_test_sleeping(test);
	struct apple_dart_hw hw = apple_dart_hw_t8103;

	/* The reset's flush times out once; the second flush completes. */
	hw.invalidate_tlb = apple_dart_test_flush_once_late;
	dart->hw = &hw;
	apple_dart_test_flush_calls = 0;
	memset_io(dart->regs, 0, APPLE_DART_TEST_REGS_SIZE);
	KUNIT_EXPECT_EQ(test, apple_dart_resume(dart->dev), 0);
	KUNIT_EXPECT_GT(test, apple_dart_test_flush_calls, 1);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TCR(dart, 2)), 0x82U);
	KUNIT_EXPECT_EQ(test, readl(dart->regs + DART_TTBR(dart, 2, 0)),
			dart->hw->ttbr_valid | 0x1000);
}

static struct kunit_case apple_dart_test_cases[] = {
	KUNIT_CASE(apple_dart_test_iommu_specifiers),
	KUNIT_CASE(apple_dart_test_consumer_window),
	KUNIT_CASE(apple_dart_test_invalid_consumer_window),
	KUNIT_CASE(apple_dart_test_independent_remap_roots),
	KUNIT_CASE(apple_dart_test_before_attach),
	KUNIT_CASE(apple_dart_test_locked_handoff),
	KUNIT_CASE(apple_dart_test_sync_unknown_roots),
	KUNIT_CASE(apple_dart_test_firmware_roots),
	KUNIT_CASE(apple_dart_test_unmap_flush_after_publish_failure),
	KUNIT_CASE(apple_dart_test_gated_commands),
	KUNIT_CASE(apple_dart_test_resume_commands_failure),
	KUNIT_CASE(apple_dart_test_tunnel_restore),
	KUNIT_CASE(apple_dart_test_tunnel_reject_unsafe),
	KUNIT_CASE(apple_dart_test_tunnel_reset_failure),
	KUNIT_CASE(apple_dart_test_failed_host_pm),
	KUNIT_CASE(apple_dart_test_gated_irq),
	KUNIT_CASE(apple_dart_test_resume_retained),
	KUNIT_CASE(apple_dart_test_resume_lost),
	KUNIT_CASE(apple_dart_test_resume_unused),
	KUNIT_CASE(apple_dart_test_resume_retained_each_stream),
	KUNIT_CASE(apple_dart_test_radio_flush_timeout),
	KUNIT_CASE(apple_dart_test_radio_unlock_timeout),
	KUNIT_CASE(apple_dart_test_radio_unlock_stuck),
	KUNIT_CASE(apple_dart_test_resume_reset_retry),
	KUNIT_CASE(apple_dart_test_resume_retained_bypass_t8103),
	KUNIT_CASE(apple_dart_test_resume_retained_bypass_t8110),
	{}
};

static struct kunit_suite apple_dart_test_suite = {
	.name = "apple-dart",
	.test_cases = apple_dart_test_cases,
};

kunit_test_suite(apple_dart_test_suite);
