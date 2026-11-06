// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * KUnit tests for the Apple PMC voter interface: agent enables and the
 * AP's DCS vote, with memory standing in for the registers.
 *
 * Included by pmc.c.
 */

#include <kunit/device.h>
#include <kunit/test.h>

#define APPLE_PMC_TEST_RAILS_SIZE	0x4000

static struct apple_pmc *apple_pmc_test_alloc_hw(struct kunit *test,
						 const struct apple_pmc_hw *hw)
{
	struct apple_pmc *pmc;

	pmc = kunit_kzalloc(test, sizeof(*pmc), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pmc);
	pmc->dev = kunit_device_register(test, "apple-pmc-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, pmc->dev);
	pmc->hw = hw;
	raw_spin_lock_init(&pmc->lock);
	pmc->voters = (void __iomem *)kunit_kzalloc(test,
						    hw->agents * hw->agent_stride,
						    GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, (void __force *)pmc->voters);
	pmc->rails = (void __iomem *)kunit_kzalloc(test,
						   APPLE_PMC_TEST_RAILS_SIZE,
						   GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, (void __force *)pmc->rails);
	dev_set_drvdata(pmc->dev, pmc);

	return pmc;
}

static struct apple_pmc *apple_pmc_test_alloc(struct kunit *test)
{
	return apple_pmc_test_alloc_hw(test, &apple_pmc_hw_t8140);
}

static void apple_pmc_test_set_dcs_vote(struct apple_pmc *pmc,
					unsigned int agent, unsigned int state)
{
	void __iomem *reg = pmc->voters + agent * pmc->hw->agent_stride;

	writel(state << APPLE_PMC_RAIL_SHIFT(pmc->hw->dcs_rail), reg);
}

/* Enabling and disabling an agent changes its bit on every rail only. */
static void apple_pmc_test_agent_enable(struct kunit *test)
{
	struct apple_pmc *pmc = apple_pmc_test_alloc(test);
	unsigned int rail;

	pmc->enable[0] = BIT(0);
	writel(BIT(0), pmc->rails + pmc->hw->rail_enable);

	KUNIT_ASSERT_EQ(test, apple_pmc_agent_enable(pmc, 1, true), 0);
	for (rail = 0; rail < pmc->hw->rails; rail++)
		KUNIT_EXPECT_EQ(test, apple_pmc_enable_read(pmc, rail),
				BIT(1) | (rail ? 0 : BIT(0)));

	KUNIT_ASSERT_EQ(test, apple_pmc_agent_enable(pmc, 1, false), 0);
	for (rail = 0; rail < pmc->hw->rails; rail++)
		KUNIT_EXPECT_EQ(test, apple_pmc_enable_read(pmc, rail),
				rail ? 0 : BIT(0));
}

/* Lifting a DCS limit gives the AP's agent back its own vote, not 0. */
static void apple_pmc_test_dcs_limit_restores_ap_vote(struct kunit *test)
{
	struct apple_pmc *pmc = apple_pmc_test_alloc(test);
	const struct apple_pmc_hw *hw = pmc->hw;

	apple_pmc_test_set_dcs_vote(pmc, hw->ap_agent, 2);
	apple_pmc_test_set_dcs_vote(pmc, 0, 4);
	pmc->enable[hw->dcs_rail] = BIT(0) | BIT(hw->ap_agent);
	pmc->ap_dcs_vote = apple_pmc_ap_dcs_read(pmc);
	KUNIT_ASSERT_EQ(test, pmc->ap_dcs_vote, 2);

	/* A limit of 3 lets the AP vote at most DCS 1 for the PMP's 4. */
	pmc->dcs_state = 3;
	apple_pmc_dcs_vote(pmc);
	KUNIT_EXPECT_EQ(test, apple_pmc_ap_dcs_read(pmc), 1);

	pmc->dcs_state = 0;
	apple_pmc_dcs_vote(pmc);
	KUNIT_EXPECT_EQ(test, apple_pmc_ap_dcs_read(pmc), 2);
}

struct apple_pmc_test_reject {
	unsigned int rail;
	u32 enable;
	bool rejected;
};

static u32 apple_pmc_test_reject_read(struct apple_pmc *pmc, unsigned int rail)
{
	struct kunit *test = kunit_get_current_test();
	struct apple_pmc_test_reject *reject = test->priv;
	void __iomem *reg = pmc->rails + pmc->hw->rail_enable + 4 * rail;
	u32 val = readl(reg);

	if (rail == reject->rail && !reject->rejected && val != reject->enable) {
		reject->rejected = true;
		writel(reject->enable, reg);
		return reject->enable;
	}
	return val;
}

/* Genpd leaves a domain off after power_on() fails on an intermediate rail. */
static void apple_pmc_test_partial_enable(struct kunit *test)
{
	struct apple_pmc *pmc = apple_pmc_test_alloc(test);
	struct apple_pmc_test_reject reject = { .rail = 1, .enable = BIT(0) };
	struct apple_pmc_domain domain = { .pmc = pmc, .agent = 1 };
	unsigned int rail;

	for (rail = 0; rail < pmc->hw->rails; rail++) {
		pmc->enable[rail] = BIT(0);
		writel(BIT(0), pmc->rails + pmc->hw->rail_enable + 4 * rail);
	}
	test->priv = &reject;
	kunit_activate_static_stub(test, apple_pmc_enable_read, apple_pmc_test_reject_read);
	KUNIT_EXPECT_EQ(test, apple_pmc_domain_power_on(&domain.genpd), -EIO);
	KUNIT_EXPECT_TRUE(test, reject.rejected);
	for (rail = 0; rail < pmc->hw->rails; rail++) {
		KUNIT_EXPECT_EQ(test, pmc->enable[rail], BIT(0));
		KUNIT_EXPECT_EQ(test, apple_pmc_enable_read(pmc, rail), BIT(0));
	}
	kunit_deactivate_static_stub(test, apple_pmc_enable_read);
	test->priv = NULL;
}

/* Probe unwind must also restore a nonzero inherited AP DCS vote. */
static void apple_pmc_test_release_restores_ap_vote(struct kunit *test)
{
	struct apple_pmc *pmc = apple_pmc_test_alloc(test);
	const struct apple_pmc_hw *hw = pmc->hw;
	unsigned int rail;

	INIT_LIST_HEAD(&pmc->domains);
	INIT_DELAYED_WORK(&pmc->dcs_work, apple_pmc_dcs_work);
	apple_pmc_test_set_dcs_vote(pmc, hw->ap_agent, 2);
	pmc->ap_dcs_vote = apple_pmc_ap_dcs_read(pmc);
	apple_pmc_test_set_dcs_vote(pmc, 0, 4);
	pmc->enable[hw->dcs_rail] = BIT(0) | BIT(hw->ap_agent);
	pmc->dcs_state = 3;
	apple_pmc_dcs_vote(pmc);
	KUNIT_EXPECT_EQ(test, apple_pmc_ap_dcs_read(pmc), 1);
	apple_pmc_release(pmc);
	KUNIT_EXPECT_EQ(test, pmc->dcs_state, 0UL);
	KUNIT_EXPECT_EQ(test, apple_pmc_ap_dcs_read(pmc), 2);
	for (rail = 0; rail < hw->rails; rail++)
		KUNIT_EXPECT_EQ(test, apple_pmc_enable_read(pmc, rail), pmc->saved_enable[rail]);
}

/* T8132: with the agent's pending bit clear, a floor update is written. */
static void apple_pmc_test_t8132_floor_update(struct kunit *test)
{
	struct apple_pmc *pmc = apple_pmc_test_alloc_hw(test, &apple_pmc_hw_t8132);
	unsigned int agent = pmc->hw->ap_agent;
	void __iomem *reg = pmc->voters + agent * pmc->hw->agent_stride;

	KUNIT_ASSERT_EQ(test, apple_pmc_floor_update(pmc->dev, agent, 0xff, 0x35), 0);
	KUNIT_EXPECT_EQ(test, readl(reg) & 0xff, 0x35);
}

/*
 * T8132: while the PMC has not taken up the agent's previous floor, the
 * update times out and leaves the floor word alone.
 */
static void apple_pmc_test_t8132_pending_stuck(struct kunit *test)
{
	struct apple_pmc *pmc = apple_pmc_test_alloc_hw(test, &apple_pmc_hw_t8132);
	unsigned int agent = pmc->hw->ap_agent;
	void __iomem *reg = pmc->voters + agent * pmc->hw->agent_stride;

	writel(0x12, reg);
	writel(BIT(agent), reg + 4);
	KUNIT_EXPECT_EQ(test, apple_pmc_floor_update(pmc->dev, agent, 0xff, 0x35),
			-ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, readl(reg), 0x12);
}

/* T8140 has no pending bit: the word after the floor word is not waited on. */
static void apple_pmc_test_t8140_ignores_pending_word(struct kunit *test)
{
	struct apple_pmc *pmc = apple_pmc_test_alloc(test);
	unsigned int agent = pmc->hw->ap_agent;
	void __iomem *reg = pmc->voters + agent * pmc->hw->agent_stride;

	writel(BIT(agent), reg + 4);
	KUNIT_ASSERT_EQ(test, apple_pmc_floor_update(pmc->dev, agent, 0xff, 0x35), 0);
	KUNIT_EXPECT_EQ(test, readl(reg) & 0xff, 0x35);
}

static struct kunit_case apple_pmc_test_cases[] = {
	KUNIT_CASE(apple_pmc_test_partial_enable),
	KUNIT_CASE(apple_pmc_test_release_restores_ap_vote),
	KUNIT_CASE(apple_pmc_test_agent_enable),
	KUNIT_CASE(apple_pmc_test_dcs_limit_restores_ap_vote),
	KUNIT_CASE(apple_pmc_test_t8132_floor_update),
	KUNIT_CASE(apple_pmc_test_t8132_pending_stuck),
	KUNIT_CASE(apple_pmc_test_t8140_ignores_pending_word),
	{}
};

static struct kunit_suite apple_pmc_test_suite = {
	.name = "apple-pmc",
	.test_cases = apple_pmc_test_cases,
};

kunit_test_suite(apple_pmc_test_suite);
