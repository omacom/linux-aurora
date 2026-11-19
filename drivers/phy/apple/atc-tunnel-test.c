// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
#include <kunit/test.h>
#include <linux/soc/apple/dp-tunnel.h>

#include "atc-tunnel.h"

static void atc_tunnel_soc_qualification(struct kunit *test)
{
	const struct {
		const char *soc;
		bool qualified;
	} cases[] = {
		{ "apple,t6020", true },
		{ "apple,t6021", true },
		{ "apple,t6022", true },
		{ "apple,t8103", false },
		{ "apple,t6000", false },
		{ "apple,t6001", false },
		{ "apple,t6002", false },
		{ "apple,t8112", false },
		{ "apple,t6030", false },
		{ "apple,t6031", false },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		struct property compatible = {
			.name = "compatible", .value = (void *)cases[i].soc,
			.length = strlen(cases[i].soc) + 1,
		};
		struct device_node root = { .properties = &compatible };

		KUNIT_EXPECT_EQ(test, apple_atc_t602x_qualified(&root), cases[i].qualified);
	}
	KUNIT_EXPECT_FALSE(test, apple_atc_t602x_qualified(NULL));
}

static void atc_tunnel_core_qualification(struct kunit *test)
{
	const struct {
		const char *compatible;
		u64 base;
		u64 size;
		bool valid;
	} cases[] = {
		{ "apple,t6020-atcphy", 0x703000000ULL, 0x7048, true },
		{ "apple,t6020-atcphy", 0xb03000000ULL, 0x7048, true },
		{ "apple,t6020-atcphy", 0xf03000000ULL, 0x7048, true },
		{ "apple,t6020-atcphy", 0x703000000ULL, 0x7047, false },
		{ "apple,t6020-atcphy", 0x1303000000ULL, 0x4c000, true },
		{ "apple,t6020-atcphy", 0x2703000000ULL, 0x4c000, false },
		{ "apple,t8103-atcphy", 0x703000000ULL, 0x4c000, false },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		struct property compatible = {
			.name = "compatible", .value = (void *)cases[i].compatible,
			.length = strlen(cases[i].compatible) + 1,
		};
		struct device_node np = { .properties = &compatible };

		KUNIT_EXPECT_EQ(test,
				apple_atc_t602x_core_valid(&np, cases[i].base, cases[i].size),
				cases[i].valid);
	}
	KUNIT_EXPECT_FALSE(test, apple_atc_t602x_core_valid(NULL, 0x703000000ULL, 0x7048));
}

static void atc_tunnel_wiring_gate(struct kunit *test)
{
	const struct {
		bool own;
		bool present;
		bool valid;
		int expected;
	} cases[] = {
		{ true, true, true, 0 },
		{ false, true, true, -EOPNOTSUPP },
		{ false, true, false, -EINVAL },
		{ true, true, false, -EINVAL },
		{ false, false, true, -EOPNOTSUPP },
		{ false, false, false, -EOPNOTSUPP },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++)
		KUNIT_EXPECT_EQ(test,
				apple_atc_t602x_gate(cases[i].own, cases[i].present,
						     cases[i].valid),
				cases[i].expected);
}

static void atc_tunnel_t6030_clock_sources(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, apple_dp_tunnel_t6030_pclk(0, 0), 0);
	KUNIT_EXPECT_EQ(test, apple_dp_tunnel_t6030_pclk(2, 0), 0);
	KUNIT_EXPECT_EQ(test, apple_dp_tunnel_t6030_pclk(0, 1), 1);
	KUNIT_EXPECT_EQ(test, apple_dp_tunnel_t6030_pclk(2, 1), 1);
	KUNIT_EXPECT_EQ(test, apple_dp_tunnel_t6030_pclk(1, 0), -EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, apple_dp_tunnel_t6030_pclk(4, 0), -EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, apple_dp_tunnel_t6030_pclk(0, 2), -EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, apple_dp_tunnel_t6030_pclk(~0U, 0), -EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, apple_dp_tunnel_t6030_pclk(0, ~0U), -EOPNOTSUPP);
}

static struct kunit_case atc_tunnel_cases[] = {
	KUNIT_CASE(atc_tunnel_t6030_clock_sources),
	KUNIT_CASE(atc_tunnel_soc_qualification),
	KUNIT_CASE(atc_tunnel_core_qualification),
	KUNIT_CASE(atc_tunnel_wiring_gate),
	{},
};

static struct kunit_suite atc_tunnel_suite = {
	.name = "apple-atc-tunnel",
	.test_cases = atc_tunnel_cases,
};

kunit_test_suite(atc_tunnel_suite);
