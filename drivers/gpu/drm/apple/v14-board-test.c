// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * The 14.x IOMFB board records: which record a DCP gets on which machine,
 * and which DCP firmware image a record admits. The J516C (T6031) record
 * admits only the T6030 image, whose IOMFB layouts were validated, and
 * refuses every other; the existing records are unchanged.
 */
#include <kunit/test.h>
#include <linux/string.h>

#include "iomfb_v14_7_board.h"

#define T6030_DCP_IMAGE	"DDF38191-93B3-324A-BC8F-643006F5AC82"
#define J613_DCP_IMAGE	"90F849E1-B422-367E-B389-50246F8DEC47"
#define J516C_DCP_IMAGE	"F493F03E-CC40-3D89-9A08-5F259BA2B246"

struct board_ctx {
	const char *dcp;
	const char *const *machine;	/* NULL-terminated root compatibles */
};

static bool board_ctx_dcp_is(const void *ctx, const char *compat)
{
	const struct board_ctx *c = ctx;

	return !strcmp(c->dcp, compat);
}

static bool board_ctx_machine_is(const void *ctx, const char *compat)
{
	const struct board_ctx *c = ctx;
	const char *const *m;

	for (m = c->machine; *m; m++)
		if (!strcmp(*m, compat))
			return true;
	return false;
}

static const struct dcp_v14_board *board_for(const char *dcp, const char *const *machine,
					     bool *known)
{
	const struct board_ctx ctx = { .dcp = dcp, .machine = machine };

	return dcp_v14_board_select(board_ctx_dcp_is, board_ctx_machine_is, &ctx, known);
}

static const char *const j516c[] = { "apple,j516c", "apple,t6031", "apple,arm-platform", NULL };
static const char *const j514c[] = { "apple,j514c", "apple,t6031", "apple,arm-platform", NULL };
static const char *const j516m[] = { "apple,j516m", "apple,t6034", "apple,arm-platform", NULL };
static const char *const j516s[] = { "apple,j516s", "apple,t6030", "apple,arm-platform", NULL };
static const char *const j613[] = { "apple,j613", "apple,t8122", "apple,arm-platform", NULL };

static void v14_board_j516c(struct kunit *test)
{
	const struct dcp_v14_board *b;
	bool known;

	b = board_for("apple,t6031-dcp", j516c, &known);
	KUNIT_ASSERT_NOT_NULL(test, b);
	KUNIT_EXPECT_TRUE(test, known);
	KUNIT_EXPECT_STREQ(test, b->name, "J516C");
	KUNIT_EXPECT_STREQ(test, b->debugfs, "dcp-j516c");
	KUNIT_EXPECT_STREQ(test, b->machine, "apple,j516c");
	KUNIT_EXPECT_STREQ(test, b->handoff, "apple,t6031-handoff");
	KUNIT_EXPECT_STREQ(test, b->firmware_uuid, J516C_DCP_IMAGE);
	KUNIT_EXPECT_EQ(test, b->panel_width, 3456);
	KUNIT_EXPECT_EQ(test, b->panel_height, 2234);
	KUNIT_EXPECT_TRUE(test, b->promotion);
	KUNIT_EXPECT_TRUE(test, b->log_bringup);
	/* No colour matrix method is known for this board. */
	KUNIT_EXPECT_EQ(test, b->ctm_set, 0);
	KUNIT_EXPECT_EQ(test, b->ctm_get, 0);
}

/*
 * The J514C: the same T6031 DCP and image, the panel from the boot
 * framebuffer, the first boots logged.
 */
static void v14_board_j514c(struct kunit *test)
{
	const struct dcp_v14_board *b;
	bool known;

	b = board_for("apple,t6031-dcp", j514c, &known);
	KUNIT_ASSERT_NOT_NULL(test, b);
	KUNIT_EXPECT_TRUE(test, known);
	KUNIT_EXPECT_STREQ(test, b->name, "J514C");
	KUNIT_EXPECT_STREQ(test, b->debugfs, "dcp-j514c");
	KUNIT_EXPECT_STREQ(test, b->machine, "apple,j514c");
	KUNIT_EXPECT_STREQ(test, b->handoff, "apple,t6031-handoff");
	KUNIT_EXPECT_STREQ(test, b->firmware_uuid, J516C_DCP_IMAGE);
	KUNIT_EXPECT_EQ(test, b->panel_width, 0);
	KUNIT_EXPECT_EQ(test, b->panel_height, 0);
	KUNIT_EXPECT_TRUE(test, b->promotion);
	KUNIT_EXPECT_TRUE(test, b->log_bringup);
	KUNIT_EXPECT_EQ(test, b->ctm_set, 0);
	KUNIT_EXPECT_TRUE(test, dcp_v14_board_admits(b, J516C_DCP_IMAGE));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, T6030_DCP_IMAGE));
	/* Not the J516C record. */
	KUNIT_EXPECT_PTR_NE(test, b, board_for("apple,t6031-dcp", j516c, &known));
}

/* A T6031 DCP on a T6034 has no record: refused. */
static void v14_board_t6031_other_boards(struct kunit *test)
{
	bool known;

	KUNIT_EXPECT_NULL(test, board_for("apple,t6031-dcp", j516m, &known));
	KUNIT_EXPECT_TRUE(test, known);
	/* Not a 14.x internal DCP at all. */
	KUNIT_EXPECT_NULL(test, board_for("apple,t8112-dcp", j516c, &known));
	KUNIT_EXPECT_FALSE(test, known);
}

/* The DCP image the record admits: this Mac's own, exactly. */
static void v14_board_j516c_image(struct kunit *test)
{
	const struct dcp_v14_board *b;
	bool known;

	b = board_for("apple,t6031-dcp", j516c, &known);
	KUNIT_ASSERT_NOT_NULL(test, b);
	KUNIT_EXPECT_TRUE(test, dcp_v14_board_admits(b, J516C_DCP_IMAGE));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, NULL));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, ""));
	/* The other 14.7 DCP images are not this Mac's. */
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, T6030_DCP_IMAGE));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, J613_DCP_IMAGE));
	/* This Mac's PMP image is not a DCP image. */
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, "07FF4C5B-CB2E-3DE6-95FF-3B107D19B7AB"));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, "f493f03e-cc40-3d89-9a08-5f259ba2b246"));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, J516C_DCP_IMAGE " "));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, "F493F03E-CC40-3D89-9A08-5F259BA2B24"));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, "00000000-0000-0000-0000-000000000000"));
}

/* The records that were there before are what they were. */
static void v14_board_unchanged(struct kunit *test)
{
	const struct dcp_v14_board *b;
	bool known;

	b = board_for("apple,t6030-dcp", j516s, &known);
	KUNIT_ASSERT_NOT_NULL(test, b);
	KUNIT_EXPECT_STREQ(test, b->name, "T6030");
	KUNIT_EXPECT_STREQ(test, b->debugfs, "dcp-t6030");
	KUNIT_EXPECT_NULL(test, b->machine);
	KUNIT_EXPECT_STREQ(test, b->handoff, "apple,t6030-handoff");
	KUNIT_EXPECT_STREQ(test, b->firmware_uuid, T6030_DCP_IMAGE);
	KUNIT_EXPECT_EQ(test, b->panel_width, 0);
	KUNIT_EXPECT_EQ(test, b->panel_height, 0);
	KUNIT_EXPECT_TRUE(test, b->promotion);
	KUNIT_EXPECT_FALSE(test, b->log_bringup);
	KUNIT_EXPECT_EQ(test, b->ctm_set, 0);
	KUNIT_EXPECT_TRUE(test, dcp_v14_board_admits(b, T6030_DCP_IMAGE));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, J613_DCP_IMAGE));
	/* The T6030 record serves a T6030 DCP on any machine, as before. */
	KUNIT_EXPECT_PTR_EQ(test, board_for("apple,t6030-dcp", j516c, &known), b);

	b = board_for("apple,t8122-dcp", j613, &known);
	KUNIT_ASSERT_NOT_NULL(test, b);
	KUNIT_EXPECT_STREQ(test, b->name, "J613");
	KUNIT_EXPECT_STREQ(test, b->handoff, "apple,t8122-handoff");
	KUNIT_EXPECT_STREQ(test, b->firmware_uuid, J613_DCP_IMAGE);
	KUNIT_EXPECT_EQ(test, b->panel_width, 2560);
	KUNIT_EXPECT_EQ(test, b->panel_height, 1664);
	KUNIT_EXPECT_FALSE(test, b->promotion);
	KUNIT_EXPECT_FALSE(test, b->log_bringup);
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, T6030_DCP_IMAGE));

	/* A T8122 DCP on a T6031 machine has no record. */
	KUNIT_EXPECT_NULL(test, board_for("apple,t8122-dcp", j516c, &known));
	KUNIT_EXPECT_TRUE(test, known);
}

static struct kunit_case v14_board_cases[] = {
	KUNIT_CASE(v14_board_j516c),
	KUNIT_CASE(v14_board_j514c),
	KUNIT_CASE(v14_board_t6031_other_boards),
	KUNIT_CASE(v14_board_j516c_image),
	KUNIT_CASE(v14_board_unchanged),
	{}
};

static struct kunit_suite v14_board_suite = {
	.name = "apple-dcp-v14-board",
	.test_cases = v14_board_cases,
};

kunit_test_suite(v14_board_suite);
