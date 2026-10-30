// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * The 14.x IOMFB board records: which record a DCP gets on which machine,
 * and which DCP firmware image a record admits. The J516C (T6031) record
 * admits only the T6030 image, whose IOMFB layouts were validated, and
 * refuses every other; the existing records are unchanged. Which DCPs stamp
 * their swaps: the internal panel of a T6031 record; the T6030 and T8122
 * panels and external processors keep zero timestamps, byte for byte.
 */
#include <kunit/test.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "iomfb_v14_7_board.h"
#include "iomfb_v14_7_swap.h"

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
	KUNIT_EXPECT_TRUE(test, b->base_edid);
	/* DPMS off only blanks its panel. */
	KUNIT_EXPECT_FALSE(test, b->panel_hw_off);
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
	/* Its panel is not the J516C's: no base EDID. */
	KUNIT_EXPECT_FALSE(test, b->base_edid);
	KUNIT_EXPECT_FALSE(test, b->panel_hw_off);
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
	KUNIT_EXPECT_FALSE(test, b->base_edid);
	/* The J514S/J516S panel, on which DPMS off stops the display controller. */
	KUNIT_EXPECT_TRUE(test, b->panel_hw_off);
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
	KUNIT_EXPECT_FALSE(test, b->base_edid);
	KUNIT_EXPECT_FALSE(test, b->panel_hw_off);
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_admits(b, T6030_DCP_IMAGE));

	/* A T8122 DCP on a T6031 machine has no record. */
	KUNIT_EXPECT_NULL(test, board_for("apple,t8122-dcp", j516c, &known));
	KUNIT_EXPECT_TRUE(test, known);
}

/* Only the internal panel of the J516C record stamps its swaps. */
static void v14_board_swap_timestamps(struct kunit *test)
{
	const struct dcp_v14_board *j516c_b, *j514c_b, *t6030_b, *j613_b;
	bool known;

	j516c_b = board_for("apple,t6031-dcp", j516c, &known);
	j514c_b = board_for("apple,t6031-dcp", j514c, &known);
	t6030_b = board_for("apple,t6030-dcp", j516s, &known);
	j613_b = board_for("apple,t8122-dcp", j613, &known);
	KUNIT_ASSERT_TRUE(test, j516c_b && j514c_b && t6030_b && j613_b);

	KUNIT_EXPECT_TRUE(test, j516c_b->swap_timestamps);
	KUNIT_EXPECT_FALSE(test, j514c_b->swap_timestamps);
	KUNIT_EXPECT_FALSE(test, t6030_b->swap_timestamps);
	KUNIT_EXPECT_FALSE(test, j613_b->swap_timestamps);

	KUNIT_EXPECT_TRUE(test, dcp_v14_board_stamps_swaps(j516c_b, false));
	/* The J514C, the external role, the T6030 (J514S, J516S) and T8122 panels, no record. */
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_stamps_swaps(j514c_b, false));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_stamps_swaps(j516c_b, true));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_stamps_swaps(t6030_b, false));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_stamps_swaps(j613_b, false));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_stamps_swaps(NULL, false));
	KUNIT_EXPECT_FALSE(test, dcp_v14_board_stamps_swaps(NULL, true));
}

/*
 * A stamped swap holds 120 in timestamp slots 0, 1 and 6 and is otherwise the
 * encoded record, with or without a surface; an unstamped one keeps every
 * slot zero.
 */
static void v14_swap_timestamps_encoding(struct kunit *test)
{
	const size_t end = DCP_V14_SWAP_TIMESTAMP(DCP_V14_SWAP_TIMESTAMPS);
	u8 surface[DCP_V14_SURFACE_SIZE];
	unsigned int i, pass;
	u8 *plain, *stamped;

	plain = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);
	stamped = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_TRUE(test, plain && stamped);
	dcp_v14_encode_surface(surface, 3456 * 4, 3456, 2160, true);

	for (pass = 0; pass < 2; pass++) {
		const u8 *s = pass ? surface : NULL;

		dcp_v14_encode_swap(plain, 0x2a, 0xff000000, s, 0x10000, pass ? 3456 : 0,
				    pass ? 2160 : 0, 74, 0);
		dcp_v14_encode_swap(stamped, 0x2a, 0xff000000, s, 0x10000, pass ? 3456 : 0,
				    pass ? 2160 : 0, 74, 0);
		dcp_v14_swap_timestamps(stamped);

		for (i = 0; i < DCP_V14_SWAP_TIMESTAMPS; i++) {
			size_t at = DCP_V14_SWAP_TIMESTAMP(i);
			u64 want = i == 0 || i == 1 || i == 6 ? 120 : 0;

			KUNIT_EXPECT_EQ_MSG(test, get_unaligned_le64(plain + at), 0,
					    "surface %u: unstamped slot %u", pass, i);
			KUNIT_EXPECT_EQ_MSG(test, get_unaligned_le64(stamped + at), want,
					    "surface %u: slot %u", pass, i);
		}
		/* The rest of the record, from the word after slot 6, is unchanged. */
		KUNIT_EXPECT_EQ(test,
				memcmp(plain + end, stamped + end, DCP_V14_SWAP_SIZE - end), 0);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(stamped + DCP_V14_SWAP_ID), 0x2a);
	}
}

static struct kunit_case v14_board_cases[] = {
	KUNIT_CASE(v14_board_j516c),
	KUNIT_CASE(v14_board_j514c),
	KUNIT_CASE(v14_board_t6031_other_boards),
	KUNIT_CASE(v14_board_j516c_image),
	KUNIT_CASE(v14_board_unchanged),
	KUNIT_CASE(v14_board_swap_timestamps),
	KUNIT_CASE(v14_swap_timestamps_encoding),
	{}
};

static struct kunit_suite v14_board_suite = {
	.name = "apple-dcp-v14-board",
	.test_cases = v14_board_cases,
};

kunit_test_suite(v14_board_suite);
