// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * The 14.7 panel level path, included at the end of iomfb_v14_7.c so that
 * the production functions run: the swap's level block, which boards carry
 * it, the gate that keeps brightness-only presents out of system sleep and
 * after unbind, and the policy as this panel uses it (no level presented
 * until one is requested, no actual level reported until one was presented,
 * a refused level never becomes the actual one).
 *
 * The objects are built here, with no DCP: nothing in a case may reach an
 * RPC. A J613, J615 or J516C panel never initialises the backlight policy;
 * its state is set to answer "yes" so that a case fails if anything on
 * those boards looks at it.
 */
#include <kunit/device.h>
#include <kunit/test.h>

static void v14_bl_unused_work(struct work_struct *work)
{
}

static void v14_bl_binary64(struct kunit *test)
{
	static const struct {
		u32 nits;
		u64 bits;
	} known[] = {
		{ 0, 0 },
		{ 1, 0x3ff0000000000000ULL },
		{ 2, 0x4000000000000000ULL },
		{ 3, 0x4008000000000000ULL },
		{ 140, 0x4061800000000000ULL },
		{ 255, 0x406fe00000000000ULL },
		{ 256, 0x4070000000000000ULL },
		{ 500, 0x407f400000000000ULL },
	};
	unsigned int i;
	u32 nits;

	for (i = 0; i < ARRAY_SIZE(known); i++)
		KUNIT_EXPECT_EQ_MSG(test, dcp_v14_nits_binary64(known[i].nits), known[i].bits,
				    "%u nits", known[i].nits);
	/* Every level up to the bound is an exact integral binary64. */
	for (nits = 1; nits <= DCP_V14_NITS_MAX; nits++) {
		u64 bits = dcp_v14_nits_binary64(nits);
		unsigned int e = (bits >> 52) - 1023;
		u64 mantissa = bits & GENMASK_ULL(51, 0);

		KUNIT_ASSERT_LT(test, e, 32U);
		KUNIT_EXPECT_EQ(test, mantissa & GENMASK_ULL(51 - e, 0), 0ULL);
		KUNIT_EXPECT_EQ(test, (1ULL << e) + (mantissa >> (52 - e)), (u64)nits);
		KUNIT_EXPECT_FALSE(test, bits >> 63);
	}
}

static void v14_bl_level_block(struct kunit *test)
{
	u8 *swap = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);
	u8 *before = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, swap);
	KUNIT_ASSERT_NOT_NULL(test, before);
	memset(swap, 0xa5, DCP_V14_SWAP_SIZE);
	memcpy(before, swap, DCP_V14_SWAP_SIZE);

	/* Above the wire bound: refused, the record untouched. */
	KUNIT_EXPECT_EQ(test, dcp_v14_encode_nits(swap, true, DCP_V14_NITS_MAX + 1), -EINVAL);
	KUNIT_EXPECT_EQ(test, memcmp(swap, before, DCP_V14_SWAP_SIZE), 0);

	/* A level touches only the present byte and the binary64. */
	KUNIT_EXPECT_EQ(test, dcp_v14_encode_nits(swap, true, 140), 0);
	for (i = 0; i < DCP_V14_SWAP_SIZE; i++) {
		if (i == DCP_V14_SWAP_NITS_PRESENT ||
		    (i >= DCP_V14_SWAP_NITS && i < DCP_V14_SWAP_NITS + 8))
			continue;
		KUNIT_EXPECT_EQ_MSG(test, swap[i], before[i], "byte %#x", i);
	}
	KUNIT_EXPECT_EQ(test, swap[DCP_V14_SWAP_NITS_PRESENT], 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(swap + DCP_V14_SWAP_NITS),
			0x4061800000000000ULL);

	/* No level: zero, as the record the session sent before. */
	KUNIT_EXPECT_EQ(test, dcp_v14_encode_nits(swap, false, 0), 0);
	KUNIT_EXPECT_EQ(test, swap[DCP_V14_SWAP_NITS_PRESENT], 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(swap + DCP_V14_SWAP_NITS), 0ULL);
}

/* A swap without a level is byte for byte the one encoded before this. */
static void v14_bl_swap_unchanged_without_level(struct kunit *test)
{
	u8 *swap = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);
	u8 *plain = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);
	u8 surface[DCP_V14_SURFACE_SIZE];

	KUNIT_ASSERT_NOT_NULL(test, swap);
	KUNIT_ASSERT_NOT_NULL(test, plain);
	dcp_v14_encode_surface(surface, 3456 * 4, 3456, 2160, true);
	dcp_v14_encode_swap(plain, 7, DCP_V14_BLACK, surface, 0x100000000ULL, 3456, 2160, 74,
			    DCP_V14_LAYER);
	dcp_v14_encode_swap(swap, 7, DCP_V14_BLACK, surface, 0x100000000ULL, 3456, 2160, 74,
			    DCP_V14_LAYER);
	dcp_v14_encode_nits(swap, false, 0);
	KUNIT_EXPECT_EQ(test, memcmp(swap, plain, DCP_V14_SWAP_SIZE), 0);
	/* The level block lies in the zeroed prefix, clear of every field. */
	KUNIT_EXPECT_EQ(test, plain[DCP_V14_SWAP_NITS_PRESENT], 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(plain + DCP_V14_SWAP_NITS), 0ULL);

	dcp_v14_encode_swap(plain, 8, DCP_V14_BLACK, NULL, 0, 0, 0, 0, DCP_V14_LAYER);
	dcp_v14_encode_swap(swap, 8, DCP_V14_BLACK, NULL, 0, 0, 0, 0, DCP_V14_LAYER);
	dcp_v14_encode_nits(swap, false, 0);
	KUNIT_EXPECT_EQ(test, memcmp(swap, plain, DCP_V14_SWAP_SIZE), 0);
}

static bool v14_bl_ctx_dcp_is(const void *ctx, const char *compat)
{
	return !strcmp(((const char *const *)ctx)[0], compat);
}

static bool v14_bl_ctx_machine_is(const void *ctx, const char *compat)
{
	return !strcmp(((const char *const *)ctx)[1], compat);
}

/* Only the T6030 record (J514S, J516S) carries levels. */
static void v14_bl_boards(struct kunit *test)
{
	static const char *const t6030_j516s[] = { "apple,t6030-dcp", "apple,j516s" };
	static const char *const t6030_j514s[] = { "apple,t6030-dcp", "apple,j514s" };
	static const char *const j613[] = { "apple,t8122-dcp", "apple,j613" };
	static const char *const j615[] = { "apple,t8122-dcp", "apple,j615" };
	static const char *const j516c[] = { "apple,t6031-dcp", "apple,j516c" };
	static const char *const other_t6030[] = { "apple,t6030-dcp", "apple,other" };
	const struct dcp_v14_board *board;
	bool known;

	board = dcp_v14_board_select(v14_bl_ctx_dcp_is, v14_bl_ctx_machine_is, t6030_j516s, &known);
	KUNIT_ASSERT_NOT_NULL(test, board);
	KUNIT_EXPECT_TRUE(test, board->panel_nits);
	KUNIT_EXPECT_TRUE(test, dcp_v14_panel_nits_admit(board, v14_bl_ctx_machine_is,
							 t6030_j516s));
	board = dcp_v14_board_select(v14_bl_ctx_dcp_is, v14_bl_ctx_machine_is, t6030_j514s, &known);
	KUNIT_ASSERT_NOT_NULL(test, board);
	KUNIT_EXPECT_TRUE(test, board->panel_nits);
	KUNIT_EXPECT_TRUE(test, dcp_v14_panel_nits_admit(board, v14_bl_ctx_machine_is,
							 t6030_j514s));
	board = dcp_v14_board_select(v14_bl_ctx_dcp_is, v14_bl_ctx_machine_is, other_t6030, &known);
	KUNIT_ASSERT_NOT_NULL(test, board);
	KUNIT_EXPECT_FALSE(test, dcp_v14_panel_nits_admit(board, v14_bl_ctx_machine_is,
							  other_t6030));
	board = dcp_v14_board_select(v14_bl_ctx_dcp_is, v14_bl_ctx_machine_is, j613, &known);
	KUNIT_ASSERT_NOT_NULL(test, board);
	KUNIT_EXPECT_FALSE(test, board->panel_nits);
	board = dcp_v14_board_select(v14_bl_ctx_dcp_is, v14_bl_ctx_machine_is, j615, &known);
	KUNIT_ASSERT_NOT_NULL(test, board);
	KUNIT_EXPECT_FALSE(test, board->panel_nits);
	board = dcp_v14_board_select(v14_bl_ctx_dcp_is, v14_bl_ctx_machine_is, j516c, &known);
	KUNIT_ASSERT_NOT_NULL(test, board);
	KUNIT_EXPECT_FALSE(test, board->panel_nits);
}

static void v14_bl_gate_transitions(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, dcp_v14_bl_may_queue(DCP_V14_BL_OPEN));
	KUNIT_EXPECT_FALSE(test, dcp_v14_bl_may_queue(DCP_V14_BL_SUSPENDED));
	KUNIT_EXPECT_FALSE(test, dcp_v14_bl_may_queue(DCP_V14_BL_CLOSED));
	KUNIT_EXPECT_EQ(test, dcp_v14_bl_gate_suspend(DCP_V14_BL_OPEN), DCP_V14_BL_SUSPENDED);
	KUNIT_EXPECT_EQ(test, dcp_v14_bl_gate_suspend(DCP_V14_BL_SUSPENDED), DCP_V14_BL_SUSPENDED);
	KUNIT_EXPECT_EQ(test, dcp_v14_bl_gate_suspend(DCP_V14_BL_CLOSED), DCP_V14_BL_CLOSED);
	KUNIT_EXPECT_EQ(test, dcp_v14_bl_gate_resume(DCP_V14_BL_SUSPENDED), DCP_V14_BL_OPEN);
	KUNIT_EXPECT_EQ(test, dcp_v14_bl_gate_resume(DCP_V14_BL_OPEN), DCP_V14_BL_OPEN);
	/* Unbind is for good: no resume opens it again. */
	KUNIT_EXPECT_EQ(test, dcp_v14_bl_gate_resume(DCP_V14_BL_CLOSED), DCP_V14_BL_CLOSED);
}

static void v14_bl_plan(struct kunit *test)
{
	const struct dcp_v14_bl_facts ready = {
		.gate = DCP_V14_BL_OPEN, .panel_nits = true, .started = true,
		.pending = true, .frame = true, .mode_valid = true,
	};
	struct dcp_v14_bl_facts f;

	KUNIT_EXPECT_EQ(test, dcp_v14_bl_plan(&ready), DCP_V14_BL_REPRESENT);
#define V14_BL_EXPECT(field, value, action) do {			\
		f = ready;						\
		f.field = value;					\
		KUNIT_EXPECT_EQ_MSG(test, dcp_v14_bl_plan(&f), action,	\
				    #field " = " #value);		\
	} while (0)
	V14_BL_EXPECT(gate, DCP_V14_BL_SUSPENDED, DCP_V14_BL_SKIP);
	V14_BL_EXPECT(gate, DCP_V14_BL_CLOSED, DCP_V14_BL_SKIP);
	V14_BL_EXPECT(panel_nits, false, DCP_V14_BL_SKIP);
	V14_BL_EXPECT(external, true, DCP_V14_BL_SKIP);
	V14_BL_EXPECT(typec, true, DCP_V14_BL_SKIP);
	V14_BL_EXPECT(started, false, DCP_V14_BL_SKIP);
	V14_BL_EXPECT(failed, true, DCP_V14_BL_SKIP);
	V14_BL_EXPECT(crashed, true, DCP_V14_BL_SKIP);
	V14_BL_EXPECT(quiescing, true, DCP_V14_BL_SKIP);
	V14_BL_EXPECT(pending, false, DCP_V14_BL_SKIP);
	V14_BL_EXPECT(frame, false, DCP_V14_BL_WAIT);
	V14_BL_EXPECT(mode_valid, false, DCP_V14_BL_WAIT);
#undef V14_BL_EXPECT
	KUNIT_EXPECT_TRUE(test, dcp_v14_swap_takes_nits(true, false, false, true));
	KUNIT_EXPECT_FALSE(test, dcp_v14_swap_takes_nits(false, false, false, true));
	KUNIT_EXPECT_FALSE(test, dcp_v14_swap_takes_nits(true, true, false, true));
	KUNIT_EXPECT_FALSE(test, dcp_v14_swap_takes_nits(true, false, true, true));
	KUNIT_EXPECT_FALSE(test, dcp_v14_swap_takes_nits(true, false, false, false));
}

/* The policy as this panel configures it: a default that is never presented. */
static void v14_bl_policy(struct kunit *test)
{
	struct dcp_backlight_state s;
	struct dcp_backlight_present p;
	u64 seq;

	KUNIT_ASSERT_EQ(test, dcp_bl_init(&s, DCP_V14_NITS_MAX, false, 0, true,
					  DCP_V14_NITS_DEFAULT), 0);
	/* Nothing to present at takeover: the panel keeps the loader's level. */
	KUNIT_EXPECT_EQ(test, dcp_bl_prepare(&s, true, &p), -EALREADY);
	KUNIT_EXPECT_EQ(test, dcp_bl_reported_actual(&s, true), -ENODATA);
	/* The H17P path keeps reporting its default. */
	KUNIT_EXPECT_EQ(test, dcp_bl_reported_actual(&s, false), DCP_V14_NITS_DEFAULT);
	KUNIT_EXPECT_FALSE(test, dcp_v14_bl_follow_dpms(true, true, s.level_known));

	/* A request is presented; a refusal never becomes the actual level. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&s, 250, false, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&s, true, &p), 0);
	KUNIT_EXPECT_EQ(test, p.nits, 250U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&s, p.sequence, false));
	KUNIT_EXPECT_EQ(test, dcp_bl_reported_actual(&s, true), -ENODATA);
	KUNIT_EXPECT_TRUE(test, s.dirty);
	KUNIT_EXPECT_EQ(test, dcp_bl_retry_delay(&s), 100U);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&s, true, &p), 0);
	seq = p.sequence;
	/* A stale completion does not complete the newer reservation. */
	KUNIT_EXPECT_FALSE(test, dcp_bl_complete(&s, seq - 1, true));
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&s, seq, true));
	KUNIT_EXPECT_EQ(test, dcp_bl_reported_actual(&s, true), 250);
	KUNIT_EXPECT_TRUE(test, dcp_v14_bl_follow_dpms(true, true, s.level_known));

	/* A later refusal keeps the last accepted level. */
	KUNIT_ASSERT_EQ(test, dcp_bl_request(&s, 300, false, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&s, true, &p), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&s, p.sequence, false));
	KUNIT_EXPECT_EQ(test, dcp_bl_reported_actual(&s, true), 250);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&s, true, &p), 0);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&s, p.sequence, true));

	/* DPMS off blanks at 0 nits, on presents the level again. */
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&s, false), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&s, true, &p), 0);
	KUNIT_EXPECT_EQ(test, p.nits, 0U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&s, p.sequence, true));
	KUNIT_ASSERT_EQ(test, dcp_bl_dpms(&s, true), 0);
	KUNIT_ASSERT_EQ(test, dcp_bl_prepare(&s, true, &p), 0);
	KUNIT_EXPECT_EQ(test, p.nits, 300U);
	KUNIT_EXPECT_TRUE(test, dcp_bl_complete(&s, p.sequence, true));
}

struct v14_bl_fixture {
	struct apple_dcp *dcp;
	struct apple_dcp_v14 *v14;
};

static void v14_bl_fixture_stop(void *data)
{
	struct apple_dcp_v14 *v14 = data;

	cancel_delayed_work_sync(&v14->backlight_work);
}

/*
 * An internal 14.7 panel on @board, started, with no DCP behind it. With
 * @policy, the T6030 panel's policy as dcp_v14_configure_backlight() sets it
 * up; without, a policy that was never initialised (other boards), whose
 * state would answer "yes" if anything looked at it.
 */
static struct v14_bl_fixture v14_bl_fixture(struct kunit *test,
					    const struct dcp_v14_board *board, bool policy)
{
	static atomic_t serial = ATOMIC_INIT(0);
	struct v14_bl_fixture f;
	struct device *dev;
	char name[32];

	snprintf(name, sizeof(name), "dcp-v14-bl.%d", atomic_inc_return(&serial));
	dev = kunit_device_register(test, name);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	/* The device's backlight cleanup must run before these objects are freed. */
	f.dcp = devm_kzalloc(dev, sizeof(*f.dcp), GFP_KERNEL);
	f.v14 = devm_kzalloc(dev, sizeof(*f.v14), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, f.dcp);
	KUNIT_ASSERT_NOT_NULL(test, f.v14);
	f.dcp->dev = dev;
	spin_lock_init(&f.dcp->mode_state.lock);
	mutex_init(&f.dcp->bl_register_mutex);
	INIT_WORK(&f.dcp->bl_register_wq, dcp_work_register_backlight);
	INIT_WORK(&f.dcp->bl_update_wq, v14_bl_unused_work);
	INIT_DELAYED_WORK(&f.dcp->swap_watchdog_wq, v14_bl_unused_work);
	f.dcp->fw_compat = DCP_FIRMWARE_V_14_7;
	f.dcp->v14 = f.v14;
	f.v14->dev = dev;
	f.v14->board = board;
	f.v14->panel_nits = board && board->panel_nits;
	f.v14->dcp = f.dcp;
	f.v14->started = true;
	mutex_init(&f.v14->lock);
	INIT_DELAYED_WORK(&f.v14->backlight_work, dcp_v14_backlight_work);
	spin_lock_init(&f.v14->bl_gate_lock);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, v14_bl_fixture_stop, f.v14), 0);
	if (policy) {
		KUNIT_ASSERT_EQ(test, dcp_backlight_init(f.dcp, dev), 0);
		KUNIT_ASSERT_EQ(test, dcp_bl_init(&f.dcp->backlight.state, DCP_V14_NITS_MAX,
						  false, 0, true, DCP_V14_NITS_DEFAULT), 0);
		f.dcp->backlight.kick = dcp_v14_backlight_kick;
	} else {
		f.dcp->backlight.state.ready = true;
		f.dcp->backlight.state.level_known = true;
		f.dcp->backlight.state.dirty = true;
	}
	return f;
}

/* Through the policy's own entry point, under its lock. */
static void v14_bl_request(struct kunit *test, struct apple_dcp *dcp, u32 nits)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dcp->backlight.lock, flags);
	ret = dcp_bl_request(&dcp->backlight.state, nits, false, false);
	spin_unlock_irqrestore(&dcp->backlight.lock, flags);
	KUNIT_ASSERT_EQ(test, ret, 0);
}

/* Whether the work is queued or running; the caller holds lock. */
static bool v14_bl_queued(struct apple_dcp_v14 *v14)
{
	return work_busy(&v14->backlight_work.work);
}

/*
 * The production paths on J613, J615 and J516C: no policy is consulted, no
 * level is encoded, the work does nothing, sleep leaves the gate alone.
 */
static void v14_bl_other_boards_untouched(struct kunit *test)
{
	const struct dcp_v14_board *boards[] = {
		&dcp_v14_board_j613, &dcp_v14_board_j615, &dcp_v14_board_j516c,
		&dcp_v14_board_j514c,
	};
	struct dcp_backlight_present present;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(boards); i++) {
		struct v14_bl_fixture f = v14_bl_fixture(test, boards[i], false);
		u8 *swap = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);

		KUNIT_ASSERT_NOT_NULL(test, swap);
		KUNIT_EXPECT_FALSE(test, iomfb_v14_7_panel_nits(f.dcp));
		KUNIT_EXPECT_FALSE(test, dcp_backlight_active(f.dcp));
		KUNIT_EXPECT_FALSE(test, dcp_backlight_level_known(f.dcp));
		KUNIT_EXPECT_FALSE(test, dcp_v14_bl_dpms_applies(f.v14, f.dcp));

		mutex_lock(&f.v14->lock);
		KUNIT_EXPECT_FALSE(test, dcp_v14_swap_nits(f.v14, swap, &present));
		mutex_unlock(&f.v14->lock);
		KUNIT_EXPECT_EQ(test, swap[DCP_V14_SWAP_NITS_PRESENT], 0);
		KUNIT_EXPECT_EQ(test, get_unaligned_le64(swap + DCP_V14_SWAP_NITS), 0ULL);

		/* Run directly: it must return without an RPC (there is no DCP). */
		dcp_v14_backlight_work(&f.v14->backlight_work.work);
		KUNIT_EXPECT_EQ(test, f.v14->brightness_presents, 0ULL);
		KUNIT_EXPECT_EQ(test, f.v14->nits_refused, 0ULL);

		iomfb_v14_7_backlight_suspend(f.dcp);
		KUNIT_EXPECT_EQ(test, f.v14->bl_gate, DCP_V14_BL_OPEN);
		dcp_v14_configure_backlight(f.dcp);
		KUNIT_EXPECT_FALSE(test, f.dcp->backlight.kick);
	}
}

/* An external T6030 processor is no panel: nothing either. */
static void v14_bl_external_untouched(struct kunit *test)
{
	struct v14_bl_fixture f = v14_bl_fixture(test, &dcp_v14_board_t6030, false);
	struct dcp_backlight_present present;
	u8 *swap = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, swap);
	f.dcp->external = true;
	f.v14->external = true;
	KUNIT_EXPECT_FALSE(test, iomfb_v14_7_panel_nits(f.dcp));
	KUNIT_EXPECT_FALSE(test, dcp_backlight_level_known(f.dcp));
	KUNIT_EXPECT_FALSE(test, dcp_v14_bl_dpms_applies(f.v14, f.dcp));
	mutex_lock(&f.v14->lock);
	KUNIT_EXPECT_FALSE(test, dcp_v14_swap_nits(f.v14, swap, &present));
	mutex_unlock(&f.v14->lock);
	KUNIT_EXPECT_EQ(test, swap[DCP_V14_SWAP_NITS_PRESENT], 0);
	dcp_v14_backlight_work(&f.v14->backlight_work.work);
	KUNIT_EXPECT_EQ(test, f.v14->brightness_presents, 0ULL);
}

/*
 * The T6030 panel: no level until one is requested; the requested level is
 * encoded; a refusal keeps the actual level without a retry; an accepted one
 * is the actual level, and only then does blanking follow DPMS.
 */
static void v14_bl_t6030_swap_levels(struct kunit *test)
{
	struct v14_bl_fixture f = v14_bl_fixture(test, &dcp_v14_board_t6030, true);
	struct dcp_backlight_present present;
	u8 *swap = kunit_kzalloc(test, DCP_V14_SWAP_SIZE, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, swap);
	KUNIT_EXPECT_TRUE(test, iomfb_v14_7_panel_nits(f.dcp));
	KUNIT_EXPECT_TRUE(test, dcp_backlight_active(f.dcp));
	KUNIT_EXPECT_FALSE(test, dcp_backlight_level_known(f.dcp));
	KUNIT_EXPECT_FALSE(test, dcp_v14_bl_dpms_applies(f.v14, f.dcp));

	mutex_lock(&f.v14->lock);
	/* Nothing requested: the swap carries no level. */
	KUNIT_EXPECT_FALSE(test, dcp_v14_swap_nits(f.v14, swap, &present));
	KUNIT_EXPECT_EQ(test, swap[DCP_V14_SWAP_NITS_PRESENT], 0);
	mutex_unlock(&f.v14->lock);

	v14_bl_request(test, f.dcp, 250);
	mutex_lock(&f.v14->lock);
	KUNIT_EXPECT_TRUE(test, dcp_v14_swap_nits(f.v14, swap, &present));
	KUNIT_EXPECT_EQ(test, swap[DCP_V14_SWAP_NITS_PRESENT], 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(swap + DCP_V14_SWAP_NITS),
			dcp_v14_nits_binary64(250));
	/* Refused: no actual level, and the panel must not queue a retry. */
	dcp_v14_swap_nits_done(f.v14, &present, false);
	mutex_unlock(&f.v14->lock);
	KUNIT_EXPECT_FALSE(test, dcp_backlight_level_known(f.dcp));
	KUNIT_EXPECT_EQ(test, f.v14->nits_refused, 1ULL);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&f.v14->backlight_work));

	/* While a Type-C display has the pipe, no level either. */
	f.dcp->fixed_connector_type = DRM_MODE_CONNECTOR_USB;
	memset(swap, 0, DCP_V14_SWAP_SIZE);
	mutex_lock(&f.v14->lock);
	KUNIT_EXPECT_FALSE(test, dcp_v14_swap_nits(f.v14, swap, &present));
	mutex_unlock(&f.v14->lock);
	KUNIT_EXPECT_EQ(test, swap[DCP_V14_SWAP_NITS_PRESENT], 0);
	f.dcp->fixed_connector_type = DRM_MODE_CONNECTOR_eDP;

	mutex_lock(&f.v14->lock);
	KUNIT_EXPECT_TRUE(test, dcp_v14_swap_nits(f.v14, swap, &present));
	dcp_v14_swap_nits_done(f.v14, &present, true);
	mutex_unlock(&f.v14->lock);
	KUNIT_EXPECT_TRUE(test, dcp_backlight_level_known(f.dcp));
	KUNIT_EXPECT_EQ(test, f.v14->nits_sent, 250U);
	KUNIT_EXPECT_TRUE(test, dcp_v14_bl_dpms_applies(f.v14, f.dcp));

	/* Unchanged: not sent again. */
	mutex_lock(&f.v14->lock);
	KUNIT_EXPECT_FALSE(test, dcp_v14_swap_nits(f.v14, swap, &present));
	mutex_unlock(&f.v14->lock);
}

/*
 * The gate, through the production queue path: open, a kick queues the work;
 * suspended, nothing queues until resume; closed, nothing ever does. The
 * caller holds lock, so a queued work stays queued or running.
 */
static void v14_bl_t6030_gate(struct kunit *test)
{
	struct v14_bl_fixture f = v14_bl_fixture(test, &dcp_v14_board_t6030, true);

	mutex_lock(&f.v14->lock);
	dcp_v14_backlight_kick(f.dcp);
	KUNIT_EXPECT_TRUE(test, v14_bl_queued(f.v14));
	mutex_unlock(&f.v14->lock);
	cancel_delayed_work_sync(&f.v14->backlight_work);

	iomfb_v14_7_backlight_suspend(f.dcp);
	KUNIT_EXPECT_EQ(test, f.v14->bl_gate, DCP_V14_BL_SUSPENDED);
	mutex_lock(&f.v14->lock);
	dcp_v14_backlight_kick(f.dcp);
	dcp_v14_bl_queue(f.v14, 0);
	KUNIT_EXPECT_FALSE(test, v14_bl_queued(f.v14));
	mutex_unlock(&f.v14->lock);

	iomfb_v14_7_backlight_resume(f.dcp);
	KUNIT_EXPECT_EQ(test, f.v14->bl_gate, DCP_V14_BL_OPEN);
	mutex_lock(&f.v14->lock);
	dcp_v14_backlight_kick(f.dcp);
	KUNIT_EXPECT_TRUE(test, v14_bl_queued(f.v14));
	mutex_unlock(&f.v14->lock);
	cancel_delayed_work_sync(&f.v14->backlight_work);

	/* A quiescing DCP takes no kick either. */
	WRITE_ONCE(f.dcp->quiescing, true);
	mutex_lock(&f.v14->lock);
	dcp_v14_backlight_kick(f.dcp);
	KUNIT_EXPECT_FALSE(test, v14_bl_queued(f.v14));
	mutex_unlock(&f.v14->lock);
	WRITE_ONCE(f.dcp->quiescing, false);

	dcp_v14_bl_shut(f.v14, true);
	KUNIT_EXPECT_EQ(test, f.v14->bl_gate, DCP_V14_BL_CLOSED);
	iomfb_v14_7_backlight_resume(f.dcp);
	KUNIT_EXPECT_EQ(test, f.v14->bl_gate, DCP_V14_BL_CLOSED);
	mutex_lock(&f.v14->lock);
	dcp_v14_backlight_kick(f.dcp);
	dcp_v14_bl_queue(f.v14, 0);
	KUNIT_EXPECT_FALSE(test, v14_bl_queued(f.v14));
	mutex_unlock(&f.v14->lock);
}

/*
 * The work itself on the T6030 panel, with no DCP: blanked (no frame) it
 * leaves the level to the next present; with the gate shut, or while
 * suspended, it does nothing. None of these may reach an RPC.
 */
static void v14_bl_t6030_work(struct kunit *test)
{
	struct v14_bl_fixture f = v14_bl_fixture(test, &dcp_v14_board_t6030, true);

	v14_bl_request(test, f.dcp, 250);
	WRITE_ONCE(f.dcp->mode_state.valid, true);
	/* No frame shown: the level stays pending for the next present. */
	dcp_v14_backlight_work(&f.v14->backlight_work.work);
	KUNIT_EXPECT_EQ(test, f.v14->brightness_presents, 0ULL);
	KUNIT_EXPECT_TRUE(test, dcp_backlight_pending(f.dcp));

	iomfb_v14_7_backlight_suspend(f.dcp);
	dcp_v14_backlight_work(&f.v14->backlight_work.work);
	KUNIT_EXPECT_TRUE(test, dcp_backlight_pending(f.dcp));
	iomfb_v14_7_backlight_resume(f.dcp);

	dcp_v14_bl_shut(f.v14, true);
	dcp_v14_backlight_work(&f.v14->backlight_work.work);
	KUNIT_EXPECT_EQ(test, f.v14->brightness_presents, 0ULL);
	KUNIT_EXPECT_TRUE(test, dcp_backlight_pending(f.dcp));
}

static struct kunit_case v14_backlight_cases[] = {
	KUNIT_CASE(v14_bl_binary64),
	KUNIT_CASE(v14_bl_level_block),
	KUNIT_CASE(v14_bl_swap_unchanged_without_level),
	KUNIT_CASE(v14_bl_boards),
	KUNIT_CASE(v14_bl_gate_transitions),
	KUNIT_CASE(v14_bl_plan),
	KUNIT_CASE(v14_bl_policy),
	KUNIT_CASE(v14_bl_other_boards_untouched),
	KUNIT_CASE(v14_bl_external_untouched),
	KUNIT_CASE(v14_bl_t6030_swap_levels),
	KUNIT_CASE(v14_bl_t6030_gate),
	KUNIT_CASE(v14_bl_t6030_work),
	{}
};

static struct kunit_suite v14_backlight_suite = {
	.name = "dcp_v14_backlight",
	.test_cases = v14_backlight_cases,
};

kunit_test_suite(v14_backlight_suite);
