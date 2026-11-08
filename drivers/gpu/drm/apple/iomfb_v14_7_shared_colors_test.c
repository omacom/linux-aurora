// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * A T6031 external processor's mode catalog, whose timings list no
 * ColorModes and take theirs from ColorElements: included at the end of
 * iomfb_v14_7.c so that the production publication callback runs. Each
 * case publishes and withdraws TimingElements and ColorElements in one
 * serialized order and checks the catalog, the description and the
 * generation after every step.
 */
#include <kunit/device.h>
#include <kunit/static_stub.h>
#include <kunit/test.h>
#include <linux/unaligned.h>

struct v14_sc {
	struct apple_dcp *dcp;
	struct apple_dcp_v14 *v14;
	unsigned int withdrawals;
};

static struct v14_sc *v14_sc_current;

struct v14_sc_blob {
	u8 data[2048];
	u32 size;
};

static void v14_sc_tag(struct v14_sc_blob *b, u32 type, u32 count)
{
	b->size = ALIGN(b->size, 4);
	put_unaligned_le32(count | type << 24, b->data + b->size);
	b->size += 4;
}

static void v14_sc_key(struct v14_sc_blob *b, const char *key)
{
	size_t length = strlen(key);

	v14_sc_tag(b, 9, length);
	memcpy(b->data + b->size, key, length);
	b->size += length;
}

static void v14_sc_value(struct v14_sc_blob *b, s64 value)
{
	v14_sc_tag(b, 4, 0);
	put_unaligned_le64(value, b->data + b->size);
	b->size += 8;
}

static void v14_sc_int(struct v14_sc_blob *b, const char *key, s64 value)
{
	v14_sc_key(b, key);
	v14_sc_value(b, value);
}

static void v14_sc_bool(struct v14_sc_blob *b, const char *key, bool value)
{
	v14_sc_key(b, key);
	v14_sc_tag(b, 11, value);
}

static void v14_sc_dimension(struct v14_sc_blob *b, const char *key, s64 active,
			     s64 total, s64 porch, s64 sync, s64 rate)
{
	v14_sc_key(b, key);
	v14_sc_tag(b, 1, 5);
	v14_sc_int(b, "Active", active);
	v14_sc_int(b, "Total", total);
	v14_sc_int(b, "FrontPorch", porch);
	v14_sc_int(b, "SyncWidth", sync);
	v14_sc_int(b, "PreciseSyncRate", rate);
}

static struct v14_sc_blob *v14_sc_blob(struct kunit *test, u32 count)
{
	struct v14_sc_blob *b = kunit_kzalloc(test, sizeof(*b), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, b);
	put_unaligned_le32(0xd3, b->data);
	b->size = 4;
	v14_sc_tag(b, 2, count);
	return b;
}

/*
 * One 1920x1080 60 Hz timing, as the J516C 14.7 image publishes it: an
 * empty ColorModes, and @unsafe (if not negative) as its one unsafe ID.
 */
static struct v14_sc_blob *v14_sc_timings(struct kunit *test, s64 id, s64 unsafe)
{
	struct v14_sc_blob *b = v14_sc_blob(test, 1);

	v14_sc_tag(b, 1, unsafe < 0 ? 6 : 7);
	v14_sc_bool(b, "IsVirtual", false);
	v14_sc_int(b, "ID", id);
	v14_sc_int(b, "Score", 100);
	v14_sc_dimension(b, "HorizontalAttributes", 1920, 2200, 88, 44, (675U << 16) / 10);
	v14_sc_dimension(b, "VerticalAttributes", 1080, 1125, 4, 5, 60 << 16);
	v14_sc_key(b, "ColorModes");
	v14_sc_tag(b, 2, 0);
	if (unsafe >= 0) {
		v14_sc_key(b, "UnsafeColorElementIDs");
		v14_sc_tag(b, 2, 1);
		v14_sc_value(b, unsafe);
	}
	return b;
}

/* ColorElements: full-range SDR RGB8 color modes with @ids. */
static struct v14_sc_blob *v14_sc_colors(struct kunit *test, const s64 *ids, u32 count)
{
	struct v14_sc_blob *b = v14_sc_blob(test, count);
	u32 i;

	for (i = 0; i < count; i++) {
		v14_sc_tag(b, 1, 8);
		v14_sc_bool(b, "IsVirtual", false);
		v14_sc_int(b, "ID", ids[i]);
		v14_sc_int(b, "Score", 10);
		v14_sc_int(b, "Depth", 8);
		v14_sc_int(b, "Colorimetry", DCP_COLORIMETRY_RGB);
		v14_sc_int(b, "DynamicRange", DCP_COLOR_YCBCR_RANGE_FULL);
		v14_sc_int(b, "EOTF", DCP_EOTF_SDR_GAMMA);
		v14_sc_int(b, "PixelEncoding", DCP_COLOR_FORMAT_RGB);
	}
	return b;
}

static void v14_sc_withdrawn(struct apple_dcp *dcp)
{
	v14_sc_current->withdrawals++;
}

static void v14_sc_retry(struct work_struct *work)
{
}

static void v14_sc_stop(void *data)
{
	struct v14_sc *s = data;
	u32 i;

	cancel_delayed_work_sync(&s->dcp->external_retry_wq);
	for (i = 0; i < s->v14->raw_count; i++)
		kvfree(s->v14->raw[i].data);
	kfree(s->dcp->modes);
}

/* A started native external session, before its display is described. */
static struct v14_sc *v14_sc_setup(struct kunit *test, bool shared_colors)
{
	struct v14_sc *s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	struct device *dev;

	KUNIT_ASSERT_NOT_NULL(test, s);
	dev = kunit_device_register(test, "dcp-v14-shared-colors");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	s->dcp = devm_kzalloc(dev, sizeof(*s->dcp), GFP_KERNEL);
	s->v14 = devm_kzalloc(dev, sizeof(*s->v14), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s->dcp);
	KUNIT_ASSERT_NOT_NULL(test, s->v14);
	s->dcp->dev = dev;
	s->dcp->fw_compat = DCP_FIRMWARE_V_14_7;
	s->dcp->external = true;
	s->dcp->external_native = true;
	dcp_modes_init(s->dcp);
	spin_lock_init(&s->dcp->metadata.lock);
	INIT_DELAYED_WORK(&s->dcp->external_retry_wq, v14_sc_retry);
	s->dcp->v14 = s->v14;
	s->v14->dev = dev;
	s->v14->dcp = s->dcp;
	s->v14->external = true;
	s->v14->shared_colors = shared_colors;
	s->v14->started = true;
	s->v14->opened = true;
	mutex_init(&s->v14->lock);
	init_waitqueue_head(&s->v14->described_wait);
	dcp_modes_begin_attachment(s->dcp);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, v14_sc_stop, s), 0);
	v14_sc_current = s;
	kunit_activate_static_stub(test, iomfb_v14_7_external_withdrawn, v14_sc_withdrawn);
	return s;
}

/* The firmware publishes @key, a transfer begun at @generation. */
static void v14_sc_publish_at(struct kunit *test, struct v14_sc *s, const char *key,
			      const struct v14_sc_blob *b, u64 generation)
{
	char name[64] = {};

	strscpy(name, key, sizeof(name));
	KUNIT_ASSERT_EQ(test, dcp_v14_raw_property(s->v14, 0, (const u8 *)name, b->data,
						   b->size), 0);
	dcp_v14_external_published(s->v14, name, generation, false);
}

static void v14_sc_publish(struct kunit *test, struct v14_sc *s, const char *key,
			   const struct v14_sc_blob *b)
{
	v14_sc_publish_at(test, s, key, b, dcp_modes_transfer_begin(s->dcp));
}

static void v14_sc_withdraw(struct v14_sc *s, const char *key)
{
	dcp_v14_raw_remove(s->v14, key);
	dcp_v14_external_published(s->v14, key, 0, true);
}

/* The catalog offers one mode, of timing @timing in color @color. */
static void v14_sc_expect_mode(struct kunit *test, struct v14_sc *s, u32 timing, u32 color)
{
	KUNIT_EXPECT_TRUE(test, s->dcp->modes_admitted);
	KUNIT_ASSERT_EQ(test, s->dcp->nr_modes, 1U);
	KUNIT_EXPECT_EQ(test, s->dcp->modes[0].timing_mode_id, timing);
	KUNIT_EXPECT_EQ(test, s->dcp->modes[0].color_mode_id, color);
	KUNIT_EXPECT_TRUE(test, s->v14->described & DCP_V14_DESC_TIMING);
}

static void v14_sc_expect_none(struct kunit *test, struct v14_sc *s)
{
	KUNIT_EXPECT_FALSE(test, s->dcp->modes_admitted);
	KUNIT_EXPECT_FALSE(test, s->v14->described & DCP_V14_DESC_TIMING);
}

static const s64 v14_sc_id7[] = { 7 };
static const s64 v14_sc_id11[] = { 11 };
static const s64 v14_sc_id7_11[] = { 7, 11 };

/* Colors first, as the J516C image publishes them: the timings use them. */
static void v14_sc_colors_then_timings(struct kunit *test)
{
	struct v14_sc *s = v14_sc_setup(test, true);

	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id7, 1));
	v14_sc_expect_none(test, s);
	v14_sc_publish(test, s, "TimingElements", v14_sc_timings(test, 3, -1));
	v14_sc_expect_mode(test, s, 3, 7);
}

/* Timings first give no mode; the colors published after them do. */
static void v14_sc_timings_then_colors(struct kunit *test)
{
	struct v14_sc *s = v14_sc_setup(test, true);
	u64 description;

	v14_sc_publish(test, s, "TimingElements", v14_sc_timings(test, 3, -1));
	v14_sc_expect_none(test, s);
	KUNIT_EXPECT_EQ(test, s->dcp->nr_modes, 0U);
	description = s->v14->description_generation;
	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id7, 1));
	KUNIT_EXPECT_GT(test, s->v14->description_generation, description);
	v14_sc_expect_mode(test, s, 3, 7);
}

/* Replaced colors are those the catalog then selects, with no new timings. */
static void v14_sc_colors_replaced(struct kunit *test)
{
	struct v14_sc *s = v14_sc_setup(test, true);
	u64 description;

	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id7, 1));
	v14_sc_publish(test, s, "TimingElements", v14_sc_timings(test, 3, -1));
	v14_sc_expect_mode(test, s, 3, 7);
	s->v14->mode_live = true;
	description = s->v14->description_generation;
	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id11, 1));
	KUNIT_EXPECT_FALSE(test, s->v14->mode_live);
	KUNIT_EXPECT_GT(test, s->v14->description_generation, description);
	v14_sc_expect_mode(test, s, 3, 11);
}

/* A timing's unsafe ID is excluded from the shared colors, in either order. */
static void v14_sc_unsafe_excluded(struct kunit *test)
{
	struct v14_sc *s = v14_sc_setup(test, true);

	v14_sc_publish(test, s, "TimingElements", v14_sc_timings(test, 3, 11));
	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id7_11, 2));
	v14_sc_expect_mode(test, s, 3, 7);
	/* Only the unsafe color is left: no mode. */
	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id11, 1));
	v14_sc_expect_none(test, s);
	KUNIT_EXPECT_EQ(test, s->withdrawals, 1U);
}

/*
 * Withdrawn colors leave the timings no mode: the catalog is withdrawn and
 * the display reported withdrawn once, also across the TimingElements
 * removal that follows on an unplug. A later description is used again.
 */
static void v14_sc_colors_withdrawn(struct kunit *test)
{
	struct v14_sc *s = v14_sc_setup(test, true);

	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id7, 1));
	v14_sc_publish(test, s, "TimingElements", v14_sc_timings(test, 3, -1));
	v14_sc_expect_mode(test, s, 3, 7);
	v14_sc_withdraw(s, "ColorElements");
	v14_sc_expect_none(test, s);
	KUNIT_EXPECT_FALSE(test, s->v14->described & DCP_V14_DESC_COLOR);
	KUNIT_EXPECT_EQ(test, s->withdrawals, 1U);
	v14_sc_withdraw(s, "TimingElements");
	KUNIT_EXPECT_EQ(test, s->withdrawals, 1U);
	KUNIT_EXPECT_FALSE(test, s->v14->colors_withdrew);

	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id11, 1));
	v14_sc_publish(test, s, "TimingElements", v14_sc_timings(test, 4, -1));
	v14_sc_expect_mode(test, s, 4, 11);
	/* Timings withdrawn first: reported once, by their removal. */
	v14_sc_withdraw(s, "TimingElements");
	KUNIT_EXPECT_EQ(test, s->withdrawals, 2U);
	v14_sc_withdraw(s, "ColorElements");
	KUNIT_EXPECT_EQ(test, s->withdrawals, 2U);
}

/* Colors withdrawn and published again: the catalog comes back. */
static void v14_sc_colors_republished(struct kunit *test)
{
	struct v14_sc *s = v14_sc_setup(test, true);

	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id7, 1));
	v14_sc_publish(test, s, "TimingElements", v14_sc_timings(test, 3, -1));
	v14_sc_withdraw(s, "ColorElements");
	v14_sc_expect_none(test, s);
	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id11, 1));
	v14_sc_expect_mode(test, s, 3, 11);
	KUNIT_EXPECT_FALSE(test, s->v14->colors_withdrew);
}

/*
 * A ColorElements transfer begun before the current attachment changes
 * neither catalog; one begun after it rebuilds the current timings.
 */
static void v14_sc_stale_colors(struct kunit *test)
{
	struct v14_sc *s = v14_sc_setup(test, true);
	u64 old;

	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id7, 1));
	v14_sc_publish(test, s, "TimingElements", v14_sc_timings(test, 3, -1));
	v14_sc_expect_mode(test, s, 3, 7);
	old = dcp_modes_transfer_begin(s->dcp);
	dcp_modes_begin_attachment(s->dcp);
	v14_sc_publish_at(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id11, 1), old);
	KUNIT_EXPECT_FALSE(test, s->dcp->modes_admitted);
	KUNIT_EXPECT_EQ(test, s->dcp->modes[0].color_mode_id, 7U);
	KUNIT_EXPECT_EQ(test, s->withdrawals, 0U);
	/* No usable colors, stale: not withdrawn either. */
	v14_sc_publish_at(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id7, 0), old);
	KUNIT_EXPECT_EQ(test, s->withdrawals, 0U);
	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id11, 1));
	v14_sc_expect_mode(test, s, 3, 11);
}

/* Without the T6031 opt-in, ColorElements are not a timing's colors. */
static void v14_sc_not_shared(struct kunit *test)
{
	struct v14_sc *s = v14_sc_setup(test, false);

	v14_sc_publish(test, s, "ColorElements", v14_sc_colors(test, v14_sc_id7, 1));
	v14_sc_publish(test, s, "TimingElements", v14_sc_timings(test, 3, -1));
	v14_sc_expect_none(test, s);
	KUNIT_EXPECT_EQ(test, s->dcp->nr_modes, 0U);
	v14_sc_withdraw(s, "ColorElements");
	KUNIT_EXPECT_EQ(test, s->withdrawals, 0U);
}

static struct kunit_case v14_sc_cases[] = {
	KUNIT_CASE(v14_sc_colors_then_timings),
	KUNIT_CASE(v14_sc_timings_then_colors),
	KUNIT_CASE(v14_sc_colors_replaced),
	KUNIT_CASE(v14_sc_unsafe_excluded),
	KUNIT_CASE(v14_sc_colors_withdrawn),
	KUNIT_CASE(v14_sc_colors_republished),
	KUNIT_CASE(v14_sc_stale_colors),
	KUNIT_CASE(v14_sc_not_shared),
	{}
};

static struct kunit_suite v14_sc_suite = {
	.name = "dcp_v14_shared_colors",
	.test_cases = v14_sc_cases,
};

kunit_test_suite(v14_sc_suite);
