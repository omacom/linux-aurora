// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <kunit/test.h>
#include <linux/module.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "ane_h16_boot_data.h"
#include "ane_h16_preload.h"

/* Synthetic framing only, with no firmware bytes. Walk records independently
 * from their widths rather than sharing the production offset manifest.
 */
static const struct {
	char tag[4];
	u8 width;
	bool mutable;
} records[] = {
	{ "BECA", 8, true }, { "SECA", 8, false }, { "GKTS", 8, true },
	{ "otcd", 8, false }, { "oscd", 8, false }, { "oecd", 8, false },
	{ "DILS", 8, true }, { "SRSA", 8, false }, { "GFCM", 8, false },
	{ "SSSC", 8, true }, { "qF8v", 4, true }, { "LRSD", 1, false },
	{ "SVSD", 8, false }, { "LCSD", 8, false }, { "ZSTR", 4, false },
	{ "BPTP", 8, false }, { "TNGI", 1, false }, { "oeNS", 1, false },
	{ "SEBC", 8, false }, { "ECAP", 8, true }, { "BVTP", 8, true },
	{ "sP1T", 8, true }, { "BtpG", 8, true }, { "StpG", 8, false },
	{ "BlpP", 8, true }, { "SPTP", 8, false }, { "SZSD", 8, false },
	{ "LZSD", 8, false }, { "S2xG", 8, true }, { "SNUT", 8, true },
	{ "ZNUT", 4, false }, { "OTTR", 8, true }, { "LLTR", 1, false },
	{ "_COS", 4, true }, { "RCOS", 4, true }, { "dApC", 8, true },
	{ "dArW", 8, true }, { "fVED", 1, true }, { "GLFp", 8, true },
	{ "ABOI", 8, false }, { "ZSOI", 4, false }, { "ARcM", 4, true },
};

struct preload_fixture {
	u8 *text;
	u8 *data;
	struct ane_h16_preload_cell cells[ARRAY_SIZE(records)];
	struct ane_h16_preload_rule rule;
};

/* Lay the synthetic records out from @base and describe them as a rule. */
static size_t preload_frame(struct preload_fixture *f, size_t base, bool write)
{
	size_t pos = base;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(records); i++) {
		if (write) {
			memcpy(f->data + pos, records[i].tag, 4);
			put_unaligned_le32(records[i].width, f->data + pos + 4);
		}
		memcpy(f->cells[i].tag, records[i].tag, 4);
		f->cells[i].value = pos + 8;
		f->cells[i].width = records[i].width;
		f->cells[i].mutable = records[i].mutable;
		pos += 8 + records[i].width;
	}
	f->rule.cells = f->cells;
	f->rule.cell_count = ARRAY_SIZE(records);
	f->rule.zero_start = 0x17c80;
	f->rule.zero_end = 0x18320;
	return pos;
}

static void preload_free(void *allocation)
{
	kvfree(allocation);
}

static int preload_fixture_init(struct kunit *test)
{
	struct preload_fixture *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);

	if (!f)
		return -ENOMEM;
	test->priv = f;
	f->text = kvmalloc(ANE_H16_PRELOAD_TEXT_SIZE, GFP_KERNEL);
	if (!f->text)
		return -ENOMEM;
	if (kunit_add_action_or_reset(test, preload_free, f->text))
		return -ENOMEM;
	f->data = kvmalloc(ANE_H16_PRELOAD_DATA_SIZE, GFP_KERNEL);
	if (!f->data)
		return -ENOMEM;
	if (kunit_add_action_or_reset(test, preload_free, f->data))
		return -ENOMEM;
	memset(f->text, 0x5a, ANE_H16_PRELOAD_TEXT_SIZE);
	memset(f->data, 0xa5, ANE_H16_PRELOAD_DATA_SIZE);
	return preload_frame(f, 0x5a18, true) == 0x5c79 ? 0 : -EINVAL;
}

static int preload_digest(struct preload_fixture *f, u8 *digest)
{
	return ane_h16_preload_digest(&f->rule, f->text, ANE_H16_PRELOAD_TEXT_SIZE,
				      f->data, ANE_H16_PRELOAD_DATA_SIZE, digest);
}

static void preload_identity_test(struct kunit *test)
{
	/* Independently normalized synthetic bytes, SHA-256 from Python hashlib. */
	static const u8 expected[SHA256_DIGEST_SIZE] = {
		0x67, 0xb5, 0x33, 0xc6, 0x0b, 0x11, 0x03, 0x2b,
		0x3e, 0x75, 0x27, 0x7b, 0x2d, 0xe1, 0x82, 0xc6,
		0xca, 0x53, 0xa7, 0x18, 0x1f, 0xfc, 0xcc, 0xfe,
		0xd3, 0xb5, 0xe8, 0x22, 0xaa, 0x12, 0xd0, 0xbb,
	};
	struct preload_fixture *f = test->priv;
	u8 digest[SHA256_DIGEST_SIZE];
	int ret;

	KUNIT_EXPECT_EQ(test, preload_digest(f, digest), 0);
	KUNIT_EXPECT_MEMEQ(test, digest, expected, sizeof(digest));
	/* Correct framing alone cannot authenticate a synthetic image. */
	ret = ane_h16_preload_validate(&f->rule, f->text, ANE_H16_PRELOAD_TEXT_SIZE,
				       f->data, ANE_H16_PRELOAD_DATA_SIZE);
	KUNIT_EXPECT_EQ(test, ret, -ENOEXEC);
	/* The rule's own digest is the whole identity. */
	memcpy(f->rule.digest, expected, sizeof(expected));
	ret = ane_h16_preload_validate(&f->rule, f->text, ANE_H16_PRELOAD_TEXT_SIZE,
				       f->data, ANE_H16_PRELOAD_DATA_SIZE);
	KUNIT_EXPECT_EQ(test, ret, 0);
}

static void preload_mutable_test(struct kunit *test)
{
	struct preload_fixture *f = test->priv;
	u8 baseline[SHA256_DIGEST_SIZE], digest[SHA256_DIGEST_SIZE];
	size_t pos = 0x5a18;
	unsigned int i, mutable = 0;

	KUNIT_ASSERT_EQ(test, preload_digest(f, baseline), 0);
	for (i = 0; i < ARRAY_SIZE(records); i++) {
		if (records[i].mutable) {
			memset(f->data + pos + 8, 0x3c, records[i].width);
			KUNIT_ASSERT_EQ(test, preload_digest(f, digest), 0);
			KUNIT_EXPECT_MEMEQ(test, digest, baseline, sizeof(digest));
			mutable++;
		}
		pos += 8 + records[i].width;
	}
	KUNIT_EXPECT_EQ(test, mutable, 20);
	memset(f->data + 0x17c80, 0x7e, 0x6a0);
	KUNIT_ASSERT_EQ(test, preload_digest(f, digest), 0);
	KUNIT_EXPECT_MEMEQ(test, digest, baseline, sizeof(digest));
	/* Validation and normalization never erase the accepted boot data. */
	KUNIT_EXPECT_EQ(test, f->data[0x5a20], 0x3c);
	KUNIT_EXPECT_EQ(test, f->data[0x17c80], 0x7e);
	KUNIT_EXPECT_EQ(test, f->data[0x1831f], 0x7e);
}

static void preload_framing_test(struct kunit *test)
{
	struct preload_fixture *f = test->priv;
	u8 digest[SHA256_DIGEST_SIZE];
	size_t pos = 0x5a18;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(records); i++) {
		f->data[pos] ^= 1;
		KUNIT_EXPECT_EQ(test, preload_digest(f, digest), -ENOEXEC);
		f->data[pos] ^= 1;
		put_unaligned_le32(U32_MAX, f->data + pos + 4);
		KUNIT_EXPECT_EQ(test, preload_digest(f, digest), -ENOEXEC);
		put_unaligned_le32(records[i].width, f->data + pos + 4);
		pos += 8 + records[i].width;
	}
	KUNIT_EXPECT_EQ(test, preload_digest(f, digest), 0);
}

static void preload_immutable_test(struct kunit *test)
{
	static const u32 boundaries[] = {
		0, 0x5a17, 0x5c79, 0x17c7f, 0x18320, 0xb8000, 0x2abfff,
	};
	struct preload_fixture *f = test->priv;
	u8 baseline[SHA256_DIGEST_SIZE], digest[SHA256_DIGEST_SIZE];
	size_t pos = 0x5a18;
	unsigned int i;

	KUNIT_ASSERT_EQ(test, preload_digest(f, baseline), 0);
	for (i = 0; i < ARRAY_SIZE(records); i++) {
		if (!records[i].mutable) {
			f->data[pos + 8] ^= 1;
			KUNIT_ASSERT_EQ(test, preload_digest(f, digest), 0);
			KUNIT_EXPECT_NE(test, memcmp(digest, baseline, sizeof(digest)), 0);
			f->data[pos + 8] ^= 1;
		}
		pos += 8 + records[i].width;
	}
	for (i = 0; i < ARRAY_SIZE(boundaries); i++) {
		f->data[boundaries[i]] ^= 1;
		KUNIT_ASSERT_EQ(test, preload_digest(f, digest), 0);
		KUNIT_EXPECT_NE(test, memcmp(digest, baseline, sizeof(digest)), 0);
		f->data[boundaries[i]] ^= 1;
	}
	f->text[ANE_H16_PRELOAD_TEXT_SIZE - 1] ^= 1;
	KUNIT_ASSERT_EQ(test, preload_digest(f, digest), 0);
	KUNIT_EXPECT_NE(test, memcmp(digest, baseline, sizeof(digest)), 0);
}

static void preload_bounds_test(struct kunit *test)
{
	struct preload_fixture *f = test->priv;
	u8 digest[SHA256_DIGEST_SIZE];
	int delta, ret;

	for (delta = -1; delta <= 1; delta += 2) {
		ret = ane_h16_preload_digest(&f->rule, f->text, ANE_H16_PRELOAD_TEXT_SIZE + delta,
					     f->data, ANE_H16_PRELOAD_DATA_SIZE, digest);
		KUNIT_EXPECT_EQ(test, ret, -EINVAL);
		ret = ane_h16_preload_digest(&f->rule, f->text, ANE_H16_PRELOAD_TEXT_SIZE,
					     f->data, ANE_H16_PRELOAD_DATA_SIZE + delta, digest);
		KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	}
	ret = ane_h16_preload_digest(&f->rule, NULL, ANE_H16_PRELOAD_TEXT_SIZE,
				     f->data, ANE_H16_PRELOAD_DATA_SIZE, digest);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = ane_h16_preload_digest(&f->rule, f->text, ANE_H16_PRELOAD_TEXT_SIZE,
				     NULL, ANE_H16_PRELOAD_DATA_SIZE, digest);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = ane_h16_preload_digest(&f->rule, f->text, ANE_H16_PRELOAD_TEXT_SIZE,
				     f->data, ANE_H16_PRELOAD_DATA_SIZE, NULL);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = ane_h16_preload_digest(NULL, f->text, ANE_H16_PRELOAD_TEXT_SIZE,
				     f->data, ANE_H16_PRELOAD_DATA_SIZE, digest);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

/* A malformed rule is a driver bug: refused as -EINVAL, never as a mismatch. */
static void preload_rule_shape_test(struct kunit *test)
{
	struct preload_fixture *f = test->priv;
	u8 digest[SHA256_DIGEST_SIZE];
	struct ane_h16_preload_rule rule;

	rule = f->rule;
	rule.cell_count = 0;
	KUNIT_EXPECT_EQ(test, ane_h16_preload_digest(&rule, f->text, ANE_H16_PRELOAD_TEXT_SIZE,
						     f->data, ANE_H16_PRELOAD_DATA_SIZE, digest),
			-EINVAL);
	rule = f->rule;
	rule.zero_start = 0x5c70;	/* inside the last record */
	KUNIT_EXPECT_EQ(test, ane_h16_preload_digest(&rule, f->text, ANE_H16_PRELOAD_TEXT_SIZE,
						     f->data, ANE_H16_PRELOAD_DATA_SIZE, digest),
			-EINVAL);
	rule = f->rule;
	rule.zero_end = ANE_H16_PRELOAD_DATA_SIZE + 1;
	KUNIT_EXPECT_EQ(test, ane_h16_preload_digest(&rule, f->text, ANE_H16_PRELOAD_TEXT_SIZE,
						     f->data, ANE_H16_PRELOAD_DATA_SIZE, digest),
			-EINVAL);
	/* Overlapping records. */
	f->cells[1].value = f->cells[0].value + 4;
	KUNIT_EXPECT_EQ(test, preload_digest(f, digest), -EINVAL);
}

/*
 * A later firmware build moves the patchbay. The rule for one layout rejects
 * the other as a mismatch (-ENOEXEC), so a profile table can try each rule in
 * turn and the right one wins.
 */
static void preload_layout_select_test(struct kunit *test)
{
	struct preload_fixture *f = test->priv;
	struct ane_h16_preload_rule old_rule;
	struct ane_h16_preload_cell old_cells[ARRAY_SIZE(records)];
	u8 digest[SHA256_DIGEST_SIZE];

	memcpy(old_cells, f->cells, sizeof(old_cells));
	old_rule = f->rule;
	old_rule.cells = old_cells;
	memset(f->data + 0x5a18, 0xa5, 0x5c79 - 0x5a18);
	KUNIT_ASSERT_EQ(test, preload_frame(f, 0x5a18 + 0x3f80, true), 0x5c79 + 0x3f80);
	f->rule.zero_start += 0x3f80;
	f->rule.zero_end += 0x3f80;
	KUNIT_EXPECT_EQ(test, ane_h16_preload_digest(&old_rule, f->text,
						     ANE_H16_PRELOAD_TEXT_SIZE, f->data,
						     ANE_H16_PRELOAD_DATA_SIZE, digest),
			-ENOEXEC);
	KUNIT_ASSERT_EQ(test, preload_digest(f, digest), 0);
	memcpy(f->rule.digest, digest, sizeof(digest));
	KUNIT_EXPECT_EQ(test, ane_h16_preload_validate(&f->rule, f->text,
						       ANE_H16_PRELOAD_TEXT_SIZE, f->data,
						       ANE_H16_PRELOAD_DATA_SIZE),
			0);
	/* A mutable value of the new layout still normalizes. */
	memset(f->data + f->cells[0].value, 0x3c, f->cells[0].width);
	KUNIT_EXPECT_EQ(test, ane_h16_preload_validate(&f->rule, f->text,
						       ANE_H16_PRELOAD_TEXT_SIZE, f->data,
						       ANE_H16_PRELOAD_DATA_SIZE),
			0);
}

/*
 * Every shipped profile is well formed: a build to boot with, a rule whose
 * records frame cleanly when written at their offsets, and an identity no
 * other profile shares. Synthetic bytes never authenticate as a profile.
 */
static void preload_profiles_check(struct kunit *test, const struct ane_h16_profile *profiles,
				   unsigned int count)
{
	struct preload_fixture *f = test->priv;
	unsigned int i, j, k;

	KUNIT_ASSERT_GT(test, count, 0);
	for (i = 0; i < count; i++) {
		const struct ane_h16_profile *p = &profiles[i];
		const struct ane_h16_preload_rule *r = p->preload;
		u8 digest[SHA256_DIGEST_SIZE];

		KUNIT_ASSERT_NOT_NULL(test, p->build);
		KUNIT_ASSERT_NOT_NULL(test, p->identity);
		KUNIT_ASSERT_NOT_NULL(test, r);
		memset(f->data, 0xa5, ANE_H16_PRELOAD_DATA_SIZE);
		for (k = 0; k < r->cell_count; k++) {
			memcpy(f->data + r->cells[k].value - 8, r->cells[k].tag, 4);
			put_unaligned_le32(r->cells[k].width, f->data + r->cells[k].value - 4);
		}
		KUNIT_EXPECT_EQ(test, ane_h16_preload_digest(r, f->text,
							     ANE_H16_PRELOAD_TEXT_SIZE, f->data,
							     ANE_H16_PRELOAD_DATA_SIZE, digest),
				0);
		KUNIT_EXPECT_EQ(test, ane_h16_preload_validate(r, f->text,
							       ANE_H16_PRELOAD_TEXT_SIZE, f->data,
							       ANE_H16_PRELOAD_DATA_SIZE),
				-ENOEXEC);
		for (j = 0; j < i; j++) {
			KUNIT_EXPECT_NE(test, memcmp(r->digest, profiles[j].preload->digest,
						     SHA256_DIGEST_SIZE), 0);
			KUNIT_EXPECT_STRNEQ(test, p->identity, profiles[j].identity);
		}
	}
}

static void preload_profiles_test(struct kunit *test)
{
	unsigned int i, j;

	preload_profiles_check(test, ane_h16_profiles, ARRAY_SIZE(ane_h16_profiles));
	preload_profiles_check(test, ane_t8132_profiles, ARRAY_SIZE(ane_t8132_profiles));
	/* The T6041 and the T8132 never accept each other's firmware. */
	for (i = 0; i < ARRAY_SIZE(ane_t8132_profiles); i++) {
		for (j = 0; j < ARRAY_SIZE(ane_h16_profiles); j++) {
			KUNIT_EXPECT_NE(test, memcmp(ane_t8132_profiles[i].preload->digest,
						     ane_h16_profiles[j].preload->digest,
						     SHA256_DIGEST_SIZE), 0);
			KUNIT_EXPECT_STRNEQ(test, ane_t8132_profiles[i].identity,
					    ane_h16_profiles[j].identity);
		}
	}
}

static void preload_26b123_ce_script_test(struct kunit *test)
{
	struct preload_fixture *f = test->priv;
	u8 baseline[SHA256_DIGEST_SIZE], digest[SHA256_DIGEST_SIZE];
	static const u32 positions[] = { 0xb9040, 0xba094, 0xbd03f };
	unsigned int i;

	f->rule = ane_h16_preload_26b123;
	KUNIT_ASSERT_EQ(test, preload_digest(f, baseline), 0);
	/* The entire live CE script stays in the identity, including its padding. */
	for (i = 0; i < ARRAY_SIZE(positions); i++) {
		f->text[positions[i]] ^= 1;
		KUNIT_ASSERT_EQ(test, preload_digest(f, digest), 0);
		KUNIT_EXPECT_NE(test, memcmp(digest, baseline, sizeof(digest)), 0);
		f->text[positions[i]] ^= 1;
	}
}

static struct kunit_case preload_cases[] = {
	KUNIT_CASE(preload_26b123_ce_script_test),
	KUNIT_CASE(preload_identity_test),
	KUNIT_CASE(preload_mutable_test),
	KUNIT_CASE(preload_framing_test),
	KUNIT_CASE(preload_immutable_test),
	KUNIT_CASE(preload_bounds_test),
	KUNIT_CASE(preload_rule_shape_test),
	KUNIT_CASE(preload_layout_select_test),
	KUNIT_CASE(preload_profiles_test),
	{}
};

static struct kunit_suite preload_suite = {
	.name = "ane-h16-preload",
	.init = preload_fixture_init,
	.test_cases = preload_cases,
};

kunit_test_suite(preload_suite);
MODULE_LICENSE("Dual MIT/GPL");
