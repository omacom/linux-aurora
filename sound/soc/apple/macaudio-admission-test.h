/* SPDX-License-Identifier: GPL-2.0-only */
/* Exercise the admission predicate used for Speaker/Speakers DT links. */
#include <kunit/test.h>

static void macaudio_admission_needs_a_safe_level(struct kunit *test)
{
	struct macaudio_platform_cfg cfg;

	/* Generic apple,macaudio plus a DT Speaker backend. */
	KUNIT_EXPECT_FALSE(test, macaudio_speakers_allowed(&macaudio_fallback_cfg));
	KUNIT_EXPECT_FALSE(test, macaudio_speakers_allowed(&macaudio_j45x_cfg));
	KUNIT_EXPECT_TRUE(test, macaudio_speakers_allowed(&macaudio_j413_cfg));
	KUNIT_EXPECT_TRUE(test, macaudio_speakers_allowed(&macaudio_j700_cfg));
	/* The AOP-fed machines: J714 and J813 with their safe level, J873 without one */
	KUNIT_EXPECT_TRUE(test, macaudio_speakers_allowed(&macaudio_j714_cfg));
	KUNIT_EXPECT_TRUE(test, macaudio_speakers_allowed(&macaudio_j813_cfg));
	KUNIT_EXPECT_FALSE(test, macaudio_speakers_allowed(&macaudio_j873_cfg));
	/* The Mac Studio M5: no sense and no protection owner yet, speaker off */
	KUNIT_EXPECT_FALSE(test, macaudio_speakers_allowed(&macaudio_j775_cfg));
	cfg = macaudio_j413_cfg;
	cfg.safe_vol = 0;
	KUNIT_EXPECT_FALSE(test, macaudio_speakers_allowed(&cfg));
	cfg = macaudio_j413_cfg;
	cfg.speakers = SPKR_NONE;
	KUNIT_EXPECT_FALSE(test, macaudio_speakers_allowed(&cfg));
	cfg = macaudio_j413_cfg;
	cfg.amp = AMP_NONE;
	KUNIT_EXPECT_FALSE(test, macaudio_speakers_allowed(&cfg));
	/* An amp the driver cannot hold at a safe level */
	cfg = macaudio_j413_cfg;
	cfg.amp = AMP_SSM3515;
	KUNIT_EXPECT_FALSE(test, macaudio_speakers_allowed(&cfg));
}

static struct kunit_case macaudio_admission_cases[] = {
	KUNIT_CASE(macaudio_admission_needs_a_safe_level),
	{}
};

static struct kunit_suite macaudio_admission_suite = {
	.name = "apple-speaker-admission",
	.test_cases = macaudio_admission_cases,
};
kunit_test_suite(macaudio_admission_suite);
