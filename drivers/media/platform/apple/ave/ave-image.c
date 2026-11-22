// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple AVE video encoder: constant words of the session commands
 *
 * macOS builds Start and Process in code, from the session settings and
 * its own defaults; nothing is stored. Besides the fields the driver
 * computes (sizes, geometry, profile, level, rate control, GOP, buffer
 * addresses, per-frame values), every captured command on firmware
 * encoder firmware 9003.78.0 holds the same small set of constant words. They
 * are listed here as facts, grouped by where they sit; for most of them
 * the meaning is not known, only that macOS always sets that value. The
 * commands are built in a zeroed buffer: these words first, then the
 * computed fields.
 *
 * Copyright The Aurora Silicon Contributors
 */
#include <linux/kernel.h>
#include <linux/unaligned.h>

#include "ave.h"
#include "ave-regs.h"

/* @count consecutive u32 words at @off, each set to @val. */
struct ave_word {
	u32 off;
	u16 count;
	u32 val;
};

#define W(o, v)		{ .off = (o), .count = 1, .val = (v) }
#define RUN(o, n, v)	{ .off = (o), .count = (n), .val = (v) }

/* IEEE 754 doubles, high words only (the low words are zero) */
#define F64_HI_1_0	0x3ff00000	/* 1.0 */
#define F64_HI_0_5	0x3fe00000	/* 0.5 */
#define F64_HI_M1_0	0xbff00000	/* -1.0 */

/* Session configuration words common to both codecs (Start +0x68..) */
static const struct ave_word ave_start_common[] = {
	/* First surface size: always the 0x12c000 header surface */
	W(0x50, 0x12c000),
	W(0x94, 0x36),
	W(0x214, 0x39d),
	W(0x218, 0x18),
	/* Rate-control doubles: 1.0, 0.5 (the 0.5 s VBV buffer), -1, -1 */
	W(0x28c, F64_HI_1_0),
	W(0x294, F64_HI_0_5),
	W(0x2c4, F64_HI_M1_0),
	W(0x2cc, F64_HI_M1_0),
	W(0x2d0, 1),
	W(0x2e4, 1),
	RUN(0x2f4, 3, 1),
	RUN(0x338, 2, 0xffffffff),
	/* Buffer set: metadata sizes of slots 0..2, +0x13d0 surface size */
	W(0xf58, 0xc000),
	W(0xf68, 0xc000),
	W(0xf78, 0xc000),
	W(0x13d8, 0x4000),
};

static const struct ave_word ave_h264_start[] = {
	/* Codec (1) and four flags after it */
	RUN(0xc0, 5, 1),
	W(0x370, 2),
	/* Codec settings block (+0x2580..): see also ave_start_codec_bytes() */
	W(0x258c, 3),
	W(0x2594, 1),
	W(0x2598, 8),		/* bit depth */
	W(0x25a4, 0x100),
	W(0x25cc, 1),
	W(0x25d4, 1),
	W(0x26dc, 4),
	W(0x26f8, 0x15),
	RUN(0x2704, 2, 1),
	W(0x2734, 1),
	W(0x2738, 0x10),
	W(0x27c0, 8),
	/* Sequence and picture parameter block (+0x2900..) */
	W(0x2940, 1),
	W(0x2970, 0x100000),
	RUN(0x2974, 239, 0x100010),
	W(0x2d30, 0x10),
	W(0x2d34, 1),
	W(0x2d3c, 2),
	RUN(0x2d54, 2, 1),
	W(0x2d5c, 0x1000000),
	W(0x2d60, 5),
	W(0x2d68, 1),
	RUN(0x2d6c, 3, 2),
	W(0x2dc8, 1),
	/* Picture parameters: byte +0x3020 is 1; +0x3023 (8x8 transform) is computed */
	W(0x3020, 1),
	/* Two runs of -13 */
	RUN(0x3004, 7, 0xfffffff3),
	RUN(0x302c, 7, 0xfffffff3),
	W(0x3048, 1),
	/* Execution-unit info */
	W(0x3150, 1),
	W(0x3158, 1),
};

static const struct ave_word ave_hevc_start[] = {
	/* Codec (2) and four flags after it */
	W(0xc0, 2),
	RUN(0xc4, 4, 1),
	W(0x370, 0x2000002),
	W(0x1358, 0x4000),	/* +0x1350 surface size, fixed for HEVC */
	/* Codec settings block (+0x2580..): see also ave_start_codec_bytes() */
	W(0x2588, 2),
	W(0x2594, 1),
	W(0x2598, 8),		/* bit depth */
	W(0x25a4, 0x100),
	W(0x25cc, 1),
	W(0x25d4, 1),
	W(0x26dc, 4),
	RUN(0x26f8, 2, 0x15),
	W(0x2700, 2),
	RUN(0x2704, 2, 1),
	W(0x2710, 1),
	W(0x2734, 1),
	W(0x2738, 0x10),
	W(0x27c0, 8),
	/* Profile, tier and level block, first copy (+0x2920..) */
	W(0x2920, 0x101),
	W(0x292c, 1),
	W(0x2934, 1),
	W(0x295c, 0x1010001),
	/* Constraint flag bytes; +0x2962 (max 8 bit) is computed */
	W(0x2960, 0x01000101),
	W(0x2964, 1),
	RUN(0x2a8c, 7, 0x1010101),
	W(0x2aa8, 0x101),
	W(0x2abc, 0x1010101),
	W(0x2ac0, 0x101),
	RUN(0x2e38, 7, 4),
	/* Timing pair 1001 / 24000, also at +0x74f0 */
	W(0x3298, 1001),
	W(0x329c, 24000),
	/* Sequence parameter block, second copy (+0x58f0..) */
	W(0x58f8, 1),
	W(0x5900, 1),
	W(0x5928, 0x1010001),
	/* Constraint flag bytes, second copy; +0x592e (max 8 bit) is computed */
	W(0x592c, 0x01000000),
	W(0x5930, 1),
	W(0x5aac, 1),
	W(0x5ae0, 7),		/* log2_max_pic_order_cnt_lsb_minus4: 11-bit POC */
	RUN(0x5ae8, 7, 4),	/* ends with sps_max_dec_pic_buffering_minus1 */
	W(0x5b40, 2),
	W(0x5b48, 3),
	W(0x5b4c, 1),
	W(0x5b54, 1),
	/* Picture parameter and slice defaults (+0x7400..) */
	W(0x7400, 0x100),
	RUN(0x7404, 2, 7),
	W(0x740c, 1),
	W(0x7414, 1),
	W(0x749c, 1),
	W(0x74b0, 0x100),
	W(0x74b4, 5),
	W(0x74bc, 1),
	RUN(0x74c0, 3, 2),
	W(0x74f0, 1001),
	W(0x74f4, 24000),
	W(0x7500, 0x101),
	W(0x7504, 1),
	W(0x7508, 3),
	W(0x750c, 5),
	RUN(0x7510, 3, 0x17),
	RUN(0x7524, 2, 0xbebb),
	W(0x7558, 0x100),
	/* Rate-control block (+0x9600..), with two runs of -13 */
	W(0x960c, 2),
	RUN(0x9614, 7, 0xfffffff3),
	RUN(0x9634, 7, 0xfffffff3),
	W(0x9654, 0x100),
	W(0x9e64, 0x10000),
	W(0xb734, 2),
	W(0xb774, 1),
	/* Execution-unit info */
	RUN(0x13ef4, 2, 1),
	W(0x13f00, 1),
};

/*
 * HEVC with frame reordering off: a low-delay P reference structure (the
 * reference picture sets hold the delta POCs -1 .. -4). All zero with
 * B-frames.
 */
static const struct ave_word ave_hevc_start_low_delay[] = {
	W(0xe120, 4), W(0xe154, 4), W(0xe17c, 0x01010101),
	W(0xe1dc, 4), W(0xe1e4, 0x01010101),
	W(0xe204, -1), W(0xe208, -2), W(0xe20c, -3), W(0xe210, -4),
	W(0xe244, 1), W(0xe248, 2), W(0xe24c, 3), W(0xe250, 4),
	W(0xe284, 4), W(0xe2b8, 1), W(0xe2e0, 1), W(0xe340, 1), W(0xe348, 1),
	W(0xe368, -1), W(0xe3a8, 1), W(0xe3e8, 1),
	W(0xe41c, 2), W(0xe444, 0x101), W(0xe4a4, 2), W(0xe4ac, 0x101),
	W(0xe4cc, -1), W(0xe4d0, -2),
	W(0xe50c, 1), W(0xe510, 2), W(0xe54c, 2),
	W(0xe580, 3), W(0xe5a8, 0x10101), W(0xe608, 3), W(0xe610, 0x10101),
	W(0xe630, -1), W(0xe634, -2), W(0xe638, -3),
	W(0xe670, 1), W(0xe674, 2), W(0xe678, 3), W(0xe6b0, 3),
};

/* Process: the first-frame flag and per-frame values are computed. */
static const struct ave_word ave_h264_process[] = {
	/* Codec block */
	W(0x99c, 1),
	W(0x9a8, 2),
	W(0x9bc, 1),
	W(0x9d4, 1),
	W(0xeb4, 1),
	W(0x12bc, 0xffffffff),
	/* Picture-management block */
	W(0x29fc, 0xffffffff),
	W(0x6238, 5),
	W(0x6278, 14400000),	/* presentation timescale, low word */
};

static const struct ave_word ave_hevc_process[] = {
	/* Codec block */
	W(0x9a8, 1),
	W(0x9b8, 2),
	W(0xab8, 0x10100),
	W(0xb44, 0x10000),
	RUN(0xb4c, 2, 5),
	RUN(0xd70, 2, 0xffffffff),
	W(0xd7c, 1),
	/* Picture-management block */
	W(0x74cc, 0xffffffff),
	W(0x74d0, 0x100),
	W(0x74e0, 1),
	W(0xad08, 5),
	W(0xad3c, 0x403e0000),	/* frame rate 30.0, rewritten per session */
	W(0xad48, 14400000),	/* presentation timescale, low word */
	W(0xad68, 0xffffffff),
};

/*
 * 26A428: independently derived invariant words from the field ledger.
 * Computed fields overwrite any invariant default; no command images or
 * captured addresses are stored here. This layout has no qualified CRC yet.
 */
static const struct ave_word ave_26a428_h264_start[] = {
	/* Invariant bytes of a word whose crop flag is computed. */
	W(0x3ac4, 0x10001),
	/* Byte zero stays set while byte three selects the 8x8 transform. */
	W(0x3820, 1),
	W(0x50, 0xec000),
	W(0x90, 0x2),
	W(0xb10, 0x10000),
	W(0xb20, 0x10000),
	W(0xb30, 0x10000),
	W(0xdb8, 0x2),
	W(0xfe0, 0x2),
	W(0x19b8, 0x1),
	W(0x1a00, 0x1),
	W(0x1a48, 0x1),
	W(0x1a58, 0x4000),
	W(0x1bb0, 0x4),
	W(0x1bb4, 0x8),
	W(0x31f0, 0x1000),
	W(0x320c, 0x1e),
	W(0x321c, 0x36),
	W(0x3234, 0x8),
	RUN(0x3248, 4, 0x1),
	W(0x3258, 0x1),
	W(0x325c, 0xffffffff),
	W(0x3298, 0x1),
	W(0x35f4, 0x39d),
	W(0x35f8, 0x18),
	W(0x3600, 0x1e),
	W(0x366c, 0x3ff00000),
	W(0x3674, 0x3fe00000),
	W(0x36f4, 0xbff00000),
	W(0x36fc, 0xbff00000),
	W(0x3700, 0x1),
	RUN(0x3714, 3, 0x1),
	W(0x3728, 0x1),
	RUN(0x372c, 2, 0x1),
	W(0x373c, 0x7),
	RUN(0x3780, 2, 0xffffffff),
	W(0x37c0, 0x1),
	W(0x37c4, 0x80800),
	W(0x37cc, 0x60005),
	W(0x37d8, 0x1000000),
	W(0x37dc, 0x5),
	W(0x37e0, 0x20100),
	W(0x37e4, 0x20002),
	W(0x3810, 0x1010100),
	W(0x3814, 0x1a1a0000),
	W(0x3818, 0xf3f3f300),
	W(0x381c, 0xf3f3f3f3),
	W(0x3824, 0xf3f30000),
	W(0x3828, 0xf3f3f3f3),
	W(0x382c, 0x1f3),
	W(0x3834, 0x100),
	W(0x3838, 0x1),
	W(0x383c, 0x8),
	W(0x3848, 0x100),
	W(0x3860, 0x1),
	W(0x3864, 0x4),
	W(0x3874, 0x15),
	W(0x3884, 0x1),
	W(0x393c, 0x8),
	W(0x3aa4, 0x1),
	W(0x3ab0, 0x20001),
	W(0x3ad0, 0x1000000),
	W(0x3ad4, 0x5),
	W(0x3ad8, 0x20100),
	W(0x3adc, 0x20002),
	W(0x3b00, 0xf3000000),
	W(0x3b04, 0xf3f3f3f3),
	W(0x3b08, 0x1f3f3),
	W(0x3b10, 0xf3f3f3f3),
	W(0x3b14, 0x1f3f3f3),
	W(0x3b1c, 0x1),
};

static const struct ave_word ave_26a428_hevc_start[] = {
	/* Constraint bytes: the max-8-bit byte is computed. */
	W(0x3a60, 0x01000101),
	W(0x6530, 0x01000000),
	W(0x50, 0xec000),
	W(0x68, 0x2),
	W(0x90, 0x2),
	W(0x94, 0x5),
	W(0xb10, 0x10000),
	W(0xb20, 0x10000),
	W(0xb30, 0x10000),
	W(0xb40, 0x10000),
	W(0xb50, 0x10000),
	W(0xdb8, 0x2),
	W(0xdbc, 0x5),
	W(0xfe0, 0x2),
	W(0xfe4, 0x5),
	W(0x19b8, 0x1),
	W(0x19c8, 0x4000),
	W(0x1a00, 0x1),
	W(0x1a48, 0x1),
	W(0x1a58, 0x4000),
	RUN(0x1bb0, 2, 0x4),
	W(0x31f0, 0x1000),
	W(0x320c, 0x1e),
	W(0x321c, 0x36),
	W(0x3248, 0x2),
	RUN(0x324c, 4, 0x1),
	RUN(0x325c, 2, 0xffffffff),
	W(0x3298, 0x1),
	W(0x35f0, 0xa),
	W(0x35f4, 0x39d),
	W(0x35f8, 0x18),
	W(0x3600, 0x1e),
	W(0x366c, 0x3ff00000),
	W(0x3674, 0x3fe00000),
	W(0x36dc, 0x1),
	RUN(0x36e0, 2, 0x3),
	W(0x36e8, 0x1e),
	W(0x36f4, 0xbff00000),
	W(0x36fc, 0xbff00000),
	W(0x3700, 0x1),
	RUN(0x3714, 3, 0x1),
	W(0x3728, 0x1),
	RUN(0x372c, 2, 0x1),
	W(0x373c, 0x7),
	W(0x3764, 0x4),
	RUN(0x3780, 2, 0xffffffff),
	W(0x37b8, 0x100),
	W(0x37bc, 0x1),
	W(0x37c4, 0x4),
	W(0x37cc, 0x100),
	W(0x37e4, 0x1),
	W(0x37e8, 0x4),
	RUN(0x37f8, 2, 0x15),
	W(0x3800, 0x2),
	W(0x3804, 0x2),
	W(0x3808, 0x1),
	W(0x38c0, 0x8),
	W(0x3a20, 0x10100),
	W(0x3a2c, 0x1),
	W(0x3a34, 0x1),
	W(0x3a5c, 0x1010001),
	W(0x3a64, 0x1),
	RUN(0x3b8c, 7, 0x1010101),
	W(0x3ba8, 0x101),
	W(0x3bbc, 0x1010101),
	W(0x3bc0, 0x101),
	RUN(0x4940, 7, 0x4),
	W(0x495c, 0x2),
	W(0x4da0, 0x3e9),
	W(0x4da4, 0x5dc0),
	W(0x64fc, 0x1),
	W(0x6504, 0x1),
	W(0x652c, 0x1010001),
	W(0x6534, 0x1),
	W(0x66b0, 0x1),
	W(0x66e4, 0x7),
	RUN(0x66ec, 7, 0x4),
	W(0x6708, 0x2),
	W(0x6744, 0x2),
	W(0x674c, 0x3),
	W(0x6750, 0x1),
	W(0x6758, 0x1),
	W(0x8004, 0x100),
	RUN(0x8008, 2, 0x7),
	W(0x8010, 0x1),
	W(0x8018, 0x1),
	W(0x80a0, 0x1),
	W(0x80b4, 0x100),
	W(0x80b8, 0x5),
	W(0x80c0, 0x1),
	RUN(0x80c4, 3, 0x2),
	W(0x80f4, 0x3e9),
	W(0x80f8, 0x5dc0),
	W(0x8104, 0x101),
	W(0x8108, 0x1),
	W(0x810c, 0x3),
	W(0x8110, 0x5),
	RUN(0x8114, 3, 0x17),
	RUN(0x8128, 2, 0xbebb),
	W(0x815c, 0x100),
	W(0x9de8, 0x10000),
	W(0x9df4, 0x2),
	RUN(0x9dfc, 7, 0xfffffff3),
	RUN(0x9e1c, 7, 0xfffffff3),
	W(0x9e3c, 0x100),
	W(0xa64c, 0x10000),
	W(0xbf10, 0x2),
	W(0xbf50, 0x1),
	RUN(0x13eac, 2, 0x1),
	W(0x13eb8, 0x1),
};

static const struct ave_word ave_26a428_h264_process[] = {
	W(0xdc0, 0x1),
	W(0x1328, 0x1e),
	W(0x25e4, 0x1),
	W(0x25ec, 0x1),
	W(0x29dc, 0xffffffff),
	W(0x5660, 0x5),
	W(0x575c, 0x403e0000),
	W(0x5768, 0xdbba00),
	W(0x58b8, 0x10000),
	W(0x58c8, 0x4000),
};

static const struct ave_word ave_26a428_hevc_process[] = {
	W(0x9a4, 0x1),
	W(0x9ac, 0x2),
	W(0xaa8, 0x10100),
	W(0xb30, 0x10000),
	RUN(0xb38, 2, 0x5),
	W(0x1568, 0x1),
	W(0x1ad0, 0x1e),
	W(0x3184, 0xffffffff),
	W(0x3194, 0x1),
	W(0x5e08, 0x5),
	W(0x5f04, 0x403e0000),
	W(0x5f10, 0xdbba00),
	W(0x5f48, 0xffffffff),
	W(0x6060, 0x10000),
	W(0x6070, 0x4000),
};

static void ave_put_words(u8 *cmd, size_t len, const struct ave_word *w, size_t n)
{
	for (size_t i = 0; i < n; i++)
		for (unsigned int j = 0; j < w[i].count; j++) {
			size_t off = w[i].off + 4 * j;

			if (WARN_ON_ONCE(off + 4 > len))
				return;
			put_unaligned_le32(w[i].val, cmd + off);
		}
}

/*
 * The 8 bytes at Start +0x2580 look like a DART address but are separate
 * settings: byte 0 is 4 for HEVC, byte 3 is 1, byte 5 is 1 unless the
 * client turned frame reordering off (H.264 only; HEVC keeps 1).
 */
static void ave_start_codec_bytes(u8 *cmd, enum ave_codec codec, bool reorder_off)
{
	cmd[0x2580] = codec == AVE_CODEC_HEVC ? 4 : 0;
	cmd[0x2583] = 1;
	cmd[0x2585] = codec == AVE_CODEC_H264 && reorder_off ? 0 : 1;
}

void ave_image_start(u8 *cmd, size_t len, const struct ave_fw_layout *fw,
		     enum ave_codec codec, bool reorder_off,
		     bool hevc_low_delay)
{
	/* 6070 has no qualified opaque defaults; its named fields are built. */
	if (fw->kind == AVE_LAYOUT_6070)
		return;
	if (fw->counted) {
		if (codec == AVE_CODEC_HEVC)
			ave_put_words(cmd, len, ave_26a428_hevc_start,
				      ARRAY_SIZE(ave_26a428_hevc_start));
		else
			ave_put_words(cmd, len, ave_26a428_h264_start,
				      ARRAY_SIZE(ave_26a428_h264_start));
		return;
	}
	ave_put_words(cmd, len, ave_start_common, ARRAY_SIZE(ave_start_common));
	if (codec == AVE_CODEC_HEVC) {
		ave_put_words(cmd, len, ave_hevc_start, ARRAY_SIZE(ave_hevc_start));
		if (hevc_low_delay)
			ave_put_words(cmd, len, ave_hevc_start_low_delay,
				      ARRAY_SIZE(ave_hevc_start_low_delay));
	} else {
		ave_put_words(cmd, len, ave_h264_start, ARRAY_SIZE(ave_h264_start));
	}
	ave_start_codec_bytes(cmd, codec, reorder_off);
}

void ave_image_process(u8 *cmd, size_t len, const struct ave_fw_layout *fw,
		       enum ave_codec codec)
{
	/* Retained 6070 codec state is copied by the session-aware writer. */
	if (fw->kind == AVE_LAYOUT_6070)
		return;
	if (fw->counted) {
		if (codec == AVE_CODEC_HEVC)
			ave_put_words(cmd, len, ave_26a428_hevc_process,
				      ARRAY_SIZE(ave_26a428_hevc_process));
		else
			ave_put_words(cmd, len, ave_26a428_h264_process,
				      ARRAY_SIZE(ave_26a428_h264_process));
		return;
	}

	if (codec == AVE_CODEC_HEVC)
		ave_put_words(cmd, len, ave_hevc_process, ARRAY_SIZE(ave_hevc_process));
	else
		ave_put_words(cmd, len, ave_h264_process, ARRAY_SIZE(ave_h264_process));
}
