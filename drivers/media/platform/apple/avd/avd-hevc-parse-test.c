// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests for the HEVC slice header parser of the Apple video decoder.
 *
 * The streams are built here bit by bit from the slice segment header syntax
 * (H.265 7.3.6.1); the expected results are worked out by hand from that
 * syntax, not from the parser.
 */

#include <kunit/test.h>
#include <linux/slab.h>

#include "avd-hevc-parse.h"

#define HEVC_NAL_TRAIL_R	1

struct bitwriter {
	u8 rbsp[256];
	size_t bits;
};

struct stream {
	u8 buf[2048];
	size_t len;
};

static void put_bits(struct bitwriter *bw, u64 val, int n)
{
	while (n--) {
		if ((val >> n) & 1)
			bw->rbsp[bw->bits / 8] |= 0x80 >> (bw->bits % 8);
		bw->bits++;
	}
}

static void put_ue(struct bitwriter *bw, u32 val)
{
	u64 x = (u64)val + 1;
	int len = fls64(x) - 1;

	put_bits(bw, 0, len);
	put_bits(bw, x, len + 1);
}

/* rbsp_slice_segment_trailing_bits: a 1, then zeros to the byte boundary. */
static void put_trailing_bits(struct bitwriter *bw)
{
	put_bits(bw, 1, 1);
	while (bw->bits % 8)
		put_bits(bw, 0, 1);
}

/* Append a start code, a NAL unit header and the escaped RBSP, then data. */
static void put_nal(struct stream *s, u8 type, const struct bitwriter *bw,
		    size_t data_len)
{
	int zeros = 0;

	s->buf[s->len++] = 0;
	s->buf[s->len++] = 0;
	s->buf[s->len++] = 1;
	s->buf[s->len++] = type << 1;
	s->buf[s->len++] = 1; /* nuh_temporal_id_plus1 */
	for (size_t i = 0; i < bw->bits / 8; i++) {
		if (zeros >= 2 && bw->rbsp[i] <= 3) {
			s->buf[s->len++] = 3;
			zeros = 0;
		}
		s->buf[s->len++] = bw->rbsp[i];
		zeros = bw->rbsp[i] ? 0 : zeros + 1;
	}
	memset(s->buf + s->len, 0xaa, data_len);
	s->len += data_len;
}

/*
 * 256x256, 64x64 CTBs: 16 CTBs, so slice_segment_address takes 4 bits.
 * One short-term RPS in the SPS, so its index takes no bits. WPP on, so
 * every slice carries entry points.
 */
static void test_params(struct v4l2_ctrl_hevc_sps *sps, struct v4l2_ctrl_hevc_pps *pps,
			struct v4l2_ctrl_hevc_decode_params *dec)
{
	*sps = (struct v4l2_ctrl_hevc_sps){
		.pic_width_in_luma_samples = 256,
		.pic_height_in_luma_samples = 256,
		.chroma_format_idc = 1,
		.log2_diff_max_min_luma_coding_block_size = 3,
		.num_short_term_ref_pic_sets = 1,
	};
	*pps = (struct v4l2_ctrl_hevc_pps){
		.flags = V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED,
	};
	*dec = (struct v4l2_ctrl_hevc_decode_params){};
}

/*
 * An I slice segment header of a TRAIL_R picture for test_params(), with
 * @num_eps entry points of @offset_len bits whose entry_point_offset_minus1
 * values are @offsets (only the first @num_offsets are written).
 */
static void put_i_slice(struct bitwriter *bw, bool first, u32 addr, u32 num_eps,
			int offset_len, const u32 *offsets, int num_offsets)
{
	memset(bw, 0, sizeof(*bw));
	put_bits(bw, first, 1);		/* first_slice_segment_in_pic_flag */
	put_ue(bw, 0);			/* slice_pic_parameter_set_id */
	if (!first)
		put_bits(bw, addr, 4);	/* slice_segment_address */
	put_ue(bw, 2);			/* slice_type: I */
	put_bits(bw, 0, 4);		/* slice_pic_order_cnt_lsb */
	put_bits(bw, 1, 1);		/* short_term_ref_pic_set_sps_flag */
	put_ue(bw, 1);			/* slice_qp_delta = 0, as se(v) */
	put_ue(bw, num_eps);		/* num_entry_point_offsets */
	if (num_eps) {
		put_ue(bw, offset_len - 1);
		for (int i = 0; i < num_offsets; i++)
			put_bits(bw, offsets[i], offset_len);
	}
	put_trailing_bits(bw);
}

static void put_se(struct bitwriter *bw, s32 val)
{
	put_ue(bw, val > 0 ? 2 * val - 1 : -2 * val);
}

struct p_slice {
	u32 num_ref_idx_minus1;
	bool temporal_mvp;		/* needs the SPS temporal MVP flag */
	u32 collocated_ref_idx;
	bool weights;			/* needs the PPS weighted pred flag */
	u32 luma_log2_weight_denom;
	s32 delta_chroma_log2_weight_denom;
};

/*
 * The first P slice segment of a TRAIL_R picture for test_params(), with
 * the reference count overridden; without entry points' data offsets
 * (num_entry_point_offsets = 0).
 */
static void put_p_slice(struct bitwriter *bw, const struct p_slice *p)
{
	memset(bw, 0, sizeof(*bw));
	put_bits(bw, 1, 1);		/* first_slice_segment_in_pic_flag */
	put_ue(bw, 0);			/* slice_pic_parameter_set_id */
	put_ue(bw, 1);			/* slice_type: P */
	put_bits(bw, 0, 4);		/* slice_pic_order_cnt_lsb */
	put_bits(bw, 1, 1);		/* short_term_ref_pic_set_sps_flag */
	if (p->temporal_mvp)
		put_bits(bw, 1, 1);	/* slice_temporal_mvp_enabled_flag */
	put_bits(bw, 1, 1);		/* num_ref_idx_active_override_flag */
	put_ue(bw, p->num_ref_idx_minus1);
	if (p->temporal_mvp && p->num_ref_idx_minus1)
		put_ue(bw, p->collocated_ref_idx);
	if (p->weights) {
		put_ue(bw, p->luma_log2_weight_denom);
		put_se(bw, p->delta_chroma_log2_weight_denom);
		/* no luma or chroma weights for any entry */
		put_bits(bw, 0, 2 * (p->num_ref_idx_minus1 + 1));
	}
	put_ue(bw, 0);			/* five_minus_max_num_merge_cand */
	put_se(bw, 0);			/* slice_qp_delta */
	put_ue(bw, 0);			/* num_entry_point_offsets */
	put_trailing_bits(bw);
}

/* Two short-term references before the current picture: DPB 0 and 1. */
static void two_refs(struct v4l2_ctrl_hevc_decode_params *dec)
{
	dec->num_active_dpb_entries = 2;
	dec->num_poc_st_curr_before = 2;
	dec->poc_st_curr_before[0] = 0;
	dec->poc_st_curr_before[1] = 1;
}

struct parse_result {
	int ret;
	struct v4l2_ctrl_hevc_slice_params *slices;
	int num_slices;
	u32 *eps;
	int num_eps;
};

static void parse(struct kunit *test, const struct stream *s,
		  const struct v4l2_ctrl_hevc_sps *sps,
		  const struct v4l2_ctrl_hevc_pps *pps,
		  const struct v4l2_ctrl_hevc_decode_params *dec,
		  struct parse_result *r)
{
	*r = (struct parse_result){};
	r->ret = avd_hevc_parse_slices(s->buf, s->len, sps, pps, dec, &r->slices,
				       &r->num_slices, &r->eps, &r->num_eps);
	if (!r->ret) {
		kunit_add_action(test, (kunit_action_t *)kvfree, r->slices);
		kunit_add_action(test, (kunit_action_t *)kvfree, r->eps);
	}
}

/*
 * Two I slices of 2 entry points each. By hand: the first header is
 * 37 bits plus trailing bits, 5 bytes, so its data starts 3 + 2 + 5 bytes
 * after the buffer start; the second header (with a 4-bit address) is
 * 6 bytes, its data 3 + 2 + 6 bytes after its own start code. Each slice
 * NAL unit carries 64 data bytes.
 */
static void avd_hevc_parse_two_slices(struct kunit *test)
{
	static const u32 offsets[] = { 15, 31 };
	struct v4l2_ctrl_hevc_decode_params dec;
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct parse_result r;
	struct bitwriter bw;
	struct stream *s;

	s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s);
	test_params(&sps, &pps, &dec);
	put_i_slice(&bw, true, 0, 2, 8, offsets, 2);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);
	put_i_slice(&bw, false, 8, 2, 8, offsets, 2);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);

	parse(test, s, &sps, &pps, &dec, &r);
	KUNIT_ASSERT_EQ(test, r.ret, 0);
	KUNIT_ASSERT_EQ(test, r.num_slices, 2);
	KUNIT_ASSERT_EQ(test, r.num_eps, 4);
	KUNIT_EXPECT_EQ(test, r.eps[0], 16);
	KUNIT_EXPECT_EQ(test, r.eps[1], 32);
	KUNIT_EXPECT_EQ(test, r.eps[2], 16);
	KUNIT_EXPECT_EQ(test, r.eps[3], 32);
	KUNIT_EXPECT_EQ(test, r.slices[0].slice_type, V4L2_HEVC_SLICE_TYPE_I);
	KUNIT_EXPECT_EQ(test, r.slices[0].num_entry_point_offsets, 2);
	KUNIT_EXPECT_EQ(test, r.slices[0].data_byte_offset, 10);
	KUNIT_EXPECT_EQ(test, r.slices[0].bit_size, 74 * 8);
	KUNIT_EXPECT_EQ(test, r.slices[1].slice_segment_addr, 8);
	KUNIT_EXPECT_EQ(test, r.slices[1].data_byte_offset, 11);
	KUNIT_EXPECT_EQ(test, r.slices[1].bit_size, 75 * 8);
}

/*
 * A later slice whose entry point count wraps the table bound when added
 * to the earlier slices' count (3 + 0xfffffffd == 0 in 32 bits).
 */
static void avd_hevc_parse_rejects_wrapping_entry_point_count(struct kunit *test)
{
	static const u32 offsets[] = { 15, 15, 15 };
	struct v4l2_ctrl_hevc_decode_params dec;
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct parse_result r;
	struct bitwriter bw;
	struct stream *s;

	s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s);
	test_params(&sps, &pps, &dec);
	put_i_slice(&bw, true, 0, 3, 8, offsets, 3);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);
	put_i_slice(&bw, false, 8, 0xfffffffd, 8, offsets, 3);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);

	parse(test, s, &sps, &pps, &dec, &r);
	KUNIT_EXPECT_EQ(test, r.ret, -EINVAL);
}

/*
 * Entry points of 16 + 32 bytes, so the last substream starts 48 bytes into
 * the slice data, but the slice carries only 40 data bytes.
 */
static void avd_hevc_parse_rejects_entry_points_past_slice_data(struct kunit *test)
{
	static const u32 offsets[] = { 15, 31 };
	struct v4l2_ctrl_hevc_decode_params dec;
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct parse_result r;
	struct bitwriter bw;
	struct stream *s;

	s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s);
	test_params(&sps, &pps, &dec);
	put_i_slice(&bw, true, 0, 2, 8, offsets, 2);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 40);
	put_i_slice(&bw, false, 8, 2, 8, offsets, 2);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);

	parse(test, s, &sps, &pps, &dec, &r);
	KUNIT_EXPECT_EQ(test, r.ret, -EINVAL);
}

/*
 * 192x256 has 3 x 4 = 12 CTBs; slice_segment_address still takes 4 bits,
 * so it can name CTB 12, which does not exist.
 */
static void avd_hevc_parse_rejects_slice_address_past_picture(struct kunit *test)
{
	static const u32 offsets[] = { 15, 31 };
	struct v4l2_ctrl_hevc_decode_params dec;
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct parse_result r;
	struct bitwriter bw;
	struct stream *s;

	s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s);
	test_params(&sps, &pps, &dec);
	sps.pic_width_in_luma_samples = 192;
	put_i_slice(&bw, true, 0, 2, 8, offsets, 2);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);
	put_i_slice(&bw, false, 12, 2, 8, offsets, 2);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);

	parse(test, s, &sps, &pps, &dec, &r);
	KUNIT_EXPECT_EQ(test, r.ret, -EINVAL);
}

/* Two active references, so collocated_ref_idx may be 0 or 1, not 2. */
static void avd_hevc_parse_rejects_collocated_ref_past_list(struct kunit *test)
{
	const struct p_slice p = {
		.num_ref_idx_minus1 = 1,
		.temporal_mvp = true,
		.collocated_ref_idx = 2,
	};
	struct v4l2_ctrl_hevc_decode_params dec;
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct parse_result r;
	struct bitwriter bw;
	struct stream *s;

	s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s);
	test_params(&sps, &pps, &dec);
	sps.flags |= V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED;
	two_refs(&dec);
	put_p_slice(&bw, &p);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);

	parse(test, s, &sps, &pps, &dec, &r);
	KUNIT_EXPECT_EQ(test, r.ret, -EINVAL);
}

/* ChromaLog2WeightDenom = 7 + 1 = 8, outside 0..7 (7.4.7.3). */
static void avd_hevc_parse_rejects_chroma_weight_denom_past_range(struct kunit *test)
{
	const struct p_slice p = {
		.weights = true,
		.luma_log2_weight_denom = 7,
		.delta_chroma_log2_weight_denom = 1,
	};
	struct v4l2_ctrl_hevc_decode_params dec;
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct parse_result r;
	struct bitwriter bw;
	struct stream *s;

	s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s);
	test_params(&sps, &pps, &dec);
	pps.flags |= V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED;
	two_refs(&dec);
	put_p_slice(&bw, &p);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);

	parse(test, s, &sps, &pps, &dec, &r);
	KUNIT_EXPECT_EQ(test, r.ret, -EINVAL);
}

/* poc_st_curr_before[] has 16 entries; a count of 17 reads past it. */
static void avd_hevc_parse_rejects_reference_count_past_dpb(struct kunit *test)
{
	const struct p_slice p = {};
	struct v4l2_ctrl_hevc_decode_params dec;
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct parse_result r;
	struct bitwriter bw;
	struct stream *s;

	s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s);
	test_params(&sps, &pps, &dec);
	dec.num_poc_st_curr_before = V4L2_HEVC_DPB_ENTRIES_NUM_MAX + 1;
	put_p_slice(&bw, &p);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);

	parse(test, s, &sps, &pps, &dec, &r);
	KUNIT_EXPECT_EQ(test, r.ret, -EINVAL);
}

/* Two active DPB entries; a reference to DPB entry 5 names no picture. */
static void avd_hevc_parse_rejects_reference_past_active_dpb(struct kunit *test)
{
	const struct p_slice p = { .num_ref_idx_minus1 = 1 };
	struct v4l2_ctrl_hevc_decode_params dec;
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct parse_result r;
	struct bitwriter bw;
	struct stream *s;

	s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s);
	test_params(&sps, &pps, &dec);
	two_refs(&dec);
	dec.num_active_dpb_entries = 2;
	dec.poc_st_curr_before[1] = 5;
	put_p_slice(&bw, &p);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);

	parse(test, s, &sps, &pps, &dec, &r);
	KUNIT_EXPECT_EQ(test, r.ret, -EINVAL);
}

/*
 * A valid P slice using everything the rejection tests above bend: two
 * active references, a collocated picture, and weights at the denominators'
 * upper bound. By 8.3.4, RefPicList0 = PocStCurrBefore = { 0, 1 }.
 */
static void avd_hevc_parse_p_slice(struct kunit *test)
{
	const struct p_slice p = {
		.num_ref_idx_minus1 = 1,
		.temporal_mvp = true,
		.collocated_ref_idx = 1,
		.weights = true,
		.luma_log2_weight_denom = 7,
		.delta_chroma_log2_weight_denom = 0,
	};
	struct v4l2_ctrl_hevc_decode_params dec;
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct parse_result r;
	struct bitwriter bw;
	struct stream *s;

	s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s);
	test_params(&sps, &pps, &dec);
	sps.flags |= V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED;
	pps.flags |= V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED;
	two_refs(&dec);
	put_p_slice(&bw, &p);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);

	parse(test, s, &sps, &pps, &dec, &r);
	KUNIT_ASSERT_EQ(test, r.ret, 0);
	KUNIT_ASSERT_EQ(test, r.num_slices, 1);
	KUNIT_EXPECT_EQ(test, r.slices[0].slice_type, V4L2_HEVC_SLICE_TYPE_P);
	KUNIT_EXPECT_EQ(test, r.slices[0].num_ref_idx_l0_active_minus1, 1);
	KUNIT_EXPECT_EQ(test, r.slices[0].ref_idx_l0[0], 0);
	KUNIT_EXPECT_EQ(test, r.slices[0].ref_idx_l0[1], 1);
	KUNIT_EXPECT_EQ(test, r.slices[0].collocated_ref_idx, 1);
	KUNIT_EXPECT_EQ(test, r.slices[0].pred_weight_table.luma_log2_weight_denom, 7);
}

/*
 * 4:2:2 is a Range Extensions format; its slice headers may carry fields
 * (cu_chroma_qp_offset_enabled_flag) gated on PPS range extension flags
 * that the V4L2 PPS does not carry, so the parser cannot read them.
 */
static void avd_hevc_parse_rejects_range_extensions_format(struct kunit *test)
{
	static const u32 offsets[] = { 15, 31 };
	struct v4l2_ctrl_hevc_decode_params dec;
	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct parse_result r;
	struct bitwriter bw;
	struct stream *s;

	s = kunit_kzalloc(test, sizeof(*s), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, s);
	test_params(&sps, &pps, &dec);
	sps.chroma_format_idc = 2;
	put_i_slice(&bw, true, 0, 2, 8, offsets, 2);
	put_nal(s, HEVC_NAL_TRAIL_R, &bw, 64);

	parse(test, s, &sps, &pps, &dec, &r);
	KUNIT_EXPECT_EQ(test, r.ret, -EINVAL);
}

static struct kunit_case avd_hevc_parse_cases[] = {
	KUNIT_CASE(avd_hevc_parse_p_slice),
	KUNIT_CASE(avd_hevc_parse_rejects_range_extensions_format),
	KUNIT_CASE(avd_hevc_parse_rejects_reference_past_active_dpb),
	KUNIT_CASE(avd_hevc_parse_rejects_reference_count_past_dpb),
	KUNIT_CASE(avd_hevc_parse_rejects_chroma_weight_denom_past_range),
	KUNIT_CASE(avd_hevc_parse_rejects_collocated_ref_past_list),
	KUNIT_CASE(avd_hevc_parse_rejects_slice_address_past_picture),
	KUNIT_CASE(avd_hevc_parse_two_slices),
	KUNIT_CASE(avd_hevc_parse_rejects_wrapping_entry_point_count),
	KUNIT_CASE(avd_hevc_parse_rejects_entry_points_past_slice_data),
	{}
};

static struct kunit_suite avd_hevc_parse_suite = {
	.name = "apple-avd-hevc-parse",
	.test_cases = avd_hevc_parse_cases,
};

kunit_test_suite(avd_hevc_parse_suite);
