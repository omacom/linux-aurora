// SPDX-License-Identifier: GPL-2.0-only
/*
 * HEVC slice segment headers parsed from the bitstream, for clients that
 * queue a whole picture in Annex B form and set no slice parameters
 * (Chromium's V4L2 stateless decoder does this: it sends SPS, PPS, scaling
 * matrix and decode parameters only, and prefixes each slice NAL unit with a
 * 3-byte start code).
 *
 * The parser fills the same struct v4l2_ctrl_hevc_slice_params array and
 * entry point table a slice-parameter client would, so the rest of the
 * driver is unchanged. Byte offsets and sizes are in the escaped (raw)
 * bitstream, counted from the slice's start code, which keeps the driver's
 * "slices follow each other in the buffer" walk valid.
 */

#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/slab.h>
#include <media/v4l2-ctrls.h>

#include "avd-hevc-parse.h"

/* RBSP bit reader over an escaped NAL unit. */
struct avd_br {
	const u8 *buf;
	size_t len;
	size_t pos;	/* next raw byte to load */
	u32 cur;	/* loaded byte */
	int bits;	/* bits left in cur */
	int zeros;	/* consecutive 0x00 raw bytes just loaded */
	bool err;
};

static void br_init(struct avd_br *br, const u8 *buf, size_t len)
{
	*br = (struct avd_br){ .buf = buf, .len = len };
}

static bool br_load(struct avd_br *br)
{
	if (br->pos >= br->len) {
		br->err = true;
		return false;
	}
	/* 00 00 03: the 03 is an emulation prevention byte. */
	if (br->zeros >= 2 && br->buf[br->pos] == 0x03) {
		br->pos++;
		br->zeros = 0;
		if (br->pos >= br->len) {
			br->err = true;
			return false;
		}
	}
	br->cur = br->buf[br->pos++];
	br->zeros = br->cur ? 0 : br->zeros + 1;
	br->bits = 8;
	return true;
}

static u32 br_u(struct avd_br *br, int n)
{
	u32 v = 0;

	while (n-- > 0) {
		if (!br->bits && !br_load(br))
			return 0;
		v = (v << 1) | ((br->cur >> --br->bits) & 1);
	}
	return v;
}

static void br_skip(struct avd_br *br, u32 n)
{
	while (n--) {
		br_u(br, 1);
		if (br->err)
			return;
	}
}

static u32 br_ue(struct avd_br *br)
{
	int lz = 0;

	while (!br_u(br, 1)) {
		if (br->err || ++lz > 31) {
			br->err = true;
			return 0;
		}
	}
	return ((1u << lz) - 1) + br_u(br, lz);
}

static s32 br_se(struct avd_br *br)
{
	u32 k = br_ue(br);

	return (k & 1) ? (s32)((k + 1) / 2) : -(s32)(k / 2);
}

/* Raw offset of the next unread bit, which must be byte aligned. */
static size_t br_raw_pos(struct avd_br *br)
{
	return br->pos;
}

static int ceil_log2(u32 v)
{
	return v > 1 ? order_base_2(v) : 0;
}

#define HEVC_NAL_BLA_W_LP	16
#define HEVC_NAL_IDR_W_RADL	19
#define HEVC_NAL_IDR_N_LP	20
#define HEVC_NAL_RSV_IRAP_23	23

static bool is_slice_nal(u8 type)
{
	return type <= 9 || (type >= 16 && type <= 21);
}

static int parse_weights(struct avd_br *br, struct v4l2_ctrl_hevc_slice_params *sl,
			 int chroma_array_type)
{
	struct v4l2_hevc_pred_weight_table *pwt = &sl->pred_weight_table;
	int lists = sl->slice_type == V4L2_HEVC_SLICE_TYPE_B ? 2 : 1;
	int chroma_denom;

	pwt->luma_log2_weight_denom = br_ue(br);
	chroma_denom = pwt->luma_log2_weight_denom;
	if (chroma_array_type) {
		pwt->delta_chroma_log2_weight_denom = br_se(br);
		chroma_denom += pwt->delta_chroma_log2_weight_denom;
	}
	/* Both denominators are 0..7 (7.4.7.3). */
	if (pwt->luma_log2_weight_denom > 7 || chroma_denom < 0 || chroma_denom > 7)
		return -EINVAL;

	for (int l = 0; l < lists; l++) {
		int n = (l ? sl->num_ref_idx_l1_active_minus1 :
			     sl->num_ref_idx_l0_active_minus1) + 1;
		s8 *dlw = l ? pwt->delta_luma_weight_l1 : pwt->delta_luma_weight_l0;
		s8 *lo = l ? pwt->luma_offset_l1 : pwt->luma_offset_l0;
		s8(*dcw)[2] = l ? pwt->delta_chroma_weight_l1 : pwt->delta_chroma_weight_l0;
		s8(*co)[2] = l ? pwt->chroma_offset_l1 : pwt->chroma_offset_l0;
		bool luma_flag[V4L2_HEVC_DPB_ENTRIES_NUM_MAX] = {};
		bool chroma_flag[V4L2_HEVC_DPB_ENTRIES_NUM_MAX] = {};

		for (int i = 0; i < n; i++)
			luma_flag[i] = br_u(br, 1);
		if (chroma_array_type)
			for (int i = 0; i < n; i++)
				chroma_flag[i] = br_u(br, 1);

		for (int i = 0; i < n; i++) {
			if (luma_flag[i]) {
				dlw[i] = br_se(br);
				lo[i] = br_se(br);
			}
			if (!chroma_flag[i])
				continue;
			for (int j = 0; j < 2; j++) {
				/* 7.4.7.3, without high precision offsets */
				const int half = 1 << 7;
				int dw = br_se(br);
				int doff = br_se(br);
				int w = (1 << chroma_denom) + dw;

				dcw[i][j] = dw;
				co[i][j] = clamp(half + doff - ((half * w) >> chroma_denom),
						 -half, half - 1);
			}
		}
	}
	return 0;
}

/* 8.3.4: reference picture lists, as DPB indices. */
static void build_ref_lists(struct v4l2_ctrl_hevc_slice_params *sl,
			    const struct v4l2_ctrl_hevc_decode_params *dec,
			    const u8 *entry_l0, bool mod_l0,
			    const u8 *entry_l1, bool mod_l1)
{
	int total = dec->num_poc_st_curr_before + dec->num_poc_st_curr_after +
		    dec->num_poc_lt_curr;
	u8 temp[2 * V4L2_HEVC_DPB_ENTRIES_NUM_MAX];

	if (!total)
		return;

	for (int l = 0; l < 2; l++) {
		int n = (l ? sl->num_ref_idx_l1_active_minus1 :
			     sl->num_ref_idx_l0_active_minus1) + 1;
		int num_temp = max(n, total), r = 0;
		const u8 *first = l ? dec->poc_st_curr_after : dec->poc_st_curr_before;
		const u8 *second = l ? dec->poc_st_curr_before : dec->poc_st_curr_after;
		int nfirst = l ? dec->num_poc_st_curr_after : dec->num_poc_st_curr_before;
		int nsecond = l ? dec->num_poc_st_curr_before : dec->num_poc_st_curr_after;
		u8 *out = l ? sl->ref_idx_l1 : sl->ref_idx_l0;
		const u8 *entry = l ? entry_l1 : entry_l0;
		bool mod = l ? mod_l1 : mod_l0;

		if (l && sl->slice_type != V4L2_HEVC_SLICE_TYPE_B)
			break;
		num_temp = min_t(int, num_temp, ARRAY_SIZE(temp));
		while (r < num_temp) {
			for (int i = 0; i < nfirst && r < num_temp; i++)
				temp[r++] = first[i];
			for (int i = 0; i < nsecond && r < num_temp; i++)
				temp[r++] = second[i];
			for (int i = 0; i < dec->num_poc_lt_curr && r < num_temp; i++)
				temp[r++] = dec->poc_lt_curr[i];
		}
		for (int i = 0; i < n && i < V4L2_HEVC_DPB_ENTRIES_NUM_MAX; i++)
			out[i] = temp[mod ? min_t(int, entry[i], num_temp - 1) : i];
	}
}

/*
 * Parse one slice segment header (7.3.6.1). @nal points at the NAL unit
 * header, @start_off is the raw offset of the NAL unit from the slice's
 * start code. @prev is the previous independent slice segment of the
 * picture, for dependent slice segments.
 */
static int parse_slice(const u8 *nal, size_t nal_len, size_t start_off,
		       const struct v4l2_ctrl_hevc_sps *sps,
		       const struct v4l2_ctrl_hevc_pps *pps,
		       const struct v4l2_ctrl_hevc_decode_params *dec,
		       const struct v4l2_ctrl_hevc_slice_params *prev,
		       struct v4l2_ctrl_hevc_slice_params *sl,
		       u32 *eps, int max_eps, int *num_eps)
{
	int chroma_array_type = (sps->flags & V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE) ?
				0 : sps->chroma_format_idc;
	int ctb_log2 = sps->log2_min_luma_coding_block_size_minus3 + 3 +
		       sps->log2_diff_max_min_luma_coding_block_size;
	u32 ctbs = DIV_ROUND_UP(sps->pic_width_in_luma_samples, 1u << ctb_log2) *
		   DIV_ROUND_UP(sps->pic_height_in_luma_samples, 1u << ctb_log2);
	int total = dec->num_poc_st_curr_before + dec->num_poc_st_curr_after +
		    dec->num_poc_lt_curr;
	u8 entry_l0[V4L2_HEVC_DPB_ENTRIES_NUM_MAX] = {};
	u8 entry_l1[V4L2_HEVC_DPB_ENTRIES_NUM_MAX] = {};
	bool mod_l0 = false, mod_l1 = false, first, dependent = false;
	bool temporal_mvp = false, dbk_disabled, lf_flag_present;
	struct avd_br br;
	u8 nal_type;
	u32 i;

	if (nal_len < 3)
		return -EINVAL;
	nal_type = (nal[0] >> 1) & 0x3f;
	br_init(&br, nal + 2, nal_len - 2);

	first = br_u(&br, 1);
	if (nal_type >= HEVC_NAL_BLA_W_LP && nal_type <= HEVC_NAL_RSV_IRAP_23)
		br_u(&br, 1); /* no_output_of_prior_pics_flag */
	br_ue(&br); /* slice_pic_parameter_set_id */

	if (!first) {
		if (pps->flags & V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED)
			dependent = br_u(&br, 1);
		sl->slice_segment_addr = br_u(&br, ceil_log2(ctbs));
		if (sl->slice_segment_addr >= ctbs)
			return -EINVAL;
	}

	if (dependent) {
		if (!prev)
			return -EINVAL;
		*sl = (struct v4l2_ctrl_hevc_slice_params){
			.slice_type = prev->slice_type,
			.colour_plane_id = prev->colour_plane_id,
			.num_ref_idx_l0_active_minus1 = prev->num_ref_idx_l0_active_minus1,
			.num_ref_idx_l1_active_minus1 = prev->num_ref_idx_l1_active_minus1,
			.collocated_ref_idx = prev->collocated_ref_idx,
			.five_minus_max_num_merge_cand = prev->five_minus_max_num_merge_cand,
			.slice_qp_delta = prev->slice_qp_delta,
			.slice_cb_qp_offset = prev->slice_cb_qp_offset,
			.slice_cr_qp_offset = prev->slice_cr_qp_offset,
			.slice_beta_offset_div2 = prev->slice_beta_offset_div2,
			.slice_tc_offset_div2 = prev->slice_tc_offset_div2,
			.slice_segment_addr = sl->slice_segment_addr,
			.pred_weight_table = prev->pred_weight_table,
			.flags = prev->flags | V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT,
		};
		memcpy(sl->ref_idx_l0, prev->ref_idx_l0, sizeof(sl->ref_idx_l0));
		memcpy(sl->ref_idx_l1, prev->ref_idx_l1, sizeof(sl->ref_idx_l1));
		goto entry_points;
	}

	br_skip(&br, pps->num_extra_slice_header_bits);
	sl->slice_type = br_ue(&br);
	if (sl->slice_type > V4L2_HEVC_SLICE_TYPE_I)
		return -EINVAL;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT)
		br_u(&br, 1); /* pic_output_flag */
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		sl->colour_plane_id = br_u(&br, 2);

	if (nal_type != HEVC_NAL_IDR_W_RADL && nal_type != HEVC_NAL_IDR_N_LP) {
		br_u(&br, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if (!br_u(&br, 1)) /* short_term_ref_pic_set_sps_flag */
			br_skip(&br, dec->short_term_ref_pic_set_size);
		else
			br_skip(&br, ceil_log2(sps->num_short_term_ref_pic_sets));
		if (sps->flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT)
			br_skip(&br, dec->long_term_ref_pic_set_size);
		if (sps->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED)
			temporal_mvp = br_u(&br, 1);
	}
	if (temporal_mvp)
		sl->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED;

	if (sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET) {
		if (br_u(&br, 1))
			sl->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA;
		if (chroma_array_type && br_u(&br, 1))
			sl->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA;
	}

	if (sl->slice_type != V4L2_HEVC_SLICE_TYPE_I) {
		bool b = sl->slice_type == V4L2_HEVC_SLICE_TYPE_B;

		sl->num_ref_idx_l0_active_minus1 = pps->num_ref_idx_l0_default_active_minus1;
		sl->num_ref_idx_l1_active_minus1 = pps->num_ref_idx_l1_default_active_minus1;
		if (br_u(&br, 1)) { /* num_ref_idx_active_override_flag */
			sl->num_ref_idx_l0_active_minus1 = br_ue(&br);
			if (b)
				sl->num_ref_idx_l1_active_minus1 = br_ue(&br);
		}
		if (sl->num_ref_idx_l0_active_minus1 >= V4L2_HEVC_DPB_ENTRIES_NUM_MAX ||
		    sl->num_ref_idx_l1_active_minus1 >= V4L2_HEVC_DPB_ENTRIES_NUM_MAX)
			return -EINVAL;

		if ((pps->flags & V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT) &&
		    total > 1) {
			int bits = ceil_log2(total);

			mod_l0 = br_u(&br, 1);
			if (mod_l0)
				for (i = 0; i <= sl->num_ref_idx_l0_active_minus1; i++)
					entry_l0[i] = br_u(&br, bits);
			if (b) {
				mod_l1 = br_u(&br, 1);
				if (mod_l1)
					for (i = 0; i <= sl->num_ref_idx_l1_active_minus1; i++)
						entry_l1[i] = br_u(&br, bits);
			}
		}
		if (b && br_u(&br, 1))
			sl->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO;
		if ((pps->flags & V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT) && br_u(&br, 1))
			sl->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT;
		if (temporal_mvp) {
			bool from_l0 = true;

			if (b)
				from_l0 = br_u(&br, 1);
			if (from_l0)
				sl->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0;
			u32 max_idx = from_l0 ? sl->num_ref_idx_l0_active_minus1 :
						sl->num_ref_idx_l1_active_minus1;

			if (max_idx) {
				u32 idx = br_ue(&br);

				if (idx > max_idx)
					return -EINVAL;
				sl->collocated_ref_idx = idx;
			}
		}
		if (((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED) && !b) ||
		    ((pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED) && b))
			if (parse_weights(&br, sl, chroma_array_type))
				return -EINVAL;
		sl->five_minus_max_num_merge_cand = br_ue(&br);
		build_ref_lists(sl, dec, entry_l0, mod_l0, entry_l1, mod_l1);
	}

	sl->slice_qp_delta = br_se(&br);
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT) {
		sl->slice_cb_qp_offset = br_se(&br);
		sl->slice_cr_qp_offset = br_se(&br);
	}

	dbk_disabled = pps->flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER;
	sl->slice_beta_offset_div2 = pps->pps_beta_offset_div2;
	sl->slice_tc_offset_div2 = pps->pps_tc_offset_div2;
	if ((pps->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED) &&
	    br_u(&br, 1)) { /* deblocking_filter_override_flag */
		dbk_disabled = br_u(&br, 1);
		if (!dbk_disabled) {
			sl->slice_beta_offset_div2 = br_se(&br);
			sl->slice_tc_offset_div2 = br_se(&br);
		}
	}
	if (dbk_disabled)
		sl->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED;

	/* slice_loop_filter_across_slices_enabled_flag, else as the PPS */
	lf_flag_present = (sl->flags & (V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA |
					V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA)) ||
			  !dbk_disabled;
	if ((pps->flags & V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED) &&
	    (!lf_flag_present || br_u(&br, 1)))
		sl->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED;

entry_points:
	sl->nal_unit_type = nal_type;
	sl->nuh_temporal_id_plus1 = nal[1] & 0x7;
	sl->slice_pic_order_cnt = dec->pic_order_cnt_val;
	sl->short_term_ref_pic_set_size = dec->short_term_ref_pic_set_size;
	sl->long_term_ref_pic_set_size = dec->long_term_ref_pic_set_size;

	if (pps->flags & (V4L2_HEVC_PPS_FLAG_TILES_ENABLED |
			  V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED)) {
		u32 n = br_ue(&br);

		/* A slice has fewer entry points than the picture has CTBs. */
		if (br.err || n >= ctbs || n > (u32)(max_eps - *num_eps))
			return -EINVAL;
		if (n) {
			int len = br_ue(&br) + 1;

			if (br.err || len > 32)
				return -EINVAL;
			for (i = 0; i < n; i++) {
				eps[(*num_eps)++] = br_u(&br, len) + 1;
				if (br.err)
					return -EINVAL;
			}
		}
		sl->num_entry_point_offsets = n;
	}
	if (pps->flags & V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT)
		br_skip(&br, br_ue(&br) * 8);

	/* byte_alignment() */
	if (br_u(&br, 1) != 1)
		return -EINVAL;
	br_skip(&br, br.bits);
	if (br.err)
		return -EINVAL;

	sl->data_byte_offset = start_off + 2 + br_raw_pos(&br);
	return 0;
}

/*
 * The entry points split the slice segment data (escaped bytes, 7.4.7.1),
 * and the last substream must not be empty.
 */
static int check_entry_points(const struct v4l2_ctrl_hevc_slice_params *sl,
			      const u32 *eps)
{
	u32 data_len, i;
	u64 sum = 0;

	if (sl->data_byte_offset >= sl->bit_size / 8)
		return -EINVAL;
	data_len = sl->bit_size / 8 - sl->data_byte_offset;
	for (i = 0; i < sl->num_entry_point_offsets; i++)
		sum += eps[i];
	return sum < data_len ? 0 : -EINVAL;
}

static bool dpb_indices_valid(const u8 *idx, u8 n, u8 num_active)
{
	if (n > V4L2_HEVC_DPB_ENTRIES_NUM_MAX)
		return false;
	for (int i = 0; i < n; i++)
		if (idx[i] >= num_active)
			return false;
	return true;
}

/*
 * The V4L2 core does not bound these; the reference lists are built from
 * them and the driver indexes the DPB by the result.
 */
static bool dec_params_valid(const struct v4l2_ctrl_hevc_decode_params *dec)
{
	u8 active = dec->num_active_dpb_entries;

	return active <= V4L2_HEVC_DPB_ENTRIES_NUM_MAX &&
	       dpb_indices_valid(dec->poc_st_curr_before, dec->num_poc_st_curr_before, active) &&
	       dpb_indices_valid(dec->poc_st_curr_after, dec->num_poc_st_curr_after, active) &&
	       dpb_indices_valid(dec->poc_lt_curr, dec->num_poc_lt_curr, active);
}

static size_t next_start_code(const u8 *buf, size_t len, size_t from)
{
	for (size_t i = from; i + 3 <= len; i++)
		if (!buf[i] && !buf[i + 1] && buf[i + 2] == 1)
			return i;
	return len;
}

int avd_hevc_parse_slices(const u8 *buf, size_t len,
			  const struct v4l2_ctrl_hevc_sps *sps,
			  const struct v4l2_ctrl_hevc_pps *pps,
			  const struct v4l2_ctrl_hevc_decode_params *dec,
			  struct v4l2_ctrl_hevc_slice_params **slices, int *num_slices,
			  u32 **entry_points, int *num_entry_points)
{
	const int max_slices = 600, max_eps = 4096;
	struct v4l2_ctrl_hevc_slice_params *sl, *prev = NULL;
	size_t sc = next_start_code(buf, len, 0), slice_start = 0, new_start;
	int n = 0, neps = 0, ret = 0;
	u32 *eps;

	/*
	 * Main and Main 10 only: Range Extensions slice headers may hold
	 * fields gated on PPS flags that the V4L2 PPS does not carry.
	 */
	if (sps->chroma_format_idc != 1 || sps->bit_depth_luma_minus8 > 2 ||
	    sps->bit_depth_chroma_minus8 > 2)
		return -EINVAL;
	if (!dec_params_valid(dec))
		return -EINVAL;

	sl = kvcalloc(max_slices, sizeof(*sl), GFP_KERNEL);
	eps = kvcalloc(max_eps, sizeof(*eps), GFP_KERNEL);
	if (!sl || !eps) {
		ret = -ENOMEM;
		goto err;
	}

	while (sc < len) {
		size_t nal = sc + 3;
		size_t next = next_start_code(buf, len, nal);
		size_t nal_end = next;
		u8 type;

		if (nal >= len)
			break;
		/* A 4-byte start code's leading zero belongs to the next unit. */
		while (nal_end > nal && !buf[nal_end - 1])
			nal_end--;
		type = (buf[nal] >> 1) & 0x3f;

		if (is_slice_nal(type)) {
			if (n == max_slices) {
				ret = -EINVAL;
				goto err;
			}
			/*
			 * The first slice starts at the buffer start, later
			 * ones at their start code: the driver walks slices
			 * as consecutive spans of bit_size / 8 bytes.
			 */
			new_start = n ? sc : 0;
			ret = parse_slice(buf + nal, nal_end - nal, nal - new_start,
					  sps, pps, dec, prev, &sl[n], eps, max_eps, &neps);
			if (ret)
				goto err;
			if (n)
				sl[n - 1].bit_size = (new_start - slice_start) * 8;
			slice_start = new_start;
			if (!(sl[n].flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT))
				prev = &sl[n];
			n++;
		}
		sc = next;
	}
	if (!n) {
		ret = -EINVAL;
		goto err;
	}
	sl[n - 1].bit_size = (len - slice_start) * 8;

	for (int i = 0, e = 0; i < n; e += sl[i].num_entry_point_offsets, i++) {
		ret = check_entry_points(&sl[i], eps + e);
		if (ret)
			goto err;
	}

	*slices = sl;
	*num_slices = n;
	*entry_points = eps;
	*num_entry_points = neps;
	return 0;
err:
	kvfree(sl);
	kvfree(eps);
	return ret;
}
