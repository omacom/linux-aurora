/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Apple AVD VPU codec driver
 *
 * Copyright The Asahi Linux Contributors
 * Copyright 2023 Eileen Yoon <eyn@gmx.com>
 *
 * Copyright (c) 2014 Rockchip Electronics Co., Ltd.
 *	Hertz Wong <hertz.wong@rock-chips.com>
 *	Herman Chen <herman.chen@rock-chips.com>
 *
 * Copyright (C) 2014 Google, Inc.
 *	Tomasz Figa <tfiga@chromium.org>
 */

#include <kunit/visibility.h>
#include <linux/dev_printk.h>
#include <linux/mm.h>

#include <media/v4l2-h264.h>
#include <media/videobuf2-dma-contig.h>

#include "avd.h"
#include "avd-inst.h"
#include "avd-h264-params.h"

#define H264_SCL_DIMS	(0x1000000 | ((64 / 4) << 5) | (((16 / 4) << 5) - 1))

#define h264_FLAG_CI_PRED(v)			FIELD_PREP(BIT(19), !!(v))
#define H264_FLAG_ENTROPY_CODING_MODE(v)	FIELD_PREP(BIT(20), !!(v))
#define H264_FLAG_NOT_IDR(v)			FIELD_PREP(BIT(21), !!(v))

#define H264_TRANSFORM_8X8_MODE(v)		FIELD_PREP(BIT(7), !!(v))

/*
 * Most slices held for one picture. This bounds the slice table, the job
 * table and the memory that a client can pin.
 */
#define MAX_SLICES	4096

struct avd_h264_run {
	struct avd_run base;

	const struct v4l2_ctrl_h264_decode_params *decode_params;
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_slice_params *slice_params;

	const struct v4l2_ctrl_h264_scaling_matrix *scaling_matrix;
	const struct v4l2_ctrl_h264_pred_weights *pred_weights;

	struct run_addr {
		dma_addr_t mv_color;
	} addresses;

	s32 cur_poc;
	u8 num_valid;
	u32 slice_data_off;
};

/*
 * One bank per picture in flight. With the queued transport two pictures of
 * a stream overlap (VP of one, PP of the other), so everything the hardware
 * reads or writes while decoding a picture, and the CPU copies of its slices,
 * exists once per bank. The legacy transport uses a single bank.
 */
#define AVD_H264_BANKS	2

struct avd_bank {
	bool monochrome;
	bool busy;		/* a picture of this bank is not retired yet */
	bool submitted;		/* ... and its last slice went to the hardware */
	struct avd_buf *slices;
	struct avd_buf *active_slice;
	size_t slice_num;
	size_t alloc_slice_num;
	/* allocated size of the held slices */
	size_t slice_bytes;
	u32 last_mb;

	struct avd_h264_bufs {
		/* VP to PP instruction FIFO for pictures of this bank */
		struct avd_buf inst;
		struct avd_buf pipe_state;
		struct avd_buf above_info;
		struct avd_buf lf_above_info;
		struct avd_buf lf_above;
		struct avd_buf ip_above;
		struct avd_buf mv_above_info;
	} bufs;
};

/* state */
struct avd_h264_ctx {
	struct avd_h264_reflists {
		struct v4l2_h264_reference p[V4L2_H264_REF_LIST_LEN];
		struct v4l2_h264_reference b0[V4L2_H264_REF_LIST_LEN];
		struct v4l2_h264_reference b1[V4L2_H264_REF_LIST_LEN];
	} reflists;

	unsigned int nbanks;
	struct avd_bank bank[AVD_H264_BANKS];
	/* bank of the picture being built */
	struct avd_bank *b;
};

/* ffmpeg submits wrong timestamp, use first_mb_in_slice as a workaround */
#define is_new_frame(sl) \
	(sl->first_mb_in_slice == 0) /* (ctx->fh.m2m_ctx->new_frame) */

/* scaling matrix */
static const u32 default_8x8_intra[] = {
	0x060a0d10, 0x0a0b1012, 0x0d101217, 0x10121719, 0x1217191b, 0x17191b1d,
	0x191b1d1f, 0x1b1d1f21, 0x1217191b, 0x17191b1d, 0x191b1d1f, 0x1b1d1f21,
	0x1d1f2124, 0x1f212426, 0x21242628, 0x2426282a,
};
static const u32 default_8x8_inter[] = {
	0x090d0f11, 0x0d0d1113, 0x0f111315, 0x11131516, 0x13151618, 0x15161819,
	0x1618191b, 0x18191b1c, 0x13151618, 0x15161819, 0x1618191b, 0x18191b1c,
	0x191b1c1e, 0x1b1c1e20, 0x1c1e2021, 0x1e202123,
};

static inline u32 mv_color_size(u32 w, u32 h)
{
	return (DIV_ROUND_UP(w, 16) + 1) * (DIV_ROUND_UP(h, 16) + 1) * 64;
}

/* sorry for the formatting */

static void stream_refs(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	const struct v4l2_ctrl_h264_decode_params *decode = run->decode_params;
	const struct v4l2_h264_dpb_entry *dpb = decode->dpb;
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct avd_decoded_buffer *dst, *ref;
	dma_addr_t addr;

	dst = vb2_to_avd_decoded_buf(&run->base.bufs.dst->vb2_buf);

	push(0, "");
	pusha(h264_ctx->b->bufs.mv_above_info.addr, "mv_above_info", 7);
	pusha(run->addresses.mv_color, "mv_color", 0);

	push(0, "");
	push(0, "");
	push(0, "");
	push(0, "");

	for (int i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		if (!(dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;

		ref = avd_get_ref_buf(ctx, &dst->base.vb, dpb[i].reference_ts);

		addr = vb2_dma_contig_plane_dma_addr(&ref->base.vb.vb2_buf, 0) +
		       ref->comp.start_offset;

		push(AVD_REF_NUM(run->num_valid - 1) | AVD_REF_FLAG_CONST |
			     AVD_REF_FLAG_LONG(
				     dpb[i].flags &
				     V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) |
			     AVD_REF_DELTA_POC(run->cur_poc -
					       dpb[i].top_field_order_cnt),
		     "hdr_d0_ref_hdr");

		push_comp(ctx, addr, ctx->comp.offsets);
	}
}

static void stream_scaling(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	const struct v4l2_ctrl_h264_scaling_matrix *scaling =
		run->scaling_matrix;

	push(H264_SCL_DIMS, "hdr_4c_pic_scaling_list_dims");

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 16; j += 4)
			push(AVD_SCALING_I3(
				     scaling->scaling_list_4x4[i][j + 0]) |
				     AVD_SCALING_I2(
					     scaling->scaling_list_4x4[i][j + 1]) |
				     AVD_SCALING_I1(
					     scaling->scaling_list_4x4[i][j + 2]) |
				     AVD_SCALING_I0(
					     scaling->scaling_list_4x4[i][j + 3]),
			     "scl_46c_pic_scaling_matrix_4x4");

	/* Instead of 8x8 raster scan order avd expects 4 4x4 subblocks */
	static const u8 map[16] = {
		0,  8,	16, 24, /* top left */
		4,  12, 20, 28, /* top right */
		32, 40, 48, 56, /* bottom left */
		36, 44, 52, 60, /* bottom right */
	};

	/* 7.3.2.2, only matrix 0 and 1 are used if chroma_format_idc < 3 */
	if (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) {
		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 16; j++)
				push(AVD_SCALING_I3(scaling->scaling_list_8x8
							    [i][map[j] + 0]) |
					     AVD_SCALING_I2(
						     scaling->scaling_list_8x8
							     [i][map[j] + 1]) |
					     AVD_SCALING_I1(
						     scaling->scaling_list_8x8
							     [i][map[j] + 2]) |
					     AVD_SCALING_I0(
						     scaling->scaling_list_8x8
							     [i][map[j] + 3]),
				     "scl_4cc_pic_scaling_matrix_8x8");

	} else {
		for (int i = 0; i < ARRAY_SIZE(default_8x8_intra); i++)
			push(default_8x8_intra[i],
			     "scl_4cc_pic_scaling_matrix_8x8");
		for (int i = 0; i < ARRAY_SIZE(default_8x8_inter); i++)
			push(default_8x8_inter[i],
			     "scl_4cc_pic_scaling_matrix_8x8");
	}
}

static bool avd_h264_intra_header(const struct avd_h264_run *run)
{
	/*
	 * frame_num gaps can evict every real reference. A non-IDR I picture
	 * still decodes without references, but an inter header has no way to
	 * encode an empty reference table: its count lives in the first entry.
	 * Use the same reference-free header as an IDR picture in that case.
	 */
	return (run->decode_params->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) ||
	       (!run->num_valid &&
		run->slice_params->slice_type == V4L2_H264_SLICE_TYPE_I);
}

static void stream_hdr(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	const struct v4l2_ctrl_h264_sps *sps = run->sps;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	struct avd_dev *avd = ctx->dev;
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	u32 bytesperline;
	u32 width = (sps->pic_width_in_mbs_minus1 + 1) * 16;
	u32 height = (sps->pic_height_in_map_units_minus1 + 1) * 16;
	bool intra_header = avd_h264_intra_header(run);

	push(AVD_OP_HDR | AVD_OP_HDR_FLAG_DECOMP(ctx->decomp) |
		     AVD_OP_HDR_FLAG_INTRA(intra_header) |
		     AVD_OP_HDR_CONST |
		     AVD_OP_HDR_FLAG_PIPE_STATE(
			     !(avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE)),
	     "hdr_34_start_hdr");

	push(AVD_HDR_CODEC_MODE(AVD_CODEC_H264), "hdr_38_mode");

	push(AVD_HDR_HEIGHT(height - 1) | AVD_HDR_WIDTH(width - 1),
	     "hdr_3c_height_width");

	push(0, "hdr_40_zero");

	push(AVD_HDR_HEIGHT((height - 1) >> 3) |
		     AVD_HDR_WIDTH((width - 1) >> 3),
	     "hdr_28_height_width_shift3");

	/* TODO: did i mix up luma/chroma in hevc or here? */
	push(AVD_HDR_COMMON_CHROMA_FORMAT(sps->chroma_format_idc) |
		     AVD_HDR_COMMON_BIT_DEPTH_L(sps->bit_depth_luma_minus8) |
		     AVD_HDR_COMMON_BIT_DEPTH_C(sps->bit_depth_chroma_minus8) |
		     AVD_HDR_COMMON_MIN_LUMA_CBS(1) |
		     AVD_HDR_COMMON_LUMA_CBS(1) |
		     H264_TRANSFORM_8X8_MODE(
			     pps->flags &
			     V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) |
		     AVD_HDR_COMMON_FLAG0(
			     sps->flags &
			     V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE),
	     "hdr_2c_sps_param");

	push(H264_FLAG_ENTROPY_CODING_MODE(
		     pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) |
		     H264_FLAG_NOT_IDR(!intra_header) |
		     h264_FLAG_CI_PRED(
			     pps->flags &
			     V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED),
	     "hdr_44_flags");

	push(AVD_HDR_H26X_QP_OFFSET_CB(pps->chroma_qp_index_offset) |
		     AVD_HDR_H26X_QP_OFFSET_CR(
			     pps->second_chroma_qp_index_offset),
	     "hdr_48_chroma_qp_index_offset");

	push(AVD_HDR_FEAT_H26X | AVD_HDR_FEAT_COMMON | AVD_HDR_FEAT_H264 |
		     AVD_HDR_FEAT_PIPE_STATE_EN(
			     !(avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE)),
	     "hdr_58_const_3a");

	push(0, "");
	push(0, "");

	if (avd->variant->revision == 3)
		push(0, "zero");

	pusha(h264_ctx->b->bufs.above_info.addr, "hdr_9c_pps_tile_addr_lsb8", 0);

	push(0, "");
	push(0, "");

	if (avd->variant->revision == 3)
		push(0, "zero");
	else if (!(avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE))
		pusha(h264_ctx->b->bufs.pipe_state.addr, "pipe_state", 0);

	pusha(h264_ctx->b->bufs.ip_above.addr, "ip_above", 1);
	pusha(h264_ctx->b->bufs.lf_above.addr, "lf_above", 2);
	pusha(h264_ctx->b->bufs.lf_above_info.addr, "lf_above_info", 3);
	push(0, "");

	push_comp(ctx, run->base.comp_out, ctx->comp.offsets);

	bytesperline = ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
	if (avd->variant->quirks & AVD_QUIRK_LSR)
		bytesperline = bytesperline >> 4;

	pusha(run->base.y_out, "hdr_210_y_addr_lsb8", 0);
	push(bytesperline, "hdr_218_width_align");
	pusha(run->base.uv_out, "hdr_214_uv_addr_lsb8", 0);
	push(bytesperline, "hdr_21c_width_align");

	push(0, "cm3_mark_end_section");
	push(((height - 1) << 16) | (width - 1), "hdr_54_height_width");

	if (!intra_header)
		stream_refs(ctx, run);

	if (pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT)
		stream_scaling(ctx, run);
	else
		push(0, "cm3_mark_end_section_scl");
}

#define DEFAULT_WEIGHT_DENOM \
	(AVD_OP_WEIGHTS_HDR_LUMA(5) | AVD_OP_WEIGHTS_HDR_CHROMA(5))

static void stream_weights(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	int luma_denom, chroma_denom;
	struct v4l2_h264_weight_factors factors;
	const struct v4l2_ctrl_h264_pred_weights *weights = run->pred_weights;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	const struct v4l2_ctrl_h264_slice_params *sl = run->slice_params;

	bool pred_weight_req = V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(pps, sl);
	bool default_weights = pps->weighted_bipred_idc == 2 &&
			       !pred_weight_req;

	push(AVD_OP_WEIGHTS_HDR | AVD_OP_WEIGHTS_HDR_FLAG1(default_weights) |
		     AVD_OP_WEIGHTS_HDR_FLAG0(pred_weight_req) |
		     AVD_OP_WEIGHTS_HDR_LUMA(
			     !default_weights ?
				     weights->luma_log2_weight_denom :
				     0) |
		     AVD_OP_WEIGHTS_HDR_CHROMA(
			     !default_weights ?
				     weights->chroma_log2_weight_denom :
				     0) |
		     (default_weights ? DEFAULT_WEIGHT_DENOM : 0),
	     "slc_76c_cmd_weights_denom");

	if (!pred_weight_req)
		return;

	luma_denom = 1 << weights->luma_log2_weight_denom;
	chroma_denom = 1 << weights->chroma_log2_weight_denom;

	for (int y = 0; y < 2; y++) {
		if (y == 1 && sl->slice_type != V4L2_H264_SLICE_TYPE_B)
			break;

		factors = weights->weight_factors[y];
		int to = y == 0 ? sl->num_ref_idx_l0_active_minus1 :
				  sl->num_ref_idx_l1_active_minus1;
		for (int i = 0; i < to + 1; i++) {
			/* Avd only expects offsets/weights if they are not the default
			 * ones, otherwise we get artifacts */
			if ((factors.luma_weight[i] != luma_denom) ||
			    (factors.luma_offset[i] != 0)) {
				push(AVD_OP_WEIGHTS | AVD_OP_WEIGHTS_IDENT(1) |
					     AVD_OP_WEIGHTS_LIST_IDX(y) |
					     AVD_OP_WEIGHTS_INDEX(i) |
					     AVD_OP_WEIGHTS_WEIGHT(
						     factors.luma_weight[i]),
				     "slc_luma_weights");
				push(AVD_OP_OFFSETS |
					     AVD_OP_OFFSETS_OFFSET(
						     factors.luma_offset[i]),
				     "slc_luma_offsets");
			}

			if ((factors.chroma_weight[i][0] != chroma_denom) ||
			    (factors.chroma_offset[i][0] != 0) ||
			    (factors.chroma_weight[i][1] != chroma_denom) ||
			    (factors.chroma_offset[i][1] != 0)) {
				push(AVD_OP_WEIGHTS | AVD_OP_WEIGHTS_IDENT(2) |
					     AVD_OP_WEIGHTS_LIST_IDX(y) |
					     AVD_OP_WEIGHTS_INDEX(i) |
					     AVD_OP_WEIGHTS_WEIGHT(
						     factors.chroma_weight[i][0]),
				     "slc_chroma_weights[0]");
				push(AVD_OP_OFFSETS |
					     AVD_OP_OFFSETS_OFFSET(
						     factors.chroma_offset[i][0]),
				     "slc_chroma_offsets[0]");
				push(AVD_OP_WEIGHTS | AVD_OP_WEIGHTS_IDENT(3) |
					     AVD_OP_WEIGHTS_LIST_IDX(y) |
					     AVD_OP_WEIGHTS_INDEX(i) |
					     AVD_OP_WEIGHTS_WEIGHT(
						     factors.chroma_weight[i][1]),
				     "slc_chroma_weights[1]");
				push(AVD_OP_OFFSETS |
					     AVD_OP_OFFSETS_OFFSET(
						     factors.chroma_offset[i][1]),
				     "slc_chroma_offsets[1]");
			}
		}
	}
}

/*
 * Find the slice data in a slice: skip the two bytes in front of the slice
 * header, then the header_len bytes of the header itself, which do not count
 * its emulation prevention bytes. At least one byte of slice data must follow.
 */
VISIBLE_IF_KUNIT bool avd_h264_find_slice_data(const u8 *data, u32 len,
					       u32 header_len, u32 *off)
{
	u32 pos = 2, bytes_read = 2;

	while (bytes_read < header_len && pos < len) {
		if (data[pos - 2] != 0x00 || data[pos - 1] != 0x00 ||
		    data[pos] != 0x03)
			bytes_read++;
		pos++;
	}

	if (pos >= len)
		return false;

	*off = pos;
	return true;
}

/* Bytes of slice header in front of the slice data. */
static u32 avd_h264_header_len(const struct avd_h264_run *run)
{
	/* CABAC slice data starts on a byte boundary. */
	bool cabac = run->pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;

	return (run->slice_params->header_bit_size + (cabac ? 7 : 0)) / 8;
}

static void stream_slice(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	const struct v4l2_ctrl_h264_decode_params *decode = run->decode_params;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	const struct v4l2_ctrl_h264_sps *sps = run->sps;
	const struct v4l2_ctrl_h264_slice_params *sl = run->slice_params;
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	u32 payload_len = h264_ctx->b->active_slice->size;
	bool en_mode = (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) ==
		       0;
	u32 off = run->slice_data_off;

	dma_addr_t slc_a84 = h264_ctx->b->active_slice->addr + off;

	push(AVD_OP_CODED_DATA |
		     AVD_OP_CODED_DATA_BIT_OFF(
			     en_mode ? (sl->header_bit_size % 8) : 0) |
		     AVD_OP_CODED_DATA_ADDR(slc_a84 >> 32),
	     "slc_a7c_cmd_set_coded_slice");
	push((u32)(slc_a84 & 0xffffffff), "slc_a84_slice_addr_low");

	/* should not include trailing 0? */
	push(payload_len - off, "slc_a88_slice_hdr_size");

	push(AVD_OP_SL_LOC |
		     AVD_OP_SL_LOC_Y(sl->first_mb_in_slice /
				     (sps->pic_width_in_mbs_minus1 + 1)) |
		     AVD_OP_SL_LOC_X(sl->first_mb_in_slice %
				     (sps->pic_width_in_mbs_minus1 + 1)),
	     "cm3_cmd_exec_mb_vp");

	push(AVD_OP_QP | AVD_OP_QP_VAL(26 + pps->pic_init_qp_minus26 +
				       sl->slice_qp_delta),
	     "slc_a70_cmd_quant_param");

	push(AVD_OP_DBLK |
		     AVD_OP_DBLK_FLAG_FULL_EN(
			     sl->disable_deblocking_filter_idc == 0) |
		     AVD_OP_DBLK_FLAG_EN(sl->disable_deblocking_filter_idc !=
					 1) |
		     AVD_OP_DBLK_OFF1(sl->slice_beta_offset_div2) |
		     AVD_OP_DBLK_OFF0(sl->slice_alpha_c0_offset_div2),
	     "slc_a74_cmd_deblocking_filter");

	if (sl->slice_type == V4L2_H264_SLICE_TYPE_P ||
	    sl->slice_type == V4L2_H264_SLICE_TYPE_B) {
		u32 num_ref_idx_active = sl->num_ref_idx_l0_active_minus1 + 1;
		for (u32 i = 0; i < num_ref_idx_active; i++)
			push(AVD_OP_REF | AVD_OP_REF_LIST_IDX(0) |
				     AVD_OP_REF_LOOP_IDX(i) |
				     AVD_OP_REF_DBP_IDX(
					     avd_h264_ref_slot(decode,
							       sl->ref_pic_list0[i].index)),
			     "slc_6e8_cmd_ref_list_0");

		if (sl->slice_type == V4L2_H264_SLICE_TYPE_B) {
			u32 num_ref_idx_active =
				sl->num_ref_idx_l1_active_minus1 + 1;
			for (u32 i = 0; i < num_ref_idx_active; i++)
				push(AVD_OP_REF | AVD_OP_REF_LIST_IDX(1) |
					     AVD_OP_REF_LOOP_IDX(i) |
					     AVD_OP_REF_DBP_IDX(
						     avd_h264_ref_slot(decode,
								       sl->ref_pic_list1[i].index)),
				     "slc_6e8_cmd_ref_list_0");
		}
		stream_weights(ctx, run);
	}

	if (sl->first_mb_in_slice == 0) {
		push(AVD_OP_SL_DIM_START, "cm3_cmd_set_mb_dims");
		push(AVD_SL_DIM_END_Y(sps->pic_height_in_map_units_minus1) |
			     AVD_SL_DIM_END_X(sps->pic_width_in_mbs_minus1),
		     "cm3_set_mb_dims");
	}

	push(AVD_OP_SL_REF | AVD_OP_SL_REF_FLAG_CABAC(sl->cabac_init_idc == 1) |
		     AVD_OP_SL_REF_FLAG1(sl->cabac_init_idc == 2) |
		     AVD_OP_SL_REF_FLAG2(
			     !(sl->flags &
			       V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED)) |
		     AVD_OP_SL_REF_NUM_L0(sl->num_ref_idx_l0_active_minus1) |
		     AVD_OP_SL_REF_NUM_L1(sl->num_ref_idx_l1_active_minus1) |
		     AVD_OP_SL_REF_SLICE_P(sl->slice_type ==
					   V4L2_H264_SLICE_TYPE_P) |
		     AVD_OP_SL_REF_SLICE_I(sl->slice_type ==
					   V4L2_H264_SLICE_TYPE_I) |
		     AVD_OP_SL_REF_SLICE_B(sl->slice_type ==
					   V4L2_H264_SLICE_TYPE_B)

		     ,
	     "slc_6e4_cmd_ref_type");

	if (sl->slice_type == V4L2_H264_SLICE_TYPE_B) {
		/* bidirectional reference of previous mv */
		struct vb2_buffer *vb = vb2_find_buffer(
			&ctx->fh.m2m_ctx->cap_q_ctx.q,
			decode->dpb[sl->ref_pic_list1[0].index].reference_ts);

		dma_addr_t mv_color_addr =
			vb ? vb2_dma_contig_plane_dma_addr(vb, 0) +
					(vb->planes[0].length -
					 mv_color_size(fmt_width(ctx),
						       fmt_height(ctx))) :
			     run->addresses.mv_color;

		pusha(mv_color_addr, "slc_a78_sps_tile_addr2_lsb8", 0);
	}
}

static int avd_h264_alloc_bufs(struct avd_ctx *ctx, struct avd_bank *bank)
{
	struct avd_dev *dev = ctx->dev;
	int ret, w, bit_depth, mb;
	w = fmt_width(ctx);
	bit_depth = (ctx->image_fmt == AVD_IMG_FMT_420_10BIT ||
		     ctx->image_fmt == AVD_IMG_FMT_422_10BIT) ?
			    10 :
			    8;

	ret = avd_buf_alloc(dev, &bank->bufs.inst, fifo_size());
	if (ret) {
		return ret;
	}

	if (!(dev->variant->quirks & AVD_QUIRK_NO_PIPE_STATE)) {
		ret = avd_buf_alloc(dev, &bank->bufs.pipe_state, 0x200);
		if (ret) {
			return ret;
		}
	}

	mb = DIV_ROUND_UP(w, 16);

	ret = avd_buf_alloc(dev, &bank->bufs.above_info, mb * 20);
	if (ret)
		return ret;

	ret = avd_buf_alloc(dev, &bank->bufs.ip_above, bit_depth * 4 * mb);
	if (ret)
		return ret;

	ret = avd_buf_alloc(dev, &bank->bufs.lf_above,
			    bit_depth * 4 * 4 * mb);
	if (ret)
		return ret;

	ret = avd_buf_alloc(dev, &bank->bufs.lf_above_info, 32 * mb);
	if (ret)
		return ret;

	ret = avd_buf_alloc(dev, &bank->bufs.mv_above_info, 32 * mb);
	if (ret)
		return ret;

	return 0;
}

static int avd_h264_validate_pps(struct avd_ctx *ctx,
				 const struct v4l2_ctrl_h264_pps *pps)
{
	if (pps->num_slice_groups_minus1 != 0) {
		dev_err_ratelimited(ctx->dev->dev,
				    "H.264 slice groups (FMO) are unsupported: no slice-group map in the stateless API\n");
		return -EINVAL;
	}

	return 0;
}

static int avd_h264_validate_slice(struct avd_ctx *ctx,
				   const struct v4l2_ctrl_h264_slice_params *sl)
{
	if (!avd_h264_slice_type_supported(sl)) {
		dev_err_ratelimited(ctx->dev->dev,
				    "H.264 slice type %u is unsupported (SP/SI switching pictures)\n",
				    sl->slice_type);
		return -EINVAL;
	}

	return 0;
}

static int avd_h264_validate_sps(struct avd_ctx *ctx,
				 const struct v4l2_ctrl_h264_sps *sps)
{
	if (sps->chroma_format_idc > 2)
		/* Only 4:0:0, 4:2:0 and 4:2:2 are supported */
		return -EINVAL;
	if (sps->bit_depth_luma_minus8 != sps->bit_depth_chroma_minus8) {
		dev_err_ratelimited(ctx->dev->dev,
				    "H.264 unequal luma/chroma bit depths (%u/%u) are unsupported\n",
				    sps->bit_depth_luma_minus8 + 8,
				    sps->bit_depth_chroma_minus8 + 8);
		return -EINVAL;
	}
	if (sps->bit_depth_luma_minus8 != 0 && sps->bit_depth_luma_minus8 != 2)
		/* Only 8 and 10 bit have capture formats */
		return -EINVAL;
	if (sps->chroma_format_idc == 2 && sps->bit_depth_luma_minus8 == 2)
		/* 4:2:2 10 bit would need P210; P010 is too small for its chroma */
		return -EINVAL;
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
		/* no interlaced support */
		return -EINVAL;

	return 0;
}

/* Free the slices held for one bank's picture. */
static void avd_h264_drop_bank_slices(struct avd_ctx *ctx, struct avd_bank *bank)
{
	for (size_t i = 0; i < bank->slice_num; i++)
		avd_buf_free(ctx->dev, &bank->slices[i]);
	bank->slice_num = 0;
	bank->slice_bytes = 0;
	bank->last_mb = 0;
}

static void avd_h264_stop(struct avd_ctx *ctx)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct avd_dev *dev = ctx->dev;
	unsigned int i;

	ctx->max_frames = 1;
	if (!h264_ctx)
		return;

	for (i = 0; i < AVD_H264_BANKS; i++) {
		struct avd_bank *bank = &h264_ctx->bank[i];

		avd_buf_free(dev, &bank->bufs.pipe_state);
		avd_buf_free(dev, &bank->bufs.inst);
		avd_buf_free(dev, &bank->bufs.above_info);
		avd_buf_free(dev, &bank->bufs.lf_above_info);
		avd_buf_free(dev, &bank->bufs.lf_above);
		avd_buf_free(dev, &bank->bufs.ip_above);
		avd_buf_free(dev, &bank->bufs.mv_above_info);
		avd_h264_drop_bank_slices(ctx, bank);
		kfree(bank->slices);
	}

	kfree(h264_ctx);
	ctx->priv = NULL;
}

static int avd_h264_start(struct avd_ctx *ctx)
{
	struct avd_h264_ctx *h264_ctx;
	struct v4l2_ctrl *ctrl;
	int ret;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_H264_SPS);
	if (!ctrl)
		return -EINVAL;

	ret = avd_h264_validate_sps(ctx, ctrl->p_new.p_h264_sps);
	if (ret)
		return ret;

	h264_ctx = kzalloc(sizeof(*h264_ctx), GFP_KERNEL);
	if (!h264_ctx)
		return -ENOMEM;

	ctx->priv = h264_ctx;
	/* Overlap pictures only where the firmware chains VP to PP itself. */
	h264_ctx->nbanks = avd_backend_is_qfw(ctx->dev) ? AVD_H264_BANKS : 1;
	h264_ctx->b = &h264_ctx->bank[0];

	for (unsigned int i = 0; i < h264_ctx->nbanks; i++) {
		struct avd_bank *bank = &h264_ctx->bank[i];

		/* assume one slice per row */
		bank->alloc_slice_num =
			ctrl->p_new.p_h264_sps->pic_height_in_map_units_minus1 + 1;
		bank->slices = kcalloc(bank->alloc_slice_num,
				       sizeof(*bank->slices), GFP_KERNEL);
		if (!bank->slices) {
			ret = -ENOMEM;
			goto err_free_ctx;
		}

		ret = avd_h264_alloc_bufs(ctx, bank);
		if (ret)
			goto err_free_ctx;
	}
	ctx->max_frames = h264_ctx->nbanks;

	return 0;

err_free_ctx:
	avd_h264_stop(ctx);
	return ret;
}

static void avd_h264_run_preamble(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	struct v4l2_ctrl *ctrl;
	u32 dst_len, mv_color_len;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	run->decode_params = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_H264_SPS);
	run->sps = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_H264_PPS);
	run->pps = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_H264_SLICE_PARAMS);
	run->slice_params = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_H264_SCALING_MATRIX);
	run->scaling_matrix = ctrl ? ctrl->p_cur.p : NULL;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl,
			      V4L2_CID_STATELESS_H264_PRED_WEIGHTS);
	run->pred_weights = ctrl ? ctrl->p_cur.p : NULL;

	avd_run_preamble(ctx, &run->base);

	dst_len = run->base.bufs.dst->vb2_buf.planes[0].length;

	mv_color_len = mv_color_size(fmt_width(ctx), fmt_height(ctx));

	run->addresses.mv_color = run->base.y_out + (dst_len - mv_color_len);
}

static int avd_h264_realloc_slices(struct avd_ctx *ctx, size_t max_slices)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	void *tmp;
	size_t alloc_slice_num;

	tmp = h264_ctx->b->slices;
	/* grow by half, and by at least one for a one-row picture */
	alloc_slice_num = h264_ctx->b->alloc_slice_num +
			  max_t(size_t, h264_ctx->b->alloc_slice_num / 2, 1);
	alloc_slice_num = min(alloc_slice_num, max_slices);
	h264_ctx->b->slices = kcalloc(alloc_slice_num, sizeof(*h264_ctx->b->slices),
				   GFP_KERNEL);
	if (!h264_ctx->b->slices) {
		/* make deallocating a little easier */
		h264_ctx->b->slices = tmp;
		return -ENOMEM;
	}

	h264_ctx->b->alloc_slice_num = alloc_slice_num;
	memcpy(h264_ctx->b->slices, tmp,
	       sizeof(*h264_ctx->b->slices) * h264_ctx->b->slice_num);
	kfree(tmp);

	/* a slice that arrives before the frame's header has no job table yet */
	if (!ctx->job.segments)
		return 0;

	tmp = ctx->job.segments;
	ctx->job.segments = kvcalloc(alloc_slice_num + 1,
				     sizeof(*ctx->job.segments), GFP_KERNEL);
	if (!ctx->job.segments) {
		ctx->job.segments = tmp;
		return -ENOMEM;
	}

	memcpy(ctx->job.segments, tmp,
	       sizeof(*ctx->job.segments) * (ctx->job.num + 1));
	kvfree(tmp);

	return 0;
}

/*
 * Check that every reference in the slice's ref lists points at a DPB entry
 * that is VALID. stream_refs() only pushes a reference header (and its RVRA
 * addresses) for VALID entries and the slice command indexes their packed
 * header slots, so a reference to any other entry makes the VP dereference an
 * uninitialised reference slot: that ends in a DART translation fault, a
 * frame that never completes and a watchdog reset of the VP, which also
 * disturbs the other contexts. It happens whenever a client starts decoding
 * on a non-IDR frame (joining a live RTSP stream mid-GOP): ffmpeg's
 * v4l2request hwaccel leaves the index of a missing reference at 0 and the
 * DPB is empty. Refuse such slices before anything is pushed; the client gets
 * the capture buffer back with V4L2_BUF_FLAG_ERROR, as for corrupt input.
 */
static bool avd_h264_refs_valid(const struct avd_h264_run *run)
{
	const struct v4l2_ctrl_h264_slice_params *sl = run->slice_params;
	const struct v4l2_h264_dpb_entry *dpb = run->decode_params->dpb;
	const struct v4l2_h264_reference *lists[2] = {
		sl->ref_pic_list0, sl->ref_pic_list1
	};
	u32 num[2] = {
		sl->num_ref_idx_l0_active_minus1 + 1,
		sl->num_ref_idx_l1_active_minus1 + 1
	};
	int nlists;

	if (sl->slice_type == V4L2_H264_SLICE_TYPE_P)
		nlists = 1;
	else if (sl->slice_type == V4L2_H264_SLICE_TYPE_B)
		nlists = 2;
	else
		return true;

	for (int l = 0; l < nlists; l++) {
		if (num[l] > V4L2_H264_REF_LIST_LEN)
			return false;
		for (u32 i = 0; i < num[l]; i++) {
			u8 idx = lists[l][i].index;

			if (idx >= V4L2_H264_NUM_DPB_ENTRIES ||
			    !(dpb[idx].flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
				return false;
		}
	}

	return true;
}

/*
 * Every slice codes at least one macroblock, so a picture has no more slices
 * than macroblocks, counting both fields of a frame.
 */
static size_t avd_h264_max_slices(const struct v4l2_ctrl_h264_sps *sps)
{
	size_t mbs = (size_t)(sps->pic_width_in_mbs_minus1 + 1) *
		     (sps->pic_height_in_map_units_minus1 + 1);

	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
		mbs *= 2;

	return min_t(size_t, mbs, MAX_SLICES);
}

/*
 * Coded data held for one picture. A picture of PCM macroblocks is about the
 * size of the decoded one and emulation prevention adds at most one byte for
 * every two, so twice the decoded buffer is enough; MAX_SLICES pages leave
 * room for small pictures cut into many slices.
 */
static size_t avd_h264_slice_budget(size_t decoded_size)
{
	return max_t(size_t, 2 * decoded_size, MAX_SLICES * PAGE_SIZE);
}

/* Check that one more slice of len bytes fits in the current picture. */
VISIBLE_IF_KUNIT bool avd_h264_slice_fits(const struct v4l2_ctrl_h264_sps *sps,
					  size_t held, size_t held_bytes,
					  u32 len, size_t decoded_size)
{
	return held < avd_h264_max_slices(sps) &&
	       held_bytes + PAGE_ALIGN((size_t)len) <=
	       avd_h264_slice_budget(decoded_size);
}

/*
 * Start a new picture in a bank no unretired picture uses, and remember the
 * bank in its CAPTURE buffer so retirement frees the right one.
 */
static int avd_h264_pick_bank(struct avd_ctx *ctx, struct vb2_v4l2_buffer *dst)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct avd_bank *bank = NULL;
	unsigned int i;

	/* A picture whose last slice never came: start over in its bank. */
	if (h264_ctx->b->busy && !h264_ctx->b->submitted)
		bank = h264_ctx->b;
	for (i = 0; !bank && i < h264_ctx->nbanks; i++)
		if (!h264_ctx->bank[i].busy)
			bank = &h264_ctx->bank[i];
	if (!bank)
		return -EBUSY;
	avd_h264_drop_bank_slices(ctx, bank);
	bank->busy = true;
	bank->submitted = false;
	h264_ctx->b = bank;
	/* 0 means no bank, so a stale or missing value never frees one. */
	vb2_to_avd_decoded_buf(&dst->vb2_buf)->bank = bank - h264_ctx->bank + 1;
	return 0;
}

/* A rejected slice must never release the other picture's DMA-owned bank. */
static void avd_h264_abort_picture(struct avd_ctx *ctx,
				   struct vb2_v4l2_buffer *dst)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct avd_bank *bank = h264_ctx->b;
	struct avd_decoded_buffer *buf = vb2_to_avd_decoded_buf(&dst->vb2_buf);

	if (!bank->busy || bank->submitted ||
	    buf->bank != bank - h264_ctx->bank + 1)
		return;
	avd_h264_drop_bank_slices(ctx, bank);
	bank->busy = false;
	buf->bank = 0;
}

static int avd_h264_run(struct avd_ctx *ctx)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct v4l2_h264_reflist_builder reflist_builder;
	struct avd_h264_run run;
	size_t held = h264_ctx->b->busy && !h264_ctx->b->submitted ?
		      h264_ctx->b->slice_num : 0;
	int ret;

	avd_h264_run_preamble(ctx, &run);

	if (!run.sps || !run.pps || !run.slice_params || !run.decode_params) {
		ret = -EINVAL;
		goto err_free_jobs;
	}

	/* Recheck controls applied from this request before building instructions. */
	ret = avd_h264_validate_sps(ctx, run.sps);
	if (!ret)
		ret = avd_h264_validate_pps(ctx, run.pps);
	if (!ret)
		ret = avd_h264_validate_slice(ctx, run.slice_params);
	if (ret)
		goto err_free_jobs;

	/* Check before pick_bank() can discard an unfinished picture's slices. */
	if (!avd_h264_slice_order_supported(held, h264_ctx->b->last_mb,
					    run.slice_params->first_mb_in_slice)) {
		dev_err_ratelimited(ctx->dev->dev,
				    "H.264 arbitrary slice order (ASO) is unsupported\n");
		ret = -EINVAL;
		goto err_free_jobs;
	}

	if (is_new_frame(run.slice_params)) {
		ret = avd_h264_pick_bank(ctx, run.base.bufs.dst);
		if (ret)
			goto err_free_jobs;
	}

	if (!h264_ctx->b->busy || h264_ctx->b->submitted ||
	    vb2_to_avd_decoded_buf(&run.base.bufs.dst->vb2_buf)->bank !=
				   h264_ctx->b - h264_ctx->bank + 1) {
		ret = -EINVAL;
		goto err_free_jobs;
	}

	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	u32 payload_len = vb2_get_plane_payload(&src->vb2_buf, 0);
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);

	if (!data || !avd_h264_find_slice_data(data, payload_len,
					       avd_h264_header_len(&run),
					       &run.slice_data_off)) {
		dev_dbg_ratelimited(ctx->dev->dev,
				    "slice has no data after its header\n");
		ret = -EINVAL;
		goto err_free_jobs;
	}

	if (!avd_h264_slice_fits(run.sps, h264_ctx->b->slice_num,
				 h264_ctx->b->slice_bytes, payload_len,
				 ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].sizeimage)) {
		dev_err_ratelimited(ctx->dev->dev,
				    "too many slices for one picture, dropping it\n");
		ret = -EINVAL;
		goto err_free_jobs;
	}

	if (h264_ctx->b->slice_num >= h264_ctx->b->alloc_slice_num) {
		ret = avd_h264_realloc_slices(ctx, avd_h264_max_slices(run.sps));
		if (ret)
			goto err_free_jobs;
	}

	h264_ctx->b->active_slice = &h264_ctx->b->slices[h264_ctx->b->slice_num];
	ret = avd_buf_alloc(ctx->dev, h264_ctx->b->active_slice, payload_len);
	if (ret)
		goto err_free_jobs;
	memcpy(h264_ctx->b->active_slice->cpu, data, payload_len);
	h264_ctx->b->slice_num++;
	h264_ctx->b->last_mb = run.slice_params->first_mb_in_slice;
	h264_ctx->b->slice_bytes += PAGE_ALIGN((size_t)payload_len);
	h264_ctx->b->monochrome = run.sps->chroma_format_idc == 0;

	/* Build the P/B{0,1} ref lists. */
	v4l2_h264_init_reflist_builder(&reflist_builder, run.decode_params,
				       run.sps, run.decode_params->dpb);

	/* stream_refs() emits VALID entries, including inactive references. */
	run.num_valid = avd_h264_num_refs(run.decode_params);
	run.cur_poc = reflist_builder.cur_pic_order_count;

	v4l2_h264_build_p_ref_list(&reflist_builder, h264_ctx->reflists.p);
	v4l2_h264_build_b_ref_lists(&reflist_builder, h264_ctx->reflists.b0,
				    h264_ctx->reflists.b1);

	if (!avd_h264_refs_valid(&run)) {
		dev_dbg_ratelimited(ctx->dev->dev,
				    "slice references an invalid DPB entry, dropping frame\n");
		ret = -EINVAL;
		goto err_free_jobs;
	}

	if (is_new_frame(run.slice_params)) {
		/* the header segment plus one per slice */
		ret = avd_init_job(ctx, AVD_CODEC_H264,
				   h264_ctx->b->alloc_slice_num + 1);
		if (ret)
			goto err_free_jobs;
		stream_hdr(ctx, &run);
	}

	if (!ctx->job.segments) {
		ret = -EINVAL;
		goto err_free_jobs;
	}

	ctx->job.num++;
	stream_slice(ctx, &run);
	avd_run_postamble(ctx, &run.base);

	if (run.base.bufs.src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF) {
		avd_job_finish(ctx, VB2_BUF_STATE_DONE);
		return 0;
	}

	ctx->job_fifo = h264_ctx->b->bufs.inst.addr;
	h264_ctx->b->submitted = true;
	return avd_submit_job(ctx);

err_free_jobs:
	avd_h264_abort_picture(ctx, run.base.bufs.dst);
	kvfree(ctx->job.segments);
	ctx->job.segments = NULL;
	avd_run_postamble(ctx, &run.base);
	return ret;
}

static void avd_h264_done(struct avd_ctx *ctx, struct vb2_v4l2_buffer *src_buf,
			  struct vb2_v4l2_buffer *dst_buf,
			  enum vb2_buffer_state result)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct avd_decoded_buffer *buf;
	struct avd_bank *bank;

	if (src_buf->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF || !dst_buf)
		return;

	buf = vb2_to_avd_decoded_buf(&dst_buf->vb2_buf);
	if (!buf->bank || buf->bank > h264_ctx->nbanks)
		return;
	bank = &h264_ctx->bank[buf->bank - 1];
	buf->bank = 0;
	avd_h264_drop_bank_slices(ctx, bank);
	bank->busy = false;
	bank->submitted = false;

	/* 4:0:0 has no chroma to decode; the picture is grey */
	if (bank->monochrome && result == VB2_BUF_STATE_DONE) {
		const struct v4l2_pix_format_mplane *pix = &ctx->decoded_fmt.fmt.pix_mp;
		struct avd_decoded_buffer *dst =
			vb2_to_avd_decoded_buf(&dst_buf->vb2_buf);

		/* vb2 must return cache ownership before the CPU writes. */
		dst->grey_chroma_offset =
			(size_t)pix->plane_fmt[0].bytesperline * pix->height;
		dst->grey_chroma_10bit =
			ctx->image_fmt == AVD_IMG_FMT_420_10BIT;
	}
}

static enum avd_image_fmt avd_h264_get_image_fmt(struct avd_ctx *ctx,
						 struct v4l2_ctrl *ctrl)
{
	const struct v4l2_ctrl_h264_sps *sps = ctrl->p_new.p_h264_sps;

	if (ctrl->id != V4L2_CID_STATELESS_H264_SPS)
		return AVD_IMG_FMT_ANY;

	if (sps->bit_depth_luma_minus8 == 0) {
		if (sps->chroma_format_idc == 2)
			return AVD_IMG_FMT_422_8BIT;
		else
			return AVD_IMG_FMT_420_8BIT;
	} else if (sps->bit_depth_luma_minus8 == 2) {
		if (sps->chroma_format_idc == 2)
			return AVD_IMG_FMT_422_10BIT;
		else
			return AVD_IMG_FMT_420_10BIT;
	}

	return AVD_IMG_FMT_ANY;
}

static void avd_h264_adjust_decoded_fmt(struct avd_ctx *ctx,
					struct v4l2_pix_format_mplane *pix_mp)
{
	pix_mp->plane_fmt[0].sizeimage +=
		mv_color_size(pix_mp->width, pix_mp->height);
}

static int avd_h264_try_ctrl(struct avd_ctx *ctx, struct v4l2_ctrl *ctrl)
{
	if (ctrl->id == V4L2_CID_STATELESS_H264_SPS)
		return avd_h264_validate_sps(ctx, ctrl->p_new.p_h264_sps);
	if (ctrl->id == V4L2_CID_STATELESS_H264_PPS)
		return avd_h264_validate_pps(ctx, ctrl->p_new.p_h264_pps);
	if (ctrl->id == V4L2_CID_STATELESS_H264_SLICE_PARAMS)
		return avd_h264_validate_slice(ctx, ctrl->p_new.p_h264_slice_params);

	return 0;
}

static void avd_h264_submit(struct avd_ctx *ctx)
{
	writel_relaxed(
		AVD_OP_EXEC |
			AVD_OP_EXEC_FLAG_START_REV4(
				ctx->dev->variant->revision == 4) |
			AVD_OP_EXEC_FLAG_START_REV3(
				ctx->dev->variant->revision == 3) |
			AVD_OP_EXEC_FIFO_IDX(ctx->fifo_idx) |
			AVD_OP_EXEC_FIFO_MASK(ctx->dev->variant->fifo_slots),
		ctx->dev->ctrl + ctx->dev->variant->submit_offset);
}

const struct avd_coded_fmt_ops avd_h264_fmt_ops = {
	.adjust_decoded_fmt = avd_h264_adjust_decoded_fmt,
	.start = avd_h264_start,
	.stop = avd_h264_stop,
	.done = avd_h264_done,
	.run = avd_h264_run,
	.submit = avd_h264_submit,
	.try_ctrl = avd_h264_try_ctrl,
	.get_image_fmt = avd_h264_get_image_fmt,
};
