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

/* no picture, or a DPB entry without a VP reference slot */
#define AVD_H264_NO_REF	V4L2_H264_NUM_DPB_ENTRIES

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
	/* VP reference slot of each DPB entry, see avd_h264_assign_slots() */
	u8 slot[V4L2_H264_NUM_DPB_ENTRIES];
	u8 num_slots;
	/* coded slice data after the slice header */
	dma_addr_t slice_addr;
	u32 slice_len;
};

/* state */
struct avd_h264_ctx {
	bool monochrome;
	struct avd_h264_reflists {
		struct v4l2_h264_reference p[V4L2_H264_REF_LIST_LEN];
		struct v4l2_h264_reference b0[V4L2_H264_REF_LIST_LEN];
		struct v4l2_h264_reference b1[V4L2_H264_REF_LIST_LEN];
	} reflists;

	/* slice header and weights parsed in-kernel in frame-based mode */
	struct v4l2_ctrl_h264_slice_params fb_slice;
	struct v4l2_ctrl_h264_pred_weights fb_weights;

	struct avd_buf *slices;
	struct avd_buf *active_slice;
	size_t slice_num;
	size_t alloc_slice_num;
	/* allocated size of the held slices */
	size_t slice_bytes;

	struct avd_h264_bufs {
		struct avd_buf inst;
		struct avd_buf pipe_state;
		struct avd_buf above_info;
		struct avd_buf lf_above_info;
		struct avd_buf lf_above;
		struct avd_buf ip_above;
		struct avd_buf mv_above_info;
	} bufs;
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
	pusha(h264_ctx->bufs.mv_above_info.addr, "mv_above_info", 7);
	pusha(run->addresses.mv_color, "mv_color", 0);

	push(0, "");
	push(0, "");
	push(0, "");
	push(0, "");

	for (int i = 0; i < ARRAY_SIZE(decode->dpb); i++) {
		if (run->slot[i] == AVD_H264_NO_REF)
			continue;

		ref = avd_get_ref_buf(ctx, &dst->base.vb, dpb[i].reference_ts);

		addr = vb2_dma_contig_plane_dma_addr(&ref->base.vb.vb2_buf, 0) +
		       ref->comp.start_offset;

		push(AVD_REF_NUM(run->num_slots - 1) | AVD_REF_FLAG_CONST |
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

static void stream_hdr(struct avd_ctx *ctx, struct avd_h264_run *run)
{
	const struct v4l2_ctrl_h264_decode_params *decode = run->decode_params;
	const struct v4l2_ctrl_h264_sps *sps = run->sps;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	struct avd_dev *avd = ctx->dev;
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	u32 bytesperline;
	u32 width = (sps->pic_width_in_mbs_minus1 + 1) * 16;
	u32 height = (sps->pic_height_in_map_units_minus1 + 1) * 16;

	push(AVD_OP_HDR | AVD_OP_HDR_FLAG_DECOMP(ctx->decomp) |
		     AVD_OP_HDR_FLAG_INTRA(
			     decode->flags &
			     V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) |
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
		     H264_FLAG_NOT_IDR(!(decode->flags &
					 V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC)) |
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

	pusha(h264_ctx->bufs.above_info.addr, "hdr_9c_pps_tile_addr_lsb8", 0);

	push(0, "");
	push(0, "");

	if (avd->variant->revision == 3)
		push(0, "zero");
	else if (!(avd->variant->quirks & AVD_QUIRK_NO_PIPE_STATE))
		pusha(h264_ctx->bufs.pipe_state.addr, "pipe_state", 0);

	pusha(h264_ctx->bufs.ip_above.addr, "ip_above", 1);
	pusha(h264_ctx->bufs.lf_above.addr, "lf_above", 2);
	pusha(h264_ctx->bufs.lf_above_info.addr, "lf_above_info", 3);
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

	if (!(decode->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC))
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
	bool en_mode = (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) ==
		       0;
	dma_addr_t slc_a84 = run->slice_addr;

	push(AVD_OP_CODED_DATA |
		     AVD_OP_CODED_DATA_BIT_OFF(
			     en_mode ? (sl->header_bit_size % 8) : 0) |
		     AVD_OP_CODED_DATA_ADDR(slc_a84 >> 32),
	     "slc_a7c_cmd_set_coded_slice");
	push((u32)(slc_a84 & 0xffffffff), "slc_a84_slice_addr_low");

	push(run->slice_len, "slc_a88_slice_hdr_size");

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
					     run->slot[sl->ref_pic_list0[i].index]),
			     "slc_6e8_cmd_ref_list_0");

		if (sl->slice_type == V4L2_H264_SLICE_TYPE_B) {
			u32 num_ref_idx_active =
				sl->num_ref_idx_l1_active_minus1 + 1;
			for (u32 i = 0; i < num_ref_idx_active; i++)
				push(AVD_OP_REF | AVD_OP_REF_LIST_IDX(1) |
					     AVD_OP_REF_LOOP_IDX(i) |
					     AVD_OP_REF_DBP_IDX(
						     run->slot[sl->ref_pic_list1[i].index]),
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

static int avd_h264_alloc_bufs(struct avd_ctx *ctx)
{
	struct avd_dev *dev = ctx->dev;
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	int ret, w, bit_depth, mb;
	w = fmt_width(ctx);
	bit_depth = (ctx->image_fmt == AVD_IMG_FMT_420_10BIT ||
		     ctx->image_fmt == AVD_IMG_FMT_422_10BIT) ?
			    10 :
			    8;

	ret = avd_buf_alloc(dev, &h264_ctx->bufs.inst, fifo_size());
	if (ret) {
		return ret;
	}

	if (!(dev->variant->quirks & AVD_QUIRK_NO_PIPE_STATE)) {
		ret = avd_buf_alloc(dev, &h264_ctx->bufs.pipe_state, 0x200);
		if (ret) {
			return ret;
		}
	}

	mb = DIV_ROUND_UP(w, 16);

	ret = avd_buf_alloc(dev, &h264_ctx->bufs.above_info, mb * 20);
	if (ret)
		return ret;

	ret = avd_buf_alloc(dev, &h264_ctx->bufs.ip_above, bit_depth * 4 * mb);
	if (ret)
		return ret;

	ret = avd_buf_alloc(dev, &h264_ctx->bufs.lf_above,
			    bit_depth * 4 * 4 * mb);
	if (ret)
		return ret;

	ret = avd_buf_alloc(dev, &h264_ctx->bufs.lf_above_info, 32 * mb);
	if (ret)
		return ret;

	ret = avd_buf_alloc(dev, &h264_ctx->bufs.mv_above_info, 32 * mb);
	if (ret)
		return ret;

	return 0;
}

static int avd_h264_validate_pps(struct avd_ctx *ctx,
				 const struct v4l2_ctrl_h264_pps *pps)
{
	if (pps->num_slice_groups_minus1 != 0) {
		dev_err(ctx->dev->dev, "pps->num_slice_groups_minus1 != 0");
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
	if (sps->bit_depth_luma_minus8 != sps->bit_depth_chroma_minus8)
		/* Luma and chroma bit depth mismatch */
		return -EINVAL;
	if (sps->bit_depth_luma_minus8 != 0 && sps->bit_depth_luma_minus8 != 2)
		/* Only 8 and 10 bit have capture formats */
		return -EINVAL;
	if (sps->chroma_format_idc == 2 && sps->bit_depth_luma_minus8 == 2)
		/* 4:2:2 10 bit would need P210; P010 is too small for its chroma */
		return -EINVAL;

	return 0;
}

/* Free the slices held for the current picture. */
static void avd_h264_drop_slices(struct avd_ctx *ctx)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;

	for (size_t i = 0; i < h264_ctx->slice_num; i++)
		avd_buf_free(ctx->dev, &h264_ctx->slices[i]);
	h264_ctx->slice_num = 0;
	h264_ctx->slice_bytes = 0;
}

static void avd_h264_stop(struct avd_ctx *ctx)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct avd_dev *dev = ctx->dev;

	if (!h264_ctx)
		return;

	avd_buf_free(dev, &h264_ctx->bufs.pipe_state);
	avd_buf_free(dev, &h264_ctx->bufs.inst);
	avd_buf_free(dev, &h264_ctx->bufs.above_info);
	avd_buf_free(dev, &h264_ctx->bufs.lf_above_info);
	avd_buf_free(dev, &h264_ctx->bufs.lf_above);
	avd_buf_free(dev, &h264_ctx->bufs.ip_above);
	avd_buf_free(dev, &h264_ctx->bufs.mv_above_info);

	avd_h264_drop_slices(ctx);
	kfree(h264_ctx->slices);

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

	/* assume one slice per row */
	h264_ctx->alloc_slice_num =
		ctrl->p_new.p_h264_sps->pic_height_in_map_units_minus1 + 1;
	h264_ctx->slices = kcalloc(h264_ctx->alloc_slice_num,
				   sizeof(*h264_ctx->slices), GFP_KERNEL);
	if (!h264_ctx->slices) {
		ret = -ENOMEM;
		goto err_free_ctx;
	}

	ret = avd_h264_alloc_bufs(ctx);
	if (ret)
		goto err_free_ctx;

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

	tmp = h264_ctx->slices;
	/* grow by half, and by at least one for a one-row picture */
	alloc_slice_num = h264_ctx->alloc_slice_num +
			  max_t(size_t, h264_ctx->alloc_slice_num / 2, 1);
	alloc_slice_num = min(alloc_slice_num, max_slices);
	h264_ctx->slices = kcalloc(alloc_slice_num, sizeof(*h264_ctx->slices),
				   GFP_KERNEL);
	if (!h264_ctx->slices) {
		/* make deallocating a little easier */
		h264_ctx->slices = tmp;
		return -ENOMEM;
	}

	h264_ctx->alloc_slice_num = alloc_slice_num;
	memcpy(h264_ctx->slices, tmp,
	       sizeof(*h264_ctx->slices) * h264_ctx->slice_num);
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
 * The VP holds the reference pictures in slots, numbered in the order that
 * stream_refs() pushes them, and the slice's reference lists address those
 * slots. Give a slot to every DPB entry that is a reference picture (VALID
 * and ACTIVE), and to nothing else: Chromium also lists the pictures that
 * only wait for output, and pushing those as well left the reference count
 * in the header short of the references pushed, which stalled the VP on
 * some P pictures.
 */
static void avd_h264_assign_slots(struct avd_h264_run *run)
{
	const struct v4l2_h264_dpb_entry *dpb = run->decode_params->dpb;
	const u32 ref = V4L2_H264_DPB_ENTRY_FLAG_VALID |
			V4L2_H264_DPB_ENTRY_FLAG_ACTIVE;

	run->num_slots = 0;
	for (int i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++)
		run->slot[i] = (dpb[i].flags & ref) == ref ? run->num_slots++ :
							      AVD_H264_NO_REF;
}

/*
 * Check that every reference in the slice's ref lists points at a DPB entry
 * with a VP slot. stream_refs() only pushes a reference header (and its RVRA
 * addresses) for those entries, so a reference to any other entry makes the
 * VP dereference an
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
			    run->slot[idx] == AVD_H264_NO_REF)
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
 * Frame-based decoding: the client queues every slice of a picture in one
 * OUTPUT buffer and sets no slice parameters, so find the slice NAL units in
 * the buffer and parse their headers here (ITU-T H.264 7.3.3).
 */

/* Reads RBSP bits from a NAL unit, skipping emulation prevention bytes. */
struct avd_h264_bits {
	const u8 *data;
	u32 len;
	u32 pos;
	u32 zeros;
	u32 consumed;
	u8 cur;
	u8 left;
	bool err;
};

static u32 avd_h264_bit(struct avd_h264_bits *br)
{
	if (!br->left) {
		if (br->pos < br->len && br->zeros >= 2 &&
		    br->data[br->pos] == 0x03) {
			br->pos++;
			br->zeros = 0;
		}
		if (br->pos >= br->len) {
			br->err = true;
			return 0;
		}
		br->cur = br->data[br->pos++];
		br->zeros = br->cur ? 0 : br->zeros + 1;
		br->left = 8;
	}

	br->left--;
	br->consumed++;
	return (br->cur >> br->left) & 1;
}

static u32 avd_h264_bits(struct avd_h264_bits *br, unsigned int n)
{
	u32 v = 0;

	while (n--)
		v = (v << 1) | avd_h264_bit(br);
	return v;
}

static u32 avd_h264_ue(struct avd_h264_bits *br)
{
	unsigned int lz = 0;

	while (!avd_h264_bit(br)) {
		if (br->err || ++lz > 31) {
			br->err = true;
			return 0;
		}
	}

	return lz ? (u32)((1ULL << lz) - 1) + avd_h264_bits(br, lz) : 0;
}

static s32 avd_h264_se(struct avd_h264_bits *br)
{
	u32 k = avd_h264_ue(br);

	return (k & 1) ? (s32)((k >> 1) + 1) : -(s32)(k >> 1);
}

/*
 * Find the next NAL unit at or after *pos. With Annex B start codes the NAL
 * unit runs up to the next start code, without the trailing zero bytes that
 * belong to a four-byte start code: a NAL unit never ends in a zero byte, and
 * the AVD never completes a CAVLC picture whose slice data has such a tail.
 */
static bool avd_h264_next_nal(const u8 *data, u32 len, u32 *pos, u32 *nal_off,
			      u32 *nal_len)
{
	u32 i = *pos, start, end;

	for (; i + 3 <= len; i++)
		if (!data[i] && !data[i + 1] && data[i + 2] == 1)
			break;
	if (i + 3 > len)
		return false;

	start = i + 3;
	for (i = start; i + 3 <= len; i++)
		if (!data[i] && !data[i + 1] && data[i + 2] == 1)
			break;
	end = i + 3 <= len ? i : len;
	*pos = end;

	while (end > start && data[end - 1] == 0)
		end--;
	*nal_off = start;
	*nal_len = end - start;
	return true;
}

static bool avd_h264_is_slice_nal(const u8 *data, u32 len)
{
	u8 type = len ? data[0] & 0x1f : 0;

	return type == 1 || type == 5;
}

/* PicNum (8-28) of a short-term reference, or LongTermPicNum (8-29). */
static s32 avd_h264_pic_num(const struct avd_h264_run *run, unsigned int i)
{
	const struct v4l2_h264_dpb_entry *e = &run->decode_params->dpb[i];
	u32 max_frame_num = 1 << (run->sps->log2_max_frame_num_minus4 + 4);

	if (e->flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM)
		return e->pic_num;
	if (e->frame_num > run->decode_params->frame_num)
		return (s32)e->frame_num - max_frame_num;
	return e->frame_num;
}

static u8 avd_h264_find_ref(const struct avd_h264_run *run, s32 pic_num,
			    bool long_term)
{
	const struct v4l2_h264_dpb_entry *dpb = run->decode_params->dpb;

	for (unsigned int i = 0; i < V4L2_H264_NUM_DPB_ENTRIES; i++) {
		if (!(dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE) ||
		    !(dpb[i].flags & V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM) !=
			    !long_term)
			continue;
		if (avd_h264_pic_num(run, i) == pic_num)
			return i;
	}

	return AVD_H264_NO_REF;
}

/*
 * Build one reference picture list of a frame: the initial list (8.2.4.2)
 * cut to the active size, then ref_pic_list_modification() applied as in
 * 8.2.4.3.
 */
static void avd_h264_ref_list(struct avd_h264_bits *br,
			      const struct avd_h264_run *run,
			      const struct v4l2_h264_reference *init,
			      unsigned int num_init,
			      struct v4l2_h264_reference *list,
			      unsigned int num_active)
{
	struct v4l2_h264_reference tmp[V4L2_H264_REF_LIST_LEN + 1];
	u32 max_pic_num = 1 << (run->sps->log2_max_frame_num_minus4 + 4);
	s32 cur_pic_num = run->decode_params->frame_num;
	s32 pred = cur_pic_num;
	unsigned int ref_idx = 0, ops = 0;

	for (unsigned int i = 0; i <= num_active; i++) {
		if (i < num_init && i < num_active) {
			tmp[i] = init[i];
		} else {
			tmp[i].index = AVD_H264_NO_REF;
			tmp[i].fields = V4L2_H264_FRAME_REF;
		}
	}

	if (avd_h264_bit(br)) {
		for (;;) {
			u32 idc = avd_h264_ue(br);
			struct v4l2_h264_reference ref = {
				.fields = V4L2_H264_FRAME_REF,
			};
			unsigned int n;

			if (idc == 3 || br->err)
				break;
			if (idc > 3 || ++ops > num_active + 1) {
				br->err = true;
				break;
			}

			if (idc < 2) {
				u32 abs_diff = avd_h264_ue(br) + 1;
				s32 pic_num;

				if (abs_diff > max_pic_num) {
					br->err = true;
					break;
				}
				if (idc == 0) {
					pred -= abs_diff;
					if (pred < 0)
						pred += max_pic_num;
				} else {
					pred += abs_diff;
					if (pred >= max_pic_num)
						pred -= max_pic_num;
				}
				pic_num = pred > cur_pic_num ? pred - max_pic_num : pred;
				ref.index = avd_h264_find_ref(run, pic_num, false);
			} else {
				ref.index = avd_h264_find_ref(run, avd_h264_ue(br),
							      true);
			}

			/* insert at ref_idx, then drop the later duplicate */
			for (unsigned int c = num_active; c > ref_idx; c--)
				tmp[c] = tmp[c - 1];
			tmp[ref_idx++] = ref;
			n = ref_idx;
			for (unsigned int c = ref_idx; c <= num_active; c++)
				if (ref.index == AVD_H264_NO_REF ||
				    tmp[c].index != ref.index)
					tmp[n++] = tmp[c];
		}
	}

	/*
	 * A conforming slice uses no entry without a picture; point any such
	 * entry at the first reference so that the VP is never handed an
	 * unset DPB slot.
	 */
	for (unsigned int i = 0; i < num_active; i++) {
		list[i] = tmp[i];
		if (list[i].index == AVD_H264_NO_REF)
			list[i].index = tmp[0].index;
	}
}

/* pred_weight_table(), 7.3.3.2, with the inferred default factors filled in */
static void avd_h264_pred_weights(struct avd_h264_bits *br,
				  const struct avd_h264_run *run,
				  const struct v4l2_ctrl_h264_slice_params *sl,
				  struct v4l2_ctrl_h264_pred_weights *pw)
{
	const struct v4l2_ctrl_h264_sps *sps = run->sps;
	bool chroma = sps->chroma_format_idc &&
		      !(sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE);
	int lists = sl->slice_type == V4L2_H264_SLICE_TYPE_B ? 2 : 1;

	pw->luma_log2_weight_denom = avd_h264_ue(br);
	if (chroma)
		pw->chroma_log2_weight_denom = avd_h264_ue(br);
	if (pw->luma_log2_weight_denom > 7 || pw->chroma_log2_weight_denom > 7) {
		br->err = true;
		return;
	}

	for (int l = 0; l < lists; l++) {
		struct v4l2_h264_weight_factors *f = &pw->weight_factors[l];
		unsigned int n = (l ? sl->num_ref_idx_l1_active_minus1 :
				      sl->num_ref_idx_l0_active_minus1) + 1;

		for (unsigned int i = 0; i < n; i++) {
			f->luma_weight[i] = 1 << pw->luma_log2_weight_denom;
			f->luma_offset[i] = 0;
			if (avd_h264_bit(br)) {
				f->luma_weight[i] = avd_h264_se(br);
				f->luma_offset[i] = avd_h264_se(br);
			}

			for (int j = 0; j < 2; j++) {
				f->chroma_weight[i][j] =
					1 << pw->chroma_log2_weight_denom;
				f->chroma_offset[i][j] = 0;
			}
			if (chroma && avd_h264_bit(br)) {
				for (int j = 0; j < 2; j++) {
					f->chroma_weight[i][j] = avd_h264_se(br);
					f->chroma_offset[i][j] = avd_h264_se(br);
				}
			}
		}
	}
}

/* dec_ref_pic_marking(), 7.3.3.3: nothing in it is needed, only its size */
static void avd_h264_skip_ref_pic_marking(struct avd_h264_bits *br, bool idr)
{
	unsigned int ops = 0;

	if (idr) {
		avd_h264_bits(br, 2);
		return;
	}

	if (!avd_h264_bit(br))
		return;

	for (;;) {
		u32 op = avd_h264_ue(br);

		if (!op || br->err)
			return;
		if (op > 6 || ++ops > 2 * V4L2_H264_NUM_DPB_ENTRIES + 2) {
			br->err = true;
			return;
		}
		if (op == 1 || op == 3)
			avd_h264_ue(br);	/* difference_of_pic_nums_minus1 */
		if (op == 2)
			avd_h264_ue(br);	/* long_term_pic_num */
		if (op == 3 || op == 6)
			avd_h264_ue(br);	/* long_term_frame_idx */
		if (op == 4)
			avd_h264_ue(br);	/* max_long_term_frame_idx_plus1 */
	}
}

/*
 * Parse the header of one slice NAL unit into the slice parameters and
 * prediction weights the client would set in slice-based mode. Returns 1 for
 * a redundant slice, which the decoder may drop.
 */
static int avd_h264_parse_slice(const struct avd_h264_run *run,
				const struct avd_h264_reflists *init,
				const u8 *nal, u32 len,
				struct v4l2_ctrl_h264_slice_params *sl,
				struct v4l2_ctrl_h264_pred_weights *pw)
{
	const struct v4l2_ctrl_h264_sps *sps = run->sps;
	const struct v4l2_ctrl_h264_pps *pps = run->pps;
	u32 mbs = (sps->pic_width_in_mbs_minus1 + 1) *
		  (sps->pic_height_in_map_units_minus1 + 1);
	struct avd_h264_bits br = { .data = nal, .len = len };
	u32 nal_ref_idc, slice_type, num_l0, num_l1, v;
	bool idr, field_pic = false;
	s32 qp_delta, alpha = 0, beta = 0;

	memset(sl, 0, sizeof(*sl));
	memset(pw, 0, sizeof(*pw));

	avd_h264_bit(&br);			/* forbidden_zero_bit */
	nal_ref_idc = avd_h264_bits(&br, 2);
	idr = avd_h264_bits(&br, 5) == 5;

	sl->first_mb_in_slice = avd_h264_ue(&br);
	slice_type = avd_h264_ue(&br);
	if (sl->first_mb_in_slice >= mbs || slice_type > 9)
		return -EINVAL;
	sl->slice_type = slice_type % 5;
	if (sl->slice_type == V4L2_H264_SLICE_TYPE_SP ||
	    sl->slice_type == V4L2_H264_SLICE_TYPE_SI)
		return -EOPNOTSUPP;
	if (idr && sl->slice_type != V4L2_H264_SLICE_TYPE_I)
		return -EINVAL;

	avd_h264_ue(&br);			/* pic_parameter_set_id */
	if (sps->flags & V4L2_H264_SPS_FLAG_SEPARATE_COLOUR_PLANE)
		sl->colour_plane_id = avd_h264_bits(&br, 2);
	avd_h264_bits(&br, sps->log2_max_frame_num_minus4 + 4);
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)) {
		field_pic = avd_h264_bit(&br);
		if (field_pic)
			return -EOPNOTSUPP;
	}
	if (idr)
		avd_h264_ue(&br);		/* idr_pic_id */

	if (sps->pic_order_cnt_type == 0) {
		avd_h264_bits(&br, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if ((pps->flags &
		     V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) &&
		    !field_pic)
			avd_h264_se(&br);	/* delta_pic_order_cnt_bottom */
	} else if (sps->pic_order_cnt_type == 1 &&
		   !(sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)) {
		avd_h264_se(&br);		/* delta_pic_order_cnt[0] */
		if ((pps->flags &
		     V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT) &&
		    !field_pic)
			avd_h264_se(&br);	/* delta_pic_order_cnt[1] */
	}

	if (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT) {
		v = avd_h264_ue(&br);
		if (v > 127)
			return -EINVAL;
		sl->redundant_pic_cnt = v;
	}

	if (sl->slice_type == V4L2_H264_SLICE_TYPE_B && avd_h264_bit(&br))
		sl->flags |= V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED;

	if (sl->slice_type != V4L2_H264_SLICE_TYPE_I) {
		num_l0 = pps->num_ref_idx_l0_default_active_minus1;
		num_l1 = pps->num_ref_idx_l1_default_active_minus1;
		if (avd_h264_bit(&br)) {	/* num_ref_idx_active_override */
			num_l0 = avd_h264_ue(&br);
			if (sl->slice_type == V4L2_H264_SLICE_TYPE_B)
				num_l1 = avd_h264_ue(&br);
		}
		if (sl->slice_type != V4L2_H264_SLICE_TYPE_B)
			num_l1 = 0;
		/* the VP takes 16 entries a list, all a frame may use */
		if (num_l0 > 15 || num_l1 > 15)
			return -EINVAL;
		sl->num_ref_idx_l0_active_minus1 = num_l0;
		sl->num_ref_idx_l1_active_minus1 = num_l1;

		if (sl->slice_type == V4L2_H264_SLICE_TYPE_P) {
			avd_h264_ref_list(&br, run, init->p, run->num_valid,
					  sl->ref_pic_list0, num_l0 + 1);
		} else {
			avd_h264_ref_list(&br, run, init->b0, run->num_valid,
					  sl->ref_pic_list0, num_l0 + 1);
			avd_h264_ref_list(&br, run, init->b1, run->num_valid,
					  sl->ref_pic_list1, num_l1 + 1);
		}

		if (V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(pps, sl))
			avd_h264_pred_weights(&br, run, sl, pw);
	}

	if (nal_ref_idc)
		avd_h264_skip_ref_pic_marking(&br, idr);

	if ((pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) &&
	    sl->slice_type != V4L2_H264_SLICE_TYPE_I) {
		v = avd_h264_ue(&br);
		if (v > 2)
			return -EINVAL;
		sl->cabac_init_idc = v;
	}

	qp_delta = avd_h264_se(&br);
	if (qp_delta < -87 || qp_delta > 77)
		return -EINVAL;
	sl->slice_qp_delta = qp_delta;

	if (pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
		v = avd_h264_ue(&br);
		if (v > 2)
			return -EINVAL;
		sl->disable_deblocking_filter_idc = v;
		if (v != 1) {
			alpha = avd_h264_se(&br);
			beta = avd_h264_se(&br);
			if (alpha < -6 || alpha > 6 || beta < -6 || beta > 6)
				return -EINVAL;
		}
	}
	sl->slice_alpha_c0_offset_div2 = alpha;
	sl->slice_beta_offset_div2 = beta;

	if (br.err)
		return -EINVAL;

	sl->header_bit_size = br.consumed;
	return sl->redundant_pic_cnt ? 1 : 0;
}

static int avd_h264_run_frame(struct avd_ctx *ctx, struct avd_h264_run *run,
			      const u8 *data, u32 len)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct v4l2_ctrl_h264_slice_params *sl = &h264_ctx->fb_slice;
	u32 pos = 0, nal_off, nal_len, off;
	size_t num = 0;
	bool first = true, starts_picture;
	int ret;

	while (avd_h264_next_nal(data, len, &pos, &nal_off, &nal_len))
		if (avd_h264_is_slice_nal(data + nal_off, nal_len))
			num++;

	if (!num || num > avd_h264_max_slices(run->sps)) {
		dev_dbg_ratelimited(ctx->dev->dev,
				    "frame has %zu slices, dropping it\n", num);
		return -EINVAL;
	}

	/* the header segment plus one per slice */
	ret = avd_init_job(ctx, AVD_CODEC_H264, num + 1);
	if (ret)
		return ret;

	run->slice_params = sl;
	run->pred_weights = &h264_ctx->fb_weights;

	pos = 0;
	while (avd_h264_next_nal(data, len, &pos, &nal_off, &nal_len)) {
		if (!avd_h264_is_slice_nal(data + nal_off, nal_len))
			continue;

		ret = avd_h264_parse_slice(run, &h264_ctx->reflists,
					   data + nal_off, nal_len, sl,
					   &h264_ctx->fb_weights);
		if (ret == 1)
			continue;
		if (ret)
			goto err_drop;

		/*
		 * The picture starts at its first macroblock: the dimensions
		 * go with that slice, and only with it.
		 */
		starts_picture = !sl->first_mb_in_slice;
		if (first != starts_picture) {
			ret = -EINVAL;
			goto err_drop;
		}

		if (!avd_h264_refs_valid(run) ||
		    !avd_h264_find_slice_data(data + nal_off, nal_len,
					      avd_h264_header_len(run), &off)) {
			ret = -EINVAL;
			goto err_drop;
		}

		/* The OUTPUT buffer stays queued until the picture completes. */
		run->slice_addr = run->base.coded_in + nal_off + off;
		run->slice_len = nal_len - off;

		if (first)
			stream_hdr(ctx, run);
		first = false;

		ctx->job.num++;
		stream_slice(ctx, run);
	}

	if (first) {
		ret = -EINVAL;
		goto err_drop;
	}

	return avd_submit_job(ctx);

err_drop:
	dev_dbg_ratelimited(ctx->dev->dev, "bad slice in frame: %d\n", ret);
	kvfree(ctx->job.segments);
	ctx->job.segments = NULL;
	return ret;
}

static int avd_h264_run(struct avd_ctx *ctx)
{
	struct avd_h264_ctx *h264_ctx = ctx->priv;
	struct v4l2_h264_reflist_builder reflist_builder;
	struct avd_h264_run run;
	struct v4l2_ctrl *ctrl;
	bool frame_based, annexb;
	u32 nal_off, nal_len, pos = 0;
	int ret;

	avd_h264_run_preamble(ctx, &run);

	/*
	 * No interlaced support. This is checked per picture rather than when
	 * the SPS is set: Chromium sets a placeholder SPS without
	 * frame_mbs_only_flag to learn the decoded formats before it has
	 * parsed the stream, and the stream's own SPS comes later.
	 */
	if (!(run.sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)) {
		dev_dbg_ratelimited(ctx->dev->dev,
				    "interlaced H.264 is not supported\n");
		avd_run_postamble(ctx, &run.base);
		return -EINVAL;
	}

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_H264_DECODE_MODE);
	frame_based = ctrl && ctrl->cur.val ==
			      V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED;
	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_H264_START_CODE);
	annexb = ctrl && ctrl->cur.val == V4L2_STATELESS_H264_START_CODE_ANNEX_B;

	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	u32 payload_len = vb2_get_plane_payload(&src->vb2_buf, 0);
	const u8 *data = vb2_plane_vaddr(&src->vb2_buf, 0);

	/*
	 * A client may send Annex B start codes and leave the start code
	 * control at its default (Chromium does). A NAL unit never begins
	 * with a zero byte, so a leading start code is unambiguous.
	 */
	if (data && payload_len >= 3 && !data[0] && !data[1] &&
	    (data[2] == 1 || (payload_len >= 4 && !data[2] && data[3] == 1)))
		annexb = true;

	if (frame_based) {
		avd_run_postamble(ctx, &run.base);
		if (!data)
			return -EINVAL;
		/*
		 * Without start codes nothing marks where one slice ends and
		 * the next begins, so a multi-slice picture cannot be split.
		 */
		if (!annexb) {
			dev_dbg_ratelimited(ctx->dev->dev,
					    "frame-based decoding needs Annex B start codes\n");
			return -EINVAL;
		}

		h264_ctx->monochrome = run.sps->chroma_format_idc == 0;

		/* Build the P/B{0,1} ref lists. */
		v4l2_h264_init_reflist_builder(&reflist_builder,
					       run.decode_params, run.sps,
					       run.decode_params->dpb);
		run.num_valid = reflist_builder.num_valid;
		run.cur_poc = reflist_builder.cur_pic_order_count;
		avd_h264_assign_slots(&run);
		v4l2_h264_build_p_ref_list(&reflist_builder,
					   h264_ctx->reflists.p);
		v4l2_h264_build_b_ref_lists(&reflist_builder,
					    h264_ctx->reflists.b0,
					    h264_ctx->reflists.b1);

		return avd_h264_run_frame(ctx, &run, data, payload_len);
	}

	/* A slice with a start code: skip to its NAL unit. */
	if (data && annexb) {
		if (!avd_h264_next_nal(data, payload_len, &pos, &nal_off, &nal_len)) {
			avd_run_postamble(ctx, &run.base);
			return -EINVAL;
		}
		data += nal_off;
		payload_len = nal_len;
	}

	if (!run.slice_params ||
	    !data || !avd_h264_find_slice_data(data, payload_len,
					       avd_h264_header_len(&run),
					       &nal_off)) {
		dev_dbg_ratelimited(ctx->dev->dev,
				    "slice has no data after its header\n");
		avd_run_postamble(ctx, &run.base);
		return -EINVAL;
	}

	if (!avd_h264_slice_fits(run.sps, h264_ctx->slice_num,
				 h264_ctx->slice_bytes, payload_len,
				 ctx->decoded_fmt.fmt.pix_mp.plane_fmt[0].sizeimage)) {
		dev_err_ratelimited(ctx->dev->dev,
				    "too many slices for one picture, dropping it\n");
		/* The unsubmitted job table points at the dropped slices. */
		avd_h264_drop_slices(ctx);
		ret = -EINVAL;
		goto err_free_jobs;
	}

	if (h264_ctx->slice_num >= h264_ctx->alloc_slice_num) {
		ret = avd_h264_realloc_slices(ctx, avd_h264_max_slices(run.sps));
		if (ret)
			goto err_free_jobs;
	}

	h264_ctx->active_slice = &h264_ctx->slices[h264_ctx->slice_num];
	ret = avd_buf_alloc(ctx->dev, h264_ctx->active_slice, payload_len);
	if (ret)
		goto err_free_jobs;
	memcpy(h264_ctx->active_slice->cpu, data, payload_len);
	h264_ctx->slice_num++;
	h264_ctx->slice_bytes += PAGE_ALIGN((size_t)payload_len);
	h264_ctx->monochrome = run.sps->chroma_format_idc == 0;
	run.slice_addr = h264_ctx->active_slice->addr + nal_off;
	run.slice_len = payload_len - nal_off;

	/* Build the P/B{0,1} ref lists. */
	v4l2_h264_init_reflist_builder(&reflist_builder, run.decode_params,
				       run.sps, run.decode_params->dpb);

	run.num_valid = reflist_builder.num_valid;
	run.cur_poc = reflist_builder.cur_pic_order_count;
	avd_h264_assign_slots(&run);

	v4l2_h264_build_p_ref_list(&reflist_builder, h264_ctx->reflists.p);
	v4l2_h264_build_b_ref_lists(&reflist_builder, h264_ctx->reflists.b0,
				    h264_ctx->reflists.b1);

	avd_run_postamble(ctx, &run.base);

	if (!avd_h264_refs_valid(&run)) {
		dev_dbg_ratelimited(ctx->dev->dev,
				    "slice references an invalid DPB entry, dropping frame\n");
		return -EINVAL;
	}

	if (is_new_frame(run.slice_params)) {
		/* the header segment plus one per slice */
		ret = avd_init_job(ctx, AVD_CODEC_H264,
				   h264_ctx->alloc_slice_num + 1);
		if (ret)
			return ret;
		stream_hdr(ctx, &run);
	}

	if (!ctx->job.segments)
		return -EINVAL;

	ctx->job.num++;
	stream_slice(ctx, &run);

	if (run.base.bufs.src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF) {
		avd_job_finish(ctx, VB2_BUF_STATE_DONE);
		return 0;
	}

	return avd_submit_job(ctx);

err_free_jobs:
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

	if (!(src_buf->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF)) {
		avd_h264_drop_slices(ctx);

		/* 4:0:0 has no chroma to decode; the picture is grey */
		if (h264_ctx->monochrome && dst_buf && result == VB2_BUF_STATE_DONE) {
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
