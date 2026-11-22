/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _APPLE_AVE_LAYOUT_H_
#define _APPLE_AVE_LAYOUT_H_

struct ave_ctx;
struct ave_dev;
struct ave_fw_layout;
struct ave_fw_abi;

struct ave_command {
	enum ave_opcode op;
	enum ave_codec codec;
	u64 token;
	/* Zero unit and arg1 mean the encoder's values. */
	u32 client, subtype, unit, arg0, arg1, arg2;
};

/* Status is normalized to zero for success by the ABI parser. */
struct ave_reply {
	enum ave_opcode op;
	u64 client;
	u32 subtype, status, raw_status, bytes;
	u32 completion_index; /* Process replies only */
};

enum ave_layout_kind {
	AVE_LAYOUT_9003,
	AVE_LAYOUT_26A428,
	AVE_LAYOUT_6070,
};

struct ave_fw_transport {
	u32 ipc_bytes;
	u32 command_offset, command_bytes, scalar_offset;
	u32 process_offset, process_bytes;
	/* Zero channel/heap sizes mean bounded negotiation on 9003. */
	u32 hello[4];
	u32 descriptors;
	u32 io_index, t2h_index;
	u32 slots[2], doorbells[2], offsets[2]; /* zero: discovered */
	/*
	 * Part of the IPC window the firmware may borrow over its
	 * SHAREDMALLOC channel; zero when the firmware has no such channel.
	 */
	u32 smalloc_offset, smalloc_bytes;
	bool boot_config;
	bool sync_host_clock;
	bool scratch_halt;
};

struct ave_fw_abi {
	const struct ave_fw_transport *transport;
	u32 wire_id[2][AVE_OP_COUNT];
	u32 reply_id[AVE_OP_COUNT];
	u32 scalar_bytes[AVE_OP_COUNT];
	u32 semantic[AVE_OP_COUNT];
	u32 header_bytes, timeout_offset, created_offset;
	bool padded_surfaces;
	void (*write_header)(u8 *cmd, const struct ave_fw_abi *abi,
			     const struct ave_command *header);
	void (*write_config)(struct ave_dev *ave, u8 *cmd);
	int (*parse_reply)(const struct ave_fw_abi *abi, const u8 *msg, u32 len,
			   struct ave_reply *reply);
};

/* Separate address/size offsets and strides also represent split arrays. */
struct ave_surface_layout {
	u32 address, size;
	u32 address_stride, size_stride;
};

#define AVE_OFFSET_ABSENT U32_MAX

#define AVE_SURFACE(off, stride)      \
	{ .address = (off),           \
	  .size = (off) + 8,          \
	  .address_stride = (stride), \
	  .size_stride = (stride) }

#define AVE_ADDRESS_BANK(off, stride)          \
	{ .address = (off),                     \
	  .size = AVE_OFFSET_ABSENT,             \
	  .address_stride = (stride),            \
	  .size_stride = 0 }

/* A fixed-capacity bank of addresses, optionally repeated by the ABI. */
struct ave_address_bank_layout {
	u32 address;
	u16 entries, banks;
	u16 entry_stride, bank_stride;
	u8 values; /* consecutive u64 values in each entry */
};

/* Four 6070 entropy sets are interleaved by set inside each entry. */
struct ave_entropy_layout {
	u32 address, size;
	u16 entries, sets;
	u16 address_entry_stride, address_set_stride;
	u16 size_entry_stride, size_set_stride;
};

struct ave_rational_layout {
	u32 numerator, denominator;
};

/* A session-owned codec-state span copied into each Process command. */
struct ave_retained_partition {
	u32 command_offset, state_offset, bytes;
};

struct ave_recon_selection_layout {
	u32 luma_data;
	u32 luma_metadata;
	u32 chroma_data;
	u32 chroma_metadata;
};

#define AVE_MAX_RETAINED_PARTITIONS 3

/* Zero offsets denote fields absent from a firmware's wire format. */
struct ave_start_layout {
	struct ave_surface_layout header, secondary, param_sets;
	struct ave_surface_layout ref_a, ref_b, ref_c, ref_c_alias, ref_d;
	struct ave_surface_layout ref_e, ref_f;
	struct ave_surface_layout hevc_transcoded;
	struct ave_surface_layout mb, coded, meta, row_a, row_b, fixed,
		neighbour_fw, hevc_extra;
	struct ave_surface_layout tiles[8];
	struct ave_address_bank_layout recon, colocated, lfs, lfs_result;
	struct ave_entropy_layout entropy;
	u32 refs_count, work_count, lfs_count, slots_count, meta_count;
	u32 mb_count, row_a_count, row_b_count, fixed_count, tiles_count;
	u32 codec;
	u32 flag_8bit;
	u32 const_214;
	u32 const_218;
	struct ave_rational_layout fps;
	u32 rc_mode;
	/* Optional ABI-specific wire value for the driver's fixed-QP mode. */
	u32 rc_mode_fixed_qp;
	u32 bitrate;
	u32 init_qp;
	u8 init_qp_words; /* zero preserves the older three-word representation */
	u32 limits;
	u32 limit_bytes;
	u32 limit_secs;
	u32 f64_288;
	u32 f64_290;
	u32 min_qp;
	u32 max_qp;
	u32 gop_flags;
	u32 p_frames;
	u32 max_b;
	u32 ref_depth;
	u32 idr_interval;
	u32 f64_2c0;
	u32 f64_2c8;
	u32 rc_flags;
	u32 min_qp2;
	u32 max_qp2;
	u32 rc_310;
	u32 size_class;
	u32 coded_w;
	u32 coded_h;
	u32 codec_flags;
	u32 h264_380;
	u32 h264_384;
	u32 codec_2588;
	u32 bit_depth;
	u32 ref_compression;
	u32 coded_h2;
	u32 h264_profile;
	u32 h264_level;
	u32 h264_max_refs;
	u32 h264_sps_id;
	u32 h264_chroma;
	u32 h264_depth_luma;
	u32 h264_depth_chroma;
	u32 h264_frame_num_bits;
	u32 h264_poc_type;
	u32 h264_poc_bits;
	u32 h264_gaps;
	u32 h264_width_mbs;
	u32 h264_height_mbs;
	u32 h264_frame_only;
	u32 h264_direct;
	u32 h264_vui;
	u32 h264_crop;
	u32 h264_crop_left;
	u32 h264_crop_right;
	u32 h264_crop_top;
	u32 h264_crop_bottom;
	u32 h264_fw_header;
	u32 h264_header_len;
	u32 h264_pps_id;
	u32 h264_pps_sps_id;
	u32 h264_cabac;
	u32 h264_slice_groups;
	u32 h264_pic_init_qp;
	u32 h264_deblock;
	u32 h264_constrained;
	u32 h264_redundant;
	u32 h264_8x8;
	u32 h264_pps_fw_header;
	/* 6070 AVC first-picture setup. */
	u32 h264_max_mvs_per_2mb;
	u32 h264_adapt_b;
	u32 h264_pixel_packing;
	u32 h264_transform_mode;
	u32 h264_skip_mode;
	u32 h264_input_bit_depth;
	u32 h264_parameter_set_count;
	u32 h264_slice_map_count;
	u32 h264_slice_map_end;
	u32 h264_input_format;
	u32 h264_multipass_inactive;
	u32 h264_num_views[2];
	u32 h264_allow_reordering;
	u32 h264_ref_spacing[3];
	u32 h264_scaling_4x4;
	u32 h264_scaling_8x8;
	u32 h264_sve_count;
	u32 multi_me; /* u16: 6070 second ME unit for two active references */
	u32 h264_vui_restriction;
	u32 h264_vui_mv_over_boundaries;
	u32 h264_vui_max_bytes_denom;
	u32 h264_vui_max_bits_denom;
	u32 h264_vui_max_mv_h;
	u32 h264_vui_max_mv_v;
	u32 h264_vui_max_reorder;
	u32 h264_vui_max_dec_buffering;
	u32 hevc_pad;
	u32 hevc_rc_270c;
	u32 hevc_rc_9604;
	u32 hevc_rc_9608;
	u32 hevc_profile_a;
	u32 hevc_compat_a;
	u32 hevc_8bit_a;
	u32 hevc_level_a;
	u32 hevc_profile_b;
	u32 hevc_compat_b;
	u32 hevc_8bit_b;
	u32 hevc_level_b;
	u32 hevc_width;
	u32 hevc_height;
	u32 hevc_width_ctb;
	u32 hevc_height_ctb;
	u32 hevc_conf;
	u32 hevc_conf_left;
	u32 hevc_conf_right;
	u32 hevc_conf_top;
	u32 hevc_conf_bottom;
	u32 hevc_chroma_format;
	u32 hevc_depth_luma;
	u32 hevc_depth_chroma;
	u32 hevc_log2_max_poc_lsb_minus4;
	u32 hevc_refs;
	u32 hevc_reorder_copy;
	u32 hevc_dpb_minus1;
	u32 hevc_reorder;
	u32 hevc_reorder_on;
	u32 hevc_input_format;
	u32 hevc_pps_count;
	u32 hevc_slice_map_count;
	u32 hevc_slice_map_end;
	u32 hevc_sao_config;
	u32 hevc_sao_offsets;
	u32 hevc_input_depth;
	u32 hevc_num_views[2];
	u32 hevc_sve_count;
	u32 hevc_sve_index;
	u32 hevc_vps_base_internal;
	u32 hevc_vps_base_available;
	u32 hevc_vps_temporal_nesting;
	u32 hevc_vps_sublayer_info;
	u32 hevc_vps_max_dec_buffering;
	u32 hevc_vps_num_reorder;
	u32 hevc_vps_max_latency;
	u32 hevc_sps_temporal_nesting;
	u32 hevc_sps_sublayer_info;
	u32 hevc_sps_max_latency;
	u32 hevc_sps_log2_min_cb;
	u32 hevc_sps_log2_diff_cb;
	u32 hevc_sps_log2_min_tb;
	u32 hevc_sps_log2_diff_tb;
	u32 hevc_sps_depth_inter;
	u32 hevc_sps_depth_intra;
	u32 hevc_sps_scaling_enabled;
	u32 hevc_sps_sao;
	u32 hevc_sps_tmvp;
	u32 hevc_sps_fw_header;
	u32 hevc_sps_header_len;
	u32 hevc_pps_extra_slice_bits;
	u32 hevc_pps_l0_default_active_minus1;
	u32 hevc_pps_l1_default_active_minus1;
	u32 hevc_pps_init_qp;
	u32 hevc_pps_cu_qp_delta;
	u32 hevc_pps_cu_qp_depth;
	u32 hevc_pps_wpp;
	u32 hevc_pps_deblock_present;
	u32 hevc_pps_fw_header;
	u32 hevc_pps_header_len;
	u32 hevc_rps;
	u32 geometry_mode, geometry_depth;
	u32 profile_idc, level_idc, max_refs_copy, mb_size, mb_height;
	u32 crop_copy, crop_right_copy, crop_bottom_copy;
	u32 codec_byte, cabac_copy, transform_copy, hevc_cq, hevc_cq_copy;
	u32 hevc_cq_codec, hevc_cq_picture;
};

struct ave_proc_layout {
	u32 pic_size;
	u32 first_frame;
	u32 first_process; /* u8: first Process command since Start */
	/* Four-entry per-picture source-neighbour address banks. */
	u32 src_neighbor_info, src_neighbor_pixel;
	u32 src_neighbor_data, src_neighbor_fw;
	u32 y, uv; /* ABI-specific plane entry; 6070 is {address, stride, zero} */
	u32 in_w, in_h;
	u32 coded_cap;
	u32 fps_int;
	u32 bitrate;
	u32 limits, limit_bytes, limit_secs;
	u32 const5;
	u32 frame_type;
	u32 fps; /* f64 */
	u32 timescale; /* u64 */
	u32 avg_variance;
	u32 static_area_mask;
	u32 static_area_threshold;
	u32 slot;
	u32 coded, meta; /* u64 addresses */
	u32 ordinal;
	u32 pts; /* u64 */
	u32 force_key; /* 1 on a forced key frame */
	u32 force_type; /* 2 on a forced key frame */
	u32 frame_buf; /* H.264 only */
	u32 shdr_table; /* HEVC only */
	u32 crop; /* HEVC only */
	u32 meta_size, extra, frame_size;
	u32 low_delay; /* HEVC only: 3 with reordering off */
	/* HEVC only: u8; one transcoder writes directly to the coded buffer. */
	u32 hevc_single_transcoder;
	/* HEVC only: nonzero selects the explicit short-term RPS in Process. */
	u32 hevc_use_explicit_rps;
	/* Per-picture reconstruction and motion-search selections. */
	struct ave_recon_selection_layout current_recon, list0_recon;
	struct ave_recon_selection_layout list0_recon_second, list1_recon;
	u32 current_colocated;
	u32 current_lowres, list0_lowres, list0_lowres_second;
	u32 list1_lowres, list1_colocated;
	u32 lowres_results;
	u32 lrme_stats;
	struct ave_retained_partition retained[AVE_MAX_RETAINED_PARTITIONS];
	u8 retained_count;
};

/* Fields written into a slot's coded-header/result buffer. */
struct ave_result_layout {
	u32 coded_bytes;
	u32 coded_bytes_copy;
	u32 parameter_set_bits;
	u32 hevc_slice_header_bytes;
	u32 coded_trim_bytes; /* u8 */
	u32 ordinal;
	u32 frame_type;
};

struct ave_fw_layout {
	const char *name;
	const struct ave_fw_abi *abi;
	enum ave_layout_kind kind;
	bool encode_ready;
	size_t bytes[2][AVE_IMAGE_COUNT];
	struct ave_start_layout start[2];
	struct ave_proc_layout process[2];
	struct ave_result_layout result[2];
	u32 hdr_bytes, meta_bytes;
	u32 process_timeout_us;
	u8 process_subtype, max_slots, mcc_dsid;
	u8 max_sessions;
	u32 codec_state_bytes[2];
	bool counted;
	/*
	 * A counted layout normally runs only 30 fps sessions with B-frames
	 * (H.264 Baseline and intra-only aside). Set where the firmware build
	 * has been shown to encode without B-frames and at other rates too.
	 */
	bool any_gop;
};

extern const struct ave_fw_abi ave_abi_9003, ave_abi_6070;
extern const struct ave_fw_layout ave_layout_9003_78_0;
extern const struct ave_fw_layout ave_layout_6070_11_1;
/* Not selectable until its loader TEXT CRC is measured and qualified. */
extern const struct ave_fw_layout ave_layout_26a428, ave_layout_26b123;
const struct ave_fw_layout *ave_layout_for_crc(u32 crc);

#endif
