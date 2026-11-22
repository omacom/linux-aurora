// SPDX-License-Identifier: GPL-2.0-only
/* Apple AVE command layouts, identified by firmware TEXT CRC. */
#include "ave.h"

const struct ave_fw_layout ave_layout_9003_78_0 = {
	.name = "9003.78.0 / 25G83",
	.abi = &ave_abi_9003,
	.encode_ready = true,
	.bytes = { { 0x3180, 0x63d8 }, { 0x13f28, 0xb1c0 } },
	.hdr_bytes = 0x12c000,
	.meta_bytes = 0xc000,
	.process_timeout_us = 46400000,
	.process_subtype = 0x15,
	.max_slots = 5,
	.mcc_dsid = 0x12,
	.start = {
		[0] = {
			.header = AVE_SURFACE(0x48, 0x10),
			.secondary = AVE_SURFACE(0x58, 0x10),
			.ref_a = AVE_SURFACE(0x390, 0x20),
			.ref_b = AVE_SURFACE(0x3a0, 0x20),
			.ref_c = AVE_SURFACE(0x7d0, 0x20),
			.ref_c_alias = AVE_SURFACE(0x7e0, 0x20),
			.ref_d = AVE_SURFACE(0x1130, 0x10),
			.mb = AVE_SURFACE(0xc10, 0x10),
			.coded = AVE_SURFACE(0xd50, 0x10),
			.meta = AVE_SURFACE(0xf50, 0x10),
			.row_a = AVE_SURFACE(0x1350, 0x10),
			.row_b = AVE_SURFACE(0x1390, 0x10),
			.fixed = AVE_SURFACE(0x13d0, 0x10),
			.hevc_extra = AVE_SURFACE(0x2520, 0x10),
			.tiles = { AVE_SURFACE(0x1410, 0x10),
				AVE_SURFACE(0x1430, 0x10),
				AVE_SURFACE(0x1490, 0x10),
				AVE_SURFACE(0x14b0, 0x10),
				AVE_SURFACE(0x1510, 0x10),
				AVE_SURFACE(0x1530, 0x10),
				AVE_SURFACE(0x1590, 0x10),
				AVE_SURFACE(0x15b0, 0x10) },
			.codec = 0xc0,
			.flag_8bit = 0x208,
			.const_214 = 0x214,
			.const_218 = 0x218,
			.fps = { .numerator = 0x220 },
			.rc_mode = 0x234,
			.bitrate = 0x238,
			.init_qp = 0x240,
			.limits = 0x250,
			.limit_bytes = 0x258,
			.limit_secs = 0x260,
			.f64_288 = 0x288,
			.f64_290 = 0x290,
			.min_qp = 0x298,
			.max_qp = 0x29c,
			.gop_flags = 0x2a0,
			.p_frames = 0x2ac,
			.max_b = 0x2b0,
			.ref_depth = 0x2b4,
			.idr_interval = 0x2b8,
			.f64_2c0 = 0x2c0,
			.f64_2c8 = 0x2c8,
			.rc_flags = 0x300,
			.min_qp2 = 0x308,
			.max_qp2 = 0x30c,
			.rc_310 = 0x310,
			.size_class = 0x320,
			.coded_w = 0x368,
			.coded_h = 0x36c,
			.codec_flags = 0x370,
			.h264_380 = 0x380,
			.h264_384 = 0x384,
			.codec_2588 = 0x2588,
			.bit_depth = 0x2598,
			.coded_h2 = 0x25dc,
			.h264_profile = 0x291c,
			.h264_level = 0x2938,
			.h264_max_refs = 0x2d44,
			.h264_width_mbs = 0x2d4c,
			.h264_height_mbs = 0x2d50,
			.h264_crop = 0x2db4,
			.h264_crop_left = 0x2db8,
			.h264_crop_right = 0x2dbc,
			.h264_crop_top = 0x2dc0,
			.h264_crop_bottom = 0x2dc4,
			.h264_cabac = 0x2fd8,
			.h264_8x8 = 0x3023,
			.hevc_pad = 0x26e0,
			.hevc_rc_270c = 0x270c,
			.hevc_rc_9604 = 0x9604,
			.hevc_rc_9608 = 0x9608,
			.hevc_profile_a = 0x2938,
			.hevc_compat_a = 0x293d,
			.hevc_8bit_a = 0x2962,
			.hevc_level_a = 0x296c,
			.hevc_profile_b = 0x5904,
			.hevc_compat_b = 0x5909,
			.hevc_8bit_b = 0x592e,
			.hevc_level_b = 0x5938,
			.hevc_width = 0x5ab4,
			.hevc_height = 0x5ab8,
			.hevc_width_ctb = 0x5abc,
			.hevc_height_ctb = 0x5ac0,
			.hevc_conf = 0x5ac4,
			.hevc_conf_left = 0x5ac8,
			.hevc_conf_right = 0x5acc,
			.hevc_conf_top = 0x5ad0,
			.hevc_conf_bottom = 0x5ad4,
			.hevc_depth_luma = 0x5ad8,
			.hevc_depth_chroma = 0x5adc,
			.hevc_refs = 0x259c,
			.hevc_reorder_copy = 0x2e54,
			.hevc_dpb_minus1 = 0x5b00,
			.hevc_reorder = 0x5b04,
			.hevc_reorder_on = 0x95fc,
		},
		[1] = {
			.header = AVE_SURFACE(0x48, 0x10),
			.secondary = AVE_SURFACE(0x58, 0x10),
			.ref_a = AVE_SURFACE(0x390, 0x20),
			.ref_b = AVE_SURFACE(0x3a0, 0x20),
			.ref_c = AVE_SURFACE(0x7d0, 0x20),
			.ref_c_alias = AVE_SURFACE(0x7e0, 0x20),
			.ref_d = AVE_SURFACE(0x1130, 0x10),
			.mb = AVE_SURFACE(0xc10, 0x10),
			.coded = AVE_SURFACE(0xd50, 0x10),
			.meta = AVE_SURFACE(0xf50, 0x10),
			.row_a = AVE_SURFACE(0x1350, 0x10),
			.row_b = AVE_SURFACE(0x1390, 0x10),
			.fixed = AVE_SURFACE(0x13d0, 0x10),
			.hevc_extra = AVE_SURFACE(0x2520, 0x10),
			.tiles = { AVE_SURFACE(0x1410, 0x10),
				AVE_SURFACE(0x1430, 0x10),
				AVE_SURFACE(0x1490, 0x10),
				AVE_SURFACE(0x14b0, 0x10),
				AVE_SURFACE(0x1510, 0x10),
				AVE_SURFACE(0x1530, 0x10),
				AVE_SURFACE(0x1590, 0x10),
				AVE_SURFACE(0x15b0, 0x10) },
			.codec = 0xc0,
			.flag_8bit = 0x208,
			.const_214 = 0x214,
			.const_218 = 0x218,
			.fps = { .numerator = 0x220 },
			.rc_mode = 0x234,
			.bitrate = 0x238,
			.init_qp = 0x240,
			.limits = 0x250,
			.limit_bytes = 0x258,
			.limit_secs = 0x260,
			.f64_288 = 0x288,
			.f64_290 = 0x290,
			.min_qp = 0x298,
			.max_qp = 0x29c,
			.gop_flags = 0x2a0,
			.p_frames = 0x2ac,
			.max_b = 0x2b0,
			.ref_depth = 0x2b4,
			.idr_interval = 0x2b8,
			.f64_2c0 = 0x2c0,
			.f64_2c8 = 0x2c8,
			.rc_flags = 0x300,
			.min_qp2 = 0x308,
			.max_qp2 = 0x30c,
			.rc_310 = 0x310,
			.size_class = 0x320,
			.coded_w = 0x368,
			.coded_h = 0x36c,
			.codec_flags = 0x370,
			.h264_380 = 0x380,
			.h264_384 = 0x384,
			.codec_2588 = 0x2588,
			.bit_depth = 0x2598,
			.coded_h2 = 0x25dc,
			.h264_profile = 0x291c,
			.h264_level = 0x2938,
			.h264_max_refs = 0x2d44,
			.h264_width_mbs = 0x2d4c,
			.h264_height_mbs = 0x2d50,
			.h264_crop = 0x2db4,
			.h264_crop_left = 0x2db8,
			.h264_crop_right = 0x2dbc,
			.h264_crop_top = 0x2dc0,
			.h264_crop_bottom = 0x2dc4,
			.h264_cabac = 0x2fd8,
			.h264_8x8 = 0x3023,
			.hevc_pad = 0x26e0,
			.hevc_rc_270c = 0x270c,
			.hevc_rc_9604 = 0x9604,
			.hevc_rc_9608 = 0x9608,
			.hevc_profile_a = 0x2938,
			.hevc_compat_a = 0x293d,
			.hevc_8bit_a = 0x2962,
			.hevc_level_a = 0x296c,
			.hevc_profile_b = 0x5904,
			.hevc_compat_b = 0x5909,
			.hevc_8bit_b = 0x592e,
			.hevc_level_b = 0x5938,
			.hevc_width = 0x5ab4,
			.hevc_height = 0x5ab8,
			.hevc_width_ctb = 0x5abc,
			.hevc_height_ctb = 0x5ac0,
			.hevc_conf = 0x5ac4,
			.hevc_conf_left = 0x5ac8,
			.hevc_conf_right = 0x5acc,
			.hevc_conf_top = 0x5ad0,
			.hevc_conf_bottom = 0x5ad4,
			.hevc_depth_luma = 0x5ad8,
			.hevc_depth_chroma = 0x5adc,
			.hevc_refs = 0x259c,
			.hevc_reorder_copy = 0x2e54,
			.hevc_dpb_minus1 = 0x5b00,
			.hevc_reorder = 0x5b04,
			.hevc_reorder_on = 0x95fc,
		},
	},
	.process = {
		[0] = {
			.first_frame = 0x12c0,
			.y = 0x5830,
			.uv = 0x5850,
			.in_w = 0x58e8,
			.in_h = 0x58ec,
			.coded_cap = 0x61c8,
			.fps_int = 0x1610,
			.bitrate = 0x1620,
			.limits = 0x1628,
			.limit_bytes = 0x1630,
			.limit_secs = 0x1638,
			.const5 = 0x6238,
			.fps = 0x6268,
			.timescale = 0x6278,
			.slot = 0x61b4,
			.coded = 0x61b8,
			.meta = 0x61c0,
			.ordinal = 0x6228,
			.pts = 0x6270,
			.force_key = 0x1658,
			.force_type = 0x29f8,
			.frame_buf = 0x58c8,
		},
		[1] = {
			.first_frame = 0x5d90,
			.y = 0xa300,
			.uv = 0xa320,
			.in_w = 0xa3b8,
			.in_h = 0xa3bc,
			.coded_cap = 0xac98,
			.fps_int = 0x60e0,
			.bitrate = 0x60f0,
			.limits = 0x60f8,
			.limit_bytes = 0x6100,
			.limit_secs = 0x6108,
			.const5 = 0xad08,
			.fps = 0xad38,
			.timescale = 0xad48,
			.slot = 0xac84,
			.coded = 0xac88,
			.meta = 0xac90,
			.ordinal = 0xacf8,
			.pts = 0xad40,
			.force_key = 0x6128,
			.force_type = 0x74c8,
			.shdr_table = 0xd88,
			.crop = 0x74dc,
			.low_delay = 0xcf8,
		},
	},
	.result = {
		[0] = {
			.coded_bytes = 0x120,
			.coded_bytes_copy = 0x12c,
		},
		[1] = {
			.coded_bytes = 0x120,
			.coded_bytes_copy = 0x12c,
			.hevc_slice_header_bytes = 0x128,
		},
	},
};

/*
 * The macOS 27.0 (26A428) command layout. The macOS 27.2 (26B123) build
 * keeps every field where 26A428 has it; only the Process commands grow
 * (ave_layout_26b123).
 */
#define AVE_LAYOUT_26A428_FIELDS \
	.abi = &ave_abi_9003, \
	.kind = AVE_LAYOUT_26A428, \
	.encode_ready = true, \
	.hdr_bytes = 0xec000, \
	.meta_bytes = 0x10000, \
	.process_timeout_us = 46400000, \
	.process_subtype = 0x13, \
	.max_slots = 7, \
	.mcc_dsid = 0x12, \
	.counted = true, \
	.start = { \
		[0] = { \
			.header = AVE_SURFACE(0x48, 0x10), \
			.secondary = AVE_SURFACE(0x58, 0x10), \
			.ref_a = AVE_SURFACE(0x98, 0x40), \
			.ref_b = AVE_SURFACE(0xa8, 0x40), \
			.ref_c = AVE_SURFACE(0xfe8, 0x40), \
			.ref_c_alias = AVE_SURFACE(0xff8, 0x40), \
			.ref_d = AVE_SURFACE(0xdc0, 0x10), \
			.ref_e = AVE_SURFACE(0xb8, 0x40), \
			.ref_f = AVE_SURFACE(0xc8, 0x40), \
			.mb = AVE_SURFACE(0x1870, 0x10), \
			.coded = AVE_SURFACE(0x920, 0x10), \
			.meta = AVE_SURFACE(0xb08, 0x10), \
			.row_a = AVE_SURFACE(0x19c0, 0x10), \
			.row_b = AVE_SURFACE(0x1a08, 0x10), \
			.fixed = AVE_SURFACE(0x1a50, 0x10), \
			.hevc_extra = AVE_SURFACE(0x70, 0x10), \
			.tiles = { AVE_SURFACE(0x1bb8, 0x10), \
				AVE_SURFACE(0x1bd8, 0x10), \
				AVE_SURFACE(0x1db8, 0x10), \
				AVE_SURFACE(0x1dd8, 0x10), \
				AVE_SURFACE(0x1bf8, 0x10), \
				AVE_SURFACE(0x1c18, 0x10), \
				AVE_SURFACE(0x1df8, 0x10), \
				AVE_SURFACE(0x1e18, 0x10) }, \
			.refs_count = 0x94, \
			.work_count = 0xdbc, \
			.lfs_count = 0xfe4, \
			.slots_count = 0x918, \
			.meta_count = 0xb00, \
			.mb_count = 0x1868, \
			.row_a_count = 0x19b8, \
			.row_b_count = 0x1a00, \
			.fixed_count = 0x1a48, \
			.tiles_count = 0x1bb4, \
			.codec = 0x3248, \
			.flag_8bit = 0x35e8, \
			.const_214 = 0x35f4, \
			.const_218 = 0x35f8, \
			.fps = { .numerator = 0x3600 }, \
			.rc_mode = 0x3614, \
			.bitrate = 0x3618, \
			.init_qp = 0x3620, \
			.limits = 0x3630, \
			.limit_bytes = 0x3638, \
			.limit_secs = 0x3640, \
			.f64_288 = 0x3668, \
			.f64_290 = 0x3670, \
			.min_qp = 0x3678, \
			.max_qp = 0x367c, \
			.gop_flags = 0x36d0, \
			.p_frames = 0x36dc, \
			.max_b = 0x36e0, \
			.ref_depth = 0x36e4, \
			.idr_interval = 0x36e8, \
			.f64_2c0 = 0x36f0, \
			.f64_2c8 = 0x36f8, \
			.rc_flags = 0x3734, \
			.min_qp2 = 0x3740, \
			.max_qp2 = 0x3744, \
			.rc_310 = 0x3748, \
			.size_class = 0x3760, \
			.coded_w = 0x322c, \
			.coded_h = 0x3230, \
			.coded_h2 = 0x32a0, \
			.bit_depth = 0x383c, \
			.h264_profile = 0x37b0, \
			.h264_level = 0x37b8, \
			.h264_max_refs = 0x37d0, \
			.h264_crop = 0x3800, \
			.h264_crop_right = 0x3804, \
			.h264_crop_bottom = 0x3808, \
			.h264_cabac = 0x380c, \
			.h264_8x8 = 0x3823, \
			.geometry_mode = 0x31f8, \
			.geometry_depth = 0x3234, \
			.profile_idc = 0x3a98, \
			.level_idc = 0x3aa0, \
			.max_refs_copy = 0x3abe, \
			.mb_size = 0x3ac0, \
			.mb_height = 0x37d4, \
			.crop_copy = 0x3ac7, \
			.crop_right_copy = 0x3aca, \
			.crop_bottom_copy = 0x3ace, \
			.codec_byte = 0x3830, \
			.cabac_copy = 0x3af9, \
			.transform_copy = 0x3b0d, \
		}, \
		[1] = { \
			.header = AVE_SURFACE(0x48, 0x10), \
			.secondary = AVE_SURFACE(0x58, 0x10), \
			.ref_a = AVE_SURFACE(0x98, 0x40), \
			.ref_b = AVE_SURFACE(0xa8, 0x40), \
			.ref_c = AVE_SURFACE(0xfe8, 0x40), \
			.ref_c_alias = AVE_SURFACE(0xff8, 0x40), \
			.ref_d = AVE_SURFACE(0xdc0, 0x10), \
			.ref_e = AVE_SURFACE(0xb8, 0x40), \
			.ref_f = AVE_SURFACE(0xc8, 0x40), \
			.mb = AVE_SURFACE(0x1870, 0x10), \
			.coded = AVE_SURFACE(0x920, 0x10), \
			.meta = AVE_SURFACE(0xb08, 0x10), \
			.row_a = AVE_SURFACE(0x19c0, 0x10), \
			.row_b = AVE_SURFACE(0x1a08, 0x10), \
			.fixed = AVE_SURFACE(0x1a50, 0x10), \
			.hevc_extra = AVE_SURFACE(0x70, 0x10), \
			.tiles = { AVE_SURFACE(0x1bb8, 0x10), \
				AVE_SURFACE(0x1bd8, 0x10), \
				AVE_SURFACE(0x1db8, 0x10), \
				AVE_SURFACE(0x1dd8, 0x10), \
				AVE_SURFACE(0x1bf8, 0x10), \
				AVE_SURFACE(0x1c18, 0x10), \
				AVE_SURFACE(0x1df8, 0x10), \
				AVE_SURFACE(0x1e18, 0x10) }, \
			.refs_count = 0x94, \
			.work_count = 0xdbc, \
			.lfs_count = 0xfe4, \
			.slots_count = 0x918, \
			.meta_count = 0xb00, \
			.mb_count = 0x1868, \
			.row_a_count = 0x19b8, \
			.row_b_count = 0x1a00, \
			.fixed_count = 0x1a48, \
			.tiles_count = 0x1bb4, \
			.codec = 0x3248, \
			.flag_8bit = 0x35e8, \
			.const_214 = 0x35f4, \
			.const_218 = 0x35f8, \
			.fps = { .numerator = 0x3600 }, \
			.rc_mode = 0x3614, \
			.bitrate = 0x3618, \
			.init_qp = 0x3620, \
			.limits = 0x3630, \
			.limit_bytes = 0x3638, \
			.limit_secs = 0x3640, \
			.f64_288 = 0x3668, \
			.f64_290 = 0x3670, \
			.min_qp = 0x3678, \
			.max_qp = 0x367c, \
			.gop_flags = 0x36d0, \
			.p_frames = 0x36dc, \
			.max_b = 0x36e0, \
			.ref_depth = 0x36e4, \
			.idr_interval = 0x36e8, \
			.f64_2c0 = 0x36f0, \
			.f64_2c8 = 0x36f8, \
			.rc_flags = 0x3734, \
			.min_qp2 = 0x3740, \
			.max_qp2 = 0x3744, \
			.rc_310 = 0x3748, \
			.size_class = 0x3760, \
			.coded_w = 0x322c, \
			.coded_h = 0x3230, \
			.coded_h2 = 0x32a0, \
			.bit_depth = 0x37c0, \
			.hevc_pad = 0x37ec, \
			.hevc_refs = 0x37c4, \
			.hevc_rc_9604 = 0x9dec, \
			.hevc_rc_9608 = 0x9df0, \
			.hevc_profile_a = 0x3a38, \
			.hevc_compat_a = 0x3a3d, \
			.hevc_8bit_a = 0x3a62, \
			.hevc_level_a = 0x3a6c, \
			.hevc_profile_b = 0x6508, \
			.hevc_compat_b = 0x650d, \
			.hevc_8bit_b = 0x6532, \
			.hevc_level_b = 0x653c, \
			.hevc_width = 0x66b8, \
			.hevc_height = 0x66bc, \
			.hevc_width_ctb = 0x66c0, \
			.hevc_height_ctb = 0x66c4, \
			.hevc_conf = 0x66c8, \
			.hevc_conf_left = 0x66cc, \
			.hevc_conf_right = 0x66d0, \
			.hevc_conf_top = 0x66d4, \
			.hevc_conf_bottom = 0x66d8, \
			.hevc_depth_luma = 0x66dc, \
			.hevc_depth_chroma = 0x66e0, \
			.hevc_reorder_copy = 0x495c, \
			.hevc_dpb_minus1 = 0x6704, \
			.hevc_reorder = 0x6708, \
			.geometry_mode = 0x31f8, \
			.geometry_depth = 0x3234, \
			.hevc_cq = 0x3708, \
			.hevc_cq_copy = 0xbefc, \
			.hevc_cq_codec = 0x37c8, \
			.hevc_cq_picture = 0x9e38, \
		}, \
	}, \
	.process = { \
		[0] = { \
			.first_frame = 0xdc0, \
			.fps_int = 0x1328, \
			.in_w = 0x5648, \
			.in_h = 0x564c, \
			.ordinal = 0x5650, \
			.const5 = 0x5660, \
			.fps = 0x5758, \
			.pts = 0x5760, \
			.timescale = 0x5768, \
			.slot = 0x57b8, \
			.y = 0x57c0, \
			.uv = 0x57e0, \
			.coded = 0x58a0, \
			.coded_cap = 0x58a8, \
			.meta = 0x58b0, \
			.meta_size = 0x58b8, \
			.extra = 0x58c0, \
			.frame_buf = 0x58d8, \
			.frame_size = 0x58e0, \
		}, \
		[1] = { \
			.first_frame = 0x1568, \
			.fps_int = 0x1ad0, \
			.in_w = 0x5df0, \
			.in_h = 0x5df4, \
			.ordinal = 0x5df8, \
			.const5 = 0x5e08, \
			.fps = 0x5f00, \
			.pts = 0x5f08, \
			.timescale = 0x5f10, \
			.slot = 0x5f60, \
			.y = 0x5f68, \
			.uv = 0x5f88, \
			.coded = 0x6048, \
			.coded_cap = 0x6050, \
			.meta = 0x6058, \
			.meta_size = 0x6060, \
			.extra = 0x6068, \
			.crop = 0x3190, \
		}, \
	}, \
	.result = { \
		[0] = { \
			.coded_bytes = 0x120, \
			.coded_bytes_copy = 0x12c, \
		}, \
		[1] = { \
			.coded_bytes = 0x120, \
			.coded_bytes_copy = 0x12c, \
			.hevc_slice_header_bytes = 0x128, \
		}, \
	},

const struct ave_fw_layout ave_layout_26a428 = {
	.name = "26A428",
	.bytes = { { 0x3b40, 0x6df8 }, { 0x13ee0, 0x78b8 } },
	AVE_LAYOUT_26A428_FIELDS
};

/*
 * macOS 27.2 (26B123), the build the M4 (T8132) runs: Start commands as in
 * 26A428, Process commands 0x10 bytes longer for each codec. The fields the
 * driver writes keep their 26A428 offsets, and the bytes the build adds are
 * left zero.
 */
const struct ave_fw_layout ave_layout_26b123 = {
	.name = "26B123",
	.bytes = { { 0x3b40, 0x6e08 }, { 0x13ee0, 0x78c8 } },
	AVE_LAYOUT_26A428_FIELDS
	.any_gop = true,
};

/*
 * 6070 carries fixed-capacity address banks and split size arrays.  The two
 * SoCs use the same command layout; streaming remains gated until hello and
 * live bitstream qualification establish that the complete contract works.
 */
const struct ave_fw_layout ave_layout_6070_11_1 = {
	.name = "6070.11.1 (hardware gated)",
	.abi = &ave_abi_6070,
	.kind = AVE_LAYOUT_6070,
	.encode_ready = true,
	.bytes = { { 0x10e10, 0x1940 }, { 0x32dc8, 0x6838 } },
	.hdr_bytes = 0x12c000,
	.meta_bytes = AVE_6070_CODED_HEADER_BYTES,
	.process_timeout_us = 46400000,
	.process_subtype = 0x15,
	.max_slots = 5,
	.max_sessions = 1,
	.codec_state_bytes = { 0x984, 0x5884 },
	.start = {
		[0] = {
			.header = AVE_SURFACE(0x40, 0x10),
			.secondary = AVE_SURFACE(0x50, 0x10),
			/* Firmware-generated SPS/PPS output; mandatory on 6070.11.1. */
			.param_sets = AVE_SURFACE(0xfb30, 0x10),
			.coded = { .address = 0x4b8, .size = 0x558,
				   .address_stride = 8, .size_stride = 4 },
			.meta = { .address = 0x5c0, .size = 0x660,
				  .address_stride = 8, .size_stride = 4 },
			.row_a = AVE_ADDRESS_BANK(0xf7d0, 8),
			.row_b = AVE_ADDRESS_BANK(0xf7f0, 8),
			.fixed = AVE_ADDRESS_BANK(0xf810, 8),
			.neighbour_fw = AVE_ADDRESS_BANK(0xfed0, 8),
			.recon = { .address = 0x88, .entries = 17, .banks = 2,
				   .entry_stride = 0x10, .bank_stride = 0x110,
				   .values = 2 },
			.colocated = { .address = 0xf6b0, .entries = 17, .banks = 2,
				       .entry_stride = 8, .bank_stride = 0x88,
				       .values = 1 },
			.lfs = { .address = 0x2a8, .entries = 17, .banks = 2,
				 .entry_stride = 8, .bank_stride = 0x88, .values = 1 },
			.lfs_result = { .address = 0x3b8, .entries = 8, .banks = 2,
					.entry_stride = 8, .bank_stride = 0x88,
					.values = 1 },
			.entropy = { .address = 0xf830, .size = 0xfa30,
				     .entries = 16, .sets = 4,
				     .address_entry_stride = 0x20,
				     .address_set_stride = 8,
				     .size_entry_stride = 0x10,
				     .size_set_stride = 4 },
			.fps = { .numerator = 0xff4c, .denominator = 0xff48 },
			.rc_mode = 0xff50,
			/* 6070 numbers constant QP 2, not 3. */
			.rc_mode_fixed_qp = 2,
			.bitrate = 0xff30,
			.init_qp = 0xffb4,
			.init_qp_words = 3,
			.limits = 0x105a8,
			.limit_bytes = 0x10588,
			.limit_secs = 0x10590,
			.min_qp = 0xff88,
			.max_qp = 0xff8c,
			.max_b = 0x78,
			.idr_interval = 0xff34,
			.rc_flags = 0xfcd8,
			.min_qp2 = 0xff90,
			.max_qp2 = 0xff94,
			.rc_310 = 0xfcec,
			.coded_w = 0x60,
			.coded_h = 0x64,
			.ref_compression = 0xfd7d,
			.h264_profile = 0x105b4,
			.h264_level = 0x105d0,
			.h264_sps_id = 0x105d4,
			.h264_chroma = 0x105d8,
			.h264_depth_luma = 0x105e0,
			.h264_depth_chroma = 0x105e4,
			.h264_frame_num_bits = 0x109cc,
			.h264_poc_type = 0x109d0,
			.h264_poc_bits = 0x109d4,
			.h264_max_refs = 0x109dc,
			.h264_gaps = 0x109e0,
			.h264_width_mbs = 0x109e4,
			.h264_height_mbs = 0x109e8,
			.h264_frame_only = 0x109ec,
			.h264_direct = 0x109f0,
			.h264_vui = 0x109f4,
			.h264_crop = 0x10a40,
			.h264_crop_left = 0x10a44,
			.h264_crop_right = 0x10a48,
			.h264_crop_top = 0x10a4c,
			.h264_crop_bottom = 0x10a50,
			.h264_fw_header = 0x10a54,
			.h264_header_len = 0x10a58,
			.h264_pps_id = 0x10c60,
			.h264_pps_sps_id = 0x10c64,
			.h264_cabac = 0x10c68,
			.h264_slice_groups = 0x10c70,
			.h264_pic_init_qp = 0x10c88,
			.h264_deblock = 0x10cb0,
			.h264_constrained = 0x10cb1,
			.h264_redundant = 0x10cb2,
			.h264_8x8 = 0x10cb3,
			.h264_pps_fw_header = 0x10cd8,
			.h264_max_mvs_per_2mb = 0x70,
			.h264_adapt_b = 0x7d,
			.h264_pixel_packing = 0xfce8,
			.h264_transform_mode = 0xfcec,
			.h264_skip_mode = 0xfcf0,
			.h264_input_bit_depth = 0xfd20,
			.h264_parameter_set_count = 0xfda4,
			.h264_slice_map_count = 0xfdac,
			.h264_slice_map_end = 0xfdb4,
			.h264_input_format = 0xfeb4,
			.h264_multipass_inactive = 0xff04,
			.h264_num_views = { 0xff24, 0xff28 },
			.h264_allow_reordering = 0xff40,
			.h264_ref_spacing = { 0x10574, 0x10578, 0x1057c },
			.h264_scaling_4x4 = 0x1060a,
			.h264_scaling_8x8 = 0x106ca,
			.h264_sve_count = 0x10de8,
			.multi_me = 0xfcea,
			.h264_vui_restriction = 0x10a20,
			.h264_vui_mv_over_boundaries = 0x10a24,
			.h264_vui_max_bytes_denom = 0x10a28,
			.h264_vui_max_bits_denom = 0x10a2c,
			.h264_vui_max_mv_h = 0x10a30,
			.h264_vui_max_mv_v = 0x10a34,
			.h264_vui_max_reorder = 0x10a38,
			.h264_vui_max_dec_buffering = 0x10a3c,
		},
		[1] = {
			.header = AVE_SURFACE(0x40, 0x10),
			.secondary = AVE_SURFACE(0x50, 0x10),
			/* Firmware-generated VPS/SPS/PPS output; mandatory on 6070.11.1. */
			.param_sets = AVE_SURFACE(0xfb30, 0x10),
			.coded = { .address = 0x4b8, .size = 0x558,
				   .address_stride = 8, .size_stride = 4 },
			.meta = { .address = 0x5c0, .size = 0x660,
				  .address_stride = 8, .size_stride = 4 },
			.row_a = AVE_ADDRESS_BANK(0xf7d0, 8),
			.row_b = AVE_ADDRESS_BANK(0xf7f0, 8),
			.fixed = AVE_ADDRESS_BANK(0xf810, 8),
			.neighbour_fw = AVE_ADDRESS_BANK(0xfed0, 8),
			.hevc_extra = AVE_ADDRESS_BANK(0xfcc8, 8),
			.hevc_transcoded = { .address = 0x5a8, .size = 0x5b8,
					       .address_stride = 8, .size_stride = 0 },
			.recon = { .address = 0x88, .entries = 17, .banks = 2,
				   .entry_stride = 0x10, .bank_stride = 0x110,
				   .values = 2 },
			.colocated = { .address = 0xf6b0, .entries = 17, .banks = 2,
				       .entry_stride = 8, .bank_stride = 0x88,
				       .values = 1 },
			.lfs = { .address = 0x2a8, .entries = 17, .banks = 2,
				 .entry_stride = 8, .bank_stride = 0x88, .values = 1 },
			.lfs_result = { .address = 0x3b8, .entries = 8, .banks = 2,
					.entry_stride = 8, .bank_stride = 0x88,
					.values = 1 },
			.entropy = { .address = 0xf830, .size = 0xfa30,
				     .entries = 16, .sets = 2,
				     .address_entry_stride = 0x20,
				     .address_set_stride = 8,
				     .size_entry_stride = 0x10,
				     .size_set_stride = 4 },
			.fps = { .numerator = 0xff4c, .denominator = 0xff48 },
			.rc_mode = 0xff50,
			/* The same rate-control numbering as H.264. */
			.rc_mode_fixed_qp = 2,
			.bitrate = 0xff30,
			.init_qp = 0xffb4,
			.init_qp_words = 3,
			.limits = 0x105a8,
			.limit_bytes = 0x10588,
			.limit_secs = 0x10590,
			.min_qp = 0xff88,
			.max_qp = 0xff8c,
			.max_b = 0x78,
			.idr_interval = 0xff34,
			.rc_flags = 0xfcd8,
			.min_qp2 = 0xff90,
			.max_qp2 = 0xff94,
			.rc_310 = 0xfcec,
			.coded_w = 0x60,
			.coded_h = 0x64,
			.ref_compression = 0xfd7d,
			.hevc_refs = 0xfd2c,
			.hevc_profile_a = 0x105d0,
			.hevc_compat_a = 0x105d4,
			.hevc_8bit_a = 0x105f4,
			.hevc_level_a = 0x10604,
			.hevc_profile_b = 0x246ac,
			.hevc_compat_b = 0x246b0,
			.hevc_8bit_b = 0x246d0,
			.hevc_level_b = 0x246e0,
			.hevc_width = 0x248d0,
			.hevc_height = 0x248d4,
			.hevc_width_ctb = 0x248d8,
			.hevc_height_ctb = 0x248dc,
			.hevc_conf = 0x248e0,
			.hevc_conf_left = 0x248e4,
			.hevc_conf_right = 0x248e8,
			.hevc_conf_top = 0x248ec,
			.hevc_conf_bottom = 0x248f0,
			.hevc_chroma_format = 0x248c8,
			.hevc_depth_luma = 0x248f4,
			.hevc_depth_chroma = 0x248f8,
			.hevc_log2_max_poc_lsb_minus4 = 0x248fc,
			.hevc_dpb_minus1 = 0x24904,
			.hevc_reorder = 0x24920,
			.hevc_reorder_copy = 0x10c4c,
			.hevc_input_format = 0xfeb4,
			.hevc_pps_count = 0xfda4,
			.hevc_slice_map_count = 0xfdac,
			.hevc_slice_map_end = 0xfdb4,
			.hevc_sao_config = 0xfd10,
			.hevc_sao_offsets = 0xfd1c,
			.hevc_input_depth = 0xfd20,
			.hevc_num_views = { 0xff24, 0xff28 },
			.hevc_sve_count = 0x32d9c,
			.hevc_sve_index = 0x32dc0,
			.multi_me = 0xfcea,
			.hevc_vps_base_internal = 0x105b8,
			.hevc_vps_base_available = 0x105b9,
			.hevc_vps_temporal_nesting = 0x105c4,
			.hevc_vps_sublayer_info = 0x10c28,
			.hevc_vps_max_dec_buffering = 0x10c2c,
			.hevc_vps_num_reorder = 0x10c4c,
			.hevc_vps_max_latency = 0x10c6c,
			.hevc_sps_temporal_nesting = 0x246a0,
			.hevc_sps_sublayer_info = 0x24900,
			.hevc_sps_max_latency = 0x2493c,
			.hevc_sps_log2_min_cb = 0x24958,
			.hevc_sps_log2_diff_cb = 0x2495c,
			.hevc_sps_log2_min_tb = 0x24960,
			.hevc_sps_log2_diff_tb = 0x24964,
			.hevc_sps_depth_inter = 0x24968,
			.hevc_sps_depth_intra = 0x2496c,
			.hevc_sps_scaling_enabled = 0x24970,
			.hevc_sps_sao = 0x2621d,
			.hevc_sps_tmvp = 0x262b8,
			.hevc_sps_fw_header = 0x26375,
			.hevc_sps_header_len = 0x26378,
			.hevc_pps_extra_slice_bits = 0x2848c,
			.hevc_pps_l0_default_active_minus1 = 0x28494,
			.hevc_pps_l1_default_active_minus1 = 0x28498,
			.hevc_pps_init_qp = 0x2849c,
			.hevc_pps_cu_qp_delta = 0x284a2,
			.hevc_pps_cu_qp_depth = 0x284a4,
			.hevc_pps_wpp = 0x284ed,
			.hevc_pps_deblock_present = 0x28cfe,
			.hevc_pps_fw_header = 0x2a60c,
			.hevc_pps_header_len = 0x2a610,
			.hevc_rps = 0x2cfbc,
		},
	},
	.process = {
		[0] = {
			.pic_size = 0x9c8,
			.first_process = 0x10a0,
			.src_neighbor_info = 0x1348,
			.src_neighbor_pixel = 0x1368,
			.src_neighbor_data = 0x1388,
			.src_neighbor_fw = 0x13a8,
			.y = 0x1288,
			.uv = 0x1298,
			.in_w = 0x132c,
			.in_h = 0x1330,
			.coded_cap = 0x15e0,
			.fps_int = 0x9d4,
			.bitrate = 0x9d0,
			.limits = 0x9d8,
			.limit_bytes = 0x9e0,
			.limit_secs = 0x9e8,
			.frame_type = 0x1674,
			.fps = 0x9f0,
			.timescale = 0x1688,
			.avg_variance = 0x1000,
			.static_area_mask = 0x108c,
			.static_area_threshold = 0x1098,
			.slot = 0x15cc,
			.coded = 0x15d0,
			.meta = 0x15d8,
			.ordinal = 0x1670,
			.pts = 0x1680,
			.force_key = 0x10b4,
			.frame_buf = 0x15e8,
			.current_recon = {
				.luma_data = 0x1260,
				.luma_metadata = 0x1268,
				.chroma_data = 0x1270,
				.chroma_metadata = 0x1278,
			},
			.list0_recon = {
				.luma_data = 0x10c0,
				.luma_metadata = 0x10e0,
				.chroma_data = 0x1100,
				.chroma_metadata = 0x1120,
			},
			.list0_recon_second = {
				.luma_data = 0x10c8,
				.luma_metadata = 0x10e8,
				.chroma_data = 0x1108,
				.chroma_metadata = 0x1128,
			},
			.list1_recon = {
				.luma_data = 0x1160,
				.luma_metadata = 0x1180,
				.chroma_data = 0x11a0,
				.chroma_metadata = 0x11c0,
			},
			.current_colocated = 0x1280,
			.current_lowres = 0x15e8,
			.list0_lowres = 0x1140,
			.list0_lowres_second = 0x1148,
			.list1_lowres = 0x11e0,
			.list1_colocated = 0x1200,
			.lowres_results = 0x15f0,
			.lrme_stats = 0x12b8,
			.retained = { { .command_offset = 0x40, .state_offset = 0,
					 .bytes = 0x984 } },
			.retained_count = 1,
		},
		[1] = {
			.pic_size = 0x55b0,
			.hevc_single_transcoder = 0x6515,
			.hevc_use_explicit_rps = 0x6504,
			.first_process = 0x5c88,
			.src_neighbor_info = 0x5f30,
			.src_neighbor_pixel = 0x5f50,
			.src_neighbor_data = 0x5f70,
			.src_neighbor_fw = 0x5f90,
			.y = 0x5e70,
			.uv = 0x5e80,
			.in_w = 0x5f14,
			.in_h = 0x5f18,
			.coded_cap = 0x61c8,
			.fps_int = 0x55bc,
			.bitrate = 0x55b8,
			.limits = 0x55c0,
			.limit_bytes = 0x55c8,
			.limit_secs = 0x55d0,
			.frame_type = 0x625c,
			.fps = 0x55d8,
			.timescale = 0x6270,
			.slot = 0x61b4,
			.coded = 0x61b8,
			.meta = 0x61c0,
			.ordinal = 0x6258,
			.pts = 0x6268,
			.force_key = 0x5c9c,
			.shdr_table = 0x5a8,
			.current_recon = {
				.luma_data = 0x5e48,
				.luma_metadata = 0x5e50,
				.chroma_data = 0x5e58,
				.chroma_metadata = 0x5e60,
			},
			.list0_recon = {
				.luma_data = 0x5ca8,
				.luma_metadata = 0x5cc8,
				.chroma_data = 0x5ce8,
				.chroma_metadata = 0x5d08,
			},
			.list0_recon_second = {
				.luma_data = 0x5cb0,
				.luma_metadata = 0x5cd0,
				.chroma_data = 0x5cf0,
				.chroma_metadata = 0x5d10,
			},
			.list1_recon = {
				.luma_data = 0x5d48,
				.luma_metadata = 0x5d68,
				.chroma_data = 0x5d88,
				.chroma_metadata = 0x5da8,
			},
			.current_colocated = 0x5e68,
			.current_lowres = 0x61d0,
			.list0_lowres = 0x5d28,
			.list0_lowres_second = 0x5d30,
			.list1_lowres = 0x5dc8,
			.list1_colocated = 0x5de8,
			.lowres_results = 0x61d8,
			.retained = {
				{ .command_offset = 0x40, .state_offset = 0,
				  .bytes = 0x5570 },
				{ .command_offset = 0x6524, .state_offset = 0x5570,
				  .bytes = 0x16c },
				{ .command_offset = 0x6690, .state_offset = 0x56dc,
				  .bytes = 0x1a8 },
			},
			.retained_count = 3,
		},
	},
	.result = {
		[0] = {
			/* The driver configures one slice per picture. */
			.coded_bytes = 0x180,
			.parameter_set_bits = 0x98,
			.coded_trim_bytes = 0x38c,
			.ordinal = 0x10c,
			.frame_type = 0x110,
		},
		[1] = {
			.coded_bytes = 0x180,
			.parameter_set_bits = 0x98,
			.hevc_slice_header_bytes = 0x398,
			.coded_trim_bytes = 0x38c,
			.ordinal = 0x10c,
			.frame_type = 0x110,
		},
	},
};

const struct ave_fw_layout *ave_layout_for_crc(u32 crc)
{
	/*
	 * The M4 Pro and M4 Max get the same 25G83 firmware build as the
	 * A18 Pro, built for their own chips, and speak the same command layout.
	 */
	if (crc == AVE_FW_TEXT_CRC_9003_78_0 || crc == AVE_FW_TEXT_CRC_T6040_25G83 ||
	    crc == AVE_FW_TEXT_CRC_T6041_25G83)
		return &ave_layout_9003_78_0;
	if (crc == 0x460c7a7c || crc == 0x25bfd039)
		return &ave_layout_6070_11_1;
	/*
	 * The macOS 27.0 build for the M5 Pro and Max, and the M5's build of the
	 * same firmware version (9013.45.1): one macOS driver builds the
	 * commands for both.
	 */
	if (crc == AVE_FW_TEXT_CRC_T6050_26A428 ||
	    crc == AVE_FW_TEXT_CRC_T8142_26A428)
		return &ave_layout_26a428;
	/*
	 * The macOS 27.3 beta 1 build speaks the same command layout: H.264
	 * and HEVC encode with it on a J714s.
	 */
	if (crc == AVE_FW_TEXT_CRC_T6050_26C5035)
		return &ave_layout_26a428;
	if (crc == AVE_FW_TEXT_CRC_T8132_26B123 ||
	    crc == AVE_FW_TEXT_CRC_T8122_26B123)
		return &ave_layout_26b123;
	/*
	 * The M5's macOS 27.2 build (9013.55.1), between 26A428 and the 27.3
	 * beta, speaks 26A428's layout: on the J815's 27.2 stub, H.264
	 * Baseline (ffmpeg), H.264 and HEVC with B-frames (v4l2-ctl) encode
	 * with it.
	 */
	if (crc == AVE_FW_TEXT_CRC_T8142_26B123)
		return &ave_layout_26a428;
	return NULL;
}
