// SPDX-License-Identifier: GPL-2.0-only
/* Byte equality against the pre-ABI-refactor 9003 command builders. */
#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/module.h>
#include <linux/crc32.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "ave.h"
#include "ave-regs.h"
#include "tests/ave-9003-reference.h"

static void ave_test_buf(struct ave_buf *b, unsigned int *index)
{
	b->dma = 0x820000000ULL + (u64)(*index) * 0x2000000;
	b->size = SZ_16K * (1 + (*index)++);
}

static void ave_test_session(struct ave_session *sess, enum ave_codec codec,
			     u32 width, u32 height, bool bframes)
{
	struct ave_params *p = &sess->p;
	unsigned int index = 0;

	*p = (struct ave_params){
		.codec = codec,
		.width = width,
		.height = height,
		.cw = ALIGN(width, 16),
		.ch = ALIGN(height, 16),
		.stride = ALIGN(width, 64),
		.fps_num = 30,
		.fps_den = 1,
		.fps_int = 30,
		.bframes = bframes,
		.reorder_off = !bframes,
		.slots = bframes ? 5 : 3,
		.refs = bframes ? 5 : 2,
		.gop = 120,
		.rc_mode = 4,
		.bitrate = 8000000,
		.limit_bytes = 2000000,
		.min_qp = -1,
		.max_qp = 51,
		.profile = codec == AVE_CODEC_H264 ? 6 : 1,
		.level = codec == AVE_CODEC_H264 ? 13 : 7,
		.high = codec == AVE_CODEC_H264,
	};
	p->mbs = p->cw / 16 * (p->ch / 16);
	p->coded_cap = ALIGN(3 * p->cw * p->ch / 2, SZ_16K);
	sess->token = 400;
	sess->client = 10;
	ave_test_buf(&sess->hdr, &index);
	ave_test_buf(&sess->client_mem, &index);
	sess->client_mem.size = AVE_6070_CLIENT_MEM_BYTES;
	ave_test_buf(&sess->param_sets, &index);
	for (unsigned int i = 0; i < AVE_MAX_REFS; i++) {
		ave_test_buf(&sess->ref_a[i], &index);
		ave_test_buf(&sess->ref_b[i], &index);
		ave_test_buf(&sess->ref_c[i], &index);
		ave_test_buf(&sess->ref_d[i], &index);
	}
	ave_test_buf(&sess->mb, &index);
	for (unsigned int set = 0; set < AVE_NEIGHBOUR_SETS; set++) {
		ave_test_buf(&sess->row_a[set], &index);
		ave_test_buf(&sess->row_b[set], &index);
		ave_test_buf(&sess->fixed[set], &index);
		ave_test_buf(&sess->neighbour_fw[set], &index);
	}
	for (unsigned int set = 0; set < AVE_ENTROPY_SETS; set++)
		for (unsigned int i = 0; i < AVE_TILE_BUFS; i++) {
			ave_test_buf(&sess->tile[set][i], &index);
			ave_test_buf(&sess->tile_aux[set][i], &index);
		}
	for (unsigned int i = 0; i < 2; i++)
		ave_test_buf(&sess->hevc_extra[i], &index);
	for (unsigned int i = 0; i < 2; i++)
		ave_test_buf(&sess->hevc_transcoded[i], &index);
	sess->hevc_transcoded[1].size = sess->hevc_transcoded[0].size;
	for (unsigned int i = 0; i < p->slots; i++) {
		ave_test_buf(&sess->slot[i].coded, &index);
		ave_test_buf(&sess->slot[i].meta, &index);
		ave_test_buf(&sess->slot[i].aux, &index);
	}
}

static void ave_test_6070_session(struct kunit *test, struct ave_dev *ave,
				  struct ave_ctx *ctx, struct ave_session *sess,
				  enum ave_codec codec, u32 width, u32 height)
{
	struct ave_fw_layout *fw = kunit_kmalloc(test, sizeof(*fw), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, fw);
	*fw = ave_layout_6070_11_1;
	fw->encode_ready = true;
	ave->layout = fw;
	ctx->ave = ave;
	ctx->sess = sess;
	ave_test_session(sess, codec, width, height, true);
	sess->p.rc_mode = 3;
	sess->p.bitrate = 4000000;
	sess->p.limit_bytes = 0;
	sess->p.initial_qp = 33;
	sess->p.min_qp = 0;
	sess->p.max_qp = 48;
	sess->p.gop = 30;
	sess->p.profile = codec == AVE_CODEC_H264 ? 4 : 1;
	sess->p.level = codec == AVE_CODEC_H264 ? 13 : 7;
	sess->p.high = false;
	sess->p.bit_depth = 8;
	sess->p.coded_cap = ALIGN(2 * sess->p.coded_cap, SZ_16K);
	sess->codec_state_bytes = fw->codec_state_bytes[codec - 1];
	sess->codec_state = kunit_kzalloc(test, sess->codec_state_bytes, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, sess->codec_state);
	KUNIT_ASSERT_EQ(test, ave_init_codec_state(sess), 0);
}

/* These are synthetic DART addresses, never addresses from a device capture. */
static void ave_mask_dart(u8 *cmd, size_t len)
{
	for (size_t off = 0; off + 8 <= len; off += 8) {
		u64 val = get_unaligned_le64(cmd + off);

		if (val >= 0x800000000ULL && val < 0x1000000000ULL)
			put_unaligned_le64(0, cmd + off);
	}
}

static void ave_9003_byte_equality(struct kunit *test)
{
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_ctx *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct ave_session *sess =
		kunit_kzalloc(test, sizeof(*sess), GFP_KERNEL);
	u8 *actual = kunit_kmalloc(test, 0x14000, GFP_KERNEL);
	u8 *expected = kunit_kmalloc(test, 0x14000, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, ave);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	KUNIT_ASSERT_NOT_NULL(test, sess);
	KUNIT_ASSERT_NOT_NULL(test, actual);
	KUNIT_ASSERT_NOT_NULL(test, expected);
	ave->layout = &ave_layout_9003_78_0;
	ctx->ave = ave;
	ctx->sess = sess;
	for (unsigned int codec = AVE_CODEC_H264; codec <= AVE_CODEC_HEVC;
	     codec++) {
		for (unsigned int setting = 0; setting < 3; setting++) {
			size_t len =
				ave->layout->bytes[codec - 1][AVE_IMAGE_START];

			ave_test_session(sess, codec,
					 setting == 2 ? 3840 : 1920,
					 setting == 2 ? 2160 : 1080,
					 setting != 1);
			memset(actual, 0, len);
			memset(expected, 0, len);
			KUNIT_ASSERT_EQ(test, ave_build_start(ctx, actual, len),
					0);
			sess->token = 400;
			ave_reference_start(ctx, expected, len);
			ave_stamp_command(ave->layout->abi, actual, 94400000,
					  0x12345678);
			put_unaligned_le64(94400000, expected + 0x30);
			put_unaligned_le64(0x12345678, expected + 0x38);
			KUNIT_EXPECT_MEMEQ(test, actual, expected, len);
			ave_mask_dart(actual, len);
			ave_mask_dart(expected, len);
			KUNIT_EXPECT_MEMEQ(test, actual, expected, len);
			for (unsigned int frame = 0; frame < 2; frame++) {
				unsigned int slot = frame ? sess->p.slots - 1 :
							    0;

				len = ave->layout->bytes[codec - 1]
							[AVE_IMAGE_PROCESS];
				sess->frames = frame ? 7 : 0;
				sess->token = 500;
				memset(actual, 0, len);
				memset(expected, 0, len);
				KUNIT_ASSERT_EQ(
					test,
					ave_build_process(ctx, actual, len,
							  slot, 0x8f0000000ULL,
							  9000000, !!frame,
							  false),
					0);
				sess->token = 500;
				ave_reference_process(ctx, expected, len, slot,
						      0x8f0000000ULL, 9000000,
						      !!frame);
				ave_stamp_command(ave->layout->abi, actual,
						  94400000, 0x12345678);
				put_unaligned_le64(94400000, expected + 0x30);
				put_unaligned_le64(0x12345678, expected + 0x38);
				KUNIT_EXPECT_MEMEQ(test, actual, expected, len);
				ave_mask_dart(actual, len);
				ave_mask_dart(expected, len);
				KUNIT_EXPECT_MEMEQ(test, actual, expected, len);
			}
		}
	}
}

/*
 * HEVC Main 10 on 9003: compared with the same Main session, Start differs
 * only in the fields the 10-30 capture showed changing with bit depth.
 */
static void ave_9003_hevc_main10(struct kunit *test)
{
	static const u32 changed[] = {
		0x208, 0x298, 0x308, 0x2598, 0x2938, 0x293c, 0x2960,
		0x5904, 0x5908, 0x592c, 0x5ad8, 0x5adc,
	};
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_ctx *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct ave_session *sess = kunit_kzalloc(test, sizeof(*sess), GFP_KERNEL);
	u8 *main8 = kunit_kzalloc(test, 0x14000, GFP_KERNEL);
	u8 *main10 = kunit_kzalloc(test, 0x14000, GFP_KERNEL);
	size_t len;

	KUNIT_ASSERT_NOT_NULL(test, ave);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	KUNIT_ASSERT_NOT_NULL(test, sess);
	KUNIT_ASSERT_NOT_NULL(test, main8);
	KUNIT_ASSERT_NOT_NULL(test, main10);
	ave->layout = &ave_layout_9003_78_0;
	ctx->ave = ave;
	ctx->sess = sess;
	len = ave->layout->bytes[AVE_CODEC_HEVC - 1][AVE_IMAGE_START];

	ave_test_session(sess, AVE_CODEC_HEVC, 1920, 1080, true);
	sess->p.min_qp = 0;
	KUNIT_ASSERT_EQ(test, ave_build_start(ctx, main8, len), 0);
	ave_test_session(sess, AVE_CODEC_HEVC, 1920, 1080, true);
	sess->p.min_qp = -12;
	sess->p.profile = 2;
	sess->p.bit_depth = 10;
	KUNIT_ASSERT_EQ(test, ave_build_start(ctx, main10, len), 0);
	ave_stamp_command(ave->layout->abi, main8, 1, 2);
	ave_stamp_command(ave->layout->abi, main10, 1, 2);

	KUNIT_EXPECT_EQ(test, get_unaligned_le32(main8 + 0x208), 1);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(main10 + 0x208), 0);
	KUNIT_EXPECT_EQ(test, (s32)get_unaligned_le32(main10 + 0x298), -12);
	KUNIT_EXPECT_EQ(test, (s32)get_unaligned_le32(main10 + 0x308), -12);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(main10 + 0x2598), 10);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(main10 + 0x2938), 2);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(main10 + 0x5904), 2);
	KUNIT_EXPECT_EQ(test, main10[0x293d], 0);
	KUNIT_EXPECT_EQ(test, main10[0x293e], 1);
	KUNIT_EXPECT_EQ(test, main10[0x5909], 0);
	KUNIT_EXPECT_EQ(test, main10[0x590a], 1);
	KUNIT_EXPECT_EQ(test, main10[0x2962], 0);
	KUNIT_EXPECT_EQ(test, main10[0x592e], 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(main10 + 0x5ad8), 2);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(main10 + 0x5adc), 2);

	for (size_t off = 0; off < len; off += 4) {
		bool expected = false;

		for (unsigned int i = 0; i < ARRAY_SIZE(changed); i++)
			expected |= off == changed[i];
		if (!expected)
			KUNIT_EXPECT_EQ_MSG(test, get_unaligned_le32(main8 + off),
					    get_unaligned_le32(main10 + off),
					    "word +%#zx", off);
	}
}

/* One 1080p HEVC Start or first Process image, with speed mode @speed. */
static void ave_speed_image(struct kunit *test, struct ave_ctx *ctx,
			    enum ave_image image, bool bframes, bool main10,
			    bool speed, u8 *cmd, size_t len)
{
	ave_test_session(ctx->sess, AVE_CODEC_HEVC, 1920, 1080, bframes);
	if (main10) {
		ctx->sess->p.min_qp = -12;
		ctx->sess->p.profile = 2;
		ctx->sess->p.bit_depth = 10;
	}
	ctx->sess->p.speed = speed;
	memset(cmd, 0, len);
	if (image == AVE_IMAGE_START)
		KUNIT_ASSERT_EQ(test, ave_build_start(ctx, cmd, len), 0);
	else
		KUNIT_ASSERT_EQ(test, ave_build_process(ctx, cmd, len, 0,
							0x8f0000000ULL, 8000000,
							false, false), 0);
	ave_stamp_command(ctx->ave->layout->abi, cmd, 1, 2);
}

/*
 * Speed over quality on 9003: HEVC Start changes only the throughput mode
 * (+0x320, 3 -> 5); Process stays the same, with and without B-frames, for
 * Main and Main 10.
 */
static void ave_9003_speed(struct kunit *test)
{
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_ctx *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct ave_session *sess = kunit_kzalloc(test, sizeof(*sess), GFP_KERNEL);
	u8 *quality = kunit_kzalloc(test, 0x14000, GFP_KERNEL);
	u8 *speed = kunit_kzalloc(test, 0x14000, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, ave);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	KUNIT_ASSERT_NOT_NULL(test, sess);
	KUNIT_ASSERT_NOT_NULL(test, quality);
	KUNIT_ASSERT_NOT_NULL(test, speed);
	ave->layout = &ave_layout_9003_78_0;
	ctx->ave = ave;
	ctx->sess = sess;

	for (unsigned int i = 0; i < 8; i++) {
		enum ave_image image = i & 1 ? AVE_IMAGE_PROCESS : AVE_IMAGE_START;
		bool bframes = i & 2, main10 = i & 4;
		size_t len = ave->layout->bytes[AVE_CODEC_HEVC - 1][image];
		u32 changed = image == AVE_IMAGE_START ? 0x320 : 0;

		ave_speed_image(test, ctx, image, bframes, main10, false, quality,
				len);
		ave_speed_image(test, ctx, image, bframes, main10, true, speed, len);
		if (changed) {
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(quality + changed), 3);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(speed + changed), 5);
		}
		for (size_t off = 0; off < len; off += 4) {
			if (changed && off == changed)
				continue;
			KUNIT_EXPECT_EQ_MSG(test, get_unaligned_le32(quality + off),
					    get_unaligned_le32(speed + off),
					    "case %u word +%#zx", i, off);
		}
	}
}

/* Constant quality uses the measured macOS table: anchors and shape. */
static void ave_cq_table(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ave_cq_initial_qp(0), 48);
	KUNIT_EXPECT_EQ(test, ave_cq_initial_qp(30), 40);
	KUNIT_EXPECT_EQ(test, ave_cq_initial_qp(50), 33);
	KUNIT_EXPECT_EQ(test, ave_cq_initial_qp(60), 28);
	KUNIT_EXPECT_EQ(test, ave_cq_initial_qp(90), 10);
	KUNIT_EXPECT_EQ(test, ave_cq_initial_qp(100), 0);
	KUNIT_EXPECT_EQ(test, ave_cq_initial_qp(1000), 0);
	for (unsigned int q = 1; q <= 100; q++)
		KUNIT_EXPECT_LE(test, ave_cq_initial_qp(q),
				ave_cq_initial_qp(q - 1));
}

static void ave_6070_images(struct kunit *test)
{
	const struct ave_fw_abi *abi = &ave_abi_6070;
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_ctx *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct ave_session *sess =
		kunit_kzalloc(test, sizeof(*sess), GFP_KERNEL);
	u8 actual[0x70] = {}, expected[0x70] = {};

	KUNIT_ASSERT_NOT_NULL(test, ave);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	KUNIT_ASSERT_NOT_NULL(test, sess);
	ave->layout = &ave_layout_6070_11_1;
	ave->extra0_dma = 0x876540000ULL;
	ctx->ave = ave;
	ctx->sess = sess;
	abi->write_config(ave, actual);
	ave_stamp_command(abi, actual, 3000000, 0x123456);
	put_unaligned_le16(1, expected);
	put_unaligned_le32(U32_MAX, expected + 0x1c);
	put_unaligned_le64(3000000, expected + 0x28);
	put_unaligned_le64(0x123456, expected + 0x30);
	expected[0x41] = 1;
	put_unaligned_le64(ave->extra0_dma, expected + 0x48);
	KUNIT_EXPECT_MEMEQ(test, actual, expected, sizeof(actual));
	for (unsigned int codec = AVE_CODEC_H264; codec <= AVE_CODEC_HEVC;
	     codec++) {
		static const enum ave_opcode ops[] = { AVE_OP_OPEN,
						       AVE_OP_CLOSE,
						       AVE_OP_HALT };
		static const u16 wire[] = { 2, 12, 14 };
		static const u32 semantic[] = { 3, 4, U32_MAX };

		for (unsigned int i = 0; i < ARRAY_SIZE(ops); i++) {
			memset(actual, 0, sizeof(actual));
			memset(expected, 0, sizeof(expected));
			sess->p.codec = codec;
			sess->client = 10;
			sess->token = 300;
			if (ops[i] == AVE_OP_HALT)
				ave_write_global(ave, actual, ops[i]);
			else
				ave_write_header(ctx, actual, ops[i], ops[i],
						 0xc8, 0x1e0);
			ave_stamp_command(abi, actual, 3000000, 0x123456);
			put_unaligned_le16(wire[i], expected);
			if (ops[i] != AVE_OP_HALT) {
				put_unaligned_le64(400, expected + 0x08);
				put_unaligned_le32(10, expected + 0x10);
				put_unaligned_le32(codec - 1, expected + 0x18);
				put_unaligned_le32(0xc8, expected + 0x20);
			}
			put_unaligned_le32(semantic[i], expected + 0x1c);
			put_unaligned_le64(3000000, expected + 0x28);
			put_unaligned_le64(0x123456, expected + 0x30);
			KUNIT_EXPECT_MEMEQ(test, actual, expected,
					   abi->scalar_bytes[ops[i]]);
		}
	}
	KUNIT_EXPECT_EQ(test, abi->wire_id[0][AVE_OP_PRIORITY], 0U);
	KUNIT_EXPECT_EQ(test, abi->scalar_bytes[AVE_OP_RESET], 0U);
	KUNIT_EXPECT_EQ(test, abi->reply_id[AVE_OP_HALT], 0U);
}

static void ave_6070_replies(struct kunit *test)
{
	const struct ave_fw_abi *abi = &ave_abi_6070;
	struct ave_reply reply;
	u8 msg[0x48] = {};

	for (unsigned int op = 1; op < AVE_OP_COUNT; op++) {
		u32 len = op == AVE_OP_PROCESS ? 0x48 : 0x40;

		if (!abi->reply_id[op])
			continue;
		put_unaligned_le16(abi->reply_id[op], msg);
		put_unaligned_le32(10, msg + 0x10);
		put_unaligned_le32(abi->semantic[op], msg + 0x1c);
		put_unaligned_le32(0x00ee0000, msg + 0x38);
		put_unaligned_le32(3, msg + 0x40);
		KUNIT_ASSERT_EQ(test, abi->parse_reply(abi, msg, len, &reply),
				0);
		KUNIT_EXPECT_EQ(test, reply.op, (enum ave_opcode)op);
		KUNIT_EXPECT_EQ(test, reply.client, 10ULL);
		KUNIT_EXPECT_EQ(test, reply.subtype, abi->semantic[op]);
		KUNIT_EXPECT_EQ(test, reply.status, 0U);
		KUNIT_EXPECT_EQ(test, reply.completion_index,
				op == AVE_OP_PROCESS ? 3U : 0U);
		KUNIT_EXPECT_EQ(test,
				abi->parse_reply(abi, msg, len - 1, &reply),
				-EPROTO);
		KUNIT_EXPECT_EQ(test,
				abi->parse_reply(abi, msg, len + 1, &reply),
				-EPROTO);
		put_unaligned_le32(0, msg + 0x38);
		KUNIT_ASSERT_EQ(test, abi->parse_reply(abi, msg, len, &reply),
				0);
		KUNIT_EXPECT_NE(test, reply.status, 0U);
	}
	put_unaligned_le16(2, msg); /* Request IDs are not completion IDs. */
	KUNIT_EXPECT_EQ(test, abi->parse_reply(abi, msg, 0x40, &reply),
			-EPROTO);
}

static void ave_6070_frame_result(struct kunit *test)
{
	struct ave_frame_result result;
	u8 *meta = kunit_kzalloc(test, 0x400, GFP_KERNEL);
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, meta);
	/* This is the macroblock count, not the coded byte count. */
	put_unaligned_le32(0x1fe0, meta);
	put_unaligned_le32(0xb0, meta + 0x98);
	put_unaligned_le32(7, meta + 0x10c);
	put_unaligned_le32(3, meta + 0x110);
	put_unaligned_le32(0x2040, meta + 0x180);
	meta[0x38c] = 3;
	put_unaligned_le32(0x31, meta + 0x398);

	ret = ave_sess_decode_result(&ave_layout_6070_11_1, AVE_CODEC_H264,
				     meta, 0x400, &result);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, result.coded_bytes, 0x203dU);
	KUNIT_EXPECT_EQ(test, result.parameter_set_bytes, 22U);
	KUNIT_EXPECT_EQ(test, result.slice_header_bytes, 0U);
	KUNIT_EXPECT_EQ(test, result.ordinal, 7U);
	KUNIT_EXPECT_EQ(test, result.frame_type, 3U);

	ret = ave_sess_decode_result(&ave_layout_6070_11_1, AVE_CODEC_HEVC,
				     meta, 0x400, &result);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, result.slice_header_bytes, 0x31U);
	meta[0x38c] = 0xff;
	put_unaligned_le32(4, meta + 0x180);
	ret = ave_sess_decode_result(&ave_layout_6070_11_1, AVE_CODEC_H264,
				     meta, 0x400, &result);
	KUNIT_EXPECT_EQ(test, ret, -EPROTO);
	ret = ave_sess_decode_result(&ave_layout_6070_11_1, AVE_CODEC_H264,
				     meta, 3, &result);
	KUNIT_EXPECT_EQ(test, ret, -EPROTO);

	memset(meta, 0, 0x400);
	put_unaligned_le32(1234, meta + 0x120);
	put_unaligned_le32(1234, meta + 0x12c);
	put_unaligned_le32(19, meta + 0x128);
	ret = ave_sess_decode_result(&ave_layout_9003_78_0, AVE_CODEC_HEVC,
				     meta, 0x400, &result);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, result.coded_bytes, 1234U);
	KUNIT_EXPECT_EQ(test, result.parameter_set_bytes, 0U);
	KUNIT_EXPECT_EQ(test, result.slice_header_bytes, 19U);
}

static void ave_6070_split_surfaces(struct kunit *test)
{
	const struct ave_surface_layout bank = {
		.address = 0x4b8,
		.size = 0x558,
		.address_stride = 8,
		.size_stride = 4,
	};
	u8 *cmd = kunit_kzalloc(test, 0x600, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, cmd);
	for (unsigned int i = 0; i < 20; i++)
		ave_write_surface(&ave_layout_6070_11_1, cmd, &bank, i,
				  0x800000000ULL + i * SZ_16K,
				  (i + 1) * SZ_16K);
	for (unsigned int i = 0; i < 20; i++) {
		KUNIT_EXPECT_EQ(test, get_unaligned_le64(cmd + 0x4b8 + i * 8),
				0x800000000ULL + i * SZ_16K);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(cmd + 0x558 + i * 4),
				(i + 1) * SZ_16K);
	}
}

/* Named Start and Process fields come from the clean rev-2/P1 ledgers. */
static void ave_6070_start_process_fields(struct kunit *test)
{
	static const struct {
		u32 width, height;
	} sizes[] = { { 1920, 1080 }, { 3840, 2160 } };
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_ctx *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct ave_session *sess = kunit_kzalloc(test, sizeof(*sess), GFP_KERNEL);
	u8 *start = kunit_kzalloc(test, 0x32dc8, GFP_KERNEL);
	u8 *process = kunit_kzalloc(test, 0x6838, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, ave);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	KUNIT_ASSERT_NOT_NULL(test, sess);
	KUNIT_ASSERT_NOT_NULL(test, start);
	KUNIT_ASSERT_NOT_NULL(test, process);
	for (unsigned int codec = AVE_CODEC_H264; codec <= AVE_CODEC_HEVC;
	     codec++)
		for (unsigned int size = 0; size < ARRAY_SIZE(sizes); size++) {
			const struct ave_start_layout *sl;
			const struct ave_proc_layout *pl;
			size_t start_len, process_len;
			unsigned int slot;

			memset(sess, 0, sizeof(*sess));
			ave_test_6070_session(test, ave, ctx, sess, codec,
					      sizes[size].width, sizes[size].height);
			sl = &ave->layout->start[codec - 1];
			pl = &ave->layout->process[codec - 1];
			start_len = ave->layout->bytes[codec - 1][AVE_IMAGE_START];
			process_len = ave->layout->bytes[codec - 1][AVE_IMAGE_PROCESS];
			KUNIT_EXPECT_EQ(test, ave->layout->meta_bytes,
					AVE_6070_CODED_HEADER_BYTES);
			memset(start, 0, 0x32dc8);
			KUNIT_ASSERT_EQ(test,
					ave_build_start(ctx, start, start_len), 0);
			KUNIT_EXPECT_EQ(test, get_unaligned_le16(start),
					codec == AVE_CODEC_H264 ? 4 : 5);
			KUNIT_EXPECT_EQ(test, get_unaligned_le64(start + 0x40),
					sess->hdr.dma);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + 0x48),
					ave->layout->hdr_bytes);
			KUNIT_EXPECT_EQ(test, get_unaligned_le64(start + 0x50),
					sess->client_mem.dma);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + 0x58),
					AVE_6070_CLIENT_MEM_BYTES);
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le64(start + sl->param_sets.address),
					sess->param_sets.dma);
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le32(start + sl->param_sets.size),
					sess->param_sets.size);
			for (unsigned int i = 0; i < AVE_6070_LOWRES_RESULTS; i++)
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(start +
								     sl->lfs_result.address +
								     8 * i),
						sess->ref_d[i].dma);
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le32(start + sl->fps.denominator), 1U);
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le32(start + sl->fps.numerator), 30U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->rc_mode),
					sl->rc_mode_fixed_qp);
			KUNIT_EXPECT_EQ(test, sl->rc_mode_fixed_qp, 2U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->bitrate),
					0U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->min_qp), 0U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->max_qp), 51U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->init_qp),
					33U);
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le32(start + sl->init_qp + 4), 33U);
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le32(start + sl->init_qp + 8), 33U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->coded_w),
					sess->p.cw);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->coded_h),
					sess->p.ch);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->max_b), 1U);
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le32(start + sl->idr_interval), 30U);
			KUNIT_EXPECT_EQ(test, start[sl->ref_compression], 1U);
			if (codec == AVE_CODEC_H264) {
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start +
								     sl->h264_slice_map_count), 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start +
								     sl->h264_slice_map_end),
						sess->p.height);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_sve_count), 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start +
								     sl->h264_num_views[0]), 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start +
								     sl->h264_num_views[1]), 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start +
								     sl->h264_transform_mode), 0U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le16(start + sl->h264_scaling_4x4),
						(u16)AVE_AVC_FLAT_SCALING_WEIGHT);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le16(start + sl->h264_scaling_8x8),
						(u16)AVE_AVC_FLAT_SCALING_WEIGHT);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_profile), 77U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_level), 41U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_max_refs),
						2U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_chroma), 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start +
								     sl->h264_frame_num_bits), 0U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_poc_type), 0U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_poc_bits), 4U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_width_mbs),
						sess->p.cw / 16 - 1);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_height_mbs),
						sess->p.ch / 16 - 1);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start +
								     sl->h264_frame_only), 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_direct), 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_crop_bottom),
						(sess->p.ch - sess->p.height) / 2);
				KUNIT_EXPECT_EQ(test, start[sl->h264_fw_header], 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_cabac), 1U);
				KUNIT_EXPECT_EQ(test, start[sl->h264_deblock], 1U);
				KUNIT_EXPECT_EQ(test, start[sl->h264_8x8], 0U);
				KUNIT_EXPECT_EQ(test, start[sl->h264_pps_fw_header], 1U);
			} else {
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->hevc_profile_a), 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->hevc_refs), 4U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->hevc_width_ctb),
						DIV_ROUND_UP(sess->p.cw, 32));
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->hevc_conf_bottom),
						(sess->p.ch - sess->p.height) / 2);
			}

			if (codec == AVE_CODEC_H264 && !size) {
				sess->p.profile = 6;
				sess->p.high = true;
				memset(start, 0, 0x32dc8);
				KUNIT_ASSERT_EQ(test,
						ave_build_start(ctx, start, start_len), 0);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start +
								     sl->h264_transform_mode), 2U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_profile), 100U);
				KUNIT_EXPECT_EQ(test, start[sl->h264_8x8], 1U);
				sess->p.profile = 4;
				sess->p.high = false;
			}

			/* The live AVC case is low-delay P with two DPB slots. */
			if (codec == AVE_CODEC_H264 && !size) {
				sess->p.bframes = false;
				sess->p.reorder_off = true;
				sess->p.refs = 2;
				sess->p.slots = 3;
				memset(start, 0, 0x32dc8);
				KUNIT_ASSERT_EQ(test,
						ave_build_start(ctx, start, start_len), 0);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_max_refs), 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(start + sl->h264_poc_type), 2U);
				KUNIT_EXPECT_EQ(test, start[sl->ref_compression], 1U);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(start + sl->recon.address),
						sess->ref_a[0].dma);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(start + sl->recon.address + 8),
						sess->ref_a[0].dma + 0x313800);
			}

			slot = sess->p.slots - 1;
			sess->frames = 7;
			memset(process, 0, 0x6838);
			KUNIT_ASSERT_EQ(test,
					ave_build_process(ctx, process, process_len, slot,
							  0x8f0000000ULL, 4000000, true,
							  false),
					0);
			KUNIT_EXPECT_EQ(test, get_unaligned_le16(process),
					codec == AVE_CODEC_H264 ? 7 : 8);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x1c),
					0x15U + slot);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + pl->pic_size),
					0xf68U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le64(process + pl->y),
					0x8f0000000ULL);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + pl->y + 8),
					sess->p.stride);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + pl->y + 12),
					0U);
			for (unsigned int set = 0; set < AVE_NEIGHBOUR_SETS; set++) {
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(process +
								   pl->src_neighbor_info + 8 * set),
						sess->row_a[set].dma);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(process +
								   pl->src_neighbor_pixel +
								   8 * set),
						sess->row_b[set].dma);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(process +
								   pl->src_neighbor_data + 8 * set),
						sess->fixed[set].dma);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(process +
								   pl->src_neighbor_fw + 8 * set),
						sess->neighbour_fw[set].dma);
			}
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + pl->slot), slot);
			KUNIT_EXPECT_EQ(test, get_unaligned_le64(process + pl->coded),
					sess->slot[slot].coded.dma);
			KUNIT_EXPECT_EQ(test, get_unaligned_le64(process + pl->meta),
					sess->slot[slot].meta.dma);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + pl->coded_cap),
					sess->p.coded_cap);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + pl->bitrate), 0U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + pl->ordinal), 7U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le64(process + pl->pts),
					3360000ULL);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + pl->force_key),
					1U);
			KUNIT_EXPECT_EQ(test, process[pl->first_process], 0U);
			if (codec == AVE_CODEC_H264) {
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(process + pl->avg_variance),
						U32_MAX);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(process + pl->static_area_mask),
						(u32)AVE_AVC_STATIC_AREA_MASK_ALL);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le32(process +
								     pl->static_area_threshold),
						100U);
			}
			if (codec == AVE_CODEC_H264) {
				struct ave_6070_avc_recon_layout recon =
					ave_6070_avc_recon_layout(sess->p.width, sess->p.height);
				unsigned int cur_slot = sess->frames % sess->p.refs;

				if (!sess->p.bframes) {
					KUNIT_EXPECT_EQ(test,
							get_unaligned_le64(process + pl->frame_buf),
							sess->ref_c[cur_slot].dma);
					KUNIT_EXPECT_EQ(test,
							get_unaligned_le64(process +
									   pl->current_recon.luma_data),
							sess->ref_a[cur_slot].dma + recon.luma_data_offset);
					KUNIT_EXPECT_EQ(test,
							get_unaligned_le64(process +
									   pl->current_recon.chroma_data),
							sess->ref_a[cur_slot].dma +
								recon.chroma_data_offset);
					KUNIT_EXPECT_EQ(test,
							get_unaligned_le64(process +
									   pl->list0_recon.luma_data),
							0ULL);
					KUNIT_EXPECT_EQ(test,
							get_unaligned_le64(process + pl->list0_lowres),
							0ULL);
					KUNIT_EXPECT_EQ(test,
							get_unaligned_le64(process + pl->current_colocated),
							sess->ref_b[cur_slot].dma);
				}
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(process + pl->lrme_stats),
						sess->slot[slot].aux.dma);
				KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x40),
						0x984U);
				KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x44), 1U);
				KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x50), 2U);
				KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x64), 1U);
				KUNIT_EXPECT_EQ(test, process[0x7c], 1U);
				KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x9c0),
						U32_MAX);
			} else {
				KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x40),
						0x5570U);
				KUNIT_EXPECT_EQ(test, process[0x4c], 1U);
				KUNIT_EXPECT_EQ(test, process[0x1ea], 1U);
				KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x414), 3U);
				KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x58c),
						U32_MAX);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(process + pl->shdr_table),
						sess->slot[slot].aux.dma);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(process + pl->shdr_table +
									 255 * 8),
						sess->slot[slot].aux.dma + 255 * 0x400);
				KUNIT_EXPECT_EQ(test, process[0x6524],
						sess->codec_state[0x5570]);
			}

			/* 6070 carries a byte flag on the first Process after Start. */
			sess->frames = 0;
			memset(process, 0, 0x6838);
			KUNIT_ASSERT_EQ(test,
					ave_build_process(ctx, process, process_len, 0,
							  0x8f0000000ULL, 4000000, false,
							  false),
					0);
			KUNIT_EXPECT_EQ(test, process[pl->first_process], 1U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + pl->ordinal), 0U);
			KUNIT_EXPECT_EQ(test, get_unaligned_le64(process + pl->pts), 0ULL);
			if (codec == AVE_CODEC_H264 && !sess->p.bframes) {
				struct ave_6070_avc_recon_layout recon =
					ave_6070_avc_recon_layout(sess->p.width, sess->p.height);

				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(process +
								   pl->current_recon.luma_data),
						sess->ref_a[0].dma + recon.luma_data_offset);
				KUNIT_EXPECT_EQ(test,
						get_unaligned_le64(process +
								   pl->list0_recon.luma_data),
						0ULL);
			}
		}
}

static void ave_6070_entropy_interleave(struct kunit *test)
{
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_ctx *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct ave_session *sess = kunit_kzalloc(test, sizeof(*sess), GFP_KERNEL);
	u8 *cmd = kunit_kzalloc(test, 0x10e10, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, ave);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	KUNIT_ASSERT_NOT_NULL(test, sess);
	KUNIT_ASSERT_NOT_NULL(test, cmd);
	ave_test_6070_session(test, ave, ctx, sess, AVE_CODEC_H264, 1920, 1080);
	KUNIT_ASSERT_EQ(test, ave_build_start(ctx, cmd, 0x10e10), 0);
	for (unsigned int tile = 0; tile < AVE_6070_AVC_ENTROPY_TILES; tile++)
		for (unsigned int set = 0; set < AVE_ENTROPY_SETS; set++) {
			unsigned int main = 2 * tile, aux = main + 1;

			KUNIT_EXPECT_EQ(test,
					get_unaligned_le64(cmd + 0xf830 + 0x20 * main + 8 * set),
					sess->tile[set][tile].dma);
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le32(cmd + 0xfa30 + 0x10 * main + 4 * set),
					sess->tile[set][tile].size);
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le64(cmd + 0xf830 + 0x20 * aux + 8 * set),
					sess->tile_aux[set][tile].dma);
		}
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le64(cmd + 0xf830 +
					     0x20 * (2 * AVE_6070_AVC_ENTROPY_TILES)),
			0ULL);
}

static void ave_6070_reordered_pictures(struct kunit *test)
{
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_ctx *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct ave_session *sess = kunit_kzalloc(test, sizeof(*sess), GFP_KERNEL);
	u8 *cmd = kunit_kzalloc(test, 0x1940, GFP_KERNEL);
	const struct ave_proc_layout *pl;
	struct ave_6070_avc_recon_layout recon;

	KUNIT_ASSERT_TRUE(test, ave && ctx && sess && cmd);
	ave_test_6070_session(test, ave, ctx, sess, AVE_CODEC_H264, 1920, 1080);
	pl = &ave->layout->process[AVE_CODEC_H264 - 1];
	recon = ave_6070_avc_recon_layout(1920, 1080);

	for (u32 frame = 0; frame < 5; frame++) {
		memset(cmd, 0, 0x1940);
		sess->frames = frame;
		KUNIT_ASSERT_EQ(test,
				ave_build_process(ctx, cmd, 0x1940, frame,
						  0x8f0000000ULL, 4000000, false,
						  false),
				0);
		KUNIT_EXPECT_EQ(test,
				get_unaligned_le64(cmd + pl->current_recon.luma_data),
				sess->ref_a[frame].dma + recon.luma_data_offset);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(cmd + pl->frame_type),
				frame ? frame & 1 ? 2U : 1U : 3U);
	}

	/* B1 writes slot 1 and reads the past/future anchors in slots 0/2. */
	sess->frames = 1;
	memset(cmd, 0, 0x1940);
	KUNIT_ASSERT_EQ(test,
			ave_build_process(ctx, cmd, 0x1940, 1, 0x8f0000000ULL,
					  4000000, false, false),
			0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(cmd + 0x44), 0U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(cmd + 0x48), 1U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(cmd + 0x5c), 2U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(cmd + 0x68), 1U);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le64(cmd + pl->list0_recon.luma_data),
			sess->ref_a[0].dma);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le64(cmd + pl->list1_recon.luma_data),
			sess->ref_a[2].dma);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(cmd + pl->list1_colocated),
			sess->ref_b[2].dma);

	/* P4 has two ordered past anchors: P2 then IDR0. */
	sess->frames = 4;
	memset(cmd, 0, 0x1940);
	KUNIT_ASSERT_EQ(test,
			ave_build_process(ctx, cmd, 0x1940, 4, 0x8f0000000ULL,
					  4000000, false, false),
			0);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le64(cmd + pl->list0_recon.luma_data),
			sess->ref_a[2].dma);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le64(cmd + pl->list0_recon_second.luma_data),
			sess->ref_a[0].dma);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(cmd + 0x84), 1U);

	/* A final unpaired display picture is promoted from B to P. */
	sess->frames = 1;
	memset(cmd, 0, 0x1940);
	KUNIT_ASSERT_EQ(test,
			ave_build_process(ctx, cmd, 0x1940, 1, 0x8f0000000ULL,
					  4000000, false, true),
			0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(cmd + pl->frame_type), 1U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(cmd + 0x50), 0U);
}

static void ave_6070_hevc_contract(struct kunit *test)
{
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_ctx *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct ave_session *sess = kunit_kzalloc(test, sizeof(*sess), GFP_KERNEL);
	u8 *start = kunit_kzalloc(test, 0x32dc8, GFP_KERNEL);
	u8 *process = kunit_kzalloc(test, 0x6838, GFP_KERNEL);
	const struct ave_start_layout *sl;
	const struct ave_proc_layout *pl;
	struct ave_6070_avc_recon_layout recon;

	KUNIT_ASSERT_TRUE(test, ave && ctx && sess && start && process);
	ave_test_6070_session(test, ave, ctx, sess, AVE_CODEC_HEVC, 1920, 1080);
	sl = &ave->layout->start[AVE_CODEC_HEVC - 1];
	pl = &ave->layout->process[AVE_CODEC_HEVC - 1];
	recon = ave_6070_recon_layout(1920, 1080, 8);
	KUNIT_ASSERT_EQ(test, ave_build_start(ctx, start, 0x32dc8), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->hevc_input_format),
			16U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->hevc_chroma_format),
			1U);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(start +
					   sl->hevc_log2_max_poc_lsb_minus4),
			4U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->hevc_num_views[0]),
			1U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->hevc_sve_count), 1U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(start + sl->multi_me), 1U);
	KUNIT_EXPECT_EQ(test, start[sl->hevc_vps_temporal_nesting], 1U);
	KUNIT_EXPECT_EQ(test, start[sl->hevc_sps_temporal_nesting], 1U);
	KUNIT_EXPECT_EQ(test, start[sl->hevc_sps_tmvp], 0U);
	KUNIT_EXPECT_EQ(test, start[sl->hevc_sps_fw_header], 1U);
	KUNIT_EXPECT_EQ(test, start[sl->hevc_pps_fw_header], 1U);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(start +
					   sl->hevc_pps_l0_default_active_minus1),
			0U);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(start +
					   sl->hevc_pps_l1_default_active_minus1),
			0U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->hevc_rps), 8U);
	KUNIT_EXPECT_EQ(test, start[sl->hevc_rps + 0x5a68], 0U);
	/* IbP set 2 is P {-2}; set 3 is B {-1, +1}. */
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le16(start + sl->hevc_rps + 4 +
					   2 * 0x164 + 0x38),
			1U);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(start + sl->hevc_rps + 4 +
					   2 * 0x164 + 0xe0),
			(u32)-2);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(start + sl->hevc_rps + 4 +
					   3 * 0x164 + 0xbc),
			1U);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(start + sl->hevc_rps + 4 +
					   3 * 0x164 + 0x120),
			1U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le64(start + sl->recon.address + 8),
			sess->ref_a[0].dma + recon.luma_metadata_offset);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le64(start + sl->hevc_transcoded.address),
			sess->hevc_transcoded[0].dma);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(start + sl->hevc_transcoded.size),
			sess->hevc_transcoded[0].size);

	sess->frames = 1;
	KUNIT_ASSERT_EQ(test,
			ave_build_process(ctx, process, 0x6838, 1,
					  0x8f0000000ULL, 4000000, false,
					  false),
			0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + pl->frame_type), 2U);
	KUNIT_EXPECT_EQ(test, process[pl->hevc_single_transcoder], 1U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x68), 1U);
	KUNIT_EXPECT_EQ(test, process[0x15d], 1U);
	KUNIT_EXPECT_EQ(test, process[0x15e], 1U);
	KUNIT_EXPECT_EQ(test, process[0x15f], 0U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x414), 3U);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(process + pl->hevc_use_explicit_rps), 0U);
	KUNIT_EXPECT_EQ(test, process[0x6524], 1U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x6528), 0U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(process + 0x652c + 0x30), 0U);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le64(process + pl->list1_recon.luma_data),
			sess->ref_a[2].dma);

	/* The P after a B also selects the firmware-indexed SPS table. */
	sess->frames = 2;
	memset(process, 0, 0x6838);
	KUNIT_ASSERT_EQ(test,
			ave_build_process(ctx, process, 0x6838, 2,
					  0x8f0000000ULL, 4000000, false,
					  false),
			0);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(process + pl->hevc_use_explicit_rps), 0U);
	KUNIT_EXPECT_EQ(test, process[0x6524], 1U);

	/* IDR syntax has no short-term RPS selector at all. */
	sess->frames = 0;
	memset(process, 0, 0x6838);
	KUNIT_ASSERT_EQ(test,
			ave_build_process(ctx, process, 0x6838, 0,
					  0x8f0000000ULL, 4000000, false,
					  false),
			0);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(process + pl->hevc_use_explicit_rps), 0U);
	KUNIT_EXPECT_EQ(test, process[pl->hevc_single_transcoder], 1U);
	KUNIT_EXPECT_EQ(test, process[0x6524], 0U);

	/* Low delay advertises four equivalent {-1} SPS sets. */
	sess->p.bframes = false;
	sess->p.reorder_off = true;
	sess->p.refs = 2;
	sess->p.slots = 3;
	memset(start, 0, 0x32dc8);
	KUNIT_ASSERT_EQ(test, ave_build_start(ctx, start, 0x32dc8), 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + sl->hevc_rps), 4U);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(start + sl->multi_me), 0U);
	for (u32 i = 0; i < 4; i++) {
		u32 entry = sl->hevc_rps + 4 + i * 0x164;

		KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + entry + 0x30), 1U);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + entry + 0xb8), 1U);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(start + entry + 0xe0),
				(u32)-1);
	}
	sess->frames = 1;
	memset(process, 0, 0x6838);
	KUNIT_ASSERT_EQ(test,
			ave_build_process(ctx, process, 0x6838, 1,
					  0x8f0000000ULL, 4000000, false,
					  false),
			0);
	KUNIT_EXPECT_EQ(test,
			get_unaligned_le32(process + pl->hevc_use_explicit_rps), 0U);
	KUNIT_EXPECT_EQ(test, process[0x6524], 1U);
}

static void ave_6070_alignment_refusal(struct kunit *test)
{
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_ctx *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct ave_session *sess = kunit_kzalloc(test, sizeof(*sess), GFP_KERNEL);
	u8 *cmd = kunit_kmalloc(test, 0x10e10, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, ave);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	KUNIT_ASSERT_NOT_NULL(test, sess);
	KUNIT_ASSERT_NOT_NULL(test, cmd);
	ave_test_6070_session(test, ave, ctx, sess, AVE_CODEC_H264, 1920, 1080);
	memset(cmd, 0xa5, 0x10e10);
	sess->row_a[2].dma += 4;
	KUNIT_EXPECT_EQ(test, ave_build_start(ctx, cmd, 0x10e10), -EINVAL);
	KUNIT_EXPECT_EQ(test, cmd[0], 0xa5);
	KUNIT_EXPECT_EQ(test, sess->token, 400ULL);
	sess->row_a[2].dma -= 4;
	sess->param_sets.dma = 0;
	memset(cmd, 0xa5, 0x10e10);
	KUNIT_EXPECT_EQ(test, ave_build_start(ctx, cmd, 0x10e10), -EINVAL);
	KUNIT_EXPECT_EQ(test, cmd[0], 0xa5);
	sess->param_sets.dma = 0x822000000ULL;
	sess->param_sets.size = 0;
	memset(cmd, 0xa5, 0x10e10);
	KUNIT_EXPECT_EQ(test, ave_build_start(ctx, cmd, 0x10e10), -EINVAL);
	KUNIT_EXPECT_EQ(test, cmd[0], 0xa5);
	sess->param_sets.size = 2 * SZ_16K;
	memset(cmd, 0xa5, 0x1940);
	KUNIT_EXPECT_EQ(test,
			ave_build_process(ctx, cmd, 0x1940, 0, 0x8f0000004ULL,
					  4000000, false, false),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, cmd[0], 0xa5);
}

static void ave_6070_gate(struct kunit *test)
{
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_ctx *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct ave_session *sess =
		kunit_kzalloc(test, sizeof(*sess), GFP_KERNEL);
	u8 cmd[0x70], before[0x70];

	KUNIT_ASSERT_NOT_NULL(test, ave);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	KUNIT_ASSERT_NOT_NULL(test, sess);
	ave->layout = &ave_layout_6070_11_1;
	ctx->ave = ave;
	ctx->sess = sess;
	sess->p.codec = AVE_CODEC_H264;
	memset(cmd, 0xa5, sizeof(cmd));
	memcpy(before, cmd, sizeof(cmd));
	KUNIT_EXPECT_PTR_EQ(test, ave_layout_for_crc(0x460c7a7c),
			    &ave_layout_6070_11_1);
	KUNIT_EXPECT_PTR_EQ(test, ave_layout_for_crc(0x25bfd039),
			    &ave_layout_6070_11_1);
	KUNIT_EXPECT_PTR_EQ(test, ave_layout_for_crc(0), NULL);
	/* H.264 low delay is qualified; a short command buffer is still refused. */
	KUNIT_EXPECT_TRUE(test, ave_layout_6070_11_1.encode_ready);
	KUNIT_EXPECT_LT(test, ave_build_start(ctx, cmd, sizeof(cmd)), 0);
	KUNIT_EXPECT_LT(test,
			ave_build_process(ctx, cmd, sizeof(cmd), 0, 0, 0, false, false),
			0);
	KUNIT_EXPECT_MEMEQ(test, cmd, before, sizeof(cmd));
	KUNIT_EXPECT_EQ(test, sess->token, 0ULL);
}

/* The channel table the T6000 6070 firmware publishes, in its order. */
static const struct {
	const char *name;
	u32 dir, doorbell, slots, offset;
} ave_6070_desc[] = {
	{ "TERMINAL", 2, 0, 512, 0x700 }, { "IO", 0, 1, 32, 0x8700 },
	{ "DEBUG", 0, 1, 8, 0x8f00 },	  { "BUF_H2T", 0, 2, 1, 0x9100 },
	{ "BUF_T2H", 1, 3, 1, 0x9140 },	  { "SHAREDMALLOC", 1, 3, 8, 0x9180 },
	{ "IO_T2H", 1, 3, 32, 0x9380 },
};

static void ave_6070_write_desc(struct ave_dev *ave, unsigned int i,
				const char *name)
{
	u8 *desc = ave->ipc.cpu + AVE_IPC_CHANNEL_OFF + i * 0x100;

	memset(desc, 0, 0x100);
	strscpy(desc, name, 0x40);
	put_unaligned_le32(ave_6070_desc[i].dir, desc + 0x40);
	put_unaligned_le32(ave_6070_desc[i].doorbell, desc + 0x44);
	put_unaligned_le32(ave_6070_desc[i].slots, desc + 0x48);
	put_unaligned_le64(ave->ipc_fw_base + AVE_IPC_CHANNEL_OFF +
				   ave_6070_desc[i].offset,
			   desc + 0x4c);
}

static void ave_6070_channels(struct kunit *test)
{
	const struct ave_fw_transport *t = ave_abi_6070.transport;
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	u32 hello[4] = { 7, 0x9bc0, 0x100, 0 };
	u8 *desc;

	KUNIT_ASSERT_NOT_NULL(test, ave);
	ave->layout = &ave_layout_6070_11_1;
	ave->ipc.cpu = kunit_kzalloc(test, SZ_64K, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ave->ipc.cpu);
	ave->ipc.size = t->ipc_bytes;
	ave->ipc_fw_base = 0x100000000ULL;
	ave->channel_bytes = hello[1];
	KUNIT_EXPECT_EQ(test, ave_ipc_check_hello(t, hello), 0);
	for (unsigned int word = 0; word < 3; word++) {
		hello[word]++;
		KUNIT_EXPECT_EQ(test, ave_ipc_check_hello(t, hello), -EPROTO);
		hello[word]--;
	}
	/* The T6000 firmware asks for a 768 KiB heap in the last word. */
	hello[3] = 0xc0000;
	KUNIT_EXPECT_EQ(test, ave_ipc_check_hello(t, hello), 0);
	hello[3] = 4 * SZ_1M + 1;
	KUNIT_EXPECT_EQ(test, ave_ipc_check_hello(t, hello), -EPROTO);
	hello[3] = 0;

	for (unsigned int i = 0; i < ARRAY_SIZE(ave_6070_desc); i++)
		ave_6070_write_desc(ave, i, ave_6070_desc[i].name);
	KUNIT_ASSERT_EQ(test, ave_ipc_parse_channels(ave), 0);
	KUNIT_EXPECT_EQ(test, ave->ring[AVE_RING_IO].slots, 32U);
	KUNIT_EXPECT_EQ(test, ave->ring[AVE_RING_IO].doorbell, 1U);
	KUNIT_EXPECT_EQ(test, ave->ring[AVE_RING_IO].offset,
			AVE_IPC_CHANNEL_OFF + 0x8700);
	/* Replies come on IO_T2H, not on SHAREDMALLOC with the same doorbell. */
	KUNIT_EXPECT_STREQ(test, ave->ring[AVE_RING_T2H].name, "IO_T2H");
	KUNIT_EXPECT_EQ(test, ave->ring[AVE_RING_T2H].slots, 32U);
	KUNIT_EXPECT_EQ(test, ave->ring[AVE_RING_T2H].doorbell, 3U);
	KUNIT_EXPECT_EQ(test, ave->ring[AVE_RING_T2H].offset,
			AVE_IPC_CHANNEL_OFF + 0x9380);
	KUNIT_EXPECT_EQ(test, ave->ring[AVE_RING_SMALLOC].slots, 8U);
	KUNIT_EXPECT_EQ(test, ave->ring[AVE_RING_SMALLOC].offset,
			AVE_IPC_CHANNEL_OFF + 0x9180);
	KUNIT_EXPECT_EQ(test, ave->ring[AVE_RING_LOG].slots, 512U);
	KUNIT_EXPECT_EQ(test, ave->ring[AVE_RING_LOG].offset,
			AVE_IPC_CHANNEL_OFF + 0x700);
	/* Every host-to-firmware slot starts with the host, used or not. */
	for (unsigned int i = 0; i < ARRAY_SIZE(ave_6070_desc); i++)
		for (unsigned int n = 0; n < ave_6070_desc[i].slots; n++)
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le64(ave->ipc.cpu + AVE_IPC_CHANNEL_OFF +
							   ave_6070_desc[i].offset +
							   n * AVE_RING_SLOT_BYTES),
					ave_6070_desc[i].dir ? 0ULL : 1ULL);

	/* The old reading: the reply ring at SHAREDMALLOC's place. */
	ave_6070_write_desc(ave, 5, "IO_T2H");
	ave_6070_write_desc(ave, 6, "SHAREDMALLOC");
	KUNIT_EXPECT_EQ(test, ave_ipc_parse_channels(ave), -EPROTO);
	ave_6070_write_desc(ave, 5, "SHAREDMALLOC");
	ave_6070_write_desc(ave, 6, "IO_T2H");
	KUNIT_EXPECT_EQ(test, ave_ipc_parse_channels(ave), 0);

	desc = ave->ipc.cpu + AVE_IPC_CHANNEL_OFF + 0x100;
	put_unaligned_le32(33, desc + 0x48);
	KUNIT_EXPECT_EQ(test, ave_ipc_parse_channels(ave), -EPROTO);
	put_unaligned_le32(32, desc + 0x48);
	/* SHAREDMALLOC must be a firmware-to-host ring. */
	desc = ave->ipc.cpu + AVE_IPC_CHANNEL_OFF + 5 * 0x100;
	put_unaligned_le32(0, desc + 0x40);
	KUNIT_EXPECT_EQ(test, ave_ipc_parse_channels(ave), -EPROTO);
	put_unaligned_le32(1, desc + 0x40);
	/* Two rings may not share slots. */
	put_unaligned_le64(ave->ipc_fw_base + AVE_IPC_CHANNEL_OFF + 0x9380,
			   desc + 0x4c);
	KUNIT_EXPECT_EQ(test, ave_ipc_parse_channels(ave), -EPROTO);
	ave_6070_write_desc(ave, 5, "SHAREDMALLOC");
	desc = ave->ipc.cpu + AVE_IPC_CHANNEL_OFF;
	put_unaligned_le64(ave->ipc_fw_base + ave->ipc.size, desc + 0x4c);
	KUNIT_EXPECT_EQ(test, ave_ipc_parse_channels(ave), -EPROTO);
}

/* Play the firmware: post one SHAREDMALLOC request in ring slot @i. */
static void ave_6070_smalloc_ask(struct ave_dev *ave, unsigned int i,
				 u64 addr, u32 bytes)
{
	__le64 *slot = ave->ipc.cpu + ave->ring[AVE_RING_SMALLOC].offset +
		       i * AVE_RING_SLOT_BYTES;

	slot[1] = cpu_to_le64(bytes);
	slot[2] = 0;
	slot[0] = cpu_to_le64(addr); /* bit 0 clear: the host's turn */
}

static void ave_6070_smalloc(struct kunit *test)
{
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);
	struct ave_fw_transport *t;
	struct ave_fw_layout *layout;
	struct ave_fw_abi *abi;
	__le64 *slot;
	u64 first, second, base;

	KUNIT_ASSERT_NOT_NULL(test, ave);
	/* The real window is 7 MiB; shrink the borrowed part for the test. */
	t = kunit_kmalloc(test, sizeof(*t), GFP_KERNEL);
	abi = kunit_kmalloc(test, sizeof(*abi), GFP_KERNEL);
	layout = kunit_kmalloc(test, sizeof(*layout), GFP_KERNEL);
	KUNIT_ASSERT_TRUE(test, t && abi && layout);
	*t = *ave_abi_6070.transport;
	*abi = ave_abi_6070;
	*layout = ave_layout_6070_11_1;
	t->smalloc_offset = 0x20000;
	t->smalloc_bytes = 0x30000;
	abi->transport = t;
	layout->abi = abi;
	ave->layout = layout;
	ave->dev = kunit_device_register(test, "apple-ave-test");
	KUNIT_ASSERT_FALSE(test, IS_ERR(ave->dev));
	ave->ipc.cpu = kunit_kzalloc(test, 0x50000, GFP_KERNEL);
	ave->mbox = (void __iomem __force *)kunit_kzalloc(test, SZ_4K,
							  GFP_KERNEL);
	KUNIT_ASSERT_TRUE(test, ave->dev && ave->ipc.cpu && ave->mbox);
	ave->ipc.size = 0x50000;
	ave->ipc_fw_base = 0xffffffff80000000ULL;
	ave->channel_bytes = 0x9bc0;
	base = ave->ipc_fw_base + t->smalloc_offset;
	for (unsigned int i = 0; i < ARRAY_SIZE(ave_6070_desc); i++)
		ave_6070_write_desc(ave, i, ave_6070_desc[i].name);
	KUNIT_ASSERT_EQ(test, ave_ipc_parse_channels(ave), 0);
	/* The firmware creates the ring with every slot its own (bit 0 set). */
	for (unsigned int i = 0; i < ave->ring[AVE_RING_SMALLOC].slots; i++)
		*(__le64 *)(ave->ipc.cpu + ave->ring[AVE_RING_SMALLOC].offset +
			    i * AVE_RING_SLOT_BYTES) = cpu_to_le64(1);

	KUNIT_EXPECT_EQ(test, ave_ipc_serve_smalloc(ave), 0);
	/* Two 64 KiB blocks, as the firmware asks before Config. */
	memset(ave->ipc.cpu + t->smalloc_offset, 0xa5, SZ_64K);
	ave_6070_smalloc_ask(ave, 0, 0, SZ_64K);
	ave_6070_smalloc_ask(ave, 1, 0, SZ_64K);
	KUNIT_EXPECT_EQ(test, ave_ipc_serve_smalloc(ave), 1);
	slot = ave->ipc.cpu + ave->ring[AVE_RING_SMALLOC].offset;
	first = le64_to_cpu(slot[0]);
	KUNIT_EXPECT_EQ(test, first & 1, 1ULL);
	first &= ~3ULL;
	KUNIT_EXPECT_EQ(test, first, base);
	KUNIT_EXPECT_EQ(test, le64_to_cpu(slot[1]), (u64)SZ_64K);
	KUNIT_EXPECT_EQ(test, *(u8 *)(ave->ipc.cpu + t->smalloc_offset), 0);
	slot += AVE_RING_SLOT_BYTES / sizeof(*slot);
	second = le64_to_cpu(slot[0]) & ~3ULL;
	KUNIT_EXPECT_EQ(test, second, base + SZ_64K);
	/* The doorbell of the SHAREDMALLOC ring was rung. */
	KUNIT_EXPECT_EQ(test, readl(ave->mbox + AVE_MBOX_DOORBELL), BIT(3));

	/* Giving the first back frees it for the next request. */
	ave_6070_smalloc_ask(ave, 2, first, 0);
	KUNIT_EXPECT_EQ(test, ave_ipc_serve_smalloc(ave), 1);
	slot = ave->ipc.cpu + ave->ring[AVE_RING_SMALLOC].offset +
	       2 * AVE_RING_SLOT_BYTES;
	KUNIT_EXPECT_EQ(test, le64_to_cpu(slot[0]) & ~3ULL, first);
	KUNIT_EXPECT_EQ(test, le64_to_cpu(slot[1]), 0ULL);
	ave_6070_smalloc_ask(ave, 3, 0, 100);
	KUNIT_EXPECT_EQ(test, ave_ipc_serve_smalloc(ave), 1);
	slot = ave->ipc.cpu + ave->ring[AVE_RING_SMALLOC].offset +
	       3 * AVE_RING_SLOT_BYTES;
	KUNIT_EXPECT_EQ(test, le64_to_cpu(slot[0]) & ~3ULL, first);
	KUNIT_EXPECT_EQ(test, le64_to_cpu(slot[1]), 100ULL);

	/* More than the pool holds is refused. */
	ave_6070_smalloc_ask(ave, 4, 0, 2 * SZ_64K);
	KUNIT_EXPECT_EQ(test, ave_ipc_serve_smalloc(ave), -ENOMEM);
}

static void ave_6070_variants(struct kunit *test)
{
	const struct ave_fw_transport *t = ave_abi_6070.transport;

	KUNIT_EXPECT_EQ(test, ave_t6000.boot_device_id, 14U);
	KUNIT_EXPECT_EQ(test, ave_t6000.device_type, 11U);
	KUNIT_EXPECT_EQ(test, ave_t6000.boot_arg_value, 0x400U);
	KUNIT_EXPECT_EQ(test, ave_t6000.timebase_multiplier, 1000000U);
	KUNIT_EXPECT_EQ(test, ave_t6000.text_crc, 0x460c7a7cU);
	KUNIT_EXPECT_EQ(test, ave_t6000.mbox_idle, 0x38U);
	KUNIT_EXPECT_EQ(test, ave_t8112.boot_device_id, 18U);
	KUNIT_EXPECT_EQ(test, ave_t8112.device_type, 15U);
	KUNIT_EXPECT_EQ(test, ave_t8112.boot_arg_value, 0x10000U);
	KUNIT_EXPECT_EQ(test, ave_t8112.timebase_multiplier, 1U);
	KUNIT_EXPECT_EQ(test, ave_t8112.text_crc, 0x25bfd039U);
	KUNIT_EXPECT_EQ(test, ave_t8112.mbox_idle, 0x38U);
	KUNIT_EXPECT_EQ(test, ave_t6000.regs->ctrl, 0x400044U);
	KUNIT_EXPECT_EQ(test, ave_t8112.regs->timebase, 0x178000U);
	KUNIT_EXPECT_GE(test, t->command_bytes, 0x32dc8U);
	KUNIT_EXPECT_LE(test, t->command_offset + t->command_bytes,
			t->scalar_offset);
	KUNIT_EXPECT_LE(test,
			t->scalar_offset +
				AVE_IPC_SCALAR_BUFS * AVE_IPC_SCALAR_BYTES,
			t->process_offset);
	KUNIT_EXPECT_LE(
		test, t->process_offset + AVE_IPC_PROC_BUFS * t->process_bytes,
		t->smalloc_offset);
	KUNIT_EXPECT_LE(test, t->smalloc_offset + t->smalloc_bytes,
			t->ipc_bytes);
	KUNIT_EXPECT_LE(test, t->smalloc_bytes,
			AVE_SMALLOC_UNITS * AVE_SMALLOC_GRANULE);
	KUNIT_EXPECT_TRUE(test,
			  IS_ALIGNED(t->smalloc_offset, AVE_SMALLOC_GRANULE));
}

/* The loader's TEXT->DATA link cell does not change the image identity. */
static void ave_6070_text_id(struct kunit *test)
{
	const u32 off = ave_t8112.text_link_cell, size = off + 0x100;
	u8 *text = kunit_kzalloc(test, size, GFP_KERNEL);
	u32 image, crc;

	KUNIT_ASSERT_NOT_NULL(test, text);
	KUNIT_EXPECT_EQ(test, off, 0x423cU);
	KUNIT_EXPECT_EQ(test, ave_t6000.text_link_cell, 0x423cU);
	for (u32 i = 0; i < size; i++)
		text[i] = i * 7 + 3;
	memset(text + off, 0, 8);
	image = crc32_le(~0U, text, size) ^ ~0U;

	KUNIT_EXPECT_EQ(test, ave_fw_text_id_crc(text, size, off, 0xd0000ULL, &crc), 0);
	KUNIT_EXPECT_EQ(test, crc, image);
	put_unaligned_le64(0x8000d0000ULL, text + off);
	KUNIT_EXPECT_NE(test, crc32_le(~0U, text, size) ^ ~0U, image);
	crc = 0;
	KUNIT_EXPECT_EQ(test, ave_fw_text_id_crc(text, size, off, 0xd0000ULL, &crc), 0);
	KUNIT_EXPECT_EQ(test, crc, image);
	/* T8112 hands the firmware over at the DART vm-base. */
	crc = 0;
	KUNIT_EXPECT_EQ(test, ave_fw_text_id_crc(text, size, off, 0x8000d0000ULL, &crc), 0);
	KUNIT_EXPECT_EQ(test, crc, image);
	/* Any other address in the cell is a different TEXT. */
	put_unaligned_le64(0x8000d4000ULL, text + off);
	KUNIT_EXPECT_EQ(test, ave_fw_text_id_crc(text, size, off, 0xd0000ULL, &crc), -ENODEV);
	KUNIT_EXPECT_EQ(test, ave_fw_text_id_crc(text, size, size - 4, 0, &crc), -EINVAL);
}

/*
 * The M4 Max loader sets one TEXT word every boot: the identity hashes it as
 * zero, so two boots give one CRC, and it leaves the rest of TEXT counted.
 */
/* The 27.2 build differs from 26A428 only in its Process command sizes. */
static void ave_26b123_layout(struct kunit *test)
{
	const struct ave_fw_layout *a = &ave_layout_26a428;
	const struct ave_fw_layout *b = ave_layout_for_crc(0x99fc929a);

	KUNIT_ASSERT_PTR_EQ(test, b, &ave_layout_26b123);
	KUNIT_EXPECT_TRUE(test, b->encode_ready);
	KUNIT_EXPECT_EQ(test, b->kind, AVE_LAYOUT_26A428);
	KUNIT_EXPECT_PTR_EQ(test, b->abi, a->abi);
	for (unsigned int c = 0; c < 2; c++) {
		KUNIT_EXPECT_EQ(test, b->bytes[c][AVE_IMAGE_START],
				a->bytes[c][AVE_IMAGE_START]);
		KUNIT_EXPECT_EQ(test, b->bytes[c][AVE_IMAGE_PROCESS],
				a->bytes[c][AVE_IMAGE_PROCESS] + 0x10);
	}
	KUNIT_EXPECT_EQ(test, b->bytes[0][AVE_IMAGE_PROCESS], 0x6e08);
	KUNIT_EXPECT_EQ(test, b->bytes[1][AVE_IMAGE_PROCESS], 0x78c8);
	KUNIT_EXPECT_EQ(test, memcmp(b->start, a->start, sizeof(a->start)), 0);
	KUNIT_EXPECT_EQ(test, memcmp(b->process, a->process, sizeof(a->process)), 0);
	KUNIT_EXPECT_EQ(test, memcmp(b->result, a->result, sizeof(a->result)), 0);
	KUNIT_EXPECT_EQ(test, b->hdr_bytes, a->hdr_bytes);
	KUNIT_EXPECT_EQ(test, b->max_slots, a->max_slots);
	KUNIT_EXPECT_EQ(test, b->process_subtype, a->process_subtype);
	/* Both counted; only the 27.2 build runs sessions without B-frames and at any rate. */
	KUNIT_EXPECT_TRUE(test, a->counted && b->counted);
	KUNIT_EXPECT_FALSE(test, a->any_gop);
	KUNIT_EXPECT_TRUE(test, b->any_gop);
	KUNIT_EXPECT_PTR_EQ(test, ave_layout_for_crc(AVE_FW_TEXT_CRC_T8122_26B123), b);
}

static void ave_t6041_text_boot_word(struct kunit *test)
{
	const u32 off = ave_t6041.text_boot_word, size = off + 0x100;
	u8 *text = kunit_kzalloc(test, size, GFP_KERNEL);
	u32 image, a, b;

	KUNIT_ASSERT_NOT_NULL(test, text);
	KUNIT_EXPECT_EQ(test, off, 0x4224U);
	KUNIT_EXPECT_EQ(test, ave_t6041.fw_data_size_alt, 0x130000U);
	KUNIT_EXPECT_EQ(test, ave_t6041.text_link_cell, 0U);
	/* The M4 27.2 build: the same word, set per boot by some loaders. */
	KUNIT_EXPECT_EQ(test, ave_t8132.text_boot_word, off);
	KUNIT_EXPECT_EQ(test, ave_t8132.text_crc, AVE_FW_TEXT_CRC_T8132_26B123);
	KUNIT_EXPECT_EQ(test, ave_t8132.text_link_cell, 0U);
	for (u32 i = 0; i < size; i++)
		text[i] = i * 7 + 3;
	memset(text + off, 0, 4);
	image = crc32_le(~0U, text, size) ^ ~0U;

	put_unaligned_le32(0x00dd0000, text + off);
	KUNIT_EXPECT_EQ(test, ave_fw_text_boot_crc(text, size, off, &a), 0);
	put_unaligned_le32(0x007d0000, text + off);
	KUNIT_EXPECT_EQ(test, ave_fw_text_boot_crc(text, size, off, &b), 0);
	KUNIT_EXPECT_EQ(test, a, image);
	KUNIT_EXPECT_EQ(test, b, image);
	/* A change anywhere else is a different TEXT. */
	text[off + 4] ^= 1;
	KUNIT_EXPECT_EQ(test, ave_fw_text_boot_crc(text, size, off, &a), 0);
	KUNIT_EXPECT_NE(test, a, image);
	KUNIT_EXPECT_EQ(test, ave_fw_text_boot_crc(text, size, size - 2, &a), -EINVAL);
}

/*
 * The M3 generation binds probe-only: no build is pinned, the coprocessor
 * is never started, and nothing is encoded, whatever layout a TEXT CRC
 * might select.
 */
static void ave_m3_probe_only(struct kunit *test)
{
	struct ave_dev *ave = kunit_kzalloc(test, sizeof(*ave), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, ave);
	KUNIT_EXPECT_FALSE(test, ave_m3.boot_qualified);
	KUNIT_EXPECT_NULL(test, ave_m3.regs);
	KUNIT_EXPECT_EQ(test, ave_m3.fw_text_size, 0U);
	KUNIT_EXPECT_EQ(test, ave_m3.build_count, 0U);
	KUNIT_EXPECT_EQ(test, ave_m3.boot_device_id, 0U);
	KUNIT_EXPECT_EQ(test, ave_m3.device_type, 0U);

	ave->variant = &ave_m3;
	ave->text_crc = 0x25bfd039;
	ave->layout = ave_layout_for_crc(ave->text_crc);
	KUNIT_ASSERT_NOT_NULL(test, ave->layout);
	KUNIT_EXPECT_EQ(test, ave_fw_check_boot(ave), -EOPNOTSUPP);
}

static struct kunit_case ave_abi_cases[] = {
	KUNIT_CASE(ave_t6041_text_boot_word),
	KUNIT_CASE(ave_9003_byte_equality), KUNIT_CASE(ave_9003_hevc_main10),
	KUNIT_CASE(ave_9003_speed),	    KUNIT_CASE(ave_cq_table),
	KUNIT_CASE(ave_6070_images),
	KUNIT_CASE(ave_6070_replies),	    KUNIT_CASE(ave_6070_frame_result),
	KUNIT_CASE(ave_6070_split_surfaces),
	KUNIT_CASE(ave_6070_start_process_fields),
	KUNIT_CASE(ave_6070_reordered_pictures),
	KUNIT_CASE(ave_6070_hevc_contract),
	KUNIT_CASE(ave_6070_entropy_interleave),
	KUNIT_CASE(ave_6070_alignment_refusal),
	KUNIT_CASE(ave_6070_gate),	    KUNIT_CASE(ave_6070_channels),
	KUNIT_CASE(ave_6070_variants),	    KUNIT_CASE(ave_6070_text_id),
	KUNIT_CASE(ave_6070_smalloc),
	KUNIT_CASE(ave_m3_probe_only),
	KUNIT_CASE(ave_26b123_layout),
	{}
};

static struct kunit_suite ave_abi_suite = {
	.name = "apple-ave-abi",
	.test_cases = ave_abi_cases,
};
kunit_test_suite(ave_abi_suite);

MODULE_LICENSE("GPL");
