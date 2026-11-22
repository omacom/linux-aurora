// SPDX-License-Identifier: GPL-2.0-only
/*
 * AVE MCTF commands (see ave-mctf.h).
 *
 * The Start and Process images are data: the constant words macOS sets in
 * every MCTF session of the MacBook Neo (firmware TEXT CRC 69c54454, macOS
 * 26.6.2), the surfaces the firmware owns, and the few words that change
 * per frame. The session is the one the public VideoToolbox temporal noise
 * filter opens at 1920x1080 (one past and two next frames): the camera
 * daemon's session differs in four Start words and the Process header
 * argument, and with it the engine returns every picture unfiltered.
 *
 * The filter looks ahead, so each picture is sent twice: frame 0 goes to
 * slot 0 alone; for every later frame, slot 0 gets the previous picture
 * again and slot 1 the new one, then a Complete makes the firmware answer
 * both. Slot 1's output is the filtered new picture, slot 0's writes the
 * firmware's own copy and is not delivered. Input surfaces are
 * driver-owned, five used in turn; each slot has one output surface.
 *
 * The surfaces are Apple's lossless compressed 4:2:0 form (ave-cmp420.c).
 * The driver writes each NV12 input picture as raw tiles and reads the
 * output back to NV12 on the CPU; a block form the reader does not know is
 * counted and shown as a marker (ave-mctf-v4l2.c).
 *
 * Copyright The Aurora Silicon Contributors
 */
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include <media/videobuf2-dma-contig.h>

#include "ave.h"
#include "ave-regs.h"
#include "ave-mctf.h"

/* Start words other than the header and the surface addresses */
static const struct ave_mctf_word ave_mctf_start_25g83[] = {
	/* Session configuration */
	{ 0x50, 0x12c000 }, { 0x78, 0x200 }, { 0x94, 0x3c0 },
	{ 0x220, 0x1e }, { 0x338, 0x1 }, { 0x33c, 0xffffffff },
	{ 0x368, 0x780 }, { 0x36c, 0x438 }, { 0x370, 0x2 },
	/* Sizes and strides of the internal surfaces */
	{ 0x398, 0x1fe000 }, { 0x3a8, 0x20000 }, { 0x3b8, 0x1fe000 },
	{ 0x3c8, 0x20000 }, { 0x3d8, 0x1fe000 }, { 0x3e8, 0x20000 },
	{ 0x7d8, 0x3fc00 }, { 0x7f8, 0x3fc00 }, { 0x818, 0x3fc00 },
	{ 0x838, 0x3fc00 }, { 0x858, 0x3fc00 }, { 0xc18, 0xa4000 },
	{ 0xc28, 0xa4000 }, { 0xc38, 0xa4000 }, { 0xcb8, 0x4c000 },
	{ 0xcc8, 0x4c000 }, { 0xcd8, 0x4c000 }, { 0xd58, 0x2f8000 },
	{ 0xd68, 0x2f8000 }, { 0xd78, 0x2f8000 }, { 0xd88, 0x2f8000 },
	{ 0xd98, 0x2f8000 }, { 0xda8, 0x2f8000 }, { 0xdb8, 0x2f8000 },
	{ 0xdc8, 0x2f8000 }, { 0xf58, 0xc000 }, { 0xf68, 0xc000 },
	{ 0xf78, 0xc000 }, { 0xf88, 0xc000 }, { 0xf98, 0xc000 },
	{ 0xfa8, 0xc000 }, { 0xfb8, 0xc000 }, { 0xfc8, 0xc000 },
	{ 0x1138, 0x20000 }, { 0x1148, 0x20000 }, { 0x1158, 0x20000 },
	{ 0x1358, 0x78000 }, { 0x1398, 0xc000 }, { 0x13d8, 0x4000 },
	{ 0x1418, 0x23dc00 }, { 0x1428, 0x480 }, { 0x1438, 0x23dc00 },
	{ 0x1448, 0x480 }, { 0x1498, 0x23dc00 }, { 0x14a8, 0x480 },
	{ 0x14b8, 0x23dc00 }, { 0x14c8, 0x480 }, { 0x1e58, 0x3fc00 },
	{ 0x1e78, 0x3fc00 }, { 0x1e98, 0x3fc00 }, { 0x1eb8, 0x3fc00 },
	{ 0x1ed8, 0x3fc00 }, { 0x1fd8, 0xa4000 }, { 0x1fe8, 0xa4000 },
	{ 0x1ff8, 0xa4000 }, { 0x2078, 0x4c000 }, { 0x2088, 0x4c000 },
	{ 0x2098, 0x4c000 }, { 0x2128, 0x27d800 }, { 0x212c, 0xf000 },
	{ 0x2138, 0x20000 }, { 0x2148, 0x13ec00 }, { 0x214c, 0x7800 },
	{ 0x2158, 0x8000 }, { 0x2168, 0x27d800 }, { 0x216c, 0xf000 },
	{ 0x2178, 0x20000 }, { 0x2188, 0x13ec00 }, { 0x218c, 0x7800 },
	{ 0x2198, 0x8000 },
	/* Work configuration */
	{ 0x2580, 0x1000000 }, { 0x2584, 0x101 }, { 0x2594, 0x1 },
	{ 0x2598, 0x8 }, { 0x259c, 0x2 }, { 0x25a4, 0x101 },
	{ 0x25d4, 0x1 }, { 0x26e8, 0x3000000 }, { 0x26f8, 0x10 },
	{ 0x26fc, 0xffffffff }, { 0x2700, 0x2 }, { 0x2704, 0x1 },
	{ 0x2738, 0x10 }, { 0x5aac, 0x1 }, { 0x7558, 0x100 },
	{ 0xb774, 0x1 },
	/* Reference structure: delta POCs -1 .. -4 */
	{ 0xe120, 0x4 }, { 0xe154, 0x4 }, { 0xe17c, 0x1010101 },
	{ 0xe1dc, 0x4 }, { 0xe1e4, 0x1010101 }, { 0xe204, 0xffffffff },
	{ 0xe208, 0xfffffffe }, { 0xe20c, 0xfffffffd }, { 0xe210, 0xfffffffc },
	{ 0xe244, 0x1 }, { 0xe248, 0x2 }, { 0xe24c, 0x3 },
	{ 0xe250, 0x4 }, { 0xe284, 0x4 }, { 0xe2b8, 0x1 },
	{ 0xe2e0, 0x1 }, { 0xe340, 0x1 }, { 0xe348, 0x1 },
	{ 0xe368, 0xffffffff }, { 0xe3a8, 0x1 }, { 0xe3e8, 0x1 },
	{ 0xe41c, 0x2 }, { 0xe444, 0x101 }, { 0xe4a4, 0x2 },
	{ 0xe4ac, 0x101 }, { 0xe4cc, 0xffffffff }, { 0xe4d0, 0xfffffffe },
	{ 0xe50c, 0x1 }, { 0xe510, 0x2 }, { 0xe54c, 0x2 },
	{ 0xe580, 0x3 }, { 0xe5a8, 0x10101 }, { 0xe608, 0x3 },
	{ 0xe610, 0x10101 }, { 0xe630, 0xffffffff }, { 0xe634, 0xfffffffe },
	{ 0xe638, 0xfffffffd }, { 0xe670, 0x1 }, { 0xe674, 0x2 },
	{ 0xe678, 0x3 }, { 0xe6b0, 0x3 },
	/* Execution units */
	{ 0x13ef4, 0x1 }, { 0x13ef8, 0x1 }, { 0x13f00, 0x1 },
};

#define S(o, a)		{ .off = (o), .alloc = (a) }
#define ALIAS(o, of)	{ .off = (o), .alias = (of) }

/*
 * Firmware-owned surfaces, mapped at least as large as macOS spaces them
 * (some are written past their size word). Each alias repeats the address
 * of the surface before it with a zero size.
 */
static const struct ave_mctf_surface ave_mctf_surfaces_25g83[] = {
	S(0x48, 0x140000), S(0x390, 0x304000), S(0x3a0, 0x28000),
	S(0x3b0, 0x304000), S(0x3c0, 0x28000), S(0x3d0, 0x304000),
	S(0x3e0, 0x28000), S(0x7d0, 0x44000), ALIAS(0x7e0, 0x7d0),
	S(0x7f0, 0x44000), ALIAS(0x800, 0x7f0), S(0x810, 0x44000),
	ALIAS(0x820, 0x810), S(0x830, 0x44000), ALIAS(0x840, 0x830),
	S(0x850, 0x44000), ALIAS(0x860, 0x850), S(0xc10, 0xa8000),
	S(0xc20, 0xa8000), S(0xc30, 0xa8000), S(0xcb0, 0x50000),
	S(0xcc0, 0x50000), S(0xcd0, 0x50000), S(0xd50, 0x2fc000),
	S(0xd60, 0x2fc000), S(0xd70, 0x2fc000), S(0xd80, 0x2fc000),
	S(0xd90, 0x2fc000), S(0xda0, 0x2fc000), S(0xdb0, 0x2fc000),
	S(0xdc0, 0x2fc000), S(0xf50, 0x10000), S(0xf60, 0x10000),
	S(0xf70, 0x10000), S(0xf80, 0x10000), S(0xf90, 0x10000),
	S(0xfa0, 0x10000), S(0xfb0, 0x10000), S(0xfc0, 0x10000),
	S(0x1130, 0x24000), S(0x1140, 0x24000), S(0x1150, 0x24000),
	S(0x1350, 0x7c000), S(0x1390, 0x10000), S(0x13d0, 0x8000),
	S(0x1410, 0x23dc00), S(0x1420, 0x8000), S(0x1430, 0x23dc00),
	S(0x1440, 0x8000), S(0x1490, 0x23dc00), S(0x14a0, 0x8000),
	S(0x14b0, 0x23dc00), S(0x14c0, 0x8000), S(0x1e50, 0x44000),
	ALIAS(0x1e60, 0x1e50), S(0x1e70, 0x44000), ALIAS(0x1e80, 0x1e70),
	S(0x1e90, 0x44000), ALIAS(0x1ea0, 0x1e90), S(0x1eb0, 0x44000),
	ALIAS(0x1ec0, 0x1eb0), S(0x1ed0, 0x44000), ALIAS(0x1ee0, 0x1ed0),
	S(0x1fd0, 0xa8000), S(0x1fe0, 0xa8000), S(0x1ff0, 0xa8000),
	S(0x2070, 0x50000), S(0x2080, 0x50000), S(0x2090, 0x50000),
	S(0x2120, 0x27d800), S(0x2130, 0x20000), S(0x2140, 0x144000),
	S(0x2150, 0x8000), S(0x2160, 0x27d800), S(0x2170, 0x20000),
	S(0x2180, 0x144000), S(0x2190, 0x8000),
};

#undef S
#undef ALIAS

/* Process words other than the header, the surface addresses and the frame fields */
static const struct ave_mctf_word ave_mctf_process_25g83[] = {
	{ 0xb4c, 0x5 }, { 0xb50, 0x5 }, { 0xd7c, 0x1 },
	{ 0x6158, 0x1 }, { 0x74cc, 0xffffffff }, { 0x74dc, 0x10000 },
	{ 0xa308, 0x1fe000 }, { 0xa30c, 0xf000 }, { 0xa318, 0x20000 },
	{ 0xa324, 0x780 }, { 0xa328, 0x438 }, { 0xa334, 0xf000 },
	{ 0xa348, 0xff000 }, { 0xa34c, 0x7800 }, { 0xa358, 0x8000 },
	{ 0xa364, 0x3c0 }, { 0xa368, 0x21c }, { 0xa374, 0x7800 },
	{ 0xa3b8, 0x780 }, { 0xa3bc, 0x438 }, { 0xac98, 0x2f8000 },
	{ 0xad08, 0x5 }, { 0xad3c, 0x403e0000 }, { 0xad60, 0x1 },
	{ 0xad74, 0x3ff00000 }, { 0xad88, 0x1fe000 }, { 0xad8c, 0xf000 },
	{ 0xad98, 0x20000 }, { 0xada4, 0x780 }, { 0xada8, 0x438 },
	{ 0xadb4, 0xf000 }, { 0xadc8, 0xff000 }, { 0xadcc, 0x7800 },
	{ 0xadd8, 0x8000 }, { 0xade4, 0x3c0 }, { 0xade8, 0x21c },
	{ 0xadf4, 0x7800 },
};

#define AVE_MCTF_25G83(crc, ok)						\
	{								\
		.text_crc = (crc),					\
		.start_bytes = 0x13f28, .process_bytes = 0xb1c0,	\
		.unit = AVE_MCTF_WORK_TYPE, .arg1 = 0x3c0,		\
		.frame_arg0 = 0,					\
		.subtype = 0x15, .slots = AVE_MCTF_SLOTS,		\
		.start_words = ave_mctf_start_25g83,			\
		.n_start_words = ARRAY_SIZE(ave_mctf_start_25g83),	\
		.process_words = ave_mctf_process_25g83,		\
		.n_process_words = ARRAY_SIZE(ave_mctf_process_25g83),	\
		.surfaces = ave_mctf_surfaces_25g83,			\
		.n_surfaces = ARRAY_SIZE(ave_mctf_surfaces_25g83),	\
		/* the slot's output and statistics surfaces */		\
		.slot_refs = { { 0xac88, 0xd50, 0x10 },			\
			       { 0xac90, 0xf50, 0x10 } },		\
		.input = { 0xa300, 0xa310, 0xa340, 0xa350 },		\
		.output = { 0xad80, 0xad90, 0xadc0, 0xadd0 },		\
		.form = { .y_hdr = 0x1fe000, .c = 0x21e000,		\
			  .c_hdr = 0x31d000, .bytes = 0x325000 },	\
		.inputs = AVE_MCTF_INPUTS,				\
		.width = 0xa3b8, .height = 0xa3bc,			\
		.first_frame = 0x5d90, .first_value = 5,		\
		.next_value = 4,					\
		.slot = 0xac84, .frame = 0xacf8, .level = 0xad68,	\
		.strength = 0x26f8,					\
		.stream = 12,						\
		.qualified = (ok),					\
	}

static const struct ave_mctf_layout ave_mctf_layouts[] = {
	/* MacBook Neo (T8140), encoder firmware 9003.78.0: captured at 1080p */
	AVE_MCTF_25G83(AVE_FW_TEXT_CRC_9003_78_0, true),
	/* M4 Pro (T6040): same carrier sizes, no capture yet */
	AVE_MCTF_25G83(AVE_FW_TEXT_CRC_T6040_25G83, false),
	/* M4 Max (J575c): same build, no capture yet */
	AVE_MCTF_25G83(AVE_FW_TEXT_CRC_T6041_25G83, false),
};

const struct ave_mctf_layout *ave_mctf_layout_for_crc(u32 text_crc)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(ave_mctf_layouts); i++)
		if (ave_mctf_layouts[i].text_crc == text_crc)
			return &ave_mctf_layouts[i];
	return NULL;
}

const struct ave_mctf_layout *ave_mctf_qualified(u32 text_crc)
{
	const struct ave_mctf_layout *l = ave_mctf_layout_for_crc(text_crc);

	return l && l->qualified ? l : NULL;
}

static int ave_mctf_words(u8 *cmd, size_t len, const struct ave_mctf_word *w,
			  unsigned int n)
{
	for (unsigned int i = 0; i < n; i++) {
		if (w[i].off + 4 > len)
			return -EINVAL;
		put_unaligned_le32(w[i].val, cmd + w[i].off);
	}
	return 0;
}

/* Index of the Start surface whose address goes to @off, or -ENOENT */
static int ave_mctf_surface_index(const struct ave_mctf_layout *l, u32 off)
{
	for (unsigned int i = 0; i < l->n_surfaces; i++)
		if (l->surfaces[i].off == off)
			return l->surfaces[i].alloc ? i :
			       ave_mctf_surface_index(l, l->surfaces[i].alias);
	return -ENOENT;
}

/*
 * Start: the constant words and every surface address, from @surf (indexed
 * like the layout's surface table; alias rows are ignored). The caller
 * writes the header.
 */
int ave_mctf_build_start(const struct ave_mctf_layout *l, u8 *cmd, size_t len,
			 const dma_addr_t *surf, u32 strength)
{
	int ret;

	if (!l || !l->qualified)
		return -EOPNOTSUPP;
	if (len != l->start_bytes || l->n_surfaces > AVE_MCTF_BUFS ||
	    strength > AVE_MCTF_STRENGTH_MAX)
		return -EINVAL;
	memset(cmd, 0, len);
	ret = ave_mctf_words(cmd, len, l->start_words, l->n_start_words);
	if (ret)
		return ret;
	for (unsigned int i = 0; i < l->n_surfaces; i++) {
		int at = ave_mctf_surface_index(l, l->surfaces[i].off);

		if (at < 0 || l->surfaces[i].off + 8 > len || !surf[at])
			return -EINVAL;
		put_unaligned_le64(surf[at], cmd + l->surfaces[i].off);
	}
	put_unaligned_le32(strength, cmd + l->strength);
	return 0;
}

static void ave_mctf_put_surface(u8 *cmd, const struct ave_mctf_picture *p,
				 const struct ave_mctf_surface_form *form, u64 base)
{
	put_unaligned_le64(base, cmd + p->y);
	put_unaligned_le64(base + form->y_hdr, cmd + p->y_hdr);
	put_unaligned_le64(base + form->c, cmd + p->c);
	put_unaligned_le64(base + form->c_hdr, cmd + p->c_hdr);
}

/* Process for one frame; the caller writes the header (subtype base + slot). */
int ave_mctf_build_process(const struct ave_mctf_layout *l, u8 *cmd,
			   size_t len, const dma_addr_t *surf,
			   const struct ave_mctf_frame *f)
{
	int ret;

	if (!l || !l->qualified)
		return -EOPNOTSUPP;
	if (len != l->process_bytes || f->slot >= l->slots ||
	    f->level < AVE_MCTF_LEVEL_MIN || f->level > AVE_MCTF_LEVEL_MAX ||
	    f->width != AVE_MCTF_WIDTH ||
	    f->height != AVE_MCTF_HEIGHT || f->stride < f->width ||
	    f->stride % 64 || !f->in || !f->out)
		return -EINVAL;
	memset(cmd, 0, len);
	ret = ave_mctf_words(cmd, len, l->process_words, l->n_process_words);
	if (ret)
		return ret;
	ave_mctf_put_surface(cmd, &l->input, &l->form, f->in);
	ave_mctf_put_surface(cmd, &l->output, &l->form, f->out);
	put_unaligned_le32(f->width, cmd + l->width);
	put_unaligned_le32(f->height, cmd + l->height);
	put_unaligned_le32(f->frame ? l->next_value : l->first_value,
			   cmd + l->first_frame);
	put_unaligned_le32(f->slot, cmd + l->slot);
	for (unsigned int i = 0; i < ARRAY_SIZE(l->slot_refs); i++) {
		const struct ave_mctf_slot_ref *r = &l->slot_refs[i];
		int at = ave_mctf_surface_index(l, r->start + r->stride * f->slot);

		if (at < 0 || !surf[at])
			return -EINVAL;
		put_unaligned_le64(surf[at], cmd + r->off);
	}
	put_unaligned_le32(f->frame, cmd + l->frame);
	put_unaligned_le32(f->level, cmd + l->level);
	return 0;
}

/*
 * MCTF needs a qualified boot, a qualified MCTF layout for this build and
 * the layout's DART stream attached on every encoder DART.
 */
int ave_mctf_check_build(struct ave_dev *ave)
{
	const struct ave_mctf_layout *l;
	int ret = ave_fw_check_boot(ave);

	if (ret)
		return ret;
	l = ave_mctf_qualified(ave->text_crc);
	if (!l)
		return -EOPNOTSUPP;
	if (!ave->n_darts)
		goto no_stream;
	for (unsigned int d = 0; d < ave->n_darts; d++)
		if (l->stream >= 32 || !(ave->dart_sid_mask[d] & BIT(l->stream)))
			goto no_stream;
	return 0;

no_stream:
	dev_warn_once(ave->dev,
		      "MCTF disabled: DART stream %u is not attached on every encoder DART\n",
		      l->stream);
	return -EOPNOTSUPP;
}

int ave_mctf_cmd_start(struct ave_ctx *ctx)
{
	struct ave_dev *ave = ctx->ave;
	struct ave_session *sess = ctx->sess;
	const struct ave_mctf_layout *l = sess->mctf;
	size_t len;
	u8 *cmd;
	int ret;

	if (!l)
		return -EOPNOTSUPP;
	len = l->start_bytes;
	cmd = ave_ipc_cmd(ave, AVE_BUF_CMD, len);
	if (!cmd)
		return -EBUSY;
	ret = ave_mctf_build_start(l, cmd, len, sess->mctf_iova,
				   READ_ONCE(ctx->mctf_strength));
	if (ret)
		return ret;
	ave_write_header(ctx, cmd, AVE_OP_START, AVE_OP_START, AVE_SESSION_ARG0,
			 ave_session_setup_arg2(ctx));
	ave_debugfs_save_mctf(ave, AVE_IMAGE_START, cmd, len);
	dev_dbg(ave->dev, "MCTF Start: client %u token %llu\n", sess->client,
		sess->token);

	/* A failed submission may already have exposed the Start buffers. */
	sess->handed = true;
	ret = ave_ipc_submit(ave, AVE_BUF_CMD, len, AVE_CMD_SESSION_TIMEOUT_US,
			     true);
	if (ret)
		return ret;
	return ave_ipc_settle(ave, AVE_START_WAIT_MS);
}

/*
 * Fill input surface @n from @src: a lossless picture as it is, an NV12
 * picture written with every tile raw. The
 * conversion runs in cached memory; the DMA surfaces see only bulk copies.
 */
static int ave_mctf_put_picture(struct ave_ctx *ctx, struct vb2_v4l2_buffer *src,
				unsigned int n)
{
	struct ave_session *sess = ctx->sess;
	const struct ave_cmp420_geom *g = &sess->mctf_geom;
	const struct ave_params *p = &sess->p;
	size_t luma = (size_t)p->stride * p->height;
	u8 *to = sess->mctf_in[n].cpu;
	const u8 *from = vb2_plane_vaddr(&src->vb2_buf, 0);

	if (!to || !from || !sess->mctf_surf || !sess->mctf_lin)
		return -EFAULT;
	if (ctx->src_fmt.pixelformat == V4L2_PIX_FMT_APPLE_L8V0) {
		if (src->vb2_buf.planes[0].data_offset + g->bytes >
		    vb2_plane_size(&src->vb2_buf, 0))
			return -EINVAL;
		memcpy(to, from + src->vb2_buf.planes[0].data_offset, g->bytes);
		return 0;
	}
	if (src->vb2_buf.planes[0].data_offset + luma * 3 / 2 >
	    vb2_plane_size(&src->vb2_buf, 0))
		return -EINVAL;
	from += src->vb2_buf.planes[0].data_offset;
	memcpy(sess->mctf_lin, from, luma * 3 / 2);
	ave_cmp420_pack(g, sess->mctf_surf, sess->mctf_lin, sess->mctf_lin + luma,
			p->stride);
	memcpy(to, sess->mctf_surf, g->bytes);
	return 0;
}

/*
 * Filter @src into @dst in @slot. With @same_input, the picture of the last
 * command goes again (its surface still holds it). A NULL @dst sends the
 * companion command whose output is not delivered. Caller holds hw_lock.
 */
int ave_mctf_cmd_process(struct ave_ctx *ctx, unsigned int slot,
			 struct vb2_v4l2_buffer *src,
			 struct vb2_v4l2_buffer *dst, bool same_input)
{
	struct ave_dev *ave = ctx->ave;
	struct ave_session *sess = ctx->sess;
	const struct ave_mctf_layout *l = sess->mctf;
	unsigned int buf = AVE_BUF_PROC(sess->index * AVE_MAX_SLOTS + slot);
	struct ave_slot *s = &sess->slot[slot];
	struct ave_mctf_frame f = {
		.stride = sess->p.stride,
		.width = sess->p.width,
		.height = sess->p.height,
		.slot = slot,
		.frame = sess->frames,
		.level = READ_ONCE(ctx->mctf_level),
	};
	u8 *cmd;
	int ret;

	if (!l || slot >= sess->p.slots)
		return -EINVAL;
	if (same_input) {
		s->mctf_input = sess->mctf_last_input;
	} else {
		ret = ave_mctf_put_picture(ctx, src, sess->frames % AVE_MCTF_INPUTS);
		if (ret)
			return ret;
		s->mctf_input = sess->frames % AVE_MCTF_INPUTS;
		sess->mctf_last_input = s->mctf_input;
	}
	f.in = sess->mctf_in[s->mctf_input].dma;
	/*
	 * A lossless CAPTURE buffer takes slot 1's output directly: the engine
	 * keeps no reference to it (slot 0's surface is the firmware's own).
	 */
	s->mctf_direct = dst && slot == 1 &&
			  ctx->dst_fmt.pixelformat == V4L2_PIX_FMT_APPLE_L8V0 &&
			  vb2_plane_size(&dst->vb2_buf, 0) >= sess->mctf_geom.bytes;
	f.out = s->mctf_direct ? vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0) :
				 sess->mctf_out[slot].dma;
	ret = ave_ipc_wait_ack(ave, buf);
	if (ret)
		return ret;
	cmd = ave_ipc_cmd(ave, buf, l->process_bytes);
	if (!cmd)
		return -EBUSY;
	ret = ave_mctf_build_process(l, cmd, l->process_bytes, sess->mctf_iova,
				     &f);
	if (ret)
		return ret;
	ave_write_header(ctx, cmd, AVE_OP_PROCESS, l->subtype + slot,
			 sess->frame_arg0, ave_session_arg2(ctx));
	if (!sess->frames)
		ave_debugfs_save_mctf(ave, AVE_IMAGE_PROCESS, cmd,
				      l->process_bytes);

	/* Pin both pictures before publishing the command. */
	s->input_mem = dst ? ave_vb2_get(&src->vb2_buf) : NULL;
	s->output_mem = dst ? ave_vb2_get(&dst->vb2_buf) : NULL;
	ret = ave_ipc_submit(ave, buf, l->process_bytes,
			     ave->layout->process_timeout_us, false);
	dev_dbg(ave->dev,
		"MCTF Process frame %u slot %u token %llu level %d in %pad out %pad: %d\n",
		f.frame, slot, sess->token, f.level, &f.in, &f.out, ret);
	if (ret)
		return ret;
	s->src = dst ? src : NULL;
	s->dst = dst;
	s->busy = true;
	s->status = 0;
	sess->progress = jiffies;
	sess->last_sent = jiffies;
	sess->busy++;
	sess->frames++;
	return 0;
}
