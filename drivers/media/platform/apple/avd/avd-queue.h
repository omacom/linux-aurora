/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef AVD_QUEUE_H_
#define AVD_QUEUE_H_

#include <linux/interrupt.h>
#include <linux/types.h>
#include <media/videobuf2-v4l2.h>

struct avd_dev;
struct avd_ctx;
struct avd_buf;	/* avd.h: { void *cpu; dma_addr_t addr; size_t size; } */

#define AVD_MAX_FRAMES		16	/* SPEC 4.1: 16 frame-param slots */

enum avd_frame_state {
	AVD_FRAME_FREE = 0,	/* SPEC: slot state 0 */
	AVD_FRAME_RESERVED,	/* 1: codec is filling it on the CPU */
	AVD_FRAME_SUBMITTED,	/* 3: owned by firmware */
	AVD_FRAME_DONE,		/* completion seen, waiting for in-order retire */
};

/* Codec says what must finish before this frame may be submitted. */
enum avd_frame_dep {
	AVD_DEP_NONE = 0,	/* independent: may overlap the previous frame */
	AVD_DEP_PREV_DONE,	/* previous frame of this ctx must be DONE first
				 * (VP9 backward adaptation, AV1 CDF update, ...) */
};

struct avd_frame {
	unsigned int idx;		/* 0..AVD_MAX_FRAMES-1, also the fp slot */
	u32 tag;			/* (seq << 4) | idx, echoed by completion */
	u64 seq;			/* global submission order */
	enum avd_frame_state state;
	enum avd_frame_dep dep;
	int status;			/* 0 or -EIO/-ETIMEDOUT from the backend */

	struct avd_ctx *ctx;
	struct vb2_v4l2_buffer *src;	/* OUTPUT, held until retire */
	struct vb2_v4l2_buffer *dst;	/* CAPTURE, held until retire */
	u32 frame_num;			/* SPEC 5.2 +0x0c */

	/* open backend: instruction program, DMA-visible, built by the codec */
	struct avd_buf *prog;		/* per-slot, allocated by core */
	size_t prog_words;

	/* apple backend: frame-params slot (SPEC 0x1130d0 on Borage) */
	struct avd_buf *fp;		/* per-slot, allocated by core, size from match data */
	u32 cmd[24];			/* 0x60 Borage command; core fills the 5.2 prefix,
					 * codec pack_cmd() fills the codec part */

	void *codec_priv;		/* per-frame codec state, owned by the codec */
	unsigned long deadline;		/* jiffies, per-frame watchdog */
	/* Legacy m2m HOLD_CAPTURE API retains scheduler ownership until retire. */
	bool scheduler_held;
	/* VP to PP instruction FIFO for this frame (open backends). */
	dma_addr_t fifo_iova;
};

struct avd_backend_ops {
	const char *name;
	int  (*probe)(struct avd_dev *avd);	/* apertures, fw source, version */
	int  (*boot)(struct avd_dev *avd);	/* power-up: fw present/loaded, link up, ready */
	void (*shutdown)(struct avd_dev *avd);
	unsigned int (*max_inflight)(struct avd_dev *avd);
	int  (*session_start)(struct avd_ctx *ctx);	/* apple: cmd 0; open: may be NULL */
	void (*session_stop)(struct avd_ctx *ctx);	/* apple: cmd 2 */
	/* Non-blocking. -EBUSY means inbox full: core retries on next completion. */
	int  (*submit)(struct avd_dev *avd, struct avd_frame *f);
	void (*abort)(struct avd_ctx *ctx);		/* apple: cmd 5 */
	irqreturn_t (*irq)(struct avd_dev *avd);	/* calls avd_frame_complete() */
	int  (*suspend)(struct avd_dev *avd);
	int  (*resume)(struct avd_dev *avd);
	int  (*set_property)(struct avd_ctx *ctx, u32 property, u32 value);
};

/* Codec half. Replaces avd_coded_fmt_ops.run/submit/done. CPU only, no MMIO. */
struct avd_codec_frame_ops {
	/* Read the request's controls, set f->dep, fill f->prog (open backend)
	 * or f->fp (apple backend), allocate f->codec_priv if needed. */
	int  (*prepare)(struct avd_ctx *ctx, struct avd_frame *f);
	/* Apple backend: fill the codec part of f->cmd (SPEC 6.x). */
	int  (*pack_cmd)(struct avd_ctx *ctx, struct avd_frame *f);
	/* Called in submission order per ctx, after completion or failure. */
	void (*retire)(struct avd_ctx *ctx, struct avd_frame *f,
		       enum vb2_buffer_state state);
};

/* Core API used by backends and codecs. */
void avd_frame_complete(struct avd_dev *avd, u32 tag, int status); /* any ctx, IRQ-safe */
bool avd_backend_is_apple(const struct avd_dev *avd);

#endif /* AVD_QUEUE_H_ */
