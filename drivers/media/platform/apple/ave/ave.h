/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Apple AVE video encoder
 *
 * Copyright The Aurora Silicon Contributors
 */
#ifndef _APPLE_AVE_H_
#define _APPLE_AVE_H_

#include <linux/atomic.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>

#include "ave-cmp420.h"
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>

#include "ave-sizes.h"
#include "ave-recovery.h"

struct debugfs_blob_wrapper;
struct dentry;
struct dev_pm_domain_list;
struct iommu_domain;
struct ave_vb2_mem;
struct ave_mctf_layout;

/* Frames the firmware can hold at once per session (Process "slots"). */
/* Firmware rate-control modes in struct ave_params.rc_mode */
#define AVE_RC_CBR 2
#define AVE_RC_CQ 3
#define AVE_RC_VBR 4

#define AVE_MAX_SLOTS 7
/* Sessions one firmware boot serves at once (macOS was seen running two). */
#define AVE_MAX_SESSIONS 2

/*
 * Coprocessor memory every firmware boot uses, all inside one allocation.
 * Host commands live in three areas: one large buffer for Start, small
 * buffers for the scalar commands and one buffer per Process slot, so a
 * command stays untouched until the firmware acknowledges it.
 */
#define AVE_IPC_BYTES (20 * SZ_1M)
#define AVE_IPC_CHANNEL_OFF 0x4000
#define AVE_IPC_HANDSHAKE_OFF 0x24000
#define AVE_IPC_SHARED_OFF 0x28000
#define AVE_IPC_SHARED_BYTES SZ_64K
#define AVE_IPC_CMD_OFF 0x40000
#define AVE_IPC_CMD_BYTES 0x1c000
#define AVE_IPC_SCALAR_OFF 0x5c000
#define AVE_IPC_SCALAR_BYTES 0x100
#define AVE_IPC_SCALAR_BUFS 16
#define AVE_IPC_PROC_OFF 0x60000
#define AVE_IPC_PROC_BYTES 0x10000
#define AVE_IPC_PROC_BUFS (AVE_MAX_SESSIONS * AVE_MAX_SLOTS)
#define AVE_IPC_END (AVE_IPC_PROC_OFF + AVE_IPC_PROC_BUFS * AVE_IPC_PROC_BYTES)

/* Command buffer ids, one bit each in ave_dev.unacked. */
#define AVE_BUF_CMD 0
#define AVE_BUF_SCALAR(i) (1 + (i))
#define AVE_BUF_PROC(s) (1 + AVE_IPC_SCALAR_BUFS + (s))
#define AVE_BUF_COUNT AVE_BUF_PROC(AVE_IPC_PROC_BUFS)

#define AVE_MSG_MAX SZ_4K

/*
 * Rings the host serves. IO and IO_T2H carry commands and answers on every
 * firmware; the 6070 firmware also borrows shared memory (SHAREDMALLOC) and
 * logs (TERMINAL) over rings of their own.
 */
enum {
	AVE_RING_IO,
	AVE_RING_T2H,
	AVE_RING_SMALLOC,
	AVE_RING_LOG,
	AVE_RINGS,
};

#define AVE_SMALLOC_GRANULE SZ_4K
#define AVE_SMALLOC_UNITS (SZ_4M / AVE_SMALLOC_GRANULE)
#define AVE_SMALLOC_MAX 32

enum ave_codec {
	AVE_CODEC_H264 = 1,
	AVE_CODEC_HEVC = 2,
};

struct ave_buf {
	void *cpu;
	dma_addr_t dma;
	size_t size; /* what the firmware is told */
	size_t alloc; /* what is mapped, page rounded */
	unsigned long attrs;
};

struct ave_fw_segment {
	phys_addr_t phys;
	u64 iova;
	size_t size;
};

struct ave_ring {
	char name[16];
	u32 mode;
	u32 doorbell;
	u32 slots;
	u32 offset;
	int consumer;
	int producer;
};

enum ave_fw_state {
	AVE_FW_OFF, /* coprocessor in reset */
	AVE_FW_RUNNING,
	AVE_FW_HALTED, /* told to halt; restarts through the boot path */
	AVE_FW_BROKEN,
	AVE_FW_RECOVERING,	/* control off; awaiting a runtime power cycle */
};

/* The two large session commands, and their sizes per codec */
enum ave_image {
	AVE_IMAGE_START,
	AVE_IMAGE_PROCESS,
	AVE_IMAGE_COUNT,
};

/* Logical operations; wire IDs are selected by the firmware ABI. */
enum ave_opcode {
	AVE_OP_CONFIG = 1,
	AVE_OP_HALT,
	AVE_OP_OPEN,
	AVE_OP_CLOSE,
	AVE_OP_START = 6,
	AVE_OP_STOP,
	AVE_OP_PROCESS,
	AVE_OP_COMPLETE,
	AVE_OP_PRIORITY,
	AVE_OP_RESET,
	AVE_OP_FLUSH,
	AVE_OP_COUNT,
};

#include "ave-layout.h"

/*
 * Driver-private codec-class controls.
 *
 * PRIORITIZE_SPEED is VideoToolbox's PrioritizeEncodingSpeedOverQuality:
 * off (the default) codes for quality, on trades some compression
 * efficiency for encoding speed. It is read when streaming starts.
 */
#define V4L2_CID_APPLE_AVE_BASE (V4L2_CTRL_CLASS_CODEC | 0x1200)
#define V4L2_CID_APPLE_AVE_PRIORITIZE_SPEED (V4L2_CID_APPLE_AVE_BASE + 0)

/* The firmware build whose command layouts the driver generates */
#define AVE_FW_TEXT_CRC_9003_78_0 0x69c54454
/* The M4 Pro (T6040) build of the same 25G83 firmware. */
#define AVE_FW_TEXT_CRC_T6040_25G83 0x68d5f9f6
/*
 * The M4 Max (J575c) build of encoder firmware 9003.78.0, as iBoot 25G83 loads it:
 * DATA is 0x130000, and the loader writes a per-boot word at TEXT 0x4224, so
 * this is the TEXT CRC with that word zeroed.
 */
#define AVE_FW_TEXT_CRC_T6041_25G83 0x9a080cad
#define AVE_FW_TEXT_CRC_T6050_26A428 0x87279429
/* macOS 27.3 beta 1 (26C5035r) / T6050: TEXT 0x158000, DATA 0x134000. */
#define AVE_FW_TEXT_CRC_T6050_26C5035 0xc25b4ddb
/* The M5 (T8142) build of the same 26A428 firmware (encoder firmware 9013.45.1). */
#define AVE_FW_TEXT_CRC_T8142_26A428 0x8a3784bd
/* The M4 (T8132) build preloaded by the macOS 27.2 (26B123) stub. */
#define AVE_FW_TEXT_CRC_T8132_26B123 0x99fc929a
/* The M3 (T8122) build preloaded by the macOS 27.2 (26B123) stub. */
#define AVE_FW_TEXT_CRC_T8122_26B123 0x918d70d3
/*
 * The M5 build of macOS 27.2 (26B123; 27.2.1 26B131 ships it unchanged),
 * AppleAVE2FW 9013.55.1: TEXT 0x154000, DATA 0x12c000, as 26A428's.
 */
#define AVE_FW_TEXT_CRC_T8142_26B123 0x24eb4625

struct ave_ctx;

/* Register offsets for the shared 0x40 coprocessor architecture. */
struct ave_coproc_regs {
	u32 vcbar;
	u32 timebase_freq;
	u32 timebase;
	u32 ctrl;
	u32 status;
	u32 boot_arg;
	u32 keep_mappings;
};

/* One preloaded firmware build of a variant: its segment sizes and TEXT CRC. */
struct ave_fw_build {
	const char *name;
	u32 text_size;
	u32 data_size;
	u32 text_crc;
};

struct ave_variant {
	const struct ave_coproc_regs *regs;
	u32 fw_text_size;
	u32 fw_data_size;
	/* A second DATA size the variant accepts (the M4 Max build), or 0 */
	u32 fw_data_size_alt;
	/*
	 * When set, the builds the variant admits. They replace fw_text_size,
	 * fw_data_size and text_crc: the segment sizes select the build, and
	 * the build names the TEXT CRC it must have.
	 */
	const struct ave_fw_build *builds;
	unsigned int build_count;
	u32 boot_device_id;
	u32 device_type;
	u32 lfs_ref_scale;
	u32 text_crc;
	u32 boot_arg_value;
	u32 timebase_multiplier;
	u64 vcbar_mask;
	u64 vcbar_flags;
	/*
	 * TEXT offset of the 64-bit cell the loader fills with the DATA
	 * address; it is zero in the firmware image. 0 when there is none.
	 */
	u32 text_link_cell;
	/*
	 * TEXT offset of a 32-bit word the loader sets differently on every
	 * boot; a TEXT CRC that is not a known build is retried with it zeroed.
	 * 0 when there is none.
	 */
	u32 text_boot_word;
	/*
	 * Mailbox offset of the SVE idle (clock gating) word that must read 0
	 * before the firmware starts, or 0 when the variant has none.
	 */
	u32 mbox_idle;
	/* Restore DATA from the boot snapshot before every start, the first included. */
	bool restore_data_first;

	size_t extra0_size;
	/* Register sequence and boot/hello configuration have been qualified. */
	bool boot_qualified;
};

/* Start surfaces of an MCTF session, aliases included */
#define AVE_MCTF_BUFS 96
/* MCTF picture surfaces: inputs used in turn, one output per slot */
#define AVE_MCTF_IN_BUFS 5
#define AVE_MCTF_OUT_BUFS 2
/* ave_dev.mctf_pool_flags */
#define AVE_MCTF_POOL_LENT	0	/* a session holds the pool */
#define AVE_MCTF_POOL_READY	1	/* every pool buffer is allocated */

struct ave_dev {
	struct device *dev;
	const struct ave_variant *variant;
	const struct ave_fw_layout *layout;
	void __iomem *codec;
	void __iomem *mbox;
	void __iomem *coproc;
	struct dev_pm_domain_list *pd_list;
	struct iommu_domain *domain;

	struct ave_fw_segment text;
	struct ave_fw_segment data;
	u32 text_crc;
	u32 data_crc;
	void *data_pristine;
	bool data_dirty;
	/* The last boot reached RUN but the firmware never posted its hello. */
	bool boot_unanswered;

	/* hw_lock protects recovery history and quarantined DMA allocations. */
	struct ave_recovery_history recovery;
	unsigned int recovery_count;
	struct ave_session *failed_sessions;
	bool inject_wedge;

	/* debugfs: the last Start and first Process built, per codec */
	struct dentry *debugfs;
	void *dump[2][AVE_IMAGE_COUNT];

	/* Serialises every firmware and ring access. */
	struct mutex hw_lock;
	enum ave_fw_state fw_state;
	struct ave_buf ipc;
	struct ave_buf boot_cfg;
	struct ave_buf log;
	struct ave_buf heap;
	dma_addr_t extra0_dma;
	size_t extra0_map_size;
	bool extra0_mapped;
	bool module_pinned;
	/* A 6070 lifecycle command is serialized per logical operation. */
	struct ave_reply expected[AVE_OP_COUNT];
	u64 ipc_fw_base;
	u32 channel_bytes;
	struct ave_ring ring[AVE_RINGS];
	/* Shared memory lent to the firmware, in AVE_SMALLOC_GRANULE units. */
	DECLARE_BITMAP(smalloc_map, AVE_SMALLOC_UNITS);
	struct {
		u32 first, units;
	} smalloc[AVE_SMALLOC_MAX];
	u32 next_client;
	u32 unacked;
	u8 want[AVE_OP_COUNT];
	int reply_err;
	struct ave_reply last_reply; /* for the debug state dump */
	unsigned int scalar_next;
	struct ave_ctx *sessions[AVE_MAX_SESSIONS];
	u8 msg[AVE_MSG_MAX];
	u32 msg_len;

	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	/* The MCTF video device, present on a qualified firmware build */
	struct video_device mctf_vdev;
	struct v4l2_m2m_dev *mctf_m2m;
	bool mctf_registered;
	void *mctf_dump[AVE_IMAGE_COUNT];
	/*
	 * MCTF DMA memory: allocated by the first MCTF session and lent to
	 * every later one at the same addresses. Never unmapped while the
	 * firmware may have run (see ave_sess_mctf_init()).
	 */
	struct ave_buf mctf_pool[AVE_MCTF_BUFS];
	struct ave_buf mctf_pool_in[AVE_MCTF_IN_BUFS];
	struct ave_buf mctf_pool_out[AVE_MCTF_OUT_BUFS];
	unsigned long mctf_pool_flags;
	/* CRC of the pool when it was last given back, checked at the next borrow */
	u32 mctf_pool_crc;
	bool mctf_pool_crc_valid;
	/* The coprocessor was started at least once since probe. */
	bool fw_ever_ran;
	/*
	 * BOOT_ARG did not read back as written after a boot: a clear
	 * BOOT_ARG proves nothing, so stopped sessions are never reaped.
	 */
	/* The encoder's DARTs, read-only for the stream audit */
	void __iomem *dart_regs[2];
	phys_addr_t dart_phys[2];
	u32 dart_sid[2];
	/* every stream the device tree attaches on that DART, bit per SID */
	u32 dart_sid_mask[2];
	unsigned int n_darts;
	bool witness_unusable;
	/* Times the MCTF pool CRC changed while no session held it */
	u32 mctf_pool_changed;
	/* debugfs: the last filtered frame's input and output surfaces, raw */
	void *mctf_surf_dump[2];
	bool mctf_capture;
	/* The video device's ioctl and queue lock. */
	struct mutex vdev_lock;
	struct workqueue_struct *wq;

	/*
	 * The mailbox interrupt: masked by its handler until the next ring
	 * poll has cleared the mailbox status (irq_lock). irq_kick wakes a
	 * waiting ave_ipc_pump(); irq_work handles events nobody waits for.
	 */
	int irq;
	spinlock_t irq_lock;
	bool irq_masked;
	bool irq_kick;
	wait_queue_head_t irq_wait;
	struct work_struct irq_work;
};

/* Encoding parameters, fixed for the lifetime of a session. */
struct ave_params {
	enum ave_codec codec;
	u32 width, height; /* visible picture */
	u32 cw, ch; /* coded size, multiples of 16 */
	u32 stride; /* NV12 or P010 line stride in bytes, both planes */
	u32 sample_bytes; /* 1 for NV12, 2 for P010 */
	u32 mbs; /* 16x16 macroblocks */
	u32 fps_num, fps_den; /* frames per second = num / den */
	u32 fps_int; /* the same, rounded */
	u32 slots;
	u32 refs; /* max_num_ref_frames + 1 */
	bool bframes;
	bool intra_only;
	bool reorder_off; /* the client asked for no B-frames */
	u32 gop;
	u32 rc_mode; /* AVE_RC_* */
	u32 bitrate;
	u32 limit_bytes; /* 0: no data-rate limit */
	s32 min_qp;
	u32 max_qp;
	u32 profile; /* H.264: 2, 4, 6; HEVC: 1 */
	u32 level; /* firmware level enum, 1-based */
	bool high;
	u32 coded_cap;
	u32 bit_depth; /* zero also means the existing 8-bit API */
	u32 initial_qp; /* CQ only; bit-rate modes use 26 */
	bool speed; /* prioritize encoding speed over quality */
};

struct ave_slot {
	struct ave_buf coded;
	struct ave_buf meta;
	struct ave_buf aux; /* H.264 per-frame buffer, HEVC slice headers */
	struct ave_buf extra; /* counted layout: per-slot 16 KiB buffer */
	struct ave_buf bounce; /* input copied to the encoder's stride */
	struct vb2_v4l2_buffer *src;
	struct vb2_v4l2_buffer *dst; /* MCTF: the firmware writes the frame here */
	struct ave_vb2_mem *input_mem; /* independent firmware mapping reference */
	struct ave_vb2_mem *output_mem; /* the same, for an MCTF CAPTURE buffer */
	u8 mctf_input; /* MCTF: the input surface this frame used */
	bool mctf_direct; /* MCTF: the engine wrote the CAPTURE buffer itself */
	u32 status;
	bool busy; /* Process sent, reply pending */
	bool done; /* reply in, output not delivered yet */
};

struct ave_frame_result {
	u32 coded_bytes;
	u32 parameter_set_bytes;
	u32 slice_header_bytes;
	u32 ordinal;
	u32 frame_type;
};

#define AVE_MAX_REFS 5
#define AVE_TILE_BUFS 8
#define AVE_ENTROPY_SETS 4
#define AVE_NEIGHBOUR_SETS 4
#define AVE_6070_LOWRES_RESULTS 4
#define AVE_6070_AVC_ENTROPY_TILES 2

/* Fixed 6070 host-interface allocations, independent of picture geometry. */
#define AVE_6070_CLIENT_MEM_BYTES SZ_1M
#define AVE_6070_CODED_HEADER_BYTES 0x23000
#define AVE_AVC_FLAT_SCALING_WEIGHT 16
#define AVE_AVC_RECON_SKIP_MODE 3
#define AVE_AVC_STATIC_AREA_MASK_ALL 0xffff

struct ave_session {
	struct ave_session *failed_next;
	struct ave_params p;
	struct ave_buf hdr;
	struct ave_buf client_mem;
	/* 6070: per-session firmware-generated VPS/SPS/PPS output. */
	struct ave_buf param_sets;
	struct ave_buf ref_a[AVE_MAX_REFS];
	struct ave_buf ref_b[AVE_MAX_REFS];
	struct ave_buf ref_c[AVE_MAX_REFS];
	struct ave_buf ref_d[AVE_MAX_REFS];
	struct ave_buf ref_e[AVE_MAX_REFS];
	struct ave_buf ref_f[AVE_MAX_REFS];
	struct ave_buf mb;
	struct ave_buf mb_extra[2];
	struct ave_buf row_a[AVE_NEIGHBOUR_SETS];
	struct ave_buf row_b[AVE_NEIGHBOUR_SETS];
	struct ave_buf fixed[AVE_NEIGHBOUR_SETS];
	struct ave_buf neighbour_fw[AVE_NEIGHBOUR_SETS];
	struct ave_buf tile[AVE_ENTROPY_SETS][AVE_TILE_BUFS];
	struct ave_buf tile_aux[AVE_ENTROPY_SETS][AVE_TILE_BUFS];
	struct ave_buf hevc_extra[2];
	struct ave_buf hevc_transcoded[2];
	u8 *codec_state;
	size_t codec_state_bytes;
	struct ave_slot slot[AVE_MAX_SLOTS];
	/* MCTF client (work type 3): its layout and firmware-owned surfaces */
	const struct ave_mctf_layout *mctf;
	struct ave_buf mctf_buf[AVE_MCTF_BUFS];
	dma_addr_t mctf_iova[AVE_MCTF_BUFS];
	struct ave_buf mctf_in[AVE_MCTF_IN_BUFS];
	struct ave_buf mctf_out[AVE_MCTF_OUT_BUFS];
	/* CPU copies of one picture surface and one NV12 picture */
	struct ave_cmp420_geom mctf_geom;
	u8 *mctf_surf, *mctf_lin;
	bool mctf_borrowed; /* mctf_buf/in/out are the device pool's */
	unsigned int mctf_last_input; /* input surface of the last new picture */
	/* Header words; zero means the encoder's values */
	u32 unit, arg1, frame_arg0;
	u8 done_fifo[AVE_MAX_SLOTS];
	unsigned int done_head;
	unsigned int done_count;
	unsigned int busy;
	u32 frames; /* Process commands sent */
	u32 client;
	u64 token; /* command order token, per session */
	unsigned int index; /* in ave_dev.sessions[] */
	bool fw_open; /* Open sent, Close not yet */
	bool resume; /* closed for system sleep, reopen on next frame */
	bool copy_noted;
	/* CBR: bytes under budget so far, and who pads (see ave_sess_cbr_fill()) */
	s64 cbr_surplus;
	bool fw_fills;
	bool host_fill_noted;
	bool warned;
	bool checked[2]; /* Start/Process images checked for stale addresses */
	bool handed; /* Start sent: the firmware knows the buffers */
	bool nudged; /* Complete sent because input stopped */
	unsigned long progress; /* jiffies of the last frame sent or answered */
	unsigned long last_sent; /* jiffies of the last frame sent */
};

struct ave_ctx {
	struct v4l2_fh fh;
	struct ave_dev *ave;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *hevc_profile_ctrl;
	bool hevc_profile_auto; /* Main 10 was set by S_FMT, not the client */
	struct v4l2_pix_format_mplane src_fmt;
	struct v4l2_pix_format_mplane dst_fmt;
	struct v4l2_fract timeperframe;
	struct work_struct run_work;
	struct delayed_work poll_work;

	u32 bitrate_mode;
	u32 quality; /* V4L2 constant quality, 0..100 */
	u32 bitrate;
	u32 bitrate_peak;
	u32 gop_size;
	u32 b_frames;
	u32 h264_profile;
	u32 h264_level;
	u32 h264_min_qp;
	u32 h264_max_qp;
	u32 hevc_profile;
	u32 hevc_level;
	u32 hevc_min_qp;
	u32 hevc_max_qp;
	bool force_key_frame;
	bool prioritize_speed;

	struct ave_session *sess;
	bool mctf; /* an MCTF context, on the MCTF video device */
	s32 mctf_level;
	u32 mctf_strength;
	atomic_t ready; /* outputs waiting for a CAPTURE buffer */
	bool draining;
	bool flush_pending;
	bool stopped;
	/* Serialises QBUF with fatal-error publication and ready-queue draining. */
	spinlock_t queue_lock;
	bool failed;
	struct vb2_v4l2_buffer *drain_src;
	u32 src_sequence;
	u32 dst_sequence;
};

static inline struct ave_ctx *ave_file_to_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct ave_ctx, fh);
}

extern const struct ave_variant ave_t6000, ave_t6041, ave_t8112, ave_t8132,
	ave_m3;

/* ave-fw.c */
u64 ave_fw_vcbar(const struct ave_variant *variant, u64 text_iova);
int ave_fw_probe(struct ave_dev *ave);
int ave_fw_text_id_crc(const void *text, size_t size, u32 cell_off,
		       u64 data_iova, u32 *crc);
int ave_fw_text_boot_crc(const void *text, size_t size, u32 word_off,
			 u32 *crc);
void ave_fw_remove(struct ave_dev *ave);
int ave_fw_ensure_running(struct ave_dev *ave);
int ave_dart_audit(struct ave_dev *ave, const char *when);
struct seq_file;
void ave_dart_dump(struct ave_dev *ave, struct seq_file *m);
void ave_fw_halt(struct ave_dev *ave);
int ave_fw_shutdown(struct ave_dev *ave);
int ave_fw_runtime_suspend(struct ave_dev *ave);
void ave_fw_mark_broken(struct ave_dev *ave, int err, const char *what);
void ave_fw_lost(struct ave_dev *ave);
int ave_fw_check_build(struct ave_dev *ave);
int ave_fw_check_boot(struct ave_dev *ave);
void ave_fw_release_boot(struct ave_dev *ave);
int ave_fw_lifecycle_test(struct ave_dev *ave, enum ave_codec codec);
int ave_alloc(struct ave_dev *ave, struct ave_buf *b, size_t size,
	      unsigned long attrs);
int ave_alloc_padded(struct ave_dev *ave, struct ave_buf *b, size_t size,
		     size_t alloc, unsigned long attrs);
void ave_free(struct ave_dev *ave, struct ave_buf *b);

/* ave-ipc.c */
int ave_ipc_check_hello(const struct ave_fw_transport *t, const u32 hello[4]);
int ave_ipc_parse_channels(struct ave_dev *ave);
int ave_ipc_serve_smalloc(struct ave_dev *ave);
void ave_ipc_drain_log(struct ave_dev *ave);
int ave_ipc_negotiate(struct ave_dev *ave, const u32 hello[4]);
void *ave_ipc_cmd(struct ave_dev *ave, unsigned int buf, size_t len);
int ave_ipc_scalar_buf(struct ave_dev *ave);
int ave_ipc_wait_ack(struct ave_dev *ave, unsigned int buf);
int ave_ipc_submit(struct ave_dev *ave, unsigned int buf, size_t len,
		   u64 timeout_us, bool reply);
int ave_ipc_pump(struct ave_dev *ave, unsigned int wait_ms,
		 bool (*cond)(struct ave_dev *ave, void *arg), void *arg);
int ave_ipc_settle(struct ave_dev *ave, unsigned int wait_ms);
int ave_ipc_irq_init(struct ave_dev *ave, struct platform_device *pdev);
void ave_ipc_dump(struct ave_dev *ave, const char *why);

/* ave-writer.c: pure command builders, also exercised by KUnit. */
u32 ave_session_arg2(struct ave_ctx *ctx);
u32 ave_session_setup_arg2(struct ave_ctx *ctx);
void ave_write_header(struct ave_ctx *ctx, u8 *cmd, enum ave_opcode op,
		      u32 echo, u32 arg0, u32 arg2);
void ave_write_global(struct ave_dev *ave, u8 *cmd, enum ave_opcode op);
void ave_stamp_command(const struct ave_fw_abi *abi, u8 *cmd, u64 timeout,
		       u64 created);
void ave_write_surface(const struct ave_fw_layout *fw, u8 *cmd,
		       const struct ave_surface_layout *l, unsigned int index,
		       dma_addr_t addr, u32 size);
int ave_build_start(struct ave_ctx *ctx, u8 *cmd, size_t len);
int ave_build_process(struct ave_ctx *ctx, u8 *cmd, size_t len,
		      unsigned int slot, dma_addr_t input, u32 bitrate,
		      bool force_key, bool last);
int ave_init_codec_state(struct ave_session *sess);

/* ave-cmd.c */
int ave_cmd_config(struct ave_dev *ave);
int ave_cmd_halt(struct ave_dev *ave);
int ave_cmd_open(struct ave_ctx *ctx);
int ave_cmd_start(struct ave_ctx *ctx);
int ave_cmd_process(struct ave_ctx *ctx, unsigned int slot,
		    struct vb2_v4l2_buffer *src, dma_addr_t input, u32 bitrate,
		    bool force_key, bool last);
int ave_cmd_complete(struct ave_ctx *ctx, unsigned int count);
int ave_cmd_flush(struct ave_ctx *ctx);
int ave_cmd_close(struct ave_ctx *ctx);

/* ave-vb2.c */
extern const struct vb2_mem_ops ave_vb2_memops;
struct ave_vb2_mem *ave_vb2_get(struct vb2_buffer *vb);
void ave_vb2_put(void *buf_priv);

/* ave-sess.c */
u32 ave_cq_initial_qp(u32 quality);
int ave_sess_decode_result(const struct ave_fw_layout *fw, enum ave_codec codec,
			   const u8 *meta, size_t meta_bytes,
			   struct ave_frame_result *result);
struct ave_session *ave_sess_create(struct ave_ctx *ctx);
void ave_sess_destroy(struct ave_ctx *ctx);
void ave_sess_reap(struct ave_dev *ave);
int ave_sess_free_slot(struct ave_session *sess);
void ave_sess_reply(struct ave_dev *ave, const struct ave_reply *reply);
int ave_sess_input(struct ave_ctx *ctx, unsigned int slot,
		   struct vb2_v4l2_buffer *src, dma_addr_t *input);
int ave_sess_output(struct ave_ctx *ctx, unsigned int slot,
		    struct vb2_v4l2_buffer *dst, bool *keyframe);
void ave_sess_prepare_slot(struct ave_session *sess, unsigned int slot);
bool ave_sess_stalled(struct ave_session *sess);
bool ave_sess_starved(struct ave_session *sess);
int ave_sess_mctf_init(struct ave_ctx *ctx, struct ave_session *sess);
void ave_mctf_pool_free(struct ave_dev *ave);
int ave_sess_mctf_borrow(struct ave_ctx *ctx);

/* ave-image.c */
void ave_image_start(u8 *cmd, size_t len, const struct ave_fw_layout *fw,
		     enum ave_codec codec, bool reorder_off,
		     bool hevc_low_delay);
void ave_image_process(u8 *cmd, size_t len, const struct ave_fw_layout *fw,
		       enum ave_codec codec);

/* ave-drv.c */
void ave_debugfs_save(struct ave_dev *ave, enum ave_codec codec,
		      enum ave_image kind, const void *cmd, size_t len);
void ave_debugfs_save_mctf(struct ave_dev *ave, enum ave_image kind,
			   const void *cmd, size_t len);

/* ave-v4l2.c: the session plumbing both video devices share */
extern const struct vb2_ops ave_qops;
void ave_v4l2_suspend(struct ave_dev *ave);
int ave_v4l2_register(struct ave_dev *ave);
void ave_v4l2_unregister(struct ave_dev *ave);
enum ave_codec ave_ctx_codec(struct ave_ctx *ctx);
int ave_session_index(struct ave_dev *ave, struct ave_ctx *ctx);
int ave_session_reopen(struct ave_ctx *ctx);
void ave_job_fail(struct ave_ctx *ctx, int err);

/* ave-mctf.c, ave-mctf-v4l2.c */
int ave_mctf_check_build(struct ave_dev *ave);
int ave_mctf_cmd_start(struct ave_ctx *ctx);
int ave_mctf_cmd_process(struct ave_ctx *ctx, unsigned int slot,
			 struct vb2_v4l2_buffer *src,
			 struct vb2_v4l2_buffer *dst, bool same_input);
int ave_mctf_register(struct ave_dev *ave);
void ave_mctf_unregister(struct ave_dev *ave);

#endif
