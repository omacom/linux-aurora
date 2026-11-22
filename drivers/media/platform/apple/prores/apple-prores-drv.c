// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple ProRes (APR) V4L2 memory-to-memory encoder.
 *
 * One job runs at a time. The driver writes a descriptor into a 128 KiB DMA
 * ring, advances the producer head and waits for the completion interrupt.
 * Raw frames come in on the OUTPUT queue as semi-planar 4:2:2 or 4:4:4 (8 or
 * 16 bits per sample) and complete ProRes frames go out on CAPTURE.
 *
 * Based on Acelogic's J700 ProRes diagnostic, which first encoded on this
 * engine under Linux, and on the m1n1 proxyclient ProRes experiment (MIT).
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../apple-video-nr.h"
#include "apple-prores.h"

#define APR_NAME		"apple-prores"
#define APR_MIN_W		16
#define APR_MIN_H		16
#define APR_MAX_W		8192
#define APR_MAX_H		8192
#define APR_TIMEOUT_MS		1000
#define APR_POLL_MS		4
#define APR_AUTOSUSPEND_MS	500

struct apr_fmt {
	u32 fourcc;
	enum apr_input input;
	u8 bytes;	/* per sample */
	bool is444;
};

static const struct apr_fmt apr_raw_fmts[] = {
	{ V4L2_PIX_FMT_NV16, APR_INPUT_422_8, 1, false },
	{ V4L2_PIX_FMT_P216, APR_INPUT_422_16, 2, false },
	{ V4L2_PIX_FMT_NV24, APR_INPUT_444_8, 1, true },
	{ V4L2_PIX_FMT_P416, APR_INPUT_444_16, 2, true },
};


/*
 * Settings the engine and its AXI-to-fabric bridge need before the first
 * job, as read-modify-write (offset, mask, value), in order. T8140 values.
 */
static const struct apr_tunable apr_t8140_engine_tunables[] = {
	{ 0x008, 0x00000008, 0x00000008 },
	{ 0x118, 0x0fff0fff, 0x00000280 },
	{ 0x11c, 0x0fff0fff, 0x000001e0 },
	{ 0x148, 0x00000001, 0x00000001 },
	{ 0x180, 0x8fff3fff, 0x800a04ff },
	{ 0x184, 0x000000ff, 0x00000001 },
	{ 0x188, 0x0fff0fff, 0x02c00040 },
	{ 0x1c0, 0x8fff3fff, 0x800404ff },
	{ 0x1c4, 0x000000ff, 0x00000003 },
	{ 0x1c8, 0x0fff0fff, 0x00000140 },
	{ 0x200, 0x8fff3fff, 0x800504ff },
	{ 0x204, 0x000000ff, 0x00000002 },
	{ 0x208, 0x0fff0fff, 0x01400140 },
	{ 0x240, 0x8fff3fff, 0x800004ff },
	{ 0x244, 0x000000ff, 0x00000007 },
	{ 0x248, 0x000000ff, 0x00000040 },
	{ 0x280, 0x8fff3fff, 0x800104ff },
	{ 0x284, 0x000000ff, 0x00000006 },
	{ 0x288, 0x000000ff, 0x0000007e },
	{ 0x2c0, 0x8fff3fff, 0x800204ff },
	{ 0x2c4, 0x000000ff, 0x00000005 },
	{ 0x2c8, 0x000000ff, 0x00000040 },
	{ 0x300, 0x8fff3fff, 0x800304ff },
	{ 0x304, 0x000000ff, 0x00000004 },
	{ 0x308, 0x000000ff, 0x00000040 },
	{ 0x340, 0xbfffff86, 0x81000006 },
	{ 0x344, 0xffffffff, 0x0b0a0908 },
	{ 0x348, 0x0000ffff, 0x00000720 },
	{ 0x34c, 0x000fffff, 0x000014ff },
	{ 0x350, 0x00ffffff, 0x00928170 },
	{ 0x380, 0x8fff3fff, 0x800b08ff },
	{ 0x384, 0x000000ff, 0x0000000c },
	{ 0x388, 0x0fff0fff, 0x02800040 },
	{ 0x394, 0xffffffc0, 0x00200000 },
	{ 0x3c0, 0x8fff3fff, 0x800b08ff },
	{ 0x3c4, 0x000000ff, 0x00000019 },
	{ 0x3c8, 0x0fff0fff, 0x03000010 },
	{ 0x4c0, 0x800f3fff, 0x800b0880 },
	{ 0x4c4, 0x000000ff, 0x0000001b },
	{ 0x4c8, 0x0fff0fff, 0x01f80008 },
	{ 0x500, 0x800f3fff, 0x800604ff },
	{ 0x504, 0x000000ff, 0x0000000d },
	{ 0x508, 0x0fff0fff, 0x00000100 },
	{ 0x540, 0x800f3fff, 0x80000500 },
	{ 0x544, 0x000000ff, 0x00000011 },
	{ 0x548, 0x000000ff, 0x0000007e },
	{ 0x580, 0x800f3fff, 0x80010500 },
	{ 0x584, 0x000000ff, 0x00000010 },
	{ 0x588, 0x000000ff, 0x0000007e },
	{ 0x5c0, 0x800f3fff, 0x80020500 },
	{ 0x5c4, 0x000000ff, 0x0000000f },
	{ 0x5c8, 0x000000ff, 0x00000040 },
	{ 0x600, 0x800f3fff, 0x80030500 },
	{ 0x604, 0x000000ff, 0x0000000e },
	{ 0x608, 0x000000ff, 0x00000040 },
	{ 0x640, 0x802ff040, 0x80070040 },
	{ 0x644, 0x00ffffff, 0x00071716 },
	{ 0x648, 0xffff0003, 0x00ff0003 },
	{ 0x650, 0x0fff0fff, 0x01e00008 },
	{ 0x680, 0x802ff040, 0x80081040 },
	{ 0x684, 0x00ffffff, 0x00071514 },
	{ 0x688, 0xffff0003, 0x00ff0003 },
	{ 0x690, 0x0fff0fff, 0x01e80010 },
	{ 0x6c0, 0x802ff040, 0x80092040 },
	{ 0x6c4, 0x00ffffff, 0x00071312 },
	{ 0x6c8, 0xffff0003, 0x00ff0003 },
	{ 0x6d0, 0x0fff0fff, 0x01f80008 },
	{ 0x700, 0x800f3fff, 0x800b0880 },
	{ 0x704, 0x000000ff, 0x00000018 },
	{ 0x708, 0x0fff0fff, 0x01000080 },
	{ 0x714, 0xffffffc0, 0x00200000 },
	{ 0x740, 0x800f3fff, 0x800b0880 },
	{ 0x744, 0x000000ff, 0x0000001a },
	{ 0x748, 0x0fff0fff, 0x01800008 },
	{ 0x780, 0x800f3fff, 0x800b0880 },
	{ 0x784, 0x000000ff, 0x0000001c },
	{ 0x788, 0x0fff0fff, 0x01e00008 },
	{ 0x7c0, 0x800f3fff, 0x800b0880 },
	{ 0x7c4, 0x000000ff, 0x0000001d },
	{ 0x7c8, 0x0fff0fff, 0x01e80008 },
};

static const struct apr_tunable apr_t8140_wrapper_tunables[] = {
	{ 0x0000, 0x800000ff, 0x8000001a },
	{ 0x0004, 0x80ff00ff, 0x800b000c },
	{ 0x0008, 0x80ff00ff, 0x800b000c },
	{ 0x000c, 0x80ff00ff, 0x800b000c },
	{ 0x0010, 0x80ff00ff, 0x80000003 },
	{ 0x0014, 0x80ff00ff, 0x80000003 },
	{ 0x0018, 0x800000ff, 0x8000000f },
	{ 0x001c, 0x800000ff, 0x80000020 },
	{ 0x0020, 0x80ff00ff, 0x8014000c },
	{ 0x0024, 0x80ff00ff, 0x801a0006 },
	{ 0x0028, 0x80ff00ff, 0x80090003 },
	{ 0x002c, 0x80ff00ff, 0x80090003 },
	{ 0x0030, 0x800000ff, 0x80000013 },
	{ 0x0034, 0x800000ff, 0x8000000d },
	{ 0x0038, 0x800000ff, 0x8000000d },
	{ 0x0100, 0x000000ff, 0x000000ff },
	{ 0x0108, 0x00000006, 0x00000000 },
	{ 0x0304, 0x3f3f0000, 0x04040000 },
	{ 0x0314, 0x80000000, 0x80000000 },
	{ 0x031c, 0x80000000, 0x80000000 },
	{ 0x0408, 0xffff0020, 0x00100020 },
	{ 0x0700, 0x00000400, 0x00000400 },
	{ 0x0748, 0x07ff07ff, 0x00100010 },
	{ 0x0800, 0x00000400, 0x00000400 },
	{ 0x0848, 0x07ff07ff, 0x00800040 },
	{ 0x0a00, 0xffffffff, 0x7fffffff },
	{ 0x0a04, 0x00000100, 0x00000100 },
	{ 0x4008, 0x00000001, 0x00000000 },
	{ 0x400c, 0x00000001, 0x00000000 },
	{ 0x4014, 0x00000001, 0x00000000 },
	{ 0x4018, 0x00000001, 0x00000000 },
	{ 0x401c, 0x00000001, 0x00000001 },
};

/*
 * T6000 constants: Copyright The Asahi Linux Contributors.
 * MIT license: see m1n1-LICENSE.
 * MIT m1n1 f3ac615eb1376d3bbba24cd84c414b9e41892adb,
 * proxyclient/experiments/prores.py:42-89, in the published order.
 * APR a0 has no wrapper or SCODEC-system initialization table.
 */
static const struct apr_tunable apr_t6000_engine_tunables[] = {
	{ 0x118, 0x07ff07ff, 0x00000600 },
	{ 0x148, 0x00000001, 0x00000001 },
	{ 0x160, 0x800f3fff, 0x800a04ff },
	{ 0x164, 0x07ff07ff, 0x07800080 },
	{ 0x170, 0x800f3fff, 0x800404ff },
	{ 0x174, 0x07ff07ff, 0x06000080 },
	{ 0x180, 0x800f3fff, 0x800504ff },
	{ 0x184, 0x07ff07ff, 0x06800080 },
	{ 0x190, 0x800f3fff, 0x800004ff },
	{ 0x194, 0x000000ff, 0x00000040 },
	{ 0x1a0, 0x800f3fff, 0x800104ff },
	{ 0x1a4, 0x000000ff, 0x00000080 },
	{ 0x1b0, 0x800f3fff, 0x800204ff },
	{ 0x1b4, 0x000000ff, 0x00000040 },
	{ 0x1c0, 0x800f3fff, 0x800304ff },
	{ 0x1c4, 0x000000ff, 0x00000040 },
	{ 0x1d0, 0xbc00ff86, 0xa4000786 },
	{ 0x1d4, 0x000000ff, 0x00000020 },
	{ 0x1d8, 0x000000ff, 0x000000ff },
	{ 0x1dc, 0x00ffffff, 0x00928170 },
	{ 0x270, 0x800f3fff, 0x800b08ff },
	{ 0x274, 0x07ff07ff, 0x07000080 },
	{ 0x280, 0xffffffc0, 0x00180000 },
	{ 0x290, 0x800f3fff, 0x800004ff },
	{ 0x294, 0x000000ff, 0x00000080 },
	{ 0x2a0, 0x800f3fff, 0x800104ff },
	{ 0x2a4, 0x000000ff, 0x00000080 },
	{ 0x2b0, 0x800f3fff, 0x800204ff },
	{ 0x2b4, 0x000000ff, 0x00000040 },
	{ 0x2c0, 0x800f3fff, 0x800304ff },
	{ 0x2c4, 0x000000ff, 0x00000040 },
	{ 0x2d0, 0x802ff04c, 0x80070040 },
	{ 0x2d4, 0x00000001, 0x00000000 },
	{ 0x2d8, 0xffff0003, 0x00ff0003 },
	{ 0x2e0, 0x07ff07ff, 0x06000040 },
	{ 0x2f8, 0x802ff04c, 0x80081040 },
	{ 0x2fc, 0x00000001, 0x00000000 },
	{ 0x300, 0xffff0003, 0x00ff0003 },
	{ 0x308, 0x07ff07ff, 0x06400040 },
	{ 0x320, 0x802ff04c, 0x80092040 },
	{ 0x324, 0x00000001, 0x00000000 },
	{ 0x328, 0xffff0003, 0x00ff0003 },
	{ 0x330, 0x07ff07ff, 0x06800040 },
	{ 0x350, 0x800f3fff, 0x800b08ff },
	{ 0x354, 0x07ff07ff, 0x076000a0 },
	{ 0x360, 0xffffffc0, 0x00180000 },
	{ 0x370, 0x800f3fff, 0x800604ff },
	{ 0x374, 0x07ff07ff, 0x06c000a0 },
};

/*
 * T6041 (M4 Pro) power-up rows, in the order the engine needs them: the
 * SCODEC-system window (aux), then the engine (core), then the AXI-to-fabric
 * wrapper. Each row replaces the whole register with the value macOS 27.0
 * writes before its first ProRes job, from an MMIO trace on a J616s.
 */
static const struct apr_tunable apr_t6041_aux_tunables[] = {
	{ 0x008, 0xffffffff, 0x00000001 },
	{ 0x00c, 0xffffffff, 0x7f7f0001 },
	{ 0x014, 0xffffffff, 0x01100110 },
	{ 0x018, 0xffffffff, 0x7f007f00 },
	{ 0x01c, 0xffffffff, 0xa000003f },
	{ 0x020, 0xffffffff, 0x001c0090 },
	{ 0x024, 0xffffffff, 0xa000003f },
	{ 0x028, 0xffffffff, 0x08000002 },
	{ 0x1000, 0xffffffff, 0x00007f00 },
	{ 0x1004, 0xffffffff, 0x00000000 },
	{ 0x1008, 0xffffffff, 0x00000000 },
	{ 0x100c, 0xffffffff, 0x00000000 },
	{ 0x1010, 0xffffffff, 0x00000000 },
	{ 0x1014, 0xffffffff, 0x00000000 },
	{ 0x1018, 0xffffffff, 0x00000000 },
	{ 0x101c, 0xffffffff, 0x00000000 },
	{ 0x1020, 0xffffffff, 0x00000000 },
	{ 0x1024, 0xffffffff, 0x00000000 },
	{ 0x1028, 0xffffffff, 0x00000000 },
	{ 0x102c, 0xffffffff, 0x00000000 },
	{ 0x1030, 0xffffffff, 0x00000000 },
	{ 0x1034, 0xffffffff, 0x00000000 },
	{ 0x1038, 0xffffffff, 0x00000000 },
	{ 0x103c, 0xffffffff, 0x00000000 },
	{ 0x1040, 0xffffffff, 0x00000000 },
	{ 0x1044, 0xffffffff, 0x00000000 },
	{ 0x1048, 0xffffffff, 0x00000000 },
	{ 0x104c, 0xffffffff, 0x00000000 },
	{ 0x1050, 0xffffffff, 0x00000000 },
	{ 0x1054, 0xffffffff, 0x00000000 },
	{ 0x1058, 0xffffffff, 0x00000000 },
	{ 0x105c, 0xffffffff, 0x00000000 },
	{ 0x1060, 0xffffffff, 0x00000000 },
	{ 0x1064, 0xffffffff, 0x00000000 },
	{ 0x1068, 0xffffffff, 0x00000000 },
	{ 0x106c, 0xffffffff, 0x00000000 },
	{ 0x1070, 0xffffffff, 0x00000000 },
	{ 0x1074, 0xffffffff, 0x00000000 },
	{ 0x1078, 0xffffffff, 0x00007f00 },
	{ 0x107c, 0xffffffff, 0x00000000 },
	{ 0x1080, 0xffffffff, 0x00000000 },
	{ 0x1084, 0xffffffff, 0x00000000 },
	{ 0x1088, 0xffffffff, 0x00000000 },
	{ 0x108c, 0xffffffff, 0x00000000 },
	{ 0x1090, 0xffffffff, 0x00000000 },
	{ 0x1094, 0xffffffff, 0x00000000 },
	{ 0x1098, 0xffffffff, 0x00000000 },
	{ 0x109c, 0xffffffff, 0x00000000 },
	{ 0x10a0, 0xffffffff, 0x00000000 },
	{ 0x10a4, 0xffffffff, 0x00000000 },
	{ 0x10a8, 0xffffffff, 0x00000000 },
	{ 0x10ac, 0xffffffff, 0x00000000 },
	{ 0x10b0, 0xffffffff, 0x00000000 },
	{ 0x10b4, 0xffffffff, 0x00000000 },
	{ 0x10b8, 0xffffffff, 0x00000000 },
	{ 0x10bc, 0xffffffff, 0x00000000 },
	{ 0x10c0, 0xffffffff, 0x00000000 },
	{ 0x10c4, 0xffffffff, 0x00000000 },
	{ 0x10c8, 0xffffffff, 0x00000000 },
	{ 0x10cc, 0xffffffff, 0x00000000 },
	{ 0x10d0, 0xffffffff, 0x00000000 },
	{ 0x10d4, 0xffffffff, 0x00000000 },
	{ 0x10d8, 0xffffffff, 0x00000000 },
	{ 0x10dc, 0xffffffff, 0x00000000 },
	{ 0x10e0, 0xffffffff, 0x00000000 },
	{ 0x10e4, 0xffffffff, 0x00000000 },
	{ 0x10e8, 0xffffffff, 0x00000000 },
	{ 0x10ec, 0xffffffff, 0x00000000 },
	{ 0x10fc, 0xffffffff, 0x00000000 },
	{ 0x1100, 0xffffffff, 0x00000000 },
	{ 0x1104, 0xffffffff, 0x00000000 },
	{ 0x1108, 0xffffffff, 0x00000000 },
	{ 0x110c, 0xffffffff, 0x00000000 },
	{ 0x1110, 0xffffffff, 0x00000000 },
	{ 0x1114, 0xffffffff, 0x00000000 },
	{ 0x1118, 0xffffffff, 0x00000000 },
	{ 0x111c, 0xffffffff, 0x00000000 },
	{ 0x1120, 0xffffffff, 0x00000000 },
	{ 0x1124, 0xffffffff, 0x00000000 },
	{ 0x1128, 0xffffffff, 0x00000000 },
	{ 0x112c, 0xffffffff, 0x00000000 },
	{ 0x1130, 0xffffffff, 0x00000000 },
};

static const struct apr_tunable apr_t6041_engine_tunables[] = {
	{ 0x008, 0xffffffff, 0x00000008 },
	{ 0x118, 0xffffffff, 0x00000600 },
	{ 0x11c, 0xffffffff, 0x00000600 },
	{ 0x148, 0xffffffff, 0x00000001 },
	{ 0x180, 0xffffffff, 0x800a0600 },
	{ 0x184, 0xffffffff, 0x00000001 },
	{ 0x188, 0xffffffff, 0x09b80038 },
	{ 0x1c0, 0xffffffff, 0x80040600 },
	{ 0x1c4, 0xffffffff, 0x00000003 },
	{ 0x1c8, 0xffffffff, 0x080000c0 },
	{ 0x200, 0xffffffff, 0x80050600 },
	{ 0x204, 0xffffffff, 0x00000002 },
	{ 0x208, 0xffffffff, 0x08c000c0 },
	{ 0x240, 0xffffffff, 0x800007ff },
	{ 0x244, 0xffffffff, 0x00000007 },
	{ 0x248, 0xffffffff, 0x00000040 },
	{ 0x280, 0xffffffff, 0x80010600 },
	{ 0x284, 0xffffffff, 0x00000006 },
	{ 0x288, 0xffffffff, 0x00000080 },
	{ 0x2c0, 0xffffffff, 0x80020600 },
	{ 0x2c4, 0xffffffff, 0x00000005 },
	{ 0x2c8, 0xffffffff, 0x00000040 },
	{ 0x300, 0xffffffff, 0x80030600 },
	{ 0x304, 0xffffffff, 0x00000004 },
	{ 0x308, 0xffffffff, 0x00000040 },
	{ 0x340, 0xffffffff, 0x81000047 },
	{ 0x344, 0xffffffff, 0x0b0a0908 },
	{ 0x348, 0xffffffff, 0x00000720 },
	{ 0x34c, 0xffffffff, 0x00000140 },
	{ 0x350, 0xffffffff, 0x00928170 },
	{ 0x380, 0xffffffff, 0x800b0a00 },
	{ 0x384, 0xffffffff, 0x0000000c },
	{ 0x388, 0xffffffff, 0x09800038 },
	{ 0x394, 0xffffffff, 0x00200000 },
	{ 0x3c0, 0xffffffff, 0x800b0a00 },
	{ 0x3c4, 0xffffffff, 0x00000019 },
	{ 0x3c8, 0xffffffff, 0x09f00010 },
	{ 0x4c0, 0xffffffff, 0x800b09ff },
	{ 0x4c4, 0xffffffff, 0x0000001b },
	{ 0x4c8, 0xffffffff, 0x09300010 },
	{ 0x500, 0xffffffff, 0x800605ff },
	{ 0x504, 0xffffffff, 0x0000000d },
	{ 0x508, 0xffffffff, 0x080000a0 },
	{ 0x540, 0xffffffff, 0x800005ff },
	{ 0x544, 0xffffffff, 0x00000011 },
	{ 0x548, 0xffffffff, 0x00000080 },
	{ 0x580, 0xffffffff, 0x800105ff },
	{ 0x584, 0xffffffff, 0x00000010 },
	{ 0x588, 0xffffffff, 0x00000080 },
	{ 0x5c0, 0xffffffff, 0x800205ff },
	{ 0x5c4, 0xffffffff, 0x0000000f },
	{ 0x5c8, 0xffffffff, 0x00000040 },
	{ 0x600, 0xffffffff, 0x800305ff },
	{ 0x604, 0xffffffff, 0x0000000e },
	{ 0x608, 0xffffffff, 0x00000040 },
	{ 0x640, 0xffffffff, 0x80070051 },
	{ 0x644, 0xffffffff, 0x00071716 },
	{ 0x648, 0xffffffff, 0x07ff0003 },
	{ 0x650, 0xffffffff, 0x06000040 },
	{ 0x680, 0xffffffff, 0x80081051 },
	{ 0x684, 0xffffffff, 0x00071514 },
	{ 0x688, 0xffffffff, 0x07ff0003 },
	{ 0x690, 0xffffffff, 0x06400040 },
	{ 0x6c0, 0xffffffff, 0x80092051 },
	{ 0x6c4, 0xffffffff, 0x00071312 },
	{ 0x6c8, 0xffffffff, 0x07ff0003 },
	{ 0x6d0, 0xffffffff, 0x06800040 },
	{ 0x700, 0xffffffff, 0x800b09ff },
	{ 0x704, 0xffffffff, 0x00000018 },
	{ 0x708, 0xffffffff, 0x08a00060 },
	{ 0x714, 0xffffffff, 0x00200000 },
	{ 0x740, 0xffffffff, 0x800b09ff },
	{ 0x744, 0xffffffff, 0x0000001a },
	{ 0x748, 0xffffffff, 0x09000010 },
	{ 0x780, 0xffffffff, 0x800b09ff },
	{ 0x784, 0xffffffff, 0x0000001c },
	{ 0x788, 0xffffffff, 0x09100010 },
	{ 0x7c0, 0xffffffff, 0x800b09ff },
	{ 0x7c4, 0xffffffff, 0x0000001d },
	{ 0x7c8, 0xffffffff, 0x09200010 },
};

static const struct apr_tunable apr_t6041_wrapper_tunables[] = {
	{ 0x000, 0xffffffff, 0x8000001d },
	{ 0x004, 0xffffffff, 0x801d0003 },
	{ 0x008, 0xffffffff, 0x801d000c },
	{ 0x00c, 0xffffffff, 0x801d0003 },
	{ 0x010, 0xffffffff, 0x80000020 },
	{ 0x014, 0xffffffff, 0x80000020 },
	{ 0x018, 0xffffffff, 0x801d0003 },
	{ 0x01c, 0xffffffff, 0x801d0003 },
	{ 0x020, 0xffffffff, 0x80000016 },
	{ 0x024, 0xffffffff, 0x8000001d },
	{ 0x028, 0xffffffff, 0x801d0003 },
	{ 0x02c, 0xffffffff, 0x801d0003 },
	{ 0x030, 0xffffffff, 0x80000020 },
	{ 0x034, 0xffffffff, 0x801d0003 },
	{ 0x038, 0xffffffff, 0x801d0003 },
	{ 0x03c, 0xffffffff, 0x80000016 },
	{ 0x040, 0xffffffff, 0x80000016 },
	{ 0x044, 0xffffffff, 0x80000016 },
	{ 0x100, 0xffffffff, 0x000000ff },
	{ 0x108, 0xffffffff, 0x00101330 },
	{ 0x304, 0xffffffff, 0x02020000 },
	{ 0x308, 0xffffffff, 0x00001681 },
	{ 0x30c, 0xffffffff, 0x00001680 },
	{ 0x310, 0xffffffff, 0x000001e0 },
	{ 0x314, 0xffffffff, 0x800001e0 },
	{ 0x318, 0xffffffff, 0x000005a0 },
	{ 0x31c, 0xffffffff, 0x800005a0 },
	{ 0x320, 0xffffffff, 0x000005a0 },
	{ 0x324, 0xffffffff, 0x800005a0 },
	{ 0x328, 0xffffffff, 0x0000003c },
	{ 0x32c, 0xffffffff, 0x8000003c },
	{ 0x330, 0xffffffff, 0x000005a0 },
	{ 0x334, 0xffffffff, 0x000005a0 },
	{ 0x338, 0xffffffff, 0x000005a0 },
	{ 0x344, 0xffffffff, 0x01000005 },
	{ 0x34c, 0xffffffff, 0x00050020 },
	{ 0x350, 0xffffffff, 0x000a0030 },
	{ 0x408, 0xffffffff, 0x00190031 },
	{ 0x40c, 0xffffffff, 0x00001681 },
	{ 0x410, 0xffffffff, 0x00001680 },
	{ 0x414, 0xffffffff, 0x000001e0 },
	{ 0x418, 0xffffffff, 0x000001e0 },
	{ 0x41c, 0xffffffff, 0x000005a0 },
	{ 0x420, 0xffffffff, 0x000005a0 },
	{ 0x424, 0xffffffff, 0x000005a0 },
	{ 0x428, 0xffffffff, 0x000005a0 },
	{ 0x42c, 0xffffffff, 0x0000003c },
	{ 0x430, 0xffffffff, 0x0000003c },
	{ 0x434, 0xffffffff, 0x000005a0 },
	{ 0x438, 0xffffffff, 0x000005a0 },
	{ 0x43c, 0xffffffff, 0x000005a0 },
	{ 0x448, 0xffffffff, 0x01000005 },
	{ 0x748, 0xffffffff, 0x00200000 },
	{ 0x848, 0xffffffff, 0x00100030 },
	{ 0x948, 0xffffffff, 0x01400100 },
	{ 0xa50, 0xffffffff, 0x01000100 },
	{ 0xc00, 0xffffffff, 0x7fffffff },
	{ 0x4008, 0xffffffff, 0x00000000 },
	{ 0x400c, 0xffffffff, 0x00000001 },
	{ 0x4014, 0xffffffff, 0x00000000 },
	{ 0x4018, 0xffffffff, 0x00000000 },
	{ 0x401c, 0xffffffff, 0x00000001 },
};

/*
 * T6050 (M5 Pro/Max) power-up rows: the SCODEC-system window (aux), the
 * SCODEC control window (ctrl), the engine (core) and the AXI-to-fabric
 * wrapper, applied in that order. Each row replaces the whole register with
 * the value macOS 27.0 writes before its first ProRes job, from an MMIO trace
 * of the four apr0 windows on a J714s. The system and engine rows sit at
 * T6041's offsets; the wrapper rows do not.
 */
static const struct apr_tunable apr_t6050_aux_tunables[] = {
	{ 0x008, 0xffffffff, 0x00000001 },
	{ 0x00c, 0xffffffff, 0x7f7f0001 },
	{ 0x014, 0xffffffff, 0x01100110 },
	{ 0x018, 0xffffffff, 0x7f007f00 },
	{ 0x01c, 0xffffffff, 0xa0000041 },
	{ 0x020, 0xffffffff, 0x001c0090 },
	{ 0x024, 0xffffffff, 0xa0000041 },
	{ 0x028, 0xffffffff, 0x08000002 },
	{ 0x1000, 0xffffffff, 0x00007f00 },
	{ 0x1004, 0xffffffff, 0x00000000 },
	{ 0x1008, 0xffffffff, 0x00000000 },
	{ 0x100c, 0xffffffff, 0x00000000 },
	{ 0x1010, 0xffffffff, 0x00000000 },
	{ 0x1014, 0xffffffff, 0x00000000 },
	{ 0x1018, 0xffffffff, 0x00000000 },
	{ 0x101c, 0xffffffff, 0x00000000 },
	{ 0x1020, 0xffffffff, 0x00000000 },
	{ 0x1024, 0xffffffff, 0x00000000 },
	{ 0x1028, 0xffffffff, 0x00000000 },
	{ 0x102c, 0xffffffff, 0x00000000 },
	{ 0x1030, 0xffffffff, 0x00000000 },
	{ 0x1034, 0xffffffff, 0x00000000 },
	{ 0x1038, 0xffffffff, 0x00000000 },
	{ 0x103c, 0xffffffff, 0x00000000 },
	{ 0x1040, 0xffffffff, 0x00000000 },
	{ 0x1044, 0xffffffff, 0x00000000 },
	{ 0x1048, 0xffffffff, 0x00000000 },
	{ 0x104c, 0xffffffff, 0x00000000 },
	{ 0x1050, 0xffffffff, 0x00000000 },
	{ 0x1054, 0xffffffff, 0x00000000 },
	{ 0x1058, 0xffffffff, 0x00000000 },
	{ 0x105c, 0xffffffff, 0x00000000 },
	{ 0x1060, 0xffffffff, 0x00000000 },
	{ 0x1064, 0xffffffff, 0x00000000 },
	{ 0x1068, 0xffffffff, 0x00000000 },
	{ 0x106c, 0xffffffff, 0x00000000 },
	{ 0x1070, 0xffffffff, 0x00000000 },
	{ 0x1074, 0xffffffff, 0x00000000 },
	{ 0x1078, 0xffffffff, 0x00007f00 },
	{ 0x107c, 0xffffffff, 0x00000000 },
	{ 0x1080, 0xffffffff, 0x00000000 },
	{ 0x1084, 0xffffffff, 0x00000000 },
	{ 0x1088, 0xffffffff, 0x00000000 },
	{ 0x108c, 0xffffffff, 0x00000000 },
	{ 0x1090, 0xffffffff, 0x00000000 },
	{ 0x1094, 0xffffffff, 0x00000000 },
	{ 0x1098, 0xffffffff, 0x00000000 },
	{ 0x109c, 0xffffffff, 0x00000000 },
	{ 0x10a0, 0xffffffff, 0x00000000 },
	{ 0x10a4, 0xffffffff, 0x00000000 },
	{ 0x10a8, 0xffffffff, 0x00000000 },
	{ 0x10ac, 0xffffffff, 0x00000000 },
	{ 0x10b0, 0xffffffff, 0x00000000 },
	{ 0x10b4, 0xffffffff, 0x00000000 },
	{ 0x10b8, 0xffffffff, 0x00000000 },
	{ 0x10bc, 0xffffffff, 0x00000000 },
	{ 0x10c0, 0xffffffff, 0x00000000 },
	{ 0x10c4, 0xffffffff, 0x00000000 },
	{ 0x10c8, 0xffffffff, 0x00000000 },
	{ 0x10cc, 0xffffffff, 0x00000000 },
	{ 0x10d0, 0xffffffff, 0x00000000 },
	{ 0x10d4, 0xffffffff, 0x00000000 },
	{ 0x10d8, 0xffffffff, 0x00000000 },
	{ 0x10dc, 0xffffffff, 0x00000000 },
	{ 0x10e0, 0xffffffff, 0x00000000 },
	{ 0x10e4, 0xffffffff, 0x00000000 },
	{ 0x10e8, 0xffffffff, 0x00000000 },
	{ 0x10ec, 0xffffffff, 0x00000000 },
	{ 0x10fc, 0xffffffff, 0x00000000 },
	{ 0x1100, 0xffffffff, 0x00000000 },
	{ 0x1104, 0xffffffff, 0x00000000 },
	{ 0x1108, 0xffffffff, 0x00000000 },
	{ 0x110c, 0xffffffff, 0x00000000 },
	{ 0x1110, 0xffffffff, 0x00000000 },
	{ 0x1114, 0xffffffff, 0x00000000 },
	{ 0x1118, 0xffffffff, 0x00000000 },
	{ 0x111c, 0xffffffff, 0x00000000 },
	{ 0x1120, 0xffffffff, 0x00000000 },
	{ 0x1124, 0xffffffff, 0x00000000 },
	{ 0x1128, 0xffffffff, 0x00000000 },
	{ 0x112c, 0xffffffff, 0x00000000 },
	{ 0x1130, 0xffffffff, 0x00000000 },
};

static const struct apr_tunable apr_t6050_engine_tunables[] = {
	{ 0x008, 0xffffffff, 0x00000008 },
	{ 0x118, 0xffffffff, 0x00000800 },
	{ 0x11c, 0xffffffff, 0x00000740 },
	{ 0x148, 0xffffffff, 0x00000001 },
	{ 0x180, 0xffffffff, 0x800a0600 },
	{ 0x184, 0xffffffff, 0x00000001 },
	{ 0x188, 0xffffffff, 0x0bb80038 },
	{ 0x1c0, 0xffffffff, 0x80040600 },
	{ 0x1c4, 0xffffffff, 0x00000003 },
	{ 0x1c8, 0xffffffff, 0x0a0000c0 },
	{ 0x200, 0xffffffff, 0x80050600 },
	{ 0x204, 0xffffffff, 0x00000002 },
	{ 0x208, 0xffffffff, 0x0ac000c0 },
	{ 0x240, 0xffffffff, 0x800007ff },
	{ 0x244, 0xffffffff, 0x00000007 },
	{ 0x248, 0xffffffff, 0x00000040 },
	{ 0x280, 0xffffffff, 0x80010600 },
	{ 0x284, 0xffffffff, 0x00000006 },
	{ 0x288, 0xffffffff, 0x00000080 },
	{ 0x2c0, 0xffffffff, 0x80020600 },
	{ 0x2c4, 0xffffffff, 0x00000005 },
	{ 0x2c8, 0xffffffff, 0x00000040 },
	{ 0x300, 0xffffffff, 0x80030600 },
	{ 0x304, 0xffffffff, 0x00000004 },
	{ 0x308, 0xffffffff, 0x00000040 },
	{ 0x340, 0xffffffff, 0x81000047 },
	{ 0x344, 0xffffffff, 0x0b0a0908 },
	{ 0x348, 0xffffffff, 0x00000820 },
	{ 0x34c, 0xffffffff, 0x00001580 },
	{ 0x350, 0xffffffff, 0x00928170 },
	{ 0x380, 0xffffffff, 0x800b0a00 },
	{ 0x384, 0xffffffff, 0x0000000c },
	{ 0x388, 0xffffffff, 0x0b800038 },
	{ 0x394, 0xffffffff, 0x00200000 },
	{ 0x3c0, 0xffffffff, 0x800b0a00 },
	{ 0x3c4, 0xffffffff, 0x00000019 },
	{ 0x3c8, 0xffffffff, 0x0bf00010 },
	{ 0x4c0, 0xffffffff, 0x800b09ff },
	{ 0x4c4, 0xffffffff, 0x0000001b },
	{ 0x4c8, 0xffffffff, 0x0b300010 },
	{ 0x500, 0xffffffff, 0x800605ff },
	{ 0x504, 0xffffffff, 0x0000000d },
	{ 0x508, 0xffffffff, 0x0a0000a0 },
	{ 0x540, 0xffffffff, 0x800005ff },
	{ 0x544, 0xffffffff, 0x00000011 },
	{ 0x548, 0xffffffff, 0x00000080 },
	{ 0x580, 0xffffffff, 0x800105ff },
	{ 0x584, 0xffffffff, 0x00000010 },
	{ 0x588, 0xffffffff, 0x00000080 },
	{ 0x5c0, 0xffffffff, 0x800205ff },
	{ 0x5c4, 0xffffffff, 0x0000000f },
	{ 0x5c8, 0xffffffff, 0x00000040 },
	{ 0x600, 0xffffffff, 0x800305ff },
	{ 0x604, 0xffffffff, 0x0000000e },
	{ 0x608, 0xffffffff, 0x00000040 },
	{ 0x640, 0xffffffff, 0x80070051 },
	{ 0x644, 0xffffffff, 0x00081716 },
	{ 0x648, 0xffffffff, 0x07ff0003 },
	{ 0x650, 0xffffffff, 0x07400040 },
	{ 0x680, 0xffffffff, 0x80081051 },
	{ 0x684, 0xffffffff, 0x00081514 },
	{ 0x688, 0xffffffff, 0x07ff0003 },
	{ 0x690, 0xffffffff, 0x07800040 },
	{ 0x6c0, 0xffffffff, 0x80092051 },
	{ 0x6c4, 0xffffffff, 0x00081312 },
	{ 0x6c8, 0xffffffff, 0x07ff0003 },
	{ 0x6d0, 0xffffffff, 0x07c00040 },
	{ 0x700, 0xffffffff, 0x800b09ff },
	{ 0x704, 0xffffffff, 0x00000018 },
	{ 0x708, 0xffffffff, 0x0aa00060 },
	{ 0x714, 0xffffffff, 0x00200000 },
	{ 0x740, 0xffffffff, 0x800b09ff },
	{ 0x744, 0xffffffff, 0x0000001a },
	{ 0x748, 0xffffffff, 0x0b000010 },
	{ 0x780, 0xffffffff, 0x800b09ff },
	{ 0x784, 0xffffffff, 0x0000001c },
	{ 0x788, 0xffffffff, 0x0b100010 },
	{ 0x7c0, 0xffffffff, 0x800b09ff },
	{ 0x7c4, 0xffffffff, 0x0000001d },
	{ 0x7c8, 0xffffffff, 0x0b200010 },
};

static const struct apr_tunable apr_t6050_wrapper_tunables[] = {
	{ 0x000, 0xffffffff, 0x80000027 },
	{ 0x004, 0xffffffff, 0x80270002 },
	{ 0x008, 0xffffffff, 0x80270009 },
	{ 0x00c, 0xffffffff, 0x80270002 },
	{ 0x010, 0xffffffff, 0x80000027 },
	{ 0x014, 0xffffffff, 0x80000027 },
	{ 0x018, 0xffffffff, 0x80270002 },
	{ 0x01c, 0xffffffff, 0x80270002 },
	{ 0x020, 0xffffffff, 0x8000001e },
	{ 0x024, 0xffffffff, 0x80000027 },
	{ 0x028, 0xffffffff, 0x80270002 },
	{ 0x02c, 0xffffffff, 0x80000027 },
	{ 0x030, 0xffffffff, 0x80270002 },
	{ 0x034, 0xffffffff, 0x80270002 },
	{ 0x038, 0xffffffff, 0x8000001e },
	{ 0x03c, 0xffffffff, 0x9000001e },
	{ 0x040, 0xffffffff, 0x90000021 },
	{ 0x100, 0xffffffff, 0x000000ff },
	{ 0x104, 0xffffffff, 0x00010002 },
	{ 0x108, 0xffffffff, 0x001013b0 },
	{ 0x300, 0xffffffff, 0x00000211 },
	{ 0x304, 0xffffffff, 0x02020000 },
	{ 0x308, 0xffffffff, 0x00000061 },
	{ 0x30c, 0xffffffff, 0x00000060 },
	{ 0x310, 0xffffffff, 0x00800008 },
	{ 0x314, 0xffffffff, 0x80800008 },
	{ 0x318, 0xffffffff, 0x00800018 },
	{ 0x31c, 0xffffffff, 0x80800018 },
	{ 0x320, 0xffffffff, 0x00800018 },
	{ 0x324, 0xffffffff, 0x80800018 },
	{ 0x328, 0xffffffff, 0x00800001 },
	{ 0x32c, 0xffffffff, 0x80800001 },
	{ 0x330, 0xffffffff, 0x00800018 },
	{ 0x334, 0xffffffff, 0x00800018 },
	{ 0x338, 0xffffffff, 0x00800018 },
	{ 0x344, 0xffffffff, 0x01000005 },
	{ 0x404, 0xffffffff, 0x00210031 },
	{ 0x408, 0xffffffff, 0x00000061 },
	{ 0x40c, 0xffffffff, 0x00000060 },
	{ 0x410, 0xffffffff, 0x80000008 },
	{ 0x414, 0xffffffff, 0x80000008 },
	{ 0x418, 0xffffffff, 0x80000018 },
	{ 0x41c, 0xffffffff, 0x80000018 },
	{ 0x420, 0xffffffff, 0x80000018 },
	{ 0x424, 0xffffffff, 0x80000018 },
	{ 0x428, 0xffffffff, 0x80000001 },
	{ 0x42c, 0xffffffff, 0x80000001 },
	{ 0x430, 0xffffffff, 0x80000018 },
	{ 0x434, 0xffffffff, 0x80000018 },
	{ 0x438, 0xffffffff, 0x80000018 },
	{ 0x444, 0xffffffff, 0x01000005 },
	{ 0x748, 0xffffffff, 0x00200000 },
	{ 0x848, 0xffffffff, 0x00100030 },
	{ 0x948, 0xffffffff, 0x01800100 },
	{ 0xa50, 0xffffffff, 0x01000100 },
	{ 0xc00, 0xffffffff, 0x7fffffff },
	{ 0xc04, 0xffffffff, 0x00000101 },
	{ 0x4008, 0xffffffff, 0x00000000 },
	{ 0x400c, 0xffffffff, 0x00000001 },
	{ 0x4014, 0xffffffff, 0x00000000 },
	{ 0x4018, 0xffffffff, 0x00000000 },
	{ 0x401c, 0xffffffff, 0x00000001 },
};

static const struct apr_tunable apr_t6050_ctrl_tunables[] = {
	{ 0x004, 0xffffffff, 0x00000001 },
	{ 0x00c, 0xffffffff, 0x00000001 },
	{ 0x010, 0xffffffff, 0x00000003 },
	{ 0x014, 0xffffffff, 0x00000000 },
	{ 0x018, 0xffffffff, 0x00000029 },
	{ 0x01c, 0xffffffff, 0x0000007f },
	{ 0x020, 0xffffffff, 0x00000000 },
	{ 0x024, 0xffffffff, 0x000000f6 },
	{ 0x028, 0xffffffff, 0x00000000 },
	{ 0x02c, 0xffffffff, 0x00001c20 },
	{ 0x034, 0xffffffff, 0x00000003 },
	{ 0x1000, 0xffffffff, 0x00000001 },
};

struct apr_stats_regs {
	u32 lo, hi, size;
};

/*
 * T8142 (M5) power-up rows, in the order the engine needs them: the
 * SCODEC-system window (aux), the SCODEC control window (ctrl), the engine
 * (core), then the AXI-to-fabric wrapper. Each row replaces the whole
 * register. They are the values the SoC needs before its first job,
 * recorded from an MMIO trace on a J815. The ADT's apr and dart-apr nodes
 * carry no tunables, and the system rows match T6041's but for one value,
 * so they belong to the SoC, not to the machine. An
 * apple/prores-t8142-init.bin replaces them.
 */
#define APR_ROW(o, v)	{ (o), 0xffffffff, (v) }

/*
 * SCODEC system: eight configuration words (T6041 writes 0xa000003f at
 * 0x01c), then the table at 0x1000-0x1130 cleared but for 0x10f0-0x10f8,
 * with its words 0 and 30 set to 0x7f00.
 */
static const struct apr_tunable apr_t8142_aux_tunables[] = {
	APR_ROW(0x008, 0x00000001),
	APR_ROW(0x00c, 0x7f7f0001),
	APR_ROW(0x014, 0x01100110),
	APR_ROW(0x018, 0x7f007f00),
	APR_ROW(0x01c, 0xa000003b),
	APR_ROW(0x020, 0x001c0090),
	APR_ROW(0x024, 0xa000003b),
	APR_ROW(0x028, 0x08000002),
	APR_ROW(0x1000, 0x00007f00),
	APR_ROW(0x1004, 0x00000000),
	APR_ROW(0x1008, 0x00000000),
	APR_ROW(0x100c, 0x00000000),
	APR_ROW(0x1010, 0x00000000),
	APR_ROW(0x1014, 0x00000000),
	APR_ROW(0x1018, 0x00000000),
	APR_ROW(0x101c, 0x00000000),
	APR_ROW(0x1020, 0x00000000),
	APR_ROW(0x1024, 0x00000000),
	APR_ROW(0x1028, 0x00000000),
	APR_ROW(0x102c, 0x00000000),
	APR_ROW(0x1030, 0x00000000),
	APR_ROW(0x1034, 0x00000000),
	APR_ROW(0x1038, 0x00000000),
	APR_ROW(0x103c, 0x00000000),
	APR_ROW(0x1040, 0x00000000),
	APR_ROW(0x1044, 0x00000000),
	APR_ROW(0x1048, 0x00000000),
	APR_ROW(0x104c, 0x00000000),
	APR_ROW(0x1050, 0x00000000),
	APR_ROW(0x1054, 0x00000000),
	APR_ROW(0x1058, 0x00000000),
	APR_ROW(0x105c, 0x00000000),
	APR_ROW(0x1060, 0x00000000),
	APR_ROW(0x1064, 0x00000000),
	APR_ROW(0x1068, 0x00000000),
	APR_ROW(0x106c, 0x00000000),
	APR_ROW(0x1070, 0x00000000),
	APR_ROW(0x1074, 0x00000000),
	APR_ROW(0x1078, 0x00007f00),
	APR_ROW(0x107c, 0x00000000),
	APR_ROW(0x1080, 0x00000000),
	APR_ROW(0x1084, 0x00000000),
	APR_ROW(0x1088, 0x00000000),
	APR_ROW(0x108c, 0x00000000),
	APR_ROW(0x1090, 0x00000000),
	APR_ROW(0x1094, 0x00000000),
	APR_ROW(0x1098, 0x00000000),
	APR_ROW(0x109c, 0x00000000),
	APR_ROW(0x10a0, 0x00000000),
	APR_ROW(0x10a4, 0x00000000),
	APR_ROW(0x10a8, 0x00000000),
	APR_ROW(0x10ac, 0x00000000),
	APR_ROW(0x10b0, 0x00000000),
	APR_ROW(0x10b4, 0x00000000),
	APR_ROW(0x10b8, 0x00000000),
	APR_ROW(0x10bc, 0x00000000),
	APR_ROW(0x10c0, 0x00000000),
	APR_ROW(0x10c4, 0x00000000),
	APR_ROW(0x10c8, 0x00000000),
	APR_ROW(0x10cc, 0x00000000),
	APR_ROW(0x10d0, 0x00000000),
	APR_ROW(0x10d4, 0x00000000),
	APR_ROW(0x10d8, 0x00000000),
	APR_ROW(0x10dc, 0x00000000),
	APR_ROW(0x10e0, 0x00000000),
	APR_ROW(0x10e4, 0x00000000),
	APR_ROW(0x10e8, 0x00000000),
	APR_ROW(0x10ec, 0x00000000),
	APR_ROW(0x10fc, 0x00000000),
	APR_ROW(0x1100, 0x00000000),
	APR_ROW(0x1104, 0x00000000),
	APR_ROW(0x1108, 0x00000000),
	APR_ROW(0x110c, 0x00000000),
	APR_ROW(0x1110, 0x00000000),
	APR_ROW(0x1114, 0x00000000),
	APR_ROW(0x1118, 0x00000000),
	APR_ROW(0x111c, 0x00000000),
	APR_ROW(0x1120, 0x00000000),
	APR_ROW(0x1124, 0x00000000),
	APR_ROW(0x1128, 0x00000000),
	APR_ROW(0x112c, 0x00000000),
	APR_ROW(0x1130, 0x00000000),
};

/* SCODEC control: eleven configuration words and one at 0x1000. */
static const struct apr_tunable apr_t8142_ctrl_tunables[] = {
	APR_ROW(0x004, 0x00000001),
	APR_ROW(0x00c, 0x00000001),
	APR_ROW(0x010, 0x00000003),
	APR_ROW(0x014, 0x00000000),
	APR_ROW(0x018, 0x00000019),
	APR_ROW(0x01c, 0x0000007f),
	APR_ROW(0x020, 0x00000000),
	APR_ROW(0x024, 0x000000fe),
	APR_ROW(0x028, 0x00000000),
	APR_ROW(0x02c, 0x00001c20),
	APR_ROW(0x034, 0x00000003),
	APR_ROW(0x1000, 0x00000001),
};

/*
 * The core rows from 0x180 come in blocks of three words at a 0x40 stride:
 * a word with bit 31 set, a small number unique to the block, and either a
 * length or start << 16 | length. Read as channels with buffers in one
 * memory, the start/length words of the blocks whose first word ends in
 * 0x600 or 0xa00 tile 0xa00-0xc00 exactly, and those ending in 0x5ff or 0x9ff
 * tile 0xa00-0xb40. The blocks at 0x380 and 0x700 hold the two statistics
 * areas' address and size registers (APR_REG_STATS0/1_*). This is the
 * layout the values show, not a documented one.
 */
#define APR_BUF(start, len)	((start) << 16 | (len))

static const struct apr_tunable apr_t8142_engine_tunables[] = {
	APR_ROW(0x008, 0x00000008),
	APR_ROW(0x118, 0x00000800),
	APR_ROW(0x11c, 0x00000740),
	APR_ROW(0x148, 0x00000001),
	APR_ROW(0x180, 0x800a0600), APR_ROW(0x184, 1), APR_ROW(0x188, APR_BUF(0xbb8, 0x038)),
	APR_ROW(0x1c0, 0x80040600), APR_ROW(0x1c4, 3), APR_ROW(0x1c8, APR_BUF(0xa00, 0x0c0)),
	APR_ROW(0x200, 0x80050600), APR_ROW(0x204, 2), APR_ROW(0x208, APR_BUF(0xac0, 0x0c0)),
	APR_ROW(0x240, 0x800007ff), APR_ROW(0x244, 7), APR_ROW(0x248, 0x40),
	APR_ROW(0x280, 0x80010600), APR_ROW(0x284, 6), APR_ROW(0x288, 0x80),
	APR_ROW(0x2c0, 0x80020600), APR_ROW(0x2c4, 5), APR_ROW(0x2c8, 0x40),
	APR_ROW(0x300, 0x80030600), APR_ROW(0x304, 4), APR_ROW(0x308, 0x40),
	/* 0x340-0x350 and 0x640-0x6d0 are not single channels. */
	APR_ROW(0x340, 0x81000047),
	APR_ROW(0x344, 0x0b0a0908),
	APR_ROW(0x348, 0x00000820),
	APR_ROW(0x34c, 0x00001580),
	APR_ROW(0x350, 0x00928170),
	APR_ROW(0x380, 0x800b0a00), APR_ROW(0x384, 12), APR_ROW(0x388, APR_BUF(0xb80, 0x038)),
	APR_ROW(APR_REG_STATS0_SIZE, APR_STATS_BYTES),
	APR_ROW(0x3c0, 0x800b0a00), APR_ROW(0x3c4, 25), APR_ROW(0x3c8, APR_BUF(0xbf0, 0x010)),
	APR_ROW(0x4c0, 0x800b09ff), APR_ROW(0x4c4, 27), APR_ROW(0x4c8, APR_BUF(0xb30, 0x010)),
	APR_ROW(0x500, 0x800605ff), APR_ROW(0x504, 13), APR_ROW(0x508, APR_BUF(0xa00, 0x0a0)),
	APR_ROW(0x540, 0x800005ff), APR_ROW(0x544, 17), APR_ROW(0x548, 0x80),
	APR_ROW(0x580, 0x800105ff), APR_ROW(0x584, 16), APR_ROW(0x588, 0x80),
	APR_ROW(0x5c0, 0x800205ff), APR_ROW(0x5c4, 15), APR_ROW(0x5c8, 0x40),
	APR_ROW(0x600, 0x800305ff), APR_ROW(0x604, 14), APR_ROW(0x608, 0x40),
	APR_ROW(0x640, 0x80070051),
	APR_ROW(0x644, 0x00081716),
	APR_ROW(0x648, 0x07ff0003),
	APR_ROW(0x650, 0x07400040),
	APR_ROW(0x680, 0x80081051),
	APR_ROW(0x684, 0x00081514),
	APR_ROW(0x688, 0x07ff0003),
	APR_ROW(0x690, 0x07800040),
	APR_ROW(0x6c0, 0x80092051),
	APR_ROW(0x6c4, 0x00081312),
	APR_ROW(0x6c8, 0x07ff0003),
	APR_ROW(0x6d0, 0x07c00040),
	APR_ROW(0x700, 0x800b09ff), APR_ROW(0x704, 24), APR_ROW(0x708, APR_BUF(0xaa0, 0x060)),
	APR_ROW(APR_REG_STATS1_SIZE, APR_STATS_BYTES),
	APR_ROW(0x740, 0x800b09ff), APR_ROW(0x744, 26), APR_ROW(0x748, APR_BUF(0xb00, 0x010)),
	APR_ROW(0x780, 0x800b09ff), APR_ROW(0x784, 28), APR_ROW(0x788, APR_BUF(0xb10, 0x010)),
	APR_ROW(0x7c0, 0x800b09ff), APR_ROW(0x7c4, 29), APR_ROW(0x7c8, APR_BUF(0xb20, 0x010)),
};

/*
 * AXI-to-fabric wrapper: one word per port at 0x000-0x040, each with bit 31
 * set, then the bridge's configuration words.
 */
static const struct apr_tunable apr_t8142_wrapper_tunables[] = {
	APR_ROW(0x000, 0x80000019),
	APR_ROW(0x004, 0x800b000b),
	APR_ROW(0x008, 0x800b000b),
	APR_ROW(0x00c, 0x800b000b),
	APR_ROW(0x010, 0x80000016),
	APR_ROW(0x014, 0x80000016),
	APR_ROW(0x018, 0x80000003),
	APR_ROW(0x01c, 0x80000003),
	APR_ROW(0x020, 0x80000010),
	APR_ROW(0x024, 0x80000010),
	APR_ROW(0x028, 0x8010000a),
	APR_ROW(0x02c, 0x8000001a),
	APR_ROW(0x030, 0x80000004),
	APR_ROW(0x034, 0x80000004),
	APR_ROW(0x038, 0x80000012),
	APR_ROW(0x03c, 0x9000000e),
	APR_ROW(0x040, 0x9000000e),
	APR_ROW(0x100, 0x000000ff),
	APR_ROW(0x104, 0x00010002),
	APR_ROW(0x108, 0x000413b0),
	APR_ROW(0x300, 0x1c000211),
	APR_ROW(0x304, 0x02020000),
	APR_ROW(0x314, 0x80000140),
	APR_ROW(0x31c, 0x800005a0),
	APR_ROW(0x324, 0x800005a0),
	APR_ROW(0x32c, 0x80000024),
	APR_ROW(0x404, 0x00100071),
	APR_ROW(0x748, 0x00200000),
	APR_ROW(0x848, 0x00100030),
	APR_ROW(0x948, 0x01400100),
	APR_ROW(0xa50, 0x00800080),
	APR_ROW(0xc00, 0x7fffffff),
	APR_ROW(0xc04, 0x00000101),
	APR_ROW(0x4008, 0x00000000),
	APR_ROW(0x400c, 0x00000000),
	APR_ROW(0x4014, 0x00000000),
	APR_ROW(0x4018, 0x00000000),
	APR_ROW(0x401c, 0x00000001),
};

/* The built-in rows have the counts a T8142 APRI file has. */
static_assert(ARRAY_SIZE(apr_t8142_aux_tunables) == APRI_SYSTEM_COUNT);
static_assert(ARRAY_SIZE(apr_t8142_ctrl_tunables) == APRI_CONTROL_COUNT);
static_assert(ARRAY_SIZE(apr_t8142_engine_tunables) == APRI_CORE_COUNT);
static_assert(ARRAY_SIZE(apr_t8142_wrapper_tunables) == APRI_T8142_WRAPPER_COUNT);

/* Selected only by the DT compatible, before mapping any resources. */
struct apr_variant {
	int (*build_desc)(const struct apr_frame *f, u8 *desc);
	struct apr_stats_regs stats[2];
	const struct apr_tunable *engine_tunables;
	size_t num_engine_tunables;
	const struct apr_tunable *wrapper_tunables;
	size_t num_wrapper_tunables;
	/* SCODEC-system rows, applied to the "aux" window before the engine. */
	const struct apr_tunable *aux_tunables;
	size_t num_aux_tunables;
	/* SCODEC control rows (T6050), applied to "ctrl" after the system rows. */
	const struct apr_tunable *ctrl_tunables;
	size_t num_ctrl_tunables;
	const char *reg_names[4];
	unsigned int num_regs;
	unsigned int num_irqs;
	u32 ring_bytes;
	u32 desc_bytes;
	/* Optional per-machine APRI file that replaces the built-in rows. */
	const char *firmware_name;
	const struct apri_layout *apri;
};

static const struct apr_variant apr_t8140 = {
	.build_desc = apr_build_desc,
	.stats = {
		{ APR_REG_STATS0_LO, APR_REG_STATS0_HI, APR_REG_STATS0_SIZE },
		{ APR_REG_STATS1_LO, APR_REG_STATS1_HI, APR_REG_STATS1_SIZE },
	},
	.engine_tunables = apr_t8140_engine_tunables,
	.num_engine_tunables = ARRAY_SIZE(apr_t8140_engine_tunables),
	.wrapper_tunables = apr_t8140_wrapper_tunables,
	.num_wrapper_tunables = ARRAY_SIZE(apr_t8140_wrapper_tunables),
	.reg_names = { "core", "wrapper" },
	.num_regs = 2,
	.num_irqs = 2,
	.ring_bytes = APR_RING_BYTES,
	.desc_bytes = APR_DESC_BYTES,
};

static const struct apr_variant apr_t6000 = {
	.build_desc = apr_build_desc_v0,
	.stats = { { 0x278, 0x27c, 0x280 }, { 0x358, 0x35c, 0x360 } },
	.engine_tunables = apr_t6000_engine_tunables,
	.num_engine_tunables = ARRAY_SIZE(apr_t6000_engine_tunables),
	.reg_names = { "core", "wrapper" },
	.num_regs = 2,
	.num_irqs = 1,
	.ring_bytes = APR_RING_BYTES,
	.desc_bytes = APR_DESC_BYTES,
};

static const struct apr_variant apr_t6041 = {
	.build_desc = apr_build_desc,
	.stats = {
		{ APR_REG_STATS0_LO, APR_REG_STATS0_HI, APR_REG_STATS0_SIZE },
		{ APR_REG_STATS1_LO, APR_REG_STATS1_HI, APR_REG_STATS1_SIZE },
	},
	.engine_tunables = apr_t6041_engine_tunables,
	.num_engine_tunables = ARRAY_SIZE(apr_t6041_engine_tunables),
	.wrapper_tunables = apr_t6041_wrapper_tunables,
	.num_wrapper_tunables = ARRAY_SIZE(apr_t6041_wrapper_tunables),
	.aux_tunables = apr_t6041_aux_tunables,
	.num_aux_tunables = ARRAY_SIZE(apr_t6041_aux_tunables),
	.reg_names = { "core", "wrapper", "aux" },
	.num_regs = 3,
	.num_irqs = 1,
	.ring_bytes = APR_RING_BYTES,
	.desc_bytes = APR_DESC_BYTES,
	.firmware_name = APRI_T6041_FIRMWARE_NAME,
	.apri = &apri_t6041,
};

/* T6041's engine and windows, with T8132's own wrapper table; no built-in rows. */
static const struct apr_variant apr_t8132 = {
	.build_desc = apr_build_desc,
	.stats = {
		{ APR_REG_STATS0_LO, APR_REG_STATS0_HI, APR_REG_STATS0_SIZE },
		{ APR_REG_STATS1_LO, APR_REG_STATS1_HI, APR_REG_STATS1_SIZE },
	},
	.reg_names = { "core", "wrapper", "aux" },
	.num_regs = 3,
	.num_irqs = 1,
	.ring_bytes = APR_RING_BYTES,
	.desc_bytes = APR_DESC_BYTES,
	.firmware_name = APRI_T8132_FIRMWARE_NAME,
	.apri = &apri_t8132,
};

/* T6041's engine plus the SCODEC control window. */
static const struct apr_variant apr_t6050 = {
	.build_desc = apr_build_desc,
	.stats = {
		{ APR_REG_STATS0_LO, APR_REG_STATS0_HI, APR_REG_STATS0_SIZE },
		{ APR_REG_STATS1_LO, APR_REG_STATS1_HI, APR_REG_STATS1_SIZE },
	},
	.engine_tunables = apr_t6050_engine_tunables,
	.num_engine_tunables = ARRAY_SIZE(apr_t6050_engine_tunables),
	.wrapper_tunables = apr_t6050_wrapper_tunables,
	.num_wrapper_tunables = ARRAY_SIZE(apr_t6050_wrapper_tunables),
	.aux_tunables = apr_t6050_aux_tunables,
	.num_aux_tunables = ARRAY_SIZE(apr_t6050_aux_tunables),
	.ctrl_tunables = apr_t6050_ctrl_tunables,
	.num_ctrl_tunables = ARRAY_SIZE(apr_t6050_ctrl_tunables),
	.reg_names = { "core", "wrapper", "aux", "ctrl" },
	.num_regs = 4,
	.num_irqs = 1,
	.ring_bytes = APR_RING_BYTES,
	.desc_bytes = APR_DESC_BYTES,
	.firmware_name = APRI_T6050_FIRMWARE_NAME,
	.apri = &apri_t6050,
};

/* T6050's engine and windows, with built-in rows (a Mac's own file replaces them). */
static const struct apr_variant apr_t8142 = {
	.build_desc = apr_build_desc,
	.stats = {
		{ APR_REG_STATS0_LO, APR_REG_STATS0_HI, APR_REG_STATS0_SIZE },
		{ APR_REG_STATS1_LO, APR_REG_STATS1_HI, APR_REG_STATS1_SIZE },
	},
	.engine_tunables = apr_t8142_engine_tunables,
	.num_engine_tunables = ARRAY_SIZE(apr_t8142_engine_tunables),
	.wrapper_tunables = apr_t8142_wrapper_tunables,
	.num_wrapper_tunables = ARRAY_SIZE(apr_t8142_wrapper_tunables),
	.aux_tunables = apr_t8142_aux_tunables,
	.num_aux_tunables = ARRAY_SIZE(apr_t8142_aux_tunables),
	.ctrl_tunables = apr_t8142_ctrl_tunables,
	.num_ctrl_tunables = ARRAY_SIZE(apr_t8142_ctrl_tunables),
	.reg_names = { "core", "wrapper", "aux", "ctrl" },
	.num_regs = 4,
	.num_irqs = 1,
	.ring_bytes = APR_RING_BYTES,
	.desc_bytes = APR_DESC_BYTES,
	.firmware_name = APRI_T8142_FIRMWARE_NAME,
	.apri = &apri_t8142,
};

struct apr_dev {
	const struct apr_variant *variant;
	struct device *dev;
	void __iomem *regs;
	void __iomem *wrapper;
	void __iomem *aux;
	void __iomem *ctrl;
	struct apr_init_data init;
	/* Rows applied at power-up, per APRI block: built-in or the file's. */
	const struct apr_tunable *tunables[APRI_MAX_BLOCKS];
	size_t num_tunables[APRI_MAX_BLOCKS];
	struct reset_control *reset;
	struct dev_pm_domain_list *pd_list;
	int irq[2];

	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex vdev_lock;	/* serialises ioctls and queue state */

	/* Descriptor ring, written by the CPU and read by the engine. */
	u8 *ring;
	dma_addr_t ring_dma;
	/* Statistics area the engine owns; size from its capacity registers. */
	void *stats;
	dma_addr_t stats_dma;
	size_t stats_size;

	spinlock_t hw_lock;	/* protects the fields below */
	struct apr_ctx *active;	/* job in the engine, or NULL */
	u32 head;		/* next free ring offset */
	bool ring_ready;	/* engine registers programmed since power-up */
	bool broken;		/* timed out with no way to reset */
	unsigned long started;	/* jiffies at submission */
	struct delayed_work watchdog;
};

struct apr_ctx {
	struct v4l2_fh fh;
	struct apr_dev *apr;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_ctrl *profile;
	struct v4l2_pix_format_mplane src_fmt;
	struct v4l2_pix_format_mplane dst_fmt;
	const struct apr_fmt *raw;
	enum apr_profile stream_profile;	/* fixed while OUTPUT streams */
	u32 sequence;
};

static struct apr_ctx *apr_file_to_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct apr_ctx, fh);
}

/* ---- hardware ---------------------------------------------------------- */

static void apr_write(struct apr_dev *apr, u32 reg, u32 val)
{
	writel_relaxed(val, apr->regs + reg);
}

static u32 apr_read(struct apr_dev *apr, u32 reg)
{
	return readl_relaxed(apr->regs + reg);
}

/*
 * Program the engine from power-up state. The ring must be installed before
 * MODE enables descriptor execution, otherwise the engine fetches from IOVA 0.
 */
static void apr_apply(void __iomem *base, const struct apr_tunable *t, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		u32 v = readl_relaxed(base + t[i].offset);

		writel_relaxed((v & ~t[i].mask) | (t[i].value & t[i].mask), base + t[i].offset);
	}
}

static void apr_hw_init(struct apr_dev *apr)
{
	const struct apr_variant *v = apr->variant;
	unsigned int t, i;

	apr_write(apr, APR_REG_MODE, APR_MODE_OFF);
	apr_write(apr, APR_REG_IRQ_ENABLE, 0);
	if (apr->aux)
		apr_apply(apr->aux, apr->tunables[APRI_SYSTEM_BLOCK],
			  apr->num_tunables[APRI_SYSTEM_BLOCK]);
	if (apr->ctrl)
		apr_apply(apr->ctrl, apr->tunables[APRI_CONTROL_BLOCK],
			  apr->num_tunables[APRI_CONTROL_BLOCK]);
	apr_apply(apr->regs, apr->tunables[APRI_CORE_BLOCK],
		  apr->num_tunables[APRI_CORE_BLOCK]);
	if (apr->wrapper)
		apr_apply(apr->wrapper, apr->tunables[APRI_WRAPPER_BLOCK],
			  apr->num_tunables[APRI_WRAPPER_BLOCK]);
	apr_write(apr, APR_REG_RING_HEAD, 0);
	apr_write(apr, APR_REG_RING_TAIL, 0);
	apr_write(apr, APR_REG_RING_SIZE, v->ring_bytes);
	apr_write(apr, APR_REG_RING_LO, lower_32_bits(apr->ring_dma));
	apr_write(apr, APR_REG_RING_HI, upper_32_bits(apr->ring_dma));
	/* Both statistics consumers share one area. */
	for (i = 0; i < ARRAY_SIZE(v->stats); i++) {
		apr_write(apr, v->stats[i].lo, lower_32_bits(apr->stats_dma) & ~0x3fU);
		apr_write(apr, v->stats[i].hi, upper_32_bits(apr->stats_dma) & 0x3ff);
	}
	apr_write(apr, APR_REG_MODE, APR_MODE_RUN);

	for (t = 0; t < APR_QTABLES; t++) {
		for (i = 0; i < 32; i++) {
			apr_write(apr, APR_REG_QUANT_LUMA(t) + 4 * i,
				  apr_qmat_reg(apr_qmat_luma[t], i));
			apr_write(apr, APR_REG_QUANT_CHROMA(t) + 4 * i,
				  apr_qmat_reg(apr_qmat_chroma[t], i));
		}
	}
	for (i = 0; i < APR_DC_SCALE_ENTRIES; i++)
		apr_write(apr, APR_REG_DC_SCALE(i), apr_dc_scale_reg(i));

	apr_write(apr, APR_REG_IRQ_STATUS, apr_read(apr, APR_REG_IRQ_STATUS));
	apr_write(apr, APR_REG_IRQ_ENABLE, APR_IRQ_DONE | APR_IRQ_ERROR);
	/* Order the setup before any descriptor publication. */
	wmb();
	apr->head = 0;
	apr->ring_ready = true;
}

static u32 apr_coded_size(u32 width, u32 height, bool is444)
{
	/* Worst case for 4444 XQ at the lowest quantiser, plus headers. */
	u64 size = (u64)ALIGN(width, 16) * ALIGN(height, 16) * (is444 ? 4 : 3) + SZ_64K;

	return min_t(u64, PAGE_ALIGN(size), SZ_256M);
}

static u8 apr_color_primaries(const struct v4l2_pix_format_mplane *f)
{
	switch (f->colorspace) {
	case V4L2_COLORSPACE_REC709:
	case V4L2_COLORSPACE_SRGB:
		return 1;
	case V4L2_COLORSPACE_SMPTE170M:
		return 6;
	case V4L2_COLORSPACE_470_SYSTEM_BG:
		return 5;
	case V4L2_COLORSPACE_BT2020:
		return 9;
	case V4L2_COLORSPACE_DCI_P3:
		return 11;
	default:
		return 2;	/* unspecified */
	}
}

static u8 apr_color_transfer(const struct v4l2_pix_format_mplane *f)
{
	switch (f->xfer_func) {
	case V4L2_XFER_FUNC_709:
		return 1;
	case V4L2_XFER_FUNC_SMPTE2084:
		return 16;
	default:
		return f->colorspace == V4L2_COLORSPACE_REC709 ? 1 : 2;
	}
}

static u8 apr_color_matrix(const struct v4l2_pix_format_mplane *f)
{
	switch (f->ycbcr_enc) {
	case V4L2_YCBCR_ENC_709:
		return 1;
	case V4L2_YCBCR_ENC_601:
		return 6;
	case V4L2_YCBCR_ENC_BT2020:
		return 9;
	default:
		return f->colorspace == V4L2_COLORSPACE_REC709 ? 1 : 2;
	}
}

static void apr_fill_frame(struct apr_ctx *ctx, struct vb2_v4l2_buffer *src,
			   struct vb2_v4l2_buffer *dst, struct apr_frame *f)
{
	const struct v4l2_pix_format_mplane *s = &ctx->src_fmt;
	u32 luma_stride = s->plane_fmt[0].bytesperline;
	dma_addr_t in = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);

	*f = (struct apr_frame){
		.width = s->width,
		.height = s->height,
		.input = ctx->raw->input,
		.profile = ctx->stream_profile,
		.luma_stride = luma_stride,
		.chroma_stride = luma_stride * (ctx->raw->is444 ? 2 : 1),
		.luma = in,
		.chroma = in + (u64)luma_stride * s->height,
		.output = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0),
		.output_size = vb2_plane_size(&dst->vb2_buf, 0),
		.primaries = apr_color_primaries(s),
		.transfer = apr_color_transfer(s),
		.matrix = apr_color_matrix(s),
	};
}

static void apr_finish_job(struct apr_dev *apr, struct apr_ctx *ctx, int err)
{
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src, *dst;
	u32 bytes = 0;

	src = v4l2_m2m_src_buf_remove(m2m);
	dst = v4l2_m2m_dst_buf_remove(m2m);
	if (WARN_ON(!src || !dst))
		goto out;

	if (!err) {
		u8 *out = vb2_plane_vaddr(&dst->vb2_buf, 0);

		/* The engine wrote the frame through the DART; see it now. */
		dma_rmb();
		err = out ? apr_check_output(out, vb2_plane_size(&dst->vb2_buf, 0),
					     &bytes) : -EIO;
		if (err)
			dev_err_ratelimited(apr->dev, "engine returned no frame\n");
	}
	vb2_set_plane_payload(&dst->vb2_buf, 0, err ? 0 : bytes);
	v4l2_m2m_buf_copy_metadata(src, dst);
	dst->sequence = ctx->sequence++;
	dst->flags |= V4L2_BUF_FLAG_KEYFRAME;
	if (v4l2_m2m_is_last_draining_src_buf(m2m, src)) {
		static const struct v4l2_event eos = { .type = V4L2_EVENT_EOS };

		dst->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_event_queue_fh(&ctx->fh, &eos);
		v4l2_m2m_mark_stopped(m2m);
	}
	v4l2_m2m_buf_done(src, err ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst, err ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
out:
	pm_runtime_mark_last_busy(apr->dev);
	pm_runtime_put_autosuspend(apr->dev);
	v4l2_m2m_job_finish(apr->m2m_dev, m2m);
}

/*
 * Claim the active job if the engine has finished it. Returns the context to
 * finish, or NULL. Called with hw_lock held.
 */
static struct apr_ctx *apr_take_done(struct apr_dev *apr, int *err)
{
	struct apr_ctx *ctx = apr->active;
	u32 status, tail;

	if (!ctx)
		return NULL;
	status = apr_read(apr, APR_REG_IRQ_STATUS);
	if (!status)
		return NULL;
	apr_write(apr, APR_REG_IRQ_STATUS, status);
	tail = apr_read(apr, APR_REG_RING_TAIL) & APR_RING_PTR_MASK;
	*err = 0;
	if (status != APR_IRQ_DONE || tail != apr->head) {
		dev_err_ratelimited(apr->dev,
				    "job failed: status %#x tail %#x head %#x st %#x/%#x\n",
				    status, tail, apr->head,
				    apr_read(apr, APR_REG_ST0), apr_read(apr, APR_REG_ST1));
		*err = -EIO;
	}
	apr->active = NULL;
	return ctx;
}

static irqreturn_t apr_irq(int irq, void *data)
{
	struct apr_dev *apr = data;
	struct apr_ctx *ctx;
	int err = 0;

	spin_lock(&apr->hw_lock);
	ctx = apr_take_done(apr, &err);
	spin_unlock(&apr->hw_lock);
	if (!ctx)
		return IRQ_NONE;
	cancel_delayed_work(&apr->watchdog);
	apr_finish_job(apr, ctx, err);
	return IRQ_HANDLED;
}

/*
 * Backstop for a lost interrupt and for hangs. A hung engine is reset
 * through its power domain and reprogrammed; without a reset line the
 * device refuses further jobs rather than reuse memory it may still own.
 */
static void apr_watchdog(struct work_struct *work)
{
	struct apr_dev *apr = container_of(work, struct apr_dev, watchdog.work);
	struct apr_ctx *ctx;
	unsigned long flags;
	int err = 0;

	spin_lock_irqsave(&apr->hw_lock, flags);
	ctx = apr_take_done(apr, &err);
	if (ctx) {
		spin_unlock_irqrestore(&apr->hw_lock, flags);
		dev_warn_once(apr->dev, "completion found without an interrupt\n");
		apr_finish_job(apr, ctx, err);
		return;
	}
	ctx = apr->active;
	if (!ctx) {
		spin_unlock_irqrestore(&apr->hw_lock, flags);
		return;
	}
	if (time_before(jiffies, apr->started + msecs_to_jiffies(APR_TIMEOUT_MS))) {
		spin_unlock_irqrestore(&apr->hw_lock, flags);
		schedule_delayed_work(&apr->watchdog, msecs_to_jiffies(APR_POLL_MS));
		return;
	}
	dev_err(apr->dev, "job timed out: head %#x tail %#x st %#x/%#x\n",
		apr->head, apr_read(apr, APR_REG_RING_TAIL),
		apr_read(apr, APR_REG_ST0), apr_read(apr, APR_REG_ST1));
	apr->active = NULL;
	apr_write(apr, APR_REG_IRQ_ENABLE, 0);
	apr_write(apr, APR_REG_MODE, APR_MODE_OFF);
	apr->ring_ready = false;
	/* No new job can start until this one is finished below. */
	spin_unlock_irqrestore(&apr->hw_lock, flags);
	if (!apr->reset || reset_control_reset(apr->reset)) {
		dev_err(apr->dev, "engine cannot be reset, refusing further jobs\n");
		spin_lock_irqsave(&apr->hw_lock, flags);
		apr->broken = true;
		spin_unlock_irqrestore(&apr->hw_lock, flags);
	}
	apr_finish_job(apr, ctx, -ETIMEDOUT);
}

static void apr_device_run(void *priv)
{
	struct apr_ctx *ctx = priv;
	struct apr_dev *apr = ctx->apr;
	struct vb2_v4l2_buffer *src, *dst;
	struct apr_frame f;
	unsigned long flags;
	void *out;
	int ret;

	src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	ret = pm_runtime_resume_and_get(apr->dev);
	if (ret < 0) {
		pm_runtime_get_noresume(apr->dev);
		apr_finish_job(apr, ctx, ret);
		return;
	}
	apr_fill_frame(ctx, src, dst, &f);
	/* Mark the output invalid so a frame the engine never wrote is caught. */
	out = vb2_plane_vaddr(&dst->vb2_buf, 0);
	if (!out) {
		apr_finish_job(apr, ctx, -EINVAL);
		return;
	}
	memset(out, 0, 8);

	spin_lock_irqsave(&apr->hw_lock, flags);
	if (apr->broken) {
		spin_unlock_irqrestore(&apr->hw_lock, flags);
		apr_finish_job(apr, ctx, -EIO);
		return;
	}
	/*
	 * The engine's ring wrap protocol is not known, so start again at the
	 * beginning of the ring instead of wrapping. The engine is idle here.
	 */
	if (!apr->ring_ready ||
	    apr->head + apr->variant->desc_bytes > apr->variant->ring_bytes)
		apr_hw_init(apr);
	ret = apr->variant->build_desc(&f, apr->ring + apr->head);
	if (ret) {
		spin_unlock_irqrestore(&apr->hw_lock, flags);
		apr_finish_job(apr, ctx, ret);
		return;
	}
	apr->head += apr->variant->desc_bytes;
	apr->active = ctx;
	apr->started = jiffies;
	/* Descriptor and buffers must be visible before the head moves. */
	dma_wmb();
	writel(apr->head, apr->regs + APR_REG_RING_HEAD);
	spin_unlock_irqrestore(&apr->hw_lock, flags);
	schedule_delayed_work(&apr->watchdog, msecs_to_jiffies(APR_POLL_MS));
}

static const struct v4l2_m2m_ops apr_m2m_ops = {
	.device_run = apr_device_run,
};

/* ---- formats ----------------------------------------------------------- */

static const struct apr_fmt *apr_find_raw(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(apr_raw_fmts); i++)
		if (apr_raw_fmts[i].fourcc == fourcc)
			return &apr_raw_fmts[i];
	return NULL;
}

/*
 * One contiguous plane: Y rows, then interleaved CbCr rows. The engine
 * reads whole 16-row macroblocks, so space for the rows that round the
 * height up follows the chroma region.
 */
static void apr_fill_raw(struct v4l2_pix_format_mplane *pix, const struct apr_fmt *fmt)
{
	u32 width = clamp_t(u32, ALIGN(pix->width, 2), APR_MIN_W, APR_MAX_W);
	u32 height = clamp_t(u32, pix->height, APR_MIN_H, APR_MAX_H);
	u32 min_stride = ALIGN(width * fmt->bytes, 64);
	u32 stride = ALIGN(pix->plane_fmt[0].bytesperline, 64);
	u32 cstride;

	/* Accept padding up to one extra row width, as other drivers do. */
	if (stride < min_stride || stride > 2 * min_stride)
		stride = min_stride;
	cstride = stride * (fmt->is444 ? 2 : 1);
	pix->pixelformat = fmt->fourcc;
	pix->width = width;
	pix->height = height;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->flags = 0;
	memset(pix->reserved, 0, sizeof(pix->reserved));
	memset(pix->plane_fmt, 0, sizeof(pix->plane_fmt));
	pix->plane_fmt[0].bytesperline = stride;
	pix->plane_fmt[0].sizeimage = (stride + cstride) * ALIGN(height, 16);
	if (pix->colorspace == V4L2_COLORSPACE_DEFAULT)
		pix->colorspace = V4L2_COLORSPACE_REC709;
	if (pix->xfer_func == V4L2_XFER_FUNC_DEFAULT)
		pix->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(pix->colorspace);
	if (pix->ycbcr_enc == V4L2_YCBCR_ENC_DEFAULT)
		pix->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix->colorspace);
	if (pix->quantization == V4L2_QUANTIZATION_DEFAULT)
		pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
}

static void apr_fill_coded(struct v4l2_pix_format_mplane *dst,
			   const struct v4l2_pix_format_mplane *src, bool is444)
{
	*dst = *src;
	dst->pixelformat = V4L2_PIX_FMT_PRORES;
	dst->num_planes = 1;
	memset(dst->plane_fmt, 0, sizeof(dst->plane_fmt));
	dst->plane_fmt[0].sizeimage = apr_coded_size(src->width, src->height, is444);
}

static void apr_reset_fmts(struct apr_ctx *ctx)
{
	ctx->raw = &apr_raw_fmts[0];
	memset(&ctx->src_fmt, 0, sizeof(ctx->src_fmt));
	ctx->src_fmt.width = 1920;
	ctx->src_fmt.height = 1080;
	apr_fill_raw(&ctx->src_fmt, ctx->raw);
	apr_fill_coded(&ctx->dst_fmt, &ctx->src_fmt, ctx->raw->is444);
}

static int apr_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, APR_NAME, sizeof(cap->driver));
	strscpy(cap->card, "Apple ProRes encoder", sizeof(cap->card));
	return 0;
}

static int apr_enum_fmt_out(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(apr_raw_fmts))
		return -EINVAL;
	f->pixelformat = apr_raw_fmts[f->index].fourcc;
	return 0;
}

static int apr_enum_fmt_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_PRORES;
	f->flags = V4L2_FMT_FLAG_COMPRESSED;
	return 0;
}

static int apr_enum_framesizes(struct file *file, void *priv,
			       struct v4l2_frmsizeenum *fs)
{
	if (fs->index || (fs->pixel_format != V4L2_PIX_FMT_PRORES &&
			  !apr_find_raw(fs->pixel_format)))
		return -EINVAL;
	fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fs->stepwise = (struct v4l2_frmsize_stepwise){
		APR_MIN_W, APR_MAX_W, 2, APR_MIN_H, APR_MAX_H, 1,
	};
	return 0;
}

static int apr_g_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.pix_mp = apr_file_to_ctx(file)->src_fmt;
	return 0;
}

static int apr_g_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.pix_mp = apr_file_to_ctx(file)->dst_fmt;
	return 0;
}

static int apr_try_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	const struct apr_fmt *fmt = apr_find_raw(f->fmt.pix_mp.pixelformat);

	apr_fill_raw(&f->fmt.pix_mp, fmt ?: &apr_raw_fmts[0]);
	return 0;
}

/* The coded size follows the raw format, as for other stateful encoders. */
static int apr_try_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct apr_ctx *ctx = apr_file_to_ctx(file);

	apr_fill_coded(&f->fmt.pix_mp, &ctx->src_fmt, ctx->raw->is444);
	return 0;
}

static int apr_s_fmt_out(struct file *file, void *priv, struct v4l2_format *f)
{
	struct apr_ctx *ctx = apr_file_to_ctx(file);
	struct v4l2_m2m_ctx *m2m = ctx->fh.m2m_ctx;

	if (vb2_is_busy(v4l2_m2m_get_src_vq(m2m)) ||
	    vb2_is_busy(v4l2_m2m_get_dst_vq(m2m)))
		return -EBUSY;
	apr_try_fmt_out(file, priv, f);
	ctx->src_fmt = f->fmt.pix_mp;
	ctx->raw = apr_find_raw(ctx->src_fmt.pixelformat);
	apr_fill_coded(&ctx->dst_fmt, &ctx->src_fmt, ctx->raw->is444);
	/* Keep the profile consistent with the chroma format. */
	if (ctx->raw->is444 != apr_profile_is_444(v4l2_ctrl_g_ctrl(ctx->profile)))
		v4l2_ctrl_s_ctrl(ctx->profile, ctx->raw->is444 ?
				   V4L2_MPEG_VIDEO_PRORES_PROFILE_4444 :
				   V4L2_MPEG_VIDEO_PRORES_PROFILE_STANDARD);
	return 0;
}

static int apr_s_fmt_cap(struct file *file, void *priv, struct v4l2_format *f)
{
	struct apr_ctx *ctx = apr_file_to_ctx(file);

	if (vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		return -EBUSY;
	apr_try_fmt_cap(file, priv, f);
	ctx->dst_fmt = f->fmt.pix_mp;
	return 0;
}

static int apr_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	if (sub->type == V4L2_EVENT_EOS)
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	return v4l2_ctrl_subscribe_event(fh, sub);
}

static const struct v4l2_ioctl_ops apr_ioctl_ops = {
	.vidioc_querycap = apr_querycap,
	.vidioc_enum_fmt_vid_out = apr_enum_fmt_out,
	.vidioc_enum_fmt_vid_cap = apr_enum_fmt_cap,
	.vidioc_enum_framesizes = apr_enum_framesizes,
	.vidioc_g_fmt_vid_out_mplane = apr_g_fmt_out,
	.vidioc_g_fmt_vid_cap_mplane = apr_g_fmt_cap,
	.vidioc_try_fmt_vid_out_mplane = apr_try_fmt_out,
	.vidioc_try_fmt_vid_cap_mplane = apr_try_fmt_cap,
	.vidioc_s_fmt_vid_out_mplane = apr_s_fmt_out,
	.vidioc_s_fmt_vid_cap_mplane = apr_s_fmt_cap,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_try_encoder_cmd = v4l2_m2m_ioctl_try_encoder_cmd,
	.vidioc_encoder_cmd = v4l2_m2m_ioctl_encoder_cmd,

	.vidioc_subscribe_event = apr_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* ---- queues ------------------------------------------------------------ */

static struct v4l2_pix_format_mplane *apr_q_fmt(struct apr_ctx *ctx, u32 type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->src_fmt : &ctx->dst_fmt;
}

static int apr_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct apr_ctx *ctx = vb2_get_drv_priv(q);
	u32 size = apr_q_fmt(ctx, q->type)->plane_fmt[0].sizeimage;

	if (*nplanes)
		return *nplanes == 1 && sizes[0] >= size ? 0 : -EINVAL;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int apr_buf_prepare(struct vb2_buffer *vb)
{
	struct apr_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	u32 size = apr_q_fmt(ctx, vb->type)->plane_fmt[0].sizeimage;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(vb->type)) {
		/* The engine always reads the whole padded surface. */
		vb2_set_plane_payload(vb, 0, size);
	} else {
		vb2_set_plane_payload(vb, 0, 0);
	}
	return 0;
}

static void apr_buf_queue(struct vb2_buffer *vb)
{
	struct apr_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (!V4L2_TYPE_IS_OUTPUT(vb->type) &&
	    vb2_is_streaming(vb->vb2_queue) &&
	    v4l2_m2m_dst_buf_is_last(ctx->fh.m2m_ctx)) {
		vb2_set_plane_payload(vb, 0, 0);
		vbuf->field = V4L2_FIELD_NONE;
		v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, vbuf);
		return;
	}
	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int apr_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct apr_ctx *ctx = vb2_get_drv_priv(q);

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		if (ctx->raw->is444 != apr_profile_is_444(v4l2_ctrl_g_ctrl(ctx->profile))) {
			struct vb2_v4l2_buffer *vbuf;

			while ((vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx)))
				v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_QUEUED);
			return -EINVAL;
		}
		ctx->stream_profile = v4l2_ctrl_g_ctrl(ctx->profile);
		v4l2_ctrl_grab(ctx->profile, true);
		ctx->sequence = 0;
	}
	v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx, q);
	return 0;
}

static void apr_stop_streaming(struct vb2_queue *q)
{
	struct apr_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		vbuf = V4L2_TYPE_IS_OUTPUT(q->type) ?
			v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vbuf)
			break;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
	}
	v4l2_m2m_update_stop_streaming_state(ctx->fh.m2m_ctx, q);
	if (V4L2_TYPE_IS_OUTPUT(q->type))
		v4l2_ctrl_grab(ctx->profile, false);
	if (V4L2_TYPE_IS_OUTPUT(q->type) && v4l2_m2m_has_stopped(ctx->fh.m2m_ctx)) {
		static const struct v4l2_event eos = { .type = V4L2_EVENT_EOS };

		v4l2_event_queue_fh(&ctx->fh, &eos);
	}
}

static const struct vb2_ops apr_qops = {
	.queue_setup = apr_queue_setup,
	.buf_prepare = apr_buf_prepare,
	.buf_queue = apr_buf_queue,
	.start_streaming = apr_start_streaming,
	.stop_streaming = apr_stop_streaming,
};

static int apr_queue_init(void *priv, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct apr_ctx *ctx = priv;
	int ret;

	src->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src->io_modes = VB2_MMAP | VB2_DMABUF;
	src->drv_priv = ctx;
	src->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src->ops = &apr_qops;
	src->mem_ops = &vb2_dma_contig_memops;
	src->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src->lock = &ctx->apr->vdev_lock;
	src->dev = ctx->apr->dev;
	ret = vb2_queue_init(src);
	if (ret)
		return ret;

	dst->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst->io_modes = VB2_MMAP | VB2_DMABUF;
	dst->drv_priv = ctx;
	dst->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst->ops = &apr_qops;
	dst->mem_ops = &vb2_dma_contig_memops;
	dst->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst->lock = &ctx->apr->vdev_lock;
	dst->dev = ctx->apr->dev;
	return vb2_queue_init(dst);
}

/* ---- controls and file operations -------------------------------------- */

/* Controls are read when a job is built; nothing to apply on change. */
static int apr_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops apr_ctrl_ops = {
	.s_ctrl = apr_s_ctrl,
};

static int apr_init_ctrls(struct apr_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->hdl;

	v4l2_ctrl_handler_init(hdl, 1);
	/* The profile must match the raw format's chroma; checked at STREAMON. */
	ctx->profile = v4l2_ctrl_new_std_menu(hdl, &apr_ctrl_ops,
					      V4L2_CID_MPEG_VIDEO_PRORES_PROFILE,
					      V4L2_MPEG_VIDEO_PRORES_PROFILE_4444_XQ, 0,
					      V4L2_MPEG_VIDEO_PRORES_PROFILE_STANDARD);
	if (hdl->error) {
		int ret = hdl->error;

		v4l2_ctrl_handler_free(hdl);
		return ret;
	}
	ctx->fh.ctrl_handler = hdl;
	return 0;
}

static int apr_open(struct file *file)
{
	struct apr_dev *apr = video_drvdata(file);
	struct apr_ctx *ctx;
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;
	ctx->apr = apr;
	apr_reset_fmts(ctx);
	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ret = apr_init_ctrls(ctx);
	if (ret)
		goto err_fh;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(apr->m2m_dev, ctx, apr_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto err_ctrls;
	}
	v4l2_fh_add(&ctx->fh, file);
	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(&ctx->hdl);
err_fh:
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return ret;
}

static int apr_release(struct file *file)
{
	struct apr_ctx *ctx = apr_file_to_ctx(file);
	struct apr_dev *apr = ctx->apr;

	mutex_lock(&apr->vdev_lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&apr->vdev_lock);
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations apr_fops = {
	.owner = THIS_MODULE,
	.open = apr_open,
	.release = apr_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* ---- power management and probe ---------------------------------------- */

static int apr_runtime_suspend(struct device *dev)
{
	struct apr_dev *apr = dev_get_drvdata(dev);
	unsigned long flags;

	/* Registers do not survive the power domain going off. */
	spin_lock_irqsave(&apr->hw_lock, flags);
	apr->ring_ready = false;
	spin_unlock_irqrestore(&apr->hw_lock, flags);
	return 0;
}

static int apr_runtime_resume(struct device *dev)
{
	return 0;
}

static int apr_suspend(struct device *dev)
{
	struct apr_dev *apr = dev_get_drvdata(dev);

	v4l2_m2m_suspend(apr->m2m_dev);
	return pm_runtime_force_suspend(dev);
}

static int apr_resume(struct device *dev)
{
	struct apr_dev *apr = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;
	v4l2_m2m_resume(apr->m2m_dev);
	return 0;
}

static const struct dev_pm_ops apr_pm_ops = {
	RUNTIME_PM_OPS(apr_runtime_suspend, apr_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(apr_suspend, apr_resume)
};

static void apr_cleanup(void *data)
{
	struct apr_dev *apr = data;

	cancel_delayed_work_sync(&apr->watchdog);
	if (apr->m2m_dev)
		v4l2_m2m_release(apr->m2m_dev);
	v4l2_device_unregister(&apr->v4l2_dev);
}

/*
 * Pick the power-up rows before touching the hardware. The built-in tables
 * are the default. A variant with a firmware name also accepts a per-machine
 * APRI file that replaces them; a file that is present but malformed fails
 * the probe rather than silently falling back.
 */
static int apr_select_tunables(struct apr_dev *apr)
{
	const struct apr_variant *v = apr->variant;
	const struct firmware *firmware;
	struct device *dev = apr->dev;
	unsigned int i;
	int ret;

	apr->tunables[APRI_SYSTEM_BLOCK] = v->aux_tunables;
	apr->num_tunables[APRI_SYSTEM_BLOCK] = v->num_aux_tunables;
	apr->tunables[APRI_CORE_BLOCK] = v->engine_tunables;
	apr->num_tunables[APRI_CORE_BLOCK] = v->num_engine_tunables;
	apr->tunables[APRI_WRAPPER_BLOCK] = v->wrapper_tunables;
	apr->num_tunables[APRI_WRAPPER_BLOCK] = v->num_wrapper_tunables;
	apr->tunables[APRI_CONTROL_BLOCK] = v->ctrl_tunables;
	apr->num_tunables[APRI_CONTROL_BLOCK] = v->num_ctrl_tunables;

	if (v->firmware_name &&
	    !firmware_request_nowarn(&firmware, v->firmware_name, dev)) {
		ret = apr_parse_init(v->apri, firmware->data, firmware->size,
				     &apr->init);
		release_firmware(firmware);
		if (ret)
			return dev_err_probe(dev, ret, "invalid %s\n",
					     v->firmware_name);
		for (i = 0; i < v->apri->blocks; i++) {
			apr->tunables[i] = apr->init.blocks[i].entries;
			apr->num_tunables[i] = apr->init.blocks[i].count;
		}
		dev_info(dev, "initialization rows from %s\n", v->firmware_name);
	}

	/*
	 * A variant whose rows come only from the optional file has nothing to
	 * program without it. The file is optional, so report its absence once
	 * at info level and leave the engine off.
	 */
	if (!apr->num_tunables[APRI_CORE_BLOCK]) {
		if (v->firmware_name) {
			dev_info(dev, "no %s and no built-in rows, engine not used\n",
				 v->firmware_name);
			return -ENODEV;
		}
		return dev_err_probe(dev, -ENOENT,
				     "no initialization rows for this engine\n");
	}

	return 0;
}

static int apr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct video_device *vdev;
	void __iomem *regs[4] = {};
	struct apr_dev *apr;
	unsigned int i;
	int ret;

	apr = devm_kzalloc(dev, sizeof(*apr), GFP_KERNEL);
	if (!apr)
		return -ENOMEM;
	apr->dev = dev;
	apr->variant = of_device_get_match_data(dev);
	if (!apr->variant)
		return -ENODEV;
	ret = apr_select_tunables(apr);
	if (ret)
		return ret;
	mutex_init(&apr->vdev_lock);
	spin_lock_init(&apr->hw_lock);
	INIT_DELAYED_WORK(&apr->watchdog, apr_watchdog);
	platform_set_drvdata(pdev, apr);

	for (i = 0; i < apr->variant->num_regs; i++) {
		regs[i] = devm_platform_ioremap_resource_byname(pdev,
							     apr->variant->reg_names[i]);
		if (IS_ERR(regs[i]))
			return PTR_ERR(regs[i]);
	}
	apr->regs = regs[0];
	apr->wrapper = regs[1];
	apr->aux = regs[2];
	apr->ctrl = regs[3];
	/*
	 * A second power domain, "perf", is the SoC performance request the
	 * engine needs for full speed. With more than one domain the core
	 * attaches none, so attach both with runtime PM links: the request is
	 * held exactly while the engine is runtime-active.
	 */
	if (of_count_phandle_with_args(dev->of_node, "power-domains",
				       "#power-domain-cells") > 1) {
		static const struct dev_pm_domain_attach_data pd_data = {
			.pd_flags = PD_FLAG_DEV_LINK_ON,
		};

		ret = devm_pm_domain_attach_list(dev, &pd_data, &apr->pd_list);
		if (ret < 0)
			return dev_err_probe(dev, ret, "failed to attach power domains\n");
	}
	apr->reset = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(apr->reset))
		return dev_err_probe(dev, PTR_ERR(apr->reset), "reset\n");

	/* Every buffer the engine touches goes through its DART. */
	if (!device_iommu_mapped(dev))
		return dev_err_probe(dev, -ENODEV, "no IOMMU\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;
	apr->ring = dmam_alloc_coherent(dev, apr->variant->ring_bytes, &apr->ring_dma,
					GFP_KERNEL);
	if (!apr->ring)
		return -ENOMEM;
	apr->stats_size = APR_STATS_BYTES;
	apr->stats = dmam_alloc_coherent(dev, apr->stats_size, &apr->stats_dma, GFP_KERNEL);
	if (!apr->stats)
		return -ENOMEM;

	for (i = 0; i < apr->variant->num_irqs; i++) {
		apr->irq[i] = platform_get_irq_optional(pdev, i);
		if (apr->irq[i] == -ENXIO && i)
			continue;
		if (apr->irq[i] < 0)
			return apr->irq[i];
		ret = devm_request_irq(dev, apr->irq[i], apr_irq, 0,
				       dev_name(dev), apr);
		if (ret)
			return ret;
	}

	ret = v4l2_device_register(dev, &apr->v4l2_dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, apr_cleanup, apr);
	if (ret)
		return ret;
	apr->m2m_dev = v4l2_m2m_init(&apr_m2m_ops);
	if (IS_ERR(apr->m2m_dev)) {
		ret = PTR_ERR(apr->m2m_dev);
		apr->m2m_dev = NULL;
		return ret;
	}

	pm_runtime_set_autosuspend_delay(dev, APR_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;
	/* The engine reports how much statistics space it writes. */
	if (max(apr_read(apr, apr->variant->stats[0].size),
		apr_read(apr, apr->variant->stats[1].size)) >
	    apr->stats_size) {
		pm_runtime_put_autosuspend(dev);
		return dev_err_probe(dev, -EINVAL, "statistics area larger than %zu bytes\n",
				     apr->stats_size);
	}
	dev_info(dev, "ProRes encoder, id %#x\n", apr_read(apr, APR_REG_ID));
	pm_runtime_put_autosuspend(dev);

	vdev = &apr->vdev;
	strscpy(vdev->name, APR_NAME, sizeof(vdev->name));
	vdev->fops = &apr_fops;
	vdev->ioctl_ops = &apr_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->lock = &apr->vdev_lock;
	vdev->v4l2_dev = &apr->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(vdev, apr);
	return video_register_device(vdev, VFL_TYPE_VIDEO, APPLE_M2M_VIDEO_NR);
}

static void apr_remove(struct platform_device *pdev)
{
	struct apr_dev *apr = platform_get_drvdata(pdev);

	video_unregister_device(&apr->vdev);
}

static const struct of_device_id apr_of_match[] = {
	{ .compatible = "apple,t6000-prores", .data = &apr_t6000 },
	{ .compatible = "apple,t6041-prores", .data = &apr_t6041 },
	{ .compatible = "apple,t6050-prores", .data = &apr_t6050 },
	{ .compatible = "apple,t8132-prores", .data = &apr_t8132 },
	{ .compatible = "apple,t8140-prores", .data = &apr_t8140 },
	{ .compatible = "apple,t8142-prores", .data = &apr_t8142 },
	{ }
};
MODULE_DEVICE_TABLE(of, apr_of_match);

static struct platform_driver apr_driver = {
	.probe = apr_probe,
	.remove = apr_remove,
	.driver = {
		.name = APR_NAME,
		.of_match_table = apr_of_match,
		.pm = pm_ptr(&apr_pm_ops),
	},
};
module_platform_driver(apr_driver);

MODULE_DESCRIPTION("Apple ProRes encoder");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(APRI_T6041_FIRMWARE_NAME);
MODULE_FIRMWARE(APRI_T6050_FIRMWARE_NAME);
MODULE_FIRMWARE(APRI_T8132_FIRMWARE_NAME);
MODULE_FIRMWARE(APRI_T8142_FIRMWARE_NAME);
