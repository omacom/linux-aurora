// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Driver for SIO coprocessor on t8103 (M1) and other Apple SoCs
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma/apple-sio.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/sleep.h>
#include <linux/suspend.h>
#include <linux/wait.h>

#include "dmaengine.h"
#include "virt-dma.h"

#define NCHANNELS_MAX	0x80

#define REG_CPU_CONTROL	0x44
#define CPU_CONTROL_RUN BIT(4)

#define SIOMSG_DATA	GENMASK(63, 32)
#define SIOMSG_TYPE	GENMASK(23, 16)
#define SIOMSG_PARAM	GENMASK(31, 24)
#define SIOMSG_TAG	GENMASK(13, 8)
#define SIOMSG_EP	GENMASK(7, 0)

#define EP_SIO		0x20

#define MSG_START	0x2
#define MSG_SETUP	0x3
#define MSG_CONFIGURE	0x5
#define MSG_ISSUE	0x6
#define MSG_TERMINATE	0x8
#define MSG_ACK		0x65
#define MSG_NACK	0x66
#define MSG_STARTED	0x67
#define MSG_REPORT	0x68

#define SIO_CALL_TIMEOUT_MS	100
#define SIO_SHMEM_SIZE		0x1000
#define SIO_NO_DESC_SLOTS	64

/* Protocol 9 publishes entry counts, rather than allocation sizes. */
#define SIO_V9_SEGMENTS		0xaaa
#define SIO_V9_SEGMENT_SIZE	12
#define SIO_V9_RESPONSES		0x1b80
#define SIO_V9_RESPONSE_SIZE	16

/*
 * There are two kinds of 'transaction descriptors' in play here.
 *
 * There's the struct sio_tx, and the struct dma_async_tx_descriptor embedded
 * inside, which jointly represent a transaction to the dmaengine subsystem.
 * At this time we only support those transactions to be cyclic.
 *
 * Then there are the coprocessor descriptors, which is what the coprocessor
 * knows and understands. These don't seem to have a cyclic regime, so we can't
 * map the dmaengine transaction on an exact coprocessor counterpart. Instead
 * we continually queue up many coprocessor descriptors to implement a cyclic
 * transaction.
 *
 * The number below is the maximum of how far ahead (how many) coprocessor
 * descriptors we should be queuing up, per channel, for a cyclic transaction.
 * Basically it's a made-up number.
 */
#define SIO_MAX_NINFLIGHT	4

struct sio_coproc_desc {
	u32 pad1;
	u32 flag;
	u64 unk;
	u64 iova;
	u64 size;
	u64 pad2;
	u64 pad3;
} __packed;
static_assert(sizeof(struct sio_coproc_desc) == 48);
static_assert(56 + SIO_NO_DESC_SLOTS * sizeof(struct sio_coproc_desc) <=
	      SIO_V9_SEGMENTS * SIO_V9_SEGMENT_SIZE);

struct sio_shmem_chan_config {
	u32 datashape;
	u32 timeout;
	u32 fifo;
	u32 threshold;
	u32 limit;
} __packed;

struct sio_data;
struct sio_tx;

struct sio_chan {
	unsigned int no;
	struct sio_data *host;
	struct virt_dma_chan vc;
	struct work_struct terminate_wq;

	bool configured;
	struct sio_shmem_chan_config cfg;

	struct sio_tx *current_tx;

	/*
	 * Until a client sets an idle transfer, the channel keeps the old
	 * accounting, charging ACKs and REPORTs to the current transfer; its
	 * ISSUEs not yet acknowledged are only counted.
	 */
	unsigned int pending;

	/*
	 * The ring accounting, from the first idle transfer until the channel
	 * is quiet again (sio_ring_leave_locked()): the client's idle transfer
	 * (struct apple_sio_idle), issued while no other transfer is;
	 * transfers ended while it is set, until their issued descriptors
	 * completed; and the owners of the issued descriptors in issue order,
	 * which is the order of their reports. The last one may still wait for
	 * its ISSUE to be acknowledged.
	 */
	bool ring;
	struct sio_tx *idle;
	struct list_head retired;
	wait_queue_head_t drained;
	struct sio_tx *issued[SIO_MAX_NINFLIGHT];
	unsigned int issued_head;
	unsigned int nissued;
	bool issuing;
	/* Nothing is issued between a TERMINATE and the ring's reset. */
	bool stopping;
	/*
	 * Ended transfers with nothing in flight, freed once no completion
	 * callback can still be running (sio_free_dead()).
	 */
	struct list_head dead;
};

#define SIO_NTAGS		16

typedef void (*sio_ack_callback)(struct sio_chan *, void *, bool);

struct sio_data;
struct sio_dma_arrays;

struct sio_hw_data {
	u32 protocol;		/* the only protocol STARTED may report, or 0 */
	u32 fifo_threshold;
	u32 cfg_word4;		/* configuration word 4 */
	bool cfg_six_words;	/* the configuration ends in a zero sixth word */
	bool clear_period_desc;
	/*
	 * Only run the firmware, so that it can be stopped for a system sleep. Nothing
	 * uses its DMA here, and its service parameters haven't been traced.
	 */
	bool firmware_only;
	/* Without the boot tables for DMA, still run the firmware so a sleep can stop it. */
	bool firmware_without_tables;
	/* macOS idles this firmware in the sleep's last step: leave it running until then. */
	bool stop_at_commit;
	/* Stopped by the sleep's commit, not in suspend_late */
	bool commit_stop;
	/* DMA arrays owned by the driver, if not protocol 9's */
	const struct sio_dma_arrays *arrays;
	int (*check_boot_tables)(struct device *dev);
	/* The boot loader's tables in this firmware's order, if not as listed */
	int (*send_dt_params)(struct sio_data *sio);
};

struct sio_data {
	const struct sio_hw_data *hw;
	void __iomem *base;
	struct dma_device dma;
	struct device *dev;
	struct apple_rtkit *rtk;
	/* The firmware is stopped for a system sleep. */
	bool asleep;
	bool firmware_only;
	bool commit_registered;
	bool commit_used;
	u32 protocol;
	/* The channel configuration's threshold and limit words */
	u32 fifo_threshold;
	u32 cfg_word4;
	void *shmem;
	dma_addr_t shmem_iova;
	void *responses;
	dma_addr_t responses_iova;
	const struct sio_dma_arrays *arrays;	/* when the driver owns them */
	struct sio_coproc_desc *shmem_desc_base;
	unsigned long *desc_allocated;

	struct sio_tagdata {
		DECLARE_BITMAP(allocated, SIO_NTAGS);
		int last_tag;

		struct completion completions[SIO_NTAGS];
		bool atomic[SIO_NTAGS];
		bool acked[SIO_NTAGS];

		sio_ack_callback ack_callback[SIO_NTAGS];
		void *cookie[SIO_NTAGS];
	} tags;

	int nchannels;
	struct sio_chan channels[];
};

struct sio_tx {
	struct virt_dma_desc vd;
	struct completion done;

	bool terminated;
	bool retired;
	size_t period_len;
	int nperiods;
	int ninflight;
	int next;

	struct sio_coproc_desc *siodesc[];
};

static int sio_send_siomsg(struct sio_data *sio, u64 msg);
static int sio_send_siomsg_atomic(struct sio_data *sio, u64 msg,
				  sio_ack_callback ack_callback,
				  void *cookie);
static int sio_call(struct sio_data *sio, u64 msg);
static int sio_set_param(struct sio_data *sio, u32 key, u32 val);

/*
 * Boot tables of the T6030 SIO firmware, protocol 10. The boot loader builds them from the ADT
 * and passes each one as two apple,sio-firmware-params: the table's IOVA page under its key and
 * its size in bytes under key + 1. Protocol 10 takes six 36-byte DMA shim records and nine device
 * types. A loader without protocol-10 support builds five 32-byte records and eight types, which
 * this firmware cannot use, so the SIO is not started with them.
 */
#define SIO_T6030_PARAMS_MAX	64	/* key and value pairs */
#define SIO_T6030_WINDOWS_MAX	8
#define SIO_T6030_IOVA_LIMIT	BIT_ULL(42)
/*
 * A parameter goes to the firmware as an endpoint byte (key >> 8) and a
 * parameter byte (key & 0xff), so a wider key would alias another one.
 */
#define SIO_T6030_KEY_MAX	0xffff

struct sio_t6030_window {
	u64 start;
	u64 size;
};

static const struct {
	u32 key;
	u32 size;
} sio_t6030_tables[] = {
	{ 0x1a, 4 * 16 },	/* map ranges */
	{ 0x1c, 9 * 8 },	/* device types */
	{ 0x22, 6 * 36 },	/* DMA shims */
};

/*
 * 0 when the parameters hold each protocol-10 table once, at its exact size, entirely inside one
 * of the windows the boot loader mapped for the SIO; -EINVAL otherwise.
 */
static int sio_t6030_check_params(const u32 *params, unsigned int nparams,
				  const struct sio_t6030_window *windows, unsigned int nwindows)
{
	unsigned int i, j, t;

	if (!nparams || nparams % 2 || nparams > 2 * SIO_T6030_PARAMS_MAX)
		return -EINVAL;
	for (i = 0; i < nparams; i += 2)
		if (params[i] > SIO_T6030_KEY_MAX)
			return -EINVAL;
	for (i = 0; i < nparams; i += 2)
		for (j = i + 2; j < nparams; j += 2)
			if (params[i] == params[j])
				return -EINVAL;

	for (t = 0; t < ARRAY_SIZE(sio_t6030_tables); t++) {
		bool have_page = false, have_size = false, inside = false;
		u64 start = 0, size = 0, end;

		for (i = 0; i < nparams; i += 2) {
			if (params[i] == sio_t6030_tables[t].key) {
				start = (u64)params[i + 1] << 12;
				have_page = true;
			} else if (params[i] == sio_t6030_tables[t].key + 1) {
				size = params[i + 1];
				have_size = true;
			}
		}
		if (!have_page || !have_size || size != sio_t6030_tables[t].size)
			return -EINVAL;
		if (check_add_overflow(start, size, &end) || end > SIO_T6030_IOVA_LIMIT)
			return -EINVAL;
		for (j = 0; j < nwindows && !inside; j++) {
			u64 window_end;

			if (check_add_overflow(windows[j].start, windows[j].size, &window_end))
				continue;
			inside = start >= windows[j].start && end <= window_end;
		}
		if (!inside)
			return -EINVAL;
	}
	return 0;
}

/* The IOVA windows of this device's memory regions, from their iommu-addresses */
static int sio_t6030_windows(struct device_node *np, struct sio_t6030_window *windows,
			     unsigned int max)
{
	struct of_phandle_iterator it;
	unsigned int n = 0;
	int err;

	of_for_each_phandle(&it, err, np, "memory-region", NULL, 0) {
		const __be32 *maps, *end;
		int size;

		maps = of_get_property(it.node, "iommu-addresses", &size);
		if (!maps)
			continue;
		if (size % sizeof(*maps)) {
			of_node_put(it.node);
			return -EINVAL;
		}
		end = maps + size / sizeof(*maps);
		while (maps < end) {
			struct device_node *owner __free(device_node) =
				of_find_node_by_phandle(be32_to_cpup(maps++));
			phys_addr_t iova;
			size_t length;

			/* An entry whose owner does not resolve is malformed, as a short one is */
			if (!owner) {
				of_node_put(it.node);
				return -EINVAL;
			}
			maps = of_translate_dma_region_checked(owner, maps, end - maps, &iova,
							       &length);
			if (!maps) {
				of_node_put(it.node);
				return -EINVAL;
			}
			if (owner != np)
				continue;
			if (n == max) {
				of_node_put(it.node);
				return -EINVAL;
			}
			windows[n].start = iova;
			windows[n].size = length;
			n++;
		}
	}
	return err == -ENOENT ? n : err;
}

/* 0 when the node's boot tables are protocol-10 tables inside its mapped windows */
static int sio_t6030_check_node(struct device_node *np)
{
	struct sio_t6030_window windows[SIO_T6030_WINDOWS_MAX];
	u32 params[2 * SIO_T6030_PARAMS_MAX];
	int nparams, nwindows;

	nparams = of_property_count_u32_elems(np, "apple,sio-firmware-params");
	if (nparams <= 0 || nparams > ARRAY_SIZE(params) ||
	    of_property_read_u32_array(np, "apple,sio-firmware-params", params, nparams))
		return -EINVAL;
	nwindows = sio_t6030_windows(np, windows, ARRAY_SIZE(windows));
	if (nwindows < 0)
		return nwindows;
	return sio_t6030_check_params(params, nparams, windows, nwindows);
}

static int sio_t6030_check_boot_tables(struct device *dev)
{
	if (sio_t6030_check_node(dev->of_node)) {
		dev_err(dev, "boot tables are not for SIO protocol 10; leaving the SIO stopped\n");
		return -ENODEV;
	}
	return 0;
}

/*
 * The T6041 (M4 Pro/Max) SIO firmware. It reports protocol 10 as T6030's
 * does, but its boot is a different contract, so the protocol number does
 * not select it; the compatible does:
 *
 * - the control tables go in this order, a table's address (IOVA page)
 *   then its size in bytes, which also completes it. Optional tables the
 *   ADT does not describe are left out whole. Key 0x2a, after the register
 *   tables, is a scalar: 0 leaves the SIO CPU's power gating off. The shim
 *   size (0x23) comes before the device types and lets the firmware set up
 *   its shims.
 * - the segment array (keys 1, 2) and the response array (0x0b, 0x0c) are
 *   published last, as entry counts: 0xaaa segments of 12 bytes and 0x1cc0
 *   responses of 16 bytes (one per endpoint and tag). The ACK of 0x0c
 *   brings up the DMA service. The driver owns both arrays.
 * - a channel configuration is six words: data shape, timeout, FIFO size,
 *   refill threshold, the host's default timeout (5) and a zero word.
 *
 * The boot loader builds the control tables from the ADT and maps them
 * into the windows it lists in memory-region.
 */
#define SIO_T6041_SEGMENTS		0xaaa
#define SIO_T6041_SEGMENT_SIZE		12
#define SIO_T6041_RESPONSES		0x1cc0
#define SIO_T6041_RESPONSE_SIZE		16
#define SIO_T6041_KEY_CPU_POWER_GATE	0x2a
#define SIO_T6041_KEY_TUNABLES		0x1e

enum sio_t6041_kind {
	SIO_T6041_REQUIRED,	/* an address and its exact size */
	SIO_T6041_OPTIONAL,	/* both or neither, any non-zero size */
	SIO_T6041_SCALAR,	/* the driver's own value, never the loader's */
};

static const struct sio_t6041_param {
	u16 key;
	u8 kind;
	u32 value;		/* the size of a required table, or a scalar */
} sio_t6041_params[] = {
	{ 0x0f, SIO_T6041_REQUIRED, 0x1e000 },	/* panic buffer */
	{ 0x1a, SIO_T6041_REQUIRED, 4 * 16 },	/* map ranges */
	{ SIO_T6041_KEY_TUNABLES, SIO_T6041_REQUIRED, 0x30 },	/* ASC-wrapper tunables */
	{ 0x20, SIO_T6041_OPTIONAL },		/* miscellaneous tunables */
	{ 0x24, SIO_T6041_OPTIONAL },		/* power-state registers */
	{ 0x28, SIO_T6041_OPTIONAL },		/* SIO registers */
	{ SIO_T6041_KEY_CPU_POWER_GATE, SIO_T6041_SCALAR, 0 },
	{ 0x22, SIO_T6041_REQUIRED, 6 * 36 },	/* DMA shims */
	{ 0x1c, SIO_T6041_REQUIRED, 9 * 8 },	/* device types */
	{ 0x30d, SIO_T6041_REQUIRED, 0x4000 },	/* performance endpoint */
};

/* Where a loader key goes in sio_t6041_params, or -1 */
static int sio_t6041_param_index(u32 key, bool *size)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sio_t6041_params); i++) {
		const struct sio_t6041_param *p = &sio_t6041_params[i];

		if (p->kind == SIO_T6041_SCALAR)
			continue;
		if (key == p->key || key == p->key + 1) {
			*size = key == p->key + 1;
			return i;
		}
	}
	return -1;
}

/*
 * Collect the loader's T6041 control tables into addr[]/size[] (indexed as
 * sio_t6041_params), refusing anything not exactly the contract above: an
 * unknown or repeated key, a scalar or a DMA array from the loader, a
 * required table missing or of another size, half an optional table, or a
 * table outside every window the loader mapped for this SIO.
 */
static int sio_t6041_collect(const u32 *params, unsigned int nparams,
			     const struct sio_t6030_window *windows, unsigned int nwindows,
			     u32 *addr, u32 *size, bool no_tunables)
{
	unsigned long seen = 0;
	unsigned int i, j;

	static_assert(2 * ARRAY_SIZE(sio_t6041_params) <= BITS_PER_LONG);
	if (!nparams || nparams % 2 || nparams > 2 * SIO_T6030_PARAMS_MAX)
		return -EINVAL;
	for (i = 0; i < nparams; i += 2) {
		bool is_size;
		int idx = sio_t6041_param_index(params[i], &is_size);
		unsigned int bit;

		if (idx < 0)
			return -EINVAL;
		bit = 2 * idx + is_size;
		if (seen & BIT(bit))
			return -EINVAL;
		seen |= BIT(bit);
		(is_size ? size : addr)[idx] = params[i + 1];
	}

	for (i = 0; i < ARRAY_SIZE(sio_t6041_params); i++) {
		const struct sio_t6041_param *p = &sio_t6041_params[i];
		bool have_addr = seen & BIT(2 * i), have_size = seen & BIT(2 * i + 1);
		bool inside = false;
		u64 start, end;

		if (p->kind == SIO_T6041_SCALAR)
			continue;
		if (have_addr != have_size)
			return -EINVAL;
		if (!have_addr) {
			if (p->kind == SIO_T6041_REQUIRED &&
			    !(no_tunables && p->key == SIO_T6041_KEY_TUNABLES))
				return -EINVAL;
			continue;
		}
		if (!size[i] || (p->kind == SIO_T6041_REQUIRED && size[i] != p->value))
			return -EINVAL;
		start = (u64)addr[i] << 12;
		if (check_add_overflow(start, (u64)size[i], &end) || end > SIO_T6030_IOVA_LIMIT)
			return -EINVAL;
		for (j = 0; j < nwindows && !inside; j++) {
			u64 window_end;

			if (check_add_overflow(windows[j].start, windows[j].size, &window_end))
				continue;
			inside = start >= windows[j].start && end <= window_end;
		}
		if (!inside)
			return -EINVAL;
	}
	return 0;
}

static int sio_t6041_read_params(struct device_node *np, u32 *params, unsigned int max)
{
	int nparams = of_property_count_u32_elems(np, "apple,sio-firmware-params");

	if (nparams <= 0 || nparams > max ||
	    of_property_read_u32_array(np, "apple,sio-firmware-params", params, nparams))
		return -EINVAL;
	return nparams;
}

static int sio_t6041_check_node(struct device_node *np, u32 *addr, u32 *size)
{
	struct sio_t6030_window windows[SIO_T6030_WINDOWS_MAX];
	u32 params[2 * SIO_T6030_PARAMS_MAX];
	int nparams, nwindows;

	nparams = sio_t6041_read_params(np, params, ARRAY_SIZE(params));
	if (nparams < 0)
		return nparams;
	nwindows = sio_t6030_windows(np, windows, ARRAY_SIZE(windows));
	if (nwindows < 0)
		return nwindows;
	/* T6050's ADT has no ASC-wrapper tunables. */
	return sio_t6041_collect(params, nparams, windows, nwindows, addr, size,
				 of_device_is_compatible(np, "apple,t6050-sio"));
}

static int sio_t6041_check_boot_tables(struct device *dev)
{
	u32 addr[ARRAY_SIZE(sio_t6041_params)] = {};
	u32 size[ARRAY_SIZE(sio_t6041_params)] = {};

	if (sio_t6041_check_node(dev->of_node, addr, size)) {
		/* T8140 still runs its firmware, and says so once it does. */
		if (!of_device_is_compatible(dev->of_node, "apple,t8140-sio"))
			dev_err(dev, "boot tables are not T6041 SIO tables; leaving the SIO stopped\n");
		return -ENODEV;
	}
	return 0;
}

/* Publish the collected control tables, and the scalar, in firmware order. */
static int sio_t6041_send_params(struct sio_data *sio, const u32 *addr, const u32 *size,
				 int (*set_param)(struct sio_data *, u32, u32))
{
	unsigned int i;
	int err;

	for (i = 0; i < ARRAY_SIZE(sio_t6041_params); i++) {
		const struct sio_t6041_param *p = &sio_t6041_params[i];

		if (p->kind == SIO_T6041_SCALAR) {
			err = set_param(sio, p->key, p->value);
			if (err)
				return err;
			continue;
		}
		if (!size[i])
			continue;	/* an optional table the ADT does not have */
		err = set_param(sio, p->key, addr[i]);
		if (!err)
			err = set_param(sio, p->key + 1, size[i]);
		if (err)
			return err;
	}
	return 0;
}

static int sio_t6041_send_dt_params(struct sio_data *sio)
{
	u32 addr[ARRAY_SIZE(sio_t6041_params)] = {};
	u32 size[ARRAY_SIZE(sio_t6041_params)] = {};
	int err;

	/* Checked before the boot already; the property cannot have changed. */
	err = sio_t6041_check_node(sio->dev->of_node, addr, size);
	if (err)
		return err;
	return sio_t6041_send_params(sio, addr, size, sio_set_param);
}

/* The DMA arrays a protocol publishes as entry counts, after its control tables. */
struct sio_dma_arrays {
	u32 segments;
	u32 segment_size;
	u32 responses;
	u32 response_size;
};

static const struct sio_dma_arrays sio_v9_arrays = {
	.segments = SIO_V9_SEGMENTS,
	.segment_size = SIO_V9_SEGMENT_SIZE,
	.responses = SIO_V9_RESPONSES,
	.response_size = SIO_V9_RESPONSE_SIZE,
};

static const struct sio_dma_arrays sio_t6041_arrays = {
	.segments = SIO_T6041_SEGMENTS,
	.segment_size = SIO_T6041_SEGMENT_SIZE,
	.responses = SIO_T6041_RESPONSES,
	.response_size = SIO_T6041_RESPONSE_SIZE,
};

static_assert(56 + SIO_NO_DESC_SLOTS * sizeof(struct sio_coproc_desc) <=
	      SIO_T6041_SEGMENTS * SIO_T6041_SEGMENT_SIZE);

static const struct sio_hw_data sio_hw_generic = {
	.fifo_threshold = 0x800,
	.cfg_word4 = 0x800,
};

static const struct sio_hw_data sio_hw_t6030 = {
	.fifo_threshold = 0x600,
	.cfg_word4 = 0x800,
	.clear_period_desc = true,
	.check_boot_tables = sio_t6030_check_boot_tables,
};

/*
 * macOS 26.6 configures a T6041 DisplayPort audio channel with
 * [2, 0, 0x800, 0x600, 0x800, 0]: the limit is 0x800, as on earlier SoCs;
 * the 5 of the host's mailbox call timeout is not part of the record.
 */
static const struct sio_hw_data sio_hw_t6041 = {
	.protocol = 10,
	.fifo_threshold = 0x600,
	.cfg_word4 = 0x800,
	.cfg_six_words = true,
	.clear_period_desc = true,
	.arrays = &sio_t6041_arrays,
	.check_boot_tables = sio_t6041_check_boot_tables,
	.send_dt_params = sio_t6041_send_dt_params,
};

/*
 * T8140 (MacBook Neo) firmware takes the T6041 boot tables (its ADT
 * describes them alike); its DisplayPort audio channels refill at 0x800
 * (ADT dp-audioN dma-channels): [2, 0, 0x800, 0x800, 0x800, 0].
 */
static const struct sio_hw_data sio_hw_t8140 = {
	.firmware_without_tables = true,
	.stop_at_commit = true,
	.protocol = 10,
	.fifo_threshold = 0x800,
	.cfg_word4 = 0x800,
	.cfg_six_words = true,
	.clear_period_desc = true,
	.arrays = &sio_t6041_arrays,
	.check_boot_tables = sio_t6041_check_boot_tables,
	.send_dt_params = sio_t6041_send_dt_params,
};

/*
 * T6041's, less the tunables (sio_t6041_check_node), and stopped by the sleep's commit. Its
 * channel words come from the device tree, else they are the generic ones.
 */
static const struct sio_hw_data sio_hw_t6050 = {
	.protocol = 10,
	.cfg_six_words = true,
	.clear_period_desc = true,
	.commit_stop = true,
	.arrays = &sio_t6041_arrays,
	.check_boot_tables = sio_t6041_check_boot_tables,
	.send_dt_params = sio_t6041_send_dt_params,
};

/*
 * The T6050 description is in force on the board it has run on. On another T6050 board the
 * node is taken as before: by its generic compatible.
 */
static bool sio_t6050_board(void)
{
	return of_machine_is_compatible("apple,j714s");
}

static const struct of_device_id sio_of_match[] = {
	{ .compatible = "apple,t6050-sio", .data = &sio_hw_t6050 },
	{ .compatible = "apple,t6041-sio", .data = &sio_hw_t6041 },
	{ .compatible = "apple,t8140-sio", .data = &sio_hw_t8140 },
	{ .compatible = "apple,t6030-sio", .data = &sio_hw_t6030 },
	{ .compatible = "apple,sio", .data = &sio_hw_generic },
	{ }
};
MODULE_DEVICE_TABLE(of, sio_of_match);

static struct sio_chan *to_sio_chan(struct dma_chan *chan)
{
	return container_of(chan, struct sio_chan, vc.chan);
}

static struct sio_tx *to_sio_tx(struct dma_async_tx_descriptor *tx)
{
	return container_of(tx, struct sio_tx, vd.tx);
}

static int sio_alloc_tag(struct sio_data *sio)
{
	struct sio_tagdata *tags = &sio->tags;
	int tag, i;

	/*
	 * Because tag number 0 is special, the usable tag range
	 * is 1...(SIO_NTAGS - 1). So, to pick the next usable tag,
	 * we do modulo (SIO_NTAGS - 1) *then* plus one.
	 */

#define SIO_USABLE_TAGS (SIO_NTAGS - 1)
	tag = (READ_ONCE(tags->last_tag) % SIO_USABLE_TAGS) + 1;

	for (i = 0; i < SIO_USABLE_TAGS; i++) {
		if (!test_and_set_bit(tag, tags->allocated))
			break;

		tag = (tag % SIO_USABLE_TAGS) + 1;
	}

	WRITE_ONCE(tags->last_tag, tag);

	if (i < SIO_USABLE_TAGS)
		return tag;
	else
		return -EBUSY;
#undef SIO_USABLE_TAGS
}

static void sio_free_tag(struct sio_data *sio, int tag)
{
	struct sio_tagdata *tags = &sio->tags;

	if (WARN_ON(tag >= SIO_NTAGS))
		return;

	tags->atomic[tag] = false;
	tags->ack_callback[tag] = NULL;

	WARN_ON(!test_and_clear_bit(tag, tags->allocated));
}

static void sio_set_tag_atomic(struct sio_data *sio, int tag,
			       sio_ack_callback ack_callback,
			       void *cookie)
{
	struct sio_tagdata *tags = &sio->tags;

	tags->atomic[tag] = true;
	tags->ack_callback[tag] = ack_callback;
	tags->cookie[tag] = cookie;
}

static struct sio_coproc_desc *sio_alloc_desc(struct sio_data *sio)
{
	int i;

	for (i = 0; i < SIO_NO_DESC_SLOTS; i++)
		if (!test_and_set_bit(i, sio->desc_allocated))
			return sio->shmem_desc_base + i;

	return NULL;
}

static void sio_free_desc(struct sio_data *sio, struct sio_coproc_desc *desc)
{
	clear_bit(desc - sio->shmem_desc_base, sio->desc_allocated);
}

static int sio_coproc_desc_slot(struct sio_data *sio, struct sio_coproc_desc *desc)
{
	return (desc - sio->shmem_desc_base) * 4;
}

static enum dma_transfer_direction sio_chan_direction(int channo)
{
	/* Channel directions are fixed based on channel number */
	return (channo & 1) ? DMA_DEV_TO_MEM : DMA_MEM_TO_DEV;
}

static void sio_tx_free(struct virt_dma_desc *vd)
{
	struct sio_data *sio = to_sio_chan(vd->tx.chan)->host;
	struct sio_tx *siotx = to_sio_tx(&vd->tx);
	int i;

	for (i = 0; i < siotx->nperiods; i++)
		if (siotx->siodesc[i])
			sio_free_desc(sio, siotx->siodesc[i]);
	kfree(siotx);
}

static struct sio_tx *sio_alloc_tx(struct sio_chan *siochan, dma_addr_t buf_addr,
				   size_t buf_len, size_t period_len, gfp_t gfp)
{
	struct sio_tx *siotx = NULL;
	int i, nperiods;

	if (!period_len || buf_len < period_len)
		return NULL;
	nperiods = buf_len / period_len;

	siotx = kzalloc(struct_size(siotx, siodesc, nperiods), gfp);
	if (!siotx)
		return NULL;

	init_completion(&siotx->done);
	siotx->vd.tx.chan = &siochan->vc.chan;
	siotx->period_len = period_len;
	siotx->nperiods = nperiods;

	for (i = 0; i < nperiods; i++) {
		struct sio_coproc_desc *d;

		siotx->siodesc[i] = d = sio_alloc_desc(siochan->host);
		if (!d) {
			sio_tx_free(&siotx->vd);
			return NULL;
		}

		/*
		 * T6030: publish the whole descriptor, its other fields zero;
		 * a slot may come back from an earlier transfer.
		 */
		if (siochan->host->hw->clear_period_desc)
			memset(d, 0, sizeof(*d));
		d->flag = 1; /* not sure what's up with this */
		d->iova = buf_addr + period_len * i;
		d->size = period_len;
	}
	dma_wmb();

	return siotx;
}

static struct dma_async_tx_descriptor *sio_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction,
		unsigned long flags)
{
	struct sio_chan *siochan = to_sio_chan(chan);
	struct sio_tx *siotx;

	if (direction != sio_chan_direction(siochan->no))
		return NULL;

	siotx = sio_alloc_tx(siochan, buf_addr, buf_len, period_len, GFP_NOWAIT);
	if (!siotx)
		return NULL;

	return vchan_tx_prep(&siochan->vc, &siotx->vd, flags);
}

static enum dma_status sio_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
				     struct dma_tx_state *txstate)
{
	struct sio_chan *siochan = to_sio_chan(chan);
	struct virt_dma_desc *vd;
	struct sio_tx *siotx;
	enum dma_status ret;
	unsigned long flags;
	int periods_residue;
	size_t residue;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE || !txstate)
		return ret;

	spin_lock_irqsave(&siochan->vc.lock, flags);
	siotx = siochan->current_tx;

	if (siotx && siotx->vd.tx.cookie == cookie) {
		ret = DMA_IN_PROGRESS;
		periods_residue = siotx->next - siotx->ninflight;
		while (periods_residue < 0)
			periods_residue += siotx->nperiods;
		residue = (siotx->nperiods - periods_residue) * siotx->period_len;
	} else {
		ret = DMA_IN_PROGRESS;
		residue = 0;
		vd = vchan_find_desc(&siochan->vc, cookie);
		if (vd) {
			siotx = to_sio_tx(&vd->tx);
			residue = siotx->period_len * siotx->nperiods;
		}
	}
	spin_unlock_irqrestore(&siochan->vc.lock, flags);
	dma_set_residue(txstate, residue);

	return ret;
}

static bool sio_fill_in_locked(struct sio_chan *siochan);

static struct sio_tx **sio_issued_slot(struct sio_chan *siochan, unsigned int i)
{
	return &siochan->issued[(siochan->issued_head + i) % SIO_MAX_NINFLIGHT];
}

/* Neither in flight nor waiting for its ISSUE to be acknowledged */
static bool sio_tx_quiet_locked(struct sio_chan *siochan, struct sio_tx *tx)
{
	unsigned int i;

	if (tx->ninflight)
		return false;
	for (i = 0; i < siochan->nissued; i++)
		if (*sio_issued_slot(siochan, i) == tx)
			return false;
	return true;
}

/* A transfer that is freed no longer owns issued descriptors */
static void sio_scrub_locked(struct sio_chan *siochan, struct sio_tx *tx)
{
	unsigned int i;

	for (i = 0; i < siochan->nissued; i++)
		if (*sio_issued_slot(siochan, i) == tx)
			*sio_issued_slot(siochan, i) = NULL;
}

/* Descriptors are issued from the current transfer, else from the idle one. */
static struct sio_tx *sio_source_locked(struct sio_chan *siochan)
{
	struct sio_tx *tx = siochan->current_tx;

	if (tx)
		return tx->terminated ? NULL : tx;
	if (siochan->idle && !siochan->idle->terminated)
		return siochan->idle;
	return NULL;
}

/*
 * The first idle transfer: the ring accounting starts from what the old one
 * has in flight, all of it the current transfer's. It cannot place two ISSUEs
 * still waiting for their acks, nor an ended transfer's descriptors.
 */
static int sio_ring_enter_locked(struct sio_chan *siochan)
{
	struct sio_tx *tx = siochan->current_tx;
	unsigned int i, n = tx ? tx->ninflight : 0;

	if (siochan->pending > 1 || (siochan->pending && !tx) || (tx && tx->terminated) ||
	    n + siochan->pending > SIO_MAX_NINFLIGHT)
		return -EBUSY;
	siochan->issued_head = 0;
	siochan->nissued = n + siochan->pending;
	for (i = 0; i < siochan->nissued; i++)
		siochan->issued[i] = tx;
	siochan->issuing = siochan->pending;
	siochan->pending = 0;
	siochan->ring = true;
	return 0;
}

/*
 * The resync point: once the channel is quiet, with no idle, current or
 * retired transfer and nothing issued, the old accounting resumes.
 */
static void sio_ring_leave_locked(struct sio_chan *siochan)
{
	if (!siochan->ring || siochan->idle || siochan->current_tx ||
	    !list_empty(&siochan->retired) || siochan->nissued || siochan->issuing)
		return;
	siochan->ring = false;
	siochan->issued_head = 0;
}

/*
 * An ended transfer with no descriptor left in flight: a retired one goes to
 * be freed, the waiter of another one is woken.
 */
static void sio_settle_locked(struct sio_chan *siochan, struct sio_tx *tx)
{
	if (!tx || !tx->terminated || !sio_tx_quiet_locked(siochan, tx))
		return;
	if (tx->retired) {
		list_move_tail(&tx->vd.node, &siochan->dead);
		wake_up(&siochan->drained);
		return;
	}
	/* sio_terminate_work() or sio_clear_idle() */
	complete(&tx->done);
}

/*
 * The old accounting, for channels that never had an idle transfer: an ACK
 * is the current transfer's, if its cookie still is.
 */
static void sio_old_issue_ack_locked(struct sio_chan *siochan, dma_cookie_t tx_cookie, bool ok)
{
	struct sio_tx *tx = siochan->current_tx;

	if (siochan->pending)
		siochan->pending--;
	if (!ok) {
		dev_err(siochan->host->dev, "nacked issue on chan %d\n", siochan->no);
		return;
	}
	if (!tx || tx_cookie != tx->vd.tx.cookie || tx->terminated)
		return;

	tx->next = (tx->next + 1) % tx->nperiods;
	tx->ninflight++;
	sio_fill_in_locked(siochan);
}

/* The ring accounting: an ACK is the last issued descriptor's. */
static void sio_ring_issue_ack_locked(struct sio_chan *siochan, bool ok)
{
	struct sio_tx *tx;

	siochan->issuing = false;
	if (!siochan->nissued) {
		dev_err_ratelimited(siochan->host->dev, "chan %d: ack without an issued descriptor\n",
				    siochan->no);
		return;
	}

	tx = *sio_issued_slot(siochan, siochan->nissued - 1);
	if (!ok) {
		dev_err(siochan->host->dev, "nacked issue on chan %d\n", siochan->no);
		siochan->nissued--;
		if (!siochan->nissued)
			wake_up(&siochan->drained);
		/*
		 * During a handover this may have been the ended transfer's
		 * last ISSUE, and no report is left to start the next one. A
		 * transfer whose own ISSUE was refused is not issued again.
		 */
		if (sio_source_locked(siochan) != tx)
			sio_fill_in_locked(siochan);
		sio_settle_locked(siochan, tx);
		return;
	}

	/* An ended transfer's descriptor is in flight all the same. */
	if (tx) {
		tx->next = (tx->next + 1) % tx->nperiods;
		tx->ninflight++;
	}
	sio_fill_in_locked(siochan);
}

static void sio_handle_issue_ack(struct sio_chan *siochan, void *cookie, bool ok)
{
	unsigned long flags;

	spin_lock_irqsave(&siochan->vc.lock, flags);
	if (siochan->ring)
		sio_ring_issue_ack_locked(siochan, ok);
	else
		sio_old_issue_ack_locked(siochan, (unsigned long)cookie, ok);
	spin_unlock_irqrestore(&siochan->vc.lock, flags);
}

/* The old accounting: the current transfer's next descriptor */
static bool sio_fill_in_old_locked(struct sio_chan *siochan)
{
	struct sio_data *sio = siochan->host;
	struct sio_tx *tx = siochan->current_tx;
	struct sio_coproc_desc *d = tx->siodesc[tx->next];
	int ret;

	if (tx->ninflight >= SIO_MAX_NINFLIGHT || tx->terminated)
		return false;

	static_assert(sizeof(dma_cookie_t) <= sizeof(void *));
	ret = sio_send_siomsg_atomic(sio, FIELD_PREP(SIOMSG_EP, siochan->no) |
				     FIELD_PREP(SIOMSG_TYPE, MSG_ISSUE) |
				     FIELD_PREP(SIOMSG_DATA, sio_coproc_desc_slot(sio, d)),
				     sio_handle_issue_ack, (void *)(uintptr_t)tx->vd.tx.cookie);
	if (ret < 0)
		dev_err_ratelimited(sio->dev, "can't issue on chan %d ninflight %d: %d\n",
				    siochan->no, tx->ninflight, ret);
	else
		siochan->pending++;
	return true;
}

/* The ring accounting: one ISSUE at a time, each after the previous one's ack */
static bool sio_fill_in_ring_locked(struct sio_chan *siochan)
{
	struct sio_data *sio = siochan->host;
	struct sio_tx *tx = sio_source_locked(siochan);
	struct sio_coproc_desc *d;
	int ret;

	if (!tx || siochan->issuing || siochan->stopping ||
	    siochan->nissued >= SIO_MAX_NINFLIGHT)
		return false;

	d = tx->siodesc[tx->next];
	ret = sio_send_siomsg_atomic(sio, FIELD_PREP(SIOMSG_EP, siochan->no) |
				     FIELD_PREP(SIOMSG_TYPE, MSG_ISSUE) |
				     FIELD_PREP(SIOMSG_DATA, sio_coproc_desc_slot(sio, d)),
				     sio_handle_issue_ack, NULL);
	if (ret < 0) {
		dev_err_ratelimited(sio->dev, "can't issue on chan %d ninflight %d: %d\n",
				    siochan->no, siochan->nissued, ret);
		return false;
	}
	/* The ack takes the channel lock, held here. */
	*sio_issued_slot(siochan, siochan->nissued++) = tx;
	siochan->issuing = true;
	return true;
}

static bool sio_fill_in_locked(struct sio_chan *siochan)
{
	if (siochan->ring)
		return sio_fill_in_ring_locked(siochan);
	return siochan->current_tx && sio_fill_in_old_locked(siochan);
}

static void sio_update_current_tx_locked(struct sio_chan *siochan)
{
	struct virt_dma_desc *vd = vchan_next_desc(&siochan->vc);

	if (vd && !siochan->current_tx) {
		list_del(&vd->node);
		siochan->current_tx = to_sio_tx(&vd->tx);
		sio_fill_in_locked(siochan);
	}
}

static void sio_issue_pending(struct dma_chan *chan)
{
	struct sio_chan *siochan = to_sio_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&siochan->vc.lock, flags);
	vchan_issue_pending(&siochan->vc);
	sio_update_current_tx_locked(siochan);
	spin_unlock_irqrestore(&siochan->vc.lock, flags);
}

static int sio_terminate_all(struct dma_chan *chan)
{
	struct sio_chan *siochan = to_sio_chan(chan);
	struct sio_tx *tx;
	unsigned long flags;
	LIST_HEAD(to_free);

	spin_lock_irqsave(&siochan->vc.lock, flags);
	tx = siochan->current_tx;
	if (tx && !tx->terminated) {
		dma_cookie_complete(&tx->vd.tx);
		tx->terminated = true;
		if (siochan->ring && siochan->idle && !siochan->idle->terminated) {
			/*
			 * No TERMINATE: the idle transfer follows the issued
			 * descriptors, and the peripheral runs on without a gap.
			 * sio_synchronize() waits for those to complete.
			 */
			if (siochan->vc.cyclic == &tx->vd)
				siochan->vc.cyclic = NULL;
			siochan->current_tx = NULL;
			/* Its callback may be running: sio_free_dead() frees it. */
			if (sio_tx_quiet_locked(siochan, tx)) {
				list_add_tail(&tx->vd.node, &siochan->dead);
			} else {
				tx->retired = true;
				list_add_tail(&tx->vd.node, &siochan->retired);
			}
			sio_fill_in_locked(siochan);
		} else {
			if (siochan->ring && sio_tx_quiet_locked(siochan, tx))
				complete(&tx->done);
			schedule_work(&siochan->terminate_wq);
		}
	}
	vchan_get_all_descriptors(&siochan->vc, &to_free);
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	vchan_dma_desc_free_list(&siochan->vc, &to_free);

	return 0;
}

static void sio_terminate_work(struct work_struct *wq)
{
	struct sio_chan *siochan = container_of(wq, struct sio_chan, terminate_wq);
	struct sio_tx *tx;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&siochan->vc.lock, flags);
	tx = siochan->current_tx;
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	if (WARN_ON(!tx))
		return;

	ret = sio_call(siochan->host, FIELD_PREP(SIOMSG_EP, siochan->no) |
				      FIELD_PREP(SIOMSG_TYPE, MSG_TERMINATE));
	if (ret < 0)
		dev_err(siochan->host->dev, "terminate call on chan %d failed: %d\n",
			siochan->no, ret);

	ret = wait_for_completion_timeout(&tx->done, msecs_to_jiffies(500));
	if (!ret)
		dev_err(siochan->host->dev, "terminate descriptor wait timed out\n");

	tasklet_kill(&siochan->vc.task);

	spin_lock_irqsave(&siochan->vc.lock, flags);
	WARN_ON(siochan->current_tx != tx);
	siochan->current_tx = NULL;
	sio_scrub_locked(siochan, tx);
	sio_update_current_tx_locked(siochan);
	sio_ring_leave_locked(siochan);
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	sio_tx_free(&tx->vd);
}

static bool sio_drained(struct sio_chan *siochan)
{
	unsigned long flags;
	bool drained;

	spin_lock_irqsave(&siochan->vc.lock, flags);
	drained = list_empty(&siochan->retired);
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	return drained;
}

static bool sio_ring_empty(struct sio_chan *siochan)
{
	unsigned long flags;
	bool empty;

	spin_lock_irqsave(&siochan->vc.lock, flags);
	empty = !siochan->nissued && !siochan->issuing;
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	return empty;
}

/*
 * Issued descriptors that did not complete in time: stop the channel and wait
 * for their reports, so that their buffers are no longer read. Nothing is
 * issued from the TERMINATE until the channel is known stopped. A channel
 * that acknowledged TERMINATE and still reports nothing is resynced: its ring
 * is emptied, as nothing it holds is played any more. A channel that refused
 * TERMINATE or did not answer cannot be known to have stopped: -EIO, and its
 * transfers stay as they are. Then the current or idle transfer is issued
 * again.
 */
static int sio_stop_quiet(struct sio_chan *siochan)
{
	struct device *dev = siochan->host->dev;
	unsigned long flags;
	struct sio_tx *tx;
	unsigned int i;
	int ret;

	dev_err(dev, "chan %d: issued descriptors did not complete, stopping it\n", siochan->no);
	spin_lock_irqsave(&siochan->vc.lock, flags);
	siochan->stopping = true;
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	/* 1 on ACK, 0 on NACK, negative without an answer */
	ret = sio_call(siochan->host, FIELD_PREP(SIOMSG_EP, siochan->no) |
				      FIELD_PREP(SIOMSG_TYPE, MSG_TERMINATE));
	if (ret == 1)
		wait_event_timeout(siochan->drained, sio_ring_empty(siochan),
				   msecs_to_jiffies(500));
	else if (!ret)
		dev_err(dev, "chan %d: TERMINATE refused, its buffers may still be read\n",
			siochan->no);
	else
		dev_err(dev, "chan %d: not stopped (%d), its buffers may still be read\n",
			siochan->no, ret);

	spin_lock_irqsave(&siochan->vc.lock, flags);
	if (ret == 1 && (siochan->nissued || siochan->issuing)) {
		dev_err(dev, "chan %d: stopped with %u descriptors unreported, ring reset\n",
			siochan->no, siochan->nissued);
		for (i = 0; i < siochan->nissued; i++) {
			tx = *sio_issued_slot(siochan, i);
			if (tx)
				tx->ninflight = 0;
		}
		siochan->nissued = 0;
		siochan->issuing = false;
		siochan->issued_head = 0;
	}
	if (ret == 1) {
		list_splice_tail_init(&siochan->retired, &siochan->dead);
		if (siochan->idle && siochan->idle->terminated)
			complete(&siochan->idle->done);
	}
	siochan->stopping = false;
	sio_fill_in_locked(siochan);
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	return ret == 1 ? 0 : -EIO;
}

/* Frees the ended transfers once no completion callback can still use them. */
static void sio_free_dead(struct sio_chan *siochan)
{
	struct sio_tx *tx, *n;
	unsigned long flags;
	LIST_HEAD(dead);

	spin_lock_irqsave(&siochan->vc.lock, flags);
	list_splice_init(&siochan->dead, &dead);
	spin_unlock_irqrestore(&siochan->vc.lock, flags);
	if (list_empty(&dead))
		return;

	/* vchan_complete() runs a callback with its descriptor unlocked. */
	tasklet_kill(&siochan->vc.task);
	list_for_each_entry_safe(tx, n, &dead, vd.node) {
		list_del(&tx->vd.node);
		sio_tx_free(&tx->vd);
	}
}

static void sio_synchronize(struct dma_chan *chan)
{
	struct sio_chan *siochan = to_sio_chan(chan);

	flush_work(&siochan->terminate_wq);
	if (READ_ONCE(siochan->ring)) {
		/* The caller frees the buffer on return; nothing can stop that. */
		if (!wait_event_timeout(siochan->drained, sio_drained(siochan),
					msecs_to_jiffies(500)) &&
		    sio_stop_quiet(siochan))
			dev_WARN_ONCE(siochan->host->dev, 1,
				      "chan %d: synchronized without being stopped\n",
				      siochan->no);
		/* No callback of an ended transfer runs after this. */
		tasklet_kill(&siochan->vc.task);
	}
	sio_free_dead(siochan);
}

/*
 * Returns once the idle transfer's descriptors completed. If the channel
 * cannot be confirmed stopped, the idle transfer stays, its buffer in use.
 */
static int sio_clear_idle(struct sio_chan *siochan)
{
	struct sio_tx *idle;
	unsigned long flags;
	bool quiet, stop;
	int ret;

	spin_lock_irqsave(&siochan->vc.lock, flags);
	idle = siochan->idle;
	if (!idle) {
		spin_unlock_irqrestore(&siochan->vc.lock, flags);
		return 0;
	}
	idle->terminated = true;
	quiet = sio_tx_quiet_locked(siochan, idle);
	/* Nothing follows the idle transfer: the channel stops. */
	stop = !quiet && !siochan->current_tx;
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	if (stop) {
		ret = sio_call(siochan->host, FIELD_PREP(SIOMSG_EP, siochan->no) |
					      FIELD_PREP(SIOMSG_TYPE, MSG_TERMINATE));
		if (ret < 0)
			dev_err(siochan->host->dev, "terminate call on chan %d failed: %d\n",
				siochan->no, ret);
	}
	if (!quiet && !wait_for_completion_timeout(&idle->done, msecs_to_jiffies(500))) {
		ret = sio_stop_quiet(siochan);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&siochan->vc.lock, flags);
	siochan->idle = NULL;
	sio_scrub_locked(siochan, idle);
	sio_ring_leave_locked(siochan);
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	sio_tx_free(&idle->vd);
	sio_free_dead(siochan);
	return 0;
}

/* See struct apple_sio_idle */
static int sio_set_idle(struct sio_chan *siochan, const struct apple_sio_idle *config)
{
	struct sio_tx *idle;
	unsigned long flags;
	int ret;

	if (!config->len)
		return sio_clear_idle(siochan);
	if (!config->period_len || config->len < config->period_len)
		return -EINVAL;

	idle = sio_alloc_tx(siochan, config->addr, config->len, config->period_len, GFP_KERNEL);
	if (!idle)
		return -ENOMEM;

	spin_lock_irqsave(&siochan->vc.lock, flags);
	ret = siochan->idle ? -EBUSY : 0;
	if (!ret && !siochan->ring)
		ret = sio_ring_enter_locked(siochan);
	if (ret) {
		spin_unlock_irqrestore(&siochan->vc.lock, flags);
		sio_tx_free(&idle->vd);
		return ret;
	}
	siochan->idle = idle;
	sio_fill_in_locked(siochan);
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	return 0;
}

static void sio_free_chan_resources(struct dma_chan *chan)
{
	sio_terminate_all(chan);
	sio_synchronize(chan);
	sio_clear_idle(to_sio_chan(chan));
	vchan_free_chan_resources(&to_sio_chan(chan)->vc);
}

static struct dma_chan *sio_dma_of_xlate(struct of_phandle_args *dma_spec,
					 struct of_dma *ofdma)
{
	struct sio_data *sio = (struct sio_data *) ofdma->of_dma_data;
	unsigned int index = dma_spec->args[0];

	if (dma_spec->args_count != 1 || index >= sio->nchannels)
		return ERR_PTR(-EINVAL);

	return dma_get_slave_channel(&sio->channels[index].vc.chan);
}

static void sio_rtk_crashed(void *cookie, const void *crashlog, size_t crashlog_size)
{
	struct sio_data *sio = cookie;

	dev_err(sio->dev, "SIO down (crashed)");
}

/* The old accounting: a report is the current transfer's. */
static void sio_old_report_locked(struct sio_chan *siochan)
{
	struct sio_tx *tx = siochan->current_tx;

	if (!tx)
		return;
	if (tx->ninflight)
		tx->ninflight--;
	vchan_cyclic_callback(&tx->vd);
	if (!sio_fill_in_locked(siochan) && !tx->ninflight)
		complete(&tx->done);
}

/* The ring accounting: the oldest issued descriptor completed. */
static void sio_ring_report_locked(struct sio_chan *siochan)
{
	struct sio_tx *tx;

	/* An ISSUE not yet acknowledged has no completed descriptor. */
	if (siochan->nissued <= (siochan->issuing ? 1 : 0)) {
		dev_warn_ratelimited(siochan->host->dev, "chan %d: report without a descriptor\n",
				     siochan->no);
		return;
	}

	tx = *sio_issued_slot(siochan, 0);
	siochan->issued_head = (siochan->issued_head + 1) % SIO_MAX_NINFLIGHT;
	siochan->nissued--;
	if (tx) {
		if (tx->ninflight)
			tx->ninflight--;
		else
			dev_err_ratelimited(siochan->host->dev,
					    "chan %d: report for a descriptor not in flight\n",
					    siochan->no);
		if (tx == siochan->current_tx && !tx->terminated)
			vchan_cyclic_callback(&tx->vd);
	}
	sio_fill_in_locked(siochan);
	sio_settle_locked(siochan, tx);
	if (!siochan->nissued)
		wake_up(&siochan->drained);
	sio_ring_leave_locked(siochan);
}

static void sio_process_report(struct sio_chan *siochan)
{
	unsigned long flags;

	spin_lock_irqsave(&siochan->vc.lock, flags);
	if (siochan->ring)
		sio_ring_report_locked(siochan);
	else
		sio_old_report_locked(siochan);
	spin_unlock_irqrestore(&siochan->vc.lock, flags);
}

static void sio_recv_msg(void *cookie, u8 ep, u64 msg)
{
	struct sio_data *sio = cookie;
	struct sio_tagdata *tags = &sio->tags;
	u32 data;
	u8 type, tag, sioep;

	if (ep != EP_SIO)
		goto unknown;

	data  = FIELD_GET(SIOMSG_DATA, msg);
	// param = FIELD_GET(SIOMSG_PARAM, msg);
	type  = FIELD_GET(SIOMSG_TYPE, msg);
	tag   = FIELD_GET(SIOMSG_TAG, msg);
	sioep = FIELD_GET(SIOMSG_EP, msg);

	switch (type) {
	case MSG_STARTED:
		sio->protocol = data;
		dev_info(sio->dev, "SIO protocol v%u\n", data);
		type = MSG_ACK; /* Pretend this is an ACK */
		fallthrough;
	case MSG_ACK:
	case MSG_NACK:
		if (WARN_ON(tag >= SIO_NTAGS))
			break;

		if (tags->atomic[tag]) {
			sio_ack_callback callback = tags->ack_callback[tag];

			if (callback && !WARN_ON(sioep >= sio->nchannels))
				callback(&sio->channels[sioep],
					 tags->cookie[tag], type == MSG_ACK);
			if (type == MSG_NACK)
				dev_err(sio->dev, "got a NACK on channel %d\n", sioep);
			sio_free_tag(sio, tag);
		} else {
			tags->acked[tag] = (type == MSG_ACK);
			complete(&tags->completions[tag]);
		}
		break;

	case MSG_REPORT:
		if (WARN_ON(sioep >= sio->nchannels))
			break;

		sio_process_report(&sio->channels[sioep]);
		break;

	default:
		goto unknown;
	}
	return;

unknown:
	dev_warn(sio->dev, "received unknown message: ep %x data %016llx\n",
		 ep, msg);
}

static int _sio_send_siomsg(struct sio_data *sio, u64 msg, bool atomic,
			    sio_ack_callback ack_callback, void *cookie)
{
	int tag, ret;

	tag = sio_alloc_tag(sio);
	if (tag < 0)
		return tag;

	if (atomic)
		sio_set_tag_atomic(sio, tag, ack_callback, cookie);
	else
		reinit_completion(&sio->tags.completions[tag]);

	msg &= ~SIOMSG_TAG;
	msg |= FIELD_PREP(SIOMSG_TAG, tag);
	ret = apple_rtkit_send_message(sio->rtk, EP_SIO, msg, NULL,
				       atomic);
	if (ret < 0) {
		sio_free_tag(sio, tag);
		return ret;
	}

	return tag;
}

static int sio_send_siomsg(struct sio_data *sio, u64 msg)
{
	return _sio_send_siomsg(sio, msg, false, NULL, NULL);
}

static int sio_send_siomsg_atomic(struct sio_data *sio, u64 msg,
				  sio_ack_callback ack_callback,
				  void *cookie)
{
	return _sio_send_siomsg(sio, msg, true, ack_callback, cookie);
}

static int sio_call(struct sio_data *sio, u64 msg)
{
	int tag, ret;

	tag = sio_send_siomsg(sio, msg);
	if (tag < 0)
		return tag;

	ret = wait_for_completion_timeout(&sio->tags.completions[tag],
					  msecs_to_jiffies(SIO_CALL_TIMEOUT_MS));
	if (!ret) {
		dev_warn(sio->dev, "call %8llx timed out\n", msg);
		sio_free_tag(sio, tag);
		return -ETIME;
	}

	ret = sio->tags.acked[tag];
	sio_free_tag(sio, tag);

	return ret;
}

static const struct apple_rtkit_ops sio_rtkit_ops = {
	.crashed = sio_rtk_crashed,
	.recv_message = sio_recv_msg,
};

/* The configuration record CONFIGURE points the firmware at (handle 0) */
static void sio_write_config(const struct sio_hw_data *hw, void *shmem,
			     const struct sio_shmem_chan_config *cfg)
{
	memcpy(shmem, cfg, sizeof(*cfg));
	/* T6041 takes a sixth configuration word, zero. */
	if (hw->cfg_six_words)
		memset(shmem + sizeof(*cfg), 0, sizeof(u32));
}

static int sio_device_config(struct dma_chan *chan,
			     struct dma_slave_config *config)
{
	struct sio_chan *siochan = to_sio_chan(chan);
	struct sio_data *sio = siochan->host;
	bool is_tx = sio_chan_direction(siochan->no) == DMA_MEM_TO_DEV;
	struct sio_shmem_chan_config *cfg_shmem = sio->shmem;
	struct sio_shmem_chan_config cfg;
	int ret;

	if (config->peripheral_config) {
		if (config->peripheral_size != sizeof(struct apple_sio_idle))
			return -EINVAL;
		return sio_set_idle(siochan, config->peripheral_config);
	}

	switch (is_tx ? config->dst_addr_width : config->src_addr_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		cfg.datashape = 0;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		cfg.datashape = 1;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		cfg.datashape = 2;
		break;
	default:
		return -EINVAL;
	}

	cfg.timeout = 0;
	cfg.fifo = 0x800;
	cfg.limit = sio->cfg_word4;
	/*
	 * The ADT records each client's channel configuration with it (the
	 * dp-audioN dma-channels: channel, data shape, timeout, FIFO size,
	 * threshold). T6030's DisplayPort audio channels refill at 0x600.
	 */
	cfg.threshold = sio->fifo_threshold;

	/*
	 * Dmaengine prescribes we ought to apply the new configuration only
	 * to newly-queued descriptors.
	 *
	 * To comply with dmaengine's interface we take the lazy path here:
	 * we apply the configuration right away, we only allow the channel
	 * to be configured once, which means subsequent calls to `device_config`
	 * either return -EBUSY if the configuration differs, or they are
	 * a no-op if the configuration is the same as the starting one.
	 *
	 * This is the reasonable thing to do given that these sio channels
	 * are tied to fixed peripherals, and what's more given that the
	 * only planned consumer of this dmaengine driver in the kernel is
	 * diplayport audio support, where the DMA configuration is fixed,
	 * and no more than a single descriptor (a cyclic one) gets ever issued
	 * at the same time.
	 *
	 * The code complexity cost of tracking to which descriptor
	 * the configuration relates would be significant here, especially
	 * since we need to do a non-atomic operation to apply it (a call to
	 * the coprocessor) and dmaengine has its bunch of atomicity
	 * restrictions. And this complexity would be for naught since it
	 * doesn't even get exercised by the only planned consumer.
	 */
	if (siochan->configured && memcmp(&siochan->cfg, &cfg, sizeof(cfg)))
		return -EBUSY;

	sio_write_config(sio->hw, cfg_shmem, &cfg);
	dma_wmb();

	ret = sio_call(sio, FIELD_PREP(SIOMSG_TYPE, MSG_CONFIGURE) |
			    FIELD_PREP(SIOMSG_EP, siochan->no));

	if (ret == 1)
		ret = 0;
	else if (ret == 0)
		ret = -EINVAL;

	if (ret == 0) {
		siochan->configured = true;
		siochan->cfg = cfg;
	}

	return ret;
}

static int sio_set_param(struct sio_data *sio, u32 key, u32 val)
{
	int err;

	err = sio_call(sio, FIELD_PREP(SIOMSG_TYPE, MSG_SETUP) |
			    FIELD_PREP(SIOMSG_PARAM, key & 0xff) |
			    FIELD_PREP(SIOMSG_EP, key >> 8) |
			    FIELD_PREP(SIOMSG_DATA, val));
	if (err == 1)
		return 0;
	if (!err)
		err = -ENXIO;
	return dev_err_probe(sio->dev, err, "sending SIO parameter %#x value %#x\n",
			     key, val);
}

/*
 * Control tables must precede performance and DMA state. In particular, the
 * shim size (0x23) makes the firmware consume the shim table. Do not enable
 * the performance endpoint or publish DMA response state before that ACK.
 */
static const u16 sio_v9_param_order[] = {
	0x0f, 0x10,	/* panic buffer */
	0x1a, 0x1b,	/* map ranges */
	0x1e, 0x1f,	/* ASC-wrapper tunables */
	0x20, 0x21,	/* optional miscellaneous tunables */
	0x24, 0x25,	/* optional power-state register table */
	0x22, 0x23,	/* DMA shims */
	0x1c, 0x1d,	/* device types, after shims */
	0x30d, 0x30e,	/* performance endpoint, after control tables */
};

#define SIO_V9_REQUIRED_PARAMS (GENMASK(15, 0) & ~GENMASK(9, 6))

static int sio_send_v9_dt_params(struct sio_data *sio, const __be32 *params,
				 unsigned int nparams,
				 int (*set_param)(struct sio_data *, u32, u32))
{
	u32 values[ARRAY_SIZE(sio_v9_param_order)];
	u32 present = 0;
	unsigned int i, j;
	int err;

	if (!nparams || nparams % 2)
		return -EINVAL;

	for (i = 0; i < nparams; i += 2) {
		u32 key = be32_to_cpu(params[i]);

		/* Replace the loader's undersized response mapping with our own. */
		if (key == 0x0b || key == 0x0c)
			continue;
		for (j = 0; j < ARRAY_SIZE(sio_v9_param_order); j++)
			if (key == sio_v9_param_order[j])
				break;
		if (j == ARRAY_SIZE(sio_v9_param_order) || (present & BIT(j)))
			return -EINVAL;
		present |= BIT(j);
		values[j] = be32_to_cpu(params[i + 1]);
	}
	if ((present & SIO_V9_REQUIRED_PARAMS) != SIO_V9_REQUIRED_PARAMS)
		return -EINVAL;
	/* An optional table still requires both its address and its size. */
	for (i = 0; i < ARRAY_SIZE(sio_v9_param_order); i += 2)
		if (!!(present & BIT(i)) != !!(present & BIT(i + 1)))
			return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(sio_v9_param_order); i++) {
		if (!(present & BIT(i)))
			continue;
		err = set_param(sio, sio_v9_param_order[i], values[i]);
		if (err)
			return err;
	}
	return 0;
}

static void sio_free_v9_shmem(void *data)
{
	struct sio_data *sio = data;
	int err;

	/* An acknowledged shutdown must precede releasing firmware DMA targets. */
	err = apple_rtkit_shutdown(sio->rtk);
	if (err) {
		dev_err(sio->dev, "SIO shutdown failed: %d; retaining DMA buffers\n", err);
		return;
	}
	dma_free_coherent(sio->dev, sio->arrays->responses * sio->arrays->response_size,
			  sio->responses, sio->responses_iova);
	dma_free_coherent(sio->dev, sio->arrays->segments * sio->arrays->segment_size,
			  sio->shmem, sio->shmem_iova);
}

static int sio_send_v9_shmem(struct sio_data *sio,
			     int (*set_param)(struct sio_data *, u32, u32))
{
	const struct {
		u32 key;
		u32 value;
	} params[] = {
		{ 1, sio->shmem_iova >> 12 },
		{ 2, (sio->arrays ?: &sio_v9_arrays)->segments },
		{ 0x0b, sio->responses_iova >> 12 },
		{ 0x0c, (sio->arrays ?: &sio_v9_arrays)->responses },
	};
	unsigned int i;
	int err;

	for (i = 0; i < ARRAY_SIZE(params); i++) {
		err = set_param(sio, params[i].key, params[i].value);
		if (err)
			return err;
	}
	return 0;
}

static int sio_alloc_shmem(struct sio_data *sio)
{
	const struct sio_dma_arrays *arrays = sio->hw->arrays;
	size_t size, responses_size = 0;
	int err;

	if (!arrays && sio->protocol == 9)
		arrays = &sio_v9_arrays;
	sio->arrays = arrays;
	if (arrays &&
	    (check_mul_overflow((size_t)arrays->segments, (size_t)arrays->segment_size, &size) ||
	     check_mul_overflow((size_t)arrays->responses, (size_t)arrays->response_size,
				&responses_size)))
		return -EOVERFLOW;
	if (!arrays)
		size = SIO_SHMEM_SIZE;

	sio->shmem = dma_alloc_coherent(sio->dev, size,
					&sio->shmem_iova, GFP_KERNEL | __GFP_ZERO);
	if (!sio->shmem)
		return -ENOMEM;

	sio->shmem_desc_base = (struct sio_coproc_desc *) (sio->shmem + 56);
	sio->desc_allocated = devm_bitmap_zalloc(sio->dev, SIO_NO_DESC_SLOTS,
					       GFP_KERNEL);
	if (!sio->desc_allocated) {
		err = -ENOMEM;
		goto free_shmem;
	}

	if (arrays) {
		sio->responses = dma_alloc_coherent(sio->dev, responses_size,
						    &sio->responses_iova, GFP_KERNEL | __GFP_ZERO);
		if (!sio->responses) {
			err = -ENOMEM;
			goto free_shmem;
		}
		/* Registered before publication, released before the RTKit handle. */
		err = devm_add_action_or_reset(sio->dev, sio_free_v9_shmem, sio);
		if (err)
			return err;
		return sio_send_v9_shmem(sio, sio_set_param);
	}

	err = sio_set_param(sio, 1, sio->shmem_iova >> 12);
	if (err)
		return err;
	return sio_set_param(sio, 2, SIO_SHMEM_SIZE);

free_shmem:
	/* No address has been published yet. */
	dma_free_coherent(sio->dev, size, sio->shmem, sio->shmem_iova);
	return err;
}

static int sio_send_dt_params(struct sio_data *sio)
{
	struct device_node *np = sio->dev->of_node;
	const char *propname = "apple,sio-firmware-params";
	int nparams, err, i;

	if (sio->hw->send_dt_params)
		return sio->hw->send_dt_params(sio);

	nparams = of_property_count_u32_elems(np, propname);
	if (nparams < 0) {
		err = nparams;
		goto badprop;
	}
	if (sio->protocol == 9) {
		const __be32 *params = of_get_property(np, propname, NULL);

		if (!params)
			return -EINVAL;
		return sio_send_v9_dt_params(sio, params, nparams, sio_set_param);
	}

	for (i = 0; i < nparams / 2; i++) {
		u32 key, val;

		err = of_property_read_u32_index(np, propname, 2 * i, &key);
		if (err)
			goto badprop;
		err = of_property_read_u32_index(np, propname, 2 * i + 1, &val);
		if (err)
			goto badprop;

		err = sio_set_param(sio, key, val);
		if (err)
			return err;
	}

	return 0;

badprop:
	return dev_err_probe(sio->dev, err, "failed to read '%s'\n", propname);
}

/* Interrupts off, one CPU: the idle pair and receive off. The sleep clears the run bit twice. */
static int sio_commit_stop(void *data)
{
	struct sio_data *sio = data;
	unsigned int us;
	int ret;

	ret = apple_rtkit_sleep_idle_polled(sio->rtk, sio->base + REG_CPU_CONTROL, &us);
	apple_rtkit_mailbox_recv_off(sio->rtk);

	return ret;
}

static int sio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	const struct sio_hw_data *hw;
	struct sio_data *sio;
	struct dma_device *dma;
	bool firmware_only;
	int nchannels;
	int err, i;

	match = of_match_node(sio_of_match, np);
	if (!match)
		return -ENODEV;
	hw = match->data;
	if (hw == &sio_hw_t6050 && !sio_t6050_board())
		hw = &sio_hw_generic;
	/* Refuse incompatible firmware tables before DMA or hardware setup. */
	firmware_only = hw->firmware_only;
	if (hw->check_boot_tables) {
		err = hw->check_boot_tables(&pdev->dev);
		if (err && !hw->firmware_without_tables)
			return err;
		if (err)
			firmware_only = true;
	}

	err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(42));
	if (err)
		return dev_err_probe(&pdev->dev, err, "Failed to set DMA mask\n");

	err = of_property_read_u32(np, "dma-channels", &nchannels);
	if (err || nchannels > NCHANNELS_MAX)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing or invalid dma-channels property\n");

	sio = devm_kzalloc(&pdev->dev, struct_size(sio, channels, nchannels), GFP_KERNEL);
	if (!sio)
		return -ENOMEM;

	platform_set_drvdata(pdev, sio);
	sio->dev = &pdev->dev;
	sio->nchannels = nchannels;
	sio->hw = hw;
	sio->firmware_only = firmware_only;

	/* The device tree's channel words if it gives them, else the SoC's, else the generic ones. */
	sio->fifo_threshold = hw->fifo_threshold ?: sio_hw_generic.fifo_threshold;
	sio->cfg_word4 = hw->cfg_word4 ?: sio_hw_generic.cfg_word4;
	of_property_read_u32(np, "apple,channel-threshold", &sio->fifo_threshold);
	of_property_read_u32(np, "apple,channel-limit", &sio->cfg_word4);

	sio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sio->base))
		return PTR_ERR(sio->base);

	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	err = devm_pm_runtime_enable(&pdev->dev);
	if (err < 0)
		return dev_err_probe(&pdev->dev, err,
				     "pm_runtime_enable failed: %d\n", err);

	sio->rtk = devm_apple_rtkit_init(&pdev->dev, sio, NULL, 0, &sio_rtkit_ops);
	if (IS_ERR(sio->rtk)) {
		err = PTR_ERR(sio->rtk);
		dev_err(&pdev->dev, "couldn't initialize rtkit\n");
		goto rpm_put;
	}
	for (i = 1; i < SIO_NTAGS; i++)
		init_completion(&sio->tags.completions[i]);

	dma = &sio->dma;
	dma_cap_set(DMA_PRIVATE, dma->cap_mask);
	dma_cap_set(DMA_CYCLIC, dma->cap_mask);

	dma->dev = &pdev->dev;
	dma->device_free_chan_resources = sio_free_chan_resources;
	dma->device_tx_status = sio_tx_status;
	dma->device_issue_pending = sio_issue_pending;
	dma->device_terminate_all = sio_terminate_all;
	dma->device_synchronize = sio_synchronize;
	dma->device_prep_dma_cyclic = sio_prep_dma_cyclic;
	dma->device_config = sio_device_config;

	dma->directions = BIT(DMA_MEM_TO_DEV);
	dma->residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;
	dma->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
			       BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
			       BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);

	INIT_LIST_HEAD(&dma->channels);
	for (i = 0; i < nchannels; i++) {
		struct sio_chan *siochan = &sio->channels[i];

		siochan->host = sio;
		siochan->no = i;
		siochan->vc.desc_free = sio_tx_free;
		INIT_WORK(&siochan->terminate_wq, sio_terminate_work);
		INIT_LIST_HEAD(&siochan->retired);
		INIT_LIST_HEAD(&siochan->dead);
		init_waitqueue_head(&siochan->drained);
		vchan_init(&siochan->vc, dma);
	}

	writel(CPU_CONTROL_RUN, sio->base + REG_CPU_CONTROL);

	err = apple_rtkit_boot(sio->rtk);
	if (err)
		return dev_err_probe(&pdev->dev, err, "SIO did not boot\n");

	if (sio->firmware_only) {
		dev_info(&pdev->dev, "firmware running, no DMA service\n");
		goto rpm_put;
	}

	err = apple_rtkit_start_ep(sio->rtk, EP_SIO);
	if (err)
		return dev_err_probe(&pdev->dev, err, "starting SIO endpoint\n");

	err = sio_call(sio, FIELD_PREP(SIOMSG_TYPE, MSG_START));
	if (err < 1) {
		if (err == 0)
			err = -ENXIO;
		return dev_err_probe(&pdev->dev, err, "starting SIO service\n");
	}
	if (hw->protocol && sio->protocol != hw->protocol)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "SIO firmware protocol %u, this SoC's is %u\n",
				     sio->protocol, hw->protocol);

	err = sio_send_dt_params(sio);
	if (err < 0)
		return dev_err_probe(&pdev->dev, err, "failed to send boot-up parameters\n");

	err = sio_alloc_shmem(sio);
	if (err < 0)
		return err;

	if (hw->commit_stop) {
		err = apple_sleep_coproc_register(&pdev->dev, sio->base, false, sio_commit_stop, sio);
		if (err)
			return dev_err_probe(&pdev->dev, err, "registering the sleep's stop\n");
		apple_sleep_coproc_set_early(&pdev->dev);
	}

	err = dma_async_device_register(&sio->dma);
	if (err)
		return dev_err_probe(&pdev->dev, err, "failed to register DMA device\n");

	err = of_dma_controller_register(pdev->dev.of_node, sio_dma_of_xlate, sio);
	if (err) {
		dma_async_device_unregister(&sio->dma);
		return dev_err_probe(&pdev->dev, err, "failed to register with OF\n");
	}

rpm_put:
	if (!err && hw->stop_at_commit)
		sio->commit_registered = !apple_sleep_coproc_register(&pdev->dev, sio->base, false,
								      sio_commit_stop, sio);
	/*
	 * One stopped by the sleep's commit runs until then: a runtime suspend would let its domain
	 * go off under the firmware.
	 */
	if (!hw->commit_stop)
		pm_runtime_put(&pdev->dev);

	return err;
}

static void sio_remove(struct platform_device *pdev)
{
	struct sio_data *sio = platform_get_drvdata(pdev);

	if (sio->commit_registered || sio->hw->commit_stop)
		apple_sleep_coproc_unregister(&pdev->dev);
	if (sio->firmware_only)
		return;

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&sio->dma);
}

static __maybe_unused int sio_suspend(struct device *dev)
{
	/*
	 * TODO: SIO coproc sleep state
	 */
	return 0;
}

static __maybe_unused int sio_resume(struct device *dev)
{
	return 0;
}

/* A channel with a transfer can't be carried through a firmware restart. */
static bool sio_chan_busy(struct sio_chan *siochan)
{
	unsigned long flags;
	bool busy;

	spin_lock_irqsave(&siochan->vc.lock, flags);
	busy = siochan->current_tx || siochan->idle || siochan->ring;
	spin_unlock_irqrestore(&siochan->vc.lock, flags);

	return busy;
}

/* One stopped in the commit stays up through the system suspend. */
static __maybe_unused int sio_system_suspend(struct device *dev)
{
	struct sio_data *sio = dev_get_drvdata(dev);

	return sio->hw->commit_stop ? 0 : pm_runtime_force_suspend(dev);
}

static __maybe_unused int sio_system_resume(struct device *dev)
{
	struct sio_data *sio = dev_get_drvdata(dev);

	return sio->hw->commit_stop ? 0 : pm_runtime_force_resume(dev);
}

static __maybe_unused int sio_suspend_late(struct device *dev)
{
	struct sio_data *sio = dev_get_drvdata(dev);
	int i, ret;

	if (!pm_suspend_via_firmware() || sio->asleep)
		return 0;

	for (i = 0; i < sio->nchannels; i++) {
		if (sio_chan_busy(&sio->channels[i])) {
			dev_err(dev, "channel %d is busy, refusing the sleep\n", i);
			return -EBUSY;
		}
	}

	if (sio->commit_registered && apple_sleep_commit_stops_coprocs()) {
		apple_sleep_coproc_set_ready(dev, true);
		/*
		 * The commit stops it, through its ASC and mailbox, after the noirq
		 * phase: keep its power domain up until then, as genpd would power it
		 * down with the device runtime-suspended. Read with the domain down,
		 * both answer with an external abort.
		 */
		device_set_wakeup_path(dev);
		sio->commit_used = true;
		sio->asleep = true;
		return 0;
	}
	if (sio->hw->commit_stop) {
		apple_sleep_coproc_set_ready(dev, true);
		sio->asleep = true;
		return 0;
	}

	/* Idled any earlier, a refusal after it would leave it stopped. */
	if (sio->hw->stop_at_commit && apple_sleep_commit_stops_coprocs()) {
		dev_err(dev, "no stop at the sleep's commit: refusing the sleep\n");
		return -EBUSY;
	}

	ret = apple_rtkit_sleep_stop(sio->rtk, sio->base + REG_CPU_CONTROL);
	if (ret) {
		dev_err(dev, "firmware did not stop for sleep: %d\n", ret);
		if (apple_rtkit_sleep_return(sio->rtk, sio->base + REG_CPU_CONTROL))
			dev_err(dev, "firmware did not come back\n");
		return ret;
	}
	sio->asleep = true;

	return 0;
}

static __maybe_unused int sio_resume_early(struct device *dev)
{
	struct sio_data *sio = dev_get_drvdata(dev);
	bool t6050;
	int ret;

	if (!sio->asleep)
		return 0;
	/* Left for the commit: if the sleep was refused before it, the firmware never idled. */
	if (sio->commit_used) {
		bool stopped = apple_sleep_coproc_stopped(dev);

		sio->commit_used = false;
		apple_sleep_coproc_set_ready(dev, false);
		if (!pm_resume_via_firmware() && !stopped) {
			sio->asleep = false;
			return 0;
		}
	}
	if (sio->hw->commit_stop)
		apple_sleep_coproc_set_ready(dev, false);
	/*
	 * A refused sleep: an idled firmware doesn't wake with its power kept, and SIO_CPU shares
	 * SIO's switch, so it can't be power-cycled alone. The sleep engine's own PMGR reset of it
	 * restarts it (S); if that fails it stays stopped until the next real sleep.
	 */
	if (!pm_resume_via_firmware()) {
		if (!apple_rtkit_sleep_return(sio->rtk, sio->base + REG_CPU_CONTROL)) {
			sio->asleep = false;
			return 0;
		}
		ret = apple_sleep_engine_reset(dev);
		if (!ret)
			ret = apple_rtkit_power_cycled_return(sio->rtk,
							      sio->base + REG_CPU_CONTROL);
		if (ret) {
			dev_warn(dev, "sleep refused: firmware not back after a reset (%d), stopped until a real sleep\n",
				 ret);
			return 0;
		}
		sio->asleep = false;
		dev_info(dev, "sleep refused: firmware back after a reset\n");
		return 0;
	}
	/* Endpoints and parameters stay with the firmware: nothing is sent again. */
	ret = apple_rtkit_sleep_return(sio->rtk, sio->base + REG_CPU_CONTROL);
	t6050 = sio->hw == &sio_hw_t6050;
	if (ret) {
		/* Still down: the next sleep restarts it, or resets it if refused. */
		dev_err(dev, "%sfirmware did not come back from sleep: %d\n",
			t6050 ? "SIO: not restarted after the sleep: " : "", ret);
		return 0;
	}
	sio->asleep = false;
	/* T6050: one line per wake: the start reply's line is the probe's alone. */
	if (t6050)
		dev_info(dev, "SIO: restarted after the sleep, firmware up, protocol v%u, CPU control 0x%x\n",
			 sio->protocol, readl(sio->base + REG_CPU_CONTROL));
	else
		dev_dbg(dev, "firmware back from sleep\n");

	return 0;
}

static const struct dev_pm_ops sio_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(sio_system_suspend, sio_system_resume)
	LATE_SYSTEM_SLEEP_PM_OPS(sio_suspend_late, sio_resume_early)
	RUNTIME_PM_OPS(sio_suspend, sio_resume, NULL)
};

static struct platform_driver apple_sio_driver = {
	.driver = {
		.name = "apple-sio",
		.of_match_table = sio_of_match,
		.pm             = pm_ptr(&sio_pm_ops),
	},
	.probe = sio_probe,
	.remove = sio_remove,
};
module_platform_driver(apple_sio_driver);

MODULE_AUTHOR("Martin Povišer <povik+lin@cutebit.org>");
MODULE_DESCRIPTION("Driver for SIO coprocessor on Apple SoCs");
MODULE_LICENSE("Dual MIT/GPL");

#if IS_ENABLED(CONFIG_APPLE_SIO_T6030_KUNIT_TEST)
#include "apple-sio-test.c"
#endif

#if IS_ENABLED(CONFIG_APPLE_SIO_T6041_KUNIT_TEST)
#include "apple-sio-t6041-test.c"
#endif

#if IS_ENABLED(CONFIG_APPLE_SIO_KUNIT_TEST)
#include "apple-sio-probe-test.c"
#include "apple-sio-boot-test.c"
#endif
