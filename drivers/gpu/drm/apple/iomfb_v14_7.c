// SPDX-License-Identifier: GPL-2.0-only OR MIT

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/workqueue.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/apple/pmp-report.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "afk.h"
#include "dcp.h"
#include "dcp-internal.h"
#include "dcpext_scanout.h"
#include "dcp-lifecycle.h"
#include "dptxep.h"
#include "iomfb_internal.h"
#include "iomfb_v14_7.h"
#include "iomfb_v14_7_board.h"
#include "iomfb_v14_7_link.h"
#include "iomfb_v14_7_swap.h"
#include "av.h"
#include "parser.h"

#define A(n) DCP_V14_TAG('A', n)
#define D(n) DCP_V14_TAG('D', n)

/* The images whose method and callback layouts this file implements. */
#define DCP_V14_FIRMWARE_UUID	"DDF38191-93B3-324A-BC8F-643006F5AC82"	/* T6030 */
#define DCP_V14_J613_FIRMWARE_UUID "90F849E1-B422-367E-B389-50246F8DEC47" /* J613 */
#define DCP_V14_J516C_FIRMWARE_UUID "F493F03E-CC40-3D89-9A08-5F259BA2B246" /* J516C */

#define DCP_V14_CPU_CONTROL	0x44
#define DCP_V14_CPU_STATUS	0x48
#define DCP_V14_PMP_TIMEOUT	msecs_to_jiffies(30000)
#define DCP_V14_RTKIT_RETRIES	4
#define DCP_V14_MAX_PROPERTIES	256
#define DCP_V14_MAX_RAW		32
/*
 * The size of one raw property and of all of them. An external processor
 * publishes an HDMI 2.1 (FRL) sink's TimingElements as one chunked transfer
 * of more than 1 MiB (1.38 MB for a 4K120 TV), so it keeps up to 8 MiB per
 * property and 24 MiB in all; the panel's properties stay small.
 */
#define DCP_V14_MAX_RAW_SIZE	SZ_1M
#define DCP_V14_MAX_RAW_BYTES	SZ_8M
#define DCP_V14_EXT_MAX_RAW_SIZE	SZ_8M
#define DCP_V14_EXT_MAX_RAW_BYTES	(24 * SZ_1M)
#define DCP_V14_MAX_BUFFERS	64
#define DCP_V14_MAX_BUFFER	SZ_16M
#define DCP_V14_MAX_BUFFER_BYTES SZ_64M
/* IOMFB layer of the primary plane. */
#define DCP_V14_LAYER		0
#define DCP_V14_BLACK		0xff000000

static const struct dcp_v14_board dcp_v14_board_t6030 = {
	.name = "T6030",
	.debugfs = "dcp-t6030",
	.dcp_compatible = "apple,t6030-dcp",
	.firmware_uuid = DCP_V14_FIRMWARE_UUID,
	.handoff = "apple,t6030-handoff",
	.promotion = true,
};

/*
 * J613 (MacBook Air 13", M3): a 2560x1664 panel at 60 Hz only, with 64 notch
 * rows above the 2560x1600 boot framebuffer.
 */
static const struct dcp_v14_board dcp_v14_board_j613 = {
	.name = "J613",
	.debugfs = "dcp-j613",
	.dcp_compatible = "apple,t8122-dcp",
	.machine = "apple,j613",
	.firmware_uuid = DCP_V14_J613_FIRMWARE_UUID,
	.handoff = "apple,t8122-handoff",
	.panel_width = 2560,
	.panel_height = 1664,
	.promotion = false,
	.ctm_set = A(421),
	.ctm_get = A(420),
};

/* J615 (MacBook Air 15", M3): the same DCP image, a 2880x1864 60 Hz panel. */
static const struct dcp_v14_board dcp_v14_board_j615 = {
	.name = "J615",
	.debugfs = "dcp-j615",
	.dcp_compatible = "apple,t8122-dcp",
	.machine = "apple,j615",
	.firmware_uuid = DCP_V14_J613_FIRMWARE_UUID,
	.handoff = "apple,t8122-handoff",
	.panel_width = 2880,
	.panel_height = 1864,
	.promotion = false,
	.ctm_set = A(421),
	.ctm_get = A(420),
};

/*
 * J516C (MacBook Pro 16", M3 Max): the J516S panel, 3456x2234 with 74 notch
 * rows above the 3456x2160 boot framebuffer, at 60 or 120 Hz, on the T6031
 * DCP, running this Mac's own 14.7 DCP image (the J516C ADT names it). The
 * 14.7 IOMFB interface follows the firmware version and has run unchanged on
 * the T6030 and T8122 images; this image's layouts are shown by its first
 * boots. Any other image is refused and the boot framebuffer stays. Every
 * panel timing and the display clock the firmware is told are logged: the
 * first boots on this board are read from the kernel log. The clock is the
 * T6030 rate (clk_disp0), unconfirmed on T6031.
 */
static const struct dcp_v14_board dcp_v14_board_j516c = {
	.name = "J516C",
	.debugfs = "dcp-j516c",
	.dcp_compatible = "apple,t6031-dcp",
	.machine = "apple,j516c",
	.firmware_uuid = DCP_V14_J516C_FIRMWARE_UUID,
	.handoff = "apple,t6031-handoff",
	.panel_width = 3456,
	.panel_height = 2234,
	.promotion = true,
	.log_bringup = true,
};

/*
 * J514C (MacBook Pro 14", M3 Max): the T6031 DCP running the J516C image
 * (the DCP image is the SoC's; its UUID is checked at probe), in the J514S
 * chassis, so the J514 panel: the board file names the J514S panel values,
 * not yet read from a J514C, and the panel size comes from the boot
 * framebuffer. Every panel timing and the display clock are logged.
 */
static const struct dcp_v14_board dcp_v14_board_j514c = {
	.name = "J514C",
	.debugfs = "dcp-j514c",
	.dcp_compatible = "apple,t6031-dcp",
	.machine = "apple,j514c",
	.firmware_uuid = DCP_V14_J516C_FIRMWARE_UUID,
	.handoff = "apple,t6031-handoff",
	.promotion = true,
	.log_bringup = true,
};

static const struct dcp_v14_board *const dcp_v14_boards[] = {
	&dcp_v14_board_t6030,
	&dcp_v14_board_j613,
	&dcp_v14_board_j615,
	&dcp_v14_board_j516c,
	&dcp_v14_board_j514c,
};

struct dcp_v14_property {
	u32 service;
	char key[64];
	u64 value;
};

struct dcp_v14_raw {
	u32 service;
	char key[64];
	void *data;
	u32 size;
};

struct dcp_v14_buffer {
	void *cpu;
	dma_addr_t iova;
	phys_addr_t phys;
	size_t size;
	bool retired;
	bool piodma_mapped;
};

struct apple_dcp_v14 {
	struct device *dev;
	/* The internal panel's board; NULL on an external processor. */
	const struct dcp_v14_board *board;
	/* Cleared on KMS unbind and on removal; the firmware session outlives it. */
	struct apple_dcp *dcp;
	struct apple_rtkit *rtk;
	struct dcp_v14_link link;

	/* Owns the RPC stream: start, swaps and idle callbacks. */
	struct mutex lock;
	/* The colour matrix the firmware holds matches the CRTC's. */
	bool ctm_valid;
	/* The firmware refused a matrix: none is sent until reboot. */
	bool ctm_disabled;
	u64 ctm_calls;
	/* The last firmware backlight correction notification; not a nits target. */
	u32 backlight_factor;
	u64 backlight_factor_updates;
	u32 ctm_status, ctm_get_status;
	u64 ctm_readback[9];
	struct work_struct idle_work;
	/* External: reports the display gone once the session stopped. */
	struct work_struct stopped_work;
	bool failed;
	bool started;
	/* Panel: the mailbox holds the firmware's messages through system sleep. */
	bool rx_held;

	/*
	 * An external processor: no panel. Its modes come from the
	 * TimingElements the firmware publishes for the attached display,
	 * and the kernel sets the mode (A411) and the power state (A472).
	 */
	bool external;
	/* External: start signal and first client open done. */
	bool opened;
	/* External: the display is powered (A472) in the mode last set. */
	bool powered;
	/*
	 * External: the mode last set (A411 then A472(1)) runs, from a
	 * description not changed since, with these ids; and swaps with a
	 * framebuffer completed since it was set.
	 */
	bool mode_live;
	u32 mode_timing, mode_color;
	u64 mode_swaps;
	/* External: catalog generation when a chunked property started. */
	u64 chunk_generation;
	/* External: the parts of the display's description published now. */
	unsigned int described;
	u64 description_generation;
	wait_queue_head_t described_wait;
	/* External: swaps the firmware reported dropped, and the status waits. */
	u64 swaps_dropped, status_waits, status_timeouts;
	u32 display_status;

	/* D300: id 19 is the logical level in nits (16.16), id 18 the correction. */
	u32 pr_nits, pr_correction;
	bool pr_nits_valid;

	/* Boot framebuffer and native panel timing (notch rows included). */
	u32 stride;
	u32 fb_width, fb_height;
	u32 panel_width, panel_height;
	/* The panel's notch rows (apple,notch-height), hidden or not. */
	u32 notch_rows;
	u64 clock_rate;

	struct dcp_v14_property properties[DCP_V14_MAX_PROPERTIES];
	u32 property_count;
	struct dcp_v14_raw raw[DCP_V14_MAX_RAW];
	u32 raw_count, raw_bytes;
	void *chunk;
	u32 chunk_size, chunk_offset;
	u64 analytics;

	struct dcp_v14_buffer buffers[DCP_V14_MAX_BUFFERS];
	u32 buffer_count;
	u64 buffer_bytes;
	struct platform_device *piodma;
	struct iommu_domain *piodma_domain;

	/* Scanned out until the next swap completes. */
	struct drm_framebuffer *active_fb;
	u64 swaps;
	u64 swap_ns_max;

	/*
	 * Panel: the last frame presented, repeated by a brightness-only
	 * present with the new level (backlight_work); not valid while the
	 * panel shows the blank background. Under the RPC lock.
	 */
	struct {
		bool valid;
		u8 surface[DCP_V14_SURFACE_SIZE];
		u64 iova;
		u32 width, height, dst_y;
	} last;
	struct delayed_work backlight_work;
	u32 nits_sent;
	u64 nits_presents, brightness_presents, nits_refused;
};

/*
 * The panel's DCP session; one per boot, RTKit is never started twice. The
 * manual external start waits for it, as the PMP runs once the panel's
 * request is acknowledged.
 */
static bool dcp_v14_panel_session;

static int dcp_v14_callback(void *cookie, u32 tag, const void *input, u32 in_size,
			    void *output, u32 out_size);
static void dcp_v14_backlight_work(struct work_struct *work);
static void dcp_v14_configure_backlight(struct apple_dcp *dcp);

/*
 * An external session stopped on a link error, not a crash: its display is
 * gone until reboot. Report that as the crash path does, from process
 * context, so that userspace stops driving the output.
 */
static void dcp_v14_external_stopped(struct apple_dcp_v14 *v14)
{
	if (v14->external)
		schedule_work(&v14->stopped_work);
}

static void dcp_v14_stopped_work(struct work_struct *work)
{
	struct apple_dcp_v14 *v14 = container_of(work, struct apple_dcp_v14, stopped_work);
	struct apple_dcp *dcp = READ_ONCE(v14->dcp);
	struct apple_connector *connector;

	if (!dcp || READ_ONCE(dcp->external_detached))
		return;
	connector = READ_ONCE(dcp->connector);
	dev_err(v14->dev, "external display session stopped: its display is reported disconnected\n");
	dcp_mode_invalidate(&dcp->mode_state);
	if (connector) {
		WRITE_ONCE(connector->connected, false);
		apple_connector_edid_set_live(connector, false);
		dcp_queue_hotplug(connector);
	}
}

/* Called with the lock held, or from a callback. */
static int dcp_v14_call_guarded(struct apple_dcp_v14 *v14, u32 tag, const void *in, u32 in_size,
			void *out, u32 out_size, u32 completion,
			dcp_v14_admit_fn admit, void *admit_cookie)
{
	bool vetoed;
	int ret;

	if (v14->failed)
		return -EIO;
	dev_dbg(v14->dev, "call %#x in %u out %u\n", tag, in_size, out_size);
	ret = dcp_v14_link_call_guarded(&v14->link, tag, in, in_size, out, out_size, completion,
				dcp_v14_callback, v14, admit, admit_cookie, &vetoed);
	if (ret && !vetoed) {
		v14->failed = true;
		dev_err(v14->dev, "DCP call %#x failed: %d; recovery requires a reboot\n",
			tag, ret);
		dcp_v14_external_stopped(v14);
	}
	return ret;
}

static int dcp_v14_call(struct apple_dcp_v14 *v14, u32 tag, const void *in, u32 in_size,
			void *out, u32 out_size, u32 completion)
{
	return dcp_v14_call_guarded(v14, tag, in, in_size, out, out_size,
				    completion, NULL, NULL);
}

/* A call with an optional u32 input of 1 and an optional u32 result to check. */
static int dcp_v14_simple_call(struct apple_dcp_v14 *v14, u32 tag, bool input, bool reply,
			       bool expected)
{
	__le32 one = cpu_to_le32(1), result = 0;
	int ret;

	ret = dcp_v14_call(v14, tag, input ? &one : NULL, input ? 4 : 0,
			   reply ? &result : NULL, reply ? 4 : 0, 0);
	if (!ret && reply && le32_to_cpu(result) != expected)
		return -EPROTO;
	return ret;
}

static struct dcp_v14_property *dcp_v14_property(struct apple_dcp_v14 *v14, u32 service,
						 const u8 *key, bool create)
{
	struct dcp_v14_property *p;
	u32 i;

	if (!memchr(key, 0, 64))
		return ERR_PTR(-EINVAL);
	for (i = 0; i < v14->property_count; i++) {
		p = &v14->properties[i];
		if (p->service == service && !strcmp(p->key, key))
			return p;
	}
	if (!create)
		return NULL;
	if (v14->property_count == DCP_V14_MAX_PROPERTIES)
		return ERR_PTR(-ENOSPC);
	p = &v14->properties[v14->property_count++];
	p->service = service;
	strscpy(p->key, key, sizeof(p->key));
	return p;
}

static u32 dcp_v14_raw_limit(const struct apple_dcp_v14 *v14)
{
	return v14->external ? DCP_V14_EXT_MAX_RAW_SIZE : DCP_V14_MAX_RAW_SIZE;
}

static u32 dcp_v14_raw_total(const struct apple_dcp_v14 *v14)
{
	return v14->external ? DCP_V14_EXT_MAX_RAW_BYTES : DCP_V14_MAX_RAW_BYTES;
}

static int dcp_v14_raw_property(struct apple_dcp_v14 *v14, u32 service, const u8 *key,
				const void *data, u32 size)
{
	u32 i, old_size = 0;
	void *copy;

	if (!memchr(key, 0, 64) || size > dcp_v14_raw_limit(v14))
		return -EINVAL;
	for (i = 0; i < v14->raw_count; i++)
		if (v14->raw[i].service == service && !strcmp(v14->raw[i].key, key))
			break;
	if (i == DCP_V14_MAX_RAW)
		return -ENOSPC;
	if (i < v14->raw_count)
		old_size = v14->raw[i].size;
	if (v14->raw_bytes - old_size + size > dcp_v14_raw_total(v14))
		return -ENOSPC;
	copy = kvmemdup(data, size ?: 1, GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	if (i == v14->raw_count)
		v14->raw_count++;
	kvfree(v14->raw[i].data);
	v14->raw[i].service = service;
	strscpy(v14->raw[i].key, key, sizeof(v14->raw[i].key));
	v14->raw[i].data = copy;
	v14->raw[i].size = size;
	v14->raw_bytes += size - old_size;
	return 0;
}

/* The firmware's allocate-buffer callback wants a physically contiguous buffer. */
static int dcp_v14_alloc(struct apple_dcp_v14 *v14, u64 request, u32 *id)
{
	struct dcp_v14_buffer *buf;
	struct sg_table table;
	dma_addr_t iova;
	size_t size;
	void *cpu;
	int ret;

	if (!request || request > DCP_V14_MAX_BUFFER)
		return -EINVAL;
	size = ALIGN(request, SZ_16K);
	if (v14->buffer_count == DCP_V14_MAX_BUFFERS ||
	    v14->buffer_bytes + size > DCP_V14_MAX_BUFFER_BYTES)
		return -ENOSPC;
	/*
	 * Contiguous memory comes from the CMA area first and from the page
	 * allocator when that is in use: the CMA miss is not a failure.
	 */
	cpu = dma_alloc_attrs(v14->dev, size, &iova, GFP_KERNEL | __GFP_NOWARN,
			      DMA_ATTR_FORCE_CONTIGUOUS);
	if (!cpu) {
		dev_err(v14->dev, "no contiguous memory for a %zu byte DCP buffer\n", size);
		return -ENOMEM;
	}
	ret = dma_get_sgtable_attrs(v14->dev, &table, cpu, iova, size,
				    DMA_ATTR_FORCE_CONTIGUOUS);
	if (ret)
		goto free;
	if (table.orig_nents != 1 || table.sgl->length < size) {
		sg_free_table(&table);
		ret = -ERANGE;
		goto free;
	}
	buf = &v14->buffers[v14->buffer_count];
	buf->phys = sg_phys(table.sgl);
	sg_free_table(&table);
	memset(cpu, 0, size);
	dma_wmb();
	buf->cpu = cpu;
	buf->iova = iova;
	buf->size = size;
	v14->buffer_bytes += size;
	/* The firmware takes id 0 as a failure. */
	*id = ++v14->buffer_count;
	dev_info(v14->dev, "DCP buffer %u: %zu bytes at %pad\n", *id, size, &iova);
	return 0;
free:
	dma_free_attrs(v14->dev, size, cpu, iova, DMA_ATTR_FORCE_CONTIGUOUS);
	return ret;
}

/* Maps a firmware buffer at the same address for the PIODMA stream. */
static int dcp_v14_map_piodma(struct apple_dcp_v14 *v14, u32 id)
{
	struct dcp_v14_buffer *buf = &v14->buffers[id - 1];
	struct device_node *node;
	size_t offset;
	u32 marker;
	int ret;

	if (buf->retired)
		return -EINVAL;
	if (buf->piodma_mapped)
		return 0;
	if (!v14->piodma) {
		/* The display gate adds an external pipe's piodma with its DART. */
		node = of_get_child_by_name(v14->dev->of_node, "piodma");
		if (!node || !of_device_is_available(node) ||
		    (!v14->external && !v14->board) ||
		    of_property_read_u32(node, v14->external ? "apple,t6030-dispext-handoff" :
						 v14->board->handoff, &marker) || marker != 1) {
			of_node_put(node);
			return -ENODEV;
		}
		v14->piodma = of_platform_device_create(node, NULL, v14->dev);
		if (!v14->piodma) {
			of_node_put(node);
			return -ENOMEM;
		}
		ret = dma_set_mask_and_coherent(&v14->piodma->dev, DMA_BIT_MASK(42));
		if (!ret)
			ret = of_dma_configure(&v14->piodma->dev, node, true);
		of_node_put(node);
		/* Never destroy a device attached to a locked DART stream. */
		if (ret)
			return ret;
		v14->piodma_domain = iommu_get_domain_for_dev(&v14->piodma->dev);
	}
	if (IS_ERR_OR_NULL(v14->piodma_domain))
		return -EIO;
	ret = iommu_map(v14->piodma_domain, buf->iova, buf->phys, buf->size,
			IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret)
		return ret;
	for (offset = 0; offset < buf->size; offset += SZ_16K)
		if (iommu_iova_to_phys(v14->piodma_domain, buf->iova + offset) !=
		    buf->phys + offset)
			return -EIO;
	buf->piodma_mapped = true;
	dev_info(v14->dev, "PIODMA mapped DCP buffer %u: %zu bytes at %pad\n",
		 id, buf->size, &buf->iova);
	return 0;
}

/* Parts of an external display's description, as the firmware publishes it. */
#define DCP_V14_DESC_TIMING	BIT(0)
#define DCP_V14_DESC_COLOR	BIT(1)
#define DCP_V14_DESC_ATTRS	BIT(2)
#define DCP_V14_DESC_ALL	(DCP_V14_DESC_TIMING | DCP_V14_DESC_COLOR | DCP_V14_DESC_ATTRS)
/* How long a mode set waits for a description being (re)published. */
#define DCP_V14_DESC_TIMEOUT_MS	3000
/* A withdrawn description that is not back by then gets its link redone. */
#define DCP_V14_DESC_RELINK_MS	2000

/* An external processor's default stride; no framebuffer is allocated for it. */
#define DCP_V14_EXT_STRIDE	(1920 * 4)
/* The display clock of every T6030 external pipe, unless the DT names one. */
#define DCP_V14_EXT_CLOCK	935000000ULL

/* Removes a raw property of service 0. */
static void dcp_v14_raw_remove(struct apple_dcp_v14 *v14, const char *key)
{
	u32 i;

	for (i = 0; i < v14->raw_count; i++) {
		if (v14->raw[i].service || strcmp(v14->raw[i].key, key))
			continue;
		v14->raw_bytes -= v14->raw[i].size;
		kvfree(v14->raw[i].data);
		v14->raw[i] = v14->raw[--v14->raw_count];
		memset(&v14->raw[v14->raw_count], 0, sizeof(v14->raw[0]));
		return;
	}
}

/* A call whose u32 answer is informational only. */
static int dcp_v14_query(struct apple_dcp_v14 *v14, u32 tag)
{
	__le32 result = 0;
	int ret;

	ret = dcp_v14_call(v14, tag, NULL, 0, &result, sizeof(result), 0);
	if (!ret)
		dev_dbg(v14->dev, "call %#x answered %u\n", tag, le32_to_cpu(result));
	return ret;
}

/*
 * The attached display came or went, as the firmware's display description
 * shows it. Mirrors the hotplug callback of the M1/M2 firmware.
 */
void dcp_v14_external_hotplug(struct apple_dcp *dcp, bool connected)
{
	struct apple_connector *connector = READ_ONCE(dcp->connector);
	unsigned int action;

	if (READ_ONCE(dcp->external_detached))
		return;

	/*
	 * Powering a Type-C CRTC off releases the link, and the firmware then
	 * withdraws the display. The cable is still in: keep the connector
	 * as it is and let the next power-on bring the link back.
	 */
	if (!connected && READ_ONCE(dcp->typec_crtc_off) &&
	    READ_ONCE(dcp->typec_cable_connected)) {
		dcp_mode_invalidate(&dcp->mode_state);
		return;
	}
	/* A description that outlives its cable is stale. */
	if (connected && dcp_is_typec_output(dcp) && !READ_ONCE(dcp->typec_cable_connected))
		return;
	/* Described again: a connector kept connected meanwhile is current. */
	if (connected)
		atomic_set(&dcp->external_held, 0);
	if (connected && dcp_is_typec_output(dcp))
		complete_all(&dcp->typec_iomfb_hpd_ready);
	if (!connected)
		WRITE_ONCE(dcp->ext_backlight, false);
	if (!connector)
		return;
	apple_connector_edid_set_live(connector, connected);
	action = dcp_mode_hotplug(&dcp->mode_state, connected, &connector->connected);
	if (connected && !READ_ONCE(dcp->mode_state.valid) &&
	    !READ_ONCE(dcp->mode_state.changing))
		action |= DCP_HOTPLUG_NOTIFY;
	if (!dcp->crtc)
		action &= ~DCP_HOTPLUG_VBLANK;
	dcp_handle_hotplug_actions(dcp, action);
}

/*
 * The firmware withdrew the description of a connected display whose port
 * still has HPD (@hdmi_hpd for the fixed HDMI output), and a retry is queued
 * that redoes the link if the description does not come back. Reporting the
 * connector disconnected until it is back lets a compositor that found it
 * connected a moment before read it back with no modes and no EDID, keep
 * that empty output once the display is back, or drop the output and add it
 * again. Keep it connected with the modes and EDID it has, until the
 * description is back or the retry finds it is not
 * (iomfb_v14_7_external_release()). A mode set meanwhile waits for the
 * description (dcp_v14_external_settle()).
 *
 * On a direct Type-C route the firmware can do this shortly after an attach.
 * Only the first withdrawal of a Type-C connection is held; later ones
 * disconnect, so a display that keeps being withdrawn is reported as it was
 * before. On the fixed HDMI output it can do this after every connect and
 * every mode set, so each withdrawal with HPD high is held. The queued retry
 * bounds that: with no retry left, a withdrawal disconnects as before.
 */
static bool dcp_v14_external_hold(struct apple_dcp *dcp, bool hdmi_hpd)
{
	struct apple_connector *connector = READ_ONCE(dcp->connector);
	u64 connection = READ_ONCE(dcp->typec_generation) + 1;
	bool typec = dcp_is_typec_output(dcp);
	unsigned int action;

	if (!dcp->external_native || READ_ONCE(dcp->external_detached) ||
	    !connector || !READ_ONCE(connector->connected) ||
	    !delayed_work_pending(&dcp->external_retry_wq))
		return false;
	if (typec) {
		if (dcp_is_usb4_output(dcp) || !READ_ONCE(dcp->typec_cable_connected) ||
		    READ_ONCE(dcp->typec_crtc_off) ||
		    READ_ONCE(dcp->external_held_connection) == connection)
			return false;
		WRITE_ONCE(dcp->external_held_connection, connection);
	} else if (!hdmi_hpd) {
		return false;
	}
	atomic_set(&dcp->external_held, 1);
	action = dcp_mode_withdraw(&dcp->mode_state);
	if (!dcp->crtc)
		action &= ~DCP_HOTPLUG_VBLANK;
	dcp_handle_hotplug_actions(dcp, action);
	dev_info(dcp->dev, "display withdrawn on %s with HPD: kept connected until it is described again\n",
		 typec ? "a Type-C route" : "the HDMI port");
	return true;
}

/* The attached sink is inactive (DPTX callback 20) until its link is up again. */
static bool dcp_v14_sink_inactive(struct apple_dcp *dcp)
{
	return READ_ONCE(dcp->dptxport[0].sink_inactive);
}

/*
 * The firmware reported the attached sink inactive (a TV in standby or on
 * another input), or active again at its next link-up. Its display stays
 * powered: the firmware retrains only a powered display's link, and that
 * link-up is how the sink comes back. Meanwhile it is reported disconnected,
 * no swap goes to it, and DPMS off does not power it down. During link
 * training the same callback is routine: an unpowered display's connector
 * is left as it is.
 */
void iomfb_v14_7_external_sink_active(struct apple_dcp *dcp, bool active)
{
	struct apple_dcp_v14 *v14 = dcp->v14;

	if (!dcp->external_native || !v14 || READ_ONCE(dcp->external_detached))
		return;
	if (!active) {
		if (!READ_ONCE(v14->powered))
			return;
		dev_info(dcp->dev, "display sink inactive: reported disconnected until its link is back\n");
		dcp_v14_external_hotplug(dcp, false);
		return;
	}
	dev_info(dcp->dev, "display sink active again\n");
	dcp_v14_external_hotplug(dcp, READ_ONCE(dcp->nr_modes) > 0);
}

/*
 * The firmware withdrew the timings of the attached display. An HPD bounce
 * withdraws the description, as can a connect or a mode set on the HDMI
 * port, and the firmware describes the display again once its link is back.
 * If that does not happen while the port still holds the display, redo the
 * link.
 */
void iomfb_v14_7_external_withdrawn(struct apple_dcp *dcp)
{
	bool hdmi_hpd = dcp_hdmi_port_attached(dcp);

	if ((READ_ONCE(dcp->typec_cable_connected) && !READ_ONCE(dcp->typec_crtc_off)) ||
	    hdmi_hpd)
		dcp_external_retry(dcp, "display withdrawn while its port has HPD",
				   0, DCP_V14_DESC_RELINK_MS);
	if (!dcp_v14_external_hold(dcp, hdmi_hpd))
		dcp_v14_external_hotplug(dcp, false);
}

/*
 * The retry found the display of a connector kept connected by
 * dcp_v14_external_hold() not described again: report it disconnected.
 * Returns whether it did.
 */
bool iomfb_v14_7_external_release(struct apple_dcp *dcp)
{
	struct apple_connector *connector = READ_ONCE(dcp->connector);

	if (!atomic_xchg(&dcp->external_held, 0) || !connector ||
	    !READ_ONCE(connector->connected))
		return false;
	dev_info(dcp->dev, "display not described again: reporting it disconnected\n");
	dcp_v14_external_hotplug(dcp, false);
	return true;
}

/*
 * A display description the firmware published (or removed, @removed) on an
 * external processor: the timings and color modes of the attached display
 * (from its EDID), its attributes, and the link transport. @generation is
 * the mode catalog generation when the transfer started.
 */
static void dcp_v14_external_published(struct apple_dcp_v14 *v14, const char *key,
				       u64 generation, bool removed)
{
	struct apple_dcp *dcp = READ_ONCE(v14->dcp);
	struct apple_connector *connector;
	struct dcp_v14_raw *raw = NULL;
	struct dcp_parse_ctx ctx;
	unsigned int part = 0;
	u32 i;
	int ret;

	if (!strcmp(key, "TimingElements"))
		part = DCP_V14_DESC_TIMING;
	else if (!strcmp(key, "ColorElements"))
		part = DCP_V14_DESC_COLOR;
	else if (!strcmp(key, "DisplayAttributes"))
		part = DCP_V14_DESC_ATTRS;
	else if (strcmp(key, "Transport"))
		return;
	for (i = 0; i < v14->raw_count && !removed; i++)
		if (!v14->raw[i].service && !strcmp(v14->raw[i].key, key))
			raw = &v14->raw[i];
	if (!removed && !raw)
		return;
	v14->description_generation++;
	/* The timings count once they parse; see below. */
	if (removed)
		v14->described &= ~part;
	else if (part != DCP_V14_DESC_TIMING)
		v14->described |= part;
	/* Mode ids refer to the timings and colors described when it was set. */
	if (part == DCP_V14_DESC_TIMING || part == DCP_V14_DESC_COLOR)
		v14->mode_live = false;
	/* A mode set waits for the whole description; see external_settle(). */
	wake_up_all(&v14->described_wait);
	if (!dcp)
		return;
	dev_info(dcp->dev, "display %s %s, %u bytes\n", key, removed ? "withdrawn" : "published",
		 raw ? raw->size : 0);
	/* DRM unbound the pipe: its connector and the users of its modes are gone. */
	if (READ_ONCE(dcp->external_detached))
		return;

	/* The connector's debugfs keeps a copy. */
	connector = READ_ONCE(dcp->connector);
	if (raw && raw->size && connector) {
		struct dcp_chunks chunks = {
			.length = raw->size,
			.data = kmemdup(raw->data, raw->size, GFP_KERNEL),
		};

		if (chunks.data)
			dcp_connector_update_dict(connector, key, &chunks);
	}

	if (!strcmp(key, "TimingElements")) {
		if (removed) {
			iomfb_v14_7_external_withdrawn(dcp);
			return;
		}
		ret = parse(raw->data, raw->size, &ctx);
		if (!ret) {
			ctx.dcp = dcp;
			/* The fw14 RGB8 profile, DSC-required colours the sink takes compressed. */
			ctx.v14_7_rgb8 = true;
			ctx.v14_7_dsc = true;
			/*
			 * The link's budget: the lanes the PHY reported to the
			 * firmware (GetMaxLaneCount) at the highest rate the
			 * port reports (HBR3 on every T6030 route).
			 */
			ctx.link_lanes = READ_ONCE(dcp->dptxport[0].lane_count);
			ctx.link_rate = LINK_RATE_HBR3;
			ret = dcp_modes_replace(dcp, &ctx, generation);
		}
		if (ret) {
			dev_warn(dcp->dev, "display timings not used: %d\n", ret);
			return;
		}
		if (READ_ONCE(dcp->nr_modes))
			v14->described |= DCP_V14_DESC_TIMING;
		wake_up_all(&v14->described_wait);
		dev_info(dcp->dev, "display has %u usable modes\n", READ_ONCE(dcp->nr_modes));
		/* A sink the firmware describes again is active: it reads only an active one. */
		if (READ_ONCE(dcp->nr_modes) && dcp_v14_sink_inactive(dcp)) {
			WRITE_ONCE(dcp->dptxport[0].sink_inactive, false);
			dev_info(dcp->dev, "display sink described again: active\n");
		}
		dcp_v14_external_hotplug(dcp, READ_ONCE(dcp->nr_modes) > 0);
	} else if (!strcmp(key, "DisplayAttributes") && !removed) {
		ret = parse(raw->data, raw->size, &ctx);
		if (!ret) {
			ctx.dcp = dcp;
			ret = dcp_attributes_replace(dcp, &ctx, generation);
		}
		if (ret)
			dev_warn(dcp->dev, "display attributes not used: %d\n", ret);
	}
}

/*
 * Callbacks that an external processor answers differently, or sends only
 * there. -ENOENT leaves @tag to the shared handler. A display's property
 * transfer that is malformed or too large is refused with a false reply:
 * it must not end the session.
 */
static int dcp_v14_external_callback(struct apple_dcp_v14 *v14, u32 tag, const u8 *in,
				     u32 in_size, u8 *out, u32 out_size)
{
	struct dcp_v14_property *p;
	u32 count, offset, service, key_offset;
	int ret;

#define SHAPE(i, o) (in_size == (i) && out_size == (o))
	/* Will power off, and a swap the firmware made itself: nothing to retire. */
	if ((tag == D(2) && SHAPE(0, 0)) || (tag == D(591) && SHAPE(20, 0)))
		return 0;
	/* Display DART power, a bool: the DART is powered with the processor. */
	if (tag == D(574) && SHAPE(4, 4))
		return in[0] > 1 ? -EINVAL : 0;
	/* Main-display query: answered by asking the firmware. */
	if (tag == D(599) && SHAPE(0, 0))
		return dcp_v14_query(v14, A(410));
	/* Late boot: the panel's sequence; the first answer differs here. */
	if (tag == D(121) && SHAPE(0, 4)) {
		ret = dcp_v14_query(v14, A(444));
		if (!ret)
			ret = dcp_v14_simple_call(v14, A(29), false, false, false);
		if (!ret)
			ret = dcp_v14_simple_call(v14, A(466), true, false, false);
		if (!ret)
			ret = dcp_v14_simple_call(v14, A(0), true, true, true);
		if (!ret)
			ret = dcp_v14_simple_call(v14, A(463), false, true, true);
		if (!ret)
			out[0] = 1;
		return ret;
	}
	/* PMU service matching; an external display has no panel backlight. */
	if (tag == D(206) && SHAPE(0, 4)) {
		ret = dcp_v14_query(v14, A(131));
		if (!ret)
			out[0] = 1;
		return ret;
	}
	if (tag == D(207) && SHAPE(0, 4)) {
		out[0] = 1;
		return 0;
	}
	/* Chunked property transfer: start, chunk, end. */
	if (tag == D(127) && SHAPE(4, 4)) {
		count = get_unaligned_le32(in);
		ret = -EINVAL;
		/* The announced count includes one byte that is never sent. */
		if (!v14->chunk && count && count <= dcp_v14_raw_limit(v14) + 1) {
			v14->chunk_size = count - 1;
			v14->chunk_offset = 0;
			v14->chunk = kvzalloc(max_t(u32, 1, v14->chunk_size), GFP_KERNEL);
			ret = v14->chunk ? 0 : -ENOMEM;
			if (v14->dcp)
				v14->chunk_generation = dcp_modes_transfer_begin(v14->dcp);
		}
		goto property_reply;
	}
	if (tag == D(128) && SHAPE(0x1008, 4)) {
		offset = get_unaligned_le32(in + 0x1000);
		count = get_unaligned_le32(in + 0x1004);
		ret = -EINVAL;
		if (v14->chunk && offset == v14->chunk_offset && count <= 4096 &&
		    offset <= v14->chunk_size && count <= v14->chunk_size - offset) {
			memcpy(v14->chunk + offset, in, count);
			v14->chunk_offset += count;
			ret = 0;
		}
		goto property_reply;
	}
	if (tag == D(129) && SHAPE(64, 4)) {
		ret = -EINVAL;
		if (v14->chunk && v14->chunk_offset == v14->chunk_size && memchr(in, 0, 64))
			ret = dcp_v14_raw_property(v14, 0, in, v14->chunk, v14->chunk_size);
		kvfree(v14->chunk);
		v14->chunk = NULL;
		if (!ret)
			dcp_v14_external_published(v14, in, v14->chunk_generation, false);
		goto property_reply;
	}
	/* Dictionary properties, kept raw. */
	if ((tag == D(413) && SHAPE(4168, 4)) ||
	    ((tag == D(552) || tag == D(561)) && SHAPE(4164, 4)) ||
	    (tag == D(567) && SHAPE(128, 4))) {
		key_offset = tag == D(413) ? 4 : 0;
		service = key_offset ? get_unaligned_le32(in) : 0;
		if (tag != D(567) && (in[key_offset + 4160] & 1)) {
			out[0] = 1;
			return 0;
		}
		count = tag == D(567) ? strnlen(in + 64, 64) : 4096;
		ret = dcp_v14_raw_property(v14, service, in + key_offset, in + key_offset + 64,
					   count);
		if (!ret && !service && v14->dcp)
			dcp_v14_external_published(v14, in + key_offset,
						   dcp_modes_transfer_begin(v14->dcp), false);
		goto property_reply;
	}
	/* Property removal, including the display's description on unplug. */
	if (tag == D(107) && SHAPE(64, 0)) {
		if (!memchr(in, 0, 64))
			return -EINVAL;
		dcp_v14_raw_remove(v14, in);
		p = dcp_v14_property(v14, 0, in, false);
		if (IS_ERR(p))
			return PTR_ERR(p);
		if (p) {
			*p = v14->properties[--v14->property_count];
			memset(&v14->properties[v14->property_count], 0, sizeof(*p));
		}
		dcp_v14_external_published(v14, in, 0, true);
		return 0;
	}
#undef SHAPE
	return -ENOENT;

property_reply:
	if (ret) {
		dev_warn_ratelimited(v14->dev, "display property refused: %d\n", ret);
		kvfree(v14->chunk);
		v14->chunk = NULL;
	}
	/* These answer a bool: false refuses the data, not the link. */
	out[0] = !ret;
	return 0;
}

/* Runs on the thread that owns the RPC stream; the lock is held. */
static int dcp_v14_callback(void *cookie, u32 tag, const void *input, u32 in_size,
			    void *output, u32 out_size)
{
	struct apple_dcp_v14 *v14 = cookie;
	const u8 *in = input;
	u8 *out = output;
	struct dcp_v14_property *p;
	u32 count, offset, service, key_offset, id;
	int ret;

	if (v14->external) {
		ret = dcp_v14_external_callback(v14, tag, in, in_size, out, out_size);
		if (ret != -ENOENT)
			return ret;
	}

#define SHAPE(i, o) (in_size == (i) && out_size == (o))
	/*
	 * Backlight correction notification: one signed 32-bit factor, no
	 * reply payload. The firmware applies it when the colour matrix
	 * changes; it is not a host request to set the panel brightness.
	 */
	if (tag == D(208) && SHAPE(4, 0)) {
		v14->backlight_factor = get_unaligned_le32(in);
		v14->backlight_factor_updates++;
		return 0;
	}
	/* get_time */
	if (tag == D(209) && SHAPE(0, 8)) {
		put_unaligned_le64(ktime_to_ms(ktime_get_real()), out);
		return 0;
	}
	/* No default framebuffer is allocated by this host. */
	if ((tag == D(596) && SHAPE(0, 4)) || (tag == D(582) && SHAPE(8, 4)))
		return 0;
	/*
	 * Tiling state get and set: event, parameter, u32 value and a nullable
	 * byte. Like the M1/M2 hosts: no tiled display, no setter.
	 */
	if ((tag == D(115) && SHAPE(16, 8)) || (tag == D(116) && SHAPE(16, 4))) {
		if (in[12] > 1)
			return -EINVAL;
		if (tag == D(115))
			out[4] = 1;
		return 0;
	}
	/*
	 * Analytics event: name[64], serialized dictionary[4096], nullable
	 * flag. The dictionary is in/out and the firmware parses it even on
	 * failure, so it is echoed back unchanged; status 0 accepts it.
	 */
	if (tag == D(114) && SHAPE(4164, 4100)) {
		if (!memchr(in, 0, 64) || in[4160] > 1 || (!in[4160] && in[64] != 'd'))
			return -EINVAL;
		if (!in[4160])
			memcpy(out, in + 64, 4096);
		put_unaligned_le32(0, out + 4096);
		v14->analytics++;
		dev_dbg(v14->dev, "analytics event %llu: %.64s\n", v14->analytics, in);
		return 0;
	}
	/* Swap information for a completed swap; the link checks its id. */
	if (tag == D(589) && SHAPE(0x6f0, 0))
		return 0;
	if ((tag == D(588) && SHAPE(8, 0)) || (tag == D(598) && SHAPE(0, 0)))
		return 0;
	/* Main-display query, answered by asking the firmware. */
	if (tag == D(599) && SHAPE(0, 0))
		return dcp_v14_simple_call(v14, A(410), false, true, true);
	/* Service creation and boot signals. */
	if ((tag == D(108) || tag == D(109) || tag == D(110) || tag == D(111) ||
	     tag == D(112) || tag == D(113) || tag == D(0) || tag == D(1)) && SHAPE(0, 4)) {
		out[0] = 1;
		return 0;
	}
	/* Hotplug: echo the tiled-display record when present. */
	if (tag == D(576) && SHAPE(88, 76)) {
		if (!(in[84] & 1))
			memcpy(out, in + 8, 76);
		return 0;
	}
	if (tag == D(577) && SHAPE(4, 0))
		return 0;
	/* Runtime property publication: the panel level and its correction. */
	if (tag == D(300) && SHAPE(16, 0)) {
		u32 id = get_unaligned_le32(in), value = get_unaligned_le32(in + 4);

		if (id == 19) {
			v14->pr_nits = value;
			v14->pr_nits_valid = true;
		} else if (id == 18) {
			v14->pr_correction = value;
		}
		return 0;
	}
	/* Chunked property transfer: start, chunk, end. */
	if (tag == D(127) && SHAPE(4, 4)) {
		count = get_unaligned_le32(in);
		if (v14->chunk || !count || count > dcp_v14_raw_limit(v14) + 1)
			return -EINVAL;
		v14->chunk_size = count - 1;
		v14->chunk_offset = 0;
		v14->chunk = kvzalloc(max_t(u32, 1, v14->chunk_size), GFP_KERNEL);
		if (!v14->chunk)
			return -ENOMEM;
		out[0] = 1;
		return 0;
	}
	if (tag == D(128) && SHAPE(0x1008, 4)) {
		offset = get_unaligned_le32(in + 0x1000);
		count = get_unaligned_le32(in + 0x1004);
		if (!v14->chunk || offset != v14->chunk_offset || count > 4096 ||
		    offset > v14->chunk_size || count > v14->chunk_size - offset)
			return -EINVAL;
		memcpy(v14->chunk + offset, in, count);
		v14->chunk_offset += count;
		out[0] = 1;
		return 0;
	}
	if (tag == D(129) && SHAPE(64, 4)) {
		if (!v14->chunk || v14->chunk_offset != v14->chunk_size || !memchr(in, 0, 64))
			return -EINVAL;
		ret = dcp_v14_raw_property(v14, 0, in, v14->chunk, v14->chunk_size);
		if (ret)
			return ret;
		kvfree(v14->chunk);
		v14->chunk = NULL;
		dev_dbg(v14->dev, "property %.64s: %u bytes\n", in, v14->chunk_size);
		out[0] = 1;
		return 0;
	}
	/*
	 * Display clock frequencies: index 0 is the display clock iBoot set
	 * up, index 1 is not used. Nothing here programs a clock.
	 */
	if (tag == D(408) && SHAPE(8, 8)) {
		count = get_unaligned_le32(in + 4);
		if (memcmp(in, "VORP", 4) || count > 1)
			return -EINVAL;
		put_unaligned_le64(count ? 0 : v14->clock_rate, out);
		if (v14->board && v14->board->log_bringup)
			dev_info(v14->dev, "display clock %u: %llu Hz\n", count,
				 get_unaligned_le64(out));
		else
			dev_dbg(v14->dev, "display clock %u: %llu Hz\n", count,
				get_unaligned_le64(out));
		return 0;
	}
	if (tag == D(125) && SHAPE(100, 36)) {
		count = get_unaligned_le32(in + 64);
		if (count > 8)
			return -EINVAL;
		memcpy(out, in + 68, count * 4);
		return 0;
	}
	/* Frame sync properties: echo them when present. */
	if (tag == D(6) && SHAPE(60, 56)) {
		if (!(in[56] & 1))
			memcpy(out, in, 56);
		return 0;
	}
	/* Dark boot and hibernation wake: no. */
	if ((tag == D(122) || tag == D(123)) && SHAPE(0, 4))
		return 0;
	/* Late boot: the calls the firmware expects from its host, in order. */
	if (tag == D(121) && SHAPE(0, 4)) {
		static const struct {
			u32 tag;
			bool input, reply, expected;
		} sequence[] = {
			{ A(444), false, true, true },
			{ A(29), false, false, false },
			{ A(466), true, false, false },
			{ A(0), true, true, true },
			{ A(463), false, true, true },
		};

		for (count = 0; count < ARRAY_SIZE(sequence); count++) {
			ret = dcp_v14_simple_call(v14, sequence[count].tag, sequence[count].input,
						  sequence[count].reply, sequence[count].expected);
			if (ret)
				return ret;
		}
		out[0] = 1;
		return 0;
	}
	/* Default stride: that of the boot framebuffer. */
	if (tag == D(101) && SHAPE(0, 4)) {
		put_unaligned_le32(v14->stride, out);
		return 0;
	}
	/* Real-time bandwidth: explicitly declined. */
	if (tag == D(3) && SHAPE(4, 60)) {
		put_unaligned_le32(1, out + 56);
		return 0;
	}
	if (tag == D(451) && SHAPE(20, 28)) {
		u32 alignment = get_unaligned_le32(in + 12);
		u64 size = get_unaligned_le64(in + 4);
		struct dcp_v14_buffer *buf;

		dev_dbg(v14->dev, "allocation flags %#x size %llu alignment %u\n",
			get_unaligned_le32(in), size, alignment);
		if (get_unaligned_le32(in) != 0x703 || !is_power_of_2(alignment) ||
		    alignment > SZ_16K || in[16] || in[17] || in[18])
			return -EINVAL;
		ret = dcp_v14_alloc(v14, size, &id);
		if (ret)
			return ret;
		buf = &v14->buffers[id - 1];
		if ((buf->phys | buf->iova) & (alignment - 1))
			return -EINVAL;
		put_unaligned_le64(buf->phys, out);
		put_unaligned_le64(buf->iova, out + 8);
		put_unaligned_le64(buf->size, out + 16);
		put_unaligned_le32(id, out + 24);
		return 0;
	}
	/* Map a buffer for PIODMA; the AP virtual address is not used. */
	if (tag == D(201) && SHAPE(12, 20)) {
		u64 buffer = get_unaligned_le64(in);

		if (!buffer || buffer > v14->buffer_count || get_unaligned_le32(in + 8) != 1)
			return -EINVAL;
		ret = dcp_v14_map_piodma(v14, buffer);
		if (ret)
			return ret;
		put_unaligned_le64(0, out);
		put_unaligned_le64(v14->buffers[buffer - 1].iova, out + 8);
		put_unaligned_le32(0, out + 16);
		return 0;
	}
	/* Release: the buffer stays allocated and mapped until reboot. */
	if (tag == D(454) && SHAPE(4, 4)) {
		id = get_unaligned_le32(in);
		if (!id || id > v14->buffer_count || v14->buffers[id - 1].retired)
			return -EINVAL;
		v14->buffers[id - 1].retired = true;
		out[0] = 1;
		return 0;
	}
	/* Raw property read. */
	if (tag == D(400) && SHAPE(76, 0xc04)) {
		count = get_unaligned_le32(in + 68);
		service = get_unaligned_le32(in);
		if (count > 0xc00 || (in[72] & 1) || !memchr(in + 4, 0, 64))
			return -EINVAL;
		for (offset = 0; offset < v14->raw_count; offset++) {
			struct dcp_v14_raw *raw = &v14->raw[offset];

			if (raw->service != service || strcmp(raw->key, in + 4))
				continue;
			if (raw->size > count)
				return -ENOSPC;
			memcpy(out, raw->data, raw->size);
			put_unaligned_le32(raw->size, out + 0xc00);
			break;
		}
		return 0;
	}
	/* Unsigned property read. */
	if (tag == D(401) && SHAPE(80, 12)) {
		/*
		 * The PMU temperature the M1/M2 mini-LED hosts report: a fixed
		 * placeholder in centidegrees, not a measurement.
		 */
		if (!memcmp(in, "SUMP", 4) && !strncmp(in + 4, "Temperature", 64)) {
			put_unaligned_le64(3029, out);
			out[8] = 1;
			return 0;
		}
		p = dcp_v14_property(v14, get_unaligned_le32(in), in + 4, false);
		if (IS_ERR(p))
			return PTR_ERR(p);
		if (p) {
			put_unaligned_le64(p->value, out);
			out[8] = 1;
		} else {
			dev_dbg(v14->dev, "no host property %#x %.64s\n",
				get_unaligned_le32(in), in + 4);
		}
		return 0;
	}
	/* Dictionary properties, kept raw. */
	if ((tag == D(413) && SHAPE(4168, 4)) ||
	    ((tag == D(552) || tag == D(561)) && SHAPE(4164, 4)) ||
	    (tag == D(567) && SHAPE(128, 4))) {
		key_offset = tag == D(413) ? 4 : 0;
		service = key_offset ? get_unaligned_le32(in) : 0;
		if (tag != D(567) && (in[key_offset + 4160] & 1)) {
			out[0] = 1;
			return 0;
		}
		count = tag == D(567) ? strnlen(in + 64, 64) : 4096;
		ret = dcp_v14_raw_property(v14, service, in + key_offset, in + key_offset + 64,
					   count);
		if (!ret)
			out[0] = 1;
		return ret;
	}
	/* Number and boolean properties. */
	if ((tag == D(563) && SHAPE(76, 4)) || (tag == D(565) && SHAPE(72, 4)) ||
	    (tag == D(414) && SHAPE(80, 4)) || (tag == D(415) && SHAPE(76, 4)) ||
	    ((tag == D(102) || tag == D(104)) && SHAPE(68, 0))) {
		key_offset = (tag == D(414) || tag == D(415)) ? 4 : 0;
		service = key_offset ? get_unaligned_le32(in) : 0;
		count = (tag == D(563) || tag == D(414)) ? 8 : 4;
		if (out_size && (in[key_offset + 64 + count] & 1)) {
			out[0] = 1;
			return 0;
		}
		p = dcp_v14_property(v14, service, in + key_offset, true);
		if (IS_ERR(p))
			return PTR_ERR(p);
		p->value = count == 8 ? get_unaligned_le64(in + key_offset + 64) :
					get_unaligned_le32(in + key_offset + 64);
		if (tag == D(104))
			p->value = in[64];
		if (out_size)
			out[0] = 1;
		return 0;
	}
	/*
	 * Property operations a host-service proxy replays when it starts
	 * with queued operations. They return nothing.
	 */
	if ((tag == D(402) && SHAPE(76, 0)) || (tag == D(404) && SHAPE(72, 0)) ||
	    (tag == D(406) && SHAPE(72, 0)) || (tag == D(407) && SHAPE(72, 0))) {
		dev_dbg(v14->dev, "proxy property callback %#x\n", tag);
		return 0;
	}
	/* Property removal. */
	if (tag == D(107) && SHAPE(64, 0)) {
		p = dcp_v14_property(v14, 0, in, false);
		if (IS_ERR(p))
			return PTR_ERR(p);
		if (p) {
			*p = v14->properties[--v14->property_count];
			memset(&v14->properties[v14->property_count], 0, sizeof(*p));
		}
		return 0;
	}
	/* PMU and backlight service matching, answered by the firmware. */
	if ((tag == D(206) || tag == D(207)) && SHAPE(0, 4)) {
		ret = dcp_v14_simple_call(v14, tag == D(206) ? A(131) : A(132), false, true,
					  false);
		if (!ret)
			out[0] = 1;
		return ret;
	}
	if (tag == D(100) && SHAPE(0, 0)) {
		__le32 result;

		return dcp_v14_call(v14, A(374), NULL, 0, &result, sizeof(result), 0);
	}
#undef SHAPE
	if (v14->external) {
		/* A zeroed reply reads as false or none; keep the display session. */
		dev_warn_ratelimited(v14->dev, "unhandled DCP callback %#x %u/%u, answered with zeros\n",
				     tag, in_size, out_size);
		return 0;
	}
	dev_err(v14->dev, "unhandled DCP callback %#x %u/%u\n", tag, in_size, out_size);
	return -EOPNOTSUPP;
}

/* Firmware notifications that arrive while no call is in progress. */
static void dcp_v14_idle(struct work_struct *work)
{
	struct apple_dcp_v14 *v14 = container_of(work, struct apple_dcp_v14, idle_work);
	int ret = 0;

	mutex_lock(&v14->lock);
	while (!v14->failed && !ret)
		ret = dcp_v14_link_pump(&v14->link, 0, dcp_v14_callback, v14);
	if (ret && ret != -ETIMEDOUT && ret != -EAGAIN) {
		v14->failed = true;
		dev_err(v14->dev, "DCP notification failed: %d; recovery requires a reboot\n", ret);
		dcp_v14_external_stopped(v14);
	}
	mutex_unlock(&v14->lock);
}

static void dcp_v14_crashed(void *cookie, const void *crashlog, size_t crashlog_size)
{
	struct apple_dcp_v14 *v14 = cookie;
	struct apple_dcp *dcp = READ_ONCE(v14->dcp);

	WRITE_ONCE(v14->failed, true);
	if (dcp) {
		struct apple_connector *connector = READ_ONCE(dcp->connector);

		WRITE_ONCE(dcp->crashed, true);
		if (dcp->external)
			dcpext_scanout_fault(dcp, -EIO);
		/* The display it drove is gone until reboot. */
		if (v14->external && connector && !READ_ONCE(dcp->external_detached)) {
			WRITE_ONCE(connector->connected, false);
			apple_connector_edid_set_live(connector, false);
			dcp_queue_hotplug(connector);
		}
	}
	dev_err(v14->dev, "DCP firmware crashed; its buffers are kept until reboot\n");
	dcp_v14_link_fail(&v14->link);
}

static void dcp_v14_recv(void *cookie, u8 endpoint, u64 message)
{
	struct apple_dcp_v14 *v14 = cookie;
	/* NULL once removal has started; removal waits for this callback. */
	struct apple_dcp *dcp = READ_ONCE(v14->dcp);

	if (endpoint == DISP0_ENDPOINT && dcp && dcp->external) {
		if (dcp->ibootep)
			afk_receive_message(dcp->ibootep, message);
		return;
	}
	if (endpoint == DPTX_ENDPOINT) {
		dev_info(v14->dev, "DPTX message %#llx\n", message);
		if (dcp && dcp->dptxep)
			afk_receive_message(dcp->dptxep, message);
		return;
	}
	if (endpoint == DPAV_CTRL_ENDPOINT) {
		dev_info(v14->dev, "DPAV message %#llx\n", message);
		if (dcp && dcp->dpavctrlep)
			afk_receive_message(dcp->dpavctrlep, message);
		return;
	}
	if (endpoint == AV_ENDPOINT) {
		dev_info(v14->dev, "AV message %#llx\n", message);
		if (dcp && dcp->avep)
			afk_receive_message(dcp->avep, message);
		return;
	}
	if (endpoint == DPAVSERV_ENDPOINT) {
		dev_info(v14->dev, "DPAVSERV message %#llx\n", message);
		if (dcp && dcp->dcpavservep)
			afk_receive_message(dcp->dcpavservep, message);
		return;
	}
	if (endpoint != APPLE_DCP_LINK_ENDPOINT) {
		dev_dbg(v14->dev, "ignored endpoint %#x message %#llx\n", endpoint, message);
		return;
	}
	dcp_v14_link_receive(&v14->link, message);
}

/*
 * Buffers the firmware already owns must lie in a reserved region the boot
 * loader described, and be mapped linearly there. The OS log is a physical
 * address, not a DART address.
 */
static int dcp_v14_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_dcp_v14 *v14 = cookie;
	struct device *dev = v14->dev;
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);
	phys_addr_t phys;
	int i;

	if (!bfr->size || bfr->size > SZ_16M || !domain)
		return -EINVAL;
	if (!bfr->iova) {
		bfr->buffer = dma_alloc_coherent(dev, bfr->size, &bfr->iova, GFP_KERNEL);
		if (!bfr->buffer)
			return -ENOMEM;
		memset(bfr->buffer, 0, bfr->size);
		dma_wmb();
		return 0;
	}
	phys = iommu_iova_to_phys(domain, bfr->iova);
	for (i = 0; ; i++) {
		struct device_node *node = of_parse_phandle(dev->of_node, "memory-region", i);
		struct resource res;
		size_t off;
		bool oslog, valid;

		if (!node)
			return -ERANGE;
		oslog = of_property_read_bool(node, "apple,dcp-os-log");
		valid = !of_address_to_resource(node, 0, &res);
		of_node_put(node);
		if (!valid)
			continue;
		if (oslog && bfr->iova == res.start)
			phys = res.start;
		if (phys < res.start || phys > res.end || bfr->size - 1 > res.end - phys)
			continue;
		if (!oslog) {
			for (off = 0; off < bfr->size; off += SZ_16K)
				if (iommu_iova_to_phys(domain, bfr->iova + off) != phys + off)
					return -ERANGE;
			if (iommu_iova_to_phys(domain, bfr->iova + bfr->size - 1) !=
			    phys + bfr->size - 1)
				return -ERANGE;
		}
		bfr->buffer = memremap(phys, bfr->size, MEMREMAP_WB);
		if (!bfr->buffer)
			return -ENOMEM;
		bfr->is_mapped = true;
		dev_dbg(dev, "RTKit buffer at %pad: %pa, %zu bytes\n", &bfr->iova, &phys,
			bfr->size);
		return 0;
	}
}

static void dcp_v14_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_dcp_v14 *v14 = cookie;

	if (bfr->is_mapped)
		memunmap(bfr->buffer);
	else
		dma_free_coherent(v14->dev, bfr->size, bfr->buffer, bfr->iova);
}

static const struct apple_rtkit_ops dcp_v14_rtkit_ops = {
	.crashed = dcp_v14_crashed,
	.recv_message = dcp_v14_recv,
	.shmem_setup = dcp_v14_shmem_setup,
	.shmem_destroy = dcp_v14_shmem_destroy,
};

/* Freed only if RTKit never started: the firmware may call into it until reboot. */
static void dcp_v14_release(void *data)
{
	struct apple_dcp_v14 *v14 = data;

	if (!v14->rtk)
		kfree(v14);
}

/* Read only: bind maps and claims these registers. */
static int dcp_v14_cpu_running(struct device *dev, const char *name)
{
	struct resource *res;
	void __iomem *coproc;
	u32 control;

	res = platform_get_resource_byname(to_platform_device(dev), IORESOURCE_MEM, "coproc");
	if (!res)
		return -ENODEV;
	coproc = ioremap_np(res->start, resource_size(res));
	if (!coproc)
		return -ENOMEM;
	control = readl(coproc + DCP_V14_CPU_CONTROL);
	iounmap(coproc);
	if (!(control & APPLE_DCP_COPROC_CPU_CONTROL_RUN))
		return dev_err_probe(dev, -EBUSY,
				     "%s display not started: the DCP CPU is stopped (%#x); the PMP is not started for it\n",
				     name, control);
	return 0;
}

/*
 * The board of an internal 14.x DCP node: NULL if the node is not one, an
 * error if it is but this machine has no board record.
 */
const struct dcp_v14_board *dcp_v14_board_select(dcp_v14_compat_fn dcp_is,
						 dcp_v14_compat_fn machine_is,
						 const void *ctx, bool *dcp_known)
{
	const struct dcp_v14_board *board;
	unsigned int i;

	*dcp_known = false;
	for (i = 0; i < ARRAY_SIZE(dcp_v14_boards); i++) {
		board = dcp_v14_boards[i];
		if (!dcp_is(ctx, board->dcp_compatible))
			continue;
		*dcp_known = true;
		if (board->machine && !machine_is(ctx, board->machine))
			continue;
		return board;
	}
	return NULL;
}

bool dcp_v14_board_admits(const struct dcp_v14_board *board, const char *uuid)
{
	return uuid && !strcmp(uuid, board->firmware_uuid);
}

static bool dcp_v14_node_is(const void *np, const char *compat)
{
	return of_device_is_compatible(np, compat);
}

static bool dcp_v14_machine_is(const void *np, const char *compat)
{
	return of_machine_is_compatible(compat);
}

const struct dcp_v14_board *iomfb_v14_7_board(struct device *dev)
{
	const struct dcp_v14_board *board;
	bool dcp_known;

	board = dcp_v14_board_select(dcp_v14_node_is, dcp_v14_machine_is, dev->of_node,
				     &dcp_known);
	if (board)
		return board;
	if (dcp_known) {
		dev_err(dev, "display not started: no board record for this machine\n");
		return ERR_PTR(-ENODEV);
	}
	return NULL;
}

const char *iomfb_v14_7_board_name(const struct dcp_v14_board *board)
{
	return board->name;
}

/*
 * Whether the CRTC may offer a colour matrix: only a panel whose board
 * record names a tested matrix setter sends one (dcp_v14_ctm_locked()). On
 * every other 14.7 pipe a matrix would be accepted and never applied, so
 * userspace must not be offered one.
 */
bool iomfb_v14_7_ctm_supported(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14 = dcp->v14;

	return v14 && v14->board && v14->board->ctm_set;
}

int iomfb_v14_7_probe(struct apple_dcp *dcp)
{
	struct device *dev = dcp->dev;
	struct device_node *np = dev->of_node, *entry;
	const struct dcp_v14_board *board = iomfb_v14_7_board(dev);
	struct apple_dcp_v14 *v14;
	const char *uuid = NULL;
	u32 marker = 0;
	int ret;

	if (IS_ERR_OR_NULL(board))
		return -ENODEV;
	if (of_property_read_string(np, "apple,firmware-uuid", &uuid) ||
	    !dcp_v14_board_admits(board, uuid))
		return dev_err_probe(dev, -ENODEV,
				     "%s display not started: DCP firmware %s is not supported\n",
				     board->name, uuid ?: "(unknown)");
	if (of_property_read_u32(np, board->handoff, &marker) || marker != 1)
		return dev_err_probe(dev, -ENODEV,
				     "%s display not started: no boot loader display handoff\n",
				     board->name);
	if (!iommu_get_domain_for_dev(dev))
		return dev_err_probe(dev, -ENODEV,
				     "%s display not started: the DCP has no DART domain\n",
				     board->name);
	ret = dcp_v14_cpu_running(dev, board->name);
	if (ret)
		return ret;

	entry = of_parse_phandle(np, "apple,pmp-report", 0);
	if (!entry)
		return dev_err_probe(dev, -ENODEV,
				     "%s display not started: no apple,pmp-report, the PMP is not described\n",
				     board->name);
	ret = apple_pmp_report_wait_ready(entry, DCP_V14_PMP_TIMEOUT);
	of_node_put(entry);
	if (ret == -EPROBE_DEFER)
		return dev_err_probe(dev, ret, "waiting for the PMP report\n");
	if (ret)
		return dev_err_probe(dev, ret,
				     "%s display not started: the PMP has not acknowledged the display request; the boot framebuffer stays\n",
				     board->name);

	v14 = kzalloc_obj(*v14);
	if (!v14)
		return -ENOMEM;
	v14->dev = dev;
	v14->board = board;
	v14->dcp = dcp;
	mutex_init(&v14->lock);
	init_waitqueue_head(&v14->described_wait);
	INIT_WORK(&v14->idle_work, dcp_v14_idle);
	ret = devm_add_action_or_reset(dev, dcp_v14_release, v14);
	if (ret)
		return ret;
	dcp->v14 = v14;
	dev_info(dev, "PMP running with the display request acknowledged\n");
	return 0;
}

/*
 * The boot framebuffer gives the stride and the panel size. The boot loader
 * either hides the notch rows from it (the default) or keeps them;
 * apple,notch-height is recorded either way. A board with a known panel
 * tells the two apart by the height; any other board is taken to have the
 * notch hidden.
 */
static int dcp_v14_geometry(struct apple_dcp_v14 *v14)
{
	struct device_node *fb = of_find_compatible_node(NULL, NULL, "simple-framebuffer");
	u32 notch = 0;
	int ret;

	if (!fb)
		return -ENODEV;
	ret = of_property_read_u32(fb, "width", &v14->fb_width);
	if (!ret)
		ret = of_property_read_u32(fb, "height", &v14->fb_height);
	if (!ret)
		ret = of_property_read_u32(fb, "stride", &v14->stride);
	of_node_put(fb);
	if (ret)
		return ret;
	of_property_read_u32(v14->dev->of_node, "apple,notch-height", &notch);
	/* The firmware takes this stride as its default: 4 bytes per pixel. */
	if (!v14->fb_width || !v14->fb_height || v14->stride != v14->fb_width * 4 ||
	    notch > MAX_NOTCH_HEIGHT)
		return -EINVAL;
	v14->notch_rows = notch;
	v14->panel_width = v14->fb_width;
	v14->panel_height = v14->fb_height + notch;
	/* The boot loader kept the notch rows: the framebuffer is the whole panel. */
	if (v14->board->panel_height && notch && v14->fb_height == v14->board->panel_height)
		v14->panel_height = v14->fb_height;
	/* A board with a known panel takes no other. */
	if (v14->board->panel_width &&
	    (v14->panel_width != v14->board->panel_width ||
	     v14->panel_height != v14->board->panel_height)) {
		dev_err(v14->dev, "boot framebuffer %ux%u with %u notch rows is not the %ux%u %s panel\n",
			v14->fb_width, v14->fb_height, notch, v14->board->panel_width,
			v14->board->panel_height, v14->board->name);
		return -EINVAL;
	}
	return 0;
}

/* An external processor's firmware: once started, it runs until reboot. */
enum {
	DCPEXT_IDLE,
	DCPEXT_STARTING,
	DCPEXT_RUNNING,
	DCPEXT_FAILED,
};

static void dcpext_bringup(struct work_struct *work);

/*
 * The IOVA at which @mem's iommu-addresses map @size bytes for the device
 * @np, as the IOMMU core reads them; 0 if they do not.
 */
static u64 dcpext_region_iova(struct device_node *np, struct device_node *mem, u64 size)
{
	const __be32 *maps, *end;
	int len;

	maps = of_get_property(mem, "iommu-addresses", &len);
	if (!maps || len <= 0 || len % sizeof(*maps))
		return 0;
	end = maps + len / sizeof(*maps);
	while (maps < end) {
		struct device_node *owner = of_find_node_by_phandle(be32_to_cpup(maps++));
		phys_addr_t iova;
		size_t length;
		bool mine = owner == np;

		if (!owner)
			return 0;
		maps = of_translate_dma_region_checked(owner, maps, end - maps, &iova, &length);
		of_node_put(owner);
		if (!maps)
			return 0;
		if (mine && length == size)
			return iova;
	}
	return 0;
}

/*
 * Verify the mappings the IOMMU core installed from the boot loader's
 * handoff, before touching the processor: each region is mapped linearly at
 * the address its iommu-addresses give this processor.
 */
static int dcpext_verify_memory(struct apple_dcp *dcp)
{
	static const char *const names[] = {"asc-firmware", "dcp_data", "heap"};
	struct device *dev = dcp->dev;
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);
	u32 ready;
	int i;

	if (!domain || of_property_read_u32(dev->of_node,
		"apple,t6030-dcpext-memory-ready", &ready) || ready != 1)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		struct device_node *mem;
		struct resource res;
		u64 off, size, iova;
		int idx = of_property_match_string(dev->of_node, "memory-region-names", names[i]);
		if (idx < 0)
			return idx;
		mem = of_parse_phandle(dev->of_node, "memory-region", idx);
		if (!mem)
			return -EINVAL;
		if (!of_property_present(mem, "no-map") || of_address_to_resource(mem, 0, &res)) {
			of_node_put(mem);
			return -EINVAL;
		}
		size = resource_size(&res);
		iova = dcpext_region_iova(dev->of_node, mem, size);
		of_node_put(mem);
		if (!size || size > SZ_256M || !IS_ALIGNED(res.start, SZ_16K) ||
		    !IS_ALIGNED(size, SZ_16K) || !iova || !IS_ALIGNED(iova, SZ_16K))
			return -EINVAL;
		for (off = 0; off < size; off += SZ_16K) {
			if (iommu_iova_to_phys(domain, iova + off) != res.start + off) {
				dev_err(dev, "dcpext %s mapping mismatch at %#llx; CPU untouched\n",
					names[i], iova + off);
				return -EINVAL;
			}
		}
		dev_info(dev, "dcpext verified %s: %pa + %#llx at %#llx\n",
			 names[i], &res.start, size, iova);
	}
	return 0;
}

static ssize_t dcpext_start_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	bool start;
	int ret = kstrtobool(buf, &start);

	if (ret || !start)
		return -EINVAL;
	/* Serialize the one-shot handoff with idle system suspend. */
	guard(mutex)(&dcp->hpd_mutex);
	if (READ_ONCE(dcp->external_suspended))
		return -EBUSY;
	if (!dcp_v14_panel_session)
		return -EAGAIN;
	if (atomic_cmpxchg(&dcp->external_requested, 0, 1))
		return -EBUSY;
	/* Keep code and firmware callbacks alive after this one-shot request. */
	__module_get(THIS_MODULE);
	schedule_work(&dcp->external_work);
	return count;
}
static DEVICE_ATTR_WO(dcpext_start);

static struct attribute *dcpext_attrs[] = { &dev_attr_dcpext_start.attr, NULL };
static const struct attribute_group dcpext_group = { .attrs = dcpext_attrs };

static void dcpext_cancel(void *data)
{
	struct apple_dcp *dcp = data;

	cancel_work_sync(&dcp->external_work);
	if (dcp->external_native) {
		cancel_work_sync(&dcp->external_ready_work);
		cancel_delayed_work_sync(&dcp->external_retry_wq);
	}
}

/* The manual diagnostic path: memory checks, then an explicit start. */
int iomfb_v14_7_external_start(struct apple_dcp *dcp)
{
	int ret;

	/* This path does not run component bind, which sets the panel's mask.
	 * The external DART window also starts above 1 TiB.
	 */
	ret = dma_set_mask_and_coherent(dcp->dev, DMA_BIT_MASK(42));
	if (ret)
		return dev_err_probe(dcp->dev, ret, "dcpext requires 42-bit DMA\n");
	ret = dcpext_verify_memory(dcp);
	if (ret)
		return ret;
	atomic_set(&dcp->external_requested, 0);
	INIT_WORK(&dcp->external_work, dcpext_bringup);
	ret = devm_add_action_or_reset(dcp->dev, dcpext_cancel, dcp);
	if (ret)
		return ret;
	ret = devm_device_add_group(dcp->dev, &dcpext_group);
	if (!ret)
		dev_info(dcp->dev, "dcpext memory verified; waiting for explicit dcpext_start\n");
	return ret;
}

static void dcpext_ready_work(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(work, struct apple_dcp, external_ready_work);

	dcp_external_ready(dcp);
}

/*
 * The native path, at probe: the same checks as a manual start, without a
 * start. The coprocessor registers are mapped here, once, for the start
 * work and component bind alike. A failed check leaves the pipe in the DRM
 * device, never started, so that the internal panel is not held up.
 */
int iomfb_v14_7_external_prepare(struct apple_dcp *dcp)
{
	struct device *dev = dcp->dev;
	const char *uuid = NULL;
	int ret;

	atomic_set(&dcp->external_requested, 0);
	atomic_set(&dcp->external_retries, 0);
	WRITE_ONCE(dcp->external_phase, DCPEXT_IDLE);
	INIT_WORK(&dcp->external_work, dcpext_bringup);
	INIT_WORK(&dcp->external_ready_work, dcpext_ready_work);
	INIT_DELAYED_WORK(&dcp->external_retry_wq, dcp_external_retry_work);
	/* Component bind enables it, once a connector can be retrained. */
	disable_delayed_work(&dcp->external_retry_wq);
	ret = devm_add_action_or_reset(dev, dcpext_cancel, dcp);
	if (ret)
		return ret;

	dcp->coproc_reg = devm_platform_ioremap_resource_byname(to_platform_device(dev), "coproc");
	if (IS_ERR(dcp->coproc_reg)) {
		ret = PTR_ERR(dcp->coproc_reg);
		dcp->coproc_reg = NULL;
		goto refuse;
	}
	/* The callbacks here are those of one firmware image. */
	if (of_property_read_string(dev->of_node, "apple,firmware-uuid", &uuid) ||
	    strcmp(uuid, DCP_V14_FIRMWARE_UUID)) {
		dev_err(dev, "external display processor firmware %s is not supported\n",
			uuid ?: "(unknown)");
		ret = -ENODEV;
		goto refuse;
	}
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (!ret)
		ret = dcpext_verify_memory(dcp);
	if (ret)
		goto refuse;
	dev_info(dev, "external display processor ready; its firmware starts when a display is attached\n");
	return 0;
refuse:
	WRITE_ONCE(dcp->external_phase, DCPEXT_FAILED);
	dev_err(dev, "external display processor unusable: handoff check failed: %d\n", ret);
	return 0;
}

/* Probe: the pipe stays in the DRM device, never started. */
void iomfb_v14_7_external_refuse(struct apple_dcp *dcp, const char *why)
{
	WRITE_ONCE(dcp->external_phase, DCPEXT_FAILED);
	dev_err(dcp->dev, "external display processor unusable: %s\n", why);
}

/* True once DPTX can be used; asks for the start if nothing has. */
bool iomfb_v14_7_external_ready(struct apple_dcp *dcp)
{
	switch (smp_load_acquire(&dcp->external_phase)) {
	case DCPEXT_RUNNING:
		return smp_load_acquire(&dcp->dptxport[0].enabled);
	case DCPEXT_IDLE:
		/*
		 * Under the lock system sleep's prepare and suspend check the
		 * phase under: a start is requested before they look, and then
		 * refuses sleep, or not between them and complete, when the
		 * next attach starts it. The start work is frozen with tasks.
		 */
		scoped_guard(mutex, &dcp->hpd_mutex) {
			if (READ_ONCE(dcp->external_suspended) ||
			    smp_load_acquire(&dcp->external_phase) != DCPEXT_IDLE)
				return false;
			if (!atomic_cmpxchg(&dcp->external_requested, 0, 1)) {
				WRITE_ONCE(dcp->external_phase, DCPEXT_STARTING);
				dev_info(dcp->dev, "display attached: starting the external display processor\n");
				/* Firmware callbacks may outlive any later module removal. */
				__module_get(THIS_MODULE);
				queue_work(system_freezable_wq, &dcp->external_work);
			}
		}
		return false;
	default:
		return false;
	}
}

/*
 * What this processor's firmware holds that system sleep would not hand
 * back, or NULL: a start in progress, or a display link or a powered
 * display. The caller also refuses sleep while a display is attached.
 */
const char *iomfb_v14_7_external_busy(struct apple_dcp *dcp)
{
	switch (smp_load_acquire(&dcp->external_phase)) {
	case DCPEXT_STARTING:
		return "its firmware is starting";
	case DCPEXT_RUNNING:
		/* A stopped session drives nothing, whatever it held last. */
		if (dcp->v14 && READ_ONCE(dcp->v14->failed))
			return NULL;
		if (READ_ONCE(dcp->dptxport[0].connected) ||
		    READ_ONCE(dcp->external_link_ready) ||
		    (dcp->v14 && READ_ONCE(dcp->v14->powered)))
			return "a display link is up";
		return NULL;
	default:
		return NULL;
	}
}

bool iomfb_v14_7_external_failed(struct apple_dcp *dcp)
{
	return smp_load_acquire(&dcp->external_phase) == DCPEXT_FAILED ||
	       (dcp->v14 && READ_ONCE(dcp->v14->failed));
}

/* Refuse startup unless both the hardware floor and PMP vote are in place. */
static int dcpext_check_power(struct apple_dcp *dcp)
{
	struct device *dev = dcp->dev;
	struct device_node *ps, *entry;
	struct regmap *map;
	const char *label;
	u32 offset, value;
	int ret;

	ps = of_parse_phandle(dev->of_node, "power-domains", 0);
	if (!ps)
		return -ENODEV;
	/* The CPU domain of this pipe: dispext<N>_cpu. */
	ret = of_property_read_string(ps, "label", &label);
	if (ret || !strstarts(label, "dispext") || strlen(label) < 4 ||
	    strcmp(label + strlen(label) - 4, "_cpu")) {
		of_node_put(ps);
		return -EINVAL;
	}
	ret = of_property_read_u32(ps, "reg", &offset);
	map = syscon_node_to_regmap(ps->parent);
	of_node_put(ps);
	if (ret)
		return ret;
	if (IS_ERR(map))
		return PTR_ERR(map);
	ret = regmap_read(map, offset, &value);
	if (ret)
		return ret;
	if (((value >> 16) & 0xf) != 0xf || ((value >> 4) & 0xf) != 0xf)
		return dev_err_probe(dev, -EIO,
			"dcpext power floor is not active: %#x\n", value);

	entry = of_parse_phandle(dev->of_node, "apple,pmp-report", 0);
	if (!entry)
		return -ENODEV;
	ret = apple_pmp_report_wait_ready(entry, DCP_V14_PMP_TIMEOUT);
	of_node_put(entry);
	return ret;
}

/*
 * Starts the processor's CPU and RTKit, then its endpoints. The native path
 * also brings up DCPLink, for the display interface opened at the first
 * attach, and the DP AV service the EDID comes from.
 */
static int dcpext_boot(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14;
	struct device *dev = dcp->dev;
	struct resource *res;
	struct apple_rtkit *rtk;
	u32 control;
	int ret, n;

	ret = dcpext_check_power(dcp);
	if (ret) {
		dev_err(dev, "dcpext startup refused: power prerequisites failed: %d\n", ret);
		return ret;
	}
	dev_info(dev, "dcpext CPU power floor active and PMP request acknowledged\n");

	if (!dcp->coproc_reg) {
		res = platform_get_resource_byname(to_platform_device(dev),
						   IORESOURCE_MEM, "coproc");
		if (!res) {
			dev_err(dev, "dcpext has no coproc register\n");
			return -ENODEV;
		}
		dcp->coproc_reg = devm_ioremap_resource(dev, res);
		if (IS_ERR(dcp->coproc_reg)) {
			ret = PTR_ERR(dcp->coproc_reg);
			dev_err(dev, "dcpext coproc map failed: %d\n", ret);
			dcp->coproc_reg = NULL;
			return ret;
		}
	}

	control = readl(dcp->coproc_reg + DCP_V14_CPU_CONTROL);
	dev_info(dev, "dcpext CPU control %#x\n", control);
	ret = dcpext_verify_memory(dcp);
	if (ret)
		return ret;

	v14 = kzalloc_obj(*v14);
	if (!v14)
		return -ENOMEM;
	v14->dev = dev;
	v14->dcp = dcp;
	v14->external = dcp->external_native;
	if (v14->external) {
		struct clk *clk = clk_get_optional(dev, NULL);

		if (!IS_ERR_OR_NULL(clk)) {
			v14->clock_rate = clk_get_rate(clk);
			clk_put(clk);
		}
		if (!v14->clock_rate)
			v14->clock_rate = DCP_V14_EXT_CLOCK;
		v14->stride = DCP_V14_EXT_STRIDE;
	}
	mutex_init(&v14->lock);
	init_waitqueue_head(&v14->described_wait);
	INIT_WORK(&v14->idle_work, dcp_v14_idle);
	INIT_WORK(&v14->stopped_work, dcp_v14_stopped_work);
	dcp->v14 = v14;
	dcp_v14_link_init(&v14->link, dev, NULL);

	rtk = apple_rtkit_init(dev, v14, "mbox", 0, &dcp_v14_rtkit_ops);
	if (IS_ERR(rtk)) {
		dev_err(dev, "dcpext RTKit init failed: %ld\n", PTR_ERR(rtk));
		return PTR_ERR(rtk);
	}
	v14->rtk = rtk;
	v14->link.rtk = rtk;
	dcp->rtk = rtk;

	if (!(control & APPLE_DCP_COPROC_CPU_CONTROL_RUN)) {
		writel(control | APPLE_DCP_COPROC_CPU_CONTROL_RUN,
		       dcp->coproc_reg + DCP_V14_CPU_CONTROL);
		dev_info(dev, "dcpext CPU started\n");
	}
	ret = apple_rtkit_wake(rtk);
	for (n = 0; ret == -ETIME && n < DCP_V14_RTKIT_RETRIES; n++)
		ret = apple_rtkit_boot(rtk);
	if (ret) {
		dev_err(dev, "dcpext RTKit did not wake: %d\n", ret);
		return ret;
	}
	dev_info(dev, "dcpext RTKit session running\n");

	if (v14->external) {
		ret = dcp_v14_link_start(&v14->link);
		if (ret) {
			v14->failed = true;
			dev_err(dev, "dcpext DCPLink did not start: %d\n", ret);
			return ret;
		}
		/* Notifications that arrive with no call in progress. */
		dcp_v14_link_set_idle_work(&v14->link, &v14->idle_work);
	}
	if (apple_rtkit_has_endpoint(rtk, DPAV_CTRL_ENDPOINT))
		dpav_ctrl_init(dcp);
	if (apple_rtkit_has_endpoint(rtk, DPTX_ENDPOINT)) {
		ret = dptxep_init(dcp);
		if (ret)
			dev_err(dev, "dcpext DPTX endpoint failed: %d\n", ret);
	}
	if (v14->external) {
		/* The service the attached display's EDID is read from. */
		if (apple_rtkit_has_endpoint(rtk, DPAVSERV_ENDPOINT)) {
			ret = dpavservep_init(dcp);
			if (ret)
				dev_info(dev, "display EDID service not available: %d\n", ret);
		}
		return 0;
	}
	/* External mode discovery only; this does not power or modeset a display. */
	if (apple_rtkit_has_endpoint(rtk, DISP0_ENDPOINT)) {
		ret = ibootep_init(dcp);
		if (ret)
			dev_err(dev, "dcpext mode-query endpoint failed: %d\n", ret);
	}
	return 0;
}

/* Native: system sleep began after the start was requested; undo the request. */
static bool dcpext_start_deferred(struct apple_dcp *dcp)
{
	guard(mutex)(&dcp->hpd_mutex);
	if (!READ_ONCE(dcp->external_suspended))
		return false;
	WRITE_ONCE(dcp->external_phase, DCPEXT_IDLE);
	atomic_set(&dcp->external_requested, 0);
	dev_info(dcp->dev, "system sleep began: the external display processor starts on the next attach\n");
	module_put(THIS_MODULE);
	return true;
}

static void dcpext_bringup(struct work_struct *work)
{
	struct apple_dcp *dcp = container_of(work, struct apple_dcp, external_work);
	int ret;

	if (dcp->external_native && dcpext_start_deferred(dcp))
		return;
	ret = dcpext_boot(dcp);
	if (!dcp->external_native)
		return;
	/* Pairs with external_ready(): the endpoints are published first. */
	smp_store_release(&dcp->external_phase, ret ? DCPEXT_FAILED : DCPEXT_RUNNING);
	if (ret) {
		dev_err(dcp->dev, "external display processor did not start: %d; no retry until reboot\n",
			ret);
		return;
	}
	dev_info(dcp->dev, "external display processor running\n");
	dcp_external_ready(dcp);
}

/*
 * Opens the display interface (start signal, first client open) on a
 * running external processor, once, after its first display is attached.
 * The firmware then describes that display and every later one.
 */
int iomfb_v14_7_external_open(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14 = dcp->v14;
	int ret;

	if (smp_load_acquire(&dcp->external_phase) != DCPEXT_RUNNING || !v14)
		return -ENODEV;
	mutex_lock(&v14->lock);
	if (v14->opened) {
		mutex_unlock(&v14->lock);
		return 0;
	}
	ret = dcp_v14_simple_call(v14, A(401), false, true, true);
	if (!ret)
		ret = dcp_v14_simple_call(v14, A(455), false, false, false);
	if (!ret) {
		v14->opened = true;
	} else {
		v14->failed = true;
		dcp_v14_external_stopped(v14);
	}
	mutex_unlock(&v14->lock);
	if (ret) {
		dev_err(dcp->dev, "external display interface did not open: %d; no retry until reboot\n",
			ret);
		return ret;
	}
	dev_info(dcp->dev, "external display interface open\n");
	/* A display described before this could not be enabled until now. */
	if (!READ_ONCE(dcp->external_detached) && dcp->connector &&
	    READ_ONCE(dcp->connector->connected))
		dcp_queue_hotplug(dcp->connector);
	return 0;
}

int iomfb_v14_7_bind(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14 = dcp->v14;
	struct device *dev = dcp->dev;
	struct apple_rtkit *rtk;
	u32 control, status;
	struct clk *clk;
	int ret, n;

	/* Only an internal panel binds here, and it always has a board. */
	if (!v14 || WARN_ON_ONCE(!v14->board))
		return -ENODEV;
	if (v14->rtk || dcp_v14_panel_session)
		return dev_err_probe(dev, -EBUSY,
				     "%s display not started: an earlier DCP session is kept; reboot to restart the display\n",
				     v14->board->name);

	control = readl(dcp->coproc_reg + DCP_V14_CPU_CONTROL);
	status = readl(dcp->coproc_reg + DCP_V14_CPU_STATUS);
	dev_info(dev, "DCP CPU control %#x status %#x\n", control, status);
	INIT_DELAYED_WORK(&v14->backlight_work, dcp_v14_backlight_work);
	if (!(control & APPLE_DCP_COPROC_CPU_CONTROL_RUN))
		return dev_err_probe(dev, -EBUSY,
				     "%s display not started: the DCP CPU is stopped, and interrupted firmware is never resumed\n",
				     v14->board->name);

	ret = dcp_v14_geometry(v14);
	if (ret)
		return dev_err_probe(dev, ret,
				     "%s display not started: no usable boot framebuffer\n",
				     v14->board->name);
	clk = clk_get(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "%s display not started: no display clock\n",
				     v14->board->name);
	v14->clock_rate = clk_get_rate(clk);
	clk_put(clk);
	if (!v14->clock_rate)
		return dev_err_probe(dev, -EINVAL,
				     "%s display not started: the display clock has no rate\n",
				     v14->board->name);

	dcp_v14_link_init(&v14->link, dev, NULL);
	rtk = apple_rtkit_init(dev, v14, "mbox", 0, &dcp_v14_rtkit_ops);
	if (IS_ERR(rtk))
		return dev_err_probe(dev, PTR_ERR(rtk),
				     "%s display not started: RTKit init failed\n",
				     v14->board->name);
	/* From here nothing is freed and the module stays loaded. */
	v14->rtk = rtk;
	v14->link.rtk = rtk;
	dcp->rtk = rtk;
	dcp_v14_panel_session = true;
	__module_get(THIS_MODULE);

	ret = apple_rtkit_wake(rtk);
	for (n = 0; ret == -ETIME && n < DCP_V14_RTKIT_RETRIES; n++)
		ret = apple_rtkit_boot(rtk);
	if (ret) {
		v14->failed = true;
		return dev_err_probe(dev, ret,
				     "%s display not started: the DCP RTKit session did not wake; the boot framebuffer stays\n",
				     v14->board->name);
	}
	dev_info(dev, "DCP RTKit session running\n");
	/* The internal panel uses IOMFB only; dock DPTX belongs to dcpext. */
	return 0;
}

void iomfb_v14_7_unbind(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14 = dcp->v14;

	dcp->active = false;
	dcp_mode_set_valid(&dcp->mode_state, false);
	if (dcp->external_native && !v14)
		WRITE_ONCE(dcp->external_detached, true);
	if (!v14)
		return;
	/*
	 * An external session keeps serving its ports; removal detaches it.
	 * Its callbacks stop using the connector and CRTC DRM frees next:
	 * the flag is set under the lock they run under, and the crash
	 * callback and pending work are drained.
	 */
	if (v14->external) {
		mutex_lock(&v14->lock);
		WRITE_ONCE(dcp->external_detached, true);
		mutex_unlock(&v14->lock);
		if (v14->rtk)
			apple_rtkit_flush_rx(v14->rtk);
		flush_work(&v14->idle_work);
		cancel_work_sync(&v14->stopped_work);
	} else {
		/* No brightness-only present once the apple_dcp is gone. */
		cancel_delayed_work_sync(&v14->backlight_work);
		WRITE_ONCE(v14->dcp, NULL);
	}
	if (v14->rtk)
		dev_info(dcp->dev, "display unbound; the DCP session and its buffers are kept until reboot\n");
}

/* Waits for the messages dcp_v14_recv() has already queued for its endpoints. */
static void dcp_v14_flush_endpoints(struct apple_dcp *dcp)
{
	struct apple_dcp_afkep *eps[] = {
		dcp->ibootep, dcp->dptxep, dcp->dpavctrlep, dcp->avep, dcp->dcpavservep,
	};
	int i;

	/* A failed afk_init() can leave an error pointer behind. */
	for (i = 0; i < ARRAY_SIZE(eps); i++)
		if (!IS_ERR_OR_NULL(eps[i]))
			flush_workqueue(eps[i]->wq);
}

void iomfb_v14_7_remove(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14;

	/* Bring-up creates the external session; a late start must not. */
	if (dcp->external)
		disable_work_sync(&dcp->external_work);
	if (dcp->external_native) {
		disable_work_sync(&dcp->external_ready_work);
		disable_delayed_work_sync(&dcp->external_retry_wq);
	}
	v14 = dcp->v14;
	if (!v14)
		return;
	/*
	 * devres frees the apple_dcp next, but RTKit keeps calling into v14.
	 * Callbacks that already loaded the old pointer finish before the
	 * flush returns; later ones see NULL.
	 */
	WRITE_ONCE(v14->dcp, NULL);
	if (v14->rtk)
		apple_rtkit_flush_rx(v14->rtk);
	/* Their endpoints are freed with the apple_dcp too; nothing queues more now. */
	dcp_v14_flush_endpoints(dcp);
	cancel_work_sync(&v14->idle_work);
	if (v14->external)
		cancel_work_sync(&v14->stopped_work);
	else
		cancel_delayed_work_sync(&v14->backlight_work);
}

static int dcp_v14_status_show(struct seq_file *m, void *unused)
{
	struct apple_dcp_v14 *v14 = m->private;
	struct drm_framebuffer *fb;
	unsigned int i;
	int ret;

	ret = mutex_lock_interruptible(&v14->lock);
	if (ret)
		return ret;
	fb = v14->active_fb;
	seq_printf(m, "started %d\nfailed %d\nswaps %llu\nswap_max_us %llu\n",
		   v14->started, v14->failed, v14->swaps, v14->swap_ns_max / NSEC_PER_USEC);
	seq_printf(m, "active_fb %u\nimported %d\n", fb ? fb->base.id : 0,
		   fb && fb->obj[0]->import_attach);
	seq_printf(m, "panel %ux%u\nboot_fb %ux%u stride %u\nclock %llu\n",
		   v14->panel_width, v14->panel_height, v14->fb_width, v14->fb_height,
		   v14->stride, v14->clock_rate);
	seq_printf(m, "backlight factor %#x updates %llu\n",
		   v14->backlight_factor, v14->backlight_factor_updates);
	seq_printf(m, "pr nits %u.%04u valid %d correction %#x\n", v14->pr_nits >> 16,
		   (u32)(((u64)(v14->pr_nits & 0xffff) * 10000) >> 16), v14->pr_nits_valid,
		   v14->pr_correction);
	if (v14->external)
		seq_printf(m, "swaps dropped %llu status waits %llu timeouts %llu last %#x\n",
			   v14->swaps_dropped, v14->status_waits, v14->status_timeouts,
			   v14->display_status);
	seq_printf(m, "ctm valid %d disabled %d calls %llu setter %#x getter %#x\n",
		   v14->ctm_valid, v14->ctm_disabled, v14->ctm_calls, v14->ctm_status,
		   v14->ctm_get_status);
	for (i = 0; i < 9; i++)
		seq_printf(m, "ctm_readback[%u] %#llx\n", i, v14->ctm_readback[i]);
	seq_printf(m, "buffers %u bytes %llu\nproperties %u raw %u\nanalytics %llu\n",
		   v14->buffer_count, v14->buffer_bytes, v14->property_count, v14->raw_count,
		   v14->analytics);
	if (!v14->external)
		seq_printf(m, "nits sent %u presents %llu brightness-only %llu refused %llu last frame %d\n",
			   v14->nits_sent, v14->nits_presents, v14->brightness_presents,
			   v14->nits_refused, v14->last.valid);
	mutex_unlock(&v14->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dcp_v14_status);

/* One mode: the native timing of the boot panel, less the hidden notch rows. */
static int dcp_v14_mode(struct apple_dcp *dcp, struct apple_dcp_v14 *v14)
{
	struct dcp_display_mode *modes, *best = NULL, *selected;
	struct dcp_parse_ctx ctx;
	unsigned int count, i;
	void *blob = NULL;
	u32 size = 0;
	int ret;

	for (i = 0; i < v14->raw_count; i++) {
		if (!v14->raw[i].service && !strcmp(v14->raw[i].key, "PreferredTimingElements")) {
			size = v14->raw[i].size;
			blob = v14->raw[i].data;
			break;
		}
	}
	if (!blob)
		return -ENODATA;
	ret = parse(blob, size, &ctx);
	if (ret)
		return ret;
	ctx.dcp = dcp;
	modes = enumerate_modes(&ctx, &count, dcp->panel.width_mm, dcp->panel.height_mm,
				dcp->notch_height, true, false);
	if (IS_ERR(modes))
		return PTR_ERR(modes);
	for (i = 0; i < count; i++) {
		struct drm_display_mode *m = &modes[i].mode;
		int hz = drm_mode_vrefresh(m);

		if (v14->board->log_bringup)
			dev_info(dcp->dev, "timing %u/%u: " DRM_MODE_FMT "%s\n", i + 1, count,
				 DRM_MODE_ARG(m), m->type & DRM_MODE_TYPE_PREFERRED ? " best" : "");
		else
			dev_dbg(dcp->dev, "timing %u/%u: " DRM_MODE_FMT "%s\n", i + 1, count,
				DRM_MODE_ARG(m), m->type & DRM_MODE_TYPE_PREFERRED ? " best" : "");
		if (m->hdisplay != v14->panel_width ||
		    m->vdisplay != v14->panel_height - dcp->notch_height ||
		    (hz != 60 && (hz != 120 || !v14->board->promotion)))
			continue;
		if (!best || (m->type & DRM_MODE_TYPE_PREFERRED))
			best = &modes[i];
	}
	if (!best) {
		dev_err(dcp->dev, "no %ux%u timing at 60%s Hz among %u\n", v14->panel_width,
			v14->panel_height - dcp->notch_height,
			v14->board->promotion ? " or 120" : "", count);
		kfree(modes);
		return -EINVAL;
	}
	best->mode.type |= DRM_MODE_TYPE_PREFERRED;
	selected = kmemdup(best, sizeof(*best), GFP_KERNEL);
	kfree(modes);
	if (!selected)
		return -ENOMEM;
	mutex_lock(&dcp->modes_lock);
	kfree(dcp->modes);
	dcp->modes = selected;
	dcp->nr_modes = 1;
	dcp->modes_admitted = true;
	mutex_unlock(&dcp->modes_lock);
	return 0;
}

int iomfb_v14_7_start(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14 = dcp->v14;
	struct drm_display_mode selected;
	const struct drm_display_mode *mode = &selected;
	const char *step;
	int ret;

	if (!v14 || !v14->board || !v14->rtk || v14->failed || v14->started)
		return -ENODEV;

	step = "DCPLink";
	ret = dcp_v14_link_start(&v14->link);
	if (ret)
		goto fail;

	mutex_lock(&v14->lock);
	step = "start signal";
	ret = dcp_v14_simple_call(v14, A(401), false, true, true);
	if (!ret) {
		step = "first client open";
		ret = dcp_v14_simple_call(v14, A(455), false, false, false);
	}
	mutex_unlock(&v14->lock);
	if (ret)
		goto fail;
	dcp_v14_link_set_idle_work(&v14->link, &v14->idle_work);

	step = "panel mode";
	mutex_lock(&v14->lock);
	ret = dcp_v14_mode(dcp, v14);
	mutex_unlock(&v14->lock);
	if (ret)
		goto fail;

	v14->started = true;
	/* Never removed, like the session it describes. */
	debugfs_create_file(v14->board->debugfs, 0400, NULL, v14, &dcp_v14_status_fops);
	dcp->connector->connected = true;
	dcp_set_dimensions(dcp, dcp_modes_transfer_begin(dcp));
	dcp_mode_set_valid(&dcp->mode_state, true);
	dcp->active = true;
	complete(&dcp->start_done);
	/* The level policy; the first present carries the default level. */
	dcp_v14_configure_backlight(dcp);

	mutex_lock(&dcp->modes_lock);
	selected = dcp->modes[0].mode;
	mutex_unlock(&dcp->modes_lock);
	dev_info(dcp->dev, "%s display started: %ux%u@%d, %u notch rows %s, %ux%u mm\n",
		 v14->board->name, mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode),
		 v14->notch_rows,
		 dcp->notch_height ? "hidden" : "shown", mode->width_mm, mode->height_mm);
	return 0;
fail:
	v14->failed = true;
	dev_err(dcp->dev,
		"%s display not started: %s failed: %d; the boot framebuffer stays (reboot to retry)\n",
		v14->board->name, step, ret);
	return ret;
}

/*
 * Sends the CRTC's colour matrix, then reads it back, under the RPC lock and
 * before the swap that should show it. Only on a board whose firmware image
 * has tested matrix methods; elsewhere the matrix is not sent.
 *
 * Request: location (9) at 0, nine unaligned 64-bit coefficients at 4, a
 * null flag at 76, 3 bytes of padding (80 bytes in, 4 out). Readback: 80 in,
 * 76 out, coefficients at 0 and the status at 72. The firmware takes signed
 * two's-complement Q32 coefficients; DRM gives sign-magnitude S31.32, so
 * negative ones are converted. No matrix means identity.
 */
static int dcp_v14_ctm_locked(struct apple_dcp_v14 *v14,
			      const struct drm_crtc_state *state)
{
	u8 request[80] = {}, readback[76] = {};
	const struct drm_color_ctm *ctm;
	__le32 status = 0;
	int i, ret;

	if (!v14->board || !v14->board->ctm_set || !state || v14->ctm_disabled ||
	    (v14->ctm_valid && !state->color_mgmt_changed &&
	     !state->mode_changed && !state->active_changed))
		return 0;
	ctm = state->ctm ? state->ctm->data : NULL;
	put_unaligned_le32(9, request);
	for (i = 0; i < 9; i++) {
		u64 coefficient = ctm ? ctm->matrix[i] :
				  ((i == 0 || i == 4 || i == 8) ? 1ULL << 32 : 0);
		u64 magnitude = coefficient & ~(1ULL << 63);
		u64 signed_q32 = coefficient & (1ULL << 63) ? -magnitude : magnitude;

		put_unaligned_le64(signed_q32, request + 4 + 8 * i);
	}
	ret = dcp_v14_call(v14, v14->board->ctm_set, request, sizeof(request),
			   &status, sizeof(status), 0);
	v14->ctm_calls++;
	v14->ctm_status = le32_to_cpu(status);
	v14->ctm_valid = false;
	if (ret || v14->ctm_status) {
		dev_err(v14->dev, "CTM setter transport=%d status=%#x; colour matrix off until reboot\n",
			ret, v14->ctm_status);
		v14->ctm_disabled = true;
		return ret ? ret : -EIO;
	}
	/* Read back before the swap, under the same lock. */
	memset(request + 4, 0, 76);
	ret = dcp_v14_call(v14, v14->board->ctm_get, request, sizeof(request),
			   readback, sizeof(readback), 0);
	v14->ctm_get_status = get_unaligned_le32(readback + 72);
	for (i = 0; i < 9; i++)
		v14->ctm_readback[i] = get_unaligned_le64(readback + i * 8);
	dev_info_ratelimited(v14->dev, "CTM call %llu setter=%#x getter_transport=%d getter=%#x diagonal=%#llx,%#llx,%#llx\n",
		 v14->ctm_calls, v14->ctm_status, ret, v14->ctm_get_status,
		 v14->ctm_readback[0], v14->ctm_readback[4], v14->ctm_readback[8]);
	if (ret || v14->ctm_get_status) {
		dev_err(v14->dev, "CTM getter failed; colour matrix off until reboot\n");
		v14->ctm_disabled = true;
		return ret ? ret : -EIO;
	}
	v14->ctm_valid = true;
	return 0;
}

static bool dcp_v14_external_described(struct apple_dcp_v14 *v14);

struct dcp_v14_swap_admission {
	struct apple_dcp_v14 *v14;
	u64 generation;
};

static bool dcp_v14_swap_admit(void *cookie)
{
	struct dcp_v14_swap_admission *admission = cookie;
	struct apple_dcp_v14 *v14 = admission->v14;
	struct apple_dcp *dcp = READ_ONCE(v14->dcp);
	struct apple_connector *connector;

	if (!v14->external)
		return true;
	if (!dcp || READ_ONCE(v14->failed) || !READ_ONCE(v14->powered) ||
	    !READ_ONCE(v14->mode_live) || dcp_v14_sink_inactive(dcp) ||
	    admission->generation != v14->description_generation ||
	    !dcp_v14_external_described(v14) || !READ_ONCE(dcp->mode_state.valid))
		return false;
	connector = READ_ONCE(dcp->connector);
	return connector && READ_ONCE(connector->connected);
}

/*
 * Swap start, then the swap; returns once the firmware completed that swap.
 * @ctm_state, if set, carries the colour matrix to send first.
 */
/*
 * The panel level the backlight policy has to present, encoded into @swap
 * (the 14.7 panel only): a reservation, @present, that the swap's result
 * completes (dcp_v14_swap_nits_done()). An unchanged level is not sent
 * again. Returns whether a level was reserved.
 */
static bool dcp_v14_swap_nits(struct apple_dcp_v14 *v14, u8 *swap,
			      struct dcp_backlight_present *present)
{
	struct apple_dcp *dcp = READ_ONCE(v14->dcp);

	dcp_v14_encode_nits(swap, false, 0);
	if (v14->external || !dcp || !dcp_backlight_active(dcp))
		return false;
	/* -EALREADY: no change; -ENODATA/-EBUSY: nothing to present now. */
	if (dcp_backlight_prepare(dcp, true, present))
		return false;
	/* The policy's maximum is at most the wire bound; never above it. */
	if (WARN_ON_ONCE(dcp_v14_encode_nits(swap, true, present->nits))) {
		dcp_backlight_complete(dcp, present->sequence, false);
		v14->nits_refused++;
		return false;
	}
	return true;
}

/* The reserved level went out with the swap: accepted, or retried later. */
static void dcp_v14_swap_nits_done(struct apple_dcp_v14 *v14,
				   const struct dcp_backlight_present *present, bool accepted)
{
	struct apple_dcp *dcp = READ_ONCE(v14->dcp);
	unsigned int delay;

	if (!dcp)
		return;
	dcp_backlight_complete(dcp, present->sequence, accepted);
	if (accepted) {
		v14->nits_sent = present->nits;
		v14->nits_presents++;
		return;
	}
	v14->nits_refused++;
	delay = dcp_backlight_retry_delay(dcp);
	if (delay)
		queue_delayed_work(system_wq, &v14->backlight_work, msecs_to_jiffies(delay));
}

/*
 * Swap start, then the swap, under the RPC lock, in the caller's @swap
 * buffer; returns once the firmware completed that swap. @ctm_state, if set,
 * carries the colour matrix to send first. On the panel the swap also
 * carries the level the backlight policy has to present, and a frame shown
 * is kept for brightness-only presents.
 */
static int dcp_v14_swap_locked(struct apple_dcp_v14 *v14, u8 *swap, const u8 *surface,
			       u64 iova, u32 width, u32 height, u32 dst_y,
			       const struct drm_crtc_state *ctm_state)
{
	struct dcp_v14_swap_admission admission = {
		.v14 = v14,
		.generation = v14->description_generation,
	};
	struct dcp_backlight_present present;
	__le32 start[4] = {}, started[2] = {}, result[3];
	bool nits = false, dropped = false;
	u32 id;
	int ret;

	lockdep_assert_held(&v14->lock);
	/*
	 * A colour matrix the firmware refuses costs night light, not the
	 * frame: present anyway, and stop sending it (ctm_disabled).
	 */
	dcp_v14_ctm_locked(v14, ctm_state);
	ret = dcp_v14_call_guarded(v14, A(406), start, sizeof(start), started, sizeof(started),
				  0, dcp_v14_swap_admit, &admission);
	if (!ret && le32_to_cpu(started[1]))
		ret = -EIO;
	id = le32_to_cpu(started[0]);
	if (!ret && !id)
		ret = -EPROTO;
	if (!ret) {
		dcp_v14_encode_swap(swap, id, DCP_V14_BLACK, surface, iova, width, height,
				    dst_y, DCP_V14_LAYER);
		nits = dcp_v14_swap_nits(v14, swap, &present);
		ret = dcp_v14_call_guarded(v14, A(407), swap, DCP_V14_SWAP_SIZE, result, sizeof(result),
				   id, dcp_v14_swap_admit, &admission);
		/* The swap status sits at byte 5 of the reply. */
		if (!ret && get_unaligned_le32((u8 *)result + 5))
			ret = -EIO;
		/*
		 * Status 0 with the flag at byte 0: the firmware took the swap
		 * and dropped it, as it does with one that arrives while its
		 * timings are off after a power-on. No D589 follows (the link
		 * knows); nothing was shown.
		 */
		if (!ret && ((u8 *)result)[0]) {
			dropped = true;
			v14->swaps_dropped++;
			ret = -EAGAIN;
		}
		if (nits)
			dcp_v14_swap_nits_done(v14, &present, !ret);
	}
	/*
	 * An external display can go away under a swap: the firmware then
	 * refuses it, which ends nothing. Link failures stop the session in
	 * dcp_v14_call() either way.
	 */
	if (ret && !v14->external && !dropped)
		v14->failed = true;
	if (!ret && !v14->external) {
		v14->last.valid = !!surface;
		if (surface && surface != v14->last.surface) {
			memcpy(v14->last.surface, surface, DCP_V14_SURFACE_SIZE);
			v14->last.iova = iova;
			v14->last.width = width;
			v14->last.height = height;
			v14->last.dst_y = dst_y;
		}
	}
	return ret;
}

static int dcp_v14_swap(struct apple_dcp_v14 *v14, const u8 *surface, u64 iova,
			u32 width, u32 height, u32 dst_y,
			const struct drm_crtc_state *ctm_state)
{
	u8 *swap;
	int ret;

	if (surface && (!iova || !width || width > v14->panel_width || !height ||
			dst_y > v14->panel_height || height > v14->panel_height - dst_y))
		return -EINVAL;
	swap = kmalloc(DCP_V14_SWAP_SIZE, GFP_KERNEL);
	if (!swap)
		return -ENOMEM;

	mutex_lock(&v14->lock);
	ret = dcp_v14_swap_locked(v14, swap, surface, iova, width, height, dst_y, ctm_state);
	mutex_unlock(&v14->lock);

	kfree(swap);
	return ret;
}

/*
 * A brightness-only present (backlight_work): the last frame again, with
 * the level the policy has to present. Blanked, or nothing presented yet:
 * the next atomic present carries the level instead.
 */
static void dcp_v14_backlight_work(struct work_struct *work)
{
	struct apple_dcp_v14 *v14 = container_of(to_delayed_work(work), struct apple_dcp_v14,
						 backlight_work);
	struct apple_dcp *dcp = READ_ONCE(v14->dcp);
	u8 *swap;
	int ret;

	if (!dcp || v14->external || READ_ONCE(v14->failed) || !v14->started ||
	    READ_ONCE(dcp->crashed) || !dcp_backlight_pending(dcp))
		return;
	swap = kmalloc(DCP_V14_SWAP_SIZE, GFP_KERNEL);
	if (!swap)
		return;
	mutex_lock(&v14->lock);
	if (v14->last.valid && !v14->failed && READ_ONCE(dcp->mode_state.valid)) {
		ret = dcp_v14_swap_locked(v14, swap, v14->last.surface, v14->last.iova,
					  v14->last.width, v14->last.height, v14->last.dst_y,
					  NULL);
		if (!ret)
			v14->brightness_presents++;
		else
			dev_err(v14->dev, "brightness present failed: %d\n", ret);
	}
	mutex_unlock(&v14->lock);
	kfree(swap);
}

/* kick only queues work; the RPC runs from backlight_work. */
static void dcp_v14_backlight_kick(struct apple_dcp *dcp)
{
	if (dcp->v14)
		mod_delayed_work(system_wq, &dcp->v14->backlight_work, 0);
}

/*
 * The panel's backlight policy, set up once the panel runs: at most 500
 * nits (less if the panel's DT record says so), 140 nits presented by the
 * first present, no fades; dcp_backlight_configure() registers the
 * backlight device from bl_register_wq. No loader level is known here, so
 * the first present carries the default (dcp_bl_jump() in configure).
 */
static void dcp_v14_configure_backlight(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14 = dcp->v14;
	u32 maximum = DCP_V14_NITS_MAX;
	int ret;

	if (dcp->external || !v14 || !v14->board || !v14->started ||
	    dcp_backlight_active(dcp) || !dcp_has_panel(dcp) || !dcp->crtc)
		return;
	/* A described panel maximum may only lower the wire bound. */
	if (dcp->brightness.maximum)
		maximum = min_t(u32, maximum, dcp->brightness.maximum);
	ret = dcp_backlight_configure(dcp, maximum, false, 0, true,
				      min_t(u32, maximum, DCP_V14_NITS_DEFAULT),
				      dcp_v14_backlight_kick);
	if (ret && ret != -EBUSY)
		dev_err(dcp->dev, "%s backlight unavailable: %d\n", v14->board->name, ret);
}

/*
 * System sleep of the panel. DRM has blanked it (the background at 0 nits)
 * before this runs, synchronously under the RPC lock, and no brightness-only
 * present may start once the mailbox sleeps. The firmware keeps publishing
 * to the AP while it scans out, and each message would wake a CPU out of
 * suspend-to-idle: once no call is in flight (calls and notifications run
 * under the lock), hold its messages in the mailbox until resume.
 */
void iomfb_v14_7_suspend(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14 = dcp->v14;

	if (!v14)
		return;
	cancel_delayed_work_sync(&v14->backlight_work);
	if (!v14->rtk || v14->external || !v14->started || READ_ONCE(v14->failed) ||
	    v14->rx_held)
		return;
	mutex_lock(&v14->lock);
	apple_rtkit_hold_rx(v14->rtk);
	v14->rx_held = true;
	mutex_unlock(&v14->lock);
}

void iomfb_v14_7_resume(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14 = dcp->v14;

	if (!v14 || !v14->rx_held)
		return;
	v14->rx_held = false;
	apple_rtkit_resume_rx(v14->rtk);
}

/*
 * Shows the top-left @width x @height of @fb (or only the black background),
 * after the colour matrix of @ctm_state if that is set,
 * and keeps @fb until the next swap.
 */
static bool dcp_v14_present(struct apple_dcp_v14 *v14, struct drm_framebuffer *fb,
			    u32 width, u32 height, u32 dst_y,
			    const struct drm_crtc_state *ctm_state)
{
	u8 surface[DCP_V14_SURFACE_SIZE];
	struct drm_framebuffer *old;
	u64 iova = 0, start, elapsed, swaps;
	int ret;

	if (READ_ONCE(v14->failed))
		return false;
	if (fb) {
		drm_framebuffer_get(fb);
		dcp_v14_encode_surface(surface, fb->pitches[0], fb->width, fb->height,
				       fb->format->format == DRM_FORMAT_XRGB8888);
		iova = drm_fb_dma_get_gem_obj(fb, 0)->dma_addr;
	}
	start = ktime_get_ns();
	ret = dcp_v14_swap(v14, fb ? surface : NULL, iova, fb ? width : 0,
			   fb ? height : 0, dst_y, ctm_state);
	elapsed = ktime_get_ns() - start;
	if (ret && (v14->external || ret == -EAGAIN) && !READ_ONCE(v14->failed)) {
		/* Refused or dropped before it was shown: the old framebuffer stays. */
		if (fb)
			drm_framebuffer_put(fb);
		WRITE_ONCE(v14->mode_swaps, 0);
		dev_warn_ratelimited(v14->dev, "display %s a swap: %d\n",
				     ret == -EAGAIN ? "dropped" : "refused", ret);
		return false;
	}
	if (ret) {
		/* Either framebuffer may still be scanned out: keep both. */
		dev_err(v14->dev, "native DCP flip failed %d; buffers pinned until reboot\n", ret);
		return false;
	}
	mutex_lock(&v14->lock);
	old = v14->active_fb;
	v14->active_fb = fb;
	swaps = ++v14->swaps;
	if (fb)
		v14->mode_swaps++;
	v14->swap_ns_max = max(v14->swap_ns_max, elapsed);
	mutex_unlock(&v14->lock);
	if (old)
		drm_framebuffer_put(old);
	if (swaps <= 3)
		dev_info(v14->dev, "swap %llu complete: surface %d, imported %d, %llu us\n",
			 swaps, !!fb, fb && fb->obj[0]->import_attach, elapsed / NSEC_PER_USEC);
	return true;
}

/* Releases helper waiters without reporting a completed flip. */
static void dcp_v14_cancel_event(struct apple_dcp *dcp)
{
	struct apple_crtc *crtc = dcp->crtc;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	spin_lock_irqsave(&crtc->base.dev->event_lock, flags);
	event = crtc->event;
	crtc->event = NULL;
	spin_unlock_irqrestore(&crtc->base.dev->event_lock, flags);
	if (!event)
		return;
	if (event->base.fence) {
		dma_fence_set_error(event->base.fence, -EIO);
		dma_fence_signal(event->base.fence);
	}
	if (event->base.completion) {
		complete_all(event->base.completion);
		if (event->base.completion_release)
			event->base.completion_release(event->base.completion);
		event->base.completion = NULL;
	}
	drm_event_cancel_free(crtc->base.dev, &event->base);
}

/* Only the layout the firmware was qualified with: one full-mode linear plane. */
int iomfb_v14_7_atomic_check(struct apple_dcp *dcp, struct drm_crtc *crtc,
			     struct drm_atomic_state *state)
{
	struct apple_dcp_v14 *v14 = dcp->v14;
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	struct drm_plane_state *p = drm_atomic_get_new_plane_state(state, crtc->primary);
	struct drm_gem_dma_object *obj;
	struct drm_framebuffer *fb;
	u32 width, height;

	if (dcp->external) {
		/* An external pipe can always be switched off, started or not. */
		if (!crtc_state || !crtc_state->active)
			return 0;
		/*
		 * A stopped native session reports its display gone. Until
		 * userspace switches the pipe off, let commits that include it
		 * through: nothing is shown on it, and other outputs must not
		 * fail with it.
		 */
		if (dcp->external_native &&
		    (dcp->crashed || (v14 && READ_ONCE(v14->failed))))
			return 0;
		if (dcp->crashed || !v14 || READ_ONCE(v14->failed) || !v14->opened)
			return -EIO;
	} else if (dcp->crashed || !v14 || READ_ONCE(v14->failed)) {
		return -EIO;
	}
	if (!crtc_state || !crtc_state->active || !p || p->crtc != crtc || !p->visible)
		return 0;

	fb = p->fb;
	width = crtc_state->mode.hdisplay;
	height = crtc_state->mode.vdisplay;
	if (fb->format->format != DRM_FORMAT_XRGB8888 &&
	    fb->format->format != DRM_FORMAT_ARGB8888)
		return -EINVAL;
	if (fb->modifier != DRM_FORMAT_MOD_LINEAR || fb->offsets[0] ||
	    fb->pitches[0] < width * 4 || (fb->pitches[0] & 63))
		return -EINVAL;
	/* An external pipe may show the top-left of a larger framebuffer. */
	if (p->src_x || p->src_y || p->crtc_x || p->crtc_y ||
	    p->src_w != width << 16 || p->src_h != height << 16 ||
	    p->crtc_w != width || p->crtc_h != height ||
	    (dcp->external ? fb->width < width || fb->height < height :
			     fb->width != width || fb->height != height))
		return -EINVAL;
	obj = drm_fb_dma_get_gem_obj(fb, 0);
	if (!obj || !obj->dma_addr || (u64)fb->pitches[0] * height > obj->base.size)
		return -EINVAL;
	return 0;
}

/* A411: the timing and color mode of the attached display, by their ids. */
static int dcp_v14_set_mode(struct apple_dcp_v14 *v14, u32 color, u32 timing)
{
	__le32 in[2] = { cpu_to_le32(color), cpu_to_le32(timing) }, status = 0;
	int ret;

	ret = dcp_v14_call(v14, A(411), in, sizeof(in), &status, sizeof(status), 0);
	return ret ?: (le32_to_cpu(status) ? -EIO : 0);
}

/*
 * A437 get_block, block 0x6e: the display status word the firmware checks
 * before it programs a swap. The request is a token, the block and sub-block
 * ids, eight arguments and their count, and the output length; the reply
 * carries the block at 0 and a status at +0x1000.
 */
#define DCP_V14_GET_BLOCK_IN		0x5c
#define DCP_V14_GET_BLOCK_OUT		0x1004
#define DCP_V14_GET_BLOCK_ID		8
#define DCP_V14_GET_BLOCK_OUT_LEN	0x54
#define DCP_V14_GET_BLOCK_STATUS	0x1000
#define DCP_V14_DISPLAY_STATUS_BLOCK	0x6e
/* Pipe, can program a swap, soft and controller power, link, timings enabled. */
#define DCP_V14_DISPLAY_READY		(BIT(0) | BIT(2) | BIT(3) | BIT(4) | BIT(6) | BIT(7))
#define DCP_V14_DISPLAY_RUN_MODE_PENDING BIT(1)
/* How long a display is given to enable its timings after A472 on. */
#define DCP_V14_DISPLAY_READY_MS	2000

static int dcp_v14_display_status(struct apple_dcp_v14 *v14, u32 *status)
{
	u8 request[DCP_V14_GET_BLOCK_IN] = {};
	u8 *out;
	int ret;

	put_unaligned_le32(DCP_V14_DISPLAY_STATUS_BLOCK, request + DCP_V14_GET_BLOCK_ID);
	put_unaligned_le64(4, request + DCP_V14_GET_BLOCK_OUT_LEN);
	out = kzalloc(DCP_V14_GET_BLOCK_OUT, GFP_KERNEL);
	if (!out)
		return -ENOMEM;
	ret = dcp_v14_call(v14, A(437), request, sizeof(request), out, DCP_V14_GET_BLOCK_OUT, 0);
	if (!ret && get_unaligned_le32(out + DCP_V14_GET_BLOCK_STATUS)) {
		dev_err(v14->dev, "display status query answered %#x\n",
			get_unaligned_le32(out + DCP_V14_GET_BLOCK_STATUS));
		ret = -EIO;
	}
	if (!ret)
		*status = get_unaligned_le32(out);
	kfree(out);
	return ret;
}

/*
 * After A472 on the firmware enables the display's timings asynchronously,
 * and drops a swap that arrives before that (dcp_v14_swap_locked() ends such
 * a swap, but shows nothing). Wait for the display status to report the pipe,
 * link and timings up, polling while the callbacks keep running; bounded, and
 * a display that went away ends the wait. Called with the lock held.
 */
static void dcp_v14_external_wait_ready(struct apple_dcp *dcp, struct apple_dcp_v14 *v14)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(DCP_V14_DISPLAY_READY_MS);
	u32 status = 0;
	int ret;

	lockdep_assert_held(&v14->lock);
	v14->status_waits++;
	for (;;) {
		ret = dcp_v14_display_status(v14, &status);
		if (ret)
			return;
		v14->display_status = status;
		if ((status & DCP_V14_DISPLAY_READY) == DCP_V14_DISPLAY_READY &&
		    !(status & DCP_V14_DISPLAY_RUN_MODE_PENDING))
			return;
		if (READ_ONCE(v14->failed) || !READ_ONCE(dcp->dptxport[0].connected) ||
		    time_after(jiffies, deadline))
			break;
		usleep_range(1000, 2000);
	}
	v14->status_timeouts++;
	dev_warn(dcp->dev, "display not ready after power-on: status %#x\n", status);
}

/* A472: the display's power state; its status sits at byte 4 of the reply. */
static int dcp_v14_set_power(struct apple_dcp_v14 *v14, bool on)
{
	u8 in[12] = {}, out[8] = {};
	int ret;

	put_unaligned_le64(on, in);
	ret = dcp_v14_call(v14, A(472), in, sizeof(in), out, sizeof(out), 0);
	if (!ret && get_unaligned_le32(out + 4))
		ret = -EIO;
	/* A refused power-down leaves nothing on: the display is gone already. */
	if (!ret || (!on && !READ_ONCE(v14->failed)))
		v14->powered = on;
	if (!on)
		v14->mode_live = false;
	return ret;
}

/* The display is fully described, and no description is in transfer. */
static bool dcp_v14_external_described(struct apple_dcp_v14 *v14)
{
	return READ_ONCE(v14->failed) ||
	       ((READ_ONCE(v14->described) & DCP_V14_DESC_ALL) == DCP_V14_DESC_ALL &&
		!READ_ONCE(v14->chunk));
}

/*
 * Waits, for a bounded time, until the firmware has published the whole
 * description of the attached display (timings, color modes, attributes)
 * and is not in the middle of publishing one. A hotplug bounce withdraws it
 * and publishes it again part by part; a mode set in between would use
 * timing ids that are gone, or run alongside the publication. Returns with
 * the RPC lock held, unless it fails.
 */
static int dcp_v14_external_settle(struct apple_dcp *dcp, struct apple_dcp_v14 *v14)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(DCP_V14_DESC_TIMEOUT_MS);
	bool waited = false;
	long left;

	mutex_lock(&v14->lock);
	while (!dcp_v14_external_described(v14)) {
		left = (long)(deadline - jiffies);
		if (left <= 0)
			break;
		if (!waited)
			dev_info(dcp->dev, "display description incomplete (%#x%s); waiting before the mode is set\n",
				 v14->described, v14->chunk ? ", in transfer" : "");
		waited = true;
		mutex_unlock(&v14->lock);
		wait_event_timeout(v14->described_wait, dcp_v14_external_described(v14), left);
		mutex_lock(&v14->lock);
	}
	if (v14->failed) {
		mutex_unlock(&v14->lock);
		return -EIO;
	}
	if (dcp_v14_external_described(v14)) {
		if (waited)
			dev_info(dcp->dev, "display description complete again\n");
		return 0;
	}
	/* The attributes only size the display: timings and colors suffice. */
	if ((v14->described & (DCP_V14_DESC_TIMING | DCP_V14_DESC_COLOR)) ==
	    (DCP_V14_DESC_TIMING | DCP_V14_DESC_COLOR) && !v14->chunk) {
		dev_warn(dcp->dev, "display description still incomplete (%#x) after %u ms; setting the mode anyway\n",
			 v14->described, DCP_V14_DESC_TIMEOUT_MS);
		return 0;
	}
	dev_warn(dcp->dev, "display description incomplete (%#x%s) after %u ms; mode not set\n",
		 v14->described, v14->chunk ? ", in transfer" : "", DCP_V14_DESC_TIMEOUT_MS);
	mutex_unlock(&v14->lock);
	return -EAGAIN;
}

/* Called with the lock held. */
static bool dcp_v14_external_mode_live(struct apple_dcp_v14 *v14,
				       const struct dcp_display_mode *mode)
{
	return !v14->failed && v14->powered && v14->mode_live &&
	       v14->mode_timing == mode->timing_mode_id &&
	       v14->mode_color == mode->color_mode_id && READ_ONCE(v14->mode_swaps);
}

/*
 * Whether the firmware runs @drm_mode, set last, from the description it
 * has now, and shows swaps in it.
 */
bool iomfb_v14_7_external_showing(struct apple_dcp *dcp,
				  const struct drm_display_mode *drm_mode)
{
	struct apple_dcp_v14 *v14 = dcp->v14;
	struct dcp_display_mode mode = {};
	bool live;

	if (!dcp->external_native || !v14 || !lookup_mode(dcp, drm_mode, &mode))
		return false;
	mutex_lock(&v14->lock);
	live = dcp_v14_external_described(v14) && dcp_v14_external_mode_live(v14, &mode);
	mutex_unlock(&v14->lock);
	return live;
}

bool iomfb_v14_7_external_powered(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14 = dcp->v14;

	return dcp->external && v14 && READ_ONCE(v14->powered) && !READ_ONCE(v14->failed);
}

/* An external display: power down, set the new mode, power up. */
static int dcp_v14_external_modeset(struct apple_dcp *dcp, struct drm_crtc_state *crtc_state)
{
	struct apple_dcp_v14 *v14 = dcp->v14;
	struct dcp_display_mode mode = {};
	int ret = 0;

	if (!v14 || READ_ONCE(v14->failed) || !v14->opened)
		return -EIO;
	/*
	 * The sink's audio stops before the display is powered down for the
	 * mode change (as dcp_poweroff() has it), outside the RPC lock; it is
	 * connected again once the display is on, below.
	 */
	if (READ_ONCE(v14->powered) && dcp->avep)
		av_service_disconnect(dcp);
	ret = dcp_v14_external_settle(dcp, v14);
	if (ret)
		goto out;
	/* Only after the wait: a new description replaces the catalog. */
	if (!lookup_mode(dcp, &crtc_state->mode, &mode)) {
		mutex_unlock(&v14->lock);
		ret = -EINVAL;
		goto out;
	}
	/*
	 * The firmware already runs this mode and shows swaps in it: setting
	 * it again only restarts its timings under the display. A retry or a
	 * replayed CRTC lands here when the display came back as it was.
	 */
	if (dcp_v14_external_mode_live(v14, &mode)) {
		mutex_unlock(&v14->lock);
		dev_info(dcp->dev, "display mode " DRM_MODE_FMT " (timing %u, color %u) already active and showing\n",
			 DRM_MODE_ARG(&crtc_state->mode), mode.timing_mode_id, mode.color_mode_id);
		atomic_set(&dcp->external_retries, 0);
		dcp_mode_set_valid(&dcp->mode_state, true);
		if (dcp->avep)
			av_service_connect(dcp);
		return 0;
	}
	if (v14->powered && dcp_v14_set_power(v14, false))
		dev_warn(dcp->dev, "display did not power down before the mode change\n");
	if (READ_ONCE(v14->failed))
		ret = -EIO;
	if (!ret)
		ret = dcp_v14_set_mode(v14, mode.color_mode_id, mode.timing_mode_id);
	if (!ret)
		ret = dcp_v14_set_power(v14, true);
	if (!ret) {
		v14->panel_width = mode.mode.hdisplay;
		v14->panel_height = mode.mode.vdisplay;
		/* D101: the default stride follows the mode from here on. */
		v14->stride = mode.mode.hdisplay * 4;
		v14->mode_timing = mode.timing_mode_id;
		v14->mode_color = mode.color_mode_id;
		v14->mode_swaps = 0;
		v14->mode_live = true;
		dcp_v14_external_wait_ready(dcp, v14);
	}
	mutex_unlock(&v14->lock);
out:
	dev_info(dcp->dev, "display mode " DRM_MODE_FMT " (timing %u, color %u): %d\n",
		 DRM_MODE_ARG(&crtc_state->mode), mode.timing_mode_id, mode.color_mode_id, ret);
	if (ret) {
		/* A mode that is gone, or a description that never settled. */
		dcp_external_retry(dcp, "display mode not set", ret, 500);
		return ret;
	}
	atomic_set(&dcp->external_retries, 0);
	dcp_mode_set_valid(&dcp->mode_state, true);
	/* A472 on: the sink's audio again. */
	if (dcp->avep)
		av_service_connect(dcp);
	return 0;
}

int iomfb_v14_7_modeset(struct apple_dcp *dcp, struct drm_crtc_state *crtc_state)
{
	if (dcp->external)
		return dcp_v14_external_modeset(dcp, crtc_state);
	/* The firmware keeps the timing it booted with, which is the only mode. */
	if (!lookup_mode(dcp, &crtc_state->mode, NULL))
		return -EINVAL;
	dcp_mode_set_valid(&dcp->mode_state, true);
	return 0;
}

void iomfb_v14_7_flush(struct apple_dcp *dcp, struct drm_crtc *crtc,
		       struct drm_atomic_state *state)
{
	struct drm_plane_state *p = drm_atomic_get_new_plane_state(state, crtc->primary);
	struct drm_crtc_state *cs = drm_atomic_get_new_crtc_state(state, crtc);
	struct drm_framebuffer *fb;

	if (!p)
		p = crtc->primary->state;
	if (!cs)
		cs = crtc->state;
	fb = p && p->visible ? p->fb : NULL;
	/*
	 * No swaps while an external display is off, unset, unplugged, or
	 * being described again.
	 */
	if (dcp->external && (!dcp->v14 || !READ_ONCE(dcp->v14->powered) ||
			      !dcp_v14_external_described(dcp->v14) ||
			      !READ_ONCE(dcp->mode_state.valid) || !dcp->connector ||
			      !READ_ONCE(dcp->connector->connected) ||
			      dcp_v14_sink_inactive(dcp))) {
		dcp_drm_crtc_vblank(dcp->crtc);
		return;
	}
	/* The panel's level policy, unless the start set it up already. */
	if (!dcp->external)
		dcp_v14_configure_backlight(dcp);
	dcp->swap_start = ktime_get();
	if (dcp->v14 && dcp_v14_present(dcp->v14, fb,
					 fb ? (dcp->external ? p->src_w >> 16 : fb->width) : 0,
					 fb ? (dcp->external ? p->src_h >> 16 : fb->height) : 0,
					 dcp->notch_height, cs))
		dcp_drm_crtc_page_flip(dcp, ktime_get());
	else
		dcp_v14_cancel_event(dcp);
}

void iomfb_v14_7_poweron(struct apple_dcp *dcp)
{
	/* The panel stays on; the next swap shows the plane again, at its level. */
	if (!dcp->external && dcp_backlight_active(dcp))
		dcp_backlight_dpms(dcp, true);
}

void iomfb_v14_7_poweroff(struct apple_dcp *dcp)
{
	struct apple_dcp_v14 *v14 = dcp->v14;

	/* An external display is powered down; the next enable sets its mode again. */
	if (dcp->external) {
		/* Audio stops with the display, as dcp_poweroff() has it. */
		if (dcp->avep)
			av_service_disconnect(dcp);
		/*
		 * An inactive sink's display stays powered: the firmware
		 * retrains only a powered display's link, and that link-up is
		 * how the sink comes back. Its release powers it off.
		 */
		if (v14 && v14->opened && !READ_ONCE(v14->failed) &&
		    !(dcp->external_native && dcp_v14_sink_inactive(dcp))) {
			mutex_lock(&v14->lock);
			if (v14->powered && dcp_v14_set_power(v14, false))
				dev_warn(dcp->dev, "display did not power down\n");
			mutex_unlock(&v14->lock);
		}
		dcp_mode_invalidate(&dcp->mode_state);
		return;
	}
	/* Blank to black, at 0 nits; the panel and the DCP stay powered. */
	if (v14 && v14->started) {
		if (dcp_backlight_active(dcp))
			dcp_backlight_dpms(dcp, false);
		dcp_v14_present(v14, NULL, 0, 0, 0, NULL);
	}
}
