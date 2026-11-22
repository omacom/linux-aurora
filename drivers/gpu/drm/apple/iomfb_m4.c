// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple DCP firmware 26.6.2 (T8132/J713, J715).
 *
 * The appledrm component and KMS paths own this controller.  This backend owns
 * only its generation-specific EP37 RPC ABI and retained firmware session.
 * Firmware can keep every published DMA address until an exact successor
 * completion; an uncertain transaction stays pinned until reboot.
 */
#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <drm/drm_edid.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_modes.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_rect.h>
#include <linux/iopoll.h>
#include <linux/iosys-map.h>

#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/soc/apple/dart.h>
#include <linux/soc/apple/dart-cio.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/pmp-display.h>
#include <linux/soc/apple/pmc.h>
#include <linux/soc/apple/pmgr-pwrstate.h>

#include "afk.h"
#include "dcp.h"
#include "dcp-sdr-edid.h"
#include "dptxep.h"
#include "epic/dpavservep.h"
#include "iomfb_m4.h"

#include "dcp-m4-profile.h"
#include "dcp-m4-protocol.h"
#include "dcp-m4-transaction.h"
#include "dcp-m4-timing.h"
#include "dcp-m4-memory.h"
#include "dcp-m4-callback.h"
#include "dcp-m4-property.h"
#include "dcp-m4-frame.h"
#include "dcp-refresh.h"
#include "parser.h"
#include "t8132_ext_drm.h"

const struct dcp_m4_profile *dcp_m4_pf = &dcp_m4_profile_j713;

static const struct dcp_m4_profile *dcp_m4_machine_profile(void)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(dcp_m4_profiles); i++)
		if (of_machine_is_compatible(dcp_m4_profiles[i]->machine))
			return dcp_m4_profiles[i];
	return NULL;
}

static unsigned int idle_cache_ms = 1000;

static int idle_cache_ms_set(const char *val, const struct kernel_param *kp)
{
	return param_set_uint_minmax(val, kp, 0, 60000);
}

static const struct kernel_param_ops idle_cache_ms_ops = {
	.set = idle_cache_ms_set,
	.get = param_get_uint,
};

module_param_cb(idle_cache_ms, &idle_cache_ms_ops, &idle_cache_ms, 0644);
MODULE_PARM_DESC(idle_cache_ms,
		 "J713: cache a static frame after this many ms without a flip (0: never, at most 60000; applies from the next swap)");

static unsigned int power_gate_ms;

static int power_gate_ms_set(const char *val, const struct kernel_param *kp)
{
	return param_set_uint_minmax(val, kp, 0, 600000);
}

static const struct kernel_param_ops power_gate_ms_ops = {
	.set = power_gate_ms_set,
	.get = param_get_uint,
};

module_param_cb(power_gate_ms, &power_gate_ms_ops, &power_gate_ms, 0644);
MODULE_PARM_DESC(power_gate_ms,
		 "J713: power the display coprocessor and its domains down after this many ms blanked, and before system sleep (0: only before a deep sleep, at most 600000)");

/*
 * DCP CPU control and status, in the coprocessor's ASC block. The idle bit
 * is dcp.c's APPLE_DCP_COPROC_CPU_STATUS_IDLE; macOS read status 0x6e at
 * its idle check before clearing RUN.
 */
#define DCP_M4_CPU_CONTROL	0x44
#define DCP_M4_CPU_RUN		BIT(4)
#define DCP_M4_CPU_STATUS	0x48
#define DCP_M4_CPU_STATUS_IDLE	BIT(5)

/* One timing of an external sink for its DRM connector. */
struct iomfb_m4_ext_mode {
	struct drm_display_mode mode;
	u32 timing, color;
	u32 bootstrap; /* the colour to set first, 0 for none */
	/* The timing's IsPreferred and firmware score */
	bool fw_preferred;
	s64 score;
	/* Its colour element is one the timing lists as needing DSC */
	bool dsc;
};

/* A scratch-only realtime bandwidth mailbox (D003) is this long */
#define DCP_M4_BW_MAILBOX_SIZE	8

/*
 * How many external direct swaps may be in flight (A407 answered, the D590
 * not yet in), 1 or 2, read at each frame. At depth 1 every flip event is
 * sent with its swap's D590. At depth 2 a frame's flip event can be sent
 * before the firmware confirms that the frame replaced the previous one, so a
 * compositor may draw into a buffer that is still on screen.
 */
#define DCP_M4_EXT_INFLIGHT	2
static unsigned int t8132_ext_swap_depth = DCP_M4_EXT_INFLIGHT;

static int t8132_ext_swap_depth_set(const char *val, const struct kernel_param *kp)
{
	return param_set_uint_minmax(val, kp, 1, DCP_M4_EXT_INFLIGHT);
}

static const struct kernel_param_ops t8132_ext_swap_depth_ops = {
	.set = t8132_ext_swap_depth_set,
	.get = param_get_uint,
};

module_param_cb(t8132_ext_swap_depth, &t8132_ext_swap_depth_ops, &t8132_ext_swap_depth, 0644);
MODULE_PARM_DESC(t8132_ext_swap_depth,
		 "J713 external direct swaps in flight at most, 2 (default) or 1, from the next frame. At 2 a flip event can come before the firmware confirms the frame replaced the previous one, so a compositor may draw into a buffer still on screen; at 1 every flip event comes with its swap's completion");

struct iomfb_m4 {
	struct apple_dcp *owner;
	struct device *dev;
	struct device *scanout;
	struct iommu_group *scanout_group;
	resource_size_t coproc_phys;
	size_t coproc_size;
	/* The realtime bandwidth mailbox from the DT (0: none), see dcp_m4_bw_mailbox() */
	u64 bw_mailbox;
	struct work_struct backlight_work;
	void __iomem *coproc;
	struct apple_rtkit *rtkit;
	struct mutex lock;
	spinlock_t state_lock;
	struct completion reply;
	struct dcp_m4_transaction transaction;
	struct platform_device *piodma;
	struct dcp_m4_memory *memory;
	struct dcp_m4_runtime *runtime;
	void *rpc;
	dma_addr_t rpc_dva;
	u64 ready_word;
	size_t allocated;
	int error;
	bool started, init_sent, ready, startup_started;
	bool powered_off; /* setPowerState(0) for system sleep */
	bool external; /* an adopted external DCP: no panel */
	/* AFK endpoints started on an adopted external DCP, by endpoint - 0x20. */
	struct apple_dcp_afkep *probe_ep[0x20];
	/*
	 * The sink's dcpav-service-epic on EP 0x28, the one last announced,
	 * and how many were: the firmware announces a new one for each sink
	 * and after each restart (state_lock)
	 */
	struct apple_epic_service *ext_avserv;
	unsigned int ext_avserv_gen;
	/* Probes again while the firmware has not read the sink yet (state_lock). */
	struct delayed_work ext_edid_retry;
	unsigned int ext_edid_retries;
	bool display_off; /* setPowerState(0) while blanked, until the next frame */
	u64 display_off_cycles;
	s64 display_off_us, display_off_us_max, display_on_us, display_on_us_max;
	/*
	 * Internal J713 DCP that can power its CPU and display domains down
	 * while blanked (apple,pmc-interface; power_gate_ms), and whether they
	 * are down: then no register of the DCP, its mailbox or its display
	 * DARTs is used until iomfb_m4_power_ungate().
	 */
	bool gate_capable;
	bool gated;
	/* External: its firmware (0) and display (1) DARTs, and stopped for a deep sleep. */
	struct apple_dart_cio *ext_darts[2];
	bool ext_stopped;
	struct delayed_work gate_work;
	u64 gate_cycles;
	s64 gate_off_us, gate_on_us;
	/* Exact completed successor or blank is the only framebuffer retirement. */
	struct drm_framebuffer *active_fb;
	struct drm_framebuffer *uncertain_fb;
	struct dcp_m4_frame active_frame;
	u64 flips, blanks, brightness_swaps;
	struct delayed_work idle_cache;
	/*
	 * Framebuffer references a window let go of, put
	 * by ext_put_work outside the session lock
	 */
	spinlock_t ext_put_lock;
	struct list_head ext_puts;
	struct work_struct ext_put_work;
	/* Woken when a direct swap in flight is done, or the display DART goes off */
	wait_queue_head_t ext_settle_wq;
	/*
	 * The firmware switched its display DART off (D572 powerUpDART(0)) and
	 * not on again: nothing may touch that DART meanwhile.
	 */
	bool ext_dart_down;
	/* ... since a system sleep (no D572(1) since) */
	bool ext_dart_since_sleep;
	/*
	 * Its display was released for a deep sleep with the cable left in
	 * (dcp.c), and is connected again after the wake in the background:
	 * the DRM replay waits for that (iomfb_m4_ext_relink_wait()).
	 * Meanwhile, until iomfb_m4_ext_relink_end(), its connector stays as
	 * it was: connected, its hotplugs held, its modes those of the link it
	 * had (ext_hold_capacity).
	 */
	bool ext_relink;
	u64 ext_hold_capacity;
	/*
	 * Its reconnect under way after the wake (armed), and the display
	 * found gone meanwhile (lost): the replay stops waiting for it. Both
	 * set under the owner's hpd_mutex, as its cable state is.
	 */
	bool ext_relink_armed, ext_relink_lost;
	/*
	 * From PM_SUSPEND_PREPARE, the DRM layer ends a hold for this sleep
	 * at PM_POST_SUSPEND (iomfb_m4_ext_relink_own()); without that, no
	 * hold starts.
	 */
	bool ext_relink_owned;
	/* The external PIODMA child, kept once created; ->piodma once usable */
	struct platform_device *ext_piodma;
	/* The display DART's state was found lost after a D572(1) */
	bool ext_dart_lost;
	/* The DVAs of D451 buffers reserved in SID16 too (pair) */
	u64 ext_pair_dva[64];
	unsigned int ext_pairs;
	/*
	 * The sink as its DRM connector sees it: whether one is connected (the
	 * last D575), and its timings with an 8-bpc RGB SDR colour (the last
	 * TimingElements), under ext_modes_lock alone.
	 */
	bool ext_sink;
	struct mutex ext_modes_lock; /* ext_modes, ext_mode_count, ext_pref*, ext_logged_* */
	struct iomfb_m4_ext_mode *ext_modes;
	unsigned int ext_mode_count;
	/*
	 * Its EDID as the connector publishes it (iomfb_m4_ext_edid()), with
	 * the link generation (ext_link_gen) and the AV service it was read
	 * in, also under ext_modes_lock
	 */
	const struct drm_edid *ext_edid;
	int ext_edid_link_gen;
	unsigned int ext_edid_avserv_gen;
	/*
	 * The sink generation, which a released route moves on (words received
	 * before carry the old one and publish nothing); whether a
	 * TimingElements of this sink was cached since it last went (D575(0), a
	 * released route); and its PreferredTimingElements.
	 */
	atomic_t ext_sink_gen;
	/*
	 * The PM epoch, which every system suspend and resume moves on: a
	 * message received in an earlier one is stale for power (a D572(1) from
	 * before a sleep confirms nothing).
	 */
	atomic_t ext_pm_epoch;
	bool ext_catalog_open;
	struct dcp_timing_id ext_pref[8];
	unsigned int ext_pref_count;
	/* what iomfb_m4_ext_get_modes() last logged */
	unsigned int ext_logged_modes, ext_logged_link;
	int ext_logged_pick;
};

static void dcp_m4_runtime_wake(struct iomfb_m4 *dcp);
struct dcp_m4_runtime;
static void dcp_m4_ext_async_work(struct work_struct *work);
static int dcp_m4_set_power_state(struct iomfb_m4 *dcp, bool on);
static int iomfb_m4_power_gate(struct iomfb_m4 *dcp);
static int iomfb_m4_power_ungate(struct iomfb_m4 *dcp);
static void iomfb_m4_gate_work(struct work_struct *work);
static void dcp_m4_runtime_receive(struct iomfb_m4 *dcp, u64 word);

static int dcp_m4_dart_check(struct device *dev)
{
	return apple_dart_dcp_check(dev);
}

static int dcp_m4_dart_check_buffer(struct device *dev, dma_addr_t iova, size_t size)
{
	return apple_dart_dcp_check_buffer(dev, iova, size);
}

static void *dcp_m4_dma_alloc(struct device *dev, size_t size, dma_addr_t *dva)
{
	return apple_dart_dcp_alloc(dev, size, dva);
}

static int dcp_m4_pmp_get(struct device *dev)
{
	return apple_pmp_j713_display_get(dev);
}

static int dcp_m4_pmp_check(struct device *dev)
{
	return apple_pmp_j713_display_check(dev);
}

static bool dcp_m4_range(u64 base, u64 length, u64 addr, u64 size)
{
	return length && size && addr >= base && addr - base < length &&
		size <= length - (addr - base);
}

/* Only the consumer's own no-map ASC reservations may back firmware buffers. */
static bool dcp_m4_reserved(struct iomfb_m4 *dcp, u64 pa, size_t size, bool oslog)
{
	struct device_node *node;
	struct resource res;
	bool found = false;

	for (int i = 0; (node = of_parse_phandle(dcp->dev->of_node, "memory-region", i)); i++) {
		struct reserved_mem *rmem = of_reserved_mem_lookup(node);

		if (of_device_is_compatible(node, "apple,asc-mem") &&
		    of_property_read_bool(node, "no-map") &&
		    oslog == of_property_read_bool(node, "apple,dcp-os-log") &&
		    !of_address_to_resource(node, 0, &res) &&
		    rmem && rmem->base == res.start && rmem->size == resource_size(&res) &&
		    dcp_m4_range(res.start, resource_size(&res), pa, size))
			found = true;
		of_node_put(node);
		if (found)
			break;
	}
	return found;
}

static void dcp_m4_failed(struct iomfb_m4 *dcp, int error)
{
	unsigned long flags;
	bool first;

	spin_lock_irqsave(&dcp->state_lock, flags);
	first = !dcp->error;
	if (first)
		dcp->error = error;
	dcp->ready = false;
	WRITE_ONCE(dcp->owner->crashed, true);
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (first)
		dev_err(dcp->dev, "Retained DCP session quarantined: %d\n", error);
	complete_all(&dcp->reply);
	dcp_m4_runtime_wake(dcp);
}

static int dcp_m4_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct iomfb_m4 *dcp = cookie;
	struct iommu_domain *domain = iommu_get_domain_for_dev(dcp->dev);
	phys_addr_t pa;
	int ret = -EINVAL;

	if (!bfr->size || bfr->size > SZ_16M || dcp->allocated > SZ_64M - bfr->size ||
	    dcp_m4_dart_check(dcp->dev))
		goto fail;
	if (!bfr->iova) {
		bfr->buffer = dcp_m4_dma_alloc(dcp->dev, bfr->size, &bfr->iova);
		if (IS_ERR(bfr->buffer)) {
			ret = PTR_ERR(bfr->buffer);
			bfr->buffer = NULL;
			goto fail;
		}
		/* Allocation stays pinned even if subsequent transport validation fails. */
		dcp->allocated += bfr->size;
		if (!dcp_m4_range(1ULL << 40, 1ULL << 36, bfr->iova, bfr->size) ||
		    dcp_m4_dart_check(dcp->dev))
			goto fail;
		bfr->is_mapped = false;
		dev_info(dcp->dev, "RTKit allocated %zu bytes at %pad\n", bfr->size, &bfr->iova);
		return 0;
	}
	/* OSLOG is explicitly marked physical; other requests are SID23 DVAs. */
	if (dcp_m4_reserved(dcp, bfr->iova, bfr->size, true)) {
		/* A physical address can also fall in the DVA aperture; refuse ambiguity. */
		for (size_t off = 0; off < bfr->size;) {
			if (iommu_iova_to_phys(domain, bfr->iova + off))
				goto fail;
			off += SZ_16K - ((bfr->iova + off) & (SZ_16K - 1));
		}
		if (dcp_m4_dart_check(dcp->dev))
			goto fail;
		pa = bfr->iova;
	} else {
		if (!dcp_m4_range(1ULL << 40, 1ULL << 36, bfr->iova, bfr->size))
			goto fail;
		ret = dcp_m4_dart_check_buffer(dcp->dev, bfr->iova, bfr->size);
		if (ret)
			goto fail;
		ret = -EINVAL;
		pa = iommu_iova_to_phys(domain, bfr->iova);
		if (!pa || !dcp_m4_reserved(dcp, pa, bfr->size, false))
			goto fail;
		/* Include the initial unaligned span and every crossed 16-KiB page. */
		for (size_t off = SZ_16K - (bfr->iova & (SZ_16K - 1)); off < bfr->size;
		     off += SZ_16K)
			if (iommu_iova_to_phys(domain, bfr->iova + off) != pa + off)
				goto fail;
	}
	/* Firmware payload is Normal-NC, independent of coherent WB table walks. */
	bfr->iomem = ioremap_wc(pa, bfr->size);
	if (!bfr->iomem) {
		ret = -ENOMEM;
		goto fail;
	}
	bfr->is_mapped = true;
	dcp->allocated += bfr->size;
	dev_info(dcp->dev, "RTKit borrowed %zu bytes at %pad -> %pa\n",
		 bfr->size, &bfr->iova, &pa);
	return 0;
fail:
	dcp_m4_failed(dcp, ret);
	return ret;
}

static void dcp_m4_shmem_destroy(void *cookie, struct apple_rtkit_shmem *bfr)
{
	/* No qualified ownership-return boundary yet. Never recycle firmware DMA. */
}

static void dcp_m4_crashed(void *cookie, const void *log, size_t size)
{
	struct iomfb_m4 *dcp = cookie;

	dev_err(dcp->dev, "RTKit crashed; retaining session and DMA until reboot\n");
	dcp_m4_failed(dcp, -EIO);
}

static void dcp_m4_receive(void *cookie, u8 endpoint, u64 message)
{
	struct iomfb_m4 *dcp = cookie;
	unsigned long flags;
	int ret;

	dev_dbg(dcp->dev, "RTKit EP%02x word %016llx\n", endpoint, message);
	if (endpoint != DCP_M4_ENDPOINT) {
		struct apple_dcp_afkep *ep = NULL;

		if (dcp->external && endpoint == DPTX_ENDPOINT)
			ep = READ_ONCE(dcp->owner->dptxep);
		else if (dcp->external && endpoint >= 0x20 && endpoint < 0x40)
			ep = READ_ONCE(dcp->probe_ep[endpoint - 0x20]);
		if (ep) {
			afk_receive_message(ep, message);
			return;
		}
		dcp_m4_failed(dcp, -EPROTO);
		return;
	}
	if (READ_ONCE(dcp->runtime)) {
		dcp_m4_runtime_receive(dcp, message);
		return;
	}
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp_m4_ready_notification(message, dcp->init_sent, dcp->ready_word);
	if (ret == 1)
		dcp->ready_word = message;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret < 0)
		dcp_m4_failed(dcp, ret);
	else if (ret == 1)
		complete_all(&dcp->reply);
}

static const struct apple_rtkit_ops dcp_m4_rtkit_ops = {
	.recv_message = dcp_m4_receive,
	.crashed = dcp_m4_crashed,
	.shmem_setup = dcp_m4_shmem_setup,
	.shmem_destroy = dcp_m4_shmem_destroy,
};

static int dcp_m4_adopt(struct iomfb_m4 *dcp)
{
	struct device *dev = dcp->dev;
	unsigned long flags;
	u64 init_word;
	int ret;

	if (dcp->started) {
		ret = -EBUSY;
		goto out;
	}
	ret = dcp_m4_dart_check(dev);
	if (ret)
		goto out;
	if (!(readl(dcp->coproc + 0x44) & BIT(4))) {
		ret = -ENODEV;
		goto out;
	}
	/* From the first message onward, neither failure nor unbind may tear down. */
	dcp->started = true;
	dev_info(dev, "Starting warm RTKit adoption; retaining firmware ownership\n");
	ret = apple_rtkit_wake(dcp->rtkit);
	if (ret)
		goto fail;
	if (READ_ONCE(dcp->error)) {
		ret = READ_ONCE(dcp->error);
		goto fail;
	}
	if (!apple_rtkit_has_endpoint(dcp->rtkit, DCP_M4_ENDPOINT)) {
		ret = -ENODEV;
		goto fail;
	}
	dev_info(dev, "RTKit awake; endpoint 0x37 advertised\n");
	dcp->rpc = dcp_m4_dma_alloc(dev, DCP_M4_RPC_SIZE, &dcp->rpc_dva);
	if (IS_ERR(dcp->rpc)) {
		ret = PTR_ERR(dcp->rpc);
		dcp->rpc = NULL;
		goto fail;
	}
	ret = dcp_m4_dart_check(dev);
	if (ret)
		goto fail;
	ret = dcp_m4_init_word(dcp->rpc_dva, &init_word);
	if (ret)
		goto fail;
	if (!dcp_m4_range(1ULL << 40, 1ULL << 36, dcp->rpc_dva, DCP_M4_RPC_SIZE)) {
		ret = -ERANGE;
		goto fail;
	}
	memset(dcp->rpc, 0, DCP_M4_RPC_SIZE);
	put_unaligned_le32(DCP_M4_ALIGNMENT, dcp->rpc + 12);
	dma_wmb();
	ret = apple_rtkit_start_ep(dcp->rtkit, DCP_M4_ENDPOINT);
	if (ret)
		goto fail;
	dev_info(dev, "Sending INIT %016llx RPC=%pad size=%x alignment=%u\n",
		 init_word, &dcp->rpc_dva, DCP_M4_RPC_SIZE, DCP_M4_ALIGNMENT);
	spin_lock_irqsave(&dcp->state_lock, flags);
	dcp->init_sent = true;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	ret = apple_rtkit_send_message(dcp->rtkit, DCP_M4_ENDPOINT, init_word, NULL, false);
	if (ret)
		goto fail;
	if (!wait_for_completion_timeout(&dcp->reply, msecs_to_jiffies(3000))) {
		ret = -ETIMEDOUT;
		goto fail;
	}
	ret = READ_ONCE(dcp->error);
	if (ret)
		goto fail;
	if (!dcp_m4_ready(dcp->ready_word) || !(readl(dcp->coproc + 0x44) & BIT(4))) {
		ret = -EPROTO;
		goto fail;
	}
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp->error;
	if (!ret)
		dcp->ready = true;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret)
		goto fail;
	dev_info(dev, "DCPLink READY %016llx\n",
		 dcp->ready_word);
	ret = 0;
	goto out;
fail:
	dcp_m4_failed(dcp, ret);
	dev_err(dev, "READY failed: %d; session pinned until guarded reboot\n", ret);
out:
	return ret;
}

static int dcp_m4_create_piodma_ext(struct iomfb_m4 *dcp);

/*
 * An external DCP's PIODMA device, created or checked when the firmware first
 * needs it (D451, D201), and only while the firmware has its display DART on
 * (the last D572, at the time of the access): it powers that DART only once the
 * display is being powered on. 0 if usable.
 */
static int dcp_m4_ext_piodma(struct iomfb_m4 *dcp, const char *why)
{
	int ret;

	if (READ_ONCE(dcp->ext_dart_down)) {
		dev_dbg(dcp->dev, "%s: the display DART is off: no PIODMA\n", why);
		return -EAGAIN;
	}
	if (dcp->piodma)
		return 0;
	ret = dcp_m4_create_piodma_ext(dcp);
	dev_dbg(dcp->dev, "%s: PIODMA created on demand: %d\n", why, ret);
	return ret;
}

static int dcp_m4_shared_alloc(void *cookie, u32 size, u64 *phys, u64 *dva)
{
	struct iomfb_m4 *dcp = cookie;
	int ret;

	/*
	 * D451 maps through the coprocessor's own DART (SID23). An
	 * external DCP with a usable PIODMA consumer also reserves the slot in
	 * its display DART's SID16, so that D201 maps it there at the same DVA;
	 * without one the buffer is SID23-only, and a D201 maps it in SID16 at
	 * a DVA of its own.
	 */
	if (dcp->external) {
		/*
		 * The display DART's own readiness too (its power confirmed since
		 * sleep), not only PIODMA's: else, or if the paired allocation
		 * finds it unconfirmed before publishing anything (-EAGAIN), the
		 * buffer is SID23-only.
		 */
		if (dcp_m4_ext_piodma(dcp, "D451") ||
		    apple_dart_dcp_dispext_stream_ok(&dcp->piodma->dev, 16))
			return apple_dart_dcp_shared_alloc_link(dcp->dev, size, phys, dva);
		ret = apple_dart_dcp_shared_alloc(dcp->dev, &dcp->piodma->dev, size, phys, dva);
		if (ret == -EAGAIN) {
			dev_dbg(dcp->dev, "D451: the display DART is not confirmed: SID23 only\n");
			return apple_dart_dcp_shared_alloc_link(dcp->dev, size, phys, dva);
		}
		if (!ret && dcp->ext_pairs < ARRAY_SIZE(dcp->ext_pair_dva))
			dcp->ext_pair_dva[dcp->ext_pairs++] = *dva;
		return ret;
	}
	if (!dcp->piodma)
		return -ENODEV;
	return apple_dart_dcp_shared_alloc(dcp->dev, &dcp->piodma->dev, size, phys, dva);
}

static int dcp_m4_shared_map(void *cookie, u64 phys, u64 dva, u32 size)
{
	struct iomfb_m4 *dcp = cookie;

	if (!dcp->piodma)
		return -ENODEV;
	return apple_dart_dcp_shared_map(dcp->dev, &dcp->piodma->dev, phys, dva, size);
}

/*
 * Whether an external DCP's display DART stream for a D201 flag may be
 * used now: the firmware has the DART on (its last D572), its state was not
 * found lost since, and PIODMA is usable (created on demand).
 */
static int dcp_m4_ext_usable(void *cookie, unsigned int flag)
{
	struct iomfb_m4 *dcp = cookie;
	int ret;

	if (!dcp->external)
		return -EOPNOTSUPP;
	if (READ_ONCE(dcp->ext_dart_lost))
		ret = -EIO;
	else
		ret = dcp_m4_ext_piodma(dcp, flag ? "D201 (SID16)" : "D201 (SID0)");
	/* The selected stream itself, cached mappings included: not retired. */
	if (!ret)
		ret = apple_dart_dcp_dispext_stream_ok(&dcp->piodma->dev, flag ? 16 : 0);
	if (ret)
		dev_dbg(dcp->dev, "D201 flag %u refused: %d%s\n", flag, ret,
			READ_ONCE(dcp->ext_dart_lost) ? " (display DART state lost)" : "");
	return ret;
}

/* An external DCP's D201 with flag 0: the backing in its display DART's SID0. */
static int dcp_m4_shared_map_sid0(void *cookie, u64 phys, u32 size, u64 *dva)
{
	struct iomfb_m4 *dcp = cookie;
	int ret = apple_dart_dcp_dispext_map(&dcp->piodma->dev, 0, phys, size, dva);

	dev_dbg(dcp->dev, "D201 flag 0: %#x bytes in SID0 at IOVA %#llx: %d\n",
		size, ret ? 0 : *dva, ret);
	return ret;
}

/*
 * An external DCP's D201 with flag 1: a buffer paired at D451 is
 * mapped in SID16 at its D451 DVA, any other at a DVA of its own.
 */
static int dcp_m4_shared_map_sid16(void *cookie, u64 phys, u64 dva, u32 size, u64 *out)
{
	struct iomfb_m4 *dcp = cookie;
	bool paired = false;
	int ret;

	for (unsigned int i = 0; i < dcp->ext_pairs; i++)
		paired |= dcp->ext_pair_dva[i] == dva;
	if (paired) {
		ret = apple_dart_dcp_shared_map(dcp->dev, &dcp->piodma->dev, phys, dva, size);
		*out = dva;
	} else {
		ret = apple_dart_dcp_dispext_map(&dcp->piodma->dev, 16, phys, size, out);
	}
	dev_dbg(dcp->dev, "D201 flag 1: %#x bytes in SID16 at IOVA %#llx (%s): %d\n",
		size, ret ? 0 : *out, paired ? "D451 DVA" : "own DVA", ret);
	return ret;
}

static const struct dcp_m4_memory_ops dcp_m4_memory_ops = {
	.alloc = dcp_m4_shared_alloc,
	.map = dcp_m4_shared_map,
	.map_sid0 = dcp_m4_shared_map_sid0,
	.map_sid16 = dcp_m4_shared_map_sid16,
	.usable = dcp_m4_ext_usable,
};

static int dcp_m4_create_piodma(struct iomfb_m4 *dcp)
{
	struct device_node *node, *alias;
	struct platform_device *existing;
	int ret = -ENODEV;

	node = of_get_child_by_name(dcp->dev->of_node, "piodma");
	alias = of_find_node_by_path("disp0_piodma");
	if (!node || node != alias || !of_device_is_available(node) ||
	    !of_property_read_bool(node, "apple,j713-inherited-mappings"))
		goto out;
	existing = of_find_device_by_node(node);
	if (existing) {
		put_device(&existing->dev);
		ret = -EBUSY;
		goto out;
	}
	dcp->piodma = of_platform_device_create(node, NULL, dcp->dev);
	if (!dcp->piodma)
		goto out;
	/* Keep the child even on failure: retained-domain attachment can have begun. */
	ret = dma_set_mask_and_coherent(&dcp->piodma->dev, DMA_BIT_MASK(42));
	if (!ret)
		ret = of_dma_configure(&dcp->piodma->dev, node, true);
	if (!ret)
		ret = dcp_m4_dart_check(&dcp->piodma->dev);
out:
	of_node_put(alias);
	of_node_put(node);
	return ret;
}

/*
 * An external DCP's PIODMA device, the piodma child of its node on SID16 of its
 * display DART (adopted once the boot loader has handed the DCP over), as the
 * internal display's. Disabled in the device tree, it is enabled here for this
 * one consumer. Kept once created.
 */
static int dcp_m4_create_piodma_ext(struct iomfb_m4 *dcp)
{
	struct device_node *node = of_get_child_by_name(dcp->dev->of_node, "piodma");
	struct platform_device *existing;
	int ret = 0;

	/* Its display DART is read here: not while the firmware has it off. */
	if (READ_ONCE(dcp->ext_dart_down)) {
		of_node_put(node);
		return -EAGAIN;
	}
	if (dcp->piodma) {
		of_node_put(node);
		return dcp_m4_dart_check(&dcp->piodma->dev);
	}
	if (!node)
		return -ENODEV;
	/* A child that did not attach yet (its DART probed later) attaches again. */
	if (dcp->ext_piodma)
		goto configure;
	if (!of_device_is_available(node)) {
		struct of_changeset *cs = kzalloc(sizeof(*cs), GFP_KERNEL);

		if (!cs) {
			ret = -ENOMEM;
			goto out;
		}
		of_changeset_init(cs);
		ret = of_changeset_update_prop_string(cs, node, "status", "okay");
		if (!ret)
			ret = of_changeset_apply(cs);
		/* Applied, it stays: the live tree refers to it. */
		if (ret) {
			of_changeset_destroy(cs);
			kfree(cs);
			goto out;
		}
	}
	existing = of_find_device_by_node(node);
	if (existing) {
		put_device(&existing->dev);
		ret = -EBUSY;
		goto out;
	}
	dcp->ext_piodma = of_platform_device_create(node, NULL, dcp->dev);
	if (!dcp->ext_piodma) {
		ret = -ENODEV;
		goto out;
	}
configure:
	/*
	 * Keep the child even on failure: retained-domain attachment can have
	 * begun. Only a checked one serves the firmware's buffers.
	 */
	ret = dma_set_mask_and_coherent(&dcp->ext_piodma->dev, DMA_BIT_MASK(42));
	if (!ret)
		ret = of_dma_configure(&dcp->ext_piodma->dev, node, true);
	if (!ret)
		ret = dcp_m4_dart_check(&dcp->ext_piodma->dev);
	if (!ret)
		dcp->piodma = dcp->ext_piodma;
out:
	if (ret && ret != -EAGAIN)
		dev_warn(dcp->dev, "PIODMA on the display DART's SID16: %d\n", ret);
	of_node_put(node);
	return ret;
}

/*
 * A host window of SID0 (dva, 0: not claimed) and the framebuffer it maps (key,
 * NULL: empty), with a reference held by @put's owner; whether the last
 * confirmed swap shows it (given-up swaps that may still show it are in
 * ext_lost_swap); pinned: its unmap failed, kept until reboot.
 */
#define DCP_M4_EXT_WINDOWS	6
struct dcp_m4_ext_window {
	u64 dva;
	void *key;
	void (*put)(void *key);
	unsigned int count;
	bool on_screen, pinned;
	unsigned long used;
};

/*
 * A swap given up before its D590 (see dcp_m4_ext_swap_give_up()): its ID (0:
 * a free slot), the window it may still put on screen (-1: none, or retired
 * by a confirmed swap since), and whether a newer swap was shown before it.
 */
struct dcp_m4_ext_lost {
	u32 id;
	int win;
	bool superseded;
};

/*
 * A direct swap whose A407 reply came and whose D590 has not: its state, its
 * window (-1: none), the link generation and the deadline it went out with,
 * whether its flip waits for its completion or the older one's, whether a
 * and whether a newer swap was shown before it completed.
 */
struct dcp_m4_ext_inflight {
	struct dcp_m4_swap_state swap;
	int win;
	int gen;
	unsigned long deadline;
	bool flip_held;
	bool superseded;
};

/* How a direct swap leaves flight, see dcp_m4_ext_async_done(). */
enum dcp_m4_ext_end {
	DCP_M4_EXT_END_SHOWN,	/* its own D590, displayed */
	DCP_M4_EXT_END_KEPT,	/* named in a later swap's D590, or shown before a newer one */
	DCP_M4_EXT_END_UNSHOWN,	/* its own D590, not displayed */
	DCP_M4_EXT_END_TAKEN,	/* its D590 before its A407 reply, taken account of then */
};

/* What the D590 being handled says: shown, and the earlier swaps it completes. */
struct dcp_m4_ext_done {
	bool valid;
	bool shown;
	u32 count;
	u32 hol[4];
};

struct dcp_m4_runtime {
	struct iomfb_m4 *dcp;
	struct dcp_m4_properties *properties;
	struct dcp_m4_timing timing;
	struct dcp_m4_swap_state swap;
	struct completion received;
	struct workqueue_struct *wq;
	struct work_struct idle;
	u64 queue[32];
	/* The sink generation each queued word was received in, see ext_sink_gen */
	int queue_gen[32];
	/* ... and the PM epoch it was received in (ext_pm_epoch) */
	int queue_pm[32];
	/* ... of the word last taken, and of the callback being handled */
	int word_gen, cb_gen;
	int word_pm, cb_pm;
	/*
	 * Host calls of our own (not remote: not made inside an async callback)
	 * outstanding, and the PM epoch the innermost one was sent in
	 */
	unsigned int ext_local_calls;
	int ext_call_epoch;
	/*
	 * The display down for a system sleep (from PM_SUSPEND_PREPARE to
	 * PM_POST_SUSPEND: frames refused, enables and disables no-ops; the DRM
	 * state is replayed after it, t8132_ext_drm.c)
	 */
	bool ext_sleeping;
	/*
	 * The replay after a sleep, from its wake to
	 * iomfb_m4_ext_restore_result() or iomfb_m4_ext_restore_end(): whether
	 * a D572(1) confirmed the display DART since (cleared by a D572(0)),
	 * and whether a frame was shown since; the PM epoch at the sleep's
	 * PREPARE, and the link and sink generations at the wake
	 */
	bool ext_restore_pending;
	bool ext_restore_dart;
	bool ext_restore_frame;
	int ext_restore_epoch;
	int ext_restore_link_gen;
	int ext_restore_sink_gen;
	u32 head, count, callbacks;
	unsigned long deadline;
	bool done;
	bool background_started, background_done;
	u32 background_id;
	bool power_started, power_done;
	bool scanout_started, scanout_done;
	bool qualified; /* KMS startup qualification passed; see dcp_m4_runtime_live() */
	void *scanout_cpu;
	struct dcp_m4_frame scanout_frame;
	/*
	 * Idle caching: one cache surface, allocated at startup, retained like
	 * all DCP-visible DMA and never written by the host.
	 */
	void *idle_cpu;
	struct dcp_m4_frame idle_frame;
	u32 idle_state, idle_fences;
	bool idle_registered;	/* from its first intent swap until D580 */
	bool idle_unavailable;
	u64 idle_entries;
	/*
	 * The sink generation of the chunked property under way, from its D127,
	 * and whether it is one (a D127 began it, and every chunk and its D129
	 * came in that generation)
	 */
	int ext_prop_gen;
	bool ext_prop_open, ext_prop_whole;
	/* The sink is connected (D575) */
	bool ext_connected;
	/* The size of the mode the firmware accepted, 0 before one */
	u32 ext_width, ext_height;
	/*
	 * The output as DRM drives it: powered on (A473), the timing and colour
	 * set (A413, 0 for none), and two host-written SID0 surfaces swapped in
	 * turn, with what each still lacks.
	 */
	bool ext_powered;
	u32 ext_cur_timing, ext_cur_color;
	void *ext_buf_cpu[2];
	struct dcp_m4_frame ext_buf[2];
	bool ext_buf_stale[2];
	bool ext_buf_pending; /* the mode is set, its surfaces are not yet */
	unsigned int ext_back;
	struct drm_rect ext_prev_damage;
	/*
	 * The host windows; the direct swaps in flight without waiting for their
	 * D590, oldest first, and how many; the window of the direct swap going
	 * out (-1: none); how many the next swap_full may leave in flight,
	 * returning after its A407 reply (0: it waits for its D590), and whether
	 * its flip may go at once (none older in flight); what the D590 being
	 * handled says (dcp_m4_external_callback()), and whether the swap going
	 * out was shown, by a D590 before its A407 reply; the work that gives the
	 * oldest up at its deadline or when its link goes.
	 */
	struct dcp_m4_ext_window ext_win[DCP_M4_EXT_WINDOWS];
	struct dcp_m4_ext_inflight ext_q[DCP_M4_EXT_INFLIGHT];
	unsigned int ext_qn;
	int ext_swap_win;
	unsigned int ext_swap_async;
	bool ext_swap_now;
	struct dcp_m4_ext_done ext_done;
	bool ext_swap_shown;
	struct delayed_work ext_async_work;
	/*
	 * Direct swaps completed by a later swap's D590 (named in it), whose own
	 * D590 may still come: dropped then (0: a free slot). Never evicted: with
	 * no slot for each swap in flight no direct swap goes out, and A406
	 * giving one of these IDs again orphans its start, as for ext_lost_swap.
	 */
	u32 ext_early[16];
	/*
	 * The link's generation, which a released route
	 * (iomfb_m4_external_link_gone()) and a D575(0) move on.
	 */
	atomic_t ext_link_gen;
	/*
	 * Swaps given up before their D590 (see dcp_m4_ext_swap_give_up()),
	 * each kept until its D590 comes (ID 0: a free slot), so that the late
	 * D590 is told apart and dropped, with the window each may still show.
	 */
	struct dcp_m4_ext_lost ext_lost_swap[16];
	/*
	 * The given-up swap that holds swaps (0: none), and whether a DRM
	 * enable lets one try (a probe); surface of the swap in flight (-1:
	 * none or surfaceless); the surface on screen as the last confirmed
	 * swap left it (-1: none); and per surface, the given-up swap that may
	 * still put it on screen (0: none).
	 */
	u32 ext_hold_id;
	bool ext_hold_probe;
	/*
	 * A swap A406 started under an unresolved ID and the host never
	 * submitted (0: none). Nothing ends or discards a started swap from the
	 * host, so no swap starts after it (until reboot).
	 */
	u32 ext_orphan_start;
	/* The last DRM enable failed: no frame until one succeeds */
	bool ext_mode_stale;
	int ext_swap_surface;
	int ext_front;
	u32 ext_buf_lost[2];
};

static void dcp_m4_ext_async_done(struct dcp_m4_runtime *r, int i, enum dcp_m4_ext_end end);
static void dcp_m4_ext_shown(struct dcp_m4_runtime *r, u32 id, int win, int older,
			     enum dcp_m4_ext_end end);

static void dcp_m4_runtime_wake(struct iomfb_m4 *dcp)
{
	struct dcp_m4_runtime *r = smp_load_acquire(&dcp->runtime);

	if (r)
		complete_all(&r->received);
}

static void dcp_m4_runtime_receive(struct iomfb_m4 *dcp, u64 word)
{
	struct dcp_m4_runtime *r = smp_load_acquire(&dcp->runtime);
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&dcp->state_lock, flags);
	if (dcp->error)
		ret = dcp->error;
	else if (r->count == ARRAY_SIZE(r->queue)) {
		/* Publish failure before another owner can consume a queued reply. */
		dcp->error = ret = -ENOSPC;
		dcp->ready = false;
	} else {
		r->queue_pm[(r->head + r->count) % ARRAY_SIZE(r->queue)] =
			atomic_read(&dcp->ext_pm_epoch);
		r->queue_gen[(r->head + r->count) % ARRAY_SIZE(r->queue)] =
			atomic_read(&dcp->ext_sink_gen);
		r->queue[(r->head + r->count++) % ARRAY_SIZE(r->queue)] = word;
		complete(&r->received);
	}
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	if (ret)
		dcp_m4_failed(dcp, ret);
	else
		queue_work(r->wq, &r->idle);
}

/* Called only while holding dcp->lock, by the synchronous RPC owner or idle worker. */
static int dcp_m4_runtime_next(struct dcp_m4_runtime *r, u64 *word, bool wait)
{
	struct iomfb_m4 *dcp = r->dcp;
	unsigned long flags, now;
	int ret;

	if (READ_ONCE(dcp->error))
		return READ_ONCE(dcp->error);
	if (wait) {
		now = jiffies;
		if (time_after_eq(now, r->deadline) ||
		    !wait_for_completion_timeout(&r->received, r->deadline - now))
			return -ETIMEDOUT;
	} else if (!try_wait_for_completion(&r->received)) {
		return -EAGAIN;
	}
	spin_lock_irqsave(&dcp->state_lock, flags);
	ret = dcp->error ? dcp->error : !r->count ? -EPROTO : 0;
	if (!ret) {
		r->word_gen = r->queue_gen[r->head % ARRAY_SIZE(r->queue)];
		r->word_pm = r->queue_pm[r->head % ARRAY_SIZE(r->queue)];
		*word = r->queue[r->head++ % ARRAY_SIZE(r->queue)];
		r->count--;
	}
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	return ret;
}

static int dcp_m4_runtime_health(struct dcp_m4_runtime *r)
{
	struct iomfb_m4 *dcp = r->dcp;
	int ret = READ_ONCE(dcp->error);

	/* Powered down by its owner: only the failure latch. */
	if (READ_ONCE(dcp->gated))
		return ret;

	if (!ret && (!(readl(dcp->coproc + 0x44) & BIT(4)) || dcp_m4_dart_check(dcp->dev) ||
		     (!dcp->external && dcp_m4_dart_check(&dcp->piodma->dev))))
		ret = -EIO;
	if (!ret && r->power_done)
		ret = dcp_m4_pmp_check(dcp->dev);
	if (!ret && dcp->scanout)
		ret = dcp_m4_dart_check(dcp->scanout);
	return ret;
}

/*
 * Full DART/PMP/CPU qualification runs through startup and KMS creation. After
 * that, per-message, per-swap and per-frame checks are only the failure latch;
 * RTKit crashes and protocol errors still latch it asynchronously. The full
 * checks remain available on demand through scanout_probe/pmp_vote reads.
 */
static int dcp_m4_runtime_live(struct dcp_m4_runtime *r)
{
	if (READ_ONCE(r->qualified))
		return READ_ONCE(r->dcp->error);
	return dcp_m4_runtime_health(r);
}

static int dcp_m4_runtime_send(struct dcp_m4_runtime *r, u64 word)
{
	struct iomfb_m4 *dcp = r->dcp;
	int ret = dcp_m4_runtime_live(r);

	if (ret)
		return ret;
	dma_wmb();
	return apple_rtkit_send_message(dcp->rtkit, DCP_M4_ENDPOINT, word, NULL, false);
}

static int dcp_m4_runtime_call(void *cookie, unsigned int method, bool remote,
				const void *input, u32 input_size, void *output, u32 output_size);

/*
 * An external DCP's sink description arrives as properties: chunked ones
 * (D127 start, D128 chunks, D129 end with the name), strings (D567) and
 * 4 KiB dictionaries (D552/D561). They are stored as on the internal DCP;
 * the sink's timing catalog (TimingElements, PreferredTimingElements) is
 * taken from them for its DRM connector.
 */
static bool dcp_m4_ext_cache_modes(struct iomfb_m4 *dcp, const void *blob, u32 size, int gen);
static bool dcp_m4_ext_cache_preferred(struct iomfb_m4 *dcp, const void *blob, u32 size,
				       int gen);
static void dcp_m4_ext_sink_gone(struct iomfb_m4 *dcp, bool route);

static void dcp_m4_ext_property(struct dcp_m4_runtime *r, unsigned int method,
				const u8 *in)
{
	struct iomfb_m4 *dcp = r->dcp;
	struct dcp_m4_properties *props = r->properties;
	int slot, gen;

	switch (method) {
	case 127:
		/* The property's sink generation is its D127's. */
		r->ext_prop_gen = r->cb_gen;
		r->ext_prop_open = true;
		r->ext_prop_whole = true;
		break;
	case 128:
		if (!r->ext_prop_open || r->cb_gen != r->ext_prop_gen)
			r->ext_prop_whole = false;
		break;
	case 129:
		/*
		 * A property whose D127, chunks and D129 did not all come
		 * in one sink generation publishes nothing (it is acknowledged
		 * as any other); one that did is checked in its D127's.
		 */
		if (!r->ext_prop_open || r->cb_gen != r->ext_prop_gen)
			r->ext_prop_whole = false;
		r->ext_prop_open = false;
		gen = r->ext_prop_whole ? r->ext_prop_gen : INT_MIN;
		slot = dcp_m4_raw_slot(props, 0, (const char *)in, false);
		/* The connector's modes, and a hotplug event. */
		if (slot >= 0 && !strcmp(props->raw[slot].name, "TimingElements") &&
		    props->raw[slot].data &&
		    dcp_m4_ext_cache_modes(dcp, props->raw[slot].data, props->raw[slot].size,
					   gen))
			t8132_ext_drm_hotplug(dcp->dev);
		/* The sink's preferred timing, for the preferred mode. */
		if (slot >= 0 && !strcmp(props->raw[slot].name, "PreferredTimingElements") &&
		    props->raw[slot].data &&
		    dcp_m4_ext_cache_preferred(dcp, props->raw[slot].data, props->raw[slot].size,
					       gen))
			t8132_ext_drm_hotplug(dcp->dev);
		break;
	}
}

static int dcp_m4_runtime_property(void *cookie, unsigned int method,
				    const void *input, void *output)
{
	struct dcp_m4_runtime *r = cookie;
	int ret = dcp_m4_property_callback(r->properties, method, input, output);

	if (!ret && r->dcp->external)
		dcp_m4_ext_property(r, method, input);
	return ret;
}

static int dcp_m4_runtime_memory(void *cookie, unsigned int method,
				  const void *input, void *output)
{
	struct dcp_m4_runtime *r = cookie;
	struct iomfb_m4 *dcp = r->dcp;
	const u8 *out = output;
	int ret;

	ret = dcp_m4_memory_callback(dcp->memory, &dcp_m4_memory_ops, dcp, method, input, output);
	if (dcp->external && ret)
		dev_warn(dcp->dev, "D%03u failed: %d (PIODMA %s)\n", method, ret,
			 dcp->piodma ? "present" : "absent");
	else if (dcp->external && method == 201 && get_unaligned_le32(out + 12))
		dev_dbg(dcp->dev, "D201 refused: status %u\n", get_unaligned_le32(out + 12));
	return ret;
}

static int dcp_m4_runtime_event(void *cookie, const char *name, const void *dict, size_t size)
{
	struct dcp_m4_runtime *r = cookie;

	dev_info(r->dcp->dev, "DCP event %.*s: %zu bytes CRC32=%08x\n", 64, name, size,
		 dict ? crc32_le(0, dict, size) : 0);
	return 0;
}

/* The slot of the given-up swap @id, or -1. */
static int dcp_m4_ext_lost_slot(struct dcp_m4_runtime *r, u32 id)
{
	for (unsigned int i = 0; id && i < ARRAY_SIZE(r->ext_lost_swap); i++)
		if (r->ext_lost_swap[i].id == id)
			return i;
	return -1;
}

/* Whether a given-up swap may still put window @i on screen. */
static bool dcp_m4_ext_win_lost(struct dcp_m4_runtime *r, int i)
{
	for (unsigned int j = 0; j < ARRAY_SIZE(r->ext_lost_swap); j++)
		if (r->ext_lost_swap[j].id && r->ext_lost_swap[j].win == i)
			return true;
	return false;
}

/*
 * The late D590 of the given-up swap @id: dropped, and its ID forgotten.
 * Displayed (@shown): a surface it put on screen, which no confirmed swap has
 * retired since, is the one on screen now and the other one free, or, with a
 * newer swap shown before it, on screen with that one. Not displayed: what is
 * on screen stays, and it no longer makes its surfaces uncertain (others
 * given up may still). The hold it is responsible for, if any, ends. False if
 * @id is no given-up swap's.
 */
static bool dcp_m4_ext_late_d590(struct dcp_m4_runtime *r, u32 id, bool shown)
{
	int slot = dcp_m4_ext_lost_slot(r, id);
	bool front = false, held = r->ext_hold_id == id;
	struct dcp_m4_ext_lost lost;

	if (slot < 0)
		return false;
	lost = r->ext_lost_swap[slot];
	r->ext_lost_swap[slot] = (struct dcp_m4_ext_lost){ .win = -1 };
	for (int i = 0; i < 2; i++) {
		if (r->ext_buf_lost[i] != id)
			continue;
		r->ext_buf_lost[i] = 0;
		if (!shown)
			continue;
		r->ext_front = i;
		r->ext_back = i ^ 1;
		r->ext_buf_stale[0] = true;
		r->ext_buf_stale[1] = true;
		front = true;
	}
	/* A window it put on screen is the one shown now, or one of them. */
	if (shown && lost.win >= 0) {
		if (!lost.superseded) {
			for (int j = 0; j < DCP_M4_EXT_WINDOWS; j++)
				r->ext_win[j].on_screen = false;
			r->ext_front = -1;
			r->ext_buf_stale[0] = true;
			r->ext_buf_stale[1] = true;
		}
		r->ext_win[lost.win].on_screen = true;
		front = true;
	}
	if (held) {
		r->ext_hold_id = 0;
		r->ext_hold_probe = false;
	}
	dev_info(r->dcp->dev, "swap %u: its D590 came late: dropped%s%s\n", id,
		 !shown ? "; not displayed, the frame on screen stays" :
		 front ? "; its surface is on screen" : "", held ? "; swaps go on" : "");
	return true;
}

/* The slot of @id among the swaps completed early (0: a free one), or -1. */
static int dcp_m4_ext_early_slot(struct dcp_m4_runtime *r, u32 id)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(r->ext_early); i++)
		if (r->ext_early[i] == id)
			return i;
	return -1;
}

/* The index of the direct swap @id in flight, or -1. */
static int dcp_m4_ext_q_find(struct dcp_m4_runtime *r, u32 id)
{
	for (unsigned int i = 0; id && i < r->ext_qn; i++)
		if (r->ext_q[i].swap.id == id)
			return i;
	return -1;
}

static int dcp_m4_runtime_swap_done(void *cookie, u32 id)
{
	struct dcp_m4_runtime *r = cookie;
	struct dcp_m4_ext_done done = r->ext_done;
	int ret, i, slot;

	if (!r->dcp->external)
		return dcp_m4_swap_record_done(&r->swap, id);
	r->ext_done.valid = false;
	/* Its payload unread: shown, and no earlier swap named. */
	if (!done.valid)
		done = (struct dcp_m4_ext_done){ .shown = true };
	/* The late D590 of a swap given up is not this swap's. */
	if (dcp_m4_ext_late_d590(r, id, done.shown))
		return 0;
	/* Nor is that of a swap a later one's D590 named. */
	slot = dcp_m4_ext_early_slot(r, id);
	if (slot >= 0) {
		r->ext_early[slot] = 0;
		dev_dbg(r->dcp->dev, "swap %u: its D590 after a later one's: dropped\n", id);
		return 0;
	}
	/*
	 * The earlier direct swaps in flight it names complete first, kept on
	 * screen with what is there (macOS completes them as displayed), each
	 * remembered: its own D590 may still come. Unnamed ones stay in flight.
	 */
	for (unsigned int j = 0; j < done.count; j++) {
		i = done.hol[j] == id ? -1 : dcp_m4_ext_q_find(r, done.hol[j]);
		if (i < 0)
			continue;
		slot = dcp_m4_ext_early_slot(r, 0);
		if (slot < 0) {
			dev_warn_ratelimited(r->dcp->dev, "swap %u named done by swap %u, with no slot to remember it: left in flight\n",
					     done.hol[j], id);
			continue;
		}
		r->ext_early[slot] = done.hol[j];
		dcp_m4_ext_async_done(r, i, DCP_M4_EXT_END_KEPT);
	}
	/* A direct swap in flight is done: its flip, or the next one's. */
	i = dcp_m4_ext_q_find(r, id);
	if (i >= 0) {
		dcp_m4_ext_async_done(r, i, done.shown ? DCP_M4_EXT_END_SHOWN :
					    DCP_M4_EXT_END_UNSHOWN);
		return 0;
	}
	ret = dcp_m4_swap_record_done(&r->swap, id);
	/*
	 * The swap going out, before its A407 reply: whether it was shown, and
	 * for a direct one what that means for the screen, now, in D590 order;
	 * only its completion and flip wait for the reply.
	 */
	if (!ret) {
		r->ext_swap_shown = done.shown;
		if (r->ext_swap_async)
			dcp_m4_ext_shown(r, id, r->ext_swap_win, r->ext_qn,
					 done.shown ? DCP_M4_EXT_END_SHOWN :
					 DCP_M4_EXT_END_UNSHOWN);
	}
	return ret;
}

static int dcp_m4_runtime_idle_cache(void *cookie, unsigned int method,
				     const void *input, void *output)
{
	struct dcp_m4_runtime *r = cookie;
	struct iomfb_m4 *dcp = r->dcp;
	unsigned int ms = READ_ONCE(idle_cache_ms);
	bool grant;

	switch (method) {
	case 117: /* set_idle_caching_state_ap(state, size, flag) */
		r->idle_state = get_unaligned_le32(input);
		if (r->idle_state)
			return 0;
		/* Leaving the cache completes every fence first. */
		r->idle_fences = 0;
		/* Retry after a rollback without a flip, backing off. */
		if (ms && !READ_ONCE(dcp->owner->quiescing))
			queue_delayed_work(r->wq, &dcp->idle_cache, msecs_to_jiffies(10 * ms));
		return 0;
	case 578: /* idle_fence_create(state) */
		/*
		 * The host never writes the cache surface, so a fence has no
		 * host-side writer to wait for. Entry (state 1) and each cache
		 * refresh (state 6) ask for one; grant them only for the surface
		 * this host registered.
		 */
		grant = r->idle_registered && !dcp->display_off && !dcp->powered_off;
		r->idle_fences += grant;
		put_unaligned_le32(grant, output);
		return 0;
	case 579: /* idle_fence_complete */
		if (r->idle_fences)
			r->idle_fences--;
		return 0;
	case 580: /* idle_surface_release */
		r->idle_registered = false;
		return 0;
	}
	return -EOPNOTSUPP;
}

/*
 * D003 rt_bandwidth_setup_ap, answered as macOS answers it for a T8132
 * external pipe: the DCP's 8-byte bandwidth request slot as the mailbox,
 * whose length (8) selects the scratch-only protocol, with no doorbell. The
 * firmware then writes its realtime bandwidth requests there. Without a
 * mailbox from the DT (the internal panel) the service is declined.
 */
static int dcp_m4_runtime_bandwidth(void *cookie, void *output)
{
	struct dcp_m4_runtime *r = cookie;
	u8 *out = output;

	if (!r->dcp->bw_mailbox) {
		put_unaligned_le32(1, out + 0x38);	/* status: declined */
		return 0;
	}
	put_unaligned_le64(r->dcp->bw_mailbox, out + 0x08);
	put_unaligned_le32(DCP_M4_BW_MAILBOX_SIZE, out + 0x2c);
	return 0;
}

static const struct dcp_m4_callback_ops dcp_m4_runtime_ops = {
	.call = dcp_m4_runtime_call,
	.property = dcp_m4_runtime_property,
	.memory = dcp_m4_runtime_memory,
	.event = dcp_m4_runtime_event,
	.swap_done = dcp_m4_runtime_swap_done,
	.idle = dcp_m4_runtime_idle_cache,
	.bandwidth = dcp_m4_runtime_bandwidth,
};

static void dcp_m4_runtime_bad_word(struct dcp_m4_runtime *r, u64 word, int error)
{
	u32 offset = (word >> 16) & 0xffff;

	dev_err(r->dcp->dev, "Rejected RPC notification %016llx: %d\n", word, error);
	/* Diagnostic bytes only: unknown control bits never grant packet ownership. */
	if ((word & 3) == 2 && !(word & 0x40) && !(offset % DCP_M4_ALIGNMENT) &&
	    offset <= DCP_M4_STREAM_SIZE - DCP_M4_PACKET_HEADER) {
		dma_rmb();
		dev_err(r->dcp->dev, "Candidate callback headers remote0=%12ph remote1=%12ph\n",
			(u8 *)r->dcp->rpc + 0x60000 + offset,
			(u8 *)r->dcp->rpc + 0x40000 + offset);
	}
}

/*
 * Callbacks whose internal-panel answer does not hold on an external DCP.
 * -ENOENT leaves the callback to the shared dispatcher.
 */
struct dcp_m4_external_nested {
	u16 callback;
	u8 count;
	u16 method[6];	/* the nested calls, in order */
	u8 input_one;	/* bit n: call n takes the u32 input 1 */
	u8 no_output;	/* bit n: call n has no output */
	bool reply_one;	/* the callback answers a u32 1 */
};

/* The internal-panel sequences of dcp_m4_callback_dispatch(), answers logged, not checked. */
static const struct dcp_m4_external_nested dcp_m4_external_nested[] = {
	{ 599, 1, { 412 } },
	{ 100, 1, { 390 } },
	{ 206, 1, { 131 }, .reply_one = true },
	{ 207, 1, { 132 }, .reply_one = true },
	{ 121, 6, { 389, 446, 29, 468, 0, 465 }, .input_one = BIT(3) | BIT(4),
	  .no_output = BIT(0) | BIT(2) | BIT(3), .reply_one = true },
};

/*
 * Callbacks only an external DCP makes, and their shapes: D134, the
 * firmware's is-display-X1 query (no input, a u32 answer), asked once the
 * external video interface is up, and those an external pipe makes around a
 * modeset, power-on and swap. D115, D116, D208, D581, D593, D595 and D598
 * get the answer the desktop 26.x external pipes give (the tiling state of
 * an unpaired pipe, false, or an acknowledgment); the others, which the
 * desktop path does not handle either, are notifications or a false
 * (D124). D593 (a swap aborted) is only acknowledged: a swap whose D590
 * does not come is given up by the host (dcp_m4_ext_swap_give_up()).
 * D202 (unmap) is acknowledged and the mapping retained, see
 * dcp_m4_ext_memdesc(). -1: any size up to 4 KiB.
 */
enum dcp_m4_ext_answer {
	DCP_M4_EXT_ACK,		/* a notification: no output */
	DCP_M4_EXT_ZERO,	/* false, zero or nothing: a zeroed output */
	DCP_M4_EXT_TILING,	/* get_tiling_state of an unpaired pipe: value 0, ret 1 */
	DCP_M4_EXT_MEMDESC,	/* D450/D452/D453 on a D451 record, see below */
};

static const struct dcp_m4_ext_cb {
	u16 method;
	s16 input, output;
	u8 answer;
	const char *name;
} dcp_m4_ext_cbs[] = {
	{ 115, 0x10, 0x8, DCP_M4_EXT_TILING, "get_tiling_state" },
	{ 116, 0x10, 0x4, DCP_M4_EXT_ZERO, "set_tiling_state" },
	{ 124, 0, 0x4, DCP_M4_EXT_ZERO, "detect_fastsim" },
	{ 131, 0x8, 0, DCP_M4_EXT_ACK, "update_ext_pipe_count" },
	{ 133, 0x4, 0, DCP_M4_EXT_ACK, "apply_suppress" },
	{ 208, 0x4, 0, DCP_M4_EXT_ACK, "update_backlight_factor_prop" },
	{ 577, -1, 0, DCP_M4_EXT_ACK, "need_swap_notify" },
	{ 581, 0xc, 0, DCP_M4_EXT_ACK, "swap_complete_head_of_line" },
	{ 584, 0x18, 0, DCP_M4_EXT_ACK, "swap_notify_gated" },
	{ 586, 0x14, 0, DCP_M4_EXT_ACK, "relbufinfo_notify_dispatch" },
	{ 589, 0x8, 0, DCP_M4_EXT_ACK, "update_ap_hdcp_encryption_status" },
	{ 593, 0x4, 0, DCP_M4_EXT_ACK, "abort_swap_ap_gated" },
	{ 595, 0x4, 0, DCP_M4_EXT_ACK, "setSystemConsoleMode" },
	{ 596, 0x4, 0, DCP_M4_EXT_ACK, "setSystemConsoleMode_gated" },
	{ 598, -1, -1, DCP_M4_EXT_ZERO, "D598" },
	{ 450, 0x8, 0x1c, DCP_M4_EXT_MEMDESC, "from_id" },
	{ 452, 0x8, 0x4, DCP_M4_EXT_MEMDESC, "prepare" },
	{ 453, 0x8, 0x4, DCP_M4_EXT_MEMDESC, "complete" },
	{ 585, 0xe4, 0xe0, DCP_M4_EXT_ZERO, "swap_info_notify_dispatch" },
	{ 202, 0x18, 0, DCP_M4_EXT_MEMDESC, "unmap_buf" },
};

/*
 * D450 from_id (a D451 record's addresses again: paddr, DVA, length,
 * ok; its NULL output flags honoured), D452 prepare and D453 complete (0,
 * the buffer being wired already; 14, not found, for an unknown ID).
 */
static void dcp_m4_ext_memdesc(struct dcp_m4_runtime *r, int method, const u8 *in, u8 *out)
{
	struct dcp_m4_memory *memory = r->dcp->memory;
	u32 id = get_unaligned_le32(in);
	const struct dcp_m4_memory_object *object =
		memory && id && id <= memory->count && !memory->object[id - 1].retired ?
		&memory->object[id - 1] : NULL;

	/*
	 * D202 unmap_buf: +0 the buffer, +8 the map ID, +0xc the DVA, +0x14 the
	 * flag. The mapping is retained (never unmapped under the firmware) and
	 * answered again by a later D201.
	 */
	if (method == 202) {
		u32 map_id = get_unaligned_le32(in + 8);
		u8 flag = in[0x14];
		u64 dva = get_unaligned_le64(in + 0xc);
		bool match = object && map_id == (id << 1 | flag) && flag <= 1 &&
			     dva == (flag ? object->dva : object->dva0);

		dev_dbg(r->dcp->dev, "D202 unmap_buf: id %u map %u SID%u IOVA %#llx: %s\n",
			id, map_id, flag ? 16 : 0, dva,
			match ? "kept mapped (retained)" : "no such mapping");
		return;
	}
	if (method == 450) {
		if (object) {
			if (!in[4])
				put_unaligned_le64(object->phys, out);
			if (!in[5])
				put_unaligned_le64(object->dva, out + 8);
			if (!in[6])
				put_unaligned_le64(object->size, out + 16);
			out[24] = 1;
		}
	} else {
		put_unaligned_le32(object ? 0 : 14, out);
	}
	dev_dbg(r->dcp->dev, "D%03d (id %u): %s\n", method, id,
		object ? "known" : "unknown ID");
}

static const struct dcp_m4_ext_cb *dcp_m4_ext_cb(int method)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(dcp_m4_ext_cbs); i++)
		if (dcp_m4_ext_cbs[i].method == method)
			return &dcp_m4_ext_cbs[i];
	return NULL;
}

static bool dcp_m4_ext_size(s16 want, u32 size)
{
	return want < 0 ? size <= SZ_4K : size == (u32)want;
}

static int dcp_m4_external_shape(u32 tag, u32 input, u32 output)
{
	int method = dcp_m4_callback_method(tag);
	const struct dcp_m4_ext_cb *cb;

	if (method == 134)
		return !input && output == 4 ? method : -EMSGSIZE;
	/* D411 mapDeviceMemoryWithIndex, see dcp_m4_external_callback(). */
	if (method == 411)
		return input == 0x10 && output == 0x1c ? method : -EMSGSIZE;
	cb = method < 0 ? NULL : dcp_m4_ext_cb(method);
	if (!cb)
		return -EOPNOTSUPP;
	return dcp_m4_ext_size(cb->input, input) && dcp_m4_ext_size(cb->output, output) ?
	       method : -EMSGSIZE;
}

static int dcp_m4_external_callback(struct dcp_m4_runtime *r, u32 tag, bool remote,
				    const u8 *input, u32 input_size,
				    void *output, u32 output_size)
{
	int method = dcp_m4_callback_method(tag);
	const struct dcp_m4_external_nested *n = NULL;
	u8 one[4], out[4];
	int ret;

	/*
	 * D590 swap_complete_ap_gated: whether the swap was displayed (+0x4),
	 * and the earlier swaps it completes, a count at +0x19 of the IDs at
	 * +0x5, with +0x1d set (and +0x72c, no completion data, clear). Kept for
	 * its swap_done hook, which the shared dispatcher calls next.
	 */
	if (method == 590 && input_size == dcp_m4_pf->done_size) {
		struct dcp_m4_ext_done *d = &r->ext_done;
		u32 count = get_unaligned_le32(input + 0x19);

		*d = (struct dcp_m4_ext_done){ .valid = true, .shown = input[4] != 0 };
		if (input[0x1d] == 1 && !input[0x72c]) {
			if (count > ARRAY_SIZE(d->hol))
				dev_warn_ratelimited(r->dcp->dev, "D590 of swap %u names %u earlier swaps: ignored\n",
						     get_unaligned_le32(input), count);
			else
				for (d->count = 0; d->count < count; d->count++)
					d->hol[d->count] =
						get_unaligned_le32(input + 5 + 4 * d->count);
		}
		return -ENOENT;
	}
	/*
	 * D581 swap_complete_head_of_line: a completion too, whose 0xc bytes the
	 * driver does not decode: logged and acknowledged, its swap left to a
	 * D590 or to its give-up.
	 */
	if (method == 581 && input_size == 0xc) {
		static DEFINE_RATELIMIT_STATE(rs, 5 * HZ, 4);
		u32 id = get_unaligned_le32(input);

		if (__ratelimit(&rs)) {
			dev_warn(r->dcp->dev, "D581 swap_complete_head_of_line, word 0 %u (%s): left to a D590 or its give-up\n",
				 id, dcp_m4_ext_q_find(r, id) >= 0 ? "a swap in flight" :
				 "no swap in flight");
			print_hex_dump_debug("D581: ", DUMP_PREFIX_OFFSET, 16, 1, input,
					     input_size, false);
		}
		return -ENOENT;
	}
	/*
	 * D572 powerUpDART(on): the firmware switches its display DART
	 * on or off, which the host does nothing for but must not touch the
	 * DART while it is off. Answered with success, as the internal DCP's.
	 */
	if (method == 572 && input_size == 4 && output_size == 4) {
		/* A bool in a 4-byte slot: its padding is not cleared. */
		bool on = input[0] & 1;

		/*
		 * After a system sleep a D572(1) confirms power only as a callback
		 * nested in a host call of our own sent after resume: such a
		 * callback (remote 0, on the command callback ring) is only ever
		 * sent while the firmware handles that call, and its reply comes
		 * after it, whatever the transport held. An async one (remote 1),
		 * one nested in a call sent earlier, or during sleep (an odd
		 * epoch) is acknowledged as any other, but confirms nothing: the
		 * DART counts as off still, and its state is not read.
		 */
		if (on) {
			int now = atomic_read(&r->dcp->ext_pm_epoch);
			bool fresh = !remote && r->ext_local_calls && r->ext_call_epoch == now &&
				     r->cb_pm == now && !(now & 1);

			if (now && !fresh) {
				dev_dbg(r->dcp->dev, "D572 powerUpDART(1) after sleep %s: acknowledged, no power confirmed\n",
					remote ? "as an async callback" :
					!r->ext_local_calls ? "outside any call of ours" :
					"in a call sent before resume");
				return 0;
			}
		}
		/*
		 * For a replay under way, a D572(1) taken (after a suspend
		 * only a nested one, above) confirms the DART, a D572(0) undoes it.
		 */
		if (r->ext_restore_pending)
			WRITE_ONCE(r->ext_restore_dart, on);
		WRITE_ONCE(r->dcp->ext_dart_down, !on);
		/* A sleep's preparation may wait for it (iomfb_m4_ext_sleep_prepare()). */
		if (!on)
			wake_up_all(&r->dcp->ext_settle_wq);
		if (on)
			r->dcp->ext_dart_since_sleep = false;
		dev_dbg(r->dcp->dev, "D572 powerUpDART(%u)\n", on);
		/* Did the display DART keep its state? Reads only. */
		if (on) {
			struct device_node *node =
				of_get_child_by_name(r->dcp->dev->of_node, "piodma");

			int state = apple_dart_dcp_dispext_state(node);

			/* Lost: every later D201 is refused, cached mappings too. */
			if (state == -EIO && !READ_ONCE(r->dcp->ext_dart_lost)) {
				WRITE_ONCE(r->dcp->ext_dart_lost, true);
				dev_err(r->dcp->dev, "display DART state lost after its power-up (see the DART's log): SID0 not used again\n");
			}
			dev_dbg(r->dcp->dev, "display DART state check: %d\n", state);
			of_node_put(node);
		}
		return 0;
	}
	/* The external-only callbacks above; the output is zeroed already. */
	{
		const struct dcp_m4_ext_cb *cb = dcp_m4_ext_cb(method);

		if (cb && dcp_m4_ext_size(cb->input, input_size) &&
		    dcp_m4_ext_size(cb->output, output_size)) {
			if (cb->answer == DCP_M4_EXT_TILING) {
				put_unaligned_le32(0, output);
				put_unaligned_le32(1, (u8 *)output + 4);
			}
			if (cb->answer == DCP_M4_EXT_MEMDESC)
				dcp_m4_ext_memdesc(r, method, input, output);
			dev_dbg(r->dcp->dev, "D%03d (%s, %u/%u): %s\n",
				method, cb->name, input_size, output_size,
				cb->answer == DCP_M4_EXT_ACK ? "acknowledged" :
				cb->answer == DCP_M4_EXT_MEMDESC ? "answered from its D451 record" :
				cb->answer == DCP_M4_EXT_TILING ? "answered as an unpaired pipe" :
				"answered zero/false");
			return 0;
		}
	}
	/*
	 * D134, is the sink a display X1: 0, the answer for an external
	 * non-Apple sink (1 only for two Apple display products), as the
	 * desktop 26.x and J700 external pipes already answer it.
	 */
	if (method == 134) {
		if (output_size != 4)
			return -EMSGSIZE;
		put_unaligned_le32(0, output);
		return 0;
	}
	/*
	 * D411 mapDeviceMemoryWithIndex, which the firmware asks for its register
	 * windows (the one holding the bandwidth mailbox among them): "VORP", the
	 * display register index and flags in; the window's DVA, address and
	 * length, then a u32 status out. Only a disp-N window the DT describes is
	 * given. As on the desktop 26.x firmware, the DVA is the physical address:
	 * the firmware reaches its register windows on its DART-bypass stream.
	 * Any other index gets status 1.
	 */
	if (method == 411) {
		struct resource *res = NULL;
		u8 *o = output;
		char name[16];
		u32 index;

		if (input_size != 0x10 || output_size != 0x1c)
			return -EMSGSIZE;
		index = get_unaligned_le32(input + 4);
		if (!memcmp(input, "VORP", 4) && index < MAX_DISP_REGISTERS) {
			snprintf(name, sizeof(name), "disp-%u", index);
			res = platform_get_resource_byname(to_platform_device(r->dcp->dev),
							   IORESOURCE_MEM, name);
		}
		memset(o, 0, 0x1c);
		if (!res) {
			put_unaligned_le32(1, o + 24);
			dev_warn(r->dcp->dev, "D411: register index %u is not described: refused\n",
				 index);
			return 0;
		}
		put_unaligned_le64(res->start, o);
		put_unaligned_le64(res->start, o + 8);
		put_unaligned_le64(resource_size(res), o + 16);
		dev_dbg(r->dcp->dev, "D411: register index %u (flags %#x) mapped: %pR\n", index,
			get_unaligned_le32(input + 8), res);
		return 0;
	}
	/*
	 * D575, the hotplug report: the connection state and a 76-byte
	 * tiled-display record (its null flag at 84) in, a 76-byte record out.
	 * An external pipe with no peer answers it as the desktop 26.x external
	 * pipes do: zeros, and 0xffffffff at 0x47 while connected.
	 */
	if (method == 575) {
		u64 connected;

		if (input_size != 88 || output_size != 76)
			return -EMSGSIZE;
		connected = get_unaligned_le64(input);
		dev_dbg(r->dcp->dev, "D575 hotplug: connected %llu, record %s\n",
			connected, (input[84] & 1) ? "absent" : "present");
		memset(output, 0, 76);
		if (connected)
			put_unaligned_le32(0xffffffff, (u8 *)output + 0x47);
		r->ext_connected = !!connected;
		/* A sink that went takes a new power-on and mode. */
		if (!connected) {
			atomic_inc(&r->ext_link_gen);
			/* The direct swaps in flight are given up now. */
			if (r->ext_qn)
				mod_delayed_work(r->wq, &r->ext_async_work, 0);
			WRITE_ONCE(r->ext_powered, false);
			WRITE_ONCE(r->ext_cur_timing, 0);
			/* The next sink's timing catalog is its own. */
			dcp_m4_ext_sink_gone(r->dcp, false);
			/* A property the old sink began (D127) publishes nothing. */
			if (r->ext_prop_open)
				r->ext_prop_whole = false;
		}
		/* The connector's state. */
		if (READ_ONCE(r->dcp->ext_sink) != !!connected) {
			WRITE_ONCE(r->dcp->ext_sink, !!connected);
			t8132_ext_drm_hotplug(r->dcp->dev);
		}
		/* A replay waiting for the link (iomfb_m4_ext_relink_wait()). */
		if (connected)
			wake_up_all(&r->dcp->ext_settle_wq);
		return 0;
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(dcp_m4_external_nested); i++)
		if (dcp_m4_external_nested[i].callback == method)
			n = &dcp_m4_external_nested[i];
	if (!n)
		return -ENOENT;
	if (output_size != (n->reply_one ? 4 : 0))
		return -EMSGSIZE;
	put_unaligned_le32(1, one);
	for (unsigned int i = 0; i < n->count; i++) {
		bool in = n->input_one & BIT(i), has_out = !(n->no_output & BIT(i));

		memset(out, 0, sizeof(out));
		ret = dcp_m4_runtime_call(r, n->method[i], remote, in ? one : NULL, in ? 4 : 0,
					  has_out ? out : NULL, has_out ? 4 : 0);
		if (ret)
			return ret;
		if (has_out)
			dev_dbg(r->dcp->dev, "D%03d: A%03u returned %u\n", method, n->method[i],
				get_unaligned_le32(out));
	}
	if (n->reply_one)
		put_unaligned_le32(1, output);
	return 0;
}

static int dcp_m4_runtime_callback(struct dcp_m4_runtime *r, u64 word)
{
	struct iomfb_m4 *dcp = r->dcp;
	struct dcp_m4_packet packet;
	u8 *input, *snapshot;
	u64 ack;
	const char *stage = "offset";
	int ret, offset, gen, pm;

	dma_rmb();
	ret = dcp_m4_callback_begin(&dcp->transaction, dcp->rpc, DCP_M4_RPC_SIZE, word, &packet);
	if (ret) {
		dcp_m4_runtime_bad_word(r, word, ret);
		return ret;
	}
	offset = dcp_m4_packet_offset(&packet, true);
	if (offset < 0) {
		ret = offset;
		goto failed;
	}
	input = (u8 *)dcp->rpc + offset + DCP_M4_PACKET_HEADER;
	ret = dcp_m4_callback_shape(packet.tag, packet.input_size, packet.output_size);
	if (ret == -EOPNOTSUPP && dcp->external)
		ret = dcp_m4_external_shape(packet.tag, packet.input_size, packet.output_size);
	if (ret < 0) {
		if (dcp->external)
			print_hex_dump(KERN_INFO, "dcpext callback: ", DUMP_PREFIX_OFFSET, 16, 1,
				       input, min(packet.input_size, 256U), false);
		goto failed;
	}
	/* Each nested callback owns its snapshot; validation and property/string
	 * consumers must not fetch shared firmware bytes a second time. Exact
	 * callback shapes bound this allocation to 4168 bytes.
	 */
	snapshot = kmemdup(input, packet.input_size, GFP_KERNEL);
	if (!snapshot && packet.input_size) {
		ret = -ENOMEM;
		goto failed;
	}
	r->callbacks++;
	dev_dbg(dcp->dev, "DCP callback D%03d %u/%u remote=%u flag9=%u\n",
		 dcp_m4_callback_method(packet.tag), packet.input_size, packet.output_size,
		 packet.remote, packet.callback_flag);
	if (packet.callback_flag && packet.input_size > 80 &&
	    packet.tag == get_unaligned_le32("D006"))
		dev_dbg(dcp->dev, "Flagged D006 header=%12ph null-byte=%02x; ACK route=342\n",
			 (u8 *)dcp->rpc + offset, input[80]);
	stage = "dispatch";
	/* This callback's word's sink generation, across nested calls. */
	gen = r->cb_gen;
	pm = r->cb_pm;
	r->cb_gen = r->word_gen;
	r->cb_pm = r->word_pm;
	ret = -ENOENT;
	if (dcp->external)
		ret = dcp_m4_external_callback(r, packet.tag, packet.remote, snapshot,
					       packet.input_size, input + packet.input_size,
					       packet.output_size);
	if (ret == -ENOENT)
		ret = dcp_m4_callback_dispatch(&dcp_m4_runtime_ops, r, packet.tag,
					       packet.remote, snapshot, packet.input_size,
					       input + packet.input_size, packet.output_size);
	r->cb_gen = gen;
	r->cb_pm = pm;
	kfree(snapshot);
	if (ret)
		goto failed;
	stage = "finish";
	ret = dcp_m4_callback_end(&dcp->transaction, dcp->rpc, DCP_M4_RPC_SIZE, &ack);
	if (ret)
		goto failed;
	stage = "ack";
	ret = dcp_m4_runtime_send(r, ack);
	if (!ret)
		return 0;
failed:
	dev_err(dcp->dev,
		"DCP callback D%03d tag=%08x %u/%u remote=%u flag9=%u %s failed: %d\n",
		dcp_m4_callback_method(packet.tag), packet.tag, packet.input_size,
		packet.output_size, packet.remote, packet.callback_flag, stage, ret);
	return ret;
}

static int dcp_m4_runtime_call_stream(struct dcp_m4_runtime *r, unsigned int method,
				      u8 stream, bool remote, const void *input,
				      u32 input_size, void *output, u32 output_size)
{
	struct iomfb_m4 *dcp = r->dcp;
	unsigned long previous = r->deadline;
	int previous_epoch = r->ext_call_epoch;
	struct dcp_m4_packet message;
	u64 word;
	u32 tag;
	int ret;

	if (method > 999)
		return -EINVAL;
	/* The epoch our own call goes out in, for its nested callbacks. */
	if (!remote) {
		r->ext_local_calls++;
		r->ext_call_epoch = atomic_read(&dcp->ext_pm_epoch);
	}
	if (!r->deadline)
		r->deadline = jiffies + msecs_to_jiffies(5000);
	if (time_after_eq(jiffies, r->deadline)) {
		ret = -ETIMEDOUT;
		goto finished;
	}
	tag = 'A' | (('0' + method / 100) << 8) |
		(('0' + method / 10 % 10) << 16) | (('0' + method % 10) << 24);
	ret = dcp_m4_call_begin_stream(&dcp->transaction, dcp->rpc, DCP_M4_RPC_SIZE,
				       tag, stream, remote, input, input_size, output_size,
				       &word);
	if (!ret)
		ret = dcp_m4_runtime_send(r, word);
	while (!ret) {
		ret = dcp_m4_runtime_next(r, &word, true);
		if (ret)
			break;
		dma_rmb();
		ret = dcp_m4_transaction_decode(&dcp->transaction, word, &message);
		if (ret) {
			dcp_m4_runtime_bad_word(r, word, ret);
			break;
		}
		if (message.reply) {
			ret = dcp_m4_call_reply(&dcp->transaction, dcp->rpc, DCP_M4_RPC_SIZE,
					word, output, output_size);
			break;
		}
		ret = dcp_m4_runtime_callback(r, word);
	}
finished:
	r->deadline = previous;
	if (!remote) {
		r->ext_local_calls--;
		r->ext_call_epoch = previous_epoch;
	}
	if (ret) {
		dcp_m4_transaction_fail(&dcp->transaction, ret);
		dcp_m4_failed(dcp, ret);
		dev_err(dcp->dev, "DCP A%03u failed: %d; retained session\n", method, ret);
	}
	return ret;
}

static int dcp_m4_runtime_call(void *cookie, unsigned int method, bool remote,
			       const void *input, u32 input_size, void *output, u32 output_size)
{
	return dcp_m4_runtime_call_stream(cookie, method, 0, remote, input, input_size,
					  output, output_size);
}

/* One bounded drain; a positive return asks the idle worker to reschedule. */
static int dcp_m4_runtime_drain(struct dcp_m4_runtime *r, unsigned int budget)
{
	u64 word;
	int ret;

	r->deadline = jiffies + msecs_to_jiffies(5000);
	for (unsigned int handled = 0; handled < budget; handled++) {
		ret = dcp_m4_runtime_next(r, &word, false);
		if (ret == -EAGAIN) {
			ret = 0;
			goto out;
		}
		if (ret)
			goto out;
		if (time_after_eq(jiffies, r->deadline)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		ret = dcp_m4_runtime_callback(r, word);
		if (ret)
			goto out;
	}
	ret = 1;
out:
	r->deadline = 0;
	return ret;
}

static void dcp_m4_runtime_idle(struct work_struct *work)
{
	struct dcp_m4_runtime *r = container_of(work, struct dcp_m4_runtime, idle);
	struct iomfb_m4 *dcp = r->dcp;
	int ret;

	mutex_lock(&dcp->lock);
	/*
	 * Powered down (iomfb_m4_power_gate()): nothing arrives then, and
	 * nothing may reach the stopped firmware or its powered-down DARTs.
	 */
	if (READ_ONCE(dcp->gated)) {
		mutex_unlock(&dcp->lock);
		return;
	}
	ret = dcp_m4_runtime_drain(r, 32);
	if (ret < 0) {
		dcp_m4_transaction_fail(&dcp->transaction, ret);
		dcp_m4_failed(dcp, ret);
	}
	mutex_unlock(&dcp->lock);
	if (ret > 0)
		queue_work(r->wq, &r->idle);
}

/*
 * The next word while a swap waits for its D590, in 50 ms
 * slices, so that a link that goes (a D575(0) handled meanwhile, or a
 * released route: the link generation moves on) ends the wait at once with
 * -ENOLINK. -ETIMEDOUT at the swap's deadline, as dcp_m4_runtime_next().
 */
static int dcp_m4_ext_swap_wait(struct dcp_m4_runtime *r, int link_gen, u64 *word)
{
	unsigned long deadline = r->deadline;
	int ret;

	for (;;) {
		if (!r->ext_connected || atomic_read(&r->ext_link_gen) != link_gen)
			return -ENOLINK;
		if (time_before(jiffies + msecs_to_jiffies(50), deadline))
			r->deadline = jiffies + msecs_to_jiffies(50);
		ret = dcp_m4_runtime_next(r, word, true);
		r->deadline = deadline;
		if (ret != -ETIMEDOUT || time_after_eq(jiffies, deadline))
			return ret;
	}
}

/*
 * A swap whose A407 reply came, but not its D590, in time
 * (-ETIMEDOUT) or before its link went (-ENOLINK), is given up without
 * ending the session. Nothing of it is outstanding on the RPC buffer then:
 * no call or callback is open, the firmware has answered both A406 and
 * A407, and the transaction has no error. What may still come is its
 * D590, a callback of its own matched by the swap ID, which is why the ID
 * is kept in a free slot of ext_lost_swap until then: that D590 is dropped
 * (dcp_m4_ext_late_d590()), and A406 giving the ID again holds swaps
 * instead of reusing it. Its surface, if any, may still go on screen: it
 * stays unavailable to frames until that D590 or a confirmed later swap.
 * After one given up on a live link, swaps are held (iomfb_m4_ext_present())
 * until its D590 or a confirmed swap after a DRM enable, so that a stuck
 * pipe does not cost every frame 5 s. With every slot taken the swap is
 * not given up. @swap is its state: r->swap, or a direct swap's in flight,
 * @win its window (-1: none) and @superseded whether a newer swap was shown
 * before it. True if the swap was given up so; anything else is the session's
 * failure as before.
 */
static bool dcp_m4_ext_swap_give_up(struct dcp_m4_runtime *r, struct dcp_m4_swap_state *swap,
				    int win, bool superseded, int error)
{
	struct iomfb_m4 *dcp = r->dcp;
	struct dcp_m4_transaction *t = &dcp->transaction;
	int slot;

	if (!dcp->external || (error != -ETIMEDOUT && error != -ENOLINK) ||
	    READ_ONCE(dcp->error) || t->error || t->calls || t->callbacks || swap->error ||
	    !swap->id || !swap->replied || swap->completed || swap->emitted)
		return false;
	/* A406 never gives an unresolved ID (see dcp_m4_runtime_swap_full()). */
	if (WARN_ON_ONCE(dcp_m4_ext_lost_slot(r, swap->id) >= 0))
		return false;
	slot = -1;
	for (unsigned int i = 0; slot < 0 && i < ARRAY_SIZE(r->ext_lost_swap); i++)
		if (!r->ext_lost_swap[i].id)
			slot = i;
	if (slot < 0) {
		dev_warn(dcp->dev, "swap %u: %d given-up swaps still without their D590: not given up\n",
			 swap->id, (int)ARRAY_SIZE(r->ext_lost_swap));
		return false;
	}
	r->ext_lost_swap[slot] = (struct dcp_m4_ext_lost){
		.id = swap->id, .win = win, .superseded = superseded,
	};
	swap->emitted = true;
	if (r->ext_swap_surface >= 0)
		r->ext_buf_lost[r->ext_swap_surface] = swap->id;
	if (error == -ETIMEDOUT) {
		r->ext_hold_id = swap->id;
		r->ext_hold_probe = false;
	}
	dev_info(dcp->dev, "swap %u (surface %d): %s before its D590: given up, session kept%s\n",
		 swap->id, r->ext_swap_surface,
		 error == -ETIMEDOUT ? "timed out" : "the link went",
		 r->ext_hold_id == swap->id ?
		 "; swaps held until its D590 or a confirmed swap after a DRM enable" : "");
	return true;
}

/*
 * Serialized owner only; both the A407 reply and its D590 fence must complete.
 * primary/frame/upper/keep as dcp_m4_swap_encode_full(); a nonzero intent
 * retargets the swap as dcp_m4_swap_set_intent() describes.
 */
static int dcp_m4_runtime_swap_full(struct dcp_m4_runtime *r, bool primary,
		const struct dcp_m4_frame *frame, const struct dcp_m4_upper *upper, u32 keep,
		bool brightness_change, u32 nits, u32 intent, u32 surface_id)
{
	struct iomfb_m4 *dcp = r->dcp;
	u8 start[16] = {0}, started[8], submitted[12];
	/* An external output is the size of its mode, any surface its own. */
	bool ext = dcp->external;
	int (*check)(const struct dcp_m4_frame *frame) =
		ext ? dcp_m4_frame_check_ext : dcp_m4_frame_check;
	u32 width = ext ? r->ext_width : DCP_M4_PANEL_WIDTH;
	u32 height = ext ? r->ext_height : DCP_M4_PANEL_HEIGHT;
	/* The link this swap is for, and whether only its D590 was missing */
	int link_gen = ext ? atomic_read(&r->ext_link_gen) : 0;
	bool d590_missing = false;
	u8 *input;
	u32 id;
	u64 word;
	int ret;

	if (((primary && frame) || (upper && upper->frame)) &&
	    (ext ? !width || !height :
	     !r->power_done || !r->background_done || !dcp->scanout))
		return -EINVAL;
	if ((primary && frame && (check(frame) || frame->width != width ||
				  frame->height != height)) ||
	    (upper && dcp_m4_upper_check_on(upper, width, height)))
		return -EINVAL;
	/*
	 * No swap, surfaceless ones included, once the display DART is retired
	 * (either frontend failed, sticky): flags only.
	 */
	if (ext && dcp->piodma && apple_dart_dcp_dispext_stream_ok(&dcp->piodma->dev, 0) == -EIO) {
		dev_warn_ratelimited(dcp->dev, "the display DART is retired: no swap\n");
		return -EIO;
	}
	/* No swap starts after an orphaned start. */
	if (ext && r->ext_orphan_start) {
		dev_warn_ratelimited(dcp->dev, "swap %u started and never submitted: no swap starts until reboot\n",
				     r->ext_orphan_start);
		return -EBUSY;
	}
	input = kzalloc(DCP_M4_SWAP_SIZE, GFP_KERNEL);
	if (!input) {
		dcp_m4_swap_fail(&r->swap, -ENOMEM);
		dcp_m4_failed(dcp, -ENOMEM);
		return -ENOMEM;
	}
	r->deadline = jiffies + msecs_to_jiffies(5000);
	ret = dcp_m4_runtime_call(r, 406, false, start, sizeof(start), started, sizeof(started));
	if (!ret)
		ret = dcp_m4_swap_start_reply(started, sizeof(started), &id);
	/*
	 * An ID whose given-up (or early-completed) swap's D590 may still come
	 * is not reused: the two completions could not be told apart, as a D590
	 * names the swap by its ID alone. A407 is not sent, and the start is left orphaned: no
	 * host call ends or discards a started swap, and completing it (with a
	 * surfaceless A407) would leave two completions under one ID that a
	 * single D590 could belong to either of, while the firmware may swallow
	 * a swap without one. So no swap starts after it
	 * (dcp_m4_runtime_swap_full()) until reboot; the old swap's late D590
	 * is still dropped, but ends no hold.
	 */
	if (!ret && ext &&
	    (dcp_m4_ext_lost_slot(r, id) >= 0 || dcp_m4_ext_early_slot(r, id) >= 0)) {
		dev_warn(dcp->dev, "A406 gave swap %u again, whose D590 may still come: not submitted, the start left orphaned; no swap starts until reboot\n",
			 id);
		r->ext_orphan_start = id;
		r->ext_hold_id = 0;
		r->ext_hold_probe = false;
		r->deadline = 0;
		kfree(input);
		return -EBUSY;
	}
	if (!ret)
		ret = dcp_m4_swap_begin(&r->swap, id);
	if (!ret)
		ret = dcp_m4_swap_encode_on(input, DCP_M4_SWAP_SIZE, id, primary, frame, upper,
					    keep, brightness_change, nits, check, width, height);
	if (!ret && intent)
		dcp_m4_swap_set_intent(input, intent, surface_id);
	/*
	 * A present on a 120 Hz panel carries the deadline, or the DCP completes
	 * it only at the second vblank. Not on a retargeted (registering) swap.
	 */
	if (!ret && !ext && !intent) {
		bool promo = dcp_refresh_is_promotion(r->timing.refresh_16_16);
		bool shown = (primary && frame) || upper;
		u64 deadline = dcp_desktop_present_deadline(shown, promo);

		dcp_m4_swap_set_deadline(input, deadline);
	}
	if (!ret)
		ret = dcp_m4_runtime_call(r, 407, false, input, DCP_M4_SWAP_SIZE,
					 submitted, sizeof(submitted));
	if (!ret)
		ret = dcp_m4_swap_record_reply(&r->swap, submitted, sizeof(submitted));
	/*
	 * A direct swap does not wait for its D590: it joins the swaps in flight
	 * and is tracked there (r->swap may begin the next one) until its D590,
	 * or a later swap's, completes it (dcp_m4_runtime_swap_done()), or its
	 * work gives it up at the deadline or when the link goes
	 * (dcp_m4_ext_async_work()). Nothing of it is outstanding on the RPC
	 * buffer: A406 and A407 are answered. Its flip goes at once with none
	 * older in flight and a depth over 1 (the previous frame's buffer may still
	 * be on screen then), else with the completion of the older one, or its
	 * own.
	 */
	if (!ret && ext && r->ext_swap_async) {
		struct dcp_m4_ext_inflight *e =
			&r->ext_q[min_t(unsigned int, r->ext_qn, DCP_M4_EXT_INFLIGHT - 1)];
		struct dcp_m4_swap_state handed;

		/* iomfb_m4_ext_present_direct() left room for it. */
		ret = WARN_ON_ONCE(r->ext_qn >= DCP_M4_EXT_INFLIGHT) ? -EBUSY :
		      dcp_m4_swap_hand_off(&r->swap, &handed);
		if (!ret) {
			r->deadline = 0;
			kfree(input);
			*e = (struct dcp_m4_ext_inflight){
				.swap = handed,
				.win = r->ext_swap_win,
				.gen = link_gen,
				.deadline = jiffies + msecs_to_jiffies(5000),
				.flip_held = r->ext_qn || r->ext_swap_async < 2,
			};
			r->ext_swap_now = !e->flip_held;
			r->ext_qn++;
			if (e->swap.completed) {
				dcp_m4_ext_async_done(r, r->ext_qn - 1, DCP_M4_EXT_END_TAKEN);
			} else {
				queue_delayed_work(r->wq, &r->ext_async_work,
						   msecs_to_jiffies(5000));
				if (!r->ext_connected || atomic_read(&r->ext_link_gen) != link_gen)
					mod_delayed_work(r->wq, &r->ext_async_work, 0);
			}
			return 0;
		}
	}
	while (!ret && !r->swap.completed) {
		if (ext) {
			ret = dcp_m4_ext_swap_wait(r, link_gen, &word);
			d590_missing = ret == -ETIMEDOUT || ret == -ENOLINK;
		} else {
			ret = dcp_m4_runtime_next(r, &word, true);
		}
		if (!ret)
			ret = dcp_m4_runtime_callback(r, word);
	}
	if (!ret)
		ret = dcp_m4_runtime_live(r);
	if (!ret && dcp_m4_swap_take_completion(&r->swap) != 1)
		ret = -EPROTO;
	r->deadline = 0;
	/* Only the copied host input is freed; the firmware-visible RPC stays pinned. */
	kfree(input);
	/* Only its D590 missing: given up, the session kept. */
	if (ret && d590_missing && dcp_m4_ext_swap_give_up(r, &r->swap, -1, false, ret))
		return ret;
	if (ret) {
		dcp_m4_swap_fail(&r->swap, ret);
		dcp_m4_transaction_fail(&dcp->transaction, ret);
		dcp_m4_failed(dcp, ret);
	} else {
		dev_dbg(dcp->dev, "Display swap %u complete: A407 reply and D590\n", r->swap.id);
	}
	return ret;
}

static int dcp_m4_runtime_swap_brightness(struct dcp_m4_runtime *r,
		const struct dcp_m4_frame *frame, bool brightness_change, u32 nits)
{
	return dcp_m4_runtime_swap_full(r, true, frame, NULL, 0, brightness_change, nits, 0, 0);
}

static int dcp_m4_runtime_swap(struct dcp_m4_runtime *r, const struct dcp_m4_frame *frame)
{
	return dcp_m4_runtime_swap_brightness(r, frame, false, 0);
}

static int dcp_m4_runtime_background(struct dcp_m4_runtime *r)
{
	int ret;

	if (!r->done || r->background_started || READ_ONCE(r->dcp->error))
		return -EBUSY;
	r->background_started = true;
	ret = dcp_m4_runtime_swap(r, NULL);
	if (!ret) {
		r->background_id = r->swap.id;
		smp_store_release(&r->background_done, true);
	}
	return ret;
}

/* D101 must report the inherited framebuffer stride, not an invented one. */
static int dcp_m4_boot_framebuffer_check(void)
{
	struct device_node *np;
	u32 width, height, stride;
	int ret = -ENODEV, found = 0;

	for_each_compatible_node(np, NULL, "simple-framebuffer") {
		if (!of_device_is_available(np))
			continue;
		found++;
		/* The inherited console may exclude the panel's notch rows. */
		if (of_property_read_u32(np, "width", &width) ||
		    of_property_read_u32(np, "height", &height) ||
		    of_property_read_u32(np, "stride", &stride) ||
		    width != dcp_m4_pf->panel_width || stride != dcp_m4_pf->boot_stride ||
		    (height != dcp_m4_pf->panel_height &&
		     height != dcp_m4_pf->panel_height - dcp_m4_pf->notch_rows)) {
			of_node_put(np);
			return -EINVAL;
		}
		ret = 0;
	}
	return found > 1 ? -EINVAL : ret;
}

static int dcp_m4_startup(struct iomfb_m4 *dcp)
{
	struct device *dev = dcp->dev;
	struct dcp_m4_runtime *r;
	unsigned long flags;
	u8 output[4];
	int ret, slot;

	ret = -EBUSY;
	if (!READ_ONCE(dcp->ready) || READ_ONCE(dcp->error) || dcp->startup_started)
		goto out;
	dcp->startup_started = true;
	ret = dcp->external ? 0 : dcp_m4_boot_framebuffer_check();
	if (ret)
		goto failed;
	ret = dcp->external ? 0 : dcp_m4_create_piodma(dcp);
	if (ret)
		goto failed;
	r = kzalloc(sizeof(*r), GFP_KERNEL);
	dcp->memory = kzalloc(sizeof(*dcp->memory), GFP_KERNEL);
	if (!r || !dcp->memory) {
		ret = -ENOMEM;
		goto failed;
	}
	/* An external firmware's D451 requests are not qualified as such. */
	dcp->memory->relaxed = dcp->external;
	dcp->memory->page_align = !dcp->external && dcp_m4_pf->alloc_page_align;
	/* An external display DART counts as off until a D572(1). */
	WRITE_ONCE(dcp->ext_dart_down, dcp->external);
	r->properties = kzalloc(sizeof(*r->properties), GFP_KERNEL);
	if (!r->properties) {
		ret = -ENOMEM;
		goto failed;
	}
	r->dcp = dcp;
	/* No surface on screen or in flight yet. */
	r->ext_swap_surface = -1;
	r->ext_front = -1;
	/* No direct swap going out. */
	r->ext_swap_win = -1;
	INIT_DELAYED_WORK(&r->ext_async_work, dcp_m4_ext_async_work);
	init_completion(&r->received);
	INIT_WORK(&r->idle, dcp_m4_runtime_idle);
	/*
	 * An external DCP's callbacks (D572, D201, D451 among them) are
	 * not handled across system sleep: its queue is frozen with the tasks,
	 * before the DARTs suspend and until after they resume.
	 */
	r->wq = alloc_ordered_workqueue("iomfb-v26-6",
					WQ_MEM_RECLAIM | (dcp->external ? WQ_FREEZABLE : 0));
	if (!r->wq) {
		ret = -ENOMEM;
		goto failed;
	}
	spin_lock_irqsave(&dcp->state_lock, flags);
	smp_store_release(&dcp->runtime, r);
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	/* An external DCP's callbacks may use streams 0..3. */
	dcp->transaction.external = dcp->external;
	/* Fresh session only: A401, then the first client open A457. */
	ret = dcp_m4_runtime_call(r, 401, false, NULL, 0, output, sizeof(output));
	if (!ret && get_unaligned_le32(output) != 1)
		ret = -EPROTO;
	if (!ret)
		ret = dcp_m4_runtime_call(r, dcp_m4_pf->open_method, false, NULL, 0, NULL, 0);
	/* A reply may be followed by property callbacks queued for this owner. */
	if (!ret)
		ret = dcp_m4_runtime_drain(r, 256);
	if (ret > 0)
		ret = -E2BIG;
	if (ret)
		goto failed;
	if (dcp->external) {
		ret = dcp_m4_runtime_health(r);
		if (ret)
			goto failed;
		/* PIODMA now if possible, else on demand. */
		dcp_m4_ext_piodma(dcp, "startup");
		/* Pairs with the acquires in iomfb_m4_external_ready() and the ext_ calls. */
		smp_store_release(&r->done, true);
		dev_info(dev, "IOMFB startup complete: %u callbacks; external\n", r->callbacks);
		goto out;
	}
	slot = dcp_m4_raw_slot(r->properties, 0, "PreferredTimingElements", false);
	ret = slot < 0 ? slot : dcp_m4_parse_timing(r->properties->raw[slot].data,
					       r->properties->raw[slot].size, &r->timing);
	if (ret)
		goto failed;
	ret = dcp_m4_runtime_health(r);
	if (ret)
		goto failed;
	smp_store_release(&r->done, true);
	dev_info(dev, "IOMFB startup complete: %u callbacks, %ux%u at %u kHz; no modeset\n",
		 r->callbacks, r->timing.hdisplay, r->timing.vdisplay, r->timing.clock_khz);
	ret = 0;
	goto out;
failed:
	dcp_m4_failed(dcp, ret);
	dev_err(dev, "IOMFB startup failed: %d; retained until guarded reboot\n", ret);
out:
	return ret;
}
static int dcp_m4_prepare_scanout(struct iomfb_m4 *dcp)
{
	struct dcp_m4_runtime *r = dcp->runtime;
	dma_addr_t dva;
	int ret;

	if (r->timing.hdisplay != DCP_M4_PANEL_WIDTH ||
	    r->timing.vdisplay != DCP_M4_PANEL_HEIGHT)
		return -EINVAL;
	ret = dcp_m4_dart_check(dcp->scanout);
	if (ret)
		return ret;
	r->scanout_started = true;
	r->scanout_frame = (struct dcp_m4_frame) {
		.width = DCP_M4_PANEL_WIDTH, .height = DCP_M4_PANEL_HEIGHT,
		.stride = DCP_M4_PANEL_WIDTH * 4,
		.backing_size = (u64)DCP_M4_PANEL_WIDTH * 4 * DCP_M4_PANEL_HEIGHT,
		.format = DCP_M4_XRGB8888,
	};
	r->scanout_cpu = apple_dart_dcp_scanout_alloc(dcp->scanout,
						   r->scanout_frame.backing_size, &dva);
	if (IS_ERR(r->scanout_cpu))
		return PTR_ERR(r->scanout_cpu);
	r->scanout_frame.dva = dva;
	ret = dcp_m4_frame_check(&r->scanout_frame);
	if (ret)
		return ret;
	/* Prepare backing while simpledrm still owns the inherited surface. */
	memset(r->scanout_cpu, 0, r->scanout_frame.backing_size);
	dma_wmb();
	/* The idle cache is optional: without its (zeroed) surface it stays off. */
	r->idle_cpu = apple_dart_dcp_scanout_alloc(dcp->scanout, r->scanout_frame.backing_size,
						   &dva);
	if (IS_ERR(r->idle_cpu)) {
		dev_warn(dcp->dev, "no idle-cache surface: %ld\n", PTR_ERR(r->idle_cpu));
		r->idle_cpu = NULL;
		r->idle_unavailable = true;
	} else {
		r->idle_frame = r->scanout_frame;
		r->idle_frame.dva = dva;
	}
	return dcp_m4_runtime_health(r);
}

/* Called with all KMS modeset locks after the DRM device is registered. */
static int dcp_m4_initial_scanout(struct iomfb_m4 *dcp)
{
	struct dcp_m4_runtime *r = dcp->runtime;
	int ret;

	if (!r || !r->scanout_cpu)
		return -ENODEV;
	ret = dcp_m4_runtime_health(r);
	if (!ret)
		ret = dcp_m4_runtime_swap(r, &r->scanout_frame);
	if (ret)
		return ret;
	dcp->active_frame = r->scanout_frame;
	smp_store_release(&r->scanout_done, true);
	return 0;
}

/*
 * The common DMA GEM helpers map this object into the pinned SID0 consumer and
 * require one contiguous DMA extent.  A framebuffer reference keeps that DMA
 * mapping stable.  Full DART/PMP validation belongs to startup and health
 * checks, rather than an expensive page-table sweep on every display flip.
 */
static int iomfb_m4_check_frame(const struct dcp_m4_frame *frame)
{
	int ret;

	ret = dcp_m4_direct_extent(frame->dva, frame->backing_size,
				   (u64)frame->stride * frame->height);
	return ret ?: dcp_m4_frame_check(frame);
}

static void iomfb_m4_backlight_work(struct work_struct *work);
static void iomfb_m4_idle_cache_work(struct work_struct *work);
static void iomfb_m4_ext_put_work(struct work_struct *work);

/* Once adopted, even driver unbind must preserve the callback cookie and DMA. */
static void iomfb_m4_release(void *cookie)
{
	struct iomfb_m4 *dcp = cookie;

	if (READ_ONCE(dcp->started) || READ_ONCE(dcp->owner->retain_dma))
		return;
	if (dcp->rtkit)
		apple_rtkit_free(dcp->rtkit);
	if (dcp->coproc)
		iounmap(dcp->coproc);
	if (dcp->coproc_size)
		release_mem_region(dcp->coproc_phys, dcp->coproc_size);
	kfree(dcp);
}

/* A UUID identifies the image; both loader version claims must agree with it. */
static bool iomfb_m4_firmware_property(struct device_node *np, const char *name)
{
	u32 version[3];

	return of_property_count_u32_elems(np, name) == ARRAY_SIZE(version) &&
	       !of_property_read_u32_array(np, name, version, ARRAY_SIZE(version)) &&
	       version[0] == 26 && version[1] == 6 && version[2] == 2;
}

/*
 * The realtime bandwidth mailbox the DT gives @dev, or 0: apple,bw-scratch
 * names the provider's reg range, the firmware's display register index and
 * an offset into the range. The mailbox is DCP_M4_BW_MAILBOX_SIZE bytes, and
 * must lie inside display register window 4, through which the firmware
 * reaches a scratch-only mailbox.
 */
static u64 dcp_m4_bw_mailbox(struct device *dev)
{
	struct of_phandle_args args;
	struct resource range, *win = NULL;
	u64 slot = 0;
	int ret;

	ret = of_parse_phandle_with_args(dev->of_node, "apple,bw-scratch",
					 "#apple,bw-scratch-cells", 0, &args);
	if (ret == -ENOENT)
		return 0;
	if (ret) {
		dev_warn(dev, "apple,bw-scratch: %d: realtime bandwidth declined\n", ret);
		return 0;
	}
	if (args.args_count == 3 && args.args[1] == 4 &&
	    !of_address_to_resource(args.np, args.args[0], &range) &&
	    resource_size(&range) >= DCP_M4_BW_MAILBOX_SIZE &&
	    args.args[2] <= resource_size(&range) - DCP_M4_BW_MAILBOX_SIZE) {
		win = platform_get_resource_byname(to_platform_device(dev), IORESOURCE_MEM,
						   "disp-4");
		slot = range.start + args.args[2];
	}
	of_node_put(args.np);
	if (!win || slot < win->start || slot + DCP_M4_BW_MAILBOX_SIZE - 1 > win->end) {
		dev_warn(dev, "apple,bw-scratch: no mailbox inside disp-4: realtime bandwidth declined\n");
		return 0;
	}
	return slot;
}

/* The DART behind @consumer's only "iommus" entry, owned for its power cycle. */
static int iomfb_m4_ext_bind_dart(struct iomfb_m4 *dcp, struct device_node *consumer,
				  struct apple_dart_cio **dart)
{
	struct of_phandle_args spec;
	struct platform_device *pdev;
	int ret;

	if (!consumer ||
	    of_parse_phandle_with_args(consumer, "iommus", "#iommu-cells", 0, &spec))
		return -ENODEV;
	pdev = of_find_device_by_node(spec.np);
	of_node_put(spec.np);
	if (!pdev)
		return -EPROBE_DEFER;
	ret = apple_dart_cio_bind(dcp->dev, &pdev->dev, dart);
	put_device(&pdev->dev);
	return ret;
}

/* The sink's EDID was not read yet (iomfb_m4_ext_edid()): probe again. */
static void iomfb_m4_ext_edid_retry_work(struct work_struct *work)
{
	struct iomfb_m4 *dcp = container_of(to_delayed_work(work), struct iomfb_m4,
					    ext_edid_retry);

	t8132_ext_drm_hotplug(dcp->dev);
}

int iomfb_m4_init(struct apple_dcp *owner)
{
	struct platform_device *pdev = to_platform_device(owner->dev);
	struct device *dev = owner->dev;
	struct iomfb_m4 *dcp;
	struct resource *res;
	const struct dcp_m4_profile *pf = dcp_m4_machine_profile();
	u32 ready = 0;
	int ret;
	/* An external DCP the boot loader handed over: same firmware, no panel. */
	bool external = of_device_is_compatible(dev->of_node, "apple,t8132-dcpext");

	if (external &&
	    (of_property_read_u32(dev->of_node, "apple,t8132-dcpext-firmware-ready", &ready) ||
	     ready != 1))
		return dev_err_probe(dev, -ENODEV, "external DCP not handed over\n");
	if (!pf ||
	    (!external && !of_device_is_compatible(dev->of_node, "apple,t8132-dcp")) ||
	    !iomfb_m4_firmware_property(dev->of_node, "apple,firmware-version") ||
	    !iomfb_m4_firmware_property(dev->of_node, "apple,firmware-compat"))
		return dev_err_probe(dev, -ENODEV, "unqualified firmware 26.6.2 handoff\n");
	/* Refuse before any message, so the boot framebuffer stays. */
	if (!external && !pf->timing)
		return dev_err_probe(dev, -ENODEV, "no qualified %s panel timing\n", pf->name);
	dcp_m4_pf = pf;
	ret = dcp_m4_dart_check(dev);
	if (ret)
		return dev_err_probe(dev, ret, "inherited SID23 domain not qualified\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "coproc");
	if (!res || resource_size(res) != SZ_16K ||
	    (external ? res->start != 0x286e00000ULL && res->start != 0x48ae00000ULL :
			res->start != 0x30ae00000ULL))
		return -ENODEV;
	/* Resolve the fallible supplier before creating a mailbox owner. */
	ret = external ? 0 : apple_pmp_j713_display_prepare(dev);
	if (ret)
		return dev_err_probe(dev, ret, "PMP supplier not ready\n");
	dcp = kzalloc(sizeof(*dcp), GFP_KERNEL);
	if (!dcp)
		return -ENOMEM;
	dcp->owner = owner;
	dcp->dev = dev;
	dcp->bw_mailbox = dcp_m4_bw_mailbox(dev);
	mutex_init(&dcp->lock);
	mutex_init(&dcp->ext_modes_lock);
	spin_lock_init(&dcp->state_lock);
	init_completion(&dcp->reply);
	INIT_WORK(&dcp->backlight_work, iomfb_m4_backlight_work);
	spin_lock_init(&dcp->ext_put_lock);
	INIT_LIST_HEAD(&dcp->ext_puts);
	INIT_WORK(&dcp->ext_put_work, iomfb_m4_ext_put_work);
	init_waitqueue_head(&dcp->ext_settle_wq);
	INIT_DELAYED_WORK(&dcp->idle_cache, iomfb_m4_idle_cache_work);
	INIT_DELAYED_WORK(&dcp->gate_work, iomfb_m4_gate_work);
	INIT_DELAYED_WORK(&dcp->ext_edid_retry, iomfb_m4_ext_edid_retry_work);
	if (!request_mem_region(res->start, resource_size(res), dev_name(dev))) {
		ret = -EBUSY;
		goto release;
	}
	dcp->coproc_phys = res->start;
	dcp->coproc_size = resource_size(res);
	dcp->coproc = ioremap(res->start, resource_size(res));
	if (!dcp->coproc) {
		ret = -ENOMEM;
		goto release;
	}
	if (!(readl(dcp->coproc + 0x44) & BIT(4))) {
		ret = -ENODEV;
		goto release;
	}
	dcp->rtkit = apple_rtkit_init(dev, dcp, "mbox", 0, &dcp_m4_rtkit_ops);
	if (IS_ERR(dcp->rtkit)) {
		ret = PTR_ERR(dcp->rtkit);
		dcp->rtkit = NULL;
		goto release;
	}
	/* No messages are sent in probe; a failed local probe has no adopted DMA. */
	ret = devm_add_action_or_reset(dev, iomfb_m4_release, dcp);
	if (ret)
		return ret;
	dcp->external = external;
	if (external) {
		struct device_node *piodma = of_get_child_by_name(dev->of_node, "piodma");

		/* Without them a deep sleep with a session is refused, nothing else. */
		ret = iomfb_m4_ext_bind_dart(dcp, dev->of_node, &dcp->ext_darts[0]) ?:
		      iomfb_m4_ext_bind_dart(dcp, piodma, &dcp->ext_darts[1]);
		of_node_put(piodma);
		if (ret)
			dev_info(dev, "deep sleep with an external display not supported here: %d\n",
				 ret);
	}
	/* The internal display's power-down: its PMC interface names the rest. */
	dcp->gate_capable = !external && apple_pmp_j713_board() &&
			    of_property_present(dev->of_node, "apple,pmc-interface");
	owner->v26_6 = dcp;
	owner->rtk = dcp->rtkit;
	owner->coproc_reg = dcp->coproc;
	pm_runtime_get_noresume(dev);
	dev_info(dev, "firmware 26.6.2 retained DCP backend prepared\n");
	return 0;
release:
	iomfb_m4_release(dcp);
	return dev_err_probe(dev, ret, "cannot prepare retained firmware 26.6.2 backend\n");
}

/*
 * The auxiliary endpoints of an external DCP are started so that the
 * firmware finds them up. Of their services only the sink's AV service on
 * EP 0x28 is used (dcp_m4_avserv_init()); every other one announced is only
 * noted.
 */
static void dcp_m4_probe_init(struct apple_epic_service *service, const char *name,
			      const char *class, s64 unit)
{
	dev_dbg(service->ep->dcp->dev,
		"ep %02x channel %u announces service \"%s\" class \"%s\" unit %lld\n",
		service->ep->endpoint, service->channel, name ?: "", class ?: "", unit);
}

/* The sink's AV service, which reads its EDID (iomfb_m4_ext_edid()). */
static void dcp_m4_avserv_init(struct apple_epic_service *service, const char *name,
			       const char *class, s64 unit)
{
	struct iomfb_m4 *dcp = service->ep->dcp->v26_6;
	unsigned long flags;

	dcp_m4_probe_init(service, name, class, unit);
	if (service->ep->endpoint != DPAVSERV_ENDPOINT)
		return;
	spin_lock_irqsave(&dcp->state_lock, flags);
	dcp->ext_avserv = service;
	dcp->ext_avserv_gen++;
	dcp->ext_edid_retries = 0;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	/* get_modes may already have run without it. */
	t8132_ext_drm_hotplug(dcp->dev);
}

static void dcp_m4_avserv_teardown(struct apple_epic_service *service)
{
	struct iomfb_m4 *dcp = service->ep->dcp->v26_6;
	unsigned long flags;

	spin_lock_irqsave(&dcp->state_lock, flags);
	if (dcp->ext_avserv == service)
		dcp->ext_avserv = NULL;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
}

static const struct apple_epic_service_ops dcp_m4_probe_ops[] = {
	{
		.name = "dcpav-service-epic",
		.init = dcp_m4_avserv_init,
		.teardown = dcp_m4_avserv_teardown,
	},
	{
		.name = "*",
		.init = dcp_m4_probe_init,
	},
	{},
};

/* AFK rings come from the retained DART allocator; they are never freed. */
static void *dcp_m4_afk_alloc(struct apple_dcp *owner, size_t size, dma_addr_t *dva)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	void *buf = dcp_m4_dma_alloc(dcp->dev, size, dva);

	if (!IS_ERR(buf))
		dcp->allocated += size;
	return buf;
}

/* Start an external DCP's endpoints, those it advertises, in a fixed order. */
static void dcp_m4_probe_endpoints(struct iomfb_m4 *dcp)
{
	static const u8 eps[] = { 0x27, 0x24, 0x26, 0x29, 0x2b, 0x2d, 0x28, 0x22, 0x25 };
	unsigned int i;
	int ret;

	dcp->owner->afk_alloc = dcp_m4_afk_alloc;
	for (i = 0; i < ARRAY_SIZE(eps); i++) {
		struct apple_dcp_afkep *ep;

		if (!apple_rtkit_has_endpoint(dcp->rtkit, eps[i]))
			continue;
		ep = afk_init(dcp->owner, eps[i], dcp_m4_probe_ops);
		if (IS_ERR(ep)) {
			dev_warn(dcp->dev, "endpoint %02x init failed: %ld\n", eps[i],
				 PTR_ERR(ep));
			continue;
		}
		WRITE_ONCE(dcp->probe_ep[eps[i] - 0x20], ep);
		ret = afk_start(ep);
		if (ret)
			dev_warn(dcp->dev, "endpoint %02x start failed: %d\n", eps[i], ret);
	}

	/* The DPTX port services live on EP 0x2a on this firmware. */
	ret = dptxep_init_desktop(dcp->owner, DPTX_ENDPOINT);
	if (ret)
		dev_err(dcp->dev, "DPTX ports on EP %#x: %d\n", DPTX_ENDPOINT, ret);
	dev_dbg(dcp->dev, "DPTX port0 %s, port1 %s\n",
		dcp->owner->dptxport[0].service ? "bound" : "absent",
		dcp->owner->dptxport[1].service ? "bound" : "absent");
}

int iomfb_m4_adopt_external(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	int ret;

	if (!dcp)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	ret = dcp_m4_adopt(dcp);
	mutex_unlock(&dcp->lock);
	if (!ret) {
		char eps[3 * 224 + 1] = "";
		size_t len = 0;
		unsigned int ep;

		/* Which application endpoints this firmware advertises. */
		for (ep = 0x20; ep < 0x100; ep++)
			if (apple_rtkit_has_endpoint(dcp->rtkit, ep))
				len += scnprintf(eps + len, sizeof(eps) - len, " %02x", ep);
		dev_info(dcp->dev, "external DCP adopted\n");
		dev_dbg(dcp->dev, "endpoints:%s\n", eps);
		dcp_m4_probe_endpoints(dcp);
		/* A failed startup quarantines the session; the device stays bound. */
		mutex_lock(&dcp->lock);
		dcp_m4_startup(dcp);
		mutex_unlock(&dcp->lock);
	}
	return ret;
}

int iomfb_m4_start(struct apple_dcp *owner, struct device *scanout)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;
	struct iommu_group *group;
	int ret;

	if (!dcp || !scanout)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	if (READ_ONCE(dcp->error)) {
		ret = READ_ONCE(dcp->error);
		goto out;
	}
	if (dcp->started) {
		ret = dcp->error ?: (dcp->runtime && dcp->runtime->scanout_cpu ? 0 : -EBUSY);
		goto out;
	}
	ret = dcp_m4_dart_check(scanout);
	if (ret)
		goto out;
	/* DMA GEM maps in this exact consumer; firmware outlives master unbind. */
	group = iommu_group_get(scanout);
	if (!group) {
		ret = -ENODEV;
		goto out;
	}
	ret = iommu_device_use_default_domain(scanout);
	if (ret) {
		iommu_group_put(group);
		goto out;
	}
	get_device(scanout);
	dcp->scanout = scanout;
	dcp->scanout_group = group;
	pm_runtime_get_noresume(scanout);
	ret = dcp_m4_adopt(dcp);
	if (ret)
		goto fail;
	ret = dcp_m4_startup(dcp);
	if (ret)
		goto fail;
	r = dcp->runtime;
	ret = dcp_m4_prepare_scanout(dcp);
	if (ret)
		goto fail;
	dev_info(dcp->dev, "firmware 26.6.2 prepared for appledrm: %ux%u; boot scanout preserved\n",
		 r->timing.hdisplay, r->timing.vdisplay);
	goto out;
fail:
	dcp_m4_failed(dcp, ret);
	dev_err(dcp->dev, "firmware 26.6.2 startup failed: %d; session retained until reboot\n",
		ret);
out:
	mutex_unlock(&dcp->lock);
	return ret;
}

int iomfb_m4_activate(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;
	int ret;

	if (!dcp || !dcp->runtime)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	ret = READ_ONCE(dcp->error);
	if (ret || READ_ONCE(dcp->runtime->qualified))
		goto out;
	r = dcp->runtime;
	/* Preserve the firmware-qualified blank -> power vote -> surface
	 * sequence, entirely after KMS owns the aperture and is registered.
	 */
	ret = dcp_m4_runtime_background(r);
	if (ret)
		goto failed;
	r->power_started = true;
	ret = dcp_m4_runtime_health(r);
	if (!ret)
		ret = dcp_m4_pmp_get(dcp->dev);
	if (!ret)
		ret = dcp_m4_pmp_check(dcp->dev);
	if (!ret)
		ret = dcp_m4_runtime_health(r);
	if (ret)
		goto failed;
	smp_store_release(&r->power_done, true);
	ret = dcp_m4_initial_scanout(dcp);
	if (!ret)
		ret = dcp_m4_runtime_health(r);
	if (ret)
		goto failed;
	WRITE_ONCE(r->qualified, true);
	dev_info(dcp->dev, "firmware 26.6.2 KMS scanout activated\n");
	goto out;
failed:
	dcp_m4_failed(dcp, ret);
out:
	mutex_unlock(&dcp->lock);
	return ret;
}

int iomfb_m4_get_mode(struct apple_dcp *owner, struct drm_display_mode *mode)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_timing *t;

	if (!dcp || !dcp->runtime || !smp_load_acquire(&dcp->runtime->done))
		return -ENODEV;
	if (READ_ONCE(dcp->error))
		return READ_ONCE(dcp->error);
	t = &dcp->runtime->timing;
	*mode = (struct drm_display_mode) {
		.clock = t->clock_khz,
		.hdisplay = t->hdisplay, .hsync_start = t->hsync_start,
		.hsync_end = t->hsync_end, .htotal = t->htotal,
		.vdisplay = t->vdisplay, .vsync_start = t->vsync_start,
		.vsync_end = t->vsync_end, .vtotal = t->vtotal,
		.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
	};
	drm_mode_set_name(mode);
	return 0;
}

int iomfb_m4_error(struct apple_dcp *owner)
{
	return owner->v26_6 ? READ_ONCE(owner->v26_6->error) : -ENODEV;
}

/*
 * An external DCP's DPTX link went: the next sink starts from scratch.
 */
void iomfb_m4_external_link_gone(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;

	/* Pairs with the release store of dcp->runtime in startup. */
	r = dcp ? smp_load_acquire(&dcp->runtime) : NULL;
	if (r && dcp->external) {
		/* A swap waiting for its D590 stops waiting (dcp_m4_ext_swap_wait()). */
		atomic_inc(&r->ext_link_gen);
		/* The direct swaps in flight are given up now. */
		if (READ_ONCE(r->ext_qn))
			mod_delayed_work(r->wq, &r->ext_async_work, 0);
		/* A new link takes a new power-on and mode. */
		WRITE_ONCE(r->ext_powered, false);
		WRITE_ONCE(r->ext_cur_timing, 0);
	}
	/* The next sink's timing catalog is its own, nothing received before. */
	if (dcp && dcp->external)
		dcp_m4_ext_sink_gone(dcp, true);
	/* No sink until the next D575 reports one. */
	if (dcp && dcp->external && READ_ONCE(dcp->ext_sink)) {
		WRITE_ONCE(dcp->ext_sink, false);
		t8132_ext_drm_hotplug(dcp->dev);
	}
}

/*
 * 1 if the adopted external DCP's session is up and a
 * sink is connected (its last D575), 0 if not. Lock-free.
 */
int iomfb_m4_ext_detect(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;

	if (!dcp || !dcp->external)
		return -ENODEV;
	/* Released for a deep sleep: connected as it was until it is back. */
	if (READ_ONCE(dcp->ext_relink))
		return 1;
	/* A tunnel whose pixel clock was refused cannot show. */
	return iomfb_m4_external_ready(owner) && READ_ONCE(dcp->ext_sink) &&
	       !READ_ONCE(owner->tb_clock_refused);
}

/*
 * The sink's modes for its DRM connector: the cached 8-bpc RGB SDR timings that
 * fit the DPTX link at 24 bits per pixel, when the link rate and lane count
 * are known (8b/10b coding); the firmware's own judgement is in the colour
 * elements it marks unsafe or DSC-only, which are never used. The preferred
 * one is the highest refresh rate at the sink's native size that fits, else
 * the largest sink-preferred timing that fits, both at 50 Hz or more (see
 * dcp_m4_ext_pick_preferred()), else 1920x1080 at 60 Hz (else the first).
 * Takes ext_modes_lock only, never the session.
 */
/* The DPTX link's capacity in kbit/s at 8b/10b, 0 while not known. */
static u64 dcp_m4_ext_capacity(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	u32 lanes = READ_ONCE(owner->dptxport[0].lane_count);
	u32 rate = READ_ONCE(owner->dptxport[0].link_rate);

	/* Released for a deep sleep: the link it had until it is back. */
	if (dcp && READ_ONCE(dcp->ext_relink))
		return READ_ONCE(dcp->ext_hold_capacity);
	return (u64)lanes * rate * 270 * 1000 * 8 / 10;
}

/*
 * Whether a cached mode fits the link: at 24 bpp uncompressed, at 8 bpp (the
 * lowest DSC target) with a colour that needs DSC, whose rate the firmware
 * picks itself.
 */
static bool dcp_m4_ext_fits(const struct iomfb_m4_ext_mode *e, u64 capacity)
{
	return !capacity || (u64)e->mode.clock * (e->dsc ? 8 : 24) <= capacity;
}

/*
 * The firmware's IDs of a mode iomfb_m4_ext_get_modes()
 * offered: -ENOENT for any other mode, -ENOSPC for one over the link.
 */
int iomfb_m4_ext_mode_ids(struct apple_dcp *owner, const struct drm_display_mode *mode,
			  struct iomfb_m4_ext_ids *ids)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	u64 capacity = dcp_m4_ext_capacity(owner);
	int ret = -ENOENT;

	if (!dcp || !dcp->external)
		return -ENODEV;
	mutex_lock(&dcp->ext_modes_lock);
	for (unsigned int i = 0; i < dcp->ext_mode_count; i++) {
		const struct iomfb_m4_ext_mode *e = &dcp->ext_modes[i];

		if (!drm_mode_match(&e->mode, mode, DRM_MODE_MATCH_TIMINGS |
				    DRM_MODE_MATCH_CLOCK | DRM_MODE_MATCH_FLAGS))
			continue;
		if (!dcp_m4_ext_fits(e, capacity)) {
			ret = -ENOSPC;
			continue;
		}
		ids->timing = e->timing;
		ids->color = e->color;
		ids->bootstrap = e->bootstrap;
		dev_dbg(dcp->dev, "mode %ux%u@%d: timing %u, colour %u%s\n",
			e->mode.hdisplay, e->mode.vdisplay, drm_mode_vrefresh(&e->mode),
			e->timing, e->color, e->dsc ? " (DSC)" : "");
		ret = 0;
		break;
	}
	mutex_unlock(&dcp->ext_modes_lock);
	return ret;
}

/* A mode's refresh rate in mHz, to compare 59.94 with 60 Hz. */
static u64 dcp_m4_ext_mhz(const struct drm_display_mode *m)
{
	u64 total = (u64)m->htotal * m->vtotal;

	return total ? div64_u64((u64)m->clock * 1000000, total) : 0;
}

static bool dcp_m4_ext_same_size(const struct drm_display_mode *a, u32 width, u32 height)
{
	return a->hdisplay == width && a->vdisplay == height;
}

/* One of the sink's preferred timings: PreferredTimingElements or IsPreferred. */
static bool dcp_m4_ext_sink_preferred(const struct iomfb_m4 *dcp,
				      const struct iomfb_m4_ext_mode *e)
{
	for (unsigned int k = 0; k < dcp->ext_pref_count; k++)
		if (dcp->ext_pref[k].id == e->timing)
			return true;
	return e->fw_preferred;
}

/* At least 50 Hz, in mHz as dcp_m4_ext_mhz(). */
#define DCP_M4_EXT_MIN_MHZ	50000

/*
 * The cached mode to mark preferred, -1 for none (the
 * 1920x1080 fallback), never one that needs DSC (the user picks those), with
 * the rule that chose it in @why:
 * 1. the native size is the largest (pixel area) of the sink's preferred
 *    timings: its PreferredTimingElements (their own sizes too) and its
 *    IsPreferred timings;
 * 2. the mode at the native size that fits the link at 50 Hz or more with
 *    the highest refresh rate, then a sink-preferred one, then the higher
 *    firmware score;
 * 3. else the largest sink-preferred timing that fits at 50 Hz or more,
 *    the higher refresh rate on a tie.
 * ext_modes_lock held.
 */
static int dcp_m4_ext_pick_preferred(struct iomfb_m4 *dcp, u64 capacity, const char **why)
{
	const struct iomfb_m4_ext_mode *e = dcp->ext_modes;
	unsigned int n = dcp->ext_mode_count;
	u64 native = 0;
	u32 width = 0, height = 0;
	int best = -1;

	for (unsigned int k = 0; k < dcp->ext_pref_count; k++)
		if ((u64)dcp->ext_pref[k].width * dcp->ext_pref[k].height > native) {
			width = dcp->ext_pref[k].width;
			height = dcp->ext_pref[k].height;
			native = (u64)width * height;
		}
	for (unsigned int i = 0; i < n; i++)
		if (dcp_m4_ext_sink_preferred(dcp, &e[i]) &&
		    (u64)e[i].mode.hdisplay * e[i].mode.vdisplay > native) {
			width = e[i].mode.hdisplay;
			height = e[i].mode.vdisplay;
			native = (u64)width * height;
		}
	for (unsigned int i = 0; native && i < n; i++) {
		u64 mhz = dcp_m4_ext_mhz(&e[i].mode), best_mhz;
		bool pref, best_pref;

		if (!dcp_m4_ext_same_size(&e[i].mode, width, height) || e[i].dsc ||
		    !dcp_m4_ext_fits(&e[i], capacity) || mhz < DCP_M4_EXT_MIN_MHZ)
			continue;
		if (best < 0) {
			best = i;
			continue;
		}
		best_mhz = dcp_m4_ext_mhz(&e[best].mode);
		pref = dcp_m4_ext_sink_preferred(dcp, &e[i]);
		best_pref = dcp_m4_ext_sink_preferred(dcp, &e[best]);
		if (mhz > best_mhz ||
		    (mhz == best_mhz && (pref != best_pref ? pref : e[i].score > e[best].score)))
			best = i;
	}
	if (best >= 0) {
		*why = "native size, the highest refresh rate within the link";
		return best;
	}
	for (unsigned int i = 0; i < n; i++) {
		u64 a = (u64)e[i].mode.hdisplay * e[i].mode.vdisplay, b;
		u64 mhz = dcp_m4_ext_mhz(&e[i].mode);

		if (!dcp_m4_ext_sink_preferred(dcp, &e[i]) || e[i].dsc ||
		    !dcp_m4_ext_fits(&e[i], capacity) || mhz < DCP_M4_EXT_MIN_MHZ)
			continue;
		b = best < 0 ? 0 : (u64)e[best].mode.hdisplay * e[best].mode.vdisplay;
		if (best < 0 || a > b || (a == b && mhz > dcp_m4_ext_mhz(&e[best].mode)))
			best = i;
	}
	if (best >= 0)
		*why = "the native size is over the link: the largest preferred timing within it";
	return best;
}

int iomfb_m4_ext_get_modes(struct apple_dcp *owner, struct drm_connector *connector)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	u32 lanes = READ_ONCE(owner->dptxport[0].lane_count);
	u32 rate = READ_ONCE(owner->dptxport[0].link_rate);
	u64 capacity = dcp_m4_ext_capacity(owner);
	struct drm_display_mode *preferred = NULL, *fallback = NULL;
	unsigned int added = 0, skipped = 0, total;
	const char *why = "1920x1080 at 60 Hz, else the first";
	int pick;

	if (!dcp || !dcp->external)
		return 0;
	mutex_lock(&dcp->ext_modes_lock);
	total = dcp->ext_mode_count;
	pick = dcp_m4_ext_pick_preferred(dcp, capacity, &why);
	for (unsigned int i = 0; i < total; i++) {
		const struct drm_display_mode *m = &dcp->ext_modes[i].mode;
		struct drm_display_mode *mode;

		if (!dcp_m4_ext_fits(&dcp->ext_modes[i], capacity)) {
			skipped++;
			continue;
		}
		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode)
			break;
		mode->type |= DRM_MODE_TYPE_DRIVER;
		if ((int)i == pick) {
			mode->type |= DRM_MODE_TYPE_PREFERRED;
			preferred = mode;
		}
		/* A DSC mode is never the fallback either: the user picks one. */
		if (!dcp->ext_modes[i].dsc &&
		    (!fallback || (mode->hdisplay == 1920 && mode->vdisplay == 1080 &&
				   drm_mode_vrefresh(mode) == 60)))
			fallback = mode;
		drm_mode_probed_add(connector, mode);
		added++;
	}
	if (!preferred && fallback)
		fallback->type |= DRM_MODE_TYPE_PREFERRED;
	if (dcp->ext_logged_modes != (added << 16 | skipped) ||
	    dcp->ext_logged_link != (lanes << 16 | rate) || dcp->ext_logged_pick != pick) {
		const struct drm_display_mode *p = preferred ?: fallback;

		dcp->ext_logged_modes = added << 16 | skipped;
		dcp->ext_logged_link = lanes << 16 | rate;
		dcp->ext_logged_pick = pick;
		dev_dbg(dcp->dev, "modes: %u of %u (%u over link %u x %u Mbit/s)%s; preferred %ux%u@%d Hz, %d kHz (%s)\n",
			added, total, skipped, lanes, rate * 270,
			capacity ? "" : ", link not known: not checked",
			p ? p->hdisplay : 0, p ? p->vdisplay : 0, p ? drm_mode_vrefresh(p) : 0,
			p ? p->clock : 0, preferred ? why : "1920x1080 at 60 Hz, else the first");
	}
	mutex_unlock(&dcp->ext_modes_lock);
	return added;
}

/*
 * The EDID the connector publishes, as native26's (dcp_edid_sdr()): this
 * scanout is 8-bpc RGB SDR, so the sink's colour capabilities, deep colour
 * and 4:2:0 timings go. The last would otherwise make the probe helper prune
 * the catalog's mode of a timing the sink lists as 4:2:0 only.
 */
static const struct drm_edid *iomfb_m4_ext_edid_sdr(const struct drm_edid *edid)
{
	const struct edid *raw = drm_edid_raw(edid);
	const struct drm_edid *sdr;
	size_t size;
	u8 *copy;

	if (!raw)
		return ERR_PTR(-EPROTO);
	size = (raw->extensions + 1) * EDID_LENGTH;
	copy = kmemdup(raw, size, GFP_KERNEL);
	if (!copy)
		return ERR_PTR(-ENOMEM);
	size = dcp_edid_sdr(copy, size);
	sdr = size ? drm_edid_alloc(copy, size) : ERR_PTR(-EPROTO);
	kfree(copy);
	return sdr ?: ERR_PTR(-ENOMEM);
}

/* A copy of the sink's kept EDID if it was read on link @link_gen, else NULL. */
static const struct drm_edid *iomfb_m4_ext_edid_kept(struct iomfb_m4 *dcp, int link_gen)
{
	lockdep_assert_held(&dcp->ext_modes_lock);
	if (!dcp->ext_edid || dcp->ext_edid_link_gen != link_gen)
		return NULL;
	return drm_edid_dup(dcp->ext_edid) ?: ERR_PTR(-ENOMEM);
}

/*
 * The sink's EDID for the connector's properties: its name, size and the
 * like. Its modes stay the firmware's catalog (iomfb_m4_ext_get_modes()).
 * It is read once for each announcement of the sink's AV service and kept
 * while the sink's link stays (ext_link_gen): when a read fails, or there is
 * no service (none yet after a restart, or a stopped or failed session,
 * which nothing is read from), the kept one is given, so that a relink or a
 * resume does not take it from the connector meanwhile. -ENODEV with
 * neither; the firmware may also not have read the sink yet.
 */
const struct drm_edid *iomfb_m4_ext_edid(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct apple_epic_service *service = NULL;
	const struct drm_edid *edid, *read, *old = NULL;
	struct dcp_m4_runtime *r;
	unsigned int avserv_gen;
	unsigned long flags;
	int link_gen;

	/* Pairs with the release store of dcp->runtime in startup. */
	r = dcp && dcp->external ? smp_load_acquire(&dcp->runtime) : NULL;
	if (!r)
		return ERR_PTR(-ENODEV);
	link_gen = atomic_read(&r->ext_link_gen);
	spin_lock_irqsave(&dcp->state_lock, flags);
	avserv_gen = dcp->ext_avserv_gen;
	if (!dcp->error && !READ_ONCE(dcp->gated))
		service = afk_service_get(dcp->ext_avserv);
	spin_unlock_irqrestore(&dcp->state_lock, flags);

	mutex_lock(&dcp->ext_modes_lock);
	edid = (!service || dcp->ext_edid_avserv_gen == avserv_gen) ?
	       iomfb_m4_ext_edid_kept(dcp, link_gen) : NULL;
	mutex_unlock(&dcp->ext_modes_lock);
	if (edid || !service) {
		if (service)
			afk_service_put(service);
		return edid ?: ERR_PTR(-ENODEV);
	}

	read = dcpavserv_read_edid(service);
	afk_service_put(service);
	if (IS_ERR(read))
		edid = read;
	else if (!drm_edid_valid(read))
		edid = ERR_PTR(-EPROTO);
	else
		edid = iomfb_m4_ext_edid_sdr(read);
	if (!IS_ERR(read))
		drm_edid_free(read);
	/*
	 * No EDID bytes yet: the firmware announces the service before it has
	 * read the sink (after resume, as on the Neo). Probe again shortly.
	 */
	spin_lock_irqsave(&dcp->state_lock, flags);
	if (!IS_ERR(edid)) {
		dcp->ext_edid_retries = 0;
	} else if (PTR_ERR(edid) == -ENODATA &&
		   dcp->ext_edid_retries < DCPAVSERV_EDID_RETRIES) {
		dcp->ext_edid_retries++;
		mod_delayed_work(system_freezable_wq, &dcp->ext_edid_retry,
				 msecs_to_jiffies(DCPAVSERV_EDID_RETRY_MS));
	}
	spin_unlock_irqrestore(&dcp->state_lock, flags);

	mutex_lock(&dcp->ext_modes_lock);
	if (atomic_read(&r->ext_link_gen) != link_gen) {
		/* The sink went during the read: nothing of it is kept or given. */
		old = IS_ERR(edid) ? NULL : edid;
		edid = ERR_PTR(-ENODEV);
	} else if (!IS_ERR(edid)) {
		old = dcp->ext_edid;
		dcp->ext_edid = edid;
		dcp->ext_edid_link_gen = link_gen;
		dcp->ext_edid_avserv_gen = avserv_gen;
		edid = drm_edid_dup(edid) ?: ERR_PTR(-ENOMEM);
	} else {
		edid = iomfb_m4_ext_edid_kept(dcp, link_gen) ?: edid;
	}
	mutex_unlock(&dcp->ext_modes_lock);
	drm_edid_free(old);
	return edid;
}

/*
 * An adopted external DCP whose IOMFB session opened (A401, open, drain
 * and a healthy runtime) and has not failed since. Lock-free: the runtime is
 * published once and the session is retained until reboot.
 */
bool iomfb_m4_external_ready(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;

	if (!dcp || !dcp->external || READ_ONCE(dcp->error))
		return false;
	/* Pairs with the release store of dcp->runtime in startup. */
	r = smp_load_acquire(&dcp->runtime);
	if (!r)
		return false;
	/* Pairs with the release store of r->done in startup. */
	return smp_load_acquire(&r->done);
}

int iomfb_m4_health(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	int ret;

	if (!dcp || !dcp->runtime)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	ret = dcp_m4_runtime_health(dcp->runtime);
	if (ret)
		dcp_m4_failed(dcp, ret);
	mutex_unlock(&dcp->lock);
	return ret;
}

/* Any surface ID other than the primary's 1 names a distinct surface object. */
#define DCP_M4_IDLE_SURFACE_ID	2

/*
 * Register the idle cache behind the frame on screen: IdleCachingMethod 2,
 * then a swap with intent 8 whose sole surface is the cache surface. The
 * DCP keeps presenting the frame, steps through idle states 1, 2 and 4
 * (D117), asking for a fence (D578) in state 1 and whenever it refreshes the
 * cache (4, 6, 2, 4), and leaves the cache at the next swap.
 * The surface stays registered across exits, so later entries resubmit it.
 */
static int dcp_m4_idle_cache_enter(struct iomfb_m4 *dcp)
{
	struct dcp_m4_runtime *r = dcp->runtime;
	struct { __le32 id, value; } method = { cpu_to_le32(126), cpu_to_le32(2) };
	__le32 out = 0;
	int ret;

	lockdep_assert_held(&dcp->lock);
	if (r->idle_unavailable || dcp->powered_off || dcp->display_off || !dcp->active_fb ||
	    r->idle_state || r->idle_fences)
		return 0;
	ret = dcp_m4_runtime_live(r);
	if (ret)
		return ret;
	ret = dcp_m4_runtime_call_stream(r, 467, 0, false, &method, sizeof(method),
					 &out, sizeof(out));
	if (ret)
		return ret;
	/* applyProperty status: a refused method stays refused. */
	if (out) {
		dev_warn(dcp->dev, "idle caching refused: %#x\n", le32_to_cpu(out));
		r->idle_unavailable = true;
		return 0;
	}
	/* D578 can arrive before the swap completes. */
	r->idle_registered = true;
	/* Like any primary present, this drops upper surfaces; none are in use yet. */
	ret = dcp_m4_runtime_swap_full(r, true, &r->idle_frame, NULL, 0, false, 0,
				       DCP_M4_SWAP_INTENT_IDLE_CACHE, DCP_M4_IDLE_SURFACE_ID);
	if (!ret)
		r->idle_entries++;
	return ret;
}

/* Called with dcp->lock after a swap: enter the cache once no flip follows. */
static void iomfb_m4_idle_cache_kick(struct iomfb_m4 *dcp)
{
	unsigned int ms = READ_ONCE(idle_cache_ms);

	if (ms && !READ_ONCE(dcp->owner->quiescing))
		mod_delayed_work(dcp->runtime->wq, &dcp->idle_cache, msecs_to_jiffies(ms));
}

static void iomfb_m4_idle_cache_work(struct work_struct *work)
{
	struct iomfb_m4 *dcp = container_of(to_delayed_work(work), struct iomfb_m4, idle_cache);

	mutex_lock(&dcp->lock);
	/* An RPC failure has already failed the session. */
	if (!READ_ONCE(dcp->owner->quiescing) && dcp->runtime && READ_ONCE(idle_cache_ms))
		dcp_m4_idle_cache_enter(dcp);
	mutex_unlock(&dcp->lock);
}

/* dcp->lock is the single owner of swaps, active descriptors and brightness. */
static int iomfb_m4_swap(struct iomfb_m4 *dcp, const struct dcp_m4_frame *frame)
{
	struct dcp_backlight_present present = {};
	int prepared = -ENODATA, ret;

	lockdep_assert_held(&dcp->lock);
	if (dcp_backlight_active(dcp->owner))
		prepared = dcp_backlight_prepare(dcp->owner, true, &present);
	if (prepared && prepared != -EALREADY && prepared != -ENODATA)
		return prepared;
	ret = dcp_m4_runtime_swap_brightness(dcp->runtime, frame,
					    !prepared, present.nits);
	if (!prepared)
		dcp_backlight_complete(dcp->owner, present.sequence, !ret);
	return ret;
}

int iomfb_m4_present(struct apple_dcp *owner, struct drm_framebuffer *fb,
		     dma_addr_t dva, size_t mapped)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct drm_framebuffer *retire = NULL;
	struct dcp_m4_frame frame;
	int ret;

	if (!dcp || !dcp->runtime || !fb || fb->format->num_planes != 1 ||
	    fb->offsets[0] || fb->modifier != DRM_FORMAT_MOD_LINEAR ||
	    (fb->format->format != DRM_FORMAT_XRGB8888 &&
	     fb->format->format != DRM_FORMAT_ARGB8888))
		return -EINVAL;
	ret = dcp_m4_direct_frame(&frame, dva, mapped, fb->width, fb->height,
				  fb->pitches[0], fb->format->format == DRM_FORMAT_ARGB8888);
	if (ret)
		return ret;
	/* A power-down not yet started is not needed; one under way finishes first. */
	cancel_delayed_work(&dcp->gate_work);
	mutex_lock(&dcp->lock);
	ret = dcp_m4_runtime_live(dcp->runtime);
	if (!ret && dcp->powered_off)
		ret = -EAGAIN;
	if (!ret)
		ret = iomfb_m4_check_frame(&frame);
	if (!ret && dcp->gated)
		ret = iomfb_m4_power_ungate(dcp);
	/* The first frame after a blank powers the display back up. */
	if (!ret && dcp->display_off) {
		ktime_t start = ktime_get();

		ret = dcp_m4_set_power_state(dcp, true);
		if (!ret) {
			dcp->display_off = false;
			dcp->display_on_us = ktime_us_delta(ktime_get(), start);
			dcp->display_on_us_max = max(dcp->display_on_us,
						     dcp->display_on_us_max);
		}
	}
	if (ret)
		goto out;
	/* Keep the source before the first work descriptor can become visible. */
	drm_framebuffer_get(fb);
	ret = iomfb_m4_swap(dcp, &frame);
	if (ret) {
		/* Neither the old surface nor this successor has a release receipt. */
		dcp->uncertain_fb = fb;
		dcp_m4_failed(dcp, ret);
		goto out;
	}
	retire = dcp->active_fb;
	dcp->active_fb = fb;
	dcp->active_frame = frame;
	dcp->flips++;
	iomfb_m4_idle_cache_kick(dcp);
out:
	mutex_unlock(&dcp->lock);
	if (retire)
		drm_framebuffer_put(retire);
	return ret;
}

int iomfb_m4_blank(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct drm_framebuffer *retire = NULL;
	int ret;

	if (!dcp || !dcp->runtime)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	ret = dcp_m4_runtime_live(dcp->runtime);
	/* Already blanked and powered down: nothing to swap or power down */
	if (!ret && dcp->display_off && !dcp->active_frame.dva) {
		mutex_unlock(&dcp->lock);
		return 0;
	}
	if (!ret)
		ret = iomfb_m4_swap(dcp, NULL);
	if (ret) {
		dcp_m4_failed(dcp, ret);
	} else {
		retire = dcp->active_fb;
		dcp->active_fb = NULL;
		memset(&dcp->active_frame, 0, sizeof(dcp->active_frame));
		dcp->blanks++;
		/*
		 * Nothing is scanned out until the next frame: power the display
		 * down as system sleep does. A refusal fails the session, as there.
		 */
		if (!dcp->display_off) {
			ktime_t start = ktime_get();

			if (!dcp_m4_set_power_state(dcp, false)) {
				dcp->display_off = true;
				dcp->display_off_cycles++;
				dcp->display_off_us = ktime_us_delta(ktime_get(), start);
				dcp->display_off_us_max = max(dcp->display_off_us,
							      dcp->display_off_us_max);
				unsigned int gate_ms = READ_ONCE(power_gate_ms);

				if (dcp->gate_capable && gate_ms)
					mod_delayed_work(system_dfl_wq, &dcp->gate_work,
							 msecs_to_jiffies(gate_ms));
			}
		}
	}
	mutex_unlock(&dcp->lock);
	if (retire)
		drm_framebuffer_put(retire);
	return ret;
}

/* setPowerState (A473), on the out-of-band stream on IOMFB 26.x. */
struct dcp_m4_power_state_req {
	__le64 state;
	u8 flags[4];
	u8 result_null;
	u8 padding[3];
} __packed;
static_assert(sizeof(struct dcp_m4_power_state_req) == 16);

struct dcp_m4_power_state_reply {
	__le32 value;
	__le32 status;
} __packed;
static_assert(sizeof(struct dcp_m4_power_state_reply) == 8);

static int dcp_m4_set_power_state(struct iomfb_m4 *dcp, bool on)
{
	struct dcp_m4_power_state_req req = { .state = cpu_to_le64(on) };
	struct dcp_m4_power_state_reply reply = {};
	int ret;

	lockdep_assert_held(&dcp->lock);
	ret = dcp_m4_runtime_live(dcp->runtime);
	if (!ret)
		ret = dcp_m4_runtime_call_stream(dcp->runtime, 473, 1, false, &req,
						 sizeof(req), &reply, sizeof(reply));
	if (!ret && reply.status) {
		dev_err(dcp->dev, "display power %s refused: status %#x\n",
			on ? "on" : "off", le32_to_cpu(reply.status));
		ret = -EIO;
		dcp_m4_failed(dcp, ret);
	}
	/* A display powered back up starts outside the idle cache. */
	if (!ret && on) {
		dcp->runtime->idle_state = 0;
		dcp->runtime->idle_fences = 0;
	}
	return ret;
}

/* Stop or start the DCP CPU; stopping waits for its idle status first. */
static int iomfb_m4_cpu_run(struct iomfb_m4 *dcp, bool run)
{
	u32 control = readl(dcp->coproc + DCP_M4_CPU_CONTROL), status;
	int ret;

	if (!!(control & DCP_M4_CPU_RUN) == run)
		return -EBUSY;
	if (!run) {
		ret = readl_poll_timeout(dcp->coproc + DCP_M4_CPU_STATUS, status,
					 status & DCP_M4_CPU_STATUS_IDLE, 100, 100000);
		if (ret)
			return ret;
	}
	control = run ? control | DCP_M4_CPU_RUN : control & ~DCP_M4_CPU_RUN;
	writel(control, dcp->coproc + DCP_M4_CPU_CONTROL);
	return readl(dcp->coproc + DCP_M4_CPU_CONTROL) == control ? 0 : -EIO;
}

static int iomfb_m4_cpu_start(void *data)
{
	return iomfb_m4_cpu_run(data, true);
}

/*
 * Whether every provider of the power-down can take part now, changing
 * nothing: its display DARTs, the PMP vote (held, and releasable without
 * losing it), the PMC interface enables and the PMGR states.
 */
static int iomfb_m4_gate_check(struct iomfb_m4 *dcp)
{
	return apple_dart_dcp_power_check(dcp->dev) ?:
	       apple_dart_dcp_power_check(&dcp->piodma->dev) ?:
	       apple_pmp_j713_display_check_release(dcp->dev) ?:
	       apple_pmc_interface_consumer_check(dcp->dev) ?:
	       apple_pmgr_ps_consumer_check(dcp->dev);
}

/*
 * After the blank's setPowerState(0), power the DCP and the display domains
 * down as macOS does for display sleep: release the AP link (A500), let RTKit
 * idle, stop the CPU, then power down its display DARTs, release its PMP
 * vote, clear its PMC interface enables and power its CPU, frontend and
 * system domains off. Firmware, endpoints and shared buffers stay in place
 * for iomfb_m4_power_ungate(). Before the first step every provider is
 * asked whether it can take part: if one cannot, nothing changes and the
 * display just stays logically off. A failure after that fails the session.
 */
static int iomfb_m4_power_gate(struct iomfb_m4 *dcp)
{
	struct dcp_m4_runtime *r = dcp->runtime;
	ktime_t start = ktime_get();
	unsigned long flags;
	__le32 zero = 0;
	const char *step;
	int ret;

	lockdep_assert_held(&dcp->lock);
	if (!dcp->gate_capable || dcp->gated || !dcp->display_off || dcp->powered_off ||
	    dcp->active_fb || dcp->active_frame.dva || dcp->transaction.calls)
		return -EBUSY;
	ret = dcp_m4_runtime_live(r);
	if (ret)
		return ret;
	ret = iomfb_m4_gate_check(dcp);
	if (ret) {
		dev_dbg(dcp->dev, "display power-down not possible now (%d): stays logically off\n",
			ret);
		return ret;
	}

	step = "A500";
	ret = dcp_m4_runtime_call(r, 500, false, &zero, sizeof(zero), NULL, 0);
	if (!ret) {
		step = "drain";
		ret = dcp_m4_runtime_drain(r, 64);
		if (ret > 0 || (!ret && r->count))
			ret = -EBUSY;
	}
	if (!ret) {
		step = "RTKit idle";
		ret = apple_rtkit_idle(dcp->rtkit);
	}
	if (ret)
		goto fail;
	apple_rtkit_hold_rx(dcp->rtkit);
	step = "CPU stop";
	ret = iomfb_m4_cpu_run(dcp, false);
	if (ret)
		goto fail;
	/*
	 * The stopped CPU sends nothing more. A message it sent during the idle
	 * handshake would be handled after the power-down, by firmware that is
	 * stopped and through DARTs that are off: take everything in now, and
	 * fail here, with nothing powered down, if anything arrived.
	 */
	step = "late message";
	ret = apple_rtkit_poll(dcp->rtkit);
	if (ret >= 0) {
		apple_rtkit_flush_rx(dcp->rtkit);
		spin_lock_irqsave(&dcp->state_lock, flags);
		ret = r->count ? -EBUSY : 0;
		spin_unlock_irqrestore(&dcp->state_lock, flags);
	}
	if (ret)
		goto fail;

	/* From here on nothing may use the DCP's or its DARTs' registers. */
	WRITE_ONCE(dcp->gated, true);
	step = "DART";
	ret = apple_dart_dcp_power_off(dcp->dev);
	if (!ret)
		ret = apple_dart_dcp_power_off(&dcp->piodma->dev);
	if (!ret) {
		step = "PMP";
		ret = apple_pmp_j713_display_put(dcp->dev);
	}
	if (!ret) {
		step = "PMC";
		ret = apple_pmc_interface_consumer_off(dcp->dev);
	}
	if (!ret) {
		step = "PMGR";
		ret = apple_pmgr_ps_consumer_off(dcp->dev);
	}
	if (ret)
		goto fail;
	dcp->gate_cycles++;
	dcp->gate_off_us = ktime_us_delta(ktime_get(), start);
	return 0;
fail:
	dev_err(dcp->dev, "display power-down failed at %s: %d\n", step, ret);
	dcp_m4_failed(dcp, ret);
	return ret;
}

/*
 * Power the DCP back up for the first frame after a gated blank, in macOS's
 * order: the domains, the PMP vote, the PMC interface enables, then the
 * display DARTs, then the CPU through RTKit's retained wake, and the display
 * preparation macOS sends before setPowerState(1). A failure fails the
 * session.
 */
static int iomfb_m4_power_ungate(struct iomfb_m4 *dcp)
{
	struct dcp_m4_runtime *r = dcp->runtime;
	ktime_t start = ktime_get();
	u8 prepare[8] = {}, prepared[8] = {};
	__le32 main_display = 0;
	const char *step;
	int ret;

	lockdep_assert_held(&dcp->lock);
	if (!dcp->gated)
		return 0;
	step = "PMGR";
	ret = apple_pmgr_ps_consumer_on(dcp->dev);
	if (!ret) {
		step = "PMP";
		ret = apple_pmp_j713_display_get(dcp->dev);
	}
	if (!ret) {
		step = "PMC";
		ret = apple_pmc_interface_consumer_on(dcp->dev);
	}
	if (!ret) {
		step = "DART";
		ret = apple_dart_dcp_power_on(dcp->dev);
	}
	if (!ret)
		ret = apple_dart_dcp_power_on(&dcp->piodma->dev);
	if (!ret) {
		step = "RTKit wake";
		ret = apple_rtkit_wake_retained(dcp->rtkit, iomfb_m4_cpu_start, dcp);
	}
	if (ret)
		goto fail;
	WRITE_ONCE(dcp->gated, false);

	step = "A414";
	ret = dcp_m4_runtime_call(r, 414, false, prepare, sizeof(prepare),
				  prepared, sizeof(prepared));
	if (!ret && get_unaligned_le32(prepared + 4))
		ret = -EREMOTEIO;
	if (!ret) {
		step = "A412";
		ret = dcp_m4_runtime_call(r, 412, false, NULL, 0, &main_display,
					  sizeof(main_display));
	}
	if (!ret && le32_to_cpu(main_display) != 1)
		ret = -EREMOTEIO;
	if (ret)
		goto fail;
	dcp->gate_on_us = ktime_us_delta(ktime_get(), start);
	return 0;
fail:
	dev_err(dcp->dev, "display power-up failed at %s: %d\n", step, ret);
	dcp_m4_failed(dcp, ret);
	return ret;
}

static void iomfb_m4_gate_work(struct work_struct *work)
{
	struct iomfb_m4 *dcp = container_of(to_delayed_work(work), struct iomfb_m4, gate_work);

	mutex_lock(&dcp->lock);
	/*
	 * Still blanked and not yet down: a frame since then cancels it, and so
	 * does turning the power-down off meanwhile (power_gate_ms = 0).
	 */
	if (dcp->runtime && dcp->display_off && !dcp->gated && !READ_ONCE(dcp->error) &&
	    !READ_ONCE(dcp->owner->quiescing) && READ_ONCE(power_gate_ms))
		iomfb_m4_power_gate(dcp);
	mutex_unlock(&dcp->lock);
}

/* A364: the firmware's record of a timing and colour pair. */
struct dcp_m4_mode_info_req {
	u8 mode[0x64];
	u8 mode_null;
	u8 padding0[3];
	__le64 opaque;
	__le32 timing_mode_id;
	__le32 color_mode_id;
	u8 result_null;
	u8 padding1[3];
} __packed;
static_assert(sizeof(struct dcp_m4_mode_info_req) == 0x7c);

#define DCP_M4_MODE_INFO_REPLY 0x74

/*
 * The colour element of a timing: RGB with the SDR gamma EOTF, not virtual,
 * not unsafe for the timing, at 8 bpc if the timing lists it as needing DSC;
 * the lowest depth offered (8 bpc if there is one); at that depth one that
 * needs no DSC over one that does, then sRGB/BT.709 colorimetry and full
 * range over others, then the higher score. Never PQ or HLG.
 */
struct dcp_m4_ext_pick {
	struct dcp_color_element best;
	bool found;
};

static unsigned int dcp_m4_ext_color_rank(const struct dcp_color_element *e)
{
	return (e->colorimetry == DCP_COLORIMETRY_SRGB ||
		e->colorimetry == DCP_COLORIMETRY_BT709 ? 2 : 0) +
	       (e->range == DCP_COLOR_YCBCR_RANGE_FULL ? 1 : 0);
}

static void dcp_m4_ext_pick_color(void *data, const struct dcp_color_element *e)
{
	struct dcp_m4_ext_pick *p = data;
	unsigned int rank, best_rank;

	if (e->is_virtual || e->unsafe || e->id < 0 || e->id > U32_MAX || e->score < 0 ||
	    e->depth <= 0 || e->encoding != DCP_COLOR_FORMAT_RGB ||
	    e->eotf != DCP_EOTF_SDR_GAMMA || (e->dsc_required && e->depth != 8))
		return;
	rank = dcp_m4_ext_color_rank(e);
	best_rank = p->found ? dcp_m4_ext_color_rank(&p->best) : 0;
	if (p->found && (e->depth > p->best.depth ||
			 (e->depth == p->best.depth &&
			  (e->dsc_required > p->best.dsc_required ||
			   (e->dsc_required == p->best.dsc_required &&
			    (rank < best_rank ||
			     (rank == best_rank && e->score <= p->best.score)))))))
		return;
	p->best = *e;
	p->found = true;
}

static bool dcp_m4_ext_timing_color(const void *blob, u32 size, u32 timing,
				    struct dcp_color_element *out)
{
	struct dcp_m4_ext_pick p = {};
	bool interlaced = false;
	int ret = dcp_parse_timing_colors(blob, size, timing, dcp_m4_ext_pick_color, &p,
					  &interlaced);

	if (!ret && interlaced)
		return false;
	if (!ret && p.found)
		*out = p.best;
	return !ret && p.found;
}

/*
 * The timings of the sink's TimingElements @blob with an
 * 8-bpc RGB SDR colour element (see dcp_m4_ext_pick_color()), not
 * interlaced, for its DRM connector; those whose only such element needs DSC
 * are marked so. Replaces the previous list and starts
 * the sink's catalog (no PreferredTimingElements yet); not if received in
 * sink generation @gen, before a released route (stale). True if cached.
 */
static bool dcp_m4_ext_cache_modes(struct iomfb_m4 *dcp, const void *blob, u32 size, int gen)
{
	struct iomfb_m4_ext_mode *list = NULL, *old;
	struct dcp_display_mode *modes = NULL;
	struct dcp_parse_ctx ctx;
	unsigned int count = 0, kept = 0;
	int ret = parse(blob, size, &ctx);

	if (!ret) {
		/* parse() resets the context: the tracing pointer goes in afterwards. */
		ctx.dcp = dcp->owner;
		modes = enumerate_modes(&ctx, &count, 0, 0, 0, false, false);
		if (IS_ERR(modes)) {
			ret = PTR_ERR(modes);
			modes = NULL;
		}
	}
	if (!ret && count) {
		list = kcalloc(count, sizeof(*list), GFP_KERNEL);
		if (!list)
			ret = -ENOMEM;
	}
	for (unsigned int i = 0; !ret && i < count; i++) {
		const struct dcp_display_mode *m = &modes[i];
		struct dcp_color_element color;

		if ((m->mode.flags & (DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK)) ||
		    !dcp_m4_ext_timing_color(blob, size, m->timing_mode_id, &color) ||
		    color.depth != 8)
			continue;
		drm_mode_copy(&list[kept].mode, &m->mode);
		/* The connector's preferred mode is chosen at get_modes. */
		list[kept].mode.type &= ~DRM_MODE_TYPE_PREFERRED;
		list[kept].timing = m->timing_mode_id;
		list[kept].color = color.id;
		list[kept].bootstrap = m->has_bootstrap ? m->bootstrap_color_id : 0;
		list[kept].fw_preferred = m->firmware_preferred;
		list[kept].score = m->score;
		list[kept].dsc = color.dsc_required;
		if (color.dsc_required)
			dev_dbg(dcp->dev, "timing %u %ux%u@%d: DSC colour %u\n",
				m->timing_mode_id, m->mode.hdisplay, m->mode.vdisplay,
				drm_mode_vrefresh(&m->mode), (u32)color.id);
		kept++;
	}
	kfree(modes);
	if (ret) {
		kfree(list);
		list = NULL;
		kept = 0;
	}
	mutex_lock(&dcp->ext_modes_lock);
	if (gen != atomic_read(&dcp->ext_sink_gen)) {
		mutex_unlock(&dcp->ext_modes_lock);
		kfree(list);
		dev_dbg(dcp->dev, "TimingElements %s: not cached\n",
			gen == INT_MIN ? "not all in one sink generation" :
			"from before a released route");
		return false;
	}
	old = dcp->ext_modes;
	dcp->ext_modes = list;
	dcp->ext_mode_count = kept;
	dcp->ext_catalog_open = true;
	dcp->ext_pref_count = 0;
	mutex_unlock(&dcp->ext_modes_lock);
	kfree(old);
	/* A replay waiting for the sink's modes (iomfb_m4_ext_relink_wait()). */
	wake_up_all(&dcp->ext_settle_wq);
	dev_dbg(dcp->dev, "%u of %u timing(s) with an 8-bpc RGB SDR colour: %d\n",
		kept, count, ret);
	return true;
}

/*
 * The sink's PreferredTimingElements (their timing IDs and sizes), for
 * iomfb_m4_ext_get_modes(): only into the catalog of the sink's own
 * TimingElements (one cached since the sink last went), and not if received in
 * sink generation @gen, before a released route. True if they changed.
 */
static bool dcp_m4_ext_cache_preferred(struct iomfb_m4 *dcp, const void *blob, u32 size,
				       int gen)
{
	struct dcp_timing_id pref[ARRAY_SIZE(dcp->ext_pref)] = {};
	int n = dcp_parse_timing_ids(blob, size, pref, ARRAY_SIZE(pref));
	bool changed;

	if (n < 0) {
		dev_dbg(dcp->dev, "PreferredTimingElements not read: %d\n", n);
		n = 0;
	}
	mutex_lock(&dcp->ext_modes_lock);
	if (gen != atomic_read(&dcp->ext_sink_gen) || !dcp->ext_catalog_open) {
		mutex_unlock(&dcp->ext_modes_lock);
		dev_dbg(dcp->dev, "PreferredTimingElements %s: not cached\n",
			gen == INT_MIN ? "not all in one sink generation" :
			gen != atomic_read(&dcp->ext_sink_gen) ?
			"from before a released route" : "without this sink's TimingElements");
		return false;
	}
	changed = dcp->ext_pref_count != n ||
		  memcmp(dcp->ext_pref, pref, n * sizeof(pref[0]));
	memcpy(dcp->ext_pref, pref, sizeof(pref));
	dcp->ext_pref_count = n;
	mutex_unlock(&dcp->ext_modes_lock);
	for (int i = 0; i < n; i++)
		dev_dbg(dcp->dev, "sink's preferred timing %u: %ux%u%s\n", pref[i].id,
			pref[i].width, pref[i].height, changed ? "" : " (unchanged)");
	return changed;
}

/*
 * The sink went: in the firmware's own order (D575(0)), or with its
 * route (@route), which runs beside the firmware's messages: then words
 * already received publish nothing more (a new sink generation). Its
 * PreferredTimingElements go, and none are taken until its next
 * TimingElements. The mode list stays until then.
 */
static void dcp_m4_ext_sink_gone(struct iomfb_m4 *dcp, bool route)
{
	mutex_lock(&dcp->ext_modes_lock);
	if (route)
		atomic_inc(&dcp->ext_sink_gen);
	dcp->ext_catalog_open = false;
	dcp->ext_pref_count = 0;
	mutex_unlock(&dcp->ext_modes_lock);
}

/* One top-level call of an external output's work, with a deadline of its own. */
static int dcp_m4_ext_call(struct dcp_m4_runtime *r, unsigned int method, u8 stream,
			   const void *input, u32 input_size, void *output, u32 output_size,
			   unsigned int timeout_ms)
{
	struct dcp_m4_transaction *t = &r->dcp->transaction;
	int ret;

	/*
	 * The transaction latches any call it refuses, which ends the
	 * session: check its rules here and only fail this call.
	 */
	if (method > 999 || stream > 1 || t->calls || t->callbacks || t->error) {
		dev_warn(r->dcp->dev, "A%03u on stream %u not sent (calls %u, callbacks %u, error %d)\n",
			 method, stream, t->calls, t->callbacks, t->error);
		return -EINVAL;
	}
	r->deadline = jiffies + msecs_to_jiffies(timeout_ms);
	ret = dcp_m4_runtime_call_stream(r, method, stream, false, input, input_size,
					 output, output_size);
	r->deadline = 0;
	/* The sink may have gone meanwhile (D575, handled inline): stop there. */
	if (!ret && !r->ext_connected) {
		dev_dbg(r->dcp->dev, "the sink went during A%03u\n", method);
		ret = -ENOLINK;
	}
	return ret;
}

/* A364 then A413 for one timing and colour pair: the firmware's status, or an error. */
static int dcp_m4_ext_set_mode(struct dcp_m4_runtime *r, const char *what, u32 timing,
			       u32 color, u32 *status)
{
	struct iomfb_m4 *dcp = r->dcp;
	struct dcp_m4_mode_info_req info;
	struct {
		__le32 color_mode_id;
		__le32 timing_mode_id;
	} __packed mode = { cpu_to_le32(color), cpu_to_le32(timing) };
	u8 reply[DCP_M4_MODE_INFO_REPLY], out[4];
	int ret;

	memset(&info, 0xaa, sizeof(info));
	memset(info.mode, 0, sizeof(info.mode));
	info.mode_null = 0;
	info.timing_mode_id = cpu_to_le32(timing);
	info.color_mode_id = cpu_to_le32(color);
	info.result_null = 0;
	memset(info.padding1, 0, sizeof(info.padding1));
	memset(reply, 0, sizeof(reply));
	ret = dcp_m4_ext_call(r, 364, 0, &info, sizeof(info), reply, sizeof(reply), 5000);
	if (ret) {
		dev_warn(dcp->dev, "%s: A364 mode info (timing %u, color %u): %d\n",
			 what, timing, color, ret);
		return ret;
	}
	memset(out, 0, sizeof(out));
	/* The firmware itself gives a modeset about 8 s. */
	ret = dcp_m4_ext_call(r, 413, 0, &mode, sizeof(mode), out, sizeof(out), 10000);
	*status = get_unaligned_le32(out);
	if (ret || *status)
		dev_warn(dcp->dev, "%s: A413 set_digital_out_mode (color %u, timing %u): %d, status %#x\n",
			 what, color, timing, ret, *status);
	return ret;
}

/*
 * The display power-on before the modeset, in the desktop 26.x external
 * order where this firmware has the calls: the digital-out state (A414, a
 * query), power state 1 (A473), then display device 2 (A411).
 */
static int dcp_m4_ext_power_on(struct dcp_m4_runtime *r)
{
	static const u8 state_req[8] = { 0, 0, 0, 0, 0, 0xaa, 0xaa, 0xaa };
	struct dcp_m4_power_state_req req = { .state = cpu_to_le64(1) };
	struct dcp_m4_power_state_reply reply = {};
	struct iomfb_m4 *dcp = r->dcp;
	__le32 handle = cpu_to_le32(2);
	u8 out[8] = {};
	int ret;

	ret = dcp_m4_ext_call(r, 414, 0, state_req, sizeof(state_req), out, sizeof(out), 5000);
	dev_dbg(dcp->dev, "A414 digital-out state: %d, %8ph\n", ret, out);
	if (ret)
		return ret;
	ret = dcp_m4_ext_call(r, 473, 1, &req, sizeof(req), &reply, sizeof(reply), 5000);
	if (ret)
		return ret;
	if (reply.status) {
		dev_warn(dcp->dev, "power-on refused (status %#x): no modeset\n",
			 le32_to_cpu(reply.status));
		return -EIO;
	}
	memset(out, 0, sizeof(out));
	ret = dcp_m4_ext_call(r, 411, 0, &handle, sizeof(handle), out, 4, 5000);
	dev_dbg(dcp->dev, "A411 set_display_device(2): %d, answer %u\n",
		ret, get_unaligned_le32(out));
	return ret;
}

/*
 * Whether the display DART's SID0 may take a host surface now, under the D201
 * rules: the firmware has the DART on (its last D572), its state was not found
 * lost since, PIODMA is usable and SID0 is adopted and not retired. 0 if so.
 */
static int dcp_m4_ext_scanout_ok(struct iomfb_m4 *dcp, const char *why)
{
	int ret;

	if (READ_ONCE(dcp->ext_dart_lost))
		ret = -EIO;
	else
		ret = dcp_m4_ext_piodma(dcp, why);
	if (!ret)
		ret = apple_dart_dcp_dispext_stream_ok(&dcp->piodma->dev, 0);
	if (ret)
		dev_dbg(dcp->dev, "%s: SID0 not usable: %d%s\n", why, ret,
			READ_ONCE(dcp->ext_dart_lost) ? " (display DART state lost)" : "");
	return ret;
}

/*
 * Whether the display DART's SID0 may be used for a host surface now, as
 * dcp_m4_ext_scanout_ok() but quiet (every frame).
 */
static int dcp_m4_ext_scanout_gate(struct iomfb_m4 *dcp)
{
	int ret;

	if (dcp->ext_dart_since_sleep)
		dev_dbg_ratelimited(dcp->dev, "the display DART not powered up since sleep: frames wait\n");
	if (READ_ONCE(dcp->ext_dart_lost) || READ_ONCE(dcp->ext_dart_down) || !dcp->piodma)
		return -EIO;
	/* Both streams: a failure of either frontend retires the DART. */
	ret = apple_dart_dcp_dispext_stream_ok(&dcp->piodma->dev, 0);
	/*
	 * Last, with the DART on as of the last D572: the session lock is
	 * held, and only its holder handles D572, so it stays on until the
	 * caller's DART access is done (no callback is handled in between):
	 * PIODMA's and SID0's registers as adopted, no fault pending.
	 */
	return ret ?: apple_dart_dcp_dispext_qualify(&dcp->piodma->dev);
}

/*
 * Two host-written XRGB8888 surfaces in the display DART's SID0, each allocated
 * once for the largest mode (3840x2160) and reused for every mode, so that
 * mode changes never use up the SID0 arena. Both are stale: the next two
 * frames are copied whole.
 */
static int dcp_m4_ext_drm_buffers(struct dcp_m4_runtime *r)
{
	struct iomfb_m4 *dcp = r->dcp;
	u32 width = r->ext_width, height = r->ext_height;
	u32 stride = ALIGN(width * 4, 64);
	size_t size = PAGE_ALIGN((size_t)ALIGN(3840 * 4, 64) * 2160);
	int ret;

	if (!width || !height || width > 3840 || height > 2160)
		return -EINVAL;
	ret = dcp_m4_ext_scanout_ok(dcp, "DRM buffers");
	if (ret)
		return ret;
	for (unsigned int i = 0; i < 2; i++) {
		struct dcp_m4_frame *f = &r->ext_buf[i];

		if (!r->ext_buf_cpu[i] || f->backing_size < (u64)stride * height) {
			u64 dva = 0;
			void *cpu = apple_dart_dcp_dispext_scanout_alloc(&dcp->piodma->dev, size,
									 &dva);

			dev_dbg(dcp->dev, "copy surface %u: %zu bytes in SID0 at IOVA %#llx: %d\n",
				i, size, dva, PTR_ERR_OR_ZERO(cpu));
			if (IS_ERR(cpu))
				return PTR_ERR(cpu);
			r->ext_buf_cpu[i] = cpu;
			f->dva = dva;
			f->backing_size = size;
		}
		f->width = width;
		f->height = height;
		f->stride = stride;
		f->format = DCP_M4_XRGB8888;
		ret = dcp_m4_frame_check_ext(f) ?:
			dcp_m4_direct_extent(f->dva, f->backing_size, (u64)stride * height);
		if (ret)
			return ret;
		r->ext_buf_stale[i] = true;
	}
	r->ext_prev_damage = DRM_RECT_INIT(0, 0, width, height);
	return 0;
}

/*
 * The route and sink the DRM output may drive: the session up, the sink
 * connected (its last D575) and its route not released since (ext_sink, which
 * a released route clears while ext_connected, the firmware's own view, stays
 * as its last D575 left it).
 */
static bool dcp_m4_ext_route_ready(struct iomfb_m4 *dcp, struct dcp_m4_runtime *r)
{
	return !READ_ONCE(dcp->error) && r->ext_connected && READ_ONCE(dcp->ext_sink);
}

/*
 * Whether @ids (from the atomic check) are still those of a @width x @height
 * mode of the current sink's catalog that fits the link.
 */
static bool dcp_m4_ext_ids_current(struct iomfb_m4 *dcp, const struct iomfb_m4_ext_ids *ids,
				   u32 width, u32 height)
{
	u64 capacity = dcp_m4_ext_capacity(dcp->owner);
	bool found = false;

	mutex_lock(&dcp->ext_modes_lock);
	for (unsigned int i = 0; !found && dcp->ext_catalog_open && i < dcp->ext_mode_count; i++) {
		const struct iomfb_m4_ext_mode *e = &dcp->ext_modes[i];

		found = e->timing == ids->timing && e->color == ids->color &&
			e->bootstrap == ids->bootstrap && e->mode.hdisplay == width &&
			e->mode.vdisplay == height && dcp_m4_ext_fits(e, capacity);
	}
	mutex_unlock(&dcp->ext_modes_lock);
	return found;
}

/*
 * DRM: before an enable or disable, which must not be refused: the direct
 * swaps in flight complete (D590) or are given up (5 s, or at once when the
 * link goes). The session lock is dropped while waiting, so those can run;
 * nothing here holds a DRM lock they need. -EBUSY if it has not after 10 s.
 */
static int dcp_m4_ext_settle(struct iomfb_m4 *dcp, struct dcp_m4_runtime *r, const char *what)
{
	unsigned long end = jiffies + msecs_to_jiffies(10000);

	lockdep_assert_held(&dcp->lock);
	while (r->ext_qn) {
		long left = (long)(end - jiffies);

		if (left <= 0) {
			dev_warn(dcp->dev, "%s: %u direct swaps still under way after 10 s\n", what,
				 r->ext_qn);
			return -EBUSY;
		}
		dev_dbg(dcp->dev, "%s waits for the %u direct swaps in flight\n", what, r->ext_qn);
		mutex_unlock(&dcp->lock);
		wait_event_timeout(dcp->ext_settle_wq, !READ_ONCE(r->ext_qn), left);
		mutex_lock(&dcp->lock);
	}
	return 0;
}

/*
 * A DRM modeset of an external output: the display power-on (A414, A473 1,
 * A411) once per link, the mode (A364/A413, the bootstrap colour first if
 * there is one) when it changes, then its two SID0 surfaces. Under the session
 * lock; a sink that goes ends it.
 */
int iomfb_m4_ext_enable(struct apple_dcp *owner, const struct iomfb_m4_ext_ids *ids,
			u32 width, u32 height)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;
	const char *why = "";
	int link_gen, sink_gen;
	u32 status = 0;
	int ret = -ENOLINK;

	if (!dcp || !dcp->external)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	r = dcp->runtime;
	/*
	 * During a sleep (the DRM helper's resume) no call goes out: the DRM
	 * state is replayed after it (t8132_ext_drm.c).
	 */
	if (r && r->ext_sleeping) {
		ret = 0;
		why = "during sleep: replayed after it";
		goto out;
	}
	/* Pairs with the release store of r->done in startup. */
	if (!r || !smp_load_acquire(&r->done) || !dcp_m4_ext_route_ready(dcp, r))
		goto out;
	/* The link and sink this enable is for, to tell if they changed meanwhile. */
	link_gen = atomic_read(&r->ext_link_gen);
	sink_gen = atomic_read(&dcp->ext_sink_gen);
	/* Never refused: the direct swaps in flight end first. */
	ret = dcp_m4_ext_settle(dcp, r, "enable");
	if (ret)
		goto out;
	/* A sleep that began during the wait has the display down: no call. */
	if (r->ext_sleeping) {
		why = "a sleep began meanwhile: replayed after it";
		goto out;
	}
	/*
	 * After the wait, the same link and sink, still ready, and the
	 * mode one of this sink's: else no call goes out, the output is stale.
	 */
	ret = -ENOLINK;
	if (!dcp_m4_ext_route_ready(dcp, r) || atomic_read(&r->ext_link_gen) != link_gen ||
	    atomic_read(&dcp->ext_sink_gen) != sink_gen) {
		why = "the link or sink changed or is not ready";
		goto out;
	}
	ret = -ESTALE;
	if (!dcp_m4_ext_ids_current(dcp, ids, width, height)) {
		why = "the mode is not one of the current sink's";
		goto out;
	}
	ret = 0;
	if (!READ_ONCE(r->ext_powered)) {
		ret = dcp_m4_ext_power_on(r);
		if (ret)
			goto out;
		WRITE_ONCE(r->ext_powered, true);
	}
	if (READ_ONCE(r->ext_cur_timing) != ids->timing || r->ext_cur_color != ids->color) {
		WRITE_ONCE(r->ext_cur_timing, 0);
		if (ids->bootstrap) {
			ret = dcp_m4_ext_set_mode(r, "DRM bootstrap", ids->timing, ids->bootstrap,
						  &status);
			if (!ret && status)
				ret = -EIO;
		}
		if (!ret)
			ret = dcp_m4_ext_set_mode(r, "DRM mode", ids->timing, ids->color, &status);
		if (!ret && status)
			ret = -EIO;
		if (ret)
			goto out;
		r->ext_cur_color = ids->color;
		WRITE_ONCE(r->ext_cur_timing, ids->timing);
	}
	r->ext_width = width;
	r->ext_height = height;
	/* After a swap given up on a live link, one swap may try (a probe). */
	if (r->ext_hold_id && !r->ext_hold_probe)
		dev_dbg(dcp->dev, "enable: swaps held for swap %u: one may try\n",
			r->ext_hold_id);
	r->ext_hold_probe = !!r->ext_hold_id;
	/* Without the display DART now (D572(0)), the first frame prepares them. */
	r->ext_buf_pending = true;
	ret = dcp_m4_ext_drm_buffers(r);
	if (!ret)
		r->ext_buf_pending = false;
	else if (dcp_m4_ext_scanout_gate(dcp))
		ret = 0;
out:
	/*
	 * DRM takes the mode as set whatever this returns: after a
	 * failure no frame goes out (with the old mode) until an enable works.
	 */
	if (r)
		r->ext_mode_stale = !!ret;
	mutex_unlock(&dcp->lock);
	if (ret)
		dev_warn(dcp->dev, "enable %ux%u (timing %u, colour %u, bootstrap %u) failed: %d%s%s%s%s; no frames until an enable succeeds\n",
			 width, height, ids->timing, ids->color, ids->bootstrap, ret,
			 status ? " (the firmware refused the mode)" : "",
			 *why ? " (" : "", why, *why ? ")" : "");
	else if (*why)
		dev_dbg(dcp->dev, "enable %ux%u: %s\n", width, height, why);
	return ret;
}

/*
 * Whether the synchronous swap just completed was displayed, as its D590
 * says; logged if not, for @what.
 */
static bool dcp_m4_ext_sync_shown(struct dcp_m4_runtime *r, const char *what)
{
	if (r->ext_swap_shown)
		return true;
	dev_dbg(r->dcp->dev, "%s: swap %u completed, not displayed: the frame on screen stays\n",
		what, r->swap.id);
	return false;
}

/*
 * A swap of surface @shown (-1: surfaceless) completed,
 * D590 included: that surface is the one on screen, the other one free;
 * every surface a given-up swap might have put on screen is retired by it
 * (those IDs stay until their D590, which is then only dropped), and a
 * hold ends: the pipe completes swaps again.
 */
static void dcp_m4_ext_retired(struct dcp_m4_runtime *r, int shown)
{
	if (r->ext_hold_id)
		dev_info(r->dcp->dev, "a swap completed: the hold for swap %u ends\n",
			 r->ext_hold_id);
	r->ext_hold_id = 0;
	r->ext_hold_probe = false;
	r->ext_buf_lost[0] = 0;
	r->ext_buf_lost[1] = 0;
	r->ext_front = shown;
	if (shown >= 0)
		r->ext_back = shown ^ 1;
	/* No window is on screen, nor may a given-up swap show one. */
	for (int i = 0; i < DCP_M4_EXT_WINDOWS; i++)
		r->ext_win[i].on_screen = false;
	for (unsigned int i = 0; i < ARRAY_SIZE(r->ext_lost_swap); i++)
		r->ext_lost_swap[i].win = -1;
}

/* Rows of @rect from @src (@pitch) into @dst (@stride); its pixels. */
static u64 dcp_m4_ext_copy(void *dst, u32 stride, const struct iosys_map *src, u32 pitch,
			   const struct drm_rect *rect)
{
	size_t len = (size_t)(rect->x2 - rect->x1) * 4;

	if (!drm_rect_visible(rect))
		return 0;
	for (int y = rect->y1; y < rect->y2; y++) {
		size_t offset = (size_t)y * pitch + rect->x1 * 4;
		void *to = dst + (size_t)y * stride + rect->x1 * 4;

		if (src->is_iomem)
			memcpy_fromio(to, src->vaddr_iomem + offset, len);
		else
			memcpy(to, src->vaddr + offset, len);
	}
	return (u64)drm_rect_width(rect) * drm_rect_height(rect);
}

/*
 * One DRM frame: the damaged rows of @src (XRGB8888 or ARGB8888, @pitch bytes a
 * row, at the mode's size) copied into the back surface, together with what
 * that surface missed of the previous frame, then a one-surface swap of it
 * (A406/A407, D590). A stale surface gets the whole frame. Nothing is copied or
 * swapped without a mode, a sink or the display DART (its last D572).
 */
int iomfb_m4_ext_present(struct apple_dcp *owner, const struct iosys_map *src, u32 pitch,
			 const struct drm_rect *damage)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;
	struct drm_rect full, rect, cur;
	struct dcp_m4_frame *f;
	unsigned int back;
	void *cpu;
	int ret = -ENOLINK;

	if (!dcp || !dcp->external)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	r = dcp->runtime;
	/* Pairs with the release store of r->done in startup. */
	if (!r || !smp_load_acquire(&r->done) || READ_ONCE(dcp->error) || !r->ext_connected ||
	    !READ_ONCE(r->ext_cur_timing))
		goto out;
	/*
	 * Surfaces an enable could not prepare (D572(0) then): now, once the
	 * display DART is on, PIODMA included. Only a DART still off retries.
	 */
	if (r->ext_buf_pending) {
		ret = READ_ONCE(dcp->ext_dart_down) ? -EAGAIN : dcp_m4_ext_drm_buffers(r);
		if (ret != -EAGAIN || !READ_ONCE(dcp->ext_dart_down))
			r->ext_buf_pending = false;
		if (ret)
			goto out;
	}
	ret = dcp_m4_ext_scanout_gate(dcp);
	if (ret)
		goto out;
	/*
	 * After a swap given up on a live link, nothing until its D590,
	 * except the one probe a DRM enable allows; a confirmed swap ends the
	 * hold (see below).
	 */
	/* After a failed enable, never a frame with the old mode. */
	ret = -ESTALE;
	if (r->ext_mode_stale)
		goto out;
	ret = -EBUSY;
	if ((r->ext_hold_id && !r->ext_hold_probe) || r->ext_orphan_start ||
	    r->ext_qn || r->ext_sleeping)
		goto out;
	r->ext_hold_probe = false;
	/*
	 * Never the surface on screen, nor one a given-up swap may still
	 * put there; with neither free, a confirmed surfaceless swap retires
	 * them first.
	 */
	back = r->ext_back;
	if (back == r->ext_front || r->ext_buf_lost[back]) {
		back ^= 1;
		r->ext_buf_stale[back] = true;
	}
	if (back == r->ext_front || r->ext_buf_lost[back]) {
		r->ext_swap_surface = -1;
		ret = dcp_m4_runtime_swap_full(r, true, NULL, NULL, 0, false, 0, 0, 0);
		dev_dbg(dcp->dev, "no free copy surface (on screen %d, uncertain swaps %u/%u): surfaceless swap first: %d\n",
			back ^ 1, r->ext_buf_lost[0], r->ext_buf_lost[1], ret);
		if (ret)
			goto out;
		/* Not displayed: no surface was freed. */
		ret = -EBUSY;
		if (!dcp_m4_ext_sync_shown(r, "copy"))
			goto out;
		dcp_m4_ext_retired(r, -1);
		r->ext_buf_stale[0] = true;
		r->ext_buf_stale[1] = true;
		back = r->ext_back;
	}
	f = &r->ext_buf[back];
	cpu = r->ext_buf_cpu[back];
	ret = -ENODEV;
	if (!cpu || f->width != r->ext_width || f->height != r->ext_height ||
	    pitch < f->width * 4)
		goto out;
	full = DRM_RECT_INIT(0, 0, f->width, f->height);
	cur = *damage;
	if (!drm_rect_intersect(&cur, &full))
		cur = DRM_RECT_INIT(0, 0, 0, 0);
	/*
	 * This frame's damage and what the back surface missed of the previous
	 * one: as one rectangle where they overlap, else each on its own (a
	 * bounding box of two distant damages would copy most of the frame).
	 */
	if (r->ext_buf_stale[back]) {
		rect = full;
		dcp_m4_ext_copy(cpu, f->stride, src, pitch, &rect);
	} else {
		rect = r->ext_prev_damage;
		if (drm_rect_visible(&cur) && drm_rect_visible(&rect)) {
			struct drm_rect both = rect;

			if (drm_rect_intersect(&both, &cur)) {
				rect.x1 = min(rect.x1, cur.x1);
				rect.y1 = min(rect.y1, cur.y1);
				rect.x2 = max(rect.x2, cur.x2);
				rect.y2 = max(rect.y2, cur.y2);
				dcp_m4_ext_copy(cpu, f->stride, src, pitch, &rect);
			} else {
				dcp_m4_ext_copy(cpu, f->stride, src, pitch, &rect);
				dcp_m4_ext_copy(cpu, f->stride, src, pitch, &cur);
			}
		} else {
			if (!drm_rect_visible(&rect))
				rect = cur;
			dcp_m4_ext_copy(cpu, f->stride, src, pitch, &rect);
		}
	}
	/* Only through the Normal-NC view; nothing else maps these pages. */
	dma_wmb();
	r->ext_swap_surface = back;
	ret = dcp_m4_runtime_swap_full(r, true, f, NULL, 0, false, 0, 0, 0);
	r->ext_swap_surface = -1;
	if (ret) {
		r->ext_buf_stale[back] = true;
		goto out;
	}
	r->ext_buf_stale[back] = false;
	r->ext_prev_damage = cur;
	/*
	 * Not displayed: what was on screen stays, this surface the back one; the
	 * other one never got this frame's damage, so it takes a whole frame.
	 */
	if (!dcp_m4_ext_sync_shown(r, "copy")) {
		r->ext_buf_stale[back ^ 1] = true;
		goto out;
	}
	dcp_m4_ext_retired(r, back);
	/* A frame shown, for the replay after a sleep. */
	if (r->ext_restore_pending)
		WRITE_ONCE(r->ext_restore_frame, true);
out:
	mutex_unlock(&dcp->lock);
	return ret;
}

/* A framebuffer reference a window let go of. */
struct dcp_m4_ext_put {
	struct list_head link;
	void *key;
	void (*put)(void *key);
};

/* Drop @key's reference later, outside the session lock. */
static void dcp_m4_ext_defer_put(struct iomfb_m4 *dcp, void *key, void (*put)(void *key))
{
	struct dcp_m4_ext_put *p = kmalloc(sizeof(*p), GFP_KERNEL);
	unsigned long flags;

	if (!p) {
		dev_warn(dcp->dev, "a framebuffer reference is kept (no memory)\n");
		return;
	}
	p->key = key;
	p->put = put;
	spin_lock_irqsave(&dcp->ext_put_lock, flags);
	list_add_tail(&p->link, &dcp->ext_puts);
	spin_unlock_irqrestore(&dcp->ext_put_lock, flags);
	schedule_work(&dcp->ext_put_work);
}

static void iomfb_m4_ext_put_work(struct work_struct *work)
{
	struct iomfb_m4 *dcp = container_of(work, struct iomfb_m4, ext_put_work);
	struct dcp_m4_ext_put *p, *n;
	unsigned long flags;
	LIST_HEAD(puts);

	spin_lock_irqsave(&dcp->ext_put_lock, flags);
	list_splice_init(&dcp->ext_puts, &puts);
	spin_unlock_irqrestore(&dcp->ext_put_lock, flags);
	list_for_each_entry_safe(p, n, &puts, link) {
		p->put(p->key);
		kfree(p);
	}
}

/*
 * Whether window @i may be unmapped: it maps a framebuffer that no swap shows
 * (as last confirmed), may still show (a given-up one) or has going out or in
 * flight, and its unmap never failed.
 */
static bool dcp_m4_ext_win_idle(struct dcp_m4_runtime *r, int i)
{
	const struct dcp_m4_ext_window *w = &r->ext_win[i];

	if (!w->key || w->on_screen || w->pinned || r->ext_swap_win == i ||
	    dcp_m4_ext_win_lost(r, i))
		return false;
	for (unsigned int j = 0; j < r->ext_qn; j++)
		if (r->ext_q[j].win == i)
			return false;
	return true;
}

/*
 * Unmap window @i (idle, see above) and let its framebuffer go. The caller ran
 * dcp_m4_ext_scanout_gate() under the session lock with no callback handled
 * since. A failed unmap keeps the window and its framebuffer until reboot.
 */
static int dcp_m4_ext_win_release(struct dcp_m4_runtime *r, int i)
{
	struct iomfb_m4 *dcp = r->dcp;
	struct dcp_m4_ext_window *w = &r->ext_win[i];
	int ret = apple_dart_dcp_dispext_window_unmap(&dcp->piodma->dev, w->dva);

	/* The DART not confirmed since sleep: nothing was done, try later. */
	if (ret == -EAGAIN)
		return ret;
	if (ret < 0) {
		w->pinned = true;
		dev_err(dcp->dev, "window %d at DVA %#llx: unmap failed: %d: kept with its framebuffer until reboot\n",
			i, w->dva, ret);
		return ret;
	}
	dcp_m4_ext_defer_put(dcp, w->key, w->put);
	w->key = NULL;
	w->put = NULL;
	return 0;
}

/*
 * Let go of the idle windows unused for @age jiffies (0: all of them), if the
 * DART may be touched now (the gate runs last).
 */
static int dcp_m4_ext_win_release_idle(struct dcp_m4_runtime *r, unsigned long age)
{
	bool gated = false;
	int ret;

	/* 0, or the first failure: a failed unmap is a hardware error (the DART is retired). */
	for (int i = 0; i < DCP_M4_EXT_WINDOWS; i++) {
		if (!dcp_m4_ext_win_idle(r, i) ||
		    (age && time_before(jiffies, r->ext_win[i].used + age)))
			continue;
		if (!gated) {
			ret = dcp_m4_ext_scanout_gate(r->dcp);
			if (ret)
				return ret;
		}
		gated = true;
		ret = dcp_m4_ext_win_release(r, i);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * The window that maps @fb: the one already mapping it, else an empty one
 * (claimed now if need be), else the least recently used idle one, let go of
 * first. @fb's pages are mapped there, its reference taken before. -ENOSPC
 * without a window, -EOPNOTSUPP for pages that do not cover @need bytes. The
 * caller ran dcp_m4_ext_scanout_gate() under the session lock with no callback
 * handled since.
 */
static int dcp_m4_ext_win_get(struct dcp_m4_runtime *r, const struct iomfb_m4_ext_fb *fb,
			      size_t need)
{
	struct iomfb_m4 *dcp = r->dcp;
	struct device *piodma = &dcp->piodma->dev;
	struct dcp_m4_ext_window *w;
	phys_addr_t *phys = NULL;
	unsigned int count = 0;
	int pick = -1, ret;

	for (int i = 0; i < DCP_M4_EXT_WINDOWS; i++)
		if (r->ext_win[i].key == fb->key) {
			r->ext_win[i].used = jiffies;
			return i;
		}
	for (int i = 0; pick < 0 && i < DCP_M4_EXT_WINDOWS; i++)
		if (r->ext_win[i].dva && !r->ext_win[i].key && !r->ext_win[i].pinned)
			pick = i;
	/*
	 * A new slice only once the copy surfaces have theirs: the copy
	 * fallback must never find the arena used up by windows.
	 */
	for (int i = 0; pick < 0 && r->ext_buf_cpu[0] && r->ext_buf_cpu[1] &&
	     i < DCP_M4_EXT_WINDOWS; i++) {
		u64 dva;

		if (r->ext_win[i].dva)
			continue;
		ret = apple_dart_dcp_dispext_window_claim(piodma, &dva);
		/* Not confirmed since sleep: the frame waits (no window, no copy). */
		if (ret == -EAGAIN)
			return ret;
		if (ret)
			break;
		r->ext_win[i].dva = dva;
		dev_dbg(dcp->dev, "window %d claimed at DVA %#llx\n", i, dva);
		pick = i;
	}
	if (pick < 0) {
		for (int i = 0; i < DCP_M4_EXT_WINDOWS; i++)
			if (dcp_m4_ext_win_idle(r, i) &&
			    (pick < 0 || time_before(r->ext_win[i].used, r->ext_win[pick].used)))
				pick = i;
		if (pick < 0)
			return -ENOSPC;
		/* A failed unmap is a hardware error, not a reason to copy. */
		ret = dcp_m4_ext_win_release(r, pick);
		if (ret)
			return ret;
	}
	w = &r->ext_win[pick];
	ret = fb->pages(fb->key, &phys, &count);
	if (ret)
		return -EOPNOTSUPP;
	if ((size_t)count * SZ_16K < need || count > SZ_32M / SZ_16K) {
		kfree(phys);
		return -EOPNOTSUPP;
	}
	/* The reference first: a failed mapping keeps the pages until reboot. */
	fb->get(fb->key);
	w->key = fb->key;
	w->put = fb->put;
	w->count = count;
	w->used = jiffies;
	ret = apple_dart_dcp_dispext_window_map(piodma, w->dva, phys, count);
	kfree(phys);
	/* Not confirmed since sleep: nothing was mapped; the window stays empty. */
	if (ret == -EAGAIN) {
		dcp_m4_ext_defer_put(dcp, w->key, w->put);
		w->key = NULL;
		w->put = NULL;
		return ret;
	}
	if (ret) {
		w->pinned = true;
		dev_err(dcp->dev, "window %d: mapping %u pages failed: %d: kept with its framebuffer until reboot\n",
			pick, count, ret);
		return ret;
	}
	return pick;
}

/* Take the direct swap at @i out of flight. */
static void dcp_m4_ext_q_remove(struct dcp_m4_runtime *r, int i)
{
	r->ext_qn--;
	memmove(&r->ext_q[i], &r->ext_q[i + 1], (r->ext_qn - i) * sizeof(r->ext_q[0]));
	if (!r->ext_qn)
		cancel_delayed_work(&r->ext_async_work);
	wake_up_all(&r->dcp->ext_settle_wq);
}

/*
 * The direct swap @e out of flight (completed or given up): its flip if it
 * waited for that, and the oldest one's if that waited for an older one.
 */
static void dcp_m4_ext_flip_after(struct dcp_m4_runtime *r, const struct dcp_m4_ext_inflight *e)
{
	bool send = e->flip_held;

	if (r->ext_qn && r->ext_q[0].flip_held) {
		r->ext_q[0].flip_held = false;
		send = true;
	}
	if (send)
		t8132_ext_drm_flip_done(r->dcp->dev);
}

/*
 * What the D590 of direct swap @id (or a later one naming it) means for the
 * screen, taken account of as it comes: @win its window (-1: none), @older how
 * many direct swaps in flight are older. SHOWN: its window is the one on
 * screen, every other window and the copy surfaces retired; the older ones
 * complete as KEPT then. KEPT: its window counts as on screen too, nothing
 * else is retired. UNSHOWN: what was on screen stays, and its window is free.
 * The copy surfaces go stale and a hold ends either way. Session lock held.
 */
static void dcp_m4_ext_shown(struct dcp_m4_runtime *r, u32 id, int win, int older,
			     enum dcp_m4_ext_end end)
{
	if (end == DCP_M4_EXT_END_SHOWN) {
		dcp_m4_ext_retired(r, -1);
		for (int j = 0; j < older; j++)
			r->ext_q[j].superseded = true;
	} else {
		if (end == DCP_M4_EXT_END_UNSHOWN)
			dev_dbg(r->dcp->dev, "swap %u completed, not displayed: the frame on screen stays\n",
				id);
		if (r->ext_hold_id)
			dev_info(r->dcp->dev, "a swap completed: the hold for swap %u ends\n",
				 r->ext_hold_id);
		r->ext_hold_id = 0;
		r->ext_hold_probe = false;
	}
	if (end != DCP_M4_EXT_END_UNSHOWN) {
		if (win >= 0)
			r->ext_win[win].on_screen = true;
		/* A frame shown, for the replay after a sleep. */
		if (r->ext_restore_pending)
			WRITE_ONCE(r->ext_restore_frame, true);
	}
	r->ext_buf_stale[0] = true;
	r->ext_buf_stale[1] = true;
}

/*
 * The direct swap in flight at @i is done, A407 reply and D590 in, and leaves
 * flight; older ones not completed stay. @end as dcp_m4_ext_shown(), SHOWN
 * as KEPT for one superseded; TAKEN: its D590 came before its A407 reply and
 * was taken account of then. Its flip, or the next one's. Session lock held.
 */
static void dcp_m4_ext_async_done(struct dcp_m4_runtime *r, int i, enum dcp_m4_ext_end end)
{
	struct dcp_m4_ext_inflight e = r->ext_q[i];

	if (end == DCP_M4_EXT_END_SHOWN && e.superseded)
		end = DCP_M4_EXT_END_KEPT;
	e.swap.completed = true;
	if (dcp_m4_swap_take_completion(&e.swap) != 1)
		dev_warn_ratelimited(r->dcp->dev, "direct swap %u: completion not taken\n",
				     e.swap.id);
	if (i)
		dev_warn_ratelimited(r->dcp->dev, "swap %u completed before the older swap %u, which stays in flight\n",
				     e.swap.id, r->ext_q[0].swap.id);
	dcp_m4_ext_q_remove(r, i);
	if (end != DCP_M4_EXT_END_TAKEN)
		dcp_m4_ext_shown(r, e.swap.id, e.win, i, end);
	dcp_m4_ext_flip_after(r, &e);
}

/* Whether the oldest direct swap in flight is due its give-up, and its link went (@went). */
static bool dcp_m4_ext_async_due(struct dcp_m4_runtime *r, bool *went)
{
	const struct dcp_m4_ext_inflight *e = &r->ext_q[0];

	*went = !r->ext_connected || atomic_read(&r->ext_link_gen) != e->gen;
	return *went || !time_before(jiffies, e->deadline);
}

/*
 * Give the oldest direct swap in flight up, as a waiting swap would be (its
 * window may still show), its link gone (@went) or its time out; its flip goes
 * as lost (or the next one's, dcp_m4_ext_flip_after()). Session lock held.
 */
static void dcp_m4_ext_async_expire(struct dcp_m4_runtime *r, bool went)
{
	struct iomfb_m4 *dcp = r->dcp;
	struct dcp_m4_ext_inflight e = r->ext_q[0];
	int ret = went ? -ENOLINK : -ETIMEDOUT;

	r->ext_swap_surface = -1;
	/* The copy surfaces never got this frame: whole frames, whatever its late D590 says. */
	r->ext_buf_stale[0] = true;
	r->ext_buf_stale[1] = true;
	if (!READ_ONCE(dcp->error) &&
	    !dcp_m4_ext_swap_give_up(r, &e.swap, e.win, e.superseded, ret)) {
		dcp_m4_swap_fail(&r->swap, ret);
		dcp_m4_transaction_fail(&dcp->transaction, ret);
		dcp_m4_failed(dcp, ret);
	}
	dcp_m4_ext_q_remove(r, 0);
	dcp_m4_ext_flip_after(r, &e);
}

/*
 * The oldest direct swap in flight at its deadline, or after its link went
 * (D575(0), a released route): words already received are taken first (its
 * D590 among them maybe); then each one due is given up as a waiting swap
 * would be (dcp_m4_ext_swap_give_up(): its window may still show, and is
 * kept), and its flip is sent as lost. The work comes back at the next one's
 * deadline.
 */
static void dcp_m4_ext_async_work(struct work_struct *work)
{
	struct dcp_m4_runtime *r = container_of(to_delayed_work(work), struct dcp_m4_runtime,
						ext_async_work);
	struct iomfb_m4 *dcp = r->dcp;
	bool went;
	int ret;

	mutex_lock(&dcp->lock);
	if (r->ext_qn && dcp_m4_ext_async_due(r, &went)) {
		ret = READ_ONCE(dcp->error) ?: dcp_m4_runtime_drain(r, 32);
		if (ret < 0) {
			dcp_m4_transaction_fail(&dcp->transaction, ret);
			dcp_m4_failed(dcp, ret);
		}
		while (r->ext_qn && dcp_m4_ext_async_due(r, &went))
			dcp_m4_ext_async_expire(r, went);
	}
	if (r->ext_qn) {
		long left = (long)(r->ext_q[0].deadline - jiffies);

		queue_delayed_work(r->wq, &r->ext_async_work, max(left, 0L));
	}
	mutex_unlock(&dcp->lock);
}

/*
 * One DRM frame scanned out directly
 * from its framebuffer, mapped into a SID0 host window (kept per
 * framebuffer, see dcp_m4_ext_win_get()), with a one-surface swap that
 * returns after its A407 reply. Up to t8132_ext_swap_depth swaps in flight:
 * -EBUSY while that many are, checked before any A406. 1 once submitted with
 * none older in flight and a depth over 1: its flip now; 0 once submitted
 * otherwise: its flip is sent with the older one's completion, or its own
 * (D590), or as lost when that is given up. -ENOSPC without a window,
 * -EOPNOTSUPP for a framebuffer that does not fit the mode or the window: the
 * caller copies it instead.
 */
int iomfb_m4_ext_present_direct(struct apple_dcp *owner, const struct iomfb_m4_ext_fb *fb)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;
	struct dcp_m4_frame f;
	int ret = -ENOLINK, w;
	unsigned int depth, free;
	size_t need;

	if (!dcp || !dcp->external)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	r = dcp->runtime;
	/* Pairs with the release store of r->done in startup. */
	if (!r || !smp_load_acquire(&r->done) || READ_ONCE(dcp->error) || !r->ext_connected ||
	    !READ_ONCE(r->ext_cur_timing))
		goto out;
	/* After a failed enable, never a frame with the old mode. */
	ret = -ESTALE;
	if (r->ext_mode_stale)
		goto out;
	ret = -EBUSY;
	depth = clamp_val(READ_ONCE(t8132_ext_swap_depth), 1, DCP_M4_EXT_INFLIGHT);
	if ((r->ext_hold_id && !r->ext_hold_probe) || r->ext_orphan_start ||
	    r->ext_qn >= depth || r->ext_sleeping)
		goto out;
	/*
	 * A slot for each swap in flight, this one included, that a later D590
	 * may complete (dcp_m4_runtime_swap_done()); else frames are copied.
	 */
	ret = -ENOSPC;
	free = 0;
	for (unsigned int i = 0; i < ARRAY_SIZE(r->ext_early); i++)
		free += !r->ext_early[i];
	if (free <= r->ext_qn) {
		dev_warn_ratelimited(dcp->dev, "%u swaps completed by a later D590 still without their own: frames copied\n",
				     (unsigned int)ARRAY_SIZE(r->ext_early) - free);
		goto out;
	}
	ret = -EOPNOTSUPP;
	need = PAGE_ALIGN((size_t)fb->pitch * fb->height);
	if (fb->width != r->ext_width || fb->height != r->ext_height ||
	    fb->pitch < fb->width * 4 || fb->pitch % 64 || need > SZ_32M)
		goto out;
	/* Last before the DART is touched; no callback is handled until the swap. */
	ret = dcp_m4_ext_scanout_gate(dcp);
	if (ret)
		goto out;
	/*
	 * The copy surfaces an enable deferred (D572(0) then), before any
	 * window takes a slice: the copy fallback keeps its own.
	 */
	if (r->ext_buf_pending) {
		ret = dcp_m4_ext_drm_buffers(r);
		r->ext_buf_pending = !!ret;
		if (ret)
			goto out;
	}
	ret = dcp_m4_ext_win_release_idle(r, 2 * HZ);
	if (ret)
		goto out;
	w = dcp_m4_ext_win_get(r, fb, need);
	if (w < 0) {
		ret = w;
		goto out;
	}
	f = (struct dcp_m4_frame) {
		.dva = r->ext_win[w].dva, .backing_size = (u64)r->ext_win[w].count * SZ_16K,
		.width = fb->width, .height = fb->height, .stride = fb->pitch,
		.format = DCP_M4_XRGB8888,
	};
	r->ext_hold_probe = false;
	r->ext_swap_surface = -1;
	r->ext_swap_win = w;
	r->ext_swap_async = depth;
	r->ext_swap_now = false;
	ret = dcp_m4_runtime_swap_full(r, true, &f, NULL, 0, false, 0, 0, 0);
	r->ext_swap_async = 0;
	r->ext_swap_win = -1;
	if (!ret && r->ext_swap_now)
		ret = 1;
out:
	mutex_unlock(&dcp->lock);
	return ret;
}

/*
 * A DRM disable: a surfaceless swap in the default background, which takes the
 * surfaces off; the power and the mode stay (A473 0 is not sent). Nothing to
 * do without a mode or a sink.
 */
int iomfb_m4_ext_blank(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;
	int ret = 0;

	if (!dcp || !dcp->external)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	r = dcp->runtime;
	/* During a sleep the display is down already. */
	if (r && r->ext_sleeping) {
		mutex_unlock(&dcp->lock);
		return 0;
	}
	/* Never refused: the direct swaps in flight end first. */
	/* Pairs with the release store of r->done in startup. */
	if (r && smp_load_acquire(&r->done)) {
		int link_gen = atomic_read(&r->ext_link_gen);
		int sink_gen = atomic_read(&dcp->ext_sink_gen);

		ret = dcp_m4_ext_settle(dcp, r, "disable");
		/* A sleep that began during the wait has the display down. */
		if (!ret && r->ext_sleeping) {
			mutex_unlock(&dcp->lock);
			return 0;
		}
		/* A link or sink gone meanwhile is not blanked: nothing to send to. */
		if (!ret && (atomic_read(&r->ext_link_gen) != link_gen ||
			     atomic_read(&dcp->ext_sink_gen) != sink_gen ||
			     !dcp_m4_ext_route_ready(dcp, r))) {
			dev_dbg(dcp->dev, "disable: the link or sink went: no surfaceless swap\n");
			r = NULL;
		}
	}
	/* Pairs with the release store of r->done in startup. */
	if (r && !ret && smp_load_acquire(&r->done) && dcp_m4_ext_route_ready(dcp, r) &&
	    READ_ONCE(r->ext_cur_timing)) {
		/* Not into a pipe that holds swaps; its surfaces stay as they are. */
		r->ext_swap_surface = -1;
		ret = r->ext_hold_id || r->ext_orphan_start ? -EBUSY :
			dcp_m4_runtime_swap_full(r, true, NULL, NULL, 0, false, 0, 0, 0);
		if (ret == -EBUSY)
			dev_info(dcp->dev, "disable: swaps held or a start orphaned: the last frame stays\n");
		/* Only a displayed blank takes the windows off screen. */
		if (!ret && dcp_m4_ext_sync_shown(r, "disable")) {
			dcp_m4_ext_retired(r, -1);
			/* The windows are off screen: let them go. */
			if (dcp_m4_ext_win_release_idle(r, 0))
				dev_dbg(dcp->dev, "disable: idle windows not let go of\n");
		}
		r->ext_buf_stale[0] = true;
		r->ext_buf_stale[1] = true;
		dev_dbg(dcp->dev, "disable: surfaceless swap: %d\n", ret);
	}
	mutex_unlock(&dcp->lock);
	return ret;
}

/*
 * PM_SUSPEND_PREPARE, under every modeset lock with no commit on its CRTC
 * pending (t8132_ext_drm.c): the output enters a system sleep: frames are
 * refused from now on (their flips sent a frame later), enables and disables
 * do nothing until iomfb_m4_ext_sleep_wake(). True if it did (an external DCP
 * with a live session).
 */
bool iomfb_m4_ext_sleep_mark(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;
	bool entered = false;

	if (!dcp || !dcp->external)
		return false;
	mutex_lock(&dcp->lock);
	r = dcp->runtime;
	/* Pairs with the release store of r->done in startup. */
	if (r && smp_load_acquire(&r->done) && !READ_ONCE(dcp->error)) {
		r->ext_sleeping = true;
		/* Whether any suspend comes before the wake (the replay's DART). */
		r->ext_restore_epoch = atomic_read(&dcp->ext_pm_epoch);
		entered = true;
	}
	mutex_unlock(&dcp->lock);
	return entered;
}

/*
 * PM_SUSPEND_PREPARE, before the tasks freeze, every worker running, after
 * iomfb_m4_ext_sleep_mark(): the display down for the sleep: the swaps in
 * flight settle (their D590, or their give-up); a surfaceless swap; every
 * window let go of; the display powered off (A473 0) and its firmware's
 * D572(0) waited for (1 s), so that the DCP's suspend finds the display DART
 * off, else it refuses the sleep. The DRM state replayed after the sleep
 * powers on and sets the mode from scratch.
 */
void iomfb_m4_ext_sleep_down(struct apple_dcp *owner)
{
	struct dcp_m4_power_state_req req = { .state = cpu_to_le64(0) };
	struct dcp_m4_power_state_reply reply = {};
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;
	int ret = 0, mapped = 0, left = 0;

	if (!dcp || !dcp->external)
		return;
	mutex_lock(&dcp->lock);
	r = dcp->runtime;
	if (!r || !r->ext_sleeping) {
		mutex_unlock(&dcp->lock);
		return;
	}
	dev_dbg(dcp->dev, "sleep: display going down (display DART %s, %u swaps in flight, oldest %u)\n",
		READ_ONCE(dcp->ext_dart_down) ? "off" : "on", r->ext_qn,
		r->ext_qn ? r->ext_q[0].swap.id : 0);
	/* The workers run: the swaps in flight complete or are given up as ever. */
	ret = dcp_m4_ext_settle(dcp, r, "sleep");
	if (!ret && dcp_m4_ext_route_ready(dcp, r) && READ_ONCE(r->ext_cur_timing)) {
		r->ext_swap_surface = -1;
		ret = r->ext_hold_id || r->ext_orphan_start ? -EBUSY :
			dcp_m4_runtime_swap_full(r, true, NULL, NULL, 0, false, 0, 0, 0);
		if (ret)
			dev_warn(dcp->dev, "sleep: surfaceless swap failed: %d\n", ret);
		/* Only a displayed blank takes the windows off screen. */
		if (!ret && dcp_m4_ext_sync_shown(r, "sleep")) {
			dcp_m4_ext_retired(r, -1);
			for (int i = 0; i < DCP_M4_EXT_WINDOWS; i++)
				mapped += !!r->ext_win[i].key;
			if (dcp_m4_ext_win_release_idle(r, 0))
				dev_dbg(dcp->dev, "sleep: idle windows not let go of\n");
			for (int i = 0; i < DCP_M4_EXT_WINDOWS; i++)
				left += !!r->ext_win[i].key;
			dev_dbg(dcp->dev, "sleep: windows released: %d of %d\n",
				mapped - left, mapped);
		}
		r->ext_buf_stale[0] = true;
		r->ext_buf_stale[1] = true;
	}
	if (!ret && READ_ONCE(r->ext_powered) && r->ext_connected) {
		ret = dcp_m4_ext_call(r, 473, 1, &req, sizeof(req), &reply, sizeof(reply), 5000);
		dev_dbg(dcp->dev, "sleep: A473 power state 0: %d, value %u, status %#x\n",
			ret, le32_to_cpu(reply.value), le32_to_cpu(reply.status));
		/* Its D572(0), nested in it or after it (the idle worker), 1 s at most. */
		if (!ret && !READ_ONCE(dcp->ext_dart_down)) {
			mutex_unlock(&dcp->lock);
			wait_event_timeout(dcp->ext_settle_wq,
					   READ_ONCE(dcp->ext_dart_down) || READ_ONCE(dcp->error),
					   msecs_to_jiffies(1000));
			mutex_lock(&dcp->lock);
		}
		if (!READ_ONCE(dcp->ext_dart_down))
			dev_warn(dcp->dev, "sleep: the display DART not powered down within 1 s of the power-off (%d): the DCP refuses to suspend\n",
				 ret);
	}
	/* After the sleep the display is powered on and its mode set from scratch. */
	WRITE_ONCE(r->ext_powered, false);
	WRITE_ONCE(r->ext_cur_timing, 0);
	mutex_unlock(&dcp->lock);
}

/*
 * PM_POST_SUSPEND, under every modeset lock, before the DRM state is
 * replayed: the output leaves the sleep; its next enable
 * powers on and sets the mode, its next frames go out again.
 */
void iomfb_m4_ext_sleep_wake(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;

	if (!dcp || !dcp->external)
		return;
	mutex_lock(&dcp->lock);
	if (dcp->runtime && dcp->runtime->ext_sleeping) {
		struct dcp_m4_runtime *r = dcp->runtime;
		/*
		 * The replay starts, no frame of it shown yet. Without any
		 * suspend since PREPARE (a sleep refused or aborted before the
		 * DCPs), the DART never left confirmation: on as it is now.
		 */
		bool kept = atomic_read(&dcp->ext_pm_epoch) == r->ext_restore_epoch &&
			    !dcp->ext_dart_since_sleep && !READ_ONCE(dcp->ext_dart_down);

		r->ext_sleeping = false;
		r->ext_restore_pending = true;
		WRITE_ONCE(r->ext_restore_dart, kept);
		WRITE_ONCE(r->ext_restore_frame, false);
		r->ext_restore_link_gen = atomic_read(&r->ext_link_gen);
		r->ext_restore_sink_gen = atomic_read(&dcp->ext_sink_gen);
		if (kept)
			dev_dbg(dcp->dev, "sleep: no suspend since it began: the display DART stays confirmed\n");
	}
	mutex_unlock(&dcp->lock);
}

/*
 * PM_SUSPEND_PREPARE: the DRM layer ends a hold of this output's connector at
 * PM_POST_SUSPEND (iomfb_m4_ext_relink_end()), if this sleep starts one.
 */
void iomfb_m4_ext_relink_own(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;

	if (dcp && dcp->external)
		WRITE_ONCE(dcp->ext_relink_owned, true);
}

/*
 * A dcpext's display released for a deep sleep with the cable left in: it is
 * connected again after the wake, by the type-C work, in the background. Its
 * connector is held meanwhile only for a DRM layer that ends the hold
 * (iomfb_m4_ext_relink_own()).
 */
void iomfb_m4_ext_relink_expect(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;

	if (!dcp || !dcp->external || !READ_ONCE(dcp->ext_relink_owned))
		return;
	/* The link's capacity before it goes down: its modes meanwhile. */
	WRITE_ONCE(dcp->ext_hold_capacity, dcp_m4_ext_capacity(owner));
	WRITE_ONCE(dcp->ext_relink_armed, false);
	WRITE_ONCE(dcp->ext_relink_lost, false);
	WRITE_ONCE(dcp->ext_relink, true);
}

/*
 * After the wake, its reconnect queued, under the owner's hpd_mutex with
 * @cable read under it: from now on, the display found gone (the cable
 * already gone, or iomfb_m4_ext_relink_lost()) ends the replay's wait for it.
 */
void iomfb_m4_ext_relink_arm(struct apple_dcp *owner, bool cable)
{
	struct iomfb_m4 *dcp = owner->v26_6;

	lockdep_assert_held(&owner->hpd_mutex);
	if (!dcp || !dcp->external || !READ_ONCE(dcp->ext_relink))
		return;
	WRITE_ONCE(dcp->ext_relink_armed, true);
	if (!cable) {
		WRITE_ONCE(dcp->ext_relink_lost, true);
		wake_up_all(&dcp->ext_settle_wq);
	}
}

/*
 * Its display unplugged after the wake, under the owner's hpd_mutex as its
 * cable goes: nothing comes back to wait for.
 */
void iomfb_m4_ext_relink_lost(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;

	lockdep_assert_held(&owner->hpd_mutex);
	if (!dcp || !dcp->external || !READ_ONCE(dcp->ext_relink) ||
	    !READ_ONCE(dcp->ext_relink_armed))
		return;
	WRITE_ONCE(dcp->ext_relink_lost, true);
	wake_up_all(&dcp->ext_settle_wq);
}

/* Whether its connector is held for a reconnect after a deep sleep. */
bool iomfb_m4_ext_relinking(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;

	return dcp && dcp->external && READ_ONCE(dcp->ext_relink);
}

/*
 * Its display connected again, the sink's modes in; or found gone; or the
 * session failed.
 */
static bool dcp_m4_ext_relinked(struct iomfb_m4 *dcp)
{
	/* Pairs with the release store of dcp->runtime in startup. */
	struct dcp_m4_runtime *r = smp_load_acquire(&dcp->runtime);

	if (READ_ONCE(dcp->error) || READ_ONCE(dcp->ext_relink_lost))
		return true;
	/* Pairs with the release store of r->done in startup. */
	return r && smp_load_acquire(&r->done) && READ_ONCE(r->ext_connected) &&
	       READ_ONCE(dcp->ext_sink) && READ_ONCE(dcp->ext_catalog_open);
}

/*
 * PM_POST_SUSPEND, before the DRM replay: after iomfb_m4_ext_relink_expect(),
 * the display connected again (until @deadline at most, in jiffies), so that
 * the replay's enable finds a link instead of failing on one not up yet (and
 * leaving the compositor a hotplug to set the mode again); or found gone
 * (unplugged during the sleep), at once; or the session failed. Its
 * connector stays held: iomfb_m4_ext_relink_end() ends that.
 */
void iomfb_m4_ext_relink_wait(struct apple_dcp *owner, unsigned long deadline)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	long left = (long)(deadline - jiffies);

	if (!dcp || !dcp->external || !READ_ONCE(dcp->ext_relink) || left <= 0)
		return;
	wait_event_timeout(dcp->ext_settle_wq, dcp_m4_ext_relinked(dcp), left);
}

/*
 * PM_POST_SUSPEND after iomfb_m4_ext_relink_wait(), or a sleep refused at
 * PM_SUSPEND_PREPARE, or the DRM device going: the hold of its connector ends,
 * and the DRM layer's part in one with it. 0 if none was kept or the display
 * is back; -ENOLINK if it was found gone; the session's error if it failed;
 * -ETIMEDOUT if it is not back otherwise. The DRM layer sends the hotplug
 * held meanwhile once its replay is done (t8132_ext_drm.c).
 */
int iomfb_m4_ext_relink_end(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	int ret;

	if (!dcp || !dcp->external)
		return 0;
	WRITE_ONCE(dcp->ext_relink_owned, false);
	if (!READ_ONCE(dcp->ext_relink))
		return 0;
	WRITE_ONCE(dcp->ext_relink, false);
	if (READ_ONCE(dcp->ext_relink_lost))
		return -ENOLINK;
	ret = READ_ONCE(dcp->error);
	if (ret)
		return ret;
	return dcp_m4_ext_relinked(dcp) ? 0 : -ETIMEDOUT;
}

/* A replay ends unchecked (it failed, or nothing to replay). */
void iomfb_m4_ext_restore_end(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;

	if (!dcp || !dcp->external)
		return;
	mutex_lock(&dcp->lock);
	if (dcp->runtime)
		dcp->runtime->ext_restore_pending = false;
	mutex_unlock(&dcp->lock);
}

/*
 * PM_POST_SUSPEND, after the replay's commit of an output that is on (its
 * flips are done or given up by then): whether the
 * replay restored the output, its enable kept (not stale), a D572(1)
 * confirmed the display DART and a frame was shown since the wake; and,
 * now, the session alive, the DART on as of the last D572, its streams
 * confirmed and not retired nor its state lost, the link and the sink the
 * wake's. What is still missing is polled for @timeout_ms at most; the
 * replay ends here. 0, else why not, logged.
 */
int iomfb_m4_ext_restore_result(struct apple_dcp *owner, unsigned int timeout_ms)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	const char *why;
	int ret;

	if (!dcp || !dcp->external)
		return -ENODEV;
	for (;;) {
		struct dcp_m4_runtime *r;
		bool done;

		mutex_lock(&dcp->lock);
		r = dcp->runtime;
		ret = -EAGAIN;
		why = "";
		if (!r || !r->ext_restore_pending) {
			ret = -ENOLINK;
			why = ": no replay under way (the session went meanwhile)";
		} else if (READ_ONCE(dcp->error)) {
			ret = -EIO;
			why = ": the session failed";
		} else if (atomic_read(&r->ext_link_gen) != r->ext_restore_link_gen ||
			   atomic_read(&dcp->ext_sink_gen) != r->ext_restore_sink_gen) {
			ret = -ENOLINK;
			why = ": the link or sink changed since the wake";
		} else if (r->ext_mode_stale) {
			ret = -ESTALE;
			why = ": the enable failed";
		} else if (READ_ONCE(dcp->ext_dart_lost) || !dcp->piodma ||
			   apple_dart_dcp_dispext_stream_ok(&dcp->piodma->dev, 0) == -EIO) {
			ret = -EIO;
			why = ": the display DART is retired or its state lost";
		} else if (!READ_ONCE(r->ext_restore_dart) || READ_ONCE(dcp->ext_dart_down) ||
			   apple_dart_dcp_dispext_stream_ok(&dcp->piodma->dev, 0)) {
			why = ": the display DART not confirmed on since the wake";
		} else if (!READ_ONCE(r->ext_restore_frame)) {
			why = ": no frame shown";
		} else {
			ret = 0;
		}
		done = ret != -EAGAIN || time_after(jiffies, deadline);
		if (done && r)
			r->ext_restore_pending = false;
		mutex_unlock(&dcp->lock);
		if (done)
			break;
		msleep(20);
	}
	if (ret == -EAGAIN)
		ret = -ETIMEDOUT;
	if (ret)
		dev_warn(dcp->dev, "resume: restore not confirmed%s: %d\n", why, ret);
	else
		dev_info(dcp->dev, "resume: display restored\n");
	return ret;
}

/* Whether the last enable failed (no frame until one succeeds). */
bool iomfb_m4_ext_stale(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	bool stale;

	if (!dcp || !dcp->external)
		return false;
	mutex_lock(&dcp->lock);
	stale = dcp->runtime && dcp->runtime->ext_mode_stale;
	mutex_unlock(&dcp->lock);
	return stale;
}

/*
 * J713: why an external DCP's display cannot go down for system sleep, or
 * NULL: swaps held or a start orphaned (a stuck pipe), or a lost display
 * DART state. Anything else goes down before the freeze (t8132_ext_drm.c).
 */
const char *iomfb_m4_ext_sleep_veto(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;
	const char *why = NULL;

	if (!dcp || !dcp->external)
		return NULL;
	mutex_lock(&dcp->lock);
	r = dcp->runtime;
	if (r && (r->ext_hold_id || r->ext_orphan_start))
		why = "swaps are held or a start is orphaned";
	else if (READ_ONCE(dcp->ext_dart_lost))
		why = "the display DART state was lost";
	mutex_unlock(&dcp->lock);
	return why;
}

/* A map or unmap in flight closes a DART only briefly: wait for it. */
static int iomfb_m4_ext_dart_off(struct apple_dart_cio *dart)
{
	int ret;

	for (unsigned int i = 0; i < 20; i++) {
		ret = apple_dart_cio_off(dart);
		if (ret != -EBUSY)
			break;
		usleep_range(500, 1000);
	}
	return ret;
}

/*
 * A deep sleep takes an external DCP down whole, as its display has gone
 * already: the AP link is released, RTKit idles, the CPU stops, then its
 * display DART and its firmware DART are powered down. Its power domains are
 * the sleep's to lower. Firmware, endpoints and shared buffers stay for
 * iomfb_m4_ext_start(). Before the first step its providers are asked; a
 * refusal before RTKit idles changes nothing, and a failure after the CPU
 * stopped is undone in reverse. Either way the sleep is refused.
 */
static int iomfb_m4_ext_stop(struct iomfb_m4 *dcp)
{
	struct dcp_m4_runtime *r = dcp->runtime;
	bool prepared[2] = {}, rx_held = false, cpu_off = false, cpu_reset;
	unsigned long flags;
	__le32 zero = 0;
	const char *step = "DART owner";
	int ret, recover;

	lockdep_assert_held(&dcp->lock);
	if (dcp->ext_stopped || dcp->gated)
		return -EALREADY;
	if (!r || r->ext_qn || dcp->transaction.calls || !READ_ONCE(dcp->ext_dart_down))
		return -EBUSY;
	ret = dcp_m4_runtime_live(r);
	if (ret)
		return ret;
	/* Bound at probe: a device link cannot be added from a PM callback. */
	ret = dcp->ext_darts[0] && dcp->ext_darts[1] ? 0 : -ENODEV;
	if (!ret) {
		step = "PMGR";
		ret = apple_pmgr_ps_consumer_check(dcp->dev);
	}
	for (unsigned int i = 0; !ret && i < 2; i++) {
		step = "DART owner";
		ret = apple_dart_cio_prepare(dcp->ext_darts[i]);
		prepared[i] = !ret;
	}
	if (ret)
		goto unprepare;

	step = "A500";
	ret = dcp_m4_runtime_call(r, 500, false, &zero, sizeof(zero), NULL, 0);
	if (!ret) {
		step = "drain";
		ret = dcp_m4_runtime_drain(r, 64);
		if (ret > 0 || (!ret && r->count))
			ret = -EBUSY;
	}
	/* Nothing has changed yet: the session stays, the sleep is refused. */
	if (ret)
		goto unprepare;
	step = "RTKit idle";
	ret = apple_rtkit_idle(dcp->rtkit);
	if (ret)
		goto fail;
	apple_rtkit_hold_rx(dcp->rtkit);
	rx_held = true;
	step = "CPU stop";
	ret = iomfb_m4_cpu_run(dcp, false);
	if (ret)
		goto fail;
	step = "late message";
	ret = apple_rtkit_poll(dcp->rtkit);
	if (ret >= 0) {
		apple_rtkit_flush_rx(dcp->rtkit);
		spin_lock_irqsave(&dcp->state_lock, flags);
		ret = r->count ? -EBUSY : 0;
		spin_unlock_irqrestore(&dcp->state_lock, flags);
	}
	if (ret)
		goto fail;
	/* From here on nothing may use the DCP's or its DARTs' registers. */
	WRITE_ONCE(dcp->gated, true);
	dcp->ext_stopped = true;
	step = "display DART";
	ret = iomfb_m4_ext_dart_off(dcp->ext_darts[1]);
	if (!ret) {
		step = "firmware DART";
		ret = iomfb_m4_ext_dart_off(dcp->ext_darts[0]);
	}
	if (!ret) {
		/* Its CPU domain off: the CPU then starts from reset, as after the sleep. */
		step = "CPU domain";
		cpu_off = true;
		ret = apple_pmgr_ps_consumer_off(dcp->dev);
	}
	if (ret)
		goto unwind;
	/* The restarted firmware announces the sink's AV service anew. */
	spin_lock_irqsave(&dcp->state_lock, flags);
	dcp->ext_avserv = NULL;
	spin_unlock_irqrestore(&dcp->state_lock, flags);
	dev_info(dcp->dev, "external DCP stopped for sleep\n");
	return 0;
unwind:
	dev_err(dcp->dev, "external DCP stop failed at %s: %d; restarting it\n", step, ret);
	/* Only a CPU whose own domain went off starts from reset. */
	cpu_reset = cpu_off && apple_pmgr_ps_consumer_is_off(dcp->dev);
	recover = cpu_off ? apple_pmgr_ps_consumer_on(dcp->dev) : 0;
	recover = recover ?: apple_dart_cio_on(dcp->ext_darts[0]) ?:
		  apple_dart_cio_on(dcp->ext_darts[1]);
	if (!recover && cpu_reset) {
		/* Start it as after the sleep. */
		recover = apple_rtkit_wake_retained(dcp->rtkit, iomfb_m4_cpu_start, dcp);
	} else if (!recover) {
		/* A CPU only stopped resumes where it idled. */
		recover = iomfb_m4_cpu_run(dcp, true);
		apple_rtkit_resume_rx(dcp->rtkit);
		recover = recover ?: apple_rtkit_wake(dcp->rtkit);
	}
	for (unsigned int i = 0; i < 2; i++)
		recover = apple_dart_cio_finish(dcp->ext_darts[i]) ?: recover;
	dcp->ext_stopped = false;
	if (recover) {
		dev_err(dcp->dev, "external DCP restart failed: %d\n", recover);
		dcp_m4_failed(dcp, recover);
		return ret;
	}
	WRITE_ONCE(dcp->gated, false);
	return ret;
fail:
	dev_err(dcp->dev, "external DCP stop failed at %s: %d\n", step, ret);
	dcp_m4_failed(dcp, ret);
	/* Its CPU still runs, or was never left: the DARTs were not touched. */
	if (rx_held)
		apple_rtkit_resume_rx(dcp->rtkit);
	for (unsigned int i = 0; i < 2; i++)
		apple_dart_cio_finish(dcp->ext_darts[i]);
	return ret;
unprepare:
	dev_warn(dcp->dev, "external DCP stop refused at %s: %d\n", step, ret);
	for (unsigned int i = 0; i < 2; i++)
		if (prepared[i])
			apple_dart_cio_finish(dcp->ext_darts[i]);
	return -EBUSY;
}

/*
 * After the sleep: the firmware DART and the display DART as they were, then
 * the CPU from its reset vector on the firmware image it kept, through
 * RTKit's retained wake. The display comes back with its next enable. The
 * DART transactions end whatever happens; a DART that did not come back
 * stays closed, and the session is failed.
 */
static int iomfb_m4_ext_start(struct iomfb_m4 *dcp)
{
	const char *step = "CPU domain";
	int ret, finish;

	lockdep_assert_held(&dcp->lock);
	if (!dcp->ext_stopped)
		return 0;
	ret = READ_ONCE(dcp->error);
	if (!ret)
		ret = apple_pmgr_ps_consumer_on(dcp->dev);
	if (!ret) {
		step = "firmware DART";
		ret = apple_dart_cio_on(dcp->ext_darts[0]);
	}
	if (!ret) {
		step = "display DART";
		ret = apple_dart_cio_on(dcp->ext_darts[1]);
	}
	if (!ret) {
		step = "RTKit wake";
		ret = apple_rtkit_wake_retained(dcp->rtkit, iomfb_m4_cpu_start, dcp);
	}
	for (unsigned int i = 0; i < 2; i++) {
		finish = apple_dart_cio_finish(dcp->ext_darts[1 - i]);
		if (!ret && finish) {
			step = "DART owner";
			ret = finish;
		}
	}
	dcp->ext_stopped = false;
	if (ret) {
		dev_err(dcp->dev, "external DCP restart failed at %s: %d\n", step, ret);
		dcp_m4_failed(dcp, ret);
		return ret;
	}
	WRITE_ONCE(dcp->gated, false);
	dev_info(dcp->dev, "external DCP restarted after sleep\n");
	return 0;
}

/* An external DCP stopped for a deep sleep, its CPU to start from reset after it. */
bool iomfb_m4_ext_stopped(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	bool stopped;

	if (!dcp || !dcp->external)
		return false;
	mutex_lock(&dcp->lock);
	stopped = dcp->ext_stopped;
	mutex_unlock(&dcp->lock);
	return stopped;
}

/* apple-drm, a consumer of this device, has blanked the panel before this. */
int iomfb_m4_suspend(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	int ret;

	/*
	 * An external DCP with a session suspends only with its display DART
	 * off (the firmware's last D572 was 0) and nothing in flight; else the
	 * sleep is refused, and the outputs are restored after it.
	 */
	if (dcp && dcp->external && dcp->runtime && smp_load_acquire(&dcp->runtime->done)) {
		const char *why = NULL;

		mutex_lock(&dcp->lock);
		if (!READ_ONCE(dcp->ext_dart_down))
			why = "the display DART is on (last D572 was 1)";
		else if (dcp->runtime->ext_qn)
			why = "a swap is under way";
		mutex_unlock(&dcp->lock);
		if (why) {
			dev_warn(dcp->dev, "refusing system sleep: %s\n", why);
			return -EBUSY;
		}
	}
	if (dcp && dcp->external) {
		/*
		 * Every message RTKit received so far is dispatched, and stamped
		 * with the old epoch, before the epoch moves on: its receive work
		 * takes no lock held here (only state_lock, then queue_work).
		 */
		if (dcp->rtkit)
			apple_rtkit_flush_rx(dcp->rtkit);
		mutex_lock(&dcp->lock);
		atomic_inc(&dcp->ext_pm_epoch);
		WRITE_ONCE(dcp->ext_dart_down, true);
		dcp->ext_dart_since_sleep = true;
		ret = 0;
		/* A deep sleep powers it off: stop it first, if it has a session. */
		if (pm_suspend_target_state == PM_SUSPEND_MEM && dcp->runtime) {
			/* Pairs with the release store of r->done in startup. */
			bool session = smp_load_acquire(&dcp->runtime->done);

			if (session)
				ret = iomfb_m4_ext_stop(dcp);
		}
		mutex_unlock(&dcp->lock);
		/*
		 * The PM core resumes no device whose suspend failed: end the
		 * sleep's epoch here, as resume would, or no later powerUpDART
		 * counts as fresh and the output stays dark.
		 */
		if (ret) {
			if (dcp->rtkit)
				apple_rtkit_flush_rx(dcp->rtkit);
			atomic_inc(&dcp->ext_pm_epoch);
		}
		return ret;
	}
	if (!dcp || !dcp->runtime)
		return -ENODEV;
	cancel_delayed_work_sync(&dcp->gate_work);
	mutex_lock(&dcp->lock);
	/* Also the startup surface, shown before the first frame */
	if (dcp->active_fb || dcp->active_frame.dva) {
		dev_warn(dcp->dev, "panel still scanning out: refusing system sleep\n");
		ret = -EBUSY;
	} else if (dcp->display_off) {
		/*
		 * Already powered down by the blank; the next frame powers it up.
		 * Power the DCP and its domains down too, if it may: a deep sleep
		 * needs that whatever power_gate_ms says, or its display DARTs
		 * refuse it, and elsewhere it is an extra saving. It is not a
		 * condition of this step: a power-down that cannot start leaves the
		 * display logically off, and one that fails has failed the session
		 * already; neither refuses the sleep here.
		 */
		ret = 0;
		if (dcp->gate_capable && !dcp->gated &&
		    (READ_ONCE(power_gate_ms) || pm_suspend_via_firmware()))
			iomfb_m4_power_gate(dcp);
	} else {
		ret = dcp_m4_set_power_state(dcp, false);
		if (!ret)
			dcp->powered_off = true;
	}
	mutex_unlock(&dcp->lock);
	return ret;
}

int iomfb_m4_resume(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;
	int ret;

	/*
	 * Messages received from now on are of this epoch (see suspend);
	 * every one RTKit received before is dispatched with the old one first.
	 */
	if (dcp && dcp->external) {
		mutex_lock(&dcp->lock);
		ret = iomfb_m4_ext_start(dcp);
		mutex_unlock(&dcp->lock);
		if (dcp->rtkit)
			apple_rtkit_flush_rx(dcp->rtkit);
		atomic_inc(&dcp->ext_pm_epoch);
		return ret;
	}
	if (!dcp || !dcp->runtime)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	if (dcp->gated) {
		/* Stays down until the next frame. */
		ret = 0;
	} else if (dcp->powered_off) {
		ret = dcp_m4_set_power_state(dcp, true);
		if (!ret)
			dcp->powered_off = false;
	} else {
		ret = dcp_m4_runtime_health(dcp->runtime);
		if (ret)
			dcp_m4_failed(dcp, ret);
	}
	mutex_unlock(&dcp->lock);
	return ret;
}

static void iomfb_m4_backlight_work(struct work_struct *work)
{
	struct iomfb_m4 *dcp = container_of(work, struct iomfb_m4, backlight_work);
	int ret;

	mutex_lock(&dcp->lock);
	if (READ_ONCE(dcp->owner->quiescing) || !dcp->runtime ||
	    !dcp_backlight_pending(dcp->owner) || !dcp->active_frame.dva)
		goto out;
	ret = dcp_m4_runtime_live(dcp->runtime);
	if (!ret)
		ret = iomfb_m4_swap(dcp, &dcp->active_frame);
	if (ret) {
		dcp_m4_failed(dcp, ret);
	} else {
		dcp->brightness_swaps++;
		iomfb_m4_idle_cache_kick(dcp);
	}
out:
	mutex_unlock(&dcp->lock);
}

static void iomfb_m4_backlight_kick(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;

	if (dcp && dcp->runtime && !READ_ONCE(owner->quiescing))
		queue_work(dcp->runtime->wq, &dcp->backlight_work);
}

void iomfb_m4_configure_backlight(struct apple_dcp *owner)
{
	int ret;

	if (dcp_backlight_active(owner))
		return;
	ret = dcp_backlight_configure(owner, 500, false, 0, true, 120,
				      iomfb_m4_backlight_kick);
	if (ret && ret != -EBUSY)
		dev_err(owner->dev, "firmware 26.6.2 backlight unavailable: %d\n", ret);
}

void iomfb_m4_quiesce(struct apple_dcp *owner)
{
	struct iomfb_m4 *dcp = owner->v26_6;

	if (!dcp)
		return;
	/* Wake any RPC waiter before waiting for workers; never recycle its DMA. */
	dcp_m4_failed(dcp, -ENODEV);
	cancel_work_sync(&dcp->backlight_work);
	if (dcp->runtime)
		flush_workqueue(dcp->runtime->wq);
	/* After the flush, as drained callbacks can queue it. */
	cancel_delayed_work_sync(&dcp->idle_cache);
	cancel_delayed_work_sync(&dcp->gate_work);
	cancel_delayed_work_sync(&dcp->ext_edid_retry);
}

static int iomfb_m4_status_show(struct seq_file *s, void *unused)
{
	struct apple_dcp *owner = s->private;
	struct iomfb_m4 *dcp = owner->v26_6;
	struct dcp_m4_runtime *r;
	int health;

	if (!dcp)
		return -ENODEV;
	mutex_lock(&dcp->lock);
	r = dcp->runtime;
	health = r ? dcp_m4_runtime_health(r) : -ENODEV;
	if (health && r)
		dcp_m4_failed(dcp, health);
	seq_printf(s, "firmware=26.6.2 ready=%u qualified=%u error=%d run=%u health=%d\n",
		   dcp->ready, r && r->qualified, dcp->error,
		   !dcp->gated && !!(readl(dcp->coproc + DCP_M4_CPU_CONTROL) & DCP_M4_CPU_RUN),
		   health);
	seq_printf(s, "gate_capable=%u gated=%u gate_cycles=%llu gate_off_us=%lld gate_on_us=%lld\n",
		   dcp->gate_capable, dcp->gated, dcp->gate_cycles, dcp->gate_off_us,
		   dcp->gate_on_us);
	seq_printf(s, "startup=%u background=%u power=%u scanout=%u callbacks=%u\n",
		   r && r->done, r && r->background_done, r && r->power_done,
		   r && r->scanout_done, r ? r->callbacks : 0);
	seq_printf(s, "flips=%llu blanks=%llu backlight_swaps=%llu active=%u uncertain=%u\n",
		   dcp->flips, dcp->blanks, dcp->brightness_swaps,
		   !!dcp->active_fb, !!dcp->uncertain_fb);
	seq_printf(s, "display_off=%u off_cycles=%llu off_us=%lld/%lld on_us=%lld/%lld\n",
		   dcp->display_off, dcp->display_off_cycles,
		   dcp->display_off_us, dcp->display_off_us_max,
		   dcp->display_on_us, dcp->display_on_us_max);
	if (r)
		seq_printf(s, "idle_cache=%u state=%u fences=%u registered=%u entries=%llu\n",
			   !r->idle_unavailable && READ_ONCE(idle_cache_ms), r->idle_state,
			   r->idle_fences, r->idle_registered, r->idle_entries);
	mutex_unlock(&dcp->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(iomfb_m4_status);

void iomfb_m4_debugfs_init(struct apple_dcp *owner, struct dentry *root)
{
	debugfs_create_file("iomfb_m4", 0400, root, owner, &iomfb_m4_status_fops);
}
