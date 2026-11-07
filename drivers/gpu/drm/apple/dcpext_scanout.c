// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Local T6030 external first-pixel diagnostic. No automatic scanout startup. */

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dma-map-ops.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/workqueue.h>
#include <drm/drm_drv.h>
#include <drm/drm_probe_helper.h>

#include "dcp.h"
#include "dcpext_mode.h"
#include "dcpext_scanout.h"
#include "dcpext_drm.h"
#include "ibootep.h"

#define SCANOUT_WIDTH DCPEXT_WIDTH
#define SCANOUT_HEIGHT DCPEXT_HEIGHT
#define SCANOUT_STRIDE DCPEXT_STRIDE
#define SCANOUT_IOVA_BASE (1ULL << 40)
#define SCANOUT_IOVA_SIZE (1ULL << 36)

static bool dcpext_pageflip;
module_param(dcpext_pageflip, bool, 0444);
MODULE_PARM_DESC(dcpext_pageflip, "Use two retained external buffers and wait for firmware swap completion");

/*
 * Some docks and monitors drop HPD once shortly after the first mode is set and
 * bring it back within a second or two. Tolerate exactly one such bounce after
 * the first pattern: hold the retained buffer, wait for the link to return, and
 * send the pattern commands again.
 *
 * A link loss that outlasts the bounce (the dock was unplugged) parks the
 * scanout: it latches -ENOLINK, so the desktop sees a disconnect, but the
 * retained buffer and the firmware session stay as they were. When a dock
 * brings the link back, the pattern commands are sent again exactly as after a
 * bounce, and the new link gets its own bounce allowance. Any other error, or a
 * link loss before the first pattern was accepted, is terminal until reboot.
 */
#define SCANOUT_BOUNCE_GRACE_MS 10000
#define SCANOUT_REPATTERN_DELAY_MS 1500
enum { SCANOUT_BOUNCE_NONE, SCANOUT_BOUNCE_PENDING, SCANOUT_BOUNCE_USED };

struct dcpext_scanout {
	struct apple_dcp *dcp;
	struct device *firmware_dev;
	struct device_node *node;
	struct platform_device *pdev;
	struct iommu_group *group;
	struct iommu_domain *firmware_domain;
	struct iommu_domain *scanout_domain;
	void *pixels;
	dma_addr_t iova;
	size_t size;
	size_t frame_size;
	bool pageflips;
	unsigned int front;
	struct mutex present_lock;
	u64 frames_completed;
	u64 present_ns_max;
	size_t mapped;
	struct sg_table sgt;
	struct work_struct work;
	struct work_struct desktop_work;
	atomic_t desktop_requested;
	atomic_t terminal_error;
	bool desktop_registered;
	struct drm_device *retained_drm;
	struct work_struct invalidate_work;
	bool pattern_ready;
	atomic_t requested;
	atomic_t bounce; /* SCANOUT_BOUNCE_* */
	struct delayed_work bounce_work;
	struct delayed_work repattern_work;
	bool reattached; /* the desktop saw a disconnect before this restore */
	bool stopping;
	bool published; /* Firmware callbacks may retain this object until reboot. */
	struct device_attribute attr;
	struct device_attribute desktop_attr;
	struct device_attribute status_attr;
	struct device_attribute present_attr;
	struct attribute *attrs[5];
	struct attribute_group attr_group;
};

static void dcpext_invalidate_work(struct work_struct *work)
{
	struct dcpext_scanout *scanout = container_of(work, struct dcpext_scanout, invalidate_work);
	struct drm_device *drm = READ_ONCE(scanout->retained_drm);

	/* Never notify while holding the Type-C fabric or HPD locks. */
	if (drm && !READ_ONCE(scanout->stopping))
		drm_kms_helper_hotplug_event(drm);
}

static void scanout_fail(struct dcpext_scanout *scanout, int error)
{
	if (READ_ONCE(scanout->stopping))
		return;
	if (error >= 0 || (scanout->pageflips && error == -ENOLINK))
		error = -EIO;
	if (!atomic_cmpxchg(&scanout->terminal_error, 0, error))
		schedule_work(&scanout->invalidate_work);
}

bool dcpext_scanout_requested(struct apple_dcp *dcp)
{
	struct dcpext_scanout *scanout = smp_load_acquire(&dcp->dcpext_scanout);

	return scanout && atomic_read(&scanout->requested);
}

/* Unplugged after the first pattern: the link is gone, nothing else is lost. */
static bool scanout_parked(struct dcpext_scanout *scanout)
{
	return atomic_read(&scanout->terminal_error) == -ENOLINK &&
	       smp_load_acquire(&scanout->pattern_ready);
}

/* True when the scanout can never be used again. A parked scanout is not: the
 * next dock may route a tunnel and bring its link up to restore it. */
bool dcpext_scanout_terminal(struct apple_dcp *dcp)
{
	struct dcpext_scanout *scanout = smp_load_acquire(&dcp->dcpext_scanout);

	return scanout && atomic_read(&scanout->terminal_error) && !scanout_parked(scanout);
}

void dcpext_scanout_fault(struct apple_dcp *dcp, int error)
{
	struct dcpext_scanout *scanout = smp_load_acquire(&dcp->dcpext_scanout);

	if (scanout)
		scanout_fail(scanout, error);
}

void dcpext_scanout_invalidate(struct apple_dcp *dcp)
{
	struct dcpext_scanout *scanout = smp_load_acquire(&dcp->dcpext_scanout);

	/* Initial unplug remains allowed; only a started scanout owns DMA. */
	if (!scanout || !atomic_read(&scanout->requested))
		return;
	/* Link loss can race a submitted swap. Until recovery also accounts for
	 * its completion, keep both buffers and require a fresh boot in this mode.
	 */
	if (scanout->pageflips) {
		scanout_fail(scanout, -EIO);
		return;
	}
	/* The desktop reuses the pattern's retained buffer, so the same restore
	 * covers a bounce before or after desktop registration. */
	if (smp_load_acquire(&scanout->pattern_ready)) {
		int prev = atomic_cmpxchg(&scanout->bounce, SCANOUT_BOUNCE_NONE, SCANOUT_BOUNCE_PENDING);

		if (prev == SCANOUT_BOUNCE_NONE) {
			dev_info(scanout->firmware_dev, "external link lost after the first pattern; waiting up to %u ms for it to return\n",
				 SCANOUT_BOUNCE_GRACE_MS);
			schedule_delayed_work(&scanout->bounce_work, msecs_to_jiffies(SCANOUT_BOUNCE_GRACE_MS));
			return;
		}
		/* Re-routing the returning tunnel disconnects the port once more
		 * before reconnecting it; that is still the same bounce. */
		if (prev == SCANOUT_BOUNCE_PENDING)
			return;
		if (!atomic_read(&scanout->terminal_error))
			dev_info(scanout->firmware_dev, "external link lost again; scanout parked until a dock brings it back\n");
	}
	scanout_fail(scanout, -ENOLINK);
}

static void dcpext_bounce_timeout(struct work_struct *work)
{
	struct dcpext_scanout *scanout =
		container_of(to_delayed_work(work), struct dcpext_scanout, bounce_work);

	if (atomic_cmpxchg(&scanout->bounce, SCANOUT_BOUNCE_PENDING, SCANOUT_BOUNCE_USED) !=
	    SCANOUT_BOUNCE_PENDING)
		return;
	dev_info(scanout->firmware_dev, "external link did not return within %u ms; scanout parked until a dock brings it back\n",
		 SCANOUT_BOUNCE_GRACE_MS);
	scanout_fail(scanout, -ENOLINK);
}

static void dcpext_repattern_work(struct work_struct *work)
{
	struct dcpext_scanout *scanout =
		container_of(to_delayed_work(work), struct dcpext_scanout, repattern_work);
	int ret;

	if (READ_ONCE(scanout->stopping) || atomic_read(&scanout->terminal_error))
		return;
	ret = ibootep_rearm_pattern(scanout->dcp);
	if (!ret)
		ret = ibootep_present_pattern(scanout->dcp, scanout->iova, scanout->size, SCANOUT_STRIDE);
	if (ret) {
		scanout_fail(scanout, ret);
		dev_err(scanout->firmware_dev, "external pattern not restored after the link returned: %d; resources retained until reboot\n",
			ret);
		return;
	}
	if (xchg(&scanout->reattached, false)) {
		dev_info(scanout->firmware_dev, "external pattern restored after an unplug\n");
		schedule_work(&scanout->invalidate_work);
		return;
	}
	dev_info(scanout->firmware_dev, "external pattern restored after one hotplug bounce\n");
}

/* Caller holds hpd_mutex; the external link has just become ready again. */
void dcpext_scanout_link_restored(struct apple_dcp *dcp)
{
	struct dcpext_scanout *scanout = smp_load_acquire(&dcp->dcpext_scanout);

	if (!scanout || scanout->pageflips || READ_ONCE(scanout->stopping))
		return;
	if (atomic_cmpxchg(&scanout->bounce, SCANOUT_BOUNCE_PENDING, SCANOUT_BOUNCE_USED) ==
	    SCANOUT_BOUNCE_PENDING) {
		cancel_delayed_work(&scanout->bounce_work);
		dev_info(scanout->firmware_dev, "external link returned; sending the pattern commands again\n");
	} else if (scanout_parked(scanout) &&
		   atomic_cmpxchg(&scanout->terminal_error, -ENOLINK, 0) == -ENOLINK) {
		/* The desktop only copies into the retained buffer, so it may see
		 * the connector again before the pattern commands go out. */
		atomic_set(&scanout->bounce, SCANOUT_BOUNCE_NONE);
		WRITE_ONCE(scanout->reattached, true);
		dev_info(scanout->firmware_dev, "external link up again after an unplug; sending the pattern commands again\n");
	} else {
		return;
	}
	queue_delayed_work(system_dfl_wq, &scanout->repattern_work,
			   msecs_to_jiffies(SCANOUT_REPATTERN_DELAY_MS));
}

static bool scanout_link_ready(struct dcpext_scanout *scanout)
{
	struct apple_dcp *dcp = scanout->dcp;

	return !READ_ONCE(dcp->crashed) && !atomic_read(&scanout->terminal_error) &&
		smp_load_acquire(&dcp->external_link_ready) &&
		(!READ_ONCE(dcp->dptx_tunnel) || READ_ONCE(dcp->tb_clock_ok)) &&
		READ_ONCE(dcp->typec_cable_connected) &&
		READ_ONCE(dcp->dptxport[0].connected) &&
		READ_ONCE(dcp->dptxport[0].enabled);
}

bool dcpext_scanout_pageflips(struct apple_dcp *dcp)
{
	/* Pair with publication of the initialized scanout in register(). */
	struct dcpext_scanout *scanout = smp_load_acquire(&dcp->dcpext_scanout);

	return scanout && scanout->pageflips;
}

int dcpext_scanout_begin_frame(struct apple_dcp *dcp, struct dcpext_frame *frame)
{
	/* Pair with register(); drm_dev_enter() protects the parent lifetime. */
	struct dcpext_scanout *scanout = smp_load_acquire(&dcp->dcpext_scanout);

	if (!scanout || !scanout->pageflips)
		return -EOPNOTSUPP;
	mutex_lock(&scanout->present_lock);
	/* Observe the buffers/mappings published before pattern_ready. */
	if (READ_ONCE(scanout->stopping) || !scanout_link_ready(scanout) ||
	    !smp_load_acquire(&scanout->pattern_ready)) {
		mutex_unlock(&scanout->present_lock);
		return -ENOLINK;
	}
	frame->pixels = scanout->pixels + (scanout->front ^ 1) * scanout->frame_size;
	frame->size = scanout->frame_size;
	return 0;
}

int dcpext_scanout_end_frame(struct apple_dcp *dcp, bool present)
{
	/* Same publication/lifetime protection as begin_frame(). */
	struct dcpext_scanout *scanout = smp_load_acquire(&dcp->dcpext_scanout);
	unsigned int next = scanout->front ^ 1;
	u64 start;
	int ret = 0;

	lockdep_assert_held(&scanout->present_lock);
	if (!present)
		goto out;
	if (READ_ONCE(scanout->stopping) || !scanout_link_ready(scanout)) {
		ret = -ENOLINK;
		goto out;
	}
	dma_wmb();
	start = ktime_get_ns();
	ret = ibootep_present_frame(dcp, scanout->iova + next * scanout->frame_size,
				   scanout->frame_size, SCANOUT_STRIDE);
	if (ret) {
		/* Even -ENOLINK from a presentation command is uncertain DMA state;
		 * use a terminal error that the hotplug restore path cannot clear.
		 */
		scanout_fail(scanout, -EIO);
		goto out;
	}
	scanout->front = next;
	scanout->frames_completed++;
	scanout->present_ns_max = max(scanout->present_ns_max, ktime_get_ns() - start);
out:
	mutex_unlock(&scanout->present_lock);
	return ret;
}

static ssize_t dcpext_present_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcpext_scanout *scanout = container_of(attr, struct dcpext_scanout, present_attr);

	return sysfs_emit(buf, "pageflips=%u buffers=%u completed=%llu front=%u wait_max_us=%llu\n",
		scanout->pageflips, scanout->pageflips ? 2 : 1,
		READ_ONCE(scanout->frames_completed), READ_ONCE(scanout->front),
		READ_ONCE(scanout->present_ns_max) / NSEC_PER_USEC);
}

static ssize_t dcpext_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dcpext_scanout *scanout = container_of(attr, struct dcpext_scanout, status_attr);
	bool requested = atomic_read(&scanout->requested);
	bool ready = smp_load_acquire(&scanout->pattern_ready);
	bool desktop_requested = atomic_read(&scanout->desktop_requested);
	bool registered = smp_load_acquire(&scanout->desktop_registered);
	int error = atomic_read(&scanout->terminal_error);
	bool fw_ready = !READ_ONCE(scanout->dcp->crashed) && scanout->dcp->rtk && apple_rtkit_is_running(scanout->dcp->rtk) &&
		ibootep_is_ready(scanout->dcp) &&
		smp_load_acquire(&scanout->dcp->dptxport[0].enabled);
	const char *phase = error == -ENOLINK && ready ? "parked" : error ? "terminal" :
		registered ? "desktop_registered" :
		desktop_requested ? "desktop_pending" : ready ? "pattern_ready" :
		requested ? "pattern_pending" : "ready";

	return sysfs_emit(buf, "schema=1 phase=%s fw_ready=%u pattern_requested=%u pattern_ready=%u desktop_requested=%u desktop_registered=%u terminal_error=%d link_ready=%u width=%u height=%u stride=%u\n",
		phase, fw_ready, requested, ready, desktop_requested, registered, error,
		scanout_link_ready(scanout), SCANOUT_WIDTH, SCANOUT_HEIGHT, SCANOUT_STRIDE);
}

static bool scanout_iova_valid(u64 iova, size_t size)
{
	return size && IS_ALIGNED(iova, SZ_16K) && IS_ALIGNED(size, SZ_16K) &&
	       iova >= SCANOUT_IOVA_BASE && size <= SCANOUT_IOVA_SIZE &&
	       iova - SCANOUT_IOVA_BASE <= SCANOUT_IOVA_SIZE - size;
}

static void scanout_fill_pattern(void *pixels, size_t size)
{
	static const u32 bars[] = {
		0xffffffff, 0xffffff00, 0xff00ffff, 0xff00ff00,
		0xffff00ff, 0xffff0000, 0xff0000ff, 0xff000000,
	};
	__le32 *pixel = pixels;
	u32 x, y;

	memset(pixels, 0, size);
	for (y = 0; y < SCANOUT_HEIGHT; y++)
		for (x = 0; x < SCANOUT_WIDTH; x++)
			pixel[y * SCANOUT_WIDTH + x] =
				cpu_to_le32(bars[x / (SCANOUT_WIDTH / ARRAY_SIZE(bars))]);
}

/* Before mapping, require a hole in scanout and the expected firmware mapping.
 * Afterwards, require both DART domains to resolve every 16K page identically.
 * SG addresses describe physical backing, never DMA addresses from another map.
 */
static int scanout_verify_buffer(struct dcpext_scanout *scanout, bool mapped)
{
	struct scatterlist *sg;
	size_t total = 0, off;
	unsigned int i;

	if (!scanout_iova_valid(scanout->iova, scanout->size))
		return -ERANGE;
	for_each_sg(scanout->sgt.sgl, sg, scanout->sgt.orig_nents, i) {
		phys_addr_t phys = sg_phys(sg);
		size_t length = sg->length;

		if (!phys || !length || !IS_ALIGNED(phys, SZ_16K) ||
		    !IS_ALIGNED(length, SZ_16K) || length > scanout->size - total)
			return -EINVAL;
		for (off = 0; off < length; off += SZ_16K) {
			u64 iova = scanout->iova + total + off;
			phys_addr_t firmware = iommu_iova_to_phys(scanout->firmware_domain, iova);
			phys_addr_t display = iommu_iova_to_phys(scanout->scanout_domain, iova);

			if (firmware != phys + off)
				return -EFAULT;
			if (mapped ? display != firmware : display != 0)
				return mapped ? -EFAULT : -EBUSY;
		}
		total += length;
	}
	return total == scanout->size ? 0 : -EINVAL;
}

static int scanout_map_buffer(struct dcpext_scanout *scanout)
{
	struct scatterlist *sg;
	unsigned int i;
	int ret;

	ret = scanout_verify_buffer(scanout, false);
	if (ret)
		return ret;
	for_each_sg(scanout->sgt.sgl, sg, scanout->sgt.orig_nents, i) {
		ret = iommu_map(scanout->scanout_domain, scanout->iova + scanout->mapped,
				sg_phys(sg), sg->length, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret)
			return ret;
		scanout->mapped += sg->length;
	}
	return scanout_verify_buffer(scanout, true);
}

static bool scanout_node_verified(struct device_node *node)
{
	u32 marker;

	return node && of_device_is_available(node) &&
	       of_device_is_compatible(node, "apple,t6030-dispext-scanout") &&
	       !of_property_read_u32(node, "apple,t6030-dispext-handoff", &marker) && marker == 1 &&
	       !of_property_read_u32(node, "apple,t6030-scanout-verified", &marker) && marker == 1;
}

struct scanout_group_check {
	struct device *expected;
	unsigned int count;
};

static int scanout_group_member(struct device *dev, void *data)
{
	struct scanout_group_check *check = data;

	check->count++;
	return dev == check->expected ? 0 : -EBUSY;
}

static int scanout_check_domains(struct dcpext_scanout *scanout)
{
	struct device *dev = &scanout->pdev->dev;
	struct scanout_group_check check = { .expected = dev };
	struct iommu_group *firmware_group;
	int ret;

	/* These raw display mappings have no IOMMU_CACHE attribute. The tested
	 * firmware allocation is uncached coherent memory on a noncoherent device.
	 */
	if (dev_is_dma_coherent(scanout->firmware_dev) || dev_is_dma_coherent(dev))
		return -EOPNOTSUPP;
	/* No driver or second DMA client may allocate addresses in this domain. */
	if (device_is_bound(dev))
		return -EBUSY;
	scanout->firmware_domain = iommu_get_domain_for_dev(scanout->firmware_dev);
	scanout->scanout_domain = iommu_get_domain_for_dev(dev);
	if (IS_ERR_OR_NULL(scanout->firmware_domain) || IS_ERR_OR_NULL(scanout->scanout_domain) ||
	    scanout->firmware_domain == scanout->scanout_domain ||
	    !iommu_is_dma_domain(scanout->firmware_domain) ||
	    !iommu_is_dma_domain(scanout->scanout_domain) ||
	    !(scanout->scanout_domain->pgsize_bitmap & SZ_16K))
		return -EINVAL;
	scanout->group = iommu_group_get(dev);
	if (!scanout->group)
		return -ENODEV;
	firmware_group = iommu_group_get(scanout->firmware_dev);
	ret = firmware_group && firmware_group != scanout->group ? 0 : -EBUSY;
	if (firmware_group)
		iommu_group_put(firmware_group);
	if (ret)
		return ret;
	ret = iommu_group_for_each_dev(scanout->group, &check, scanout_group_member);
	if (ret)
		return ret;
	return check.count == 1 ? 0 : -EBUSY;
}

/*
 * The early gate predates external firmware startup. Recheck immediately
 * before attaching: neither firmware nor another client may have populated
 * the inherited roots in the meantime. This function performs no MMIO writes.
 */
static int scanout_recheck_dart(struct device_node *dart, struct device_node *dcp)
{
	struct device_node *dart_pd, *dcp_pd;
	struct resource regs, table, first = {};
	struct device_node *region = NULL;
	struct reserved_mem *rmem;
	void __iomem *mmio;
	u32 state[6], marker;
	u64 phys, size;
	void *root;
	int i, ret = -EINVAL;

	if (of_property_read_u32(dart, "apple,t6030-dispext-handoff", &marker) || marker != 1 ||
	    of_property_count_u32_elems(dart, "apple,inherited-dart-state") != 6 ||
	    of_property_read_u32_array(dart, "apple,inherited-dart-state", state, 6) ||
	    state[0] != 0 || state[3] != 4 ||
	    of_count_phandle_with_args(dart, "memory-region", NULL) != 2 ||
	    of_address_to_resource(dart, 0, &regs) || resource_size(&regs) != SZ_16K)
		return -EINVAL;
	/* The display DART of this processor's own pipe shares its CPU domain. */
	dart_pd = of_parse_phandle(dart, "power-domains", 0);
	dcp_pd = dcp_ext_cpu_domain(dcp);
	of_node_put(dart_pd);
	of_node_put(dcp_pd);
	if (!dart_pd || dart_pd != dcp_pd)
		return -EINVAL;
	mmio = ioremap(regs.start, resource_size(&regs));
	if (!mmio)
		return -ENOMEM;
	if (!(readl(mmio + 0x200) & BIT(0)))
		goto out;
	for (i = 0; i < 2; i++) {
		u32 sid = state[3 * i], tcr = state[3 * i + 1], ttbr = state[3 * i + 2];

		ret = -EINVAL;
		if ((tcr & (BIT(0) | BIT(1) | BIT(3))) != BIT(0) || !(ttbr & BIT(0)) ||
		    (ttbr & ~(GENMASK(29, 2) | BIT(0))) ||
		    readl(mmio + 0x1000 + 4 * sid) != tcr ||
		    readl(mmio + 0x1400 + 4 * sid) != ttbr)
			goto out;
		phys = (u64)(ttbr & GENMASK(29, 2)) << 12;
		region = of_parse_phandle(dart, "memory-region", i);
		if (!region || !of_property_read_bool(region, "no-map") ||
		    of_property_read_bool(region, "reusable") ||
		    of_address_to_resource(region, 0, &table) || table.end < table.start)
			goto out;
		size = resource_size(&table);
		rmem = of_reserved_mem_lookup(region);
		if (!rmem || size < SZ_16K || size > SZ_1M ||
		    !IS_ALIGNED(table.start, SZ_16K) || !IS_ALIGNED(size, SZ_16K) ||
		    rmem->base != table.start || rmem->size != size ||
		    phys < table.start || phys > table.end - SZ_16K + 1 ||
		    (i && table.start <= first.end && first.start <= table.end))
			goto out;
		if (!i)
			first = table;
		root = memremap(phys, SZ_16K, MEMREMAP_WB);
		if (!root) {
			ret = -ENOMEM;
			goto out;
		}
		ret = memchr_inv(root, 0, SZ_16K) ? -EBUSY : 0;
		memunmap(root);
		if (ret)
			goto out;
		of_node_put(region);
		region = NULL;
	}
out:
	of_node_put(region);
	iounmap(mmio);
	return ret;
}

static int scanout_create_device(struct dcpext_scanout *scanout)
{
	struct of_phandle_args spec;
	struct platform_device *existing;
	int ret;

	if (PAGE_SIZE != SZ_16K || !scanout_node_verified(scanout->node))
		return -EINVAL;
	if (of_count_phandle_with_args(scanout->node, "iommus", "#iommu-cells") != 1)
		return -EINVAL;
	ret = of_parse_phandle_with_args(scanout->node, "iommus", "#iommu-cells", 0, &spec);
	if (ret)
		return ret;
	ret = spec.args_count == 1 && spec.args[0] == 0 &&
	      of_device_is_compatible(spec.np, "apple,t8110-dart") &&
	      of_device_is_available(spec.np) ? 0 : -EINVAL;
	if (!ret)
		ret = scanout_recheck_dart(spec.np, scanout->firmware_dev->of_node);
	of_node_put(spec.np);
	if (ret)
		return ret;
	existing = of_find_device_by_node(scanout->node);
	if (existing) {
		put_device(&existing->dev);
		return -EBUSY;
	}
	scanout->pdev = of_platform_device_create(scanout->node, NULL, scanout->firmware_dev);
	if (!scanout->pdev)
		return -ENODEV;
	/* Once attached, never destroy this consumer of the locked inherited DART. */
	ret = dma_set_mask_and_coherent(&scanout->pdev->dev, DMA_BIT_MASK(42));
	if (!ret)
		ret = of_dma_configure(&scanout->pdev->dev, scanout->node, true);
	if (ret)
		return ret;
	return scanout_check_domains(scanout);
}

static void dcpext_pattern_work(struct work_struct *work)
{
	struct dcpext_scanout *scanout = container_of(work, struct dcpext_scanout, work);
	struct device *dev = scanout->firmware_dev;
	const char *step = "scanout DMA consumer";
	int ret;

	if (READ_ONCE(scanout->stopping) || atomic_read(&scanout->terminal_error))
		return;
	ret = scanout_create_device(scanout);
	if (ret)
		goto fail;
	scanout->frame_size = ALIGN((size_t)SCANOUT_STRIDE * SCANOUT_HEIGHT, SZ_16K);
	scanout->size = scanout->frame_size * (scanout->pageflips ? 2 : 1);
	step = "coherent framebuffer allocation";
	scanout->pixels = dma_alloc_coherent(dev, scanout->size, &scanout->iova, GFP_KERNEL);
	if (!scanout->pixels) {
		ret = -ENOMEM;
		goto fail;
	}
	if (!scanout_iova_valid(scanout->iova, scanout->size)) {
		ret = -ERANGE;
		goto fail;
	}
	step = "coherent backing scatterlist";
	ret = dma_get_sgtable(dev, &scanout->sgt, scanout->pixels, scanout->iova, scanout->size);
	if (ret)
		goto fail;
	step = "scanout IOVA mapping and page verification";
	ret = scanout_map_buffer(scanout);
	if (ret)
		goto fail;
	scanout_fill_pattern(scanout->pixels, scanout->size);
	dma_wmb();
	dev_info(dev, "external pattern: verified %zu bytes at %pad in separate firmware/scanout domains; BGRA 3840x2160 stride %u\n",
		 scanout->size, &scanout->iova, SCANOUT_STRIDE);
	step = "iBoot first-pattern commands";
	if (atomic_read(&scanout->terminal_error)) {
		ret = -ENOLINK;
		goto fail;
	}
	ret = ibootep_present_pattern(scanout->dcp, scanout->iova,
				     scanout->frame_size, SCANOUT_STRIDE);
	if (!ret && scanout->pageflips) {
		step = "initial firmware swap completion";
		ret = ibootep_present_frame(scanout->dcp, scanout->iova,
					   scanout->frame_size, SCANOUT_STRIDE);
	}
	if (ret)
		goto fail;
	if (atomic_read(&scanout->terminal_error)) {
		ret = -ENOLINK;
		goto fail;
	}
	smp_store_release(&scanout->pattern_ready, true);
	dev_info(dev, "external pattern commands accepted; all scanout resources retained until reboot\n");
	return;
fail:
	scanout_fail(scanout, ret);
	dev_err(dev, "external pattern stopped at %s: %d (%zu bytes mapped); no retry, resources retained until reboot\n",
		step, ret, scanout->mapped);
}

static ssize_t dcpext_pattern_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct dcpext_scanout *scanout = container_of(attr, struct dcpext_scanout, attr);
	struct apple_dcp *dcp = scanout->dcp;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	if (READ_ONCE(scanout->stopping) || !dcp->external || !dcp->rtk ||
	    !apple_rtkit_is_running(dcp->rtk) || READ_ONCE(dcp->crashed) || !scanout_node_verified(scanout->node))
		return -ENODEV;
	/* Serialize the first request against the disconnect invalidation hook. */
	mutex_lock(&dcp->hpd_mutex);
	if (atomic_read(&scanout->terminal_error) || !scanout_link_ready(scanout)) {
		mutex_unlock(&dcp->hpd_mutex);
		return -ENOLINK;
	}
	if (atomic_cmpxchg(&scanout->requested, 0, 1)) {
		mutex_unlock(&dcp->hpd_mutex);
		return -EALREADY;
	}
	mutex_unlock(&dcp->hpd_mutex);
	/* Firmware DMA and callbacks may outlive any error returned by this worker. */
	__module_get(THIS_MODULE);
	queue_work(system_dfl_wq, &scanout->work);
	return count;
}

/* Separate explicit step: keep a successful pattern visible until requested. */
static void dcpext_desktop_work(struct work_struct *work)
{
	struct dcpext_scanout *scanout = container_of(work, struct dcpext_scanout, desktop_work);
	int ret;

	if (READ_ONCE(scanout->stopping) || atomic_read(&scanout->terminal_error) ||
	    !smp_load_acquire(&scanout->pattern_ready))
		return;
	ret = dcpext_drm_register(scanout->dcp, scanout->pixels, scanout->frame_size,
				 SCANOUT_STRIDE, &scanout->terminal_error, &scanout->retained_drm);
	if (ret) {
		scanout_fail(scanout, ret);
		dev_err(scanout->firmware_dev, "external desktop registration failed: %d; scanout retained\n", ret);
	} else {
		smp_store_release(&scanout->desktop_registered, true);
		if (atomic_read(&scanout->terminal_error))
			schedule_work(&scanout->invalidate_work);
	}
}

static ssize_t dcpext_desktop_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct dcpext_scanout *scanout = container_of(attr, struct dcpext_scanout, desktop_attr);

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	if (READ_ONCE(scanout->stopping))
		return -ENODEV;
	if (atomic_read(&scanout->terminal_error))
		return -ENOLINK;
	if (!smp_load_acquire(&scanout->pattern_ready))
		return -EAGAIN;
	if (atomic_cmpxchg(&scanout->desktop_requested, 0, 1))
		return -EALREADY;
	queue_work(system_dfl_wq, &scanout->desktop_work);
	return count;
}

static void dcpext_scanout_cleanup(void *data)
{
	struct dcpext_scanout *scanout = data;

	WRITE_ONCE(scanout->stopping, true);
	cancel_work_sync(&scanout->desktop_work);
	cancel_work_sync(&scanout->work);
	cancel_delayed_work_sync(&scanout->bounce_work);
	cancel_delayed_work_sync(&scanout->repattern_work);
	cancel_work_sync(&scanout->invalidate_work);
	if (atomic_read(&scanout->requested) || READ_ONCE(scanout->published))
		return; /* Firmware callbacks and possible DMA survive parent cleanup. */
	WRITE_ONCE(scanout->dcp->dcpext_scanout, NULL);
	of_node_put(scanout->node);
	put_device(scanout->firmware_dev);
	kfree(scanout);
}

int dcpext_scanout_register(struct apple_dcp *dcp)
{
	struct device_node *node;
	struct dcpext_scanout *scanout;
	int ret;

	if (!dcp->external || dcp->fw_compat != DCP_FIRMWARE_V_14_7)
		return -ENODEV;
	if (dcpext_pageflip && !dcp_ext_firmware_admitted(dcp->dev->of_node))
		return -EOPNOTSUPP;
	node = of_get_child_by_name(dcp->dev->of_node, "scanout");
	if (!scanout_node_verified(node)) {
		of_node_put(node);
		return -ENODEV;
	}
	scanout = kzalloc(sizeof(*scanout), GFP_KERNEL);
	if (!scanout) {
		of_node_put(node);
		return -ENOMEM;
	}
	scanout->dcp = dcp;
	scanout->firmware_dev = get_device(dcp->dev);
	scanout->node = node;
	scanout->pageflips = dcpext_pageflip;
	mutex_init(&scanout->present_lock);
	atomic_set(&scanout->requested, 0);
	atomic_set(&scanout->terminal_error, 0);
	INIT_WORK(&scanout->invalidate_work, dcpext_invalidate_work);
	INIT_WORK(&scanout->work, dcpext_pattern_work);
	atomic_set(&scanout->bounce, SCANOUT_BOUNCE_NONE);
	INIT_DELAYED_WORK(&scanout->bounce_work, dcpext_bounce_timeout);
	INIT_DELAYED_WORK(&scanout->repattern_work, dcpext_repattern_work);
	atomic_set(&scanout->desktop_requested, 0);
	INIT_WORK(&scanout->desktop_work, dcpext_desktop_work);
	sysfs_attr_init(&scanout->attr.attr);
	scanout->attr.attr.name = "dcpext_pattern";
	scanout->attr.attr.mode = 0200;
	scanout->attr.store = dcpext_pattern_store;
	scanout->attrs[0] = &scanout->attr.attr;
	sysfs_attr_init(&scanout->desktop_attr.attr);
	scanout->desktop_attr.attr.name = "dcpext_desktop";
	scanout->desktop_attr.attr.mode = 0200;
	scanout->desktop_attr.store = dcpext_desktop_store;
	scanout->attrs[1] = &scanout->desktop_attr.attr;
	sysfs_attr_init(&scanout->status_attr.attr);
	scanout->status_attr.attr.name = "dcpext_status";
	scanout->status_attr.attr.mode = 0444;
	scanout->status_attr.show = dcpext_status_show;
	scanout->attrs[2] = &scanout->status_attr.attr;
	sysfs_attr_init(&scanout->present_attr.attr);
	scanout->present_attr.attr.name = "dcpext_present";
	scanout->present_attr.attr.mode = 0444;
	scanout->present_attr.show = dcpext_present_show;
	scanout->attrs[3] = &scanout->present_attr.attr;
	scanout->attr_group.attrs = scanout->attrs;
	ret = devm_add_action_or_reset(dcp->dev, dcpext_scanout_cleanup, scanout);
	if (ret)
		return ret;
	WRITE_ONCE(scanout->published, true);
	smp_store_release(&dcp->dcpext_scanout, scanout);
	ret = devm_device_add_group(dcp->dev, &scanout->attr_group);
	if (!ret)
		dev_info(dcp->dev, "external scanout handoff verified; explicit dcpext_pattern=1 is available\n");
	return ret;
}
