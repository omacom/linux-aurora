// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple Image Signal Processor driver
 *
 * Copyright (C) 2023 The Asahi Linux Contributors
 */

#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>

#include "isp-cam.h"
#include "isp-fw.h"
#include "isp-iommu.h"
#include "isp-regs.h"
#include "isp-v4l2.h"

static void apple_isp_kvfree(void *data)
{
	kvfree(data);
}

static void apple_isp_detach_genpd(struct apple_isp *isp)
{
	if (isp->pd_count <= 1)
		return;

	for (int i = isp->pd_count - 1; i >= 0; i--) {
		if (isp->pd_link[i])
			device_link_del(isp->pd_link[i]);
		if (IS_ERR_OR_NULL(isp->pd_dev[i]))
			continue;
		if (isp->hw->resident_fw)
			dev_pm_syscore_device(isp->pd_dev[i], false);
		dev_pm_domain_detach(isp->pd_dev[i], true);
	}

	return;
}

static int apple_isp_attach_genpd(struct apple_isp *isp)
{
	struct device *dev = isp->dev;

	isp->pd_count = of_count_phandle_with_args(
		dev->of_node, "power-domains", "#power-domain-cells");
	if (isp->pd_count <= 1)
		return 0;

	isp->pd_dev = devm_kcalloc(dev, isp->pd_count, sizeof(*isp->pd_dev),
				   GFP_KERNEL);
	if (!isp->pd_dev)
		return -ENOMEM;

	isp->pd_link = devm_kcalloc(dev, isp->pd_count, sizeof(*isp->pd_link),
				    GFP_KERNEL);
	if (!isp->pd_link)
		return -ENOMEM;

	for (int i = 0; i < isp->pd_count; i++) {
		int flags = DL_FLAG_STATELESS;

		/* Primary power domain uses RPM integration */
		if (i == 0)
			flags |= DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE;

		isp->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(isp->pd_dev[i])) {
			apple_isp_detach_genpd(isp);
			return PTR_ERR(isp->pd_dev[i]);
		}

		isp->pd_link[i] =
			device_link_add(dev, isp->pd_dev[i], flags);

		if (!isp->pd_link[i]) {
			apple_isp_detach_genpd(isp);
			return -EINVAL;
		}

		/*
		 * The domains of resident firmware are switched by the
		 * driver alone, through runtime PM: the firmware does not
		 * start again once they have been off. System sleep would
		 * otherwise also have genpd switch them off in the noirq
		 * phase of suspend and back on in the noirq phase of resume,
		 * where the CPU core domains of a stopped coprocessor fail
		 * to reach the active state.
		 */
		if (isp->hw->resident_fw)
			dev_pm_syscore_device(isp->pd_dev[i], true);
	}

	return 0;
}

static void apple_isp_unmap_fw_mmio(struct apple_isp *isp, unsigned int count)
{
	for (unsigned int i = 0; i < count; i++)
		iommu_unmap(isp->domain, isp->hw->fw_mmio[i].base,
			    isp->hw->fw_mmio[i].size);
}

/*
 * Some firmware accesses registers of other blocks, such as the PMGR
 * scratch registers named in PMP_CTRL_SET, through its DART at their
 * physical addresses. Map those windows 1:1.
 */
static int apple_isp_map_fw_mmio(struct apple_isp *isp)
{
	u64 start = isp->fw.heap_top, end = start + isp->iova_size;

	for (unsigned int i = 0; i < isp->hw->num_fw_mmio; i++) {
		const struct isp_mmio_window *w = &isp->hw->fw_mmio[i];
		int err;

		/* Keep them out of the range the surfaces come from. */
		if (w->base < end && w->base + w->size > start) {
			dev_err(isp->dev,
				"MMIO window 0x%llx+0x%llx overlaps the IOVA range\n",
				w->base, w->size);
			err = -EINVAL;
		} else {
			/*
			 * Cacheable: with the no-cache attribute, the ISP took
			 * an SError on its first write to the PMP scratch
			 * register when streaming started.
			 */
			err = iommu_map(isp->domain, w->base, w->base, w->size,
					IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE,
					GFP_KERNEL);
			if (err)
				dev_err(isp->dev,
					"failed to map MMIO window 0x%llx+0x%llx: %d\n",
					w->base, w->size, err);
		}

		if (err) {
			apple_isp_unmap_fw_mmio(isp, i);
			return err;
		}
	}

	return 0;
}

/* The IOVA range of memory-region @idx, from its last iommu-addresses entry. */
static int isp_of_region_iova_idx(struct device *dev, int idx,
				  phys_addr_t *base, size_t *size)
{
	const __be32 *maps, *end;
	int len;

	struct device_node *np __free(device_node) =
		of_parse_phandle(dev->of_node, "memory-region", idx);
	if (!np)
		return -ENODEV;

	maps = of_get_property(np, "iommu-addresses", &len);
	if (!maps || !len)
		return -ENODEV;

	end = maps + len / sizeof(__be32);

	while (maps < end) {
		maps++;
		maps = of_translate_dma_region(dev->of_node, maps, base, size);
	}

	return 0;
}

/* The IOVA range of a named memory-region. */
static int isp_of_region_iova(struct device *dev, const char *name,
			      phys_addr_t *base, size_t *size)
{
	int idx;

	idx = of_property_match_string(dev->of_node, "memory-region-names",
				       name);
	if (idx < 0)
		return -ENOENT;

	return isp_of_region_iova_idx(dev, idx, base, size);
}

/*
 * The memory-region of the firmware data segment: the one named "data", or,
 * where the boot loader lists the firmware segments without names (T8140),
 * the one that ends where the heap starts. The loader places the text and
 * data segments and the heap back to back in the firmware's address space.
 */
static int isp_of_fw_data_region(struct device *dev)
{
	phys_addr_t heap_base, base;
	size_t heap_size, size;
	int idx, count;

	idx = of_property_match_string(dev->of_node, "memory-region-names",
				       "data");
	if (idx >= 0)
		return idx;

	if (isp_of_region_iova(dev, "heap", &heap_base, &heap_size))
		return -ENOENT;

	count = of_count_phandle_with_args(dev->of_node, "memory-region", NULL);
	for (idx = 0; idx < count; idx++) {
		if (isp_of_region_iova_idx(dev, idx, &base, &size))
			continue;
		if (size && base + size == heap_base)
			return idx;
	}

	return -ENOENT;
}

static void *apple_isp_copy_region(struct device *dev, struct resource *res,
				   void **copy)
{
	void *mem;

	mem = devm_memremap(dev, res->start, resource_size(res), MEMREMAP_WB);
	if (IS_ERR(mem))
		return mem;

	*copy = kvmalloc(resource_size(res), GFP_KERNEL);
	if (!*copy)
		return ERR_PTR(-ENOMEM);
	if (devm_add_action_or_reset(dev, apple_isp_kvfree, *copy))
		return ERR_PTR(-ENOMEM);

	memcpy(*copy, mem, resource_size(res));
	return mem;
}

/*
 * Keep a copy of the firmware data segment as the boot loader loaded it
 * (hw->fw_restore), and of the heap where the boot loader provides its
 * memory instead of the host. This must run before the firmware first
 * starts.
 */
static int apple_isp_copy_fw_data(struct apple_isp *isp)
{
	struct device *dev = isp->dev;
	struct resource res;
	int idx, err;

	idx = isp_of_fw_data_region(dev);
	if (idx < 0)
		return idx;

	err = of_reserved_mem_region_to_resource(dev->of_node, idx, &res);
	if (err)
		return err;

	isp->fw_data_size = resource_size(&res);
	isp->fw_data = apple_isp_copy_region(dev, &res, &isp->fw_data_copy);
	if (IS_ERR(isp->fw_data))
		return PTR_ERR(isp->fw_data);

	/* A host-backed main memory tail is cleared instead. */
	if (isp->fw.data_top ||
	    of_reserved_mem_region_to_resource_byname(dev->of_node, "heap", &res))
		return 0;

	isp->fw_heap_size = resource_size(&res);
	isp->fw_heap = apple_isp_copy_region(dev, &res, &isp->fw_heap_copy);
	return PTR_ERR_OR_ZERO(isp->fw_heap);
}

/*
 * Firmware that starts again needs the DART address filter registers, to
 * program the filter after each power-up, and its data segment. Device
 * trees from before those were described run it resident instead
 * (hw->resident_fallback), as it ran before.
 */
static void apple_isp_select_hw(struct apple_isp *isp)
{
	struct device *dev = isp->dev;
	struct apple_isp_hw *hw = &isp->hw_local;

	*hw = *isp->hw;
	isp->hw = hw;

	struct device_node *dart __free(device_node) =
		of_parse_phandle(dev->of_node, "iommus", 0);

	/*
	 * Resident firmware that starts again (hw->fw_restart) runs from its own
	 * memory, but after a power-down it reaches its MMIO only once the first
	 * DART has replayed the boot loader's address filter. Without
	 * apple,dapf-ranges on that DART it keeps running, as it ran before.
	 */
	if (hw->fw_restart &&
	    !(dart && of_property_present(dart, "apple,dapf-ranges"))) {
		dev_info(dev, "no address filter replay in the device tree, keeping the firmware resident\n");
		hw->fw_restart = false;
		hw->deferred_start = false;
		hw->coproc_quiesce = 0;
		hw->fw_idle_ms = 0;
		return;
	}

	if (!hw->resident_fallback)
		return;

	if (dart && of_property_match_string(dart, "reg-names", "dapf") >= 0 &&
	    isp_of_fw_data_region(dev) >= 0)
		return;

	dev_info(dev, "no address filter or firmware data in the device tree, keeping the firmware resident\n");
	hw->fw_restore = false;
	hw->deferred_start = false;
	hw->resident_fw = true;
	hw->coproc_quiesce = 0;
	hw->dapf = NULL;
	hw->num_dapf = 0;
	hw->fw_idle_ms = 0;
}

/*
 * The firmware reaches its fixed MMIO windows through the first DART, whose
 * address filter the ADT describes for them. Its registers are on that DART.
 */
static int apple_isp_map_dapf(struct apple_isp *isp)
{
	struct device *dev = isp->dev;
	/* The first iommus entry, whatever its #iommu-cells */
	struct device_node *dart __free(device_node) =
		of_parse_phandle(dev->of_node, "iommus", 0);
	struct resource res;
	int idx, err;

	if (!dart)
		return dev_err_probe(dev, -ENODEV, "no ISP DART\n");

	/* DAPF_* describe the T8110 DART's address filter */
	if (!of_device_is_compatible(dart, "apple,t8110-dart"))
		return dev_err_probe(dev, -ENODEV,
				     "%pOF has no known address filter\n", dart);

	idx = of_property_match_string(dart, "reg-names", "dapf");
	if (idx < 0)
		return dev_err_probe(dev, idx,
				     "%pOF has no address filter registers\n", dart);

	err = of_address_to_resource(dart, idx, &res);
	if (err)
		return dev_err_probe(dev, err, "%pOF: bad address filter registers\n",
				     dart);

	if (resource_size(&res) < isp->hw->num_dapf * DAPF_WINDOW_STRIDE)
		return dev_err_probe(dev, -EINVAL, "%pOF: address filter too small\n",
				     dart);

	isp->dapf = devm_ioremap_resource(dev, &res);
	return PTR_ERR_OR_ZERO(isp->dapf);
}

/* Program the address filter windows, then check that they took. */
int apple_isp_program_dapf(struct apple_isp *isp)
{
	const struct apple_isp_hw *hw = isp->hw;
	u32 sidmask = hw->dapf_sidmask ?: DAPF_CTRL2_ISP;
	void __iomem *w;
	unsigned int i;

	for (i = 0; i < hw->num_dapf; i++) {
		w = isp->dapf + i * DAPF_WINDOW_STRIDE;
		writel(0, w + DAPF_CTRL1);
		writeq(hw->dapf[i].start, w + DAPF_START);
		writeq(hw->dapf[i].end, w + DAPF_END);
		writel(DAPF_CTRL0_ISP, w + DAPF_CTRL0);
		writel(sidmask, w + DAPF_CTRL2);
	}

	/* Only bits 7:0 of CTRL0 and bits 63:2 of END read back. */
	for (i = 0; i < hw->num_dapf; i++) {
		w = isp->dapf + i * DAPF_WINDOW_STRIDE;
		if (readl(w + DAPF_CTRL1) ||
		    readq(w + DAPF_START) != hw->dapf[i].start ||
		    (readq(w + DAPF_END) | 3) != (hw->dapf[i].end | 3) ||
		    (readl(w + DAPF_CTRL0) & 0xff) != DAPF_CTRL0_ISP ||
		    readl(w + DAPF_CTRL2) != sidmask) {
			dev_err(isp->dev, "DAPF window %u (0x%llx-0x%llx) failed readback\n",
				i, hw->dapf[i].start, hw->dapf[i].end);
			return -EIO;
		}
	}

	return 0;
}

static int apple_isp_init_iommu(struct apple_isp *isp)
{
	struct device *dev = isp->dev;
	phys_addr_t heap_base, data_base;
	size_t heap_size, data_size;
	u64 vm_size;
	int err;

	isp->domain = iommu_get_domain_for_dev(isp->dev);
	if (!isp->domain)
		return -ENODEV;
	isp->shift = __ffs(isp->domain->pgsize_bitmap);
	isp->fw_iova_mask = isp->hw->fw_iova_mask ?: U64_MAX;

	err = isp_of_region_iova(dev, "heap", &heap_base, &heap_size);
	if (err) {
		dev_err(dev, "No valid iommu-addresses found for heap\n");
		return -ENODEV;
	}

	isp->fw.heap_base = heap_base;
	isp->fw.heap_top = heap_base + heap_size;

	/*
	 * Where the bootloader loads the firmware's data, it backs the
	 * firmware's main memory only up to the end of it. The rest, up to
	 * the heap top, is the host's to back.
	 */
	err = -ENOENT;
	if (isp->hw->gen == ISP_GEN_T6040)
		err = isp_of_region_iova(dev, "data", &data_base, &data_size);
	if (!err) {
		isp->fw.data_top = data_base + data_size;
		if (isp->fw.data_top <= isp->fw.heap_base ||
		    isp->fw.data_top > isp->fw.heap_top ||
		    !IS_ALIGNED(isp->fw.data_top, 1UL << isp->shift)) {
			dev_err(dev, "data 0x%llx outside the heap 0x%llx-0x%llx\n",
				isp->fw.data_top, isp->fw.heap_base,
				isp->fw.heap_top);
			return -EINVAL;
		}
	} else if (err != -ENOENT) {
		dev_err(dev, "No valid iommu-addresses found for data\n");
		return -ENODEV;
	}

	err = of_property_read_u64(dev->of_node, "apple,dart-vm-size",
				   &vm_size);
	if (err) {
		dev_err(dev, "failed to read 'apple,dart-vm-size': %d\n", err);
		return err;
	}

	// FIXME: refactor this, maybe use regular iova stuff?
	isp->iova_size = vm_size - (heap_base & 0xffffffff);
	mt_init_flags(&isp->iovas, MT_FLAGS_ALLOC_RANGE);

	err = apple_isp_map_fw_mmio(isp);
	if (err) {
		mtree_destroy(&isp->iovas);
		return err;
	}

	return 0;
}

static void apple_isp_free_iommu(struct apple_isp *isp)
{
	apple_isp_unmap_fw_mmio(isp, isp->hw->num_fw_mmio);
	WARN_ON(!mtree_empty(&isp->iovas));
	mtree_destroy(&isp->iovas);
}

static int isp_of_read_coord(struct device *dev, struct device_node *np,
			     const char *prop, struct coord *val)
{
	u32 xy[2];
	int ret;

	ret = of_property_read_u32_array(np, prop, xy, 2);
	if (ret) {
		dev_err(dev, "failed to read '%s' property\n", prop);
		return ret;
	}

	val->x = xy[0];
	val->y = xy[1];
	return 0;
}

static int apple_isp_init_presets(struct apple_isp *isp)
{
	struct device *dev = isp->dev;
	struct isp_preset *preset;
	int err = 0;

	struct device_node *np __free(device_node) =
		of_get_child_by_name(dev->of_node, "sensor-presets");
	if (!np) {
		dev_err(dev, "failed to get DT node 'presets'\n");
		return -EINVAL;
	}

	isp->num_presets = of_get_child_count(np);
	if (!isp->num_presets) {
		dev_err(dev, "no sensor presets found\n");
		return -EINVAL;
	}

	isp->presets = devm_kzalloc(
		dev, sizeof(*isp->presets) * isp->num_presets, GFP_KERNEL);
	if (!isp->presets)
		return -ENOMEM;

	preset = isp->presets;
	for_each_child_of_node_scoped(np, child) {
		u32 xywh[4];

		err = of_property_read_u32(child, "apple,config-index",
					   &preset->index);
		if (err) {
			dev_err(dev, "no apple,config-index property\n");
			return err;
		}

		err = isp_of_read_coord(dev, child, "apple,input-size",
					&preset->input_dim);
		if (err)
			return err;
		err = isp_of_read_coord(dev, child, "apple,output-size",
					&preset->output_dim);
		if (err)
			return err;

		err = of_property_read_u32_array(child, "apple,crop", xywh, 4);
		if (err) {
			dev_err(dev, "failed to read 'apple,crop' property\n");
			return err;
		}
		preset->crop_offset.x = xywh[0];
		preset->crop_offset.y = xywh[1];
		preset->crop_size.x = xywh[2];
		preset->crop_size.y = xywh[3];

		preset++;
	}

	return 0;
}

/* Preview only; processing and image quality need separate qualification. */
static bool preview_26 = true;
module_param(preview_26, bool, 0400);
MODULE_PARM_DESC(preview_26, "Enable J613 preview on ISP firmware 26.6.2 (default: true; false disables)");

static const char * isp_fw2str(enum isp_firmware_version version)
{
	switch (version) {
	case ISP_FIRMWARE_V_12_3:
		return "12.3";
	case ISP_FIRMWARE_V_12_4:
		return "12.4";
	case ISP_FIRMWARE_V_13_5:
		return "13.5";
	case ISP_FIRMWARE_V_14_7:
		return "14.7";
	case ISP_FIRMWARE_V_26_6_2:
		return "26.6.2";
	default:
		return "unknown";
	}
}

#define ISP_FW_VERSION_MIN_LEN	3
#define ISP_FW_VERSION_MAX_LEN	5

static enum isp_firmware_version isp_read_fw_version(struct device *dev,
						     const char *name)
{
	u32 ver[ISP_FW_VERSION_MAX_LEN];
	int size, len;

	/* The array reader truncates trailing bytes; require complete cells. */
	if (of_find_property(dev->of_node, name, &size) && size % sizeof(u32)) {
		dev_warn(dev, "could not parse %s: invalid length %d\n", name, size);
		return ISP_FIRMWARE_V_UNKNOWN;
	}

	len = of_property_read_variable_u32_array(dev->of_node, name, ver,
						  ISP_FW_VERSION_MIN_LEN,
						  ISP_FW_VERSION_MAX_LEN);

	switch (len) {
	case -EINVAL:
		/* not provided, see isp_check_firmware_version() */
		break;
	case 3:
		if (ver[0] == 26 && ver[1] == 6 && ver[2] == 2)
			return ISP_FIRMWARE_V_26_6_2;
		if (ver[0] == 12 && ver[1] == 3 && ver[2] <= 1)
			return ISP_FIRMWARE_V_12_3;
		else if (ver[0] == 12 && ver[1] == 4 && ver[2] == 0)
			return ISP_FIRMWARE_V_12_4;
		else if (ver[0] == 13 && ver[1] == 5 && ver[2] == 0)
			return ISP_FIRMWARE_V_13_5;
		else if (ver[0] == 14 && ver[1] == 7 && ver[2] == 0)
			return ISP_FIRMWARE_V_14_7;

		dev_warn(dev, "unknown %s: %d.%d.%d\n", name, ver[0], ver[1], ver[2]);
		break;
	case 4:
		dev_warn(dev, "unknown %s: %d.%d.%d.%d\n", name, ver[0], ver[1],
			 ver[2], ver[3]);
		break;
	case 5:
		dev_warn(dev, "unknown %s: %d.%d.%d.%d.%d\n", name, ver[0],
			 ver[1], ver[2], ver[3], ver[4]);
		break;
	default:
		dev_warn(dev, "could not parse %s: %d\n", name, len);
		break;
	}

	return ISP_FIRMWARE_V_UNKNOWN;
}

static enum isp_firmware_version
isp_check_firmware_version(struct apple_isp *isp)
{
	struct device *dev = isp->dev;
	enum isp_firmware_version version, compat;

	/* The H17 interface is selected by match data, not by version. */
	if (isp->hw->fw_abi == ISP_FW_ABI_H17)
		return ISP_FIRMWARE_V_UNKNOWN;

	/* firmware version is just informative */
	version = isp_read_fw_version(dev, "apple,firmware-version");
	compat = isp_read_fw_version(dev, "apple,firmware-compat");

	dev_dbg(dev, "ISP firmware-compat: %s (FW: %s)\n", isp_fw2str(compat),
		isp_fw2str(version));

	if (!of_property_present(dev->of_node, "apple,firmware-compat"))
		dev_warn(dev, "firmware compatibility version not provided, assuming 12.x\n");

	return compat;
}

static bool isp_preview26_admitted(struct device *dev,
				   enum isp_firmware_version compat)
{
	if (compat != ISP_FIRMWARE_V_26_6_2 || !preview_26 ||
	    !of_machine_is_compatible("apple,j613"))
		return false;
	if (!of_device_is_compatible(dev->of_node, "apple,t8122-isp"))
		return false;
	return isp_read_fw_version(dev, "apple,firmware-version") ==
		ISP_FIRMWARE_V_26_6_2;
}

/* After a system sleep, see apple_isp_resume(). */
static void apple_isp_resume_work(struct work_struct *work)
{
	struct apple_isp *isp = container_of(work, struct apple_isp, resume_work);

	mutex_lock(&isp->video_lock);
	isp->fw_sleep_stopped = false;
	mutex_unlock(&isp->video_lock);
	apple_isp_video_resume(isp);
}

/*
 * Idle firmware that starts again from its own memory is stopped before a
 * system sleep begins, as macOS's camera service powers an idle ISP off
 * before the kernel sleeps: the sleep then finds it off, as after a stream.
 */
static int apple_isp_pm_notify(struct notifier_block *nb, unsigned long event,
			       void *unused)
{
	struct apple_isp *isp = container_of(nb, struct apple_isp, pm_nb);

	if (event == PM_SUSPEND_PREPARE || event == PM_HIBERNATION_PREPARE)
		apple_isp_firmware_flush(isp);
	return NOTIFY_DONE;
}

static int apple_isp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_isp *isp;
	struct resource *res;
	int err;

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (err)
		return err;

	/*
	 * Not devres: an open video device file keeps this until it is
	 * closed, also after unbind. See apple_isp_release().
	 */
	isp = kzalloc_obj(*isp);
	if (!isp)
		return -ENOMEM;

	isp->dev = dev;
	isp->hw = of_device_get_match_data(dev);
	platform_set_drvdata(pdev, isp);
	dev_set_drvdata(dev, isp);

	/* Differences between firmware versions are rather minor so try to work
	 * with unknown firmware.
	 */
	isp->fw_compat = isp_check_firmware_version(isp);
	if (isp->hw->fw_abi != ISP_FW_ABI_H17 &&
	    (isp->fw_compat == ISP_FIRMWARE_V_26_6_2 ||
	     isp_read_fw_version(dev, "apple,firmware-version") ==
						ISP_FIRMWARE_V_26_6_2)) {
		if (!isp_preview26_admitted(dev, isp->fw_compat)) {
			err = dev_err_probe(dev, -ENODEV,
					    "26.6.2 preview requires J613/T8122; preview_26=0 disables it\n");
			goto free_isp;
		}
		isp->preview_26 = true;
	}

	err = of_property_read_u32(dev->of_node, "apple,platform-id",
				   &isp->platform_id);
	if (err) {
		dev_err(dev, "failed to get 'apple,platform-id' property: %d\n",
			err);
		goto free_isp;
	}

	err = of_property_read_u32(dev->of_node, "apple,temporal-filter",
				   &isp->temporal_filter);
	if (err)
		isp->temporal_filter = 0;

	/* the sensor connection layout, for firmware that takes it at boot */
	of_property_read_u32(dev->of_node, "apple,cam-connections-scheme",
			     &isp->cam_connections_scheme);

	err = apple_isp_init_presets(isp);
	if (err) {
		dev_err(dev, "failed to initialize presets\n");
		goto free_isp;
	}

	apple_isp_select_hw(isp);

	err = apple_isp_attach_genpd(isp);
	if (err) {
		dev_err(dev, "failed to attatch power domains\n");
		goto free_isp;
	}

	isp->coproc = devm_platform_ioremap_resource_byname(pdev, "coproc");
	if (IS_ERR(isp->coproc)) {
		err = PTR_ERR(isp->coproc);
		goto detach_genpd;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mbox");
	isp->mbox = devm_ioremap_resource(dev, res);
	if (IS_ERR(isp->mbox)) {
		err = PTR_ERR(isp->mbox);
		goto detach_genpd;
	}

	isp->gpio = devm_platform_ioremap_resource_byname(pdev, "gpio");
	if (IS_ERR(isp->gpio)) {
		err = PTR_ERR(isp->gpio);
		goto detach_genpd;
	}

	if (isp->hw->mbox2_offset) {
		if (resource_size(res) <
		    isp->hw->mbox2_offset + ISP_MBOX2_SIZE) {
			dev_err(dev, "mbox window too small for the doorbells\n");
			err = -EINVAL;
			goto detach_genpd;
		}
		isp->mbox2 = isp->mbox + isp->hw->mbox2_offset;
	} else {
		isp->mbox2 = devm_platform_ioremap_resource_byname(pdev,
								   "mbox2");
		if (IS_ERR(isp->mbox2)) {
			err = PTR_ERR(isp->mbox2);
			goto detach_genpd;
		}
	}

	if (isp->hw->num_dapf) {
		err = apple_isp_map_dapf(isp);
		if (err)
			goto detach_genpd;
	}

	/* Capture watchdog, where the ISP gates its frames on one */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wdt");
	if (res) {
		isp->wdt = devm_ioremap_resource(dev, res);
		if (IS_ERR(isp->wdt)) {
			err = PTR_ERR(isp->wdt);
			goto detach_genpd;
		}
		apple_isp_wdt_init(isp);
	}

	isp->irq = platform_get_irq(pdev, 0);
	if (isp->irq < 0) {
		err = isp->irq;
		goto detach_genpd;
	}
	if (!isp->irq) {
		err = -ENODEV;
		goto detach_genpd;
	}

	mutex_init(&isp->video_lock);
	INIT_WORK(&isp->resume_work, apple_isp_resume_work);
	apple_isp_firmware_init(isp);
	spin_lock_init(&isp->buf_lock);
	init_waitqueue_head(&isp->wait);
	INIT_LIST_HEAD(&isp->gc);
	INIT_LIST_HEAD(&isp->gc_released);
	mutex_init(&isp->gc_lock);
	INIT_LIST_HEAD(&isp->bufs_pending);
	INIT_LIST_HEAD(&isp->bufs_submitted);
	INIT_LIST_HEAD(&isp->bufs_returned);
	isp->wq = alloc_workqueue("apple-isp-wq", WQ_UNBOUND, 0);
	if (!isp->wq) {
		dev_err(dev, "failed to create workqueue\n");
		err = -ENOMEM;
		goto detach_genpd;
	}

	err = apple_isp_init_iommu(isp);
	if (err) {
		dev_err(dev, "failed to init iommu: %d\n", err);
		goto destroy_wq;
	}

	err = apple_isp_alloc_firmware_surface(isp);
	if (err) {
		dev_err(dev, "failed to alloc firmware surface: %d\n", err);
		goto free_iommu;
	}

	if (isp->hw->fw_restore) {
		err = apple_isp_copy_fw_data(isp);
		if (err) {
			dev_err(dev, "failed to copy the firmware data: %d\n", err);
			apple_isp_free_firmware_surface(isp);
			goto free_iommu;
		}
	}

	pm_runtime_enable(dev);

	if (!isp->hw->deferred_start) {
		err = apple_isp_detect_camera(isp);
		if (err) {
			dev_err(dev, "failed to detect camera: %d\n", err);
			goto free_surface;
		}
	}

	err = apple_isp_setup_video(isp);
	if (err) {
		dev_err(dev, "failed to register video device: %d\n", err);
		goto halt_firmware;
	}

	if (isp->hw->fw_idle_ms && isp->hw->resident_fw) {
		isp->pm_nb.notifier_call = apple_isp_pm_notify;
		register_pm_notifier(&isp->pm_nb);
	}

	return 0;

halt_firmware:
	apple_isp_firmware_halt(isp);
free_surface:
	pm_runtime_disable(dev);
	apple_isp_free_firmware_surface(isp);
free_iommu:
	apple_isp_free_iommu(isp);
destroy_wq:
	destroy_workqueue(isp->wq);
detach_genpd:
	apple_isp_detach_genpd(isp);
free_isp:
	kfree(isp->sensor_configs);
	/* Nothing else can reach it before the video device is registered. */
	kfree(isp);
	return err;
}

static void apple_isp_remove(struct platform_device *pdev)
{
	struct apple_isp *isp = platform_get_drvdata(pdev);

	if (isp->pm_nb.notifier_call)
		unregister_pm_notifier(&isp->pm_nb);
	cancel_work_sync(&isp->resume_work);
	apple_isp_remove_video(isp);
	/* Stop resident firmware before freeing what it may still use. */
	apple_isp_firmware_flush(isp);
	apple_isp_firmware_halt(isp);
	apple_isp_free_video(isp);
	pm_runtime_disable(isp->dev);
	apple_isp_free_firmware_surface(isp);
	apple_isp_free_iommu(isp);
	destroy_workqueue(isp->wq);
	apple_isp_detach_genpd(isp);
	/* Freed now, or when the last open file is closed. */
	v4l2_device_put(&isp->v4l2_dev);
}

static const struct apple_isp_hw apple_isp_hw_t8103 = {
	.gen = ISP_GEN_T8103,
	.pmu_base = 0x23b704000,

	.dsid_count = 4,
	.dsid_clr_base0 = 0x200014000,
	.dsid_clr_base1 = 0x200054000,
	.dsid_clr_base2 = 0x200094000,
	.dsid_clr_base3 = 0x2000d4000,
	.dsid_clr_range0 = 0x1000,
	.dsid_clr_range1 = 0x1000,
	.dsid_clr_range2 = 0x1000,
	.dsid_clr_range3 = 0x1000,

	.clock_scratch = 0x23b738010,
	.clock_base = 0x23bc3c000,
	.clock_bit = 0x1,
	.clock_size = 0x4,
	.bandwidth_scratch = 0x23b73800c,
	.bandwidth_base = 0x23bc3c000,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x4,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE,

	.scl1 = false,
	.lpdp = false,
	.meta_size = ISP_META_SIZE_T8103,
};

static const struct apple_isp_hw apple_isp_hw_t6000 = {
	.gen = ISP_GEN_T8103,
	.pmu_base = 0x28e584000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200014000,
	.dsid_clr_base1 = 0x200054000,
	.dsid_clr_base2 = 0x200094000,
	.dsid_clr_base3 = 0x2000d4000,
	.dsid_clr_range0 = 0x1000,
	.dsid_clr_range1 = 0x1000,
	.dsid_clr_range2 = 0x1000,
	.dsid_clr_range3 = 0x1000,

	.clock_scratch = 0x28e3d0868,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x28e3d0980,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE,

	.scl1 = false,
	.lpdp = false,
	.meta_size = ISP_META_SIZE_T8103,
};

static const struct apple_isp_hw apple_isp_hw_t8112 = {
	.gen = ISP_GEN_T8112,
	.pmu_base = 0x23b704000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200f14000,
	.dsid_clr_range0 = 0x1000,

	.clock_scratch = 0x23b3d0560,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x23b3d05d0,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE,

	.scl1 = false,
	.lpdp = false,
	.meta_size = ISP_META_SIZE_T8112,
};

static const struct apple_isp_hw apple_isp_hw_t6020 = {
	.gen = ISP_GEN_T8112,
	.pmu_base = 0x290284000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200f14000,
	.dsid_clr_range0 = 0x1000,

	.clock_scratch = 0x28e3d10a8,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x28e3d1200,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE,

	.scl1 = true,
	.lpdp = true,
	.meta_size = ISP_META_SIZE_T8112,
};

static const struct apple_isp_hw apple_isp_hw_t8122 = {
	.gen = ISP_GEN_T8112,
	.pmu_base = 0x2d0704000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x2221c4000,
	.dsid_clr_range0 = 0x1000,

	.clock_scratch = 0x2d03d08a0,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x2d03d0900,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE_T6031,

	.scl1 = true,
	.lpdp = true,
	.meta_size = ISP_META_SIZE_T6031,
};

static const struct apple_isp_hw apple_isp_hw_t6030 = {
	.gen = ISP_GEN_T6031,
	.pmu_base = 0x350704000,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200f14000,
	.dsid_clr_range0 = 0x1000,

	.clock_scratch = 0x3503d0920,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x3503d0980,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE_T6031,

	.scl1 = true,
	.lpdp = false,
	.meta_size = ISP_META_SIZE_T6031,
};

static const struct apple_isp_hw apple_isp_hw_t6031 = {
	.gen = ISP_GEN_T6031,
	.pmu_base = 0x292284008,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x200f14000,
	.dsid_clr_range0 = 0x1000,

	.clock_scratch = 0x0,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE_T6031,

	.scl1 = true,
	.lpdp = false,
	.meta_size = ISP_META_SIZE_T6031,
};

/*
 * Registers the T8140 firmware accesses at their physical addresses: the
 * bootloader's DART address filter ranges for the ISP, rounded out to
 * 16 KiB pages.
 */
static const struct isp_mmio_window apple_isp_fw_mmio_t8140[] = {
	{ 0x220004000, 0x14000 },	/* DSID broadcast-clear windows */
	{ 0x220044000, 0x14000 },
	{ 0x220084000, 0x14000 },
	{ 0x2200c4000, 0x14000 },
	{ 0x220104000, 0x14000 },
	{ 0x3003c0000, 0x28000 },	/* PMGR, with the PMP clock scratch */
	{ 0x300704000, 0x4000 },	/* PMGR, ISP power states */
	{ 0x300730000, 0x4000 },
	{ 0x300e3c000, 0x4000 },
	{ 0x302824000, 0x4000 },	/* PMP bandwidth scratch */
	{ 0x31062c000, 0x20000 },
	{ 0x401660000, 0x4000 },
};

static const struct apple_isp_hw apple_isp_hw_t8140 = {
	.gen = ISP_GEN_T8140,
	.fw_abi = ISP_FW_ABI_H17,
	.pmu_base = 0x0,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x220114000,
	.dsid_clr_range0 = 0x2fc,

	.clock_scratch = 0x3003d0ca0,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x302824000,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE_T6031,

	.scl1 = false,
	.lpdp = true,
	.meta_size = ISP_META_SIZE_T8140,

	.fw_iova_mask = GENMASK_ULL(35, 0),
	.mbox2_offset = 0x410,
	.mbox_irq_route = true,
	.coproc_control = ISP_COPROC_CONTROL_T8140,
	/*
	 * macOS quiesces the coprocessor before the core words go off
	 * (DisableISPClocksAndPower, 25G83); without it they reach off with BUSY
	 * latched.
	 */
	.coproc_status = ISP_COPROC_STATUS_T8140,
	.coproc_quiesce = ISP_COPROC_QUIESCE_T8140,
	.fw_mmio = apple_isp_fw_mmio_t8140,
	.num_fw_mmio = ARRAY_SIZE(apple_isp_fw_mmio_t8140),
	.boot_mode = 1,
	.capture_meta_size = ISP_CAPTURE_META_SIZE_T8140,
	.resident_fw = true,
	.fw_restart = true,
	/* As on macOS, the firmware first starts when a client opens the camera. */
	.deferred_start = true,
	/* macOS powers it off 5 s after the last frame (twice in its recording). */
	.fw_idle_ms = 5000,
	.meta_exposure = true,
};

/* The same for T6040, from the J616s ADT's DART address filter. */
static const struct isp_mmio_window apple_isp_fw_mmio_t6040[] = {
	{ 0x220004000, 0x1d4000 },	/* DSID broadcast-clear windows */
	{ 0x222004000, 0x1d4000 },
	{ 0x224004000, 0x1d4000 },
	{ 0x226004000, 0x1d4000 },
	{ 0x485660000, 0x4000 },
	{ 0x5003c0000, 0x28000 },	/* PMGR, with the PMP clock scratch */
	{ 0x500e3c000, 0x4000 },
	{ 0x502284000, 0x4000 },	/* PMGR, ISP power states */
	{ 0x502880000, 0x8000 },
	{ 0x504824000, 0x4000 },	/* PMP bandwidth scratch */
	{ 0x51062c000, 0x20000 },
};

/*
 * T6040 runs the same coprocessor and firmware interface as T8140, booted
 * as macOS boots it on the J616s (macOS 26.6.2 camera log, kext register
 * map) and as aurora's m1n1 proxy captures with it: no coprocessor reset,
 * four broadcast-clear windows in eight slots, the scaler 1 output path and
 * no capture metadata pool or watchdog.
 *
 * Its firmware starts again from its own memory, as T8140's does, so the
 * ISP is powered only while the camera is in use: the firmware starts at
 * probe to detect the camera and stops again, and each stream starts it.
 * It stays up for 5 s after a stream, as on T8140, and the coprocessor is
 * quiesced before its domains go off.
 */
static const struct apple_isp_hw apple_isp_hw_t6040 = {
	.gen = ISP_GEN_T6040,
	.fw_abi = ISP_FW_ABI_H17,
	.pmu_base = 0x0,

	.dsid_count = 4,
	.dsid_slots = 8,
	.dsid_clr_base0 = 0x2201d4000,
	.dsid_clr_base1 = 0x2221d4000,
	.dsid_clr_base2 = 0x2241d4000,
	.dsid_clr_base3 = 0x2261d4000,
	.dsid_clr_range0 = 0x2fc,

	/* the PMGR and PMP scratch words of the ISP's DAPF windows */
	.clock_scratch = 0x5003d0fa8,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x504824000,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE_T6031,

	.scl1 = true,
	.lpdp = false,
	.meta_size = ISP_META_SIZE_T8140,
	.fw_mmio = apple_isp_fw_mmio_t6040,
	.num_fw_mmio = ARRAY_SIZE(apple_isp_fw_mmio_t6040),

	.coproc_control = ISP_COPROC_CONTROL_T8140,
	.coproc_status = ISP_COPROC_STATUS_T8140,
	.coproc_quiesce = ISP_COPROC_QUIESCE_T8140,
	.boot_mode = 1,
	.shared_size = 0xde600000,
	.resident_fw = true,
	.fw_restart = true,
	.fw_idle_ms = 5000,
	.meta_exposure = true,
};

/* T8132: the firmware's MMIO windows, from the J713 ADT's DART address filter */
static const struct isp_mmio_window apple_isp_fw_mmio_t8132[] = {
	{ 0x220004000, 0x1d4000 },	/* DSID broadcast-clear windows */
	{ 0x222004000, 0x1d4000 },
	{ 0x3803c0000, 0x28000 },	/* PMGR, with the PMP clock scratch */
	{ 0x380704000, 0x4000 },	/* PMGR, ISP power states */
	{ 0x380730000, 0x4000 },
	{ 0x380e3c000, 0x4000 },
	{ 0x382824000, 0x4000 },	/* PMP bandwidth scratch */
	{ 0x39062c000, 0x20000 },
	{ 0x501660000, 0x4000 },
};

/* The ADT's DART address filter windows, in its order and with its ends */
static const struct isp_dapf_window apple_isp_dapf_t8132[] = {
	{ 0x380704000, 0x380704063 },	/* PMGR, ISP power states */
	{ 0x380730000, 0x380733fff },
	{ 0x380e3c000, 0x380e3c003 },
	{ 0x39062c000, 0x390648003 },
	{ 0x501660000, 0x501660003 },
	{ 0x220004000, 0x2200162fc },	/* DSID broadcast-clear windows */
	{ 0x220044000, 0x2200562fc },
	{ 0x2201c4000, 0x2201d62fc },
	{ 0x222004000, 0x2220162fc },
	{ 0x222044000, 0x2220562fc },
	{ 0x2221c4000, 0x2221d62fc },
	{ 0x3803c0000, 0x3803e4000 },	/* PMGR, with the PMP clock scratch */
	{ 0x382824000, 0x382824004 },	/* PMP bandwidth scratch */
};

/*
 * T8132 runs T6040's coprocessor and firmware interface, booted the same
 * way. Its two AMCCs (ADT mcc amcc_aperture_count) give two broadcast-clear
 * windows, passed one per AMCC without padding slots. No PMP_CTRL_SET is
 * sent: T8132's PMP scratch offsets are not established. Its firmware
 * starts again from its data segment as loaded once the coprocessor was
 * quiesced before the power-down, so the ISP is powered only while the
 * camera is in use.
 */
static const struct apple_isp_hw apple_isp_hw_t8132 = {
	.gen = ISP_GEN_T6040,
	.fw_abi = ISP_FW_ABI_H17,
	.pmu_base = 0x0,

	.dsid_count = 2,
	.dsid_clr_base0 = 0x2201d4000,
	.dsid_clr_base1 = 0x2221d4000,
	.dsid_clr_range0 = 0x2fc,

	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE_T6031,

	.scl1 = true,
	.lpdp = false,
	.meta_size = ISP_META_SIZE_T8140,
	.fw_mmio = apple_isp_fw_mmio_t8132,
	.num_fw_mmio = ARRAY_SIZE(apple_isp_fw_mmio_t8132),
	.dapf = apple_isp_dapf_t8132,
	.num_dapf = ARRAY_SIZE(apple_isp_dapf_t8132),

	.coproc_control = ISP_COPROC_CONTROL_T8140,
	.coproc_status = ISP_COPROC_STATUS_T8140,
	.coproc_quiesce = ISP_COPROC_QUIESCE_T8140,
	.boot_mode = 1,
	.shared_size = 0xdec00000,
	.fw_restore = true,
	.deferred_start = true,
};

/*
 * T8142 (M5, J813): the Neo's ISP17a. The J813 ADT's dart-isp
 * dapf-instance-0 ranges, rounded out to 16 KiB pages; mostly the T8140
 * windows at +0x80000000, with one DSID window and two new ranges.
 */
static const struct isp_mmio_window apple_isp_fw_mmio_t8142[] = {
	{ 0x220104000, 0x14000 },	/* DSID broadcast-clear window */
	{ 0x3803c0000, 0x28000 },	/* PMGR, with the PMP clock scratch */
	{ 0x380704000, 0x4000 },	/* PMGR, ISP power states */
	{ 0x380730000, 0x4000 },
	{ 0x380e3c000, 0x4000 },
	{ 0x382724000, 0x4000 },	/* PMP bandwidth scratch */
	{ 0x39062c000, 0x30000 },
	{ 0x3a1340000, 0x4000 },
	{ 0x3a1358000, 0xc000 },
	{ 0x421660000, 0x4000 },
};

static const struct apple_isp_hw apple_isp_hw_t8142 = {
	.gen = ISP_GEN_T8140,
	.fw_abi = ISP_FW_ABI_H17,
	.pmu_base = 0x0,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x220116000,
	.dsid_clr_range0 = 0x2fc,

	.clock_scratch = 0x3803d0ca0,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x382724000,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE_T6031,

	/*
	 * No BES scaler: the firmware asserts on CH_CROP_SET ("BesProc is
	 * removed/not present. Must config AuxSclPr") and takes the SCL1 set.
	 */
	.scl1 = true,
	.lpdp = true,
	.meta_size = ISP_META_SIZE_T8140,

	.fw_iova_mask = GENMASK_ULL(35, 0),
	.mbox2_offset = 0x410,
	.mbox_irq_route = true,
	.coproc_control = ISP_COPROC_CONTROL_T8140,
	.coproc_status = ISP_COPROC_STATUS_T8140,
	.coproc_quiesce = ISP_COPROC_QUIESCE_T8140,
	.fw_mmio = apple_isp_fw_mmio_t8142,
	.num_fw_mmio = ARRAY_SIZE(apple_isp_fw_mmio_t8142),
	.boot_mode = 1,
	.capture_meta_size = ISP_CAPTURE_META_SIZE_T8140,
	.resident_fw = true,
	/*
	 * As on T6040: the firmware starts at probe to detect the camera and
	 * stops again, each stream starts it from its own memory, and it stays
	 * up for 5 s after a stream. The coprocessor is quiesced before its
	 * domains go off.
	 */
	.fw_restart = true,
	.fw_idle_ms = 5000,
	/* The J813 IMX958 runs the xx24 setfile 2224_01, as T6040 does. */
	.setfile_stride_fixup = true,
};

/* T6050 ISP17c: CPU-visible firmware MMIO windows and DART filters. */
static const struct isp_mmio_window apple_isp_fw_mmio_t6050[] = {
	{ 0x280680000, 0x8000 },
	{ 0x280900000, 0x4000 },
	{ 0x284240000, 0x28000 },
	{ 0x284e3c000, 0x4000 },
	{ 0x286724000, 0x4000 },
	{ 0x29062c000, 0x20000 },
	{ 0x509660000, 0x4000 },
	{ 0x2220004000, 0x14000 },
	{ 0x2222004000, 0x14000 },
	{ 0x2224004000, 0x14000 },
	{ 0x2226004000, 0x14000 },
	{ 0x2228004000, 0x14000 },
	{ 0x222a004000, 0x14000 },
	{ 0x222c004000, 0x14000 },
	{ 0x222e004000, 0x14000 },
};

static const struct isp_dapf_window apple_isp_dapf_t6050[] = {
	{ 0x2809003b0, 0x2809003f3 },
	{ 0x280680000, 0x280687fff },
	{ 0x284e3c000, 0x284e3c003 },
	{ 0x29062c000, 0x290648003 },
	{ 0x509660000, 0x509660003 },
	{ 0x286724000, 0x286724004 },
	{ 0x284240000, 0x284264000 },
	{ 0x2220004000, 0x22200162fc },
	{ 0x2222004000, 0x22220162fc },
	{ 0x2224004000, 0x22240162fc },
	{ 0x2226004000, 0x22260162fc },
	{ 0x2228004000, 0x22280162fc },
	{ 0x222a004000, 0x222a0162fc },
	{ 0x222c004000, 0x222c0162fc },
	{ 0x222e004000, 0x222e0162fc },
};

static const u64 apple_isp_dsid_t6050[] = {
	0x2220016000,
	0x2222016000,
	0x2224016000,
	0x2226016000,
	0x2228016000,
	0x222a016000,
	0x222c016000,
	0x222e016000,
};

static const struct apple_isp_hw apple_isp_hw_t6050 = {
	.gen = ISP_GEN_T6040,
	.fw_abi = ISP_FW_ABI_H17,
	.dsid_count = ARRAY_SIZE(apple_isp_dsid_t6050),
	.dsid_clr_bases = apple_isp_dsid_t6050,
	.dsid_clr_range0 = 0x2fc,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE_T6031,
	.mbox_extra_enables = true,
	.scl1 = true,
	.meta_size = ISP_META_SIZE_T8140,
	.capture_meta_size = ISP_CAPTURE_META_SIZE_T8140,
	.fw_mmio = apple_isp_fw_mmio_t6050,
	.num_fw_mmio = ARRAY_SIZE(apple_isp_fw_mmio_t6050),
	.dapf = apple_isp_dapf_t6050,
	.num_dapf = ARRAY_SIZE(apple_isp_dapf_t6050),
	.dapf_sidmask = 0xcc3,
	.coproc_control = ISP_COPROC_CONTROL_T8140,
	.coproc_status = ISP_COPROC_STATUS_T8140,
	.coproc_quiesce = ISP_COPROC_QUIESCE_T8140,
	.boot_mode = 1,
	.shared_size = 0xde000000,
	.fw_restore = true,
	.deferred_start = true,
	.setfile_stride_fixup = true,
};

static const struct of_device_id apple_isp_of_match[] = {
	{ .compatible = "apple,t6050-isp", .data = &apple_isp_hw_t6050 },
	{ .compatible = "apple,t8103-isp", .data = &apple_isp_hw_t8103 },
	{ .compatible = "apple,t8112-isp", .data = &apple_isp_hw_t8112 },
	{ .compatible = "apple,t8122-isp", .data = &apple_isp_hw_t8122 },
	{ .compatible = "apple,t6000-isp", .data = &apple_isp_hw_t6000 },
	{ .compatible = "apple,t6020-isp", .data = &apple_isp_hw_t6020 },
	{ .compatible = "apple,t6030-isp", .data = &apple_isp_hw_t6030 },
	{ .compatible = "apple,t6031-isp", .data = &apple_isp_hw_t6031 },
	{ .compatible = "apple,t8140-isp", .data = &apple_isp_hw_t8140 },
	{ .compatible = "apple,t8142-isp", .data = &apple_isp_hw_t8142 },
	{ .compatible = "apple,t6041-isp", .data = &apple_isp_hw_t6040 },
	{ .compatible = "apple,t8132-isp", .data = &apple_isp_hw_t8132 },
	{},
};
MODULE_DEVICE_TABLE(of, apple_isp_of_match);

static __maybe_unused int apple_isp_runtime_suspend(struct device *dev)
{
	/* RPM sleep is called when the V4L2 file handle is closed */
	return 0;
}

static __maybe_unused int apple_isp_runtime_resume(struct device *dev)
{
	return 0;
}

/*
 * Stop resident firmware. For a system sleep it starts again on resume where
 * it can; otherwise, or for hibernation, the camera is gone until the next
 * boot.
 */
static void apple_isp_stop_resident(struct apple_isp *isp, bool sleep)
{
	bool restart = sleep && isp->hw->fw_restart;

	if (!isp->hw->resident_fw)
		return;

	mutex_lock(&isp->video_lock);
	if (isp->fw_state == ISP_FW_RUNNING) {
		if (restart)
			dev_dbg(isp->dev, "stopping the firmware for system sleep\n");
		else
			dev_warn(isp->dev, "stopping the firmware for %s, the camera is unavailable until the next boot\n",
				 sleep ? "system sleep" : "hibernation");
		isp->fw_sleep_stopped = restart;
	}
	apple_isp_firmware_halt(isp);
	/* Hibernation resumes in another boot, where that memory isn't ours. */
	if (!restart)
		isp->fw_state = ISP_FW_DEAD;
	mutex_unlock(&isp->video_lock);
}

static __maybe_unused int apple_isp_suspend(struct device *dev)
{
	struct apple_isp *isp = dev_get_drvdata(dev);
	bool power_lost = isp->hw->resident_fw && pm_suspend_via_firmware();

	/* A stream that couldn't restart on the other side keeps the system up. */
	if (power_lost && !isp->hw->fw_restart &&
	    test_bit(ISP_STATE_STREAMING, &isp->state))
		return -EBUSY;

	/* We must restore V4L2 context on system resume. If we were streaming
	 * before, we (essentially) stop streaming and start streaming again.
	 */
	apple_isp_video_suspend(isp);

	/*
	 * Resident firmware stays up, idle, across suspend-to-idle: its domains
	 * stay on, see apple_isp_attach_genpd(). A sleep that powers the SoC down
	 * takes it anyway, so stop it first. Firmware that starts again and still
	 * idles after a stream stops.
	 */
	if (power_lost)
		apple_isp_stop_resident(isp, true);
	apple_isp_firmware_flush(isp);

	/*
	 * A stream stopped for suspend-to-idle has stopped firmware that starts
	 * again, too. Its start from .resume never gets the firmware's first
	 * boot message; from process context it does. Restart it there, as
	 * after a sleep that powered the SoC down.
	 */
	mutex_lock(&isp->video_lock);
	if (isp->hw->fw_restart && isp->fw_state == ISP_FW_OFF &&
	    test_bit(ISP_STATE_SLEEPING, &isp->state))
		isp->fw_sleep_stopped = true;
	mutex_unlock(&isp->video_lock);
	return 0;
}

static __maybe_unused int apple_isp_freeze(struct device *dev)
{
	struct apple_isp *isp = dev_get_drvdata(dev);

	apple_isp_video_suspend(isp);
	apple_isp_firmware_flush(isp);

	/*
	 * Hibernation hands the system from one kernel to another, and
	 * firmware left running would go on using the memory of the kernel
	 * that started it. Stop it cleanly instead; if it ran, the camera is
	 * then unavailable until the next system boot.
	 */
	apple_isp_stop_resident(isp, false);

	return 0;
}

static __maybe_unused int apple_isp_resume(struct device *dev)
{
	struct apple_isp *isp = dev_get_drvdata(dev);

	/*
	 * Firmware stopped for the sleep starts again in process context: its
	 * handshake here held up the whole resume. A stream that ran across the
	 * sleep restarts from a freezable work item, once tasks thaw; without one
	 * the firmware starts at the next stream.
	 */
	if (isp->fw_sleep_stopped) {
		queue_work(system_freezable_wq, &isp->resume_work);
		return 0;
	}
	apple_isp_video_resume(isp);

	return 0;
}

static const struct dev_pm_ops apple_isp_pm_ops = {
	.suspend	= pm_sleep_ptr(apple_isp_suspend),
	.resume		= pm_sleep_ptr(apple_isp_resume),
	.freeze		= pm_sleep_ptr(apple_isp_freeze),
	.thaw		= pm_sleep_ptr(apple_isp_resume),
	.poweroff	= pm_sleep_ptr(apple_isp_freeze),
	.restore	= pm_sleep_ptr(apple_isp_resume),
	RUNTIME_PM_OPS(apple_isp_runtime_suspend, apple_isp_runtime_resume, NULL)
};

/*
 * Stop firmware that starts again (hw->fw_restore) before a reboot or a
 * kexec, so that the next kernel finds its data segment as loaded.
 */
static void apple_isp_shutdown(struct platform_device *pdev)
{
	struct apple_isp *isp = platform_get_drvdata(pdev);

	if (!isp || !isp->hw->fw_restore)
		return;
	/* A stream that still runs is stopped cleanly first, as for hibernation */
	apple_isp_video_suspend(isp);
	apple_isp_firmware_flush(isp);
	mutex_lock(&isp->video_lock);
	apple_isp_firmware_halt(isp);
	mutex_unlock(&isp->video_lock);
}

static struct platform_driver apple_isp_driver = {
	.driver	= {
		.name		= "apple-isp",
		.of_match_table	= apple_isp_of_match,
		.pm		= pm_ptr(&apple_isp_pm_ops),
	},
	.probe	= apple_isp_probe,
	.remove	= apple_isp_remove,
	.shutdown = apple_isp_shutdown,
};
module_platform_driver(apple_isp_driver);

MODULE_AUTHOR("Eileen Yoon <eyn@gmx.com>");
MODULE_DESCRIPTION("Apple ISP driver");
MODULE_LICENSE("GPL v2");

#if IS_ENABLED(CONFIG_VIDEO_APPLE_ISP_KUNIT_TEST)
#include "isp-drv-test.c"
#endif
