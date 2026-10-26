// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * G15 (T6030/T6031, M3 Pro/Max) helpers that have no Rust bindings: static inlines,
 * macros and THIS_MODULE. Built only with CONFIG_DRM_ASAHI_G15.
 */
#include <crypto/sha2.h>
#include <linux/device.h>
#include <linux/device/driver.h>
#include <linux/platform_device.h>
#include <linux/dma-map-ops.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/overflow.h>
#include <linux/page-flags.h>
#include <linux/pfn.h>
#include <linux/pm_runtime.h>
#include <linux/soc/apple/pmp.h>
#include <linux/soc/apple/display.h>

void asahi_g15_sha256(const u8 *data, size_t len, u8 *out);
bool asahi_g15_board(void);
bool asahi_g15_machine(const char *compatible);
void asahi_g15_hold_power(struct device *dev);
void asahi_g15_release_power(struct device *dev);
bool asahi_g15_module_try_get(void);
void asahi_g15_module_put(void);
void asahi_g15_reprobe(void);
void asahi_g15_report_loader_gpu(void);
int asahi_g15_adt_phram(u64 *start, u64 *size);
void __iomem *asahi_g15_ioremap_wc(u64 start, size_t size);
bool asahi_g15_dma_coherent(struct device *dev);
int asahi_g15_pmp_prepare(struct device *dev);
int asahi_g15_pmp_get(struct device *dev);
int asahi_g15_pmp_ready(void);
void asahi_g15_pmp_put(struct device *dev);
bool asahi_g15_reserved_uat(u32 index, u64 start, u64 size);

/*
 * The boot loader enables the T6030/T6031 GPU node only when its
 * chosen.asahi,t603x-gpu setting is on; otherwise the node stays disabled,
 * the adapter never probes and nothing says why native G15 is absent. Name
 * the setting once, at module activation, on a machine whose /chosen has it
 * and whose GPU node is disabled. Nothing else changes.
 */
void asahi_g15_report_loader_gpu(void)
{
	static const struct {
		const char *prop;
		const char *compatible;
	} socs[] = {
		{ "asahi,t6031-gpu", "apple,agx-t6031" },
		{ "asahi,t6030-gpu", "apple,agx-t6030" },
	};
	struct device_node *chosen, *np;
	const char *value;
	unsigned int i;
	u32 cell;

	chosen = of_find_node_by_path("/chosen");
	if (!chosen)
		return;
	for (i = 0; i < ARRAY_SIZE(socs); i++) {
		if (!of_property_present(chosen, socs[i].prop))
			continue;
		np = of_find_compatible_node(NULL, NULL, socs[i].compatible);
		if (np && !of_device_is_available(np)) {
			if (!of_property_read_string(chosen, socs[i].prop, &value))
				pr_info("asahi: G15: native G15 not started: the boot loader left the GPU node %pOF disabled (/chosen/%s = \"%s\")\n",
					np, socs[i].prop, value);
			else if (!of_property_read_u32(chosen, socs[i].prop, &cell))
				pr_info("asahi: G15: native G15 not started: the boot loader left the GPU node %pOF disabled (/chosen/%s = %u)\n",
					np, socs[i].prop, cell);
		}
		of_node_put(np);
	}
	of_node_put(chosen);
}

/*
 * The maintained m1n1 handoff keeps these four UAT regions in the linear map.
 * Require the exact enabled reserved-memory owner, in addition to the ADT
 * bounds/size/disjointness and per-page PageReserved checks in admission.
 */
bool asahi_g15_reserved_uat(u32 index, u64 start, u64 size)
{
	static const char *const names[] = {
		"uat-ttbs",
		"uat-pagetables",
		"uat-handoff",
		"uat-pagetables-l2",
	};
	struct device_node *rmem, *child, *np = NULL;
	struct resource res;
	unsigned int count = 0;
	bool match;

	if (index >= ARRAY_SIZE(names))
		return false;

	/* Unit addresses belong to the DT layout, not the reservation name. */
	rmem = of_find_node_by_path("/reserved-memory");
	for_each_child_of_node(rmem, child) {
		if (!of_node_name_eq(child, names[index]))
			continue;
		count++;
		if (!np)
			np = of_node_get(child);
	}
	of_node_put(rmem);
	match = count == 1 && np && of_device_is_available(np) &&
		!of_property_read_bool(np, "reusable") &&
		!of_get_address(np, 1, NULL, NULL) &&
		!of_address_to_resource(np, 0, &res) &&
		res.start == start && resource_size(&res) == size;
	of_node_put(np);
	return match;
}

/*
 * Keep a single PMP RTKit owner. The maintained bridge serializes requests,
 * waits for supplier readiness and acknowledgement, and latches lost replies.
 * The GPU retains both its module and supplier dependency until reboot.
 */
int asahi_g15_pmp_prepare(struct device *dev)
{
	int ret;

	if (!asahi_g15_board() || !dev->of_node ||
	    !of_device_is_compatible(dev->of_node, "apple,agx-t6030"))
		return -ENODEV;
	/* DISP ACK alone does not prove the native DRM component set is up. */
	/* The direct import waits for the display module to become LIVE; a
	 * symbol_get during its probe could defer without any post-LIVE retry.
	 */
	ret = apple_drm_t6030_link_device(dev);
	if (ret)
		return ret;
	return apple_pmp_link_device(dev);
}

int asahi_g15_pmp_get(struct device *dev)
{
	if (!asahi_g15_board() || !dev->of_node ||
	    !of_device_is_compatible(dev->of_node, "apple,agx-t6030"))
		return -ENODEV;
	return apple_pmp_set_device_power(0x0f, 5, 1);

}

int asahi_g15_pmp_ready(void)
{
	return apple_pmp_t6030_gpu_ready();
}

/*
 * Give back the vote of asahi_g15_pmp_get() after a start that failed before
 * the firmware was touched. The full M3 stack's J514S bridge has no release:
 * its vote stays, as before.
 */
void asahi_g15_pmp_put(struct device *dev)
{
	int ret = apple_pmp_set_device_power(0x0f, 5, 0);

	if (ret)
		dev_warn(dev, "G15: PMP AGX vote release failed: %d\n", ret);

}

/* SHA-256 of the firmware TEXT and the board templates. */
void asahi_g15_sha256(const u8 *data, size_t len, u8 *out)
{
	sha256(data, len, out);
}

/* Whether this is a J514S/J516S T6030 board. */
bool asahi_g15_board(void)
{
	return of_machine_is_compatible("apple,t6030") &&
	       (of_machine_is_compatible("apple,j514s") ||
		of_machine_is_compatible("apple,j516s"));
}

/* Whether the machine carries this compatible (of_machine_is_compatible()). */
bool asahi_g15_machine(const char *compatible)
{
	return of_machine_is_compatible(compatible) > 0;
}

/*
 * Keep the GFX domain powered for the whole (retained) session, like the
 * firmware session itself; released only by a start that failed before the
 * firmware was touched (asahi_g15_release_power()).
 */
void asahi_g15_hold_power(struct device *dev)
{
	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
}

/*
 * Give the hold of asahi_g15_hold_power() back: the start failed before
 * anything firmware-visible was written, so the session is not retained.
 * Only where the AGX vote is given back as well (asahi_g15_pmp_put()): the
 * full M3 stack's J514S bridge keeps its vote, and with it the hold stays.
 */
void asahi_g15_release_power(struct device *dev)
{
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);

}

/*
 * Pin this module until reboot, unless its removal has begun: a deferred or
 * asynchronous probe can run after an ordinary rmmod has passed its reference
 * check, which __module_get() would not stop.
 */
bool asahi_g15_module_try_get(void)
{
	return try_module_get(THIS_MODULE);
}

/* Drop a reference asahi_g15_module_try_get() took. */
void asahi_g15_module_put(void)
{
	module_put(THIS_MODULE);
}

/*
 * The boot loader's phram "adt" export: the first /reserved-memory child
 * compatible with "phram" and labelled "adt".
 */
int asahi_g15_adt_phram(u64 *start, u64 *size)
{
	struct device_node *rmem, *np;
	struct resource res;
	int ret = -ENODEV;

	rmem = of_find_node_by_path("/reserved-memory");
	for_each_child_of_node(rmem, np) {
		const char *label;

		if (of_device_is_compatible(np, "phram") &&
		    !of_property_read_string(np, "label", &label) && !strcmp(label, "adt") &&
		    !of_address_to_resource(np, 0, &res)) {
			of_node_put(np);
			ret = 0;
			break;
		}
	}
	of_node_put(rmem);
	if (ret)
		return ret;
	*start = res.start;
	*size = resource_size(&res);
	return 0;
}

/*
 * Whether DMA mappings of the device skip cache maintenance. The G15 firmware
 * objects rely on the DMA mapping cleaning their pages to the point of
 * coherency, so the device must not be DMA-coherent.
 */
bool asahi_g15_dma_coherent(struct device *dev)
{
	return dev_is_dma_coherent(dev);
}

/* The inherited RTKit buffer inside firmware DATA. */
void __iomem *asahi_g15_ioremap_wc(u64 start, size_t size)
{
	return ioremap_wc(start, size);
}

/*
 * Called only by the final infallible DriverModule initializer, after both
 * platform registrations have completed. driver_find() returns a borrowed
 * pointer, not an owned reference. The pinned registration is still owned by
 * DriverModule throughout this call; module exit cannot race its initialization.
 * No put_driver() is appropriate. A failed attach must not fail module init:
 * another device in this pass may already have started retained firmware.
 */
void asahi_g15_reprobe(void)
{
	const struct device_driver *drv;
	int ret;

	drv = driver_find("asahi-g15", &platform_bus_type);
	if (!drv) {
		pr_err("asahi-g15: registered adapter not found for activation\n");
		return;
	}
	ret = driver_attach(drv);
	if (ret)
		pr_err("asahi-g15: activation attach failed: %d\n", ret);
}
