// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * G15 (T6030/T6031, M3 Pro/Max) helpers that have no Rust bindings: static inlines,
 * macros and THIS_MODULE.
 */
#include <crypto/sha2.h>
#include <linux/device.h>
#include <linux/dma-map-ops.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>

void asahi_g15_sha256(const u8 *data, size_t len, u8 *out);
bool asahi_g15_board(void);
bool asahi_g15_machine(const char *compatible);
void asahi_g15_hold_power(struct device *dev);
void asahi_g15_release_power(struct device *dev);
void asahi_g15_module_get(void);
int asahi_g15_adt_phram(u64 *start, u64 *size);
bool asahi_g15_ram_disjoint(u64 start, u64 size);
bool asahi_g15_ram_within(u64 start, u64 size);
void __iomem *asahi_g15_ioremap_wc(u64 start, size_t size);
bool asahi_g15_dma_coherent(struct device *dev);

/* SHA-256 of the firmware TEXT and the board templates. */
void asahi_g15_sha256(const u8 *data, size_t len, u8 *out)
{
	sha256(data, len, out);
}

/* Whether this is a J514S/J516S T6030 board (the boards the generation was qualified on). */
bool asahi_g15_board(void)
{
	return of_machine_is_compatible("apple,j514s") > 0 ||
	       of_machine_is_compatible("apple,j516s") > 0;
}

/* Whether the machine carries this compatible (of_machine_is_compatible()). */
bool asahi_g15_machine(const char *compatible)
{
	return of_machine_is_compatible(compatible) > 0;
}

/*
 * Keep the GFX domain powered for the whole (retained) session. Never released,
 * like the firmware session itself.
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
 */
void asahi_g15_release_power(struct device *dev)
{
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
}

/* Pin this module until reboot. */
void asahi_g15_module_get(void)
{
	__module_get(THIS_MODULE);
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

/* The range is disjoint from System RAM (ADT region admission). */
bool asahi_g15_ram_disjoint(u64 start, u64 size)
{
	return region_intersects(start, size, IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) ==
	       REGION_DISJOINT;
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

/* The range lies entirely in System RAM (a reserved-memory region the kernel maps). */
bool asahi_g15_ram_within(u64 start, u64 size)
{
	return region_intersects(start, size, IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) ==
	       REGION_INTERSECTS;
}

/* The inherited RTKit buffer inside firmware DATA. */
void __iomem *asahi_g15_ioremap_wc(u64 start, size_t size)
{
	return ioremap_wc(start, size);
}
