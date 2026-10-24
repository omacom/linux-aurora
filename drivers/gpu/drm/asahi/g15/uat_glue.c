// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * C helpers of the UAT adoption (g15/uat.rs, g15/regs.rs), built for the G15
 * backend and for the T8122 runtime, which adopts the firmware's UAT the same
 * way (CONFIG_DRM_ASAHI_UAT_ADOPTION).
 */
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/page-flags.h>
#include <linux/pfn.h>

bool asahi_g15_ram_disjoint(u64 start, u64 size);
bool asahi_g15_ram_within(u64 start, u64 size);
unsigned int asahi_g15_memory_mode(u64 start, u64 size);

/* The range is disjoint from System RAM (ADT region admission). */
bool asahi_g15_ram_disjoint(u64 start, u64 size)
{
	return region_intersects(start, size, IORESOURCE_SYSTEM_RAM, IORES_DESC_NONE) ==
	       REGION_DISJOINT;
}

/*
 * Every page must be valid, System RAM and still reserved from the allocator.
 * REGION_INTERSECTS alone also accepts RAM mixed with undefined memory holes.
 */
bool asahi_g15_ram_within(u64 start, u64 size)
{
	u64 end, pa;

	if (!size || !IS_ALIGNED(start, PAGE_SIZE) ||
	    !IS_ALIGNED(size, PAGE_SIZE) || check_add_overflow(start, size, &end))
		return false;
	for (pa = start; pa < end; pa += PAGE_SIZE) {
		unsigned long pfn = PHYS_PFN(pa);

		if (!pfn_valid(pfn) || !page_is_ram(pfn) ||
		    !PageReserved(pfn_to_page(pfn)))
			return false;
	}
	return true;
}

/* One classification for every adopted-UAT and firmware-region CPU mapping. */
unsigned int asahi_g15_memory_mode(u64 start, u64 size)
{
	u64 end;

	if (!size || !IS_ALIGNED(start, PAGE_SIZE) ||
	    !IS_ALIGNED(size, PAGE_SIZE) || check_add_overflow(start, size, &end))
		return 0;
	if (asahi_g15_ram_within(start, size))
		return MEMREMAP_WB;
	if (asahi_g15_ram_disjoint(start, size))
		return MEMREMAP_WC;
	return 0;
}
