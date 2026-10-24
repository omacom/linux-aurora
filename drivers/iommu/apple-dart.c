// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple DART (Device Address Resolution Table) IOMMU driver
 *
 * Copyright (C) 2021 The Asahi Linux Contributors
 *
 * Based on arm/arm-smmu/arm-ssmu.c and arm/arm-smmu-v3/arm-smmu-v3.c
 *  Copyright (C) 2013 ARM Limited
 *  Copyright (C) 2015 ARM Limited
 * and on exynos-iommu.c
 *  Copyright (c) 2011,2016 Samsung Electronics Co., Ltd.
 */

#include <kunit/static_stub.h>
#include <linux/align.h>
#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io-pgtable.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/minmax.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/soc/apple/dart.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/overflow.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/soc/apple/tunable.h>
#include <linux/swab.h>
#include <linux/types.h>

#include "dma-iommu.h"

#if IS_ENABLED(CONFIG_APPLE_DART_KUNIT_TEST)
#include <kunit/static_stub.h>
#endif

#if IS_ENABLED(CONFIG_KUNIT) && !defined(MODULE)
#include <kunit/test.h>
#endif

#define DART_MAX_STREAMS 256
#define DART_MAX_TTBR 4
#define MAX_DARTS_PER_DEVICE 8
#define DART_PCIEC_PAGE_SHIFT 14
#define DART_PCIEC_ADDR_WIDTH 42
#define DART_PCIEC_STREAMS 64

/* Common registers */

#define DART_PARAMS1 0x00
#define DART_PARAMS1_PAGE_SHIFT GENMASK(27, 24)

#define DART_PARAMS2 0x04
#define DART_PARAMS2_BYPASS_SUPPORT BIT(0)

/* T8020/T6000 registers */

#define DART_T8020_STREAM_COMMAND 0x20
#define DART_T8020_STREAM_COMMAND_BUSY BIT(2)
#define DART_T8020_STREAM_COMMAND_INVALIDATE BIT(20)

#define DART_T8020_STREAM_SELECT 0x34

#define DART_T8020_ERROR 0x40
#define DART_T8020_ERROR_STREAM GENMASK(27, 24)
#define DART_T8020_ERROR_CODE GENMASK(11, 0)
#define DART_T8020_ERROR_FLAG BIT(31)

#define DART_T8020_ERROR_READ_FAULT BIT(4)
#define DART_T8020_ERROR_WRITE_FAULT BIT(3)
#define DART_T8020_ERROR_NO_PTE BIT(2)
#define DART_T8020_ERROR_NO_PMD BIT(1)
#define DART_T8020_ERROR_NO_TTBR BIT(0)

#define DART_T8020_CONFIG 0x60
#define DART_T8020_CONFIG_LOCK BIT(15)

#define DART_STREAM_COMMAND_BUSY_TIMEOUT 100

#define DART_T8020_ERROR_ADDR_HI 0x54
#define DART_T8020_ERROR_ADDR_LO 0x50

#define DART_T8020_STREAMS_ENABLE 0xfc

#define DART_T8020_REMAP       	0x80

#define DART_REMAP_MASK(sid)  GENMASK(7 + (8 * (sid % 4)), 8 * (sid % 4))
#define DART_REMAP(dart, sid) ((dart)->regs + DART_T8020_REMAP + ((sid) & ~0x3))

#define DART_T8020_TCR                  0x100
#define DART_T8020_TCR_TRANSLATE_ENABLE BIT(7)
#define DART_T8020_TCR_BYPASS_DART      BIT(8)
#define DART_T8020_TCR_BYPASS_DAPF      BIT(12)
#define DART_T8020_TCR_REMAP_ENABLE     BIT(24)

#define DART_T8020_TTBR       0x200
#define DART_T8020_USB4_TTBR  0x400
#define DART_T8020_TTBR_VALID BIT(31)
#define DART_T8020_TTBR_ADDR_FIELD_SHIFT 0
#define DART_T8020_TTBR_SHIFT 12

/* T8110 registers */

#define DART_T8110_PARAMS3 0x08
#define DART_T8110_PARAMS3_PA_WIDTH GENMASK(29, 24)
#define DART_T8110_PARAMS3_VA_WIDTH GENMASK(21, 16)
#define DART_T8110_PARAMS3_VER_MAJ GENMASK(15, 8)
#define DART_T8110_PARAMS3_VER_MIN GENMASK(7, 0)

#define DART_T8110_PARAMS4 0x0c
#define DART_T8110_PARAMS4_NUM_CLIENTS GENMASK(24, 16)
#define DART_T8110_PARAMS4_NUM_SIDS GENMASK(8, 0)

#define DART_T8110_TLB_CMD              0x80
#define DART_T8110_TLB_CMD_BUSY         BIT(31)
#define DART_T8110_TLB_CMD_OP           GENMASK(10, 8)
#define DART_T8110_TLB_CMD_OP_FLUSH_ALL 0
#define DART_T8110_TLB_CMD_OP_FLUSH_SID 1
#define DART_T8110_TLB_CMD_OP_FLUSH_UNLOCK 4
#define DART_T8110_TLB_CMD_V2 BIT(15)
#define DART_T8110_TLB_CMD_RANGE BIT(14)
#define DART_T8110_TLB_CMD_STT BIT(13)
#define DART_T8110_TLB_START 0x98
#define DART_T8110_TLB_END 0xa0
#define DART_T8110_FLUSH_BUSY_TIMEOUT 2000
#define DART_T8110_TLB_CMD_STREAM       GENMASK(7, 0)

#define DART_T8110_ERROR 0x100
#define DART_T8110_ERROR_STREAM GENMASK(27, 20)
#define DART_T8110_ERROR_CODE GENMASK(14, 0)
#define DART_T8110_ERROR_FLAG BIT(31)

#define DART_T8110_ERROR_MASK 0x104

#define DART_T8110_ERROR_READ_FAULT BIT(5)
#define DART_T8110_ERROR_WRITE_FAULT BIT(4)
#define DART_T8110_ERROR_NO_PTE BIT(3)
#define DART_T8110_ERROR_NO_PMD BIT(2)
#define DART_T8110_ERROR_NO_PGD BIT(1)
#define DART_T8110_ERROR_NO_TTBR BIT(0)

#define DART_T8110_ERROR_ADDR_LO 0x170
#define DART_T8110_ERROR_ADDR_HI 0x174

#define DART_T8110_ERROR_STREAMS 0x1c0

#define DART_T8110_EXCEPTION(i) (0x4000 + 4 * (i))
#define DART_T8110_ERR_SID(sid) (0x8000 + 0x40 * (sid))
#define DART_T8110_ERR_SID_STATUS(sid) (DART_T8110_ERR_SID(sid) + 0x00)
#define DART_T8110_ERR_SID_STATUS_WRITE BIT(8)
#define DART_T8110_EXCEPTION_CODE GENMASK(3, 0)
#define DART_T8110_EXCEPTION_LEVEL GENMASK(13, 12)

/* DART v2.3 adds a block-level exception summary. */
#define DART_T8110_BLOCK_ERROR 0x704
#define DART_T8110_BLOCK_ERROR_MASK 0x777

#define DART_T8110_EXCEPTION_INFO(sid) (0x8000 + 0x40 * (sid))
#define DART_T8110_EXCEPTION_ADDR_LO(sid) (DART_T8110_ERR_SID(sid) + 0x8)
#define DART_T8110_EXCEPTION_ADDR_HI(sid) (DART_T8110_ERR_SID(sid) + 0xc)

#define DART_T8110_PROTECT 0x200
#define DART_T8110_UNPROTECT 0x204
#define DART_T8110_PROTECT_LOCK 0x208
#define DART_T8110_PROTECT_TTBR_TCR BIT(0)
#define DART_T8110_DIAG_LOCK 0x210
#define DART_T8110_DIAG_LOCK_ON_ERR BIT(1)

/* T8152 Gen3: low nibble selects the TLBI route for this DART instance */
#define DART_GEN3_ROUTE 0x4604
/* AppleGen3DART _DART_ERR_SID_AXI_ID/_AXI_USER: 2 and 8 words per SID record */
#define DART_GEN3_ERR_SID_AXI_ID(sid) (DART_T8110_ERR_SID(sid) + 0x18)
#define DART_GEN3_ERR_SID_AXI_USER(sid) (DART_T8110_ERR_SID(sid) + 0x20)
#define DART_T8110_ENABLE_STREAMS  0xc00
#define DART_T8110_DISABLE_STREAMS 0xc20

#define DART_T8110_TCR                  0x1000
#define DART_T8110_TCR_REMAP            GENMASK(11, 8)
#define DART_T8110_TCR_REMAP_ENABLE     BIT(7)
#define DART_T8110_TCR_FOUR_LEVEL       BIT(3)
#define DART_T8110_TCR_BYPASS_DAPF      BIT(2)
#define DART_T8110_TCR_BYPASS_DART      BIT(1)
#define DART_T8110_TCR_TRANSLATE_ENABLE BIT(0)

#define DART_T8110_TTBR       0x1400
#define DART_T8110_TTBR_VALID BIT(0)
#define DART_T8110_TTBR_ADDR_FIELD_SHIFT 2
#define DART_T8110_TTBR_SHIFT 14

#define DART_TCR(dart, sid) ((dart)->hw->tcr + ((sid) << 2))

#define DART_TTBR(dart, sid, idx) ((dart)->hw->ttbr + \
				   (((dart)->hw->ttbr_count * (sid)) << 2) + \
				   ((idx) << 2))

struct apple_dart_stream_map;

enum dart_type {
	DART_T8020,
	DART_T6000,
	DART_T8110,
};

struct apple_dart;

struct apple_dart_hw {
	enum dart_type type;
	irqreturn_t (*irq_handler)(int irq, void *dev);
	int (*invalidate_tlb)(struct apple_dart_stream_map *stream_map);
	void (*enable_remap)(struct apple_dart *dart, int sid, int remap_to);
	bool gen3;

	u32 oas;
	enum io_pgtable_fmt fmt;

	int max_sid_count;

	u32 lock;
	u32 lock_bit;

	u32 error;

	u32 enable_streams;

	u32 tcr;
	u32 tcr_enabled;
	u32 tcr_disabled;
	u32 tcr_bypass;
	u32 tcr_4level;

	u32 ttbr;
	u32 ttbr_valid;
	u32 ttbr_addr_field_shift;
	u32 ttbr_shift;
	int ttbr_count;
};

/*
 * Snapshot of a locked root's firmware-owned slots, taken in process context
 * when the root is mapped: the root entry and a mapping of its leaf table.
 */
struct apple_dart_fw_root {
	u64 *entry;
	u64 **leaf;
	/* Linux root entry already verified against each firmware slot. */
	u64 *accepted;
};

/*
 * Private structure associated with each DART device.
 *
 * @dev: device struct
 * @hw: SoC-specific hardware data
 * @regs: mapped MMIO region
 * @irq: interrupt number, can be shared with other DARTs
 * @clks: clocks associated with this DART
 * @num_clks: number of @clks
 * @lock: lock for hardware operations involving this dart
 * @pgsize: pagesize supported by this DART
 * @supports_bypass: indicates if this DART supports bypass mode
 * @locked: indicates if this DART is locked
 * @sid2group: maps stream ids to iommu_groups
 * @iommu: iommu core device
 */
struct apple_dart {
	struct device *dev;
	const struct apple_dart_hw *hw;

	void __iomem *regs;

	int irq;
	struct clk_bulk_data *clks;
	int num_clks;

	spinlock_t lock;

	u32 ias;
	u32 oas;
	u32 pgsize;
	u32 num_streams;
	u32 supports_bypass : 1;
	u32 four_level : 1;
	u32 locked : 1;
	u32 force_available : 1;
	u32 tunneled : 1;
	u32 per_stream : 1;
	u32 power_retained : 1;
	/* Locked roots may hold firmware slots that mirror Linux mappings. */
	u32 fw_mirror : 1;
	u32 fw_handoff : 1;
	u32 tunnel_state_saved : 1;
	u32 tunnel_state_restored : 1;
	bool commands_gated;
	/* Resume restored the translations, but the last steps of its reset failed. */
	bool reset_incomplete;
	struct apple_tunable *tunables;
	/*
	 * Gen3 static topology: no DART MMIO at probe. Topology comes from fixed
	 * values; tunables, parameter read, lock check, reset and the fault IRQ
	 * wait for the first runtime resume (first consumer).
	 */
	u32 static_topo : 1;
	u32 hw_ready : 1;
	u32 irq_requested : 1;
	/* Serialises the first-use init: resume, attach and the fault IRQ can all reach it. */
	spinlock_t late_lock;
	/* Gen3 (T8152): shared TLBI doorbell and per-instance route register */
	void __iomem *tlbi;
	u16 version;

	DECLARE_BITMAP(remapped, DART_MAX_STREAMS);
	u8 remap_target[DART_MAX_STREAMS];

	dma_addr_t dma_min;
	dma_addr_t dma_max;
	dma_addr_t dma_offset;

	struct iommu_group *sid2group[DART_MAX_STREAMS];
	struct iommu_device iommu;

	u32 save_tcr[DART_MAX_STREAMS];
	u32 save_ttbr[DART_MAX_STREAMS][DART_MAX_TTBR];

	u64 *locked_ttbr[DART_MAX_STREAMS][DART_MAX_TTBR];
	/* Values published by this driver into initially vacant root slots. */
	u64 *locked_owned[DART_MAX_STREAMS][DART_MAX_TTBR];
	/* Leaf tables behind the root slots firmware held when we attached. */
	struct apple_dart_fw_root *locked_fw[DART_MAX_STREAMS][DART_MAX_TTBR];
	/*
	 * First DVA of the window of the domain using each locked stream.  The
	 * window can be the client's own (apple,dma-range on the client node)
	 * rather than the DART's.
	 */
	u64 locked_window[DART_MAX_STREAMS];
};

/*
 * PCIe-C is a cable-powered aperture and can report a failed transaction as
 * an asynchronous external abort unless each access is completed before the
 * next instruction retires.  The DART reset sequence is only stable with
 * Device-nGnRnE attributes and a dsb sy + isb after every access.
 * ioremap_np() supplies the memory type, but writel() only supplies the
 * normal I/O ordering barrier on arm64, so add the barriers here for the
 * tunneled DART and leave all always-on DARTs on the normal fast path.
 */
static inline void apple_dart_writel(struct apple_dart *dart, u32 value,
				    u32 offset)
{
#if IS_ENABLED(CONFIG_APPLE_DART_KUNIT_TEST)
	KUNIT_STATIC_STUB_REDIRECT(apple_dart_writel, dart, value, offset);
#endif
	writel(value, dart->regs + offset);
	if (unlikely(dart->tunneled)) {
		mb();
		isb();
	}
}

static inline u32 apple_dart_readl(struct apple_dart *dart, u32 offset)
{
	u32 value = readl(dart->regs + offset);

	if (unlikely(dart->tunneled)) {
		mb();
		isb();
	}

	return value;
}

static void apple_dart_apply_tunables(struct apple_dart *dart)
{
	size_t i;

	for (i = 0; i < dart->tunables->sz; ++i) {
		u32 old_value, value;

		old_value = apple_dart_readl(dart,
					      dart->tunables->values[i].offset);
		value = old_value & ~dart->tunables->values[i].mask;
		value |= dart->tunables->values[i].value;
		if (value != old_value)
			apple_dart_writel(dart, value,
					   dart->tunables->values[i].offset);
	}
}

/*
 * Convenience struct to identify streams.
 *
 * The normal variant is used inside apple_dart_master_cfg which isn't written
 * to concurrently.
 * The atomic variant is used inside apple_dart_domain where we have to guard
 * against races from potential parallel calls to attach/detach_device.
 * Note that even inside the atomic variant the apple_dart pointer is not
 * protected: This pointer is initialized once under the domain init mutex
 * and never changed again afterwards. Devices with different dart pointers
 * cannot be attached to the same domain.
 *
 * @dart dart pointer
 * @sid stream id bitmap
 */
struct apple_dart_stream_map {
	struct apple_dart *dart;
	DECLARE_BITMAP(sidmap, DART_MAX_STREAMS);
};
struct apple_dart_atomic_stream_map {
	struct apple_dart *dart;
	atomic_long_t sidmap[BITS_TO_LONGS(DART_MAX_STREAMS)];
};

/*
 * This structure is attached to each iommu domain handled by a DART.
 *
 * @pgtbl_ops: pagetable ops allocated by io-pgtable
 * @finalized: true if the domain has been completely initialized
 * @init_lock: protects domain initialization
 * @stream_maps: streams attached to this domain (valid for DMA/UNMANAGED only)
 * @fw_sidmap: streams whose locked firmware roots the domain imported at
 *             finalization, per entry of @stream_maps; unlike @stream_maps
 *             they are known before any device is attached
 * @domain: core iommu domain pointer
 */
struct apple_dart_domain {
	struct io_pgtable_ops *pgtbl_ops;

	bool finalized;
	bool has_dma_range;
	dma_addr_t dma_min;
	dma_addr_t dma_max;
	u64 mask;
	dma_addr_t dma_offset;
	struct mutex init_lock;
	struct apple_dart_atomic_stream_map stream_maps[MAX_DARTS_PER_DEVICE];
	DECLARE_BITMAP(fw_sidmap[MAX_DARTS_PER_DEVICE], DART_MAX_STREAMS);

	struct iommu_domain domain;
};

/*
 * This structure is attached to devices with dev_iommu_priv_set() on of_xlate
 * and contains a list of streams bound to this device.
 * So far the worst case seen is a single device with two streams
 * from different darts, such that this simple static array is enough.
 *
 * @streams: streams for this device
 */
struct apple_dart_master_cfg {
	/* Intersection of DART capabilitles */
	u32 supports_bypass : 1;
	u32 locked : 1;

	/*
	 * DMA aperture start and end to be overridden by "iommus"' phandle
	 * args. By default determined by DART's ias but may be outside of it.
	 */
	dma_addr_t dma_min;
	dma_addr_t dma_max;

	struct apple_dart_stream_map stream_maps[MAX_DARTS_PER_DEVICE];
};

/*
 * Helper macro to iterate over apple_dart_master_cfg.stream_maps and
 * apple_dart_domain.stream_maps
 *
 * @i int used as loop variable
 * @base pointer to base struct (apple_dart_master_cfg or apple_dart_domain)
 * @stream pointer to the apple_dart_streams struct for each loop iteration
 */
#define for_each_stream_map(i, base, stream_map)                               \
	for (i = 0, stream_map = &(base)->stream_maps[0];                      \
	     i < MAX_DARTS_PER_DEVICE && stream_map->dart;                     \
	     stream_map = &(base)->stream_maps[++i])

static struct platform_driver apple_dart_driver;
static int apple_dart_cfg_ensure_ready(struct apple_dart_master_cfg *cfg);
static int apple_dart_gen3_ensure_ready(struct apple_dart *dart);
static const struct iommu_ops apple_dart_iommu_ops;

static struct apple_dart_domain *to_dart_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct apple_dart_domain, domain);
}

/*
 * Asahi's stream remapping programs only the first stream of a map and
 * remaps the others onto it. A PCIe-C tunnel DART keeps programming and
 * invalidating every stream on its own, with the ordered accesses of
 * apple_dart_writel(). So do the T8140 and T8142 DARTs (per_stream): their
 * locked streams keep firmware-owned roots and filtering state of their own,
 * and their static remaps come from apple,remap-streams.
 */
static bool apple_dart_remaps_streams(struct apple_dart *dart)
{
	return dart->hw->enable_remap && !dart->tunneled && !dart->per_stream;
}

static void
apple_dart_hw_enable_translation(struct apple_dart_stream_map *stream_map, int levels)
{
	struct apple_dart *dart = stream_map->dart;
	u32 tcr = dart->hw->tcr_enabled;
	int sid, remap_to;

	if (levels == 4)
		tcr |= dart->hw->tcr_4level;

	WARN_ON(levels != 3 && levels != 4);
	WARN_ON(levels == 4 && !dart->four_level);

	if (!apple_dart_remaps_streams(dart)) {
		for_each_set_bit(sid, stream_map->sidmap, dart->num_streams)
			apple_dart_writel(dart, tcr, DART_TCR(dart, sid));
		return;
	}

	/* Search for first enabled sid and enable translation. */
	sid = find_first_bit(stream_map->sidmap, dart->num_streams);
	if (sid >= dart->num_streams)
		return;
	writel(tcr, dart->regs + DART_TCR(dart, sid));

	/* Enable remapping for the remaining used sids. */
	remap_to = sid++;
	for_each_set_bit_from(sid, stream_map->sidmap, dart->num_streams)
		dart->hw->enable_remap(dart, sid, remap_to);
}

static void apple_dart_hw_disable_dma(struct apple_dart_stream_map *stream_map)
{
	struct apple_dart *dart = stream_map->dart;
	int sid;

	/* This will also disable remap enabled in apple_dart_hw_enable_translation(). */
	for_each_set_bit(sid, stream_map->sidmap, dart->num_streams)
		apple_dart_writel(dart, dart->hw->tcr_disabled,
				   DART_TCR(dart, sid));
}

static void
apple_dart_hw_enable_bypass(struct apple_dart_stream_map *stream_map)
{
	struct apple_dart *dart = stream_map->dart;
	int sid;

	WARN_ON(!stream_map->dart->supports_bypass);
	for_each_set_bit(sid, stream_map->sidmap, dart->num_streams)
		apple_dart_writel(dart, dart->hw->tcr_bypass,
				   DART_TCR(dart, sid));
}

static void apple_dart_hw_set_ttbr(struct apple_dart_stream_map *stream_map,
				   u8 idx, phys_addr_t paddr)
{
	struct apple_dart *dart = stream_map->dart;
	int sid;

	WARN_ON(paddr & ((1 << dart->hw->ttbr_shift) - 1));
	if (!apple_dart_remaps_streams(dart)) {
		for_each_set_bit(sid, stream_map->sidmap, dart->num_streams)
			apple_dart_writel(dart,
					   dart->hw->ttbr_valid |
					   (paddr >> dart->hw->ttbr_shift) <<
					   dart->hw->ttbr_addr_field_shift,
					   DART_TTBR(dart, sid, idx));
		return;
	}

	/* Search for the first used stream and set its TTBR. All other sids
	 * of this DART are handled via the remap feature and must not set a
	 * valid TTBR.
	 */
	sid = find_first_bit(stream_map->sidmap, dart->num_streams);
	if (sid >= dart->num_streams)
		return;

	writel(dart->hw->ttbr_valid |
	       (paddr >> dart->hw->ttbr_shift) << dart->hw->ttbr_addr_field_shift,
	       dart->regs + DART_TTBR(dart, sid, idx));
}

static void apple_dart_hw_clear_ttbr(struct apple_dart_stream_map *stream_map,
				     u8 idx)
{
	struct apple_dart *dart = stream_map->dart;
	int sid;

	for_each_set_bit(sid, stream_map->sidmap, dart->num_streams)
		apple_dart_writel(dart, 0, DART_TTBR(dart, sid, idx));
}

static void
apple_dart_hw_clear_all_ttbrs(struct apple_dart_stream_map *stream_map)
{
	int i;

	for (i = 0; i < stream_map->dart->hw->ttbr_count; ++i)
		apple_dart_hw_clear_ttbr(stream_map, i);
}

/* APPLE_DART2 page-table entry layout, as in io-pgtable-dart. */
#define APPLE_DART_PTE_VALID BIT_ULL(0)

static phys_addr_t apple_dart_pte_to_paddr(const struct apple_dart *dart,
					   u64 pte)
{
	return (pte & GENMASK_ULL(37, 10)) << 4;
}

/* Firmware table pages must remain outside Linux's page allocator. */
static bool apple_dart_fw_handoff_enabled(const struct apple_dart *dart)
{
	struct device_node *np = dart->dev->of_node;
	u32 handoff;

	return dart->locked && of_device_is_compatible(np, "apple,t8140-dart") &&
		of_property_count_u32_elems(np, "apple,firmware-root-handoff") == 1 &&
		!of_property_read_u32(np, "apple,firmware-root-handoff", &handoff) &&
		handoff == 1;
}

static bool apple_dart_fw_table_ram(const struct apple_dart *dart,
				   phys_addr_t phys)
{
	unsigned long first, last, pfn;
	phys_addr_t end;

	if (dart->pgsize != SZ_16K || !IS_ALIGNED(phys, dart->pgsize) ||
	    check_add_overflow(phys, (phys_addr_t)dart->pgsize - 1, &end) ||
	    dart->oas >= 64 || end >= BIT_ULL(dart->oas))
		return false;
	first = PHYS_PFN(phys);
	last = PHYS_PFN(end);
	for (pfn = first; pfn <= last; pfn++)
		if (!pfn_valid(pfn) || !PageReserved(pfn_to_page(pfn)))
			return false;
	return true;
}

static bool apple_dart_fw_table_decode(const struct apple_dart *dart,
				      u64 pte, phys_addr_t *phys)
{
	if (!(pte & APPLE_DART_PTE_VALID) ||
	    (pte & ~(GENMASK_ULL(37, 10) | APPLE_DART_PTE_VALID)))
		return false;
	*phys = apple_dart_pte_to_paddr(dart, pte);
	return apple_dart_fw_table_ram(dart, *phys);
}

static bool apple_dart_fw_slot_owned(const struct apple_dart_fw_root *fw,
				     const u64 *live, size_t i)
{
	return fw && fw->leaf[i] && READ_ONCE(live[i]) == fw->entry[i];
}

static bool apple_dart_fw_leaf_decode(u64 pte, dma_addr_t iova,
				      phys_addr_t *phys, int *prot)
{
	if (!(pte & BIT_ULL(0)) || (pte & GENMASK_ULL(63, 52)) ||
	    (pte & GENMASK_ULL(51, 40)) != GENMASK_ULL(51, 40))
		return false;

	*phys = ((pte & GENMASK_ULL(37, 10)) << 4) | (iova & (SZ_16K - 1));
	*prot = (!(pte & BIT_ULL(3)) ? IOMMU_READ : 0) |
		(!(pte & BIT_ULL(2)) ? IOMMU_WRITE : 0) |
		(!(pte & BIT_ULL(1)) ? IOMMU_CACHE : 0);
	return true;
}

static bool apple_dart_fw_mapping_matches(phys_addr_t phys, int prot,
					  phys_addr_t expected, int required)
{
	const int mask = IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE;

	return phys == expected && (prot & mask) == (required & mask);
}

/*
 * A root slot firmware already uses can still back Linux mappings when the
 * firmware's leaf table translates every page Linux mapped there to the same
 * physical address. That is the case for the boot loader's handoff, which
 * re-exports live firmware translations as reserved IOVA regions; the
 * hardware keeps walking the firmware table and Linux's copy stays private.
 */
static bool apple_dart_fw_slot_mirrors(const struct apple_dart *dart,
				       const struct apple_dart_fw_root *fw,
				       u64 next, size_t i, size_t entries)
{
	const u64 *ours_leaf, *fw_leaf;
	size_t j;

	ours_leaf = phys_to_virt(apple_dart_pte_to_paddr(dart, next));
	fw_leaf = fw->leaf[i];
	for (j = 0; j < entries; j++) {
		u64 ours = READ_ONCE(ours_leaf[j]);
		u64 theirs = READ_ONCE(fw_leaf[j]);

		phys_addr_t ours_phys, fw_phys;
		int ours_prot, fw_prot;

		if (!(ours & APPLE_DART_PTE_VALID))
			continue;

		if (!apple_dart_fw_leaf_decode(ours, 0, &ours_phys, &ours_prot) ||
		    !apple_dart_fw_leaf_decode(theirs, 0, &fw_phys, &fw_prot) ||
		    !apple_dart_fw_mapping_matches(fw_phys, fw_prot,
					   ours_phys, ours_prot))
			return false;
	}
	return true;
}

/*
 * A locked root belongs to firmware. Publish only into vacant slots and
 * remember exactly what we installed, so teardown cannot erase firmware
 * mappings added after Linux attached. A slot firmware already holds is
 * never written; its descendants must still mirror Linux's mappings, and
 * apple_dart_check_fw_map() rejects new mappings anywhere in that slot.
 */
static int apple_dart_publish_root(const struct apple_dart *dart,
				   u64 *live, u64 *owned, const u64 *ours,
				   struct apple_dart_fw_root *fw,
				   size_t entries)
{
	size_t i;

	for (i = 0; i < entries; i++) {
		u64 next = READ_ONCE(ours[i]);
		u64 previous = READ_ONCE(owned[i]);

		if (!next && !previous)
			continue;
		if (READ_ONCE(live[i]) == previous)
			continue;
		if (previous || !apple_dart_fw_slot_owned(fw, live, i))
			return -EBUSY;
		if ((dart->fw_handoff || next != fw->accepted[i]) &&
		    !apple_dart_fw_slot_mirrors(dart, fw, next, i, entries))
			return -EBUSY;
	}

	dma_wmb();
	for (i = 0; i < entries; i++) {
		u64 next = READ_ONCE(ours[i]);
		u64 previous = READ_ONCE(owned[i]);

		if (next == previous)
			continue;
		if (!previous && apple_dart_fw_slot_owned(fw, live, i)) {
			fw->accepted[i] = next;
			continue;
		}
		if (cmpxchg(&live[i], previous, next) != previous)
			return -EBUSY;
		WRITE_ONCE(owned[i], next);
	}
	return 0;
}

static void apple_dart_retire_root(u64 *live, u64 *owned, size_t entries)
{
	size_t i;

	for (i = 0; i < entries; i++) {
		u64 previous = READ_ONCE(owned[i]);

		if (!previous)
			continue;
		/* Preserve a slot if its owner changed after we published it. */
		cmpxchg(&live[i], previous, 0);
		WRITE_ONCE(owned[i], 0);
	}
	dma_wmb();
}

static void apple_dart_free_fw_root(struct apple_dart *dart,
				    struct apple_dart_fw_root *fw)
{
	size_t i;

	if (!fw)
		return;
	if (fw->leaf)
		for (i = 0; i < dart->pgsize / sizeof(u64); i++)
			if (fw->leaf[i])
				memunmap(fw->leaf[i]);
	kfree(fw->accepted);
	kfree(fw->leaf);
	kfree(fw->entry);
	kfree(fw);
}

/*
 * Record the leaf tables behind the slots firmware holds in a three-level
 * T8110-format locked root, on DARTs whose loader re-exports firmware
 * translations (fw_handoff). Other roots are not snapshotted; their firmware
 * slots keep rejecting Linux mappings as before.
 */
static struct apple_dart_fw_root *
apple_dart_snapshot_fw_root(struct apple_dart *dart, int sid, const u64 *live)
{
	size_t i, entries = dart->pgsize / sizeof(u64);
	struct apple_dart_fw_root *fw;
	struct apple_dart_fw_root *nomem = dart->fw_handoff ? ERR_PTR(-ENOMEM) : NULL;

	/* Only the T8110-style table layout this walker decodes. */
	if ((!dart->fw_mirror && !dart->fw_handoff) || dart->hw->fmt != APPLE_DART2)
		return NULL;
	if (dart->hw->tcr_4level &&
	    (apple_dart_readl(dart, DART_TCR(dart, sid)) & dart->hw->tcr_4level))
		return NULL;

	fw = kzalloc(sizeof(*fw), GFP_KERNEL);
	if (!fw)
		return nomem;
	fw->entry = kcalloc(entries, sizeof(*fw->entry), GFP_KERNEL);
	fw->leaf = kcalloc(entries, sizeof(*fw->leaf), GFP_KERNEL);
	fw->accepted = kcalloc(entries, sizeof(*fw->accepted), GFP_KERNEL);
	if (!fw->entry || !fw->leaf || !fw->accepted) {
		apple_dart_free_fw_root(dart, fw);
		return nomem;
	}

	for (i = 0; i < entries; i++) {
		u64 pte = READ_ONCE(live[i]);
		phys_addr_t phys;

		if (!(pte & APPLE_DART_PTE_VALID))
			continue;
		if (!dart->fw_handoff) {
			fw->leaf[i] = memremap(apple_dart_pte_to_paddr(dart, pte),
					       dart->pgsize, MEMREMAP_WB);
			if (!fw->leaf[i])
				continue;
			fw->entry[i] = pte;
			continue;
		}
		if (!apple_dart_fw_table_decode(dart, pte, &phys)) {
			apple_dart_free_fw_root(dart, fw);
			return ERR_PTR(-EINVAL);
		}
		fw->leaf[i] = memremap(phys, dart->pgsize, MEMREMAP_WB);
		if (!fw->leaf[i]) {
			apple_dart_free_fw_root(dart, fw);
			return ERR_PTR(-ENOMEM);
		}
		if (READ_ONCE(live[i]) != pte) {
			apple_dart_free_fw_root(dart, fw);
			return ERR_PTR(-EBUSY);
		}
		fw->entry[i] = pte;
	}
	return fw;
}

static int apple_dart_fw_invalidate(struct apple_dart_stream_map *stream_map,
				    u64 lo, u64 hi);

static int
apple_dart_hw_map_locked_ttbr(struct apple_dart_stream_map *stream_map, u8 idx)
{
	struct apple_dart *dart = stream_map->dart;
	int sid;

	for_each_set_bit(sid, stream_map->sidmap, dart->num_streams) {
		struct apple_dart_fw_root *fw;
		unsigned long flags;
		u32 ttbr;
		phys_addr_t phys;
		u64 *l1_tbl, *owned;

		if (dart->locked_ttbr[sid][idx])
			continue;
		/* The matched firmware handoff exports three-level tables only. */
		if (dart->fw_handoff && dart->hw->tcr_4level &&
		    (apple_dart_readl(dart, DART_TCR(dart, sid)) & dart->hw->tcr_4level))
			return -EINVAL;

		ttbr = apple_dart_readl(dart, DART_TTBR(dart, sid, idx));

		if (!(ttbr & dart->hw->ttbr_valid)) {
			dev_err(dart->dev, "Invalid ttbr[%u] for locked dart\n",
				idx);
			return -EIO;
		}

		ttbr &= ~dart->hw->ttbr_valid;

		if (dart->hw->ttbr_addr_field_shift)
			ttbr >>= dart->hw->ttbr_addr_field_shift;
		phys = ((phys_addr_t) ttbr) << dart->hw->ttbr_shift;

		if (dart->fw_handoff && !apple_dart_fw_table_ram(dart, phys))
			return -EINVAL;

		l1_tbl = devm_memremap(dart->dev, phys, dart->pgsize,
				       MEMREMAP_WB);
		if (IS_ERR(l1_tbl))
			return PTR_ERR(l1_tbl);

		owned = devm_kzalloc(dart->dev, dart->pgsize, GFP_KERNEL);
		if (!owned) {
			devm_memunmap(dart->dev, l1_tbl);
			return -ENOMEM;
		}
		fw = apple_dart_snapshot_fw_root(dart, sid, l1_tbl);
		if (IS_ERR(fw)) {
			devm_kfree(dart->dev, owned);
			devm_memunmap(dart->dev, l1_tbl);
			return PTR_ERR(fw);
		}
		spin_lock_irqsave(&dart->lock, flags);
		dart->locked_owned[sid][idx] = owned;
		dart->locked_ttbr[sid][idx] = l1_tbl;
		dart->locked_fw[sid][idx] = fw;
		spin_unlock_irqrestore(&dart->lock, flags);
	}

	return 0;
}

static int
apple_dart_hw_unmap_locked_ttbr(struct apple_dart_stream_map *stream_map, u8 idx)
{
	struct apple_dart *dart = stream_map->dart;
	int sid;

	for_each_set_bit(sid, stream_map->sidmap, dart->num_streams) {
		u64 *live = dart->locked_ttbr[sid][idx];
		u64 *owned = dart->locked_owned[sid][idx];
		struct apple_dart_fw_root *fw;
		unsigned long flags;

		if (!live)
			continue;
		apple_dart_retire_root(live, owned, dart->pgsize / sizeof(*live));
		if (dart->fw_mirror && dart->version >= 0x0202 && !idx) {
			struct apple_dart_stream_map stream = { .dart = dart };

			__set_bit(sid, stream.sidmap);
			(void)apple_dart_fw_invalidate(&stream, 0, U64_MAX);
		}
		spin_lock_irqsave(&dart->lock, flags);
		fw = dart->locked_fw[sid][idx];
		dart->locked_fw[sid][idx] = NULL;
		dart->locked_ttbr[sid][idx] = NULL;
		spin_unlock_irqrestore(&dart->lock, flags);
		apple_dart_free_fw_root(dart, fw);
		devm_memunmap(dart->dev, live);
		devm_kfree(dart->dev, owned);
		dart->locked_owned[sid][idx] = NULL;
	}
	return 0;
}

static int
apple_dart_hw_sync_locked(struct io_pgtable_cfg *cfg,
			 struct apple_dart_stream_map *stream_map,
			 bool defer_unknown)
{
	struct apple_dart *dart = stream_map->dart;
	unsigned long flags;
	int sid, idx, ret = 0;

	spin_lock_irqsave(&dart->lock, flags);
	for_each_set_bit(sid, stream_map->sidmap, dart->num_streams) {
		bool ready = true;

		if (defer_unknown) {
			for (idx = 0; idx < cfg->apple_dart_cfg.n_ttbrs; idx++) {
				if (!dart->locked_ttbr[sid][idx] ||
				    !dart->locked_owned[sid][idx]) {
					ready = false;
					break;
				}
			}
			if (!ready)
				continue;
		}

		for (idx = 0; idx < cfg->apple_dart_cfg.n_ttbrs; idx++) {
			u64 *live = dart->locked_ttbr[sid][idx];
			u64 *owned = dart->locked_owned[sid][idx];
			u64 *ours = cfg->apple_dart_cfg.ttbr[idx];

			/*
			 * Firmware mappings are created before attach maps the
			 * locked roots. Attach will publish these entries once
			 * both the live root and its ownership shadow exist.
			 */
			if (!live && !owned)
				continue;
			if (!live || !owned || !ours) {
				ret = -EIO;
				goto out;
			}
			/*
			 * Legacy display DARTs hand their locked root to Linux.
			 * The IOMMU core has rebuilt the firmware reservations
			 * before attachment, so replace the old root entries.
			 * T8110 v2.2+ instead retains firmware-owned entries.
			 */
			if (!dart->fw_mirror && !dart->fw_handoff &&
			    (dart->hw->type != DART_T8110 || dart->version < 0x0202)) {
				size_t entry;

				dma_wmb();
				for (entry = 0; entry < dart->pgsize / sizeof(*live); entry++) {
					u64 next = READ_ONCE(ours[entry]);

					WRITE_ONCE(live[entry], next);
					WRITE_ONCE(owned[entry], next);
				}
				dma_wmb();
				continue;
			}
			ret = apple_dart_publish_root(dart, live, owned, ours,
						      dart->locked_fw[sid][idx],
						      dart->pgsize / sizeof(*live));
			if (ret)
				goto out;
		}
	}
out:
	spin_unlock_irqrestore(&dart->lock, flags);
	return ret;
}

static int
apple_dart_t8020_hw_stream_command(struct apple_dart_stream_map *stream_map,
			     u32 command)
{
	struct apple_dart *dart = stream_map->dart;
	unsigned long flags;
	int ret, i;
	u32 sidmap[BITS_TO_U32(DART_MAX_STREAMS)];
	u32 command_reg;

	spin_lock_irqsave(&dart->lock, flags);
	if (dart->commands_gated) {
		spin_unlock_irqrestore(&dart->lock, flags);
		return -EHOSTDOWN;
	}

	bitmap_to_arr32(sidmap, stream_map->sidmap, dart->num_streams);
	for (i = 0; i < BITS_TO_U32(dart->num_streams); i++)
		apple_dart_writel(dart, sidmap[i],
				  DART_T8020_STREAM_SELECT + 4 * i);
	apple_dart_writel(dart, command, DART_T8020_STREAM_COMMAND);

	ret = read_poll_timeout_atomic(
		apple_dart_readl, command_reg,
		!(command_reg & DART_T8020_STREAM_COMMAND_BUSY), 1,
		DART_STREAM_COMMAND_BUSY_TIMEOUT, false, dart,
		DART_T8020_STREAM_COMMAND);

	spin_unlock_irqrestore(&dart->lock, flags);

	if (ret) {
		dev_err(stream_map->dart->dev,
			"busy bit did not clear after command %x for streams %lx\n",
			command, stream_map->sidmap[0]);
		return ret;
	}

	return 0;
}

static int
apple_dart_t8110_hw_tlb_command_range(struct apple_dart_stream_map *stream_map,
				   u32 command, bool range, u64 first, u64 last)
{
	struct apple_dart *dart = stream_map->dart;
	unsigned long flags;
	u32 status;
	int sid, ret = 0;

	spin_lock_irqsave(&dart->lock, flags);
	if (dart->commands_gated) {
		spin_unlock_irqrestore(&dart->lock, flags);
		return -EHOSTDOWN;
	}
	/* A domain bitmap can contain independent remap targets. */
	for_each_set_bit(sid, stream_map->sidmap, dart->num_streams) {
		u32 val = FIELD_PREP(DART_T8110_TLB_CMD_OP, command) |
			  FIELD_PREP(DART_T8110_TLB_CMD_STREAM, sid);

		ret = read_poll_timeout_atomic(apple_dart_readl, status,
			!(status & DART_T8110_TLB_CMD_BUSY), 1,
			DART_T8110_FLUSH_BUSY_TIMEOUT, false, dart,
			DART_T8110_TLB_CMD);
		if (ret)
			break;
		if (dart->version >= 0x0202 &&
		    command == DART_T8110_TLB_CMD_OP_FLUSH_SID)
			val |= DART_T8110_TLB_CMD_V2;
		if (range) {
			val |= DART_T8110_TLB_CMD_RANGE | DART_T8110_TLB_CMD_STT;
			apple_dart_writel(dart, (first >> ilog2(dart->pgsize)) << 2,
					  DART_T8110_TLB_START);
			apple_dart_writel(dart, (last >> ilog2(dart->pgsize)) << 2,
					  DART_T8110_TLB_END);
		}
		dma_wmb();
		apple_dart_writel(dart, val, DART_T8110_TLB_CMD);
		ret = read_poll_timeout_atomic(apple_dart_readl, status,
			!(status & DART_T8110_TLB_CMD_BUSY), 1,
			DART_T8110_FLUSH_BUSY_TIMEOUT, false, dart,
			DART_T8110_TLB_CMD);
		if (ret)
			break;
	}
	spin_unlock_irqrestore(&dart->lock, flags);
	if (ret)
		dev_err_ratelimited(dart->dev,
			"TLB command %#x for stream %d timed out (%#x)\n",
			command, sid, status);
	return ret;
}

/* A root table is at most 16 KiB of 64-bit entries. */
#define DART_MAX_ROOT_ENTRIES (SZ_16K / sizeof(u64))

/*
 * A slot of a four-level locked root that firmware holds: valid, and not an
 * entry Linux published. The loader's four-level roots are not snapshotted,
 * so this is the only firmware state Linux knows about them.
 */
static bool apple_dart_fw_slot_foreign(const u64 *live, const u64 *owned,
				       size_t i)
{
	u64 pte = READ_ONCE(live[i]);

	return (pte & APPLE_DART_PTE_VALID) && (!owned || pte != READ_ONCE(owned[i]));
}

/*
 * On T8140 DARTs that keep firmware roots, the hardware keeps
 * using the translations behind the firmware-owned root slots, e.g. the
 * display controller scanning out the boot framebuffer while Linux attaches,
 * or the AOP firmware's own DMA on the AOP DART streams that Linux shares.
 * Invalidating those translations, for the whole stream or by DVA range, can
 * break that DMA: the scanout fails a translation and the display
 * coprocessor panics on the failed read, and a whole-stream invalidate on a
 * busy AOP stream can stay busy for good. Linux never changes what those
 * slots resolve to, so only invalidate the parts of [lo, hi] reached through
 * other root slots, by DVA range, on three-level and four-level streams alike.
 */
static int apple_dart_fw_invalidate(struct apple_dart_stream_map *stream_map,
				    u64 lo, u64 hi)
{
	const u32 op = DART_T8110_TLB_CMD_OP_FLUSH_SID;
	struct apple_dart *dart = stream_map->dart;
	struct apple_dart_stream_map stream = { .dart = dart };
	size_t entries = dart->pgsize / sizeof(u64);
	u32 table_bits = ilog2(entries);
	DECLARE_BITMAP(fw_slots, DART_MAX_ROOT_ENTRIES);
	size_t first, last, i, end;
	unsigned long flags;
	int sid, ret = 0;

	if (entries > DART_MAX_ROOT_ENTRIES)
		return 0;

	for_each_set_bit(sid, stream_map->sidmap, dart->num_streams) {
		const struct apple_dart_fw_root *fw;
		const u64 *live, *owned;
		u64 base, span, start_dva, end_dva;
		bool four_level;
		u32 slot_shift;

		__set_bit(sid, stream.sidmap);
		four_level = apple_dart_readl(dart, DART_TCR(dart, sid)) &
			     dart->hw->tcr_4level;
		if (four_level) {
			/*
			 * A four-level stream walks the whole DVA: its root slots
			 * cover the input address space from zero.
			 */
			slot_shift = ilog2(dart->pgsize) + 2 * table_bits;
			span = BIT_ULL(min(slot_shift + table_bits, dart->ias));
			base = 0;
		} else {
			/*
			 * A three-level stream walks the low bits of the DVA; the
			 * TLB is tagged with the whole DVA, inside the window of
			 * the stream's domain.
			 */
			slot_shift = ilog2(dart->pgsize) + table_bits;
			span = BIT_ULL(slot_shift + table_bits);
			base = READ_ONCE(dart->locked_window[sid]) & ~(span - 1);
		}
		if (hi < base || lo > base + span - 1)
			goto next;
		start_dva = max(lo, base);
		end_dva = min(hi, base + span - 1);
		first = (start_dva - base) >> slot_shift;
		last = (end_dva - base) >> slot_shift;

		/* Nothing of Linux reaches the hardware without the root. */
		bitmap_zero(fw_slots, entries);
		spin_lock_irqsave(&dart->lock, flags);
		live = dart->locked_ttbr[sid][0];
		owned = dart->locked_owned[sid][0];
		fw = dart->locked_fw[sid][0];
		/*
		 * Four-level roots are not snapshotted, and with the reserved-table
		 * handoff firmware may replace its slots' descendants: there every
		 * valid slot Linux did not publish is firmware's.
		 */
		for (i = first; live && i <= last; i++)
			if (four_level || dart->fw_handoff ?
			    apple_dart_fw_slot_foreign(live, owned, i) :
			    apple_dart_fw_slot_owned(fw, live, i))
				__set_bit(i, fw_slots);
		spin_unlock_irqrestore(&dart->lock, flags);
		if (!live)
			goto next;

		for (i = find_next_zero_bit(fw_slots, last + 1, first); i <= last;
		     i = find_next_zero_bit(fw_slots, last + 1, end + 1)) {
			u64 start, stop;

			end = find_next_bit(fw_slots, last + 1, i) - 1;
			start = max(start_dva, base + ((u64)i << slot_shift));
			stop = min(end_dva, base + ((u64)(end + 1) << slot_shift) - 1);
			ret = apple_dart_t8110_hw_tlb_command_range(&stream, op, true,
								    start, stop);
			if (ret)
				break;
		}
next:
		__clear_bit(sid, stream.sidmap);
		if (ret)
			return ret;
	}
	return 0;
}

static int
apple_dart_t8020_hw_invalidate_tlb(struct apple_dart_stream_map *stream_map)
{
	return apple_dart_t8020_hw_stream_command(
		stream_map, DART_T8020_STREAM_COMMAND_INVALIDATE);
}

static int
apple_dart_t8110_hw_invalidate_tlb(struct apple_dart_stream_map *stream_map)
{
	if (stream_map->dart->fw_mirror && stream_map->dart->version >= 0x0202)
		return apple_dart_fw_invalidate(stream_map, 0, U64_MAX);
	return apple_dart_t8110_hw_tlb_command_range(
		stream_map, DART_T8110_TLB_CMD_OP_FLUSH_SID, false, 0, 0);
}

/*
 * Gen3 (T8152) DARTs do not take TLB invalidates through TLB_CMD. Apple's SPTM
 * (gen3dart_write_tlbi_cmd_payload) stores a 128-bit payload to a TLBI
 * aperture, then issues TLBI VALE1OS. Layout as used by m1n1 (accepted on
 * DCP SID 23 without an exception): lo = BIT(60) | route << 56 | sid << 48.
 * Writes one flush-SID doorbell per stream; no range invalidate.
 */
#ifdef CONFIG_ARM64
static int apple_dart_gen3_hw_invalidate_tlb(struct apple_dart_stream_map *stream_map)
{
	struct apple_dart *dart = stream_map->dart;
	u64 route = readl(dart->regs + DART_GEN3_ROUTE) & 0xf;
	unsigned long flags;
	int sid;

	spin_lock_irqsave(&dart->lock, flags);
	for_each_set_bit(sid, stream_map->sidmap, dart->num_streams) {
		u64 lo = BIT_ULL(60) | route << 56 | (u64)sid << 48;
		u64 hi = 0;

		dma_wmb();
		asm volatile("stp %0, %1, [%2]\n"
			     "dsb ish\n"
			     ".inst 0xd50881bf\n" /* tlbi vale1os, xzr */
			     "dsb osh\n"
			     :: "r"(lo), "r"(hi), "r"(dart->tlbi) : "memory");
	}
	spin_unlock_irqrestore(&dart->lock, flags);
	return 0;
}
#else
/* The doorbell store and TLBI are arm64 instructions; other architectures build only for COMPILE_TEST. */
static int apple_dart_gen3_hw_invalidate_tlb(struct apple_dart_stream_map *stream_map)
{
	return -EOPNOTSUPP;
}
#endif

static const char *apple_dart_t8110_v2_fault_name(u32 exception);

static void apple_dart_drain_stale_exceptions(struct apple_dart *dart)
{
	unsigned int i, drained = 0;

	if (dart->hw->type != DART_T8110 || dart->version < 0x0202)
		return;

	for (i = 0; i < DIV_ROUND_UP(dart->num_streams, 32) && i < 8; i++) {
		u32 pending = readl(dart->regs + DART_T8110_EXCEPTION(i));
		unsigned long bits = pending;
		unsigned int bit;

		if (!pending)
			continue;

		for_each_set_bit(bit, &bits, 32) {
			unsigned int sid = i * 32 + bit;
			u32 status;
			u64 addr;

			if (sid >= dart->num_streams)
				break;

			status = readl(dart->regs +
				       DART_T8110_ERR_SID_STATUS(sid));
			addr = readl(dart->regs +
				     DART_T8110_EXCEPTION_ADDR_LO(sid)) |
			       (u64)readl(dart->regs +
					  DART_T8110_EXCEPTION_ADDR_HI(sid)) << 32;
			dev_dbg(dart->dev,
				 "stale exception latched before probe: SID:%u %s %s at %#llx (status %#010x)\n",
				 sid,
				 status & DART_T8110_ERR_SID_STATUS_WRITE ?
					 "write" : "read",
				 apple_dart_t8110_v2_fault_name(status),
				 addr, status);
			drained++;
		}

		writel(pending, dart->regs + DART_T8110_EXCEPTION(i));
	}

	if (drained) {
		dsb(sy);
		readl(dart->regs + DART_T8110_EXCEPTION(0));
	}

	/* Gen3: 0x704 is not in AppleGen3DART's register capture list; do not touch it. */
	if (dart->version >= 0x0203 && !dart->hw->gen3)
		readl(dart->regs + DART_T8110_BLOCK_ERROR);

	if (drained)
		dev_dbg(dart->dev,
			 "%u stale exception(s) drained; these predate Linux\n",
			 drained);
}

static int apple_dart_t8110_v2_unlock(struct apple_dart *dart)
{
	struct apple_dart_stream_map stream_map = { .dart = dart };

	KUNIT_STATIC_STUB_REDIRECT(apple_dart_t8110_v2_unlock, dart);

	if (!(readl(dart->regs + DART_T8110_DIAG_LOCK) & DART_T8110_DIAG_LOCK_ON_ERR))
		return 0;

	/* Gen3 does not take commands through TLB_CMD, so the unlock command cannot be sent there. */
	if (dart->hw->gen3) {
		dev_warn_once(dart->dev,
			      "Gen3 DART: unlock after a lock-on-error exception is not supported\n");
		return 0;
	}

	/* Unlock after acknowledging a lock-on-error exception. */
	__set_bit(0, stream_map.sidmap);
	return apple_dart_t8110_hw_tlb_command_range(&stream_map,
					    DART_T8110_TLB_CMD_OP_FLUSH_UNLOCK,
					    false, 0, 0);
}

static int apple_dart_read_remaps(struct apple_dart *dart)
{
	struct device_node *np = dart->dev->of_node;
	u32 src, dst;
	int count, i, ret;

	if (!of_property_present(np, "apple,remap-streams"))
		return 0;
	count = of_property_count_u32_elems(np, "apple,remap-streams");
	if (count <= 0 || count > 2 * dart->num_streams || count % 2 ||
	    dart->hw->type != DART_T8110)
		return -EINVAL;
	for (i = 0; i < count; i += 2) {
		ret = of_property_read_u32_index(np, "apple,remap-streams", i, &src);
		if (!ret)
			ret = of_property_read_u32_index(np, "apple,remap-streams", i + 1, &dst);
		if (ret)
			return ret;
		if (src >= dart->num_streams || dst >= dart->num_streams ||
		    dst > FIELD_MAX(DART_T8110_TCR_REMAP) || src == dst)
			return -ERANGE;
		if (test_and_set_bit(src, dart->remapped))
			return -EINVAL;
		dart->remap_target[src] = dst;
	}
	/* Only a single remap hop is described by this binding. */
	for_each_set_bit(i, dart->remapped, dart->num_streams)
		if (test_bit(dart->remap_target[i], dart->remapped))
			return -EINVAL;
	return 0;
}

static void apple_dart_hw_apply_remaps(struct apple_dart *dart)
{
	int sid;

	for_each_set_bit(sid, dart->remapped, dart->num_streams)
		writel(DART_T8110_TCR_REMAP_ENABLE |
		       FIELD_PREP(DART_T8110_TCR_REMAP, dart->remap_target[sid]),
		       dart->regs + DART_TCR(dart, sid));
}

static void
apple_dart_t8020_hw_enable_remap(struct apple_dart *dart, int sid, int remap_to)
{
	u32 remap = readl(DART_REMAP(dart, sid));
	remap = (remap & ~DART_REMAP_MASK(sid)) |
		field_prep(DART_REMAP_MASK(sid), remap_to);
	writel(remap, DART_REMAP(dart, sid));

	u32 tcr = FIELD_PREP(DART_T8020_TCR_REMAP_ENABLE, 1) |
		  FIELD_PREP(DART_T8020_TCR_TRANSLATE_ENABLE, 1);
	writel(tcr, dart->regs + DART_TCR(dart, sid));
}

static void
apple_dart_t8110_hw_enable_remap(struct apple_dart *dart, int sid, int remap_to)
{
	u32 tcr = FIELD_PREP(DART_T8110_TCR_REMAP, remap_to) |
		  FIELD_PREP(DART_T8110_TCR_REMAP_ENABLE, 1);
	writel(tcr, dart->regs + DART_TCR(dart, sid));
}

/*
 * The last steps of a reset, which leave the translation registers alone:
 * acknowledge the errors latched so far, unlock a DART that locks on errors,
 * unmask the error interrupt and flush the TLBs.
 */
static int apple_dart_hw_finish_reset(struct apple_dart *dart)
{
	struct apple_dart_stream_map stream_map = { .dart = dart };
	int ret;

	/* Newer revisions expose per-stream exception records. */
	if (dart->hw->type == DART_T8110 && dart->version >= 0x0202) {
		apple_dart_drain_stale_exceptions(dart);
		ret = apple_dart_t8110_v2_unlock(dart);
		if (ret) {
			dev_err(dart->dev, "failed to unlock after reset: %d\n", ret);
			return ret;
		}
	} else {
		/* Clear pending errors before the interrupt is unmasked. */
		apple_dart_writel(dart, apple_dart_readl(dart, dart->hw->error),
				  dart->hw->error);
	}

	/* Gen3: 0x104 is not in AppleGen3DART's register capture list; leave it alone. */
	if (dart->hw->type == DART_T8110 && !dart->hw->gen3)
		apple_dart_writel(dart, 0, DART_T8110_ERROR_MASK);

	bitmap_set(stream_map.sidmap, 0, dart->num_streams);
	ret = dart->hw->invalidate_tlb(&stream_map);
	if (ret)
		dev_err(dart->dev, "failed to flush the TLBs after reset: %d\n", ret);

	return ret;
}

static int apple_dart_hw_reset(struct apple_dart *dart)
{
	struct apple_dart_stream_map stream_map;
	int i;

	stream_map.dart = dart;
	bitmap_zero(stream_map.sidmap, DART_MAX_STREAMS);
	bitmap_set(stream_map.sidmap, 0, dart->num_streams);

	apple_dart_hw_disable_dma(&stream_map);
	apple_dart_hw_clear_all_ttbrs(&stream_map);
	apple_dart_hw_apply_remaps(dart);

	/* enable all streams globally since TCR is used to control isolation */
	for (i = 0; i < BITS_TO_U32(dart->num_streams); i++)
		apple_dart_writel(dart, U32_MAX,
				   dart->hw->enable_streams + 4 * i);

	return apple_dart_hw_finish_reset(dart);
}

static int apple_dart_domain_flush_tlb_range(struct apple_dart_domain *domain,
					   bool range, u64 first, u64 last)
{
	struct apple_dart_atomic_stream_map *map;
	struct apple_dart_stream_map stream;
	struct io_pgtable_cfg *cfg = &io_pgtable_ops_to_pgtable(domain->pgtbl_ops)->cfg;
	const u32 op = DART_T8110_TLB_CMD_OP_FLUSH_SID;
	int i, j, ret, err = 0;
	u64 start, end;

	/*
	 * After an unmap, the caller frees the pages once this returns. Every
	 * DART that can be reached must therefore be invalidated, even if
	 * publishing the page table or powering another DART failed.
	 */
	for_each_stream_map(i, domain, map) {
		stream.dart = map->dart;
		for (j = 0; j < BITS_TO_LONGS(stream.dart->num_streams); j++)
			stream.sidmap[j] = atomic_long_read(&map->sidmap[j]);
		if (bitmap_empty(stream.sidmap, stream.dart->num_streams))
			continue;
		ret = pm_runtime_resume_and_get(stream.dart->dev);
		if (ret < 0) {
			dev_err_ratelimited(stream.dart->dev,
				"failed to power DART for TLB synchronization: %d\n", ret);
			err = err ?: ret;
			continue;
		}
		ret = stream.dart->locked ?
			apple_dart_hw_sync_locked(cfg, &stream, true) : 0;
		if (ret) {
			dev_err_ratelimited(stream.dart->dev,
				"failed to publish host page-table entries: %d\n", ret);
			err = err ?: ret;
		}
		/* The range registers take the DVA the device issues. */
		start = first + domain->dma_offset;
		end = last + domain->dma_offset;
		if (range && (stream.dart->fw_mirror || stream.dart->fw_handoff) &&
		    stream.dart->version >= 0x0202)
			ret = apple_dart_fw_invalidate(&stream, start, end);
		else if (range && stream.dart->locked && !stream.dart->hw->gen3 &&
			 stream.dart->version >= 0x0202)
			ret = apple_dart_t8110_hw_tlb_command_range(&stream, op, true,
								    start, end);
		else
			ret = stream.dart->hw->invalidate_tlb(&stream);
		pm_runtime_put(stream.dart->dev);
		err = err ?: ret;
	}
	return err;
}

static void apple_dart_domain_flush_tlb(struct apple_dart_domain *domain)
{
	(void)apple_dart_domain_flush_tlb_range(domain, false, 0, 0);
}

static void apple_dart_flush_iotlb_all(struct iommu_domain *domain)
{
	apple_dart_domain_flush_tlb(to_dart_domain(domain));
}

static void apple_dart_iotlb_sync(struct iommu_domain *domain,
				  struct iommu_iotlb_gather *gather)
{
	(void)apple_dart_domain_flush_tlb_range(to_dart_domain(domain),
					      gather->start <= gather->end,
					      gather->start, gather->end);
}

static int apple_dart_iotlb_sync_map(struct iommu_domain *domain,
				   unsigned long iova, size_t size)
{
	if (!size || iova + size - 1 < iova)
		return -EINVAL;
	return apple_dart_domain_flush_tlb_range(to_dart_domain(domain), true,
					       iova, iova + size - 1);
}

/*
 * The core installs reserved mappings before it attaches the first device,
 * while the attached-stream map is still empty. A locked stream whose
 * firmware root the domain imported keeps its inherited translations from
 * then on, so treat it as part of the domain for firmware lookups.
 */
static bool apple_dart_domain_uses_fw(struct apple_dart_domain *domain, int i,
				      struct apple_dart_atomic_stream_map *map,
				      int sid)
{
	return test_bit(sid, domain->fw_sidmap[i]) ||
	       (atomic_long_read(&map->sidmap[BIT_WORD(sid)]) & BIT_MASK(sid));
}

/*
 * Hardware continues to use every firmware-owned slot. Its entire IOVA
 * span is excluded from Linux allocation, including currently unmapped holes.
 */
static int apple_dart_check_fw_map(struct apple_dart_domain *domain, u64 dva,
				   phys_addr_t paddr, size_t size, int prot)
{
	struct apple_dart_atomic_stream_map *map;
	int i, sid, ret = 0;

	for_each_stream_map(i, domain, map) {
		struct apple_dart *dart = map->dart;
		u32 entries = dart->pgsize / sizeof(u64);
		u32 pgshift = ilog2(dart->pgsize), bits = ilog2(entries);
		unsigned long flags;
		size_t off;

		if (!dart->fw_handoff && !dart->fw_mirror)
			continue;
		spin_lock_irqsave(&dart->lock, flags);
		for (off = 0; off < size; off += dart->pgsize) {
			u64 addr = dva + off;

			for (sid = 0; sid < dart->num_streams; sid++) {
				bool four;
				u64 idx, slot;
				const u64 *live, *owned;

				if (!apple_dart_domain_uses_fw(domain, i, map, sid))
					continue;
				if (!dart->fw_handoff) {
					const struct apple_dart_fw_root *fw;
					phys_addr_t phys;
					u64 pte;
					int fw_prot;
					u64 leaf = (addr >> pgshift) & (entries - 1);

					slot = (addr >> (pgshift + bits)) & (entries - 1);
					idx = addr >> (pgshift + 2 * bits);
					if (idx >= DART_MAX_TTBR)
						continue;
					fw = dart->locked_fw[sid][idx];
					live = dart->locked_ttbr[sid][idx];
					if (!live || !apple_dart_fw_slot_owned(fw, live, slot))
						continue;
					pte = READ_ONCE(fw->leaf[slot][leaf]);
					if (!apple_dart_fw_leaf_decode(pte, addr, &phys, &fw_prot) ||
					    !apple_dart_fw_mapping_matches(phys, fw_prot,
									   paddr + off, prot)) {
						ret = -EBUSY;
						goto unlock;
					}
					continue;
				}
				four = apple_dart_readl(dart, DART_TCR(dart, sid)) &
					       dart->hw->tcr_4level;
				idx = four ? 0 : addr >> (pgshift + 2 * bits);
				slot = (addr >> (pgshift + (four ? 2 : 1) * bits)) &
					(entries - 1);
				if (idx >= DART_MAX_TTBR)
					continue;
				live = dart->locked_ttbr[sid][idx];
				owned = dart->locked_owned[sid][idx];
				/* All pages of an inherited slot remain firmware's. */
				if (live && apple_dart_fw_slot_foreign(live, owned, slot)) {
					ret = -EBUSY;
					goto unlock;
				}
			}
		}
 unlock:
		spin_unlock_irqrestore(&dart->lock, flags);
		if (ret)
			return ret;
	}
	return 0;
}

static phys_addr_t
apple_dart_inherited_phys(struct apple_dart_domain *domain, dma_addr_t address)
{
	struct apple_dart_atomic_stream_map *map;
	phys_addr_t result = 0;
	int i, sid;

	for_each_stream_map(i, domain, map) {
		struct apple_dart *dart = map->dart;
		unsigned long flags;
		size_t entries = dart->pgsize / sizeof(u64);
		size_t page = (address / dart->pgsize) % entries;
		size_t slot = address / dart->pgsize / entries;

		if (!dart->locked || (!dart->fw_handoff && !dart->fw_mirror) || slot >= entries)
			return 0;
		spin_lock_irqsave(&dart->lock, flags);
		for (sid = 0; sid < dart->num_streams; sid++) {
			struct apple_dart_fw_root *fw;
			phys_addr_t phys;
			int prot;

			if (!apple_dart_domain_uses_fw(domain, i, map, sid))
				continue;
			fw = dart->locked_fw[sid][0];
			if (!apple_dart_fw_slot_owned(fw, dart->locked_ttbr[sid][0], slot) ||
			    !apple_dart_fw_leaf_decode(READ_ONCE(fw->leaf[slot][page]),
						       address, &phys, &prot) ||
			    (result && result != phys)) {
				spin_unlock_irqrestore(&dart->lock, flags);
				return 0;
			}
			result = phys;
		}
		spin_unlock_irqrestore(&dart->lock, flags);
	}
	return result;
}

static phys_addr_t apple_dart_iova_to_phys(struct iommu_domain *domain,
					   dma_addr_t iova)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);
	struct io_pgtable_ops *ops = dart_domain->pgtbl_ops;
	struct io_pgtable_cfg *cfg;
	phys_addr_t phys;
	dma_addr_t address;

	if (!ops)
		return 0;

	address = (iova + dart_domain->dma_offset) & dart_domain->mask;
	phys = ops->iova_to_phys(ops, address);
	if (phys)
		return phys;
	cfg = &io_pgtable_ops_to_pgtable(ops)->cfg;
	if (cfg->apple_dart_cfg.n_levels != 3)
		return 0;
	/* Existing firmware translations need no replacement software mapping. */
	return apple_dart_inherited_phys(dart_domain, address);
}

static int apple_dart_map_pages(struct iommu_domain *domain, unsigned long iova,
				phys_addr_t paddr, size_t pgsize,
				size_t pgcount, int prot, gfp_t gfp,
				size_t *mapped)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);
	struct io_pgtable_ops *ops = dart_domain->pgtbl_ops;

	u64 dva;
	int ret;

	if (!ops)
		return -ENODEV;

	dva = (iova + dart_domain->dma_offset) & dart_domain->mask;
	ret = apple_dart_check_fw_map(dart_domain, dva, paddr, pgsize * pgcount,
				      prot);
	if (ret)
		return ret;

	return ops->map_pages(ops, dva, paddr, pgsize, pgcount, prot, gfp,
			      mapped);
}

static size_t apple_dart_unmap_pages(struct iommu_domain *domain,
				     unsigned long iova, size_t pgsize,
				     size_t pgcount,
				     struct iommu_iotlb_gather *gather)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);
	struct io_pgtable_ops *ops = dart_domain->pgtbl_ops;

	return ops->unmap_pages(ops,
				(iova + dart_domain->dma_offset) &
				dart_domain->mask, pgsize, pgcount,
				gather);
}

static void
apple_dart_setup_translation(struct apple_dart_domain *domain,
			     struct apple_dart_stream_map *stream_map)
{
	int i;
	struct apple_dart *dart = stream_map->dart;
	struct io_pgtable_cfg *pgtbl_cfg =
		&io_pgtable_ops_to_pgtable(domain->pgtbl_ops)->cfg;

	/* enable all streams globally since TCR is used to control isolation */
	for (i = 0; i < BITS_TO_U32(dart->num_streams); i++)
		apple_dart_writel(dart, U32_MAX,
				   dart->hw->enable_streams + 4 * i);

	for (i = 0; i < pgtbl_cfg->apple_dart_cfg.n_ttbrs; ++i) {
		u64 ttbr = virt_to_phys(pgtbl_cfg->apple_dart_cfg.ttbr[i]);
		apple_dart_hw_set_ttbr(stream_map, i, ttbr);
	}
	for (; i < stream_map->dart->hw->ttbr_count; ++i)
		apple_dart_hw_clear_ttbr(stream_map, i);

	apple_dart_hw_enable_translation(stream_map,
					 pgtbl_cfg->apple_dart_cfg.n_levels);
	stream_map->dart->hw->invalidate_tlb(stream_map);
}

/* Record the DVA window of @domain for invalidates of @stream_map's streams. */
static void apple_dart_set_locked_window(struct apple_dart_domain *domain,
					 struct apple_dart_stream_map *stream_map)
{
	struct apple_dart *dart = stream_map->dart;
	u64 start = domain->domain.geometry.aperture_start + domain->dma_offset;
	int sid;

	for_each_set_bit(sid, stream_map->sidmap, dart->num_streams)
		WRITE_ONCE(dart->locked_window[sid], start);
}

static int
apple_dart_setup_translation_locked(struct apple_dart_domain *domain,
				    struct apple_dart_stream_map *stream_map)
{
	struct io_pgtable_cfg *cfg = &io_pgtable_ops_to_pgtable(domain->pgtbl_ops)->cfg;
	int i, ret;

	apple_dart_set_locked_window(domain, stream_map);
	for (i = 0; i < cfg->apple_dart_cfg.n_ttbrs; i++) {
		ret = apple_dart_hw_map_locked_ttbr(stream_map, i);
		if (ret)
			goto unmap;
	}
	ret = apple_dart_hw_sync_locked(cfg, stream_map, false);
	if (!ret)
		ret = stream_map->dart->hw->invalidate_tlb(stream_map);
	if (!ret)
		return 0;
unmap:
	while (i--)
		apple_dart_hw_unmap_locked_ttbr(stream_map, i);
	return ret;
}

static int apple_dart_dma_window(u64 base, u64 size, u64 *last)
{
	if (!size || check_add_overflow(base, size - 1, last))
		return -EINVAL;
	return 0;
}

static bool apple_dart_dma_window_aligned(u64 base, u64 last, u32 pgsize)
{
	/* An inclusive end has all page-offset bits set, even at U64_MAX. */
	return IS_ALIGNED(base, pgsize) && IS_ALIGNED(~last, pgsize);
}

static int apple_dart_finalize_domain(struct apple_dart_domain *dart_domain,
				      struct apple_dart_master_cfg *cfg)
{
	struct apple_dart *dart = cfg->stream_maps[0].dart;
	struct io_pgtable_cfg pgtbl_cfg;
	dma_addr_t dma_min = dart_domain->has_dma_range ? dart_domain->dma_min : cfg->dma_min;
	dma_addr_t dma_max = dart_domain->has_dma_range ? dart_domain->dma_max : cfg->dma_max;
	u64 translated_max;
	u32 ias;
	int ret = 0;
	int i, j;

	if (dart->pgsize > PAGE_SIZE)
		return -EINVAL;
	if (check_add_overflow((u64)dma_max, (u64)dart->dma_offset,
			       &translated_max))
		return -ERANGE;
	ias = min_t(u32, dart->ias, fls64(translated_max));
	if (!apple_dart_dma_window_aligned(dma_min, dma_max, dart->pgsize))
		return -EINVAL;
	if ((dma_min ^ dma_max) & ~DMA_BIT_MASK(dart->ias))
		return -ERANGE;

	mutex_lock(&dart_domain->init_lock);

	if (dart_domain->finalized)
		goto done;

	/* Reserved mappings may be installed before any device is attached. */
	for (i = 0; i < MAX_DARTS_PER_DEVICE; ++i) {
		dart_domain->stream_maps[i].dart = cfg->stream_maps[i].dart;
		for (j = 0; j < BITS_TO_LONGS(DART_MAX_STREAMS); j++)
			atomic_long_set(&dart_domain->stream_maps[i].sidmap[j], 0);
	}

	pgtbl_cfg = (struct io_pgtable_cfg){
		.pgsize_bitmap = dart->pgsize,
		.ias = ias,
		.oas = dart->oas,
		.coherent_walk = 1,
		.iommu_dev = dart->dev,
	};

	if (dart->locked) {
		unsigned long *sidmap;
		int sid;
		u32 ttbr;

		/* Locked DARTs can only have a single stream bound */
		sidmap = cfg->stream_maps[0].sidmap;
		sid = find_first_bit(sidmap, dart->num_streams);

		if (sid >= dart->num_streams || bitmap_weight(sidmap, dart->num_streams) != 1) {
			ret = -EINVAL;
			goto done;
		}
		ttbr = apple_dart_readl(dart, DART_TTBR(dart, sid, 0));

		WARN_ON(!(ttbr & dart->hw->ttbr_valid));

		/* If the DART is locked, we need to keep the translation level count. */
		if (dart->hw->tcr_4level && dart->ias > 36) {
			if (apple_dart_readl(dart, DART_TCR(dart, sid)) &
			    dart->hw->tcr_4level) {
				if (ias < 37) {
					dev_dbg(dart->dev, "Expanded to ias=37 due to lock\n");
					pgtbl_cfg.ias = 37;
				}
			} else if (ias > 36) {
				dev_dbg(dart->dev, "Limited to ias=36 due to lock\n");
				pgtbl_cfg.ias = 36;
				if (dma_min == 0 && dma_max == DMA_BIT_MASK(dart->ias)) {
					dma_max = DMA_BIT_MASK(pgtbl_cfg.ias);
				} else if ((dma_min ^ dma_max) & ~DMA_BIT_MASK(36)) {
					dev_err(dart->dev,
						"Invalid DMA range for locked 3-level PT\n");
					ret = -ENOMEM;
					goto done;
				}
			}
		}
	}

	dart_domain->pgtbl_ops = alloc_io_pgtable_ops(dart->hw->fmt, &pgtbl_cfg,
						      &dart_domain->domain);
	if (!dart_domain->pgtbl_ops) {
		ret = -ENOMEM;
		goto done;
	}

	dart_domain->mask = DMA_BIT_MASK(pgtbl_cfg.ias);
	dart_domain->domain.pgsize_bitmap = pgtbl_cfg.pgsize_bitmap;
	dart_domain->domain.geometry.aperture_start = dma_min;
	dart_domain->domain.geometry.aperture_end = dma_max;
	dart_domain->domain.geometry.force_aperture = true;
	dart_domain->dma_offset = dart->dma_offset;

	if (pgtbl_cfg.apple_dart_cfg.n_levels == 3) {
		struct apple_dart_stream_map *stream;
		int index;

		/* Make inherited translations visible before core reservation setup. */
		for_each_stream_map(i, cfg, stream) {
			if (!stream->dart->locked ||
			    (!stream->dart->fw_handoff && !stream->dart->fw_mirror))
				continue;
			apple_dart_set_locked_window(dart_domain, stream);
			for (index = 0; index < pgtbl_cfg.apple_dart_cfg.n_ttbrs; index++) {
				ret = apple_dart_hw_map_locked_ttbr(stream, index);
				if (ret) {
					free_io_pgtable_ops(dart_domain->pgtbl_ops);
					dart_domain->pgtbl_ops = NULL;
					goto done;
				}
			}
			bitmap_copy(dart_domain->fw_sidmap[i], stream->sidmap,
				    stream->dart->num_streams);
		}
	}

	dart_domain->finalized = true;

done:
	mutex_unlock(&dart_domain->init_lock);
	return ret;
}

static int
apple_dart_mod_streams(struct apple_dart_atomic_stream_map *domain_maps,
		       struct apple_dart_stream_map *master_maps,
		       bool add_streams)
{
	int i, j;

	for (i = 0; i < MAX_DARTS_PER_DEVICE; ++i) {
		if (domain_maps[i].dart != master_maps[i].dart)
			return -EINVAL;
	}

	for (i = 0; i < MAX_DARTS_PER_DEVICE; ++i) {
		if (!domain_maps[i].dart)
			break;
		for (j = 0; j < BITS_TO_LONGS(domain_maps[i].dart->num_streams); j++) {
			if (add_streams)
				atomic_long_or(master_maps[i].sidmap[j],
					       &domain_maps[i].sidmap[j]);
			else
				atomic_long_and(~master_maps[i].sidmap[j],
						&domain_maps[i].sidmap[j]);
		}
	}

	return 0;
}

static int apple_dart_domain_add_streams(struct apple_dart_domain *domain,
					 struct apple_dart_master_cfg *cfg)
{
	return apple_dart_mod_streams(domain->stream_maps, cfg->stream_maps,
				      true);
}

static int apple_dart_attach_dev_paging(struct iommu_domain *domain,
					struct device *dev,
					struct iommu_domain *old)
{
	int ret, i;
	struct apple_dart_stream_map *stream_map;
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);

	for_each_stream_map(i, cfg, stream_map)
		WARN_ON(pm_runtime_get_sync(stream_map->dart->dev) < 0);

	ret = apple_dart_cfg_ensure_ready(cfg);
	if (ret)
		goto err;
	ret = apple_dart_finalize_domain(dart_domain, cfg);
	if (ret)
		goto err;

	ret = apple_dart_domain_add_streams(dart_domain, cfg);
	if (ret)
		goto err;

	for_each_stream_map(i, cfg, stream_map) {
		if (!stream_map->dart->locked)
			apple_dart_setup_translation(dart_domain, stream_map);
		else {
			ret = apple_dart_setup_translation_locked(dart_domain, stream_map);
			if (ret)
				goto err;
		}
	}

err:
	for_each_stream_map(i, cfg, stream_map)
		pm_runtime_put(stream_map->dart->dev);
	return ret;
}

static int apple_dart_attach_dev_identity(struct iommu_domain *domain,
					  struct device *dev,
					  struct iommu_domain *old)
{
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct apple_dart_stream_map *stream_map;
	int i;

	if (!cfg->supports_bypass)
		return -EINVAL;

	if (cfg->locked)
		return -EINVAL;

	for_each_stream_map(i, cfg, stream_map)
		WARN_ON(pm_runtime_get_sync(stream_map->dart->dev) < 0);

	if (apple_dart_cfg_ensure_ready(cfg)) {
		for_each_stream_map(i, cfg, stream_map)
			pm_runtime_put(stream_map->dart->dev);
		return -ENODEV;
	}

	for_each_stream_map(i, cfg, stream_map)
		apple_dart_hw_enable_bypass(stream_map);

	for_each_stream_map(i, cfg, stream_map)
		pm_runtime_put(stream_map->dart->dev);
	return 0;
}

static const struct iommu_domain_ops apple_dart_identity_ops = {
	.attach_dev = apple_dart_attach_dev_identity,
};

static struct iommu_domain apple_dart_identity_domain = {
	.type = IOMMU_DOMAIN_IDENTITY,
	.ops = &apple_dart_identity_ops,
};

static int apple_dart_attach_dev_blocked(struct iommu_domain *domain,
					 struct device *dev,
					 struct iommu_domain *old)
{
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct apple_dart_stream_map *stream_map;
	int i;

	if (cfg->locked)
		return -EINVAL;

	for_each_stream_map(i, cfg, stream_map)
		WARN_ON(pm_runtime_get_sync(stream_map->dart->dev) < 0);

	if (apple_dart_cfg_ensure_ready(cfg)) {
		for_each_stream_map(i, cfg, stream_map)
			pm_runtime_put(stream_map->dart->dev);
		return -ENODEV;
	}

	for_each_stream_map(i, cfg, stream_map)
		apple_dart_hw_disable_dma(stream_map);

	for_each_stream_map(i, cfg, stream_map)
		pm_runtime_put(stream_map->dart->dev);
	return 0;
}

static const struct iommu_domain_ops apple_dart_blocked_ops = {
	.attach_dev = apple_dart_attach_dev_blocked,
};

static struct iommu_domain apple_dart_blocked_domain = {
	.type = IOMMU_DOMAIN_BLOCKED,
	.ops = &apple_dart_blocked_ops,
};

static struct iommu_device *apple_dart_probe_device(struct device *dev)
{
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct apple_dart_stream_map *stream_map;
	int i;

	if (!dev_iommu_fwspec_get(dev) || !cfg)
		return ERR_PTR(-ENODEV);

	for_each_stream_map(i, cfg, stream_map) {
		device_link_add(dev, stream_map->dart->dev,
			DL_FLAG_PM_RUNTIME | DL_FLAG_AUTOREMOVE_SUPPLIER |
			DL_FLAG_RPM_ACTIVE);
		if (stream_map->dart->static_topo)
			dev_dbg(dev, "uses static-topology DART %s, sids %*pbl, hw_ready %d, rpm status %d\n",
				 dev_name(stream_map->dart->dev), stream_map->dart->num_streams,
				 stream_map->sidmap, stream_map->dart->hw_ready,
				 stream_map->dart->dev->power.runtime_status);
	}

	return &cfg->stream_maps[0].dart->iommu;
}

static void apple_dart_release_device(struct device *dev)
{
	int i, j, ret;
	struct apple_dart_stream_map *stream_map;
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);

	for_each_stream_map(j, cfg, stream_map) {
		if (!stream_map->dart->locked)
			continue;
		ret = pm_runtime_resume_and_get(stream_map->dart->dev);
		if (ret < 0) {
			dev_err_ratelimited(stream_map->dart->dev,
				"failed to power DART for consumer release: %d\n", ret);
			continue;
		}
		for (i = 0; i < stream_map->dart->hw->ttbr_count; ++i)
			apple_dart_hw_unmap_locked_ttbr(stream_map, i);
		(void)stream_map->dart->hw->invalidate_tlb(stream_map);
		pm_runtime_put(stream_map->dart->dev);
	}

	kfree(cfg);
}

static struct iommu_domain *apple_dart_domain_alloc_paging(struct device *dev)
{
	struct apple_dart_domain *dart_domain;

	dart_domain = kzalloc_obj(*dart_domain);
	if (!dart_domain)
		return NULL;

	mutex_init(&dart_domain->init_lock);

	if (dev) {
		struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
		u64 window[2];
		int ret;

		if (of_property_present(dev->of_node, "apple,dma-range")) {
			ret = of_property_read_u64_array(dev->of_node, "apple,dma-range",
						window, 2);
			if (!ret)
				ret = apple_dart_dma_window(window[0], window[1],
						    &dart_domain->dma_max);
			if (ret) {
				kfree(dart_domain);
				return ERR_PTR(ret);
			}
			dart_domain->dma_min = window[0];
			dart_domain->has_dma_range = true;
		}
		ret = apple_dart_finalize_domain(dart_domain, cfg);
		if (ret) {
			kfree(dart_domain);
			return ERR_PTR(ret);
		}
	}
	return &dart_domain->domain;
}

static void apple_dart_domain_free(struct iommu_domain *domain)
{
	struct apple_dart_domain *dart_domain = to_dart_domain(domain);
	struct apple_dart_atomic_stream_map *map;
	struct apple_dart_stream_map stream;
	int i, j, idx;

	for_each_stream_map(i, dart_domain, map) {
		if (!map->dart->locked)
			continue;
		stream.dart = map->dart;
		for (j = 0; j < BITS_TO_LONGS(stream.dart->num_streams); j++)
			stream.sidmap[j] = atomic_long_read(&map->sidmap[j]);
		if (bitmap_empty(stream.sidmap, stream.dart->num_streams))
			continue;
		j = pm_runtime_resume_and_get(stream.dart->dev);
		if (j < 0) {
			dev_err_ratelimited(stream.dart->dev,
				"retaining page tables after power failure: %d\n", j);
			return;
		}
		for (idx = 0; idx < stream.dart->hw->ttbr_count; idx++)
			apple_dart_hw_unmap_locked_ttbr(&stream, idx);
		j = stream.dart->hw->invalidate_tlb(&stream);
		pm_runtime_put(stream.dart->dev);
		/* Do not free page tables that hardware may still cache. */
		if (j) {
			dev_err_ratelimited(stream.dart->dev,
				"retaining page tables after TLB invalidation failure: %d\n", j);
			return;
		}
	}
	free_io_pgtable_ops(dart_domain->pgtbl_ops);
	kfree(dart_domain);
}

/*
 * Work out the DMA window of one "iommus" specifier. One cell is the stream
 * ID alone and keeps the window of the DART node. Five cells add the 64-bit
 * start and length of the window that the device uses.
 */
static int apple_dart_parse_dma_window(struct device *dev,
				       const struct apple_dart *dart,
				       const struct of_phandle_args *args,
				       dma_addr_t *dma_min,
				       dma_addr_t *dma_max)
{
	dma_addr_t length;

	*dma_min = dart->dma_min;
	*dma_max = dart->dma_max;

	if (args->args_count == 1)
		return 0;
	if (args->args_count != 5)
		return -EINVAL;

	length = ((dma_addr_t)args->args[3] << 32) | args->args[4];
	if (!length)
		return -EINVAL;

	if (*dma_min != 0 || *dma_max != DMA_BIT_MASK(dart->ias))
		dev_err(dev, "still using legacy \"apple,dma-range\" property\n");

	*dma_min = ((dma_addr_t)args->args[1] << 32) | args->args[2];

	if (!IS_ALIGNED(*dma_min, dart->pgsize) ||
	    !IS_ALIGNED(length, dart->pgsize)) {
		dev_err(dev, "Unaligned DMA window %pad, %pad (0x%x)\n",
			dma_min, &length, dart->pgsize);
		return -EINVAL;
	}
	if (check_add_overflow(*dma_min, length - 1, dma_max)) {
		dev_err(dev, "DMA window length (%pad) overflows range for start %pad\n",
			&length, dma_min);
		return -EINVAL;
	}

	/* Ensure that the DMA window does not exceed the DART's ias. */
	if ((*dma_min ^ *dma_max) & ~DMA_BIT_MASK(dart->ias)) {
		dev_err(dev, "Invalid DMA window for ias=%d\n", dart->ias);
		return -EINVAL;
	}

	return 0;
}

static int apple_dart_of_xlate(struct device *dev,
			       const struct of_phandle_args *args)
{
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct platform_device *iommu_pdev = of_find_device_by_node(args->np);
	struct apple_dart *dart = platform_get_drvdata(iommu_pdev);
	struct apple_dart *cfg_dart;
	dma_addr_t dma_max;
	dma_addr_t dma_min;
	int i, sid;

	put_device(&iommu_pdev->dev);

	i = apple_dart_parse_dma_window(dev, dart, args, &dma_min, &dma_max);
	if (i)
		return i;

	sid = args->args[0];

	if (!cfg) {
		cfg = kzalloc_obj(*cfg);
		if (!cfg)
			return -ENOMEM;
		/* Will be ANDed with DART capabilities */
		cfg->supports_bypass = true;
		/* Will be ORed with DART capabilities*/
		cfg->locked = false;
		/* Will be merged with other DARTs to the common range. */
		cfg->dma_min = dma_min;
		cfg->dma_max = dma_max;
	} else {
		if (dma_min >= cfg->dma_max || cfg->dma_min >= dma_max) {
			dev_err(dev, "non-overlapping DMA windows: %pad..%pad, %pad..%pad\n",
				&dma_min, &dma_max,
				&cfg->dma_min, &cfg->dma_max);
			return -EINVAL;
		}
		cfg->dma_min = max(dma_min, cfg->dma_min);
		cfg->dma_max = min(dma_max, cfg->dma_max);
	}
	dev_iommu_priv_set(dev, cfg);

	cfg_dart = cfg->stream_maps[0].dart;
	if (cfg_dart) {
		if (cfg_dart->pgsize != dart->pgsize)
			return -EINVAL;
		if (cfg_dart->ias != dart->ias)
			return -EINVAL;
	}

	cfg->supports_bypass &= dart->supports_bypass;
	cfg->locked |= dart->locked;

	for (i = 0; i < MAX_DARTS_PER_DEVICE; ++i) {
		if (cfg->stream_maps[i].dart == dart) {
			set_bit(sid, cfg->stream_maps[i].sidmap);
			return 0;
		}
	}
	for (i = 0; i < MAX_DARTS_PER_DEVICE; ++i) {
		if (!cfg->stream_maps[i].dart) {
			cfg->stream_maps[i].dart = dart;
			set_bit(sid, cfg->stream_maps[i].sidmap);
			return 0;
		}
	}

	return -EINVAL;
}

static DEFINE_MUTEX(apple_dart_groups_lock);

static void apple_dart_release_group(void *iommu_data)
{
	int i, sid;
	struct apple_dart_stream_map *stream_map;
	struct apple_dart_master_cfg *group_master_cfg = iommu_data;

	mutex_lock(&apple_dart_groups_lock);

	for_each_stream_map(i, group_master_cfg, stream_map)
		for_each_set_bit(sid, stream_map->sidmap, stream_map->dart->num_streams)
			stream_map->dart->sid2group[sid] = NULL;

	kfree(iommu_data);
	mutex_unlock(&apple_dart_groups_lock);
}

static int apple_dart_merge_master_cfg(struct apple_dart_master_cfg *dst,
				       struct apple_dart_master_cfg *src)
{
	/*
	 * We know that this function is only called for groups returned from
	 * pci_device_group and that all Apple Silicon platforms never spread
	 * PCIe devices from the same bus across multiple DARTs such that we can
	 * just assume that both src and dst only have the same single DART.
	 */
	if (src->stream_maps[1].dart)
		return -EINVAL;
	if (dst->stream_maps[1].dart)
		return -EINVAL;
	if (src->stream_maps[0].dart != dst->stream_maps[0].dart)
		return -EINVAL;

	bitmap_or(dst->stream_maps[0].sidmap,
		  dst->stream_maps[0].sidmap,
		  src->stream_maps[0].sidmap,
		  dst->stream_maps[0].dart->num_streams);
	return 0;
}

static struct iommu_group *apple_dart_device_group(struct device *dev)
{
	int i, sid;
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct apple_dart_stream_map *stream_map;
	struct apple_dart_master_cfg *group_master_cfg;
	struct iommu_group *group = NULL;
	struct iommu_group *res = ERR_PTR(-EINVAL);

	mutex_lock(&apple_dart_groups_lock);

	for_each_stream_map(i, cfg, stream_map) {
		for_each_set_bit(sid, stream_map->sidmap, stream_map->dart->num_streams) {
			struct iommu_group *stream_group =
				stream_map->dart->sid2group[sid];

			if (group && group != stream_group) {
				res = ERR_PTR(-EINVAL);
				goto out;
			}

			group = stream_group;
		}
	}

	if (group) {
		res = iommu_group_ref_get(group);
		goto out;
	}

#ifdef CONFIG_PCI
	if (dev_is_pci(dev))
		group = pci_device_group(dev);
	else
#endif
		group = generic_device_group(dev);

	res = ERR_PTR(-ENOMEM);
	if (!group)
		goto out;

	group_master_cfg = iommu_group_get_iommudata(group);
	if (group_master_cfg) {
		int ret;

		ret = apple_dart_merge_master_cfg(group_master_cfg, cfg);
		if (ret) {
			dev_err(dev, "Failed to merge DART IOMMU groups.\n");
			iommu_group_put(group);
			res = ERR_PTR(ret);
			goto out;
		}
	} else {
		group_master_cfg = kmemdup(cfg, sizeof(*group_master_cfg),
					   GFP_KERNEL);
		if (!group_master_cfg) {
			iommu_group_put(group);
			goto out;
		}

		iommu_group_set_iommudata(group, group_master_cfg,
			apple_dart_release_group);
	}

	for_each_stream_map(i, cfg, stream_map)
		for_each_set_bit(sid, stream_map->sidmap, stream_map->dart->num_streams)
			stream_map->dart->sid2group[sid] = group;

	res = group;

out:
	mutex_unlock(&apple_dart_groups_lock);
	return res;
}

static bool apple_dart_capable(struct device *dev, enum iommu_cap cap)
{
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct apple_dart_stream_map *stream_map;
	int i;

	switch (cap) {
	case IOMMU_CAP_DEFERRED_FLUSH:
		/*
		 * A deferred flush invalidates whole streams. On a T8140 or
		 * T8142 DART the firmware may still be translating through
		 * the same stream, and a whole-stream invalidate can fault
		 * its live accesses, so keep those DARTs on strict flushes.
		 */
		if (cfg)
			for_each_stream_map(i, cfg, stream_map)
				if (stream_map->dart->per_stream)
					return false;
		return true;
	default:
		return false;
	}
}

static int apple_dart_def_domain_type(struct device *dev)
{
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);

	if (cfg->stream_maps[0].dart->pgsize > PAGE_SIZE)
		return IOMMU_DOMAIN_IDENTITY;
	if (!cfg->supports_bypass)
		return IOMMU_DOMAIN_DMA;
	if (cfg->locked)
		return IOMMU_DOMAIN_DMA;

	return 0;
}

#ifndef CONFIG_PCIE_APPLE_MSI_DOORBELL_ADDR
/* Keep things compiling when CONFIG_PCIE_APPLE isn't selected */
#define CONFIG_PCIE_APPLE_MSI_DOORBELL_ADDR	0
#endif
#define DOORBELL_ADDR	(CONFIG_PCIE_APPLE_MSI_DOORBELL_ADDR & PAGE_MASK)

/* Convert an occupied root slot's device window back to allocator IOVA. */
static bool apple_dart_fw_slot_range(const struct apple_dart *dart, u64 window,
				    size_t slot, bool four, u64 *start, u64 *length)
{
	u32 bits = ilog2(dart->pgsize / sizeof(u64));
	u32 shift = ilog2(dart->pgsize) + (four ? 2 : 1) * bits;
	u32 width = four ? dart->ias : shift + bits;
	u64 span, base, offset, end;

	if (dart->ias >= 64 || shift >= dart->ias || width >= 64 ||
	    slot >= dart->pgsize / sizeof(u64))
		return false;
	span = BIT_ULL(shift);
	base = four ? 0 : window & ~(BIT_ULL(width) - 1);
	if (check_mul_overflow((u64)slot, span, &offset) ||
	    check_add_overflow(base, offset, start) ||
	    check_add_overflow(*start, span - 1, &end) ||
	    end >= BIT_ULL(dart->ias) || *start < dart->dma_offset)
		return false;
	*start -= dart->dma_offset;
	*length = span;
	return true;
}

static void apple_dart_get_fw_resv_regions(struct device *dev,
					 struct list_head *head)
{
	struct apple_dart_master_cfg *cfg = dev_iommu_priv_get(dev);
	struct apple_dart_stream_map *stream;
	int i, sid, idx;

	if (!cfg)
		return;
	for_each_stream_map(i, cfg, stream) {
		struct apple_dart *dart = stream->dart;
		size_t entries = dart->pgsize / sizeof(u64);

		if (!dart->fw_handoff || entries > DART_MAX_ROOT_ENTRIES)
			continue;
		for_each_set_bit(sid, stream->sidmap, dart->num_streams) {
			for (idx = 0; idx < dart->hw->ttbr_count; idx++) {
				DECLARE_BITMAP(slots, DART_MAX_ROOT_ENTRIES);
				const u64 *live, *owned;
				unsigned long flags;
				size_t slot;
				u64 window;
				bool four;

				bitmap_zero(slots, entries);
				/* Teardown may unmap both tables after dropping this lock. */
				spin_lock_irqsave(&dart->lock, flags);
				live = dart->locked_ttbr[sid][idx];
				owned = dart->locked_owned[sid][idx];
				window = dart->locked_window[sid];
				four = apple_dart_readl(dart, DART_TCR(dart, sid)) &
				       dart->hw->tcr_4level;
				for (slot = 0; live && slot < entries; slot++)
					if (apple_dart_fw_slot_foreign(live, owned, slot))
						__set_bit(slot, slots);
				spin_unlock_irqrestore(&dart->lock, flags);

				if (!four && check_add_overflow(window,
					((u64)idx << (ilog2(dart->pgsize) + 2 * ilog2(entries))),
					&window))
					continue;
				for_each_set_bit(slot, slots, entries) {
					struct iommu_resv_region *region;
					u64 start, length;

					if (!apple_dart_fw_slot_range(dart, window, slot,
						four, &start, &length))
						continue;
					/* RESERVED excludes holes without asking the core to map them. */
					region = iommu_alloc_resv_region(start, length, 0,
						IOMMU_RESV_RESERVED, GFP_KERNEL);
					if (!region)
						return;
					list_add_tail(&region->list, head);
				}
			}
		}
	}
}

static void apple_dart_get_resv_regions(struct device *dev,
					struct list_head *head)
{
	struct iommu_resv_region *region;

	if (IS_ENABLED(CONFIG_PCIE_APPLE) && dev_is_pci(dev)) {
		int prot = IOMMU_WRITE | IOMMU_NOEXEC | IOMMU_MMIO;

		region = iommu_alloc_resv_region(DOORBELL_ADDR,
						 PAGE_SIZE, prot,
						 IOMMU_RESV_MSI, GFP_KERNEL);
		if (!region)
			return;

		list_add_tail(&region->list, head);
	}

	/* Keep the translated-domain requirement and inherited translations. */
	iommu_dma_get_resv_regions(dev, head);
	apple_dart_get_fw_resv_regions(dev, head);
}

static const struct iommu_ops apple_dart_iommu_ops = {
	.capable = apple_dart_capable,
	.identity_domain = &apple_dart_identity_domain,
	.blocked_domain = &apple_dart_blocked_domain,
	.def_domain_type = apple_dart_def_domain_type,
	.domain_alloc_paging = apple_dart_domain_alloc_paging,
	.probe_device = apple_dart_probe_device,
	.release_device = apple_dart_release_device,
	.device_group = apple_dart_device_group,
	.of_xlate = apple_dart_of_xlate,
	.get_resv_regions = apple_dart_get_resv_regions,
	.owner = THIS_MODULE,
	.default_domain_ops = &(const struct iommu_domain_ops) {
		.attach_dev	= apple_dart_attach_dev_paging,
		.map_pages	= apple_dart_map_pages,
		.unmap_pages	= apple_dart_unmap_pages,
		.flush_iotlb_all = apple_dart_flush_iotlb_all,
		.iotlb_sync	= apple_dart_iotlb_sync,
		.iotlb_sync_map	= apple_dart_iotlb_sync_map,
		.iova_to_phys	= apple_dart_iova_to_phys,
		.free		= apple_dart_domain_free,
	}
};

static irqreturn_t apple_dart_t8020_irq(int irq, void *dev)
{
	struct apple_dart *dart = dev;
	const char *fault_name = NULL;
	u32 error = apple_dart_readl(dart, DART_T8020_ERROR);
	u32 error_code = FIELD_GET(DART_T8020_ERROR_CODE, error);
	u32 addr_lo = apple_dart_readl(dart, DART_T8020_ERROR_ADDR_LO);
	u32 addr_hi = apple_dart_readl(dart, DART_T8020_ERROR_ADDR_HI);
	u64 addr = addr_lo | (((u64)addr_hi) << 32);
	u8 stream_idx = FIELD_GET(DART_T8020_ERROR_STREAM, error);

	if (!(error & DART_T8020_ERROR_FLAG))
		return IRQ_NONE;

	/* there should only be a single bit set but let's use == to be sure */
	if (error_code == DART_T8020_ERROR_READ_FAULT)
		fault_name = "READ FAULT";
	else if (error_code == DART_T8020_ERROR_WRITE_FAULT)
		fault_name = "WRITE FAULT";
	else if (error_code == DART_T8020_ERROR_NO_PTE)
		fault_name = "NO PTE FOR IOVA";
	else if (error_code == DART_T8020_ERROR_NO_PMD)
		fault_name = "NO PMD FOR IOVA";
	else if (error_code == DART_T8020_ERROR_NO_TTBR)
		fault_name = "NO TTBR FOR IOVA";
	else
		fault_name = "unknown";

	dev_err_ratelimited(
		dart->dev,
		"translation fault: status:0x%x stream:%d code:0x%x (%s) at 0x%llx",
		error, stream_idx, error_code, fault_name, addr);

	apple_dart_writel(dart, error, DART_T8020_ERROR);
	return IRQ_HANDLED;
}

static irqreturn_t apple_dart_t8110_legacy_irq(int irq, void *dev)
{
	struct apple_dart *dart = dev;
	const char *fault_name = NULL;
	u32 error = apple_dart_readl(dart, DART_T8110_ERROR);
	u32 error_code = FIELD_GET(DART_T8110_ERROR_CODE, error);
	u32 addr_lo = apple_dart_readl(dart, DART_T8110_ERROR_ADDR_LO);
	u32 addr_hi = apple_dart_readl(dart, DART_T8110_ERROR_ADDR_HI);
	u64 addr = addr_lo | (((u64)addr_hi) << 32);
	u8 stream_idx = FIELD_GET(DART_T8110_ERROR_STREAM, error);

	if (!(error & DART_T8110_ERROR_FLAG))
		return IRQ_NONE;

	/* there should only be a single bit set but let's use == to be sure */
	if (error_code == DART_T8110_ERROR_READ_FAULT)
		fault_name = "READ FAULT";
	else if (error_code == DART_T8110_ERROR_WRITE_FAULT)
		fault_name = "WRITE FAULT";
	else if (error_code == DART_T8110_ERROR_NO_PTE)
		fault_name = "NO PTE FOR IOVA";
	else if (error_code == DART_T8110_ERROR_NO_PMD)
		fault_name = "NO PMD FOR IOVA";
	else if (error_code == DART_T8110_ERROR_NO_PGD)
		fault_name = "NO PGD FOR IOVA";
	else if (error_code == DART_T8110_ERROR_NO_TTBR)
		fault_name = "NO TTBR FOR IOVA";
	else
		fault_name = "unknown";

	dev_err_ratelimited(
		dart->dev,
		"translation fault: status:0x%x stream:%d code:0x%x (%s) at 0x%llx",
		error, stream_idx, error_code, fault_name, addr);

	apple_dart_writel(dart, error, DART_T8110_ERROR);
	for (int i = 0; i < BITS_TO_U32(dart->num_streams); i++)
		apple_dart_writel(dart, U32_MAX,
				   DART_T8110_ERROR_STREAMS + 4 * i);

	return IRQ_HANDLED;
}

static const char *apple_dart_t8110_v2_fault_name(u32 exception)
{
	switch (FIELD_GET(DART_T8110_EXCEPTION_CODE, exception)) {
	case 0:
		switch (FIELD_GET(DART_T8110_EXCEPTION_LEVEL, exception)) {
		case 0:
			return "TTBR invalid";
		case 1:
			return "CTE invalid";
		case 2:
			return "STE invalid";
		case 3:
			return "PTE invalid";
		}
		break;
	case 1:
		return "write protect";
	case 2:
		return "read protect";
	case 3:
		return "drop protect";
	case 4:
		return "AXI decode";
	case 5:
		return "AXI error";
	case 6:
		return "fill region";
	case 7:
		return "region protect";
	case 8:
		return "CTRR write protect";
	case 9:
		return "APF reject";
	case 10:
		return "BPF reject";
	case 11:
		return "STT mismatch";
	case 12:
		return "STT flush";
	case 13:
		return "external";
	case 14:
		return "illegal SID";
	}

	return "unknown";
}

static bool apple_dart_t8110_v2_block_pending(u16 version, u32 block_error)
{
	return version >= 0x0203 &&
	       (block_error & DART_T8110_BLOCK_ERROR_MASK);
}

static irqreturn_t apple_dart_t8110_v2_irq(int irq, void *dev, bool gen3)
{
	struct apple_dart *dart = dev;
	irqreturn_t ret = IRQ_NONE;
	unsigned int words = DIV_ROUND_UP(dart->num_streams, 32);
	unsigned int i;
	u32 block_error;

	/*
	 * 0x4000 + 4*i is a bitmap of SIDs with a pending exception, one word
	 * per 32 SIDs -- not a per-SID code.  Iterating it per SID (as this
	 * used to) runs off the end of the array into the register arrays at
	 * 0x4020/0x4040 for every sid >= ceil(nsid/32).
	 */
	for (i = 0; i < words && i < 8; i++) {
		u32 pending = readl(dart->regs + DART_T8110_EXCEPTION(i));
		unsigned long bits = pending;
		unsigned int bit;

		if (!pending)
			continue;

		for_each_set_bit(bit, &bits, 32) {
			unsigned int sid = i * 32 + bit;
			u32 status;
			u64 addr;

			if (sid >= dart->num_streams)
				break;

			status = readl(dart->regs +
				       DART_T8110_ERR_SID_STATUS(sid));
			addr = readl(dart->regs +
				     DART_T8110_EXCEPTION_ADDR_LO(sid)) |
			       (u64)readl(dart->regs +
					  DART_T8110_EXCEPTION_ADDR_HI(sid)) << 32;

			dev_err_ratelimited(dart->dev,
					    "translation fault: SID:%u %s %s at %#llx (status %#010x)\n",
					    sid,
					    status & DART_T8110_ERR_SID_STATUS_WRITE ?
						    "write" : "read",
						    apple_dart_t8110_v2_fault_name(status),
						    addr, status);
			if (gen3)
				dev_err_ratelimited(dart->dev,
					"  SID:%u AXI_ID %#x/%#x AXI_USER %#x/%#x raw+4 %#x raw+10 %#x\n",
					sid,
					readl(dart->regs + DART_GEN3_ERR_SID_AXI_ID(sid)),
					readl(dart->regs + DART_GEN3_ERR_SID_AXI_ID(sid) + 4),
					readl(dart->regs + DART_GEN3_ERR_SID_AXI_USER(sid)),
					readl(dart->regs + DART_GEN3_ERR_SID_AXI_USER(sid) + 4),
					readl(dart->regs + DART_T8110_ERR_SID(sid) + 0x4),
					readl(dart->regs + DART_T8110_ERR_SID(sid) + 0x10));
		}

		/*
		 * Acknowledge: the bitmap is write-1-to-clear.  Without this the
		 * same latched bit re-fires immediately and one historical
		 * exception looks like a fault storm.
		 */
		writel(pending, dart->regs + DART_T8110_EXCEPTION(i));
		ret = IRQ_HANDLED;
	}

	if (ret == IRQ_HANDLED) {
		dsb(sy);
		readl(dart->regs + DART_T8110_EXCEPTION(0));
	}

	/* Gen3: 0x704 is not in AppleGen3DART's register capture list; do not touch it. */
	if (!gen3 && dart->version >= 0x0203) {
		block_error = readl(dart->regs + DART_T8110_BLOCK_ERROR);
		if (apple_dart_t8110_v2_block_pending(dart->version, block_error)) {
			dev_err_ratelimited(dart->dev,
					    "block exception: status:0x%08x\n",
					    block_error);
			writel(block_error & DART_T8110_BLOCK_ERROR_MASK,
			       dart->regs + DART_T8110_BLOCK_ERROR);
			ret = IRQ_HANDLED;
		}
	}
	if (ret == IRQ_HANDLED)
		apple_dart_t8110_v2_unlock(dart);

	return ret;
}

static irqreturn_t apple_dart_t8110_irq(int irq, void *dev)
{
	struct apple_dart *dart = dev;

	if (dart->version >= 0x0202)
		return apple_dart_t8110_v2_irq(irq, dev, false);

	return apple_dart_t8110_legacy_irq(irq, dev);
}

static irqreturn_t apple_dart_gen3_irq(int irq, void *dev)
{
	return apple_dart_t8110_v2_irq(irq, dev, true);
}

static irqreturn_t apple_dart_irq(int irq, void *dev)
{
	irqreturn_t ret = IRQ_NONE;
	struct apple_dart *dart = dev;

	if (READ_ONCE(dart->commands_gated))
		return IRQ_NONE;
	/*
	 * Static topology: no MMIO before the first consumer. A powered (runtime-active) DART that
	 * still raises a fault was never initialised: do the first-use init here, which also
	 * clears the pending error, rather than leave a level IRQ nobody claims.
	 */
	if (dart->static_topo && !dart->hw_ready) {
		if (!pm_runtime_active(dart->dev) || apple_dart_gen3_ensure_ready(dart))
			return IRQ_NONE;
		dev_warn_ratelimited(dart->dev, "fault before first-use init; initialised from the IRQ path\n");
		return IRQ_HANDLED;
	}
	if (pm_runtime_resume_and_get(dart->dev) < 0)
		return IRQ_NONE;
	if (!READ_ONCE(dart->commands_gated))
		ret = dart->hw->irq_handler(irq, dev);
	pm_runtime_put(dart->dev);
	return ret;
}

static bool apple_dart_is_locked(struct apple_dart *dart)
{
	return !!(apple_dart_readl(dart, dart->hw->lock) & dart->hw->lock_bit);
}

static void apple_dart_iounmap_np(void *data)
{
	iounmap((__force void __iomem *)data);
}

static void __iomem *
apple_dart_platform_ioremap_np_resource(struct platform_device *pdev,
					unsigned int index,
					struct resource **res_out)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *regs;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, index);
	if (!res)
		return IOMEM_ERR_PTR(-EINVAL);

	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     dev_name(dev)))
		return IOMEM_ERR_PTR(-EBUSY);

	regs = ioremap_np(res->start, resource_size(res));
	if (!regs)
		return IOMEM_ERR_PTR(-ENOMEM);

	ret = devm_add_action_or_reset(dev, apple_dart_iounmap_np,
				       (__force void *)regs);
	if (ret)
		return IOMEM_ERR_PTR(ret);

	*res_out = res;
	return regs;
}

static const struct apple_dart_hw apple_dart_hw_t8152;

static void apple_dart_read_params(struct apple_dart *dart)
{
	u32 dart_params[4];

	dart_params[0] = apple_dart_readl(dart, DART_PARAMS1);
	dart_params[1] = apple_dart_readl(dart, DART_PARAMS2);
	dart->pgsize = 1 << FIELD_GET(DART_PARAMS1_PAGE_SHIFT, dart_params[0]);
	dart->supports_bypass = dart_params[1] & DART_PARAMS2_BYPASS_SUPPORT;

	switch (dart->hw->type) {
	case DART_T8020:
	case DART_T6000:
		dart->ias = 32;
		dart->oas = dart->hw->oas;
		dart->num_streams = dart->hw->max_sid_count;
		break;

	case DART_T8110:
		dart_params[2] = apple_dart_readl(dart, DART_T8110_PARAMS3);
		dart->ias = FIELD_GET(DART_T8110_PARAMS3_VA_WIDTH, dart_params[2]);
		dart->oas = FIELD_GET(DART_T8110_PARAMS3_PA_WIDTH, dart_params[2]);
		dart_params[3] = apple_dart_readl(dart, DART_T8110_PARAMS4);
		dart->num_streams = FIELD_GET(DART_T8110_PARAMS4_NUM_SIDS,
					      dart_params[3]);
		dart->four_level = dart->ias > 36;
		dart->version = dart_params[2] & GENMASK(15, 0);
		break;
	}
}

/*
 * Gen3 static topology, no MMIO. ADT: page-size 16384 (m1n1 also expects page shift 14
 * and version 0x300), sid-count 16 for the USB DARTs (optional apple,num-streams). The
 * address widths are placeholders wide enough for probe-time window checks; the
 * first resume re-reads the real PARAMS and re-validates.
 */
static void apple_dart_set_static_params(struct apple_dart *dart)
{
	u32 n = 16;

	of_property_read_u32(dart->dev->of_node, "apple,num-streams", &n);
	dart->pgsize = SZ_16K;
	dart->supports_bypass = false;
	dart->ias = 42;
	dart->oas = 42;
	dart->num_streams = n;
	dart->four_level = true;
	dart->version = 0x0300;
}

/*
 * First use of a static-topology Gen3 DART: everything probe skipped, in the safe order.
 * Runs from the runtime-resume callback, which is irq-safe here: no sleeping calls. The
 * fault IRQ is requested at probe (no MMIO) and ignored until hw_ready.
 */
static int apple_dart_gen3_late_init(struct apple_dart *dart)
{
	struct device *dev = dart->dev;
	int ret;
	u32 pgsize;

	pgsize = dart->pgsize;
	apple_dart_read_params(dart);
	if (dart->pgsize != pgsize)
		return dev_err_probe(dev, -EINVAL, "page size %#x differs from the assumed %#x\n",
				     dart->pgsize, pgsize);
	if (dart->num_streams > DART_MAX_STREAMS)
		return dev_err_probe(dev, -EINVAL, "too many streams (%d)\n", dart->num_streams);
	if (of_property_present(dev->of_node, "apple,dma-range")) {
		if ((dart->dma_min ^ dart->dma_max) & ~DMA_BIT_MASK(dart->ias))
			return dev_err_probe(dev, -EINVAL, "DMA range does not fit ias=%d\n", dart->ias);
	} else {
		dart->dma_max = DMA_BIT_MASK(dart->ias);
	}

	/* Read the lock before any write. A locked root needs the locked-TTBR path. */
	if (apple_dart_is_locked(dart))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "locked Gen3 DART is not supported in static-topology mode\n");

	apple_dart_apply_tunables(dart);

	ret = apple_dart_hw_reset(dart);
	if (ret)
		return ret;
	dart->hw_ready = true;
	dev_info(dev, "Gen3 DART initialised on first use: pagesize %#x, %d streams, AS %d -> %d\n",
		 dart->pgsize, dart->num_streams, dart->ias, dart->oas);
	return 0;
}

/*
 * Run the first-use init exactly once from whichever path gets there first: runtime
 * resume, a consumer attach, or a fault IRQ on a powered but never initialised DART.
 * The caller guarantees the DART is powered. Safe in atomic context.
 */
static int apple_dart_gen3_ensure_ready(struct apple_dart *dart)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&dart->late_lock, flags);
	if (!dart->hw_ready)
		ret = apple_dart_gen3_late_init(dart);
	spin_unlock_irqrestore(&dart->late_lock, flags);
	return ret;
}

/* Every DART in a consumer's iommus list must be initialised before it is programmed. */
static int apple_dart_cfg_ensure_ready(struct apple_dart_master_cfg *cfg)
{
	struct apple_dart_stream_map *stream_map;
	int i, ret;

	for_each_stream_map(i, cfg, stream_map) {
		struct apple_dart *dart = stream_map->dart;

		if (!dart->static_topo || dart->hw_ready)
			continue;
		dev_warn(dart->dev,
			 "first-use init was not run by runtime resume (rpm status %d); initialising from attach\n",
			 dart->dev->power.runtime_status);
		ret = apple_dart_gen3_ensure_ready(dart);
		if (ret)
			return ret;
	}
	return 0;
}

/*
 * A DART of a PCIe-C tunnel sits in the container node of its tunneled root complex:
 * a sibling with a PCIe-C compatible marks it. The legacy container name is kept as a
 * second test so that no existing device tree changes its behaviour.
 */
static bool apple_dart_is_tunneled(struct device_node *np)
{
	struct device_node *parent = of_get_parent(np);
	struct device_node *child;
	bool tunneled;

	if (!parent)
		return false;

	tunneled = of_node_name_prefix(parent, "usb4-pcie-tunnel-");
	for_each_child_of_node(parent, child) {
		if (of_device_is_compatible(child, "apple,t8103-pciec") ||
		    of_device_is_compatible(child, "apple,t6000-pciec"))
			tunneled = true;
	}
	of_node_put(parent);
	return tunneled;
}

static int apple_dart_probe(struct platform_device *pdev)
{
	int ret;
	u32 sid;
	struct device_node *pd_np;
	struct resource *res;
	struct apple_dart *dart;
	struct device *dev = &pdev->dev;
	u64 dma_range[2];
	bool tunneled = apple_dart_is_tunneled(dev->of_node);

	dart = devm_kzalloc(dev, sizeof(*dart), GFP_KERNEL);
	if (!dart)
		return -ENOMEM;

	dart->dev = dev;
	dart->hw = of_device_get_match_data(dev);
	dart->static_topo = dart->hw->gen3 &&
		of_property_read_bool(dev->of_node, "apple,static-topology");
	dart->tunneled = tunneled;
	dart->per_stream = of_device_is_compatible(dev->of_node, "apple,t8140-dart") ||
			   of_device_is_compatible(dev->of_node, "apple,t8142-dart");
	if (tunneled) {
		pd_np = of_parse_phandle(dev->of_node, "power-domains", 0);
		if (pd_np) {
			dart->power_retained =
				of_property_read_bool(pd_np, "apple,always-on");
			of_node_put(pd_np);
		}
	}
	spin_lock_init(&dart->lock);
	spin_lock_init(&dart->late_lock);
	platform_set_drvdata(pdev, dart);

	/*
	 * PCIe-C is a cable-powered aperture whose writes must complete before
	 * the following transaction.  The normal arm64 ioremap() mapping is
	 * Device-nGnRE and allowed the first posted DART reset write to surface as
	 * an asynchronous external abort at a later instruction.  The same
	 * reset sequence is stable with a Device-nGnRnE mapping, and PCI
	 * configuration forwarding already requires the same non-posted
	 * ordering.  Keep ordinary always-on DARTs on their existing mapping.
	 */
	if (tunneled)
		dart->regs = apple_dart_platform_ioremap_np_resource(pdev, 0,
							      &res);
	else
		dart->regs = devm_platform_get_and_ioremap_resource(pdev, 0,
							     &res);
	if (IS_ERR(dart->regs))
		return PTR_ERR(dart->regs);

	if (resource_size(res) < 0x4000) {
		dev_err(dev, "MMIO region too small (%pr)\n", res);
		return -EINVAL;
	}
	if (tunneled || dart->hw == &apple_dart_hw_t8152) {
		dart->tunables = devm_apple_tunable_parse(dev, dev->of_node,
							  "apple,tunable", res);
		if (IS_ERR(dart->tunables)) {
			/*
			 * T8110 publishes tunables and the driver requires them.
			 * A t8103 PCIe-C DART may omit them. Its values include
			 * the configuration lock, which has to be written after
			 * the translation context is programmed, so leave them
			 * off until that ordering exists.
			 */
			if (PTR_ERR(dart->tunables) == -ENOENT &&
			    dart->hw->type != DART_T8110) {
				dart->tunables = NULL;
			} else {
				return dev_err_probe(dev, PTR_ERR(dart->tunables),
						     "failed to parse DART tunables\n");
			}
		}
	}
	if (dart->hw == &apple_dart_hw_t8152) {
		/* TLBI doorbell: reg[1], shared by every Gen3 DART (no mem region claim) */
		struct resource *tr = platform_get_resource(pdev, IORESOURCE_MEM, 1);

		if (!tr || resource_size(tr) < 16)
			return dev_err_probe(dev, -EINVAL, "Gen3 DART needs a 16-byte TLBI reg[1]\n");
		/*
		 * The doorbell uses the normal Device mapping, like the registers of the
		 * on-SoC DARTs; only the tunneled DARTs need ioremap_np(). Each doorbell
		 * store is followed by dsb ish, the TLBI and dsb osh (see the invalidate
		 * function), the vendor sequence.
		 */
		dart->tlbi = devm_ioremap(dev, tr->start, resource_size(tr));
		if (!dart->tlbi)
			return -ENOMEM;
	}

	dart->irq = platform_get_irq(pdev, 0);
	if (dart->irq < 0)
		return -ENODEV;

	ret = devm_clk_bulk_get_all(dev, &dart->clks);
	if (ret < 0)
		return ret;
	dart->num_clks = ret;

	ret = clk_bulk_prepare_enable(dart->num_clks, dart->clks);
	if (ret)
		return ret;

	pm_runtime_get_noresume(dev);
	pm_runtime_set_active(dev);
	pm_runtime_irq_safe(dev);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		goto err_clk_disable;
	if (tunneled && dart->tunables) {
		dev_dbg(dev, "applying %zu PCIe-C DART tunables\n",
			dart->tunables->sz);
		apple_dart_apply_tunables(dart);
	} else if (dart->hw == &apple_dart_hw_t8152 && !dart->static_topo) {
		/*
		 * Firmware (SPTM) owns a locked Gen3 DART: its protection and tunables are
		 * already set and the 0x20c group sits in the write-protect block, so decide
		 * lock state with a read before any write.
		 */
		if (apple_dart_is_locked(dart))
			dev_info(dev, "Gen3 DART is locked: not applying tunables\n");
		else
			apple_dart_apply_tunables(dart);
	}

	if (tunneled) {
		/*
		 * The parameter registers of a tunneled DART cannot be read
		 * safely at this point of PCIe tunnel activation: the read
		 * raises an asynchronous SError.  The topology is fixed
		 * instead.  T8110 is 16 KiB pages, 42-bit addresses, 64
		 * streams and four levels.  A T8020 PCIe-C DART is 16 KiB
		 * pages, a 32-bit IOVA, a 36-bit PA and as many streams as its
		 * register block describes.
		 */
		if (dart->hw->type == DART_T8110) {
			dart->pgsize = 1 << DART_PCIEC_PAGE_SHIFT;
			dart->supports_bypass = true;
			dart->ias = DART_PCIEC_ADDR_WIDTH;
			dart->oas = DART_PCIEC_ADDR_WIDTH;
			dart->num_streams = DART_PCIEC_STREAMS;
			dart->four_level = true;
		} else {
			dart->pgsize = 1 << DART_PCIEC_PAGE_SHIFT;
			dart->supports_bypass = false;
			dart->ias = 32;
			dart->oas = dart->hw->oas;
			dart->num_streams = dart->hw->max_sid_count;
			dart->four_level = false;
		}
		dev_dbg(dev,
			"PCIe-C DART using fixed topology: pagesize %#x, %u streams, AS %u -> %u\n",
			dart->pgsize, dart->num_streams, dart->ias, dart->oas);
		goto params_done;
	}

	if (dart->static_topo)
		apple_dart_set_static_params(dart);
	else
		apple_dart_read_params(dart);

params_done:
	dart->dma_min = 0;
	dart->dma_max = DMA_BIT_MASK(dart->ias);
	dart->dma_offset = 0;

	ret = of_property_read_u64_array(dev->of_node, "apple,dma-range", dma_range, 2);
	if (ret == -EINVAL) {
		ret = 0;
	} else if (ret) {
		goto err_clk_disable;
	} else {
		dart->dma_min = dma_range[0];
		ret = apple_dart_dma_window(dma_range[0], dma_range[1], &dart->dma_max);
		if (ret)
			goto err_clk_disable;
		if (!apple_dart_dma_window_aligned(dart->dma_min, dart->dma_max,
						   dart->pgsize)) {
			ret = -EINVAL;
			goto err_clk_disable;
		}
		if ((dart->dma_min ^ dart->dma_max) & ~DMA_BIT_MASK(dart->ias)) {
			dev_err(&pdev->dev, "Invalid DMA range for ias=%d\n",
				dart->ias);
			ret = -EINVAL;
			goto err_clk_disable;
		}
		dev_dbg(&pdev->dev, "Limiting DMA range to %pad..%pad\n",
			 &dart->dma_min, &dart->dma_max);
	}

	ret = of_property_read_u64(dev->of_node, "apple,dma-offset",
				   &dart->dma_offset);
	if (ret == -EINVAL) {
		dart->dma_offset = 0;
		ret = 0;
	} else if (ret) {
		goto err_clk_disable;
	} else if (dart->dma_offset > DMA_BIT_MASK(dart->ias) ||
		   dart->dma_max > DMA_BIT_MASK(dart->ias) - dart->dma_offset) {
		dev_err(dev, "Invalid DMA offset for ias=%u\n", dart->ias);
		ret = -EINVAL;
		goto err_clk_disable;
	} else {
		dev_dbg(dev,
			"aliasing DMA range %pad..%pad at DART offset %pad\n",
			&dart->dma_min, &dart->dma_max, &dart->dma_offset);
	}

	/*
	 * Older FDTs described the T602x PCIe-C DART-visible 1 TiB window
	 * directly. PCIe endpoints cannot issue those addresses: the host bridge
	 * accepts the upper 2 GiB of its 32-bit bus window and adds the 1 TiB
	 * alias before DART translation. Keep those FDTs bootable while returning
	 * the low bus address to DMA clients.
	 */
	if (!dart->dma_offset && tunneled &&
	    (of_machine_is_compatible("apple,t6020") ||
	     of_machine_is_compatible("apple,t6021")) &&
	    ((dart->dma_min == 0 &&
	      dart->dma_max == DMA_BIT_MASK(dart->ias)) ||
	     (dart->dma_min == BIT_ULL(40) &&
	      dart->dma_max == BIT_ULL(40) + BIT_ULL(32) - 1))) {
		dart->dma_min = BIT_ULL(31);
		dart->dma_max = DMA_BIT_MASK(32);
		dart->dma_offset = BIT_ULL(40);
		dev_info(dev,
			 "using T602x PCIe-C bus DMA window %pad..%pad at DART offset %pad\n",
			 &dart->dma_min, &dart->dma_max, &dart->dma_offset);
	}

	for (sid = 0; sid < DART_MAX_STREAMS; sid++)
		dart->locked_window[sid] = dart->dma_min + dart->dma_offset;

	if (dart->num_streams > DART_MAX_STREAMS) {
		dev_err(&pdev->dev, "Too many streams (%d > %d)\n",
			dart->num_streams, DART_MAX_STREAMS);
		ret = -EINVAL;
		goto err_clk_disable;
	}

	ret = apple_dart_read_remaps(dart);
	if (ret)
		goto err_clk_disable;

	/*
	 * A hot-plug PCIe-C DART has no retained/locked translation context.
	 * A DART the bootloader leaves locked is one that hardware we do not drive
	 * still translates through (on T8152 Gen3 a display pipe scans out through
	 * the boot stream the whole time Linux is booting). Letting runtime PM
	 * suspend it drops the TTBRs underneath that traffic, and the engine then
	 * faults "TTBR invalid" forever. Hold the reference for the DART's lifetime.
	 */
	if (tunneled)
		dart->locked = false;
	else
		dart->locked = !dart->static_topo && apple_dart_is_locked(dart);
	if (dart->hw->gen3 && dart->locked && !dart->force_available) {
		dev_info(dev, "locked DART: holding it available for its lifetime\n");
		dart->force_available = 1;
	}
	/* Reserved table-page lifetime requires the matched three-level loader. */
	dart->fw_handoff = apple_dart_fw_handoff_enabled(dart);
	/* Preserve our retained-root path when no reserved-table handoff exists. */
	dart->fw_mirror = dart->locked && !dart->fw_handoff &&
		(of_device_is_compatible(dev->of_node, "apple,t8140-dart") ||
		 of_device_is_compatible(dev->of_node, "apple,t8142-dart"));
	if (!dart->locked && !dart->static_topo) {
		ret = apple_dart_hw_reset(dart);
		if (ret)
			goto err_clk_disable;
	}

	if (dart->static_topo)
		dev_info(dev, "Gen3 DART static topology: no MMIO until first use\n");
	if (dart->locked)
		apple_dart_drain_stale_exceptions(dart);
	ret = request_irq(dart->irq, apple_dart_irq, IRQF_SHARED,
			  "apple-dart fault handler", dart);
	if (ret)
		goto err_clk_disable;
	dart->irq_requested = true;

	ret = iommu_device_sysfs_add(&dart->iommu, dev, NULL, "apple-dart.%s",
				     dev_name(&pdev->dev));
	if (ret)
		goto err_free_irq;

	ret = iommu_device_register(&dart->iommu, &apple_dart_iommu_ops, dev);
	if (ret)
		goto err_sysfs_remove;

	if (!dart->force_available)
		pm_runtime_put(dev);

	dev_dbg(
		&pdev->dev,
		"DART [pagesize %x, %d streams, bypass support: %d, bypass forced: %d, locked: %d, AS %d -> %d] initialized\n",
		dart->pgsize, dart->num_streams, dart->supports_bypass,
		dart->pgsize > PAGE_SIZE, dart->locked, dart->ias, dart->oas);
	return 0;

err_sysfs_remove:
	iommu_device_sysfs_remove(&dart->iommu);
err_free_irq:
	if (dart->irq_requested)
		free_irq(dart->irq, dart);
err_clk_disable:
	pm_runtime_put(dev);
	clk_bulk_disable_unprepare(dart->num_clks, dart->clks);

	return ret;
}

int apple_dart_quiesce_commands(struct device *dev)
{
	struct apple_dart *dart = dev_get_drvdata(dev);

	if (dart) {
		/* Finish any in-flight command before the port clock is stopped. */
		scoped_guard(spinlock_irqsave, &dart->lock)
			WRITE_ONCE(dart->commands_gated, true);
		/* The fault handler can take dart->lock while acknowledging errors. */
		if (dart->irq > 0)
			synchronize_irq(dart->irq);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(apple_dart_quiesce_commands);

int apple_dart_resume_commands(struct device *dev)
{
	struct apple_dart *dart = dev_get_drvdata(dev);
	int ret;

	if (!dart)
		return -ENODEV;
	if (!READ_ONCE(dart->commands_gated))
		return 0;

	scoped_guard(spinlock_irqsave, &dart->lock)
		WRITE_ONCE(dart->commands_gated, false);
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		apple_dart_quiesce_commands(dev);
		return ret;
	}
	pm_runtime_put(dev);
	return 0;
}
EXPORT_SYMBOL_GPL(apple_dart_resume_commands);

/*
 * System sleep has quiesced every downstream device. PCIe-C calls this
 * before stopping APPCLK because its domain reset also loses DART state,
 * even when ordinary runtime PM treats the domain as retained.
 */
int apple_dart_save_tunnel_state(struct device *dev)
{
	struct apple_dart *dart = dev_get_drvdata(dev);
	unsigned int sid, idx;

	if (!dart || !dart->tunneled || dart->locked ||
	    dart->hw->type != DART_T8020 || dart->tunables)
		return -EOPNOTSUPP;

	guard(spinlock_irqsave)(&dart->lock);
	if (dart->commands_gated)
		return -EBUSY;
	dart->tunnel_state_saved = false;
	dart->tunnel_state_restored = false;

	for (sid = 0; sid < dart->num_streams; sid++) {
		dart->save_tcr[sid] = apple_dart_readl(dart, DART_TCR(dart, sid));
		for (idx = 0; idx < dart->hw->ttbr_count; idx++)
			dart->save_ttbr[sid][idx] =
				apple_dart_readl(dart, DART_TTBR(dart, sid, idx));
	}
	dart->tunnel_state_saved = true;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_dart_save_tunnel_state);

/* APPCLK is running again, but PCIe link training has not started. */
int apple_dart_restore_tunnel_state(struct device *dev)
{
	struct apple_dart *dart = dev_get_drvdata(dev);
	unsigned int sid, idx;
	int ret;

	if (!dart || !dart->tunneled || dart->locked ||
	    dart->hw->type != DART_T8020 || dart->tunables)
		return -EOPNOTSUPP;
	if (!dart->tunnel_state_saved || READ_ONCE(dart->commands_gated))
		return -EINVAL;

	ret = apple_dart_hw_reset(dart);
	if (ret)
		return ret;

	for (sid = 0; sid < dart->num_streams; sid++) {
		for (idx = 0; idx < dart->hw->ttbr_count; idx++)
			apple_dart_writel(dart, dart->save_ttbr[sid][idx],
					  DART_TTBR(dart, sid, idx));
		apple_dart_writel(dart, dart->save_tcr[sid], DART_TCR(dart, sid));
	}
	dart->tunnel_state_saved = false;
	dart->tunnel_state_restored = true;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_dart_restore_tunnel_state);

static void apple_dart_remove(struct platform_device *pdev)
{
	struct apple_dart *dart = platform_get_drvdata(pdev);

	/*
	 * Cable removal gates the port clock before this device is removed.
	 * A command issued then never completes. The next probe resets the
	 * block after that clock is running again.
	 */
	if (!dart->locked && (!dart->static_topo || dart->hw_ready) &&
	    !READ_ONCE(dart->commands_gated))
		apple_dart_hw_reset(dart);

	if (dart->irq_requested)
		free_irq(dart->irq, dart);

	iommu_device_unregister(&dart->iommu);
	iommu_device_sysfs_remove(&dart->iommu);
	if (dart->force_available)
		pm_runtime_put_noidle(dart->dev);

	clk_bulk_disable_unprepare(dart->num_clks, dart->clks);
}

static const struct apple_dart_hw apple_dart_hw_t8103 = {
	.type = DART_T8020,
	.irq_handler = apple_dart_t8020_irq,
	.invalidate_tlb = apple_dart_t8020_hw_invalidate_tlb,
	.enable_remap = apple_dart_t8020_hw_enable_remap,
	.oas = 36,
	.fmt = APPLE_DART,
	.max_sid_count = 16,

	.enable_streams = DART_T8020_STREAMS_ENABLE,
	.lock = DART_T8020_CONFIG,
	.lock_bit = DART_T8020_CONFIG_LOCK,

	.error = DART_T8020_ERROR,

	.tcr = DART_T8020_TCR,
	.tcr_enabled = DART_T8020_TCR_TRANSLATE_ENABLE,
	.tcr_disabled = 0,
	.tcr_bypass = DART_T8020_TCR_BYPASS_DAPF | DART_T8020_TCR_BYPASS_DART,

	.ttbr = DART_T8020_TTBR,
	.ttbr_valid = DART_T8020_TTBR_VALID,
	.ttbr_addr_field_shift = DART_T8020_TTBR_ADDR_FIELD_SHIFT,
	.ttbr_shift = DART_T8020_TTBR_SHIFT,
	.ttbr_count = 4,
};

static const struct apple_dart_hw apple_dart_hw_t8103_usb4 = {
	.type = DART_T8020,
	.irq_handler = apple_dart_t8020_irq,
	.invalidate_tlb = apple_dart_t8020_hw_invalidate_tlb,
	.enable_remap = apple_dart_t8020_hw_enable_remap,
	.oas = 36,
	.fmt = APPLE_DART,
	.max_sid_count = 64,

	.enable_streams = DART_T8020_STREAMS_ENABLE,
	.lock = DART_T8020_CONFIG,
	.lock_bit = DART_T8020_CONFIG_LOCK,

	.error = DART_T8020_ERROR,

	.tcr = DART_T8020_TCR,
	.tcr_enabled = DART_T8020_TCR_TRANSLATE_ENABLE,
	.tcr_disabled = 0,
	.tcr_bypass = 0,

	.ttbr = DART_T8020_USB4_TTBR,
	.ttbr_valid = DART_T8020_TTBR_VALID,
	.ttbr_addr_field_shift = DART_T8020_TTBR_ADDR_FIELD_SHIFT,
	.ttbr_shift = DART_T8020_TTBR_SHIFT,
	.ttbr_count = 4,
};

static const struct apple_dart_hw apple_dart_hw_t6000 = {
	.type = DART_T6000,
	.irq_handler = apple_dart_t8020_irq,
	.invalidate_tlb = apple_dart_t8020_hw_invalidate_tlb,
	.enable_remap = apple_dart_t8020_hw_enable_remap,
	.oas = 42,
	.fmt = APPLE_DART2,
	.max_sid_count = 16,

	.enable_streams = DART_T8020_STREAMS_ENABLE,
	.lock = DART_T8020_CONFIG,
	.lock_bit = DART_T8020_CONFIG_LOCK,

	.error = DART_T8020_ERROR,

	.tcr = DART_T8020_TCR,
	.tcr_enabled = DART_T8020_TCR_TRANSLATE_ENABLE,
	.tcr_disabled = 0,
	.tcr_bypass = DART_T8020_TCR_BYPASS_DAPF | DART_T8020_TCR_BYPASS_DART,

	.ttbr = DART_T8020_TTBR,
	.ttbr_valid = DART_T8020_TTBR_VALID,
	.ttbr_addr_field_shift = DART_T8020_TTBR_ADDR_FIELD_SHIFT,
	.ttbr_shift = DART_T8020_TTBR_SHIFT,
	.ttbr_count = 4,
};

static const struct apple_dart_hw apple_dart_hw_t8110 = {
	.type = DART_T8110,
	.irq_handler = apple_dart_t8110_irq,
	.invalidate_tlb = apple_dart_t8110_hw_invalidate_tlb,
	.enable_remap = apple_dart_t8110_hw_enable_remap,
	.fmt = APPLE_DART2,
	.max_sid_count = 256,

	.enable_streams = DART_T8110_ENABLE_STREAMS,
	.lock = DART_T8110_PROTECT,
	.lock_bit = DART_T8110_PROTECT_TTBR_TCR,

	.error = DART_T8110_ERROR,

	.tcr = DART_T8110_TCR,
	.tcr_enabled = DART_T8110_TCR_TRANSLATE_ENABLE,
	.tcr_disabled = 0,
	.tcr_bypass = DART_T8110_TCR_BYPASS_DAPF | DART_T8110_TCR_BYPASS_DART,
	.tcr_4level = DART_T8110_TCR_FOUR_LEVEL,

	.ttbr = DART_T8110_TTBR,
	.ttbr_valid = DART_T8110_TTBR_VALID,
	.ttbr_addr_field_shift = DART_T8110_TTBR_ADDR_FIELD_SHIFT,
	.ttbr_shift = DART_T8110_TTBR_SHIFT,
	.ttbr_count = 1,
};

static const struct apple_dart_hw apple_dart_hw_t8152 = {
	.type = DART_T8110,
	.irq_handler = apple_dart_gen3_irq,
	.invalidate_tlb = apple_dart_gen3_hw_invalidate_tlb,
	.gen3 = true,
	.fmt = APPLE_DART2,
	.max_sid_count = 256,

	.enable_streams = DART_T8110_ENABLE_STREAMS,
	.lock = DART_T8110_PROTECT,
	.lock_bit = DART_T8110_PROTECT_TTBR_TCR,

	.error = DART_T8110_ERROR,

	.tcr = DART_T8110_TCR,
	.tcr_enabled = DART_T8110_TCR_TRANSLATE_ENABLE,
	.tcr_disabled = 0,
	.tcr_bypass = DART_T8110_TCR_BYPASS_DAPF | DART_T8110_TCR_BYPASS_DART,
	.tcr_4level = DART_T8110_TCR_FOUR_LEVEL,

	.ttbr = DART_T8110_TTBR,
	.ttbr_valid = DART_T8110_TTBR_VALID,
	.ttbr_addr_field_shift = DART_T8110_TTBR_ADDR_FIELD_SHIFT,
	.ttbr_shift = DART_T8110_TTBR_SHIFT,
	.ttbr_count = 1,
};

static __maybe_unused int apple_dart_suspend(struct device *dev)
{
	struct apple_dart *dart = dev_get_drvdata(dev);
	unsigned int sid, idx;

	dart->tunnel_state_restored = false;
	/* A static-topology DART untouched so far has no state and must not be read. */
	if (dart->static_topo && !dart->hw_ready)
		return 0;

	if (READ_ONCE(dart->commands_gated))
		return 0;
	/* The tunneled DART has no state to save when its domain stays on. */
	if (dart->power_retained)
		return 0;

	/* Locked DARTs can't be restored so skip saving their registers. */
	if (dart->locked)
		return 0;

	for (sid = 0; sid < dart->num_streams; sid++) {
		dart->save_tcr[sid] = apple_dart_readl(dart,
						       DART_TCR(dart, sid));
		for (idx = 0; idx < dart->hw->ttbr_count; idx++)
			dart->save_ttbr[sid][idx] =
				apple_dart_readl(dart,
						 DART_TTBR(dart, sid, idx));
	}

	return 0;
}

/*
 * Whether the DART still holds the translation state that apple_dart_suspend()
 * saved, because its power domain stayed on across system sleep. That is the
 * case while a consumer that keeps running in suspend-to-idle, such as a DWC3
 * controller in host mode, holds the domain on. Such a consumer may still be
 * doing DMA, so the DART must not be reset under it.
 */
static bool apple_dart_state_retained(struct apple_dart *dart)
{
	bool live = false;
	unsigned int sid, idx;

	for (sid = 0; sid < dart->num_streams; sid++) {
		u32 tcr = apple_dart_readl(dart, DART_TCR(dart, sid));

		if (tcr != dart->save_tcr[sid])
			return false;
		/* An identity domain bypasses the DART without any page table. */
		if (dart->supports_bypass && dart->hw->tcr_bypass && tcr == dart->hw->tcr_bypass)
			live = true;

		for (idx = 0; idx < dart->hw->ttbr_count; idx++) {
			u32 ttbr = apple_dart_readl(dart, DART_TTBR(dart, sid, idx));

			if (ttbr != dart->save_ttbr[sid][idx])
				return false;
			if (ttbr & dart->hw->ttbr_valid)
				live = true;
		}
	}

	/*
	 * A DART that lost power reads back as reset: no stream bypasses it and
	 * none has a valid page table.
	 */
	return live;
}

/*
 * Flush the TLBs of every stream with a command of its own. A stream map flush
 * is not enough for the whole DART: where the streams of a map are remapped
 * onto its first stream, it only reaches that first stream.
 */
static int apple_dart_invalidate_each_stream(struct apple_dart *dart)
{
	unsigned int sid;
	int ret;

	for (sid = 0; sid < dart->num_streams; sid++) {
		struct apple_dart_stream_map stream_map = { .dart = dart };

		__set_bit(sid, stream_map.sidmap);
		ret = dart->hw->invalidate_tlb(&stream_map);
		if (ret)
			return ret;
	}

	return 0;
}

static __maybe_unused int apple_dart_resume(struct device *dev)
{
	struct apple_dart *dart = dev_get_drvdata(dev);
	unsigned int sid, idx;
	int ret;

	/* First resume of a static-topology DART: do everything probe skipped. */
	if (dart->static_topo && !dart->hw_ready)
		return apple_dart_gen3_ensure_ready(dart);

	/* PCIe-C restored these mappings before starting its downstream link. */
	if (dart->tunnel_state_restored) {
		dart->tunnel_state_restored = false;
		return 0;
	}

	/*
	 * PCIe-C DARTs on an apple,always-on domain retain their translation
	 * state. Resetting one here is unnecessary; DART access must remain
	 * ordered after PCIe-C port resume.
	 */
	if (dart->power_retained || READ_ONCE(dart->commands_gated))
		return 0;

	/* Locked DARTs can't be restored, and they should not need it */
	if (dart->locked)
		return 0;

	if (apple_dart_state_retained(dart)) {
		/* Nothing was lost, only drop what the TLBs may still hold. */
		if (!dart->reset_incomplete)
			return apple_dart_invalidate_each_stream(dart);

		/* A failed resume restored the translations: finish its reset. */
		ret = apple_dart_hw_finish_reset(dart);
		dart->reset_incomplete = ret;
		return ret;
	}

	ret = apple_dart_hw_reset(dart);

	/*
	 * Restore the translations even if the reset failed. It can only fail
	 * in its last steps, after clearing them, and the saved registers still
	 * point to the page tables in use. Leaving them cleared would fault
	 * every DMA of the consumers until the next reset.
	 */
	for (sid = 0; sid < dart->num_streams; sid++) {
		for (idx = 0; idx < dart->hw->ttbr_count; idx++)
			apple_dart_writel(dart, dart->save_ttbr[sid][idx],
					  DART_TTBR(dart, sid, idx));
		apple_dart_writel(dart, dart->save_tcr[sid],
				   DART_TCR(dart, sid));
	}

	/*
	 * Then repeat all of those last steps once. A TLB flush that completes
	 * now does not show that the DART was unlocked and its error interrupt
	 * unmasked.
	 */
	if (ret) {
		dev_warn(dev, "reset on resume failed (%d), retrying\n", ret);
		ret = apple_dart_hw_finish_reset(dart);
	}

	/* Should they fail again, the next resume repeats them. */
	dart->reset_incomplete = ret;
	return ret;
}

static DEFINE_RUNTIME_DEV_PM_OPS(apple_dart_pm_ops, apple_dart_suspend, apple_dart_resume, NULL);

static const struct of_device_id apple_dart_of_match[] = {
	{ .compatible = "apple,t8103-dart", .data = &apple_dart_hw_t8103 },
	{ .compatible = "apple,t8103-usb4-dart", .data = &apple_dart_hw_t8103_usb4 },
	{ .compatible = "apple,t8110-dart", .data = &apple_dart_hw_t8110 },
	{ .compatible = "apple,t8152-dart", .data = &apple_dart_hw_t8152 },
	{ .compatible = "apple,t6000-dart", .data = &apple_dart_hw_t6000 },
	{},
};
MODULE_DEVICE_TABLE(of, apple_dart_of_match);

static struct platform_driver apple_dart_driver = {
	.driver	= {
		.name			= "apple-dart",
		.of_match_table		= apple_dart_of_match,
		.suppress_bind_attrs    = true,
		.pm			= pm_ptr(&apple_dart_pm_ops),
	},
	.probe	= apple_dart_probe,
	.remove = apple_dart_remove,
};

module_platform_driver(apple_dart_driver);

#if IS_ENABLED(CONFIG_KUNIT) && !defined(MODULE)
static void apple_dart_fw_leaf_test(struct kunit *test)
{
	const u64 leaf = GENMASK_ULL(51, 40) |
			 ((u64)(0x280000 >> 4) & GENMASK_ULL(37, 10)) |
			 BIT_ULL(0);
	const int rwc = IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE;
	phys_addr_t phys;
	int prot;
	bool valid, matched;

	valid = apple_dart_fw_leaf_decode(0, 0x4000, &phys, &prot);
	KUNIT_EXPECT_FALSE(test, valid);
	valid = apple_dart_fw_leaf_decode(BIT_ULL(0), 0x4000,
					  &phys, &prot);
	KUNIT_EXPECT_FALSE(test, valid);
	valid = apple_dart_fw_leaf_decode(leaf, 0x4000, &phys, &prot);
	KUNIT_ASSERT_TRUE(test, valid);
	matched = apple_dart_fw_mapping_matches(phys, prot, 0x280000, rwc);
	KUNIT_EXPECT_TRUE(test, matched);
	matched = apple_dart_fw_mapping_matches(phys, prot, 0x284000, rwc);
	KUNIT_EXPECT_FALSE(test, matched);
	valid = apple_dart_fw_leaf_decode(leaf | BIT_ULL(2), 0x4000,
					  &phys, &prot);
	KUNIT_ASSERT_TRUE(test, valid);
	matched = apple_dart_fw_mapping_matches(phys, prot, 0x280000, rwc);
	KUNIT_EXPECT_FALSE(test, matched);
}

static struct kunit_case apple_dart_fw_cases[] = {
	KUNIT_CASE(apple_dart_fw_leaf_test),
	{}
};

static struct kunit_suite apple_dart_fw_suite = {
	.name = "apple-dart-firmware-leaf",
	.test_cases = apple_dart_fw_cases,
};

kunit_test_suite(apple_dart_fw_suite);
#endif

#if IS_ENABLED(CONFIG_APPLE_DART_KUNIT_TEST)
#include "apple-dart-test.c"
#endif

MODULE_DESCRIPTION("IOMMU API for Apple's DART");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_LICENSE("GPL v2");
