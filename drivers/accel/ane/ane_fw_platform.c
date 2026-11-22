// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * H16/H17 facts from the 2026-10-31 H16/H17 specification, sections 3-7,
 * 12 and 15, and the H16 cold-boot facts, round 5. Metadata preflight
 * performs no MMIO, DMA mapping or power access.
 */
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <crypto/sha2.h>

#include "ane_fw_cold.h"
#include "ane_fw_platform.h"
#include "ane_h16_boot_data.h"

static const char * const ane_h16_reg_names[] = {
	"engine", "pmgr", "aux-0", "aux-1", "aux-2", "aux-3", "aux-4",
};

static const u32 ane_h16_reg_sizes[] = {
	0x2000000, 0x18000, 0x4000, 0x30000, 0xff4000, 0x2000000, 0x4000,
};

/* T8132 has no second engine (aux-3); retain the ADT window ordering. */
static const char * const ane_t8132_reg_names[] = {
	"engine", "pmgr", "aux-0", "aux-1", "aux-2", "aux-4",
};

static const u32 ane_t8132_reg_sizes[] = {
	0x2000000, 0x18000, 0x4000, 0x30000, 0xff4000, 0x4000,
};

/* Offsets in reg[1]; genpd, never the ANE driver, owns transitions. */
static const u32 ane_h16_power_words[] = {
	0x3c0, 0xc008, 0xc000, 0xc010, 0xc018,
};

/* The same five domains; ANE_SYS sits at +0x570 in the T8132 PMGR. */
static const u32 ane_t8132_power_words[] = {
	0x570, 0xc008, 0xc000, 0xc010, 0xc018,
};

static const char * const ane_h17_reg_names[] = { "engine", "pmgr", "pwgate" };
static const u32 ane_h17_reg_sizes[] = { 0x2000000, 0x18000, 0x34 };

/*
 * The measured TEXT of each staged firmware. A different build is refused
 * before anything is powered or written.
 */
static const u8 ane_h17_text_pin_26_6_2[] = {
	0xcd, 0xa0, 0x9e, 0xc1, 0xae, 0xe6, 0x8b, 0x88,
	0xb4, 0x7e, 0xdc, 0x0e, 0xeb, 0xc8, 0x10, 0x45,
	0x25, 0x32, 0xa8, 0x55, 0x56, 0x3f, 0xbd, 0x65,
	0x0b, 0x95, 0xe2, 0x84, 0x6c, 0x34, 0x53, 0x10,
};

static const u8 ane_t6050_text_pin_27_0[] = {
	0x93, 0xae, 0x3a, 0x37, 0x1c, 0xaa, 0x93, 0xf2,
	0xa9, 0x26, 0x8c, 0x5b, 0xfd, 0x94, 0xba, 0xa9,
	0x9d, 0x8d, 0xd5, 0x9c, 0xbe, 0xc9, 0x99, 0xe2,
	0xd1, 0xa2, 0x20, 0xd0, 0xee, 0x43, 0x8b, 0x8f,
};

const struct ane_fw_platform ane_h16_platform = {
	.h16_profiles = ane_h16_profiles,
	.h16_profile_count = ARRAY_SIZE(ane_h16_profiles),
	.h16_mmio = ane_h16_mmio_t6041,
	.h16_mmio_count = ARRAY_SIZE(ane_h16_mmio_t6041),
	.dart_params = 0x8,
	.dapf_dart = 0,
	.dapf_offset = 0x10000,
	.dapf_size = SZ_16K,
	.dapf_count = 4,
	.has_xl = true,
	/* The DT compatible is t6041 on the M4 Max and the M4 Pro (T6040) alike. */
	.soc = 0x6040,
	.chip_family = 16,
	.ane_type = 0x110,
	.hwx_subtype = 7,
	.task_generation = 17,
	.interrupt_count = 1,
	/* J616s IRQ 1054 drains replies and events from all F2H channels. */
	.irq_role = ANE_FW_IRQ_CHANNELS,
	.owner = ANE_FW_OWNER_AP,
	.reg_names = ane_h16_reg_names,
	.reg_sizes = ane_h16_reg_sizes,
	.reg_count = ARRAY_SIZE(ane_h16_reg_names),
	.wrapper = 0x1600000,
	.mailbox = 0x1608000,
	.scratch = 0x1880020,
	.doorbell = 0x1884000,
	.irq_status = 0x188c000,
	.irq_ack = 0x1890000,
	.power_words = ane_h16_power_words,
	.power_count = ARRAY_SIZE(ane_h16_power_words),
	.preload_dva = { 0x10000000000, 0x100000c0000 },
	.preload_size = { 0xc0000, 0x2ac000 },
	.stream_mask = BIT(0) | BIT(11),
	/*
	 * System sleep cycles the engine cold and replays every program.
	 * The warm path stays off: on 25G83 the J616s firmware does not take
	 * SUSPEND (0x24), bare or after SET_SNE_PMU_BASE2 and
	 * EXCLAVE_MODE_STOP.
	 */
	.sleep_cold = true,
	.boot_qualified = true,
	.firmware_preloaded = true,
	.safe_reset = false,
	.pmp_report = "apple,t6041-pmp-v2-report",
};

/*
 * M4: the H16 engine and firmware interface of the T6041, with one engine
 * (no aux-3 window), its own engine writes and firmware builds, ANE_SYS at
 * PMGR +0x570 and five DAPF records. The backend polls the F2H channels.
 */
#define ANE_T8132_PLATFORM						\
	.h16_mmio = ane_h16_mmio_t6041,					\
	.h16_mmio_count = ARRAY_SIZE(ane_h16_mmio_t6041),		\
	.dart_params = 0x8,						\
	.soc = 0x8132,							\
	.chip_family = 16,						\
	.ane_type = 0x100,						\
	.hwx_subtype = 7,						\
	.task_generation = 17,						\
	.interrupt_count = 2,						\
	.irq_role = ANE_FW_IRQ_UNKNOWN,					\
	.owner = ANE_FW_OWNER_AP,					\
	.reg_names = ane_t8132_reg_names,				\
	.reg_sizes = ane_t8132_reg_sizes,				\
	.reg_count = ARRAY_SIZE(ane_t8132_reg_names),			\
	.wrapper = 0x1600000,						\
	.mailbox = 0x1608000,						\
	.scratch = 0x1880020,						\
	.doorbell = 0x1884000,						\
	.irq_status = 0x188c000,					\
	.irq_ack = 0x1890000,						\
	.power_words = ane_t8132_power_words,				\
	.power_count = ARRAY_SIZE(ane_t8132_power_words),		\
	.dapf_dart = 0,							\
	.dapf_offset = 0x10000,						\
	.dapf_size = SZ_16K,						\
	.dapf_count = 5,						\
	.stream_mask = BIT(0) | BIT(11),				\
	/* System sleep cycles the engine cold and replays every program, as on T6041. */ \
	.sleep_cold = true,						\
	.boot_qualified = true,						\
	.firmware_preloaded = true,					\
	.safe_reset = false,						\
	.pmp_report = "apple,t8132-pmp-v2-report"

/*
 * The macOS 27.2 (26B123) firmware: iBoot preloads TEXT 0xbc000 and DATA
 * 0x2b0000, DATA at 0x100_000b_c000, as on T6050 and T8142 at 27.2.
 */
static const struct ane_fw_platform ane_t8132_platform_26b123 = {
	ANE_T8132_PLATFORM,
	.h16_profiles = ane_t8132_profiles_26b123,
	.h16_profile_count = ARRAY_SIZE(ane_t8132_profiles_26b123),
	.build_name = "27.2 26B123",
	.preload_dva = { 0x10000000000, 0x100000bc000 },
	.preload_size = { 0xbc000, 0x2b0000 },
};

const struct ane_fw_platform ane_t8132_platform = {
	ANE_T8132_PLATFORM,
	.h16_profiles = ane_t8132_profiles,
	.h16_profile_count = ARRAY_SIZE(ane_t8132_profiles),
	.build_name = "26.6.2 25G83",
	.preload_dva = { 0x10000000000, 0x100000c0000 },
	.preload_size = { 0xc0000, 0x2ac000 },
	.next_build = &ane_t8132_platform_26b123,
};

const struct ane_fw_platform ane_h17_platform = {
	.soc = 0x8140,
	.chip_family = 17,
	.ane_type = 0x200,
	.hwx_subtype = 9,
	.task_generation = 19,
	.interrupt_count = 2,
	.owner = ANE_FW_OWNER_EXCLAVE,
	.reg_names = ane_h17_reg_names,
	.reg_sizes = ane_h17_reg_sizes,
	.reg_count = ARRAY_SIZE(ane_h17_reg_names),
	.wrapper = 0x1600000,
	.mailbox = 0x1608000,
	.scratch = 0x1880020,
	.doorbell = 0x1884000,
	.irq_status = 0x188c000,
	.irq_ack = 0x1890000,
	.preload_dva = { 0x10000000000, 0x100000c0000 },
	.preload_size = { 0xc0000, 0x2ac000 },
	.stream_mask = BIT(0),
	.boot_qualified = true,
	.firmware_preloaded = true,
	.safe_reset = false,
	/* 25G83 boots this firmware in Exclave mode (SPEC-ANE-SLEEP-H17.md). */
	.exclave_mode_stop = true,
	.macos_sleep = true,
	/*
	 * Off: on 25G83 SUSPEND is still not taken after 0x31 (C7), so sleep
	 * cycles the engine cold and replays every program behind its handle.
	 */
	.sleep_warm = false,
	/* Cold AP-direct acquisition is staged before any power or DMA write.
	 * The observer uses the measured H17 26.6.2 register layout.
	 */
	.pmgr_sys = 0x290,
	.pmgr_cpu = 0xc000,
	.pmgr_subblock = 0xc008,
	.dart_compatible = "apple,t8140-ane-dart",
	.dart_revision = 0x203,
	.dapf_count = 4,
	.text_pin = ane_h17_text_pin_26_6_2,
};

/*
 * The M5 Pro and Max (J714s, J716c) ANE: the H17 engine and firmware protocol
 * (ADT "ane,t8132exclave", ane-type 0x220), exclave-assigned and preloaded by
 * iBoot. From the ADT and macOS's own bring-up on this machine: ANE_SYS at
 * PMGR +0x3a8, then ANE_CPU (+0x3c8) and ANE_TDBASE (+0x3d0) behind it; the
 * scratch cells start at +0x1880028; the DARTs are revision 0x204; five DAPF
 * records. System sleep cycles the engine cold, as on T8140.
 */
#define ANE_T6050_PLATFORM						\
	.soc = 0x6050,							\
	.chip_family = 17,						\
	.ane_type = 0x220,						\
	.hwx_subtype = 9,						\
	.task_generation = 19,						\
	.interrupt_count = 2,						\
	.owner = ANE_FW_OWNER_EXCLAVE,					\
	.reg_names = ane_h17_reg_names,					\
	.reg_sizes = ane_h17_reg_sizes,					\
	.reg_count = ARRAY_SIZE(ane_h17_reg_names),			\
	.wrapper = 0x1600000,						\
	.mailbox = 0x1608000,						\
	.scratch = 0x1880028,						\
	.doorbell = 0x1884000,						\
	.irq_status = 0x188c000,					\
	.irq_ack = 0x1890000,						\
	.stream_mask = BIT(0),						\
	.boot_qualified = true,						\
	.firmware_preloaded = true,					\
	.safe_reset = false,						\
	.sleep_warm = false,						\
	.pmgr_sys = 0x3a8,						\
	.pmgr_cpu = 0x3c8,						\
	.pmgr_subblock = 0x3d0,						\
	/* Normal T6050 DARTs, as on T6041; SID0 is the host's. */	\
	.dart_compatible = "apple,t6050-dart",				\
	.dart_revision = 0x204,						\
	.dapf_count = 5

/*
 * The macOS 27.2 (26B123) firmware (h17_ane0_fw_hyperion_j71y, RTKit
 * 3514.41.1): iBoot preloads TEXT 0xbc000 and DATA 0x2b0000, DATA at
 * 0x100_000b_c000, as on T8142. The loader fills ce_env (0x4000 at 0xb4f80)
 * and text_env (0x20 at 0xb8f80); the code pin is from the image with both
 * zeroed.
 */
static const u8 ane_t6050_code_26b123[] = {
	0x9f, 0x3c, 0xc8, 0x2b, 0x43, 0xc6, 0x16, 0x80,
	0xdf, 0x68, 0x06, 0x47, 0x10, 0x50, 0xc3, 0xd4,
	0x69, 0xd6, 0x94, 0x7c, 0x6d, 0x7b, 0x9d, 0x84,
	0x56, 0x63, 0xff, 0xe5, 0x89, 0x1a, 0xc7, 0x19,
};

static const struct ane_fw_platform ane_t6050_platform_26b123 = {
	ANE_T6050_PLATFORM,
	.build_name = "27.2 26B123",
	.preload_dva = { 0x10000000000, 0x100000bc000 },
	.preload_size = { 0xbc000, 0x2b0000 },
	.text_code_pin = ane_t6050_code_26b123,
	.text_env = { { 0xb4f80, 0x4000 }, { 0xb8f80, 0x20 } },
};

/*
 * The macOS 27.0 (26A428) firmware (h17_ane0_fw_hyperion_j71y): TEXT 0xc0000
 * and DATA 0x2ac000. The loader fills ce_env (0x4000 at 0xb8280; it carries
 * a board table, so the J714s and the J716c load different TEXT) and
 * text_env (0x20 at 0xbc280). text_pin is the J714s's TEXT as loaded; the
 * code pin, from the image with both sections zeroed, admits either board.
 */
static const u8 ane_t6050_code_26a428[] = {
	0xf8, 0x14, 0x14, 0x39, 0x08, 0x71, 0xd7, 0xd5,
	0xeb, 0x1b, 0x4f, 0x7d, 0xdd, 0x5b, 0xdb, 0x25,
	0x8b, 0xd9, 0x41, 0x44, 0x6e, 0xcc, 0xe1, 0x98,
	0x3c, 0xe8, 0x70, 0x12, 0x48, 0x82, 0xdb, 0x29,
};

const struct ane_fw_platform ane_t6050_platform = {
	ANE_T6050_PLATFORM,
	.build_name = "27.0 26A428",
	.preload_dva = { 0x10000000000, 0x100000c0000 },
	.preload_size = { 0xc0000, 0x2ac000 },
	.text_pin = ane_t6050_text_pin_27_0,
	.text_code_pin = ane_t6050_code_26a428,
	.text_env = { { 0xb8280, 0x4000 }, { 0xbc280, 0x20 } },
	.next_build = &ane_t6050_platform_26b123,
};

/*
 * The M5 (T8142, J813/J815) ANE: "ane,t8132exclave", ane-type 0x210,
 * exclave-assigned and preloaded by iBoot. From the T8142 ADT: ANE_SYS at
 * PMGR +0x2e8 (pmgr device 73, parent AFISOCNI3), ANE_CPU +0xc000 and
 * ANE_TDBASE +0xc008 as on T8140; four DAPF records.
 *
 * Measured under aura's hypervisor on a J815 running macOS 27.2.1 (cold
 * model load, power-up and power-down traced): the PWGATE window at PMGR
 * +0x27640 (request +0xc, acknowledge +0x30, as on T8140), the scratch
 * cells from +0x1880028 (as on T6050), DART revision 0x204 (PARAMS_8
 * 0x2a2a0204) and the four DAPF records. macOS powers the engine down in
 * T8140's order: sub-block, PWGATE request 3, acknowledge 0, CPU, then
 * ANE_SYS. So sleep and the parked start follow T8140.
 */
#define ANE_T8142_PLATFORM						\
	.soc = 0x8142,							\
	.chip_family = 17,						\
	.ane_type = 0x210,						\
	.hwx_subtype = 9,						\
	.task_generation = 19,						\
	.interrupt_count = 2,						\
	.owner = ANE_FW_OWNER_EXCLAVE,					\
	.reg_names = ane_h17_reg_names,					\
	.reg_sizes = ane_h17_reg_sizes,					\
	.reg_count = ARRAY_SIZE(ane_h17_reg_names),			\
	.wrapper = 0x1600000,						\
	.mailbox = 0x1608000,						\
	.scratch = 0x1880028,						\
	.doorbell = 0x1884000,						\
	.irq_status = 0x188c000,					\
	.irq_ack = 0x1890000,						\
	.stream_mask = BIT(0),						\
	.firmware_preloaded = true,					\
	.safe_reset = false,						\
	/* Exclave mode at boot, cold sleep cycle and parked start. */	\
	.exclave_mode_stop = true,					\
	.macos_sleep = true,						\
	.sleep_warm = false,						\
	.pmgr_sys = 0x2e8,						\
	.pmgr_cpu = 0xc000,						\
	.pmgr_subblock = 0xc008,					\
	.dart_compatible = "apple,t8142-ane-dart",			\
	.dart_revision = 0x204,						\
	.dapf_count = 4

/*
 * The macOS 27.2 (26B123) firmware, which 27.2.1 (26B131) ships unchanged:
 * the 27.2 host ADT has iBoot preload TEXT 0xbc000 and DATA 0x2b0000, DATA
 * at 0x100_000b_c000; both end where 27.0's do. Its TEXT has two sections
 * the loader fills: ce_env (0x4000 bytes at 0xb4f80, new in this build, all
 * zero in the image but its first byte) and text_env (0x20 at 0xb8f80). The
 * code pin is from the image (h17_ane_fw_theia_j73y, RTKit 3514.41.1) with
 * both zeroed; with text_env alone the first J815 boot missed it.
 */
static const u8 ane_t8142_code_26b123[] = {
	0x9b, 0x57, 0x48, 0x67, 0x90, 0xff, 0x41, 0x45,
	0x8e, 0x1d, 0x33, 0x46, 0x34, 0xed, 0x36, 0x53,
	0xa4, 0xb4, 0xd3, 0xcd, 0xf4, 0x43, 0xe2, 0x17,
	0x7d, 0x95, 0x6d, 0x11, 0x1d, 0xa7, 0xf4, 0xc2,
};

static const struct ane_fw_platform ane_t8142_platform_26b123 = {
	ANE_T8142_PLATFORM,
	.build_name = "27.2 26B123",
	.preload_dva = { 0x10000000000, 0x100000bc000 },
	.preload_size = { 0xbc000, 0x2b0000 },
	.text_code_pin = ane_t8142_code_26b123,
	.text_env = { { 0xb4f80, 0x4000 }, { 0xb8f80, 0x20 } },
};

/*
 * The macOS 27.0 (26A428) stub's iBoot preloads TEXT 0xc0000 and DATA
 * 0x2ac000 at T8140's IOVAs. TEXT as loaded was measured on a J815 under
 * aura; it differs from the image's (RTKit 3514.0.15) only by what iBoot
 * writes into text_env (0xbc280), which the code pin leaves out.
 */
static const u8 ane_t8142_text_26a428[] = {
	0x12, 0x12, 0xc2, 0x60, 0x6e, 0xf8, 0xca, 0xce,
	0x51, 0xfd, 0x95, 0x1e, 0x60, 0xbc, 0xce, 0x75,
	0xbe, 0x8e, 0x65, 0xd5, 0xf0, 0xfe, 0x19, 0x37,
	0xac, 0x3b, 0x58, 0x07, 0x49, 0xd6, 0xb4, 0xe3,
};

static const u8 ane_t8142_code_26a428[] = {
	0xaa, 0x5c, 0x7b, 0xd4, 0xdb, 0x5b, 0x17, 0x25,
	0x5d, 0x4a, 0x16, 0xa5, 0x80, 0x06, 0xca, 0x37,
	0x96, 0x26, 0xf3, 0x91, 0x58, 0x1b, 0x9c, 0x49,
	0x0a, 0xcd, 0x7c, 0x56, 0x6f, 0xda, 0x24, 0x9c,
};

const struct ane_fw_platform ane_t8142_platform = {
	ANE_T8142_PLATFORM,
	.build_name = "27.0 26A428",
	.preload_dva = { 0x10000000000, 0x100000c0000 },
	.preload_size = { 0xc0000, 0x2ac000 },
	.text_pin = ane_t8142_text_26a428,
	.text_code_pin = ane_t8142_code_26a428,
	.text_env = { { 0xbc280, 0x20 } },
	.next_build = &ane_t8142_platform_26b123,
};

/*
 * The iBoot preload is firmware memory. Accept it only as a no-map child of
 * /reserved-memory that Linux does not manage as System RAM, whether the
 * loader reserved it at boot or left it outside the memory map. The physical
 * range comes from the node itself, so the check does not depend on when the
 * node was added.
 */
int ane_fw_preload_range(struct device_node *np, unsigned int index, u64 *phys, u64 *size)
{
	struct device_node *reserved __free(device_node) =
		of_find_node_by_path("/reserved-memory");
	struct device_node *mem __free(device_node) =
		of_parse_phandle(np, "memory-region", index);
	struct resource r;

	if (!reserved || !mem || mem->parent != reserved ||
	    !of_device_is_available(mem) ||
	    !of_property_read_bool(mem, "no-map") ||
	    of_property_read_bool(mem, "reusable") ||
	    of_address_to_resource(mem, 0, &r) || !resource_size(&r))
		return -EINVAL;
	if (region_intersects(r.start, resource_size(&r), IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_DISJOINT)
		return -EBUSY;
	*phys = r.start;
	*size = resource_size(&r);
	return 0;
}
EXPORT_SYMBOL_GPL(ane_fw_preload_range);

/*
 * The build whose preload layout the loader handed over: the SoC's own data
 * or one of its next_build entries. Anything else (no usable hand-off, an
 * unknown layout) keeps the SoC's own data, which then refuses as before.
 */
const struct ane_fw_platform *ane_fw_platform_build(struct device_node *np,
						    const struct ane_fw_platform *soc)
{
	const struct ane_fw_platform *build;
	u64 phys[2], size[2], dva[2];

	if (!soc->next_build ||
	    of_property_count_u64_elems(np, "apple,firmware-iovas") != 2 ||
	    of_property_read_u64_array(np, "apple,firmware-iovas", dva, 2) ||
	    ane_fw_preload_range(np, 0, &phys[0], &size[0]) ||
	    ane_fw_preload_range(np, 1, &phys[1], &size[1]))
		return soc;
	for (build = soc; build; build = build->next_build)
		if (ane_fw_preload_ranges_valid(build, phys, size, dva))
			return build;
	return soc;
}
EXPORT_SYMBOL_GPL(ane_fw_platform_build);

static int ane_fw_preload_check(struct device *dev,
				const struct ane_fw_platform *soc)
{
	struct device_node *np = dev->of_node;
	u64 phys[2], size[2], dva[2];
	unsigned int i;
	int ret;

	if (of_property_count_u32_elems(np, "memory-region") != 2 ||
	    of_property_count_strings(np, "memory-region-names") != 2 ||
	    of_property_count_u64_elems(np, "apple,firmware-iovas") != 2 ||
	    of_property_read_u64_array(np, "apple,firmware-iovas", dva, 2))
		return -ENODATA;

	for (i = 0; i < 2; i++) {
		struct device_node *mem __free(device_node) =
			of_parse_phandle(np, "memory-region", i);
		const char *name;

		/* Board DTs carry disabled placeholders the loader fills in. */
		if (mem && !of_device_is_available(mem))
			return -ENODEV;
		if (of_property_read_string_index(np, "memory-region-names", i,
						  &name) ||
		    strcmp(name, i ? "firmware-data" : "firmware-text"))
			return -EINVAL;
		ret = ane_fw_preload_range(np, i, &phys[i], &size[i]);
		if (ret)
			return ret;
	}

	return ane_fw_preload_ranges_valid(soc, phys, size, dva) ? 0 : -EINVAL;
}

int ane_fw_platform_preflight(struct platform_device *pdev,
			      const struct ane_fw_platform *soc)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	u32 ane_type;
	unsigned int i;
	int ret;

	/* Check the build the loader preloaded (a no-op for a selected build). */
	soc = ane_fw_platform_build(np, soc);
	/* Refuse unqualified SoCs before resources, DMA, power or MMIO. */
	if (!soc->boot_qualified)
		return dev_err_probe(dev, -EOPNOTSUPP,
			"ANE boot unqualified for SoC %#x; requires owner qualification and bootloader firmware preload support\n",
			soc->soc);

	/* No backend has acquired ownership, even if DT omits the marker.
	 * Check before resources, firmware, power, IOMMU or engine access.
	 */
	/*
	 * H17 is started (or left alone) by the staged path before the driver
	 * registers; the platform probe never acquires it. -ENODEV keeps the
	 * driver core quiet; the staged path already said why.
	 */
	if (soc->owner == ANE_FW_OWNER_EXCLAVE)
		return -ENODEV;
	if (of_property_read_bool(np, "apple,exclave-assigned")) {
		dev_info(dev, "ANE: exclave-assigned device has no Linux ownership provider\n");
		return -EOPNOTSUPP;
	}

	if (!of_property_read_bool(np, "apple,firmware-preloaded") ||
	    of_property_read_u32(np, "apple,ane-type", &ane_type) ||
	    ane_type != soc->ane_type)
		return dev_err_probe(dev, -EINVAL, "H16 firmware identity missing or invalid\n");

	if (of_property_count_strings(np, "reg-names") != soc->reg_count ||
	    platform_get_resource(pdev, IORESOURCE_MEM, soc->reg_count) ||
	    of_irq_count(np) != soc->interrupt_count)
		return dev_err_probe(dev, -EINVAL, "H16 resource count mismatch\n");

	for (i = 0; i < soc->reg_count; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   soc->reg_names[i]);
		if (!res || resource_size(res) != soc->reg_sizes[i])
			return dev_err_probe(dev, -EINVAL, "H16 resource geometry mismatch\n");
	}

	ret = ane_fw_preload_check(dev, soc);
	if (ret == -ENODEV) {
		dev_info(dev, "H16 firmware preload not handed off by the bootloader\n");
		return ret;
	}
	if (ret)
		return dev_err_probe(dev, ret,
			"H16 requires bootloader-reserved no-map TEXT/DATA preload ranges\n");

	if (!soc->h16_profile_count || !soc->h16_mmio_count || soc->dapf_dart >= 3 ||
	    !soc->dapf_size || !soc->dapf_count || soc->dapf_count > ANE_FW_DAPF_MAX ||
	    of_property_count_u64_elems(np, "apple,dapf-ranges") != soc->dapf_count * 2)
		return -ENODATA;
	return 0;
}

int ane_fw_platform_probe(struct platform_device *pdev, const struct ane_fw_platform *soc)
{
	int ret;

	/* H17: the staged backend owns the engine; the device binds for it. */
	if (ane_fw_platform_staged(soc))
		return ane_fw_staged_bind(pdev);
	/* The firmware build the loader preloaded, if the SoC has several. */
	soc = ane_fw_platform_build(pdev->dev.of_node, soc);
	ret = ane_fw_platform_preflight(pdev, soc);

	if (ret)
		return ret;
	return ane_h16_backend_probe(pdev, soc);
}

void ane_fw_platform_remove(struct platform_device *pdev)
{
	if (ane_fw_staged_bound(&pdev->dev)) {
		ane_fw_staged_unbind(pdev);
		return;
	}
	ane_h16_backend_remove(pdev);
}

/* System sleep callbacks of the bound H16 or H17 device. */
int ane_fw_platform_suspend(struct device *dev)
{
	if (ane_fw_staged_bound(dev))
		return ane_fw_staged_suspend(dev);
	return ane_h16_backend_suspend(dev);
}

int ane_fw_platform_resume(struct device *dev)
{
	if (ane_fw_staged_bound(dev))
		return 0;
	return ane_h16_backend_resume(dev);
}
