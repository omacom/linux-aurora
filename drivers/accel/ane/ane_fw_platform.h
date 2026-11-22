/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __ANE_FW_PLATFORM_H__
#define __ANE_FW_PLATFORM_H__

#include <linux/align.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/overflow.h>
#include <linux/sizes.h>
#include <linux/types.h>

/* DAPF records: four on T6041 and T8140, five on T6050 and T8132. */
#define ANE_FW_DAPF_MAX 5

struct device_node;
struct platform_device;
struct ane_h16_profile;
struct ane_h16_mmio;

enum ane_fw_owner {
	ANE_FW_OWNER_AP,
	ANE_FW_OWNER_EXCLAVE,
};

enum ane_fw_irq_role {
	ANE_FW_IRQ_UNKNOWN,
	ANE_FW_IRQ_CHANNELS,
};

/* Later firmware generations must never enter the H14 staging path. */
struct ane_fw_platform {
	const struct ane_h16_profile *h16_profiles;
	unsigned int h16_profile_count;
	/* DART/DAPF register tables, one per supported DART revision. */
	const struct ane_h16_mmio *h16_mmio;
	unsigned int h16_mmio_count;
	/* DART PARAMS3, the one DART register read before a table is chosen. */
	u32 dart_params;
	/* The DAPF bank: iommus entry that owns it, offset from its reg, size. */
	u32 dapf_dart;
	u32 dapf_offset;
	u32 dapf_size;
	u32 soc;
	u16 chip_family;
	u16 ane_type;
	u8 hwx_subtype;
	u8 task_generation;
	u8 interrupt_count;
	enum ane_fw_irq_role irq_role;
	enum ane_fw_owner owner;
	const char * const *reg_names;
	const u32 *reg_sizes;
	u32 reg_count;
	u32 wrapper;
	u32 mailbox;
	u32 scratch;
	u32 doorbell;
	u32 irq_status;
	u32 irq_ack;
	const u32 *power_words;
	u32 power_count;
	u64 preload_dva[2];
	u64 preload_size[2];
	u16 stream_mask;
	/* Boot qualification is platform-specific, never implied by generation. */
	bool boot_qualified;
	bool firmware_preloaded;
	bool safe_reset;
	/*
	 * System sleep first takes the firmware out of Exclave mode
	 * (EXCLAVE_MODE_STOP on the control ring) so that SUSPEND can run.
	 */
	bool exclave_mode_stop;
	/*
	 * The sleep macOS was recorded doing on this SoC: the engine stays off with
	 * no firmware until its first request, and powers down in macOS's word
	 * order. Other SoCs keep the staged backend's boot-time start.
	 */
	bool macos_sleep;
	/*
	 * System sleep keeps the firmware (EXCLAVE_MODE_STOP, SUSPEND, warm
	 * boot); otherwise the engine is cycled cold and the programs replayed.
	 */
	bool sleep_warm;
	/*
	 * H16: system sleep cycles the engine cold (no firmware command, cold
	 * boot and program replay after resume) when sleep_warm is off.
	 * Without either, H16 refuses system sleep.
	 */
	bool sleep_cold;
	/*
	 * The PMP report of the SoC: the firmware is first released at boot only
	 * once that PMP runs (see ane_h16_wait_pmp()).
	 */
	const char *pmp_report;
	/*
	 * Staged (H17) backend: the power state words in the "pmgr" window
	 * (ANE_SYS, which genpd owns, then the CPU and the sub-block, which the
	 * backend switches), the ANE DART compatible and revision, the DAPF
	 * record count and the pinned TEXT digest.
	 */
	u32 pmgr_sys, pmgr_cpu, pmgr_subblock;
	const char *dart_compatible;
	u16 dart_revision;
	u8 dapf_count;
	const u8 *text_pin;
	/* H16: an aux-3 window, which the build's XL tunables go to. */
	bool has_xl;
	/*
	 * Or the build's code: the TEXT digest with the loader-filled
	 * environment sections zeroed (__TEXT,text_env and, from 27.2,
	 * __TEXT,ce_env; ascending, not overlapping, unused ones size 0). The
	 * loader fills them in (the image has a fixed pattern or zeroes there),
	 * so unlike text_pin this is computed from the image.
	 */
	const u8 *text_code_pin;
	struct ane_fw_text_env {
		u32 offset, size;
	} text_env[2];
	/*
	 * Another firmware build the loader may preload on this SoC, with its
	 * own preload layout and pins; the handed-over layout selects it.
	 */
	const struct ane_fw_platform *next_build;
	const char *build_name;
};

/* Pure range check: the boot loader supplies the physical addresses. */
static inline bool
ane_fw_preload_ranges_valid(const struct ane_fw_platform *soc, const u64 phys[2],
			    const u64 size[2], const u64 dva[2])
{
	u64 end[2];
	unsigned int i;

	for (i = 0; i < 2; i++) {
		if (!phys[i] || !IS_ALIGNED(phys[i], SZ_16K) ||
		    size[i] != soc->preload_size[i] ||
		    dva[i] != soc->preload_dva[i] ||
		    check_add_overflow(phys[i], size[i], &end[i]))
			return false;
	}
	return end[0] <= phys[1] || end[1] <= phys[0];
}

/*
 * The system sleep path of a device the ane_t6021 driver bound. Devices of
 * the shared firmware platform (H16, H17) carry the backend's drvdata and
 * must never reach the H13/H14 rtclient sleep path, which would read that
 * drvdata as an rtclient.
 */
enum ane_fw_sleep_path {
	ANE_SLEEP_RTCLIENT,
	ANE_SLEEP_FW_BACKEND,
};

/*
 * The device binds for the staged backend (H17): the engine is started by
 * the backend, the bound device carries its power domain and sleep callbacks.
 */
static inline bool ane_fw_platform_staged(const struct ane_fw_platform *platform)
{
	return platform && platform->owner == ANE_FW_OWNER_EXCLAVE;
}

static inline enum ane_fw_sleep_path
ane_fw_sleep_path(const struct ane_fw_platform *platform)
{
	return platform ? ANE_SLEEP_FW_BACKEND : ANE_SLEEP_RTCLIENT;
}

extern const struct ane_fw_platform ane_h16_platform, ane_h17_platform, ane_t6050_platform,
				    ane_t8142_platform;
extern const struct ane_fw_platform ane_t8132_platform;
int ane_fw_platform_preflight(struct platform_device *pdev,
			      const struct ane_fw_platform *soc);

int ane_fw_platform_probe(struct platform_device *pdev,
			  const struct ane_fw_platform *soc);
void ane_fw_platform_remove(struct platform_device *pdev);
int ane_fw_platform_suspend(struct device *dev);
int ane_fw_platform_resume(struct device *dev);
int ane_fw_preload_range(struct device_node *np, unsigned int index, u64 *phys, u64 *size);
const struct ane_fw_platform *ane_fw_platform_build(struct device_node *np,
						    const struct ane_fw_platform *soc);
int ane_h16_backend_probe(struct platform_device *pdev, const struct ane_fw_platform *soc);
void ane_h16_backend_remove(struct platform_device *pdev);
int ane_h16_backend_suspend(struct device *dev);
int ane_h16_backend_resume(struct device *dev);

#endif /* __ANE_FW_PLATFORM_H__ */
