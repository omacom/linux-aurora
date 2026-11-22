// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple AVE video encoder
 *
 * The AVE block runs an encoder firmware on its own coprocessor, which the
 * bootloader loads but does not start. This driver starts it on first use,
 * talks to it over two shared-memory rings and exposes it as a V4L2
 * stateful encoder.
 *
 * Copyright The Aurora Silicon Contributors
 */
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/limits.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>

#include "ave.h"
#include "ave-regs.h"
#include "ave-mctf.h"

#define AVE_AUTOSUSPEND_MS 2000

static const struct ave_coproc_regs ave_coproc_v40 = {
	.vcbar = AVE_COPROC_VCBAR,
	.timebase_freq = AVE_COPROC_TIMEBASE_FREQ,
	.timebase = AVE_COPROC_TIMEBASE,
	.ctrl = AVE_COPROC_CTRL,
	.status = AVE_COPROC_STATUS,
	.boot_arg = AVE_COPROC_BOOT_ARG,
	.keep_mappings = AVE_COPROC_KEEP_MAPPINGS,
};

/* Panda/Castor register family used by 6070.11.1. */
static const struct ave_coproc_regs ave_coproc_6070 = {
	.vcbar = 0x50000,
	.timebase_freq = 0x160020,
	.timebase = 0x178000,
	.ctrl = 0x400044,
	.status = 0x400048,
	.boot_arg = 0x400400,
	.keep_mappings = 0x400808,
};

const struct ave_variant ave_t6000 = {
	.regs = &ave_coproc_6070,
	.fw_text_size = 0xd0000,
	.fw_data_size = 0x128000,
	.boot_device_id = 14,
	.device_type = 11,
	.lfs_ref_scale = 1,
	.text_crc = 0x460c7a7c,
	.boot_arg_value = 0x400,
	.timebase_multiplier = 1000000,
	.text_link_cell = 0x423c,
	.mbox_idle = 0x38,

	.extra0_size = 0x24,
	.boot_qualified = true,
};

const struct ave_variant ave_t8112 = {
	.regs = &ave_coproc_6070,
	.fw_text_size = 0xd0000,
	.fw_data_size = 0x128000,
	.boot_device_id = 18,
	.device_type = 15,
	.lfs_ref_scale = 1,
	.text_crc = 0x25bfd039,
	.boot_arg_value = 0x10000,
	.timebase_multiplier = 1,
	.text_link_cell = 0x423c,
	.mbox_idle = 0x38,
	.restore_data_first = true,

	.extra0_size = 0x4000,
	.boot_qualified = true,
};

/*
 * The M4 Pro (T6040 in the ADT, apple,t6041 in the DT) starts its encoder
 * like the A18 Pro: same coprocessor registers and boot sequence, its own
 * device ID and type. The M4 Max (J575c) runs its first encoder, ADT ave0
 * with soc-id t6040, the same way; iBoot 25G83 preloads it with a 0x130000
 * DATA and sets a word at TEXT 0x4224 differently on every boot.
 */
const struct ave_variant ave_t6041 = {
	.regs = &ave_coproc_v40,
	.fw_text_size = 0x14c000,
	.fw_data_size = 0x12c000,
	.fw_data_size_alt = 0x130000,
	.text_boot_word = 0x4224,
	.boot_device_id = 27,
	.boot_arg_value = AVE_COPROC_BOOT_ARG_VALUE,
	.timebase_multiplier = 1,
	.vcbar_mask = U64_MAX,
	.vcbar_flags = AVE_COPROC_VCBAR_VALID,
	.device_type = 25,
	.lfs_ref_scale = 2,
	.boot_qualified = true,
};

/*
 * The M3 (T8122) with the macOS 27.2 (26B123) stub, as on the MacBook Air
 * (J615): the bootloader preloads TEXT 0x134000 and DATA 0x128000, TEXT CRC
 * 918d70d3, the same on every boot. The coprocessor registers and their start
 * are the M2's; the firmware is the 27.2 build of the M4 generation, so it
 * takes the boot configuration handshake, as device 17 with handshake type 17,
 * and speaks the 26B123 command layout. Its codec clocks stay gated until the
 * mailbox idle word is cleared, and the bootloader's code base (marked valid)
 * is left as it is. DATA is restored from the boot snapshot before every start.
 */
static const struct ave_variant ave_t8122 = {
	.regs = &ave_coproc_6070,
	.fw_text_size = 0x134000,
	.fw_data_size = 0x128000,
	.boot_device_id = 17,
	.boot_arg_value = AVE_COPROC_BOOT_ARG_VALUE,
	.timebase_multiplier = 1,
	.vcbar_mask = GENMASK_ULL(41, 11),
	.vcbar_flags = AVE_COPROC_VCBAR_VALID,
	.device_type = 17,
	.lfs_ref_scale = 2,
	.text_crc = AVE_FW_TEXT_CRC_T8122_26B123,
	.mbox_idle = 0x38,
	.restore_data_first = true,
	.boot_qualified = true,
};

/*
 * The M4 (T8132) with the macOS 27.2 (26B123) stub: the bootloader preloads
 * TEXT 0x154000 and DATA 0x12c000, TEXT CRC 99fc929a with the per-boot word
 * at 0x4224 zeroed. Some loaders leave that word zero (the MacBook Air, J713),
 * others set it on every boot (the Mac mini, J773g, raw CRC f641c64b), so an
 * unknown CRC is retried with it zeroed. The coprocessor and its start are
 * the M4 Pro's; this build boots as device 23 and takes handshake type 23.
 */
const struct ave_variant ave_t8132 = {
	.regs = &ave_coproc_v40,
	.fw_text_size = 0x154000,
	.fw_data_size = 0x12c000,
	.boot_device_id = 23,
	.boot_arg_value = AVE_COPROC_BOOT_ARG_VALUE,
	.timebase_multiplier = 1,
	.vcbar_mask = U64_MAX,
	.vcbar_flags = AVE_COPROC_VCBAR_VALID,
	.device_type = 23,
	.lfs_ref_scale = 2,
	.text_crc = AVE_FW_TEXT_CRC_T8132_26B123,
	.text_boot_word = 0x4224,
	.boot_qualified = true,
};

static const struct ave_variant ave_t8140 = {
	.regs = &ave_coproc_v40,
	.fw_text_size = 0x14c000,
	.fw_data_size = 0x130000,
	.boot_device_id = 29,
	.boot_arg_value = AVE_COPROC_BOOT_ARG_VALUE,
	.timebase_multiplier = 1,
	.vcbar_mask = U64_MAX,
	.vcbar_flags = AVE_COPROC_VCBAR_VALID,
	.device_type = 27,
	.lfs_ref_scale = 2,
	.boot_qualified = true,
};

/*
 * The M5 Pro and Max (T6050) run the macOS 27.0 (26A428) firmware build, preloaded with TEXT
 * 0x154000 and DATA 0x134000 (ADT ave0 segment-ranges). macOS starts the coprocessor as on the
 * A18 Pro, after clearing the SVE idle word at mailbox +0x118. Its boot configuration names
 * device 28, and so does its handshake's device type. The macOS 27.3 beta 1 build is admitted
 * beside it.
 */
static const struct ave_fw_build ave_t6050_builds[] = {
	{ "26A428", 0x154000, 0x134000, AVE_FW_TEXT_CRC_T6050_26A428 },
	/*
	 * macOS 27.3 beta 1 (iBoot 20457.60.65): TEXT is 0x4000 larger, DATA
	 * keeps its size. It starts as 26A428 does and takes 26A428's command
	 * layout (ave_layout_for_crc()).
	 */
	{ "26C5035", 0x158000, 0x134000, AVE_FW_TEXT_CRC_T6050_26C5035 },
};

static const struct ave_variant ave_t6050 = {
	.regs = &ave_coproc_v40,
	.fw_text_size = 0x154000,
	.fw_data_size = 0x134000,
	.builds = ave_t6050_builds,
	.build_count = ARRAY_SIZE(ave_t6050_builds),
	.boot_device_id = 28,
	.boot_arg_value = AVE_COPROC_BOOT_ARG_VALUE,
	.timebase_multiplier = 1,
	.vcbar_mask = U64_MAX,
	.vcbar_flags = AVE_COPROC_VCBAR_VALID,
	.device_type = 28,
	.lfs_ref_scale = 2,
	.text_crc = AVE_FW_TEXT_CRC_T6050_26A428,
	.mbox_idle = 0x118,
	.boot_qualified = true,
};

/*
 * The M5 (T8142) with the macOS 27.0 stub runs the 26A428 build of the
 * firmware compiled for its chip (9013.45.1, the version T6050 runs):
 * TEXT 0x154000 and DATA 0x12c000, TEXT CRC 8a3784bd. The 27.2 stub's build
 * (9013.55.1) has the same segment sizes, so its CRC selects it.
 *
 * The chip identity comes from the ADT (ave soc-id
 * "t8142"): device 27, generation 16. The 26A428 firmware carries the same
 * device table, and on T6050 (device 28 in both) the traced boot
 * configuration and handshake both name that number. Generation 16 shares
 * T6050's SVE register block, so the idle word at mailbox +0x118 is cleared
 * before the start as there. The coprocessor is the A18 Pro's.
 */
static const struct ave_fw_build ave_t8142_builds[] = {
	{ "26A428", 0x154000, 0x12c000, AVE_FW_TEXT_CRC_T8142_26A428 },
	/* macOS 27.2 (and 27.2.1): CRC from the image, 26A428's command layout. */
	{ "26B123", 0x154000, 0x12c000, AVE_FW_TEXT_CRC_T8142_26B123 },
};

static const struct ave_variant ave_t8142 = {
	.regs = &ave_coproc_v40,
	.fw_text_size = 0x154000,
	.fw_data_size = 0x12c000,
	.builds = ave_t8142_builds,
	.build_count = ARRAY_SIZE(ave_t8142_builds),
	.boot_device_id = 27,
	.boot_arg_value = AVE_COPROC_BOOT_ARG_VALUE,
	.timebase_multiplier = 1,
	.vcbar_mask = U64_MAX,
	.vcbar_flags = AVE_COPROC_VCBAR_VALID,
	.device_type = 27,
	.lfs_ref_scale = 2,
	.text_crc = AVE_FW_TEXT_CRC_T8142_26A428,
	.mbox_idle = 0x118,
	.boot_qualified = true,
};

/*
 * The M3 generation (T8122, T6030, T6031, and both dies of T6032) has the
 * T8112 encoder's register windows, interrupt order and power states, but
 * iBoot preloads a firmware build of its own: on a J615 under macOS 27.2 the
 * ADT segment-ranges give TEXT 0x134000 and DATA 0x128000, which is none of
 * the builds above. Its boot sequence, device numbers and command layout are
 * not known yet, so the encoder is probe-only: the driver checks the
 * bootloader's firmware handoff and reports the build's CRCs, and refuses to
 * start the coprocessor or encode. No segment sizes are pinned, as the M3
 * Pro, Max and Ultra builds have not been seen.
 */
const struct ave_variant ave_m3 = {
	.lfs_ref_scale = 1,
};

/*
 * debugfs: apple-ave-<dev>/last_start_{h264,hevc} hold the last Start built and
 * last_process0_{h264,hevc} the first Process of the last session, so the
 * generated commands can be compared with reference captures.
 */
void ave_debugfs_save(struct ave_dev *ave, enum ave_codec codec,
		      enum ave_image kind, const void *cmd, size_t len)
{
	void *dst = ave->dump[codec - 1][kind];

	if (ave->layout && dst && len == ave->layout->bytes[codec - 1][kind])
		memcpy(dst, cmd, len);
}

/* The last MCTF Start and the first Process of the last MCTF session */
void ave_debugfs_save_mctf(struct ave_dev *ave, enum ave_image kind,
			   const void *cmd, size_t len)
{
	const struct ave_mctf_layout *l = ave_mctf_layout_for_crc(ave->text_crc);
	void *dst = ave->mctf_dump[kind];

	if (l && dst &&
	    len == (kind == AVE_IMAGE_START ? l->start_bytes : l->process_bytes))
		memcpy(dst, cmd, len);
}

static void ave_vfree(void *p)
{
	vfree(p);
}

/*
 * last_mctf_in and last_mctf_out: with mctf_capture set, the raw compressed
 * input surface (as the driver wrote it) and output surface (as the firmware
 * wrote it) of the last delivered frame, a known-plaintext pair.
 */
static void ave_debugfs_mctf_surfaces(struct ave_dev *ave,
				      const struct ave_mctf_layout *l)
{
	static const char *const names[2] = { "last_mctf_in", "last_mctf_out" };

	for (unsigned int k = 0; k < 2; k++) {
		struct debugfs_blob_wrapper *blob;
		void *data = vzalloc(l->form.bytes);

		if (!data || devm_add_action_or_reset(ave->dev, ave_vfree, data))
			continue;
		blob = devm_kzalloc(ave->dev, sizeof(*blob), GFP_KERNEL);
		if (!blob)
			continue;
		ave->mctf_surf_dump[k] = data;
		blob->data = data;
		blob->size = l->form.bytes;
		debugfs_create_blob(names[k], 0400, ave->debugfs, blob);
	}
	debugfs_create_bool("mctf_capture", 0600, ave->debugfs, &ave->mctf_capture);
}

static void ave_debugfs_mctf(struct ave_dev *ave)
{
	static const char *const names[AVE_IMAGE_COUNT] = {
		"last_start_mctf", "last_process0_mctf",
	};
	const struct ave_mctf_layout *l = ave_mctf_qualified(ave->text_crc);

	if (!l)
		return;
	for (unsigned int k = 0; k < AVE_IMAGE_COUNT; k++) {
		struct debugfs_blob_wrapper *blob;
		size_t size = k == AVE_IMAGE_START ? l->start_bytes :
						     l->process_bytes;

		blob = devm_kzalloc(ave->dev, sizeof(*blob), GFP_KERNEL);
		ave->mctf_dump[k] = devm_kzalloc(ave->dev, size, GFP_KERNEL);
		if (!blob || !ave->mctf_dump[k]) {
			ave->mctf_dump[k] = NULL;
			continue;
		}
		blob->data = ave->mctf_dump[k];
		blob->size = size;
		debugfs_create_blob(names[k], 0400, ave->debugfs, blob);
	}
	ave_debugfs_mctf_surfaces(ave, l);
}

static ssize_t ave_lifecycle_write(struct file *file, const char __user *user,
				   size_t count, loff_t *ppos)
{
	struct ave_dev *ave = file->private_data;
	unsigned int codec;
	int ret = kstrtouint_from_user(user, count, 0, &codec);

	if (ret)
		return ret;
	if (codec != AVE_CODEC_H264 && codec != AVE_CODEC_HEVC)
		return -EINVAL;
	ret = ave_fw_lifecycle_test(ave, codec);
	return ret ?: count;
}

static const struct file_operations ave_lifecycle_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = ave_lifecycle_write,
};

/* Root-only software timeout injection; does not send malformed firmware commands. */
static int ave_inject_wedge(void *data, u64 value)
{
	struct ave_dev *ave = data;
	int ret = 0;

	if (value != 1)
		return -EINVAL;
	mutex_lock(&ave->hw_lock);
	if (ave->fw_state != AVE_FW_RUNNING) {
		ret = -EBUSY;
	} else {
		ave->inject_wedge = true;
		ave_fw_mark_broken(ave, -ETIMEDOUT, "debugfs injection");
	}
	mutex_unlock(&ave->hw_lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(ave_inject_wedge_fops, NULL, ave_inject_wedge, "%llu\n");

/*
 * debugfs dart_audit: power the encoder (and through it its DARTs) without
 * starting the coprocessor, run the stream audit and dump the DART
 * registers, all read-only, then let it suspend again.
 */
static int ave_dart_audit_show(struct seq_file *m, void *unused)
{
	struct ave_dev *ave = m->private;
	int ret;

	ret = pm_runtime_resume_and_get(ave->dev);
	if (ret)
		return ret;
	mutex_lock(&ave->hw_lock);
	ret = ave_dart_audit(ave, "debugfs");
	seq_printf(m, "audit of %u DARTs: %s\n", ave->n_darts,
		   !ave->n_darts ? "NONE MAPPED" : ret ? "UNSAFE streams" : "all streams safe");
	ave_dart_dump(ave, m);
	mutex_unlock(&ave->hw_lock);
	pm_runtime_mark_last_busy(ave->dev);
	pm_runtime_put_autosuspend(ave->dev);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ave_dart_audit);

static void ave_debugfs_init(struct ave_dev *ave)
{
	static const char *const names[2][AVE_IMAGE_COUNT] = {
		{ "last_start_h264", "last_process0_h264" },
		{ "last_start_hevc", "last_process0_hevc" },
	};
	char dir_name[64];

	if (!IS_ENABLED(CONFIG_DEBUG_FS) || !ave->layout)
		return;
	snprintf(dir_name, sizeof(dir_name), "apple-ave-%s",
		 dev_name(ave->dev));
	ave->debugfs = debugfs_create_dir(dir_name, NULL);
	if (ave->layout->abi == &ave_abi_6070) {
		debugfs_create_file("lifecycle_test", 0200, ave->debugfs, ave,
				    &ave_lifecycle_fops);
		return;
	}
	debugfs_create_file("inject_wedge", 0200, ave->debugfs, ave,
			    &ave_inject_wedge_fops);
	debugfs_create_file("dart_audit", 0400, ave->debugfs, ave, &ave_dart_audit_fops);
	for (unsigned int c = 0; c < 2; c++)
		for (unsigned int k = 0; k < AVE_IMAGE_COUNT; k++) {
			struct debugfs_blob_wrapper *blob;
			size_t size = ave->layout->bytes[c][k];

			blob = devm_kzalloc(ave->dev, sizeof(*blob),
					    GFP_KERNEL);
			ave->dump[c][k] =
				devm_kzalloc(ave->dev, size, GFP_KERNEL);
			if (!blob || !ave->dump[c][k]) {
				ave->dump[c][k] = NULL;
				continue;
			}
			blob->data = ave->dump[c][k];
			blob->size = ave->layout->bytes[c][k];
			debugfs_create_blob(names[c][k], 0400, ave->debugfs,
					    blob);
		}
	ave_debugfs_mctf(ave);
}

/*
 * The encoder pipes, the DMA block and the motion engines, then optionally
 * "perf": a performance floor the encoder needs to run at full speed, such
 * as the SoC rail floor on the M4 Pro. It is held while the encoder is
 * powered, like the other domains, so it ends with the last session.
 */
static const char *const ave_pd_names[] = { "dma", "pipe4", "pipe5", "me0",
					    "me1", "perf" };

/*
 * Map the encoder's T8110-layout DARTs for the read-only stream audit
 * (ave_dart_audit()). The DART driver owns them; nothing here writes.
 */
static void ave_map_darts(struct ave_dev *ave)
{
	struct device_node *np = ave->dev->of_node;
	int n = of_count_phandle_with_args(np, "iommus", "#iommu-cells");

	for (int i = 0; i < n; i++) {
		struct of_phandle_args args;
		struct resource res;
		void __iomem *r;
		unsigned int d;

		if (of_parse_phandle_with_args(np, "iommus", "#iommu-cells", i, &args))
			break;
		if (!of_device_is_compatible(args.np, "apple,t8110-dart") ||
		    args.args_count != 1 || args.args[0] >= 32 ||
		    of_address_to_resource(args.np, 0, &res)) {
			of_node_put(args.np);
			continue;
		}
		of_node_put(args.np);
		/* A DART that carries several of our streams is mapped once. */
		for (d = 0; d < ave->n_darts; d++)
			if (ave->dart_phys[d] == res.start)
				break;
		if (d < ave->n_darts) {
			ave->dart_sid_mask[d] |= BIT(args.args[0]);
			continue;
		}
		if (ave->n_darts == ARRAY_SIZE(ave->dart_regs))
			continue;
		r = devm_ioremap(ave->dev, res.start, resource_size(&res));
		if (!r)
			continue;
		ave->dart_regs[ave->n_darts] = r;
		ave->dart_phys[ave->n_darts] = res.start;
		ave->dart_sid[ave->n_darts] = args.args[0];
		ave->dart_sid_mask[ave->n_darts] = BIT(args.args[0]);
		ave->n_darts++;
	}
	dev_dbg(ave->dev, "%u DARTs for the stream audit\n", ave->n_darts);
}

static int ave_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	bool perf = of_property_match_string(dev->of_node, "power-domain-names",
					     "perf") >= 0;
	const struct dev_pm_domain_attach_data pd_data = {
		.pd_names = ave_pd_names,
		.num_pd_names = ARRAY_SIZE(ave_pd_names) - !perf,
		.pd_flags = PD_FLAG_DEV_LINK_ON,
	};
	struct ave_dev *ave;
	int ret;

	ave = devm_kzalloc(dev, sizeof(*ave), GFP_KERNEL);
	if (!ave)
		return -ENOMEM;
	ave->dev = dev;
	ave->variant = of_device_get_match_data(dev);
	if (!ave->variant)
		return -ENODEV;
	mutex_init(&ave->hw_lock);
	platform_set_drvdata(pdev, ave);

	ave->codec = devm_platform_ioremap_resource_byname(pdev, "codec");
	if (IS_ERR(ave->codec))
		return PTR_ERR(ave->codec);
	ave->mbox = devm_platform_ioremap_resource_byname(pdev, "mailbox");
	if (IS_ERR(ave->mbox))
		return PTR_ERR(ave->mbox);
	ave->coproc = devm_platform_ioremap_resource_byname(pdev, "coproc");
	if (IS_ERR(ave->coproc))
		return PTR_ERR(ave->coproc);

	ave->domain = iommu_get_domain_for_dev(dev);
	if (!ave->domain || !(ave->domain->type & __IOMMU_DOMAIN_PAGING))
		return dev_err_probe(dev, -ENODEV, "needs a translating IOMMU\n");
	ave_map_darts(ave);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(42));
	if (ret)
		return ret;

	ret = devm_pm_domain_attach_list(dev, &pd_data, &ave->pd_list);
	if (ret < 0)
		return dev_err_probe(dev, ret, "cannot attach power domains\n");

	ret = ave_fw_probe(ave);
	if (ret)
		return ret;

	/*
	 * Domain attachment took active supplier references, but the driver
	 * starts firmware only on first use and begins suspended. Reconcile that
	 * state before enabling runtime PM, otherwise those references survive
	 * until the first session has resumed and suspended the encoder.
	 */
	ret = pm_runtime_set_suspended(dev);
	if (ret)
		goto err_fw;

	pm_runtime_set_autosuspend_delay(dev, AVE_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		goto err_pm;

	ret = ave_v4l2_register(ave);
	if (ret)
		goto err_pm;
	ave_debugfs_init(ave);
	return 0;

err_pm:
	pm_runtime_dont_use_autosuspend(dev);
err_fw:
	ave_fw_remove(ave);
	return ret;
}

static void ave_remove(struct platform_device *pdev)
{
	struct ave_dev *ave = platform_get_drvdata(pdev);

	debugfs_remove_recursive(ave->debugfs);
	ave_v4l2_unregister(ave);

	/*
	 * The coprocessor memory can only go once the firmware never ran or
	 * was reset by a power cycle. A halted coprocessor may keep power
	 * (its domain is shared with the DARTs), so its memory stays.
	 */
	pm_runtime_dont_use_autosuspend(ave->dev);
	mutex_lock(&ave->hw_lock);
	if (ave->fw_state != AVE_FW_OFF) {
		mutex_unlock(&ave->hw_lock);
		dev_warn(ave->dev,
			 "firmware was started, keeping its memory\n");
		return;
	}
	/* OFF also follows a proven power loss; free the pool only if it never ran. */
	if (!ave->fw_ever_ran)
		ave_mctf_pool_free(ave);
	mutex_unlock(&ave->hw_lock);
	ave_fw_remove(ave);
}

static int ave_runtime_suspend(struct device *dev)
{
	struct ave_dev *ave = dev_get_drvdata(dev);
	int ret;

	/*
	 * The IPC, heap and log buffers stay mapped: runtime suspend gives no
	 * reset proof, and the next boot reuses them at the same addresses.
	 */
	mutex_lock(&ave->hw_lock);
	ret = ave_fw_runtime_suspend(ave);
	mutex_unlock(&ave->hw_lock);
	return ret;
}

static int ave_runtime_resume(struct device *dev)
{
	struct ave_dev *ave = dev_get_drvdata(dev);

	mutex_lock(&ave->hw_lock);
	if (ave->layout && ave->layout->abi->transport->scratch_halt &&
	    ave->fw_state == AVE_FW_HALTED)
		ave_fw_lost(ave);
	mutex_unlock(&ave->hw_lock);
	return 0;
}

static int ave_suspend(struct device *dev)
{
	struct ave_dev *ave = dev_get_drvdata(dev);
	bool hung;
	int ret;

	/* Let a running job finish; the m2m core holds back new ones. */
	v4l2_m2m_suspend(ave->m2m_dev);

	/*
	 * The firmware does not survive the power going down. Drain and close
	 * every open session and halt the firmware, as at the end of a stream;
	 * each session reopens in a cold-started firmware on its next frame.
	 */
	mutex_lock(&ave->hw_lock);
	ave_v4l2_suspend(ave);
	hung = ave->fw_state == AVE_FW_BROKEN;
	ret = ave_fw_shutdown(ave);
	/*
	 * A hung 6070 firmware is stopped without a confirmed Halt: its boot
	 * memory stays mapped until the power cycle, like a session's.
	 */
	if (!ret && !hung && ave->layout && ave->layout->abi->transport->scratch_halt)
		ave_fw_release_boot(ave);
	mutex_unlock(&ave->hw_lock);
	if (ret) {
		/*
		 * genpd may gate the encoder and its DARTs after this callback.
		 * A stuck firmware may still DMA, so abort before that happens.
		 * PM does not resume a device whose suspend callback failed.
		 */
		dev_err(dev, "firmware cannot be stopped for sleep: %d\n", ret);
		v4l2_m2m_resume(ave->m2m_dev);
	}
	return ret;
}

static int ave_resume(struct device *dev)
{
	struct ave_dev *ave = dev_get_drvdata(dev);

	/* The next boot checks the coprocessor state itself. */
	ave_runtime_resume(dev);
	v4l2_m2m_resume(ave->m2m_dev);
	return 0;
}

static const struct dev_pm_ops ave_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(ave_suspend, ave_resume)
		RUNTIME_PM_OPS(ave_runtime_suspend, ave_runtime_resume, NULL)
};

static const struct of_device_id ave_of_match[] = {
	{ .compatible = "apple,t6000-ave", .data = &ave_t6000 },
	{ .compatible = "apple,t6030-ave", .data = &ave_m3 },
	{ .compatible = "apple,t6031-ave", .data = &ave_m3 },
	{ .compatible = "apple,t6032-ave", .data = &ave_m3 },
	{ .compatible = "apple,t6041-ave", .data = &ave_t6041 },
	{ .compatible = "apple,t6050-ave", .data = &ave_t6050 },
	{ .compatible = "apple,t8112-ave", .data = &ave_t8112 },
	{ .compatible = "apple,t8122-ave", .data = &ave_t8122 },
	{ .compatible = "apple,t8132-ave", .data = &ave_t8132 },
	{ .compatible = "apple,t8140-ave", .data = &ave_t8140 },
	{ .compatible = "apple,t8142-ave", .data = &ave_t8142 },
	{}
};
MODULE_DEVICE_TABLE(of, ave_of_match);

static struct platform_driver ave_driver = {
	.probe = ave_probe,
	.remove = ave_remove,
	.driver = {
		.name = "apple-ave",
		.of_match_table = ave_of_match,
		.pm = pm_ptr(&ave_pm_ops),
		/* A started coprocessor keeps its memory; do not unbind at will. */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(ave_driver);

MODULE_DESCRIPTION("Apple AVE video encoder");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("DMA_BUF");
