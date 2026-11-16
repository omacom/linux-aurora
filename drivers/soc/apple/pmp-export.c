// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/device.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/soc/apple/pmp.h>
#include <linux/soc/apple/pmp-report.h>

/* Join removal and serialize the firmware's untagged power reply stream. */
static DEFINE_MUTEX(pmp_lock);
static const void *pmp_data;
static struct device *pmp_device;

int apple_pmp_register(struct device *dev, const void *data)
{
	int ret = 0;

	guard(mutex)(&pmp_lock);
	if (pmp_data)
		ret = -EBUSY;
	else {
		pmp_data = data;
		pmp_device = dev;
	}
	return ret;
}

void apple_pmp_unregister(const void *data)
{
	guard(mutex)(&pmp_lock);
	if (pmp_data == data) {
		pmp_data = NULL;
		pmp_device = NULL;
	}
}

int apple_pmp_link_device(struct device *consumer)
{
	struct device_node *node;
	struct device *supplier = NULL;
	struct device_link *link;

	node = of_parse_phandle(consumer->of_node, "apple,pmp", 0);
	if (!node)
		return -EINVAL;
	scoped_guard(mutex, &pmp_lock) {
		if (pmp_device && pmp_device->of_node == node && device_is_bound(pmp_device))
			supplier = get_device(pmp_device);
	}
	of_node_put(node);
	if (!supplier)
		return -EPROBE_DEFER;
	link = device_link_add(consumer, supplier, DL_FLAG_AUTOREMOVE_CONSUMER);
	put_device(supplier);
	return link ? 0 : -EINVAL;
}
EXPORT_SYMBOL_GPL(apple_pmp_link_device);

int apple_pmp_set_device_power(u8 command, u16 device_id, u32 enabled)
{
	bool report_gfx;
	int ret;

	if ((command != 0x0e && command != 0x0f) || device_id > 15 || enabled > 1)
		return -EINVAL;
	guard(mutex)(&pmp_lock);
	if (!pmp_data)
		return -ENODEV;
	/* The 25G83 PTD readiness is required, with no elapsed-time fallback. */
	ret = apple_pmp_report_wait_supplier_ready(pmp_device, 5000);
	if (ret)
		return ret;
	/* T6030 needs both the RTKit power vote and the PTD bandwidth report. */
	report_gfx = IS_ENABLED(CONFIG_APPLE_T6030_GPU_PMP_LINK) &&
		     command == 0x0f && device_id == 5 &&
		     of_machine_is_compatible("apple,t6030") &&
		     of_device_is_compatible(pmp_device->of_node, "apple,t6030-pmp-v2");
	if (report_gfx) {
		ret = apple_pmp_report_check_t6030_gfx(pmp_device);
		if (ret)
			return ret;
	}
	if (report_gfx && !enabled) {
		ret = apple_pmp_report_set_t6030_gfx(pmp_device, false);
		if (ret) {
			/* Clearing was written before the ACK timed out. Restore it. */
			apple_pmp_report_set_t6030_gfx(pmp_device, true);
			return ret;
		}
	}
	ret = apple_pmp_send_power_command(pmp_data, command, device_id, enabled);
	/* Power is unknown after a lost reply; assert the report conservatively. */
	if (report_gfx && !enabled && ret)
		apple_pmp_report_set_t6030_gfx(pmp_device, true);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_pmp_set_device_power);

/* Reassert after startup, including a report left set by an earlier failure. */
int apple_pmp_t6030_gpu_ready(void)
{
	int ret;

	guard(mutex)(&pmp_lock);
	if (!pmp_data)
		return -ENODEV;
	ret = apple_pmp_report_set_t6030_gfx(pmp_device, false);
	if (ret)
		return ret;
	return apple_pmp_report_set_t6030_gfx(pmp_device, true);
}
EXPORT_SYMBOL_GPL(apple_pmp_t6030_gpu_ready);
