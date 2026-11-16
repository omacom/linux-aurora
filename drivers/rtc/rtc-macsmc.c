// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC RTC driver
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/mfd/macsmc.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

/* 48-bit RTC */
#define RTC_BYTES 6
#define RTC_BITS (8 * RTC_BYTES)

/* 32768 Hz clock */
#define RTC_SEC_SHIFT 15

/* Offset kept as a 64-bit little-endian count of whole seconds */
#define RTC_OFFSET_SECS_BYTES 8

/*
 * PMU alarm control. The alarm compares a 32-bit count of whole seconds of
 * the free-running counter, i.e. CLKM >> RTC_SEC_SHIFT, without the offset.
 */
#define RTC_CTRL_MONITOR	BIT(0)
#define RTC_CTRL_ALARM_EVENT	BIT(1)
#define RTC_CTRL_ALARM_EN	BIT(6)

struct macsmc_rtc {
	struct device *dev;
	struct apple_smc *smc;
	struct rtc_device *rtc_dev;
	struct nvmem_cell *rtc_offset;
	bool offset_in_seconds;
	/* Alarm, if the PMU's alarm registers and interrupt are described */
	struct nvmem_cell *alarm_ctrl;
	struct nvmem_cell *alarm_value;
	struct nvmem_cell *alarm_mask;
	struct mutex alarm_lock;
	u32 alarm_secs;
};

static int macsmc_rtc_get_time_secs(struct macsmc_rtc *rtc, u64 ctr,
				    struct rtc_time *tm)
{
	void *p_off;
	size_t len;
	u64 off;

	p_off = nvmem_cell_read(rtc->rtc_offset, &len);
	if (IS_ERR(p_off))
		return PTR_ERR(p_off);
	if (len < RTC_OFFSET_SECS_BYTES) {
		kfree(p_off);
		return -EIO;
	}

	off = get_unaligned_le64(p_off);
	kfree(p_off);

	/*
	 * Nothing has been stored yet: the counter alone is the time since
	 * the PMU first powered up, not the wall-clock time.
	 */
	if (!off)
		return -EINVAL;

	rtc_time64_to_tm((time64_t)(ctr >> RTC_SEC_SHIFT) + (s64)off, tm);

	return 0;
}

static int macsmc_rtc_get_time(struct device *dev, struct rtc_time *tm)
{
	struct macsmc_rtc *rtc = dev_get_drvdata(dev);
	u64 ctr = 0, off = 0;
	time64_t now;
	void *p_off;
	size_t len;
	int ret;

	ret = apple_smc_read(rtc->smc, SMC_KEY(CLKM), &ctr, RTC_BYTES);
	if (ret < 0)
		return ret;
	if (ret != RTC_BYTES) {
		ctr = 0;
		return -EIO;
	}

	if (rtc->offset_in_seconds)
		return macsmc_rtc_get_time_secs(rtc, ctr, tm);

	p_off = nvmem_cell_read(rtc->rtc_offset, &len);
	if (IS_ERR(p_off))
		return PTR_ERR(p_off);
	if (len < RTC_BYTES) {
		kfree(p_off);
		return -EIO;
	}

	memcpy(&off, p_off, RTC_BYTES);
	kfree(p_off);

	/* Sign extend from 48 to 64 bits, then arithmetic shift right 15 bits to get seconds */
	now = sign_extend64(ctr + off, RTC_BITS - 1) >> RTC_SEC_SHIFT;
	rtc_time64_to_tm(now, tm);

	return ret;
}

static int macsmc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct macsmc_rtc *rtc = dev_get_drvdata(dev);
	u64 ctr = 0, off = 0;
	int ret;

	ret = apple_smc_read(rtc->smc, SMC_KEY(CLKM), &ctr, RTC_BYTES);
	if (ret < 0)
		return ret;
	if (ret != RTC_BYTES) {
		ctr = 0;
		return -EIO;
	}

	if (rtc->offset_in_seconds) {
		__le64 off_secs;

		/*
		 * The offset holds whole seconds, so the counter's fraction of a
		 * second is lost: the clock may run up to a second ahead.
		 */
		off_secs = cpu_to_le64(rtc_tm_to_time64(tm) - (ctr >> RTC_SEC_SHIFT));
		ret = nvmem_cell_write(rtc->rtc_offset, &off_secs, sizeof(off_secs));
		return ret < 0 ? ret : 0;
	}

	/* This sets the offset such that the set second begins now */
	off = (rtc_tm_to_time64(tm) << RTC_SEC_SHIFT) - ctr;
	return nvmem_cell_write(rtc->rtc_offset, &off, RTC_BYTES);
}


static int macsmc_rtc_read_u8(struct nvmem_cell *cell, u8 *val)
{
	size_t len;
	void *p;

	p = nvmem_cell_read(cell, &len);
	if (IS_ERR(p))
		return PTR_ERR(p);
	if (len < 1) {
		kfree(p);
		return -EIO;
	}
	*val = *(u8 *)p;
	kfree(p);

	return 0;
}

static int macsmc_rtc_write_u8(struct nvmem_cell *cell, u8 val)
{
	int ret = nvmem_cell_write(cell, &val, sizeof(val));

	return ret < 0 ? ret : 0;
}

/* The wall-clock offset in whole seconds: wall time = counter seconds + offset */
static int macsmc_rtc_offset_secs(struct macsmc_rtc *rtc, s64 *secs)
{
	u64 off = 0;
	size_t len;
	void *p;

	p = nvmem_cell_read(rtc->rtc_offset, &len);
	if (IS_ERR(p))
		return PTR_ERR(p);

	if (rtc->offset_in_seconds) {
		if (len < RTC_OFFSET_SECS_BYTES) {
			kfree(p);
			return -EIO;
		}
		*secs = (s64)get_unaligned_le64(p);
	} else {
		if (len < RTC_BYTES) {
			kfree(p);
			return -EIO;
		}
		memcpy(&off, p, RTC_BYTES);
		*secs = sign_extend64(off, RTC_BITS - 1) >> RTC_SEC_SHIFT;
	}
	kfree(p);

	return 0;
}

/*
 * Arm the alarm at counter second @secs, or disarm it if @secs is 0. The
 * other bits of the control byte are kept. A latched event is cleared by
 * writing it back, so the event bit is written as 1 only to clear it.
 */
static int macsmc_rtc_program_alarm(struct macsmc_rtc *rtc, u32 secs)
{
	__le32 val = cpu_to_le32(secs);
	u8 ctrl;
	int ret;

	ret = macsmc_rtc_read_u8(rtc->alarm_ctrl, &ctrl);
	if (ret)
		return ret;
	if (ctrl & RTC_CTRL_ALARM_EVENT) {
		ret = macsmc_rtc_write_u8(rtc->alarm_ctrl, ctrl);
		if (ret)
			return ret;
	}
	ctrl &= ~RTC_CTRL_ALARM_EVENT;

	if (!secs) {
		ret = macsmc_rtc_write_u8(rtc->alarm_mask, 1);
		if (ret)
			return ret;
		return macsmc_rtc_write_u8(rtc->alarm_ctrl,
					   (ctrl & ~RTC_CTRL_ALARM_EN) |
					   RTC_CTRL_MONITOR);
	}

	ret = nvmem_cell_write(rtc->alarm_value, &val, sizeof(val));
	if (ret < 0)
		return ret;
	ret = macsmc_rtc_write_u8(rtc->alarm_mask, 0);
	if (ret)
		return ret;
	return macsmc_rtc_write_u8(rtc->alarm_ctrl, ctrl | RTC_CTRL_ALARM_EN |
				   RTC_CTRL_MONITOR);
}

static int macsmc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct macsmc_rtc *rtc = dev_get_drvdata(dev);
	size_t len;
	s64 off;
	u32 secs;
	void *p;
	u8 ctrl;
	int ret;

	ret = macsmc_rtc_offset_secs(rtc, &off);
	if (ret)
		return ret;

	mutex_lock(&rtc->alarm_lock);
	ret = macsmc_rtc_read_u8(rtc->alarm_ctrl, &ctrl);
	if (ret)
		goto out;
	p = nvmem_cell_read(rtc->alarm_value, &len);
	if (IS_ERR(p)) {
		ret = PTR_ERR(p);
		goto out;
	}
	secs = len >= sizeof(u32) ? get_unaligned_le32(p) : 0;
	kfree(p);

	rtc_time64_to_tm((time64_t)secs + off, &alrm->time);
	alrm->enabled = !!(ctrl & RTC_CTRL_ALARM_EN);
	alrm->pending = !!(ctrl & RTC_CTRL_ALARM_EVENT);
out:
	mutex_unlock(&rtc->alarm_lock);
	return ret;
}

static int macsmc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct macsmc_rtc *rtc = dev_get_drvdata(dev);
	s64 off, secs;
	int ret;

	ret = macsmc_rtc_offset_secs(rtc, &off);
	if (ret)
		return ret;

	/*
	 * With a tick offset, off is rounded down to whole seconds but the
	 * time is floor((ctr + off) / 2^15), so the alarm fires at the set
	 * second or up to one second after it.
	 */
	secs = rtc_tm_to_time64(&alrm->time) - off;
	if (secs <= 0 || secs > U32_MAX)
		return -ERANGE;

	mutex_lock(&rtc->alarm_lock);
	rtc->alarm_secs = secs;
	ret = macsmc_rtc_program_alarm(rtc, alrm->enabled ? rtc->alarm_secs : 0);
	mutex_unlock(&rtc->alarm_lock);

	return ret;
}

static int macsmc_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct macsmc_rtc *rtc = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&rtc->alarm_lock);
	ret = macsmc_rtc_program_alarm(rtc, enabled ? rtc->alarm_secs : 0);
	mutex_unlock(&rtc->alarm_lock);

	return ret;
}

static irqreturn_t macsmc_rtc_alarm_irq(int irq, void *data)
{
	struct macsmc_rtc *rtc = data;
	u8 ctrl;

	mutex_lock(&rtc->alarm_lock);
	if (macsmc_rtc_read_u8(rtc->alarm_ctrl, &ctrl) ||
	    !(ctrl & RTC_CTRL_ALARM_EVENT)) {
		mutex_unlock(&rtc->alarm_lock);
		return IRQ_NONE;
	}
	/* One-shot: clear the event and disarm until the alarm is set again */
	macsmc_rtc_program_alarm(rtc, 0);
	mutex_unlock(&rtc->alarm_lock);

	rtc_update_irq(rtc->rtc_dev, 1, RTC_AF | RTC_IRQF);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops macsmc_rtc_ops = {
	.read_time = macsmc_rtc_get_time,
	.set_time = macsmc_rtc_set_time,
	.read_alarm = macsmc_rtc_read_alarm,
	.set_alarm = macsmc_rtc_set_alarm,
	.alarm_irq_enable = macsmc_rtc_alarm_irq_enable,
};

/* A missing cell means no alarm, not an error */
static struct nvmem_cell *macsmc_rtc_cell_optional(struct device *dev,
						   const char *id)
{
	struct nvmem_cell *cell = devm_nvmem_cell_get(dev, id);

	if (IS_ERR(cell) && PTR_ERR(cell) == -ENOENT)
		return NULL;
	return cell;
}

/* The alarm is optional: it needs the PMU's alarm registers and interrupt. */
static int macsmc_rtc_init_alarm(struct platform_device *pdev,
				 struct macsmc_rtc *rtc)
{
	struct device *dev = &pdev->dev;
	int irq, ret;

	rtc->alarm_ctrl = macsmc_rtc_cell_optional(dev, "rtc_ctrl");
	rtc->alarm_value = macsmc_rtc_cell_optional(dev, "rtc_alarm");
	rtc->alarm_mask = macsmc_rtc_cell_optional(dev, "rtc_alarm_mask");
	if (IS_ERR(rtc->alarm_ctrl))
		return PTR_ERR(rtc->alarm_ctrl);
	if (IS_ERR(rtc->alarm_value))
		return PTR_ERR(rtc->alarm_value);
	if (IS_ERR(rtc->alarm_mask))
		return PTR_ERR(rtc->alarm_mask);
	if (!rtc->alarm_ctrl || !rtc->alarm_value || !rtc->alarm_mask)
		return -ENODEV;

	irq = platform_get_irq_optional(pdev, 0);
	if (irq == -EPROBE_DEFER)
		return irq;
	if (irq <= 0)
		return -ENODEV;

	/*
	 * Firmware can leave the alarm armed or its event latched. Neither is
	 * an alarm Linux set, so disarm it before the interrupt is requested.
	 */
	ret = macsmc_rtc_program_alarm(rtc, 0);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, irq, NULL, macsmc_rtc_alarm_irq,
					IRQF_ONESHOT, dev_name(dev), rtc);
	if (ret)
		return ret;

	if (of_property_read_bool(dev->of_node, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (!ret)
			ret = devm_pm_set_wake_irq(dev, irq);
		if (ret)
			dev_warn(dev, "alarm cannot wake the system: %d\n", ret);
	}

	return 0;
}

static int macsmc_rtc_probe(struct platform_device *pdev)
{
	struct apple_smc *smc = dev_get_drvdata(pdev->dev.parent);
	struct macsmc_rtc *rtc;
	int ret;

	/*
	 * MFD will probe this device even without a node in the device tree,
	 * thus bail out early if the SMC on the current machines does not
	 * support RTC and has no node in the device tree.
	 */
	if (!pdev->dev.of_node)
		return -ENODEV;

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->dev = &pdev->dev;
	rtc->smc = smc;
	mutex_init(&rtc->alarm_lock);
	platform_set_drvdata(pdev, rtc);

	rtc->rtc_offset = devm_nvmem_cell_get(&pdev->dev, "rtc_offset");
	if (IS_ERR(rtc->rtc_offset))
		return dev_err_probe(&pdev->dev, PTR_ERR(rtc->rtc_offset),
				     "Failed to get rtc_offset NVMEM cell\n");

	rtc->rtc_dev = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtc->rtc_dev))
		return PTR_ERR(rtc->rtc_dev);

	rtc->offset_in_seconds = of_property_read_bool(pdev->dev.of_node,
						       "apple,rtc-offset-seconds");

	rtc->rtc_dev->ops = &macsmc_rtc_ops;
	ret = macsmc_rtc_init_alarm(pdev, rtc);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret) {
		if (ret != -ENODEV)
			dev_warn(&pdev->dev, "no alarm: %d\n", ret);
		clear_bit(RTC_FEATURE_ALARM, rtc->rtc_dev->features);
	}
	if (rtc->offset_in_seconds) {
		rtc->rtc_dev->range_min = 0;
		rtc->rtc_dev->range_max = RTC_TIMESTAMP_END_9999;
	} else {
		rtc->rtc_dev->range_min = S64_MIN >> (RTC_SEC_SHIFT + (64 - RTC_BITS));
		rtc->rtc_dev->range_max = S64_MAX >> (RTC_SEC_SHIFT + (64 - RTC_BITS));
	}

	return devm_rtc_register_device(rtc->rtc_dev);
}

static const struct of_device_id macsmc_rtc_of_table[] = {
	{ .compatible = "apple,smc-rtc", },
	{}
};
MODULE_DEVICE_TABLE(of, macsmc_rtc_of_table);

static struct platform_driver macsmc_rtc_driver = {
	.driver = {
		.name = "macsmc-rtc",
		.of_match_table = macsmc_rtc_of_table,
	},
	.probe = macsmc_rtc_probe,
};
module_platform_driver(macsmc_rtc_driver);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple SMC RTC driver");
MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
