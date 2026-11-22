// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC hwmon driver for Apple Silicon platforms
 *
 * The System Management Controller on Apple Silicon devices is responsible for
 * measuring data from sensors across the SoC and machine. These include power,
 * temperature, voltage and current sensors. Some "sensors" actually expose
 * derived values. An example of this is the key PHPC, which is an estimate
 * of the heat energy being dissipated by the SoC.
 *
 * While each SoC only has one SMC variant, each platform exposes a different
 * set of sensors. For example, M1 MacBooks expose battery telemetry sensors
 * which are not present on the M1 Mac mini. For this reason, the available
 * sensors for a given platform are described in the device tree in a child
 * node of the SMC device. We must walk this list of available sensors and
 * populate the required hwmon data structures at runtime.
 *
 * Originally based on a concept by Jean-Francois Bortolotti <jeff@borto.fr>
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitfield.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/math64.h>
#include <linux/mfd/macsmc.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>

#define MAX_LABEL_LENGTH	32

/* Temperature, voltage, current, power, fan(s) */
#define NUM_SENSOR_TYPES	5

#define FLT_EXP_BIAS	127
#define FLT_EXP_MASK	GENMASK(30, 23)
#define FLT_MANT_BIAS	23
#define FLT_MANT_MASK	GENMASK(22, 0)
#define FLT_SIGN_MASK	BIT(31)

struct macsmc_hwmon_sensor {
	struct apple_smc_key_info info;
	smc_key macsmc_key;
	char label[MAX_LABEL_LENGTH];
	u32 attrs;
	/* Part of the SoC die temperature zone. */
	bool die;
	/* Readings below no_data_below are powered-down placeholders, not temperatures. */
	bool has_no_data_below;
	s32 no_data_below;
};

struct macsmc_hwmon_fan {
	struct macsmc_hwmon_sensor now;
	struct macsmc_hwmon_sensor min;
	struct macsmc_hwmon_sensor max;
	struct macsmc_hwmon_sensor set;
	struct macsmc_hwmon_sensor mode;
	char label[MAX_LABEL_LENGTH];
	u32 attrs;
	bool manual;
	/* The device tree allows a manual speed from userspace. */
	bool manual_control;
	/* Manual for a kernel thermal policy (macsmc_hwmon_fans_request()). */
	bool kernel;
};

struct macsmc_hwmon_sensors {
	struct hwmon_channel_info channel_info;
	struct macsmc_hwmon_sensor *sensors;
	u32 count;
};

/* Fan mode key values */
#define MACSMC_FAN_MODE_AUTO	0
#define MACSMC_FAN_MODE_MANUAL	1
/* The firmware's own fan policy; it refuses a manual mode request. */
#define MACSMC_FAN_MODE_SYSTEM	3

/* How long the firmware gets to leave or re-enter its system mode */
#define MACSMC_FAN_MODE_TIMEOUT_MS	10000
#define MACSMC_FAN_MODE_POLL_MS		100

struct macsmc_hwmon_fans {
	struct hwmon_channel_info channel_info;
	struct macsmc_hwmon_fan *fans;
	u32 count;
	/* Serialises manual fan control and its unlock. */
	struct mutex lock;
	/*
	 * Firmware that keeps its fans in system mode releases them to the
	 * OS only while this key is 1. Zero if the SMC has no such key.
	 */
	smc_key unlock_key;
	bool unlocked;
	/* Fans in manual mode */
	u32 manual;
	/* Suspended, shut down or going away: every fan stays with the SMC. */
	bool stopped;
};

struct macsmc_hwmon {
	struct device *dev;
	struct apple_smc *smc;
	struct device *hwmon_dev;
	struct hwmon_chip_info chip_info;
	/* Chip + sensor types + NULL */
	const struct hwmon_channel_info *channel_infos[1 + NUM_SENSOR_TYPES + 1];
	struct macsmc_hwmon_sensors temp;
	struct macsmc_hwmon_sensors volt;
	struct macsmc_hwmon_sensors curr;
	struct macsmc_hwmon_sensors power;
	struct macsmc_hwmon_fans fan;
	struct macsmc_hwmon_sensor pressure;
	bool register_thermal_zones;
	/* SoC die temperature keys listed, and the ones present. */
	u32 die_listed;
	u32 die_count;
};

static ssize_t thermal_pressure_level_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);
	s32 val;
	int ret;

	ret = apple_smc_read_s32(hwmon->smc, hwmon->pressure.macsmc_key, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(thermal_pressure_level);

static umode_t macsmc_hwmon_extra_is_visible(struct kobject *kobj,
					     struct attribute *attr, int index)
{
	struct device *dev = kobj_to_dev(kobj);
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);

	return hwmon->pressure.macsmc_key ? 0444 : 0;
}

static struct attribute *macsmc_hwmon_extra_attrs[] = {
	&dev_attr_thermal_pressure_level.attr,
	NULL,
};

static const struct attribute_group macsmc_hwmon_extra_group = {
	.attrs = macsmc_hwmon_extra_attrs,
	.is_visible = macsmc_hwmon_extra_is_visible,
};

static const struct attribute_group *macsmc_hwmon_extra_groups[] = {
	&macsmc_hwmon_extra_group,
	NULL,
};

static int macsmc_hwmon_read_label(struct device *dev,
				   enum hwmon_sensor_types type, u32 attr,
				   int channel, const char **str)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		*str = hwmon->temp.sensors[channel].label;
		break;
	case hwmon_in:
		*str = hwmon->volt.sensors[channel].label;
		break;
	case hwmon_curr:
		*str = hwmon->curr.sensors[channel].label;
		break;
	case hwmon_power:
		*str = hwmon->power.sensors[channel].label;
		break;
	case hwmon_fan:
		*str = hwmon->fan.fans[channel].label;
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/*
 * A number of sensors report data in a 48.16 fixed-point decimal format that is
 * not used by any other function of the SMC.
 */
static int macsmc_hwmon_read_ioft_scaled(struct apple_smc *smc, smc_key key,
					 u64 *p, int scale)
{
	u64 val;
	int ret;

	ret = apple_smc_read_u64(smc, key, &val);
	if (ret < 0)
		return ret;

	*p = mul_u64_u32_div(val, scale, 65536);

	return 0;
}

/*
 * Many sensors report their data as IEEE-754 floats. No other SMC function uses
 * them.
 */
static int macsmc_hwmon_read_f32_scaled(struct apple_smc *smc, smc_key key,
					long *p, int scale)
{
	u32 fval;
	u64 val;
	int ret, exp;

	ret = apple_smc_read_u32(smc, key, &fval);
	if (ret < 0)
		return ret;

	exp = FIELD_GET(FLT_EXP_MASK, fval);
	/* NaN and infinity are unavailable readings, not extreme temperatures. */
	if (exp == 0xff)
		return -ENODATA;

	val = fval & FLT_MANT_MASK;
	if (exp)
		val |= BIT(23);
	else
		exp = 1; /* Zero and subnormal values have no implicit leading one. */
	exp -= FLT_EXP_BIAS + FLT_MANT_BIAS;

	/* We never have negatively scaled SMC floats */
	val *= scale;

	if (exp > 63)
		return -ERANGE;
	else if (exp < -63)
		val = 0;
	else if (exp < 0)
		val >>= -exp;
	else if (exp != 0 && (val & ~((1ULL << (64 - exp)) - 1))) /* overflow */
		return -ERANGE;
	else
		val <<= exp;

	if (fval & FLT_SIGN_MASK) {
		if (val > (u64)LONG_MAX + 1)
			return -ERANGE;
		if (val == (u64)LONG_MAX + 1)
			*p = LONG_MIN;
		else
			*p = -(long)val;
	} else {
		if (val > (u64)LONG_MAX)
			return -ERANGE;
		else
			*p = (long)val;
	}

	return 0;
}

/*
 * The SMC has keys of multiple types, denoted by a FourCC of the same format
 * as the key ID. We don't know what data type a key encodes until we poke at it.
 */
static int macsmc_hwmon_read_key(struct apple_smc *smc,
				 struct macsmc_hwmon_sensor *sensor, int scale,
				 long *val)
{
	int ret;

	switch (sensor->info.type_code) {
	/* 32-bit IEEE 754 float */
	case __SMC_KEY('f', 'l', 't', ' '): {
		long flt_ = 0;

		ret = macsmc_hwmon_read_f32_scaled(smc, sensor->macsmc_key,
						   &flt_, scale);
		if (ret)
			return ret;

		*val = flt_;
		break;
	}
	/* 48.16 fixed point decimal */
	case __SMC_KEY('i', 'o', 'f', 't'): {
		u64 ioft = 0;

		ret = macsmc_hwmon_read_ioft_scaled(smc, sensor->macsmc_key,
						    &ioft, scale);
		if (ret)
			return ret;

		if (ioft > LONG_MAX)
			*val = LONG_MAX;
		else
			*val = (long)ioft;
		break;
	}
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int macsmc_hwmon_write_f32(struct apple_smc *smc, smc_key key, long value)
{
	u64 val;
	u32 fval = 0;
	int exp, neg;

	neg = value < 0;
	val = abs(value);

	if (val) {
		exp = __fls(val);

		if (exp > 23)
			val >>= exp - 23;
		else
			val <<= 23 - exp;

		fval = FIELD_PREP(FLT_SIGN_MASK, neg) |
		       FIELD_PREP(FLT_EXP_MASK, exp + FLT_EXP_BIAS) |
		       FIELD_PREP(FLT_MANT_MASK, val & FLT_MANT_MASK);
	}

	return apple_smc_write_u32(smc, key, fval);
}

static int macsmc_hwmon_write_key(struct apple_smc *smc,
				  struct macsmc_hwmon_sensor *sensor, long val)
{
	switch (sensor->info.type_code) {
	/* 32-bit IEEE 754 float */
	case __SMC_KEY('f', 'l', 't', ' '):
		return macsmc_hwmon_write_f32(smc, sensor->macsmc_key, val);
	/* unsigned 8-bit integer */
	case __SMC_KEY('u', 'i', '8', ' '):
		return apple_smc_write_u8(smc, sensor->macsmc_key, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int macsmc_hwmon_read_fan(struct macsmc_hwmon *hwmon, u32 attr, int chan,
				 long *val)
{
	switch (attr) {
	case hwmon_fan_input:
		return macsmc_hwmon_read_key(hwmon->smc,
					     &hwmon->fan.fans[chan].now, 1, val);
	case hwmon_fan_min:
		return macsmc_hwmon_read_key(hwmon->smc,
					     &hwmon->fan.fans[chan].min, 1, val);
	case hwmon_fan_max:
		return macsmc_hwmon_read_key(hwmon->smc,
					     &hwmon->fan.fans[chan].max, 1, val);
	case hwmon_fan_target:
		return macsmc_hwmon_read_key(hwmon->smc,
					     &hwmon->fan.fans[chan].set, 1, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int macsmc_hwmon_fan_read_mode(struct macsmc_hwmon *hwmon,
				      struct macsmc_hwmon_fan *fan, u8 *mode)
{
	return apple_smc_read_u8(hwmon->smc, fan->mode.macsmc_key, mode);
}

/*
 * Hand the fans back to the firmware once no fan is manual: clear the unlock
 * key and wait for every fan to return to system mode. Called with the fan
 * lock held.
 */
static int macsmc_hwmon_fans_relock(struct macsmc_hwmon *hwmon)
{
	unsigned long timeout;
	u32 i;
	u8 mode;
	int ret;

	ret = apple_smc_write_u8(hwmon->smc, hwmon->fan.unlock_key, 0);
	if (ret < 0) {
		dev_err(hwmon->dev, "cannot hand the fans back to the SMC (%d)\n", ret);
		return ret;
	}
	hwmon->fan.unlocked = false;

	timeout = jiffies + msecs_to_jiffies(MACSMC_FAN_MODE_TIMEOUT_MS);
	for (i = 0; i < hwmon->fan.count; i++) {
		struct macsmc_hwmon_fan *fan = &hwmon->fan.fans[i];

		if (!fan->mode.macsmc_key)
			continue;
		for (;;) {
			ret = macsmc_hwmon_fan_read_mode(hwmon, fan, &mode);
			if (ret)
				return ret;
			if (mode == MACSMC_FAN_MODE_SYSTEM)
				break;
			if (time_after(jiffies, timeout)) {
				dev_warn(hwmon->dev, "%s stays in mode %u after release\n",
					 fan->label, mode);
				return -ETIMEDOUT;
			}
			msleep(MACSMC_FAN_MODE_POLL_MS);
		}
	}

	return 0;
}

/* Return a fan to firmware control. Called with the fan lock held. */
static int macsmc_hwmon_fan_auto(struct macsmc_hwmon *hwmon,
				 struct macsmc_hwmon_fan *fan)
{
	int ret, relock;

	if (!fan->manual)
		return 0;

	ret = macsmc_hwmon_write_key(hwmon->smc, &fan->mode, MACSMC_FAN_MODE_AUTO);
	fan->manual = false;
	fan->kernel = false;
	hwmon->fan.manual--;

	if (hwmon->fan.unlocked && !hwmon->fan.manual) {
		relock = macsmc_hwmon_fans_relock(hwmon);
		if (relock)
			return relock;
	}

	return ret < 0 ? ret : 0;
}

/*
 * Put a fan in manual mode. Firmware in system mode refuses that until the
 * unlock key is set; it then leaves system mode and accepts the request.
 * Called with the fan lock held.
 */
static int macsmc_hwmon_fan_manual(struct macsmc_hwmon *hwmon,
				   struct macsmc_hwmon_fan *fan)
{
	unsigned long timeout;
	u8 mode;
	int ret;

	if (fan->manual)
		return 0;

	/* Older firmware may expose a write-only mode key. */
	if (!(fan->mode.info.flags & APPLE_SMC_READABLE))
		goto set_manual;

	/* Do not send a mode write the firmware is known to refuse. */
	ret = macsmc_hwmon_fan_read_mode(hwmon, fan, &mode);
	if (ret)
		return ret;

	if (mode == MACSMC_FAN_MODE_SYSTEM) {
		if (!hwmon->fan.unlock_key)
			return -EOPNOTSUPP;

		if (!hwmon->fan.unlocked) {
			ret = apple_smc_write_u8(hwmon->smc, hwmon->fan.unlock_key, 1);
			if (ret < 0) {
				/* The key may have been set anyway; clear it. */
				apple_smc_write_u8(hwmon->smc, hwmon->fan.unlock_key, 0);
				return ret;
			}
			hwmon->fan.unlocked = true;
		}

		timeout = jiffies + msecs_to_jiffies(MACSMC_FAN_MODE_TIMEOUT_MS);
		for (;;) {
			ret = macsmc_hwmon_fan_read_mode(hwmon, fan, &mode);
			if (ret)
				goto relock;
			if (mode != MACSMC_FAN_MODE_SYSTEM) {
				/* Leaving system mode need not mean writes are ready. */
				ret = macsmc_hwmon_write_key(hwmon->smc, &fan->mode,
							     MACSMC_FAN_MODE_MANUAL);
				if (ret >= 0)
					goto manual;
				if (ret != -EIO && ret != -EBUSY)
					goto relock;
			}
			if (time_after(jiffies, timeout)) {
				if (mode == MACSMC_FAN_MODE_SYSTEM)
					ret = -ETIMEDOUT;
				goto relock;
			}
			msleep(MACSMC_FAN_MODE_POLL_MS);
		}
	}

set_manual:
	ret = macsmc_hwmon_write_key(hwmon->smc, &fan->mode, MACSMC_FAN_MODE_MANUAL);
	if (ret < 0)
		goto relock;

manual:
	fan->manual = true;
	hwmon->fan.manual++;
	return 0;

relock:
	if (hwmon->fan.unlocked && !hwmon->fan.manual)
		macsmc_hwmon_fans_relock(hwmon);
	return ret;
}

/*
 * Whether userspace may set this fan's speed: only fans whose device tree
 * node has apple,fan-manual-control. On other fans fanX_target is read-only.
 */
static bool macsmc_hwmon_fan_user_control(const struct macsmc_hwmon_fan *fan)
{
	return fan->mode.macsmc_key && fan->manual_control;
}

static int macsmc_hwmon_set_fan(struct macsmc_hwmon *hwmon, int channel, long val)
{
	struct macsmc_hwmon_fan *fan = &hwmon->fan.fans[channel];
	long min, max;
	int ret;

	if (!macsmc_hwmon_fan_user_control(fan))
		return -EOPNOTSUPP;
	if (val < 0)
		return -EINVAL;

	guard(mutex)(&hwmon->fan.lock);

	if (!val)
		return macsmc_hwmon_fan_auto(hwmon, fan);
	if (hwmon->fan.stopped)
		return -EBUSY;

	/* The SMC does no sanity checks on requested fan speeds. */
	ret = macsmc_hwmon_read_key(hwmon->smc, &fan->min, 1, &min);
	if (!ret)
		ret = macsmc_hwmon_read_key(hwmon->smc, &fan->max, 1, &max);
	if (!ret && (min <= 0 || max < min))
		ret = -EIO;
	if (!ret)
		ret = macsmc_hwmon_fan_manual(hwmon, fan);
	if (!ret) {
		/* Userspace takes the fan over from a kernel policy. */
		fan->kernel = false;
		ret = macsmc_hwmon_write_key(hwmon->smc, &fan->set, clamp(val, min, max));
		if (ret >= 0)
			return 0;
	}
	macsmc_hwmon_fan_auto(hwmon, fan);

	return ret;
}

/**
 * macsmc_hwmon_fans_request() - set the fans for a kernel thermal policy
 * @dev: the bound macsmc-hwmon device; the caller holds its device lock
 * @permille: each fan's target as a share of its minimum-to-maximum range;
 *	0 hands the fans this interface set back to the SMC
 * @min_step_rpm: a fan already set by this interface whose target is closer
 *	than this to the new one keeps its target
 *
 * The fans go through the same manual mode as a userspace request, so the
 * mode key has one owner: fans userspace holds are left alone, and every fan
 * returns to the SMC at suspend, shutdown and unbind. Only fans whose device
 * tree node names their mode, target, minimum and maximum keys take part.
 *
 * Return: the first such fan's target in rpm (0 once handed back), -ENODEV
 * without such a fan, -EBUSY while stopped, or the first error.
 */
int macsmc_hwmon_fans_request(struct device *dev, unsigned int permille,
			      unsigned int min_step_rpm)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);
	long first = -ENODEV;
	int ret = 0;
	u32 i;

	if (!hwmon)
		return -ENODEV;
	permille = min(permille, 1000U);

	guard(mutex)(&hwmon->fan.lock);

	if (permille && hwmon->fan.stopped)
		return -EBUSY;

	for (i = 0; i < hwmon->fan.count; i++) {
		struct macsmc_hwmon_fan *fan = &hwmon->fan.fans[i];
		long min, max, now, target;
		int err;

		if (!fan->mode.macsmc_key ||
		    (fan->attrs & (HWMON_F_TARGET | HWMON_F_MIN | HWMON_F_MAX)) !=
		    (HWMON_F_TARGET | HWMON_F_MIN | HWMON_F_MAX))
			continue;
		if (fan->manual && !fan->kernel)
			continue;

		if (!permille) {
			if (first < 0)
				first = 0;
			err = macsmc_hwmon_fan_auto(hwmon, fan);
			if (err && !ret)
				ret = err;
			continue;
		}

		err = macsmc_hwmon_read_key(hwmon->smc, &fan->min, 1, &min);
		if (!err)
			err = macsmc_hwmon_read_key(hwmon->smc, &fan->max, 1, &max);
		if (!err && (min <= 0 || max < min))
			err = -EIO;
		if (err) {
			dev_warn_ratelimited(hwmon->dev, "%s: no fan range for a kernel request (%d)\n",
					     fan->label, err);
			if (!ret)
				ret = err;
			continue;
		}
		target = min + (max - min) * permille / 1000;
		if (first < 0)
			first = target;

		if (fan->kernel &&
		    !macsmc_hwmon_read_key(hwmon->smc, &fan->set, 1, &now) &&
		    abs(now - target) < (long)min_step_rpm)
			continue;

		err = macsmc_hwmon_fan_manual(hwmon, fan);
		if (!err) {
			fan->kernel = true;
			err = macsmc_hwmon_write_key(hwmon->smc, &fan->set, target);
			if (err > 0)
				err = 0;
		}
		if (err) {
			/* -EOPNOTSUPP: the fan is in system mode and this SMC has no unlock key. */
			dev_warn_ratelimited(hwmon->dev, "%s: kernel fan request refused (%d)\n",
					     fan->label, err);
			macsmc_hwmon_fan_auto(hwmon, fan);
			if (!ret)
				ret = err;
		}
	}

	return ret ? ret : first;
}
EXPORT_SYMBOL_GPL(macsmc_hwmon_fans_request);

static int macsmc_hwmon_write_fan(struct device *dev, u32 attr, int channel,
				  long val)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);
	int ret;

	/*
	 * Unlocking or releasing system fan control polls the SMC for up to
	 * MACSMC_FAN_MODE_TIMEOUT_MS. Drop the chip-wide hwmon lock so thermal
	 * zones and sensor reads are not stalled while waiting on the SMC;
	 * hwmon->fan.lock and smc->mutex serialize fan state and SMC accesses.
	 */
	hwmon_unlock(dev);
	ret = macsmc_hwmon_set_fan(hwmon, channel, val);
	hwmon_lock(dev);

	return ret;
}

/* Hand every manual fan back to the firmware. */
static void macsmc_hwmon_fans_restore(struct macsmc_hwmon *hwmon)
{
	u32 i;

	guard(mutex)(&hwmon->fan.lock);
	for (i = 0; i < hwmon->fan.count; i++)
		macsmc_hwmon_fan_auto(hwmon, &hwmon->fan.fans[i]);
}

static void macsmc_hwmon_fans_restore_action(void *data)
{
	struct macsmc_hwmon *hwmon = data;

	scoped_guard(mutex, &hwmon->fan.lock)
		hwmon->fan.stopped = true;
	macsmc_hwmon_fans_restore(hwmon);
}

static int macsmc_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			     u32 attr, int channel, long *val)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);
	int ret = 0;

	switch (type) {
	case hwmon_temp: {
		struct macsmc_hwmon_sensor *sensor = &hwmon->temp.sensors[channel];

		ret = macsmc_hwmon_read_key(hwmon->smc, sensor, 1000, val);
		/* A powered-down sensor's placeholder is not a temperature. */
		if (!ret && sensor->has_no_data_below && *val < sensor->no_data_below)
			ret = -ENODATA;
		break;
	}
	case hwmon_in:
		ret = macsmc_hwmon_read_key(hwmon->smc,
					    &hwmon->volt.sensors[channel], 1000, val);
		break;
	case hwmon_curr:
		ret = macsmc_hwmon_read_key(hwmon->smc,
					    &hwmon->curr.sensors[channel], 1000, val);
		break;
	case hwmon_power:
		/* SMC returns power in Watts with acceptable precision to scale to uW */
		ret = macsmc_hwmon_read_key(hwmon->smc,
					    &hwmon->power.sensors[channel],
					    1000000, val);
		break;
	case hwmon_fan:
		ret = macsmc_hwmon_read_fan(hwmon, attr, channel, val);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ret;
}

static int macsmc_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_fan:
		return macsmc_hwmon_write_fan(dev, attr, channel, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t macsmc_hwmon_fan_is_visible(const struct macsmc_hwmon_fan *fan,
					   u32 attr)
{
	if (fan->attrs & BIT(attr)) {
		if (attr == hwmon_fan_target && macsmc_hwmon_fan_user_control(fan))
			return 0644;

		return 0444;
	}

	return 0;
}

static umode_t macsmc_hwmon_is_visible(const void *data,
				       enum hwmon_sensor_types type, u32 attr,
				       int channel)
{
	const struct macsmc_hwmon *hwmon = data;
	struct macsmc_hwmon_sensor *sensor;

	switch (type) {
	case hwmon_in:
		sensor = &hwmon->volt.sensors[channel];
		break;
	case hwmon_curr:
		sensor = &hwmon->curr.sensors[channel];
		break;
	case hwmon_power:
		sensor = &hwmon->power.sensors[channel];
		break;
	case hwmon_temp:
		sensor = &hwmon->temp.sensors[channel];
		break;
	case hwmon_fan:
		return macsmc_hwmon_fan_is_visible(&hwmon->fan.fans[channel], attr);
	default:
		return 0;
	}

	/* A sensor without a key could not be set up; see macsmc_hwmon_add_temp_channel(). */
	if (!sensor->macsmc_key)
		return 0;

	/* Sensors only register ro attributes */
	if (sensor->attrs & BIT(attr))
		return 0444;

	return 0;
}

static const struct hwmon_ops macsmc_hwmon_ops = {
	.is_visible = macsmc_hwmon_is_visible,
	.read = macsmc_hwmon_read,
	.read_string = macsmc_hwmon_read_label,
	.write = macsmc_hwmon_write,
};

/*
 * Get the key metadata, including key data type, from the SMC.
 */
static int macsmc_hwmon_parse_key(struct device *dev, struct apple_smc *smc,
				  struct macsmc_hwmon_sensor *sensor,
				  const char *key)
{
	int ret;

	ret = apple_smc_get_key_info(smc, _SMC_KEY(key), &sensor->info);
	if (ret) {
		dev_dbg(dev, "Failed to retrieve key info for %s\n", key);
		return ret;
	}

	sensor->macsmc_key = _SMC_KEY(key);

	return 0;
}

/*
 * A sensor is a single key-value pair as made available by the SMC.
 * The devicetree gives us the SMC key ID and a friendly name where the
 * purpose of the sensor is known.
 */
static int macsmc_hwmon_create_sensor(struct device *dev, struct apple_smc *smc,
				      struct device_node *sensor_node,
				      struct macsmc_hwmon_sensor *sensor)
{
	const char *key, *label;
	int ret;

	ret = of_property_read_string(sensor_node, "apple,key-id", &key);
	if (ret) {
		dev_dbg(dev, "Could not find apple,key-id in sensor node\n");
		return ret;
	}

	ret = macsmc_hwmon_parse_key(dev, smc, sensor, key);
	if (ret)
		return ret;

	/*
	 * macsmc_hwmon_read_key() decodes only these two types, so a key of any
	 * other type, or one the SMC does not let us read, could only ever
	 * return an error: do not set up a channel for it.
	 */
	if (!(sensor->info.flags & APPLE_SMC_READABLE))
		return -EACCES;

	switch (sensor->info.type_code) {
	case __SMC_KEY('f', 'l', 't', ' '):
		if (sensor->info.size != sizeof(u32))
			return -EINVAL;
		break;
	case __SMC_KEY('i', 'o', 'f', 't'):
		if (sensor->info.size != sizeof(u64))
			return -EINVAL;
		break;
	default:
		return -EOPNOTSUPP;
	}

	ret = of_property_read_string(sensor_node, "label", &label);
	if (ret)
		dev_dbg(dev, "No label found for sensor %s\n", key);
	else
		strscpy_pad(sensor->label, label, sizeof(sensor->label));

	sensor->has_no_data_below =
		!of_property_read_s32(sensor_node, "apple,no-data-below-millicelsius",
				      &sensor->no_data_below);

	return 0;
}

/*
 * A SoC die temperature sensor: a readable float temperature key, labelled
 * "SoC die <key>".
 */
/* The temperature channel set up so far for @key, if any. */
static struct macsmc_hwmon_sensor *macsmc_hwmon_find_temp(struct macsmc_hwmon *hwmon,
							  smc_key key)
{
	u32 i;

	for (i = 0; i < hwmon->temp.count; i++)
		if (hwmon->temp.sensors[i].macsmc_key == key)
			return &hwmon->temp.sensors[i];
	return NULL;
}

static int macsmc_hwmon_create_die_sensor(struct macsmc_hwmon *hwmon, const char *key,
					  struct macsmc_hwmon_sensor *sensor)
{
	int ret;

	ret = macsmc_hwmon_parse_key(hwmon->dev, hwmon->smc, sensor, key);
	if (ret)
		return ret;
	if (!(sensor->info.flags & APPLE_SMC_READABLE) ||
	    sensor->info.type_code != __SMC_KEY('f', 'l', 't', ' ') ||
	    sensor->info.size != sizeof(u32)) {
		dev_info(hwmon->dev, "SoC die key %s is not a readable float; ignored\n", key);
		return -EINVAL;
	}
	snprintf(sensor->label, sizeof(sensor->label), "SoC die %s", key);
	sensor->attrs = HWMON_T_INPUT | HWMON_T_LABEL;
	sensor->die = true;
	return 0;
}

static int macsmc_hwmon_create_pressure(struct macsmc_hwmon *hwmon,
					struct device_node *hwmon_node)
{
	struct macsmc_hwmon_sensor pressure = {};
	const char *key;
	s32 val;
	int ret;

	ret = of_property_read_string(hwmon_node, "apple,thermal-pressure-key",
				      &key);
	if (ret == -EINVAL)
		return 0;
	if (ret)
		return ret;

	ret = macsmc_hwmon_parse_key(hwmon->dev, hwmon->smc, &pressure, key);
	if (ret)
		goto invalid;

	if (pressure.info.size != sizeof(s32) ||
	    pressure.info.type_code != __SMC_KEY('s', 'i', '3', '2') ||
	    !(pressure.info.flags & APPLE_SMC_READABLE)) {
		ret = -EINVAL;
		goto invalid;
	}

	ret = apple_smc_read_s32(hwmon->smc, pressure.macsmc_key, &val);
	if (ret)
		goto invalid;

	hwmon->pressure = pressure;
	return 0;

invalid:
	dev_warn(hwmon->dev, "Ignoring invalid thermal pressure key %s (%d)\n",
		 key, ret);
	return 0;
}

/*
 * Account for the temperature sensor just set up at the end of @temp.
 *
 * Thermal zones refer to a temperature channel by the index of its node
 * among the temperature-* nodes, so every node takes a channel. A sensor
 * that could not be set up keeps its channel with no key, which hides it:
 * hwmon creates no attributes and no thermal sensor for it, and only the
 * zones that use it fail to register, instead of every later channel
 * moving onto the wrong sensor.
 */
static void macsmc_hwmon_add_temp_channel(struct macsmc_hwmon_sensors *temp,
					  int create_ret)
{
	struct macsmc_hwmon_sensor *sensor = &temp->sensors[temp->count++];

	if (create_ret)
		memset(sensor, 0, sizeof(*sensor));

	/* A zero entry would end the list of channels here. */
	sensor->attrs = HWMON_T_INPUT;
	if (*sensor->label)
		sensor->attrs |= HWMON_T_LABEL;
}

/* Discover reporting-only temperatures without a board-specific key table. */
static int macsmc_hwmon_discover_temperatures(struct macsmc_hwmon *hwmon)
{
	struct apple_smc *smc = hwmon->smc;
	struct macsmc_hwmon_sensor *sensor;
	struct apple_smc_key_info info;
	smc_key key, previous = 0;
	u32 i;
	int ret;

	if (!of_property_read_bool(hwmon->dev->of_node, "apple,discover-temperatures"))
		return 0;

	/* Dynamic channel numbers must not replace DT-described channels. */
	if (hwmon->temp.count)
		return -EINVAL;

	/* Bound both allocation and mailbox transactions for invalid firmware data. */
	if (!smc->key_count || smc->key_count > U16_MAX)
		return -EINVAL;

	ret = of_property_count_strings(hwmon->dev->of_node, "apple,soc-die-keys");
	hwmon->die_listed = ret > 0 ? ret : 0;

	hwmon->temp.sensors = devm_kcalloc(hwmon->dev, smc->key_count,
					   sizeof(*sensor), GFP_KERNEL);
	if (!hwmon->temp.sensors)
		return -ENOMEM;

	for (i = 0; i < smc->key_count; i++) {
		ret = apple_smc_get_key_by_index(smc, i, &key);
		if (ret < 0)
			return ret;
		/* SMC key tables are strictly ordered; reject duplicates as well. */
		if (i && key <= previous)
			return -EINVAL;
		previous = key;
		if ((key >> 24) < 'T')
			continue;
		if ((key >> 24) > 'T')
			break;

		ret = apple_smc_get_key_info(smc, key, &info);
		if (ret < 0)
			return ret;
		if (!(info.flags & APPLE_SMC_READABLE) ||
		    (info.flags & APPLE_SMC_FUNCTION) ||
		    info.type_code != __SMC_KEY('f', 'l', 't', ' ') || info.size != 4)
			continue;

		sensor = &hwmon->temp.sensors[hwmon->temp.count++];
		sensor->macsmc_key = key;
		sensor->info = info;
		sensor->attrs = HWMON_T_INPUT | HWMON_T_LABEL;
		/* Key identity only: do not invent locations or calibration. */
		snprintf(sensor->label, sizeof(sensor->label), "%c%c%c%c",
			 key >> 24, (key >> 16) & 0xff, (key >> 8) & 0xff, key & 0xff);
		/* A listed SoC die key keeps its discovered channel and joins the die zone. */
		if (of_property_match_string(hwmon->dev->of_node, "apple,soc-die-keys",
					     sensor->label) >= 0) {
			sensor->die = true;
			hwmon->die_count++;
		}
	}

	return 0;
}

/*
 * Fan data is exposed by the SMC as multiple sensors.
 *
 * The devicetree schema reuses apple,key-id for the actual fan speed sensor.
 * Min, max and target keys do not need labels, so we can reuse label
 * for naming the entire fan.
 */
static int macsmc_hwmon_create_fan(struct device *dev, struct apple_smc *smc,
				   struct device_node *fan_node,
				   struct macsmc_hwmon_fan *fan)
{
	const char *label, *now, *min, *max, *set, *mode;
	char missing[64] = "";
	int len = 0;
	int ret;

	ret = of_property_read_string(fan_node, "apple,key-id", &now);
	if (ret) {
		dev_err(dev, "apple,key-id not found in fan node!\n");
		return ret;
	}

	ret = macsmc_hwmon_parse_key(dev, smc, &fan->now, now);
	if (ret)
		return ret;

	fan->attrs = HWMON_F_INPUT;

	ret = of_property_read_string(fan_node, "label", &label);
	if (ret) {
		dev_dbg(dev, "No label found for fan %s\n", now);
	} else {
		strscpy_pad(fan->label, label, sizeof(fan->label));
		fan->attrs |= HWMON_F_LABEL;
	}

	/*
	 * The following keys are not required to simply monitor fan speed.
	 * A key the firmware lacks (the T6050 SMC spells the mode key F0md,
	 * not F0Md) drops only that attribute, not the whole fan; the missing
	 * keys are reported together below.
	 */
	if (!of_property_read_string(fan_node, "apple,fan-minimum", &min)) {
		if (macsmc_hwmon_parse_key(dev, smc, &fan->min, min))
			len += scnprintf(missing + len, sizeof(missing) - len,
					 " minimum %s", min);
		else
			fan->attrs |= HWMON_F_MIN;
	}

	if (!of_property_read_string(fan_node, "apple,fan-maximum", &max)) {
		if (macsmc_hwmon_parse_key(dev, smc, &fan->max, max))
			len += scnprintf(missing + len, sizeof(missing) - len,
					 " maximum %s", max);
		else
			fan->attrs |= HWMON_F_MAX;
	}

	if (!of_property_read_string(fan_node, "apple,fan-target", &set)) {
		if (macsmc_hwmon_parse_key(dev, smc, &fan->set, set))
			len += scnprintf(missing + len, sizeof(missing) - len,
					 " target %s", set);
		else
			fan->attrs |= HWMON_F_TARGET;
	}

	if (!of_property_read_string(fan_node, "apple,fan-mode", &mode)) {
		if (macsmc_hwmon_parse_key(dev, smc, &fan->mode, mode))
			len += scnprintf(missing + len, sizeof(missing) - len,
					 " mode %s", mode);
	}

	if (len)
		dev_info(dev, "Fan %s: optional keys not found:%s\n", now, missing);

	/* Manual control needs the minimum, maximum and target keys too */
	if ((fan->attrs & (HWMON_F_MIN | HWMON_F_MAX | HWMON_F_TARGET)) !=
	    (HWMON_F_MIN | HWMON_F_MAX | HWMON_F_TARGET))
		fan->mode.macsmc_key = 0;

	if (of_property_read_bool(fan_node, "apple,fan-manual-control")) {
		if (fan->mode.macsmc_key && (fan->attrs & HWMON_F_TARGET))
			fan->manual_control = true;
		else
			dev_warn(dev, "%s: apple,fan-manual-control needs apple,fan-target and apple,fan-mode\n",
				 fan->label);
	}

	/* Initialise fan control mode to automatic */
	fan->manual = false;

	return 0;
}

static int macsmc_hwmon_populate_sensors(struct macsmc_hwmon *hwmon,
					 struct device_node *hwmon_node)
{
	struct device_node *key_node __maybe_unused;
	struct macsmc_hwmon_sensor *sensor;
	u32 n_current = 0, n_fan = 0, n_power = 0, n_temperature = 0, n_voltage = 0;
	const char *key;
	bool indexed;
	int n_die_keys, i, ret;

	n_die_keys = of_property_count_strings(hwmon_node, "apple,soc-die-keys");
	if (n_die_keys == -EINVAL)
		n_die_keys = 0;
	else if (n_die_keys < 0)
		return n_die_keys;

	for_each_child_of_node_with_prefix(hwmon_node, key_node, "current-") {
		n_current++;
	}

	if (n_current) {
		hwmon->curr.sensors = devm_kcalloc(hwmon->dev, n_current,
						   sizeof(struct macsmc_hwmon_sensor), GFP_KERNEL);
		if (!hwmon->curr.sensors)
			return -ENOMEM;

		for_each_child_of_node_with_prefix(hwmon_node, key_node, "current-") {
			sensor = &hwmon->curr.sensors[hwmon->curr.count];
			if (!macsmc_hwmon_create_sensor(hwmon->dev, hwmon->smc, key_node, sensor)) {
				sensor->attrs = HWMON_C_INPUT;

				if (*sensor->label)
					sensor->attrs |= HWMON_C_LABEL;

				hwmon->curr.count++;
			}
		}
	}

	for_each_child_of_node_with_prefix(hwmon_node, key_node, "fan-") {
		n_fan++;
	}

	if (n_fan) {
		hwmon->fan.fans = devm_kcalloc(hwmon->dev, n_fan,
					       sizeof(struct macsmc_hwmon_fan), GFP_KERNEL);
		if (!hwmon->fan.fans)
			return -ENOMEM;

		n_fan = 0;
		for_each_child_of_node_with_prefix(hwmon_node, key_node, "fan-") {
			struct macsmc_hwmon_fan *fan = &hwmon->fan.fans[hwmon->fan.count];

			if (!macsmc_hwmon_create_fan(hwmon->dev, hwmon->smc, key_node, fan))
				hwmon->fan.count++;
		}
	}

	for_each_child_of_node_with_prefix(hwmon_node, key_node, "power-") {
		n_power++;
	}

	if (n_power) {
		hwmon->power.sensors = devm_kcalloc(hwmon->dev, n_power,
						    sizeof(struct macsmc_hwmon_sensor), GFP_KERNEL);
		if (!hwmon->power.sensors)
			return -ENOMEM;

		for_each_child_of_node_with_prefix(hwmon_node, key_node, "power-") {
			sensor = &hwmon->power.sensors[hwmon->power.count];
			if (!macsmc_hwmon_create_sensor(hwmon->dev, hwmon->smc, key_node, sensor)) {
				sensor->attrs = HWMON_P_INPUT;

				if (*sensor->label)
					sensor->attrs |= HWMON_P_LABEL;

				hwmon->power.count++;
			}
		}
	}

	for_each_child_of_node_with_prefix(hwmon_node, key_node, "temperature-") {
		n_temperature++;
	}
	/* With discovery, listed die keys mark discovered channels instead (see there). */
	if (!of_property_read_bool(hwmon_node, "apple,discover-temperatures"))
		n_temperature += n_die_keys;
	else
		n_die_keys = 0;

	if (n_temperature) {
		hwmon->temp.sensors = devm_kcalloc(hwmon->dev, n_temperature,
						   sizeof(struct macsmc_hwmon_sensor), GFP_KERNEL);
		if (!hwmon->temp.sensors)
			return -ENOMEM;

		/*
		 * Only a node that is a thermal sensor provider (T8140) has zones
		 * referring to its temperature channels by index: there a node
		 * that fails to set up keeps its channel. Elsewhere it is skipped.
		 */
		indexed = of_property_present(hwmon_node, "#thermal-sensor-cells");
		for_each_child_of_node_with_prefix(hwmon_node, key_node, "temperature-") {
			sensor = &hwmon->temp.sensors[hwmon->temp.count];
			ret = macsmc_hwmon_create_sensor(hwmon->dev, hwmon->smc,
							 key_node, sensor);
			if (ret && !indexed)
				continue;
			if (ret)
				dev_warn(hwmon->dev, "%pOFn unavailable (%d), temp%u hidden\n",
					 key_node, ret, hwmon->temp.count + 1);
			macsmc_hwmon_add_temp_channel(&hwmon->temp, ret);
		}

		for (i = 0; i < n_die_keys; i++) {
			ret = of_property_read_string_index(hwmon_node, "apple,soc-die-keys",
							    i, &key);
			if (ret)
				return ret;
			if (strlen(key) != 4) {
				dev_warn(hwmon->dev, "Ignoring invalid SoC die key %s\n", key);
				continue;
			}
			hwmon->die_listed++;
			/*
			 * A key whose temperature node has a no-data threshold
			 * joins the zone through that channel instead of a
			 * second channel without the threshold.
			 */
			sensor = macsmc_hwmon_find_temp(hwmon, _SMC_KEY(key));
			if (sensor && sensor->has_no_data_below) {
				if (!sensor->die) {
					sensor->die = true;
					hwmon->die_count++;
				}
				continue;
			}
			sensor = &hwmon->temp.sensors[hwmon->temp.count];
			if (!macsmc_hwmon_create_die_sensor(hwmon, key, sensor)) {
				hwmon->die_count++;
				hwmon->temp.count++;
			}
		}
	}

	for_each_child_of_node_with_prefix(hwmon_node, key_node, "voltage-") {
		n_voltage++;
	}

	if (n_voltage) {
		hwmon->volt.sensors = devm_kcalloc(hwmon->dev, n_voltage,
						   sizeof(struct macsmc_hwmon_sensor), GFP_KERNEL);
		if (!hwmon->volt.sensors)
			return -ENOMEM;

		for_each_child_of_node_with_prefix(hwmon_node, key_node, "voltage-") {
			sensor = &hwmon->volt.sensors[hwmon->volt.count];
			if (!macsmc_hwmon_create_sensor(hwmon->dev, hwmon->smc, key_node, sensor)) {
				sensor->attrs = HWMON_I_INPUT;

				if (*sensor->label)
					sensor->attrs |= HWMON_I_LABEL;

				hwmon->volt.count++;
			}
		}
	}

	return 0;
}

/* Create NULL-terminated config arrays */
static void macsmc_hwmon_populate_configs(u32 *configs, const struct macsmc_hwmon_sensors *sensors)
{
	int idx;

	for (idx = 0; idx < sensors->count; idx++)
		configs[idx] = sensors->sensors[idx].attrs;
}

static void macsmc_hwmon_populate_fan_configs(u32 *configs, const struct macsmc_hwmon_fans *fans)
{
	int idx;

	for (idx = 0; idx < fans->count; idx++)
		configs[idx] = fans->fans[idx].attrs;
}

/*
 * The SoC die temperature zone: the hottest of the SoC die temperature
 * sensors, for in-kernel users that limit SoC power by temperature (they look
 * the zone up by name). It has no trip points and is not polled; each read
 * queries every SoC die sensor. A read fails if any of them cannot be read or
 * reads outside -40..150 degrees Celsius, so users never act on a partial or
 * implausible view. A sensor below its no-data threshold is powered down and
 * left out.
 */
#if IS_ENABLED(CONFIG_THERMAL)
#define MACSMC_SOC_DIE_ZONE	"macsmc_soc_die"
#define MACSMC_DIE_MIN_MC	-40000
#define MACSMC_DIE_MAX_MC	150000

static int macsmc_hwmon_die_hottest(struct macsmc_hwmon *hwmon, int *temp)
{
	bool found = false;
	long hottest = 0, value;
	u32 i;
	int ret;

	for (i = 0; i < hwmon->temp.count; i++) {
		struct macsmc_hwmon_sensor *sensor = &hwmon->temp.sensors[i];

		if (!sensor->die)
			continue;
		ret = macsmc_hwmon_read_key(hwmon->smc, sensor, 1000, &value);
		if (ret)
			return ret;
		/* A powered-down sensor (a GPU key while the GPU is off) has no say. */
		if (sensor->has_no_data_below && value < sensor->no_data_below)
			continue;
		if (value < MACSMC_DIE_MIN_MC || value > MACSMC_DIE_MAX_MC)
			return -ERANGE;
		if (!found || value > hottest)
			hottest = value;
		found = true;
	}
	if (!found)
		return -ENODATA;

	*temp = hottest;
	return 0;
}

static int macsmc_hwmon_die_get_temp(struct thermal_zone_device *tz, int *temp)
{
	return macsmc_hwmon_die_hottest(thermal_zone_device_priv(tz), temp);
}

static const struct thermal_zone_device_ops macsmc_hwmon_die_ops = {
	.get_temp = macsmc_hwmon_die_get_temp,
};

static void macsmc_hwmon_die_unregister(void *data)
{
	thermal_zone_device_unregister(data);
}

static int macsmc_hwmon_die_register(struct macsmc_hwmon *hwmon)
{
	struct thermal_zone_device *tz;
	char keys[16 * 5] = "";
	size_t len = 0;
	u32 i;

	if (!hwmon->die_listed)
		return 0;
	if (!hwmon->die_count) {
		dev_info(hwmon->dev, "no SoC die temperature zone: none of the %u listed keys is present\n",
			 hwmon->die_listed);
		return 0;
	}

	for (i = 0; i < hwmon->temp.count && len + 6 <= sizeof(keys); i++) {
		smc_key key = hwmon->temp.sensors[i].macsmc_key;

		if (hwmon->temp.sensors[i].die)
			len += scnprintf(keys + len, sizeof(keys) - len, " %c%c%c%c",
					 key >> 24, (key >> 16) & 0xff, (key >> 8) & 0xff, key & 0xff);
	}

	tz = thermal_tripless_zone_device_register(MACSMC_SOC_DIE_ZONE, hwmon,
						   &macsmc_hwmon_die_ops, NULL);
	if (IS_ERR(tz)) {
		dev_warn(hwmon->dev, "cannot register the SoC die temperature zone (%pe)\n", tz);
		return 0;
	}
	dev_info(hwmon->dev, "SoC die temperature zone %s: hottest of%s (%u of %u listed keys present)\n",
		 MACSMC_SOC_DIE_ZONE, keys, hwmon->die_count, hwmon->die_listed);
	return devm_add_action_or_reset(hwmon->dev, macsmc_hwmon_die_unregister, tz);
}
#else
static int macsmc_hwmon_die_register(struct macsmc_hwmon *hwmon)
{
	return 0;
}
#endif

/*
 * debugfs "keys": every key the SMC reports, with its type, size and flags, and the value of the
 * readable 32-bit float keys whose names start with 'T' (temperatures, in millidegrees Celsius)
 * or 'P' (power, in milliwatts). Which keys a machine has differs even between machines with
 * the same SoC, so this list is how sensors for the device tree are found. Only key metadata is
 * queried and only those float keys are read; nothing is written.
 */
static void macsmc_hwmon_seq_fourcc(struct seq_file *m, u32 code)
{
	int shift;

	for (shift = 24; shift >= 0; shift -= 8) {
		char c = (code >> shift) & 0xff;

		seq_putc(m, isprint(c) ? c : '.');
	}
}

static int macsmc_hwmon_keys_show(struct seq_file *m, void *unused)
{
	struct macsmc_hwmon *hwmon = m->private;
	struct apple_smc *smc = hwmon->smc;
	struct apple_smc_key_info info;
	smc_key key;
	long value;
	u32 i;
	int ret;

	seq_printf(m, "# %u keys: index key type size flags [value: T* in mC, P* in mW]\n",
		   smc->key_count);
	for (i = 0; i < smc->key_count; i++) {
		if (fatal_signal_pending(current))
			return -EINTR;
		ret = apple_smc_get_key_by_index(smc, i, &key);
		if (ret < 0) {
			seq_printf(m, "%4u error %d\n", i, ret);
			continue;
		}
		seq_printf(m, "%4u ", i);
		macsmc_hwmon_seq_fourcc(m, key);
		ret = apple_smc_get_key_info(smc, key, &info);
		if (ret < 0) {
			seq_printf(m, " info error %d\n", ret);
			continue;
		}
		seq_putc(m, ' ');
		macsmc_hwmon_seq_fourcc(m, info.type_code);
		seq_printf(m, " %3u %02x", info.size, info.flags);
		if (info.type_code == __SMC_KEY('f', 'l', 't', ' ') && info.size == sizeof(u32) &&
		    (info.flags & APPLE_SMC_READABLE) &&
		    ((key >> 24) == 'T' || (key >> 24) == 'P')) {
			ret = macsmc_hwmon_read_f32_scaled(smc, key, &value, 1000);
			if (ret < 0)
				seq_printf(m, " read error %d", ret);
			else
				seq_printf(m, " %ld", value);
		}
		seq_putc(m, '\n');
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(macsmc_hwmon_keys);

static void macsmc_hwmon_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

static void macsmc_hwmon_debugfs_init(struct macsmc_hwmon *hwmon)
{
	struct dentry *dir;

	dir = debugfs_create_dir(dev_name(hwmon->dev), NULL);
	if (IS_ERR(dir))
		return;
	debugfs_create_file("keys", 0400, dir, hwmon, &macsmc_hwmon_keys_fops);
	devm_add_action_or_reset(hwmon->dev, macsmc_hwmon_debugfs_remove, dir);
}

static const struct hwmon_channel_info *const macsmc_chip_channel_info =
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ);
static const struct hwmon_channel_info *const macsmc_chip_channel_info_no_tz =
	HWMON_CHANNEL_INFO(chip, 0);

static int macsmc_hwmon_create_infos(struct macsmc_hwmon *hwmon)
{
	struct hwmon_channel_info *channel_info;
	int i = 0;

	/* chip */
	hwmon->channel_infos[i++] = hwmon->register_thermal_zones ?
		macsmc_chip_channel_info : macsmc_chip_channel_info_no_tz;

	if (hwmon->curr.count) {
		channel_info = &hwmon->curr.channel_info;
		channel_info->type = hwmon_curr;
		channel_info->config = devm_kcalloc(hwmon->dev, hwmon->curr.count + 1,
						    sizeof(u32), GFP_KERNEL);
		if (!channel_info->config)
			return -ENOMEM;

		macsmc_hwmon_populate_configs((u32 *)channel_info->config, &hwmon->curr);
		hwmon->channel_infos[i++] = channel_info;
	}

	if (hwmon->fan.count) {
		channel_info = &hwmon->fan.channel_info;
		channel_info->type = hwmon_fan;
		channel_info->config = devm_kcalloc(hwmon->dev, hwmon->fan.count + 1,
						    sizeof(u32), GFP_KERNEL);
		if (!channel_info->config)
			return -ENOMEM;

		macsmc_hwmon_populate_fan_configs((u32 *)channel_info->config, &hwmon->fan);
		hwmon->channel_infos[i++] = channel_info;
	}

	if (hwmon->power.count) {
		channel_info = &hwmon->power.channel_info;
		channel_info->type = hwmon_power;
		channel_info->config = devm_kcalloc(hwmon->dev, hwmon->power.count + 1,
						    sizeof(u32), GFP_KERNEL);
		if (!channel_info->config)
			return -ENOMEM;

		macsmc_hwmon_populate_configs((u32 *)channel_info->config, &hwmon->power);
		hwmon->channel_infos[i++] = channel_info;
	}

	if (hwmon->temp.count) {
		channel_info = &hwmon->temp.channel_info;
		channel_info->type = hwmon_temp;
		channel_info->config = devm_kcalloc(hwmon->dev, hwmon->temp.count + 1,
						    sizeof(u32), GFP_KERNEL);
		if (!channel_info->config)
			return -ENOMEM;

		macsmc_hwmon_populate_configs((u32 *)channel_info->config, &hwmon->temp);
		hwmon->channel_infos[i++] = channel_info;
	}

	if (hwmon->volt.count) {
		channel_info = &hwmon->volt.channel_info;
		channel_info->type = hwmon_in;
		channel_info->config = devm_kcalloc(hwmon->dev, hwmon->volt.count + 1,
						    sizeof(u32), GFP_KERNEL);
		if (!channel_info->config)
			return -ENOMEM;

		macsmc_hwmon_populate_configs((u32 *)channel_info->config, &hwmon->volt);
		hwmon->channel_infos[i++] = channel_info;
	}

	return 0;
}

static int macsmc_hwmon_probe(struct platform_device *pdev)
{
	struct apple_smc *smc = dev_get_drvdata(pdev->dev.parent);
	struct macsmc_hwmon *hwmon;
	int ret;

	/*
	 * The MFD driver will try to probe us unconditionally. Some devices
	 * with the SMC do not have hwmon capabilities. Only probe if we have
	 * a hwmon node.
	 */
	if (!pdev->dev.of_node)
		return -ENODEV;

	hwmon = devm_kzalloc(&pdev->dev, sizeof(*hwmon),
			     GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	hwmon->dev = &pdev->dev;
	hwmon->smc = smc;
	hwmon->register_thermal_zones = true;
	platform_set_drvdata(pdev, hwmon);

	ret = devm_mutex_init(&pdev->dev, &hwmon->fan.lock);
	if (ret)
		return ret;

	ret = macsmc_hwmon_populate_sensors(hwmon, hwmon->dev->of_node);
	if (ret) {
		dev_err(hwmon->dev, "Could not parse sensors\n");
		return ret;
	}

	ret = macsmc_hwmon_create_pressure(hwmon, hwmon->dev->of_node);
	if (ret)
		return ret;

	ret = macsmc_hwmon_discover_temperatures(hwmon);
	if (ret)
		return dev_err_probe(hwmon->dev, ret, "Temperature discovery failed\n");

	if (!hwmon->curr.count && !hwmon->fan.count &&
	    !hwmon->power.count && !hwmon->temp.count &&
	    !hwmon->volt.count && !hwmon->pressure.macsmc_key) {
		dev_err(hwmon->dev,
			"No valid sensors found of any supported type\n");
		return -ENODEV;
	}

	ret = macsmc_hwmon_create_infos(hwmon);
	if (ret)
		return ret;

	if (hwmon->fan.count) {
		struct apple_smc_key_info info;

		if (!apple_smc_get_key_info(smc, SMC_KEY(Ftst), &info) &&
		    info.type_code == __SMC_KEY('u', 'i', '8', ' ') &&
		    (info.flags & APPLE_SMC_WRITABLE))
			hwmon->fan.unlock_key = SMC_KEY(Ftst);

		/* Runs after the hwmon device is gone, so no write races it. */
		ret = devm_add_action_or_reset(&pdev->dev,
					       macsmc_hwmon_fans_restore_action, hwmon);
		if (ret)
			return ret;
	}

	hwmon->chip_info.ops = &macsmc_hwmon_ops;
	hwmon->chip_info.info =
		(const struct hwmon_channel_info *const *)&hwmon->channel_infos;

	hwmon->hwmon_dev = devm_hwmon_device_register_with_info(&pdev->dev,
								"macsmc_hwmon", hwmon,
								&hwmon->chip_info,
								macsmc_hwmon_extra_groups);
	if (IS_ERR(hwmon->hwmon_dev))
		return dev_err_probe(hwmon->dev, PTR_ERR(hwmon->hwmon_dev),
				     "Probing SMC hwmon device failed\n");

	macsmc_hwmon_debugfs_init(hwmon);

	ret = macsmc_hwmon_die_register(hwmon);
	if (ret)
		return ret;

	dev_dbg(hwmon->dev, "Registered SMC hwmon device. Sensors:\n");
	dev_dbg(hwmon->dev,
		"Current: %d, Fans: %d, Power: %d, Temperature: %d, Voltage: %d",
		hwmon->curr.count, hwmon->fan.count,
		hwmon->power.count, hwmon->temp.count,
		hwmon->volt.count);

	return 0;
}

static const struct of_device_id macsmc_hwmon_of_table[] = {
	{ .compatible = "apple,smc-hwmon" },
	{}
};
MODULE_DEVICE_TABLE(of, macsmc_hwmon_of_table);

static void macsmc_hwmon_shutdown(struct platform_device *pdev)
{
	struct macsmc_hwmon *hwmon = platform_get_drvdata(pdev);

	if (hwmon)
		macsmc_hwmon_fans_restore_action(hwmon);
}

static int macsmc_hwmon_suspend(struct device *dev)
{
	macsmc_hwmon_fans_restore_action(dev_get_drvdata(dev));
	return 0;
}

/* Manual requests end at suspend; resume never writes fan keys. */
static int macsmc_hwmon_resume(struct device *dev)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);

	guard(mutex)(&hwmon->fan.lock);
	hwmon->fan.stopped = false;
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(macsmc_hwmon_pm_ops, macsmc_hwmon_suspend,
				macsmc_hwmon_resume);

static struct platform_driver macsmc_hwmon_driver = {
	.probe = macsmc_hwmon_probe,
	.shutdown = macsmc_hwmon_shutdown,
	.driver = {
		.name = "macsmc-hwmon",
		.of_match_table = macsmc_hwmon_of_table,
		.pm = pm_sleep_ptr(&macsmc_hwmon_pm_ops),
	},
};
module_platform_driver(macsmc_hwmon_driver);

#if IS_ENABLED(CONFIG_SENSORS_MACSMC_HWMON_KUNIT_TEST)
#include "macsmc-hwmon-test.c"
#endif

MODULE_DESCRIPTION("Apple Silicon SMC hwmon driver");
MODULE_AUTHOR("James Calligeros <jcalligeros99@gmail.com>");
MODULE_LICENSE("Dual MIT/GPL");
