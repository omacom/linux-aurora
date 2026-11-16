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
#include <linux/hwmon.h>
#include <linux/math64.h>
#include <linux/mfd/macsmc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
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

static bool fan_control;
module_param_unsafe(fan_control, bool, 0644);
MODULE_PARM_DESC(fan_control,
		 "Override the SMC to set your own fan speeds on supported machines");

/*
 * SoC die temperature keys: temperature channels labelled "SoC die <key>"
 * whose hottest reading is the SoC die thermal zone (see
 * macsmc_hwmon_die_get_temp()). They come from this parameter or, without it,
 * from the machine's entry in macsmc_hwmon_boards[]. Keys the SMC does not
 * have are skipped.
 */
static char *soc_die_keys;
module_param(soc_die_keys, charp, 0444);
MODULE_PARM_DESC(soc_die_keys,
		 "Comma-separated SMC temperature keys whose hottest reading is the SoC die thermal zone, \"none\" for no zone (default: the machine's list)");

/*
 * Keys a machine's device tree does not list yet. The device tree comes from
 * the boot loader, so these let a kernel use keys before its device tree
 * catches up. board_keys=0 turns them off: the device tree alone then decides
 * (soc_die_keys still applies).
 */
static bool board_keys = true;
module_param(board_keys, bool, 0444);
MODULE_PARM_DESC(board_keys,
		 "Use the built-in per-machine SoC die and thermal pressure keys the device tree does not list (default: on)");

static const struct macsmc_hwmon_board {
	const char *compatible;
	const char *die_keys;
	const char *pressure_key;
} macsmc_hwmon_boards[] = {
	/*
	 * MacBook Pro (16-inch, M3 Pro): key names used for the CPU
	 * performance-core and GPU die sensors of M3-family machines. They
	 * are not confirmed on this machine: the debugfs key list shows which
	 * temperature keys it has.
	 */
	{
		.compatible = "apple,j516s",
		.die_keys = "Tf04,Tf09,Tf0A,Tf0B,Tf0D,Tf0E,Tf14,Tf18,Tf19,Tf1A,Tf24,Tf25,Tf2A,Tf2B",
		.pressure_key = "mTPL",
	},
	/*
	 * MacBook Pro (14-inch and 16-inch, M3 Max): the same key family in
	 * five groups. On a J516C every one of these keys read a die
	 * temperature (47-62 C at idle); the keys ending in D and E repeated
	 * the one ending in 4 and are left out. The J514C list is assumed to
	 * be the same; keys the SMC does not have are skipped.
	 */
	{
		.compatible = "apple,j516c",
		.die_keys = "Tf04,Tf06,Tf09,Tf0A,Tf0B,Tf14,Tf16,Tf19,Tf1A,Tf1B,Tf24,Tf26,Tf29,Tf2A,Tf2B,"
			    "Tf34,Tf36,Tf39,Tf3A,Tf3B,Tf44,Tf46,Tf49,Tf4A,Tf4B",
	},
	{
		.compatible = "apple,j514c",
		.die_keys = "Tf04,Tf06,Tf09,Tf0A,Tf0B,Tf14,Tf16,Tf19,Tf1A,Tf1B,Tf24,Tf26,Tf29,Tf2A,Tf2B,"
			    "Tf34,Tf36,Tf39,Tf3A,Tf3B,Tf44,Tf46,Tf49,Tf4A,Tf4B",
	},
	/*
	 * MacBook Air (13-inch, M3): the base M3 has no Tf keys. With only
	 * the GPU busy, the six Tg keys below rose by about 50 C, the most
	 * of any key, and each has a twin about 4.5 C lower; they read 0 C
	 * or slightly below while the GPU is powered off. The others are the
	 * CPU die keys of the board's device tree (the E-cluster groups and
	 * one per P core), so the zone also follows a hot CPU.
	 */
	{
		.compatible = "apple,j613",
		.die_keys = "Tg0D,Tg0P,Tg0X,Tg0b,Tg0j,Tg0v,Te05,Te0P,Tp05,Tp0L,Tp0b,Tp0r",
		.pressure_key = "mTPL",
	},
};

static bool macsmc_hwmon_is_m3(void)
{
	return of_machine_is_compatible("apple,t6030") ||
	       of_machine_is_compatible("apple,t6031") ||
	       of_machine_is_compatible("apple,t6032") ||
	       of_machine_is_compatible("apple,t6034") ||
	       of_machine_is_compatible("apple,t8122");
}

static const struct macsmc_hwmon_board *macsmc_hwmon_board(void)
{
	unsigned int i;

	if (!board_keys)
		return NULL;
	for (i = 0; i < ARRAY_SIZE(macsmc_hwmon_boards); i++)
		if (of_machine_is_compatible(macsmc_hwmon_boards[i].compatible))
			return &macsmc_hwmon_boards[i];
	return NULL;
}

/* The list of SoC die temperature keys, or NULL for none. */
static const char *macsmc_hwmon_die_key_list(void)
{
	const struct macsmc_hwmon_board *board;

	if (!macsmc_hwmon_is_m3())
		return NULL;
	if (soc_die_keys)
		return strcmp(soc_die_keys, "none") ? soc_die_keys : NULL;
	board = macsmc_hwmon_board();
	return board ? board->die_keys : NULL;
}

/*
 * The next key of a comma-separated list: returns 1 and the key, 0 at the end
 * of the list, or -EINVAL for an entry that is not four characters long.
 */
static int macsmc_hwmon_next_key(const char **list, char key[5])
{
	const char *p = *list, *end;

	while (*p == ',' || *p == ' ')
		p++;
	if (!*p)
		return 0;
	end = strchrnul(p, ',');
	*list = end;
	if (end - p != 4)
		return -EINVAL;
	memcpy(key, p, 4);
	key[4] = '\0';
	return 1;
}

struct macsmc_hwmon_sensor {
	struct apple_smc_key_info info;
	smc_key macsmc_key;
	char label[MAX_LABEL_LENGTH];
	u32 attrs;
	/* Part of the SoC die temperature zone. */
	bool die;
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
};

struct macsmc_hwmon_sensors {
	struct hwmon_channel_info channel_info;
	struct macsmc_hwmon_sensor *sensors;
	u32 count;
};

struct macsmc_hwmon_fans {
	struct hwmon_channel_info channel_info;
	struct macsmc_hwmon_fan *fans;
	u32 count;
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

static int macsmc_hwmon_write_fan(struct device *dev, u32 attr, int channel,
				  long val)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);
	long min, max;
	int ret;

	if (!fan_control || hwmon->fan.fans[channel].mode.macsmc_key == 0)
		return -EOPNOTSUPP;

	/*
	 * The SMC does no sanity checks on requested fan speeds, so we need to.
	 */
	ret = macsmc_hwmon_read_key(hwmon->smc, &hwmon->fan.fans[channel].min,
				    1, &min);
	if (ret)
		return ret;

	ret = macsmc_hwmon_read_key(hwmon->smc, &hwmon->fan.fans[channel].max,
				    1, &max);
	if (ret)
		return ret;

	if (val >= min && val <= max) {
		if (!hwmon->fan.fans[channel].manual) {
			/* Write 1 to mode key for manual control */
			ret = macsmc_hwmon_write_key(hwmon->smc,
						     &hwmon->fan.fans[channel].mode, 1);
			if (ret < 0)
				return ret;

			hwmon->fan.fans[channel].manual = true;
		}
		return macsmc_hwmon_write_key(hwmon->smc,
					      &hwmon->fan.fans[channel].set, val);
	} else if (!val) {
		if (hwmon->fan.fans[channel].manual) {
			ret = macsmc_hwmon_write_key(hwmon->smc,
						     &hwmon->fan.fans[channel].mode, 0);
			if (ret < 0)
				return ret;

			hwmon->fan.fans[channel].manual = false;
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

static int macsmc_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			     u32 attr, int channel, long *val)
{
	struct macsmc_hwmon *hwmon = dev_get_drvdata(dev);
	int ret = 0;

	switch (type) {
	case hwmon_temp:
		ret = macsmc_hwmon_read_key(hwmon->smc,
					    &hwmon->temp.sensors[channel], 1000, val);
		break;
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
		if (attr == hwmon_fan_target && fan_control && fan->mode.macsmc_key)
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

	if (macsmc_hwmon_is_m3()) {
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
	}

	ret = of_property_read_string(sensor_node, "label", &label);
	if (ret)
		dev_dbg(dev, "No label found for sensor %s\n", key);
	else
		strscpy_pad(sensor->label, label, sizeof(sensor->label));

	return 0;
}

/*
 * A SoC die temperature sensor: a readable float temperature key, labelled
 * "SoC die <key>".
 */
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
	if (ret == -EINVAL) {
		const struct macsmc_hwmon_board *board = macsmc_hwmon_board();

		if (!board || !board->pressure_key)
			return 0;
		key = board->pressure_key;
	} else if (ret) {
		return ret;
	}

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

	/* The following keys are not required to simply monitor fan speed */
	if (!of_property_read_string(fan_node, "apple,fan-minimum", &min)) {
		ret = macsmc_hwmon_parse_key(dev, smc, &fan->min, min);
		if (ret)
			return ret;

		fan->attrs |= HWMON_F_MIN;
	}

	if (!of_property_read_string(fan_node, "apple,fan-maximum", &max)) {
		ret = macsmc_hwmon_parse_key(dev, smc, &fan->max, max);
		if (ret)
			return ret;

		fan->attrs |= HWMON_F_MAX;
	}

	if (!of_property_read_string(fan_node, "apple,fan-target", &set)) {
		ret = macsmc_hwmon_parse_key(dev, smc, &fan->set, set);
		if (ret)
			return ret;

		fan->attrs |= HWMON_F_TARGET;
	}

	if (!of_property_read_string(fan_node, "apple,fan-mode", &mode)) {
		ret = macsmc_hwmon_parse_key(dev, smc, &fan->mode, mode);
		if (ret)
			return ret;
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
	const char *die_keys = macsmc_hwmon_die_key_list(), *list;
	char key[5];
	bool indexed;
	int ret;

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

		for_each_child_of_node_with_prefix(hwmon_node, key_node, "fan-") {
			if (!macsmc_hwmon_create_fan(hwmon->dev, hwmon->smc, key_node,
						     &hwmon->fan.fans[hwmon->fan.count]))
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
	for (list = die_keys; list && (ret = macsmc_hwmon_next_key(&list, key));)
		n_temperature++;

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

		for (list = die_keys; list && (ret = macsmc_hwmon_next_key(&list, key));) {
			if (ret < 0) {
				dev_warn(hwmon->dev, "ignoring an entry of the SoC die key list \"%s\"\n",
					 die_keys);
				continue;
			}
			hwmon->die_listed++;
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
 * implausible view.
 */
#if IS_ENABLED(CONFIG_THERMAL)
#define MACSMC_SOC_DIE_ZONE	"macsmc_soc_die"
#define MACSMC_DIE_MIN_MC	-40000
#define MACSMC_DIE_MAX_MC	150000

static int macsmc_hwmon_die_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct macsmc_hwmon *hwmon = thermal_zone_device_priv(tz);
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

	ret = macsmc_hwmon_populate_sensors(hwmon, hwmon->dev->of_node);
	if (ret) {
		dev_err(hwmon->dev, "Could not parse sensors\n");
		return ret;
	}

	if (macsmc_hwmon_is_m3()) {
		ret = macsmc_hwmon_create_pressure(hwmon, hwmon->dev->of_node);
		if (ret)
			return ret;
	}

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

	hwmon->chip_info.ops = &macsmc_hwmon_ops;
	hwmon->chip_info.info =
		(const struct hwmon_channel_info *const *)&hwmon->channel_infos;

	hwmon->hwmon_dev = devm_hwmon_device_register_with_info(&pdev->dev,
								"macsmc_hwmon", hwmon,
								&hwmon->chip_info,
								macsmc_hwmon_is_m3() ? macsmc_hwmon_extra_groups : NULL);
	if (IS_ERR(hwmon->hwmon_dev))
		return dev_err_probe(hwmon->dev, PTR_ERR(hwmon->hwmon_dev),
				     "Probing SMC hwmon device failed\n");

	if (macsmc_hwmon_is_m3()) {
		macsmc_hwmon_debugfs_init(hwmon);
		ret = macsmc_hwmon_die_register(hwmon);
		if (ret)
			return ret;
	}

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

static struct platform_driver macsmc_hwmon_driver = {
	.probe = macsmc_hwmon_probe,
	.driver = {
		.name = "macsmc-hwmon",
		.of_match_table = macsmc_hwmon_of_table,
	},
};
module_platform_driver(macsmc_hwmon_driver);

#if IS_ENABLED(CONFIG_SENSORS_MACSMC_HWMON_KUNIT_TEST)
#include "macsmc-hwmon-test.c"
#endif

MODULE_DESCRIPTION("Apple Silicon SMC hwmon driver");
MODULE_AUTHOR("James Calligeros <jcalligeros99@gmail.com>");
MODULE_LICENSE("Dual MIT/GPL");
