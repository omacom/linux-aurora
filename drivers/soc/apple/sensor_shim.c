/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Copyright 2026 Dj */
/* Fingerprint sensor SPI shim: moves bytes over the bus and toggles power. */

#include <linux/build_bug.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/interrupt.h>
#include <linux/mfd/macsmc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#include <linux/mutex.h>

#include "shim.h"

/*
 * Userspace checks this before loading the driver. T6000 retries sensor
 * registration before both verification and enrolment. Keep the marker
 * with both profile policies when backporting.
 */
MODULE_INFO(t6000_registration, "1");

/* The board selects the SPI mode; an override is diagnostic only. */
#define SEP_SENSOR_HZ		8000000
#define SEP_SENSOR_BITS		8
static unsigned int sep_sensor_mode = ~0U;
module_param_named(sensor_spi_mode, sep_sensor_mode, uint, 0444);
MODULE_PARM_DESC(sensor_spi_mode,
		 "Diagnostic SPI mode override (0-3); default retains the board mode");

/* Chip-select setup and hold, in ns. */
#define SEP_SENSOR_CS_NS		20

#define SEP_SENSOR_OFF_DELAY_MS	10
#define SEP_SENSOR_ON_DELAY_MS	7

struct sep_sensor_lifetime {
	/* Serializes sensor operations with this device's resource teardown. */
	struct mutex lock;
};

static DEFINE_MUTEX(sep_sensor_registry_lock);
static struct sep_sensor_lifetime *sep_active_sensor;

static struct sep_sensor_lifetime *sep_lock_sensor(void)
{
	struct sep_sensor_lifetime *sensor;

	mutex_lock(&sep_sensor_registry_lock);
	sensor = sep_active_sensor;
	if (sensor)
		mutex_lock(&sensor->lock);
	mutex_unlock(&sep_sensor_registry_lock);
	return sensor;
}

static void sep_sensor_irq_teardown_locked(void);

static struct spi_device *sep_spi;
static struct gpio_desc *sep_power;
static bool sep_registered;

/*
 * Power line resolved by DT node path, not gpiochip index: registration order
 * shifts, so an index would silently pick the wrong chip.
 */
#define SEP_SENSOR_GPIO_NODE		"/soc/pinctrl@39b028000"
#define SEP_SENSOR_GPIO_LINE		122

#define SEP_POWER_NONE		0
#define SEP_POWER_NODE_PROPERTY	1
#define SEP_POWER_CHIP_LINE		2
#define SEP_POWER_SMC_RAILS		3

static int sep_power_source;

/* J700's Mesa power rails are selected by the pcLD SMC pKW8 operation.
 * The DT gives the key and selectors; each write is { u32 on/off, u32 rail }.
 */
#define SEP_SMC_MAX_RAILS	4
static struct apple_smc *sep_smc;
static smc_key sep_smc_key;
static u32 sep_smc_rails[SEP_SMC_MAX_RAILS];
static int sep_smc_nrails;

/*
 * Data-ready line for interrupt-driven capture. On j414s it is the pin adjacent
 * to the power line (122) on the same controller; other boards describe it as
 * the SPI node's interrupt, which the SPI core resolves into spi->irq.
 */
#define SEP_SENSOR_DRDY_LINE	121
static struct gpio_desc *sep_drdy;	/* set only when we own the gpiochip line (j414s) */
static int sep_drdy_irq = -1;
static DECLARE_COMPLETION(sep_drdy_done);

#if IS_ENABLED(CONFIG_MFD_MACSMC)
static int sep_smc_write_rail(u32 rail, u16 value)
{
	u8 payload[8];
	int ret;

	put_unaligned_le32(value, payload);
	put_unaligned_le32(rail, payload + 4);
	ret = apple_smc_write(sep_smc, sep_smc_key, payload, sizeof(payload));
	/* macsmc forwards the SMC response size. Write replies may report no
	 * payload or echo the request length; neither is a failure status.
	 */
	if (ret == 0 || ret == (int)sizeof(payload))
		return 0;
	return ret < 0 ? ret : -EIO;
}

static int sep_smc_set_power(int on)
{
	int i, rc, first_error = 0;

	if (on) {
		for (i = 0; i < sep_smc_nrails; i++) {
			rc = sep_smc_write_rail(sep_smc_rails[i], 1);
			if (rc) {
				first_error = rc;
				/* The failed write may have taken effect too. */
				for (; i >= 0; i--) {
					rc = sep_smc_write_rail(sep_smc_rails[i], 0);
					if (rc)
						pr_err("apple_sep: sensor: SMC rail rollback failed (%d)\n", rc);
				}
				break;
			}
		}
	} else {
		/* A failed off write must not skip the remaining rails. */
		for (i = sep_smc_nrails - 1; i >= 0; i--) {
			rc = sep_smc_write_rail(sep_smc_rails[i], 0);
			if (rc && !first_error)
				first_error = rc;
		}
	}
	if (first_error)
		pr_err("apple_sep: sensor: SMC rail write failed (%d)\n", first_error);
	return first_error;
}

/* Nothing is written until the expected eight-byte SMC key is verified. */
static int sep_acquire_smc_power(struct spi_device *spi)
{
	struct device_node *np = spi->dev.of_node, *smc_np;
	struct platform_device *smc_pdev;
	struct apple_smc_key_info info;
	const char *key = NULL;
	int n, rc;

	if (!np || of_property_read_string(np, "apple,smc-power-key", &key))
		return -EINVAL;
	if (!key || strlen(key) != 4) {
		dev_warn(&spi->dev, "sep sensor: apple,smc-power-key must have four characters\n");
		return -EINVAL;
	}
	n = of_property_count_u32_elems(np, "apple,smc-power-rails");
	if (n <= 0 || n > SEP_SMC_MAX_RAILS ||
	    of_property_read_u32_array(np, "apple,smc-power-rails", sep_smc_rails, n)) {
		dev_warn(&spi->dev, "sep sensor: expected 1..%d SMC rail words\n",
			 SEP_SMC_MAX_RAILS);
		return -EINVAL;
	}

	smc_np = of_parse_phandle(np, "apple,smc", 0);
	if (!smc_np) {
		dev_warn(&spi->dev, "sep sensor: apple,smc-power-key needs an apple,smc phandle\n");
		return -EINVAL;
	}
	smc_pdev = of_find_device_by_node(smc_np);
	of_node_put(smc_np);
	if (!smc_pdev)
		return -EPROBE_DEFER;
	/* Keep the supplier bound for as long as this sensor uses its drvdata. */
	if (!device_link_add(&spi->dev, &smc_pdev->dev,
			     DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(&smc_pdev->dev);
		return -ENOMEM;
	}
	sep_smc = dev_get_drvdata(&smc_pdev->dev);
	put_device(&smc_pdev->dev);
	if (!sep_smc)
		return -EPROBE_DEFER;

	sep_smc_key = __SMC_KEY(key[0], key[1], key[2], key[3]);
	rc = apple_smc_get_key_info(sep_smc, sep_smc_key, &info);
	if (rc || info.size != 8) {
		dev_warn(&spi->dev, "sep sensor: SMC key %s absent or not eight bytes\n", key);
		sep_smc = NULL;
		return rc ?: -EINVAL;
	}
	sep_smc_nrails = n;
	sep_power_source = SEP_POWER_SMC_RAILS;
	dev_info(&spi->dev, "sep sensor: power from SMC key %s (%d rails)\n", key, n);
	rc = sep_smc_set_power(0);
	if (rc) {
		sep_power_source = SEP_POWER_NONE;
		sep_smc_nrails = 0;
		sep_smc = NULL;
		return rc;
	}
	return 0;
}
#else
static int sep_smc_set_power(int on)
{
	return -ENODEV;
}

static int sep_acquire_smc_power(struct spi_device *spi)
{
	if (spi->dev.of_node &&
	    of_property_present(spi->dev.of_node, "apple,smc-power-key")) {
		dev_warn(&spi->dev, "sep sensor: SMC power needs CONFIG_MFD_MACSMC\n");
		return -ENODEV;
	}
	return -ENOENT;
}
#endif

/*
 * Takes the power line as an output driven low: the power cycle begins with an
 * off phase, so the line must be actively driven off, not merely read as low.
 */
static int sep_acquire_power(struct spi_device *spi)
{
	struct device_node *np;
	struct gpio_device *gdev;
	struct gpio_chip *gc;

	if (spi->dev.of_node &&
	    of_property_present(spi->dev.of_node, "apple,smc-power-key"))
		return sep_acquire_smc_power(spi);

	sep_power = gpiod_get(&spi->dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(sep_power) && PTR_ERR(sep_power) == -ENOENT)
		sep_power = gpiod_get_index(&spi->dev, NULL, 0, GPIOD_OUT_LOW);
	if (IS_ERR(sep_power) && PTR_ERR(sep_power) != -ENOENT) {
		int error = PTR_ERR(sep_power);

		sep_power = NULL;
		return error;
	}
	if (!IS_ERR(sep_power)) {
		sep_power_source = SEP_POWER_NODE_PROPERTY;
		dev_info(&spi->dev,
			 "sep sensor: power line from the device node, driven low\n");
		return 0;
	}
	sep_power = NULL;

	/*
	 * The fallback below is the j414s power line; a sibling board's sensor sits
	 * on a different pin, so only j414s may use it. Every other machine must
	 * describe enable-gpios in its device node.
	 */
	if (!of_machine_is_compatible("apple,j414s")) {
		dev_warn(&spi->dev,
			 "sep sensor: no power GPIO in the device node; describe enable-gpios in DT\n");
		return -ENODEV;
	}

	np = of_find_node_by_path(SEP_SENSOR_GPIO_NODE);
	if (!np) {
		dev_warn(&spi->dev,
			 "sep sensor: no DT node at %s, no power line\n",
			 SEP_SENSOR_GPIO_NODE);
		return -ENODEV;
	}

	gdev = gpio_device_find_by_fwnode(of_fwnode_handle(np));
	of_node_put(np);
	if (!gdev) {
		dev_warn(&spi->dev,
			 "sep sensor: %s has no registered GPIO device\n",
			 SEP_SENSOR_GPIO_NODE);
		return -EPROBE_DEFER;
	}

	gc = gpio_device_get_chip(gdev);
	if (gc)
		sep_power = gpiochip_request_own_desc(gc,
							 SEP_SENSOR_GPIO_LINE,
							 "apple-mesa-power",
							 GPIO_LOOKUP_FLAGS_DEFAULT,
							 GPIOD_OUT_LOW);
	if (!gc || IS_ERR(sep_power)) {
		sep_power = NULL;
		dev_warn(&spi->dev,
			 "sep sensor: could not take line %d on %s (%s)\n",
			 SEP_SENSOR_GPIO_LINE, SEP_SENSOR_GPIO_NODE,
			 gpio_device_get_label(gdev));
		gpio_device_put(gdev);
		return -ENODEV;
	}

	sep_power_source = SEP_POWER_CHIP_LINE;
	dev_info(&spi->dev,
		 "sep sensor: power line %s line %d (%s), driven low\n",
		 SEP_SENSOR_GPIO_NODE, SEP_SENSOR_GPIO_LINE,
		 gpio_device_get_label(gdev));
	gpio_device_put(gdev);
	return 0;
}

static void sep_release_power(void)
{
	if (sep_power_source == SEP_POWER_SMC_RAILS) {
		sep_smc_set_power(0);
		sep_smc = NULL;
		sep_smc_nrails = 0;
		sep_power_source = SEP_POWER_NONE;
		return;
	}
	if (!sep_power)
		return;
	gpiod_set_value_cansleep(sep_power, 0);
	if (sep_power_source == SEP_POWER_CHIP_LINE)
		gpiochip_free_own_desc(sep_power);
	else
		gpiod_put(sep_power);
	sep_power = NULL;
	sep_power_source = SEP_POWER_NONE;
}

#define SEP_CS_TIMING_SOFTWARE	0
#define SEP_CS_TIMING_HOOK_BYPASSED	1
#define SEP_CS_TIMING_HARDWARE	2

static int sep_cs_timing_mode;

/*
 * Programs the 20 ns chip-select setup/hold and applies mode/speed via
 * spi_setup(), which reaches the controller's set_cs_timing hook.
 *
 * This sensor needs hardware timing. The hook runs only for a native chip
 * select; with a GPIO chip select the core silently emulates the delays in
 * software, which this sensor rejects but which looks like success.
 */
static int sep_apply_cs_timing(struct spi_device *spi)
{
	struct spi_controller *ctlr = spi->controller;
	int rc;

	if (!ctlr->set_cs_timing)
		sep_cs_timing_mode = SEP_CS_TIMING_SOFTWARE;
	else if (spi_get_csgpiod(spi, 0))
		sep_cs_timing_mode = SEP_CS_TIMING_HOOK_BYPASSED;
	else
		sep_cs_timing_mode = SEP_CS_TIMING_HARDWARE;

	/*
	 * Hold the bus lock: these fields are read when a transfer asserts chip
	 * select. spi_setup() takes the controller io_mutex, not the bus lock,
	 * so no deadlock.
	 */
	if (sep_sensor_mode != ~0U && sep_sensor_mode > SPI_MODE_3)
		return -EINVAL;

	rc = spi_bus_lock(ctlr);
	if (rc)
		return rc;

	if (sep_sensor_mode != ~0U)
		spi->mode = sep_sensor_mode;
	spi->bits_per_word = SEP_SENSOR_BITS;
	spi->max_speed_hz = SEP_SENSOR_HZ;
	spi->cs_setup.value = SEP_SENSOR_CS_NS;
	spi->cs_setup.unit = SPI_DELAY_UNIT_NSECS;
	spi->cs_hold.value = SEP_SENSOR_CS_NS;
	spi->cs_hold.unit = SPI_DELAY_UNIT_NSECS;

	rc = spi_setup(spi);

	spi_bus_unlock(ctlr);

	if (rc)
		return rc;

	switch (sep_cs_timing_mode) {
	case SEP_CS_TIMING_HARDWARE:
		dev_info(&spi->dev,
			 "sep sensor: CS timing %u ns programmed in hardware\n",
			 SEP_SENSOR_CS_NS);
		break;
	case SEP_CS_TIMING_HOOK_BYPASSED:
		dev_warn(&spi->dev,
			 "sep sensor: GPIO chip select, CS timing emulated in software (sensor needs hardware timing)\n");
		break;
	default:
		dev_warn(&spi->dev,
			 "sep sensor: no set_cs_timing hook, %u ns emulated in software (sensor needs hardware timing)\n",
			 SEP_SENSOR_CS_NS);
		break;
	}
	return 0;
}

static int sep_sensor_probe_locked(struct spi_device *spi)
{
	int rc;

	rc = sep_apply_cs_timing(spi);
	if (rc)
		return rc;

	rc = sep_acquire_power(spi);
	if (rc)
		return dev_err_probe(&spi->dev, rc, "sep sensor: power unavailable\n");
	sep_spi = spi;
	/* Read the mode back: a mode that did not take must not be logged as
	 * the one requested. */
	dev_info(&spi->dev,
		 "sep sensor: bound, %u Hz, mode %u (CPOL=%d CPHA=%d)\n",
		 spi->max_speed_hz,
		 (unsigned int)(spi->mode & (SPI_CPOL | SPI_CPHA)),
		 !!(spi->mode & SPI_CPOL), !!(spi->mode & SPI_CPHA));
	return 0;
}

static int sep_sensor_probe(struct spi_device *spi)
{
	struct sep_sensor_lifetime *sensor;
	int error;

	sensor = devm_kzalloc(&spi->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	mutex_init(&sensor->lock);
	mutex_lock(&sep_sensor_registry_lock);
	if (sep_active_sensor) {
		error = -EBUSY;
	} else {
		error = sep_sensor_probe_locked(spi);
		if (!error) {
			spi_set_drvdata(spi, sensor);
			sep_active_sensor = sensor;
			sep_diag_set_sensor_bound(true);
		}
	}
	mutex_unlock(&sep_sensor_registry_lock);
	return error;
}

static void sep_sensor_remove(struct spi_device *spi)
{
	struct sep_sensor_lifetime *sensor = spi_get_drvdata(spi);

	mutex_lock(&sep_sensor_registry_lock);
	mutex_lock(&sensor->lock);
	sep_active_sensor = NULL;
	sep_sensor_irq_teardown_locked();
	sep_release_power();
	sep_spi = NULL;
	sep_diag_set_sensor_bound(false);
	mutex_unlock(&sensor->lock);
	mutex_unlock(&sep_sensor_registry_lock);
}

/*
 * Its own compatible, deliberately not spidev's: capture must stay in the
 * kernel, and a spidev node would leak an image to userspace. No
 * MODULE_DEVICE_TABLE, so udev cannot auto-load this module.
 */
static const struct of_device_id sep_sensor_of_match[] = {
	{ .compatible = "apple,mesa-fingerprint" },
	{ }
};

/*
 * Legacy ID table only to silence the SPI core's "no spi_device_id" note. No
 * MODULE_DEVICE_TABLE: it quiets a message, it does not advertise a binding.
 */
static const struct spi_device_id sep_sensor_spi_ids[] = {
	{ "mesa-fingerprint", 0 },
	{ }
};

static struct spi_driver sep_sensor_driver = {
	.driver = {
		.name = "apple-mesa",
		.of_match_table = sep_sensor_of_match,
	},
	.id_table = sep_sensor_spi_ids,
	.probe = sep_sensor_probe,
	.remove = sep_sensor_remove,
};

/*
 * Registered before the device tree gains the sensor node, so the bind happens
 * when the SPI core's notifier creates the device.
 */
int sep_sensor_register(void)
{
	int rc;

	if (sep_registered)
		return 0;
	rc = spi_register_driver(&sep_sensor_driver);
	if (rc)
		return rc;
	sep_registered = true;
	return 0;
}

void sep_sensor_unregister(void)
{
	if (!sep_registered)
		return;
	spi_unregister_driver(&sep_sensor_driver);
	sep_registered = false;
}

static int sep_sensor_bound_locked(void)
{
	return sep_spi != NULL;
}

/* Whether those delays reach hardware, are emulated, or are silently bypassed. */
static int sep_sensor_cs_timing_mode_locked(void)
{
	return sep_cs_timing_mode;
}

/* Power cycle: off, 10 ms, on, 7 ms. -ENODEV if there is no power line. */
static int sep_sensor_power_cycle_locked(void)
{
	if (sep_power_source == SEP_POWER_SMC_RAILS) {
		int rc = sep_smc_set_power(0);

		if (rc)
			return rc;
		msleep(SEP_SENSOR_OFF_DELAY_MS);
		rc = sep_smc_set_power(1);
		if (rc)
			return rc;
		msleep(SEP_SENSOR_ON_DELAY_MS);
		return 0;
	}
	if (!sep_power)
		return -ENODEV;

	gpiod_set_value_cansleep(sep_power, 0);
	msleep(SEP_SENSOR_OFF_DELAY_MS);
	gpiod_set_value_cansleep(sep_power, 1);
	msleep(SEP_SENSOR_ON_DELAY_MS);
	return 0;
}

/* How the power line was obtained: none, the node's property, or chip+line. */
static int sep_sensor_power_source_locked(void)
{
	return sep_power_source;
}

static int sep_sensor_power_line_locked(void)
{
	if (sep_power_source == SEP_POWER_SMC_RAILS)
		return -ENODEV;
	if (!sep_power)
		return -ENODEV;
	return desc_to_gpio(sep_power);
}

/*
 * The per-device calibration blob name, as the board description gives it in
 * the sensor node's "firmware-name" property. NULL when absent, so the caller
 * can fall back to a default.
 */
static const char *sep_sensor_firmware_name_locked(void)
{
	const char *name = NULL;

	if (!sep_spi)
		return NULL;
	if (device_property_read_string(&sep_spi->dev, "firmware-name", &name))
		return NULL;
	return name;
}

/* Powers the sensor on/off, holding the hardware settling delay. */
static int sep_sensor_power_locked(int on)
{
	if (sep_power_source == SEP_POWER_SMC_RAILS) {
		int rc = sep_smc_set_power(on ? 1 : 0);

		if (rc)
			return rc;
		msleep(on ? SEP_SENSOR_ON_DELAY_MS : SEP_SENSOR_OFF_DELAY_MS);
		return 0;
	}
	if (!sep_power)
		return -ENODEV;

	gpiod_set_value_cansleep(sep_power, on ? 1 : 0);
	if (on)
		msleep(SEP_SENSOR_ON_DELAY_MS);
	else
		msleep(SEP_SENSOR_OFF_DELAY_MS);
	return 0;
}

/*
 * Threaded data-ready handler. IRQF_ONESHOT keeps the line masked while this
 * runs, so a level-asserted line cannot storm the CPU; it only wakes the
 * capture loop, which still confirms the sensor state over SPI before reading.
 */
static irqreturn_t sep_drdy_isr(int irq, void *dev_id)
{
	complete(&sep_drdy_done);
	return IRQ_HANDLED;
}

/*
 * Set up the data-ready interrupt once, while the sensor is idle. It is left
 * configured and enabled for the driver's lifetime -- never toggled per capture
 * -- because reconfiguring this line mid-capture is what drops the sensor's
 * firmware patch. Returns 0 when an interrupt is available, -errno otherwise
 * (the caller then falls back to polling).
 */
static int sep_sensor_irq_setup_locked(void)
{
	struct device_node *np;
	struct gpio_device *gdev;
	struct gpio_chip *gc;
	int irq, rc;

	if (!sep_spi)
		return -ENODEV;
	if (sep_drdy_irq >= 0)
		return 0;

	if (sep_spi->irq > 0) {
		/* A board that describes the data-ready line as the SPI node's
		 * interrupt gets it resolved by the SPI core. */
		irq = sep_spi->irq;
	} else if (of_machine_is_compatible("apple,j414s")) {
		np = of_find_node_by_path(SEP_SENSOR_GPIO_NODE);
		if (!np)
			return -ENODEV;
		gdev = gpio_device_find_by_fwnode(of_fwnode_handle(np));
		of_node_put(np);
		if (!gdev)
			return -ENODEV;
		gc = gpio_device_get_chip(gdev);
		if (!gc) {
			gpio_device_put(gdev);
			return -ENODEV;
		}
		sep_drdy = gpiochip_request_own_desc(gc, SEP_SENSOR_DRDY_LINE,
						     "apple-mesa-drdy",
						     GPIO_LOOKUP_FLAGS_DEFAULT,
						     GPIOD_IN);
		gpio_device_put(gdev);
		if (IS_ERR(sep_drdy)) {
			sep_drdy = NULL;
			return -ENODEV;
		}
		irq = gpiod_to_irq(sep_drdy);
		if (irq < 0) {
			gpiochip_free_own_desc(sep_drdy);
			sep_drdy = NULL;
			return irq;
		}
	} else {
		return -ENODEV;
	}

	init_completion(&sep_drdy_done);
	/*
	 * Edge-triggered on the rising edge only. The line idles low, so a level
	 * trigger storms and both-edges fires again on the deassert -- a spurious
	 * wake that, mid-enrolment, spins the capture loop. The data-ready assert
	 * is the low->high edge; the capture loop's status read gates the frame,
	 * and a missed edge only costs one status poll.
	 */
	rc = request_threaded_irq(irq, NULL, sep_drdy_isr,
				  IRQF_ONESHOT | IRQF_TRIGGER_RISING,
				  "apple-mesa-drdy", sep_spi);
	if (rc) {
		if (sep_drdy) {
			gpiochip_free_own_desc(sep_drdy);
			sep_drdy = NULL;
		}
		return rc;
	}
	sep_drdy_irq = irq;
	dev_info(&sep_spi->dev,
		 "sep sensor: data-ready IRQ %d configured for interrupt capture\n",
		 irq);
	return 0;
}

static void sep_sensor_irq_teardown_locked(void)
{
	if (sep_drdy_irq >= 0) {
		free_irq(sep_drdy_irq, sep_spi);
		sep_drdy_irq = -1;
	}
	if (sep_drdy) {
		gpiochip_free_own_desc(sep_drdy);
		sep_drdy = NULL;
	}
}

static int sep_sensor_irq_available_locked(void)
{
	return sep_drdy_irq >= 0;
}

/* Clear any stale signal before a capture so the next wait reflects a fresh
 * data-ready edge, not a leftover from the previous frame. */
static void sep_sensor_irq_arm_locked(void)
{
	if (sep_drdy_irq >= 0)
		reinit_completion(&sep_drdy_done);
}

/*
 * Wait for the data-ready line to assert, up to timeout_ms. 0 = fired,
 * -ETIMEDOUT = no signal (the caller re-polls status over SPI regardless, so a
 * missed interrupt only costs one poll interval), -ENODEV = no interrupt.
 */
static int sep_sensor_irq_wait_locked(unsigned int timeout_ms)
{
	if (sep_drdy_irq < 0)
		return -ENODEV;
	if (wait_for_completion_timeout(&sep_drdy_done,
					msecs_to_jiffies(timeout_ms)) == 0)
		return -ETIMEDOUT;
	return 0;
}

/*
 * One chip-select assertion, `len` clocks, full duplex. The caller supplies all
 * `len` transmit bytes (trailing ones as 0xff) rather than relying on the
 * controller running its transmit buffer dry.
 */
static int sep_sensor_xfer_locked(const void *tx, void *rx, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = len,
		.speed_hz = SEP_SENSOR_HZ,
		.bits_per_word = SEP_SENSOR_BITS,
	};

	if (!sep_spi)
		return -ENODEV;
	if (!len)
		return -EINVAL;

	return spi_sync_transfer(sep_spi, &xfer, 1);
}

/*
 * One chip-select assertion, transmit only, rx_buf NULL (not a discard buffer).
 * A simultaneous receive changes the long transfer's pacing and the sensor
 * rejects the patch blob, so the receive must be absent entirely.
 */
static int sep_sensor_xfer_tx_locked(const void *tx, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = tx,
		.rx_buf = NULL,
		.len = len,
		.speed_hz = SEP_SENSOR_HZ,
		.bits_per_word = SEP_SENSOR_BITS,
	};

	if (!sep_spi)
		return -ENODEV;
	if (!len)
		return -EINVAL;

	return spi_sync_transfer(sep_spi, &xfer, 1);
}

/*
 * Two transfers in one chip-select assertion: command out, then read in.
 * Chip-select stays asserted because neither transfer sets cs_change.
 */
static int sep_sensor_xfer2_locked(const void *tx, size_t tx_len, void *rx, size_t rx_len)
{
	struct spi_transfer xfers[2] = {
		{
			.tx_buf = tx,
			.len = tx_len,
			.speed_hz = SEP_SENSOR_HZ,
			.bits_per_word = SEP_SENSOR_BITS,
		},
		{
			.rx_buf = rx,
			.len = rx_len,
			.speed_hz = SEP_SENSOR_HZ,
			.bits_per_word = SEP_SENSOR_BITS,
		},
	};

	if (!sep_spi)
		return -ENODEV;
	if (!tx_len || !rx_len)
		return -EINVAL;

	return spi_sync_transfer(sep_spi, xfers, 2);
}

int sep_sensor_bound(void)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return 0;
	result = sep_sensor_bound_locked();
	mutex_unlock(&sensor->lock);
	return result;
}

int sep_sensor_cs_timing_mode(void)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return 0;
	result = sep_sensor_cs_timing_mode_locked();
	mutex_unlock(&sensor->lock);
	return result;
}

int sep_sensor_power_cycle(void)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return -ENODEV;
	result = sep_sensor_power_cycle_locked();
	mutex_unlock(&sensor->lock);
	return result;
}

int sep_sensor_power_source(void)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return 0;
	result = sep_sensor_power_source_locked();
	mutex_unlock(&sensor->lock);
	return result;
}

int sep_sensor_power_line(void)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return -ENODEV;
	result = sep_sensor_power_line_locked();
	mutex_unlock(&sensor->lock);
	return result;
}

const char *sep_sensor_firmware_name(void)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	const char *result;

	if (!sensor)
		return NULL;
	result = sep_sensor_firmware_name_locked();
	mutex_unlock(&sensor->lock);
	return result;
}

int sep_sensor_power(int on)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return -ENODEV;
	result = sep_sensor_power_locked(on);
	mutex_unlock(&sensor->lock);
	return result;
}

int sep_sensor_irq_setup(void)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return -ENODEV;
	result = sep_sensor_irq_setup_locked();
	mutex_unlock(&sensor->lock);
	return result;
}

void sep_sensor_irq_teardown(void)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();

	if (!sensor)
		return;
	sep_sensor_irq_teardown_locked();
	mutex_unlock(&sensor->lock);
}

int sep_sensor_irq_available(void)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return 0;
	result = sep_sensor_irq_available_locked();
	mutex_unlock(&sensor->lock);
	return result;
}

void sep_sensor_irq_arm(void)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();

	if (!sensor)
		return;
	sep_sensor_irq_arm_locked();
	mutex_unlock(&sensor->lock);
}

int sep_sensor_irq_wait(unsigned int timeout_ms)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return -ENODEV;
	result = sep_sensor_irq_wait_locked(timeout_ms);
	mutex_unlock(&sensor->lock);
	return result;
}

int sep_sensor_xfer(const void *tx, void *rx, size_t len)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return -ENODEV;
	result = sep_sensor_xfer_locked(tx, rx, len);
	mutex_unlock(&sensor->lock);
	return result;
}

int sep_sensor_xfer_tx(const void *tx, size_t len)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return -ENODEV;
	result = sep_sensor_xfer_tx_locked(tx, len);
	mutex_unlock(&sensor->lock);
	return result;
}

int sep_sensor_xfer2(const void *tx, size_t tx_len, void *rx, size_t rx_len)
{
	struct sep_sensor_lifetime *sensor = sep_lock_sensor();
	int result;

	if (!sensor)
		return -ENODEV;
	result = sep_sensor_xfer2_locked(tx, tx_len, rx, rx_len);
	mutex_unlock(&sensor->lock);
	return result;
}
