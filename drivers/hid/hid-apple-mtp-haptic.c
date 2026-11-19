// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/soc/apple/actuator.h>

#include "hid-ids.h"

int apple_taptic_send(struct hid_device *haptic_hdev, u16 effect_type, u8 strength, u8 softness)
{
	u8 event_click[] = {0x53, 0x01, strength, softness, 0x03, 0x02, 0x22, 0x48, 0x49,
			    0x01, 0x04, 0x60, 0x17, 0x2D, 0x03, 0x02, 0x18, 0x32, 0x1E};
	u8 event_deep_click[] = {0x53, 0x01, strength, softness, 0x01, 0x07, 0x18, 0x18, 0x4F,
				 0x02, 0x02, 0x60, 0x0C, 0x25};
	u8 event_release[] = {0x53, 0x01, strength, softness};
	u8 *msg;
	int msg_length;
	int ret;

	switch (effect_type) {
	case HID_HP_WAVEFORMPRESS & HID_USAGE:
		msg = event_click;
		msg_length = sizeof(event_click);
		break;
	case APPLE_HP_WAVEFORMDEEPCLICK & HID_USAGE:
		msg = event_deep_click;
		msg_length = sizeof(event_deep_click);
		break;
	case HID_HP_WAVEFORMRELEASE & HID_USAGE:
		msg = event_release;
		msg_length = sizeof(event_release);
		break;
	default:
		return -EINVAL;
	}

	ret = hid_hw_raw_request(haptic_hdev, msg[0], msg, msg_length,
				 HID_OUTPUT_REPORT, HID_REQ_SET_REPORT);

	if (ret < 0)
		return ret;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_taptic_send);

static bool apple_taptic_has_report(struct hid_device *hdev, unsigned int type,
				    unsigned int id)
{
	return hdev->report_enum[type].report_id_hash[id] != NULL;
}

#define APPLE_TAPTIC_MODE_REPORT	0x21
#define APPLE_TAPTIC_MODE_TIMEOUT_MS	300

static int apple_taptic_mode_request(struct hid_device *haptic_hdev, u8 *buf,
				     u8 value, int reqtype)
{
	buf[0] = APPLE_TAPTIC_MODE_REPORT;
	buf[1] = value;
	return hid_hw_raw_request(haptic_hdev, APPLE_TAPTIC_MODE_REPORT, buf, 2,
				  HID_FEATURE_REPORT, reqtype);
}

/*
 * Set the mode and confirm it by reading it back.
 *
 * A handoff asked for while the firmware is playing its own click waveform
 * is queued: the report reads 02 until the waveform ends, then the new mode
 * (J413: up to 160 ms). The firmware completes a queued handoff on its own,
 * so giving up on it without cancelling would leave the actuator
 * host-controlled with nobody driving it: no click, and no button.
 */
static int apple_taptic_write_mode(struct hid_device *haptic_hdev, bool host)
{
	u8 *buf __free(kfree) = kzalloc(2, GFP_KERNEL);
	unsigned long timeout;
	int ret;

	if (!buf)
		return -ENOMEM;

	ret = apple_taptic_mode_request(haptic_hdev, buf, host,
					HID_REQ_SET_REPORT);
	if (ret < 0)
		return ret;

	timeout = jiffies + msecs_to_jiffies(APPLE_TAPTIC_MODE_TIMEOUT_MS);
	for (;;) {
		ret = apple_taptic_mode_request(haptic_hdev, buf, 0xff,
						HID_REQ_GET_REPORT);
		if (ret == 2 && buf[1] == host)
			return 0;
		if (ret < 0 || time_after(jiffies, timeout))
			break;
		usleep_range(4000, 6000);
	}

	hid_warn(haptic_hdev, "mode %u not confirmed (%d, reads %02x)\n",
		 host, ret, buf[1]);

	/* Do not leave a queued handoff behind; keep the firmware's click. */
	if (host)
		apple_taptic_mode_request(haptic_hdev, buf, 0,
					  HID_REQ_SET_REPORT);

	return ret < 0 ? ret : -EIO;
}

int apple_taptic_switch_modes(struct hid_device *haptic_hdev, bool enable_host_controlled)
{
	/*
	 * Report 0x21 hands click feedback and the button to the host (1) or
	 * back to the firmware (0).
	 *
	 * The MTP actuators do not declare it, but their firmware takes it as
	 * a feature report and reads it back (J413: "Intercepted 0x21 click
	 * handoff" in the RTKit log).
	 *
	 * The J313 SPI actuator declares only input 0x3f and output 0x53, and
	 * its controller answers 0x21 with status 9c 17 e5 f8. That part powers
	 * up device-controlled, so asking for that mode needs nothing sent.
	 * Asking for host mode cannot be honoured: report it, because a caller
	 * told it succeeded plays its own click over the firmware's and every
	 * press clicks twice.
	 */
	if (haptic_hdev->bus != BUS_HOST &&
	    !apple_taptic_has_report(haptic_hdev, HID_FEATURE_REPORT,
				     APPLE_TAPTIC_MODE_REPORT) &&
	    !apple_taptic_has_report(haptic_hdev, HID_OUTPUT_REPORT,
				     APPLE_TAPTIC_MODE_REPORT))
		return enable_host_controlled ? -EOPNOTSUPP : 0;

	return apple_taptic_write_mode(haptic_hdev, enable_host_controlled);
}
EXPORT_SYMBOL_GPL(apple_taptic_switch_modes);

static int apple_actuator_probe(struct hid_device *haptic_hdev, const struct hid_device_id *id)
{
	int ret;

	ret = hid_parse(haptic_hdev);
	if (ret) {
		hid_err(haptic_hdev, "hid parse failed\n");
		return ret;
	}

	ret = hid_hw_start(haptic_hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(haptic_hdev, "hw start failed\n");
		return ret;
	}

	/*
	 * The actuator keeps its mode across a warm reboot, and every user of
	 * it starts out believing it is device-controlled. One left in
	 * host-controlled mode by a previous boot is silent: its firmware plays
	 * nothing, and the host that was driving it is gone. Assert the state
	 * the drivers assume rather than inheriting the last one. A failure
	 * here is not fatal; it only means the actuator keeps whatever mode it
	 * had, which is what would have happened anyway.
	 */
	ret = apple_taptic_switch_modes(haptic_hdev, false);
	if (ret)
		hid_warn(haptic_hdev,
			 "cannot return the actuator to device-controlled mode (%d)\n",
			 ret);

	return 0;
}

static bool actuator_match(struct hid_device *hdev, bool ignore_special_drivers)
{
	return apple_taptic_is_actuator(hdev);
}

static void actuator_remove(struct hid_device *haptic_hdev)
{
	apple_taptic_switch_modes(haptic_hdev, false);
	hid_hw_stop(haptic_hdev);
}

static const struct hid_device_id apple_haptic_devices[] = {
	{ HID_DEVICE(BUS_HOST, HID_GROUP_ANY, HOST_VENDOR_ID_APPLE,
		     HID_ANY_ID), .driver_data = 0 },
	{ HID_DEVICE(BUS_SPI, HID_GROUP_ANY, SPI_VENDOR_ID_APPLE,
		     HID_ANY_ID), .driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(hid, apple_haptic_devices);

static struct hid_driver apple_actuator_driver = {
	.name = "hid-apple-mtp-haptic",
	.id_table = apple_haptic_devices,
	.probe = apple_actuator_probe,
	.match = actuator_match,
	.remove = actuator_remove
};
module_hid_driver(apple_actuator_driver);

MODULE_DESCRIPTION("Apple MTP Haptic Actuator driver");
MODULE_LICENSE("GPL");
