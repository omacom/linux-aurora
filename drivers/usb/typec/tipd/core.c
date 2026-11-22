// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for TI TPS6598x USB Power Delivery controller family
 *
 * Copyright (C) 2017, Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include <kunit/static_stub.h>
#include <kunit/visibility.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/soc/apple/j613-display.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/reboot.h>
#include <linux/unaligned.h>
#include <linux/usb/pd.h>
#include <linux/soc/apple/sleep.h>
#include <linux/suspend.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_tbt.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <drm/drm_connector.h>

#include "cd321x-dp.h"
#include "tps6598x.h"
#include "cd321x-setup.h"
#include "trace.h"

/* Register offsets */
#define TPS_REG_VID			0x00
#define TPS_REG_CMD1			0x08
#define TPS_REG_DATA1			0x09
#define TPS_REG_VERSION			0x0F
#define TPS_REG_INT_EVENT1		0x14
#define TPS_REG_INT_EVENT2		0x15
#define TPS_REG_INT_MASK1		0x16
#define TPS_REG_INT_MASK2		0x17
#define TPS_REG_INT_CLEAR1		0x18
#define TPS_REG_INT_CLEAR2		0x19
#define TPS_REG_STATUS			0x1a
#define TPS_REG_SYSTEM_POWER_STATE	0x20
#define TPS_REG_USB4_STATUS		0x24
#define TPS_REG_SYSTEM_CONF		0x28
#define TPS_REG_CTRL_CONF		0x29
#define TPS_REG_BOOT_STATUS		0x2D
#define TPS_REG_RX_SOURCE_CAPS		0x30
#define TPS_REG_POWER_STATUS		0x3f
#define TPS_REG_PD_STATUS		0x40
#define TPS_REG_RX_IDENTITY_SOP		0x48
#define TPS_REG_CF_VID_STATUS		0x5e
#define TPS_REG_DP_SID_STATUS		0x58
#define TPS_REG_INTEL_VID_STATUS	0x59
#define TPS_REG_DATA_STATUS		0x5f
#define TPS_REG_SLEEP_CONF		0x70

/* TPS_REG_SYSTEM_CONF bits */
#define TPS_SYSCONF_PORTINFO(c)		((c) & 7)

/*
 * BPMs task timeout, recommended 5 seconds
 * pg.48 TPS2575 Host Interface Technical Reference
 * Manual (Rev. A)
 * https://www.ti.com/lit/ug/slvuc05a/slvuc05a.pdf
 */
#define TPS_BUNDLE_TIMEOUT	0x32

/* BPMs return code */
#define TPS_TASK_BPMS_INVALID_BUNDLE_SIZE	0x4
#define TPS_TASK_BPMS_INVALID_SLAVE_ADDR	0x5
#define TPS_TASK_BPMS_INVALID_TIMEOUT		0x6

/* PBMc data out */
#define TPS_PBMC_RC	0 /* Return code */
#define TPS_PBMC_DPCS	2 /* device patch complete status */

/* reset de-assertion to ready for operation */
#define TPS_SETUP_MS			1000

enum {
	TPS_PORTINFO_SINK,
	TPS_PORTINFO_SINK_ACCESSORY,
	TPS_PORTINFO_DRP_UFP,
	TPS_PORTINFO_DRP_UFP_DRD,
	TPS_PORTINFO_DRP_DFP,
	TPS_PORTINFO_DRP_DFP_DRD,
	TPS_PORTINFO_SOURCE,
};

/* TPS_REG_RX_IDENTITY_SOP */
struct tps6598x_rx_identity_reg {
	u8 status;
	struct usb_pd_identity identity;
} __packed;

/* Debounce delay for mode changes, in milliseconds */
#define CD321X_DEBOUNCE_DELAY_MS 500

enum {
	TPS_MODE_APP,
	TPS_MODE_BOOT,
	TPS_MODE_BIST,
	TPS_MODE_DISC,
	TPS_MODE_PTCH,
	TPS_MODE_APP1,
	TPS_MODE_DBMA,
};

static const char *const modes[] = {
	[TPS_MODE_APP]	= "APP ",
	[TPS_MODE_BOOT]	= "BOOT",
	[TPS_MODE_BIST]	= "BIST",
	[TPS_MODE_DISC]	= "DISC",
	[TPS_MODE_PTCH] = "PTCH",
	[TPS_MODE_APP1]	= "APP1",
	[TPS_MODE_DBMA] = "DBMa",
};

/* Unrecognized commands will be replaced with "!CMD" */
#define INVALID_CMD(_cmd_)		(_cmd_ == 0x444d4321)

static enum power_supply_property tps6598x_psy_props[] = {
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
};

static const char *tps6598x_psy_name_prefix = "tps6598x-source-psy-";

int tps6598x_block_read(struct tps6598x *tps, u8 reg, void *val, size_t len)
{
	u8 data[TPS_MAX_LEN + 1];
	int ret;

	if (len + 1 > sizeof(data))
		return -EINVAL;

	if (!tps->i2c_protocol)
		return regmap_raw_read(tps->regmap, reg, val, len);

	ret = regmap_raw_read(tps->regmap, reg, data, len + 1);
	if (ret)
		return ret;

	if (data[0] < len)
		return -EIO;

	memcpy(val, &data[1], len);
	return 0;
}
EXPORT_SYMBOL_GPL(tps6598x_block_read);

static int tps6598x_block_write(struct tps6598x *tps, u8 reg,
				const void *val, size_t len)
{
	u8 data[TPS_MAX_LEN + 1];

	if (len + 1 > sizeof(data))
		return -EINVAL;

	if (!tps->i2c_protocol)
		return regmap_raw_write(tps->regmap, reg, val, len);

	data[0] = len;
	memcpy(&data[1], val, len);

	return regmap_raw_write(tps->regmap, reg, data, len + 1);
}

static inline int tps6598x_read8(struct tps6598x *tps, u8 reg, u8 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u8));
}

static inline int tps6598x_read16(struct tps6598x *tps, u8 reg, u16 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u16));
}

static inline int tps6598x_read32(struct tps6598x *tps, u8 reg, u32 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u32));
}

static inline int tps6598x_read64(struct tps6598x *tps, u8 reg, u64 *val)
{
	return tps6598x_block_read(tps, reg, val, sizeof(u64));
}

static inline int tps6598x_write8(struct tps6598x *tps, u8 reg, u8 val)
{
	return tps6598x_block_write(tps, reg, &val, sizeof(u8));
}

static inline int tps6598x_write64(struct tps6598x *tps, u8 reg, u64 val)
{
	return tps6598x_block_write(tps, reg, &val, sizeof(u64));
}

static inline int
tps6598x_write_4cc(struct tps6598x *tps, u8 reg, const char *val)
{
	return tps6598x_block_write(tps, reg, val, 4);
}

static int tps6598x_read_partner_identity(struct tps6598x *tps)
{
	struct tps6598x_rx_identity_reg id;
	int ret;

	ret = tps6598x_block_read(tps, TPS_REG_RX_IDENTITY_SOP,
				  &id, sizeof(id));
	if (ret)
		return ret;

	tps->partner_identity = id.identity;

	return 0;
}

static int tps6598x_set_data_role(struct tps6598x *tps,
				  enum typec_data_role role, bool connected)
{
	enum usb_role role_val;
	int ret;

	if (role == TYPEC_HOST)
		role_val = USB_ROLE_HOST;
	else
		role_val = USB_ROLE_DEVICE;

	if (!connected)
		role_val = USB_ROLE_NONE;

	ret = usb_role_switch_set_role(tps->role_sw, role_val);
	/* An unregistered role switch has already stopped its controller. */
	if (ret && (connected || ret != -EOPNOTSUPP ||
		    usb_role_switch_get_role(tps->role_sw) != USB_ROLE_NONE))
		return ret;
	typec_set_data_role(tps->port, role);
	return 0;
}

static int tps6598x_connect(struct tps6598x *tps, u32 status)
{
	struct typec_partner_desc desc = { };
	enum typec_pwr_opmode mode;
	int ret;

	if (tps->partner)
		return 0;

	mode = TPS_POWER_STATUS_PWROPMODE(tps->pwr_status);

	desc.usb_pd = mode == TYPEC_PWR_MODE_PD;
	desc.accessory = TYPEC_ACCESSORY_NONE; /* XXX: handle accessories */

	if (desc.usb_pd) {
		ret = tps6598x_read_partner_identity(tps);
		if (ret)
			return ret;
		desc.identity = &tps->partner_identity;
	}

	typec_set_pwr_opmode(tps->port, mode);
	typec_set_pwr_role(tps->port, TPS_STATUS_TO_TYPEC_PORTROLE(status));
	typec_set_vconn_role(tps->port, TPS_STATUS_TO_TYPEC_VCONN(status));
	if (TPS_STATUS_TO_UPSIDE_DOWN(status))
		typec_set_orientation(tps->port, TYPEC_ORIENTATION_REVERSE);
	else
		typec_set_orientation(tps->port, TYPEC_ORIENTATION_NORMAL);
	typec_set_mode(tps->port, TYPEC_STATE_USB);
	ret = tps6598x_set_data_role(tps, TPS_STATUS_TO_TYPEC_DATAROLE(status), true);
	if (ret)
		return ret;

	tps->partner = typec_register_partner(tps->port, &desc);
	if (IS_ERR(tps->partner)) {
		ret = PTR_ERR(tps->partner);
		tps->partner = NULL;
		return ret;
	}

	if (desc.identity)
		typec_partner_set_identity(tps->partner);

	power_supply_changed(tps->psy);

	return 0;
}

static void tps6598x_drop_partner(struct tps6598x *tps, u32 status)
{
	if (tps->partner_altmode_dp) {
		typec_altmode_update_active(tps->partner_altmode_dp, false);
		typec_unregister_altmode(tps->partner_altmode_dp);
		tps->partner_altmode_dp = NULL;
	}
	if (!IS_ERR(tps->partner))
		typec_unregister_partner(tps->partner);
	tps->partner = NULL;
	typec_set_pwr_opmode(tps->port, TYPEC_PWR_MODE_USB);
	typec_set_pwr_role(tps->port, TPS_STATUS_TO_TYPEC_PORTROLE(status));
	typec_set_vconn_role(tps->port, TPS_STATUS_TO_TYPEC_VCONN(status));
}

/* The mux provider knows whether this is a fixed host or a switched PHY. */
static bool tps6598x_safe_after_role(struct tps6598x *tps)
{
	return tps->role_sw && typec_mux_safe_after_role(tps->mux);
}

static int tps6598x_disconnect(struct tps6598x *tps, u32 status)
{
	struct typec_mux_state safe = { .mode = TYPEC_STATE_SAFE };
	bool prepared = tps->remove_safe_prepared;
	int ret;

	tps->remove_safe_prepared = false;
	tps6598x_drop_partner(tps, status);
	if (!prepared) {
		typec_set_orientation(tps->port, TYPEC_ORIENTATION_NONE);
		ret = typec_mux_begin(tps->mux, &safe);
		if (ret)
			return ret;
		/* Leave shared PHY shutdown to its USB role consumer. */
		if (!tps6598x_safe_after_role(tps)) {
			ret = typec_mux_apply(tps->mux);
			if (ret)
				return ret;
		}
	}
	ret = tps6598x_set_data_role(tps, TPS_STATUS_TO_TYPEC_DATAROLE(status), false);
	if (ret) {
		/* Keep recovery ownership; a live PHY must not receive SAFE. */
		typec_mux_abort(tps->mux);
		return ret;
	}
	ret = typec_mux_end(tps->mux);
	if (ret)
		return ret;

	power_supply_changed(tps->psy);
	return 0;
}

/*
 * The sleep's rules for a port controller's system power state command (the controller is
 * woken first, the command's end is read at intervals, a failed S3 refuses the sleep, the
 * command's limit and its lines), for a controller whose DT asks for them
 * (apple,sleep-ssps-rules: the J714s's, J716c's, J813's and J815's). Others keep the original
 * command behaviour.
 */
static bool tipd_sleep_ssps_rules(struct tps6598x *tps)
{
	return device_property_read_bool(tps->dev, "apple,sleep-ssps-rules");
}

/*
 * With a gap, the command register is read once per gap until the command has ended, the first
 * time a quarter of a gap after the command was written, and the caller may sleep. Without one
 * it is read back to back, as before.
 */
static int tps6598x_exec_cmd_gap(struct tps6598x *tps, const char *cmd,
				 size_t in_len, const u8 *in_data,
				 size_t out_len, u8 *out_data,
				 u32 cmd_timeout_ms, u32 res_delay_ms, u32 read_gap_us)
{
	unsigned long timeout, note;
	bool first = true;
	u32 val;
	int ret;

	ret = tps6598x_read32(tps, TPS_REG_CMD1, &val);
	if (ret)
		return ret;
	if (val && !INVALID_CMD(val))
		return -EBUSY;

	if (in_len) {
		ret = tps6598x_block_write(tps, TPS_REG_DATA1,
					   in_data, in_len);
		if (ret)
			return ret;
	}

	ret = tps6598x_write_4cc(tps, TPS_REG_CMD1, cmd);
	if (ret < 0)
		return ret;

	timeout = jiffies + msecs_to_jiffies(cmd_timeout_ms);
	note = jiffies + HZ;

	do {
		if (read_gap_us) {
			u32 gap = first ? read_gap_us / 4 : read_gap_us;

			usleep_range(gap, gap + gap / 4);
			first = false;
		}
		ret = tps6598x_read32(tps, TPS_REG_CMD1, &val);
		if (ret)
			return ret;
		if (INVALID_CMD(val))
			return -EINVAL;

		/*
		 * Under the sleep's rules a command that has ended has ended, whatever the time
		 * its last read was made at. Elsewhere a read made past the limit is a timeout,
		 * as before.
		 */
		if ((val || !tipd_sleep_ssps_rules(tps)) && time_is_before_jiffies(timeout))
			return -ETIMEDOUT;
		/* A command with a long limit is not waited for in silence: a line each second */
		if (val && cmd_timeout_ms > 1000 && time_is_before_jiffies(note) &&
		    tipd_sleep_ssps_rules(tps)) {
			dev_info(tps->dev, "command %.4s not done after %u ms (limit %u ms)\n", cmd,
				 jiffies_to_msecs(jiffies - (timeout - msecs_to_jiffies(cmd_timeout_ms))),
				 cmd_timeout_ms);
			note += HZ;
		}
	} while (val);

	/* some commands require delay for the result to be available */
	mdelay(res_delay_ms);

	if (out_len) {
		ret = tps6598x_block_read(tps, TPS_REG_DATA1,
					  out_data, out_len);
		if (ret)
			return ret;
		val = out_data[0];
	} else {
		ret = tps6598x_block_read(tps, TPS_REG_DATA1, &val, sizeof(u8));
		if (ret)
			return ret;
	}

	switch (val) {
	case TPS_TASK_TIMEOUT:
		return -ETIMEDOUT;
	case TPS_TASK_REJECTED:
		return -EPERM;
	default:
		break;
	}

	return 0;
}

int tps6598x_exec_cmd_tmo(struct tps6598x *tps, const char *cmd,
			  size_t in_len, const u8 *in_data,
			  size_t out_len, u8 *out_data,
			  u32 cmd_timeout_ms, u32 res_delay_ms)
{
	return tps6598x_exec_cmd_gap(tps, cmd, in_len, in_data, out_len, out_data,
				     cmd_timeout_ms, res_delay_ms, 0);
}
EXPORT_SYMBOL_GPL(tps6598x_exec_cmd_tmo);

static int tps6598x_exec_cmd(struct tps6598x *tps, const char *cmd,
			     size_t in_len, const u8 *in_data,
			     size_t out_len, u8 *out_data)
{
	return tps6598x_exec_cmd_tmo(tps, cmd, in_len, in_data,
				     out_len, out_data, 1000, 0);
}

static int tps6598x_dr_set(struct typec_port *port, enum typec_data_role role)
{
	const char *cmd = (role == TYPEC_DEVICE) ? "SWUF" : "SWDF";
	struct tps6598x *tps = typec_get_drvdata(port);
	u32 status;
	int ret;

	/*
	 * On the J613 SN201202x, SWDF/SWUF can stay busy forever. Keep the
	 * negotiated data role rather than wedge the command mailbox.
	 */
	if (of_machine_is_compatible("apple,j613") &&
	    device_is_compatible(tps->dev, "apple,sn201202x"))
		return -EOPNOTSUPP;

	mutex_lock(&tps->lock);

	ret = tps6598x_exec_cmd(tps, cmd, 0, NULL, 0, NULL);
	if (ret)
		goto out_unlock;

	ret = tps6598x_read32(tps, TPS_REG_STATUS, &status);
	if (ret)
		goto out_unlock;

	if (role != TPS_STATUS_TO_TYPEC_DATAROLE(status)) {
		ret = -EPROTO;
		goto out_unlock;
	}

	ret = tps6598x_set_data_role(tps, role, true);

out_unlock:
	mutex_unlock(&tps->lock);

	return ret;
}

/*
 * After a PR_Swap is accepted, the initial sink waits up to tPSSourceOff
 * (920 ms) for the initial source's PS_RDY, and the initial source then
 * waits up to tPSSourceOn (480 ms) for the new source's PS_RDY (USB Power
 * Delivery, PR_Swap timers).
 */
#define TPS_PR_SWAP_TIMEOUT_US	(1400 * USEC_PER_MSEC)
#define TPS_PR_SWAP_POLL_US	(10 * USEC_PER_MSEC)

static int tps6598x_pr_set(struct typec_port *port, enum typec_role role)
{
	const char *cmd = (role == TYPEC_SINK) ? "SWSk" : "SWSr";
	struct tps6598x *tps = typec_get_drvdata(port);
	u32 status;
	int ret, err;

	mutex_lock(&tps->lock);

	ret = tps6598x_exec_cmd(tps, cmd, 0, NULL, 0, NULL);
	if (ret)
		goto out_unlock;

	/*
	 * The task completes once the swap has been accepted, but the port
	 * role in STATUS changes only after the power supply transition that
	 * follows, so wait for it before calling the swap a failure.
	 */
	err = read_poll_timeout(tps6598x_read32, ret,
				ret || TPS_STATUS_TO_TYPEC_PORTROLE(status) == role,
				TPS_PR_SWAP_POLL_US, TPS_PR_SWAP_TIMEOUT_US, false,
				tps, TPS_REG_STATUS, &status);
	if (ret)
		goto out_unlock;
	if (err) {
		ret = -EPROTO;
		goto out_unlock;
	}

	typec_set_pwr_role(tps->port, role);

out_unlock:
	mutex_unlock(&tps->lock);

	return ret;
}

static const struct typec_operations tps6598x_ops = {
	.dr_set = tps6598x_dr_set,
	.pr_set = tps6598x_pr_set,
};

static bool tps6598x_read_status(struct tps6598x *tps, u32 *status)
{
	int ret;

	ret = tps6598x_read32(tps, TPS_REG_STATUS, status);
	if (ret) {
		dev_err(tps->dev, "%s: failed to read status\n", __func__);
		return false;
	}

	if (tps->data->trace_status)
		tps->data->trace_status(*status);

	return true;
}

static bool tps6598x_read_data_status(struct tps6598x *tps)
{
	u32 data_status;
	int ret;

	ret = tps6598x_read32(tps, TPS_REG_DATA_STATUS, &data_status);
	if (ret < 0) {
		dev_err(tps->dev, "failed to read data status: %d\n", ret);
		return false;
	}
	tps->data_status = data_status;

	if (tps->data->trace_data_status)
		tps->data->trace_data_status(data_status);

	return true;
}

static bool cd321x_read_data_status(struct tps6598x *tps)
{
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);
	int ret;

	ret = tps6598x_read_data_status(tps);
	if (!ret)
		return false;

	if (tps->data_status & TPS_DATA_STATUS_DP_CONNECTION) {
		ret = tps6598x_block_read(tps, TPS_REG_DP_SID_STATUS,
				&cd321x->dp_sid_status, sizeof(cd321x->dp_sid_status));
		if (ret) {
			dev_err(tps->dev, "Failed to read DP SID Status: %d\n",
				ret);
			return false;
		}
	}

	if (tps->data_status & TPS_DATA_STATUS_TBT_CONNECTION) {
		ret = tps6598x_block_read(tps, TPS_REG_INTEL_VID_STATUS,
				&cd321x->intel_vid_status, sizeof(cd321x->intel_vid_status));
		if (ret) {
			dev_err(tps->dev, "Failed to read Intel VID Status: %d\n", ret);
			return false;
		}
	}

	if (tps->data_status & CD321X_DATA_STATUS_USB4_CONNECTION) {
		ret = tps6598x_block_read(tps, TPS_REG_USB4_STATUS,
				&cd321x->usb4_status, sizeof(cd321x->usb4_status));
		if (ret) {
			dev_err(tps->dev,
				"Failed to read USB4 Status: %d\n", ret);
			return false;
		}
	}

	return true;
}

static bool tps6598x_read_power_status(struct tps6598x *tps)
{
	u16 pwr_status;
	int ret;

	ret = tps6598x_read16(tps, TPS_REG_POWER_STATUS, &pwr_status);
	if (ret < 0) {
		dev_err(tps->dev, "failed to read power status: %d\n", ret);
		return false;
	}
	tps->pwr_status = pwr_status;

	if (tps->data->trace_power_status)
		tps->data->trace_power_status(pwr_status);

	return true;
}

/*
 * TPS66993 deprecated Power_Status register (0x3F). BC1.2 is not supported
 * and the remaining bits are redundant with STATUS register (0x1A).
 * Synthesize pwr_status from the already-read STATUS register.
 */
static bool tps66993_read_power_status(struct tps6598x *tps)
{
	u16 pwr_status = 0;

	/* Same masks as TPS_POWER_STATUS_CONNECTION() / SOURCESINK() / PWROPMODE() in tps6598x.h */
	if (tps->status & TPS_STATUS_PLUG_PRESENT)
		pwr_status |= FIELD_PREP(TPS_POWER_STATUS_CONNECTION_MASK, 1);

	/* SOURCESINK: 1=sink; STATUS.PortRole 1=source, opposite convention */
	if (!TPS_STATUS_TO_TYPEC_PORTROLE(tps->status))
		pwr_status |= FIELD_PREP(TPS_POWER_STATUS_SOURCESINK_MASK, 1);

	if (TPS_STATUS_VBUS_STATUS(tps->status) == TPS_STATUS_VBUS_STATUS_PD)
		pwr_status |= FIELD_PREP(TPS_POWER_STATUS_TYPEC_CURRENT_MASK,
					 TPS_POWER_STATUS_TYPEC_CURRENT_PD);

	tps->pwr_status = pwr_status;

	tps->data->trace_power_status(pwr_status);

	return true;
}

static void tps6598x_handle_plug_event(struct tps6598x *tps, u32 status)
{
	int ret;

	if (status & TPS_STATUS_PLUG_PRESENT) {
		ret = tps6598x_connect(tps, status);
		if (ret)
			dev_err(tps->dev, "failed to register partner\n");
	} else {
		ret = tps6598x_disconnect(tps, status);
		if (ret)
			dev_err(tps->dev, "hardware disconnect failed: %d\n", ret);
	}
}

/* The mux held the change up, see cd321x_pm_mux_held(). */
static bool cd321x_mux_held(struct cd321x *cd321x, int ret)
{
	return ret == -ETIMEDOUT && typec_mux_timeout_transient(cd321x->mux);
}

/* Set the mux to the mode state, telling whether a provider held it up */
static int cd321x_mux_set(struct cd321x *cd321x, bool *held)
{
	int ret = typec_mux_set(cd321x->mux, &cd321x->state);

	*held = cd321x_mux_held(cd321x, ret);
	return ret;
}

/*
 * Report a change the mux failed. A mux that @held the change up is retried
 * (see cd321x_retry_held_mux()), and the hold-up is a fault worth a warning
 * only once the retries end, as cd321x_mux_gave_up() reports. Any other
 * failure is a fault now.
 */
VISIBLE_IF_KUNIT void cd321x_mux_failed(struct cd321x *cd321x, const char *what, int ret,
					bool held)
{
	KUNIT_STATIC_STUB_REDIRECT(cd321x_mux_failed, cd321x, what, ret, held);

	if (held)
		dev_dbg(cd321x->tps.dev, "%s, still held up: %d\n", what, ret);
	else
		dev_warn(cd321x->tps.dev, "%s: %d\n", what, ret);
}
EXPORT_SYMBOL_IF_KUNIT(cd321x_mux_failed);

/* The retries of a change that a mux held up have ended (see cd321x_pm_mux_held()). */
VISIBLE_IF_KUNIT void cd321x_mux_gave_up(struct cd321x *cd321x, unsigned int retries,
					 unsigned int held_ms)
{
	KUNIT_STATIC_STUB_REDIRECT(cd321x_mux_gave_up, cd321x, retries, held_ms);

	dev_warn(cd321x->tps.dev,
		 "Type-C mux still held up after %u retries in %u s, waiting for the next event\n",
		 retries, held_ms / 1000);
}
EXPORT_SYMBOL_IF_KUNIT(cd321x_mux_gave_up);

/*
 * The HPD level of a DisplayPort partner. DATA_STATUS reports it. An
 * SN201202x also signals every new DP status of the partner, and a DP-to-HDMI
 * converter can keep DATA_STATUS at HPD high while the display behind it is
 * unplugged, and report that only in its DP status. Once the controller has
 * signalled a DP status for this connection, that status must show HPD too.
 */
static bool cd321x_uses_dp_sid(const struct tps6598x *tps)
{
	return device_is_compatible(tps->dev, "apple,sn201202x") &&
	       !of_machine_is_compatible("apple,t8140") &&
	       !of_machine_is_compatible("apple,t8142");
}

static u64 cd321x_irq_mask(const struct tps6598x *tps)
{
	u64 mask = tps->data->irq_mask1;

	if (device_is_compatible(tps->dev, "apple,sn201202x") &&
	    !cd321x_uses_dp_sid(tps))
		mask &= ~SN201202X_INT_DP_SID_UPDATE;
	return mask;
}

static bool cd321x_dp_hpd(const struct cd321x *cd321x, const struct cd321x_status *st)
{
	if (!(st->data_status & CD321X_DATA_STATUS_HPD_LEVEL))
		return false;
	return !cd321x->dp_sid_valid ||
	       (le32_to_cpu(st->dp_sid_status.status_rx) & DP_STATUS_HPD_STATE);
}

/*
 * SN201202x, from the interrupt: the partner sent a new DP status. Note what
 * it shows for the next update, which may come after several of these: HPD
 * low (the display behind a converter went away, and maybe came back before
 * the update), or IRQ_HPD with HPD high (the sink asks the source to look).
 */
static void cd321x_dp_sid_event(struct cd321x *cd321x)
{
	u32 rx = le32_to_cpu(cd321x->dp_sid_status.status_rx);

	if (!cd321x_uses_dp_sid(&cd321x->tps) ||
	    !(cd321x->tps.data_status & TPS_DATA_STATUS_DP_CONNECTION))
		return;
	cd321x->update_status.dp_sid_event = true;
	if (!(rx & DP_STATUS_HPD_STATE))
		cd321x->update_status.dp_sid_hpd_low = true;
	else if (rx & DP_STATUS_IRQ_HPD)
		cd321x->update_status.dp_sid_irq = true;
	dev_dbg(cd321x->tps.dev, "DP status %#x\n", rx);
}

static int cd321x_typec_update_mode(struct tps6598x *tps,
				    struct cd321x_status *st, bool *display_busy,
				    bool *held)
{
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);
	struct typec_mux_state old_state = cd321x->state;
	struct typec_thunderbolt_switch_data tbt_switch_data = {
		.state = TYPEC_THUNDERBOLT_SWITCH_OFF,
	};
	int ret = 0;

	*display_busy = false;
	*held = false;
	if (!(st->data_status & TPS_DATA_STATUS_DATA_CONNECTION)) {
		if (cd321x->state_valid && cd321x->state.mode == TYPEC_STATE_SAFE)
			return 0;
		cd321x->state.alt = NULL;
		cd321x->state.mode = TYPEC_STATE_SAFE;
		cd321x->state.data = NULL;
		ret = typec_thunderbolt_switch_set(cd321x->tbt_switch, &tbt_switch_data);
		if (ret)
			goto out;
		ret = cd321x_mux_set(cd321x, held);
	} else if (st->data_status & CD321X_DATA_STATUS_USB4_CONNECTION) {
		struct enter_usb_data eusb_data;

		if (cd321x->state_valid && !cd321x->state.alt &&
		    cd321x->state.mode == TYPEC_MODE_USB4)
			return 0;

		eusb_data.eudo = le32_to_cpu(st->usb4_status.eudo);
		eusb_data.active_link_training =
			!!(st->data_status & TPS_DATA_STATUS_ACTIVE_LINK_TRAIN);

		cd321x->state.alt = NULL;
		cd321x->state.data = &eusb_data;
		cd321x->state.mode = TYPEC_MODE_USB4;
		ret = cd321x_mux_set(cd321x, held);
		if (ret)
			goto out;

		tbt_switch_data.state = TYPEC_THUNDERBOLT_SWITCH_USB4;
		tbt_switch_data.usb4 = eusb_data;
		tbt_switch_data.orientation = TPS_STATUS_TO_UPSIDE_DOWN(st->status) ?
						      TYPEC_ORIENTATION_REVERSE :
						      TYPEC_ORIENTATION_NORMAL;
		ret = typec_thunderbolt_switch_set(cd321x->tbt_switch, &tbt_switch_data);
	} else if (st->data_status & TPS_DATA_STATUS_TBT_CONNECTION) {
		struct typec_thunderbolt_data tbt_data;

		if (cd321x->state_valid && cd321x->state.alt == cd321x->port_altmode_tbt &&
		    cd321x->state.mode == TYPEC_TBT_MODE)
			return 0;

		tbt_data.cable_mode = TBT_MODE |
			TBT_SET_CABLE_SPEED(TPS_DATA_STATUS_TBT_CABLE_SPEED(st->data_status)) |
			TBT_SET_CABLE_ROUNDED(TPS_DATA_STATUS_TBT_CABLE_GEN(st->data_status));
		if (st->data_status & TPS_DATA_STATUS_OPTICAL_CABLE)
			tbt_data.cable_mode |= TBT_CABLE_OPTICAL;
		if (st->data_status & TPS_DATA_STATUS_ACTIVE_LINK_TRAIN)
			tbt_data.cable_mode |= TBT_CABLE_LINK_TRAINING;
		if (st->data_status & CD321X_DATA_STATUS_ACTIVE_CABLE)
			tbt_data.cable_mode |= TBT_CABLE_ACTIVE_PASSIVE;
		tbt_data.device_mode = TBT_MODE |
			(u32)le16_to_cpu(st->intel_vid_status.device_mode) << 16;
		tbt_data.enter_vdo =
			(u32)le16_to_cpu(st->intel_vid_status.enter_vdo) << 16;
		cd321x->state.alt = cd321x->port_altmode_tbt;
		cd321x->state.mode = TYPEC_TBT_MODE;
		cd321x->state.data = &tbt_data;
		ret = cd321x_mux_set(cd321x, held);
		if (ret)
			goto out;

		tbt_switch_data.state = TYPEC_THUNDERBOLT_SWITCH_TBT;
		tbt_switch_data.tbt = tbt_data;
		tbt_switch_data.orientation = TPS_STATUS_TO_UPSIDE_DOWN(st->status) ?
						      TYPEC_ORIENTATION_REVERSE :
						      TYPEC_ORIENTATION_NORMAL;
		ret = typec_thunderbolt_switch_set(cd321x->tbt_switch, &tbt_switch_data);
	} else if (st->data_status & TPS_DATA_STATUS_DP_CONNECTION) {
		struct typec_displayport_data dp_data;
		enum cd321x_dp_send send;
		unsigned long mode;
		bool hpd_pulse;

		switch (TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT(st->data_status)) {
		case TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_A:
			mode = TYPEC_DP_STATE_A;
			break;
		case TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_B:
			mode = TYPEC_DP_STATE_B;
			break;
		case TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_C:
			mode = TYPEC_DP_STATE_C;
			break;
		case TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_D:
			mode = TYPEC_DP_STATE_D;
			break;
		case TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_E:
			mode = TYPEC_DP_STATE_E;
			break;
		case TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_F:
			mode = TYPEC_DP_STATE_F;
			break;
		default:
			dev_err(tps->dev, "Invalid DP pin assignment\n");
			return -EINVAL;
		}

		dp_data.status = le32_to_cpu(st->dp_sid_status.status_rx);
		dp_data.conf = le32_to_cpu(st->dp_sid_status.configure);
		if (cd321x_dp_hpd(cd321x, st))
			dp_data.status |= DP_STATUS_HPD_STATE;
		else
			dp_data.status &= ~DP_STATUS_HPD_STATE;

		/*
		 * The sink dropped HPD and raised it again within the debounce
		 * window, as a monitor does after losing the link. The coalesced
		 * state looks unchanged, so the display side would never hear of
		 * it: report the pulse as IRQ_HPD. A port with a displayport
		 * reference reports the pulse out of band instead, as a
		 * disconnect and a connect (see cd321x_update_work()).
		 */
		hpd_pulse = !tps->h17_display &&
			    !(IS_REACHABLE(CONFIG_DRM) && cd321x->connector_fwnode) &&
			    (st->data_status_changed & CD321X_DATA_STATUS_HPD_LEVEL) &&
			    (dp_data.status & DP_STATUS_HPD_STATE) &&
			    (cd321x->dp_status & DP_STATUS_HPD_STATE);
		if (hpd_pulse)
			dp_data.status |= DP_STATUS_IRQ_HPD;

		if (cd321x->state_valid && cd321x->state.alt == cd321x->port_altmode_dp &&
		    cd321x->state.mode == mode && cd321x->dp_conf == dp_data.conf)
			send = cd321x_dp_send(cd321x->dp_status, dp_data.status, hpd_pulse);
		else
			send = CD321X_DP_SEND;
		if (send == CD321X_DP_SEND_NONE)
			return 0;

		cd321x->state.alt = cd321x->port_altmode_dp;
		cd321x->state.data = &dp_data;
		cd321x->state.mode = mode;
		ret = typec_thunderbolt_switch_set(cd321x->tbt_switch, &tbt_switch_data);
		if (ret)
			goto out;
		if (send == CD321X_DP_SEND_EDGE) {
			struct typec_displayport_data edge = {
				.status = dp_data.status & ~DP_STATUS_IRQ_HPD,
				.conf = dp_data.conf,
			};

			cd321x->state.data = &edge;
			ret = cd321x_mux_set(cd321x, held);
			cd321x->state.data = &dp_data;
			if (ret) {
				*display_busy = cd321x_display_taken(ret);
				goto out;
			}
			cd321x->dp_status = edge.status;
		}
		ret = cd321x_mux_set(cd321x, held);
		*display_busy = cd321x_display_taken(ret);
		if (!ret) {
			cd321x->dp_status = dp_data.status;
			cd321x->dp_conf = dp_data.conf;
		}
	} else {
		if (cd321x->state_valid && !cd321x->state.alt &&
		    cd321x->state.mode == TYPEC_STATE_USB)
			return 0;
		cd321x->state.alt = NULL;
		cd321x->state.mode = TYPEC_STATE_USB;
		cd321x->state.data = NULL;
		ret = typec_thunderbolt_switch_set(cd321x->tbt_switch, &tbt_switch_data);
		if (ret)
			goto out;
		ret = cd321x_mux_set(cd321x, held);
	}

out:
	/* Clear data since it's no longer used after typec_mux_set and points to the stack */
	cd321x->state.data = NULL;
	cd321x->state_valid = !ret;
	if (ret) {
		/* The mux may have changed before the switch failed. */
		cd321x->state = old_state;
		/* A provider that is not ready yet asks for a retry. */
		if (ret == -EAGAIN)
			dev_dbg(tps->dev, "Type-C connection deferred\n");
		/* The DCP route says which display waits; USB goes on. */
		else if (cd321x_display_taken(ret))
			dev_dbg(tps->dev, "Type-C display waits for a free pipeline: %d\n", ret);
		else
			cd321x_mux_failed(cd321x, "failed to configure Type-C connection", ret,
					  *held);
	} else if (cd321x->state.alt != cd321x->port_altmode_dp) {
		cd321x->dp_status = 0;
		cd321x->dp_conf = 0;
	}

	return ret;
}

/*
 * SN201202x with a display route in use and HPD staying high: pass on what
 * only the DP status showed since the last update, as the edge the final
 * state hides. The normal update that follows sends the final state.
 *  - HPD went low: the display behind the converter was unplugged, and
 *    maybe another one plugged in. Send HPD low first, so that the display
 *    is released and read anew.
 *  - IRQ_HPD: send the status without it first, so that the request reaches
 *    the display route even when the status it had already carried it.
 */
static int cd321x_dp_sid_replay(struct cd321x *cd321x,
				const struct cd321x_status *st, bool *display_busy, bool *held)
{
	struct device *dev = cd321x->tps.dev;
	struct cd321x_status edge = *st;
	u32 rx = le32_to_cpu(st->dp_sid_status.status_rx);

	if (!cd321x_uses_dp_sid(&cd321x->tps) || !cd321x->state_valid ||
	    cd321x->state.alt != cd321x->port_altmode_dp ||
	    !(cd321x->dp_status & DP_STATUS_HPD_STATE) || !cd321x_dp_hpd(cd321x, st))
		return 0;
	if (st->dp_sid_hpd_low) {
		dev_info(dev, "DP partner reported its display gone and back: HPD low, then high\n");
		edge.data_status &= ~CD321X_DATA_STATUS_HPD_LEVEL;
	} else if (st->dp_sid_irq && (rx & DP_STATUS_IRQ_HPD)) {
		dev_info(dev, "DP partner raised IRQ_HPD\n");
		edge.dp_sid_status.status_rx = cpu_to_le32(rx & ~DP_STATUS_IRQ_HPD);
	} else {
		return 0;
	}
	/* The following update reports the HPD pulse once. */
	edge.data_status_changed &= ~CD321X_DATA_STATUS_HPD_LEVEL;
	return cd321x_typec_update_mode(&cd321x->tps, &edge, display_busy, held);
}

/*
 * SN201202x: the display route in use is about to go (unplug, a new partner,
 * or the partner leaving DP). Tell it HPD is low first, while the PHY still
 * runs the link, so that the display is released and its link taken down
 * before the PHY stops. The update that follows moves the PHY.
 */
static int cd321x_dp_release_first(struct cd321x *cd321x, bool *held)
{
	struct typec_displayport_data dp_data;
	struct typec_mux_state state;
	int ret;

	if (!cd321x_uses_dp_sid(&cd321x->tps) || !cd321x->display_route_active ||
	    !cd321x->state_valid || cd321x->state.alt != cd321x->port_altmode_dp ||
	    !(cd321x->dp_status & DP_STATUS_HPD_STATE))
		return 0;
	dp_data.status = cd321x->dp_status & ~(DP_STATUS_HPD_STATE | DP_STATUS_IRQ_HPD);
	dp_data.conf = cd321x->dp_conf;
	state = cd321x->state;
	state.data = &dp_data;
	ret = typec_mux_set(cd321x->mux, &state);
	*held = cd321x_mux_held(cd321x, ret);
	if (ret)
		cd321x_mux_failed(cd321x, "display not released before the port changes", ret,
				  *held);
	else
		cd321x->dp_status = dp_data.status;
	return ret;
}

/* The boot order's first update has completed, or will not come: see tipd_boot_power_on(). */
static void tipd_boot_order_done(struct tps6598x *tps)
{
	tps->boot_no_debounce = false;
	tps->boot_usbw_due = false;
}

static void cd321x_retry_revalidation(struct cd321x *cd321x, bool reconnect)
{
	lockdep_assert_held(&cd321x->tps.lock);
	if (cd321x->pm.phase != CD321X_PM_RUNNING &&
	    cd321x->pm.phase != CD321X_PM_REVALIDATE &&
	    cd321x->pm.phase != CD321X_PM_APPLY)
		return;

	/* Keep a later physical event retryable if the automatic budget expires. */
	cd321x->state_valid = false;
	cd321x->wake_apply = false;
	if (cd321x_pm_retry(&cd321x->pm, reconnect)) {
		mod_delayed_work(system_freezable_wq, &cd321x->resume_work,
				 msecs_to_jiffies(CD321X_DEBOUNCE_DELAY_MS));
	} else {
		dev_err(cd321x->tps.dev, "cable revalidation failed after %u attempts\n",
			CD321X_RESUME_ATTEMPTS);
		tipd_boot_order_done(&cd321x->tps);
	}
}

static void cd321x_retry_held_mux(struct cd321x *cd321x)
{
	unsigned int now_ms = jiffies_to_msecs(jiffies);
	unsigned int retries, held_ms, delay;

	lockdep_assert_held(&cd321x->tps.lock);
	/* For the warning, should the retries end here */
	retries = cd321x->pm.mux_retries;
	held_ms = now_ms - cd321x->pm.mux_since_ms;
	cd321x->state_valid = false;
	cd321x->wake_apply = false;
	if (!cd321x->pm.mux_retry_ms)
		dev_info(cd321x->tps.dev, "Type-C mux held up, retrying\n");
	delay = cd321x_pm_mux_held(&cd321x->pm, now_ms);
	/* Probe records the hold, but ready() must enable the workers first. */
	if (delay && cd321x->pm.phase != CD321X_PM_INITIALIZING)
		mod_delayed_work(system_freezable_wq, &cd321x->resume_work,
				 msecs_to_jiffies(delay));
	else if (!delay)
		cd321x_mux_gave_up(cd321x, retries, held_ms);
}

static void cd321x_update_done(struct cd321x *cd321x)
{
	tipd_boot_order_done(&cd321x->tps);
	if (cd321x_pm_mux_done(&cd321x->pm))
		dev_info(cd321x->tps.dev, "Type-C mux change went through after retrying\n");
	cd321x_pm_complete(&cd321x->pm);
}

static void cd321x_dp_notify(struct cd321x *cd321x,
			     const struct cd321x_dp_update *u,
			     enum drm_connector_status status)
{
	if (!IS_REACHABLE(CONFIG_DRM) || !cd321x->connector_fwnode)
		return;

	trace_cd321x_dp_hotplug(cd321x->tps.dev, u, status);
	cd321x->dp_sink_reported = status != connector_status_disconnected;
	drm_connector_oob_hotplug_event(cd321x->connector_fwnode, status);
}

/* An identity of all zeros has not been received from the partner yet. */
static bool cd321x_identity_known(const struct usb_pd_identity *id)
{
	return memchr_inv(id, 0, sizeof(*id));
}

static void cd321x_macos_usbw(struct cd321x *cd321x);
static void tipd_boot_host_started(struct tps6598x *tps, enum usb_role role);

/*
 * DisplayPort pin D comes to a connection whose USB 3 already runs: the partner entered the
 * mode after the port's first update had set it up for USB. A lane provider that adds pin D
 * beside a running USB 3 link (typec_mux_dp_joins_usb()) gets the change with the USB role
 * kept, as macOS keeps the port's host running there and only adds the display. Every other
 * mode change stops the role first.
 */
static bool cd321x_dp_joins_usb(struct cd321x *cd321x, const struct cd321x_status *st)
{
	u32 ds = st->data_status;

	return cd321x->state_valid && !cd321x->state.alt &&
	       cd321x->state.mode == TYPEC_STATE_USB &&
	       (ds & TPS_DATA_STATUS_DATA_CONNECTION) &&
	       (ds & TPS_DATA_STATUS_DP_CONNECTION) &&
	       TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT(ds) ==
		       TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_D &&
	       !(st->data_status_changed & TPS_DATA_STATUS_USB3_CONNECTION) &&
	       typec_mux_dp_joins_usb(cd321x->mux);
}

static void cd321x_update_work(struct work_struct *work)
{
	struct cd321x *cd321x = container_of(to_delayed_work(work),
					    struct cd321x, update_work);
	struct tps6598x *tps = &cd321x->tps;
	struct cd321x_status st;
	struct typec_thunderbolt_switch_data tbt_switch_data = {
		.state = TYPEC_THUNDERBOLT_SWITCH_OFF,
	};
	bool display_busy, held = false;
	int ret;

	guard(mutex)(&tps->lock);
	if (!cd321x_pm_can_update(&cd321x->pm))
		return;
	cd321x_macos_usbw(cd321x);

	st = cd321x->update_status;
	cd321x->update_status.status_changed = 0;
	cd321x->update_status.data_status_changed = 0;
	cd321x->update_status.dp_sid_event = false;
	cd321x->update_status.dp_sid_hpd_low = false;
	cd321x->update_status.dp_sid_irq = false;

	bool old_connected = !!tps->partner;
	bool new_connected = st.status & TPS_STATUS_PLUG_PRESENT;
	u32 reset = 0;

	if (cd321x->pm.force_reconnect)
		reset |= CD321X_DP_RESET_FORCED;
	if (st.status_changed & TPS_STATUS_PLUG_PRESENT)
		reset |= CD321X_DP_RESET_PLUG;
	if (st.status_changed & TPS_STATUS_PLUG_UPSIDE_DOWN)
		reset |= CD321X_DP_RESET_FLIP;
	if (st.data_status_changed & (TPS_DATA_STATUS_TBT_CONNECTION |
				      CD321X_DATA_STATUS_USB4_CONNECTION))
		reset |= CD321X_DP_RESET_TUNNEL;

	bool was_disconnected = reset;

	bool usb_connection = st.data_status &
			      (TPS_DATA_STATUS_USB2_CONNECTION | TPS_DATA_STATUS_USB3_CONNECTION);
	bool dp_mode_changed = cd321x_dp_mode_changed(st.data_status,
						       st.data_status_changed);
	bool dp_connected = st.data_status & TPS_DATA_STATUS_DP_CONNECTION;
	bool dp_route_was_active = cd321x->display_route_active;
	struct cd321x_dp_update dp = {
		.data_status = st.data_status,
		.changed = st.data_status_changed,
		.irq_hpd = cd321x->dp_irq_hpd,
		.route_active = cd321x->display_route_active,
		.sink_reported = cd321x->dp_sink_reported,
	};

	bool usb4_started =
		(st.data_status_changed & CD321X_DATA_STATUS_USB4_CONNECTION) &&
		(st.data_status & CD321X_DATA_STATUS_USB4_CONNECTION);

	bool dp_hpd;

	enum usb_role old_role = usb_role_switch_get_role(tps->role_sw);
	enum usb_role new_role = USB_ROLE_NONE;
	enum typec_pwr_opmode pwr_opmode = TYPEC_PWR_MODE_USB;
	enum typec_orientation orientation = TYPEC_ORIENTATION_NONE;

	if (usb_connection) {
		if (tps->data_status & TPS_DATA_STATUS_USB_DATA_ROLE)
			new_role = USB_ROLE_DEVICE;
		else
			new_role = USB_ROLE_HOST;
	}

	if (new_connected) {
		pwr_opmode = TPS_POWER_STATUS_PWROPMODE(st.pwr_status);
		orientation = TPS_STATUS_TO_UPSIDE_DOWN(st.status) ?
			TYPEC_ORIENTATION_REVERSE : TYPEC_ORIENTATION_NORMAL;
	}

	bool is_pd = pwr_opmode == TYPEC_PWR_MODE_PD;
	/*
	 * The partner can be registered before its Discover Identity response
	 * arrives. An identity that was never received tells nothing about a
	 * partner change: take the one the partner sent later instead.
	 */
	bool identity_late = old_connected && new_connected && is_pd &&
		cd321x->cur_partner_is_pd &&
		!cd321x_identity_known(&cd321x->cur_partner_identity);
	bool new_partner = old_connected && new_connected &&
		(is_pd != cd321x->cur_partner_is_pd ||
		 (is_pd && !identity_late &&
		  memcmp(&st.partner_identity, &cd321x->cur_partner_identity,
			 sizeof(struct usb_pd_identity))));
	bool partner_changed = new_partner ||
		(old_connected && new_connected && was_disconnected);

	/* A different dock must release the old routing before reconfiguration. */
	was_disconnected |= partner_changed;
	if (new_partner)
		reset |= CD321X_DP_RESET_PARTNER;
	if (!new_connected)
		reset |= CD321X_DP_RESET_UNPLUG;

	/* DisplayPort pin D joining the running USB 3 link keeps the role. */
	bool dp_joins = dp_mode_changed && !was_disconnected && new_role == old_role &&
			cd321x_dp_joins_usb(cd321x, &st);

	/* A new DP connection: its DP status counts once one is signalled. */
	if (!new_connected || was_disconnected || !dp_connected || dp_mode_changed)
		cd321x->dp_sid_valid = false;
	if (dp_connected && st.dp_sid_event)
		cd321x->dp_sid_valid = true;
	dp_hpd = cd321x_dp_hpd(cd321x, &st);
	/* Feed the effective legacy sink level into the shared HPD policy. */
	if (!dp_hpd)
		dp.data_status &= ~CD321X_DATA_STATUS_HPD_LEVEL;
	if (st.dp_sid_hpd_low)
		dp.changed |= CD321X_DATA_STATUS_HPD_LEVEL;
	if (dp_hpd && cd321x_uses_dp_sid(tps))
		dp.irq_hpd |= st.dp_sid_irq;

	/* The display goes before anything else on the port changes. */
	if (dp_route_was_active &&
	    (!new_connected || was_disconnected || !dp_connected || dp_mode_changed)) {
		ret = cd321x_dp_release_first(cd321x, &held);
		if (ret)
			goto retry;
	}

	/*
	 * Report the sink as gone before any hardware transition that can fail:
	 * a failed teardown keeps the route, but the sink must not stay present.
	 * All ports reference the same external display connector, so only the
	 * port that owns the display route may report a disconnect. A second DP
	 * sink can enter Alt Mode while its mux request fails with -ENOSPC, and
	 * must not tear down the display carried by the other port.
	 */
	dp.link_reset = reset;
	if (cd321x_dp_teardown(&dp))
		cd321x_dp_notify(cd321x, &dp, connector_status_disconnected);

	/*
	 * ACIO carries the tunneled PCIe reset handshake over the still-live
	 * Type-C data path. Close a disappearing USB4/TBT session before dropping
	 * the USB role; otherwise the role transition tears down the ATC lanes and
	 * PCIe-C cannot quiesce its hierarchy before NHI removal.
	 */
	if ((!new_connected || was_disconnected) &&
	    (!cd321x->state_valid || cd321x->state.mode != TYPEC_STATE_SAFE)) {
		ret = typec_thunderbolt_switch_set(cd321x->tbt_switch,
						   &tbt_switch_data);
		if (ret)
			goto retry;
	}

	/* If we are switching from an active role, transition to USB_ROLE_NONE. */
	if (old_role != USB_ROLE_NONE &&
	    (new_role != old_role || was_disconnected || (dp_mode_changed && !dp_joins))) {
		ret = usb_role_switch_set_role(tps->role_sw, USB_ROLE_NONE);
		if (ret)
			goto retry;
	}

	/* Process partner disconnection or change */
	if (!new_connected || partner_changed) {
		if (tps->partner_altmode_dp) {
			typec_altmode_update_active(tps->partner_altmode_dp, false);
			typec_unregister_altmode(tps->partner_altmode_dp);
			tps->partner_altmode_dp = NULL;
		}
		if (!IS_ERR(tps->partner))
			typec_unregister_partner(tps->partner);
		tps->partner = NULL;
		cd321x->cur_partner_is_pd = false;
	}

	/* If there was a disconnection, set PHY to off */
	if (!new_connected || was_disconnected) {
		struct typec_mux_state old_state = cd321x->state;

		cd321x->state.alt = NULL;
		cd321x->state.mode = TYPEC_STATE_SAFE;
		cd321x->state.data = NULL;
		/* Through cd321x->mux, which can then tell if SAFE was held up */
		ret = cd321x_mux_set(cd321x, &held);
		cd321x->state_valid = !ret;
		if (ret) {
			cd321x_mux_failed(cd321x, "failed to place Type-C mux in safe mode", ret,
					  held);
			cd321x->state = old_state;
			goto retry;
		}
		cd321x->display_route_active = false;
	}

	/* Update Type-C properties */
	typec_set_pwr_opmode(tps->port, pwr_opmode);
	typec_set_pwr_role(tps->port, TPS_STATUS_TO_TYPEC_PORTROLE(st.status));
	typec_set_vconn_role(tps->port, TPS_STATUS_TO_TYPEC_VCONN(st.status));
	typec_set_orientation(tps->port, orientation);
	typec_set_data_role(tps->port, TPS_STATUS_TO_TYPEC_DATAROLE(st.status));
	power_supply_changed(tps->psy);

	/* If the plug is disconnected, we are done */
	if (!new_connected) {
		cd321x->dp_irq_hpd = false;
		cd321x_update_done(cd321x);
		return;
	}

	/* Set up partner if we were previously disconnected (or changed). */
	if (!tps->partner) {
		struct typec_partner_desc desc = { };

		desc.usb_pd = is_pd;
		desc.accessory = TYPEC_ACCESSORY_NONE; /* XXX: handle accessories */

		/*
		 * typec_register_partner() stores this pointer rather than
		 * copying what it points at, and the partner outlives this
		 * function, so it must not point into "st" on the stack.
		 */
		if (desc.usb_pd) {
			cd321x->cur_partner_identity = st.partner_identity;
			desc.identity = &cd321x->cur_partner_identity;
		}

		tps->partner = typec_register_partner(tps->port, &desc);
		if (IS_ERR(tps->partner)) {
			ret = PTR_ERR(tps->partner);
			dev_warn(tps->dev, "%s: failed to register partner\n", __func__);
			tps->partner = NULL;
			goto retry;
		}
		cd321x->cur_partner_is_pd = is_pd;

		if (desc.identity)
			typec_partner_set_identity(tps->partner);
	} else if (identity_late && cd321x_identity_known(&st.partner_identity)) {
		cd321x->cur_partner_identity = st.partner_identity;
		typec_partner_set_identity(tps->partner);
	}

	/* The CD321x firmware owns VDM discovery and Enter Mode. Publish the
	 * discovered DP mode only after it reports an entered DP connection;
	 * keep Linux's DP Alt Mode driver from issuing a second Enter Mode.
	 */
	if (dp_connected && !tps->partner_altmode_dp) {
		struct typec_altmode_desc desc = {
			.svid = USB_TYPEC_DP_SID,
			.mode = USB_TYPEC_DP_MODE,
			.vdo = le32_to_cpu(st.dp_sid_status.mode_data),
			.mode_selection = true,
		};

		tps->partner_altmode_dp = typec_partner_register_altmode(tps->partner,
								   &desc);
		if (IS_ERR(tps->partner_altmode_dp)) {
			ret = PTR_ERR(tps->partner_altmode_dp);
			tps->partner_altmode_dp = NULL;
			goto retry;
		}
	}
	if (tps->partner_altmode_dp)
		typec_altmode_update_active(tps->partner_altmode_dp, dp_connected);

	/* Update the TypeC MUX/PHY state */
	if (dp_route_was_active && dp_connected && !was_disconnected && !dp_mode_changed &&
	    st.dp_sid_event) {
		ret = cd321x_dp_sid_replay(cd321x, &st, &display_busy, &held);
		if (ret && !display_busy)
			goto retry;
	}
	ret = cd321x_typec_update_mode(tps, &st, &display_busy, &held);
	/*
	 * A provider failed the change with the role kept: stop the role and set the mode
	 * again, as for any other mode change.
	 */
	if (ret && dp_joins && !display_busy && !held) {
		ret = usb_role_switch_set_role(tps->role_sw, USB_ROLE_NONE);
		if (ret)
			goto retry;
		ret = cd321x_typec_update_mode(tps, &st, &display_busy, &held);
	}
	if (ret && !display_busy)
		goto retry;
	/* A shared display route can be busy while the dock's USB path is usable. */
	cd321x->display_route_active = !ret &&
		cd321x->state.alt == cd321x->port_altmode_dp;

	/* Launch the USB role switch */
	ret = usb_role_switch_set_role(tps->role_sw, new_role);
	if (ret)
		goto retry;
	tipd_boot_host_started(tps, new_role);

	/* A new USB4 route can disturb another port on the shared fabric. */
	if (IS_REACHABLE(CONFIG_DRM) && cd321x->connector_fwnode &&
	    !of_machine_is_compatible("apple,t8140") &&
	    !of_machine_is_compatible("apple,t8142") &&
	    usb4_started && !cd321x->display_route_active)
		drm_connector_oob_hotplug_event(cd321x->connector_fwnode,
						connector_status_unknown);

	/* An IRQ_HPD is delivered now, or overtaken by a connect. */
	cd321x->dp_irq_hpd = false;
	switch (cd321x_dp_event(&dp, cd321x->display_route_active)) {
	case CD321X_DP_EVENT_NONE:
		break;
	case CD321X_DP_EVENT_CONNECTED:
		cd321x_dp_notify(cd321x, &dp, connector_status_connected);
		break;
	case CD321X_DP_EVENT_RETRAIN:
		/*
		 * Report the HPD level as for any other update, so a connector
		 * that has lost the sink meanwhile connects it again.  Then an
		 * unknown status asks the connector to recheck its link.
		 */
		cd321x_dp_notify(cd321x, &dp, connector_status_connected);
		cd321x_dp_notify(cd321x, &dp, connector_status_unknown);
		break;
	}

	power_supply_changed(tps->psy);
	cd321x_update_done(cd321x);
	return;

retry:
	/*
	 * The Thunderbolt provider disabled the port for good after a failed
	 * resume recovery (see apple_cio_tbt_switch_set()). Other errors,
	 * including -ESHUTDOWN while the provider unbinds, are retried as before.
	 */
	if (ret == -EHOSTDOWN) {
		struct typec_mux_state safe = { .mode = TYPEC_STATE_SAFE };
		int cleanup;

		/* A terminal provider failure must not cycle the complex again. */
		cd321x->state.alt = NULL;
		cd321x->state.mode = TYPEC_STATE_SAFE;
		cd321x->state.data = NULL;
		/* Retain prepare/role/finish ownership even for a failed provider. */
		cleanup = typec_mux_begin(cd321x->mux, &safe);
		if (!cleanup) {
			if (!typec_mux_safe_after_role(cd321x->mux))
				cleanup = typec_mux_apply(cd321x->mux);
			if (!cleanup) {
				cleanup = usb_role_switch_set_role(tps->role_sw, USB_ROLE_NONE);
				if (cleanup)
					typec_mux_abort(cd321x->mux);
				else
					cleanup = typec_mux_end(cd321x->mux);
			}
		}
		if (cleanup)
			dev_warn(tps->dev, "terminal provider cleanup incomplete: %d\n", cleanup);
		cd321x_pm_mux_done(&cd321x->pm);
		cd321x->pm.phase = CD321X_PM_STALE;
		cd321x->pm.attempts_left = 0;
		cd321x->pm.force_reconnect = false;
		cd321x->pm.provider_busy = false;
		cd321x->state_valid = false;
		dev_err(tps->dev, "Type-C provider shut down; automatic retries stopped\n");
		tipd_boot_order_done(tps);
		return;
	}
	/* A held mux must not consume a coalesced partner edge. */
	cd321x->update_status.dp_sid_event |= st.dp_sid_event;
	cd321x->update_status.dp_sid_hpd_low |= st.dp_sid_hpd_low;
	cd321x->update_status.dp_sid_irq |= st.dp_sid_irq;
	if (held) {
		cd321x_retry_held_mux(cd321x);
		return;
	}
	/* A deferred PCIe activation is not evidence of a failed cable. */
	cd321x_retry_revalidation(cd321x, ret != -EAGAIN);
}

static void cd321x_queue_status(struct cd321x *cd321x)
{
	cd321x->dp_irq_hpd =
		cd321x_dp_irq_hpd(cd321x->dp_irq_hpd,
				  cd321x->update_status.data_status,
				  cd321x->tps.data_status,
				  le32_to_cpu(cd321x->update_status.dp_sid_status.status_rx),
				  le32_to_cpu(cd321x->dp_sid_status.status_rx));
	cd321x->update_status.status_changed |= cd321x->update_status.status ^ cd321x->tps.status;
	cd321x->update_status.data_status_changed |=
		cd321x->update_status.data_status ^ cd321x->tps.data_status;

	cd321x->update_status.status = cd321x->tps.status;
	cd321x->update_status.pwr_status = cd321x->tps.pwr_status;
	cd321x->update_status.data_status = cd321x->tps.data_status;

	cd321x->update_status.partner_identity = cd321x->tps.partner_identity;
	cd321x->update_status.dp_sid_status = cd321x->dp_sid_status;
	cd321x->update_status.intel_vid_status = cd321x->intel_vid_status;
	cd321x->update_status.usb4_status = cd321x->usb4_status;
}

static int cd321x_switch_power_state(struct tps6598x *tps, u8 target_state);

/*
 * J714s: what macOS sends a port controller when its cable goes and comes (traced on the Dell
 * monitor's port). 60 ms after the detach: SYS_PWR_STATE read (0x00), DATA1 = 0x02 and SSPS. At
 * the attach: the events and status read, then DATA1 = 0x00 and SSPS; ~530 ms later
 * SYS_PWR_STATE read twice, DATA1 = 0x00 and USBw. Linux sent neither outside a sleep, so after a
 * cable change the controller was left as the change left it. Here: SSPS 0x02 at a detach, SSPS
 * 0x00 at an attach and USBw 0x00 at the update after the attach's debounce (500 ms), for an
 * SN201202x whose DT asks for it (apple,cable-change-ssps, the traced J714s's USB-C ports), only
 * while running (not in a sleep's phases).
 */
#define TIPD_SSPS_MACOS_DETACHED	0x02
/*
 * 28 ms after the detach and before its SSPS, macOS writes register 0x50 with 64 bytes, 00 20 and
 * 62 zeros, and reads it back as all zeros from 3 ms later. Its function is not known; it is
 * written as macOS writes it.
 */
#define TIPD_REG_MACOS_DETACH		0x50

/*
 * An SN201202x whose DT asks for macOS's cable-change sequence (apple,cable-change-ssps), and only
 * a USB-C port's, as traced: MagSafe's has no connector. Not a controller in a debug route, from
 * before Linux or opened through debugfs: that route is its owner's, and gets no power state
 * commands.
 */
static bool cd321x_macos_cable_port(struct cd321x *cd321x)
{
	struct tps6598x *tps = &cd321x->tps;
	struct device *dev = tps->dev;
	struct fwnode_handle *con;

	if (!device_is_compatible(dev, "apple,sn201202x") || tps->dbma_at_probe ||
	    tipd_debugfs_dbma_held(tps) || !device_property_read_bool(dev, "apple,cable-change-ssps"))
		return false;
	con = device_get_named_child_node(dev, "connector");
	fwnode_handle_put(con);
	return con;
}

static bool cd321x_macos_cable_on(struct cd321x *cd321x)
{
	return cd321x->pm.phase == CD321X_PM_RUNNING && cd321x_macos_cable_port(cd321x);
}

static void cd321x_macos_cable(struct cd321x *cd321x, bool attached)
{
	struct tps6598x *tps = &cd321x->tps;
	enum cd321x_pm_phase phase = cd321x->pm.phase;
	u8 state;
	int ret;

	lockdep_assert_held(&tps->lock);
	cd321x->macos_usbw_due = false;
	/*
	 * A detach is followed while running; the attach that undoes its SSPS 0x02 is followed in
	 * any phase but a sleep's or the port's setup and removal, whose own S0 or reset covers it.
	 */
	if (attached && cd321x->macos_detached)
		cd321x->macos_detached = phase != CD321X_PM_PREPARED &&
					 phase != CD321X_PM_REMOVED &&
					 phase != CD321X_PM_INITIALIZING;
	if (!(attached && cd321x->macos_detached) && !cd321x_macos_cable_on(cd321x))
		return;
	if (!attached) {
		u8 reg50[TPS_MAX_LEN] = { 0x00, 0x20 };

		ret = tps6598x_block_write(tps, TIPD_REG_MACOS_DETACH, reg50, sizeof(reg50));
		dev_info(tps->dev, "cable detach: register 0x50 = 00 20 + 62 zeros: %d\n", ret);
	}
	state = attached ? TPS_SYSTEM_POWER_STATE_S0 : TIPD_SSPS_MACOS_DETACHED;
	ret = cd321x_switch_power_state(tps, state);
	cd321x->macos_detached = !attached;
	cd321x->macos_usbw_due = attached;
	dev_info(tps->dev, "cable %s: SSPS %#x: %d\n", attached ? "attach" : "detach", state, ret);
}

static void cd321x_macos_usbw(struct cd321x *cd321x)
{
	struct tps6598x *tps = &cd321x->tps;
	u8 mode = 0x00;
	int ret;

	lockdep_assert_held(&tps->lock);
	if (!cd321x->macos_usbw_due)
		return;
	cd321x->macos_usbw_due = false;
	if (!cd321x_macos_cable_on(cd321x))
		return;
	ret = tipd_usbw_locked(tps, mode);
	dev_info(tps->dev, "cable attach: USBw %#x after the debounce: %d\n", mode, ret);
}

static int cd321x_connect(struct tps6598x *tps, u32 status)
{
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);
	bool plug_changed;
	unsigned int delay = CD321X_DEBOUNCE_DELAY_MS;

	/* Keep every local role and data path down until the connector is dry. */
	if (tps->contaminant_ops &&
	    tps->contaminant != TYPEC_CONTAMINANT_NONE)
		status &= ~TPS_STATUS_PLUG_PRESENT;

	/*
	 * The resume's first fresh read, with a plug that was in when the sleep began, is in now,
	 * and raised no plug event and no change since: nothing was plugged in just now, so its
	 * state is applied at once, not after the debounce of a plug. The external display and the
	 * port's USB then come back that much sooner. Any plug change on the way, in any phase,
	 * and every later read keep the debounce.
	 */
	plug_changed = (tps->status ^ status) & TPS_STATUS_PLUG_PRESENT;
	if (plug_changed)
		cd321x->wake_apply = false;
	if (cd321x->pm.phase == CD321X_PM_APPLY) {
		if (cd321x->wake_apply && (status & TPS_STATUS_PLUG_PRESENT))
			delay = 0;
		cd321x->wake_apply = false;
	}

	if (plug_changed)
		cd321x_pm_new_connection(&cd321x->pm);
	if (plug_changed)
		cd321x_macos_cable(cd321x, status & TPS_STATUS_PLUG_PRESENT);
	tps->status = status;
	cd321x_queue_status(cd321x);
	/* A plug or power IRQ alone does not refresh the cached cable metadata. */
	if (cd321x_pm_event(&cd321x->pm))
		mod_delayed_work(system_freezable_wq, &cd321x->resume_work, 0);
	/* A held mux waits out its backoff only while nothing else happens. */
	if (cd321x_pm_mux_event(&cd321x->pm))
		mod_delayed_work(system_freezable_wq, &cd321x->resume_work,
				 msecs_to_jiffies(CD321X_DEBOUNCE_DELAY_MS));
	if (!cd321x_pm_can_update(&cd321x->pm))
		return 0;
	/*
	 * The first update after the probe left the boot power state (queued by the ready
	 * revalidation: the probe's own connect queues none) runs without the debounce, see
	 * tipd_boot_power_on().
	 */
	if (tps->boot_no_debounce) {
		tps->boot_no_debounce = false;
		if (status & TPS_STATUS_PLUG_PRESENT)
			delay = 0;
	}

	/*
	 * Requeue even when the previous update is already running.  A separate
	 * cancel_delayed_work() and schedule_delayed_work() pair can lose a cable
	 * event in that window because the latter sees the running work as busy.
	 * The worker consumes every accumulated status change on its next pass.
	 */
	mod_delayed_work(system_freezable_wq, &cd321x->update_work, msecs_to_jiffies(delay));

	return 0;
}

/*
 * The controller raises its interrupt only when events are latched while none
 * were pending, so latched events that are never acknowledged silence the port
 * for good. Acknowledge them before a snapshot read: the snapshot covers them.
 */
static int cd321x_ack_events(struct tps6598x *tps)
{
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);
	u64 event;
	int ret;

	ret = tps6598x_read64(tps, TPS_REG_INT_EVENT1, &event);
	if (ret || !event)
		return ret;
	trace_cd321x_irq(event);
	/* A plug event latched while asleep: a cable came or went, see cd321x_connect() */
	if (event & APPLE_CD_REG_INT_PLUG_EVENT)
		cd321x->wake_apply = false;

	return tps6598x_write64(tps, TPS_REG_INT_CLEAR1, event);
}

/* Recover events that an interrupt could not read, acknowledge or apply. */
static void cd321x_event_lost(struct cd321x *cd321x)
{
	lockdep_assert_held(&cd321x->tps.lock);
	cd321x->wake_apply = false;
	if (cd321x_pm_event_lost(&cd321x->pm))
		mod_delayed_work(system_freezable_wq, &cd321x->resume_work,
				 msecs_to_jiffies(CD321X_DEBOUNCE_DELAY_MS));
}

/*
 * An attached cable need not generate an IRQ after sleep. Read a fresh
 * snapshot and reapply it with the mode cache invalidated, allowing the
 * provider to validate the resumed session. Preserve a healthy connection;
 * only a real cable change or a failed read/apply requests forced recovery.
 */
static int cd321x_check_setup(struct cd321x *cd321x, u64 event);

static void cd321x_resume_work(struct work_struct *work)
{
	struct cd321x *cd321x = container_of(to_delayed_work(work),
					    struct cd321x, resume_work);
	struct tps6598x *tps = &cd321x->tps;
	unsigned int notes;
	u32 status;
	int ret;
	bool fault, ready, restore;

	guard(mutex)(&tps->lock);
	if (cd321x->pm.phase == CD321X_PM_INITIALIZING ||
	    cd321x->pm.phase == CD321X_PM_PREPARED ||
	    cd321x->pm.phase == CD321X_PM_REMOVED)
		return;

	if (cd321x->pm.setup_pending && cd321x_check_setup(cd321x, 0) < 0)
		return;

	fault = atomic_xchg(&cd321x->link_event, 0);
	ready = atomic_xchg(&cd321x->ready_event, 0);
	restore = atomic_xchg(&cd321x->restore_event, 0);
	notes = cd321x_pm_notifications(&cd321x->pm, fault, ready, restore);
	if (notes & CD321X_PM_FAULT_ACCEPTED)
		dev_dbg(tps->dev, "revalidating cable after tunnel state change\n");
	if (notes & CD321X_PM_FAULT_REFUSED)
		dev_warn_ratelimited(tps->dev,
				     "tunnel link recovery budget exhausted\n");
	if (notes & (CD321X_PM_FAULT_ACCEPTED | CD321X_PM_READY_ACCEPTED |
		     CD321X_PM_RESTORE_ACCEPTED)) {
		cd321x->state_valid = false;
		cd321x->wake_apply = false;
	}
	if (!cd321x_pm_begin_read(&cd321x->pm))
		return;

	ret = cd321x_ack_events(tps);
	if (ret) {
		/* Retried until it succeeds, so report only the first failure. */
		if (!cd321x->pm.ack_retry_ms)
			dev_err(tps->dev, "failed to acknowledge events: %d\n", ret);
		mod_delayed_work(system_freezable_wq, &cd321x->resume_work,
				 msecs_to_jiffies(cd321x_pm_ack_failed(&cd321x->pm)));
		return;
	}
	if (cd321x_pm_acked(&cd321x->pm))
		dev_info(tps->dev, "acknowledged events after retrying\n");

	if (!tps6598x_read_status(tps, &status))
		goto retry;
	/* Without a plug too, or a power supply removed in sleep stays online */
	if (!tps6598x_read_power_status(tps))
		goto retry;

	if (status & TPS_STATUS_PLUG_PRESENT) {
		if (TPS_POWER_STATUS_PWROPMODE(tps->pwr_status) == TYPEC_PWR_MODE_PD &&
		    tps6598x_read_partner_identity(tps))
			goto retry;
		if (!tps->data->read_data_status(tps))
			goto retry;
	}

	/* Also report a cable removed during sleep, even if its IRQ was lost. */
	cd321x_pm_snapshot_ready(&cd321x->pm);
	tps->data->connect(tps, status);
	return;

retry:
	cd321x_retry_revalidation(cd321x, true);
}

/*
 * The limit of an SN201202x's system power state command under the sleep's rules. The
 * controller raises no interrupt for a command's end, so the wait ends on the controller's own
 * signal: the command register read at intervals until the command has ended, then the system
 * power state read back. Woken first, the controller ends this command within the first few
 * reads (7-12 ms in macOS's traces of the J714s). The limit is only the fallback, this driver's
 * default command limit: a controller that has not ended the command by then is not answering,
 * and the sleep is refused (S3), or the controller is set up again by its next event (S0).
 */
#define CD321X_SLEEP_SSPS_LIMIT_MS	1000

static int cd321x_prepare(struct tps6598x *tps)
{
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);
	int ret = 0;

	scoped_guard(mutex, &tps->lock) {
		cd321x_pm_prepare(&cd321x->pm);
		tipd_boot_order_done(tps);
		cd321x->state_valid = false;
		cd321x->wake_apply = false;
		cd321x->sleep_plug = tps->status & TPS_STATUS_PLUG_PRESENT;
	}
	/* Gate scheduling first; no worker may be drained under its mutex. */
	cancel_delayed_work_sync(&cd321x->resume_work);
	cancel_delayed_work_sync(&cd321x->update_work);
	cancel_delayed_work_sync(&cd321x->rearm_work);

	/* As macOS does a second or so before the commit, measured on the same controller. */
	if (pm_suspend_via_firmware() && device_property_read_bool(tps->dev, "apple,sleep-ssps")) {
		ktime_t start = ktime_get();

		guard(mutex)(&tps->lock);
		ret = tps->data->switch_power_state(tps, TPS_SYSTEM_POWER_STATE_S3);
		/* Without the rules: the lines and their levels as they were */
		if (!tipd_sleep_ssps_rules(tps) && ret)
			dev_warn(tps->dev, "didn't take system power state S3: %d\n", ret);
		else if (!tipd_sleep_ssps_rules(tps))
			dev_dbg(tps->dev, "system power state S3\n");
		else if (ret)
			dev_warn(tps->dev, "didn't take system power state S3: %d after %lld ms: sleep refused\n",
				 ret, ktime_ms_delta(ktime_get(), start));
		else
			dev_info(tps->dev, "system power state S3 in %lld ms\n",
				 ktime_ms_delta(ktime_get(), start));
	}
	/*
	 * A port controller that did not take S3 refuses the sleep. The one sleep entered after
	 * such a failure (J714s: the command ended after 3.6 s with the controller not answering on
	 * its bus) went on to the commit, where the SMC did not answer the sleep notice: a reset
	 * instead of a sleep. Refused here, nothing has been stopped yet and the controller is set
	 * up again.
	 */
	if (ret && tipd_sleep_ssps_rules(tps))
		return -EBUSY;
	return 0;
}

/* A try every 250 ms for 10 s */
#define CD321X_REARM_TRIES	40
#define CD321X_REARM_RETRY_MS	250

static int cd321x_debug_cmd(struct tps6598x *tps, const char *cmd, size_t len, const u8 *in)
{
	u8 out = 0;
	int ret;

	ret = tps6598x_exec_cmd(tps, cmd, len, in, 1, &out);
	return ret ?: (out & 0xf ? -EIO : 0);
}

/* The firmware rejects a command for the mode it is switching to until it has switched. */
static int cd321x_wait_dbma(struct tps6598x *tps, bool on)
{
	u32 mode;
	int ret, err;

	err = read_poll_timeout(tps6598x_read32, ret,
				ret || (!memcmp(&mode, modes[TPS_MODE_DBMA], 4) == on),
				1000, 100000, false, tps, TPS_REG_MODE, &mode);
	return ret ?: err;
}

/*
 * The boot loader's sequence: unlock with the board's key, debug mode on, enable the debug
 * VDM, debug mode off, lock. Debug mode is left and the lock set again after a failure too.
 */
static int cd321x_arm_debug_link(struct tps6598x *tps, const char *key, const char **step)
{
	static const u8 vdm[] = { 0x06, 0x46, 0x82, 0x01 };
	static const u8 no_key[4];
	u8 on = 1, off = 0;
	int ret, err;

	*step = "unlock";
	ret = cd321x_debug_cmd(tps, "LOCK", sizeof(no_key), (const u8 *)key);
	if (ret)
		return ret;

	*step = "debug mode";
	ret = cd321x_debug_cmd(tps, "DBMa", 1, &on);
	if (!ret)
		ret = cd321x_wait_dbma(tps, true);
	if (!ret) {
		*step = "debug VDM";
		ret = cd321x_debug_cmd(tps, "DVEn", sizeof(vdm), vdm);
	}

	err = cd321x_debug_cmd(tps, "DBMa", 1, &off);
	if (!err)
		err = cd321x_wait_dbma(tps, false);
	if (err && !ret) {
		*step = "leaving debug mode";
		ret = err;
	}
	err = cd321x_debug_cmd(tps, "LOCK", sizeof(no_key), no_key);
	if (err && !ret) {
		*step = "lock";
		ret = err;
	}
	return ret;
}

/*
 * A sleep cut this port's debug link. Right after the wake the controller can reject the
 * sequence, so a failed try is repeated until the link is back.
 */
static void cd321x_rearm_debug_link(struct cd321x *cd321x)
{
	struct tps6598x *tps = &cd321x->tps;
	const char *key, *step;
	int ret;

	lockdep_assert_held(&tps->lock);
	if (device_property_read_string(tps->dev, "apple,debug-lock-key", &key) ||
	    strlen(key) != 4) {
		cd321x->rearm_pending = false;
		return;
	}

	ret = cd321x_arm_debug_link(tps, key, &step);
	cd321x->rearm_tries++;
	if (!ret) {
		if (cd321x->rearm_tries > 1)
			dev_info(tps->dev, "debug link rearmed, try %u\n", cd321x->rearm_tries);
		cd321x->rearm_pending = false;
		return;
	}
	if (cd321x->rearm_tries < CD321X_REARM_TRIES) {
		mod_delayed_work(system_freezable_wq, &cd321x->rearm_work,
				 msecs_to_jiffies(CD321X_REARM_RETRY_MS));
		return;
	}
	dev_warn(tps->dev, "debug link not rearmed after %u tries: %s: %d\n",
		 cd321x->rearm_tries, step, ret);
	cd321x->rearm_pending = false;
}

/* The cut's cold reset, and how it waits for the contract to come back, see apple_sleep_usb_down(). */
#define CD321X_REARM_RESET_AFTER	4
#define CD321X_REARM_RESETS		3
#define CD321X_RESET_MS			1000
#define CD321X_CONTRACT_MS		5000

static bool cd321x_has_contract(struct tps6598x *tps)
{
	u16 pwr;

	return !tps6598x_read16(tps, TPS_REG_POWER_STATUS, &pwr) &&
	       TPS_POWER_STATUS_CONNECTION(pwr) &&
	       TPS_POWER_STATUS_PWROPMODE(pwr) == TYPEC_PWR_MODE_PD;
}

/*
 * The debug VDM needs a PD contract. After a cut whose sleep was refused, the partner was
 * found to have dropped it, both ends at default USB power, and no try could rearm the link.
 * Do what the cut does when the contract doesn't come back: reset the controller again and
 * wait for it. The controller then needs setting up as after the cut.
 */
static void cd321x_rearm_recover_contract(struct cd321x *cd321x)
{
	struct tps6598x *tps = &cd321x->tps;
	int ms, ret;

	lockdep_assert_held(&tps->lock);
	cd321x->rearm_resets++;
	dev_warn(tps->dev, "debug link: no contract with the partner, reset %u\n",
		 cd321x->rearm_resets);
	ret = tps6598x_exec_cmd_tmo(tps, "Gaid", 0, NULL, 0, NULL, 2000, 0);
	if (ret)
		dev_dbg(tps->dev, "reset command: %d\n", ret);
	msleep(CD321X_RESET_MS);
	for (ms = 0; ms < CD321X_CONTRACT_MS && !cd321x_has_contract(tps); ms += 50)
		msleep(50);
	if (ms < CD321X_CONTRACT_MS)
		dev_info(tps->dev, "debug link: contract back %d ms after reset %u\n",
			 CD321X_RESET_MS + ms, cd321x->rearm_resets);
	cd321x->pm.setup_pending = true;
	cd321x->reset_by_cut = true;
	cd321x->state_valid = false;
	mod_delayed_work(system_freezable_wq, &cd321x->resume_work, 0);
	cd321x->rearm_tries = 0;
}

static void cd321x_rearm_work(struct work_struct *work)
{
	struct cd321x *cd321x = container_of(to_delayed_work(work), struct cd321x, rearm_work);
	struct tps6598x *tps = &cd321x->tps;

	guard(mutex)(&tps->lock);
	if (!cd321x->rearm_pending || cd321x->pm.phase == CD321X_PM_PREPARED ||
	    cd321x->pm.phase == CD321X_PM_REMOVED)
		return;
	if (cd321x->rearm_tries >= CD321X_REARM_RESET_AFTER &&
	    cd321x->rearm_resets < CD321X_REARM_RESETS && !cd321x_has_contract(tps))
		cd321x_rearm_recover_contract(cd321x);
	cd321x_rearm_debug_link(cd321x);
}

static void cd321x_complete(struct tps6598x *tps)
{
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);

	guard(mutex)(&tps->lock);
	/*
	 * The cut was a cold reset of this controller's firmware: its interrupt mask and power
	 * state are back at their defaults, so set them up again whether or not an event comes.
	 */
	if (apple_sleep_debug_link_cut_peek(tps->dev)) {
		cd321x->pm.setup_pending = true;
		cd321x->reset_by_cut = true;
	}
	if (pm_suspend_via_firmware() && device_property_read_bool(tps->dev, "apple,sleep-ssps")) {
		ktime_t start = ktime_get();
		int ret = tps->data->switch_power_state(tps, TPS_SYSTEM_POWER_STATE_S0);

		/* After a sleep or a refused one: never waited for in silence, and never longer */
		if (!tipd_sleep_ssps_rules(tps))
			ret = 0;
		else if (ret)
			dev_warn(tps->dev, "didn't take system power state S0: %d after %lld ms; set up again by its next event\n",
				 ret, ktime_ms_delta(ktime_get(), start));
		else
			dev_info(tps->dev, "system power state S0 in %lld ms\n",
				 ktime_ms_delta(ktime_get(), start));
		if (ret)
			cd321x->pm.setup_pending = true;
	}
	/*
	 * A link still down from an earlier sleep isn't cut again, so it stays pending. A dark
	 * wake sleeps again at once: the link stays down for it, and the full wake rearms it.
	 */
	if (apple_sleep_debug_link_was_cut(tps->dev))
		cd321x->rearm_pending = true;
	if (cd321x->rearm_pending && !pm_dark_wake()) {
		cd321x->rearm_tries = 0;
		cd321x->rearm_resets = 0;
		cd321x_rearm_debug_link(cd321x);
	}
	if (cd321x_pm_resume(&cd321x->pm)) {
		/* Not after a dark wake, which sleeps again at once, or a controller set up again */
		cd321x->wake_apply = cd321x->sleep_plug && !pm_dark_wake() &&
				     !cd321x->pm.setup_pending;
		mod_delayed_work(system_freezable_wq, &cd321x->resume_work, 0);
	}
}

static int cd321x_setup_read_state(void *ctx, u8 *state)
{
	struct cd321x *cd321x = ctx;

	return tps6598x_read8(&cd321x->tps, TPS_REG_SYSTEM_POWER_STATE, state);
}

static int cd321x_setup_read_mask(void *ctx, u64 *mask)
{
	struct cd321x *cd321x = ctx;

	return tps6598x_read64(&cd321x->tps, TPS_REG_INT_MASK1, mask);
}

static int cd321x_setup_set_s0(void *ctx)
{
	struct cd321x *cd321x = ctx;
	struct tps6598x *tps = &cd321x->tps;

	return tps->data->switch_power_state(tps, TPS_SYSTEM_POWER_STATE_S0);
}

static int cd321x_setup_set_mask(void *ctx, u64 mask)
{
	struct cd321x *cd321x = ctx;

	return tps6598x_write64(&cd321x->tps, TPS_REG_INT_MASK1, mask);
}

static const struct cd321x_setup_ops cd321x_setup_ops = {
	.read_state = cd321x_setup_read_state,
	.read_mask = cd321x_setup_read_mask,
	.set_s0 = cd321x_setup_set_s0,
	.set_mask = cd321x_setup_set_mask,
};

/* Restore a reset controller even if the sole reset IRQ has been cleared. */
static int cd321x_check_setup(struct cd321x *cd321x, u64 event)
{
	struct tps6598x *tps = &cd321x->tps;
	int ret;

	lockdep_assert_held(&tps->lock);
	if (cd321x->pm.phase == CD321X_PM_REMOVED ||
	    (!cd321x->pm.setup_pending && !(event & ~cd321x_irq_mask(tps))))
		return 0;
	ret = cd321x_setup_run(&cd321x->pm, event, cd321x_irq_mask(tps),
			       &cd321x_setup_ops, cd321x);
	/*
	 * A controller that had to be restored was reset, which also removed
	 * its contaminant mitigation. An intact controller kept it.
	 */
	if (ret && tps->contaminant_ops)
		tps->contaminant_ops->invalidate(tps);
	if (cd321x->pm.phase == CD321X_PM_PREPARED ||
	    cd321x->pm.phase == CD321X_PM_INITIALIZING)
		return 0;
	if (ret < 0) {
		if (cd321x->pm.setup_attempts_left)
			mod_delayed_work(system_freezable_wq, &cd321x->resume_work,
					 msecs_to_jiffies(CD321X_DEBOUNCE_DELAY_MS));
		else
			dev_err(tps->dev, "controller setup recovery exhausted: %d\n", ret);
		return ret;
	}
	if (ret && cd321x->reset_by_cut)
		dev_dbg(tps->dev, "set up again after the debug link's cut\n");
	else if (ret)
		dev_warn(tps->dev, "restored controller power state and interrupt mask\n");
	cd321x->reset_by_cut = false;
	/* Only a fresh snapshot may be applied after restoring the controller. */
	cd321x->state_valid = false;
	cd321x->pm.phase = CD321X_PM_REVALIDATE;
	cd321x->pm.attempts_left = CD321X_RESUME_ATTEMPTS;
	cd321x->pm.force_reconnect |= ret > 0;
	cd321x->pm.provider_busy = false;
	mod_delayed_work(system_freezable_wq, &cd321x->resume_work, 0);
	return 0;
}

/**
 * cd321x_sleep_events - Acknowledge the events of a controller in system sleep
 * @tps: The controller, woken for this
 * @st: Its data status and status, and for a power sink whether its source is
 *	capable of USB communication
 *
 * The resume revalidates the whole port state, so nothing else is read.
 *
 * Context: tps->lock held.
 * Return: 0, or a negative error code.
 */
int cd321x_sleep_events(struct tps6598x *tps, struct cd321x_sleep_state *st)
{
	/* The PDO count, then the first PDO, vSafe5V, which carries the flags */
	u8 caps[1 + sizeof(u32)];
	u64 event;
	int ret;

	lockdep_assert_held(&tps->lock);
	st->usb_comm = false;
	st->contaminant_event = false;
	ret = tps6598x_read64(tps, TPS_REG_INT_EVENT1, &event);
	if (!ret && event)
		ret = tps6598x_write64(tps, TPS_REG_INT_CLEAR1, event);
	if (ret)
		return ret;
	st->contaminant_event = event & SN201202X_INT_LDCM_UPDATE;
	/* A cable came or went while asleep: the wake's read gets the debounce, see cd321x_connect() */
	if (event & APPLE_CD_REG_INT_PLUG_EVENT)
		container_of(tps, struct cd321x, tps)->sleep_plug = false;
	/* A controller that was reset is set up again on resume */
	cd321x_check_setup(container_of(tps, struct cd321x, tps), event);

	ret = tps6598x_read32(tps, TPS_REG_DATA_STATUS, &st->data_status);
	if (!ret)
		ret = tps6598x_read32(tps, TPS_REG_STATUS, &st->status);
	if (!ret && !(st->status & TPS_STATUS_PORTROLE)) {
		ret = tps6598x_block_read(tps, TPS_REG_RX_SOURCE_CAPS, caps, sizeof(caps));
		if (!ret && (caps[0] & GENMASK(2, 0)))
			st->usb_comm = get_unaligned_le32(&caps[1]) & PDO_FIXED_USB_COMM;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(cd321x_sleep_events);

/**
 * cd321x_wake_events - The first reads at the wake, as macOS makes them (J714s)
 * @tps: The controller, just woken with its bus's wake command, still in S3
 * @event: Its INT_EVENT1, which is cleared as read
 * @status: Its status
 * @data_status: Its data status
 *
 * At every wake macOS reads INT_EVENT1, writes the same bits, all CD321X_WAKE_EVENT_LEN bytes,
 * to INT_CLEAR1 (even none), then reads the status registers, and only then switches the
 * controller back to S0 (traced on the J714s). Nothing is applied here: the resume's
 * revalidation reads the whole port state again. What the poll would have kept of these events
 * is kept, as cd321x_sleep_events() keeps it: a plug event takes away the wake's quiet first read
 * (see cd321x_connect()), a controller found reset is set up again by the resume, and a new
 * DisplayPort status is noted for it. A liquid-detection result is read at the complete.
 *
 * Context: tps->lock held.
 * Return: 0, or a negative error code.
 */
int cd321x_wake_events(struct tps6598x *tps, u8 event[CD321X_WAKE_EVENT_LEN], u32 *status,
		       u32 *data_status)
{
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);
	u64 low;
	int ret;

	lockdep_assert_held(&tps->lock);
	ret = tps6598x_block_read(tps, TPS_REG_INT_EVENT1, event, CD321X_WAKE_EVENT_LEN);
	if (!ret)
		ret = tps6598x_block_write(tps, TPS_REG_INT_CLEAR1, event, CD321X_WAKE_EVENT_LEN);
	if (ret)
		return ret;
	low = get_unaligned_le64(event);
	if (low & APPLE_CD_REG_INT_PLUG_EVENT)
		cd321x->sleep_plug = false;
	cd321x_check_setup(cd321x, low);

	ret = tps6598x_read32(tps, TPS_REG_STATUS, status);
	if (ret)
		return ret;
	if (low & cd321x_irq_mask(tps) & SN201202X_INT_DP_SID_UPDATE) {
		if (!tps->data->read_data_status(tps))
			return -EIO;
		cd321x_dp_sid_event(cd321x);
		*data_status = tps->data_status;
		return 0;
	}
	return tps6598x_read32(tps, TPS_REG_DATA_STATUS, data_status);
}
EXPORT_SYMBOL_GPL(cd321x_wake_events);

static irqreturn_t cd321x_interrupt(int irq, void *data)
{
	struct tps6598x *tps = data;
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);
	u64 event = 0;
	u32 status;
	int ret;

	if (tps->sleep_event && tps->sleep_event(tps))
		return IRQ_HANDLED;

	mutex_lock(&tps->lock);

	ret = tps6598x_read64(tps, TPS_REG_INT_EVENT1, &event);
	if (ret) {
		dev_err(tps->dev, "%s: failed to read events\n", __func__);
		goto err_lost;
	}
	trace_cd321x_irq(event);

	if (!event)
		goto err_unlock;
	/* A cable came or went: a wake's first read is no longer a quiet one, see cd321x_connect() */
	if (event & APPLE_CD_REG_INT_PLUG_EVENT)
		cd321x->wake_apply = false;

	ret = tps6598x_write64(tps, TPS_REG_INT_CLEAR1, event);
	if (ret) {
		dev_err(tps->dev, "%s: failed to clear events\n", __func__);
		goto err_lost;
	}
	if (cd321x_check_setup(container_of(tps, struct cd321x, tps), event) < 0)
		goto err_unlock;
	/* Read the new detector result before the cable state is applied. */
	if (event & SN201202X_INT_LDCM_UPDATE && tps->contaminant_ops)
		tps->contaminant_ops->refresh(tps);

	if (!tps6598x_read_status(tps, &status))
		goto err_lost;

	if (event & APPLE_CD_REG_INT_POWER_STATUS_UPDATE) {
		if (!tps->data->read_power_status(tps))
			goto err_lost;
	}
	/* The identity can arrive after the contract, before a mode is entered. */
	if ((event & (APPLE_CD_REG_INT_POWER_STATUS_UPDATE |
		      APPLE_CD_REG_INT_DATA_STATUS_UPDATE)) &&
	    TPS_POWER_STATUS_PWROPMODE(tps->pwr_status) == TYPEC_PWR_MODE_PD) {
		if (tps6598x_read_partner_identity(tps)) {
			dev_err(tps->dev, "failed to read partner identity\n");
			goto err_lost;
		}
	}

	if (event & (APPLE_CD_REG_INT_DATA_STATUS_UPDATE |
		     (cd321x_irq_mask(tps) & SN201202X_INT_DP_SID_UPDATE)))
		if (!tps->data->read_data_status(tps))
			goto err_lost;
	if (event & cd321x_irq_mask(tps) & SN201202X_INT_DP_SID_UPDATE)
		cd321x_dp_sid_event(container_of(tps, struct cd321x, tps));

	/* Can be called uncondtionally since it will check for any changes itself */
	cd321x_connect(tps, status);
	goto err_unlock;

err_lost:
	cd321x_event_lost(cd321x);
err_unlock:
	mutex_unlock(&tps->lock);

	if (event)
		return IRQ_HANDLED;
	return IRQ_NONE;
}

static bool tps6598x_has_role_changed(struct tps6598x *tps, u32 status)
{
	status ^= tps->status;

	return status & (TPS_STATUS_PORTROLE | TPS_STATUS_DATAROLE);
}

static irqreturn_t tps25750_interrupt(int irq, void *data)
{
	struct tps6598x *tps = data;
	u64 event[2] = { };
	u32 status;
	int ret;

	mutex_lock(&tps->lock);

	ret = tps6598x_block_read(tps, TPS_REG_INT_EVENT1, event, 11);
	if (ret) {
		dev_err(tps->dev, "%s: failed to read events\n", __func__);
		goto err_unlock;
	}
	trace_tps25750_irq(event[0]);

	if (!(event[0] | event[1]))
		goto err_unlock;

	if (!tps6598x_read_status(tps, &status))
		goto err_clear_ints;

	if (event[0] & TPS_REG_INT_POWER_STATUS_UPDATE)
		if (!tps->data->read_power_status(tps))
			goto err_clear_ints;

	if (event[0] & TPS_REG_INT_DATA_STATUS_UPDATE)
		if (!tps->data->read_data_status(tps))
			goto err_clear_ints;

	/*
	 * data/port roles could be updated independently after
	 * a plug event. Therefore, we need to check
	 * for pr/dr status change to set TypeC dr/pr accordingly.
	 */
	if (event[0] & TPS_REG_INT_PLUG_EVENT ||
	    tps6598x_has_role_changed(tps, status))
		tps6598x_handle_plug_event(tps, status);

	tps->status = status;

err_clear_ints:
	tps6598x_block_write(tps, TPS_REG_INT_CLEAR1, event, 11);

err_unlock:
	mutex_unlock(&tps->lock);

	if (event[0] | event[1])
		return IRQ_HANDLED;
	return IRQ_NONE;
}

static irqreturn_t tps6598x_interrupt(int irq, void *data)
{
	int intev_len = TPS_65981_2_6_INTEVENT_LEN;
	struct tps6598x *tps = data;
	u64 event1[2] = { };
	u64 event2[2] = { };
	u32 version;
	u32 status;
	int ret;

	mutex_lock(&tps->lock);

	ret = tps6598x_read32(tps, TPS_REG_VERSION, &version);
	if (ret)
		dev_warn(tps->dev, "%s: failed to read version (%d)\n",
			 __func__, ret);

	if (TPS_VERSION_HW_VERSION(version) == TPS_VERSION_HW_65987_8_DH ||
	    TPS_VERSION_HW_VERSION(version) == TPS_VERSION_HW_65987_8_DK)
		intev_len = TPS_65987_8_INTEVENT_LEN;

	ret = tps6598x_block_read(tps, TPS_REG_INT_EVENT1, event1, intev_len);

	ret = tps6598x_block_read(tps, TPS_REG_INT_EVENT1, event1, intev_len);
	if (ret) {
		dev_err(tps->dev, "%s: failed to read event1\n", __func__);
		goto err_unlock;
	}
	ret = tps6598x_block_read(tps, TPS_REG_INT_EVENT2, event2, intev_len);
	if (ret) {
		dev_err(tps->dev, "%s: failed to read event2\n", __func__);
		goto err_unlock;
	}
	trace_tps6598x_irq(event1[0], event2[0]);

	if (!(event1[0] | event1[1] | event2[0] | event2[1]))
		goto err_unlock;

	tps6598x_block_write(tps, TPS_REG_INT_CLEAR1, event1, intev_len);
	tps6598x_block_write(tps, TPS_REG_INT_CLEAR2, event2, intev_len);

	if (!tps6598x_read_status(tps, &status))
		goto err_unlock;

	tps->status = status;

	if ((event1[0] | event2[0]) & TPS_REG_INT_POWER_STATUS_UPDATE)
		if (!tps->data->read_power_status(tps))
			goto err_unlock;

	if ((event1[0] | event2[0]) & TPS_REG_INT_DATA_STATUS_UPDATE)
		if (!tps->data->read_data_status(tps))
			goto err_unlock;

	/*
	 * Refresh power status before connect - needed for TPS66993 which
	 * synthesizes pwr_status from STATUS and never gets POWER_STATUS_UPDATE.
	 */
	if ((event1[0] | event2[0]) & TPS_REG_INT_PLUG_EVENT) {
		if (!tps->data->read_power_status(tps))
			goto err_unlock;
		tps6598x_handle_plug_event(tps, status);
	}

err_unlock:
	mutex_unlock(&tps->lock);

	if (event1[0] | event1[1] | event2[0] | event2[1])
		return IRQ_HANDLED;

	return IRQ_NONE;
}

/* Time interval for Polling */
#define POLL_INTERVAL	500 /* msecs */
static void tps6598x_poll_work(struct work_struct *work)
{
	struct tps6598x *tps = container_of(to_delayed_work(work),
					    struct tps6598x, wq_poll);

	tps->data->irq_handler(0, tps);
	queue_delayed_work(system_power_efficient_wq,
			   &tps->wq_poll, msecs_to_jiffies(POLL_INTERVAL));
}

static int tps6598x_check_mode(struct tps6598x *tps)
{
	char mode[5] = { };
	int ret;

	ret = tps6598x_read32(tps, TPS_REG_MODE, (void *)mode);
	if (ret)
		return ret;

	ret = match_string(modes, ARRAY_SIZE(modes), mode);

	switch (ret) {
	case TPS_MODE_APP:
	case TPS_MODE_APP1:
	case TPS_MODE_PTCH:
		return ret;
	case TPS_MODE_BOOT:
		dev_warn(tps->dev, "dead-battery condition\n");
		return ret;
	case TPS_MODE_DBMA:
		/*
		 * Apple firmware only: the application firmware runs in its
		 * debug mode (entered with LOCK + "DBMa"), in which it still
		 * handles the port and additionally accepts VDMs/DVEn. A boot
		 * loader arming Apple Debug USB on this port leaves it here.
		 */
		return ret;
	case TPS_MODE_BIST:
	case TPS_MODE_DISC:
	default:
		dev_err(tps->dev, "controller in unsupported mode \"%s\"\n",
			mode);
		break;
	}

	return -ENODEV;
}

const struct regmap_config tps6598x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7F,
};
EXPORT_SYMBOL_GPL(tps6598x_regmap_config);

static int tps6598x_psy_get_online(struct tps6598x *tps,
				   union power_supply_propval *val)
{
	if (TPS_POWER_STATUS_CONNECTION(tps->pwr_status) &&
	    TPS_POWER_STATUS_SOURCESINK(tps->pwr_status)) {
		val->intval = 1;
	} else {
		val->intval = 0;
	}
	return 0;
}

static int tps6598x_psy_get_prop(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct tps6598x *tps = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_USB_TYPE:
		if (TPS_POWER_STATUS_PWROPMODE(tps->pwr_status) == TYPEC_PWR_MODE_PD)
			val->intval = POWER_SUPPLY_USB_TYPE_PD;
		else
			val->intval = POWER_SUPPLY_USB_TYPE_C;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = tps6598x_psy_get_online(tps, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * The gap between the reads that wait for the end of an SN201202x's system power state command,
 * under the sleep's rules. Each read of the command register is a register selection of its
 * own, which the controller answers by loading the register again: read back to back there is
 * nothing between the loads, and on the J714s the command then ended late, or not within its
 * limit.
 */
#define CD321X_SSPS_READ_GAP_US	5000

static int cd321x_switch_power_state(struct tps6598x *tps, u8 target_state)
{
	u8 state;
	int ret;

	/*
	 * Under the sleep's rules the command goes to a controller that has just been sent its
	 * bus's wake command and has answered it, whatever it was doing: woken, read, commanded.
	 */
	if (tps->wake && tipd_sleep_ssps_rules(tps)) {
		ret = tps->wake(tps);
		if (ret)
			return ret;
	}

	ret = tps6598x_read8(tps, TPS_REG_SYSTEM_POWER_STATE, &state);
	if (ret)
		return ret;

	if (state == target_state)
		return 0;

	/*
	 * SN201202x can still be negotiating an attached dock while waking: 5 s for S0 outside
	 * the sleep's rules. Under them S0 and S3 have the sleep's own limit and are read with
	 * gaps.
	 */
	if ((target_state == TPS_SYSTEM_POWER_STATE_S0 || tipd_sleep_ssps_rules(tps)) &&
	    device_is_compatible(tps->dev, "apple,sn201202x"))
		ret = tps6598x_exec_cmd_gap(tps, "SSPS", sizeof(target_state),
					  &target_state, 0, NULL,
					  tipd_sleep_ssps_rules(tps) ?
					  CD321X_SLEEP_SSPS_LIMIT_MS : 5000, 0,
					  tipd_sleep_ssps_rules(tps) ? CD321X_SSPS_READ_GAP_US : 0);
	else
		ret = tps6598x_exec_cmd(tps, "SSPS", sizeof(target_state),
				      &target_state, 0, NULL);
	if (ret)
		return ret;

	ret = tps6598x_read8(tps, TPS_REG_SYSTEM_POWER_STATE, &state);
	if (ret)
		return ret;

	if (state != target_state)
		return -EINVAL;

	return 0;
}

static int devm_tps6598_psy_register(struct tps6598x *tps)
{
	struct power_supply_config psy_cfg = {};
	const char *port_dev_name = dev_name(tps->dev);
	char *psy_name;

	psy_cfg.drv_data = tps;
	psy_cfg.fwnode = dev_fwnode(tps->dev);

	psy_name = devm_kasprintf(tps->dev, GFP_KERNEL, "%s%s", tps6598x_psy_name_prefix,
				  port_dev_name);
	if (!psy_name)
		return -ENOMEM;

	tps->psy_desc.name = psy_name;
	tps->psy_desc.type = POWER_SUPPLY_TYPE_USB;
	tps->psy_desc.usb_types = BIT(POWER_SUPPLY_USB_TYPE_C) |
				  BIT(POWER_SUPPLY_USB_TYPE_PD);
	tps->psy_desc.properties = tps6598x_psy_props;
	tps->psy_desc.num_properties = ARRAY_SIZE(tps6598x_psy_props);
	tps->psy_desc.get_property = tps6598x_psy_get_prop;

	tps->usb_type = POWER_SUPPLY_USB_TYPE_C;

	tps->psy = devm_power_supply_register(tps->dev, &tps->psy_desc,
					       &psy_cfg);
	return PTR_ERR_OR_ZERO(tps->psy);
}

static int
tps6598x_register_port(struct tps6598x *tps, struct fwnode_handle *fwnode)
{
	int ret;
	u32 conf;
	struct typec_capability typec_cap = { };

	ret = tps6598x_read32(tps, TPS_REG_SYSTEM_CONF, &conf);
	if (ret)
		return ret;

	typec_cap.revision = USB_TYPEC_REV_1_2;
	typec_cap.pd_revision = 0x200;
	typec_cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;
	typec_cap.driver_data = tps;
	typec_cap.ops = &tps6598x_ops;
	typec_cap.fwnode = fwnode;
	typec_cap.contaminant_detection = !!tps->contaminant_ops;

	switch (TPS_SYSCONF_PORTINFO(conf)) {
	case TPS_PORTINFO_SINK_ACCESSORY:
	case TPS_PORTINFO_SINK:
		typec_cap.type = TYPEC_PORT_SNK;
		typec_cap.data = TYPEC_PORT_UFP;
		break;
	case TPS_PORTINFO_DRP_UFP_DRD:
	case TPS_PORTINFO_DRP_DFP_DRD:
		typec_cap.type = TYPEC_PORT_DRP;
		typec_cap.data = TYPEC_PORT_DRD;
		break;
	case TPS_PORTINFO_DRP_UFP:
		typec_cap.type = TYPEC_PORT_DRP;
		typec_cap.data = TYPEC_PORT_UFP;
		break;
	case TPS_PORTINFO_DRP_DFP:
		typec_cap.type = TYPEC_PORT_DRP;
		typec_cap.data = TYPEC_PORT_DFP;
		break;
	case TPS_PORTINFO_SOURCE:
		typec_cap.type = TYPEC_PORT_SRC;
		typec_cap.data = TYPEC_PORT_DFP;
		break;
	default:
		return -ENODEV;
	}

	tps->port = typec_register_port(tps->dev, &typec_cap);
	if (IS_ERR(tps->port))
		return PTR_ERR(tps->port);

	return 0;
}

static int cd321x_register_port_altmodes(struct cd321x *cd321x)
{
	struct typec_altmode_desc desc;
	struct typec_altmode *amode;

	memset(&desc, 0, sizeof(desc));
	desc.svid = USB_TYPEC_DP_SID;
	desc.mode = USB_TYPEC_DP_MODE;
	desc.vdo = DP_CONF_SET_PIN_ASSIGN(BIT(DP_PIN_ASSIGN_C) |
					   BIT(DP_PIN_ASSIGN_D) | BIT(DP_PIN_ASSIGN_E));
	desc.vdo |= DP_CAP_DFP_D;
	amode = typec_port_register_altmode(cd321x->tps.port, &desc);
	if (IS_ERR(amode))
		return PTR_ERR(amode);
	cd321x->port_altmode_dp = amode;

	memset(&desc, 0, sizeof(desc));
	desc.svid = USB_TYPEC_TBT_SID;
	desc.mode = TYPEC_ANY_MODE;
	amode = typec_port_register_altmode(cd321x->tps.port, &desc);
	if (IS_ERR(amode)) {
		typec_unregister_altmode(cd321x->port_altmode_dp);
		cd321x->port_altmode_dp = NULL;
		return PTR_ERR(amode);
	}
	cd321x->port_altmode_tbt = amode;

	return 0;
}

static int cd321x_tbt_notify(struct notifier_block *nb,
			     unsigned long event, void *data)
{
	struct cd321x *cd321x = container_of(nb, struct cd321x, tbt_notifier);

	if (event != TYPEC_THUNDERBOLT_SWITCH_REVALIDATE &&
	    event != TYPEC_THUNDERBOLT_SWITCH_READY &&
	    event != TYPEC_THUNDERBOLT_SWITCH_RESTORE)
		return NOTIFY_DONE;

	/* Publish the event even if the worker is already reading or applying. */
	if (event == TYPEC_THUNDERBOLT_SWITCH_READY)
		atomic_set(&cd321x->ready_event, 1);
	else if (event == TYPEC_THUNDERBOLT_SWITCH_RESTORE)
		atomic_set(&cd321x->restore_event, 1);
	else
		atomic_set(&cd321x->link_event, 1);
	mod_delayed_work(system_freezable_wq, &cd321x->resume_work, 0);
	return NOTIFY_OK;
}

static void cd321x_ready(struct tps6598x *tps)
{
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);

	guard(mutex)(&tps->lock);
	/* Include failures reported while the port was still being initialized. */
	if (cd321x_pm_ready(&cd321x->pm)) {
		dev_dbg(tps->dev, "Type-C probe ready: held=%u\n",
			!!cd321x->pm.mux_retry_ms);
		cd321x->state_valid = false;
		mod_delayed_work(system_freezable_wq, &cd321x->resume_work, 0);
	}
}

VISIBLE_IF_KUNIT void cd321x_stop_work(struct cd321x *cd321x)
{
	scoped_guard(mutex, &cd321x->tps.lock)
		cd321x_pm_remove(&cd321x->pm);

	/* A router that binds from now on must not hand this port its switch. */
	if (cd321x->tbt_watch_registered) {
		typec_thunderbolt_switch_unwatch(&cd321x->tbt_watch);
		cd321x->tbt_watch_registered = false;
	}
	cancel_work_sync(&cd321x->tbt_attach_work);

	/* Drain producers before cancelling work, including on probe rollback. */
	if (cd321x->tbt_notifier_registered) {
		typec_thunderbolt_switch_unregister_notifier(cd321x->tbt_switch,
							     &cd321x->tbt_notifier);
		cd321x->tbt_notifier_registered = false;
	}
	cancel_delayed_work_sync(&cd321x->resume_work);
	cancel_delayed_work_sync(&cd321x->update_work);
	cancel_delayed_work_sync(&cd321x->rearm_work);
}
EXPORT_SYMBOL_IF_KUNIT(cd321x_stop_work);

/*
 * Close the port's USB4/Thunderbolt session while the system still runs. A
 * cable event that closes it inside device_shutdown() makes pid 1 wait for the
 * PCIe-C reset handshake of an ACIO that is already going down. Stopping the
 * work first keeps every later event away from the router.
 */
VISIBLE_IF_KUNIT int cd321x_reboot_close(struct cd321x *cd321x)
{
	struct typec_thunderbolt_switch_data off = {
		.state = TYPEC_THUNDERBOLT_SWITCH_OFF,
	};

	cd321x_stop_work(cd321x);
	return typec_thunderbolt_switch_set(cd321x->tbt_switch, &off);
}
EXPORT_SYMBOL_IF_KUNIT(cd321x_reboot_close);

VISIBLE_IF_KUNIT int cd321x_reboot_notify(struct notifier_block *nb,
					  unsigned long action, void *data)
{
	struct cd321x *cd321x = container_of(nb, struct cd321x, reboot_nb);
	struct tps6598x *tps = &cd321x->tps;
	int ret;

	if (action != SYS_RESTART && action != SYS_HALT && action != SYS_POWER_OFF)
		return NOTIFY_DONE;

	if (tps->irq)
		disable_irq(tps->irq);
	if (cd321x->tbt_switch)
		dev_info(tps->dev, "closing the USB4/Thunderbolt session before shutdown\n");
	ret = cd321x_reboot_close(cd321x);
	if (cd321x->tbt_switch)
		dev_info(tps->dev, "closed the USB4/Thunderbolt session before shutdown: %d\n",
			 ret);
	return NOTIFY_OK;
}
EXPORT_SYMBOL_IF_KUNIT(cd321x_reboot_notify);

/* Separate so that the KUnit tests can choose the driver core's answer. */
VISIBLE_IF_KUNIT bool cd321x_tbt_switch_keep_waiting(struct device *dev)
{
	KUNIT_STATIC_STUB_REDIRECT(cd321x_tbt_switch_keep_waiting, dev);

	return driver_deferred_probe_check_state(dev) == -EPROBE_DEFER;
}
EXPORT_SYMBOL_IF_KUNIT(cd321x_tbt_switch_keep_waiting);

/*
 * The USB4/Thunderbolt router is only needed once a USB4 or Thunderbolt link
 * comes up. If its driver is not built or has not bound yet, do not leave power
 * delivery, USB and DisplayPort on this port without a driver as well: stop
 * waiting for the router when the driver core stops waiting for missing
 * dependencies. The router can still bind later, so set @late to take it then.
 */
VISIBLE_IF_KUNIT struct typec_thunderbolt_switch *
cd321x_tbt_switch_get(struct device *dev, struct fwnode_handle *fwnode,
		      bool *late)
{
	struct typec_thunderbolt_switch *sw;

	*late = false;
	sw = fwnode_typec_thunderbolt_switch_get(fwnode);
	if (sw != ERR_PTR(-EPROBE_DEFER) || cd321x_tbt_switch_keep_waiting(dev))
		return sw;

	dev_info(dev, "USB4/Thunderbolt router unavailable, continuing without it until it binds\n");
	*late = true;
	return NULL;
}
EXPORT_SYMBOL_IF_KUNIT(cd321x_tbt_switch_get);

/*
 * Take a router that bound after the port gave up on it, or that unbound and
 * bound again. An attached cable raises no new event, so reapply it through
 * the router as after system sleep.
 */
static void cd321x_tbt_attach_work(struct work_struct *work)
{
	struct cd321x *cd321x = container_of(work, struct cd321x,
					    tbt_attach_work);
	struct tps6598x *tps = &cd321x->tps;
	struct typec_thunderbolt_switch *sw, *old;
	int ret;

	scoped_guard(mutex, &tps->lock) {
		if (cd321x->pm.phase == CD321X_PM_REMOVED)
			return;
		/* The router this port uses is still bound. */
		if (typec_thunderbolt_switch_is_live(cd321x->tbt_switch))
			return;
		old = cd321x->tbt_switch;
	}

	/* Another port's router, or this one is not usable yet: keep watching. */
	sw = fwnode_typec_thunderbolt_switch_get(cd321x->tbt_fwnode);
	if (IS_ERR_OR_NULL(sw))
		return;

	/* The router went away: its switch sends nothing more, stop listening. */
	if (old && cd321x->tbt_notifier_registered) {
		typec_thunderbolt_switch_unregister_notifier(old,
							     &cd321x->tbt_notifier);
		cd321x->tbt_notifier_registered = false;
	}

	ret = typec_thunderbolt_switch_register_notifier(sw,
							 &cd321x->tbt_notifier);
	if (ret) {
		typec_thunderbolt_switch_put(sw);
		dev_warn(tps->dev,
			 "failed to watch the USB4/Thunderbolt router: %d\n", ret);
		return;
	}

	scoped_guard(mutex, &tps->lock) {
		cd321x->tbt_switch = sw;
		cd321x->tbt_notifier_registered = true;
	}
	typec_thunderbolt_switch_put(old);
	dev_info(tps->dev, "USB4/Thunderbolt router bound%s, reapplying the cable state\n",
		 old ? " again" : "");
	atomic_set(&cd321x->restore_event, 1);
	mod_delayed_work(system_freezable_wq, &cd321x->resume_work, 0);
}

/* Runs under the registering router's locks, so only queue the lookup. */
static int cd321x_tbt_watch(struct notifier_block *nb, unsigned long event,
			    void *data)
{
	struct cd321x *cd321x = container_of(nb, struct cd321x, tbt_watch);

	if (event != TYPEC_THUNDERBOLT_SWITCH_ADDED)
		return NOTIFY_DONE;

	queue_work(system_freezable_wq, &cd321x->tbt_attach_work);
	return NOTIFY_OK;
}

/* Watch for the router of a port that had to probe without it. */
VISIBLE_IF_KUNIT int cd321x_tbt_watch_late(struct cd321x *cd321x,
					   struct fwnode_handle *fwnode)
{
	int ret;

	cd321x->tbt_fwnode = fwnode_handle_get(fwnode);
	cd321x->tbt_watch.notifier_call = cd321x_tbt_watch;
	ret = typec_thunderbolt_switch_watch(&cd321x->tbt_watch);
	if (ret) {
		fwnode_handle_put(cd321x->tbt_fwnode);
		cd321x->tbt_fwnode = NULL;
		return ret;
	}
	cd321x->tbt_watch_registered = true;

	/* The router may have registered since the lookup at probe. */
	queue_work(system_freezable_wq, &cd321x->tbt_attach_work);
	return 0;
}
EXPORT_SYMBOL_IF_KUNIT(cd321x_tbt_watch_late);

/* Separate so that the KUnit tests can drive the workers without a port. */
VISIBLE_IF_KUNIT void cd321x_init_work(struct cd321x *cd321x)
{
	INIT_DELAYED_WORK(&cd321x->update_work, cd321x_update_work);
	INIT_DELAYED_WORK(&cd321x->resume_work, cd321x_resume_work);
	INIT_DELAYED_WORK(&cd321x->rearm_work, cd321x_rearm_work);
	INIT_WORK(&cd321x->tbt_attach_work, cd321x_tbt_attach_work);
	atomic_set(&cd321x->link_event, 0);
	atomic_set(&cd321x->ready_event, 0);
	atomic_set(&cd321x->restore_event, 0);
	cd321x_pm_init(&cd321x->pm);
}
EXPORT_SYMBOL_IF_KUNIT(cd321x_init_work);

/* Use the same handle as the worker: timeout provenance belongs to the handle. */
VISIBLE_IF_KUNIT int cd321x_init_mux(struct cd321x *cd321x)
{
	bool held;
	int ret;

	guard(mutex)(&cd321x->tps.lock);
	cd321x->state.alt = NULL;
	cd321x->state.mode = TYPEC_STATE_SAFE;
	cd321x->state.data = NULL;
	ret = cd321x_mux_set(cd321x, &held);
	cd321x->state_valid = !ret;
	if (ret == -EAGAIN)
		return -EPROBE_DEFER;
	if (held)
		cd321x_retry_held_mux(cd321x);

	return 0;
}
EXPORT_SYMBOL_IF_KUNIT(cd321x_init_mux);

/* The J613 profile enables its disabled DCP routes during DRM coldplug. */
static bool cd321x_display_gate_pending(struct tps6598x *tps)
{
	return apple_j613_display_pending(tps->dev);
}

static int
cd321x_register_port(struct tps6598x *tps, struct fwnode_handle *fwnode)
{
	struct fwnode_handle *connector_fwnode;
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);
	bool tbt_late;
	int ret;

	cd321x_init_work(cd321x);

	ret = tps6598x_register_port(tps, fwnode);
	if (ret)
		return ret;

	ret = cd321x_register_port_altmodes(cd321x);
	if (ret)
		goto err_unregister_port;

	cd321x->mux = fwnode_typec_mux_get(fwnode);
	if (IS_ERR(cd321x->mux)) {
		ret = PTR_ERR(cd321x->mux);
		goto err_unregister_altmodes;
	}

	if (fwnode_property_present(fwnode, "displayport"))
		connector_fwnode = fwnode_find_reference(fwnode, "displayport", 0);
	else
		connector_fwnode = fwnode_handle_get(fwnode);
	if (!IS_ERR_OR_NULL(connector_fwnode))
		cd321x->connector_fwnode = connector_fwnode;

	cd321x->tbt_switch = cd321x_tbt_switch_get(tps->dev, fwnode, &tbt_late);
	if (IS_ERR(cd321x->tbt_switch)) {
		ret = PTR_ERR(cd321x->tbt_switch);
		goto err_put_mux;
	}

	/*
	 * A fixed host can register its SuperSpeed root after the mux provider.
	 * Wait for that consumer at probe instead of exhausting the cable retry
	 * budget before it exists. Host startup does not depend on this port.
	 */
	ret = cd321x_init_mux(cd321x);
	if (ret)
		goto err_put_switch;

	cd321x->tbt_notifier.notifier_call = cd321x_tbt_notify;
	ret = typec_thunderbolt_switch_register_notifier(cd321x->tbt_switch,
							 &cd321x->tbt_notifier);
	if (ret)
		goto err_put_switch;
	cd321x->tbt_notifier_registered = true;

	/*
	 * Keep the port without the router rather than fail it: see above. A
	 * router that is there can also unbind and bind again, which gives it
	 * a new switch: watch for that as well.
	 */
	if ((tbt_late || cd321x->tbt_switch) &&
	    cd321x_tbt_watch_late(cd321x, fwnode))
		dev_warn(tps->dev, "cannot watch for the USB4/Thunderbolt router\n");

	cd321x->reboot_nb.notifier_call = cd321x_reboot_notify;
	cd321x->reboot_nb.priority = INT_MAX;
	if (register_reboot_notifier(&cd321x->reboot_nb))
		dev_warn(tps->dev, "cannot close the USB4/Thunderbolt session at shutdown\n");
	else
		cd321x->reboot_nb_registered = true;

	return 0;

err_put_switch:
	typec_thunderbolt_switch_put(cd321x->tbt_switch);
	cd321x->tbt_switch = NULL;
err_put_mux:
	fwnode_handle_put(cd321x->connector_fwnode);
	cd321x->connector_fwnode = NULL;
	typec_mux_put(cd321x->mux);
	cd321x->mux = NULL;
err_unregister_altmodes:
	typec_unregister_altmode(cd321x->port_altmode_dp);
	typec_unregister_altmode(cd321x->port_altmode_tbt);
	cd321x->port_altmode_dp = NULL;
	cd321x->port_altmode_tbt = NULL;
err_unregister_port:
	typec_unregister_port(tps->port);
	return ret;
}

static void
tps6598x_unregister_port(struct tps6598x *tps)
{
	typec_unregister_port(tps->port);
}

static void
cd321x_unregister_port(struct tps6598x *tps)
{
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);

	if (cd321x->reboot_nb_registered) {
		unregister_reboot_notifier(&cd321x->reboot_nb);
		cd321x->reboot_nb_registered = false;
	}

	cd321x_stop_work(cd321x);
	fwnode_handle_put(cd321x->connector_fwnode);
	cd321x->connector_fwnode = NULL;
	fwnode_handle_put(cd321x->tbt_fwnode);
	cd321x->tbt_fwnode = NULL;
	typec_thunderbolt_switch_put(cd321x->tbt_switch);
	cd321x->tbt_switch = NULL;
	typec_mux_put(cd321x->mux);
	cd321x->mux = NULL;
	typec_unregister_altmode(cd321x->port_altmode_dp);
	cd321x->port_altmode_dp = NULL;
	typec_unregister_altmode(cd321x->port_altmode_tbt);
	cd321x->port_altmode_tbt = NULL;
	typec_unregister_port(tps->port);
}

static int tps_request_firmware(struct tps6598x *tps, const struct firmware **fw,
				const char **firmware_name)
{
	int ret;

	ret = device_property_read_string(tps->dev, "firmware-name",
					  firmware_name);
	if (ret)
		return ret;

	ret = request_firmware(fw, *firmware_name, tps->dev);
	if (ret) {
		dev_err(tps->dev, "failed to retrieve \"%s\"\n", *firmware_name);
		return ret;
	}

	if ((*fw)->size == 0) {
		release_firmware(*fw);
		ret = -EINVAL;
	}

	return ret;
}

static int
tps25750_write_firmware(struct tps6598x *tps,
			u8 bpms_addr, const u8 *data, size_t len)
{
	struct i2c_client *client = to_i2c_client(tps->dev);
	int ret;
	u8 slave_addr;
	int timeout;

	slave_addr = client->addr;
	timeout = client->adapter->timeout;

	/*
	 * binary configuration size is around ~16Kbytes
	 * which might take some time to finish writing it
	 */
	client->adapter->timeout = msecs_to_jiffies(5000);
	client->addr = bpms_addr;

	ret = regmap_raw_write(tps->regmap, data[0], &data[1], len - 1);

	client->addr = slave_addr;
	client->adapter->timeout = timeout;

	return ret;
}

static int
tps25750_exec_pbms(struct tps6598x *tps, u8 *in_data, size_t in_len)
{
	int ret;
	u8 rc;

	ret = tps6598x_exec_cmd_tmo(tps, "PBMs", in_len, in_data,
				    sizeof(rc), &rc, 4000, 0);
	if (ret)
		return ret;

	switch (rc) {
	case TPS_TASK_BPMS_INVALID_BUNDLE_SIZE:
		dev_err(tps->dev, "%s: invalid fw size\n", __func__);
		return -EINVAL;
	case TPS_TASK_BPMS_INVALID_SLAVE_ADDR:
		dev_err(tps->dev, "%s: invalid slave address\n", __func__);
		return -EINVAL;
	case TPS_TASK_BPMS_INVALID_TIMEOUT:
		dev_err(tps->dev, "%s: timed out\n", __func__);
		return -ETIMEDOUT;
	default:
		break;
	}

	return 0;
}

static int tps25750_abort_patch_process(struct tps6598x *tps)
{
	int ret;

	ret = tps6598x_exec_cmd(tps, "PBMe", 0, NULL, 0, NULL);
	if (ret)
		return ret;

	ret = tps6598x_check_mode(tps);
	if (ret != TPS_MODE_PTCH)
		dev_err(tps->dev, "failed to switch to \"PTCH\" mode\n");

	return ret;
}

static int tps25750_start_patch_burst_mode(struct tps6598x *tps)
{
	int ret;
	const struct firmware *fw;
	const char *firmware_name;
	struct {
		u32 fw_size;
		u8 addr;
		u8 timeout;
	} __packed bpms_data;
	u32 addr;
	struct device_node *np = tps->dev->of_node;

	ret = tps_request_firmware(tps, &fw, &firmware_name);
	if (ret)
		return ret;

	ret = of_property_match_string(np, "reg-names", "patch-address");
	if (ret < 0) {
		dev_err(tps->dev, "failed to get patch-address %d\n", ret);
		goto release_fw;
	}

	ret = of_property_read_u32_index(np, "reg", ret, &addr);
	if (ret)
		goto release_fw;

	if (addr == 0 || (addr >= 0x20 && addr <= 0x23)) {
		dev_err(tps->dev, "wrong patch address %u\n", addr);
		ret = -EINVAL;
		goto release_fw;
	}

	bpms_data.addr = (u8)addr;
	bpms_data.fw_size = fw->size;
	bpms_data.timeout = TPS_BUNDLE_TIMEOUT;

	ret = tps25750_exec_pbms(tps, (u8 *)&bpms_data, sizeof(bpms_data));
	if (ret)
		goto release_fw;

	ret = tps25750_write_firmware(tps, bpms_data.addr, fw->data, fw->size);
	if (ret) {
		dev_err(tps->dev, "Failed to write patch %s of %zu bytes\n",
			firmware_name, fw->size);
		goto release_fw;
	}

	/*
	 * A delay of 500us is required after the firmware is written
	 * based on pg.62 in tps6598x Host Interface Technical
	 * Reference Manual
	 * https://www.ti.com/lit/ug/slvuc05a/slvuc05a.pdf
	 */
	udelay(500);

release_fw:
	release_firmware(fw);

	return ret;
}

static int tps25750_complete_patch_process(struct tps6598x *tps)
{
	int ret;
	u8 out_data[40];
	u8 dummy[2] = { };

	/*
	 * Without writing something to DATA_IN, this command would
	 * return an error
	 */
	ret = tps6598x_exec_cmd_tmo(tps, "PBMc", sizeof(dummy), dummy,
				    sizeof(out_data), out_data, 2000, 20);
	if (ret)
		return ret;

	if (out_data[TPS_PBMC_RC]) {
		dev_err(tps->dev,
			"%s: pbmc failed: %u\n", __func__,
			out_data[TPS_PBMC_RC]);
		return -EIO;
	}

	if (out_data[TPS_PBMC_DPCS]) {
		dev_err(tps->dev,
			"%s: failed device patch complete status: %u\n",
			__func__, out_data[TPS_PBMC_DPCS]);
		return -EIO;
	}

	return 0;
}

static int tps25750_apply_patch(struct tps6598x *tps)
{
	int ret;
	unsigned long timeout;
	u64 status = 0;

	ret = tps6598x_block_read(tps, TPS_REG_BOOT_STATUS, &status, 5);
	if (ret)
		return ret;
	/*
	 * Nothing to be done if the configuration
	 * is being loaded from EERPOM
	 */
	if (status & TPS_BOOT_STATUS_I2C_EEPROM_PRESENT)
		goto wait_for_app;

	ret = tps25750_start_patch_burst_mode(tps);
	if (ret) {
		tps25750_abort_patch_process(tps);
		return ret;
	}

	ret = tps25750_complete_patch_process(tps);
	if (ret)
		return ret;

wait_for_app:
	timeout = jiffies + msecs_to_jiffies(1000);

	do {
		ret = tps6598x_check_mode(tps);
		if (ret < 0)
			return ret;

		if (time_is_before_jiffies(timeout))
			return -ETIMEDOUT;

	} while (ret != TPS_MODE_APP);

	/*
	 * The dead battery flag may be triggered when the controller
	 * port is connected to a device that can source power and
	 * attempts to power up both the controller and the board it is on.
	 * To restore controller functionality, it is necessary to clear
	 * this flag
	 */
	if (status & TPS_BOOT_STATUS_DEAD_BATTERY_FLAG) {
		ret = tps6598x_exec_cmd(tps, "DBfg", 0, NULL, 0, NULL);
		if (ret) {
			dev_err(tps->dev, "failed to clear dead battery %d\n", ret);
			return ret;
		}
	}

	dev_info(tps->dev, "controller switched to \"APP\" mode\n");

	return 0;
};

static int tps6598x_apply_patch(struct tps6598x *tps)
{
	u8 in = TPS_PTCS_CONTENT_DEV | TPS_PTCS_CONTENT_APP;
	u8 out[TPS_MAX_LEN] = {0};
	size_t in_len = sizeof(in);
	size_t copied_bytes = 0;
	size_t bytes_left;
	const struct firmware *fw;
	const char *firmware_name;
	int ret;

	ret = tps_request_firmware(tps, &fw, &firmware_name);
	if (ret)
		return ret;

	ret = tps6598x_exec_cmd(tps, "PTCs", in_len, &in,
				TPS_PTCS_OUT_BYTES, out);
	if (ret || out[TPS_PTCS_STATUS] == TPS_PTCS_STATUS_FAIL) {
		if (!ret)
			ret = -EBUSY;
		dev_err(tps->dev, "Update start failed (%d)\n", ret);
		goto release_fw;
	}

	bytes_left = fw->size;
	while (bytes_left) {
		in_len = min(bytes_left, TPS_MAX_LEN);
		ret = tps6598x_exec_cmd(tps, "PTCd", in_len,
					fw->data + copied_bytes,
					TPS_PTCD_OUT_BYTES, out);
		if (ret || out[TPS_PTCD_TRANSFER_STATUS] ||
		    out[TPS_PTCD_LOADING_STATE] == TPS_PTCD_LOAD_ERR) {
			if (!ret)
				ret = -EBUSY;
			dev_err(tps->dev, "Patch download failed (%d)\n", ret);
			goto release_fw;
		}
		copied_bytes += in_len;
		bytes_left -= in_len;
	}

	ret = tps6598x_exec_cmd(tps, "PTCc", 0, NULL, TPS_PTCC_OUT_BYTES, out);
	if (ret || out[TPS_PTCC_DEV] || out[TPS_PTCC_APP]) {
		if (!ret)
			ret = -EBUSY;
		dev_err(tps->dev, "Update completion failed (%d)\n", ret);
		goto release_fw;
	}
	msleep(TPS_SETUP_MS);
	dev_info(tps->dev, "Firmware update succeeded\n");

release_fw:
	if (ret) {
		dev_err(tps->dev, "Failed to write patch %s of %zu bytes\n",
			firmware_name, fw->size);
	}
	release_firmware(fw);

	return ret;
}

static int cd321x_init(struct tps6598x *tps)
{
	return 0;
}

static int tps25750_init(struct tps6598x *tps)
{
	int ret;

	ret = tps->data->apply_patch(tps);
	if (ret)
		return ret;

	ret = tps6598x_write8(tps, TPS_REG_SLEEP_CONF,
			      TPS_SLEEP_CONF_SLEEP_MODE_ALLOWED);
	if (ret)
		dev_warn(tps->dev,
			 "%s: failed to enable sleep mode: %d\n",
			 __func__, ret);

	return 0;
}

static int tps6598x_init(struct tps6598x *tps)
{
	return tps->data->apply_patch(tps);
}

static int cd321x_reset(struct tps6598x *tps)
{
	return 0;
}

static int tps25750_reset(struct tps6598x *tps)
{
	return tps6598x_exec_cmd_tmo(tps, "GAID", 0, NULL, 0, NULL, 2000, 0);
}

static int tps6598x_reset(struct tps6598x *tps)
{
	return 0;
}

static int
tps25750_register_port(struct tps6598x *tps, struct fwnode_handle *fwnode)
{
	struct typec_capability typec_cap = { };
	const char *data_role;
	u8 pd_status;
	int ret;

	ret = tps6598x_read8(tps, TPS_REG_PD_STATUS, &pd_status);
	if (ret)
		return ret;

	ret = fwnode_property_read_string(fwnode, "data-role", &data_role);
	if (ret) {
		dev_err(tps->dev, "data-role not found: %d\n", ret);
		return ret;
	}

	ret = typec_find_port_data_role(data_role);
	if (ret < 0) {
		dev_err(tps->dev, "unknown data-role: %s\n", data_role);
		return ret;
	}

	typec_cap.data = ret;
	typec_cap.revision = USB_TYPEC_REV_1_3;
	typec_cap.pd_revision = 0x300;
	typec_cap.orientation_aware = true;
	typec_cap.driver_data = tps;
	typec_cap.ops = &tps6598x_ops;
	typec_cap.fwnode = fwnode;
	typec_cap.prefer_role = TYPEC_NO_PREFERRED_ROLE;

	switch (TPS_PD_STATUS_PORT_TYPE(pd_status)) {
	case TPS_PD_STATUS_PORT_TYPE_SINK_SOURCE:
	case TPS_PD_STATUS_PORT_TYPE_SOURCE_SINK:
		typec_cap.type = TYPEC_PORT_DRP;
		break;
	case TPS_PD_STATUS_PORT_TYPE_SINK:
		typec_cap.type = TYPEC_PORT_SNK;
		break;
	case TPS_PD_STATUS_PORT_TYPE_SOURCE:
		typec_cap.type = TYPEC_PORT_SRC;
		break;
	default:
		return -ENODEV;
	}

	tps->port = typec_register_port(tps->dev, &typec_cap);
	if (IS_ERR(tps->port))
		return PTR_ERR(tps->port);

	return 0;
}

static int cd321x_remove(struct tps6598x *tps)
{
	struct cd321x *cd321x = container_of(tps, struct cd321x, tps);
	struct typec_thunderbolt_switch_data tbt_switch_data = {
		.state = TYPEC_THUNDERBOLT_SWITCH_OFF,
	};
	struct typec_mux_state safe = { .mode = TYPEC_STATE_SAFE };
	int ret;

	cd321x_stop_work(cd321x);

	/*
	 * Driver teardown must close the same hardware sessions as a physical
	 * disconnect.  Otherwise ACIO retains the old USB4 cable state while a
	 * reprobe enters a new session, which its firmware treats as an invalid
	 * live-to-live transition and resolves by resetting the SoC.
	 *
	 * Close ACIO first while the ATC lanes can still carry PCIe-C's tunneled
	 * reset handshake, then drop the USB role after NHI and PCIe-C teardown.
	 */
	ret = typec_thunderbolt_switch_set(cd321x->tbt_switch,
					   &tbt_switch_data);
	if (ret)
		return ret;
	/* Display quiescence must precede even an ordinary host-role shutdown. */
	typec_set_orientation(tps->port, TYPEC_ORIENTATION_NONE);
	ret = typec_mux_begin(tps->mux, &safe);
	if (ret)
		return ret;
	/*
	 * Ordinary PHYs need role shutdown before SAFE. A fixed host must park
	 * now while its controller supplies the PIPE backend clock, even when
	 * the firmware graph exposes a no-op role switch for that host.
	 */
	if (!tps6598x_safe_after_role(tps)) {
		ret = typec_mux_apply(tps->mux);
		if (ret)
			return ret;
	}
	ret = usb_role_switch_set_role(tps->role_sw, USB_ROLE_NONE);
	/* An unregistered switch means the controller is already stopped. */
	if (ret && (ret != -EOPNOTSUPP ||
		    usb_role_switch_get_role(tps->role_sw) != USB_ROLE_NONE)) {
		/* Keep recovery ownership; a live PHY must not receive SAFE. */
		typec_mux_abort(tps->mux);
		return ret;
	}
	/* Leave the SAFE transaction open for tps6598x_disconnect() to end. */
	tps->remove_safe_prepared = true;
	return 0;
}

/*
 * Boards whose inherited boot connection must be renegotiated. The port
 * controller key is the reversed target identifier.
 *
 * J716c: the boot port keeps a USB2 repeater connection.
 * J575c: a connection that was up before Linux keeps its Type-C retimer set up
 * from before the host PHY ran, so SuperSpeed stays in RxDetect until the
 * partner is replugged. Byte 7 of register 0x7e has bit 3 set once the
 * controller has completed an attach with the OS running (after a replug, and
 * under macOS) and bit 2 in the inherited state; renegotiate only without
 * bit 3. A failure leaves the port as it was (USB2 only) instead of failing
 * the probe.
 */
#define CD321X_ATTACH_STATE_REG		0x7e
#define CD321X_ATTACH_STATE_BYTE	7
#define CD321X_ATTACH_STATE_DONE	BIT(3)

struct cd321x_inherited_board {
	const char *compatible;
	u8 key[4];
	bool check_attach_state;
	bool best_effort;
};

static const struct cd321x_inherited_board cd321x_inherited_boards[] = {
	{ .compatible = "apple,j716c", .key = { '6', '1', '7', 'J' } },
	{ .compatible = "apple,j575c", .key = { '5', '7', '5', 'J' },
	  .check_attach_state = true, .best_effort = true },
};

static const struct cd321x_inherited_board *cd321x_inherited_board(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cd321x_inherited_boards); i++)
		if (of_machine_is_compatible(cd321x_inherited_boards[i].compatible))
			return &cd321x_inherited_boards[i];
	return NULL;
}

static int cd321x_attach_state_done(struct tps6598x *tps)
{
	u8 state[CD321X_ATTACH_STATE_BYTE + 1];
	int ret;

	ret = tps6598x_block_read(tps, CD321X_ATTACH_STATE_REG, state, sizeof(state));
	if (ret)
		return ret;
	dev_dbg(tps->dev, "inherited connection, attach state 0x%02x\n",
		state[CD321X_ATTACH_STATE_BYTE]);
	return !!(state[CD321X_ATTACH_STATE_BYTE] & CD321X_ATTACH_STATE_DONE);
}

static int __cd321x_reset_inherited_connection(struct tps6598x *tps,
						const struct cd321x_inherited_board *board,
						u32 *status)
{
	static const u8 locked[4];
	u8 delay = 5, result;
	unsigned long timeout;
	int ret, lock_ret, mode;

	mode = tps6598x_check_mode(tps);
	if (mode != TPS_MODE_APP && mode != TPS_MODE_APP1)
		return 0;
	if (board->check_attach_state) {
		ret = cd321x_attach_state_done(tps);
		if (ret)
			return ret < 0 ? ret : 0;
	}

	/* All suppliers are available, but no USB or router session has started. */
	ret = tps6598x_exec_cmd(tps, "LOCK", sizeof(board->key), board->key, sizeof(result),
				&result);
	if (ret)
		return ret;
	if (result) {
		ret = -EIO;
		goto relock;
	}

	/*
	 * Nonzero DISC delay restores normal CC detection automatically. The
	 * task can take more than a second to acknowledge at boot; wait for it
	 * before issuing LOCK, which a still-busy controller would reject.
	 */
	ret = tps6598x_exec_cmd_tmo(tps, "DISC", sizeof(delay), &delay, 0, NULL,
				  5000, 0);
	if (!ret) {
		msleep(5100);
		timeout = jiffies + msecs_to_jiffies(2000);
		do {
			mode = tps6598x_check_mode(tps);
			if (mode == TPS_MODE_APP || mode == TPS_MODE_APP1)
				break;
			msleep(20);
		} while (time_before(jiffies, timeout));
		if (mode != TPS_MODE_APP && mode != TPS_MODE_APP1)
			ret = mode < 0 ? mode : -ETIMEDOUT;
	}
relock:
	/* CC/PD negotiation may still be finishing when application mode returns. */
	lock_ret = tps6598x_exec_cmd_tmo(tps, "LOCK", sizeof(locked), locked, 0, NULL,
				       5000, 0);
	if (ret || lock_ret)
		return ret ?: lock_ret;
	if (!tps6598x_read_status(tps, status))
		return -EIO;
	tps->status = *status;
	dev_info(tps->dev, "reinitialized inherited connection\n");
	return 0;
}

static int cd321x_reset_inherited_connection(struct tps6598x *tps, u32 *status)
{
	const struct cd321x_inherited_board *board;
	int ret;

	if (!device_property_read_bool(tps->dev, "apple,reset-inherited-usb2-connection") ||
	    !device_is_compatible(tps->dev, "apple,sn201202x") ||
	    tps->dbma_at_probe || !(*status & TPS_STATUS_PLUG_PRESENT) ||
	    !TPS_STATUS_TO_TYPEC_DATAROLE(*status))
		return 0;
	board = cd321x_inherited_board();
	if (!board)
		return 0;

	ret = __cd321x_reset_inherited_connection(tps, board, status);
	if (!ret || !board->best_effort)
		return ret;

	dev_warn(tps->dev, "inherited connection not reinitialized: %d\n", ret);
	if (!tps6598x_read_status(tps, status))
		return -EIO;
	tps->status = *status;
	return 0;
}

/* The ports tipd_port_replug() looks up by the USB role switch their data role drives. */
static LIST_HEAD(tipd_ports);
static DEFINE_MUTEX(tipd_ports_lock);

static void tipd_replug_work(struct work_struct *work);

static void tipd_port_list_add(struct tps6598x *tps)
{
	INIT_LIST_HEAD(&tps->port_node);
	INIT_WORK(&tps->replug_work, tipd_replug_work);
	if (!tps->role_sw)
		return;
	mutex_lock(&tipd_ports_lock);
	list_add_tail(&tps->port_node, &tipd_ports);
	mutex_unlock(&tipd_ports_lock);
}

static void tipd_port_list_del(struct tps6598x *tps)
{
	if (!tps->port_node.next)
		return;
	mutex_lock(&tipd_ports_lock);
	list_del_init(&tps->port_node);
	mutex_unlock(&tipd_ports_lock);
	cancel_work_sync(&tps->replug_work);
}

/*
 * An SN201202x takes "DISC" unlocked with the key its DT gives it (apple,debug-lock-key, the board
 * number reversed).
 */
static int tipd_replug_key(struct tps6598x *tps, u8 key[4])
{
	const char *s;

	if (!device_is_compatible(tps->dev, "apple,sn201202x") ||
	    device_property_read_string(tps->dev, "apple,debug-lock-key", &s) || strlen(s) != 4)
		return -EOPNOTSUPP;
	memcpy(key, s, 4);
	return 0;
}

bool tipd_replug_supported(struct tps6598x *tps)
{
	u8 key[4];

	return !tipd_replug_key(tps, key);
}

/*
 * As cd321x_reset_inherited_connection() does it: unlocked, "DISC", the app mode back, locked
 * again. The port lock is held only for the commands: meanwhile the interrupt handler sees the
 * partner go and come back, and sets the mux, the alternate modes and the data role again.
 * The commands are read with the gap the sleep's own command is (CD321X_SSPS_READ_GAP_US): read
 * back to back while the partner attaches again, the relock ended late, or in SPMI parity faults
 * and -EIO. A relock that fails is tried once more.
 */
#define TIPD_REPLUG_RELOCK_TRIES	2
static void tipd_replug_work(struct work_struct *work)
{
	struct tps6598x *tps = container_of(work, struct tps6598x, replug_work);
	static const u8 locked[4];
	u8 seconds = tps->replug_seconds, result = 0;
	unsigned long timeout;
	int ret, lock_ret, mode = -1;
	unsigned int tries;
	u8 key[4];

	mutex_lock(&tps->lock);
	ret = tipd_replug_key(tps, key);
	if (!ret)
		ret = tps6598x_exec_cmd_gap(tps, "LOCK", sizeof(key), key, sizeof(result), &result,
					    1000, 0, CD321X_SSPS_READ_GAP_US);
	if (!ret && result)
		ret = -EIO;
	if (!ret)
		ret = tps6598x_exec_cmd_gap(tps, "DISC", sizeof(seconds), &seconds, 0, NULL, 5000, 0,
					    CD321X_SSPS_READ_GAP_US);
	mutex_unlock(&tps->lock);

	if (!ret) {
		msleep(seconds * 1000 + 100);
		timeout = jiffies + msecs_to_jiffies(2000);
		do {
			mutex_lock(&tps->lock);
			mode = tps6598x_check_mode(tps);
			mutex_unlock(&tps->lock);
			if (mode == TPS_MODE_APP || mode == TPS_MODE_APP1)
				break;
			msleep(20);
		} while (time_before(jiffies, timeout));
		if (mode != TPS_MODE_APP && mode != TPS_MODE_APP1)
			ret = mode < 0 ? mode : -ETIMEDOUT;
	}

	for (tries = 1; ; tries++) {
		mutex_lock(&tps->lock);
		lock_ret = tps6598x_exec_cmd_gap(tps, "LOCK", sizeof(locked), locked, 0, NULL, 5000, 0,
						 CD321X_SSPS_READ_GAP_US);
		mutex_unlock(&tps->lock);
		if (!lock_ret || tries >= TIPD_REPLUG_RELOCK_TRIES)
			break;
		msleep(500);
	}

	if (ret || lock_ret)
		dev_warn(tps->dev, "port replug for its partner failed: %d (unlock result %u), locked again: %d (try %u)\n",
			 ret, result, lock_ret, tries);
	else
		dev_info(tps->dev, "port replugged for its partner (%u s disconnected), locked again (try %u)\n",
			 seconds, tries);
}

/* Context: tipd_ports_lock held, @tps on tipd_ports. */
static int tipd_replug_queue(struct tps6598x *tps, u8 seconds)
{
	int ret;

	lockdep_assert_held(&tipd_ports_lock);
	if (!tipd_replug_supported(tps))
		return -EOPNOTSUPP;
	if (work_pending(&tps->replug_work))
		return -EBUSY;
	tps->replug_seconds = seconds;
	ret = queue_work(system_freezable_wq, &tps->replug_work) ? 0 : -EBUSY;
	dev_info(tps->dev, "port replug for its partner requested (%u s disconnected): %d\n",
		 seconds, ret);
	return ret;
}

/**
 * tipd_port_replug - disconnect and reattach a port, as a cable pulled and plugged back does
 * @sw: the USB role switch the port drives
 * @seconds: how long the port stays disconnected
 *
 * The controller's "DISC" command: it detaches the partner for @seconds and attaches it again
 * on its own, with a new contract, its alternate modes entered and the mux and the data role set
 * again. For a partner that a system sleep left in a state only a replug ends: on the J714s a
 * DP alt mode hub whose SuperSpeed side does not come back after some wakes (dwc3-apple). A
 * display on the port goes dark meanwhile. The replug runs on the port's own work, after this
 * returns: 0 when it is queued, -EBUSY if one already is, -EOPNOTSUPP on a controller that is
 * not known to take it (an SN201202x whose DT gives apple,debug-lock-key), -ENODEV if no port
 * drives @sw.
 */
int tipd_port_replug(struct usb_role_switch *sw, u8 seconds)
{
	struct tps6598x *tps;
	int ret = -ENODEV;

	mutex_lock(&tipd_ports_lock);
	list_for_each_entry(tps, &tipd_ports, port_node) {
		if (tps->role_sw != sw)
			continue;
		ret = tipd_replug_queue(tps, seconds);
		break;
	}
	mutex_unlock(&tipd_ports_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(tipd_port_replug);

/*
 * The same replug for this port, asked for through debugfs ("replug"). -ENODEV while the port is
 * not on tipd_ports: without a role switch, and from tipd_port_list_del() on, which cancels the
 * work after taking the port off, so nothing is queued after it.
 */
int tipd_replug_request(struct tps6598x *tps, u8 seconds)
{
	guard(mutex)(&tipd_ports_lock);
	if (!tps->port_node.next || list_empty(&tps->port_node))
		return -ENODEV;
	return tipd_replug_queue(tps, seconds);
}

/*
 * SN201202x: a port controller found outside S0 at the probe (in its boot power state) leaves it as
 * macOS has it do, at the port's host start. The macOS-HV trace of XNU's boot (11-20, a J714s with
 * the Dell S2725QC on its Left Front port) has XNU find each of the machine's SN201202x in its boot
 * power state (SYS_PWR_STATE 0x07) and read its
 * status there, send it HChk (64 bytes of 0xfe) and then SSPS 0x00, have the port's xHCI running
 * 86 ms after that SSPS and its SuperSpeed port powered at 113 ms, and send USBw 0x00 right after
 * the xHCI runs. Only at 143 ms does the controller enter the partner's alternate mode (DP, pin
 * assignment D). The Dell's USB3 hub resets its links when the mode comes up and again at HPD,
 * and each time comes back about 1.2 s later onto a host that is already there, SuperSpeed first.
 *
 * Linux used to send SSPS 0x00 at the very start of the probe, 2.5 to 3 s before the port's host
 * (the LDCM command, the rest of the probe, the attach debounce, the PHY's start). The hub then
 * came back to no host: its SuperSpeed side gave up and came up only after the USB2 hub's
 * enumeration resets (96 of 108 boots), or not at all (5 of 108: lost until a replug).
 *
 * So the probe keeps the boot power state until its end, the decision taken from the state the
 * controller reports, whatever the machine. There, as XNU: HChk, then SSPS 0x00, before anything
 * that needs S0 (an inherited connection's renegotiation). The
 * PHY takes the pin assignment when it starts, so the mode the controller enters is waited for
 * (at most 1.5 s, 143 ms in the trace), and the first update runs without the attach debounce;
 * once it has set the host role, USBw 0x00.
 */
#define TIPD_BOOT_MODE_WAIT_MS		1500
#define TIPD_BOOT_MODE_POLL_MS		20
#define TIPD_BOOT_CMD1_WAIT_MS		5000
#define TIPD_BOOT_HCHK_FILL		0xfe

/* Whether the probe keeps the controller's boot power state until its end. */
static bool tipd_boot_power_kept(struct tps6598x *tps)
{
	u8 state;

	tps->boot_s0_deferred = false;
	if (!device_is_compatible(tps->dev, "apple,sn201202x") ||
	    tps6598x_read8(tps, TPS_REG_SYSTEM_POWER_STATE, &state) ||
	    state == TPS_SYSTEM_POWER_STATE_S0)
		return false;
	dev_dbg(tps->dev, "boot power state %#x kept until the end of the probe\n", state);
	tps->boot_s0_deferred = true;
	return true;
}

/* A command still running from the probe (LDCM can take seconds) ends before HChk and SSPS. */
static int tipd_boot_cmd1_idle(struct tps6598x *tps)
{
	ktime_t start = ktime_get();
	u32 val;
	int ret;

	do {
		ret = tps6598x_read32(tps, TPS_REG_CMD1, &val);
		if (ret)
			return ret;
		if (!val || INVALID_CMD(val))
			return 0;
		usleep_range(CD321X_SSPS_READ_GAP_US, CD321X_SSPS_READ_GAP_US + 1000);
	} while (ktime_ms_delta(ktime_get(), start) < TIPD_BOOT_CMD1_WAIT_MS);
	return -ETIMEDOUT;
}

/* The probe's last step before the partner is registered, if it kept the boot power state. */
static int tipd_boot_power_on(struct tps6598x *tps, u32 *status)
{
	u8 hchk[TPS_MAX_LEN], reply[TPS_MAX_LEN] = {};
	ktime_t start;
	u32 data = 0;
	bool debug_port;
	u64 mask;
	int ret;

	if (!tps->boot_s0_deferred)
		return 0;
	tps->boot_s0_deferred = false;
	/*
	 * A debug port (the boot loader's debug link: in DBMa, or described with its lock key)
	 * gets only the SSPS, as before: no HChk, and no USBw, which moves the USB2 pair that
	 * the debug link's route may use.
	 */
	debug_port = tps->dbma_at_probe ||
		     device_property_present(tps->dev, "apple,debug-lock-key");
	ret = tipd_boot_cmd1_idle(tps);
	if (ret)
		dev_dbg(tps->dev, "command still running before HChk: %d\n", ret);
	if (!debug_port) {
		memset(hchk, TIPD_BOOT_HCHK_FILL, sizeof(hchk));
		ret = tps6598x_exec_cmd_gap(tps, "HChk", sizeof(hchk), hchk, sizeof(reply), reply,
					    1000, 0, CD321X_SSPS_READ_GAP_US);
		dev_dbg(tps->dev, "HChk: %d, %16ph\n", ret, reply);
		if (ret)
			tipd_boot_cmd1_idle(tps);
	}

	/* -EBUSY: a command the probe left running past both waits, see sn201202x_init() */
	ret = tps->data->switch_power_state(tps, TPS_SYSTEM_POWER_STATE_S0);
	if (ret)
		return ret;
	start = ktime_get();
	/* XNU writes the mask in the boot power state too; it is read back once in S0. */
	if (tps6598x_read64(tps, TPS_REG_INT_MASK1, &mask) || mask != cd321x_irq_mask(tps)) {
		ret = tps6598x_write64(tps, TPS_REG_INT_MASK1, cd321x_irq_mask(tps));
		if (ret)
			return ret;
	}
	if (!tps6598x_read_status(tps, status))
		return -EIO;
	tps->status = *status;
	/*
	 * A port without USB data (MagSafe) has no mode to wait for, and a port whose inherited
	 * connection is renegotiated next (cd321x_reset_inherited_connection()) attaches again
	 * through the debounce, as before.
	 */
	if (!(*status & TPS_STATUS_PLUG_PRESENT) || !tps->role_sw ||
	    device_property_read_bool(tps->dev, "apple,reset-inherited-usb2-connection"))
		goto contaminant;

	do {
		if (!tps6598x_read32(tps, TPS_REG_DATA_STATUS, &data) &&
		    data & (TPS_DATA_STATUS_DP_CONNECTION | TPS_DATA_STATUS_TBT_CONNECTION |
			    CD321X_DATA_STATUS_USB4_CONNECTION))
			break;
		msleep(TIPD_BOOT_MODE_POLL_MS);
	} while (ktime_ms_delta(ktime_get(), start) < TIPD_BOOT_MODE_WAIT_MS);
	dev_dbg(tps->dev, "S0 at the end of the probe, data status %#x after %lld ms\n", data,
		ktime_ms_delta(ktime_get(), start));

	if (!tps6598x_read_status(tps, status))
		return -EIO;
	tps->status = *status;
	tps->boot_no_debounce = true;
	tps->boot_usbw_due = !debug_port;
contaminant:
	/* A connector state read in the boot power state is read again in S0. */
	if (tps->contaminant_ops && tps->contaminant != TYPEC_CONTAMINANT_NONE)
		tps->contaminant_ops->update(tps);
	return 0;
}

/*
 * From the port lock's holder, the update that set @role: after the first host role following the
 * boot power state, USBw 0x00 (tipd_usbw_locked(), with the sleep's SSPS read gaps).
 */
static void tipd_boot_host_started(struct tps6598x *tps, enum usb_role role)
{
	u8 mode = 0x00;	/* USBw off, as XNU sends it */
	int ret;

	lockdep_assert_held(&tps->lock);
	if (!tps->boot_usbw_due)
		return;
	tps->boot_usbw_due = false;
	if (role != USB_ROLE_HOST)
		return;
	ret = tipd_usbw_locked(tps, mode);
	/* -EINVAL is a firmware without the command ("!CMD"): nothing to do there. */
	if (ret == -EINVAL)
		dev_dbg(tps->dev, "USBw %#x not known to the firmware\n", mode);
	else if (ret)
		dev_warn(tps->dev, "USBw %#x after the boot's host start failed: %d\n", mode, ret);
}

int tipd_init(struct tps6598x *tps)
{
	struct fwnode_handle *fwnode;
	bool patch_loaded = false;
	u32 status;
	u32 vid = 0;
	int ret, cleanup_ret;
	bool reset_allowed = true;

	/* Do not retain a PHY-only mux handle before the route changeset runs. */
	if (cd321x_display_gate_pending(tps))
		return -EPROBE_DEFER;

	dev_dbg(tps->dev, "Type-C probe start\n");
	tps->reset = devm_gpiod_get_optional(tps->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(tps->reset))
		return dev_err_probe(tps->dev, PTR_ERR(tps->reset),
				     "failed to get reset GPIO\n");
	if (tps->reset)
		msleep(TPS_SETUP_MS);

	if (!device_is_compatible(tps->dev, "ti,tps25750")) {
		ret = tps6598x_read32(tps, TPS_REG_VID, &vid);
		if (ret < 0 || !vid) {
			dev_err(tps->dev, "failed to read vendor ID: %d, vid: %#x\n",
				ret, vid);
			return -ENODEV;
		}
	}

	/* An SN201202x in its boot power state leaves it at the end: tipd_boot_power_on(). */
	if (tps->data->switch_power_state && !tipd_boot_power_kept(tps)) {
		ret = tps->data->switch_power_state(tps, TPS_SYSTEM_POWER_STATE_S0);
		if (ret)
			return ret;
	}

	/* Make sure the controller has application firmware running */
	ret = tps6598x_check_mode(tps);
	if (ret < 0)
		return ret;

	if (ret == TPS_MODE_PTCH) {
		patch_loaded = true;
		ret = tps->data->init(tps);
		if (ret)
			return ret;
	}

	/*
	 * A controller found in "DBMa" is used as it is. Nothing below issues a
	 * mode-changing 4CC command (the S0 power state switch, above or at the end
	 * of the probe for an SN201202x in its boot power state, only does if the
	 * controller is not in S0 already, and HChk is skipped), so probing neither
	 * leaves the debug mode nor disturbs a Debug USB or serial route set up before
	 * Linux. The LDCM safety command below does not change controller mode.
	 * Remember it, so that the debugfs interface never tears that mode down.
	 */
	tps->dbma_at_probe = ret == TPS_MODE_DBMA;
	if (tps->dbma_at_probe)
		dev_info(tps->dev, "controller in debug mode \"DBMa\", leaving it there\n");

	ret = tps6598x_write64(tps, TPS_REG_INT_MASK1, cd321x_irq_mask(tps));
	if (ret)
		goto err_reset_controller;

	if (!tps6598x_read_status(tps, &status)) {
		ret = -ENODEV;
		goto err_clear_mask;
	}

	tps->status = status;

	/*
	 * This fwnode has a "compatible" property, but is never populated as a
	 * struct device. Instead we simply parse it to read the properties.
	 * This breaks fw_devlink=on. To maintain backward compatibility
	 * with existing DT files, we work around this by deleting any
	 * fwnode_links to/from this fwnode.
	 */
	fwnode = device_get_named_child_node(tps->dev, "connector");
	if (fwnode)
		fw_devlink_purge_absent_suppliers(fwnode);

	tps->role_sw = fwnode_usb_role_switch_get(fwnode);
	if (IS_ERR(tps->role_sw)) {
		ret = PTR_ERR(tps->role_sw);
		goto err_fwnode_put;
	}

	ret = devm_tps6598_psy_register(tps);
	if (ret)
		goto err_role_put;

	tps->mux = fwnode_typec_mux_get(fwnode);
	if (IS_ERR(tps->mux)) {
		ret = PTR_ERR(tps->mux);
		/* No provider reference means rollback cannot prove SAFE. */
		reset_allowed = false;
		goto err_role_put;
	}

	ret = tps->data->register_port(tps, fwnode);
	if (ret) {
		/* No live port remains to prove SAFE on failed registration. */
		reset_allowed = false;
		goto err_mux_put;
	}

	if (tps->contaminant_ops) {
		ret = tps->contaminant_ops->update(tps);
		if (ret) {
			if (ret != -EINPROGRESS)
				dev_warn(tps->dev,
					 "connector contaminant state unavailable: %d\n", ret);
			ret = 0;
		}
	}

	/* Before the inherited connection's renegotiation, which runs in S0. */
	ret = tipd_boot_power_on(tps, &status);
	if (ret)
		goto err_disconnect;

	ret = cd321x_reset_inherited_connection(tps, &status);
	if (ret)
		goto err_disconnect;

	if (status & TPS_STATUS_PLUG_PRESENT) {
		ret = -EINVAL;
		if (!tps->data->read_power_status(tps))
			goto err_disconnect;
		if (!tps->data->read_data_status(tps))
			goto err_disconnect;
		ret = tps->data->connect(tps, status);
		if (ret) {
			dev_err(tps->dev, "failed to register partner\n");
			goto err_disconnect;
		}
	}

	tps->wakeup = device_property_read_bool(tps->dev, "wakeup-source");
	if (tps->irq) {
		/*
		 * A variant that filters its events in system sleep keeps the
		 * interrupt live and decides itself whether to wake the system.
		 */
		ret = devm_request_threaded_irq(tps->dev, tps->irq, NULL,
						tps->data->irq_handler,
						IRQF_SHARED | IRQF_ONESHOT |
						(tps->wakeup && tps->sleep_event ?
						 IRQF_NO_SUSPEND : 0),
						dev_name(tps->dev), tps);
	} else {
		dev_dbg(tps->dev, "no IRQ specified, using polling mode\n");
		INIT_DELAYED_WORK(&tps->wq_poll, tps6598x_poll_work);
		queue_delayed_work(system_power_efficient_wq, &tps->wq_poll,
				   msecs_to_jiffies(POLL_INTERVAL));
	}

	if (ret)
		goto err_disconnect;

	fwnode_handle_put(fwnode);

	if (tps->wakeup && tps->irq) {
		devm_device_init_wakeup(tps->dev);
		if (!tps->sleep_event)
			enable_irq_wake(tps->irq);
	}

	tipd_port_list_add(tps);
	tipd_debugfs_register(tps);
	if (tps->data->ready)
		tps->data->ready(tps);

	return 0;

err_disconnect:
	cleanup_ret = tps6598x_disconnect(tps, 0);
	if (cleanup_ret) {
		dev_err(tps->dev, "probe hardware rollback failed: %d\n", cleanup_ret);
		reset_allowed = false;
	}
	tps->data->unregister_port(tps);
err_mux_put:
	typec_mux_put(tps->mux);
err_role_put:
	usb_role_switch_put(tps->role_sw);
err_fwnode_put:
	fwnode_handle_put(fwnode);
err_clear_mask:
	tps6598x_write64(tps, TPS_REG_INT_MASK1, 0);
err_reset_controller:
	/* Reset PD controller to remove any applied patch */
	if (patch_loaded && reset_allowed)
		tps->data->reset(tps);

	return ret;
}
EXPORT_SYMBOL_GPL(tipd_init);

void tipd_remove(struct tps6598x *tps)
{
	int ret = 0;

	tipd_port_list_del(tps);
	tipd_debugfs_unregister(tps);

	if (!tps->irq)
		cancel_delayed_work_sync(&tps->wq_poll);
	else
		devm_free_irq(tps->dev, tps->irq, tps);
	if (tps->contaminant_ops)
		tps->contaminant_ops->stop(tps);

	if (tps->data->remove)
		ret = tps->data->remove(tps);

	if (!ret)
		ret = tps6598x_disconnect(tps, 0);
	else
		tps6598x_drop_partner(tps, 0);
	if (ret)
		dev_err(tps->dev, "hardware teardown failed on remove: %d\n", ret);
	tps->data->unregister_port(tps);
	typec_mux_put(tps->mux);
	usb_role_switch_put(tps->role_sw);

	/* Reset PD controller to remove any applied patch */
	if (!ret)
		tps->data->reset(tps);

	if (!ret && tps->reset)
		gpiod_set_value_cansleep(tps->reset, 1);
}
EXPORT_SYMBOL_GPL(tipd_remove);

/*
 * USBw with the port lock held. CMD1 is read with the sleep's SSPS gaps (CD321X_SSPS_READ_GAP_US):
 * read back to back, a J714s controller's USBw ends late or not within its limit (1 s), where
 * macOS sees it end in 9-16 ms.
 */
int tipd_usbw_locked(struct tps6598x *tps, u8 mode)
{
	lockdep_assert_held(&tps->lock);
	return tps6598x_exec_cmd_gap(tps, "USBw", sizeof(mode), &mode, 0, NULL, 1000, 0,
				     CD321X_SSPS_READ_GAP_US);
}
EXPORT_SYMBOL_GPL(tipd_usbw_locked);

/*
 * macOS tells the port controller of a USB root port with dock-remote-wake when that port
 * sleeps, and the controller then watches the USB2 lines for a remote wakeup (static; what its
 * firmware does with it isn't known). The J700's root port has none, but measured on a J700,
 * a USB keyboard on a connector wakes it this way. The mode goes by the speed on the port: 0x09
 * for full and high speed. At the wake the controller hands the lines back before the port
 * resumes. Only a firmware-driven sleep needs this, and a failure only logs.
 */
static void tipd_usbw(struct tps6598x *tps, u8 mode)
{
	int ret;

	guard(mutex)(&tps->lock);
	ret = tps6598x_exec_cmd(tps, "USBw", sizeof(mode), &mode, 0, NULL);
	if (ret)
		dev_warn(tps->dev, "USBw %#x failed: %d\n", mode, ret);
	else
		dev_dbg(tps->dev, "USBw %#x\n", mode);
}

/* The fixed hub's port that the connector's USB2 pair goes to, from the graph, or 0. */
static u32 tipd_hub_port(struct tps6598x *tps)
{
	struct fwnode_handle *con __free(fwnode_handle) =
		device_get_named_child_node(tps->dev, "connector");
	struct fwnode_handle *ep __free(fwnode_handle) =
		fwnode_graph_get_endpoint_by_id(con, 0, 0, 0);
	struct fwnode_handle *port __free(fwnode_handle) = fwnode_graph_get_remote_port(ep);
	u32 reg = 0;

	fwnode_property_read_u32(port, "reg", &reg);
	return reg;
}

void tipd_usb_wake(struct tps6598x *tps, bool asleep)
{
	bool armed;

	if (!pm_suspend_via_firmware() ||
	    !device_property_read_bool(tps->dev, "apple,usb-remote-wake"))
		return;
	if (asleep) {
		u32 port = tipd_hub_port(tps);
		bool wake = apple_sleep_usb_port_can_wake(port);

		/*
		 * Only for a device on this connector that can wake. Armed without one, the
		 * controller reported a USB2 plug at the sleep's entry and the sleep came straight
		 * back, on an empty port and on one left with a passive adapter (measured on a
		 * J700). A device plugged in during the sleep still wakes through the SMC's plug
		 * event.
		 */
		if (wake)
			dev_dbg(tps->dev, "a device on hub port %u can wake: arming USB wake\n",
				port);
		else
			dev_dbg(tps->dev, "nothing on hub port %u can wake: USB wake not armed\n",
				port);
		scoped_guard(mutex, &tps->lock)
			tps->usbw_armed = wake;
		if (wake)
			tipd_usbw(tps, TIPD_USBW_HIGH_SPEED);
		return;
	}
	/* Out of S3 first: .complete would only come once the port has resumed. */
	scoped_guard(mutex, &tps->lock) {
		tps->data->switch_power_state(tps, TPS_SYSTEM_POWER_STATE_S0);
		armed = tps->usbw_armed;
		tps->usbw_armed = false;
	}
	if (!armed)
		return;
	tipd_usbw(tps, TIPD_USBW_HAND_BACK);
	tipd_usbw(tps, TIPD_USBW_OFF);
}
EXPORT_SYMBOL_GPL(tipd_usb_wake);

int tipd_suspend(struct tps6598x *tps)
{
	if (tps->wakeup) {
		if (!tps->sleep_event) {
			disable_irq(tps->irq);
			enable_irq_wake(tps->irq);
		}
	} else if (tps->reset) {
		gpiod_set_value_cansleep(tps->reset, 1);
	}

	if (!tps->irq)
		cancel_delayed_work_sync(&tps->wq_poll);

	return 0;
}
EXPORT_SYMBOL_GPL(tipd_suspend);

int tipd_resume(struct tps6598x *tps)
{
	int ret;

	tps->h17_display = of_machine_is_compatible("apple,t8140") ||
			   of_machine_is_compatible("apple,t8142");

	ret = tps6598x_check_mode(tps);
	if (ret < 0)
		return ret;

	if (ret == TPS_MODE_PTCH) {
		ret = tps->data->init(tps);
		if (ret)
			return ret;
	}

	if (tps->wakeup) {
		if (!tps->sleep_event) {
			disable_irq_wake(tps->irq);
			enable_irq(tps->irq);
		}
	} else if (tps->reset) {
		gpiod_set_value_cansleep(tps->reset, 0);
		msleep(TPS_SETUP_MS);
	}

	if (!tps->irq)
		queue_delayed_work(system_power_efficient_wq, &tps->wq_poll,
				   msecs_to_jiffies(POLL_INTERVAL));

	return 0;
}
EXPORT_SYMBOL_GPL(tipd_resume);

/* The poll of a controller without an interrupt, stopped for the sleep's entry (J714s) */
void tipd_poll_stop(struct tps6598x *tps)
{
	if (!tps->irq)
		cancel_delayed_work_sync(&tps->wq_poll);
}
EXPORT_SYMBOL_GPL(tipd_poll_stop);

/* ... and started again: a poll already queued, by tipd_resume() for one, stays as it is */
void tipd_poll_start(struct tps6598x *tps)
{
	if (!tps->irq)
		queue_delayed_work(system_power_efficient_wq, &tps->wq_poll,
				   msecs_to_jiffies(POLL_INTERVAL));
}
EXPORT_SYMBOL_GPL(tipd_poll_start);

/* Freezable cable work has drained before device PM calls prepare. */
int tipd_prepare(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);
	int ret = 0;

	if (tps->data->prepare)
		ret = tps->data->prepare(tps);
	/* A device whose prepare fails gets no .complete: what was prepared is undone here. */
	if (ret && tps->data->complete)
		tps->data->complete(tps);

	return ret;
}
EXPORT_SYMBOL_GPL(tipd_prepare);

/* Completion also runs after an aborted suspend; work starts after thaw. */
void tipd_complete(struct device *dev)
{
	struct tps6598x *tps = dev_get_drvdata(dev);

	if (tps->data->complete)
		tps->data->complete(tps);
}
EXPORT_SYMBOL_GPL(tipd_complete);

static const struct tipd_data tipd_cd321x_data = {
	.irq_handler = cd321x_interrupt,
	.irq_mask1 = APPLE_CD_REG_INT_POWER_STATUS_UPDATE |
		     APPLE_CD_REG_INT_DATA_STATUS_UPDATE |
		     APPLE_CD_REG_INT_PLUG_EVENT,
	.tps_struct_size = sizeof(struct cd321x),
	.remove = cd321x_remove,
	.register_port = cd321x_register_port,
	.unregister_port = cd321x_unregister_port,
	.ready = cd321x_ready,
	.trace_data_status = trace_cd321x_data_status,
	.trace_power_status = trace_tps6598x_power_status,
	.trace_status = trace_tps6598x_status,
	.init = cd321x_init,
	.read_data_status = cd321x_read_data_status,
	.read_power_status = tps6598x_read_power_status,
	.reset = cd321x_reset,
	.switch_power_state = cd321x_switch_power_state,
	.connect = cd321x_connect,
	.prepare = cd321x_prepare,
	.complete = cd321x_complete,
};

static const struct tipd_data tipd_tps6598x_data = {
	.irq_handler = tps6598x_interrupt,
	.irq_mask1 = TPS_REG_INT_POWER_STATUS_UPDATE |
		     TPS_REG_INT_DATA_STATUS_UPDATE |
		     TPS_REG_INT_PLUG_EVENT,
	.tps_struct_size = sizeof(struct tps6598x),
	.register_port = tps6598x_register_port,
	.unregister_port = tps6598x_unregister_port,
	.trace_data_status = trace_tps6598x_data_status,
	.trace_power_status = trace_tps6598x_power_status,
	.trace_status = trace_tps6598x_status,
	.apply_patch = tps6598x_apply_patch,
	.init = tps6598x_init,
	.read_data_status = tps6598x_read_data_status,
	.read_power_status = tps6598x_read_power_status,
	.reset = tps6598x_reset,
	.connect = tps6598x_connect,
};

static const struct tipd_data tipd_tps66993_data = {
	.irq_handler = tps6598x_interrupt,
	.irq_mask1 = TPS_REG_INT_DATA_STATUS_UPDATE |
		     TPS_REG_INT_PLUG_EVENT,
	.tps_struct_size = sizeof(struct tps6598x),
	.register_port = tps6598x_register_port,
	.unregister_port = tps6598x_unregister_port,
	.trace_data_status = trace_tps6598x_data_status,
	.trace_power_status = trace_tps6598x_power_status,
	.trace_status = trace_tps6598x_status,
	.apply_patch = tps6598x_apply_patch,
	.init = tps6598x_init,
	.read_data_status = tps6598x_read_data_status,
	.read_power_status = tps66993_read_power_status,
	.reset = tps6598x_reset,
	.connect = tps6598x_connect,
};

static const struct tipd_data tipd_tps25750_data = {
	.irq_handler = tps25750_interrupt,
	.irq_mask1 = TPS_REG_INT_POWER_STATUS_UPDATE |
		     TPS_REG_INT_DATA_STATUS_UPDATE |
		     TPS_REG_INT_PLUG_EVENT,
	.tps_struct_size = sizeof(struct tps6598x),
	.register_port = tps25750_register_port,
	.unregister_port = tps6598x_unregister_port,
	.trace_data_status = trace_tps6598x_data_status,
	.trace_power_status = trace_tps25750_power_status,
	.trace_status = trace_tps25750_status,
	.apply_patch = tps25750_apply_patch,
	.init = tps25750_init,
	.read_data_status = tps6598x_read_data_status,
	.read_power_status = tps6598x_read_power_status,
	.reset = tps25750_reset,
	.connect = tps6598x_connect,
};

static const struct tipd_data tipd_sn201202x_data = {
	.irq_handler = cd321x_interrupt,
	.irq_mask1 = APPLE_CD_REG_INT_POWER_STATUS_UPDATE |
		     APPLE_CD_REG_INT_DATA_STATUS_UPDATE |
		     APPLE_CD_REG_INT_PLUG_EVENT |
		     SN201202X_INT_DP_SID_UPDATE |
		     SN201202X_INT_LDCM_UPDATE,
	.tps_struct_size = sizeof(struct sn201202x),
	.remove = cd321x_remove,
	.register_port = cd321x_register_port,
	.unregister_port = cd321x_unregister_port,
	.ready = cd321x_ready,
	.trace_data_status = trace_cd321x_data_status,
	.trace_power_status = trace_tps6598x_power_status,
	.trace_status = trace_tps6598x_status,
	.init = cd321x_init,
	.read_data_status = cd321x_read_data_status,
	.read_power_status = tps6598x_read_power_status,
	.reset = cd321x_reset,
	.switch_power_state = cd321x_switch_power_state,
	.connect = cd321x_connect,
	.prepare = cd321x_prepare,
	.complete = cd321x_complete,
};

const struct tipd_data *tipd_hw_variants[] = {
	[TIPD_VARIANT_CD321X] = &tipd_cd321x_data,
	[TIPD_VARIANT_TPS6598X] = &tipd_tps6598x_data,
	[TIPD_VARIANT_TPS25750] = &tipd_tps25750_data,
	[TIPD_VARIANT_SN201202X] = &tipd_sn201202x_data,
	[TIPD_VARIANT_TPS66993] = &tipd_tps66993_data,
};
EXPORT_SYMBOL_GPL(tipd_hw_variants);

static void __exit tipd_core_exit(void)
{
	tipd_debugfs_exit();
}
module_exit(tipd_core_exit);

MODULE_AUTHOR("Heikki Krogerus <heikki.krogerus@linux.intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI TPS6598x USB Power Delivery Controller Core Functions");
