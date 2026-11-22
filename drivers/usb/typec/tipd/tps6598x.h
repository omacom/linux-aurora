/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Driver for TI TPS6598x USB Power Delivery controller family
 *
 * Copyright (C) 2017, Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_mux.h>

#include "cd321x-pm.h"

#ifndef __TPS6598X_H__
#define __TPS6598X_H__

/* Also read by the SPMI glue to tell when the controller has woken up */
#define TPS_REG_MODE			0x03

#define TPS_FIELD_GET(_mask, _reg) ((typeof(_mask))(((_reg) & (_mask)) >> __bf_shf(_mask)))

/* TPS_REG_STATUS bits */
#define TPS_STATUS_PLUG_PRESENT		BIT(0)
#define TPS_STATUS_PLUG_UPSIDE_DOWN	BIT(4)
#define TPS_STATUS_TO_UPSIDE_DOWN(s)	(!!((s) & TPS_STATUS_PLUG_UPSIDE_DOWN))
#define TPS_STATUS_PORTROLE		BIT(5)
#define TPS_STATUS_TO_TYPEC_PORTROLE(s) (!!((s) & TPS_STATUS_PORTROLE))
#define TPS_STATUS_DATAROLE		BIT(6)
#define TPS_STATUS_TO_TYPEC_DATAROLE(s)	(!!((s) & TPS_STATUS_DATAROLE))
#define TPS_STATUS_VCONN		BIT(7)
#define TPS_STATUS_TO_TYPEC_VCONN(s)	(!!((s) & TPS_STATUS_VCONN))
#define TPS_STATUS_OVERCURRENT		BIT(16)
#define TPS_STATUS_GOTO_MIN_ACTIVE	BIT(26)
#define TPS_STATUS_BIST			BIT(27)
#define TPS_STATUS_HIGH_VOLTAGE_WARNING	BIT(28)
#define TPS_STATUS_HIGH_LOW_VOLTAGE_WARNING BIT(29)

#define TPS_STATUS_CONN_STATE_MASK		GENMASK(3, 1)
#define TPS_STATUS_CONN_STATE(x)		TPS_FIELD_GET(TPS_STATUS_CONN_STATE_MASK, (x))
#define TPS_STATUS_PP_5V0_SWITCH_MASK		GENMASK(9, 8)
#define TPS_STATUS_PP_5V0_SWITCH(x)		TPS_FIELD_GET(TPS_STATUS_PP_5V0_SWITCH_MASK, (x))
#define TPS_STATUS_PP_HV_SWITCH_MASK		GENMASK(11, 10)
#define TPS_STATUS_PP_HV_SWITCH(x)		TPS_FIELD_GET(TPS_STATUS_PP_HV_SWITCH_MASK, (x))
#define TPS_STATUS_PP_EXT_SWITCH_MASK		GENMASK(13, 12)
#define TPS_STATUS_PP_EXT_SWITCH(x)		TPS_FIELD_GET(TPS_STATUS_PP_EXT_SWITCH_MASK, (x))
#define TPS_STATUS_PP_CABLE_SWITCH_MASK		GENMASK(15, 14)
#define TPS_STATUS_PP_CABLE_SWITCH(x)		TPS_FIELD_GET(TPS_STATUS_PP_CABLE_SWITCH_MASK, (x))
#define TPS_STATUS_POWER_SOURCE_MASK		GENMASK(19, 18)
#define TPS_STATUS_POWER_SOURCE(x)		TPS_FIELD_GET(TPS_STATUS_POWER_SOURCE_MASK, (x))
#define TPS_STATUS_VBUS_STATUS_MASK		GENMASK(21, 20)
#define TPS_STATUS_VBUS_STATUS(x)		TPS_FIELD_GET(TPS_STATUS_VBUS_STATUS_MASK, (x))
#define TPS_STATUS_USB_HOST_PRESENT_MASK	GENMASK(23, 22)
#define TPS_STATUS_USB_HOST_PRESENT(x)		TPS_FIELD_GET(TPS_STATUS_USB_HOST_PRESENT_MASK, (x))
#define TPS_STATUS_LEGACY_MASK			GENMASK(25, 24)
#define TPS_STATUS_LEGACY(x)			TPS_FIELD_GET(TPS_STATUS_LEGACY_MASK, (x))

#define TPS_STATUS_CONN_STATE_NO_CONN		0
#define TPS_STATUS_CONN_STATE_DISABLED		1
#define TPS_STATUS_CONN_STATE_AUDIO_CONN	2
#define TPS_STATUS_CONN_STATE_DEBUG_CONN	3
#define TPS_STATUS_CONN_STATE_NO_CONN_R_A	4
#define TPS_STATUS_CONN_STATE_RESERVED		5
#define TPS_STATUS_CONN_STATE_CONN_NO_R_A	6
#define TPS_STATUS_CONN_STATE_CONN_WITH_R_A	7

#define TPS_STATUS_PP_SWITCH_STATE_DISABLED	0
#define TPS_STATUS_PP_SWITCH_STATE_FAULT	1
#define TPS_STATUS_PP_SWITCH_STATE_OUT		2
#define TPS_STATUS_PP_SWITCH_STATE_IN		3

#define TPS_STATUS_POWER_SOURCE_UNKNOWN		0
#define TPS_STATUS_POWER_SOURCE_VIN_3P3		1
#define TPS_STATUS_POWER_SOURCE_DEAD_BAT	2
#define TPS_STATUS_POWER_SOURCE_VBUS		3

#define TPS_STATUS_VBUS_STATUS_VSAFE0V		0
#define TPS_STATUS_VBUS_STATUS_VSAFE5V		1
#define TPS_STATUS_VBUS_STATUS_PD		2
#define TPS_STATUS_VBUS_STATUS_FAULT		3

#define TPS_STATUS_USB_HOST_PRESENT_NO		0
#define TPS_STATUS_USB_HOST_PRESENT_PD_NO_USB	1
#define TPS_STATUS_USB_HOST_PRESENT_NO_PD	2
#define TPS_STATUS_USB_HOST_PRESENT_PD_USB	3

#define TPS_STATUS_LEGACY_NO			0
#define TPS_STATUS_LEGACY_SINK			1
#define TPS_STATUS_LEGACY_SOURCE		2

/* TPS_REG_INT_* bits */
#define TPS_REG_INT_USER_VID_ALT_MODE_OTHER_VDM		BIT_ULL(27+32)
#define TPS_REG_INT_USER_VID_ALT_MODE_ATTN_VDM		BIT_ULL(26+32)
#define TPS_REG_INT_USER_VID_ALT_MODE_EXIT		BIT_ULL(25+32)
#define TPS_REG_INT_USER_VID_ALT_MODE_ENTERED		BIT_ULL(24+32)
#define TPS_REG_INT_EXIT_MODES_COMPLETE			BIT_ULL(20+32)
#define TPS_REG_INT_DISCOVER_MODES_COMPLETE		BIT_ULL(19+32)
#define TPS_REG_INT_VDM_MSG_SENT			BIT_ULL(18+32)
#define TPS_REG_INT_VDM_ENTERED_MODE			BIT_ULL(17+32)
#define TPS_REG_INT_ERROR_UNABLE_TO_SOURCE		BIT_ULL(14+32)
#define TPS_REG_INT_SRC_TRANSITION			BIT_ULL(10+32)
#define TPS_REG_INT_ERROR_DISCHARGE_FAILED		BIT_ULL(9+32)
#define TPS_REG_INT_ERROR_MESSAGE_DATA			BIT_ULL(7+32)
#define TPS_REG_INT_ERROR_PROTOCOL_ERROR		BIT_ULL(6+32)
#define TPS_REG_INT_ERROR_MISSING_GET_CAP_MESSAGE	BIT_ULL(4+32)
#define TPS_REG_INT_ERROR_POWER_EVENT_OCCURRED		BIT_ULL(3+32)
#define TPS_REG_INT_ERROR_CAN_PROVIDE_PWR_LATER		BIT_ULL(2+32)
#define TPS_REG_INT_ERROR_CANNOT_PROVIDE_PWR		BIT_ULL(1+32)
#define TPS_REG_INT_ERROR_DEVICE_INCOMPATIBLE		BIT_ULL(0+32)
#define TPS_REG_INT_CMD2_COMPLETE			BIT(31)
#define TPS_REG_INT_CMD1_COMPLETE			BIT(30)
#define TPS_REG_INT_ADC_HIGH_THRESHOLD			BIT(29)
#define TPS_REG_INT_ADC_LOW_THRESHOLD			BIT(28)
#define TPS_REG_INT_PD_STATUS_UPDATE			BIT(27)
#define TPS_REG_INT_STATUS_UPDATE			BIT(26)
#define TPS_REG_INT_DATA_STATUS_UPDATE			BIT(25)
#define TPS_REG_INT_POWER_STATUS_UPDATE			BIT(24)
#define TPS_REG_INT_PP_SWITCH_CHANGED			BIT(23)
#define TPS_REG_INT_HIGH_VOLTAGE_WARNING		BIT(22)
#define TPS_REG_INT_USB_HOST_PRESENT_NO_LONGER		BIT(21)
#define TPS_REG_INT_USB_HOST_PRESENT			BIT(20)
#define TPS_REG_INT_GOTO_MIN_RECEIVED			BIT(19)
#define TPS_REG_INT_PR_SWAP_REQUESTED			BIT(17)
#define TPS_REG_INT_SINK_CAP_MESSAGE_READY		BIT(15)
#define TPS_REG_INT_SOURCE_CAP_MESSAGE_READY		BIT(14)
#define TPS_REG_INT_NEW_CONTRACT_AS_PROVIDER		BIT(13)
#define TPS_REG_INT_NEW_CONTRACT_AS_CONSUMER		BIT(12)
#define TPS_REG_INT_VDM_RECEIVED			BIT(11)
#define TPS_REG_INT_ATTENTION_RECEIVED			BIT(10)
#define TPS_REG_INT_OVERCURRENT				BIT(9)
#define TPS_REG_INT_BIST				BIT(8)
#define TPS_REG_INT_RDO_RECEIVED_FROM_SINK		BIT(7)
#define TPS_REG_INT_DR_SWAP_COMPLETE			BIT(5)
#define TPS_REG_INT_PR_SWAP_COMPLETE			BIT(4)
#define TPS_REG_INT_PLUG_EVENT				BIT(3)
#define TPS_REG_INT_HARD_RESET				BIT(1)
#define TPS_REG_INT_PD_SOFT_RESET			BIT(0)

/* Apple-specific TPS_REG_INT_* bits */
#define APPLE_CD_REG_INT_DATA_STATUS_UPDATE		BIT(10)
#define APPLE_CD_REG_INT_POWER_STATUS_UPDATE		BIT(9)
#define APPLE_CD_REG_INT_STATUS_UPDATE			BIT(8)
#define APPLE_CD_REG_INT_PLUG_EVENT			BIT(1)

/*
 * SN201202x: the DP SID status register changed, that is the DisplayPort
 * status the partner sent last (DP Status Update or Attention).
 */
#define SN201202X_INT_DP_SID_UPDATE			BIT_ULL(37)
/* The controller has a new liquid-detection status snapshot. */
#define SN201202X_INT_LDCM_UPDATE			BIT_ULL(61)

#define SN201202X_LDCM_STATUS_LEN	55

/* TPS_REG_SYSTEM_POWER_STATE states */
#define TPS_SYSTEM_POWER_STATE_S0	0x00
#define TPS_SYSTEM_POWER_STATE_S3	0x03
#define TPS_SYSTEM_POWER_STATE_S4	0x04
#define TPS_SYSTEM_POWER_STATE_S5	0x05

/* TPS_REG_POWER_STATUS bits (masks shared by TPS_FIELD_GET accessors and FIELD_PREP) */
#define TPS_POWER_STATUS_CONNECTION_MASK	BIT(0)
#define TPS_POWER_STATUS_SOURCESINK_MASK	BIT(1)
#define TPS_POWER_STATUS_CONNECTION(x) \
	TPS_FIELD_GET(TPS_POWER_STATUS_CONNECTION_MASK, (x))
#define TPS_POWER_STATUS_SOURCESINK(x) \
	TPS_FIELD_GET(TPS_POWER_STATUS_SOURCESINK_MASK, (x))
#define TPS_POWER_STATUS_BC12_DET(x)	TPS_FIELD_GET(BIT(2), (x))

#define TPS_POWER_STATUS_TYPEC_CURRENT_MASK GENMASK(3, 2)
#define TPS_POWER_STATUS_PWROPMODE(p)	    TPS_FIELD_GET(TPS_POWER_STATUS_TYPEC_CURRENT_MASK, (p))
#define TPS_POWER_STATUS_BC12_STATUS_MASK   GENMASK(6, 5)
#define TPS_POWER_STATUS_BC12_STATUS(p)	    TPS_FIELD_GET(TPS_POWER_STATUS_BC12_STATUS_MASK, (p))

#define TPS_POWER_STATUS_TYPEC_CURRENT_USB     0
#define TPS_POWER_STATUS_TYPEC_CURRENT_1A5     1
#define TPS_POWER_STATUS_TYPEC_CURRENT_3A0     2
#define TPS_POWER_STATUS_TYPEC_CURRENT_PD      3

#define TPS_POWER_STATUS_BC12_STATUS_SDP 0
#define TPS_POWER_STATUS_BC12_STATUS_CDP 2
#define TPS_POWER_STATUS_BC12_STATUS_DCP 3

/* TPS25750_REG_POWER_STATUS bits */
#define TPS25750_POWER_STATUS_CHARGER_DETECT_STATUS_MASK	GENMASK(7, 4)
#define TPS25750_POWER_STATUS_CHARGER_DETECT_STATUS(p) \
	TPS_FIELD_GET(TPS25750_POWER_STATUS_CHARGER_DETECT_STATUS_MASK, (p))
#define TPS25750_POWER_STATUS_CHARGER_ADVERTISE_STATUS_MASK	GENMASK(9, 8)
#define TPS25750_POWER_STATUS_CHARGER_ADVERTISE_STATUS(p) \
	TPS_FIELD_GET(TPS25750_POWER_STATUS_CHARGER_ADVERTISE_STATUS_MASK, (p))

#define TPS25750_POWER_STATUS_CHARGER_DET_STATUS_DISABLED	0
#define TPS25750_POWER_STATUS_CHARGER_DET_STATUS_IN_PROGRESS	1
#define TPS25750_POWER_STATUS_CHARGER_DET_STATUS_NONE		2
#define TPS25750_POWER_STATUS_CHARGER_DET_STATUS_SPD		3
#define TPS25750_POWER_STATUS_CHARGER_DET_STATUS_BC_1_2_CPD	4
#define TPS25750_POWER_STATUS_CHARGER_DET_STATUS_BC_1_2_DPD	5
#define TPS25750_POWER_STATUS_CHARGER_DET_STATUS_DIV_1_DCP	6
#define TPS25750_POWER_STATUS_CHARGER_DET_STATUS_DIV_2_DCP	7
#define TPS25750_POWER_STATUS_CHARGER_DET_STATUS_DIV_3_DCP	8
#define TPS25750_POWER_STATUS_CHARGER_DET_STATUS_1_2V_DCP	9

/* TPS_REG_DATA_STATUS bits */
#define TPS_DATA_STATUS_DATA_CONNECTION	     BIT(0)
#define TPS_DATA_STATUS_UPSIDE_DOWN	     BIT(1)
#define TPS_DATA_STATUS_ACTIVE_CABLE	     BIT(2)
#define TPS_DATA_STATUS_USB2_CONNECTION	     BIT(4)
#define TPS_DATA_STATUS_USB3_CONNECTION	     BIT(5)
#define TPS_DATA_STATUS_USB3_GEN2	     BIT(6)
#define TPS_DATA_STATUS_USB_DATA_ROLE	     BIT(7)
#define TPS_DATA_STATUS_DP_CONNECTION	     BIT(8)
#define TPS_DATA_STATUS_DP_SINK		     BIT(9)
#define TPS_DATA_STATUS_TBT_CONNECTION	     BIT(16)
#define TPS_DATA_STATUS_TBT_TYPE	     BIT(17)
#define TPS_DATA_STATUS_OPTICAL_CABLE	     BIT(18)
#define TPS_DATA_STATUS_ACTIVE_LINK_TRAIN    BIT(20)
#define TPS_DATA_STATUS_FORCE_LSX	     BIT(23)
#define TPS_DATA_STATUS_POWER_MISMATCH	     BIT(24)

/* modified TPS_REG_DATA_STATUS bits for CD321x (and likely also TPS65987DDK) */
#define CD321X_DATA_STATUS_HPD_IRQ	     BIT(14)
#define CD321X_DATA_STATUS_HPD_LEVEL	     BIT(15)
#define CD321X_DATA_STATUS_ACTIVE_CABLE	     BIT(22)
#define CD321X_DATA_STATUS_USB4_CONNECTION   BIT(23)

#define TPS_DATA_STATUS_DP_PIN_ASSIGNMENT_MASK GENMASK(11, 10)
#define TPS_DATA_STATUS_DP_PIN_ASSIGNMENT(x) \
	TPS_FIELD_GET(TPS_DATA_STATUS_DP_PIN_ASSIGNMENT_MASK, (x))
#define TPS_DATA_STATUS_TBT_CABLE_SPEED_MASK   GENMASK(27, 25)
#define TPS_DATA_STATUS_TBT_CABLE_SPEED(x) \
	TPS_FIELD_GET(TPS_DATA_STATUS_TBT_CABLE_SPEED_MASK, (x))
#define TPS_DATA_STATUS_TBT_CABLE_GEN_MASK     GENMASK(29, 28)
#define TPS_DATA_STATUS_TBT_CABLE_GEN(x) \
	TPS_FIELD_GET(TPS_DATA_STATUS_TBT_CABLE_GEN_MASK, (x))

/* Map data status to DP spec assignments */
#define TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT(x) \
	((TPS_DATA_STATUS_DP_PIN_ASSIGNMENT(x) << 1) | \
		TPS_FIELD_GET(TPS_DATA_STATUS_USB3_CONNECTION, (x)))
#define TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_E    0
#define TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_F    BIT(0)
#define TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_C    BIT(1)
#define TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_D    (BIT(1) | BIT(0))
#define TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_A    BIT(2)
#define TPS_DATA_STATUS_DP_SPEC_PIN_ASSIGNMENT_B    (BIT(2) | BIT(1))

/* BOOT STATUS REG*/
#define TPS_BOOT_STATUS_DEAD_BATTERY_FLAG	BIT(2)
#define TPS_BOOT_STATUS_I2C_EEPROM_PRESENT	BIT(3)

/* PD STATUS REG */
#define TPS_REG_PD_STATUS_PORT_TYPE_MASK	GENMASK(5, 4)
#define TPS_PD_STATUS_PORT_TYPE(x) \
	TPS_FIELD_GET(TPS_REG_PD_STATUS_PORT_TYPE_MASK, x)

#define TPS_PD_STATUS_PORT_TYPE_SINK_SOURCE	0
#define TPS_PD_STATUS_PORT_TYPE_SINK		1
#define TPS_PD_STATUS_PORT_TYPE_SOURCE		2
#define TPS_PD_STATUS_PORT_TYPE_SOURCE_SINK	3

/* SLEEP CONF REG */
#define TPS_SLEEP_CONF_SLEEP_MODE_ALLOWED	BIT(0)

/* Start Patch Download Sequence */
#define TPS_PTCS_CONTENT_APP			BIT(0)
#define TPS_PTCS_CONTENT_DEV			BIT(1)
#define TPS_PTCS_OUT_BYTES			4
#define TPS_PTCS_STATUS				1

#define TPS_PTCS_STATUS_FAIL			0x80
/* Patch Download */
#define TPS_PTCD_OUT_BYTES			10
#define TPS_PTCD_TRANSFER_STATUS		1
#define TPS_PTCD_LOADING_STATE			2

#define TPS_PTCD_LOAD_ERR			0x09
/* Patch Download Complete */
#define TPS_PTCC_OUT_BYTES			4
#define TPS_PTCC_DEV				2
#define TPS_PTCC_APP				3

/* Version Register */
#define TPS_VERSION_HW_VERSION_MASK            GENMASK(31, 24)
#define TPS_VERSION_HW_VERSION(x)              TPS_FIELD_GET(TPS_VERSION_HW_VERSION_MASK, (x))
#define TPS_VERSION_HW_65981_2_6               0x00
#define TPS_VERSION_HW_65987_8_DH              0xF7
#define TPS_VERSION_HW_65987_8_DK              0xF9

/* Int Event Register length */
#define TPS_65981_2_6_INTEVENT_LEN             8
#define TPS_65987_8_INTEVENT_LEN               11

/*
 * Max data bytes for Data1, Data2, and other registers. See ch 1.3.2:
 * https://www.ti.com/lit/ug/slvuan1a/slvuan1a.pdf
 */
#define TPS_MAX_LEN	64

/* Standard Task return codes */
#define TPS_TASK_TIMEOUT		1
#define TPS_TASK_REJECTED		3

enum tipd_hw_variant {
	TIPD_VARIANT_INVALID = 0,
	TIPD_VARIANT_CD321X,
	TIPD_VARIANT_TPS6598X,
	TIPD_VARIANT_TPS25750,
	TIPD_VARIANT_SN201202X,
	TIPD_VARIANT_TPS66993,
	TIPD_VARIANT_MAX,
};

struct tps6598x;

struct tipd_contaminant_ops {
	int (*update)(struct tps6598x *tps);
	/* update(), retried in the background on failure; tps->lock held */
	void (*refresh)(struct tps6598x *tps);
	void (*event)(struct tps6598x *tps);
	void (*invalidate)(struct tps6598x *tps);
	void (*stop)(struct tps6598x *tps);
};

struct sn201202x_ldcm_ops {
	int (*read)(void *ctx, enum typec_contaminant *contaminant);
	int (*set_mitigation)(void *ctx, bool enable);
	void (*publish)(void *ctx, enum typec_contaminant contaminant);
};

struct tipd_data {
	irq_handler_t irq_handler;
	u64 irq_mask1;
	size_t tps_struct_size;
	int (*remove)(struct tps6598x *tps);
	int (*register_port)(struct tps6598x *tps, struct fwnode_handle *node);
	void (*unregister_port)(struct tps6598x *tps);
	void (*ready)(struct tps6598x *tps);
	void (*trace_data_status)(u32 status);
	void (*trace_power_status)(u16 status);
	void (*trace_status)(u32 status);
	int (*apply_patch)(struct tps6598x *tps);
	int (*init)(struct tps6598x *tps);
	int (*switch_power_state)(struct tps6598x *tps, u8 target_state);
	bool (*read_data_status)(struct tps6598x *tps);
	bool (*read_power_status)(struct tps6598x *tps);
	int (*reset)(struct tps6598x *tps);
	int (*connect)(struct tps6598x *tps, u32 status);
	int (*prepare)(struct tps6598x *tps);
	void (*complete)(struct tps6598x *tps);
};

struct tps6598x {
	/* Native H17 routes retain ENOSPC admission and their own HPD path. */
	u8 h17_display:1;
	/* On the list tipd_port_replug() looks a port up on, by its role switch; its replug. */
	struct list_head port_node;
	struct work_struct replug_work;
	u8 replug_seconds;
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock; /* device lock */
	int irq;
	u8 i2c_protocol:1;
	u8 dbma_at_probe:1; /* Apple "DBMa" debug mode was active at probe */

	struct gpio_desc *reset;
	struct typec_port *port;
	struct typec_partner *partner;
	struct usb_pd_identity partner_identity;
	struct usb_role_switch *role_sw;
	struct typec_capability typec_cap;
	struct typec_mux *mux;
	/*
	 * Set by ->remove() while its SAFE transaction is still open, so that
	 * tps6598x_disconnect() ends it after shutting down the USB role.
	 */
	bool remove_safe_prepared;

	struct power_supply *psy;
	struct power_supply_desc psy_desc;
	enum power_supply_usb_type usb_type;

	int wakeup;
	/* Handles events while the system sleeps, if the variant can filter them */
	bool (*sleep_event)(struct tps6598x *tps);
	/* Wakes the controller, where its bus has a command for that */
	int (*wake)(struct tps6598x *tps);
	u32 status; /* status reg */
	u32 data_status;
	u16 pwr_status;
	struct delayed_work	wq_poll;

	const struct tipd_data *data;
	const struct tipd_contaminant_ops *contaminant_ops;
	enum typec_contaminant contaminant;

	struct tipd_debugfs *debugfs;
	struct typec_altmode *partner_altmode_dp;
	/* USB remote wake armed on this port for the sleep (USBw). */
	bool usbw_armed;
	/* An SN201202x's boot power state is left at the end of the probe: tipd_boot_power_on() */
	bool boot_s0_deferred;
	bool boot_no_debounce;
	bool boot_usbw_due;
};

/* TPS_REG_USB4_STATUS */
struct tps6598x_usb4_status_reg {
	u8 mode_status;
	__le32 eudo;
	__le32 unknown;
} __packed;

/* TPS_REG_DP_SID_STATUS */
struct tps6598x_dp_sid_status_reg {
	u8 mode_status;
	__le32 status_tx;
	__le32 status_rx;
	__le32 configure;
	__le32 mode_data;
} __packed;

/* TPS_REG_INTEL_VID_STATUS */
struct tps6598x_intel_vid_status_reg {
	u8 mode_status;
	__le32 attention_vdo;
	__le16 enter_vdo;
	__le16 device_mode;
} __packed;

struct cd321x_status {
	u32 status;
	u32 pwr_status;
	u32 data_status;
	u32 status_changed;
	u32 data_status_changed;
	struct usb_pd_identity partner_identity;
	struct tps6598x_dp_sid_status_reg dp_sid_status;
	struct tps6598x_intel_vid_status_reg intel_vid_status;
	struct tps6598x_usb4_status_reg usb4_status;
	/* SN201202x: what the DP status showed since the last update */
	bool dp_sid_event;
	bool dp_sid_hpd_low;
	bool dp_sid_irq;
};

struct cd321x {
	struct tps6598x tps;

	struct tps6598x_dp_sid_status_reg dp_sid_status;
	struct tps6598x_intel_vid_status_reg intel_vid_status;
	struct tps6598x_usb4_status_reg usb4_status;

	struct typec_altmode *port_altmode_dp;
	struct typec_altmode *port_altmode_tbt;

	struct typec_mux *mux;
	struct typec_mux_state state;
	bool state_valid;
	u32 dp_status;
	u32 dp_conf;
	/* SN201202x: the partner's DP status is current for this DP connection */
	bool dp_sid_valid;
	struct typec_thunderbolt_switch *tbt_switch;
	struct notifier_block tbt_notifier;
	bool tbt_notifier_registered;
	/* Router that had not bound at probe: look it up when a switch registers */
	struct fwnode_handle *tbt_fwnode;
	struct notifier_block tbt_watch;
	struct work_struct tbt_attach_work;
	bool tbt_watch_registered;
	/* Closes the USB4/Thunderbolt session before device_shutdown() */
	struct notifier_block reboot_nb;
	bool reboot_nb_registered;
	atomic_t link_event;
	atomic_t ready_event;
	atomic_t restore_event;
	bool display_route_active;
	/* The sleep's debug link cut reset this controller: its setup is expected. */
	bool reset_by_cut;
	/* The debug link is still to be rearmed after a cut, and the tries so far */
	bool rearm_pending;
	unsigned int rearm_tries;
	/* Resets taken since the cut for a contract the partner dropped, see cd321x_rearm_work() */
	unsigned int rearm_resets;
	bool dp_sink_reported;
	bool dp_irq_hpd;
	/* A plug was in when the sleep began, and a wake's first port state is still to be applied */
	bool sleep_plug;
	bool wake_apply;
	/* J714s: macOS's USBw 0x00 after an attach is due at the next update */
	bool macos_usbw_due;
	/* ... and the controller is in macOS's no-partner state 0x02 since a detach */
	bool macos_detached;

	struct cd321x_status update_status;
	struct delayed_work update_work;
	struct delayed_work resume_work;
	struct delayed_work rearm_work;
	struct cd321x_pm_state pm;
	struct usb_pd_identity cur_partner_identity;
	bool cur_partner_is_pd;

	struct fwnode_handle *connector_fwnode;
};

struct sn201202x {
	struct cd321x cd;
	bool polled;
	bool select_polled;
	/* Sleeps while idle: its event interrupt stays live while it sleeps */
	bool idle_sleep;
	struct completion select_completion;
	struct completion sleep_completion;
	struct completion wake_completion;
	struct spmi_device *sdev;
	/* Links that wake the system, while the controller's events are filtered */
	u32 sleep_links;
	bool sleep_watch;
	/* Busy-wait: in suspend-to-idle a sleeping wait outlasts the timers */
	bool atomic;
	/* Failed tries to sleep while idle since it last slept, gave up or was accessed */
	unsigned int sleep_tries;
	struct delayed_work ldcm_work;
	bool ldcm_mitigated;
	/* ldcm_mitigated matches the controller (unknown after a reset) */
	bool ldcm_mitigation_known;
	/* A mitigation was requested since the controller's last reset */
	bool ldcm_requested;
	/* J714s: this sleep is entered and left as macOS does, see sn201202x_prepare() */
	bool macos_sleep;
	/* ... and the controller has been sent SLEEP at the prepare and not woken since */
	bool entry_asleep;
	/* ... and its suspend has run since: nothing sends SLEEP again before the wake */
	bool entry_suspended;
	/* Register accesses that reached it meanwhile */
	unsigned int entry_strays;
	/* J714s: macOS's USBw 0x09 after the PHYs' stop, handed back at the wake */
	struct notifier_block usbw_nb;
	/* ... for a controller with a connector (not MagSafe's) */
	bool usbw_port;
	/* ... and sent in this sleep, not yet handed back */
	bool usbw_macos;
};

extern const struct tipd_data *tipd_hw_variants[];
extern const struct regmap_config tps6598x_regmap_config;

int tipd_init(struct tps6598x *tps);
void tipd_remove(struct tps6598x *tps);
bool tipd_replug_supported(struct tps6598x *tps);
int tipd_replug_request(struct tps6598x *tps, u8 seconds);
int tipd_prepare(struct device *dev);
void tipd_complete(struct device *dev);
int tipd_suspend(struct tps6598x *tps);
int tipd_resume(struct tps6598x *tps);
void tipd_poll_stop(struct tps6598x *tps);
void tipd_poll_start(struct tps6598x *tps);
int tipd_usbw_locked(struct tps6598x *tps, u8 mode);
/* USBw modes: the USB2 lines watched for a full/high-speed remote wakeup, handed back, off */
#define TIPD_USBW_HIGH_SPEED	0x09
#define TIPD_USBW_HAND_BACK	0x80
#define TIPD_USBW_OFF		0x00

/* Internal to tps6598x-core */
int tps6598x_block_read(struct tps6598x *tps, u8 reg, void *val, size_t len);
/* What the system-sleep event filter of a port reads */
struct cd321x_sleep_state {
	u32 data_status;
	u32 status;
	/* A power source this port sinks from is capable of USB communication */
	bool usb_comm;
	bool contaminant_event;
};

int cd321x_sleep_events(struct tps6598x *tps, struct cd321x_sleep_state *st);
/* INT_EVENT1 and INT_CLEAR1 as macOS reads and clears them at the wake: 13 bytes */
#define CD321X_WAKE_EVENT_LEN	13
int cd321x_wake_events(struct tps6598x *tps, u8 event[CD321X_WAKE_EVENT_LEN], u32 *status,
		       u32 *data_status);
void tipd_usb_wake(struct tps6598x *tps, bool asleep);
int tps6598x_exec_cmd_tmo(struct tps6598x *tps, const char *cmd,
			  size_t in_len, const u8 *in_data,
			  size_t out_len, u8 *out_data,
			  u32 cmd_timeout_ms, u32 res_delay_ms);

struct tipd_debugfs {
	struct dentry *dentry;
	/*
	 * Open "command" files that entered "DBMa"; written under tps->lock,
	 * read without it by tipd_debugfs_dbma_held()
	 */
	unsigned int dbma_holders;
};

#ifdef CONFIG_DEBUG_FS
bool tipd_debugfs_dbma_held(struct tps6598x *tps);
void tipd_debugfs_register(struct tps6598x *tps);
void tipd_debugfs_unregister(struct tps6598x *tps);
void tipd_debugfs_exit(void);
#else
static inline bool tipd_debugfs_dbma_held(struct tps6598x *tps) { return false; }
static inline void tipd_debugfs_register(struct tps6598x *tps) { }
static inline void tipd_debugfs_unregister(struct tps6598x *tps) { }
static inline void tipd_debugfs_exit(void) { }
#endif

#if IS_ENABLED(CONFIG_KUNIT)
void cd321x_mux_failed(struct cd321x *cd321x, const char *what, int ret, bool held);
void cd321x_mux_gave_up(struct cd321x *cd321x, unsigned int retries, unsigned int held_ms);
void cd321x_init_work(struct cd321x *cd321x);
int cd321x_init_mux(struct cd321x *cd321x);
void cd321x_stop_work(struct cd321x *cd321x);
int cd321x_reboot_close(struct cd321x *cd321x);
int cd321x_reboot_notify(struct notifier_block *nb, unsigned long action,
			 void *data);
bool cd321x_tbt_switch_keep_waiting(struct device *dev);
struct typec_thunderbolt_switch *
cd321x_tbt_switch_get(struct device *dev, struct fwnode_handle *fwnode,
		      bool *late);
int cd321x_tbt_watch_late(struct cd321x *cd321x, struct fwnode_handle *fwnode);
int sn201202x_suspend(struct device *dev);
int sn201202x_resume(struct device *dev);
int sn201202x_runtime_suspend(struct device *dev);
int sn201202x_runtime_resume(struct device *dev);
void sn201202x_idle_sleep_start(struct tps6598x *tps);
void sn201202x_idle_sleep_stop(struct tps6598x *tps);
extern const struct regmap_bus regmap_sn201202x;
extern const struct tipd_contaminant_ops sn201202x_contaminant_ops;
u32 sn201202x_sleep_links(const struct cd321x_sleep_state *st);
enum typec_contaminant
sn201202x_ldcm_decode(const u8 status[SN201202X_LDCM_STATUS_LEN]);
int sn201202x_ldcm_update_state(void *ctx,
				const struct sn201202x_ldcm_ops *ops,
				enum typec_contaminant last);
#endif

#endif /* __TPS6598X_H__ */
