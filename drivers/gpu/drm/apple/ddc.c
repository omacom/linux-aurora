// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * DDC/CI for displays on an external DCP (dcpext).
 *
 * The connectors of an external DCP have no I2C bus the host can reach: DDC
 * goes through the DCP firmware, on the same dcpav-service-epic service that
 * copies the EDID. macOS reaches it through DCPAVServiceProxy::readI2C() and
 * ::writeI2C() (IOAVServiceReadI2C/WriteI2C, used by m1ddc and BetterDisplay).
 * Their messages are EPIC standard service calls (group 1, or 0 when no data
 * follows; command 9 = read, 10 = write) with a 64-byte argument block:
 *
 *   read:  +0x00 chip  +0x10 offset  +0x20 length  +0x30 status (reply),
 *          the data read follows at +0x40
 *   write: +0x00 status (reply)  +0x10 chip  +0x20 offset  +0x30 length,
 *          the data to write follows at +0x40
 *
 * (DCPAVFamilyProxy in the macOS 26 kernel cache; copyEDID is command 7 there
 * as well, the call the driver already makes.) "offset" is the first byte of
 * the I2C write: for DDC/CI the host's source address 0x51.
 *
 * This registers one I2C adapter per external DCP and maps I2C transfers to
 * these calls, so ddcutil and libddcutil (and the desktops built on it) work
 * unchanged:
 *   0x37 (DDC/CI)  write [0x51, payload...] -> writeI2C(chip, 0x51, payload)
 *                  read  n bytes           -> readI2C(chip, 0x51, n)
 *                  chip is 0x37, or 0xB7 behind the MCDP29xx DP->HDMI
 *                  converter (AppleDCPMCDP29XX), as m1ddc does.
 *   0x50 / 0x30    EDID / E-DDC segment, answered from the connector's EDID,
 *                  so that tools find the display on the bus.
 * appledrm.ddc=0 (also at runtime, /sys/module/appledrm/parameters/ddc) turns
 * the firmware calls off.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/slab.h>

#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <drm/drm_property.h>

#include "afk.h"
#include "connector.h"
#include "dcp-internal.h"
#include "ddc.h"

static bool ddc_enable = true;
module_param_named(ddc, ddc_enable, bool, 0644);
MODULE_PARM_DESC(ddc, "DDC/CI over the DCP firmware for external displays (default: on)");

#define DDC_CI_ADDR		0x37
#define DDC_CI_ADDR_MCDP29XX	0xb7
#define DDC_HOST_ADDR		0x51
#define DDC_EDID_ADDR		0x50
#define DDC_SEGMENT_ADDR	0x30
#define DDC_MAX_LEN		128
/* m1ddc: the MCDP29xx returns empty replies when read sooner after a write */
#define DDC_MCDP_READ_WAIT_MS	50

#define DCPAV_GROUP(len)	((len) ? 1 : 0)
#define DCPAV_CMD_READ_I2C	9
#define DCPAV_CMD_WRITE_I2C	10

struct dcpav_i2c_read {
	__le32 chip;
	u8 _pad0[12];
	__le32 offset;
	u8 _pad1[12];
	__le32 len;
	u8 _pad2[12];
	__le32 status;
	u8 _pad3[12];
	u8 data[];
} __packed;

struct dcpav_i2c_write {
	__le32 status;
	u8 _pad0[12];
	__le32 chip;
	u8 _pad1[12];
	__le32 offset;
	u8 _pad2[12];
	__le32 len;
	u8 _pad3[12];
	u8 data[];
} __packed;

static_assert(sizeof(struct dcpav_i2c_read) == 0x40);
static_assert(sizeof(struct dcpav_i2c_write) == 0x40);

struct dcp_ddc {
	struct i2c_adapter adap;
	struct apple_dcp *dcp;
	unsigned long last_write;	/* jiffies of the last DDC/CI write */
	u8 edid_segment;
	u8 edid_offset;
};

static struct apple_epic_service *ddc_service(struct apple_dcp *dcp, bool *mcdp)
{
	struct apple_epic_service *service;

	mutex_lock(&dcp->dcpavserv.lock);
	service = dcp->dcpavserv.enabled ? afk_service_get(dcp->dcpavserv.service) : NULL;
	*mcdp = dcp->dcpavserv.mcdp29xx;
	mutex_unlock(&dcp->dcpavserv.lock);
	return service;
}

static int ddc_fw_read(struct dcp_ddc *ddc, u8 *buf, u16 len)
{
	struct apple_dcp *dcp = ddc->dcp;
	struct apple_epic_service *service;
	struct dcpav_i2c_read cmd = {}, *resp;
	bool mcdp;
	int ret;

	service = ddc_service(dcp, &mcdp);
	if (!service)
		return -ENODEV;
	if (mcdp) {
		unsigned long ready = ddc->last_write +
				      msecs_to_jiffies(DDC_MCDP_READ_WAIT_MS);

		if (time_before(jiffies, ready))
			msleep(jiffies_to_msecs(ready - jiffies));
	}
	resp = kzalloc(sizeof(*resp) + len, GFP_KERNEL);
	if (!resp) {
		afk_service_put(service);
		return -ENOMEM;
	}
	cmd.chip = cpu_to_le32(mcdp ? DDC_CI_ADDR_MCDP29XX : DDC_CI_ADDR);
	cmd.offset = cpu_to_le32(DDC_HOST_ADDR);
	cmd.len = cpu_to_le32(len);
	ret = afk_service_call_len(service, DCPAV_GROUP(len), DCPAV_CMD_READ_I2C,
				   &cmd, sizeof(cmd), len, resp, sizeof(*resp) + len, 0,
				   NULL);
	afk_service_put(service);
	if (!ret && le32_to_cpu(resp->status)) {
		dev_dbg(dcp->dev, "DDC: read of %u bytes: status 0x%08x\n", len,
			le32_to_cpu(resp->status));
		ret = -EIO;
	}
	if (!ret)
		memcpy(buf, resp->data, len);
	kfree(resp);
	return ret;
}

static int ddc_fw_write(struct dcp_ddc *ddc, u8 offset, const u8 *data, u16 len)
{
	struct apple_dcp *dcp = ddc->dcp;
	struct apple_epic_service *service;
	struct dcpav_i2c_write *cmd, resp = {};
	bool mcdp;
	int ret;

	service = ddc_service(dcp, &mcdp);
	if (!service)
		return -ENODEV;
	cmd = kzalloc(sizeof(*cmd) + len, GFP_KERNEL);
	if (!cmd) {
		afk_service_put(service);
		return -ENOMEM;
	}
	cmd->chip = cpu_to_le32(mcdp ? DDC_CI_ADDR_MCDP29XX : DDC_CI_ADDR);
	cmd->offset = cpu_to_le32(offset);
	cmd->len = cpu_to_le32(len);
	memcpy(cmd->data, data, len);
	ret = afk_service_call_len(service, DCPAV_GROUP(len), DCPAV_CMD_WRITE_I2C,
				   cmd, sizeof(*cmd) + len, 0, &resp, sizeof(resp), 0,
				   NULL);
	afk_service_put(service);
	kfree(cmd);
	ddc->last_write = jiffies;
	if (!ret && le32_to_cpu(resp.status)) {
		dev_dbg(dcp->dev, "DDC: write of %u bytes: status 0x%08x\n", len,
			le32_to_cpu(resp.status));
		ret = -EIO;
	}
	return ret;
}

/* EDID reads at 0x50, from the EDID the connector already has. */
static int ddc_edid_read(struct dcp_ddc *ddc, u8 *buf, u16 len)
{
	struct apple_connector *con = READ_ONCE(ddc->dcp->connector);
	struct drm_property_blob *blob = NULL;
	struct drm_device *drm;
	size_t pos;
	int ret = 0;

	if (!con)
		return -ENXIO;
	drm = con->base.dev;
	mutex_lock(&drm->mode_config.mutex);
	if (con->base.edid_blob_ptr)
		blob = drm_property_blob_get(con->base.edid_blob_ptr);
	mutex_unlock(&drm->mode_config.mutex);
	if (!blob)
		return -ENXIO;
	pos = ddc->edid_segment * 256 + ddc->edid_offset;
	if (pos + len > blob->length)
		ret = -ENXIO;
	else
		memcpy(buf, (u8 *)blob->data + pos, len);
	drm_property_blob_put(blob);
	ddc->edid_offset += len;
	return ret;
}

static bool ddc_display_present(struct apple_dcp *dcp)
{
	struct apple_connector *con = READ_ONCE(dcp->connector);

	return con && READ_ONCE(con->connected);
}

static int dcp_ddc_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct dcp_ddc *ddc = i2c_get_adapdata(adap);
	int i, ret = 0;

	if (!ddc_display_present(ddc->dcp))
		return -ENXIO;
	for (i = 0; i < num && !ret; i++) {
		struct i2c_msg *m = &msgs[i];
		bool rd = m->flags & I2C_M_RD;

		if (m->len > DDC_MAX_LEN + 1)
			return -EOPNOTSUPP;
		switch (m->addr) {
		case DDC_SEGMENT_ADDR:
			if (rd || m->len != 1)
				return -EOPNOTSUPP;
			ddc->edid_segment = m->buf[0];
			break;
		case DDC_EDID_ADDR:
			if (!rd) {
				ddc->edid_offset = m->len ? m->buf[0] : 0;
				if (m->len > 1)
					return -EOPNOTSUPP;
			} else {
				ret = ddc_edid_read(ddc, m->buf, m->len);
				ddc->edid_segment = 0;
			}
			break;
		case DDC_CI_ADDR:
			if (!READ_ONCE(ddc_enable))
				return -ENXIO;
			if (rd)
				ret = m->len ? ddc_fw_read(ddc, m->buf, m->len) : 0;
			else if (m->len)
				ret = ddc_fw_write(ddc, m->buf[0], m->buf + 1, m->len - 1);
			break;
		default:
			return -ENXIO;
		}
	}
	return ret ? ret : num;
}

static u32 dcp_ddc_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm dcp_ddc_algo = {
	.xfer = dcp_ddc_xfer,
	.functionality = dcp_ddc_func,
};

void dcp_ddc_register(struct apple_dcp *dcp)
{
	struct dcp_ddc *ddc;
	int ret;

	if (!dcp || !of_device_is_compatible(dcp->dev->of_node, "apple,dcpext") ||
	    dcp->external)
		return;
	ddc = kzalloc_obj(*ddc);
	if (!ddc)
		return;
	ddc->dcp = dcp;
	ddc->adap.owner = THIS_MODULE;
	ddc->adap.algo = &dcp_ddc_algo;
	ddc->adap.dev.parent = dcp->dev;
	ddc->adap.retries = 0;
	snprintf(ddc->adap.name, sizeof(ddc->adap.name), "apple-dcp DDC %s",
		 dev_name(dcp->dev));
	i2c_set_adapdata(&ddc->adap, ddc);
	ret = i2c_add_adapter(&ddc->adap);
	if (ret) {
		dev_warn(dcp->dev, "DDC: adapter not registered: %d\n", ret);
		kfree(ddc);
		return;
	}
	dcp->ddc = ddc;
	dev_info(dcp->dev, "DDC: %s\n", dev_name(&ddc->adap.dev));
}

void dcp_ddc_unregister(struct apple_dcp *dcp)
{
	struct dcp_ddc *ddc = dcp ? dcp->ddc : NULL;

	if (!ddc)
		return;
	dcp->ddc = NULL;
	i2c_del_adapter(&ddc->adap);
	kfree(ddc);
}
