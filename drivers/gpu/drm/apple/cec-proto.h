/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Wire protocol for the compact EPIC CEC service ("dcpav-cec-interface-epic",
 * unit 0) on RTKit endpoint 0x2b (AV_AUDIO_ENDPOINT_DESKTOP).
 *
 * The following layouts apply to endpoint 0x2b on T6041 (J616c, firmware
 * 27.2). Offsets are relative to the start of the message body.
 *
 * Group 0 calls:
 *   4  Open (0x20 B): u32 status at +0x00, u64 opened at +0x10.
 *   5  Close (0x10 B): u32 status at +0x00.
 *   6  Set logical addresses (0x20 B): u16 mask at +0x10.
 *   7  Read EDID attributes (0x50 B): __le16 physical address at +0x00.
 *   9  Transmit (0x40 B): up to 16 frame bytes at +0x00, length minus one
 *      at +0x10, retry count (0..5) at +0x20, u32 reply status at +0x30.
 *
 * Group 2 reports carry a struct epic_service_call header:
 *   1  Terminate (0x20 B).
 *   2  Receive (0x20 B): up to 16 frame bytes at +0x00, length minus one
 *      in the low nibble at +0x10.
 *   3  HPD (0x10 B): u8 connected at +0x00.
 *   4  EDID attributes (0x50 B): __le16 physical address at +0x00.
 *
 * The driver only decodes this layout on endpoint 0x2b. The report layout
 * on endpoint 0x29 (14.x firmware) still needs a hardware capture.
 */
#ifndef __APPLE_DCP_CEC_PROTO_H__
#define __APPLE_DCP_CEC_PROTO_H__

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <uapi/linux/cec.h>

#include "afk.h"

#define DCP_CEC_GROUP_CALL			0
#define DCP_CEC_GROUP_REPORT			2

/* Group 0 commands and body sizes */
#define DCP_CEC_CMD_OPEN			4
#define DCP_CEC_OPEN_LEN			0x20

#define DCP_CEC_CMD_CLOSE			5
#define DCP_CEC_CLOSE_LEN			0x10

#define DCP_CEC_CMD_SET_LOGICAL_ADDR_MASK	6
#define DCP_CEC_SET_LA_MASK_LEN			0x20

#define DCP_CEC_CMD_GET_EDID_ATTRS		7
#define DCP_CEC_EDID_ATTRS_LEN			0x50

#define DCP_CEC_CMD_SEND_CEC			9
#define DCP_CEC_SEND_LEN			0x40

/* Group 2 async reports and body sizes */
#define DCP_CEC_RPT_TERMINATE			1
#define DCP_CEC_RPT_TERMINATE_LEN		0x20

#define DCP_CEC_RPT_RX_FRAME			2
#define DCP_CEC_RPT_RX_FRAME_LEN		0x20

#define DCP_CEC_RPT_HPD				3
#define DCP_CEC_RPT_HPD_LEN			0x10

#define DCP_CEC_RPT_EDID_ATTRS			4
#define DCP_CEC_RPT_EDID_ATTRS_LEN		0x50

/*
 * Timeouts (ms): synchronous AFK transmit call (7500 ms), driver completion
 * guard window (8500 ms), and CEC core transfer watchdog (9500 ms). Open runs
 * outside tx_work, so tx_work spends at most DCP_CEC_TX_TIMEOUT_MS in AFK.
 */
#define DCP_CEC_TX_TIMEOUT_MS			7500
#define DCP_CEC_TX_COMPLETE_GUARD_MS		8500
#define DCP_CEC_ADAP_XFER_TIMEOUT_MS		9500

/* Transmit reply status at body offset +0x30 */
#define DCP_CEC_RET_OK				0x00000000U
#define DCP_CEC_RET_NACK			0xe0118000U
#define DCP_CEC_RET_LINE_ERR			0xe0118001U
#define DCP_CEC_RET_TIMEOUT			0xe00002d6U
#define DCP_CEC_RET_OFFLINE			0xe00002d7U
#define DCP_CEC_RET_ABORTED			0xe00002ebU

struct dcp_cec_parsed_report {
	u32 cmd;
	bool hpd;
	u16 phys_addr;
	struct cec_msg rx_msg;
};

static inline int dcp_cec_decode_open_resp(const u8 resp[DCP_CEC_OPEN_LEN])
{
	u32 status = get_unaligned_le32(resp + 0x00);
	u64 opened = get_unaligned_le64(resp + 0x10);

	return (status == 0 && opened != 0) ? 0 : -EIO;
}

static inline int dcp_cec_decode_status_resp(const u8 *resp)
{
	return get_unaligned_le32(resp + 0x00) == 0 ? 0 : -EIO;
}

/*
 * Whenever CEC is enabled, OR in BIT(CEC_LOG_ADDR_BROADCAST) (0x8000) so
 * the adapter receives frames addressed to all logical devices.
 */
static inline u16 dcp_cec_encode_la_mask(bool enabled, u16 la_mask)
{
	if (!enabled)
		return 0x0000;

	return (la_mask & GENMASK(14, 0)) | BIT(CEC_LOG_ADDR_BROADCAST);
}

static inline void dcp_cec_build_la_mask_req(u8 req[DCP_CEC_SET_LA_MASK_LEN],
					     bool enabled, u16 la_mask)
{
	memset(req, 0, DCP_CEC_SET_LA_MASK_LEN);
	put_unaligned_le16(dcp_cec_encode_la_mask(enabled, la_mask),
			   req + 0x10);
}

static inline u16 dcp_cec_decode_edid_attrs(const u8 resp[DCP_CEC_EDID_ATTRS_LEN])
{
	return get_unaligned_le16(resp + 0x00);
}

static inline int dcp_cec_build_send_req(u8 req[DCP_CEC_SEND_LEN],
					 const struct cec_msg *msg,
					 u8 attempts)
{
	if (!msg || msg->len < 1 || msg->len > CEC_MAX_MSG_SIZE)
		return -EINVAL;

	memset(req, 0, DCP_CEC_SEND_LEN);
	memcpy(req + 0x00, msg->msg, msg->len);
	req[0x10] = (u8)((msg->len - 1) & 0x0f);
	req[0x20] = min_t(u8, 5, attempts ? attempts - 1 : 0);
	return 0;
}

static inline u8 dcp_cec_tx_status(int afk_ret, u32 retcode)
{
	if (afk_ret < 0)
		return CEC_TX_STATUS_ERROR | CEC_TX_STATUS_MAX_RETRIES;

	switch (retcode) {
	case DCP_CEC_RET_OK:
		return CEC_TX_STATUS_OK;
	case DCP_CEC_RET_NACK:
		return CEC_TX_STATUS_NACK | CEC_TX_STATUS_MAX_RETRIES;
	case DCP_CEC_RET_LINE_ERR:
		return CEC_TX_STATUS_ERROR | CEC_TX_STATUS_MAX_RETRIES;
	case DCP_CEC_RET_TIMEOUT:
		return CEC_TX_STATUS_TIMEOUT | CEC_TX_STATUS_MAX_RETRIES;
	case DCP_CEC_RET_OFFLINE:
	case DCP_CEC_RET_ABORTED:
		return CEC_TX_STATUS_ABORTED;
	default:
		return CEC_TX_STATUS_ERROR | CEC_TX_STATUS_MAX_RETRIES;
	}
}

static inline u8 dcp_cec_decode_send_resp(int afk_ret,
					  const u8 resp[DCP_CEC_SEND_LEN])
{
	if (afk_ret < 0)
		return dcp_cec_tx_status(afk_ret, 0);

	return dcp_cec_tx_status(0, get_unaligned_le32(resp + 0x30));
}

static inline int dcp_cec_parse_report(const void *data, size_t data_size,
				       struct dcp_cec_parsed_report *out)
{
	const struct epic_service_call *call = data;
	const u8 *body;
	size_t body_avail;
	u32 cmd, data_len;
	u8 rx_len;

	if (!data || !out || data_size < sizeof(*call))
		return -EMSGSIZE;

	if (le32_to_cpu(call->magic) != EPIC_SERVICE_CALL_MAGIC ||
	    le16_to_cpu(call->group) != DCP_CEC_GROUP_REPORT)
		return -EINVAL;

	cmd = le32_to_cpu(call->command);
	data_len = le32_to_cpu(call->data_len);
	body = (const u8 *)(call + 1);
	body_avail = data_size - sizeof(*call);
	if (data_len > body_avail)
		return -EMSGSIZE;

	memset(out, 0, sizeof(*out));
	out->cmd = cmd;
	out->phys_addr = CEC_PHYS_ADDR_INVALID;

	switch (cmd) {
	case DCP_CEC_RPT_TERMINATE:
		if (body_avail < DCP_CEC_RPT_TERMINATE_LEN ||
		    (data_len && data_len < DCP_CEC_RPT_TERMINATE_LEN))
			return -EMSGSIZE;
		return 0;
	case DCP_CEC_RPT_RX_FRAME:
		if (body_avail < DCP_CEC_RPT_RX_FRAME_LEN ||
		    (data_len && data_len < DCP_CEC_RPT_RX_FRAME_LEN))
			return -EMSGSIZE;
		rx_len = (body[0x10] & 0x0f) + 1;
		out->rx_msg.len = rx_len;
		memcpy(out->rx_msg.msg, body + 0x00, rx_len);
		return 0;
	case DCP_CEC_RPT_HPD:
		if (body_avail < DCP_CEC_RPT_HPD_LEN ||
		    (data_len && data_len < DCP_CEC_RPT_HPD_LEN))
			return -EMSGSIZE;
		out->hpd = body[0x00] != 0;
		return 0;
	case DCP_CEC_RPT_EDID_ATTRS:
		if (body_avail < DCP_CEC_RPT_EDID_ATTRS_LEN ||
		    (data_len && data_len < DCP_CEC_RPT_EDID_ATTRS_LEN))
			return -EMSGSIZE;
		out->phys_addr = get_unaligned_le16(body + 0x00);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

#endif /* __APPLE_DCP_CEC_PROTO_H__ */
