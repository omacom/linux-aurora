// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Standalone KUnit tests for Apple Silicon DCP HDMI-CEC wire framing,
 * TX status mapping, report parsing, and 2 ms IRQ_HPD pulse classification.
 */

#include <kunit/test.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "cec-proto.h"
#include "dcp-hdmi.h"

static void build_epic_report(u8 *buf, u16 group, u32 cmd, u32 body_len,
			      u32 magic)
{
	struct epic_service_call *call = (struct epic_service_call *)buf;

	memset(buf, 0, sizeof(*call) + body_len);
	call->group = cpu_to_le16(group);
	call->command = cpu_to_le32(cmd);
	call->data_len = cpu_to_le32(body_len);
	call->magic = cpu_to_le32(magic);
}

static void cec_open_close_decode_test(struct kunit *test)
{
	u8 open_resp[DCP_CEC_OPEN_LEN] = {};
	u8 close_resp[DCP_CEC_CLOSE_LEN] = {};

	/* OPEN requires status == 0 at +0x00 and opened != 0 at +0x10. */
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_open_resp(open_resp), -EIO);

	put_unaligned_le64(1, open_resp + 0x10);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_open_resp(open_resp), 0);

	put_unaligned_le32(0xe00002bcU, open_resp + 0x00);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_open_resp(open_resp), -EIO);

	/* CLOSE checks status == 0 at +0x00. */
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_status_resp(close_resp), 0);
	put_unaligned_le32(1, close_resp + 0x00);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_status_resp(close_resp), -EIO);
}

static void cec_la_mask_framing_test(struct kunit *test)
{
	u8 req[DCP_CEC_SET_LA_MASK_LEN];
	unsigned int i;

	/* Enabled with no unicast LA claimed still sets broadcast (0x8000). */
	dcp_cec_build_la_mask_req(req, true, 0);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(req + 0x10), 0x8000);
	for (i = 0; i < sizeof(req); i++) {
		if (i == 0x10 || i == 0x11)
			continue;
		KUNIT_EXPECT_EQ(test, req[i], 0);
	}

	/* Enabled with Playback 1 (LA 4) and Playback 2 (LA 8) -> 0x8110. */
	dcp_cec_build_la_mask_req(req, true, BIT(4) | BIT(8));
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(req + 0x10), 0x8110);

	/* Disabled clears mask to 0x0000 even if LAs were passed. */
	dcp_cec_build_la_mask_req(req, false, BIT(4) | BIT(8));
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(req + 0x10), 0x0000);
}

static void cec_edid_decode_test(struct kunit *test)
{
	u8 edid_resp[DCP_CEC_EDID_ATTRS_LEN] = {};

	put_unaligned_le16(0x2100, edid_resp + 0x00);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_edid_attrs(edid_resp), 0x2100);

	put_unaligned_le16(CEC_PHYS_ADDR_INVALID, edid_resp + 0x00);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_edid_attrs(edid_resp),
			CEC_PHYS_ADDR_INVALID);
}

static void cec_send_framing_test(struct kunit *test)
{
	u8 req[DCP_CEC_SEND_LEN];
	struct cec_msg msg = {};
	unsigned int len, i;

	/* Invalid lengths are rejected. */
	msg.len = 0;
	KUNIT_EXPECT_EQ(test, dcp_cec_build_send_req(req, &msg, 2), -EINVAL);
	msg.len = 17;
	KUNIT_EXPECT_EQ(test, dcp_cec_build_send_req(req, &msg, 2), -EINVAL);

	/* Lengths 1..16 encode (len - 1) & 0x0f at +0x10. */
	for (len = 1; len <= CEC_MAX_MSG_SIZE; len++) {
		memset(&msg, 0, sizeof(msg));
		msg.len = len;
		for (i = 0; i < len; i++)
			msg.msg[i] = 0x40 + i;

		KUNIT_ASSERT_EQ(test, dcp_cec_build_send_req(req, &msg, 2), 0);
		KUNIT_EXPECT_MEMEQ(test, req + 0x00, msg.msg, len);
		for (i = len; i < CEC_MAX_MSG_SIZE; i++)
			KUNIT_EXPECT_EQ(test, req[i], 0);
		KUNIT_EXPECT_EQ(test, req[0x10], len - 1);
		KUNIT_EXPECT_EQ(test, req[0x20], 1);
	}

	/* Retry count encoding at +0x20: min(5, attempts ? attempts - 1 : 0). */
	msg.len = 2;
	msg.msg[0] = 0x40;
	msg.msg[1] = 0x83;
	KUNIT_ASSERT_EQ(test, dcp_cec_build_send_req(req, &msg, 0), 0);
	KUNIT_EXPECT_EQ(test, req[0x20], 0);
	KUNIT_ASSERT_EQ(test, dcp_cec_build_send_req(req, &msg, 1), 0);
	KUNIT_EXPECT_EQ(test, req[0x20], 0);
	KUNIT_ASSERT_EQ(test, dcp_cec_build_send_req(req, &msg, 4), 0);
	KUNIT_EXPECT_EQ(test, req[0x20], 3);
	KUNIT_ASSERT_EQ(test, dcp_cec_build_send_req(req, &msg, 6), 0);
	KUNIT_EXPECT_EQ(test, req[0x20], 5);
	KUNIT_ASSERT_EQ(test, dcp_cec_build_send_req(req, &msg, 12), 0);
	KUNIT_EXPECT_EQ(test, req[0x20], 5);
}

static void cec_tx_status_mapping_test(struct kunit *test)
{
	u8 resp[DCP_CEC_SEND_LEN] = {};

	put_unaligned_le32(DCP_CEC_RET_OK, resp + 0x30);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_send_resp(0, resp),
			CEC_TX_STATUS_OK);

	put_unaligned_le32(DCP_CEC_RET_NACK, resp + 0x30);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_send_resp(0, resp),
			CEC_TX_STATUS_NACK | CEC_TX_STATUS_MAX_RETRIES);

	put_unaligned_le32(DCP_CEC_RET_LINE_ERR, resp + 0x30);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_send_resp(0, resp),
			CEC_TX_STATUS_ERROR | CEC_TX_STATUS_MAX_RETRIES);

	put_unaligned_le32(DCP_CEC_RET_TIMEOUT, resp + 0x30);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_send_resp(0, resp),
			CEC_TX_STATUS_TIMEOUT | CEC_TX_STATUS_MAX_RETRIES);

	put_unaligned_le32(DCP_CEC_RET_OFFLINE, resp + 0x30);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_send_resp(0, resp),
			CEC_TX_STATUS_ABORTED);

	put_unaligned_le32(DCP_CEC_RET_ABORTED, resp + 0x30);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_send_resp(0, resp),
			CEC_TX_STATUS_ABORTED);

	put_unaligned_le32(0xdeadbeefU, resp + 0x30);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_send_resp(0, resp),
			CEC_TX_STATUS_ERROR | CEC_TX_STATUS_MAX_RETRIES);

	/* Negative AFK return maps to ERROR | MAX_RETRIES regardless of body. */
	put_unaligned_le32(DCP_CEC_RET_OK, resp + 0x30);
	KUNIT_EXPECT_EQ(test, dcp_cec_decode_send_resp(-ETIMEDOUT, resp),
			CEC_TX_STATUS_ERROR | CEC_TX_STATUS_MAX_RETRIES);
}

static void cec_report_parsing_test(struct kunit *test)
{
	const size_t hdr = sizeof(struct epic_service_call);
	u8 buf[sizeof(struct epic_service_call) + DCP_CEC_RPT_EDID_ATTRS_LEN];
	struct dcp_cec_parsed_report rpt;
	unsigned int i;

	/* Terminate (cmd 1, 0x20 B). */
	build_epic_report(buf, DCP_CEC_GROUP_REPORT, DCP_CEC_RPT_TERMINATE,
			  DCP_CEC_RPT_TERMINATE_LEN, EPIC_SERVICE_CALL_MAGIC);
	KUNIT_ASSERT_EQ(test,
			dcp_cec_parse_report(buf,
					     hdr + DCP_CEC_RPT_TERMINATE_LEN,
					     &rpt),
			0);
	KUNIT_EXPECT_EQ(test, rpt.cmd, DCP_CEC_RPT_TERMINATE);

	/* RX_FRAME (cmd 2, 0x20 B): 1-byte polling frame and 16-byte frame. */
	build_epic_report(buf, DCP_CEC_GROUP_REPORT, DCP_CEC_RPT_RX_FRAME,
			  DCP_CEC_RPT_RX_FRAME_LEN, EPIC_SERVICE_CALL_MAGIC);
	buf[hdr + 0x00] = 0x04;
	buf[hdr + 0x10] = 0xf0; /* upper nibble masked off */
	KUNIT_ASSERT_EQ(test,
			dcp_cec_parse_report(buf,
					     hdr + DCP_CEC_RPT_RX_FRAME_LEN,
					     &rpt),
			0);
	KUNIT_EXPECT_EQ(test, rpt.cmd, DCP_CEC_RPT_RX_FRAME);
	KUNIT_EXPECT_EQ(test, rpt.rx_msg.len, 1);
	KUNIT_EXPECT_EQ(test, rpt.rx_msg.msg[0], 0x04);

	for (i = 0; i < CEC_MAX_MSG_SIZE; i++)
		buf[hdr + i] = 0x10 + i;
	buf[hdr + 0x10] = 0x0f;
	KUNIT_ASSERT_EQ(test,
			dcp_cec_parse_report(buf,
					     hdr + DCP_CEC_RPT_RX_FRAME_LEN,
					     &rpt),
			0);
	KUNIT_EXPECT_EQ(test, rpt.rx_msg.len, 16);
	KUNIT_EXPECT_EQ(test, rpt.rx_msg.msg[15], 0x1f);

	/* HPD (cmd 3, 0x10 B). */
	build_epic_report(buf, DCP_CEC_GROUP_REPORT, DCP_CEC_RPT_HPD,
			  DCP_CEC_RPT_HPD_LEN, EPIC_SERVICE_CALL_MAGIC);
	buf[hdr + 0x00] = 1;
	KUNIT_ASSERT_EQ(test,
			dcp_cec_parse_report(buf, hdr + DCP_CEC_RPT_HPD_LEN,
					     &rpt),
			0);
	KUNIT_EXPECT_EQ(test, rpt.cmd, DCP_CEC_RPT_HPD);
	KUNIT_EXPECT_TRUE(test, rpt.hpd);

	/* EDID_ATTRS (cmd 4, 0x50 B). */
	build_epic_report(buf, DCP_CEC_GROUP_REPORT, DCP_CEC_RPT_EDID_ATTRS,
			  DCP_CEC_RPT_EDID_ATTRS_LEN, EPIC_SERVICE_CALL_MAGIC);
	put_unaligned_le16(0x3200, buf + hdr + 0x00);
	KUNIT_ASSERT_EQ(test,
			dcp_cec_parse_report(buf, sizeof(buf), &rpt), 0);
	KUNIT_EXPECT_EQ(test, rpt.cmd, DCP_CEC_RPT_EDID_ATTRS);
	KUNIT_EXPECT_EQ(test, rpt.phys_addr, 0x3200);

	/* Reject bad magic, wrong group, and truncated bodies. */
	build_epic_report(buf, DCP_CEC_GROUP_REPORT, DCP_CEC_RPT_HPD,
			  DCP_CEC_RPT_HPD_LEN, 0x12345678U);
	KUNIT_EXPECT_EQ(test,
			dcp_cec_parse_report(buf, hdr + DCP_CEC_RPT_HPD_LEN,
					     &rpt),
			-EINVAL);

	build_epic_report(buf, DCP_CEC_GROUP_CALL, DCP_CEC_RPT_HPD,
			  DCP_CEC_RPT_HPD_LEN, EPIC_SERVICE_CALL_MAGIC);
	KUNIT_EXPECT_EQ(test,
			dcp_cec_parse_report(buf, hdr + DCP_CEC_RPT_HPD_LEN,
					     &rpt),
			-EINVAL);

	build_epic_report(buf, DCP_CEC_GROUP_REPORT, DCP_CEC_RPT_EDID_ATTRS,
			  DCP_CEC_RPT_EDID_ATTRS_LEN, EPIC_SERVICE_CALL_MAGIC);
	KUNIT_EXPECT_EQ(test,
			dcp_cec_parse_report(buf, hdr + 0x20, &rpt),
			-EMSGSIZE);
}

static void hdmi_irq_hpd_pulse_classify_test(struct kunit *test)
{
	u64 base = 10 * NSEC_PER_MSEC;
	bool late_rising = false;
	u64 claimed_ns = 0;
	u64 falling_ns = 0;

	/* 0.7 ms converter IRQ_HPD pulse: arm on falling, forward on rising. */
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns, base, 0),
			DCP_HDMI_PULSE_ARM_TIMER);
	KUNIT_EXPECT_EQ(test, falling_ns, base);

	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns,
					       base + 700 * NSEC_PER_USEC, 1),
			DCP_HDMI_PULSE_SHORT);
	KUNIT_EXPECT_EQ(test, falling_ns, 0ULL);

	/* Sub-100 us pulse (or delayed falling hardirq) still forwards IRQ_HPD. */
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns, base, 0),
			DCP_HDMI_PULSE_ARM_TIMER);
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns,
					       base + 50 * NSEC_PER_USEC, 1),
			DCP_HDMI_PULSE_SHORT);
	KUNIT_EXPECT_EQ(test, falling_ns, 0ULL);

	/* Pulse that already rose before GPIO was sampled forwards IRQ_HPD. */
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns, base, 1),
			DCP_HDMI_PULSE_SHORT);

	/* Near 2.0 ms: if hrtimer already counted the pulse, do not double count. */
	claimed_ns = base + 2 * NSEC_PER_MSEC;
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns,
					       base + 2 * NSEC_PER_MSEC +
					       10 * NSEC_PER_USEC, 1),
			DCP_HDMI_PULSE_CLAIMED);
	KUNIT_EXPECT_EQ(test, claimed_ns, 0ULL);

	/* Stale claimed_ns (> 2 ms ago) never swallows a later fast pulse. */
	claimed_ns = base + 2 * NSEC_PER_MSEC;
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns,
					       base + 10 * NSEC_PER_MSEC, 1),
			DCP_HDMI_PULSE_SHORT);
	KUNIT_EXPECT_EQ(test, claimed_ns, 0ULL);

	/* Exact 2.0 ms boundary is still accepted as IRQ_HPD. */
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns, base, 0),
			DCP_HDMI_PULSE_ARM_TIMER);
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns,
					       base + DCP_HDMI_IRQ_HPD_MAX_NS,
					       1),
			DCP_HDMI_PULSE_SHORT);

	/* > 2.0 ms low pulse passes to the threaded HPD presence handler. */
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns, base, 0),
			DCP_HDMI_PULSE_ARM_TIMER);
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns,
					       base + DCP_HDMI_IRQ_HPD_MAX_NS + 1,
					       1),
			DCP_HDMI_PULSE_PASS);

	/* Rising edge after 2 ms timer already expired low wakes HPD thread. */
	falling_ns = 0;
	late_rising = true;
	KUNIT_EXPECT_EQ(test,
			dcp_hdmi_classify_edge(&falling_ns, &late_rising,
					       &claimed_ns,
					       base + 20 * NSEC_PER_MSEC, 1),
			DCP_HDMI_PULSE_PASS);
	KUNIT_EXPECT_FALSE(test, late_rising);
}

static struct kunit_case dcp_cec_cases[] = {
	KUNIT_CASE(cec_open_close_decode_test),
	KUNIT_CASE(cec_la_mask_framing_test),
	KUNIT_CASE(cec_edid_decode_test),
	KUNIT_CASE(cec_send_framing_test),
	KUNIT_CASE(cec_tx_status_mapping_test),
	KUNIT_CASE(cec_report_parsing_test),
	KUNIT_CASE(hdmi_irq_hpd_pulse_classify_test),
	{}
};

static struct kunit_suite dcp_cec_suite = {
	.name = "apple-dcp-cec",
	.test_cases = dcp_cec_cases,
};

kunit_test_suite(dcp_cec_suite);

MODULE_DESCRIPTION("KUnit tests for Apple Silicon DCP HDMI-CEC and IRQ_HPD");
MODULE_LICENSE("Dual MIT/GPL");
