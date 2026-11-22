// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * KUnit integration tests for Apple Silicon DCP HDMI-CEC adapter lifecycle,
 * deferred EDID PA, HPD-low close, singleton reuse on Terminate, and TX
 * sequence guarding.
 */

#include <kunit/device.h>
#include <kunit/test.h>

struct dcp_cec_test_ctx {
	struct device *dev;
	struct apple_dcp *dcp;
	struct apple_dcp_afkep *ep;
	struct apple_epic_service srv[2];
	struct apple_epic_service port_srv;
	/* Serializes mock RPC counters and return state across kworker threads. */
	spinlock_t lock;

	unsigned int open_calls;
	unsigned int close_calls;
	unsigned int set_la_calls;
	unsigned int get_edid_calls;
	unsigned int send_calls;

	u16 last_la_mask;
	struct cec_msg last_tx_msg;
	u8 last_tx_retries;
	unsigned int last_tx_timeout_ms;

	u16 edid_pa;
	u32 send_retcode;
	int forced_err;

	void (*on_open)(struct dcp_cec_test_ctx *ctx);
	void (*on_close)(struct dcp_cec_test_ctx *ctx);
	void (*on_send)(struct dcp_cec_test_ctx *ctx);
};

static struct dcp_cec_test_ctx *dcp_cec_active_ctx;

static int dcp_cec_stub_call(struct apple_epic_service *srv, u32 cmd,
			     const void *req, void *resp, size_t body_len,
			     unsigned int timeout_ms)
{
	struct dcp_cec_test_ctx *ctx = dcp_cec_active_ctx;
	void (*hook)(struct dcp_cec_test_ctx *ctx) = NULL;
	const u8 *in = req;
	u8 *out = resp;

	if (!ctx)
		return -ENODEV;

	memset(out, 0, body_len);

	scoped_guard(spinlock_irqsave, &ctx->lock) {
		if (ctx->forced_err)
			return ctx->forced_err;

		switch (cmd) {
		case DCP_CEC_CMD_OPEN:
			if (body_len != DCP_CEC_OPEN_LEN)
				return -EMSGSIZE;
			ctx->open_calls++;
			put_unaligned_le32(0, out + 0x00);
			put_unaligned_le64(1, out + 0x10);
			hook = ctx->on_open;
			ctx->on_open = NULL;
			break;

		case DCP_CEC_CMD_CLOSE:
			if (body_len != DCP_CEC_CLOSE_LEN)
				return -EMSGSIZE;
			ctx->close_calls++;
			put_unaligned_le32(0, out + 0x00);
			hook = ctx->on_close;
			ctx->on_close = NULL;
			break;

		case DCP_CEC_CMD_SET_LOGICAL_ADDR_MASK:
			if (body_len != DCP_CEC_SET_LA_MASK_LEN)
				return -EMSGSIZE;
			ctx->set_la_calls++;
			ctx->last_la_mask = get_unaligned_le16(in + 0x10);
			put_unaligned_le32(0, out + 0x00);
			break;

		case DCP_CEC_CMD_GET_EDID_ATTRS:
			if (body_len != DCP_CEC_EDID_ATTRS_LEN)
				return -EMSGSIZE;
			ctx->get_edid_calls++;
			put_unaligned_le16(ctx->edid_pa, out + 0x00);
			break;

		case DCP_CEC_CMD_SEND_CEC:
			if (body_len != DCP_CEC_SEND_LEN)
				return -EMSGSIZE;
			ctx->send_calls++;
			ctx->last_tx_msg.len = (in[0x10] & 0x0f) + 1;
			memcpy(ctx->last_tx_msg.msg, in + 0x00,
			       ctx->last_tx_msg.len);
			ctx->last_tx_retries = in[0x20];
			ctx->last_tx_timeout_ms = timeout_ms;
			put_unaligned_le32(ctx->send_retcode, out + 0x30);
			hook = ctx->on_send;
			ctx->on_send = NULL;
			break;

		default:
			return -EOPNOTSUPP;
		}
	}

	if (hook)
		hook(ctx);

	return 0;
}

static void dcp_cec_init_fake_srv(struct dcp_cec_test_ctx *ctx, unsigned int idx,
				  u32 channel)
{
	struct apple_epic_service *s = &ctx->srv[idx];

	memset(s, 0, sizeof(*s));
	spin_lock_init(&s->lock);
	s->ep = ctx->ep;
	s->ops = &dcp_cec_desktop_ops[1];
	s->channel = channel;
	s->enabled = true;
}

static void dcp_cec_emit_report(struct apple_epic_service *srv, u32 cmd,
				const void *body, u32 body_len)
{
	const size_t hdr = sizeof(struct epic_service_call);
	u8 buf[sizeof(struct epic_service_call) + DCP_CEC_RPT_EDID_ATTRS_LEN] = {};
	struct epic_service_call *call = (struct epic_service_call *)buf;

	call->group = cpu_to_le16(DCP_CEC_GROUP_REPORT);
	call->command = cpu_to_le32(cmd);
	call->data_len = cpu_to_le32(body_len);
	call->magic = cpu_to_le32(EPIC_SERVICE_CALL_MAGIC);
	if (body && body_len)
		memcpy(buf + hdr, body, body_len);

	dcp_cec_service_report(srv, EPIC_SUBTYPE_STD_SERVICE, buf,
			       hdr + body_len);
}

static struct dcp_cec_test_ctx *dcp_cec_test_setup(struct kunit *test)
{
	struct dcp_cec_test_ctx *ctx;
	struct cec_adapter *adap;
	struct dcp_cec *cec;
	int ret;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx);
	spin_lock_init(&ctx->lock);

	ctx->dev = kunit_device_register(test, "apple-dcp-cec-kunit");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->dev);

	ctx->dcp = kunit_kzalloc(test, sizeof(*ctx->dcp), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->dcp);
	ctx->dcp->dev = ctx->dev;
	ctx->dcp->fixed_connector_type = DRM_MODE_CONNECTOR_HDMIA;

	ctx->ep = kunit_kzalloc(test, sizeof(*ctx->ep), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->ep);
	ctx->ep->dcp = ctx->dcp;
	ctx->ep->endpoint = AV_AUDIO_ENDPOINT_DESKTOP;
	ctx->ep->compact = true;

	ctx->port_srv.ep = ctx->ep;
	ctx->dcp->dptxport[0].service = &ctx->port_srv;

	dcp_cec_init_fake_srv(ctx, 0, 1);
	dcp_cec_init_fake_srv(ctx, 1, 2);

	cec = dcp_cec_ensure_state(ctx->dcp);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, cec);
	dcp_cec_active_ctx = ctx;
	cec->test_call = dcp_cec_stub_call;

	adap = cec_allocate_adapter(&dcp_cec_adap_ops, cec, "apple-cec-test",
				    CEC_CAP_DEFAULTS | CEC_CAP_NEEDS_HPD,
				    CEC_MAX_LOG_ADDRS);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, adap);
	adap->xfer_timeout_ms = DCP_CEC_ADAP_XFER_TIMEOUT_MS;

	WRITE_ONCE(cec->adap, adap);
	ret = cec_register_adapter(adap, ctx->dev);
	if (ret) {
		WRITE_ONCE(cec->adap, NULL);
		cec_delete_adapter(adap);
	}
	KUNIT_ASSERT_EQ(test, ret, 0);

	return ctx;
}

static void dcp_cec_test_teardown(struct dcp_cec_test_ctx *ctx)
{
	dcp_cec_unregister(ctx->dcp);
	dcp_cec_active_ctx = NULL;
}

static void dcp_cec_reuse_slot_on_close(struct dcp_cec_test_ctx *ctx)
{
	/* Simulate afk_compact_slot() reusing singleton srv[0] in place. */
	scoped_guard(spinlock_irqsave, &ctx->srv[0].lock) {
		ctx->srv[0].channel = 9;
		ctx->srv[0].enabled = true;
	}
}

static void cec_adap_pa_and_terminate_test(struct kunit *test)
{
	struct dcp_cec_test_ctx *ctx = dcp_cec_test_setup(test);
	struct dcp_cec *cec = ctx->dcp->cec;
	struct cec_adapter *adap = cec->adap;
	u8 edid_body[DCP_CEC_RPT_EDID_ATTRS_LEN] = {};
	u8 term_body[DCP_CEC_RPT_TERMINATE_LEN] = {};
	u8 hpd_body[DCP_CEC_RPT_HPD_LEN] = {};

	/*
	 * On initial plug, the EDID query can return an invalid physical
	 * address until a later poll or EDID attributes report.
	 */
	ctx->edid_pa = CEC_PHYS_ADDR_INVALID;
	dcp_cec_service_init(&ctx->srv[0], "dcpav-cec-interface-epic",
			     "DCPAVCECInterface", 0);
	flush_work(&cec->lifecycle_work);

	KUNIT_EXPECT_EQ(test, ctx->open_calls, 1U);
	KUNIT_EXPECT_EQ(test, ctx->get_edid_calls, 1U);
	KUNIT_EXPECT_EQ(test, adap->phys_addr, CEC_PHYS_ADDR_INVALID);

	/* Hotplug EDID poll picks up PA once DDC EDID finishes. */
	ctx->edid_pa = 0x2100;
	dcp_cec_poll_edid(ctx->dcp);
	flush_work(&cec->lifecycle_work);
	KUNIT_EXPECT_EQ(test, adap->phys_addr, 0x2100);

	/* EDID_ATTRS async report updates PA to 3.4.0.0. */
	put_unaligned_le16(0x3400, edid_body + 0x00);
	dcp_cec_emit_report(&ctx->srv[0], DCP_CEC_RPT_EDID_ATTRS,
			    edid_body, sizeof(edid_body));
	flush_work(&cec->lifecycle_work);
	KUNIT_EXPECT_EQ(test, adap->phys_addr, 0x3400);

	/* HPD=0 async report closes the client and invalidates PA. */
	hpd_body[0] = 0;
	dcp_cec_emit_report(&ctx->srv[0], DCP_CEC_RPT_HPD,
			    hpd_body, sizeof(hpd_body));
	flush_work(&cec->lifecycle_work);
	KUNIT_EXPECT_EQ(test, ctx->close_calls, 1U);
	KUNIT_EXPECT_EQ(test, adap->phys_addr, CEC_PHYS_ADDR_INVALID);

	/* An HPD-high report triggers a new open and EDID query. */
	ctx->edid_pa = 0x1000;
	hpd_body[0] = 1;
	dcp_cec_emit_report(&ctx->srv[0], DCP_CEC_RPT_HPD,
			    hpd_body, sizeof(hpd_body));
	flush_work(&cec->lifecycle_work);
	KUNIT_EXPECT_EQ(test, ctx->open_calls, 2U);
	KUNIT_EXPECT_EQ(test, adap->phys_addr, 0x1000);

	/*
	 * Terminate while afk_compact_slot() reuses singleton srv[0] in place:
	 * afk_service_retire_channel(srv[0], 1) must NOT disable channel 9.
	 */
	ctx->on_close = dcp_cec_reuse_slot_on_close;
	dcp_cec_emit_report(&ctx->srv[0], DCP_CEC_RPT_TERMINATE,
			    term_body, sizeof(term_body));
	flush_work(&cec->lifecycle_work);

	KUNIT_EXPECT_EQ(test, ctx->close_calls, 2U);
	KUNIT_EXPECT_TRUE(test, ctx->srv[0].enabled);
	KUNIT_EXPECT_EQ(test, ctx->srv[0].channel, 9U);

	/* Retiring matching channel 9 disables and marks the slot retired. */
	afk_service_retire_channel(&ctx->srv[0], 9);
	KUNIT_EXPECT_TRUE(test, afk_service_retired(&ctx->srv[0], 9));
	KUNIT_EXPECT_FALSE(test, ctx->srv[0].enabled);

	dcp_cec_test_teardown(ctx);
}

static void dcp_cec_close_during_open(struct dcp_cec_test_ctx *ctx)
{
	/* Simulate HPD-low port close racing an in-flight Open. */
	scoped_guard(spinlock_irqsave, &ctx->dcp->cec->lock)
		ctx->dcp->cec->close_gen++;
}

static void cec_adap_la_mask_and_singleton_replace_test(struct kunit *test)
{
	struct dcp_cec_test_ctx *ctx = dcp_cec_test_setup(test);
	struct dcp_cec *cec = ctx->dcp->cec;
	struct cec_adapter *adap = cec->adap;

	ctx->edid_pa = 0x1000;

	dcp_cec_service_init(&ctx->srv[0], "dcpav-cec-interface-epic",
			     "DCPAVCECInterface", 0);
	flush_work(&cec->lifecycle_work);

	/* Enable + claim LA 4 -> 0x8000 then 0x8010. */
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_enable(adap, true), 0);
	KUNIT_EXPECT_EQ(test, ctx->last_la_mask, 0x8000);
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_log_addr(adap, 4), 0);
	KUNIT_EXPECT_EQ(test, ctx->last_la_mask, 0x8010);

	/*
	 * Singleton replacement via teardown + init: teardown must NOT send
	 * CMD_CLOSE (slot is reused in place) and must drop users to 0;
	 * init on replacement opens it and restores 0x8010 automatically.
	 */
	afk_service_retire(&ctx->srv[0]);
	dcp_cec_service_teardown(&ctx->srv[0]);
	KUNIT_EXPECT_EQ(test, ctx->close_calls, 0U);
	KUNIT_EXPECT_EQ(test, ctx->srv[0].users, 0U);

	ctx->last_la_mask = 0;
	dcp_cec_service_init(&ctx->srv[1], "dcpav-cec-interface-epic",
			     "DCPAVCECInterface", 0);
	flush_work(&cec->lifecycle_work);
	KUNIT_EXPECT_EQ(test, ctx->open_calls, 2U);
	KUNIT_EXPECT_EQ(test, ctx->last_la_mask, 0x8010);

	/* Pre-HPD-low close closes the active CEC client and invalidates PA. */
	dcp_cec_close_port(ctx->dcp, &ctx->port_srv);
	flush_work(&cec->lifecycle_work);
	KUNIT_EXPECT_EQ(test, ctx->close_calls, 1U);
	KUNIT_EXPECT_EQ(test, adap->phys_addr, CEC_PHYS_ADDR_INVALID);

	/* An Open raced by HPD-low close immediately closes again. */
	ctx->on_open = dcp_cec_close_during_open;
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_enable(adap, true), 0);
	KUNIT_EXPECT_EQ(test, ctx->open_calls, 3U);
	KUNIT_EXPECT_EQ(test, ctx->close_calls, 2U);
	KUNIT_EXPECT_EQ(test, cec->opened_gen, 0ULL);

	/* Re-open cleanly, then verify unregister keeps cec->srv pinned. */
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_enable(adap, true), 0);
	KUNIT_EXPECT_EQ(test, ctx->open_calls, 4U);

	/* adap_log_addr(INVALID) and adap_enable(false) always return 0 even on AFK error. */
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_log_addr(adap, CEC_LOG_ADDR_INVALID), 0);
	KUNIT_EXPECT_EQ(test, ctx->last_la_mask, 0x8000);

	ctx->forced_err = -EIO;
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_log_addr(adap, CEC_LOG_ADDR_INVALID), 0);
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_enable(adap, false), 0);
	ctx->forced_err = 0;

	dcp_cec_unregister(ctx->dcp);
	KUNIT_EXPECT_PTR_EQ(test, cec->srv, &ctx->srv[1]);
	KUNIT_EXPECT_EQ(test, ctx->close_calls, 3U);

	/*
	 * Component rebind re-registers an adapter and re-opens the pinned
	 * cec->srv without waiting for a hotplug relink.
	 */
	adap = cec_allocate_adapter(&dcp_cec_adap_ops, cec, "apple-cec-rebind",
				    CEC_CAP_DEFAULTS | CEC_CAP_NEEDS_HPD,
				    CEC_MAX_LOG_ADDRS);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, adap);
	cec->stopping = false;
	cec->opened_gen = 0;
	cec->open_attempted_gen = 0;
	WRITE_ONCE(cec->adap, adap);
	KUNIT_ASSERT_EQ(test, cec_register_adapter(adap, ctx->dev), 0);
	queue_work(system_dfl_wq, &cec->lifecycle_work);
	flush_work(&cec->lifecycle_work);
	KUNIT_EXPECT_EQ(test, ctx->open_calls, 5U);
	KUNIT_EXPECT_EQ(test, adap->phys_addr, 0x1000);

	dcp_cec_test_teardown(ctx);
}

static void dcp_cec_supersede_tx_during_send(struct dcp_cec_test_ctx *ctx)
{
	struct dcp_cec *cec = ctx->dcp->cec;
	struct cec_adapter *adap = cec->adap;
	struct cec_msg newer = { .len = 2, .msg = { 0x40, 0x83 } };

	/*
	 * Simulate CEC core aborting TX #1 (adap_enable(false) -> true) and
	 * starting TX #2 before the AFK reply for TX #1 arrives.
	 */
	dcp_cec_adap_enable(adap, false);
	dcp_cec_adap_enable(adap, true);
	ctx->send_retcode = DCP_CEC_RET_OK;
	adap->transmit_in_progress = true;
	dcp_cec_adap_transmit(adap, 1, 0, &newer);
}

static void cec_adap_tx_and_rx_test(struct kunit *test)
{
	struct dcp_cec_test_ctx *ctx = dcp_cec_test_setup(test);
	struct dcp_cec *cec = ctx->dcp->cec;
	struct cec_adapter *adap = cec->adap;
	u8 rx_body[DCP_CEC_RPT_RX_FRAME_LEN] = {};
	struct cec_msg tx = {};

	ctx->edid_pa = 0x1000;

	dcp_cec_service_init(&ctx->srv[0], "dcpav-cec-interface-epic",
			     "DCPAVCECInterface", 0);
	flush_work(&cec->lifecycle_work);
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_enable(adap, true), 0);

	/* Non-blocking adap_transmit queues tx_work and calls SEND_CEC (7500 ms). */
	tx.len = 2;
	tx.msg[0] = 0x40;
	tx.msg[1] = 0x8f; /* Give Device Power Status */
	ctx->send_retcode = DCP_CEC_RET_OK;
	adap->transmit_in_progress = true;
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_transmit(adap, 2, 0, &tx), 0);
	flush_work(&cec->tx_work);
	KUNIT_EXPECT_FALSE(test, adap->transmit_in_progress);

	KUNIT_EXPECT_EQ(test, ctx->send_calls, 1U);
	KUNIT_EXPECT_EQ(test, ctx->last_tx_msg.len, 2U);
	KUNIT_EXPECT_EQ(test, ctx->last_tx_msg.msg[0], 0x40);
	KUNIT_EXPECT_EQ(test, ctx->last_tx_msg.msg[1], 0x8f);
	KUNIT_EXPECT_EQ(test, ctx->last_tx_retries, 1U);
	KUNIT_EXPECT_EQ(test, ctx->last_tx_timeout_ms,
			(unsigned int)DCP_CEC_TX_TIMEOUT_MS);

	/* Verify TIMEOUT and ABORTED complete cleanly without WARN_ON. */
	ctx->send_retcode = DCP_CEC_RET_TIMEOUT;
	adap->transmit_in_progress = true;
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_transmit(adap, 1, 0, &tx), 0);
	flush_work(&cec->tx_work);
	KUNIT_EXPECT_FALSE(test, adap->transmit_in_progress);
	KUNIT_EXPECT_EQ(test, ctx->send_calls, 2U);

	ctx->send_retcode = DCP_CEC_RET_ABORTED;
	adap->transmit_in_progress = true;
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_transmit(adap, 1, 0, &tx), 0);
	flush_work(&cec->tx_work);
	KUNIT_EXPECT_FALSE(test, adap->transmit_in_progress);
	KUNIT_EXPECT_EQ(test, ctx->send_calls, 3U);

	/*
	 * Stale TX #1 returning NACK after TX #2 was queued must not complete
	 * TX #2 with NACK; tx_work loops to send and complete TX #2 with OK.
	 */
	ctx->send_retcode = DCP_CEC_RET_NACK;
	ctx->on_send = dcp_cec_supersede_tx_during_send;
	adap->transmit_in_progress = true;
	KUNIT_EXPECT_EQ(test, dcp_cec_adap_transmit(adap, 1, 0, &tx), 0);
	flush_work(&cec->tx_work);
	KUNIT_EXPECT_FALSE(test, adap->transmit_in_progress);
	KUNIT_EXPECT_EQ(test, ctx->send_calls, 5U);
	KUNIT_EXPECT_EQ(test, ctx->last_tx_msg.msg[1], 0x83);

	/* RX_FRAME report queues to rx_work and delivers via cec_received_msg_ts. */
	rx_body[0x00] = 0x04;
	rx_body[0x01] = 0x90;
	rx_body[0x02] = 0x00;
	rx_body[0x10] = 0x02; /* len = 3 */
	dcp_cec_emit_report(&ctx->srv[0], DCP_CEC_RPT_RX_FRAME,
			    rx_body, sizeof(rx_body));
	flush_work(&cec->rx_work);
	KUNIT_EXPECT_EQ(test, cec->rx_count, 0U);

	dcp_cec_test_teardown(ctx);
}

static struct kunit_case dcp_cec_adap_cases[] = {
	KUNIT_CASE(cec_adap_pa_and_terminate_test),
	KUNIT_CASE(cec_adap_la_mask_and_singleton_replace_test),
	KUNIT_CASE(cec_adap_tx_and_rx_test),
	{}
};

static struct kunit_suite dcp_cec_adap_suite = {
	.name = "apple-dcp-cec-adap",
	.test_cases = dcp_cec_adap_cases,
};

kunit_test_suite(dcp_cec_adap_suite);
