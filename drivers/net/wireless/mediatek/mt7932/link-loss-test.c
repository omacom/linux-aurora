// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <linux/unaligned.h>

#include "mt7932.h"

struct mt_link_loss_test {
	struct mt7932 m;
	u8 *packet;
	struct mt7932_event event;
	unsigned int ended, recovered;
};

static void mt_link_loss_test_release(struct device *dev)
{
	/* The enclosing synthetic PCI device is owned by KUnit. */
}

static void mt_link_loss_test_disconnect_work(struct work_struct *work)
{
	struct mt_link_loss_test *ctx = container_of(work, struct mt_link_loss_test,
						     m.disconnect_work);

	ctx->ended++;
}

static void mt_link_loss_test_recover_work(struct work_struct *work)
{
	struct mt_link_loss_test *ctx = container_of(work, struct mt_link_loss_test,
						     m.recover_work);

	ctx->recovered++;
}

static void mt_link_loss_test_cleanup(void *data)
{
	struct mt7932 *m = data;

	cancel_work_sync(&m->recover_work);
	cancel_work_sync(&m->disconnect_work);
	free_netdev(m->netdev);
	put_device(&m->pdev->dev);
}

static int mt_link_loss_test_init(struct kunit *test)
{
	struct mt_link_loss_test *ctx;
	struct mt7932 *m;
	int ret;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->packet = kunit_kzalloc(test, 36 + MT7932_BEACON_TIMEOUT_LENGTH, GFP_KERNEL);
	if (!ctx->packet)
		return -ENOMEM;
	m = &ctx->m;
	m->pdev = kunit_kzalloc(test, sizeof(*m->pdev), GFP_KERNEL);
	if (!m->pdev)
		return -ENOMEM;
	device_initialize(&m->pdev->dev);
	m->pdev->dev.release = mt_link_loss_test_release;
	ret = dev_set_name(&m->pdev->dev, "mt7932-link-loss-test");
	if (ret) {
		put_device(&m->pdev->dev);
		return ret;
	}
	INIT_WORK(&m->disconnect_work, mt_link_loss_test_disconnect_work);
	INIT_WORK(&m->recover_work, mt_link_loss_test_recover_work);
	m->netdev = alloc_etherdev(sizeof(m));
	if (!m->netdev) {
		put_device(&m->pdev->dev);
		return -ENOMEM;
	}
	ret = kunit_add_action_or_reset(test, mt_link_loss_test_cleanup, m);
	if (ret)
		return ret;
	*(struct mt7932 **)netdev_priv(m->netdev) = m;
	spin_lock_init(&m->response_lock);
	spin_lock_init(&m->data_lock);
	init_completion(&m->disconnect_done);
	m->interface_registered = true;
	m->connected = true;
	m->peer_valid = true;
	/* Published as probe publishes it, for the handler's acquire. */
	smp_store_release(&m->data_ready, true);
	netif_carrier_on(m->netdev);
	netif_start_queue(m->netdev);
	ctx->event.eid = MT7932_EID_BEACON_TIMEOUT;
	ctx->event.packet = ctx->packet;
	ctx->event.length = 36 + MT7932_BEACON_TIMEOUT_LENGTH;
	test->priv = ctx;
	return 0;
}

/* One beacon timeout for BSS @bss with @reason; returns whether it ended the link. */
static bool mt_link_loss_test_event(struct mt_link_loss_test *ctx, u8 bss, u8 reason)
{
	struct mt7932 *m = &ctx->m;
	unsigned int ended = ctx->ended;
	unsigned long flags;

	ctx->packet[36] = bss;
	ctx->packet[37] = reason;
	put_unaligned_le16(WLAN_REASON_DEAUTH_LEAVING, ctx->packet + 38);
	spin_lock_irqsave(&m->response_lock, flags);
	mt_beacon_timeout_event(m, &ctx->event);
	spin_unlock_irqrestore(&m->response_lock, flags);
	flush_work(&m->disconnect_work);
	return ctx->ended != ended;
}

static void mt_link_loss_test_heard(struct mt7932 *m, unsigned long ago)
{
	m->station_rx_packets = 1;
	m->station_rx_time = jiffies - ago;
}

static void mt_beacon_loss_span_test(struct kunit *test)
{
	/* Seven beacon intervals: 100 TU is 102.4 ms. */
	KUNIT_EXPECT_EQ(test, mt7932_beacon_loss_ms(100), 717U);
	KUNIT_EXPECT_EQ(test, mt7932_beacon_loss_ms(1), 8U);
	KUNIT_EXPECT_EQ(test, mt7932_beacon_loss_ms(1000), 7168U);
	/* An unknown interval gives no span: no frame keeps the link. */
	KUNIT_EXPECT_EQ(test, mt7932_beacon_loss_ms(0), 0U);
}

static void mt_beacon_timeout_ends_connection_test(struct kunit *test)
{
	struct mt_link_loss_test *ctx = test->priv;
	struct mt7932 *m = &ctx->m;

	/* Nothing received since the association: the link is gone. */
	KUNIT_EXPECT_TRUE(test, mt_link_loss_test_event(ctx, 0, 3));
	KUNIT_EXPECT_FALSE(test, m->connected);
	KUNIT_EXPECT_TRUE(test, m->disconnecting);
	/* The worker asks the firmware to leave, then reports as from the AP. */
	KUNIT_EXPECT_FALSE(test, m->disconnect_seen);
	KUNIT_EXPECT_FALSE(test, m->disconnect_event_received);
	KUNIT_EXPECT_FALSE(test, m->disconnect_local);
	KUNIT_EXPECT_EQ(test, m->disconnect_reason, WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY);
	KUNIT_EXPECT_FALSE(test, netif_carrier_ok(m->netdev));
	KUNIT_EXPECT_TRUE(test, netif_queue_stopped(m->netdev));
	/* One end per connection: a second report while it ends changes nothing. */
	KUNIT_EXPECT_FALSE(test, mt_link_loss_test_event(ctx, 0, 3));
	KUNIT_EXPECT_EQ(test, ctx->ended, 1U);
}

static void mt_beacon_timeout_recent_rx_test(struct kunit *test)
{
	struct mt_link_loss_test *ctx = test->priv;
	struct mt7932 *m = &ctx->m;

	/* A frame from the AP within the beacon-loss span keeps the link, whatever
	 * cause the firmware gives...
	 */
	m->station_beacon_tu = 100;
	mt_link_loss_test_heard(m, 0);
	KUNIT_EXPECT_FALSE(test, mt_link_loss_test_event(ctx, 0, 3));
	KUNIT_EXPECT_FALSE(test, mt_link_loss_test_event(ctx, 0, 0x0e));
	KUNIT_EXPECT_TRUE(test, m->connected);
	KUNIT_EXPECT_TRUE(test, netif_carrier_ok(m->netdev));
	/* ...but not without a known beacon interval. */
	m->station_beacon_tu = 0;
	KUNIT_EXPECT_TRUE(test, mt_link_loss_test_event(ctx, 0, 3));
	KUNIT_EXPECT_FALSE(test, m->connected);
}

static void mt_beacon_timeout_stale_rx_test(struct kunit *test)
{
	struct mt_link_loss_test *ctx = test->priv;

	/* The last frame came before the span: the link is gone. */
	ctx->m.station_beacon_tu = 100;
	mt_link_loss_test_heard(&ctx->m, msecs_to_jiffies(mt7932_beacon_loss_ms(100)) + 1);
	KUNIT_EXPECT_TRUE(test, mt_link_loss_test_event(ctx, 0, 0));
	KUNIT_EXPECT_FALSE(test, ctx->m.connected);
}

static void mt_beacon_timeout_ignored_test(struct kunit *test)
{
	struct mt_link_loss_test *ctx = test->priv;
	struct mt7932 *m = &ctx->m;

	/* Another BSS's AP (AWDL's is 1). */
	KUNIT_EXPECT_FALSE(test, mt_link_loss_test_event(ctx, 1, 3));
	KUNIT_EXPECT_TRUE(test, m->connected);
	/* A short event. */
	ctx->event.length--;
	KUNIT_EXPECT_FALSE(test, mt_link_loss_test_event(ctx, 0, 3));
	KUNIT_EXPECT_TRUE(test, m->connected);
	ctx->event.length++;
	/* A connection that is already ending, or none. */
	m->disconnecting = true;
	KUNIT_EXPECT_FALSE(test, mt_link_loss_test_event(ctx, 0, 3));
	m->disconnecting = false;
	m->stopping = true;
	KUNIT_EXPECT_FALSE(test, mt_link_loss_test_event(ctx, 0, 3));
	m->stopping = false;
	m->connected = false;
	KUNIT_EXPECT_FALSE(test, mt_link_loss_test_event(ctx, 0, 3));
	KUNIT_EXPECT_FALSE(test, m->disconnecting);
	KUNIT_EXPECT_EQ(test, ctx->ended, 0U);
}

static unsigned int mt_link_loss_test_recover(struct mt_link_loss_test *ctx)
{
	mt_request_recovery(&ctx->m, "test");
	flush_work(&ctx->m.recover_work);
	return ctx->recovered;
}

static void mt_recovery_request_test(struct kunit *test)
{
	struct mt_link_loss_test *ctx = test->priv;
	struct mt7932 *m = &ctx->m;
	int restarts = mt_restart_count();

	/* Probe's own failure path settles a radio that never finished it. */
	KUNIT_EXPECT_EQ(test, mt_link_loss_test_recover(ctx), 0U);
	m->ready = true;
	m->stopping = true;
	KUNIT_EXPECT_EQ(test, mt_link_loss_test_recover(ctx), 0U);
	m->stopping = false;
	KUNIT_EXPECT_EQ(test, mt_restart_count(), restarts);
	KUNIT_EXPECT_FALSE(test, mt_restart_done(m));
	/* One reset per binding, however many latches ask for it, and counted once. */
	KUNIT_EXPECT_EQ(test, mt_link_loss_test_recover(ctx), 1U);
	KUNIT_EXPECT_EQ(test, mt_link_loss_test_recover(ctx), 1U);
	KUNIT_EXPECT_EQ(test, mt_restart_count(), restarts + 1);
	/* The binding the restart brings up says it is back, once. */
	KUNIT_EXPECT_TRUE(test, mt_restart_done(m));
	KUNIT_EXPECT_FALSE(test, mt_restart_done(m));
}

static struct kunit_case mt_link_loss_test_cases[] = {
	KUNIT_CASE(mt_beacon_loss_span_test),
	KUNIT_CASE(mt_beacon_timeout_ends_connection_test),
	KUNIT_CASE(mt_beacon_timeout_recent_rx_test),
	KUNIT_CASE(mt_beacon_timeout_stale_rx_test),
	KUNIT_CASE(mt_beacon_timeout_ignored_test),
	KUNIT_CASE(mt_recovery_request_test),
	{}
};

static struct kunit_suite mt_link_loss_test_suite = {
	.name = "mt7932-link-loss",
	.init = mt_link_loss_test_init,
	.test_cases = mt_link_loss_test_cases,
};

kunit_test_suite(mt_link_loss_test_suite);
