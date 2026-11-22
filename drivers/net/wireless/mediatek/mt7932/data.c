// SPDX-License-Identifier: GPL-2.0-only
#include "mt7932.h"

static bool mt_bss_tx_allowed(struct mt7932 *m);
static void mt_bss_tx_account(struct mt7932 *m);

/* data_lock held: quota is packets admitted while absent, not TX-free credits.
 * A stopped netdev leaves unsubmitted packets in the ordinary bounded qdisc.
 */
static bool mt_bss_tx_allowed(struct mt7932 *m)
{
	return !m->bss_absent || m->bss_quota;
}

static void mt_bss_tx_account(struct mt7932 *m)
{
	if (m->bss_absent)
		m->bss_quota--;
}

void mt_data_clean_locked(struct mt7932 *m)
{
	struct mt7932_ring *q = &m->tx[0];

	while (q->queued && (le32_to_cpu(READ_ONCE(q->desc[q->tail].control)) & DONE)) {
		dma_rmb();
		m->data_retired[q->tail] = true;
		if (!m->data_token[q->tail])
			m->data_live[q->tail] = false;
		q->tail = (q->tail + 1) % q->count;
		q->queued--;
	}
	if (READ_ONCE(m->connected) && mt_rf_allowed(m) && mt_bss_tx_allowed(m) &&
	    !m->data_live[q->head] && q->queued < q->count - 1)
		netif_wake_queue(m->netdev);
#if IS_ENABLED(CONFIG_MT7932_AWDL)
	if (READ_ONCE(m->awdl.active) && READ_ONCE(m->awdl.sync_enabled) &&
	    READ_ONCE(m->awdl.slot_open) && !m->data_live[q->head] &&
	    q->queued < q->count - 1)
		netif_wake_queue(m->awdl.netdev);
#endif
}

/*
 * The stack marks a packet that has another one right behind it with
 * netdev_xmit_more(), so the TX0 doorbell is rung once per burst instead of
 * once per packet, and at least every MT7932_TX_BATCH packets so that the
 * device starts on a long burst early.
 */
#define MT7932_TX_BATCH 64

/* data_lock held: hand every published TX0 descriptor to the device. */
void mt_data_kick_locked(struct mt7932 *m)
{
	struct mt7932_ring *q = &m->tx[0];

	if (!m->data_unkicked)
		return;
	m->data_unkicked = 0;
	mt_write(m, q->reg + 8, q->head);
}

void mt_data_clean(struct mt7932 *m)
{
	unsigned long flags;

	if (!READ_ONCE(m->data_ready))
		return;
	spin_lock_irqsave(&m->data_lock, flags);
	/* The queue can be stopped from outside ndo_start_xmit() in the middle
	 * of a burst; no descriptor waits longer than the next interrupt.
	 */
	mt_data_kick_locked(m);
	mt_data_clean_locked(m);
	spin_unlock_irqrestore(&m->data_lock, flags);
}

/* response_lock held. EID11 replaces the BSS0 absence/quota state. It is not
 * a scan completion or a DMA fence; neither carrier nor tokens change here.
 */
void mt_bss_presence(struct mt7932 *m, const struct mt7932_event *event)
{
	const u8 *body = event->packet + 36;
	unsigned long flags;

	if (event->length < 40 || body[0] || !READ_ONCE(m->bss_active))
		return;
	/* response_lock: how often, and for how long, the firmware reports
	 * the BSS absent. TX is held while it is absent without quota.
	 */
	if (body[1] && !m->bss_absent) {
		m->bss_absent_events++;
		m->bss_absent_since = jiffies;
	} else if (!body[1] && m->bss_absent) {
		m->bss_absent_ms += jiffies_to_msecs(jiffies - m->bss_absent_since);
	}
	spin_lock_irqsave(&m->data_lock, flags);
	m->bss_absent = !!body[1];
	m->bss_quota = body[2];
	if (!mt_bss_tx_allowed(m)) {
		netif_stop_queue(m->netdev);
		mt_data_kick_locked(m);
	} else if (READ_ONCE(m->data_ready)) {
		mt_data_clean_locked(m);
	}
	spin_unlock_irqrestore(&m->data_lock, flags);
	dev_dbg_ratelimited(&m->pdev->dev, "BSS_PRESENCE: absent=%u quota=%u\n",
			    !!body[1], body[2]);
}

void mt_data_complete(struct mt7932 *m, const u8 *packet, size_t length)
{
	u16 tokens[512];
	unsigned long flags;
	int count, i;

	if (!READ_ONCE(m->data_ready))
		return;
	count = mt7932_tx_free(packet, length, tokens, ARRAY_SIZE(tokens));
	if (count < 0) {
		dev_warn_ratelimited(&m->pdev->dev, "TX_FREE_INVALID: %d length=%zu\n", count, length);
		return;
	}
	spin_lock_irqsave(&m->data_lock, flags);
	m->tx_free_events++;
	mt_data_clean_locked(m);
	for (i = 0; i < count; i++) {
		unsigned int token = tokens[i];

		if (token >= ARRAY_SIZE(m->data_live) || !m->data_live[token] || !m->data_token[token])
			continue;
		m->data_token[token] = false;
		m->tx_released++;
		if (m->data_retired[token])
			m->data_live[token] = false;
	}
	mt_data_clean_locked(m);
	spin_unlock_irqrestore(&m->data_lock, flags);
	dev_dbg_ratelimited(&m->pdev->dev, "TX_FREE: count=%d total=%llu\n", count, m->tx_released);
}

netdev_tx_t mt_net_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct mt7932 *m = *(struct mt7932 **)netdev_priv(netdev);
	struct mt7932_ring *q = &m->tx[0];
	struct mt7932_desc *desc;
	u8 *payload, *header;
	dma_addr_t dma;
	unsigned long flags;
	unsigned int slot;
	int ret;

	if (!READ_ONCE(m->connected) || !mt_rf_allowed(m) || !READ_ONCE(m->data_ready) ||
	    !READ_ONCE(m->peer_valid) || skb->len < ETH_HLEN || skb->len > ETH_FRAME_LEN + VLAN_HLEN)
		goto drop;
	spin_lock_irqsave(&m->data_lock, flags);
	/* Disconnect closes admission before synchronizing this publisher. */
	if (!READ_ONCE(m->connected) || READ_ONCE(m->disconnecting) || !mt_rf_allowed(m)) {
		spin_unlock_irqrestore(&m->data_lock, flags);
		goto drop;
	}
	mt_data_clean_locked(m);
	slot = q->head;
	if (!mt_bss_tx_allowed(m) || m->data_live[slot] || q->queued >= q->count - 1) {
		netif_stop_queue(netdev);
		mt_data_kick_locked(m);
		spin_unlock_irqrestore(&m->data_lock, flags);
		return NETDEV_TX_BUSY;
	}
	payload = m->data_payloads + 4096 * slot;
	header = m->data_headers + 64 * slot;
	ret = skb_copy_bits(skb, 0, payload, skb->len);
	if (!ret)
		ret = mt7932_data_header(header, payload, skb->len, m->data_payloads_dma + 4096 * slot,
					 slot, m->peer_wtbl, 0, 0, !m->connect_open);
	if (ret) {
		spin_unlock_irqrestore(&m->data_lock, flags);
		goto drop;
	}
	dma = m->data_headers_dma + 64 * slot;
	desc = &q->desc[slot];
	desc->address = cpu_to_le32(lower_32_bits(dma));
	desc->second = 0;
	desc->info = cpu_to_le32(upper_32_bits(dma));
	m->data_live[slot] = m->data_token[slot] = true;
	m->data_retired[slot] = false;
	dma_wmb();
	desc->control = cpu_to_le32((64 << 16) | BIT(30));
	q->head = (slot + 1) % q->count;
	q->queued++;
	dma_wmb();
	m->data_unkicked++;
	mt_bss_tx_account(m);
	m->station_tx_packets++;
	m->station_tx_bytes += skb->len;
	if (!mt_bss_tx_allowed(m) || m->data_live[q->head] || q->queued >= q->count - 1)
		netif_stop_queue(netdev);
	if (!netdev_xmit_more() || netif_queue_stopped(netdev) ||
	    m->data_unkicked >= MT7932_TX_BATCH)
		mt_data_kick_locked(m);
	spin_unlock_irqrestore(&m->data_lock, flags);
	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += skb->len;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
drop:
	/* The burst ends here: publish what earlier packets queued. */
	if (!netdev_xmit_more() && READ_ONCE(m->data_ready)) {
		spin_lock_irqsave(&m->data_lock, flags);
		mt_data_kick_locked(m);
		spin_unlock_irqrestore(&m->data_lock, flags);
	}
	netdev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

/* Width in MHz of a rate decoded by mt7932_rx_rate(). */
static int mt_rate_width(enum rate_info_bw bw)
{
	switch (bw) {
	case RATE_INFO_BW_20:
		return 20;
	case RATE_INFO_BW_40:
		return 40;
	case RATE_INFO_BW_80:
		return 80;
	case RATE_INFO_BW_160:
		return 160;
	default:
		return INT_MAX;
	}
}

void mt_data_receive(struct mt7932 *m, const struct mt7932_rx_frame *frame)
{
	struct sk_buff *skb;
	enum mt7932_rx_verdict verdict;
	unsigned long flags;
	int signal;

	if (!READ_ONCE(m->data_ready))
		return;
	/* Serialize peer identity and accounting with association retirement. */
	spin_lock_irqsave(&m->response_lock, flags);
	if (!m->connected || !m->peer_valid || m->disconnecting || m->stopping)
		goto unlock;
	if ((frame->unicast && frame->wtbl != m->peer_wtbl) ||
	    (!frame->unicast && frame->wtbl != m->peer_ids.wtbl[0] &&
	     frame->wtbl != m->peer_wtbl)) {
		m->rx_wrong_peer++;
		m->netdev->stats.rx_dropped++;
		goto unlock;
	}
	/* These count MSDU records, including split A-MSDU subframes. */
	if (frame->group4 && (frame->frame_control & IEEE80211_FCTL_RETRY))
		m->rx_retry++;
	if (frame->amsdu)
		m->rx_amsdu++;
	verdict = mt7932_rx_filter(m->duplicates, frame, m->connect_open);
	if (verdict == MT7932_RX_DUPLICATE) {
		m->rx_duplicate++;
		m->netdev->stats.rx_dropped++;
		goto unlock;
	}
	if (verdict == MT7932_RX_UNPROTECTED) {
		m->rx_unprotected++;
		m->netdev->stats.rx_dropped++;
		dev_dbg_ratelimited(&m->pdev->dev,
				    "RX_UNPROTECTED_DROPPED: cipher=%u mismatch=%u\n",
				    frame->sec_mode, frame->cipher_mismatch);
		goto unlock;
	}
	skb = netdev_alloc_skb_ip_align(m->netdev, frame->length);
	if (!skb) {
		m->rx_no_memory++;
		m->netdev->stats.rx_dropped++;
		goto unlock;
	}
	skb_put_data(skb, frame->data, frame->length);
	skb->protocol = eth_type_trans(skb, m->netdev);
	skb->ip_summed = CHECKSUM_NONE;
	m->netdev->stats.rx_packets++;
	m->netdev->stats.rx_bytes += frame->length;
	m->rx_ethernet++;
	spin_lock(&m->data_lock);
	m->station_rx_packets++;
	m->station_rx_time = jiffies;
	m->rx_band_records[frame->band == NL80211_BAND_2GHZ ? 0 :
			   frame->band == NL80211_BAND_5GHZ ? 1 : 2]++;
	m->station_rx_bytes += frame->length;
	signal = mt7932_rx_signal(frame, m->antenna_mask);
	if (frame->signal_valid && signal > -128) {
		m->station_signal = signal;
		m->station_signal_valid = true;
		mt_cqm_signal(m, signal);
	}
	if (frame->unicast && frame->rate_valid) {
		struct rate_info *rate = &m->station_rx_rate;

		m->station_rx_rate_valid = mt7932_rx_rate(frame->rxv, m->phy_cap[4], rate);
		/* No CCK on 5 GHz, and no rate wider than the associated channel. */
		if ((frame->band != NL80211_BAND_2GHZ && !((frame->rxv >> 24) & 15)) ||
		    mt_rate_width(rate->bw) > cfg80211_chandef_get_width(&m->connect_chandef))
			m->station_rx_rate_valid = false;
		m->station_rx_rate_time = jiffies;
	}
	spin_unlock(&m->data_lock);
	spin_unlock_irqrestore(&m->response_lock, flags);
	netif_rx(skb);
	dev_dbg_ratelimited(&m->pdev->dev, "ETHERNET_RX: bytes=%zu peer=%u total=%llu\n",
		frame->length, frame->wtbl, m->rx_ethernet);
	return;
unlock:
	spin_unlock_irqrestore(&m->response_lock, flags);
}

int mt_data_prepare(struct mt7932 *m)
{
	unsigned int band, i;
	u8 protection[12];
	int ret;

	if (m->tx[0].count != ARRAY_SIZE(m->data_live))
		return -EINVAL;
	spin_lock_init(&m->data_lock);
	m->data_headers = mt_alloc(m, 512 * 64, &m->data_headers_dma);
	m->data_payloads = mt_alloc(m, 512 * 4096, &m->data_payloads_dma);
	if (!m->data_headers || !m->data_payloads)
		return -ENOMEM;
	/* Source-defined fresh MAC init; preserve all unrelated register bits. */
	mt_rmw(m, 0xf004, 0xfff8, 0x3000);
	mt_rmw(m, 0xf000, 0, 0x88000);
	for (i = 0; i < 20; i++) {
		mt_rmw(m, 0x34230, 0x3ff, i | BIT(12));
		if (mt_poll(m, 0x34230, BIT(31), 0, 5000))
			return -ETIMEDOUT;
	}
	for (band = 0; band < 2; band++) {
		u32 offset = band ? 0x80000 : 0;

		mt_rmw(m, 0x210f4 + offset, 0x3f, 0x3f | BIT(17) | BIT(18));
		mt_rmw(m, 0x217c4 + offset, 0, BIT(30));
		mt_rmw(m, 0x21780 + offset, 0, BIT(30));
		mt_rmw(m, 0x24804 + offset, 0, BIT(8) | BIT(9));
		mt_rmw(m, 0x21e00 + offset, 0xfff8 | BIT(23), 0x3000);
		mt_rmw(m, 0x23408 + offset, 0xc3000000, 0x03000000);
	}
	mt7932_rts_config(protection);
	ret = mt_request_ext(m, 0xed, 0x3e, true, true, true,
			     protection, sizeof(protection));
	if (ret)
		return ret;
	/* The selected response parser establishes sequence completion only,
	 * not a command-specific firmware status. No payload-status invention.
	 */
	dev_dbg(&m->pdev->dev, "MAC_RTS_RESPONSE: sequence matched, EID=%02x length=%zu\n",
		m->reply[28], m->reply_length);
	/* Publish the initialized data_lock and DMA state to failure handlers. */
	smp_store_release(&m->data_ready, true);
	dev_dbg(&m->pdev->dev, "ETHERNET_PREPARED: TX0 retained coherent slots=512, RX translation enabled\n");
	return 0;
}
