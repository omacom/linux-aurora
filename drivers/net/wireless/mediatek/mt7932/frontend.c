// SPDX-License-Identifier: GPL-2.0-only
#include "mt7932.h"

static void mt_scan_finish_work(struct work_struct *work);
static void mt_scan_timeout_work(struct work_struct *work);
static int mt_scan_submit_next(struct mt7932 *m);
static int mt_scan_admit(struct mt7932 *m);
static int mt_set_bss(struct mt7932 *m, bool active);
static int mt_net_stop(struct net_device *netdev);
static void mt_cal_work(struct work_struct *work);

/* Standard cfg80211 scan and station interface. */
struct mt7932 *mt_from_wiphy(struct wiphy *wiphy)
{
	return *(struct mt7932 **)wiphy_priv(wiphy);
}

static void mt_scan_finish_work(struct work_struct *work)
{
	struct mt7932 *m = container_of(work, struct mt7932, scan_finish_work);
	struct cfg80211_scan_request *request;
	struct cfg80211_scan_info info = {};
	unsigned long flags;
	u8 cancel[4] = {};
	int ret;

	/* Retire the old timer before allowing another request to take ownership. */
	cancel_delayed_work_sync(&m->scan_timeout_work);
	mutex_lock(&m->command_mutex);
	spin_lock_irqsave(&m->response_lock, flags);
	request = m->scan_request;
	info.aborted = m->scan_aborted || m->reg_pending || m->stopping;
	cancel[0] = m->scan_seq;
	spin_unlock_irqrestore(&m->response_lock, flags);
	if (request && !info.aborted) {
		ret = mt_scan_submit_next(m);
		if (!ret) {
			mutex_unlock(&m->command_mutex);
			return;
		}
		if (ret < 0)
			info.aborted = true;
	}
	cancel_delayed_work(&m->scan_timeout_work);
	spin_lock_irqsave(&m->response_lock, flags);
	/* A terminal failure may have followed the worker's earlier snapshot. */
	info.aborted |= m->scan_aborted || !mt_rf_allowed(m);
	cancel[0] = m->scan_seq;
	if (request && info.aborted && !m->scan_finished)
		m->retired_scan_seq = m->scan_seq;
	m->scan_request = NULL;
	spin_unlock_irqrestore(&m->response_lock, flags);
	if (request && info.aborted && !READ_ONCE(m->stopping))
		mt_transport_snapshot(m);
	if (request && info.aborted && !READ_ONCE(m->stopping) && READ_ONCE(m->retired_scan_seq)) {
		ret = mt_request(m, 0x1b, true, true, false, cancel, sizeof(cancel));
		if (ret) {
			WRITE_ONCE(m->link_failed, true);
			dev_err(&m->pdev->dev, "SCAN_RETIREMENT_FAILED: %d; recovery required\n", ret);
		}
	}
	mutex_unlock(&m->command_mutex);
	if (request) {
		dev_dbg(&m->pdev->dev, "SCAN_FINISHED: seq=%u aborted=%u received-BSS=%llu\n",
			cancel[0], info.aborted, m->beacons);
		cfg80211_scan_done(request, &info);
	}
}

static void mt_scan_timeout_work(struct work_struct *work)
{
	struct mt7932 *m = container_of(to_delayed_work(work), struct mt7932, scan_timeout_work);
	unsigned long flags;

	spin_lock_irqsave(&m->response_lock, flags);
	if (!m->stopping && m->scan_request && !m->scan_finished) {
		m->scan_aborted = true;
		schedule_work(&m->scan_finish_work);
	}
	spin_unlock_irqrestore(&m->response_lock, flags);
}

/* Called under response_lock; scan payload sequence differs from MCU sequence. */
void mt_scan_event(struct mt7932 *m, const struct mt7932_event *event)
{
	if (m->stopping)
		return;
	/* Abort completes userspace locally, not by an invented cancel ACK.
	 * Keep the sequence reserved until the old scan's own done event arrives.
	 */
	if (event->length >= 37 && m->retired_scan_seq &&
	    event->packet[36] == m->retired_scan_seq) {
		m->retired_scan_seq = 0;
		complete(&m->scan_done);
		if (m->reg_pending) {
			m->reg_generation++;
			schedule_work(&m->startup_work);
		}
		return;
	}
	if (event->length >= 37 && m->discovering && event->packet[36] == m->scan_seq) {
		m->discovery_finished = true;
		complete(&m->discovery_done);
		return;
	}
	if (event->length < 37 || !m->scan_request || event->packet[36] != m->scan_seq)
		return;
	m->scan_finished = true;
	complete(&m->scan_done);
	schedule_work(&m->scan_finish_work);
}

void mt_abort_scan(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	struct mt7932 *m = mt_from_wiphy(wiphy);
	unsigned long flags;

	spin_lock_irqsave(&m->response_lock, flags);
	if (!m->stopping && m->scan_request && wdev == &m->wdev) {
		m->scan_aborted = true;
		schedule_work(&m->scan_finish_work);
	}
	spin_unlock_irqrestore(&m->response_lock, flags);
}

/* Wait at most @timeout in total for a cancelled scan to retire. Callers
 * may hold RTNL, so this never waits longer than asked. A stopping driver
 * completes scan_done for good, which would turn the wait into a spin.
 */
static void mt_scan_retire_wait(struct mt7932 *m, unsigned long timeout)
{
	unsigned long deadline = jiffies + timeout;
	long left;

	mt_abort_scan(m->wiphy, &m->wdev);
	flush_work(&m->scan_finish_work);
	while (READ_ONCE(m->retired_scan_seq) && !READ_ONCE(m->stopping)) {
		left = (long)(deadline - jiffies);
		if (left <= 0 || !wait_for_completion_timeout(&m->scan_done, left))
			break;
	}
}

/* No command/response lock held. The caller closes new scan admission first.
 * Natural EID0d completion fences the scan; a local cancel alone does not.
 * The batch awaited here may already have been followed by the next one,
 * which the abort then cancels: give the firmware time to end that one too,
 * rather than declaring the link unrecoverable while its cancel is in flight.
 */
int mt_scan_quiesce(struct mt7932 *m)
{
	if (READ_ONCE(m->scan_request) || READ_ONCE(m->retired_scan_seq))
		wait_for_completion_timeout(&m->scan_done, msecs_to_jiffies(5000));
	mt_scan_retire_wait(m, msecs_to_jiffies(2000));
	return READ_ONCE(m->retired_scan_seq) ? -EBUSY : 0;
}

/* command_mutex held: one RF-band/type batch at a time, with matching EID0d
 * completion before the next batch. A request without SSIDs is passive;
 * active requests must also listen passively on NO_IR channels.
 */
static int mt_scan_submit_next(struct mt7932 *m)
{
	struct cfg80211_scan_request *request = m->scan_request;
	u8 body[0x4d4], channels[MT7932_CHANNELS_5G];
	unsigned int count = 0, i;
	unsigned long flags;
	bool passive = false;
	int ret;

	/* 2GHz active/passive, then 5GHz active/passive. Never put a NO_IR
	 * channel in a firmware request which can emit probe requests.
	 */
	while (!count && ++m->scan_batch <= 4) {
		m->scan_band = (m->scan_batch + 1) / 2;
		passive = !(m->scan_batch & 1);
		for (i = 0; i < request->n_channels; i++) {
			struct ieee80211_channel *channel = request->channels[i];
			bool radar = channel->flags & IEEE80211_CHAN_RADAR;
			bool listen = !request->n_ssids || radar ||
				      (channel->flags & IEEE80211_CHAN_NO_IR);

			/* While associated, a passive dwell on a radar channel
			 * only delays the home channel.
			 */
			if (mt7932_channel_band(channel->hw_value) != m->scan_band ||
			    listen != passive || (radar && m->scan_home_channel))
				continue;
			if (channel->flags & (IEEE80211_CHAN_DISABLED | IEEE80211_CHAN_NO_20MHZ) ||
			    count == ARRAY_SIZE(channels))
				return -EINVAL;
			channels[count++] = channel->hw_value;
		}
	}
	if (!count)
		return 1;
	spin_lock_irqsave(&m->response_lock, flags);
	if (m->scan_aborted || !mt_rf_allowed(m) || m->disconnecting ||
	    (m->scan_home_channel && !m->connected)) {
		spin_unlock_irqrestore(&m->response_lock, flags);
		return -ECANCELED;
	}
	m->scan_seq = m->scan_seq % 127 + 1;
	m->scan_finished = false;
	reinit_completion(&m->scan_done);
	spin_unlock_irqrestore(&m->response_lock, flags);
	ret = mt7932_scan_body(body, sizeof(body), m->scan_seq, channels, count);
	if (ret >= 0 && passive) {
		if (m->scan_home_channel)
			mt7932_scan_passive(body);
		else
			mt7932_scan_disconnected_passive(body);
	}
	if (ret >= 0)
		ret = mt7932_scan_home(body, m->scan_home_channel);
	if (ret >= 0)
		ret = mt7932_scan_options(body, NULL, 0, request->ie, request->ie_len);
	for (i = 0; ret >= 0 && !passive && i < request->n_ssids; i++)
		ret = mt7932_scan_ssid_slot(body, request->ssids[i].ssid,
					    request->ssids[i].ssid_len);
	if (ret >= 0) {
		schedule_delayed_work(&m->scan_timeout_work, msecs_to_jiffies(60000));
		ret = mt_request(m, 3, true, true, false, body, sizeof(body));
	}
	if (!ret)
		dev_dbg(&m->pdev->dev, "SCAN_SUBMITTED: seq=%u band=%u channels=%u passive=%u\n",
			m->scan_seq, m->scan_band, count, passive);
	return ret;
}

/* Internal, disconnected discovery shares the firmware scan owner, but never
 * invents a cfg80211_scan_request or completes a userspace scan. The connect
 * epoch is already admitted, so external scans/connects remain excluded.
 * command_mutex is held on entry/exit; RX can run during all bounded waits.
 */
int mt_connect_discover(struct mt7932 *m)
{
	u8 body[0x4d4], channels[MT7932_CHANNELS_5G], cancel[4] = {};
	unsigned long flags;
	unsigned int attempt, band, i, count;
	int ret = 0;

	/* A short directed probe can miss a just-restarted AP. One bounded
	 * passive fallback obtains real beacons, without treating a stale
	 * userspace candidate as evidence or relaxing association admission.
	 */
	for (attempt = 0; attempt < 4; attempt++) {
		struct ieee80211_supported_band *sband;
		bool passive = attempt >= 2;

		band = 1 + attempt % 2;
		sband = band == 1 ? &m->band2 : &m->band5;
		count = 0;
		for (i = 0; i < sband->n_channels; i++) {
			struct ieee80211_channel *channel = &sband->channels[i];

			if (m->connect_channel_req && channel != m->connect_channel_req)
				continue;
			if (channel->flags & (IEEE80211_CHAN_DISABLED | IEEE80211_CHAN_NO_20MHZ))
				continue;
			/* Probes are forbidden on no-IR and radar channels; leave those
			 * to the passive batches so the rest of the band keeps probing.
			 */
			if (!passive && (channel->flags & (IEEE80211_CHAN_NO_IR | IEEE80211_CHAN_RADAR)))
				continue;
			channels[count++] = channel->hw_value;
		}
		if (!count)
			continue;
		spin_lock_irqsave(&m->response_lock, flags);
		if (m->connect_cancelled || m->connect_error || !mt_rf_allowed(m)) {
			spin_unlock_irqrestore(&m->response_lock, flags);
			return -ECANCELED;
		}
		reinit_completion(&m->discovery_done);
		m->scan_seq = m->scan_seq % 127 + 1;
		m->discovering = true;
		m->discovery_finished = false;
		spin_unlock_irqrestore(&m->response_lock, flags);
		ret = mt7932_scan_body(body, sizeof(body), m->scan_seq, channels, count);
		if (ret >= 0) {
			/* NO_IR forbids probes, not listening. A missing candidate in
			 * the world domain still needs discovery; the connect worker
			 * independently rechecks permission before association.
			 */
			if (passive)
				mt7932_scan_disconnected_passive(body);
			ret = mt7932_scan_options(body, passive ? NULL : m->connect_ssid,
						  passive ? 0 : m->connect_ssid_length, NULL, 0);
		}
		if (ret >= 0)
			ret = mt_request(m, 3, true, true, false, body, sizeof(body));
		dev_dbg(&m->pdev->dev, "CONNECT_DISCOVERY_SCAN: seq=%u band=%u channels=%u passive=%u status=%d\n",
			m->scan_seq, band, count, passive, ret);
		mutex_unlock(&m->command_mutex);
		/* A passive dwell is about 220 ms per channel; never wait less than 5 s. */
		if (!ret && !wait_for_completion_timeout(&m->discovery_done,
							 msecs_to_jiffies(passive ?
								max(5000U, 250 * count + 1000) : 5000)))
			ret = -ETIMEDOUT;
		mutex_lock(&m->command_mutex);
		/* Cancellation wakes the waiter, but is not a firmware completion.
		 * Keep ownership closed unless the matching terminal event arrives.
		 */
		if (!READ_ONCE(m->discovery_finished)) {
			cancel[0] = m->scan_seq;
			reinit_completion(&m->discovery_done);
			mt_request(m, 0x1b, true, true, false, cancel, sizeof(cancel));
			mutex_unlock(&m->command_mutex);
			wait_for_completion_timeout(&m->discovery_done, msecs_to_jiffies(1000));
			mutex_lock(&m->command_mutex);
		}
		spin_lock_irqsave(&m->response_lock, flags);
		if (!m->discovery_finished) {
			m->link_failed = true;
			ret = ret ?: -ETIMEDOUT;
		}
		m->discovering = false;
		if (m->connect_cancelled)
			ret = -ECANCELED;
		spin_unlock_irqrestore(&m->response_lock, flags);
		if (ret)
			return ret;
		m->connect_bss = mt_connect_find_bss(m);
		if (m->connect_bss)
			return 0;
	}
	return -ENOENT;
}

/* response_lock held. EBUSY means a real scan can later complete, not merely
 * that RF is unavailable. iwd waits for that completion before retrying.
 */
static int mt_scan_admit(struct mt7932 *m)
{
	if (m->link_failed || m->retired_scan_seq)
		return -EIO;
	if (!mt_rf_allowed(m) || !m->bss_active || m->connecting || m->disconnecting)
		return -EAGAIN;
	if (m->scan_request)
		return -EBUSY;
	return 0;
}

int mt_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request)
{
	struct mt7932 *m = mt_from_wiphy(wiphy);
	unsigned long flags;
	unsigned int i;
	int ret;

	if (request->wdev != &m->wdev)
		return -ENODEV;
	mt_retry_missing_policy(m);
	dev_dbg(&m->pdev->dev,
		"SCAN_REQUEST: channels=%u ssids=%d ies=%zu flags=%08x duration=%u/%u\n",
		request->n_channels, request->n_ssids, request->ie_len, request->flags,
		request->duration, request->duration_mandatory);
	/* cfg80211 lists each advertised channel at most once. */
	if (request->n_channels > MT7932_CHANNELS)
		return -EOPNOTSUPP;
	if (!request->n_channels || request->ie_len > 600 ||
	    request->duration_mandatory ||
	    (request->flags & ~(NL80211_SCAN_FLAG_FLUSH | NL80211_SCAN_FLAG_COLOCATED_6GHZ)) ||
	    request->n_ssids > MT7932_SCAN_SSIDS)
		return -EOPNOTSUPP;
	for (i = 0; i < request->n_ssids; i++)
		if (request->ssids[i].ssid_len > 32)
			return -EOPNOTSUPP;
	for (i = 0; i < request->n_channels; i++) {
		struct ieee80211_channel *channel = request->channels[i];

		if (!mt7932_channel_band(channel->hw_value) ||
		    (channel->band != NL80211_BAND_2GHZ &&
		     channel->band != NL80211_BAND_5GHZ) ||
		    channel->flags & (IEEE80211_CHAN_DISABLED | IEEE80211_CHAN_NO_20MHZ))
			return -EINVAL;
	}
	mutex_lock(&m->command_mutex);
	spin_lock_irqsave(&m->response_lock, flags);
	ret = mt_scan_admit(m);
	if (ret) {
		spin_unlock_irqrestore(&m->response_lock, flags);
		goto out;
	}
	m->scan_band = 0;
	m->scan_batch = 0;
	m->scan_home_channel = m->connected ? m->connect_channel : 0;
	m->scan_request = request;
	m->scan_finished = m->scan_aborted = false;
	spin_unlock_irqrestore(&m->response_lock, flags);
	ret = mt_scan_submit_next(m);
	if (ret < 0) {
		spin_lock_irqsave(&m->response_lock, flags);
		m->scan_request = NULL;
		spin_unlock_irqrestore(&m->response_lock, flags);
		cancel_delayed_work(&m->scan_timeout_work);
	} else {
		/* Nothing eligible to submit: complete the request. */
		if (ret)
			schedule_work(&m->scan_finish_work);
		ret = 0;
	}
out:
	mutex_unlock(&m->command_mutex);
	return ret;
}

/* command_mutex held; host interface existence is independent of RF policy. */
static int mt_set_bss(struct mt7932 *m, bool active)
{
	u8 body[12] = {0,1,0,0,0,0,0,0,0,0,0xff,1};
	int ret;

	body[1] = active;
	if (active)
		memcpy(body + 4, m->mac, 6);
	ret = mt_request(m, 0x11, true, true, false, body, sizeof(body));
	if (!ret)
		m->bss_active = active;
	return ret;
}

int mt_net_open(struct net_device *netdev)
{
	struct mt7932 *m = *(struct mt7932 **)netdev_priv(netdev);
	unsigned long flags;
	int ret = 0;

	mutex_lock(&m->command_mutex);
	mt_retry_missing_policy(m);
	spin_lock_irqsave(&m->response_lock, flags);
	if (m->stopping)
		ret = -ESHUTDOWN;
	else if (m->link_failed || m->cal_state.error)
		ret = m->cal_state.error ?: -EIO;
	spin_unlock_irqrestore(&m->response_lock, flags);
	if (ret) {
		mutex_unlock(&m->command_mutex);
		return ret;
	}
	if (READ_ONCE(m->rf_ready))
		ret = mt_set_bss(m, true);
	spin_lock_irqsave(&m->response_lock, flags);
	if (!ret && (m->stopping || m->link_failed || m->cal_state.error))
		ret = m->cal_state.error ?: (m->stopping ? -ESHUTDOWN : -EIO);
	if (!ret)
		WRITE_ONCE(m->interface_up, true);
	spin_unlock_irqrestore(&m->response_lock, flags);
	mutex_unlock(&m->command_mutex);
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);
	return ret;
}

static int mt_net_stop(struct net_device *netdev)
{
	struct mt7932 *m = *(struct mt7932 **)netdev_priv(netdev);
	WRITE_ONCE(m->interface_up, false);
	if (READ_ONCE(m->stopping)) {
		netif_carrier_off(netdev);
		netif_stop_queue(netdev);
		return 0;
	}

	mt_disconnect(m->wiphy, netdev, WLAN_REASON_DEAUTH_LEAVING);
	flush_work(&m->connect_work);
	flush_work(&m->disconnect_work);
	mt_abort_scan(m->wiphy, &m->wdev);
	flush_work(&m->scan_finish_work);
	mutex_lock(&m->command_mutex);
	if (m->bss_active)
		mt_set_bss(m, false);
	mutex_unlock(&m->command_mutex);
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);
	return 0;
}

static int mt_set_mac_address(struct net_device *netdev, void *address)
{
	struct mt7932 *m = *(struct mt7932 **)netdev_priv(netdev);
	unsigned long flags;
	int ret;

	/* No live-MAC change: the next BSS activation and connect commands use
	 * this address together. A failed retirement must not change identity.
	 * NetworkManager takes the interface down before changing the address,
	 * which cancels any scan; let that scan retire instead of failing.
	 */
	if (!netif_running(netdev))
		mt_scan_retire_wait(m, msecs_to_jiffies(2000));
	mutex_lock(&m->command_mutex);
	spin_lock_irqsave(&m->response_lock, flags);
	if (m->stopping || m->link_failed || m->bss_active || m->connecting ||
	    m->connected || m->disconnecting || m->peer_valid ||
	    m->scan_request || m->retired_scan_seq || m->discovering) {
		dev_dbg(&m->pdev->dev,
			"address change refused: bss=%u connecting=%u connected=%u peer=%u scan=%u retired=%u\n",
			m->bss_active, m->connecting, m->connected, m->peer_valid,
			!!m->scan_request, m->retired_scan_seq);
		ret = -EBUSY;
	} else {
		ret = eth_mac_addr(netdev, address);
		if (!ret)
			ether_addr_copy(m->mac, netdev->dev_addr);
	}
	spin_unlock_irqrestore(&m->response_lock, flags);
	mutex_unlock(&m->command_mutex);
	return ret;
}

static const struct net_device_ops mt_net_ops = {
	.ndo_open = mt_net_open, .ndo_stop = mt_net_stop, .ndo_start_xmit = mt_net_xmit,
	.ndo_set_mac_address = mt_set_mac_address,
};

static void mt_cal_work(struct work_struct *work)
{
	struct mt7932 *m = container_of(work, struct mt7932, cal_work);
	const struct firmware *oca;
	struct mt7932_cal_piece pieces[7];
	u8 request[16];
	unsigned long flags;
	unsigned int count, input_tag;
	int ret;

	mutex_lock(&m->command_mutex);
	if (!READ_ONCE(m->rf_ready))
		goto unlock;
	ret = mt_request_input(m, &oca, "mediatek/mt7932/oca2.bin");
	if (ret)
		goto fail;
	for (;;) {
		spin_lock_irqsave(&m->response_lock, flags);
		count = m->cal_request_count;
		if (count) {
			memcpy(request, m->cal_requests[m->cal_request_head], 16);
			m->cal_request_head = (m->cal_request_head + 1) % ARRAY_SIZE(m->cal_requests);
			m->cal_request_count--;
		}
		spin_unlock_irqrestore(&m->response_lock, flags);
		if (!count)
			break;
		ret = mt7932_cal_requested(oca->data, oca->size, request, sizeof(request),
					  m->smart_version, m->module_byte, pieces, &input_tag);
		if (ret < 0) {
			mt_cal_input_error(m, ret, input_tag, "runtime D7 plan");
			break;
		}
		ret = mt_cal_procedure(m, pieces, ret,
				       mt7932_cal_request_replies(request),
				       m->preload_version);
		if (ret)
			break;
		dev_dbg(&m->pdev->dev, "D7_CALIBRATION_COMPLETE: channel=%u\n",
			get_unaligned_le32(request + 12));
	}
	release_firmware(oca);
	if (!ret)
		goto unlock;
fail:
	spin_lock_irqsave(&m->response_lock, flags);
	mt_rf_fail_locked(m, ret);
	spin_unlock_irqrestore(&m->response_lock, flags);
	dev_err(&m->pdev->dev, "D7_CALIBRATION_FAILED: %d; mediatek/mt7932/oca2.bin RF operations stopped\n", ret);
unlock:
	mutex_unlock(&m->command_mutex);
}

int mt_register_interface(struct mt7932 *m)
{
	struct net_device *netdev;
	int ret;

	netdev = alloc_etherdev(sizeof(m));
	if (!netdev)
		return -ENOMEM;
	*(struct mt7932 **)netdev_priv(netdev) = m;
	m->wdev.wiphy = m->wiphy;
	m->wdev.iftype = NL80211_IFTYPE_STATION;
	m->wdev.netdev = netdev;
	netdev->ieee80211_ptr = &m->wdev;
	netdev->netdev_ops = &mt_net_ops;
	netdev->ethtool_ops = &mt_ethtool_ops;
	netdev->min_mtu = ETH_MIN_MTU;
	netdev->max_mtu = ETH_DATA_LEN;
	SET_NETDEV_DEV(netdev, &m->pdev->dev);
	eth_hw_addr_set(netdev, m->mac);
	ether_addr_copy(netdev->perm_addr, m->wiphy->perm_addr);
	strscpy(netdev->name, "wlan%d", IFNAMSIZ);
	netif_carrier_off(netdev);
	INIT_WORK(&m->scan_finish_work, mt_scan_finish_work);
	INIT_WORK(&m->cal_work, mt_cal_work);
	INIT_WORK(&m->connect_work, mt_connect_work);
	INIT_WORK(&m->disconnect_work, mt_disconnect_work);
	INIT_WORK(&m->power_work, mt_power_work);
	init_completion(&m->assoc_start);
	init_completion(&m->assoc_done);
	init_completion(&m->disconnect_done);
	init_completion(&m->discovery_done);
	init_completion(&m->scan_done);
	m->peer_alloc = BIT(0) | BIT(19);
	INIT_DELAYED_WORK(&m->scan_timeout_work, mt_scan_timeout_work);
	m->netdev = netdev;
	ret = register_netdev(netdev);
	if (ret) {
		m->netdev = NULL;
		free_netdev(netdev);
		return ret;
	}
	dev_dbg(&m->pdev->dev, "CFG80211_INTERFACE_READY: %s, RF awaits country policy\n",
		netdev->name);
	return 0;
}

int mt_enable_scan(struct mt7932 *m)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&m->response_lock, flags);
	ret = m->cal_state.error ?: ((m->stopping || m->link_failed) ? -EIO : 0);
	spin_unlock_irqrestore(&m->response_lock, flags);
	if (ret)
		return ret;
	ret = mt_data_prepare(m);

	if (ret)
		return ret;
	spin_lock_irqsave(&m->response_lock, flags);
	ret = m->cal_state.error ?: ((m->stopping || m->link_failed) ? -EIO : 0);
	spin_unlock_irqrestore(&m->response_lock, flags);
	if (ret)
		return ret;
	if (READ_ONCE(m->interface_up)) {
		ret = mt_set_bss(m, true);
		if (ret)
			return ret;
	}
	/* An RX calibration failure must win over late startup publication. */
	spin_lock_irqsave(&m->response_lock, flags);
	if (m->stopping || m->link_failed || m->cal_state.error) {
		ret = m->cal_state.error ?: -EIO;
		spin_unlock_irqrestore(&m->response_lock, flags);
		return ret;
	}
	WRITE_ONCE(m->rf_ready, true);
	dev_dbg(&m->pdev->dev, "CFG80211_SCAN_READY: %s, carrier off until WPA2 authorization\n",
		m->netdev->name);
	schedule_work(&m->cal_work);
	spin_unlock_irqrestore(&m->response_lock, flags);
	return 0;
}

/* A station on a radar channel must stop transmitting when its AP announces a
 * switch away; the firmware's CSA handling is unknown. The beacon's IEs are
 * already bounds-checked by mt7932_rx_beacon().
 */
static bool mt_radar_switch_announced(const struct ieee80211_channel *channel,
				      const struct mt7932_rx_frame *frame)
{
	const u8 *ies = frame->data + 36;
	size_t length = frame->length - 36;
	const struct element *csa, *ecsa;

	if (!(channel->flags & IEEE80211_CHAN_RADAR))
		return false;
	csa = cfg80211_find_elem(WLAN_EID_CHANNEL_SWITCH, ies, length);
	ecsa = cfg80211_find_elem(WLAN_EID_EXT_CHANSWITCH_ANN, ies, length);
	return (csa && csa->datalen >= 3) || (ecsa && ecsa->datalen >= 4);
}

void mt_packet_receive(struct mt7932 *m, const u8 *packet, size_t length)
{
	struct mt7932_rx_frame frame;
	struct cfg80211_bss *bss;
	struct ieee80211_channel *channel;
	unsigned int band;
	int ret = mt7932_rx_frame(packet, length, &frame);

	if (ret) {
		unsigned long flags;

		spin_lock_irqsave(&m->response_lock, flags);
		if (ret == -EBADMSG)
			m->rx_integrity++;
		else if (ret == -EOPNOTSUPP)
			m->rx_unsupported++;
		else if (ret != -ENOMSG)
			m->rx_malformed++;
		spin_unlock_irqrestore(&m->response_lock, flags);
		return;
	}
	if (!READ_ONCE(m->rf_ready))
		return;
	band = mt7932_channel_band(frame.channel);
	channel = ieee80211_get_channel(m->wiphy, band == 1 ? 2407 + 5 * frame.channel : 5000 + 5 * frame.channel);
	if (!channel || (channel->flags & IEEE80211_CHAN_DISABLED))
		return;
	if (frame.translated) {
		mt_data_receive(m, &frame);
		return;
	}
	if (!frame.signal_valid || !mt7932_rx_beacon(&frame))
		return;
	if ((frame.frame_control & 0xfc) == IEEE80211_STYPE_BEACON) {
		const struct element *tim = cfg80211_find_elem(WLAN_EID_TIM,
				frame.data + 36, frame.length - 36);
		unsigned long flags;

		spin_lock_irqsave(&m->response_lock, flags);
		if (m->connecting && !m->disconnecting && !m->stopping && !m->connect_error &&
		    frame.channel == m->connect_channel &&
		    ether_addr_equal(frame.data + 16, m->connect_bssid) &&
		    mt_radar_switch_announced(channel, &frame)) {
			/* The connect worker owns teardown: fail its attempt and wake it. */
			m->connect_error = -ECONNRESET;
			complete(&m->assoc_start);
			complete(&m->assoc_done);
			complete(&m->discovery_done);
			dev_info(&m->pdev->dev, "RADAR_CHANNEL_SWITCH: AP announced a switch; abandoning the join on channel %u\n",
				 m->connect_channel);
		} else if (m->connected && !m->disconnecting && !m->stopping &&
		    frame.channel == m->connect_channel &&
		    ether_addr_equal(frame.data + 16, m->connect_bssid)) {
			mt_cqm_signal(m, mt7932_rx_signal(&frame, m->antenna_mask));
			if (mt_radar_switch_announced(channel, &frame)) {
				m->connected = false;
				m->disconnecting = true;
				m->disconnect_seen = false;
				m->disconnect_event_received = false;
				m->disconnect_local = true;
				m->disconnect_reason = WLAN_REASON_DISASSOC_STA_HAS_LEFT;
				netif_carrier_off(m->netdev);
				netif_stop_queue(m->netdev);
				reinit_completion(&m->disconnect_done);
				schedule_work(&m->disconnect_work);
				dev_info(&m->pdev->dev, "RADAR_CHANNEL_SWITCH: AP announced a switch; leaving channel %u\n",
					 m->connect_channel);
			} else if (tim && tim->datalen >= 4 && !m->power_tim) {
				m->power_tim = true;
				schedule_work(&m->power_work);
			}
		}
		spin_unlock_irqrestore(&m->response_lock, flags);
	}
	bss = cfg80211_inform_bss_frame(m->wiphy, channel,
				      (struct ieee80211_mgmt *)frame.data, frame.length,
				      mt7932_rx_signal(&frame, m->antenna_mask) * 100, GFP_ATOMIC);
	if (bss) {
		m->beacons++;
		dev_dbg_ratelimited(&m->pdev->dev, "BSS_RECEIVED: channel=%u BSSID=%pM\n", frame.channel, bss->bssid);
		cfg80211_put_bss(m->wiphy, bss);
	}
}
