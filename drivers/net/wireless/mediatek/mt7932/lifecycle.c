// SPDX-License-Identifier: GPL-2.0-only
#include "mt7932.h"

/* Join all host users before resetting hardware. No RTNL/wiphy mutex held:
 * network callbacks and workers may need command_mutex while being joined.
 * DMA storage and Linux objects remain owned until the hardware fence.
 */
void mt_stop_host(struct mt7932 *m)
{
	struct cfg80211_scan_request *scan;
	struct cfg80211_scan_info info = { .aborted = true };
	unsigned long flags;

	/* Serialize with command publication and callbacks that queue work. */
	mutex_lock(&m->command_mutex);
	spin_lock_irqsave(&m->response_lock, flags);
	WRITE_ONCE(m->stopping, true);
	WRITE_ONCE(m->rf_ready, false);
	m->connect_error = -ESHUTDOWN;
	m->cal_state.error = -ESHUTDOWN;
	complete_all(&m->response);
	complete_all(&m->cal_response);
	if (m->interface_registered) {
		complete_all(&m->assoc_start);
		complete_all(&m->assoc_done);
		complete_all(&m->disconnect_done);
		complete_all(&m->discovery_done);
		complete_all(&m->scan_done);
	}
	spin_unlock_irqrestore(&m->response_lock, flags);
	mutex_unlock(&m->command_mutex);
	if (m->netdev) {
		netif_carrier_off(m->netdev);
		netif_tx_disable(m->netdev);
	}
	WRITE_ONCE(m->running, false);
	mt_write(m, W + 0x204, 0);
	if (m->irq_requested) {
		devm_free_irq(&m->pdev->dev, pci_irq_vector(m->pdev, 0), m);
		m->irq_requested = false;
	}
	/* A service invocation already in flight may have re-enabled the mask. */
	mt_write(m, W + 0x204, 0);
	if (!m->interface_registered)
		return;
	cancel_work_sync(&m->startup_work);
	/* Let an admitted connect publish its one terminal cfg80211 result. */
	flush_work(&m->connect_work);
	flush_work(&m->disconnect_work);
	cancel_work_sync(&m->cal_work);
	cancel_work_sync(&m->power_work);
	cancel_delayed_work_sync(&m->scan_timeout_work);
	cancel_work_sync(&m->scan_finish_work);
	/* With all producers joined, this is the sole owner of any remainder. */
	scan = m->scan_request;
	m->scan_request = NULL;
	if (scan)
		cfg80211_scan_done(scan, &info);
	if (m->connect_bss) {
		cfg80211_put_bss(m->wiphy, m->connect_bss);
		m->connect_bss = NULL;
	}
	if (m->connected) {
		m->connected = false;
		mt_report_disconnected(m, WLAN_REASON_DEAUTH_LEAVING, true, GFP_KERNEL);
	}
	memzero_explicit(m->connect_pmk, sizeof(m->connect_pmk));
	memzero_explicit(m->connect_sae_pwd, sizeof(m->connect_sae_pwd));
	m->connect_sae_pwd_length = 0;
	m->connect_pmk_valid = false;
}
