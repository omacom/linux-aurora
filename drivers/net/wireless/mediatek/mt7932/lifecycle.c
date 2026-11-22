// SPDX-License-Identifier: GPL-2.0-only
#include <linux/debugfs.h>
#include <linux/soc/apple/pci-apple-piodma.h>

#include "mt7932.h"

/*
 * A restart removes the binding that asked for it, so the count of restarts and the
 * note that the next binding comes from one belong to the module: one radio per machine.
 */
static atomic_t mt_restarts = ATOMIC_INIT(0);
static atomic_t mt_restart_pending = ATOMIC_INIT(0);

/* A restart of the radio is being asked for: its number since the module loaded. */
int mt_note_restart(void)
{
	atomic_set(&mt_restart_pending, 1);
	return atomic_inc_return(&mt_restarts);
}

/* Probe finished: whether this binding is the one a restart brought up, which it says. */
bool mt_restart_done(struct mt7932 *m)
{
	if (!atomic_xchg(&mt_restart_pending, 0))
		return false;
	dev_info(&m->pdev->dev, "Wi-Fi is back after radio restart %d: firmware loaded again\n",
		 atomic_read(&mt_restarts));
	return true;
}

int mt_restart_count(void)
{
	return atomic_read(&mt_restarts);
}

void mt_restart_debugfs_init(struct mt7932 *m)
{
	debugfs_create_atomic_t("mt7932_restarts", 0400, m->wiphy->debugfsdir, &mt_restarts);
}

/*
 * The end of a connection or a scan that the firmware never confirmed leaves the
 * driver unsure what the firmware still owns, so it refuses every later scan and
 * connection. Only a reset settles that: restart the radio, as after a wake that
 * lost it, rather than leave Wi-Fi refusing everything until a reboot. Once per
 * binding, from any context, and neither before probe finished nor once stopping.
 */
void mt_request_recovery(struct mt7932 *m, const char *why)
{
	if (!READ_ONCE(m->ready) || READ_ONCE(m->stopping) || test_and_set_bit(0, &m->recovery))
		return;
	dev_warn(&m->pdev->dev, "%s, which only a reset settles: restarting the radio (restart %d)\n",
		 why, mt_note_restart());
	schedule_work(&m->recover_work);
}

void mt_recover_work(struct work_struct *work)
{
	apple_piodma_radio_recover();
}

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
#if IS_ENABLED(CONFIG_MT7932_AWDL)
	if (m->awdl.netdev) {
		netif_carrier_off(m->awdl.netdev);
		netif_tx_disable(m->awdl.netdev);
	}
#endif
	WRITE_ONCE(m->running, false);
	mt_write(m, W + 0x204, 0);
	if (m->irq_requested) {
		devm_free_irq(&m->pdev->dev, pci_irq_vector(m->pdev, 0), m);
		m->irq_requested = false;
	}
	/* A service invocation already in flight may have re-enabled the mask. */
	mt_write(m, W + 0x204, 0);
	if (!m->interface_registered) {
		cancel_work_sync(&m->recover_work);
		return;
	}
	cancel_work_sync(&m->startup_work);
	/* Let an admitted connect publish its one terminal cfg80211 result. */
	flush_work(&m->connect_work);
	flush_work(&m->disconnect_work);
	cancel_work_sync(&m->cal_work);
	cancel_work_sync(&m->power_work);
	cancel_work_sync(&m->wow_work);
	cancel_delayed_work_sync(&m->scan_timeout_work);
	cancel_work_sync(&m->scan_finish_work);
	/* After every worker that may ask for it. */
	cancel_work_sync(&m->recover_work);
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
