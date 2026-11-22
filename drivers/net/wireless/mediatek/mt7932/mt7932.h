/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MT7932_H
#define MT7932_H

#include <linux/average.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/pci.h>
#include <linux/random.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <net/cfg80211.h>
#include "firmware.h"
#include "calibration.h"
#include "packet.h"
#include "awdl.h"
#include "scan.h"
#include "association.h"
#include "regulatory.h"
#include "rate.h"
#include "capabilities.h"

#define W 0xd4000
#define H 0xd6000
#define C 0xd7000
#define IRQ_MASK 0xe2c8000d
#define DONE BIT(31)
#define STRIDE 8192
#define RX_STRIDE ALIGN(MT7932_RX_CAPACITY, 64)

DECLARE_EWMA(mt7932_signal, 4, 4)

struct mt7932;
int mt_dma_stop(struct mt7932 *m);
int mt_function_reset(struct mt7932 *m);
void mt_stop_host(struct mt7932 *m);

struct mt7932_desc {
	__le32 address, control, second, info;
};

struct mt7932_ring {
	struct mt7932_desc *desc;
	dma_addr_t dma, buffers_dma;
	u8 *buffers;
	u32 reg;
	u16 count, head, tail, queued;
	u16 slots, used;
};

struct mt7932 {
	struct pci_dev *pdev;
	void __iomem *bar;
	struct mt7932_ring tx[19], rx[9];
	struct completion response;
	struct completion cal_response;
	struct mt7932_cal_completion cal_state;
#if IS_ENABLED(CONFIG_MT7932_RF_KUNIT_TEST)
	/* Hardware-free calibration transport seam; production builds omit it. */
	int (*cal_test_send)(struct mt7932 *m, const void *body, size_t length);
	void *cal_test_context;
	int (*power_test_send)(struct mt7932 *m, const void *body, size_t length);
	void *power_test_context;
#endif
	u8 cal_requests[32][16];
	u8 cal_request_seq[32];
	unsigned int cal_request_head, cal_request_count;
	spinlock_t response_lock;
	u8 seq, waiting;
	/* Event that answers the waiting command; see mt7932_reply_eid(). */
	int waiting_eid;
	u8 mac[ETH_ALEN];
	u8 reply[MT7932_RX_CAPACITY];
	size_t reply_length;
	u64 interrupts, packets;
	bool running;
	bool stopping;
	/* Probe finished initialization; the Bluetooth function may start. */
	bool ready;
	/* Bit 0: a radio reset was asked for, once per binding. */
	unsigned long recovery;
	struct work_struct recover_work;
	bool irq_requested, vectors_allocated, wiphy_registered;
	bool dma_owned, module_pinned;
	struct completion reset_retry;
	int removal_error;
	/* The firmware keeps running across a deep sleep with its host interface stopped. */
	struct completion hif_done;
	int hif_result;
	u32 hif_glo;
	/* hif_suspended stops new work; hif_gate then closes the BAR to all but the control. */
	bool hif_waiting, hif_suspending, hif_suspended, hif_gate;
	bool ram_config;
	bool runtime_epoch;
	bool runtime_cal_valid;
	u8 runtime_cal, board_type, smart_version, preload_version;
	u8 module_byte;
	u8 six_ghz;
	bool six_ghz_valid;
	bool stof_supported;
	u8 phy_cap[12];
	u8 antenna_mask;
	struct wiphy *wiphy;
	struct ieee80211_supported_band band2;
	struct cfg80211_chan_def connect_chandef;
	u8 connect_center;
	struct ieee80211_channel channels[MT7932_CHANNELS_2G];
	struct ieee80211_channel channels5[MT7932_CHANNELS_5G];
	struct ieee80211_supported_band band5, band6;
	struct ieee80211_channel channels6[MT7932_CHANNELS_6G];
	struct ieee80211_sband_iftype_data he6;
	u8 six_assoc_power[2][1020];
	bool six_power_valid, six_legacy_assoc, six_ap_valid, six_class_dirty;
	u64 six_legacy_channels;
	struct mt7932_6g_ap six_ap;
	u8 scan_band, scan_batch;
	unsigned int scan_cursor;
	enum nl80211_band connect_band, scan_home_band;
	u32 connect_akm;
	enum nl80211_mfp connect_mfp;
	bool connect_pmf; /* admitted request, never firmware readback */
	enum nl80211_sae_pwe_mechanism connect_sae_pwe;
	struct ieee80211_rate rates[12];
	struct work_struct startup_work;
	struct mutex command_mutex;
	bool startup_started, rf_ready, reg_pending, policy_failed, reg_retryable;
	bool interface_registered, interface_up;
	u32 reg_generation, reg_attempted;
	struct mt7932_reg_snapshot reg_desired;
	/* A relaxation kept from the firmware while associated; applied by the
	 * next pass once the connection has ended. Under response_lock.
	 */
	struct mt7932_reg_snapshot reg_deferred;
	bool reg_deferral;
	/* Channels the applied country package leaves out; under RTNL. */
	DECLARE_BITMAP(policy_disabled, MT7932_CHANNELS);
	/* Last package pass for the current request; under response_lock. */
	struct mt7932_policy_choice policy_choice;
	struct net_device *netdev;
	struct wireless_dev wdev;
	struct cfg80211_scan_request *scan_request;
	struct work_struct scan_finish_work;
	struct work_struct cal_work;
	struct delayed_work scan_timeout_work;
	u8 scan_seq, retired_scan_seq, scan_home_channel;
	bool scan_finished, scan_aborted, bss_active;
	u64 beacons;
	/* Station and security-table identities are independent host allocators. */
	unsigned long station_alloc, wtbl_alloc, wmm_alloc;
	struct mt7932_peer_ids peer_ids;
	struct mt7932_awdl awdl;
	struct cfg80211_bss *connect_bss;
	struct work_struct connect_work, disconnect_work;
	struct work_struct power_work;
	/* Desired/submitted PS configuration uses command_mutex; TIM uses response_lock. */
	u8 power_mode, power_applied;
	bool power_applied_valid, power_tim;
	/* WoWLAN detectors cfg80211 asked for at this sleep; 0 leaves the network. */
	u8 wow_detect;
	/* The firmware was given WoW at the sleep; command_mutex. */
	bool wow_armed;
	/* A wake reason is awaited, and the one received; response_lock. */
	bool wow_report;
	u8 wow_reason;
	struct work_struct wow_work;
	struct completion assoc_start, assoc_done, disconnect_done;
	struct completion discovery_done;
	struct completion scan_done;
	struct ieee80211_channel *connect_channel_req, *connect_channel_hint;
	/* Boot time after which a radar or no-IR candidate must have been
	 * heard; set when an attempt starts listening, under command_mutex.
	 */
	u64 connect_heard_since;
	u8 connect_bssid_req[ETH_ALEN], connect_bssid_hint[ETH_ALEN];
	bool discovering, discovery_finished, connect_cancelled;
	u8 connect_ssid[32], connect_pmk[32], connect_bssid[6];
	u8 connect_sae_pwd[MT7932_SAE_PASSWORD_MAX], connect_sae_pwd_length;
	bool connect_pmk_valid;
	u8 assoc_request_ies[512];
	u16 assoc_request_ie_len;
	bool assoc_request_seen;
	u32 connection_generation;
	u8 connect_ssid_length, connect_channel, peer_wtbl;
	bool connecting, connected, peer_valid, link_failed, connect_open, connect_owe;
	bool owe_cold_prepared;
	bool disconnecting, disconnect_seen, disconnect_event_received, disconnect_local;
	/* The connect worker is reporting its outcome; an RF failure meanwhile
	 * sets connect_lost for the worker to report. Under response_lock.
	 */
	bool connect_publishing, connect_lost;
	/* A connect with the previous BSSID is moving the connection to another
	 * AP (roam_moving), or the current connection is the result of one
	 * (roam_landed). Written under response_lock with WRITE_ONCE(); read
	 * under it, or with READ_ONCE() where the lock is not held.
	 */
	bool roam_moving, roam_landed;
	u16 disconnect_reason;
	int connect_error;
	u32 event_cipher;
	spinlock_t data_lock;
	/* Per-association host Ethernet accounting, protected by data_lock. */
	u64 station_tx_bytes, station_rx_bytes;
	u32 station_tx_packets, station_rx_packets;
	unsigned long station_connected;
	s8 station_signal;
	bool station_signal_valid;
	struct rate_info station_rx_rate;
	unsigned long station_rx_rate_time;
	bool station_rx_rate_valid;
	/* The last frame from the AP, once station_rx_packets counts one. */
	unsigned long station_rx_time;
	/* The associated AP's beacon interval, in TU. */
	u16 station_beacon_tu;
	/* Connection quality monitor, protected by response_lock. */
	struct ewma_mt7932_signal cqm_signal;
	unsigned long cqm_sample_time;
	s32 cqm_threshold, cqm_last_event;
	u32 cqm_hysteresis;
	u8 cqm_samples;
	u8 *data_headers, *data_payloads;
	dma_addr_t data_headers_dma, data_payloads_dma;
	bool data_live[512], data_token[512], data_retired[512];
	struct mt7932_duplicate duplicates[17];
	u64 tx_released, rx_ethernet;
	/* Lifetime host RX observations, protected by response_lock. */
	u64 rx_integrity, rx_malformed, rx_unsupported;
	u64 rx_wrong_peer, rx_unprotected, rx_duplicate, rx_no_memory;
	u64 rx_retry, rx_amsdu;
	/* Lifetime transport observations for ethtool. The ring and cause
	 * counters are written only by the interrupt thread; the BSS presence
	 * counters are under response_lock, the data counters under data_lock.
	 */
	u64 irq_passes, irq_unmasked_passes, rx_ring_records[9];
	u32 irq_unmasked;
	u64 tx_free_events, bss_absent_events, bss_absent_ms;
	unsigned long bss_absent_since;
	u64 home_beacons, rx_band_records[3];
	/* TX0 descriptors published since the last doorbell; data_lock. */
	unsigned int data_unkicked;
	bool data_ready;
	bool bss_absent;
	u8 bss_quota;
	u32 irq_mask;
	void *ipc, *aux;
	dma_addr_t ipc_dma, aux_dma;
};

/* What the connect worker reports once an attempt has its result. A move, a
 * connect with the previous BSSID while connected, ends in a connect result,
 * which is what iwd's reassociation waits for; cfg80211_roamed() is never
 * used. A failed connect result leaves cfg80211 holding the connection the
 * move replaced, so a move that fails or is cancelled then reports a
 * disconnect, for the reason the cancel gave if it was cancelled. A
 * cancelled connect that was not a move reports only the disconnect. A
 * cancel always leaves an error in ret.
 */
struct mt7932_connect_report {
	bool result;		/* cfg80211_connect_done() */
	bool disconnect;	/* then a disconnect, for reason */
	bool landed;		/* the new connection is the result of a move */
	u16 reason;
};

static inline struct mt7932_connect_report
mt7932_connect_report(bool move, bool cancelled, int ret, u16 cancel_reason)
{
	struct mt7932_connect_report report = {
		.result = move || !cancelled,
		.disconnect = move ? ret != 0 : cancelled,
		.landed = move && !ret,
	};

	if (report.disconnect)
		report.reason = cancelled ? cancel_reason : WLAN_REASON_DEAUTH_LEAVING;
	return report;
}

/* Whether a disconnect the driver ends is reported as local. See
 * mt_report_disconnected() for why a connection a move made reports it as
 * from the AP.
 */
static inline bool mt7932_disconnect_local(bool local, bool landed)
{
	return local && !landed;
}

/*
 * The firmware lost a BSS's AP: its beacons, or the answers to the frames it
 * sends to keep the link. It leaves ending the connection to the host. The body
 * is the BSS index, the firmware's cause and a le16 deauthentication reason.
 */
#define MT7932_EID_BEACON_TIMEOUT		0x13
#define MT7932_BEACON_TIMEOUT_LENGTH		4
/*
 * A frame from the AP shows it is still there, if it came within the span in
 * which a missing beacon would count as lost: as many beacon intervals as
 * mac80211's own beacon-loss monitor allows (IEEE80211_BEACON_LOSS_COUNT in
 * net/mac80211/mlme.c).
 */
#define MT7932_BEACON_LOSS_COUNT		7

/* That span in ms, for a beacon interval in TU (1024 us); 0 if it isn't known. */
static inline unsigned int mt7932_beacon_loss_ms(u16 beacon_tu)
{
	return DIV_ROUND_UP(beacon_tu * 1024U * MT7932_BEACON_LOSS_COUNT, 1000);
}

bool mt_rf_allowed(struct mt7932 *m);
void mt_retry_missing_policy(struct mt7932 *m);
int mt_6g_connect_policy(struct mt7932 *m, const struct mt7932_6g_ap *ap);
void mt_reg_apply_deferred_locked(struct mt7932 *m);
void mt_policy_debugfs_init(struct mt7932 *m);
int mt_net_open(struct net_device *netdev);
int mt_request_input(struct mt7932 *m, const struct firmware **fw, const char *name);
/* response_lock held; closes RF admission and reports terminal host state. */
void mt_rf_fail_locked(struct mt7932 *m, int error);
void mt_request_recovery(struct mt7932 *m, const char *why);
void mt_recover_work(struct work_struct *work);
int mt_note_restart(void);
bool mt_restart_done(struct mt7932 *m);
int mt_restart_count(void);
void mt_restart_debugfs_init(struct mt7932 *m);
u32 mt_read(struct mt7932 *m, u32 reg);
void mt_write(struct mt7932 *m, u32 reg, u32 value);
void mt_rmw(struct mt7932 *m, u32 reg, u32 mask, u32 value);
int mt_poll(struct mt7932 *m, u32 reg, u32 mask, u32 expected, u32 timeout);
void *mt_alloc(struct mt7932 *m, size_t size, dma_addr_t *dma);
void mt_tx_clean(struct mt7932_ring *q);
int mt_request_ext(struct mt7932 *m, u8 cid, u8 ext, bool runtime, bool set, bool wait,
		      const void *payload, size_t length);
int mt_request(struct mt7932 *m, u8 cid, bool runtime, bool set, bool wait,
		      const void *payload, size_t length);
void mt_transport_snapshot(struct mt7932 *m);
int mt_recovery_gate(struct mt7932 *m);
int mt_cal_procedure(struct mt7932 *m, struct mt7932_cal_piece *pieces,
				    unsigned int count, unsigned int logical, u8 context_version);
int mt_calibration_gate(struct mt7932 *m);
int mt_cal_drain_requests(struct mt7932 *m, const struct firmware *oca);
void mt_cal_input_error(struct mt7932 *m, int error, unsigned int tag, const char *stage);
void mt_data_clean_locked(struct mt7932 *m);
void mt_data_kick_locked(struct mt7932 *m);
void mt_data_clean(struct mt7932 *m);
void mt_bss_presence(struct mt7932 *m, const struct mt7932_event *event);
void mt_data_complete(struct mt7932 *m, const u8 *packet, size_t length);
netdev_tx_t mt_net_xmit(struct sk_buff *skb, struct net_device *netdev);
void mt_data_receive(struct mt7932 *m, const struct mt7932_rx_frame *frame);
extern const struct ethtool_ops mt_ethtool_ops;
int mt_data_prepare(struct mt7932 *m);
struct mt7932 *mt_from_wiphy(struct wiphy *wiphy);
void mt_scan_event(struct mt7932 *m, const struct mt7932_event *event);
void mt_abort_scan(struct wiphy *wiphy, struct wireless_dev *wdev);
int mt_scan_quiesce(struct mt7932 *m);
void mt_scan_settle_retired(struct mt7932 *m);
int mt_connect_discover(struct mt7932 *m);
int mt_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request);
int mt_register_interface(struct mt7932 *m);
int mt_enable_scan(struct mt7932 *m);
void mt_packet_receive(struct mt7932 *m, const u8 *packet, size_t length);
void mt_link_event(struct mt7932 *m, const struct mt7932_event *event);
void mt_beacon_timeout_event(struct mt7932 *m, const struct mt7932_event *event);
struct cfg80211_bss *mt_connect_find_bss(struct mt7932 *m);
void mt_connect_work(struct work_struct *work);
int mt_connect(struct wiphy *wiphy, struct net_device *netdev,
		      struct cfg80211_connect_params *params);
void mt_disconnect_work(struct work_struct *work);
void mt_report_disconnected(struct mt7932 *m, u16 reason, bool local, gfp_t gfp);
int mt_disconnect(struct wiphy *wiphy, struct net_device *netdev, u16 reason);
int mt_deauth(struct wiphy *wiphy, struct net_device *netdev,
		     struct cfg80211_deauth_request *request);
int mt_change_interface(struct wiphy *wiphy, struct net_device *netdev,
			       enum nl80211_iftype type, struct vif_params *params);
int mt_register_regulatory_gate(struct mt7932 *m);
int mt_get_channel(struct wiphy *wiphy, struct wireless_dev *wdev,
		   unsigned int link_id, struct cfg80211_chan_def *chandef);
int mt_get_station(struct wiphy *wiphy, struct wireless_dev *wdev,
		   const u8 *mac, struct station_info *sinfo);
int mt_dump_station(struct wiphy *wiphy, struct wireless_dev *wdev,
		    int idx, u8 *mac, struct station_info *sinfo);
int mt_set_cqm_rssi_config(struct wiphy *wiphy, struct net_device *netdev,
			   s32 rssi_thold, u32 rssi_hyst);
void mt_cqm_reset(struct mt7932 *m);
void mt_cqm_signal(struct mt7932 *m, int signal);
int mt_set_power_mgmt(struct wiphy *wiphy, struct net_device *netdev,
		      bool enabled, int timeout);
void mt_power_work(struct work_struct *work);
int mt_power_init(struct mt7932 *m);
int mt_power_submit(struct mt7932 *m, u8 mode);
#ifdef CONFIG_PM
extern const struct wiphy_wowlan_support mt_wowlan_support;
int mt_wow_suspend(struct wiphy *wiphy, struct cfg80211_wowlan *wow);
int mt_wow_resume(struct wiphy *wiphy);
#endif
void mt_wow_arm(struct mt7932 *m);
int mt_wow_disarm(struct mt7932 *m);
void mt_wow_reason_event(struct mt7932 *m, const struct mt7932_event *event);
void mt_wow_work(struct work_struct *work);

#endif
