// SPDX-License-Identifier: GPL-2.0-only
/* cfg80211 connection to the measured Alpha command/management rings.
 * Control queues recycle across hardware-validated wraps. Packet data uses
 * a separate protocol; never report transmit success before implementing it.
 */
#include <linux/etherdevice.h>
#include <linux/debugfs.h>
#include <linux/ieee80211.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/seq_file.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <net/cfg80211.h>
#include <net/regulatory.h>
#include "centauri.h"

#define CR_BASE 0x100000
#define TR_BASE 0x400000
#define RING_SPAN 0x20000
#define TX_BUFFER 0x600000
#define BUFFER_SIZE 2048
#define DATA_TX_BUFFER 0xd00000
#define DATA_RX_BUFFER 0xe00000
#define DATA_TX_SLOTS 128
#define DATA_TX_QUEUE_LIMIT 64

/* The firmware owns window +8 MiB through +13 MiB as scratch memory. */
static_assert(DATA_TX_BUFFER >= CEN_ALPHA_SCRATCH_OFFSET + CEN_ALPHA_SCRATCH_SIZE);
static_assert(DATA_TX_BUFFER + DATA_TX_SLOTS * BUFFER_SIZE <= DATA_RX_BUFFER);

static bool auto_rings = true;
module_param(auto_rings, bool, 0644);
MODULE_PARM_DESC(auto_rings, "Open the data rings (data_probe ids 76, 24, 15, 12 before the join, 25 after it) from the connect path; 1 = on (default). The data_probe debugfs files still work and are skipped for rings already open");

static int data_probe_locked(struct centauri_wifi *wifi, unsigned int id, bool internal);

static bool roam_scan;
module_param(roam_scan, bool, 0644);
MODULE_PARM_DESC(roam_scan, "Let the firmware run its own roam and background scans after a join; 0 = off (default). The scans take the radio off channel for seconds (72 percent ping loss measured) until the host programs a roam profile");

static bool host_setup = true;
module_param(host_setup, bool, 0644);
MODULE_PARM_DESC(host_setup, "Before a join, tell the firmware the host is awake (command 6) and apply the reference TX defaults (commands 43 and 52) when the tx_config files have not; 1 = on (default)");

static bool verify_pmk;
module_param(verify_pmk, bool, 0400);
MODULE_PARM_DESC(verify_pmk, "Compare firmware PMK with the requested key without exposing either");

static bool eapol_after_assoc;
module_param(eapol_after_assoc, bool, 0644);
MODULE_PARM_DESC(eapol_after_assoc, "One EAPOL-Start probe at the next successful association event");

static bool link_diagnostics;
module_param(link_diagnostics, bool, 0644);
MODULE_PARM_DESC(link_diagnostics, "Query non-secret firmware link status during association");

static bool eapol_control_profile;
module_param(eapol_control_profile, bool, 0644);
MODULE_PARM_DESC(eapol_control_profile, "Use the measured control TX profile for the EAPOL-Start probe");

static bool eapol_unicast;
module_param(eapol_unicast, bool, 0644);
MODULE_PARM_DESC(eapol_unicast, "Address the bounded EAPOL-Start probe to the associated AP");

static bool eapol_relative_lifetime;
module_param(eapol_relative_lifetime, bool, 0644);
MODULE_PARM_DESC(eapol_relative_lifetime, "Use a measured two-second relative expiration for the probe");

static bool data_tx_broadcast;
module_param(data_tx_broadcast, bool, 0400);
MODULE_PARM_DESC(data_tx_broadcast, "Diagnostic: open TR 25 with measured broadcast metadata; transmit only broadcast Ethernet frames");

static bool data_tx_control_profile;
module_param(data_tx_control_profile, bool, 0600);
MODULE_PARM_DESC(data_tx_control_profile, "Diagnostic: select configured TX policy 1 for normal data; apply tx_config action 2 first");

static bool data_tx_native_prepare = true;
module_param(data_tx_native_prepare, bool, 0600);
MODULE_PARM_DESC(data_tx_native_prepare, "Use native packet preparation value 4; disable only to reproduce the legacy data TX failure");

static bool station_link_addresses;
module_param(station_link_addresses, bool, 0400);
MODULE_PARM_DESC(station_link_addresses, "Diagnostic: use the netdev address for each firmware link in this single-link WPA2 client");

static bool alpha_sleep_untested;
module_param(alpha_sleep_untested, bool, 0600);
MODULE_PARM_DESC(alpha_sleep_untested, "TEST ONLY: J813 Alpha host-power and ACIPC sleep handshake");

struct centauri_rx_snapshot {
	__le32 version, count;
	struct {
		u8 completion[32];
		u8 packet[4096];
	} sample[8];
};

struct centauri_wifi {
	struct centauri *control;
	struct cen_alpha_resources a;
	struct wiphy *wiphy;
	struct wireless_dev wdev;
	struct net_device *netdev;
	struct work_struct scan_work;
	struct work_struct connect_work;
	struct delayed_work link_work;
	struct cfg80211_scan_request *scan_request;
	bool scan_abort;
	struct mutex lock;
	/* The logger has its own coherent allocation. Reads/acknowledgements
	 * must remain possible while scan/connect hold the command mutex.
	 * Initialization takes lock then fwlog_lock; readers take only the latter.
	 */
	struct mutex fwlog_lock;
	u16 main_head, command_head, sequence;
	u32 rx_seen[5];
	u16 rx_next[5], rx_head[5];
	u16 reply_sequence, reply_status;
	u32 reply_command, channels_seen, scans_done, beacons_seen;
	bool reply_valid, registered, configured;
	bool stopping;
	bool transport_paused, transport_sleep_sent, host_sleep_sent;
	bool connecting, connected, connect_event, disconnect_event;
	bool associated, pmk_seen, pmk_matches;
	u8 expected_pmk[32];
	u16 connect_status;
	u8 bssid[ETH_ALEN], connect_payload[1024], request_ie[256];
	size_t connect_size, request_ie_size;
	struct dentry *data_probe_file;
	struct dentry *data_status_file;
	struct dentry *data_rx_snapshot_file;
	struct centauri_rx_snapshot rx_snapshot;
	u32 data_rx_received, data_rx_reposted, data_rx_bad;
	struct sk_buff_head data_tx_queue;
	u16 data_tx_length[DATA_TX_SLOTS];
	bool data_tx_busy[DATA_TX_SLOTS];
	u32 data_tx_submitted, data_tx_completed, data_tx_failed;
	u32 data_tx_coalesced, data_tx_bad;
	struct dentry *station_stats_file;
	struct dentry *eapol_start_file;
	struct dentry *country_file;
	struct dentry *link_status_file;
	struct dentry *tx_config_file;
	struct dentry *host_wakeup_file;
	struct dentry *fwlog_start_file, *fwlog_status_file, *fwlog_buffer_file, *fwlog_ack_file;
	void *fwlog_buffer;
	dma_addr_t fwlog_dma;
	size_t fwlog_size;
	bool fwlog_attempted, fwlog_ready;
	u8 tx_profile[10], tx_defaults_applied;
	bool host_awake_sent;
	bool tx_profile_valid;
	u8 link_count, link_prime[3], link_channel[3], link_band[3], emlsr_mode;
	bool link_status_valid, emlsr_valid;
	u16 link_status_size;
	u16 link_tags[12], link_lengths[12];
	u8 link_tlv_count;
	u8 country[3];
	bool country_valid;
	u8 station_stats[0x378];
	bool station_stats_valid;
	u8 rx_crypto_stats[64];
	bool rx_crypto_stats_valid;
	struct dentry *rx_crypto_stats_file;
	u8 dot11_mib_stats[192];
	bool dot11_mib_stats_valid;
	struct dentry *dot11_mib_stats_file;
	u8 tx_power_report[280];
	bool tx_power_report_valid;
	struct dentry *tx_power_report_file;
	u16 stats_reply_size, stats_tag, stats_length;
	u32 data_probe_done;
	struct ieee80211_supported_band bands[2];
	struct ieee80211_channel channels[39];
};

/* These helpers require wifi->lock, including from the connect worker. */
static int transmit_eapol_start(struct centauri_wifi *wifi);
static int query_station_stats(struct centauri_wifi *wifi);
static int query_link_status(struct centauri_wifi *wifi);

static const u32 rx_buffer[5] = { 0, 0, 0x620000, 0x680000, 0x700000 };
static const u16 ring_size[5] = { 0, 64, 192, 96, 192 };
static const u16 five_ghz[] = {
	36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112,
	116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165,
};
static struct ieee80211_rate rates[] = {
	{ .bitrate = 10 }, { .bitrate = 20 }, { .bitrate = 55 }, { .bitrate = 110 },
	{ .bitrate = 60 }, { .bitrate = 90 }, { .bitrate = 120 }, { .bitrate = 180 },
	{ .bitrate = 240 }, { .bitrate = 360 }, { .bitrate = 480 }, { .bitrate = 540 },
};
static const u32 cipher_suites[] = {
	WLAN_CIPHER_SUITE_CCMP, WLAN_CIPHER_SUITE_AES_CMAC,
};

static u16 index_read(struct centauri_wifi *wifi, unsigned int array, unsigned int index)
{
	return le16_to_cpu(READ_ONCE(*(__le16 *)(wifi->a.context + wifi->a.index[array] + 2 * index)));
}

static void index_write(struct centauri_wifi *wifi, unsigned int array,
			unsigned int index, u16 value)
{
	WRITE_ONCE(*(__le16 *)(wifi->a.context + wifi->a.index[array] + 2 * index), cpu_to_le16(value));
}

static void completions(struct centauri_wifi *wifi)
{
	unsigned int vector;

	for (vector = 0; vector <= 4; vector++)
		index_write(wifi, 2, vector, index_read(wifi, 0, vector));
	dma_wmb();
}

static int alpha_message(struct centauri_wifi *wifi, const void *message, size_t size)
{
	u16 head = wifi->main_head;
	u8 *descriptor;
	unsigned long deadline = jiffies + msecs_to_jiffies(2000);

	if (READ_ONCE(wifi->transport_paused))
		return -EBUSY;
	if (head >= 15 || size > 512)
		return -ENOSPC;
	memcpy(wifi->a.messages + (head + 1) * 512, message, size);
	descriptor = wifi->a.context + wifi->a.mtr + 16 * head;
	put_unaligned_le32(1 | (size << 8), descriptor);
	put_unaligned_le64(wifi->a.messages_dma + (head + 1) * 512, descriptor + 4);
	put_unaligned_le32(head + 1, descriptor + 12);
	index_write(wifi, 3, 0, head + 1);
	if (head)
		index_write(wifi, 3, 8, head + 1);
	dma_wmb();
	wifi->main_head++;
	writel(1, wifi->a.bar + 0x1004);
	do {
		if (readl(wifi->a.bar + 8) != 2)
			return -EIO;
		if (index_read(wifi, 1, 0) >= head + 1) {
			dma_rmb();
			completions(wifi);
			return 0;
		}
		usleep_range(100, 200);
	} while (time_before(jiffies, deadline));
	return -ETIMEDOUT;
}

static int open_pair(struct centauri_wifi *wifi, u16 id)
{
	u8 cr[0x2c] = { 2 }, tr[0x34] = { 1 };
	int ret;

	put_unaligned_le32(id | (id << 16), cr + 4);
	put_unaligned_le64(wifi->a.window_dma + CR_BASE + (id - 1) * RING_SPAN, cr + 8);
	put_unaligned_le16(1024, cr + 0x10);
	put_unaligned_le16(0xffff, cr + 0x12);
	ret = alpha_message(wifi, cr, sizeof(cr));
	if (ret)
		return ret;
	put_unaligned_le32(id | (id << 16), tr + 4);
	put_unaligned_le64(wifi->a.window_dma + TR_BASE + (id - 1) * RING_SPAN, tr + 8);
	put_unaligned_le64(~0ULL, tr + 0x10);
	put_unaligned_le16(ring_size[id], tr + 0x18);
	put_unaligned_le16(id, tr + 0x1a);
	put_unaligned_le16(id, tr + 0x1c);
	put_unaligned_le16(0x40, tr + 0x1e);
	put_unaligned_le16(2, tr + 0x24);
	return alpha_message(wifi, tr, sizeof(tr));
}

static void report_beacon(struct centauri_wifi *wifi, const u8 *payload, size_t size)
{
	const u8 *frame, *ie, *end;
	struct ieee80211_channel *channel;
	struct cfg80211_bss *bss;
	u16 frame_size;
	int number = 0, frequency, signal;
	enum nl80211_band band;

	if (!wifi->registered || size < 0x60 + 36)
		return;
	/* The ACIPC packet length is authoritative; the vendor descriptor's
	 * separate 802.11 length can include a stripped FCS. The measured parser
	 * consumes the frame bytes that were actually returned.
	 */
	frame_size = size - 0x60;
	frame = payload + 0x60;
	if (!ieee80211_is_beacon(*(__le16 *)frame) && !ieee80211_is_probe_resp(*(__le16 *)frame))
		return;
	end = frame + frame_size;
	for (ie = frame + 36; ie + 2 <= end && ie + 2 + ie[1] <= end; ie += 2 + ie[1]) {
		if ((ie[0] == WLAN_EID_DS_PARAMS || ie[0] == WLAN_EID_HT_OPERATION) && ie[1])
			number = ie[2];
	}
	if (!number)
		return;
	band = number <= 14 ? NL80211_BAND_2GHZ : NL80211_BAND_5GHZ;
	frequency = ieee80211_channel_to_frequency(number, band);
	channel = ieee80211_get_channel(wifi->wiphy, frequency);
	if (!channel)
		return;
	signal = max((s8)payload[0x16], (s8)payload[0x17]);
	if (signal >= 0 || signal < -127)
		signal = -95;
	bss = cfg80211_inform_bss_frame(wifi->wiphy, channel,
		(struct ieee80211_mgmt *)frame, frame_size, signal * 100, GFP_KERNEL);
	if (bss) {
		wifi->beacons_seen++;
		cfg80211_put_bss(wifi->wiphy, bss);
	}
}

static void post_receive(struct centauri_wifi *wifi, unsigned int ring, u16 slot)
{
	u8 *descriptor = wifi->a.window + TR_BASE + (ring - 1) * RING_SPAN + slot * 16;
	u32 offset = rx_buffer[ring] + slot * BUFFER_SIZE;

	memset(wifi->a.window + offset, 0, BUFFER_SIZE);
	put_unaligned_le32(1 | (BUFFER_SIZE << 8), descriptor);
	put_unaligned_le64(wifi->a.window_dma + offset, descriptor + 4);
	put_unaligned_le32(slot + 1, descriptor + 12);
}

/* Host-owned RX buffer cookies and completion length are the ACIPC fields
 * validated in probe 28. Probe 29 validates the footer's prefix length and
 * Ethernet framing with IPv4 header checksums and IPv6 payload lengths.
 * Called with wifi->lock, including during connect/command polling.
 */
static void drain_data_rx(struct centauri_wifi *wifi)
{
	u16 head, tail, post;
	unsigned int processed = 0;

	if (!(wifi->data_probe_done & BIT(4)) || readl(wifi->a.bar + 8) != 2)
		return;
	head = index_read(wifi, 0, 207);
	tail = index_read(wifi, 2, 207);
	post = index_read(wifi, 3, 12);
	if (head >= 128 || tail >= 128 || post >= 128) {
		wifi->data_rx_bad++;
		return;
	}
	dma_rmb();
	while (tail != head && processed < 127) {
		u8 *completion = wifi->a.window + 0x240000 + tail * 32;
		u32 header = get_unaligned_le32(completion);
		u32 info = get_unaligned_le32(completion + 4);
		u16 cookie = info & 0xffff, length = info >> 16;
		u16 next = (post + 1) % 128;
		u32 count = le32_to_cpu(wifi->rx_snapshot.count);
		u8 *packet, *descriptor;
		u32 buffer;

		if ((header >> 16) != 12 || !cookie || cookie > 127 ||
		    length > 4096 || get_unaligned_le32(completion + 8)) {
			wifi->data_rx_bad++;
			break;
		}
		/* Do not overwrite a descriptor the receive engine still owns. */
		if (next == index_read(wifi, 1, 12))
			break;
		buffer = DATA_RX_BUFFER + (cookie - 1) * 4096;
		packet = wifi->a.window + buffer;
		if (count < ARRAY_SIZE(wifi->rx_snapshot.sample)) {
			memcpy(wifi->rx_snapshot.sample[count].completion, completion, 32);
			memcpy(wifi->rx_snapshot.sample[count].packet, packet, length);
			wifi->rx_snapshot.version = cpu_to_le32(1);
			wifi->rx_snapshot.count = cpu_to_le32(count + 1);
		}
		wifi->data_rx_received++;
		if (wifi->connected && netif_running(wifi->netdev)) {
			u32 footer = get_unaligned_le32(completion + 12);
			unsigned int padding = (footer >> 12) & 15;
			struct sk_buff *skb = NULL;

			/* Extended prefix headers are not decoded yet. */
			if (!(footer & BIT(24)) && length >= padding + ETH_HLEN)
				skb = netdev_alloc_skb_ip_align(wifi->netdev, length - padding);
			if (skb) {
				skb_put_data(skb, packet + padding, length - padding);
				skb->protocol = eth_type_trans(skb, wifi->netdev);
				skb->ip_summed = CHECKSUM_NONE;
				wifi->netdev->stats.rx_packets++;
				wifi->netdev->stats.rx_bytes += length - padding;
				netif_rx(skb);
			} else {
				wifi->netdev->stats.rx_dropped++;
			}
		} else {
			wifi->netdev->stats.rx_dropped++;
		}
		descriptor = wifi->a.window + 0x520000 + post * 16;
		put_unaligned_le32(1 | (4096 << 8), descriptor);
		put_unaligned_le64(wifi->a.window_dma + buffer, descriptor + 4);
		put_unaligned_le32(cookie, descriptor + 12);
		wifi->data_rx_reposted++;
		tail = (tail + 1) % 128;
		post = next;
		processed++;
	}
	if (processed) {
		dma_wmb();
		index_write(wifi, 2, 207, tail);
		index_write(wifi, 3, 12, post);
		dma_wmb();
		writel(1, wifi->a.bar + 0x1050);
	}
}

/* Serialized with the firmware poller. Payload slots remain owned until a
 * CR completion, even if the device has already consumed the TR descriptor.
 */
static void drain_data_tx(struct centauri_wifi *wifi)
{
	u16 head, tail, post;
	unsigned int budget = 256;
	bool posted = false, completed = false;

	if (!(wifi->data_probe_done & BIT(5)) || readl(wifi->a.bar + 8) != 2)
		return;
	head = index_read(wifi, 0, 192);
	tail = index_read(wifi, 2, 192);
	post = index_read(wifi, 3, 25);
	if (head >= 2048 || tail >= 2048 || post >= DATA_TX_SLOTS)
		return;
	dma_rmb();
	while (tail != head && budget--) {
		u8 *entry = wifi->a.window + 0x200000 + tail * 32;
		u16 ring = get_unaligned_le32(entry) >> 16;
		u16 cookie = get_unaligned_le16(entry + 4);
		u16 count = get_unaligned_le16(entry + 9);
		u8 status = entry[12];

		if (ring == 25) {
			unsigned int first, i;

			/* Airship cr_controller::sync maps CD bytes 9:10 to the
			 * transfer count. With the multicomp flag (CD byte 1 bit 0),
			 * the cookie names the LAST transfer; prior transfers share
			 * its completion status. We submit one contiguous DMA buffer
			 * per slot, so those preceding cookies must all still be busy.
			 */
			if (!cookie || cookie > DATA_TX_SLOTS || !count ||
			    count >= DATA_TX_SLOTS || !(entry[1] & 1)) {
				wifi->data_tx_bad++;
				break;
			}
			first = (cookie + DATA_TX_SLOTS - count) % DATA_TX_SLOTS;
			for (i = 0; i < count; i++)
				if (!wifi->data_tx_busy[(first + i) % DATA_TX_SLOTS])
					break;
			if (i != count) {
				wifi->data_tx_bad++;
				break;
			}
			if (count > 1)
				wifi->data_tx_coalesced++;
			for (i = 0; i < count; i++) {
				unsigned int slot = (first + i) % DATA_TX_SLOTS;

				wifi->data_tx_completed++;
				if (status == 0 || status == 0x83) {
					wifi->netdev->stats.tx_packets++;
					wifi->netdev->stats.tx_bytes += wifi->data_tx_length[slot];
				} else {
					wifi->data_tx_failed++;
					wifi->netdev->stats.tx_errors++;
				}
				wifi->data_tx_busy[slot] = false;
			}
		}
		tail = (tail + 1) % 2048;
		completed = true;
	}
	if (completed)
		index_write(wifi, 2, 192, tail);
	budget = DATA_TX_SLOTS - 1;
	while (wifi->connected && !READ_ONCE(wifi->stopping) &&
	       netif_running(wifi->netdev) && budget--) {
		u16 next = (post + 1) % DATA_TX_SLOTS;
		u32 buffer = DATA_TX_BUFFER + post * BUFFER_SIZE;
		u8 *descriptor = wifi->a.window + 0x540000 + post * 32;
		struct sk_buff *skb;
		u8 destination[ETH_ALEN];
		u64 metadata = 6ULL << 27;

		if (wifi->data_tx_busy[post] || next == index_read(wifi, 1, 25))
			break;
		skb = skb_dequeue(&wifi->data_tx_queue);
		if (!skb)
			break;
		/* This diagnostic ring is only valid for its broadcast destination.
		 * Do not silently send unicast or IPv6 multicast through that flow.
		 */
		if (data_tx_broadcast &&
		    (skb_copy_bits(skb, 0, destination, ETH_ALEN) ||
		     !is_broadcast_ether_addr(destination))) {
			wifi->netdev->stats.tx_dropped++;
			dev_kfree_skb_any(skb);
			continue;
		}
		if (skb_copy_bits(skb, 0, wifi->a.window + buffer, skb->len)) {
			wifi->netdev->stats.tx_dropped++;
			dev_kfree_skb_any(skb);
			continue;
		}
		memset(descriptor, 0, 32);
		put_unaligned_le32(3 | (skb->len << 8), descriptor);
		put_unaligned_le64(wifi->a.window_dma + buffer, descriptor + 4);
		put_unaligned_le16(post + 1, descriptor + 12);
		/* AirshipDataPacket type 1 supplies this 16-byte TX header inline.
		 * setTxPolicyId(1), 0x1809ed27c, sets validity bit 60 and ID bit 48.
		 * Keep normal policy zero by default; the switch permits a bounded
		 * comparison with the existing, read-back-verified control profile.
		 */
		if (READ_ONCE(data_tx_control_profile) &&
		    (wifi->tx_defaults_applied & BIT(1)))
			metadata |= BIT_ULL(60) | BIT_ULL(48);
		/* Bits 17:15 carry a 3-bit packet class. A DHCP Discover
		 * captured from the native stack has value 4 (0x20000). The same-connection Linux A/B changes
		 * zero DHCP offers into three offers by setting only this field.
		 * The opt-out retains the old value for diagnostic reproduction.
		 */
		if (READ_ONCE(data_tx_native_prepare))
			metadata |= 4ULL << 15;
		put_unaligned_le64(metadata, descriptor + 16);
		put_unaligned_le64(2000000, descriptor + 24);
		wifi->data_tx_length[post] = skb->len;
		wifi->data_tx_busy[post] = true;
		wifi->data_tx_submitted++;
		dev_kfree_skb_any(skb);
		post = next;
		posted = true;
	}
	if (posted || completed) {
		dma_wmb();
		if (posted)
			index_write(wifi, 3, 25, post);
		dma_wmb();
		writel(1, wifi->a.bar + 0x1030);
	}
	if (wifi->connected && !READ_ONCE(wifi->stopping) && netif_running(wifi->netdev) &&
	    skb_queue_len(&wifi->data_tx_queue) < DATA_TX_QUEUE_LIMIT)
		netif_wake_queue(wifi->netdev);
}

static void drain_control(struct centauri_wifi *wifi, bool pm_command)
{
	unsigned int ring;
	bool posted = false;

	if (READ_ONCE(wifi->transport_paused) && !pm_command)
		return;
	for (ring = 2; ring <= 4; ring++) {
		unsigned int budget = ring_size[ring];

		while (budget--) {
			u16 slot = wifi->rx_next[ring], size, status, sequence;
			u32 command;
			u8 *descriptor = wifi->a.window + TR_BASE + (ring - 1) * RING_SPAN + slot * 16;
			u8 *packet = wifi->a.window + rx_buffer[ring] + slot * BUFFER_SIZE;

			if (!(READ_ONCE(descriptor[15]) & 4))
				break;
			dma_rmb();
			size = get_unaligned_le16(packet + 0x10);
			status = get_unaligned_le16(packet + 0xe);
			sequence = get_unaligned_le16(packet + 0x12);
			command = get_unaligned_le32(packet + 0x14) & ~BIT(30);
			wifi->rx_seen[ring]++;
			wifi->rx_next[ring] = (slot + 1) % ring_size[ring];
			if (size < 24 || size > BUFFER_SIZE)
				goto refill;
			if (ring == 4) {
				report_beacon(wifi, packet + 24, size - 24);
			} else if (command & BIT(31)) {
				if (command == 0x80000030)
					wifi->channels_seen++;
				if (command == 0x80000005)
					wifi->scans_done++;
				if (command == 0x80000000 && wifi->connecting) {
					wifi->connect_event = true;
					wifi->connect_status = status;
				}
				if (command == 0x80000001) {
					wifi->associated = status == 0;
					dev_info(&wifi->control->pdev->dev, "Wi-Fi association event: status=%u\n", status);
					if (eapol_after_assoc && wifi->associated && wifi->connecting &&
					    !wifi->connect_event) {
						int probe_ret = transmit_eapol_start(wifi);

						eapol_after_assoc = false;
						dev_info(&wifi->control->pdev->dev,
							 "association-event EAPOL-Start: rc=%d\n", probe_ret);
					}
				}
				/* ACI event 0x50 carries fixed, non-secret supplicant
				 * diagnostics: result, phase and handshake timestamps.
				 * Do not dump the event or any EAPOL/key material.
				 */
				if (command == 0x80000050 && size >= 64)
					dev_info(&wifi->control->pdev->dev,
						 "Wi-Fi supplicant: result=%u phase=%u times=%u/%u/%u/%u/%u\n",
						 get_unaligned_le32(packet + 0x18), packet[0x1c],
						 get_unaligned_le32(packet + 0x24),
						 get_unaligned_le32(packet + 0x28),
						 get_unaligned_le32(packet + 0x2c),
						 get_unaligned_le32(packet + 0x30),
						 get_unaligned_le32(packet + 0x34));
				if (command == 0x80000003 || command == 0x80000004)
					wifi->disconnect_event = true;
				if (wifi->connecting && command != 0x80000000 &&
				    command != 0x80000001 && command != 0x80000030 &&
				    command != 0x80000005 && command != 0x80000050)
					dev_info_ratelimited(&wifi->control->pdev->dev,
						"Wi-Fi event: id=%#x status=%u size=%u\n",
						command, status, size);
			} else {
				wifi->reply_valid = true;
				wifi->reply_sequence = sequence;
				wifi->reply_status = status;
				wifi->reply_command = command;
				if (command == 10) {
					wifi->country_valid = !status && size >= 27;
					if (wifi->country_valid)
						memcpy(wifi->country, packet + 24, 3);
				}
				if (command == 56) {
					wifi->tx_profile_valid = !status && size >= 34;
					if (wifi->tx_profile_valid)
						memcpy(wifi->tx_profile, packet + 24, sizeof(wifi->tx_profile));
				}
				if (command == 20 && !status) {
					/* getInterfaceStatus skips its 32-byte fixed response. */
					unsigned int pos = 32;

					wifi->link_status_size = size;
					while (pos + 4 <= size) {
						u16 tag = get_unaligned_le16(packet + pos);
						u16 length = get_unaligned_le16(packet + pos + 2);
						u8 *value = packet + pos + 4;
						unsigned int i;

						if (length > size - pos - 4)
							break;
						if (wifi->link_tlv_count < ARRAY_SIZE(wifi->link_tags)) {
							wifi->link_tags[wifi->link_tlv_count] = tag;
							wifi->link_lengths[wifi->link_tlv_count++] = length;
						}
						/* MLO status report, TLVs 377/378.
						 * Keep only link count/channel/role. Do not retain MACs.
						 */
						if (tag == 377 && length >= 64 && value[0] <= 3) {
							wifi->link_count = value[0];
							wifi->link_status_valid = true;
							for (i = 0; i < value[0]; i++) {
								u16 chanspec = get_unaligned_le16(value + 26 + 17 * i);

								wifi->link_prime[i] = value[25 + 17 * i];
								wifi->link_channel[i] = chanspec & 0xff;
								wifi->link_band[i] = (chanspec >> 14) & 3;
							}
						}
						if (tag == 378 && length >= 9) {
							wifi->emlsr_valid = true;
							wifi->emlsr_mode = value[0];
						}
						pos += 4 + length;
					}
				}
				if (command == 0xb0001) {
					wifi->stats_reply_size = size;
					wifi->stats_tag = size >= 28 ? get_unaligned_le16(packet + 24) : 0;
					wifi->stats_length = size >= 28 ? get_unaligned_le16(packet + 26) : 0;
				}
				if (command == 0xb0001 && !status && wifi->stats_tag == 0xf928 &&
				    (wifi->stats_length == 752 || wifi->stats_length == 888) &&
				    size >= 28 + wifi->stats_length) {
					/* Native firmware returns the older 752-byte counter record. */
					memset(wifi->station_stats, 0, sizeof(wifi->station_stats));
					memcpy(wifi->station_stats, packet + 28, wifi->stats_length);
					wifi->station_stats_valid = true;
				}
				if (command == 0xb0001 && !status && wifi->stats_tag == 0xf921 &&
				    wifi->stats_length == sizeof(wifi->rx_crypto_stats) &&
				    size >= 28 + sizeof(wifi->rx_crypto_stats)) {
					memcpy(wifi->rx_crypto_stats, packet + 28, sizeof(wifi->rx_crypto_stats));
					wifi->rx_crypto_stats_valid = true;
				}
				if (command == 0xb0001 && !status && wifi->stats_tag == 0xf91b &&
				    wifi->stats_length == sizeof(wifi->dot11_mib_stats) &&
				    size >= 28 + sizeof(wifi->dot11_mib_stats)) {
					memcpy(wifi->dot11_mib_stats, packet + 28, sizeof(wifi->dot11_mib_stats));
					wifi->dot11_mib_stats_valid = true;
				}
				if (command == 0xb0001 && !status && wifi->stats_tag == 0xf920 &&
				    wifi->stats_length == sizeof(wifi->tx_power_report) &&
				    size >= 28 + sizeof(wifi->tx_power_report)) {
					memcpy(wifi->tx_power_report, packet + 28, sizeof(wifi->tx_power_report));
					wifi->tx_power_report_valid = true;
				}
				if (command == 0xf0025) {
					wifi->pmk_seen = !status && size >= 60;
					wifi->pmk_matches = wifi->pmk_seen &&
						get_unaligned_le16(packet + 26) == 32 &&
						!memcmp(packet + 28, wifi->expected_pmk, 32);
					memzero_explicit(packet + 24, size - 24);
				}
			}
		refill:
			post_receive(wifi, ring, wifi->rx_head[ring]);
			wifi->rx_head[ring] = (wifi->rx_head[ring] + 1) % ring_size[ring];
			dma_wmb();
			index_write(wifi, 3, ring, wifi->rx_head[ring]);
			posted = true;
		}
	}
	completions(wifi);
	if (posted)
		writel(1, wifi->a.bar + 0x1004);
	if (!READ_ONCE(wifi->transport_paused)) {
		drain_data_rx(wifi);
		drain_data_tx(wifi);
	}
}

static void drain(struct centauri_wifi *wifi)
{
	drain_control(wifi, false);
}

static int command_control(struct centauri_wifi *wifi, u32 id, const void *payload,
			   size_t size, bool pm_command)
{
	u16 slot = wifi->command_head, sequence = ++wifi->sequence;
	u8 *packet, *descriptor;
	unsigned long deadline = jiffies + msecs_to_jiffies(1500);

	if (READ_ONCE(wifi->transport_paused) && !pm_command)
		return -EBUSY;
	if (size + 24 > BUFFER_SIZE)
		return -EMSGSIZE;
	if ((slot + 1) % ring_size[1] == index_read(wifi, 1, 1))
		return -ENOSPC;
	packet = wifi->a.window + TX_BUFFER + slot * BUFFER_SIZE;
	memset(packet, 0, 24 + size);
	put_unaligned_le16(24 + size, packet + 0x10);
	put_unaligned_le16(sequence, packet + 0x12);
	put_unaligned_le32(id, packet + 0x14);
	if (size)
		memcpy(packet + 24, payload, size);
	descriptor = wifi->a.window + TR_BASE + slot * 16;
	put_unaligned_le32(1 | ((24 + size) << 8), descriptor);
	put_unaligned_le64(wifi->a.window_dma + TX_BUFFER + slot * BUFFER_SIZE, descriptor + 4);
	put_unaligned_le32(slot + 1, descriptor + 12);
	dma_wmb();
	wifi->command_head = (slot + 1) % ring_size[1];
	index_write(wifi, 3, 1, wifi->command_head);
	wifi->reply_valid = false;
	dma_wmb();
	writel(1, wifi->a.bar + 0x1004);
	do {
		drain_control(wifi, pm_command);
		if (readl(wifi->a.bar + 8) != 2)
			return -EIO;
		/* Bit 30 is more-data-pending, not part of the command identity.
		 * Scan/connect replies set it before their later completion event.
		 */
		if (wifi->reply_valid && wifi->reply_sequence == sequence &&
		    (wifi->reply_command & ~BIT(30)) == id) {
			if (wifi->reply_status)
				dev_info(&wifi->control->pdev->dev,
					 "Wi-Fi command %#x rejected: firmware_status=%u\n",
					 id, wifi->reply_status);
			return wifi->reply_status ? -EREMOTEIO : 0;
		}
		usleep_range(500, 1000);
	} while (time_before(jiffies, deadline));
	dev_err(&wifi->control->pdev->dev, "Wi-Fi command %#x timed out (rx=%u)\n", id, wifi->rx_seen[2]);
	return -ETIMEDOUT;
}

static int command(struct centauri_wifi *wifi, u32 id, const void *payload, size_t size)
{
	return command_control(wifi, id, payload, size, false);
}

static int radio_start(struct centauri_wifi *wifi)
{
	u8 start[5 + 4 + 3 * ETH_ALEN] = { 0, 0x0b, 0, 0, 0 };
	size_t start_size = 5;
	unsigned int ring, slot;
	int ret;

	/* The stock 26.x startInterface supplies TLV 385, length 18; the macOS 27.0
	 * adapter uses 386 (board table). Firmware 0x5736c0 otherwise generates three unrelated
	 * random addresses. This diagnostic keeps the sole active link equal
	 * to the netdev identity; it does not enable multi-link association.
	 */
	if (station_link_addresses) {
		const struct centauri_board *board = centauri_board();

		put_unaligned_le16(board ? board->link_tlv : 385, start + 5);
		put_unaligned_le16(3 * ETH_ALEN, start + 7);
		for (slot = 0; slot < 3; slot++)
			ether_addr_copy(start + 9 + slot * ETH_ALEN, wifi->netdev->dev_addr);
		start_size = sizeof(start);
	}
	for (ring = 1; ring <= 4; ring++) {
		ret = open_pair(wifi, ring);
		if (ret)
			return ret;
	}
	for (ring = 2; ring <= 4; ring++) {
		for (slot = 0; slot < ring_size[ring] - 1; slot++)
			post_receive(wifi, ring, slot);
		wifi->rx_head[ring] = ring_size[ring] - 1;
		dma_wmb();
		index_write(wifi, 3, ring, ring_size[ring] - 1);
	}
	dma_wmb();
	writel(1, wifi->a.bar + 0x1004);
	msleep(400);
	ret = command(wifi, 0, NULL, 0);
	if (!ret)
		ret = command(wifi, 1, wifi->netdev->dev_addr, ETH_ALEN);
	if (!ret)
		ret = command(wifi, 2, start, start_size);
	if (ret)
		return ret;
	msleep(1000);
	wifi->configured = true;
	dev_info(&wifi->control->pdev->dev, "Wi-Fi command rings ready; interface started\n");
	return 0;
}

/* The station's RSN element (body) with PSK as its only AKM, written to selected; returns its size. */
static int select_rsn(const u8 *body, size_t size, u8 *selected)
{
	size_t offset, tail;
	u16 count;
	unsigned int i;

	if (size < 8 || get_unaligned_le16(body) != 1)
		return -EINVAL;
	count = get_unaligned_le16(body + 6);
	offset = 8 + 4 * count;
	if (!count || offset + 2 > size)
		return -EINVAL;
	count = get_unaligned_le16(body + offset);
	tail = offset + 2 + 4 * count;
	if (!count || tail > size)
		return -EINVAL;
	for (i = 0; i < count; i++)
		if (get_unaligned_be32(body + offset + 2 + 4 * i) == WLAN_AKM_SUITE_PSK)
			break;
	if (i == count)
		return -EOPNOTSUPP;
	memcpy(selected, body, offset);
	put_unaligned_le16(1, selected + offset);
	put_unaligned_be32(WLAN_AKM_SUITE_PSK, selected + offset + 2);
	memcpy(selected + offset + 6, body + tail, size - tail);
	return offset + 6 + size - tail;
}

static void append_tlv(u8 *payload, size_t *size, u16 tag, const void *data, u16 length);

/* Dwell times in ms for the scan request. The macOS 27 scan adapter copies
 * the first request dwell field into frame +0x20 and +0x22 and the second into +0x24 and logs
 * "dwelltime active[%d, %d] passive %d". A zero passive dwell makes the J873 scan state machine
 * assert in SCHED_Req_Add (the dwell it selects must not be zero) and the whole firmware traps.
 */
#define SCAN_ACTIVE_DWELL_MS	110
#define SCAN_PASSIVE_DWELL_MS	110

/* Fixed scan request (command 0x1000f), 14 bytes, chunk number first. */
static size_t build_scan(struct centauri_wifi *wifi, unsigned int chunk, u8 *buf)
{
	u8 fixed[14] = { 1, 0, 0, 0x80, 0, 2, 0, 0x10 };
	const struct centauri_board *board = centauri_board();
	u8 list[1 + 4 * 13], oce[5] = { 0, 0, 1, 0, 0 };
	unsigned int first = chunk * 13, n, i;
	size_t size = sizeof(fixed);

	put_unaligned_le16(SCAN_ACTIVE_DWELL_MS, fixed + 8);
	put_unaligned_le16(SCAN_ACTIVE_DWELL_MS, fixed + 10);
	put_unaligned_le16(SCAN_PASSIVE_DWELL_MS, fixed + 12);
	memcpy(buf, fixed, sizeof(fixed));
	buf[0] = chunk + 1;
	if (!board || !board->scan_tlvs)
		return size;
	/* The macOS 27.0 scan request appends TLV 0x104, a count and then
	 * {channel, band (0 = 2.4 GHz, 1 = 5 GHz), 0, 0} per channel, and TLV 0x165, an OCE block
	 * whose flags byte always has bit 0 set. One chunk covers 13 entries of the wiphy table.
	 */
	n = min_t(unsigned int, 13, 13 + ARRAY_SIZE(five_ghz) - first);
	list[0] = n;
	for (i = 0; i < n; i++) {
		const struct ieee80211_channel *channel = &wifi->channels[first + i];

		list[1 + 4 * i] = channel->hw_value;
		list[2 + 4 * i] = channel->band == NL80211_BAND_5GHZ;
		list[3 + 4 * i] = 0;
		list[4 + 4 * i] = 0;
	}
	append_tlv(buf, &size, 0x104, list, 1 + 4 * n);
	append_tlv(buf, &size, 0x165, oce, sizeof(oce));
	return size;
}

static void scan_worker(struct work_struct *work)
{
	struct centauri_wifi *wifi = container_of(work, struct centauri_wifi, scan_work);
	struct cfg80211_scan_info info = {};
	struct cfg80211_scan_request *request;
	u8 scan[14 + 4 + 1 + 4 * 13 + 4 + 5];
	size_t scan_size;
	unsigned int chunk, chunks = DIV_ROUND_UP(13 + ARRAY_SIZE(five_ghz), 13);
	u32 channels_start, beacons_start;
	int ret = 0;

	mutex_lock(&wifi->lock);
	channels_start = wifi->channels_seen;
	beacons_start = wifi->beacons_seen;
	for (chunk = 0; chunk < chunks && !ret; chunk++) {
		u32 channels = wifi->channels_seen, done = wifi->scans_done, seen = channels;
		unsigned int count = min_t(unsigned int, 13,
					   13 + ARRAY_SIZE(five_ghz) - chunk * 13);
		unsigned long deadline, progress;
		bool stopped = false, refused = false;

		scan_size = build_scan(wifi, chunk, scan);
		ret = command(wifi, 65551, scan, scan_size);
		if (ret)
			break;
		/*
		 * The firmware reports every channel it finishes (event 0x30) and then the scan
		 * (event 0x05). This loop used to stop each chunk after its 8th channel, so a scan
		 * covered 24 of the 38 channels (J815 boot 2d103457: "channels=24"): 2.4 GHz 9-13,
		 * 5 GHz 100-116 and 153-165 were never scanned and 149 was the last one kept. A
		 * 5 GHz BSS there came and went between scans, and the supplicant joined 2.4 GHz.
		 * The chunk now runs to its own end. It's stopped only once all of its channels
		 * have reported without a scan event, when it goes quiet for a second after some
		 * progress (a channel the firmware skips under the country rules), at its
		 * deadline, or on an abort. A chunk that ends early keeps what it found.
		 */
		deadline = jiffies +
			   msecs_to_jiffies(max(8000U, count * 2 * SCAN_PASSIVE_DWELL_MS + 2000));
		progress = jiffies;
		do {
			drain(wifi);
			if (readl(wifi->a.bar + 8) != 2) {
				ret = -EIO;
				break;
			}
			if (wifi->scans_done != done)
				break;
			if (READ_ONCE(wifi->scan_abort)) {
				if (!stopped)
					command(wifi, 65537, NULL, 0);
				ret = -ECANCELED;
				break;
			}
			if (wifi->channels_seen != seen) {
				seen = wifi->channels_seen;
				progress = jiffies;
			}
			if (!stopped &&
			    (seen - channels >= count ||
			     (seen != channels &&
			      time_after(jiffies, progress + msecs_to_jiffies(1000))))) {
				stopped = true;
				/*
				 * A refused stop (firmware status 342 on J815) means the chunk
				 * had ended already: keep its results and go on.
				 */
				if (command(wifi, 65537, NULL, 0)) {
					dev_dbg(&wifi->control->pdev->dev,
						"Wi-Fi scan chunk %u: stop refused (status %u), chunk over\n",
						chunk, wifi->reply_status);
					refused = true;
					break;
				}
				/* The stop's own scan event ends the wait. */
				deadline = jiffies + msecs_to_jiffies(1000);
			}
			usleep_range(1000, 2000);
		} while (time_before(jiffies, deadline));
		if (!ret && !refused && wifi->scans_done == done) {
			dev_warn(&wifi->control->pdev->dev,
				 "Wi-Fi scan chunk %u: %u of %u channels, no scan event%s\n", chunk,
				 wifi->channels_seen - channels, count,
				 stopped ? " after the stop" : "");
			if (!stopped)
				command(wifi, 65537, NULL, 0);
		}
	}
	info.aborted = !!ret;
	request = wifi->scan_request;
	wifi->scan_request = NULL;
	mutex_unlock(&wifi->lock);
	dev_info(&wifi->control->pdev->dev, "Wi-Fi scan finished: rc=%d channels=%u of %zu beacons=%u\n",
		 ret, wifi->channels_seen - channels_start, 13 + ARRAY_SIZE(five_ghz),
		 wifi->beacons_seen - beacons_start);
	if (request)
		cfg80211_scan_done(request, &info);
}

static int wifi_scan(struct wiphy *wiphy, struct cfg80211_scan_request *request)
{
	struct centauri_wifi *wifi = *(struct centauri_wifi **)wiphy_priv(wiphy);
	int ret = 0;

	mutex_lock(&wifi->lock);
	if (!wifi->configured)
		ret = -ENETDOWN;
	else if (wifi->scan_request || wifi->connecting)
		ret = -EBUSY;
	else {
		wifi->scan_request = request;
		schedule_work(&wifi->scan_work);
	}
	mutex_unlock(&wifi->lock);
	return ret;
}

static int rsn_caps_offset(const u8 *body, size_t size)
{
	size_t offset = 6;
	u16 count;
	unsigned int i;

	for (i = 0; i < 2; i++) {
		if (offset + 2 > size)
			return -EINVAL;
		count = get_unaligned_le16(body + offset);
		offset += 2 + 4 * count;
		if (offset > size)
			return -EINVAL;
	}
	return offset + 2 <= size ? offset : -EINVAL;
}

static void append_tlv(u8 *payload, size_t *size, u16 tag, const void *data, u16 length)
{
	put_unaligned_le16(tag, payload + *size);
	put_unaligned_le16(length, payload + *size + 2);
	memcpy(payload + *size + 4, data, length);
	*size += 4 + length;
}

static int wifi_connect(struct wiphy *wiphy, struct net_device *netdev,
			struct cfg80211_connect_params *params)
{
	/* The stock 26.x join emits a 12-byte TLV 376. The macOS 27.0.1
	 * adapter writes tag 0x178 with length 14: the MLD configuration bytes
	 * that its default initialisation fills with the defaults below, bytes
	 * 12 and 13 being 0 and the flags byte (bits from the connection parameters, 0 by default).
	 * Preserve the unspecified-link masks; all-zero is not the non-MLO default.
	 */
	static const u8 mld_defaults[14] = {
		1, 0, 1, 0xff, 0xff, 0xff, 0xff, 0xff,
		0xff, 0xff, 0xff, 3, 0, 0,
	};
	/* Firmware connection-context defaults (aciw 0x511ee8). TLV 0xf913
	 * replaces all five values. Sending zeros arms Supp_AddTimer with a
	 * zero duration and aborts WPA before message 2 can be transmitted.
	 */
	static const __le32 connection_timeouts[] = {
		cpu_to_le32(260), cpu_to_le32(6000), cpu_to_le32(2000),
		cpu_to_le32(15000), cpu_to_le32(300),
	};
	struct centauri_wifi *wifi = *(struct centauri_wifi **)wiphy_priv(wiphy);
	struct cfg80211_bss *bss;
	const struct cfg80211_bss_ies *ies;
	const u8 *rsn, *ap_rsn;
	u8 rsn_body[255], ssid[35], zero[20] = {}, pmf;
	u8 *payload = wifi->connect_payload;
	size_t size = 130, rsn_size;
	u16 caps = 0;
	int ret = 0, offset, i;

	/*
	 * wpa_supplicant without SME lists every AKM the network allows
	 * (NetworkManager's "WPA-PSK WPA-PSK-SHA256" gives PSK and PSK-SHA256).
	 * Accept any list that offers PSK and send the firmware PSK alone, as
	 * the T6050 driver does (select_rsn).
	 */
	for (i = 0; i < params->crypto.n_akm_suites; i++)
		if (params->crypto.akm_suites[i] == WLAN_AKM_SUITE_PSK)
			break;
	if (!params->ssid_len || params->ssid_len > 32 || !params->crypto.psk ||
	    i == params->crypto.n_akm_suites ||
	    params->crypto.n_ciphers_pairwise != 1 || params->crypto.ciphers_pairwise[0] != WLAN_CIPHER_SUITE_CCMP ||
	    params->crypto.cipher_group != WLAN_CIPHER_SUITE_CCMP || params->ie_len > sizeof(wifi->request_ie)) {
		dev_info(&wifi->control->pdev->dev,
			 "connect refused: ssid_len=%zu psk=%d akms=%d [%08x %08x] pairwise=%d [%08x] group=%08x ie_len=%zu\n",
			 params->ssid_len, !!params->crypto.psk, params->crypto.n_akm_suites,
			 params->crypto.n_akm_suites > 0 ? params->crypto.akm_suites[0] : 0,
			 params->crypto.n_akm_suites > 1 ? params->crypto.akm_suites[1] : 0,
			 params->crypto.n_ciphers_pairwise,
			 params->crypto.n_ciphers_pairwise ? params->crypto.ciphers_pairwise[0] : 0,
			 params->crypto.cipher_group, params->ie_len);
		return -EOPNOTSUPP;
	}
	rsn = cfg80211_find_ie(WLAN_EID_RSN, params->ie, params->ie_len);
	if (!rsn) {
		dev_info(&wifi->control->pdev->dev, "connect refused: no RSN element in the request\n");
		return -EINVAL;
	}
	ret = select_rsn(rsn + 2, rsn[1], rsn_body);
	if (ret < 0 || rsn_caps_offset(rsn_body, ret) < 0) {
		dev_info(&wifi->control->pdev->dev, "connect refused: RSN element %*ph not usable (%d)\n",
			 rsn[1] + 2, rsn, ret);
		return ret < 0 ? ret : -EINVAL;
	}
	rsn_size = ret;
	ret = 0;
	mutex_lock(&wifi->lock);
	if (!wifi->configured || wifi->connecting || wifi->connected || wifi->scan_request) {
		ret = -EBUSY;
		goto out;
	}
	bss = cfg80211_get_bss(wiphy, params->channel, params->bssid,
		params->ssid, params->ssid_len, IEEE80211_BSS_TYPE_ESS, IEEE80211_PRIVACY_ON);
	if (!bss) {
		ret = -ENOENT;
		goto out;
	}
	/* Match the measured successful J813 association shape. The firmware
	 * expects the AP RSN capability bits even with the legal single-AKM STA IE.
	 */
	rcu_read_lock();
	ies = rcu_dereference(bss->ies);
	ap_rsn = ies ? cfg80211_find_ie(WLAN_EID_RSN, ies->data, ies->len) : NULL;
	if (ap_rsn) {
		offset = rsn_caps_offset(ap_rsn + 2, ap_rsn[1]);
		if (offset >= 0)
			caps = get_unaligned_le16(ap_rsn + 2 + offset);
	}
	rcu_read_unlock();
	offset = rsn_caps_offset(rsn_body, rsn_size);
	put_unaligned_le16(caps, rsn_body + offset);
	pmf = !!(caps & BIT(7));
	memset(payload, 0, sizeof(wifi->connect_payload));
	payload[0] = 1;
	memcpy(payload + 1, bss->bssid, ETH_ALEN);
	memcpy(wifi->bssid, bss->bssid, ETH_ALEN);
	payload[7] = ieee80211_frequency_to_channel(bss->channel->center_freq);
	/* Firmware band field: chanspec 0xc000 (5 GHz) is band 1 and
	 * 0x4000 (6 GHz) is band 2.
	 */
	payload[8] = bss->channel->band == NL80211_BAND_2GHZ ? 0 : 1;
	payload[0x47] = 4;
	/* The firmware's fixed security field is 4 for CCMP.
	 * The RSN IE does not replace this fixed cipher field.
	 */
	put_unaligned_le16(4, payload + 0x48);
	put_unaligned_le16(32, payload + 0x4a);
	memcpy(payload + 0x4c, params->crypto.psk, 32);
	if (verify_pmk)
		memcpy(wifi->expected_pmk, params->crypto.psk, 32);
	append_tlv(payload, &size, 304, zero, 1);
	append_tlv(payload, &size, 63763, connection_timeouts, sizeof(connection_timeouts));
	ssid[0] = 1;
	ssid[1] = 4;
	ssid[2] = params->ssid_len;
	memcpy(ssid + 3, params->ssid, params->ssid_len);
	append_tlv(payload, &size, 342, ssid, 3 + params->ssid_len);
	append_tlv(payload, &size, WLAN_EID_RSN, rsn_body, rsn_size);
	/* The fixed field carries the PMK. The firmware accepts a join with the PMK alone (measured); the macOS TLV 313 carries the
	 * passphrase and is not sent.
	 */
	append_tlv(payload, &size, 316, &pmf, 1);
	append_tlv(payload, &size, 376, mld_defaults, sizeof(mld_defaults));
	wifi->connect_size = size;
	memcpy(wifi->request_ie, params->ie, params->ie_len);
	wifi->request_ie_size = params->ie_len;
	wifi->connecting = true;
	wifi->connect_status = 0;
	wifi->associated = wifi->pmk_seen = wifi->pmk_matches = false;
	wifi->connect_event = wifi->disconnect_event = false;
	cfg80211_put_bss(wiphy, bss);
	schedule_work(&wifi->connect_work);
out:
	mutex_unlock(&wifi->lock);
	return ret;
}

/* Without the host power state the firmware treats the host as asleep, powers the PHYs down and traps about 15 s after the
 * join. Same three commands as the host_wakeup and tx_config debugfs files; called with wifi->lock held.
 */
static void host_setup_commands(struct centauri_wifi *wifi)
{
	static const u8 awake[3] = {};
	static const u8 control_profile[] = { 0x54, 1, 10, 0, 0, 8, 0, 0, 0xff, 0xff, 0xff, 0xff, 1, 1 };
	u8 lifetime[20] = {};
	int ret, i;

	if (!host_setup)
		return;
	if (!wifi->host_awake_sent) {
		ret = command(wifi, 6, awake, sizeof(awake));
		dev_info(&wifi->control->pdev->dev, "host power state awake: rc=%d status=%u\n", ret, wifi->reply_status);
		wifi->host_awake_sent = !ret;
	}
	if (!(wifi->tx_defaults_applied & BIT(0))) {
		for (i = 0; i < 4; i++)
			put_unaligned_le32(2000000, lifetime + 4 + 4 * i);
		ret = command(wifi, 43, lifetime, sizeof(lifetime));
		if (!ret)
			wifi->tx_defaults_applied |= BIT(0);
		dev_info(&wifi->control->pdev->dev, "TX packet lifetime defaults: rc=%d\n", ret);
	}
	if (!(wifi->tx_defaults_applied & BIT(1))) {
		ret = command(wifi, 52, control_profile, sizeof(control_profile));
		if (!ret)
			wifi->tx_defaults_applied |= BIT(1);
		dev_info(&wifi->control->pdev->dev, "TX control profile defaults: rc=%d\n", ret);
	}
}

static void connect_worker(struct work_struct *work)
{
	struct centauri_wifi *wifi = container_of(work, struct centauri_wifi, connect_work);
	unsigned long deadline;
	u16 slot, status, command_status;
	bool checked_pmk = false, probed_eapol = false, queried_link = false;
	int ret;

	mutex_lock(&wifi->lock);
	if (auto_rings) {
		static const unsigned int ring_ids[] = { 76, 24, 15, 12 };
		unsigned int n;

		for (n = 0; n < ARRAY_SIZE(ring_ids); n++) {
			int ring_ret = data_probe_locked(wifi, ring_ids[n], true);

			if (ring_ret && ring_ret != -EBUSY)
				dev_warn(&wifi->control->pdev->dev, "data ring %u setup failed: %d\n", ring_ids[n], ring_ret);
		}
	}
	host_setup_commands(wifi);
	slot = wifi->command_head;
	ret = command(wifi, 131072, wifi->connect_payload, wifi->connect_size);
	command_status = wifi->reply_status;
	memzero_explicit(wifi->connect_payload, sizeof(wifi->connect_payload));
	deadline = jiffies + msecs_to_jiffies(15000);
	while (!ret && !wifi->connect_event && time_before(jiffies, deadline)) {
		drain(wifi);
		if (link_diagnostics && wifi->associated && !queried_link) {
			unsigned int i;
			int link_ret;

			queried_link = true;
			link_ret = query_link_status(wifi);
			dev_info(&wifi->control->pdev->dev,
				 "connection link status: rc=%d complete=%d valid=%d count=%u emlsr=%d/%u\n",
				 link_ret, wifi->connect_event, wifi->link_status_valid, wifi->link_count,
				 wifi->emlsr_valid, wifi->emlsr_mode);
			for (i = 0; i < wifi->link_tlv_count; i++)
				dev_info(&wifi->control->pdev->dev, "connection link TLV: tag=%u length=%u\n",
					 wifi->link_tags[i], wifi->link_lengths[i]);
			if (wifi->link_status_valid)
				for (i = 0; i < wifi->link_count; i++)
					dev_info(&wifi->control->pdev->dev,
						 "connection link %u: prime=%u channel=%u band=%u\n",
						 i, wifi->link_prime[i], wifi->link_channel[i], wifi->link_band[i]);
		}
		if (eapol_after_assoc && wifi->associated && !probed_eapol) {
			unsigned int i;
			int probe_ret;

			probed_eapol = true;
			probe_ret = query_station_stats(wifi);
			dev_info(&wifi->control->pdev->dev,
				 "connection counters: complete=%d rc=%d valid=%d length=%u\n",
				 wifi->connect_event, probe_ret, wifi->station_stats_valid, wifi->stats_length);
			if (!probe_ret && wifi->station_stats_valid)
				for (i = 0; i < wifi->stats_length; i += 4) {
					u32 value = get_unaligned_le32(wifi->station_stats + i);

					if (value)
						dev_info(&wifi->control->pdev->dev,
							 "station counter %03x=%u\n", i, value);
				}
		}
		if (verify_pmk && wifi->associated && !checked_pmk) {
			int key_ret = command(wifi, 0xf0025, wifi->bssid, ETH_ALEN);

			checked_pmk = true;
			dev_info(&wifi->control->pdev->dev,
				 "Wi-Fi firmware key check: rc=%d available=%d matches=%d\n",
				 key_ret, wifi->pmk_seen, wifi->pmk_matches);
		}
		if (readl(wifi->a.bar + 8) != 2)
			ret = -EIO;
		usleep_range(1000, 2000);
	}
	if (!ret && !wifi->connect_event)
		ret = -ETIMEDOUT;
	status = !ret && wifi->connect_status == 0 ? WLAN_STATUS_SUCCESS : WLAN_STATUS_UNSPECIFIED_FAILURE;
	wifi->connecting = false;
	wifi->connected = status == WLAN_STATUS_SUCCESS;
	if (wifi->connected && !roam_scan) {
		int roam_ret = command(wifi, 0x20016, NULL, 0);

		dev_info(&wifi->control->pdev->dev, "roam scan off: rc=%d status=%u\n", roam_ret, wifi->reply_status);
	}
	memzero_explicit(wifi->expected_pmk, sizeof(wifi->expected_pmk));
	/* The command and connection completion have consumed the PMK buffer. */
	if (slot < ring_size[1])
		memzero_explicit(wifi->a.window + TX_BUFFER + slot * BUFFER_SIZE, BUFFER_SIZE);
	mutex_unlock(&wifi->lock);
	dev_info(&wifi->control->pdev->dev,
		 "Wi-Fi authentication result: rc=%d command_status=%u event_status=%u complete_event=%d\n",
		 ret, command_status, wifi->connect_status, wifi->connect_event);
	cfg80211_connect_result(wifi->netdev, wifi->bssid,
		wifi->request_ie, wifi->request_ie_size, NULL, 0, status, GFP_KERNEL);
	if (status == WLAN_STATUS_SUCCESS && auto_rings && !READ_ONCE(wifi->stopping)) {
		/* The normal TX ring is opened after the connection result, as in the manual recipe (data_probe 25 after wpa completed);
		 * doing it after cfg80211_connect_result keeps the join inside the supplicant's timeout.
		 */
		int ring_ret;

		mutex_lock(&wifi->lock);
		ring_ret = data_probe_locked(wifi, 25, true);
		mutex_unlock(&wifi->lock);
		if (ring_ret && ring_ret != -EBUSY)
			dev_warn(&wifi->control->pdev->dev, "data ring 25 setup failed: %d\n", ring_ret);
	}
	if (status == WLAN_STATUS_SUCCESS && !READ_ONCE(wifi->stopping)) {
		netif_carrier_on(wifi->netdev);
		if (wifi->data_probe_done & BIT(5))
			netif_wake_queue(wifi->netdev);
		schedule_delayed_work(&wifi->link_work, msecs_to_jiffies(20));
	}
}

static void link_worker(struct work_struct *work)
{
	struct centauri_wifi *wifi = container_of(to_delayed_work(work), struct centauri_wifi, link_work);
	bool disconnected;

	mutex_lock(&wifi->lock);
	if (wifi->transport_paused) {
		mutex_unlock(&wifi->lock);
		return;
	}
	drain(wifi);
	disconnected = wifi->disconnect_event || READ_ONCE(wifi->stopping) ||
		readl(wifi->a.bar + 8) != 2;
	if (disconnected)
		wifi->connected = false;
	mutex_unlock(&wifi->lock);
	if (disconnected) {
		netif_stop_queue(wifi->netdev);
		skb_queue_purge(&wifi->data_tx_queue);
		netif_carrier_off(wifi->netdev);
		cfg80211_disconnected(wifi->netdev, WLAN_REASON_UNSPECIFIED, NULL, 0, false, GFP_KERNEL);
	} else {
		schedule_delayed_work(&wifi->link_work, msecs_to_jiffies(20));
	}
}

/* MSI from Control or Alpha: service the data rings now instead of at the next poll. */
void centauri_wifi_kick(struct centauri *c)
{
	struct centauri_wifi *wifi = READ_ONCE(c->wifi);

	if (wifi && READ_ONCE(wifi->connected) && !READ_ONCE(wifi->stopping) &&
	    !READ_ONCE(wifi->transport_paused))
		mod_delayed_work(system_percpu_wq, &wifi->link_work, 0);
}

static int wifi_disconnect(struct wiphy *wiphy, struct net_device *netdev, u16 reason)
{
	struct centauri_wifi *wifi = *(struct centauri_wifi **)wiphy_priv(wiphy);
	u8 payload[16] = {};
	bool connected;
	int ret;

	mutex_lock(&wifi->lock);
	if (wifi->connecting) {
		mutex_unlock(&wifi->lock);
		return -EBUSY;
	}
	put_unaligned_le16(reason, payload);
	put_unaligned_le16(328, payload + 8);
	put_unaligned_le16(4, payload + 10);
	/* Preserve the firmware's default disconnect budget. */
	put_unaligned_le32(500, payload + 12);
	ret = command(wifi, 131073, payload, sizeof(payload));
	connected = wifi->connected;
	wifi->connected = false;
	mutex_unlock(&wifi->lock);
	cancel_delayed_work_sync(&wifi->link_work);
	netif_stop_queue(netdev);
	skb_queue_purge(&wifi->data_tx_queue);
	netif_carrier_off(netdev);
	if (connected)
		cfg80211_disconnected(netdev, reason, NULL, 0, true, GFP_KERNEL);
	return ret == -EREMOTEIO && wifi->reply_status == 326 ? 0 : ret;
}

static int wifi_power_mgmt(struct wiphy *wiphy, struct net_device *netdev,
			   bool enabled, int timeout)
{
	struct centauri_wifi *wifi = *(struct centauri_wifi **)wiphy_priv(wiphy);
	u8 mode = enabled ? 0 : 3;
	int ret;

	/* Power-save mode, command 7. */
	mutex_lock(&wifi->lock);
	ret = wifi->configured ? command(wifi, 7, &mode, sizeof(mode)) : -ENETDOWN;
	mutex_unlock(&wifi->lock);
	return ret;
}

static void wifi_regulatory(struct wiphy *wiphy, struct regulatory_request *request)
{
	struct centauri_wifi *wifi = *(struct centauri_wifi **)wiphy_priv(wiphy);
	u8 country[3] = { request->alpha2[0], request->alpha2[1], 1 };
	int ret;

	/* Country code, command 9. Leave world-domain
	 * handling to cfg80211; do not invent a device country or bypass SAR.
	 */
	if (country[0] < 'A' || country[0] > 'Z' || country[1] < 'A' || country[1] > 'Z')
		return;
	mutex_lock(&wifi->lock);
	if (wifi->configured) {
		ret = command(wifi, 9, country, sizeof(country));
		dev_info(&wifi->control->pdev->dev,
			 "Wi-Fi country %c%c: rc=%d\n", country[0], country[1], ret);
	}
	mutex_unlock(&wifi->lock);
}

/*
 * The joined BSS as cfg80211 knows it from the last scan. No firmware query
 * for live link statistics is known yet, so its signal is the scan's.
 */
static struct cfg80211_bss *wifi_joined_bss(struct centauri_wifi *wifi)
{
	if (!wifi->connected)
		return NULL;
	return cfg80211_get_bss(wifi->wiphy, NULL, wifi->bssid, NULL, 0,
				IEEE80211_BSS_TYPE_ESS, IEEE80211_PRIVACY_ANY);
}

static int wifi_get_station(struct wiphy *wiphy, struct wireless_dev *wdev,
			    const u8 *mac, struct station_info *sinfo)
{
	struct centauri_wifi *wifi = *(struct centauri_wifi **)wiphy_priv(wiphy);
	struct net_device_stats *stats = &wifi->netdev->stats;
	struct cfg80211_bss *bss;
	int ret = 0;

	mutex_lock(&wifi->lock);
	if (!wifi->connected || !ether_addr_equal(mac, wifi->bssid)) {
		ret = -ENOENT;
		goto out;
	}
	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_RX_PACKETS) |
			 BIT_ULL(NL80211_STA_INFO_TX_PACKETS) |
			 BIT_ULL(NL80211_STA_INFO_RX_BYTES64) |
			 BIT_ULL(NL80211_STA_INFO_TX_BYTES64) |
			 BIT_ULL(NL80211_STA_INFO_TX_FAILED) |
			 BIT_ULL(NL80211_STA_INFO_RX_DROP_MISC);
	sinfo->rx_packets = stats->rx_packets;
	sinfo->tx_packets = stats->tx_packets;
	sinfo->rx_bytes = stats->rx_bytes;
	sinfo->tx_bytes = stats->tx_bytes;
	sinfo->tx_failed = stats->tx_errors;
	sinfo->rx_dropped_misc = stats->rx_dropped;

	bss = wifi_joined_bss(wifi);
	if (bss) {
		sinfo->filled |= BIT_ULL(NL80211_STA_INFO_SIGNAL);
		sinfo->signal = bss->signal / 100;	/* mBm */
		cfg80211_put_bss(wiphy, bss);
	}
out:
	mutex_unlock(&wifi->lock);
	return ret;
}

static int wifi_get_channel(struct wiphy *wiphy, struct wireless_dev *wdev,
			    unsigned int link_id, struct cfg80211_chan_def *chandef)
{
	struct centauri_wifi *wifi = *(struct centauri_wifi **)wiphy_priv(wiphy);
	struct cfg80211_bss *bss;
	int ret = -ENODATA;

	mutex_lock(&wifi->lock);
	bss = wifi_joined_bss(wifi);
	if (bss) {
		/* The join asks the firmware for a 20 MHz channel. */
		cfg80211_chandef_create(chandef, bss->channel, NL80211_CHAN_NO_HT);
		cfg80211_put_bss(wiphy, bss);
		ret = 0;
	}
	mutex_unlock(&wifi->lock);
	return ret;
}

/*
 * The factory address, which the loader copies from ADT /chosen
 * mac-address-wifi0 into the wifi0 alias target's local-mac-address (as on
 * T6050). A board without that node, or a loader that left the placeholder,
 * keeps the old per-boot random address.
 */
static void centauri_wifi_set_address(struct net_device *netdev)
{
	struct device_node *np = of_find_node_by_path("wifi0");
	u8 addr[ETH_ALEN];
	int ret = np ? of_get_mac_address(np, addr) : -ENODEV;

	of_node_put(np);
	if (!ret && is_valid_ether_addr(addr)) {
		eth_hw_addr_set(netdev, addr);
		return;
	}
	eth_hw_addr_random(netdev);
}

static const struct cfg80211_ops wifi_ops = {
	.scan = wifi_scan,
	.connect = wifi_connect,
	.disconnect = wifi_disconnect,
	.set_power_mgmt = wifi_power_mgmt,
	.get_station = wifi_get_station,
	.get_channel = wifi_get_channel,
};

static int net_open(struct net_device *netdev)
{
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);
	return 0;
}

static int net_stop(struct net_device *netdev)
{
	struct centauri_wifi *wifi = *(struct centauri_wifi **)netdev_priv(netdev);

	netif_stop_queue(netdev);
	skb_queue_purge(&wifi->data_tx_queue);
	/*
	 * Finish a running scan before cfg80211's NETDEV_DOWN. Otherwise
	 * cfg80211 completes it as driver-owned and stale, keeps its scan
	 * message, and refuses every later scan with -EBUSY.
	 */
	WRITE_ONCE(wifi->scan_abort, true);
	flush_work(&wifi->scan_work);
	WRITE_ONCE(wifi->scan_abort, false);
	return 0;
}

static netdev_tx_t net_transmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct centauri_wifi *wifi = *(struct centauri_wifi **)netdev_priv(netdev);

	if (READ_ONCE(wifi->transport_paused)) {
		netif_stop_queue(netdev);
		return NETDEV_TX_BUSY;
	}
	if (READ_ONCE(wifi->stopping) || !READ_ONCE(wifi->connected) ||
	    skb->len < ETH_HLEN || skb->len > BUFFER_SIZE) {
		netdev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	if (skb_queue_len(&wifi->data_tx_queue) >= DATA_TX_QUEUE_LIMIT) {
		netif_stop_queue(netdev);
		return NETDEV_TX_BUSY;
	}
	skb_queue_tail(&wifi->data_tx_queue, skb);
	if (skb_queue_len(&wifi->data_tx_queue) >= DATA_TX_QUEUE_LIMIT)
		netif_stop_queue(netdev);
	mod_delayed_work(system_percpu_wq, &wifi->link_work, 0);
	return NETDEV_TX_OK;
}

static const struct net_device_ops net_ops = {
	.ndo_open = net_open,
	.ndo_stop = net_stop,
	.ndo_start_xmit = net_transmit,
	.ndo_validate_addr = eth_validate_addr,
};

/* Airship direct_memregion::map (0x1802864e0) sends opcode 8, a region ID,
 * byte size and a 64-bit DVA in a 24-byte header. Alpha's region-debug is ID 1.
 * Firmware 0x175340 passes those fields to DramPhyService_handle_buffer;
 * InitDramLoggerMgr writes a 0x158-byte control header to the supplied buffer.
 * This is distinct from the fw_debug transfer ring. Never print log payloads:
 * firmware diagnostics may contain private connection material.
 */
static ssize_t fwlog_start_write(struct file *file, const char __user *input,
				size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = file->private_data;
	u8 message[24] = { 8, 0, 1, 0 };
	unsigned int action;
	unsigned long deadline;
	int ret;

	ret = kstrtouint_from_user(input, count, 0, &action);
	if (ret || action != 1)
		return ret ?: -EINVAL;
	mutex_lock(&wifi->lock);
	mutex_lock(&wifi->fwlog_lock);
	if (!wifi->configured || wifi->connecting || wifi->connected || wifi->scan_request) {
		ret = -EBUSY;
		goto out;
	}
	if (wifi->fwlog_attempted) {
		ret = -EALREADY;
		goto out;
	}
	ret = centauri_alpha_log_buffer(wifi->control, &wifi->fwlog_buffer,
					 &wifi->fwlog_dma, &wifi->fwlog_size);
	if (ret)
		goto out;
	put_unaligned_le32(wifi->fwlog_size, message + 4);
	put_unaligned_le64(wifi->fwlog_dma, message + 8);
	wifi->fwlog_attempted = true;
	ret = alpha_message(wifi, message, sizeof(message));
	if (ret)
		goto report;
	deadline = jiffies + msecs_to_jiffies(2000);
	do {
		if (readl(wifi->a.bar + 8) != 2) {
			ret = -EIO;
			goto report;
		}
		dma_rmb();
		if (get_unaligned_le16(wifi->fwlog_buffer) == 0x201) {
			wifi->fwlog_ready = true;
			goto report;
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	ret = -ETIMEDOUT;
report:
	dev_info(&wifi->control->pdev->dev,
		 "Wi-Fi DRAM logger: rc=%d initialized=%u stage=%u\n",
		 ret, wifi->fwlog_ready, readl(wifi->a.bar + 8));
out:
	mutex_unlock(&wifi->fwlog_lock);
	mutex_unlock(&wifi->lock);
	return ret ?: count;
}

static const struct file_operations fwlog_start_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = fwlog_start_write,
};

static ssize_t fwlog_status_read(struct file *file, char __user *output,
				size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = file->private_data;
	char text[320];
	u8 header[0xb4] = { 0 };
	int length;

	mutex_lock(&wifi->fwlog_lock);
	if (wifi->fwlog_buffer) {
		dma_rmb();
		memcpy(header, wifi->fwlog_buffer, sizeof(header));
	}
	length = scnprintf(text, sizeof(text),
		"attempted=%u initialized=%u bytes=%zu version=%#x blocks=%u block_size=%u mask=%#x data_offset=%u consumer=%u producer=%u header14=%u header18=%u header1c=%u connecting=%u scanning=%u\n",
		wifi->fwlog_attempted, wifi->fwlog_ready, wifi->fwlog_size,
		get_unaligned_le16(header), get_unaligned_le16(header + 2),
		get_unaligned_le32(header + 4), get_unaligned_le32(header + 8),
		get_unaligned_le32(header + 12), get_unaligned_le32(header + 16),
		get_unaligned_le32(header + 0xb0),
		get_unaligned_le32(header + 20), get_unaligned_le32(header + 24),
		get_unaligned_le32(header + 28),
		!!READ_ONCE(wifi->connecting), !!READ_ONCE(wifi->scan_request));
	mutex_unlock(&wifi->fwlog_lock);
	return simple_read_from_buffer(output, count, position, text, length);
}

static const struct file_operations fwlog_status_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = fwlog_status_read,
	.llseek = default_llseek,
};

/* The exact firmware's 0x144cfc compares header +0xb0 (producer) against
 * +0x10 (host consumer). InitDramLoggerMgr registers a GDMA read of +0x10.
 * Only acknowledge records after a private snapshot has been verified.
 * Leave three entries reserved for the firmware's three DMA descriptors
 * until completion/record sequencing is independently decoded.
 */
static ssize_t fwlog_ack_write(struct file *file, const char __user *input,
			       size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = file->private_data;
	u8 *header;
	u32 requested, consumer, producer, pending;
	u16 blocks;
	int ret;

	ret = kstrtou32_from_user(input, count, 0, &requested);
	if (ret)
		return ret;
	mutex_lock(&wifi->fwlog_lock);
	if (READ_ONCE(wifi->transport_paused)) {
		ret = -EBUSY;
		goto out;
	}
	if (!wifi->fwlog_ready || readl(wifi->a.bar + 8) != 2) {
		ret = -ENETDOWN;
		goto out;
	}
	header = wifi->fwlog_buffer;
	dma_rmb();
	blocks = get_unaligned_le16(header + 2);
	consumer = get_unaligned_le32(header + 0x10);
	producer = get_unaligned_le32(header + 0xb0);
	pending = producer - consumer;
	if (get_unaligned_le16(header) != 0x201 || blocks < 8 ||
	    pending > blocks || pending < 3 || requested - consumer > pending - 3) {
		ret = -EINVAL;
		goto out;
	}
	WRITE_ONCE(*(__le32 *)(header + 0x10), cpu_to_le32(requested));
	dma_wmb();
	ret = 0;
out:
	mutex_unlock(&wifi->fwlog_lock);
	return ret ?: count;
}

static const struct file_operations fwlog_ack_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = fwlog_ack_write,
};

/* Root-only binary diagnostics. Save directly to a protected artifact, never
 * the console or a text log. The device may update blocks during a read.
 */
static ssize_t fwlog_buffer_read(struct file *file, char __user *output,
				size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = file->private_data;
	ssize_t ret;

	mutex_lock(&wifi->fwlog_lock);
	if (!wifi->fwlog_ready) {
		ret = -ENODATA;
	} else {
		dma_rmb();
		ret = simple_read_from_buffer(output, count, position,
					    wifi->fwlog_buffer, wifi->fwlog_size);
	}
	mutex_unlock(&wifi->fwlog_lock);
	return ret;
}

static const struct file_operations fwlog_buffer_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = fwlog_buffer_read,
	.llseek = default_llseek,
};

/* The hardware data-ring callback requires the ten-byte WLAN clientdata
 * extension after the 44-byte ACIPC header. Without it UMAC asserts at
 * 0x11760e. Keep these empty-ring probes explicit until packet DMA is proven.
 * The reserved regions below do not overlap control rings or scan buffers.
 */
/* Called with wifi->lock held. From the connect path (internal) the "connecting" flag is expected to be set. */
static int data_probe_locked(struct centauri_wifi *wifi, unsigned int id, bool internal)
{
	u8 message[0x34 + 10] = { 0 };
	unsigned int vector, offset, size, bit, doorbell, irq;
	bool transfer = false;
	int ret;

	if (id == 76) {
		vector = 192;
		offset = 0x200000;
		size = 2048;
		bit = 0;
		doorbell = 18;
		irq = 3;
	} else if (id == 14) {
		vector = 206;
		offset = 0x220000;
		size = 256;
		bit = 1;
		doorbell = 11;
		irq = 8;
	} else if (id == 15) {
		vector = 207;
		offset = 0x240000;
		size = 128;
		bit = 2;
		doorbell = 11;
		irq = 8;
	} else if (id == 24 || id == 25 || id == 12) {
		transfer = true;
		vector = id;
		offset = id == 12 ? 0x520000 : id == 24 ? 0x500000 : 0x540000;
		size = 128;
		bit = id == 12 ? 4 : id == 24 ? 3 : 5;
		doorbell = id == 12 ? 11 : 18;
		irq = id == 12 ? 8 : 3;
	} else {
		return -EINVAL;
	}
	if (readl(wifi->a.bar + 8) != 2) {
		ret = -EIO;
		return ret;
	}
	if ((!internal && wifi->connecting) || wifi->scan_request ||
	    (wifi->data_probe_done & BIT(bit))) {
		ret = -EBUSY;
		return ret;
	}
	wifi->data_probe_done |= BIT(bit);
	memset(wifi->a.window + offset, 0, size * 32);
	index_write(wifi, transfer ? 1 : 0, vector, 0);
	index_write(wifi, transfer ? 3 : 2, vector, 0);
	message[0] = transfer ? 1 : 2;
	message[2] = transfer && id == 12 ? 0 : 4;
	put_unaligned_le16(id, message + 4);
	put_unaligned_le16(vector, message + 6);
	put_unaligned_le64(wifi->a.window_dma + offset, message + 8);
	if (transfer) {
		put_unaligned_le64(~0ULL, message + 0x10);
		put_unaligned_le16(size, message + 0x18);
		put_unaligned_le16(id == 12 ? 12 : 76, message + 0x1a);
		put_unaligned_le16(doorbell, message + 0x1c);
		put_unaligned_le16(id == 12 ? 0x220 : 0, message + 0x1e);
		put_unaligned_le32(16, message + 0x20);
		put_unaligned_le16(irq, message + 0x24);
	} else {
		put_unaligned_le16(size, message + 0x10);
		put_unaligned_le16(id == 15 ? 12 : 0xffff, message + 0x12);
		put_unaligned_le16(doorbell, message + 0x14);
		put_unaligned_le32(1, message + 0x18);
		put_unaligned_le16(irq, message + 0x1c);
	}
	/* The host MCBC transfer ring takes version 0, hardware
	 * interface 0, TID 6, AC 3, and
	 * six broadcast destination bytes. The default probe is unchanged.
	 */
	if (id == 25 && data_tx_broadcast) {
		message[0x34 + 2] = 6;
		message[0x34 + 3] = 3;
		eth_broadcast_addr(message + 0x34 + 4);
	}
	ret = alpha_message(wifi, message, (transfer ? 0x34 : 0x2c) + 10);
	msleep(1000);
	dev_info(&wifi->control->pdev->dev,
		 "data %s probe id=%u index=%u extension=10 rc=%d stage=%u head=%u tail=%u\n",
		 transfer ? "TR" : "CR", id, vector, ret, readl(wifi->a.bar + 8),
		 index_read(wifi, transfer ? 3 : 0, vector),
		 index_read(wifi, transfer ? 1 : 2, vector));
	if (!ret && readl(wifi->a.bar + 8) != 2)
		ret = -EIO;
	if (!ret && transfer && id == 12) {
		unsigned int slot;

		/* Give the receive engine bounded, individually identifiable buffers.
		 * No recycling or network delivery until the completion ABI is proven.
		 */
		for (slot = 0; slot < size - 1; slot++) {
			u8 *descriptor = wifi->a.window + offset + slot * 16;
			u32 buffer = DATA_RX_BUFFER + slot * 4096;

			memset(wifi->a.window + buffer, 0, 4096);
			put_unaligned_le32(1 | (4096 << 8), descriptor);
			put_unaligned_le64(wifi->a.window_dma + buffer, descriptor + 4);
			put_unaligned_le32(slot + 1, descriptor + 12);
		}
		dma_wmb();
		index_write(wifi, 3, vector, size - 1);
		dma_wmb();
		writel(1, wifi->a.bar + 0x1050);
		msleep(100);
		dev_info(&wifi->control->pdev->dev,
			 "data RX buffers posted=%u stage=%u tr_tail=%u cr_head=%u\n",
			 size - 1, readl(wifi->a.bar + 8), index_read(wifi, 1, 12),
			 index_read(wifi, 0, 207));
	}
	return ret;
}

static ssize_t data_probe_write(struct file *file, const char __user *input,
			       size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = file->private_data;
	unsigned int id;
	int ret;

	ret = kstrtouint_from_user(input, count, 0, &id);
	if (ret)
		return ret;
	mutex_lock(&wifi->lock);
	ret = data_probe_locked(wifi, id, false);
	mutex_unlock(&wifi->lock);
	return ret ?: count;
}

static int transmit_eapol_start(struct centauri_wifi *wifi)
{
	u16 head = index_read(wifi, 3, 24);
	u8 *descriptor = wifi->a.window + 0x500000 + head * 32;
	u8 *packet = wifi->a.window + 0xf00000 + head * 64;
	u64 metadata = 0;
	unsigned int attempt;

	if (READ_ONCE(wifi->transport_paused))
		return -EBUSY;
	if (readl(wifi->a.bar + 8) != 2 || !(wifi->data_probe_done & BIT(3)))
		return -ENETDOWN;
	/* Eight independent buffers bound the probe without ring reuse. */
	if (head >= 8 || index_read(wifi, 1, 24) != head)
		return -EBUSY;
	if (eapol_unicast && (!wifi->associated || !is_valid_ether_addr(wifi->bssid)))
		return -ENOTCONN;
	/* A standards-defined EAPOL-Start, with no credential/key material.
	 * Descriptor flags 3 mean a DMA buffer plus the 16-byte WLAN footer.
	 * Interface zero and default TX policy have a zero footer.
	 */
	memset(packet, 0, 64);
	if (eapol_unicast)
		memcpy(packet, wifi->bssid, ETH_ALEN);
	else
		memcpy(packet, "\x01\x80\xc2\x00\x00\x03", ETH_ALEN);
	memcpy(packet + ETH_ALEN, wifi->netdev->dev_addr, ETH_ALEN);
	put_unaligned_be16(ETH_P_PAE, packet + 12);
	packet[14] = 2;
	packet[15] = 1;
	memset(descriptor, 0, 32);
	put_unaligned_le32(3 | (18 << 8), descriptor);
	put_unaligned_le64(wifi->a.window_dma + 0xf00000 + head * 64, descriptor + 4);
	put_unaligned_le16(head + 1, descriptor + 12);
	if (eapol_control_profile)
		metadata |= BIT_ULL(60) | BIT_ULL(48);
	if (eapol_relative_lifetime) {
		/* A relative expiration time is mode 6 in
		 * bits 29:27 and microseconds in the second metadata word.
		 */
		metadata |= 6ULL << 27;
		put_unaligned_le64(2000000, descriptor + 24);
	}
	put_unaligned_le64(metadata, descriptor + 16);
	dma_wmb();
	index_write(wifi, 3, 24, head + 1);
	dma_wmb();
	writel(1, wifi->a.bar + 0x1030);
	for (attempt = 0; attempt < 20; attempt++) {
		if (index_read(wifi, 1, 24) == head + 1 || readl(wifi->a.bar + 8) != 2)
			break;
		usleep_range(5000, 6000);
	}
	dev_info(&wifi->control->pdev->dev,
		 "EAPOL-Start DMA probe: slot=%u associated=%d unicast=%d relative_expiry=%d stage=%u tr_tail=%u cr_head=%u\n",
		 head, wifi->associated, eapol_unicast, eapol_relative_lifetime, readl(wifi->a.bar + 8),
		 index_read(wifi, 1, 24), index_read(wifi, 0, 192));
	return readl(wifi->a.bar + 8) == 2 ? 0 : -EIO;
}

static ssize_t eapol_start_write(struct file *file, const char __user *input,
				size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = file->private_data;
	bool enabled;
	int ret;

	ret = kstrtobool_from_user(input, count, &enabled);
	if (ret || !enabled)
		return ret ?: -EINVAL;
	mutex_lock(&wifi->lock);
	if (wifi->connecting || wifi->scan_request) {
		ret = -EBUSY;
		goto out;
	}
	ret = transmit_eapol_start(wifi);
out:
	mutex_unlock(&wifi->lock);
	return ret ?: count;
}

static const struct file_operations eapol_start_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = eapol_start_write,
};

static const struct file_operations data_probe_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = data_probe_write,
};

static ssize_t data_status_read(struct file *file, char __user *output,
			       size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = file->private_data;
	char status[1536];
	unsigned int i, length;

	mutex_lock(&wifi->lock);
	dma_rmb();
	length = scnprintf(status, sizeof(status),
			   "stage=%u tx_cr=%u tx_tr=%u/%u rx_cr=%u rx_tr=%u/%u received=%u reposted=%u bad=%u samples=%u normal_tx_tr=%u/%u submitted=%u completed=%u failed=%u queued=%u coalesced=%u tx_bad=%u\n",
			   readl(wifi->a.bar + 8), index_read(wifi, 0, 192),
			   index_read(wifi, 1, 24), index_read(wifi, 3, 24),
			   index_read(wifi, 0, 207), index_read(wifi, 1, 12),
			   index_read(wifi, 3, 12), wifi->data_rx_received,
			   wifi->data_rx_reposted, wifi->data_rx_bad,
			   le32_to_cpu(wifi->rx_snapshot.count),
			   index_read(wifi, 1, 25), index_read(wifi, 3, 25),
			   wifi->data_tx_submitted, wifi->data_tx_completed,
			   wifi->data_tx_failed, skb_queue_len(&wifi->data_tx_queue),
			   wifi->data_tx_coalesced, wifi->data_tx_bad);
	for (i = 0; i < 8; i++) {
		u8 *entry = wifi->a.window + 0x240000 + i * 32;

		length += scnprintf(status + length, sizeof(status) - length,
			"rx_completion[%u]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
			i, get_unaligned_le32(entry), get_unaligned_le32(entry + 4),
			get_unaligned_le32(entry + 8), get_unaligned_le32(entry + 12),
			get_unaligned_le32(entry + 16), get_unaligned_le32(entry + 20),
			get_unaligned_le32(entry + 24), get_unaligned_le32(entry + 28));
	}
	for (i = 0; i < 8; i++) {
		u8 *entry = wifi->a.window + 0x200000 + i * 32;

		length += scnprintf(status + length, sizeof(status) - length,
			"tx_completion[%u]=%08x %08x %08x %08x %08x %08x %08x %08x\n",
			i, get_unaligned_le32(entry), get_unaligned_le32(entry + 4),
			get_unaligned_le32(entry + 8), get_unaligned_le32(entry + 12),
			get_unaligned_le32(entry + 16), get_unaligned_le32(entry + 20),
			get_unaligned_le32(entry + 24), get_unaligned_le32(entry + 28));
	}
	mutex_unlock(&wifi->lock);
	return simple_read_from_buffer(output, count, position, status, length);
}

static const struct file_operations data_status_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = data_status_read,
};

/* Contains private network traffic: retrieve only as a protected binary file.
 * Samples are copied once into host memory and never change after count 8.
 */
static ssize_t data_rx_snapshot_read(struct file *file, char __user *output,
				    size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = file->private_data;
	ssize_t ret;

	mutex_lock(&wifi->lock);
	ret = simple_read_from_buffer(output, count, position, &wifi->rx_snapshot,
				      sizeof(wifi->rx_snapshot));
	mutex_unlock(&wifi->lock);
	return ret;
}

static const struct file_operations data_rx_snapshot_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = data_rx_snapshot_read,
	.llseek = default_llseek,
};

static ssize_t country_status_read(struct file *file, char __user *output,
				  size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = file->private_data;
	char status[96];
	unsigned int length;
	int ret;

	mutex_lock(&wifi->lock);
	wifi->country_valid = false;
	ret = wifi->configured ? command(wifi, 10, NULL, 0) : -ENETDOWN;
	length = scnprintf(status, sizeof(status), "rc=%d valid=%d country_bytes=%02x/%02x/%02x\n",
			   ret, wifi->country_valid, wifi->country[0], wifi->country[1], wifi->country[2]);
	mutex_unlock(&wifi->lock);
	return simple_read_from_buffer(output, count, position, status, length);
}

static const struct file_operations country_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = country_status_read,
};

static int query_link_status(struct centauri_wifi *wifi)
{
	wifi->link_status_valid = wifi->emlsr_valid = false;
	wifi->link_count = wifi->link_status_size = 0;
	wifi->link_tlv_count = 0;
	return wifi->configured ? command(wifi, 20, NULL, 0) : -ENETDOWN;
}

static int link_status_show(struct seq_file *seq, void *unused)
{
	struct centauri_wifi *wifi = seq->private;
	unsigned int i;
	int ret;

	mutex_lock(&wifi->lock);
	ret = query_link_status(wifi);
	seq_printf(seq, "rc=%d valid=%d size=%u count=%u emlsr=%d/%u\n", ret,
		   wifi->link_status_valid, wifi->link_status_size, wifi->link_count,
		   wifi->emlsr_valid, wifi->emlsr_mode);
	for (i = 0; i < wifi->link_tlv_count; i++)
		seq_printf(seq, "tag=%u length=%u\n", wifi->link_tags[i], wifi->link_lengths[i]);
	if (wifi->link_status_valid)
		for (i = 0; i < wifi->link_count; i++)
			seq_printf(seq, "link=%u prime=%u channel=%u band=%u\n", i,
				   wifi->link_prime[i], wifi->link_channel[i], wifi->link_band[i]);
	mutex_unlock(&wifi->lock);
	return 0;
}

static int link_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, link_status_show, inode->i_private);
}

static const struct file_operations link_status_ops = {
	.owner = THIS_MODULE,
	.open = link_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int tx_config_show(struct seq_file *seq, void *unused)
{
	struct centauri_wifi *wifi = seq->private;
	u8 id;
	int ret;

	mutex_lock(&wifi->lock);
	seq_printf(seq, "applied=%u\n", wifi->tx_defaults_applied);
	for (id = 0; id <= 1; id++) {
		wifi->tx_profile_valid = false;
		ret = wifi->configured ? command(wifi, 56, &id, sizeof(id)) : -ENETDOWN;
		seq_printf(seq, "profile=%u rc=%d status=%u valid=%d", id, ret,
			   wifi->reply_status, wifi->tx_profile_valid);
		if (wifi->tx_profile_valid)
			seq_printf(seq, " rate=%#x rts_threshold=%u rts_retries=%u retries=%u type=%u id=%u",
				   get_unaligned_le32(wifi->tx_profile),
				   get_unaligned_le16(wifi->tx_profile + 4), wifi->tx_profile[6],
				   wifi->tx_profile[7], wifi->tx_profile[8], wifi->tx_profile[9]);
		seq_putc(seq, '\n');
	}
	mutex_unlock(&wifi->lock);
	return 0;
}

static int tx_config_open(struct inode *inode, struct file *file)
{
	return single_open(file, tx_config_show, inode->i_private);
}

static ssize_t tx_config_write(struct file *file, const char __user *input,
			       size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = ((struct seq_file *)file->private_data)->private;
	/* Exact configureTxDataCtrlPacketProfile: command 52, TLV 340/10.
	 * 0xa0154 is the TLV header, not the command ID.
	 */
	static const u8 control_profile[] = {
		0x54, 1, 10, 0, 0, 8, 0, 0, 0xff, 0xff, 0xff, 0xff, 1, 1,
	};
	u8 lifetime[20] = {};
	unsigned int action, i;
	int ret;

	ret = kstrtouint_from_user(input, count, 0, &action);
	if (ret || (action != 1 && action != 2))
		return ret ?: -EINVAL;
	mutex_lock(&wifi->lock);
	if (!wifi->configured || wifi->connecting || wifi->scan_request) {
		ret = -EBUSY;
		goto out;
	}
	if (action == 1) {
		/* The default TX packet lifetime is 2000 ms for each AC,
		 * sent to the firmware in microseconds.
		 */
		for (i = 0; i < 4; i++)
			put_unaligned_le32(2000000, lifetime + 4 + 4 * i);
		ret = command(wifi, 43, lifetime, sizeof(lifetime));
	} else {
		ret = command(wifi, 52, control_profile, sizeof(control_profile));
	}
	if (!ret)
		wifi->tx_defaults_applied |= BIT(action - 1);
	dev_info(&wifi->control->pdev->dev, "reference TX setup action=%u rc=%d\n", action, ret);
out:
	mutex_unlock(&wifi->lock);
	return ret ?: count;
}

static const struct file_operations tx_config_ops = {
	.owner = THIS_MODULE,
	.open = tx_config_open,
	.read = seq_read,
	.write = tx_config_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t host_wakeup_write(struct file *file, const char __user *input,
				size_t count, loff_t *position)
{
	struct centauri_wifi *wifi = file->private_data;
	/* Host wakeup is state 0 in command 6.
	 * This firmware's SysCmdSetHostPowerState accepts the fixed 27-byte
	 * request without TLVs. Keep its existing event filter: the newer host
	 * adapter's 92-byte event list exceeds this firmware's 86-byte limit.
	 */
	const u8 payload[3] = {};
	unsigned int action;
	int ret;

	ret = kstrtouint_from_user(input, count, 0, &action);
	if (ret || action != 1)
		return ret ?: -EINVAL;
	mutex_lock(&wifi->lock);
	if (!wifi->configured || wifi->connecting || wifi->scan_request || wifi->connected) {
		ret = -EBUSY;
		goto out;
	}
	ret = command(wifi, 6, payload, sizeof(payload));
	dev_info(&wifi->control->pdev->dev, "host wakeup state=0 rc=%d status=%u stage=%u\n",
		 ret, wifi->reply_status, readl(wifi->a.bar + 8));
out:
	mutex_unlock(&wifi->lock);
	return ret ?: count;
}

static const struct file_operations host_wakeup_ops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = host_wakeup_write,
};

static int query_station_stats(struct centauri_wifi *wifi)
{
	u8 payload[22] = {};
	int ret;

	/* Station counters: statistic bit 37, all peers.
	 * Its fixed counter record contains no key or packet payload bytes.
	 */
	put_unaligned_le64(BIT_ULL(37), payload);
	payload[8] = 1;
	put_unaligned_le16(363, payload + 9);
	put_unaligned_le16(9, payload + 11);
	wifi->station_stats_valid = false;
	wifi->stats_reply_size = wifi->stats_tag = wifi->stats_length = 0;
	ret = wifi->configured ? command(wifi, 0xb0001, payload, sizeof(payload)) : -ENETDOWN;
	if (!ret) {
		usleep_range(1000, 2000);
		drain(wifi);
	}
	return ret;
}

static int station_stats_show(struct seq_file *seq, void *unused)
{
	struct centauri_wifi *wifi = seq->private;
	unsigned int i;
	int ret;

	mutex_lock(&wifi->lock);
	ret = query_station_stats(wifi);
	seq_printf(seq, "rc=%d valid=%d size=%u tag=%#x length=%u\n", ret,
		   wifi->station_stats_valid, wifi->stats_reply_size, wifi->stats_tag, wifi->stats_length);
	if (!ret && wifi->station_stats_valid)
		for (i = 0; i < wifi->stats_length; i += 4) {
			u32 value = get_unaligned_le32(wifi->station_stats + i);

			if (value)
				seq_printf(seq, "%03x %u\n", i, value);
		}
	mutex_unlock(&wifi->lock);
	return 0;
}

static int station_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, station_stats_show, inode->i_private);
}

static const struct file_operations station_stats_ops = {
	.owner = THIS_MODULE,
	.open = station_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int rx_crypto_stats_show(struct seq_file *seq, void *unused)
{
	/* RX chip counters: statistic bit 30, response TLV 0xf921, sixteen LE32 counters.
	 * Unlike the station record, this fixed structure contains no MACs.
	 */
	static const char * const names[] = {
		"rx_duplicate_errors", "rx_ampdu_duplicate_errors",
		"rx_unicast_tkip_replay_errors", "rx_unicast_tkip_decrypt_errors",
		"rx_multicast_tkip_replay_errors", "rx_multicast_tkip_decrypt_errors",
		"rx_unicast_ccmp_replay_errors", "rx_unicast_ccmp_decrypt_errors",
		"rx_multicast_ccmp_replay_errors", "rx_multicast_ccmp_decrypt_errors",
		"phy_total_detect_count", "phy_ofdm_detect_count", "phy_11b_detect_count",
		"phy_plcp_error_count", "phy_total_receive_count", "phy_plcp_good_count",
	};
	struct centauri_wifi *wifi = seq->private;
	u8 payload[22] = {};
	unsigned int i;
	int ret;

	put_unaligned_le64(BIT_ULL(30), payload);
	payload[8] = 1;
	put_unaligned_le16(363, payload + 9);
	put_unaligned_le16(9, payload + 11);
	mutex_lock(&wifi->lock);
	wifi->rx_crypto_stats_valid = false;
	wifi->stats_reply_size = wifi->stats_tag = wifi->stats_length = 0;
	ret = wifi->configured ? command(wifi, 0xb0001, payload, sizeof(payload)) : -ENETDOWN;
	if (!ret) {
		usleep_range(1000, 2000);
		drain(wifi);
	}
	seq_printf(seq, "rc=%d valid=%d size=%u tag=%#x length=%u\n", ret,
		   wifi->rx_crypto_stats_valid, wifi->stats_reply_size, wifi->stats_tag, wifi->stats_length);
	if (!ret && wifi->rx_crypto_stats_valid)
		for (i = 0; i < ARRAY_SIZE(names); i++)
			seq_printf(seq, "%s=%u\n", names[i],
				   get_unaligned_le32(wifi->rx_crypto_stats + i * 4));
	mutex_unlock(&wifi->lock);
	return 0;
}

static int rx_crypto_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, rx_crypto_stats_show, inode->i_private);
}

static const struct file_operations rx_crypto_stats_ops = {
	.owner = THIS_MODULE,
	.open = rx_crypto_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dot11_mib_stats_show(struct seq_file *seq, void *unused)
{
	/* 802.11 MIB counters: statistic bit 25, scope 2/interface 0, TLV 0xf91b, 24 LE64s.
	 * Keep the host field names: tx_status_success is firmware accounting,
	 * not independent proof of over-the-air acknowledgement or AP delivery.
	 */
	static const char * const names[] = {
		"rx_msdu_pushed", "rx_msdu_pushed_multi", "rx_mpdu", "rx_mpdu_dup",
		"rx_ampdu_dup", "rx_mpdu_drop_error", "rx_msdu_drop_filter",
		"rx_holes_0", "rx_holes_1", "rx_holes_2", "rx_holes_3", "rx_holes_4",
		"rx_holes_5", "rx_holes_6", "rx_holes_7", "rx_holes_8",
		"tx_pulled", "tx_pulled_multi", "tx_pulled_core", "tx_status_success",
		"tx_status_abort_request", "tx_status_peer_gone",
		"tx_status_retries_exhausted", "tx_status_lifetime_expired",
	};
	struct centauri_wifi *wifi = seq->private;
	u8 payload[22] = {};
	unsigned int i;
	int ret;

	put_unaligned_le64(BIT_ULL(25), payload);
	payload[8] = 1;
	put_unaligned_le16(363, payload + 9);
	put_unaligned_le16(9, payload + 11);
	/* The scope byte follows the eight-byte interface value in TLV 363. */
	payload[21] = 2;
	mutex_lock(&wifi->lock);
	wifi->dot11_mib_stats_valid = false;
	wifi->stats_reply_size = wifi->stats_tag = wifi->stats_length = 0;
	ret = wifi->configured ? command(wifi, 0xb0001, payload, sizeof(payload)) : -ENETDOWN;
	if (!ret) {
		usleep_range(1000, 2000);
		drain(wifi);
	}
	seq_printf(seq, "rc=%d valid=%d size=%u tag=%#x length=%u\n", ret,
		   wifi->dot11_mib_stats_valid, wifi->stats_reply_size, wifi->stats_tag, wifi->stats_length);
	if (!ret && wifi->dot11_mib_stats_valid)
		for (i = 0; i < ARRAY_SIZE(names); i++)
			seq_printf(seq, "%s=%llu\n", names[i],
				   (unsigned long long)get_unaligned_le64(wifi->dot11_mib_stats + i * 8));
	mutex_unlock(&wifi->lock);
	return 0;
}

static int dot11_mib_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, dot11_mib_stats_show, inode->i_private);
}

static const struct file_operations dot11_mib_stats_ops = {
	.owner = THIS_MODULE,
	.open = dot11_mib_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int tx_power_report_show(struct seq_file *seq, void *unused)
{
	/* Last TX stats: bit 29, scope 1/radio bitmap, TLV 0xf920 with 280
	 * bytes on this firmware. Newer firmware returns 320 bytes with a
	 * different record stride. The first five 28-byte records are read.
	 * These are rate/power observations, not proof of ACK or AP delivery.
	 */
	struct centauri_wifi *wifi = seq->private;
	u8 payload[22] = {};
	unsigned int radio, i;
	int ret;

	put_unaligned_le64(BIT_ULL(29), payload);
	payload[8] = 1;
	put_unaligned_le16(363, payload + 9);
	put_unaligned_le16(9, payload + 11);
	payload[21] = 1;
	mutex_lock(&wifi->lock);
	for (radio = 1; radio <= 2; radio++) {
		put_unaligned_le64(radio, payload + 13);
		wifi->tx_power_report_valid = false;
		wifi->stats_reply_size = wifi->stats_tag = wifi->stats_length = 0;
		ret = wifi->configured ? command(wifi, 0xb0001, payload, sizeof(payload)) : -ENETDOWN;
		if (!ret) {
			usleep_range(1000, 2000);
			drain(wifi);
		}
		seq_printf(seq, "radio_bitmap=%u rc=%d valid=%d size=%u tag=%#x length=%u\n",
			   radio, ret, wifi->tx_power_report_valid, wifi->stats_reply_size,
			   wifi->stats_tag, wifi->stats_length);
		if (ret || !wifi->tx_power_report_valid)
			continue;
		for (i = 0; i < 5; i++) {
			const u8 *record = wifi->tx_power_report + i * 28;

			/* Keep hardware enums and power bytes raw until interpreted
			 * against this exact firmware; omit unreviewed record fields.
			 */
			seq_printf(seq, "sample=%u power0_qdb_raw=%u power1_qdb_raw=%u power_qdb_raw=%u "
				   "hw_preamble=%u hw_rate=%u nss=%u hw_bw=%u hw_gi=%u\n",
				   i, record[14], record[15], record[16], record[17],
				   record[20], record[21] + 1, record[24], record[27]);
		}
	}
	mutex_unlock(&wifi->lock);
	return 0;
}

static int tx_power_report_open(struct inode *inode, struct file *file)
{
	return single_open(file, tx_power_report_show, inode->i_private);
}

static const struct file_operations tx_power_report_ops = {
	.owner = THIS_MODULE,
	.open = tx_power_report_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int centauri_wifi_register(struct centauri *c)
{
	struct centauri_wifi *wifi;
	unsigned int i;
	int ret;

	wifi = kzalloc(sizeof(*wifi), GFP_KERNEL);
	if (!wifi)
		return -ENOMEM;
	c->wifi = wifi;
	wifi->control = c;
	mutex_init(&wifi->lock);
	mutex_init(&wifi->fwlog_lock);
	skb_queue_head_init(&wifi->data_tx_queue);
	INIT_WORK(&wifi->scan_work, scan_worker);
	INIT_WORK(&wifi->connect_work, connect_worker);
	INIT_DELAYED_WORK(&wifi->link_work, link_worker);
	ret = centauri_alpha_resources(c, &wifi->a);
	if (ret)
		goto failed;
	wifi->wiphy = wiphy_new(&wifi_ops, sizeof(struct centauri_wifi *));
	wifi->netdev = alloc_etherdev(sizeof(struct centauri_wifi *));
	if (!wifi->wiphy || !wifi->netdev) {
		ret = -ENOMEM;
		goto failed;
	}
	*(struct centauri_wifi **)wiphy_priv(wifi->wiphy) = wifi;
	*(struct centauri_wifi **)netdev_priv(wifi->netdev) = wifi;
	set_wiphy_dev(wifi->wiphy, &c->pdev->dev);
	wifi->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	wifi->wiphy->max_scan_ssids = 1;
	/*
	 * cfg80211 refuses a scan whose IEs exceed this. wpa_supplicant 2.11
	 * adds 15 bytes to every scan (Extended Capabilities and an empty
	 * Mesh ID); 512, wl1251's limit for the same reason, leaves room for
	 * optional ones such as WPS or Interworking. build_scan() follows the
	 * macOS request, which carries no IEs, so they are dropped.
	 */
	wifi->wiphy->max_scan_ie_len = 512;
	wifi->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wifi->wiphy->reg_notifier = wifi_regulatory;
	wifi->wiphy->cipher_suites = cipher_suites;
	wifi->wiphy->n_cipher_suites = ARRAY_SIZE(cipher_suites);
	wiphy_ext_feature_set(wifi->wiphy, NL80211_EXT_FEATURE_4WAY_HANDSHAKE_STA_PSK);
	for (i = 0; i < 13; i++) {
		wifi->channels[i].band = NL80211_BAND_2GHZ;
		wifi->channels[i].hw_value = i + 1;
		wifi->channels[i].center_freq = ieee80211_channel_to_frequency(i + 1, NL80211_BAND_2GHZ);
		wifi->channels[i].max_power = 20;
	}
	for (i = 0; i < ARRAY_SIZE(five_ghz); i++) {
		wifi->channels[13 + i].band = NL80211_BAND_5GHZ;
		wifi->channels[13 + i].hw_value = five_ghz[i];
		wifi->channels[13 + i].center_freq = ieee80211_channel_to_frequency(five_ghz[i], NL80211_BAND_5GHZ);
		wifi->channels[13 + i].max_power = 20;
	}
	wifi->bands[0].channels = wifi->channels;
	wifi->bands[0].n_channels = 13;
	wifi->bands[0].bitrates = rates;
	wifi->bands[0].n_bitrates = ARRAY_SIZE(rates);
	wifi->bands[1].channels = wifi->channels + 13;
	wifi->bands[1].n_channels = ARRAY_SIZE(five_ghz);
	wifi->bands[1].bitrates = rates + 4;
	wifi->bands[1].n_bitrates = ARRAY_SIZE(rates) - 4;
	wifi->wiphy->bands[NL80211_BAND_2GHZ] = &wifi->bands[0];
	wifi->wiphy->bands[NL80211_BAND_5GHZ] = &wifi->bands[1];
	ret = wiphy_register(wifi->wiphy);
	if (ret)
		goto failed;
	wifi->registered = true;
	wifi->wdev.wiphy = wifi->wiphy;
	wifi->wdev.iftype = NL80211_IFTYPE_STATION;
	wifi->wdev.netdev = wifi->netdev;
	wifi->netdev->ieee80211_ptr = &wifi->wdev;
	wifi->netdev->netdev_ops = &net_ops;
	SET_NETDEV_DEV(wifi->netdev, &c->pdev->dev);
	strscpy(wifi->netdev->name, "wlan%d", IFNAMSIZ);
	wifi->netdev->max_mtu = ETH_DATA_LEN;
	centauri_wifi_set_address(wifi->netdev);
	ret = register_netdev(wifi->netdev);
	if (ret)
		goto failed;
	ret = radio_start(wifi);
	if (ret) {
		dev_err(&c->pdev->dev, "Wi-Fi ring/interface setup failed: %d\n", ret);
		goto failed;
	}
	dev_info(&c->pdev->dev, "cfg80211 interface %s ready for scan tests; packet data unavailable\n",
		 wifi->netdev->name);
	wifi->data_probe_file = debugfs_create_file("data_probe", 0200, c->debug, wifi, &data_probe_ops);
	wifi->data_status_file = debugfs_create_file("data_status", 0400, c->debug, wifi, &data_status_ops);
	wifi->data_rx_snapshot_file = debugfs_create_file("data_rx_snapshot", 0400,
		c->debug, wifi, &data_rx_snapshot_ops);
	wifi->station_stats_file = debugfs_create_file("station_stats", 0400, c->debug, wifi, &station_stats_ops);
	wifi->rx_crypto_stats_file = debugfs_create_file("rx_crypto_stats", 0400,
		c->debug, wifi, &rx_crypto_stats_ops);
	wifi->dot11_mib_stats_file = debugfs_create_file("dot11_mib_stats", 0400,
		c->debug, wifi, &dot11_mib_stats_ops);
	wifi->tx_power_report_file = debugfs_create_file("tx_power_report", 0400,
		c->debug, wifi, &tx_power_report_ops);
	wifi->eapol_start_file = debugfs_create_file("eapol_start", 0200, c->debug, wifi, &eapol_start_ops);
	wifi->country_file = debugfs_create_file("country", 0400, c->debug, wifi, &country_ops);
	wifi->link_status_file = debugfs_create_file("link_status", 0400, c->debug, wifi, &link_status_ops);
	wifi->tx_config_file = debugfs_create_file("tx_config", 0600, c->debug, wifi, &tx_config_ops);
	wifi->host_wakeup_file = debugfs_create_file("host_wakeup", 0200, c->debug, wifi, &host_wakeup_ops);
	wifi->fwlog_start_file = debugfs_create_file("fwlog_start", 0200, c->debug, wifi, &fwlog_start_ops);
	wifi->fwlog_status_file = debugfs_create_file("fwlog_status", 0400, c->debug, wifi, &fwlog_status_ops);
	wifi->fwlog_buffer_file = debugfs_create_file("fwlog_buffer", 0400, c->debug, wifi, &fwlog_buffer_ops);
	wifi->fwlog_ack_file = debugfs_create_file("fwlog_ack", 0200, c->debug, wifi, &fwlog_ack_ops);
	return 0;
failed:
	centauri_wifi_unregister(c);
	return ret;
}

/*
 * 26B123 t2026-CentauriAlpha uses BAR0 + 0x1024 for sleep_control.
 * AirshipDK's ACIPC host-sleep handshake writes 3 and waits for status 3;
 * wake writes 0 and waits for status 0 or 1. The peripheral-info block
 * starts at context + 0x68, with sleep_status at +8, as on Beta.
 * Stop every ring producer before requesting host sleep. Keep them stopped
 * until the wake acknowledgment, including when suspend entry is refused.
 */
static u32 alpha_sleep_status(struct centauri_wifi *wifi)
{
	return le32_to_cpu(READ_ONCE(*(__le32 *)(wifi->a.context + 0x70)));
}

static int alpha_transport_state(struct centauri_wifi *wifi, bool asleep)
{
	u32 status;
	int ret;

	lockdep_assert_held(&wifi->lock);
	if (readl(wifi->a.bar + 8) != 2 || readl(wifi->a.bar + 0xc) != 2)
		return -EIO;
	dma_wmb();
	writel(asleep ? 3 : 0, wifi->a.bar + 0x1024);
	ret = read_poll_timeout(alpha_sleep_status, status,
				(asleep ? status == 3 : status <= 1) ||
				readl(wifi->a.bar + 8) != 2,
				100, 2000000, false, wifi);
	dma_rmb();
	if (readl(wifi->a.bar + 8) != 2)
		ret = -EIO;
	dev_info(&wifi->control->pdev->dev,
		 "TEST: Alpha ACIPC host %s: ret=%d status=%u exec=%u ipc=%u\n",
		 asleep ? "sleep" : "wake", ret, status,
		 readl(wifi->a.bar + 8), readl(wifi->a.bar + 0xc));
	return ret;
}

/*
 * WLANACIAdapter sends command 6 before ACIPC host sleep. For an interface
 * already disconnected by cfg80211, its LPNA mode is firmware state 3.
 * The first fields are le16 state and u8 hint. Native sleep also includes
 * TLV 303 (value 1) and TLV 280 (the filtered event list); wake restores all
 * events. Installed Alpha 48ef6696 rejects TLV 280 lengths >= 87 and
 * event counts >= 86, so use its 85 events rather than 26B123's 91.
 * The data-ring producers stay paused while this control command
 * and its response use the command/management rings.
 */
static int alpha_host_power(struct centauri_wifi *wifi, bool asleep)
{
	static const u8 sleep_events[] = { 24, 26, 27, 15, 51, 8, 9, 10, 84 };
	u8 payload[98] = {}, events[86] = {}, sleep_option = 1;
	size_t size = 3;
	int i, ret;

	lockdep_assert_held(&wifi->lock);
	if (readl(wifi->a.bar + 8) != 2)
		return -EIO;
	if (asleep) {
		put_unaligned_le16(3, payload);
		payload[2] = 3;
		append_tlv(payload, &size, 303, &sleep_option, sizeof(sleep_option));
		events[0] = ARRAY_SIZE(sleep_events);
		memcpy(events + 1, sleep_events, sizeof(sleep_events));
	} else {
		events[0] = sizeof(events) - 1;
		for (i = 0; i < events[0]; i++)
			events[i + 1] = i;
	}
	append_tlv(payload, &size, 280, events, sizeof(events));
	ret = command_control(wifi, 6, payload, size, true);
	dev_info(&wifi->control->pdev->dev,
		 "TEST: Alpha firmware host %s: ret=%d status=%u exec=%u\n",
		 asleep ? "LPNA" : "awake", ret, wifi->reply_status,
		 readl(wifi->a.bar + 8));
	return ret;
}

static int alpha_resume_locked(struct centauri_wifi *wifi)
{
	int ret;

	lockdep_assert_held(&wifi->lock);
	if (!wifi->transport_sleep_sent && !wifi->host_sleep_sent)
		return 0;
	if (wifi->transport_sleep_sent) {
		ret = alpha_transport_state(wifi, false);
		if (ret)
			return ret;
		wifi->transport_sleep_sent = false;
	}
	if (wifi->host_sleep_sent) {
		ret = alpha_host_power(wifi, false);
		if (ret)
			return ret;
		wifi->host_sleep_sent = false;
	}
	mutex_lock(&wifi->fwlog_lock);
	WRITE_ONCE(wifi->transport_paused, false);
	mutex_unlock(&wifi->fwlog_lock);
	if (wifi->connected && !READ_ONCE(wifi->stopping)) {
		if (netif_running(wifi->netdev))
			netif_wake_queue(wifi->netdev);
		mod_delayed_work(system_percpu_wq, &wifi->link_work, 0);
	}
	return 0;
}

int centauri_wifi_resume(struct centauri *c)
{
	struct centauri_wifi *wifi = c->wifi;
	int ret;

	if (!wifi || !of_machine_is_compatible("apple,j813"))
		return 0;
	mutex_lock(&wifi->lock);
	ret = alpha_resume_locked(wifi);
	mutex_unlock(&wifi->lock);
	return ret;
}

int centauri_wifi_suspend(struct centauri *c)
{
	struct centauri_wifi *wifi = c->wifi;
	int ret, restore;

	if (!wifi || !of_machine_is_compatible("apple,j813") ||
	    !READ_ONCE(alpha_sleep_untested))
		return 0;
	mutex_lock(&wifi->lock);
	if (wifi->transport_paused || wifi->connected || wifi->connecting || wifi->scan_request ||
	    READ_ONCE(wifi->stopping)) {
		mutex_unlock(&wifi->lock);
		return -EBUSY;
	}
	mutex_lock(&wifi->fwlog_lock);
	WRITE_ONCE(wifi->transport_paused, true);
	mutex_unlock(&wifi->fwlog_lock);
	mutex_unlock(&wifi->lock);
	netif_tx_disable(wifi->netdev);
	cancel_work_sync(&wifi->scan_work);
	cancel_work_sync(&wifi->connect_work);
	cancel_delayed_work_sync(&wifi->link_work);

	mutex_lock(&wifi->lock);
	wifi->host_sleep_sent = true;
	ret = alpha_host_power(wifi, true);
	if (!ret) {
		wifi->transport_sleep_sent = true;
		ret = alpha_transport_state(wifi, true);
	}
	if (ret) {
		restore = alpha_resume_locked(wifi);
		if (restore)
			dev_err(&c->pdev->dev,
				"Alpha transport remains paused after refused sleep: %d\n",
				restore);
	}
	mutex_unlock(&wifi->lock);
	return ret;
}

void centauri_wifi_unregister(struct centauri *c)
{
	struct centauri_wifi *wifi = c->wifi;

	if (!wifi)
		return;
	WRITE_ONCE(wifi->stopping, true);
	if (wifi->netdev && wifi->netdev->reg_state == NETREG_REGISTERED)
		netif_tx_disable(wifi->netdev);
	debugfs_remove(wifi->data_probe_file);
	debugfs_remove(wifi->data_status_file);
	debugfs_remove(wifi->data_rx_snapshot_file);
	debugfs_remove(wifi->station_stats_file);
	debugfs_remove(wifi->rx_crypto_stats_file);
	debugfs_remove(wifi->dot11_mib_stats_file);
	debugfs_remove(wifi->tx_power_report_file);
	debugfs_remove(wifi->eapol_start_file);
	debugfs_remove(wifi->country_file);
	debugfs_remove(wifi->link_status_file);
	debugfs_remove(wifi->tx_config_file);
	debugfs_remove(wifi->host_wakeup_file);
	debugfs_remove(wifi->fwlog_start_file);
	debugfs_remove(wifi->fwlog_status_file);
	debugfs_remove(wifi->fwlog_buffer_file);
	debugfs_remove(wifi->fwlog_ack_file);
	cancel_work_sync(&wifi->scan_work);
	cancel_work_sync(&wifi->connect_work);
	cancel_delayed_work_sync(&wifi->link_work);
	skb_queue_purge(&wifi->data_tx_queue);
	if (wifi->scan_request) {
		struct cfg80211_scan_info info = { .aborted = true };

		cfg80211_scan_done(wifi->scan_request, &info);
		wifi->scan_request = NULL;
	}
	if (wifi->netdev && wifi->netdev->reg_state == NETREG_REGISTERED)
		unregister_netdev(wifi->netdev);
	if (wifi->registered)
		wiphy_unregister(wifi->wiphy);
	if (wifi->wiphy)
		wiphy_free(wifi->wiphy);
	if (wifi->netdev)
		free_netdev(wifi->netdev);
	memzero_explicit(wifi, sizeof(*wifi));
	kfree(wifi);
	c->wifi = NULL;
}
