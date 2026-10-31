// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the validation of firmware input: RX descriptors and
 * command replies, plus the directed-SSID slots of scan requests. The
 * firmware and the device are treated as untrusted.
 */
#include <kunit/test.h>

#include "mt7932.h"

#define RX_HEADER 24
#define RX_ETHERNET 14
#define RX_PAYLOAD 40
#define RX_SIZE (RX_HEADER + RX_ETHERNET + RX_PAYLOAD)

/* A translated (Ethernet) unicast data frame on channel 36 for WTBL 5. */
static void rx_build(u8 *packet, u32 extra_w1)
{
	memset(packet, 0, RX_SIZE);
	put_unaligned_le32(RX_SIZE | (2U << 27), packet);
	put_unaligned_le32(5 | extra_w1, packet + 4);
	put_unaligned_le32(BIT(13), packet + 8);
	put_unaligned_le32((36U << 8) | (1U << 16), packet + 12);
	memset(packet + RX_HEADER, 0xa5, RX_ETHERNET + RX_PAYLOAD);
	put_unaligned_be16(ETH_P_IP, packet + RX_HEADER + 12);
}

static u32 rx_cipher(unsigned int mode)
{
	return mode << MT7932_RXD1_SEC_MODE_SHIFT;
}

static void mt7932_rx_decrypted_test(struct kunit *test)
{
	struct mt7932_rx_frame frame;
	u8 packet[RX_SIZE];

	rx_build(packet, rx_cipher(4));
	KUNIT_ASSERT_EQ(test, mt7932_rx_frame(packet, sizeof(packet), &frame), 0);
	KUNIT_EXPECT_TRUE(test, frame.translated);
	KUNIT_EXPECT_EQ(test, frame.sec_mode, 4);
	KUNIT_EXPECT_EQ(test, frame.wtbl, 5);
	KUNIT_EXPECT_EQ(test, frame.length, (size_t)(RX_ETHERNET + RX_PAYLOAD));
	KUNIT_EXPECT_TRUE(test, mt7932_rx_protection_ok(&frame, false));
	KUNIT_EXPECT_TRUE(test, mt7932_rx_protection_ok(&frame, true));
}

/* A plaintext frame on a protected connection is never delivered. */
static void mt7932_rx_plaintext_test(struct kunit *test)
{
	struct mt7932_rx_frame frame;
	u8 packet[RX_SIZE];

	rx_build(packet, 0);
	KUNIT_ASSERT_EQ(test, mt7932_rx_frame(packet, sizeof(packet), &frame), 0);
	KUNIT_EXPECT_EQ(test, frame.sec_mode, 0);
	KUNIT_EXPECT_FALSE(test, mt7932_rx_protection_ok(&frame, false));
	/* An open connection has nothing to decrypt. */
	KUNIT_EXPECT_TRUE(test, mt7932_rx_protection_ok(&frame, true));
}

/* Decrypted with the wrong cipher or key length: refused on WPA2. */
static void mt7932_rx_mismatch_test(struct kunit *test)
{
	struct mt7932_rx_frame frame;
	u8 packet[RX_SIZE];

	rx_build(packet, rx_cipher(4) | MT7932_RXD1_CIPHER_LEN_MISMATCH);
	KUNIT_ASSERT_EQ(test, mt7932_rx_frame(packet, sizeof(packet), &frame), 0);
	KUNIT_EXPECT_TRUE(test, frame.cipher_mismatch);
	KUNIT_EXPECT_FALSE(test, mt7932_rx_protection_ok(&frame, false));
	/* A translated frame with a cipher mismatch is rejected by the parser. */
	rx_build(packet, rx_cipher(4) | MT7932_RXD1_CIPHER_MISMATCH);
	KUNIT_EXPECT_EQ(test, mt7932_rx_frame(packet, sizeof(packet), &frame), -EBADMSG);
}

/* ICV, TKIP MIC and FCS errors are rejected before any field is used. */
static void mt7932_rx_integrity_test(struct kunit *test)
{
	struct mt7932_rx_frame frame;
	u8 packet[RX_SIZE];
	unsigned int bit;

	for (bit = 25; bit <= 27; bit++) {
		rx_build(packet, rx_cipher(4) | BIT(bit));
		KUNIT_EXPECT_EQ(test, mt7932_rx_frame(packet, sizeof(packet), &frame),
				-EBADMSG);
	}
}

/* Short buffers and length fields beyond the buffer never parse. */
static void mt7932_rx_short_test(struct kunit *test)
{
	struct mt7932_rx_frame frame;
	u8 packet[RX_SIZE], *copy;
	size_t cut;

	rx_build(packet, rx_cipher(4));
	for (cut = 0; cut < RX_HEADER + RX_ETHERNET; cut++) {
		/* An exact-size copy lets KASAN catch any read past the end. */
		copy = kmemdup(packet, cut, GFP_KERNEL);
		KUNIT_ASSERT_TRUE(test, copy || !cut);
		if (cut >= 4)
			put_unaligned_le32(cut | (2U << 27), copy);
		KUNIT_EXPECT_LT_MSG(test, mt7932_rx_frame(copy, cut, &frame), 0,
				    "length %zu", cut);
		kfree(copy);
	}
	/* The descriptor claims more than the DMA buffer holds. */
	copy = kmemdup(packet, RX_SIZE - 1, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, copy);
	KUNIT_EXPECT_EQ(test, mt7932_rx_frame(copy, RX_SIZE - 1, &frame), -EMSGSIZE);
	kfree(copy);
	/* Optional groups (3 and 5) that do not fit. */
	rx_build(packet, rx_cipher(4) | BIT(13) | BIT(15));
	put_unaligned_le32((RX_HEADER + 20) | (2U << 27), packet);
	KUNIT_EXPECT_EQ(test, mt7932_rx_frame(packet, RX_HEADER + 20, &frame), -EMSGSIZE);
}

static void event_build(u8 *packet, u8 eid, u8 seq, u8 option)
{
	memset(packet, 0, 64);
	put_unaligned_le32(64 | (7U << 27), packet);
	put_unaligned_le16(40, packet + 24);
	packet[28] = eid;
	packet[29] = seq;
	packet[30] = option;
}

/* Only the event that answers a command completes it. */
static void mt7932_event_reply_test(struct kunit *test)
{
	struct mt7932_event event;
	u8 packet[64];
	int eid = mt7932_reply_eid(0xed, true);

	event_build(packet, 0xed, 9, 0);
	KUNIT_ASSERT_EQ(test, mt7932_event_parse(packet, sizeof(packet), &event), 0);
	KUNIT_EXPECT_TRUE(test, mt7932_event_matches(&event, 9, eid));
	/* Another sequence, the no-reply option bit, or nothing waiting. */
	KUNIT_EXPECT_FALSE(test, mt7932_event_matches(&event, 10, eid));
	event.option = 4;
	KUNIT_EXPECT_FALSE(test, mt7932_event_matches(&event, 9, eid));
	event.option = 0;
	KUNIT_EXPECT_FALSE(test, mt7932_event_matches(&event, 0, eid));
	/* An unrelated event that happens to carry the waiting sequence. */
	event_build(packet, 0x99, 9, 0);
	KUNIT_ASSERT_EQ(test, mt7932_event_parse(packet, sizeof(packet), &event), 0);
	KUNIT_EXPECT_FALSE(test, mt7932_event_matches(&event, 9, eid));
	KUNIT_EXPECT_FALSE(test, mt7932_event_matches(&event, 9, mt7932_reply_eid(0x8a, true)));
	KUNIT_EXPECT_FALSE(test, mt7932_event_matches(&event, 9, mt7932_reply_eid(0x03, false)));
	/* The ROM's patch semaphore answers with its own event only. */
	KUNIT_EXPECT_FALSE(test, mt7932_event_matches(&event, 9, mt7932_reply_eid(0x10, false)));
	event_build(packet, 0x04, 9, 0);
	KUNIT_ASSERT_EQ(test, mt7932_event_parse(packet, sizeof(packet), &event), 0);
	KUNIT_EXPECT_TRUE(test, mt7932_event_matches(&event, 9, mt7932_reply_eid(0x10, false)));
	event_build(packet, 0x01, 9, 0);
	KUNIT_ASSERT_EQ(test, mt7932_event_parse(packet, sizeof(packet), &event), 0);
	KUNIT_EXPECT_FALSE(test, mt7932_event_matches(&event, 9, mt7932_reply_eid(0x10, false)));
}

static void mt7932_event_reply_eid_test(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, mt7932_reply_eid(0xed, false), 0xed);
	KUNIT_EXPECT_EQ(test, mt7932_reply_eid(0xed, true), 0xed);
	KUNIT_EXPECT_EQ(test, mt7932_reply_eid(0x8a, true), 0xec);
	KUNIT_EXPECT_EQ(test, mt7932_reply_eid(0xbf, true), 0xbf);
	KUNIT_EXPECT_EQ(test, mt7932_reply_eid(0x03, false), 0x02);
	KUNIT_EXPECT_EQ(test, mt7932_reply_eid(0x01, false), 0x01);
	KUNIT_EXPECT_EQ(test, mt7932_reply_eid(0x05, false), 0x01);
	KUNIT_EXPECT_EQ(test, mt7932_reply_eid(0x10, false), 0x04);
	KUNIT_EXPECT_EQ(test, mt7932_reply_eid(0x50, false), -1);
}

static struct mt7932_rx_frame rx_qos_frame(void)
{
	return (struct mt7932_rx_frame) {
		.translated = true,
		.group4 = true,
		.unicast = true,
		.frame_control = IEEE80211_FTYPE_DATA | IEEE80211_STYPE_QOS_DATA,
		.sequence_control = 0x1230,
		.qos_control = 3,
		.sec_mode = 4,
	};
}

static void mt7932_rx_reject_preserves_sequence_test(struct kunit *test)
{
	struct mt7932_duplicate cache[17] = {}, before[17];
	struct mt7932_rx_frame frame = rx_qos_frame();

	KUNIT_ASSERT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_ACCEPT);
	memcpy(before, cache, sizeof(cache));
	frame.sequence_control += 0x10;
	frame.sec_mode = 0;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_UNPROTECTED);
	KUNIT_EXPECT_MEMEQ(test, cache, before, sizeof(cache));
	frame.sec_mode = 4;
	frame.cipher_mismatch = true;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_UNPROTECTED);
	KUNIT_EXPECT_MEMEQ(test, cache, before, sizeof(cache));
}

static void mt7932_rx_decrypted_retry_test(struct kunit *test)
{
	struct mt7932_duplicate cache[17] = {};
	struct mt7932_rx_frame frame = rx_qos_frame();

	frame.sec_mode = 0;
	KUNIT_ASSERT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_UNPROTECTED);
	frame.sec_mode = 4;
	frame.frame_control |= IEEE80211_FCTL_RETRY;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_ACCEPT);
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_DUPLICATE);
}

static void mt7932_rx_retry_amsdu_test(struct kunit *test)
{
	struct mt7932_duplicate cache[17] = {};
	struct mt7932_rx_frame frame = rx_qos_frame();

	/* The rejected first copy must not suppress the decrypted aggregate. */
	frame.amsdu = 3;
	frame.sec_mode = 0;
	KUNIT_ASSERT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_UNPROTECTED);
	frame.sec_mode = 4;
	frame.frame_control |= IEEE80211_FCTL_RETRY;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_ACCEPT);
	frame.amsdu = 2;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_ACCEPT);
	frame.amsdu = 1;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_ACCEPT);
	/* A second retry of the completed aggregate is still suppressed. */
	frame.amsdu = 3;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_DUPLICATE);
	frame.amsdu = 2;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_DUPLICATE);
	frame.amsdu = 1;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_DUPLICATE);
}

static void mt7932_rx_sequence_domains_test(struct kunit *test)
{
	struct mt7932_duplicate cache[17] = {};
	struct mt7932_rx_frame frame = rx_qos_frame();

	KUNIT_ASSERT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_ACCEPT);
	/* Same sequence without Retry is a new frame, including after wrap. */
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_ACCEPT);
	frame.frame_control |= IEEE80211_FCTL_RETRY;
	frame.qos_control = 4;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_ACCEPT);
	frame.qos_control = 3;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_DUPLICATE);
	frame.sequence_control = 0xfff0;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_ACCEPT);
	frame.sequence_control = 0;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, false), MT7932_RX_ACCEPT);
}

static void mt7932_rx_open_duplicate_test(struct kunit *test)
{
	struct mt7932_duplicate cache[17] = {};
	struct mt7932_rx_frame frame = rx_qos_frame();

	frame.sec_mode = 0;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, true), MT7932_RX_ACCEPT);
	frame.frame_control |= IEEE80211_FCTL_RETRY;
	KUNIT_EXPECT_EQ(test, mt7932_rx_filter(cache, &frame, true), MT7932_RX_DUPLICATE);
}

static void mt7932_scan_directed_ssids_test(struct kunit *test)
{
	static const u8 channels[] = { 36 };
	u8 body[0x4d4];

	KUNIT_ASSERT_EQ(test, mt7932_scan_body(body, sizeof(body), 1, channels, 1), 0x4d4);
	KUNIT_EXPECT_EQ(test, mt7932_scan_options(body, NULL, 0, NULL, 0), 0);
	KUNIT_EXPECT_EQ(test, body[3], 1);
	KUNIT_EXPECT_EQ(test, body[4], 0);
	KUNIT_EXPECT_EQ(test, mt7932_scan_options(body, (const u8 *)"hidden", 6, NULL, 0), 0);
	KUNIT_EXPECT_EQ(test, body[3], 8);
	KUNIT_EXPECT_EQ(test, body[4], 1);
	/* Further directed SSIDs fill the next slots; wildcards take none. */
	KUNIT_EXPECT_EQ(test, mt7932_scan_ssid_slot(body, NULL, 0), 0);
	KUNIT_EXPECT_EQ(test, mt7932_scan_ssid_slot(body, (const u8 *)"second", 6), 0);
	KUNIT_EXPECT_EQ(test, body[4], 2);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(body + 8 + 36), 6);
	KUNIT_EXPECT_EQ(test, memcmp(body + 12 + 36, "second", 6), 0);
	KUNIT_EXPECT_EQ(test, memcmp(body + 12, "hidden", 6), 0);
	KUNIT_EXPECT_EQ(test, mt7932_scan_ssid_slot(body, (const u8 *)"c", 1), 0);
	KUNIT_EXPECT_EQ(test, mt7932_scan_ssid_slot(body, (const u8 *)"d", 1), 0);
	KUNIT_EXPECT_EQ(test, body[4], 4);
	KUNIT_EXPECT_EQ(test, mt7932_scan_ssid_slot(body, (const u8 *)"e", 1), -ENOSPC);
	KUNIT_EXPECT_EQ(test, get_unaligned_le16(body + 0xe0), 0);
	KUNIT_EXPECT_EQ(test, body[0xa0], 2);
	KUNIT_EXPECT_EQ(test, body[0xa1], 36);
	/* A passive request carries no directed SSID. */
	mt7932_scan_passive(body);
	KUNIT_EXPECT_EQ(test, body[4], 0);
	KUNIT_EXPECT_LT(test, mt7932_scan_ssid_slot(body, (const u8 *)"hidden", 6), 0);
}

/* Only a complete channel switch element announces that the AP is leaving. */
static void mt7932_rx_switch_announced_test(struct kunit *test)
{
	static const struct {
		u8 ies[8];
		size_t length;
		bool announced;
	} cases[] = {
		{ {}, 0, false },
		{ { WLAN_EID_SSID, 0 }, 2, false },
		{ { WLAN_EID_CHANNEL_SWITCH, 3, 1, 100, 5 }, 5, true },
		{ { WLAN_EID_CHANNEL_SWITCH, 2, 1, 100 }, 4, false },
		{ { WLAN_EID_EXT_CHANSWITCH_ANN, 4, 1, 121, 100, 5 }, 6, true },
		{ { WLAN_EID_EXT_CHANSWITCH_ANN, 3, 1, 121, 100 }, 5, false },
		{ { WLAN_EID_SSID, 0, WLAN_EID_CHANNEL_SWITCH, 3, 0, 36, 1 }, 7, true },
	};
	u8 beacon[36 + 8] = {};
	struct mt7932_rx_frame frame = {
		.data = beacon,
		.frame_control = IEEE80211_STYPE_BEACON,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		memcpy(beacon + 36, cases[i].ies, cases[i].length);
		frame.length = 36 + cases[i].length;
		KUNIT_ASSERT_TRUE_MSG(test, mt7932_rx_beacon(&frame), "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, mt7932_rx_switch_announced(&frame),
				    cases[i].announced, "case %u", i);
	}
}

static struct kunit_case mt7932_input_test_cases[] = {
	KUNIT_CASE(mt7932_rx_decrypted_test),
	KUNIT_CASE(mt7932_rx_plaintext_test),
	KUNIT_CASE(mt7932_rx_mismatch_test),
	KUNIT_CASE(mt7932_rx_integrity_test),
	KUNIT_CASE(mt7932_rx_short_test),
	KUNIT_CASE(mt7932_event_reply_test),
	KUNIT_CASE(mt7932_event_reply_eid_test),
	KUNIT_CASE(mt7932_rx_reject_preserves_sequence_test),
	KUNIT_CASE(mt7932_rx_decrypted_retry_test),
	KUNIT_CASE(mt7932_rx_retry_amsdu_test),
	KUNIT_CASE(mt7932_rx_sequence_domains_test),
	KUNIT_CASE(mt7932_rx_open_duplicate_test),
	KUNIT_CASE(mt7932_scan_directed_ssids_test),
	KUNIT_CASE(mt7932_rx_switch_announced_test),
	{}
};

static struct kunit_suite mt7932_input_test_suite = {
	.name = "mt7932-input",
	.test_cases = mt7932_input_test_cases,
};

kunit_test_suite(mt7932_input_test_suite);
