// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/test.h>
#include <linux/hex.h>

#include "association.h"

static void mt_sae_pmk_vectors_test(struct kunit *test)
{
	/* IEEE 802.11i-2004, Annex H.4. These are public test credentials. */
	static const struct {
		const char *password, *ssid, *pmk;
	} vectors[] = {
		{
			"password", "IEEE",
			"f42c6fc52df0ebef9ebb4b90b38a5f902e83fe1b135a70e23aed762e9710a12e",
		}, {
			"ThisIsAPassword", "ThisIsASSID",
			"0dc0d6eb90555ed6419756b9a15ec3e3209b63df707dd508d14581f8982721af",
		}, {
			"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
			"ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ",
			"becb93866bb8c3832cb777c2f559807c8c59afcb6eae734885001300a981cc62",
		},
	};
	u8 pmk[32], expected[32];
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(vectors); i++) {
		KUNIT_ASSERT_EQ(test, hex2bin(expected, vectors[i].pmk, 32), 0);
		ret = mt7932_sae_pmk(pmk, vectors[i].password,
				     strlen(vectors[i].password), vectors[i].ssid,
				     strlen(vectors[i].ssid));
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_MEMEQ(test, pmk, expected, sizeof(pmk));
	}
	memzero_explicit(pmk, sizeof(pmk));
}

static void mt_sae_pmk_bounds_test(struct kunit *test)
{
	u8 password[64], ssid[33], pmk[32], zero[32] = {};
	static const struct {
		size_t password_length, ssid_length;
	} invalid[] = { { 7, 4 }, { 64, 4 }, { 8, 0 }, { 8, 33 } };
	unsigned int i;

	memset(password, 'p', sizeof(password));
	memset(ssid, 's', sizeof(ssid));
	for (i = 0; i < ARRAY_SIZE(invalid); i++) {
		int ret;

		memset(pmk, 0xa5, sizeof(pmk));
		ret = mt7932_sae_pmk(pmk, password, invalid[i].password_length,
				     ssid, invalid[i].ssid_length);
		KUNIT_EXPECT_EQ(test, ret, -EINVAL);
		KUNIT_EXPECT_MEMEQ(test, pmk, zero, sizeof(pmk));
	}
	KUNIT_EXPECT_EQ(test, mt7932_sae_pmk(pmk, NULL, 8, ssid, 4), -EINVAL);
	KUNIT_EXPECT_EQ(test, mt7932_sae_pmk(pmk, password, 8, NULL, 4), -EINVAL);
	KUNIT_EXPECT_EQ(test, mt7932_sae_pmk(pmk, password, 63, ssid, 32), 0);
	memzero_explicit(pmk, sizeof(pmk));
}

static void mt_connect_body_test(struct kunit *test)
{
	static const struct mt7932_peer_ids ids = { { 1, 2 }, { 3, 4, 5, 6 } };
	static const u8 bssid[6] = { 2, 0, 0, 0, 0, 1 };
	static const u8 channels[] = { 11, 157 };
	u8 pmk[32], password[63], body[856], expected[856];
	unsigned int mode, band;

	memset(pmk, 0xa5, sizeof(pmk));
	memset(password, 'p', sizeof(password));
	for (mode = 0; mode < 4; mode++) {
		bool open = mode == 0, sae = mode == 2, owe = mode == 3;

		for (band = 0; band < ARRAY_SIZE(channels); band++) {
			int ret;

			memset(body, 0xff, sizeof(body));
			ret = mt7932_connect_body(body, sizeof(body), "IEEE", 4,
						  open || owe ? NULL : pmk, bssid, channels[band],
						  40, 2, &ids, open,
						  sae ? password : NULL, sae ? 63 : 0, owe);
			KUNIT_ASSERT_EQ(test, ret, 856);
			/* Complete golden requests also catch writes to reserved
			 * fields and keep the established PSK and OPEN encodings.
			 */
			memset(expected, 0, sizeof(expected));
			expected[1] = open || owe ? 1 : sae ? 8 : 4;
			expected[4] = open || owe ? 0 : sae ? 6 : 0x10;
			expected[8] = 0x78;
			expected[0x12] = sae ? 5 : 1;
			if (owe)
				expected[0x20e] = 4;
			if (!open && !owe) {
				put_unaligned_le16(sae ? 0x1000 : 8, expected + 0x14);
				expected[0x40] = sae ? 63 : 32;
				if (sae) {
					expected[0x44] = 10;
					memcpy(expected + 0x4c, password, 63);
					expected[0x1c8] = 32;
					memcpy(expected + 0x1cc, pmk, 32);
					expected[0x20d] = 1;
				} else {
					expected[0x44] = 6;
					memcpy(expected + 0x4c, pmk, 32);
				}
				expected[0x220] = 5;
			}
			expected[0x18] = 4;
			memcpy(expected + 0x1c, "IEEE", 4);
			expected[0x1c0] = 2;
			expected[0x228] = 20;
			memcpy(expected + 0x32c, bssid, sizeof(bssid));
			expected[0x338] = band + 1;
			expected[0x339] = channels[band];
			expected[0x33c] = 1;
			expected[0x33d] = 2;
			expected[0x33e] = 4;
			memcpy(expected + 0x340, ids.station, sizeof(ids.station));
			memcpy(expected + 0x348, ids.wtbl, sizeof(ids.wtbl));
			expected[0x354] = 1;
			KUNIT_EXPECT_MEMEQ(test, body, expected, sizeof(body));
		}
	}
	memzero_explicit(body, sizeof(body));
	memzero_explicit(expected, sizeof(expected));
	memzero_explicit(pmk, sizeof(pmk));
}

static void mt_sae_body_bounds_test(struct kunit *test)
{
	static const struct mt7932_peer_ids ids = { { 1, 2 }, { 3, 4, 5, 6 } };
	static const u8 bssid[6] = { 2, 0, 0, 0, 0, 1 };
	static const struct {
		unsigned int capacity, password_length;
		bool open, password, pmk;
		int expected;
	} requests[] = {
		{ 856, 8, false, true, true, 856 },
		{ 856, 63, false, true, true, 856 },
		{ 855, 8, false, true, true, -EINVAL },
		{ 856, 7, false, true, true, -EINVAL },
		{ 856, 64, false, true, true, -EINVAL },
		{ 856, 8, false, false, true, -EINVAL },
		{ 856, 0, false, true, true, -EINVAL },
		{ 856, 8, true, true, false, -EINVAL },
		{ 856, 8, false, true, false, -EINVAL },
	};
	u8 body[857], password[64] = {}, pmk[32] = {};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(requests); i++) {
		int ret;

		memset(body, 0xa5, sizeof(body));
		ret = mt7932_connect_body(body, requests[i].capacity, "IEEE", 4,
					  requests[i].pmk ? pmk : NULL, bssid, 11, 20, 2, &ids,
					  requests[i].open, requests[i].password ? password : NULL,
					  requests[i].password_length, false);
		KUNIT_EXPECT_EQ(test, ret, requests[i].expected);
		KUNIT_EXPECT_EQ(test, body[856], (u8)0xa5);
		if (requests[i].expected < 0)
			KUNIT_EXPECT_EQ(test, body[0], (u8)0xa5);
	}
	memzero_explicit(body, sizeof(body));
}

enum mt_auth_change {
	MT_AUTH_NONE, MT_AUTH_TYPE, MT_AUTH_MFP, MT_AUTH_PWE, MT_AUTH_PASSWORD_LEN,
	MT_AUTH_NO_PASSWORD, MT_AUTH_WPA, MT_AUTH_PAIR_COUNT,
	MT_AUTH_PAIR, MT_AUTH_GROUP, MT_AUTH_NO_AKM, MT_AUTH_PSK_FIRST,
	MT_AUTH_SAE_FIRST, MT_AUTH_OTHER_FIRST, MT_AUTH_OTHER_AKM, MT_AUTH_PSK,
};

static void mt_connect_auth_test(struct kunit *test)
{
	static const struct {
		const char *name;
		enum mt_auth_change change;
		unsigned int value;
		int expected;
	} cases[] = {
		{ "pure PSK", MT_AUTH_PSK, 0, WLAN_AKM_SUITE_PSK },
		{ "PSK missing PMK", MT_AUTH_PSK, 1, -EOPNOTSUPP },
		{ "PSK required MFP", MT_AUTH_PSK, 2, -EOPNOTSUPP },
		{ "pure SAE", MT_AUTH_NONE, 0, WLAN_AKM_SUITE_SAE },
		{ "optional MFP", MT_AUTH_MFP, NL80211_MFP_OPTIONAL, WLAN_AKM_SUITE_SAE },
		{ "no MFP", MT_AUTH_MFP, NL80211_MFP_NO, -EOPNOTSUPP },
		{ "wrong auth", MT_AUTH_TYPE, NL80211_AUTHTYPE_OPEN_SYSTEM, -EOPNOTSUPP },
		{ "automatic auth", MT_AUTH_TYPE, NL80211_AUTHTYPE_AUTOMATIC, -EOPNOTSUPP },
		{ "both PWE", MT_AUTH_PWE, NL80211_SAE_PWE_BOTH, WLAN_AKM_SUITE_SAE },
		{ "H2E", MT_AUTH_PWE, NL80211_SAE_PWE_HASH_TO_ELEMENT, WLAN_AKM_SUITE_SAE },
		{ "HnP", MT_AUTH_PWE, NL80211_SAE_PWE_HUNT_AND_PECK, WLAN_AKM_SUITE_SAE },
		{ "unknown PWE", MT_AUTH_PWE, 255, -EOPNOTSUPP },
		{ "short password", MT_AUTH_PASSWORD_LEN, 7, -EOPNOTSUPP },
		{ "long password", MT_AUTH_PASSWORD_LEN, 64, -EOPNOTSUPP },
		{ "maximum password", MT_AUTH_PASSWORD_LEN, 63, WLAN_AKM_SUITE_SAE },
		{ "no password", MT_AUTH_NO_PASSWORD, 0, -EOPNOTSUPP },
		{ "SAE VERSION_3", MT_AUTH_WPA, NL80211_WPA_VERSION_3, WLAN_AKM_SUITE_SAE },
		{ "SAE legacy VERSION_2", MT_AUTH_WPA, NL80211_WPA_VERSION_2, WLAN_AKM_SUITE_SAE },
		{ "SAE mixed RSN versions", MT_AUTH_WPA,
		  NL80211_WPA_VERSION_2 | NL80211_WPA_VERSION_3, WLAN_AKM_SUITE_SAE },
		{ "no WPA version", MT_AUTH_WPA, 0, -EOPNOTSUPP },
		{ "unknown WPA version", MT_AUTH_WPA, 8, -EOPNOTSUPP },
		{ "WPA1", MT_AUTH_WPA, NL80211_WPA_VERSION_1, -EOPNOTSUPP },
		{ "mixed WPA", MT_AUTH_WPA, 3, -EOPNOTSUPP },
		{ "no pairwise", MT_AUTH_PAIR_COUNT, 0, -EOPNOTSUPP },
		{ "multiple pairwise", MT_AUTH_PAIR_COUNT, 2, -EOPNOTSUPP },
		{ "TKIP pairwise", MT_AUTH_PAIR, WLAN_CIPHER_SUITE_TKIP, -EOPNOTSUPP },
		{ "TKIP group", MT_AUTH_GROUP, WLAN_CIPHER_SUITE_TKIP, -EOPNOTSUPP },
		{ "no AKM", MT_AUTH_NO_AKM, 0, -EOPNOTSUPP },
		{ "SAE EXT KEY", MT_AUTH_OTHER_AKM, SUITE(0x000fac, 24), -EOPNOTSUPP },
		{ "PSK SHA256", MT_AUTH_OTHER_AKM, WLAN_AKM_SUITE_PSK_SHA256, -EOPNOTSUPP },
		{ "SAE first with both credentials", MT_AUTH_SAE_FIRST, 0, WLAN_AKM_SUITE_SAE },
		{ "SAE first missing password", MT_AUTH_SAE_FIRST, 1, -EOPNOTSUPP },
		{ "PSK first optional MFP", MT_AUTH_PSK_FIRST, 0, WLAN_AKM_SUITE_PSK },
		{ "PSK first missing PMK", MT_AUTH_PSK_FIRST, 1, -EOPNOTSUPP },
		{ "PSK first required MFP", MT_AUTH_PSK_FIRST, 2, -EOPNOTSUPP },
		{ "PSK first no MFP", MT_AUTH_PSK_FIRST, 3, WLAN_AKM_SUITE_PSK },
		{ "PSK first mixed versions", MT_AUTH_PSK_FIRST, 4, WLAN_AKM_SUITE_PSK },
		{ "PSK first VERSION_3 only", MT_AUTH_PSK_FIRST, 5, -EOPNOTSUPP },
		{ "SAE first mixed versions", MT_AUTH_SAE_FIRST, 2, WLAN_AKM_SUITE_SAE },
		{ "unsupported AKM before SAE", MT_AUTH_OTHER_FIRST, 0, -EOPNOTSUPP },
	};
	u8 password[64] = {}, pmk[32] = {};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		struct cfg80211_connect_params p = {
			.auth_type = NL80211_AUTHTYPE_SAE,
			.mfp = NL80211_MFP_REQUIRED,
			.crypto = {
				.wpa_versions = NL80211_WPA_VERSION_3,
				.n_ciphers_pairwise = 1,
				.ciphers_pairwise = { WLAN_CIPHER_SUITE_CCMP },
				.cipher_group = WLAN_CIPHER_SUITE_CCMP,
				.n_akm_suites = 1,
				.akm_suites = { WLAN_AKM_SUITE_SAE },
				.sae_pwd = password,
				.sae_pwd_len = 8,
			},
		};
		unsigned int value = cases[i].value;

		switch (cases[i].change) {
		case MT_AUTH_NONE:
			break;
		case MT_AUTH_TYPE:
			p.auth_type = value;
			break;
		case MT_AUTH_MFP:
			p.mfp = value;
			break;
		case MT_AUTH_PWE:
			p.crypto.sae_pwe = value;
			break;
		case MT_AUTH_PASSWORD_LEN:
			p.crypto.sae_pwd_len = value;
			break;
		case MT_AUTH_NO_PASSWORD:
			p.crypto.sae_pwd = NULL;
			break;
		case MT_AUTH_WPA:
			p.crypto.wpa_versions = value;
			break;
		case MT_AUTH_PAIR_COUNT:
			p.crypto.n_ciphers_pairwise = value;
			break;
		case MT_AUTH_PAIR:
			p.crypto.ciphers_pairwise[0] = value;
			break;
		case MT_AUTH_GROUP:
			p.crypto.cipher_group = value;
			break;
		case MT_AUTH_NO_AKM:
			p.crypto.n_akm_suites = 0;
			break;
		case MT_AUTH_OTHER_AKM:
			p.crypto.akm_suites[0] = value;
			break;
		case MT_AUTH_PSK:
			p.crypto.wpa_versions = NL80211_WPA_VERSION_2;
			p.crypto.akm_suites[0] = WLAN_AKM_SUITE_PSK;
			p.crypto.psk = value == 1 ? NULL : pmk;
			p.crypto.sae_pwd = NULL;
			p.crypto.sae_pwd_len = 0;
			p.auth_type = NL80211_AUTHTYPE_AUTOMATIC;
			p.mfp = value == 2 ? NL80211_MFP_REQUIRED : NL80211_MFP_NO;
			break;
		case MT_AUTH_PSK_FIRST:
			p.crypto.wpa_versions = value == 5 ? NL80211_WPA_VERSION_3 :
				NL80211_WPA_VERSION_2;
			if (value == 4)
				p.crypto.wpa_versions |= NL80211_WPA_VERSION_3;
			p.crypto.n_akm_suites = 2;
			p.crypto.akm_suites[0] = WLAN_AKM_SUITE_PSK;
			p.crypto.akm_suites[1] = WLAN_AKM_SUITE_SAE;
			p.crypto.psk = value == 1 ? NULL : pmk;
			p.auth_type = NL80211_AUTHTYPE_OPEN_SYSTEM;
			p.mfp = value == 2 ? NL80211_MFP_REQUIRED :
				value == 3 ? NL80211_MFP_NO : NL80211_MFP_OPTIONAL;
			break;
		case MT_AUTH_SAE_FIRST:
			p.crypto.n_akm_suites = 2;
			p.crypto.akm_suites[1] = WLAN_AKM_SUITE_PSK;
			p.crypto.psk = pmk;
			if (value == 1)
				p.crypto.sae_pwd = NULL;
			if (value == 2)
				p.crypto.wpa_versions |= NL80211_WPA_VERSION_2;
			p.mfp = NL80211_MFP_OPTIONAL;
			break;
		case MT_AUTH_OTHER_FIRST:
			p.crypto.n_akm_suites = 2;
			p.crypto.akm_suites[0] = WLAN_AKM_SUITE_PSK_SHA256;
			p.crypto.akm_suites[1] = WLAN_AKM_SUITE_SAE;
			break;
		}
		KUNIT_EXPECT_EQ_MSG(test, mt7932_connect_auth(&p), cases[i].expected,
				    "%s", cases[i].name);
	}
}

static void mt_owe_body_bounds_test(struct kunit *test)
{
	static const struct mt7932_peer_ids ids = { { 1, 2 }, { 3, 4, 5, 6 } };
	static const u8 bssid[6] = { 2, 0, 0, 0, 0, 1 };
	u8 body[857], pmk[32] = {}, password[8] = {};
	unsigned int i;

	for (i = 0; i < 6; i++) {
		int ret;

		memset(body, 0xa5, sizeof(body));
		ret = mt7932_connect_body(body, i == 0 ? 855 : 856, "IEEE", 4,
					  i == 1 ? pmk : NULL, bssid, 11, 20, 2, &ids,
					  i == 2, i == 3 || i == 4 ? password : NULL,
					  i == 3 || i == 5 ? 8 : 0, true);
		KUNIT_EXPECT_EQ(test, ret, -EINVAL);
		KUNIT_EXPECT_EQ(test, body[0], (u8)0xa5);
		KUNIT_EXPECT_EQ(test, body[856], (u8)0xa5);
	}
}

static void mt_owe_connect_auth_test(struct kunit *test)
{
	u8 credential[32] = {};
	unsigned int i;

	for (i = 0; i < 20; i++) {
		struct cfg80211_connect_params p = {
			.auth_type = NL80211_AUTHTYPE_OPEN_SYSTEM,
			.mfp = NL80211_MFP_REQUIRED,
			.crypto = {
				.wpa_versions = NL80211_WPA_VERSION_2,
				.n_ciphers_pairwise = 1,
				.ciphers_pairwise = { WLAN_CIPHER_SUITE_CCMP },
				.cipher_group = WLAN_CIPHER_SUITE_CCMP,
				.n_akm_suites = 1,
				.akm_suites = { WLAN_AKM_SUITE_OWE },
			},
		};
		int expected = -EOPNOTSUPP;

		switch (i) {
		case 0:
			expected = WLAN_AKM_SUITE_OWE;
			break;
		case 1:
			p.crypto.psk = credential;
			break;
		case 2:
			p.crypto.sae_pwd = credential;
			break;
		case 3:
			p.crypto.sae_pwd_len = 8;
			break;
		case 4:
			p.auth_type = NL80211_AUTHTYPE_AUTOMATIC;
			break;
		case 5:
			p.auth_type = NL80211_AUTHTYPE_SAE;
			break;
		case 6:
			p.mfp = NL80211_MFP_OPTIONAL;
			break;
		case 7:
			p.mfp = NL80211_MFP_NO;
			break;
		case 8:
			p.crypto.wpa_versions = NL80211_WPA_VERSION_1;
			break;
		case 9:
			p.crypto.n_ciphers_pairwise = 0;
			break;
		case 10:
			p.crypto.n_ciphers_pairwise = 2;
			break;
		case 11:
			p.crypto.ciphers_pairwise[0] = WLAN_CIPHER_SUITE_TKIP;
			break;
		case 12:
			p.crypto.cipher_group = WLAN_CIPHER_SUITE_TKIP;
			break;
		case 13:
			p.crypto.n_akm_suites = 0;
			break;
		case 14:
			p.key = credential;
			break;
		case 15:
			p.key_len = 1;
			break;
		case 16:
			p.want_1x = true;
			break;
		case 17:
			p.crypto.n_akm_suites = 2;
			p.crypto.akm_suites[1] = WLAN_AKM_SUITE_PSK;
			expected = WLAN_AKM_SUITE_OWE;
			break;
		case 18:
			p.crypto.n_akm_suites = 2;
			p.crypto.akm_suites[0] = WLAN_AKM_SUITE_PSK;
			p.crypto.akm_suites[1] = WLAN_AKM_SUITE_OWE;
			break;
		case 19:
			p.crypto.wpa_versions |= NL80211_WPA_VERSION_1;
			break;
		}
		KUNIT_EXPECT_EQ_MSG(test, mt7932_connect_auth(&p), expected, "case %u", i);
	}
}

/* A radar or no-IR candidate counts only once heard during this attempt. */
static void mt_bss_heard_test(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test, mt7932_bss_heard(0, 0, 1000));
	KUNIT_EXPECT_FALSE(test, mt7932_bss_heard(IEEE80211_CHAN_RADAR, 999, 1000));
	KUNIT_EXPECT_TRUE(test, mt7932_bss_heard(IEEE80211_CHAN_RADAR, 1000, 1000));
	KUNIT_EXPECT_FALSE(test, mt7932_bss_heard(IEEE80211_CHAN_NO_IR, 0, 1000));
	KUNIT_EXPECT_TRUE(test, mt7932_bss_heard(IEEE80211_CHAN_NO_IR | IEEE80211_CHAN_RADAR,
						 2000, 1000));
}

static struct kunit_case mt_association_test_cases[] = {
	KUNIT_CASE(mt_owe_body_bounds_test),
	KUNIT_CASE(mt_owe_connect_auth_test),
	KUNIT_CASE(mt_sae_pmk_vectors_test),
	KUNIT_CASE(mt_sae_pmk_bounds_test),
	KUNIT_CASE(mt_connect_body_test),
	KUNIT_CASE(mt_sae_body_bounds_test),
	KUNIT_CASE(mt_connect_auth_test),
	KUNIT_CASE(mt_bss_heard_test),
	{}
};

static struct kunit_suite mt_association_test_suite = {
	.name = "mt7932-association",
	.test_cases = mt_association_test_cases,
};

kunit_test_suite(mt_association_test_suite);
