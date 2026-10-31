/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef MT7932_ASSOCIATION_H
#define MT7932_ASSOCIATION_H

#include <crypto/hash.h>
#include <crypto/sha1.h>
#include <net/cfg80211.h>

#include "protocol.h"
#include "channels.h"

#define MT7932_SAE_PASSWORD_MIN 8
#define MT7932_SAE_PASSWORD_MAX 63

/* On a radar or no-IR channel the station may transmit only to an AP it has
 * heard since the current connect attempt began listening; cfg80211 keeps a
 * BSS for 30 s, long after an AP may have left the channel for radar.
 */
static inline bool mt7932_bss_heard(u32 channel_flags, u64 seen_ns, u64 since_ns)
{
	return !(channel_flags & (IEEE80211_CHAN_RADAR | IEEE80211_CHAN_NO_IR)) ||
	       seen_ns >= since_ns;
}

/* The first AKM is selected by userspace. Credentials for an alternative
 * must never change the selected security arm on a transition BSS.
 */
static inline int
mt7932_connect_auth(const struct cfg80211_connect_params *p)
{
	u32 versions = p->crypto.wpa_versions;

	/* SAE offload clients use VERSION_3; older clients use VERSION_2.
	 * A transition request can carry both. The selected AKM still owns
	 * the firmware arm, so these version bits never enable a fallback.
	 */
	if (!versions ||
	    (versions & ~(NL80211_WPA_VERSION_2 | NL80211_WPA_VERSION_3)) ||
	    p->crypto.n_ciphers_pairwise != 1 ||
	    p->crypto.ciphers_pairwise[0] != WLAN_CIPHER_SUITE_CCMP ||
	    p->crypto.cipher_group != WLAN_CIPHER_SUITE_CCMP ||
	    p->crypto.n_akm_suites < 1)
		return -EOPNOTSUPP;

	switch (p->crypto.akm_suites[0]) {
	case WLAN_AKM_SUITE_PSK:
		if ((versions & NL80211_WPA_VERSION_2) && p->crypto.psk &&
		    (p->mfp == NL80211_MFP_NO || p->mfp == NL80211_MFP_OPTIONAL))
			return WLAN_AKM_SUITE_PSK;
		break;
	case WLAN_AKM_SUITE_OWE:
		if (p->auth_type == NL80211_AUTHTYPE_OPEN_SYSTEM &&
		    p->mfp == NL80211_MFP_REQUIRED && !p->crypto.psk &&
		    !p->crypto.sae_pwd && !p->crypto.sae_pwd_len &&
		    !p->key && !p->key_len && !p->want_1x)
			return WLAN_AKM_SUITE_OWE;
		break;
	case WLAN_AKM_SUITE_SAE:
		if (p->auth_type != NL80211_AUTHTYPE_SAE ||
		    !p->crypto.sae_pwd ||
		    p->crypto.sae_pwd_len < MT7932_SAE_PASSWORD_MIN ||
		    p->crypto.sae_pwd_len > MT7932_SAE_PASSWORD_MAX ||
		    (p->mfp != NL80211_MFP_REQUIRED &&
		     p->mfp != NL80211_MFP_OPTIONAL))
			break;
		/* Firmware selects PWE itself; there is no request field for it. */
		if (p->crypto.sae_pwe == NL80211_SAE_PWE_UNSPECIFIED ||
		    p->crypto.sae_pwe == NL80211_SAE_PWE_HUNT_AND_PECK ||
		    p->crypto.sae_pwe == NL80211_SAE_PWE_BOTH ||
		    p->crypto.sae_pwe == NL80211_SAE_PWE_HASH_TO_ELEMENT)
			return WLAN_AKM_SUITE_SAE;
		break;
	}
	return -EOPNOTSUPP;
}

/* The SAE offload needs both the password and a WPA passphrase-derived PMK.
 * This is PBKDF2-HMAC-SHA1(password, SSID, 4096, 32), not host SAE crypto.
 * Run only in sleepable context and erase every keyed intermediate.
 */
static inline int mt7932_sae_pmk(u8 pmk[32], const u8 *password,
				 size_t password_length, const u8 *ssid,
				 size_t ssid_length)
{
	u8 salt[32 + sizeof(__be32)];
	u8 u[SHA1_DIGEST_SIZE], next[SHA1_DIGEST_SIZE];
	u8 block_value[SHA1_DIGEST_SIZE];
	struct crypto_shash *tfm;
	struct shash_desc *desc;
	unsigned int block, iteration, i, copied = 0;
	int ret;

	memzero_explicit(pmk, 32);
	if (!password || password_length < MT7932_SAE_PASSWORD_MIN ||
	    password_length > MT7932_SAE_PASSWORD_MAX ||
	    !ssid || !ssid_length || ssid_length > 32)
		return -EINVAL;
	tfm = crypto_alloc_shash("hmac(sha1)", 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);
	desc = kzalloc(sizeof(*desc) + crypto_shash_descsize(tfm), GFP_KERNEL);
	if (!desc) {
		ret = -ENOMEM;
		goto free_tfm;
	}
	desc->tfm = tfm;
	ret = crypto_shash_setkey(tfm, password, password_length);
	if (ret)
		goto clear;
	memcpy(salt, ssid, ssid_length);
	for (block = 1; copied < 32; block++) {
		size_t length = min_t(size_t, SHA1_DIGEST_SIZE, 32 - copied);

		put_unaligned_be32(block, salt + ssid_length);
		ret = crypto_shash_digest(desc, salt,
					  ssid_length + sizeof(__be32), u);
		if (ret)
			goto clear;
		memcpy(block_value, u, sizeof(block_value));
		for (iteration = 1; iteration < 4096; iteration++) {
			ret = crypto_shash_digest(desc, u, sizeof(u), next);
			if (ret)
				goto clear;
			memcpy(u, next, sizeof(u));
			for (i = 0; i < sizeof(block_value); i++)
				block_value[i] ^= u[i];
		}
		memcpy(pmk + copied, block_value, length);
		copied += length;
		cond_resched();
	}
clear:
	memzero_explicit(salt, sizeof(salt));
	memzero_explicit(u, sizeof(u));
	memzero_explicit(next, sizeof(next));
	memzero_explicit(block_value, sizeof(block_value));
	if (ret)
		memzero_explicit(pmk, 32);
	kfree_sensitive(desc);
free_tfm:
	crypto_free_shash(tfm);
	return ret;
}

struct mt7932_peer_ids {
	u8 station[2], wtbl[4];
};

/* Firmware owns association IE generation. iwd always declares BSS transition
 * and FILS capability bits even for ordinary OPEN authentication. Accept only
 * those declarations (or zero subsets), not a FILS operation or arbitrary IE
 * carriage request. The generated capabilities are reported separately.
 */
static inline bool mt7932_open_ies(const u8 *ies, size_t length)
{
	unsigned int i;

	if (!length)
		return true;
	if (!ies || length != 12 || ies[0] != 127 || ies[1] != 10)
		return false;
	for (i = 0; i < 10; i++) {
		u8 allowed = i == 2 ? 8 : i == 9 ? 1 : 0;

		if (ies[i + 2] & ~allowed)
			return false;
	}
	return true;
}

/* Bounded, nonsecret subset of firmware-generated request IEs. Unknown/vendor
 * contents are not exported. These bytes are actual generated capabilities,
 * not an echo of the caller's capability declaration or replacement RF IEs.
 */
static inline int mt7932_assoc_capabilities(u8 *out, size_t capacity,
					 const u8 *ies, size_t length, bool open)
{
	size_t offset = 0, used = 0;
	unsigned int seen = 0;

	while (offset < length) {
		u8 id, size;
		unsigned int bit = 0, expected = 0;

		if (length - offset < 2)
			return -EINVAL;
		id = ies[offset];
		size = ies[offset + 1];
		if (size > length - offset - 2)
			return -EINVAL;
		if (open && (id == 48 || (id == 221 && size >= 4 &&
		    !memcmp(ies + offset + 2, "\x00\x50\xf2\x01", 4))))
			return -EINVAL;
		switch (id) {
		case 45: bit = 1; expected = 26; break;
		case 61: bit = 2; expected = 22; break;
		case 191: bit = 4; expected = 12; break;
		case 192: bit = 8; expected = 5; break;
		case 127: bit = 16; expected = size; break;
		case WLAN_EID_EXTENSION:
			if (!size)
				return -EINVAL;
			if (ies[offset + 2] != WLAN_EID_EXT_HE_CAPABILITY)
				break;
			/* Validate the complete standard HE layout, including optional
			 * wider MCS maps and PPE thresholds, before reporting the
			 * firmware's actual request. This does not advertise new local
			 * PHY modes or alter the native association/rate owner.
			 */
			if (!ieee80211_he_capa_size_ok(ies + offset + 3, size - 1))
				return -EINVAL;
			bit = 32;
			expected = size;
			break;
		}
		if (bit) {
			if ((seen & bit) || size != expected ||
			    (id == 127 && (!size || size > 16)) ||
			    size + 2U > capacity - used)
				return -EINVAL;
			memcpy(out + used, ies + offset, size + 2);
			used += size + 2;
			seen |= bit;
		}
		offset += size + 2;
	}
	return used;
}

/* Original25G83 pure WPA2 binary-key result: structured type6, not a string.
 * The standard nl80211 PMK maps directly; no passphrase API, key derivation,
 * duplicate independent PMK or extra flags.
 */
static inline int mt7932_connect_binary_pmk(u8 *body, size_t capacity,
					 const u8 *pmk, size_t length)
{
	if (capacity < 856 || length != 32)
		return -EINVAL;
	memset(body + 0x40, 0, 0x90);
	put_unaligned_le32(32, body + 0x40);
	put_unaligned_le32(6, body + 0x44);
	memcpy(body + 0x4c, pmk, 32);
	/* Clear length, all64 independent storage bytes and explicit allow-auth.
	 * Policy0 is the coherent original64-hex-input RESULT, not open security.
	 */
	memset(body + 0x1c8, 0, 4 + 64 + 4);
	return 0;
}

/* SAE offload owns PMF and the full handshake; no host key installation. */
static inline void mt7932_connect_sae_body(u8 *body, const u8 *password,
					   size_t password_length, const u8 pmk[32])
{
	body[1] = 8;
	put_unaligned_le32(6, body + 4);
	put_unaligned_le32(0x78, body + 8);
	put_unaligned_le16(5, body + 0x12);
	put_unaligned_le16(0x1000, body + 0x14);
	put_unaligned_le32(password_length, body + 0x40);
	/* Key cipher shares +0x44 with the PSK arm's binary-PMK key type;
	 * +0x48 holds 16-bit key flags and must stay zero.
	 */
	put_unaligned_le32(10, body + 0x44);
	memcpy(body + 0x4c, password, password_length);
	put_unaligned_le32(32, body + 0x1c8);
	memcpy(body + 0x1cc, pmk, 32);
	/* SAE implies required PMF; BIT(30) would exclude pure SAE. */
	put_unaligned_le32(BIT(8), body + 0x20c);
}

/* Explicit OPEN, OWE, PMK-only WPA2/CCMP or SAE; no automatic security fallback.
 * Caller owns identities/resources and validated BSS state. Never log this
 * body: it carries the PMK.
 */
static inline int mt7932_connect_body(u8 *body, size_t capacity, const u8 *ssid,
				      size_t ssid_length, const u8 pmk[32], const u8 bssid[6],
				      u8 channel, unsigned int listen_interval, u8 connection_flags,
				      const struct mt7932_peer_ids *ids, bool open,
				      const u8 *sae_pwd, size_t sae_pwd_length, bool owe)
{
	u8 resources[6];
	bool sae = sae_pwd_length != 0;
	unsigned int i, j;
	int ret;

	if (((open || owe) ? !!pmk : !pmk) ||
	    (open && owe) ||
	    (sae && (open || owe || !sae_pwd ||
		     sae_pwd_length < MT7932_SAE_PASSWORD_MIN ||
		     sae_pwd_length > MT7932_SAE_PASSWORD_MAX)) ||
	    (sae_pwd && !sae) ||
	    capacity < 856 || !ssid_length || ssid_length > 32 ||
	    !mt7932_channel_band(channel) || connection_flags > 63 || (bssid[0] & 1))
		return -EINVAL;
	memcpy(resources, ids->station, 2);
	memcpy(resources + 2, ids->wtbl, 4);
	for (i = 0; i < 6; i++) {
		/* Global0 and per-interface BMC19 remain separately reserved. */
		if (!resources[i] || resources[i] >= (i < 2 ? 15 : 14))
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (resources[i] == resources[j])
				return -EINVAL;
	}
	memset(body, 0, 856);
	/* NORMAL25G83 original strict keyless OPEN branch, without optional
	 * automatic-authentication policy. Keep this coherent version/auth/
	 * policy tuple together; no dummy keys or stale security state.
	 */
	if (open || owe) {
		body[1] = 1;
		put_unaligned_le32(0x78, body + 8);
		put_unaligned_le16(1, body + 0x12);
		/* Firmware owns OWE DH, keys and RSN generation. No host key
		 * or cipher encoding belongs in this request arm.
		 */
		if (owe)
			put_unaligned_le32(BIT(18), body + 0x20c);
	} else if (sae) {
		mt7932_connect_sae_body(body, sae_pwd, sae_pwd_length, pmk);
	} else {
		body[1] = 4;
		put_unaligned_le32(0x10, body + 4);
		put_unaligned_le32(0x78, body + 8);
		put_unaligned_le16(1, body + 0x12);
		put_unaligned_le16(8, body + 0x14);
	}
	put_unaligned_le32(ssid_length, body + 0x18);
	memcpy(body + 0x1c, ssid, ssid_length);
	/* Retain the established policy byte for the qualified request. */
	body[0x1c0] = connection_flags;
	/* Fresh HOST AIS media state1 selects CCMP type5 here. This is not
	 * structured-key type0x44 or the association event's RSN suite encoding.
	 * This builder is only used for an unassociated attempt.
	 */
	put_unaligned_le32(open || owe ? 0 : 5, body + 0x220);
	put_unaligned_le16(listen_interval > 20 ? 20 : listen_interval, body + 0x228);
	memcpy(body + 0x32c, bssid, 6);
	body[0x338] = mt7932_channel_band(channel);
	body[0x339] = channel;
	body[0x33c] = 1;
	body[0x33d] = 2;
	body[0x33e] = 4;
	memcpy(body + 0x340, ids->station, 2);
	memcpy(body + 0x348, ids->wtbl, 4);
	body[0x354] = 1;
	ret = open || owe || sae ? 0 :
		mt7932_connect_binary_pmk(body, capacity, pmk, 32);
	return ret ? ret : 856;
}

static inline void mt7932_peer_queue(u8 body[32], u8 station, bool bind)
{
	static const u8 tids[] = {1,0,0,1,2,2,3,3};

	memset(body, 0, 32);
	put_unaligned_le16(15, body);
	body[2] = 5;
	body[14] = 1;
	body[15] = station;
	put_unaligned_le16(bind ? 15 : 2, body + 16);
	memcpy(body + 20, tids, sizeof(tids));
}

static inline bool mt7932_assoc_resources(const u8 *body, size_t length,
					 const struct mt7932_peer_ids *ids, u8 channel, bool open)
{
	unsigned int i;

	if (length < 52 || get_unaligned_le16(body) || body[16] ||
	    body[17] != ids->wtbl[0] || body[20] != ids->station[0] ||
	    !mt7932_channel_band(channel) || body[11] != mt7932_channel_band(channel) || body[12] != channel ||
	    (!open && get_unaligned_le32(body + 48) != 0x000fac04))
		return false;
	for (i = 0; i < 4; i++)
		if (body[21] == ids->wtbl[i])
			return true;
	return false;
}

#endif
