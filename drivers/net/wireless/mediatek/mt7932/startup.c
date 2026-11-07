// SPDX-License-Identifier: GPL-2.0-only
#include <crypto/sha2.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/rtnetlink.h>
#include <linux/seq_file.h>

#include "mt7932.h"

static int mt_stock_config(struct mt7932 *m);
static noinline_for_stack int mt_publish_policy(struct mt7932 *m,
						const struct mt7932_reg_snapshot *reg,
						const struct mt7932_policy *policy);
static int mt_privacy_defaults(struct mt7932 *m);
static int mt_startup_once(struct mt7932 *m);
static void mt_startup_work(struct work_struct *work);
static void mt_regulatory_notify(struct wiphy *wiphy, struct regulatory_request *request);
static const char *mt7932_policy_fallback_reason(int fallback);

struct mt_config_record {
	u8 type, key_len, value_len, reserved;
	char key[32], value[32];
};

static int mt_stock_config(struct mt7932 *m)
{
	static const struct { const char *key; u8 at, fallback; } updates[] = {
		{"StaVHT",1,0}, {"ApVHT",1,1}, {"P2pGoVHT",1,1}, {"P2pGcVHT",1,1},
		{"Nss",4,0}, {"LdpcTx",6,0}, {"LdpcRx",7,0}, {"StbcTx",8,0}, {"StbcRx",9,0},
	};
	struct mt_config_record *records;
	const struct firmware *file;
	const char *phase = "file-format";
	u8 body[284];
	unsigned int count, i, j, batch;
	int ret;

	ret = mt_request_input(m, &file, "mediatek/mt7932/config-original.bin");
	if (ret)
		return ret;
	ret = -EINVAL;
	if (file->size < 8 || memcmp(file->data, "J7CF", 4))
		goto out;
	count = get_unaligned_le32(file->data + 4);
	if ((count != 64 && count != 65) || file->size != 8 + count * 68)
		goto out;
	/*
	 * Each absent normalized key appends one record. Reserve for all of
	 * them, including structurally valid packages missing every such key.
	 */
	records = kcalloc(count + ARRAY_SIZE(updates), sizeof(*records), GFP_KERNEL);
	if (!records) {
		phase = "host allocation";
		ret = -ENOMEM;
		goto out;
	}
	memcpy(records, file->data + 8, count * 68);
	for (i = 0; i < count; i++) {
		if (records[i].type != 3 || !records[i].key_len || records[i].key_len >= 32 ||
		    records[i].value_len >= 32 || records[i].reserved ||
		    strnlen(records[i].key, 32) != records[i].key_len ||
		    strnlen(records[i].value, 32) != records[i].value_len)
			goto free;
		for (j = 0; j < i; j++)
			if (!strcmp(records[i].key, records[j].key))
				goto free;
	}
	/* The measured J700 profile has PHY DBDC=1. A different capability needs
	 * the separately specified conditional DbdcMode insertion, not a default.
	 */
	if (!m->phy_cap[5]) {
		phase = "capability profile";
		dev_err(&m->pdev->dev, "STOCK_CONFIG_CAPABILITY_MISMATCH: DBDC expected=1 observed=0\n");
		ret = -EOPNOTSUPP;
		goto free;
	}
	for (i = 0; i < ARRAY_SIZE(updates); i++) {
		u8 requested = updates[i].fallback, value;

		for (j = 0; j < count; j++)
			if (!strcmp(records[j].key, updates[i].key))
				break;
		if (j < count) {
			if (kstrtou8(records[j].value, 0, &requested))
				goto free;
		} else {
			count++;
			records[j].type = 3;
			records[j].key_len = strscpy(records[j].key, updates[i].key, 32);
		}
		value = updates[i].at == 4 ? min(requested, m->phy_cap[4]) : requested & m->phy_cap[updates[i].at];
		memset(records[j].value, 0, 32);
		records[j].value_len = scnprintf(records[j].value, 32, "0x%x", value);
	}
	{
		/* Same ceiling as BASIC_CONFIG, without changing original artifacts.
		 * CID0f uses a distinct enum; keep Sta2gBw untouched.
		 */
		for (j = 0; j < count; j++)
			if (!strcmp(records[j].key, "Sta5gBw"))
				break;
		if (j == count) {
			ret = -EINVAL;
			goto free;
		}
		memset(records[j].value, 0, sizeof(records[j].value));
		records[j].value[0] = '0' + MT7932_MAX_5G_BW;
		records[j].value_len = 1;
		for (j = 0; j < count; j++)
			if (!strcmp(records[j].key, "DisRoaming"))
				break;
		if (j == count) {
			ret = -EINVAL;
			goto free;
		}
		memset(records[j].value, 0, sizeof(records[j].value));
		records[j].value[0] = '1';
		records[j].value_len = 1;
	}
	phase = "native SET submission";
	for (i = 0; i < count; i += batch) {
		batch = min(4U, count - i);
		memset(body, 0, sizeof(body));
		put_unaligned_le32(1, body + 4);
		body[8] = batch;
		put_unaligned_le16(batch * 68, body + 10);
		memcpy(body + 12, records + i, batch * 68);
		ret = mt_request(m, 0x70, true, true, false, body, sizeof(body));
		if (ret)
			goto free;
	}
	dev_dbg(&m->pdev->dev, "STOCK_CONFIG_SUBMITTED: %u original/PHY-derived entries\n", count);
free:
	kfree(records);
out:
	if (ret) {
		if (!strcmp(phase, "file-format"))
			dev_err(&m->pdev->dev, "local input mediatek/mt7932/config-original.bin has invalid format: %d\n", ret);
		else
			dev_err(&m->pdev->dev, "STOCK_CONFIG_FAILED: phase=%s error=%d\n", phase, ret);
	}
	release_firmware(file);
	return ret;
}

static noinline_for_stack int mt_publish_policy(struct mt7932 *m,
						const struct mt7932_reg_snapshot *reg,
						const struct mt7932_policy *policy)
{
	u8 dbdc[36] = {0,0,0,0,2,0,36};
	unsigned int i;
	struct mt7932_6g_limits limits =
		mt7932_6g_limits_mask(reg, policy->legacy_channels);
	u8 body[1020];
	unsigned int j;
	int ret;

	if (!m->startup_started) {
		ret = mt_request(m, 0x28, true, true, false, dbdc, sizeof(dbdc));
		if (ret)
			return ret;
	}
	for (i = 0; i < policy->modes; i++) {
		ret = mt_request(m, 0xca, true, true, false, policy->mode[i], 328);
		if (ret)
			return ret;
	}
	memcpy(body, reg->domain, reg->length);
	for (j = reg->domain[8] + reg->domain[9];
	     j < reg->domain[8] + reg->domain[9] + reg->domain[10]; j++) {
		u32 flags = get_unaligned_le32(body + 16 + j * 8);

		put_unaligned_le32(flags & ~IEEE80211_CHAN_PSD, body + 16 + j * 8);
	}
	ret = mt_request(m, 0x0f, true, true, false, body, reg->length);
	if (ret)
		return ret;
	for (i = 0; i < MT7932_POLICY_TABLES; i++) {
		ret = mt_request(m, 0x5d, true, true, false, policy->table[i], policy->length[i]);
		if (ret)
			return ret;
	}
	m->six_power_valid = false;
	m->six_legacy_channels = policy->legacy_channels;
	for (i = 0; i < policy->six_pages; i++) {
		memcpy(body, policy->six_page[i], policy->six_length[i]);
		if (policy->six_cid[i] == 0x5d)
			mt7932_6g_limit_page(body, &limits);
		ret = mt_request(m, policy->six_cid[i], true, true, false,
				 body, policy->six_length[i]);
		for (j = 0; j < 2; j++)
			if (policy->six_page[i] == policy->assoc_power[j])
				memcpy(m->six_assoc_power[j], body, 1020);
		if (ret)
			return ret;
	}
	if (m->six_class_dirty) {
		struct mt7932_6g_ap clear = m->six_ap;

		/* The established all-0x7f command removes the prior AP ceiling.
		 * Publish VLP's country tables first, while RF admission is fenced.
		 */
		memset(clear.limit, 127, sizeof(clear.limit));
		ret = mt7932_6g_tpe_body(body, sizeof(body), &clear, NULL);
		if (ret < 0)
			return ret;
		ret = mt_request(m, 0x5d, true, true, false, body, 60);
		if (ret)
			return ret;
		m->six_class_dirty = false;
	}
	if (policy->six_channels)
		m->six_power_valid = true;
	/* Native SETs have no response/atomic policy-commit event. Command
	 * submission is not misrepresented as firmware readback or an ACK.
	 */
	dev_dbg(&m->pdev->dev, "REGULATORY_POLICY_SUBMITTED: %c%c firmware=%c%c channels=%u (native no-response SETs)\n",
		reg->alpha2[0], reg->alpha2[1], reg->domain[0], reg->domain[1],
		(reg->length - 12) / 8);
	return 0;
}

static int mt_privacy_defaults(struct mt7932 *m)
{
	static const unsigned int ids[] = {0,1,3,45,50,72,107,108,127,191,221,255};
	u8 body[60] = {};
	unsigned int i;
	int ret;

	if (m->stof_supported) {
		body[0] = 5;
		put_unaligned_le16(32, body + 2);
		put_unaligned_le32(6000, body + 4);
		put_unaligned_le32(6000, body + 20);
		ret = mt_request(m, 0x92, true, true, false, body, 36);
		if (ret)
			return ret;
	}
	memset(body, 0, sizeof(body));
	body[4] = 0x27;
	for (i = 0; i < ARRAY_SIZE(ids); i++)
		body[7 + ids[i] / 8] |= 1U << (ids[i] % 8);
	body[43] = 8;
	body[46] = 4;
	ret = mt_request(m, 0x9b, true, true, false, body, sizeof(body));
	if (ret)
		return ret;
	memset(body, 0, sizeof(body));
	body[0] = 1;
	body[5] = body[6] = 0x14;
	ret = mt_request(m, 0x9b, true, true, false, body, sizeof(body));
	if (ret)
		return ret;
	memset(body, 0, 4);
	body[0] = 2;
	return mt_request(m, 0xc5, true, true, false, body, 4);
}

static int mt_startup_once(struct mt7932 *m)
{
	int ret;
	u8 rm_cap[44] = {1,0,0,0,0,1,0x70,0,2};
	/* Original fullmac platform roam lock, not a scan-disable/DMA fence.
	 * Legacy CID3e is distinct from the RTS extended CIDed/ext3e.
	 */
	u8 roam_lock[12] = {0,0,0,0,0,0,0,1};

	m->startup_started = true;
	ret = mt_request(m, 0x5a, true, true, false, rm_cap, sizeof(rm_cap));
	if (!ret)
		ret = mt_stock_config(m);
	if (!ret)
		ret = mt_privacy_defaults(m);
	if (!ret)
		ret = mt_calibration_gate(m);
	if (!ret)
		ret = mt_request(m, 0x3e, true, true, false, roam_lock, sizeof(roam_lock));
	dev_dbg(&m->pdev->dev, "OWN_CALIBRATION_STARTUP_RESULT: %d\n", ret);
	if (!ret)
		ret = mt_enable_scan(m);
	if (ret)
		dev_err(&m->pdev->dev, "SCAN_INTERFACE_SETUP_FAILED: %d\n", ret);
	return ret;
}

static struct ieee80211_channel *mt_channel(struct mt7932 *m, unsigned int i)
{
	if (i < MT7932_CHANNELS_2G)
		return &m->channels[i];
	i -= MT7932_CHANNELS_2G;
	return i < MT7932_CHANNELS_5G ? &m->channels5[i] :
		&m->channels6[i - MT7932_CHANNELS_5G];
}

/* Keep cfg80211's channel list in step with the channels the country
 * package leaves out of the firmware domain. cfg80211 rebuilds the flags
 * on every regulatory update; the notifier applies the recorded set again
 * when the update keeps this package, or starts a new pass otherwise.
 */
static void mt_policy_disable(struct mt7932 *m, const struct mt7932_policy *policy,
			      u32 generation)
{
	unsigned int i;

	rtnl_lock();
	/* cfg80211 recomputes the channel flags for a new country under RTNL
	 * and only then calls the notifier, which starts a new generation.
	 * Once that has happened this package belongs to an older request,
	 * and applying it would disable channels on top of the new country's
	 * flags until the next regulatory change. The newer generation runs
	 * its own pass.
	 */
	if (generation != READ_ONCE(m->reg_generation)) {
		rtnl_unlock();
		return;
	}
	wiphy_lock(m->wiphy);
	for (i = 0; i < MT7932_CHANNELS; i++) {
		struct ieee80211_channel *channel = mt_channel(m, i);
		bool forbidden = !mt7932_policy_permits(policy, channel->band, channel->hw_value);

		if (!forbidden && channel->band == NL80211_BAND_6GHZ) {
			const struct ieee80211_reg_rule *rule;

			rule = freq_reg_info(m->wiphy, MHZ_TO_KHZ(channel->center_freq));
			if (!IS_ERR(rule) && !(rule->flags & NL80211_RRF_NO_IR))
				channel->flags &= ~IEEE80211_CHAN_NO_IR;
		}

		__assign_bit(i, m->policy_disabled, forbidden);
		if (forbidden)
			channel->flags |= IEEE80211_CHAN_DISABLED;
	}
	wiphy_unlock(m->wiphy);
	rtnl_unlock();
}

/* Retry only a completed file lookup that failed before any policy SET.
 * Consume the flag under the same lock as the worker publication, so repeated
 * userspace requests cannot continually invalidate an in-flight attempt.
 */
/* response_lock held: start the pass a deferred relaxation is waiting for. */
void mt_reg_apply_deferred_locked(struct mt7932 *m)
{
	lockdep_assert_held(&m->response_lock);
	if (!m->reg_deferral || m->stopping)
		return;
	m->reg_deferral = false;
	m->reg_desired = m->reg_deferred;
	m->reg_generation++;
	WRITE_ONCE(m->reg_pending, true);
	if (READ_ONCE(m->interface_registered))
		schedule_work(&m->startup_work);
}

void mt_retry_missing_policy(struct mt7932 *m)
{
	unsigned long flags;

	spin_lock_irqsave(&m->response_lock, flags);
	if (m->reg_retryable && m->reg_pending && m->interface_registered &&
	    !m->stopping && !m->policy_failed && !m->link_failed && !m->cal_state.error) {
		m->reg_retryable = false;
		m->reg_generation++;
		schedule_work(&m->startup_work);
	}
	spin_unlock_irqrestore(&m->response_lock, flags);
}

static int mt_policy_get(void *ctx, const char *path, struct mt7932_policy_file *file)
{
	struct mt7932 *m = ctx;
	const struct firmware *fw;
	int ret;

	/* Every candidate but the last is optional: no warning on a miss. */
	ret = request_firmware_direct(&fw, path, &m->pdev->dev);
	if (ret) {
		dev_dbg(&m->pdev->dev, "REGULATORY_PACKAGE_MISSING: %s: %d\n", path, ret);
		return ret;
	}
	file->handle = fw;
	file->data = fw->data;
	file->size = fw->size;
	return 0;
}

static void mt_policy_put(void *ctx, struct mt7932_policy_file *file)
{
	release_firmware(file->handle);
	release_firmware(file->class_handle);
	file->handle = NULL;
	file->class_handle = NULL;
}

/* command_mutex held. Reopen and authenticate the exact installed policy,
 * then publish the AP's class before EA/40. No active-class readback exists.
 */
int mt_6g_connect_policy(struct mt7932 *m, const struct mt7932_6g_ap *ap)
{
	struct mt7932_policy_file file = {};
	struct mt7932_policy *policy;
	struct mt7932_6g_limits limits;
	struct mt7932_reg_snapshot *reg;
	u8 hash[SHA256_DIGEST_SIZE], *body;
	char path[MT7932_POLICY_PATH_MAX];
	u8 tpe[60];
	s8 gain[3][4];
	bool sidecar = m->policy_choice.class_valid;
	u64 channels;
	unsigned long flags;
	unsigned int first, channel, begin, end;
	int ret, length;
	bool submitted = false;

	lockdep_assert_held(&m->command_mutex);
	ret = mt7932_j700_6g_gains(ap, gain);
	if (ret)
		return ret;
	ret = mt7932_6g_tpe_body(tpe, sizeof(tpe), ap, gain);
	if (ret < 0)
		return ret;
	policy = kzalloc_obj(*policy, GFP_KERNEL);
	reg = kzalloc_obj(*reg, GFP_KERNEL);
	body = kmalloc(1020, GFP_KERNEL);
	if (!policy || !reg || !body) {
		ret = -ENOMEM;
		goto free;
	}
	mt7932_policy_path(path, sizeof(path),
			   sidecar ? MT7932_POLICY_V4 : m->policy_choice.source,
			   m->policy_choice.alpha2);
	ret = mt_policy_get(m, path, &file);
	if (ret)
		goto free;
	sha256(file.data, file.size, hash);
	if (memcmp(hash,
		   sidecar ? m->policy_choice.class_sha256 :
			     m->policy_choice.sha256,
		   sizeof(hash))) {
		ret = -ESTALE;
		goto free;
	}
	ret = mt7932_policy_parse(policy, file.data, file.size,
				  m->policy_choice.domain);
	if (ret)
		goto free;
	channels = policy->six_class[ap->mode].channels;
	begin = ap->center - 2 * (BIT(ap->bandwidth) - 1);
	end = ap->center + 2 * (BIT(ap->bandwidth) - 1);
	for (channel = begin; channel <= end; channel += 4) {
		int frequency = ieee80211_channel_to_frequency(channel, NL80211_BAND_6GHZ);
		struct ieee80211_channel *c = ieee80211_get_channel(m->wiphy, frequency);

		if (!c || !(channels & BIT_ULL((channel - 1) / 4)) ||
		    mt7932_6g_class(ap->mode == MT7932_6G_LPI ? 0 : 2,
				    c->flags) < 0) {
			ret = -EACCES;
			goto free;
		}
	}
	spin_lock_irqsave(&m->response_lock, flags);
	*reg = m->reg_desired;
	spin_unlock_irqrestore(&m->response_lock, flags);
	limits = mt7932_6g_limits(reg);
	if (ap->mode == MT7932_6G_LPI) {
		/* The LP table, cfg80211 and AP envelope all remain upper bounds.
		 * Reusing VLP's fixed 14 dBm ceiling would obscure the LP data.
		 */
		limits.eirp = 254;
		for (channel = 0; channel < MT7932_CHANNELS_6G; channel++) {
			unsigned int index =
				mt7932_6g_reg_index(1 + 4 * channel);

			if (channels & BIT_ULL(channel))
				limits.eirp = min_t(int, limits.eirp,
						    2 * reg->power[index]);
		}
	}
	m->six_legacy_assoc = false;
	for (first = 0; first < MT7932_6G_RECORDS; first += 8) {
		length = mt7932_6g_class_page(body, 1020, policy, ap->mode,
					      m->policy_choice.domain, first);
		if (length < 0) {
			ret = length;
			goto free;
		}
		mt7932_6g_limit_page(body, &limits);
		submitted = true;
		m->six_class_dirty = true;
		ret = mt_request(m, 0x5d, true, true, false, body, length);
		if (ret)
			goto free;
	}
	dev_dbg(&m->pdev->dev, "CID5D_CLASS: byte1=%02x band=3 channel=%u\n",
		0x0a | (ap->mode << 4), ap->primary);
	ret = mt_request(m, 0x5d, true, true, false, tpe, sizeof(tpe));
	if (!ret)
		dev_dbg(&m->pdev->dev,
			"CID5D_TPE: length=60 band=3 channel=%u count=%u\n",
			ap->primary, ap->count);
free:
	/* No atomic SET/rollback exists. A partial publication closes RF
	 * admission; a checked firmware reset is required before reuse.
	 */
	if (ret && submitted) {
		spin_lock_irqsave(&m->response_lock, flags);
		mt_rf_fail_locked(m, ret);
		spin_unlock_irqrestore(&m->response_lock, flags);
	}
	if (file.handle)
		mt_policy_put(m, &file);
	kfree(body);
	kfree(reg);
	kfree(policy);
	return ret;
}

static void mt_policy_report(struct mt7932 *m, const struct mt7932_policy_choice *choice,
			     int ret)
{
	char path[MT7932_POLICY_PATH_MAX];
	unsigned int i;

	for (i = 0; i < choice->n_tried; i++) {
		if (choice->tried[i].error != -EINVAL)
			continue;
		mt7932_policy_path(path, sizeof(path), choice->tried[i].source, choice->alpha2);
		dev_warn_ratelimited(&m->pdev->dev, "local input %s has invalid policy: %d\n",
				     path, -EINVAL);
	}
	if (ret || choice->source != MT7932_POLICY_WORLD || !choice->fallback)
		return;
	dev_info(&m->pdev->dev, "REGULATORY_FALLBACK: %c%c firmware=XZ reason=%s, 6 GHz off\n",
		 choice->alpha2[0], choice->alpha2[1],
		 mt7932_policy_fallback_reason(choice->fallback));
}

static void mt_startup_work(struct work_struct *work)
{
	struct mt7932 *m = container_of(work, struct mt7932, startup_work);
	const struct mt7932_policy_loader loader = {
		.get = mt_policy_get, .put = mt_policy_put, .ctx = m,
	};
	struct mt7932_policy_choice choice;
	struct mt7932_policy_file file;
	struct mt7932_reg_snapshot *reg;
	struct mt7932_policy *policy;
	unsigned long flags;
	u32 generation;
	bool submitted;
	int ret;

	reg = kzalloc_obj(*reg, GFP_KERNEL);
	if (!reg) {
		dev_err(&m->pdev->dev,
			"REGULATORY_BLOCKED: snapshot allocation failed\n");
		return;
	}
	for (;;) {
		spin_lock_irqsave(&m->response_lock, flags);
		generation = m->reg_generation;
		*reg = m->reg_desired;
		/* Stopped for a sleep, the restart runs this generation again. */
		if (m->stopping || READ_ONCE(m->hif_suspended) || generation == m->reg_attempted) {
			spin_unlock_irqrestore(&m->response_lock, flags);
			kfree(reg);
			return;
		}
		m->reg_attempted = generation;
		m->reg_retryable = false;
		spin_unlock_irqrestore(&m->response_lock, flags);

		/* No command/wiphy lock while joining lifecycle workers. RX, D7
		 * calibration and TX-free remain alive until ownership is retired.
		 */
		if (m->netdev) {
			mt_disconnect(m->wiphy, m->netdev, WLAN_REASON_DEAUTH_LEAVING);
			/* Prefer natural completion: CID1b has no guaranteed stop ACK.
			 * reg_pending prevents another band batch being submitted.
			 */
			if (READ_ONCE(m->scan_request) || READ_ONCE(m->retired_scan_seq))
				wait_for_completion_timeout(&m->scan_done, msecs_to_jiffies(5000));
			mt_abort_scan(m->wiphy, &m->wdev);
			flush_work(&m->scan_finish_work);
			flush_work(&m->cal_work);
		}
		ret = reg->error;
		if (READ_ONCE(m->retired_scan_seq))
			ret = -EBUSY;
		memset(&file, 0, sizeof(file));
		memset(&choice, 0, sizeof(choice));
		memcpy(choice.alpha2, reg->alpha2, 2);
		policy = NULL;
		submitted = false;
		if (READ_ONCE(m->policy_failed) || READ_ONCE(m->link_failed) ||
		    (m->startup_started && !READ_ONCE(m->rf_ready)))
			ret = -EIO;
		if (!ret) {
			policy = kzalloc_obj(*policy, GFP_KERNEL);
			ret = policy ?
				      mt7932_policy_select(&loader, reg, policy,
							   &file, &choice) :
				      -ENOMEM;
			mt_policy_report(m, &choice, ret);
			if (!ret)
				mt_policy_disable(m, policy, generation);
		}
		mutex_lock(&m->command_mutex);
		if (READ_ONCE(m->stopping) || generation != READ_ONCE(m->reg_generation))
			goto next;
		if (!ret) {
			submitted = true;
			ret = mt_publish_policy(m, reg, policy);
			if (!ret && !m->startup_started)
				ret = mt_startup_once(m);
			/* A partial SET sequence is not a rollback. Only missing or
			 * invalid files BEFORE submission are recoverable by a hint.
			 */
		}
		spin_lock_irqsave(&m->response_lock, flags);
		if (!ret && (m->stopping || m->link_failed || m->cal_state.error))
			ret = m->cal_state.error ?: -EIO;
		if (ret && submitted)
			WRITE_ONCE(m->policy_failed, true);
		if (generation == m->reg_generation)
			m->reg_retryable = ret == -ENOENT && !submitted &&
				!m->stopping && !m->policy_failed && !m->link_failed &&
				!m->cal_state.error;
		if (!ret && generation == m->reg_generation)
			WRITE_ONCE(m->reg_pending, false);
		if (generation == m->reg_generation) {
			choice.generation = generation;
			choice.error = ret;
			m->policy_choice = choice;
		}
		if (ret)
			dev_warn_ratelimited(&m->pdev->dev,
					     "REGULATORY_BLOCKED: %c%c generation=%u error=%d recovery-required=%u\n",
					     reg->alpha2[0], reg->alpha2[1], generation, ret,
					     m->policy_failed || m->link_failed);
		else
			dev_dbg(&m->pdev->dev,
				"REGULATORY_READY: %c%c firmware=%c%c generation=%u current=%u\n",
				reg->alpha2[0], reg->alpha2[1], reg->domain[0],
				reg->domain[1], generation, !m->reg_pending);
		spin_unlock_irqrestore(&m->response_lock, flags);
next:
		mutex_unlock(&m->command_mutex);
		if (file.handle)
			mt_policy_put(m, &file);
		kfree(policy);
	}
}

static void mt_regulatory_notify(struct wiphy *wiphy, struct regulatory_request *request)
{
	struct mt7932 *m = mt_from_wiphy(wiphy);
	struct mt7932_reg_snapshot reg = {};
	unsigned long irqflags;
	unsigned int i, count = 0;

	/* cfg80211 calls after updating channels under RTNL in this kernel.
	 * Copy, never retain the transient request or take command_mutex here.
	 */
	if (!request)
		return;
	memcpy(reg.alpha2, request->alpha2, 2);
	/* Countries, 00 and cfg80211's numeric codes (98 intersection, 99
	 * custom); the numeric ones have no package and use the world one.
	 */
	if (!mt7932_policy_country(reg.alpha2) &&
	    !(isdigit(reg.alpha2[0]) && isdigit(reg.alpha2[1])))
		reg.error = -EOPNOTSUPP;
	memcpy(reg.domain, reg.alpha2, 2);
	/* Stock database-present startup resolves the absent 00 row to XZ.
	 * Keep Linux's world channel restrictions and requested identity; only
	 * the firmware country/domain and original power package use XZ.
	 * A country without a usable package of its own falls back the same
	 * way in mt7932_policy_select(), keeping its own channel list: the
	 * firmware domain is the intersection of both, so the radio stays up
	 * and is never more permissive than the country or the world profile.
	 */
	if (!memcmp(reg.alpha2, "00", 2))
		memcpy(reg.domain, "XZ", 2);
	reg.domain[4] = 1; /* 2 GHz remains 20 MHz. */
	reg.domain[5] = mt7932_domain_5g_bw();
	reg.domain[6] = 2;
	for (i = 0; i < MT7932_CHANNELS; i++) {
		struct ieee80211_channel *channel = mt_channel(m, i);
		u32 flags = channel->flags;

		if (channel->band == NL80211_BAND_6GHZ) {
			const struct ieee80211_reg_rule *rule;

			rule = freq_reg_info(m->wiphy, MHZ_TO_KHZ(channel->center_freq));
			if (!IS_ERR(rule) && !(rule->flags & NL80211_RRF_NO_IR))
				flags &= ~IEEE80211_CHAN_NO_IR;
		}

		reg.power[i] = channel->max_power;
		reg.psd[i] = channel->psd;
		if (flags & IEEE80211_CHAN_DISABLED)
			continue;
		/* Native per-rate units are not a proven dBm conversion. Do not
		 * silently ignore a numeric reduction below the qualified 20 dBm
		 * channel envelope or reinterpret sentinel bytes as power caps.
		 * A channel with a restriction the driver cannot honour is left
		 * out of the firmware domain and disabled for cfg80211 instead.
		 */
		if (flags & (IEEE80211_CHAN_NO_OFDM | IEEE80211_CHAN_NO_20MHZ |
			     IEEE80211_CHAN_IR_CONCURRENT) ||
		    (channel->band != NL80211_BAND_6GHZ &&
		     ((flags & IEEE80211_CHAN_PSD) || channel->max_power < 20)) ||
		    (channel->band == NL80211_BAND_6GHZ && channel->max_power < -64)) {
			channel->flags |= IEEE80211_CHAN_DISABLED;
			continue;
		}
		/* Radar channels carry cfg80211's RADAR flag: the station never
		 * probes them, and joins only an AP it has heard there.
		 */
		put_unaligned_le16(channel->hw_value, reg.domain + 12 + count * 8);
		/* Driver bandwidth limits are already in channel->flags. Adding
		 * the old blanket 20 MHz mask here would contradict the wiphy
		 * and silently force every native 5 GHz association back to 20.
		 */
		put_unaligned_le32(flags, reg.domain + 16 + count * 8);
		count++;
		reg.domain[8 + mt7932_channel_band(channel->band, channel->hw_value) - 1]++;
	}
	reg.length = 12 + count * 8;
	if (!count)
		reg.error = -EOPNOTSUPP;
	spin_lock_irqsave(&m->response_lock, irqflags);
	if (m->stopping) {
		spin_unlock_irqrestore(&m->response_lock, irqflags);
		return;
	}
	if (!m->reg_pending &&
	    ((m->connected && !m->disconnecting) || m->roam_moving) &&
	    mt7932_reg_only_relaxes(&m->reg_desired, &reg) &&
	    memcmp(&reg, &m->reg_desired, sizeof(reg))) {
		/* A pass would end the connection, or the move to another AP
		 * under way, only to permit more than the firmware now allows.
		 * Keep the stricter applied domain and run the pass once the
		 * connection has ended.
		 */
		m->reg_deferred = reg;
		m->reg_deferral = true;
		spin_unlock_irqrestore(&m->response_lock, irqflags);
		for_each_set_bit(i, m->policy_disabled, MT7932_CHANNELS)
			mt_channel(m, i)->flags |= IEEE80211_CHAN_DISABLED;
		dev_dbg(&m->pdev->dev, "REGULATORY_RELAXATION_DEFERRED: %c%c until disconnected\n",
			reg.alpha2[0], reg.alpha2[1]);
		return;
	}
	if (!memcmp(&reg, &m->reg_desired, sizeof(reg)) && !m->reg_pending) {
		m->reg_deferral = false;
		spin_unlock_irqrestore(&m->response_lock, irqflags);
		/* The applied package still holds, but cfg80211 has just
		 * rebuilt the channel flags without its disables.
		 */
		for_each_set_bit(i, m->policy_disabled, MT7932_CHANNELS)
			mt_channel(m, i)->flags |= IEEE80211_CHAN_DISABLED;
		return;
	}
	m->reg_desired = reg;
	m->six_ap_valid = false;
	m->reg_deferral = false;
	m->reg_generation++;
	/* The next package pass records its own set. */
	bitmap_zero(m->policy_disabled, MT7932_CHANNELS);
	WRITE_ONCE(m->reg_pending, true);
	if (m->netdev) {
		netif_stop_queue(m->netdev);
		/* The stop can land in the middle of a burst: hand the device the
		 * TX0 descriptors already published behind the doorbell.
		 */
		if (READ_ONCE(m->data_ready)) {
			spin_lock(&m->data_lock);
			mt_data_kick_locked(m);
			spin_unlock(&m->data_lock);
		}
		if (m->connecting) {
			m->connect_error = -ECANCELED;
			complete(&m->assoc_start);
			complete(&m->assoc_done);
			complete(&m->discovery_done);
		}
	}
	if (READ_ONCE(m->interface_registered))
		schedule_work(&m->startup_work);
	spin_unlock_irqrestore(&m->response_lock, irqflags);
	dev_dbg(&m->pdev->dev, "REGULATORY_REQUEST: %c%c initiator=%u generation=%u\n",
		reg.alpha2[0], reg.alpha2[1], request->initiator, m->reg_generation);
}

static const struct cfg80211_ops mt_cfg_ops = {
	.scan = mt_scan, .abort_scan = mt_abort_scan,
	.connect = mt_connect, .disconnect = mt_disconnect,
	.deauth = mt_deauth,
	.change_virtual_intf = mt_change_interface,
	.get_station = mt_get_station,
	.get_channel = mt_get_channel,
	.dump_station = mt_dump_station,
	.set_power_mgmt = mt_set_power_mgmt,
	.set_cqm_rssi_config = mt_set_cqm_rssi_config,
#if IS_ENABLED(CONFIG_MT7932_AWDL)
	.add_station = mt_awdl_add_station,
	.del_station = mt_awdl_del_station,
	.change_station = mt_awdl_change_station,
#endif
#ifdef CONFIG_PM
	.suspend = mt_wow_suspend,
	.resume = mt_wow_resume,
#endif
};

int mt_register_regulatory_gate(struct mt7932 *m)
{
	static const int rates[] = {10,20,55,110,60,90,120,180,240,360,480,540};
	static const u32 ciphers[] = { WLAN_CIPHER_SUITE_CCMP,
				       WLAN_CIPHER_SUITE_TKIP,
				       WLAN_CIPHER_SUITE_AES_CMAC };
	static const u32 akms[] = {
		WLAN_AKM_SUITE_PSK,
		WLAN_AKM_SUITE_PSK_SHA256,
		WLAN_AKM_SUITE_SAE,
		WLAN_AKM_SUITE_OWE,
	};
	unsigned int i;
	int ret;

	INIT_WORK(&m->startup_work, mt_startup_work);
	m->reg_pending = true;
	m->wiphy = wiphy_new(&mt_cfg_ops, sizeof(m));
	if (!m->wiphy)
		return -ENOMEM;
	*(struct mt7932 **)wiphy_priv(m->wiphy) = m;
	set_wiphy_dev(m->wiphy, &m->pdev->dev);
	ether_addr_copy(m->wiphy->perm_addr, m->mac);
	m->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	mt_awdl_wiphy_setup(m);
	m->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	m->wiphy->cipher_suites = ciphers;
	m->wiphy->n_cipher_suites = ARRAY_SIZE(ciphers);
	m->wiphy->akm_suites = akms;
	m->wiphy->n_akm_suites = ARRAY_SIZE(akms);
	wiphy_ext_feature_set(m->wiphy, NL80211_EXT_FEATURE_4WAY_HANDSHAKE_STA_PSK);
	wiphy_ext_feature_set(m->wiphy, NL80211_EXT_FEATURE_SAE_OFFLOAD);
	wiphy_ext_feature_set(m->wiphy, NL80211_EXT_FEATURE_OWE_OFFLOAD);
	wiphy_ext_feature_set(m->wiphy, NL80211_EXT_FEATURE_MFP_OPTIONAL);
	m->wiphy->max_scan_ssids = MT7932_SCAN_SSIDS;
	m->wiphy->max_scan_ie_len = 600;
#ifdef CONFIG_PM
	m->wiphy->wowlan = &mt_wowlan_support;
#endif
	m->wiphy->reg_notifier = mt_regulatory_notify;
	/* World roaming may relax NO_IR after observing an AP beacon without
	 * changing alpha2. Request the callback so firmware's CID0f domain is
	 * updated through the same idle/scan fence as other regulatory changes.
	 */
	m->wiphy->flags |= WIPHY_FLAG_CHANNEL_CHANGE_ON_BEACON;
	/* The native fullmac path applies policy at idle, not seamlessly inside
	 * an association. Keep the existing explicit user/core-country model:
	 * accepting this STA's AP hint then disconnecting to apply it would make
	 * cfg80211 restore the user country and repeat the join forever.
	 * Global hints from other radios still reach the notifier and are handled.
	 */
	m->wiphy->regulatory_flags |= REGULATORY_COUNTRY_IE_IGNORE;
	for (i = 0; i < MT7932_CHANNELS_2G; i++) {
		m->channels[i].band = NL80211_BAND_2GHZ;
		m->channels[i].center_freq = 2412 + 5 * i;
		m->channels[i].hw_value = i + 1;
		m->channels[i].max_power = 20;
		m->channels[i].flags = IEEE80211_CHAN_NO_HT40 | IEEE80211_CHAN_NO_80MHZ | IEEE80211_CHAN_NO_160MHZ;
	}
	for (i = 0; i < ARRAY_SIZE(rates); i++) {
		m->rates[i].bitrate = rates[i];
		m->rates[i].hw_value = i;
	}
	m->band2.band = NL80211_BAND_2GHZ;
	m->band2.channels = m->channels;
	m->band2.n_channels = ARRAY_SIZE(m->channels);
	m->band2.bitrates = m->rates;
	m->band2.n_bitrates = ARRAY_SIZE(m->rates);
	m->wiphy->bands[NL80211_BAND_2GHZ] = &m->band2;
	{
		for (i = 0; i < ARRAY_SIZE(m->channels5); i++) {
			unsigned int primary = mt7932_channel_5g(i);
			unsigned int pair = mt7932_channel_ht40_centre(primary);
			u32 flags = IEEE80211_CHAN_NO_160MHZ;

			/* The secondary channel sits on the far side of the pair centre. */
			if (!pair)
				flags |= IEEE80211_CHAN_NO_HT40 | IEEE80211_CHAN_NO_80MHZ;
			else if (pair < primary)
				flags |= IEEE80211_CHAN_NO_HT40PLUS;
			else
				flags |= IEEE80211_CHAN_NO_HT40MINUS;
			m->channels5[i].band = NL80211_BAND_5GHZ;
			m->channels5[i].center_freq = 5000 + 5 * primary;
			m->channels5[i].hw_value = primary;
			m->channels5[i].max_power = 20;
			m->channels5[i].flags = flags;
			if (MT7932_MAX_5G_BW < 2)
				m->channels5[i].flags |= IEEE80211_CHAN_NO_80MHZ;
			if (!MT7932_MAX_5G_BW)
				m->channels5[i].flags |= IEEE80211_CHAN_NO_HT40;
		}
		m->band5.band = NL80211_BAND_5GHZ;
		m->band5.channels = m->channels5;
		m->band5.n_channels = ARRAY_SIZE(m->channels5);
		m->band5.bitrates = m->rates + 4;
		m->band5.n_bitrates = ARRAY_SIZE(m->rates) - 4;
		m->wiphy->bands[NL80211_BAND_5GHZ] = &m->band5;
	}
	if (mt7932_phy_profile(m->phy_cap)) {
		mt7932_band_capabilities(&m->band2, false);
		mt7932_band_capabilities(&m->band5, true);
		if (m->six_ghz) {
			for (i = 0; i < MT7932_CHANNELS_6G; i++) {
				struct ieee80211_channel *ch = &m->channels6[i];
				unsigned int pair, primary = mt7932_channel_6g(i);

				ch->band = NL80211_BAND_6GHZ;
				ch->hw_value = primary;
				ch->center_freq = ieee80211_channel_to_frequency(primary, ch->band);
				ch->max_power = 14;
				ch->flags = IEEE80211_CHAN_NO_IR | IEEE80211_CHAN_NO_160MHZ;
				pair = mt7932_channel_6g_center(primary, false);
				if (!pair)
					ch->flags |= IEEE80211_CHAN_NO_HT40;
				else
					ch->flags |= pair < primary ? IEEE80211_CHAN_NO_HT40PLUS :
						IEEE80211_CHAN_NO_HT40MINUS;
				if (!mt7932_channel_6g_center(primary, true))
					ch->flags |= IEEE80211_CHAN_NO_80MHZ;
			}
			m->band6.band = NL80211_BAND_6GHZ;
			m->band6.channels = m->channels6;
			m->band6.n_channels = MT7932_CHANNELS_6G;
			m->band6.bitrates = m->rates + 4;
			m->band6.n_bitrates = ARRAY_SIZE(m->rates) - 4;
			mt7932_he_6g_capabilities(&m->he6, &m->band5);
			m->band6.iftype_data = &m->he6;
			m->band6.n_iftype_data = 1;
			m->wiphy->bands[NL80211_BAND_6GHZ] = &m->band6;
			m->wiphy->flags |= WIPHY_FLAG_SPLIT_SCAN_6GHZ;
		}
	}
	ret = wiphy_register(m->wiphy);
	if (ret) {
		wiphy_free(m->wiphy);
		m->wiphy = NULL;
		return ret;
	}
	m->wiphy_registered = true;
	mt_policy_debugfs_init(m);
	ret = mt_register_interface(m);
	if (ret)
		return ret;
	ret = mt_awdl_register(m);
	if (ret)
		return ret;
	WRITE_ONCE(m->interface_registered, true);
	schedule_work(&m->startup_work);
	dev_dbg(&m->pdev->dev, "REGULATORY_GATE_REGISTERED: cfg80211 country policy required\n");
	return 0;
}

/* Country package selection. The lookup goes through a loader so the order,
 * the version rules and the world fallback can be tested without the
 * firmware loader.
 */
static const char *mt7932_policy_fallback_reason(int fallback)
{
	switch (fallback) {
	case 0:
		return "none";
	case -ENOENT:
		return "missing";
	case -EOPNOTSUPP:
		return "no-country-code";
	default:
		return "invalid";
	}
}

static const char *const mt7932_policy_names[] = {
	[MT7932_POLICY_NONE] = "none",	  [MT7932_POLICY_V4] = "policy-v4",
	[MT7932_POLICY_V2] = "policy-v2", [MT7932_POLICY_V1] = "policy",
	[MT7932_POLICY_WORLD] = "world",
};

unsigned int mt7932_policy_candidates(const u8 alpha2[2],
				      enum mt7932_policy_source sources[MT7932_POLICY_CANDIDATES])
{
	unsigned int n = 0;

	/* 00 and the other numeric codes have no country package. */
	if (mt7932_policy_country(alpha2)) {
		sources[n++] = MT7932_POLICY_V4;
		sources[n++] = MT7932_POLICY_V2;
		sources[n++] = MT7932_POLICY_V1;
	}
	sources[n++] = MT7932_POLICY_WORLD;
	return n;
}

void mt7932_policy_path(char *buf, size_t len, enum mt7932_policy_source source,
			const u8 alpha2[2])
{
	switch (source) {
	case MT7932_POLICY_V4:
		scnprintf(buf, len, "mediatek/mt7932/policy-v4/%c%c.bin",
			  alpha2[0], alpha2[1]);
		break;
	case MT7932_POLICY_V2:
		scnprintf(buf, len, "mediatek/mt7932/policy-v2/%c%c.bin", alpha2[0], alpha2[1]);
		break;
	case MT7932_POLICY_V1:
		scnprintf(buf, len, "mediatek/mt7932/policy/%c%c.bin", alpha2[0], alpha2[1]);
		break;
	case MT7932_POLICY_WORLD:
		scnprintf(buf, len, "mediatek/mt7932/policy/world-XZ.bin");
		break;
	default:
		scnprintf(buf, len, "-");
		break;
	}
}

/*
 * policy/ holds version 1 only: kernels that predate version 2 read that
 * directory and reject anything else, so a version 2 package there is a
 * packaging error rather than a newer package. policy-v2/ holds version 2
 * only.
 */
static int mt7932_policy_check_version(enum mt7932_policy_source source,
				       const u8 *data, size_t size)
{
	unsigned int version = size >= 6 ? get_unaligned_le16(data + 4) : 0;
	unsigned int expected = 1;

	if (source == MT7932_POLICY_V4)
		expected = 4;
	else if (source == MT7932_POLICY_V2)
		expected = 2;
	return version == expected ? 0 : -EINVAL;
}

/* Prefer the original v2 VLP publication whenever it is installed. Retain
 * v4's LP view as a sidecar, rather than replacing the working VLP pages or
 * changing the two post-queue association SETs. Both files stay pinned until
 * publication and both hashes are checked again before class selection.
 */
static int mt7932_policy_keep_vlp(const struct mt7932_policy_loader *loader,
				  struct mt7932_policy *policy,
				  struct mt7932_policy_file *file,
				  struct mt7932_policy_choice *choice)
{
	struct mt7932_policy_file legacy = {};
	struct mt7932_policy *old;
	struct mt7932_6g_table lpi = policy->six_class[MT7932_6G_LPI];
	struct mt7932_6g_table vlp = policy->six_class[MT7932_6G_VLP];
	char path[MT7932_POLICY_PATH_MAX];

	old = kzalloc_obj(*old, GFP_KERNEL);
	if (!old)
		return -ENOMEM;
	mt7932_policy_path(path, sizeof(path), MT7932_POLICY_V2,
			   choice->alpha2);
	if (loader->get(loader->ctx, path, &legacy))
		goto free;
	if (mt7932_policy_check_version(MT7932_POLICY_V2, legacy.data,
					legacy.size) ||
	    mt7932_policy_parse(old, legacy.data, legacy.size,
				choice->alpha2)) {
		loader->put(loader->ctx, &legacy);
		goto free;
	}
	sha256(file->data, file->size, choice->class_sha256);
	choice->class_valid = true;
	legacy.class_handle = file->handle;
	*file = legacy;
	*policy = *old;
	policy->six_class[MT7932_6G_LPI] = lpi;
	policy->six_class[MT7932_6G_VLP] = vlp;
	policy->six_channels |= lpi.channels | vlp.channels;
	choice->source = MT7932_POLICY_V2;
free:
	kfree(old);
	return 0;
}

int mt7932_policy_select(const struct mt7932_policy_loader *loader,
			 struct mt7932_reg_snapshot *reg, struct mt7932_policy *policy,
			 struct mt7932_policy_file *file, struct mt7932_policy_choice *choice)
{
	enum mt7932_policy_source sources[MT7932_POLICY_CANDIDATES];
	unsigned int i, n = mt7932_policy_candidates(reg->alpha2, sources);
	char path[MT7932_POLICY_PATH_MAX];
	int country = mt7932_policy_country(reg->alpha2) ? 0 : -EOPNOTSUPP;
	int ret = -ENOENT;

	memset(choice, 0, sizeof(*choice));
	memcpy(choice->alpha2, reg->alpha2, 2);
	memset(file, 0, sizeof(*file));
	for (i = 0; i < n; i++) {
		const u8 *alpha2 = sources[i] == MT7932_POLICY_WORLD ? (const u8 *)"XZ" :
			reg->alpha2;

		mt7932_policy_path(path, sizeof(path), sources[i], reg->alpha2);
		choice->tried[i].source = sources[i];
		ret = loader->get(loader->ctx, path, file);
		if (!ret) {
			ret = mt7932_policy_check_version(sources[i], file->data, file->size);
			if (!ret)
				ret = mt7932_policy_parse(policy, file->data, file->size, alpha2);
			if (ret) {
				loader->put(loader->ctx, file);
				memset(file, 0, sizeof(*file));
			}
		}
		choice->tried[i].error = ret;
		choice->n_tried = i + 1;
		if (!ret) {
			choice->source = sources[i];
			break;
		}
		/* A present but unusable country package is the more useful
		 * reason to report than a missing second candidate.
		 */
		if (sources[i] != MT7932_POLICY_WORLD && country != -EINVAL)
			country = ret;
	}
	if (ret)
		return ret;

	if (choice->source == MT7932_POLICY_V4) {
		ret = mt7932_policy_keep_vlp(loader, policy, file, choice);
		if (ret) {
			loader->put(loader->ctx, file);
			memset(file, 0, sizeof(*file));
			return ret;
		}
	}

	/*
	 * A country without a usable package of its own keeps its cfg80211
	 * identity and channel list; the firmware gets the world profile and
	 * the CID0f domain loses every channel the world package forbids. The
	 * result is the intersection of both, never more permissive than
	 * either. The world package has no 6 GHz section, so 6 GHz stays off.
	 */
	if (choice->source == MT7932_POLICY_WORLD) {
		memcpy(reg->domain, "XZ", 2);
		if (memcmp(reg->alpha2, "00", 2))
			choice->fallback = country;
	}
	mt7932_policy_filter(reg, policy);
	memcpy(choice->domain, reg->domain, 2);
	choice->version = get_unaligned_le16(file->data + 4);
	choice->size = file->size;
	sha256(file->data, file->size, choice->sha256);
	choice->six_primaries = reg->domain[10];
	return 0;
}

static int mt_policy_show(struct seq_file *s, void *unused)
{
	struct mt7932 *m = s->private;
	struct mt7932_policy_choice *c;
	char path[MT7932_POLICY_PATH_MAX];
	unsigned long flags;
	unsigned int i;
	bool pending;

	c = kmalloc_obj(*c, GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	spin_lock_irqsave(&m->response_lock, flags);
	*c = m->policy_choice;
	pending = m->reg_pending;
	spin_unlock_irqrestore(&m->response_lock, flags);

	if (!c->alpha2[0]) {
		seq_puts(s, "state: none\n");
		goto out;
	}
	mt7932_policy_path(path, sizeof(path), c->source, c->alpha2);
	seq_printf(s, "country: %c%c\n", c->alpha2[0], c->alpha2[1]);
	if (c->error)
		seq_printf(s, "state: blocked (%d)\n", c->error);
	else
		seq_printf(s, "state: %s\n", pending ? "applied, update pending" : "applied");
	seq_printf(s, "generation: %u\n", c->generation);
	if (c->source != MT7932_POLICY_NONE) {
		seq_printf(s, "firmware_domain: %c%c\n", c->domain[0], c->domain[1]);
		seq_printf(s, "source: %s\n", mt7932_policy_names[c->source]);
		seq_printf(s, "path: %s\n", path);
		seq_printf(s, "version: %u\n", c->version);
		seq_printf(s, "size: %u\n", c->size);
		seq_printf(s, "sha256: %8phN\n", c->sha256);
		seq_printf(s, "6ghz_primaries: %u\n", c->six_primaries);
		if (c->class_valid)
			seq_printf(s, "class_source: policy-v4\nclass_sha256: %*phN\n",
				   SHA256_DIGEST_SIZE, c->class_sha256);
	}
	seq_printf(s, "fallback: %s\n", mt7932_policy_fallback_reason(c->fallback));
	for (i = 0; i < c->n_tried; i++) {
		mt7932_policy_path(path, sizeof(path), c->tried[i].source, c->alpha2);
		seq_printf(s, "tried: %s %d\n", path, c->tried[i].error);
	}
out:
	kfree(c);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt_policy);

void mt_policy_debugfs_init(struct mt7932 *m)
{
	debugfs_create_file("mt7932_policy", 0400, m->wiphy->debugfsdir, m, &mt_policy_fops);
}
