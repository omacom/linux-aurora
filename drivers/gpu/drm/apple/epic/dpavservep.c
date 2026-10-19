// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright The Asahi Linux Contributors */

#include "dpavservep.h"
#include "dpavservep-edid.h"
#include "dpavserv-edid.h"

#define EDID_EXT_BLOCK_COUNT_OFFSET 0x7E

#include <drm/drm_edid.h>

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "../afk.h"
#include "../dcp.h"
#include "../dcp-internal.h"
#include "../connector.h"
#include "../trace.h"

static void dcpavserv_notify_edid(struct apple_epic_service *service)
{
	struct apple_dcp *dcp = service->ep->dcp;
	struct apple_connector *connector = READ_ONCE(dcp->connector);

	if (!service->ep->wire_v2 || READ_ONCE(dcp->quiescing) || !connector ||
	    !READ_ONCE(dcp->nr_modes) || !READ_ONCE(connector->connected) ||
	    READ_ONCE(connector->base.registration_state) != DRM_CONNECTOR_REGISTERED ||
	    READ_ONCE(connector->dcp) != to_platform_device(dcp->dev))
		return;
	/* get_modes may already have run before this service was announced. */
	schedule_work(&connector->hotplug_wq);
}

static bool dcpavserv_identity(struct apple_epic_service *service, const char *name,
			       const char *class, s64 unit)
{
	if (!name || strcmp(name, "dcpav-service-epic"))
		return false;
	if (!service->ep->wire_v2)
		return unit == 0;
	/* H17P's lower-case interface-id is dynamic, not a physical-port unit. */
	return service->ep->endpoint == DPAVSERV_ENDPOINT_V2 && class &&
		(!strcmp(class, "AppleDCPDP2HDMI") ||
		 !strcmp(class, "DCPDP13Service")) && service->channel &&
		service->channel <= U16_MAX && unit == service->channel;
}

static bool dcpavserv_admission_current(struct apple_dcp *dcp, u64 generation)
{
	lockdep_assert_held(&dcp->hpd_mutex);
	/* Initial service discovery may precede the first Type-C attachment. */
	return !READ_ONCE(dcp->quiescing) && !READ_ONCE(dcp->external_physically_absent) &&
		generation == dcp->typec_generation &&
		(!generation || dcp->typec_cable_connected);
}

/* Admission runs before AFKV2 OPEN and before readers can pin the service. */
static int dcpavserv_prepare(struct apple_epic_service *service, const char *name,
			    const char *class, s64 unit)
{
	struct apple_dcp *dcp = service->ep->dcp;
	bool v2 = service->ep->wire_v2;
	int ret = 0;

	if (!dcpavserv_identity(service, name, class, unit))
		return -ENODEV;
	if (v2)
		mutex_lock(&dcp->hpd_mutex);
	mutex_lock(&dcp->dcpavserv.lock);
	if (dcp->dcpavserv.enabled)
		ret = -EBUSY;
	else if (v2 && !dcpavserv_admission_current(dcp, dcp->typec_generation))
		ret = -ENODEV;
	else {
		service->admission.generation = v2 ? dcp->typec_generation : 0;
		service->admission.epoch = dcp->dcpavserv.epoch;
	}
	mutex_unlock(&dcp->dcpavserv.lock);
	if (v2)
		mutex_unlock(&dcp->hpd_mutex);
	return ret;
}

static int dcpavserv_revalidate(struct apple_epic_service *service, const char *name,
				const char *class, s64 unit)
{
	struct apple_dcp *dcp = service->ep->dcp;
	int ret = 0;

	if (!dcpavserv_identity(service, name, class, unit))
		return -ENODEV;
	mutex_lock(&dcp->hpd_mutex);
	mutex_lock(&dcp->dcpavserv.lock);
	if (dcp->dcpavserv.enabled)
		ret = -EBUSY;
	else if (service->admission.epoch != dcp->dcpavserv.epoch ||
		 !dcpavserv_admission_current(dcp, service->admission.generation))
		ret = -ENODEV;
	mutex_unlock(&dcp->dcpavserv.lock);
	mutex_unlock(&dcp->hpd_mutex);
	return ret;
}

static void dcpavserv_init_h17p(struct apple_epic_service *service, const char *name,
			       const char *class, s64 unit)
{
	struct apple_dcp *dcp = service->ep->dcp;
	bool v2 = service->ep->wire_v2;
	bool published = false;

	trace_dcpavserv_init(dcp, unit);
	if (v2)
		mutex_lock(&dcp->hpd_mutex);
	mutex_lock(&dcp->dcpavserv.lock);
	if (dcpavserv_identity(service, name, class, unit) &&
	    !dcp->dcpavserv.enabled &&
	    service->admission.epoch == dcp->dcpavserv.epoch &&
	    (!v2 || dcpavserv_admission_current(dcp, service->admission.generation))) {
		dcp->dcpavserv.service = afk_service_get(service);
		if (dcp->dcpavserv.service) {
			dcp->dcpavserv.enabled = true;
			dcp->dcpavserv.epoch++;
			service->cookie = &dcp->dcpavserv;
			complete(&dcp->dcpavserv.enable_completion);
			published = true;
		}
	}
	mutex_unlock(&dcp->dcpavserv.lock);
	if (v2)
		mutex_unlock(&dcp->hpd_mutex);
	if (!published) {
		/* A generation/registry change during OPEN cannot enable stale work. */
		afk_service_retire(service);
		return;
	}
	if (v2) {
		dev_dbg(dcp->dev, "DCPAVSERV: AFKV2 EDID service ready on channel %u\n",
			service->channel);
		dcpavserv_notify_edid(service);
	}
}

/* Defer DRM cleanup: AFK callbacks must not take mode_config.mutex. */
static void dcpavserv_invalidate_edid(struct apple_dcp *dcp)
{
	struct apple_connector *connector = READ_ONCE(dcp->connector);

	lockdep_assert_held(&dcp->dcpavserv.lock);
	if (!connector || READ_ONCE(connector->dcp) != to_platform_device(dcp->dev))
		return;
	WRITE_ONCE(connector->edid_invalidation_pending, true);
	if (!READ_ONCE(dcp->quiescing) &&
	    READ_ONCE(connector->base.registration_state) == DRM_CONNECTOR_REGISTERED)
		schedule_work(&connector->hotplug_wq);
}

static void dcpavserv_teardown_h17p(struct apple_epic_service *service)
{
	struct apple_dcp *dcp = service->ep->dcp;
	bool owned = false;

	mutex_lock(&dcp->dcpavserv.lock);
	if (dcp->dcpavserv.service == service) {
		dcp->dcpavserv.enabled = false;
		dcp->dcpavserv.service = NULL;
		dcp->dcpavserv.epoch++;
		dcpavserv_invalidate_edid(dcp);
		reinit_completion(&dcp->dcpavserv.enable_completion);
		owned = true;
	}
	mutex_unlock(&dcp->dcpavserv.lock);
	if (owned)
		afk_service_put(service);
}

/*
 * Every firmware other than H17P (M1/M2/M3): integration's service
 * lifetime. The H17P variants below add admission, service epochs and
 * connector EDID invalidation.
 */
static void dcpavserv_init(struct apple_epic_service *service, const char *name,
			  const char *class, s64 unit)
{
	struct apple_dcp *dcp = service->ep->dcp;

	trace_dcpavserv_init(dcp, unit);

	if (unit == 0 && name && !strcmp(name, "dcpav-service-epic")) {
		mutex_lock(&dcp->dcpavserv.lock);
		if (dcp->dcpavserv.enabled) {
			mutex_unlock(&dcp->dcpavserv.lock);
			dev_err(dcp->dev,
				"DCPAVSERV: unit %lld already exists\n", unit);
			return;
		}
		dcp->dcpavserv.service = afk_service_get(service);
		dcp->dcpavserv.enabled = true;
		/* A new instance: the host is no client of it yet. */
		dcp->dcpavserv.opened = NULL;
		dcp->dcpavserv.terminated = NULL;
		service->cookie = &dcp->dcpavserv;
		complete(&dcp->dcpavserv.enable_completion);
		mutex_unlock(&dcp->dcpavserv.lock);
	}
}

static void dcpavserv_teardown(struct apple_epic_service *service)
{
	struct apple_dcp *dcp = service->ep->dcp;
	bool owned = false;

	mutex_lock(&dcp->dcpavserv.lock);
	if (dcp->dcpavserv.service == service) {
		dcp->dcpavserv.enabled = false;
		dcp->dcpavserv.service = NULL;
		/* A removed instance took its open with it. */
		dcp->dcpavserv.opened = NULL;
		dcp->dcpavserv.terminated = NULL;
		reinit_completion(&dcp->dcpavserv.enable_completion);
		owned = true;
	}
	mutex_unlock(&dcp->dcpavserv.lock);
	if (owned)
		afk_service_put(service);
}

void dpavservep_detach(struct apple_dcp *dcp)
{
	struct apple_epic_service *service;

	if (dcp->fw_compat != DCP_FIRMWARE_H17P) {
		mutex_lock(&dcp->dcpavserv.lock);
		service = dcp->dcpavserv.service;
		dcp->dcpavserv.service = NULL;
		dcp->dcpavserv.enabled = false;
		dcp->dcpavserv.opened = NULL;
		dcp->dcpavserv.terminated = NULL;
		mutex_unlock(&dcp->dcpavserv.lock);
		cancel_work_sync(&dcp->dcpavserv.close_wq);
		if (service) {
			afk_service_disable(service);
			afk_service_put(service);
		}
		return;
	}

	mutex_lock(&dcp->dcpavserv.lock);
	service = dcp->dcpavserv.service;
	dcp->dcpavserv.service = NULL;
	dcp->dcpavserv.enabled = false;
	/* Also reject an in-flight prepare/OPEN with no published owner yet. */
	dcp->dcpavserv.epoch++;
	reinit_completion(&dcp->dcpavserv.enable_completion);
	if (service) {
		afk_service_retire(service);
		dcpavserv_invalidate_edid(dcp);
	}
	mutex_unlock(&dcp->dcpavserv.lock);
	if (service)
		afk_service_put(service);
}

static int dcpdpserv_prepare(struct apple_epic_service *service, const char *name,
			    const char *class, s64 unit)
{
	/* No production command transport is needed for this H17P service. */
	return service->ep->wire_v2 ? -EOPNOTSUPP : 0;
}

static void dcpdpserv_init(struct apple_epic_service *service, const char *name,
			  const char *class, s64 unit)
{
}

static void dcpdpserv_teardown(struct apple_epic_service *service)
{
	afk_service_disable(service);
}

struct dcpavserv_status_report {
	u32 unk00[4];
	u8 flag0;
	u8 flag1;
	u8 flag2;
	u8 flag3;
	u32 unk14[3];
	u32 status;
	u32 unk24[3];
} __packed;

struct dpavserv_copy_edid_cmd {
	__le64 max_size;
	u8 _pad1[24];
	__le64 used_size;
	u8 _pad2[8];
} __packed;

static_assert(sizeof(struct dpavserv_copy_edid_resp) == 48);

static int parse_report(struct apple_epic_service *service, enum epic_subtype type,
			 const void *data, size_t data_size)
{
#if defined(DEBUG)
	struct apple_dcp *dcp = service->ep->dcp;
	const struct epic_service_call *call;
	const void *payload;
	size_t payload_size;

	dev_dbg(dcp->dev, "dcpavserv[ch:%u]: report type:%02x len:%zu\n",
		service->channel, type, data_size);

	if (type != EPIC_SUBTYPE_STD_SERVICE)
		return 0;

	if (data_size < sizeof(*call))
		return 0;

	call = data;

	if (le32_to_cpu(call->magic) != EPIC_SERVICE_CALL_MAGIC) {
		dev_warn(dcp->dev, "dcpavserv[ch:%u]: report magic 0x%08x != 0x%08x\n",
			service->channel, le32_to_cpu(call->magic), EPIC_SERVICE_CALL_MAGIC);
		return 0;
	}

	payload_size = data_size - sizeof(*call);
	if (payload_size < le32_to_cpu(call->data_len)) {
		dev_warn(dcp->dev, "dcpavserv[ch:%u]: report payload size %zu call len %u\n",
			service->channel, payload_size, le32_to_cpu(call->data_len));
		return 0;
	}
	payload_size = le32_to_cpu(call->data_len);
	payload = data + sizeof(*call);

	if (le16_to_cpu(call->group) == 2 && le16_to_cpu(call->command) == 0) {
		if (payload_size == sizeof(struct dcpavserv_status_report)) {
			const struct dcpavserv_status_report *stat = payload;
			dev_info(dcp->dev, "dcpavserv[ch:%u]: flags: 0x%02x,0x%02x,0x%02x,0x%02x status:%u\n",
				service->channel, stat->flag0, stat->flag1,
				stat->flag2, stat->flag3, stat->status);
		} else {
			dev_dbg(dcp->dev, "dcpavserv[ch:%u]: report payload size %zu\n", service->channel, payload_size);
		}
	} else {
		print_hex_dump(KERN_DEBUG, "dcpavserv report: ", DUMP_PREFIX_NONE,
			       16, 1, payload, payload_size, true);
	}
#endif

	return 0;
}

/*
 * The 14.7 external role: the host opens the published AV instance as a
 * client (DCPAV Open, command 4, a zero 32-byte body; the reply carries a
 * status at +0 and the opened Boolean at +16) to read the sink's EDID
 * through it. An open client is one the firmware waits for, up to 10 s,
 * before it removes the instance: it holds the reply to the sink's HPD low
 * until the instance's device has closed, and with it the sink's release and
 * the next sink. So the client is closed (DCPAV Close, command 5, a zero
 * 16-byte body, status at +0) before every HPD low and when the firmware
 * terminates the instance (report group 2, command 1). Close goes only to the
 * instance that was opened; a removed instance took its open with it.
 * Neither call is made under dcpavserv.lock, which never waits for firmware.
 */
static int dcpavserv_v14_open(struct apple_dcp *dcp, struct apple_epic_service *service)
{
	u8 reply[32] = {};
	int ret;

	ret = afk_service_call(service, 0, 4, NULL, 0, 32, reply, sizeof(reply), 0);
	if (ret)
		return ret;
	if (get_unaligned_le32(reply) || !get_unaligned_le64(reply + 16)) {
		dev_err(dcp->dev, "DCPAVSERV: open answered %#x, opened %llu\n",
			get_unaligned_le32(reply), get_unaligned_le64(reply + 16));
		return -EIO;
	}
	mutex_lock(&dcp->dcpavserv.lock);
	/* An instance gone meanwhile took its open with it. */
	if (dcp->dcpavserv.service == service)
		dcp->dcpavserv.opened = service;
	mutex_unlock(&dcp->dcpavserv.lock);
	dev_info(dcp->dev, "DCPAVSERV: AV client open\n");
	return 0;
}

void dcpavserv_v14_close(struct apple_dcp *dcp)
{
	struct apple_epic_service *service = NULL;
	u8 reply[16] = {};
	int ret;

	if (!dcp_is_v14_7_external(dcp))
		return;
	mutex_lock(&dcp->dcpavserv.lock);
	if (dcp->dcpavserv.opened && dcp->dcpavserv.opened == dcp->dcpavserv.service)
		service = afk_service_get(dcp->dcpavserv.service);
	dcp->dcpavserv.opened = NULL;
	dcp->dcpavserv.terminated = NULL;
	mutex_unlock(&dcp->dcpavserv.lock);
	if (!service)
		return;
	ret = afk_service_call(service, 0, 5, NULL, 0, 16, reply, sizeof(reply), 0);
	afk_service_put(service);
	/* Removed while the Close was on its way: nothing is left open. */
	if (ret == -ENODEV)
		return;
	if (!ret && get_unaligned_le32(reply))
		ret = -EIO;
	if (ret)
		dev_err(dcp->dev, "DCPAVSERV: AV client close failed: %d\n", ret);
	else
		dev_info(dcp->dev, "DCPAVSERV: AV client closed\n");
}

static void dcpavserv_close_work(struct work_struct *work)
{
	struct dcpavserv *av = container_of(work, struct dcpavserv, close_wq);

	dcpavserv_v14_close(container_of(av, struct apple_dcp, dcpavserv));
}

/* Terminate on the opened instance: no new use of it, and the client's Close. */
static void dcpavserv_v14_report(struct apple_epic_service *service, enum epic_subtype type,
				 const void *data, size_t size)
{
	struct apple_dcp *dcp = service->ep->dcp;
	const struct epic_service_call *call = data;
	bool terminate = false;

	if (type != EPIC_SUBTYPE_STD_SERVICE || size < sizeof(*call) ||
	    le32_to_cpu(call->magic) != EPIC_SERVICE_CALL_MAGIC ||
	    le16_to_cpu(call->group) != 2 || le32_to_cpu(call->command) != 1)
		return;
	mutex_lock(&dcp->dcpavserv.lock);
	if (dcp->dcpavserv.service == service && dcp->dcpavserv.opened == service &&
	    dcp->dcpavserv.terminated != service) {
		dcp->dcpavserv.terminated = service;
		terminate = true;
	}
	mutex_unlock(&dcp->dcpavserv.lock);
	if (!terminate)
		return;
	dev_info(dcp->dev, "DCPAVSERV: AV instance terminated: closing the client\n");
	schedule_work(&dcp->dcpavserv.close_wq);
}

static int dcpavserv_report(struct apple_epic_service *service,
			    enum epic_subtype type, const void *data,
			    size_t data_size)
{
	if (dcp_is_v14_7_external(service->ep->dcp))
		dcpavserv_v14_report(service, type, data, data_size);
	return parse_report(service, type, data, data_size);
}

static int dcpdpserv_report(struct apple_epic_service *service,
			    enum epic_subtype type, const void *data,
			    size_t data_size)
{
	return parse_report(service, type, data, data_size);
}

/* Integration's bounded read, for every firmware other than H17P. */
static const struct drm_edid *dcpavserv_read_edid(struct apple_epic_service *service)
{
	struct dpavserv_copy_edid_cmd cmd;
	struct dpavserv_copy_edid_resp *resp __free(kfree) = NULL;
	size_t resp_size = sizeof(*resp) + DPAVSERV_EDID_BUF_SIZE;
	size_t reply_size, edid_size;
	int ret;

	memset(&cmd, 0, sizeof(cmd));
	cmd.max_size = cpu_to_le64(DPAVSERV_EDID_BUF_SIZE);
	resp = kzalloc(resp_size, GFP_KERNEL);
	if (!resp)
		return ERR_PTR(-ENOMEM);

	ret = afk_service_call_with_reply_len(service, 1, 7, &cmd, sizeof(cmd),
					      DPAVSERV_EDID_BUF_SIZE, resp,
					      resp_size, 0, &reply_size);
	if (ret < 0)
		return ERR_PTR(ret);

	ret = dcpavserv_edid_size(resp, reply_size, &edid_size);
	if (ret)
		return ERR_PTR(ret);

	return drm_edid_alloc(resp->data + DPAVSERV_EDID_LEADING_SIZE, edid_size);
}

static const struct drm_edid *dcpavserv_read_edid_legacy(struct apple_epic_service *service)
{
	struct dpavserv_copy_edid_cmd cmd;
	struct dpavserv_copy_edid_resp *resp __free(kfree) = NULL;
	size_t resp_size = sizeof(*resp) + EDID_BUF_SIZE;
	int num_blocks;
	u64 data_size;
	int ret;

	memset(&cmd, 0, sizeof(cmd));
	cmd.max_size = cpu_to_le64(EDID_BUF_SIZE);
	resp = kzalloc(resp_size, GFP_KERNEL);
	if (!resp)
		return ERR_PTR(-ENOMEM);

	ret = afk_service_call(service, 1, 7, &cmd, sizeof(cmd), EDID_BUF_SIZE, resp,
			       resp_size, 0);
	if (ret < 0)
		return ERR_PTR(ret);

	if (le64_to_cpu(resp->max_size) != EDID_BUF_SIZE)
		return ERR_PTR(-EIO);

	// print_hex_dump(KERN_DEBUG, "dpavserv EDID cmd: ", DUMP_PREFIX_NONE,
	// 	       16, 1, resp, 192, true);

	data_size = le64_to_cpu(resp->used_size);
	if (data_size < EDID_LEADING_DATA_SIZE + EDID_BLOCK_SIZE ||
	    data_size > EDID_BUF_SIZE)
		return ERR_PTR(-EIO);

	/*
	 * An HDMI 2.x sink using the HF-EEODB announces fewer extension blocks
	 * in the base block than it sends, so the payload may be longer than
	 * the base block says. Accept any whole number of blocks at least that
	 * long and leave the block count to drm_edid, which knows about
	 * HF-EEODB.
	 */
	num_blocks = resp->data[EDID_LEADING_DATA_SIZE + EDID_EXT_BLOCK_COUNT_OFFSET];
	if ((1 + num_blocks) * EDID_BLOCK_SIZE > data_size - EDID_LEADING_DATA_SIZE ||
	    (data_size - EDID_LEADING_DATA_SIZE) % EDID_BLOCK_SIZE)
		return ERR_PTR(-EIO);

	return drm_edid_alloc(resp->data + EDID_LEADING_DATA_SIZE,
			      data_size - EDID_LEADING_DATA_SIZE);
}

static const struct drm_edid *dcpavserv_read_edid_inline(struct apple_epic_service *service)
{
	struct dpavserv_copy_edid_cmd cmd;
	struct dpavserv_copy_edid_resp *resp __free(kfree) = NULL;
	size_t resp_size = sizeof(*resp) + EDID_BUF_SIZE;
	size_t received, edid_size;
	int ret;

	memset(&cmd, 0, sizeof(cmd));
	cmd.max_size = cpu_to_le64(EDID_BUF_SIZE);
	resp = kzalloc(resp_size, GFP_KERNEL);
	if (!resp)
		return ERR_PTR(-ENOMEM);

	ret = afk_service_call_len(service, 1, 7, &cmd, sizeof(cmd), EDID_BUF_SIZE,
				   resp, resp_size, 0, &received);
	if (ret < 0)
		return ERR_PTR(ret);

	ret = dpavserv_edid_size(resp, received, &edid_size);
	if (ret)
		return ERR_PTR(ret);

	return drm_edid_alloc(resp->data + EDID_LEADING_DATA_SIZE, edid_size);
}

bool dcpavserv_edid_current(struct apple_dcp *dcp, u64 generation)
{
	lockdep_assert_held(&dcp->hpd_mutex);
	return dcp->hw.iomfb_method_profile !=
		DCP_IOMFB_METHODS_H17P_EXTERNAL ||
		(dcp->typec_cable_connected && generation == dcp->typec_generation &&
		 !READ_ONCE(dcp->quiescing) && !READ_ONCE(dcp->external_physically_absent));
}

bool dcpavserv_edid_service_current(struct apple_dcp *dcp, u64 epoch)
{
	lockdep_assert_held(&dcp->dcpavserv.lock);
	return dcp->dcpavserv.enabled && dcp->dcpavserv.service &&
		epoch == dcp->dcpavserv.epoch &&
		(dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17P_EXTERNAL ||
		 !READ_ONCE(dcp->external_physically_absent));
}

const struct drm_edid *dcpavserv_copy_edid(struct apple_dcp *dcp, u64 generation,
					u64 *epoch)
{
	struct apple_epic_service *service;
	const struct drm_edid *edid;

	if (dcp->fw_compat != DCP_FIRMWARE_H17P) {
		bool v14_7_ext = dcp_is_v14_7_external(dcp);
		bool open = false;

		mutex_lock(&dcp->dcpavserv.lock);
		service = afk_service_get(dcp->dcpavserv.service);
		*epoch = dcp->dcpavserv.epoch;
		if (service && v14_7_ext) {
			/* A terminated instance gets no new use; its successor is published. */
			if (dcp->dcpavserv.terminated == service) {
				afk_service_put(service);
				service = NULL;
			} else {
				open = dcp->dcpavserv.opened != service;
			}
		}
		mutex_unlock(&dcp->dcpavserv.lock);
		if (!service)
			return ERR_PTR(-ENODEV);

		/*
		 * The 14.7 firmware of an external processor answers EDID
		 * requests through an instance its client has opened (DCPAV
		 * Open): open each announced instance once, and close it
		 * again before the sink's release (dcpavserv_v14_close()).
		 * An open the firmware refuses is reported; the read that
		 * follows validates its own result.
		 *
		 * Only on Aurora's DCPLink session (T6030). The T8122 dcpext,
		 * which Linux starts and drives through the IOMFB template,
		 * reads the EDID directly, as in AsahiLinux/linux#607. Its
		 * firmware never answers the open call and drops the display
		 * link instead, about 50 ms after it, which brings the display
		 * back 4 s later: hotplug in a loop.
		 */
		if (open) {
			int ret = dcpavserv_v14_open(dcp, service);

			if (ret)
				dev_warn(dcp->dev, "DCPAVSERV: AV client not opened: %d\n", ret);
		}

		edid = dcpavserv_read_edid(service);
		/* The 14.7 external role: every block's header and checksum. */
		if (v14_7_ext && !IS_ERR(edid) && !drm_edid_valid(edid)) {
			dev_warn(dcp->dev, "DCPAVSERV: the sink's EDID is not valid\n");
			drm_edid_free(edid);
			edid = ERR_PTR(-EPROTO);
		}
		mutex_lock(&dcp->dcpavserv.lock);
		if (dcp->dcpavserv.service != service) {
			if (!IS_ERR(edid))
				drm_edid_free(edid);
			edid = ERR_PTR(-ENODEV);
		}
		mutex_unlock(&dcp->dcpavserv.lock);
		afk_service_put(service);
		return edid;
	}

	scoped_guard(mutex, &dcp->hpd_mutex) {
		if (!dcpavserv_edid_current(dcp, generation))
			return ERR_PTR(-ESTALE);
	}
	mutex_lock(&dcp->dcpavserv.lock);
	service = dcp->dcpavserv.enabled ? afk_service_get(dcp->dcpavserv.service) : NULL;
	*epoch = dcp->dcpavserv.epoch;
	mutex_unlock(&dcp->dcpavserv.lock);
	if (!service)
		return ERR_PTR(-ENODEV);

	edid = service->ep->wire_v2 ? dcpavserv_read_edid_inline(service) :
		dcpavserv_read_edid_legacy(service);
	mutex_lock(&dcp->dcpavserv.lock);
	if (dcp->dcpavserv.service != service ||
	    !dcpavserv_edid_service_current(dcp, *epoch)) {
		if (!IS_ERR(edid))
			drm_edid_free(edid);
		edid = ERR_PTR(-ENODEV);
	}
	mutex_unlock(&dcp->dcpavserv.lock);
	afk_service_put(service);
	scoped_guard(mutex, &dcp->hpd_mutex) {
		if (!dcpavserv_edid_current(dcp, generation)) {
			if (!IS_ERR(edid))
				drm_edid_free(edid);
			edid = ERR_PTR(-ESTALE);
		}
	}
	return edid;
}

static const struct apple_epic_service_ops dpavservep_ops[] = {
	{
		.name = "dcpav-service-epic",
		.reusable = true,
		.init = dcpavserv_init,
		.teardown = dcpavserv_teardown,
		.report = dcpavserv_report,
	},
	{
		.name = "dcpdp-service-epic",
		.reusable = true,
		.init = dcpdpserv_init,
		.teardown = dcpdpserv_teardown,
		.report = dcpdpserv_report,
	},
	{},
};

/* H17P (T8140, T8142): admission before OPEN, and the optional DP service. */
static const struct apple_epic_service_ops dpavservep_h17p_ops[] = {
	{
		.name = "dcpav-service-epic",
		.reusable = true,
		.prepare = dcpavserv_prepare,
		.revalidate = dcpavserv_revalidate,
		.init = dcpavserv_init_h17p,
		.teardown = dcpavserv_teardown_h17p,
		.report = dcpavserv_report,
	},
	{
		.name = "dcpdp-service-epic",
		.reusable = true,
		.optional = true,
		.prepare = dcpdpserv_prepare,
		.init = dcpdpserv_init,
		.teardown = dcpdpserv_teardown,
		.report = dcpdpserv_report,
	},
	{},
};

int dpavservep_init(struct apple_dcp *dcp)
{
	struct apple_dcp_afkep *ep;
	int ret;

	init_completion(&dcp->dcpavserv.enable_completion);
	INIT_WORK(&dcp->dcpavserv.close_wq, dcpavserv_close_work);

	ep = afk_init(dcp, dcp->hw.iomfb_method_profile ==
		      DCP_IOMFB_METHODS_H17P_EXTERNAL ?
		      DPAVSERV_ENDPOINT_V2 : DPAVSERV_ENDPOINT,
		      dcp->fw_compat == DCP_FIRMWARE_H17P ?
		      dpavservep_h17p_ops : dpavservep_ops);
	if (IS_ERR(ep))
		return PTR_ERR(ep);
	/* External H17P uses AFKV2 service calls with host-owned OOB buffers. */
	ep->wire_v2 = dcp->hw.iomfb_method_profile ==
		DCP_IOMFB_METHODS_H17P_EXTERNAL;
	ep->inline_v2 = false;
	dcp->dcpavservep = ep;

	dcp->dcpavservep->match_epic_name = true;

	ret = afk_start(dcp->dcpavservep);
	if (ret) {
		afk_shutdown(ep);
		dpavservep_detach(dcp);
		dcp->dcpavservep = NULL;
		return ret;
	}

	/* Admission may arrive after startup; exact guards and late notify own it. */
	if (ep->wire_v2)
		return 0;

	ret = wait_for_completion_timeout(&dcp->dcpavserv.enable_completion,
					  msecs_to_jiffies(1000));
	/* Integration: a missing announcement is not an error elsewhere. */
	if (dcp->fw_compat != DCP_FIRMWARE_H17P)
		return 0;
	if (!ret) {
		dev_warn(dcp->dev, "DCPAVSERV: timed out waiting for EDID service announcement\n");
		return -ETIMEDOUT;
	}

	return 0;
}
