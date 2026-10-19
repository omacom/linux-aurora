// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2023 Martin Povišer <povik+lin@cutebit.org> */

// #define DEBUG

#include <linux/debugfs.h>
#include <linux/kconfig.h>
#include <linux/unaligned.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "audio.h"
#include "afk.h"
#include "av.h"
#include "dcp.h"
#include "dcp-internal.h"

struct dcp_av_audio_cmds {
	/* commands in group 0*/
	u32 open;
	u32 close;
	u32 prepare;
	u32 start_link;
	u32 stop_link;
	u32 unprepare;
	/* commands in group 1*/
	u32 get_elements;
	u32 get_product_attrs;
};

static const struct dcp_av_audio_cmds dcp_av_audio_cmds_v12_3 = {
	.open = 6,
	.close = 7,
	.prepare = 8,
	.start_link = 9,
	.stop_link = 12,
	.unprepare = 13,
	.get_elements = 18,
	.get_product_attrs = 20,
};

static const struct dcp_av_audio_cmds dcp_av_audio_cmds_v13_5 = {
	.open = 4,
	.close = 5,
	.prepare = 6,
	.start_link = 7,
	.stop_link = 10,
	.unprepare = 11,
	.get_elements = 16,
	.get_product_attrs = 18,
};

struct audiosrv_data {
	struct platform_device *audio_dev;
	bool plugged;
	struct mutex plug_lock;

	struct apple_epic_service *srv;
	struct rw_semaphore srv_rwsem;
	/* Workqueue for starting the audio service */
	struct work_struct start_av_service_wq;
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	/* The 14.7 external role: let a terminated audio interface go. */
	struct work_struct terminate_wq;
	/* The 14.7 external role: the instance the PCM opened (open_lock). */
	struct mutex open_lock;
	struct apple_epic_service *open_srv;
#endif

	struct dcp_av_audio_cmds cmds;

	bool warned_get_elements;
	bool warned_get_product_attrs;
	bool is_open;
};

static void av_interface_init(struct apple_epic_service *service, const char *name,
			      const char *class, s64 unit)
{
}

static void av_interface_teardown(struct apple_epic_service *service)
{
	struct apple_dcp *dcp = service->ep->dcp;
	struct audiosrv_data *asrv = dcp->audiosrv;

	afk_service_disable(service);

	mutex_lock(&asrv->plug_lock);

	asrv->plugged = false;
	if (asrv->audio_dev)
		dcpaud_disconnect(asrv->audio_dev);

	mutex_unlock(&asrv->plug_lock);
}

static void av_audiosrv_init(struct apple_epic_service *service, const char *name,
			     const char *class, s64 unit)
{
	struct apple_dcp *dcp = service->ep->dcp;
	struct audiosrv_data *asrv = dcp->audiosrv;
	struct apple_epic_service *old;

	/* The owner pin keeps the slot from being recycled while published. */
	service = afk_service_get(service);
	if (!service)
		return;

	mutex_lock(&asrv->plug_lock);

	down_write(&asrv->srv_rwsem);
	old = asrv->srv;
	asrv->srv = service;
	up_write(&asrv->srv_rwsem);
	if (old)
		afk_service_put(old);

	asrv->plugged = true;
	mutex_unlock(&asrv->plug_lock);
	schedule_work(&asrv->start_av_service_wq);
}

static void av_audiosrv_teardown(struct apple_epic_service *service)
{
	struct apple_dcp *dcp = service->ep->dcp;
	struct audiosrv_data *asrv = dcp->audiosrv;
	bool owned;

	mutex_lock(&asrv->plug_lock);

	down_write(&asrv->srv_rwsem);
	owned = asrv->srv == service;
	if (owned)
		asrv->srv = NULL;
	up_write(&asrv->srv_rwsem);

	/* A stale generation must not unplug its successor. */
	if (owned) {
		asrv->plugged = false;
		if (asrv->audio_dev)
			dcpaud_disconnect(asrv->audio_dev);
	}

	mutex_unlock(&asrv->plug_lock);

	if (owned)
		afk_service_put(service);
}

int dcp_audiosrv_prepare(struct device *dev, struct dcp_sound_cookie *cookie)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = afk_service_call(asrv->srv, 0, asrv->cmds.prepare, cookie,
			       sizeof(*cookie), 64 - sizeof(*cookie), NULL, 0,
			       64);
	up_write(&asrv->srv_rwsem);

	return ret;
}

int dcp_audiosrv_startlink(struct device *dev, struct dcp_sound_cookie *cookie)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = afk_service_call(asrv->srv, 0, asrv->cmds.start_link, cookie,
			       sizeof(*cookie), 64 - sizeof(*cookie), NULL, 0,
			       64);
	up_write(&asrv->srv_rwsem);

	return ret;
}

int dcp_audiosrv_stoplink(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = afk_service_call(asrv->srv, 0, asrv->cmds.stop_link, NULL, 0, 64,
			       NULL, 0, 64);
	up_write(&asrv->srv_rwsem);

	return ret;
}

int dcp_audiosrv_unprepare(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = afk_service_call(asrv->srv, 0, asrv->cmds.unprepare, NULL, 0, 64,
			       NULL, 0, 64);
	up_write(&asrv->srv_rwsem);

	return ret;
}

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
/* The audio service of the 14.7 external role, known before it starts. */
bool dcp_audiosrv_native_link(struct device *dev)
{
	return dcp_is_v14_7_external(dev_get_drvdata(dev));
}

/*
 * The 14.7 external role: the service is open while a PCM is (see
 * av_service_connect()). The instance the PCM opened is kept under open_lock,
 * which the endpoint's receive path never takes, and the calls are made
 * without srv_rwsem: a teardown arriving meanwhile goes through, and so do
 * the replies. A removed instance stays removed (its slot is not reusable):
 * calls on it fail unsent with -ENODEV, so it is never closed, and no Close
 * goes to its successor.
 */
int dcp_audiosrv_open(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	struct apple_epic_service *srv;
	u8 reply[32] = {};
	int ret;

	if (!asrv)
		return -ENODEV;

	guard(mutex)(&asrv->open_lock);
	scoped_guard(rwsem_read, &asrv->srv_rwsem)
		srv = asrv->srv;
	if (!srv)
		return -ENODEV;
	if (asrv->open_srv == srv)
		return 0;

	/* Open: a zero body; status at +0, the opened Boolean at +16. */
	ret = afk_service_call(srv, 0, asrv->cmds.open, NULL, 0, 32, reply, sizeof(reply), 0);
	if (ret)
		return ret;
	if (get_unaligned_le32(reply) || !get_unaligned_le64(reply + 16)) {
		dev_err(dev, "audiosrv: open answered %#x, opened %llu\n",
			get_unaligned_le32(reply), get_unaligned_le64(reply + 16));
		return -EIO;
	}
	asrv->open_srv = srv;
	return 0;
}

static void av_audiosrv_close(struct apple_dcp *dcp, struct audiosrv_data *asrv)
{
	struct apple_epic_service *srv;
	u8 reply[16] = {};
	int ret;

	guard(mutex)(&asrv->open_lock);
	srv = asrv->open_srv;
	asrv->open_srv = NULL;
	if (!srv)
		return;

	/* Close: a zero body; status at +0. */
	ret = afk_service_call(srv, 0, asrv->cmds.close, NULL, 0, 16, reply, sizeof(reply), 0);
	if (ret == -ENODEV)
		return;
	if (ret || get_unaligned_le32(reply))
		dev_err(dcp->dev, "audiosrv: close failed: %d, answered %#x\n", ret,
			get_unaligned_le32(reply));
}

void dcp_audiosrv_close(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	if (dcp->audiosrv)
		av_audiosrv_close(dcp, dcp->audiosrv);
}

/*
 * The link methods of the 14.7 external role's audio service, the native
 * audio driver's: PrepareLink, StartLink, StopLink and CompleteLink all take
 * the selected element's 32-byte link data at body+0 and link source 0 at
 * +32, and answer an IOReturn at body+48.
 */
int dcp_audiosrv_link(struct device *dev, enum dcp_audio_link op,
		      const struct dcp_sound_cookie *cookie)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	u8 body[64] = {}, reply[64];
	u32 command, status;
	int ret;

	switch (op) {
	case DCP_AUDIO_LINK_PREPARE:
		command = asrv->cmds.prepare;
		break;
	case DCP_AUDIO_LINK_START:
		command = asrv->cmds.start_link;
		break;
	case DCP_AUDIO_LINK_STOP:
		command = asrv->cmds.stop_link;
		break;
	case DCP_AUDIO_LINK_COMPLETE:
		command = asrv->cmds.unprepare;
		break;
	default:
		return -EINVAL;
	}
	static_assert(sizeof(*cookie) == DCP_SOUND_COOKIE_SIZE_V14_7);
	memcpy(body, cookie, sizeof(*cookie));

	scoped_guard(mutex, &asrv->open_lock) {
		if (!asrv->open_srv)
			return -ENODEV;
		ret = afk_service_call(asrv->open_srv, 0, command, body, sizeof(body), 0,
				       reply, sizeof(reply), 0);
	}
	if (ret)
		return ret;
	status = get_unaligned_le32(reply + 48);
	if (status) {
		dev_err(dev, "audiosrv: link method %u answered %#x\n", command, status);
		return -EIO;
	}
	return 0;
}
#endif

static int
dcp_audiosrv_osobject_call(struct apple_epic_service *service, u16 group,
			   u32 command, void *output, size_t output_maxsize,
			   size_t *output_size)
{
	struct {
		__le64 max_size;
		u8 _pad1[24];
		__le64 used_size;
		u8 _pad2[8];
	} __attribute__((packed)) *hdr;
	static_assert(sizeof(*hdr) == 48);
	size_t bfr_len = output_maxsize + sizeof(*hdr);
	void *bfr __free(kfree) = NULL;
	int ret;

	bfr = kzalloc(bfr_len, GFP_KERNEL);
	if (!bfr)
		return -ENOMEM;

	hdr = bfr;
	hdr->max_size = cpu_to_le64(output_maxsize);
	ret = afk_service_call(service, group, command, hdr, sizeof(*hdr), output_maxsize,
			       bfr, sizeof(*hdr) + output_maxsize, 0);
	if (ret)
		return ret;

	if (output)
		memcpy(output, bfr + sizeof(*hdr), output_maxsize);

	if (output_size)
		*output_size = le64_to_cpu(hdr->used_size);

	return 0;
}

int dcp_audiosrv_get_elements(struct device *dev, void *elements, size_t maxsize)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	size_t size;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = dcp_audiosrv_osobject_call(asrv->srv, 1, asrv->cmds.get_elements,
					 elements, maxsize, &size);
	up_write(&asrv->srv_rwsem);

	if (ret && !asrv->warned_get_elements) {
		dev_err(dev, "audiosrv: error getting elements: %d\n", ret);
		asrv->warned_get_elements = true;
	} else if (!ret) {
		dev_dbg(dev, "audiosrv: got %zd bytes worth of elements\n", size);
	}

	return ret;
}

int dcp_audiosrv_get_product_attrs(struct device *dev, void *attrs, size_t maxsize)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	size_t size;
	int ret;

	down_write(&asrv->srv_rwsem);
	ret = dcp_audiosrv_osobject_call(asrv->srv, 1,
					 asrv->cmds.get_product_attrs, attrs,
					 maxsize, &size);
	up_write(&asrv->srv_rwsem);

	if (ret && !asrv->warned_get_product_attrs) {
		dev_err(dev, "audiosrv: error getting product attributes: %d\n", ret);
		asrv->warned_get_product_attrs = true;
	} else if (!ret) {
		dev_dbg(dev, "audiosrv: got %zd bytes worth of product attributes\n", size);
	}

	return ret;
}

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
/* A DCPAV report (group 2) with command 1: Terminate. */
static bool av_report_terminate(const void *data, size_t size)
{
	const struct epic_service_call *call = data;

	return size >= sizeof(*call) && le32_to_cpu(call->magic) == EPIC_SERVICE_CALL_MAGIC &&
	       le16_to_cpu(call->group) == 2 && le32_to_cpu(call->command) == 1;
}
#endif

static int av_audiosrv_report(struct apple_epic_service *service, u32 idx,
						  const void *data, size_t data_size)
{
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	struct apple_dcp *dcp = service->ep->dcp;

	/*
	 * The 14.7 external role terminates an open audio interface when the
	 * sink changes (after it reports the sink's physical address). Its
	 * client must close it (Close, command 5): the firmware waits up to
	 * 10 s for that before it removes the interface, and the sink's link
	 * with it. A later publication brings the new instance.
	 */
	if (dcp_is_v14_7_external(dcp) && av_report_terminate(data, data_size)) {
		dev_info(dcp->dev, "audio interface terminated\n");
		schedule_work(&dcp->audiosrv->terminate_wq);
	}
#endif
	dev_dbg(service->ep->dcp->dev, "got audio report %d size %zx\n", idx, data_size);
#ifdef DEBUG
	print_hex_dump(KERN_DEBUG, "audio report: ", DUMP_PREFIX_NONE, 16, 1, data, data_size, true);
#endif

	return 0;
}

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
/*
 * The 14.7 external role behind a DP-to-HDMI converter (the J514S HDMI
 * port) also publishes a CEC interface on this endpoint. The host makes no
 * use of CEC; the interface's event reports (group 2: the sink's physical
 * address, link training, link start and stop) are taken and dropped.
 */
static void av_cec_init(struct apple_epic_service *service, const char *name,
			const char *class, s64 unit)
{
}

static int av_cec_report(struct apple_epic_service *service, enum epic_subtype type,
			 const void *data, size_t data_size)
{
	return 0;
}

static void av_cec_teardown(struct apple_epic_service *service)
{
	afk_service_disable(service);
}
#endif

static const struct apple_epic_service_ops avep_ops[] = {
	{
		.name = "DCPAVSimpleVideoInterface",
		.reusable = true,
		.init = av_interface_init,
		.teardown = av_interface_teardown,
	},
	{
		.name = "DCPAVAudioInterface",
		.reusable = true,
		.init = av_audiosrv_init,
		.report = av_audiosrv_report,
		.teardown = av_audiosrv_teardown,
	},
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	{
		.name = "DCPDPCECInterface",
		.init = av_cec_init,
		.report = av_cec_report,
		.teardown = av_cec_teardown,
	},
#endif
	{}
};

void av_service_connect(struct apple_dcp *dcp)
{
	struct apple_epic_service *service;
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	scoped_guard(rwsem_write, &asrv->srv_rwsem) {
		service = afk_service_pin(asrv->srv);
		if (!service)
			return;
	}

	/* open AV audio service */
	dev_info(dcp->dev, "%s: starting audio service, plugged:%d\n", __func__,  asrv->plugged);
	if (asrv->is_open)
		goto out;

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	/*
	 * The 14.7 external role opens its audio service only while a PCM is
	 * (dcp_audiosrv_open()): an open audio interface is a client the
	 * firmware must wait for whenever the sink changes (see
	 * av_audiosrv_report()); an idle one need not be. The card follows
	 * the display's power (A472 on): the CRTC enable connects before the
	 * mode is set, and the modeset connects again once the display is on.
	 */
	if (dcp_is_v14_7_external(dcp)) {
		if (!iomfb_v14_7_external_powered(dcp))
			goto out;
		scoped_guard(mutex, &asrv->plug_lock) {
			if (asrv->audio_dev)
				dcpaud_connect(asrv->audio_dev, asrv->plugged);
		}
		goto out;
	}
#endif

	ret = afk_service_call(service, 0, asrv->cmds.open, NULL, 0, 32,
			       NULL, 0, 32);
	if (ret) {
		dev_err(dcp->dev, "error opening audio service: %d\n", ret);
		goto out;
	}
	mutex_lock(&asrv->plug_lock);
	asrv->is_open = true;

	if (asrv->audio_dev)
		dcpaud_connect(asrv->audio_dev, asrv->plugged);
	mutex_unlock(&asrv->plug_lock);
out:
	afk_service_put(service);
}

void av_service_disconnect(struct apple_dcp *dcp)
{
	struct apple_epic_service *service;
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	scoped_guard(rwsem_write, &asrv->srv_rwsem) {
		service = afk_service_pin(asrv->srv);
		if (!service)
			return;
	}

	/* close AV audio service */
	dev_info(dcp->dev, "%s: stopping audio service\n", __func__);
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	/* The 14.7 external role's card goes with the display, and an open PCM's service. */
	if (dcp_is_v14_7_external(dcp)) {
		struct platform_device *audio_dev;

		scoped_guard(mutex, &asrv->plug_lock) {
			audio_dev = asrv->audio_dev;
			if (audio_dev)
				dcpaud_disconnect(audio_dev);
		}
		/*
		 * The open PCM's link ends on the service, before its Close;
		 * outside plug_lock, which the endpoint's receive path takes.
		 */
		if (audio_dev)
			dcpaud_release_link(audio_dev);
		av_audiosrv_close(dcp, asrv);
		goto out;
	}
#endif
	if (!asrv->is_open)
		goto out;

	mutex_lock(&asrv->plug_lock);

	if (asrv->audio_dev)
		dcpaud_disconnect(asrv->audio_dev);

	mutex_unlock(&asrv->plug_lock);

	ret = afk_service_call(service, 0, asrv->cmds.close, NULL, 0, 16,
			       NULL, 0, 16);
	if (ret) {
		dev_err(dcp->dev, "error closing audio service: %d\n", ret);
	}
	if (service->torndown)
		afk_service_disable(service);
	asrv->is_open = false;
out:
	afk_service_put(service);
}

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
static void av_work_service_terminate(struct work_struct *work)
{
	struct audiosrv_data *asrv = container_of(work, struct audiosrv_data, terminate_wq);
	struct apple_dcp *dcp = NULL;

	scoped_guard(rwsem_read, &asrv->srv_rwsem) {
		if (asrv->srv && asrv->srv->ep)
			dcp = asrv->srv->ep->dcp;
	}
	if (dcp)
		av_service_disconnect(dcp);
}
#endif

static void av_work_service_start(struct work_struct *work)
{
	struct audiosrv_data *audiosrv_data;
	struct apple_dcp *dcp;

	audiosrv_data = container_of(work, struct audiosrv_data, start_av_service_wq);

	scoped_guard(rwsem_read, &audiosrv_data->srv_rwsem) {
		if (!audiosrv_data->srv ||
		!audiosrv_data->srv->ep ||
		!audiosrv_data->srv->ep->dcp) {
			pr_err("%s: dcp: av: NULL ptr during startup\n", __func__);
			return;
		}
		dcp = audiosrv_data->srv->ep->dcp;
	}

	av_service_connect(dcp);
}

int avep_init(struct apple_dcp *dcp)
{
	struct audiosrv_data *audiosrv_data;
	struct platform_device *audio_pdev;
	struct device *dev = dcp->dev;
	struct device_node *endpoint, *audio_node = NULL;

	audiosrv_data = devm_kzalloc(dcp->dev, sizeof(*audiosrv_data), GFP_KERNEL);
	if (!audiosrv_data)
		return -ENOMEM;
	init_rwsem(&audiosrv_data->srv_rwsem);
	mutex_init(&audiosrv_data->plug_lock);

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		audiosrv_data->cmds = dcp_av_audio_cmds_v12_3;
		break;
	case DCP_FIRMWARE_V_13_5:
		audiosrv_data->cmds = dcp_av_audio_cmds_v13_5;
		break;
	default:
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
		/*
		 * The 14.7 external role (T6030 external processor) numbers its
		 * methods as 13.5 does: open 4, close 5, prepare 6, start 7,
		 * stop 10, complete 11, elements 1/16, product attributes 1/18.
		 */
		if (dcp_is_v14_7_external(dcp)) {
			audiosrv_data->cmds = dcp_av_audio_cmds_v13_5;
			break;
		}
#endif
		dev_err(dcp->dev, "Audio not supported for firmware\n");
		return -ENODEV;
	}

	dcp->audiosrv = audiosrv_data;
	INIT_WORK(&audiosrv_data->start_av_service_wq, av_work_service_start);
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	INIT_WORK(&audiosrv_data->terminate_wq, av_work_service_terminate);
	mutex_init(&audiosrv_data->open_lock);
#endif

	endpoint = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	if (endpoint) {
		audio_node = of_graph_get_remote_port_parent(endpoint);
		of_node_put(endpoint);
	}
	if (!audio_node || !of_device_is_available(audio_node)) {
		of_node_put(audio_node);
		dev_info(dev, "No audio support\n");
		return 0;
	}

	audio_pdev = of_find_device_by_node(audio_node);
	of_node_put(audio_node);
	if (!audio_pdev) {
		dev_info(dev, "No DP/HDMI audio device not ready\n");
		return 0;
	}
	dcp->audiosrv->audio_dev = audio_pdev;

	device_link_add(&audio_pdev->dev, dev,
			DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME);

	dcp->avep = afk_init(dcp, AV_ENDPOINT, avep_ops);
	if (IS_ERR(dcp->avep))
		return PTR_ERR(dcp->avep);
	dcp->avep->debugfs_entry = dcp->ep_debugfs[AV_ENDPOINT - 0x20];
	return afk_start(dcp->avep);
}
