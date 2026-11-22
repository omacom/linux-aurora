// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2023 Martin Povišer <povik+lin@cutebit.org> */

// #define DEBUG

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/kconfig.h>
#include <linux/unaligned.h>
#include <kunit/static_stub.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "audio.h"
#include "afk.h"
#include "av.h"
#include "cec.h"
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
	/* after start_link and before stop_link; 0 where the firmware has none */
	u32 did_start_link;
	u32 will_stop_link;
	/* commands in group 1*/
	u32 get_elements;
	u32 get_product_attrs;
	/* The link state query around a stream; 0 where the firmware has none */
	u32 link_query;
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
	.did_start_link = 8,
	.will_stop_link = 9,
	.get_elements = 16,
	.get_product_attrs = 18,
};

/*
 * Firmware 27's external role (the T6050 DCPEXTs), as macOS 27 drives it on
 * the M5 Pro: the 13.5 numbers without the started and stopping
 * notifications, and method 13 as a stream starts and stops, with a zero
 * body; it answers 1 at +48 while the link runs.
 */
static const struct dcp_av_audio_cmds dcp_av_audio_cmds_v27 = {
	.open = 4,
	.close = 5,
	.prepare = 6,
	.start_link = 7,
	.stop_link = 10,
	.unprepare = 11,
	.get_elements = 16,
	.get_product_attrs = 18,
	.link_query = 13,
};

struct audiosrv_data;

struct av_audiosrv_cleanup {
	struct apple_dcp *dcp;
	struct audiosrv_data *owner;
	struct apple_dcp_afkep *ep;
};

struct audiosrv_data {
	struct av_audiosrv_cleanup cleanup;
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
	/*
	 * The instance generation, changed when an instance is published or
	 * removed (srv_rwsem), so a slot reused for the next instance is not
	 * taken for the old one; the Terminate work's instance.
	 */
	u64 gen;
	u64 terminate_gen;
	/*
	 * The 14.7 external role: the instance the PCM opened and its
	 * generation (open_lock); open_srv holds a pin, so its slot is not
	 * reused while it is open.
	 */
	struct mutex open_lock;
	struct apple_epic_service *open_srv;
	u64 open_gen;
	/* Woken when gen changes: an Open waits there for a replacement. */
	wait_queue_head_t gen_wq;
	/* The instance the link was last prepared on (open_lock) */
	u64 link_gen;
	/* Firmware 27: the video interface (srv_rwsem), the instance its start calls went to */
	struct apple_epic_service *video_srv;
	u64 v27_boot_gen;
#endif

	struct dcp_av_audio_cmds cmds;

	bool warned_get_elements;
	bool warned_get_product_attrs;
	bool is_open;
	/* INIT_WORK has run for the embedded owner works. */
	bool works_live;
	/* Report and service-start admission. Writers hold srv_rwsem. */
	bool reports_stopped;
	/* One disable_work_sync is outstanding. Restart enables once. */
	bool work_disabled;
	/* afk_init/afk_start failed. Rebind must not enable these works. */
	bool restart_blocked;
	/* The element capacity a compact endpoint's last catalog fitted */
	size_t elements_cap;
	/* The published instance answered the whole capacity with an empty catalog */
	bool elements_empty;
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
	bool queue_start;

	/* The owner pin keeps the slot from being recycled while published. */
	service = afk_service_get(service);
	if (!service)
		return;

	mutex_lock(&asrv->plug_lock);

	down_write(&asrv->srv_rwsem);
	old = asrv->srv;
	asrv->srv = service;
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	asrv->gen++;
#endif
	queue_start = !asrv->reports_stopped;
	up_write(&asrv->srv_rwsem);
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	wake_up_all(&asrv->gen_wq);
#endif
	if (old)
		afk_service_put(old);

	asrv->plugged = true;
	/* A new instance, maybe a new sink: its catalog is asked for afresh. */
	WRITE_ONCE(asrv->elements_empty, false);
	mutex_unlock(&asrv->plug_lock);
	if (queue_start)
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
	if (owned) {
		asrv->srv = NULL;
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
		asrv->gen++;
#endif
	}
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
/*
 * The external processors whose audio service is the desktop 26.x one:
 * dcpav-audio-interface-epic on endpoint 0x2b, compact EPIC. The T6041
 * DCPEXTs, and the J700 (T8140, H17P firmware) DCPEXT, whose service macOS
 * 26.6 drives with the same methods and link data; and firmware 27's
 * (T6050) DCPEXTs, whose link data differs (see dcp_audiosrv_link()).
 */
static bool av_desktop_audio_role(struct apple_dcp *dcp)
{
	return (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP && dcp->desktop_external) ||
	       dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL ||
	       dcp_is_v27_external(dcp);
}

bool dcp_audiosrv_link_by_format(struct device *dev)
{
	return dcp_is_v27_external(dev_get_drvdata(dev));
}

/* Native audio links are selected from the firmware role, before service start. */
bool dcp_audiosrv_native_link(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);

	return dcp_is_v14_7_external(dcp) || av_desktop_audio_role(dcp);
}

/* The 14.7 external role's audio service calls; KUnit stubs them. */
static int av_audiosrv_call(struct apple_epic_service *srv, u32 command, const void *body,
			    size_t body_len, size_t body_pad, void *reply, size_t reply_len)
{
	KUNIT_STATIC_STUB_REDIRECT(av_audiosrv_call, srv, command, body, body_len, body_pad,
				   reply, reply_len);
	if (READ_ONCE(srv->ep->compact)) {
		size_t received;
		int ret;

		ret = afk_service_call_with_reply_len(srv, 0, command, body, body_len,
						    body_pad, reply, reply_len, 0, &received);
		return ret ?: (received == reply_len ? 0 : -EMSGSIZE);
	}
	return afk_service_call(srv, 0, command, body, body_len, body_pad, reply, reply_len, 0);
}

/* How long an Open refused by an instance being replaced waits for the next one */
#define AV_AUDIOSRV_REPLACE_MS	1000

/*
 * The 14.7 external role: the service is open while a PCM is (see
 * av_service_connect()). The instance the PCM opened is kept under open_lock,
 * which the endpoint's receive path never takes, and the calls are made
 * without srv_rwsem: a teardown arriving meanwhile goes through, and so do
 * the replies. The audio slots are reusable, so the open instance is kept
 * pinned, with its generation: its slot cannot be reused while it is open,
 * a successor is opened even in the same slot, and a removed instance is
 * never closed (it took its open with it), so no Close goes to a successor.
 */
int dcp_audiosrv_open(struct device *dev)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	struct apple_epic_service *srv;
	u8 reply[32];
	bool replaced;
	u64 gen;
	int ret;

	if (!asrv)
		return -ENODEV;

	guard(mutex)(&asrv->open_lock);
	/*
	 * A display that wakes up (DPMS on) reports its sink on the audio
	 * interface it had, and the firmware replaces that instance a few
	 * hundred milliseconds later: an Open sent in between is refused
	 * (kIOReturnOffline). The Open that follows the replacement is
	 * answered, so wait for that instance and try it once more.
	 * Userspace drops an output whose first open fails, so this is the
	 * difference between a display with and without audio after it
	 * wakes up.
	 */
	for (int attempt = 0; ; attempt++) {
		scoped_guard(rwsem_read, &asrv->srv_rwsem) {
			srv = afk_service_get(asrv->srv);
			gen = asrv->gen;
		}
		if (!srv)
			return -ENODEV;
		if (asrv->open_srv) {
			if (asrv->open_gen == gen) {
				afk_service_put(srv);
				return 0;
			}
			/* An earlier instance's open went with that instance. */
			afk_service_put(asrv->open_srv);
			asrv->open_srv = NULL;
		}

		/* Open: a zero body; status at +0, the opened Boolean at +16. */
		memset(reply, 0, sizeof(reply));
		ret = av_audiosrv_call(srv, asrv->cmds.open, NULL, 0, 32, reply, sizeof(reply));
		if (!ret && (get_unaligned_le32(reply) || !get_unaligned_le64(reply + 16))) {
			dev_err(dev, "audiosrv: open answered %#x, opened %llu\n",
				get_unaligned_le32(reply), get_unaligned_le64(reply + 16));
			ret = -EIO;
		}
		if (!ret)
			break;
		afk_service_put(srv);
		if (attempt)
			return ret;
		/*
		 * The instance refused it as offline: its successor is on the
		 * way (published within some 200 ms on the J616s), wait for it.
		 */
		if (ret == -ENODEV)
			wait_event_timeout(asrv->gen_wq,
					   READ_ONCE(asrv->srv) && READ_ONCE(asrv->gen) != gen,
					   msecs_to_jiffies(AV_AUDIOSRV_REPLACE_MS));
		scoped_guard(rwsem_read, &asrv->srv_rwsem)
			replaced = asrv->srv && asrv->gen != gen;
		if (!replaced)
			return ret;
		dev_dbg(dev, "audiosrv: open on a replaced instance failed (%d), retrying\n", ret);
	}
	asrv->open_srv = srv;
	asrv->open_gen = gen;
	return 0;
}

/* @gen: only the client of that instance (NULL: whichever is open) */
static void av_audiosrv_close(struct apple_dcp *dcp, struct audiosrv_data *asrv,
			      const u64 *gen)
{
	struct apple_epic_service *srv;
	u8 reply[16] = {};
	bool published;
	int ret;

	guard(mutex)(&asrv->open_lock);
	/* Checked here, at the Close: a later instance may be open by now. */
	if (gen && asrv->open_gen != *gen)
		return;
	srv = asrv->open_srv;
	asrv->open_srv = NULL;
	if (!srv)
		return;
	scoped_guard(rwsem_read, &asrv->srv_rwsem)
		published = asrv->srv == srv && asrv->gen == asrv->open_gen;

	/* Close: a zero body; status at +0. Only the instance still published. */
	ret = published ? av_audiosrv_call(srv, asrv->cmds.close, NULL, 0, 16, reply,
					   sizeof(reply)) : -ENODEV;
	afk_service_put(srv);
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
		av_audiosrv_close(dcp, dcp->audiosrv, NULL);
}

bool dcp_audiosrv_link_on(struct device *dev, u64 gen)
{
	struct apple_dcp *dcp;
	struct audiosrv_data *asrv;

	KUNIT_STATIC_STUB_REDIRECT(dcp_audiosrv_link_on, dev, gen);
	dcp = dev_get_drvdata(dev);
	asrv = dcp->audiosrv;
	if (!asrv)
		return false;
	guard(mutex)(&asrv->open_lock);
	return asrv->open_srv && asrv->open_gen == gen && asrv->link_gen == gen;
}

/* open_lock pins the client; the endpoint can replace its published instance. */
static bool av_audiosrv_open_published(struct audiosrv_data *asrv)
{
	lockdep_assert_held(&asrv->open_lock);
	guard(rwsem_read)(&asrv->srv_rwsem);
	return asrv->open_srv && asrv->srv == asrv->open_srv &&
	       asrv->gen == asrv->open_gen;
}

bool dcp_audiosrv_link_live(struct device *dev)
{
	struct apple_dcp *dcp;
	struct audiosrv_data *asrv;

	KUNIT_STATIC_STUB_REDIRECT(dcp_audiosrv_link_live, dev);
	dcp = dev_get_drvdata(dev);
	asrv = dcp->audiosrv;
	if (!asrv)
		return false;
	guard(mutex)(&asrv->open_lock);
	return asrv->link_gen == asrv->open_gen && av_audiosrv_open_published(asrv);
}

/*
 * The link commands of the 14.7 external role's audio service, the native
 * audio driver's: prepare, start, stop and complete all take the selected
 * element's 32-byte link data at body+0 and link source 0 at +32, and
 * answer a status at body+48.
 *
 * Firmware 27's take the stream's format (struct dcp_audio_link_format) at
 * body+0 instead, with the rest zero, and answer 1 at body+0 when they took
 * the link, as macOS 27 has them. Its link query takes 1 at body+0x20 and
 * answers the link's state at body+0, 1 while it runs.
 */
#define AV_V27_QUERY_ARG	0x20

int dcp_audiosrv_link(struct device *dev, enum dcp_audio_link op,
		      const struct dcp_sound_cookie *cookie)
{
	struct apple_dcp *dcp;
	struct audiosrv_data *asrv;
	u8 body[64] = {}, reply[64];
	u32 command, status;
	bool by_format;
	int ret;

	KUNIT_STATIC_STUB_REDIRECT(dcp_audiosrv_link, dev, op, cookie);
	dcp = dev_get_drvdata(dev);
	asrv = dcp->audiosrv;
	if (!asrv)
		return -ENODEV;
	if (!cookie)
		return -EINVAL;

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
	case DCP_AUDIO_LINK_DID_START:
		command = asrv->cmds.did_start_link;
		break;
	case DCP_AUDIO_LINK_WILL_STOP:
		command = asrv->cmds.will_stop_link;
		break;
	case DCP_AUDIO_LINK_QUERY:
		command = asrv->cmds.link_query;
		break;
	default:
		return -EINVAL;
	}
	if (!command)
		return 0;
	static_assert(sizeof(*cookie) == DCP_SOUND_COOKIE_SIZE_V14_7);
	static_assert(sizeof(struct dcp_audio_link_format) <= sizeof(*cookie));
	by_format = dcp_is_v27_external(dcp);
	if (!by_format)
		memcpy(body, cookie, sizeof(*cookie));
	else if (op != DCP_AUDIO_LINK_QUERY)
		memcpy(body, cookie, sizeof(struct dcp_audio_link_format));
	else
		put_unaligned_le32(1, body + AV_V27_QUERY_ARG);

	scoped_guard(mutex, &asrv->open_lock) {
		if (!asrv->open_srv)
			return -ENODEV;
		if (op == DCP_AUDIO_LINK_PREPARE) {
			asrv->link_gen = 0;
			if (!av_audiosrv_open_published(asrv))
				return -ENODEV;
		}
		/* A link ends on the instance it was prepared on; one gone took it along. */
		if (op != DCP_AUDIO_LINK_PREPARE && asrv->link_gen != asrv->open_gen)
			return -ENODEV;
		ret = av_audiosrv_call(asrv->open_srv, command, body, sizeof(body), 0,
				       reply, sizeof(reply));
		if (ret)
			return ret;
		status = get_unaligned_le32(by_format ? reply : reply + 48);
		if (op == DCP_AUDIO_LINK_QUERY) {
			dev_dbg(dev, "audiosrv: link state %#x\n", status);
			return 0;
		}
		if (by_format ? status != 1 : status) {
			dev_err(dev, "audiosrv: link method %u answered %#x\n", command, status);
			return -EIO;
		}
		if (op == DCP_AUDIO_LINK_PREPARE) {
			if (!av_audiosrv_open_published(asrv))
				return -ENODEV;
			asrv->link_gen = asrv->open_gen;
		}
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
	size_t received = 0, used;
	size_t bfr_len = output_maxsize + sizeof(*hdr);
	void *bfr __free(kfree) = NULL;
	int ret;

	KUNIT_STATIC_STUB_REDIRECT(dcp_audiosrv_osobject_call, service, group, command, output,
				   output_maxsize, output_size);
	bfr = kzalloc(bfr_len, GFP_KERNEL);
	if (!bfr)
		return -ENOMEM;

	hdr = bfr;
	hdr->max_size = cpu_to_le64(output_maxsize);
	if (READ_ONCE(service->ep->compact))
		ret = afk_service_call_with_reply_len(service, group, command, hdr,
						    sizeof(*hdr), output_maxsize, bfr,
						    bfr_len, 0, &received);
	else
		ret = afk_service_call(service, group, command, hdr, sizeof(*hdr),
				       output_maxsize, bfr, bfr_len, 0);
	if (ret)
		return ret;

	used = le64_to_cpu(hdr->used_size);
	if (READ_ONCE(service->ep->compact)) {
		if (used > output_maxsize || received < sizeof(*hdr) ||
		    used > received - sizeof(*hdr))
			return -EMSGSIZE;
		if (output) {
			memset(output, 0, output_maxsize);
			memcpy(output, bfr + sizeof(*hdr), used);
		}
	} else if (output) {
		memcpy(output, bfr + sizeof(*hdr), output_maxsize);
	}
	if (output_size)
		*output_size = used;

	return 0;
}

/*
 * A compact EPIC endpoint (desktop 26.x) carries a call's whole reply
 * capacity in band, through the endpoint's rings, where the transports
 * before it passed a buffer: a 256 KiB element capacity is some 255
 * fragments, which the firmware queues for the audio interface's thread.
 * While that thread is busy, as when a display that is plugged in has its
 * modes read, that queue overflows and the processor stops ("MBI queue
 * overflow:dcpav-interface-ep"). So a compact endpoint is asked with a
 * capacity a display's catalog fits in (7784 bytes for a 2560x1440
 * monitor), and the capacity grows only for a catalog that does not fit.
 * The firmware answers a capacity too small for the catalog with an empty
 * one (no error, nothing used), so an empty answer grows it too, as do one
 * that fills it and one reported larger than it; nothing else does (a
 * timeout leaves the endpoint unusable, other errors are not about size).
 * The capacity that fitted is kept for the next query. An instance that
 * answers even the whole capacity with an empty catalog has none: it is
 * asked once more only with the smallest capacity, until it is replaced.
 */
#define AV_COMPACT_ELEMENTS_MIN		SZ_16K
#define AV_COMPACT_ELEMENTS_GROWTH	4

static bool av_elements_need_more(int ret, size_t size, size_t cap, bool empty)
{
	if (!ret)
		return (!size && !empty) || size >= cap;
	return ret == -EMSGSIZE;
}

int dcp_audiosrv_get_elements(struct device *dev, void *elements, size_t maxsize)
{
	struct apple_dcp *dcp = dev_get_drvdata(dev);
	struct audiosrv_data *asrv = dcp->audiosrv;
	struct apple_epic_service *srv;
	size_t size = 0, cap;
	bool compact, empty;
	int ret;

	if (!asrv)
		return -ENODEV;
	scoped_guard(rwsem_read, &asrv->srv_rwsem)
		srv = afk_service_get(asrv->srv);
	if (!srv)
		return -ENODEV;

	compact = READ_ONCE(srv->ep->compact);
	empty = READ_ONCE(asrv->elements_empty);
	cap = maxsize;
	if (compact)
		cap = clamp(READ_ONCE(asrv->elements_cap) ?: AV_COMPACT_ELEMENTS_MIN,
			    min_t(size_t, AV_COMPACT_ELEMENTS_MIN, maxsize), maxsize);
	for (;;) {
		ret = dcp_audiosrv_osobject_call(srv, 1, asrv->cmds.get_elements,
						 elements, cap, &size);
		if (cap >= maxsize || !av_elements_need_more(ret, size, cap, empty))
			break;
		dev_dbg(dev, "audiosrv: elements do not fit %zu bytes (%d, %zu used), asking for more\n",
			cap, ret, size);
		cap = min(cap * AV_COMPACT_ELEMENTS_GROWTH, maxsize);
	}
	if (compact && !ret) {
		/* The call cleared the capacity it asked with; the rest goes too. */
		memset(elements + cap, 0, maxsize - cap);
		/* A smaller display's catalog goes back to a smaller capacity. */
		if (!size && cap >= maxsize)
			WRITE_ONCE(asrv->elements_empty, true);
		while (cap > AV_COMPACT_ELEMENTS_MIN &&
		       size < cap / AV_COMPACT_ELEMENTS_GROWTH)
			cap /= AV_COMPACT_ELEMENTS_GROWTH;
		WRITE_ONCE(asrv->elements_cap, max_t(size_t, cap, AV_COMPACT_ELEMENTS_MIN));
	}
	afk_service_put(srv);

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
	struct apple_epic_service *srv;
	size_t size;
	int ret;

	if (!asrv)
		return -ENODEV;
	scoped_guard(rwsem_read, &asrv->srv_rwsem)
		srv = afk_service_get(asrv->srv);
	if (!srv)
		return -ENODEV;
	ret = dcp_audiosrv_osobject_call(srv, 1,
					 asrv->cmds.get_product_attrs, attrs,
					 maxsize, &size);
	afk_service_put(srv);

	if (ret && !asrv->warned_get_product_attrs) {
		dev_err(dev, "audiosrv: error getting product attributes: %d\n", ret);
		asrv->warned_get_product_attrs = true;
	} else if (!ret) {
		dev_dbg(dev, "audiosrv: got %zd bytes worth of product attributes\n", size);
	}

	return ret;
}

/*
 * The ELD of the display this processor drives, as drm builds it from the
 * connector's EDID: monitor name, vendor and product codes, connection type
 * and the audio descriptors of a CTA-861 audio block, if there is one.
 * Zeroes without a connector or EDID.
 */
int dcp_audio_sink_eld(struct device *dev, u8 *eld, size_t size)
{
	struct apple_dcp *dcp;
	struct apple_connector *connector;

	KUNIT_STATIC_STUB_REDIRECT(dcp_audio_sink_eld, dev, eld, size);
	memset(eld, 0, size);
	dcp = dev_get_drvdata(dev);
	if (!dcp)
		return -ENODEV;
	connector = READ_ONCE(dcp->connector);
	if (!connector)
		return -ENODEV;
	guard(mutex)(&connector->base.eld_mutex);
	memcpy(eld, connector->base.eld, min(size, sizeof(connector->base.eld)));
	return 0;
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
	if (dcp_audiosrv_native_link(dcp->dev) && av_report_terminate(data, data_size)) {
		struct audiosrv_data *asrv = dcp->audiosrv;
		bool published;

		/* The work acts on this instance only, not on a later one. */
		scoped_guard(rwsem_read, &asrv->srv_rwsem) {
			published = asrv->srv == service && !asrv->reports_stopped;
			if (published)
				WRITE_ONCE(asrv->terminate_gen, asrv->gen);
		}
		if (published) {
			dev_info(dcp->dev, "audio interface terminated\n");
			schedule_work(&asrv->terminate_wq);
		}
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

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
/* Firmware 27: the video interface is kept for av_v27_start(). */
static void av_video_init(struct apple_epic_service *service, const char *name,
			  const char *class, s64 unit)
{
	struct audiosrv_data *asrv = service->ep->dcp->audiosrv;
	struct apple_epic_service *old;

	if (!asrv || !dcp_is_v27_external(service->ep->dcp))
		return;
	service = afk_service_get(service);
	if (!service)
		return;
	scoped_guard(rwsem_write, &asrv->srv_rwsem) {
		old = asrv->video_srv;
		asrv->video_srv = service;
	}
	if (old)
		afk_service_put(old);
}

static void av_video_teardown(struct apple_epic_service *service)
{
	struct audiosrv_data *asrv = service->ep->dcp->audiosrv;
	bool owned = false;

	if (asrv) {
		scoped_guard(rwsem_write, &asrv->srv_rwsem) {
			owned = asrv->video_srv == service;
			if (owned)
				asrv->video_srv = NULL;
		}
	}
	if (owned)
		afk_service_put(service);
	av_cec_teardown(service);
}

/*
 * The calls macOS 27 makes as the audio service starts (M5 Pro), before its
 * first link: on the audio interface 19, 15, 18, 14, 16, 20 and 13, then on
 * the video interface 22 and 33, with macOS's arguments (at body+0x20) and
 * sizes. 19, 18, 16 and 33 are OSObject calls; 18 and 16 are the product
 * attributes and the catalog. The answers are not used here.
 */
static const struct av_v27_start_call {
	bool video;
	bool object;
	u32 command;
	u32 len;
	u32 arg[4];
} av_v27_start_calls[] = {
	{ .object = true, .command = 19, .len = SZ_32K },
	{ .command = 15, .len = 0x10 },
	{ .object = true, .command = 18, .len = SZ_32K },
	{ .command = 14, .len = 0x30, .arg = { 2 } },
	{ .object = true, .command = 16, .len = SZ_256K },
	{ .command = 20, .len = 0x40, .arg = { 1, 2, 24, 48000 } },
	{ .command = 13, .len = 0x40, .arg = { 1 } },
	{ .video = true, .command = 22, .len = 0x30, .arg = { 2 } },
	{ .video = true, .object = true, .command = 33, .len = SZ_32K },
};

static void av_v27_start(struct apple_dcp *dcp, struct apple_epic_service *srv)
{
	struct audiosrv_data *asrv = dcp->audiosrv;
	struct apple_epic_service *video;
	unsigned int i, j;
	int ret;

	scoped_guard(rwsem_read, &asrv->srv_rwsem)
		video = afk_service_get(asrv->video_srv);
	for (i = 0; i < ARRAY_SIZE(av_v27_start_calls); i++) {
		const struct av_v27_start_call *c = &av_v27_start_calls[i];
		struct apple_epic_service *to = c->video ? video : srv;
		u8 body[0x40] = {}, reply[0x40];

		if (!to) {
			dev_warn(dcp->dev, "audiosrv: no video interface for its calls\n");
			continue;
		}
		if (c->object) {
			/* group 1, as the catalog and the attributes */
			ret = dcp_audiosrv_osobject_call(to, 1, c->command, NULL, c->len, NULL);
		} else {
			for (j = 0; j < ARRAY_SIZE(c->arg); j++)
				put_unaligned_le32(c->arg[j], body + 0x20 + 4 * j);
			ret = av_audiosrv_call(to, c->command, body, c->len, 0, reply, c->len);
		}
		if (ret)
			dev_warn(dcp->dev, "audiosrv: start call %u failed: %d\n", c->command, ret);
	}
	if (video)
		afk_service_put(video);
}
#endif
#endif

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
static const struct apple_epic_service_ops av_desktop_ops[];

/*
 * The desktop 26.x external processors (T6041 DCPEXT) publish their audio
 * interface on endpoint 0x2b as dcpav-audio-interface-epic, unit 0. With
 * the role in front ("dispext0:") the announcement is longer than its
 * field, so it arrives cut, without the unit: admit the cut too, as unit 0.
 */
static int av_desktop_audio_prepare(struct apple_epic_service *service, const char *name,
				    const char *class, s64 unit)
{
	struct apple_dcp *dcp = service->ep->dcp;

	if (!av_desktop_audio_role(dcp) ||
	    service->ep->endpoint != AV_AUDIO_ENDPOINT_DESKTOP || !name)
		return -EINVAL;
	if (!strcmp(name, av_desktop_ops[0].name) && unit == 0)
		return 0;
	if (unit == -1 && afk_compact_cut_of(&av_desktop_ops[0], name))
		return 0;
	return -EINVAL;
}

static const struct apple_epic_service_ops av_desktop_ops[] = {
	{
		.name = "dcpav-audio-interface-epic",
		.compact_full = "dcpav-audio-interface-epic:0",
		.reusable = true,
		.singleton = true,
		.prepare = av_desktop_audio_prepare,
		.init = av_audiosrv_init,
		.report = av_audiosrv_report,
		.teardown = av_audiosrv_teardown,
	},
	DCP_CEC_COMPACT_SERVICE_OPS,
	{
		/*
		 * And its video interface, cut to "dcpav-video-interface-" on
		 * every display power-on. The video side runs through the
		 * IOMFB endpoint; take it like the CEC interface. Firmware 27
		 * gets macOS's two calls on it as audio starts (av_v27_start()).
		 */
		.name = "dcpav-video-interface-epic",
		.compact_full = "dcpav-video-interface-epic:0",
		.singleton = true,
		.init = av_video_init,
		.report = av_cec_report,
		.teardown = av_video_teardown,
	},
	{}
};
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

	/*
	 * dcp->avep alone does not mean an audio service: an HDMI processor
	 * whose audio is not started gets the endpoint for CEC only
	 * (dcp_cec_init_avep()), with no audio owner.
	 */
	if (!asrv)
		return;

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
	if (dcp_audiosrv_native_link(dcp->dev)) {
		if (dcp_is_v14_7_external(dcp) && !iomfb_v14_7_external_powered(dcp))
			goto out;
		if (dcp_is_v27_external(dcp)) {
			u64 gen;

			scoped_guard(rwsem_read, &asrv->srv_rwsem)
				gen = asrv->gen;
			if (READ_ONCE(asrv->v27_boot_gen) != gen) {
				WRITE_ONCE(asrv->v27_boot_gen, gen);
				av_v27_start(dcp, service);
			}
		}
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

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
/*
 * Before the display's mode is set: a native link's connection holds its
 * default link through the mode set (see dcpaud_before_modeset()).
 */
void av_audio_before_modeset(struct apple_dcp *dcp)
{
	struct audiosrv_data *asrv = dcp->audiosrv;
	struct platform_device *audio_dev = NULL;

	if (!asrv || !dcp_audiosrv_native_link(dcp->dev))
		return;
	scoped_guard(mutex, &asrv->plug_lock) {
		if (asrv->plugged)
			audio_dev = asrv->audio_dev;
	}
	/* Outside plug_lock, which the endpoint's receive path takes. */
	if (audio_dev)
		dcpaud_before_modeset(audio_dev);
}
#endif

/* @gen: only while that instance is published (NULL: whichever is) */
static void __av_service_disconnect(struct apple_dcp *dcp, const u64 *gen)
{
	struct apple_epic_service *service;
	struct audiosrv_data *asrv = dcp->audiosrv;
	int ret;

	/* A CEC-only endpoint has no audio owner (av_service_connect()). */
	if (!asrv)
		return;

	scoped_guard(rwsem_write, &asrv->srv_rwsem) {
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
		if (gen && asrv->gen != *gen)
			return;
#endif
		service = afk_service_pin(asrv->srv);
		if (!service)
			return;
	}

	/* close AV audio service */
	dev_info(dcp->dev, "%s: stopping audio service\n", __func__);
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	/* The 14.7 external role's card goes with the display, and an open PCM's service. */
	if (dcp_audiosrv_native_link(dcp->dev)) {
		struct platform_device *audio_dev = NULL;
		bool published = true;

		/*
		 * Each step acts on instance @gen only, checked as it is taken:
		 * a successor may be published and opened in between.
		 */
		scoped_guard(mutex, &asrv->plug_lock) {
			if (gen) {
				scoped_guard(rwsem_read, &asrv->srv_rwsem)
					published = asrv->gen == *gen;
			}
			if (published) {
				audio_dev = asrv->audio_dev;
				if (audio_dev)
					dcpaud_disconnect(audio_dev);
			}
		}
		if (!published)
			goto out;
		/*
		 * The open PCM's link ends on the service, before its Close;
		 * outside plug_lock, which the endpoint's receive path takes.
		 */
		if (audio_dev)
			dcpaud_release_link(audio_dev, gen);
		av_audiosrv_close(dcp, asrv, gen);
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

void av_service_disconnect(struct apple_dcp *dcp)
{
	__av_service_disconnect(dcp, NULL);
}

#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
/* The instance the Terminate came from, if it is still published. */
static void av_work_service_terminate(struct work_struct *work)
{
	struct audiosrv_data *asrv = container_of(work, struct audiosrv_data, terminate_wq);
	u64 gen = READ_ONCE(asrv->terminate_gen);
	struct apple_dcp *dcp = NULL;

	scoped_guard(rwsem_read, &asrv->srv_rwsem) {
		if (asrv->srv && asrv->srv->ep && asrv->gen == gen)
			dcp = asrv->srv->ep->dcp;
	}
	if (dcp)
		__av_service_disconnect(dcp, &gen);
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

static void av_audiosrv_flush_owner(struct audiosrv_data *asrv)
{
	flush_work(&asrv->start_av_service_wq);
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	flush_work(&asrv->terminate_wq);
#endif
}

/* Drain the captured owner, without following a later dcp publication. */
static bool av_audiosrv_drain_owner(struct audiosrv_data *asrv)
{
	bool first_stop;

	if (!asrv || !asrv->works_live)
		return false;

	down_write(&asrv->srv_rwsem);
	first_stop = !asrv->work_disabled;
	asrv->reports_stopped = true;
	asrv->work_disabled = true;
	up_write(&asrv->srv_rwsem);

	/*
	 * Queue admission is already refused. disable_work_sync then raises
	 * the work disable count and waits. No audio lock is held: the
	 * callback takes srv_rwsem and may RPC. A later call only flushes,
	 * so the disable count stays one for the rebind enable.
	 */
	if (first_stop) {
		disable_work_sync(&asrv->start_av_service_wq);
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
		disable_work_sync(&asrv->terminate_wq);
#endif
	} else {
		av_audiosrv_flush_owner(asrv);
	}
	return true;
}

static void av_audiosrv_block_owner_restart(struct audiosrv_data *asrv)
{
	if (!asrv)
		return;
	down_write(&asrv->srv_rwsem);
	asrv->restart_blocked = true;
	up_write(&asrv->srv_rwsem);
}

void av_audiosrv_quiesce(struct apple_dcp *dcp)
{
	struct audiosrv_data *asrv;
	struct apple_dcp_afkep *ep;
	struct workqueue_struct *wq;

	if (!dcp)
		return;
	/* Lifecycle callers stop the native initializer before this snapshot. */
	asrv = dcp->audiosrv;
	ep = dcp->avep;
	if (!av_audiosrv_drain_owner(asrv))
		return;
	if (IS_ERR_OR_NULL(ep))
		return;
	wq = ep->wq;
	if (!wq)
		return;
	flush_workqueue(wq);
	av_audiosrv_flush_owner(asrv);
}

static void av_audiosrv_cleanup(void *data)
{
	struct av_audiosrv_cleanup *cleanup = data;
	struct audiosrv_data *asrv = cleanup->owner;
	struct apple_dcp *dcp = cleanup->dcp;

	/* Keep RX alive until any owner work that may make an RPC has left. */
	if (asrv && asrv->works_live) {
		av_audiosrv_block_owner_restart(asrv);
		av_audiosrv_drain_owner(asrv);
	}
	afk_quiesce(cleanup->ep);
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
	    !IS_ERR_OR_NULL(cleanup->ep) &&
	    READ_ONCE(dcp->desktop->graph[cleanup->ep->endpoint - 0x20]) == cleanup->ep)
		WRITE_ONCE(dcp->desktop->graph[cleanup->ep->endpoint - 0x20], NULL);
	/* An old devres group must not unpublish a later session. */
	if (dcp->audiosrv == asrv) {
		if (dcp->avep == cleanup->ep)
			dcp->avep = NULL;
		dcp->audiosrv = NULL;
	}
}

static int av_audiosrv_add_action(struct device *dev, void (*action)(void *),
				  void *data)
{
	KUNIT_STATIC_STUB_REDIRECT(av_audiosrv_add_action, dev, action, data);
	return devm_add_action_or_reset(dev, action, data);
}

static int av_audiosrv_register_cleanup(struct apple_dcp *dcp,
					struct audiosrv_data *owner,
					struct apple_dcp_afkep *ep)
{
	owner->cleanup = (struct av_audiosrv_cleanup) {
		.dcp = dcp,
		.owner = owner,
		.ep = ep,
	};
	return av_audiosrv_add_action(dcp->dev, av_audiosrv_cleanup, &owner->cleanup);
}

void av_audiosrv_reports_restart(struct apple_dcp *dcp)
{
	struct audiosrv_data *asrv;

	if (!dcp)
		return;
	asrv = dcp->audiosrv;
	if (!asrv || !asrv->works_live || asrv->restart_blocked)
		return;

	down_write(&asrv->srv_rwsem);
	if (!asrv->work_disabled || asrv->restart_blocked) {
		up_write(&asrv->srv_rwsem);
		return;
	}
	asrv->work_disabled = false;
	enable_work(&asrv->start_av_service_wq);
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	enable_work(&asrv->terminate_wq);
#endif
	asrv->reports_stopped = false;
	up_write(&asrv->srv_rwsem);
}

int avep_init(struct apple_dcp *dcp)
{
	struct audiosrv_data *audiosrv_data;
	struct platform_device *audio_pdev;
	struct device *dev = dcp->dev;
	struct device_node *endpoint, *audio_node = NULL;
	const struct apple_epic_service_ops *ops = avep_ops;
	u32 av_endpoint = AV_ENDPOINT;
	int ret;

	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP ||
	    dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL ||
	    dcp_is_v27_external(dcp)) {
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
		if (!av_desktop_audio_role(dcp))
			return -EOPNOTSUPP;
		ops = av_desktop_ops;
		av_endpoint = AV_AUDIO_ENDPOINT_DESKTOP;
#else
		return -EOPNOTSUPP;
#endif
	}

	/*
	 * Only a processor whose port@0 links an audio device has DP/HDMI
	 * audio to start. Look before the firmware: a processor without one
	 * (an internal panel's, or one whose firmware has no audio service
	 * here, such as H17P) stays as it is, quietly.
	 */
	endpoint = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	if (endpoint) {
		audio_node = of_graph_get_remote_port_parent(endpoint);
		of_node_put(endpoint);
	}
	if (!audio_node || !of_device_is_available(audio_node)) {
		of_node_put(audio_node);
		dev_dbg(dev, "No audio support\n");
		return 0;
	}

	audiosrv_data = devm_kzalloc(dcp->dev, sizeof(*audiosrv_data), GFP_KERNEL);
	if (!audiosrv_data) {
		of_node_put(audio_node);
		return -ENOMEM;
	}
	init_rwsem(&audiosrv_data->srv_rwsem);
	mutex_init(&audiosrv_data->plug_lock);

	switch (dcp->fw_compat) {
	case DCP_FIRMWARE_V_12_3:
		audiosrv_data->cmds = dcp_av_audio_cmds_v12_3;
		break;
	case DCP_FIRMWARE_H17P:
		/* The J700 DCPEXT numbers its audio methods as 13.5 does. */
		if (dcp->hw.iomfb_method_profile != DCP_IOMFB_METHODS_H17P_EXTERNAL)
			goto unsupported;
		audiosrv_data->cmds = dcp_av_audio_cmds_v13_5;
		break;
	case DCP_FIRMWARE_V_13_5:
	case DCP_FIRMWARE_V_14_7_TEMPLATE:
	case DCP_FIRMWARE_V_26_6_DESKTOP:
		audiosrv_data->cmds = dcp_av_audio_cmds_v13_5;
		break;
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	case DCP_FIRMWARE_V_27_0:
		audiosrv_data->cmds = dcp_av_audio_cmds_v27;
		break;
#endif
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
unsupported:
		of_node_put(audio_node);
		dev_err(dcp->dev, "Audio not supported for firmware\n");
		return -ENODEV;
	}

	INIT_WORK(&audiosrv_data->start_av_service_wq, av_work_service_start);
#if IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
	INIT_WORK(&audiosrv_data->terminate_wq, av_work_service_terminate);
	mutex_init(&audiosrv_data->open_lock);
	init_waitqueue_head(&audiosrv_data->gen_wq);
#endif
	audiosrv_data->works_live = true;
	dcp->audiosrv = audiosrv_data;

	audio_pdev = of_find_device_by_node(audio_node);
	of_node_put(audio_node);
	if (!audio_pdev) {
		dev_info(dev, "No DP/HDMI audio device not ready\n");
		return av_audiosrv_register_cleanup(dcp, audiosrv_data, NULL);
	}
	dcp->audiosrv->audio_dev = audio_pdev;

	device_link_add(&audio_pdev->dev, dev,
			DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME);

	/* The J700 and T6050 DCPEXTs: start the audio endpoint only if the firmware has it. */
	if ((dcp->hw.iomfb_method_profile == DCP_IOMFB_METHODS_H17P_EXTERNAL ||
	     dcp_is_v27_external(dcp)) &&
	    !apple_rtkit_has_endpoint(dcp->rtk, av_endpoint)) {
		dev_info(dev, "no DP/HDMI audio endpoint\n");
		return av_audiosrv_register_cleanup(dcp, audiosrv_data, NULL);
	}

	dcp->avep = afk_init(dcp, av_endpoint, ops);
	if (IS_ERR(dcp->avep)) {
		ret = PTR_ERR(dcp->avep);
		if (!av_audiosrv_register_cleanup(dcp, audiosrv_data, dcp->avep))
			av_audiosrv_cleanup(&audiosrv_data->cleanup);
		return ret;
	}
	/* Newer than afk_release_context, so owner work drains before RX dies. */
	ret = av_audiosrv_register_cleanup(dcp, audiosrv_data, dcp->avep);
	if (ret)
		return ret;
	dcp->avep->debugfs_entry = dcp->ep_debugfs[av_endpoint - 0x20];
	/* afk_start waits for RX: publish the desktop route before starting. */
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP)
		WRITE_ONCE(dcp->desktop->graph[av_endpoint - 0x20], dcp->avep);
	ret = afk_start(dcp->avep);
	if (ret)
		av_audiosrv_cleanup(&audiosrv_data->cleanup);
	return ret;
}

#if IS_ENABLED(CONFIG_DRM_APPLE_DISPLAY_KUNIT_TEST) && IS_ENABLED(CONFIG_DRM_APPLE_DCP_T6030_AUDIO)
#include "av-v14-test.c"
#include "av-lifetime-test.c"
#endif
