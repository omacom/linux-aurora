// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple RTKit IPC library
 * Copyright (C) The Asahi Linux Contributors
 */

#include "rtkit-internal.h"
#include <linux/of.h>

enum {
	APPLE_RTKIT_PWR_STATE_OFF = 0x00, /* power off, cannot be restarted */
	APPLE_RTKIT_PWR_STATE_SLEEP = 0x01, /* sleeping, can be restarted */
	APPLE_RTKIT_PWR_STATE_IDLE = 0x201, /* sleeping, retain state */
	APPLE_RTKIT_PWR_STATE_QUIESCED = 0x10, /* running but no communication */
	APPLE_RTKIT_PWR_STATE_ON = 0x20, /* normal operating state */
	APPLE_RTKIT_PWR_STATE_INIT = 0x220, /* init after starting the coproc */
};

enum {
	APPLE_RTKIT_EP_MGMT = 0,
	APPLE_RTKIT_EP_CRASHLOG = 1,
	APPLE_RTKIT_EP_SYSLOG = 2,
	APPLE_RTKIT_EP_DEBUG = 3,
	APPLE_RTKIT_EP_IOREPORT = 4,
	APPLE_RTKIT_EP_PMP_AUX = 7,
	APPLE_RTKIT_EP_OSLOG = 8,
	APPLE_RTKIT_EP_TRACEKIT = 0xa,
};

#define APPLE_RTKIT_MGMT_TYPE GENMASK_ULL(59, 52)

enum {
	APPLE_RTKIT_MGMT_HELLO = 1,
	APPLE_RTKIT_MGMT_HELLO_REPLY = 2,
	APPLE_RTKIT_MGMT_STARTEP = 5,
	APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE = 6,
	APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE_ACK = 7,
	APPLE_RTKIT_MGMT_EPMAP = 8,
	APPLE_RTKIT_MGMT_EPMAP_REPLY = 8,
	APPLE_RTKIT_MGMT_SET_AP_PWR_STATE = 0xb,
	APPLE_RTKIT_MGMT_SET_AP_PWR_STATE_ACK = 0xb,
	APPLE_RTKIT_MGMT_IOP_IDLE = 0xc,
};

#define APPLE_RTKIT_MGMT_HELLO_MINVER GENMASK_ULL(15, 0)
#define APPLE_RTKIT_MGMT_HELLO_MAXVER GENMASK_ULL(31, 16)

#define APPLE_RTKIT_MGMT_EPMAP_LAST   BIT_ULL(51)
#define APPLE_RTKIT_MGMT_EPMAP_BASE   GENMASK_ULL(34, 32)
#define APPLE_RTKIT_MGMT_EPMAP_BITMAP GENMASK_ULL(31, 0)

#define APPLE_RTKIT_MGMT_EPMAP_REPLY_MORE BIT_ULL(0)

#define APPLE_RTKIT_MGMT_STARTEP_EP   GENMASK_ULL(39, 32)
#define APPLE_RTKIT_MGMT_STARTEP_FLAG BIT_ULL(1)

#define APPLE_RTKIT_MGMT_PWR_STATE GENMASK_ULL(15, 0)

#define APPLE_RTKIT_CRASHLOG_CRASH 1

#define APPLE_RTKIT_BUFFER_REQUEST	1
#define APPLE_RTKIT_BUFFER_REQUEST_SIZE GENMASK_ULL(51, 44)
#define APPLE_RTKIT_BUFFER_REQUEST_IOVA GENMASK_ULL(43, 0)

#define APPLE_RTKIT_SYSLOG_TYPE GENMASK_ULL(59, 52)

#define APPLE_RTKIT_SYSLOG_LOG 5

#define APPLE_RTKIT_SYSLOG_INIT	     8
#define APPLE_RTKIT_SYSLOG_N_ENTRIES GENMASK_ULL(7, 0)
#define APPLE_RTKIT_SYSLOG_MSG_SIZE  GENMASK_ULL(31, 24)

#define APPLE_RTKIT_OSLOG_TYPE GENMASK_ULL(63, 56)
#define APPLE_RTKIT_OSLOG_BUFFER_REQUEST 1
#define APPLE_RTKIT_OSLOG_SIZE GENMASK_ULL(55, 36)
#define APPLE_RTKIT_OSLOG_IOVA GENMASK_ULL(35, 0)

#define APPLE_RTKIT_MIN_SUPPORTED_VERSION 11
#define APPLE_RTKIT_MAX_SUPPORTED_VERSION 12

struct apple_rtkit_rx_work {
	struct apple_rtkit *rtk;
	u8 ep;
	u64 msg;
	struct work_struct work;
};

bool apple_rtkit_is_running(struct apple_rtkit *rtk)
{
	if (rtk->crashed)
		return false;
	if ((rtk->iop_power_state & 0xff) != APPLE_RTKIT_PWR_STATE_ON)
		return false;
	if ((rtk->ap_power_state & 0xff) != APPLE_RTKIT_PWR_STATE_ON)
		return false;
	return true;
}
EXPORT_SYMBOL_GPL(apple_rtkit_is_running);

bool apple_rtkit_is_crashed(struct apple_rtkit *rtk)
{
	return rtk->crashed;
}
EXPORT_SYMBOL_GPL(apple_rtkit_is_crashed);

static int apple_rtkit_management_send(struct apple_rtkit *rtk, u8 type,
					u64 msg)
{
	int ret;

	msg &= ~APPLE_RTKIT_MGMT_TYPE;
	msg |= FIELD_PREP(APPLE_RTKIT_MGMT_TYPE, type);
	ret = apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_MGMT, msg, NULL, false);

	if (ret)
		dev_err(rtk->dev, "RTKit: Failed to send management message: %d\n", ret);

	return ret;
}

static void apple_rtkit_management_rx_hello(struct apple_rtkit *rtk, u64 msg)
{
	u64 reply;

	int min_ver = FIELD_GET(APPLE_RTKIT_MGMT_HELLO_MINVER, msg);
	int max_ver = FIELD_GET(APPLE_RTKIT_MGMT_HELLO_MAXVER, msg);
	int want_ver = min(APPLE_RTKIT_MAX_SUPPORTED_VERSION, max_ver);

	dev_dbg(rtk->dev, "RTKit: Min ver %d, max ver %d\n", min_ver, max_ver);

	if (min_ver > APPLE_RTKIT_MAX_SUPPORTED_VERSION) {
		dev_err(rtk->dev, "RTKit: Firmware min version %d is too new\n",
			min_ver);
		goto abort_boot;
	}

	if (max_ver < APPLE_RTKIT_MIN_SUPPORTED_VERSION) {
		dev_err(rtk->dev, "RTKit: Firmware max version %d is too old\n",
			max_ver);
		goto abort_boot;
	}

	dev_info(rtk->dev, "RTKit: Initializing (protocol version %d)\n",
		 want_ver);
	rtk->version = want_ver;

	reply = FIELD_PREP(APPLE_RTKIT_MGMT_HELLO_MINVER, want_ver);
	reply |= FIELD_PREP(APPLE_RTKIT_MGMT_HELLO_MAXVER, want_ver);
	apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_HELLO_REPLY, reply);

	return;

abort_boot:
	rtk->boot_result = -EINVAL;
	complete_all(&rtk->epmap_completion);
}

static void apple_rtkit_management_rx_epmap(struct apple_rtkit *rtk, u64 msg)
{
	int i, ep, ret;
	u64 reply;
	unsigned long bitmap = FIELD_GET(APPLE_RTKIT_MGMT_EPMAP_BITMAP, msg);
	u32 base = FIELD_GET(APPLE_RTKIT_MGMT_EPMAP_BASE, msg);

	dev_dbg(rtk->dev,
		"RTKit: received endpoint bitmap 0x%lx with base 0x%x\n",
		bitmap, base);

	for_each_set_bit(i, &bitmap, 32) {
		ep = 32 * base + i;
		dev_dbg(rtk->dev, "RTKit: Discovered endpoint 0x%02x\n", ep);
		set_bit(ep, rtk->endpoints);
	}

	/* G17P can request its crash DVA immediately after EP1 starts. */
	if (rtk->firmware_owned_crashlog &&
	    test_bit(APPLE_RTKIT_EP_CRASHLOG, rtk->endpoints) &&
	    !rtk->crashlog_endpoint_started) {
		ret = apple_rtkit_start_ep(rtk, APPLE_RTKIT_EP_CRASHLOG);
		if (ret) {
			rtk->boot_result = ret;
			complete_all(&rtk->epmap_completion);
			return;
		}
		rtk->crashlog_endpoint_started = true;
	}

	reply = FIELD_PREP(APPLE_RTKIT_MGMT_EPMAP_BASE, base);
	if (msg & APPLE_RTKIT_MGMT_EPMAP_LAST)
		reply |= APPLE_RTKIT_MGMT_EPMAP_LAST;
	else
		reply |= APPLE_RTKIT_MGMT_EPMAP_REPLY_MORE;

	apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_EPMAP_REPLY, reply);

	if (!(msg & APPLE_RTKIT_MGMT_EPMAP_LAST))
		return;

	for_each_set_bit(ep, rtk->endpoints, APPLE_RTKIT_APP_ENDPOINT_START) {
		switch (ep) {
		/* the management endpoint is started by default */
		case APPLE_RTKIT_EP_MGMT:
			break;
		case APPLE_RTKIT_EP_PMP_AUX:
			if (rtk->t8152_pmp) {
				apple_rtkit_start_ep(rtk, ep);
				break;
			}
			dev_warn(rtk->dev, "RTKit: unqualified endpoint 7\n");
			break;

		/* without starting these RTKit refuses to boot */
		case APPLE_RTKIT_EP_SYSLOG:
		case APPLE_RTKIT_EP_CRASHLOG:
		case APPLE_RTKIT_EP_DEBUG:
		case APPLE_RTKIT_EP_IOREPORT:
		case APPLE_RTKIT_EP_OSLOG:
		case APPLE_RTKIT_EP_TRACEKIT:
			if (ep == APPLE_RTKIT_EP_CRASHLOG &&
			    rtk->crashlog_endpoint_started)
				break;
			dev_dbg(rtk->dev,
				"RTKit: Starting system endpoint 0x%02x\n", ep);
			apple_rtkit_start_ep(rtk, ep);
			break;

		default:
			dev_warn(rtk->dev,
				 "RTKit: Unknown system endpoint: 0x%02x\n",
				 ep);
		}
	}

	rtk->boot_result = 0;
	complete_all(&rtk->epmap_completion);
}

static void apple_rtkit_management_rx_iop_pwr_ack(struct apple_rtkit *rtk,
						  u64 msg)
{
	unsigned int new_state = FIELD_GET(APPLE_RTKIT_MGMT_PWR_STATE, msg);

	dev_dbg(rtk->dev, "RTKit: IOP power state transition: 0x%x -> 0x%x\n",
		rtk->iop_power_state, new_state);
	rtk->iop_power_state = new_state;

	complete_all(&rtk->iop_pwr_ack_completion);
}

static void apple_rtkit_management_rx_ap_pwr_ack(struct apple_rtkit *rtk,
						 u64 msg)
{
	unsigned int new_state = FIELD_GET(APPLE_RTKIT_MGMT_PWR_STATE, msg);

	dev_dbg(rtk->dev, "RTKit: AP power state transition: 0x%x -> 0x%x\n",
		rtk->ap_power_state, new_state);
	rtk->ap_power_state = new_state;

	complete_all(&rtk->ap_pwr_ack_completion);
}

static void apple_rtkit_management_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_MGMT_TYPE, msg);

	if (type == APPLE_RTKIT_MGMT_IOP_IDLE &&
	    READ_ONCE(rtk->awaiting_iop_idle) &&
	    !(msg & ~APPLE_RTKIT_MGMT_TYPE)) {
		complete_all(&rtk->iop_idle_completion);
		return;
	}

	switch (type) {
	case APPLE_RTKIT_MGMT_HELLO:
		apple_rtkit_management_rx_hello(rtk, msg);
		break;
	case APPLE_RTKIT_MGMT_EPMAP:
		apple_rtkit_management_rx_epmap(rtk, msg);
		break;
	case APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE_ACK:
		apple_rtkit_management_rx_iop_pwr_ack(rtk, msg);
		break;
	case APPLE_RTKIT_MGMT_SET_AP_PWR_STATE_ACK:
		apple_rtkit_management_rx_ap_pwr_ack(rtk, msg);
		break;
	default:
		dev_warn(
			rtk->dev,
			"RTKit: unknown management message: 0x%llx (type: 0x%02x)\n",
			msg, type);
	}
}

static int apple_rtkit_common_rx_get_buffer(struct apple_rtkit *rtk,
					    struct apple_rtkit_shmem *buffer,
					    u8 ep, u64 msg)
{
	struct apple_rtkit_shmem request = {};
	u64 reply;
	int err;

	/* The different size vs. IOVA shifts look odd but are indeed correct this way */
	if (ep == APPLE_RTKIT_EP_OSLOG) {
		request.size = FIELD_GET(APPLE_RTKIT_OSLOG_SIZE, msg);
		request.iova = FIELD_GET(APPLE_RTKIT_OSLOG_IOVA, msg) << 12;
	} else {
		request.size = FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_SIZE, msg) << 12;
		request.iova = FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_IOVA, msg);
	}
	if (!request.size) {
		err = -EINVAL;
		goto error;
	}

	/*
	 * A repeated request must not retire a buffer the firmware may still be
	 * using. Re-send the reply for an identical request and refuse a changed
	 * one; a different geometry needs a confirmed firmware restart (reinit)
	 * first.
	 */
	if (buffer->size) {
		if (request.size != buffer->size ||
		    (request.iova != buffer->iova &&
		     (request.iova || buffer->is_mapped))) {
			dev_err(rtk->dev,
				"RTKit: buffer request for 0x%zx bytes at %pad while 0x%zx bytes at %pad are live\n",
				request.size, &request.iova, buffer->size,
				&buffer->iova);
			return -EBUSY;
		}
		if (rtk->firmware_owned_crashlog &&
		    ep == APPLE_RTKIT_EP_CRASHLOG)
			return apple_rtkit_send_message(rtk, ep, msg, NULL, false);
		goto reply;
	}

	dev_dbg(rtk->dev, "RTKit: buffer request for 0x%zx bytes at %pad\n",
		request.size, &request.iova);
	/* This DVA belongs to G17P's GPU UAT, not Linux's device DMA domain. */
	if (rtk->firmware_owned_crashlog &&
	    ep == APPLE_RTKIT_EP_CRASHLOG && request.iova) {
		*buffer = request;
		buffer->is_mapped = true;
		err = apple_rtkit_send_message(rtk, ep, msg, NULL, false);
		if (err) {
			memset(buffer, 0, sizeof(*buffer));
			goto error;
		}
		return 0;
	}

	if (request.iova && !rtk->ops->shmem_setup) {
		err = -EINVAL;
		goto error;
	}

	if (rtk->ops->shmem_setup) {
		err = rtk->ops->shmem_setup(rtk->cookie, &request);
		if (err)
			goto error;
	} else {
		request.buffer = dma_alloc_coherent(rtk->dev, request.size,
						    &request.iova, GFP_KERNEL);
		if (!request.buffer) {
			err = -ENOMEM;
			goto error;
		}
	}
	*buffer = request;

reply:
	if (!buffer->is_mapped) {
		/* oslog uses different fields and needs a shifted IOVA instead of size */
		if (ep == APPLE_RTKIT_EP_OSLOG) {
			reply = FIELD_PREP(APPLE_RTKIT_OSLOG_TYPE,
					   APPLE_RTKIT_OSLOG_BUFFER_REQUEST);
			reply |= FIELD_PREP(APPLE_RTKIT_OSLOG_SIZE, buffer->size);
			reply |= FIELD_PREP(APPLE_RTKIT_OSLOG_IOVA,
					    buffer->iova >> 12);
		} else {
			reply = FIELD_PREP(APPLE_RTKIT_SYSLOG_TYPE,
					   ep == APPLE_RTKIT_EP_PMP_AUX ? 5 :
					   APPLE_RTKIT_BUFFER_REQUEST);
			reply |= FIELD_PREP(APPLE_RTKIT_BUFFER_REQUEST_SIZE,
					    buffer->size >> 12);
			reply |= FIELD_PREP(APPLE_RTKIT_BUFFER_REQUEST_IOVA,
					    buffer->iova);
		}
		apple_rtkit_send_message(rtk, ep, reply, NULL, false);
	}

	return 0;

error:
	dev_err(rtk->dev, "RTKit: failed buffer request for 0x%zx bytes (%d)\n",
		request.size, err);
	return err;
}

static void apple_rtkit_free_buffer(struct apple_rtkit *rtk,
				    struct apple_rtkit_shmem *bfr)
{
	if (bfr->size == 0)
		return;

	if (rtk->ops->shmem_destroy)
		rtk->ops->shmem_destroy(rtk->cookie, bfr);
	else if (bfr->buffer)
		dma_free_coherent(rtk->dev, bfr->size, bfr->buffer, bfr->iova);

	bfr->buffer = NULL;
	bfr->iomem = NULL;
	bfr->iova = 0;
	bfr->size = 0;
	bfr->is_mapped = false;
	bfr->needs_dma_sync = false;
	bfr->private = NULL;
}

static void apple_rtkit_memcpy(struct apple_rtkit *rtk, void *dst,
			       struct apple_rtkit_shmem *bfr, size_t offset,
			       size_t len)
{
	bool sync = bfr->needs_dma_sync && !WARN_ON_ONCE(bfr->iomem);

	if (sync)
		dma_sync_single_range_for_cpu(rtk->dev, bfr->iova, offset, len,
					      DMA_FROM_DEVICE);
	if (bfr->iomem)
		memcpy_fromio(dst, bfr->iomem + offset, len);
	else
		memcpy(dst, bfr->buffer + offset, len);
	if (sync)
		dma_sync_single_range_for_device(rtk->dev, bfr->iova, offset,
						 len, DMA_FROM_DEVICE);
}

static void apple_rtkit_crashlog_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_SYSLOG_TYPE, msg);
	u8 *bfr __free(kfree) = NULL;

	if (type != APPLE_RTKIT_CRASHLOG_CRASH) {
		dev_warn(rtk->dev, "RTKit: Unknown crashlog message: %llx\n",
			 msg);
		return;
	}

	if (rtk->crashlog_inherited && !rtk->crashlog_buffer.size) {
		/*
		 * The previous owner negotiated the crashlog buffer, so this
		 * session never learned its geometry. A CRASH message on an
		 * inherited session is therefore a crash notification, not a
		 * buffer request, and must not be answered as one.
		 */
		dev_err(rtk->dev,
			"RTKit: adopted co-processor has crashed (crashlog at %#llx, %#llx bytes, not mapped)\n",
			(u64)FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_IOVA, msg),
			(u64)FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_SIZE, msg) << 12);
		rtk->crashed = true;
		if (rtk->ops->crashed)
			rtk->ops->crashed(rtk->cookie, NULL, 0);
		return;
	}

	if (!rtk->crashlog_buffer.size) {
		struct apple_rtkit_crashlog_header_probe {
			u8 bytes[0x20];
		} probe;

		apple_rtkit_common_rx_get_buffer(rtk, &rtk->crashlog_buffer,
						 APPLE_RTKIT_EP_CRASHLOG, msg);
		/*
		 * A firmware whose crashlog endpoint was never started by us --
		 * the J700's loader-booted DCP, handed over running -- sends its
		 * first crashlog message only once it is already crashing.  The
		 * buffer it names then holds the finished crash log and no second
		 * message follows (2026-09-19 06:02: "undiscovered endpoint 0x01",
		 * the buffer mapped, silence, and a dead panel 530 s later).  If
		 * the buffer already carries a crash log header, this IS the
		 * crash: fall through and dump it now.
		 */
		if (rtk->crashlog_buffer.size < sizeof(probe) ||
		    (!rtk->crashlog_buffer.buffer && !rtk->crashlog_buffer.iomem))
			return;
	}
	if (reports_crash) {
		if (rtk->crashlog_buffer.size <
		    sizeof(struct apple_rtkit_crashlog_header))
			goto no_buffer;
		/* A firmware-owned buffer has no CPU mapping to read a header from. */
		if ((rtk->crashlog_buffer.buffer || rtk->crashlog_buffer.iomem) &&
		    !apple_rtkit_crashlog_header_valid(rtk))
			goto no_log;
	}

	dev_err(rtk->dev, "RTKit: co-processor has crashed\n");
	if (rtk->firmware_owned_crashlog && !rtk->crashlog_buffer.buffer &&
	    !rtk->crashlog_buffer.iomem) {
		dev_err(rtk->dev,
			"RTKit: GPU-owned crash log at DVA %pad (0x%zx bytes)\n",
			&rtk->crashlog_buffer.iova, rtk->crashlog_buffer.size);
		rtk->crashed = true;
		if (rtk->ops->crashed)
			rtk->ops->crashed(rtk->cookie, NULL, 0);
		return;
	}

	/*
	 * create a shadow copy here to make sure the co-processor isn't able
	 * to change the log while we're dumping it. this also ensures
	 * the buffer is in normal memory and not iomem for e.g. the SMC
	 */
	bfr = kzalloc(rtk->crashlog_buffer.size, GFP_KERNEL);
	if (bfr) {
		apple_rtkit_memcpy(rtk, bfr, &rtk->crashlog_buffer, 0,
				   rtk->crashlog_buffer.size);
		apple_rtkit_crashlog_dump(rtk, bfr, rtk->crashlog_buffer.size);
	} else {
		dev_err(rtk->dev,
			"RTKit: Couldn't allocate crashlog shadow buffer\n");
	}

	rtk->crashed = true;
	if (rtk->ops->crashed)
		rtk->ops->crashed(rtk->cookie, bfr, bfr ? rtk->crashlog_buffer.size : 0);
}

static void apple_rtkit_ioreport_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_SYSLOG_TYPE, msg);

	switch (type) {
	case APPLE_RTKIT_BUFFER_REQUEST:
		apple_rtkit_common_rx_get_buffer(rtk, &rtk->ioreport_buffer,
						 APPLE_RTKIT_EP_IOREPORT, msg);
		break;
	/* unknown, must be ACKed or the co-processor will hang */
	case 0x8:
	case 0xc:
		apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_IOREPORT, msg,
					 NULL, false);
		break;
	default:
		dev_warn(rtk->dev, "RTKit: Unknown ioreport message: %llx\n",
			 msg);
	}
}

static void apple_rtkit_syslog_rx_init(struct apple_rtkit *rtk, u64 msg)
{
	size_t entries = FIELD_GET(APPLE_RTKIT_SYSLOG_N_ENTRIES, msg);
	size_t size = FIELD_GET(APPLE_RTKIT_SYSLOG_MSG_SIZE, msg);
	char *buffer = NULL;

	rtk->syslog_inherited = false;

	/*
	 * Firmware may send SYSLOG_INIT again after it restarts. The ordered RX
	 * worker is the only user of the message buffer, so it can simply be
	 * replaced here. A zero-sized geometry must not allocate: kzalloc(0)
	 * returns ZERO_SIZE_PTR, which the log path would then index.
	 */
	if (entries && size)
		buffer = kzalloc(size, GFP_KERNEL);
	kfree(rtk->syslog_msg_buffer);
	rtk->syslog_msg_buffer = buffer;
	rtk->syslog_n_entries = entries;
	rtk->syslog_msg_size = size;

	dev_dbg(rtk->dev,
		"RTKit: syslog initialized: entries: %zd, msg_size: %zd\n",
		entries, size);
}

static bool should_crop_syslog_char(char c)
{
	return c == '\n' || c == '\r' || c == ' ' || c == '\0';
}

static void apple_rtkit_syslog_rx_log(struct apple_rtkit *rtk, u64 msg)
{
	u8 idx = msg & 0xff;
	char log_context[24];
	size_t entry_size = 0x20 + rtk->syslog_msg_size;
	int msglen;

	/*
	 * A session inherited from the bootloader never saw SYSLOG_INIT, so the
	 * ring cannot be interpreted; the records still have to be acknowledged
	 * for the co-processor to make progress. A later INIT restores parsing.
	 */
	if (rtk->syslog_inherited)
		goto done;

	if (!rtk->syslog_msg_buffer) {
		dev_warn_ratelimited(
			rtk->dev,
			"RTKit: received syslog message but no syslog_msg_buffer\n");
		goto done;
	}
	if (!rtk->syslog_buffer.size) {
		dev_warn_ratelimited(
			rtk->dev,
			"RTKit: received syslog message but syslog_buffer.size is zero\n");
		goto done;
	}
	if (!rtk->syslog_buffer.buffer && !rtk->syslog_buffer.iomem) {
		dev_warn_ratelimited(
			rtk->dev,
			"RTKit: received syslog message but no syslog_buffer.buffer or syslog_buffer.iomem\n");
		goto done;
	}
	if (idx >= rtk->syslog_n_entries ||
	    idx >= rtk->syslog_buffer.size / entry_size) {
		dev_warn_ratelimited(rtk->dev,
				     "RTKit: syslog index %d out of range\n", idx);
		goto done;
	}

	apple_rtkit_memcpy(rtk, log_context, &rtk->syslog_buffer,
			   idx * entry_size + 8, sizeof(log_context));
	apple_rtkit_memcpy(rtk, rtk->syslog_msg_buffer, &rtk->syslog_buffer,
			   idx * entry_size + 8 + sizeof(log_context),
			   rtk->syslog_msg_size);

	log_context[sizeof(log_context) - 1] = 0;

	msglen = strnlen(rtk->syslog_msg_buffer, rtk->syslog_msg_size - 1);
	while (msglen > 0 &&
		   should_crop_syslog_char(rtk->syslog_msg_buffer[msglen - 1]))
		msglen--;

	rtk->syslog_msg_buffer[msglen] = 0;
	dev_info(rtk->dev, "RTKit: syslog message: %s: %s\n", log_context,
		 rtk->syslog_msg_buffer);

done:
	apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_SYSLOG, msg, NULL, false);
}

static void apple_rtkit_syslog_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_SYSLOG_TYPE, msg);

	switch (type) {
	case APPLE_RTKIT_BUFFER_REQUEST:
		apple_rtkit_common_rx_get_buffer(rtk, &rtk->syslog_buffer,
						 APPLE_RTKIT_EP_SYSLOG, msg);
		break;
	case APPLE_RTKIT_SYSLOG_INIT:
		apple_rtkit_syslog_rx_init(rtk, msg);
		break;
	case APPLE_RTKIT_SYSLOG_LOG:
		apple_rtkit_syslog_rx_log(rtk, msg);
		break;
	default:
		dev_warn(rtk->dev, "RTKit: Unknown syslog message: %llx\n",
			 msg);
	}
}

static void apple_rtkit_oslog_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_OSLOG_TYPE, msg);

	switch (type) {
	case APPLE_RTKIT_OSLOG_BUFFER_REQUEST:
		apple_rtkit_common_rx_get_buffer(rtk, &rtk->oslog_buffer,
						 APPLE_RTKIT_EP_OSLOG, msg);
		break;
	default:
		dev_warn(rtk->dev, "RTKit: Unknown oslog message: %llx\n",
			 msg);
	}
}

/* Native J873 26A428 capture074: EP7 requests 8KiB with type1 and
 * receives type5, the same size and an unshifted 44-bit DVA. Keep the
 * buffer alive for the RTKit session; never replace a live allocation.
 */
static void apple_rtkit_pmp_aux_rx(struct apple_rtkit *rtk, u64 msg)
{
	int ret;

	if (!rtk->t8152_pmp || msg != 0x10200000000000ULL ||
	    rtk->pmp_aux_buffer.size) {
		dev_err(rtk->dev, "RTKit: refused PMP EP7 request %#llx\n", msg);
		return;
	}
	ret = apple_rtkit_common_rx_get_buffer(rtk, &rtk->pmp_aux_buffer,
					     APPLE_RTKIT_EP_PMP_AUX, msg);
	dev_info(rtk->dev, "RTKit: PMP EP7 8KiB buffer handshake result=%d\n", ret);
}

static void apple_rtkit_rx_work(struct work_struct *work)
{
	struct apple_rtkit_rx_work *rtk_work =
		container_of(work, struct apple_rtkit_rx_work, work);
	struct apple_rtkit *rtk = rtk_work->rtk;

	if (READ_ONCE(rtk->shutting_down))
		goto out;

	switch (rtk_work->ep) {
	case APPLE_RTKIT_EP_MGMT:
		apple_rtkit_management_rx(rtk, rtk_work->msg);
		break;
	case APPLE_RTKIT_EP_CRASHLOG:
		apple_rtkit_crashlog_rx(rtk, rtk_work->msg);
		break;
	case APPLE_RTKIT_EP_SYSLOG:
		apple_rtkit_syslog_rx(rtk, rtk_work->msg);
		break;
	case APPLE_RTKIT_EP_IOREPORT:
		apple_rtkit_ioreport_rx(rtk, rtk_work->msg);
		break;
	case APPLE_RTKIT_EP_OSLOG:
		apple_rtkit_oslog_rx(rtk, rtk_work->msg);
		break;
	case APPLE_RTKIT_EP_PMP_AUX:
		apple_rtkit_pmp_aux_rx(rtk, rtk_work->msg);
		break;
	case APPLE_RTKIT_APP_ENDPOINT_START ... 0xff:
		if (rtk->ops->recv_message)
			rtk->ops->recv_message(rtk->cookie, rtk_work->ep,
					       rtk_work->msg);
		else
			dev_warn(
				rtk->dev,
				"Received unexpected message to EP%02d: %llx\n",
				rtk_work->ep, rtk_work->msg);
		break;
	default:
		dev_warn(rtk->dev,
			 "RTKit: message to unknown endpoint %02x: %llx\n",
			 rtk_work->ep, rtk_work->msg);
	}

out:
	kfree(rtk_work);
}

static void apple_rtkit_rx(struct apple_mbox *mbox, struct apple_mbox_msg msg,
			   void *cookie)
{
	struct apple_rtkit *rtk = cookie;
	struct apple_rtkit_rx_work *work;
	u8 ep = msg.msg1;

	if (READ_ONCE(rtk->shutting_down))
		return;

	/*
	 * The message was read from a MMIO FIFO and we have to make
	 * sure all reads from buffers sent with that message happen
	 * afterwards.
	 */
	dma_rmb();

	if (!test_bit(ep, rtk->endpoints))
		dev_warn(rtk->dev,
			 "RTKit: Message to undiscovered endpoint 0x%02x\n",
			 ep);

	if (ep >= APPLE_RTKIT_APP_ENDPOINT_START &&
	    rtk->ops->recv_message_early &&
	    rtk->ops->recv_message_early(rtk->cookie, ep, msg.msg0))
		return;

	work = kzalloc_obj(*work, GFP_ATOMIC);
	if (!work)
		return;

	work->rtk = rtk;
	work->ep = ep;
	work->msg = msg.msg0;
	INIT_WORK(&work->work, apple_rtkit_rx_work);
	queue_work(rtk->wq, &work->work);
}

int apple_rtkit_send_message(struct apple_rtkit *rtk, u8 ep, u64 message,
			     struct completion *completion, bool atomic)
{
	struct apple_mbox_msg msg = {
		.msg0 = message,
		.msg1 = ep,
	};

	if (rtk->crashed) {
		dev_warn(rtk->dev,
			 "RTKit: Device is crashed, cannot send message\n");
		return -EINVAL;
	}

	if (ep >= APPLE_RTKIT_APP_ENDPOINT_START &&
	    !apple_rtkit_is_running(rtk)) {
		dev_warn(rtk->dev,
			 "RTKit: Endpoint 0x%02x is not running, cannot send message\n", ep);
		return -EINVAL;
	}

	/*
	 * The message will be sent with a MMIO write. We need the barrier
	 * here to ensure any previous writes to buffers are visible to the
	 * device before that MMIO write happens.
	 */
	dma_wmb();

	return apple_mbox_send(rtk->mbox, msg, atomic);
}
EXPORT_SYMBOL_GPL(apple_rtkit_send_message);

int apple_rtkit_poll(struct apple_rtkit *rtk)
{
	return apple_mbox_poll(rtk->mbox);
}
EXPORT_SYMBOL_GPL(apple_rtkit_poll);

bool apple_rtkit_has_endpoint(struct apple_rtkit *rtk, u8 ep)
{
	return test_bit(ep, rtk->endpoints);
}
EXPORT_SYMBOL_GPL(apple_rtkit_has_endpoint);

int apple_rtkit_start_ep(struct apple_rtkit *rtk, u8 endpoint)
{
	u64 msg;

	if (!test_bit(endpoint, rtk->endpoints))
		return -EINVAL;
	if (endpoint >= APPLE_RTKIT_APP_ENDPOINT_START &&
	    !apple_rtkit_is_running(rtk))
		return -EINVAL;

	msg = FIELD_PREP(APPLE_RTKIT_MGMT_STARTEP_EP, endpoint);
	msg |= APPLE_RTKIT_MGMT_STARTEP_FLAG;
	apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_STARTEP, msg);

	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_start_ep);

static void apple_rtkit_mark_running(struct apple_rtkit *rtk)
{
	static const u8 system_endpoints[] = {
		APPLE_RTKIT_EP_CRASHLOG,
		APPLE_RTKIT_EP_SYSLOG,
		APPLE_RTKIT_EP_DEBUG,
		APPLE_RTKIT_EP_IOREPORT,
		APPLE_RTKIT_EP_OSLOG,
		APPLE_RTKIT_EP_TRACEKIT,
	};
	int i;

	/*
	 * The previous owner already completed HELLO and acknowledged this
	 * EPMAP; the firmware will not repeat either for this session.
	 */
	for (i = 0; i < ARRAY_SIZE(system_endpoints); i++)
		set_bit(system_endpoints[i], rtk->endpoints);

	rtk->iop_power_state = APPLE_RTKIT_PWR_STATE_ON;
	rtk->ap_power_state = APPLE_RTKIT_PWR_STATE_ON;
	rtk->syslog_inherited = true;
	rtk->crashlog_inherited = true;
}

static void apple_rtkit_claim_rx(struct apple_rtkit *rtk)
{
	struct apple_mbox *mbox = rtk->mbox;
	unsigned long flags;

	spin_lock_irqsave(&mbox->rx_lock, flags);
	if (mbox->rx || mbox->cookie)
		dev_warn(rtk->dev,
			 "RTKit: mailbox receiver already claimed, taking it over\n");
	mbox->cookie = rtk;
	mbox->rx = apple_rtkit_rx;
	spin_unlock_irqrestore(&mbox->rx_lock, flags);
}

static void apple_rtkit_detach_rx(struct apple_rtkit *rtk)
{
	unsigned long flags;

	WRITE_ONCE(rtk->shutting_down, true);
	apple_mbox_stop(rtk->mbox);
	/*
	 * IRQ receive and explicit polling both hold rx_lock. Detach before
	 * draining the queue, but keep the cookie as an ownership reservation
	 * until all host/buffer teardown is complete.
	 */
	spin_lock_irqsave(&rtk->mbox->rx_lock, flags);
	if (rtk->mbox->cookie == rtk)
		rtk->mbox->rx = NULL;
	spin_unlock_irqrestore(&rtk->mbox->rx_lock, flags);
}

static void apple_rtkit_stop_rx(struct apple_rtkit *rtk)
{
	apple_rtkit_detach_rx(rtk);
	destroy_workqueue(rtk->wq);
}

static void apple_rtkit_release_rx(struct apple_rtkit *rtk)
{
	unsigned long flags;

	spin_lock_irqsave(&rtk->mbox->rx_lock, flags);
	if (rtk->mbox->cookie == rtk)
		rtk->mbox->cookie = NULL;
	spin_unlock_irqrestore(&rtk->mbox->rx_lock, flags);
}

static struct apple_rtkit *__apple_rtkit_init(struct device *dev, void *cookie,
					      const char *mbox_name, int mbox_idx,
					      const struct apple_rtkit_ops *ops,
					      bool adopted,
					      bool firmware_owned_crashlog)
{
	struct apple_rtkit *rtk;
	int ret;

	if (!ops) {
		dev_err(dev, "RTKit init: missing callbacks for mailbox index %d\n",
			mbox_idx);
		return ERR_PTR(-EINVAL);
	}

	rtk = kzalloc_obj(*rtk);
	if (!rtk)
		return ERR_PTR(-ENOMEM);

	rtk->dev = dev;
	rtk->cookie = cookie;
	rtk->ops = ops;
	rtk->firmware_owned_crashlog = firmware_owned_crashlog;
	rtk->t8152_pmp = of_device_is_compatible(dev->of_node, "apple,t8152-pmp-v2");
	rtk->adopted = adopted;

	init_completion(&rtk->epmap_completion);
	init_completion(&rtk->iop_pwr_ack_completion);
	init_completion(&rtk->ap_pwr_ack_completion);
	init_completion(&rtk->iop_idle_completion);

	bitmap_zero(rtk->endpoints, APPLE_RTKIT_MAX_ENDPOINTS);
	set_bit(APPLE_RTKIT_EP_MGMT, rtk->endpoints);

	if (mbox_name)
		rtk->mbox = apple_mbox_get_byname(dev, mbox_name);
	else
		rtk->mbox = apple_mbox_get(dev, mbox_idx);

	if (IS_ERR(rtk->mbox)) {
		ret = PTR_ERR(rtk->mbox);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "RTKit init: mailbox index %d lookup failed: %d\n",
				mbox_idx, ret);
		goto free_rtk;
	}

	rtk->wq = alloc_ordered_workqueue("rtkit-%s", WQ_HIGHPRI | WQ_MEM_RECLAIM,
					  dev_name(rtk->dev));
	if (!rtk->wq) {
		ret = -ENOMEM;
		goto free_rtk;
	}

	/* Inherited system traffic may arrive as soon as mailbox RX is armed. */
	if (adopted)
		apple_rtkit_mark_running(rtk);

	apple_rtkit_claim_rx(rtk);

	ret = apple_mbox_start(rtk->mbox);
	if (ret) {
		dev_err(dev, "RTKit init: mailbox index %d start failed: %d\n",
			mbox_idx, ret);
		goto stop_rx;
	}

	return rtk;

stop_rx:
	apple_rtkit_stop_rx(rtk);
	apple_rtkit_release_rx(rtk);
free_rtk:
	kfree(rtk);
	return ERR_PTR(ret);
}

struct apple_rtkit *apple_rtkit_init(struct device *dev, void *cookie,
					    const char *mbox_name, int mbox_idx,
					    const struct apple_rtkit_ops *ops)
{
	return __apple_rtkit_init(dev, cookie, mbox_name, mbox_idx, ops,
				  false, false);
}
EXPORT_SYMBOL_GPL(apple_rtkit_init);

struct apple_rtkit *apple_rtkit_init_g17p(struct device *dev, void *cookie,
					  const char *mbox_name, int mbox_idx,
					  const struct apple_rtkit_ops *ops)
{
	return __apple_rtkit_init(dev, cookie, mbox_name, mbox_idx, ops,
				  false, true);
}
EXPORT_SYMBOL_GPL(apple_rtkit_init_g17p);

struct apple_rtkit *apple_rtkit_init_adopted(struct device *dev, void *cookie,
					     const char *mbox_name, int mbox_idx,
					     const struct apple_rtkit_ops *ops)
{
	return __apple_rtkit_init(dev, cookie, mbox_name, mbox_idx, ops,
				  true, false);
}
EXPORT_SYMBOL_GPL(apple_rtkit_init_adopted);

static int apple_rtkit_wait_for_completion(struct completion *c)
{
	long t;

	t = wait_for_completion_interruptible_timeout(c,
						      msecs_to_jiffies(1000));
	if (t < 0)
		return t;
	else if (t == 0)
		return -ETIME;
	else
		return 0;
}

int apple_rtkit_reinit(struct apple_rtkit *rtk)
{
	unsigned long flags;
	int ret;

	/* make sure we don't handle any messages while reinitializing */
	apple_rtkit_detach_rx(rtk);
	flush_workqueue(rtk->wq);

	apple_rtkit_free_buffer(rtk, &rtk->ioreport_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->crashlog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->oslog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->syslog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->pmp_aux_buffer);

	kfree(rtk->syslog_msg_buffer);

	rtk->syslog_msg_buffer = NULL;
	rtk->syslog_n_entries = 0;
	rtk->syslog_msg_size = 0;
	rtk->syslog_inherited = false;
	rtk->crashlog_inherited = false;
	rtk->crashlog_endpoint_started = false;
	rtk->adopted = false;
	WRITE_ONCE(rtk->awaiting_iop_idle, false);

	bitmap_zero(rtk->endpoints, APPLE_RTKIT_MAX_ENDPOINTS);
	set_bit(APPLE_RTKIT_EP_MGMT, rtk->endpoints);

	reinit_completion(&rtk->epmap_completion);
	reinit_completion(&rtk->iop_pwr_ack_completion);
	reinit_completion(&rtk->ap_pwr_ack_completion);

	rtk->crashed = false;
	rtk->iop_power_state = APPLE_RTKIT_PWR_STATE_OFF;
	rtk->ap_power_state = APPLE_RTKIT_PWR_STATE_OFF;

	spin_lock_irqsave(&rtk->mbox->rx_lock, flags);
	WRITE_ONCE(rtk->shutting_down, false);
	rtk->mbox->rx = apple_rtkit_rx;
	spin_unlock_irqrestore(&rtk->mbox->rx_lock, flags);

	ret = apple_mbox_start(rtk->mbox);
	if (ret) {
		apple_rtkit_detach_rx(rtk);
		flush_workqueue(rtk->wq);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_reinit);

static int apple_rtkit_set_ap_power_state(struct apple_rtkit *rtk,
					  unsigned int state)
{
	u64 msg;
	int ret;

	reinit_completion(&rtk->ap_pwr_ack_completion);

	msg = FIELD_PREP(APPLE_RTKIT_MGMT_PWR_STATE, state);
	ret = apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_SET_AP_PWR_STATE,
					  msg);
	if (ret)
		return ret;

	ret = apple_rtkit_wait_for_completion(&rtk->ap_pwr_ack_completion);
	if (ret)
		return ret;

	if (rtk->ap_power_state != state)
		return -EINVAL;
	return 0;
}

static int apple_rtkit_set_iop_power_state(struct apple_rtkit *rtk,
					   unsigned int state)
{
	u64 msg;
	int ret;

	reinit_completion(&rtk->iop_pwr_ack_completion);

	msg = FIELD_PREP(APPLE_RTKIT_MGMT_PWR_STATE, state);
	ret = apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE,
					  msg);
	if (ret)
		return ret;

	ret = apple_rtkit_wait_for_completion(&rtk->iop_pwr_ack_completion);
	if (ret)
		return ret;

	if (rtk->iop_power_state != state)
		return -EINVAL;
	return 0;
}

static int apple_rtkit_boot_common(struct apple_rtkit *rtk, bool early_ap_power)
{
	int ret;

	if (apple_rtkit_is_running(rtk))
		return 0;
	if (rtk->crashed)
		return -EINVAL;

	dev_dbg(rtk->dev, "RTKit: waiting for boot to finish\n");
	ret = apple_rtkit_wait_for_completion(&rtk->epmap_completion);
	if (ret)
		return ret;
	if (rtk->boot_result)
		return rtk->boot_result;

	/* Some helpers wait for AP=ON before acknowledging their own ON state. */
	if (early_ap_power) {
		ret = apple_rtkit_set_ap_power_state(rtk, APPLE_RTKIT_PWR_STATE_ON);
		if (ret)
			return ret;
	}

	dev_dbg(rtk->dev, "RTKit: waiting for IOP power state ACK\n");
	ret = apple_rtkit_wait_for_completion(&rtk->iop_pwr_ack_completion);
	if (ret)
		return ret;

	return early_ap_power ? 0 :
		apple_rtkit_set_ap_power_state(rtk, APPLE_RTKIT_PWR_STATE_ON);
}

int apple_rtkit_boot(struct apple_rtkit *rtk)
{
	return apple_rtkit_boot_common(rtk, false);
}
EXPORT_SYMBOL_GPL(apple_rtkit_boot);

int apple_rtkit_adopt_running(struct apple_rtkit *rtk)
{
	if (rtk->crashed)
		return -EINVAL;

	apple_rtkit_mark_running(rtk);
	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_adopt_running);

int apple_rtkit_shutdown(struct apple_rtkit *rtk)
{
	int ret;

	/* if OFF is used here the co-processor will not wake up again */
	ret = apple_rtkit_set_ap_power_state(rtk,
					     APPLE_RTKIT_PWR_STATE_QUIESCED);
	if (ret)
		return ret;

	ret = apple_rtkit_set_iop_power_state(rtk, APPLE_RTKIT_PWR_STATE_SLEEP);
	if (ret)
		return ret;

	return apple_rtkit_reinit(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_shutdown);

int apple_rtkit_poweroff(struct apple_rtkit *rtk)
{
	int ret;

	ret = apple_rtkit_set_ap_power_state(rtk, APPLE_RTKIT_PWR_STATE_OFF);
	if (ret)
		return ret;

	ret = apple_rtkit_set_iop_power_state(rtk, APPLE_RTKIT_PWR_STATE_OFF);
	if (ret)
		return ret;

	return apple_rtkit_reinit(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_poweroff);

int apple_rtkit_idle(struct apple_rtkit *rtk)
{
	int ret;

	/* if OFF is used here the co-processor will not wake up again */
	ret = apple_rtkit_set_ap_power_state(rtk,
					     APPLE_RTKIT_PWR_STATE_IDLE);
	if (ret)
		return ret;

	ret = apple_rtkit_set_iop_power_state(rtk, APPLE_RTKIT_PWR_STATE_IDLE);
	if (ret)
		return ret;

	rtk->iop_power_state = APPLE_RTKIT_PWR_STATE_IDLE;
	rtk->ap_power_state = APPLE_RTKIT_PWR_STATE_IDLE;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_idle);

int apple_rtkit_quiesce(struct apple_rtkit *rtk)
{
	int ret;

	ret = apple_rtkit_set_ap_power_state(rtk,
					     APPLE_RTKIT_PWR_STATE_QUIESCED);
	if (ret)
		return ret;

	ret = apple_rtkit_set_iop_power_state(rtk,
					      APPLE_RTKIT_PWR_STATE_QUIESCED);
	if (ret)
		return ret;

	ret = apple_rtkit_reinit(rtk);
	if (ret)
		return ret;

	rtk->iop_power_state = APPLE_RTKIT_PWR_STATE_QUIESCED;
	rtk->ap_power_state = APPLE_RTKIT_PWR_STATE_QUIESCED;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_quiesce);

int apple_rtkit_poweroff_adopted(struct apple_rtkit *rtk)
{
	int ret;

	if (!rtk->adopted)
		return -EINVAL;

	/* H17P's OFF ACK is followed by a separate idle/DMA retirement fence. */
	reinit_completion(&rtk->iop_idle_completion);
	WRITE_ONCE(rtk->awaiting_iop_idle, true);
	ret = apple_rtkit_set_iop_power_state(rtk, APPLE_RTKIT_PWR_STATE_OFF);
	if (!ret)
		ret = apple_rtkit_wait_for_completion(&rtk->iop_idle_completion);
	WRITE_ONCE(rtk->awaiting_iop_idle, false);
	if (ret)
		return ret;

	return apple_rtkit_reinit(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_poweroff_adopted);

static int apple_rtkit_wake_common(struct apple_rtkit *rtk, bool early_ap_power)
{
	u64 msg;
	int ret;

	if (apple_rtkit_is_running(rtk))
		return -EINVAL;

	reinit_completion(&rtk->iop_pwr_ack_completion);

	/*
	 * Use open-coded apple_rtkit_set_iop_power_state since apple_rtkit_boot
	 * will wait for the completion anyway.
	 */
	msg = FIELD_PREP(APPLE_RTKIT_MGMT_PWR_STATE, APPLE_RTKIT_PWR_STATE_INIT);
	ret = apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE,
					  msg);
	if (ret)
		return ret;

	return apple_rtkit_boot_common(rtk, early_ap_power);
}

int apple_rtkit_wake(struct apple_rtkit *rtk)
{
	return apple_rtkit_wake_common(rtk, false);
}
EXPORT_SYMBOL_GPL(apple_rtkit_wake);

int apple_rtkit_wake_early_ap(struct apple_rtkit *rtk)
{
	return apple_rtkit_wake_common(rtk, true);
}
EXPORT_SYMBOL_GPL(apple_rtkit_wake_early_ap);

void apple_rtkit_free(struct apple_rtkit *rtk)
{
	apple_rtkit_stop_rx(rtk);

	apple_rtkit_free_buffer(rtk, &rtk->ioreport_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->crashlog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->oslog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->syslog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->pmp_aux_buffer);

	kfree(rtk->syslog_msg_buffer);
	apple_rtkit_release_rx(rtk);
	kfree(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_free);

void apple_rtkit_free_retaining_buffers(struct apple_rtkit *rtk)
{
	/* No callback or worker may retain the consumer's context after return. */
	apple_rtkit_stop_rx(rtk);

	/*
	 * A failed shutdown provides no guarantee that firmware stopped DMA.
	 * Retain shared buffers, including custom allocation contexts, rather
	 * than returning their storage to an allocator while it may be live.
	 */
	dev_warn(rtk->dev,
		 "RTKit: retaining the shared buffers of an unstopped co-processor (ioreport %zu, crashlog %zu, oslog %zu, syslog %zu bytes)\n",
		 rtk->ioreport_buffer.size, rtk->crashlog_buffer.size,
		 rtk->oslog_buffer.size, rtk->syslog_buffer.size);
	kfree(rtk->syslog_msg_buffer);
	apple_rtkit_release_rx(rtk);
	kfree(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_free_retaining_buffers);

static void apple_rtkit_free_wrapper(void *data)
{
	apple_rtkit_free(data);
}

struct apple_rtkit *devm_apple_rtkit_init(struct device *dev, void *cookie,
					  const char *mbox_name, int mbox_idx,
					  const struct apple_rtkit_ops *ops)
{
	struct apple_rtkit *rtk;
	int ret;

	rtk = apple_rtkit_init(dev, cookie, mbox_name, mbox_idx, ops);
	if (IS_ERR(rtk))
		return rtk;

	ret = devm_add_action_or_reset(dev, apple_rtkit_free_wrapper, rtk);
	if (ret)
		return ERR_PTR(ret);

	return rtk;
}
EXPORT_SYMBOL_GPL(devm_apple_rtkit_init);

struct apple_rtkit *devm_apple_rtkit_init_adopted(
	struct device *dev, void *cookie, const char *mbox_name, int mbox_idx,
	const struct apple_rtkit_ops *ops)
{
	struct apple_rtkit *rtk;
	int ret;

	rtk = apple_rtkit_init_adopted(dev, cookie, mbox_name, mbox_idx, ops);
	if (IS_ERR(rtk))
		return rtk;

	ret = devm_add_action_or_reset(dev, apple_rtkit_free_wrapper, rtk);
	if (ret)
		return ERR_PTR(ret);

	return rtk;
}
EXPORT_SYMBOL_GPL(devm_apple_rtkit_init_adopted);

void devm_apple_rtkit_free(struct device *dev, struct apple_rtkit *rtk)
{
	devm_release_action(dev, apple_rtkit_free_wrapper, rtk);
}
EXPORT_SYMBOL_GPL(devm_apple_rtkit_free);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple RTKit driver");
