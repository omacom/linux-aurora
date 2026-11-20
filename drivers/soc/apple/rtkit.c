// SPDX-License-Identifier: (GPL-2.0-only OR MIT) AND GPL-2.0-only
/*
 * Apple RTKit IPC library
 * Copyright (C) The Asahi Linux Contributors
 */

#include "rtkit-internal.h"
#include <linux/soc/apple/dcp-v27-sleep.h>
#include <linux/of.h>

#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/sizes.h>
#include <linux/soc/apple/dart.h>
#include <linux/soc/apple/sleep.h>
#include <linux/suspend.h>

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
	APPLE_RTKIT_EP_COREANALYTICS = 7,
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
	/* Sent by some co-processors once idle after an IOP IDLE request, argument 0. */
	APPLE_RTKIT_MGMT_IDLE_NOTIFY = 0xc,
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

#define APPLE_RTKIT_POLLED_STEP_US 10
#define APPLE_RTKIT_POLLED_LOOPS   20000

#define APPLE_RTKIT_CRASHLOG_CRASH 1

#define APPLE_RTKIT_BUFFER_REQUEST	1
#define APPLE_RTKIT_BUFFER_REQUEST_SIZE GENMASK_ULL(51, 44)
#define APPLE_RTKIT_BUFFER_REQUEST_IOVA GENMASK_ULL(43, 0)

#define APPLE_RTKIT_SYSLOG_TYPE GENMASK_ULL(59, 52)

/* unknown ioreport messages, must be ACKed or the co-processor will hang */
#define APPLE_RTKIT_IOREPORT_UNKNOWN1 0x8
#define APPLE_RTKIT_IOREPORT_UNKNOWN2 0xc

#define APPLE_RTKIT_SYSLOG_LOG 5

#define APPLE_RTKIT_SYSLOG_INIT	     8
#define APPLE_RTKIT_SYSLOG_N_ENTRIES GENMASK_ULL(7, 0)
#define APPLE_RTKIT_SYSLOG_MSG_SIZE  GENMASK_ULL(31, 24)

#define APPLE_RTKIT_OSLOG_TYPE GENMASK_ULL(63, 56)
#define APPLE_RTKIT_OSLOG_BUFFER_REQUEST 1
#define APPLE_RTKIT_OSLOG_LOG 2
#define APPLE_RTKIT_OSLOG_LOG_RESERVED GENMASK_ULL(55, 32)
#define APPLE_RTKIT_OSLOG_SIZE GENMASK_ULL(55, 36)
#define APPLE_RTKIT_OSLOG_IOVA GENMASK_ULL(35, 0)

#define APPLE_RTKIT_KDEBUG_TYPE GENMASK_ULL(55, 48)
#define APPLE_RTKIT_KDEBUG_DATA GENMASK_ULL(47, 0)

enum {
	APPLE_RTKIT_KDEBUG_GET_BUFFER = 1,
	APPLE_RTKIT_KDEBUG_SET_BUFFER0 = 1,
	APPLE_RTKIT_KDEBUG_SET_BUFFER1 = 2,
	APPLE_RTKIT_KDEBUG_PREALLOC_BUFFER0 = 2,
	APPLE_RTKIT_KDEBUG_PREALLOC_BUFFER1 = 3,
	APPLE_RTKIT_KDEBUG_START = 8,
};

#define APPLE_RTKIT_KDEBUG_ENTRY_SIZE 0x20
#define APPLE_RTKIT_KDEBUG_BUF_ALIGN  SZ_16K
#define APPLE_RTKIT_KDEBUG_BUF1_SIZE  SZ_8K
/* The firmware gives the entry count; cap what it can make the kernel pin. */
#define APPLE_RTKIT_KDEBUG_BUF0_MAX   SZ_4M

#define APPLE_RTKIT_CA_TYPE GENMASK_ULL(55, 54)
#define APPLE_RTKIT_CA_UNIT_1M BIT_ULL(53)
#define APPLE_RTKIT_CA_UNIT_4K BIT_ULL(52)
#define APPLE_RTKIT_CA_COUNT GENMASK_ULL(51, 44)
#define APPLE_RTKIT_CA_IOVA GENMASK_ULL(43, 0)
#define APPLE_RTKIT_CA_EVENT_ID GENMASK_ULL(47, 32)
#define APPLE_RTKIT_CA_BUFFER_REQUEST 0
#define APPLE_RTKIT_CA_BUFFER_REPLY 1
#define APPLE_RTKIT_CA_EVENT 2
#define APPLE_RTKIT_CA_EVENT_ACK 3

#define APPLE_ASC_CPU_CONTROL_RUN BIT(4)

/* From the CPU control register. Both bits wait for the firmware's last message to be read. */
#define APPLE_ASC_CPU_STATUS 4
#define APPLE_ASC_CPU_STATUS_STOPPED BIT(1)
#define APPLE_ASC_CPU_STATUS_IDLE BIT(5)
#define APPLE_ASC_CPU_STATUS_DOWN (APPLE_ASC_CPU_STATUS_STOPPED | APPLE_ASC_CPU_STATUS_IDLE)

#define APPLE_RTKIT_MIN_SUPPORTED_VERSION 11
#define APPLE_RTKIT_MAX_SUPPORTED_VERSION 12

/* Default wait for each handshake reply, see apple_rtkit_set_handshake_timeout() */
#define APPLE_RTKIT_HANDSHAKE_TIMEOUT_MS 1000

struct apple_rtkit_rx_work {
	struct apple_rtkit *rtk;
	u8 ep;
	u64 msg;
	u64 received_ns;
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

static bool apple_rtkit_coproc_places_buffers(struct apple_rtkit *rtk)
{
	return rtk->ops->flags & APPLE_RTKIT_COPROC_PLACES_BUFFERS;
}

static bool apple_rtkit_needs_coreanalytics(struct apple_rtkit *rtk)
{
	return rtk->ops->flags & APPLE_RTKIT_NEEDS_COREANALYTICS;
}

static bool apple_rtkit_firmware_owns_crashlog(struct apple_rtkit *rtk)
{
	return rtk->ops->flags & APPLE_RTKIT_FIRMWARE_OWNED_CRASHLOG;
}

/* The crashlog endpoint is started while the endpoint map is handled. */
static bool apple_rtkit_early_crashlog(struct apple_rtkit *rtk)
{
	return apple_rtkit_coproc_places_buffers(rtk) ||
	       (rtk->ops->flags & APPLE_RTKIT_EARLY_CRASHLOG);
}

static int apple_rtkit_management_send(struct apple_rtkit *rtk, u8 type,
					u64 msg)
{
	int ret;

	msg &= ~APPLE_RTKIT_MGMT_TYPE;
	msg |= FIELD_PREP(APPLE_RTKIT_MGMT_TYPE, type);
	ret = apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_MGMT, msg, NULL, false);

	if (ret)
		dev_err(rtk->dev,
			"RTKit: Failed to send management message type 0x%02x: %d\n",
			type, ret);

	return ret;
}

static int apple_rtkit_ans_fail(struct apple_rtkit *rtk, int error)
{
	/* Sticky until a new reversible transaction; never a buffer lifetime fence. */
	if (!READ_ONCE(rtk->ans_error))
		WRITE_ONCE(rtk->ans_error, error);
	rtk->boot_result = error;
	complete_all(&rtk->epmap_completion);
	complete_all(&rtk->iop_pwr_ack_completion);
	complete_all(&rtk->ap_pwr_ack_completion);
	return error;
}

static void apple_rtkit_management_rx_hello(struct apple_rtkit *rtk, u64 msg)
{
	u64 reply;

	int min_ver = FIELD_GET(APPLE_RTKIT_MGMT_HELLO_MINVER, msg);
	int max_ver = FIELD_GET(APPLE_RTKIT_MGMT_HELLO_MAXVER, msg);
	int want_ver = min(APPLE_RTKIT_MAX_SUPPORTED_VERSION, max_ver);
	int ret;

	if (rtk->ans_wake) {
		if (rtk->ans_hello || min_ver != 12 || max_ver != 12) {
			apple_rtkit_ans_fail(rtk, -EPROTO);
			return;
		}
		rtk->ans_hello = true;
	}

	if (rtk->ops->protocol_version)
		want_ver = rtk->ops->protocol_version;
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
	if (want_ver < min_ver || want_ver > max_ver) {
		dev_err(rtk->dev, "RTKit: required protocol %d outside remote range %d..%d\n",
			want_ver, min_ver, max_ver);
		goto abort_boot;
	}

	dev_info(rtk->dev, "RTKit: Initializing (protocol version %d)\n",
		 want_ver);
	rtk->version = want_ver;

	reply = FIELD_PREP(APPLE_RTKIT_MGMT_HELLO_MINVER, want_ver);
	reply |= FIELD_PREP(APPLE_RTKIT_MGMT_HELLO_MAXVER, want_ver);
	ret = apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_HELLO_REPLY, reply);
	if (rtk->ans_wake && ret)
		apple_rtkit_ans_fail(rtk, ret);

	return;

abort_boot:
	rtk->boot_result = -EINVAL;
	complete_all(&rtk->epmap_completion);
}

/*
 * Endpoint map of a retained wake: the endpoints and their buffers are still
 * set up, so answer with the started endpoints of this part of the map, as
 * the native retained wake does, and start nothing. A started endpoint that
 * the firmware no longer offers fails the wake.
 */
static void apple_rtkit_management_rx_epmap_retained(struct apple_rtkit *rtk, u64 msg,
						     u32 base, unsigned long bitmap)
{
	unsigned long started = 0;
	u64 reply;
	int i;

	for (i = 0; i < 32 && 32 * base + i < APPLE_RTKIT_MAX_ENDPOINTS; i++)
		if (test_bit(32 * base + i, rtk->started))
			started |= BIT(i);
	if (started & ~bitmap) {
		dev_err(rtk->dev, "RTKit: retained endpoints %#lx missing from map %#lx (base %u)\n",
			started & ~bitmap, bitmap, base);
		rtk->boot_result = -EPROTO;
		complete_all(&rtk->epmap_completion);
		return;
	}
	/*
	 * Bit 0 of a reply is also MORE: a last part cannot echo endpoint
	 * 32 * base there. The DCP's map has two parts, and its endpoint 0
	 * (management) is in the first.
	 */
	if ((msg & APPLE_RTKIT_MGMT_EPMAP_LAST) && (started & APPLE_RTKIT_MGMT_EPMAP_REPLY_MORE)) {
		dev_err(rtk->dev, "RTKit: retained endpoint %u cannot be echoed in the last map part\n",
			32 * base);
		rtk->boot_result = -EPROTO;
		complete_all(&rtk->epmap_completion);
		return;
	}

	reply = FIELD_PREP(APPLE_RTKIT_MGMT_EPMAP_BASE, base) | started;
	if (msg & APPLE_RTKIT_MGMT_EPMAP_LAST)
		reply |= APPLE_RTKIT_MGMT_EPMAP_LAST;
	else
		reply |= APPLE_RTKIT_MGMT_EPMAP_REPLY_MORE;
	apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_EPMAP_REPLY, reply);

	if (msg & APPLE_RTKIT_MGMT_EPMAP_LAST) {
		rtk->boot_result = 0;
		complete_all(&rtk->epmap_completion);
	}
}

static void apple_rtkit_management_rx_epmap(struct apple_rtkit *rtk, u64 msg)
{
	int i, ep, ret;
	u64 reply;
	unsigned long bitmap = FIELD_GET(APPLE_RTKIT_MGMT_EPMAP_BITMAP, msg);
	u32 base = FIELD_GET(APPLE_RTKIT_MGMT_EPMAP_BASE, msg);

	/* ANS's wake: the endpoint map as before the sleep, no new endpoint or buffer negotiation. */
	if (rtk->ans_wake) {
		bool last = msg & APPLE_RTKIT_MGMT_EPMAP_LAST;

		if (READ_ONCE(rtk->ans_error) || !rtk->ans_hello ||
		    base != rtk->ans_map_next || base > 1 ||
		    bitmap != (base ? 0x3 : 0x51f) || last != !!base) {
			apple_rtkit_ans_fail(rtk, -EPROTO);
			return;
		}
		rtk->ans_map_next++;
	}

	dev_dbg(rtk->dev,
		"RTKit: received endpoint bitmap 0x%lx with base 0x%x\n",
		bitmap, base);

	for_each_set_bit(i, &bitmap, 32) {
		ep = 32 * base + i;
		dev_dbg(rtk->dev, "RTKit: Discovered endpoint 0x%02x\n", ep);
		set_bit(ep, rtk->endpoints);
	}

	/*
	 * Co-processors that place all buffers, and clients that explicitly
	 * request early crashlog startup, need the endpoint before the map is
	 * acknowledged: G17P can request its crash DVA right after.
	 */
	if (!rtk->warm_restart && apple_rtkit_early_crashlog(rtk) && base == 0 &&
	    test_bit(APPLE_RTKIT_EP_CRASHLOG, &bitmap)) {
		ret = apple_rtkit_start_ep(rtk, APPLE_RTKIT_EP_CRASHLOG);
		if (ret) {
			rtk->boot_result = ret;
			complete_all(&rtk->epmap_completion);
			return;
		}
	}

	/* macOS's secure side sends each part back unchanged. */
	if (rtk->port_only) {
		apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_EPMAP_REPLY, msg);
		if (msg & APPLE_RTKIT_MGMT_EPMAP_LAST) {
			rtk->boot_result = 0;
			complete_all(&rtk->epmap_completion);
		}
		return;
	}
	if (rtk->retained_wake) {
		apple_rtkit_management_rx_epmap_retained(rtk, msg, base, bitmap);
		return;
	}

	reply = FIELD_PREP(APPLE_RTKIT_MGMT_EPMAP_BASE, base);
	if (msg & APPLE_RTKIT_MGMT_EPMAP_LAST)
		reply |= APPLE_RTKIT_MGMT_EPMAP_LAST;
	else
		reply |= APPLE_RTKIT_MGMT_EPMAP_REPLY_MORE;

	/*
	 * macOS's reply to a restarted DCP echoes the map, and adds bit 1 to the
	 * first part: 0x51d becomes 0x51f. It starts no endpoints. Measured under
	 * aura, 3 of 3. Why bit 1 isn't known, but ep 1 is the crash log's.
	 */
	if (rtk->warm_restart) {
		if (msg & APPLE_RTKIT_MGMT_EPMAP_LAST)
			reply |= bitmap & rtk->warm_last_mask;
		else
			reply |= bitmap | BIT_ULL(1);
	}

	ret = apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_EPMAP_REPLY, reply);
	if (rtk->ans_wake && ret) {
		apple_rtkit_ans_fail(rtk, ret);
		return;
	}

	if (!(msg & APPLE_RTKIT_MGMT_EPMAP_LAST))
		return;

	if (rtk->warm_restart) {
		rtk->warm_restart = false;
		rtk->boot_result = 0;
		complete_all(&rtk->epmap_completion);
		return;
	}
	if (!test_bit(APPLE_RTKIT_EP_CRASHLOG, rtk->endpoints))
		dev_info(rtk->dev, "RTKit: crashlog endpoint not in the endpoint map\n");

	for_each_set_bit(ep, rtk->endpoints, APPLE_RTKIT_APP_ENDPOINT_START) {
		switch (ep) {
		/* the management endpoint is started by default */
		case APPLE_RTKIT_EP_MGMT:
			break;

		case APPLE_RTKIT_EP_COREANALYTICS:
			if (!rtk->pmp_registry_v2 && !apple_rtkit_needs_coreanalytics(rtk))
				break;
			fallthrough;

		/* without starting these RTKit refuses to boot */
		case APPLE_RTKIT_EP_SYSLOG:
		case APPLE_RTKIT_EP_CRASHLOG:
		case APPLE_RTKIT_EP_DEBUG:
		case APPLE_RTKIT_EP_IOREPORT:
		case APPLE_RTKIT_EP_OSLOG:
		case APPLE_RTKIT_EP_TRACEKIT:
			/* already started while handling the map */
			if (ep == APPLE_RTKIT_EP_CRASHLOG &&
			    apple_rtkit_early_crashlog(rtk))
				break;
			dev_dbg(rtk->dev,
				"RTKit: Starting system endpoint 0x%02x\n", ep);
			ret = apple_rtkit_start_ep(rtk, ep);
			/* #607: start kdebug, which then asks for its buffers. */
			if (!ret && rtk->sync_syslog_ack && ep == APPLE_RTKIT_EP_DEBUG)
				ret = apple_rtkit_send_message(rtk, ep,
							       FIELD_PREP(APPLE_RTKIT_KDEBUG_TYPE,
									  APPLE_RTKIT_KDEBUG_START),
							       NULL, false);
			if (ret) {
				rtk->boot_result = ret;
				complete_all(&rtk->epmap_completion);
				return;
			}
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

	if (rtk->ans_wake && (!rtk->ans_hello || rtk->ans_map_next != 2 ||
			     new_state != APPLE_RTKIT_PWR_STATE_ON)) {
		apple_rtkit_ans_fail(rtk, -EPROTO);
		return;
	}

	dev_dbg(rtk->dev, "RTKit: IOP power state transition: 0x%x -> 0x%x\n",
		rtk->iop_power_state, new_state);
	rtk->iop_power_state = new_state;

	/*
	 * A retained wake: a co-processor may report QUIESCED when it starts
	 * running and wait for an ON request before its HELLO. Ask once, and
	 * wait for ON.
	 */
	if (rtk->retained_wake && new_state != APPLE_RTKIT_PWR_STATE_ON) {
		if (new_state == APPLE_RTKIT_PWR_STATE_QUIESCED && !rtk->retained_on_sent) {
			rtk->retained_on_sent = true;
			apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE,
						    FIELD_PREP(APPLE_RTKIT_MGMT_PWR_STATE,
							       APPLE_RTKIT_PWR_STATE_ON));
			return;
		}
		dev_err(rtk->dev, "RTKit: unexpected IOP power state 0x%x on retained wake\n",
			new_state);
		/* Whichever wait the wake is in: the map, or IOP ON after it. */
		rtk->boot_result = -EPROTO;
		complete_all(&rtk->epmap_completion);
		complete_all(&rtk->iop_pwr_ack_completion);
		return;
	}

	complete_all(&rtk->iop_pwr_ack_completion);
}

static void apple_rtkit_management_rx_ap_pwr_ack(struct apple_rtkit *rtk,
						 u64 msg)
{
	unsigned int new_state = FIELD_GET(APPLE_RTKIT_MGMT_PWR_STATE, msg);

	/* The strict ANS transaction never accepts masked legacy encodings. */
	if (READ_ONCE(rtk->ans_wake) && new_state != APPLE_RTKIT_PWR_STATE_ON) {
		apple_rtkit_ans_fail(rtk, -EPROTO);
		return;
	}

	dev_dbg(rtk->dev, "RTKit: AP power state transition: 0x%x -> 0x%x\n",
		rtk->ap_power_state, new_state);
	rtk->ap_power_state = new_state;

	complete_all(&rtk->ap_pwr_ack_completion);
}

static void apple_rtkit_management_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_MGMT_TYPE, msg);

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
	case APPLE_RTKIT_MGMT_IDLE_NOTIFY:
		if (READ_ONCE(rtk->strict_idle_waiting) && !(msg & GENMASK_ULL(51, 0)) &&
		    rtk->iop_power_state == APPLE_RTKIT_PWR_STATE_IDLE &&
		    rtk->ap_power_state == APPLE_RTKIT_PWR_STATE_IDLE)
			complete(&rtk->strict_idle_done);
		dev_dbg(rtk->dev, "RTKit: co-processor idle: 0x%llx\n", msg);
		break;
	default:
		dev_warn(
			rtk->dev,
			"RTKit: unknown management message: 0x%llx (type: 0x%02x)\n",
			msg, type);
	}
}

static int apple_rtkit_alloc_shmem(struct apple_rtkit *rtk, u8 ep,
				   struct apple_rtkit_shmem *buffer)
{
	if (READ_ONCE(rtk->ans_hold))
		return apple_rtkit_ans_fail(rtk, -EBUSY);
	/* The crash log's buffer is asked for only at a crash: serve it after a strict restart too. */
	if (READ_ONCE(rtk->strict_retained_buffers) && ep != APPLE_RTKIT_EP_CRASHLOG)
		return -EPROTO;

	if (rtk->ops->shmem_setup_ep)
		return rtk->ops->shmem_setup_ep(rtk->cookie, ep, buffer);
	if (rtk->ops->shmem_setup)
		return rtk->ops->shmem_setup(rtk->cookie, buffer);

	buffer->buffer = dma_alloc_coherent(rtk->dev, buffer->size,
					    &buffer->iova, GFP_KERNEL);
	return buffer->buffer ? 0 : -ENOMEM;
}

static int apple_rtkit_common_rx_get_buffer(struct apple_rtkit *rtk,
					    struct apple_rtkit_shmem *buffer,
					    u8 ep, u64 msg)
{
	struct apple_rtkit_shmem request = {};
	u64 reply;
	int err;

	if (READ_ONCE(rtk->strict_retained_buffers) && !buffer->size && ep != APPLE_RTKIT_EP_CRASHLOG) {
		rtk->boot_result = -EPROTO;
		complete_all(&rtk->epmap_completion);
		complete_all(&rtk->iop_pwr_ack_completion);
		return -EPROTO;
	}

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
			if (READ_ONCE(rtk->ans_hold))
				return apple_rtkit_ans_fail(rtk, -EBUSY);
			return -EBUSY;
		}
		goto reply;
	}

	dev_dbg(rtk->dev, "RTKit: buffer request for 0x%zx bytes at %pad\n",
		request.size, &request.iova);

	if ((rtk->ops->flags & APPLE_RTKIT_FIRMWARE_OWNED_CRASHLOG) &&
	    ep == APPLE_RTKIT_EP_CRASHLOG && request.iova) {
		*buffer = request;
		buffer->is_mapped = true;
		return apple_rtkit_send_message(rtk, ep, msg, NULL, false);
	}

	if ((rtk->ops->flags & APPLE_RTKIT_FIRMWARE_OWNED_LOGS) && request.iova &&
	    (ep == APPLE_RTKIT_EP_SYSLOG || ep == APPLE_RTKIT_EP_OSLOG)) {
		*buffer = request;
		buffer->is_mapped = true;
		dev_dbg(rtk->dev, "RTKit: the firmware's own buffer on ep %#x: 0x%zx bytes at %pad\n",
			ep, request.size, &request.iova);
		return 0;
	}

	if (request.iova && !rtk->ops->shmem_setup && !rtk->ops->shmem_setup_ep) {
		err = -EINVAL;
		goto error;
	}

	err = apple_rtkit_alloc_shmem(rtk, ep, &request);
	if (err)
		goto error;
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
					   ep == APPLE_RTKIT_EP_PMP_AUX ?
						   5 :
						   APPLE_RTKIT_BUFFER_REQUEST);
			reply |= FIELD_PREP(APPLE_RTKIT_BUFFER_REQUEST_SIZE,
					    buffer->size >> 12);
			reply |= FIELD_PREP(APPLE_RTKIT_BUFFER_REQUEST_IOVA,
					    buffer->iova);
		}
		apple_rtkit_send_message(rtk, ep, reply, NULL, false);
	} else if (apple_rtkit_coproc_places_buffers(rtk) ||
		   (apple_rtkit_firmware_owns_crashlog(rtk) &&
		    ep == APPLE_RTKIT_EP_CRASHLOG)) {
		/* acknowledge the co-processor's own buffer with its request */
		apple_rtkit_send_message(rtk, ep, msg, NULL, false);
	}

	return 0;

error:
	dev_err(rtk->dev,
		"RTKit: failed buffer request ep=%#x for 0x%zx bytes at %pad msg=%#llx (%d)\n",
		ep, request.size, &request.iova, msg, err);
	return err;
}

static void apple_rtkit_free_buffer(struct apple_rtkit *rtk,
				    struct apple_rtkit_shmem *bfr)
{
	if (bfr->size == 0)
		return;

	if (bfr == &rtk->crashlog_buffer && rtk->crashlog_placed) {
		memunmap(bfr->buffer);
		rtk->crashlog_placed = false;
	} else if (rtk->ops->shmem_destroy) {
		rtk->ops->shmem_destroy(rtk->cookie, bfr);
	} else if (bfr->buffer) {
		dma_free_coherent(rtk->dev, bfr->size, bfr->buffer, bfr->iova);
	}

	bfr->buffer = NULL;
	bfr->iomem = NULL;
	bfr->iova = 0;
	bfr->size = 0;
	bfr->is_mapped = false;
	bfr->needs_dma_sync = false;
	bfr->private = NULL;
}

static int apple_rtkit_kdebug_alloc_buffer(struct apple_rtkit *rtk,
					   struct apple_rtkit_shmem *buffer,
					   size_t size)
{
	int err;

	/* Firmware may still be using buffers from an earlier request. */
	if (buffer->size)
		return buffer->size == size ? 0 : -EINVAL;

	buffer->size = size;
	buffer->iova = 0;
	buffer->buffer = NULL;
	buffer->iomem = NULL;
	buffer->is_mapped = false;

	err = apple_rtkit_alloc_shmem(rtk, APPLE_RTKIT_EP_DEBUG, buffer);
	if (err)
		buffer->size = 0;
	return err;
}

/*
 * kdebug buffer requests (AsahiLinux/linux#607), apple,sync-syslog-ack only.
 * A failed request is logged and left unanswered, as in #607; the firmware
 * then runs without kdebug. Buffers allocated so far stay until reinit or
 * free, and a repeated request of the same size reuses them.
 */
static void apple_rtkit_kdebug_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_KDEBUG_TYPE, msg);
	u64 count, reply;
	size_t size;
	int err;

	switch (type) {
	case APPLE_RTKIT_KDEBUG_GET_BUFFER:
		count = FIELD_GET(APPLE_RTKIT_KDEBUG_DATA, msg);
		if (!count ||
		    check_mul_overflow((size_t)count,
				       (size_t)APPLE_RTKIT_KDEBUG_ENTRY_SIZE,
				       &size) ||
		    size > SIZE_MAX - (APPLE_RTKIT_KDEBUG_BUF_ALIGN - 1)) {
			err = -EINVAL;
			goto error;
		}

		size = ALIGN(size, APPLE_RTKIT_KDEBUG_BUF_ALIGN);
		if (size > APPLE_RTKIT_KDEBUG_BUF0_MAX) {
			err = -E2BIG;
			goto error;
		}
		err = apple_rtkit_kdebug_alloc_buffer(rtk, &rtk->kdebug_buffer0,
						      size);
		if (err)
			goto error;

		err = apple_rtkit_kdebug_alloc_buffer(rtk, &rtk->kdebug_buffer1,
						      APPLE_RTKIT_KDEBUG_BUF1_SIZE);
		if (err)
			goto error;

		reply = FIELD_PREP(APPLE_RTKIT_KDEBUG_TYPE,
				   APPLE_RTKIT_KDEBUG_SET_BUFFER0) |
			FIELD_PREP(APPLE_RTKIT_KDEBUG_DATA,
				   rtk->kdebug_buffer0.iova);
		err = apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_DEBUG, reply,
					       NULL, false);
		if (err)
			goto error;

		reply = FIELD_PREP(APPLE_RTKIT_KDEBUG_TYPE,
				   APPLE_RTKIT_KDEBUG_SET_BUFFER1) |
			FIELD_PREP(APPLE_RTKIT_KDEBUG_DATA,
				   rtk->kdebug_buffer1.iova);
		err = apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_DEBUG, reply,
					       NULL, false);
		if (err)
			goto error;
		break;
	case APPLE_RTKIT_KDEBUG_PREALLOC_BUFFER0:
	case APPLE_RTKIT_KDEBUG_PREALLOC_BUFFER1:
		break;
	default:
		dev_warn_ratelimited(rtk->dev, "RTKit: Unknown kdebug message: %llx\n", msg);
	}
	return;

error:
	dev_err_ratelimited(rtk->dev, "RTKit: kdebug buffer request failed: %d\n", err);
}

static size_t apple_rtkit_coreanalytics_size(u64 msg)
{
	size_t count = FIELD_GET(APPLE_RTKIT_CA_COUNT, msg);

	if (msg & APPLE_RTKIT_CA_UNIT_4K)
		return count * 0x1000;
	if (msg & APPLE_RTKIT_CA_UNIT_1M)
		return count * 0x100000;
	return 0;
}

static int apple_rtkit_coreanalytics_reply(size_t size, dma_addr_t iova, u64 *reply)
{
	size_t unit = size < 0x100000 ? 0x1000 : 0x100000;
	size_t count;

	if (!size || !FIELD_FIT(APPLE_RTKIT_CA_IOVA, iova))
		return -EINVAL;
	count = (size - 1) / unit + 1;
	if (!FIELD_FIT(APPLE_RTKIT_CA_COUNT, count))
		return -EINVAL;

	*reply = FIELD_PREP(APPLE_RTKIT_CA_TYPE, APPLE_RTKIT_CA_BUFFER_REPLY) |
		 FIELD_PREP(APPLE_RTKIT_CA_COUNT, count) |
		 FIELD_PREP(APPLE_RTKIT_CA_IOVA, iova) |
		 (unit == 0x1000 ? APPLE_RTKIT_CA_UNIT_4K : APPLE_RTKIT_CA_UNIT_1M);
	return 0;
}

static int apple_rtkit_coreanalytics_get_buffer(struct apple_rtkit *rtk, u64 msg)
{
	struct apple_rtkit_shmem *buffer = &rtk->coreanalytics_buffer;
	size_t size = apple_rtkit_coreanalytics_size(msg);
	dma_addr_t iova = FIELD_GET(APPLE_RTKIT_CA_IOVA, msg);
	bool mapped = iova != 0;
	u64 reply;
	int ret;

	/* Never overwrite a live allocation on a duplicate request. */
	if (buffer->size)
		return -EBUSY;
	if (!size || (iova && !rtk->ops->shmem_setup && !rtk->ops->shmem_setup_ep))
		return -EINVAL;

	buffer->size = size;
	buffer->iova = iova;
	buffer->is_mapped = mapped;
	ret = apple_rtkit_alloc_shmem(rtk, APPLE_RTKIT_EP_COREANALYTICS, buffer);
	if (ret) {
		/* A failed setup must unwind its own partial allocation. */
		memset(buffer, 0, sizeof(*buffer));
		return ret;
	}

	if (buffer->size < size || buffer->is_mapped != mapped) {
		ret = -EINVAL;
		goto free_buffer;
	}
	if (buffer->iomem) {
		memset_io(buffer->iomem, 0, buffer->size);
	} else if (buffer->buffer) {
		memset(buffer->buffer, 0, buffer->size);
	} else {
		ret = -EINVAL;
		goto free_buffer;
	}

	if (!buffer->is_mapped) {
		ret = apple_rtkit_coreanalytics_reply(buffer->size, buffer->iova, &reply);
		if (ret)
			goto free_buffer;
		ret = apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_COREANALYTICS,
					       reply, NULL, false);
		/* Retain memory until teardown if delivery status is uncertain. */
		if (ret)
			return ret;
	}

	dev_dbg(rtk->dev, "RTKit: CoreAnalytics buffer ready (%zu bytes)\n", buffer->size);
	complete_all(&rtk->coreanalytics_completion);
	return 0;

free_buffer:
	apple_rtkit_free_buffer(rtk, buffer);
	return ret;
}

static void apple_rtkit_coreanalytics_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_CA_TYPE, msg);
	u64 reply;
	int ret;

	switch (type) {
	case APPLE_RTKIT_CA_BUFFER_REQUEST:
		ret = apple_rtkit_coreanalytics_get_buffer(rtk, msg);
		break;
	case APPLE_RTKIT_CA_EVENT:
		reply = FIELD_PREP(APPLE_RTKIT_CA_TYPE, APPLE_RTKIT_CA_EVENT_ACK) |
			(msg & APPLE_RTKIT_CA_EVENT_ID);
		ret = apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_COREANALYTICS,
					       reply, NULL, false);
		break;
	default:
		dev_warn(rtk->dev, "RTKit: Unknown CoreAnalytics message: %llx\n", msg);
		return;
	}
	if (ret)
		dev_err(rtk->dev, "RTKit: CoreAnalytics message %llx failed: %d\n", msg, ret);
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

/*
 * Where a DVA of the crash buffer lands: in a mapping Linux made, or else in one the
 * firmware kept from the bootloader in a root its DART inherited (the T8140 AOP's own
 * slot). 0 when neither translates it.
 */
static phys_addr_t apple_rtkit_crashlog_phys(struct apple_rtkit *rtk,
					     struct iommu_domain *domain, dma_addr_t iova)
{
	phys_addr_t phys = iommu_iova_to_phys(domain, iova);

	if (!phys && IS_REACHABLE(CONFIG_APPLE_DART) &&
	    apple_dart_fw_remote(rtk->dev, iova, false, &phys))
		phys = 0;

	return phys;
}

/*
 * The buffer a crashing firmware placed at its own address, mapped if its IOMMU
 * translates every page of it to one contiguous range, and acknowledged as a placed
 * buffer is. The firmware doesn't run again until a reboot.
 */
static int apple_rtkit_crashlog_map(struct apple_rtkit *rtk, u64 msg)
{
	struct apple_rtkit_shmem *bfr = &rtk->crashlog_buffer;
	struct iommu_domain *domain = iommu_get_domain_for_dev(rtk->dev);
	dma_addr_t iova = FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_IOVA, msg);
	size_t size = FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_SIZE, msg) << 12;
	phys_addr_t phys, page;
	size_t off;
	void *va;

	if (!domain || !iova || !size)
		return -ENODEV;
	phys = apple_rtkit_crashlog_phys(rtk, domain, iova);
	for (off = 0; phys && off < size; off += PAGE_SIZE) {
		page = apple_rtkit_crashlog_phys(rtk, domain, iova + off);
		if (page != phys + off) {
			dev_err(rtk->dev, "RTKit: crash log at %pad isn't contiguous at +%#zx\n",
				&iova, off);
			return -EINVAL;
		}
	}
	if (!phys)
		return -ENOENT;
	va = memremap(phys, size, MEMREMAP_WB);
	if (!va)
		return -ENOMEM;
	bfr->buffer = va;
	bfr->size = size;
	bfr->iova = iova;
	bfr->is_mapped = true;
	rtk->crashlog_placed = true;

	return apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_CRASHLOG, msg, NULL, false);
}

#define APPLE_RTKIT_CRASHLOG_LATE_POLL_MS 250
#define APPLE_RTKIT_CRASHLOG_LATE_POLLS 20

/*
 * Whether the crashlog buffer holds a log: an ordinary 'CLHE' log at its
 * start, or a 'CLGH' container whose current record leads to a complete one.
 */
static bool apple_rtkit_crashlog_header_valid(struct apple_rtkit *rtk)
{
	struct apple_rtkit_crashlog_header header;
	size_t size = rtk->crashlog_buffer.size, offset;
	unsigned int depth;
	u32 id;

	if (size < sizeof(header))
		return false;
	apple_rtkit_memcpy(rtk, &header, &rtk->crashlog_buffer, 0,
			   sizeof(header));
	if (header.fourcc == APPLE_RTKIT_CRASHLOG_CONTAINER_FOURCC) {
		u8 *bfr __free(kfree) = kmalloc(size, GFP_KERNEL);

		if (!bfr)
			return false;
		apple_rtkit_memcpy(rtk, bfr, &rtk->crashlog_buffer, 0, size);
		return !apple_rtkit_crashlog_locate(bfr, size, &offset, &id,
						    &depth);
	}
	return header.fourcc == APPLE_RTKIT_CRASHLOG_HEADER_FOURCC &&
	       header.size >= sizeof(header) &&
	       header.size <= size;
}

static void apple_rtkit_crashlog_late(struct work_struct *work)
{
	struct apple_rtkit *rtk = container_of(to_delayed_work(work),
					       struct apple_rtkit,
					       crashlog_late_work);
	u8 *bfr __free(kfree) = NULL;

	if (!apple_rtkit_crashlog_header_valid(rtk)) {
		if (++rtk->crashlog_late_polls < APPLE_RTKIT_CRASHLOG_LATE_POLLS)
			queue_delayed_work(rtk->wq, &rtk->crashlog_late_work,
				msecs_to_jiffies(APPLE_RTKIT_CRASHLOG_LATE_POLL_MS));
		else
			dev_err(rtk->dev,
				"RTKit: crash reported, but no crashlog was written in %u ms\n",
				APPLE_RTKIT_CRASHLOG_LATE_POLLS *
					APPLE_RTKIT_CRASHLOG_LATE_POLL_MS);
		return;
	}

	/* The crash was reported and the client told; only dump the log. */
	dev_err(rtk->dev, "RTKit: co-processor crashed; crashlog written after %u ms\n",
		(rtk->crashlog_late_polls + 1) * APPLE_RTKIT_CRASHLOG_LATE_POLL_MS);
	bfr = kzalloc(rtk->crashlog_buffer.size, GFP_KERNEL);
	if (!bfr)
		return;
	apple_rtkit_memcpy(rtk, bfr, &rtk->crashlog_buffer, 0,
			   rtk->crashlog_buffer.size);
	apple_rtkit_crashlog_dump(rtk, bfr, rtk->crashlog_buffer.size);
}

static void apple_rtkit_crashlog_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_SYSLOG_TYPE, msg);
	u8 *bfr __free(kfree) = NULL;
	/*
	 * An undiscovered crashlog endpoint was never started, so the
	 * co-processor was never asked for a buffer and, as in an inherited
	 * session, a message there can only be a crash report. Firmware that
	 * leaves this endpoint out of its map still reports crashes on it,
	 * with a buffer of its own.
	 */
	bool reports_crash = rtk->crashlog_inherited ||
			     !test_bit(APPLE_RTKIT_EP_CRASHLOG, rtk->endpoints);
	bool await_log = false;

	if (type != APPLE_RTKIT_CRASHLOG_CRASH) {
		dev_warn(rtk->dev, "RTKit: Unknown crashlog message: %llx\n",
			 msg);
		return;
	}

	/*
	 * A firmware that never announced this endpoint asks for its buffer only when
	 * it crashes, so there's no handshake to take the request for. Report the
	 * crash now, as the log may never come. A client that sets such a buffer up
	 * itself takes the path below, which also reads a log written later.
	 */
	if (!rtk->crashlog_buffer.size && !test_bit(APPLE_RTKIT_EP_CRASHLOG, rtk->endpoints) &&
	    !rtk->ops->shmem_setup && !rtk->ops->shmem_setup_ep) {
		int ret = apple_rtkit_crashlog_map(rtk, msg);

		dev_err(rtk->dev, "RTKit: co-processor has crashed: crash log buffer at %#llx %s (%d)\n",
			FIELD_GET(APPLE_RTKIT_BUFFER_REQUEST_IOVA, msg),
			ret ? "not mapped" : "mapped, waiting for its log", ret);
		if (!rtk->crashed) {
			if (READ_ONCE(rtk->ans_hold))
				apple_rtkit_ans_fail(rtk, -EIO);
			rtk->crashed = true;
			apple_sleep_coproc_crashed(rtk->dev);
			if (rtk->ops->crashed)
				rtk->ops->crashed(rtk->cookie, NULL, 0);
		}
		/* The log follows the acknowledgment, maybe with no message after it. */
		if (!ret) {
			rtk->crashlog_late_polls = 0;
			queue_delayed_work(rtk->wq, &rtk->crashlog_late_work,
					   msecs_to_jiffies(APPLE_RTKIT_CRASHLOG_LATE_POLL_MS));
		}
		return;
	}

	if (!rtk->crashlog_buffer.size) {
		if (apple_rtkit_common_rx_get_buffer(rtk, &rtk->crashlog_buffer,
						     APPLE_RTKIT_EP_CRASHLOG, msg))
			goto no_buffer;
		/*
		 * In a session that went through the handshake, the first
		 * message on a started crashlog endpoint is the buffer request.
		 * Firmware may place that buffer in memory which still holds
		 * the log of an earlier crash, so only look for one when the
		 * message can only be a crash report.
		 */
		if (!reports_crash)
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
	/* The log is read here, not by the poll a placed buffer started. */
	cancel_delayed_work(&rtk->crashlog_late_work);

	/* A firmware-owned crash buffer has no CPU mapping to read the log from. */
	if (!rtk->crashlog_buffer.buffer && !rtk->crashlog_buffer.iomem) {
		if (apple_rtkit_firmware_owns_crashlog(rtk))
			dev_err(rtk->dev,
				"RTKit: firmware-owned crash log at DVA %pad (0x%zx bytes)\n",
				&rtk->crashlog_buffer.iova, rtk->crashlog_buffer.size);
		if (READ_ONCE(rtk->ans_hold))
			apple_rtkit_ans_fail(rtk, -EIO);
		rtk->crashed = true;
		apple_sleep_coproc_crashed(rtk->dev);
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

	/* Its driver heard of the crash when the buffer was asked for. */
	if (rtk->crashed)
		return;
	if (READ_ONCE(rtk->ans_hold))
		apple_rtkit_ans_fail(rtk, -EIO);
	rtk->crashed = true;
	apple_sleep_coproc_crashed(rtk->dev);
	if (rtk->ops->crashed)
		rtk->ops->crashed(rtk->cookie, bfr, bfr ? rtk->crashlog_buffer.size : 0);
	return;

no_buffer:
	/* Otherwise this was a buffer request that could not be served. */
	if (!reports_crash)
		return;
	dev_err(rtk->dev, "RTKit: co-processor crashed without a usable crashlog buffer\n");
	goto crashed;

no_log:
	/* The buffer is mapped, but holds no valid log (yet). */
	dev_err(rtk->dev, "RTKit: co-processor crashed without a readable log\n");
	/*
	 * A co-processor whose crashlog endpoint was never discovered or
	 * started (the external DCP on T6050) requests the buffer only when
	 * it crashes, and writes the log after receiving it. Read the log once
	 * it is there, rather than dropping it.
	 */
	await_log = !test_bit(APPLE_RTKIT_EP_CRASHLOG, rtk->endpoints);

crashed:
	if (READ_ONCE(rtk->ans_hold))
		apple_rtkit_ans_fail(rtk, -EIO);
	rtk->crashed = true;
	apple_sleep_coproc_crashed(rtk->dev);
	if (rtk->ops->crashed)
		rtk->ops->crashed(rtk->cookie, NULL, 0);

	if (await_log) {
		rtk->crashlog_late_polls = 0;
		queue_delayed_work(rtk->wq, &rtk->crashlog_late_work,
			msecs_to_jiffies(APPLE_RTKIT_CRASHLOG_LATE_POLL_MS));
	}
}

static void apple_rtkit_ioreport_rx(struct apple_rtkit *rtk, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_SYSLOG_TYPE, msg);

	switch (type) {
	case APPLE_RTKIT_BUFFER_REQUEST:
		apple_rtkit_common_rx_get_buffer(rtk, &rtk->ioreport_buffer,
						 APPLE_RTKIT_EP_IOREPORT, msg);
		break;
	case APPLE_RTKIT_IOREPORT_UNKNOWN1:
	case APPLE_RTKIT_IOREPORT_UNKNOWN2:
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
	/* The firmware's own buffer, not mapped here (APPLE_RTKIT_FIRMWARE_OWNED_LOGS). */
	if (rtk->syslog_buffer.is_mapped && !rtk->syslog_buffer.buffer &&
	    !rtk->syslog_buffer.iomem)
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
	if (rtk->ops->syslog_is_debug &&
	    rtk->ops->syslog_is_debug(rtk->cookie, log_context, rtk->syslog_msg_buffer))
		dev_dbg(rtk->dev, "RTKit: syslog message: %s: %s\n", log_context,
			rtk->syslog_msg_buffer);
	else
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
	int err;

	/*
	 * A firmware counts its log writes and holds the next counter until the last is
	 * echoed, and with one outstanding takes no AP power notice: a DCP after a warm
	 * restart, the MTP and the SMC after about eight sleeps. macOS echoes every one.
	 */
	if (type == APPLE_RTKIT_OSLOG_LOG && rtk->oslog_echo) {
		/* A polled idle that came first echoed it already. */
		if (cmpxchg(&rtk->oslog_owed, msg, 0) != msg)
			return;
		dev_dbg(rtk->dev, "RTKit: oslog counter %016llx, echoed\n", msg);
		err = apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_OSLOG, msg, NULL, false);
		if (err && IS_ENABLED(CONFIG_APPLE_SOC_SLEEP))
			WRITE_ONCE(rtk->v27_rx_failed, true);
		if (err)
			dev_err(rtk->dev, "RTKit: oslog echo failed: %d\n", err);
		return;
	}

	/* Preserve the tested Neo/M5 and synchronous-log no-ACK policy. */
	if (type == APPLE_RTKIT_OSLOG_LOG &&
	    (rtk->sync_syslog_ack || of_machine_is_compatible("apple,t8140") ||
	     of_machine_is_compatible("apple,t8142"))) {
		dev_dbg(rtk->dev, "RTKit: oslog notification: %016llx\n", msg);
		return;
	}

	switch (type) {
	case APPLE_RTKIT_OSLOG_BUFFER_REQUEST:
		apple_rtkit_common_rx_get_buffer(rtk, &rtk->oslog_buffer,
						 APPLE_RTKIT_EP_OSLOG, msg);
		break;
	case APPLE_RTKIT_OSLOG_LOG:
		/* Channel zero reports a cumulative 32-bit write counter. Even
		 * though we discard these logs, acknowledge that exact counter:
		 * AppleDCP waits for its log transport to drain before AP sleep.
		 * This is not a buffer offset and may wrap. Other channels do not
		 * have a buffer admitted by this endpoint implementation. An inherited
		 * channel zero needs no buffer mapping to acknowledge its counter.
		 */
		if ((!rtk->oslog_buffer.size && !rtk->oslog_inherited) ||
		    (msg & APPLE_RTKIT_OSLOG_LOG_RESERVED)) {
			dev_warn(rtk->dev, "RTKit: invalid oslog notification: %llx\n", msg);
			break;
		}
		err = apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_OSLOG,
					       msg, NULL, false);
		if (err)
			dev_err(rtk->dev, "RTKit: oslog acknowledgment failed: %d\n", err);
		break;
	case 3 ... 5:
		/*
		 * Undocumented notifications sent during normal operation. They
		 * expect no reply, and the co-processor keeps running without
		 * one.
		 */
		dev_dbg(rtk->dev, "RTKit: oslog notification: %016llx\n", msg);
		break;
	default:
		dev_warn(rtk->dev, "RTKit: Unknown oslog message: %llx\n",
			 msg);
	}
}

/* Registry-v2 firmware requests one 8 KiB auxiliary buffer at endpoint 7.
 * Identical requests reuse the session buffer; changed geometry is refused.
 * On the J714s the same endpoint also carries a CoreAnalytics event with every
 * field zero, some minutes after the PMP has started; it is acknowledged with
 * an event acknowledgement whose fields are zero too, as the CoreAnalytics
 * endpoint of other coprocessors acknowledges an event
 * (apple_rtkit_coreanalytics_rx()). Any other message is refused as before.
 */
static void apple_rtkit_pmp_aux_rx(struct apple_rtkit *rtk, u64 msg)
{
	if (rtk->pmp_ca_event_ack &&
	    msg == FIELD_PREP(APPLE_RTKIT_CA_TYPE, APPLE_RTKIT_CA_EVENT)) {
		int ret = apple_rtkit_send_message(rtk, APPLE_RTKIT_EP_PMP_AUX,
						   FIELD_PREP(APPLE_RTKIT_CA_TYPE,
							      APPLE_RTKIT_CA_EVENT_ACK),
						   NULL, false);

		if (ret)
			dev_err(rtk->dev, "RTKit: CoreAnalytics event on endpoint 7 not acknowledged: %d\n",
				ret);
		else
			dev_dbg(rtk->dev, "RTKit: CoreAnalytics event on endpoint 7 acknowledged\n");
		return;
	}
	if (!rtk->pmp_registry_v2 ||
	    msg != (FIELD_PREP(APPLE_RTKIT_SYSLOG_TYPE,
			       APPLE_RTKIT_BUFFER_REQUEST) |
		    FIELD_PREP(APPLE_RTKIT_BUFFER_REQUEST_SIZE, 2))) {
		dev_err(rtk->dev, "RTKit: refused auxiliary buffer request\n");
		return;
	}
	apple_rtkit_common_rx_get_buffer(rtk, &rtk->pmp_aux_buffer,
					 APPLE_RTKIT_EP_PMP_AUX, msg);
}

static void apple_rtkit_rx_dispatch(struct apple_rtkit *rtk, u8 ep, u64 msg,
				    bool retained, u64 received_ns)
{
	if (READ_ONCE(rtk->shutting_down)) {
		if (retained) {
			rtk->m5_saved.error = -ESHUTDOWN;
			dev_err(rtk->dev, "message retained at teardown: ep=%u msg=%#llx\n",
				ep, msg);
		}
		return;
	}

	switch (ep) {
	case APPLE_RTKIT_EP_MGMT:
		apple_rtkit_management_rx(rtk, msg);
		break;
	case APPLE_RTKIT_EP_CRASHLOG:
		apple_rtkit_crashlog_rx(rtk, msg);
		break;
	case APPLE_RTKIT_EP_SYSLOG:
		apple_rtkit_syslog_rx(rtk, msg);
		break;
	case APPLE_RTKIT_EP_IOREPORT:
		apple_rtkit_ioreport_rx(rtk, msg);
		break;
	case APPLE_RTKIT_EP_DEBUG:
		if (rtk->sync_syslog_ack)
			apple_rtkit_kdebug_rx(rtk, msg);
		else
			dev_warn(rtk->dev,
				 "RTKit: message to unknown endpoint %02x: %llx\n",
				 ep, msg);
		break;
	case APPLE_RTKIT_EP_COREANALYTICS:
		if (rtk->pmp_registry_v2)
			apple_rtkit_pmp_aux_rx(rtk, msg);
		else
			apple_rtkit_coreanalytics_rx(rtk, msg);
		break;
	case APPLE_RTKIT_EP_OSLOG:
		apple_rtkit_oslog_rx(rtk, msg);
		break;
	case APPLE_RTKIT_APP_ENDPOINT_START ... 0xff:
		/* Buffered sleep packets have no original receive timestamp. */
		if (received_ns && rtk->ops->recv_message_timed)
			rtk->ops->recv_message_timed(rtk->cookie, ep, msg, received_ns);
		else if (rtk->ops->recv_message)
			rtk->ops->recv_message(rtk->cookie, ep, msg);
		else if (rtk->ops->recv_message_timed)
			rtk->ops->recv_message_timed(rtk->cookie, ep, msg, 0);
		else
			dev_warn(
				rtk->dev,
				"Received unexpected message to EP%02d: %llx\n",
				ep, msg);
		break;
	default:
		dev_warn(rtk->dev,
			 "RTKit: message to unknown endpoint %02x: %llx\n",
			 ep, msg);
	}
}

/* Saved-lane tracking is immutable and selected by the SMC's sleep constructor.
 * This one admission/completion path also balances the optional GPU diagnostic counter.
 * Reserve before publication; acquire-zero after RX synchronization means every
 * accepted callback has returned, including shutdown-capable notifications.
 */
static void apple_rtkit_saved_queue(struct apple_rtkit *rtk,
				   struct workqueue_struct *wq, struct work_struct *work)
{
	if (rtk->m5_saved.tracked)
		atomic_inc(&rtk->m5_saved.jobs);
	if (IS_ENABLED(CONFIG_APPLE_SOC_SLEEP))
		atomic_inc(&rtk->v27_jobs);
	if (!queue_work(wq, work)) {
		if (rtk->m5_saved.tracked)
			atomic_dec_return_release(&rtk->m5_saved.jobs);
		if (IS_ENABLED(CONFIG_APPLE_SOC_SLEEP))
			atomic_dec_return_release(&rtk->v27_jobs);
	}
}

static void apple_rtkit_saved_job_done(struct apple_rtkit *rtk)
{
	if (rtk->m5_saved.tracked)
		atomic_dec_return_release(&rtk->m5_saved.jobs);
	if (IS_ENABLED(CONFIG_APPLE_SOC_SLEEP))
		atomic_dec_return_release(&rtk->v27_jobs);
}

static void apple_rtkit_rx_work(struct work_struct *work)
{
	struct apple_rtkit_rx_work *w = container_of(work, struct apple_rtkit_rx_work, work);

	apple_rtkit_rx_dispatch(w->rtk, w->ep, w->msg, false, w->received_ns);
	apple_rtkit_saved_job_done(w->rtk);
	kfree(w);
}

static void apple_rtkit_m5_saved_work(struct work_struct *work)
{
	struct apple_rtkit *rtk = container_of(work, struct apple_rtkit, m5_saved.work);
	struct apple_mbox_msg msg;

	while (kfifo_get(&rtk->m5_saved.deferred, &msg))
		apple_rtkit_rx_dispatch(rtk, msg.msg1, msg.msg0, true, 0);
	apple_rtkit_saved_job_done(rtk);
}

/* ACKs have no transaction token: never overlap requests or reuse a timed-out
 * request's completion. Only its exact target resolves pending ownership.
 */
static bool apple_rtkit_m5_saved_ap_ack(struct apple_rtkit *rtk,
				       struct apple_mbox_msg msg)
{
	unsigned int state = FIELD_GET(APPLE_RTKIT_MGMT_PWR_STATE, msg.msg0);

	if (msg.msg1 != APPLE_RTKIT_EP_MGMT ||
	    FIELD_GET(APPLE_RTKIT_MGMT_TYPE, msg.msg0) !=
		APPLE_RTKIT_MGMT_SET_AP_PWR_STATE_ACK)
		return false;
	if (!rtk->m5_saved.ap_pending || state != rtk->m5_saved.ap_target) {
		rtk->m5_saved.error = -EPROTO;
		return false;
	}
	rtk->ap_power_state = state;
	smp_store_release(&rtk->m5_saved.ap_pending, false);
	return true;
}

/* An OSLOG counter this firmware waits to see echoed. */
static bool apple_rtkit_oslog_owed_msg(struct apple_rtkit *rtk, u8 ep, u64 msg)
{
	return ep == APPLE_RTKIT_EP_OSLOG &&
	       FIELD_GET(APPLE_RTKIT_OSLOG_TYPE, msg) == APPLE_RTKIT_OSLOG_LOG && rtk->oslog_echo;
}

static bool apple_rtkit_notification_msg(u8 ep, u64 msg)
{
	u8 type = FIELD_GET(APPLE_RTKIT_SYSLOG_TYPE, msg);

	if (ep == APPLE_RTKIT_EP_SYSLOG)
		return type == APPLE_RTKIT_SYSLOG_LOG;
	if (ep == APPLE_RTKIT_EP_IOREPORT)
		return type == APPLE_RTKIT_IOREPORT_UNKNOWN1 ||
		       type == APPLE_RTKIT_IOREPORT_UNKNOWN2;
	return false;
}

static bool apple_rtkit_sync_ack_msg(struct apple_rtkit *rtk, u8 ep, u64 msg)
{
	return rtk->sync_syslog_ack && apple_rtkit_notification_msg(ep, msg);
}

static void apple_rtkit_sync_ack_work(struct work_struct *work)
{
	struct apple_rtkit *rtk =
		container_of(work, struct apple_rtkit, sync_ack_work);
	struct apple_mbox_msg msg;
	int ret;

	while (!READ_ONCE(rtk->shutting_down) &&
	       kfifo_get(&rtk->sync_ack_fifo, &msg)) {
		ret = apple_rtkit_send_message(rtk, msg.msg1, msg.msg0, NULL,
					       false);
		if (ret && IS_ENABLED(CONFIG_APPLE_SOC_SLEEP))
			WRITE_ONCE(rtk->v27_rx_failed, true);
		if (ret)
			dev_err_ratelimited(rtk->dev,
					    "RTKit: failed to acknowledge %llx on endpoint 0x%02x: %d\n",
					    msg.msg0, msg.msg1, ret);
	}
	apple_rtkit_saved_job_done(rtk);
}

static void apple_rtkit_rx(struct apple_mbox *mbox, struct apple_mbox_msg msg,
			   void *cookie)
{
	struct apple_rtkit *rtk = cookie;
	struct apple_rtkit_rx_work *work;
	u8 ep = msg.msg1;
	u64 received_ns = ktime_get_ns();

	if (READ_ONCE(rtk->shutting_down))
		return;

	/*
	 * The message was read from a MMIO FIFO and we have to make
	 * sure all reads from buffers sent with that message happen
	 * afterwards.
	 */
	dma_rmb();

	/* A late AP ACK belongs to the timed-out request, not to any
	 * generic completion. Mismatches retain pending and take normal dispatch
	 * for evidence, with a permanent lane error latched.
	 */
	if (smp_load_acquire(&rtk->m5_saved.ap_pending) &&
	    apple_rtkit_m5_saved_ap_ack(rtk, msg))
		return;

	/*
	 * AsahiLinux/linux#607: this firmware wants its syslog and ioreport
	 * notifications acknowledged right away, not behind other receive
	 * work on the ordered queue, which may be waiting on the firmware.
	 * The ack work sends them from process context, so it may overtake
	 * messages that arrived earlier; nothing else depends on these acks.
	 * The syslog records themselves are not printed, as in #607: the
	 * syslog message buffer belongs to the ordered receive work. Should
	 * the ring be full, the message takes the ordinary path, which
	 * acknowledges it as well.
	 */
	if (apple_rtkit_sync_ack_msg(rtk, ep, msg.msg0)) {
		if (kfifo_put(&rtk->sync_ack_fifo, msg)) {
			apple_rtkit_saved_queue(rtk, system_highpri_wq, &rtk->sync_ack_work);
			return;
		}
		dev_warn_ratelimited(rtk->dev,
				     "RTKit: acknowledgment ring full, deferring %llx\n",
				     msg.msg0);
	}

	/*
	 * Firmware that leaves the crashlog endpoint out of its map (the AOP)
	 * still reports a crash there, and apple_rtkit_crashlog_rx() says so.
	 * Warning about it here too reads as if the AP sent the message.
	 */
	if (!test_bit(ep, rtk->endpoints) && ep != APPLE_RTKIT_EP_CRASHLOG)
		dev_warn(rtk->dev,
			 "RTKit: Message from undiscovered endpoint 0x%02x\n",
			 ep);

	if (ep >= APPLE_RTKIT_APP_ENDPOINT_START &&
	    rtk->ops->recv_message_early &&
	    rtk->ops->recv_message_early(rtk->cookie, ep, msg.msg0))
		return;

	work = kzalloc_obj(*work, GFP_ATOMIC);
	if (!work) {
		if (IS_ENABLED(CONFIG_APPLE_SOC_SLEEP))
			WRITE_ONCE(rtk->v27_rx_failed, true);
		return;
	}

	if (apple_rtkit_oslog_owed_msg(rtk, ep, msg.msg0))
		WRITE_ONCE(rtk->oslog_owed, msg.msg0);
	work->rtk = rtk;
	work->ep = ep;
	work->msg = msg.msg0;
	work->received_ns = received_ns;
	INIT_WORK(&work->work, apple_rtkit_rx_work);
	apple_rtkit_saved_queue(rtk, rtk->wq, &work->work);
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

#define APPLE_RTKIT_M5_STEP_US	10
#define APPLE_RTKIT_M5_LOOPS	50000 /* 500 ms plus bounded FIFO accesses */

int apple_rtkit_m5_saved_begin(struct apple_rtkit *rtk, u8 endpoint,
				u64 (*stamp)(void))
{
	if (rtk->m5_saved.active || !stamp || endpoint < APPLE_RTKIT_APP_ENDPOINT_START)
		return -EBUSY;
	if (!rtk->m5_saved.tracked)
		return -EOPNOTSUPP;
	/* No workqueue flush under the consumer device lock. A queued power-off
	 * notifier may need that same lock. Refuse and let it run after unwind.
	 */
	apple_mbox_hold_rx(rtk->mbox);
	if (atomic_read_acquire(&rtk->m5_saved.jobs)) {
		apple_rtkit_resume_rx(rtk);
		return -EBUSY;
	}
	if (rtk->m5_saved.error || rtk->m5_saved.ap_pending ||
	    READ_ONCE(rtk->shutting_down) || !apple_rtkit_is_running(rtk) ||
	    !test_bit(endpoint, rtk->endpoints)) {
		apple_rtkit_resume_rx(rtk);
		return rtk->m5_saved.error ?: -EIO;
	}
	rtk->m5_saved.notification = false;
	rtk->m5_saved.endpoint = endpoint;
	rtk->m5_saved.stamp = stamp;
	rtk->m5_saved.active = true;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_m5_saved_begin);

void apple_rtkit_m5_saved_end(struct apple_rtkit *rtk)
{
	if (!rtk->m5_saved.active)
		return;
	rtk->m5_saved.active = false;
	/* Replay ahead of new IRQ RX on the same ordered queue. No allocation,
	 * no dropped accepted crash/report packet, no callback under SMC's lock.
	 */
	if (!kfifo_is_empty(&rtk->m5_saved.deferred))
		apple_rtkit_saved_queue(rtk, rtk->wq, &rtk->m5_saved.work);
	apple_rtkit_resume_rx(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_m5_saved_end);

bool apple_rtkit_m5_saved_notification_pending(struct apple_rtkit *rtk)
{
	return rtk->m5_saved.notification;
}
EXPORT_SYMBOL_GPL(apple_rtkit_m5_saved_notification_pending);

bool apple_rtkit_m5_saved_unresolved(struct apple_rtkit *rtk)
{
	return rtk->m5_saved.error || smp_load_acquire(&rtk->m5_saved.ap_pending) ||
		READ_ONCE(rtk->crashed) || READ_ONCE(rtk->shutting_down);
}
EXPORT_SYMBOL_GPL(apple_rtkit_m5_saved_unresolved);

int apple_rtkit_m5_saved_poll(struct apple_rtkit *rtk)
{
	struct apple_mbox_msg msg;
	int ret;

	if (!rtk->m5_saved.active)
		return -EINVAL;
	if (rtk->m5_saved.error)
		return rtk->m5_saved.error;
	if (READ_ONCE(rtk->crashed) || READ_ONCE(rtk->shutting_down))
		return -EIO;
	/* Reserve a slot BEFORE consuming: overflow leaves the packet in HW. */
	if (kfifo_is_full(&rtk->m5_saved.deferred))
		return rtk->m5_saved.error = -ENOBUFS;
	ret = apple_mbox_recv_nowait(rtk->mbox, &msg);
	if (ret <= 0)
		return ret;
	dma_rmb();
	if (apple_rtkit_m5_saved_ap_ack(rtk, msg))
		return 1;
	if (msg.msg1 == rtk->m5_saved.endpoint &&
	    rtk->ops->recv_message_early &&
	    rtk->ops->recv_message_early(rtk->cookie, msg.msg1, msg.msg0))
		return 1;
	/* SMC notifications remain ordered for the ordinary notifier worker.
	 * Every other packet is retained, and refuses this attempt. In
	 * particular no crash/report message is silently eaten by polling.
	 */
	kfifo_put(&rtk->m5_saved.deferred, msg);
	/*
	 * The firmware may log before it acknowledges: a log or report word waits for the ordinary
	 * handler like a notification, and refuses nothing.
	 */
	if (msg.msg1 == APPLE_RTKIT_EP_SYSLOG || msg.msg1 == APPLE_RTKIT_EP_OSLOG ||
	    msg.msg1 == APPLE_RTKIT_EP_IOREPORT)
		return 1;
	if (msg.msg1 != rtk->m5_saved.endpoint)
		rtk->m5_saved.error = -EPROTO;
	else
		rtk->m5_saved.notification = true;
	return rtk->m5_saved.error ?: 1;
}
EXPORT_SYMBOL_GPL(apple_rtkit_m5_saved_poll);

static int apple_rtkit_m5_saved_tx(struct apple_rtkit *rtk, struct apple_mbox_msg msg,
				 u64 *sent)
{
	int i, ret;

	if (!rtk->m5_saved.active)
		return -EINVAL;
	for (i = 0; i < APPLE_RTKIT_M5_LOOPS; i++) {
		if (rtk->m5_saved.error)
			return rtk->m5_saved.error;
		if (READ_ONCE(rtk->crashed) || READ_ONCE(rtk->shutting_down))
			return -EIO;
		dma_wmb();
		ret = apple_mbox_send_nowait(rtk->mbox, msg);
		if (!ret) {
			if (sent)
				*sent = rtk->m5_saved.stamp();
			return 0;
		}
		if (ret != -EAGAIN && ret != -EBUSY)
			return ret;
		/* Do not recurse into RX while a not-yet-posted AP/SMC ID is
		 * reserved. The FIFO-only attempt has an independent fixed cap.
		 */
		udelay(APPLE_RTKIT_M5_STEP_US);
	}
	return -ETIMEDOUT;
}

int apple_rtkit_m5_saved_send(struct apple_rtkit *rtk, u64 message)
{
	struct apple_mbox_msg msg = {
		.msg0 = message, .msg1 = rtk->m5_saved.endpoint,
	};

	if (!apple_rtkit_is_running(rtk) || rtk->m5_saved.ap_pending)
		return -EIO;
	return apple_rtkit_m5_saved_tx(rtk, msg, NULL);
}
EXPORT_SYMBOL_GPL(apple_rtkit_m5_saved_send);

static int apple_rtkit_m5_saved_wait_ap(struct apple_rtkit *rtk, u64 *ack)
{
	int i, ret;

	for (i = 0; i < APPLE_RTKIT_M5_LOOPS; i++) {
		ret = apple_rtkit_m5_saved_poll(rtk);
		if (ret < 0)
			return ret;
		if (!rtk->m5_saved.ap_pending) {
			if (ack)
				*ack = rtk->m5_saved.stamp();
			return 0;
		}
		udelay(APPLE_RTKIT_M5_STEP_US);
	}
	return -ETIMEDOUT;
}

int apple_rtkit_m5_saved_ap_power(struct apple_rtkit *rtk, bool on,
				u64 *sent, u64 *ack)
{
	unsigned int target = on ? APPLE_RTKIT_PWR_STATE_ON : APPLE_RTKIT_PWR_STATE_IDLE;
	struct apple_mbox_msg msg = {
		.msg0 = FIELD_PREP(APPLE_RTKIT_MGMT_TYPE, APPLE_RTKIT_MGMT_SET_AP_PWR_STATE) |
			FIELD_PREP(APPLE_RTKIT_MGMT_PWR_STATE, target),
		.msg1 = APPLE_RTKIT_EP_MGMT,
	};
	int ret;

	if (!rtk->m5_saved.active)
		return -EINVAL;
	/* A timeout is not cancellation. Reconcile the old target first. */
	if (rtk->m5_saved.ap_pending) {
		ret = apple_rtkit_m5_saved_wait_ap(rtk, NULL);
		if (ret)
			return ret;
	}
	rtk->m5_saved.ap_target = target;
	smp_store_release(&rtk->m5_saved.ap_pending, true);
	ret = apple_rtkit_m5_saved_tx(rtk, msg, sent);
	if (ret) {
		/* nowait TX guarantees a negative return means never posted. */
		smp_store_release(&rtk->m5_saved.ap_pending, false);
		return ret;
	}
	return apple_rtkit_m5_saved_wait_ap(rtk, ack);
}
EXPORT_SYMBOL_GPL(apple_rtkit_m5_saved_ap_power);

static int apple_rtkit_poll_handshake(struct apple_rtkit *rtk, unsigned int timeout_ms)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);
	int ret;

	do {
		ret = apple_rtkit_poll(rtk);
		if (ret < 0)
			return ret;
		if (try_wait_for_completion(&rtk->epmap_completion))
			return rtk->boot_result;
		usleep_range(50, 100);
	} while (time_before(jiffies, timeout));

	return try_wait_for_completion(&rtk->epmap_completion) ? rtk->boot_result : -ETIME;
}

static int apple_rtkit_port_power(struct apple_rtkit *rtk, unsigned int state,
				  unsigned int timeout_ms)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);
	int ret;

	reinit_completion(&rtk->ap_pwr_ack_completion);
	ret = apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_SET_AP_PWR_STATE,
					  FIELD_PREP(APPLE_RTKIT_MGMT_PWR_STATE, state));
	if (ret)
		return ret;

	do {
		ret = apple_rtkit_poll(rtk);
		if (ret < 0)
			return ret;
		if (try_wait_for_completion(&rtk->ap_pwr_ack_completion))
			return 0;
		usleep_range(50, 100);
	} while (time_before(jiffies, timeout));

	return -ETIME;
}

int apple_rtkit_port_map(struct apple_rtkit *rtk, unsigned int timeout_ms)
{
	rtk->port_only = true;

	return apple_rtkit_poll_handshake(rtk, timeout_ms);
}
EXPORT_SYMBOL_GPL(apple_rtkit_port_map);

int apple_rtkit_port_on(struct apple_rtkit *rtk, unsigned int timeout_ms)
{
	/* The co-processor starts its side of this port itself. */
	rtk->iop_power_state = APPLE_RTKIT_PWR_STATE_ON;

	return apple_rtkit_port_power(rtk, APPLE_RTKIT_PWR_STATE_INIT, timeout_ms);
}
EXPORT_SYMBOL_GPL(apple_rtkit_port_on);

int apple_rtkit_port_handshake(struct apple_rtkit *rtk, unsigned int timeout_ms)
{
	int ret;

	ret = apple_rtkit_port_map(rtk, timeout_ms);
	if (ret)
		return ret;

	return apple_rtkit_port_on(rtk, timeout_ms);
}
EXPORT_SYMBOL_GPL(apple_rtkit_port_handshake);

int apple_rtkit_port_idle(struct apple_rtkit *rtk, unsigned int timeout_ms)
{
	return apple_rtkit_port_power(rtk, APPLE_RTKIT_PWR_STATE_IDLE, timeout_ms);
}
EXPORT_SYMBOL_GPL(apple_rtkit_port_idle);

void apple_rtkit_hold_rx(struct apple_rtkit *rtk)
{
	apple_mbox_hold_rx(rtk->mbox);
	if (rtk->sync_syslog_ack)
		flush_work(&rtk->sync_ack_work);
}
EXPORT_SYMBOL_GPL(apple_rtkit_hold_rx);

void apple_rtkit_resume_rx(struct apple_rtkit *rtk)
{
	apple_mbox_release_rx(rtk->mbox);
}
EXPORT_SYMBOL_GPL(apple_rtkit_resume_rx);

int apple_rtkit_set_rx_util_min(struct apple_rtkit *rtk, int util_min)
{
	return apple_mbox_set_rx_util_min(rtk->mbox, util_min);
}
EXPORT_SYMBOL_GPL(apple_rtkit_set_rx_util_min);

void apple_rtkit_flush_rx(struct apple_rtkit *rtk)
{
	flush_workqueue(rtk->wq);
}
EXPORT_SYMBOL_GPL(apple_rtkit_flush_rx);

bool apple_rtkit_has_endpoint(struct apple_rtkit *rtk, u8 ep)
{
	return test_bit(ep, rtk->endpoints);
}
EXPORT_SYMBOL_GPL(apple_rtkit_has_endpoint);

bool apple_rtkit_endpoint_started(struct apple_rtkit *rtk, u8 ep)
{
	return test_bit(ep, rtk->started);
}
EXPORT_SYMBOL_GPL(apple_rtkit_endpoint_started);

bool apple_rtkit_v27_rx_drained(struct apple_rtkit *rtk)
{
	/* A point-in-time query; final use requires synchronized RX admission. */
	return IS_ENABLED(CONFIG_APPLE_SOC_SLEEP) &&
		!READ_ONCE(rtk->v27_rx_failed) && !atomic_read_acquire(&rtk->v27_jobs) &&
		kfifo_is_empty(&rtk->sync_ack_fifo) &&
		kfifo_is_empty(&rtk->m5_saved.deferred) && !READ_ONCE(rtk->oslog_owed);
}
EXPORT_SYMBOL_GPL(apple_rtkit_v27_rx_drained);

int apple_rtkit_start_ep(struct apple_rtkit *rtk, u8 endpoint)
{
	u64 msg;
	int ret;

	if (!test_bit(endpoint, rtk->endpoints))
		return -EINVAL;
	if (endpoint >= APPLE_RTKIT_APP_ENDPOINT_START &&
	    !apple_rtkit_is_running(rtk))
		return -EINVAL;

	msg = FIELD_PREP(APPLE_RTKIT_MGMT_STARTEP_EP, endpoint);
	msg |= APPLE_RTKIT_MGMT_STARTEP_FLAG;
	ret = apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_STARTEP, msg);
	/* A mailbox FIFO timeout returns before either message word is written. */
	if (ret == -ETIMEDOUT) {
		dev_warn(rtk->dev, "RTKit: Retrying unsent endpoint start 0x%02x\n",
			 endpoint);
		ret = apple_rtkit_management_send(rtk, APPLE_RTKIT_MGMT_STARTEP, msg);
	}
	if (!ret)
		set_bit(endpoint, rtk->started);
	return ret;
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

	/* apple_rtkit_wake() must not wait for an EPMAP that already happened. */
	rtk->boot_result = 0;
	complete_all(&rtk->epmap_completion);

	rtk->iop_power_state = APPLE_RTKIT_PWR_STATE_ON;
	rtk->ap_power_state = APPLE_RTKIT_PWR_STATE_ON;
	rtk->syslog_inherited = true;
	rtk->crashlog_inherited = true;
	rtk->oslog_inherited = true;
}

static int apple_rtkit_claim_rx(struct apple_rtkit *rtk)
{
	struct apple_mbox *mbox = rtk->mbox;
	unsigned long flags;

	spin_lock_irqsave(&mbox->rx_lock, flags);
	if (mbox->rx || mbox->cookie) {
		spin_unlock_irqrestore(&mbox->rx_lock, flags);
		return -EBUSY;
	}
	mbox->cookie = rtk;
	mbox->rx = apple_rtkit_rx;
	spin_unlock_irqrestore(&mbox->rx_lock, flags);

	return 0;
}

static void apple_rtkit_detach_rx(struct apple_rtkit *rtk)
{
	unsigned long flags;

	WRITE_ONCE(rtk->shutting_down, true);
	/* No acknowledgment may be sent once the mailbox stops. */
	if (cancel_work_sync(&rtk->sync_ack_work))
		apple_rtkit_saved_job_done(rtk);
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
	/* A receive that raced with the first cancel may have queued it again. */
	if (cancel_work_sync(&rtk->sync_ack_work))
		apple_rtkit_saved_job_done(rtk);
}

static void apple_rtkit_stop_rx(struct apple_rtkit *rtk)
{
	apple_rtkit_detach_rx(rtk);
	/* A crashlog message still being handled may queue the late read. */
	flush_workqueue(rtk->wq);
	cancel_delayed_work_sync(&rtk->crashlog_late_work);
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

static int apple_rtkit_inherit_buffers(
	struct apple_rtkit *rtk,
	const struct apple_rtkit_inherited_buffer *buffers, unsigned int count)
{
	struct apple_rtkit_shmem *dst;
	unsigned int i;

	if (count > 4 || (count && !buffers))
		return -EINVAL;

	for (i = 0; i < count; i++) {
		const struct apple_rtkit_inherited_buffer *b = &buffers[i];

		switch (b->endpoint) {
		case APPLE_RTKIT_EP_CRASHLOG:
			dst = &rtk->crashlog_buffer;
			break;
		case APPLE_RTKIT_EP_SYSLOG:
			dst = &rtk->syslog_buffer;
			break;
		case APPLE_RTKIT_EP_IOREPORT:
			dst = &rtk->ioreport_buffer;
			break;
		case APPLE_RTKIT_EP_OSLOG:
			dst = &rtk->oslog_buffer;
			break;
		default:
			return -EINVAL;
		}

		if (dst->size || !b->shmem.size ||
		    (!b->shmem.buffer && !b->shmem.iomem) ||
		    (b->shmem.buffer && b->shmem.iomem))
			return -EINVAL;
		if (b->endpoint == APPLE_RTKIT_EP_OSLOG) {
			if (b->shmem.iova & 0xfff ||
			    !FIELD_FIT(APPLE_RTKIT_OSLOG_IOVA, b->shmem.iova >> 12) ||
			    !FIELD_FIT(APPLE_RTKIT_OSLOG_SIZE, b->shmem.size))
				return -EINVAL;
		} else if (b->shmem.size & 0xfff ||
			   !FIELD_FIT(APPLE_RTKIT_BUFFER_REQUEST_IOVA, b->shmem.iova) ||
			   !FIELD_FIT(APPLE_RTKIT_BUFFER_REQUEST_SIZE, b->shmem.size >> 12)) {
			return -EINVAL;
		}
		*dst = b->shmem;
	}
	return 0;
}

static struct apple_rtkit *__apple_rtkit_init(struct device *dev, void *cookie,
					      const char *mbox_name, int mbox_idx,
					      const struct apple_rtkit_ops *ops,
					      bool adopted,
					      const struct apple_rtkit_inherited_buffer *buffers,
					      unsigned int count, bool track_saved)
{
	struct apple_rtkit *rtk;
	int ret;

	if (!ops)
		return ERR_PTR(-EINVAL);
	if (ops->protocol_version &&
	    (ops->protocol_version < APPLE_RTKIT_MIN_SUPPORTED_VERSION ||
	     ops->protocol_version > APPLE_RTKIT_MAX_SUPPORTED_VERSION))
		return ERR_PTR(-EINVAL);
	if (ops->shmem_setup && ops->shmem_setup_ep)
		return ERR_PTR(-EINVAL);

	rtk = kzalloc_obj(*rtk);
	if (!rtk)
		return ERR_PTR(-ENOMEM);

	rtk->dev = dev;
	rtk->cookie = cookie;
	rtk->ops = ops;
	rtk->handshake_timeout_ms = APPLE_RTKIT_HANDSHAKE_TIMEOUT_MS;
	rtk->sync_syslog_ack =
		of_property_read_bool(dev->of_node, "apple,sync-syslog-ack");
	/*
	 * As macOS does; a driver whose firmware shouldn't get them turns it off. T8142 too:
	 * its MTP left a counter unechoed after the first sleep's resume, then took no AP
	 * idle notice for the next one (J815, -ETIME at prepare) and asserted on the one
	 * after ("Invalid AP power").
	 */
	rtk->oslog_echo = of_machine_is_compatible("apple,t8140") ||
			  of_machine_is_compatible("apple,t8142");
	rtk->m5_saved.tracked = track_saved;
	atomic_set(&rtk->m5_saved.jobs, 0);
	atomic_set(&rtk->v27_jobs, 0);
	rtk->v27_rx_failed = false;
	INIT_KFIFO(rtk->m5_saved.deferred);
	INIT_WORK(&rtk->m5_saved.work, apple_rtkit_m5_saved_work);
	INIT_KFIFO(rtk->sync_ack_fifo);
	INIT_WORK(&rtk->sync_ack_work, apple_rtkit_sync_ack_work);

	ret = apple_rtkit_inherit_buffers(rtk, buffers, count);
	if (ret)
		goto free_rtk;
	rtk->pmp_registry_v2 =
		of_device_is_compatible(dev->of_node, "apple,t6050-pmp-v2");
	rtk->pmp_ca_event_ack = rtk->pmp_registry_v2 &&
				of_machine_is_compatible("apple,j714s");

	init_completion(&rtk->epmap_completion);
	init_completion(&rtk->coreanalytics_completion);
	init_completion(&rtk->iop_pwr_ack_completion);
	init_completion(&rtk->ap_pwr_ack_completion);
	init_completion(&rtk->strict_idle_done);
	INIT_DELAYED_WORK(&rtk->crashlog_late_work, apple_rtkit_crashlog_late);

	bitmap_zero(rtk->endpoints, APPLE_RTKIT_MAX_ENDPOINTS);
	set_bit(APPLE_RTKIT_EP_MGMT, rtk->endpoints);
	set_bit(APPLE_RTKIT_EP_MGMT, rtk->started);

	if (mbox_name)
		rtk->mbox = apple_mbox_get_byname(dev, mbox_name);
	else
		rtk->mbox = apple_mbox_get(dev, mbox_idx);

	if (IS_ERR(rtk->mbox)) {
		ret = PTR_ERR(rtk->mbox);
		goto free_rtk;
	}

	rtk->warm_last_mask = U32_MAX;

	rtk->wq = alloc_ordered_workqueue("rtkit-%s", WQ_HIGHPRI | WQ_MEM_RECLAIM,
					  dev_name(rtk->dev));
	if (!rtk->wq) {
		ret = -ENOMEM;
		goto free_rtk;
	}

	/* Inherited system traffic may arrive as soon as mailbox RX is armed. */
	if (adopted)
		apple_rtkit_mark_running(rtk);

	ret = apple_rtkit_claim_rx(rtk);
	if (ret)
		goto destroy_wq;

	ret = apple_mbox_start(rtk->mbox);
	if (ret)
		goto stop_rx;

	return rtk;

stop_rx:
	apple_rtkit_stop_rx(rtk);
	apple_rtkit_release_rx(rtk);
	goto free_rtk;
destroy_wq:
	destroy_workqueue(rtk->wq);
free_rtk:
	kfree(rtk);
	return ERR_PTR(ret);
}

struct apple_rtkit *apple_rtkit_init(struct device *dev, void *cookie,
					    const char *mbox_name, int mbox_idx,
					    const struct apple_rtkit_ops *ops)
{
	return __apple_rtkit_init(dev, cookie, mbox_name, mbox_idx, ops, false,
				  NULL, 0, false);
}
EXPORT_SYMBOL_GPL(apple_rtkit_init);

struct apple_rtkit *apple_rtkit_init_adopted(struct device *dev, void *cookie,
					     const char *mbox_name, int mbox_idx,
					     const struct apple_rtkit_ops *ops)
{
	return __apple_rtkit_init(dev, cookie, mbox_name, mbox_idx, ops, true,
				  NULL, 0, false);
}
EXPORT_SYMBOL_GPL(apple_rtkit_init_adopted);

static int apple_rtkit_wait_for_completion(struct apple_rtkit *rtk,
					   struct completion *c);

/*
 * A handshake given up halfway can leave a firmware started that nothing stops. The
 * freezer signals every user task as a sleep begins, so only a fatal signal ends the
 * wait, and during a system sleep not even that. Nor does a retained wake's: it has
 * already started the co-processor's CPU, and nothing would undo that.
 */
static long apple_rtkit_wait_timeout(struct apple_rtkit *rtk, struct completion *c,
				     unsigned long timeout)
{
	if (READ_ONCE(rtk->retained_wake) || pm_suspend_target_state == PM_SUSPEND_MEM)
		return wait_for_completion_timeout(c, timeout);

	return wait_for_completion_killable_timeout(c, timeout);
}

/*
 * apple,sync-syslog-ack (AsahiLinux/linux#607): the mailbox can leave
 * messages queued without another interrupt during bootstrap, so poll it
 * while waiting. Everyone else waits as before.
 */
static int apple_rtkit_wait(struct apple_rtkit *rtk, struct completion *c)
{
	unsigned long timeout;
	long t;
	int ret;

	if (!rtk->sync_syslog_ack && !rtk->poll_waits)
		return apple_rtkit_wait_for_completion(rtk, c);

	timeout = jiffies + msecs_to_jiffies(rtk->handshake_timeout_ms);
	do {
		ret = apple_rtkit_poll(rtk);
		if (ret < 0)
			return ret;

		t = apple_rtkit_wait_timeout(rtk, c, msecs_to_jiffies(10));
		if (t < 0)
			return t;
		if (t > 0)
			return 0;
	} while (time_before(jiffies, timeout));

	/* Completed after the last wait, before the deadline check. */
	return try_wait_for_completion(c) ? 0 : -ETIME;
}

struct apple_rtkit *apple_rtkit_init_adopted_with_buffers(
	struct device *dev, void *cookie, const char *mbox_name, int mbox_idx,
	const struct apple_rtkit_ops *ops,
	const struct apple_rtkit_inherited_buffer *buffers, unsigned int count)
{
	return __apple_rtkit_init(dev, cookie, mbox_name, mbox_idx, ops, true,
				  buffers, count, false);
}
EXPORT_SYMBOL_GPL(apple_rtkit_init_adopted_with_buffers);

static int apple_rtkit_wait_for_completion(struct apple_rtkit *rtk,
					   struct completion *c)
{
	long t;

	t = apple_rtkit_wait_timeout(rtk, c, msecs_to_jiffies(rtk->handshake_timeout_ms));
	if (t < 0)
		return t;
	else if (t == 0)
		return -ETIME;
	else
		return 0;
}

void apple_rtkit_set_handshake_timeout(struct apple_rtkit *rtk,
				       unsigned int timeout_ms)
{
	rtk->handshake_timeout_ms = timeout_ms ?: APPLE_RTKIT_HANDSHAKE_TIMEOUT_MS;
}
EXPORT_SYMBOL_GPL(apple_rtkit_set_handshake_timeout);

/*
 * Expect HELLO from a firmware being restarted warm: it keeps the buffers it was
 * given and its rings, so unlike apple_rtkit_reinit() nothing is freed, and
 * endpoints aren't started again. The caller then sets the run bit and calls
 * apple_rtkit_boot().
 */
int apple_rtkit_warm_restart(struct apple_rtkit *rtk)
{
	unsigned long flags;
	int ret;

	apple_rtkit_detach_rx(rtk);
	flush_workqueue(rtk->wq);

	reinit_completion(&rtk->epmap_completion);
	reinit_completion(&rtk->iop_pwr_ack_completion);
	reinit_completion(&rtk->ap_pwr_ack_completion);

	rtk->crashed = false;
	rtk->warm_restart = true;
	rtk->iop_power_state = APPLE_RTKIT_PWR_STATE_OFF;
	rtk->ap_power_state = APPLE_RTKIT_PWR_STATE_OFF;
	kfifo_reset(&rtk->sync_ack_fifo);

	spin_lock_irqsave(&rtk->mbox->rx_lock, flags);
	WRITE_ONCE(rtk->shutting_down, false);
	rtk->mbox->rx = apple_rtkit_rx;
	spin_unlock_irqrestore(&rtk->mbox->rx_lock, flags);

	ret = apple_mbox_start(rtk->mbox);
	if (ret) {
		apple_rtkit_detach_rx(rtk);
		flush_workqueue(rtk->wq);
	} else {
		apple_mbox_set_recv_enabled(rtk->mbox, true);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_warm_restart);

void apple_rtkit_set_warm_epmap_mask(struct apple_rtkit *rtk, u32 last_mask)
{
	rtk->warm_last_mask = last_mask;
}
EXPORT_SYMBOL_GPL(apple_rtkit_set_warm_epmap_mask);

int apple_rtkit_port_restart(struct apple_rtkit *rtk)
{
	int ret = apple_rtkit_warm_restart(rtk);

	rtk->warm_restart = false;
	rtk->port_only = true;

	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_port_restart);

/* Everything of a firmware session but the mailbox hardware. */
static void apple_rtkit_reset_session(struct apple_rtkit *rtk)
{
	unsigned long flags;

	/* make sure we don't handle any messages while reinitializing */
	apple_rtkit_detach_rx(rtk);
	flush_workqueue(rtk->wq);
	cancel_delayed_work_sync(&rtk->crashlog_late_work);

	apple_rtkit_free_buffer(rtk, &rtk->coreanalytics_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->ioreport_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->crashlog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->oslog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->pmp_aux_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->syslog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->kdebug_buffer0);
	apple_rtkit_free_buffer(rtk, &rtk->kdebug_buffer1);

	kfree(rtk->syslog_msg_buffer);

	rtk->syslog_msg_buffer = NULL;
	rtk->syslog_n_entries = 0;
	rtk->syslog_msg_size = 0;
	rtk->syslog_inherited = false;
	rtk->crashlog_inherited = false;
	rtk->oslog_inherited = false;

	bitmap_zero(rtk->endpoints, APPLE_RTKIT_MAX_ENDPOINTS);
	set_bit(APPLE_RTKIT_EP_MGMT, rtk->endpoints);
	bitmap_zero(rtk->started, APPLE_RTKIT_MAX_ENDPOINTS);
	set_bit(APPLE_RTKIT_EP_MGMT, rtk->started);

	reinit_completion(&rtk->epmap_completion);
	reinit_completion(&rtk->coreanalytics_completion);
	reinit_completion(&rtk->iop_pwr_ack_completion);
	reinit_completion(&rtk->ap_pwr_ack_completion);

	rtk->crashed = false;
	rtk->iop_power_state = APPLE_RTKIT_PWR_STATE_OFF;
	rtk->ap_power_state = APPLE_RTKIT_PWR_STATE_OFF;

	/* Acknowledgements still queued belong to the old firmware session. */
	kfifo_reset(&rtk->sync_ack_fifo);

	spin_lock_irqsave(&rtk->mbox->rx_lock, flags);
	WRITE_ONCE(rtk->shutting_down, false);
	rtk->mbox->rx = apple_rtkit_rx;
	spin_unlock_irqrestore(&rtk->mbox->rx_lock, flags);
}

int apple_rtkit_reinit(struct apple_rtkit *rtk)
{
	int ret;

	apple_rtkit_reset_session(rtk);
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

	ret = apple_rtkit_wait(rtk, &rtk->ap_pwr_ack_completion);
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

	ret = apple_rtkit_wait(rtk, &rtk->iop_pwr_ack_completion);
	if (ret)
		return ret;

	if (rtk->iop_power_state != state)
		return -EINVAL;
	return 0;
}

static int apple_rtkit_boot_common(struct apple_rtkit *rtk, bool early_ap_power,
				   void (*before_ap)(void *data), void *data)
{
	int ret;

	if (apple_rtkit_is_running(rtk))
		return 0;
	if (rtk->crashed)
		return -EINVAL;

	dev_dbg(rtk->dev, "RTKit: waiting for boot to finish\n");
	ret = apple_rtkit_wait(rtk, &rtk->epmap_completion);
	if (ret) {
		dev_err(rtk->dev, "RTKit: endpoint discovery failed: %d\n", ret);
		return ret;
	}
	if (rtk->boot_result)
		return rtk->boot_result;

	/* PMP waits for CoreAnalytics setup before acknowledging IOP power. */
	if (apple_rtkit_needs_coreanalytics(rtk) &&
	    test_bit(APPLE_RTKIT_EP_COREANALYTICS, rtk->endpoints)) {
		ret = apple_rtkit_wait_for_completion(rtk,
						      &rtk->coreanalytics_completion);
		if (ret) {
			dev_err(rtk->dev, "RTKit: CoreAnalytics readiness failed: %d\n", ret);
			return ret;
		}
	}

	/* Some helpers need AP readiness before acknowledging IOP power. */
	if (early_ap_power) {
		ret = apple_rtkit_set_ap_power_state(rtk, APPLE_RTKIT_PWR_STATE_ON);
		if (ret) {
			dev_err(rtk->dev, "RTKit: early AP readiness failed: %d\n", ret);
			return ret;
		}
	}

	dev_dbg(rtk->dev, "RTKit: waiting for IOP power state ACK\n");
	ret = apple_rtkit_wait(rtk, &rtk->iop_pwr_ack_completion);
	if (ret) {
		dev_err(rtk->dev, "RTKit: IOP power acknowledgment failed: %d\n", ret);
		return ret;
	}
	/* A retained wake fails on any IOP state but ON (still 0 otherwise). */
	if (rtk->boot_result)
		return rtk->boot_result;

	if (rtk->ans_wake && (READ_ONCE(rtk->ans_error) ||
			     rtk->iop_power_state != APPLE_RTKIT_PWR_STATE_ON))
		return apple_rtkit_ans_fail(rtk, -EPROTO);

	if (early_ap_power)
		return 0;

	if (before_ap)
		before_ap(data);

	return apple_rtkit_set_ap_power_state(rtk, APPLE_RTKIT_PWR_STATE_ON);
}

int apple_rtkit_boot(struct apple_rtkit *rtk)
{
	return apple_rtkit_boot_common(rtk, false, NULL, NULL);
}
EXPORT_SYMBOL_GPL(apple_rtkit_boot);

int apple_rtkit_boot_with(struct apple_rtkit *rtk, void (*before_ap)(void *data), void *data)
{
	return apple_rtkit_boot_common(rtk, false, before_ap, data);
}
EXPORT_SYMBOL_GPL(apple_rtkit_boot_with);

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

int apple_rtkit_idle_with(struct apple_rtkit *rtk, void (*after_ap)(void *data), void *data)
{
	int ret;

	/* if OFF is used here the co-processor will not wake up again */
	ret = apple_rtkit_set_ap_power_state(rtk,
					     APPLE_RTKIT_PWR_STATE_IDLE);
	if (ret) {
		dev_err(rtk->dev, "RTKit: AP idle not acknowledged: %d\n", ret);
		return ret;
	}

	if (after_ap)
		after_ap(data);

	ret = apple_rtkit_set_iop_power_state(rtk, APPLE_RTKIT_PWR_STATE_IDLE);
	if (ret) {
		dev_err(rtk->dev, "RTKit: IOP idle not acknowledged: %d\n", ret);
		return ret;
	}

	rtk->iop_power_state = APPLE_RTKIT_PWR_STATE_IDLE;
	rtk->ap_power_state = APPLE_RTKIT_PWR_STATE_IDLE;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_idle_with);

void apple_rtkit_set_recv_enabled(struct apple_rtkit *rtk, bool enabled)
{
	apple_mbox_set_recv_enabled(rtk->mbox, enabled);
}
EXPORT_SYMBOL_GPL(apple_rtkit_set_recv_enabled);

int apple_rtkit_idle(struct apple_rtkit *rtk)
{
	return apple_rtkit_idle_with(rtk, NULL, NULL);
}
EXPORT_SYMBOL_GPL(apple_rtkit_idle);

/* For the firmware 27 sleep: both state ACKs and the firmware's final IDLE_DONE.
 * A failed request has no wire generation; prohibit retry in this lifetime.
 */
static int apple_rtkit_strict_idle_state(struct apple_rtkit *rtk, bool ap)
{
	struct completion *done = ap ? &rtk->ap_pwr_ack_completion :
		&rtk->iop_pwr_ack_completion;
	int ret;

	reinit_completion(done);
	ret = apple_rtkit_management_send(rtk, ap ? APPLE_RTKIT_MGMT_SET_AP_PWR_STATE :
		APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE,
		FIELD_PREP(APPLE_RTKIT_MGMT_PWR_STATE, APPLE_RTKIT_PWR_STATE_IDLE));
	if (ret)
		return ret;
	if (!wait_for_completion_timeout(done, msecs_to_jiffies(250)))
		return -ETIMEDOUT;
	return (ap ? READ_ONCE(rtk->ap_power_state) : READ_ONCE(rtk->iop_power_state)) ==
		APPLE_RTKIT_PWR_STATE_IDLE ? 0 : -EPROTO;
}

int apple_rtkit_idle_strict(struct apple_rtkit *rtk)
{
	int ret;

	if (rtk->strict_idle_poisoned || rtk->crashed ||
	    rtk->iop_power_state != APPLE_RTKIT_PWR_STATE_ON ||
	    rtk->ap_power_state != APPLE_RTKIT_PWR_STATE_ON)
		return -EUCLEAN;
	reinit_completion(&rtk->strict_idle_done);
	WRITE_ONCE(rtk->strict_idle_waiting, true);
	ret = apple_rtkit_strict_idle_state(rtk, true);
	if (!ret)
		ret = apple_rtkit_strict_idle_state(rtk, false);
	if (!ret && !wait_for_completion_timeout(&rtk->strict_idle_done,
						 msecs_to_jiffies(250)))
		ret = -ETIMEDOUT;
	WRITE_ONCE(rtk->strict_idle_waiting, false);
	if (!ret && READ_ONCE(rtk->crashed))
		ret = -EIO;
	if (ret)
		rtk->strict_idle_poisoned = true;
	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_idle_strict);

int apple_rtkit_warm_restart_strict(struct apple_rtkit *rtk)
{
	if (rtk->strict_idle_poisoned || rtk->crashed ||
	    rtk->iop_power_state != APPLE_RTKIT_PWR_STATE_IDLE ||
	    rtk->ap_power_state != APPLE_RTKIT_PWR_STATE_IDLE) {
		dev_err(rtk->dev, "RTKit: no restart after the sleep: poisoned %d crashed %d IOP state %#x AP state %#x\n",
			rtk->strict_idle_poisoned, rtk->crashed,
			rtk->iop_power_state, rtk->ap_power_state);
		return -EUCLEAN;
	}
	WRITE_ONCE(rtk->strict_retained_buffers, true);
	return apple_rtkit_warm_restart(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_warm_restart_strict);

/*
 * The caller stopped this co-processor's CPU itself in the final step of a sleep that then
 * happened, and has just read its run bit as clear. The firmware boots from its reset vector
 * next, so the power states and the crash flag tracked from messages before that stop say
 * nothing about it.
 */
int apple_rtkit_warm_restart_stopped(struct apple_rtkit *rtk)
{
	dev_warn(rtk->dev, "RTKit: restart of a stopped CPU with stale state: poisoned %d crashed %d IOP state %#x AP state %#x\n",
		 rtk->strict_idle_poisoned, rtk->crashed,
		 rtk->iop_power_state, rtk->ap_power_state);
	rtk->strict_idle_poisoned = false;
	WRITE_ONCE(rtk->strict_retained_buffers, true);
	return apple_rtkit_warm_restart(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_warm_restart_stopped);

void apple_rtkit_mailbox_off(struct apple_rtkit *rtk)
{
	apple_mbox_set_recv_enabled(rtk->mbox, false);
	apple_rtkit_detach_rx(rtk);
	flush_workqueue(rtk->wq);
}
EXPORT_SYMBOL_GPL(apple_rtkit_mailbox_off);

int apple_rtkit_ap_power(struct apple_rtkit *rtk, bool on)
{
	return apple_rtkit_set_ap_power_state(rtk, on ? APPLE_RTKIT_PWR_STATE_ON :
							APPLE_RTKIT_PWR_STATE_IDLE);
}
EXPORT_SYMBOL_GPL(apple_rtkit_ap_power);

int apple_rtkit_ap_power_noirq(struct apple_rtkit *rtk, bool on)
{
	int ret;

	rtk->poll_waits = true;
	ret = apple_rtkit_ap_power(rtk, on);
	rtk->poll_waits = false;

	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_ap_power_noirq);

/* Power is back, the CPU is stopped, and poweroff() discarded the old session. */
int apple_rtkit_cold_restart(struct apple_rtkit *rtk, int (*start)(void *), void *ctx)
{
	int ret;

	/* A fresh boot: no warm or port-only restart may answer its endpoint map. */
	apple_rtkit_reset_session(rtk);
	rtk->warm_restart = false;
	rtk->port_only = false;
	rtk->warm_last_mask = U32_MAX;
	/* The mailbox registers may only be clocked once @start has run. */
	ret = start(ctx);
	if (!ret)
		ret = apple_mbox_start(rtk->mbox);
	if (ret) {
		apple_rtkit_detach_rx(rtk);
		flush_workqueue(rtk->wq);
		return ret;
	}
	apple_mbox_set_recv_enabled(rtk->mbox, true);
	return apple_rtkit_boot(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_cold_restart);

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

	return apple_rtkit_boot_common(rtk, early_ap_power, NULL, NULL);
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

static int apple_rtkit_sleep_idle(struct apple_rtkit *rtk, void __iomem *cpu_control)
{
	u32 status;
	int ret;

	ret = apple_rtkit_idle(rtk);
	if (ret)
		return ret;

	ret = readl_poll_timeout(cpu_control + APPLE_ASC_CPU_STATUS, status,
				 (status & APPLE_ASC_CPU_STATUS_DOWN) == APPLE_ASC_CPU_STATUS_DOWN,
				 100, 5000);
	/* Both idles took, so it doesn't run on: stop it as if the status had shown it. */
	if (ret)
		dev_warn(rtk->dev, "RTKit: idle, but not stopped: status %#x\n", status);

	return 0;
}

/* With interrupts off: room in the FIFO, then the message. */
static bool apple_rtkit_polled_send(struct apple_rtkit *rtk, struct apple_mbox_msg msg)
{
	int i;

	for (i = 0; !apple_mbox_send_ready(rtk->mbox); i++) {
		if (i == APPLE_RTKIT_POLLED_LOOPS)
			return false;
		udelay(APPLE_RTKIT_POLLED_STEP_US);
	}

	return !apple_mbox_send(rtk->mbox, msg, true);
}

/*
 * Only what an idle needs: the state acks, notifications answered as they come, and OSLOG
 * counters echoed, without which the firmware takes no AP power notice.
 */
static void apple_rtkit_polled_rx(struct apple_mbox *mbox, struct apple_mbox_msg msg, void *cookie)
{
	struct apple_rtkit *rtk = cookie;
	u8 ep = msg.msg1;

	rtk->polled_seen++;
	rtk->polled_last = msg;
	if (rtk->polled_mask && ep == rtk->polled_ep &&
	    (msg.msg0 & rtk->polled_mask) == rtk->polled_want)
		rtk->polled_matched = true;
	if (apple_rtkit_oslog_owed_msg(rtk, ep, msg.msg0)) {
		if (!apple_rtkit_polled_send(rtk, msg))
			dev_err(rtk->dev, "RTKit: oslog echo failed while idling\n");
		return;
	}
	if (ep == APPLE_RTKIT_EP_MGMT) {
		u8 type = FIELD_GET(APPLE_RTKIT_MGMT_TYPE, msg.msg0);
		unsigned int state = FIELD_GET(APPLE_RTKIT_MGMT_PWR_STATE, msg.msg0);

		if (type == APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE_ACK)
			rtk->iop_power_state = state;
		else if (type == APPLE_RTKIT_MGMT_SET_AP_PWR_STATE_ACK)
			rtk->ap_power_state = state;
	} else if (apple_rtkit_notification_msg(ep, msg.msg0) && apple_mbox_send_ready(mbox)) {
		apple_mbox_send(mbox, msg, true);
	}
}

static int apple_rtkit_polled_state(struct apple_rtkit *rtk, u8 type, unsigned int *state,
				    unsigned int *us)
{
	struct apple_mbox_msg msg = {
		.msg0 = FIELD_PREP(APPLE_RTKIT_MGMT_TYPE, type) |
			FIELD_PREP(APPLE_RTKIT_MGMT_PWR_STATE, APPLE_RTKIT_PWR_STATE_IDLE),
		.msg1 = APPLE_RTKIT_EP_MGMT,
	};
	int i;

	for (i = 0; !apple_mbox_send_ready(rtk->mbox); i++) {
		if (i == APPLE_RTKIT_POLLED_LOOPS)
			return -ETIMEDOUT;
		udelay(APPLE_RTKIT_POLLED_STEP_US);
	}
	if (READ_ONCE(rtk->ans_hold)) {
		int ret = apple_mbox_send(rtk->mbox, msg, true);

		if (ret)
			return ret;
	} else {
		apple_mbox_send(rtk->mbox, msg, true);
	}

	for (i = 0; i < APPLE_RTKIT_POLLED_LOOPS; i++) {
		apple_mbox_poll(rtk->mbox);
		if (READ_ONCE(*state) == APPLE_RTKIT_PWR_STATE_IDLE)
			return 0;
		udelay(APPLE_RTKIT_POLLED_STEP_US);
		*us += APPLE_RTKIT_POLLED_STEP_US;
	}

	return -ETIMEDOUT;
}

void apple_rtkit_polled_begin(struct apple_rtkit *rtk)
{
	unsigned long flags;
	u64 owed;

	spin_lock_irqsave(&rtk->mbox->rx_lock, flags);
	rtk->mbox->rx = apple_rtkit_polled_rx;
	spin_unlock_irqrestore(&rtk->mbox->rx_lock, flags);

	apple_mbox_poll(rtk->mbox);
	owed = xchg(&rtk->oslog_owed, 0);
	if (owed) {
		struct apple_mbox_msg echo = { .msg0 = owed, .msg1 = APPLE_RTKIT_EP_OSLOG };

		if (!apple_rtkit_polled_send(rtk, echo))
			dev_err(rtk->dev, "RTKit: oslog echo failed before the polled exchange\n");
	}
	rtk->polled_seen = 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_polled_begin);

void apple_rtkit_polled_end(struct apple_rtkit *rtk)
{
	unsigned long flags;

	rtk->polled_mask = 0;
	spin_lock_irqsave(&rtk->mbox->rx_lock, flags);
	rtk->mbox->rx = apple_rtkit_rx;
	spin_unlock_irqrestore(&rtk->mbox->rx_lock, flags);
}
EXPORT_SYMBOL_GPL(apple_rtkit_polled_end);

int apple_rtkit_polled_exchange(struct apple_rtkit *rtk, u8 ep, u64 msg, u64 mask, u64 want)
{
	struct apple_mbox_msg m = { .msg0 = msg, .msg1 = ep };
	int i;

	rtk->polled_ep = ep;
	rtk->polled_want = want;
	rtk->polled_matched = false;
	rtk->polled_mask = mask;
	if (!apple_rtkit_polled_send(rtk, m))
		return -EIO;
	for (i = 0; i < APPLE_RTKIT_POLLED_LOOPS; i++) {
		apple_mbox_poll(rtk->mbox);
		if (rtk->polled_matched)
			return 0;
		udelay(APPLE_RTKIT_POLLED_STEP_US);
	}

	return -ETIMEDOUT;
}
EXPORT_SYMBOL_GPL(apple_rtkit_polled_exchange);

int apple_rtkit_polled_ap_idle(struct apple_rtkit *rtk, unsigned int *us)
{
	*us = 0;

	return apple_rtkit_polled_state(rtk, APPLE_RTKIT_MGMT_SET_AP_PWR_STATE,
					&rtk->ap_power_state, us);
}
EXPORT_SYMBOL_GPL(apple_rtkit_polled_ap_idle);

int apple_rtkit_sleep_idle_polled(struct apple_rtkit *rtk, void __iomem *cpu_control,
				  unsigned int *us)
{
	unsigned long flags;
	u64 owed;
	u32 status;
	int ret;

	*us = 0;
	spin_lock_irqsave(&rtk->mbox->rx_lock, flags);
	rtk->mbox->rx = apple_rtkit_polled_rx;
	spin_unlock_irqrestore(&rtk->mbox->rx_lock, flags);

	/* What's already waiting is answered first, or the firmware may not go idle. */
	apple_mbox_poll(rtk->mbox);
	/* Then a counter left for the receive work, which can't run now. */
	owed = xchg(&rtk->oslog_owed, 0);
	if (owed) {
		struct apple_mbox_msg echo = { .msg0 = owed, .msg1 = APPLE_RTKIT_EP_OSLOG };

		if (!apple_rtkit_polled_send(rtk, echo))
			dev_err(rtk->dev, "RTKit: oslog echo failed before the idle\n");
	}

	rtk->polled_seen = 0;
	ret = apple_rtkit_polled_state(rtk, APPLE_RTKIT_MGMT_SET_AP_PWR_STATE,
				       &rtk->ap_power_state, us);
	if (ret)
		dev_err(rtk->dev, "RTKit: AP IDLE not echoed (%d, AP %u): %u received, last ep %#x %#llx\n",
			ret, rtk->ap_power_state, rtk->polled_seen, rtk->polled_last.msg1,
			rtk->polled_last.msg0);
	if (!ret) {
		ret = apple_rtkit_polled_state(rtk, APPLE_RTKIT_MGMT_SET_IOP_PWR_STATE,
					       &rtk->iop_power_state, us);
		if (ret)
			dev_err(rtk->dev, "RTKit: IOP IDLE not acked (%d, IOP %u): %u received, last ep %#x %#llx\n",
				ret, rtk->iop_power_state, rtk->polled_seen, rtk->polled_last.msg1,
				rtk->polled_last.msg0);
	}
	if (!ret && READ_ONCE(rtk->ans_hold)) {
		int i;

		/* The mailbox's idle status reads 0x6e before RUN is cleared.
		 * Its bits' meanings aren't known.
		 * This is never used to free DMA or claim reversible idle.
		 */
		for (i = 0; i < 500; i++) {
			ret = apple_mbox_ans_idle_status(rtk->mbox, &status);
			if (ret || status == 0x6e)
				break;
			udelay(10);
		}
		if (!ret && status != 0x6e)
			ret = -ETIMEDOUT;
	} else if (!ret && !apple_mbox_ans_idle_status(rtk->mbox, &status)) {
		int i;

		/*
		 * M5 ASC mailboxes: the idle state shows in the mailbox's +0x0 idle status, 0x6e, not
		 * in CPU_STATUS, which the PMP's does not report DOWN in.
		 */
		for (i = 0; i < 500 && status != 0x6e; i++) {
			udelay(10);
			apple_mbox_ans_idle_status(rtk->mbox, &status);
		}
		if (status != 0x6e) {
			dev_err(rtk->dev, "RTKit: idle status %#x after the idle pair\n", status);
			ret = -ETIMEDOUT;
		}
	} else if (!ret) {
		ret = readl_poll_timeout_atomic(cpu_control + APPLE_ASC_CPU_STATUS, status,
						(status & APPLE_ASC_CPU_STATUS_DOWN) ==
						APPLE_ASC_CPU_STATUS_DOWN,
						APPLE_RTKIT_POLLED_STEP_US, 5000);
	}

	spin_lock_irqsave(&rtk->mbox->rx_lock, flags);
	rtk->mbox->rx = apple_rtkit_rx;
	spin_unlock_irqrestore(&rtk->mbox->rx_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_sleep_idle_polled);

void apple_rtkit_mailbox_recv_off(struct apple_rtkit *rtk)
{
	apple_mbox_set_recv_enabled(rtk->mbox, false);
}
EXPORT_SYMBOL_GPL(apple_rtkit_mailbox_recv_off);

int apple_rtkit_sleep_stop(struct apple_rtkit *rtk, void __iomem *cpu_control)
{
	int ret;

	ret = apple_rtkit_sleep_idle(rtk, cpu_control);
	if (ret)
		return ret;

	apple_rtkit_mailbox_off(rtk);

	/* Twice, each time from what it reads back. */
	writel(readl(cpu_control) & ~APPLE_ASC_CPU_CONTROL_RUN, cpu_control);
	writel(readl(cpu_control) & ~APPLE_ASC_CPU_CONTROL_RUN, cpu_control);

	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_sleep_stop);

int apple_rtkit_sleep_return(struct apple_rtkit *rtk, void __iomem *cpu_control)
{
	int ret;

	/*
	 * No real sleep: a firmware that idled doesn't come back from that with its power kept,
	 * and stays down until the next sleep restarts it. One whose idle failed still runs.
	 */
	if (!pm_resume_via_firmware()) {
		if (!(readl(cpu_control) & APPLE_ASC_CPU_CONTROL_RUN) ||
		    (rtk->iop_power_state & 0xff) != APPLE_RTKIT_PWR_STATE_ON)
			return -EAGAIN;
		/* Its AP idle took and its IOP idle didn't: endpoints stay shut until AP on. */
		if ((rtk->ap_power_state & 0xff) != APPLE_RTKIT_PWR_STATE_ON)
			return apple_rtkit_ap_power(rtk, true);
		return 0;
	}

	ret = apple_rtkit_warm_restart(rtk);
	if (ret)
		return ret;

	writel(readl(cpu_control) | APPLE_ASC_CPU_CONTROL_RUN, cpu_control);

	return apple_rtkit_boot(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_sleep_return);

int apple_rtkit_power_cycled_return(struct apple_rtkit *rtk, void __iomem *cpu_control)
{
	int ret = apple_rtkit_warm_restart(rtk);

	if (ret)
		return ret;
	writel(readl(cpu_control) | APPLE_ASC_CPU_CONTROL_RUN, cpu_control);

	return apple_rtkit_boot(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_power_cycled_return);

void apple_rtkit_set_oslog_echo(struct apple_rtkit *rtk, bool echo)
{
	rtk->oslog_echo = echo;
	dev_dbg(rtk->dev, "RTKit: OSLOG counters %s\n", echo ? "echoed" : "dropped");
}
EXPORT_SYMBOL_GPL(apple_rtkit_set_oslog_echo);

int apple_rtkit_sleep_return_start(struct apple_rtkit *rtk, void __iomem *cpu_control)
{
	int ret;

	if (!pm_resume_via_firmware())
		return -EOPNOTSUPP;

	ret = apple_rtkit_warm_restart(rtk);
	if (ret)
		return ret;
	if (READ_ONCE(rtk->ans_hold) && READ_ONCE(rtk->ans_error))
		return READ_ONCE(rtk->ans_error);

	writel(readl(cpu_control) | APPLE_ASC_CPU_CONTROL_RUN, cpu_control);

	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_sleep_return_start);

int apple_rtkit_ans_check(struct apple_rtkit *rtk)
{
	if (!READ_ONCE(rtk->ans_hold) || READ_ONCE(rtk->ans_error) ||
	    READ_ONCE(rtk->shutting_down) || !apple_rtkit_is_running(rtk))
		return -EIO;
	if (READ_ONCE(rtk->ans_wake) &&
	    (READ_ONCE(rtk->iop_power_state) != APPLE_RTKIT_PWR_STATE_ON ||
	     READ_ONCE(rtk->ap_power_state) != APPLE_RTKIT_PWR_STATE_ON))
		return -EPROTO;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_ans_check);

int apple_rtkit_ans_prepare(struct apple_rtkit *rtk)
{
	if (READ_ONCE(rtk->ans_hold) || !apple_rtkit_is_running(rtk))
		return -EBUSY;
	WRITE_ONCE(rtk->ans_error, 0);
	WRITE_ONCE(rtk->ans_hold, true);
	/* Earlier RX work finishes under the hold; no buffer can be replaced. */
	flush_workqueue(rtk->wq);
	return apple_rtkit_ans_check(rtk);
}
EXPORT_SYMBOL_GPL(apple_rtkit_ans_prepare);

void apple_rtkit_ans_release(struct apple_rtkit *rtk)
{
	/* Caller proved either no final-stop or a fully live rebuilt controller. */
	rtk->ans_wake = false;
	WRITE_ONCE(rtk->ans_hold, false);
}
EXPORT_SYMBOL_GPL(apple_rtkit_ans_release);

int apple_rtkit_ans_start(struct apple_rtkit *rtk, void __iomem *cpu_control)
{
	if (!READ_ONCE(rtk->ans_hold) || READ_ONCE(rtk->ans_error))
		return -EIO;
	rtk->ans_wake = true;
	rtk->ans_hello = false;
	rtk->ans_map_next = 0;
	rtk->boot_result = -EINPROGRESS;
	apple_rtkit_set_warm_epmap_mask(rtk, BIT(0));
	/* Real-wake-only helper preserves buffers and enables RX before RUN. */
	return apple_rtkit_sleep_return_start(rtk, cpu_control);
}
EXPORT_SYMBOL_GPL(apple_rtkit_ans_start);

int apple_rtkit_ans_finish(struct apple_rtkit *rtk)
{
	int ret;

	if (!READ_ONCE(rtk->ans_wake))
		return -EINVAL;
	if (READ_ONCE(rtk->ans_error))
		return READ_ONCE(rtk->ans_error);
	/* Three bounded waits at the owner's configured timeout; never retry INIT. */
	ret = apple_rtkit_boot(rtk);
	if (ret)
		return ret;
	flush_workqueue(rtk->wq);
	if (READ_ONCE(rtk->ans_error) || !rtk->ans_hello || rtk->ans_map_next != 2 ||
	    READ_ONCE(rtk->iop_power_state) != APPLE_RTKIT_PWR_STATE_ON ||
	    READ_ONCE(rtk->ap_power_state) != APPLE_RTKIT_PWR_STATE_ON ||
	    !apple_rtkit_is_running(rtk))
		return -EPROTO;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_rtkit_ans_finish);

/*
 * Wake a co-processor whose CPU was stopped after apple_rtkit_idle(), with
 * its firmware, endpoints and shared buffers kept (and its power domain
 * possibly off meanwhile). The caller holds RX since the idle; this restores
 * the mailbox, resumes RX and calls @start, which starts the CPU. The
 * firmware then says HELLO (possibly after reporting QUIESCED, which gets an
 * ON request) and sends its endpoint map, answered with the started
 * endpoints; no INIT is sent and nothing is started or allocated again.
 * Then IOP ON is awaited and AP ON requested, as for any boot.
 */
int apple_rtkit_wake_retained(struct apple_rtkit *rtk, int (*start)(void *), void *ctx)
{
	int ret;

	if (apple_rtkit_is_running(rtk) || rtk->crashed || rtk->shutting_down ||
	    rtk->iop_power_state != APPLE_RTKIT_PWR_STATE_IDLE || !start)
		return -EINVAL;

	reinit_completion(&rtk->epmap_completion);
	reinit_completion(&rtk->iop_pwr_ack_completion);
	reinit_completion(&rtk->ap_pwr_ack_completion);
	rtk->boot_result = -EINPROGRESS;
	rtk->retained_on_sent = false;
	WRITE_ONCE(rtk->retained_wake, true);

	ret = apple_mbox_restore(rtk->mbox);
	if (ret)
		goto out;
	apple_rtkit_resume_rx(rtk);
	ret = start(ctx);
	if (ret)
		goto out;
	ret = apple_rtkit_boot_common(rtk, false, NULL, NULL);
out:
	WRITE_ONCE(rtk->retained_wake, false);
	if (ret)
		dev_err(rtk->dev, "RTKit: retained wake failed: %d\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_rtkit_wake_retained);

void apple_rtkit_free(struct apple_rtkit *rtk)
{
	apple_rtkit_stop_rx(rtk);

	apple_rtkit_free_buffer(rtk, &rtk->coreanalytics_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->ioreport_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->crashlog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->oslog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->pmp_aux_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->syslog_buffer);
	apple_rtkit_free_buffer(rtk, &rtk->kdebug_buffer0);
	apple_rtkit_free_buffer(rtk, &rtk->kdebug_buffer1);

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
	 * The firmware may still be using the shared buffers, so keep them,
	 * including custom allocation contexts, rather than returning their
	 * storage to an allocator. Callers use this on purpose for firmware
	 * that cannot be stopped, and report a failed shutdown themselves.
	 */
	dev_dbg(rtk->dev,
		"RTKit: retaining the shared buffers of a running co-processor (ioreport %zu, crashlog %zu, oslog %zu, syslog %zu bytes)\n",
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

struct apple_rtkit *devm_apple_rtkit_init_m5_saved(struct device *dev, void *cookie,
						 const char *mbox_name, int mbox_idx,
						 const struct apple_rtkit_ops *ops)
{
	struct apple_rtkit *rtk;
	int ret;

	rtk = __apple_rtkit_init(dev, cookie, mbox_name, mbox_idx, ops, false,
				NULL, 0, true);
	if (IS_ERR(rtk))
		return rtk;
	ret = devm_add_action_or_reset(dev, apple_rtkit_free_wrapper, rtk);
	if (ret)
		return ERR_PTR(ret);
	return rtk;
}
EXPORT_SYMBOL_GPL(devm_apple_rtkit_init_m5_saved);

void devm_apple_rtkit_free(struct device *dev, struct apple_rtkit *rtk)
{
	devm_release_action(dev, apple_rtkit_free_wrapper, rtk);
}
EXPORT_SYMBOL_GPL(devm_apple_rtkit_free);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple RTKit driver");

#include "rtkit-v27-late.inc"
