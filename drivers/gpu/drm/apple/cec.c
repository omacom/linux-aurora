// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple Silicon DCP HDMI-CEC adapter driver
 *
 * Drives the compact EPIC "dcpav-cec-interface-epic" service on RTKit endpoint
 * 0x2b (AV_AUDIO_ENDPOINT_DESKTOP) for built-in HDMI ports behind an on-board
 * DisplayPort-to-HDMI converter with CEC tunneling over AUX.
 */

#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#include <drm/drm_mode.h>
#include <media/cec.h>

#include "afk.h"
#include "cec.h"
#include "connector.h"
#include "dcp.h"
#include "dcp-internal.h"

#define DCP_CEC_RX_RING_SIZE	32

struct dcp_cec_rx_item {
	struct cec_msg msg;
	ktime_t ts;
};

struct dcp_cec {
	struct apple_dcp *dcp;
	struct cec_adapter *adap;
	struct apple_dcp_afkep *owned_avep;
	bool avep_cleanup_registered;

	/*
	 * Serializes Open and Close RPCs across lifecycle_work, adap_enable,
	 * and HPD-low close. Never take adap->lock inside open_lock.
	 */
	struct mutex open_lock;

	/* Leaf lock: protects service state, LA mask, flags, and queued TX. */
	spinlock_t lock;
	struct apple_epic_service *srv;
	u64 gen;
	u64 opened_gen;
	u64 open_attempted_gen;
	u64 close_gen;
	u64 terminate_gen;
	u32 terminate_chan;
	u16 la_mask;
	bool enabled;
	bool rx_enabled;
	bool stopping;
	bool terminate_pending;
	bool pa_invalidate_pending;
	bool hpd_event_pending;
	bool hpd_event_val;
	bool pa_event_pending;
	u16 pa_event_val;
	bool poll_pa_pending;

	u64 tx_seq;
	u64 tx_active_seq;
	unsigned long tx_started;
	bool tx_pending;
	struct cec_msg tx_msg;
	u8 tx_attempts;

	/* Leaf lock: protects received frames queued from ep->wq to rx_work. */
	spinlock_t rx_lock;
	struct dcp_cec_rx_item rx_ring[DCP_CEC_RX_RING_SIZE];
	unsigned int rx_head;
	unsigned int rx_tail;
	unsigned int rx_count;

	struct work_struct lifecycle_work;
	struct work_struct tx_work;
	struct work_struct rx_work;
#if IS_ENABLED(CONFIG_DRM_APPLE_KUNIT_TEST)
	int (*test_call)(struct apple_epic_service *srv, u32 cmd,
			 const void *req, void *resp, size_t body_len,
			 unsigned int timeout_ms);
#endif
};

static int dcp_cec_call(struct apple_epic_service *srv, u32 cmd,
			const void *req, void *resp, size_t body_len,
			unsigned int timeout_ms)
{
	struct apple_dcp *dcp;

	if (!srv || !srv->ep)
		return -ENODEV;

	dcp = srv->ep->dcp;
	if (!dcp || READ_ONCE(dcp->crashed) || READ_ONCE(dcp->quiescing) ||
	    READ_ONCE(dcp->sleep_v27_closed))
		return -ESHUTDOWN;

#if IS_ENABLED(CONFIG_DRM_APPLE_KUNIT_TEST)
	if (dcp->cec && dcp->cec->test_call)
		return dcp->cec->test_call(srv, cmd, req, resp, body_len,
					   timeout_ms);
#endif

	return afk_service_call_timeout_len(srv, DCP_CEC_GROUP_CALL, cmd,
					    req, body_len, 0, resp, body_len, 0,
					    timeout_ms, NULL);
}

static int dcp_cec_cmd_open(struct apple_epic_service *srv)
{
	u8 req[DCP_CEC_OPEN_LEN] = {};
	u8 resp[DCP_CEC_OPEN_LEN] = {};
	int ret;

	ret = dcp_cec_call(srv, DCP_CEC_CMD_OPEN, req, resp, sizeof(req),
			   MSEC_PER_SEC);
	if (ret)
		return ret;

	return dcp_cec_decode_open_resp(resp);
}

static int dcp_cec_cmd_close(struct apple_epic_service *srv)
{
	u8 req[DCP_CEC_CLOSE_LEN] = {};
	u8 resp[DCP_CEC_CLOSE_LEN] = {};
	int ret;

	ret = dcp_cec_call(srv, DCP_CEC_CMD_CLOSE, req, resp, sizeof(req),
			   MSEC_PER_SEC);
	if (ret == -ENODEV)
		return 0;
	if (ret)
		return ret;

	return dcp_cec_decode_status_resp(resp);
}

static int dcp_cec_cmd_set_la_mask(struct apple_epic_service *srv,
				   bool enabled, u16 la_mask)
{
	u8 req[DCP_CEC_SET_LA_MASK_LEN];
	u8 resp[DCP_CEC_SET_LA_MASK_LEN] = {};
	int ret;

	dcp_cec_build_la_mask_req(req, enabled, la_mask);
	ret = dcp_cec_call(srv, DCP_CEC_CMD_SET_LOGICAL_ADDR_MASK, req, resp,
			   sizeof(req), MSEC_PER_SEC);
	if (ret)
		return ret;

	return dcp_cec_decode_status_resp(resp);
}

static int dcp_cec_cmd_get_edid_attrs(struct apple_epic_service *srv,
				      u16 *phys_addr)
{
	u8 req[DCP_CEC_EDID_ATTRS_LEN] = {};
	u8 resp[DCP_CEC_EDID_ATTRS_LEN] = {};
	int ret;

	ret = dcp_cec_call(srv, DCP_CEC_CMD_GET_EDID_ATTRS, req, resp,
			   sizeof(req), MSEC_PER_SEC);
	if (ret)
		return ret;

	*phys_addr = dcp_cec_decode_edid_attrs(resp);
	return 0;
}

static u8 dcp_cec_cmd_send(struct apple_epic_service *srv,
			   const struct cec_msg *msg, u8 attempts)
{
	u8 req[DCP_CEC_SEND_LEN];
	u8 resp[DCP_CEC_SEND_LEN] = {};
	int ret;

	ret = dcp_cec_build_send_req(req, msg, attempts);
	if (ret)
		return CEC_TX_STATUS_ERROR | CEC_TX_STATUS_MAX_RETRIES;

	ret = dcp_cec_call(srv, DCP_CEC_CMD_SEND_CEC, req, resp, sizeof(req),
			   DCP_CEC_TX_TIMEOUT_MS);
	return dcp_cec_decode_send_resp(ret, resp);
}

static bool dcp_cec_gen_current(struct dcp_cec *cec,
				struct apple_epic_service *srv, u64 gen)
{
	guard(spinlock_irqsave)(&cec->lock);
	return !cec->stopping && cec->srv == srv && cec->gen == gen;
}

/*
 * Ensure @srv (at @gen) has received Open. Returns 1 when newly opened, 0 when
 * already open, -ECANCELED if closed or terminated while opening (closing the
 * newly opened client again), or negative errno on failure / stale generation.
 * Caller must NOT acquire adap->lock inside open_lock.
 */
static int dcp_cec_ensure_open(struct dcp_cec *cec,
			       struct apple_epic_service *srv, u64 gen)
{
	bool cancel_close = false;
	u64 cgen;
	int ret;

	guard(mutex)(&cec->open_lock);

	scoped_guard(spinlock_irqsave, &cec->lock) {
		if (cec->stopping || cec->srv != srv || cec->gen != gen)
			return -ESTALE;
		if (cec->opened_gen == gen)
			return 0;
		cgen = cec->close_gen;
	}

	ret = dcp_cec_cmd_open(srv);
	if (ret) {
		dev_warn(cec->dcp->dev, "CEC: failed to open EPIC service: %d\n",
			 ret);
		return ret;
	}

	scoped_guard(spinlock_irqsave, &cec->lock) {
		if (cec->srv == srv && cec->gen == gen &&
		    !cec->stopping && cec->close_gen == cgen) {
			cec->opened_gen = gen;
		} else {
			/* An instance replaced meanwhile took its open with it. */
			if (cec->srv == srv && cec->gen == gen)
				cancel_close = true;
			ret = -ECANCELED;
		}
	}

	if (cancel_close)
		dcp_cec_cmd_close(srv);

	return ret ?: 1;
}

static void dcp_cec_close_client(struct dcp_cec *cec, bool queue_invalidate)
{
	struct apple_epic_service *srv = NULL;
	bool wake = false;

	scoped_guard(spinlock_irqsave, &cec->lock) {
		cec->close_gen++;
		cec->tx_seq++;
		cec->tx_pending = false;
		if (cec->srv && cec->opened_gen == cec->gen) {
			srv = afk_service_get(cec->srv);
			cec->opened_gen = 0;
		}
		if (queue_invalidate && !cec->stopping) {
			cec->pa_event_pending = false;
			cec->poll_pa_pending = false;
			cec->pa_invalidate_pending = true;
			wake = true;
		}
	}

	if (srv) {
		scoped_guard(mutex, &cec->open_lock)
			dcp_cec_cmd_close(srv);
		afk_service_put(srv);
	}

	if (wake)
		queue_work(system_dfl_wq, &cec->lifecycle_work);
}

static void dcp_cec_complete_tx(struct cec_adapter *adap, u8 status)
{
	/*
	 * cec_transmit_attempt_done_ts() only accepts OK, ARB_LOST, NACK,
	 * LOW_DRIVE and ERROR; report TIMEOUT and ABORTED via cec_transmit_done.
	 */
	if (status & (CEC_TX_STATUS_TIMEOUT | CEC_TX_STATUS_ABORTED))
		cec_transmit_done(adap, status | CEC_TX_STATUS_MAX_RETRIES,
				  0, 0, 0, 0);
	else
		cec_transmit_attempt_done(adap, status);
}

static void dcp_cec_tx_work(struct work_struct *work)
{
	struct dcp_cec *cec = container_of(work, struct dcp_cec, tx_work);

	while (!READ_ONCE(cec->stopping)) {
		struct cec_adapter *adap = READ_ONCE(cec->adap);
		struct apple_epic_service *srv = NULL;
		struct cec_msg msg;
		unsigned long started = 0;
		bool is_open = false;
		bool complete = false;
		u8 attempts = 0;
		u64 seq = 0;
		u8 status;

		scoped_guard(spinlock_irqsave, &cec->lock) {
			if (cec->stopping || !cec->tx_pending || !cec->enabled) {
				cec->tx_pending = false;
				return;
			}
			msg = cec->tx_msg;
			attempts = cec->tx_attempts;
			seq = cec->tx_active_seq;
			started = cec->tx_started;
			cec->tx_pending = false;
			srv = afk_service_get(cec->srv);
			is_open = srv && cec->opened_gen == cec->gen;
		}

		if (!srv)
			status = CEC_TX_STATUS_ERROR | CEC_TX_STATUS_MAX_RETRIES;
		else if (!is_open)
			status = CEC_TX_STATUS_ABORTED;
		else
			status = dcp_cec_cmd_send(srv, &msg, attempts);

		if (srv)
			afk_service_put(srv);

		scoped_guard(spinlock_irqsave, &cec->lock) {
			unsigned long guard_jiffies =
				msecs_to_jiffies(DCP_CEC_TX_COMPLETE_GUARD_MS);

			complete = adap && !cec->stopping && cec->enabled &&
				   cec->tx_seq == seq &&
				   time_before(jiffies, started + guard_jiffies);
		}

		if (complete)
			dcp_cec_complete_tx(adap, status);
	}
}

static void dcp_cec_rx_work(struct work_struct *work)
{
	struct dcp_cec *cec = container_of(work, struct dcp_cec, rx_work);
	struct cec_adapter *adap;

	for (;;) {
		struct dcp_cec_rx_item item;
		bool have_item = false;

		scoped_guard(spinlock_irqsave, &cec->rx_lock) {
			if (cec->rx_count > 0) {
				item = cec->rx_ring[cec->rx_head];
				cec->rx_head = (cec->rx_head + 1) %
					       DCP_CEC_RX_RING_SIZE;
				cec->rx_count--;
				have_item = true;
			}
		}

		if (!have_item)
			break;

		adap = READ_ONCE(cec->adap);
		if (adap && !READ_ONCE(cec->stopping) &&
		    READ_ONCE(cec->rx_enabled))
			cec_received_msg_ts(adap, &item.msg, item.ts);
	}
}

static void dcp_cec_lifecycle_open(struct dcp_cec *cec,
				   struct cec_adapter *adap,
				   struct apple_epic_service *srv, u64 gen)
{
	bool is_enabled = false;
	bool has_newer_pa = false;
	u16 la_mask = 0;
	u16 pa = CEC_PHYS_ADDR_INVALID;
	int ret;

	ret = dcp_cec_ensure_open(cec, srv, gen);
	if (ret < 0) {
		if (ret == -ESTALE || ret == -ECANCELED)
			return;
		scoped_guard(spinlock_irqsave, &cec->lock) {
			if (cec->gen == gen)
				cec->open_attempted_gen = gen;
		}
		return;
	}

	scoped_guard(spinlock_irqsave, &cec->lock) {
		if (!cec->stopping && cec->srv == srv && cec->gen == gen) {
			is_enabled = cec->enabled;
			la_mask = cec->la_mask;
		}
	}

	if (is_enabled)
		dcp_cec_cmd_set_la_mask(srv, true, la_mask);

	if (!adap || !dcp_cec_gen_current(cec, srv, gen))
		return;

	/*
	 * HPD status can still read low on plug. Query the physical address
	 * directly. If EDID is not ready, the EDID attributes report or the
	 * poll after the hotplug EDID read can supply the address later.
	 */
	if (dcp_cec_cmd_get_edid_attrs(srv, &pa) != 0 ||
	    pa == CEC_PHYS_ADDR_INVALID)
		return;

	scoped_guard(spinlock_irqsave, &cec->lock)
		has_newer_pa = cec->pa_event_pending ||
			       cec->pa_invalidate_pending;
	if (!has_newer_pa && dcp_cec_gen_current(cec, srv, gen))
		cec_s_phys_addr(adap, pa, false);
}

static void dcp_cec_lifecycle_work(struct work_struct *work)
{
	struct dcp_cec *cec = container_of(work, struct dcp_cec, lifecycle_work);

	while (!READ_ONCE(cec->stopping)) {
		struct cec_adapter *adap = READ_ONCE(cec->adap);
		struct apple_epic_service *term_srv = NULL;
		struct apple_epic_service *srv = NULL;
		u32 term_chan = 0;
		bool term_was_open = false;
		bool do_pa_inv = false;
		bool do_open = false;
		bool do_hpd = false;
		bool hpd_val = false;
		bool do_pa = false;
		u16 pa_val = CEC_PHYS_ADDR_INVALID;
		bool do_poll_pa = false;
		u64 gen = 0;

		scoped_guard(spinlock_irqsave, &cec->lock) {
			if (cec->stopping)
				return;

			if (cec->terminate_pending) {
				cec->terminate_pending = false;
				if (cec->srv && cec->gen == cec->terminate_gen) {
					term_srv = cec->srv;
					term_chan = cec->terminate_chan;
					term_was_open = cec->opened_gen == cec->gen;
					cec->srv = NULL;
					cec->opened_gen = 0;
					cec->gen++;
				}
			} else if (cec->pa_invalidate_pending) {
				cec->pa_invalidate_pending = false;
				do_pa_inv = true;
			} else if (cec->srv && cec->opened_gen != cec->gen &&
				   cec->open_attempted_gen != cec->gen) {
				cec->open_attempted_gen = cec->gen;
				srv = afk_service_get(cec->srv);
				gen = cec->gen;
				do_open = !!srv;
			} else if (cec->hpd_event_pending) {
				cec->hpd_event_pending = false;
				hpd_val = cec->hpd_event_val;
				srv = afk_service_get(cec->srv);
				gen = cec->gen;
				do_hpd = true;
			} else if (cec->pa_event_pending) {
				cec->pa_event_pending = false;
				pa_val = cec->pa_event_val;
				do_pa = true;
			} else if (cec->poll_pa_pending) {
				cec->poll_pa_pending = false;
				srv = afk_service_get(cec->srv);
				gen = cec->gen;
				do_poll_pa = !!srv;
			}
		}

		if (term_srv) {
			bool invalidate = false;

			if (term_was_open) {
				scoped_guard(mutex, &cec->open_lock)
					dcp_cec_cmd_close(term_srv);
			}
			afk_service_retire_channel(term_srv, term_chan);
			afk_service_put(term_srv);

			scoped_guard(spinlock_irqsave, &cec->lock)
				invalidate = !cec->srv && !cec->stopping;
			if (invalidate && adap)
				cec_phys_addr_invalidate(adap);
			continue;
		}

		if (do_pa_inv) {
			if (adap && !READ_ONCE(cec->stopping))
				cec_phys_addr_invalidate(adap);
			continue;
		}

		if (do_open) {
			dcp_cec_lifecycle_open(cec, adap, srv, gen);
			afk_service_put(srv);
			continue;
		}

		if (do_hpd) {
			u16 pa = CEC_PHYS_ADDR_INVALID;

			if (!hpd_val) {
				dcp_cec_close_client(cec, false);
				if (adap && !READ_ONCE(cec->stopping))
					cec_phys_addr_invalidate(adap);
			} else if (srv && dcp_cec_ensure_open(cec, srv, gen) >= 0 &&
				   dcp_cec_cmd_get_edid_attrs(srv, &pa) == 0) {
				if (adap && dcp_cec_gen_current(cec, srv, gen) &&
				    pa != CEC_PHYS_ADDR_INVALID)
					cec_s_phys_addr(adap, pa, false);
			}
			if (srv)
				afk_service_put(srv);
			continue;
		}

		if (do_pa) {
			if (adap && !READ_ONCE(cec->stopping))
				cec_s_phys_addr(adap, pa_val, false);
			continue;
		}

		if (do_poll_pa) {
			u16 pa = CEC_PHYS_ADDR_INVALID;

			if (adap &&
			    READ_ONCE(adap->phys_addr) == CEC_PHYS_ADDR_INVALID &&
			    dcp_cec_ensure_open(cec, srv, gen) >= 0 &&
			    dcp_cec_cmd_get_edid_attrs(srv, &pa) == 0 &&
			    pa != CEC_PHYS_ADDR_INVALID &&
			    dcp_cec_gen_current(cec, srv, gen))
				cec_s_phys_addr(adap, pa, false);
			afk_service_put(srv);
			continue;
		}

		break;
	}
}

static int dcp_cec_adap_enable(struct cec_adapter *adap, bool enable)
{
	struct dcp_cec *cec = cec_get_drvdata(adap);
	struct apple_epic_service *srv = NULL;
	u64 gen = 0;
	int ret = 0;

	scoped_guard(spinlock_irqsave, &cec->rx_lock) {
		cec->rx_head = 0;
		cec->rx_tail = 0;
		cec->rx_count = 0;
	}

	scoped_guard(spinlock_irqsave, &cec->lock) {
		if (cec->stopping || cec->adap != adap)
			return enable ? -ENODEV : 0;
		cec->la_mask = 0;
		WRITE_ONCE(cec->enabled, enable);
		WRITE_ONCE(cec->rx_enabled, enable);
		if (!enable) {
			cec->tx_seq++;
			cec->tx_pending = false;
		}
		srv = afk_service_get(cec->srv);
		gen = cec->gen;
	}

	if (!srv)
		return 0;

	if (enable) {
		ret = dcp_cec_ensure_open(cec, srv, gen);
		if (ret >= 0)
			ret = dcp_cec_cmd_set_la_mask(srv, true, 0);
		/*
		 * If the service was concurrently replaced or closed,
		 * lifecycle_work will open the successor and set the mask.
		 */
		if (ret < 0 &&
		    (ret == -ECANCELED || !dcp_cec_gen_current(cec, srv, gen)))
			ret = 0;
	} else {
		/* CEC core wraps adap_enable(adap, false) in WARN_ON(). */
		if (READ_ONCE(cec->opened_gen) == gen)
			dcp_cec_cmd_set_la_mask(srv, false, 0);
		ret = 0;
	}

	afk_service_put(srv);
	return enable ? ret : 0;
}

static int dcp_cec_adap_log_addr(struct cec_adapter *adap, u8 logical_addr)
{
	struct dcp_cec *cec = cec_get_drvdata(adap);
	struct apple_epic_service *srv = NULL;
	bool enabled = false;
	u16 la_mask = 0;
	u64 gen = 0;
	int ret = 0;

	scoped_guard(spinlock_irqsave, &cec->lock) {
		if (cec->stopping || cec->adap != adap)
			return logical_addr == CEC_LOG_ADDR_INVALID ?
			       0 : -ENODEV;
		if (logical_addr == CEC_LOG_ADDR_INVALID)
			cec->la_mask = 0;
		else if (logical_addr < CEC_LOG_ADDR_BROADCAST)
			cec->la_mask |= BIT(logical_addr);
		la_mask = cec->la_mask;
		enabled = cec->enabled;
		srv = afk_service_get(cec->srv);
		gen = cec->gen;
	}

	if (!srv)
		return 0;

	if (logical_addr == CEC_LOG_ADDR_INVALID) {
		/* CEC core wraps adap_log_addr(INVALID) in WARN_ON(). */
		if (enabled && READ_ONCE(cec->opened_gen) == gen)
			dcp_cec_cmd_set_la_mask(srv, true, 0);
		afk_service_put(srv);
		return 0;
	}

	ret = dcp_cec_ensure_open(cec, srv, gen);
	if (ret >= 0)
		ret = dcp_cec_cmd_set_la_mask(srv, true, la_mask);
	if (ret < 0 &&
	    (ret == -ECANCELED || !dcp_cec_gen_current(cec, srv, gen)))
		ret = 0;

	afk_service_put(srv);
	return ret;
}

static int dcp_cec_adap_transmit(struct cec_adapter *adap, u8 attempts,
				 u32 signal_free_time, struct cec_msg *msg)
{
	struct dcp_cec *cec = cec_get_drvdata(adap);

	/* Called under adap->lock: never block or call cec_transmit_* here. */
	scoped_guard(spinlock_irqsave, &cec->lock) {
		if (cec->stopping || cec->adap != adap)
			return -ENODEV;
		cec->tx_active_seq = ++cec->tx_seq;
		cec->tx_started = jiffies;
		cec->tx_msg = *msg;
		cec->tx_attempts = attempts;
		cec->tx_pending = true;
	}

	queue_work(system_dfl_wq, &cec->tx_work);
	return 0;
}

static const struct cec_adap_ops dcp_cec_adap_ops = {
	.adap_enable = dcp_cec_adap_enable,
	.adap_log_addr = dcp_cec_adap_log_addr,
	.adap_transmit = dcp_cec_adap_transmit,
};

static void dcp_cec_devm_release(void *data)
{
	struct dcp_cec *cec = data;
	struct apple_epic_service *srv = NULL;

	dcp_cec_unregister(cec->dcp);

	scoped_guard(spinlock_irqsave, &cec->lock) {
		srv = cec->srv;
		cec->srv = NULL;
		cec->opened_gen = 0;
		cec->gen++;
	}
	if (srv)
		afk_service_put(srv);

	if (cec->dcp->cec == cec)
		cec->dcp->cec = NULL;
}

static struct dcp_cec *dcp_cec_ensure_state(struct apple_dcp *dcp)
{
	struct dcp_cec *cec;
	int ret;

	if (!dcp || dcp->fixed_connector_type != DRM_MODE_CONNECTOR_HDMIA)
		return NULL;
	if (dcp->cec)
		return dcp->cec;

	cec = devm_kzalloc(dcp->dev, sizeof(*cec), GFP_KERNEL);
	if (!cec)
		return NULL;

	cec->dcp = dcp;
	mutex_init(&cec->open_lock);
	spin_lock_init(&cec->lock);
	spin_lock_init(&cec->rx_lock);
	INIT_WORK(&cec->lifecycle_work, dcp_cec_lifecycle_work);
	INIT_WORK(&cec->tx_work, dcp_cec_tx_work);
	INIT_WORK(&cec->rx_work, dcp_cec_rx_work);

	ret = devm_add_action_or_reset(dcp->dev, dcp_cec_devm_release, cec);
	if (ret)
		return NULL;

	dcp->cec = cec;
	return cec;
}

/*
 * Only compact endpoint 0x2b publishes dcpav-cec-interface-epic (13.x/14.x use
 * 0x29 and do not get an adapter). Firmware 27.x announces the interface only
 * once HPD rises, so register as soon as 0x2b is live to let CEC_CAP_NEEDS_HPD
 * clients configure /dev/cecN before a sink is plugged in.
 */
static bool dcp_cec_has_endpoint(struct apple_dcp *dcp)
{
	if (!IS_ERR_OR_NULL(dcp->avep) &&
	    dcp->avep->endpoint == AV_AUDIO_ENDPOINT_DESKTOP)
		return true;
	return !!READ_ONCE(dcp->sleep_v27_ep[AV_AUDIO_ENDPOINT_DESKTOP - 0x20]);
}

int dcp_cec_register(struct apple_dcp *dcp)
{
	struct cec_connector_info conn_info;
	struct cec_adapter *adap;
	struct dcp_cec *cec;
	bool need_lifecycle = false;
	int ret;

	if (!dcp || dcp->fixed_connector_type != DRM_MODE_CONNECTOR_HDMIA ||
	    !dcp->fixed_connector || !dcp_cec_has_endpoint(dcp))
		return 0;

	cec = dcp_cec_ensure_state(dcp);
	if (!cec)
		return -ENOMEM;
	if (READ_ONCE(cec->adap))
		return 0;

	scoped_guard(spinlock_irqsave, &cec->rx_lock) {
		cec->rx_head = 0;
		cec->rx_tail = 0;
		cec->rx_count = 0;
	}

	scoped_guard(spinlock_irqsave, &cec->lock) {
		cec->stopping = false;
		cec->enabled = false;
		cec->rx_enabled = false;
		cec->la_mask = 0;
		cec->tx_pending = false;
		/* Re-open a pinned service across component unbind/rebind. */
		cec->opened_gen = 0;
		cec->open_attempted_gen = 0;
		need_lifecycle = !!cec->srv;
	}

	adap = cec_allocate_adapter(&dcp_cec_adap_ops, cec, dev_name(dcp->dev),
				    CEC_CAP_DEFAULTS | CEC_CAP_CONNECTOR_INFO |
				    CEC_CAP_NEEDS_HPD, CEC_MAX_LOG_ADDRS);
	if (IS_ERR(adap))
		return PTR_ERR(adap);

	adap->xfer_timeout_ms = DCP_CEC_ADAP_XFER_TIMEOUT_MS;
	cec_fill_conn_info_from_drm(&conn_info, &dcp->fixed_connector->base);
	cec_s_conn_info(adap, &conn_info);

	WRITE_ONCE(cec->adap, adap);
	ret = cec_register_adapter(adap, dcp->dev);
	if (ret) {
		WRITE_ONCE(cec->adap, NULL);
		cec_delete_adapter(adap);
		dev_warn(dcp->dev, "CEC: failed to register adapter: %d\n", ret);
		return ret;
	}

	if (need_lifecycle)
		queue_work(system_dfl_wq, &cec->lifecycle_work);

	return 0;
}

void dcp_cec_unregister(struct apple_dcp *dcp)
{
	struct apple_epic_service *srv = NULL;
	struct cec_adapter *adap;
	struct dcp_cec *cec;

	if (!dcp || !dcp->cec)
		return;

	cec = dcp->cec;
	scoped_guard(spinlock_irqsave, &cec->lock) {
		cec->stopping = true;
		cec->rx_enabled = false;
		cec->tx_seq++;
		cec->tx_pending = false;
		cec->close_gen++;
		cec->terminate_pending = false;
		cec->pa_invalidate_pending = false;
		cec->hpd_event_pending = false;
		cec->pa_event_pending = false;
		cec->poll_pa_pending = false;
		if (cec->srv && cec->opened_gen == cec->gen) {
			srv = afk_service_get(cec->srv);
			cec->opened_gen = 0;
		}
	}

	/*
	 * Drain all works before unregistering the adapter so no worker can
	 * touch a freed struct cec_adapter. Keep cec->srv pinned across
	 * component unbind so a rebind re-opens the live service without
	 * waiting for a hotplug relink.
	 */
	cancel_work_sync(&cec->tx_work);
	cancel_work_sync(&cec->lifecycle_work);
	cancel_work_sync(&cec->rx_work);

	if (srv) {
		scoped_guard(mutex, &cec->open_lock)
			dcp_cec_cmd_close(srv);
		afk_service_put(srv);
	}

	adap = READ_ONCE(cec->adap);
	if (adap) {
		WRITE_ONCE(cec->adap, NULL);
		cec_unregister_adapter(adap);
	}
}

void dcp_cec_close_port(struct apple_dcp *dcp,
			struct apple_epic_service *port_srv)
{
	if (!dcp || !dcp->cec || !port_srv ||
	    dcp->fixed_connector_type != DRM_MODE_CONNECTOR_HDMIA ||
	    port_srv != READ_ONCE(dcp->dptxport[0].service))
		return;

	dcp_cec_close_client(dcp->cec, true);
}

void dcp_cec_poll_edid(struct apple_dcp *dcp)
{
	struct dcp_cec *cec;
	bool wake = false;

	if (!dcp || !dcp->cec ||
	    dcp->fixed_connector_type != DRM_MODE_CONNECTOR_HDMIA)
		return;

	cec = dcp->cec;
	scoped_guard(spinlock_irqsave, &cec->lock) {
		if (!cec->stopping && cec->srv) {
			cec->poll_pa_pending = true;
			wake = true;
		}
	}

	if (wake)
		queue_work(system_dfl_wq, &cec->lifecycle_work);
}

bool dcp_cec_busy(struct apple_dcp *dcp)
{
	struct dcp_cec *cec;

	if (!dcp || !dcp->cec)
		return false;

	cec = dcp->cec;
	return work_busy(&cec->lifecycle_work) || work_busy(&cec->tx_work);
}

void dcp_cec_service_init(struct apple_epic_service *service, const char *name,
			  const char *class, s64 unit)
{
	struct apple_dcp *dcp = service->ep->dcp;
	struct apple_epic_service *old = NULL;
	struct dcp_cec *cec;
	bool queue = false;

	if (!dcp || dcp->fixed_connector_type != DRM_MODE_CONNECTOR_HDMIA ||
	    READ_ONCE(dcp->active_typec_route) ||
	    service->ep->endpoint != AV_AUDIO_ENDPOINT_DESKTOP)
		return;

	cec = dcp_cec_ensure_state(dcp);
	if (!cec)
		return;

	/*
	 * Start the compact interface on ep->wq before publishing cec->srv so
	 * lifecycle_work or adap_enable cannot send Open ahead of compact 0x12.
	 */
#if IS_ENABLED(CONFIG_DRM_APPLE_KUNIT_TEST)
	if (!cec->test_call && afk_start_service(service))
		return;
#else
	if (afk_start_service(service))
		return;
#endif

	service = afk_service_get(service);
	if (!service)
		return;

	scoped_guard(spinlock_irqsave, &cec->lock) {
		old = cec->srv;
		cec->srv = service;
		cec->gen++;
		cec->opened_gen = 0;
		cec->open_attempted_gen = 0;
		cec->terminate_pending = false;
		cec->pa_invalidate_pending = false;
		queue = !cec->stopping && !!cec->adap;
	}

	if (old)
		afk_service_put(old);

	if (queue)
		queue_work(system_dfl_wq, &cec->lifecycle_work);
}

int dcp_cec_service_report(struct apple_epic_service *service,
			   enum epic_subtype type,
			   const void *data, size_t data_size)
{
	struct apple_dcp *dcp = service->ep->dcp;
	struct dcp_cec *cec = dcp->cec;
	struct dcp_cec_parsed_report rpt;
	bool queue_lifecycle = false;
	bool queue_rx = false;
	ktime_t now;
	int ret;

	if (!cec || READ_ONCE(cec->stopping) ||
	    service->ep->endpoint != AV_AUDIO_ENDPOINT_DESKTOP)
		return 0;

	ret = dcp_cec_parse_report(data, data_size, &rpt);
	if (ret)
		return 0;

	switch (rpt.cmd) {
	case DCP_CEC_RPT_TERMINATE:
		scoped_guard(spinlock_irqsave, &cec->lock) {
			if (!cec->stopping && cec->srv == service) {
				cec->terminate_pending = true;
				cec->terminate_gen = cec->gen;
				cec->terminate_chan = READ_ONCE(service->channel);
				cec->close_gen++;
				cec->tx_seq++;
				cec->tx_pending = false;
				queue_lifecycle = true;
			}
		}
		break;

	case DCP_CEC_RPT_RX_FRAME:
		if (!READ_ONCE(cec->rx_enabled))
			return 0;
		scoped_guard(spinlock_irqsave, &cec->lock) {
			if (cec->stopping || cec->srv != service ||
			    !cec->rx_enabled)
				return 0;
		}
		now = ktime_get();
		scoped_guard(spinlock_irqsave, &cec->rx_lock) {
			if (cec->rx_count == DCP_CEC_RX_RING_SIZE) {
				/* Drop oldest unread frame on ring overflow. */
				cec->rx_head = (cec->rx_head + 1) %
					       DCP_CEC_RX_RING_SIZE;
				cec->rx_count--;
			}
			cec->rx_ring[cec->rx_tail].msg = rpt.rx_msg;
			cec->rx_ring[cec->rx_tail].ts = now;
			cec->rx_tail = (cec->rx_tail + 1) %
				       DCP_CEC_RX_RING_SIZE;
			cec->rx_count++;
			queue_rx = true;
		}
		break;

	case DCP_CEC_RPT_HPD:
		scoped_guard(spinlock_irqsave, &cec->lock) {
			if (!cec->stopping && cec->srv == service) {
				cec->hpd_event_pending = true;
				cec->hpd_event_val = rpt.hpd;
				if (!rpt.hpd) {
					cec->close_gen++;
					cec->tx_seq++;
					cec->tx_pending = false;
					cec->pa_event_pending = false;
					cec->poll_pa_pending = false;
				}
				queue_lifecycle = true;
			}
		}
		break;

	case DCP_CEC_RPT_EDID_ATTRS:
		scoped_guard(spinlock_irqsave, &cec->lock) {
			if (!cec->stopping && cec->srv == service) {
				cec->pa_invalidate_pending = false;
				cec->pa_event_pending = true;
				cec->pa_event_val = rpt.phys_addr;
				queue_lifecycle = true;
			}
		}
		break;

	default:
		break;
	}

	if (queue_lifecycle)
		queue_work(system_dfl_wq, &cec->lifecycle_work);
	if (queue_rx)
		queue_work(system_dfl_wq, &cec->rx_work);

	return 0;
}

void dcp_cec_service_teardown(struct apple_epic_service *service)
{
	struct apple_dcp *dcp = service->ep->dcp;
	struct dcp_cec *cec = dcp->cec;
	bool owned = false;
	bool queue_inv = false;

	afk_service_disable(service);
	if (!cec)
		return;

	/*
	 * Drop the owner pin synchronously and never send an async Close
	 * here: afk_compact_slot() reuses singleton slots in place right after
	 * ops->teardown().
	 */
	scoped_guard(spinlock_irqsave, &cec->lock) {
		if (cec->srv == service) {
			cec->srv = NULL;
			cec->opened_gen = 0;
			cec->close_gen++;
			cec->tx_seq++;
			cec->tx_pending = false;
			cec->terminate_pending = false;
			cec->gen++;
			owned = true;
			if (!cec->stopping) {
				cec->pa_invalidate_pending = true;
				queue_inv = true;
			}
		}
	}

	if (owned)
		afk_service_put(service);
	if (queue_inv)
		queue_work(system_dfl_wq, &cec->lifecycle_work);
}

static void dcp_cec_unused_init(struct apple_epic_service *service,
				const char *name, const char *class, s64 unit)
{
}

static int dcp_cec_unused_report(struct apple_epic_service *service,
				 enum epic_subtype type,
				 const void *data, size_t data_size)
{
	return 0;
}

static void dcp_cec_unused_teardown(struct apple_epic_service *service)
{
	afk_service_disable(service);
}

static const struct apple_epic_service_ops dcp_cec_desktop_ops[] = {
	{
		.name = "dcpav-audio-interface-epic",
		.compact_full = "dcpav-audio-interface-epic:0",
		.singleton = true,
		.init = dcp_cec_unused_init,
		.report = dcp_cec_unused_report,
		.teardown = dcp_cec_unused_teardown,
	},
	DCP_CEC_COMPACT_SERVICE_OPS,
	{
		.name = "dcpav-video-interface-epic",
		.compact_full = "dcpav-video-interface-epic:0",
		.singleton = true,
		.init = dcp_cec_unused_init,
		.report = dcp_cec_unused_report,
		.teardown = dcp_cec_unused_teardown,
	},
	{}
};

static void dcp_cec_avep_cleanup(void *data)
{
	struct dcp_cec *cec = data;
	struct apple_dcp *dcp = cec->dcp;
	struct apple_dcp_afkep *ep = cec->owned_avep;

	if (IS_ERR_OR_NULL(ep))
		return;

	afk_quiesce(ep);
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP &&
	    dcp->desktop &&
	    READ_ONCE(dcp->desktop->graph[ep->endpoint - 0x20]) == ep)
		WRITE_ONCE(dcp->desktop->graph[ep->endpoint - 0x20], NULL);
	if (dcp->avep == ep)
		dcp->avep = NULL;
	cec->owned_avep = NULL;
}

/*
 * Start compact endpoint 0x2b on HDMI DCPs when audio did not start dcp->avep
 * (e.g. appledrm.hdmi_audio=0 or !CONFIG_DRM_APPLE_AUDIO). dcp_start() and
 * dcp_start_desktop_external() are one-shot per device bind, and
 * avep_cleanup_registered guards against registering duplicate devm actions.
 */
int dcp_cec_init_avep(struct apple_dcp *dcp)
{
	struct apple_dcp_afkep *ep;
	struct dcp_cec *cec;
	int ret;

	if (!dcp || dcp->fixed_connector_type != DRM_MODE_CONNECTOR_HDMIA ||
	    !IS_ERR_OR_NULL(dcp->avep) || !afk_compact_av_capable(dcp) ||
	    !apple_rtkit_has_endpoint(dcp->rtk, AV_AUDIO_ENDPOINT_DESKTOP))
		return 0;

	cec = dcp_cec_ensure_state(dcp);
	if (!cec)
		return -ENOMEM;

	ep = afk_init(dcp, AV_AUDIO_ENDPOINT_DESKTOP, dcp_cec_desktop_ops);
	if (IS_ERR(ep))
		return PTR_ERR(ep);

	cec->owned_avep = ep;
	if (!cec->avep_cleanup_registered) {
		ret = devm_add_action_or_reset(dcp->dev, dcp_cec_avep_cleanup,
					       cec);
		if (ret)
			return ret;
		cec->avep_cleanup_registered = true;
	}

	ep->debugfs_entry = dcp->ep_debugfs[AV_AUDIO_ENDPOINT_DESKTOP - 0x20];
	dcp->avep = ep;
	if (dcp->fw_compat == DCP_FIRMWARE_V_26_6_DESKTOP)
		WRITE_ONCE(dcp->desktop->graph[AV_AUDIO_ENDPOINT_DESKTOP - 0x20],
			   ep);

	ret = afk_start(ep);
	if (ret)
		dcp_cec_avep_cleanup(cec);

	return ret;
}

#if IS_ENABLED(CONFIG_DRM_APPLE_KUNIT_TEST)
#include "cec_adap_test.c"
#endif
