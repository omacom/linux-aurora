// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright The Asahi Linux Contributors */

#ifndef _DRM_APPLE_EPIC_DPAVSERV_H
#define _DRM_APPLE_EPIC_DPAVSERV_H

#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct drm_edid;
struct apple_epic_service;
struct apple_dcp;

struct dcpavserv {
	/* Process context only: serializes retirement and EDID publication.
	 * Nest below mode_config.mutex and hpd_mutex; never wait for firmware.
	 */
	struct mutex lock;
	u64 epoch; /* Changes even when the same service slot is reused. */
	bool enabled;
	struct completion enable_completion;
	u32 unit;
	struct apple_epic_service *service;
	/* The MCDP29xx DP->HDMI converter's service: DDC/CI at 0xB7 */
	bool mcdp29xx;
	/*
	 * The 14.7 external role's AV client, per instance: an instance is
	 * named by the epoch it was published in, so a slot reused for its
	 * successor is not mistaken for it.
	 *
	 * opened/opened_epoch: the instance the host opened as a client (DCPAV
	 * Open), closed again before the sink's HPD low or at its Terminate;
	 * the pointer is compared, and pinned before use.
	 * opening: an Open is on its way. close_gen: bumped by every Close and
	 * Terminate; an Open that sees it change was cancelled, and its opener
	 * closes what it opened.
	 * terminated_epoch: Terminate reported; that instance gets no new use.
	 * close_epoch: the instance the Close work was queued for.
	 */
	struct apple_epic_service *opened;
	u64 opened_epoch;
	bool opening;
	u64 close_gen;
	u64 terminated_epoch;
	u64 close_epoch;
	struct work_struct close_wq;
};

/* The 14.7 external role: close the AV client before the sink is released. */
void dcpavserv_v14_close(struct apple_dcp *dcp);
/* Its Terminate work; set up once with the DCP (dcp_platform_probe_pipe()). */
void dcpavserv_close_work(struct work_struct *work);

bool dcpavserv_edid_current(struct apple_dcp *dcp, u64 generation);
bool dcpavserv_edid_service_current(struct apple_dcp *dcp, u64 epoch);
const struct drm_edid *dcpavserv_copy_edid(struct apple_dcp *dcp, u64 generation,
					u64 *epoch);
void dpavservep_detach(struct apple_dcp *dcp);

#endif /* _DRM_APPLE_EPIC_DPAVSERV_H */
