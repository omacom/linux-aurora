/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple mailbox message format
 *
 * Copyright The Asahi Linux Contributors
 */

#ifndef _APPLE_MAILBOX_H_
#define _APPLE_MAILBOX_H_

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/types.h>

/* encodes a single 96bit message sent over the single channel */
struct apple_mbox_msg {
	u64 msg0;
	u32 msg1;
};

/**
 * enum apple_mbox_ascwrap_v6_role - firmware role owned by an ASCWrap v6
 * mailbox provider
 * @APPLE_MBOX_ASCWRAP_V6_ROLE_NONE: not an ASCWrap v6 provider
 * @APPLE_MBOX_ASCWRAP_V6_ROLE_GFX: primary GPU firmware role
 * @APPLE_MBOX_ASCWRAP_V6_ROLE_GFX1: secondary GPU firmware role
 */
enum apple_mbox_ascwrap_v6_role {
	APPLE_MBOX_ASCWRAP_V6_ROLE_NONE,
	APPLE_MBOX_ASCWRAP_V6_ROLE_GFX,
	APPLE_MBOX_ASCWRAP_V6_ROLE_GFX1,
};

/**
 * struct apple_mbox_ascwrap_v6_lifecycle - provider-owned lifecycle facts
 * @role: firmware role owned by this provider
 * @wrapper_start: CPU physical base of the provider's ASCWrap window
 * @wrapper_size: size of the provider's ASCWrap window
 * @iop_vbar_start: CPU physical base of the provider's IOP VBAR window
 * @iop_vbar_size: size of the provider's IOP VBAR window
 * @missing: remaining provider lifecycle work; zero for T8140
 *
 * The apple-mailbox provider owns both mappings.  Consumers request CPU
 * transitions through the provider and never map either resource themselves.
 */
struct apple_mbox_ascwrap_v6_lifecycle {
	enum apple_mbox_ascwrap_v6_role role;
	u64 wrapper_start;
	u64 wrapper_size;
	u64 iop_vbar_start;
	u64 iop_vbar_size;
	u32 missing;
};

struct apple_mbox {
	struct device *dev;
	void __iomem *regs;
	void __iomem *wrapper_regs;
	void __iomem *iop_vbar_regs;
	const struct apple_mbox_hw *hw;
	struct apple_mbox_ascwrap_v6_lifecycle ascwrap_v6;
	/* Serializes transport start/stop against provider removal. */
	struct mutex lifecycle_lock;
	bool active;
	bool ascwrap_v6_cpu_running;
	bool removing;

	int irq_recv_not_empty;
	int irq_send_empty; /* < 0: no send-empty IRQ, TX polls */

	spinlock_t rx_lock;
	spinlock_t tx_lock;

	struct completion tx_empty;

	/** Receive callback for incoming messages */
	void (*rx)(struct apple_mbox *mbox, struct apple_mbox_msg msg, void *cookie);
	void *cookie;

	/*
	 * Minimum utilization (uclamp_min) a consumer asked the threaded receive
	 * handler to run with, and the value it last applied to itself; -1 is the
	 * scheduler's default. See apple_mbox_set_rx_util_min().
	 */
	int rx_util_min;
	int rx_util_min_applied;

	/*
	 * Protected by tx_lock: set while irq_send_empty is unmasked, so that
	 * the IRQ handler, a failed send and provider removal mask it exactly
	 * once between them.
	 */
	bool tx_irq_unmasked;
};

struct apple_mbox *apple_mbox_get(struct device *dev, int index);
struct apple_mbox *apple_mbox_get_byname(struct device *dev, const char *name);

int apple_mbox_start(struct apple_mbox *mbox);
void apple_mbox_stop(struct apple_mbox *mbox);
int apple_mbox_poll(struct apple_mbox *mbox);
/*
 * Leave received messages in the hardware FIFO, without an interrupt, until
 * apple_mbox_release_rx().  For consumers that must not wake the system.
 */
void apple_mbox_hold_rx(struct apple_mbox *mbox);
void apple_mbox_release_rx(struct apple_mbox *mbox);
/*
 * Set the minimum utilization (0..1024) the threaded receive handler runs with,
 * or -1 to restore the scheduler's default; the handler applies it to itself
 * when it next runs. Only mailboxes with a threaded receive handler support it.
 */
int apple_mbox_set_rx_util_min(struct apple_mbox *mbox, int util_min);
int apple_mbox_send(struct apple_mbox *mbox, struct apple_mbox_msg msg,
		    bool atomic);

int apple_mbox_ascwrap_v6_get_lifecycle(
	struct apple_mbox *mbox,
	struct apple_mbox_ascwrap_v6_lifecycle *lifecycle);
int apple_mbox_ascwrap_v6_require_safe_cpu_lifecycle(struct apple_mbox *mbox);
int apple_mbox_ascwrap_v6_start_cpu(struct apple_mbox *mbox);
int apple_mbox_ascwrap_v6_stop_cpu(struct apple_mbox *mbox);

#endif
