/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple RTKit IPC library
 * Copyright (C) The Asahi Linux Contributors
 */

#ifndef _APPLE_RTKIT_INTERAL_H
#define _APPLE_RTKIT_INTERAL_H

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/soc/apple/mailbox.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/workqueue.h>

#define APPLE_RTKIT_APP_ENDPOINT_START 0x20
#define APPLE_RTKIT_MAX_ENDPOINTS 0x100
#define APPLE_RTKIT_CRASHLOG_HEADER_FOURCC 0x434c4845

struct apple_rtkit_crashlog_header {
	u32 fourcc;
	u32 version;
	u32 size;
	u32 flags;
	u8 _unk[16];
};

static_assert(sizeof(struct apple_rtkit_crashlog_header) == 0x20);

struct apple_rtkit {
	void *cookie;
	const struct apple_rtkit_ops *ops;
	struct device *dev;

	struct apple_mbox *mbox;

	struct completion epmap_completion;
	struct completion iop_pwr_ack_completion;
	struct completion ap_pwr_ack_completion;

	int boot_result;
	int version;
	/* How long to wait for each handshake reply from the co-processor. */
	unsigned int handshake_timeout_ms;

	unsigned int iop_power_state;
	unsigned int ap_power_state;
	bool crashed;
	bool shutting_down;
	/*
	 * apple,sync-syslog-ack (AsahiLinux/linux#607, the T8122 dcpext):
	 * acknowledge syslog and ioreport traffic ahead of other receive work,
	 * hand out kdebug buffers, and poll the mailbox while waiting.
	 */
	bool sync_syslog_ack;
	DECLARE_KFIFO(sync_ack_fifo, struct apple_mbox_msg, 64);
	struct work_struct sync_ack_work;

	DECLARE_BITMAP(endpoints, APPLE_RTKIT_MAX_ENDPOINTS);

	struct apple_rtkit_shmem ioreport_buffer;
	struct apple_rtkit_shmem crashlog_buffer;
	struct apple_rtkit_shmem oslog_buffer;
	struct apple_rtkit_shmem kdebug_buffer0;
	struct apple_rtkit_shmem kdebug_buffer1;

	struct apple_rtkit_shmem syslog_buffer;
	char *syslog_msg_buffer;
	size_t syslog_n_entries;
	size_t syslog_msg_size;
	/* Session inherited from the bootloader: INIT geometry never observed. */
	bool syslog_inherited;
	bool crashlog_inherited;
	bool oslog_inherited;

	struct workqueue_struct *wq;
};

void apple_rtkit_crashlog_dump(struct apple_rtkit *rtk, u8 *bfr, size_t size);

#endif
