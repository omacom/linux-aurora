/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple RTKit IPC Library
 * Copyright (C) The Asahi Linux Contributors
 *
 * Apple's SoCs come with various co-processors running their RTKit operating
 * system. This protocol library is used by client drivers to use the
 * features provided by them.
 */
#ifndef _LINUX_APPLE_RTKIT_H_
#define _LINUX_APPLE_RTKIT_H_

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/mailbox_client.h>

/*
 * Struct to represent implementation-specific RTKit operations.
 *
 * @buffer:    Shared memory buffer allocated inside normal RAM.
 * @iomem:     Shared memory buffer controlled by the co-processors.
 * @size:      Size of the shared memory buffer.
 * @iova:      Device VA of shared memory buffer.
 * @is_mapped: Shared memory buffer is managed by the co-processor.
 * @needs_dma_sync: The buffer is a streaming DMA mapping of normal RAM made by
 *                  the parent driver's shmem_setup, with @iova as its DMA
 *                  handle. RTKit synchronizes it for the CPU before reading.
 *                  Never set this for coherent allocations or for iomem.
 * @private:   Private data pointer for the parent driver.
 */

struct apple_rtkit_shmem {
	void *buffer;
	void __iomem *iomem;
	size_t size;
	dma_addr_t iova;
	bool is_mapped;
	bool needs_dma_sync;
	void *private;
};

/*
 * The co-processor places its shared buffers itself. A buffer request that
 * carries an IOVA is set up with shmem_setup and acknowledged by sending the
 * request back unchanged. The crashlog endpoint is started while the endpoint
 * map is handled, before the map is acknowledged.
 */
#define APPLE_RTKIT_COPROC_PLACES_BUFFERS	BIT(0)

/*
 * Struct to represent implementation-specific RTKit operations.
 *
 * @flags:         Combination of APPLE_RTKIT_* flags describing the
 *                 co-processor.
 * @crashed:       Called when the co-processor has crashed. Runs in process
 *                 context.
 * @recv_message:  Function called when a message from RTKit is received
 *                 on a non-system endpoint. Called from a worker thread.
 * @recv_message_timed: Like recv_message, carrying the monotonic nanosecond
 *                 timestamp captured before deferring mailbox RX. If set,
 *                 this callback is used instead of recv_message.
 * @recv_message_early:
 *                 Like recv_message, but called from atomic context. It
 *                 should return true if it handled the message. If it
 *                 returns false, the message will be passed on to the
 *                 worker thread.
 * @shmem_setup:   Setup shared memory buffer. A nonzero input iova requests a
 *                 mapping of firmware-owned memory; validate the ownership
 *                 before mapping it. Otherwise allocate a Linux-owned buffer.
 *                 On failure undo any partial setup. Do not retain the
 *                 descriptor pointer and do not change size: repeated
 *                 requests are matched against it. If not specified
 *                 dma_alloc_coherent is used. Called in process context.
 * @shmem_destroy: Undo the shared memory buffer setup in shmem_setup. If not
 *                 specified dma_free_coherent is used. Called in process
 *                 context.
 */
struct apple_rtkit_ops {
	unsigned int flags;
	void (*crashed)(void *cookie, const void *crashlog, size_t crashlog_size);
	void (*recv_message)(void *cookie, u8 endpoint, u64 message);
	void (*recv_message_timed)(void *cookie, u8 endpoint, u64 message, u64 received_ns);
	bool (*recv_message_early)(void *cookie, u8 endpoint, u64 message);
	int (*shmem_setup)(void *cookie, struct apple_rtkit_shmem *bfr);
	void (*shmem_destroy)(void *cookie, struct apple_rtkit_shmem *bfr);
};

struct apple_rtkit;

/*
 * Initializes the internal state required to handle RTKit. This
 * should usually be called within _probe.
 *
 * @dev:         Pointer to the device node this coprocessor is associated with
 * @cookie:      opaque cookie passed to all functions defined in rtkit_ops
 * @mbox_name:   mailbox name used to communicate with the co-processor
 * @mbox_idx:    mailbox index to be used if mbox_name is NULL
 * @ops:         pointer to rtkit_ops to be used for this co-processor
 */
struct apple_rtkit *devm_apple_rtkit_init(struct device *dev, void *cookie,
					  const char *mbox_name, int mbox_idx,
					  const struct apple_rtkit_ops *ops);
/*
 * Frees internal RTKit state allocated by devm_apple_rtkit_init().
 *
 * @dev:	Pointer to the device node this coprocessor is assocated with
 * @rtk:	Internal RTKit state initialized by devm_apple_rtkit_init()
 */
void devm_apple_rtkit_free(struct device *dev, struct apple_rtkit *rtk);

/*
 * Non-devm version of devm_apple_rtkit_init. Must be freed with
 * apple_rtkit_free.
 *
 * @dev:         Pointer to the device node this coprocessor is associated with
 * @cookie:      opaque cookie passed to all functions defined in rtkit_ops
 * @mbox_name:   mailbox name used to communicate with the co-processor
 * @mbox_idx:    mailbox index to be used if mbox_name is NULL
 * @ops:         pointer to rtkit_ops to be used for this co-processor
 */
struct apple_rtkit *apple_rtkit_init(struct device *dev, void *cookie,
					  const char *mbox_name, int mbox_idx,
					  const struct apple_rtkit_ops *ops);

/*
 * Initialize an instance for a co-processor that the bootloader left running
 * and that cannot be reset. No HELLO/EPMAP handshake takes place: the system
 * endpoints are marked available and syslog/OSLog records are acknowledged without
 * being parsed. The caller must verify the device-specific running/ready
 * indicators before using the instance.
 */
struct apple_rtkit *apple_rtkit_init_adopted(struct device *dev, void *cookie,
					     const char *mbox_name, int mbox_idx,
					     const struct apple_rtkit_ops *ops);

/*
 * Free an instance of apple_rtkit.
 */
void apple_rtkit_free(struct apple_rtkit *rtk);

/*
 * Detach a co-processor that is still running, or that could not be
 * quiesced, without releasing its shared buffers.
 *
 * Stops mailbox callbacks and drains the private RTKit workqueue before
 * freeing the transport state. Firmware-visible mappings and their allocation
 * contexts are intentionally retained until reboot; shmem_destroy is not
 * called. The caller must also retain any buffers it supplied independently
 * and must not treat this as a successful firmware shutdown or as permission
 * to reprobe. The caller must exclude concurrent users of rtk, as for
 * apple_rtkit_free().
 */
void apple_rtkit_free_retaining_buffers(struct apple_rtkit *rtk);

/*
 * Reinitialize internal structures. Must only be called with the co-processor
 * is held in reset.
 */
int apple_rtkit_reinit(struct apple_rtkit *rtk);

/*
 * Handle RTKit's boot process. Should be called after the CPU of the
 * co-processor has been started.
 */
int apple_rtkit_boot(struct apple_rtkit *rtk);

/*
 * Mark an instance created with apple_rtkit_init() as running without a
 * handshake, see apple_rtkit_init_adopted().
 */
int apple_rtkit_adopt_running(struct apple_rtkit *rtk);

/*
 * Quiesce the co-processor.
 */
int apple_rtkit_quiesce(struct apple_rtkit *rtk);

/*
 * Wake the co-processor up from hibernation mode.
 */
int apple_rtkit_wake(struct apple_rtkit *rtk);

/* Wake helpers which require AP readiness before their IOP power acknowledgment. */
int apple_rtkit_wake_early_ap(struct apple_rtkit *rtk);

/*
 * Shutdown the co-processor
 */
int apple_rtkit_shutdown(struct apple_rtkit *rtk);

/*
 * Put the co-processor into the lowest power state. Note that it usually
 * is not possible to recover from this state without a full SoC reset.
 */

int apple_rtkit_poweroff(struct apple_rtkit *rtk);

/*
 * Put the co-processor into idle mode
 */
int apple_rtkit_idle(struct apple_rtkit *rtk);

/*
 * Checks if RTKit is running and ready to handle messages.
 */
bool apple_rtkit_is_running(struct apple_rtkit *rtk);

/*
 * Checks if RTKit has crashed.
 */
bool apple_rtkit_is_crashed(struct apple_rtkit *rtk);

/*
 * Starts an endpoint. Must be called after boot but before any messages can be
 * sent or received from that endpoint.
 */
int apple_rtkit_start_ep(struct apple_rtkit *rtk, u8 endpoint);

/*
 * Send a message to the given endpoint.
 *
 * @rtk:            RTKit reference
 * @ep:             target endpoint
 * @message:        message to be sent
 * @completeion:    will be completed once the message has been submitted
 *                  to the hardware FIFO. Can be NULL.
 * @atomic:         if set to true this function can be called from atomic
 *                  context.
 */
int apple_rtkit_send_message(struct apple_rtkit *rtk, u8 ep, u64 message,
			     struct completion *completion, bool atomic);

/*
 * Process incoming messages in atomic context.
 * This only guarantees that messages arrive as far as the recv_message_early
 * callback; drivers expecting to handle incoming messages synchronously
 * by calling this function must do it that way.
 * Will return 1 if some data was processed, 0 if none was, or a
 * negative error code on failure.
 *
 * @rtk:            RTKit reference
 */
int apple_rtkit_poll(struct apple_rtkit *rtk);

/*
 * Stop taking interrupts for messages from the co-processor, which then
 * stay queued in the mailbox until apple_rtkit_resume_rx().
 */
void apple_rtkit_hold_rx(struct apple_rtkit *rtk);
void apple_rtkit_resume_rx(struct apple_rtkit *rtk);

/*
 * Wait until the messages already received have been handled, including the
 * recv_message, crashed and shmem_setup callbacks the worker thread ran for
 * them. Must not be called from one of those callbacks.
 *
 * @rtk:            RTKit reference
 */
void apple_rtkit_flush_rx(struct apple_rtkit *rtk);

/*
 * Checks if an endpoint with a given index exists
 *
 * @rtk:            RTKit reference
 * @ep:             endpoint to check for
 */
bool apple_rtkit_has_endpoint(struct apple_rtkit *rtk, u8 ep);

#endif /* _LINUX_APPLE_RTKIT_H_ */
