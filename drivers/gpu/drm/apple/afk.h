// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * AFK (Apple Firmware Kit) EPIC (EndPoint Interface Client) support
 */
/* Copyright 2022 Sven Peter <sven@svenpeter.dev> */

#ifndef _DRM_APPLE_DCP_AFK_H
#define _DRM_APPLE_DCP_AFK_H

#include <linux/bitmap.h>
#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/kconfig.h>
#include <linux/math.h>
#include <linux/types.h>

#include "dcp.h"

#define AFK_MAX_CHANNEL 16
#define MAX_PENDING_CMDS 16
#define AFKV2_INTERFACES BIT(16)

struct apple_epic_service_ops;
struct apple_dcp_afkep;

void afk_service_retry(struct apple_dcp_afkep *ep);

struct epic_cmd_info {
	u16 tag;
	u8 type;
	size_t reply_length;

	void *rxbuf;
	void *txbuf;
	dma_addr_t rxbuf_dma;
	dma_addr_t txbuf_dma;
	size_t rxlen;
	size_t txlen;
	/* DMA allocation lengths above stay immutable until buffers are freed. */
	size_t reply_len;
	bool reply_len_valid;

	u32 retcode;
	bool done;
	bool free_on_ack;
	struct completion *completion;
};

struct apple_epic_service {
	const struct apple_epic_service_ops *ops;
	struct apple_dcp_afkep *ep;

	struct epic_cmd_info cmds[MAX_PENDING_CMDS];
	DECLARE_BITMAP(cmd_map, MAX_PENDING_CMDS);
	/* Legacy EPIC sequence; AFKV2 history belongs to the interface. */
	u8 cmd_tag;
	spinlock_t lock;
	/* Pins held by owners and synchronous callers; protected by lock. */
	unsigned int users;

	u32 channel;
	bool enabled;
	bool torndown;

	/* Provider-owned prepare/init ticket; AFK core never interprets it. */
	struct { u64 generation, epoch; } admission;
	void *cookie;

	/* Admitted reannouncement waiting for the previous channel to drain. */
	bool pending_open;
	const char *pending_name, *pending_class;
	s64 pending_unit;

    struct {
        struct dentry *entry;
        u8 *scratch;
    } debugfs;
};

enum epic_subtype;

struct apple_epic_service_ops {
	const char name[32];
	/* All retained pointers must be pinned before enabling slot reuse. */
	bool reusable;
	/* EOPNOTSUPP from prepare means intentional absence only for this role. */
	bool optional;

	/*
	 * Optional admission, before OPEN/publication. Must not retain service
	 * on failure; teardown must accept a failed/partial preparation.
	 */
	int (*prepare)(struct apple_epic_service *service, const char *name,
		       const char *class, s64 unit);
	/* Check the original prepare ticket before a deferred OPEN. */
	int (*revalidate)(struct apple_epic_service *service, const char *name,
			  const char *class, s64 unit);
	void (*init)(struct apple_epic_service *service, const char *name,
			      const char *class, s64 unit);
	int (*call)(struct apple_epic_service *service, u32 idx,
		    const void *data, size_t data_size, void *reply,
		    size_t reply_size);
	int (*call_v2)(struct apple_epic_service *service, u16 group, u32 command,
		       const void *data, size_t size, void *reply, size_t room);
	int (*report)(struct apple_epic_service *service, enum epic_subtype type,
		      const void *data, size_t data_size);
	void (*teardown)(struct apple_epic_service *service);
};

static inline int afk_ring_stride(u32 total, u32 bufsz, u32 *stride)
{
	u32 header, block;

	if (bufsz >= total)
		return -EINVAL;
	header = total - bufsz;
	block = header / 3;
	if (header % 3 || block < 0x40 || block % 0x40 ||
	    bufsz < block || bufsz % block)
		return -EINVAL;

	*stride = block;
	return 0;
}

static inline u32 afk_ring_advance(u32 ptr, u32 bytes, u32 bufsz,
				   u32 stride)
{
	u32 next = roundup(ptr + bytes, stride);

	return next == bufsz ? 0 : next;
}
static inline struct apple_epic_service *afk_service_get(struct apple_epic_service *service)
{
	unsigned long flags;
	bool available;

	if (!service)
		return NULL;

	spin_lock_irqsave(&service->lock, flags);
	available = service->enabled && !service->torndown;
	if (available)
		service->users++;
	spin_unlock_irqrestore(&service->lock, flags);

	return available ? service : NULL;
}

static inline void afk_service_put(struct apple_epic_service *service)
{
	unsigned long flags;

	spin_lock_irqsave(&service->lock, flags);
	if (!WARN_ON(!service->users))
		service->users--;
	spin_unlock_irqrestore(&service->lock, flags);
	afk_service_retry(service->ep);
}

/*
 * Take another pin on a service the caller already keeps pinned, e.g. an
 * owner pointer read under the owner's lock. Unlike afk_service_get() this
 * also succeeds after teardown, so a close can still be issued.
 */
static inline struct apple_epic_service *afk_service_pin(struct apple_epic_service *service)
{
	unsigned long flags;

	if (!service)
		return NULL;

	spin_lock_irqsave(&service->lock, flags);
	service->users++;
	spin_unlock_irqrestore(&service->lock, flags);

	return service;
}

static inline void afk_service_disable(struct apple_epic_service *service)
{
	unsigned long flags;

	spin_lock_irqsave(&service->lock, flags);
	service->enabled = false;
	spin_unlock_irqrestore(&service->lock, flags);
}

/* Retire local ownership while keeping timed-out commands discoverable. */
static inline void afk_service_retire(struct apple_epic_service *service)
{
	unsigned long flags;

	spin_lock_irqsave(&service->lock, flags);
	service->enabled = false;
	service->torndown = true;
	spin_unlock_irqrestore(&service->lock, flags);
}

static inline bool afk_service_matches(struct apple_epic_service *service,
				       u32 channel)
{
	unsigned long flags;
	bool found;

	spin_lock_irqsave(&service->lock, flags);
	/* Teardown does not cancel commands or relinquish their DMA. */
	found = service->channel == channel &&
		(service->enabled ||
		 !bitmap_empty(service->cmd_map, MAX_PENDING_CMDS));
	spin_unlock_irqrestore(&service->lock, flags);
	return found;
}

/*
 * True if this slot last served @channel and was retired or discarded on our
 * side: the co-processor may still close that channel afterwards.
 */
static inline bool afk_service_retired(struct apple_epic_service *service,
				       u32 channel)
{
	unsigned long flags;
	bool retired;

	spin_lock_irqsave(&service->lock, flags);
	retired = service->channel == channel && service->torndown;
	spin_unlock_irqrestore(&service->lock, flags);
	return retired;
}

/* The caller holds service->lock. */
static inline bool afk_service_reclaimable(struct apple_epic_service *service)
{
	return !service->ops ||
		(service->ops->reusable && service->torndown &&
		 !service->enabled && !service->users &&
		 bitmap_empty(service->cmd_map, MAX_PENDING_CMDS));
}

static inline bool
afk_service_reinit(struct apple_epic_service *service,
		   struct apple_dcp_afkep *ep,
		   const struct apple_epic_service_ops *ops, u32 channel)
{
	unsigned long flags;
	bool reusable;

	spin_lock_irqsave(&service->lock, flags);
	reusable = afk_service_reclaimable(service);
	if (reusable) {
		service->enabled = !ops->prepare;
		service->torndown = false;
		service->ops = ops;
		service->ep = ep;
		service->channel = channel;
		service->cookie = NULL;
		/* Keep legacy slot history; AFKV2 history belongs to the interface. */
	}
	spin_unlock_irqrestore(&service->lock, flags);
	return reusable;
}

/* Only used before a prepared service has been published to callers. */
static inline void afk_service_discard(struct apple_epic_service *service)
{
	unsigned long flags;

	if (service->ops->teardown)
		service->ops->teardown(service);
	spin_lock_irqsave(&service->lock, flags);
	service->enabled = false;
	service->torndown = true;
	service->cookie = NULL;
	service->ops = NULL;
	spin_unlock_irqrestore(&service->lock, flags);
}

static inline void afk_service_publish(struct apple_epic_service *service)
{
	unsigned long flags;

	spin_lock_irqsave(&service->lock, flags);
	service->enabled = true;
	spin_unlock_irqrestore(&service->lock, flags);
}

struct afk_ringbuffer_header {
	__le32 bufsz;
	u32 unk;
	u32 _pad1[14];
	__le32 rptr;
	u32 _pad2[15];
	__le32 wptr;
	u32 _pad3[15];
};

struct afk_qe {
#define QE_MAGIC 0x20504f49 // ' POI'
	__le32 magic;
	__le32 size;
	__le32 channel;
	__le32 type;
	u8 data[];
};

static inline bool afk_ring_payload_fits(u32 ptr, u32 bytes, size_t bufsz)
{
	return ptr <= bufsz && sizeof(struct afk_qe) <= bufsz - ptr &&
	       bytes <= bufsz - ptr - sizeof(struct afk_qe);
}

/*
 * Check the receive ring pointers before reading an entry. The consumer
 * only advances rptr in whole blocks of @stride bytes and stops when it
 * meets wptr, so the producer has to keep wptr block aligned and inside
 * the ring as well; wptr == bufsz is never written, it wraps to 0.
 */
static inline bool afk_ring_ptrs_valid(u32 rptr, u32 wptr, size_t bufsz,
				       u32 stride)
{
	return stride && bufsz >= sizeof(struct afk_qe) &&
	       rptr <= bufsz - sizeof(struct afk_qe) && wptr < bufsz &&
	       !(rptr % stride) && !(wptr % stride);
}

/*
 * One RECV notification drains at most as many entries as the ring has
 * blocks, so a producer that keeps moving wptr cannot hold the ordered
 * receive worker. Entries written after this pass come with their own
 * RECV notification.
 */
static inline unsigned int afk_ring_drain_budget(size_t bufsz, u32 stride)
{
	return stride ? bufsz / stride : 1;
}

static inline unsigned int afk_ring_drain(bool (*recv)(void *ctx), void *ctx,
					  unsigned int budget)
{
	unsigned int n = 0;

	while (n < budget && recv(ctx))
		n++;
	return n;
}

struct epic_hdr {
	u8 version;
	__le16 seq;
	u8 _pad;
	__le32 unk;
	__le64 timestamp;
} __attribute__((packed));

struct epic_sub_hdr {
	__le32 length;
	u8 version;
	u8 category;
	__le16 type;
	__le64 timestamp;
	__le16 tag;
	__le16 unk;
	__le32 inline_len;
} __attribute__((packed));

/*
 * H17P compresses the EPIC sub-header from 24 bytes to 8 and drops the
 * separate length/timestamp: the announce for the "system" service puts its
 * name at data + 0x18, not data + 0x28.  An announce carries type 0x11 and a
 * standard-service report carries the usual 0xc0.
 */
struct epic_sub_hdr_h17p {
	u8 type;
	u8 category;
	__le16 flags;
	__le32 tag;
} __packed;

#define EPIC_SUBTYPE_ANNOUNCE_H17P 0x11

struct epic_cmd {
	__le32 retcode;
	__le64 rxbuf;
	__le64 txbuf;
	__le32 rxlen;
	__le32 txlen;
	u8 rxcookie;
	u8 txcookie;
} __attribute__((packed));

struct epic_service_call {
	u8 _pad0[2];
	__le16 group;
	__le32 command;
	__le32 data_len;
#define EPIC_SERVICE_CALL_MAGIC 0x69706378
	__le32 magic;
	u8 _pad1[48];
} __attribute__((packed));
static_assert(sizeof(struct epic_service_call) == 64);

enum epic_type {
	EPIC_TYPE_NOTIFY = 0,
	EPIC_TYPE_COMMAND = 3,
	EPIC_TYPE_REPLY = 4,
	EPIC_TYPE_NOTIFY_ACK = 8,
};

enum epic_category {
	EPIC_CAT_REPORT = 0x00,
	EPIC_CAT_NOTIFY = 0x10,
	EPIC_CAT_REPLY = 0x20,
	EPIC_CAT_COMMAND = 0x30,
};

enum epic_subtype {
	EPIC_SUBTYPE_ANNOUNCE = 0x30,
	EPIC_SUBTYPE_TEARDOWN = 0x32,
	EPIC_SUBTYPE_STD_SERVICE = 0xc0,
};

struct afk_ringbuffer {
	bool ready;
	__le32 *rptr;
	__le32 *wptr;
	void *buf;
	size_t bufsz;
	u32 stride;
};


struct apple_dcp_afkep {
	struct apple_dcp *dcp;

	u32 endpoint;
	/* Set before afk_start; protocol selection is per endpoint. */
	bool wire_v2;
	/* External H17P DPAV only; other endpoints retain owned-DMA commands. */
	bool inline_v2;
	/* Fixed 16-bit interface domain; survives slot and channel reuse. */
	u8 *command_tags;
	struct workqueue_struct *wq;
	struct work_struct service_work;

	struct completion started;
	struct completion stopped;

	void *bfr;
	u16 bfr_tag;
	size_t bfr_size;
	size_t ext26_command_bytes;
	unsigned int ext26_command_count;
	dma_addr_t bfr_dma;

	struct afk_ringbuffer txbfr;
	struct afk_ringbuffer rxbfr;
	/* Private receive entry, owned by the ordered endpoint worker. */
	void *rx_scratch;

	spinlock_t lock;
	u16 qe_seq;
	bool stopping; /* lock: no new receive work after shutdown. */

	const struct apple_epic_service_ops *ops;
	struct apple_epic_service services[AFK_MAX_CHANNEL];
	u32 num_channels;

	struct dentry *debugfs_entry;

	bool match_epic_name;
};

struct apple_dcp_afkep *afk_init(struct apple_dcp *dcp, u32 endpoint,
				 const struct apple_epic_service_ops *ops);
int afk_v2_copy_owned(struct apple_epic_service *service, u64 dva,
		      void *buffer, size_t size, bool to_peer);
int afk_start(struct apple_dcp_afkep *ep);
/* A sampled host boundary, never a remote-producer or DMA fence. */
bool afk_host_idle(struct apple_dcp_afkep *ep);
void afk_shutdown(struct apple_dcp_afkep *ep);
void afk_quiesce(struct apple_dcp_afkep *ep);
int afk_receive_message(struct apple_dcp_afkep *ep, u64 message);
int afk_send_epic(struct apple_dcp_afkep *ep, u32 channel, u16 tag,
		  enum epic_type etype, enum epic_category ecat, u8 stype,
		  const void *payload, size_t payload_len);
int afk_send_command(struct apple_epic_service *service, u8 type,
		     const void *payload, size_t payload_len, void *output,
		     size_t output_len, u32 *retcode);
int afk_send_command_timeout(struct apple_epic_service *service, u8 type,
			     const void *payload, size_t payload_len,
			     void *output, size_t output_len, u32 *retcode,
			     unsigned int timeout_ms);
int afk_service_call(struct apple_epic_service *service, u16 group, u32 command,
		     const void *data, size_t data_len, size_t data_pad,
		     void *output, size_t output_len, size_t output_pad);
int afk_service_call_timeout(struct apple_epic_service *service, u16 group,
			     u32 command, const void *data, size_t data_len,
			     size_t data_pad, void *output, size_t output_len,
			     size_t output_pad, unsigned int timeout_ms);
int afk_service_call_len(struct apple_epic_service *service, u16 group,
			 u32 command, const void *data, size_t data_len,
			 size_t data_pad, void *output, size_t output_len,
			 size_t output_pad, size_t *received);
/*
 * As afk_service_call_timeout(), but the length the reply declares is checked
 * against what arrived and against @output_len + @output_pad, and @received
 * is set to the number of bytes of @output that the reply filled.
 */
int afk_service_call_timeout_len(struct apple_epic_service *service, u16 group,
				 u32 command, const void *data, size_t data_len,
				 size_t data_pad, void *output, size_t output_len,
				 size_t output_pad, unsigned int timeout_ms,
				 size_t *received);

/* Strict variants return only firmware-reported bytes; legacy callers unchanged. */
int afk_send_command_with_reply_len(struct apple_epic_service *service, u8 type,
		const void *payload, size_t payload_len, void *output,
		size_t output_len, u32 *retcode, size_t *reply_len);
int afk_service_call_with_reply_len(struct apple_epic_service *service,
		u16 group, u32 command, const void *data, size_t data_len,
		size_t data_pad, void *output, size_t output_len,
		size_t output_pad, size_t *reply_len);
#endif
