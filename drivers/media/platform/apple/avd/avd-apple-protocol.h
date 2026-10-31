/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef AVD_APPLE_PROTOCOL_H_
#define AVD_APPLE_PROTOCOL_H_

#include <linux/errno.h>
#include <linux/types.h>

#include "avd-queue.h"

#define AVD_APPLE_INBOX_BASE 0x01482cf8
#define AVD_APPLE_COMMAND_SIZE 0x60
#define AVD_APPLE_RESULT_WORDS 20
#define AVD_APPLE_SHADOW_SIZE 64

/*
 * [C] Expected address notification, from the MIT V3 trace_avd.py precedent.
 * Keep this rule in one place for correction by the Borage capture.
 */
#define AVD_APPLE_NOTIFY_BASE AVD_APPLE_INBOX_BASE

/* SPEC 5.1: only these session opcodes have proved builders. */
static inline int
avd_apple_session_command(u32 cmd[24], u32 client, u32 opcode,
			  u32 property, u32 value)
{
	unsigned int i;

	if (client >= 1024 || (opcode != 0 && opcode != 2 && opcode != 5 && opcode != 6))
		return -EINVAL;
	for (i = 0; i < 24; i++)
		cmd[i] = 0;
	cmd[0] = opcode | (client << 16);
	if (opcode == 6) {
		cmd[1] = property;
		cmd[2] = value;
	}
	return 0;
}

static inline int avd_apple_decode_prefix(struct avd_frame *f, u32 client,
					  u32 selector, u32 work_sequence)
{
	unsigned int i;

	if (f->idx >= AVD_MAX_FRAMES || client >= 1024 || selector >= 16 || work_sequence >= 4)
		return -EINVAL;
	for (i = 0; i < 24; i++)
		f->cmd[i] = 0;
	f->cmd[0] = 1 | (selector << 10) | (client << 22);
	f->cmd[1] = f->idx | (work_sequence << 16);
	f->cmd[3] = f->frame_num;
	return 0;
}

struct avd_apple_reply_layout {
	u8 type_word;
	u8 type_shift;
	u32 type_mask;
	u8 client_word;
	u8 client_shift;
	u32 client_mask;
	u8 frame_word;
	u8 status_word;
	u32 done_type;
	bool verified;
};

struct avd_apple_reply {
	u32 type;
	u32 client;
	u32 frame;
	int status;
};

static inline int
avd_apple_parse_reply(const u32 *record, size_t words,
		      const struct avd_apple_reply_layout *layout,
		      struct avd_apple_reply *reply)
{
	if (words != AVD_APPLE_RESULT_WORDS)
		return -EMSGSIZE;
	if (!layout->verified)
		return -EOPNOTSUPP;
	if (layout->type_word >= words || layout->client_word >= words ||
	    layout->frame_word >= words || layout->status_word >= words ||
	    layout->type_shift >= 32 || layout->client_shift >= 32 ||
	    !layout->type_mask || !layout->client_mask || layout->done_type == 7)
		return -EINVAL;
	reply->type = (record[layout->type_word] >> layout->type_shift) & layout->type_mask;
	reply->client = (record[layout->client_word] >> layout->client_shift) & layout->client_mask;
	reply->frame = record[layout->frame_word];
	/* SPEC 4.3: type 7 is parsing/VP done, never final buffer ownership. */
	if (reply->type == 7)
		return -EINPROGRESS;
	if (reply->type != layout->done_type)
		return -ENOMSG;
	reply->status = record[layout->status_word] ? -EIO : 0;
	return 0;
}

static inline u32 avd_apple_notification(unsigned int slot)
{
	return AVD_APPLE_NOTIFY_BASE + AVD_APPLE_COMMAND_SIZE * slot;
}

struct avd_apple_hw {
	const char *fw_name;
	/* All offsets below are relative to a DT-owned "apple" resource. */
	u32 aperture_origin;
	u32 imem;
	u32 imem_size;
	u32 version_offset;
	u32 itcm_start;
	u32 itcm_end;
	u32 itcm_clear;
	u32 itcm_clear_value;
	u32 itcm_enable;
	u32 itcm_enable_value;
	u32 link_setup;
	u32 link_setup_value;
	u32 link_state;
	u32 link_up;
	u32 link_up_value;
	u32 ready;
	u32 notify;
	u32 receive;
	u32 receive_status;
	u32 receive_pending;
	u32 receive_ack;
	u32 receive_ack_value;
	u32 inbox_consumer;
	u32 result_base;
	struct avd_apple_reply_layout reply;
	/* Match-data fact, never a module parameter or a userspace setting. */
	bool verified;
};

extern const struct avd_apple_hw avd_borage_hw;
extern const u8 avd_borage_config[64];

#endif /* AVD_APPLE_PROTOCOL_H_ */
