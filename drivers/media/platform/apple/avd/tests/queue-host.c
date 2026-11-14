// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Exercise the production helpers with deterministic shuffled completions. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../avd-apple-protocol.h"
#include "../avd-queue-internal.h"

int main(void)
{
	struct avd_frame pool[AVD_MAX_FRAMES] = {};
	int clients[3];
	u64 last[3] = {}, seq = 0;
	u32 rng = 0x3851, old_tag[AVD_MAX_FRAMES] = {};
	unsigned int i, step, completed = 0;

	for (i = 0; i < AVD_MAX_FRAMES; i++)
		pool[i].idx = i;
	for (step = 0; step < 100000; step++) {
		struct avd_frame *f;

		rng = rng * 1664525 + 1013904223;
		f = &pool[(rng >> 16) & 15];
		if (f->state == AVD_FRAME_FREE) {
			struct avd_ctx *ctx = (void *)&clients[(rng >> 8) % 3];

			assert(!avd_slot_reserve(f, ctx, ++seq));
			assert(!avd_slot_complete(f, f->tag, 0));
			f->deadline = step + 100;
			f->state = AVD_FRAME_SUBMITTED;
			assert(!avd_slot_complete(f, old_tag[f->idx], 0));
			old_tag[f->idx] = f->tag;
		} else if (f->state == AVD_FRAME_SUBMITTED) {
			assert(avd_slot_complete(f, f->tag, (rng & 1) ? -EIO : 0));
			assert(!avd_slot_complete(f, f->tag, 0));
		}
		for (i = 0; i < AVD_MAX_FRAMES; i++) {
			unsigned int c;

			f = &pool[i];
			if (f->state != AVD_FRAME_DONE || !avd_slot_first(pool, f))
				continue;
			for (c = 0; c < 3; c++)
				if (f->ctx == (void *)&clients[c])
					break;
			assert(c < 3 && f->seq > last[c]);
			last[c] = f->seq;
			f->state = AVD_FRAME_FREE;
			completed++;
		}
	}
	for (i = 0; i < 8; i++) {
		assert(avd_inbox_full(i, (i + 1) & 7));
		assert(!avd_inbox_full(i, (i + 2) & 7));
		assert(avd_apple_notification(i) == 0x01482cf8U + 0x60 * i);
	}
	{
		struct avd_apple_reply_layout layout = {
			.type_word = 0, .type_mask = 31, .client_word = 1,
			.client_mask = 1023, .frame_word = 3, .status_word = 4,
			.done_type = 8, .verified = true,
		};
		struct avd_apple_reply reply;
		u32 record[20] = { 7, 9, 0, 1234 };

		assert(avd_apple_parse_reply(record, 20, &layout, &reply) == -EINPROGRESS);
		record[0] = 8;
		assert(!avd_apple_parse_reply(record, 20, &layout, &reply));
		assert(reply.frame == 1234 && reply.client == 9 && !reply.status);
		record[4] = 1;
		assert(!avd_apple_parse_reply(record, 20, &layout, &reply));
		assert(reply.status == -EIO);
		assert(avd_apple_parse_reply(record, 19, &layout, &reply) == -EMSGSIZE);
	}
	assert(completed > 1000);
	{
		struct avd_frame ordered[AVD_MAX_FRAMES] = {};
		struct avd_ctx *a = (void *)&clients[0], *b = (void *)&clients[1];

		ordered[1].idx = 1;
		ordered[2].idx = 2;
		assert(!avd_slot_reserve(&ordered[0], a, 1));
		assert(!avd_slot_reserve(&ordered[1], a, 2));
		assert(!avd_slot_reserve(&ordered[2], b, 3));
		assert(avd_slot_submit_first(ordered, &ordered[0]));
		assert(!avd_slot_submit_first(ordered, &ordered[1]));
		assert(avd_slot_submit_first(ordered, &ordered[2]));
		ordered[0].state = AVD_FRAME_SUBMITTED;
		assert(avd_slot_submit_first(ordered, &ordered[1]));
		avd_slot_submit_result(&ordered[0], -EBUSY);
		assert(!avd_slot_submit_first(ordered, &ordered[1]));
		ordered[0].state = AVD_FRAME_DONE;
		assert(avd_slot_submit_first(ordered, &ordered[1]));
	}
	printf("PASS: 100000 shuffled operations, %u ordered retirements\n", completed);
	return 0;
}
