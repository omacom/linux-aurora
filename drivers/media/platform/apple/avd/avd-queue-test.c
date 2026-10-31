// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <kunit/test.h>

#include "avd-apple-protocol.h"
#include "avd-queue-internal.h"

static void avd_slots_and_tags(struct kunit *test)
{
	struct avd_frame *pool;
	struct avd_ctx *ctx = (void *)test;
	unsigned int i;

	pool = kunit_kcalloc(test, AVD_MAX_FRAMES, sizeof(*pool), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pool);
	for (i = 0; i < AVD_MAX_FRAMES; i++) {
		pool[i].idx = i;
		KUNIT_EXPECT_EQ(test, avd_slot_reserve(&pool[i], ctx, i + 1), 0);
		KUNIT_EXPECT_EQ(test, pool[i].tag, ((i + 1) << 4) | i);
		KUNIT_EXPECT_EQ(test, pool[i].state, AVD_FRAME_RESERVED);
		KUNIT_EXPECT_EQ(test, avd_slot_reserve(&pool[i], ctx, 100), -EBUSY);
		KUNIT_EXPECT_FALSE(test, avd_slot_complete(&pool[i], pool[i].tag, 0));
		pool[i].state = AVD_FRAME_SUBMITTED;
		KUNIT_EXPECT_FALSE(test, avd_slot_complete(&pool[i], pool[i].tag + 16, 0));
		KUNIT_EXPECT_TRUE(test, avd_slot_complete(&pool[i], pool[i].tag, 0));
		KUNIT_EXPECT_FALSE(test, avd_slot_complete(&pool[i], pool[i].tag, -EIO));
	}
	pool[0].state = AVD_FRAME_FREE;
	KUNIT_EXPECT_EQ(test, avd_slot_reserve(&pool[0], ctx, 0), -EOVERFLOW);
	KUNIT_EXPECT_EQ(test, avd_slot_reserve(&pool[0], ctx, (u64)(U32_MAX >> 4) + 1),
			-EOVERFLOW);
	KUNIT_ASSERT_EQ(test, avd_slot_reserve(&pool[0], ctx, 100), 0);
	pool[0].state = AVD_FRAME_SUBMITTED;
	KUNIT_EXPECT_FALSE(test, avd_slot_complete(&pool[0], 16, 0));
}

static void avd_retirement_order(struct kunit *test)
{
	struct avd_frame *pool;
	struct avd_ctx *a = (void *)test;
	struct avd_ctx *b = (void *)&pool;

	pool = kunit_kcalloc(test, AVD_MAX_FRAMES, sizeof(*pool), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pool);
	KUNIT_ASSERT_EQ(test, avd_slot_reserve(&pool[0], a, 1), 0);
	KUNIT_ASSERT_EQ(test, avd_slot_reserve(&pool[1], b, 2), 0);
	KUNIT_ASSERT_EQ(test, avd_slot_reserve(&pool[2], a, 3), 0);
	pool[0].state = AVD_FRAME_SUBMITTED;
	pool[1].state = AVD_FRAME_SUBMITTED;
	pool[2].state = AVD_FRAME_SUBMITTED;
	KUNIT_ASSERT_TRUE(test, avd_slot_complete(&pool[2], pool[2].tag, 0));
	KUNIT_EXPECT_FALSE(test, avd_slot_first(pool, &pool[2]));
	KUNIT_EXPECT_TRUE(test, avd_slot_first(pool, &pool[1]));
	KUNIT_ASSERT_TRUE(test, avd_slot_complete(&pool[0], pool[0].tag, -ETIMEDOUT));
	KUNIT_EXPECT_TRUE(test, avd_slot_first(pool, &pool[0]));
	pool[0].state = AVD_FRAME_FREE;
	KUNIT_EXPECT_TRUE(test, avd_slot_first(pool, &pool[2]));
	KUNIT_EXPECT_EQ(test, pool[0].status, -ETIMEDOUT);
}

static void avd_watchdog_and_retry(struct kunit *test)
{
	struct avd_frame f = { .state = AVD_FRAME_SUBMITTED, .deadline = ULONG_MAX - 2 };
	unsigned int producer, consumer;

	KUNIT_EXPECT_FALSE(test, avd_slot_expired(&f, ULONG_MAX - 3));
	KUNIT_EXPECT_TRUE(test, avd_slot_expired(&f, ULONG_MAX - 2));
	KUNIT_EXPECT_TRUE(test, avd_slot_expired(&f, 1));
	f.state = AVD_FRAME_DONE;
	KUNIT_EXPECT_FALSE(test, avd_slot_expired(&f, 1));
	f.state = AVD_FRAME_RESERVED;
	KUNIT_EXPECT_TRUE(test, avd_slot_expired(&f, 1));
	for (producer = 0; producer < 8; producer++) {
		for (consumer = 0; consumer < 8; consumer++)
			KUNIT_EXPECT_EQ(test, avd_inbox_full(producer, consumer),
					((producer + 1) & 7) == consumer);
		consumer = (producer + 1) & 7;
		KUNIT_EXPECT_TRUE(test, avd_inbox_full(producer, consumer));
		/* A consumer advance reopens the same pending inbox slot. */
		consumer = (consumer + 1) & 7;
		KUNIT_EXPECT_FALSE(test, avd_inbox_full(producer, consumer));
		KUNIT_EXPECT_EQ(test, avd_apple_notification(producer),
				0x01482cf8U + 0x60 * producer);
	}
}

static void avd_completion_parser(struct kunit *test)
{
	/* Synthetic layout for parser testing; never a claimed Borage wire ABI. */
	struct avd_apple_reply_layout layout = {
		.type_word = 0, .type_mask = 0x1f,
		.client_word = 1, .client_shift = 16, .client_mask = 0x3ff,
		.frame_word = 3, .status_word = 4, .done_type = 8, .verified = true,
	};
	struct avd_apple_reply reply;
	u32 record[AVD_APPLE_RESULT_WORDS] = { 7, 5 << 16, 0, 0x12340 };

	KUNIT_EXPECT_EQ(test, avd_apple_parse_reply(record, 19, &layout, &reply), -EMSGSIZE);
	KUNIT_EXPECT_EQ(test, avd_apple_parse_reply(record, 20, &layout, &reply), -EINPROGRESS);
	record[0] = 8;
	KUNIT_ASSERT_EQ(test, avd_apple_parse_reply(record, 20, &layout, &reply), 0);
	KUNIT_EXPECT_EQ(test, reply.client, 5U);
	KUNIT_EXPECT_EQ(test, reply.frame, 0x12340U);
	KUNIT_EXPECT_EQ(test, reply.status, 0);
	record[4] = 1;
	KUNIT_ASSERT_EQ(test, avd_apple_parse_reply(record, 20, &layout, &reply), 0);
	KUNIT_EXPECT_EQ(test, reply.status, -EIO);
	record[0] = 3;
	KUNIT_EXPECT_EQ(test, avd_apple_parse_reply(record, 20, &layout, &reply), -ENOMSG);
	layout.frame_word = 20;
	KUNIT_EXPECT_EQ(test, avd_apple_parse_reply(record, 20, &layout, &reply), -EINVAL);
	layout.verified = false;
	KUNIT_EXPECT_EQ(test, avd_apple_parse_reply(record, 20, &layout, &reply), -EOPNOTSUPP);
}

static struct kunit_case avd_queue_cases[] = {
	KUNIT_CASE(avd_slots_and_tags),
	KUNIT_CASE(avd_retirement_order),
	KUNIT_CASE(avd_watchdog_and_retry),
	KUNIT_CASE(avd_completion_parser),
	{}
};

static struct kunit_suite avd_queue_suite = {
	.name = "apple-avd-queue",
	.test_cases = avd_queue_cases,
};

kunit_test_suite(avd_queue_suite);
