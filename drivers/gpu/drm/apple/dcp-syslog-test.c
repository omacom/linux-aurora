// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Copyright 2026 Ryan Murray */

#include <kunit/test.h>
#include <linux/module.h>

#include "dcp-syslog.h"

static void dcp_syslog_transition_test(struct kunit *test)
{
	static const struct {
		const char *context;
		const char *message;
	} records[] = {
		{ "UPPipeDCP_H17P.cpp:2096", "set_run_mode_safe: deferring: 4 -> 1" },
		{ "UPPipeDCP_H17P.cpp:2096", "set_run_mode_safe: deferring: 2 -> 4" },
		{ "UPPipeDCP_H17P.cpp:2014",
		  "ready_for_run_mode_change: initiating deferred run mode change: 4 -> 1" },
		{ "UPPipeDCP_H17P.cpp:2014",
		  "ready_for_run_mode_change: initiating deferred run mode change: 2 -> 4" },
		{ "itorContext_v4.cpp:1324",
		  "SMC sensor TAOC is an invalid temp -127.0, using 2DVT fallback values" },
	};
	char message[128];
	int i;

	for (i = 0; i < ARRAY_SIZE(records); i++) {
		const char *context = records[i].context;
		const char *text = records[i].message;

		KUNIT_EXPECT_TRUE(test, dcp_h17p_syslog_is_debug(context, text));
		/* Never hide diagnostics appended to a recognized transition. */
		snprintf(message, sizeof(message), "%s: failed", records[i].message);
		KUNIT_EXPECT_FALSE(test, dcp_h17p_syslog_is_debug(records[i].context, message));
		KUNIT_EXPECT_FALSE(test,
				   dcp_h17p_syslog_is_debug("other.cpp:2096", records[i].message));
	}
}

static void dcp_syslog_unknown_test(struct kunit *test)
{
	static const struct {
		const char *context;
		const char *message;
	} records[] = {
		{ "", "" },
		{ "UPPipeDCP_H17P.cpp:2096", "set_run_mode_safe: deferring: 4 -> 0" },
		{ "UPPipeDCP_H17P.cpp:2014", "ready_for_run_mode_change: timeout" },
		{ "UPPipeDCP_H17P.cpp:2096", "set_run_mode_safe: error -110" },
		{ "UPPipeDCP_H17P.cpp:2097", "set_run_mode_safe: deferring: 4 -> 1" },
		/* A different sensor, value or verdict is news. */
		{ "itorContext_v4.cpp:1324",
		  "SMC sensor TAOJ is an invalid temp -127.0, using 2DVT fallback values" },
		{ "itorContext_v4.cpp:1324",
		  "SMC sensor TAOC is an invalid temp -40.0, using 2DVT fallback values" },
		{ "itorContext_v4.cpp:1324",
		  "2DVT thermal sensors valid, using computed virtual temperatures" },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(records); i++) {
		const char *context = records[i].context;
		const char *text = records[i].message;

		KUNIT_EXPECT_FALSE(test, dcp_h17p_syslog_is_debug(context, text));
	}
}

static struct kunit_case dcp_syslog_cases[] = {
	KUNIT_CASE(dcp_syslog_transition_test),
	KUNIT_CASE(dcp_syslog_unknown_test),
	{}
};

static struct kunit_suite dcp_syslog_suite = {
	.name = "apple-dcp-syslog",
	.test_cases = dcp_syslog_cases,
};

kunit_test_suite(dcp_syslog_suite);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple DCP firmware syslog policy tests");
