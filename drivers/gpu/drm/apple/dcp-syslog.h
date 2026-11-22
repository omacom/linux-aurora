/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/* Copyright 2026 Ryan Murray */

#ifndef __APPLE_DCP_SYSLOG_H__
#define __APPLE_DCP_SYSLOG_H__

#include <linux/string.h>
#include <linux/types.h>

/*
 * Routine H17P pipe transitions, including the deferred transition's start.
 * Match the entire record: an error mentioning the same function must remain
 * visible. Unknown contexts, states and trailing diagnostics stay at info.
 */
static inline bool dcp_h17p_syslog_is_debug(const char *context, const char *message)
{
	static const char deferred_4_1[] =
		"ready_for_run_mode_change: initiating deferred run mode change: 4 -> 1";
	static const char deferred_2_4[] =
		"ready_for_run_mode_change: initiating deferred run mode change: 2 -> 4";

	if (!strcmp(context, "UPPipeDCP_H17P.cpp:2096"))
		return !strcmp(message, "set_run_mode_safe: deferring: 4 -> 1") ||
		       !strcmp(message, "set_run_mode_safe: deferring: 2 -> 4");
	if (!strcmp(context, "UPPipeDCP_H17P.cpp:2014"))
		return !strcmp(message, deferred_4_1) || !strcmp(message, deferred_2_4);
	/*
	 * The 27.x panel temperature monitor (IOMFBTemperatureMonitorContext_v4)
	 * reads its sensor keys from the SMC once a second and falls back to the
	 * default 2D virtual temperature grid while TAOC holds the SMC's -127.0
	 * "no sample" sentinel (the AOP feeds TAOC/TAOJ/TAOP in macOS; nothing
	 * does under Linux).  Match the exact record so another sensor or value
	 * stays visible.
	 */
	if (!strcmp(context, "itorContext_v4.cpp:1324"))
		return !strcmp(message, "SMC sensor TAOC is an invalid temp -127.0, "
					"using 2DVT fallback values");
	return false;
}

#endif /* __APPLE_DCP_SYSLOG_H__ */
