/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef APPLE_CPU_CONTEXT_LAB_POLICY_H
#define APPLE_CPU_CONTEXT_LAB_POLICY_H

enum lab_phase {
	LAB_IDLE,
	LAB_GO,
	LAB_STOPPING,
	LAB_RETURNED,
	LAB_ABORT,
};

/* Once the finisher committed, the reset core owns the delivery field. */
static inline int apple_context_lab_may_restore_after_cancel(int phase, int started)
{
	return !started && (phase == LAB_IDLE || phase == LAB_GO);
}

/* Keep the physical start gate usable by a host test without kernel headers. */
static inline int apple_context_lab_may_start(unsigned int status_before,
					      unsigned int status_after, int stopping,
					      int already_started, unsigned long long elapsed_us)
{
	return (status_before & 0xff) && !(status_after & 0xff) &&
	       stopping && !already_started && elapsed_us < 50000;
}

#endif
