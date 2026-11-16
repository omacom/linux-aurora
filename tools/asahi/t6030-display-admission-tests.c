// SPDX-License-Identifier: GPL-2.0-only
#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#define EPROBE_DEFER 517
#define DL_FLAG_AUTOREMOVE_CONSUMER 1
#define EXPORT_SYMBOL_GPL(x)
struct device_node {
	const char *compatible;
};
struct device {
	struct device_node *of_node;
	atomic_int refs;
};
struct device_link {
	int unused;
};
static pthread_mutex_t apple_g15_display_lock = PTHREAD_MUTEX_INITIALIZER;
static struct device *apple_t6030_display;
static struct device *apple_t6031_display;
static struct device *apple_t6034_display;
static struct device_node display_node = { "apple,t6030-display-subsystem" },
			  gpu_node = { "apple,agx-t6030" },
			  other_node = { "apple,display-subsystem" },
			  t6031_display_node = { "apple,t6031-display-subsystem" },
			  t6034_display_node = { "apple,t6034-display-subsystem" };
static struct device display, duplicate, gpu, t6031_display, t6034_display;
static const char *machine = "apple,t6030";
static int pmp_calls, link_calls, pmp_result;
static bool link_ok = true;
static atomic_bool pause_link, link_entered, release_link, withdrew;
struct lock_guard {
	pthread_mutex_t *lock;
	bool once;
};
static _Thread_local bool locked;
static struct lock_guard lock_take(pthread_mutex_t *lock)
{
	assert(!locked);
	assert(!pthread_mutex_lock(lock));
	locked = true;
	return (struct lock_guard){ lock, true };
}
static void lock_drop(struct lock_guard *g)
{
	assert(locked);
	locked = false;
	assert(!pthread_mutex_unlock(g->lock));
}
#define guard(kind)                   \
	struct lock_guard local_guard \
		__attribute__((cleanup(lock_drop))) = lock_take
#define scoped_guard(kind, lock)                                    \
	for (struct lock_guard scoped                               \
	     __attribute__((cleanup(lock_drop))) = lock_take(lock); \
	     scoped.once; scoped.once = false)
static bool of_device_is_compatible(const struct device_node *n,
				    const char *name)
{
	return n && !strcmp(n->compatible, name);
}
static bool of_machine_is_compatible(const char *name)
{
	return !strcmp(machine, name);
}
static bool asahi_g15_board(void)
{
	return of_machine_is_compatible("apple,t6030");
}
static struct device *get_device(struct device *dev)
{
	assert(locked);
	atomic_fetch_add(&dev->refs, 1);
	return dev;
}
static void put_device(struct device *dev)
{
	assert(!locked);
	if (dev)
		assert(atomic_fetch_sub(&dev->refs, 1) > 1);
}
static struct device_link host_link;
static struct device_link *device_link_add(struct device *consumer,
					   struct device *supplier, int flags)
{
	assert(locked && consumer == &gpu && supplier == &display &&
	       atomic_load(&display.refs) == 2);
	assert(flags == DL_FLAG_AUTOREMOVE_CONSUMER);
	link_calls++;
	if (atomic_load(&pause_link)) {
		atomic_store(&link_entered, true);
		while (!atomic_load(&release_link))
			sched_yield();
	}
	return link_ok ? &host_link : NULL;
}
static int apple_pmp_link_device(struct device *dev)
{
	assert(!locked && dev == &gpu && link_calls > 0);
	pmp_calls++;
	return pmp_result;
}
#include "production.h"
static void reset(void)
{
	assert(!apple_t6030_display && !apple_t6031_display &&
	       !apple_t6034_display && !locked);
	display = (struct device){ .of_node = &display_node, .refs = 1 };
	duplicate = (struct device){ .of_node = &display_node, .refs = 1 };
	gpu = (struct device){ .of_node = &gpu_node, .refs = 1 };
	pmp_calls = link_calls = pmp_result = 0;
	link_ok = true;
	machine = "apple,t6030";
	atomic_store(&pause_link, false);
	atomic_store(&link_entered, false);
	atomic_store(&release_link, false);
	atomic_store(&withdrew, false);
}
static void *admit_thread(void *unused)
{
	(void)unused;
	assert(asahi_g15_pmp_prepare(&gpu) == 0);
	return NULL;
}
static void *withdraw_thread(void *unused)
{
	(void)unused;
	apple_g15_display_withdraw(&display);
	atomic_store(&withdrew, true);
	return NULL;
}
int main(void)
{
	unsigned int cases = 0;
	(void)get_device;
	(void)put_device;
	reset();
	assert(asahi_g15_pmp_prepare(&gpu) == -EPROBE_DEFER && !pmp_calls);
	cases++;
	/* Component platform bind alone never calls this publication. Retry succeeds only afterward. */
	assert(apple_g15_display_publish(&display) == 0 &&
	       atomic_load(&display.refs) == 2);
	cases++;
	assert(asahi_g15_pmp_prepare(&gpu) == 0 && pmp_calls == 1);
	cases++;
	assert(apple_g15_display_publish(&display) == 0 &&
	       atomic_load(&display.refs) == 2);
	cases++;
	assert(apple_g15_display_publish(&duplicate) == -EBUSY &&
	       apple_t6030_display == &display);
	cases++;
	apple_g15_display_withdraw(&duplicate);
	assert(apple_t6030_display == &display &&
	       atomic_load(&display.refs) == 2);
	cases++;
	link_ok = false;
	assert(asahi_g15_pmp_prepare(&gpu) == -EINVAL && pmp_calls == 1);
	cases++;
	link_ok = true;
	pmp_result = -EPROBE_DEFER;
	assert(asahi_g15_pmp_prepare(&gpu) == -EPROBE_DEFER && pmp_calls == 2);
	cases++;
	machine = "apple,t6031";
	assert(apple_drm_t6030_link_device(&gpu) == -ENODEV);
	cases++;
	machine = "apple,t6030";
	gpu.of_node = &other_node;
	assert(apple_drm_t6030_link_device(&gpu) == -ENODEV);
	cases++;
	gpu.of_node = &gpu_node;
	apple_g15_display_withdraw(&display);
	assert(atomic_load(&display.refs) == 1 && !apple_t6030_display);
	cases++;
	assert(asahi_g15_pmp_prepare(&gpu) == -EPROBE_DEFER && pmp_calls == 2);
	cases++;
	display.of_node = &other_node;
	assert(apple_g15_display_publish(&display) == 0 &&
	       !apple_t6030_display && !apple_t6031_display && !apple_t6034_display &&
	       atomic_load(&display.refs) == 1);
	cases++;
	/* The T6031 display has its own slot: it never satisfies T6030 admission. */
	t6031_display = (struct device){ .of_node = &t6031_display_node, .refs = 1 };
	assert(apple_g15_display_publish(&t6031_display) == 0 &&
	       apple_t6031_display == &t6031_display && !apple_t6030_display &&
	       atomic_load(&t6031_display.refs) == 2);
	assert(asahi_g15_pmp_prepare(&gpu) == -EPROBE_DEFER && pmp_calls == 2);
	/* T6034 can coexist with T6031 without satisfying T6030 admission. */
	t6034_display = (struct device){ .of_node = &t6034_display_node, .refs = 1 };
	assert(apple_g15_display_publish(&t6034_display) == 0 &&
	       apple_t6034_display == &t6034_display &&
	       apple_t6031_display == &t6031_display && !apple_t6030_display &&
	       atomic_load(&t6034_display.refs) == 2);
	assert(asahi_g15_pmp_prepare(&gpu) == -EPROBE_DEFER && pmp_calls == 2);
	cases++;
	apple_g15_display_withdraw(&t6034_display);
	assert(!apple_t6034_display && apple_t6031_display == &t6031_display &&
	       atomic_load(&t6034_display.refs) == 1 &&
	       atomic_load(&t6031_display.refs) == 2);
	cases++;
	apple_g15_display_withdraw(&t6031_display);
	assert(!apple_t6031_display && atomic_load(&t6031_display.refs) == 1);
	cases++;
	reset();
	assert(!apple_g15_display_publish(&display));
	atomic_store(&pause_link, true);
	pthread_t admit, withdraw;
	assert(!pthread_create(&admit, NULL, admit_thread, NULL));
	while (!atomic_load(&link_entered))
		sched_yield();
	assert(!pthread_create(&withdraw, NULL, withdraw_thread, NULL));
	usleep(10000);
	assert(!atomic_load(&withdrew) && atomic_load(&display.refs) == 2);
	atomic_store(&release_link, true);
	assert(!pthread_join(admit, NULL) && !pthread_join(withdraw, NULL));
	assert(atomic_load(&withdrew) && atomic_load(&display.refs) == 1 &&
	       !apple_t6030_display);
	cases++;
	printf("PASS: %u actual display publisher/admission controls, including concurrent link/withdraw\n",
	       cases);
	return 0;
}
