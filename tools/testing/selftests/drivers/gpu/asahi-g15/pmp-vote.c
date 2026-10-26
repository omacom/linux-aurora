// SPDX-License-Identifier: GPL-2.0-only OR MIT
/* Exercise the production PMP vote transaction with failed firmware replies. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#define CONFIG_APPLE_T6030_GPU_PMP_LINK 1
#define IS_ENABLED(x) (x)
#define guard(kind) ignore_lock
#define dev_warn(...) ((void)0)
static int pmp_lock;
static void ignore_lock(void *p) { (void)p; }
struct device { void *of_node; };
static struct device owner;
static struct device *pmp_device = &owner;
static const void *pmp_data = &owner;
static bool t6030, requested;
static int ready, check, power_result[2], report_result[3], powers, reports;
static char calls[16];
static unsigned int used;
static void called(char c) { assert(used < sizeof(calls) - 1); calls[used++] = c; }
static bool of_machine_is_compatible(const char *s) { (void)s; return t6030; }
static bool of_device_is_compatible(void *p, const char *s)
{
	(void)p; (void)s; return t6030;
}
static int apple_pmp_report_wait_supplier_ready(struct device *d, unsigned int ms)
{
	assert(d == &owner && ms == 5000); called('W'); return ready;
}
static int apple_pmp_send_power_command(const void *p, u8 cmd, u16 id, u32 enabled)
{
	assert(p == &owner && cmd == 0x0f && id == 5);
	called(enabled ? 'P' : 'p'); assert(powers < 2); return power_result[powers++];
}
static int apple_pmp_report_check_t6030_gfx(struct device *d)
{
	assert(d == &owner); called('V'); return check;
}
static int apple_pmp_report_set_t6030_gfx(struct device *d, bool enabled)
{
	assert(d == &owner); called(enabled ? 'R' : 'r'); requested = enabled;
	assert(reports < 3); return report_result[reports++];
}
#include "pmp-vote-production.h"
static void reset(void)
{
	pmp_data = &owner; t6030 = true; requested = false; ready = check = 0; powers = reports = used = 0;
	memset(calls, 0, sizeof(calls));
	memset(power_result, 0, sizeof(power_result));
	memset(report_result, 0, sizeof(report_result));
}
static void result(int expected, const char *sequence, bool enabled)
{
	assert(apple_pmp_set_device_power(0x0f, 5, enabled) == expected);
	assert(strcmp(calls, sequence) == 0);
}
int main(void)
{
	reset();
	assert(apple_pmp_set_device_power(0, 5, 1) == -EINVAL);
	assert(apple_pmp_set_device_power(0x0f, 16, 1) == -EINVAL);
	assert(apple_pmp_set_device_power(0x0f, 5, 2) == -EINVAL);
	assert(!used);
	pmp_data = NULL; result(-ENODEV, "", true);
	reset(); ready = -EAGAIN; result(-EAGAIN, "W", true);
	reset(); t6030 = false; result(0, "WP", true);
	reset(); t6030 = false; result(0, "Wp", false);
	reset(); check = -EINVAL; result(-EINVAL, "WV", true);
	reset(); result(0, "WVP", true);
	/* Power-on must not start bandwidth tracking before firmware READY. */
	assert(!reports);
	assert(apple_pmp_t6030_gpu_ready() == 0);
	assert(strcmp(calls, "WVPrR") == 0 && requested);
	reset(); requested = true;
	assert(apple_pmp_t6030_gpu_ready() == 0);
	assert(strcmp(calls, "rR") == 0 && requested);
	reset(); pmp_data = NULL;
	assert(apple_pmp_t6030_gpu_ready() == -ENODEV && !used);
	reset(); report_result[0] = -ETIMEDOUT;
	assert(apple_pmp_t6030_gpu_ready() == -ETIMEDOUT);
	assert(strcmp(calls, "r") == 0);
	reset(); report_result[1] = -ETIMEDOUT;
	assert(apple_pmp_t6030_gpu_ready() == -ETIMEDOUT);
	assert(strcmp(calls, "rR") == 0);
	reset(); result(0, "WVrp", false);
	reset(); power_result[0] = -ETIMEDOUT; result(-ETIMEDOUT, "WVP", true);
	reset(); report_result[0] = -ETIMEDOUT; result(-ETIMEDOUT, "WVrR", false);
	reset(); report_result[0] = -ETIMEDOUT; report_result[1] = -EIO;
	result(-ETIMEDOUT, "WVrR", false);
	reset(); power_result[0] = -EIO; result(-EIO, "WVrpR", false);
	puts("production PMP vote: scope, sequencing, reply errors and rollback: PASS");
}
