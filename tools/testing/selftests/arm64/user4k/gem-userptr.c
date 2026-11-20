// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM_IOCTL_ASAHI_GEM_USERPTR in a 4K process on a 16K kernel: a native page
 * backs the object only where the process maps all of it, in order. No GPU
 * work is submitted. A 16K process runs the test in a 4K exec of itself.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../../../../../include/uapi/linux/prctl.h"
#include <sys/auxv.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <unistd.h>
#include <drm/asahi_drm.h>
#include "../../kselftest.h"

#define NATIVE 16384UL
#define LEAF 4096UL
#define LEN (2 * NATIVE)

static long vm_pin_kb(void)
{
	char line[256];
	long kb = -1;
	FILE *f = fopen("/proc/self/status", "r");

	if (!f)
		return -1;
	while (fgets(line, sizeof(line), f))
		if (sscanf(line, "VmPin: %ld kB", &kb) == 1)
			break;
	fclose(f);
	return kb;
}

static char *map(size_t len, int prot)
{
	char *p = mmap(NULL, len, prot, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

	if (p == MAP_FAILED)
		ksft_exit_fail_msg("mmap: %s\n", strerror(errno));
	return p;
}

/* A written, native-aligned LEN range: each native page mapped whole, in order. */
static char *range(void)
{
	char *p = map(LEN + NATIVE, PROT_READ | PROT_WRITE);

	p = (char *)(((uintptr_t)p + NATIVE - 1) & ~(NATIVE - 1));
	memset(p, 0x5a, LEN);
	return p;
}

static void move(char *from, char *to)
{
	if (mremap(from, LEAF, LEAF, MREMAP_MAYMOVE | MREMAP_FIXED, to) != to)
		ksft_exit_fail_msg("mremap: %s\n", strerror(errno));
}

/* GEM_USERPTR of the range fails with @err, or pins LEN until closed. */
static void userptr(int fd, char *p, int err, const char *name)
{
	struct drm_asahi_gem_userptr args = {
		.userptr = (uintptr_t)p, .size = LEN,
	};
	long pin0 = vm_pin_kb(), pinned, after;
	int ret = ioctl(fd, DRM_IOCTL_ASAHI_GEM_USERPTR, &args) ? errno : 0;

	pinned = vm_pin_kb() - pin0;
	if (!ret) {
		struct drm_gem_close close_arg = { .handle = args.handle };

		ioctl(fd, DRM_IOCTL_GEM_CLOSE, &close_arg);
	}
	after = vm_pin_kb() - pin0;
	ksft_print_msg("%s: errno %d, VmPin +%ld kB, +%ld kB after close\n",
		       name, ret, pinned, after);
	ksft_test_result(ret == err && pinned == (err ? 0 : (long)LEN >> 10) &&
			 !after, "%s\n", name);
}

int main(int argc, char **argv)
{
	char name[16] = "";
	struct drm_version version = {
		.name = name, .name_len = sizeof(name) - 1,
	};
	struct drm_asahi_params_global params = {};
	struct drm_asahi_get_params get = {
		.pointer = (uintptr_t)&params, .size = sizeof(params),
	};
	char *p, *q;
	int fd;

	if (getauxval(AT_PAGESZ) == NATIVE && argc == 1) {
		if (prctl(PR_AURORA_SET_EXEC_PAGE_SIZE, LEAF, 0UL, 0UL, 0UL))
			ksft_exit_skip("no 4K processes: %s\n", strerror(errno));
		execl("/proc/self/exe", argv[0], "--4k", NULL);
		ksft_exit_fail_msg("exec: %s\n", strerror(errno));
	}
	ksft_print_header();
	if (getauxval(AT_PAGESZ) != LEAF)
		ksft_exit_skip("not a 4K process\n");
	fd = open("/dev/dri/renderD128", O_RDWR | O_CLOEXEC);
	/* "asahi" is the driver's name before it became "aurora". */
	if (fd < 0 || ioctl(fd, DRM_IOCTL_VERSION, &version) ||
	    (strcmp(name, "aurora") && strcmp(name, "asahi")))
		ksft_exit_skip("renderD128 is not an asahi render node\n");
	if (ioctl(fd, DRM_IOCTL_ASAHI_GET_PARAMS, &get) ||
	    !(params.features & DRM_ASAHI_FEATURE_USERPTR))
		ksft_exit_skip("no DRM_ASAHI_FEATURE_USERPTR\n");
	ksft_set_plan(3);

	userptr(fd, range(), 0, "native pages mapped whole and in order");

	p = range();
	q = map(LEAF, PROT_READ | PROT_WRITE);
	memset(q, 0xa5, LEAF);
	move(q, p + NATIVE + LEAF);
	userptr(fd, p, EFAULT, "a 4K page from elsewhere in a native page");

	/* Swap two 4K pages of one native page: all of it, out of order. */
	p = range();
	q = map(LEAF, PROT_NONE);
	move(p + NATIVE + LEAF, q);
	move(p + NATIVE + 2 * LEAF, p + NATIVE + LEAF);
	move(q, p + NATIVE + 2 * LEAF);
	userptr(fd, p, EFAULT, "a native page mapped out of order");

	close(fd);
	ksft_finished();
}
