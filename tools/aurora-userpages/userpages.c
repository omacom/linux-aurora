// SPDX-License-Identifier: GPL-2.0-only
#define _GNU_SOURCE
#include "../../include/uapi/linux/prctl.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/auxv.h>
#include <sys/prctl.h>
#include <unistd.h>

static int usage(const char *name, int status)
{
	fprintf(status ? stderr : stdout,
		"Usage: %s [--once] {native|4k|16k|64k} COMMAND [ARG ...]\n"
		"       %s --status\n"
		"Select the userspace page size for this program and its descendants.\n",
		name, name);
	return status;
}

/*
 * The kernel refuses every smaller size with EINVAL when it was booted with
 * user4k=off or built without the per-process granule support.
 */
static int off_hint(unsigned long size, int error)
{
	if (error == EINVAL && size && size < getauxval(AT_PAGESZ))
		fputs("Smaller page sizes are off: booted with user4k=off, or not built in.\n",
		      stderr);
	return 1;
}

int main(int argc, char **argv)
{
	unsigned long size;
	int once = argc > 1 && !strcmp(argv[1], "--once");
	int first = once ? 2 : 1;

	if (argc == 2 && !strcmp(argv[1], "--help"))
		return usage(argv[0], 0);
	if (argc == 2 && !strcmp(argv[1], "--status")) {
		int pending = prctl(PR_AURORA_GET_EXEC_PAGE_SIZE, 0UL, 0UL, 0UL, 0UL);
		int error = errno;

		printf("Current process page size: %lu bytes\n", getauxval(AT_PAGESZ));
		if (pending < 0) {
			fprintf(stderr, "Page-size selection unavailable: %s\n", strerror(error));
			return 1;
		}
		int inherited = prctl(PR_AURORA_GET_DEFAULT_PAGE_SIZE, 0UL, 0UL, 0UL, 0UL);

		if (inherited < 0) {
			perror("Cannot read inherited page-size preference");
			return 1;
		}
		if (inherited)
			printf("Inherited exec preference: %d bytes\n", inherited);
		else
			puts("Inherited exec preference: native default");
		if (pending)
			printf("Next exec page size: %d bytes\n", pending);
		else
			puts("Next exec page size: inherited preference");
		return 0;
	}
	if (argc < first + 2)
		return usage(argv[0], 2);
	if (!strcmp(argv[first], "native"))
		size = 0;
	else if (!strcmp(argv[first], "4k"))
		size = 4096;
	else if (!strcmp(argv[first], "16k"))
		size = 16384;
	else if (!strcmp(argv[first], "64k"))
		size = 65536;
	else
		return usage(argv[0], 2);

	if (!once && prctl(PR_AURORA_SET_DEFAULT_PAGE_SIZE, size, 0UL, 0UL, 0UL)) {
		int error = errno;

		fprintf(stderr, "Cannot set inherited %s pages: %s\n",
			argv[first], strerror(error));
		return off_hint(size, error);
	}
	if (prctl(PR_AURORA_SET_EXEC_PAGE_SIZE, size, 0UL, 0UL, 0UL)) {
		int error = errno;

		fprintf(stderr, "Cannot select %s userspace pages: %s\n",
			argv[first], strerror(error));
		return off_hint(size, error);
	}
	execvp(argv[first + 1], &argv[first + 1]);
	int error = errno;

	fprintf(stderr, "Cannot execute %s: %s\n", argv[first + 1], strerror(error));
	return error == ENOENT ? 127 : 126;
}
