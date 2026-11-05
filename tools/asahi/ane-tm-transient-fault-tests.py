#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run the production ANE task-manager completion poll against a simulated DART fault latch."""
import argparse
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = 'drivers/accel/ane/ane_tm.c'

HARNESS = r'''
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef uint32_t u32;
typedef uint64_t u64;
typedef int64_t ktime_t;
#define ANE_DART_SCRATCH_MAX 4
#define THIS_MODULE 0
#define TQ_NID1(qid) 0
#define TQ_STATUS(qid) 1
#define lockdep_assert_held(x) ((void)0)
#define wmb() ((void)0)
#define rmb() ((void)0)
#define dev_err(...) ((void)0)
#define __module_get(x) ((void)0)
#define atomic_xchg(ptr, value) (*(ptr) = (value), 0)
#define read_poll_timeout(op, status, cond, delay, timeout, sleep, ...) \
	((status) = op(__VA_ARGS__), 0)

struct ane_dart_scratch { int unused; };
struct ane_device {
	int engine_lock, wedged, dev, tm_retention;
	struct ane_dart_scratch scratch[ANE_DART_SCRATCH_MAX];
	int scratch_pages;
};
struct ane_request { int qid, nid; };

static int latch_reads, scenario, recoveries, drains, released_pages;

static __attribute__((unused)) ktime_t ktime_get(void) { return 0; }
static __attribute__((unused)) ktime_t ktime_add_ms(ktime_t t, int ms) { return t + ms; }
static __attribute__((unused)) int ktime_compare(ktime_t a, ktime_t b) { return a < b ? -1 : a > b; }
static bool ane_dart_faulted(struct ane_device *ane, u64 *iova, u32 *status)
{
	int n = latch_reads++;
	(void)ane;
	if (iova) *iova = 0x1000;
	if (status) *status = 1;
	switch (scenario) {
	case 0: return false;
	case 1: return n == 0;
	case 2: return n < 3;
	case 3: return n < 2;
	case 6: return (n % 2 == 1) && n <= 9;
	default: return n < 100;
	}
}
static int ane_tm_collect_events(struct ane_device *ane, struct ane_request *req, u32 *finished)
{
	(void)ane;
	if (!req)
		return 0;
	*finished = 3;
	return 1;
}
static void ane_dart_mask(struct ane_device *ane) { (void)ane; }
static void ane_dart_unmask(struct ane_device *ane) { (void)ane; }
static void ane_tm_push_tq(struct ane_device *ane, struct ane_request *req) { (void)ane; (void)req; }
static int ane_dart_drain_fault(struct ane_device *ane, u64 iova, struct ane_dart_scratch *s, int *pages)
{
	(void)iova; (void)s;
	drains++;
	if (scenario == 4 && drains == 2)
		return -ENOMEM;
	ane->scratch_pages = ++*pages;
	return 0;
}
static void ane_dart_release_scratch(struct ane_device *ane, struct ane_dart_scratch *s, int pages)
{
	(void)ane; (void)s;
	released_pages += pages;
}
static int ane_tm_recover(struct ane_device *ane) { (void)ane; recoveries++; return -EIO; }
static u32 tq_read32(struct ane_device *ane, int reg) { (void)ane; (void)reg; return 4 << 8; }
static void tq_write32(struct ane_device *ane, int reg, u32 value) { (void)ane; (void)reg; (void)value; }
'''

MAIN = r'''
static int run(int which, struct ane_device *dev)
{
	struct ane_request req = {.qid = 4, .nid = 4};

	*dev = (struct ane_device){0};
	scenario = which;
	latch_reads = recoveries = drains = released_pages = 0;
	return ane_tm_execute(dev, &req);
}

#define CHECK(cond, ...) do { if (!(cond)) { fprintf(stderr, __VA_ARGS__); failures++; } } while (0)

int main(void)
{
	struct ane_device dev;
	int result, failures = 0;

	result = run(0, &dev);
	CHECK(result == 0 && !recoveries, "normal completion: rc=%d recoveries=%d\n", result, recoveries);

	result = run(1, &dev);
	CHECK(result == -EIO && recoveries == 1,
	      "fault gone before the latch read: rc=%d recoveries=%d\n", result, recoveries);

	result = run(3, &dev);
	CHECK(result == -EIO && !recoveries && released_pages == 1 && !dev.scratch_pages,
	      "drained fault: rc=%d recoveries=%d released=%d pages=%d\n",
	      result, recoveries, released_pages, dev.scratch_pages);

	result = run(2, &dev);
	CHECK(result == -EIO && recoveries == 1 && !released_pages && dev.scratch_pages == 1,
	      "fault seen by the last poll: rc=%d recoveries=%d released=%d pages=%d\n",
	      result, recoveries, released_pages, dev.scratch_pages);

	result = run(4, &dev);
	CHECK(result == -ENOMEM && drains == 2 && recoveries == 1 && !released_pages && dev.scratch_pages == 1,
	      "second drain fails: rc=%d drains=%d recoveries=%d released=%d pages=%d\n",
	      result, drains, recoveries, released_pages, dev.scratch_pages);

	result = run(6, &dev);
	CHECK(result == -EIO && drains == ANE_DART_SCRATCH_MAX && recoveries == 1 && !released_pages &&
	      dev.scratch_pages == ANE_DART_SCRATCH_MAX,
	      "scratch cap then a clear latch: rc=%d drains=%d recoveries=%d released=%d pages=%d\n",
	      result, drains, recoveries, released_pages, dev.scratch_pages);

	result = run(5, &dev);
	CHECK(result == -EIO && drains == ANE_DART_SCRATCH_MAX && recoveries == 1 && !released_pages &&
	      dev.scratch_pages == ANE_DART_SCRATCH_MAX,
	      "scratch cap: rc=%d drains=%d recoveries=%d released=%d pages=%d\n",
	      result, drains, recoveries, released_pages, dev.scratch_pages);

	if (failures)
		return 1;
	puts("ane tm completion: PASS");
	return 0;
}
'''


def production(ref):
    if ref:
        return subprocess.check_output(['git', 'show', f'{ref}:{SOURCE}'], cwd=ROOT, text=True)
    return (ROOT / SOURCE).read_text()


def build(text, work):
    start = text.index('static int ane_tm_poll_step(')
    end = text.index('\n/*\n * Bounded recovery', start)
    (work / 'tm.c').write_text(HARNESS + text[start:end] + MAIN)
    subprocess.run(['cc', '-std=gnu11', '-Wall', '-Werror', str(work / 'tm.c'), '-o', str(work / 'tm')], check=True)
    return subprocess.run([str(work / 'tm')], capture_output=True, text=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--old-ref', help='Require this older revision to fail the same checks')
    args = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        result = build(production(args.old_ref), Path(tmp))
    if args.old_ref:
        if result.returncode == 0:
            raise SystemExit(f'{args.old_ref} passed; the test does not catch the original code')
        print(result.stderr, end='')
        print('old revision fails as required')
        return
    print(result.stdout + result.stderr, end='')
    if result.returncode:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
