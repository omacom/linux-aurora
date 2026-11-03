# SPDX-License-Identifier: GPL-2.0-only
"""diag reports the sensor's bind, so that touchid can read "ready".

On the Neo every boot logged "Touch ID: restored 3" while diag/sensor read
"unbound" and diag/touchid "not-ready": nothing called
sep_diag_set_sensor_bound(), so every bring-up result was refused and
aurora-sep-ready rightly said NOT ready. This builds the real diag_shim.c on
the host against minimal kernel headers, drives it as the sensor's SPI probe,
a bring-up and the SPI remove do, checks each attribute on the way, and runs
aurora-sep-ready against the attributes it produced. It also checks that the
sensor's probe and remove call the hook.
"""

import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
APPLE = ROOT / 'drivers/soc/apple'
SCRIPT = Path(__file__).resolve().parents[1] / 'aurora-sep-ready'

HEADERS = {
    'linux/types.h': r'''
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;
typedef int8_t s8; typedef int16_t s16; typedef int32_t s32; typedef int64_t s64;
typedef unsigned short umode_t;
typedef uint64_t __u64; typedef uint32_t __u32; typedef uint16_t __u16; typedef uint8_t __u8;
typedef int64_t __s64; typedef int32_t __s32;
#define __user
''',
    'linux/atomic.h': r'''
#include <linux/types.h>
typedef struct { int counter; } atomic_t;
#define atomic_read(v) __atomic_load_n(&(v)->counter, __ATOMIC_SEQ_CST)
#define atomic_set(v, i) __atomic_store_n(&(v)->counter, (i), __ATOMIC_SEQ_CST)
static inline int atomic_cmpxchg(atomic_t *v, int old, int new_)
{
	__atomic_compare_exchange_n(&v->counter, &old, new_, false,
				    __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
	return old;
}
static inline bool atomic_try_cmpxchg(atomic_t *v, int *old, int new_)
{
	return __atomic_compare_exchange_n(&v->counter, old, new_, false,
					   __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}
#define atomic_inc(v) __atomic_add_fetch(&(v)->counter, 1, __ATOMIC_SEQ_CST)
''',
    'linux/device.h': r'''
#include <linux/types.h>
struct device { int unused; };
struct attribute { const char *name; umode_t mode; };
struct device_attribute {
	struct attribute attr;
	ssize_t (*show)(struct device *, struct device_attribute *, char *);
};
struct attribute_group { const char *name; struct attribute **attrs; };
#define DEVICE_ATTR_RO(_name) \
	struct device_attribute dev_attr_##_name = \
		{ .attr = { .name = #_name, .mode = 0444 }, .show = _name##_show }
extern const struct attribute_group *registered_group;
static inline int devm_device_add_group(struct device *dev,
					const struct attribute_group *grp)
{
	registered_group = grp;
	return 0;
}
''',
    'linux/sysfs.h': r'''
#include <linux/types.h>
#include <stdarg.h>
#include <stdio.h>
static inline int sysfs_emit(char *buf, const char *fmt, ...)
{
	va_list args;
	int n;

	va_start(args, fmt);
	n = vsnprintf(buf, 4096, fmt, args);
	va_end(args);
	return n;
}
''',
    'linux/minmax.h': '#define min(a, b) ((a) < (b) ? (a) : (b))\n',
    'linux/string.h': '#include <string.h>\n',
}

HARNESS = r'''
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/types.h>
#include <linux/device.h>
#include "shim.h"

const struct attribute_group *registered_group;

static const char *attr(const char *name)
{
	static char buf[4096];
	struct attribute **a;

	for (a = registered_group->attrs; *a; a++) {
		struct device_attribute *da = (struct device_attribute *)*a;

		if (!strcmp((*a)->name, name)) {
			da->show(NULL, da, buf);
			buf[strcspn(buf, "\n")] = '\0';
			return buf;
		}
	}
	return "absent";
}

static int failures;

static void expect(const char *step, const char *name, const char *want)
{
	const char *got = attr(name);

	if (strcmp(got, want)) {
		printf("FAIL %s: %s is \"%s\", want \"%s\"\n", step, name, got, want);
		failures++;
	}
}

/* Writes the diag directory as sysfs would show it now. */
static void dump(const char *dir)
{
	static const char *names[] = { "abi", "touchid", "sensor", "sensor_attach",
				       "restore", "keybag", "keystore", "boot", "xart",
				       "xart_guard", "xart_guard_list",
				       "xart_guard_refusals", "matching", NULL };
	char path[4096];

	for (const char **n = names; *n; n++) {
		FILE *f;

		snprintf(path, sizeof(path), "%s/%s", dir, *n);
		f = fopen(path, "w");
		fprintf(f, "%s\n", attr(*n));
		fclose(f);
	}
}

int main(int argc, char **argv)
{
	struct device dev;
	unsigned int bind, stale;

	/* The Neo: warm attach, xART writable, no guard. */
	sep_diag_set_xart_guard(0, false, SEP_DIAG_MATCHING_USABLE);
	sep_diag_register(&dev, "T8140/J700", 10, false, false, true);
	/* The guard's attributes were added within version 2. */
	expect("probe", "abi", "3");
	expect("probe", "sensor", "unbound");
	expect("probe", "touchid", "unknown");
	expect("probe", "xart_guard", "0");
	expect("probe", "xart_guard_list", "none");
	expect("probe", "xart_guard_refusals", "0");
	expect("probe", "matching", "usable");

	/* The sensor's SPI probe. */
	sep_diag_set_sensor_bound(true);
	expect("spi probe", "sensor", "bound");

	/* Activation: the Catacomb restore, then the attach-time bring-up. */
	sep_diag_set_bio_published();
	bind = sep_diag_sensor_bind();
	sep_diag_set_restore(1);
	sep_diag_set_sensor_result(bind, true);
	sep_diag_set_sensor_attach(1);
	sep_diag_set_touchid(true);
	expect("attach", "sensor", "online");
	expect("attach", "sensor_attach", "attached");
	expect("attach", "restore", "restored");
	expect("attach", "touchid", "ready");
	if (argc > 1)
		dump(argv[1]);

	/* The SPI remove: the sensor is gone, and so is "ready". */
	sep_diag_set_sensor_bound(false);
	expect("spi remove", "sensor", "unbound");
	expect("spi remove", "touchid", "not-ready");
	/* A bring-up that began before the remove cannot report against it. */
	sep_diag_set_sensor_result(bind, true);
	expect("late result", "sensor", "unbound");

	/* Bound again: a stale result is refused, the new device's counts. */
	stale = bind;
	sep_diag_set_sensor_bound(true);
	expect("rebind", "sensor", "bound");
	expect("rebind", "touchid", "not-ready");
	sep_diag_set_sensor_result(stale, true);
	expect("stale result", "sensor", "bound");
	bind = sep_diag_sensor_bind();
	sep_diag_set_sensor_result(bind, false);
	expect("failed bring-up", "sensor", "failed");
	expect("failed bring-up", "touchid", "not-ready");
	sep_diag_set_sensor_result(bind, true);
	expect("later bring-up", "sensor", "online");
	expect("later bring-up", "touchid", "ready");

	puts(failures ? "FAILED" : "OK");
	return failures ? 1 : 0;
}
'''

# The xART guard's attributes as the shim shows what the driver set. The
# matching value itself is derived in xart_guard.rs (MatchFacts), with KUnit
# tests for each case; this checks that each value reads back as its name and
# that refusals are counted. Each process is a fresh probe.
GUARD_HARNESS = r'''
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/types.h>
#include <linux/device.h>
#include "shim.h"

const struct attribute_group *registered_group;

static const char *attr(const char *name)
{
	static char buf[4096];
	struct attribute **a;

	for (a = registered_group->attrs; *a; a++) {
		struct device_attribute *da = (struct device_attribute *)*a;

		if (!strcmp((*a)->name, name)) {
			da->show(NULL, da, buf);
			buf[strcspn(buf, "\n")] = '\0';
			return buf;
		}
	}
	return "absent";
}

/* argv: level list matching-at-probe refusals matching-after-refusal */
int main(int argc, char **argv)
{
	struct device dev;
	int refusals = atoi(argv[4]);

	sep_diag_set_xart_guard(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
	sep_diag_register(&dev, "TEST", 4, false, false, true);
	while (refusals--)
		sep_diag_xart_guard_refused(atoi(argv[5]));
	/* attr() returns one static buffer: print each before the next. */
	printf("%s ", attr("abi"));
	printf("%s ", attr("xart_guard"));
	printf("%s ", attr("xart_guard_list"));
	printf("%s ", attr("xart_guard_refusals"));
	printf("%s\n", attr("matching"));
	return 0;
}
'''


def function(source, signature):
    """The body of the C function whose definition starts with `signature`."""
    begin = source.index(signature)
    opening = source.index('{', begin)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[opening:end]


class DiagSensorTest(unittest.TestCase):
    def build(self, root, harness, name):
        for header, text in HEADERS.items():
            path = root / 'include' / header
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#pragma once\n' + text)
        (root / (name + '.c')).write_text(harness)
        subprocess.run(['cc', '-std=gnu11', '-Wall', '-Werror', '-Wno-unused-function',
                        '-I', str(root / 'include'), '-I', str(APPLE),
                        str(APPLE / 'diag_shim.c'), str(root / (name + '.c')),
                        '-o', str(root / name)], check=True)
        return root / name

    @unittest.skipUnless(shutil.which('cc'), 'needs a C compiler')
    def test_matching_follows_the_guard(self):
        usable, unverified, blocked = 1, 2, 3  # SEP_DIAG_MATCHING_* in shim.h
        cases = [
            # level, list, matching at probe, refusals, matching after them
            ((0, 0, usable, 0, 0), '3 0 none 0 usable'),            # Neo, M1 Pro (pinned 0)
            ((1, 1, unverified, 0, 0), '3 1 loaded 0 unverified'),  # M2 under the guard
            ((1, 1, unverified, 2, blocked), '3 1 loaded 2 blocked'),  # ... after refusals
            ((2, 1, unverified, 0, 0), '3 2 loaded 0 unverified'),
            ((1, 0, blocked, 0, 0), '3 1 unavailable 0 blocked'),   # no list
            ((1, 1, blocked, 0, 0), '3 1 loaded 0 blocked'),        # cold boot, level 1
            ((0, 0, 0, 0, 0), '3 0 none 0 unknown'),                # never set
            ((0, 0, 7, 0, 0), '3 0 none 0 unknown'),                # out of range
        ]
        with tempfile.TemporaryDirectory() as directory:
            harness = self.build(Path(directory), GUARD_HARNESS, 'guard')
            for args, want in cases:
                done = subprocess.run([str(harness), *map(str, args)],
                                      capture_output=True, text=True, check=True)
                self.assertEqual(done.stdout.strip(), want, args)

    def test_probe_and_remove_report_the_bind(self):
        source = (APPLE / 'sensor_shim.c').read_text()
        probe = function(source, 'static int sep_sensor_probe(struct spi_device *spi)')
        remove = function(source, 'static void sep_sensor_remove(struct spi_device *spi)')
        self.assertRegex(probe, r'sep_active_sensor = sensor;\s*sep_diag_set_sensor_bound\(true\);')
        self.assertIn('sep_diag_set_sensor_bound(false);', remove)
        # Nothing else claims the bind.
        calls = re.findall(r'sep_diag_set_sensor_bound\(', source)
        self.assertEqual(len(calls), 2)

    @unittest.skipUnless(shutil.which('cc'), 'needs a C compiler')
    def test_diag_goes_bound_attached_ready_and_back(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, text in HEADERS.items():
                path = root / 'include' / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('#pragma once\n' + text)
            (root / 'harness.c').write_text(HARNESS)
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Werror', '-Wno-unused-function',
                            '-I', str(root / 'include'), '-I', str(APPLE),
                            str(APPLE / 'diag_shim.c'), str(root / 'harness.c'),
                            '-o', str(root / 'harness')], check=True)
            diag = root / 'sys' / '1234.sep' / 'diag'
            diag.mkdir(parents=True)
            done = subprocess.run([str(root / 'harness'), str(diag)],
                                  capture_output=True, text=True)
            self.assertEqual(done.returncode, 0, done.stdout + done.stderr)

            # The attributes of the attached boot are what aurora-sep-ready
            # calls ready.
            env = dict(os.environ, AURORA_SEP_DEV='/dev/null',
                       AURORA_SEP_SYSFS=str(root / 'sys'), AURORA_SEP_READY_WAIT='3')
            ready = subprocess.run(['sh', str(SCRIPT)], env=env, capture_output=True,
                                   text=True, timeout=30)
            self.assertEqual(ready.returncode, 0, ready.stderr)
            self.assertIn('is ready', ready.stderr)


if __name__ == '__main__':
    unittest.main()
