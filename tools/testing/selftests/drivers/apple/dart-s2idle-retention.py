#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise the suspend-to-idle resume check of retained DARTs on the host.

The actual state check, quarantine and map functions are extracted from
apple-dart.c. A retained DART whose translation registers changed must fail
resume and quarantine its consumers, so that later maps are refused. These CPU
tests do not qualify physical retention on a J713.
"""
from pathlib import Path
import re
import subprocess
import tempfile


def definition(source, name):
    start = source.index(name)
    start = source.rfind('\n', 0, start) + 1
    brace = source.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def macro(source, name):
    start = source.index('#define ' + name + '(')
    end = start
    while True:
        end = source.index('\n', end) + 1
        if source[end - 2] != '\\':
            return source[start:end]


PRELUDE = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef u64 phys_addr_t;
typedef unsigned int gfp_t;
#define BIT(n) (1UL << (n))
#define BIT_ULL(n) (1ULL << (n))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define WRITE_ONCE(x, v) ((x) = (v))
#define READ_ONCE(x) (x)
#define DART_MAX_STREAMS 32
#define DART_MAX_TTBR 4
#define DART_TCR(d, sid) (0x100 + 4 * (sid))
#define DART_TTBR(d, sid, idx) (0x200 + 16 * (sid) + 4 * (idx))
#define container_of(p, t, m) ((t *)((char *)(p) - offsetof(t, m)))
#define lockdep_assert_held(l) ((void)(l))
#define spin_lock_irqsave(l, f) ((void)(l), (f) = 0)
#define spin_unlock_irqrestore(l, f) ((void)(l), (void)(f))
static int errors;
#define dev_err(dev, ...) (errors++)
#define dev_info(dev, ...) ((void)0)
#define MAX_DARTS_PER_DEVICE 8
static void __set_bit(unsigned int n, unsigned long *p) { p[n / 64] |= 1UL << (n % 64); }

struct apple_dart_hw { u32 tcr_bypass, ttbr_valid; unsigned int ttbr_count; };
struct apple_dart;
struct apple_dart_dcp { bool failed, failed_logged; unsigned int sid; struct apple_dart *dart; };
struct apple_dart {
    const struct apple_dart_hw *hw;
    u16 managed_sids;
    bool managed_failed, supports_bypass, s2idle_kept;
    u32 host_off_slept : 1;
    int managed_lock;
    struct { bool ready; } iommu;
    struct apple_dart_dcp *dcp, *frontend[2];
    void *dev;
    unsigned int num_streams;
    u32 save_tcr[DART_MAX_STREAMS], save_ttbr[DART_MAX_STREAMS][DART_MAX_TTBR];
    u32 regs[0x400 / 4];
};
struct apple_dart_stream_map { struct apple_dart *dart; unsigned long sidmap[4]; };
static u32 apple_dart_readl(struct apple_dart *d, unsigned int o) { return d->regs[o / 4]; }
static unsigned long disabled;
static void apple_dart_hw_disable_dma(struct apple_dart_stream_map *m) { disabled |= m->sidmap[0]; }

struct iommu_domain { int unused; };
struct io_pgtable_ops {
    int (*map_pages)(struct io_pgtable_ops *, unsigned long, phys_addr_t, size_t, size_t, int,
                     gfp_t, size_t *);
};
struct apple_dart_domain {
    struct iommu_domain domain;
    struct io_pgtable_ops *pgtbl_ops;
    struct apple_dart_dcp *dcp;
    struct apple_dart_stream_map stream_maps[1];
    u64 dma_offset, mask;
};
#define to_dart_domain(d) container_of(d, struct apple_dart_domain, domain)
static int apple_dart_check_fw_map(struct apple_dart_domain *d, u64 dva, phys_addr_t pa,
                                   size_t size, int prot) { return 0; }
static int apple_dart_dcp_map(struct apple_dart_dcp *dcp, unsigned long iova, phys_addr_t pa,
                              size_t pgsize, size_t count, int prot, gfp_t gfp, size_t *mapped)
{ assert(!"the managed radio domain is not a borrowed display domain"); return 0; }
static int maps, writers, v27_ops;
/* The sleep and T6050 display gates are not under test here: admit, but balance. */
static int apple_dart_sleep_writer_get(struct apple_dart *d) { assert(d); writers++; return 1; }
static void apple_dart_sleep_writer_put(struct apple_dart *d, int gate) { assert(d && gate == 1); writers--; }
static int apple_dart_v27_enter(struct apple_dart *d, bool unmap) { assert(d && !unmap); v27_ops++; return 0; }
static void apple_dart_v27_leave(struct apple_dart *d) { assert(d); v27_ops--; }
static int test_map(struct io_pgtable_ops *ops, unsigned long iova, phys_addr_t pa, size_t pgsize,
                    size_t count, int prot, gfp_t gfp, size_t *mapped)
{ maps++; *mapped = pgsize * count; return 0; }
'''

TEST = r'''
static const struct apple_dart_hw hw = { .tcr_bypass = 0, .ttbr_valid = 1, .ttbr_count = 1 };
static struct apple_dart_dcp dcp, front0, front1;
static struct apple_dart d;
static struct io_pgtable_ops ops = { .map_pages = test_map };
static struct apple_dart_domain radio;

static void setup(void)
{
    memset(&d, 0, sizeof(d));
    dcp = (struct apple_dart_dcp){ .sid = 16, .dart = &d };
    front0 = (struct apple_dart_dcp){ .sid = 0, .dart = &d };
    front1 = (struct apple_dart_dcp){ .sid = 1, .dart = &d };
    d.hw = &hw;
    d.num_streams = 16;
    d.managed_sids = BIT(1) | BIT(2);
    d.dcp = &dcp;
    d.frontend[0] = &front0;
    d.frontend[1] = &front1;
    d.iommu.ready = true;
    /* Live translations on the radio's streams and on an ordinary one. */
    d.regs[DART_TCR(&d, 1) / 4] = d.regs[DART_TCR(&d, 2) / 4] = d.regs[DART_TCR(&d, 5) / 4] = 1;
    d.regs[DART_TTBR(&d, 1, 0) / 4] = 0x1001;
    d.regs[DART_TTBR(&d, 2, 0) / 4] = 0x2001;
    d.regs[DART_TTBR(&d, 5, 0) / 4] = 0x5001;
    for (unsigned int sid = 0; sid < d.num_streams; sid++) {
        d.save_tcr[sid] = d.regs[DART_TCR(&d, sid) / 4];
        d.save_ttbr[sid][0] = d.regs[DART_TTBR(&d, sid, 0) / 4];
    }
    d.s2idle_kept = true;
    disabled = 0;
    maps = 0;
    errors = 0;
    radio = (struct apple_dart_domain){ .pgtbl_ops = &ops, .mask = ~0ULL,
                                        .stream_maps = {{ .dart = &d }} };
}

static int map_16k(void)
{
    size_t mapped = 0;
    int ret = apple_dart_map_pages(&radio.domain, 0x10000, 0x800000000ULL, 0x4000, 1, 3, 0, &mapped);
    assert(!writers && !v27_ops);
    return ret;
}

static void expect_quarantined(int ret)
{
    /* One resume error, then each borrowed stream logs its retirement once. */
    assert(ret == -EIO && errors == 4);
    assert(d.managed_failed && disabled == d.managed_sids);
    assert(dcp.failed && front0.failed && front1.failed);
    assert(dcp.failed_logged && front0.failed_logged && front1.failed_logged);
    assert(map_16k() == -EIO && maps == 0);
}

int main(void)
{
    /* Retained: resume succeeds and the radio maps as before. */
    setup();
    assert(apple_dart_resume_retained(&d) == 0 && !d.s2idle_kept);
    assert(!d.managed_failed && !disabled && !dcp.failed && !errors);
    assert(map_16k() == 0 && maps == 1);

    /* A changed TTBR on a managed stream. */
    setup();
    d.regs[DART_TTBR(&d, 1, 0) / 4] = 0x9001;
    expect_quarantined(apple_dart_resume_retained(&d));

    /* A changed TCR on an ordinary stream of the same DART. */
    setup();
    d.regs[DART_TCR(&d, 5) / 4] = 0;
    expect_quarantined(apple_dart_resume_retained(&d));

    /* A DART that lost power reads back as reset. */
    setup();
    memset(d.regs, 0, sizeof(d.regs));
    memset(d.save_tcr, 0, sizeof(d.save_tcr));
    memset(d.save_ttbr, 0, sizeof(d.save_ttbr));
    expect_quarantined(apple_dart_resume_retained(&d));

    /* No suspend-to-idle suspend: refused, nothing quarantined. */
    setup();
    d.s2idle_kept = false;
    assert(apple_dart_resume_retained(&d) == -EBUSY);
    assert(!d.managed_failed && !dcp.failed && map_16k() == 0);

    puts("PASS retained DART resume: kept state resumes; lost state quarantines and refuses maps");
    return 0;
}
'''


def main():
    root = Path(__file__).resolve().parents[5]
    dart = (root / 'drivers/iommu/apple-dart.c').read_text()
    dcp = (root / 'drivers/iommu/apple-dart-dcp.h').read_text()
    code = PRELUDE
    code += macro(dcp, 'apple_dart_dcp_fail') + '\n'
    code += macro(dcp, 'apple_dart_fail_borrowed') + '\n'
    for name in ('static bool apple_dart_state_retained_except(',
                 'static bool apple_dart_state_retained(',
                 'static void apple_dart_managed_quarantine_locked(',
                 'static void apple_dart_managed_quarantine(',
                 'static void apple_dart_quarantine_borrowed(',
                 'static int apple_dart_resume_retained(',
                 'static int apple_dart_map_pages_unguarded(',
                 'static int apple_dart_map_pages_v27(',
                 'static int apple_dart_map_pages('):
        code += definition(dart, name) + '\n'
    code += TEST
    resume = definition(dart, 'static int apple_dart_resume_retained(')
    found = re.findall(r'\tif \(dart->managed_sids\)\n\t\tapple_dart_managed_quarantine\(dart\);\n'
                       r'\tapple_dart_quarantine_borrowed\(dart, [^\n]*\);\n', resume)
    assert len(found) == 1, 'the resume quarantine was not found'
    quarantine = found[0]
    assert code.count(quarantine) == 1, 'the resume quarantine is not unique'
    with tempfile.TemporaryDirectory() as directory:
        c, exe = Path(directory) / 'resume.c', Path(directory) / 'resume'
        for control in ('production', 'missing-quarantine'):
            tested = code if control == 'production' else code.replace(quarantine, '')
            c.write_text(tested)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-Wno-unused-parameter', '-Wno-unused-function',
                            str(c), '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            if control == 'production':
                assert result.returncode == 0, result.stderr
                print(result.stdout.strip())
            else:
                assert result.returncode and 'Assertion' in result.stderr, result
                print('Retained DART resume ' + control + ' negative control: detected at runtime')


if __name__ == '__main__':
    main()
