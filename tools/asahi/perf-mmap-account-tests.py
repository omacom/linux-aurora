#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Host check for the perf mmap locked-memory accounting.

Linux 7.2.15 moved the per-user perf ring-buffer accounting from
user->locked_vm to user->perf_mlock and kept the page count in
data_page_nr(); the aurora user4k tree counts the ring in native pages.
The two sides have to agree: the pages charged by perf mmap() have to be
the pages returned by perf munmap(), on the same per-user counter.

This test compiles the accounting code straight out of the tree and runs
the charge/uncharge cycle for a range of ring sizes on a 16K page kernel.
"""
from pathlib import Path
import argparse
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tree', type=Path, default=root,
                    help='kernel tree holding kernel/events (default: this tree)')
parser.add_argument('--old-source', type=Path,
                    help='core.c from before the fix; it must not balance')
args = parser.parse_args()

core_path = args.tree/'kernel/events/core.c'
internal_path = args.tree/'kernel/events/internal.h'
ring_path = args.tree/'kernel/events/ring_buffer.c'


def definition(text, name):
    """Return the C text of a function definition, not of its prototype."""
    for match in re.finditer(r'\b' + re.escape(name) + r'\s*\(', text):
        index = match.end() - 1
        depth = 0
        while index < len(text):
            if text[index] == '(':
                depth += 1
            elif text[index] == ')':
                depth -= 1
                if depth == 0:
                    break
            index += 1
        brace = text.find('{', index)
        semi = text.find(';', index)
        if brace == -1 or (semi != -1 and semi < brace):
            continue
        depth = 0
        for position in range(brace, len(text)):
            if text[position] == '{':
                depth += 1
            elif text[position] == '}':
                depth -= 1
                if depth == 0:
                    start = text.rfind('\n', 0, match.start()) + 1
                    return text[start:position + 1]
    raise ValueError('no definition for ' + name)


def statement(text, name):
    """Return the right hand side of the last assignment to name."""
    found = None
    for match in re.finditer(r'\b' + re.escape(name) + r'\s*=\s*([^;]+);', text):
        found = match.group(1).strip()
    if found is None:
        raise ValueError('no assignment to ' + name)
    return found


def source_problems(core):
    """Return the reasons the tree's accounting is not the requested one."""
    problems = []
    unaccount = definition(core, 'perf_mmap_unaccount')
    if 'locked_vm' in core:
        problems.append('kernel/events/core.c still accounts into user->locked_vm')
    if 'perf_mlock' not in unaccount:
        problems.append('perf_mmap_unaccount() does not charge user->perf_mlock')
    if 'data_page_nr' not in unaccount:
        problems.append('perf_mmap_unaccount() does not use the aurora data_page_nr() page count')
    if 'perf_data_size' in unaccount:
        problems.append('perf_mmap_unaccount() derives the page count from perf_data_size()')
    return problems


HARNESS = r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef long atomic_long_t;
typedef long long atomic64_t;

#define PAGE_SHIFT 14
#define PAGE_SIZE (1UL << PAGE_SHIFT)
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define ilog2(n) (31 - __builtin_clz((unsigned int)(n)))

struct user_struct {
	atomic_long_t perf_mlock;
	atomic_long_t locked_vm;
};

static struct user_struct test_user;

static struct user_struct *current_user(void)
{
	return &test_user;
}

struct mm_struct {
	atomic64_t pinned_vm;
};

struct vm_area_struct {
	struct mm_struct *vm_mm;
};

struct perf_buffer {
	int nr_pages;
	unsigned int data_page_shift;
	unsigned int mmap_page_shift;
	unsigned long mmap_locked;
	struct user_struct *mmap_user;
	int aux_nr_pages;
	unsigned long aux_mmap_locked;
};

#define atomic_long_sub(v, p) (*(p) -= (v))
#define atomic_long_add(v, p) (*(p) += (v))
#define atomic64_sub(v, p) (*(p) -= (v))
#define atomic64_add(v, p) (*(p) += (v))

/* Accounting code taken from the tree under test. */
%(helpers)s

static void fake_rb_alloc(struct perf_buffer *rb, unsigned long data_size,
			   unsigned int mmap_page_shift)
{
%(alloc)s
}

int main(void)
{
	static const unsigned int shifts[] = { 12, 14 };
	static const unsigned long rings[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 1024, 4096 };
	unsigned int shift_index, ring_index;

	for (shift_index = 0; shift_index < sizeof(shifts) / sizeof(shifts[0]); shift_index++) {
		for (ring_index = 0; ring_index < sizeof(rings) / sizeof(rings[0]); ring_index++) {
			struct perf_buffer rb;
			struct mm_struct mm;
			struct vm_area_struct vma;
			unsigned int page_shift = shifts[shift_index];
			unsigned long nr_pages = rings[ring_index];
			unsigned long data_size;
			long user_extra, extra;

			memset(&rb, 0, sizeof(rb));
			memset(&mm, 0, sizeof(mm));
			vma.vm_mm = &mm;

			data_size = nr_pages << page_shift;
			user_extra = %(user_extra)s;

			fake_rb_alloc(&rb, data_size, page_shift);
			rb.mmap_user = &test_user;

			/*
			 * perf_mmap_calc_limits() moves the part of the charge
			 * that does not fit the user limit into pinned_vm and
			 * remembers it as rb->mmap_locked.
			 */
			for (extra = 0; extra <= 2; extra++) {
				test_user.perf_mlock = 0;
				test_user.locked_vm = 0;
				mm.pinned_vm = 0;
				rb.mmap_locked = extra;

				perf_mmap_account(&vma, user_extra - extra, extra);
				perf_mmap_unaccount(&vma, &rb);

				assert(test_user.perf_mlock == 0);
				assert(test_user.locked_vm == 0);
				assert(mm.pinned_vm == 0);
			}
		}
	}

	puts("charge and uncharge move the same pages: PASS");
	return 0;
}
'''


def build(core, label, out):
    """Compile the harness around the given core.c and run it."""
    helpers = '\n'.join([
        definition(internal, 'perf_data_size'),
        definition(internal, 'data_page_nr'),
        definition(core, 'perf_mmap_account'),
        definition(core, 'perf_mmap_unaccount'),
    ])
    rb_func = definition(ring, 'rb_alloc')
    alloc = '\n'.join([
        re.search(r'int nr_pages = [^;]+;', rb_func).group(0),
        'rb->nr_pages = nr_pages;',
        'rb->mmap_page_shift = mmap_page_shift;',
        'rb->data_page_shift = data_size && data_size < PAGE_SIZE ?\n'
        '\t\t\t      ilog2(data_size) : PAGE_SHIFT;',
    ])

    rb_mmap = definition(core, 'perf_mmap_rb')
    user_extra = statement(rb_mmap, 'user_extra')

    source = HARNESS % {'helpers': helpers, 'alloc': alloc, 'user_extra': user_extra}
    path = out/(label + '.c')
    path.write_text(source)
    binary = out/(label + '.bin')
    subprocess.run(['cc', '-std=gnu11', '-Wall', '-Werror',
                    '-Wno-unused-function', str(path), '-o', str(binary)], check=True)
    return binary


internal = internal_path.read_text()
ring = ring_path.read_text()
core = core_path.read_text()

problems = source_problems(core)
if problems:
    for problem in problems:
        print('FAIL: ' + problem)
    raise SystemExit(1)

with tempfile.TemporaryDirectory(prefix='perf-mmap-account-') as directory:
    out = Path(directory)
    try:
        binary = build(core, 'current', out)
    except ValueError as error:
        print('FAIL: ' + str(error))
        raise SystemExit(1)
    subprocess.run([str(binary)], check=True)

    if args.old_source:
        old = args.old_source.read_text()
        old_problems = source_problems(old)
        old_binary = build(old, 'old', out)
        result = subprocess.run([str(old_binary)], capture_output=True, text=True)
        if result.returncode == 0 and not old_problems:
            print('FAIL: the pre-fix source still balances on the requested counter')
            raise SystemExit(1)
        print('pre-fix source rejected: PASS')
        if result.returncode != 0:
            print('  rejected because: the charge and the uncharge differ')
            for line in result.stderr.strip().splitlines():
                if 'Assertion' in line:
                    print('  ' + line.strip())
        for problem in old_problems:
            print('  rejected because: ' + problem)
