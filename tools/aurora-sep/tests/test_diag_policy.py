#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Compile the real passive policy cache/show bodies against synthetic flags."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'drivers/soc/apple/diag_shim.c'


def function(text, name):
    match = re.search(r'^(?:static ssize_t|void) ' + name + r'\(', text, re.M)
    if not match:
        raise AssertionError(name)
    start = text.index('{', match.start())
    depth = 1
    at = start + 1
    while depth:
        depth += (text[at] == '{') - (text[at] == '}')
        at += 1
    return text[match.start():at]


class Tests(unittest.TestCase):
    def test_selected_policy_and_identity_source_only(self):
        compiler = shutil.which('cc')
        self.assertIsNotNone(compiler, 'C compiler required; do not count a skip as a pass')
        text = SOURCE.read_text()
        state = re.search(r'static struct \{.*?\} sep_diag;', text, re.S).group(0)
        body = '\n'.join(function(text, name) for name in
                         ('policy_show', 'sep_diag_set_policy', 'sep_diag_set_identity_source'))
        prefix = '''#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <sys/types.h>
typedef int atomic_t;
struct device { int unused; };
struct device_attribute { int unused; };
#define atomic_read(p) (*(p))
#define atomic_set(p, v) (*(p) = (v))
static int sysfs_emit(char *out, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static int sysfs_emit(char *out, const char *fmt, ...) {
 va_list args; va_start(args, fmt); int n = vsnprintf(out, 4096, fmt, args); va_end(args); return n;
}
'''
        main = '''int main(void) {
 char out[4096]; sep_diag.xart = true; sep_diag.xart_guard = 1;
 sep_diag_set_policy(false, false, false, true, false, false);
 for (unsigned int i = 0; i < 4; i++) {
  sep_diag_set_identity_source(i); policy_show(NULL, NULL, out); fputs(out, stdout);
 }
 sep_diag_set_policy(true, true, true, false, true, true);
 sep_diag.xart = false; sep_diag.xart_guard = 0;
 policy_show(NULL, NULL, out); fputs(out, stdout); return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            source, program = Path(tmp) / 'policy.c', Path(tmp) / 'policy'
            source.write_text(prefix + state + body + main)
            subprocess.run([compiler, '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                            '-Wno-unused-parameter', str(source), '-o', str(program)], check=True)
            lines = subprocess.check_output([str(program)], text=True).splitlines()
        expected = dict(xart_writes='1', provision_keybag='0', xart_guard='1',
                        trusted_keys='0', filevault_api='0', xarm_trace='1',
                        refkey_v2='0', identity_override='0')
        for line, identity in zip(lines[:4], ('pending', 'dt', 'override', 'missing')):
            self.assertEqual(dict(word.split('=') for word in line.split()),
                             {**expected, 'identity_source': identity})
        self.assertEqual(dict(word.split('=') for word in lines[4].split()),
                         dict(xart_writes='0', provision_keybag='1', xart_guard='0',
                              trusted_keys='1', filevault_api='1', xarm_trace='0',
                              refkey_v2='1', identity_override='1', identity_source='pending'))
        self.assertNotIn('os_uuid_hi=', '\n'.join(lines))
        self.assertNotIn('os_uuid_lo=', '\n'.join(lines))


if __name__ == '__main__':
    unittest.main(verbosity=2)
