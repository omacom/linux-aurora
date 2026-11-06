# SPDX-License-Identifier: MIT
"""The regular-file C/Rust boundary must not turn an I/O error into absence.

Compile the actual C wrapper bodies with a fake open_as_kernel(). No kernel,
files, enclave, or stored identity is opened. When rustc is available, also
compile the unchanged Rust module against a tiny kernel API
facade and those C wrappers; this is a userspace boundary fixture, not a kernel
build or a real filesystem test.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
C_SOURCE = ROOT / 'drivers/soc/apple/store_shim.c'
RUST_SOURCE = ROOT / 'drivers/soc/apple/shim.rs'
OPENS = ('sep_store_open', 'sep_store_open_trunc', 'sep_store_open_ro')


def function(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


PREFIX = r'''
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#ifndef O_LARGEFILE
#define O_LARGEFILE 0
#endif
struct file { int value; };
static struct file live_file;
static struct file *next_result;
static struct file *open_as_kernel(const char *path, int flags, unsigned mode)
{
    (void)path; (void)flags; (void)mode;
    return next_result;
}
'''

RUNNER = r'''
int main(void)
{
    void *(*opens[])(const char *) = {
        sep_store_open, sep_store_open_trunc, sep_store_open_ro
    };
    const int errors[] = { ENOENT, EACCES, EIO, ENOMEM, EROFS, ELOOP };
    for (unsigned i = 0; i < sizeof(opens) / sizeof(opens[0]); ++i) {
        next_result = &live_file;
        if (opens[i]("unused") != &live_file) return 2;
        for (unsigned j = 0; j < sizeof(errors) / sizeof(errors[0]); ++j) {
            next_result = (struct file *)(intptr_t)-errors[j];
            if (opens[i]("unused") != next_result) {
                fprintf(stderr, "wrapper %u lost errno %d\n", i, errors[j]);
                return 1;
            }
        }
    }
    puts("21 open results preserved");
    return 0;
}
'''


class StoreOpenErrors(unittest.TestCase):
    def native(self, negative_control=False):
        compiler = shutil.which('cc')
        if compiler is None:
            self.skipTest('C compiler unavailable')
        source = C_SOURCE.read_text()
        bodies = '\n'.join(function(source, 'void *' + name + '(') for name in OPENS)
        prefix = PREFIX
        if negative_control:
            # Reintroduce the old error-pointer-to-NULL collapse. The same
            # executable must reject it, rather than merely compile it.
            prefix += r'''
static void *collapse(struct file *f)
{
    return (uintptr_t)f >= (uintptr_t)-4095 ? NULL : f;
}
'''
            bodies, changed = re.subn(r'return (open_as_kernel\([^;]+\));', r'return collapse(\1);', bodies)
            self.assertEqual(changed, 3)
        with tempfile.TemporaryDirectory(prefix='sep-open-errors-') as tmp:
            root = Path(tmp)
            (root / 'test.c').write_text(prefix + bodies + RUNNER)
            subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror',
                            str(root / 'test.c'), '-o', str(root / 'test')], check=True)
            return subprocess.run([str(root / 'test')], capture_output=True, text=True)

    def test_native_open_results(self):
        result = self.native()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('21 open results preserved', result.stdout)

    def test_old_error_collapse_is_rejected(self):
        result = self.native(negative_control=True)
        self.assertEqual(result.returncode, 1)
        self.assertIn('lost errno', result.stderr)

    def test_block_device_open_contract_unchanged(self):
        body = function(C_SOURCE.read_text(), 'void *sep_store_open_block(')
        self.assertRegex(body, r'if \(IS_ERR\(f\)\)\s+return NULL;')

    def test_native_rust_c_boundary(self):
        compiler, rustc = shutil.which('cc'), shutil.which('rustc')
        if compiler is None or rustc is None:
            self.skipTest('C and Rust compilers required')
        bodies = '\n'.join(function(C_SOURCE.read_text(), 'void *' + name + '(') for name in OPENS)
        c = PREFIX + bodies + r'''
static unsigned closes;
void fixture_result(int error)
{
    next_result = error ? (struct file *)(intptr_t)-error : &live_file;
}
void sep_store_close(void *file)
{
    if (file == &live_file) ++closes;
}
unsigned fixture_closes(void) { return closes; }
'''
        rust = r'''
#![allow(dead_code)]
extern crate self as kernel;
pub mod error {
    #[derive(Debug, PartialEq)]
    pub struct Error(pub i32);
    impl Error { pub fn from_errno(n: i32) -> Self { Self(n) } }
    pub type Result<T = ()> = core::result::Result<T, Error>;
    pub const EIO: Error = Error(-5);
    pub const ENODEV: Error = Error(-19);
    pub fn to_result(n: i32) -> Result { if n < 0 { Err(Error(n)) } else { Ok(()) } }
    pub unsafe fn from_err_ptr<T>(p: *mut T) -> Result<*mut T> {
        if p as usize >= usize::MAX - 4094 { Err(Error(p as isize as i32)) } else { Ok(p) }
    }
}
pub mod prelude {
    pub use core::ffi::{c_char,c_int,c_void,c_long,c_uint,c_ulong};
    pub use crate::error::{Error, Result, EIO, ENODEV};
    pub struct CStr(pub &'static [u8]);
    impl CStr { pub fn as_char_ptr(&self) -> *const c_char { self.0.as_ptr().cast() } }
}
#[path="shim.rs"] mod shim;
extern "C" { fn fixture_result(error: i32); fn fixture_closes() -> u32; }
fn main() {
    let path = prelude::CStr(b"unused\0");
    let opens: [fn(&prelude::CStr) -> error::Result<shim::StoreFile>; 3] = [
        shim::StoreFile::open, shim::StoreFile::open_trunc, shim::StoreFile::open_readonly
    ];
    for open in opens {
        unsafe { fixture_result(0); }
        drop(open(&path).expect("successful handle"));
        for errno in [2, 13, 5, 12, 30, 40] {
            unsafe { fixture_result(errno); }
            match open(&path) {
                Err(e) if e == error::Error(-errno) => (),
                _ => panic!("Rust lost an open error"),
            }
        }
    }
    if unsafe { fixture_closes() } != 3 { panic!("only successful handles may close"); }
    println!("21 Rust/C results preserved; 3 successful handles closed");
}
'''
        with tempfile.TemporaryDirectory(prefix='sep-open-rust-') as tmp:
            root = Path(tmp)
            (root / 'test.c').write_text(c)
            (root / 'test.rs').write_text(rust)
            shutil.copyfile(RUST_SOURCE, root / 'shim.rs')
            subprocess.run([compiler, '-std=c11', '-Wall', '-Wextra', '-Werror', '-c',
                            str(root / 'test.c'), '-o', str(root / 'test.o')], check=True)
            subprocess.run([rustc, '--edition=2021', '-C', 'opt-level=1',
                            '-C', 'link-arg=' + str(root / 'test.o'),
                            str(root / 'test.rs'), '-o', str(root / 'test')], check=True)
            result = subprocess.run([str(root / 'test')], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('21 Rust/C results preserved', result.stdout)


if __name__ == '__main__':
    unittest.main()
