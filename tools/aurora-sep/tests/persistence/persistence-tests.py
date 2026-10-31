#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Fault tests of current SEP production sources; no hardware, root or network."""


import argparse


import hashlib


import json


import os


from pathlib import Path


import re


import shutil


import subprocess


HERE = Path(__file__).resolve().parent


TREE = HERE.parents[3]


APPLE = TREE / 'drivers/soc/apple'


FIXTURES = HERE / 'fixtures'


def function(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError('Mutation no longer targets exactly one production site: ' + old)
    return text.replace(old, new)


def fixture(name):
    return (FIXTURES / name).read_text()


def command(argv, log, env=None):
    result = subprocess.run(list(map(str, argv)), capture_output=True, text=True,
                            timeout=120, env=env)
    log.write_text(result.stdout + result.stderr)
    return result


def require_compile(result, log):
    if result.returncode:
        raise RuntimeError('Compilation failed; never credited as a caught mutant: ' + str(log))


def c_build(source, output, binary=False):
    c = output.with_suffix('.c')
    c.write_text(source)
    # Host fixture only: ASan/UBSan provide instrumentation without a distro
    # stack-protector ABI dependency when linking with an older Rust compiler.
    flags = [os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Wextra', '-Werror',
             '-Wno-misleading-indentation', '-fsanitize=address,undefined',
             '-fno-omit-frame-pointer', '-fno-stack-protector', '-g']
    log = output.with_suffix('.compile.log')
    result = command(flags + ([] if binary else ['-c']) + [c, '-o', output], log)
    require_compile(result, log)


def runtime_environment(linked_c):
    env = os.environ.copy()
    env['ASAN_OPTIONS'] = 'detect_leaks=0:abort_on_error=1'
    env['UBSAN_OPTIONS'] = 'halt_on_error=1:print_stacktrace=1'
    if linked_c:
        cc = os.environ.get('CC', 'cc')
        env['LD_PRELOAD'] = subprocess.check_output(
            [cc, '-print-file-name=libasan.so'], text=True).strip()
    return env


def rust_run(code, destination, expected, test_count, obj=None, required_failure=None):
    source = destination / 'harness.rs'
    source.write_text(code)
    binary = destination / 'tests'
    log = destination / 'compile-rust.log'
    args = [os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--test', source, '-o', binary]
    if obj:
        args += ['-C', 'link-arg=' + str(obj), '-C', 'link-arg=-lasan', '-C', 'link-arg=-lubsan']
    require_compile(command(args, log), log)
    result = command([binary, '--test-threads=1', '--nocapture'], destination / 'tests.log',
                     runtime_environment(bool(obj)))
    match = re.search(r'test result: (\w+)\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;', result.stdout)
    if not match:
        raise RuntimeError('Runtime failure without test summary; not a caught mutant: ' + str(destination))
    passed, failed = int(match[2]), int(match[3])
    if passed + failed != test_count or any(int(match[i]) for i in (4, 5, 6)):
        raise RuntimeError('Missing, ignored or filtered tests: ' + str(destination))
    failures = re.findall(r'^test (\S+) \.\.\. FAILED$', result.stdout, re.M)
    if expected == 'pass':
        if failed or result.returncode or passed == 0:
            raise RuntimeError('Production regression failed: ' + str(destination))
    elif result.returncode != 101 or not failed or not any(required_failure in n for n in failures):
        raise RuntimeError('Mutation did not produce the required semantic failure: ' + str(destination))
    return {'passed': passed, 'failed': failed, 'failed_tests': failures,
            'expected': expected, 'returncode': result.returncode,
            'compiled': True, 'harness_sha256': hashlib.sha256(code.encode()).hexdigest()}


def error_functions():
    error = (TREE / 'rust/kernel/error.rs').read_text()
    return ('pub mod error { use crate::prelude::*; use crate::bindings;\n' +
            function(error, 'pub fn from_err_ptr<T>') + '\n' +
            function(error, 'pub fn to_result(') + '\n}\n')


def shim_module(apple):
    text = (apple / 'shim.rs').read_text()
    return 'mod shim {\n' + text[:text.index('extern "C" {', text.index('impl Drop for StoreFile'))] + '\n}\n'


def base_common():
    return fixture('common.rs').split('pub struct FakeMutex')[0]


def crc_module(apple):
    return ('mod store { pub const MAX_VALUE:usize=4096;\n' +
            function((apple / 'store.rs').read_text(), 'pub(crate) const fn crc16_ccitt_false(') + '\n}\n')


def source_module(name, path):
    return '#[path=' + json.dumps(str(path)) + '] mod ' + name + ';\n'


def atomic_c(apple):
    c = (apple / 'store_shim.c').read_text()
    signatures = ['static int store_write_all(', 'static int store_remove_temp(',
                  'static int store_replace(', 'int sep_store_replace(',
                  'int sep_store_sync_parent(']
    c = fixture('atomic-vfs.h') + '\n'.join(function(c, s) for s in signatures)
    return c


def atomic_suite(apple, output):
    c = atomic_c(apple)
    obj = output / 'vfs.o'
    c_build(c, obj)
    if output.name == 'production':
        binary = output / 'boundaries'
        c_build(c + fixture('atomic.c'), binary, binary=True)
        result = command([binary], output / 'boundaries.log', runtime_environment(False))
        if result.returncode or 'C cases: 33 passed, 0 failed' not in result.stdout:
            raise RuntimeError('Atomic C boundary regression failed')
    common = base_common().replace('let _ = (&$dev, format_args!($($arg)*));', 'let _ = &$dev;')
    code = common + error_functions() + shim_module(apple) + crc_module(apple)
    reader = (apple / 'catacomb.rs').read_text()
    atomic = fixture('atomic.rs').replace('catacomb::read_checked(PRIVATE_TYPE_CATACOMB_USER)',
        'catacomb::read(PRIVATE_TYPE_CATACOMB_USER)' if 'fn read(kind: u8) -> Result<' in reader
        else 'Ok(catacomb::read(PRIVATE_TYPE_CATACOMB_USER))')
    code += source_module('catacomb', apple / 'catacomb.rs') + atomic
    code += '\nimpl SepData{\n' + function((apple / 'sbio.rs').read_text(), 'fn save_catacomb(') + '\n}\n'
    return code, obj


SUITES = {'atomic': atomic_suite}

TEST_COUNTS = {'atomic': 15}

MUTATIONS = {'atomic': ('catacomb.rs', 'shim::replace_file(path, &head, blob)', 'let file = shim::StoreFile::open_trunc(path)?;\nfile.write_all(0, &head)?;\nfile.write_all(HEADER as u64, blob)?;\nfile.sync()', 'partial_payload_error_preserves')}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', choices=SUITES, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    names = [p for p in APPLE.iterdir() if p.is_file() and p.suffix in ('.rs', '.c', '.h')]
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in names}
    report = {'suite': args.suite, 'hardware_access': False, 'production_source_hashes': hashes, 'variants': {}}
    for variant in ['production', 'negative-control']:
        output = args.output / variant
        output.mkdir()
        source = output / 'source'
        source.mkdir()
        for p in names: shutil.copy2(p, source / p.name)
        if variant == 'negative-control':
            name, old, new, failure = MUTATIONS[args.suite]
            p = source / name
            p.write_text(replace_once(p.read_text(), old, new))
        code, obj = SUITES[args.suite](source, output)
        report['variants'][variant] = rust_run(code, output, 'pass' if variant == 'production' else 'caught',
                                               TEST_COUNTS[args.suite], obj, None if variant == 'production' else failure)
    if hashes != {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in names}:
        raise RuntimeError('Production source tree changed during the test')
    report['status'] = 'PASS'
    (args.output / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'suite': args.suite, 'status': 'PASS', 'variants': report['variants']}, indent=2))


if __name__ == '__main__':
    main()
