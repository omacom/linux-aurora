#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run the production capture owner/parser with synthetic SPI and allocation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile


def function(text, signature):
    start = text.index(signature)
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def main():
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=here.parents[3] / 'drivers/soc/apple/sensor.rs')
    parser.add_argument('--expect-failures', type=int, choices=(0, 3), default=0)
    args = parser.parse_args()
    text = args.source.read_text()
    header = (here / 'prefix.rs').read_text()
    for name in ('CMD_LEN', 'READ_CMD_LEN', 'CMD_READ_PREFIX', 'CRC_LEN', 'MAX_CAPTURE'):
        pattern = r'(?:pub\(crate\) )?const ' + name + r'\s*:[^\n]+;'
        real = re.search(pattern, text).group(0).replace('pub(crate) ', '')
        fake = re.search(pattern, header).group(0)
        if re.sub(r'\s+', '', real) != re.sub(r'\s+', '', fake):
            raise SystemExit('Fixture constant differs from production: ' + name)
    start = text.index('pub(crate) struct Capture(')
    end = text.index('pub(crate) fn read_capture(', start)
    production = (function(text, 'fn crc16_ansi(') + '\n' + text[start:end] +
                  function(text, 'pub(crate) fn read_capture('))
    harness = header + '\n' + production + '\n' + (here / 'cases.rs').read_text()
    with tempfile.TemporaryDirectory(prefix='sep-capture-regression-') as name:
        temp = Path(name)
        source = temp / 'harness.rs'
        binary = temp / 'tests'
        source.write_text(harness)
        subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', '--test',
                        str(source), '-o', str(binary)], check=True, timeout=90)
        result = subprocess.run([str(binary), '--test-threads=1'], text=True,
                                capture_output=True, timeout=90)
    print(result.stdout, end='')
    print(result.stderr, end='')
    summary = re.search(r'test result: (\w+)\. (\d+) passed; (\d+) failed;', result.stdout)
    if not summary:
        raise SystemExit('Missing test summary')
    passed, failed = int(summary[2]), int(summary[3])
    expected_code = 101 if args.expect_failures else 0
    if (passed, failed, result.returncode) != (9 - args.expect_failures,
                                              args.expect_failures, expected_code):
        raise SystemExit('Unexpected regression result')
    print(json.dumps({'passed': passed, 'failed': failed,
                      'expected_failures': args.expect_failures,
                      'source_sha256': hashlib.sha256(text.encode()).hexdigest(),
                      'production_excerpts_sha256': hashlib.sha256(production.encode()).hexdigest(),
                      'hardware_access': False}, sort_keys=True))


if __name__ == '__main__':
    main()
