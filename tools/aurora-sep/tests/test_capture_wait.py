# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Execute the production capture wait with synthetic time, SPI and IRQ leaves.

No sensor is opened. Long idle windows take virtual time only. The timeout
mutation must compile and fail the delayed-finger assertion, not compilation.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest

APPLE = Path(__file__).resolve().parents[3] / 'drivers/soc/apple'
FIXTURE = Path(__file__).with_name('capture-wait.rs')


def item(source, signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def harness(mutation=None):
    source = (APPLE / 'sbio.rs').read_text()
    method = item(source, 'fn await_capture(')
    sep = (APPLE / 'sep.rs').read_text()
    constants = '\n'.join(re.findall(r'^const (?:ENROL_(?:POLL_MS|IRQ_WAIT_MS|CAPTURE_TIMEOUT_MS|POLL_ATTEMPTS)|VERIFY_CAPTURE_TIMEOUT_MS):[^;]+;', sep, re.M))
    kinds = item(sep, 'enum CaptureWait {')
    # The unmodified parent compiles too, so the before-fix run tests behavior.
    if 'enum CapturePurpose {' in sep:
        kinds += '\n' + item(sep, 'enum CapturePurpose {')
        call = 'self.await_capture(if verify { CapturePurpose::Verification } else { CapturePurpose::Enrollment })'
    else:
        call = 'self.await_capture()'
    if mutation == 'idle-timeout':
        old = 'let mut waiting_for_finger = matches!(purpose, CapturePurpose::Verification);'
        if method.count(old) != 1:
            raise AssertionError('timeout mutation no longer has one target')
        method = method.replace(old, 'let mut waiting_for_finger = false;')
    elif mutation == 'ignore-cancel':
        old = 'if !bio::capture_is_live('
        if method.count(old) != 2:
            raise AssertionError('cancellation mutation no longer has two checks')
        method = method.replace(old, 'if false && !bio::capture_is_live(')
    elif mutation == 'unbounded-verification':
        old = 'if matches!(purpose, CapturePurpose::Verification) && now_ns >= verification_deadline_ns {'
        if method.count(old) != 1:
            raise AssertionError('absolute timeout mutation no longer has one target')
        method = method.replace(old, 'if false {')
    elif mutation == 'late-progress-expiry':
        old = '            if now_ns >= deadline_ns {\n                return CaptureWait::Timeout;\n            }\n'
        if method.count(old) != 1:
            raise AssertionError('progress timeout mutation no longer has one target')
        method = method.replace(old, '')
        marker = '            if use_irq {\n                // Block on the data-ready line'
        if method.count(marker) != 1:
            raise AssertionError('progress timeout mutation lost its late target')
        method = method.replace(marker, old + '\n' + marker)
    return (FIXTURE.read_text().replace('/* PRODUCTION */', constants + '\n' + kinds)
            .replace('/* METHOD */', method)
            .replace('/* CALL */', call))


class CaptureWaitTests(unittest.TestCase):
    def run_variant(self, mutation=None):
        with tempfile.TemporaryDirectory(prefix='sep-capture-wait-') as name:
            path = Path(name)
            source = path / 'wait.rs'
            source.write_text(harness(mutation))
            built = subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021',
                                    '--test', str(source), '-o', str(path / 'tests')],
                                   text=True, capture_output=True, timeout=60)
            self.assertEqual(built.returncode, 0, built.stderr)
            result = subprocess.run([str(path / 'tests'), '--test-threads=1'],
                                    text=True, capture_output=True, timeout=30)
            if mutation:
                self.assertEqual(result.returncode, 101, result.stdout + result.stderr)
                expected = {
                    'idle-timeout': 'delayed_finger_stays_in_one_capture',
                    'ignore-cancel': 'cancellation_and_epoch_change_are_bounded',
                    'unbounded-verification': 'unattended_verification_has_absolute_upper_bound',
                    'late-progress-expiry': 'late_initial_arming_cannot_reopen_the_progress_budget',
                }[mutation]
                self.assertIn('test ' + expected + ' ... FAILED', result.stdout)
            else:
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn('31 passed; 0 failed; 0 ignored', result.stdout)

    def test_production(self):
        self.run_variant()

    def test_timeout_negative_control(self):
        self.run_variant('idle-timeout')

    def test_cancellation_negative_control(self):
        self.run_variant('ignore-cancel')

    def test_absolute_timeout_negative_control(self):
        self.run_variant('unbounded-verification')

    def test_progress_expiry_order_negative_control(self):
        self.run_variant('late-progress-expiry')


if __name__ == '__main__':
    unittest.main()
