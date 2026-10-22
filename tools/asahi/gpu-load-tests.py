#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""A transient kernel snapshot collision must not terminate load monitoring."""
import errno
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock

spec = importlib.util.spec_from_file_location('gpu_load', Path(__file__).with_name('gpu-load.py'))
gpu_load = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gpu_load)


class LoadTests(unittest.TestCase):
    def test_busy_snapshot_is_unavailable(self):
        path = Mock()
        path.parent.resolve.return_value = '/sys/devices/platform/gpu'
        path.read_text.side_effect = BlockingIOError(errno.EAGAIN, 'snapshot busy')
        sample = gpu_load.snapshot(path)
        self.assertFalse(sample['load_valid'])
        self.assertIsNone(sample['busy_percent'])

    def test_other_read_errors_are_reported(self):
        path = Mock()
        path.read_text.side_effect = PermissionError(errno.EACCES, 'permission denied')
        with self.assertRaises(PermissionError):
            gpu_load.snapshot(path)


if __name__ == '__main__':
    unittest.main()
