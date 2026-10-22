#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("gpu_memory_report",
                                            Path(__file__).with_name("gpu-memory-report.py"))
reporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reporter)


class MemoryReportTests(unittest.TestCase):
    def test_units_and_unavailable_residency(self):
        self.assertEqual(reporter.size_bytes("3 MiB"), 3 * 1024**2)
        self.assertEqual(reporter.size_bytes("17"), 17)
        self.assertEqual(reporter.size_bytes("64 KiB"), 65536)
        with self.assertRaises(ValueError):
            reporter.size_bytes("3 bananas")
        self.assertIsNone(reporter.parse_fdinfo("pos:\t0\n"))

    def test_shared_descriptors_are_not_recounted_as_allocations(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            node = root / "renderD128"
            node.touch()
            for pid, fds in [(100, [5, 6]), (200, [8])]:
                process = root / str(pid)
                (process / "fdinfo").mkdir(parents=True)
                (process / "fd").mkdir()
                (process / "comm").write_text(f"process-{pid}\n")
                for fd in fds:
                    (process / "fd" / str(fd)).symlink_to(node)
                    (process / "fdinfo" / str(fd)).write_text(
                        "drm-driver:\tasahi\ndrm-client-id:\t7\n"
                        "drm-total-memory:\t10 MiB\ndrm-shared-memory:\t2 MiB\n")
            result = reporter.report(root, root / "debug")
            self.assertEqual(len(result["clients"]), 1)
            client = result["clients"][0]
            self.assertEqual(client["fd_count"], 3)
            self.assertEqual(len(client["processes"]), 2)
            self.assertEqual(result["private_handle_bytes"], 8 * 1024**2)
            self.assertIsNone(client["memory_bytes"]["resident"])


if __name__ == "__main__":
    unittest.main()
