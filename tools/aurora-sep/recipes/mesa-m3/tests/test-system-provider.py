#!/usr/bin/env python3
"""The system-provider decision must see legacy selectors as overrides."""
from pathlib import Path
import subprocess
import tempfile
import unittest

RECIPE = Path(__file__).resolve().parents[1]


class SystemProviderTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.home = "/home/tester"
        (self.root / "home/tester/.config").mkdir(parents=True)
        prefix = self.root / "opt/mesa-m3/lib"
        prefix.mkdir(parents=True)
        (prefix / "libgallium-26.1.4.so").touch()

    def run_detector(self, environment=None, system=True):
        command = [str(RECIPE / "mesa-m3-user-setup"), "--home", self.home,
                   "--root", str(self.root), "--list", str(RECIPE / "user-setup.list"),
                   "--session-desktops", "Hyprland", "--env"]
        if system:
            command.append("--system-provider")
        return subprocess.run(command, env={"PATH": "/usr/bin:/bin", "HOME": self.home,
                                            **(environment or {})},
                              capture_output=True, text=True, timeout=10)

    def test_clean_setup(self):
        self.assertEqual(self.run_detector().returncode, 0)

    def test_legacy_path_default_stays_accepted(self):
        value = {"VK_DRIVER_FILES": "/opt/mesa-m3/share/vulkan/icd.d/asahi.json"}
        self.assertEqual(self.run_detector(value, system=False).returncode, 0)
        self.assertEqual(self.run_detector(value).returncode, 1)

    def test_legacy_loader_path_is_not_system_mesa(self):
        value = {"LD_LIBRARY_PATH": "/opt/mesa-m3/lib"}
        self.assertEqual(self.run_detector(value, system=False).returncode, 0)
        self.assertEqual(self.run_detector(value).returncode, 1)

    def test_later_config_can_reselect_legacy_mesa(self):
        folder = self.root / "home/tester/.config/uwsm"
        folder.mkdir()
        (folder / "env").write_text("export LIBGL_DRIVERS_PATH=/opt/mesa-m3/lib/dri\n")
        self.assertEqual(self.run_detector(system=False).returncode, 0)
        self.assertEqual(self.run_detector().returncode, 1)

    def test_explicit_software_choice(self):
        self.assertEqual(self.run_detector({"LIBGL_ALWAYS_SOFTWARE": "1"}).returncode, 1)

    def test_only_owned_software_pair_is_ignored(self):
        self.assertEqual(self.run_detector({"LIBGL_ALWAYS_SOFTWARE": "1",
                                           "MESA_M3_FALLBACK": "software"}).returncode, 0)

    def test_preload_and_legacy_flags_are_not_clean_system_environment(self):
        for variable in ("LD_PRELOAD", "LD_AUDIT", "MESA_M3_SESSION",
                         "ASAHI_M3_EXPERIMENTAL", "AGX_MESA_DEBUG"):
            with self.subTest(variable=variable):
                # File assignments avoid loading a preload into the test
                # interpreter itself while exercising the same detector.
                folder = self.root / "home/tester/.config/uwsm"
                folder.mkdir(exist_ok=True)
                (folder / "env").write_text(variable + "=/opt/mesa-m3/choice\n")
                self.assertEqual(self.run_detector().returncode, 1)

    def test_missing_file_does_not_mask_unresolved_choice(self):
        folder = self.root / "home/tester/.config/uwsm"
        folder.mkdir()
        (folder / "env").write_text("VK_DRIVER_FILES=$OTHER_MESA\n")
        result = self.run_detector()
        self.assertNotEqual(result.returncode, 0, result.stdout)


if __name__ == "__main__":
    unittest.main()
