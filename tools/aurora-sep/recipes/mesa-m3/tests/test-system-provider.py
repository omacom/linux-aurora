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

    def test_loader_only_directories_are_another_provider(self):
        for library in ("libEGL.so.1", "libGLESv2.so.2", "libGL.so.1", "libGLX_mesa.so.0",
                        "libglapi.so.0", "libgbm.so.1", "gbm/dri_gbm.so"):
            with self.subTest(library=library):
                folder = self.root / "provider" / library.replace("/", "-")
                (folder / library).parent.mkdir(parents=True, exist_ok=True)
                (folder / library).touch()
                value = {"LD_LIBRARY_PATH": "/provider/" + library.replace("/", "-")}
                self.assertEqual(self.run_detector(value, system=False).returncode, 0)
                self.assertEqual(self.run_detector(value).returncode, 1)

    def test_unrelated_library_directory_stays_clean(self):
        (self.root / "provider/other").mkdir(parents=True)
        (self.root / "provider/other/libfoo.so.1").touch()
        self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": "/provider/other"}).returncode, 0)

    def test_relative_loader_entry_is_unknown(self):
        for value in ("lib", "./lib", "/usr/lib/extra:lib"):
            with self.subTest(value=value):
                self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": value}, system=False).returncode, 0)
                self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": value}).returncode, 3)

    def test_loader_path_parsed_like_glibc(self):
        (self.root / "provider/other").mkdir(parents=True)
        (self.root / "provider/evil").mkdir(parents=True)
        (self.root / "provider/evil/libEGL.so.1").touch()
        for value, code in ((":", 3), ("/provider/other:", 3), (":/provider/other", 3),
                            ("/provider/other::/usr/lib/extra", 3),
                            ("/provider/other;/provider/evil", 1)):
            with self.subTest(value=value):
                self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": value}).returncode, code)

    def test_previous_value_expands_like_the_shell(self):
        folder = self.root / "home/tester/.config/uwsm"
        folder.mkdir(parents=True)
        (self.root / "provider/other").mkdir(parents=True)
        plain_after = "export LD_LIBRARY_PATH=/provider/other:$LD_LIBRARY_PATH\n"
        plain_before = "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/provider/other\n"
        conditional = "export LD_LIBRARY_PATH=/provider/other${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\n"
        other_form = "export LD_LIBRARY_PATH=/provider/other:${LD_LIBRARY_PATH:-/x}\n"
        nested = "export LD_LIBRARY_PATH=/provider/other${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}\n"
        # An earlier file can clear the inherited value, so a plain reference
        # counts as possibly empty even when the session inherits one.
        for inherited in (None, "", "/usr/lib/extra"):
            expect = {plain_after: 3, plain_before: 3, conditional: 0, nested: 0}
            environment = {} if inherited is None else {"LD_LIBRARY_PATH": inherited}
            for line, code in expect.items():
                with self.subTest(inherited=inherited, line=line):
                    (folder / "env").write_text(line)
                    self.assertEqual(self.run_detector(environment).returncode, code)
        (folder / "env").write_text(other_form)
        self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": "/usr/lib/extra"}).returncode, 3)
        (folder / "env").write_text(conditional)
        self.assertEqual(self.run_detector(system=False).returncode, 0)

    def test_empty_inherited_value_is_no_entry(self):
        self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": ""}).returncode, 0)

    def test_reference_with_a_suffix_is_unknown(self):
        (self.root / "provider/suffix").mkdir(parents=True)
        (self.root / "provider/suffix/libEGL.so.1").touch()
        folder = self.root / "home/tester/.config/uwsm"
        folder.mkdir(parents=True)
        for line in ("export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}/suffix\n",
                     "export LD_LIBRARY_PATH=/a${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}/suffix\n",
                     "export LD_LIBRARY_PATH=/prefix$LD_LIBRARY_PATH\n"):
            with self.subTest(line=line):
                (folder / "env").write_text(line)
                self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": "/provider"}).returncode, 3)

    def test_environment_value_is_literal(self):
        self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": "$LD_LIBRARY_PATH"}).returncode, 3)

    def test_legacy_ldpath_rules_are_unchanged(self):
        (self.root / "provider/mesa").mkdir(parents=True)
        (self.root / "provider/mesa/libgallium-26.so").touch()
        folder = self.root / "home/tester/.config/uwsm"
        folder.mkdir(parents=True)
        for line, code in (("export LD_LIBRARY_PATH=/x:$LD_LIBRARY_PATH\n", 0),
                           ("export LD_LIBRARY_PATH=/provider/mesa:$LD_LIBRARY_PATH\n", 1),
                           ("export LD_LIBRARY_PATH=lib:$LD_LIBRARY_PATH\n", 0),
                           ("export LD_LIBRARY_PATH=/x::$LD_LIBRARY_PATH\n", 0)):
            with self.subTest(line=line):
                (folder / "env").write_text(line)
                self.assertEqual(self.run_detector(system=False).returncode, code)

    def test_spaces_are_a_relative_entry(self):
        for value in (" ", "/usr/lib/extra: "):
            with self.subTest(value=value):
                self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": value}).returncode, 3)

    def test_provider_and_version_overrides(self):
        for variable, value in (("GBM_BACKEND", "nvidia-drm"),
                                ("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
                                ("MESA_GL_VERSION_OVERRIDE", "4.6"),
                                ("MESA_GLES_VERSION_OVERRIDE", "3.2"),
                                ("MESA_GLSL_VERSION_OVERRIDE", "460")):
            with self.subTest(variable=variable):
                self.assertEqual(self.run_detector({variable: value}, system=False).returncode, 0)
                self.assertEqual(self.run_detector({variable: value}).returncode, 1)
                folder = self.root / "home/tester/.config/hypr"
                folder.mkdir(parents=True, exist_ok=True)
                (folder / "env.conf").write_text(f"env = {variable},{value}\n")
                self.assertEqual(self.run_detector().returncode, 1)
                (folder / "env.conf").unlink()

    def test_missing_file_does_not_mask_unresolved_choice(self):
        folder = self.root / "home/tester/.config/uwsm"
        folder.mkdir()
        (folder / "env").write_text("VK_DRIVER_FILES=$OTHER_MESA\n")
        result = self.run_detector()
        self.assertNotEqual(result.returncode, 0, result.stdout)


if __name__ == "__main__":
    unittest.main()
