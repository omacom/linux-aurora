#!/usr/bin/env python3
"""The system-provider decision must see legacy selectors as overrides."""
from pathlib import Path
import shutil
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

    def test_bash_append_cannot_override_system_admission_silently(self):
        folder = self.root / "home/tester/.config/uwsm"
        folder.mkdir()
        config = folder / "env"
        cases = (("LIBGL_ALWAYS_SOFTWARE", "1"),
                 ("LD_LIBRARY_PATH", "/opt/custom/lib"))
        for name, value in cases:
            with self.subTest(variable=name):
                config.write_text("export " + name + "+=" + value + "\n")
                # The target's sh is Bash. This is a real assignment in
                # POSIX mode, although the legacy parser ignores it.
                shell = subprocess.run(
                    ["bash", "--posix", "-c",
                     'unset LIBGL_ALWAYS_SOFTWARE LD_LIBRARY_PATH; . "$1"; '
                     'printf "%s" "${!2}"', "bash", str(config), name],
                    env={"PATH": "/usr/bin:/bin"}, capture_output=True,
                    text=True, timeout=10)
                self.assertEqual(shell.returncode, 0, shell.stderr)
                self.assertEqual(shell.stdout, value)
                result = self.run_detector()
                self.assertEqual(result.returncode, 3, result.stdout)
                self.assertIn(name, result.stdout)
                self.assertIn("user_setup=unknown", result.stdout)
                # Keep the default legacy decision unchanged.
                self.assertEqual(self.run_detector(system=False).returncode, 0)

    def test_unrelated_bash_append_is_not_a_mesa_choice(self):
        folder = self.root / "home/tester/.config/uwsm"
        folder.mkdir()
        (folder / "env").write_text("export MY_TOOLS_PATH+=/tools\n")
        self.assertEqual(self.run_detector().returncode, 0)

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
        (self.root / "home/tester/provider").mkdir(parents=True)
        (self.root / "home/tester/provider/libEGL.so.1").touch()
        for value in ("$LD_LIBRARY_PATH", "$HOME/provider", "~/provider"):
            with self.subTest(value=value):
                self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": value}).returncode, 3)

    def test_conditional_append_needs_an_entry_before_it(self):
        folder = self.root / "home/tester/.config/uwsm"
        folder.mkdir(parents=True)
        for line in ("export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\n",
                     "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}\n",
                     "export LD_LIBRARY_PATH=/x:${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\n"):
            with self.subTest(line=line):
                (folder / "env").write_text(line)
                self.assertEqual(self.run_detector({"LD_LIBRARY_PATH": "/usr/lib/extra"}).returncode, 3)

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


class SystemAdmissionTest(unittest.TestCase):
    """_mm3_system_admit: which boards and nodes may keep the system Mesa.

    The helper is a stub that records its arguments; its own board parser is
    covered by test-system-board.c. Without a file opt-out the admission is
    automatic; the same checks apply either way.
    """

    BOARDS = {"j613": ("apple,j613", "apple,t8122"),
              "j615": ("apple,j615", "apple,t8122"),
              "j516s": ("apple,j516s", "apple,t6030")}

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.bin = self.base / "bin"
        self.bin.mkdir()
        journal = self.bin / "journalctl"
        journal.write_text('#!/bin/sh\n[ -z "${FAKE_FAULT:-}" ] || echo "$FAKE_FAULT"\n'
                           'echo "asahi 292400000.gpu: M3: firmware ready"\n')
        journal.chmod(0o755)

    def make_root(self, board="j615", node=True, display=True):
        compatibles = self.BOARDS.get(board, (board, "apple,t8122"))
        soc = compatibles[-1].split(",")[1]
        root = self.base / ("root-" + board)
        (root / "proc/device-tree").mkdir(parents=True)
        (root / "proc/cmdline").write_text("root=UUID=x rw quiet\n")
        (root / "proc/device-tree/compatible").write_bytes(
            b"".join(c.encode() + b"\0" for c in (*compatibles, "apple,arm-platform")))
        driver = root / "sys/bus/platform/drivers/asahi"
        driver.mkdir(parents=True)
        (root / "sys/class/drm").mkdir(parents=True)
        (root / "dev/dri").mkdir(parents=True)
        if node:
            gpu = root / "sys/devices/platform/gpu0"
            (gpu / "of_node").mkdir(parents=True)
            (gpu / "of_node/compatible").write_bytes(b"apple,agx-" + soc.encode() + b"\0")
            (gpu / "driver").symlink_to(driver)
            (root / "sys/class/drm/renderD128").mkdir()
            (root / "sys/class/drm/renderD128/device").symlink_to(gpu)
            (root / "dev/dri/renderD128").touch(mode=0o600)
        if display:
            dcp = root / "sys/devices/platform/display"
            (dcp / "of_node").mkdir(parents=True)
            (dcp / "of_node/compatible").write_bytes(
                b"apple," + soc.encode() + b"-display-subsystem\0")
            (root / "sys/class/drm/card2").mkdir()
            (root / "sys/class/drm/card2/device").symlink_to(dcp)
        prefix = root / "opt/mesa-m3"
        (prefix / "libexec").mkdir(parents=True)
        (prefix / "share/mesa-m3").mkdir(parents=True)
        (prefix / "lib").mkdir()
        (prefix / "lib/libgallium-26.1.4.so").touch()
        shutil.copy(RECIPE / "mesa-m3-user-setup", prefix / "libexec/mesa-m3-user-setup")
        shutil.copy(RECIPE / "user-setup.list", prefix / "share/mesa-m3/user-setup.list")
        helper = prefix / "libexec/mesa-m3-system-check"
        helper.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "${0%/*}/args"\n'
                          '[ ! -e "${0%/*}/refuse" ]\n')
        helper.chmod(0o755)
        (root / "home/tester/.config").mkdir(parents=True)
        return root

    def admit(self, root, **environment):
        hook = RECIPE / "mesa-m3-session-env"
        return subprocess.run(
            ["sh", "-c", 'MESA_M3_SESSION_ENV_NO_RUN=1 . "$1" && _mm3_p=/opt/mesa-m3 && '
             '_mm3_system_admit "$2"', "sh", str(hook), str(root)],
            env={"PATH": str(self.bin) + ":/usr/bin:/bin", "HOME": "/home/tester",
                 "XDG_CURRENT_DESKTOP": "Hyprland", **environment},
            capture_output=True, text=True, timeout=30)

    def helper_args(self, root):
        path = root / "opt/mesa-m3/libexec/args"
        return path.read_text().split() if path.exists() else None

    def test_supported_boards_are_admitted_without_opt_out(self):
        for board, chip in (("j613", "t8122"), ("j615", "t8122"), ("j516s", "t6030")):
            with self.subTest(board=board):
                root = self.make_root(board)
                result = self.admit(root)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn(f"({chip}, apple,{board})", result.stdout)
                self.assertEqual(self.helper_args(root), ["/dev/dri/renderD128", chip])

    def test_file_opt_out_still_admits(self):
        root = self.make_root("j615")
        (root / "etc/mesa-m3").mkdir(parents=True)
        (root / "etc/mesa-m3/disable").touch()
        self.assertEqual(self.admit(root).returncode, 0)

    def test_unknown_or_ambiguous_boards_are_refused(self):
        for board in ("apple,j504", "apple,j615s", "apple,j516s", "apple,j614s"):
            with self.subTest(board=board):
                root = self.make_root(board)
                self.assertEqual(self.admit(root).returncode, 1)
                self.assertIsNone(self.helper_args(root))
        root = self.make_root("j615")
        (root / "proc/device-tree/compatible").write_bytes(b"apple,j613\0apple,j615\0apple,t8122\0")
        self.assertEqual(self.admit(root).returncode, 1)
        self.assertIsNone(self.helper_args(root))

    def test_j615_without_its_nodes_is_refused(self):
        for node, display in ((False, True), (True, False)):
            with self.subTest(node=node, display=display):
                root = self.make_root("j615", node=node, display=display)
                self.assertEqual(self.admit(root).returncode, 1)
                self.assertIsNone(self.helper_args(root))
                shutil.rmtree(root)

    def test_kernel_command_line_refuses(self):
        for flag in ("mesa_m3=off", "nomodeset"):
            with self.subTest(flag=flag):
                root = self.make_root("j615")
                (root / "proc/cmdline").write_text(f"root=UUID=x rw {flag} quiet\n")
                self.assertEqual(self.admit(root).returncode, 1)
                self.assertIsNone(self.helper_args(root))
                shutil.rmtree(root)

    def test_helper_refusal_is_final(self):
        root = self.make_root("j615")
        (root / "opt/mesa-m3/libexec/refuse").touch()
        result = self.admit(root)
        self.assertEqual(result.returncode, 1)
        self.assertEqual(result.stdout, "")
        self.assertEqual(self.helper_args(root), ["/dev/dri/renderD128", "t8122"])

    def test_kernel_fault_and_other_desktops_refuse(self):
        root = self.make_root("j615")
        self.assertEqual(self.admit(root, FAKE_FAULT="asahi: M3 G15G: firmware crashed").returncode, 1)
        self.assertEqual(self.admit(root, XDG_CURRENT_DESKTOP="sway").returncode, 1)
        self.assertIsNone(self.helper_args(root))

    def test_native_25g83_profile_stays_on_its_gate_without_opt_out(self):
        root = self.make_root("j613")
        (root / "etc/mesa-m3").mkdir(parents=True)
        (root / "etc/mesa-m3/t8122-profile").write_text("j613-25g83-hal200\n")
        self.assertEqual(self.admit(root).returncode, 1)
        (root / "etc/mesa-m3/disable").touch()
        self.assertEqual(self.admit(root).returncode, 0)
        root = self.make_root("j615")
        handoff = root / "sys/devices/platform/gpu0/of_node/apple,j613-25g83-gpu-handoff"
        handoff.write_bytes(b"\0\0\0\1")
        self.assertEqual(self.admit(root).returncode, 1)
        (root / "etc/mesa-m3").mkdir(parents=True)
        (root / "etc/mesa-m3/disable").touch()
        self.assertEqual(self.admit(root).returncode, 0)

    def test_unconfirmed_attempt_of_this_boot_refuses(self):
        root = self.make_root("j615")
        (root / "proc/sys/kernel/random").mkdir(parents=True)
        (root / "proc/sys/kernel/random/boot_id").write_text("boot-a\n")
        state = root / "home/tester/.local/state/mesa-m3"
        state.mkdir(parents=True)
        (state / "attempt").write_text("schema=aurora.mesa-m3-attempt/2\nboot_id=boot-a\n"
                                       "session_id=5\nhook_run_id=run-1\n")
        self.assertEqual(self.admit(root).returncode, 1)
        (state / "session-ok").write_text("schema=aurora.mesa-m3-session-ok/1\nboot_id=boot-a\n"
                                          "session_id=5\nhook_run_id=run-1\n")
        self.assertEqual(self.admit(root).returncode, 0)


if __name__ == "__main__":
    unittest.main()
