#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""load-driver keeps going when apple_sep is already loaded.

Rust module parameters are registered with perm 0, so they never show up
under /sys/module/apple_sep/parameters. Exiting there used to fail
aurora-sep.service and, because aurora-sep-ready Requires it, skip Touch
ID activation and fingerprint restore.
"""

import os
import stat
import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path

SEP = Path(__file__).resolve().parents[1]
LOAD = SEP / "load-driver"
INSTALL = SEP / "install-aurora-sep.sh"


def write_exe(path, body):
    path.write_text(textwrap.dedent(body).lstrip("\n"))
    path.chmod(path.stat().st_mode | stat.S_IEXEC)


class LoadDriverTest(unittest.TestCase):
    def run_loader(self, module_present, diag=None):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bindir = root / "bin"
            bindir.mkdir()
            module = root / "sys" / "module" / "apple_sep"
            if module_present:
                module.mkdir(parents=True)
            # parameters/ is absent on purpose: that is what the kernel exports.
            write_exe(bindir / "keyctl", """\
                #!/bin/sh
                if [ "$1" = session ]; then
                    # keyctl session <name> env VAR=1 <script>
                    export "${4%%=*}=${4#*=}"
                    exec "$5"
                fi
                exit 1
                """)
            write_exe(bindir / "modprobe", """\
                #!/bin/sh
                printf '%s\\n' "$*" >> "$MODPROBE_LOG"
                exit 0
                """)
            if diag is None:
                diag_env = "none"
            else:
                diag_path = root / "xart"
                diag_path.write_text(diag)
                diag_env = str(diag_path)
            env = os.environ.copy()
            env.update({
                "PATH": "%s:%s" % (bindir, env.get("PATH", "")),
                "AURORA_SEP_MODULE_DIR": str(module),
                "AURORA_SEP_DIAG_XART": diag_env,
                "AURORA_SEP_READY_TIMEOUT": "0",
                "AURORA_SEP_DEVICE_TIMEOUT": "0",
                "AURORA_SEP_RUN_DIR": str(root),
                "MODPROBE_LOG": str(root / "modprobe.log"),
                "AURORA_SEP_SESSION": "0",
            })
            proc = subprocess.run(
                [str(LOAD)], capture_output=True, text=True, env=env
            )
            modprobe_log = (root / "modprobe.log").read_text() if (root / "modprobe.log").is_file() else ""
            return proc, modprobe_log

    def test_already_loaded_without_sysfs_params_continues(self):
        proc, modprobe_log = self.run_loader(module_present=True)
        self.assertNotIn("reboot before expecting", proc.stderr)
        self.assertIn("did not seal", proc.stderr)
        self.assertEqual(modprobe_log, "")
        self.assertNotEqual(proc.returncode, 0)

    def test_missing_module_still_modprobes(self):
        proc, modprobe_log = self.run_loader(module_present=False)
        self.assertIn("apple_sep", modprobe_log)
        self.assertNotIn("reboot before expecting", proc.stderr)
        self.assertIn("did not seal", proc.stderr)

    def test_already_loaded_read_only_hints_and_continues(self):
        proc, modprobe_log = self.run_loader(module_present=True, diag="disabled\n")
        self.assertIn("diag xart=disabled", proc.stderr)
        self.assertIn("reboot before expecting", proc.stderr)
        self.assertIn("did not seal", proc.stderr)
        self.assertEqual(modprobe_log, "")
        self.assertNotEqual(proc.returncode, 0)

    def test_already_loaded_with_writes_does_not_hint(self):
        proc, modprobe_log = self.run_loader(module_present=True, diag="enabled\n")
        self.assertNotIn("reboot before expecting", proc.stderr)
        self.assertIn("did not seal", proc.stderr)
        self.assertEqual(modprobe_log, "")

    def test_modprobe_path_does_not_hint_when_diag_is_disabled(self):
        proc, modprobe_log = self.run_loader(module_present=False, diag="disabled\n")
        self.assertNotIn("reboot before expecting", proc.stderr)
        self.assertIn("apple_sep", modprobe_log)

    def test_does_not_write_the_packaged_modprobe_file(self):
        text = LOAD.read_text()
        self.assertNotIn("/usr/lib/modprobe.d/apple-sep.conf'", text)
        self.assertNotIn("printf '%s\\n' 'options apple_sep", text)
        self.assertIn("AURORA_SEP_MODULE_DIR", text)

    def test_read_only_install_uses_the_packaged_name(self):
        text = INSTALL.read_text()
        self.assertIn("MODPROBE_CONF=/etc/modprobe.d/apple-sep.conf", text)
        self.assertIn("MODPROBE_CONF_OLD=/etc/modprobe.d/aurora-sep.conf", text)
        self.assertNotIn("MODPROBE_CONF=/etc/modprobe.d/aurora-sep.conf", text)


if __name__ == "__main__":
    unittest.main()
