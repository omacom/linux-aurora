#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""The old read-only modprobe name is renamed, and both names still count."""

import subprocess
import tempfile
import unittest
from pathlib import Path

SEP = Path(__file__).resolve().parents[1]
INSTALL = SEP / "install-aurora-sep.sh"
TOUCHID_INSTALL = SEP / "aurora-touchid" / "aurora-touchid.install"


class MigrateTest(unittest.TestCase):
    def migrate_with_installer(self, root):
        etc = root / "etc" / "modprobe.d"
        etc.mkdir(parents=True, exist_ok=True)
        script = """
set -euo pipefail
AURORA_SEP_SOURCE_ONLY=1 source '%s'
sudo=""
MODPROBE_CONF='%s'
MODPROBE_CONF_OLD='%s'
sep_migrate_modprobe_conf
sep_read_only_conf
""" % (INSTALL, etc / "apple-sep.conf", etc / "aurora-sep.conf")
        proc = subprocess.run(
            ["bash", "-c", script], capture_output=True, text=True
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)
        return proc.stdout.strip()

    def migrate_with_package(self, root):
        etc = root / "etc" / "modprobe.d"
        etc.mkdir(parents=True, exist_ok=True)
        text = TOUCHID_INSTALL.read_text().replace(
            "/etc/modprobe.d", str(etc)
        )
        text += "\n_migrate_read_only_modprobe\n"
        proc = subprocess.run(
            ["bash", "-c", text], capture_output=True, text=True
        )
        self.assertEqual(proc.returncode, 0, proc.stderr)

    def test_installer_renames_the_old_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = root / "etc" / "modprobe.d" / "aurora-sep.conf"
            new = root / "etc" / "modprobe.d" / "apple-sep.conf"
            self.migrate_with_installer(root)  # creates the directory
            old.write_text("options apple_sep xart_writes=0 provision_keybag=0\n")
            seen = self.migrate_with_installer(root)
            self.assertFalse(old.exists())
            self.assertEqual(
                new.read_text(),
                "options apple_sep xart_writes=0 provision_keybag=0\n",
            )
            self.assertEqual(seen, str(new))

    def test_installer_drops_the_old_name_when_both_exist(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            etc = root / "etc" / "modprobe.d"
            self.migrate_with_installer(root)
            old = etc / "aurora-sep.conf"
            new = etc / "apple-sep.conf"
            old.write_text("old\n")
            new.write_text("new\n")
            seen = self.migrate_with_installer(root)
            self.assertFalse(old.exists())
            self.assertEqual(new.read_text(), "new\n")
            self.assertEqual(seen, str(new))

    def test_installer_still_sees_the_old_name_before_migration(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            etc = root / "etc" / "modprobe.d"
            etc.mkdir(parents=True)
            old = etc / "aurora-sep.conf"
            old.write_text("options apple_sep xart_writes=0 provision_keybag=0\n")
            script = """
set -euo pipefail
AURORA_SEP_SOURCE_ONLY=1 source '%s'
sudo=""
MODPROBE_CONF='%s'
MODPROBE_CONF_OLD='%s'
sep_read_only_conf
""" % (INSTALL, etc / "apple-sep.conf", old)
            proc = subprocess.run(
                ["bash", "-c", script], capture_output=True, text=True
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertEqual(proc.stdout.strip(), str(old))
            self.assertTrue(old.exists())

    def test_package_renames_like_the_installer(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = root / "etc" / "modprobe.d" / "aurora-sep.conf"
            new = root / "etc" / "modprobe.d" / "apple-sep.conf"
            (root / "etc" / "modprobe.d").mkdir(parents=True)
            old.write_text("options apple_sep xart_writes=0 provision_keybag=0\n")
            self.migrate_with_package(root)
            self.assertFalse(old.exists())
            self.assertIn("xart_writes=0", new.read_text())

    def test_uninstall_removes_both_names(self):
        text = INSTALL.read_text()
        self.assertIn(
            'rm -f "$MODPROBE_CONF" "$MODPROBE_CONF_OLD"', text
        )

    def test_reset_uses_either_name(self):
        text = INSTALL.read_text()
        self.assertIn("ro=$(sep_read_only_conf)", text)
        self.assertNotIn("[[ -f $MODPROBE_CONF ]] &&", text)


if __name__ == "__main__":
    unittest.main()
