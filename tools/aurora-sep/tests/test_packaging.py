#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""The packaging must describe the files in this tree, not stale copies."""

import hashlib
import re
import unittest
from pathlib import Path

SEP = Path(__file__).resolve().parents[1]
ARCH = SEP / "packaging" / "arch"
FEDORA = SEP / "packaging" / "fedora"


def bash_array(text, name):
    match = re.search(r"^%s=\((.*?)\)" % re.escape(name), text, re.M | re.S)
    if match is None:
        raise AssertionError("no %s array" % name)
    return re.findall(r"'([^']*)'|\"([^\"]*)\"|(\S+)", match.group(1))


def words(text, name):
    return ["".join(parts) for parts in bash_array(text, name)]


class PkgbuildTest(unittest.TestCase):
    def check(self, directory):
        text = (directory / "PKGBUILD").read_text()
        sources = words(text, "source")
        sums = words(text, "sha256sums")
        self.assertEqual(len(sources), len(sums))
        pkgver = re.search(r"^pkgver=(\S+)$", text, re.M).group(1)
        for source, expected in zip(sources, sums):
            if "://" in source:
                continue
            name = source.replace("$pkgver", pkgver)
            path = directory / name
            self.assertTrue(path.is_file(), "%s is not a file" % path)
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(
                digest, expected, "%s changed: run updpkgsums in %s" % (name, directory)
            )

    def test_aurora_sep_sums_match_tree(self):
        self.check(ARCH / "aurora-sep")

    def test_aurora_touchid_sums_match_tree(self):
        self.check(SEP / "aurora-touchid")

    def test_aurora_sep_ships_the_touchid_option_file(self):
        text = (ARCH / "aurora-sep" / "PKGBUILD").read_text()
        self.assertIn("apple-sep.conf", words(text, "source"))
        self.assertIn("aurora-touchid", words(text, "conflicts"))
        self.assertIn(
            '"$pkgdir/usr/lib/modprobe.d/apple-sep.conf"', text
        )
        option = "options apple_sep xart_writes=1 provision_keybag=1"
        packaged = (ARCH / "aurora-sep" / "apple-sep.conf").read_text()
        touchid = (SEP / "aurora-touchid" / "apple-sep.conf").read_text()
        self.assertIn(option, packaged)
        self.assertEqual(packaged, touchid)

    def test_fedora_ships_the_same_option_file(self):
        text = (FEDORA / "aurora-sep.spec").read_text()
        sources = re.findall(r"^Source\d+:\s+(\S+)$", text, re.M)
        self.assertIn("apple-sep.conf", sources)
        self.assertRegex(text, r"(?m)^Conflicts:\s+aurora-touchid$")
        self.assertIn("%{_prefix}/lib/modprobe.d/apple-sep.conf", text)
        self.assertIn(
            "options apple_sep xart_writes=1 provision_keybag=1",
            (SEP / "apple-sep.conf").read_text(),
        )


class FedoraSpecTest(unittest.TestCase):
    def test_sources_are_in_tree(self):
        text = (FEDORA / "aurora-sep.spec").read_text()
        sources = re.findall(r"^Source\d+:\s+(\S+)$", text, re.M)
        self.assertTrue(sources)
        for source in sources:
            self.assertTrue((SEP / source).is_file(), source)

    def test_selinux_module_labels_the_loader_device(self):
        path, kind, context = (SEP / "aurora_sep.fc").read_text().split()
        self.assertEqual(kind, "-c")
        self.assertIn(path, (SEP / "load-driver").read_text())
        label = re.search(r":(\w+_t),", context).group(1)
        te = (SEP / "aurora_sep.te").read_text()
        self.assertRegex(te, r"(?m)^type %s;$" % label)

    def test_build_script_builds_only_aurora_sep(self):
        text = (FEDORA / "build.sh").read_text()
        self.assertIn("aurora-sep.spec", text)
        self.assertNotIn("dnf -y download --source libfprint", text)


if __name__ == "__main__":
    unittest.main()
