#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise apple-dart-pd-check.py with synthetic DTS fixtures; needs dtc + pylibfdt."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


CHECKER = Path(__file__).with_name("apple-dart-pd-check.py")

GAP = 'dart_gap: iommu@1000 { compatible = "apple,t8110-dart"; #iommu-cells = <1>; };'
NEW_GAP = 'iommu@2000 { compatible = "apple,t8110-dart"; #iommu-cells = <1>; };'
DOMAIN = 'ps_x: power-controller@3000 { #power-domain-cells = <0>; };'
OK_DART = 'iommu@4000 { compatible = "apple,t8110-dart"; power-domains = <&ps_x>; };'


def dts(board, soc, *nodes):
    return '''/dts-v1/;
        / { compatible = "apple,%s", "apple,%s", "apple,arm-platform";
            #address-cells = <1>; #size-cells = <0>;
            soc { #address-cells = <1>; #size-cells = <0>; %s };
        };
    ''' % (board, soc, " ".join(nodes))


class DartPdTest(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.TemporaryDirectory(prefix="dart-pd-test-", dir=Path.cwd())
        self.addCleanup(self.work.cleanup)
        self.root = Path(self.work.name)
        self.base = self.root / "base"
        self.head = self.root / "head"
        self.base.mkdir()
        self.head.mkdir()
        # The baseline J413 already has one gap (iommu@1000), like the 893 known ones.
        self.dtb(self.base, "t8112-j413", dts("j413", "t8112", DOMAIN, GAP, OK_DART))
        self.dtb(self.head, "t8112-j413", dts("j413", "t8112", DOMAIN, GAP, OK_DART))

    def dtb(self, directory, name, source):
        path = self.root / (name + ".dts")
        path.write_text(source)
        subprocess.run(["dtc", "-q", "-@", "-I", "dts", "-O", "dtb", "-o",
                        str(directory / (name + ".dtb")), str(path)], check=True)

    def check(self, *args):
        return subprocess.run([sys.executable, str(CHECKER), "--dtb-dir", str(self.head),
                               "--baseline-dtb-dir", str(self.base), *args],
                              text=True, capture_output=True)

    def test_unchanged_passes(self):
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("new: 0", result.stderr)

    def test_new_variant_inherits_its_boards_gap(self):
        # t8112-j413-fw27 includes t8112-j413.dts: the same gap is not new.
        self.dtb(self.head, "t8112-j413-fw27", dts("j413", "t8112", DOMAIN, GAP, OK_DART))
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn("(new)", result.stdout)
        self.assertIn("new: 0, inherited by new DTBs: 1", result.stderr)
        report = self.check("--report")
        self.assertIn("t8112-j413-fw27 /soc/iommu@1000 dart_gap apple,t8110-dart okay: "
                      "no power-domains (inherited from t8112-j413)", report.stdout)

    def test_new_variant_with_its_own_gap_fails(self):
        self.dtb(self.head, "t8112-j413-fw27",
                 dts("j413", "t8112", DOMAIN, GAP, OK_DART, NEW_GAP))
        result = self.check()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("t8112-j413-fw27 /soc/iommu@2000 - apple,t8110-dart okay: "
                      "no power-domains (new)", result.stdout)
        self.assertNotIn("iommu@1000", result.stdout)
        self.assertIn("new: 1, inherited by new DTBs: 1", result.stderr)

    def test_new_dtb_of_another_soc_inherits_nothing(self):
        self.dtb(self.head, "t8122-j613", dts("j613", "t8122", DOMAIN, GAP, OK_DART))
        result = self.check()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("t8122-j613 /soc/iommu@1000 dart_gap apple,t8110-dart okay: "
                      "no power-domains (new)", result.stdout)

    def test_new_gap_on_an_existing_board_fails(self):
        # An existing board never inherits from its SoC siblings.
        self.dtb(self.base, "t8112-j493", dts("j493", "t8112", DOMAIN, NEW_GAP))
        self.dtb(self.head, "t8112-j493", dts("j493", "t8112", DOMAIN, NEW_GAP))
        self.dtb(self.head, "t8112-j413",
                 dts("j413", "t8112", DOMAIN, GAP, OK_DART, NEW_GAP))
        result = self.check()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("t8112-j413 /soc/iommu@2000 - apple,t8110-dart okay: "
                      "no power-domains (new)", result.stdout)

    def test_disabled_domain_problem_must_match(self):
        # Same path and compatible, but a different problem: still new.
        disabled = DOMAIN.replace("{", '{ status = "disabled";')
        bad = 'iommu@1000 { compatible = "apple,t8110-dart"; power-domains = <&ps_x>; };'
        self.dtb(self.head, "t8112-j413-fw27", dts("j413", "t8112", disabled, bad))
        result = self.check()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("power domain /soc/power-controller@3000 is disabled (new)", result.stdout)

    def test_disabled_dart_on_disabled_domain_is_skipped(self):
        # The DART, its domain and its user are all off: consistent, not a finding.
        disabled = DOMAIN.replace("{", '{ status = "disabled";')
        dart = ('dart_x: iommu@5000 { compatible = "apple,t8110-dart"; #iommu-cells = <1>; '
                'power-domains = <&ps_x>; status = "disabled"; };')
        user = 'engine@6000 { iommus = <&dart_x 0>; status = "disabled"; };'
        self.dtb(self.head, "t8112-j413", dts("j413", "t8112", disabled, GAP, dart, user))
        result = self.check("--report")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn("iommu@5000", result.stdout)
        self.assertEqual(self.check().returncode, 0)

    def test_disabled_dart_under_a_disabled_user_is_skipped(self):
        # The user is okay itself but sits under a disabled node (a DCP's piodma).
        dart = ('dart_x: iommu@5000 { compatible = "apple,t8110-dart"; #iommu-cells = <1>; '
                'status = "disabled"; };')
        user = ('dcp@6000 { #address-cells = <1>; #size-cells = <0>; status = "disabled"; '
                'piodma { iommus = <&dart_x 0>; }; };')
        self.dtb(self.head, "t8112-j413", dts("j413", "t8112", DOMAIN, GAP, dart, user))
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn("iommu@5000", result.stdout)

    def test_dart_inside_a_disabled_node_is_skipped(self):
        # A router's DART, okay itself, inside the disabled router that is its only
        # user, on the router's disabled domain: it is never populated.
        disabled = DOMAIN.replace("{", '{ status = "disabled";')
        router = ('acio@5000 { #address-cells = <1>; #size-cells = <0>; status = "disabled"; '
                  'iommus = <&dart_x 0>; dart_x: iommu@a00 { compatible = "apple,t8110-dart"; '
                  '#iommu-cells = <1>; power-domains = <&ps_x>; }; };')
        self.dtb(self.head, "t8112-j413", dts("j413", "t8112", disabled, GAP, router))
        result = self.check()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn("iommu@a00", result.stdout)

    def test_dart_inside_an_enabled_node_on_a_disabled_domain_fails(self):
        disabled = DOMAIN.replace("{", '{ status = "disabled";')
        router = ('acio@5000 { #address-cells = <1>; #size-cells = <0>; '
                  'iommus = <&dart_x 0>; dart_x: iommu@a00 { compatible = "apple,t8110-dart"; '
                  '#iommu-cells = <1>; power-domains = <&ps_x>; }; };')
        self.dtb(self.head, "t8112-j413", dts("j413", "t8112", disabled, GAP, router))
        result = self.check()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("/soc/acio@5000/iommu@a00 dart_x apple,t8110-dart okay: "
                      "power domain /soc/power-controller@3000 is disabled (new)", result.stdout)

    def test_enabled_dart_on_disabled_or_missing_domain_fails(self):
        disabled = DOMAIN.replace("{", '{ status = "disabled";')
        on_disabled = ('iommu@5000 { compatible = "apple,t8110-dart"; '
                       'power-domains = <&ps_x>; };')
        self.dtb(self.head, "t8112-j413",
                 dts("j413", "t8112", disabled, GAP, on_disabled, NEW_GAP))
        result = self.check()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("t8112-j413 /soc/iommu@5000 - apple,t8110-dart okay: "
                      "power domain /soc/power-controller@3000 is disabled (new)", result.stdout)
        self.assertIn("t8112-j413 /soc/iommu@2000 - apple,t8110-dart okay: "
                      "no power-domains (new)", result.stdout)

    def test_disabled_dart_of_an_enabled_user_fails(self):
        # The loader enables such a DART when it hands its device over.
        disabled = DOMAIN.replace("{", '{ status = "disabled";')
        dart = ('dart_x: iommu@5000 { compatible = "apple,t8110-dart"; #iommu-cells = <1>; '
                'power-domains = <&ps_x>; status = "disabled"; };')
        user = 'engine@6000 { iommus = <&dart_x 0>; };'
        self.dtb(self.head, "t8112-j413", dts("j413", "t8112", disabled, GAP, dart, user))
        result = self.check()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("t8112-j413 /soc/iommu@5000 dart_x apple,t8110-dart disabled: "
                      "power domain /soc/power-controller@3000 is disabled (new)", result.stdout)

    def test_without_baseline_every_gap_fails(self):
        result = subprocess.run([sys.executable, str(CHECKER), "--dtb-dir", str(self.head)],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
