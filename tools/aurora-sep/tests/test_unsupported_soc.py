"""Chips this release doesn't support stop before anything changes (issue #32).

The kernel carries device trees for the M3 Ultra (t6032) and M4 and later
chips (t8132, t8142, t8152). On those Macs the M1/M2 path would install
m1n1-aurora and rebuild boot.bin with an m1n1 that can't start them, so
install_all and uninstall_all refuse first. --agent-prompt still works.
Runs against the fake Mac of test_m3_flow.
"""
import re
import unittest

import test_m3_flow as flow

SRC = flow.INSTALLER.read_text()

# The fake Mac's helpers: M3FlowBase, or on older trees the test class that
# carries them, with its own tests switched off here.
Base = getattr(flow, "M3FlowBase", None)
if Base is None:
    Base = type("Base", (flow.M3FlowTest,),
                {n: None for n in dir(flow.M3FlowTest) if n.startswith("test")})

UNSUPPORTED = {
    "j575d": "t6032",   # Mac Studio M3 Ultra
    "j713": "t8132",    # M4
    "j813": "t8142",
    "j873g": "t8152",
}
SUPPORTED = {
    "j293": "t8103", "j314s": "t6000", "j314c": "t6001", "j375d": "t6002",
    "j413": "t8112", "j414s": "t6020", "j414c": "t6021", "j180d": "t6022",
    "j700": "t8140", "j613": "t8122", "j516s": "t6030", "j514c": "t6031", "j514m": "t6034",
}


class UnsupportedSocTest(Base):
    def setUp(self):
        super().setUp()
        for board, soc in {**UNSUPPORTED, **SUPPORTED}.items():
            if board not in flow.BOARDS:
                flow.BOARDS[board] = [f"apple,{board}", f"apple,{soc}", "apple,arm-platform"]
                self.addCleanup(flow.BOARDS.pop, board)

    def test_supported_list_matches_is_m3_and_is_neo(self):
        listed = set(re.search(r'^SUPPORTED_SOCS="([^"]*)"$', SRC, re.M).group(1).split())
        m3 = set(re.search(r"grep -Eqx 'apple,\(([^)]*)\)'", SRC).group(1).split("|"))
        self.assertTrue(m3 <= listed, m3 - listed)
        self.assertIn("t8140", listed)
        self.assertFalse(listed & set(UNSUPPORTED.values()))
        self.assertEqual(listed, set(SUPPORTED.values()))

    def test_supported_chips_pass(self):
        for board in SUPPORTED:
            with self.subTest(board=board):
                self.mac(board)
                self.assertEqual(self.run_sh("require_supported_soc && echo ok").stdout.strip(), "ok")

    def test_install_refuses_before_anything(self):
        for board, soc in UNSUPPORTED.items():
            for try_ in (0, 1):
                with self.subTest(board=board, try_=try_):
                    self.mac(board)
                    before = self.boot.read_bytes()
                    proc = self.run_sh(f"M3_TRY={try_}\ninstall_all", check=False)
                    self.assertNotEqual(proc.returncode, 0)
                    self.assertIn(f"apple,{soc}", proc.stderr)
                    self.assertIn("Nothing was changed", proc.stderr)
                    self.assertEqual(self.boot.read_bytes(), before)
                    self.assertEqual(self.downloaded(), [])
                    self.assertNotIn("pacman", self.log())
                    self.assertNotIn("update-m1n1", self.log())
                    self.assertFalse(self.m1n1_conf.exists())
                    self.assertFalse(self.state.exists() and any(self.state.iterdir()))

    def test_install_options_refuse_too(self):
        # An install with one of its options refuses the same way, before the option acts.
        for option in ("ESP_ARCHIVE_HISTORY=1", "DESKTOP_FIXES=1", "M3_PRO_MESA=0"):
            with self.subTest(option=option):
                self.mac("j713")
                before = self.boot.read_bytes()
                proc = self.run_sh(f"esp_history_run() {{ echo ARCHIVE; }}\n{option}\ninstall_all",
                                   check=False)
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn("apple,t8132", proc.stderr)
                self.assertNotIn("ARCHIVE", proc.stdout)
                self.assertEqual(self.boot.read_bytes(), before)
                self.assertEqual(self.downloaded(), [])

    def test_uninstall_refuses(self):
        for board, soc in UNSUPPORTED.items():
            with self.subTest(board=board):
                self.mac(board)
                before = self.boot.read_bytes()
                proc = self.run_sh("uninstall_all", check=False)
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn(f"apple,{soc}", proc.stderr)
                self.assertIn("Uninstalling", proc.stderr)
                self.assertEqual(self.boot.read_bytes(), before)
                self.assertNotIn("pacman", self.log())

    def test_no_chip_in_the_device_tree_refuses(self):
        self.mac("j713")
        (self.tmp / "dt/compatible").write_bytes(b"apple,j999\0apple,arm-platform\0")
        proc = self.run_sh("install_all", check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("Nothing was changed", proc.stderr)

    def test_agent_prompt_still_works(self):
        self.mac("j575d")
        proc = self.run_sh("agent_prompt >/dev/null && echo printed")
        self.assertEqual(proc.stdout.strip(), "printed")


if __name__ == "__main__":
    unittest.main()
