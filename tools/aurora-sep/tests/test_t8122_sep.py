"""T8122 (MacBook Air M3) SEP and Mesa sensor device tree and profile (issue #79).

Set DTC to this kernel's built scripts/dtc/dtc for Apple floating-point cells.
"""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
DTS = ROOT / "arch/arm64/boot/dts/apple"
DTC = os.environ.get("DTC", "dtc")
PROFILE = (ROOT / "drivers/soc/apple/profile.rs").read_text()

SEP_DART = "/soc/iommu@29cac0000"
SEP = "/soc/sep@29e400000"
BUS = "/soc/spi@2a1108000"
SENSOR = BUS + "/fingerprint@0"
PINCTRL_NUB = "/soc/pinctrl@2e41f0000"
MBOX = "/soc/mbox@29e408000"
# AIC_IRQ cells: <type irq flags>, IRQ_TYPE_LEVEL_HIGH = 4.
LEVEL_HIGH = 4


@unittest.skipUnless(all(shutil.which(tool) for tool in ("cpp", DTC, "fdtget")),
                     "requires cpp, dtc and fdtget")
class T8122SepTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="aurora-t8122-sep-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.out = Path(cls.temp.name)

    def compile_board(self, board):
        pp = subprocess.run(["cpp", "-nostdinc", "-undef", "-D__DTS__", "-x",
                             "assembler-with-cpp", "-I", str(ROOT / "include"),
                             "-I", str(DTS), str(DTS / f"t8122-{board}.dts")],
                            check=True, capture_output=True)
        output = self.out / f"{board}.dtb"
        subprocess.run([DTC, "-q", "-I", "dts", "-O", "dtb", "-o", str(output)],
                       input=pp.stdout, check=True, capture_output=True)
        return output

    def prop(self, dtb, node, name, kind="x"):
        return subprocess.run(["fdtget", "-t", kind, str(dtb), node, name],
                              check=True, capture_output=True, text=True).stdout.split()

    def props(self, dtb, node):
        return subprocess.run(["fdtget", "-p", str(dtb), node],
                              check=True, capture_output=True, text=True).stdout.split()

    def test_air_boards_describe_sep_and_sensor(self):
        for board in ("j613", "j615", "j613-25g83", "j615-25g83"):
            with self.subTest(board=board):
                dtb = self.compile_board(board)
                # apple_sep enables the SEP and its DART at module load.
                self.assertEqual(self.prop(dtb, SEP_DART, "status", "s"), ["disabled"])
                self.assertEqual(self.prop(dtb, SEP, "status", "s"), ["disabled"])
                window = [int(cell, 16) for cell in self.prop(dtb, SEP_DART, "apple,dma-range")]
                offset = [int(cell, 16) for cell in self.prop(dtb, SEP_DART, "apple,dma-offset")]
                # ADT vm-base 0x10000004000, vm-size 0xffff0000, as on t8140.
                self.assertEqual(window, [0x100, 0x4000, 0, 0xffff0000])
                self.assertEqual(offset, [0, 0])
                self.assertEqual(self.prop(dtb, "/aliases", "sep", "s"), [SEP])

                self.assertEqual(self.prop(dtb, BUS, "status", "s"), ["okay"])
                self.assertEqual(self.prop(dtb, SENSOR, "status", "s"), ["okay"])
                name = board.split("-")[0]
                self.assertEqual(self.prop(dtb, SENSOR, "firmware-name", "s"),
                                 [f"apple/mesacal-{name}.bin"])
                power = [int(value, 16) for value in self.prop(dtb, SENSOR, "enable-gpios")]
                nub = int(self.prop(dtb, PINCTRL_NUB, "phandle")[0], 16)
                self.assertEqual(power, [nub, 29, 0])
                self.assertEqual(self.prop(dtb, SENSOR, "spi-max-frequency", "u"), ["8000000"])
                self.assertEqual(self.prop(dtb, SENSOR, "spi-cs-setup-delay-ns", "u"), ["20"])
                self.assertEqual(self.prop(dtb, SENSOR, "spi-cs-hold-delay-ns", "u"), ["20"])
                properties = self.props(dtb, SENSOR)
                self.assertIn("spi-cpha", properties)
                self.assertNotIn("spi-cpol", properties)
                self.assertNotIn("interrupts", properties)
                self.assertNotIn("interrupts-extended", properties)
                self.assertNotIn("cs-gpios", self.props(dtb, BUS))

    def cells(self, dtb, node, name):
        return [int(cell, 16) for cell in self.prop(dtb, node, name)]

    def irqs(self, dtb, node):
        cells = self.cells(dtb, node, "interrupts")
        self.assertEqual(len(cells) % 3, 0)
        triples = [cells[i:i + 3] for i in range(0, len(cells), 3)]
        for kind, _, flags in triples:
            self.assertEqual((kind, flags), (0, LEVEL_HIGH))
        return [irq for _, irq, _ in triples]

    def test_addresses_and_interrupts_match_the_adt(self):
        # The J613/J615 ADT values (26.6.2), /arm-io base 0x210000000 added.
        dtb = self.compile_board("j613")
        self.assertEqual(self.cells(dtb, SEP_DART, "reg"), [0x2, 0x9cac0000, 0, 0x4000])
        self.assertEqual(self.irqs(dtb, SEP_DART), [324])
        self.assertEqual(self.cells(dtb, SEP, "reg"), [0x2, 0x9e400000, 0, 0x6c000])
        self.assertEqual(self.cells(dtb, MBOX, "reg"), [0x2, 0x9e408000, 0, 0x4000])
        self.assertEqual(self.prop(dtb, MBOX, "interrupt-names", "s"),
                         ["send-empty", "send-not-empty", "recv-empty", "recv-not-empty"])
        self.assertEqual(self.irqs(dtb, MBOX), [318, 319, 320, 321])
        self.assertEqual(self.prop(dtb, MBOX, "compatible", "s"),
                         ["apple,t8122-asc-mailbox", "apple,asc-mailbox-v4"])
        mbox = int(self.prop(dtb, MBOX, "phandle")[0], 16)
        dart = int(self.prop(dtb, SEP_DART, "phandle")[0], 16)
        self.assertEqual(self.cells(dtb, SEP, "mboxes"), [mbox])
        self.assertEqual(self.cells(dtb, SEP, "iommus"), [dart, 0])
        self.assertEqual(self.cells(dtb, BUS, "reg"), [0x2, 0xa1108000, 0, 0x4000])
        self.assertEqual(self.irqs(dtb, BUS), [767])
        self.assertEqual(self.cells(dtb, SENSOR, "reg"), [0])

    def test_other_t8122_boards_leave_sep_and_sensor_off(self):
        for board in ("j433", "j434", "j504"):
            with self.subTest(board=board):
                dtb = self.compile_board(board)
                self.assertNotIn("sep", self.props(dtb, "/aliases"))
                self.assertEqual(self.prop(dtb, BUS, "status", "s"), ["disabled"])
                self.assertEqual(self.prop(dtb, SENSOR, "status", "s"), ["disabled"])

    def test_profile_selected_only_on_the_airs(self):
        detect = PROFILE.split("pub(crate) fn detect()", 1)[1]
        line = next(line for line in detect.splitlines() if 'b"apple,t8122"' in line)
        self.assertIn('b"apple,j613"', line)
        self.assertIn('b"apple,j615"', line)
        profile = re.search(r"const T8122: PlatformProfile = PlatformProfile \{(.*?)\n\};",
                            PROFILE, re.S).group(1)
        self.assertIn("wide_dma_mask: true", profile)
        self.assertIn("..T8112", profile)
        # The diagnostic controller base is spi2's, not T8112's.
        self.assertIn("controller_base: 0x2_a110_8000", profile)


if __name__ == "__main__":
    unittest.main()
