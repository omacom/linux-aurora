"""T6030 (MacBook Pro 14" M3 Pro, J514s) SEP and Mesa sensor device tree and profile.

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
BUS = "/soc/spi@289108000"
SENSOR = BUS + "/fingerprint@0"
PINCTRL_NUB = "/soc/pinctrl@3641f0000"
MBOX = "/soc/mbox@29e408000"
# AIC_IRQ cells: <type irq flags>, IRQ_TYPE_LEVEL_HIGH = 4.
LEVEL_HIGH = 4


@unittest.skipUnless(all(shutil.which(tool) for tool in ("cpp", DTC, "fdtget")),
                     "requires cpp, dtc and fdtget")
class T6030SepTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="aurora-t6030-sep-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.out = Path(cls.temp.name)

    def compile_board(self, board):
        pp = subprocess.run(["cpp", "-nostdinc", "-undef", "-D__DTS__", "-x",
                             "assembler-with-cpp", "-I", str(ROOT / "include"),
                             "-I", str(DTS), str(DTS / f"t6030-{board}.dts")],
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

    def cells(self, dtb, node, name):
        return [int(cell, 16) for cell in self.prop(dtb, node, name)]

    def irqs(self, dtb, node):
        cells = self.cells(dtb, node, "interrupts")
        self.assertEqual(len(cells) % 3, 0)
        triples = [cells[i:i + 3] for i in range(0, len(cells), 3)]
        for kind, _, flags in triples:
            self.assertEqual((kind, flags), (0, LEVEL_HIGH))
        return [irq for _, irq, _ in triples]

    def test_j514s_describes_sep_and_sensor(self):
        dtb = self.compile_board("j514s")
        # apple_sep enables the SEP and its DART at module load.
        self.assertEqual(self.prop(dtb, SEP_DART, "status", "s"), ["disabled"])
        self.assertEqual(self.prop(dtb, SEP, "status", "s"), ["disabled"])
        self.assertEqual(self.prop(dtb, MBOX, "status", "s"), ["okay"])
        # ADT vm-base 0x10000004000, vm-size 0xffff0000, as on t8122.
        self.assertEqual(self.cells(dtb, SEP_DART, "apple,dma-range"),
                         [0x100, 0x4000, 0, 0xffff0000])
        self.assertEqual(self.cells(dtb, SEP_DART, "apple,dma-offset"), [0, 0])
        self.assertEqual(self.prop(dtb, "/aliases", "sep", "s"), [SEP])

        self.assertEqual(self.prop(dtb, BUS, "status", "s"), ["okay"])
        self.assertEqual(self.prop(dtb, SENSOR, "status", "s"), ["okay"])
        self.assertEqual(self.prop(dtb, SENSOR, "firmware-name", "s"),
                         ["apple/mesacal-j514s.bin"])
        nub = int(self.prop(dtb, PINCTRL_NUB, "phandle")[0], 16)
        self.assertEqual(self.cells(dtb, SENSOR, "enable-gpios"), [nub, 33, 0])
        self.assertEqual(self.prop(dtb, SENSOR, "spi-max-frequency", "u"), ["8000000"])
        self.assertEqual(self.prop(dtb, SENSOR, "spi-cs-setup-delay-ns", "u"), ["20"])
        self.assertEqual(self.prop(dtb, SENSOR, "spi-cs-hold-delay-ns", "u"), ["20"])
        properties = self.props(dtb, SENSOR)
        self.assertIn("spi-cpha", properties)
        self.assertNotIn("spi-cpol", properties)
        self.assertNotIn("interrupts", properties)
        self.assertNotIn("cs-gpios", self.props(dtb, BUS))

    def test_addresses_and_interrupts_match_the_adt(self):
        # The J514s ADT values (system firmware 27.0), /arm-io base 0x210000000 added.
        dtb = self.compile_board("j514s")
        self.assertEqual(self.cells(dtb, SEP_DART, "reg"), [0x2, 0x9cac0000, 0, 0x4000])
        self.assertEqual(self.irqs(dtb, SEP_DART), [372])
        self.assertEqual(self.cells(dtb, SEP, "reg"), [0x2, 0x9e400000, 0, 0x6c000])
        self.assertEqual(self.cells(dtb, MBOX, "reg"), [0x2, 0x9e408000, 0, 0x4000])
        self.assertEqual(self.prop(dtb, MBOX, "interrupt-names", "s"),
                         ["send-empty", "send-not-empty", "recv-empty", "recv-not-empty"])
        self.assertEqual(self.irqs(dtb, MBOX), [366, 367, 368, 369])
        mbox = int(self.prop(dtb, MBOX, "phandle")[0], 16)
        dart = int(self.prop(dtb, SEP_DART, "phandle")[0], 16)
        self.assertEqual(self.cells(dtb, SEP, "mboxes"), [mbox])
        self.assertEqual(self.cells(dtb, SEP, "iommus"), [dart, 0])
        self.assertEqual(self.cells(dtb, BUS, "reg"), [0x2, 0x89108000, 0, 0x4000])
        self.assertEqual(self.irqs(dtb, BUS), [876])
        self.assertEqual(self.cells(dtb, SENSOR, "reg"), [0])

    def test_j516s_leaves_sep_and_sensor_off(self):
        dtb = self.compile_board("j516s")
        self.assertNotIn("sep", self.props(dtb, "/aliases"))
        self.assertEqual(self.prop(dtb, MBOX, "status", "s"), ["disabled"])
        self.assertEqual(self.prop(dtb, BUS, "status", "s"), ["disabled"])
        self.assertEqual(self.prop(dtb, SENSOR, "status", "s"), ["disabled"])

    def test_profile_selected_only_on_j514s(self):
        detect = PROFILE.split("pub(crate) fn detect()", 1)[1]
        line = next(line for line in detect.splitlines() if 'b"apple,t6030"' in line)
        self.assertIn('b"apple,j514s"', line)
        profile = re.search(r"const T6030: PlatformProfile = PlatformProfile \{(.*?)\n\};",
                            PROFILE, re.S).group(1)
        self.assertIn("..T8122", profile)
        # The diagnostic controller base is spi2's, not T8122's.
        self.assertIn("controller_base: 0x2_8910_8000", profile)


if __name__ == "__main__":
    unittest.main()
