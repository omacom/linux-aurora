"""Run the production credential guard and its focused cleanup tests on the host."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


@unittest.skipUnless(shutil.which("rustc"), "rustc required")
class ContextCleanupTests(unittest.TestCase):
    def test_production_context_disposal(self):
        source = (ROOT / "drivers/soc/apple/key.rs").read_text()
        trait = source[source.index("pub(crate) trait ContextDelete"):
                       source.index("impl ContextDelete for SepData")]
        guard = source[source.index("/// A credential context"):
                       source.index("/// Self-test copies")]
        tests = source[source.index("    struct Deleter {"):
                       source.index("    // Production kernels do not build")]
        harness = r'''
use std::cell::Cell;
type Result<T> = std::result::Result<T, i32>;
const EIO: i32 = -5;
const EINTR: i32 = -4;
const ECANCELED: i32 = -125;
const ETIMEDOUT: i32 = -110;
const ENODEV: i32 = -19;
mod scrd { pub const SCRD_ACM_HANDLE_LEN: usize = 16; }
mod image { pub fn wipe(bytes: &mut [u8]) { bytes.fill(0); } }
TRAIT
GUARD
TESTS
'''
        harness = harness.replace("TRAIT", trait).replace("GUARD", guard).replace("TESTS", tests)
        with tempfile.TemporaryDirectory() as directory:
            source_path = Path(directory) / "context_cleanup.rs"
            binary_path = Path(directory) / "context_cleanup"
            source_path.write_text(harness)
            subprocess.run(["rustc", "--edition=2021", "--test", str(source_path),
                            "-o", str(binary_path)], check=True, capture_output=True, text=True)
            subprocess.run([str(binary_path)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
