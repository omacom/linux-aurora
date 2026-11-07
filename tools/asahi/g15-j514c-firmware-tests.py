#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise production full firmware lookup against the actual T6031 rows.

Only public table identities are read, never firmware images. --table allows
integration checks against a table containing another proposed firmware row.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
hw = root / "drivers/gpu/drm/asahi/g15"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--table", type=Path, default=hw / "hw/t6031.rs")
args = parser.parse_args()
rows = []
for match in re.finditer(
        r"FirmwareBuild\s*\{(.*?)text_sha256:\s*\[(.*?)\],\s*\}",
        args.table.read_text(), re.S):
    body, raw = match.groups()
    digest = re.findall(r"0x[0-9a-f]+", raw)
    assert len(digest) == 32
    fields = {
        key: re.search(r"\b" + key + r":\s*(0x[0-9a-f]+)", body).group(1)
        for key in ("text_size", "data_size", "marker")
    }
    machine = re.search(r'machine:\s*Some\(kernel::c_str!\("([^"]+)"\)\)', body)
    rows.append((fields, digest, machine.group(1) if machine else None))
assert sum(machine == "apple,j514c" for _, _, machine in rows) == 1
assert any(machine == "apple,j516c" for _, _, machine in rows)
assert all(machine == "apple,j516c" for _, _, machine in rows if machine != "apple,j514c")
measurements = []
machines = []
for fields, digest, machine in rows:
    measurements.append(
        "Measured {text_size:" + fields["text_size"] + ",data_size:"
        + fields["data_size"] + ",marker:" + fields["marker"]
        + ",sha256:[" + ",".join(digest) + "]}"
    )
    machines.append("None" if machine is None else 'Some("' + machine + '")')
code = r'''#[path="MATCHING"] mod matching;
#[path="LOOKUP"] mod lookup;
use lookup::{Measured, Verdict};
const MACHINES: [Option<&str>; ROW_COUNT] = [BOARD_BINDINGS];
fn rows(board: &str) -> Vec<(Measured, bool)> {
    let measurements = [MEASUREMENTS];
    measurements.into_iter().zip(MACHINES).map(|(m, b)| {
        (m, b.map_or(true, |b| b == board))
    }).collect()
}
#[test]
fn j514c_exact_candidate() {
    let index = MACHINES.iter().position(|b| *b == Some("apple,j514c")).unwrap();
    let exact = rows("apple,j514c")[index].0;
    assert_eq!(lookup::verdict(&exact, rows("apple,j514c"), 0), Verdict::Pinned(index));
    for board in ["apple,j516c", "apple,j514m", "apple,j516m"] {
        assert_eq!(lookup::verdict(&exact, rows(board), 0), Verdict::Refused);
    }
    let mut bad = exact; bad.sha256[31] ^= 1;
    assert_eq!(lookup::verdict(&bad, rows("apple,j514c"), 0), Verdict::Refused);
    let mut bad = exact; bad.text_size += 0x4000;
    assert_eq!(lookup::verdict(&bad, rows("apple,j514c"), 0), Verdict::Refused);
    let mut bad = exact; bad.data_size += 0x4000;
    assert_eq!(lookup::verdict(&bad, rows("apple,j514c"), 0), Verdict::Refused);
    let mut bad = exact; bad.marker += 8;
    assert_eq!(lookup::verdict(&bad, rows("apple,j514c"), 0), Verdict::Refused);
}
#[test]
fn existing_rows_unchanged() {
    for (index, board) in MACHINES.into_iter().enumerate() {
        if board.is_none() || board == Some("apple,j516c") {
            let measured = rows("apple,j516c")[index].0;
            assert_eq!(lookup::verdict(&measured, rows("apple,j516c"), 0), Verdict::Pinned(index));
            if board.is_some() {
                for other in ["apple,j514c", "apple,j514m", "apple,j516m"] {
                    assert_eq!(lookup::verdict(&measured, rows(other), 0), Verdict::Refused);
                }
            }
        }
    }
}
'''
for token, replacement in {
        "MATCHING": str(hw / "identity/matching.rs"),
        "LOOKUP": str(hw / "identity/lookup.rs"),
        "ROW_COUNT": str(len(rows)),
        "MEASUREMENTS": ",".join(measurements),
        "BOARD_BINDINGS": ",".join(machines),
}.items():
    code = code.replace(token, replacement)
with tempfile.TemporaryDirectory(prefix="g15-fw-") as directory:
    path = Path(directory)
    (path / "test.rs").write_text(code)
    subprocess.run([
        "rustc", "--edition=2021", "--test", str(path / "test.rs"),
        "-o", str(path / "test"),
    ], check=True)
    subprocess.run([str(path / "test")], check=True)
