#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Check static PMP ownership in the compiled T6030 device trees."""

import argparse
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
BOARDS = ("t6030-j514s", "t6030-j516s")
PMP_NODES = {
    "iommu@350300000",
    "pmp_report@3503c0000",
    "pmp@350500000",
    "mbox@350c08000",
}
PMP_COMPATIBLES = {"apple,t6000-pmp-v2", "apple,t6030-pmp-v2-report"}


def check_default_compatibles(dtb, board):
    pending = ["/"]
    while pending:
        node = pending.pop()
        raw = subprocess.check_output(
            ["fdtget", "-t", "bx", "-d", "", str(dtb), node, "compatible"],
            text=True,
        )
        # Match the gate's whole-tree OF lookup, including disabled nodes
        # and every string in a compatible list.
        compatible = bytes(int(value, 16) for value in raw.split())
        matches = PMP_COMPATIBLES.intersection(
            value.decode("ascii").lower() for value in compatible.split(b"\0")
            if value
        )
        if matches:
            raise AssertionError(f"{board} default: static PMP compatible "
                                 f"{sorted(matches)} at {node}")
        children = subprocess.check_output(
            ["fdtget", "-l", str(dtb), node], text=True,
        )
        pending.extend(node.rstrip("/") + "/" + child
                       for child in children.splitlines())


def check_profile(tree, output, dtc, board, static):
    profile = "static" if static else "default"
    dtb = output / f"{board}-{profile}.dtb"
    command = [
        "cpp", "-nostdinc", "-undef", "-D__DTS__",
        "-I", str(tree / "scripts/dtc/include-prefixes"),
        "-x", "assembler-with-cpp",
    ]
    if static:
        command.append("-DAPPLE_USE_PMP")
    command.append(str(tree / f"arch/arm64/boot/dts/apple/{board}.dts"))
    source = subprocess.check_output(command)
    subprocess.run([dtc, "-q", "-I", "dts", "-O", "dtb", "-o", str(dtb)],
                   input=source, check=True)

    children = subprocess.check_output(["fdtget", "-l", str(dtb), "/soc"], text=True)
    present = PMP_NODES.intersection(children.split())
    expected = PMP_NODES if static else set()
    if present != expected:
        raise AssertionError(f"{board} {profile}: static PMP nodes "
                             f"{sorted(present)}, expected {sorted(expected)}")
    if not static:
        check_default_compatibles(dtb, board)

    alias = subprocess.run(["fdtget", "-t", "s", str(dtb), "/aliases", "pmp"],
                           capture_output=True, text=True)
    if static:
        if alias.returncode or alias.stdout.strip() != "/soc/pmp@350500000":
            raise AssertionError(f"{board} {profile}: missing or incorrect PMP alias")
    elif alias.returncode == 0:
        raise AssertionError(f"{board} {profile}: unexpected static PMP alias")
    print(f"{board} {profile}: PASS")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tree", type=Path, default=ROOT)
    parser.add_argument("--dtc", required=True, help="this kernel's built scripts/dtc/dtc")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for board in BOARDS:
        for static in (False, True):
            check_profile(args.tree.resolve(), args.output, args.dtc, board, static)


if __name__ == "__main__":
    main()
