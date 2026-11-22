#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Check that every Apple DART in the compiled device trees names a power domain.

A DART probes as a device of its own. Without its engine's power state it can be
probed, and read, while the block it sits in is off, which takes an SError. The
DART has no power gate of its own; the engine's covers it. So every node with an
"apple,*dart*" compatible must have power-domains, and no DART may name a power
state that is disabled in that DTB. That holds for a disabled DART too when a
device that uses it is enabled: the boot loader enables such DARTs (the
display's, for one) when it hands their device over, and the DART would then
wait for its state forever. A disabled DART that no enabled device uses (a
device is enabled when it and every node above it are okay) is skipped: the
block is off as a whole, which is consistent.

Exempt: the DARTs of the SEP and of the AOP. Those coprocessors run from
before the handoff on states the AP never switches off.

Install python3-libfdt (or pylibfdt) to read DTBs. Labels are shown when the DTBs
were built with DTC_FLAGS=-@.

Without --baseline-dtb-dir every finding fails. With it, only findings that the
baseline DTBs do not have fail, so the existing ones can be burnt down while no
new DART lands without its domain. A DTB that the baseline does not have (a new
board, or a variant such as t8112-j413-fw27 that includes its board's .dts)
inherits the findings of the baseline DTBs of its SoC: a finding with the same
node path, DART compatible and problem in any of them is not new. Anything the
new DTB adds still is.
"""

import argparse
from pathlib import Path
import re
import struct
import sys

DART = re.compile(r"apple,.*dart")
SOC = re.compile(r"apple,[st]\d{4}$")
EXEMPT = (
    (re.compile(r"apple,sep$"), "SEP"),
    (re.compile(r"apple,(t\d+-)?aop$"), "AOP"),
)


def findings(filename):
    """Yield the DTB's SoC compatible (or None), then (path, label, compatible,
    status, problem) for each finding."""
    import libfdt

    NF = (libfdt.NOTFOUND,)
    fdt = libfdt.Fdt(filename.read_bytes())
    nodes = {}

    def walk(offset, path):
        nodes[offset] = path
        child = fdt.first_subnode(offset, NF)
        while child >= 0:
            walk(child, path.rstrip("/") + "/" + fdt.get_name(child))
            child = fdt.next_subnode(child, NF)

    walk(0, "/")

    def prop(offset, name):
        value = fdt.getprop(offset, name, NF)
        return None if isinstance(value, int) else bytes(value)

    def cells(value):
        return struct.unpack(">%dI" % (len(value) // 4), value) if value else ()

    def strings(value):
        return value.rstrip(b"\0").decode().split("\0") if value else []

    def status(offset):
        return (strings(prop(offset, "status")) or ["okay"])[0]

    offsets = {path: offset for offset, path in nodes.items()}

    def enabled(offset):
        """The node and every node above it are okay."""
        path = nodes[offset]
        while True:
            if status(offsets[path]) != "okay":
                return False
            if path == "/":
                return True
            path = path.rsplit("/", 1)[0] or "/"

    soc = [c for c in strings(prop(0, "compatible")) if SOC.match(c)]
    yield soc[0] if soc else None

    phandles = {fdt.get_phandle(o): o for o in nodes if fdt.get_phandle(o)}
    labels = {}
    symbols = fdt.path_offset("/__symbols__", NF)
    if symbols >= 0:
        p = fdt.first_property_offset(symbols, NF)
        while p >= 0:
            sym = fdt.get_property_by_offset(p)
            labels.setdefault(bytes(sym).rstrip(b"\0").decode(), sym.name)
            p = fdt.next_property_offset(p, NF)

    def iommu_cells(offset):
        value = cells(prop(offset, "#iommu-cells"))
        return value[0] if value else 1

    consumers = {}
    for offset in nodes:
        spec = cells(prop(offset, "iommus"))
        i = 0
        while i < len(spec) and spec[i] in phandles:
            consumers.setdefault(phandles[spec[i]], []).append(offset)
            i += 1 + iommu_cells(phandles[spec[i]])
        spec = cells(prop(offset, "iommu-map"))
        i = 0
        while i + 1 < len(spec) and spec[i + 1] in phandles:
            consumers.setdefault(phandles[spec[i + 1]], []).append(offset)
            i += 3 + iommu_cells(phandles[spec[i + 1]])

    for offset, path in nodes.items():
        compatible = strings(prop(offset, "compatible"))
        if not any(DART.match(c) for c in compatible):
            continue
        # A disabled DART that no enabled device uses stays off with its block.
        # A DART inside a disabled node is disabled too: it is never populated.
        if not enabled(offset) and not any(enabled(o) for o in consumers.get(offset, [])):
            continue
        label = labels.get(path, "-")
        domains = cells(prop(offset, "power-domains"))
        if not domains:
            users = [c for o in consumers.get(offset, [])
                     for c in strings(prop(o, "compatible"))]
            why = [name for pattern, name in EXEMPT if any(pattern.match(c) for c in users)]
            problem = "exempt (%s)" % why[0] if why else "no power-domains"
            yield path, label, compatible[0], status(offset), problem
            continue
        i = 0
        while i < len(domains):
            provider = phandles.get(domains[i])
            if provider is None:
                break
            if status(provider) != "okay":
                yield (path, label, compatible[0], status(offset),
                       "power domain %s is disabled" % nodes[provider])
            n = cells(prop(provider, "#power-domain-cells"))
            i += 1 + (n[0] if n else 0)


def collect(directory):
    dtbs = sorted(Path(directory).glob("*.dtb"))
    if not dtbs:
        raise SystemExit("%s: no DTBs" % directory)
    result, socs = {}, {}
    for dtb in dtbs:
        found = findings(dtb)
        socs[dtb.stem] = next(found)
        for path, label, compatible, status, problem in found:
            result[(dtb.stem, path, problem)] = (label, compatible, status)
    return result, socs


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dtb-dir", required=True,
                        help="directory with the compiled Apple DTBs")
    parser.add_argument("--baseline-dtb-dir",
                        help="DTBs of the base revision; only new findings fail")
    parser.add_argument("--report", action="store_true",
                        help="list every finding and exemption, never fail")
    args = parser.parse_args()

    head, head_socs = collect(args.dtb_dir)
    base, base_socs = collect(args.baseline_dtb_dir) if args.baseline_dtb_dir else ({}, {})
    # What a DTB the baseline lacks may inherit: the baseline findings of its SoC.
    inheritable = {}
    for (board, path, problem), (_, compatible, _) in sorted(base.items()):
        if base_socs[board]:
            inheritable.setdefault((base_socs[board], path, compatible, problem), board)
    failing = inherited = 0
    for key in sorted(head):
        board, path, problem = key
        label, compatible, status = head[key]
        exempt = problem.startswith("exempt")
        new = not exempt and (not args.baseline_dtb_dir or key not in base)
        origin = None
        if new and args.baseline_dtb_dir and board not in base_socs and head_socs[board]:
            origin = inheritable.get((head_socs[board], path, compatible, problem))
        if origin:
            new = False
            inherited += 1
        if args.report or new:
            print("%s %s %s %s %s: %s%s" % (board, path, label, compatible, status, problem,
                                            " (new)" if new and args.baseline_dtb_dir else
                                            " (inherited from %s)" % origin if origin else ""))
        failing += new
    counted = ("exempt",)
    gone = [k for k in base if k not in head and not k[2].startswith(counted)]
    total = sum(1 for k in head if not k[2].startswith(counted))
    print("DARTs without a usable power domain: %d, new: %d, inherited by new DTBs: %d, "
          "fixed: %d" % (total, failing if args.baseline_dtb_dir else total, inherited,
                         len(gone)),
          file=sys.stderr)
    return 0 if args.report or not failing else 1


if __name__ == "__main__":
    sys.exit(main())
