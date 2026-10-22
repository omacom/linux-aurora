#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Attribute Asahi GEM handles without counting duplicated DRM file descriptors twice."""
import argparse
import json
from pathlib import Path
import re


def size_bytes(value):
    match = re.fullmatch(r"(\d+)(?:\s+(B|KiB|MiB|GiB))?", value.strip())
    if not match:
        raise ValueError(f"unsupported DRM size: {value!r}")
    return int(match[1]) * {None: 1, "B": 1, "KiB": 1024,
                           "MiB": 1024**2, "GiB": 1024**3}[match[2]]


def parse_fdinfo(text):
    fields = dict(line.split(":", 1) for line in text.splitlines() if ":" in line)
    fields = {key: value.strip() for key, value in fields.items()}
    if "drm-driver" not in fields or "drm-client-id" not in fields:
        return None
    return fields


def collect(proc_root, driver="asahi"):
    clients = {}
    unreadable = 0
    for process in sorted(proc_root.iterdir()):
        if not process.name.isdigit():
            continue
        try:
            name = (process / "comm").read_text().strip()
            infos = list((process / "fdinfo").iterdir())
        except (OSError, ValueError):
            unreadable += 1
            continue
        for info in infos:
            try:
                fields = parse_fdinfo(info.read_text())
                if not fields or fields["drm-driver"] != driver:
                    continue
                node = (process / "fd" / info.name).resolve(strict=True)
                identity = fields.get("drm-pdev", str(node))
                key = (driver, identity, int(fields["drm-client-id"]))
                if key not in clients:
                    memory = {
                        kind: size_bytes(fields[f"drm-{kind}-memory"])
                        if f"drm-{kind}-memory" in fields else None
                        for kind in ["total", "shared", "resident", "active", "purgeable"]
                    }
                    clients[key] = {
                        "driver": driver, "device": identity, "client_id": key[2],
                        "name": fields.get("drm-client-name"), "memory_bytes": memory,
                        "processes": {}, "fd_count": 0,
                    }
                clients[key]["processes"][int(process.name)] = name
                clients[key]["fd_count"] += 1
            except (OSError, ValueError):
                unreadable += 1
    rows = []
    for client in clients.values():
        client["processes"] = [{"pid": pid, "name": name}
                               for pid, name in sorted(client["processes"].items())]
        rows.append(client)
    return rows, unreadable


def report(proc_root, debug_root, driver="asahi"):
    clients, unreadable = collect(proc_root, driver)
    allocation_extents = {}
    for generation in ["g17", "m3", "g16"]:
        path = debug_root / f"asahi-{generation}" / "memory"
        try:
            text = path.read_text()
        except OSError:
            continue
        allocation_extents[str(path)] = text
    known = [row["memory_bytes"] for row in clients
             if row["memory_bytes"]["total"] is not None
             and row["memory_bytes"]["shared"] is not None]
    return {
        "clients": clients,
        "clients_with_memory_reporting": len(known),
        "private_handle_bytes": sum(row["total"] - row["shared"] for row in known),
        "driver_allocation_extents": allocation_extents,
        "unreadable_entries": unreadable,
        "notes": [
            "Each DRM client is counted once across duplicated and inherited descriptors.",
            "Resident/shared values from different clients must not be added as unique physical memory.",
            "Private handle bytes exclude kernel allocations and objects retained without client handles.",
            "Driver allocation extents are logical sizes, not a resident-page measurement.",
            "Missing resident counters mean unavailable, not zero. Root access gives fuller process coverage.",
        ],
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--proc-root", type=Path, default=Path("/proc"))
    parser.add_argument("--debug-root", type=Path, default=Path("/sys/kernel/debug"))
    parser.add_argument("--driver", default="asahi")
    args = parser.parse_args()
    print(json.dumps(report(args.proc_root, args.debug_root, args.driver), indent=2))
