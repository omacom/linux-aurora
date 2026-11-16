#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Host controls for default-off T6030 GPU/PMP selection and error propagation.

Actual selection, dispatch, device-link and serialized power functions are
extracted. OF enumeration/property storage, device binding, MMIO polling,
and locks are controlled host boundaries. The T6030 managed entry wait
and start wait are extracted too; the firmware/workqueue boundary is controlled.
The GFX-report service is modeled separately from supplier readiness; its
transaction has its own production-body PMP-vote selftest. This models
ownership/error handling, not real teardown concurrency or DMA.
"""
import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import resource
import shlex
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
GATE = "drivers/soc/apple/t6030-display-gate.c"
REPORT = "drivers/pmdomain/apple/pmp-report.c"
EXPORT = "drivers/soc/apple/pmp-export.c"


def function(source, signature):
    start = source.index(signature)
    end = source.index("{", start) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end] + "\n"


def digest(text):
    return hashlib.sha256(text.encode()).hexdigest()


def no_core():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


# (CONFIG_APPLE_T6030_GPU_PMP_LINK, CONFIG_APPLE_T8122_GPU_PMP_LINK): off, G15 only, and G15
# with the M3 runtime (DRM_ASAHI_M3_T8122 depends on DRM_ASAHI_G15).
CONFIGS = ((0, 0), (1, 0), (1, 1))


def run(out, name, code, compiler, configs=CONFIGS):
    dest = out / name
    dest.mkdir()
    (dest / "t6030-pmp-production.h").write_text(code)
    results = []
    for t6030, t8122 in configs:
        config = f"{t6030}{t8122}"
        binary = dest / f"config-{config}"
        command = compiler + ["-std=gnu11", "-g", "-O1", "-Wall", "-Wextra", "-Werror",
                              "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                              f"-DCONFIG_APPLE_T6030_GPU_PMP_LINK={t6030}",
                              f"-DCONFIG_APPLE_T8122_GPU_PMP_LINK={t8122}",
                              "-I", str(dest),
                              str(ROOT / "tools/asahi/t6030-pmp-supplier-tests.c"),
                              "-o", str(binary)]
        built = subprocess.run(command, capture_output=True, text=True)
        (dest / f"config-{config}-build.log").write_text(built.stdout + built.stderr)
        ran = None
        if built.returncode == 0:
            ran = subprocess.run([str(binary)], capture_output=True, text=True,
                                 preexec_fn=no_core)
            (dest / f"config-{config}-run.log").write_text(ran.stdout + ran.stderr)
        results.append({"config": config, "command": command,
                        "build_rc": built.returncode, "run_rc": ran.returncode if ran else None,
                        "sanitizer_diagnostic": bool(ran and (
                            "AddressSanitizer" in ran.stderr or "runtime error:" in ran.stderr)),
                        "stdout": ran.stdout if ran else ""})
    return {"name": name, "results": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path,
                        help="new directory for generated sources, logs and receipt")
    parser.add_argument("--mutations", action="store_true")
    parser.add_argument("--baseline-ref", help="also verify unchanged entry-ready/start consumers")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    gate, report, export = [(ROOT / path).read_text() for path in (GATE, REPORT, EXPORT)]
    kconfig = (ROOT / "drivers/soc/apple/Kconfig").read_text()
    section = kconfig.split("config APPLE_T6030_GPU_PMP_LINK\n", 1)[1].split("\nconfig ", 1)[0]
    assert '\n\tbool\n' in "\n" + section and "\n\tdefault n\n" in section
    assert "depends on APPLE_T6030_DISPLAY_GATE" in section
    parts = [function(gate, "static struct device_node *__init gate_find_one("),
             function(gate, "static int __init gate_gpu_pmp_link("),
             function(report, "static struct device_node *apple_pmp_report_t6030_entry("),
             function(report, "static int apple_pmp_report_wait_t6030_14("),
             function(report, "int apple_pmp_report_wait_supplier_ready("),
             function(export, "int apple_pmp_link_device("),
             function(export, "int apple_pmp_set_device_power(")]
    offsets = re.search(r'static const struct apple_pmp_report_offsets apple_pmp_offsets_t6030 = \{.*?\n\};', report, re.S)
    assert offsets
    code = offsets.group() + "\n" + function(report, "static int apple_pmp_start_wait(") + "\n" + function(report, "int apple_pmp_report_wait_ready(") + "\n" + "\n".join(parts)
    # Existing managed-start consumers must remain byte-identical.
    consumers = ["int apple_pmp_report_wait_ready(", "static int apple_pmp_start_wait("]
    consumer_hashes = {name: digest(function(report, name)) for name in consumers}
    baseline_checked = False
    baseline_code = None
    if args.baseline_ref:
        baseline = subprocess.check_output(["git", "-C", str(ROOT), "show",
                                            f"{args.baseline_ref}:{REPORT}"], text=True)
        for name in consumers:
            assert function(report, name) == function(baseline, name), name
        baseline_gate = subprocess.check_output(["git", "-C", str(ROOT), "show",
                                                 f"{args.baseline_ref}:{GATE}"], text=True)
        old_gate = function(baseline_gate, "static int __init gate_t8122_gpu_pmp_link(")
        old_gate = old_gate.replace("gate_t8122_gpu_pmp_link", "gate_gpu_pmp_link")
        old_dispatch = function(baseline, "int apple_pmp_report_wait_supplier_ready(")
        baseline_code = code.replace(parts[1], old_gate).replace(function(report, "int apple_pmp_report_wait_supplier_ready("), old_dispatch)
        baseline_checked = True
    compiler = shlex.split(os.environ.get("CC", "cc"))
    version = subprocess.check_output(compiler + ["--version"], text=True).splitlines()[0]
    variants = [("control-before", code)]
    if args.mutations:
        mutations = [
            ("config-scope", "if (!IS_ENABLED(CONFIG_APPLE_T6030_GPU_PMP_LINK) ||", "if (false ||"),
            ("machine-scope", '!of_machine_is_compatible("apple,t6030") ||', "false ||"),
            ("supplier-compat", '!of_device_is_compatible(supplier->of_node, "apple,t6030-pmp-v2"))', "false)"),
            ("duplicate-report", "\t\tif (report) {", "\t\tif (false) {"),
            ("disabled-report", "!of_device_is_available(report) ||", "false ||"),
            ("report-phandle-shape", 'of_property_count_u32_elems(report, "apple,pmp") != 1', "false"),
            ("supplier-identity", "pmp == supplier->of_node && entry ?", "entry ?"),
            ("entry-shape", 'of_property_count_u32_elems(child, "reg") != 1', "false"),
            ("duplicate-entry", "if (entry ||", "if (false ||"),
            ("disabled-entry", "!of_device_is_available(child) ||", "false ||"),
            ("entry-compat", '!of_device_is_compatible(child, "apple,t6000-pmp-v2-report-entry") ||', "false ||"),
            ("entry-no-ack", 'of_property_read_bool(child, "apple,no-ack"))', "false)"),
            ("declared-report-fallback", '!np && !of_property_present(supplier->of_node, "apple,pmp-report")', "!np"),
            ("readiness-error", "apple_pmp_report_wait_ready(entry, msecs_to_jiffies(timeout_ms)) : -ENODEV;",
             "(apple_pmp_report_wait_ready(entry, msecs_to_jiffies(timeout_ms)), 0) : -ENODEV;"),
            ("managed-owner", '.starts_pmp = IS_ENABLED(CONFIG_APPLE_T6030_GPU_PMP_LINK)', '.starts_pmp = false'),
            ("managed-ack", 'status == PMP_REPORT_READY && (ack & BIT_ULL(id))', '((void)ack, status == PMP_REPORT_READY)'),
            ("managed-ready", 'status == PMP_REPORT_READY && (ack & BIT_ULL(id))', '((void)status, (ack & BIT_ULL(id)))'),
            ("entry-reference", "\tof_node_put(entry);\n\tof_node_put(pmp);", "\tof_node_put(pmp);"),
            ("gate-config", "IS_ENABLED(CONFIG_APPLE_T6030_GPU_PMP_LINK) && gate_soc == &gate_t6030", "gate_soc == &gate_t6030"),
            ("gate-phandle-shape", 'of_property_count_u32_elems(gpu, "apple,pmp") != 1', "false"),
            ("serialized-command", "\tguard(mutex)(&pmp_lock);", "\t(void)&pmp_lock;"),
            ("gfx-check-error", "ret = apple_pmp_report_check_t6030_gfx(pmp_device);",
             "ret = (apple_pmp_report_check_t6030_gfx(pmp_device), 0);"),
        ]
        for name, old, new in mutations:
            assert code.count(old) == 1, (name, code.count(old))
            variants.append((name, code.replace(old, new)))
        variants.append(("control-after", code))
    # The T8122 14.x wait mirrors the T6030 one, so it is added to every variant after the
    # T6030 mutations are placed; its own mutants change only its text.
    dispatch = "int apple_pmp_report_wait_supplier_ready("
    t8122 = function(report, "static int apple_pmp_report_wait_t8122_14(") + "\n"
    def with_t8122(source, text=t8122):
        if dispatch not in source:
            return source
        return source.replace(dispatch, text + dispatch, 1)
    variants = [(name, with_t8122(source)) for name, source in variants]
    if args.mutations:
        t8122_mutations = [
            ("t8122-config-scope", "if (!IS_ENABLED(CONFIG_APPLE_T8122_GPU_PMP_LINK) ||", "if (false ||"),
            ("t8122-supplier-compat", '!of_device_is_compatible(supplier->of_node, "apple,t8122-pmp-v2"))', "false)"),
            ("t8122-report-compat", 'for_each_compatible_node(np, NULL, "apple,t8122-pmp-v2-report")',
             'for_each_compatible_node(np, NULL, "apple,t6030-pmp-v2-report")'),
            ("t8122-duplicate-report", "\t\tif (report) {", "\t\tif (false) {"),
            ("t8122-disabled-report", "!of_device_is_available(report) ||", "false ||"),
            ("t8122-report-phandle-shape", 'of_property_count_u32_elems(report, "apple,pmp") != 1', "false"),
            ("t8122-supplier-identity", "pmp == supplier->of_node && entry ?", "entry ?"),
            ("t8122-readiness-error", "apple_pmp_report_wait_ready(entry, msecs_to_jiffies(timeout_ms)) : -ENODEV;",
             "(apple_pmp_report_wait_ready(entry, msecs_to_jiffies(timeout_ms)), 0) : -ENODEV;"),
            ("t8122-entry-reference", "\tof_node_put(entry);\n\tof_node_put(pmp);", "\tof_node_put(pmp);"),
        ]
        for name, old, new in t8122_mutations:
            assert t8122.count(old) == 1, (name, t8122.count(old))
            variants.insert(-1, (name, with_t8122(code, t8122.replace(old, new))))
        # The dispatch owns the machine scope: T8122 takes the T8122 wait, every other SoC the
        # T6030 one. (The wait's own machine check repeats it and cannot be reached otherwise.)
        machine = 'if (of_machine_is_compatible("apple,t8122"))'
        assert code.count(machine) == 1
        for name, new in (("t8122-dispatch-never", "if (false)"), ("t8122-dispatch-always", "if (true)")):
            variants.insert(-1, (name, with_t8122(code.replace(machine, new))))
    if baseline_code and "apple_pmp_report_wait_t8122_14(" in baseline_code:
        baseline_code = with_t8122(baseline_code)
    results = [run(args.output, name, source, compiler) for name, source in variants]
    if baseline_code:
        results.append(run(args.output, "control-prior-config-off", baseline_code, compiler, ((0, 0),)))
    valid = True
    for item in results:
        control = item["name"].startswith("control-")
        built = all(x["build_rc"] == 0 for x in item["results"])
        good = all(x["run_rc"] == 0 and not x["sanitizer_diagnostic"] for x in item["results"])
        killed = any(x["run_rc"] == -signal.SIGABRT or x["sanitizer_diagnostic"] for x in item["results"])
        item["verdict"] = ("PASS" if good else "INVALID") if control else (
            "NOBUILD" if not built else "KILLED" if killed else "SURVIVED")
        valid &= item["verdict"] == ("PASS" if control else "KILLED")
    receipt = {"passed": valid, "compiler": version, "production_sha256": digest(code),
               "fixture_sha256": hashlib.sha256((ROOT / "tools/asahi/t6030-pmp-supplier-tests.c").read_bytes()).hexdigest(),
               "entry_ready_consumers_unchanged_checked": baseline_checked,
               "consumer_hashes": consumer_hashes, "results": results}
    (args.output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps({"passed": valid, "variants": [{"name": x["name"], "verdict": x["verdict"]}
                                                       for x in results]}))
    return 0 if valid else 1


if __name__ == "__main__":
    sys.exit(main())
