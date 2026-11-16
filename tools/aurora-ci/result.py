#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Turn the kernel CI jobs' results into the one required check.

Branch protection and the merge queue require a single check, so this job
decides what the run as a whole means. A job that was supposed to run must
have succeeded; a skipped job passes only when the plan skips it on purpose:

  - full-run jobs (package, kunit, boot) are skipped on a light run;
  - kunit and boot are skipped when an earlier run already verified the same
    build key (same kernel tree and packaging inputs);
  - commit, compile and device tree checks are skipped for events without a
    pull request or merge group range (push, schedule, manual runs).

A light run that passes says only that the checks passed: the kernel itself
is built in the merge queue (or, there, taken from the pull request's run of
the same tree), and the summary says so.
"""

import argparse
import json
import os
import sys

LIGHT = ("host-tests", "self-test")
RANGE = ("commits", "compile", "devicetree")
FULL = ("package", "kunit", "boot")
RANGE_EVENTS = {"pull_request", "merge_group"}


def evaluate(needs, event_name):
    """Return (ok, rows) where rows are (job, result, expected, verdict)."""
    plan = needs.get("plan", {})
    if plan.get("result") != "success":
        return False, [("plan", plan.get("result", "missing"), "success", "failed")]
    outputs = plan.get("outputs", {})
    tier = outputs.get("tier")
    verified = bool(outputs.get("verified_run"))

    expected = {job: "success" for job in LIGHT}
    for job in RANGE:
        expected[job] = "success" if event_name in RANGE_EVENTS else "skipped"
    for job in FULL:
        if tier != "full":
            expected[job] = "skipped"
        elif job != "package" and verified:
            expected[job] = "skipped"
        else:
            expected[job] = "success"

    rows, ok = [("plan", "success", "success", "ok")], True
    for job, want in expected.items():
        got = needs.get(job, {}).get("result", "missing")
        good = got == want
        ok &= good
        rows.append((job, got, want, "ok" if good else "failed"))
    return ok, rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--needs", default=os.environ.get("NEEDS_JSON"),
                        help="the workflow's toJSON(needs)")
    parser.add_argument("--event-name", default=os.environ.get("GITHUB_EVENT_NAME"))
    args = parser.parse_args(argv)
    if not args.needs or not args.event_name:
        parser.error("--needs and --event-name (or NEEDS_JSON and GITHUB_EVENT_NAME) are required")

    needs = json.loads(args.needs)
    ok, rows = evaluate(needs, args.event_name)
    outputs = needs.get("plan", {}).get("outputs", {})
    lines = [f"### Kernel CI: {'passed' if ok else 'FAILED'}", "",
             f"{outputs.get('tier', '?')} run: {outputs.get('reason', '')}.", "",
             "| Job | Result | Expected | |", "|---|---|---|---|"]
    lines += [f"| {job} | {got} | {want} | {verdict} |" for job, got, want, verdict in rows]
    if outputs.get("tier") == "light" and ok:
        lines += ["", "The kernel was not built in this run; the merge queue builds and boots it, "
                      "unless the run of the head it merges passed for exactly the tree it merges."]
    if outputs.get("pull_request_run"):
        lines += ["", f"Packages, KUnit and boot taken from pull request run {outputs['pull_request_run']}, "
                      "whose record names a commit with this exact tree; that record is the run's own word."]
    elif outputs.get("verified_run"):
        lines += ["", f"KUnit and the boot test passed for this build key in run {outputs['verified_run']}."]
    text = "\n".join(lines) + "\n"
    print(text)
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as out:
            out.write(text)
    if os.environ.get("GITHUB_OUTPUT"):
        with open(os.environ["GITHUB_OUTPUT"], "a") as out:
            out.write(f"ok={'true' if ok else 'false'}\n")
    for job, got, want, verdict in rows:
        if verdict != "ok":
            print(f"::error title=Kernel CI::{job} {got}, expected {want}", file=sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
