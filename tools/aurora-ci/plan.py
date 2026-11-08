#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Decide what a kernel CI run does, from the GitHub event that started it.

A light run is the checks that cost nothing on GitHub-hosted runners: commit
hygiene, host test suites and the CI's own tests. A full run adds the
package build, KUnit and the boot test. Builds are paid for, so they run only
when someone with write access asks for one:

  merge_group              full: the queue tests exactly what will land
  push to the branch       full: provenance for what landed (reuses the
                           queue's build when the tree is unchanged)
  schedule                 full, never reusing an earlier build: catches
                           changes in omarchy-pkgs and in the packages its
                           builder installs
  workflow_dispatch        full, never reusing an earlier build
  pull_request, labeled    full when the label is the build label. Applying
                           a label needs triage access, so this is a
                           maintainer asking for this exact head commit.
  pull_request, other      full when the author is trusted and the pull
                           request is not a draft, from a fork too; full when
                           the build label is already on a branch of this
                           repository (only writers can push there); light
                           for forks, where a new push needs the label
                           re-applied, and light otherwise.

Trusted authors are the team, those vouched for in .github/VOUCHED.td,
which the workflow reads from the target branch so that a pull request cannot
vouch for itself. Repository or organization access alone does not count:
the organization is wider than the kernel team. An author denounced there
gets light runs only, whatever their labels.

Repository variables do not reach a fork's pull request run, so the plan names
the runner for a trusted author's fork; any other run leaves it empty and the
workflow uses KERNEL_CI_RUNNER, or GitHub's runner without it.
"""

import argparse
import json
import os
import sys
from pathlib import Path

DEFAULT_LABEL = "ci:build"
ZERO_SHA = "0" * 40
PR_REFRESH_ACTIONS = {"opened", "synchronize", "reopened", "ready_for_review"}
VOUCHED_FORK_RUNNER = '["namespace-profile-arm64-linux-apple-aurora"]'


def read_vouched(text):
    """Lower-cased GitHub login -> "vouched" or "denounced", from VOUCHED.td.
    A denouncement wins over any vouch for the same login."""
    entries = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        denounced = line.startswith("-")
        platform, _, login = line.lstrip("-").split(None, 1)[0].partition(":")
        if platform != "github" or not login:
            continue
        login = login.lower()
        if denounced:
            entries[login] = "denounced"
        else:
            entries.setdefault(login, "vouched")
    return entries


def trust(event, vouched):
    """(status, login) for a pull request event: "vouched" only when both the
    pull request's author and whoever triggered the event (the pusher, on a
    push) are vouched for, so no one outside the team can push to a team
    member's pull request and have it built. Otherwise the login that is
    denounced, or failing that unknown."""
    logins = [(event["pull_request"].get("user") or {}).get("login", ""),
              (event.get("sender") or {}).get("login", "")]
    statuses = [(vouched.get(login.lower(), "unknown"), login or "?") for login in logins]
    for wanted in ("denounced", "unknown"):
        for status, login in statuses:
            if status == wanted:
                return status, login
    return "vouched", statuses[0][1]


def plan(event_name, event, label=DEFAULT_LABEL, vouched=None):
    """Return the plan as a dict of strings, ready for $GITHUB_OUTPUT."""
    out = {"tier": "light", "reason": "", "base_sha": "", "head_sha": "",
           "pr_number": "", "fork": "false", "force": "false", "runner": ""}

    if event_name == "pull_request":
        pr = event["pull_request"]
        action = event.get("action", "")
        labels = {item["name"] for item in pr.get("labels", [])}
        fork = pr["head"]["repo"] is None or pr["head"]["repo"]["full_name"] != pr["base"]["repo"]["full_name"]
        status, author = trust(event, vouched or {})
        out.update(base_sha=pr["base"]["sha"], head_sha=pr["head"]["sha"],
                   pr_number=str(pr["number"]), fork=str(fork).lower())
        if status == "denounced":
            out["reason"] = f"checks only: {author} is denounced in .github/VOUCHED.td"
        elif action == "labeled" and event.get("label", {}).get("name") == label:
            out.update(tier="full", reason=f"`{label}` applied to {pr['head']['sha'][:12]}")
        elif action in PR_REFRESH_ACTIONS and status == "vouched" and not pr.get("draft"):
            out.update(tier="full", reason=f"{author} is vouched for")
        elif action in PR_REFRESH_ACTIONS and label in labels and not fork:
            out.update(tier="full", reason=f"`{label}` is on this pull request")
        elif action in PR_REFRESH_ACTIONS and label in labels:
            out["reason"] = (f"fork pull request: re-apply `{label}` to build "
                             f"{pr['head']['sha'][:12]}")
        elif status == "vouched":
            out["reason"] = (f"draft: {author}'s builds start when it is ready for review, "
                             f"or add `{label}` for a build now")
        else:
            out["reason"] = (f"checks only ({author} is not in .github/VOUCHED.td); the merge "
                             f"queue builds the kernel, or add `{label}` for a build now")
        if fork and status == "vouched" and out["tier"] == "full":
            out["runner"] = VOUCHED_FORK_RUNNER
    elif event_name == "merge_group":
        group = event["merge_group"]
        out.update(tier="full", reason="merge queue", base_sha=group["base_sha"],
                   head_sha=group["head_sha"])
    elif event_name == "push":
        before = event.get("before", "")
        out.update(tier="full", reason=f"push to {event.get('ref', '?')}", head_sha=event["after"])
        if before and before != ZERO_SHA and not event.get("forced"):
            out["base_sha"] = before
    elif event_name == "schedule":
        # Rebuild from scratch: the Arch Linux ARM packages the builder
        # installs change without any change to this tree or omarchy-pkgs.
        out.update(tier="full", reason="nightly rebuild", force="true")
    elif event_name == "workflow_dispatch":
        out.update(tier="full", reason="manual run", force="true")
    else:
        raise ValueError(f"unsupported event {event_name!r}")
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--event-name", default=os.environ.get("GITHUB_EVENT_NAME"))
    parser.add_argument("--event-path", type=Path, default=os.environ.get("GITHUB_EVENT_PATH"))
    parser.add_argument("--label", default=DEFAULT_LABEL)
    parser.add_argument("--vouched", type=Path,
                        help="the target branch's VOUCHED.td; without it no author is trusted")
    args = parser.parse_args(argv)
    if not args.event_name or not args.event_path:
        parser.error("--event-name and --event-path (or the GitHub environment) are required")

    try:
        vouched = read_vouched(args.vouched.read_text()) if args.vouched else {}
        result = plan(args.event_name, json.loads(args.event_path.read_text()), args.label, vouched)
    except (KeyError, TypeError, ValueError, OSError) as error:
        print(f"::error title=CI plan::cannot plan this {args.event_name} event: {error!r}", file=sys.stderr)
        return 1

    print(json.dumps(result, indent=1))
    if os.environ.get("GITHUB_OUTPUT"):
        with open(os.environ["GITHUB_OUTPUT"], "a") as out:
            for key, value in result.items():
                out.write(f"{key}={value}\n")
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as out:
            out.write(f"### Plan: {result['tier']} run\n\n{result['reason']}.\n\n")
    if result["tier"] == "light":
        print(f"::notice title=Checks only::{result['reason']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
