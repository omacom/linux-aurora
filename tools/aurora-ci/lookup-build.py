#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Name a kernel build by its inputs and find earlier results for it.

A build is identified by what goes into it: the kernel tree without the CI
itself (.github and tools/aurora-ci never reach the package), and the parts
of omarchy-pkgs that build it (the linux-aurora recipe, the builder image
definition, bin/build and the helpers it sources). The same key means the
same package, so CI never builds or tests one twice; a change to the CI
alone, or to omarchy-pkgs outside those parts, reuses the last build. What
the key cannot see (package updates in the Arch Linux ARM repositories the
builder installs from) is what the nightly rebuild is for.

  kernel-package-<key>     the packages and their metadata (reused instead of
                           building again)
  kernel-verified-<key>    written when every full-run job passed; a later
                           run with the same key skips KUnit and the boot test
  kernel-metadata-<id>     package metadata for a kernel (its tree without
                           the CI), the baseline a later change is compared with

This script computes the key, pins the omarchy-pkgs commit it was computed
from, and looks those artifacts up through the Actions API. It accepts only
artifacts that kernel-ci.yml uploaded after review: from the branch, the
merge queue, the nightly run or a manual run of the branch (see
TRUSTED_EVENTS). The one exception is the merge queue: it also accepts the
build and verification of a pull request run at the very head it is merging,
the code its reviewers approved, when that run tested exactly the tree being
merged (see pull_request_run).
"""

import argparse
import hashlib
import http.client
import io
import json
import os
import re
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request
import zipfile
import zlib
from pathlib import Path

# What omarchy-pkgs' bin/build uses to build linux-aurora: the recipe, the
# builder image and its build script, bin/build itself and the helpers that
# bin/build and build/build.sh source.
RECIPE_PARTS = ("pkgbuilds/linux-aurora", "build", "bin/build", "helpers/docker-helpers.sh",
                "helpers/message-helpers.sh", "helpers/package-metadata.sh", "helpers/paths.sh")
# Paths in the kernel tree that never reach the package.
CI_PATHS = {"": {".github"}, "tools": {"aurora-ci"}}


def git(repo, *args):
    return subprocess.run(["git", "-C", str(repo), *args], check=True, text=True,
                          stdout=subprocess.PIPE).stdout.strip()


def kernel_id(repo, rev):
    """Hash of the tree at rev, leaving out CI_PATHS."""
    digest = hashlib.sha256()
    for directory, skip in sorted(CI_PATHS.items()):
        spec = f"{rev}:{directory}" if directory else f"{rev}^{{tree}}"
        listing = subprocess.run(["git", "-C", str(repo), "ls-tree", "--full-tree", spec], text=True,
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        if listing.returncode and directory:
            continue  # no such directory at this commit
        listing.check_returncode()
        for line in listing.stdout.splitlines():
            name = line.split("\t", 1)[1]
            if name in skip or (not directory and name in CI_PATHS):
                continue
            digest.update(f"{directory}/{line}\n".encode())
    return digest.hexdigest()[:40]


def ensure_commit(repo, sha, remote="origin"):
    """Fetch a commit's trees (no file contents) if the checkout lacks it.

    The check must not fetch: a partial clone fetches a missing object on
    demand with all of its history, millions of kernel commits and trees."""
    if subprocess.run(["git", "-C", str(repo), "--no-lazy-fetch", "cat-file", "-e", f"{sha}^{{commit}}"],
                      stderr=subprocess.DEVNULL).returncode:
        git(repo, "fetch", "--quiet", "--no-tags", "--depth=1", "--filter=blob:none", remote, sha)


def build_key(kernel_tree, recipe_trees):
    text = f"kernel {kernel_tree}\n" + "".join(f"{part} {recipe_trees[part]}\n" for part in RECIPE_PARTS)
    return hashlib.sha256(text.encode()).hexdigest()[:32]


# Runs whose artifacts may be reused. Every artifact name is chosen by the
# workflow that uploads it, and a pull request, or a manual run of a branch,
# runs that branch's own copy of the workflow files, before anyone reviewed
# them. So a name alone proves nothing: reuse only artifacts from runs of
# reviewed code, the branch itself and the merge queue's branches, which
# only a merged or queued pull request reaches.
TRUSTED_WORKFLOWS = {".github/workflows/kernel-ci.yml"}
TRUSTED_EVENTS = {"push", "merge_group", "schedule", "workflow_dispatch"}
TRUSTED_BRANCH = "integration/aurora"
QUEUE_PREFIX = f"gh-readonly-queue/{TRUSTED_BRANCH}/"


def trusted_branch(event, branch):
    if event == "merge_group":
        return branch.startswith(QUEUE_PREFIX)
    return branch == TRUSTED_BRANCH


# A pull request's own run is reviewed code only at the head being merged:
# the merge queue takes only approved pull requests, the approval has to
# follow the last push, and a push takes the pull request out of the queue.
# Runs of earlier heads ran code nobody approved, so they never count. Nor
# does a run of the same head in another pull request, which may target a
# branch with other workflows: GitHub runs the workflows of the merge.
PULL_REQUEST_WORKFLOWS = {".github/workflows/kernel-ci.yml", ".github/workflows/kernel-ci-label.yml"}
JOBS_WORKFLOW = ".github/workflows/kernel-ci-jobs.yml"


def queued_pull_request(ref):
    """The number of the pull request a merge queue ref merges last, or ''."""
    match = re.fullmatch(rf"refs/heads/{re.escape(QUEUE_PREFIX)}pr-([0-9]+)-[0-9a-f]{{40}}", ref)
    return match.group(1) if match else ""


def pull_request_head(repo, rev="HEAD"):
    """The pull request head a merge queue commit merges: its second parent,
    read from the commit object, which a depth-1 checkout has. A later
    queue entry also carries the entries ahead of it; its tree then differs
    from the one the pull request's run tested, which pull_request_run
    rejects."""
    header = git(repo, "cat-file", "commit", rev).split("\n\n", 1)[0]
    parents = [line.split()[1] for line in header.splitlines() if line.startswith("parent ")]
    return parents[1] if len(parents) == 2 else ""


# verified.json is a few hundred bytes; anything far larger is not one.
RECORD_LIMIT = 64 * 1024


class _KeepRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


class Api:
    def __init__(self, url, repository, token, repository_id):
        self.url, self.repository, self.token = url.rstrip("/"), repository, token
        self.repository_id = int(repository_id)
        self._runs = {}

    def request(self, path, **query):
        address = f"{self.url}/repos/{self.repository}/{path}"
        if query:
            address += "?" + urllib.parse.urlencode(query)
        return urllib.request.Request(address, headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {self.token}",
            "X-GitHub-Api-Version": "2022-11-28",
        })

    def get(self, path, **query):
        with urllib.request.urlopen(self.request(path, **query), timeout=60) as response:
            return json.load(response)

    def artifact_file(self, artifact_id, member, limit=RECORD_LIMIT):
        """One small file of an artifact, at most limit bytes zipped or not.
        The API redirects to storage, which must not be sent the token."""
        try:
            urllib.request.build_opener(_KeepRedirect).open(
                self.request(f"actions/artifacts/{artifact_id}/zip"), timeout=60).close()
            raise ValueError(f"artifact {artifact_id}: no download address")
        except urllib.error.HTTPError as error:
            if error.code not in (301, 302, 303, 307, 308):
                raise
            location = error.headers["Location"]
        with urllib.request.urlopen(location, timeout=60) as response:
            data = response.read(limit + 1)
        return read_member(data, member, limit)

    def trusted_run(self, run_id):
        """Whether a run is this repository's kernel CI on reviewed code:
        the branch or the merge queue, never a pull request or another
        branch."""
        if run_id not in self._runs:
            run = self.get(f"actions/runs/{run_id}")
            self._runs[run_id] = (
                run.get("event") in TRUSTED_EVENTS
                and trusted_branch(run.get("event"), run.get("head_branch") or "")
                and run.get("path") in TRUSTED_WORKFLOWS
                and (run.get("repository") or {}).get("id") == self.repository_id
                and (run.get("head_repository") or {}).get("id") == self.repository_id)
        return self._runs[run_id]

    def newest_artifact(self, name):
        """Run ID of the newest unexpired artifact with this exact name that a
        trusted run uploaded, or ''."""
        data = self.get("actions/artifacts", name=name, per_page=100)
        live = [a for a in data.get("artifacts", [])
                if not a.get("expired") and a.get("name") == name
                and a.get("workflow_run", {}).get("repository_id") == self.repository_id
                and a.get("workflow_run", {}).get("head_repository_id") == self.repository_id]
        live.sort(key=lambda a: a.get("created_at", ""), reverse=True)
        for artifact in live:
            if self.trusted_run(artifact["workflow_run"]["id"]):
                return str(artifact["workflow_run"]["id"])
        return ""

    def pull_request_merge(self, run_id, number, head_sha):
        """The merge commit a run ran on, if the run is this repository's
        kernel CI for pull request number at head head_sha (the head may
        come from a fork), passed at its first attempt and was never re-run,
        so its artifacts all come from that one attempt; else ''.

        The commit is the one GitHub recorded for the run's call of the jobs
        workflow. A call from the run's own workflow file is recorded with
        the run's ref (refs/pull/<number>/merge) and the commit it ran, the
        same commit as the calling file; a call to another repository or
        ref keeps that ref in its path instead. A run with any other shape
        does not count."""
        run = self.get(f"actions/runs/{run_id}")
        if not (run.get("event") == "pull_request"
                and run.get("path") in PULL_REQUEST_WORKFLOWS
                and run.get("head_sha") == head_sha
                and run.get("run_attempt") == 1
                and run.get("status") == "completed" and run.get("conclusion") == "success"
                and (run.get("repository") or {}).get("id") == self.repository_id):
            return ""
        calls = run.get("referenced_workflows")
        if not isinstance(calls, list) or len(calls) != 1 or not isinstance(calls[0], dict):
            return ""
        sha = calls[0].get("sha")
        if (not isinstance(sha, str) or not re.fullmatch(r"[0-9a-f]{40}", sha)
                or calls[0].get("ref") != f"refs/pull/{number}/merge"
                or calls[0].get("path") != f"{self.repository}/{JOBS_WORKFLOW}@{sha}"):
            return ""
        return sha

    def run_artifacts(self, run_id, name):
        """Every artifact named name in a run, expired or not."""
        data = self.get(f"actions/runs/{run_id}/artifacts", name=name, per_page=100)
        artifacts = [a for a in data.get("artifacts", []) if a.get("name") == name]
        if data.get("total_count") != len(artifacts):
            raise ValueError(f"run {run_id}: incomplete listing of {name}")
        return artifacts

    def pull_request_evidence(self, key, number, head_sha, tree):
        """(run ID, package artifact ID) of the newest run of pull request
        number at head_sha (see pull_request_merge) whose merge commit has
        this tree, which uploaded one live kernel-package-<key> and one live
        kernel-verified-<key> (a run that reused another run's packages
        uploads none), and whose verification record names key, the run and
        that commit; else ('', ''). The run then ran exactly the reviewed
        workflows; the record is still its own word: nothing here proves
        which job of the run wrote it, or which package bytes it verified."""
        verified_name, package_name = f"kernel-verified-{key}", f"kernel-package-{key}"
        data = self.get("actions/artifacts", name=verified_name, per_page=100)
        candidates = {str(a["workflow_run"]["id"]): a.get("created_at", "") for a in data.get("artifacts", [])
                      if a.get("name") == verified_name
                      and a.get("workflow_run", {}).get("repository_id") == self.repository_id
                      and a.get("workflow_run", {}).get("head_sha") == head_sha}
        for run in sorted(candidates, key=candidates.get, reverse=True):
            merge = self.pull_request_merge(int(run), number, head_sha)
            if not merge or self.commit_tree(merge) != tree:
                continue
            verified, packages = self.run_artifacts(run, verified_name), self.run_artifacts(run, package_name)
            if len(verified) != 1 or len(packages) != 1 or verified[0].get("expired") or packages[0].get("expired"):
                continue
            record = json.loads(self.artifact_file(verified[0]["id"], "verified.json"))
            if (isinstance(record, dict) and record.get("key") == key
                    and str(record.get("github_run_id")) == run and record.get("github_sha") == merge):
                return run, str(packages[0]["id"])
        return "", ""

    def commit_tree(self, sha):
        return self.get(f"git/commits/{sha}")["tree"]["sha"]


def read_member(data, member, limit):
    if len(data) > limit:
        raise ValueError(f"artifact larger than {limit} bytes")
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        info = archive.getinfo(member)
        if info.file_size > limit:
            raise ValueError(f"{member} larger than {limit} bytes")
        if info.flag_bits & 0x1 or info.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED):
            raise ValueError(f"{member} is encrypted or compressed in an unexpected way")
        with archive.open(info) as file:
            content = file.read(limit + 1)
    if len(content) > limit:
        raise ValueError(f"{member} larger than {limit} bytes")
    return content


def pull_request_run(api, kernel, key, number, head_sha):
    """The run of the queued pull request whose packages and verification
    stand in for the merge queue's, or ''. It must have run on a merge
    commit with exactly the queue commit's tree, CI and expected results
    included. The queue then records no verification of its own, so none of
    this becomes evidence a later run trusts. Anything missing, ambiguous or
    unreadable leaves the usual lookup, which trusts the branch and the
    queue only. Returns (run ID, package artifact ID) or ('', '')."""
    tree = git(kernel, "rev-parse", "HEAD^{tree}")
    try:
        return api.pull_request_evidence(key, number, head_sha, tree)
    except (OSError, EOFError, KeyError, TypeError, ValueError, RuntimeError, NotImplementedError,
            http.client.HTTPException, zipfile.BadZipFile, zlib.error) as error:
        print(f"::warning title=Pull request results::not reused: {error}", file=sys.stderr)
    return "", ""


def lookup(api, kernel, recipe_repo, base_sha, force, pull_request=None):
    """pull_request: in the merge queue, (number, head) of the pull request
    being merged, whose own run may stand in for the queue's build and tests."""
    kernel_tree = kernel_id(kernel, "HEAD")
    recipe_commit = git(recipe_repo, "rev-parse", "HEAD")
    recipe_trees = {part: git(recipe_repo, "rev-parse", f"HEAD:{part}") for part in RECIPE_PARTS}
    key = build_key(kernel_tree, recipe_trees)
    out = {"key": key, "kernel_tree": kernel_tree, "recipe_commit": recipe_commit,
           "package_run": "", "verified_run": "", "pull_request_run": "", "package_artifact": "",
           "base_tree": "", "baseline_run": ""}
    if not force:
        out["package_run"] = api.newest_artifact(f"kernel-package-{key}")
        out["verified_run"] = api.newest_artifact(f"kernel-verified-{key}")
        if pull_request and not out["verified_run"]:
            run, package = pull_request_run(api, kernel, key, *pull_request)
            if run:
                out.update(package_run=run, verified_run=run, pull_request_run=run, package_artifact=package)
    if base_sha:
        ensure_commit(kernel, base_sha)
        out["base_tree"] = kernel_id(kernel, base_sha)
        out["baseline_run"] = api.newest_artifact(f"kernel-metadata-{out['base_tree']}")
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--kernel", type=Path, required=True, help="checkout of the commit to build")
    parser.add_argument("--recipe-repo", type=Path, required=True, help="omarchy-pkgs checkout")
    parser.add_argument("--base-sha", default="", help="commit whose build is the comparison baseline")
    parser.add_argument("--force", default="false", help="'true' ignores earlier builds and results")
    parser.add_argument("--merge-queue-ref", default="",
                        help="the merge queue ref --kernel was checked out from: accept the queued pull request's run")
    args = parser.parse_args(argv)

    api = Api(os.environ.get("GITHUB_API_URL", "https://api.github.com"),
              os.environ["GITHUB_REPOSITORY"], os.environ["GITHUB_TOKEN"],
              os.environ["GITHUB_REPOSITORY_ID"])
    try:
        number = queued_pull_request(args.merge_queue_ref)
        head = pull_request_head(args.kernel) if number else ""
        pull_request = (number, head) if head else None
        result = lookup(api, args.kernel, args.recipe_repo, args.base_sha, args.force == "true", pull_request)
    except (subprocess.CalledProcessError, OSError, KeyError, ValueError) as error:
        print(f"::error title=Build lookup::{error}", file=sys.stderr)
        return 1

    print(json.dumps(result, indent=1))
    if os.environ.get("GITHUB_OUTPUT"):
        with open(os.environ["GITHUB_OUTPUT"], "a") as out:
            for name, value in result.items():
                out.write(f"{name}={value}\n")
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        reuse = []
        if result["package_run"]:
            reuse.append(f"packages from run {result['package_run']}")
        if result["verified_run"]:
            reuse.append(f"verified by run {result['verified_run']}")
        if result["pull_request_run"]:
            reuse = [f"packages and verification from run {result['pull_request_run']} "
                     f"of pull request #{pull_request[0]} at {pull_request[1][:12]}"]
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as out:
            out.write(f"Build key `{result['key']}` (kernel `{result['kernel_tree'][:12]}`, "
                      f"omarchy-pkgs `{result['recipe_commit'][:12]}`)"
                      + (f"; reusing {', '.join(reuse)}" if reuse else "") + ".\n\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
