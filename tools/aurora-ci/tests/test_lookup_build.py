# SPDX-License-Identifier: GPL-2.0
import http.client
import io
import json
import subprocess
import unittest
import zipfile

from support import TempDirTestCase, commit_file, git, load

lookup = load("lookup-build")


class FakeApi:
    def __init__(self, artifacts=None, evidence=None):
        self.artifacts, self.asked = artifacts or {}, []
        self.evidence = evidence or {}

    def newest_artifact(self, name):
        self.asked.append(name)
        return self.artifacts.get(name, "")

    def pull_request_evidence(self, key, number, head_sha, tree):
        self.asked.append(("evidence", key, number, head_sha))
        run, tested, package = self.evidence.get((key, number, head_sha), ("", None, ""))
        return (run, package) if tested == tree else ("", "")


class LookupTest(TempDirTestCase):
    def setUp(self):
        super().setUp()
        self.kernel = self.init_repo("kernel")
        self.base = commit_file(self.kernel, "Makefile", "VERSION = 7\n", "kernel")
        commit_file(self.kernel, "tools/lib/x.c", "int x;\n", "a tool")
        self.recipe = self.init_repo("recipe")
        for part in lookup.RECIPE_PARTS:
            path = part if part.startswith(("bin/", "helpers/")) else f"{part}/file"
            commit_file(self.recipe, path, part, f"add {part}")
        commit_file(self.recipe, "bin/omarchy-release", "x", "release tooling")
        commit_file(self.recipe, "pkgbuilds/other/PKGBUILD", "x", "another package")

    def key(self):
        tree = lookup.kernel_id(self.kernel, "HEAD")
        return lookup.build_key(tree, {p: git(self.recipe, "rev-parse", f"HEAD:{p}") for p in lookup.RECIPE_PARTS})

    def test_finds_earlier_build_verification_and_baseline(self):
        key = self.key()
        base_id = lookup.kernel_id(self.kernel, self.base)
        api = FakeApi({f"kernel-package-{key}": "11", f"kernel-verified-{key}": "12",
                       f"kernel-metadata-{base_id}": "13"})
        result = lookup.lookup(api, self.kernel, self.recipe, self.base, force=False)
        self.assertEqual((result["key"], result["package_run"], result["verified_run"]), (key, "11", "12"))
        self.assertEqual((result["base_tree"], result["baseline_run"]), (base_id, "13"))
        self.assertEqual(result["recipe_commit"], git(self.recipe, "rev-parse", "HEAD"))

    def test_force_ignores_earlier_results(self):
        api = FakeApi({f"kernel-package-{self.key()}": "11"})
        result = lookup.lookup(api, self.kernel, self.recipe, "", force=True)
        self.assertEqual((result["package_run"], result["verified_run"], result["baseline_run"]), ("", "", ""))
        self.assertEqual(api.asked, [])

    def test_other_packages_and_tooling_do_not_change_the_key(self):
        before = self.key()
        commit_file(self.recipe, "pkgbuilds/other/PKGBUILD", "y", "bump another package")
        commit_file(self.recipe, "bin/omarchy-release", "y", "release tooling")
        commit_file(self.recipe, "helpers/upstream-watch.py", "y", "an unrelated helper")
        self.assertEqual(self.key(), before)
        commit_file(self.recipe, "helpers/docker-helpers.sh", "changed", "change the builder")
        self.assertNotEqual(self.key(), before)
        before = self.key()
        commit_file(self.recipe, "pkgbuilds/linux-aurora/file", "changed", "change the kernel recipe")
        self.assertNotEqual(self.key(), before)

    def test_kernel_tree_changes_the_key(self):
        for path in ("README", "tools/lib/y.c", "tools/aurora-sep/z.py"):
            with self.subTest(path=path):
                before = self.key()
                commit_file(self.kernel, path, "x\n", "change")
                self.assertNotEqual(self.key(), before)

    def pull_request_api(self, tested_tree=None, artifacts=None):
        tree = git(self.kernel, "rev-parse", "HEAD^{tree}")
        return FakeApi(artifacts, {(self.key(), "7", "c0ffee"): ("21", tested_tree or tree, "77")})

    def queue(self, api, force=False, pull_request=("7", "c0ffee")):
        result = lookup.lookup(api, self.kernel, self.recipe, "", force=force, pull_request=pull_request)
        return result["package_run"], result["verified_run"], result["pull_request_run"], result["package_artifact"]

    def test_the_merge_queue_takes_the_pull_requests_run_of_this_tree(self):
        api = self.pull_request_api(artifacts={f"kernel-package-{self.key()}": "11"})
        self.assertEqual(self.queue(api), ("21", "21", "21", "77"))

    def test_a_run_of_another_tree_does_not_count(self):
        # Same build key, but the run tested other CI files or expectations.
        self.assertEqual(self.queue(self.pull_request_api(tested_tree="0" * 40)), ("", "", "", ""))

    def test_a_trusted_verification_comes_first(self):
        key = self.key()
        api = self.pull_request_api(artifacts={f"kernel-package-{key}": "11", f"kernel-verified-{key}": "12"})
        self.assertEqual(self.queue(api), ("11", "12", "", ""))
        self.assertNotIn(("evidence", key, "7", "c0ffee"), api.asked)

    def test_no_pull_request_run_outside_the_merge_queue_or_when_forced(self):
        for pull_request, force in ((None, False), (("7", "c0ffee"), True)):
            with self.subTest(pull_request=pull_request, force=force):
                api = self.pull_request_api()
                self.assertEqual(self.queue(api, force, pull_request), ("", "", "", ""))
                self.assertEqual([a for a in api.asked if isinstance(a, tuple)], [])

    def test_unreadable_evidence_leaves_the_usual_lookup(self):
        key = self.key()
        for error in (OSError("HTTP 404"), ValueError("incomplete listing"), RuntimeError("encrypted"),
                      NotImplementedError("compression"), zipfile.BadZipFile("bad"),
                      http.client.IncompleteRead(b"{")):
            with self.subTest(error=error):
                api = self.pull_request_api(artifacts={f"kernel-package-{key}": "11"})
                api.pull_request_evidence = lambda key, number, head, tree, error=error: (_ for _ in ()).throw(error)
                self.assertEqual(self.queue(api), ("11", "", "", ""))

    def test_ci_changes_do_not_change_the_key(self):
        before = self.key()
        commit_file(self.kernel, ".github/workflows/kernel-ci.yml", "on: push\n", "workflow")
        commit_file(self.kernel, "tools/aurora-ci/plan.py", "print()\n", "ci tool")
        self.assertEqual(self.key(), before)


class EnsureCommitTest(TempDirTestCase):
    def present(self, repo, sha):
        return subprocess.run(["git", "-C", str(repo), "--no-lazy-fetch", "cat-file", "-e", f"{sha}^{{commit}}"],
                              stderr=subprocess.DEVNULL).returncode == 0

    def test_fetches_only_the_missing_commit_into_a_partial_clone(self):
        # The plan job's checkout: depth 1, blob:none. Asking a partial clone
        # for a missing object fetches it with all of its history.
        origin = self.init_repo("origin")
        first = commit_file(origin, "Makefile", "1\n", "first")
        base = commit_file(origin, "Makefile", "2\n", "base")
        commit_file(origin, "Makefile", "3\n", "head")
        git(origin, "config", "uploadpack.allowFilter", "true")
        git(origin, "config", "uploadpack.allowAnySHA1InWant", "true")
        clone = self.tmp / "clone"
        git(self.tmp, "clone", "--quiet", "--depth=1", "--filter=blob:none", "--no-checkout",
            origin.as_uri(), str(clone))
        lookup.ensure_commit(clone, base)
        self.assertTrue(self.present(clone, base))
        self.assertFalse(self.present(clone, first))


class QueuedPullRequestTest(unittest.TestCase):
    def test_the_number_in_the_queue_ref(self):
        base = "937b727ef9a3a9ff09489d597ef4b28f3b6fdbfa"
        self.assertEqual(lookup.queued_pull_request(f"refs/heads/gh-readonly-queue/integration/aurora/pr-249-{base}"),
                         "249")
        for ref in ("", "refs/heads/integration/aurora", f"refs/heads/gh-readonly-queue/topic/pr-249-{base}",
                    "refs/heads/gh-readonly-queue/integration/aurora/pr-249-short", "refs/pull/249/merge",
                    f"refs/heads/gh-readonly-queue/integration/aurora/pr-x-{base}"):
            with self.subTest(ref=ref):
                self.assertEqual(lookup.queued_pull_request(ref), "")


class PullRequestHeadTest(TempDirTestCase):
    def test_the_second_parent_of_a_merge(self):
        repo = self.init_repo()
        commit_file(repo, "Makefile", "1\n", "base")
        git(repo, "checkout", "--quiet", "-b", "topic")
        head = commit_file(repo, "README", "x\n", "the pull request")
        git(repo, "checkout", "--quiet", "main")
        commit_file(repo, "Makefile", "2\n", "the branch moved")
        git(repo, "merge", "--quiet", "--no-ff", "-m", "Merge\n\nparent " + "0" * 40, "topic")
        self.assertEqual(lookup.pull_request_head(repo), head)
        self.assertEqual(lookup.pull_request_head(repo, "HEAD^1"), "")


class ReadMemberTest(unittest.TestCase):
    def archive(self, files, compression=zipfile.ZIP_DEFLATED):
        data = io.BytesIO()
        with zipfile.ZipFile(data, "w", compression) as archive:
            for name, content in files.items():
                archive.writestr(name, content)
        return data.getvalue()

    def test_reads_a_small_member(self):
        self.assertEqual(lookup.read_member(self.archive({"verified.json": b"{}"}), "verified.json", 1000), b"{}")

    def test_refuses_what_is_not_a_small_record(self):
        # The encryption flag, set in the central directory entry.
        plain = self.archive({"verified.json": b"{}"}, zipfile.ZIP_STORED)
        entry = plain.index(b"PK\x01\x02") + 8
        encrypted = plain[:entry] + bytes([plain[entry] | 0x1]) + plain[entry + 1:]
        cases = {
            "an encrypted member": (encrypted, ValueError),
            "another compression": (self.archive({"verified.json": b"{}"}, zipfile.ZIP_BZIP2), ValueError),
            "a large download": (b"x" * 1001, ValueError),
            "a member that unpacks large": (self.archive({"verified.json": b"0" * 10000}), ValueError),
            "not a zip": (b"not a zip", zipfile.BadZipFile),
            "no record": (self.archive({"other.json": b"{}"}), KeyError),
        }
        for name, (data, error) in cases.items():
            with self.subTest(name):
                with self.assertRaises(error):
                    lookup.read_member(data, "verified.json", 1000)


REPO = 1000
HEAD = "c0ffee" * 6 + "c0ff"


def artifact(run, created, head_repo=REPO, expired=False, name="kernel-package-x", head_sha=HEAD):
    return {"name": name, "expired": expired, "created_at": created,
            "workflow_run": {"id": run, "repository_id": REPO, "head_repository_id": head_repo,
                             "head_sha": head_sha}}


QUEUE = "gh-readonly-queue/integration/aurora/pr-7-0123456789abcdef0123456789abcdef01234567"


MERGE = "b" * 40


def call(number=7, sha=MERGE, path=None, ref=None):
    """A referenced_workflows entry as GitHub records a call from the run's
    own workflow file."""
    return {"path": path or f"o/r/.github/workflows/kernel-ci-jobs.yml@{sha}",
            "ref": ref or f"refs/pull/{number}/merge", "sha": sha}


def run(event="push", path=".github/workflows/kernel-ci.yml", head_repo=REPO, branch=None, head_sha=HEAD,
        repo=REPO, conclusion="success", attempt=1, calls=None):
    if branch is None:
        branch = QUEUE if event == "merge_group" else "integration/aurora"
    return {"event": event, "path": path, "head_branch": branch, "head_sha": head_sha,
            "status": "completed", "conclusion": conclusion, "run_attempt": attempt,
            "repository": {"id": repo}, "head_repository": {"id": head_repo},
            "referenced_workflows": [call()] if calls is None else calls}


class ApiTest(TempDirTestCase):
    def api(self, artifacts, runs):
        api = lookup.Api("https://api.example", "o/r", "token", str(REPO))
        api.calls = []

        def get(path, **query):
            api.calls.append(path)
            if path == "actions/artifacts":
                return {"artifacts": [a for a in artifacts if a["name"] == query["name"]]}
            return runs[int(path.rsplit("/", 1)[1])]
        api.get = get
        return api

    def test_newest_unexpired_artifact_from_a_trusted_run(self):
        api = self.api([artifact(1, "2026-10-20", expired=True), artifact(2, "2026-10-18"),
                        artifact(3, "2026-10-19"), artifact(4, "2026-10-21", name="other")],
                       {2: run(), 3: run("merge_group")})
        self.assertEqual(api.newest_artifact("kernel-package-x"), "3")
        self.assertEqual(self.api([], {}).newest_artifact("kernel-package-x"), "")

    def test_a_fork_run_is_never_reused(self):
        # A fork's pull request uploads a newer artifact under the same name.
        api = self.api([artifact(2, "2026-10-18"), artifact(9, "2026-10-20", head_repo=2000)],
                       {2: run(), 9: run("pull_request", head_repo=2000)})
        self.assertEqual(api.newest_artifact("kernel-package-x"), "2")
        self.assertNotIn("actions/runs/9", api.calls)

    def test_the_run_itself_must_be_trusted(self):
        # The artifact claims this repository, but the run says otherwise.
        cases = {
            "fork head": run("pull_request", head_repo=2000),
            "another workflow": run(path=".github/workflows/something-else.yml"),
            "an untrusted event": run("pull_request_target"),
            "a pull request from this repository": run("pull_request", branch="topic"),
            "a labelled build": run("pull_request", ".github/workflows/kernel-ci-label.yml", branch="topic"),
            "a manual run of another branch": run("workflow_dispatch", branch="topic"),
            "a push to another branch": run("push", branch="topic"),
            "a queue for another branch": run("merge_group", branch="gh-readonly-queue/topic/pr-7-0123"),
            "the branch name as a queue": run("merge_group", branch="integration/aurora"),
        }
        for name, untrusted in cases.items():
            with self.subTest(name):
                api = self.api([artifact(5, "2026-10-20")], {5: untrusted})
                self.assertEqual(api.newest_artifact("kernel-package-x"), "")

    def test_every_trusted_event_on_the_branch_or_its_queue(self):
        for event in sorted(lookup.TRUSTED_EVENTS):
            with self.subTest(event):
                api = self.api([artifact(5, "2026-10-20")], {5: run(event)})
                self.assertEqual(api.newest_artifact("kernel-package-x"), "5")

    def test_an_unreviewed_run_is_skipped_for_an_older_reviewed_one(self):
        # A writer's pull request uploads a newer artifact under the same name.
        api = self.api([artifact(2, "2026-10-18"), artifact(9, "2026-10-20")],
                       {2: run(), 9: run("pull_request", branch="topic")})
        self.assertEqual(api.newest_artifact("kernel-package-x"), "2")

    def pair(self, run_id, created, head_sha=HEAD, head_repo=REPO, key="x"):
        return [dict(artifact(run_id, created, head_repo, name=f"kernel-verified-{key}", head_sha=head_sha),
                     id=run_id * 10),
                dict(artifact(run_id, created, head_repo, name=f"kernel-package-{key}", head_sha=head_sha),
                     id=run_id * 10 + 1)]

    def evidence(self, artifacts, runs, records=None, trees=None, listings=None):
        """listings: run ID -> that run's own artifacts, when not the same
        as the repository-wide ones."""
        api = self.api(artifacts, runs)
        records, trees, listings = records or {}, trees or {MERGE: "tree"}, listings or {}
        search = api.get

        def get(path, **query):
            if path.endswith("/artifacts") and path != "actions/artifacts":
                run_id = int(path.split("/")[2])
                own = [a for a in listings.get(run_id, artifacts)
                       if a["workflow_run"]["id"] == run_id and a["name"] == query["name"]]
                return {"total_count": query.get("total", len(own)), "artifacts": own}
            return search(path, **query)
        api.get = get
        api.artifact_file = lambda artifact_id, member: json.dumps(
            records.get(artifact_id, {"key": "x", "github_run_id": artifact_id // 10, "github_sha": MERGE}))
        api.commit_tree = lambda sha: trees.get(sha, "another tree")
        return api.pull_request_evidence("x", "7", HEAD, "tree")

    def test_the_newest_eligible_run_of_the_pull_request_with_both_artifacts(self):
        pull_request = run("pull_request", branch="topic", head_repo=2000)
        artifacts = (self.pair(3, "2026-10-19", head_repo=2000) + self.pair(4, "2026-10-20", head_repo=2000)
                     + self.pair(5, "2026-10-21", head_repo=2000)[:1])
        self.assertEqual(self.evidence(artifacts, {3: pull_request, 4: pull_request, 5: pull_request}), ("4", "41"))

    def test_a_requested_build_counts(self):
        self.assertEqual(self.evidence(self.pair(3, "2026-10-19"), {
            3: run("pull_request", ".github/workflows/kernel-ci-label.yml", branch="topic")}), ("3", "31"))

    def test_an_older_run_of_this_tree_when_the_newest_ran_another(self):
        other = "c" * 40
        runs = {3: run("pull_request", branch="topic"),
                4: run("pull_request", branch="topic", calls=[call(sha=other)])}
        records = {40: {"key": "x", "github_run_id": 4, "github_sha": other}}
        self.assertEqual(self.evidence(self.pair(3, "2026-10-19") + self.pair(4, "2026-10-20"), runs, records,
                                       {MERGE: "tree", other: "another tree"}), ("3", "31"))

    def test_only_an_eligible_run_of_the_queued_pull_request_at_the_head(self):
        other = "0" * 40
        pull_request = run("pull_request", branch="topic")
        cases = {
            "an earlier head": (self.pair(3, "2026-10-19", head_sha=other),
                                run("pull_request", branch="topic", head_sha=other), None),
            "a run that says another head": (self.pair(3, "2026-10-19"),
                                             run("pull_request", branch="topic", head_sha=other), None),
            "a failed run": (self.pair(3, "2026-10-19"), run("pull_request", branch="topic", conclusion="failure"),
                             None),
            "a re-run": (self.pair(3, "2026-10-19"), run("pull_request", branch="topic", attempt=2), None),
            "another workflow": (self.pair(3, "2026-10-19"),
                                 run("pull_request", ".github/workflows/something-else.yml", branch="topic"), None),
            "pull_request_target": (self.pair(3, "2026-10-19"), run("pull_request_target", branch="topic"), None),
            "another repository": (self.pair(3, "2026-10-19"), run("pull_request", branch="topic", repo=3000), None),
            "no packages": (self.pair(3, "2026-10-19")[:1], pull_request, None),
            "expired packages": (self.pair(3, "2026-10-19"), pull_request,
                                 {3: [self.pair(3, "2026-10-19")[0], dict(self.pair(3, "2026-10-19")[1],
                                                                          expired=True)]}),
            "two package uploads in the run": (self.pair(3, "2026-10-19"), pull_request,
                                               {3: self.pair(3, "2026-10-19")
                                                + [dict(self.pair(3, "2026-10-20")[1], id=99)]}),
        }
        for name, (artifacts, untrusted, listings) in cases.items():
            with self.subTest(name):
                self.assertEqual(self.evidence(artifacts, {3: untrusted}, listings=listings), ("", ""))

    def test_the_run_must_have_run_the_queued_pull_requests_merge_of_this_tree(self):
        # The run's call of the jobs workflow, as GitHub recorded it, names
        # the pull request and the commit it ran; nothing else will do.
        cases = {
            "another pull request of the same head (say, onto another branch)": [call(number=8)],
            "a call to a pinned commit": [{"path": f"o/r/.github/workflows/kernel-ci-jobs.yml@{MERGE}",
                                           "sha": MERGE}],
            "a call to a branch": [call(path="o/r/.github/workflows/kernel-ci-jobs.yml@main", ref="refs/heads/main")],
            "a call to the pull request's merge ref by name":
                [call(path="o/r/.github/workflows/kernel-ci-jobs.yml@refs/pull/7/merge")],
            "a call into another repository": [call(path=f"x/r/.github/workflows/kernel-ci-jobs.yml@{MERGE}")],
            "another workflow file": [call(path=f"o/r/.github/workflows/other.yml@{MERGE}")],
            "two calls (a nested one)": [call(), call(path=f"o/r/.github/workflows/nested.yml@{MERGE}")],
            "no call": [],
            "an abbreviated commit": [call(sha="b" * 12)],
            "not a list": {"path": "x"},
            "not an entry": ["o/r/.github/workflows/kernel-ci-jobs.yml"],
        }
        for name, calls in cases.items():
            with self.subTest(name):
                self.assertEqual(self.evidence(self.pair(3, "2026-10-19"),
                                               {3: run("pull_request", branch="topic", calls=calls)}), ("", ""))
        with self.subTest("a merge of another tree"):
            self.assertEqual(self.evidence(self.pair(3, "2026-10-19"), {3: run("pull_request", branch="topic")},
                                           trees={MERGE: "another tree"}), ("", ""))

    def test_the_record_must_name_the_key_the_run_and_the_merge_it_ran(self):
        pull_request = run("pull_request", branch="topic")
        for record in ({"key": "y", "github_run_id": 3, "github_sha": MERGE},
                       {"key": "x", "github_run_id": 9, "github_sha": MERGE},
                       # A commit with the same tree is not the commit the run ran.
                       {"key": "x", "github_run_id": 3, "github_sha": "c" * 40},
                       {"key": "x", "github_run_id": 3, "github_sha": 7},
                       [], None, "verified"):
            with self.subTest(record=record):
                self.assertEqual(self.evidence(self.pair(3, "2026-10-19"), {3: pull_request}, {30: record},
                                               {MERGE: "tree", "c" * 40: "tree"}), ("", ""))
