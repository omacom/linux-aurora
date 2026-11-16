# SPDX-License-Identifier: GPL-2.0
import fnmatch
import json
import re
import sys
import unittest.mock

from support import TREE, TempDirTestCase, commit_file, load

host = load("host-tests")

PY = sys.executable
SCRIPTS = {
    "pass": "print('ok')\n",
    "fail": "import sys; sys.exit(3)\n",
    "slow": "import time; time.sleep(30)\n",
    "dirty": "open('tools/x/left-behind', 'w').write('x')\n",
    "output": "import os, sys; assert not os.path.exists(sys.argv[1]); os.mkdir(sys.argv[1]); "
              "open(os.path.join(sys.argv[1], 'out.txt'), 'w').write('x')\n",
    "environment": "import os, subprocess, sys, tempfile\n"
                   "assert os.environ.get('RUSTUP_TOOLCHAIN') == '1.85.0'\n"
                   "assert 'XDG_CONFIG_HOME' not in os.environ\n"
                   "assert not tempfile.gettempdir().startswith('/home/')\n"
                   "assert os.environ['MARKER'].endswith('/out/marker')\n"
                   "assert subprocess.run(['sudo', 'true']).returncode != 0\n",
    "uses-setup": "import sys; assert open(sys.argv[1] + '.txt').read() == 'made'\n",
}


class RunnerTest(TempDirTestCase):
    def setUp(self):
        super().setUp()
        self.tree = self.init_repo("tree")
        for name, text in SCRIPTS.items():
            commit_file(self.tree, f"tools/x/{name}-tests.py", text, name)

    def suite(self, name, **extra):
        return dict({"name": name, "argv": [PY, "-I", f"tools/x/{name}-tests.py"]}, **extra)

    def run_main(self, *suites, extra=()):
        manifest = self.tmp / "manifest.json"
        manifest.write_text(json.dumps({"suites": list(suites), "excluded": {}}))
        output = self.tmp / "out"
        status = host.main(["--manifest", str(manifest), "--tree", str(self.tree), "--output", str(output), *extra])
        return status, {r["name"]: r for r in json.loads((output / "results.json").read_text())}

    def test_passing_suites_and_their_environment(self):
        self.github_env()
        patcher = unittest.mock.patch.dict("os.environ", {"XDG_CONFIG_HOME": "/home/someone/.config"})
        patcher.start()
        self.addCleanup(patcher.stop)
        status, results = self.run_main(
            self.suite("pass"),
            self.suite("output", argv=[PY, "-I", "tools/x/output-tests.py", "{output}"]),
            self.suite("environment", rust="1.85.0", env={"MARKER": "{output}/marker"}),
            self.suite("uses-setup", argv=[PY, "-I", "tools/x/uses-setup-tests.py", "{output}"],
                       setup=[[PY, "-c", "import sys; open(sys.argv[1] + '.txt', 'w').write('made')", "{output}"]]))
        self.assertEqual({n: r["status"] for n, r in results.items()},
                         {"pass": "pass", "output": "pass", "environment": "pass", "uses-setup": "pass"})
        self.assertEqual(status, 0)

    def test_failure_timeout_and_setup_failure(self):
        self.github_env()
        status, results = self.run_main(
            self.suite("fail"), self.suite("slow", timeout=1),
            self.suite("pass", setup=[[PY, "-c", "raise SystemExit(5)"]]))
        self.assertEqual(status, 1)
        self.assertEqual([r["status"] for r in results.values()],
                         ["exit 3, expected 0", "timeout", "setup failed (5)"])

    def test_expected_failure(self):
        self.github_env()
        status, results = self.run_main(self.suite("fail", expect="fail"),
                                        dict(self.suite("pass", expect="fail"), name="pass-but-should-fail"))
        self.assertEqual(results["fail"]["status"], "pass")
        self.assertEqual(results["pass-but-should-fail"]["status"], "exit 0, expected non-zero")
        self.assertEqual(status, 1)

    def test_known_failures_do_not_block(self):
        _, summary = self.github_env()
        status, results = self.run_main(self.suite("fail", known_failure="stale mocks"),
                                        self.suite("pass", known_failure="was broken"))
        self.assertEqual(status, 0)
        self.assertEqual(host.verdict(results["pass"]), "known failure, now passes")
        self.assertIn("remove `known_failure`", summary.read_text())

    def test_a_suite_that_changes_the_tree_fails(self):
        _, summary = self.github_env()
        status, results = self.run_main(self.suite("dirty"))
        self.assertEqual((status, results["dirty"]["status"]), (1, "pass"))
        self.assertIn("left-behind", summary.read_text())

    def test_parallel_runs_keep_manifest_order(self):
        self.github_env()
        suites = [dict(self.suite("pass"), name=f"pass-{i}") for i in range(6)]
        status, results = self.run_main(*suites, extra=["--jobs", "3"])
        self.assertEqual((status, list(results)), (0, [f"pass-{i}" for i in range(6)]))

    def test_serial_suites_run_after_the_parallel_ones(self):
        self.github_env()
        suites = [dict(self.suite("pass"), name="serial", serial=True)] + [
            dict(self.suite("pass"), name=f"pass-{i}") for i in range(3)]
        status, results = self.run_main(*suites, extra=["--jobs", "2"])
        self.assertEqual((status, list(results)), (0, ["pass-0", "pass-1", "pass-2", "serial"]))

    def test_suites_whose_history_is_missing_are_skipped(self):
        _, summary = self.github_env()
        missing = "f" * 40
        status, results = self.run_main(self.suite("pass", commits=[missing]), dict(self.suite("pass"), name="plain"),
                                        extra=["--fetch-commits"])
        self.assertEqual(status, 0)
        self.assertEqual((host.verdict(results["pass"]), results["plain"]["status"]), ("skipped", "pass"))
        self.assertIn("not in this repository", summary.read_text())

    def test_manifest_validation_and_selection(self):
        manifest = {"suites": [self.suite("pass", group="a"), self.suite("fail", group="b")]}
        self.assertEqual([s["name"] for s in host.select(manifest, ["b"])], ["fail"])
        with self.assertRaises(ValueError):
            host.select(manifest, None, ["missing"])
        path = self.tmp / "m.json"
        path.write_text(json.dumps({"suites": [self.suite("pass"), self.suite("pass")]}))
        with self.assertRaises(ValueError):
            host.load_manifest(path)
        path.write_text(json.dumps({"suites": [self.suite("pass", expect="maybe")]}))
        with self.assertRaises(ValueError):
            host.load_manifest(path)


TEST_LIKE = re.compile(r"(^|/)(test[_-][^/]*|[^/]*[_-]tests?|check-[^/]*|run[^/]*|[^/]*-regression)\.(py|rs|sh)$")


class ManifestTest(TempDirTestCase):
    """The real host-tests.json against the tree it ships in."""

    def setUp(self):
        super().setUp()
        self.manifest = host.load_manifest()

    def workflow_groups(self):
        """The groups the host-tests job's matrix runs."""
        workflow = (TREE / ".github/workflows/kernel-ci-jobs.yml").read_text()
        job = workflow.split("\n  host-tests:\n", 1)[1].split("\n  self-test:\n", 1)[0]
        return re.search(r"^ +group: \[(.*)\]$", job, re.M).group(1).replace(" ", "").split(",")

    def test_the_workflow_runs_every_group(self):
        groups = {suite.get("group", "tools") for suite in self.manifest["suites"]}
        self.assertEqual(sorted(groups), sorted(self.workflow_groups()))

    def test_every_test_like_file_is_listed_or_excluded(self):
        listed = set(self.manifest["excluded"]) | set(self.manifest.get("helpers", {}))
        for suite in self.manifest["suites"]:
            listed.update(part for part in suite["argv"] + sum(suite.get("setup", []), [])
                          if part.startswith("tools/"))
        missing = []
        for root in self.manifest["discovery"]:
            for directory in sorted(TREE.glob(root)):
                for path in directory.rglob("*"):
                    relative = path.relative_to(TREE).as_posix()
                    if (path.is_file() and TEST_LIKE.search(relative)
                            and not relative.startswith("tools/aurora-ci/")
                            and not any(fnmatch.fnmatch(relative, pattern) or relative.startswith(pattern.rstrip("/") + "/")
                                        for pattern in listed)):
                        missing.append(relative)
        self.assertEqual(missing, [], "add these to host-tests.json suites or excluded")

    def test_entries_are_well_formed(self):
        for suite in self.manifest["suites"]:
            with self.subTest(suite=suite["name"]):
                self.assertTrue(suite["argv"])
                self.assertIn(suite.get("group", "tools"), self.workflow_groups())
                for commit in suite.get("commits", []):
                    self.assertRegex(commit, r"^[0-9a-f]{40}$")
                if "known_failure" in suite:
                    self.assertTrue(suite["known_failure"].strip())
        for path, reason in self.manifest["excluded"].items():
            with self.subTest(excluded=path):
                self.assertTrue(reason.strip())
