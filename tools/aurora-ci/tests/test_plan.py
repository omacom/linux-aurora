# SPDX-License-Identifier: GPL-2.0
import json

from support import TREE, TempDirTestCase, load, read_outputs

plan_module = load("plan")
plan = plan_module.plan

BASE = "a" * 40
HEAD = "b" * 40


def pull_request(action, labels=(), fork=False, label=None, author="someone",
                 association="CONTRIBUTOR", draft=False, sender=None):
    event = {
        "action": action,
        "sender": {"login": sender or author},
        "pull_request": {
            "number": 42,
            "user": {"login": author},
            "author_association": association,
            "draft": draft,
            "labels": [{"name": name} for name in labels],
            "base": {"sha": BASE, "repo": {"full_name": "omacom/linux-aurora"}},
            "head": {"sha": HEAD, "repo": {"full_name": "someone/linux-aurora" if fork
                                           else "omacom/linux-aurora"}},
        },
    }
    if label:
        event["label"] = {"name": label}
    return event


class PullRequestTest(TempDirTestCase):
    def test_push_without_label_runs_checks_only(self):
        for action in ("opened", "synchronize", "reopened", "ready_for_review"):
            with self.subTest(action=action):
                result = plan("pull_request", pull_request(action))
                self.assertEqual(result["tier"], "light")
                self.assertEqual((result["base_sha"], result["head_sha"]), (BASE, HEAD))
                self.assertEqual(result["pr_number"], "42")

    def test_label_on_branch_of_this_repository_builds_every_push(self):
        result = plan("pull_request", pull_request("synchronize", labels=["ci:build"]))
        self.assertEqual(result["tier"], "full")
        self.assertEqual(result["fork"], "false")

    def test_label_on_fork_does_not_build_a_new_push(self):
        result = plan("pull_request", pull_request("synchronize", labels=["ci:build"], fork=True))
        self.assertEqual(result["tier"], "light")
        self.assertEqual(result["fork"], "true")
        self.assertIn("re-apply", result["reason"])

    def test_applying_the_label_builds_even_a_fork(self):
        result = plan("pull_request", pull_request("labeled", labels=["ci:build"], fork=True,
                                                   label="ci:build"))
        self.assertEqual(result["tier"], "full")

    def test_other_labels_do_not_build(self):
        result = plan("pull_request", pull_request("labeled", labels=["ci:build", "docs"], label="docs"))
        self.assertEqual(result["tier"], "light")

    def test_deleted_fork_counts_as_a_fork(self):
        event = pull_request("synchronize", labels=["ci:build"])
        event["pull_request"]["head"]["repo"] = None
        self.assertEqual(plan("pull_request", event)["tier"], "light")

    def test_custom_label(self):
        event = pull_request("labeled", label="build-approved")
        self.assertEqual(plan("pull_request", event, label="build-approved")["tier"], "full")


VOUCHED = """\
# comment
github:Acelogic
github:Teammate
-github:Banned spam
github:banned
-github:Revoked
gitlab:other
"""


class TrustedAuthorTest(TempDirTestCase):
    def setUp(self):
        super().setUp()
        self.vouched = plan_module.read_vouched(VOUCHED)

    def test_read_vouched(self):
        self.assertEqual(self.vouched, {"acelogic": "vouched", "teammate": "vouched",
                                        "banned": "denounced", "revoked": "denounced"})

    def test_vouched_author_builds_every_push_from_a_fork(self):
        for action in ("opened", "synchronize", "reopened", "ready_for_review"):
            with self.subTest(action=action):
                result = plan("pull_request", pull_request(action, fork=True, author="acelogic"),
                              vouched=self.vouched)
                self.assertEqual(result["tier"], "full")
                self.assertIn("acelogic is vouched for", result["reason"])

    def test_unlisted_author_runs_checks_only_whatever_their_access(self):
        for association in ("OWNER", "MEMBER", "COLLABORATOR", "CONTRIBUTOR",
                            "FIRST_TIME_CONTRIBUTOR", "NONE"):
            with self.subTest(association=association):
                event = pull_request("synchronize", fork=True, association=association)
                self.assertEqual(plan("pull_request", event, vouched=self.vouched)["tier"], "light")

    def test_push_by_someone_outside_the_team_runs_checks_only(self):
        for sender in ("stranger", "banned"):
            with self.subTest(sender=sender):
                event = pull_request("synchronize", fork=True, author="Teammate", sender=sender)
                result = plan("pull_request", event, vouched=self.vouched)
                self.assertEqual(result["tier"], "light")
                self.assertIn(sender, result["reason"])

    def test_missing_sender_is_not_trusted(self):
        event = pull_request("synchronize", author="Teammate")
        del event["sender"]
        self.assertEqual(plan("pull_request", event, vouched=self.vouched)["tier"], "light")

    def test_maintainer_label_still_builds_an_unknown_author(self):
        event = pull_request("labeled", label="ci:build", author="stranger", sender="Teammate")
        self.assertEqual(plan("pull_request", event, vouched=self.vouched)["tier"], "full")

    def test_draft_waits_until_ready_for_review(self):
        draft = plan("pull_request", pull_request("synchronize", author="Teammate", draft=True),
                     vouched=self.vouched)
        self.assertEqual(draft["tier"], "light")
        self.assertIn("ready for review", draft["reason"])
        labeled = plan("pull_request", pull_request("labeled", author="Teammate", draft=True,
                                                    label="ci:build"), vouched=self.vouched)
        self.assertEqual(labeled["tier"], "full")

    def test_vouched_fork_names_the_runner(self):
        vouched = {"team": "vouched"}
        for event in (pull_request("synchronize", fork=True, author="team"),
                      pull_request("labeled", labels=["ci:build"], fork=True, label="ci:build",
                                   author="team", draft=True)):
            with self.subTest(action=event["action"]):
                result = plan("pull_request", event, vouched=vouched)
                self.assertEqual(result["tier"], "full")
                self.assertEqual(json.loads(result["runner"]),
                                 ["namespace-profile-arm64-linux-apple-aurora"])

    def test_no_other_pull_request_names_a_runner(self):
        vouched = {"team": "vouched", "gone": "denounced"}
        cases = {
            "same repository": pull_request("synchronize", author="team"),
            "unknown fork": pull_request("synchronize", fork=True),
            "maintainer label on an unknown fork": pull_request(
                "labeled", labels=["ci:build"], fork=True, label="ci:build", sender="team"),
            "unknown maintainer label on a team fork": pull_request(
                "labeled", labels=["ci:build"], fork=True, label="ci:build", author="team",
                sender="outsider"),
            "outsider's push to a team fork": pull_request(
                "synchronize", fork=True, author="team", sender="outsider"),
            "draft team fork": pull_request("synchronize", fork=True, author="team", draft=True),
            "denounced fork": pull_request(
                "labeled", labels=["ci:build"], fork=True, label="ci:build", author="gone",
                sender="team"),
        }
        for name, event in cases.items():
            with self.subTest(name):
                self.assertEqual(plan("pull_request", event, vouched=vouched)["runner"], "")

    def test_denouncement_overrides_access_and_label(self):
        for event in (pull_request("synchronize", author="banned", association="COLLABORATOR"),
                      pull_request("labeled", labels=["ci:build"], label="ci:build", author="Revoked"),
                      pull_request("synchronize", labels=["ci:build"], author="BANNED")):
            with self.subTest(event=event):
                result = plan("pull_request", event, vouched=self.vouched)
                self.assertEqual(result["tier"], "light")
                self.assertIn("denounced", result["reason"])

    def test_without_a_list_no_author_is_trusted(self):
        for event in (pull_request("opened", author="Acelogic"), pull_request("opened", association="MEMBER")):
            with self.subTest(event=event):
                self.assertEqual(plan("pull_request", event)["tier"], "light")

    def test_the_repository_list_is_well_formed(self):
        path = TREE / ".github" / "VOUCHED.td"
        entries = [line for line in path.read_text().splitlines() if line and not line.startswith("#")]
        logins = [entry.lstrip("-").split()[0].partition(":")[2] for entry in entries]
        self.assertTrue(all(entry.lstrip("-").startswith("github:") for entry in entries))
        self.assertTrue(all(len(entry.split()) > 1 for entry in entries if entry.startswith("-")),
                        "a denouncement needs a reason")
        self.assertIn("vouched", plan_module.read_vouched(path.read_text()).values())
        self.assertEqual(logins, sorted(logins, key=str.lower))
        self.assertEqual(len(logins), len({login.lower() for login in logins}))


class OtherEventsTest(TempDirTestCase):
    def test_merge_group_builds_the_queue_commit(self):
        result = plan("merge_group", {"merge_group": {"base_sha": BASE, "head_sha": HEAD}})
        self.assertEqual(result["tier"], "full")
        self.assertEqual((result["base_sha"], result["head_sha"]), (BASE, HEAD))

    def test_push_compares_with_the_previous_tip(self):
        result = plan("push", {"ref": "refs/heads/integration/aurora", "before": BASE, "after": HEAD})
        self.assertEqual(result["tier"], "full")
        self.assertEqual((result["base_sha"], result["head_sha"]), (BASE, HEAD))

    def test_forced_or_new_branch_push_has_no_baseline(self):
        for before, forced in (("0" * 40, False), (BASE, True)):
            with self.subTest(before=before, forced=forced):
                result = plan("push", {"ref": "refs/heads/x", "before": before, "after": HEAD,
                                       "forced": forced})
                self.assertEqual(result["base_sha"], "")

    def test_schedule_and_dispatch(self):
        self.assertEqual(plan("schedule", {})["tier"], "full")
        self.assertEqual(plan("schedule", {})["force"], "true")
        dispatch = plan("workflow_dispatch", {})
        self.assertEqual((dispatch["tier"], dispatch["force"]), ("full", "true"))

    def test_unknown_event_is_an_error(self):
        with self.assertRaises(ValueError):
            plan("issue_comment", {})


class OtherEventsRunnerTest(TempDirTestCase):
    def test_no_run_of_the_branch_names_a_runner(self):
        for name, event in (("schedule", {}), ("workflow_dispatch", {}),
                            ("push", {"after": HEAD, "ref": "refs/heads/integration/aurora"}),
                            ("merge_group", {"merge_group": {"base_sha": BASE, "head_sha": HEAD}})):
            with self.subTest(name):
                self.assertEqual(plan(name, event)["runner"], "")


class MainTest(TempDirTestCase):
    def test_writes_github_outputs(self):
        output, summary = self.github_env()
        event = self.tmp / "event.json"
        event.write_text(json.dumps(pull_request("labeled", label="ci:build")))
        self.assertEqual(plan_module.main(["--event-name", "pull_request", "--event-path", str(event)]), 0)
        values = read_outputs(output)
        self.assertEqual(values["tier"], "full")
        self.assertEqual(values["head_sha"], HEAD)
        self.assertIn("full run", summary.read_text())

    def test_reads_the_vouch_list(self):
        output, _ = self.github_env()
        event = self.tmp / "event.json"
        event.write_text(json.dumps(pull_request("synchronize", fork=True, author="Teammate")))
        vouched = self.tmp / "VOUCHED.td"
        vouched.write_text(VOUCHED)
        self.assertEqual(plan_module.main(["--event-name", "pull_request", "--event-path", str(event),
                                           "--vouched", str(vouched)]), 0)
        self.assertEqual(read_outputs(output)["tier"], "full")

    def test_malformed_event_fails_cleanly(self):
        self.github_env()
        event = self.tmp / "event.json"
        event.write_text(json.dumps({"action": "opened"}))
        self.assertEqual(plan_module.main(["--event-name", "pull_request", "--event-path", str(event)]), 1)
