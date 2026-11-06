"""The M3 Pro's Mesa (mesa-m3), on by default on every M3 Pro (t6030), against the fake Mac of
test_m3_flow.

ProMesaTest: an M3 Pro downloads the package with the other release assets, installs it in a
pacman transaction of its own after the kernel, and records what happened in $STATE/m3-pro-mesa;
--no-m3-mesa, dependencies that are too old, a pacman failure, reruns, upgrades, --uninstall, and a
setup of the owner's own that is left alone.

OtherMacsTest: every other Mac (M1, M2, the M3 Max, the M3 MacBook Air) runs exactly the commands
12.2's script ran, with the same results on disk: nothing Mesa is downloaded, installed, recorded
or removed, and a Mesa the owner installed stays. Every $sudo command is logged too, and the
logs are compared line by line. The Air with --m3-gpu-experiment is compared in test_air_gpu.
"""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import os
import re
import shutil
import subprocess
import tempfile
import unittest

import test_m3_flow as flow

SRC = flow.SRC
VERSION = flow.VERSION
TAG = re.search(r"^TAG=(\S+)$", SRC, re.M).group(1)
PRO_MESA = flow.PRO_MESA
PRO_MESA_VERSION = flow.PRO_MESA_VERSION
# 12.2's script, the last one without the M3 Pro's Mesa.
REL_12_2 = "26cdc069"
# Every $sudo command goes to the fake's log as well, then runs as before.
SUDO_LOG = 'fake_sudo() { echo "sudo $*" >>"$FAKE/log"; "$@"; }\nsudo=fake_sudo\n'
# What mktemp makes: another name on every run.
MKTEMP = re.compile(r"(?:/tmp|" + re.escape(tempfile.gettempdir()) + r")/tmp\.[A-Za-z0-9]{6,}")
# A session setup of the owner's own, as an M3 Pro owner might have it before 12.3.
# The record: its first line, its keys in order (each once), and the keys that repeat (one line
# per path), which may come after user_setup and opt_out.
SCHEMA = "aurora.m3-pro-mesa-state/1"
RUN_KEYS = ["schema", "run_id", "release", "written_at", "boot_id", "kernel", "board", "installer_sha256",
            "installer_source"]
KEYS = RUN_KEYS + ["package", "version", "file", "sha256", "prefix", "result", "installed_version",
                   "installed_by", "preexisting", "user", "render_member", "render_preexisting", "render_added",
                   "render_by_installer", "user_setup_source", "user_setup_session", "user_setup", "opt_out",
                   "opt_out_cmdline", "created_files"]
# An error record: the run's keys, and the ownership history.
ERROR_KEYS = RUN_KEYS + ["package", "result", "record_error", "installed_version", "installed_by", "preexisting",
                         "render_user", "render_preexisting", "render_by_installer"]
REPEATED = {"integration_path", "replaced_package", "render_added_user", "user_setup_path", "user_setup_ignored",
            "opt_out_path", "record_error_key"}
INTEGRATION = re.search(r'^M3_PRO_MESA_INTEGRATION="([^"]*)"$', SRC, re.M).group(1).splitlines()
# The reasons a login gives in mesa-m3's state file (its report, section 4.3).
REASONS = ["active", "opt-out", "not-supported", "experimental", "no-gpu", "no-display", "no-access",
           "incomplete-prefix", "user-setup",
           "user-setup-ldpath", "user-setup-unknown", "missing-soname", "load-failed", "log-unreadable", "gpu-fault",
           "previous-failed"]
# The M3 Airs (M3_AIR_BOARDS), which get mesa-m3 from 12.4 on.
AIRS = re.search(r'^M3_AIR_BOARDS="([^"]*)"$', SRC, re.M).group(1).split()
UUID = r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}"
RESULTS = ["installed", "current", "newer-kept", "skipped-flag", "skipped-deps", "failed"]
TAG = re.search(r"^TAG=(\S+)$", SRC, re.M).group(1)
CHONKSTEP_ENV = (b"CHONKSTEP_M3_CLIENTS=gpu\n"
                 b"CHONKSTEP_M3_MESA_PREFIX=/home/owner/src/mesa-prefix\n"
                 b"CHONKSTEP_M3_XWAYLAND_GLAMOR=1\n")


def old_installer(tc, rev, name):
    old = subprocess.run(["git", "show", f"{rev}:tools/aurora-sep/install-aurora-sep.sh"],
                         cwd=flow.INSTALLER.parent, capture_output=True)
    if old.returncode:
        tc.skipTest(f"git can't show the earlier script ({rev})")
    path = tc.tmp / name
    path.write_bytes(old.stdout)
    version = re.search(rb"^VERSION=(\S+)$", old.stdout, re.M).group(1).decode()
    tc.assertEqual(re.search(rb"^TAG=(\S+)$", old.stdout, re.M).group(1).decode(), "sep-" + version)
    return path, version


def reset_mac(tc, board):
    # Everything a run can write goes; the packages, stubs and scripts stay.
    keep = {"pkgs", "bin"}
    for p in tc.tmp.iterdir():
        if p.name in keep or p.name.startswith(("root-", "install-")):
            continue
        shutil.rmtree(p) if p.is_dir() and not p.is_symlink() else p.unlink()
    for d in ("dt/chosen", "esp/m1n1", "esp/asahi", "etc/default", "state", "fake"):
        (tc.tmp / d).mkdir(parents=True, exist_ok=True)
    tc.home.mkdir(parents=True, exist_ok=True)
    tc.mac(board)


def tree(tc):
    # Every file a run can touch, by path, with its bytes.
    skip = ("pkgs/", "bin/", "fake/log")
    out = {}
    for p in sorted(tc.tmp.rglob("*")):
        rel = p.relative_to(tc.tmp).as_posix()
        if rel.startswith(skip) or rel.startswith(("root-", "install-")) or not p.is_file():
            continue
        out[rel] = p.read_bytes()
    return out


def package_check_log(tc, text):
    lines = text.splitlines()
    queries = [i for i, line in enumerate(lines) if line == 'pacman -Qu --color never']
    if queries:
        tc.assertEqual(len(queries), lines.count('pacman -Sy --noconfirm'))
        for i in queries:
            tc.assertGreater(i, 0)
            tc.assertEqual(lines[i - 1], 'pacman -Sy --noconfirm')
        downloads = [i for i, line in enumerate(lines) if line.startswith('curl ')]
        if downloads: tc.assertLess(queries[0], downloads[0])
    out = []
    for line in lines:
        if line in ('pacman -Qu --color never', 'pacman -Sy --noconfirm', 'sudo pacman -Sy --noconfirm'): continue
        line = line.replace('pacman -Sy --noconfirm --ask 4 ', 'pacman -S --noconfirm --ask 4 ')
        # Releases before the apple-sep.conf rename had no old name to remove on uninstall.
        line = re.sub(r' \S*/etc/aurora-sep-old\.conf(?=\s|$)', '', line)
        out.append(line)
    return out


def refresh_notice(text):
    return re.sub(r'(?m)^(?:\x1b\[[0-9;]*m)*==>(?:\x1b\[[0-9;]*m)* Refreshing the package database\n', '', text)


def same_commands(tc, before, after):
    """The two command logs ran the same commands. The $sudo lines and the others are compared
    each in order, and all lines as a multiset: the two sides of a pipeline such as
    "pacman -Q ... | $sudo tee ..." log in either order."""
    a, b = package_check_log(tc, before), package_check_log(tc, after)
    tc.assertEqual(sorted(b), sorted(a))
    tc.assertEqual([l for l in b if l.startswith("sudo ")], [l for l in a if l.startswith("sudo ")])
    tc.assertEqual([l for l in b if not l.startswith("sudo ")], [l for l in a if not l.startswith("sudo ")])


def release_repository_text(text):
    """Map the historical release and new-report endpoints to their current repository."""
    old = 'https://github.com/iconidentify/aurora-linux/releases/'
    new = 'https://github.com/omacom/linux-aurora/releases/'
    binary = isinstance(text, bytes)
    value = text.decode('utf-8', errors='surrogateescape') if binary else text
    value = value.replace(old, new)
    # Only the unnumbered issue endpoint moved; historical issue IDs stay put.
    value = re.sub(r'https://github\.com/iconidentify/aurora-linux/issues(?![A-Za-z0-9_/-])',
                   'https://github.com/omacom/linux-aurora/issues', value)
    return value.encode('utf-8', errors='surrogateescape') if binary else value


def release_text(text, old, new):
    """Replace only the earlier release's exact legacy tag and package version."""
    text = release_repository_text(text)
    binary = isinstance(text, bytes)
    tag = "sep-" + old
    pattern = r"(?<![A-Za-z0-9._-])" + re.escape(tag) + r"(?![A-Za-z0-9._-])"
    if binary:
        return re.sub(pattern.encode(), TAG.encode(), text).replace(old.encode(), new.encode())
    return re.sub(pattern, TAG, text).replace(old, new)


def as_this_release(run, old, new):
    """A release names its own packages, tag and boot.bin copy; nothing else may differ."""
    out = dict(run)
    out["log"] = release_text(run["log"], old, new)
    out["tree"] = {release_text(k, old, new): release_text(v, old, new)
                   for k, v in run["tree"].items()}
    return out


class ReleaseIdentityComparisonTest(unittest.TestCase):
    def test_only_exact_release_repository_is_mapped(self):
        old = 'https://github.com/iconidentify/aurora-linux/releases/latest/download/install-aurora-sep.sh'
        new = 'https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh'
        self.assertEqual(release_repository_text(old), new)
        self.assertEqual(release_repository_text(old.encode()), new.encode())
        for untouched in (old.replace('github.com', 'wrong.example'),
                          old.replace('aurora-linux/', 'aurora-linux-other/'),
                          old.replace('/releases/', '/blob/'),
                          old.replace('https://', 'http://')):
            self.assertEqual(release_repository_text(untouched), untouched)
        for changed in (new.replace('github.com', 'wrong.example'), new+' --insecure',
                        new.replace('install-aurora-sep.sh', 'foreign.sh')):
            with self.subTest(changed=changed), self.assertRaises(AssertionError):
                same_commands(self, new, changed)

    def test_report_endpoint_moves_but_numbered_reports_stay_historical(self):
        old = 'https://github.com/iconidentify/aurora-linux/issues'
        self.assertEqual(release_repository_text(old),
                         'https://github.com/omacom/linux-aurora/issues')
        self.assertEqual(release_repository_text(old+'/35'), old+'/35')
        self.assertEqual(release_repository_text(old+'/147'), old+'/147')
        self.assertEqual(release_repository_text(old+'-else'), old+'-else')
        self.assertEqual(release_repository_text(old+'/new'), old+'/new')
        self.assertEqual(release_repository_text(old.replace('github.com','wrong.example')),
                         old.replace('github.com','wrong.example'))

    def test_only_known_identity_is_normalized(self):
        old = "7.1.12.aurora2-12.2"
        command = f"curl https://github.com/iconidentify/aurora-linux/releases/download/sep-{old}/linux-{old}.pkg"
        expected = f"curl https://github.com/omacom/linux-aurora/releases/download/{TAG}/linux-{VERSION}.pkg"
        self.assertEqual(release_text(command, old, VERSION), expected)
        for changed in (expected.replace("github.com", "wrong.example"), expected + " --insecure",
                        expected.replace("linux-", "other-")):
            with self.subTest(changed=changed), self.assertRaises(AssertionError):
                same_commands(self, expected, changed)
        unknown = "sep-" + old + ".unexpected"
        self.assertEqual(release_text(unknown, old, VERSION), "sep-" + VERSION + ".unexpected")


def run_with(tc, installer, board, run, setup=None):
    """Runs `run` with `installer` on a fresh fake `board` (after `setup`), and returns what is
    compared: the exit statuses, the command log and every file a run can touch."""
    reset_mac(tc, board)
    if setup:
        setup()
    tc.installer = installer
    try:
        codes = run()
    finally:
        tc.installer = flow.INSTALLER
    return {"codes": codes, "log": MKTEMP.sub("<tmp>", tc.log()), "tree": tree(tc)}


def owner_mesa(tc):
    # A Mac whose owner installed Mesa packages and a prefix of their own, and set up a session
    # for it, before this script ever ran.
    with open(tc.fake / "installed", "a") as f:
        f.write("mesa-m3\nmesa-m3-g15g\n")
    (tc.fake / "versions").write_text("mesa-m3 26.0.0.owner-1\nmesa-m3-g15g 26.1.4.g15g1-5\n")
    (tc.tmp / "opt/mesa-m3/lib").mkdir(parents=True)
    (tc.tmp / "opt/mesa-m3/lib/libvulkan_asahi.so").write_bytes(b"the owner's own build")
    (tc.home / ".config/chonkstep").mkdir(parents=True)
    (tc.home / ".config/chonkstep/m3gpu-session.env").write_bytes(CHONKSTEP_ENV)


def parse_record(tc, data):
    """The record as {key: value} plus {repeated key: [values]}, after checking its form: the
    schema first, key=value lines, every scalar key exactly once and in order, and only the
    declared keys repeated."""
    text = data.decode()
    tc.assertTrue(text.endswith("\n"), text)
    lines = text[:-1].split("\n")
    tc.assertNotIn("\r", text)
    tc.assertEqual(lines[0], f"schema={SCHEMA}")
    scalars, lists, order = {}, {k: [] for k in REPEATED}, []
    for line in lines:
        key, sep, value = line.partition("=")
        tc.assertEqual(sep, "=", line)
        tc.assertRegex(key, r"^[a-z0-9_]+$")
        if key in REPEATED:
            lists[key].append(value)
        else:
            tc.assertNotIn(key, scalars, f"{key} twice")
            scalars[key] = value
            order.append(key)
    tc.assertRegex(scalars["run_id"], f"^{UUID}$")
    if scalars["result"] == "record-error":
        tc.assertEqual(order, ERROR_KEYS)
        tc.assertIn(scalars["record_error"], ("value", "write"))
        tc.assertEqual(bool(lists["record_error_key"]), scalars["record_error"] == "value")
        tc.assertEqual(lists["user_setup_path"] + lists["opt_out_path"], [])
        return scalars, lists
    tc.assertEqual(order, KEYS)
    tc.assertEqual(lists["record_error_key"], [])
    tc.assertEqual(lists["integration_path"], INTEGRATION)
    tc.assertIn(scalars["user_setup_source"], ("package-detector", "installer-builtin"))
    tc.assertIn(scalars["user_setup"], ("present", "none", "unknown"))
    tc.assertIn(scalars["user_setup_session"], ("hyprland", "unknown"))
    for line in lists["user_setup_ignored"]:
        tc.assertRegex(line, r"^(all|desktop:[a-z0-9_+:-]+) (sure|unsure) /")
    if scalars["user_setup"] != "unknown":
        tc.assertEqual(scalars["user_setup"], "present" if lists["user_setup_path"] else "none")
    tc.assertIn(scalars["render_member"], ("yes", "no", "unknown"))
    tc.assertIn(scalars["render_preexisting"], ("yes", "no", "unknown"))
    tc.assertIn(scalars["render_added"], ("yes", "no", "failed"))
    tc.assertIn(scalars["render_by_installer"], ("yes", "no"))
    tc.assertEqual(scalars["user"] in lists["render_added_user"], scalars["render_by_installer"] == "yes")
    tc.assertEqual(scalars["opt_out"],
                   "present" if lists["opt_out_path"] or scalars["opt_out_cmdline"] == "yes" else "none")
    return scalars, lists


def record_warnings(proc):
    """The warnings about the M3 Pro's Mesa record, one string each."""
    blocks = " ".join(proc.stderr.split()).split("warning:")[1:]
    return [b for b in blocks if "m3-pro-mesa" in b or "this run's record" in b]


def summary_line(tc, proc):
    """The summary's record line, as {run_id, result, write}."""
    lines = [l.strip() for l in proc.stdout.splitlines() if l.strip().startswith("m3-pro-mesa record:")]
    tc.assertEqual(len(lines), 1, proc.stdout)
    m = re.fullmatch(rf"m3-pro-mesa record: run_id=({UUID}) result=(\S+) write=(\S+)", lines[0])
    tc.assertTrue(m, lines[0])
    return dict(zip(("run_id", "result", "write"), m.groups()))


class ProMesaTest(flow.M3FlowBase):

    def record(self):
        return self.record_lists()[0]

    def record_lists(self):
        path = self.state / "m3-pro-mesa"
        self.assertEqual(path.stat().st_mode & 0o7777, 0o644)
        return parse_record(self, path.read_bytes())

    def installed(self):
        return (self.fake / "installed").read_text().split()

    def temps(self):
        return sorted(p.name for p in self.state.iterdir() if p.name.startswith(".m3-pro-mesa."))

    def stale(self):
        return sorted(p.name for p in self.state.iterdir() if p.name.startswith("m3-pro-mesa.stale-"))

    def mesa_lines(self, since=0):
        # What changes packages (pacman -U, -R...), for the Mesa; not the -Q queries.
        return [l for l in self.log()[since:].splitlines()
                if l.startswith("pacman") and not l.startswith("pacman -Q") and "mesa" in l]

    def older(self, version="26.1.3.g15s1-1"):
        name = f"mesa-m3-{version}-aarch64.pkg.tar.zst"
        self.pro_mesa_fixture(name, "mesa-m3", version, "opt/mesa-m3")
        return name

    def owner_has(self, version):
        with open(self.fake / "installed", "a") as f:
            f.write("mesa-m3\n")
        (self.fake / "versions").write_text(f"mesa-m3 {version}\n")

    def test_every_m3_pro_gets_it_after_the_kernel(self):
        for board in ("j516s", "j514s"):
            with self.subTest(board=board):
                reset_mac(self, board)
                proc = self.install()
                self.assertIn(PRO_MESA, self.downloaded())
                lines = self.log().splitlines()
                u = [i for i, l in enumerate(lines) if l.startswith("pacman -U")]
                # Two transactions: the kernel's (without the Mesa package), then the Mesa's alone.
                self.assertEqual(len(u), 2, lines)
                self.assertNotIn("mesa", lines[u[0]])
                self.assertIn("linux-aurora-", lines[u[0]])
                self.assertRegex(lines[u[1]], rf"^pacman -U --noconfirm \S+/m3-pro/{re.escape(PRO_MESA)}$")
                self.assertIn("mesa-m3", self.installed())
                rec = self.record()
                self.assertEqual(rec["package"], "mesa-m3")
                self.assertEqual(rec["version"], PRO_MESA_VERSION)
                self.assertEqual(rec["file"], PRO_MESA)
                self.assertEqual(rec["sha256"], self.shas[PRO_MESA])
                self.assertEqual(rec["prefix"], "/opt/mesa-m3")
                self.assertEqual(rec["result"], "installed")
                self.assertEqual(rec["installed_version"], PRO_MESA_VERSION)
                self.assertEqual(rec["installed_by"], "installer")
                self.assertEqual(rec["preexisting"], "none")
                self.assertEqual(rec["user_setup_source"], "installer-builtin")   # no detector in the fake
                self.assertEqual((rec["render_member"], rec["render_preexisting"], rec["render_added"],
                                  rec["render_by_installer"]), ("yes", "no", "yes", "yes"))
                self.assertEqual(rec["user_setup"], "none")
                self.assertEqual(rec["opt_out"], "none")
                self.assertEqual(rec["opt_out_cmdline"], "no")
                self.assertEqual(rec["created_files"], "0")
                self.assertEqual(summary_line(self, proc),
                                 {"run_id": rec["run_id"], "result": "installed", "write": "ok"})
                out = " ".join(proc.stdout.split())
                self.assertIn(f"The M3 Pro's Mesa (mesa-m3 {PRO_MESA_VERSION}, in /opt/mesa-m3) is installed.", out)
                self.assertIn("The new graphics take effect at the next login", out)
                # j514s is not on the handoff list: kernel-only, so the package's login check stays off.
                self.assertEqual("boots kernel-only (no GPU handoff)" in out, board == "j514s")

    def test_the_invoking_user_is_recorded(self):
        self.mac("j516s")
        self.extra_env["SUDO_USER"] = "someone"
        self.install()
        self.assertEqual(self.record()["user"], "someone")
        self.extra_env.pop("SUDO_USER")
        self.extra_env["USER"] = "ignored"
        self.install()
        me = subprocess.run(["id", "-un"], capture_output=True, text=True).stdout.strip()
        self.assertEqual(self.record()["user"], me)

    def test_no_m3_mesa_leaves_it_out(self):
        self.mac("j516s")
        proc = self.install(env="M3_PRO_MESA=0")
        self.assertNotIn(PRO_MESA, self.downloaded())
        self.assertEqual(self.mesa_lines(), [])
        self.assertNotIn("mesa-m3", self.installed())
        self.assertEqual(self.record()["result"], "skipped-flag")
        self.assertIn("--no-m3-mesa: leaving out the M3 Pro's Mesa (mesa-m3)", proc.stdout)
        self.assertIn("the desktop renders in software", proc.stdout)
        self.assertIn("linux-aurora", self.installed())

    def test_no_m3_mesa_keeps_an_installed_copy(self):
        self.mac("j516s")
        self.install()
        since = len(self.log())
        proc = self.install(env="M3_PRO_MESA=0")
        self.assertEqual(self.mesa_lines(since), [])     # neither -U nor -R
        self.assertIn("mesa-m3", self.installed())
        self.assertIn(f"the {PRO_MESA_VERSION} installed earlier stays as it is", proc.stdout)
        rec = self.record()
        self.assertEqual(rec["result"], "skipped-flag")
        self.assertEqual(rec["preexisting"], "none")     # still this script's: --uninstall removes it

    def test_dependencies_too_old(self):
        self.mac("j516s")
        self.extra_env.update(FAKE_GLIBC="2.42+r1-1", FAKE_SPIRV_TOOLS="")
        proc = self.install(check=False)
        self.assertEqual(proc.returncode, 3, proc.stderr)
        self.assertIn("linux-aurora", self.installed())      # the kernel install is complete
        self.assertIn(PRO_MESA, self.downloaded())
        self.assertFalse([l for l in self.mesa_lines() if l.startswith("pacman -U")])
        self.assertNotIn("mesa-m3", self.installed())
        err = " ".join(proc.stderr.split())
        self.assertIn("glibc 2.42+r1-1 (needs 2.43 or newer); spirv-tools not installed (needs 1:1.4.357.0 or newer)", err)
        self.assertIn("sudo pacman -Syu glibc spirv-tools", err)
        self.assertEqual(err.count("left out the M3 Pro's Mesa"), 1)
        self.assertEqual(self.record()["result"], "skipped-deps")
        out = " ".join(proc.stdout.split())
        self.assertIn("The M3 Pro's Mesa was NOT installed: it needs newer packages", out)
        self.assertIn("The exit status is 3.", out)

    def test_split_libgcc_and_libstdcxx(self):
        # Current Arch Linux ARM ships libgcc and libstdc++ as packages of their own; an older
        # system may have them from a package that provides them. pacman -T decides either way.
        self.mac("j516s")
        for case, env, unmet in (
                ("separate packages", {}, None),
                ("provided by another package", {"FAKE_LIBGCC": "", "FAKE_LIBSTDCXX": "",
                                                 "FAKE_PROVIDES": "libgcc=15.2.1-1 libstdc++=15.2.1-1"}, None),
                ("missing", {"FAKE_LIBGCC": "", "FAKE_PROVIDES": "libstdc++=15.2.1-1"},
                 "libgcc not installed (needs 3.0 or newer)"),
                ("too old", {"FAKE_LIBSTDCXX": "10.2.0-1"}, "libstdc++ 10.2.0-1 (needs 11.1 or newer)")):
            with self.subTest(case):
                reset_mac(self, "j516s")
                self.extra_env.update(env)
                try:
                    proc = self.install(check=False)
                finally:
                    for k in env:
                        self.extra_env.pop(k)
                if unmet is None:
                    self.assertEqual(proc.returncode, 0, proc.stderr)
                    self.assertEqual(self.record()["result"], "installed")
                else:
                    self.assertEqual(proc.returncode, 3)
                    self.assertEqual(self.record()["result"], "skipped-deps")
                    self.assertIn(unmet, " ".join(proc.stderr.split()))
                    self.assertIn(f"sudo pacman -Syu {unmet.split()[0]})", " ".join(proc.stderr.split()))

    def test_a_bare_name_needs_only_to_be_installed(self):
        self.mac("j516s")
        self.extra_env.update(FAKE_SPIRV_TOOLS="")
        proc = self.install(env='M3_PRO_MESA_NEEDS="glibc>=2.43 spirv-tools"', check=False)
        self.assertEqual(proc.returncode, 3)
        self.assertIn("spirv-tools not installed.", " ".join(proc.stderr.split()))
        self.extra_env.update(FAKE_SPIRV_TOOLS="1:1.0-1")
        self.install(env='M3_PRO_MESA_NEEDS="glibc>=2.43 spirv-tools"')
        self.assertEqual(self.record()["result"], "installed")

    def test_a_pacman_failure_keeps_the_kernel_install(self):
        self.mac("j516s")
        self.extra_env["FAKE_FAIL_U_FOR"] = "mesa-m3-*"
        proc = self.install(check=False)
        self.assertEqual(proc.returncode, 3, proc.stderr)
        self.assertIn("linux-aurora", self.installed())
        self.assertNotIn("mesa-m3", self.installed())
        self.assertIn("could not install the M3 Pro's Mesa (mesa-m3)", " ".join(proc.stderr.split()))
        self.assertIn("pacman could not install it", " ".join(proc.stdout.split()))
        self.assertEqual(self.record()["result"], "failed")
        # The next run installs it.
        self.extra_env.pop("FAKE_FAIL_U_FOR")
        self.install()
        self.assertEqual(self.record()["result"], "installed")

    def test_a_package_by_another_name_is_not_installed(self):
        self.mac("j516s")
        self.pro_mesa_fixture(PRO_MESA, "mesa", PRO_MESA_VERSION, "usr")
        proc = self.install(check=False)
        self.assertEqual(proc.returncode, 3)
        self.assertFalse([l for l in self.mesa_lines() if l.startswith("pacman -U")])
        self.assertIn('names the package "mesa", not mesa-m3', " ".join(proc.stderr.split()))
        self.assertEqual(self.record()["result"], "failed")

    def test_a_failed_kernel_install_installs_no_mesa(self):
        self.mac("j516s")
        self.extra_env["FAKE_FAIL_U_FOR"] = "linux-aurora-*"
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertNotIn("mesa-m3", self.installed())
        self.assertFalse([l for l in self.mesa_lines() if l.startswith("pacman -U")])
        self.assertFalse((self.state / "m3-pro-mesa").exists())

    def test_same_version_rerun_installs_nothing_new(self):
        self.mac("j516s")
        self.install()
        since, n = len(self.log()), len(self.downloaded())
        proc = self.install()
        self.assertNotIn(PRO_MESA, self.downloaded()[n:])
        self.assertEqual(self.mesa_lines(since), [])
        self.assertIn(f"The M3 Pro's Mesa (mesa-m3 {PRO_MESA_VERSION}) is installed already", proc.stdout)
        rec = self.record()
        self.assertEqual(rec["result"], "current")
        self.assertEqual(rec["preexisting"], "none")

    def test_an_older_one_from_an_earlier_run_is_upgraded(self):
        self.mac("j516s")
        new = self.pro_mesa
        self.pro_mesa = self.older()
        self.install()
        self.assertEqual(self.record()["installed_version"], "26.1.3.g15s1-1")
        self.pro_mesa = new
        since = len(self.log())
        self.install()
        self.assertEqual(len([l for l in self.mesa_lines(since) if l.startswith("pacman -U")]), 1)
        rec = self.record()
        self.assertEqual((rec["result"], rec["installed_version"], rec["preexisting"]),
                         ("installed", PRO_MESA_VERSION, "none"))
        self.uninstall()
        self.assertNotIn("mesa-m3", self.installed())

    def test_a_newer_one_is_kept(self):
        self.mac("j516s")
        self.owner_has("27.0.0-1")
        proc = self.install()
        self.assertNotIn(PRO_MESA, self.downloaded())
        self.assertEqual(self.mesa_lines(), [])
        self.assertIn(f"A newer mesa-m3 (27.0.0-1) than this release's ({PRO_MESA_VERSION}) is installed; keeping it",
                      proc.stdout)
        self.assertEqual(self.record()["result"], "newer-kept")

    def test_uninstall_removes_what_this_script_installed(self):
        self.mac("j516s")
        self.install()
        since = len(self.log())
        proc = self.uninstall()
        self.assertIn("pacman -Rn --noconfirm mesa-m3", self.log()[since:].splitlines())
        self.assertNotIn("mesa-m3", self.installed())
        self.assertIn(f"Removed the M3 Pro's Mesa (mesa-m3 {PRO_MESA_VERSION})", proc.stdout)
        self.assertFalse((self.state / "m3-pro-mesa").exists())

    def test_uninstall_keeps_the_owners_own(self):
        # Installed before this script first ran, at this release's version, an older one this
        # script updated, or one it has no record of: it stays.
        for case in ("same", "older", "no-record"):
            with self.subTest(case=case):
                reset_mac(self, "j516s")
                self.owner_has(PRO_MESA_VERSION if case != "older" else "26.0.0-1")
                self.install()
                if case == "no-record":
                    (self.state / "m3-pro-mesa").unlink()
                since = len(self.log())
                proc = self.uninstall()
                self.assertEqual(self.mesa_lines(since), [])
                self.assertIn("mesa-m3", self.installed())
                self.assertIn("Keeping mesa-m3", proc.stdout)

    def test_uninstall_after_no_m3_mesa_removes_nothing(self):
        self.mac("j516s")
        self.install(env="M3_PRO_MESA=0")
        self.assertEqual(self.record()["installed_by"], "none")
        since = len(self.log())
        self.uninstall()
        self.assertEqual(self.mesa_lines(since), [])

    def test_one_the_owner_installed_after_this_script_stays(self):
        # --no-m3-mesa (or a failed install), then the owner installs mesa-m3 themselves.
        for first in ("M3_PRO_MESA=0", "FAIL"):
            with self.subTest(first=first):
                reset_mac(self, "j516s")
                if first == "FAIL":
                    self.extra_env["FAKE_FAIL_U_FOR"] = "mesa-m3-*"
                    self.install(check=False)
                    self.extra_env.pop("FAKE_FAIL_U_FOR")
                else:
                    self.install(env=first)
                self.owner_has(PRO_MESA_VERSION)
                self.install(env="M3_PRO_MESA=0")
                self.assertEqual(self.record()["installed_by"], "owner")
                since = len(self.log())
                proc = self.uninstall()
                self.assertEqual(self.mesa_lines(since), [])
                self.assertIn("Keeping mesa-m3", proc.stdout)
                self.assertIn("mesa-m3", self.installed())

    def test_a_rerun_keeps_it_this_scripts(self):
        # current, --no-m3-mesa and a failed upgrade keep "installer" while it stays installed.
        self.mac("j516s")
        self.install()
        self.install()
        self.assertEqual(self.record()["installed_by"], "installer")
        self.install(env="M3_PRO_MESA=0")
        self.assertEqual(self.record()["installed_by"], "installer")
        self.uninstall()
        self.assertNotIn("mesa-m3", self.installed())

    def test_a_failed_removal_is_a_warning(self):
        self.mac("j516s")
        self.install()
        proc = self.run_sh("pacman() { [[ $1 == -Rn ]] && return 1; command pacman \"$@\"; }\nuninstall_all")
        self.assertIn("could not remove mesa-m3; remove it with: sudo pacman -R mesa-m3", " ".join(proc.stderr.split()))
        self.assertIn("Done. Reboot to run", proc.stdout)

    def test_the_owners_setup_is_left_as_it_is(self):
        self.mac("j516s")
        # A home whose path has spaces (and an "=" and a ":"): each path stays whole.
        self.home_path = h = "/home dir/of the=owner: x"
        self.home = home = self.sysroot / h.lstrip("/")
        home.mkdir(parents=True)
        files = {
            ".config/chonkstep/m3gpu-session.env": CHONKSTEP_ENV,
            ".config/environment.d/90-vulkan.conf": b"VK_ICD_FILENAMES=/home/owner/icd.json\n",
            ".config/environment.d/91-ours.conf": b"VK_ICD_FILENAMES=/opt/mesa-m3/share/vulkan/icd.d/x.json\n",
            ".config/hypr/hyprland.conf": b"env = LIBGL_DRIVERS_PATH,/home/owner/dri\n",
            ".drirc": b'<driconf><device><application><option name="dri_driver" value="zink"/>'
                      b'</application></device></driconf>\n',
            ".config/drirc": b"<driconf/>\n",                                       # no driver choice
            ".config/mesa-m3/disable": b"",
            "unrelated.txt": b"keep me\n",
        }
        for rel, data in files.items():
            (home / rel).parent.mkdir(parents=True, exist_ok=True)
            (home / rel).write_bytes(data)
        before = {p: (p.read_bytes(), p.stat().st_mtime_ns) for p in home.rglob("*") if p.is_file()}
        proc = self.install()
        self.assertEqual(self.record()["result"], "installed")       # still on by default
        out = " ".join(proc.stdout.split())
        self.assertIn(f"Left as it is: {self.record()['user']}'s own M3 Mesa setup ({h}/.config/environment.d/"
                      "90-vulkan.conf:1 sets VK_ICD_FILENAMES (/home/owner/icd.json); ", out)
        self.assertIn(f"{h}/.config/hypr/hyprland.conf:1 sets LIBGL_DRIVERS_PATH (/home/owner/dri); ", out)
        self.assertIn(f"{h}/.drirc chooses a driver (dri_driver)).", out)
        # chonkstep's settings: listed, but a Hyprland session does not read them.
        self.assertIn("Not counted, as a Hyprland session does not read it (left as it is): desktop:chonkstep sure "
                      f"{h}/.config/chonkstep/m3gpu-session.env sets CHONKSTEP_M3_MESA_PREFIX to "
                      "/home/owner/src/mesa-prefix", out)
        self.assertNotIn("91-ours.conf", out)                          # it points at the package's prefix
        self.assertNotIn(f"{h}/.config/drirc", out)
        self.assertIn(f"switched off at login by {h}/.config/mesa-m3/disable: off for", out)
        rec, lists = self.record_lists()
        self.assertEqual((rec["user_setup_source"], rec["user_setup"]), ("installer-builtin", "present"))
        self.assertEqual(rec["user_setup_session"], "hyprland")
        self.assertEqual(lists["user_setup_path"], [
            f"{h}/.config/environment.d/90-vulkan.conf", f"{h}/.config/hypr/hyprland.conf", f"{h}/.drirc"])
        self.assertEqual(lists["user_setup_ignored"], [
            f"desktop:chonkstep sure {h}/.config/chonkstep/m3gpu-session.env sets CHONKSTEP_M3_MESA_PREFIX to "
            "/home/owner/src/mesa-prefix"])
        self.assertEqual(rec["opt_out"], "present")
        self.assertEqual(lists["opt_out_path"], [f"{h}/.config/mesa-m3/disable"])
        self.assertEqual(rec["opt_out_cmdline"], "no")
        self.uninstall()
        after = {p: (p.read_bytes(), p.stat().st_mtime_ns) for p in home.rglob("*") if p.is_file()}
        self.assertEqual(after, before)

    def test_chonkstep_alone_is_not_a_setup_of_the_hyprland_session(self):
        # The lab m3pro today: only chonkstep's file names a private prefix. Stock Hyprland does
        # not read it: user_setup=none, the finding listed as ignored.
        self.mac("j516s")
        (self.home / ".config/chonkstep").mkdir(parents=True)
        (self.home / ".config/chonkstep/m3gpu-session.env").write_bytes(CHONKSTEP_ENV)
        proc = self.install()
        rec, lists = self.record_lists()
        self.assertEqual((rec["user_setup"], rec["user_setup_session"], lists["user_setup_path"]),
                         ("none", "hyprland", []))
        self.assertEqual(lists["user_setup_ignored"], [
            f"desktop:chonkstep sure {self.home_path}/.config/chonkstep/m3gpu-session.env sets "
            "CHONKSTEP_M3_MESA_PREFIX to /home/owner/src/mesa-prefix"])
        out = " ".join(proc.stdout.split())
        self.assertNotIn("Left as it is: ", out.replace("Left as it is): ", ""))
        self.assertIn("Not counted, as a Hyprland session does not read it", out)
        # A uwsm env file for Hyprland choosing a private Mesa is one it reads.
        (self.home / ".config/uwsm").mkdir(parents=True)
        (self.home / ".config/uwsm/env-hyprland").write_text("export GALLIUM_DRIVER=zink\n")
        self.install()
        rec, lists = self.record_lists()
        self.assertEqual((rec["user_setup"], lists["user_setup_path"]),
                         ("present", [f"{self.home_path}/.config/uwsm/env-hyprland"]))

    def test_an_unreadable_setting_is_unknown_never_none(self):
        self.mac("j516s")
        f = self.home / ".config/hypr/hyprland.conf"
        f.parent.mkdir(parents=True)
        f.write_text("env = GALLIUM_DRIVER,zink\n")
        f.chmod(0)
        self.addCleanup(f.chmod, 0o644)
        self.install()
        rec, lists = self.record_lists()
        self.assertEqual((rec["user_setup"], lists["user_setup_path"]), ("unknown", [f"{self.home_path}/.config/hypr/hyprland.conf"]))

    def test_the_system_opt_out_is_reported_and_kept(self):
        self.mac("j516s")
        off = self.sysroot / "etc/mesa-m3/disable"
        off.parent.mkdir(parents=True)
        off.write_text("")
        proc = self.install()
        self.assertIn("switched off at login by /etc/mesa-m3/disable: off for every user", " ".join(proc.stdout.split()))
        rec, lists = self.record_lists()
        self.assertEqual((rec["opt_out"], lists["opt_out_path"]), ("present", ["/etc/mesa-m3/disable"]))
        self.uninstall()
        self.assertTrue(off.exists())

    def test_placeholders_stop_before_anything_is_downloaded(self):
        shipped_pkg = re.search(r'^M3_PRO_MESA_PACKAGE="([^"]*)"$', SRC, re.M).group(1)
        shipped_needs = re.search(r'^M3_PRO_MESA_NEEDS="([^"]*)"$', SRC, re.M).group(1)
        bad = [f'M3_PRO_MESA_PACKAGE="mesa-m3-PENDING-aarch64.pkg.tar.zst PENDING-U1-PACKAGE-BUILD"',
               'M3_PRO_MESA_NEEDS="PENDING-U1-INVENTORY"',
               f'M3_PRO_MESA_PACKAGE="{PRO_MESA}"',
               f'M3_PRO_MESA_PACKAGE="mesa-m3-g15g-26.1.4.g15g1-5-aarch64.pkg.tar.zst {"0" * 64}"',
               f'M3_PRO_MESA_PACKAGE="mesa-m3-1-1-aarch64.pkg.tar.zst {"0" * 63}"',
               'M3_PRO_MESA_NEEDS="glibc>=2.43 ;rm"',
               'M3_PRO_MESA_DETECTOR="PENDING-U1-DETECTOR"',
               'M3_PRO_MESA_SETUP_LIST="PENDING-U1-SETUP-LIST"',
               'M3_PRO_MESA_DETECTOR="relative/path"']
        if "PENDING" in shipped_pkg + shipped_needs:
            bad.append(f'M3_PRO_MESA_PACKAGE="{shipped_pkg}"\nM3_PRO_MESA_NEEDS="{shipped_needs}"')
        for env in bad:
            with self.subTest(env=env):
                reset_mac(self, "j516s")
                proc = self.install(env=env, check=False)
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn("(a packaging mistake). Nothing was installed.", proc.stderr)
                self.assertEqual(self.downloaded(), [])
                self.assertNotIn("pacman -U", self.log())

    def test_a_release_without_the_package_installs_none(self):
        self.mac("j516s")
        self.pro_mesa = ""
        proc = self.install()
        self.assertNotIn("mesa", " ".join(self.downloaded()))
        self.assertEqual(self.mesa_lines(), [])
        self.assertFalse((self.state / "m3-pro-mesa").exists())
        self.assertNotIn("M3 Pro's Mesa", proc.stdout)


    def test_every_result_writes_one_line_per_key(self):
        # parse_record: the schema first, each scalar key once and in order, only the path keys
        # repeated; for every result, with and without paths to list.
        def setup_paths():
            (self.home / ".drirc").write_text('<option name="dri_driver" value="zink"/>')
            (self.home / ".config/environment.d").mkdir(parents=True, exist_ok=True)
            (self.home / ".config/environment.d/a.conf").write_text("VK_DRIVER_FILES=/x\n")
            (self.sysroot / "etc/mesa-m3").mkdir(parents=True, exist_ok=True)
            (self.sysroot / "etc/mesa-m3/disable").write_text("")
        runs = {
            "installed": dict(),
            "current": dict(before=lambda: self.owner_has(PRO_MESA_VERSION)),
            "newer-kept": dict(before=lambda: self.owner_has("27.0-1")),
            "skipped-flag": dict(env="M3_PRO_MESA=0"),
            "skipped-deps": dict(extra={"FAKE_GLIBC": "2.0-1"}),
            "failed": dict(extra={"FAKE_FAIL_U_FOR": "mesa-m3-*"}),
        }
        self.assertEqual(sorted(runs), sorted(RESULTS))
        for result, run in runs.items():
            for paths in (False, True):
                with self.subTest(result=result, paths=paths):
                    reset_mac(self, "j516s")
                    if paths:
                        setup_paths()
                    run.get("before", lambda: None)()
                    self.extra_env.update(run.get("extra", {}))
                    try:
                        self.install(env=run.get("env", ""), check=False)
                    finally:
                        for k in run.get("extra", {}):
                            self.extra_env.pop(k)
                    rec, lists = self.record_lists()
                    self.assertEqual(rec["result"], result)
                    self.assertEqual(len(lists["user_setup_path"]), 2 if paths else 0)
                    self.assertEqual(len(lists["opt_out_path"]), 1 if paths else 0)

    def test_the_record_is_bound_to_this_run(self):
        self.mac("j516s")
        before = datetime.now(timezone.utc).replace(microsecond=0)
        self.install()
        rec = self.record()
        self.assertEqual(rec["release"], TAG)
        written = datetime.strptime(rec["written_at"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
        self.assertLessEqual(abs((written - before).total_seconds()), 120)
        self.assertEqual(rec["boot_id"], Path("/proc/sys/kernel/random/boot_id").read_text().strip())
        self.assertEqual(rec["kernel"], os.uname().release)
        self.assertEqual(rec["board"], "apple,j516s")
        self.assertEqual(rec["installer_sha256"], hashlib.sha256(flow.INSTALLER.read_bytes()).hexdigest())
        self.assertEqual(rec["installer_source"], "file")
        # A new id for every run, the one the summary line names.
        proc = self.install()
        again = self.record()
        self.assertNotEqual(again["run_id"], rec["run_id"])
        self.assertEqual(summary_line(self, proc)["run_id"], again["run_id"])
        # Through a pipe the script can't be hashed again: no guess.
        self.install(env="SELF_SOURCE=stdin SELF_SHA256=unavailable")
        rec = self.record()
        self.assertEqual((rec["installer_sha256"], rec["installer_source"]), ("unavailable", "stdin"))

    def test_how_the_script_is_read_decides_its_hash(self):
        # The top of the script itself, run as the lab runs it (bash <file>) and as the one-liner
        # does (curl ... | bash): the source-only return is swapped for a print.
        src = SRC.replace('if [[ ${AURORA_SEP_SOURCE_ONLY:-} == 1 ]]; then return 0; fi',
                          'echo "$SELF_SOURCE $SELF_SHA256"; exit 0', 1)
        self.assertNotEqual(src, SRC)
        copy = self.tmp / "dir with space" / "install-aurora-sep.sh"
        copy.parent.mkdir()
        copy.write_text(src)
        sha = hashlib.sha256(src.encode()).hexdigest()
        env = {**os.environ, "AURORA_SEP_SOURCE_ONLY": "", "NO_COLOR": "1"}
        for how, cmd, stdin, want in (
                ("file", ["bash", str(copy)], None, f"file {sha}"),
                ("relative file", ["bash", copy.name], None, f"file {sha}"),
                ("pipe", ["bash"], src, "stdin unavailable"),
                ("pipe with arguments", ["bash", "-s", "--", "--read-only"], src, "stdin unavailable"),
                ("process substitution", ["bash", "-c", f"bash <(cat '{copy}')"], None, "stdin unavailable")):
            with self.subTest(how=how):
                out = subprocess.run(cmd, input=stdin, capture_output=True, text=True, env=env,
                                     cwd=copy.parent).stdout.strip()
                self.assertEqual(out, want)

    def test_the_record_is_written_through_sudo_and_renamed_into_place(self):
        self.mac("j516s")
        self.install(env=SUDO_LOG)
        rec = str(self.state / "m3-pro-mesa")
        tmp_rx = re.escape(str(self.state)) + r"/\.m3-pro-mesa\.[A-Za-z0-9]{6}"
        lines = [l for l in self.log().splitlines() if l.startswith("sudo ") and "m3-pro-mesa" in l]
        self.assertEqual(len(lines), 5, lines)
        self.assertRegex(lines[0], rf"^sudo mktemp {re.escape(str(self.state))}/\.m3-pro-mesa\.XXXXXX$")
        self.assertRegex(lines[1], rf"^sudo tee {tmp_rx}$")
        self.assertRegex(lines[2], rf"^sudo chmod 0644 {tmp_rx}$")
        self.assertRegex(lines[3], rf"^sudo sync {tmp_rx}$")
        self.assertRegex(lines[4], rf"^sudo mv -f {tmp_rx} {re.escape(rec)}$")
        self.assertEqual(lines[1].split()[-1], lines[4].split()[-2])
        log = self.log().splitlines()
        self.assertEqual(log[log.index(lines[4]) + 1], f"sudo sync {self.state}")
        self.assertNotIn("sudo rm", "\n".join(l for l in log if "m3-pro-mesa" in l))
        self.assertEqual([p.name for p in self.state.iterdir() if p.name.startswith(".m3-pro-mesa")], [])

    # Failures, injected with stubs of the commands the writer runs. FIRST fails only the first
    # write of the run (this run's record); ALWAYS fails the error record too.
    FIRST = '[[ ${1:-} == *"/.m3-pro-mesa."* && ! -e $FAKE/failed-once ]] && { touch "$FAKE/failed-once"; %s; }'
    ALWAYS = '[[ ${1:-} == *"/.m3-pro-mesa."* ]] && { %s; }'
    STUBS = {
        "temporary file": ("mktemp", "return 1"),
        "write": ("tee", 'cat >/dev/null; printf partial >"$1"; return 1'),
        "fsync": ("sync", "return 1"),
        "rename": ("mv", "return 1"),
    }

    def stub(self, name, when):
        cmd, body = self.STUBS[name]
        arg = '"${2:-}"' if cmd == "mv" else '"${1:-}"'
        test = (when % body).replace('${1:-}', arg[1:-1])
        (self.fake / "failed-once").unlink(missing_ok=True)
        return f'{cmd}() {{ {test}; command {cmd} "$@"; }}'

    def test_a_failure_before_the_rename_leaves_an_error_record(self):
        # This run's record fails before its rename, the error record works: the earlier record
        # is gone, an error record (record_error=write) is current, and the exit status is 4.
        for name in self.STUBS:
            with self.subTest(fails=name):
                reset_mac(self, "j516s")
                self.install()
                proc = self.install(env=self.stub(name, self.FIRST) + "\nM3_PRO_MESA=0", check=False)
                self.assertEqual(proc.returncode, 4, proc.stderr)
                rec, lists = self.record_lists()
                self.assertEqual((rec["result"], rec["record_error"]), ("record-error", "write"))
                self.assertEqual(lists["record_error_key"], [])
                self.assertEqual((rec["installed_by"], rec["preexisting"]), ("installer", "none"))
                err = " ".join(proc.stderr.split())
                self.assertEqual(len(record_warnings(proc)), 1, proc.stderr)
                self.assertEqual(err.count("did not write this run's record"), 1, proc.stderr)
                self.assertIn("An error record (result=record-error) replaces", err)
                self.assertEqual(summary_line(self, proc),
                                 {"run_id": rec["run_id"], "result": "record-error", "write": "error-record"})
                self.assertIn("The exit status is 4.", proc.stdout)
                self.assertEqual(self.temps(), [])
                self.assertEqual(self.stale(), [])

    def test_when_no_record_can_be_written_the_previous_one_is_moved_aside(self):
        for name in self.STUBS:
            with self.subTest(fails=name):
                reset_mac(self, "j516s")
                self.install()
                before = (self.state / "m3-pro-mesa").read_bytes()
                written = re.search(rb"^written_at=(\S+)$", before, re.M).group(1).decode()
                proc = self.install(env=self.stub(name, self.ALWAYS) + "\nM3_PRO_MESA=0", check=False)
                self.assertEqual(proc.returncode, 4, proc.stderr)
                self.assertFalse((self.state / "m3-pro-mesa").exists())
                self.assertEqual(self.stale(), [f"m3-pro-mesa.stale-{written}"])
                self.assertEqual((self.state / f"m3-pro-mesa.stale-{written}").read_bytes(), before)
                err = " ".join(proc.stderr.split())
                self.assertEqual(len(record_warnings(proc)), 1, proc.stderr)
                # Both failures, in the one warning.
                self.assertIn("this run's record: ", err)
                self.assertIn("; the error record: ", err)
                self.assertTrue(err.rstrip().endswith(f"The previous record is now {self.state}/m3-pro-mesa.stale-{written}, "
                                                      "so no record is current."), err)
                self.assertEqual(summary_line(self, proc)["write"], "none")
                self.assertEqual(summary_line(self, proc)["result"], "none")
                self.assertEqual(self.temps(), [])
        # With no earlier record: nothing to move, still a warning and 4.
        reset_mac(self, "j516s")
        proc = self.install(env=self.stub("rename", self.ALWAYS), check=False)
        self.assertEqual(proc.returncode, 4)
        self.assertIn("There is no record.", " ".join(proc.stderr.split()))
        self.assertEqual(os.listdir(self.state).count("m3-pro-mesa"), 0)

    def test_a_stale_record_is_never_overwritten(self):
        self.mac("j516s")
        self.install()
        before = (self.state / "m3-pro-mesa").read_bytes()
        written = re.search(rb"^written_at=(\S+)$", before, re.M).group(1).decode()
        taken = {f"m3-pro-mesa.stale-{written}": b"an earlier stale record\n",
                 f"m3-pro-mesa.stale-{written}.1": b"another\n"}
        for name, data in taken.items():
            (self.state / name).write_bytes(data)
        proc = self.install(env=self.stub("rename", self.ALWAYS) + "\nM3_PRO_MESA=0", check=False)
        self.assertEqual(proc.returncode, 4)
        for name, data in taken.items():
            self.assertEqual((self.state / name).read_bytes(), data)
        self.assertEqual((self.state / f"m3-pro-mesa.stale-{written}.2").read_bytes(), before)
        self.assertFalse((self.state / "m3-pro-mesa").exists())
        # The fsync after the move fails: still said.
        self.install()
        stub = self.stub("rename", self.ALWAYS) + '\nsync() { [[ ${1:-} == "$STATE" ]] && return 1; command sync "$@"; }'
        proc = self.install(env=stub + "\nM3_PRO_MESA=0", check=False)
        self.assertEqual(proc.returncode, 4)
        self.assertIn(f"But the fsync of {self.state} failed after the move.", " ".join(proc.stderr.split()))
        # And when the move itself fails: the earlier record stays, named as an earlier run's.
        self.install()
        before = (self.state / "m3-pro-mesa").read_bytes()
        stub = self.stub("rename", self.ALWAYS) + '\nmv() { [[ ${1:-} == -n ]] && return 1; ' \
            '[[ ${2:-} == *"/.m3-pro-mesa."* ]] && return 1; command mv "$@"; }'
        proc = self.install(env=stub + "\nM3_PRO_MESA=0", check=False)
        self.assertEqual(proc.returncode, 4)
        self.assertEqual((self.state / "m3-pro-mesa").read_bytes(), before)
        err = " ".join(proc.stderr.split())
        self.assertIn("could not move the previous record aside", err)
        self.assertIn("is an earlier run's record, not this run's", err)
        self.assertEqual(summary_line(self, proc)["write"], "stale")

    def test_a_failed_directory_fsync_after_the_rename_is_reported(self):
        # The new record is in place, but its durability is not confirmed: said, and exit 4.
        self.mac("j516s")
        self.install()
        dirsync = 'sync() { [[ ${1:-} == "$STATE" ]] && return 1; command sync "$@"; }'
        proc = self.install(env=dirsync + "\nM3_PRO_MESA=0", check=False)
        self.assertEqual(proc.returncode, 4, proc.stderr)
        rec = self.record()
        self.assertEqual(rec["result"], "skipped-flag")                 # this run's record, published
        err = " ".join(proc.stderr.split())
        self.assertIn(f"wrote {self.state}/m3-pro-mesa, but its durability is not confirmed: fsync of {self.state} "
                      "failed after the rename", err)
        self.assertEqual(summary_line(self, proc),
                         {"run_id": rec["run_id"], "result": "skipped-flag", "write": "unsynced"})
        self.assertIn("was written, but its durability is not confirmed", " ".join(proc.stdout.split()))
        # Distinct from a failure before the rename (test above): there no new record exists.

    def test_an_interrupted_write_keeps_the_previous_record(self):
        self.mac("j516s")
        self.install()
        before = (self.state / "m3-pro-mesa").read_bytes()
        # Killed after the new record is written to its temporary file, before the rename.
        self.extra_env["TMPDIR"] = str(self.tmp)
        proc = self.install(env='sync() { [[ ${1:-} == *"/.m3-pro-mesa."* ]] && kill -9 $$; command sync "$@"; }\n'
                                'M3_PRO_MESA=0', check=False)
        self.assertEqual(proc.returncode, -9)
        self.assertEqual((self.state / "m3-pro-mesa").read_bytes(), before)
        left = self.temps()
        self.assertEqual(len(left), 1)
        leftover = (self.state / left[0]).read_bytes()
        self.assertIn(b"result=skipped-flag", leftover)
        # The next run writes its own record and leaves the leftover alone, in one line.
        proc = self.install()
        self.assertEqual(self.temps(), left)
        self.assertEqual((self.state / left[0]).read_bytes(), leftover)
        self.assertIn("Left alone: 1 temporary record file(s) of another or an interrupted run", proc.stdout)
        self.assertEqual(self.record()["result"], "current")

    def test_another_runs_temporary_file_is_left_alone(self):
        self.mac("j516s")
        other = self.state / ".m3-pro-mesa.AbC123"
        other.write_bytes(b"schema=aurora.m3-pro-mesa-state/1\nanother writer, still writing\n")
        proc = self.install()
        self.assertEqual(other.read_bytes(), b"schema=aurora.m3-pro-mesa-state/1\nanother writer, still writing\n")
        self.assertEqual(self.temps(), [".m3-pro-mesa.AbC123"])
        self.assertEqual(self.record()["result"], "installed")
        self.assertEqual(proc.stdout.count("Left alone: 1 temporary record file(s)"), 1)
        # Also when this run's own write fails: only its own temporary file goes.
        proc = self.install(env=self.stub("rename", self.FIRST), check=False)
        self.assertEqual(self.temps(), [".m3-pro-mesa.AbC123"])

    def test_a_newline_or_carriage_return_in_any_value_gives_an_error_record(self):
        self.mac("j516s")
        self.install()
        # A file name with a newline: mesa-m3's detector refuses it (exit 2), so the record says
        # user_setup=unknown, never the name.
        bad_file = self.home / ".config/environment.d/a\nuser_setup=none.conf"
        bad_file.parent.mkdir(parents=True)
        bad_file.write_text("VK_ICD_FILENAMES=/x\n")
        proc = self.install(env="M3_PRO_MESA=0")
        data = (self.state / "m3-pro-mesa").read_bytes()
        rec, lists = parse_record(self, data)
        self.assertEqual((rec["result"], rec["user_setup"], lists["user_setup_path"]), ("skipped-flag", "unknown", []))
        self.assertNotIn(b"user_setup=none.conf", data)
        self.assertIn("the record says user_setup=unknown", " ".join(proc.stderr.split()))
        bad_file.unlink()
        # Any other value with a carriage return or a newline (here the user's name): an error record.
        self.extra_env["SUDO_USER"] = "own\rer"
        proc = self.install(env="M3_PRO_MESA=0", check=False)
        self.extra_env.pop("SUDO_USER")
        self.assertEqual(proc.returncode, 4)
        data = (self.state / "m3-pro-mesa").read_bytes()
        rec, lists = parse_record(self, data)
        self.assertEqual((rec["result"], rec["record_error"]), ("record-error", "value"))
        self.assertEqual(lists["record_error_key"], ["user"])
        self.assertEqual(rec["render_user"], "none")
        self.assertNotIn(b"own\r", data)                                 # never the refused value
        self.assertNotIn(b"\r", data)
        err = " ".join(proc.stderr.split())
        self.assertEqual(len(record_warnings(proc)), 1, proc.stderr)
        self.assertIn("a value holds a newline or a carriage return", err)
        self.assertIn("user=$'own\\rer'", err)
        # Refused, and the error record fails too: moved aside.
        self.extra_env["SUDO_USER"] = "own\rer"
        proc = self.install(env=self.stub("rename", self.ALWAYS) + "\nM3_PRO_MESA=0", check=False)
        self.extra_env.pop("SUDO_USER")
        self.assertEqual(proc.returncode, 4)
        self.assertFalse((self.state / "m3-pro-mesa").exists())
        self.assertEqual(len(self.stale()), 1)

    def test_after_an_error_record(self):
        # --uninstall keeps mesa-m3 and says why; the next install keeps the ownership history.
        self.mac("j516s")
        self.install()
        self.extra_env["SUDO_USER"] = "own\rer"
        self.install(env="M3_PRO_MESA=0", check=False)
        self.extra_env.pop("SUDO_USER")
        self.assertEqual(self.record()["result"], "record-error")
        since = len(self.log())
        proc = self.uninstall()
        self.assertEqual(self.mesa_lines(since), [])
        self.assertIn("the last install could not write its record (result=record-error)", " ".join(proc.stdout.split()))
        self.assertIn("mesa-m3", self.installed())
        # Again, with an error record in place: the next good run is this script's again.
        reset_mac(self, "j516s")
        self.install()
        self.extra_env["SUDO_USER"] = "own\rer"
        self.install(env="M3_PRO_MESA=0", check=False)
        self.extra_env.pop("SUDO_USER")
        proc = self.install()
        rec = self.record()
        self.assertEqual((rec["result"], rec["installed_by"], rec["preexisting"]), ("current", "installer", "none"))
        self.assertEqual(proc.returncode, 0)

    def test_a_record_of_another_schema_is_not_read(self):
        # Not this schema (an earlier draft, or anything else): never read, never trusted for
        # removal; the next install writes a new one.
        self.mac("j516s")
        self.install()
        rec = self.state / "m3-pro-mesa"
        body = rec.read_text().split("\n", 1)[1]
        rec.write_text(body)            # no schema line
        since = len(self.log())
        proc = self.uninstall()
        self.assertEqual(self.mesa_lines(since), [])
        self.assertIn("is not a record this script reads", " ".join(proc.stderr.split()))
        self.assertIn("mesa-m3", self.installed())
        reset_mac(self, "j516s")
        self.owner_has(PRO_MESA_VERSION)
        rec.write_text("schema=aurora.m3-pro-mesa-state/2\ninstalled_by=installer\npreexisting=none\n"
                       f"installed_version={PRO_MESA_VERSION}\n")
        self.install()
        new = self.record()
        self.assertEqual((new["installed_by"], new["preexisting"]), ("owner", PRO_MESA_VERSION))

    def test_a_version_changed_outside_this_script_is_the_owners(self):
        self.mac("j516s")
        self.install()
        self.owner_has("26.1.4.mine-1")     # replaced outside this script: no longer this script's
        self.install(env="M3_PRO_MESA=0")
        self.assertEqual(self.record()["installed_by"], "owner")
        since = len(self.log())
        self.uninstall()
        self.assertEqual(self.mesa_lines(since), [])


    def detector(self, body):
        path = self.tmp / "opt/mesa-m3/libexec/mesa-m3-user-setup"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("#!/bin/bash\n" + 'echo "$*" >"$FAKE/detector-args"\n' + body)
        path.chmod(0o755)

    # mesa-m3-user-setup's answers (its report, section 4.5): schema 2; exit 0 none, 1 present,
    # 3 unknown.
    PRESENT = ("echo schema=aurora.mesa-m3-user-setup/3; echo session_desktops=hyprland; echo user_setup=present\n"
               "echo 'user_setup_path=/a path/from the detector'\n"
               "echo 'user_setup_path=/b'\n"
               "echo 'user_setup_detail=/a path/from the detector:3 sets GALLIUM_DRIVER'\n"
               "echo 'user_setup_detail=/b chooses a driver (dri_driver)'\n"
               "echo 'user_setup_finding=applied desktop:hyprland sure /a path/from the detector:3 sets GALLIUM_DRIVER'\n"
               "echo 'user_setup_finding=ignored desktop:chonkstep sure /c/m3gpu-session.env sets CHONKSTEP_M3_MESA_PREFIX to /p'\n"
               "exit 1\n")
    NONE = "echo schema=aurora.mesa-m3-user-setup/3; echo session_desktops=hyprland; echo user_setup=none; exit 0\n"
    UNKNOWN = ("echo schema=aurora.mesa-m3-user-setup/3; echo session_desktops=hyprland; echo user_setup=unknown\n"
               "echo user_setup_path=/c; echo 'user_setup_detail=/c cannot be read (Permission denied)'; exit 3\n")

    def test_the_user_setup_comes_from_the_packages_detector_once_installed(self):
        self.mac("j516s")
        self.detector(self.PRESENT)
        (self.home / ".drirc").write_text('<option name="dri_driver" value="zink"/>')   # the built-in check's
        proc = self.install()
        rec, lists = self.record_lists()
        self.assertEqual((rec["user_setup_source"], rec["user_setup"]), ("package-detector", "present"))
        self.assertEqual(lists["user_setup_path"], ["/a path/from the detector", "/b"])
        self.assertEqual(lists["user_setup_ignored"],
                         ["desktop:chonkstep sure /c/m3gpu-session.env sets CHONKSTEP_M3_MESA_PREFIX to /p"])
        self.assertEqual((self.fake / "detector-args").read_text().strip(),
                         f"--home {self.home_path} --session-desktops Hyprland --root {self.sysroot}")
        out = " ".join(proc.stdout.split())
        self.assertIn("(/a path/from the detector:3 sets GALLIUM_DRIVER; /b chooses a driver (dri_driver))", out)
        for body, want in ((self.NONE, ("none", [])), (self.UNKNOWN, ("unknown", ["/c"]))):
            self.detector(body)
            self.install()
            rec, lists = self.record_lists()
            self.assertEqual((rec["user_setup_source"], rec["user_setup"], lists["user_setup_path"]),
                             ("package-detector",) + want)

    def test_the_built_in_copy_without_mesa_m3_or_its_detector(self):
        # Not installed (--no-m3-mesa), or the installed detector fails: this script's copy.
        self.mac("j516s")
        (self.home / ".drirc").write_text('<option name="dri_driver" value="zink"/>')
        self.detector(self.NONE)
        self.install(env="M3_PRO_MESA=0")
        self.assertFalse((self.fake / "detector-args").exists())
        rec, lists = self.record_lists()
        self.assertEqual((rec["user_setup_source"], rec["user_setup"], lists["user_setup_path"]),
                         ("installer-builtin", "present", [f"{self.home_path}/.drirc"]))
        for body in ("exit 2\n",                                                  # an error
                     "echo schema=other/1; exit 0\n",                             # another form
                     "echo schema=aurora.mesa-m3-user-setup/3; echo session_desktops=x; echo user_setup=present; exit 0\n",
                     "echo schema=aurora.mesa-m3-user-setup/3; echo user_setup=none; exit 0\n",  # no session line
                     "echo schema=aurora.mesa-m3-user-setup/2; echo user_setup=none; exit 0\n",  # the old schema
                     self.NONE.replace("exit 0", "exit 1")):
            with self.subTest(detector=body):
                self.detector(body)
                proc = self.install()
                rec, lists = self.record_lists()
                self.assertEqual((rec["user_setup_source"], lists["user_setup_path"]),
                                 ("installer-builtin", [f"{self.home_path}/.drirc"]))
                self.assertIn("mesa-m3-user-setup failed", " ".join(proc.stderr.split()))
        # And when the copy cannot run either: unknown, never "none".
        for name, stub in (("no python3", "python3() { return 127; }"),
                           ("not the release's bytes", "M3_PRO_MESA_DETECTOR_SHA256=" + "0" * 64)):
            with self.subTest(copy=name):
                proc = self.install(env=stub + "\nM3_PRO_MESA=0")
                rec, lists = self.record_lists()
                self.assertEqual((rec["user_setup"], lists["user_setup_path"]), ("unknown", []))
                self.assertIn("copy of mesa-m3's user-setup check failed", " ".join(proc.stderr.split()))

    def builtin(self):
        """The built-in copy's answer for the harness's home: (exit status, [(path, detail)]) of the
        findings a Hyprland session uses."""
        proc = self.run_sh(f"rc=0; m3_pro_mesa_builtin_setup '{self.home_path}' || rc=$?; printf 'rc=%s' \"$rc\"")
        out, _, rc = proc.stdout.rpartition("rc=")
        out = out.split("\0")
        self.assertEqual(out[-1], "")
        triples = list(zip(out[:-1:3], out[1:-1:3], out[2:-1:3]))
        self.assertEqual(triples[0][:2], ("session", ""))
        return int(rc), [(p, w) for k, p, w in triples if k == "applied"]

    def test_the_built_in_copy_is_the_releases_bytes(self):
        # The heredocs print the files whose sha256 the release names (fill-m3-pro-mesa.sh writes
        # both from the package), and the copy answers like the detector.
        for fn, var in (("m3_pro_mesa_builtin_detector", "M3_PRO_MESA_DETECTOR_SHA256"),
                        ("m3_pro_mesa_builtin_list", "M3_PRO_MESA_SETUP_LIST_SHA256")):
            out = subprocess.run(["bash", "-c", f"AURORA_SEP_SOURCE_ONLY=1 source '{flow.INSTALLER}'; {fn}"],
                                 capture_output=True, check=True).stdout
            self.assertEqual(hashlib.sha256(out).hexdigest(), re.search(rf"^{var}=(\S+)$", SRC, re.M).group(1))
        self.assertEqual(self.builtin(), (0, []))
        (self.home / ".drirc").write_text('<option name="dri_driver" value="zink"/>')
        self.assertEqual(self.builtin(), (1, [(f"{self.home_path}/.drirc", " chooses a driver (dri_driver)")]))
        (self.home / ".drirc").chmod(0)
        self.addCleanup((self.home / ".drirc").chmod, 0o644)
        rc, found = self.builtin()
        self.assertEqual((rc, [p for p, _ in found]), (3, [f"{self.home_path}/.drirc"]))
        self.assertEqual(self.run_sh("rc=0; m3_pro_mesa_builtin_setup '/no such home' || rc=$?; echo $rc").stdout,
                         "session\0\0hyprland\0" + "0\n")

    def test_u1s_fixtures_pass_on_the_built_in_copy(self):
        # mesa-m3's own detector fixtures (42 cases, exact output and exit status) on this
        # script's copy, as the package build runs them on the package's.
        fixtures = flow.INSTALLER.parent / "recipes/mesa-m3/tests/test-user-setup.sh"
        if not fixtures.is_file() or os.geteuid() == 0:
            self.skipTest("in-tree Mesa user-setup fixtures and a non-root test user are required")
        det, lst = self.tmp / "mesa-m3-user-setup", self.tmp / "user-setup.list"
        for fn, path in (("m3_pro_mesa_builtin_detector", det), ("m3_pro_mesa_builtin_list", lst)):
            path.write_bytes(subprocess.run(["bash", "-c", f"AURORA_SEP_SOURCE_ONLY=1 source '{flow.INSTALLER}'; {fn}"],
                                            capture_output=True, check=True).stdout)
        det.chmod(0o755)
        proc = subprocess.run(["sh", str(fixtures), str(det), str(lst)], capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stdout[-3000:] + proc.stderr)
        self.assertNotIn("FAIL", proc.stdout)
        self.assertGreaterEqual(proc.stdout.count("\nok ") + proc.stdout.startswith("ok "), 40)

    # The render group (mesa-m3's udev rule gives the M3 Pro's render node to it).

    def groups(self):
        try:
            return (self.fake / "groups").read_text()
        except FileNotFoundError:
            return ""

    def me(self):
        return subprocess.run(["id", "-un"], capture_output=True, text=True).stdout.strip()

    def test_the_desktop_user_is_added_to_render(self):
        self.mac("j516s")
        proc = self.install()
        me = self.me()
        self.assertIn(f"gpasswd -a {me} render", self.log().splitlines())
        self.assertIn(f"{me} render", self.groups())
        rec = self.record()
        self.assertEqual((rec["render_member"], rec["render_preexisting"], rec["render_added"],
                          rec["render_by_installer"]), ("yes", "no", "yes", "yes"))
        out = " ".join(proc.stdout.split())
        self.assertIn(f"Added {me} to the render group, which the M3 Pro's GPU now needs. That takes effect at the "
                      "next login: the reboot does that.", out)
        # A rerun adds nothing and keeps the history.
        since = len(self.log())
        proc = self.install()
        self.assertNotIn("gpasswd", self.log()[since:])
        rec = self.record()
        self.assertEqual((rec["render_member"], rec["render_preexisting"], rec["render_added"],
                          rec["render_by_installer"]), ("yes", "no", "no", "yes"))
        self.assertNotIn("Added", proc.stdout)
        # --uninstall takes them out again.
        since = len(self.log())
        proc = self.uninstall()
        self.assertIn(f"gpasswd -d {me} render", self.log()[since:].splitlines())
        self.assertNotIn("render", self.groups())
        self.assertIn(f"Took {me} out of the render group, which this script had added them to", proc.stdout)

    def test_a_member_already_stays_a_member(self):
        self.mac("j516s")
        me = self.me()
        (self.fake / "groups").write_text(f"{me} wheel render\n")
        proc = self.install()
        self.assertNotIn("gpasswd", self.log())
        rec = self.record()
        self.assertEqual((rec["render_member"], rec["render_preexisting"], rec["render_added"],
                          rec["render_by_installer"]), ("yes", "yes", "no", "no"))
        self.assertNotIn("render group,", proc.stdout)
        self.uninstall()
        self.assertNotIn("gpasswd", self.log())
        self.assertEqual(self.groups(), f"{me} wheel render\n")

    def test_uninstall_takes_out_only_what_this_script_added(self):
        me = self.me()
        for case in ("taken out by hand since", "error record", "another user's run"):
            with self.subTest(case):
                reset_mac(self, "j516s")
                self.install()
                if case == "taken out by hand since":
                    (self.fake / "groups").write_text(f"{me} wheel\n")
                elif case == "error record":
                    self.extra_env["SUDO_USER"] = "own\rer"
                    self.install(env="M3_PRO_MESA=0", check=False)
                    self.extra_env.pop("SUDO_USER")
                    self.assertEqual(self.record_lists()[1]["render_added_user"], [me])   # kept in the error record
                else:
                    self.extra_env["SUDO_USER"] = "other"
                    self.install()
                    self.extra_env.pop("SUDO_USER")
                    rec, lists = self.record_lists()
                    self.assertEqual((rec["user"], lists["render_added_user"]), ("other", [me, "other"]))
                before = self.groups()
                since = len(self.log())
                proc = self.uninstall()
                if case == "another user's run":
                    # Both were added by this script: both come out, and nobody else.
                    self.assertEqual(sorted(l for l in self.log()[since:].splitlines() if l.startswith("gpasswd")),
                                     sorted([f"gpasswd -d {me} render", "gpasswd -d other render"]))
                    self.assertNotIn("render", self.groups())
                else:
                    self.assertNotIn("gpasswd", self.log()[since:])
                    self.assertEqual(self.groups(), before)
                if case == "error record":
                    self.assertIn("Keeping " + me + " in the render group", proc.stdout)

    def test_render_stays_while_mesa_m3_stays(self):
        # The package's udev rule keeps the render node for group render as long as mesa-m3 is
        # installed: --uninstall takes nobody out of render unless its own pacman -Rn removed it.
        me = self.me()
        for case in ("the owner replaced it", "installed before this script", "error record",
                     "pacman -Rn fails", "removed by hand before"):
            with self.subTest(case):
                reset_mac(self, "j516s")
                if case == "installed before this script":
                    self.owner_has("26.0.0-1")
                self.install()
                self.assertIn(f"{me} render", self.groups())
                body = "uninstall_all"
                if case == "the owner replaced it":
                    self.owner_has("26.1.4.mine-1")
                elif case == "error record":
                    self.extra_env["SUDO_USER"] = "own\rer"
                    self.install(env="M3_PRO_MESA=0", check=False)
                    self.extra_env.pop("SUDO_USER")
                elif case == "pacman -Rn fails":
                    body = 'pacman() { [[ $1 == -Rn ]] && return 1; command pacman "$@"; }\nuninstall_all'
                elif case == "removed by hand before":
                    (self.fake / "installed").write_text(
                        "".join(l + "\n" for l in self.installed() if l != "mesa-m3"))
                since = len(self.log())
                proc = self.run_sh(body)
                self.assertNotIn("gpasswd", self.log()[since:])
                self.assertIn(f"{me} render", self.groups())
                self.assertEqual("mesa-m3" in self.installed(), case != "removed by hand before")
                self.assertIn(f"Keeping {me} in the render group: this run did not remove mesa-m3",
                              " ".join(proc.stdout.split()))

    def test_a_root_run_adds_no_one(self):
        self.mac("j516s")
        proc = self.install(env="m3_pro_mesa_user() { echo root; }")
        self.assertNotIn("gpasswd", self.log())
        rec = self.record()
        self.assertEqual((rec["user"], rec["render_added"], rec["render_by_installer"]), ("root", "no", "no"))
        self.assertIn("This ran as root, so no desktop user was added to the render group the M3 Pro's GPU now "
                      "needs: add yours with sudo gpasswd -a <user> render and log in again.", " ".join(proc.stdout.split()))
        self.assertEqual(proc.returncode, 0)

    def test_render_group_failures(self):
        me = self.me()
        for case, env, extra, setup in (
                ("gpasswd fails", "", {"FAKE_FAIL_GPASSWD": "1"}, None),
                ("no render group", "", {}, lambda: (self.fake / "no-render-group").write_text("")),
                ("not a user name", "", {"SUDO_USER": "Not A User"}, None)):
            with self.subTest(case):
                reset_mac(self, "j516s")
                if setup:
                    setup()
                self.extra_env.update(extra)
                try:
                    proc = self.install(env=env, check=False)
                finally:
                    for k in extra:
                        self.extra_env.pop(k)
                self.assertEqual(proc.returncode, 3, proc.stderr)
                rec = self.record()
                self.assertEqual((rec["result"], rec["render_added"], rec["render_by_installer"]),
                                 ("installed", "failed", "no"))
                self.assertIn("render group", " ".join(proc.stderr.split()))
                self.assertIn("The exit status is 3.", " ".join(proc.stdout.split()))
                self.assertNotIn(f"{me} render", self.groups())

    def test_no_render_change_without_the_mesa(self):
        for case, env, extra in (("--no-m3-mesa", "M3_PRO_MESA=0", {}),
                                 ("dependencies too old", "", {"FAKE_GLIBC": "2.0-1"}),
                                 ("pacman fails", "", {"FAKE_FAIL_U_FOR": "mesa-m3-*"})):
            with self.subTest(case):
                reset_mac(self, "j516s")
                self.extra_env.update(extra)
                try:
                    self.install(env=env, check=False)
                finally:
                    for k in extra:
                        self.extra_env.pop(k)
                self.assertNotIn("gpasswd", self.log())
                rec = self.record()
                self.assertEqual((rec["render_added"], rec["render_member"]), ("no", "no"))

    def test_the_recovery_text(self):
        self.mac("j516s")
        proc = self.install()
        out = proc.stdout
        start = out.index("   Each login records whether the GPU graphics are on and why:")
        block = out[start:out.index("(the wheel or adm group).", start)]
        for text in ("cat /run/user/$(id -u)/mesa-m3-session.state", "journalctl -b -t mesa-m3",
                     "mkdir -p ~/.config/mesa-m3 && touch ~/.config/mesa-m3/disable", "mesa_m3=off",
                     "rm ~/.config/mesa-m3/disable", "Ctrl+Alt+F3", "render group"):
            self.assertIn(text, block)
        self.assertIn("   If a login with GPU graphics did not reach a working desktop, the next logins of that\n"
                      "   boot render in software (previous-failed). To try the GPU again:\n"
                      "     rm ~/.local/state/mesa-m3/attempt\n"
                      "   then log out and in (or reboot).\n", block)
        self.assertNotIn("mesa-m3-attempt", out)
        words = " ".join(block.split())
        self.assertIn("Reasons: " + ", ".join(REASONS) + ".", words)
        self.assertLessEqual(len(block.splitlines()), 15)
        self.assertTrue(all(len(l) <= 105 for l in block.splitlines()))
        # Not when mesa-m3 is not installed.
        reset_mac(self, "j516s")
        proc = self.install(env="M3_PRO_MESA=0")
        self.assertNotIn("Each login records", proc.stdout)


def pro_mesa_package():
    """This release's mesa-m3 package, when at hand: AURORA_PRO_MESA_PKG, the release staging
    directory or its build directory."""
    m = re.search(r'^M3_PRO_MESA_PACKAGE="(\S+) (\S+)"$', SRC, re.M)
    if not m or "PENDING" in m.group(0):
        return None
    name, sha = m.groups()
    recipes = Path.home() / "source/aurora-recipes"
    for c in (os.environ.get("AURORA_PRO_MESA_PKG", ""), recipes / f"stage-{VERSION.split('-')[-1]}" / name,
              recipes / "builds" / name.removesuffix("-aarch64.pkg.tar.zst") / name):
        c = Path(c) if c else None
        if c and c.is_file() and c.name == name and hashlib.sha256(c.read_bytes()).hexdigest() == sha:
            return c
    return None


@unittest.skipUnless(pro_mesa_package() and shutil.which("python3") and shutil.which("bsdtar"),
                     "this release's mesa-m3 package (AURORA_PRO_MESA_PKG), python3 and bsdtar are needed")
class DetectorAgreementTest(flow.M3FlowBase):
    """mesa-m3's own detector (mesa-m3-user-setup, from the package, with its own list) and this
    script's built-in copy, as the installer runs it, on the same homes: the same answer, the same
    paths in the same order. The session hook yields on the detector's answer; the record before
    mesa-m3 is installed comes from the built-in copy."""

    def setUp(self):
        super().setUp()
        pkg = pro_mesa_package()
        detector = re.search(r'^M3_PRO_MESA_DETECTOR="([^"]*)"$', SRC, re.M).group(1)
        listed = re.search(r'^M3_PRO_MESA_SETUP_LIST="([^"]*)"$', SRC, re.M).group(1)
        self.pkgroot = self.tmp / "pkgroot"
        self.pkgroot.mkdir()
        subprocess.run(["bsdtar", "-xf", str(pkg), "-C", str(self.pkgroot), detector.lstrip("/"), listed.lstrip("/")],
                       check=True)
        self.detector_bin = self.pkgroot / detector.lstrip("/")

    def answers(self):
        """((exit status, detector paths), (exit status, built-in paths))."""
        proc = subprocess.run(["python3", str(self.detector_bin), "--home", self.home_path, "--root", str(self.sysroot),
                               "--session-desktops", "Hyprland"], capture_output=True, text=True)
        self.assertIn(proc.returncode, (0, 1, 3), proc.stderr)
        lines = proc.stdout.splitlines()
        self.assertEqual(lines[:2], ["schema=aurora.mesa-m3-user-setup/3", "session_desktops=hyprland"])
        theirs = [l.split("=", 1)[1] for l in lines if l.startswith("user_setup_path=")]
        ignored = [l[len("user_setup_finding=ignored "):] for l in lines if l.startswith("user_setup_finding=ignored ")]
        self.assertEqual(lines[2], "user_setup=" + {0: "none", 1: "present", 3: "unknown"}[proc.returncode])
        # As the installer collects it, through its own call of its copy.
        body = (f"m3_pro_mesa_user_home() {{ echo '{self.home_path}'; }}\nm3_pro_mesa_installed() {{ :; }}\n"
                "m3_pro_mesa_setup_collect\nprintf '%s\\n' \"$M3_PRO_MESA_SETUP_ANSWER\" \"$M3_PRO_MESA_SETUP_SESSION\"\n"
                "printf 'path=%s\\n' \"${M3_PRO_MESA_SETUP[@]}\" | sed -n '1~2p'\n"
                "printf 'ignored=%s\\n' \"${M3_PRO_MESA_SETUP_IGNORED[@]}\"\n")
        got = self.run_sh(body).stdout.splitlines()
        answer, session = got[0], got[1]
        ours = [l[5:] for l in got[2:] if l.startswith("path=") and l != "path="]
        ours_ignored = [l[8:] for l in got[2:] if l.startswith("ignored=") and l != "ignored="]
        self.assertEqual(session, "hyprland")
        rc = {"none": 0, "present": 1, "unknown": 3}[answer]
        return (proc.returncode, theirs, ignored), (rc, ours, ours_ignored)

    def write(self, files):
        for rel, data in files.items():
            path = self.sysroot / rel.lstrip("/")
            path.parent.mkdir(parents=True, exist_ok=True)
            if data is None:
                path.mkdir(exist_ok=True)
            else:
                path.write_text(data)

    def scenarios(self):
        h, c = self.home_path, f"{self.home_path}/.config"
        variables = [l.split("\t")[1] for l in (self.pkgroot / "opt/mesa-m3/share/mesa-m3/user-setup.list").read_text().splitlines()
                     if l.startswith(("variable\t", "flag\t"))]
        self.assertEqual(len(variables), 12)
        yield "empty home", {}
        yield "the lab m3pro today", {f"{c}/chonkstep/m3gpu-session.env": CHONKSTEP_ENV.decode(),
                                      f"{c}/hypr/hyprland.conf": "monitor=,preferred,auto,1\n"}
        yield "a uwsm env file for Hyprland", {f"{c}/uwsm/env-hyprland": "export GALLIUM_DRIVER=zink\n"}
        yield "a uwsm env file for another desktop", {f"{c}/uwsm/env-sway": "export GALLIUM_DRIVER=zink\n",
                                                      f"{c}/uwsm/env-chonkstep.d/x": "export VK_ICD_FILENAMES=/x\n"}
        yield "CHONKSTEP_M3_MESA_PREFIX in a common file", {f"{c}/uwsm/env": "export CHONKSTEP_M3_MESA_PREFIX=/x/p\n"}
        yield "the m3pro after the switch-over", {f"{c}/chonkstep/m3gpu-session.env":
                                                  "CHONKSTEP_M3_CLIENTS=gpu\nCHONKSTEP_M3_MESA_PREFIX=/opt/mesa-m3\n"}
        yield "chonkstep without a prefix", {f"{c}/chonkstep/m3gpu-session.env": "CHONKSTEP_M3_CLIENTS=gpu\n"}
        yield "chonkstep quoted, commented, CRLF", {f"{c}/chonkstep/m3gpu-session.env":
                                                    'export CHONKSTEP_M3_MESA_PREFIX="/x/p" # mine\r\n'}
        for v in variables:
            yield f"{v} elsewhere", {f"{c}/environment.d/{v}.conf": f"{v}=/somewhere/else\n"}
            yield f"{v} in the prefix", {f"{c}/environment.d/{v}.conf": f"{v}=/opt/mesa-m3/lib/x\n",
                                         f"{c}/uwsm/env": f'export {v}="/opt/mesa-m3"\n'}
            yield f"{v} commented", {f"{c}/hypr/a.conf": f"# env = {v},/x\n", f"{c}/hypr/b.lua": f"-- {v}=/x\n"}
            yield f"{v} as part of a longer name", {f"{c}/uwsm/env-hyprland": f"MY_{v}_X=/x\n"}
            yield f"{v} set empty", {f"{c}/uwsm/env": f"export {v}=\n"}
        # Dave's three counterexamples.
        yield "d1 a comment naming the prefix", {f"{c}/uwsm/env": "export LIBGL_ALWAYS_SOFTWARE=1 # /opt/mesa-m3/\n"}
        yield "d2 a list with a private entry", {
            f"{c}/uwsm/env": "VK_DRIVER_FILES=/opt/mesa-m3/share/vulkan/icd.d/asahi_icd.aarch64.json:/opt/private/icd.json\n"}
        yield "d3 an unreadable file", {f"{c}/uwsm/env": "UNREADABLE"}
        yield "d3 an unlistable directory", {f"{c}/hypr/conf.d": "UNLISTABLE"}
        # Normalised paths, quoting, a later LD_LIBRARY_PATH, Hyprland subdirectories.
        yield "a path out of the prefix by ..", {f"{c}/uwsm/env": "export GBM_BACKENDS_PATH=/opt/mesa-m3/../x\n"}
        yield "a quoted ; and a second assignment", {f"{c}/uwsm/env": "A=1; export GALLIUM_DRIVER='zink' # x\n"}
        yield "LD_LIBRARY_PATH replaced after the hook", {f"{c}/uwsm/env": "export LD_LIBRARY_PATH=/usr/lib\n"}
        yield "LD_LIBRARY_PATH kept after the hook", {f"{c}/uwsm/env": 'export LD_LIBRARY_PATH="/x:$LD_LIBRARY_PATH"\n'}
        yield "LD_LIBRARY_PATH replaced before the hook", {f"{c}/environment.d/a.conf": "LD_LIBRARY_PATH=/usr/lib\n"}
        yield "Hyprland subdirectories", {f"{c}/hypr/conf.d/deep/envs.conf": "env = GBM_BACKENDS_PATH,/home/u/gbm\n",
                                          f"{c}/hypr/lua/x.lua": 'env("LIBGL_DRIVERS_PATH", "/x")\n'}
        yield "a Hyprland lua value that is not a literal", {f"{c}/hypr/x.lua": 'env("GALLIUM_DRIVER", os.getenv("X"))\n'}
        yield "Hyprland env lines", {f"{c}/hypr/envs.conf": "env = GBM_BACKENDS_PATH,/home/u/gbm\n",
                                     f"{c}/hypr/envs.lua": 'env("LIBGL_DRIVERS_PATH", "/x")\n'}
        yield "LD_LIBRARY_PATH", {f"{h}/mesa/lib/libgallium-26.so": "", f"{h}/plain/lib/libfoo.so": "",
                                  "/opt/mesa-m3/lib/libgallium-26.so": "", "/usr/local/m/libEGL_mesa.so.0": "",
                                  f"{h}/drm/dri/zink_dri.so": "",
                                  f"{c}/environment.d/1.conf": "LD_LIBRARY_PATH=$HOME/plain/lib:/opt/mesa-m3/lib\n",
                                  f"{c}/environment.d/2.conf": "LD_LIBRARY_PATH=${HOME}/mesa/lib\n",
                                  f"{c}/uwsm/env-x.d/b": 'LD_LIBRARY_PATH="/usr/local/m:$LD_LIBRARY_PATH"\n',
                                  f"{c}/hypr/c.conf": "env = LD_LIBRARY_PATH,~/drm\n",
                                  f"{c}/hypr/d.conf": "# LD_LIBRARY_PATH=~/mesa/lib\n"}
        yield "system files", {"/etc/xdg/uwsm/env": "export VK_ICD_FILENAMES=/x.json\n",
                               "/etc/xdg/uwsm/env-hyprland": "export GALLIUM_DRIVER=zink\n",
                               "/etc/xdg/uwsm/env.d/z": "MESA_LOADER_DRIVER_OVERRIDE=zink\n",
                               "/usr/local/share/uwsm/env": "export GBM_ALWAYS_SOFTWARE=1\n",
                               "/usr/share/uwsm/env-hyprland": "export DRIRC_CONFIGDIR=/x\n",
                               "/usr/share/uwsm/env.d/60-later": "export VK_ADD_DRIVER_FILES=/x.json\n",
                               "/usr/share/uwsm/env.d/40-earlier": "export VK_ADD_DRIVER_FILES=/y.json\n",
                               "/etc/environment": "LIBGL_ALWAYS_SOFTWARE=1\n",
                               "/etc/drirc": '<option name="dri_driver" value="zink"/>'}
        yield "drirc", {f"{h}/.drirc": '<option name = "dri_driver" value="zink"/>', f"{c}/drirc": "<driconf/>"}
        yield "drirc in an XML comment", {f"{h}/.drirc": '<!-- <option name="dri_driver" value="zink"/> -->'}
        yield "everything at once", {f"{c}/environment.d/1.conf": "VK_DRIVER_FILES=/x\n",
                                     f"{c}/uwsm/default": "GBM_ALWAYS_SOFTWARE=1\n",
                                     f"{c}/hypr/x.conf": "env = DRIRC_CONFIGDIR,/x\n",
                                     f"{h}/.drirc": '<option name="dri_driver" value="zink"/>',
                                     f"{c}/chonkstep/m3gpu-session.env": "CHONKSTEP_M3_MESA_PREFIX=/home/u/p\n"}

    def test_the_detector_and_the_built_in_copy_agree(self):
        if os.geteuid() == 0:
            self.skipTest("unreadable files need a normal user")
        n, kinds, ignored = 0, set(), 0
        for name, files in self.scenarios():
            with self.subTest(name):
                for p in self.sysroot.rglob("*"):
                    p.chmod(0o755 if p.is_dir() else 0o644)
                shutil.rmtree(self.sysroot)
                self.home.mkdir(parents=True)
                special = {k: v for k, v in files.items() if v in ("UNREADABLE", "UNLISTABLE")}
                self.write({k: ("x\n" if v == "UNREADABLE" else None if v == "UNLISTABLE" else v)
                            for k, v in files.items()})
                for rel in special:
                    (self.sysroot / rel.lstrip("/")).chmod(0)
                theirs, ours = self.answers()
                self.assertEqual(ours, theirs)
                n += 1
                kinds.add(theirs[0])
                ignored += bool(theirs[2])
        for p in self.sysroot.rglob("*"):
            p.chmod(0o755 if p.is_dir() else 0o644)
        self.assertGreaterEqual(n, 70)
        self.assertEqual(kinds, {0, 1, 3})                   # none, present and unknown all covered
        self.assertGreater(ignored, 3)                        # and findings the session does not use


class FromEarlierReleasesTest(flow.M3FlowBase):
    """12.0's, 12.1's or 12.2's own script, then this one with no new option: an M3 Pro gets its
    Mesa, and an M1 still gets none."""

    REVS = (("df0c4330", "12.0"), ("1d41e1b7", "12.1"), (REL_12_2, "12.2"))

    def test_an_m3_pro_gets_it_on_a_plain_rerun(self):
        for rev, name in self.REVS:
            with self.subTest(release=name):
                old, _ = old_installer(self, rev, f"install-{name}.sh")
                reset_mac(self, "j516s")
                self.installer = old
                try:
                    self.install()
                finally:
                    self.installer = flow.INSTALLER
                self.assertNotIn(PRO_MESA, self.downloaded())
                self.assertFalse((self.state / "m3-pro-mesa").exists())
                proc = self.install()
                self.assertIn(PRO_MESA, self.downloaded())
                self.assertIn("mesa-m3", (self.fake / "installed").read_text().split())
                rec = dict(l.split("=", 1) for l in (self.state / "m3-pro-mesa").read_text().splitlines())
                self.assertEqual((rec["result"], rec["preexisting"]), ("installed", "none"))
                self.assertIn("take effect at the next login", " ".join(proc.stdout.split()))

    def test_an_m1_still_gets_none(self):
        for rev, name in self.REVS:
            with self.subTest(release=name):
                old, _ = old_installer(self, rev, f"install-{name}.sh")
                reset_mac(self, "j314s")
                self.installer = old
                try:
                    self.install()
                finally:
                    self.installer = flow.INSTALLER
                self.install()
                self.assertNotIn("mesa", self.log())
                self.assertFalse((self.state / "m3-pro-mesa").exists())


class OtherMacsTest(flow.M3FlowBase):
    """Every Mac but the M3 Pro, with this script and with 12.2's: the same commands, in the same
    order, with the same results on disk."""

    # Not the J613 either: from this release on it gets m1n1's display handoff by default
    # (test_m3_air_default compares everything else with 12.3).
    # Not the M3 Pro, nor the M3 Airs, which get mesa-m3 from 12.4 on (test_m3_air_mesa).
    BOARDS = [b for b, compat in flow.BOARDS.items() if "apple,t6030" not in compat and b not in AIRS]

    def setUp(self):
        super().setUp()
        self.old, self.old_version = old_installer(self, REL_12_2, "install-12.2.sh")

    def same(self, run, setup=None, boards=None):
        for board in boards or self.BOARDS:
            with self.subTest(board=board):
                before = run_with(self, self.old, board, run, setup)
                after = run_with(self, flow.INSTALLER, board, run, setup)
                before = as_this_release(before, self.old_version, VERSION)
                self.assertEqual(after["codes"], before["codes"])
                same_commands(self, before["log"], after["log"])
                self.assertEqual(sorted(after["tree"]), sorted(before["tree"]))
                for path, data in before["tree"].items():
                    self.assertEqual(after["tree"][path], data, path)
                # Nothing of the M3 Pro's Mesa, by name or on disk.
                self.assertNotIn("state/m3-pro-mesa", after["tree"])
                self.assertNotIn(PRO_MESA, after["log"])
                self.assertNotIn("pacman -Q mesa-m3\n", after["log"] + "\n")
                yield board, after

    def install_and_uninstall(self):
        a = self.run_sh(SUDO_LOG + "M3_TRY=0\ninstall_all", check=False).returncode
        b = self.run_sh(SUDO_LOG + "uninstall_all", check=False).returncode if a == 0 else None
        return (a, b)

    def test_an_install_runs_as_on_12_2(self):
        for board, after in self.same(lambda: self.run_sh(SUDO_LOG + "M3_TRY=0\ninstall_all",
                                                          check=False).returncode):
            # Mesa before the install: none. After it: none, and nothing downloaded or recorded.
            self.assertNotIn("mesa", after["log"])
            self.assertFalse([p for p in after["tree"] if "mesa" in p])
            self.assertFalse([l for l in after["tree"]["fake/installed"].decode().split() if l.startswith("mesa")])

    def test_install_and_uninstall_with_the_owners_mesa(self):
        for board, after in self.same(self.install_and_uninstall, setup=lambda: owner_mesa(self)):
            log = after["log"]
            self.assertNotIn("pacman -Rn --noconfirm mesa-m3", log)
            self.assertFalse([l for l in log.splitlines() if l.startswith(("pacman -U", "pacman -R")) and "mesa" in l])
            installed = after["tree"]["fake/installed"].decode().split()
            self.assertIn("mesa-m3", installed)
            self.assertIn("mesa-m3-g15g", installed)
            self.assertEqual(after["tree"]["opt/mesa-m3/lib/libvulkan_asahi.so"], b"the owner's own build")
            home = self.home.relative_to(self.tmp).as_posix()
            self.assertEqual(after["tree"][f"{home}/.config/chonkstep/m3gpu-session.env"], CHONKSTEP_ENV)
            self.assertNotIn("gpasswd", log)

    def test_no_m3_mesa_changes_nothing_elsewhere(self):
        # The option is taken on every Mac (one release, one set of options) and does nothing there.
        def run():
            return self.run_sh(SUDO_LOG + "M3_TRY=0\nM3_PRO_MESA=0\ninstall_all", check=False).returncode
        for board, after in self.same(run, boards=["j314s", "j504", "j516c"]):
            self.assertNotIn("--no-m3-mesa", after["log"])


class ReleaseAssetTest(unittest.TestCase):
    def test_the_package_is_an_asset_of_its_own(self):
        # Never in PACKAGES (what every Mac downloads, and what the lab's common manifest checks).
        block = re.search(r"^PACKAGES=\(\n(.*?)^\)", SRC, re.M | re.S).group(1)
        self.assertNotIn("mesa", block)
        self.assertRegex(SRC, r'(?m)^M3_PRO_MESA_PACKAGE="mesa-m3-')
        self.assertRegex(SRC, r'(?m)^M3_PRO_MESA_PREFIX="/opt/mesa-m3"$')

    def test_nothing_is_written_for_it_outside_the_package(self):
        # The package owns the session integration: this script never writes, links or deletes
        # anything under the prefix, the package's switch-offs or a home directory. Its only
        # writes are the set-aside in its work directory and the record's publication in $STATE.
        body = SRC[SRC.index("# ---- the M3's Mesa"):SRC.index("this_board() {")]
        # Without the byte copies of mesa-m3's files and the recovery text (heredocs, printed only).
        body = re.sub(r"cat <<'(M3_PRO_MESA_[A-Z_]+)'\n.*?\n\1\n", "", body, flags=re.S)
        code = [l.strip() for l in body.splitlines() if not l.strip().startswith("#")]
        allowed = {
            'if ! tmp=$($sudo mktemp "$STATE/.$M3_PRO_MESA_RECORD_NAME.XXXXXX"); then',
            'if ! printf \'%s\\n\' "$@" | $sudo tee "$tmp" >/dev/null; then',
            'elif ! $sudo chmod 0644 "$tmp"; then',
            'elif ! $sudo mv -f "$tmp" "$rec"; then',
            '$sudo rm -f "$tmp" || M3_PRO_MESA_WRITE_ERR+="; $tmp could not be removed"',
            '$sudo mv -n "$rec" "$name" || true',
            'dir=$(mktemp -d) || return 2',
            'm3_pro_mesa_builtin_detector >"$dir/mesa-m3-user-setup"',
            'm3_pro_mesa_builtin_list >"$dir/user-setup.list"',
            'rm -rf "$dir"',
            'if $sudo gpasswd -a "$user" "$g" >/dev/null && m3_pro_mesa_in_group "$user" "$g"; then',
            'if $sudo gpasswd -d "$u" "$g" >/dev/null; then',
            'mkdir -p "$work/m3-pro"',
            'mv "$work/$file" "$work/m3-pro/"',
        }
        # Commands in command position (line start, after a pipe, a list operator, if, then or $( ),
        # and redirections into a variable's path.
        verb = (r"(^|&&|\|\||\||;|\bthen\b|\b(el)?if !?|\$\()\s*(\$sudo\s+)?"
                r"(rm|ln|mv|tee|mktemp|chmod|chown|mkdir|cp|install|touch|truncate|dd|sed -i|gpasswd|usermod)\b")
        writes = [l for l in code if re.search(verb, l) or re.search(r"(^|\s)>>?\s*\"?\$(?!\(|\{?work)", l)]
        self.assertEqual(sorted(set(writes) - allowed), [])
        self.assertEqual(sorted(allowed - set(writes)), [])


if __name__ == "__main__":
    unittest.main()
