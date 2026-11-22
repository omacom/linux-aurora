#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""SEP writes are always on (aurora-silicon/linux#2591).

The driver has no xart_writes or provision_keybag parameter any more, so nothing here passes or
ships one: load-driver runs a plain `modprobe apple_sep`, no modprobe.d file the packages install
sets either, and the installer refuses --read-only. An install, a re-run and --reset-touchid
delete the file an earlier --read-only install wrote, and keep any other file at that path, with
a warning. Nothing here touches the host: the installer's commands are stubbed.
"""
import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

SEP = Path(__file__).resolve().parents[1]
INSTALLER = SEP / "install-aurora-sep.sh"
SRC = INSTALLER.read_text()
REMOVED = "--read-only was removed: SEP writes are always on (aurora-silicon/linux#2591)"
# What an earlier --read-only install wrote to /etc/modprobe.d/aurora-sep.conf.
OLD_READ_ONLY_CONF = (
    "# aurora-sep --read-only: the driver attaches and reports, but never writes\n"
    "# to the enclave. Delete this file to allow enrolment.\n"
    "options apple_sep xart_writes=0 provision_keybag=0\n")
WRITE_PARAMS = re.compile(r"\b(xart_writes|provision_keybag)\b")
KEPT = "is not the file an earlier --read-only install wrote, so it stays as it is"
IGNORED = "Kernels from aurora-silicon/linux#2591 on ignore xart_writes and provision_keybag"


def flat(text):
    return " ".join(text.split())


def code_lines(text):
    return [line for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")]


class LoadDriverTest(unittest.TestCase):
    def test_modprobe_gets_no_options(self):
        source = (SEP / "load-driver").read_text().split("work=$(mktemp", 1)[0]
        for parameter in (None, "0"):
            with self.subTest(keystore_only=parameter), tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)
                param = path / "keystore_only"
                if parameter is not None:
                    param.write_text(parameter + "\n")
                modprobe = path / "modprobe"
                modprobe.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >>"$CALLS"\n')
                modprobe.chmod(0o755)
                script = path / "load-driver"
                script.write_text(source.replace("/sys/module/apple_sep/parameters/keystore_only", str(param))
                                  .replace("/sys/bus/platform/devices", str(path / "devices")))
                calls = path / "calls"
                env = dict(os.environ, AURORA_SEP_SESSION="1", CALLS=str(calls),
                           PATH=str(path) + os.pathsep + os.environ["PATH"])
                done = subprocess.run(["sh", str(script)], env=env, capture_output=True, text=True)
                self.assertEqual(done.returncode, 0, done.stderr)
                self.assertEqual(calls.read_text(), "apple_sep\n")

    def test_no_write_parameter_outside_comments(self):
        for line in code_lines((SEP / "load-driver").read_text()):
            self.assertNotRegex(line, WRITE_PARAMS)


class PackagedModprobeTest(unittest.TestCase):
    def packaged_modprobe_sources(self):
        found = set()
        pkgbuild = (SEP / "packaging/arch/aurora-sep/PKGBUILD").read_text()
        for source, dest in re.findall(r"install -Dm\d+ (\S+) (\S*modprobe\.d/\S*)", pkgbuild):
            found.add(source)
        spec = (SEP / "packaging/fedora/aurora-sep.spec").read_text()
        sources = dict(re.findall(r"(?m)^Source(\d+):\s+(\S+)$", spec))
        for number in re.findall(r"install -D\w* \d+ %\{SOURCE(\d+)\} \S*modprobe\.d/", spec):
            found.add(sources[number])
        return found

    def test_no_packaged_modprobe_file_sets_a_write_parameter(self):
        sources = self.packaged_modprobe_sources()
        self.assertIn("aurora-sep-load.conf", sources)
        for name in sources | {p.name for p in SEP.glob("*.conf")}:
            with self.subTest(file=name):
                for line in code_lines((SEP / name).read_text()):
                    self.assertNotRegex(line, WRITE_PARAMS)

    def test_the_installer_never_writes_the_read_only_file(self):
        self.assertNotRegex(SRC, r'(tee|>)\s*"?\$MODPROBE_CONF')
        self.assertIsNone(re.search(r"\bREAD_ONLY\b", SRC))
        self.assertNotIn("bash -s -- --read-only", SRC)
        for line in code_lines(SRC):
            if "xart_writes" in line or "provision_keybag" in line:
                # Only to recognise the old file, to say these are ignored, and to find
                # (and tell the owner to remove) a leftover 0 a kernel from before
                # linux#2591 still honours. Never to set one.
                allowed = ("READ_ONLY_CONF_OPTIONS=", "ignore xart_writes",
                           "any xart_writes=0 or provision_keybag=0 it sets",
                           "modinfo -F parm apple_sep",
                           "sets xart_writes=0 or provision_keybag=0, and this kernel")
                self.assertTrue(line.startswith(allowed[0]) or any(a in line for a in allowed[1:]),
                                line)


class InstallerTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = Path(tmp.name)
        self.conf = self.tmp / "etc/modprobe.d/aurora-sep.conf"
        self.conf.parent.mkdir(parents=True)
        self.calls = self.tmp / "calls"
        self.calls.write_text("")
        self.bin = self.tmp / "bin"
        self.bin.mkdir()
        # Anything that would change the host, should the script reach it, only logs.
        for name in ("sudo", "pacman", "curl", "systemctl", "modprobe", "update-m1n1",
                     "grub-mkconfig", "mkinitcpio", "fprintd-list"):
            (self.bin / name).write_text(f'#!/bin/sh\necho "{name} $*" >>"$CALLS"\nexit 1\n')
            (self.bin / name).chmod(0o755)
        self.env = {**os.environ, "NO_COLOR": "1", "CALLS": str(self.calls),
                    "PATH": f"{self.bin}{os.pathsep}{os.environ['PATH']}"}

    def sh(self, body, real_block=False):
        # The real sep_block_line reads only this test's modprobe.d, never the host's.
        block = (f"SEP_MODPROBE_DIRS='{self.conf.parent}'" if real_block
                 else "sep_block_line() { :; }")
        script = f"""set -euo pipefail
AURORA_SEP_SOURCE_ONLY=1 source '{INSTALLER}'
sudo=""
MODPROBE_CONF='{self.conf}'
systemctl() {{ echo "systemctl $*" >>"$CALLS"; }}
{block}
{body}
"""
        # A session of its own: no controlling terminal, so nothing can prompt on it.
        return subprocess.run(["bash", "-c", script], capture_output=True, text=True, env=self.env,
                              start_new_session=True)

    def log(self):
        return self.calls.read_text()

    def test_read_only_is_refused(self):
        for args in (["--read-only"], ["--desktop-fixes", "--read-only"], ["--read-only", "--uninstall"],
                     ["--m3-gpu-experiment", "--read-only"], ["--reset-touchid", "--read-only"]):
            with self.subTest(args=args):
                proc = subprocess.run(["bash", str(INSTALLER), *args], capture_output=True, text=True,
                                      env=self.env, start_new_session=True, stdin=subprocess.DEVNULL)
                self.assertEqual(proc.returncode, 1, proc.stderr)
                self.assertIn(REMOVED, proc.stderr)
                self.assertEqual(proc.stdout, "")
                self.assertEqual(self.log(), "")

    def test_read_only_is_not_an_option_any_more(self):
        self.assertNotIn("--read-only)", SRC.replace(f'--read-only) die "{REMOVED}"', ""))
        usage = re.findall(r"(?m)^#   \.\.\. \| bash -s -- (\S+)", SRC[:SRC.index("\nset -euo pipefail\n")])
        self.assertIn("--uninstall", usage)
        self.assertNotIn("--read-only", usage)
        unknown = re.search(r'\*\) die "unknown option \$1 \(([^)]*)\)"', SRC).group(1)
        self.assertNotIn("--read-only", unknown)

    def test_install_deletes_an_earlier_read_only_file(self):
        for content in (OLD_READ_ONLY_CONF, OLD_READ_ONLY_CONF.rstrip("\n"),
                        OLD_READ_ONLY_CONF + "\n# kept for now\n",
                        OLD_READ_ONLY_CONF.replace("options", "  options") + "   \n"):
            with self.subTest(content=content):
                self.conf.write_text(content)
                self.calls.write_text("")
                proc = self.sh("sep_policy")
                self.assertEqual(proc.returncode, 0, proc.stderr)
                self.assertFalse(self.conf.exists())
                self.assertIn(f"Removed {self.conf}, left by an earlier --read-only install: "
                              "SEP writes are always on", proc.stdout)
                self.assertNotIn("warning", proc.stderr)
                self.assertIn("systemctl enable apple-sep.path apple-sep.service", self.log())
                # A re-run finds nothing to do.
                again = self.sh("sep_policy")
                self.assertEqual(again.returncode, 0, again.stderr)
                self.assertNotIn("Removed", again.stdout)
                self.assertEqual(again.stderr, "")

    def test_install_runs_the_cleanup(self):
        proc = self.sh("declare -f install_all; declare -f sep_policy")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        install_all, sep_policy = proc.stdout.split("\nsep_policy ()", 1)
        self.assertRegex(install_all, r"(?m)^\s+sep_policy;?$")
        self.assertRegex(sep_policy, r"(?m)^\s+read_only_conf_cleanup;?$")

    def test_any_other_file_stays_with_a_warning(self):
        target = self.tmp / "elsewhere.conf"
        target.write_text(OLD_READ_ONLY_CONF)
        for name, content in (("owner's", "options apple_sep xart_guard=0\n"),
                              ("by hand", "options apple_sep xart_writes=0 provision_keybag=0\n"),
                              ("edited", OLD_READ_ONLY_CONF + "options apple_sep xart_guard=0\n"),
                              ("blacklist", OLD_READ_ONLY_CONF + "blacklist apple_sep\n"),
                              ("header later", "# mine\n" + OLD_READ_ONLY_CONF),
                              ("empty", ""),
                              ("symlink", None)):
            with self.subTest(file=name):
                self.conf.unlink(missing_ok=True)
                if content is None:
                    self.conf.symlink_to(target)
                else:
                    self.conf.write_text(content)
                self.calls.write_text("")
                proc = self.sh("sep_policy")
                self.assertEqual(proc.returncode, 0, proc.stderr)
                self.assertEqual(self.conf.read_text(), OLD_READ_ONLY_CONF if content is None else content)
                self.assertEqual(self.conf.is_symlink(), content is None)
                self.assertIn(KEPT, flat(proc.stderr))
                self.assertIn(IGNORED, flat(proc.stderr))
                self.assertNotIn("Removed", proc.stdout)
                self.assertIn("systemctl enable apple-sep.path apple-sep.service", self.log())
        self.assertEqual(target.read_text(), OLD_READ_ONLY_CONF)

    def test_an_owners_block_still_wins(self):
        self.conf.write_text(OLD_READ_ONLY_CONF)
        proc = self.sh("sep_block_line() { echo '/etc/modprobe.d/mine.conf:1:blacklist apple_sep'; }\n"
                       "sep_policy")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertFalse(self.conf.exists())
        self.assertIn("apple_sep is blocked on this Mac (/etc/modprobe.d/mine.conf:1:blacklist apple_sep)",
                      proc.stderr)
        self.assertIn("systemctl disable apple-sep.path apple-sep.service", self.log())
        self.assertNotIn("systemctl enable", self.log())

    def test_an_owners_block_in_the_kept_file_wins(self):
        # The script no longer writes $MODPROBE_CONF, so a block an owner put there counts too.
        for line in ("blacklist apple_sep", "install apple-sep /bin/false"):
            with self.subTest(line=line):
                self.conf.write_text(OLD_READ_ONLY_CONF + line + "\n")
                self.calls.write_text("")
                proc = self.sh("sep_policy", real_block=True)
                self.assertEqual(proc.returncode, 0, proc.stderr)
                self.assertTrue(self.conf.exists())
                self.assertIn(f"apple_sep is blocked on this Mac ({self.conf}:4:{line})", flat(proc.stderr))
                self.assertIn("systemctl disable apple-sep.path apple-sep.service", self.log())
                self.assertNotIn("systemctl enable", self.log())

    def test_the_real_block_check_ignores_options_lines(self):
        self.conf.write_text("options apple_sep xart_guard=0\n")
        proc = self.sh("sep_policy", real_block=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("systemctl enable apple-sep.path apple-sep.service", self.log())

    def test_uninstall_removes_only_the_read_only_file(self):
        proc = self.sh("declare -f uninstall_all")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertRegex(proc.stdout, r"(?m)^\s+read_only_conf_cleanup;?$")
        self.assertNotIn('rm -f "$MODPROBE_CONF"', proc.stdout)

    def test_the_write_notice_offers_no_read_only_install(self):
        self.conf.write_text(OLD_READ_ONLY_CONF)
        proc = self.sh("sep_write_notice")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        err = flat(proc.stderr)
        self.assertIn("the SEP driver creates an identity keybag and writes to the enclave's anti-replay "
                      "store (xART). SEP writes are always on. Use a Mac you can DFU-restore", err)
        self.assertNotIn("read-only", err)
        # An owner's block: the driver is not loaded, so there is nothing to warn about.
        proc = self.sh("sep_block_line() { echo '/etc/modprobe.d/mine.conf:1:blacklist apple_sep'; }\n"
                       "sep_write_notice")
        self.assertEqual((proc.returncode, proc.stderr), (0, ""))

    def test_the_agent_prompt_offers_no_read_only_install(self):
        start = SRC.index("SAFETY, NON-NEGOTIABLE")
        safety = flat(SRC[start:SRC.index("\nPROMPT\n", start)])
        self.assertIn("SEP writes are always on; there is no read-only install.", safety)
        self.assertIn("If the owner cannot DFU-restore, do not install.", safety)
        self.assertNotIn("--read-only", safety)

    # --reset-touchid with its commands stubbed: the device tree says Apple, the driver reports a
    # failed keybag, and the state "moves" only into the log.
    RESET_STUBS = r"""
grep() { if [[ ${!#} == /proc/device-tree/compatible ]]; then return 0; fi; command grep "$@"; }
sep_diag() { echo failed; }
sleep() { :; }
find() { :; }
mkdir() { echo "mkdir $*" >>"$CALLS"; }
mv() { echo "mv $*" >>"$CALLS"; }
"""

    def reset(self, *args):
        state = self.tmp / "aurora-sep-keybag.bin"
        state.write_text("keybag")
        return self.sh(f"{self.RESET_STUBS}\nTOUCHID_STATE_FILES=('{state}')\n"
                       f"reset_touchid {' '.join(args)}")

    def test_reset_touchid_is_not_refused_for_the_read_only_file(self):
        self.conf.write_text(OLD_READ_ONLY_CONF)
        proc = self.reset("--yes")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertNotIn("installed read-only", proc.stderr)
        self.assertFalse(self.conf.exists())
        self.assertIn(f"Removed {self.conf}, left by an earlier --read-only install", proc.stdout)
        self.assertIn("Moved the Touch ID state to /var/lib/aurora-sep-touchid-reset-", proc.stdout)
        self.assertIn("systemctl mask --runtime --now fprintd.service", self.log())
        self.assertIn(f"mv {self.tmp / 'aurora-sep-keybag.bin'} ", self.log())

    def test_reset_touchid_changes_nothing_before_it_is_confirmed(self):
        self.conf.write_text(OLD_READ_ONLY_CONF)
        proc = self.reset()
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("no terminal to confirm on; run again with --reset-touchid --yes", proc.stderr)
        self.assertEqual(self.conf.read_text(), OLD_READ_ONLY_CONF)
        self.assertEqual(self.log(), "")

    def modinfo(self, parms):
        (self.bin / "modinfo").write_text("#!/bin/sh\nprintf '%s' \"$PARMS\"\n")
        (self.bin / "modinfo").chmod(0o755)
        self.env["PARMS"] = parms

    OLD_PARMS = "xart_writes:Allow shared xART writes (u8)\nprovision_keybag:Create the keybag (u8)\n"
    NEW_PARMS = "xart_guard:-1: the platform's level (i8)\n"

    def test_kept_conf_turns_writes_off(self):
        cases = [("options apple_sep xart_writes=0 provision_keybag=0\n", self.OLD_PARMS, True),
                 ("options apple-sep provision-keybag=0\n", self.OLD_PARMS, True),
                 ("options apple_sep xart_guard=0 xart_writes=00\n", self.OLD_PARMS, True),
                 ("options apple_sep xart_writes=0 provision_keybag=0\n", self.NEW_PARMS, False),
                 ("options apple_sep xart_guard=0\n", self.OLD_PARMS, False),
                 ("options apple_sep xart_writes=1\n", self.OLD_PARMS, False),
                 ("# options apple_sep xart_writes=0\n", self.OLD_PARMS, False),
                 (OLD_READ_ONLY_CONF, self.OLD_PARMS, False)]
        for content, parms, off in cases:
            with self.subTest(content=content, parms=parms):
                self.conf.write_text(content)
                self.modinfo(parms)
                proc = self.sh("kept_conf_turns_writes_off && echo OFF || echo ON")
                self.assertEqual(proc.stdout.strip(), "OFF" if off else "ON", proc.stderr)

    def test_reset_touchid_refuses_an_owners_off_setting_on_an_old_kernel(self):
        self.conf.write_text("options apple_sep xart_writes=0 provision_keybag=0\n")
        self.modinfo(self.OLD_PARMS)
        proc = self.reset("--yes")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("still honours it: the next boot could not create a new keybag", flat(proc.stderr))
        self.assertEqual(self.log(), "")
        # The same file under a kernel without the parameters: the reset goes ahead.
        self.modinfo(self.NEW_PARMS)
        proc = self.reset("--yes")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertIn("Moved the Touch ID state", proc.stdout)

    def test_reset_touchid_stops_if_the_read_only_file_stays(self):
        # rm fails (a read-only /etc, say): nothing is moved, and the reset says why.
        self.conf.write_text(OLD_READ_ONLY_CONF)
        state = self.tmp / "aurora-sep-keybag.bin"
        state.write_text("keybag")
        proc = self.sh(f"{self.RESET_STUBS}\nrm() {{ return 1; }}\nTOUCHID_STATE_FILES=('{state}')\n"
                       "reset_touchid --yes")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("could not be removed; nothing was reset", flat(proc.stderr))
        self.assertEqual(self.conf.read_text(), OLD_READ_ONLY_CONF)
        self.assertNotIn("systemctl mask", self.log())
        self.assertNotIn("mv ", self.log())

    def test_the_modprobe_dirs_override_is_test_only(self):
        self.assertIn('SEP_MODPROBE_DIRS=${AURORA_SEP_TEST_MODPROBE_DIRS:-', SRC)
        self.assertNotIn("${SEP_MODPROBE_DIRS:-", SRC)

    def test_reset_touchid_keeps_an_owners_file(self):
        self.conf.write_text("options apple_sep xart_guard=0\n")
        proc = self.reset("--yes")
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(self.conf.read_text(), "options apple_sep xart_guard=0\n")
        self.assertIn(KEPT, flat(proc.stderr))
        self.assertIn("Moved the Touch ID state", proc.stdout)


if __name__ == "__main__":
    unittest.main()
