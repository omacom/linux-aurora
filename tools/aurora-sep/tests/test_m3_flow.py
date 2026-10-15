"""install_all and uninstall_all end to end, against a fake Mac.

The real functions run; pacman, update-m1n1, curl, findmnt and systemctl are
small stubs that record what happened. The fake update-m1n1 behaves like the
real one where it matters here: it honours M1N1_UPDATE_DISABLED and M1N1=,
builds boot.bin from the installed m1n1 and device trees, and appends
/etc/m1n1.conf's chosen.* lines. pacman -U and the m1n1/kernel installs run it,
as the hooks do. pacman -U installs a package's own m1n1.bin, so boot.bin
starts with the bytes of the package the installer downloaded; every fixture
m1n1.bin starts with "M1N1:<package name>".
"""
from pathlib import Path
import hashlib
import os
import re
import shutil
import subprocess
import tempfile
import unittest

INSTALLER = Path(__file__).resolve().parent.parent / "install-aurora-sep.sh"
SRC = INSTALLER.read_text()
# The installer's VERSION names the boot.bin copy it keeps on the EFI partition.
VERSION = re.search(r"^VERSION=(\S+)$", SRC, re.M).group(1)
# The one m1n1 package every Mac gets, by the name this release ships.
M1N1_PKG = re.search(r'^M1N1_PACKAGE="(\S+) ', SRC, re.M).group(1)
M1N1_BASE = M1N1_PKG[:-len("-aarch64.pkg.tar.zst")]
# What $STATE/m3-mode records for an M3 Pro on this release's m1n1.
PRO_VARIANT = re.search(r'^M3_PRO_VARIANT="([^"]*)"$', SRC, re.M).group(1)
# Every switch name the shipped m1n1 has to know: the M3 Pro's and the Air's.
SWITCH_NAMES = sorted({w[len("chosen."):].split("=")[0]
                       for w in re.findall(r"chosen\.asahi,t(?:6030|8122)-[a-z0-9-]+=1", SRC)})

BOARDS = {
    "j516s": ["apple,j516s", "apple,t6030", "apple,arm-platform"],
    "j514s": ["apple,j514s", "apple,t6030", "apple,arm-platform"],
    "j613": ["apple,j613", "apple,t8122", "apple,arm-platform"],
    "j615": ["apple,j615", "apple,t8122", "apple,arm-platform"],
    "j504": ["apple,j504", "apple,t8122", "apple,arm-platform"],
    "j516c": ["apple,j516c", "apple,t6031", "apple,arm-platform"],
    "j516m": ["apple,j516m", "apple,t6034", "apple,arm-platform"],
    "j514c": ["apple,j514c", "apple,t6031", "apple,arm-platform"],
    "j514m": ["apple,j514m", "apple,t6034", "apple,arm-platform"],
    "j314s": ["apple,j314s", "apple,t6000", "apple,arm-platform"],
    "j293": ["apple,j293", "apple,t8103", "apple,arm-platform"],
    "j313": ["apple,j313", "apple,t8103", "apple,arm-platform"],
    "j314c": ["apple,j314c", "apple,t6001", "apple,arm-platform"],
    "j414s": ["apple,j414s", "apple,t6020", "apple,arm-platform"],
    "j700": ["apple,j700", "apple,t8140", "apple,arm-platform"],
}
SWITCHES = b"chosen.asahi,t6030-gpu=1\nchosen.asahi,t6030-dcp=1\nchosen.asahi,t6030-dcpext=1\n"
BRINGUP_FREEZE = (
    "# Added by m3-gpu-work/scripts/install-m3gpu.sh: keep the M3 GPU candidate boot.bin.\n"
    "# Undo with rollback-m3gpu.sh (then run: sudo update-m1n1).\n"
    "M1N1_UPDATE_DISABLED=1\n"
)
OUR_FREEZE = (
    "# >>> aurora-sep: keep this M3's boot.bin as it is (remove with: install-aurora-sep.sh --uninstall)\n"
    "M1N1_UPDATE_DISABLED=1\n"
    "# <<< aurora-sep: keep this M3's boot.bin as it is\n"
)
# The M3 Pro's Mesa package of the fake release, and what it needs.
PRO_MESA_VERSION = "26.1.4.g15s1-1"
PRO_MESA = f"mesa-m3-{PRO_MESA_VERSION}-aarch64.pkg.tar.zst"
PRO_MESA_NEEDS = re.search(r'^M3_PRO_MESA_NEEDS="([^"]*)"$', SRC, re.M).group(1)
# An older M3 m1n1 that knows only the M3 Pro's switches.
AURORA6 = "m1n1-aurora-1.6.1.aurora6-1-aarch64.pkg.tar.zst"
OTHERS = [
    "linux-aurora-7.1.12.aurora2-11.36-aarch64.pkg.tar.zst",
    "linux-aurora-headers-7.1.12.aurora2-11.36-aarch64.pkg.tar.zst",
    "libfprint-1.94.100-1.1-aarch64.pkg.tar.zst",
    "aurora-touchid-20261003-1-any.pkg.tar.zst",
]

PACMAN = r"""#!/bin/bash
echo "pacman $*" >>"$FAKE/log"
# The installed version of a package, or nothing when it is not installed.
ver() {
  case $1 in
    linux-aurora | linux-asahi) grep -qx "$1" "$FAKE/installed" && echo 1-1 ;;
    m1n1-aurora) grep -q '^m1n1-aurora' "$FAKE/m1n1" && echo 1-1 ;;
    m1n1) grep -q '^m1n1-stock' "$FAKE/m1n1" && echo 1-1 ;;
    # The Mesa packages' versioned dependencies; FAKE_<NAME>="" means not installed.
    glibc) echo "${FAKE_GLIBC-2.43+r9+g1-1}" ;;
    gcc-libs) echo "${FAKE_GCC_LIBS-15.2.1+r22-1}" ;;
    libgcc) echo "${FAKE_LIBGCC-16.1.1+r12-1}" ;;
    libstdc++) echo "${FAKE_LIBSTDCXX-16.1.1+r12-1}" ;;
    spirv-tools) echo "${FAKE_SPIRV_TOOLS-1:1.4.357.0-1}" ;;
    # A Mesa package has the version its .PKGINFO had when -U installed it.
    mesa-*) grep -qx "$1" "$FAKE/installed" &&
      awk -v p="$1" '$1 == p { v = $2 } END { print v ? v : "1-1" }' "$FAKE/versions" 2>/dev/null ;;
    *) echo 1-1 ;;
  esac
  return 0
}
op=$1; shift
case $op in
  -Q)
    # Without a name: every package in $FAKE/installed, with its version.
    if (($# == 0)); then
      while read -r p; do [[ -n $p ]] && echo "$p $(ver "$p")"; done <"$FAKE/installed"
      exit 0
    fi
    for p in "$@"; do
      v=$(ver "$p")
      [[ -n $v ]] || exit 1
      echo "$p $v"
    done ;;
  -T)
    # As pacman: the dependencies not satisfied, by name or by FAKE_PROVIDES ("name=version ...").
    rc=0
    for d in "$@"; do
      n=${d%%[<>=]*} m=""
      [[ $d == *">="* ]] && m=${d#*>=}
      v=$(ver "$n")
      if [[ -z $v ]]; then
        for pv in ${FAKE_PROVIDES:-}; do [[ ${pv%%=*} == "$n" ]] && v=${pv#*=}; done
      fi
      if [[ -z $v ]] || { [[ -n $m ]] && [[ $(vercmp "$v" "$m") == -1 ]]; }; then echo "$d"; rc=127; fi
    done
    exit $rc ;;
  -Qq) exit 1 ;;
  # Pending upgrades after -Sy, as "name old -> new [ignored]" lines; FAKE_UPGRADES has them.
  -Qu)
    if [[ -n ${FAKE_FAIL_QUERY:-} ]]; then echo "error: local database failed" >&2; exit 1; fi
    [[ -n ${FAKE_UPGRADES:-} ]] || exit 1
    printf '%s\n' "$FAKE_UPGRADES" ;;
  # A repository's packages as "repo name version [installed]"; FAKE_REPO_LIST has them,
  # and as pacman, a repository with none of them is not configured.
  -Sl)
    rows=$(while IFS= read -r r; do [[ ${r%% *} == "${!#}" ]] && echo "$r"; done <<<"${FAKE_REPO_LIST:-}") || true
    [[ -n $rows ]] || { echo "error: repository \"${!#}\" was not found." >&2; exit 1; }
    printf '%s\n' "$rows"
    exit 0 ;;
  -Qlq)
    [[ $1 == linux-aurora ]] && grep -qx linux-aurora "$FAKE/installed" &&
      echo /usr/lib/modules/7.1.12-2-11.36-sep-ARCH/dtbs/fake.dtb
    exit 0 ;;
  -U)
    if [[ -n ${FAKE_FAIL_U:-} ]]; then echo "error: failed to commit transaction" >&2; exit 1; fi
    # FAKE_FAIL_U_FOR: a glob; a transaction with a package whose file name matches it fails.
    for f in "$@"; do
      case $(basename -- "$f") in ${FAKE_FAIL_U_FOR:-/}) echo "error: failed to commit transaction (fake)" >&2; exit 1 ;; esac
    done
    hook=0
    for f in "$@"; do
      [[ $f == -* || $f == 4 ]] && continue
      b=$(basename "$f")
      case $b in
        m1n1-aurora-*) echo "${b%-aarch64.pkg.tar.zst}" >"$FAKE/m1n1"; bsdtar -xOf "$f" usr/lib/asahi-boot/m1n1.bin >"$FAKE/m1n1.bin"; hook=1 ;;
        linux-aurora-headers-*) ;;
        linux-aurora-*) sed -i '/^linux-asahi$/d' "$FAKE/installed"; echo linux-aurora >>"$FAKE/installed"; hook=1 ;;
        mesa-*)
          n=$(bsdtar -xOf "$f" .PKGINFO | sed -n 's/^pkgname = //p')
          v=$(bsdtar -xOf "$f" .PKGINFO | sed -n 's/^pkgver = //p')
          grep -qx "$n" "$FAKE/installed" || echo "$n" >>"$FAKE/installed"
          echo "$n $v" >>"$FAKE/versions" ;;
      esac
    done
    # As the real hooks: only a kernel or m1n1 package rebuilds boot.bin.
    if ((hook)); then update-m1n1 hook; fi ;;
  -Rns | -Rn)
    if [[ -n ${FAKE_FAIL_R:-} ]]; then echo "error: failed to remove (fake)" >&2; exit 1; fi
    for p in "$@"; do [[ $p == -* ]] || { sed -i "/^$p\$/d" "$FAKE/installed"; sed -i "/^$p /d" "$FAKE/versions" 2>/dev/null || true; }; done ;;
  -S | -Sy)
    if [[ $op == -Sy && -n ${FAKE_FAIL_REFRESH:-} ]]; then echo "error: mirror unavailable" >&2; exit 1; fi
    hook=0
    for p in "$@"; do
      case $p in
        m1n1) echo m1n1-stock >"$FAKE/m1n1"; printf 'M1N1:m1n1-stock\n' >"$FAKE/m1n1.bin"; hook=1 ;;
        linux-asahi) sed -i '/^linux-aurora$/d' "$FAKE/installed"; echo linux-asahi >>"$FAKE/installed"; hook=1 ;;
      esac
    done
    if ((hook)); then update-m1n1 hook; fi ;;
esac
exit 0
"""

# Group membership from $FAKE/groups ("user group ..." lines); everything else from the real id.
ID = r"""#!/bin/bash
if [[ ${1:-} == -nG ]]; then
  shift
  [[ ${1:-} == -- ]] && shift
  u=${1:-$(/usr/bin/id -un)}
  [[ -e $FAKE/no-such-user-$u ]] && exit 1
  line=$(grep -m1 "^$u " "$FAKE/groups" 2>/dev/null) || line=""
  echo "$u${line#"$u"}"
  exit 0
fi
exec /usr/bin/id "$@"
"""
GETENT = r"""#!/bin/bash
if [[ ${1:-} == group && ${2:-} == render ]]; then
  [[ -e $FAKE/no-render-group ]] && exit 2
  echo "render:x:989:"
  exit 0
fi
exec /usr/bin/getent "$@"
"""
GPASSWD = r"""#!/bin/bash
echo "gpasswd $*" >>"$FAKE/log"
if [[ -n ${FAKE_FAIL_GPASSWD:-} ]]; then echo "gpasswd: failed (fake)" >&2; exit 1; fi
u=$2 g=$3
case $1 in
  -a) if grep -q "^$u " "$FAKE/groups" 2>/dev/null; then sed -i "/^$u /s/\$/ $g/" "$FAKE/groups"
      else echo "$u $g" >>"$FAKE/groups"; fi ;;
  -d) sed -i -E "/^$u /s/ $g( |\$)/\1/" "$FAKE/groups" ;;
esac
"""

UPDATE_M1N1 = r"""#!/bin/bash
echo "update-m1n1 $*" >>"$FAKE/log"
# As the real one: sourced by sh under set -e (bash in POSIX mode on Arch).
if [[ -f $FAKE_UPDATE_CONF ]]; then
  if ! sh -c 'set -e; . "$1"' _ "$FAKE_UPDATE_CONF" 2>/dev/null; then
    echo "update-m1n1 failed sourcing" >>"$FAKE/log"
    exit 1
  fi
  if sh -c 'set -e; . "$1"; [ -n "${M1N1_UPDATE_DISABLED:-}" ]' _ "$FAKE_UPDATE_CONF" 2>/dev/null; then
    echo "update-m1n1 frozen" >>"$FAKE/log"
    exit 0
  fi
fi
dtbs=$(sh -c 'set -e; DTBS=; [ -f "$1" ] && . "$1"; echo "$DTBS"' _ "$FAKE_UPDATE_CONF")
m1n1=$(sh -c 'set -e; M1N1=; [ -f "$1" ] && . "$1"; echo "$M1N1"' _ "$FAKE_UPDATE_CONF")
# A rebuild that picks up another m1n1 than the installed one.
m1n1=${FAKE_OTHER_M1N1:-$m1n1}
{
  cat "${m1n1:-$FAKE/m1n1.bin}"
  printf 'DTBS:%s\n' "${dtbs:-asahi}"
  printf 'UBOOT'
  grep -E '^chosen\.' "$FAKE_M1N1_CONF" 2>/dev/null || true
} >"$FAKE_ESP/m1n1/boot.bin"
echo "update-m1n1 rebuilt" >>"$FAKE/log"
"""

CURL = r"""#!/bin/bash
out= url= writeout=
while (($#)); do
  case $1 in -o) out=$2; shift ;; -w) writeout=$2; shift ;; http*) url=$1 ;; esac
  shift
done
echo "curl $url" >>"$FAKE/log"
if cp "$FAKE_PKGS/$(basename "$url")" "$out"; then
  [[ -z $writeout ]] || printf 200
else
  [[ -z $writeout ]] || printf 404
  exit 22
fi
"""


class M3FlowBase(unittest.TestCase):
    """The fake Mac and its helpers, without tests (test_m3_air.py uses it too)."""

    def setUp(self):
        if not shutil.which("bsdtar") or not shutil.which("zstd"):
            self.skipTest("bsdtar and zstd are needed to build the fixture m1n1 packages")
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp)
        for d in ("dt/chosen", "esp/m1n1", "esp/asahi", "etc/default", "state", "pkgs", "bin", "fake"):
            (self.tmp / d).mkdir(parents=True)
        self.boot = self.tmp / "esp/m1n1/boot.bin"
        self.update_conf = self.tmp / "etc/default/update-m1n1"
        self.m1n1_conf = self.tmp / "etc/m1n1.conf"
        self.state = self.tmp / "state"
        self.fake = self.tmp / "fake"
        for name, body in (("pacman", PACMAN), ("update-m1n1", UPDATE_M1N1), ("curl", CURL),
                           ("pacman-conf", '#!/bin/sh\n[ -z "$FAKE_FAIL_CONFIG" ] || exit 1\nprintf "%s\\n" "$FAKE_HOLDS"\n'),
                           ("id", ID), ("getent", GETENT), ("gpasswd", GPASSWD),
                           ("findmnt", "#!/bin/sh\ncase \"$*\" in *PARTUUID*) echo fake-uuid ;; *) echo vfat ;; esac\n"),
                           ("systemctl", "#!/bin/sh\nexit 0\n")):
            (self.tmp / "bin" / name).write_text(body)
            (self.tmp / "bin" / name).chmod(0o755)
        self.extra_env = {}
        self.shas = {}
        self.bin_shas = {}
        for name in OTHERS:
            self.fixture(name, name.encode())
        self.fixture(M1N1_PKG, None, SWITCH_NAMES)
        self.fixture(AURORA6, None, ["asahi,t6030-gpu", "asahi,t6030-dcp", "asahi,t6030-dcpext"])
        # The M3 Pro's Mesa package of this fake release (a stand-in by the shipped name's form).
        self.pro_mesa = PRO_MESA
        self.pro_mesa_fixture(PRO_MESA, "mesa-m3", PRO_MESA_VERSION, "opt/mesa-m3")
        # The Mac's own files for the user-setup and switch-off checks (their --root), and the
        # invoking user's home there: home_path as getent gives it on the Mac, home on this host.
        self.sysroot = self.tmp / "sysroot"
        self.home_path = "/home/u"
        self.home = self.sysroot / self.home_path.lstrip("/")
        self.home.mkdir(parents=True, exist_ok=True)
        # The m1n1 package every Mac gets in this fake release.
        self.m1n1_pkg = M1N1_PKG
        # The script under test; a test can run an earlier release's first.
        self.installer = INSTALLER

    def pro_mesa_fixture(self, name, pkgname, pkgver, prefix):
        # A pacman package holding only .PKGINFO and a file under its prefix.
        root = self.tmp / ("root-" + name)
        shutil.rmtree(root, ignore_errors=True)
        (root / prefix / "lib").mkdir(parents=True)
        (root / prefix / "lib/libvulkan_asahi.so").write_bytes(b"stand-in " + name.encode())
        (root / ".PKGINFO").write_text(f"pkgname = {pkgname}\npkgver = {pkgver}\narch = aarch64\n")
        path = self.tmp / "pkgs" / name
        subprocess.run(["bsdtar", "--zstd", "-cf", str(path), "-C", str(root), ".PKGINFO", prefix.split("/")[0]],
                       check=True)
        self.shas[name] = hashlib.sha256(path.read_bytes()).hexdigest()

    def fixture(self, name, data, strings=None):
        path = self.tmp / "pkgs" / name
        if data is not None:
            path.write_bytes(data)
        else:
            root = self.tmp / ("root-" + name)
            shutil.rmtree(root, ignore_errors=True)
            (root / "usr/lib/asahi-boot").mkdir(parents=True)
            base = name.removesuffix("-aarch64.pkg.tar.zst").removesuffix(".pkg.tar.zst")
            m1n1 = (b"M1N1:" + base.encode() + b"\n" + b"m1n1\0" + b"\0".join(s.encode() for s in strings)
                    + b"\0end" + bytes(range(256)) * 4096)
            (root / "usr/lib/asahi-boot/m1n1.bin").write_bytes(m1n1)
            subprocess.run(["bsdtar", "--zstd", "-cf", str(path), "-C", str(root), "usr"], check=True)
            self.bin_shas[name] = hashlib.sha256(m1n1).hexdigest()
        self.shas[name] = hashlib.sha256(path.read_bytes()).hexdigest()

    def mac(self, board, kernel="linux-asahi", m1n1="m1n1-stock", bootbin=b"M1N1:original\n",
            stub="14.8.3", iboot="iBoot-10151.140.19.700.2", stage1="v1.6.1-dirty"):
        (self.tmp / "dt/compatible").write_bytes(b"".join(c.encode() + b"\0" for c in BOARDS[board]))
        (self.tmp / "dt/chosen/asahi,iboot2-version").write_bytes(iboot.encode() + b"\0")
        stage1_node = self.tmp / "dt/chosen/asahi,m1n1-stage1-version"
        if stage1 is None:
            stage1_node.unlink(missing_ok=True)
        else:
            stage1_node.write_bytes(stage1.encode() + b"\0")
        (self.tmp / "esp/asahi/stub_info.json").write_text(
            '{"system_version": {"ProductVersion": "%s"}}' % stub)
        (self.fake / "installed").write_text(kernel + "\n")
        (self.fake / "m1n1").write_text(m1n1 + "\n")
        (self.fake / "m1n1.bin").write_text("M1N1:" + m1n1 + "\n")
        (self.fake / "log").write_text("")
        self.boot.write_bytes(bootbin)

    def run_sh(self, body, check=True):
        pkgs = "\n".join(f'  "{n} {self.shas[n]}"' for n in OTHERS)
        script = f"""
set -euo pipefail
AURORA_SEP_SOURCE_ONLY=1 source '{self.installer}'
sudo=""
DT='{self.tmp}/dt'
STATE='{self.state}'
M1N1_CONF='{self.m1n1_conf}'
UPDATE_M1N1_CONF='{self.update_conf}'
M3GPU_MARKER='{self.tmp}/etc/default/.update-m1n1.created-by-m3gpu'
MODPROBE_CONF='{self.tmp}/etc/aurora-sep.conf'
OMARCHY_ARM_SOURCES='{self.tmp}/omarchy/arm-package-sources.sh'
M1N1_BIN='{self.fake}/m1n1.bin'
PACKAGES=(
{pkgs}
)
M1N1_PACKAGE="{self.m1n1_pkg} {self.shas[self.m1n1_pkg]}"
M1N1_BIN_SHA={self.bin_shas[self.m1n1_pkg]}
M3_PRO_MESA_PACKAGE="{self.pro_mesa + ' ' + self.shas[self.pro_mesa] if self.pro_mesa else ''}"
M3_PRO_MESA_NEEDS="{PRO_MESA_NEEDS}"
M3_PRO_NATIVE_PACKAGE=""
M3_PRO_VULKAN_PACKAGE=""
M3_PRO_PERF_MIGRATOR=""
M3_PRO_MESA_DETECTOR='{self.tmp}/opt/mesa-m3/libexec/mesa-m3-user-setup'
M3_PRO_MESA_SETUP_ROOT='{self.sysroot}'
m3_pro_mesa_user_home() {{ echo '{self.home_path}'; }}
esp_bootbin() {{ echo '{self.boot}'; }}
version_notice() {{ :; }}; sep_write_notice() {{ :; }}; ane_dkms_notice() {{ :; }}
snapshot() {{ :; }}; add_pin() {{ :; }}; remove_pin() {{ :; }}
calibration() {{ :; }}; sep_policy() {{ :; }}; neo_radio_notice() {{ :; }}
boot_chain() {{ echo limine; }}
{body}
"""
        env = {**os.environ, "NO_COLOR": "1", "PATH": f"{self.tmp}/bin:{os.environ['PATH']}",
               "FAKE": str(self.fake), "FAKE_PKGS": str(self.tmp / "pkgs"),
               "FAKE_ESP": str(self.tmp / "esp"), "FAKE_UPDATE_CONF": str(self.update_conf),
               "FAKE_M1N1_CONF": str(self.m1n1_conf)}
        env.update(self.extra_env)
        proc = subprocess.run(["bash", "-c", script], capture_output=True, text=True, env=env)
        if check:
            self.assertEqual(proc.returncode, 0, proc.stderr + proc.stdout)
        return proc

    def fresh_state(self):
        # A Mac this script never ran on, for the next subtest.
        shutil.rmtree(self.state)
        self.state.mkdir()
        self.m1n1_conf.unlink(missing_ok=True)
        self.update_conf.unlink(missing_ok=True)
        for kept in self.boot.parent.glob("boot.bin.*"):
            kept.unlink()

    def install(self, try_=0, check=True, env=""):
        return self.run_sh(f"{env}\nM3_TRY={try_}\ninstall_all", check=check)

    def uninstall(self):
        return self.run_sh("uninstall_all")

    def log(self):
        return (self.fake / "log").read_text()

    def downloaded(self):
        return [l.rsplit("/", 1)[1] for l in self.log().splitlines() if l.startswith("curl ")]

    def kept_copy(self):
        return self.tmp / f"esp/m1n1/boot.bin.before-{VERSION}"


class M3FlowTest(M3FlowBase):
    # M3 Pro on the handoff path

    def test_fresh_m3_pro_gets_the_handoff(self):
        self.mac("j516s")
        out = self.install().stdout
        self.assertIn("built-in display at its native resolution", out)
        self.assertNotIn("run:  aurora-touchid-setup", out)
        boot = self.boot.read_bytes()
        self.assertTrue(boot.startswith(b"M1N1:" + M1N1_BASE.encode() + b"\n"), boot[:80])
        self.assertTrue(boot.endswith(SWITCHES), boot[-200:])
        self.assertEqual([d for d in self.downloaded() if d.startswith("m1n1-")], [M1N1_PKG])
        self.assertEqual(self.kept_copy().read_bytes(), b"M1N1:original\n")
        self.assertEqual((self.state / "m3-mode").read_text().strip(), f"handoff {PRO_VARIANT}")
        self.assertFalse(self.update_conf.read_text().count("M1N1_UPDATE_DISABLED"))

        self.uninstall()
        boot = self.boot.read_bytes()
        self.assertTrue(boot.startswith(b"M1N1:m1n1-stock\n"), boot)
        self.assertNotIn(b"chosen.", boot)
        self.assertFalse(self.m1n1_conf.exists())
        self.assertFalse(self.state.exists())

    def test_m3_pro_over_an_11_36_kernel_only_install(self):
        self.mac("j516s", kernel="linux-aurora", bootbin=b"M1N1:original\n")
        self.update_conf.write_text(OUR_FREEZE)
        self.install()
        boot = self.boot.read_bytes()
        self.assertTrue(boot.startswith(b"M1N1:" + M1N1_BASE.encode() + b"\n"), boot[:80])
        self.assertTrue(boot.endswith(SWITCHES), boot)
        self.assertNotIn("M1N1_UPDATE_DISABLED", self.update_conf.read_text())

    def test_bringup_m3_pro_and_back(self):
        handbuilt = b"M1N1:bringup\nDTBS:hand\nUBOOT" + SWITCHES
        self.mac("j516s", kernel="linux-asahi", m1n1="m1n1-bringup", bootbin=handbuilt)
        self.update_conf.write_text(BRINGUP_FREEZE)
        marker = self.tmp / "etc/default/.update-m1n1.created-by-m3gpu"
        marker.touch()
        self.install()
        boot = self.boot.read_bytes()
        self.assertTrue(boot.startswith(b"M1N1:" + M1N1_BASE.encode() + b"\n"), boot[:80])
        self.assertTrue(boot.endswith(SWITCHES))
        # The hook ran while the bring-up freeze still held, so the first
        # rebuild is the installer's own, with the new m1n1 in place.
        log = self.log()
        self.assertLess(log.index("update-m1n1 frozen"), log.index("update-m1n1 rebuilt"))
        self.assertFalse(marker.exists())

        self.uninstall()
        self.assertEqual(self.boot.read_bytes(), handbuilt)
        self.assertEqual(self.update_conf.read_text(), BRINGUP_FREEZE)
        self.assertTrue(marker.exists())

    def test_trial_on_an_unlisted_m3_pro(self):
        self.mac("j514s")
        self.install(try_=1)
        self.assertTrue(self.boot.read_bytes().endswith(SWITCHES))
        self.assertTrue(self.kept_copy().exists())

    # M3s that stay kernel-only

    def assert_kernel_only(self, board, **kw):
        self.mac(board, **kw)
        before = self.boot.read_bytes()
        out = self.install().stdout
        self.assertIn("Touch ID is not supported", out)
        self.assertNotIn("run:  aurora-touchid-setup", out)
        self.assertEqual(self.boot.read_bytes(), before)
        self.assertIn("boot.bin is unchanged", out)
        self.assertFalse([d for d in self.downloaded() if d.startswith("m1n1-aurora-")])
        self.assertNotIn("update-m1n1 rebuilt", self.log())
        self.assertFalse(self.kept_copy().exists())
        self.assertEqual((self.state / "m3-mode").read_text().strip(), "kernel")
        self.assertIn("M1N1_UPDATE_DISABLED=1", self.update_conf.read_text())

        self.uninstall()
        self.assertEqual(self.boot.read_bytes(), before)
        self.assertNotIn("pacman -Sy --noconfirm --ask 4 linux-asahi linux-asahi-headers libfprint m1n1",
                         self.log())
        self.assertFalse(self.update_conf.exists())

    def test_unlisted_m3_pro_is_kernel_only(self):
        self.assert_kernel_only("j514s")

    def test_max_keeps_aurora12_bootloader_and_has_no_gpu_handoff(self):
        for board in ('j514c', 'j514m', 'j516c', 'j516m'):
            with self.subTest(board=board):
                self.assert_kernel_only(board, m1n1='m1n1-aurora-1.6.1.aurora12-1')
                self.assertNotIn('chosen.asahi,t6030-gpu=1', self.boot.read_bytes().decode())

    def test_kernel_only_pro_keeps_preexisting_aurora_bootloader_on_uninstall(self):
        self.assert_kernel_only('j516s', stub='15.6', m1n1='m1n1-aurora-1.6.1.aurora12-1')

    def test_max_without_mode_record_keeps_preexisting_aurora_bootloader(self):
        self.mac('j516c', m1n1='m1n1-aurora-1.6.1.aurora12-1')
        before = self.boot.read_bytes()
        self.install()
        (self.state / 'm3-mode').unlink()
        self.uninstall()
        self.assertEqual(self.boot.read_bytes(), before)
        self.assertNotIn('update-m1n1 rebuilt', self.log())

    def test_m3_is_kernel_only(self):
        # The J615; the J613 gets the display handoff by default (test_m3_air_default).
        self.assert_kernel_only("j615")

    def test_listed_m3_pro_on_another_stub_is_kernel_only(self):
        self.assert_kernel_only("j516s", stub="15.6")

    # Stage 1 (M3_STAGE1_VERSIONS) and the log-buffer overlap marker

    def test_listed_m3_pro_on_another_stage1_is_kernel_only(self):
        self.assert_kernel_only("j516s", stage1="v1.7.0")

    def test_listed_m3_pro_without_a_stage1_version_is_kernel_only(self):
        self.assert_kernel_only("j516s", stage1=None)

    def test_handoff_mac_on_another_stage1_stops_with_nothing_installed(self):
        # A Mac with the handoff from an earlier install (M3_TRY=1 from its
        # record) keeps its boot.bin as it is: nothing is downloaded or rebuilt.
        self.mac("j516s")
        self.install()
        self.assertEqual((self.state / "m3-mode").read_text().strip(), f"handoff {PRO_VARIANT}")
        self.mac("j516s", kernel="linux-aurora", bootbin=self.boot.read_bytes(), stage1="v1.7.0")
        before = self.boot.read_bytes()
        before_log = self.log()
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assert_kept_refusal(proc, "stage 1 is v1.7.0")
        self.assertEqual(self.boot.read_bytes(), before)
        self.assertNotIn("curl ", self.log()[len(before_log):])

    def assert_kept_refusal(self, proc, why):
        # No "keeping it" before the error, no flag the owner did not type,
        # and what to do next.
        err = " ".join(proc.stderr.split())
        self.assertIn(why, err)
        self.assertIn("Nothing was installed; this Mac keeps the boot loader and kernel it has", err)
        self.assertIn("issues/6", err)
        self.assertIn("bash -s -- --m3-report", err)
        self.assertNotIn("--m3-handoff", err)
        self.assertNotIn("keeping it", proc.stdout)

    def test_handoff_mac_on_another_stub_stops_with_nothing_installed(self):
        self.mac("j516s")
        self.install()
        self.mac("j516s", kernel="linux-aurora", bootbin=self.boot.read_bytes(), stub="15.6")
        before = self.boot.read_bytes()
        before_log = self.log()
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assert_kept_refusal(proc, "stub is 15.6")
        self.assertEqual(self.boot.read_bytes(), before)
        self.assertNotIn("curl ", self.log()[len(before_log):])

    def test_trial_on_another_stage1_is_refused(self):
        self.mac("j514s", stage1="v1.5.2")
        before = self.boot.read_bytes()
        proc = self.install(try_=1, check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("stage 1 is v1.5.2", proc.stderr)
        self.assertIn("Nothing was installed", proc.stderr)
        self.assertEqual(self.boot.read_bytes(), before)
        self.assertNotIn("pacman -U", self.log())

    def test_oslog_overlap_is_reported_after_a_boot(self):
        self.mac("j516s")
        self.install()
        (self.tmp / "dt/chosen/asahi,m1n1-oslog-overlap").write_bytes(b"\0" * 16)
        proc = self.install()
        self.assertIn("display log buffer overlaps", proc.stderr)
        self.assertIn("--m3-report", proc.stderr)
        (self.tmp / "dt/chosen/asahi,m1n1-oslog-overlap").unlink()
        self.assertNotIn("overlaps", self.install().stderr)

    def test_m1_ignores_the_m3_stage1_and_overlap_checks(self):
        # M1 and M2 are unchanged: no stage 1 gate, no overlap warning.
        self.mac("j314s", stage1=None)
        (self.tmp / "dt/chosen/asahi,m1n1-oslog-overlap").write_bytes(b"\0" * 16)
        proc = self.install()
        self.assertNotIn("stage 1", proc.stderr + proc.stdout)
        self.assertNotIn("overlaps", proc.stderr + proc.stdout)
        self.assertTrue(self.boot.read_bytes().startswith(b"M1N1:" + M1N1_BASE.encode() + b"\n"))

    def test_trial_refused_on_an_m3(self):
        # A plain M3 that isn't an Air (the Air's opt-in is in test_m3_air.py).
        self.mac("j504")
        before = self.boot.read_bytes()
        proc = self.install(try_=1, check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("Nothing was installed", proc.stderr)
        self.assertEqual(self.boot.read_bytes(), before)
        self.assertNotIn("pacman -U", self.log())

    # Later runs and failures (review of 5c7daa75)

    def test_rerun_without_the_flag_keeps_a_tried_handoff(self):
        self.mac("j514s")
        self.install(try_=1)
        proc = self.install()
        out = proc.stdout
        self.assertIn("keeping it", out)
        self.assertNotIn("--m3-handoff: trying", proc.stderr)
        self.assertEqual((self.state / "m3-mode").read_text().strip(), f"handoff {PRO_VARIANT}")
        self.assertTrue(self.boot.read_bytes().endswith(SWITCHES))
        self.assertNotIn("M1N1_UPDATE_DISABLED", self.update_conf.read_text())
        self.uninstall()
        self.assertTrue(self.boot.read_bytes().startswith(b"M1N1:m1n1-stock\n"))
        self.assertFalse(self.m1n1_conf.exists())

    def test_m3_pro_variant_shipped(self):
        # A release decision: change it with each m1n1 that changes what an
        # M3 Pro boots. 11.36.1 to 11.38 recorded t6030 (or nothing).
        self.assertRegex(PRO_VARIANT, r"^t6030-\S+$")

    def test_unlisted_pro_on_an_earlier_m1n1_needs_a_new_opt_in(self):
        # A J514S that took --m3-handoff on 11.37 or 11.38 runs this
        # release's plain one-liner: nothing moves it to the new m1n1.
        for recorded in ("handoff t6030", "handoff"):
            with self.subTest(recorded=recorded):
                self.fresh_state()
                self.mac("j514s")
                self.install(try_=1)
                (self.state / "m3-mode").write_text(recorded + "\n")
                (self.state / "m1n1-installed").unlink()
                before = self.boot.read_bytes()
                before_conf = self.m1n1_conf.read_bytes()
                log = self.log()
                proc = self.install(check=False)
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn("nobody has booted on this model yet", " ".join(proc.stderr.split()))
                self.assertIn("bash -s -- --m3-handoff", proc.stderr)
                self.assertIn("Nothing was installed", proc.stderr)
                self.assertNotIn("keeping it", proc.stdout)
                self.assertNotIn("--m3-handoff: trying", proc.stderr)
                self.assertEqual(self.boot.read_bytes(), before)
                self.assertEqual(self.m1n1_conf.read_bytes(), before_conf)
                self.assertNotIn("curl ", self.log()[len(log):])
                # Asked for, it moves; later plain runs keep it.
                self.install(try_=1)
                self.assertEqual((self.state / "m3-mode").read_text().strip(), f"handoff {PRO_VARIANT}")
                self.assertIn("keeping it", self.install().stdout)

    def test_listed_pro_on_an_earlier_m1n1_moves_with_the_release(self):
        self.mac("j516s")
        self.install()
        (self.state / "m3-mode").write_text("handoff t6030\n")
        self.install()
        self.assertEqual((self.state / "m3-mode").read_text().strip(), f"handoff {PRO_VARIANT}")
        self.assertTrue(self.boot.read_bytes().endswith(SWITCHES))

    def test_failed_install_then_uninstall_keeps_the_bringup(self):
        handbuilt = b"M1N1:bringup\nDTBS:hand\nUBOOT" + SWITCHES
        self.mac("j516s", m1n1="m1n1-bringup", bootbin=handbuilt)
        self.update_conf.write_text(BRINGUP_FREEZE)
        marker = self.tmp / "etc/default/.update-m1n1.created-by-m3gpu"
        marker.touch()
        self.extra_env = {"FAKE_FAIL_U": "1"}
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.extra_env = {}
        self.uninstall()
        self.assertEqual(self.boot.read_bytes(), handbuilt)
        self.assertEqual(self.update_conf.read_text(), BRINGUP_FREEZE)
        self.assertTrue(marker.exists())

    def test_failed_install_then_uninstall_keeps_omarchys_config(self):
        own = "export LC_ALL=C\n"
        self.mac("j516s")
        self.update_conf.write_text(own)
        self.extra_env = {"FAKE_FAIL_U": "1"}
        self.assertNotEqual(self.install(check=False).returncode, 0)
        self.extra_env = {}
        self.uninstall()
        self.assertEqual(self.update_conf.read_text(), own)

    def test_uninstall_after_an_11_36_kernel_only_install_and_update(self):
        # 11.36 recorded linux-asahi, then rewrote it to linux-aurora on a re-run.
        self.mac("j516s", kernel="linux-aurora")
        self.update_conf.write_text(OUR_FREEZE)
        (self.state / "previous-package").write_text("linux-aurora 7.1.12.aurora2-11.36-1\n")
        self.install()
        self.uninstall()
        self.assertIn("linux-asahi linux-asahi-headers libfprint m1n1", self.log())
        self.assertNotIn("-S --noconfirm --ask 4 linux-aurora", self.log())

    def test_previous_package_is_kept_across_updates(self):
        self.mac("j314s")
        self.install()
        self.install()
        self.assertTrue((self.state / "previous-package").read_text().startswith("linux-asahi"))

    def test_handoff_refuses_a_boot_bin_that_was_not_rebuilt(self):
        # A freeze update-m1n1 honours but the pattern check would miss, on a
        # hand-built boot.bin that already ends with the switches.
        handbuilt = b"M1N1:bringup\nDTBS:hand\nUBOOT" + SWITCHES
        self.mac("j516s", m1n1="m1n1-bringup", bootbin=handbuilt)
        self.update_conf.write_text(BRINGUP_FREEZE + ": ${M1N1_UPDATE_DISABLED:=1}\n")
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(self.boot.read_bytes(), handbuilt)

    def test_handoff_refuses_a_custom_m1n1_path(self):
        self.mac("j516s")
        self.update_conf.write_text("M1N1=/opt/my-m1n1.bin\n")
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("Nothing was installed", proc.stderr)
        self.assertNotIn("pacman -U", self.log())

    def test_empty_mode_file_does_not_break_uninstall(self):
        self.mac("j613")
        self.install()
        (self.state / "m3-mode").write_text("")
        self.uninstall()

    # M1/M2: the same m1n1 as every Mac, no switches, no M3 state, and the
    # boot.bin they booted with kept first

    def test_m1_pro_gets_the_one_m1n1(self):
        self.mac("j314s")
        proc = self.install()
        out = proc.stdout
        self.assertIn("run:  aurora-touchid-setup", out)
        boot = self.boot.read_bytes()
        self.assertTrue(boot.startswith(b"M1N1:" + M1N1_BASE.encode() + b"\n"), boot[:80])
        self.assertNotIn(b"chosen.", boot)
        self.assertEqual([d for d in self.downloaded() if d.startswith("m1n1-")], [M1N1_PKG])
        self.assertFalse((self.state / "m3-mode").exists())
        self.assertFalse(self.m1n1_conf.exists())
        # The safety net every Mac whose boot loader changes gets.
        self.assertEqual(self.kept_copy().read_bytes(), b"M1N1:original\n")
        self.assertIn("put the\n    boot loader it booted with back from macOS", proc.stderr)
        self.assertIn("m1n1-aurora " + M1N1_BASE[len("m1n1-aurora-"):], proc.stderr)
        self.uninstall()
        self.assertTrue(self.boot.read_bytes().startswith(b"M1N1:m1n1-stock\n"))

    def test_every_mac_gets_the_same_m1n1(self):
        # M1, M2 and the M3s on the handoff path: one package, and only the
        # switches differ.
        for board, try_, switches in [("j293", 0, b""), ("j314s", 0, b""), ("j414s", 0, b""),
                                      ("j516s", 0, SWITCHES), ("j514s", 1, SWITCHES)]:
            with self.subTest(board=board):
                self.fresh_state()
                self.mac(board)
                self.install(try_=try_)
                boot = self.boot.read_bytes()
                self.assertTrue(boot.startswith(b"M1N1:" + M1N1_BASE.encode() + b"\n"), boot[:80])
                self.assertTrue(boot.endswith(b"UBOOT" + switches), boot[-200:])
                self.assertEqual([d for d in self.downloaded() if d.startswith("m1n1-")], [M1N1_PKG])

    # The m1n1 by its bytes (M1N1_BIN_SHA), never by the version it reports

    def test_m1n1_is_checked_and_recorded_by_its_bytes(self):
        sha = self.bin_shas[M1N1_PKG]
        size = len((self.tmp / ("root-" + M1N1_PKG) / "usr/lib/asahi-boot/m1n1.bin").read_bytes())
        for board in ("j314s", "j516s"):
            with self.subTest(board=board):
                self.fresh_state()
                self.mac(board)
                out = self.install().stdout
                self.assertIn(f"starts with this release's m1n1 (sha256 {sha})", out)
                self.assertEqual((self.state / "m1n1-installed").read_text(), f"{sha} {size} {M1N1_PKG}\n")

    def test_a_package_with_other_m1n1_bytes_is_refused(self):
        # Same file name and version, other bytes: refused before anything changes.
        for board in ("j314s", "j516s"):
            with self.subTest(board=board):
                self.fresh_state()
                self.mac(board)
                before = self.boot.read_bytes()
                proc = self.install(env="M1N1_BIN_SHA=" + "0" * 64, check=False)
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn(f"holds an m1n1.bin with sha256 {self.bin_shas[M1N1_PKG]}", proc.stderr)
                self.assertIn("Nothing was installed", proc.stderr)
                self.assertEqual(self.boot.read_bytes(), before)
                self.assertNotIn("pacman -U", self.log())
                self.assertFalse(self.kept_copy().exists())

    def test_a_rebuild_with_other_m1n1_bytes_stops(self):
        # update-m1n1 rebuilt boot.bin, but not from this release's m1n1.
        other = self.tmp / "other-m1n1.bin"
        other.write_bytes(b"M1N1:" + M1N1_BASE.encode() + b"\n" + b"\0" * 64)
        self.mac("j314s")
        self.extra_env = {"FAKE_OTHER_M1N1": str(other)}
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("not this release's", proc.stderr)
        self.assertIn(f"boot.bin.before-{VERSION}", proc.stderr)
        self.assertFalse((self.state / "m1n1-installed").exists())

    def test_frozen_m1_keeps_its_boot_bin_unchecked(self):
        # As in 11.38: a freeze the owner set is reported, and nothing checks
        # a boot.bin that was not rebuilt.
        self.mac("j314s")
        self.update_conf.write_text("M1N1_UPDATE_DISABLED=1\n")
        proc = self.install()
        self.assertIn("was not rebuilt", proc.stderr)
        self.assertEqual(self.boot.read_bytes(), b"M1N1:original\n")
        self.assertFalse((self.state / "m1n1-installed").exists())

    # The MacBook Neo keeps its own m1n1 until NEO_AURORA_M1N1 is 1

    NEO_CONF = "M1N1=__NEO__\nU_BOOT=/usr/lib/neo/u-boot.bin\n"

    def neo(self):
        own = self.tmp / "neo-m1n1.bin"
        own.write_bytes(b"M1N1:neo-own\n")
        self.mac("j700", m1n1="m1n1-stock")
        conf = self.NEO_CONF.replace("__NEO__", str(own))
        self.update_conf.write_text(conf)
        return conf

    def test_neo_keeps_its_own_m1n1(self):
        conf = self.neo()
        out = self.install().stdout
        self.assertIn("Keeping this MacBook Neo's own m1n1", out)
        self.assertEqual([d for d in self.downloaded() if d.startswith("m1n1-")], [])
        self.assertTrue(self.boot.read_bytes().startswith(b"M1N1:neo-own\n"))
        self.assertIn(conf, self.update_conf.read_text())
        self.assertFalse(self.kept_copy().exists())

    def test_neo_on_the_one_m1n1_with_the_switch(self):
        conf = self.neo()
        proc = self.install(env="NEO_AURORA_M1N1=1")
        self.assertNotIn("own m1n1", proc.stdout)
        self.assertEqual([d for d in self.downloaded() if d.startswith("m1n1-")], [M1N1_PKG])
        self.assertTrue(self.boot.read_bytes().startswith(b"M1N1:" + M1N1_BASE.encode() + b"\n"))
        text = self.update_conf.read_text()
        self.assertNotIn("M1N1=", text.replace("M1N1_UPDATE", ""))
        self.assertIn("U_BOOT=/usr/lib/neo/u-boot.bin", text)
        self.assertEqual(self.kept_copy().read_bytes(), b"M1N1:original\n")
        self.run_sh("NEO_AURORA_M1N1=1\nuninstall_all")
        self.assertEqual(self.update_conf.read_text(), conf)
        self.assertTrue(self.boot.read_bytes().startswith(b"M1N1:neo-own\n"))
        self.assertIn("libfprint m1n1", self.log())

    # Package database refresh and pending upgrades

    def test_pending_upgrades_stop_the_install_before_anything_changes(self):
        self.mac("j516s")
        self.extra_env["FAKE_UPGRADES"] = "llvm-libs 20.1.8-1 -> 21.1.2-1\nmesa 1:25.2.4-1 -> 1:25.2.5-1"
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        err = " ".join(proc.stderr.split())
        self.assertIn("2 package upgrade(s) pending (pacman -Qu; first: llvm-libs mesa)", err)
        self.assertIn("Nothing was installed", err)
        log = self.log()
        self.assertLess(log.index("pacman -Sy --noconfirm"), log.index("pacman -Qu"))
        self.assertNotIn("pacman -U", log)
        self.assertNotIn("pacman -S ", log)
        self.assertEqual(self.downloaded(), [])
        self.assertEqual(self.boot.read_bytes(), b"M1N1:original\n")
        self.assertFalse((self.state / "m3-mode").exists())

    def test_held_packages_do_not_count_as_pending(self):
        self.mac("j516s")
        self.extra_env["FAKE_UPGRADES"] = "libfprint 1.94.9-1 -> 1.94.10-1 [ignored]"
        self.install()
        self.assertIn("pacman -U", self.log())

    def test_held_mac_image_packages_do_not_block_install(self):
        self.mac("j613")
        self.extra_env["FAKE_UPGRADES"] = "\n".join(
            name + " 1-1 -> 2-1 [ignored]"
            for name in ("omarchy", "omarchy-mac", "omarchy-mac-boot", "omarchy-settings"))
        self.install()
        self.assertIn("pacman -U", self.log())

    def test_held_mac_image_packages_do_not_hide_pending_dependency(self):
        self.mac("j613")
        self.extra_env["FAKE_UPGRADES"] = (
            "omarchy-mac 1-1 -> 2-1 [ignored]\nllvm-libs 20.1-1 -> 22.1-1")
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("first: llvm-libs", proc.stderr)
        self.assertNotIn("pacman -U", self.log())
        self.assertEqual(self.downloaded(), [])

    def omarchy_arm(self, *targets):
        # Omarchy on ARM's helper naming the packages it installs from its own repository.
        helper = self.tmp / "omarchy/arm-package-sources.sh"
        helper.parent.mkdir(exist_ok=True)
        helper.write_text("omarchy_arm_package_targets() {\n  printf '%s\\n' " + " ".join(targets) + "\n}\n")

    def test_omarchy_arm_repository_pins_do_not_block_install(self):
        self.mac("j313")
        self.omarchy_arm("omarchy/hyprland", "omarchy/hyprtoolkit", "omarchy/hyprland-guiutils")
        self.extra_env["FAKE_REPO_LIST"] = (
            "omarchy hyprland 0.56.2-5 [installed]\nomarchy hyprtoolkit 0.5.4-5.1 [installed]\n"
            "omarchy hyprland-guiutils 0.2.2-3 [installed]")
        self.extra_env["FAKE_UPGRADES"] = (
            "hyprland-guiutils 0.2.2-3 -> 0.2.2-4\nhyprtoolkit 0.5.4-5.1 -> 0.6.0-1")
        self.install()
        self.assertIn("pacman -U", self.log())

    def test_omarchy_arm_repository_pins_do_not_hide_other_upgrades(self):
        self.mac("j313")
        self.omarchy_arm("omarchy/hyprtoolkit", "omarchy/hyprland-guiutils")
        # hyprland-guiutils has a newer build in Omarchy's repository, so 'omarchy update' installs it.
        self.extra_env["FAKE_REPO_LIST"] = (
            "omarchy hyprtoolkit 0.5.4-5.1 [installed]\n"
            "omarchy hyprland-guiutils 0.2.2-4 [installed: 0.2.2-3]")
        self.extra_env["FAKE_UPGRADES"] = (
            "hyprland-guiutils 0.2.2-3 -> 0.2.2-4\nhyprtoolkit 0.5.4-5.1 -> 0.6.0-1\n"
            "llvm-libs 20.1-1 -> 22.1-1")
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        err = " ".join(proc.stderr.split())
        self.assertIn("2 package upgrade(s) pending (pacman -Qu; first: hyprland-guiutils llvm-libs)", err)
        self.assertNotIn("pacman -U", self.log())
        self.assertEqual(self.downloaded(), [])

    def test_omarchy_arm_pins_need_omarchys_helper(self):
        self.mac("j313")
        self.extra_env["FAKE_REPO_LIST"] = "omarchy hyprtoolkit 0.5.4-5.1 [installed]"
        self.extra_env["FAKE_UPGRADES"] = "hyprtoolkit 0.5.4-5.1 -> 0.6.0-1"
        helper = self.tmp / "omarchy/arm-package-sources.sh"
        # No helper, then one that fails: nothing is pinned and the usual refusal follows.
        for body in (None, "exit 3\n"):
            with self.subTest(helper=body):
                if body is not None:
                    helper.parent.mkdir(exist_ok=True)
                    helper.write_text(body)
                proc = self.install(check=False)
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn("1 package upgrade(s) pending (pacman -Qu; first: hyprtoolkit)",
                              " ".join(proc.stderr.split()))

    def test_omarchy_arm_pin_in_an_unknown_repository_is_skipped(self):
        self.mac("j313")
        # Last, as the loop's status is its last target's.
        self.omarchy_arm("omarchy/hyprtoolkit", "gone/hyprland")
        self.extra_env["FAKE_REPO_LIST"] = "omarchy hyprtoolkit 0.5.4-5.1 [installed]"
        self.extra_env["FAKE_UPGRADES"] = "hyprtoolkit 0.5.4-5.1 -> 0.6.0-1"
        self.install()
        self.assertIn("pacman -U", self.log())

    def test_database_errors_refuse_before_download_or_install(self):
        self.mac("j516s")
        for failure in ("FAKE_FAIL_REFRESH", "FAKE_FAIL_QUERY"):
            with self.subTest(failure=failure):
                self.extra_env = {failure: "1"}
                proc = self.install(check=False)
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn("Nothing was installed", proc.stderr)
                self.assertNotIn("pacman -U", self.log())
                self.assertEqual(self.downloaded(), [])
                self.assertEqual(self.boot.read_bytes(), b"M1N1:original\n")

    def test_ignored_non_candidate_upgrade_still_requires_full_update(self):
        self.mac("j516s")
        self.extra_env["FAKE_UPGRADES"] = "llvm-libs 20.1-1 -> 22.1-1 [ignored]"
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("omarchy update", proc.stderr)
        self.assertNotIn("pacman -U", self.log())

    def test_frozen_image_installs_one_admitted_local_transaction(self):
        self.mac("j516s")
        self.extra_env["FAKE_HOLDS"] = "*"
        self.extra_env["FAKE_UPGRADES"] = "\n".join(
            f"held-{i} 1-1 -> 2-1 [ignored]" for i in range(108))
        self.install(env='''
frozen_dependency_prepare() {
  echo "missing dependency plan" >>"$FAKE/log"
  FROZEN_TRANSACTION_CONFIG="$work/transaction.conf"
  FROZEN_TRANSACTION_FILES=("$work"/*.pkg.tar.zst)
  printf '[options]\\nIgnorePkg = *\\n' >"$FROZEN_TRANSACTION_CONFIG"
}
snapshot() { echo snapshot >>"$FAKE/log"; }
''')
        log = self.log()
        self.assertLess(log.index("missing dependency plan"), log.index("snapshot"))
        transactions = [row for row in log.splitlines() if row.startswith("pacman -U")]
        self.assertEqual(len(transactions), 1)
        self.assertIn("--config", transactions[0])
        self.assertIn(self.pro_mesa, transactions[0])
        self.assertNotIn("pacman -S ", log)

    def test_frozen_image_dependency_refusal_precedes_boot_changes(self):
        self.mac("j613")
        self.extra_env["FAKE_HOLDS"] = "*"
        self.extra_env["FAKE_UPGRADES"] = "llvm-libs 20-1 -> 22-1 [ignored]"
        before = self.boot.read_bytes()
        proc = self.install(check=False, env='''
frozen_dependency_prepare() { die "dependency would upgrade installed provider"; }
snapshot() { echo snapshot >>"$FAKE/log"; }
''')
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("installed provider", proc.stderr)
        self.assertNotIn("snapshot", self.log())
        self.assertNotIn("pacman -U", self.log())
        self.assertEqual(self.boot.read_bytes(), before)
        self.assertFalse(self.m1n1_conf.exists())

    def test_frozen_image_does_not_hide_unheld_upgrades(self):
        self.mac("j516s")
        self.extra_env["FAKE_HOLDS"] = "*"
        self.extra_env["FAKE_UPGRADES"] = "llvm-libs 20-1 -> 22-1"
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("omarchy update", proc.stderr)
        self.assertEqual(self.downloaded(), [])

    def test_package_hold_configuration_error_refuses_before_changes(self):
        self.mac("j516s")
        self.extra_env["FAKE_FAIL_CONFIG"] = "1"
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("could not read package holds", proc.stderr)
        self.assertEqual(self.downloaded(), [])
        self.assertNotIn("pacman -U", self.log())

    def test_partial_hold_pattern_does_not_enter_frozen_route(self):
        self.mac("j516s")
        self.extra_env["FAKE_HOLDS"] = "linux-*"
        self.extra_env["FAKE_UPGRADES"] = "llvm-libs 20-1 -> 22-1 [ignored]"
        proc = self.install(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("omarchy update", proc.stderr)
        self.assertEqual(self.downloaded(), [])

    def test_frozen_preparation_binds_requirements_and_explicit_archive_list(self):
        self.mac("j516s")
        proc = self.run_sh('''
work="$FAKE/preparation"
mkdir -p "$work"
touch "$work/$(m3_pro_mesa_file)" "$work/unrelated.pkg.tar.zst" "$work/transaction.conf"
python3() {
  if [[ $1 == "$work/frozen-dependencies.py" ]]; then
    printf 'planner-arg:%s\\n' "$@" >>"$FAKE/log"
    printf '{"transaction_config":"%s/transaction.conf","candidate_sha256":{"%s/%s":"fixture"},"dependencies":[{"file":"%s/approved.pkg.tar.gz"}]}' "$work" "$work" "$(m3_pro_mesa_file)" "$work"
  else command python3 "$@"; fi
}
FROZEN_PACKAGES=1
M3_PRO_MESA_NEEDS='glibc>=2.43 libgcc>=3.0'
M3_GPU_EXPERIMENT=1
frozen_dependency_prepare "$work/$(m3_pro_mesa_file)"
printf 'admitted-file:%s\\n' "${FROZEN_TRANSACTION_FILES[@]}" >>"$FAKE/log"
M3_GPU_EXPERIMENT=0
m3_install_packages
''')
        log = self.log()
        for need in ("fprintd", "glibc>=2.43", "libgcc>=3.0", "python", "vulkan-icd-loader"):
            self.assertIn("planner-arg:--require\nplanner-arg:" + need, log)
        transaction = next(row for row in log.splitlines() if row.startswith("pacman -U"))
        self.assertIn("approved.pkg.tar.gz", transaction)
        self.assertNotIn("unrelated.pkg.tar.zst", transaction)

    def test_the_refresh_comes_before_any_download(self):
        self.mac("j516s")
        self.install()
        log = self.log()
        self.assertLess(log.index("pacman -Sy --noconfirm"), log.index("curl "))
        self.assertEqual(log.count("pacman -Sy --noconfirm\n"), 1)

    # Limine --uninstall after the persistent GPU route retained the previous kernel

    def test_limine_uninstall_removes_the_retained_gpu_off_entry(self):
        self.mac("j516s")
        self.install()
        helper = self.tmp / "bin/fake-boot-profile"
        # Run as "python3 HELPER remove ...", as the real one is.
        helper.write_text("import os, sys\n"
                          "with open(os.environ['FAKE'] + '/log', 'a') as log:\n"
                          "    print('boot-profile', *sys.argv[1:], file=log)\n")
        helper.chmod(0o755)
        (self.state / "m3-known-entry.json").write_text("{}\n")
        self.run_sh(f"""
M3_BOOT_PROFILE_HELPER='{helper}'
m3_boot_profile_install() {{ echo "boot-profile install" >>"$FAKE/log"; }}
systemctl() {{ echo "systemctl $*" >>"$FAKE/log"; }}
uninstall_all
""")
        log = self.log()
        self.assertIn(f"boot-profile remove --esp {self.tmp}/esp --state {self.state}\n", log)
        self.assertLess(log.index("boot-profile install"), log.index("boot-profile remove"))
        self.assertIn("systemctl disable aurora-sep-fallback-modules.service", log)
        self.assertFalse(helper.exists())
        self.assertFalse(self.state.exists())

    def test_failed_limine_remove_check_preserves_packages_and_recovery(self):
        self.mac("j516s")
        self.install()
        (self.state / "m3-known-entry.json").write_text("{}\n")
        modules = self.state / "modules-kept"
        modules.mkdir()
        (modules / "modules.dep").write_text("retained")
        before = self.boot.read_bytes()
        (self.fake / "log").write_text("")
        helper = self.tmp / "bin/failing-boot-profile"
        helper.write_text("import sys; sys.exit(1)\n")
        proc = self.run_sh(f"""
M3_BOOT_PROFILE_HELPER='{helper}'
m3_boot_profile_install() {{ :; }}
uninstall_all
""", check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("recovery state and modules were kept", proc.stderr)
        self.assertNotRegex(self.log(), r"pacman -(?:S|U|R)")
        self.assertTrue((self.state / "m3-known-entry.json").exists())
        self.assertEqual((modules / "modules.dep").read_text(), "retained")
        self.assertEqual(self.boot.read_bytes(), before)

    def test_limine_fallback_without_ownership_refuses_uninstall(self):
        self.mac("j516s")
        self.install()
        esp=self.tmp / "esp"
        (esp / "EFI/BOOT").mkdir(parents=True)
        (esp / "EFI/Linux").mkdir()
        (esp / "EFI/BOOT/BOOTAA64.EFI").write_bytes(b"limine.conf")
        conf=esp / "EFI/BOOT/limine.conf"
        conf.write_text("/Aurora previous (GPU off)\n protocol: efi\n path: boot():/EFI/Linux/aurora-m3-previous-aaaaaaaaaaaaaaaa.efi\n cmdline: root=UUID=abc mesa_m3=off\n")
        retained=esp / "EFI/Linux/aurora-m3-previous-aaaaaaaaaaaaaaaa.efi"
        retained.write_bytes(b"retained kernel")
        defaults=self.tmp / "defaults"
        defaults.write_text("ENABLE_ENROLL_LIMINE_CONFIG=no\n")
        before=conf.read_bytes()
        (self.fake / "log").write_text("")
        proc=self.run_sh(f"""
M3_BOOT_PROFILE_HELPER='{INSTALLER.parent / 'm3-boot-profile.py'}'
m3_boot_profile_install() {{ :; }}
python3() {{ command python3 "$@" --defaults '{defaults}' --lock '{self.tmp / 'lock1'}' --lock '{self.tmp / 'lock2'}'; }}
uninstall_all
""",check=False)
        self.assertNotEqual(proc.returncode,0)
        self.assertIn("custom fallback has no saved ownership record",proc.stderr)
        self.assertNotRegex(self.log(),r"pacman -(?:S|U|R)")
        self.assertEqual(conf.read_bytes(),before)
        self.assertEqual(retained.read_bytes(),b"retained kernel")
        self.assertTrue(self.state.exists())
        conf.write_text(conf.read_text().replace('/Aurora previous (GPU off)', '/Renamed recovery').replace(' path:', ' PATH:'))
        before=conf.read_bytes()
        proc=self.run_sh(f"""
M3_BOOT_PROFILE_HELPER='{INSTALLER.parent / 'm3-boot-profile.py'}'
m3_boot_profile_install() {{ :; }}
python3() {{ command python3 "$@" --defaults '{defaults}' --lock '{self.tmp / 'lock1'}' --lock '{self.tmp / 'lock2'}'; }}
uninstall_all
""",check=False)
        self.assertNotEqual(proc.returncode,0)
        self.assertIn("custom fallback has no saved ownership record",proc.stderr)
        self.assertNotRegex(self.log(),r"pacman -(?:S|U|R)")
        self.assertEqual(conf.read_bytes(),before)
        self.assertTrue(self.state.exists())


    def test_pending_upgrades_refuse_uninstall_before_package_or_boot_changes(self):
        self.mac("j516s")
        self.install()
        before=self.boot.read_bytes()
        (self.fake / "log").write_text("")
        self.extra_env["FAKE_UPGRADES"]="llvm-libs 20-1 -> 22-1"
        proc=self.run_sh("uninstall_all",check=False)
        self.assertNotEqual(proc.returncode,0)
        self.assertIn("omarchy update",proc.stderr)
        self.assertNotRegex(self.log(),r"pacman -(?:S |U|R)")
        self.assertEqual(self.boot.read_bytes(),before)
        self.assertTrue(self.state.exists())

    def test_limine_uninstall_without_a_retained_entry_leaves_limine_alone(self):
        self.mac("j516s")
        self.install()
        self.run_sh("""
m3_boot_profile_install() { echo "boot-profile install" >>"$FAKE/log"; }
systemctl() { echo "systemctl $*" >>"$FAKE/log"; }
uninstall_all
""")
        self.assertNotIn("boot-profile", self.log())
        self.assertNotIn("aurora-sep-fallback-modules", self.log())

    # EFI partition space before the persistent GPU route

    def esp_check(self, avail, chain="limine"):
        return self.run_sh(f"""
boot_chain() {{ echo {chain}; }}
df() {{ printf 'Avail\\n%s\\n' {avail}; }}
m3_esp_space_check
""", check=False)

    def test_a_full_esp_stops_the_persistent_route(self):
        self.mac("j613")
        uki = self.tmp / "esp/EFI/Linux/omarchy_linux-aurora.efi"
        uki.parent.mkdir(parents=True)
        with uki.open("wb") as f:
            f.truncate(60 * 1048576)
        # Twice the 60 MB UKI (retained and new), twice boot.bin, and 16 MB.
        proc = self.esp_check(135)
        self.assertNotEqual(proc.returncode, 0)
        err = " ".join(proc.stderr.split())
        self.assertIn(f"The EFI partition ({self.tmp}/esp) has 135 MB free".lower(), err.lower())
        self.assertIn("about 136 MB", err)
        self.assertIn("Nothing was installed", err)
        self.assertEqual(self.esp_check(136).returncode, 0)

    def test_an_esp_without_a_uki_assumes_a_100_mb_kernel(self):
        self.mac("j613")
        self.assertNotEqual(self.esp_check(205).returncode, 0)
        self.assertEqual(self.esp_check(206).returncode, 0)

    def test_grub_needs_esp_room_for_boot_bin_only(self):
        self.mac("j613")
        self.assertEqual(self.esp_check(16, chain="grub").returncode, 0)
        self.assertNotEqual(self.esp_check(15, chain="grub").returncode, 0)

    def test_missing_or_unmeasurable_esp_refuses(self):
        self.mac("j613")
        for override in ("esp_bootbin() { return 1; }", "stat() { return 1; }", "find() { return 1; }"):
            with self.subTest(override=override):
                proc=self.run_sh(override+"\nm3_esp_space_check",check=False)
                self.assertNotEqual(proc.returncode,0)
                self.assertIn("Nothing was installed",proc.stderr)

    def test_unreadable_free_space_refuses(self):
        self.mac("j613")
        proc = self.esp_check("''")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("could not read the free space", proc.stderr)


if __name__ == "__main__":
    unittest.main()
