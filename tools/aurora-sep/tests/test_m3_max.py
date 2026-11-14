"""M3 bring-up from the one-liner, on any M3 and on the M3 Max (t6031, t6034) in particular.

Runs against the fake Mac of test_m3_flow, with a fake device tree, sysfs, procfs and debugfs, and
stubbed commands (dmesg, journalctl, lspci, lsusb, uname, hostname, getent, id, modprobe, taskset).

ReportTest: --m3-report on t6031, t6034, t6030 and t8122: what each file of the tgz holds.
PrivacyTest: the masking, and the check that refuses a file that still has a host name, a user
name, a serial number or a MAC address (and an owner or a file it should not have).
AdtTest: the on-demand ADT read (Dave's contract r2): phram loaded only when it isn't, only the
read-only node of the device named adt that matches its region, unloaded on every path (an error
in the reader and an interruption included), and the MTD inventory before and after compared.
NextStepsTest: the block that ends an install on the M3s with no handoff path yet.
SurveyTest: --m3-power-survey: refused on M1 and M2, its phases in order, the CPU split, the load
stopped by the traps (Ctrl-C too), the early stop on temperature, and a t6034 without SMC keys.
SameAs122Test: on every board, an install and an uninstall run exactly the commands 12.2's script
ran, with the same files; only the summary may differ, by the block at its end.
"""
from pathlib import Path
from unittest import mock
import hashlib
import os
import re
import shutil
import signal
import struct
import subprocess
import tarfile
import tempfile
import time
import unittest

import test_m3_flow as flow
import test_m3_pro_mesa as pro
from test_m3_air_default import normalize_undo_label

SRC = flow.SRC
VERSION = flow.VERSION
REL_12_2 = "26cdc069"
KREL = re.sub(r"^(\d+\.\d+\.\d+)\.aurora(\d+)-(.+)$", r"\1-\2-\3-sep-ARCH", VERSION)
# Boards the flow harness doesn't have: the M3 Max (14" and 16", t6031 and t6034) and an iMac.
EXTRA_BOARDS = {
    "j514c": ["apple,j514c", "apple,t6031", "apple,arm-platform"],
    "j516c": ["apple,j516c", "apple,t6031", "apple,arm-platform"],
    "j514m": ["apple,j514m", "apple,t6034", "apple,arm-platform"],
    "j516m": ["apple,j516m", "apple,t6034", "apple,arm-platform"],
    "j433": ["apple,j433", "apple,t8122", "apple,arm-platform"],
}
MODELS = {
    "j514c": "Apple MacBook Pro (14-inch, M3 Max, Nov 2023)",
    "j516c": "Apple MacBook Pro (16-inch, M3 Max, Nov 2023)",
    "j514m": "Apple MacBook Pro (14-inch, M3 Max, Nov 2023)",
    "j516m": "Apple MacBook Pro (16-inch, M3 Max, Nov 2023)",
    "j516s": "Apple MacBook Pro (16-inch, M3 Pro, Nov 2023)",
    "j504": "Apple MacBook Pro (14-inch, M3, Nov 2023)",
    "j613": "Apple MacBook Air (13-inch, M3, 2024)",
}

# What must never leave the Mac, planted all over the fake one.
HOST = "myhost-4711"
FQDN = "myhost-4711.example.lan"
USERS = ("alice", "bob")
SERIAL = "C02ZK1ABCDEF"
BATTERY_SERIAL = "F5D1234567ABC"
USB_SERIAL = "USBSER12345"
MAC = "12:34:56:78:9a:bc"
BT = "a1:b2:c3:d4:e5:f6"
SECRETS = [HOST, FQDN, *USERS, SERIAL, BATTERY_SERIAL, USB_SERIAL, MAC, MAC.replace(":", ""),
           BT, "f6e5d4c3b2a1"]

SMC_KEYS = """\
# 7 keys: index key type size flags [value: T* in mC, P* in mW]
   0 Tp0a flt    4 80 45000
   1 Te05 flt    4 80 40000
   2 Tg0a flt    4 80 -2850
   3 PSTR flt    4 80 4320
   4 RPlt ch8*   8 80
   5 TB0T flt    4 80 30000
   6 PP0b flt    4 80 230
"""

JOURNAL = f"""\
[    0.000000] {HOST} kernel: Booting Linux on physical CPU 0x0000010100 [0x611f0451]
[    0.000000] {HOST} kernel: Linux version {KREL} (linux-aurora@archlinux)
[    0.000000] {HOST} kernel: Machine model: Apple MacBook Pro (14-inch, M3 Max, Nov 2023)
[    0.010000] {HOST} kernel: apple-t6031-pmgr: probing the M3 Max power manager
[    0.055969] {HOST} kernel: [drm] Initialized simpledrm 1.0.0 for 10000000.framebuffer on minor 0
[    2.100000] {HOST} kernel: usb 1-1: SerialNumber: {USB_SERIAL}
[    3.000000] {HOST} kernel: brcmfmac: wlan0 address {MAC}
[    3.100000] {HOST} kernel: Bluetooth: hci0: BD address {BT.upper()}
[    3.200000] {HOST} kernel: audit: exe=/home/alice/bin/tool comm=tool
[    3.300000] {HOST} kernel: some unrelated line from {FQDN} for bob
[    3.400000] {HOST} kernel: smbios: system serial {SERIAL} raw 123456789ABC
"""
DMESG = """\
[    0.010000] apple-t6031-pmgr: probing the M3 Max power manager
[    0.055969] [drm] Initialized simpledrm 1.0.0 for 10000000.framebuffer on minor 0
[    4.000000] some unrelated line
"""

STUBS = {
    "dmesg": '#!/bin/sh\ncat "$FAKE/dmesg.txt"\n',
    "journalctl": '#!/bin/sh\n[ -f "$FAKE/journal.txt" ] && cat "$FAKE/journal.txt"\nexit 0\n',
    "lsusb": ('#!/bin/sh\nif [ "$1" = -t ]; then echo "/:  Bus 001.Port 001: Dev 001, Class=root_hub, '
              'Driver=xhci-hcd/1p, 5000M"; else echo "Bus 001 Device 002: ID 05ac:0281 Apple, Inc. keyboard"; fi\n'),
    "lspci": '#!/bin/sh\necho "0000:01:00.0 Network controller [0280]: Broadcom BCM4388 [14e4:4433]"\n',
    "uname": '#!/bin/sh\n[ "$*" = -n ] && { echo "$FAKE_HOST"; exit 0; }\nexec /usr/bin/uname "$@"\n',
    "hostname": '#!/bin/sh\necho "$FAKE_HOST.example.lan"\n',
    "hostnamectl": '#!/bin/sh\necho "$FAKE_HOST"\n',
    # The passwd list and user of these tests; the render group and group membership as the flow
    # fixture's own stubs give them (the M3 Pro's Mesa adds the desktop user to render).
    "getent": ('#!/bin/sh\n[ "$1" = passwd ] && { printf "root:x:0:0::/root:/bin/bash\\n'
               'alice:x:1000:1000:Alice:/home/alice:/bin/bash\\nbob:x:1001:1001::/home/bob:/bin/bash\\n'
               'nobody:x:65534:65534::/:/usr/bin/nologin\\n"; exit 0; }\n'
               '[ "$1" = group ] && [ "$2" = render ] && { [ -e "$FAKE/no-render-group" ] && exit 2; '
               'echo "render:x:989:"; exit 0; }\nexec /usr/bin/getent "$@"\n'),
    "id": ('#!/bin/bash\n[[ $* == -un ]] && { echo alice; exit 0; }\n'
           'if [[ ${1:-} == -nG ]]; then shift; [[ ${1:-} == -- ]] && shift; u=${1:-alice}\n'
           '  line=$(grep -m1 "^$u " "$FAKE/groups" 2>/dev/null) || line=""; echo "$u${line#"$u"}"; exit 0; fi\n'
           'exec /usr/bin/id "$@"\n'),
    "udevadm": '#!/bin/sh\necho "udevadm $*" >>"$FAKE/log"\n',
    # phram as the kernel's: loading it binds the two reserved-memory regions and makes their MTD
    # devices (mtd1 adt, mtd2 m1n1_stage2.log) and nodes; unloading removes them. As in the
    # kernel, an MTD device has its own of_node link and no parent device (FAKE_ADT_NODE binds adt
    # to another region).
    "modprobe": r"""#!/bin/bash
echo "modprobe $*" >>"$FAKE/log"
S=$M3T/sys
case "$*" in
  phram)
    if [[ -n ${FAKE_NO_PHRAM:-} ]]; then echo "modprobe: FATAL: Module phram not found" >&2; exit 1; fi
    mkdir -p "$S/module/phram"
    [[ -n ${FAKE_NO_ADT_MTD:-} ]] && exit 0
    n=1
    for r in "adt ${FAKE_ADT_NODE:-10003528000} ${FAKE_ADT_SIZE:-507904}" "m1n1_stage2.log 108d994c000 16384"; do
      read -r name addr size <<<"$r"
      m=$S/class/mtd/mtd$n
      mkdir -p "$m" "$S/class/mtd/mtd${n}ro"
      echo "$name" >"$m/name"; echo ram >"$m/type"; echo "$size" >"$m/size"; echo 4096 >"$m/erasesize"
      echo "90:$((n * 2))" >"$m/dev"
      ln -sfn "$M3T/dt/reserved-memory/flash@$addr" "$m/of_node"
      touch "$M3T/dev/mtd$n" "$M3T/dev/mtd${n}ro"
      n=$((n + 1))
    done
    # FAKE_ADT_FILE: what the adt region holds, seen through its read-only node.
    [[ -n ${FAKE_ADT_FILE:-} ]] && cp "$FAKE_ADT_FILE" "$M3T/dev/mtd1ro"
    if [[ -n ${FAKE_EXTRA_MTD:-} ]]; then
      mkdir -p "$S/class/mtd/mtd3"; echo other >"$S/class/mtd/mtd3/name"
    fi ;;
  "-r phram")
    if [[ -n ${FAKE_FAIL_UNLOAD:-} ]]; then echo "modprobe: FATAL: Module phram is in use." >&2; exit 1; fi
    rm -rf "$S/module/phram" "$S"/class/mtd/mtd[123] "$S"/class/mtd/mtd[12]ro "$M3T"/dev/mtd[12] "$M3T"/dev/mtd[12]ro ;;
esac
exit 0
""",
    # A pinned busy loop stands in as a sleep: its pid is recorded, so a test can see it end.
    # With FAKE_HOT, the first load makes the SMC's P-cluster key read 101 C. The first load can
    # also break the sampler: FAKE_KILL_SAMPLER kills it, FAKE_KEYS_HANG makes its next read of
    # the key list hang (a FIFO nobody writes), FAKE_KEYS_GONE makes that read fail.
    "taskset": r"""#!/bin/bash
echo "taskset $*" >>"$FAKE/log"
echo $$ >>"$FAKE/load-pids"
keys=$M3T/debug/macsmc-hwmon/keys
if [[ -n ${FAKE_HOT:-} ]]; then sed -i 's/ Tp0a flt    4 80 45000$/ Tp0a flt    4 80 101000/' "$keys"; fi
if mkdir "$FAKE/first-load" 2>/dev/null; then
  if [[ -n ${FAKE_KILL_SAMPLER:-} ]]; then pkill -KILL -f -- "$keys"; fi
  if [[ -n ${FAKE_KEYS_HANG:-} ]]; then
    # Publish the blocked reader atomically: an unlink/create gap can instead
    # make the sampler report a missing key list, which is a different test.
    mkfifo "$keys.hang" && mv -f "$keys.hang" "$keys"
  fi
  if [[ -n ${FAKE_KEYS_GONE:-} ]]; then rm -f "$keys"; fi
fi
exec sleep 60
""",
}

READER = r"""import os, sys, time
fake = os.environ["FAKE"]
with open(fake + "/log", "a") as f:
    f.write("reader " + " ".join(sys.argv[1:]) + "\n")
mode = os.environ.get("FAKE_READER", "ok")
if sys.argv[1:2] == ["--stage2-log"]:
    # m1n1's log of this boot, read from the other phram device.
    print("# aurora-adt-extract: m1n1's stage 2 log of this boot (m1n1_stage2.log); read-only")
    print("m1n1: published the boot loader facts on myhost-4711")
    sys.exit(0)
if sys.argv[1:2] == ["--check"]:
    if mode == "refuse":
        sys.stderr.write("aurora-adt-extract: mtd1 is of type 'nor', not 'ram' (phram)\n")
        sys.exit(3)
    print(sys.argv[2] + ": MTD device mtd1 named 'adt', checked")
    sys.exit(0)
print("/ compatible = apple,j514c")
sys.stdout.flush()
if mode == "hang":
    open(fake + "/reader-started", "w").close()
    time.sleep(60)
if mode == "fail":
    sys.stderr.write("adt: a node runs past the end of the ADT at 0x400\n")
    sys.exit(3)
print("/arm-io/sgx compatible = gpu,t6031")
print("serial-number = C02ZK1ABCDEF on myhost-4711")
"""


def u32(n):
    return struct.pack(">I", n)


def fdt(board, chip):
    # Enough of a flattened device tree for the report: its header (magic, size, version 17)
    # and the root's compatible strings.
    body = f"apple,{board}\0apple,{chip}\0apple,arm-platform\0".encode() + b"\0" * 7
    head = b"\xd0\x0d\xfe\xed" + u32(40 + len(body)) + u32(40) * 3 + u32(17) + u32(16) + u32(0) * 3
    return head + body


def adt_prop(name, value):
    if isinstance(value, int):
        value = struct.pack("<I", value)
    elif isinstance(value, str):
        value = value.encode() + b"\0"
    return name.encode().ljust(32, b"\0") + struct.pack("<I", len(value)) + value + b"\0" * (-len(value) % 4)


def adt_node(name, props=(), children=()):
    props = [("name", name)] + list(props)
    return (struct.pack("<II", len(props), len(children)) + b"".join(adt_prop(k, v) for k, v in props)
            + b"".join(children))


# A small ADT in the format the reader parses, with an allowed GPU node and things it must leave out.
ADT = adt_node("device-tree", [("#address-cells", 2), ("#size-cells", 2), ("serial-number", SERIAL)], [
    adt_node("arm-io", [("compatible", "arm-io,t6031"), ("#address-cells", 2), ("#size-cells", 2)], [
        adt_node("sgx", [("compatible", "gpu,t6031"), ("gpu-num-clusters", 4), ("unique-id", "0123456789abcdef")]),
        adt_node("wlan", [("local-mac-address", bytes.fromhex(MAC.replace(":", "")))]),
    ]),
]) + b"\0" * 64


def k2_reader():
    """The ADT reader: the tree's, or the one its branch added (commit 2ca6ff6e)."""
    here = flow.INSTALLER.parent / "aurora-adt-extract.py"
    if here.exists():
        return here.read_bytes()
    got = subprocess.run(["git", "show", "2ca6ff6e:tools/aurora-sep/aurora-adt-extract.py"],
                         cwd=flow.INSTALLER.parent, capture_output=True)
    return got.stdout if got.returncode == 0 else None


class MaxBase(flow.M3FlowBase):
    """The flow harness's fake Mac, with the extra boards, a device tree, sysfs, procfs and debugfs
    for the report and the survey, and the stubs above."""

    def setUp(self):
        super().setUp()
        patcher = mock.patch.dict(flow.BOARDS, EXTRA_BOARDS)
        patcher.start()
        self.addCleanup(patcher.stop)
        for name, body in STUBS.items():
            (self.tmp / "bin" / name).write_text(body)
            (self.tmp / "bin" / name).chmod(0o755)
        self.reader = self.tmp / "reader.py"
        self.reader.write_text(READER)
        self.out = self.tmp / "out"
        self.out.mkdir()
        self.extra_env.update({"M3T": str(self.tmp), "FAKE_HOST": HOST, "USER": "alice", "LOGNAME": "alice"})
        self.extra_env.pop("SUDO_USER", None)

    # ---- the fake Mac --------------------------------------------------------------------------

    def w(self, rel, data):
        p = self.tmp / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(data, str):
            data = data.encode()
        p.write_bytes(data)
        return p

    def link(self, rel, target):
        p = self.tmp / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        if p.is_symlink() or p.exists():
            p.unlink()
        p.symlink_to(self.tmp / target)

    def node(self, path, compatible=None, status=None, **props):
        d = self.tmp / "dt" / path.strip("/")
        d.mkdir(parents=True, exist_ok=True)
        if compatible:
            (d / "compatible").write_bytes(b"".join(c.encode() + b"\0" for c in compatible.split()))
        if status:
            (d / "status").write_bytes(status.encode() + b"\0")
        for k, v in props.items():
            (d / k.replace("_", "-")).write_bytes(v if isinstance(v, bytes) else v.encode() + b"\0")
        return d

    def platform(self, name, node, driver=None):
        dev = f"sys/bus/platform/devices/{name}"
        (self.tmp / dev).mkdir(parents=True, exist_ok=True)
        self.link(f"{dev}/of_node", f"dt/{node.strip('/')}")
        if driver:
            (self.tmp / f"sys/bus/platform/drivers/{driver}").mkdir(parents=True, exist_ok=True)
            self.link(f"{dev}/driver", f"sys/bus/platform/drivers/{driver}")

    def max_mac(self, board="j514c", smc=True, cpus=16, ecores=4, backlight=False, zones=("macsmc-battery",)):
        """A Mac with this board, as the report and the survey see it."""
        self.mac(board)
        chip = flow.BOARDS[board][1].split(",")[1]
        self.w("dt/model", MODELS.get(board, "Apple Mac") + "\0")
        self.w("dt/serial-number", SERIAL + "\0")
        self.w("dt/smbios/smbios/system/serial", SERIAL + "\0")
        self.w("dt/aliases/serial0", "/soc/serial@39b200000\0")
        self.w("dt/chosen/asahi,os-fw-version", "14.8.3\0")
        self.w("dt/chosen/asahi,system-fw-version", "26.0\0")
        self.w("dt/chosen/asahi,m1n1-stage2-version", "v1.6.1\0")
        self.w("dt/chosen/asahi,efi-system-partition", "1234abcd-0000-4000-8000-000000000000\0")
        self.w(f"dt/chosen/asahi,{chip}-facts", b"\x01\x02\0" + HOST.encode() + b"\0" + SERIAL.encode() + b"\0\xff")
        fb = self.node("chosen/framebuffer@10000000", "simple-framebuffer", "okay", format="a8r8g8b8")
        (fb / "width").write_bytes(u32(3024))
        (fb / "height").write_bytes(u32(1964))
        (fb / "stride").write_bytes(u32(3024 * 4))
        self.node("soc", "simple-bus")
        self.node("soc/serial@39b200000", "apple,s5l-uart", "okay")
        self.node("soc/smc@2a4400000", f"apple,{chip}-smc apple,t8103-smc")
        self.node("soc/i2c@39b040000", "apple,t8103-i2c apple,i2c")
        self.node("soc/dcp@38bc00000", "apple,dcp", "disabled")
        wifi = self.node("soc/pcie@580000000/pci@0,0/wifi@0,0", "pci14e4,4433")
        (wifi / "local-mac-address").write_bytes(bytes.fromhex(MAC.replace(":", "")))
        bt = self.node("soc/pcie@580000000/pci@0,0/bluetooth@0,1", "pci14e4,5f72")
        (bt / "local-bd-address").write_bytes(bytes.fromhex("f6e5d4c3b2a1"))
        rm = self.node("reserved-memory")
        (rm / "#address-cells").write_bytes(u32(2))
        (rm / "#size-cells").write_bytes(u32(2))
        for name, label, addr, size in (("flash@10003528000", "adt", 0x10003528000, 0x7C000),
                                        ("flash@108d994c000", "m1n1_stage2.log", 0x108D994C000, 0x4000)):
            n = self.node(f"reserved-memory/{name}", "phram", label=label)
            (n / "reg").write_bytes(u32(addr >> 32) + u32(addr & 0xFFFFFFFF) + u32(0) + u32(size))
            self.platform(f"{name.split('@')[1]}.flash", f"reserved-memory/{name}")
        self.platform("39b200000.serial", "soc/serial@39b200000", "apple-uart")
        self.platform("2a4400000.smc", "soc/smc@2a4400000")
        # CPUs: the first `ecores` are E-cores, the rest P-cores, as on an M3 Max.
        for n in range(cpus):
            c = f"sys/devices/system/cpu/cpu{n}"
            self.w(f"{c}/cpu_capacity", "600\n" if n < ecores else "1024\n")
            if n:
                self.w(f"{c}/online", "1\n")
            self.w(f"{c}/topology/cluster_id", ("0\n" if n < ecores else f"{1 + (n - ecores) // 6}\n"))
            self.w(f"{c}/regs/identification/midr_el1", "0x00000000611f0491\n")
        for p, rel in (("policy0", "0-3"), ("policy4", "4-9"), ("policy10", "10-15")):
            for k, v in (("related_cpus", rel), ("scaling_driver", "apple-cpufreq"),
                         ("cpuinfo_min_freq", "744000"), ("cpuinfo_max_freq", "4056000")):
                self.w(f"sys/devices/system/cpu/cpufreq/{p}/{k}", v + "\n")
        self.w("sys/class/typec/port0/data_role", "[host] device\n")
        self.w("sys/class/typec/port0/power_role", "[source] sink\n")
        self.w("sys/class/drm/card0-Unknown-1/status", "connected\n")
        self.w("sys/class/drm/card0-Unknown-1/modes", "3024x1964\n")
        self.w("sys/class/power_supply/macsmc-battery/uevent",
               f"POWER_SUPPLY_NAME=macsmc-battery\nPOWER_SUPPLY_SERIAL_NUMBER={BATTERY_SERIAL}\nPOWER_SUPPLY_CAPACITY=80\n")
        self.w("sys/class/power_supply/macsmc-battery/serial_number", BATTERY_SERIAL + "\n")
        for i, z in enumerate(zones):
            self.w(f"sys/class/thermal/thermal_zone{i}/type", z + "\n")
            self.w(f"sys/class/thermal/thermal_zone{i}/temp", "30000\n")
        self.w("sys/class/hwmon/hwmon0/name", "macsmc_hwmon\n")
        self.w("sys/class/hwmon/hwmon0/temp1_label", "Tp0a\n")
        self.w("sys/class/hwmon/hwmon0/temp1_input", "45000\n")
        self.w("sys/class/net/wlan0/address", MAC + "\n")
        self.w("sys/class/net/wlan0/operstate", "up\n")
        (self.tmp / "sys/bus/pci/drivers/brcmfmac").mkdir(parents=True, exist_ok=True)
        (self.tmp / "sys/devices/pci0/0000:01:00.0").mkdir(parents=True, exist_ok=True)
        self.link("sys/devices/pci0/0000:01:00.0/driver", "sys/bus/pci/drivers/brcmfmac")
        self.link("sys/class/net/wlan0/device", "sys/devices/pci0/0000:01:00.0")
        self.w("sys/class/rfkill/rfkill0/name", "hci0\n")
        self.w("sys/bus/usb/devices/usb1/serial", "xhci-hcd.3.auto\n")
        self.w("sys/bus/usb/devices/1-1/serial", USB_SERIAL + "\n")
        self.w("sys/class/mtd/mtd0/name", "nvram\n")
        self.w("sys/class/mtd/mtd0/type", "nor\n")
        self.w("sys/class/mtd/mtd0/size", "1048576\n")
        self.w("sys/class/mtd/mtd0/dev", "90:0\n")
        (self.tmp / "sys/class/mtd/mtd0ro").mkdir(parents=True, exist_ok=True)
        self.w("dev/mtd0", "")
        (self.tmp / "dev/mtd/by-name").mkdir(parents=True, exist_ok=True)
        (self.tmp / "dev/mtd/by-name/nvram").unlink(missing_ok=True)
        os.symlink("../../mtd0", self.tmp / "dev/mtd/by-name/nvram")
        self.w("proc/cmdline", f"root=UUID=x rw quiet systemd.hostname={HOST}\n")
        self.w("proc/meminfo", "MemTotal:       49920000 kB\nMemFree: 1 kB\n")
        self.w("proc/cpuinfo", "processor\t: 0\nCPU part\t: 0x049\n")
        self.w("proc/modules", "apple_dcp 16384 0 - Live 0x0\n")
        self.w("proc/interrupts", "  1:  10  AIC 0 Level  timer\n")
        self.w("proc/iomem", ("10000000000-1003fffffff : System RAM\n  10000010000-1000123ffff : Kernel code\n"
                              "  10003528000-100035a3fff : reserved\n200000000-2000fffff : 39b200000.serial\n"))
        self.w("proc/asound/cards", " 0 [AppleJ514]: macaudio - MacBook Pro J514\n")
        self.w("proc/bus/input/devices", f"I: Bus=0003\nN: Name=\"Keyboard\"\nP: Phys={BT}\nU: Uniq={USB_SERIAL}\n")
        if smc:
            self.w("debug/macsmc-hwmon/keys", SMC_KEYS)
        self.w("debug/devices_deferred", "2a4400000.smc\tdeferred: waiting for the mailbox\n")
        self.w("debug/pm_genpd/pm_genpd_summary", "domain  status\nps_dcp  off-0\n")
        self.w("etc/hostname", HOST + "\n")
        if backlight:
            self.w("sys/class/backlight/apple-panel-bl/max_brightness", "500\n")
            self.w("sys/class/backlight/apple-panel-bl/brightness", "321\n")
        self.w("fake/journal.txt", JOURNAL)
        self.w("fake/dmesg.txt", DMESG)
        # boot.bin: the m1n1, then two device trees; this board's is linux-asahi's.
        blob = fdt(board, chip)
        # (a stray magic in the m1n1 part, with no device tree behind it, is not counted).
        self.boot.write_bytes(b"M1N1:original\n\xd0\x0d\xfe\xed\0\0\1\0" + b"\0" * 64 + fdt("j516s", "t6030") + blob
                              + b"UBOOT")
        self.w(f"modules/7.1.12-2-11.36-sep-ARCH/dtbs/{chip}-{board}.dtb", blob)
        self.w(f"modules/6.18.1-asahi-ARCH/dtbs/{chip}-{board}.dtb", blob + b"other")

    def env_body(self):
        t = self.tmp
        return (f"M3_SYSFS='{t}/sys'\nM3_PROCFS='{t}/proc'\nM3_DEBUGFS='{t}/debug'\nM3_ETC='{t}/etc'\n"
                f"M3_MODULES='{t}/modules'\nM3_DEVFS='{t}/dev'\nm3_adt_reader() {{ echo '{self.reader}'; }}\n")

    # ---- running it ----------------------------------------------------------------------------

    def report(self, extra="", check=True, env=None):
        if env:
            self.extra_env.update(env)
        proc = self.run_sh(f"{self.env_body()}{extra}\ncd '{self.out}'\nm3_report", check=check)
        files = list(self.out.glob("aurora-m3-report-*.tgz"))
        return proc, files

    def report_files(self, extra="", env=None):
        proc, files = self.report(extra, env=env)
        self.assertEqual(len(files), 1, proc.stdout + proc.stderr)
        self.assertEqual(list(self.out.glob("*.partial")), [])
        with tarfile.open(files[0]) as t:
            members = t.getmembers()
            for m in members:
                self.assertEqual((m.uid, m.gid, m.uname, m.gname), (0, 0, "", ""), m.name)
            data = {m.name.removeprefix("./"): t.extractfile(m).read() for m in members if m.isfile()}
        files[0].unlink()
        return proc, data

    def text(self, files, name):
        return files[name].decode()


class ReportTest(MaxBase):
    def check_common(self, board, chip, proc, files):
        system = self.text(files, "system.txt")
        self.assertIn(f"board: {board} soc: {chip}", system)
        # The boot loader's versions, from /chosen.
        self.assertIn("m1n1-stage1-version: v1.6.1-dirty", system)
        self.assertIn("m1n1-stage2-version: v1.6.1", system)
        self.assertIn("os-fw-version: 14.8.3", system)
        self.assertIn(f"model: {MODELS[board]}", system)
        self.assertIn(f"compatible: apple,{board} apple,{chip} apple,arm-platform", system)
        self.assertIn("memory: 47.6 GiB (MemTotal 49920000 kB)", system)
        self.assertIn("cmdline: root=UUID=x rw quiet systemd.hostname=host", system)
        self.assertIn("boot framebuffer framebuffer@10000000: 3024x1964, stride 12096, format a8r8g8b8", system)
        self.assertIn("packages:", system)
        self.assertRegex(system, r"boot.bin: sha256 [0-9a-f]{64}")
        # Every node, with its compatible and status.
        nodes = self.text(files, "dt-nodes.txt")
        self.assertIn("/soc/serial@39b200000\tcompatible=apple,s5l-uart\tstatus=okay\n", nodes)
        self.assertIn("/soc/dcp@38bc00000\tcompatible=apple,dcp\tstatus=disabled\n", nodes)
        self.assertIn("/reserved-memory/flash@10003528000\tcompatible=phram\tstatus=-\n", nodes)
        self.assertIn("/\tcompatible=apple,", nodes)
        # Devices with and without a driver, nodes without a device, deferred probes.
        drivers = self.text(files, "drivers.txt")
        bound = drivers.split("== devices with a device-tree node and no driver")[0]
        self.assertIn("platform\t39b200000.serial\tapple-uart\t/soc/serial@39b200000", bound)
        unbound = drivers.split("no driver: bus, device, node\n")[1].split("==")[0]
        self.assertIn("platform\t2a4400000.smc\t/soc/smc@2a4400000", unbound)
        nodev = drivers.split("enabled SoC nodes with a compatible and no device\n")[1].split("==")[0]
        self.assertIn("/soc/i2c@39b040000\tcompatible=apple,t8103-i2c apple,i2c", nodev)
        self.assertNotIn("dcp@", nodev)      # disabled
        self.assertNotIn("serial@", nodev)   # has a device
        self.assertIn("2a4400000.smc\tdeferred: waiting for the mailbox", drivers)
        self.assertIn("ps_dcp  off-0", drivers)
        self.assertIn("apple_dcp 16384", drivers)
        # The whole kernel log, from the journal, its host column masked.
        log = self.text(files, "kernel-log.txt")
        self.assertIn("] host kernel: some unrelated line", log)
        self.assertIn("apple-t6031-pmgr", log)
        self.assertIn("dmesg", self.text(files, "README.txt"))
        # The CPUs.
        cpu = self.text(files, "cpu.txt")
        self.assertIn("cpu0 online=- capacity=600 cluster=0", cpu)
        self.assertIn("cpu15 online=1 capacity=1024 cluster=2", cpu)
        self.assertIn("policy10:\n  related_cpus: 10-15\n  scaling_driver: apple-cpufreq", cpu)
        self.assertIn("CPU part\t: 0x049", cpu)
        # Reserved memory: the boot loader's ADT and log regions, by address and size.
        rm = self.text(files, "reserved-memory.txt")
        self.assertIn("flash@10003528000\tcompatible=phram\tlabel=adt\treg=0x10003528000+0x7c000\tstatus=-\tno-map=no", rm)
        self.assertIn("adt node (the boot loader's copy of the ADT): flash@10003528000", rm)
        self.assertIn("m1n1_stage2.log node (m1n1's log of this boot): flash@108d994c000", rm)
        # Which device tree it boots: this board's in boot.bin is linux-aurora 11.36's file.
        bdt = self.text(files, "boot-dt.txt")
        self.assertIn(f"running device tree: model {MODELS[board]}; compatible apple,{board} apple,{chip}", bdt)
        self.assertIn("system-firmware stub (stub_info.json ProductVersion): 14.8.3", bdt)
        self.assertIn("asahi,os-fw-version: 14.8.3", bdt)
        self.assertIn(f"asahi,{chip}-facts: <", bdt)
        self.assertIn("apple,j516s apple,t6030", bdt)
        self.assertIn(f"apple,{board} apple,{chip}", bdt)
        self.assertIn("device trees in boot.bin: 2", bdt)
        self.assertIn(f"is 7.1.12-2-11.36-sep-ARCH/dtbs/{chip}-{board}.dtb\n", bdt)
        # /proc/iomem: System RAM and reserved only.
        iomem = self.text(files, "iomem.txt")
        self.assertIn("10000000000-1003fffffff : System RAM", iomem)
        self.assertIn("10003528000-100035a3fff : reserved", iomem)
        self.assertNotIn("Kernel code", iomem)
        self.assertNotIn("serial", iomem)
        # Devices.
        self.assertIn("Broadcom BCM4388 [14e4:4433]", self.text(files, "buses.txt"))
        self.assertIn("ID 05ac:0281", self.text(files, "buses.txt"))
        self.assertIn("data_role: [host] device", self.text(files, "typec.txt"))
        self.assertIn("card0-Unknown-1: status connected", self.text(files, "display.txt"))
        self.assertIn("modes: 3024x1964", self.text(files, "display.txt"))
        power = self.text(files, "power.txt")
        self.assertIn("POWER_SUPPLY_CAPACITY=80", power)
        self.assertIn("thermal_zone0: type macsmc-battery, temp 30000", power)
        self.assertIn("temp1: Tp0a = 45000", power)
        self.assertIn("macaudio - MacBook Pro J514", self.text(files, "sound-input.txt"))
        self.assertIn("U: Uniq=(masked)", self.text(files, "sound-input.txt"))
        self.assertIn("wlan0: driver brcmfmac, type -, operstate up", self.text(files, "network.txt"))
        # The /chosen entries of the chip, copied whole (masked, same length).
        facts = files[f"chosen/asahi,{chip}-facts"]
        self.assertEqual(len(facts), 3 + len(HOST) + 1 + len(SERIAL) + 2)
        self.assertTrue(facts.startswith(b"\x01\x02\0"))
        # The attach instructions.
        self.assertIn(f"aurora-m3-report-{board}-", proc.stdout)
        self.assertIn("https://github.com/omacom/linux-aurora/issues", proc.stdout)

    def test_m3_max_t6031(self):
        self.max_mac("j514c")
        proc, files = self.report_files()
        self.check_common("j514c", "t6031", proc, files)
        smc = self.text(files, "smc-keys.txt")
        self.assertIn("Tp0a flt    4 80 45000", smc)
        self.assertIn("PSTR", smc)
        self.assertNotIn("RPlt", smc)
        self.assertIn("apple-t6031-pmgr", self.text(files, "dmesg-m3.txt"))
        self.assertIn("adt-allowlist.txt", files)

    def test_m3_max_t6034_without_smc_keys(self):
        # No key list (debugfs not mounted, or the SMC driver did not start): one clear line, no
        # failure. The kernel's SMC driver knows the t6034 (macsmc_hwmon_is_m3), so the line no
        # longer blames the chip.
        self.max_mac("j514m", smc=False)
        proc, files = self.report_files()
        self.check_common("j514m", "t6034", proc, files)
        line = (f"no SMC key list on this kernel (no {self.tmp}/debug/*smc*/keys: debugfs is not mounted, "
                "or the SMC driver did not start)")
        self.assertNotIn("not yet supported", proc.stderr)
        self.assertEqual(self.text(files, "smc-keys.txt"), line + "\n")
        self.assertIn(line, proc.stderr)

    def test_m3_pro_t6030(self):
        self.max_mac("j516s")
        (self.tmp / "dt/chosen/asahi,t6030-pmp").write_bytes(b"pmp\0")
        _, files = self.report_files()
        self.check_common("j516s", "t6030", _, files)
        self.assertEqual(files["chosen/asahi,t6030-pmp"], b"pmp\0")

    def test_m3_t8122(self):
        for board in ("j504", "j613"):
            with self.subTest(board=board):
                self.max_mac(board)
                proc, files = self.report_files()
                self.check_common(board, "t8122", proc, files)
                self.assertIn("chosen/asahi,t8122-facts", files)

    def test_other_m3_chips_are_copied(self):
        # The /chosen copy covers every M3 chip, t6031 and t6034 included.
        self.assertRegex(SRC, r'(?m)^M3_CHOSEN_SOCS="t8122 t6030 t6031 t6034"$')
        dm = re.search(r"^M3_REPORT_DMESG='([^']*)'$", SRC, re.M).group(1).split("|")
        for chip in ("t8122", "t6030", "t6031", "t6034"):
            self.assertIn(chip, dm)

    def test_the_report_only_reads(self):
        self.max_mac("j514c")
        before = {p: p.read_bytes() for p in self.tmp.rglob("*") if p.is_file() and not p.is_symlink()
                  and "fake" not in p.parts and p != self.boot}
        boot = self.boot.read_bytes()
        self.report_files()
        self.assertEqual(self.boot.read_bytes(), boot)
        after = {p: p.read_bytes() for p in before}
        self.assertEqual(after, before)
        log = self.log()
        self.assertNotRegex(log, r"(?m)^pacman -[USR]")
        self.assertNotIn("update-m1n1", log)
        self.assertNotIn("taskset", log)

    def test_bad_cwd(self):
        self.max_mac("j514c")
        self.out.chmod(0o555)
        self.addCleanup(self.out.chmod, 0o755)
        proc, files = self.report(check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(files, [])
        self.assertIn("directory you can write to", proc.stderr)


class PrivacyTest(MaxBase):
    def test_nothing_private_leaves_the_mac(self):
        self.max_mac("j514c")
        proc, files = self.report_files()
        for name, data in files.items():
            for s in SECRETS:
                with self.subTest(file=name, secret=s):
                    self.assertNotIn(s.lower().encode(), data.lower())
        log = self.text(files, "kernel-log.txt")
        self.assertIn("audit: exe=/home/USER/bin/tool", log)
        self.assertIn("some unrelated line from host for user", log)
        self.assertIn("smbios: system serial SERIAL raw xxxxxxxxxxxx", log)
        self.assertNotIn("SerialNumber", log)
        self.assertIn("brcmfmac: wlan0 address xx:xx:xx:xx:xx:xx", log)
        self.assertIn("serial-number = SERIAL on host", self.text(files, "adt-allowlist.txt"))
        # A root hub's "serial" is not a secret, and a word is masked only where it stands alone.
        self.assertNotIn("SERIAL", self.text(files, "buses.txt"))
        self.assertIn("myhost-4711", "".join(self.run_sh("m3_privacy_hosts").stdout.split()))
        for s in SECRETS:
            self.assertNotIn(s, proc.stdout + proc.stderr)

    def secrets(self):
        return self.run_sh(self.env_body() + "m3_privacy_secrets").stdout.splitlines()

    def test_the_secrets_found(self):
        self.max_mac("j514c")
        got = self.secrets()
        for line in (f"host {FQDN}", f"host {HOST}", "user alice", "user bob", f"serial {SERIAL}",
                     f"serial {BATTERY_SERIAL}", f"serial {USB_SERIAL}", "mac 123456789abc",
                     "mac bc9a78563412", "mac f6e5d4c3b2a1", "mac a1b2c3d4e5f6"):
            self.assertIn(line, got)
        self.assertEqual(len(got), len(set(got)))
        self.assertNotIn("user root", got)
        self.assertNotIn("user nobody", got)
        self.assertFalse([l for l in got if "xhci" in l or "/soc/" in l])

    KINDS = ("the host name", "a user name", "a serial number", "a MAC address")

    def clean_content(self):
        # The fake Mac with nothing private left where the report collects it (the secrets it
        # looks for are still found: the host name in M3_ETC, the users, the serials, the MACs).
        self.max_mac("j514c")
        self.w("fake/journal.txt", f"[    0.000000] h kernel: Linux version {KREL}\n")
        self.reader.write_text('print("/ compatible = apple,j514c")\n')
        self.w("proc/cmdline", "quiet\n")
        self.w("proc/bus/input/devices", "I: Bus=0003\n")
        self.w("dt/chosen/asahi,t6031-facts", b"\x01\x02")
        self.extra_env["FAKE_HOST"] = "h"

    NO_MASK = "m3_privacy_mask() { return 0; }\n"

    def refused(self, plant, kind, file="kernel-log.txt"):
        # With masking switched off, the check alone must catch it, and name only that kind. The
        # kernel log drops serial-number lines and masks MAC addresses as it is collected, so
        # those two are planted in the ADT reader's output.
        self.clean_content()
        if file == "kernel-log.txt":
            with open(self.tmp / "fake/journal.txt", "a") as f:
                f.write(f"[    1.000000] h kernel: {plant}\n")
        else:
            self.reader.write_text(f'print("/ compatible = apple,j514c")\nprint({plant!r})\n')
        proc, files = self.report(extra=self.NO_MASK, check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(files, [])
        self.assertEqual(list(self.out.iterdir()), [])
        self.assertIn(f"{kind} in {file}", proc.stderr)
        for other in self.KINDS:
            if other != kind:
                self.assertNotIn(f"{other} in", proc.stderr)
        self.assertIn("the report was not kept", proc.stderr)
        self.assertNotIn(plant.split()[-1], proc.stderr)

    def test_a_clean_file_passes_unmasked(self):
        self.clean_content()
        proc, files = self.report(extra=self.NO_MASK)
        self.assertEqual(len(files), 1, proc.stderr)
        self.assertNotIn("SERIAL", proc.stderr)

    def test_a_host_name_is_refused(self):
        self.refused("on myhost-4711 today", "the host name")

    def test_a_user_name_is_refused(self):
        self.refused("ran by alice", "a user name")

    def test_a_serial_number_is_refused(self):
        self.refused(f"system serial {SERIAL}", "a serial number")

    def test_a_serial_number_label_is_refused(self):
        self.refused("disk Serial Number: ZZ99XY77", "a serial number", "adt-allowlist.txt")

    def test_a_mac_address_is_refused(self):
        self.refused("peer de:ad:be:ef:00:01", "a MAC address", "adt-allowlist.txt")

    def test_a_known_mac_without_separators_is_refused(self):
        self.refused("bd 123456789ABC", "a MAC address")

    def check(self, tgz, listing):
        secrets = self.tmp / "secrets"
        secrets.write_text(f"host {HOST}\n")
        (self.tmp / "list").write_text(listing)
        return self.run_sh(f"m3_privacy_check '{tgz}' '{secrets}' '{self.tmp / 'list'}'", check=False)

    def test_an_owner_or_an_extra_file_is_refused(self):
        d = self.tmp / "pack"
        d.mkdir()
        (d / "a.txt").write_text("fine\n")
        tgz = self.tmp / "x.tgz"
        subprocess.run(["tar", "-czf", str(tgz), "-C", str(d), "."], check=True)
        proc = self.check(tgz, "a.txt\n")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("the file names an owner", proc.stdout)
        subprocess.run(["tar", "--owner=0", "--group=0", "--numeric-owner", "-czf", str(tgz), "-C", str(d), "."],
                       check=True)
        self.assertEqual(self.check(tgz, "a.txt\n").returncode, 0)
        proc = self.check(tgz, "")
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("the file list differs", proc.stdout)
        (d / "b.txt").write_text(f"x {HOST} y\n")
        subprocess.run(["tar", "--owner=0", "--group=0", "--numeric-owner", "-czf", str(tgz), "-C", str(d), "."],
                       check=True)
        proc = self.check(tgz, "a.txt\nb.txt\n")
        self.assertIn("the host name in b.txt", proc.stdout)
        self.assertNotIn(HOST, proc.stdout)

    def test_binary_files_keep_their_length(self):
        self.max_mac("j514c")
        before = (self.tmp / "dt/chosen/asahi,t6031-facts").read_bytes()
        _, files = self.report_files()
        after = files["chosen/asahi,t6031-facts"]
        self.assertEqual(len(after), len(before))
        self.assertEqual(after, before.replace(HOST.encode(), b"x" * len(HOST)).replace(SERIAL.encode(), b"x" * len(SERIAL)))


class AdtTest(MaxBase):
    def setUp(self):
        super().setUp()
        self.max_mac("j514c")

    def files(self, extra="", **env):
        _, files = self.report_files(extra, env=env)
        return self.text(files, "adt-allowlist.txt"), self.text(files, "adt-check.txt")

    def modprobes(self):
        return [l for l in self.log().splitlines() if l.startswith("modprobe ")]

    LOAD_UNLOAD = ["modprobe phram", "udevadm settle --timeout=10", "udevadm settle --timeout=10", "modprobe -r phram"]

    def steps(self):
        return [l for l in self.log().splitlines() if l.startswith(("modprobe ", "udevadm "))]

    def assert_restored(self, check):
        before = check.split("== before\n")[1].split("\n==")[0].split("\nregion:")[0].split("\nstopped:")[0]
        after = check.split("== after\n")[1].split("\nrestored:")[0]
        self.assertEqual(before.strip(), after.strip())
        self.assertIn("restored: yes", check)
        self.assertFalse((self.tmp / "sys/module/phram").exists())
        self.assertEqual(sorted(p.name for p in (self.tmp / "sys/class/mtd").iterdir()), ["mtd0", "mtd0ro"])

    def test_read_and_restore(self):
        allow, check = self.files()
        self.assertIn("/arm-io/sgx compatible = gpu,t6031", allow)
        # udev settles after the load and before the unload.
        self.assertEqual(self.steps(), self.LOAD_UNLOAD)
        self.assertIn("== with phram\nphram: loaded\nmtd0 name=nvram", check)
        self.assertIn("mtd1 name=adt type=ram size=507904", check)
        self.assertIn("mtd2 name=m1n1_stage2.log type=ram size=16384", check)
        # Only the read-only node of the device named adt; then, once the ADT was read, the
        # read-only node of m1n1's log (G1), while phram is still loaded.
        reader = [l for l in self.log().splitlines() if l.startswith("reader ")]
        self.assertEqual(reader, [f"reader --check {self.tmp}/dev/mtd1ro", f"reader {self.tmp}/dev/mtd1ro",
                                  f"reader --stage2-log {self.tmp}/dev/mtd2ro",
                                  f"reader --stage2-log --raw {self.tmp}/dev/mtd2ro"])
        self.assertIn(f"log: {self.tmp}/dev/mtd2ro with reader.py --stage2-log\nlog: done\nlog raw: done\n", check)
        self.assertLess(check.index("log: done"), check.index("unloaded: phram"))
        self.assertIn("mtd1ro: MTD device mtd1 named 'adt', checked", check)
        self.assertIn("phram: not loaded\nmtd0 name=nvram type=nor size=1048576 erasesize=- dev=90:0 by-name=nvram", check)
        self.assertIn("region: /reserved-memory/flash@10003528000 reg 0x10003528000+0x7c000 (507904 bytes)", check)
        self.assertIn("loaded: phram", check)
        self.assertIn("read: done", check)
        self.assertIn("unloaded: phram", check)
        self.assert_restored(check)

    def test_owner_loaded_phram_is_left_alone(self):
        (self.tmp / "sys/module/phram").mkdir(parents=True)
        allow, check = self.files()
        self.assertIn("phram is already loaded on this Mac, not by this report", allow)
        self.assertEqual(self.modprobes(), [])
        self.assertTrue((self.tmp / "sys/module/phram").exists())
        self.assertNotIn("reader ", self.log())
        self.assertIn("phram: loaded", check.split("== after")[1])
        self.assertIn("restored: yes", check)

    def test_an_unknown_mtd_device_stops_it(self):
        self.w("sys/class/mtd/mtd5/name", "something\n")
        allow, _ = self.files()
        self.assertIn("an MTD device other than nvram is present", allow)
        self.assertEqual(self.modprobes(), [])

    def test_a_mismatched_region_is_not_read(self):
        allow, check = self.files(FAKE_ADT_SIZE="4096")
        self.assertIn("the adt MTD device (mtd1, 4096 bytes) does not match its reserved-memory region "
                      "(flash@10003528000, 507904 bytes)", allow)
        self.assertNotIn("reader ", self.log())
        self.assertEqual(self.steps(), self.LOAD_UNLOAD)
        self.assert_restored(check)

    def test_an_oversized_region_is_not_loaded(self):
        n = self.tmp / "dt/reserved-memory/flash@10003528000/reg"
        n.write_bytes(n.read_bytes()[:8] + u32(0) + u32(0x2000000))
        allow, check = self.files()
        self.assertIn("the adt region is 33554432 bytes, not 1 to 16777216", allow)
        self.assertEqual(self.modprobes(), [])
        self.assertIn("restored: yes", check)

    def test_a_device_bound_to_another_node_is_not_read(self):
        allow, check = self.files(FAKE_ADT_NODE="108d994c000")
        self.assertIn("does not match its reserved-memory region", allow)
        self.assertNotIn("reader ", self.log())
        self.assert_restored(check)

    def test_the_readers_check_refuses(self):
        allow, check = self.files(FAKE_READER="refuse")
        self.assertIn("the ADT reader's check refused", allow)
        self.assertIn("not 'ram' (phram)", allow)
        reader = [l for l in self.log().splitlines() if l.startswith("reader ")]
        self.assertEqual(reader, [f"reader --check {self.tmp}/dev/mtd1ro"])
        self.assertEqual(self.steps(), self.LOAD_UNLOAD)
        self.assert_restored(check)

    def test_an_error_mid_read_still_unloads(self):
        allow, check = self.files(FAKE_READER="fail")
        self.assertIn("the ADT reader failed (exit 3), so its output was left out: adt: a node runs past", allow)
        self.assertNotIn("compatible = apple,j514c", allow)
        self.assertEqual(self.steps(), self.LOAD_UNLOAD)
        self.assert_restored(check)

    def test_an_interruption_mid_read_still_unloads(self):
        captured = {}

        def capture(args, **kw):
            captured.update(args=args, env=kw["env"])
            return subprocess.CompletedProcess(args, 0, "", "")

        with mock.patch.object(flow.subprocess, "run", side_effect=capture):
            self.run_sh(f"{self.env_body()}\ncd '{self.out}'\nm3_report")
        env = {**captured["env"], "FAKE_READER": "hang"}
        p = subprocess.Popen(captured["args"], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             text=True, start_new_session=True)
        for _ in range(200):
            if (self.fake / "reader-started").exists():
                break
            time.sleep(0.05)
        self.assertTrue((self.fake / "reader-started").exists())
        os.killpg(p.pid, signal.SIGINT)
        out, err = p.communicate(timeout=30)
        self.assertEqual(p.returncode, 130, err)
        self.assertIn("interrupted", err)
        self.assertEqual(self.steps(), self.LOAD_UNLOAD)
        self.assertFalse((self.tmp / "sys/module/phram").exists())
        self.assertEqual(list(self.out.iterdir()), [])

    def test_a_failed_unload_shows(self):
        # Settled and tried twice, never forced; one line on the terminal, and the record in the tgz.
        proc, files = self.report_files(env={"FAKE_FAIL_UNLOAD": "1"})
        check = self.text(files, "adt-check.txt")
        self.assertIn("unload FAILED: phram stays loaded (in use after udev settled, twice; not forced)", check)
        self.assertIn("restored: NO", check)
        self.assertEqual(self.steps(), ["modprobe phram", "udevadm settle --timeout=10", "udevadm settle --timeout=10",
                                        "modprobe -r phram", "udevadm settle --timeout=10", "modprobe -r phram"])
        self.assertNotRegex(self.log(), r"modprobe .*(-f|--force)")
        lines = [l for l in proc.stderr.splitlines() if "phram" in l]
        self.assertEqual(len(lines), 1, proc.stderr)
        self.assertIn("still loaded", lines[0])

    def test_exactly_the_two_regions_devices(self):
        allow, check = self.files(FAKE_EXTRA_MTD="1")
        self.assertIn("phram made the MTD devices adt m1n1_stage2.log other, not exactly adt and m1n1_stage2.log", allow)
        self.assertNotIn("reader ", self.log())
        self.assertEqual(self.steps(), self.LOAD_UNLOAD)
        self.assert_restored(check)

    def test_with_the_real_reader(self):
        # The reader from the kernel branch, on the fake region (a regular file in place of the node:
        # the reader then checks no device and parses the file).
        src = k2_reader()
        if src is None:
            self.skipTest("the ADT reader is not in this tree, and git can't show 2ca6ff6e")
        self.reader.write_bytes(src)
        adt = self.tmp / "adt.bin"
        adt.write_bytes(ADT)
        allow, check = self.files(FAKE_ADT_FILE=str(adt))
        self.assertIn("a regular file, no device to check", check)
        self.assertIn("/arm-io/sgx", allow)
        self.assertIn("gpu-num-clusters", allow)
        self.assertIn("# summary:", allow)
        self.assertNotIn("wlan", allow)
        self.assertNotIn("0123456789abcdef", allow)
        self.assertNotIn(SERIAL, allow)
        self.assertNotIn("SERIAL", allow)  # left out by the reader, so there was nothing to mask
        self.assert_restored(check)

    def test_no_phram_module(self):
        allow, check = self.files(FAKE_NO_PHRAM="1")
        self.assertEqual(allow, "no phram module on this kernel (it comes with 12.3), so the ADT was not read\n")
        self.assertIn("restored: yes", check)

    def test_no_adt_region(self):
        shutil.rmtree(self.tmp / "dt/reserved-memory/flash@10003528000")
        allow, check = self.files()
        self.assertIn("no adt region in the device tree", allow)
        self.assertEqual(self.modprobes(), [])

    def test_no_adt_device(self):
        allow, check = self.files(FAKE_NO_ADT_MTD="1")
        self.assertIn("phram made no MTD device, so the ADT was not read", allow)
        self.assertEqual(self.steps(), self.LOAD_UNLOAD)
        self.assertIn("restored: yes", check)

    def test_not_an_m3(self):
        # M1 and M2: no ADT step, no module touched.
        self.mac("j314s")
        _, files = self.report_files()
        self.assertNotIn("adt-allowlist.txt", files)
        self.assertEqual(self.modprobes(), [])

    def reader_path(self, extra, **env):
        proc = self.run_sh(f"M3_WORK='{self.tmp}'\n{extra}\nm3_adt_reader", check=False)
        return proc

    def test_this_release_has_no_reader(self):
        # Run from a copy with no reader next to it (a checkout may have one).
        d = self.tmp / "alone"
        d.mkdir()
        shutil.copy(flow.INSTALLER, d / "install-aurora-sep.sh")
        self.installer = d / "install-aurora-sep.sh"
        try:
            proc = self.reader_path('M3_ADT_READER=""')
        finally:
            self.installer = flow.INSTALLER
        self.assertEqual(proc.returncode, 1)
        self.assertIn("has no ADT reader", proc.stdout)

    def test_the_reader_is_downloaded_and_checked(self):
        shutil.copy(self.reader, self.tmp / "pkgs/aurora-adt-extract.py")
        sha = hashlib.sha256(self.reader.read_bytes()).hexdigest()
        proc = self.reader_path(f'M3_ADT_READER="aurora-adt-extract.py {sha}"')
        self.assertEqual(proc.stdout.strip(), f"{self.tmp}/aurora-adt-extract.py", proc.stderr)
        self.assertIn("aurora-adt-extract.py", self.log())
        proc = self.reader_path(f'M3_ADT_READER="aurora-adt-extract.py {"0" * 64}"')
        self.assertEqual(proc.returncode, 1)
        self.assertIn("does not match its published checksum", proc.stdout)

    def test_a_copy_next_to_the_script(self):
        # Used as it is in a checkout whose release names no reader; with a release's entry, only
        # when it has that release's bytes, else the release's own is downloaded.
        d = self.tmp / "checkout"
        d.mkdir()
        shutil.copy(flow.INSTALLER, d / "install-aurora-sep.sh")
        (d / "aurora-adt-extract.py").write_text("print(1)\n")
        shutil.copy(self.reader, self.tmp / "pkgs/aurora-adt-extract.py")
        mine = hashlib.sha256(b"print(1)\n").hexdigest()
        released = hashlib.sha256(self.reader.read_bytes()).hexdigest()
        self.installer = d / "install-aurora-sep.sh"
        try:
            local = self.reader_path('M3_ADT_READER=""')
            same = self.reader_path(f'M3_ADT_READER="aurora-adt-extract.py {mine}"')
            other = self.reader_path(f'M3_ADT_READER="aurora-adt-extract.py {released}"')
        finally:
            self.installer = flow.INSTALLER
        self.assertEqual(local.stdout.strip(), f"{d}/aurora-adt-extract.py")
        self.assertEqual(same.stdout.strip(), f"{d}/aurora-adt-extract.py")
        self.assertEqual(other.stdout.strip(), f"{self.tmp}/aurora-adt-extract.py")


class NextStepsTest(MaxBase):
    BLOCK = "NEXT STEPS: M3 BRING-UP"

    def test_the_block_ends_the_summary(self):
        for board in ("j514c", "j516c", "j514m", "j516m", "j504", "j433"):
            with self.subTest(board=board):
                self.fresh_state()
                self.mac(board)
                before = self.boot.read_bytes()
                out = self.install().stdout
                self.assertEqual(self.boot.read_bytes(), before)
                self.assertEqual((self.state / "m3-mode").read_text().strip(), "kernel")
                lines = out.rstrip("\n").splitlines()
                start = [i for i, l in enumerate(lines) if self.BLOCK in l]
                self.assertEqual(len(start), 1, out)
                block = "\n".join(lines[start[0]:])
                self.assertTrue(lines[-1].startswith("====="), lines[-1])
                self.assertIn(f"({board})", lines[start[0]])
                self.assertIn("1. Reboot.", block)
                self.assertIn("uname -r", block)
                self.assertIn(f"It should print {KREL}.", block)
                self.assertIn("curl -fsSL https://github.com/omacom/linux-aurora/releases/latest/download/"
                              "install-aurora-sep.sh | bash -s -- --m3-report", block)
                self.assertIn("https://github.com/omacom/linux-aurora/issues", block)
                # In that order, after everything else.
                self.assertLess(block.index("Reboot"), block.index("uname -r"))
                self.assertLess(block.index("uname -r"), block.index("--m3-report"))
                self.assertLess(block.index("--m3-report"), block.index("/issues"))
                self.assertLess(out.index("To undo the kernel install"), out.index(self.BLOCK))

    def test_other_macs_have_no_block(self):
        for board, try_ in (("j613", 0), ("j516s", 0), ("j514s", 0), ("j514s", 1), ("j314s", 0), ("j414s", 0),
                            ("j700", 0)):
            with self.subTest(board=board, try_=try_):
                self.fresh_state()
                self.mac(board)
                self.assertNotIn(self.BLOCK, self.install(try_=try_).stdout)

    def test_m3_max_still_refuses_the_handoff(self):
        # The 14-core M3 Max (t6034) has no boot loader variant; the 16-core one (t6031) has an
        # opt-in one (test_m3max_kit.VariantTest).
        for board in ("j514m", "j516m"):
            with self.subTest(board=board):
                self.fresh_state()
                self.mac(board)
                before = self.boot.read_bytes()
                proc = self.install(try_=1, check=False)
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn("has no display and GPU handoff in m1n1 yet", proc.stderr)
                self.assertEqual(self.boot.read_bytes(), before)
                self.assertNotIn("pacman -U", self.log())

    def test_kernel_release(self):
        self.assertEqual(self.run_sh("m3_kernel_release").stdout.strip(), KREL)
        self.assertEqual(self.run_sh("VERSION=7.1.12.aurora2-12.2\nm3_kernel_release").stdout.strip(),
                         "7.1.12-2-12.2-sep-ARCH")
        self.assertEqual(self.run_sh("VERSION=odd\nm3_kernel_release").stdout, "")


class SurveyTest(MaxBase):
    FAST = "M3_SURVEY_PHASE_S=3\nM3_SURVEY_REST_S=1\nM3_SURVEY_TICK=0.1\nM3_SURVEY_WAIT_S=0\nM3_SURVEY_GAP=0.05\n"

    def body(self, extra=""):
        return f"{self.env_body()}{self.FAST}{extra}\ncd '{self.out}'\nm3_power_survey"

    def survey(self, extra="", check=True, **env):
        self.extra_env.update(env)
        proc = self.run_sh(self.body(extra), check=check)
        tgz = list(self.out.glob("aurora-m3-power-*.tgz"))
        files = {}
        if tgz:
            with tarfile.open(tgz[0]) as t:
                for m in t.getmembers():
                    self.assertEqual((m.uid, m.gid, m.uname, m.gname), (0, 0, "", ""))
                files = {m.name.removeprefix("./"): t.extractfile(m).read().decode()
                         for m in t.getmembers() if m.isfile()}
        return proc, tgz, files

    def phases(self, files):
        return [l.split()[0] for l in files["phases.txt"].splitlines() if l.split()[2] == "start"]

    def loads(self):
        return [l.split()[2] for l in self.log().splitlines() if l.startswith("taskset -c ")]

    def assert_loads_ended(self):
        pids = [int(p) for p in (self.fake / "load-pids").read_text().split()] if (self.fake / "load-pids").exists() else []
        for _ in range(100):
            alive = [p for p in pids if Path(f"/proc/{p}").exists()
                     and "Z" not in Path(f"/proc/{p}/stat").read_text().split()[2:3]]
            if not alive:
                break
            time.sleep(0.05)
        self.assertEqual(alive if pids else [], [])
        keys = str(self.tmp / "debug/macsmc-hwmon/keys")
        sampler = subprocess.run(["pgrep", "-f", keys], capture_output=True, text=True).stdout.split()
        self.assertEqual(sampler, [])

    def test_refused_on_m1_and_m2(self):
        for board in ("j314s", "j293", "j414s", "j313", "j700"):
            with self.subTest(board=board):
                self.max_mac("j514c", backlight=True)
                self.mac(board)
                proc, tgz, _ = self.survey(check=False)
                self.assertNotEqual(proc.returncode, 0)
                self.assertIn("--m3-power-survey is for an M3 Mac", proc.stderr)
                self.assertIn("Nothing was run", proc.stderr)
                self.assertEqual(tgz, [])
                self.assertEqual(self.loads(), [])
                self.assertEqual((self.tmp / "sys/class/backlight/apple-panel-bl/brightness").read_text(), "321\n")

    def test_phases_in_order_with_a_backlight(self):
        self.max_mac("j514c", backlight=True)
        proc, tgz, files = self.survey()
        self.assertEqual(len(tgz), 1, proc.stderr)
        self.assertEqual(self.phases(files),
                         ["rest-before-idle", "idle", "rest-before-cpu-all", "cpu-all", "rest-before-cpu-p", "cpu-p",
                          "rest-before-cpu-e", "cpu-e", "rest-before-backlight-max", "backlight-max",
                          "rest-before-backlight-min", "backlight-min"])
        # The CPU split from the topology: P-cores 4-15, E-cores 0-3; all, then P, then E.
        p = [str(n) for n in range(4, 16)]
        e = [str(n) for n in range(4)]
        loads = self.loads()
        self.assertEqual([sorted(loads[:16], key=int), sorted(loads[16:28], key=int), sorted(loads[28:], key=int)],
                         [sorted(p + e, key=int), p, e])
        self.assertIn("P-cores: " + " ".join(p), files["cpus.txt"])
        self.assertIn("E-cores: " + " ".join(e), files["cpus.txt"])
        self.assertIn("policy10: cpus 10-15, 744000-4056000 kHz, apple-cpufreq", files["cpus.txt"])
        # The backlight went to its maximum, its minimum, and back.
        self.assertEqual((self.tmp / "sys/class/backlight/apple-panel-bl/brightness").read_text(), "321\n")
        samples = files["samples.txt"].splitlines()
        self.assertTrue([l for l in samples if l.split()[1:3] == ["idle", "Tp0a"]])
        self.assertTrue([l for l in samples if l.split()[1:3] == ["cpu-e", "tz:macsmc-battery"]])
        self.assertFalse([l for l in samples if "RPlt" in l])
        summary = files["summary.txt"]
        self.assertIn("result: completed", summary)
        self.assertRegex(summary, r"(?m)^Tp0a +45\.00 +45\.00 +45\.00 +45\.00 +45\.00 +45\.00$")
        self.assertRegex(summary, r"(?m)^PSTR +4\.32 ")
        self.assertIn("idle", summary.splitlines()[0])
        self.assertIn("backlight-min", summary.splitlines()[0])
        self.assertIn("Starting in 0 seconds. Press Ctrl-C now to cancel.", proc.stdout)
        self.assertIn("about 1 minutes", proc.stdout)
        self.assertIn("aurora-m3-power-j514c-", proc.stdout)
        for name, data in files.items():
            for s in SECRETS:
                self.assertNotIn(s.lower(), data.lower(), name)
        self.assert_loads_ended()

    def test_without_a_backlight(self):
        self.max_mac("j514c")
        proc, tgz, files = self.survey()
        self.assertEqual(self.phases(files)[-1], "cpu-e")
        self.assertIn("no backlight device: backlight phases skipped", files["cpus.txt"])
        self.assertIn("no backlight device, so the backlight phases are left out", proc.stdout)

    def test_the_wait_comes_first(self):
        self.assertRegex(SRC, r"(?m)^M3_SURVEY_WAIT_S=10 ")
        body = SRC[SRC.index("m3_power_survey() {"):]
        self.assertLess(body.index('sleep "$M3_SURVEY_WAIT_S"'), body.index("m3_survey_sampler_start"))
        self.assertLess(body.index("trap 'm3_survey_interrupted' INT TERM HUP"), body.index('sleep "$M3_SURVEY_WAIT_S"'))

    def popen(self, extra="", **env):
        captured = {}

        def capture(args, **kw):
            captured.update(args=args, env=kw["env"])
            return subprocess.CompletedProcess(args, 0, "", "")

        with mock.patch.object(flow.subprocess, "run", side_effect=capture):
            self.run_sh(self.body(extra))
        process = subprocess.Popen(captured["args"], env={**captured["env"], **env}, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, text=True, start_new_session=True)

        def cleanup():
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            try:
                process.communicate(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.communicate(timeout=10)

        self.addCleanup(cleanup)
        return process

    def wait_for(self, cond, what, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if cond():
                return
            time.sleep(0.025)
        self.fail(f"timed out waiting for {what}")

    def test_ctrl_c_stops_the_load(self):
        self.max_mac("j514c", backlight=True)
        p = self.popen("M3_SURVEY_PHASE_S=10\n")
        self.wait_for(lambda: len(self.loads()) >= 16, "the load")
        # Under load bash sometimes drops one SIGINT and the survey runs to the end (exit 0). A user
        # presses Ctrl-C again, so send it again.
        for _ in range(4):
            os.killpg(p.pid, signal.SIGINT)
            try:
                out, err = p.communicate(timeout=2)
                break
            except subprocess.TimeoutExpired:
                pass
        else:
            out, err = p.communicate(timeout=30)
        self.assertEqual(p.returncode, 130, err)
        self.assertIn("interrupted: the load is stopped", err)
        self.assertEqual(list(self.out.iterdir()), [])
        self.assert_loads_ended()

    def test_a_signal_during_the_backlight_puts_it_back(self):
        self.max_mac("j514c", backlight=True)
        bl = self.tmp / "sys/class/backlight/apple-panel-bl/brightness"
        p = self.popen("M3_SURVEY_PHASE_S=3\nM3_SURVEY_REST_S=1\n")
        self.wait_for(lambda: bl.read_text() == "500\n", "the backlight at its maximum")
        p.send_signal(signal.SIGTERM)
        out, err = p.communicate(timeout=30)
        self.assertEqual(p.returncode, 130, err)
        self.assertEqual(bl.read_text(), "321\n")
        self.assertEqual(list(self.out.iterdir()), [])
        self.assert_loads_ended()

    def test_ctrl_c_in_the_wait_changes_nothing(self):
        self.max_mac("j514c", backlight=True)
        p = self.popen("M3_SURVEY_WAIT_S=20\n")
        time.sleep(1)
        os.killpg(p.pid, signal.SIGINT)
        out, err = p.communicate(timeout=30)
        self.assertEqual(p.returncode, 130, err)
        self.assertIn("Press Ctrl-C now to cancel", out)
        self.assertEqual(self.loads(), [])
        self.assertEqual(list(self.out.iterdir()), [])

    def test_too_hot_stops_early(self):
        self.max_mac("j514c", backlight=True)
        proc, tgz, files = self.survey(FAKE_HOT="1")
        self.assertEqual(len(tgz), 1, proc.stderr)
        self.assertEqual(self.phases(files), ["rest-before-idle", "idle", "rest-before-cpu-all", "cpu-all"])
        self.assertIn("cpu-all", files["phases.txt"].splitlines()[-1])
        self.assertIn("stopped: Tp0a read 101.0 C during cpu-all", files["phases.txt"])
        self.assertIn("result: stopped early: Tp0a read 101.0 C during cpu-all (limit 100 C)", files["summary.txt"])
        self.assertIn("the survey stopped early: Tp0a read 101.0 C during cpu-all", proc.stderr)
        self.assertEqual(len(self.loads()), 16)
        self.assertEqual((self.tmp / "sys/class/backlight/apple-panel-bl/brightness").read_text(), "321\n")
        self.assert_loads_ended()

    def test_a_hot_thermal_zone_stops_it_too(self):
        self.max_mac("j514c", zones=("macsmc-battery", "macsmc_soc_die"))
        (self.tmp / "sys/class/thermal/thermal_zone1/temp").write_text("100500\n")
        proc, tgz, files = self.survey()
        self.assertIn("tz:macsmc_soc_die read 100.5 C during rest-before-idle", files["summary.txt"])
        self.assertEqual(self.loads(), [])

    def test_implausible_readings_do_not_stop_it(self):
        # Outside -40..150 C is not a temperature, as macsmc-hwmon's die zone has it; a key that
        # is not a die key (TB0T) never stops it.
        self.max_mac("j514c")
        keys = self.tmp / "debug/macsmc-hwmon/keys"
        keys.write_text(SMC_KEYS.replace("Tg0a flt    4 80 -2850", "Tg0a flt    4 80 200000")
                        .replace("TB0T flt    4 80 30000", "TB0T flt    4 80 120000"))
        proc, tgz, files = self.survey()
        self.assertIn("result: completed", files["summary.txt"])

    def test_t6034_without_smc_keys(self):
        # No key list on this kernel: one clear line, then the CPU topology and the thermal zones.
        # With no die temperature to watch, no load runs.
        self.max_mac("j514m", smc=False, backlight=True)
        proc, tgz, files = self.survey()
        self.assertEqual(len(tgz), 1, proc.stderr)
        line = (f"no SMC key list on this kernel (no {self.tmp}/debug/*smc*/keys: debugfs is not mounted, "
                "or the SMC driver did not start)")
        self.assertIn(line, proc.stderr)
        self.assertEqual(files["smc-keys.txt"], line + "\n")
        self.assertIn(f"smc: {line}", files["summary.txt"])
        self.assertEqual(self.phases(files), ["rest-before-idle", "idle"])
        self.assertEqual(self.loads(), [])
        self.assertIn("P-cores: 4 5 6 7 8 9 10 11 12 13", files["cpus.txt"])
        self.assertIn("thermal zones: tz:macsmc-battery", files["cpus.txt"])
        self.assertTrue([l for l in files["samples.txt"].splitlines() if "tz:macsmc-battery 30000" in l])
        self.assertEqual((self.tmp / "sys/class/backlight/apple-panel-bl/brightness").read_text(), "321\n")
        self.assertIn("idle only: there is no CPU or SoC die temperature to watch", proc.stdout)

    def test_t6034_with_a_die_zone_runs_the_loads(self):
        self.max_mac("j514m", smc=False, cpus=14, zones=("macsmc_soc_die",))
        proc, tgz, files = self.survey()
        self.assertEqual(self.phases(files)[-1], "cpu-e")
        self.assertEqual(len(self.loads()), 14 + 10 + 4)

    def test_options(self):
        block = SRC[SRC.index('\nargs=()\nfor a in "$@"; do'):SRC.index('if preflight_needed "${1:-}"')]
        for args, want in [("--m3-power-survey", "ok --m3-power-survey"),
                           ("--m3-power-survey --m3-handoff", "error: --m3-handoff goes with an install"),
                           ("--m3-power-survey --read-only", "error: unexpected arguments after --m3-power-survey")]:
            with self.subTest(args=args):
                # The option defaults of this script (M3_PRO_MESA is the M3 Pro Mesa's, once merged).
                script = ('die() { echo "error: $*"; exit 1; }\nM3_TRY=0\nM3_GPU_EXPERIMENT=0\nM3_PRO_MESA=1\n'
                          f"set -- {args}\n{block}\necho \"ok ${{1:--}}\"\n")
                out = subprocess.run(["bash", "-c", script], capture_output=True, text=True).stdout
                self.assertTrue(out.startswith(want), out)
        self.assertIn("  --m3-power-survey) m3_power_survey ;;", SRC)
        self.assertIn("--m3-report | --m3-power-survey) return 1", SRC)


class SurveyFailureTest(SurveyTest):
    """Dave's round 1: a load runs only under a running sampler with current, valid die samples,
    and nothing is called restored unless it reads back."""

    test_refused_on_m1_and_m2 = test_phases_in_order_with_a_backlight = test_without_a_backlight = None
    test_the_wait_comes_first = test_ctrl_c_stops_the_load = test_a_signal_during_the_backlight_puts_it_back = None
    test_ctrl_c_in_the_wait_changes_nothing = test_too_hot_stops_early = test_a_hot_thermal_zone_stops_it_too = None
    test_implausible_readings_do_not_stop_it = test_t6034_without_smc_keys = None
    test_t6034_with_a_die_zone_runs_the_loads = test_options = None

    def assert_failed(self, proc, tgz, files, why, loads):
        self.assertNotEqual(proc.returncode, 0)
        errors = [l for l in ANSI.sub("", proc.stderr).splitlines() if l.startswith("error:")]
        self.assertEqual(len(errors), 1, proc.stderr)
        self.assertIn(f"the survey failed: {why}", errors[0])
        self.assertEqual(len(tgz), 1, proc.stderr)
        self.assertIn(f"result: failed: {why}", files["summary.txt"])
        self.assertNotIn("result: completed", files["summary.txt"])
        self.assertNotIn("Checked:", proc.stdout)
        self.assertNotIn("back as it was", proc.stdout + proc.stderr)
        self.assertEqual(len(self.loads()), loads)
        self.assertEqual((self.tmp / "sys/class/backlight/apple-panel-bl/brightness").read_text(), "321\n")
        self.assert_loads_ended()

    # Dave's reproduction: the die sensors exist, and the sampler is missing or dead.

    def test_an_empty_sampler_runs_no_load(self):
        self.max_mac("j514c", backlight=True)
        proc, tgz, files = self.survey("m3_survey_sampler_start() { :; }\n", check=False)
        self.assert_failed(proc, tgz, files, "the temperature sampler is not running (during rest-before-idle)", 0)
        self.assertEqual(files["samples.txt"], "")

    def test_a_dead_sampler_runs_no_load(self):
        self.max_mac("j514c", backlight=True)
        proc, tgz, files = self.survey("m3_survey_sampler_start() { true & M3_SURVEY_SAMPLER=$!; "
                                       "M3_SURVEY_STARTED=$(date +%s.%N); }\n", check=False)
        self.assert_failed(proc, tgz, files, "the temperature sampler is not running (during rest-before-idle)", 0)

    def test_a_silent_sampler_runs_no_load(self):
        # Running, but no sample: refused before the first load.
        self.max_mac("j514c", backlight=True)
        proc, tgz, files = self.survey("m3_survey_sampler_start() { sleep 30 & M3_SURVEY_SAMPLER=$!; "
                                       "M3_SURVEY_STARTED=$(date +%s.%N); }\n", check=False)
        self.assert_failed(proc, tgz, files,
                           "there is no valid CPU or SoC die temperature sample yet (before cpu-all)", 0)
        self.assertNotIn("cpu-all", self.phases(files))

    def test_only_implausible_die_samples_run_no_load(self):
        self.max_mac("j514c", backlight=True)
        keys = self.tmp / "debug/macsmc-hwmon/keys"
        keys.write_text(SMC_KEYS.replace(" 45000\n", " 990000\n").replace(" 40000\n", " -99000\n")
                        .replace(" -2850\n", " 400000\n"))
        proc, tgz, files = self.survey(check=False)
        self.assert_failed(proc, tgz, files,
                           "there is no valid CPU or SoC die temperature sample yet (before cpu-all)", 0)

    # The sampler breaks during the load: the load stops at once.

    def test_a_sampler_that_dies_during_the_load(self):
        self.max_mac("j514c", backlight=True)
        proc, tgz, files = self.survey("M3_SURVEY_PHASE_S=30\n", check=False, FAKE_KILL_SAMPLER="1")
        self.assert_failed(proc, tgz, files, "the temperature sampler is not running (during cpu-all)", 16)
        self.assertEqual(self.phases(files)[-1], "cpu-all")

    def test_a_stale_sampler_during_the_load(self):
        self.max_mac("j514c", backlight=True)
        proc, tgz, files = self.survey("M3_SURVEY_PHASE_S=60\nM3_SURVEY_STALE_S=1\n", check=False,
                                       FAKE_KEYS_HANG="1")
        self.assert_failed(proc, tgz, files,
                           "no valid CPU or SoC die temperature sample for more than 1 s (during cpu-all)", 16)

    def test_a_sampler_that_cannot_read_the_keys(self):
        self.max_mac("j514c", backlight=True)
        proc, tgz, files = self.survey("M3_SURVEY_PHASE_S=30\n", check=False, FAKE_KEYS_GONE="1")
        self.assert_failed(proc, tgz, files,
                           "the temperature sampler could not read the SMC key list (during cpu-all)", 16)

    def test_no_load_without_a_watched_temperature(self):
        # The idle-only run's guard: m3_survey_load itself refuses.
        self.max_mac("j514m", smc=False)
        proc = self.run_sh(f"{self.env_body()}M3_SURVEY_WATCH_RE=''\n"
                           "if m3_survey_load 0 1 2; then echo loaded; else echo \"refused: $M3_SURVEY_FAIL\"; fi")
        self.assertEqual(proc.stdout.strip(), "refused: no CPU or SoC die temperature is watched, so no load may run")
        self.assertEqual(self.loads(), [])

    # Dave's reproduction: the original brightness (321) can't be written back.

    TEE = """#!/bin/bash
v=$(cat)
if [[ -n ${FAKE_TEE_FAIL:-} && $v == "$FAKE_TEE_FAIL" ]]; then echo "tee: write error" >&2; exit 1; fi
printf '%s\\n' "$v" | exec /usr/bin/tee "$@"
"""

    def failing_tee(self):
        (self.tmp / "bin/tee").write_text(self.TEE)
        (self.tmp / "bin/tee").chmod(0o755)

    def test_a_failed_brightness_restore(self):
        self.max_mac("j514c", backlight=True)
        self.failing_tee()
        bl = self.tmp / "sys/class/backlight/apple-panel-bl"
        proc, tgz, files = self.survey(check=False, FAKE_TEE_FAIL="321")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual((bl / "brightness").read_text(), "1\n")
        command = f"echo 321 | sudo tee {bl}/brightness"
        self.assertIn(f"not restored: the backlight (apple-panel-bl) reads 1, not its original 321; "
                      f"restore it with: {command}", proc.stderr)
        self.assertIn("error: the survey could not put everything back", ANSI.sub("", proc.stderr))
        self.assertNotIn("Checked:", proc.stdout)
        self.assertNotIn("back as it was", proc.stdout + proc.stderr)
        self.assertNotIn("reads 321 again", proc.stdout + proc.stderr)
        self.assertEqual(len(tgz), 1)
        self.assertIn("result: completed", files["summary.txt"])
        self.assertIn("restore: FAILED\n  not restored: the backlight (apple-panel-bl) reads 1", files["summary.txt"])
        self.assert_loads_ended()
        # The command it prints puts it back.
        subprocess.run(["bash", "-c", command.replace("sudo ", "")], check=True, capture_output=True)
        self.assertEqual((bl / "brightness").read_text(), "321\n")

    def test_a_failed_brightness_restore_after_ctrl_c(self):
        self.max_mac("j514c", backlight=True)
        self.failing_tee()
        bl = self.tmp / "sys/class/backlight/apple-panel-bl/brightness"
        marker = self.tmp / "minimum-phase-active"
        extra = f"""
M3_SURVEY_PHASE_S=3
# Keep the minimum phase active until the test delivers the interrupt.
eval "$(declare -f m3_survey_sample | sed 1s/m3_survey_sample/m3_survey_sample_original/)"
m3_survey_sample() {{
  if [[ $1 == backlight-min ]]; then
    : > {marker}
    while :; do sleep 60; done
  else
    m3_survey_sample_original "$@"
  fi
}}
"""
        p = self.popen(extra, FAKE_TEE_FAIL="321")
        self.wait_for(marker.exists, "the active minimum-backlight phase", timeout=30)
        self.assertEqual(bl.read_text(), "1\n")
        os.killpg(p.pid, signal.SIGINT)
        out, err = p.communicate(timeout=30)
        self.assertEqual(p.returncode, 130, err)
        self.assertIn("interrupted. Nothing was written, and not everything is back as it was", err)
        self.assertIn(f"restore it with: echo 321 | sudo tee {bl}", err)
        self.assertNotIn("(checked)", err)
        self.assertEqual(bl.read_text(), "1\n")
        self.assert_loads_ended()

    def test_a_verified_restore_says_so(self):
        self.max_mac("j514c", backlight=True)
        proc, tgz, files = self.survey()
        self.assertIn("Checked: the load and the sampler have stopped, and the backlight reads 321 again.", proc.stdout)
        self.assertIn("restore: checked: the load and the sampler have stopped, the backlight reads 321 again",
                      files["summary.txt"])


# ---- the install path against 12.2 ---------------------------------------------------------------

ANSI = re.compile(r"\x1b\[[0-9;]*m")
MKTEMP = re.compile(r"(?:/tmp|" + re.escape(tempfile.gettempdir()) + r")/tmp\.[A-Za-z0-9]{6,}")
SUDO_LOG = 'fake_sudo() { echo "sudo $*" >>"$FAKE/log"; "$@"; }\nsudo=fake_sudo\n'


class SameAs122Test(MaxBase):
    """Every board, with 12.2's script and with this one: the same commands in the same order, the
    same files, and the same output but the block at the end of an install's summary."""

    def setUp(self):
        super().setUp()
        old = subprocess.run(["git", "show", f"{REL_12_2}:tools/aurora-sep/install-aurora-sep.sh"],
                             cwd=flow.INSTALLER.parent, capture_output=True)
        if old.returncode:
            self.skipTest(f"git can't show 12.2's script ({REL_12_2})")
        self.old = self.tmp / "install-12.2.sh"
        self.old.write_bytes(old.stdout)
        self.old_version = re.search(rb"^VERSION=(\S+)$", old.stdout, re.M).group(1).decode()
        self.assertEqual(re.search(rb"^TAG=(\S+)$", old.stdout, re.M).group(1).decode(), "sep-" + self.old_version)

    def as_this_release(self, run):
        # 12.2's run with its release number read as this one's: a release names its own packages,
        # tag and boot.bin copy, and nothing else may differ.
        old, new = self.old_version, VERSION
        result = pro.as_this_release(run, old, new)
        result["out"] = [pro.release_text(x, old, new) if x else x for x in run["out"]]
        return result

    def boards(self):
        for board, compat in flow.BOARDS.items():
            # An M3 Pro gets the M3 Pro's Mesa once that is merged (test_m3_pro_mesa compares the rest).
            if "apple,t6030" in compat and "M3_PRO_MESA_PACKAGE" in SRC:
                continue
            # From 12.4 the M3 Airs get it too (test_m3_air_mesa, test_m3_air_default).
            if board in ("j613", "j615") and "m3_mesa_mac() { is_m3_pro || is_m3_air; }" in SRC:
                continue
            # The J613 gets m1n1's display handoff by default from this release on
            # (test_m3_air_default compares every other board with 12.3).
            if board == "j613" and "j613" in re.search(r'^M3_HANDOFF_BOARDS="([^"]*)"$', SRC, re.M).group(1).split():
                continue
            yield board

    def reset(self, board):
        keep = {"pkgs", "bin", "out"}
        for p in self.tmp.iterdir():
            if p.name in keep or p.name.startswith(("root-", "install-", "reader")):
                continue
            shutil.rmtree(p) if p.is_dir() and not p.is_symlink() else p.unlink()
        for d in ("dt/chosen", "esp/m1n1", "esp/asahi", "etc/default", "state", "fake"):
            (self.tmp / d).mkdir(parents=True, exist_ok=True)
        self.mac(board)

    def tree(self):
        out = {}
        for p in sorted(self.tmp.rglob("*")):
            rel = p.relative_to(self.tmp).as_posix()
            if rel.startswith(("pkgs/", "bin/", "fake/log", "root-", "install-", "reader")) or not p.is_file():
                continue
            out[rel] = p.read_bytes()
        return out

    def run_with(self, installer, board):
        self.reset(board)
        self.installer = installer
        try:
            a = self.run_sh(SUDO_LOG + "M3_TRY=0\ninstall_all", check=False)
            b = self.run_sh(SUDO_LOG + "uninstall_all", check=False) if a.returncode == 0 else None
        finally:
            self.installer = flow.INSTALLER
        norm = lambda s: MKTEMP.sub("<tmp>", s)
        return {"codes": (a.returncode, b and b.returncode), "log": norm(self.log()), "tree": self.tree(),
                "out": [norm(a.stdout), norm(a.stderr), b and norm(b.stdout), b and norm(b.stderr)]}

    def test_every_board_runs_as_on_12_2(self):
        seen = []
        for board in self.boards():
            with self.subTest(board=board):
                before = self.as_this_release(self.run_with(self.old, board))
                after = self.run_with(flow.INSTALLER, board)
                seen.append(board)
                self.assertEqual(after["codes"], before["codes"])
                # The same commands: as a multiset, and in order among the $sudo lines and among
                # the others. The two sides of a pipeline such as "pacman -Q ... | $sudo tee ..."
                # log in either order (test_m3_pro_mesa.same_commands).
                pro.same_commands(self, before["log"], after["log"])
                self.assertEqual(sorted(after["tree"]), sorted(before["tree"]))
                for path, data in before["tree"].items():
                    self.assertEqual(after["tree"][path], data, path)
                # Only the undo label and the next-steps block may differ; commands stay exact.
                old_out, new_out = (pro.refresh_notice(normalize_undo_label(run["out"][0])) for run in (before, after))
                self.assertTrue(new_out.startswith(old_out), board)
                extra = new_out[len(old_out):]
                compat = flow.BOARDS[board]
                group_c = ("apple,t6031" in compat or "apple,t6034" in compat
                           or ("apple,t8122" in compat and board not in ("j613", "j615")))
                if group_c:
                    self.assertIn("NEXT STEPS: M3 BRING-UP", extra)
                    self.assertTrue(extra.startswith("\n====="), extra)
                else:
                    self.assertEqual(extra, "")
                self.assertEqual([pro.refresh_notice(x) if x else x for x in after["out"][1:]],
                                 [pro.refresh_notice(x) if x else x for x in before["out"][1:]])
        # The M3 Airs get mesa-m3 from 12.4 on: test_m3_air_default compares them with 12.3.
        airs_have_mesa = "m3_mesa_mac() { is_m3_pro || is_m3_air; }" in SRC
        for board in ("j514c", "j516c", "j514m", "j516m", "j504", "j433", "j615", "j314s", "j700"):
            if board == "j615" and airs_have_mesa:
                continue
            self.assertIn(board, seen)


class ReleaseTest(unittest.TestCase):
    def test_the_adt_reader_asset(self):
        # Empty until the reader is in this tree; then its file name and its sha256.
        entry = re.search(r'^M3_ADT_READER="([^"]*)"$', SRC, re.M).group(1)
        reader = flow.INSTALLER.parent / "aurora-adt-extract.py"
        if reader.exists():
            self.assertEqual(entry, f"aurora-adt-extract.py {hashlib.sha256(reader.read_bytes()).hexdigest()}",
                             "M3_ADT_READER must name tools/aurora-sep/aurora-adt-extract.py and its sha256")
        else:
            self.assertEqual(entry, "")

    def test_the_survey_is_part_of_the_installer(self):
        # No extra release asset: it ships inside install-aurora-sep.sh.
        self.assertIn("m3_power_survey() {", SRC)
        self.assertNotRegex(SRC, r"(?m)^M3_SURVEY_SCRIPT=")


if __name__ == "__main__":
    unittest.main()
