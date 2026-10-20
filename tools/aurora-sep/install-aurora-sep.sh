#!/bin/bash
# Install the aurora custom/sep kernel and Touch ID on an Omarchy Mac.
#
#   curl -fsSL https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh | bash
#   ... | bash -s -- --read-only      install, but never let the driver write to the enclave
#   ... | bash -s -- --uninstall      go back to the kernel this Mac had before
#   ... | bash -s -- --reset-touchid  start Touch ID over: new keybag, enrol again
#   ... | bash -s -- --m3-report      M3: write a bring-up report to attach to an issue (read-only)
#   ... | bash -s -- --m3-power-survey   M3: sample the SMC's temperature and power keys through
#                                         short CPU and backlight loads (opt-in, about 5 minutes)
#   ... | bash -s -- --m3max-kit          16-core M3 Max (j514c, j516c): the test kit. One consent
#                                         screen, then the kernel and the boot loader variant, and
#                                         test boots one at a time across restarts, by itself; one
#                                         file to upload at the end. Then, at any time:
#                                         sudo aurora-m3max-kit --status | --stop | --restore
#   ... | bash -s -- --m3-gpu-experiment   M3 MacBook Air only: install the GPU start
#                                         experiment's scripts with the kernel (arms nothing)
#   ... | bash -s -- --m3-gpu-persistent   J613 on current14: install the matched stack and
#                                         select experimental GPU acceleration for later boots
#   ... | bash -s -- --m3-gpu              J613: detect the supported GPU profile and persist it
#   ... | bash -s -- --m3-profile=j613-25g83   J613 already on exact25G83: select the matched
#                                              experimental native OpenGL profile (no migration)
#   ... | bash -s -- --m3-profile=j615-25g83   J615 on exact25G83: the same profile, EXPERIMENTAL
#                                              and untested on a J615 (a release must list it)
#   ... | bash -s -- --archive-esp-history  Archive old unreferenced EFI history before installing
#   ... | bash -s -- --esp-history          Read-only EFI history inventory
#   ... | bash -s -- --desktop-fixes  Optional matched stable desktop fixes; unsupported versions are preserved.
#   ... | bash -s -- --no-m3-mesa     M3 Pro: leave out the M3 Pro's Mesa (installed by default)
#
# Kernel: iconidentify/aurora-linux custom/sep (dbbfb92908ba), aurora-silicon/linux aurora-wip plus the
# Secure Enclave (Touch ID) driver, Thunderbolt (#8), the Apple video
# decoder (#45), the M2 Max (t6021) profile and the consolidated Touch ID
# series (aurora-silicon/linux#69: matching after a reboot on every profile,
# M1 Pro J314s, j293 SPI mode, a scan in progress ended before sleep),
# Thunderbolt displays on M1 and M1 Pro/Max by default, and two displays
# through one dock on every M2 Pro/Max laptop (aurora-silicon/linux#8, with
# #46, #50 and #64), and the MacBook Neo (J700) work from #40, #42,
# #43, #54 and #55, including its Touch ID device tree.
# 11.20 adds Touch ID on the MacBook Air M2 15" (J415, aurora-silicon/linux#152),
# a retry from a fresh sensor registration when the first sensor patch fails at
# boot, battery time estimates that read "no data" instead of 0 (#53), the
# Omarchy boot logo in place of Tux, and aurora-wip through 3bb0a6104a11 (the
# MacBook Neo video decoder, PCIe bring-up and radio drivers, inert elsewhere).
# On a MacBook Neo the script keeps the Neo's own m1n1.
# 11.21 adds Bluetooth recovery for the Broadcom PCIe controllers: a radio
# wedged by an rfkill cycle under heavy traffic is reset on the next open, and
# its calibration and address restored (aurora-silicon/linux#7).
# 11.22 installs the VA-API bridge for the video decoder too, so players use
# the hardware decoder instead of falling back to software, and
# aurora-touchid-setup --help prints its usage instead of starting an enrolment.
# The display manager waits (up to 30 s) for the Apple display driver, so a
# late driver can't leave the built-in screen black, and reloading the Touch ID
# driver on M1 is refused with "reboot to re-attach" instead of hanging.
# 11.23 routes USB-C displays on the M2 Pro/Max MacBook Pros in the order the
# compositor pairs them, so two monitors attached at boot each keep their own
# modes, and describes the Touch ID sensor on every M2 Pro/Max MacBook Pro.
# 11.24 adds Touch ID on the MacBook Pro 14"/16" M1 Max and the MacBook Air 13"
# M2, untested on both. On those boards the script first keeps the boot.bin the
# Mac booted with on the EFI partition and prints how to restore it from macOS.
# 11.25 adds the in-tree Apple Neural Engine driver (aurora-silicon/linux#155),
# switched on for the M1 Max and M2 Max only, and the read-only SEP diagnostics
# under /sys/bus/platform/devices/*.sep/diag/. When omarchy-ane-dkms is
# installed the script says so: its modules take precedence over this kernel's.
# 11.25.1 (installer only, same packages): rebuilding m1n1's stage 2 now keeps
# the rest of /etc/default/update-m1n1, so a MacBook Neo keeps its own M1N1=
# and U_BOOT= instead of being rebuilt from an m1n1 that cannot boot it.
# 11.31 brings up PCIe over Thunderbolt on the M1 Pro/Max and M2 Pro/Max by
# default (iconidentify/aurora-linux#10, Wesley Grimes, with follow-up fixes;
# pcie_apple.tunnel_kernel_init=0 turns it off): USB, Ethernet and audio
# behind Thunderbolt docks and displays now work there as they do on M1. It
# builds the drivers for the Intel Ethernet in Thunderbolt 3 and 4 docks (igb
# and igc), and carries the MacBook Neo's Wi-Fi in the shared kernel
# (iconidentify/aurora-linux#11). On a Neo the radios need that Neo's own
# firmware, calibration and country files, which the script checks for; sleep
# is refused while they are active, and Bluetooth is off by default.
# The Neural Engine now powers off when idle (aurora-silicon/linux#155, Joshua
# Warren), and is switched on for the M1, M1 Pro and M2 Pro as well as the M1
# Max and M2 Max.
# 11.32 carries three Touch ID fixes from Justin Pfister's review
# (aurora-silicon/linux#69): a deleted fingerprint is saved as deleted and no
# longer returns after a reboot; an M2 Pro/Max on system firmware 26.2 or
# earlier uses the 13.5 key store, whose keybag that enclave accepts; and the
# kernel's random-number thread no longer asks the enclave for data while the
# Mac sleeps, which left Touch ID failing after resume on an M2 Pro.
# 11.33 adds --reset-touchid, which starts Touch ID over on a Mac whose
# keybag no longer loads (such an M2 Pro/Max after a macOS update): it moves
# the Touch ID state aside for a new keybag at the next boot. Enrolment
# failures are now logged with their cause, and aurora-touchid-setup verifies
# as root, so it also completes over SSH.
# 11.34 sets the brightness of an Apple Studio Display through DCP
# (iconidentify/aurora-linux#15, Wesley Grimes): brightnessctl -d 'dcp-DP-*',
# and nothing is sent until a level is chosen. On the M1/M2 Pro and Max it
# keeps the Thunderbolt root port's link out of ASPM L1, so unplugging an idle
# dock no longer leaves the port dead until reboot.
# 11.35 lets a Touch ID enrolment take up to 36 captures, enough for the 16 to
# 22 an M2 Pro needs (iconidentify/aurora-linux#17, Justin Pfister), and
# reports one stage per capture, so every accepted touch shows progress.
# 11.36 is one kernel for M1, M2 and M3: on M1 Pro/Max laptops a second
# USB-C or Thunderbolt display gets the HDMI port's pipeline when HDMI is idle,
# and a dock display left waiting lights once a pipeline frees. M3 support is
# experimental and kernel-only (see is_m3).
# 11.109-test (not a release): the M3 Pro display handoff on the 14" J514S.
# Kernel 11.109 takes each Mac's PMP values from the boot loader, and m1n1
# aurora7 passes them (/chosen/asahi,t6030-pmp). For --m3-handoff on a J514S.
# 11.36.1 changes only this script and adds an M3 m1n1: an M3 Pro model whose
# m1n1 display and GPU handoff has been booted (the 16" J516S for now) gets
# m1n1-aurora with it switched on, and the built-in display and GPU work.
# Every other M3 stays kernel-only, and --m3-handoff lets an M3 Pro owner try
# the handoff on a model not yet on the list (see --agent-prompt). M1, M2 and
# the Neo are unchanged.
# 11.37: the unified M1/M2/M3 kernel, now 11.110 (aurora-linux air/t8122). Its
# only device-tree change from 11.36 is the five M3 (T8122) trees; the T8122
# GPU and display nodes stay disabled and the driver fails closed, so without
# an opt-in M3 Air m1n1 (shipped only in the -test channel) an Air is kernel-
# only, exactly as in 11.36. The 16" J516S keeps its display + GPU handoff
# (now m1n1 aurora7, which also passes each Mac's PMP values). --m3-handoff is
# available on the 14" J514S (opt-in, not yet on by default). This release adds
# --m3-report and refuses chips it does not support (SUPPORTED_SOCS; #32).
# 11.38: the same device trees and boot loaders as 11.37. The display driver
# checks each display's modes and attributes before using them and refreshes a
# connector's state before reporting a plug or unplug; the M3 Pro PMP's boot
# settings and startup tables are checked before it starts; the M3 GPU
# settings are per chip; each PCI USB controller reserves only the interrupt it
# uses, leaving interrupts for a dock's other devices. An M3 Air stays kernel-only.
# 12.0: one release and one kernel for every Mac this script supports; the
# 11.1xx test builds end here. On the 16" M3 Pro (J516S) the kernel drives
# external displays on the USB-C and HDMI ports, and its m1n1 also hands over
# the second external display processor. The M3 Type-C PHY changes behind this
# apply to every M3. Other M3 Pros keep the handoff opt-in (--m3-handoff). An
# M3 MacBook Air stays kernel-only unless its owner asks with --m3-handoff for
# m1n1's display handoff with GPU diagnostics (the desktop stays on the boot
# framebuffer). An Air that has an earlier test build's boot loader must ask
# again with --m3-handoff.
# Every Mac that gets an m1n1 from this script now gets the same one,
# m1n1-aurora aurora12: M1 and M2 move to it from aurora3, and the M3s on the
# handoff path from aurora7 and 8.x. Only the switches in /etc/m1n1.conf
# differ between Macs. Each Mac's boot.bin is kept on the EFI partition before
# it is rebuilt, with the steps to put it back.
# The kernel also describes the Air's internal display and GPU configuration,
# both inert for now, and the M2 Max Neural Engine cleans up after a failed
# probe (iconidentify/aurora-linux#37, Joshua Warren). M1, M2 and the Neo
# keep their boot loaders.
# 12.1: on M1 and M2, a direct USB-C display's route follows the display
# pipeline the desktop gives it (iconidentify/aurora-linux#39;
# appledrm.typec_follow_crtc=0 turns it off). Its boot loader and device
# trees are 12.0's.
# 12.2 is 12.1 plus a GPU start experiment for the M3 MacBook Air, off unless
# the Air's owner asks for it. The kernel starts the Air's GPU firmware only
# with asahi.t8122_start=1 on the kernel command line. --m3-gpu-experiment
# (Air only) installs the scripts that arm that for one boot and collect the
# result, with a separate G15G Mesa build in /opt/mesa-m3-g15g; it arms
# nothing itself. Every Mac moves to m1n1-aurora aurora12.1: aurora12 plus a
# stand-in power model for the Air's GPU handoff, which stays off unless
# /etc/m1n1.conf has both chosen.asahi,t8122-gpu=1 and
# chosen.asahi,t8122-gpu-power-standin=1. This script writes neither line.
# Without them every Mac boots as it did with aurora12 (12.0 and 12.1).
# 12.3: every M3 Pro (T6030) gets the M3 Pro's Mesa by default: mesa-m3, a
# pacman package with Mesa and the M3 Pro GPU driver in /opt/mesa-m3, never
# over the system Mesa, and a login hook that uses it only on an M3 Pro whose
# GPU is up. It is installed in a pacman transaction of its own after the
# kernel. If it can't be (a package it needs is missing or too old, or pacman
# fails), the kernel install still completes, the summary says so and the
# exit status is 3. Its udev rule gives the GPU's render node to group render,
# so the desktop user goes into that group (from the next login). The summary
# says how to see why the GPU graphics are on or off at a login and how to
# switch them off. --no-m3-mesa leaves it out; --uninstall removes it, and
# takes the user out of render only if this script put them there. M1, M2,
# the M3 Max and the M3 MacBook Air download and install nothing of it.
# Also in 12.3, for every M3: --m3-report writes one bring-up report with the
# host name, user names, serial numbers and MAC addresses masked, including an
# allowlist of the boot loader's device tree (read once through the phram
# driver, which the kernel now builds but never loads at boot);
# --m3-power-survey samples the SMC's temperature and power keys through short
# CPU and backlight loads (opt-in). An M3 that stays kernel-only (the M3 Max,
# and an M3 that is not an Air) ends its install with a NEXT STEPS box. The
# kernel reads the 14-core M3 Max's (t6034) SMC sensors as an M3's.
# 12.4: the M3 MacBook Air (j613, j615) gets mesa-m3 too, the same package and
# asset as the M3 Pro (now built for both GPUs), with its desktop user added to
# group render, recorded and undone as on the M3 Pro. Its login hook leaves the
# Air's GPU off (reason "experimental", software rendering) unless
# /etc/mesa-m3/t8122-gpu-experiment exists, which only --m3-gpu-experiment
# writes (and --uninstall removes). The experiment's own Mesa, mesa-m3-g15g,
# is gone: an Air that has it loses it, recorded, just before mesa-m3 goes on,
# and air-gpu-job.sh runs with /opt/mesa-m3. The hook's reason not-t6030 is now
# not-supported. On the M3 Pro, nothing else changes.
# Also in 12.4: the 13" M3 MacBook Air (j613) gets m1n1's display handoff by
# default (chosen.asahi,t8122-dcp=1 with chosen.asahi,t8122-gpu-handoff-diag=1),
# so the kernel drives its built-in display; an owner's own chosen.<name>=0 line
# in /etc/m1n1.conf switches one off. The j615 stays kernel-only. Every Mac
# moves to m1n1-aurora aurora13: aurora12.1 plus, on a j613 only, a read of the
# GPU's leakage fuses at every boot, logged and published for Linux and used
# only with chosen.asahi,t8122-gpu-fuse-leakage=1 in the GPU experiment. The
# kernel adds the Air's GPU (still only with asahi.t8122_start=1), its display,
# trackpad haptics for every MacBook (the host-driven click off by default),
# and keeps a late USB-C display on the M3 Pro connected while its modes are
# re-described.
# It replaces linux-asahi (or linux-aurora) as a pacman package,
# so mkinitcpio and update-m1n1 run from their own hooks; on a GRUB Mac this
# script regenerates grub.cfg and keeps the previous kernel as a fallback entry.
#
# Touch ID: libfprint with the Apple SEP driver, fprintd, the apple-sep
# service, a sleep hook that stops fprintd before suspend (so a lock screen
# waiting for a finger at lid close can't leave the sensor claimed), and
# this Mac's own sensor calibration. After the reboot, run
# aurora-touchid-setup to enrol a finger and use it for sudo and the lock screen.
#
# m1n1: one m1n1-aurora (M1N1_PACKAGE) for every Mac. It builds
# AsahiLinux/m1n1 main (3e354a24), which knows the macOS 26.5 to 27.0 firmware
# the MacBook Neo ships with, plus Omarchy's patches: the SEP
# warm-registration guard and preboot-UUID forwarding from aurora-silicon/m1n1,
# the usb4-N-pcie-adapter alias fallback (aurora-silicon/m1n1#4), and the M3
# display and GPU handoff, which stays off unless /etc/m1n1.conf arms it.
# From aurora12.1 it also carries the M3 Air GPU handoff's stand-in power
# model, off unless /etc/m1n1.conf arms it too.
# Before boot.bin is rebuilt with it, the boot.bin the Mac booted with is kept
# on the EFI partition (keep_bootbin_on_esp). A MacBook Neo keeps its own m1n1
# unless NEO_AURORA_M1N1 is 1.
# M3: experimental. linux-aurora goes on. On an M3 Pro model in
# M3_HANDOFF_BOARDS (or with --m3-handoff), m1n1-aurora replaces m1n1 and
# three chosen.asahi,t6030-* lines in /etc/m1n1.conf switch its display and
# GPU handoff on. Every other M3 keeps m1n1's boot.bin exactly as it is: no
# m1n1-aurora, no update-m1n1 run, and a freeze on update-m1n1 unless one is
# in place already (see m3_plan). Every M3 Pro and M3 MacBook Air also gets
# mesa-m3 (see the M3's Mesa below, m3_pro_mesa_*).
# On M2 and later the platform hands Linux an already-running Secure
# Enclave, and the driver attaches to it with one registration that can only be
# sent once per boot. Stock m1n1 asks the enclave for randomness on the way up,
# spending that attempt before Linux sees it. aurora-silicon/m1n1 has a guard
# that skips the request when the enclave is already running. On M1, where the
# enclave is still in its boot ROM, the guard never fires.
#
# Testing this build? Run with --agent-prompt for the test plan and the format
# to report results in.
set -euo pipefail

# The script this run reads, for the M3 Pro's Mesa record (m3_pro_mesa_record): its sha256 when
# bash reads it from a regular file (bash /path/install-aurora-sep.sh), hashed here, before
# anything else runs. Through a pipe (curl ... | bash) it can't be read again: then
# "unavailable" and source "stdin", never a guess.
SELF_SOURCE=stdin SELF_SHA256=unavailable SELF_PATH=""
case ${BASH_SOURCE[0]:-} in
  "" | /dev/* | /proc/*) ;;
  *)
    if [[ -f ${BASH_SOURCE[0]} ]]; then
      SELF_SOURCE="file" SELF_PATH=${BASH_SOURCE[0]}
      SELF_SHA256=$(sha256sum <"${BASH_SOURCE[0]}" 2>/dev/null | cut -d' ' -f1) || SELF_SHA256=""
      [[ $SELF_SHA256 =~ ^[0-9a-f]{64}$ ]] || SELF_SHA256=unavailable
    fi
    ;;
esac
# This run's id, for the M3 Pro's Mesa record and the summary line that names it, so a check can
# match the record to the output of the run that wrote it.
RUN_ID=$(cat /proc/sys/kernel/random/uuid 2>/dev/null) || RUN_ID=""
[[ $RUN_ID =~ ^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$ ]] || RUN_ID=unavailable

# The kernel package version and the release tag move independently: a release
# that only changes m1n1 reuses the previous kernel packages unchanged.
VERSION=7.1.12.aurora2-12.4
TAG=sep-7.1.12.aurora2-12.4
# Packages are fetched from this script's own tag, never from "latest": the
# checksums below belong to this release and nothing else.
PUBLIC_RELEASE_URL=https://github.com/omacom/linux-aurora/releases/download/$TAG
PUBLIC_RELEASES_API=https://api.github.com/repos/omacom/linux-aurora/releases
# AURORA_RELEASE_URL and AURORA_RELEASES_API are a staging or mirror override
# for these two (a file://, http:// or https:// URL; see release_source); the
# checksums below still decide what is installed. The commands this script
# prints name the public release (PUBLIC_RELEASE_URL or LATEST_URL), never the
# override: a staged copy can go away, and the checksums don't cover the
# script itself.
RELEASE_URL=${AURORA_RELEASE_URL:-$PUBLIC_RELEASE_URL}
RELEASES_API=${AURORA_RELEASES_API:-$PUBLIC_RELEASES_API}
# Where to always get the current script, whatever this copy turns out to be.
LATEST_URL=https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh
PACKAGES=(
  "linux-aurora-$VERSION-aarch64.pkg.tar.zst fded7d73e13dbe7f82722b925a257835761584203c5189337a251abab298bbde"
  "linux-aurora-headers-$VERSION-aarch64.pkg.tar.zst a8da8cd00a40010ebe6096ae50c53b1ae461ae6570aa437f5b79a02f10b80cca"
  "libfprint-1.94.100-1.1-aarch64.pkg.tar.zst bc7d9762db6644f2cfb58ddb209602c1d513845eb1498c098e01f12600fcbdf9"
  "aurora-touchid-20261003-1-any.pkg.tar.zst 29b0360fac8c257d754e64bd1b9c33c487eb2595dd3c31e9138d7a476afa3d64"
)
# The one m1n1 for every Mac, as "file sha256": M1 and M2, an M3 on the handoff
# path (see m3_plan), and the MacBook Neo once NEO_AURORA_M1N1 is 1. Macs differ
# only in the switches /etc/m1n1.conf arms (m3_switches), never in the binary.
M1N1_PACKAGE="m1n1-aurora-1.6.1.aurora13-1-aarch64.pkg.tar.zst 23c7aff0f7b2da5ce272bce0eab7893e01091ce4d9350e4e650af1e9195208fa"
# The sha256 of the m1n1.bin in M1N1_PACKAGE: the bytes update-m1n1 puts at
# the start of boot.bin. The script tells m1n1 builds apart by these bytes,
# never by the version string they report: aurora8.5-1 and 8.5-2 both reported
# v1.6.1-omarchy.aurora8.5.
M1N1_BIN_SHA=bf7b74a4648216432ac36171d896b11fa0584da19b242b16a11aa055963091de
# 0: a MacBook Neo keeps its own m1n1 (its M1N1= and U_BOOT= in
# /etc/default/update-m1n1), as before 12.0. 1: it gets M1N1_PACKAGE like every
# other Mac, and update-m1n1 builds its boot.bin from that m1n1 and the Neo's
# own U-Boot. Set it to 1 only once M1N1_PACKAGE carries every patch of the
# Neo's own m1n1 (aurora-silicon/m1n1, J700) and has booted on a Neo.
NEO_AURORA_M1N1=0
PINNED="linux-aurora linux-aurora-headers libfprint m1n1-aurora"
PIN_BEGIN="# >>> aurora-sep pin (remove with: install-aurora-sep.sh --uninstall)"
PIN_END="# <<< aurora-sep pin"
STATE=/var/lib/aurora-sep
FALLBACK_ID=aurora-sep-previous-kernel
# Written only by --read-only; keeps the driver from writing to the enclave.
MODPROBE_CONF=/etc/modprobe.d/aurora-sep.conf
READ_ONLY=0
DESKTOP_FIXES=0
DESKTOP_FIXES_DATA=""
DESKTOP_FIXES_PACKAGES=""
DT=/proc/device-tree

say() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33mwarning:\033[0m %s\n' "$*" >&2; }
die() { printf '\033[1;31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# Downloads $1 to $2. A dropped connection (an HTTP/2 stream reset, a timeout, a cut transfer) is not a missing file:
# curl --retry skips most of those, and it restarts from byte 0, so each try here is its own curl call that continues from
# the bytes already on disk. Returns 0 on success (a 416 answer means the file was already complete; the checksum check
# after the call still decides). Returns 22 for HTTP 404/410, and 101 for other non-retryable client errors.
# After five failed tries it returns 100 if the last answer was a server error (HTTP 408, 429 or 5xx), else curl's last
# exit code. Exit 33 (the server ignored the Range request) removes the partial file, so the next try starts at byte 0.
fetch_release_file() {
  local url=$1 out=$2 try rc=0 code why last
  for try in 1 2 3 4 5; do
    code=$(curl -fL --progress-bar --connect-timeout 30 --speed-limit 1024 --speed-time 60 -C - -o "$out" \
      -w '%{http_code}' "$url") && return 0
    rc=$?
    [[ $rc == 22 && $code == 416 ]] && return 0
    if [[ $rc == 22 && $code == 4* && $code != 408 && $code != 429 ]]; then
      [[ $code == 404 || $code == 410 ]] && return 22
      return 101
    fi
    last=$rc why="curl exit $rc"
    if [[ $rc == 22 ]]; then last=100 why="HTTP $code"; fi
    if [[ $rc == 33 ]]; then rm -f "$out"; fi
    if (( try < 5 )); then
      warn "the download of ${out##*/} was interrupted ($why); trying again ($try of 4)"
      sleep $((try * 2))
    fi
  done
  return "$last"
}

sudo=""
if (( EUID != 0 )); then
  command -v sudo >/dev/null || die "run as root or install sudo"
  sudo=sudo
fi

# AURORA_RELEASE_URL and AURORA_RELEASES_API go to curl and into printed
# text, so only a plain file://, http:// or https:// URL is taken: nothing curl
# could read as an option, and no spaces, control characters or other bytes
# outside printable ASCII (percent-encode those).
override_ok() {
  local LC_ALL=C
  [[ $1 != -* && $1 =~ ^(file|https?)://. && $1 != *[![:graph:]]* ]]
}

# True when either override points away from the public release.
release_overridden() {
  [[ $RELEASE_URL != "$PUBLIC_RELEASE_URL" || $RELEASES_API != "$PUBLIC_RELEASES_API" ]]
}

# Checked before the first download or release lookup that uses them. A run
# from a staging or mirror copy says so once, at the start.
release_source() {
  local name shown
  for name in AURORA_RELEASE_URL AURORA_RELEASES_API; do
    [[ -z ${!name:-} ]] || override_ok "${!name}" ||
      die "$name must be a file://, http:// or https:// URL without spaces or
    control characters (percent-encode anything outside printable ASCII).
    Unset it to use the public release. Nothing was installed."
  done
  if ! release_overridden; then
    return 0
  elif [[ $RELEASE_URL != "$PUBLIC_RELEASE_URL" && $RELEASES_API != "$PUBLIC_RELEASES_API" ]]; then
    shown="$RELEASE_URL (release list $RELEASES_API)"
  elif [[ $RELEASE_URL != "$PUBLIC_RELEASE_URL" ]]; then
    shown=$RELEASE_URL
  else
    shown="release list $RELEASES_API"
  fi
  say "Using a staging/mirror copy: $shown; checksums are still verified"
}

# A saved copy of this script keeps installing its own build forever. Tell the
# operator - human or agent - when a newer one exists. Never fatal: no network,
# rate limit or API change should stop an install that was going to work.
release_order() {
  local tag=$1 year month day limit increment
  if [[ $tag =~ ^sep-([A-Za-z0-9._]+)-([0-9]+(\.[0-9]+)*)(-stable)?$ ]]; then
    printf '0.%s%s\n' "${BASH_REMATCH[2]}" "${BASH_REMATCH[4]}"
  elif [[ $tag =~ ^aurora-([0-9]{4})\.([0-9]{2})\.([0-9]{2})(\.([0-9]+))?$ ]]; then
    year=${BASH_REMATCH[1]} month=${BASH_REMATCH[2]} day=${BASH_REMATCH[3]}
    increment=${BASH_REMATCH[5]:-0}
    (( 10#$year > 0 && 10#$month >= 1 && 10#$month <= 12 && 10#$day >= 1 )) || return 1
    case $month in
      04|06|09|11) limit=30 ;;
      02) limit=28; (( 10#$year % 4 == 0 && (10#$year % 100 != 0 || 10#$year % 400 == 0) )) && limit=29 ;;
      *) limit=31 ;;
    esac
    (( 10#$day <= limit )) || return 1
    printf '1.%s.%s.%s.%s\n' "$year" "$month" "$day" "$increment"
  else
    return 1
  fi
}

newer_release() {
  local seen="" mine theirs
  # Only the Latest release and printable tag tokens are admitted. An unavailable
  # API or an unknown tag must not stop installation or enter the terminal notice.
  seen=$(curl -fsSL --max-time 8 "$RELEASES_API/latest" 2>/dev/null |
    LC_ALL=C grep -o '"tag_name"[[:space:]]*:[[:space:]]*"[A-Za-z0-9._-]*"' |
    head -1 | sed 's/.*"\([A-Za-z0-9._-]*\)"$/\1/') || true
  mine=$(release_order "$TAG") || return 0
  theirs=$(release_order "$seen") || return 0
  # Calendar releases follow legacy releases; legacy ordering ignores the kernel
  # version. Calendar ordering uses the date and optional numeric increment.
  [[ $theirs != "$mine" &&
    $(printf '%s\n' "$mine" "$theirs" | LC_ALL=C sort -V | tail -1) == "$theirs" ]] && echo "$seen"
  return 0
}

# A staging or mirror run names a newer release but points at nothing: the
# public script is not what it is testing.
version_notice() {
  local seen
  seen=$(newer_release)
  if [[ -n $seen ]] && release_overridden; then
    warn "$seen is newer than $TAG, which this script installs"
  elif [[ -n $seen ]]; then
    warn "this script installs $TAG, but $seen is published.
    You are probably running a saved copy. The current one:
      curl -fsSL $LATEST_URL | bash"
  elif ! release_overridden; then
    say "$TAG is the current release"
  fi
}

# The same check for --agent-prompt, on stderr only.
prompt_notice() {
  local seen
  seen=$(newer_release)
  if [[ -n $seen ]] && release_overridden; then
    warn "$seen is newer than $TAG, which this plan is for"
  elif [[ -n $seen ]]; then
    warn "the current release is $seen; this plan is for $TAG"
  fi
  return 0
}

boot_chain() {
  # /boot/efi can be readable only by root, so test through sudo. Limine is
  # active where omarchy-mac-boot is installed, or where its activation marker
  # and defaults exist without it (the test omarchy-mac-limine-active and
  # limine-mkinitcpio-hook's Apple gate make), as on Macs switched by hand.
  if { pacman -Q omarchy-mac-boot >/dev/null 2>&1 ||
    [[ -f /var/lib/omarchy/limine.enabled && -f /etc/default/limine ]]; } &&
    { $sudo test -e /boot/EFI/BOOT/BOOTAA64.EFI || $sudo test -e /boot/efi/EFI/BOOT/BOOTAA64.EFI; } &&
    command -v limine >/dev/null; then
    echo limine
  elif [[ -f /boot/grub/grub.cfg ]] && command -v grub-mkconfig >/dev/null; then
    echo grub
  else
    echo unknown
  fi
}

# The MacBook Neo (T8140) boots an m1n1 built from aurora-silicon's J700
# branch. m1n1-aurora has no T8140 support, so on a Neo this script never
# installs it or the stock m1n1 over the one the Mac already boots.
is_neo() {
  tr '\0' '\n' <"$DT/compatible" 2>/dev/null | grep -qx 'apple,t8140'
}

# The chips this release installs on: M1 (t8103, t6000, t6001, t6002), M2
# (t8112, t6020, t6021, t6022), the MacBook Neo (t8140) and the M3s that is_m3
# lists. The kernel also carries device trees for the M3 Ultra (t6032) and
# M4 and later chips (t8132, t8142, t8152), but nobody has booted this kernel
# on them, and the M1/M2 path would replace their boot loader with an m1n1
# that can't start them. Keep this list in step with is_m3 and is_neo.
SUPPORTED_SOCS="t8103 t6000 t6001 t6002 t8112 t6020 t6021 t6022 t8140 t8122 t6030 t6031 t6034"

# This Mac's chip from the device tree, as tNNNN; empty when there is none.
this_soc() {
  tr '\0' '\n' <"$DT/compatible" 2>/dev/null | sed -n 's/^apple,\(t[0-9][0-9][0-9][0-9]\)$/\1/p' | head -1
}

# Stops before anything is downloaded or changed on a chip this release
# doesn't support. --agent-prompt and --reset-touchid don't come here.
require_supported_soc() {
  local soc
  soc=$(this_soc)
  [[ -n $soc && " $SUPPORTED_SOCS " == *" $soc "* ]] && return 0
  die "this Mac (${soc:+apple,$soc, }$(this_board)) is not one $VERSION supports: M1, M2, the MacBook
    Neo, and M3, M3 Pro and M3 Max. ${1:-Installing} would replace its boot loader with an m1n1 that
    can't start it. Nothing was changed. If you are bringing this Mac up, please open an issue at
    https://github.com/omacom/linux-aurora/issues"
}

# Every M3 chip: M3 (t8122), M3 Pro (t6030), M3 Max (t6031, t6034).
is_m3() {
  tr '\0' '\n' <"$DT/compatible" 2>/dev/null | grep -Eqx 'apple,(t8122|t6030|t6031|t6034)'
}

is_m3_pro() {
  tr '\0' '\n' <"$DT/compatible" 2>/dev/null | grep -qx 'apple,t6030'
}

# The M3 MacBook Airs. The other T8122 Macs (the 14" MacBook Pro M3, J504, and
# the iMacs, J433 and J434) have no handoff path and stay kernel-only.
M3_AIR_BOARDS="j613 j615"
is_m3_air() {
  local board
  tr '\0' '\n' <"$DT/compatible" 2>/dev/null | grep -qx 'apple,t8122' || return 1
  board=$(this_board)
  [[ -n $board && " $M3_AIR_BOARDS " == *" $board "* ]]
}

# The M3 Maxes with a boot loader variant (M3_MAX_BOARDS, t6031). The t6034 is not one of them.
is_m3_max() {
  local board
  tr '\0' '\n' <"$DT/compatible" 2>/dev/null | grep -qx 'apple,t6031' || return 1
  board=$(this_board)
  [[ -n $board && " $M3_MAX_BOARDS " == *" $board "* ]]
}

# M3 support is experimental. Every M3 gets linux-aurora; what happens to m1n1's
# stage 2 (boot.bin: m1n1, device trees and U-Boot) depends on the model, and
# m3_plan decides it before anything is downloaded (M3_MODE):
#   kernel   boot.bin stays exactly as it is. No m1n1-aurora, no /etc/m1n1.conf
#            change, no update-m1n1 run, and a freeze on update-m1n1 unless one
#            is in place already (the M3 bring-up's install-m3gpu.sh has one),
#            because pacman's hook would otherwise rebuild boot.bin when the
#            kernel's device trees arrive. --uninstall lifts it.
#   handoff  An M3 Pro model in M3_HANDOFF_BOARDS on the tested stub, or any M3
#            Pro whose owner asked for it with --m3-handoff. It gets
#            M1N1_PACKAGE, whose T6030 display and GPU handoff (from
#            iconidentify/m3-m1n1) stays off unless boot.bin ends with the
#            three M3_SWITCHES lines; update-m1n1 copies chosen.* lines from
#            /etc/m1n1.conf into every rebuild, so they survive updates.
#            An M3 MacBook Air takes the same path with the same m1n1 and its
#            own switches (m3_switches). A J613, which is in M3_HANDOFF_BOARDS,
#            gets the display handoff and the GPU firmware description
#            (M3_AIR_SWITCHES) by default; with --m3-handoff, and on the J615
#            only with it, an Air gets the display handoff with GPU
#            diagnostics (M3_AIR_DRY_RUN_SWITCHES). An owner can switch any
#            Air switch off for good with a chosen.<name>=0 line of their own
#            in /etc/m1n1.conf (m3_air_switch_off).
# M1 and M2 get the same M1N1_PACKAGE with no switches; the Neo keeps its own
# m1n1 unless NEO_AURORA_M1N1 is 1.
UPDATE_M1N1_CONF=/etc/default/update-m1n1
M3_FREEZE_BEGIN="# >>> aurora-sep: keep this M3's boot.bin as it is (remove with: install-aurora-sep.sh --uninstall)"
M3_FREEZE_END="# <<< aurora-sep: keep this M3's boot.bin as it is"
M3_FROZEN_BY=""
M3_BOOTBIN_SHA=""
# Models the handoff has been booted on, by us or by a tester's report. Only
# an M3 Pro (t6030) or an M3 MacBook Air (M3_AIR_BOARDS) listed here gets it
# by default. The J613 is listed: aurora8.3 and later published m1n1's display
# handoff on a J613 (11.111-test), with Linux on the boot framebuffer, and an
# Air listed here gets M3_AIR_SWITCHES by default. The J615 is not listed.
M3_HANDOFF_BOARDS="j516s j613"
M3_SWITCHES="chosen.asahi,t6030-gpu=1 chosen.asahi,t6030-dcp=1 chosen.asahi,t6030-dcpext=1"
# The M3 Pro handoff's name in $STATE/m3-mode. A plain run keeps the handoff on
# an M3 Pro that is not in M3_HANDOFF_BOARDS only while this matches, so it
# changes with each m1n1 that changes what a Pro boots: a J514S that opted in
# on an earlier release (t6030: aurora7 or 8.x) moves to this release's m1n1
# only with a new --m3-handoff. A listed Pro moves with the release.
# aurora12.1 boots an M3 Pro exactly as aurora12 does (its additions are for
# the T8122 only), so the name stays t6030-12: a Pro that opted in on 12.0
# or 12.1 moves to aurora12.1 on a plain run, as it would have on 12.0.
# aurora13 boots an M3 Pro exactly as aurora12.1 does (its additions act on a
# j613, or only with switches no variant sets), so the name stays t6030-12.
M3_PRO_VARIANT="t6030-12"
# The M3 MacBook Air's handoff switches, for a later build that starts the
# Air's GPU (M3_AIR_DRY_RUN=0): the GPU alone, or with M3_AIR_DCP=1 the
# display too, once m1n1's T8122 DCP handoff has been booted on an Air.
M3_AIR_GPU_SWITCH="chosen.asahi,t8122-gpu=1"
M3_AIR_DCP_SWITCH="chosen.asahi,t8122-dcp=1"
M3_AIR_DCP=0
# A dry run in place of the handoff (M3_AIR_DRY_RUN=1): m1n1 reads the Air's
# GPU and display details from the boot firmware and reports them on the
# serial console and under /chosen, powers the GPU only for a bounded
# identity read, and starts nothing. The desktop stays as it was.
# chosen.asahi,t8122-dcp=1 is a dry run only while m1n1 pins no T8122 DCP
# firmware image (aurora8 pins none); a release whose m1n1 pins one must give
# the dry run another variant name in m3_variant, so no Air gets it unasked.
# The display handoff (M3_AIR_DISPLAY_HANDOFF=1) is that case: the same
# switches with an m1n1 that pins the image (aurora8.3 and later), so m1n1
# hands the internal display over and publishes its checked state for Linux.
# The kernel keeps the boot framebuffer until it has a T8122 PMP description,
# and the GPU switches stay diagnostics: nothing starts the GPU.
# M3_AIR_DISPLAY_VARIANT names it in $STATE/m3-mode. A plain run keeps an
# Air's boot loader only while that name matches, so it changes with each m1n1
# that changes what an Air boots: an Air on an earlier test build's boot loader
# (air-display-handoff, aurora8.3 and 8.4) moves to this one only with a new
# --m3-handoff.
# aurora12.1 boots an Air with these switches exactly as aurora12 does: its
# GPU stand-in needs chosen.asahi,t8122-gpu=1 and
# chosen.asahi,t8122-gpu-power-standin=1, which no variant here sets. So the
# name stays air-display-handoff-12, and an Air that opted in on 12.0 or 12.1
# moves to aurora12.1 on a plain run with the same switches.
# aurora13 adds, on a j613, a read of the GPU's leakage fuses at every boot,
# logged and published for Linux; the GPU setup uses it only with
# chosen.asahi,t8122-gpu-fuse-leakage=1, which no variant sets. The names stay,
# so an Air on either variant moves to aurora13 on a plain run, with its own
# switches.
M3_AIR_DISPLAY_HANDOFF=1
M3_AIR_DISPLAY_VARIANT="air-display-handoff-12"
M3_AIR_DRY_RUN=1
M3_AIR_DRY_RUN_SWITCHES="chosen.asahi,t8122-gpu-diag=1 chosen.asahi,t8122-gpu-handoff-diag=1 chosen.asahi,t8122-gpu-power-diag=1 chosen.asahi,t8122-dcp=1"
# The default for an Air in M3_HANDOFF_BOARDS (m3_air_default): m1n1's display
# handoff (chosen.asahi,t8122-dcp=1) and the GPU firmware description
# (chosen.asahi,t8122-gpu-handoff-diag=1: the GPU's firmware and page-table
# memory reserved and described for Linux, the GPU and its mailbox left
# disabled). Both ran on a J613 in 11.110-test and 11.111-test. The GPU
# diagnostics (chosen.asahi,t8122-gpu-diag=1, chosen.asahi,t8122-gpu-power-diag=1,
# which powers the GPU for a short identity read) stay opt-in: --m3-handoff
# gives an Air M3_AIR_DRY_RUN_SWITCHES instead, and an Air that has those from
# an earlier --m3-handoff keeps them. No variant writes the GPU start,
# chosen.asahi,t8122-gpu=1.
M3_AIR_SWITCHES="chosen.asahi,t8122-dcp=1 chosen.asahi,t8122-gpu-handoff-diag=1"
# Its name in $STATE/m3-mode: this release's m1n1 with M3_AIR_SWITCHES.
M3_AIR_DEFAULT_VARIANT="air-handoff-12"
# The M3 Max (t6031: the 14" J514C and the 16" J516C) has one boot loader variant, opt-in only:
# --m3-handoff, or the M3 Max test kit (--m3max-kit), puts M1N1_PACKAGE on with
# M3_MAX_SAFE_SWITCHES. A plain run stays kernel-only (is_m3_kernel_only_chip), and the t6034
# (J514M, J516M) has no variant: it needs its own ADT first. The T6031 m1n1 (aurora17.t6031 and
# later) runs its handoffs on a J514C/J516C unless a switch says =0: chosen.asahi,t6031-dcp
# (the display handoff; the dcp-oslog reservations stay either way) and chosen.asahi,t6031-gpu
# (the GPU image facts, the powered identity read and the GPU node fill). The variant boots with
# the GPU part off: its identity read powers the GPU on every boot, and a read that stalled the
# bus would hang every boot of that boot.bin, which no one-shot entry can undo. Only the test kit
# turns it on (M3_MAX_GPU_HANDOFF, M3_MAX_SWITCHES), for its boot loader GPU stage, after the
# interim results are packed, and turns it off again. The display handoff, the log reservations,
# the PMP values and the firmware UUIDs stay on; the kernel's new T6031 paths are off unless a
# one-shot boot of the kit switches one on. m1n1_pkg_has_handoff refuses an m1n1 that lacks
# either name, so a release whose m1n1 names them differently stops before anything changes. A
# release whose m1n1 changes what a Max boots gives the variant a new name: a Max on another
# variant then moves to it only with a new --m3-handoff.
M3_MAX_BOARDS="j514c j516c"
M3_MAX_SWITCHES="chosen.asahi,t6031-dcp=1 chosen.asahi,t6031-gpu=1"
M3_MAX_SAFE_SWITCHES="chosen.asahi,t6031-dcp=1 chosen.asahi,t6031-gpu=0"
M3_MAX_VARIANT="t6031-gpu-off-1"
# 1 only while the test kit's boot loader GPU stage needs M3_MAX_SWITCHES (m3max_kit_loader).
M3_MAX_GPU_HANDOFF=0
# --m3-gpu-experiment (M3 MacBook Air only) installs the GPU start experiment's tools with the
# kernel: three scripts in M3_GPU_BIN, from this release (M3_GPU_SCRIPTS, checked like the
# packages), and the opt-in file M3_GPU_OPTIN that lets mesa-m3's login hook use the Air's GPU.
# It arms nothing and changes no boot setting: air-gpu-oneshot.sh arms one boot at a time, only
# when its owner runs it. Their sources are tools/aurora-sep/air-gpu/ in this repository. The
# Mesa they run with is mesa-m3, which every M3 Air gets (see the M3's Mesa below).
M3_GPU_EXPERIMENT=0
M3_GPU_PERSISTENT=0
M3_GPU_AUTO=0
M3_GPU_EXPLICIT_PROFILE=0
ESP_ARCHIVE_HISTORY=0
M3_GPU_PROFILE=legacy
M3_STACK_ID=""
M3_PERSISTENT_BOARDS="j613"
# J615 uses the shared 25G83 ABI with its own board identity and stage1 list.
# Release capability and explicit owner intent are both required for activation.
M3_NATIVE25_BOARDS="j613"
M3_25_J615=0
M3_STAGE1_25_VERSIONS=""
M3_STAGE1_25_J615_VERSIONS=""
M3_MESA_NATIVE_MARKER=/opt/mesa-m3/25g83/share/mesa-m3/profile
# The boards whose 25G83 sessions the installed Mesa's hook admits (mesa-m3 26.1.4.m3.2-4 on);
# a J615 needs j615-experimental there, or its sessions stop at profile-mismatch.
M3_MESA_NATIVE25_BOARDS=/opt/mesa-m3/share/mesa-m3/native25-boards
M3_PROFILE_SELECTOR=j613-25g83-hal200
M3_BOOT_PROFILE_HELPER=/usr/local/libexec/aurora-m3-boot-profile
M3_GPU_PROFILE_FILE=/etc/mesa-m3/t8122-profile
M3_GPU_CHECK=/usr/local/bin/aurora-m3-gpu-check
M3_GRUB_DEFAULTS=/etc/default/grub
M3_LIMINE_DEFAULTS=/etc/default/limine
M3_LIMINE_VENDOR_CONF=/usr/share/limine-entry-tool.d
M3_PROFILE_HOOK=/etc/pacman.d/hooks/zzzz-aurora-m3-profile.hook
M3_PROFILE_UPDATE=/usr/local/libexec/aurora-m3-profile-update
M3_PERSISTENT_TRANSACTION_ACTIVE=0
M3_PERSISTENT_MAIN_ARMED=0
M3_GPU_BIN=/usr/local/bin
M3_GPU_SCRIPTS=(
  "air-gpu-oneshot.sh d0259869b8519439a4dbcdae08ef1b0e17fdcaf84ac746fca61febf6a325f8aa"
  "air-gpu-collect.sh face8fac811b59c31a8e98e6ca704a2cb33eceada8bc37986397cb393b050b30"
  "air-gpu-job.sh 9a0eb9aab52fd099488054e4ed4846c579b19fc9f78144245242b6975a40da1d"
)
# The Mesa prefix air-gpu-job.sh runs with: mesa-m3's (12.4 on; it replaces the separate
# mesa-m3-g15g of 12.2 and 12.3, which this script removes before mesa-m3's pacman -U).
M3_GPU_MESA_PREFIX="/opt/mesa-m3"
# The earlier experiment-only Mesa package, removed from an Air before mesa-m3 goes on: mesa-m3
# conflicts with it, and pacman --noconfirm does not remove a conflicting package by itself.
M3_GPU_OLD_MESA="mesa-m3-g15g"
# mesa-m3's opt-in for the T8122 GPU: its login hook uses an Air's GPU only when this file exists
# (otherwise the reason is "experimental"). Written, root's and 0644, only by
# --m3-gpu-experiment on an Air, recorded in $STATE/m3-gpu-experiment, removed by --uninstall.
M3_GPU_OPTIN=/etc/mesa-m3/t8122-gpu-experiment
# The M3's Mesa (the M3 Pro, t6030, and the M3 MacBook Air, M3_AIR_BOARDS; on by default;
# --no-m3-mesa leaves it out): a pacman package, "file sha256", that installs Mesa with the M3
# Pro (G15S) and M3 (G15G) GPU drivers into a prefix of its own, never over the system Mesa,
# plus the files that make a login session use that prefix: on an M3 Pro by default, on an Air
# only with M3_GPU_OPTIN. It is a release asset of its own, never one of the PACKAGES every Mac
# gets, and is installed in its own pacman transaction once the kernel install is done
# (m3_pro_mesa_install). The names keep "M3_PRO_MESA" and "m3_pro_mesa" from 12.3, when only
# the M3 Pro had it: the record, $STATE/m3-pro-mesa, keeps its name and schema on both.
M3_PRO_MESA_PACKAGE="mesa-m3-26.1.4.m3.1-6-aarch64.pkg.tar.zst 2ad015a06c9cdbee670be5e8af485893f0a6c673e887e152d62f1690cdeb33dd"
M3_PRO_MESA_PREFIX="/opt/mesa-m3"
# The package's name, as its .PKGINFO gives it; --uninstall removes it by this exact name.
M3_PRO_MESA_NAME="mesa-m3"
# What the package depends on (its own depends, derived from its ELF files), as "name>=version"
# or a bare "name": each must be satisfied already (pacman -T: by name or by what a package
# provides), or the package is left out, so its pacman -U never pulls in an upgrade of the C
# library or the compiler runtime alone, or a new package.
M3_PRO_MESA_NEEDS="expat glibc>=2.43 libdisplay-info libdrm libgcc>=3.0 libglvnd libstdc++>=11.1 libx11 libxcb libxext libxshmfence libxxf86vm python spirv-tools>=1:1.4.357.0 systemd systemd-libs util-linux vulkan-icd-loader wayland xcb-util-keysyms zlib zstd"
# The package's own switch-offs, read only to say so in the summary: this file, the user's
# ~/.config/mesa-m3/disable, or mesa_m3=off on the kernel command line.
M3_PRO_MESA_DISABLE=/etc/mesa-m3/disable
M3_PRO_NATIVE_PACKAGE=""
M3_PRO_VULKAN_PACKAGE=""
M3_PRO_PERF_MIGRATOR=""
M3_PRO_PERF_SELECTED=0

M3_PRO_PERF_PRE_NATIVE=none M3_PRO_PERF_PRE_VULKAN=none
m3_pro_perf_plan() {
  is_m3_pro && ((M3_PRO_MESA)) && [[ -n $M3_PRO_NATIVE_PACKAGE && -n $M3_PRO_VULKAN_PACKAGE ]] || return 0
  M3_PRO_PERF_PRE_NATIVE=$(pacman -Q mesa-m3-native-pro 2>/dev/null | cut -d' ' -f2) || M3_PRO_PERF_PRE_NATIVE=none
  M3_PRO_PERF_PRE_VULKAN=$(pacman -Q mesa-m3-pro-vulkan 2>/dev/null | cut -d' ' -f2) || M3_PRO_PERF_PRE_VULKAN=none
  if [[ -f $STATE/m3-pro-perf-state ]]; then
    M3_PRO_PERF_PRE_NATIVE=$(sed -n 's/^pre_native=//p' "$STATE/m3-pro-perf-state")
    M3_PRO_PERF_PRE_VULKAN=$(sed -n 's/^pre_vulkan=//p' "$STATE/m3-pro-perf-state")
  fi
}

m3_pro_perf_files() {
  is_m3_pro && ((M3_PRO_MESA)) && [[ -n $M3_PRO_NATIVE_PACKAGE && -n $M3_PRO_VULKAN_PACKAGE ]] || return 0
  printf '%s\n' "$M3_PRO_NATIVE_PACKAGE" "$M3_PRO_VULKAN_PACKAGE"
}

m3_pro_perf_verify() {
  local pin file digest name
  is_m3_pro && ((M3_PRO_MESA)) && [[ -n $M3_PRO_NATIVE_PACKAGE && -n $M3_PRO_VULKAN_PACKAGE ]] || return 0
  for name in mesa-m3-native-pro mesa-m3-pro-vulkan; do
    if [[ $name == mesa-m3-native-pro ]]; then pin=$M3_PRO_NATIVE_PACKAGE; else pin=$M3_PRO_VULKAN_PACKAGE; fi
    read -r file digest <<<"$pin"
    [[ $(bsdtar -xOf "$work/$file" .PKGINFO | sed -n 's/^pkgname = //p') == "$name" ]] || die "paired Pro Mesa package identity differs"
  done
  [[ -n $M3_PRO_PERF_MIGRATOR ]] || die "paired Pro session helper is missing"
  M3_PRO_PERF_SELECTED=1
}

m3_pro_perf_activate() {
  ((M3_PRO_PERF_SELECTED)) || return 0
  local user
  [[ $(cat /opt/mesa-m3/native-pro/share/mesa-m3/profile) == t6030-native-gl-legacy-hal &&
     $(cat /opt/mesa-m3/pro-vulkan/share/mesa-m3/profile) == t6030-pro-vulkan-legacy-hal ]] || die "paired Pro Mesa installation is incomplete"
  user=$(m3_pro_mesa_user)
  if [[ $user != root ]]; then
    printf '%s' "$M3_PRO_PERF_MIGRATOR" | base64 -d > "$work/migrate-pro-session.py"
    $sudo runuser -u "$user" -- python3 - "$user" < "$work/migrate-pro-session.py" || die "could not update the managed native Pro session"
  fi
  $sudo systemctl enable mesa-m3-pro-performance.service
  printf 'pre_native=%s\npre_vulkan=%s\nnative=%s\nvulkan=%s\n' "$M3_PRO_PERF_PRE_NATIVE" "$M3_PRO_PERF_PRE_VULKAN"     "$(pacman -Q mesa-m3-native-pro | cut -d' ' -f2)" "$(pacman -Q mesa-m3-pro-vulkan | cut -d' ' -f2)" |     $sudo tee "$STATE/m3-pro-perf-state" >/dev/null
  say "Paired native Pro OpenGL and Honeykrisp Vulkan installed; the guarded performance profile activates at the next boot"
}
# mesa-m3's own user-setup detector and the list file it reads: what counts as a user's own Mesa
# setup, for the session hook and for this script alike. Once mesa-m3 is installed the record's
# user_setup comes from that detector; before that (or without it) from this script's byte copies
# of the same two files (m3_pro_mesa_builtin_detector and _list), whose sha256 are these.
# fill-m3-pro-mesa.sh fills all four from the package and refuses copies that differ from it.
M3_PRO_MESA_DETECTOR="/opt/mesa-m3/libexec/mesa-m3-user-setup"
M3_PRO_MESA_SETUP_LIST="/opt/mesa-m3/share/mesa-m3/user-setup.list"
M3_PRO_MESA_DETECTOR_SHA256=7e7914877c5f1ccc85b985b071c838271608c78f858489d69b7a3ba64c4f51f0
M3_PRO_MESA_SETUP_LIST_SHA256=8f19ebea2b8e669a764d7c4b40ecbc11e8234acfc3a2bfe7910e22de0b64a49d
# The session the record's user_setup is for (the detector's --session-desktops): stock
# Omarchy Hyprland under uwsm, the session this release supports. A setting only another
# desktop reads (chonkstep's, a uwsm env-NAME file for another NAME) is listed as ignored.
M3_PRO_MESA_SESSION_DESKTOPS=Hyprland
# Where the user-setup and switch-off checks find the Mac's files (the detector's --root); tests
# point it elsewhere. Paths in the record and the summary are the Mac's, without it.
M3_PRO_MESA_SETUP_ROOT=""
# The package's own files that make a login session use the prefix, keep the GPU's render node
# from the greeter, and confirm a session came up, for the record (fill-m3-pro-mesa.sh checks
# that the package has each of them). The package owns them: pacman -R mesa-m3 removes them.
M3_PRO_MESA_INTEGRATION="/usr/share/uwsm/env.d/50-mesa-m3
/usr/lib/udev/rules.d/72-mesa-m3-render-node.rules
/usr/lib/systemd/user/mesa-m3-session-confirm.service
/usr/lib/systemd/user/graphical-session.target.wants/mesa-m3-session-confirm.service"
# The group the package's udev rule gives the M3 Pro's render node to (mode 0660). This script adds
# the desktop user to it, and --uninstall takes them out only if this script added them.
M3_PRO_MESA_RENDER_GROUP=render
# 0 with --no-m3-mesa.
M3_PRO_MESA=1
# What happened on this run (m3_pro_mesa_plan/_install): "" (not an M3 Pro, or no package in
# this release), installed, current, newer-kept, skipped-flag, skipped-deps or failed.
M3_PRO_MESA_RESULT=""
# The record --uninstall and the lab check read, on an M3 Pro only, and its first line
# (m3_pro_mesa_record says what is in it).
M3_PRO_MESA_RECORD_NAME="m3-pro-mesa"
M3_PRO_MESA_SCHEMA="aurora.m3-pro-mesa-state/1"
# What this run did about the render group (m3_pro_mesa_render).
M3_PRO_MESA_RENDER_ADDED=no M3_PRO_MESA_RENDER_NOTE=""
# How this run's record write went (m3_pro_mesa_record), and the result it holds.
M3_PRO_MESA_RECORD_WRITE=""
M3_PRO_MESA_RECORD_RESULT=""
# The handoff is tested with one macOS system-firmware stub only, 14.8.3 (GPU
# firmware 14.8.3, DCP 14.7), which the Omarchy installer gives every M3. m1n1
# reports the stub's iBoot as asahi,iboot2-version.
M3_STUB_VERSION=14.8.3
M3_STUB_IBOOT=iBoot-10151.140.19
# The M3 m1n1 (stage 2) must fit below the display processors' log buffers,
# which every M3 boot so far placed 0x120000 bytes above where stage 1 loaded
# m1n1. That has only been seen with these stage 1 versions, as m1n1 reports
# them in /chosen/asahi,m1n1-stage1-version; the handoff is refused with any
# other, and on a Mac that reports none.
# The check is for M3s only. That log buffer placement is the M3's; on M1 and
# M2 nothing has been seen at that offset, and they have booted aurora3, whose
# file part reached about 0x110000, from every stage 1 in the field since 11.x.
# M1N1_PACKAGE is linked to end its file part below 0x120000 (0xe0000 for the
# aurora8.5 line it comes from), so a stage 1 allowlist there would only
# refuse Macs that boot today.
M3_STAGE1_VERSIONS="v1.6.1-dirty"
# m1n1 (aurora8.5-2 and later) adds this node when one of those log buffers
# overlaps where stage 1 loaded it. It keeps the buffer reserved and boots on,
# but it was never checked in that layout.
M3_OSLOG_OVERLAP=chosen/asahi,m1n1-oslog-overlap
M1N1_CONF=/etc/m1n1.conf
# What update-m1n1 puts at the start of boot.bin.
M1N1_BIN=/usr/lib/asahi-boot/m1n1.bin
# After a rebuild with M1N1_PACKAGE, $STATE/m1n1-installed records the m1n1
# that boot.bin was checked to start with: "sha256 bytes package".
M1N1_CONF_BEGIN="# >>> aurora-sep: M3 Pro display and GPU handoff (remove with: install-aurora-sep.sh --uninstall)"
M1N1_CONF_END="# <<< aurora-sep: M3 Pro display and GPU handoff"
M1N1_CONF_AIR_BEGIN="# >>> aurora-sep: M3 Air handoff (remove with: install-aurora-sep.sh --uninstall)"
M1N1_CONF_AIR_END="# <<< aurora-sep: M3 Air handoff"
M1N1_CONF_MAX_BEGIN="# >>> aurora-sep: M3 Max boot loader variant (remove with: install-aurora-sep.sh --uninstall)"
M1N1_CONF_MAX_END="# <<< aurora-sep: M3 Max boot loader variant"
# The M3 bring-up's install-m3gpu.sh froze boot.bin with exactly these lines,
# and left this marker when it created the file.
M3GPU_FREEZE=(
  "# Added by m3-gpu-work/scripts/install-m3gpu.sh: keep the M3 GPU candidate boot.bin."
  "# Undo with rollback-m3gpu.sh (then run: sudo update-m1n1)."
  "M1N1_UPDATE_DISABLED=1"
)
M3GPU_MARKER=/etc/default/.update-m1n1.created-by-m3gpu
# none (not an M3), kernel or handoff; set by m3_plan, kept in $STATE/m3-mode
# for --uninstall.
M3_MODE=none
# Set by --m3-handoff: the owner asks for the handoff on an M3 Pro or M3
# MacBook Air model that isn't in M3_HANDOFF_BOARDS yet.
M3_TRY=0
# Whether the owner gave --m3-handoff on this run (m3_plan copies M3_TRY before
# it keeps an earlier install's handoff). On an Air it asks for the GPU
# diagnostics too.
M3_ASKED=0
# 1 or 0 once m3_plan has decided whether this Air gets M3_AIR_SWITCHES
# (m3_air_default); empty before that.
M3_AIR_DEFAULT=""
# 1 when this run keeps the boot.bin this Mac has because an m1n1 from this
# script failed on it before ($STATE/m1n1-failed): set by m3_plan and
# m1n1_keep_plan.
M1N1_KEEP=0
# Where issue reports for a boot loader that failed go.
ISSUE_URL=https://github.com/iconidentify/aurora-linux/issues/6

# Whether update-m1n1 would exit without building, judged the way it judges:
# it sources this file and stops when M1N1_UPDATE_DISABLED is non-empty.
update_m1n1_frozen() {
  [[ -f $UPDATE_M1N1_CONF ]] || return 1
  # shellcheck disable=SC2016 # expanded by that sh, not here
  env -i PATH="$PATH" sh -c 'set -e; . "$1"; [ -n "${M1N1_UPDATE_DISABLED:-}" ]' _ \
    "$UPDATE_M1N1_CONF" >/dev/null 2>&1
}

m3_freeze() {
  local tmp
  if update_m1n1_frozen; then
    M3_FROZEN_BY=already
    return 0
  fi
  tmp=$(mktemp)
  if [[ -f $UPDATE_M1N1_CONF ]]; then cat "$UPDATE_M1N1_CONF" >"$tmp"; fi
  printf '%s\nM1N1_UPDATE_DISABLED=1\n%s\n' "$M3_FREEZE_BEGIN" "$M3_FREEZE_END" >>"$tmp"
  $sudo install -m 644 "$tmp" "$UPDATE_M1N1_CONF"
  rm -f "$tmp"
  update_m1n1_frozen || die "could not freeze update-m1n1 in $UPDATE_M1N1_CONF; nothing was installed"
  M3_FROZEN_BY=aurora-sep
}

# update-m1n1's configuration without this script's freeze, on stdout.
m3_unfrozen() {
  awk -v b="$M3_FREEZE_BEGIN" -v e="$M3_FREEZE_END" '
    $0 == b { skip = 1; next }
    skip && $0 == e { skip = 0; next }
    !skip' "$UPDATE_M1N1_CONF"
}

# Lift only this script's freeze; a file that held nothing else goes.
m3_unfreeze() {
  local tmp
  [[ -f $UPDATE_M1N1_CONF ]] && grep -qxF "$M3_FREEZE_BEGIN" "$UPDATE_M1N1_CONF" || return 0
  tmp=$(mktemp)
  m3_unfrozen >"$tmp"
  if grep -q '[^[:space:]]' "$tmp"; then
    $sudo install -m 644 "$tmp" "$UPDATE_M1N1_CONF"
  else
    $sudo rm -f "$UPDATE_M1N1_CONF"
  fi
  rm -f "$tmp"
}

# m1n1's boot.bin on the EFI partition, where macOS can reach it. Omarchy
# mounts that partition readable by root only, so look through sudo.
esp_bootbin() {
  local target
  for target in /boot/efi/m1n1/boot.bin /boot/m1n1/boot.bin; do
    $sudo test -f "$target" || continue
    [[ $(findmnt -no FSTYPE --target "${target%/m1n1/boot.bin}") == vfat ]] || continue
    echo "$target"
    return 0
  done
  return 1
}

m3_bootbin_sha() {
  local target
  target=$(esp_bootbin) || return 0
  $sudo sha256sum "$target" | cut -d' ' -f1
}

# After a kernel-only install: say plainly what happened to boot.bin, and prove it.
m3_bootbin_report() {
  local now
  now=$(m3_bootbin_sha)
  if [[ -n $M3_BOOTBIN_SHA && $now != "$M3_BOOTBIN_SHA" ]]; then
    die "m1n1's boot.bin changed during the install, which it must not on an M3.
    Please report it at https://github.com/omacom/linux-aurora/issues before rebooting."
  fi
  if [[ $M3_FROZEN_BY == aurora-sep ]]; then
    say "M3: m1n1's boot.bin is unchanged. pacman's \"Updating m1n1 image\" step did nothing:
    this script froze update-m1n1 in $UPDATE_M1N1_CONF, so kernel and m1n1
    updates keep boot.bin as it is. --uninstall lifts the freeze."
  else
    say "M3: m1n1's boot.bin is unchanged. pacman's \"Updating m1n1 image\" step did nothing:
    update-m1n1 was already frozen in $UPDATE_M1N1_CONF, and stays that way."
  fi
}

is_m3_handoff_board() {
  local board
  board=$(this_board)
  { is_m3_pro || is_m3_air; } && [[ -n $board && " $M3_HANDOFF_BOARDS " == *" $board "* ]]
}

# Whether this Air gets the default profile, M3_AIR_SWITCHES: a board in
# M3_HANDOFF_BOARDS, with the display handoff build (M3_AIR_DISPLAY_HANDOFF=1),
# whose owner did not ask for the GPU diagnostics, with --m3-handoff on this run
# or on an earlier install that is still recorded with that variant. m3_plan
# decides it (M3_AIR_DEFAULT) before the install records this run's variant.
m3_air_default() {
  if [[ -n $M3_AIR_DEFAULT ]]; then
    [[ $M3_AIR_DEFAULT == 1 ]]
    return
  fi
  [[ $M3_AIR_DISPLAY_HANDOFF == 1 ]] && is_m3_air && is_m3_handoff_board && ((M3_ASKED == 0)) || return 1
  [[ $(m3_recorded_mode) != handoff || $(m3_recorded_variant) != "$M3_AIR_DISPLAY_VARIANT" ]]
}

# The Air's per-Mac off switch: a line "chosen.<name>=0" of the owner's in
# /etc/m1n1.conf, outside this script's block, keeps switch $1 (chosen.<name>=1)
# out of the block. m1n1 then reads only the owner's 0, and arms a switch only
# when it reads 1. The line stays when this script rewrites its block.
m3_air_switch_off() {
  m1n1_conf_without_switches | grep -qxF "${1%%=*}=0"
}

# The Air switches this Mac's owner switched off, separated by spaces.
m3_air_switches_off() {
  local s off=()
  for s in $(m3_air_switch_set); do
    if m3_air_switch_off "$s"; then off+=("${s%%=*}"); fi
  done
  echo "${off[*]}"
}

# The Air's switches before its owner's off switches.
m3_air_switch_set() {
  if ((M3_GPU_PERSISTENT)); then
    if m3_25_j615; then
      echo "chosen.asahi,t8122-dcp=1 chosen.asahi,t8122-gpu=1 chosen.asahi,j615-25g83-experimental=1"
    elif [[ $M3_GPU_PROFILE == j613-25g83 ]]; then
      echo "chosen.asahi,t8122-dcp=1 chosen.asahi,t8122-gpu=1"
    else
      echo "chosen.asahi,t8122-dcp=1 chosen.asahi,t8122-gpu=1 chosen.asahi,t8122-gpu-power-standin=1 chosen.asahi,t8122-gpu-fuse-leakage=1"
    fi
    return 0
  fi
  if m3_air_default; then
    echo "$M3_AIR_SWITCHES"
  elif [[ $M3_AIR_DRY_RUN == 1 ]]; then
    echo "$M3_AIR_DRY_RUN_SWITCHES"
  elif [[ $M3_AIR_DCP == 1 ]]; then
    echo "$M3_AIR_GPU_SWITCH $M3_AIR_DCP_SWITCH"
  else
    echo "$M3_AIR_GPU_SWITCH"
  fi
}

# How to switch an Air switch off on this Mac, and which ones are off. With
# "quiet", only a list of the ones that are off, if there are any.
m3_air_off_notice() {
  local off
  off=$(m3_air_switches_off)
  if [[ -n $off ]]; then say "Switched off on this Mac in $M1N1_CONF: ${off// /, }"; fi
  [[ ${1:-} == quiet ]] && return 0
  say "To switch one of m1n1's M3 Air switches off on this Mac, add it with =0 on a line at the end
    of $M1N1_CONF (chosen.asahi,t8122-dcp=0 for the display handoff,
    chosen.asahi,t8122-gpu-handoff-diag=0 for the GPU firmware description), then run:
      sudo update-m1n1
    Later runs of this script keep it off. Delete the line to switch it back on."
}

# The switch lines this Mac's handoff needs, separated by spaces: the T6030
# ones, or the M3 Air's without those its owner switched off.
m3_switches() {
  local s on=()
  if is_m3_max; then
    if ((M3_MAX_GPU_HANDOFF)); then echo "$M3_MAX_SWITCHES"; else echo "$M3_MAX_SAFE_SWITCHES"; fi
    return 0
  fi
  if ! is_m3_air; then
    echo "$M3_SWITCHES"
    return 0
  fi
  for s in $(m3_air_switch_set); do
    if ! m3_air_switch_off "$s"; then on+=("$s"); fi
  done
  echo "${on[*]}"
}

# Whether this run puts M1N1_PACKAGE on this Mac: M1 and M2, an M3 on the
# handoff path, and a MacBook Neo only with NEO_AURORA_M1N1=1. Needs m3_plan
# first.
m1n1_for_this_mac() {
  if ((M1N1_KEEP)); then
    return 1
  elif is_neo; then
    [[ $NEO_AURORA_M1N1 == 1 ]]
  elif [[ $M3_MODE != none ]]; then
    [[ $M3_MODE == handoff ]]
  fi
}

# M1N1_PACKAGE's version, as pacman prints it: 1.6.1.aurora13-1.
m1n1_version() {
  local file=${M1N1_PACKAGE%% *}
  file=${file#m1n1-aurora-}
  file=${file#m1n1-neo-}
  echo "${file%-aarch64.pkg.tar.zst}"
}

# What the handoff does on this Mac, for messages.
m3_handoff_name() {
  if ((M3_GPU_PERSISTENT)) && m3_25_j615; then echo "experimental J615 (untested) $M3_GPU_PROFILE GPU/display handoff"; return; fi
  if ((M3_GPU_PERSISTENT)); then echo "experimental J613 $M3_GPU_PROFILE GPU/display handoff"; return; fi
  if is_m3_max; then echo "M3 Max boot loader variant ($M3_MAX_VARIANT)"; return; fi
  if ! is_m3_air; then
    echo "M3 Pro display and GPU handoff"
  elif m3_air_default; then
    echo "M3 Air display handoff"
  elif [[ $M3_AIR_DISPLAY_HANDOFF == 1 ]]; then
    echo "M3 Air display handoff with GPU diagnostics"
  elif [[ $M3_AIR_DRY_RUN == 1 ]]; then
    echo "M3 Air GPU and display dry run"
  elif [[ $M3_AIR_DCP == 1 ]]; then
    echo "M3 Air display and GPU handoff"
  else
    echo "M3 Air GPU handoff"
  fi
}

# Why this Mac's system-firmware stub isn't the one the handoff is tested
# with; nothing when it is. The installer's stub_info.json names the stub's
# macOS version, and m1n1 reports the stub's iBoot.
m3_stub_problem() {
  if [[ $M3_GPU_PROFILE == j613-25g83 ]]; then m3_25_boot_problem; return; fi
  local iboot bootbin info="" version=""
  iboot=$({ tr -d '\0' <"$DT/chosen/asahi,iboot2-version"; } 2>/dev/null) || iboot=""
  if bootbin=$(esp_bootbin); then
    info=${bootbin%/m1n1/boot.bin}/asahi/stub_info.json
    version=$($sudo cat "$info" 2>/dev/null | grep -o '"ProductVersion": *"[^"]*"' | head -1 |
      sed 's/.*"\([^"]*\)"$/\1/') || version=""
  fi
  if [[ -n $version && $version != "$M3_STUB_VERSION" ]]; then
    echo "its macOS system-firmware stub is $version ($info)"
  elif [[ $iboot != "$M3_STUB_IBOOT"* ]]; then
    echo "its system-firmware stub's iBoot is ${iboot:-not reported by m1n1}"
  fi
  return 0
}

# Why this Mac's m1n1 stage 1 isn't one the M3 m1n1 is checked with (see
# M3_STAGE1_VERSIONS); nothing when it is.
# Clean v1.6.1 has the same base/entry/BootArgs chainload ABI as aurora13.
# Both supported Air boards use the same T8122 chainload ABI and exact stub.
m3_clean_stage1_14_qualified() {
  local fw target version
  [[ $M3_GPU_PROFILE == legacy && ($(this_board) == j613 || $(this_board) == j615) && $(this_soc) == t8122 &&
      " $M3_PERSISTENT_BOARDS " == *" $(this_board) "* ]] || return 1
  [[ ! -e $DT/$M3_OSLOG_OVERLAP ]] || return 1
  fw=$({ tr -d '\0' <"$DT/chosen/asahi,os-fw-version"; } 2>/dev/null) || return 1
  [[ ($fw == 14.7 || $fw == 14.8.3) && -z $(m3_stub_problem) ]] || return 1
  target=$(esp_bootbin) || return 1
  version=$($sudo cat "${target%/m1n1/boot.bin}/asahi/stub_info.json" 2>/dev/null |
    grep -o '"ProductVersion": *"[^"]*"' | head -1 | sed 's/.*"\([^"]*\)"$/\1/') || return 1
  [[ $version == 14.8.3 ]]
}

m3_stage1_problem() {
  local stage1 allowed=$M3_STAGE1_VERSIONS
  [[ $M3_GPU_PROFILE != j613-25g83 ]] || allowed=$M3_STAGE1_25_VERSIONS
  # The J613-only 25 stage 1 refuses a J615; a J615 has its own list.
  ! m3_25_j615 || allowed=$M3_STAGE1_25_J615_VERSIONS
  stage1=$({ tr -d '\0' <"$DT/chosen/asahi,m1n1-stage1-version"; } 2>/dev/null) || stage1=""
  if [[ -z $stage1 ]]; then
    echo "its m1n1 reports no stage 1 version"
  elif [[ $stage1 == v1.6.1 ]] && m3_clean_stage1_14_qualified; then
    :
  elif [[ " $allowed " != *" $stage1 "* ]]; then
    echo "its m1n1 stage 1 is $stage1"
  fi
  return 0
}

# After a boot: warn when this boot's m1n1 found a display log buffer over
# the place stage 1 loaded it (M3_OSLOG_OVERLAP). Only an M3 m1n1 sets it.
m3_oslog_overlap_check() {
  [[ -e $DT/$M3_OSLOG_OVERLAP ]] || return 0
  warn "this boot's m1n1 reports that a display log buffer overlaps where its stage 1 loaded
    it ($DT/$M3_OSLOG_OVERLAP). This m1n1 was only checked with those buffers clear of it.
    Please report it at https://github.com/omacom/linux-aurora/issues with the file that
    --m3-report writes. The boot loader this Mac had before the handoff is kept on the EFI
    partition as m1n1/boot.bin.before-<version>."
}

# The sha256 of a downloaded m1n1-aurora package's m1n1.bin.
m1n1_pkg_sha() {
  local path=usr/lib/asahi-boot/m1n1.bin
  if ((NEO_GPU)); then path=usr/lib/m1n1-neo/m1n1.bin; fi
  { bsdtar -xOf "$1" "$path" | sha256sum | cut -d' ' -f1; } 2>/dev/null
}

# The sha256 of the first $2 bytes of boot.bin $1: the m1n1 at its start, when
# $2 is that m1n1's size.
bootbin_m1n1_sha() {
  $sudo head -c "$2" "$1" | sha256sum | cut -d' ' -f1
}

# True when update-m1n1's configuration names its own m1n1 or target: boot.bin
# then never starts with M1N1_PACKAGE's m1n1, which is not checked there.
update_m1n1_own_m1n1() {
  [[ -f $UPDATE_M1N1_CONF ]] || return 1
  # shellcheck disable=SC2016 # expanded by that sh, not here
  env -i PATH="$PATH" sh -c '. "$1" >/dev/null 2>&1; [ -n "${M1N1:-}${SOURCE:-}${TARGET:-}" ]' _ \
    "$UPDATE_M1N1_CONF" >/dev/null 2>&1
}

# The m1n1 builds that failed on this Mac: the sha256 that starts each line of
# $STATE/m1n1-failed. The restore steps (keep_bootbin_on_esp) have the owner
# add a line; m1n1_rollback_check adds one when it finds the boot.bin put back.
m1n1_failed_shas() {
  [[ -f $STATE/m1n1-failed ]] || return 0
  grep -oE '^[0-9a-f]{64}' "$STATE/m1n1-failed" || true
}

m1n1_failed_add() {
  grep -qxF "$1" <<<"$(m1n1_failed_shas)" && return 0
  $sudo install -d "$STATE"
  echo "$1 $2" | $sudo tee -a "$STATE/m1n1-failed" >/dev/null
}

# A Mac whose boot.bin was put back by hand after this script rebuilt it, as
# the restore steps say: boot.bin starts neither with the m1n1 recorded in
# $STATE/m1n1-installed nor with the m1n1 that is installed now. That m1n1 is
# recorded as failed on this Mac. A rebuild that only changed the device trees
# or the switches keeps the same m1n1, and is not taken for one.
m1n1_rollback_check() {
  local sha size pkg target
  [[ -f $STATE/m1n1-installed ]] || return 0
  read -r sha size pkg _ <"$STATE/m1n1-installed" || return 0
  [[ $sha =~ ^[0-9a-f]{64}$ && $size =~ ^[0-9]+$ ]] || return 0
  target=$(esp_bootbin) || return 0
  [[ $(bootbin_m1n1_sha "$target" "$size") == "$sha" ]] && return 0
  if [[ -f $M1N1_BIN ]] && $sudo cmp -s -n "$(stat -c %s "$M1N1_BIN")" "$M1N1_BIN" "$target"; then
    return 0
  fi
  m1n1_failed_add "$sha" "${pkg:-m1n1}: boot.bin was put back by hand, found $(date +%F)"
}

# On a Mac that is not an M3 (m3_plan decides for those): when this release's
# m1n1 failed on it before, keep the boot.bin it has (M1N1_KEEP). update-m1n1
# must stay frozen, or pacman's hook would put that m1n1 back.
m1n1_keep_plan() {
  if is_m3 || ! m1n1_for_this_mac; then return 0; fi
  grep -qxF "$M1N1_BIN_SHA" <<<"$(m1n1_failed_shas)" || return 0
  update_m1n1_frozen ||
    die "this release's m1n1 (sha256 $M1N1_BIN_SHA) failed on this Mac before (recorded in
    $STATE/m1n1-failed), so this script does not put it back, and update-m1n1 must not either.
    Keep updates from rebuilding boot.bin, then run this again:
      echo M1N1_UPDATE_DISABLED=1 | sudo tee -a $UPDATE_M1N1_CONF
    Nothing was installed. If you have not yet, please report what happened at $ISSUE_URL"
  M1N1_KEEP=1
  say "This release's m1n1 (sha256 $M1N1_BIN_SHA) failed on this Mac before (recorded in
    $STATE/m1n1-failed), so it is not put back: boot.bin stays as it is, and update-m1n1 stays
    frozen in $UPDATE_M1N1_CONF. If you have not yet, please report what happened at $ISSUE_URL"
}

# After an install that kept boot.bin (M1N1_KEEP) on a Mac that is not an M3.
m1n1_keep_report() {
  [[ $(m3_bootbin_sha) == "$M3_BOOTBIN_SHA" ]] ||
    die "m1n1's boot.bin changed during the install, which it must not here. Please report it
    at $ISSUE_URL before rebooting."
  say "m1n1's boot.bin is unchanged: update-m1n1 stays frozen in $UPDATE_M1N1_CONF."
}

# After update-m1n1 rebuilt boot.bin with M1N1_PACKAGE: check the m1n1 at its
# start by its bytes, and record it in $STATE/m1n1-installed.
m1n1_check_and_record() {
  local target size sha
  target=$(esp_bootbin) || die "could not find m1n1's boot.bin to check its m1n1 in"
  size=$(stat -c %s "$M1N1_BIN")
  sha=$(bootbin_m1n1_sha "$target" "$size")
  [[ $sha == "$M1N1_BIN_SHA" ]] ||
    die "the m1n1 at the start of $target is sha256 $sha, not this release's
    ($M1N1_BIN_SHA), so boot.bin was not rebuilt as it should be. The boot loader this Mac booted
    with is kept as m1n1/boot.bin.before-$VERSION on the EFI partition. Please report it at
    https://github.com/omacom/linux-aurora/issues before rebooting."
  printf '%s %s %s\n' "$sha" "$size" "${M1N1_PACKAGE%% *}" | $sudo tee "$STATE/m1n1-installed" >/dev/null
  say "m1n1's boot.bin starts with this release's m1n1 (sha256 $sha)"
}

# True when a downloaded m1n1-aurora package knows every switch this Mac's
# handoff needs (m3_switches): asahi,t6030-gpu for chosen.asahi,t6030-gpu=1.
m1n1_pkg_has_handoff() {
  local bin s rc=0
  bin=$(mktemp)
  # One string per line, in a file: grep -q on a pipe would stop reading early
  # and fail the 3.8 MB tr with SIGPIPE under pipefail.
  { bsdtar -xOf "$1" usr/lib/asahi-boot/m1n1.bin | tr '\0' '\n' >"$bin"; } 2>/dev/null || rc=1
  for s in $(m3_switches); do
    s=${s#chosen.}
    s=${s%%=*}
    ((rc == 0)) && ! grep -qaxF "$s" "$bin" && rc=1
  done
  if [[ $M3_GPU_PROFILE == j613-25g83 ]]; then
    for s in apple,j613-25g83-profile apple,j613-25g83-mapping-handoff apple,j613-25g83-gpu-handoff; do
      ((rc == 0)) && ! grep -qaxF "$s" "$bin" && rc=1
    done
    # A J615 hands over only behind its own experimental switch, which this m1n1 must know.
    if m3_25_j615; then
      ((rc == 0)) && ! grep -qaxF asahi,j615-25g83-experimental "$bin" && rc=1
    fi
  fi
  rm -f "$bin"
  return "$rc"
}

# update-m1n1's configuration without the M3 bring-up's freeze, on stdout.
m3gpu_unfrozen() {
  awk -v a="${M3GPU_FREEZE[0]}" -v b="${M3GPU_FREEZE[1]}" -v c="${M3GPU_FREEZE[2]}" '
    { line[NR] = $0 }
    END {
      for (i = 1; i <= NR; i++) {
        if (line[i] == a && line[i + 1] == b && line[i + 2] == c) { i += 2; continue }
        print line[i]
      }
    }' "$UPDATE_M1N1_CONF"
}

# True when something other than the M3 bring-up or this script stops update-m1n1.
update_m1n1_frozen_by_others() {
  local tmp rc=1
  [[ -f $UPDATE_M1N1_CONF ]] || return 1
  tmp=$(mktemp)
  m3gpu_unfrozen | awk -v b="$M3_FREEZE_BEGIN" -v e="$M3_FREEZE_END" '
    $0 == b { skip = 1; next }
    skip && $0 == e { skip = 0; next }
    !skip' >"$tmp"
  # shellcheck disable=SC2016 # expanded by that sh, not here
  env -i PATH="$PATH" sh -c 'set -e; . "$1"; [ -n "${M1N1_UPDATE_DISABLED:-}" ]' _ "$tmp" \
    >/dev/null 2>&1 && rc=0
  rm -f "$tmp"
  return "$rc"
}

# True when update-m1n1's configuration points it at another m1n1, U-Boot or
# config than the packaged ones: the handoff check could then pass on a boot.bin
# this release did not build.
update_m1n1_customised() {
  [[ -f $UPDATE_M1N1_CONF ]] || return 1
  # shellcheck disable=SC2016 # expanded by that sh, not here
  env -i PATH="$PATH" sh -c '. "$1" >/dev/null 2>&1; [ -n "${M1N1:-}${SOURCE:-}${U_BOOT:-}${CONFIG:-}${TARGET:-}" ]' _ \
    "$UPDATE_M1N1_CONF" >/dev/null 2>&1
}

# Lift the M3 bring-up's freeze: only its own three lines, and its marker.
# The original is kept so --uninstall can put the freeze back.
m3gpu_unfreeze() {
  local tmp
  [[ -f $UPDATE_M1N1_CONF ]] && grep -qxF "${M3GPU_FREEZE[0]}" "$UPDATE_M1N1_CONF" || return 0
  [[ -f $STATE/update-m1n1.m3gpu.saved ]] || $sudo cp -p "$UPDATE_M1N1_CONF" "$STATE/update-m1n1.m3gpu.saved"
  tmp=$(mktemp)
  m3gpu_unfrozen >"$tmp"
  if [[ -e $M3GPU_MARKER ]]; then
    [[ -f $STATE/m3gpu-marker.saved ]] || $sudo touch "$STATE/m3gpu-marker.saved"
    $sudo rm -f "$M3GPU_MARKER"
  fi
  if grep -q '[^[:space:]]' "$tmp"; then
    $sudo install -m 644 "$tmp" "$UPDATE_M1N1_CONF"
  else
    $sudo rm -f "$UPDATE_M1N1_CONF"
  fi
  rm -f "$tmp"
  say "Lifted the M3 bring-up's freeze on update-m1n1 (from install-m3gpu.sh)"
}

# /etc/m1n1.conf without this script's switch block (M3 Pro or M3 Air), on
# stdout.
m1n1_conf_without_switches() {
  [[ -f $M1N1_CONF ]] || return 0
  awk -v b="$M1N1_CONF_BEGIN" -v e="$M1N1_CONF_END" \
    -v ab="$M1N1_CONF_AIR_BEGIN" -v ae="$M1N1_CONF_AIR_END" \
    -v mb="$M1N1_CONF_MAX_BEGIN" -v me="$M1N1_CONF_MAX_END" '
    $0 == b || $0 == ab || $0 == mb { skip = 1; next }
    skip && ($0 == e || $0 == ae || $0 == me) { skip = 0; next }
    !skip' "$M1N1_CONF"
}

m3_switches_write() {
  local tmp s begin=$M1N1_CONF_BEGIN end=$M1N1_CONF_END
  if is_m3_air; then
    begin=$M1N1_CONF_AIR_BEGIN
    end=$M1N1_CONF_AIR_END
  elif is_m3_max; then
    begin=$M1N1_CONF_MAX_BEGIN
    end=$M1N1_CONF_MAX_END
  fi
  tmp=$(mktemp)
  m1n1_conf_without_switches >"$tmp"
  {
    echo "$begin"
    for s in $(m3_switches); do echo "$s"; done
    echo "$end"
  } >>"$tmp"
  $sudo install -m 644 "$tmp" "$M1N1_CONF"
  rm -f "$tmp"
}

m3_switches_remove() {
  local tmp
  [[ -f $M1N1_CONF ]] && grep -qxF -e "$M1N1_CONF_BEGIN" -e "$M1N1_CONF_AIR_BEGIN" -e "$M1N1_CONF_MAX_BEGIN" "$M1N1_CONF" ||
    return 0
  tmp=$(mktemp)
  m1n1_conf_without_switches >"$tmp"
  if grep -q '[^[:space:]]' "$tmp"; then
    $sudo install -m 644 "$tmp" "$M1N1_CONF"
  else
    $sudo rm -f "$M1N1_CONF"
  fi
  rm -f "$tmp"
}

# The M3 path an earlier install took, or nothing.
m3_recorded_mode() {
  local mode=""
  if [[ -f $STATE/m3-mode ]]; then read -r mode _ <"$STATE/m3-mode" || true; fi
  echo "$mode"
}

# What this release's handoff does on this Mac. It is recorded with the mode,
# so a later release never turns one kind of Air test into another (a dry run
# into a GPU start) without a new --m3-handoff.
m3_variant() {
  if ((M3_GPU_PERSISTENT)); then echo "air-gpu-persistent-$M3_GPU_PROFILE"; return; fi
  if is_m3_max; then echo "$M3_MAX_VARIANT"; return; fi
  if ! is_m3_air; then echo "$M3_PRO_VARIANT"
  elif m3_air_default; then echo "$M3_AIR_DEFAULT_VARIANT"
  elif [[ $M3_AIR_DISPLAY_HANDOFF == 1 ]]; then echo "$M3_AIR_DISPLAY_VARIANT"
  elif [[ $M3_AIR_DRY_RUN == 1 ]]; then echo air-dry-run
  elif [[ $M3_AIR_DCP == 1 ]]; then echo air-gpu-dcp
  else echo air-gpu
  fi
}

m3_recorded_variant() {
  local mode="" variant=""
  if [[ -f $STATE/m3-mode ]]; then read -r mode variant _ <"$STATE/m3-mode" || true; fi
  echo "$variant"
}

# A Mac that has the handoff from an earlier install, on which this release's
# m1n1 is not checked ($1, and how this Mac differs, $2): it keeps what it has.
m3_kept_refusal() {
  die "M3 ($(this_board)): this Mac has m1n1's $(m3_handoff_name) from an earlier install, and
    this release's m1n1 is only checked with $1, and this Mac differs: $2.
    Nothing was installed; this Mac keeps the boot loader and kernel it has. Please report it
    at $ISSUE_URL with the file that this writes:
      curl -fsSL $LATEST_URL | bash -s -- --m3-report"
}

# Decide the M3 path before anything is downloaded, so a Mac this release can't
# set up as asked stops with nothing changed.
m3_plan() {
  if ((M3_GPU_AUTO)); then m3_gpu_auto_profile; fi
  if [[ -f $STATE/m3-gpu-persistent ]] && ((M3_GPU_PERSISTENT == 0)); then
    local saved
    saved=$(cat "$STATE/m3-gpu-persistent")
    if [[ $saved == legacy || $saved == j613-25g83 ]]; then
      M3_GPU_PERSISTENT=1; M3_GPU_PROFILE=$saved; M3_TRY=1
      # Only --m3-profile=j615-25g83 records 25G83 on a J615; this release must still list it.
      if [[ $saved == j613-25g83 && $(this_board) == j615 ]]; then M3_25_J615=1; fi
    fi
  fi
  local board problem failed variant again="run this again" air=0 kept=0
  M3_MODE=none
  M3_ASKED=$M3_TRY M3_AIR_DEFAULT=""
  if m3_air_default; then M3_AIR_DEFAULT=1; else M3_AIR_DEFAULT=0; fi
  if ! is_m3; then
    ((M3_TRY == 0)) || die "--m3-handoff is for an M3 Pro or an M3 MacBook Air, and this Mac isn't an M3. Nothing was installed."
    return 0
  fi
  board=$(this_board)
  if is_m3_air; then air=1; fi
  m3_oslog_overlap_check
  # An m1n1 from this script failed on this Mac before: a plain run keeps the
  # boot loader it has now, whatever this release's m1n1 is, and --m3-handoff
  # never puts back the one that failed.
  failed=$(m1n1_failed_shas)
  if [[ -n $failed ]] && ((M3_TRY == 0)); then
    M3_MODE=kernel M1N1_KEEP=1
    say "M3 ($board): an m1n1 from this script failed on this Mac before (recorded in
    $STATE/m1n1-failed). Installing the kernel only: the boot loader this Mac has now stays as
    it is. If you have not yet, please report what happened at $ISSUE_URL"
    return 0
  fi
  if [[ -n $failed ]] && grep -qxF "$M1N1_BIN_SHA" <<<"$failed"; then
    die "--m3-handoff: this release's m1n1 (sha256 $M1N1_BIN_SHA) is the one that failed on this
    Mac (recorded in $STATE/m1n1-failed), so this script does not put it back.
    Nothing was installed. Keep the kernel-only install: run this again without --m3-handoff.
    If you have not yet, please report what happened at $ISSUE_URL"
  fi
  # An update never takes the handoff away again: once a Mac has it (listed, or
  # tried with --m3-handoff), a plain run keeps it, and its checks still apply.
  if ((M3_TRY == 0)) && [[ $(m3_recorded_mode) == handoff ]]; then
    # A freeze that is neither this script's nor the bring-up's: the line the
    # restore steps give after a boot loader failed. Keep that boot.bin.
    if update_m1n1_frozen_by_others; then
      M3_MODE=kernel M1N1_KEEP=1
      say "M3 ($board): this Mac has m1n1's handoff from an earlier install, and update-m1n1 is
    frozen in $UPDATE_M1N1_CONF, not by this script (the restore steps add that line).
    Installing the kernel only: the boot loader this Mac has now stays as it is. If a boot
    loader from this script failed on this Mac, please report it at $ISSUE_URL"
      return 0
    fi
    # An Air that gets the default profile (m3_air_default) moves with the
    # release, as a listed M3 Pro does.
    if ((air)) && ! m3_air_default && [[ $(m3_recorded_variant) != "$(m3_variant)" ]]; then
      die "M3 MacBook Air ($board): this Mac has an earlier test build's boot loader
    ($(m3_recorded_variant)), and this release's Air boot loader is a different one: the
    $(m3_handoff_name) ($(m3_variant)). Run this again with --m3-handoff to switch to it.
    Nothing was installed."
    fi
    # The same for an M3 Max: it opted in to an earlier variant, not to this one.
    variant=$(m3_recorded_variant)
    if is_m3_max && [[ $variant != "$(m3_variant)" ]]; then
      die "M3 Max ($board): this Mac has m1n1's boot loader variant from an earlier release
    (${variant:-no variant recorded}), and this release's is another one ($(m3_variant)). Nothing was
    installed; this Mac keeps the boot loader and kernel it has. To switch to the new variant (it
    replaces this Mac's boot loader, and the steps to put the old one back follow), run this again
    with --m3-handoff:
      curl -fsSL $LATEST_URL | bash -s -- --m3-handoff"
    fi
    # The same for an M3 Pro that is not on the list: it opted in to an
    # earlier m1n1, not to this one.
    if is_m3_pro && ! is_m3_handoff_board && [[ $variant != "$(m3_variant)" ]]; then
      die "M3 Pro ($board): this Mac has m1n1's display and GPU handoff from an earlier release
    (${variant:-no variant recorded}), and this release's m1n1 is a newer one ($(m3_variant)) that
    nobody has booted on this model yet. Nothing was installed; this Mac keeps the boot loader
    and kernel it has. To switch to the new m1n1 (it replaces this Mac's boot loader, and the
    steps to put the old one back follow), run this again with --m3-handoff:
      curl -fsSL $LATEST_URL | bash -s -- --m3-handoff"
    fi
    # Kept from the record, not asked for: say so only once every check
    # below has passed, and refuse in words that name no flag.
    M3_TRY=1 kept=1
  fi
  M3_MODE=kernel
  if ((air)); then
    # A plain run gives an Air only the default profile (m3_air_default: a board
    # in M3_HANDOFF_BOARDS, M3_AIR_SWITCHES); everything else needs --m3-handoff.
    if ! m3_air_default && ((M3_TRY == 0)); then
      say "M3 MacBook Air ($board): installing the kernel only, and boot.bin stays as it is. m1n1's
    $(m3_handoff_name) for the Air is being tested and is not on by default. To help test it
    (it replaces this Mac's boot loader), see case D in the M3 section of: bash -s -- --agent-prompt"
      return 0
    fi
  elif is_m3_max && ((M3_TRY)); then
    # The opt-in M3 Max variant: the same stub, stage 1 and freeze checks as the M3 Pro below.
    :
  elif ! is_m3_pro; then
    # 12.3's words, unchanged for every Mac that reaches them (the 16-core M3 Max no longer does).
    ((M3_TRY == 0)) || die "--m3-handoff is for an M3 Pro (t6030) or an M3 MacBook Air (j613, j615);
    this M3 ($board) has no display and GPU handoff in m1n1 yet. Nothing was installed."
    say "M3 ($board): installing the kernel only. m1n1 has no display and GPU handoff for this chip
    yet, so boot.bin stays as it is and the display runs on the boot framebuffer."
    return 0
  elif ! is_m3_handoff_board && ((M3_TRY == 0)); then
    say "M3 Pro ($board): installing the kernel only, and boot.bin stays as it is. The display and
    GPU handoff in m1n1 hasn't been booted on this model yet. To try it (it replaces this
    Mac's boot loader), see the M3 section of: bash -s -- --agent-prompt"
    return 0
  fi
  if ((M3_GPU_PERSISTENT)) && [[ $M3_GPU_PROFILE == j613-25g83 ]]; then
    m3_25_board_choice
    [[ -z $(m3_25_boot_problem) ]] || die "$(m3_25_boot_problem). Firmware migration is separate from a Linux package update."
  fi
  problem=$(m3_stub_problem)
  if [[ -n $problem ]]; then
    if ((kept)); then m3_kept_refusal "the macOS $M3_STUB_VERSION system-firmware stub" "$problem"; fi
    ((M3_TRY == 0)) || die "--m3-handoff: the handoff is only tested with the macOS $M3_STUB_VERSION
    system-firmware stub, and this Mac differs: $problem. Nothing was installed."
    warn "the $(m3_handoff_name) is only tested with the macOS $M3_STUB_VERSION
    system-firmware stub, and this Mac differs: $problem. Installing the kernel only;
    boot.bin stays as it is."
    return 0
  fi
  problem=$(m3_stage1_problem)
  if [[ -n $problem ]]; then
    if ((kept)); then m3_kept_refusal "m1n1 stage 1 $M3_STAGE1_VERSIONS" "$problem"; fi
    ((M3_TRY == 0)) || die "--m3-handoff: this release's M3 m1n1 is only checked with m1n1 stage 1
    $M3_STAGE1_VERSIONS, and this Mac differs: $problem. Nothing was installed."
    warn "the $(m3_handoff_name) is only checked with m1n1 stage 1 $M3_STAGE1_VERSIONS, and
    this Mac differs: $problem. Installing the kernel only; boot.bin stays as it is."
    return 0
  fi
  if ((M3_TRY)); then again+=" with --m3-handoff"; fi
  if update_m1n1_frozen_by_others; then
    die "$UPDATE_M1N1_CONF sets M1N1_UPDATE_DISABLED, and not from this script or the M3
    bring-up's install-m3gpu.sh. Switching the $(m3_handoff_name) on needs m1n1's boot.bin
    rebuilt with this release's m1n1. Remove that line and $again. Nothing was installed."
  fi
  if update_m1n1_customised; then
    die "$UPDATE_M1N1_CONF points update-m1n1 at its own m1n1, U-Boot, config or target
    (M1N1=, SOURCE=, U_BOOT=, CONFIG= or TARGET=). The $(m3_handoff_name) needs boot.bin
    built from this release's m1n1. Remove those lines and $again. Nothing was installed."
  fi
  M3_MODE=handoff
  if ((kept && air)); then
    say "M3 MacBook Air ($board): this Mac has m1n1's $(m3_handoff_name) from an earlier install; keeping it"
  elif ((kept)) && is_m3_max; then
    say "M3 Max ($board): this Mac has m1n1's $(m3_handoff_name) from an earlier install; keeping it"
  elif ((kept)); then
    say "M3 ($board): this Mac has m1n1's display and GPU handoff from an earlier install; keeping it"
  fi
  if ((M3_GPU_PERSISTENT)); then
    if m3_25_j615; then
      say "J615: installing matched experimental GPU profile $M3_GPU_PROFILE; current OS firmware is retained"
      m3_25_j615_warning
    else
      say "J613: installing matched experimental GPU profile $M3_GPU_PROFILE; current OS firmware is retained"
    fi
  elif is_m3_max; then
    if ((!kept)); then
      warn "--m3-handoff: installing m1n1's $(m3_handoff_name) on this M3 Max ($board). It reserves
    the display processors' log buffers, publishes the M3 Max facts and hands the internal display
    over for the kernel to take or leave; its GPU handoff stays off (chosen.asahi,t6031-gpu=0). The
    kernel's new M3 Max paths stay off, so the desktop stays on the boot framebuffer.
    This replaces the Mac's boot loader; the steps to put the old one back from macOS follow."
    fi
  elif ((air)); then
    if m3_air_default; then
      say "M3 MacBook Air ($board, macOS $M3_STUB_VERSION stub): installing m1n1 with the $(m3_handoff_name)"
    elif ((kept)); then
      # Its owner asked with --m3-handoff when it was installed.
      :
    elif [[ $M3_AIR_DISPLAY_HANDOFF == 1 ]]; then
      warn "--m3-handoff: trying m1n1's display handoff on this M3 MacBook Air ($board).
    It publishes the checked display state for Linux and collects GPU diagnostics;
    it does not start the GPU. Native display still needs T8122 PMP support, so the
    desktop stays on the boot framebuffer and renders in software.
    This replaces the Mac's boot loader; the steps to put the old one back from macOS follow."
    elif [[ $M3_AIR_DRY_RUN == 1 ]]; then
      warn "--m3-handoff: installing m1n1's GPU and display dry run on this M3 MacBook Air ($board).
    It reads the Air's GPU and display details and reports them for the bring-up, powering the
    GPU only for a short identity read; it switches nothing on, so the desktop stays as it is,
    on the boot framebuffer and rendered in software.
    This replaces the Mac's boot loader; the steps to put the old one back from macOS follow."
    else
      warn "--m3-handoff: trying m1n1's GPU handoff on an M3 MacBook Air ($board). It is in testing
    and not on by default. This replaces the Mac's boot loader; the steps to put the old one
    back from macOS follow. The display stays on the boot framebuffer, and the desktop keeps
    rendering in software: this is for testing the GPU handoff, not a faster desktop."
    fi
  elif is_m3_handoff_board; then
    say "M3 Pro ($board, macOS $M3_STUB_VERSION stub): installing m1n1 with the display and GPU handoff"
  elif ((!kept)); then
    warn "--m3-handoff: trying m1n1's display and GPU handoff on an M3 Pro model ($board) nobody
    has booted it on yet. This replaces the Mac's boot loader; the steps to put the old one
    back from macOS follow."
  fi
}

# After update-m1n1, boot.bin must end with the switch lines. update-m1n1
# appends /etc/m1n1.conf's lines straight after the gzipped U-Boot, so the
# first one has no newline in front: look for the block, not whole lines.
m3_verify_bootbin() {
  local target tail block size kind="M3 Pro"
  if is_m3_air; then kind="M3 Air"; fi
  if is_m3_max; then kind="M3 Max"; fi
  target=$(esp_bootbin) || die "could not find m1n1's boot.bin to check the $kind switches in"
  size=$(stat -c %s "$M1N1_BIN")
  $sudo cmp -s -n "$size" "$M1N1_BIN" "$target" ||
    die "$target does not start with this release's m1n1 ($M1N1_BIN), so it was not rebuilt.
    The boot loader this Mac booted with is kept as m1n1/boot.bin.before-$VERSION on the EFI
    partition. Please report it before rebooting."
  block=$(m3_switches | tr ' ' '\n')
  tail=$($sudo tail -c 1024 "$target" | tr -d '\0')
  [[ $tail == *"$block"* ]] ||
    die "the rebuilt $target does not carry the $kind switch lines. The boot loader this Mac
    booted with is kept as m1n1/boot.bin.before-$VERSION on the EFI partition. Please report
    it before rebooting."
  say "m1n1's boot.bin is this release's m1n1 with the $kind handoff switches"
}

# --uninstall on a bring-up Mac: update-m1n1 has just rebuilt boot.bin from the
# stock m1n1, which has no handoff. Put back the boot.bin the bring-up froze, and
# the freeze.
m3_restore_bringup() {
  local target
  [[ -f $STATE/update-m1n1.m3gpu.saved ]] || return 0
  $sudo cp -p "$STATE/update-m1n1.m3gpu.saved" "$UPDATE_M1N1_CONF"
  if [[ -f $STATE/m3gpu-marker.saved ]]; then $sudo touch "$M3GPU_MARKER"; fi
  if [[ -f $STATE/boot.bin.saved ]] && target=$(esp_bootbin); then
    replace_on_esp "$STATE/boot.bin.saved" "$target"
    say "Restored the M3 bring-up's boot.bin and its freeze on update-m1n1"
  else
    warn "restored the M3 bring-up's freeze on update-m1n1, but not its boot.bin (no saved copy)"
  fi
}

# Boards whose Touch ID support nobody has booted yet. Once a board's device
# tree names the enclave, m1n1 needs two boot manifests from the platform for
# it, and a board without them stops in m1n1 before any boot entry. Every
# board checked so far has them; these have not been checked.
UNPROVEN_SEP_BOARDS="j314c j316c j413"

# --m3-gpu-experiment: stops before anything is downloaded on a Mac that is not an M3 Air, or
# when the Mesa entry is malformed.
m3_gpu_plan() {
  ((M3_GPU_EXPERIMENT || M3_GPU_PERSISTENT)) || return 0
  if ((M3_GPU_PERSISTENT)); then
    m3_persistent_preflight
    warn "Persistent M3 GPU activation is experimental. This run retains a known boot entry and installs the matched kernel, Mesa and unified bootloader together."
    return 0
  fi
  is_m3_air || die "--m3-gpu-experiment is for the M3 MacBook Air (j613, j615) only, and this Mac is
    $(this_board) ($(this_soc)). Nothing was installed."
  warn "--m3-gpu-experiment: installing the M3 Air GPU experiment's scripts with the kernel, and
    $M3_GPU_OPTIN, which lets mesa-m3's login hook use the GPU. They arm nothing: every boot
    stays as it is until air-gpu-oneshot.sh arms one."
}

# The experiment's files for the download loop, "file sha256" per line (none without the flag).
m3_gpu_files() {
  ((M3_GPU_EXPERIMENT)) || return 0
  printf '%s\n' "${M3_GPU_SCRIPTS[@]}"
}

# After the kernel's pacman -U: the scripts, what they need, the opt-in file, and
# $STATE/m3-gpu-experiment, which --uninstall reads.
m3_gpu_install() {
  local entry file record=""
  ((M3_GPU_EXPERIMENT)) || return 0
  for entry in "${M3_GPU_SCRIPTS[@]}"; do
    file=${entry%% *}
    $sudo install -D -m 0755 "$work/$file" "$M3_GPU_BIN/$file"
    record+="script $file ${entry#* }"$'\n'
  done
  # air-gpu-job.sh runs its job through Python and the Vulkan loader.
  if ((FROZEN_PACKAGES)); then
    pacman -T python vulkan-icd-loader >/dev/null ||
      die "the admitted frozen transaction did not satisfy the GPU job dependencies"
  else
    $sudo pacman -S --needed --noconfirm python vulkan-icd-loader ||
      warn "could not install python and vulkan-icd-loader; air-gpu-job.sh needs them"
  fi
  record+=$(m3_gpu_optin)
  if [[ -n $record && $record != *$'\n' ]]; then record+=$'\n'; fi
  printf '%s' "$record" | $sudo tee "$STATE/m3-gpu-experiment" >/dev/null
}

# --m3-gpu-experiment on an Air: writes M3_GPU_OPTIN (and its directory, 0755, when missing),
# root's and 0644, and prints its record lines ("optin PATH", and "optin-dir DIR" when this
# script made the directory, now or in an earlier run). An opt-in file this script did not
# write (the owner's) is left as it is and not recorded, so --uninstall leaves it too.
m3_gpu_optin() {
  local dir=${M3_GPU_OPTIN%/*} rec=$STATE/m3-gpu-experiment ours=0 made=0
  if [[ -f $rec ]] && grep -qxF "optin $M3_GPU_OPTIN" "$rec"; then ours=1; fi
  if [[ -f $rec ]] && grep -qxF "optin-dir $dir" "$rec" && [[ -d $dir ]]; then made=1; fi
  if [[ -e $M3_GPU_OPTIN || -L $M3_GPU_OPTIN ]] && ((!ours)); then
    if ((made)); then printf 'optin-dir %s\n' "$dir"; fi
    return 0
  fi
  if [[ ! -d $dir ]]; then
    if $sudo install -d -m 0755 "$dir"; then made=1; else warn "could not make $dir"; fi
  fi
  if printf '# Written by install-aurora-sep.sh --m3-gpu-experiment on %s: mesa-m3 uses this M3 GPU only\n# while this file exists. --uninstall removes it.\n' \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" | $sudo tee "$M3_GPU_OPTIN" >/dev/null && $sudo chmod 0644 "$M3_GPU_OPTIN"; then
    printf 'optin %s\n' "$M3_GPU_OPTIN"
  else
    warn "could not write $M3_GPU_OPTIN; until it exists, mesa-m3 leaves this Air's GPU off at login"
    if ((ours)); then printf 'optin %s\n' "$M3_GPU_OPTIN"; fi
  fi
  if ((made)); then printf 'optin-dir %s\n' "$dir"; fi
}

# The packages of NEEDS ("name>=version" words) that are missing or older than their minimum,
# one "name version (needs min or newer)" per line; nothing when all are new enough.
mesa_needs_too_old() {
  local need name min have order
  for need in $1; do
    # name>=version, or a bare name: installed at any version.
    name=${need%%>=*} min="" order=""
    if [[ $need == *">="* ]]; then min=${need#*>=}; fi
    have=$(pacman -Q "$name" 2>/dev/null | cut -d' ' -f2) || have=""
    if [[ -n $have && -z $min ]]; then continue; fi
    if [[ -n $have ]]; then order=$(vercmp "$have" "$min" 2>/dev/null) || order=""; fi
    # vercmp prints -1, 0 or 1.
    [[ $order =~ ^[0-9]+$ ]] || echo "$name ${have:-not installed}${min:+ (needs $min or newer)}"
  done
  return 0
}

# What the owner reads at the end of an install with the flag, or of a plain run that keeps
# the tools of an earlier one.
m3_gpu_notice() {
  if ((M3_GPU_PERSISTENT)); then
    say "Experimental GPU profile $M3_GPU_PROFILE is selected for subsequent boots with the matched kernel, Mesa and bootloader. The retained previous entry uses asahi.t8122_start=0 and mesa_m3=off."
    say "Reboot, log into your desktop, then run: aurora-m3-gpu-check"
    [[ $M3_GPU_PROFILE != j613-25g83 ]] || say "25G83 native OpenGL is experimental; Vulkan hardware support is unavailable."
    if m3_25_j615; then m3_25_j615_warning; fi
    return 0
  fi
  if ((M3_GPU_EXPERIMENT)); then
    say "The M3 Air GPU experiment's scripts are in $M3_GPU_BIN: air-gpu-oneshot.sh,
    air-gpu-collect.sh and air-gpu-job.sh. Nothing is armed; every boot stays normal until
      sudo air-gpu-oneshot.sh start
    arms the next boot only (sudo air-gpu-oneshot.sh --check first says whether it can)."
    if [[ -f $M3_GPU_OPTIN ]]; then
      echo "   mesa-m3's opt-in for this Air's GPU is in place: $M3_GPU_OPTIN"
    else
      echo "   mesa-m3's opt-in for this Air's GPU is NOT in place ($M3_GPU_OPTIN; see above)."
    fi
    if [[ -n $(m3_pro_mesa_installed) ]]; then
      echo "   The Mesa prefix for air-gpu-job.sh: $M3_GPU_MESA_PREFIX (mesa-m3)"
    else
      echo "   mesa-m3 is not installed (see above), so air-gpu-job.sh has nothing to run with yet"
      echo "   ($M3_GPU_MESA_PREFIX)."
    fi
    if [[ " $(m3_switches 2>/dev/null) " != *" $M3_AIR_GPU_SWITCH "* ]]; then
      echo "   This Mac's boot loader does not hand the GPU over ($M3_AIR_GPU_SWITCH is not armed), so an"
      echo "   armed boot ends ARMED-NOT-STARTED until the boot loader hands the GPU over (see"
      echo "   https://github.com/iconidentify/aurora-linux/issues/35)."
    fi
  elif [[ -f $STATE/m3-gpu-experiment ]]; then
    echo "   Keeping the M3 Air GPU experiment's scripts from an earlier install in $M3_GPU_BIN;"
    echo "   run this again with --m3-gpu-experiment to update them to this release's."
  fi
  return 0
}

# The installed one-shot script.
m3_gpu_oneshot() { $sudo "$M3_GPU_BIN/air-gpu-oneshot.sh" "$@"; }

# Clear any armed experiment boot and its boot entry. Called before anything replaces the kernel:
# an install (a new release, or a rerun) rebuilds the UKI the armed entry pins, and --uninstall
# replaces the kernel, so an entry left armed would fail Limine's hash check at the next boot.
# Returns 0 when the experiment was never installed, or nothing was armed; non-zero when the
# disarm failed (the caller stops before changing anything).
m3_gpu_disarm() {
  [[ -f $STATE/m3-gpu-experiment && -x $M3_GPU_BIN/air-gpu-oneshot.sh ]] || return 0
  say "Clearing any armed M3 Air GPU experiment boot before the kernel is replaced (arm it again after the reboot)"
  m3_gpu_oneshot --disarm
}

m3_gpu_remove() {
  local kind name leftover=0 optin_dir=""
  [[ -f $STATE/m3-gpu-experiment ]] || return 0
  # uninstall_all already cleared any armed boot, before the kernel was replaced.
  while read -r kind name _ || [[ -n ${kind:-} ]]; do
    case $kind in
      script) [[ $name =~ ^air-gpu-[a-z]+\.sh$ ]] && $sudo rm -f "$M3_GPU_BIN/$name" ;;
      mesa) [[ $name =~ ^[A-Za-z0-9._+-]+$ ]] && pacman -Q "$name" >/dev/null 2>&1 &&
        { $sudo pacman -Rns --noconfirm "$name" || warn "could not remove $name"; } ;;
      optin) [[ $name == "$M3_GPU_OPTIN" ]] && { $sudo rm -f "$name" || warn "could not remove $name"; } ;;
      optin-dir) [[ $name == "${M3_GPU_OPTIN%/*}" ]] && optin_dir=$name ;;
    esac
  done <"$STATE/m3-gpu-experiment"
  # Any Mesa package still installed under a name a later release renamed (the record is
  # overwritten by a plain rerun), plus the state dir's own leftovers.
  for name in $(pacman -Qq 2>/dev/null | grep -E '^mesa-m3-g15g' || true); do
    $sudo pacman -Rns --noconfirm "$name" 2>/dev/null && leftover=1 || true
  done
  ((leftover == 0)) || warn "removed a leftover G15G Mesa package"
  # The opt-in's directory, when this script made it, and only while it is empty (the owner's
  # own /etc/mesa-m3/disable, say, keeps it).
  if [[ -n $optin_dir && -d $optin_dir ]]; then $sudo rmdir "$optin_dir" 2>/dev/null || true; fi
  $sudo rm -rf /var/lib/air-gpu
  say "Removed the M3 Air GPU experiment's scripts"
}

# ---- the M3's Mesa (t6030 and the M3 Air, on by default) ---------------------------------------
# On an M3 Pro or an M3 MacBook Air, the release's mesa-m3 package is downloaded and checked
# with the other release assets, then installed in a pacman transaction of its own once the
# kernel install is done. Everything that makes a login session use its prefix belongs to the
# package itself, so pacman -R mesa-m3 undoes it all; this script writes no file of its own for
# it (on an Air, --m3-gpu-experiment's opt-in file is the experiment's: m3_gpu_optin). It writes
# one record, $STATE/m3-pro-mesa (the name from 12.3, kept so an M3 Pro's history carries over;
# its board line says which Mac), which --uninstall and the lab check read. Every other Mac (M1,
# M2, the M3 Max, the other T8122 Macs) never downloads, installs or records anything of it.

# The Macs that get it, and what the summary calls their Mesa and GPU.
m3_mesa_mac() { is_m3_pro || is_m3_air; }
m3_mesa_name() { if is_m3_air; then echo "M3 MacBook Air's Mesa"; else echo "M3 Pro's Mesa"; fi; }
m3_mesa_gpu() { if is_m3_air; then echo "M3 MacBook Air's GPU"; else echo "M3 Pro's GPU"; fi; }

m3_pro_mesa_file() { echo "${M3_PRO_MESA_PACKAGE%% *}"; }
# The package's version (pkgver-pkgrel) from its file name, <name>-<version>-<arch>.pkg.tar.zst.
m3_pro_mesa_version() {
  local f
  f=$(m3_pro_mesa_file)
  f=${f#"$M3_PRO_MESA_NAME"-}
  echo "${f%-*.pkg.tar.zst}"
}
# The installed mesa-m3's version, or nothing.
m3_pro_mesa_installed() { pacman -Q "$M3_PRO_MESA_NAME" 2>/dev/null | cut -d' ' -f2 || true; }

# The user whose session the package serves: the one who ran this (through sudo, or not).
m3_pro_mesa_user() { echo "${SUDO_USER:-$(id -un)}"; }
m3_pro_mesa_user_home() { getent passwd "$1" 2>/dev/null | cut -d: -f6; }

# mesa-m3-user-setup's answer (schema 3) as NUL-terminated triples: "applied", a path and what was
# found there, once per place of a finding the session uses; "ignored", nothing and the finding
# ("SCOPE CERTAINTY DETAIL"), once per finding it does not use; and "session", nothing and the
# desktops the answer is for. From DETECTOR and its arguments. Exit status 0: none, 1: present,
# 3: unknown (an unreadable or unresolvable setting: never "none"), all from the findings that
# apply; anything else, or an answer that does not match its exit status, is a failure (2).
m3_pro_mesa_setup_run() {
  local out rc=0 line want="" session="" p d w paths=() details=() ignored=()
  out=$("$@" 2>/dev/null) || rc=$?
  case $rc in 0 | 1 | 3) ;; *) return 2 ;; esac
  [[ ${out%%$'\n'*} == "schema=aurora.mesa-m3-user-setup/3" ]] || return 2
  while IFS= read -r line; do
    case $line in
      session_desktops=*) session=${line#*=} ;;
      user_setup=*) want=${line#*=} ;;
      user_setup_path=*) paths+=("${line#*=}") ;;
      user_setup_detail=*) details+=("${line#*=}") ;;
      "user_setup_finding=ignored "*) ignored+=("${line#user_setup_finding=ignored }") ;;
    esac
  done <<<"$out"
  [[ -n $session ]] || return 2
  case $rc:$want in 0:none) ((${#paths[@]} == 0)) || return 2 ;; 1:present) ((${#paths[@]})) || return 2 ;; 3:unknown) ;; *) return 2 ;; esac
  printf 'session\0\0%s\0' "$session"
  for p in "${paths[@]}"; do
    w=""
    for d in "${details[@]}"; do
      if [[ $d == "$p"[:\ ]* ]]; then w=${d#"$p"}; break; fi
    done
    printf 'applied\0%s\0%s\0' "$p" "${w:- (no detail)}"
  done
  for w in "${ignored[@]}"; do printf 'ignored\0\0%s\0' "$w"; done
  return "$rc"
}

# This script's byte copies of mesa-m3's detector and list, run for HOME: the same answer as the
# installed detector's (m3_pro_mesa_setup_run). They are written to a private temporary
# directory, checked against their sha256 and run with python3; a mismatch or no python3 is a
# failure (2).
m3_pro_mesa_builtin_setup() {
  local dir rc=0
  [[ $M3_PRO_MESA_DETECTOR_SHA256 =~ ^[0-9a-f]{64}$ ]] && command -v python3 >/dev/null || return 2
  dir=$(mktemp -d) || return 2
  m3_pro_mesa_builtin_detector >"$dir/mesa-m3-user-setup"
  m3_pro_mesa_builtin_list >"$dir/user-setup.list"
  if [[ $(sha256sum <"$dir/mesa-m3-user-setup" | cut -d' ' -f1) == "$M3_PRO_MESA_DETECTOR_SHA256" &&
    $(sha256sum <"$dir/user-setup.list" | cut -d' ' -f1) == "$M3_PRO_MESA_SETUP_LIST_SHA256" ]]; then
    m3_pro_mesa_setup_run python3 "$dir/mesa-m3-user-setup" --home "$1" --list "$dir/user-setup.list" \
      --session-desktops "$M3_PRO_MESA_SESSION_DESKTOPS" ${M3_PRO_MESA_SETUP_ROOT:+--root "$M3_PRO_MESA_SETUP_ROOT"} || rc=$?
  else
    rc=2
  fi
  rm -rf "$dir"
  return "$rc"
}

# The invoking user's own Mesa setup for the session M3_PRO_MESA_SESSION_DESKTOPS names: the
# settings it uses as pairs (path, what) in M3_PRO_MESA_SETUP, the ones it does not use as
# "SCOPE CERTAINTY DETAIL" in M3_PRO_MESA_SETUP_IGNORED, the session as the detector names it in
# M3_PRO_MESA_SETUP_SESSION, its answer in M3_PRO_MESA_SETUP_ANSWER (present, none or unknown),
# and where it came from in M3_PRO_MESA_SETUP_SOURCE: package-detector (mesa-m3's installed
# detector) or installer-builtin (this script's byte copies: before mesa-m3 is installed, or when
# the installed one failed, with one warning). When neither answers: unknown, with a warning.
M3_PRO_MESA_SETUP=() M3_PRO_MESA_SETUP_IGNORED=() M3_PRO_MESA_SETUP_SOURCE="" M3_PRO_MESA_SETUP_ANSWER=""
M3_PRO_MESA_SETUP_SESSION=""
m3_pro_mesa_setup_collect() {
  local home rc=0 out=()
  home=$(m3_pro_mesa_user_home "$(m3_pro_mesa_user)")
  M3_PRO_MESA_SETUP=() M3_PRO_MESA_SETUP_IGNORED=() M3_PRO_MESA_SETUP_SOURCE=installer-builtin
  M3_PRO_MESA_SETUP_ANSWER=unknown M3_PRO_MESA_SETUP_SESSION=unknown
  if [[ -z $home || $home != /* ]]; then
    warn "no home directory for $(m3_pro_mesa_user); the record says user_setup=unknown"
    return 0
  fi
  if [[ -n $(m3_pro_mesa_installed) && -x $M3_PRO_MESA_DETECTOR ]]; then
    mapfile -d '' -t out < <(m3_pro_mesa_setup_run "$M3_PRO_MESA_DETECTOR" --home "$home" \
      --session-desktops "$M3_PRO_MESA_SESSION_DESKTOPS" ${M3_PRO_MESA_SETUP_ROOT:+--root "$M3_PRO_MESA_SETUP_ROOT"})
    wait $! || rc=$?
    if ((rc != 2 && ${#out[@]} % 3 == 0 && ${#out[@]})); then
      M3_PRO_MESA_SETUP_SOURCE=package-detector
      m3_pro_mesa_setup_take "$rc" "${out[@]}"
      return 0
    fi
    warn "$M3_PRO_MESA_DETECTOR failed; the record's user setup comes from this script's copy of it"
  fi
  rc=0
  mapfile -d '' -t out < <(m3_pro_mesa_builtin_setup "$home")
  wait $! || rc=$?
  if ((rc == 2 || ${#out[@]} % 3 != 0 || ${#out[@]} == 0)); then
    warn "this script's copy of mesa-m3's user-setup check failed (no python3, or not the release's bytes);
    the record says user_setup=unknown"
    return 0
  fi
  m3_pro_mesa_setup_take "$rc" "${out[@]}"
}

# Sorts the triples of m3_pro_mesa_setup_run (after its exit status) into M3_PRO_MESA_SETUP*.
m3_pro_mesa_setup_take() {
  local rc=$1
  shift
  case $rc in 0) M3_PRO_MESA_SETUP_ANSWER=none ;; 1) M3_PRO_MESA_SETUP_ANSWER=present ;; *) M3_PRO_MESA_SETUP_ANSWER=unknown ;; esac
  while (($# >= 3)); do
    case $1 in
      session) M3_PRO_MESA_SETUP_SESSION=$3 ;;
      applied) M3_PRO_MESA_SETUP+=("$2" "$3") ;;
      ignored) M3_PRO_MESA_SETUP_IGNORED+=("$3") ;;
    esac
    shift 3
  done
}

# Byte copies of mesa-m3's /opt/mesa-m3/libexec/mesa-m3-user-setup and its user-setup.list
# (MIT), from the package this release names. fill-m3-pro-mesa.sh --embed writes them.
m3_pro_mesa_builtin_detector() {
  cat <<'M3_PRO_MESA_USER_SETUP_PY'
#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""mesa-m3-user-setup: does this user (or the administrator) have an own Mesa setup?

    mesa-m3-user-setup --home HOME [--config-home DIR] [--env | --environ FILE]
                       [--session-desktops NAMES] [--list FILE] [--root DIR]

The one definition of "the user's own M3 Mesa setup", used by the mesa-m3
session hook at every login and by the installer. What counts is listed in
--list (default: share/mesa-m3/user-setup.list next to this program's prefix);
see the comments there.

  --home HOME        the user's home ("~" in the list)
  --config-home DIR  the user's XDG_CONFIG_HOME ("{config}"; default HOME/.config)
  --env              also check this process's own environment
  --environ FILE     also check a NUL-separated environment file (such as
                     /proc/PID/environ, which is that process's initial one)
  --session-desktops NAMES
                     the desktop names of the session this is for, ":"-separated
                     as in XDG_CURRENT_DESKTOP (uwsm exports them before it reads
                     any environment file); empty or absent: unknown
  --root DIR         prefix every path with DIR (tests)

Nothing is sourced or evaluated. Files are read as text and split into words
the way a POSIX shell would (quotes, backslashes, comments, ";"), with no
expansion except $HOME, ${HOME} and a leading ~ in LD_LIBRARY_PATH entries.

A setting counts as a user setup when it chooses a Mesa outside /opt/mesa-m3:
every component of a list value (":"-separated) must lie in /opt/mesa-m3,
except a reference to the variable's own previous value. A value that cannot
be resolved (another $variable, a command substitution) cannot be cleared and
makes the result "unknown", as does a candidate file or directory that exists
but cannot be read.

Only a setting the session uses counts. Each finding has a scope, the
desktops whose sessions read it (all of them must be among the session's
desktop names, compared with the names lowercased as uwsm does):
  all                 no condition: the environment checked, environment.d,
                      /etc/environment, uwsm's common env files, drirc
  desktop:NAME        a uwsm env-NAME file or env-NAME.d directory (uwsm reads
                      them only for a session with desktop name NAME)
  desktop:hyprland    Hyprland's own configuration ({config}/hypr)
  desktop:chonkstep   chonkstep's settings file, and CHONKSTEP_M3_MESA_PREFIX
                      wherever it is set (only chonkstep's launcher reads it)
Several conditions are joined with "+". With unknown session desktops only
scope "all" applies: a finding for one desktop is then reported but ignored.

Output, key=value lines:
  schema=aurora.mesa-m3-user-setup/3
  session_desktops=NAMES|unknown
  user_setup=present|unknown|none   (from the findings that apply)
  user_setup_path=P      once per place of an applied finding: an absolute
                         file or directory path, or environment:NAME
  user_setup_detail=T    once per applied finding: the place, the variable, why
  user_setup_finding=APPLIES SCOPE CERTAINTY T
                         once per finding, applied or not: APPLIES is applied
                         or ignored, CERTAINTY sure or unsure
"present" wins over "unknown": one sure finding is enough. No value holds a
carriage return or a newline: a path that would is refused (exit 2).
Exit status: 0 none, 1 present, 3 unknown, 2 error (usage, unreadable list,
refused path).
"""
import glob, os, re, stat, sys

PREFIX = '/opt/mesa-m3'
SCHEMA = 'aurora.mesa-m3-user-setup/3'
HERE = os.path.dirname(os.path.realpath(__file__))
DEFAULT_LIST = os.path.normpath(os.path.join(HERE, '..', 'share', 'mesa-m3', 'user-setup.list'))
NAME = re.compile(r'[A-Za-z_][A-Za-z0-9_]*$')
KINDS = ('variable', 'flag', 'prefix', 'ldpath', 'envfile', 'envfile-before', 'envfile-hookdir', 'drirc',
         'chonkstep')
HOOK = '50-mesa-m3'  # the session hook's own name in /usr/share/uwsm/env.d


def fail(msg):
    print(f'mesa-m3-user-setup: {msg}', file=sys.stderr)
    sys.exit(2)


def parse_args(argv):
    opts = {'home': None, 'config': None, 'env': False, 'environ': None, 'list': DEFAULT_LIST, 'root': '',
            'desktops': None}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == '--env':
            opts['env'] = True
            i += 1
            continue
        if a in ('--home', '--config-home', '--environ', '--list', '--root', '--session-desktops') and i + 1 < len(argv):
            key = {'--home': 'home', '--config-home': 'config', '--environ': 'environ',
                   '--list': 'list', '--root': 'root', '--session-desktops': 'desktops'}[a]
            opts[key] = argv[i + 1]
            i += 2
            continue
        fail(f'unknown or incomplete option {a!r}')
    if not opts['home'] or not opts['home'].startswith('/'):
        fail('--home must be an absolute path')
    if opts['env'] and opts['environ']:
        fail('--env and --environ exclude each other')
    opts['config'] = opts['config'] or os.path.join(opts['home'], '.config')
    names = [d.strip().lower() for d in (opts['desktops'] or '').split(':') if d.strip()]
    opts['desktops'] = names or None
    return opts


def read_list(path):
    items = []
    try:
        lines = open(path, encoding='utf-8').read().splitlines()
    except OSError as e:
        fail(f'cannot read the list {path}: {e.strerror}')
    for n, line in enumerate(lines, 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        parts = line.split('\t')
        if len(parts) != 2 or parts[0] not in KINDS:
            fail(f'{path}:{n}: not KIND<TAB>ITEM')
        items.append((parts[0], parts[1]))
    return items


def shell_words(line):
    """Split one line into words the way a POSIX shell does, without any
    expansion: quotes and backslashes are removed, a "#" that starts a word
    starts a comment, and ; & | ( ) < > separate words. None when a quote is
    left open."""
    words, word, started, i, n = [], [], False, 0, len(line)
    while i < n:
        c = line[i]
        if c in ' \t':
            if started:
                words.append(''.join(word))
                word, started = [], False
            i += 1
        elif c == '#' and not started:
            break
        elif c in ';&|()<>':
            if started:
                words.append(''.join(word))
                word, started = [], False
            words.append(c)
            i += 1
        elif c == '\\':
            started = True
            if i + 1 < n:
                word.append(line[i + 1])
            i += 2
        elif c == "'":
            j = line.find("'", i + 1)
            if j < 0:
                return None
            word.append(line[i + 1:j])
            started = True
            i = j + 1
        elif c == '"':
            started = True
            i += 1
            while True:
                if i >= n:
                    return None
                c = line[i]
                if c == '"':
                    i += 1
                    break
                if c == '\\' and i + 1 < n and line[i + 1] in '"\\$`':
                    word.append(line[i + 1])
                    i += 2
                    continue
                word.append(c)
                i += 1
        else:
            word.append(c)
            started = True
            i += 1
    if started:
        words.append(''.join(word))
    return words


def shell_assignments(line):
    """The NAME=VALUE words of one line of shell, environment.d or
    /etc/environment text. None when the line cannot be split."""
    words = shell_words(line)
    if words is None:
        return None
    out = []
    for w in words:
        m = re.match(r'([A-Za-z_][A-Za-z0-9_]*)=(.*)$', w, re.S)
        if m:
            out.append((m.group(1), m.group(2)))
    return out


def hyprlang_assignments(line):
    """`env = NAME,VALUE` of a Hyprland .conf line ("#" starts a comment,
    "##" is a literal "#")."""
    text = line.replace('##', '\0')
    if '#' in text:
        text = text[:text.index('#')]
    text = text.replace('\0', '#').strip()
    m = re.match(r'env\s*=\s*([A-Za-z_][A-Za-z0-9_]*)\s*,(.*)$', text)
    return [(m.group(1), m.group(2).strip())] if m else []


def lua_strip_comment(line):
    """The line without a trailing Lua comment ("--" outside a string)."""
    quote = None
    i = 0
    while i < len(line):
        c = line[i]
        if quote:
            if c == '\\':
                i += 2
                continue
            if c == quote:
                quote = None
        elif c in '"\'':
            quote = c
        elif line.startswith('--', i):
            return line[:i]
        i += 1
    return line


def lua_assignments(line):
    """env("NAME", "VALUE") calls of a Hyprland .lua line (hl.env and the
    like). A value that is not one string literal is returned as None."""
    text = lua_strip_comment(line)
    out = []
    for m in re.finditer(r'''env\s*\(\s*(["'])([A-Za-z_][A-Za-z0-9_]*)\1\s*,\s*''', text):
        rest = text[m.end():]
        lit = re.match(r'''(["'])((?:\\.|(?!\1).)*)\1\s*\)''', rest)
        out.append((m.group(2), lit.group(2) if lit else None))
    return out


UWSM_DESKTOP_FILE = re.compile(r'/uwsm/env-([^/]+?)(?:\.d)?(?:/|$)')


class Finder:
    def __init__(self, opts, items):
        self.o, self.items = opts, items
        self.paths, self.details, self.findings = [], [], []
        self.present = self.unknown = False
        self.cur_kind = None
        self.kind = {}
        for k, i in items:
            if k in ('variable', 'flag', 'prefix', 'ldpath'):
                self.kind[i] = k

    def real(self, p):
        return self.o['root'] + p

    def expand(self, item):
        if item.startswith('~/'):
            return os.path.join(self.o['home'], item[2:])
        if item.startswith('{config}/'):
            return os.path.join(self.o['config'], item[9:])
        return item

    def scope(self, place, name=None):
        """The desktops a finding at PLACE (and for variable NAME) needs."""
        need = set()
        if self.cur_kind == 'chonkstep' or (name and self.kind.get(name) == 'prefix'):
            need.add('chonkstep')
        if place.startswith('/'):
            hypr = self.expand('{config}/hypr')
            if place == hypr or place.startswith(hypr + '/'):
                need.add('hyprland')
            m = UWSM_DESKTOP_FILE.search(place)
            if m and self.cur_kind in ('envfile', 'envfile-before', 'envfile-hookdir'):
                need.add(m.group(1))
        return need

    def note(self, place, detail, sure=True, name=None):
        if '\n' in place or '\r' in place:
            fail(f'refusing a path with a line break: {place!r}')
        detail = detail.replace('\r', ' ').replace('\n', ' ')
        need = self.scope(place, name)
        have = self.o['desktops']
        applies = not need or (have is not None and need <= set(have))
        scope = '+'.join(f'desktop:{d}' for d in sorted(need)) or 'all'
        line = f"{'applied' if applies else 'ignored'} {scope} {'sure' if sure else 'unsure'} {detail}"
        if line not in self.findings:
            self.findings.append(line)
        if not applies:
            return
        if sure:
            self.present = True
        else:
            self.unknown = True
        if place not in self.paths:
            self.paths.append(place)
        if detail not in self.details:
            self.details.append(detail)

    def ours(self, path):
        p = os.path.normpath(path) if path.startswith('/') else path
        return p == PREFIX or p.startswith(PREFIX + '/')

    def holds_mesa(self, d):
        r = self.real(d)
        pats = ('libgallium*', 'libEGL_mesa*', 'libvulkan_asahi*', 'dri/*_dri.so')
        return any(glob.glob(os.path.join(glob.escape(r), p)) for p in pats)

    def expand_home(self, s):
        h = self.o['home']
        s = s.replace('${HOME}', h).replace('$HOME', h)
        return h + s[1:] if s == '~' or s.startswith('~/') else s

    def judge(self, place, name, value, label, from_file=False):
        """One setting of a listed variable, from the environment or a file."""
        note = lambda pl, detail, sure=True: self.note(pl, detail, sure, name=name)
        kind = self.kind.get(name)
        if value is None:
            note(place, f'{label} sets {name} to a value that is not a plain string', sure=False)
            return
        if kind == 'flag':
            if value != '':
                note(place, f'{label} sets {name}')
            return
        if kind == 'prefix':
            v = value.rstrip('/') or value
            if v == '':
                note(place, f'{label} sets {name} empty (the default prefix is used)')
            elif '$' in v or '`' in v:
                note(place, f'{label} sets {name} to an unresolved value', sure=False)
            elif not self.ours(v) or v.rstrip('/') != PREFIX:
                note(place, f'{label} sets {name} to {v}')
            return
        if kind == 'ldpath':
            if from_file and not re.search(r'\$\{?' + re.escape(name) + r'(?![A-Za-z0-9_])', value):
                # A later file that replaces the variable drops /opt/mesa-m3/lib
                # while the other Mesa variables still point into the prefix.
                note(place, f'{label} replaces {name} (without ${name})')
                return
            # Drop the references to the previous value (also ${NAME:+:$NAME}).
            rest = re.sub(r'\$\{' + re.escape(name) + r'(?::?[-+=?][^}]*)?\}|\$' + re.escape(name) + r'(?![A-Za-z0-9_])',
                          '', value)
            for comp in rest.split(':'):
                comp = self.expand_home(comp.strip())
                if comp == '':
                    continue
                if '$' in comp or '`' in comp:
                    note(place, f'{label} puts an unresolved entry in {name} ({comp})', sure=False)
                elif comp.startswith('/') and not self.ours(comp) and self.holds_mesa(os.path.normpath(comp)):
                    note(place, f'{label} puts another Mesa in {name} ({os.path.normpath(comp)})')
            return
        # A path variable: every component must lie in /opt/mesa-m3.
        comps = [c.strip() for c in value.split(':')]
        if all(c == '' for c in comps):
            note(place, f'{label} sets {name} empty')
            return
        for comp in comps:
            if comp in ('', '$' + name, '${' + name + '}'):
                continue
            if '$' in comp or '`' in comp:
                note(place, f'{label} sets {name} to an unresolved entry ({comp})', sure=False)
            elif not self.ours(comp):
                note(place, f'{label} sets {name} ({comp})')

    def check_env(self, env, label):
        ours_fallback = env.get('MESA_M3_FALLBACK') == 'software'
        for name, kind in self.kind.items():
            if name not in env:
                continue
            if ours_fallback and name == 'LIBGL_ALWAYS_SOFTWARE' and env[name] == '1':
                continue  # the session hook's own software fallback
            self.judge(f'environment:{name}', name, env[name], label)

    def readable(self, path):
        """Text of a candidate file; None (and a finding) if it can't be read."""
        try:
            with open(self.real(path), encoding='utf-8', errors='replace') as f:
                return f.read()
        except OSError as e:
            self.note(path, f'{path} cannot be read ({e.strerror})', sure=False)
            return None

    def glob(self, pattern):
        root = self.o['root']
        hits = (glob.glob(glob.escape(root) + pattern, recursive=True) if root
                else glob.glob(pattern, recursive=True))
        return sorted(h[len(root):] if root else h for h in hits)

    def candidates(self, pattern):
        """Existing regular files matching pattern. A directory or file on the
        way that exists but cannot be examined or listed is reported (unknown)
        instead of skipped; a missing one or a dangling link sets nothing."""
        base = pattern
        while any(ch in base for ch in '*?['):
            base = os.path.dirname(base)
        try:
            st = os.stat(self.real(base))
        except (FileNotFoundError, NotADirectoryError):
            return []
        except OSError as e:
            self.note(base, f'{base} cannot be examined ({e.strerror})', sure=False)
            return []
        if base == pattern:
            return [pattern] if stat.S_ISREG(st.st_mode) else []
        if not stat.S_ISDIR(st.st_mode):
            return []
        # Every directory a match could be in must be listable.
        parent = os.path.dirname(pattern)
        if '**' in pattern:
            root = self.o['root']
            def unlistable(err):
                d = err.filename[len(root):] if root else err.filename
                self.note(d, f'{d} cannot be listed', sure=False)
            dirs = [base]
            for d, _, _ in os.walk(self.real(base), onerror=unlistable):
                dirs.append(d[len(root):] if root else d)
        elif any(ch in parent for ch in '*?['):
            dirs = self.glob(parent)
        else:
            dirs = [parent]
        for d in dirs:
            rd = self.real(d)
            if os.path.isdir(rd) and not os.access(rd, os.R_OK | os.X_OK):
                self.note(d, f'{d} cannot be listed', sure=False)
        out = []
        for path in self.glob(pattern):
            if '\n' in path or '\r' in path:
                fail(f'refusing a path with a line break: {path!r}')
            try:
                st = os.stat(self.real(path))
            except FileNotFoundError:
                continue  # a dangling link sets nothing
            except OSError as e:
                self.note(path, f'{path} cannot be examined ({e.strerror})', sure=False)
                continue
            if stat.S_ISREG(st.st_mode):
                out.append(path)
        return out

    def check_envfile(self, path, text, after):
        names = set(self.kind)
        is_lua = path.endswith('.lua')
        is_hypr = path.endswith('.conf') and '/hypr/' in path
        for n, line in enumerate(text.splitlines(), 1):
            s = line.strip()
            if not s or s.startswith('#') or (is_lua and s.startswith('--')):
                continue
            if is_lua:
                found = lua_assignments(line)
            elif is_hypr:
                found = hyprlang_assignments(line)
            else:
                found = shell_assignments(line)
                if found is None:
                    if any(re.search(rf'(?<![A-Za-z0-9_]){re.escape(v)}(?![A-Za-z0-9_])', line) for v in names):
                        self.note(path, f'{path}:{n} cannot be split (unbalanced quotes)', sure=False)
                    continue
            for name, value in found:
                if name in names:
                    self.judge(path, name, value, f'{path}:{n}', from_file=after)

    def check_drirc(self, path, text):
        body = re.sub(r'<!--.*?-->', '', text, flags=re.S)
        if re.search(r'''name\s*=\s*["']dri_driver["']''', body):
            self.note(path, f'{path} chooses a driver (dri_driver)')

    def check_chonkstep(self, path, text, env_prefix):
        last = None
        for line in text.splitlines():
            found = shell_assignments(line)
            if found is None:
                if 'CHONKSTEP_M3_MESA_PREFIX' in line:
                    self.note(path, f'{path} cannot be split around CHONKSTEP_M3_MESA_PREFIX', sure=False)
                continue
            for name, value in found:
                if name == 'CHONKSTEP_M3_MESA_PREFIX':
                    last = value
        if last is not None:
            self.judge(path, 'CHONKSTEP_M3_MESA_PREFIX', last, path)
        elif not (env_prefix is not None and env_prefix.rstrip('/') == PREFIX):
            self.note(path, f"{path} sets no CHONKSTEP_M3_MESA_PREFIX: chonkstep's M3 launcher "
                            'uses its own default prefix')

    def run(self, env):
        if env is not None:
            self.check_env(env, 'the environment')
        env_prefix = env.get('CHONKSTEP_M3_MESA_PREFIX') if env else None
        for kind, item in self.items:
            if kind in ('variable', 'flag', 'prefix', 'ldpath'):
                continue
            self.cur_kind = kind
            for path in self.candidates(self.expand(item)):
                text = self.readable(path)
                if text is None:
                    continue
                if kind == 'envfile':
                    self.check_envfile(path, text, after=True)
                elif kind == 'envfile-before':
                    self.check_envfile(path, text, after=False)
                elif kind == 'envfile-hookdir':
                    if os.path.basename(path) != HOOK:
                        self.check_envfile(path, text, after=os.path.basename(path) > HOOK)
                elif kind == 'drirc':
                    self.check_drirc(path, text)
                elif kind == 'chonkstep':
                    self.check_chonkstep(path, text, env_prefix)
        return self


def main():
    o = parse_args(sys.argv[1:])
    items = read_list(o['list'])
    env = None
    if o['env']:
        env = dict(os.environ)
    elif o['environ']:
        try:
            raw = open(o['environ'], 'rb').read()
        except OSError as e:
            fail(f'cannot read {o["environ"]}: {e.strerror}')
        env = {}
        for entry in raw.split(b'\0'):
            k, sep, v = entry.decode('utf-8', 'replace').partition('=')
            if sep:
                env[k] = v
    f = Finder(o, items).run(env)
    state = 'present' if f.present else 'unknown' if f.unknown else 'none'
    lines = [f'schema={SCHEMA}', f"session_desktops={':'.join(o['desktops']) if o['desktops'] else 'unknown'}",
             f'user_setup={state}']
    lines += [f'user_setup_path={p}' for p in f.paths]
    lines += [f'user_setup_detail={d}' for d in f.details]
    lines += [f'user_setup_finding={x}' for x in f.findings]
    for line in lines:
        if '\n' in line or '\r' in line:
            fail('refusing a value with a line break')
    print('\n'.join(lines))
    sys.exit({'present': 1, 'unknown': 3, 'none': 0}[state])


try:
    main()
except SystemExit:
    raise
except BaseException as e:  # never let a crash read as "present"
    print(f'mesa-m3-user-setup: internal error: {type(e).__name__}: {e}'.replace('\n', ' '), file=sys.stderr)
    sys.exit(2)
M3_PRO_MESA_USER_SETUP_PY
}
m3_pro_mesa_builtin_list() {
  cat <<'M3_PRO_MESA_USER_SETUP_LIST'
# mesa-m3: what counts as a user's (or administrator's) own Mesa setup.
# Read by /opt/mesa-m3/libexec/mesa-m3-user-setup; the session hook and the
# installer both use that one detector. Format: KIND<TAB>ITEM, one per line.
#
# variable  a path variable that chooses a Mesa. A finding when it is set (in
#           the environment checked, or in one of the files below) and any
#           ":"-separated component lies outside /opt/mesa-m3, or when it is
#           set empty. A reference to its own previous value is allowed.
# flag      a variable that forces a driver choice: a finding when set to any
#           non-empty value.
# prefix    chonkstep's M3 Mesa prefix: a finding unless it is /opt/mesa-m3.
# ldpath    LD_LIBRARY_PATH: a finding for an entry, other than
#           /opt/mesa-m3/lib, that holds libgallium*, libEGL_mesa*,
#           libvulkan_asahi* or a dri/ directory with *_dri.so; and, in a
#           file, for an assignment that replaces it (does not keep
#           $LD_LIBRARY_PATH), since that drops /opt/mesa-m3/lib after the
#           session hook ran. Unrelated entries added to it are no finding.
# envfile   a file that sets session variables, searched for the variables
#           above. Shell, environment.d and /etc/environment lines are split
#           like a POSIX shell (quotes, comments, "export", ";"); Hyprland
#           .conf lines as "env = NAME,VALUE"; Hyprland .lua lines as
#           env("NAME", "VALUE"). "~" is the user's home; a "{config}" path
#           is under XDG_CONFIG_HOME (default ~/.config); "**" matches any
#           depth of subdirectories. envfile: read after the session hook
#           (uwsm env files and Hyprland's own); envfile-before: applied before
#           it (environment.d, /etc/environment, the system uwsm env); and
#           envfile-hookdir: the hook's own directory, where a file sorting
#           after 50-mesa-m3 is read after it. Replacing LD_LIBRARY_PATH counts
#           only after the hook.
# drirc     a drirc file: a finding when it contains a dri_driver option
#           outside XML comments.
# chonkstep chonkstep's M3 session settings: a finding when the file exists and
#           its last CHONKSTEP_M3_MESA_PREFIX names a prefix other than
#           /opt/mesa-m3, or when it sets none (chonkstep's M3 launcher then
#           uses its own default prefix) and the environment checked does not
#           set it to /opt/mesa-m3 either.
# A candidate that exists but cannot be read, or a value that cannot be
# resolved, makes the result "unknown" (never "none").
variable	VK_ICD_FILENAMES
variable	VK_DRIVER_FILES
variable	VK_ADD_DRIVER_FILES
variable	LIBGL_DRIVERS_PATH
variable	__EGL_VENDOR_LIBRARY_FILENAMES
variable	__EGL_VENDOR_LIBRARY_DIRS
variable	GBM_BACKENDS_PATH
variable	DRIRC_CONFIGDIR
flag	MESA_LOADER_DRIVER_OVERRIDE
flag	GALLIUM_DRIVER
flag	LIBGL_ALWAYS_SOFTWARE
flag	GBM_ALWAYS_SOFTWARE
prefix	CHONKSTEP_M3_MESA_PREFIX
ldpath	LD_LIBRARY_PATH
envfile-before	{config}/environment.d/*.conf
envfile	{config}/uwsm/env
envfile	{config}/uwsm/env-*
envfile	{config}/uwsm/env.d/*
envfile	{config}/uwsm/env-*.d/*
envfile-before	{config}/uwsm/default
envfile	{config}/hypr/**/*.conf
envfile	{config}/hypr/**/*.lua
envfile	/etc/xdg/uwsm/env
envfile	/etc/xdg/uwsm/env-*
envfile	/etc/xdg/uwsm/env.d/*
envfile	/etc/xdg/uwsm/env-*.d/*
envfile	/usr/local/share/uwsm/env
envfile	/usr/local/share/uwsm/env-*
envfile	/usr/local/share/uwsm/env.d/*
envfile	/usr/local/share/uwsm/env-*.d/*
envfile-before	/usr/share/uwsm/env
envfile	/usr/share/uwsm/env-*
envfile-hookdir	/usr/share/uwsm/env.d/*
envfile	/usr/share/uwsm/env-*.d/*
envfile-before	/etc/environment
drirc	~/.drirc
drirc	{config}/drirc
drirc	/etc/drirc
chonkstep	{config}/chonkstep/m3gpu-session.env
M3_PRO_MESA_USER_SETUP_LIST
}

# The package's switch-offs that are set, as NUL-terminated pairs like m3_pro_mesa_setup_run's
# (an empty path for mesa_m3=off on the kernel command line). Left as they are.
m3_pro_mesa_optout() {
  local home r=$M3_PRO_MESA_SETUP_ROOT
  home=$(m3_pro_mesa_user_home "$(m3_pro_mesa_user)")
  if [[ -e $r$M3_PRO_MESA_DISABLE ]]; then printf '%s\0%s\0' "$M3_PRO_MESA_DISABLE" "off for every user"; fi
  if [[ -n $home && -e $r$home/.config/mesa-m3/disable ]]; then
    printf '%s\0%s\0' "$home/.config/mesa-m3/disable" "off for $(m3_pro_mesa_user)"
  fi
  if [[ " $(cat /proc/cmdline 2>/dev/null) " == *" mesa_m3=off "* ]]; then
    printf '%s\0%s\0' "" "mesa_m3=off on the kernel command line"
  fi
  return 0
}

# "path: what; path: what" for the summary, from such pairs on stdin. A path with a newline or a
# carriage return in it is shown quoted ($'...').
m3_pro_mesa_describe() {
  local p w text=""
  while IFS= read -r -d '' p && IFS= read -r -d '' w; do
    if [[ $p == *[$'\n\r']* ]]; then p=$(printf '%q' "$p"); fi
    # The detector's detail goes on from its path (":3 sets X"); the switch-offs' is a phrase.
    if [[ -n $p && $w == [:\ ]* ]]; then text+="${text:+; }$p$w"; else text+="${text:+; }${p:+$p: }$w"; fi
  done
  echo "$text"
}

# The user the record's render keys are about: render_user in an error record, else user.
m3_pro_mesa_record_user() {
  local u
  u=$(m3_pro_mesa_record_get render_user)
  if [[ -z $u ]]; then u=$(m3_pro_mesa_record_get user); fi
  echo "$u"
}

# A scalar value of this Mac's record, read as data, never sourced. Nothing when there is no
# record, or its first line is not this script's schema.
m3_pro_mesa_record_ok() {
  local first="" rec=$STATE/$M3_PRO_MESA_RECORD_NAME
  [[ -f $rec ]] || return 1
  IFS= read -r first <"$rec" || true
  [[ $first == "schema=$M3_PRO_MESA_SCHEMA" ]]
}
# Every value of a repeated KEY of this Mac's record, one per line (nothing without a record).
m3_pro_mesa_record_list() {
  m3_pro_mesa_record_ok || return 0
  sed -n "s/^$1=//p" "$STATE/$M3_PRO_MESA_RECORD_NAME"
}
m3_pro_mesa_record_get() {
  m3_pro_mesa_record_ok || return 0
  sed -n "/^$1=/{s/^$1=//p;q}" "$STATE/$M3_PRO_MESA_RECORD_NAME"
}

# Before anything is downloaded: decides whether this run installs the package, and stops on a
# malformed entry. Says one line when the package is left out by --no-m3-mesa or is current.
M3_PRO_MESA_PREEXISTING=""
m3_pro_mesa_plan() {
  local have want order
  m3_mesa_mac && [[ -n $M3_PRO_MESA_PACKAGE ]] || return 0
  have=$(m3_pro_mesa_installed)
  # Before the first record (or with one this script can't read): what was installed before.
  if ! m3_pro_mesa_record_ok; then M3_PRO_MESA_PREEXISTING=${have:-none}; fi
  if ((!M3_PRO_MESA)); then
    M3_PRO_MESA_RESULT=skipped-flag
    say "--no-m3-mesa: leaving out the $(m3_mesa_name) ($M3_PRO_MESA_NAME)${have:+; the $have installed earlier stays as it is}"
    return 0
  fi
  [[ $M3_PRO_MESA_PACKAGE =~ ^${M3_PRO_MESA_NAME}-[A-Za-z0-9._+:]+-[0-9.]+-(aarch64|any)\.pkg\.tar\.zst\ [0-9a-f]{64}$ &&
    $M3_PRO_MESA_PACKAGE != *PENDING* ]] ||
    die "M3_PRO_MESA_PACKAGE is not \"file sha256\" for $M3_PRO_MESA_NAME (a packaging mistake). Nothing was installed."
  [[ $M3_PRO_MESA_NEEDS =~ ^([A-Za-z0-9._+-]+(\>=[A-Za-z0-9._+:-]+)?[[:space:]]*)+$ && $M3_PRO_MESA_NEEDS != *PENDING* ]] ||
    die "M3_PRO_MESA_NEEDS is not a list of name>=version or name (a packaging mistake). Nothing was installed."
  [[ $M3_PRO_MESA_DETECTOR =~ ^/[A-Za-z0-9._/+-]+$ && $M3_PRO_MESA_SETUP_LIST =~ ^/[A-Za-z0-9._/+-]+$ &&
    $M3_PRO_MESA_DETECTOR_SHA256 =~ ^[0-9a-f]{64}$ && $M3_PRO_MESA_SETUP_LIST_SHA256 =~ ^[0-9a-f]{64}$ ]] ||
    die "M3_PRO_MESA_DETECTOR and M3_PRO_MESA_SETUP_LIST are not mesa-m3's paths (a packaging mistake). Nothing was installed."
  [[ -n $have ]] || return 0
  want=$(m3_pro_mesa_version)
  order=$(vercmp "$have" "$want" 2>/dev/null) || order=""
  if [[ $order == 0 ]]; then
    M3_PRO_MESA_RESULT=current
    say "The $(m3_mesa_name) ($M3_PRO_MESA_NAME $have) is installed already; nothing new to install"
  elif [[ $order == 1 ]]; then
    M3_PRO_MESA_RESULT=newer-kept
    say "A newer $M3_PRO_MESA_NAME ($have) than this release's ($want) is installed; keeping it"
  fi
  return 0
}

# The package for the download loop, "file sha256" (only when this run installs it).
m3_pro_mesa_files() {
  m3_mesa_mac && [[ -n $M3_PRO_MESA_PACKAGE ]] && ((M3_PRO_MESA)) && [[ -z $M3_PRO_MESA_RESULT ]] || return 0
  echo "$M3_PRO_MESA_PACKAGE"
}

# After the download loop: keep the package out of the kernel's pacman -U ("$work"/*.pkg.tar.zst).
m3_pro_mesa_set_aside() {
  ((M3_GPU_PERSISTENT || FROZEN_PACKAGES || M3_PRO_PERF_SELECTED)) && return 0
  local file
  file=$(m3_pro_mesa_file)
  [[ -n $M3_PRO_MESA_PACKAGE && -f $work/$file ]] || return 0
  mkdir -p "$work/m3-pro"
  mv "$work/$file" "$work/m3-pro/"
}

# The M3_PRO_MESA_NEEDS entries this Mac does not satisfy, one "name version (needs min or
# newer)" per line; nothing when all are. pacman -T decides, as pacman -U would: by name or by
# what a package provides, so the separate libgcc and libstdc++ of current Arch Linux ARM count,
# and so does an older package that provides them. When pacman -T itself fails, one line says so.
m3_pro_mesa_needs_unmet() {
  local out rc=0 need name min have needs=()
  read -ra needs <<<"$M3_PRO_MESA_NEEDS"
  out=$(pacman -T "${needs[@]}" 2>/dev/null) || rc=$?
  if ((rc == 0)); then return 0; fi
  if ((rc != 127)); then
    echo "pacman -T could not check them (exit $rc)"
    return 0
  fi
  while IFS= read -r need; do
    [[ -n $need ]] || continue
    name=${need%%[<>=]*} min=""
    if [[ $need == *">="* ]]; then min=${need#*>=}; fi
    have=$(pacman -Q "$name" 2>/dev/null | cut -d' ' -f2) || have=""
    echo "$name ${have:-not installed}${min:+ (needs $min or newer)}"
  done <<<"$out"
  return 0
}

# Once the kernel install is done, on an M3 Pro: the package in a pacman transaction of its own,
# so a problem with it can't stop the kernel install halfway. It is left out when a package it
# needs is missing or too old, rather than letting it pull an upgrade in. Any failure is a
# warning, the kernel install stays as it is, and the summary and exit status say so.
m3_pro_mesa_install() {
  local file old name
  if ((M3_GPU_PERSISTENT || FROZEN_PACKAGES)) && [[ $M3_PRO_MESA_RESULT == installed ]]; then return 0; fi
  file=$work/m3-pro/$(m3_pro_mesa_file)
  [[ -n $M3_PRO_MESA_PACKAGE && -f $file ]] || return 0
  old=$(m3_pro_mesa_needs_unmet)
  if [[ -n $old ]]; then
    M3_PRO_MESA_RESULT=skipped-deps
    warn "left out the $(m3_mesa_name) ($M3_PRO_MESA_NAME), because it needs newer packages than this
    Mac has: $(paste -sd ';' <<<"$old" | sed 's/;/; /g'). Nothing of it was installed, and the
    kernel install is complete. Update them (sudo pacman -Syu $(cut -d' ' -f1 <<<"$old" | grep -v '^pacman$' |
      paste -sd' ')), then run this again."
    return 0
  fi
  name=$(bsdtar -xOf "$file" .PKGINFO 2>/dev/null | sed -n 's/^pkgname = //p' | head -1) || name=""
  if [[ $name != "$M3_PRO_MESA_NAME" ]]; then
    M3_PRO_MESA_RESULT=failed
    warn "left out the $(m3_mesa_name): ${file##*/} names the package \"$name\", not $M3_PRO_MESA_NAME.
    The kernel install is complete. Please report it at https://github.com/omacom/linux-aurora/issues"
    return 0
  fi
  if is_m3_air && ! m3_pro_mesa_replace_old; then
    M3_PRO_MESA_RESULT=failed
    return 0
  fi
  say "Installing the $(m3_mesa_name) ($name $(m3_pro_mesa_version)) on its own, now that the kernel is in"
  if ! $sudo pacman -U --noconfirm "$file"; then
    M3_PRO_MESA_RESULT=failed
    warn "could not install the $(m3_mesa_name) ($name); pacman says why above. Nothing of it was
    installed by this run, and the kernel install is complete and stays as it is. Run this again
    to try again."
    return 0
  fi
  M3_PRO_MESA_RESULT=installed
}

# On an Air, before mesa-m3's pacman -U: removes the experiment's earlier Mesa package
# (M3_GPU_OLD_MESA, the 12.2 and 12.3 --m3-gpu-experiment's), which mesa-m3 replaces and
# conflicts with, and adds "name version" to M3_PRO_MESA_REPLACED for the record. Returns 1, with
# a warning, when it stays: then mesa-m3 is left out (pacman would refuse it).
M3_PRO_MESA_REPLACED=()
m3_pro_mesa_replace_old() {
  local name ver
  while read -r name ver; do
    [[ $name == "$M3_GPU_OLD_MESA" || $name == "$M3_GPU_OLD_MESA"-* ]] || continue
    say "Removing $name $ver: mesa-m3 replaces it"
    if ! $sudo pacman -Rns --noconfirm "$name"; then
      warn "could not remove $name, which mesa-m3 replaces; so the $(m3_mesa_name) was left out.
    The kernel install is complete. Remove it (sudo pacman -Rns $name), then run this again."
      return 1
    fi
    M3_PRO_MESA_REPLACED+=("$name $ver")
  done < <(pacman -Q 2>/dev/null || true)
  return 0
}

# USER is in GROUP (by the group database, as at the user's next login): 0 yes, 1 no, 2 unknown.
m3_pro_mesa_in_group() {
  local groups
  groups=$(id -nG -- "$1" 2>/dev/null) || return 2
  [[ " $groups " == *" $2 "* ]]
}

# On an M3 Pro with mesa-m3 installed after this run (installed, current or newer-kept): the
# package's udev rule gives the GPU's render node to group render, so the desktop user (the
# invoking user, never root) goes into it, from the next login. Sets M3_PRO_MESA_RENDER_ADDED and
# a summary line; m3_pro_mesa_record records it, and whether the user was in the group before
# this script first ran. A failure is a warning, and the exit status is 3.
m3_pro_mesa_render() {
  local user g=$M3_PRO_MESA_RENDER_GROUP rc=0
  M3_PRO_MESA_RENDER_ADDED=no M3_PRO_MESA_RENDER_NOTE=""
  m3_mesa_mac && [[ -n $M3_PRO_MESA_PACKAGE ]] || return 0
  case $M3_PRO_MESA_RESULT in installed | current | newer-kept) ;; *) return 0 ;; esac
  user=$(m3_pro_mesa_user)
  if [[ $user == root ]]; then
    M3_PRO_MESA_RENDER_NOTE="This ran as root, so no desktop user was added to the $g group the $(m3_mesa_gpu) now
   needs: add yours with  sudo gpasswd -a <user> $g  and log in again."
    return 0
  fi
  if [[ ! $user =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || ! getent group "$g" >/dev/null 2>&1; then
    warn "could not add $(printf '%q' "$user") to the $g group: no such group, or not a user name this script handles.
    Until the user is in it, the desktop renders in software."
    M3_PRO_MESA_RENDER_NOTE="$(printf '%q' "$user") is not in the $g group the $(m3_mesa_gpu) now needs (see the warning above);
   until then the desktop renders in software. The exit status is 3."
    M3_PRO_MESA_RENDER_ADDED=failed
    return 0
  fi
  m3_pro_mesa_in_group "$user" "$g" || rc=$?
  ((rc != 0)) || return 0
  if $sudo gpasswd -a "$user" "$g" >/dev/null && m3_pro_mesa_in_group "$user" "$g"; then
    M3_PRO_MESA_RENDER_ADDED=yes
    M3_PRO_MESA_RENDER_NOTE="Added $user to the $g group, which the $(m3_mesa_gpu) now needs. That takes effect at the
   next login: the reboot does that."
  else
    M3_PRO_MESA_RENDER_ADDED=failed
    warn "could not add $user to the $g group. Until $user is in it, the desktop renders in software: add
    them with  sudo gpasswd -a $user $g  and log in again."
    M3_PRO_MESA_RENDER_NOTE="$user is not in the $g group the $(m3_mesa_gpu) now needs (see the warning above). The exit
   status is 3."
  fi
}

# The record of this run, on an M3 Pro: $STATE/m3-pro-mesa, one "key=value" per line, for
# --uninstall and the lab check. Read it as data, never source it. The first line is
# schema=aurora.m3-pro-mesa-state/1; a record with another first line is not read. Every key
# appears exactly once, except integration_path, replaced_package, render_added_user,
# user_setup_path, user_setup_ignored, opt_out_path and record_error_key (one line per item,
# each path whole). The keys, in order:
#   schema; run_id (this run's, also in its summary line), release (this script's tag),
#   written_at (UTC, ISO 8601), boot_id, kernel (uname -r) and board (the first device-tree
#   compatible string), all of this run;
#   installer_sha256 and installer_source: the sha256 of the script file this run reads and
#     "file", or "unavailable" and "stdin" when it came through a pipe;
#   package, version, file, sha256 (this release's mesa-m3 entry) and prefix, an
#     integration_path line per file of the package that ties it into the system, and on an Air
#     a replaced_package line ("name version") per earlier Mesa package a run of this script
#     removed for it (mesa-m3-g15g);
#   result (below), installed_version (or none), installed_by (below), preexisting (the mesa-m3
#     version installed before this script first wrote a record on this Mac, or none);
#   user (the invoking user); render_member (yes, no or unknown: in group render, as at the next
#     login), render_preexisting (yes or no: in it before this script first ran; unknown when
#     not known), render_added (yes, no or failed: by this run), render_by_installer (yes when
#     a run of this script added this user and they are still in it), and a render_added_user
#     line per user any run of this script added who is still in it (this one and earlier
#     invoking users): --uninstall takes out only those;
#   user_setup_source (package-detector or installer-builtin: mesa-m3's detector, or this
#     script's byte copy of it), user_setup_session (the session desktops it judged for:
#     M3_PRO_MESA_SESSION_DESKTOPS, lowercased; unknown when no check answered), user_setup
#     (present, none, or unknown when a setting the session uses could not be ruled out or the
#     check failed: from the findings that apply to that session only), a user_setup_path line
#     per place of such a finding, and a user_setup_ignored line ("SCOPE CERTAINTY DETAIL") per
#     finding that session does not use (chonkstep's settings in a Hyprland session, say),
#     listed but never counted; opt_out (present or none), an opt_out_path line per switch-off file and
#     opt_out_cmdline (yes or no); all left as they are;
#   created_files: 0, as the package owns every file.
# result, what this run did:
#   installed     this run installed the package (none was installed, or an older one)
#   current       the same version was installed already: nothing new was installed
#   newer-kept    a newer mesa-m3 was installed, and was kept
#   skipped-flag  --no-m3-mesa: left out; one installed earlier stays
#   skipped-deps  a package it needs is missing or too old: left out (exit status 3)
#   failed        its pacman -U failed, or its file holds another package (exit status 3)
#   record-error  this run could not write its record (an error record, below)
# installed_by, who put the installed mesa-m3 there:
#   installer     this script's pacman -U. Set by a run that installs it, and kept by later runs
#                 while the same version stays installed.
#   owner         anyone else: installed before this script first ran, or replaced or
#                 installed outside it since (another version than the record's).
#   none          no mesa-m3 is installed.
# When a value holds a newline or a carriage return, or the write fails before its rename, the
# previous run's record must not look current: an error record replaces it (schema, the run's
# keys from run_id to installer_source, package, result=record-error, record_error (value or
# write), a record_error_key line per refused key, never its value, and installed_version,
# installed_by, preexisting, render_user (the user, or none when refused), render_preexisting,
# render_by_installer and the render_added_user lines, which keep the ownership history). If that write fails too, the
# previous record is moved aside to m3-pro-mesa.stale-<its written_at> (never over another file),
# so no record is current. Every failure is in the one warning, and the exit status is 4.
# --uninstall removes mesa-m3 only when installed_by is installer, the installed version is the
# record's, preexisting is none, and result is not record-error; and takes a user out of group
# render only when a render_added_user line names them, they are still in it, and result is not
# record-error. The lab's positive check:
# result installed or current, installed_by installer, installed_version the pinned version,
# sha256 the pinned package's sha256, and run_id the one in the summary line of the installer
# run under test. The record says what this script did, not what renders.
m3_pro_mesa_record() {
  local have by pre k v p w rc errs="" cause bad="" keys=() optout=() lines=() head=() elines=()
  local user rmember rpre rby u added=()
  local rec=$STATE/$M3_PRO_MESA_RECORD_NAME
  m3_mesa_mac && [[ -n $M3_PRO_MESA_PACKAGE ]] || return 0
  have=$(m3_pro_mesa_installed)
  pre=$M3_PRO_MESA_PREEXISTING
  if [[ -z $pre ]]; then pre=$(m3_pro_mesa_record_get preexisting); fi
  if [[ -z $have ]]; then
    by=none
  elif [[ $M3_PRO_MESA_RESULT == installed ]] ||
    [[ $(m3_pro_mesa_record_get installed_by) == installer && $(m3_pro_mesa_record_get installed_version) == "$have" ]]; then
    by=installer
  else
    by=owner
  fi
  m3_pro_mesa_setup_collect
  mapfile -d '' -t optout < <(m3_pro_mesa_optout)
  # The render group: as at the next login, before this script first ran, and whose doing it is.
  user=$(m3_pro_mesa_user) rc=0
  if [[ $user =~ ^[a-z_][a-z0-9_-]*[$]?$ ]]; then m3_pro_mesa_in_group "$user" "$M3_PRO_MESA_RENDER_GROUP" || rc=$?; else rc=2; fi
  case $rc in 0) rmember=yes ;; 1) rmember=no ;; *) rmember=unknown ;; esac
  rpre=$(m3_pro_mesa_record_get render_preexisting)
  if [[ -z $rpre || $rpre == unknown || $(m3_pro_mesa_record_user) != "$user" ]]; then
    case $M3_PRO_MESA_RENDER_ADDED:$rmember in yes:*) rpre=no ;; *:yes) rpre=yes ;; *:no) rpre=no ;; *) rpre=unknown ;; esac
  fi
  # The users a run of this script added and who are still in the group: this run's, and the
  # earlier ones the record names.
  rby=no
  while IFS= read -r u; do
    [[ $u =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || continue
    if [[ $u == "$user" ]]; then
      [[ $rmember == yes ]] && rby=yes
    elif m3_pro_mesa_in_group "$u" "$M3_PRO_MESA_RENDER_GROUP"; then
      added+=("$u")
    fi
  done < <(m3_pro_mesa_record_list render_added_user)
  if [[ $rmember == yes && $M3_PRO_MESA_RENDER_ADDED == yes ]]; then rby=yes; fi
  if [[ $rby == yes ]]; then added+=("$user"); fi
  head=("schema=$M3_PRO_MESA_SCHEMA" "run_id=$RUN_ID" "release=$TAG" "written_at=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    "boot_id=$(cat /proc/sys/kernel/random/boot_id 2>/dev/null || echo unavailable)"
    "kernel=$(uname -r)" "board=$(tr '\0' '\n' <"$DT/compatible" 2>/dev/null | sed -n 1p)"
    "installer_sha256=$SELF_SHA256" "installer_source=$SELF_SOURCE")
  lines=("${head[@]}" "package=$M3_PRO_MESA_NAME" "version=$(m3_pro_mesa_version)" "file=$(m3_pro_mesa_file)"
    "sha256=${M3_PRO_MESA_PACKAGE#* }" "prefix=$M3_PRO_MESA_PREFIX")
  while IFS= read -r p; do lines+=("integration_path=$p"); done <<<"$M3_PRO_MESA_INTEGRATION"
  for p in "${M3_PRO_MESA_REPLACED[@]}"; do lines+=("replaced_package=$p"); done
  while IFS= read -r p; do
    if [[ -n $p && " ${M3_PRO_MESA_REPLACED[*]} " != *" $p "* ]]; then lines+=("replaced_package=$p"); fi
  done < <(m3_pro_mesa_record_list replaced_package)
  lines+=("result=$M3_PRO_MESA_RESULT" "installed_version=${have:-none}" "installed_by=$by" "preexisting=${pre:-none}"
    "user=$user" "render_member=$rmember" "render_preexisting=$rpre" "render_added=$M3_PRO_MESA_RENDER_ADDED"
    "render_by_installer=$rby")
  for u in "${added[@]}"; do lines+=("render_added_user=$u"); done
  lines+=("user_setup_source=$M3_PRO_MESA_SETUP_SOURCE" "user_setup_session=$M3_PRO_MESA_SETUP_SESSION"
    "user_setup=$M3_PRO_MESA_SETUP_ANSWER")
  for ((k = 0; k < ${#M3_PRO_MESA_SETUP[@]}; k += 2)); do
    if [[ -n ${M3_PRO_MESA_SETUP[k]} ]]; then lines+=("user_setup_path=${M3_PRO_MESA_SETUP[k]}"); fi
  done
  for p in "${M3_PRO_MESA_SETUP_IGNORED[@]}"; do lines+=("user_setup_ignored=$p"); done
  if ((${#optout[@]})); then lines+=(opt_out=present); else lines+=(opt_out=none); fi
  v=no
  for ((k = 0; k < ${#optout[@]}; k += 2)); do
    if [[ -n ${optout[k]} ]]; then lines+=("opt_out_path=${optout[k]}"); else v=yes; fi
  done
  lines+=("opt_out_cmdline=$v" "created_files=0")
  for p in "${lines[@]}"; do
    w=${p#*=}
    if [[ $w == *[$'\n\r']* ]]; then
      bad+="${bad:+, }${p%%=*}=$(printf '%q' "$w")"
      [[ " ${keys[*]} " == *" ${p%%=*} "* ]] || keys+=("${p%%=*}")
    fi
  done

  M3_PRO_MESA_RECORD_RESULT=$M3_PRO_MESA_RESULT
  if [[ -z $bad ]]; then
    rc=0
    m3_pro_mesa_record_write "${lines[@]}" || rc=$?
    if ((rc == 0)); then
      M3_PRO_MESA_RECORD_WRITE=ok
    elif ((rc == 2)); then
      M3_PRO_MESA_RECORD_WRITE=unsynced
      warn "wrote $rec, but its durability is not confirmed: $M3_PRO_MESA_WRITE_ERR"
    fi
    if ((rc != 1)); then m3_pro_mesa_record_leftovers; return 0; fi
    cause=write errs="this run's record: $M3_PRO_MESA_WRITE_ERR"
  else
    cause=value errs="a value holds a newline or a carriage return ($bad)"
  fi

  # The error record: the run's keys (a refused one as "unavailable"), never a refused value.
  for p in "${head[@]}"; do
    if [[ ${p#*=} == *[$'\n\r']* ]]; then elines+=("${p%%=*}=unavailable"); else elines+=("$p"); fi
  done
  elines+=("package=$M3_PRO_MESA_NAME" "result=record-error" "record_error=$cause")
  for k in "${keys[@]}"; do elines+=("record_error_key=$k"); done
  elines+=("installed_version=${have:-none}" "installed_by=$by" "preexisting=${pre:-none}")
  if [[ $user =~ ^[a-z_][a-z0-9_-]*[$]?$ ]]; then elines+=("render_user=$user"); else elines+=("render_user=none"); fi
  elines+=("render_preexisting=$rpre" "render_by_installer=$rby")
  for u in "${added[@]}"; do elines+=("render_added_user=$u"); done
  M3_PRO_MESA_RECORD_RESULT="record-error"
  rc=0
  m3_pro_mesa_record_write "${elines[@]}" || rc=$?
  if ((rc == 0)); then
    M3_PRO_MESA_RECORD_WRITE=error-record
    warn "did not write this run's record ($errs). An error record (result=record-error) replaces
    $rec, so no earlier run's record looks current."
  elif ((rc == 2)); then
    M3_PRO_MESA_RECORD_WRITE=error-record-unsynced
    warn "did not write this run's record ($errs). An error record (result=record-error) replaces
    $rec, but its durability is not confirmed: $M3_PRO_MESA_WRITE_ERR"
  else
    errs+="; the error record: $M3_PRO_MESA_WRITE_ERR"
    M3_PRO_MESA_RECORD_RESULT=""
    if [[ ! -e $rec ]]; then
      M3_PRO_MESA_RECORD_WRITE=none
      warn "could not write this run's record or an error record ($errs). There is no record."
    elif m3_pro_mesa_record_aside; then
      M3_PRO_MESA_RECORD_WRITE=none
      warn "could not write this run's record or an error record ($errs). The previous record is
      now $M3_PRO_MESA_ASIDE, so no record is current.${M3_PRO_MESA_WRITE_ERR:+ But $M3_PRO_MESA_WRITE_ERR.}"
    else
      M3_PRO_MESA_RECORD_WRITE=stale
      warn "could not write this run's record or an error record ($errs), and could not move the
      previous record aside ($M3_PRO_MESA_WRITE_ERR): $rec is an earlier run's record, not this
      run's."
    fi
  fi
  m3_pro_mesa_record_leftovers
}

# Writes LINES as the record: an exclusive temporary file in $STATE made through $sudo (so
# root's), written in full, mode 0644, fsynced, renamed over the record, then $STATE fsynced.
# Returns 0 when all of that worked; 1 when it failed before the rename (the previous record, if
# any, is untouched, and only this call's own temporary file is removed); 2 when the new record
# is in place but the fsync of $STATE failed (written, durability not confirmed).
# M3_PRO_MESA_WRITE_ERR says what failed.
M3_PRO_MESA_WRITE_ERR=""
m3_pro_mesa_record_write() {
  local rec=$STATE/$M3_PRO_MESA_RECORD_NAME tmp
  M3_PRO_MESA_WRITE_ERR=""
  if ! tmp=$($sudo mktemp "$STATE/.$M3_PRO_MESA_RECORD_NAME.XXXXXX"); then
    M3_PRO_MESA_WRITE_ERR="no temporary file could be made in $STATE"
    return 1
  fi
  if ! printf '%s\n' "$@" | $sudo tee "$tmp" >/dev/null; then
    M3_PRO_MESA_WRITE_ERR="writing $tmp failed"
  elif ! $sudo chmod 0644 "$tmp"; then
    M3_PRO_MESA_WRITE_ERR="chmod 0644 $tmp failed"
  elif ! $sudo sync "$tmp"; then
    M3_PRO_MESA_WRITE_ERR="fsync of $tmp failed"
  elif ! $sudo mv -f "$tmp" "$rec"; then
    M3_PRO_MESA_WRITE_ERR="renaming $tmp to $rec failed"
  elif ! $sudo sync "$STATE"; then
    M3_PRO_MESA_WRITE_ERR="fsync of $STATE failed after the rename"
    return 2
  else
    return 0
  fi
  $sudo rm -f "$tmp" || M3_PRO_MESA_WRITE_ERR+="; $tmp could not be removed"
  return 1
}

# Moves the previous record aside to m3-pro-mesa.stale-<its written_at> (or .stale-unknown), with
# .1, .2, ... when that name is taken; never over an existing file. The new name is in
# M3_PRO_MESA_ASIDE. Returns 1, with M3_PRO_MESA_WRITE_ERR, when it could not; a failed fsync of
# $STATE afterwards goes in M3_PRO_MESA_WRITE_ERR too.
M3_PRO_MESA_ASIDE=""
m3_pro_mesa_record_aside() {
  local rec=$STATE/$M3_PRO_MESA_RECORD_NAME ts name n=0
  M3_PRO_MESA_WRITE_ERR="" M3_PRO_MESA_ASIDE=""
  ts=$(sed -n '/^written_at=/{s/^written_at=//p;q}' "$rec" 2>/dev/null) || ts=""
  [[ $ts =~ ^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z$ ]] || ts=unknown
  name=$rec.stale-$ts
  while ((n < 100)); do
    if [[ ! -e $name && ! -L $name ]]; then
      $sudo mv -n "$rec" "$name" || true
      if [[ ! -e $rec && -f $name ]]; then
        M3_PRO_MESA_ASIDE=$name
        $sudo sync "$STATE" || M3_PRO_MESA_WRITE_ERR="the fsync of $STATE failed after the move"
        return 0
      fi
      if [[ ! -e $name ]]; then
        M3_PRO_MESA_WRITE_ERR="moving $rec to $name failed"
        return 1
      fi
    fi
    n=$((n + 1)) name=$rec.stale-$ts.$n
  done
  M3_PRO_MESA_WRITE_ERR="no free name for $rec.stale-$ts"
  return 1
}

# Says, in one line, when temporary record files of another or an interrupted run are in
# $STATE. They are left alone: another run may still be writing one.
m3_pro_mesa_record_leftovers() {
  local left=() f
  for f in "$STATE/.$M3_PRO_MESA_RECORD_NAME".??????; do
    if [[ -e $f ]]; then left+=("$f"); fi
  done
  if ((${#left[@]})); then
    say "Left alone: ${#left[@]} temporary record file(s) of another or an interrupted run ($STATE/.$M3_PRO_MESA_RECORD_NAME.*)"
  fi
  return 0
}

# What the owner reads at the end of an install on an M3 Pro.
m3_pro_mesa_notice() {
  local want have setup="" optout p
  m3_mesa_mac && [[ -n $M3_PRO_MESA_PACKAGE ]] || return 0
  want=$(m3_pro_mesa_version) have=$(m3_pro_mesa_installed)
  case $M3_PRO_MESA_RESULT in
    installed)
      echo "   The $(m3_mesa_name) ($M3_PRO_MESA_NAME $want, in $M3_PRO_MESA_PREFIX) is installed. The new"
      echo "   graphics take effect at the next login: the reboot does that."
      if is_m3_pro && [[ $M3_MODE != handoff ]]; then
        echo "   This M3 Pro boots kernel-only (no GPU handoff), so at login the package finds no GPU,"
        echo "   leaves itself off and the desktop keeps rendering in software."
      fi ;;
    current)
      echo "   The $(m3_mesa_name) ($M3_PRO_MESA_NAME $have) was installed already; nothing new was installed." ;;
    newer-kept)
      echo "   A newer $M3_PRO_MESA_NAME ($have) than this release's ($want) is installed and was kept." ;;
    skipped-flag)
      if [[ -n $have ]]; then
        echo "   --no-m3-mesa: this run left the $(m3_mesa_name) out; the $M3_PRO_MESA_NAME $have installed"
        echo "   earlier stays installed (sudo pacman -R $M3_PRO_MESA_NAME removes it)."
      else
        echo "   --no-m3-mesa: the $(m3_mesa_name) was not installed, so the desktop renders in software."
      fi ;;
    skipped-deps)
      echo "   The $(m3_mesa_name) was NOT installed: it needs newer packages (see the warning above)."
      echo "   The kernel install is complete; until it is installed the desktop renders in software."
      echo "   Update the packages the warning names, then run this again. The exit status is 3." ;;
    failed)
      echo "   The $(m3_mesa_name) was NOT installed: pacman could not install it (see above). The"
      echo "   kernel install is complete; until it is installed the desktop renders in software."
      echo "   Run this again to try again. The exit status is 3." ;;
  esac
  if [[ -n $M3_PRO_MESA_RENDER_NOTE ]]; then echo "   $M3_PRO_MESA_RENDER_NOTE"; fi
  for p in "${M3_PRO_MESA_REPLACED[@]}"; do echo "   Removed ${p% *} ${p##* }, which mesa-m3 replaces."; done
  if is_m3_air && [[ -n $have ]]; then m3_pro_mesa_air_notice; fi
  if [[ -n $have ]]; then m3_pro_mesa_recovery; fi
  if ((${#M3_PRO_MESA_SETUP[@]})); then setup=$(printf '%s\0' "${M3_PRO_MESA_SETUP[@]}" | m3_pro_mesa_describe); fi
  if [[ -n $setup ]]; then
    echo "   Left as it is: $(m3_pro_mesa_user)'s own M3 Mesa setup ($setup)."
  fi
  if ((${#M3_PRO_MESA_SETUP_IGNORED[@]})); then
    echo "   Not counted, as a $M3_PRO_MESA_SESSION_DESKTOPS session does not read it (left as it is):"
    printf '     %s\n' "${M3_PRO_MESA_SETUP_IGNORED[@]}"
  fi
  optout=$(m3_pro_mesa_optout | m3_pro_mesa_describe)
  if [[ -n $optout && -n $have ]]; then
    echo "   Left as it is: the $(m3_mesa_name) is switched off at login by $optout."
  fi
  case $M3_PRO_MESA_RECORD_WRITE in
    ok) ;;
    unsynced | error-record-unsynced)
      echo "   This run's record ($STATE/$M3_PRO_MESA_RECORD_NAME) was written, but its durability is not"
      echo "   confirmed (see the warning above). The exit status is 4." ;;
    *)
      echo "   This run could not write its record ($STATE/$M3_PRO_MESA_RECORD_NAME; see the warning above)."
      echo "   The exit status is 4." ;;
  esac
  echo "   m3-pro-mesa record: run_id=$RUN_ID result=${M3_PRO_MESA_RECORD_RESULT:-none} write=${M3_PRO_MESA_RECORD_WRITE:-none}"
  return 0
}

# On an Air with mesa-m3: whether its login hook may use the GPU (M3_GPU_OPTIN), for the summary.
m3_pro_mesa_air_notice() {
  if ((M3_GPU_PERSISTENT)); then
    echo "   Experimental GPU profile $M3_GPU_PROFILE is persistent; each login checks the GPU before selecting Mesa."
    if [[ $M3_GPU_PROFILE == j613-25g83 ]]; then
      echo "   Native OpenGL uses /opt/mesa-m3/25g83; Vulkan hardware support is unavailable."
    fi
  elif [[ -f $M3_GPU_OPTIN ]]; then
    echo "   $M3_GPU_OPTIN is present: a login uses this Air's GPU, which is experimental, when"
    echo "   the GPU is started (sudo air-gpu-oneshot.sh start arms one boot); otherwise it renders in software."
  else
    echo "   This Air's GPU is experimental: GPU sessions need --m3-gpu-experiment (it writes"
    echo "   $M3_GPU_OPTIN). Until then each login says experimental and renders in software."
  fi
}

# How to see and undo what the M3's Mesa does at login (mesa-m3's state file, journal and
# switch-off), for the summary when mesa-m3 is installed.
m3_pro_mesa_recovery() {
  cat <<'M3_PRO_MESA_RECOVERY'
   Each login records whether the GPU graphics are on and why:
     cat /run/user/$(id -u)/mesa-m3-session.state     (or: journalctl -b -t mesa-m3)
   Reasons: active, opt-out, not-supported, experimental, no-gpu, no-display, no-access,
   incomplete-prefix, user-setup, user-setup-ldpath, user-setup-unknown, missing-soname,
   load-failed, log-unreadable, gpu-fault, previous-failed. When off, the desktop renders in software.
   If the desktop does not come up: Ctrl+Alt+F3, log in, run
     mkdir -p ~/.config/mesa-m3 && touch ~/.config/mesa-m3/disable
   and reboot, or add mesa_m3=off to the kernel command line at the boot menu. To turn it
   back on: rm ~/.config/mesa-m3/disable and log in again.
   If a login with GPU graphics did not reach a working desktop, the next logins of that
   boot render in software (previous-failed). To try the GPU again:
     rm ~/.local/state/mesa-m3/attempt
   then log out and in (or reboot).
   GPU graphics need the account in the render group and able to read the kernel log
   (the wheel or adm group).
M3_PRO_MESA_RECOVERY
}

# After a complete kernel install, on an M3 Pro: exit status 4 when this run's record is not
# written and durable (whatever happened to the Mesa: the summary line says), else 3 when the
# M3's Mesa was not installed or the desktop user could not be added to group render.
m3_pro_mesa_status() {
  m3_mesa_mac && [[ -n $M3_PRO_MESA_PACKAGE ]] || return 0
  [[ $M3_PRO_MESA_RECORD_WRITE == ok ]] || return 4
  case $M3_PRO_MESA_RESULT in skipped-deps | failed) return 3 ;; esac
  [[ $M3_PRO_MESA_RENDER_ADDED != failed ]] || return 3
  return 0
}

# --uninstall, when mesa-m3 stays (or was not there to remove): the users a run of this script
# added to group render stay in it; one line says so.
m3_pro_mesa_render_kept() {
  local u users=()
  m3_pro_mesa_record_ok || return 0
  while IFS= read -r u; do
    if [[ $u =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] && m3_pro_mesa_in_group "$u" "$M3_PRO_MESA_RENDER_GROUP"; then users+=("$u"); fi
  done < <(m3_pro_mesa_record_list render_added_user)
  ((${#users[@]})) || return 0
  say "Keeping ${users[*]} in the $M3_PRO_MESA_RENDER_GROUP group: this run did not remove $M3_PRO_MESA_NAME, and while it
    is installed its GPU render node is for that group (take them out with: sudo gpasswd -d <user> $M3_PRO_MESA_RENDER_GROUP)"
}

# --uninstall, on an M3 Pro (or a Mac with a record): removes mesa-m3 by its exact name when the
# record says this script installed the version that is installed and no mesa-m3 was there
# before this script first ran. The owner's own (installed by them, there before, or changed
# since) stays, and so does one this script has no readable record of. This script created no
# other file for it.
m3_pro_perf_remove() {
  local state=$STATE/m3-pro-perf-state role name pre have want
  [[ -f $state ]] || return 0
  for role in native vulkan; do
    if [[ $role == native ]]; then name=mesa-m3-native-pro; else name=mesa-m3-pro-vulkan; fi
    pre=$(sed -n "s/^pre_$role=//p" "$state")
    want=$(sed -n "s/^$role=//p" "$state")
    have=$(pacman -Q "$name" 2>/dev/null | cut -d' ' -f2) || continue
    if [[ $pre == none && -n $want && $have == "$want" ]]; then
      if [[ $role == native ]]; then $sudo systemctl disable --now mesa-m3-pro-performance.service || true; fi
      $sudo pacman -Rn --noconfirm "$name" || die "could not remove owned Pro Mesa package $name"
    else
      say "Keeping $name $have: it predates this installer or changed afterwards"
    fi
  done
}

m3_pro_mesa_remove() {
  m3_pro_perf_remove
  local rec=$STATE/$M3_PRO_MESA_RECORD_NAME
  # An M3 Pro as in 12.3; an Air only with a record (from 12.4 on), so an Air this script never
  # gave mesa-m3 runs --uninstall as before.
  is_m3_pro || [[ -f $rec ]] || return 0
  # The render group only goes with the package: while mesa-m3 stays, its udev rule keeps the
  # GPU's render node for that group, and a user taken out of it would lose the GPU.
  if m3_pro_mesa_remove_package; then
    m3_pro_mesa_render_remove
  else
    m3_pro_mesa_render_kept
  fi
}

# Removes mesa-m3 when this script installed it (see m3_pro_mesa_remove); 0 only when this run's
# pacman -Rn removed it.
m3_pro_mesa_remove_package() {
  local rec=$STATE/$M3_PRO_MESA_RECORD_NAME have pre
  have=$(m3_pro_mesa_installed)
  [[ -n $have ]] || return 1
  if [[ -f $rec ]] && ! m3_pro_mesa_record_ok; then
    warn "keeping $M3_PRO_MESA_NAME $have: $rec is not a record this script reads (its first line is not schema=$M3_PRO_MESA_SCHEMA)"
    return 1
  fi
  if [[ $(m3_pro_mesa_record_get result) == record-error ]]; then
    say "Keeping $M3_PRO_MESA_NAME $have: the last install could not write its record (result=record-error), so this script can't tell it installed it"
    return 1
  fi
  if [[ $(m3_pro_mesa_record_get installed_by) != installer || $(m3_pro_mesa_record_get installed_version) != "$have" ]]; then
    say "Keeping $M3_PRO_MESA_NAME $have: this script did not install it"
    return 1
  fi
  pre=$(m3_pro_mesa_record_get preexisting)
  if [[ $pre != none ]]; then
    say "Keeping $M3_PRO_MESA_NAME $have: a $M3_PRO_MESA_NAME (${pre:-of unknown version}) was installed before this script first ran"
    return 1
  fi
  if $sudo pacman -Rn --noconfirm "$M3_PRO_MESA_NAME"; then
    say "Removed the $(m3_mesa_name) ($M3_PRO_MESA_NAME $have)"
    return 0
  fi
  warn "could not remove $M3_PRO_MESA_NAME; remove it with: sudo pacman -R $M3_PRO_MESA_NAME"
  return 1
}

# --uninstall, after this run's pacman -Rn removed mesa-m3: takes a user out of group render only
# when the record (readable, and not an error record) names them as added by a run of this script
# and they are still in it. Never otherwise.
m3_pro_mesa_render_remove() {
  local u g=$M3_PRO_MESA_RENDER_GROUP users=()
  m3_pro_mesa_record_ok || return 0
  mapfile -t users < <(m3_pro_mesa_record_list render_added_user)
  ((${#users[@]})) || return 0
  if [[ $(m3_pro_mesa_record_get result) == record-error ]]; then
    say "Keeping ${users[*]} in the $g group: the last install could not write its record (result=record-error)"
    return 0
  fi
  for u in "${users[@]}"; do
    if [[ ! $u =~ ^[a-z_][a-z0-9_-]*[$]?$ ]] || ! m3_pro_mesa_in_group "$u" "$g"; then continue; fi
    if $sudo gpasswd -d "$u" "$g" >/dev/null; then
      say "Took $u out of the $g group, which this script had added them to (from the next login)"
    else
      warn "could not take $u out of the $g group; do it with: sudo gpasswd -d $u $g"
    fi
  done
}

this_board() {
  tr '\0' '\n' <"$DT/compatible" 2>/dev/null | sed -n '1s/^apple,//p'
}

# Before anything rebuilds boot.bin with this release's m1n1 (every Mac that
# m1n1_for_this_mac names), keep the one this boot came up on next to it on the
# EFI partition, where macOS can reach it, and say how to put it back. A later
# run of the same release keeps the first copy.
bootbin_backup() {
  local board
  m1n1_for_this_mac || return 0
  board=$(this_board)
  if [[ $M3_MODE == handoff ]] && is_m3_air; then
    keep_bootbin_on_esp "This replaces the boot loader of this M3 MacBook Air ($board)
    with m1n1-aurora and its $(m3_handoff_name)."
  elif [[ $M3_MODE == handoff ]] && is_m3_max; then
    keep_bootbin_on_esp "This replaces the boot loader of this M3 Max ($board) with
    m1n1-aurora and its $(m3_handoff_name)."
    m3max_bootbin_record
  elif [[ $M3_MODE == handoff ]]; then
    keep_bootbin_on_esp "This replaces the boot loader of this M3 Pro ($board) with
    m1n1-aurora and its display and GPU handoff."
  elif [[ -n $board && " $UNPROVEN_SEP_BOARDS " == *" $board "* ]]; then
    keep_bootbin_on_esp "Touch ID on this Mac model ($board) is new in $VERSION, and nobody has
    booted it on this model yet. This also replaces its boot loader with m1n1-aurora
    $(m1n1_version), the one m1n1 every Mac gets from $VERSION on."
  else
    keep_bootbin_on_esp "This replaces this Mac's boot loader with m1n1-aurora $(m1n1_version),
    the one m1n1 every Mac gets from $VERSION on."
  fi
}

# Replace a file on the EFI partition without ever leaving a partial one in
# its place: copy next to it, compare, then rename.
replace_on_esp() {
  local src=$1 dst=$2
  if ! { $sudo cp "$src" "$dst.new" && $sudo cmp -s "$src" "$dst.new" && sync &&
    $sudo mv "$dst.new" "$dst" && sync; }; then
    die "could not write $dst; the previous one is still there"
  fi
}

keep_bootbin_on_esp() {
  local why=$1 target keep uuid
  if target=$(esp_bootbin); then
    keep=$target.before-$VERSION
    # Only a complete, compared copy gets the final name, so a later run can
    # trust one it finds.
    if ! $sudo test -f "$keep"; then
      if ! { $sudo cp "$target" "$keep.new" && $sudo cmp -s "$target" "$keep.new" && sync &&
        $sudo mv "$keep.new" "$keep"; }; then
        die "could not keep a copy of $target; nothing was installed"
      fi
    fi
    uuid=$(findmnt -no PARTUUID --target "${target%/m1n1/boot.bin}")
    warn "$why If the Mac stops in m1n1 after this install (m1n1
    text on screen, often \"No valid payload found\", and no boot menu), put the
    boot loader it booted with back from macOS:
      1. Hold the power button until the Mac turns off, then press and hold it
         again for the startup options, and start macOS (or Options, then
         Utilities > Terminal).
      2. In Terminal, run: diskutil list
         Find the partition whose UUID is $uuid (any case) with:
           diskutil info diskNsM | grep -i 'partition uuid'
      3. sudo diskutil mount diskNsM   (it prints the /Volumes path)
      4. cp -X '<that path>/m1n1/boot.bin.before-$VERSION' '<that path>/m1n1/boot.bin'
    If it boots but something is wrong, the same copy goes back from Linux with:
      sudo cp '$keep' '$target.new' && sync && sudo mv '$target.new' '$target' && sync
    After either, start Linux and run these two lines. The first keeps updates from
    rebuilding the new boot loader; the second records that its m1n1 failed on this
    Mac, so no later run of this script puts it back:
      echo M1N1_UPDATE_DISABLED=1 | sudo tee -a $UPDATE_M1N1_CONF
      echo $M1N1_BIN_SHA | sudo tee -a $STATE/m1n1-failed
    The copy is kept at $keep. Either way, please report it at
    $ISSUE_URL"
    return 0
  fi
  die "could not find m1n1's boot.bin on the EFI partition to keep a copy of; nothing was installed"
}

current_kernel() {
  if pacman -Q linux-aurora >/dev/null 2>&1; then echo linux-aurora
  elif pacman -Q linux-asahi >/dev/null 2>&1; then echo linux-asahi
  fi
}

preflight() {
  [[ $(uname -m) == aarch64 ]] || die "this is not an aarch64 machine"
  grep -qa 'apple,' /proc/device-tree/compatible 2>/dev/null || die "this is not an Apple Silicon Mac"
  command -v pacman >/dev/null || die "pacman not found; this is for Omarchy on Arch Linux ARM"
  [[ -n $(current_kernel) ]] || die "neither linux-asahi nor linux-aurora is installed; this Mac's kernel is not one this script replaces"
  [[ $(boot_chain) != unknown ]] || die "could not find GRUB or Limine on this Mac; not touching its boot setup"
}

remove_pin() {
  $sudo sed -i "/^$(sed 's/[][\/.*^$]/\\&/g' <<<"$PIN_BEGIN")\$/,/^$PIN_END\$/d" /etc/pacman.conf
}

add_pin() {
  remove_pin
  # IgnorePkg lines accumulate, so any existing IgnorePkg stays in force.
  $sudo sed -i "/^\[options\]/a $PIN_BEGIN\nIgnorePkg = $PINNED\n$PIN_END" /etc/pacman.conf
}

snapshot() {
  if command -v snapper >/dev/null && $sudo snapper list-configs 2>/dev/null | grep -q '^root'; then
    say "Taking a snapper snapshot"
    $sudo snapper -c root create -c important -d "before $1" || warn "snapshot failed; continuing"
  fi
}

# /boot has to hold the fallback copy of the old kernel and the new kernel's
# image and initramfs; checked before anything changes.
boot_space() {
  local kernel=$1 need free
  need=$(du -cm "/boot/vmlinuz-$kernel" "/boot/initramfs-$kernel.img" 2>/dev/null | tail -1 | cut -f1)
  [[ -f $STATE/previous ]] && need=0
  # Room for linux-aurora's image and initramfs, unless it is already installed.
  if [[ $kernel == linux-aurora ]]; then need=$(( need + 20 )); else need=$(( need + 90 )); fi
  free=$(df -m --output=avail /boot | tail -1 | tr -d ' ')
  (( free >= need )) || die "/boot has ${free} MB free and this needs about ${need} MB (the old kernel kept as a fallback plus the new one).
Free space in /boot first (old test kernels, for example), then run this again. Nothing was changed."
}

# GRUB: keep the running kernel bootable from the menu, whatever pacman removes.
keep_grub_fallback() {
  local kernel=$1 release="" dir root_uuid subvol cmdline linux initrd
  # The modules that belong to the package being replaced (not necessarily
  # the running kernel, which may be a hand-installed test kernel).
  for dir in /usr/lib/modules/*/; do
    [[ $(cat "${dir}pkgbase" 2>/dev/null) == "$kernel" ]] && release=$(basename "$dir")
  done
  [[ -n $release ]] || { warn "could not find the modules of $kernel; no fallback entry"; return 0; }
  [[ -f /boot/vmlinuz-$kernel && -f /boot/initramfs-$kernel.img ]] || { warn "no /boot/vmlinuz-$kernel to keep as a fallback"; return 0; }
  $sudo install -d "$STATE"
  if ! $sudo cp /boot/vmlinuz-$kernel /boot/vmlinuz-aurora-sep-previous ||
    ! $sudo cp /boot/initramfs-$kernel.img /boot/initramfs-aurora-sep-previous.img; then
    $sudo rm -f /boot/vmlinuz-aurora-sep-previous /boot/initramfs-aurora-sep-previous.img
    die "could not copy the previous kernel to /boot (out of space?). Nothing else was changed."
  fi
  # pacman removes the old kernel's modules; keep the running kernel's for the fallback.
  $sudo cp -a "/usr/lib/modules/$release" "$STATE/modules-$release"
  # The fallback's device trees must never be what update-m1n1 picks.
  $sudo rm -rf "$STATE/modules-$release/dtbs"
  echo "$kernel $release" | $sudo tee "$STATE/previous" >/dev/null
  # kernel-modules-hook's linux-modules-cleanup.service deletes every
  # /usr/lib/modules tree that no package owns and that is not the running
  # kernel, so the fallback's modules live in $STATE and are put back only when
  # the fallback kernel itself boots, before anything loads a module.
  $sudo tee /etc/systemd/system/aurora-sep-fallback-modules.service >/dev/null <<EOF
[Unit]
Description=Restore the modules of the pre-aurora-sep fallback kernel
DefaultDependencies=no
ConditionKernelVersion=$release
ConditionPathExists=!/usr/lib/modules/$release/modules.dep
After=systemd-remount-fs.service
Before=systemd-modules-load.service systemd-udevd.service systemd-udev-trigger.service linux-modules-cleanup.service

[Service]
Type=oneshot
ExecStart=/usr/bin/cp -a $STATE/modules-$release /usr/lib/modules/$release

[Install]
WantedBy=sysinit.target
EOF
  $sudo systemctl daemon-reload
  $sudo systemctl enable aurora-sep-fallback-modules.service

  root_uuid=$(findmnt -no UUID /)
  subvol=$(findmnt -no FSROOT /)
  cmdline=$(sed -e 's/BOOT_IMAGE=[^ ]*//' /proc/cmdline)
  if ((M3_GPU_PERSISTENT)); then
    cmdline=$(sed -E 's/(^| )asahi\.t8122[_-]start(=[^ ]*)?//g; s/(^| )mesa_m3=[^ ]*//g' <<<"$cmdline")
    cmdline+=" asahi.t8122_start=0 mesa_m3=off"
  fi
  if [[ $(findmnt -no FSTYPE /boot) == vfat ]]; then
    linux=/vmlinuz-aurora-sep-previous initrd=/initramfs-aurora-sep-previous.img
    search="search --no-floppy --fs-uuid --set=root $(findmnt -no UUID /boot)"
  else
    linux="${subvol%/}/boot/vmlinuz-aurora-sep-previous" initrd="${subvol%/}/boot/initramfs-aurora-sep-previous.img"
    search="search --no-floppy --fs-uuid --set=root $root_uuid"
  fi
  $sudo tee /etc/grub.d/42_aurora_sep_previous >/dev/null <<EOF
#!/bin/sh
cat <<'MENU'
menuentry 'Previous kernel ($kernel $release, before aurora-sep)' --class omarchy --class gnu-linux --id $FALLBACK_ID {
    load_video
    set gfxpayload=keep
    insmod part_gpt
    insmod fat
    insmod btrfs
    $search
    linux $linux $cmdline
    initrd $initrd
}
MENU
EOF
  $sudo chmod 755 /etc/grub.d/42_aurora_sep_previous
}

# update-m1n1 takes the device trees of the highest-versioned *-ARCH kernel
# directory, which after this install can be the old linux-asahi's (a newer
# version number, kept for the fallback entry) without the Touch ID sensor node.
# Point it at whatever directory linux-aurora owns, now and after its updates.
m1n1_update() {
  local target conf=$UPDATE_M1N1_CONF tmp
  local -a drop=(-e '/^# aurora-sep: build m1n1/,/^DTBS=/d' -e '/^DTBS=/d')
  [[ -f $conf && ! -f $STATE/update-m1n1.default.saved ]] &&
    $sudo cp "$conf" "$STATE/update-m1n1.default.saved"
  # Replace only the DTBS setting and keep the rest of the file: a MacBook
  # Neo's M1N1= and U_BOOT= point at its own J700 builds, and dropping them
  # would rebuild boot.bin from an m1n1 that cannot boot it. A Neo on
  # M1N1_PACKAGE (NEO_AURORA_M1N1=1) drops only its M1N1=, so update-m1n1
  # takes the packaged m1n1 with the Neo's own U-Boot. --uninstall puts the
  # saved file back.
  if is_neo && [[ $NEO_AURORA_M1N1 == 1 ]]; then
    drop+=(-e '/^[[:space:]]*\(export[[:space:]]\{1,\}\)\{0,1\}M1N1=/d')
  fi
  tmp=$(mktemp)
  if [[ -f $conf ]]; then
    $sudo sed "${drop[@]}" "$conf" >"$tmp"
  fi
  cat >>"$tmp" <<'EOF'
# aurora-sep: build m1n1's stage 2 from the device trees the installed
# linux-aurora package owns, which carry the Touch ID sensor node.
#
# Not from a module directory whose pkgbase says linux-aurora: right after an
# upgrade the running kernel's modules are restored unowned but still carry that
# pkgbase, so both kernels' DTBs would be bundled. m1n1 keeps the last matching
# DTB, and the glob sorts 11.10 before 11.9, so the stale one can win.
# (update-m1n1 runs under sh's set -e, which on Arch also applies inside
# $(...), so the pipeline must succeed even when grep matches nothing.)
DTBS=$(pacman -Qlq linux-aurora 2>/dev/null | grep '/dtbs/[^/]*\.dtb$' || true)
EOF
  if ((NEO_GPU)); then echo "M1N1=/usr/lib/m1n1-neo/m1n1.bin" >>"$tmp"; fi
  $sudo install -m 644 "$tmp" "$conf"
  rm -f "$tmp"
  if [[ ! -f $STATE/boot.bin.saved ]] && target=$(esp_bootbin); then
    $sudo cp "$target" "$STATE/boot.bin.saved"
  fi
  # update-m1n1 exits without a word while this is set; say so rather than
  # claim a rebuild.
  if update_m1n1_frozen; then
    warn "$conf sets M1N1_UPDATE_DISABLED, so m1n1's boot.bin was not rebuilt;
    it keeps the m1n1 and device trees it already had"
    return 0
  fi
  say "Rebuilding m1n1 with the aurora device trees"
  $sudo update-m1n1 || die "update-m1n1 failed; the previous boot.bin is saved in $STATE/boot.bin.saved"
}

grub_update() {
  # mkinitcpio's hook installs /boot/vmlinuz-linux-aurora; a preset whose
  # kernel is gone would make later mkinitcpio -P runs fail.
  if [[ -f /etc/mkinitcpio.d/linux-asahi.preset ]] && ! pacman -Q linux-asahi >/dev/null 2>&1; then
    $sudo mv /etc/mkinitcpio.d/linux-asahi.preset "$STATE/linux-asahi.preset.saved"
  fi
  say "Regenerating the GRUB menu"
  $sudo grub-mkconfig -o /boot/grub/grub.cfg

  # Select the linux-aurora entry by its id, never by position. Entry 0 is
  # whichever kernel grub-mkconfig considers newest, and a Mac that has been
  # used for testing often has hand-installed vmlinuz-* files in /boot with no
  # matching initramfs. grub-mkconfig happily writes those as entries with no
  # initrd line at all, and booting one hangs before it can mount root.
  # grub.cfg can be root-only, so read it through sudo like the ESP check above.
  # grub-mkconfig puts every kernel but its idea of the newest inside the
  # "Advanced options" submenu, and a bare id only resolves against top-level
  # entries: setting one that lives in the submenu silently falls back to entry
  # 0. The documented way to reach it is the submenu>entry path, and it has to
  # be quoted in /etc/default/grub or the > is a shell redirect.
  local entry submenu target
  entry=$($sudo grep -oE 'gnulinux-linux-aurora-advanced-[0-9a-f-]+' /boot/grub/grub.cfg | head -1)
  [[ -n $entry ]] || die "grub.cfg has no linux-aurora entry; the previous kernel is still the fallback entry"
  submenu=$($sudo grep -oE 'gnulinux-advanced-[0-9a-f-]+' /boot/grub/grub.cfg | head -1)
  if [[ -n $submenu ]]; then target="$submenu>$entry"; else target="$entry"; fi

  if ! grep -qxF "GRUB_DEFAULT=\"$target\"" /etc/default/grub; then
    [[ -f $STATE/grub.default.saved ]] || $sudo cp /etc/default/grub "$STATE/grub.default.saved"
    if grep -qE '^GRUB_DEFAULT=' /etc/default/grub; then
      $sudo sed -i "s|^GRUB_DEFAULT=.*|GRUB_DEFAULT=\"$target\"|" /etc/default/grub
    else
      echo "GRUB_DEFAULT=\"$target\"" | $sudo tee -a /etc/default/grub >/dev/null
    fi
    $sudo grub-mkconfig -o /boot/grub/grub.cfg
  fi

  # Prove the default resolves to an entry that can actually boot: the path
  # must be written out, and that entry must carry both a kernel and an initrd.
  # A vmlinuz-* in /boot with no matching initramfs generates an entry with no
  # initrd line at all, which hangs before it can mount root.
  $sudo grep -qF "set default=\"$target\"" /boot/grub/grub.cfg ||
    die "GRUB's default entry is not linux-aurora. Not leaving this Mac pointing at an unbootable default: pick 'Omarchy Linux, with Linux linux-aurora' from the menu, or run --uninstall."
  $sudo awk -v id="$entry" '
    /^[\t ]*menuentry/ && index($0, id) { found = 1 }
    found && /vmlinuz-linux-aurora/ { k = 1 }
    found && /initramfs-linux-aurora\.img/ { i = 1 }
    found && /^[\t ]*}/ { exit }
    END { exit !(k && i) }' /boot/grub/grub.cfg ||
    die "the linux-aurora GRUB entry is missing its kernel or initramfs line; refusing to make it the default"

  # Stray hand-installed kernels poison the auto-generated top entry.
  local stray="" k n
  for k in /boot/vmlinuz-*; do
    n=${k#/boot/vmlinuz-}
    [[ -e /boot/initramfs-$n.img ]] || stray+=" $n"
  done
  [[ -z $stray ]] ||
    warn "these kernels in /boot have no initramfs and boot to a black screen if picked:$stray"
}

calibration() {
  local node name="" out board dtb
  # The driver reads firmware-name from the device tree it boots, which is the
  # aurora one installed above. When this runs from linux-asahi, the running
  # tree has no sensor node at all, so ask the installed aurora DTB first.
  board=$(tr '\0' '\n' </proc/device-tree/compatible 2>/dev/null | sed -n '1s/^apple,//p')
  if [[ -n $board ]]; then
    dtb=$(pacman -Qlq linux-aurora 2>/dev/null | grep -m1 "/dtbs/t[0-9]*-$board\.dtb$" || true)
    [[ -n $dtb && -f $dtb ]] &&
      name=$(grep -aoE -m1 'apple/mesacal-[A-Za-z0-9_-]+\.bin' "$dtb" | head -1 || true)
  fi
  if [[ -z $name ]]; then
    for node in /proc/device-tree/soc*/spi*/*/ /proc/device-tree/*/spi*/*/; do
      [[ -f $node/compatible ]] && grep -qa mesa "$node/compatible" || continue
      [[ -f $node/firmware-name ]] && name=$(tr -d '\0' <"$node/firmware-name")
      break
    done
  fi
  name=${name:-apple/mesa_calibration.bin}
  out=/usr/lib/firmware/$name
  if [[ -s $out ]]; then
    say "Touch ID calibration already present: $out"
    return 0
  fi
  say "Extracting this Mac's Touch ID calibration (read-only) to $out"
  $sudo install -d -m 755 "$(dirname "$out")"
  $sudo /usr/lib/aurora-touchid/extract-mesa-calibration -o "$out" ||
    warn "could not extract the calibration; Touch ID will not work until it is present at $out"
}

# The first line in modprobe.d that blacklists or redirects apple_sep, other
# than our own file. Either spelling is the owner saying "do not load this".
sep_block_line() {
  local f
  for f in /etc/modprobe.d/*.conf /run/modprobe.d/*.conf /usr/lib/modprobe.d/*.conf; do
    [[ -f $f && $f != "$MODPROBE_CONF" ]] || continue
    grep -m1 -HnE '^[[:space:]]*(blacklist|install)[[:space:]]+apple[-_]sep([[:space:]]|$)' "$f" 2>/dev/null &&
      return 0
  done
  return 0
}

# With its defaults the driver provisions an identity keybag and writes to the
# enclave's anti-replay store (xART) on the first boot of a Mac that has a
# Touch ID profile -- no enrolment needed. That is the intended behaviour, but
# the owner has to be able to opt out, and a block they already set must win.
sep_policy() {
  local block
  block=$(sep_block_line)
  if [[ -n $block ]]; then
    warn "apple_sep is blocked on this Mac ($block).
    Leaving it that way: the SEP service is not enabled. Note that a plain
    'blacklist' line would NOT have stopped the service, which loads the driver
    by name; that is why this script checks for it."
    $sudo systemctl disable apple-sep.path apple-sep.service 2>/dev/null || true
    return 0
  fi
  if (( READ_ONLY )); then
    printf '%s\n' \
      "# aurora-sep --read-only: the driver attaches and reports, but never writes" \
      "# to the enclave. Delete this file to allow enrolment." \
      "options apple_sep xart_writes=0 provision_keybag=0" |
      $sudo tee "$MODPROBE_CONF" >/dev/null
    say "Read-only: the SEP driver will attach but not write to the enclave ($MODPROBE_CONF)"
  elif [[ -f $MODPROBE_CONF ]]; then
    # Re-running to update must never quietly turn writes back on.
    say "Still read-only from an earlier --read-only install; delete $MODPROBE_CONF to allow enrolment"
  fi
  $sudo systemctl enable apple-sep.path apple-sep.service
}

sep_write_notice() {
  (( READ_ONLY )) && return 0
  [[ -f $MODPROBE_CONF || -n $(sep_block_line) ]] && return 0
  warn "on the first boot of a Mac with a Touch ID profile, the SEP driver creates
    an identity keybag and writes to the enclave's anti-replay store. Use a Mac
    you can DFU-restore. If you cannot, press Ctrl-C now and run with --read-only:
      curl -fsSL $LATEST_URL | bash -s -- --read-only"
  return 0
}

# omarchy-ane-dkms builds its own Neural Engine modules into updates/dkms,
# which modprobe prefers over the ones this kernel ships. Say so rather than
# remove it: it also carries the M2 firmware fetch its owner may rely on.
ane_dkms_notice() {
  pacman -Q omarchy-ane-dkms >/dev/null 2>&1 || return 0
  warn "omarchy-ane-dkms is installed. Its Neural Engine modules take precedence
    over the driver built into this kernel, so a test would exercise that
    package's driver instead. To test this kernel's driver, remove it first:
      sudo pacman -R omarchy-ane-dkms
    Removing it also removes the Neural Engine firmware it fetched on an M2
    Pro/Max."
}

# The MacBook Neo's Wi-Fi and Bluetooth need this Neo's own firmware,
# calibration and country files from its macOS; none ship with Linux, and
# another unit's files can't be substituted. Report what is there without
# failing the install: the kernel runs without them, the radios stay off.
# File names follow Documentation/networking/device_drivers/wifi/mt7932-neo.rst.
neo_radio_notice() {
  local fw=/usr/lib/firmware/mediatek f missing="" mac cc=""
  for f in IZUBA_WIFI_MT7932_patch_mcu_1_2_hdr.bin IZUBA_W7932_2.bin ppr.bin \
    config-original.bin wcal.bin oca2.bin; do
    [[ -s $fw/mt7932/$f ]] || missing+=" mt7932/$f"
  done
  # The policy is per country with no fallback to the world file once a
  # country is set; world-XZ.bin only covers the unset ("00") case.
  if command -v iw >/dev/null; then
    cc=$(iw reg get 2>/dev/null | awk '$1=="global"{g=1; next} g && $1=="country"{sub(":","",$2); print $2; exit}')
  fi
  if [[ -z $cc || $cc == 00 ]]; then
    [[ -s $fw/mt7932/policy/world-XZ.bin ]] || missing+=" mt7932/policy/world-XZ.bin"
  else
    [[ -s $fw/mt7932/policy/$cc.bin ]] || missing+=" mt7932/policy/$cc.bin (country $cc)"
  fi
  mac=$(find /proc/device-tree -path '*wifi*' -name local-mac-address 2>/dev/null | head -1)
  if [[ -n $mac ]] && od -An -tx1 "$mac" | grep -q '[1-9a-f]'; then
    say "Wi-Fi MAC address: provided by this Neo's m1n1"
  else
    warn "m1n1 did not provide the Wi-Fi MAC address (wifi0 local-mac-address); Wi-Fi will refuse to start"
  fi
  if [[ -z $missing ]]; then
    say "Wi-Fi firmware, calibration and country files: all present under $fw/mt7932"
  else
    warn "Wi-Fi needs this Neo's own files under $fw, and these are missing:$missing
    They come from this Neo's own macOS and can't be shared between machines.
    See Documentation/networking/device_drivers/wifi/mt7932-neo.rst in
    iconidentify/aurora-linux. Wi-Fi stays off until they are all there."
  fi
  say "MacBook Neo, read before you reboot:
    - Sleep isn't supported on the Neo yet. While Wi-Fi/Bluetooth support is
      active (the default) the kernel refuses suspend, so closing the lid does
      nothing: shut down instead of putting it in a bag.
    - Wi-Fi works on 2.4 GHz and on 5 GHz channels 36-48 only, with WPA2 (AES)
      or open networks. 5 GHz networks on channels 149-165 won't be listed.
    - Bluetooth is off by default in this release."
}

# The first install records what it found, before anything changes, so that
# --uninstall can put it back even after a run that stopped partway:
# update-m1n1's configuration (without this script's own M3 freeze from 11.36),
# or that there was none; the bring-up's freeze; and boot.bin itself.
snapshot_boot_state() {
  local tmp target
  if [[ ! -e $STATE/update-m1n1.default.saved && ! -e $STATE/update-m1n1.absent ]]; then
    if [[ -f $UPDATE_M1N1_CONF ]]; then
      tmp=$(mktemp)
      m3_unfrozen >"$tmp"
      $sudo install -m 644 "$tmp" "$STATE/update-m1n1.default.saved"
      rm -f "$tmp"
    else
      $sudo touch "$STATE/update-m1n1.absent"
    fi
  fi
  if [[ -f $UPDATE_M1N1_CONF && ! -f $STATE/update-m1n1.m3gpu.saved ]] &&
    grep -qxF "${M3GPU_FREEZE[0]}" "$UPDATE_M1N1_CONF"; then
    $sudo cp -p "$UPDATE_M1N1_CONF" "$STATE/update-m1n1.m3gpu.saved"
    if [[ -e $M3GPU_MARKER ]]; then $sudo touch "$STATE/m3gpu-marker.saved"; fi
  fi
  if [[ ! -f $STATE/boot.bin.saved ]] && target=$(esp_bootbin); then
    $sudo cp "$target" "$STATE/boot.bin.saved"
  fi
  return 0
}

# The packages this Mac gets, one "file sha256" per line: PACKAGES, and
# M1N1_PACKAGE where m1n1_for_this_mac says so. Needs m3_plan first.
packages_for_this_mac() {
  printf '%s\n' "${PACKAGES[@]}"
  if m1n1_for_this_mac; then echo "$M1N1_PACKAGE"; fi
  return 0
}

FROZEN_PACKAGES=0
FROZEN_TRANSACTION_CONFIG=""
FROZEN_TRANSACTION_FILES=()
PACMAN_CONFIG=/etc/pacman.conf

frozen_package_detection() {
  local holds="" token
  local -a held_words=()
  FROZEN_PACKAGES=0
  command -v pacman-conf >/dev/null || return 0
  holds=$(pacman-conf --config "$PACMAN_CONFIG" IgnorePkg) ||
    die "could not read package holds. Nothing was installed."
  read -ra held_words <<<"${holds//$'\n'/ }"
  for token in "${held_words[@]}"; do
    if [[ $token == '*' ]]; then FROZEN_PACKAGES=1; fi
  done
}

frozen_dependency_prepare() {
  ((FROZEN_PACKAGES)) || return 0
  local archive need
  local -a args=(--config "$PACMAN_CONFIG" --work "$work" --require fprintd)
  local -a mesa_needs=()
  if [[ -n $M3_PRO_MESA_PACKAGE && -f $work/$(m3_pro_mesa_file) ]]; then
    read -ra mesa_needs <<<"$M3_PRO_MESA_NEEDS"
    for need in "${mesa_needs[@]}"; do args+=(--require "$need"); done
  fi
  if ((M3_GPU_EXPERIMENT)); then args+=(--require python --require vulkan-icd-loader); fi
  for archive in "$@"; do args+=(--candidate "$archive"); done
  args+=(--allow-remove linux-asahi --allow-remove linux-asahi-headers
         --allow-remove m1n1 --allow-remove mesa-m3-g15g)
  cat >"$work/frozen-dependencies.py" <<'FROZEN_DEPENDENCIES_PY'
#!/usr/bin/env python3
"""Admit missing repository dependencies without changing installed packages."""
import argparse
import ctypes
import ctypes.util
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tarfile
import tempfile

NAME = re.compile(r"[A-Za-z0-9@_+][A-Za-z0-9@_.+:-]*\Z")
DEPEND = re.compile(r"([^<>=\s]+)(>=|<=|=|>|<)?([^\s]*)\Z")
FORMAT = "%n\t%v\t%h\t%l"


def run(args, codes=(0,)):
    result = subprocess.run(args, text=True, capture_output=True,
                            env={**os.environ, "LC_ALL": "C"})
    if result.returncode not in codes:
        raise ValueError("Command failed: " + " ".join(args) + "\n" + result.stderr + result.stdout)
    return result


def metadata(text, pkginfo=False):
    fields = {}
    if pkginfo:
        for line in text.splitlines():
            if " = " in line:
                key, value = line.split(" = ", 1)
                fields.setdefault(key, []).append(value)
    else:
        key = None
        for line in text.splitlines():
            if line.startswith("%") and line.endswith("%"):
                key = line.strip("%")
                fields.setdefault(key, [])
            elif line and key:
                fields[key].append(line)
    aliases = {"name": "pkgname" if pkginfo else "NAME",
               "version": "pkgver" if pkginfo else "VERSION",
               "arch": "arch" if pkginfo else "ARCH",
               "depends": "depend" if pkginfo else "DEPENDS",
               "provides": "provides" if pkginfo else "PROVIDES",
               "conflicts": "conflict" if pkginfo else "CONFLICTS",
               "replaces": "replaces" if pkginfo else "REPLACES"}
    result = {key: fields.get(value, []) for key, value in aliases.items()}
    for key in ("name", "version", "arch"):
        if len(result[key]) != 1:
            raise ValueError("Invalid package metadata " + key)
        result[key] = result[key][0]
    if not NAME.fullmatch(result["name"]):
        raise ValueError("Invalid package name")
    return result


def archive(path):
    result = run(["bsdtar", "-xOf", str(path), ".PKGINFO"])
    return metadata(result.stdout, True)


def sha(path):
    with open(path, "rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def dependency(value):
    match = DEPEND.fullmatch(value)
    if not match or not NAME.fullmatch(match[1]) or (match[2] and not match[3]):
        raise ValueError("Invalid dependency: " + value)
    return match[1], match[2], match[3]


_alpm = ctypes.CDLL(ctypes.util.find_library("alpm"))
_alpm.alpm_pkg_vercmp.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
_alpm.alpm_pkg_vercmp.restype = ctypes.c_int


def satisfies(pkg, requirement):
    name, operator, version = dependency(requirement)
    offers = [(pkg["name"], pkg["version"])]
    for provide in pkg["provides"]:
        pn, po, pv = dependency(provide)
        if po not in (None, "="):
            raise ValueError("Invalid versioned provider")
        offers.append((pn, pv if po else None))
    for offered, available in offers:
        if offered != name:
            continue
        if not operator:
            return True
        if available is None:
            continue
        cmp = _alpm.alpm_pkg_vercmp(available.encode(), version.encode())
        if {"=": cmp == 0, ">=": cmp >= 0, "<=": cmp <= 0,
            ">": cmp > 0, "<": cmp < 0}[operator]:
            return True
    return False


def database(root, repositories):
    installed = []
    for entry in sorted((root / "local").glob("*/desc")):
        installed.append(metadata(entry.read_text()))
    available = []
    for repo in repositories:
        if not NAME.fullmatch(repo):
            raise ValueError("Invalid repository")
        with tarfile.open(root / "sync" / (repo + ".db")) as db:
            for entry in db:
                if entry.isfile() and entry.name.endswith("/desc"):
                    raw = db.extractfile(entry).read()
                    if raw and not raw.strip(b"\0"):
                        continue
                    try:
                        pkg = metadata(raw.decode())
                    except (ValueError, UnicodeError) as error:
                        raise ValueError(f"{repo}.db:{entry.name}: {error}") from error
                    pkg["repo"] = repo
                    available.append(pkg)
    return installed, available


def resolve(requirements, candidates, installed, available):
    selected = []
    pending = list(requirements) + [d for pkg in candidates for d in pkg["depends"]]
    while pending:
        req = pending.pop(0)
        dependency(req)
        if any(satisfies(pkg, req) for pkg in candidates + installed + selected):
            continue
        name = dependency(req)[0]
        if any(satisfies(pkg, name) for pkg in installed + candidates):
            advice = ("; the kernel package and retained bootloader do not match. "
                      "Report this packaging error; do not select another Mac's GPU profile"
                      if name in ("m1n1", "m1n1-aurora", "m1n1-neo") else "; run the full updater")
            raise ValueError("Installed or matched provider is too old for " + req + advice)
        matches = [pkg for pkg in available if satisfies(pkg, req)]
        # Repository order wins for an exact package name; alternative providers must be unique.
        exact = [pkg for pkg in matches if pkg["name"] == name]
        if exact:
            chosen = exact[0]
        else:
            unique = {pkg["name"] for pkg in matches}
            if len(unique) != 1:
                raise ValueError("No unique missing provider for " + req)
            chosen = matches[0]
        if any(pkg["name"] == chosen["name"] for pkg in installed + candidates + selected):
            raise ValueError("Repository plan would replace an installed or matched package: " + chosen["name"])
        if chosen["replaces"] or chosen["conflicts"]:
            raise ValueError("Missing dependency declares replacements/conflicts: " + chosen["name"])
        selected.append(chosen)
        pending.extend(chosen["depends"])
    return selected


def parse_plan(output):
    result = []
    for line in output.splitlines():
        parts = line.split("\t")
        if len(parts) != 4 or not NAME.fullmatch(parts[0]) or not re.fullmatch(r"[0-9a-f]{64}", parts[2]):
            raise ValueError("Malformed repository transaction plan")
        result.append(dict(zip(("name", "version", "sha256", "location"), parts)))
    if len({p["name"] for p in result}) != len(result):
        raise ValueError("Duplicate repository transaction package")
    return result


def local_snapshot(dbpath):
    return {str(p.relative_to(dbpath / "local")): sha(p)
            for p in sorted((dbpath / "local").glob("*/desc"))}


def verify_plan(path, expected_hash=None):
    if expected_hash is not None and sha(path) != expected_hash:
        raise ValueError("Admitted transaction receipt changed")
    receipt = json.loads(Path(path).read_text())
    if local_snapshot(Path(receipt["dbpath"])) != receipt["database_sha256"]:
        raise ValueError("Installed package database changed; rerun dependency admission")
    files = dict(receipt["candidate_sha256"])
    files.update({p["file"]: p["sha256"] for p in receipt["dependencies"]})
    files.update({p["signature_file"]: p["signature_sha256"] for p in receipt["dependencies"]
                  if "signature_file" in p})
    files[receipt["transaction_config"]] = receipt["transaction_config_sha256"]
    files[receipt["helper"]] = receipt["helper_sha256"]
    for filename, expected in files.items():
        if sha(filename) != expected:
            raise ValueError("Admitted transaction input changed: " + filename)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="/etc/pacman.conf")
    parser.add_argument("--work")
    parser.add_argument("--verify-plan")
    parser.add_argument("--plan-sha256")
    parser.add_argument("--candidate", action="append", default=[])
    parser.add_argument("--require", action="append", default=[])
    parser.add_argument("--allow-remove", action="append", default=[])
    args = parser.parse_args()
    if args.verify_plan:
        verify_plan(args.verify_plan, args.plan_sha256)
        return
    if not args.work:
        raise ValueError("Work directory is required")
    work = Path(args.work).resolve()
    if not work.is_dir() or not args.candidate:
        raise ValueError("Work directory and verified candidate archives are required")
    initial_hashes = {str(Path(p).resolve()): sha(p) for p in args.candidate}
    candidates = [archive(Path(p)) for p in args.candidate]
    if len({p["name"] for p in candidates}) != len(candidates):
        raise ValueError("Duplicate local candidate")
    full = run(["pacman-conf", "--config", args.config]).stdout
    lines = full.splitlines()
    if not lines or lines[0] != "[options]":
        raise ValueError("Cannot resolve pacman configuration")
    end = next((i for i, line in enumerate(lines[1:], 1) if line.startswith("[")), len(lines))
    options = lines[:end]
    dbpath = run(["pacman-conf", "--config", args.config, "DBPath"]).stdout.strip()
    repos = run(["pacman-conf", "--config", args.config, "--repo-list"]).stdout.splitlines()
    dbpath = Path(dbpath)
    initial_database = local_snapshot(dbpath)
    installed, available = database(dbpath, repos)
    candidate_names = {p["name"] for p in candidates}
    allowed_removals = set(args.allow_remove)
    if not all(NAME.fullmatch(name) for name in allowed_removals):
        raise ValueError("Invalid authorized removal name")
    removed = set()
    for current in installed:
        if current["name"] in candidate_names:
            continue
        affected = any(satisfies(current, d) for p in candidates
                       for d in p["conflicts"] + p["replaces"])
        affected |= any(satisfies(p, d) for p in candidates for d in current["conflicts"])
        if affected:
            removed.add(current["name"])
        if affected and current["name"] not in allowed_removals:
            raise ValueError("Local candidates conflict with another installed package: " + current["name"])
    remaining = [p for p in installed
                 if p["name"] not in candidate_names | removed]
    selected = resolve(args.require, candidates, remaining, available)
    for current in remaining:
        if any(
                satisfies(p, d) for p in selected for d in current["conflicts"]):
            raise ValueError("Installed package conflicts with a missing dependency: " + current["name"])
    staging = Path(tempfile.mkdtemp(prefix=".dependencies-", dir=work))
    # Only the private download cache and log change; holds, trust and repository order remain intact.
    private_options = [line for line in options if not line.startswith(("CacheDir = ", "LogFile = "))]
    private_options += ["CacheDir = " + str(staging), "LogFile = " + str(staging / "pacman.log")]
    download_config = staging / "download.conf"
    download_config.write_text("\n".join(private_options + lines[end:]) + "\n")
    commit_config = staging / "transaction.conf"
    hookdir = staging / "hooks"
    hookdir.mkdir()
    helper = Path(__file__).resolve()
    receipt_path = work / "dependency-plan.json"
    if any(c.isspace() for c in str(helper) + str(receipt_path)):
        raise ValueError("Helper and work paths must not contain whitespace")
    hook = hookdir / "00-aurora-frozen-dependencies.hook"

    commit_config.write_text("\n".join(private_options + ["HookDir = " + str(hookdir)]) + "\n")
    # The local candidates join only the final transaction. pacman still resolves the repository
    # targets' dependencies, with the candidates' exact names and provisions assumed installed.
    assumed = []
    for pkg in candidates:
        for provision in [pkg["name"] + "=" + pkg["version"]] + pkg["provides"]:
            assumed += ["--assume-installed", provision]
    base = ["pacman", "--config", str(download_config), "--noconfirm", *assumed]
    plan = []
    if selected:
        targets = [p["repo"] + "/" + p["name"] for p in selected]
        plan = parse_plan(run(base + ["-Sp", "--print-format", FORMAT, *targets]).stdout)
        expected = {(p["name"], p["version"]) for p in selected}
        if {(p["name"], p["version"]) for p in plan} != expected:
            raise ValueError("Repository resolver changed the missing-only plan")
        if any(p["name"] in {x["name"] for x in installed + candidates} for p in plan):
            raise ValueError("Repository resolver would change an installed package")
        run(base + ["-Sw", *targets])
    verified = []
    for record in plan:
        matches = [p for p in staging.iterdir() if p.is_file() and sha(p) == record["sha256"]]
        if len(matches) != 1:
            raise ValueError("Downloaded dependency hash does not match resolver: " + record["name"])
        path = matches[0]
        pkg = archive(path)
        if (pkg["name"], pkg["version"]) != (record["name"], record["version"]):
            raise ValueError("Downloaded dependency metadata does not match resolver")
        selected_pkg = next(p for p in selected if p["name"] == pkg["name"])
        if any(pkg[k] != selected_pkg[k] for k in ("arch", "depends", "provides", "conflicts", "replaces")):
            raise ValueError("Downloaded dependency declarations differ from repository")
        signature = Path(str(path) + ".sig")
        if signature.exists():
            record["signature_sha256"] = sha(signature)
        verified.append((path, record))
    # With no repositories, this final check cannot discover extra upgrades or dependencies.
    paths = args.candidate + [str(path) for path, _ in verified]
    check = run(["pacman", "--config", str(commit_config), "-Up", "--noconfirm",
                 "--ask", "4", "--print-format", "%n\t%v", *paths]).stdout
    final = [tuple(line.split("\t")) for line in check.splitlines()]
    expected = {(p["name"], p["version"]) for p in candidates + selected}
    if len(final) != len(expected) or set(final) != expected:
        raise ValueError("Local transaction differs from admitted packages")
    if initial_database != local_snapshot(dbpath):
        raise ValueError("Installed package database changed during dependency admission")
    if initial_hashes != {str(Path(p).resolve()): sha(p) for p in args.candidate}:
        raise ValueError("Local candidate changed during dependency admission")
    # Publish dependency files only once the complete offline transaction is admitted.
    published = []
    for path, record in verified:
        target = work / path.name
        if target.exists() or Path(str(target) + ".sig").exists():
            raise ValueError("Dependency output already exists")
        published.append((path, target, record))
    for path, target, record in published:
        path.rename(target)
        record["file"] = str(target)
        if "signature_sha256" in record:
            signature_target = Path(str(target) + ".sig")
            Path(str(path) + ".sig").rename(signature_target)
            record["signature_file"] = str(signature_target)
    receipt = {"dependencies": plan, "transaction_config": str(commit_config),
               "transaction_config_sha256": sha(commit_config),
               "hook": str(hook), "helper": str(helper), "helper_sha256": sha(helper),
               "dbpath": str(dbpath), "database_sha256": initial_database,
               "candidate_sha256": initial_hashes,
               "installed": {p["name"]: p["version"] for p in installed}}
    (work / "dependency-plan.json").write_text(json.dumps(receipt, indent=2) + "\n")
    hook.write_text("[Trigger]\nOperation = Install\nOperation = Upgrade\nOperation = Remove\n"
                    "Type = Package\nTarget = *\n[Action]\nWhen = PreTransaction\n"
                    "Exec = /usr/bin/python3 " + str(helper) + " --verify-plan " + str(receipt_path) +
                    " --plan-sha256 " + sha(receipt_path) + "\nAbortOnFail\n")
    print(json.dumps(receipt))


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, tarfile.TarError, KeyError, TypeError) as error:
        print("Frozen dependency admission refused: " + str(error), file=sys.stderr)
        sys.exit(1)
FROZEN_DEPENDENCIES_PY
  say "Resolving only missing dependencies while preserving this image's package holds"
  $sudo python3 "$work/frozen-dependencies.py" "${args[@]}" >"$work/dependency-result.json" ||
    die "could not admit a missing-only dependency transaction. Nothing was installed. The package holds were preserved."
  FROZEN_TRANSACTION_CONFIG=$(python3 - "$work/dependency-result.json" <<'FROZEN_CONFIG_PY'
import json, sys
print(json.load(open(sys.argv[1]))['transaction_config'])
FROZEN_CONFIG_PY
  )
  $sudo test -f "$FROZEN_TRANSACTION_CONFIG" || die "missing admitted package transaction configuration"
  python3 - "$work/dependency-result.json" >"$work/dependency-files" <<'FROZEN_FILES_PY'
import json, sys
plan = json.load(open(sys.argv[1]))
for filename in list(plan['candidate_sha256']) + [p['file'] for p in plan['dependencies']]:
    if '\n' in filename: raise SystemExit('invalid dependency archive path')
    print(filename)
FROZEN_FILES_PY
  mapfile -t FROZEN_TRANSACTION_FILES <"$work/dependency-files"
  ((${#FROZEN_TRANSACTION_FILES[@]})) || die "missing admitted package archives"
}

package_database_check() {
  local pending errors status=0 row name
  local -a unheld=()
  say "Refreshing the package database"
  $sudo pacman -Sy --noconfirm || die "could not refresh the package database. Nothing was installed. Fix the mirror/network error and retry."
  errors=$(mktemp)
  pending=$(LC_ALL=C pacman -Qu --color never 2>"$errors") || status=$?
  # pacman returns 1 for an empty query as well as for a database error.
  if [[ -s $errors ]] || ((status > 1)) || { ((status == 1)) && [[ -n $pending ]]; }; then
    cat "$errors" >&2
    rm -f "$errors"
    die "could not check pending package upgrades. Nothing was installed. Fix pacman's error and retry."
  fi
  rm -f "$errors"
  while IFS= read -r row; do
    [[ -n $row ]] || continue
    name=${row%% *}
    # The release replaces its pins; Mac image packages keep their deliberate hold.
    if [[ $row == *" [ignored]" ]]; then
      if [[ ${1:-} == install ]] && ((FROZEN_PACKAGES)); then continue; fi
      if [[ " $PINNED " == *" $name "* ]]; then continue; fi
      case $name in omarchy|omarchy-mac|omarchy-mac-boot|omarchy-settings) continue ;; esac
    fi
    unheld+=("$name")
  done <<<"$pending"
  ((${#unheld[@]} == 0)) || die "this Mac has ${#unheld[@]} package upgrade(s) pending (pacman -Qu; first: ${unheld[*]:0:8}).
    Run 'omarchy update', reboot, then retry. Installing repository dependencies before a full update would be a partial upgrade. Nothing was installed."
}

install_all() {
  local entry file sha kernel chain rc
  local -a entries candidate_archives=()
  release_source
  require_supported_soc
  if ((ESP_ARCHIVE_HISTORY)); then
    ((!READ_ONLY)) || die "--archive-esp-history changes the EFI partition and cannot be combined with --read-only"
    esp_history_run archive
  fi
  neo_gpu_plan
  version_notice
  sep_write_notice
  ane_dkms_notice
  kernel=$(current_kernel)
  chain=$(boot_chain)
  if [[ $chain == grub ]]; then boot_space "$kernel"; fi
  desktop_fixes_prepare
  m1n1_rollback_check
  m3_plan
  m3_gpu_plan
  if ((M3_GPU_PERSISTENT)); then m3_gpu_check_plan; fi
  m3_pro_mesa_plan
  m3_pro_perf_plan
  frozen_package_detection
  if ((M3_GPU_PERSISTENT)); then
    [[ $M3_PRO_MESA_RESULT != current ]] || M3_PRO_MESA_RESULT=""
    [[ -z $M3_PRO_MESA_RESULT ]] || die "persistent GPU needs the matched Mesa package"
    if ((!FROZEN_PACKAGES)); then
      [[ -z $(m3_pro_mesa_needs_unmet) ]] || die "persistent GPU requires satisfied Mesa dependencies before installation"
    fi
  fi
  m1n1_keep_plan
  # Repository dependencies must resolve against an up-to-date system.
  # Install held release candidates together; update other packages first.
  package_database_check install
  if ((!DESKTOP_FIXES)); then work=$(mktemp -d); fi
  trap 'm3_install_cleanup' EXIT
  if is_neo && ! m1n1_for_this_mac; then say "Keeping this MacBook Neo's own m1n1 (m1n1-aurora has no T8140 support)"; fi
  if [[ $M3_MODE == kernel ]]; then say "Keeping this M3's own m1n1 and boot.bin"; fi
  mapfile -t entries < <(packages_for_this_mac; m3_gpu_files; m3_pro_mesa_files; neo_gpu_files; m3_pro_perf_files)
  for entry in "${entries[@]}"; do
    read -r file sha <<<"$entry"
    say "Downloading $file"
    fetch_release_file "$RELEASE_URL/$file" "$work/$file" && rc=0 || rc=$?
    if ((rc != 0)); then
      [[ $RELEASE_URL == "$PUBLIC_RELEASE_URL" ]] ||
        die "could not download $file from the staging/mirror copy that
    AURORA_RELEASE_URL names (shown at the start). That copy is missing the file
    or can't be reached: check it, or unset AURORA_RELEASE_URL to install from
    the public release. Nothing was installed."
      ((rc != 101)) ||
        die "could not download $file from $TAG: the server refused the request.
    See curl's HTTP error above. This does not establish that the release asset is missing.
    Nothing was installed. Check access to GitHub or the intervening proxy; if this persists,
    report the file name and HTTP status."
      ((rc == 22)) ||
        die "could not download $file from $TAG: the download failed on all five tries
    (curl exit $rc; exit 100 means the server kept answering with an error such as HTTP 503).
    That points at the network or a busy server, not at the release or this Mac.
    Nothing was installed. Wait a minute, then run the same command again; it is safe to repeat."
      die "could not download $file from $TAG.
    The release is missing a file this script expects, which is a packaging
    mistake rather than anything wrong with this Mac. Nothing was installed.
    Please report it with the file name above."
    fi
    [[ $(sha256sum "$work/$file" | cut -d' ' -f1) == "$sha" ]] || die "$file does not match its published checksum"
    if [[ $file == *.pkg.tar.zst ]]; then candidate_archives+=("$work/$file"); fi
  done
  neo_gpu_package_check
  desktop_fixes_verify "$work"
  while read -r file sha; do
    [[ -n $file ]] || continue
    candidate_archives+=("$work/$file")
  done < <(desktop_fixes_files)
  if ((M3_GPU_PERSISTENT)); then m3_persistent_package_check "$work/${M3_PRO_MESA_PACKAGE%% *}"; fi
  m3_pro_perf_verify
  m3_pro_mesa_set_aside
  frozen_dependency_prepare "${candidate_archives[@]}"
  if m1n1_for_this_mac; then
    sha=$(m1n1_pkg_sha "$work/${M1N1_PACKAGE%% *}")
    [[ $sha == "$M1N1_BIN_SHA" ]] ||
      die "${M1N1_PACKAGE%% *} holds an m1n1.bin with sha256 ${sha:-(none)}, not the
    $M1N1_BIN_SHA this release names. Nothing was installed. Please report it at
    https://github.com/omacom/linux-aurora/issues"
  fi
  if [[ $M3_MODE == handoff ]]; then
    m1n1_pkg_has_handoff "$work/${M1N1_PACKAGE%% *}" ||
      die "this release's m1n1 has no $(m3_handoff_name), so it can't switch it
    on. Nothing was installed. Please report it at https://github.com/omacom/linux-aurora/issues"
  fi

  m3_gpu_disarm || die "air-gpu-oneshot.sh --disarm failed, so a boot may still be armed for the
    kernel this install replaces. Nothing was installed. Run: sudo air-gpu-oneshot.sh --disarm"
  snapshot "aurora-sep $VERSION"
  # Before pacman's update-m1n1 hook rebuilds boot.bin below.
  bootbin_backup
  $sudo install -d "$STATE"
  snapshot_boot_state
  neo_gpu_transaction_begin
  if [[ ! -f $STATE/previous && $chain == grub ]]; then
    keep_grub_fallback "$kernel"
  fi
  if ((M3_GPU_PERSISTENT)); then
    m3_persistent_keep_entry
    m3_persistent_transaction_begin "$chain"
  fi
  # The M3 Max test kit keeps the kernel this Mac runs as a boot menu entry before it changes.
  if ((M3MAX_KIT)); then m3_keep_limine_entry retain; fi
  # The kernel this Mac had before the first install; an update keeps it.
  if [[ ! -f $STATE/previous-package ]]; then
    pacman -Q "$kernel" | $sudo tee "$STATE/previous-package" >/dev/null
  fi
  if [[ $M3_MODE == handoff ]]; then
    echo "$M3_MODE $(m3_variant)" | $sudo tee "$STATE/m3-mode" >/dev/null
  elif [[ $M3_MODE != none ]]; then
    echo "$M3_MODE" | $sudo tee "$STATE/m3-mode" >/dev/null
  fi
  # Before pacman's update-m1n1 hook runs on the kernel's device trees.
  case $M3_MODE in
    kernel)
      M3_BOOTBIN_SHA=$(m3_bootbin_sha)
      m3_freeze
      ;;
    # So the hook's rebuild, where it runs, already carries the switches.
    handoff) m3_switches_write ;;
    # update-m1n1 is frozen (m1n1_keep_plan): prove boot.bin stays as it is.
    *) if ((M1N1_KEEP)); then M3_BOOTBIN_SHA=$(m3_bootbin_sha); fi ;;
  esac

  say "Installing the aurora-sep kernel, libfprint with the Apple SEP driver, fprintd and aurora-touchid"
  # --ask 4 accepts replacing linux-asahi (and its headers), which linux-aurora conflicts with.
  m3_install_packages
  if ((!FROZEN_PACKAGES)); then $sudo pacman -S --needed --noconfirm fprintd; fi
  # linux-aurora carries the Apple video decoder, whose firmware linux-asahi
  # installs never needed; without it the decoder fails to load at boot.
  if ((FROZEN_PACKAGES)); then
    pacman -Q avd-fw >/dev/null 2>&1 ||
      warn "avd-fw is absent; hardware video decode remains unavailable while this image is frozen"
  else
    $sudo pacman -S --needed --noconfirm avd-fw ||
      warn "could not install avd-fw; hardware video decode will not work until it is installed"
  fi
  # The VA-API bridge to that decoder. Without it, players fall back to
  # software decode with no error (reported on a 16" M1 Pro installed from the
  # Omarchy Mac ISO). Leave any other build of the bridge alone: the AUR
  # libva-v4l2_request packages conflict with it.
  if ! pacman -Qq libva-v4l2_request-avd libva-v4l2_request >/dev/null 2>&1 &&
    [[ ! -e /usr/lib/dri/v4l2_request_drv_video.so ]]; then
    if ((FROZEN_PACKAGES)); then
      warn "the VA-API bridge is absent; video players will decode in software while this image is frozen"
    else
      $sudo pacman -S --needed --noconfirm libva-v4l2_request-avd ||
        warn "could not install libva-v4l2_request-avd; video players will decode in software until it is installed"
    fi
  fi
  m3_gpu_install
  add_pin
  # Both boot chains boot through m1n1, and both need the aurora device trees
  # in boot.bin: update-m1n1 otherwise takes the DTBs of the highest-versioned
  # kernel directory, which on a Mac with an older hand-installed test kernel
  # is not this one -- and those DTBs lack the Touch ID sensor node. pacman's
  # own hook already ran update-m1n1 during the install above, before this
  # configuration existed, so run it again now that it is in place.
  # A kernel-only M3 keeps the boot.bin it has. On the handoff path, the
  # freezes kept the hook from rebuilding boot.bin before the new m1n1 was in
  # place; lift them only now (see m3_plan).
  case $M3_MODE in
    kernel) m3_bootbin_report ;;
    handoff)
      # grub.cfg first, so a failed check below never leaves it pointing at
      # the replaced kernel.
      if [[ $chain == grub ]]; then grub_update; fi
      m3gpu_unfreeze
      m3_unfreeze
      if update_m1n1_frozen; then
        die "update-m1n1 is still frozen in $UPDATE_M1N1_CONF, so boot.bin can't be rebuilt with
    the handoff. boot.bin was not changed. Please report it with that file."
      fi
      m1n1_update
      m3_verify_bootbin
      m1n1_check_and_record
      # The test reboot may be a hard reset (m3-serial.py reboot): get the new
      # boot.bin onto the EFI partition first.
      sync
      ;;
    *)
      if ((M1N1_KEEP)); then
        m1n1_keep_report
      else
        m1n1_update
        # Not where update-m1n1 is frozen (m1n1_update said so) or builds from
        # an m1n1 of its own, as a Neo without NEO_AURORA_M1N1 does.
        if m1n1_for_this_mac && ! update_m1n1_frozen && ! update_m1n1_own_m1n1; then
          m1n1_check_and_record
        fi
      fi
      ;;
  esac
  if [[ $chain == grub && $M3_MODE != handoff ]]; then
    grub_update
  fi

  calibration
  $sudo systemctl daemon-reload
  sep_policy
  if is_neo; then neo_radio_notice; fi
  m3_pro_perf_activate
  m3_pro_mesa_install
  m3_pro_mesa_render
  m3_pro_mesa_record
  neo_gpu_activate
  pacman -Q linux-aurora libfprint aurora-touchid
  if ((M3_GPU_PERSISTENT)); then m3_gpu_check_install; fi
  if ((M3_GPU_PERSISTENT)); then m3_persistent_transaction_commit; fi
  echo
  if ((M3_GPU_PERSISTENT)); then
    say "Done. The matched kernel, Mesa and bootloader are installed with experimental GPU
    profile $M3_GPU_PROFILE selected for subsequent boots. The retained previous entry stays
    in the boot menu with GPU start disabled and Mesa off. Touch ID is not supported on M3 yet."
  elif [[ $M3_MODE == handoff ]] && is_m3_max; then
    say "Done. Reboot: expect the boot menu, then the same desktop on the boot framebuffer. m1n1
    now reserves the display processors' log buffers, publishes the M3 Max facts and hands the
    internal display over (its GPU handoff stays off); the kernel's new M3 Max paths stay off.
    Touch ID is not supported on M3 yet."
  elif [[ $M3_MODE == handoff ]] && m3_air_default; then
    say "Done. Reboot: expect the Omarchy logo, the boot menu, then the same desktop on the boot
    framebuffer. m1n1 now hands the built-in display over and describes the GPU firmware for
    Linux; the kernel keeps the boot framebuffer until it can drive the display, and nothing
    starts the GPU. Touch ID is not supported on M3 yet."
    m3_air_off_notice
  elif [[ $M3_MODE == handoff ]] && is_m3_air && [[ $M3_AIR_DISPLAY_HANDOFF == 1 ]]; then
    m3_air_off_notice quiet
    say "Done. Reboot with the serial recorder running and someone watching: expect the Omarchy
    logo, the boot menu, then the same desktop on the boot framebuffer. Check the serial log
    for the T8122 handoff result and the \"PMP: T8122:\" lines.
    Native display and GPU acceleration are not enabled. Touch ID is not supported on M3 yet.
    What to send back: case D of step 11 of the test plan that the --agent-prompt command below
    prints."
  elif [[ $M3_MODE == handoff ]] && is_m3_air && [[ $M3_AIR_DRY_RUN == 1 ]]; then
    say "Done. Reboot with the serial recorder running and someone watching: expect the Omarchy
    logo, the boot menu, then the desktop on the boot framebuffer, as before. This build only
    reads and reports, so nothing on the desktop changes. Touch ID is not supported on M3 yet.
    What to send back: case D of step 11 of the test plan that the --agent-prompt command below
    prints."
  elif [[ $M3_MODE == handoff ]] && is_m3_air && [[ $M3_AIR_DCP != 1 ]]; then
    say "Done. Reboot with someone watching: expect the Omarchy logo, the boot menu, then the
    desktop on the boot framebuffer, as before. The GPU handoff is for testing: the desktop
    still renders in software. Touch ID is not supported on M3 yet. What to check and
    report: case D of step 11 of the test plan that the --agent-prompt command below prints."
  elif [[ $M3_MODE == handoff ]]; then
    say "Done. Reboot with someone watching: expect the Omarchy logo, the boot menu, then the
    desktop on the built-in display at its native resolution. Touch ID is not supported
    on M3 yet."
  elif [[ $M3_MODE == kernel ]] && ((M1N1_KEEP)); then
    say "Done. Reboot: this Mac keeps the boot loader it has now, with this release's kernel.
    Touch ID is not supported on M3 yet."
  elif [[ $M3_MODE == kernel ]]; then
    say "Done. Reboot: expect the desktop on the boot framebuffer. Touch ID is not supported
    on M3 yet. How to help bring this M3 further: step 11 of the test plan that the
    --agent-prompt command below prints."
  elif [[ -f $MODPROBE_CONF ]]; then
    say "Done, read-only. Reboot; the SEP driver will attach and report without writing."
    echo "   Enrolling a finger needs writes: delete $MODPROBE_CONF, reboot, then run aurora-touchid-setup."
  else
    say "Done. Reboot, then run:  aurora-touchid-setup"
  fi
  if ((M1N1_KEEP)) && ! is_m3; then
    echo "   This Mac keeps the boot loader it has: the m1n1 that failed on it is not put back."
  fi
  m3_gpu_notice
  m3_pro_mesa_notice
  desktop_fixes_notice
  echo "   Testing this build? The plan and reporting format:"
  echo "      curl -fsSL $PUBLIC_RELEASE_URL/install-aurora-sep.sh | bash -s -- --agent-prompt"
  if [[ $chain == grub ]]; then
    echo "   The previous kernel stays in the GRUB menu as 'Previous kernel … before aurora-sep'."
  fi
  echo "   To undo the kernel install: curl -fsSL $PUBLIC_RELEASE_URL/install-aurora-sep.sh | bash -s -- --uninstall"
  m3_next_steps
  # The kernel install is complete either way; 3 or 4: the M3 Pro's Mesa or its record.
  m3_pro_mesa_status || exit $?
}

uninstall_all() {
  local previous=linux-asahi m3_mode=none neo_restore=0 max_variant=0 max_m1n1=0
  if is_neo && [[ -f $STATE/neo-before.json ]]; then neo_restore=1; fi
  require_supported_soc "Uninstalling, which rebuilds boot.bin with the stock m1n1,"
  # An M3 Max on its boot loader variant gets the boot.bin it booted before the variant back,
  # checked by its recorded sha256 before anything changes (m3max_bootbin_restore).
  if is_m3_max && [[ $(m3_recorded_mode) == handoff ]]; then
    max_variant=1
    m3max_bootbin_restore check ||
      die "this M3 Max has m1n1's boot loader variant, and the boot.bin it booted before it can't be
    put back (see above). Nothing was uninstalled."
  fi
  # Before anything changes: a boot left armed for the kernel being removed would fail Limine's
  # hash check at the next boot.
  m3_gpu_disarm || die "air-gpu-oneshot.sh --disarm failed, so a boot may still be armed for the
    kernel --uninstall would remove. Nothing was uninstalled. Run: sudo air-gpu-oneshot.sh --disarm"
  # The M3 Max test kit stops for good (its results stay in $M3MAX_KIT_STATE).
  m3max_kit_uninstall
  if [[ -f $STATE/previous-package ]]; then read -r previous _ <"$STATE/previous-package" || true; fi
  # 11.36 rewrote this on every run, so an updated Mac may name linux-aurora
  # itself, which the repositories don't carry for these Macs.
  if [[ -z $previous || $previous == linux-aurora ]]; then previous="linux-asahi"; fi
  # A kernel-only install can retain a previously installed m1n1-aurora.
  # Without a mode record, older supported handoff installs used that package.
  m3_mode=$(m3_recorded_mode)
  if is_m3; then
    if pacman -Q m1n1-aurora >/dev/null 2>&1; then
      if [[ $m3_mode == kernel ]] || is_m3_kernel_only_chip; then
        m3_mode=kernel
      else
        m3_mode=handoff
      fi
    elif [[ $m3_mode != handoff ]] || is_m3_kernel_only_chip; then
      m3_mode=kernel
    fi
  else
    m3_mode=none
  fi
  # An M3 Max keeps the m1n1 it has; after the variant (or the kit's --restore) that is this
  # release's T6031 build, so its freeze stays at the end.
  if is_m3_max && m3max_m1n1_installed; then max_m1n1=1; fi
  if [[ $(boot_chain) == limine ]] && m3_limine_cleanup_needed; then
    m3_limine_remove_check
  fi
  package_database_check
  command -v aurora-touchid-setup >/dev/null && aurora-touchid-setup --remove || true
  $sudo systemctl disable apple-sep.path apple-sep.service 2>/dev/null || true
  remove_pin
  snapshot "removing aurora-sep"
  local m1n1=m1n1
  # A Neo keeps its own m1n1, unless it got m1n1-aurora (NEO_AURORA_M1N1):
  # then the stock m1n1 package goes back, and its own M1N1= with the saved
  # update-m1n1 configuration below.
  if is_neo && { [[ $NEO_AURORA_M1N1 != 1 ]] || ! pacman -Q m1n1-aurora >/dev/null 2>&1; }; then
    m1n1=
    say "Reinstalling $previous and the stock libfprint; this MacBook Neo keeps its own m1n1"
  elif ((max_variant)); then
    m1n1=
    say "Reinstalling $previous and the stock libfprint; this M3 Max gets back the boot.bin it booted
    before m1n1's boot loader variant"
  elif [[ $m3_mode == kernel ]]; then
    m1n1=
    say "Reinstalling $previous and the stock libfprint; this M3 keeps its m1n1 and boot.bin as they are"
  else
    say "Reinstalling $previous, the stock m1n1 and the stock libfprint"
  fi
  $sudo pacman -Rdd --noconfirm aurora-touchid 2>/dev/null || true
  # The stock m1n1 has no M3 handoff; drop the switches before its rebuild.
  m3_switches_remove
  # The M3 Max's earlier boot.bin, frozen as a kernel-only M3's is, so the kernel's hook below
  # leaves it as it is; the freeze comes off at the end, as on any kernel-only M3.
  if ((max_variant)); then
    m3max_bootbin_restore
    m3max_conf_kernel_only
  fi
  $sudo pacman -S --noconfirm --ask 4 "$previous" "$previous-headers" libfprint $m1n1
  # Restore the stock update-m1n1 configuration on either chain before the
  # rebuild below, so boot.bin goes back to the packaged m1n1 and DTBs.
  # A kernel-only M3's boot.bin and update-m1n1 configuration were never
  # changed; only this script's freeze comes off, at the end.
  if [[ $(boot_chain) != grub && $m3_mode != kernel ]] && ((!neo_restore)); then
    if [[ -f $STATE/update-m1n1.default.saved ]]; then
      $sudo cp "$STATE/update-m1n1.default.saved" "$UPDATE_M1N1_CONF"
    else
      $sudo rm -f "$UPDATE_M1N1_CONF"
    fi
    $sudo update-m1n1 || warn "update-m1n1 failed; boot.bin still has the aurora m1n1"
  fi

  if [[ $(boot_chain) == grub ]]; then
    [[ -f $STATE/linux-asahi.preset.saved && ! -f /etc/mkinitcpio.d/linux-asahi.preset ]] &&
      $sudo mv "$STATE/linux-asahi.preset.saved" /etc/mkinitcpio.d/linux-asahi.preset
    [[ -f $STATE/grub.default.saved ]] && $sudo cp "$STATE/grub.default.saved" /etc/default/grub
    m3_persistent_remove
    if [[ $m3_mode != kernel ]] && ((!neo_restore)); then
      if [[ -f $STATE/update-m1n1.default.saved ]]; then
        $sudo cp "$STATE/update-m1n1.default.saved" "$UPDATE_M1N1_CONF"
      else
        $sudo rm -f "$UPDATE_M1N1_CONF"
      fi
      $sudo update-m1n1
    fi
    $sudo systemctl disable aurora-sep-fallback-modules.service 2>/dev/null || true
    $sudo rm -f /etc/systemd/system/aurora-sep-fallback-modules.service
    $sudo rm -f /etc/grub.d/42_aurora_sep_previous /boot/vmlinuz-aurora-sep-previous /boot/initramfs-aurora-sep-previous.img
    $sudo grub-mkconfig -o /boot/grub/grub.cfg
  fi
  case $m3_mode in
    kernel)
      if ((max_m1n1)); then
        say "update-m1n1 stays frozen ($UPDATE_M1N1_CONF): the installed m1n1 is this release's M3 Max
    build, not the one this Mac had, and a boot.bin rebuilt from it would run its GPU handoff at
    every boot. boot.bin stays as it is. Reinstall the m1n1 package this Mac had before you remove
    the lines from \"$M3_FREEZE_BEGIN\" to \"$M3_FREEZE_END\"."
      else
        m3_unfreeze
      fi
      ;;
    handoff) m3_restore_bringup ;;
  esac
  $sudo rm -f "$MODPROBE_CONF"
  # Keep recovery state and modules if the checked entry cannot be removed.
  if [[ $(boot_chain) == limine ]] && m3_limine_cleanup_needed; then
    m3_limine_remove_check remove
    $sudo systemctl disable aurora-sep-fallback-modules.service 2>/dev/null || true
    $sudo rm -f /etc/systemd/system/aurora-sep-fallback-modules.service
  fi
  m3_persistent_remove
  m3_gpu_remove
  m3_pro_mesa_remove
  m3_gpu_check_remove
  if ((neo_restore)); then
    neo_gpu_restore "$STATE/neo-before.json"
    $sudo update-m1n1 || {
      neo_gpu_restore "$STATE/neo-before.json"
      warn "stock boot.bin rebuild failed; this Neo's original boot.bin was restored"
    }
    say "Restored this Neo's original stage 2 and configuration; native Neo GPU selection removed"
  fi
  # The m1n1 builds that failed on this Mac stay recorded, so a later install
  # never puts one of them back.
  local failed=""
  if [[ -f $STATE/m1n1-failed ]]; then failed=$(cat "$STATE/m1n1-failed"); fi
  $sudo rm -rf "$STATE"
  if [[ -n $failed ]]; then
    $sudo install -d "$STATE"
    printf '%s\n' "$failed" | $sudo tee "$STATE/m1n1-failed" >/dev/null
    say "Kept $STATE/m1n1-failed: a later install never puts back an m1n1 that failed on this Mac"
  fi
  say "Done. Reboot to run $previous."
}

# --reset-touchid: start Touch ID over on this Mac. For a Mac whose stored
# identity keybag no longer loads, such as an M2 Pro/Max enrolled on system
# firmware 26.2 or earlier and then updated. Moves the SEP driver's state and
# fprintd's prints for the Apple sensor aside; the next boot creates a new
# keybag, and every finger is enrolled again. The old keybag stays in the
# enclave unused: there is no way to delete it.
#
# The keybag is last, so a reset that stops partway leaves it in place and
# the next boot does not create a second one next to the old Catacombs.
TOUCHID_STATE_FILES=(
  /var/lib/aurora-sep-host-state.bin
  /var/lib/apple-sep-catacomb-master.bin
  /var/lib/apple-sep-catacomb-owner.bin
  /var/lib/apple-sep-catacomb-user.bin
  /var/lib/aurora-sep-refkey.bin
  /var/lib/aurora-sep-refkey-v2.bin
  /var/lib/aurora-sep-keybag.bin
)

sep_diag() {
  local f
  for f in /sys/bus/platform/devices/*.sep/diag/"$1"; do
    [[ -r $f ]] && { cat "$f"; return; }
  done
  echo none
}

reset_touchid() {
  local yes=0 force=0 arg f d keybag found=0
  for arg in "$@"; do
    case $arg in
      --yes) yes=1 ;;
      --force) force=1 ;;
      *) die "unknown option $arg for --reset-touchid (--yes, --force)" ;;
    esac
  done
  grep -qa 'apple,' /proc/device-tree/compatible 2>/dev/null || die "this is not an Apple Silicon Mac"
  [[ -f $MODPROBE_CONF ]] &&
    die "this Mac was installed read-only ($MODPROBE_CONF): a new keybag needs enclave writes. Delete that file first if you mean to allow them."
  for f in "${TOUCHID_STATE_FILES[@]}"; do [[ -e $f ]] && found=1; done
  ((found)) || die "no Touch ID state on this Mac; nothing to reset"

  # Ask for sudo now, in the foreground: a password prompt under timeout
  # below could not read the terminal. Not sudo -v, which wants a password
  # unless every rule for the user is NOPASSWD (a wheel member's is not).
  if [[ -n $sudo ]]; then $sudo true || die "--reset-touchid needs sudo"; fi

  # Only a keybag the driver could not load is worth replacing. At boot the
  # driver reports keybag=present for any keybag file it finds; it loads the
  # keybag, and reports failed if that does not work, only when the sensor is
  # first opened. Until then touchid=unknown, so open it through fprintd once.
  keybag=$(sep_diag keybag)
  [[ $keybag == none ]] &&
    die "the Touch ID driver is not running (no SEP diagnostics). Boot the aurora kernel first."
  if [[ $(sep_diag touchid) == unknown ]]; then
    timeout --foreground 20 $sudo fprintd-list root >/dev/null 2>&1 || true
    keybag=$(sep_diag keybag)
  fi
  if [[ $keybag != failed ]]; then
    ((force)) || die "the Touch ID keybag is not broken (driver reports keybag=$keybag, touchid=$(sep_diag touchid)).
    A reset would throw away a working keybag. If Touch ID still fails and you mean it, run again with --reset-touchid --force."
    warn "resetting although the driver reports keybag=$keybag (--force)"
  fi

  warn "this removes every enrolled fingerprint on this Mac and creates a new Touch ID keybag at the next boot.
    Anything sealed with the Secure Enclave (kernel trusted keys) can no longer be unsealed.
    The old keybag stays in the Secure Enclave, unused; it cannot be deleted, and this cannot be undone once the Mac has rebooted.
    If Touch ID has failed only this once, reboot and try it again first: a passing error looks the same."
  if ((!yes)); then
    # Piped from curl, stdin is the script; ask on the terminal, if there is one.
    { : </dev/tty; } 2>/dev/null || die "no terminal to confirm on; run again with --reset-touchid --yes"
    local answer=""
    printf 'Type RESET to continue: ' >/dev/tty
    read -r answer </dev/tty || true
    [[ $answer == RESET ]] || die "not reset"
  fi

  # Until the reboot nothing may touch the sensor: a match or enrolment would
  # write new state behind the move. The runtime mask goes away at boot. A
  # verify already running finishes its save within about a second.
  $sudo systemctl mask --runtime --now fprintd.service
  sleep 3

  # Outside /var/lib/aurora-sep, which --uninstall removes. Each move is
  # recorded so that a failure puts everything back as it was.
  local dest src to moved=() sources=()
  dest=/var/lib/aurora-sep-touchid-reset-$(date +%Y%m%d-%H%M%S)
  # fprintd's directories are root-only, so look for them as root.
  while IFS= read -r d; do sources+=("$d"); done < <(
    $sudo find /var/lib/fprint -mindepth 2 -maxdepth 2 -type d -name apple-sep 2>/dev/null)
  for f in "${TOUCHID_STATE_FILES[@]}"; do [[ -e $f ]] && sources+=("$f"); done
  for src in "${sources[@]}"; do
    case $src in
      /var/lib/fprint/*) to=$dest/fprint/$(basename "$(dirname "$src")")/apple-sep ;;
      *) to=$dest/$(basename "$src") ;;
    esac
    if ! { $sudo mkdir -p "$(dirname "$to")" && $sudo mv "$src" "$to"; }; then
      for d in "${moved[@]}"; do $sudo mv "${d#*|}" "${d%%|*}" || warn "could not put back ${d%%|*} from ${d#*|}"; done
      $sudo systemctl unmask --runtime fprintd.service || true
      die "could not move $src; put back everything already moved, so Touch ID is as it was"
    fi
    moved+=("$src|$to")
  done
  say "Moved the Touch ID state to $dest"
  say "Reboot now. Touch ID creates its new keybag during boot; then enrol your fingers again."
}

# The m1n1 in boot.bin by its bytes, for --m3-report: two builds can report the
# same stage 2 version.
m3_report_m1n1() {
  local target size="" installed=""
  if [[ -f $M1N1_BIN ]]; then
    size=$(stat -c %s "$M1N1_BIN")
    installed=$(sha256sum "$M1N1_BIN" | cut -d' ' -f1)
  fi
  echo "installed m1n1.bin: sha256 ${installed:--} (${size:--} bytes)"
  if [[ -n $size ]] && target=$(esp_bootbin); then
    echo "boot.bin: sha256 $($sudo sha256sum "$target" | cut -d' ' -f1)"
    echo "boot.bin's first $size bytes: sha256 $(bootbin_m1n1_sha "$target" "$size")"
  else
    echo "boot.bin: -"
  fi
  printf 'm1n1-installed: '; cat "$STATE/m1n1-installed" 2>/dev/null || echo -
}

# ---- the M3 bring-up report (--m3-report) ------------------------------------------------------
# --m3-report: one file with what bringing up an M3 needs, in the current directory, to attach to
# an issue. It works on every M3 (t8122, t6030, t6031, t6034) and only reads: who this Mac is, the
# boot loader's /chosen entries, every device-tree node, which devices got a driver, the whole
# kernel log of this boot, the SMC's temperature and power keys, the CPUs, and the state of each
# kind of device a bring-up checks. The host name, user names, serial numbers and MAC addresses
# are masked (m3_privacy_mask), and the finished file is checked for them (m3_privacy_check):
# a file that still has any is not kept.
M3_REPORT_DMESG='asahi|agx|gpu|g15|dcp|dart|t8122|t6030|t6031|t6034|reserved|iommu|mailbox|pmp|simpledrm|m1n1|tipd|typec|sn201202|atc|usb|xhci|dwc3|thermal|macsmc'
# The M3 chips whose /chosen/asahi,<chip>-* entries (the boot loader's facts and switches) the
# report copies whole.
M3_CHOSEN_SOCS="t8122 t6030 t6031 t6034"
# Where the report and the power survey read the running system; tests point them at a fake Mac.
M3_SYSFS=/sys
M3_PROCFS=/proc
M3_DEBUGFS=/sys/kernel/debug
M3_ETC=/etc
M3_MODULES=/usr/lib/modules
M3_DEVFS=/dev
# The report's (or survey's) work directory, removed on exit.
M3_WORK=""
# The ADT reader (tools/aurora-sep/aurora-adt-extract.py), as "file sha256": a release asset that
# --m3-report downloads and checks like M3_GPU_SCRIPTS, unless the script runs from a directory
# that has its own copy (a checkout). It reads the boot loader's copy of the ADT through the
# read-only node of the phram MTD device named adt and prints an allowlist of it;
# "--check <node>" checks the node and what it is bound to, and reads nothing. Empty: a
# release with none, and the report says so.
M3_ADT_READER="aurora-adt-extract.py cfef5935f35c412e006ae70229406b0eac51e830a175ca9b98fcdabe873389f5"

# A file's first line, or "-" when it can't be read. No fork: the report reads a few thousand.
m3_attr() {
  local v=""
  { IFS= read -r v || [[ -n $v ]]; } 2>/dev/null <"$1" || { printf -- '-'; return 0; }
  printf '%s' "$v"
}

# A device-tree property's strings, separated by spaces, or "-".
m3_dt_words() {
  local -a w=()
  { mapfile -d '' -t w; } 2>/dev/null <"$1" || true
  if ((${#w[@]})); then printf '%s' "${w[*]}"; else printf -- '-'; fi
}

# A device-tree cell (big-endian 32 bits) as a number, or "-".
m3_dt_u32() {
  local v
  v=$(od -An -tu4 --endian=big -N4 "$1" 2>/dev/null | tr -d ' ') || v=""
  printf '%s' "${v:--}"
}

# Who this Mac is and what it runs: appended to system.txt after the lines earlier releases wrote.
m3_report_identity() {
  local f
  echo "model: $(m3_dt_words "$DT/model")"
  echo "compatible: $(m3_dt_words "$DT/compatible")"
  echo "memory: $(awk '/^MemTotal:/ { printf "%.1f GiB (MemTotal %s kB)", $2 / 1048576, $2 }' "$M3_PROCFS/meminfo" 2>/dev/null)"
  echo "uname: $(uname -srvm)"
  echo "cmdline: $(m3_attr "$M3_PROCFS/cmdline")"
  for f in "$DT"/chosen/framebuffer*; do
    [[ -d $f ]] || continue
    echo "boot framebuffer ${f##*/}: $(m3_dt_u32 "$f/width")x$(m3_dt_u32 "$f/height"), stride $(m3_dt_u32 "$f/stride"), format $(m3_dt_words "$f/format")"
  done
  echo "packages:"
  pacman -Q 2>/dev/null | grep -E '^(linux-(aurora|asahi)|m1n1|uboot-asahi|asahi-|mesa|vulkan-|libfprint|fprintd|aurora-|limine|omarchy-mac|alsa-ucm|speakersafetyd|tiny-dfr|libva|avd-fw|linux-firmware)' ||
    echo -
}

# A device-tree reg property as "address+size" pairs in hex, with the parent's cell counts.
m3_dt_reg() { # FILE ADDRESS-CELLS SIZE-CELLS
  local ac=$2 sc=$3
  [[ $ac =~ ^[0-9]+$ ]] || ac=2
  [[ $sc =~ ^[0-9]+$ ]] || sc=2
  [[ -f $1 ]] || { printf -- '-'; return 0; }
  od -An -tx4 --endian=big -v "$1" 2>/dev/null |
    awk -v ac="$ac" -v sc="$sc" '
      { for (i = 1; i <= NF; i++) w[n++] = $i }
      END {
        for (i = 0; i + ac + sc <= n; i += ac + sc) {
          a = ""; s = ""
          for (j = 0; j < ac; j++) a = a w[i + j]
          for (j = 0; j < sc; j++) s = s w[i + ac + j]
          sub(/^0+/, "", a); sub(/^0+/, "", s)
          printf "%s0x%s+0x%s", (i ? " " : ""), (a == "" ? "0" : a), (s == "" ? "0" : s)
        }
      }'
}

# The reserved-memory nodes: each one's name, compatible, label, reg and status (the addresses,
# never what is in them), and whether the boot loader's ADT and log nodes are there.
m3_report_reserved() {
  local rm=$DT/reserved-memory d ac sc label adt="" log=""
  ac=$(m3_dt_u32 "$rm/#address-cells") sc=$(m3_dt_u32 "$rm/#size-cells")
  echo "# $rm: node, compatible, label, reg (address+size), status, no-map"
  for d in "$rm"/*/; do
    d=${d%/}
    [[ -d $d ]] || continue
    label=$(m3_dt_words "$d/label")
    printf '%s\tcompatible=%s\tlabel=%s\treg=%s\tstatus=%s\tno-map=%s\n' "${d##*/}" "$(m3_dt_words "$d/compatible")" \
      "$label" "$(m3_dt_reg "$d/reg" "$ac" "$sc")" "$(m3_dt_words "$d/status")" "$([[ -e $d/no-map ]] && echo yes || echo no)"
    [[ $label == adt ]] && adt=${d##*/}
    [[ $label == m1n1_stage2.log ]] && log=${d##*/}
  done
  echo "adt node (the boot loader's copy of the ADT): ${adt:-absent}"
  echo "m1n1_stage2.log node (m1n1's log of this boot): ${log:-absent}"
}

# A device-tree property as text (strings joined by " | "), or its size when it isn't text.
m3_dt_text() {
  local v
  v=$(tr '\0' '\n' <"$1" 2>/dev/null | sed '/^$/d' | paste -sd'|' | sed 's/|/ | /g')
  if [[ -n $v ]] && ! LC_ALL=C grep -q '[^[:print:]]' <<<"$v"; then
    printf '%s' "$v"
  else
    printf '<%s bytes>' "$(stat -c %s "$1" 2>/dev/null || echo ?)"
  fi
}

# The macOS system-firmware stub's version, from the installer's stub_info.json.
m3_report_stub_version() {
  local bootbin v=""
  if bootbin=$(esp_bootbin); then
    v=$($sudo cat "${bootbin%/m1n1/boot.bin}/asahi/stub_info.json" 2>/dev/null |
      grep -o '"ProductVersion": *"[^"]*"' | head -1 | sed 's/.*"\([^"]*\)"$/\1/') || v=""
  fi
  echo "${v:--}"
}

# Which device tree this Mac boots: the running tree's model and compatible, the boot loader's
# asahi,* entries in /chosen, and the device trees in m1n1's boot.bin, each by its first board
# and chip compatible; this board's is matched by its bytes against the installed kernels'.
m3_report_boot_dt() {
  local board f target bin=$M3_WORK/boot.bin off size ver j t n=0 sha match="" dtbs
  board=$(this_board)
  echo "running device tree: model $(m3_dt_words "$DT/model"); compatible $(m3_dt_words "$DT/compatible")"
  echo "system-firmware stub (stub_info.json ProductVersion): $(m3_report_stub_version)"
  echo "== $DT/chosen asahi,* entries"
  for f in "$DT"/chosen/asahi,*; do
    [[ -e $f ]] || continue
    if [[ -d $f ]]; then echo "${f##*/}/ (a node)"; else echo "${f##*/}: $(m3_dt_text "$f")"; fi
  done
  echo "== the device trees in m1n1's boot.bin"
  if ! target=$(esp_bootbin) || ! $sudo cat "$target" >"$bin" 2>/dev/null; then
    echo "boot.bin: not found or not readable"
    return 0
  fi
  while read -r off; do
    # A device tree's header: the magic, its total size, ..., and version 17 at byte 20.
    size=$(od -An -tu4 --endian=big -j "$((off + 4))" -N4 "$bin" 2>/dev/null | tr -d ' ')
    ver=$(od -An -tu4 --endian=big -j "$((off + 20))" -N4 "$bin" 2>/dev/null | tr -d ' ')
    if ! [[ $size =~ ^[0-9]+$ && $ver == 17 ]] || ((size < 64 || size > 4194304)); then continue; fi
    tail -c +"$((off + 1))" "$bin" | head -c "$size" >"$M3_WORK/dtb.$n"
    j=$(LC_ALL=C grep -aoE 'apple,j[0-9a-z]+' "$M3_WORK/dtb.$n" | head -1)
    t=$(LC_ALL=C grep -aoE 'apple,t[0-9]{4}' "$M3_WORK/dtb.$n" | head -1)
    echo "dtb $n at byte $off, $size bytes: ${j:--} ${t:--}"
    if [[ -n $board && ${j#apple,} == "$board" ]]; then
      sha=$(sha256sum <"$M3_WORK/dtb.$n" | cut -d' ' -f1)
      dtbs=$(for f in "$M3_MODULES"/*/dtbs/*-"$board".dtb "$M3_MODULES"/*/dtbs/*/*-"$board".dtb; do
        [[ -f $f && $(sha256sum <"$f" | cut -d' ' -f1) == "$sha" ]] && echo "${f#"$M3_MODULES"/}"
      done | paste -sd' ')
      match+="dtb $n (sha256 $sha) is ${dtbs:-none of the device trees of the installed kernels}"$'\n'
    fi
    n=$((n + 1))
  # Only the offsets: grep -o also prints the matched bytes, which are not
  # valid UTF-8, and bash 5.2's read returns nothing for them in a UTF-8 locale.
  done < <(LC_ALL=C grep -obUaP '\xd0\x0d\xfe\xed' "$bin" 2>/dev/null | cut -d: -f1)
  echo "device trees in boot.bin: $n"
  if [[ -n $match ]]; then printf '%s' "$match"; else echo "no device tree in boot.bin names this board (${board:-?})"; fi
}

# The ADT reader to run. With M3_ADT_READER set, only bytes with its sha256: the copy next to
# this script when it has them, else this release's, downloaded and checked. Without it (a
# checkout before a release names one), the copy next to this script. Prints its path; or,
# returning 1, why there is none.
m3_adt_reader() {
  local self=${BASH_SOURCE[0]:-} local_copy="" file sha got
  if [[ -n $self && -f $self && -f $(dirname "$self")/aurora-adt-extract.py ]]; then
    local_copy=$(dirname "$self")/aurora-adt-extract.py
  fi
  if [[ -z $M3_ADT_READER ]]; then
    if [[ -n $local_copy ]]; then echo "$local_copy"; return 0; fi
    echo "this release ($TAG) has no ADT reader, so the ADT was not read"
    return 1
  fi
  read -r file sha <<<"$M3_ADT_READER"
  if [[ -n $local_copy && $(sha256sum "$local_copy" | cut -d' ' -f1) == "$sha" ]]; then
    echo "$local_copy"
    return 0
  fi
  if ! (release_source >/dev/null 2>&1); then
    echo "AURORA_RELEASE_URL or AURORA_RELEASES_API is not a URL this takes, so the ADT was not read"
    return 1
  fi
  if ! curl -fsSL --retry 3 -o "$M3_WORK/$file" "$RELEASE_URL/$file" 2>/dev/null; then
    echo "could not download $file from $TAG, so the ADT was not read"
    return 1
  fi
  got=$(sha256sum "$M3_WORK/$file" | cut -d' ' -f1)
  if [[ $got != "$sha" ]]; then
    echo "$file does not match its published checksum, so the ADT was not read"
    return 1
  fi
  echo "$M3_WORK/$file"
}

# The largest adt region the report reads (an ADT is well under 1 MiB).
M3_ADT_MAX_BYTES=$((16 * 1024 * 1024))
# The on-demand ADT read's state, for the traps: whether this run loaded phram, and the reader.
M3_PHRAM_LOADED=0
M3_ADT_PID=""

# Whether phram is loaded (or built in).
m3_phram_loaded() {
  [[ -d $M3_SYSFS/module/phram ]] || grep -q '^phram ' "$M3_PROCFS/modules" 2>/dev/null
}

# The MTD inventory and phram's state, as compared before and after the ADT read: one line per
# MTD device (not its read-only twin) with its number, name, type, size, erase size, device
# number and /dev/mtd/by-name link.
m3_mtd_inventory() {
  local m l by devfs
  devfs=$(readlink -f "$M3_DEVFS")
  if m3_phram_loaded; then echo "phram: loaded"; else echo "phram: not loaded"; fi
  for m in "$M3_SYSFS"/class/mtd/mtd*; do
    [[ ${m##*/} =~ ^mtd[0-9]+$ ]] || continue
    by=-
    for l in "$M3_DEVFS"/mtd/by-name/*; do
      [[ -L $l && $(readlink -f "$l") == "$devfs/${m##*/}" ]] && by=${l##*/}
    done
    printf '%s name=%s type=%s size=%s erasesize=%s dev=%s by-name=%s\n' "${m##*/}" "$(m3_attr "$m/name")" \
      "$(m3_attr "$m/type")" "$(m3_attr "$m/size")" "$(m3_attr "$m/erasesize")" "$(m3_attr "$m/dev")" "$by"
  done | sort -V
  return 0
}

# The device names in an m3_mtd_inventory, one per line, sorted.
m3_mtd_names() {
  sed -n 's/^mtd[0-9]* name=\([^ ]*\) .*/\1/p' | LC_ALL=C sort
}

# The reserved-memory node phram makes the adt MTD device from: label adt, compatible phram.
m3_adt_dt_node() {
  local d
  for d in "$DT"/reserved-memory/*/; do
    d=${d%/}
    [[ $(m3_dt_words "$d/label") == adt && " $(m3_dt_words "$d/compatible") " == *" phram "* ]] &&
      { echo "$d"; return 0; }
  done
  return 0
}

# The device-tree node an MTD device (mtdN) was made from: its own of_node, or its device's.
m3_mtd_of_node() {
  local m=$M3_SYSFS/class/mtd/$1
  if [[ -e $m/of_node ]]; then readlink -f "$m/of_node"; else readlink -f "$m/device/of_node"; fi
}

# The MTD device (mtdN) named NAME, or nothing.
m3_mtd_named() {
  local m
  for m in "$M3_SYSFS"/class/mtd/mtd*; do
    [[ ${m##*/} =~ ^mtd[0-9]+$ && $(m3_attr "$m/name") == "$1" ]] && { echo "${m##*/}"; return 0; }
  done
  return 0
}

# Waits (up to 10 s) until udev has handled every event, such as its own probe of a new MTD
# node, which holds the node open for a moment.
m3_udev_settle() {
  if command -v udevadm >/dev/null; then udevadm settle --timeout=10 2>/dev/null || true; fi
}

# Unloads phram if this run loaded it: after the reader has stopped and udev has settled, and
# once more after a short wait if phram is still in use. Never forced. Returns 1 when phram
# stays loaded; M3_PHRAM_LOADED is then 0 all the same, since the owner was told.
m3_adt_unload() {
  local try
  ((M3_PHRAM_LOADED)) || return 0
  if [[ -n $M3_ADT_PID ]]; then
    kill "$M3_ADT_PID" 2>/dev/null || true
    wait "$M3_ADT_PID" 2>/dev/null || true
    M3_ADT_PID=""
  fi
  for try in 1 2; do
    m3_udev_settle
    if $sudo modprobe -r phram 2>/dev/null; then
      M3_PHRAM_LOADED=0
      return 0
    fi
    ((try == 2)) || sleep 2
  done
  M3_PHRAM_LOADED=0
  warn "phram, which this report loaded, is still loaded (modprobe -r phram failed: in use). Unload it later with: sudo modprobe -r phram"
  return 1
}

# The report's traps: the reader stopped, phram unloaded if this run loaded it, the work
# directory removed.
m3_report_cleanup() {
  m3_adt_unload || true
  if [[ -n $M3_PARTIAL ]]; then rm -f "$M3_PARTIAL"; fi
  if [[ -n $M3_WORK ]]; then rm -rf "$M3_WORK"; fi
  M3_WORK=""
}

# The on-demand ADT read, on an M3. adt-allowlist.txt gets the reader's output, or one line
# saying why there is none; adt-check.txt records each step, and the MTD inventory and phram's
# state before and after, which must match. phram is loaded only when it isn't (an owner's
# phram is left alone, and the step stops), only when the device tree has the adt region, and
# only when no MTD device but nvram is present; it is unloaded on every path (the traps cover
# an interruption). The reader gets only the read-only node /dev/mtdNro of the device named adt,
# once its size and device-tree node match the region's and the reader's own --check agrees.
# Once the ADT was read, and while phram is still loaded, m1n1's log of this boot (the
# m1n1_stage2.log region, the other device phram makes) is read the same way, read-only, into
# m1n1-stage2-log.txt (m3_report_stage2_log); a step that stopped reads no log either.
m3_report_adt() { # DIR
  local out=$1/adt-allowlist.txt log=$1/adt-check.txt reader node want reg mtd dev i rc=0 why="" before after
  local loaded new
  is_m3 || return 0
  before=$(m3_mtd_inventory)
  printf '== before\n%s\n' "$before" >"$log"
  if ! reader=$(m3_adt_reader); then
    why=$reader
  elif ! command -v python3 >/dev/null; then
    why="python3 is not installed, so the ADT was not read"
  elif m3_phram_loaded; then
    why="phram is already loaded on this Mac, not by this report, so the ADT step stopped and phram stays loaded"
  elif grep -v '^phram:' <<<"$before" | grep -vq ' name=nvram '; then
    why="an MTD device other than nvram is present, so the ADT step stopped (see adt-check.txt)"
  elif ! node=$(m3_adt_dt_node) || [[ -z $node ]]; then
    why="no adt region in the device tree: this boot's m1n1 reserved none, so the ADT was not read"
  fi
  if [[ -z $why ]]; then
    reg=$(m3_dt_reg "$node/reg" "$(m3_dt_u32 "$DT/reserved-memory/#address-cells")" "$(m3_dt_u32 "$DT/reserved-memory/#size-cells")")
    if [[ $reg =~ ^0x[0-9a-f]+\+0x([0-9a-f]{1,15})$ ]]; then
      want=$((16#${BASH_REMATCH[1]}))
      echo "region: ${node#"$DT"} reg $reg ($want bytes)" >>"$log"
      if ((want == 0 || want > M3_ADT_MAX_BYTES)); then
        why="the adt region is $want bytes, not 1 to $M3_ADT_MAX_BYTES, so the ADT was not read"
      fi
    else
      why="the adt region's reg ($reg) is not one address and size, so the ADT was not read"
    fi
  fi
  if [[ -z $why ]]; then
    say "Reading the boot loader's copy of the ADT: phram is loaded for it, and unloaded again"
    if ! $sudo modprobe phram 2>>"$log"; then
      why="no phram module on this kernel (it comes with 12.3), so the ADT was not read"
    else
      M3_PHRAM_LOADED=1
      m3_udev_settle
      loaded=$(m3_mtd_inventory)
      printf 'loaded: phram\n== with phram\n%s\n' "$loaded" >>"$log"
      # Exactly the two regions' devices are new: adt and m1n1_stage2.log.
      new=$(diff <(m3_mtd_names <<<"$before") <(m3_mtd_names <<<"$loaded") | sed -n 's/^> //p' | paste -sd' ') || true
      mtd=$(m3_mtd_named adt)
      if [[ -z $new ]]; then
        why="phram made no MTD device, so the ADT was not read"
      elif [[ $new != "adt m1n1_stage2.log" ]]; then
        why="phram made the MTD devices $new, not exactly adt and m1n1_stage2.log, so the ADT was not read"
      elif [[ -z $mtd ]]; then
        why="phram made no MTD device named adt, so the ADT was not read"
      elif [[ $(m3_attr "$M3_SYSFS/class/mtd/$mtd/size") != "$want" ||
        $(m3_mtd_of_node "$mtd") != "$(readlink -f "$node")" ]]; then
        why="the adt MTD device ($mtd, $(m3_attr "$M3_SYSFS/class/mtd/$mtd/size") bytes) does not match its reserved-memory region (${node##*/}, $want bytes), so the ADT was not read"
      else
        dev=$M3_DEVFS/${mtd}ro
        for ((i = 0; i < 10; i++)); do [[ -e $dev ]] && break; sleep 0.5; done
        if [[ ! -e $dev ]]; then
          why="no read-only device node $dev, so the ADT was not read"
        elif ! $sudo python3 "$reader" --check "$dev" >>"$log" 2>"$M3_WORK/adt.err"; then
          why="the ADT reader's check refused $dev, so the ADT was not read: $(tail -1 "$M3_WORK/adt.err" 2>/dev/null | cut -c1-200 || true)"
        else
          echo "read: $dev with ${reader##*/}" >>"$log"
          $sudo python3 "$reader" "$dev" >"$out" 2>"$M3_WORK/adt.err" &
          M3_ADT_PID=$!
          wait "$M3_ADT_PID" || rc=$?
          M3_ADT_PID=""
          if ((rc)); then
            why="the ADT reader failed (exit $rc), so its output was left out: $(tail -1 "$M3_WORK/adt.err" 2>/dev/null | cut -c1-200 || true)"
          else
            # Only after the ADT was read: phram made exactly the two regions' devices, and the adt
            # device matched its region, so the log device is its region's too.
            m3_report_stage2_log "$1" "$reader" "$log" || true
          fi
        fi
      fi
    fi
  fi
  if [[ ! -e $1/m1n1-stage2-log.txt ]]; then
    echo "m1n1's log was not read: ${why:-the ADT step did not read it}" >"$1/m1n1-stage2-log.txt"
  fi
  if [[ -n $why ]]; then echo "$why" >"$out"; echo "stopped: $why" >>"$log"; else echo "read: done" >>"$log"; fi
  local unload=ok
  if ((M3_PHRAM_LOADED)); then
    if m3_adt_unload; then
      echo "unloaded: phram" >>"$log"
    else
      unload=failed
      echo "unload FAILED: phram stays loaded (in use after udev settled, twice; not forced)" >>"$log"
    fi
  fi
  after=$(m3_mtd_inventory)
  printf '== after\n%s\n' "$after" >>"$log"
  if [[ $after == "$before" ]]; then
    echo "restored: yes (the MTD inventory and phram's state match the before-state)" >>"$log"
  else
    echo "restored: NO (the MTD inventory or phram's state differs from the before-state)" >>"$log"
    # A failed unload has said so already, in one line.
    [[ $unload == failed ]] || warn "the ADT step left the MTD devices or phram not as it found them: see adt-check.txt in the report"
  fi
  return 0
}

# m1n1's log of this boot, while phram is loaded (m3_report_adt): the read-only node of the MTD
# device named m1n1_stage2.log, read by the ADT reader's --stage2-log, which checks that it is
# that region's and prints it as text with the lines that name serial numbers, UUIDs and the like
# left out. Its output, or one line why there is none, goes to DIR/m1n1-stage2-log.txt; each step
# goes to LOG. Returns 1 when there is none.
m3_report_stage2_log() { # DIR READER LOG
  local out=$1/m1n1-stage2-log.txt reader=$2 log=$3 mtd dev i rc=0
  mtd=$(m3_mtd_named m1n1_stage2.log)
  if [[ -z $mtd ]]; then
    echo "m1n1's log was not read: phram made no MTD device named m1n1_stage2.log" >"$out"
    echo "log: no m1n1_stage2.log device" >>"$log"
    return 1
  fi
  dev=$M3_DEVFS/${mtd}ro
  for ((i = 0; i < 10; i++)); do [[ -e $dev ]] && break; sleep 0.5; done
  if [[ ! -e $dev ]]; then
    echo "m1n1's log was not read: no read-only device node $dev" >"$out"
    echo "log: no $dev" >>"$log"
    return 1
  fi
  echo "log: $dev with ${reader##*/} --stage2-log" >>"$log"
  $sudo python3 "$reader" --stage2-log "$dev" >"$out" 2>"$M3_WORK/log.err" &
  M3_ADT_PID=$!
  wait "$M3_ADT_PID" || rc=$?
  M3_ADT_PID=""
  if ((rc)); then
    echo "m1n1's log was not read: the reader refused or failed (exit $rc): $(tail -1 "$M3_WORK/log.err" 2>/dev/null | cut -c1-200 || true)" >"$out"
    echo "log: failed (exit $rc)" >>"$log"
    return 1
  fi
  echo "log: done" >>"$log"
  # And all 16 KiB of it, byte for byte but the same lines overwritten (a reader without --raw
  # leaves this file out).
  rc=0
  $sudo python3 "$reader" --stage2-log --raw "$dev" >"$1/m1n1-stage2-log.raw" 2>>"$M3_WORK/log.err" &
  M3_ADT_PID=$!
  wait "$M3_ADT_PID" || rc=$?
  M3_ADT_PID=""
  if ((rc)); then
    rm -f "$1/m1n1-stage2-log.raw"
    echo "log raw: failed (exit $rc)" >>"$log"
  else
    echo "log raw: done" >>"$log"
  fi
}

# Every device-tree node, one per line: its path, compatible and status.
m3_report_dt_nodes() {
  local d rel
  echo "# every node of $DT: path, compatible, status (- when the node has none)"
  while IFS= read -r -d '' d; do
    rel=${d#"$DT"}
    printf '%s\tcompatible=%s\tstatus=%s\n' "/${rel#/}" "$(m3_dt_words "$d/compatible")" "$(m3_dt_words "$d/status")"
  done < <(find -H "$DT" -type d -print0 2>/dev/null | LC_ALL=C sort -z)
}

# Which devices got a driver and which didn't, the enabled SoC nodes no device was made for, the
# probes still deferred, the power domains and the loaded modules.
m3_report_drivers() {
  local dev bus drv node dt d c s tsv=$M3_WORK/devices.tsv
  dt=$(readlink -f "$DT")
  for dev in "$M3_SYSFS"/bus/*/devices/*; do
    [[ -e $dev/of_node ]] || continue
    bus=${dev#"$M3_SYSFS"/bus/}
    bus=${bus%%/*}
    drv=-
    if [[ -e $dev/driver ]]; then drv=$(basename "$(readlink -f "$dev/driver")"); fi
    node=$(readlink -f "$dev/of_node")
    node=${node#"$dt"}
    printf '%s\t%s\t%s\t/%s\n' "$bus" "${dev##*/}" "$drv" "${node#/}"
  done | LC_ALL=C sort >"$tsv"
  echo "== devices with a device-tree node and a driver: bus, device, driver, node"
  awk -F'\t' '$3 != "-"' "$tsv"
  echo "== devices with a device-tree node and no driver: bus, device, node"
  awk -F'\t' '$3 == "-" { print $1 "\t" $2 "\t" $4 }' "$tsv"
  echo "== enabled SoC nodes with a compatible and no device"
  for d in "$DT"/soc/*/; do
    d=${d%/}
    [[ -f $d/compatible ]] || continue
    s=$(m3_dt_words "$d/status")
    [[ $s == - || $s == okay || $s == ok ]] || continue
    c=/${d#"$DT"/}
    awk -F'\t' -v n="$c" '$4 == n { found = 1 } END { exit !found }' "$tsv" ||
      printf '%s\tcompatible=%s\n' "$c" "$(m3_dt_words "$d/compatible")"
  done
  echo "== deferred probes ($M3_DEBUGFS/devices_deferred)"
  $sudo cat "$M3_DEBUGFS/devices_deferred" 2>/dev/null || echo "(not readable)"
  echo "== power domains ($M3_DEBUGFS/pm_genpd/pm_genpd_summary)"
  $sudo cat "$M3_DEBUGFS/pm_genpd/pm_genpd_summary" 2>/dev/null || echo "(not readable)"
  echo "== loaded modules ($M3_PROCFS/modules)"
  cat "$M3_PROCFS/modules" 2>/dev/null || echo "(not readable)"
}

# Kernel log lines without the ones naming a serial number, and MAC addresses masked.
m3_report_scrub() {
  { LC_ALL=C grep -aviE 'serialnumber|serial number|serial-number|serial_number' || true; } |
    LC_ALL=C sed -E 's/([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2}/xx:xx:xx:xx:xx:xx/g'
}

# This boot's whole kernel log into OUT: from the journal when it has the start of the boot
# (as this user, then through sudo), else the kernel's buffer (DMESG, read already). Prints the
# source: journal or dmesg.
m3_report_klog() { # DMESG OUT
  local j=$M3_WORK/journal.raw
  journalctl -k -b 0 -o short-monotonic --no-pager >"$j" 2>/dev/null || true
  if ! LC_ALL=C grep -qaE 'Linux version|Kernel command line:' "$j" && [[ -n $sudo ]]; then
    $sudo journalctl -k -b 0 -o short-monotonic --no-pager >"$j" 2>/dev/null || true
  fi
  if LC_ALL=C grep -qaE 'Linux version|Kernel command line:' "$j"; then
    m3_report_scrub <"$j" >"$2"
    echo journal
  else
    m3_report_scrub <"$1" >"$2"
    echo dmesg
  fi
}

# The CPUs: each one's capacity, cluster and core ID, the cpufreq policies, cpuidle, cpuinfo.
m3_report_cpus() {
  local c p f
  echo "== CPUs: cpu, online, capacity, cluster, package, core, cluster CPUs, MIDR"
  for c in "$M3_SYSFS"/devices/system/cpu/cpu[0-9]*; do
    printf '%s online=%s capacity=%s cluster=%s package=%s core=%s cluster_cpus=%s midr=%s\n' "${c##*/}" \
      "$(m3_attr "$c/online")" "$(m3_attr "$c/cpu_capacity")" "$(m3_attr "$c/topology/cluster_id")" \
      "$(m3_attr "$c/topology/physical_package_id")" "$(m3_attr "$c/topology/core_id")" \
      "$(m3_attr "$c/topology/cluster_cpus_list")" "$(m3_attr "$c/regs/identification/midr_el1")"
  done | sort -V
  echo "== cpufreq policies"
  for p in "$M3_SYSFS"/devices/system/cpu/cpufreq/policy*; do
    [[ -d $p ]] || continue
    echo "${p##*/}:"
    for f in affected_cpus related_cpus scaling_driver scaling_governor cpuinfo_min_freq cpuinfo_max_freq \
      cpuinfo_transition_latency scaling_min_freq scaling_max_freq scaling_cur_freq \
      scaling_available_frequencies scaling_available_governors; do
      [[ -e $p/$f ]] && echo "  $f: $(m3_attr "$p/$f")"
    done
  done
  echo "== cpuidle: driver $(m3_attr "$M3_SYSFS/devices/system/cpu/cpuidle/current_driver"), governor $(m3_attr "$M3_SYSFS/devices/system/cpu/cpuidle/current_governor")"
  for f in "$M3_SYSFS"/devices/system/cpu/cpu0/cpuidle/state*; do
    [[ -d $f ]] && echo "cpu0 ${f##*/}: $(m3_attr "$f/name") latency $(m3_attr "$f/latency") us, residency $(m3_attr "$f/residency") us, disabled $(m3_attr "$f/disable")"
  done
  echo "== $M3_PROCFS/cpuinfo"
  cat "$M3_PROCFS/cpuinfo" 2>/dev/null || echo "(not readable)"
}

# Every readable attribute of a sysfs directory (one level), but its uevent, as "  name: value".
m3_report_attrs() {
  local f
  for f in "$1"/*; do
    [[ -f $f && ${f##*/} != uevent ]] && echo "  ${f##*/}: $(m3_attr "$f")"
  done
  return 0
}

# A sysfs device's driver, or "-".
m3_report_driver_of() {
  if [[ -e $1/driver ]]; then basename "$(readlink -f "$1/driver")"; else echo -; fi
}

# PCI and USB.
m3_report_buses() {
  echo "== lspci -nnk"
  lspci -nnk 2>&1 || echo "(lspci not installed)"
  echo "== lsusb"
  lsusb 2>&1 || echo "(lsusb not installed)"
  echo "== lsusb -t"
  lsusb -t 2>&1 || echo "(lsusb not installed)"
}

# USB-C ports, partners, cables and alternate modes (not a partner's identity).
m3_report_typec() {
  local d
  for d in "$M3_SYSFS"/class/typec/* "$M3_SYSFS"/class/typec/*/*.[0-9]*; do
    [[ -d $d ]] || continue
    echo "== ${d#"$M3_SYSFS"/class/typec/}"
    m3_report_attrs "$d"
  done
  for d in "$M3_SYSFS"/class/usb_power_delivery/*; do
    [[ -d $d ]] && echo "== usb_power_delivery ${d##*/}"
  done
  return 0
}

# DRM cards and connectors with their modes, and the backlight (never a display's EDID).
m3_report_display() {
  local c
  echo "== /dev/dri"
  ls -l "$M3_DEVFS/dri" 2>&1 || true
  for c in "$M3_SYSFS"/class/drm/card*; do
    [[ -d $c ]] || continue
    if [[ ${c##*/} == card+([0-9]) ]]; then
      echo "== ${c##*/}: driver $(m3_report_driver_of "$c/device")"
    else
      echo "== ${c##*/}: status $(m3_attr "$c/status"), enabled $(m3_attr "$c/enabled"), dpms $(m3_attr "$c/dpms")"
      [[ -r $c/modes ]] && echo "  modes: $(tr '\n' ' ' <"$c/modes" 2>/dev/null)"
    fi
  done
  for c in "$M3_SYSFS"/class/backlight/*; do
    [[ -d $c ]] || continue
    echo "== backlight ${c##*/}: driver $(m3_report_driver_of "$c/device")"
    m3_report_attrs "$c"
  done
  return 0
}

# Power supplies (without their serial numbers), thermal zones, cooling devices and hwmon.
m3_report_power() {
  local d f
  for d in "$M3_SYSFS"/class/power_supply/*; do
    [[ -d $d ]] || continue
    echo "== power_supply ${d##*/}"
    { grep -viE 'serial' "$d/uevent" 2>/dev/null || true; } | sed 's/^/  /'
  done
  for d in "$M3_SYSFS"/class/thermal/thermal_zone*; do
    [[ -d $d ]] || continue
    echo "== ${d##*/}: type $(m3_attr "$d/type"), temp $(m3_attr "$d/temp"), mode $(m3_attr "$d/mode"), policy $(m3_attr "$d/policy")"
    for f in "$d"/trip_point_*_type; do
      [[ -e $f ]] && echo "  ${f##*/}: $(m3_attr "$f") at $(m3_attr "${f%_type}_temp")"
    done
  done
  for d in "$M3_SYSFS"/class/thermal/cooling_device*; do
    [[ -d $d ]] && echo "== ${d##*/}: type $(m3_attr "$d/type"), state $(m3_attr "$d/cur_state") of $(m3_attr "$d/max_state")"
  done
  for d in "$M3_SYSFS"/class/hwmon/hwmon*; do
    [[ -d $d ]] || continue
    echo "== ${d##*/}: $(m3_attr "$d/name"), driver $(m3_report_driver_of "$d/device")"
    for f in "$d"/*_input; do
      [[ -e $f ]] || continue
      f=${f##*/}
      echo "  ${f%_input}: $(m3_attr "$d/${f%_input}_label") = $(m3_attr "$d/$f")"
    done
  done
  return 0
}

# Sound cards and input devices (a device's unique ID masked).
m3_report_sound_input() {
  echo "== $M3_PROCFS/asound/cards"
  cat "$M3_PROCFS/asound/cards" 2>/dev/null || echo "(none)"
  echo "== $M3_PROCFS/asound/pcm"
  cat "$M3_PROCFS/asound/pcm" 2>/dev/null || echo "(none)"
  echo "== $M3_PROCFS/bus/input/devices"
  sed -E 's/^(U: Uniq=).+/\1(masked)/' "$M3_PROCFS/bus/input/devices" 2>/dev/null || echo "(not readable)"
}

# Network interfaces and their drivers (never an address), Bluetooth controllers and rfkill.
m3_report_network() {
  local d
  for d in "$M3_SYSFS"/class/net/*; do
    [[ -e $d ]] || continue
    echo "${d##*/}: driver $(m3_report_driver_of "$d/device"), type $(m3_attr "$d/type"), operstate $(m3_attr "$d/operstate"), mtu $(m3_attr "$d/mtu")"
  done
  for d in "$M3_SYSFS"/class/bluetooth/*; do
    [[ -e $d ]] && echo "${d##*/}: driver $(m3_report_driver_of "$d/device")"
  done
  for d in "$M3_SYSFS"/class/rfkill/rfkill*; do
    [[ -e $d ]] && echo "${d##*/}: $(m3_attr "$d/name") $(m3_attr "$d/type") soft $(m3_attr "$d/soft") hard $(m3_attr "$d/hard")"
  done
  return 0
}

# The SMC's key list in debugfs (macsmc-hwmon), or nothing.
m3_smc_keys_file() {
  $sudo find "$M3_DEBUGFS" -maxdepth 3 -name keys -path '*smc*' 2>/dev/null | head -1 || true
}

# Why there is no SMC key list, in one line. This kernel's SMC driver makes the list on every M3
# (macsmc_hwmon_is_m3: t8122, t6030, t6031, t6032 and t6034), so its absence means debugfs or the
# driver, on any of them.
m3_smc_missing() {
  echo "no SMC key list on this kernel (no $M3_DEBUGFS/*smc*/keys: debugfs is not mounted, or the SMC driver did not start)"
}

# The SMC's key list: its header line and every T* (temperature, mC) and P* (power, mW) key.
m3_report_smc() {
  local kf
  kf=$(m3_smc_keys_file)
  if [[ -n $kf ]]; then
    # shellcheck disable=SC2016 # awk's fields
    $sudo awk 'NR == 1 || $2 ~ /^[TP]/' "$kf" 2>/dev/null || echo "(could not read $kf)"
  else
    m3_smc_missing
  fi
}

# The physical memory map: only the System RAM and reserved ranges. The addresses need root.
m3_report_iomem() {
  echo "# $M3_PROCFS/iomem, System RAM and reserved ranges only$([[ $EUID != 0 && -z $sudo ]] && echo " (not root: the kernel shows every address as 0)")"
  $sudo grep -E ': (System RAM|reserved)$' "$M3_PROCFS/iomem" 2>/dev/null || echo "(none, or not readable)"
}

# What the report holds, for whoever opens it.
m3_report_readme() {
  cat <<EOF
aurora-sep --m3-report ($TAG), $(date -u +%Y-%m-%dT%H:%M:%SZ)
Read-only. The host name is replaced by "host", user names by "user", serial numbers by
SERIAL and MAC addresses by xx:xx:xx:xx:xx:xx; kernel log lines naming a USB serial are left out.
  system.txt        who this Mac is, its boot loader, kernel, command line and packages
  dt-nodes.txt      every device-tree node: path, compatible, status
  drivers.txt       devices with and without a driver, deferred probes, power domains, modules
  kernel-log.txt    this boot's whole kernel log
  dmesg-m3.txt      the kernel log lines of the M3 bring-up
  smc-keys.txt      the SMC's temperature (T*, mC) and power (P*, mW) keys
  cpu.txt           CPU topology, cpufreq policies, cpuidle, cpuinfo
  buses.txt         lspci -nnk, lsusb, lsusb -t
  typec.txt         USB-C ports, partners and alternate modes
  display.txt       DRM connectors and modes, backlight
  power.txt         power supplies, thermal zones, cooling devices, hwmon
  sound-input.txt   sound cards and input devices
  network.txt       network interfaces and their drivers, Bluetooth, rfkill
  reserved-memory.txt  the reserved-memory nodes (name, compatible, label, reg, status), and
                    whether the boot loader's adt and m1n1_stage2.log nodes are there
  boot-dt.txt       which device tree this Mac boots: model, compatible, the /chosen asahi,*
                    entries, the stub's version, the device trees in boot.bin
  adt-allowlist.txt the ADT reader's allowlist of the boot loader's ADT (on an M3), or why
                    there is none
  adt-check.txt     the ADT read's steps, and the MTD devices and phram's state before and after
  m1n1-stage2-log.txt  m1n1's log of this boot, read with the ADT (lines naming serial numbers,
                    UUIDs other than firmware images' and the like left out), or why there is none
  m1n1-stage2-log.raw  the same log region whole (16 KiB), those lines overwritten with x
  interrupts.txt    $M3_PROCFS/interrupts
  iomem.txt         $M3_PROCFS/iomem, its System RAM and reserved ranges only
  usb-display.txt   USB tree, USB-C roles and DRM connector states (as in earlier reports)
  chosen/           the boot loader's /chosen entries for the M3 chips
EOF
}

# Where m3_report_collect's kernel-log.txt came from: journal or dmesg (only a journal file has
# the host column m3_privacy_mask masks).
M3_REPORT_KLOG_SRC=""

# Collects the report's files into DIR (made by the caller), unmasked, with M3_WORK as the work
# directory: --m3-report packs them (m3_report), and the M3 Max test kit keeps one per stage.
m3_report_collect() { # DIR
  local dir=$1 board soc f s src
  board=$(this_board) soc=$(this_soc)
  {
    echo "board: ${board:-?} soc: ${soc:-?}"
    echo "kernel: $(uname -r)"
    for f in os-fw-version system-fw-version iboot2-version m1n1-stage1-version m1n1-stage2-version; do
      printf '%s: ' "$f"; { tr -d '\0' <"$DT/chosen/asahi,$f"; } 2>/dev/null || printf '-'; echo
    done
    pacman -Q linux-aurora m1n1-aurora m1n1 2>/dev/null || true
    printf 'm3-mode: '; cat "$STATE/m3-mode" 2>/dev/null || echo -
    m3_report_m1n1
    echo "m1n1.conf switches:"; grep '^chosen\.' "$M1N1_CONF" 2>/dev/null || echo -
    printf 'm1n1-oslog-overlap: '; if [[ -e $DT/$M3_OSLOG_OVERLAP ]]; then echo present; else echo absent; fi
    echo "reserved display logs:"; ls -d "$DT"/reserved-memory/dcp-oslog@* 2>/dev/null || echo -
    ( set +e +o pipefail; m3_report_identity )
  } >"$dir/system.txt"
  { dmesg 2>/dev/null || $sudo dmesg; } >"$M3_WORK/dmesg.raw" 2>/dev/null || true
  grep -iE "$M3_REPORT_DMESG" "$M3_WORK/dmesg.raw" | grep -viE 'serialnumber|serial number' |
    sed -E 's/([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}/xx:xx:xx:xx:xx:xx/g' >"$dir/dmesg-m3.txt" || true
  src=$(m3_report_klog "$M3_WORK/dmesg.raw" "$dir/kernel-log.txt")
  mkdir -p "$dir/chosen"
  for s in $M3_CHOSEN_SOCS; do
    for f in "$DT"/chosen/asahi,"$s"-*; do
      [[ -e $f ]] && cp -r "$f" "$dir/chosen/"
    done
  done
  if [[ -e $DT/$M3_OSLOG_OVERLAP ]]; then cp -r "$DT/$M3_OSLOG_OVERLAP" "$dir/chosen/"; fi
  {
    echo "== lsusb -t"; lsusb -t 2>/dev/null || echo "(lsusb not installed)"
    echo "== /sys/class/typec"
    for f in "$M3_SYSFS"/class/typec/port*; do
      [[ -d $f ]] || continue
      echo "$(basename "$f"): data_role=$(cat "$f/data_role" 2>/dev/null) power_role=$(cat "$f/power_role" 2>/dev/null)"
    done
    echo "== /dev/dri"; ls -l "$M3_DEVFS/dri" 2>/dev/null || true
    echo "== drm connectors"
    for f in "$M3_SYSFS"/class/drm/card*-*/status; do [[ -e $f ]] && echo "$f: $(cat "$f")"; done
  } >"$dir/usb-display.txt" 2>&1
  # Each part reads on through anything it can't read; none of them writes.
  ( set +e +o pipefail; m3_report_dt_nodes ) >"$dir/dt-nodes.txt" 2>&1
  ( set +e +o pipefail; m3_report_drivers ) >"$dir/drivers.txt" 2>&1
  ( set +e +o pipefail; m3_report_smc ) >"$dir/smc-keys.txt" 2>&1
  ( set +e +o pipefail; m3_report_cpus ) >"$dir/cpu.txt" 2>&1
  ( set +e +o pipefail; m3_report_buses ) >"$dir/buses.txt" 2>&1
  ( set +e +o pipefail; m3_report_typec ) >"$dir/typec.txt" 2>&1
  ( set +e +o pipefail; m3_report_display ) >"$dir/display.txt" 2>&1
  ( set +e +o pipefail; m3_report_power ) >"$dir/power.txt" 2>&1
  ( set +e +o pipefail; m3_report_sound_input ) >"$dir/sound-input.txt" 2>&1
  ( set +e +o pipefail; m3_report_network ) >"$dir/network.txt" 2>&1
  ( set +e +o pipefail; m3_report_reserved ) >"$dir/reserved-memory.txt" 2>&1
  ( set +e +o pipefail; m3_report_boot_dt ) >"$dir/boot-dt.txt" 2>&1
  # In this shell, not a subshell: the traps must see whether it loaded phram.
  m3_report_adt "$dir"
  cat "$M3_PROCFS/interrupts" >"$dir/interrupts.txt" 2>&1 || true
  ( set +e +o pipefail; m3_report_iomem ) >"$dir/iomem.txt" 2>&1
  m3_report_readme >"$dir/README.txt"
  M3_REPORT_KLOG_SRC=$src
}

m3_report() {
  local dir out board
  board=$(this_board)
  [[ -w $PWD ]] || die "can't write to $PWD. Change to a directory you can write to (cd ~) and run this
    again. Nothing was written."
  out=$PWD/aurora-m3-report-${board:-mac}-$(date +%Y%m%d-%H%M%S).tgz
  M3_WORK=$(mktemp -d)
  trap 'm3_report_cleanup' EXIT
  trap 'm3_report_cleanup; warn "interrupted: nothing was written"; exit 130' INT TERM HUP
  dir=$M3_WORK/report
  mkdir "$dir"
  # What must not leave this Mac, gathered first (m3_privacy_mask, m3_privacy_check).
  m3_privacy_secrets >"$M3_WORK/secrets"
  m3_report_collect "$dir"
  if [[ $M3_REPORT_KLOG_SRC == journal ]]; then
    m3_privacy_pack "$dir" "$out" "$M3_WORK/secrets" "$dir/kernel-log.txt" || die "the report was not kept (see above). Nothing was written.
    Please tell us at https://github.com/omacom/linux-aurora/issues what this printed, without any file."
  else
    m3_privacy_pack "$dir" "$out" "$M3_WORK/secrets" || die "the report was not kept (see above). Nothing was written.
    Please tell us at https://github.com/omacom/linux-aurora/issues what this printed, without any file."
  fi
  m3_oslog_overlap_check
  [[ -n $(m3_smc_keys_file) ]] || warn "$(m3_smc_missing): the report has no SMC keys."
  say "Report written to $out
    It only read this Mac. The host name, user names, serial numbers and MAC addresses in it
    are masked, and the file was checked for them before it was kept.
    Attach it to an issue at https://github.com/omacom/linux-aurora/issues (drag the file
    into the comment box), together with the serial log if you recorded one."
}

# ---- privacy: what a report or survey file must not carry --------------------------------------
# This Mac's host names, as air-gpu-collect.sh finds them: this boot's, the static and transient
# ones and the FQDN, each with its short form. Generic names and names under 3 characters are
# left alone.
m3_privacy_hosts() {
  local n
  {
    uname -n 2>/dev/null
    cat "$M3_ETC/hostname" 2>/dev/null
    timeout 5 hostnamectl hostname 2>/dev/null
    timeout 5 hostnamectl --static 2>/dev/null
    timeout 5 hostname -f 2>/dev/null
  } | tr -d '[:blank:]\r' | while IFS= read -r n; do
    [[ -n $n ]] || continue
    echo "$n"
    echo "${n%%.*}"
  done | grep -E '^[A-Za-z0-9][A-Za-z0-9.-]{2,}$' |
    grep -vixE 'localhost|localhost\.localdomain|archlinux|omarchy|localdomain|host|user' || true
}

# The user names: whoever runs this, the sudo caller, and every regular account (uid 1000 to
# 59999). Root, one-character names and the mask words are left alone (a home directory is
# masked whatever its name).
m3_privacy_users() {
  {
    printf '%s\n' "${USER:-}" "${LOGNAME:-}" "${SUDO_USER:-}"
    id -un 2>/dev/null
    getent passwd 2>/dev/null | awk -F: '$3 >= 1000 && $3 < 60000 { print $1 }'
  } | grep -E '^[A-Za-z0-9._-]{2,}$' | grep -vixE 'root|nobody|host|user' || true
}

# This Mac's serial numbers, as the device tree (m1n1's root and SMBIOS entries), the firmware
# tables, USB devices, the battery and the SSD give them. Six characters or more.
m3_privacy_serials() {
  local f
  {
    while IFS= read -r -d '' f; do
      [[ ${f#"$DT"} == /aliases/* ]] && continue
      tr -d '\0' <"$f" 2>/dev/null
      echo
    done < <(find -H "$DT" -type f \( -name serial-number -o -name serial -o -name '*-serial-number' \
      -o -name '*serial_number' \) -print0 2>/dev/null)
    for f in "$M3_SYSFS"/class/dmi/id/product_serial "$M3_SYSFS"/class/dmi/id/board_serial \
      "$M3_SYSFS"/class/dmi/id/chassis_serial; do
      if [[ -e $f ]]; then $sudo cat "$f" 2>/dev/null; echo; fi
    done
    for f in "$M3_SYSFS"/bus/usb/devices/*/serial; do
      # A root hub's "serial" is its controller's name.
      [[ ${f%/serial} == */usb[0-9]* ]] || { cat "$f" 2>/dev/null; echo; }
    done
    for f in "$M3_SYSFS"/class/power_supply/*/serial_number "$M3_SYSFS"/class/nvme/*/serial; do
      [[ -e $f ]] && { cat "$f" 2>/dev/null; echo; }
    done
  } | tr -d '\r' | sed 's/^[[:space:]]*//; s/[[:space:]]*$//' | grep -E '^[A-Za-z0-9._-]{6,}$' |
    grep -vE '^[0.]+$' || true
}

# This Mac's MAC and Bluetooth addresses as 12 lowercase hex digits: from the device tree (as
# the boot loader passes them, in both byte orders) and from every network interface.
m3_privacy_macs() {
  local f h
  {
    while IFS= read -r -d '' f; do
      [[ $(stat -c %s "$f" 2>/dev/null) == 6 ]] || continue
      h=$(od -An -tx1 -v "$f" 2>/dev/null | tr -d ' \n')
      echo "$h"
      echo "${h:10:2}${h:8:2}${h:6:2}${h:4:2}${h:2:2}${h:0:2}"
    done < <(find -H "$DT" -type f \( -name local-mac-address -o -name mac-address -o -name local-bd-address \) \
      -print0 2>/dev/null)
    for f in "$M3_SYSFS"/class/net/*/address "$M3_SYSFS"/class/bluetooth/*/address; do
      [[ -e $f ]] && tr -d ':\n' <"$f" 2>/dev/null && echo
    done
  } | tr 'A-F' 'a-f' | grep -xE '[0-9a-f]{12}' | grep -vxE '0{12}|f{12}' || true
}

# What m3_privacy_mask masks and m3_privacy_check looks for: "host NAME", "user NAME",
# "serial VALUE" and "mac HEX12" lines, longest first within each kind, each once.
m3_privacy_secrets() {
  local kind
  for kind in host user serial mac; do
    "m3_privacy_${kind}s" 2>/dev/null | awk -v k="$kind" 'NF { print length($0), k, $0 }' |
      sort -k1,1nr | awk '!seen[tolower($3)]++ { print $2, $3 }' || true
  done
}

# The sed script m3_privacy_mask runs over a text file (MODE text) or a binary one (MODE bin): a
# binary file keeps its length, with x in place of every masked byte. A name is masked where it
# stands alone (not inside a longer word); a MAC address wherever it has the shape of one.
m3_privacy_sed() { # SECRETS MODE
  local kind value re rep n=0
  while read -r kind value; do
    [[ -n $value ]] || continue
    n=$((n + 1))
    re=${value//./\\.}
    case $kind in
      host) rep=host ;;
      user) rep=user ;;
      serial) rep=SERIAL ;;
      mac) rep=xxxxxxxxxxxx ;;
      *) continue ;;
    esac
    [[ $2 == text ]] || rep=$(printf '%*s' "${#value}" '' | tr ' ' x)
    if [[ $kind == mac ]]; then
      printf ':m%d\ns/(^|[^[:xdigit:]])%s([^[:xdigit:]]|$)/\\1%s\\2/I\ntm%d\n' "$n" "$re" "$rep" "$n"
    else
      printf ':m%d\ns/(^|[^[:alnum:]])%s([^[:alnum:]]|$)/\\1%s\\2/I\ntm%d\n' "$n" "$re" "$rep" "$n"
    fi
  done <"$1"
  echo 's/([0-9a-fA-F]{2}[:-]){5}[0-9a-fA-F]{2}/xx:xx:xx:xx:xx:xx/g'
  if [[ $2 == text ]]; then
    printf '%s\n' 's/(serial[ _-]?(number|num|no)\b[^[:alnum:]]{0,4})[[:alnum:]][[:alnum:]._-]{3,}/\1SERIAL/gI' \
      's#/home/[^/[:space:]]+/#/home/USER/#g' 's/Hostname set to <[^>]*>/Hostname set to <host>/g'
  fi
}

# Masks every file under DIR in place (see m3_privacy_sed). Each JOURNAL file is a kernel log in
# the journal's format, whose host column is masked on every line.
m3_privacy_mask() { # DIR SECRETS [JOURNAL...]
  local dir=$1 secrets=$2 f
  shift 2
  m3_privacy_sed "$secrets" text >"$secrets.text.sed" || return 1
  m3_privacy_sed "$secrets" bin >"$secrets.bin.sed" || return 1
  for f in "$@"; do
    [[ -f $f ]] || continue
    LC_ALL=C sed -i -E 's/^(\[ *[0-9.]+\]) [^ ]+ /\1 host /' -- "$f" || return 1
  done
  while IFS= read -r -d '' f; do
    if LC_ALL=C grep -qI . "$f"; then
      LC_ALL=C sed -i -E -f "$secrets.text.sed" -- "$f" || return 1
    else
      LC_ALL=C sed -i -E -f "$secrets.bin.sed" -- "$f" || return 1
    fi
  done < <(find "$dir" -type f -print0)
}

# Looks for what m3_privacy_mask masks in the finished tgz: in every file it holds, in the file
# list (only the files packed) and in the owner fields (numeric, 0). Prints one line per finding,
# naming the kind and the file, never the value. Returns 1 when anything is found.
m3_privacy_check() { # TGZ SECRETS LIST
  local tgz=$1 secrets=$2 list=$3 x kind value what f found=0
  x=$(mktemp -d)
  if ! tar -tzf "$tgz" >"$x/names" 2>/dev/null || ! tar --numeric-owner -tvzf "$tgz" >"$x/long" 2>/dev/null ||
    ! mkdir "$x/files" || ! tar -xzf "$tgz" -C "$x/files" 2>/dev/null; then
    echo "the file could not be read back"
    rm -rf "$x"
    return 1
  fi
  sed 's#^\./##' "$x/names" | grep -v '/$' | grep -vx '\.\?' | LC_ALL=C sort | cmp -s - "$list" ||
    { echo "the file list differs from what was collected"; found=1; }
  awk '$2 != "0/0" { bad = 1 } END { exit !bad }' "$x/long" && { echo "the file names an owner"; found=1; }
  for kind in host user serial mac; do
    while read -r what value; do
      [[ $what == "$kind" && -n $value ]] || continue
      if [[ $kind == mac ]]; then
        printf '(^|[^[:xdigit:]])%s([^[:xdigit:]]|$)\n' "$value"
      else
        printf '(^|[^[:alnum:]])%s([^[:alnum:]]|$)\n' "${value//./\\.}"
      fi
    done <"$secrets" >"$x/$kind.re"
  done
  while IFS= read -r -d '' f; do
    for kind in host user serial mac; do
      [[ -s $x/$kind.re ]] || continue
      if LC_ALL=C grep -qaiE -f "$x/$kind.re" "$f"; then
        case $kind in
          host) what="the host name" ;;
          user) what="a user name" ;;
          serial) what="a serial number" ;;
          mac) what="a MAC address" ;;
        esac
        echo "$what in ${f#"$x/files/"}"
        found=1
      fi
    done
    if LC_ALL=C grep -aoiE '([0-9a-f]{2}[:-]){5}[0-9a-f]{2}' "$f" | grep -q .; then
      echo "a MAC address in ${f#"$x/files/"}"
      found=1
    fi
    if LC_ALL=C grep -aoiE 'serial[ _-]?(number|num|no)\b[^[:alnum:]]{0,4}[[:alnum:]][[:alnum:]._-]{3,}' "$f" |
      grep -vq 'SERIAL$'; then
      echo "a serial number in ${f#"$x/files/"}"
      found=1
    fi
  done < <(find "$x/files" -type f -print0)
  rm -rf "$x"
  return $((found))
}

# Masks DIR's files, packs them into a tgz in the work directory and checks it
# (m3_privacy_check). Only a file that passed is copied to OUT (through OUT.partial, which the
# traps remove); one that didn't is removed, and it returns 1.
M3_PARTIAL=""
m3_privacy_pack() { # DIR OUT SECRETS [JOURNAL...]
  local dir=$1 out=$2 secrets=$3 list tgz problems
  shift 3
  list=$secrets.list tgz=$secrets.tgz
  m3_privacy_mask "$dir" "$secrets" "$@" || { warn "could not mask the collected files"; return 1; }
  (cd "$dir" && find . -type f -printf '%P\n') | LC_ALL=C sort >"$list"
  if ! tar --owner=0 --group=0 --numeric-owner -czf "$tgz" -C "$dir" .; then
    warn "could not pack the collected files"
    return 1
  fi
  if ! problems=$(m3_privacy_check "$tgz" "$secrets" "$list"); then
    rm -f "$tgz"
    warn "the privacy check found what should have been masked, so the file was removed:
    ${problems//$'\n'/$'\n'    }"
    return 1
  fi
  M3_PARTIAL=$out.partial
  if ! { cp "$tgz" "$M3_PARTIAL" && mv -f "$M3_PARTIAL" "$out"; }; then
    rm -f "$M3_PARTIAL"
    M3_PARTIAL=""
    warn "could not write $out (run this from a directory you can write to, such as your home)"
    return 1
  fi
  M3_PARTIAL=""
}

# ---- the M3 power survey (--m3-power-survey) ---------------------------------------------------
# Opt-in, on any M3: which SMC temperature (T*) and power (P*) keys follow the CPU clusters and
# the display. It samples every T* and P* key about once a second while it runs short, fixed
# loads, each after a rest: idle, all CPUs busy, P-cores only, E-cores only, and, when the Mac has
# a backlight, the backlight at maximum and at minimum. The load is plain busy loops, one pinned
# to each CPU. The traps stop them and put the backlight back on any exit, Ctrl-C included, and
# each loop ends on its own (timeout) shortly after its phase anyway. It stops early when a CPU or
# SoC die temperature key reads M3_SURVEY_LIMIT_MC or more. One masked tgz in the current
# directory. It is the M3 Air survey of issue #35 (air-smc-rails.sh), for every M3.
M3_SURVEY_PHASE_S=30   # seconds of each phase
M3_SURVEY_REST_S=10    # seconds at rest before each phase, so it starts from the floor
M3_SURVEY_WAIT_S=10    # seconds to press Ctrl-C after the plan is printed
M3_SURVEY_TICK=1       # the length of a second (tests shorten it)
M3_SURVEY_GAP=0.2      # pause between two reads of the key list; a read takes about a second
# The CPU and SoC die temperature keys: Tp* and Te*, the P- and E-cluster dies (as this kernel's
# T8140 hwmon node labels them); Tf*, the M3 family's CPU and GPU die keys (macsmc-hwmon's J516S
# list); Tg*, the GPU. A reading outside -40..150 C is not taken as a temperature, as
# macsmc-hwmon's SoC die zone does not.
M3_SURVEY_DIE_KEYS='^T[pefg]'
M3_SURVEY_LIMIT_MC=100000
M3_SURVEY_MIN_MC=-40000
M3_SURVEY_MAX_MC=150000
# A load runs only while the sampler runs and a watched temperature is current: a valid sample
# (a number, -40..150 C) of a die key or a SoC thermal zone no older than this many real seconds.
M3_SURVEY_STALE_S=10
# The run's state, for the traps.
M3_SURVEY_PIDS=()
M3_SURVEY_P=()
M3_SURVEY_E=()
M3_SURVEY_SAMPLER=""
M3_SURVEY_BL=""
M3_SURVEY_BL_START=""
M3_SURVEY_SEEN=0
M3_SURVEY_HOT=""
M3_SURVEY_STOP=""
# The samples that count as a watched temperature (set by m3_power_survey; empty: nothing is
# watched, and no load may run), when the sampler started, and the newest such sample's time.
M3_SURVEY_WATCH_RE=""
M3_SURVEY_STARTED=""
M3_SURVEY_LAST=""
# Why the survey failed (no load runs after it is set), what the restore left undone (one line
# per item, with the command that undoes it), and what it verified.
M3_SURVEY_FAIL=""
M3_SURVEY_CLEANUP_ERR=""
M3_SURVEY_BL_DONE=""
M3_SURVEY_REPORTED=0

# The CPUs split by capacity, as the Air survey split them: those with the highest cpu_capacity
# are the P-cores (M3_SURVEY_P), the others the E-cores (M3_SURVEY_E). Offline CPUs are left out.
m3_survey_cpus() {
  local c n cap max=0
  local -a cpus=()
  M3_SURVEY_P=() M3_SURVEY_E=()
  for c in "$M3_SYSFS"/devices/system/cpu/cpu[0-9]*; do
    [[ -d $c && $(m3_attr "$c/online") != 0 ]] || continue
    cpus+=("${c##*/cpu}")
    cap=$(m3_attr "$c/cpu_capacity")
    if [[ $cap =~ ^[0-9]+$ ]] && ((cap > max)); then max=$cap; fi
  done
  while read -r n; do
    [[ -n $n ]] || continue
    cap=$(m3_attr "$M3_SYSFS/devices/system/cpu/cpu$n/cpu_capacity")
    # A CPU without a capacity counts as a P-core, as in the Air survey.
    if [[ ! $cap =~ ^[0-9]+$ ]] || ((cap == max)); then M3_SURVEY_P+=("$n"); else M3_SURVEY_E+=("$n"); fi
  done < <(printf '%s\n' ${cpus[@]+"${cpus[@]}"} | sort -n)
}

# The panel's backlight: of the backlight devices, the one with the largest range; or nothing.
m3_survey_backlight() {
  local b m best="" bm=-1
  for b in "$M3_SYSFS"/class/backlight/*; do
    m=$(m3_attr "$b/max_brightness")
    [[ $m =~ ^[0-9]+$ ]] || continue
    if ((m > bm)); then best=$b bm=$m; fi
  done
  echo "$best"
}

# Sets the backlight's brightness to VALUE and reads it back: returns 1 unless it reads VALUE.
m3_survey_bl_set() {
  echo "$1" | $sudo tee "$M3_SURVEY_BL/brightness" >/dev/null 2>&1 || return 1
  [[ $(m3_attr "$M3_SURVEY_BL/brightness") == "$1" ]]
}

# One busy loop pinned to each CPU given, each ending on its own shortly after a phase.
m3_survey_load() {
  local n max
  # Never without a watched temperature (the idle-only run has none).
  if [[ -z $M3_SURVEY_WATCH_RE ]]; then
    M3_SURVEY_FAIL="no CPU or SoC die temperature is watched, so no load may run"
    return 1
  fi
  max=$(awk -v s="$M3_SURVEY_PHASE_S" -v t="$M3_SURVEY_TICK" 'BEGIN { printf "%d", s * t + 15 }')
  for n in "$@"; do
    timeout "$max" taskset -c "$n" sh -c 'while :; do :; done' &
    M3_SURVEY_PIDS+=("$!")
  done
}

# Whether a process (by pid) still runs: not gone, not a zombie. From the real /proc.
m3_survey_alive() {
  local st
  [[ -n $1 ]] || return 1
  st=$(cat "/proc/$1/stat" 2>/dev/null) || return 1
  st=${st##*) }
  [[ ${st%% *} != Z ]]
}

# The process groups of loads (timeout makes one per loop) that are still running.
M3_SURVEY_LOAD_LEFT=""
m3_survey_stop_load() {
  local pid
  if ((${#M3_SURVEY_PIDS[@]})); then
    kill "${M3_SURVEY_PIDS[@]}" 2>/dev/null || true
    wait "${M3_SURVEY_PIDS[@]}" 2>/dev/null || true
    for pid in "${M3_SURVEY_PIDS[@]}"; do
      if pgrep -g "$pid" >/dev/null 2>&1; then
        kill -KILL -- "-$pid" 2>/dev/null || true
        sleep 0.2
        if pgrep -g "$pid" >/dev/null 2>&1; then M3_SURVEY_LOAD_LEFT+=" $pid"; fi
      fi
    done
  fi
  M3_SURVEY_PIDS=()
}

# The thermal zones to sample, as "path tz:<type>" lines.
m3_survey_zones() {
  local z
  for z in "$M3_SYSFS"/class/thermal/thermal_zone*; do
    [[ -r $z/temp ]] && echo "$z tz:$(m3_attr "$z/type" | tr -c 'A-Za-z0-9_.\n-' _)"
  done
  return 0
}

# The sampler, as root: while $M3_WORK/run exists it reads the SMC key list (KEYS, when there is
# one) and every thermal zone, back to back, and labels each read with the phase in
# $M3_WORK/phase. samples.txt gets "time phase key value" lines; a thermal zone's key is
# tz:<type>.
m3_survey_sampler_start() { # KEYS SECONDS
  : >"$M3_WORK/run"
  echo start >"$M3_WORK/phase"
  # shellcheck disable=SC2016 # expanded by that bash
  $sudo timeout "$2" bash -c '
    while [ -e "$2" ]; do
      ts=$(date +%s.%N)
      read -r ph <"$3" || ph=unknown
      if [ -n "$1" ]; then
        awk -v ts="$ts" -v ph="$ph" '\''$2 ~ /^[TP]/ && NF == 6 && $6 ~ /^-?[0-9]+$/ { print ts, ph, $2, $6 }'\'' "$1" ||
          echo "$ts $ph ERR smc-key-list"
      fi
      while read -r z k; do
        read -r v <"$z/temp" 2>/dev/null && echo "$ts $ph $k $v"
      done <"$4"
      sleep "$5"
    done' _ "$1" "$M3_WORK/run" "$M3_WORK/phase" "$M3_WORK/zones" "$M3_SURVEY_GAP" >>"$M3_WORK/out/samples.txt" \
    2>>"$M3_WORK/sampler.err" &
  M3_SURVEY_SAMPLER=$!
  M3_SURVEY_STARTED=$(date +%s.%N)
}

# Stops the sampler, and checks that none of it runs any more (its processes carry the run
# file's path). Returns 1 when some still does.
M3_SURVEY_SAMPLER_LEFT=""
m3_survey_sampler_stop() {
  local left
  [[ -n $M3_WORK ]] || return 0
  rm -f "$M3_WORK/run"
  if [[ -n $M3_SURVEY_SAMPLER ]]; then
    kill "$M3_SURVEY_SAMPLER" 2>/dev/null || true
    wait "$M3_SURVEY_SAMPLER" 2>/dev/null || true
  fi
  M3_SURVEY_SAMPLER=""
  left=$(pgrep -f -- "$M3_WORK/run" 2>/dev/null | paste -sd' ') || left=""
  if [[ -n $left ]]; then
    # shellcheck disable=SC2086 # a list of pids
    $sudo kill -KILL $left 2>/dev/null || true
    sleep 0.2
    left=$(pgrep -f -- "$M3_WORK/run" 2>/dev/null | paste -sd' ') || left=""
  fi
  M3_SURVEY_SAMPLER_LEFT=$left
  [[ -z $left ]]
}

# Everything a run started or changed, put back: on every exit, and before the tgz is made.
# Everything verified, M3_SURVEY_CLEANUP_ERR empty and 0; else one line per thing left, each with
# the command that puts it right, and 1. The backlight's original value is kept until it reads
# back, so a later call tries again.
m3_survey_restore() {
  local err="" now pgid groups=""
  m3_survey_stop_load
  m3_survey_sampler_stop || true
  if [[ -n $M3_SURVEY_LOAD_LEFT ]]; then
    for pgid in $M3_SURVEY_LOAD_LEFT; do groups+=" -$pgid"; done
    err+="the CPU load still runs (process groups$M3_SURVEY_LOAD_LEFT); stop it with: kill -KILL --$groups"$'\n'
  fi
  if [[ -n $M3_SURVEY_SAMPLER_LEFT ]]; then
    err+="the sampler still runs (pids $M3_SURVEY_SAMPLER_LEFT); stop it with: sudo kill -KILL $M3_SURVEY_SAMPLER_LEFT"$'\n'
  fi
  if [[ -n $M3_SURVEY_BL && -n $M3_SURVEY_BL_START ]]; then
    if m3_survey_bl_set "$M3_SURVEY_BL_START" || { sleep 1; m3_survey_bl_set "$M3_SURVEY_BL_START"; }; then
      M3_SURVEY_BL_DONE="the backlight reads $M3_SURVEY_BL_START again"
      M3_SURVEY_BL_START=""
    else
      now=$(m3_attr "$M3_SURVEY_BL/brightness")
      err+="the backlight (${M3_SURVEY_BL##*/}) reads $now, not its original $M3_SURVEY_BL_START; restore it with: echo $M3_SURVEY_BL_START | sudo tee $M3_SURVEY_BL/brightness"$'\n'
    fi
  fi
  M3_SURVEY_CLEANUP_ERR=${err%$'\n'}
  [[ -z $err ]]
}

# The restore's problems as warnings, once.
m3_survey_report_cleanup() {
  local line
  [[ -n $M3_SURVEY_CLEANUP_ERR ]] && ((!M3_SURVEY_REPORTED)) || return 0
  M3_SURVEY_REPORTED=1
  while IFS= read -r line; do warn "not restored: $line"; done <<<"$M3_SURVEY_CLEANUP_ERR"
}

m3_survey_cleanup() { # [report]
  m3_survey_restore || true
  if [[ ${1:-} == report ]]; then m3_survey_report_cleanup; fi
  if [[ -n $M3_PARTIAL ]]; then rm -f "$M3_PARTIAL"; fi
  if [[ -n $M3_WORK ]]; then rm -rf "$M3_WORK"; fi
  M3_WORK=""
}

# Ctrl-C (or TERM, HUP): everything put back and checked, then exit 130.
m3_survey_interrupted() {
  m3_survey_cleanup
  if [[ -n $M3_SURVEY_CLEANUP_ERR ]]; then
    warn "interrupted. Nothing was written, and not everything is back as it was:"
    m3_survey_report_cleanup
  else
    warn "interrupted: the load is stopped${M3_SURVEY_BL_DONE:+ and $M3_SURVEY_BL_DONE} (checked). Nothing was written."
  fi
  exit 130
}

# Reads the samples taken since the last call (complete lines only). Sets M3_SURVEY_HOT to
# "<key> read <C> C" when a die key or a thermal zone read M3_SURVEY_LIMIT_MC or more, and
# M3_SURVEY_LAST to the time of the newest valid watched sample. While a temperature is watched,
# sets M3_SURVEY_FAIL when the sampler is gone, could not read the key list, or has no valid
# watched sample from the last M3_SURVEY_STALE_S seconds; with "current" (before a load), there
# must be such a sample, however short the run so far.
m3_survey_check() { # [current]
  local raw=$M3_WORK/out/samples.txt n now hot last err ref
  n=$(wc -l <"$raw")
  now=$(date +%s.%N)
  # "|" separates the three fields: a tab would merge empty ones.
  IFS='|' read -r hot last err < <(awk -v from="$M3_SURVEY_SEEN" -v to="$n" -v re="$M3_SURVEY_DIE_KEYS" \
    -v watch="$M3_SURVEY_WATCH_RE" -v lim="$M3_SURVEY_LIMIT_MC" -v lo="$M3_SURVEY_MIN_MC" -v hi="$M3_SURVEY_MAX_MC" '
    NR > to { exit }
    NR <= from { next }
    $3 == "ERR" { err = $4; next }
    $4 !~ /^-?[0-9]+$/ || $4 + 0 < lo || $4 + 0 > hi { next }
    hot == "" && ($3 ~ re || $3 ~ /^tz:/) && $4 + 0 >= lim { hot = sprintf("%s read %.1f C", $3, $4 / 1000) }
    watch != "" && $3 ~ watch && $1 + 0 > last { last = $1 + 0 }
    END { printf "%s|%s|%s\n", hot, (last ? sprintf("%.3f", last) : ""), err }' "$raw")
  M3_SURVEY_SEEN=$n
  M3_SURVEY_HOT=$hot
  if [[ -n $last ]]; then M3_SURVEY_LAST=$last; fi
  [[ -n $M3_SURVEY_WATCH_RE ]] || return 0
  if ! m3_survey_alive "$M3_SURVEY_SAMPLER"; then
    M3_SURVEY_FAIL="the temperature sampler is not running"
  elif [[ -n $err ]]; then
    M3_SURVEY_FAIL="the temperature sampler could not read the SMC key list"
  elif [[ ${1:-} == current && -z $M3_SURVEY_LAST ]]; then
    M3_SURVEY_FAIL="there is no valid CPU or SoC die temperature sample yet"
  else
    ref=${M3_SURVEY_LAST:-$M3_SURVEY_STARTED}
    if [[ ${1:-} == current ]]; then ref=$M3_SURVEY_LAST; fi
    if awk -v n="$now" -v r="${ref:-0}" -v s="$M3_SURVEY_STALE_S" 'BEGIN { exit !(n - r > s) }'; then
      M3_SURVEY_FAIL="no valid CPU or SoC die temperature sample for more than $M3_SURVEY_STALE_S s"
    fi
  fi
  return 0
}

# One stretch of sampling under a phase NAME, for SECONDS. Returns 1 when a die key read too
# hot (M3_SURVEY_STOP says so) or the temperatures are no longer watched (M3_SURVEY_FAIL): the
# load is stopped at once.
m3_survey_sample() { # NAME SECONDS
  local i tmp=$M3_WORK/phase.new
  echo "$1" >"$tmp" && mv -f "$tmp" "$M3_WORK/phase"
  echo "$1 $(date +%s.%N) start" >>"$M3_WORK/out/phases.txt"
  for ((i = 0; i < $2; i++)); do
    sleep "$M3_SURVEY_TICK"
    m3_survey_check
    if [[ -n $M3_SURVEY_HOT ]]; then
      m3_survey_stop_load
      M3_SURVEY_STOP="$M3_SURVEY_HOT during $1"
      echo "$1 $(date +%s.%N) stopped: $M3_SURVEY_STOP" >>"$M3_WORK/out/phases.txt"
      return 1
    fi
    if [[ -n $M3_SURVEY_FAIL ]]; then
      m3_survey_stop_load
      M3_SURVEY_FAIL+=" (during $1)"
      echo "$1 $(date +%s.%N) failed: $M3_SURVEY_FAIL" >>"$M3_WORK/out/phases.txt"
      return 1
    fi
  done
  echo "$1 $(date +%s.%N) end" >>"$M3_WORK/out/phases.txt"
}

# A rest, then the phase NAME: idle, a CPU load (on CPUS), or a backlight setting.
m3_survey_phase() { # NAME [CPUS...]
  local name=$1
  shift
  m3_survey_stop_load
  m3_survey_sample "rest-before-$name" "$M3_SURVEY_REST_S" || return 1
  if [[ $name != idle ]]; then
    # Only on a current watched temperature, from a running sampler.
    m3_survey_check current
    if [[ -n $M3_SURVEY_FAIL ]]; then
      M3_SURVEY_FAIL+=" (before $name)"
      echo "$name $(date +%s.%N) failed: $M3_SURVEY_FAIL" >>"$M3_WORK/out/phases.txt"
      return 1
    fi
  fi
  case $name in
    idle) ;;
    backlight-max)
      m3_survey_bl_set "$(m3_attr "$M3_SURVEY_BL/max_brightness")" ||
        { M3_SURVEY_FAIL="could not set the backlight to its maximum"; return 1; } ;;
    backlight-min) m3_survey_bl_set 1 || { M3_SURVEY_FAIL="could not set the backlight to 1"; return 1; } ;;
    *) m3_survey_load "$@" || return 1 ;;
  esac
  m3_survey_sample "$name" "$M3_SURVEY_PHASE_S" || return 1
  m3_survey_stop_load
}

# summary.txt: each key's mean per phase (T* in C, P* in W), the hottest die key per phase, and
# how the run ended.
m3_survey_summary() { # PHASES...
  local out=$M3_WORK/out
  printf '%-6s' "#key"
  printf ' %13s' "$@"
  printf '   (T* and tz:* in C, P* in W; mean per phase)\n'
  awk -v order="$*" '
    BEGIN { np = split(order, o, " ") }
    $2 !~ /^rest-/ { s[$3, $2] += $4; n[$3, $2]++; keys[$3] = 1 }
    END {
      for (k in keys) {
        line = sprintf("%-6s", k)
        for (i = 1; i <= np; i++) line = line sprintf(" %13.2f", n[k, o[i]] ? s[k, o[i]] / n[k, o[i]] / 1000 : 0)
        print line
      }
    }' "$out/samples.txt" | LC_ALL=C sort
  echo
  echo "hottest CPU or SoC die key (${M3_SURVEY_DIE_KEYS}) or thermal zone per phase, C:"
  awk -v re="$M3_SURVEY_DIE_KEYS" -v lo="$M3_SURVEY_MIN_MC" -v hi="$M3_SURVEY_MAX_MC" '
    ($3 ~ re || $3 ~ /^tz:/) && $4 + 0 >= lo && $4 + 0 <= hi && (!($2 in m) || $4 + 0 > m[$2]) { m[$2] = $4 + 0; k[$2] = $3 }
    END { for (p in m) printf "  %-22s %7.1f (%s)\n", p, m[p] / 1000, k[p] }' "$out/samples.txt" | LC_ALL=C sort
  echo
  if [[ -n $M3_SURVEY_FAIL ]]; then
    echo "result: failed: $M3_SURVEY_FAIL; the load was stopped and the later phases did not run"
  elif [[ -n $M3_SURVEY_STOP ]]; then
    echo "result: stopped early: $M3_SURVEY_STOP (limit $((M3_SURVEY_LIMIT_MC / 1000)) C); the later phases did not run"
  else
    echo "result: completed"
  fi
  if [[ -n $M3_SURVEY_CLEANUP_ERR ]]; then
    echo "restore: FAILED"
    echo "  not restored: ${M3_SURVEY_CLEANUP_ERR//$'\n'/$'\n'  not restored: }"
  else
    echo "restore: checked: the load and the sampler have stopped${M3_SURVEY_BL_DONE:+, $M3_SURVEY_BL_DONE}"
  fi
}

m3_power_survey() {
  local board soc keys out tool total n bl_name="" smc_note="" watch="" plan
  local -a phases=()
  board=$(this_board) soc=$(this_soc)
  is_m3 || die "--m3-power-survey is for an M3 Mac (M3, M3 Pro or M3 Max), and this Mac is ${board:-?} (${soc:-?}).
    Nothing was run."
  for tool in taskset timeout; do
    command -v "$tool" >/dev/null || die "--m3-power-survey needs $tool. Nothing was run."
  done
  if [[ -n $sudo ]]; then
    $sudo true || die "--m3-power-survey needs root (sudo) to read the SMC keys. Nothing was run."
  fi
  [[ -w $PWD ]] || die "can't write to $PWD. Change to a directory you can write to (cd ~) and run this
    again. Nothing was run."
  m3_survey_cpus
  ((${#M3_SURVEY_P[@]})) || die "found no CPUs in $M3_SYSFS/devices/system/cpu. Nothing was run."
  out=$PWD/aurora-m3-power-${board:-mac}-$(date +%Y%m%d-%H%M%S).tgz
  M3_WORK=$(mktemp -d)
  mkdir "$M3_WORK/out"
  trap 'm3_survey_cleanup report' EXIT
  trap 'm3_survey_interrupted' INT TERM HUP
  keys=$(m3_smc_keys_file)
  if [[ -n $keys ]]; then
    # shellcheck disable=SC2016 # awk's fields
    $sudo awk 'NR == 1 || $2 ~ /^[TP]/' "$keys" >"$M3_WORK/out/smc-keys.txt" 2>/dev/null || true
  else
    smc_note=$(m3_smc_missing)
    echo "$smc_note" >"$M3_WORK/out/smc-keys.txt"
    warn "$smc_note. The survey records the CPU topology and the thermal zones only."
  fi
  m3_survey_zones >"$M3_WORK/zones"
  # A load runs only while a die temperature is watched: an SMC die key, or a thermal zone of the
  # SoC or the CPUs.
  M3_SURVEY_WATCH_RE=""
  n=$(awk -v re="$M3_SURVEY_DIE_KEYS" '$2 ~ re && NF == 6 && $6 ~ /^-?[0-9]+$/' "$M3_WORK/out/smc-keys.txt" | wc -l)
  if ((n)); then watch="$n SMC die keys (Tp*, Te*, Tf*, Tg*)" M3_SURVEY_WATCH_RE=$M3_SURVEY_DIE_KEYS; fi
  n=$(awk '$2 ~ /^tz:.*(die|cpu|soc|hotspot)/' "$M3_WORK/zones" | wc -l)
  if ((n)); then
    watch+="${watch:+ and }$n SoC thermal zones"
    M3_SURVEY_WATCH_RE+="${M3_SURVEY_WATCH_RE:+|}^tz:.*(die|cpu|soc|hotspot)"
  fi
  M3_SURVEY_BL=$(m3_survey_backlight)
  if [[ -z $watch ]]; then
    phases=(idle)
    plan="idle only: there is no CPU or SoC die temperature to watch on this kernel, so the load
      and backlight phases are left out."
  else
    phases=(idle cpu-all cpu-p)
    plan="idle;
      all CPUs busy (${M3_SURVEY_P[*]} ${M3_SURVEY_E[*]});
      the P-cores only (${M3_SURVEY_P[*]});"
    if ((${#M3_SURVEY_E[@]})); then
      phases+=(cpu-e)
      plan+=$'\n'"      the E-cores only (${M3_SURVEY_E[*]});"
    else
      plan+=$'\n'"      (no E-cores found: that phase is left out);"
    fi
    if [[ -n $M3_SURVEY_BL ]]; then
      phases+=(backlight-max backlight-min) bl_name=${M3_SURVEY_BL##*/}
      plan+=$'\n'"      the backlight ($bl_name) at maximum, then at minimum, then at its starting value."
    else
      plan+=$'\n'"      no backlight device, so the backlight phases are left out."
    fi
    plan+=$'\n'"    It stops early if a watched temperature ($watch, or any thermal zone)
    reads $((M3_SURVEY_LIMIT_MC / 1000)) C, and fails, stopping the load, if no fresh one comes in for
    $M3_SURVEY_STALE_S s or the sampler stops."
  fi
  total=$(awk -v n="${#phases[@]}" -v p="$M3_SURVEY_PHASE_S" -v r="$M3_SURVEY_REST_S" -v t="$M3_SURVEY_TICK" \
    'BEGIN { printf "%d", n * (p + r) * t }')
  say "M3 power survey on this ${board:-Mac} (${soc:-?}): about $(((total + 59) / 60)) minutes.
    It samples the SMC's temperature (T*) and power (P*) keys and the thermal zones about once a
    second through short, fixed phases, each $M3_SURVEY_PHASE_S s after $M3_SURVEY_REST_S s at rest:
      $plan
    The load stops and the backlight goes back (read back to check) at the end, on an error and on
    Ctrl-C.
    Nothing else changes. It writes one file in this directory: $out
    Close other programs, keep the display on and the Mac on power."
  say "Starting in $M3_SURVEY_WAIT_S seconds. Press Ctrl-C now to cancel."
  sleep "$M3_SURVEY_WAIT_S"
  m3_privacy_secrets >"$M3_WORK/secrets"
  {
    echo "board: ${board:-?} soc: ${soc:-?}"
    echo "model: $(m3_dt_words "$DT/model")"
    echo "uname: $(uname -srvm)"
    echo "installer: $TAG"
    echo "phases: ${phases[*]}; $M3_SURVEY_PHASE_S s each after $M3_SURVEY_REST_S s at rest (one second = $M3_SURVEY_TICK s)"
    echo "watched: ${watch:-nothing (no load phases)}; stop at $M3_SURVEY_LIMIT_MC mC on a die key (${M3_SURVEY_DIE_KEYS}) or any thermal zone"
    echo "smc: ${smc_note:-key list $keys}"
  } >"$M3_WORK/out/system.txt"
  {
    echo "P-cores: ${M3_SURVEY_P[*]}"
    echo "E-cores: ${M3_SURVEY_E[*]:-none}"
    for n in "${M3_SURVEY_P[@]}" ${M3_SURVEY_E[@]+"${M3_SURVEY_E[@]}"}; do
      echo "cpu$n capacity $(m3_attr "$M3_SYSFS/devices/system/cpu/cpu$n/cpu_capacity") cluster $(m3_attr "$M3_SYSFS/devices/system/cpu/cpu$n/topology/cluster_id")"
    done
    for n in "$M3_SYSFS"/devices/system/cpu/cpufreq/policy*; do
      [[ -d $n ]] && echo "${n##*/}: cpus $(m3_attr "$n/related_cpus"), $(m3_attr "$n/cpuinfo_min_freq")-$(m3_attr "$n/cpuinfo_max_freq") kHz, $(m3_attr "$n/scaling_driver")"
    done
    echo "thermal zones: $(awk '{ print $2 }' "$M3_WORK/zones" | paste -sd' ')"
    if [[ -n $bl_name ]]; then
      echo "backlight: $bl_name, max_brightness $(m3_attr "$M3_SURVEY_BL/max_brightness")"
    elif [[ -n $watch ]]; then
      echo "no backlight device: backlight phases skipped"
    fi
  } >"$M3_WORK/out/cpus.txt"
  : >"$M3_WORK/out/samples.txt"
  : >"$M3_WORK/out/phases.txt"
  if [[ -n $bl_name ]]; then M3_SURVEY_BL_START=$(m3_attr "$M3_SURVEY_BL/brightness"); fi
  [[ $M3_SURVEY_BL_START != - ]] || M3_SURVEY_BL_START=""
  m3_survey_sampler_start "$keys" "$((total + 60))"
  for n in "${phases[@]}"; do
    case $n in
      cpu-all) m3_survey_phase "$n" "${M3_SURVEY_P[@]}" ${M3_SURVEY_E[@]+"${M3_SURVEY_E[@]}"} ;;
      cpu-p) m3_survey_phase "$n" "${M3_SURVEY_P[@]}" ;;
      cpu-e) m3_survey_phase "$n" "${M3_SURVEY_E[@]}" ;;
      *) m3_survey_phase "$n" ;;
    esac || break
  done
  m3_survey_restore || true
  {
    m3_survey_summary "${phases[@]}"
    if [[ -n $smc_note ]]; then echo "smc: $smc_note"; fi
    if [[ -z $watch ]]; then echo "loads: none (no CPU or SoC die temperature to watch on this kernel)"; fi
  } >"$M3_WORK/out/summary.txt"
  cp "$M3_WORK/sampler.err" "$M3_WORK/out/sampler-errors.txt" 2>/dev/null || true
  m3_privacy_pack "$M3_WORK/out" "$out" "$M3_WORK/secrets" ||
    die "the survey's file was not kept (see above). Please tell us at
    https://github.com/omacom/linux-aurora/issues what this printed, without any file."
  m3_survey_cleanup
  trap - EXIT INT TERM HUP
  if [[ -n $M3_SURVEY_STOP && -z $M3_SURVEY_FAIL ]]; then
    warn "the survey stopped early: $M3_SURVEY_STOP, at or over the $((M3_SURVEY_LIMIT_MC / 1000)) C limit.
    The load stopped at once; the phases after it did not run."
  fi
  if [[ -n $smc_note ]]; then warn "$smc_note: the file has the CPU topology and the thermal zones only."; fi
  say "Power survey written to $out
    The host name, user names, serial numbers and MAC addresses in it are masked, and the file
    was checked for them before it was kept. Attach it to your issue at
    https://github.com/omacom/linux-aurora/issues (drag the file into the comment box)."
  m3_survey_report_cleanup
  if [[ -n $M3_SURVEY_FAIL ]]; then die "the survey failed: $M3_SURVEY_FAIL. The load was stopped."; fi
  [[ -z $M3_SURVEY_CLEANUP_ERR ]] || die "the survey could not put everything back (see above)."
  say "Checked: the load and the sampler have stopped${M3_SURVEY_BL_DONE:+, and $M3_SURVEY_BL_DONE}."
}

# ---- the end of an install on an M3 with no handoff path yet -----------------------------------
# The M3 Max (t6031, t6034) and the T8122 Macs that are not an Air (J504, J433, J434): case C of
# the test plan. m3_plan always keeps them kernel-only.
is_m3_kernel_only_chip() {
  tr '\0' '\n' <"$DT/compatible" 2>/dev/null | grep -Eqx 'apple,(t6031|t6034)' && return 0
  tr '\0' '\n' <"$DT/compatible" 2>/dev/null | grep -qx 'apple,t8122' && ! is_m3_air
}

# The release of this script's kernel as uname -r prints it (VERSION 7.1.12.aurora2-12.2 is
# 7.1.12-2-12.2-sep-ARCH), or nothing when VERSION has another form.
m3_kernel_release() {
  [[ $VERSION =~ ^([0-9]+\.[0-9]+\.[0-9]+)\.aurora([0-9]+)-(.+)$ ]] || return 0
  echo "${BASH_REMATCH[1]}-${BASH_REMATCH[2]}-${BASH_REMATCH[3]}-sep-ARCH"
}

# The last block of an install's summary on those Macs: what to do next, in order. An M3 Max on
# its boot loader variant (not through the test kit, which says what happens next itself) gets
# the same block, plus where its earlier boot.bin is.
m3_next_steps() {
  local rel
  if [[ $M3_MODE == handoff ]] && is_m3_max && ((!M3MAX_KIT)); then
    rel=$(m3_kernel_release)
    echo
    echo "======================== NEXT STEPS: M3 MAX BOOT LOADER VARIANT ($(this_board)) ========================"
    echo "  1. Reboot. The boot menu and the desktop should come up as before."
    echo "  2. Check that the Mac runs the new kernel: uname -r should print ${rel:-a version that ends in -sep-ARCH}."
    echo "  3. Write the bring-up report (read-only; host name, user names, serial numbers and MAC"
    echo "     addresses masked):"
    echo "       curl -fsSL $LATEST_URL | bash -s -- --m3-report"
    echo "  4. Attach aurora-m3-report-$(this_board)-<date>.tgz to an issue at https://github.com/omacom/linux-aurora/issues"
    echo "  The boot loader this Mac had is kept as m1n1/boot.bin.before-$VERSION on the EFI partition;"
    echo "  the steps above say how to put it back. --uninstall puts it back too."
    echo "=========================================================================================="
    return 0
  fi
  [[ $M3_MODE == kernel ]] && is_m3_kernel_only_chip || return 0
  rel=$(m3_kernel_release)
  echo
  echo "======================== NEXT STEPS: M3 BRING-UP ($(this_board)) ========================"
  echo "  1. Reboot."
  echo "  2. Check that the Mac runs the new kernel:"
  echo "       uname -r"
  echo "     It should print ${rel:-a version that ends in -sep-ARCH}."
  echo "  3. Write the bring-up report. It only reads, and writes one file in the current"
  echo "     directory, aurora-m3-report-$(this_board)-<date>.tgz, with the host name, user names,"
  echo "     serial numbers and MAC addresses masked:"
  echo "       curl -fsSL $LATEST_URL | bash -s -- --m3-report"
  echo "  4. Attach that file to an issue at https://github.com/omacom/linux-aurora/issues"
  echo "=========================================================================================="
}

# Printed by --agent-prompt, and pointed at from the end of a successful
# install. This is written for an agent driving the test on a real Mac: it says
# what to establish, what counts as a pass, and how to write it up.
agent_prompt() {
  cat <<'PROMPT'
You are testing an experimental Secure Enclave / Touch ID kernel on an Apple
Silicon Mac. Work through this in order and report at the end. Do not skip the
identification step: nearly every past report was unusable because the machine
was not pinned down.

SAFETY, NON-NEGOTIABLE
  - This touches the Secure Enclave. Confirm the owner can DFU-restore this Mac
    before you start.
  - A default install is NOT read-only. On the first boot of a Mac with a
    Touch ID profile, the SEP driver loads automatically, creates an identity
    keybag and writes to the enclave's anti-replay store (xART) -- before
    anyone enrols a finger. If the owner cannot DFU-restore, install with
    --read-only instead, before the first reboot:
      curl -fsSL https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh | bash -s -- --read-only
    A 'blacklist apple_sep' line does not prevent this on its own; the
    installer detects one and leaves the driver off.
  - Do not run any enrol/delete/re-provision loop unless the owner asks for
    it in writing: repeated cycles drift a device-wide counter.
  - Do not paste key material, serial numbers, or the contents of
    mesa_calibration.bin into a report.
  - The install replaces the Mac's boot loader (m1n1), on every Mac but an
    M3 that stays kernel-only and a MacBook Neo. Before it does, it keeps
    the old one on the EFI partition and prints the steps to put it back:
    keep them, they name this Mac's EFI partition.

IF THE MAC STOPS IN m1n1 AFTER AN INSTALL (any Mac)
  m1n1 text on screen and no boot menu, or a black screen for more than two
  minutes: the owner puts the old boot.bin back from macOS with the printed
  steps, starts Linux, and runs the two printed lines. The first keeps
  updates from rebuilding the new boot loader; the second records that its
  m1n1 failed on this Mac. Later plain runs of the one-liner then keep the
  boot loader the Mac has and install the rest, and never put that m1n1
  back. If it boots but something is wrong, the same copy goes back from
  Linux with the printed line, followed by the same two lines. Do not use
  --uninstall until the maintainer says so. Report what the screen showed
  and when, with the serial log if there is one, at
  https://github.com/iconidentify/aurora-linux/issues/6

ON AN M3 (M3, M3 Pro, M3 Max): Touch ID is not supported there yet. Do steps
0 and 1, then go to step 11, which says what should happen on your model and
how to get it added. Steps 6, 7 and 9 apply as well, and step 8 without the
fingerprint.

0. CONFIRM YOU HAVE THE CURRENT BUILD
   Results against a superseded build waste everyone's time, and a saved copy
   of this script installs its own packages forever. This script checks on
   every install and prints either "<tag> is the current release" or a warning
   naming the newer one. To check without installing:

     curl -fsSL https://api.github.com/repos/omacom/linux-aurora/releases/latest \
       | grep -m1 '"tag_name"'

   Always fetch the script from the "latest" URL rather than a tag you were
   handed, so you get the current one automatically:

     curl -fsSL https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh | bash

   State the tag you installed in your report. If you were given a specific
   older tag on purpose, say so and say why.

1. IDENTIFY THE MACHINE
     tr -d '\0' < /proc/device-tree/compatible; echo
     for p in os-fw-version system-fw-version iboot2-version; do
       printf '%s = ' "$p"
       tr -d '\0' < /proc/device-tree/chosen/asahi,$p 2>/dev/null; echo
     done
     uname -r; pacman -Q linux-aurora m1n1-aurora libfprint aurora-touchid
   asahi,system-fw-version is the sepOS the enclave actually runs and is the
   single most useful line in the whole report. asahi,os-fw-version is only the
   stub. Report both; they are often different.

2. CONFIRM THE BOOTLOADER GUARD (M2 and later)
   In the m1n1 stage 2 boot log, expect:
     SEP: Preserving iBoot warm registration; seeding RNG from ADT
   and expect NO line reading "SEP: couldn't get enough random bytes".
   You should also see the OS identity forwarded:
     FDT: apfs-preboot-uuid = '<uuid>'
   On M1 neither line is expected: the enclave is cold and m1n1 talks to it
   normally. If you see the "couldn't get enough random bytes" line on any
   machine, the stock m1n1 is still in boot.bin - say so and stop, because every
   later result is then measuring the wrong thing.
   On an M3 installed kernel-only (step 11), the boot loader is the one the Mac
   already had, so this step does not apply: say "M3, kernel-only" instead.

3. DRIVER ATTACH
   The driver is already loaded at boot by apple-sep.service; do not reload
   it. Its write mode was fixed at install time. Read what it did:
     sudo dmesg | grep -iE 'apple_sep|apple-mesa'
   The xART line ends "writes ENABLED" or "writes disabled" -- quote it, and
   say which you expected.
   PASS:  "attach: N endpoints advertised in M messages" with N >= 7, and
          /dev/sep-bio exists.
   FAIL:  "attach: 0 messages received but no endpoint advertised", or no
          apple_sep lines at all. Capture the whole block either way.
   If the profile line says a SoC you did not expect, report that verbatim.
   On every MacBook Pro M2 Pro/Max (J414s, J414c, J416s, J416c) it reads
   "T6020/J414s", or "T6020/J414s (13.5 key store)" on system firmware 26.2
   or earlier; on the MacBook Pro 14"/16" M1 Max (J314c, J316c)
   "T6000/J316s", and on the MacBook Air 13" M2 (J413) "T8112/J415". Those
   are expected. On an M2 Pro/Max, also quote the line that starts
   "M2 Pro/Max on system firmware".
   If "CREATE_KEYBAG" fails with status -13, quote that line too. Firmware
   up to 26.2 uses the 13.5 key store and later firmware variant 5. Variant
   5 works on 26.6.x and 27.0; 26.3 to 26.5 have not been tried, so a -13
   there is the report we need.
   Updating macOS from 26.2 or earlier to a later release after enrolling
   loses the enrolments: Touch ID then needs a new keybag. Say so if that
   is what happened. With the owner's agreement, start Touch ID over:
     curl -fsSL https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh | bash -s -- --reset-touchid
   then reboot and enrol again.
   The sensor itself: "sudo dmesg | grep apple-mesa" should say the power line
   came "from the device node". Quote it if it says anything else.

4. IF ATTACH FAILED, GET THE MAILBOX STATE
     sudo busybox devmem 0x396408110 32   # A2I control (t600x/t602x)
     sudo busybox devmem 0x396408114 32   # I2A control
     grep -i mbox /proc/interrupts
   Those addresses are for t600x and t602x. The mailbox base differs per SoC,
   so take it from the device tree rather than a table:

     sudo cat /proc/device-tree/soc/mbox@*/reg 2>/dev/null | xxd | head -2
     # or read the sep_mbox node address from your board's dtsi:
     #   t8103            0x242408000  -> 0x242408110 / 0x242408114
     #   t8112            0x25e408000  -> 0x25e408110 / 0x25e408114
     #   t600x, t602x     0x396408000  -> 0x396408110 / 0x396408114
     #   t8140 (Neo)      0x282608000  (a v4 mailbox: report the base and
     #                    the /proc/interrupts lines, not these offsets)

   Bit 0 is the enable; a non-zero FIFO count with an unchanged read pointer
   means the enclave never took the message. Report the raw values, not your
   reading of them.

5. TOUCH ID END TO END (only if step 3 passed)
     aurora-touchid-setup
   Enrol one finger, then:
     fprintd-verify
   Then REBOOT and run fprintd-verify again without re-enrolling. Matching
   right after enrolment and matching after a reboot are different paths (the
   second restores the enrolment from disk), so report them separately.
   If a verify fails, the driver logs the step that ended it:
     sudo dmesg | grep -E 'verify:|scrd:|Touch ID|catacomb|matching unavailable'
   fprintd's "verify-unknown-error" is not a result on its own; quote those
   lines with it.
   Then check the consumers the owner actually cares about: sudo, polkit, and
   the lock screen. Report each as pass/fail separately - partial success here
   is the normal outcome and is worth knowing precisely.

6. THUNDERBOLT AND DISPLAYPORT - TEST THIS EVEN IF YOU CAME FOR TOUCH ID
   This build carries the Thunderbolt/USB4 display and PCIe work, and it needs
   hardware reports as much as the enclave does. Do not skip it because the
   machine has no dock: "no dock available" is itself a useful answer, and
   step 6a still applies.

   6a. The controllers (no dock needed)
         ls /sys/bus/platform/devices/ | grep cio
         sudo dmesg | grep -iE 'thunderbolt|usb4|acio|tunnel' | head -30
       Expect one *.cio device per controller, bound to thunderbolt-apple-acio.
       An EMPTY /sys/bus/thunderbolt/devices/ with nothing plugged in is
       normal: routers only appear when a device is attached. Do not report
       that as a failure.
       Anything behind a Thunderbolt 3 dock's PCIe controller -- Ethernet,
       storage, and on docks such as the CalDigit TS3 Plus the USB ports too --
       now comes up on M1, M1 Pro/Max and M2 Pro/Max. On the Pro/Max chips the
       kernel starts the tunnel itself when a dock is plugged in, and logs
         port ... cold init done, status 0x3 ...
       (pcie_apple.tunnel_kernel_init=0 turns that off). Displays, and USB on
       USB4 docks, do not need the PCIe tunnel. On the M2 MacBook Air/Pro 13"
       and on M3 the tunnel isn't supported yet, so this line is normal there:
         PCIe-C tunnel disabled: not initialized by m1n1 or the kernel
       Report it if you see it on an M1, M1 Pro/Max or M2 Pro/Max.

   6b. With a dock or a DisplayPort monitor, in this order:
         - attached at boot: does the display come up, and at what resolution
           and refresh rate
         - hot unplug: does the tunnel tear down cleanly, no hang, no stuck
           compositor
         - hot replug: does the display come back, same mode as before
         - repeat both of the above on the SECOND USB-C port, not just the
           first: the ports are separate controllers and have behaved
           differently
         - two displays or a dual-output dock, if you have one
         - M2 Pro/Max: two monitors plugged straight into two USB-C ports
           (no dock), attached at boot. Does each come up at its own native
           resolution? Quote "hyprctl monitors" (name, mode, and the port each
           is on). Then log out and back in, and repeat.
         - PCIe behind the dock: Ethernet, USB storage, card readers, and a
           keyboard and mouse on the dock's USB ports. Do they all work, and
           still work after a replug? Quote:
             lspci -nn
             sudo dmesg | grep -E 'cold init done|link up after|translation fault|HC died'
           Any "translation fault" or "HC died" line is a failure to report.
           For a full report, after plugging the dock in:
             curl -fsSLO https://raw.githubusercontent.com/iconidentify/aurora-linux/refs/tags/sep-7.1.12.aurora2-11.110-test/tools/aurora-tb/tb-pcie-report
             sudo sh tb-pcie-report --no-wait

   6c. Across suspend:
         systemctl suspend
       Then wake and re-check: display back, dock devices back, and
       /dev/sep-bio still present. Report anything that needed a replug to
       recover.

7. HARDWARE VIDEO DECODE
   Play an H.264 file and an HEVC file and confirm it is not silently falling
   back to software:
     sudo dmesg | grep -iE 'avd|apple-vpu' | tail -20
   Report which codecs you exercised and at what resolution.

8. ONE SUSPEND/RESUME CYCLE AT THE END
   Ask the owner to lock the screen and, while it waits for a fingerprint,
   close the lid. Then open it and unlock with the finger. Report whether
   the Mac woke and whether the finger unlocked it, and quote:
     sudo dmesg | grep -E 'Touch ID: ending|still running|PM: suspend'
   Re-check /dev/sep-bio, the display and the dock after resume.

9. NEURAL ENGINE
   This build carries the in-tree Apple Neural Engine driver
   (aurora-silicon/linux#155), switched on for the M1, M1 Pro, M1 Max, M2 Pro
   and M2 Max. On every other Mac no Neural Engine lines are expected, and that
   is a pass. The ANE now powers off about 1.5 s after its last use:
     cat /sys/bus/platform/drivers/ane*/*.ane/power/runtime_status
   should read "suspended" while nothing uses it (M1 family).
     pacman -Q omarchy-ane-dkms 2>/dev/null   # if installed, say so: its modules replace this kernel's
     modinfo -F filename ane ane_t6021
     ls -l /dev/accel/ 2>/dev/null
     sudo dmesg | grep -iE '\bane\b|ane_t6021|neural' | head -40
   M1, M1 Pro, M1 Max PASS: the ane module is bound, /dev/accel/accel0 exists,
   and the module path is under kernel/drivers/accel/ane (not updates/dkms).
   M2 Pro, M2 Max: without the Neural Engine firmware, expect a firmware load
   error and nothing else broken. With it (fetched by Joshua Warren's
   omarchy-ane-firmware-fetch), expect ane_t6021 bound and /dev/accel/accel0.
   The M2 Pro has not run it before this build: report it either way.
   If omarchy-ane's tools are installed, also run omarchy-ane-check, and on an
   M2 Pro/Max omarchy-ane-check --smoke; quote their result lines.
   Report any kernel log line at emergency or alert level, and whether idle
   battery drain changed against the previous build. If the Mac does not finish
   booting, add module_blacklist=ane,ane_t6021 to the kernel command line from
   the boot menu, and report that.

10. MACBOOK NEO (J700) ONLY
   The Neo's Wi-Fi is in the regular kernel now. It needs this Neo's own
   firmware, calibration and country files; the installer listed any that
   were missing. Never install another Neo's files.
     sudo dmesg | grep -iE 'mt7932|piodma|REGULATORY|CALIBRATION|admission'
     iw reg get | head -3
     nmcli device wifi list | head
   - Without the country file, expect
       REGULATORY_BLOCKED: <CC> generation=N error=-2 recovery-required=0
     and nothing else broken; quote it. After installing that file, retry a
     scan (nmcli device wifi rescan): it should come up without a reboot.
   - With every file present: does it scan, connect to a WPA2 network and
     pass traffic? Try a 2.4 GHz network and a 5 GHz one on channels 36-48.
     Networks on channels 149-165 are not listed, by design.
   - systemctl suspend is refused by design while the radios are active.
     Expect "sleep refused: Neo radio bootstrap retains DMA memory until full
     hardware reset" and quote it.
   - Bluetooth is off by default. Only test it if the owner asks (see
     Documentation/networking/device_drivers/wifi/mt7932-neo.rst).

   Report "not tested" honestly rather than guessing, for any of the above.

11. M3 MACS (M3, M3 PRO, M3 MAX): WHAT SHOULD HAPPEN, AND HOW YOURS GETS ADDED
   Every M3 is experimental, and every M3 report moves M3 support forward.
   Touch ID is not supported on M3 yet: skip steps 2 to 5 and the fingerprint
   part of step 8, and expect
   "apple_sep: no platform profile for this SoC; the SEP stays disabled".
   First find the model and chip, then follow the matching case below:
     tr '\0' '\n' < /proc/device-tree/compatible | head -2
     cat /var/lib/aurora-sep/m3-mode     # after an install: kernel or handoff
     cat /var/lib/aurora-sep/m3-pro-mesa # M3 Pro: what happened to its Mesa
   On an M3 MacBook Air, follow case D.

   A. apple,j516s + apple,t6030 (MacBook Pro 16" M3 Pro): SUPPORTED, handoff on.
      The plain one-liner installs the kernel and replaces m1n1 with one
      that hands the built-in display and the GPU over to Linux. While it
      runs it prints:
        M3 Pro (j516s, macOS 14.8.3 stub): installing m1n1 with the display and GPU handoff
      then the macOS steps to put the old boot loader back (it is kept as
      m1n1/boot.bin.before-<version> on the EFI partition), and at the end:
        m1n1's boot.bin is this release's m1n1 with the M3 Pro handoff switches
      Every M3 Pro also gets the M3 Pro's Mesa (mesa-m3, in /opt/mesa-m3),
      installed on its own after the kernel, and the desktop user goes into
      the render group, which the GPU now needs. The summary says:
        The M3 Pro's Mesa (mesa-m3 <version>, in /opt/mesa-m3) is installed.
        Added <user> to the render group, ... (unless already a member)
      If it says "The M3 Pro's Mesa was NOT installed" instead (exit status
      3), quote the warning above it: the kernel install is complete, and the
      desktop renders in software until a rerun installs it. (--no-m3-mesa
      leaves the Mesa out; only use it if the owner asks.)
      Reboot with the owner watching. Expected within about a minute: the
      Omarchy logo, the boot menu, then the desktop on the built-in display
      at 3456x2234, GPU accelerated. After login, quote
        cat /run/user/$(id -u)/mesa-m3-session.state
      (the Mesa's login check: decision=active, or the reason it stayed off,
      such as user-setup when the owner has a Mesa of their own). Then run
      the checks below and report.
      External displays: try each USB-C port and the HDMI port, one at a
      time and then HDMI with one USB-C display, and say which port, which
      display and which mode lit. Sleep has known limits on M3; if you try
      it with an external display attached, quote what happened.
      If it STOPS (m1n1 text and no boot menu, or a black screen for more
      than two minutes), follow "IF THE MAC STOPS IN m1n1" above. If it
      boots but the display or the GPU is WRONG, collect
        sudo journalctl -b -k | grep -iE 'dcp|asahi|t6030|m1n1'
      first, then follow the same section from Linux.

   B. any other apple,t6030 model (MacBook Pro 14" M3 Pro, J514S): NOT ON
      THE LIST YET. A REPORT FROM YOU IS HOW IT GETS ADDED.
      The plain one-liner installs the kernel only (boot.bin untouched, boot
      framebuffer) and says so:
        M3 Pro (j514s): installing the kernel only, and boot.bin stays as it is.
      It installs the M3 Pro's Mesa as in case A, and the summary says that
      without the handoff it stays off at login (software rendering).
      Run the checks below on that first; it is a useful report on its own.
      Then, to try the display and GPU handoff on this model:
        - Only with the owner's explicit agreement, with the owner at the
          Mac for the first boot, and only if the owner can start macOS on
          this Mac (hold the power button at startup). If the new boot
          loader does not start, macOS on the same Mac is the only way back.
        - The installer itself refuses unless the Mac has the macOS 14.8.3
          system-firmware stub that the Omarchy installer gives every M3.
        - Run:
            curl -fsSL https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh | bash -s -- --m3-handoff
          Keep the restore steps it prints (they name this Mac's EFI
          partition). It must end with "m1n1's boot.bin is this release's
          m1n1 with the M3 Pro handoff switches"; if it stops with an error
          instead, nothing was changed: report the error.
        - Reboot, with the owner watching. Then report exactly one of:
          * WORKS: the desktop on the built-in display at native resolution.
            Give the checks below, and again after one suspend/resume. Title
            it "<board> (<model>): M3 handoff works". This report is what
            adds the model to the list in the next release; until then a
            plain re-run of the one-liner keeps the handoff on this Mac.
            A release with a newer m1n1 asks again: its plain run stops
            with "run this again with --m3-handoff" and changes nothing.
          * STOPS: m1n1 text and no boot menu, or a black screen for more than
            two minutes. Follow "IF THE MAC STOPS IN m1n1" above, and report
            what the screen showed and when.
          * WRONG: it boots, but the display or the GPU is wrong. Collect
              sudo journalctl -b -k | grep -iE 'dcp|asahi|t6030|m1n1'
            over SSH if you can, then follow the same section from Linux,
            reboot, and report.

   C. apple,t8122 other than the MacBook Air (MacBook Pro 14" M3, J504;
      iMac M3, J433/J434) or apple,t6031 / apple,t6034 (M3 Max): KERNEL
      ONLY for now.
      m1n1 has no display and GPU handoff for these Macs yet, and
      --m3-handoff refuses here. The one-liner installs the kernel only and
      leaves boot.bin as it is. Expected: the desktop on the boot framebuffer
      (one fixed resolution, software rendering). Report what works and
      what does not: Wi-Fi, Bluetooth, keyboard, trackpad, audio, USB and
      Thunderbolt, suspend/resume, battery. These reports decide which chip
      gets the handoff next.
      The install ends with a NEXT STEPS block. After the reboot, check that
      "uname -r" names this release's kernel, then run the one-liner with
      --m3-report ("bash -s -- --m3-report"). It only reads, and writes one
      file in the current directory, aurora-m3-report-<board>-<date>.tgz,
      with the host name, user names, serial numbers and MAC addresses
      masked; a file that still has any is not kept. On an M3 it also
      loads the phram module for a moment, to read the boot loader's copy
      of the ADT through a read-only node, and unloads it again (one it
      did not load stays as it is). Attach the file to the issue.
      Only when the maintainer asks: --m3-power-survey (about five minutes,
      needs sudo) runs short CPU and backlight loads while it samples the
      SMC's temperature and power keys, stops early if a CPU or SoC die
      key reads 100 C, puts everything back, and writes
      aurora-m3-power-<board>-<date>.tgz.

   D. apple,j613 or apple,j615 + apple,t8122 (MacBook Air 13" or 15" M3):
      A plain J613 install keeps the ordinary display handoff and GPU
      activation policy. A plain J615 install keeps the kernel-only path;
      --m3-handoff remains its optional display/diagnostic handoff.
      A new matched bundle supports current14 GPU acceleration on both
      J613 and J615; older bundles remain J613-only.

      Experimental Air acceleration with the matched installer:
      To select the supported profile from this boot's GPU firmware:
        bash install-aurora-sep.sh --m3-gpu
      This is an explicit persistent opt-in. A diagnostics-only current14
      boot may omit the GPU descriptor; the exact stub/iBoot checks still
      apply. The system-firmware version never selects the GPU profile.
      For the command, prerequisites and recovery steps, see M3-GPU.md.
      Current14 and exact25G83 are separate profiles. On a supported Air
      already using current14 firmware, run the matched installer:
        bash install-aurora-sep.sh --m3-gpu-persistent
      This installs the matching kernel, Mesa and unified bootloader
      together, retains a reachable GPU-off previous entry, and selects
      the experimental GPU for subsequent boots. It does not require a
      firmware migration or a separate one-shot arming command. After
      reboot and normal desktop login, run:
        aurora-m3-gpu-check
      It checks actual Apple GPU OpenGL/Vulkan readback, not just packages.
      Native25 is qualified on J613. On a J613 already booted from its own exact26.6.2/25G83 volume group,
      with the source-qualified stage1 named by the matched installer:
        bash install-aurora-sep.sh --m3-profile=j613-25g83
      This selects native experimental OpenGL under /opt/mesa-m3/25g83;
      hardware Vulkan is unavailable. The installed selector is
      /etc/mesa-m3/t8122-profile=j613-25g83-hal200. Firmware and loaded
      GPU identity checks must pass. Linux14 cannot select this profile;
      neither command migrates stage1 or macOS firmware.
      J615 on 26.6.2 is EXPERIMENTAL and has not been booted on a J615.
      It needs a J615 booted from its own 26.6.2 volume group with the
      J615-capable stage1 v1.6.1-m3air25.stage1, and a matched installer
      whose release lists j615 for 25G83. Then:
        bash install-aurora-sep.sh --m3-profile=j615-25g83
      It is the same 25G83 profile plus m1n1's
      chosen.asahi,j615-25g83-experimental=1 switch; --m3-gpu and
      --m3-profile=j613-25g83 never select it on a J615. Any identity
      mismatch leaves the GPU off on the boot framebuffer. Send
      aurora-m3-gpu-check --details and --m3-report output either way.
      A refusal leaves activation unchanged. If installation fails,
      use 'Aurora previous (GPU off)' in Limine, or the retained previous
      kernel in GRUB. Quote the failure and keep the boot report.
      If it stops before the menu, follow "IF THE MAC STOPS IN m1n1" above.
      Plain later installs preserve the selected persistent profile.
      Every Air also gets mesa-m3 under /opt/mesa-m3. Ordinary J613/J615
      installs retain their existing activation behavior. The optional
      --m3-gpu-experiment installs air-gpu-oneshot.sh, air-gpu-collect.sh
      and air-gpu-job.sh for a single-boot experiment; it arms nothing
      by itself. It is separate from the persistent matched profiles.
      For that optional one-shot path, the first arming is
        sudo air-gpu-oneshot.sh t8122_pstate_cap=1
      In the armed boot, --status must say "ubootefi.var: does not name
      air-gpu-oneshot" before another knob is tried. This clearing behavior
      has been tested on an M3 Pro only.
      After the first accelerated boot, run --m3-report and record
      /run/user/$(id -u)/mesa-m3-session.state and the kernel GPU log.

   Checks for every M3 (quote the output; on a kernel-only M3 the handoff
   lines are expected to be missing, so say so):
     tr -d '\0' < /proc/device-tree/chosen/asahi,m1n1-stage2-version; echo
     ls /proc/device-tree/soc/dcp@*/apple,t6030-handoff 2>&1
     for c in /sys/class/drm/card*-*; do [ -e "$c/status" ] && echo "$c $(cat "$c/status") $(head -1 "$c/modes")"; done
     sudo dmesg | grep -E 't6030-display|\[drm\] Initialized|GPU firmware|aop.*crash|apple_sep' | head -12
     nproc
     ls /proc/device-tree/soc/usb4-pcie-tunnel-0/pcie@730000000/pci@0,0/apple,tunable 2>&1
     ls /proc/device-tree/chosen/asahi,m1n1-oslog-overlap 2>&1
     ls -d /proc/device-tree/reserved-memory/dcp-oslog@* 2>&1
   With the handoff, the first of those two must say "No such file or
   directory": if the node exists, report that before anything else. The
   second lists three dcp-oslog nodes on an M3 Pro and one on an Air.
   In the report header, set **M3 path:** to kernel, handoff, or handoff
   with --m3-handoff.

HOW TO REPORT
  Open one issue per Mac at https://github.com/omacom/linux-aurora/issues
  (not on omacom/linux#7 any more), with the first line of the report as its
  title. Post later results for the same Mac as comments on that issue.
  Structure it exactly like this:

    # <board> (<marketing name>): <one-line outcome>
    **Machine:** apple,jXXX / apple,tXXXX, <model>
    **Firmware:** asahi,os-fw-version = X, asahi,system-fw-version = Y
    **Build:** <release tag>, linux-aurora <ver>, m1n1-aurora <ver>
    **m1n1 guard:** fired / did not fire / stock m1n1 still installed
    **Touch ID:** enrol <pass/fail>, verify <pass/fail>, verify after reboot <pass/fail>
    **Thunderbolt:** <dock model, or "no dock"> on port <1 / 2 / both>
    **M3 path:** not an M3 / kernel / handoff / handoff with --m3-handoff

    ## What worked
    ## What did not
    ## Logs
    (fenced blocks, trimmed to the relevant lines - never a whole dmesg)

  Rules for the write-up:
    - Lead with the outcome, not the narrative.
    - Quote log lines exactly; do not paraphrase an error.
    - Say explicitly what you did NOT test. "No dock available" and "only
      tested port 1" are useful answers; silence is not.
    - Name the dock and the display mode. "CalDigit TS3 Plus, 3840x2160 @ 60"
      is actionable; "external display worked" is not.
    - If something failed, give the last known-good state and the first bad one.
    - Do not claim a fix works because it compiled or because the module
      loaded. Only step 3's endpoint line and step 5's verify-match count.
PROMPT
}

# A reset needs none of the kernel and boot checks; it checks for itself.
preflight_needed() { case ${1:-} in --agent-prompt | --reset-touchid | --m3-report | --m3-power-survey) return 1 ;; --m3-gpu-check) return 1 ;; --m3max-kit-runner | --m3max-kit-status | --m3max-kit-stop | --m3max-kit-restore) return 1 ;; *) return 0 ;; esac; }

# Exact stack pins are filled when the installer is assembled from its manifest.
m3_gpu_firmware_compat() {
  python3 - "$DT" <<'M3_GPU_FIRMWARE'
import pathlib
import struct
import sys

try:
    root = pathlib.Path(sys.argv[1]).resolve()
    alias = root / 'aliases/gpu'
    if not alias.exists():
        print('absent')
        sys.exit(0)
    raw = alias.read_bytes()
    if not raw.endswith(b'\0') or b'\0' in raw[:-1]:
        raise ValueError('malformed GPU alias')
    name = raw[:-1].decode('ascii')
    if not name.startswith('/') or '..' in name.split('/'):
        raise ValueError('invalid GPU alias path')
    node = (root / name.lstrip('/')).resolve()
    if not node.is_relative_to(root) or not node.is_dir():
        raise ValueError('GPU alias does not resolve inside the device tree')
    prop = node / 'apple,firmware-compat'
    if not prop.exists():
        print('absent')
    else:
        raw = prop.read_bytes()
        if len(raw) != 12:
            raise ValueError('GPU firmware compatibility must contain three cells')
        print('.'.join(map(str, struct.unpack('>III', raw))))
except (OSError, UnicodeError, ValueError) as error:
    print(str(error), file=sys.stderr)
    sys.exit(1)
M3_GPU_FIRMWARE
}

# GPU firmware is independent of the Mac's system-firmware version. An
# unarmed current14 boot can omit the GPU descriptor; its stub and iBoot
# must still pass the existing legacy checks before that profile is used.
m3_gpu_auto_profile() {
  local compat problem
  [[ ($(this_board) == j613 || $(this_board) == j615) && " $M3_PERSISTENT_BOARDS " == *" $(this_board) "* && $(this_soc) == t8122 ]] ||
    die "--m3-gpu requires a matched bundle supporting this M3 MacBook Air"
  compat=$(m3_gpu_firmware_compat) || die "--m3-gpu cannot read this boot's GPU firmware description. Nothing was installed.
    Run this installer with --m3-report and include the error above."
  case $compat in
    14.8.3) M3_GPU_PROFILE=legacy ;;
    26.6.2)
      # --m3-gpu never selects the untested J615 25G83 path by itself.
      [[ $(this_board) == j613 ]] || die "--m3-gpu selects native 25G83 OpenGL on a J613 only. On a J615 it is experimental:
    with a release that lists J615 for 25G83, choose it explicitly with --m3-profile=j615-25g83.
    Nothing was installed."
      M3_GPU_PROFILE=j613-25g83 ;;
    absent)
      M3_GPU_PROFILE=legacy
      problem=$(m3_stub_problem)
      [[ -z $problem ]] || die "--m3-gpu has no GPU firmware descriptor and the supported current14 stub checks failed: $problem.
    Nothing was installed. Run this installer with --m3-report. The system-firmware version does not select a GPU profile."
      ;;
    *) die "--m3-gpu does not support GPU firmware compatibility $compat. Nothing was installed.
    Run this installer with --m3-report. This installer does not migrate macOS firmware." ;;
  esac
  say "$(this_board): selected experimental GPU profile $M3_GPU_PROFILE; bootloader and matched-package checks still apply"
}

m3_25_boot_problem() {
  local osfw
  osfw=$({ tr -d '\0' <"$DT/chosen/asahi,os-fw-version"; } 2>/dev/null) || osfw=""
  if [[ $(this_board) == j615 ]]; then
    # Experimental: both the release (M3_NATIVE25_BOARDS) and the owner (M3_25_J615) opt in.
    if ((M3_25_J615 == 0)); then
      echo "25G83 on a J615 is experimental and needs --m3-profile=j615-25g83"
    elif [[ " $M3_NATIVE25_BOARDS " != *" j615 "* ]]; then
      echo "experimental J615 25G83 needs a release that lists j615 for 25G83 (this one lists: $M3_NATIVE25_BOARDS)"
    elif [[ $(this_soc) != t8122 || $osfw != 26.6.2 ]]; then
      echo "experimental J615 25G83 requires a J615 booted from its own 26.6.2 volume group; this boot is $(this_board) / $osfw"
    fi
    return 0
  fi
  if ((M3_25_J615)); then
    echo "--m3-profile=j615-25g83 is for a J615; this boot is $(this_board) (a J613 uses --m3-profile=j613-25g83)"
    return 0
  fi
  [[ $(this_board) == j613 && $(this_soc) == t8122 && $osfw == 26.6.2 ]] ||
    echo "25G83 requires a J613 booted from its own 26.6.2 volume group; this boot is $(this_board) / $osfw"
}

# The 25G83 firmware ABI profile on a J615: experimental, behind its own switch and stage 1 list.
m3_25_j615() {
  [[ $M3_GPU_PROFILE == j613-25g83 && $(this_board) == j615 ]]
}

# The installed Mesa's session hook admits a J615 on the 25G83 profile (its experimental switch).
m3_mesa_admits_j615() {
  grep -qxF j615-experimental "$M3_MESA_NATIVE25_BOARDS" 2>/dev/null
}

# The board and the 25G83 option must agree before any firmware check.
m3_25_board_choice() {
  if ((M3_25_J615)) && [[ $(this_board) != j615 ]]; then
    die "--m3-profile=j615-25g83 is for the 15-inch M3 MacBook Air (J615), and this Mac is $(this_board).
    On a J613 use --m3-profile=j613-25g83. Nothing was installed."
  fi
  if ((M3_25_J615 == 0)) && [[ $(this_board) == j615 ]]; then
    die "--m3-profile=j613-25g83 does not select 25G83 on a J615. J615 support is experimental and
    untested: with a release that lists J615 for 25G83, choose --m3-profile=j615-25g83. Nothing was installed."
  fi
  return 0
}

m3_25_j615_warning() {
  warn "J615 native OpenGL on macOS 26.6.2 (25G83) is EXPERIMENTAL and has not been qualified on a J615.
    Only the explicit J615 profile selects this path. Firmware, board and per-Mac resource checks
    remain required. Vulkan hardware support is unavailable for this profile.
    'Aurora previous (GPU off)' stays in the boot menu. Speakers are unchanged by this profile.
    Please send aurora-m3-gpu-check --details and --m3-report output, working or not."
}

m3_persistent_preflight() {
  local boards=$M3_PERSISTENT_BOARDS s required="chosen.asahi,t8122-gpu=1 chosen.asahi,t8122-dcp=1"
  # A J615's legacy profile is pinned to its own kernel and m1n1 pair; its 25G83 profile is a
  # separate release capability, so it is checked against M3_NATIVE25_BOARDS alone.
  if m3_25_j615; then
    boards=$M3_NATIVE25_BOARDS
    required+=" chosen.asahi,j615-25g83-experimental=1"
  fi
  [[ ($(this_board) == j613 || $(this_board) == j615) && " $boards " == *" $(this_board) "* && $(this_soc) == t8122 ]] || die "persistent GPU activation requires a matched bundle supporting this M3 MacBook Air"
  [[ $M3_STACK_ID =~ ^[0-9a-f]{64}$ ]] || die "persistent GPU requires an installer assembled from an exact matched stack manifest"
  ((M3_PRO_MESA)) || die "persistent GPU activation requires matching Mesa"
  [[ $M3_MODE == handoff ]] || die "persistent GPU requires a validated bootloader handoff"
  if [[ $M3_GPU_PROFILE == j613-25g83 ]]; then
    m3_25_board_choice
    [[ -z $(m3_25_boot_problem) ]] || die "$(m3_25_boot_problem). Firmware migration is separate from a Linux package update."
  elif [[ $M3_GPU_PROFILE != legacy ]]; then
    die "unknown M3 GPU profile"
  fi
  for s in $required; do
    ! m3_air_switch_off "$s" || die "persistent GPU conflicts with an explicit owner switch-off: $s"
  done
  m3_esp_space_check
}

# The persistent route writes to the ESP: a kept copy of boot.bin (keep_bootbin_on_esp)
# next to the rebuilt one and, on Limine, the retained GPU-off UKI and the new
# linux-aurora UKI. Asahi ESPs are 500 MB and often hold snapshot UKIs too; refuse
# before anything changes rather than fail mid-transaction with ENOSPC. GRUB keeps its
# kernels in /boot, which boot_space checks.
esp_history_builtin() {
  cat <<'ESP_HISTORY_PY'
#!/usr/bin/env python3
"""Archive unreferenced EFI history while retaining boot and recovery entries."""
import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import tempfile

LIMIT = 32 * 1024 * 1024


def regular(path):
    if not stat.S_ISREG(path.lstat().st_mode):
        raise ValueError(f'not a regular file: {path}')
    return path


def checked_tree(root, *, state_metadata=False):
    root = root.absolute()
    for path in (root, *root.parents):
        if path.is_symlink():
            raise ValueError(f'symlink in directory path: {path}')
    if not root.is_dir():
        raise ValueError(f'directory missing: {root}')
    files = []
    for directory, dirs, names in os.walk(root):
        if state_metadata and Path(directory) == root:
            # Saved modules contain header symlinks; archives hold payload bytes.
            # Neither directory supplies boot-reference metadata.
            dirs[:] = [name for name in dirs
                       if not name.startswith('modules-') and name != 'esp-history']
        for name in dirs + names:
            path = Path(directory) / name
            if path.is_symlink():
                raise ValueError(f'symlink in boot metadata: {path}')
        files.extend(Path(directory) / name for name in names)
        if len(files) > 20000:
            raise ValueError('too many boot metadata files')
    return files


def digest(path):
    with regular(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def references(esp, state):
    refs = ''
    total = 0
    for path in checked_tree(esp) + (checked_tree(state, state_metadata=True)
                                    if state.exists() or state.is_symlink() else []):
        if path.suffix.lower() not in ('.conf', '.cfg', '.json', '.var', '.env') and path.name not in ('BOOTAA64.EFI', 'm1n1-good', 'm1n1-failed'):
            continue
        total += regular(path).stat().st_size
        if total > LIMIT:
            raise ValueError('boot metadata exceeds read limit')
        data = path.read_bytes()
        refs += '\n' + data.decode('utf-8', errors='ignore').lower()
        refs += '\n' + data.decode('utf-16-le', errors='ignore').lower()
    if not any((esp / p).is_file() for p in ('EFI/BOOT/limine.conf', 'boot/limine/limine.conf', 'boot/limine.conf', 'limine/limine.conf', 'limine.conf')):
        raise ValueError('Limine configuration missing; history cannot be classified')
    return refs


def inventory(esp, state):
    esp, state = esp.absolute(), state.absolute()
    refs = references(esp, state)
    groups = [[], []]
    for path in checked_tree(esp):
        relative = path.relative_to(esp)
        if len(relative.parts) == 2 and relative.parts[0] == 'm1n1' and re.fullmatch(r'boot\.bin\.before-[A-Za-z0-9._+-]+', relative.name):
            groups[0].append(path)
        elif 'limine_history' in relative.parts[:-1] and re.fullmatch(r'[A-Za-z0-9._+-]+\.efi', relative.name, re.I):
            groups[1].append(path)
    records = []
    for group in groups:
        newest = set(sorted(group, key=lambda p: (regular(p).stat().st_mtime_ns, p.name), reverse=True)[:2])
        for path in sorted(group):
            reason = 'referenced' if path.name.lower() in refs else ('recent' if path in newest else '')
            records.append({'file':str(path.relative_to(esp)), 'bytes':regular(path).stat().st_size,
                            'sha256':digest(path), 'protected':reason})
    return records


@contextlib.contextmanager
def lock(paths):
    fds = []
    try:
        for path in paths:
            path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
            fds.append(fd)
            if not stat.S_ISREG(os.fstat(fd).st_mode):
                raise ValueError('nonregular boot lock')
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield
    finally:
        for fd in reversed(fds):
            os.close(fd)


def archive(esp, state, destination, lock_paths=None):
    esp, state, destination = esp.absolute(), state.absolute(), destination.absolute()
    destination.mkdir(parents=True, exist_ok=True)
    checked_tree(destination)
    if destination.resolve().is_relative_to(esp.resolve()) or os.stat(destination).st_dev == os.stat(esp).st_dev:
        raise ValueError('archive must be on a different filesystem from the EFI partition')
    with lock(lock_paths or [Path('/run/lock/boot-partition.lock'), Path('/tmp/limine-global.lock')]):
        planned = inventory(esp, state)
        moved = []
        for item in planned:
            if item['protected']:
                continue
            source = esp / item['file']
            target = destination / (item['sha256'] + '.bin')
            if target.exists():
                if digest(target) != item['sha256']:
                    raise ValueError('archive hash collision or damaged archive')
            else:
                fd, name = tempfile.mkstemp(prefix='.esp-history-', dir=destination)
                try:
                    with os.fdopen(fd, 'wb') as stream, regular(source).open('rb') as incoming:
                        while chunk := incoming.read(1024 * 1024):
                            stream.write(chunk)
                        stream.flush()
                        os.fsync(stream.fileno())
                    if digest(Path(name)) != item['sha256']:
                        raise ValueError('history file changed during copy')
                    os.replace(name, target)
                finally:
                    if os.path.exists(name):
                        os.unlink(name)
            # A verified pre-existing archive may still have dirty data pages.
            with regular(target).open('rb') as stream:
                os.fsync(stream.fileno())
            path_key = hashlib.sha256(item['file'].encode()).hexdigest()[:16]
            receipt = destination / (item['sha256'] + '-' + path_key + '.receipt')
            entry = {'original':item['file'], 'sha256':item['sha256'], 'bytes':item['bytes'], 'archive':target.name}
            # Persist restore information before removing the FAT copy.
            fd, name = tempfile.mkstemp(prefix='.esp-receipt-', dir=destination)
            try:
                with os.fdopen(fd, 'w') as stream:
                    json.dump(entry, stream, sort_keys=True);stream.write('\n');stream.flush();os.fsync(stream.fileno())
                os.replace(name, receipt)
            finally:
                if os.path.exists(name):os.unlink(name)
            directory = os.open(destination, os.O_RDONLY | os.O_DIRECTORY)
            try:os.fsync(directory)
            finally:os.close(directory)
            # Re-read references and contents immediately before unlinking.
            current = next((x for x in inventory(esp, state) if x['file'] == item['file']), None)
            if current is None or current['protected'] or current['sha256'] != item['sha256']:
                raise ValueError('boot state changed during archive; original retained')
            source.unlink()
            directory = os.open(source.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:os.fsync(directory)
            finally:os.close(directory)
            moved.append(entry)
        return moved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('report', 'archive'))
    parser.add_argument('--esp', type=Path, required=True)
    parser.add_argument('--state', type=Path, default=Path('/var/lib/aurora-sep'))
    parser.add_argument('--archive', type=Path, default=Path('/var/lib/aurora-sep/esp-history'))
    args = parser.parse_args()
    try:
        if args.action == 'report':
            result = inventory(args.esp, args.state)
        else:
            result = archive(args.esp, args.state, args.archive)
        print(json.dumps(result, indent=2))
    except (OSError, ValueError) as error:
        parser.exit(1, f'EFI history: {error}\n')


if __name__ == '__main__':
    main()
ESP_HISTORY_PY
}

esp_history_run() {
  local target code
  [[ $(boot_chain) == limine ]] || die "EFI history archiving is available only with Limine"
  target=$(esp_bootbin) || die "could not find the mounted FAT boot partition"
  code=$(esp_history_builtin)
  $sudo python3 -c "$code" "$1" --esp "${target%/m1n1/boot.bin}" --state "$STATE" \
    --archive "$STATE/esp-history" || die "EFI history was retained where archive checks failed; retry after checking the error"
}

m3_esp_space_check() {
  local target esp uki=0 bootbin need free
  target=$(esp_bootbin) || die "could not find the mounted FAT boot partition. Nothing was installed."
  esp=${target%/m1n1/boot.bin}
  bootbin=$($sudo stat -c %s "$target") || die "could not measure boot.bin. Nothing was installed."
  [[ $bootbin =~ ^[0-9]+$ ]] || die "invalid boot.bin size. Nothing was installed."
  if [[ $(boot_chain) == limine ]]; then
    # The largest UKI there stands for the running kernel's; 100 MB when there is none.
    uki=$($sudo find "$esp" -maxdepth 3 -name '*.efi' -size +8M -printf '%s\n' 2>/dev/null | sort -n | tail -1) || die "could not measure existing EFI kernels. Nothing was installed."
    uki=${uki:-100000000}
    [[ $uki =~ ^[0-9]+$ ]] || die "invalid EFI kernel size. Nothing was installed."
  fi
  need=$(( (2 * uki + 2 * bootbin) / 1048576 + 16 ))
  free=$($sudo df -m --output=avail "$esp" 2>/dev/null | tail -1 | tr -d ' ') || free=""
  [[ $free =~ ^[0-9]+$ ]] || die "could not read the free space on $esp. Nothing was installed."
  ((free >= need)) || die "the EFI partition ($esp) has ${free} MB free and the persistent GPU route needs
    about ${need} MB (a kept boot.bin, the retained GPU-off kernel and the new one). Free space there
    first, or rerun this command with --archive-esp-history to archive unreferenced old history
    outside the EFI partition. Active entries and recent recovery backups are retained. Nothing was installed."
}

m3_install_cleanup() {
  local status=$?
  if ((NEO_GPU_PREPARED && !NEO_GPU_COMMITTED)); then neo_gpu_restore "$STATE/neo-transaction.json" || warn "could not restore Neo boot state"; fi
  if ((M3_PERSISTENT_TRANSACTION_ACTIVE)); then
    m3_persistent_transaction_rollback || warn "could not restore experimental activation settings; use the retained GPU-off entry"
    if ((M3_PERSISTENT_MAIN_ARMED)); then
      if [[ $chain == grub ]]; then
        $sudo grub-mkconfig -o /boot/grub/grub.cfg || warn "could not disarm the main entry; use the retained GPU-off entry"
      else
        (m3_persistent_keep_entry disarm) || warn "could not disarm the main entry; use the retained GPU-off entry"
      fi
    fi
  fi
  if ((FROZEN_PACKAGES)); then $sudo rm -rf "${work:-}"
  else rm -rf "${work:-}"; fi
  return "$status"
}

m3_persistent_transaction_begin() {
  local defaults=$M3_LIMINE_DEFAULTS target
  [[ $1 != grub ]] || defaults=$M3_GRUB_DEFAULTS
  target=$(esp_bootbin) || die "persistent GPU requires the mounted boot.bin"
  $sudo python3 - "$STATE/m3-persistent-transaction.json" "$defaults" "$M1N1_CONF" "$target" \
    "$M3_GPU_OPTIN" "$M3_GPU_PROFILE_FILE" "$STATE/m3-gpu-persistent" "$STATE/m3-mode" \
    "$M3_PROFILE_HOOK" "$M3_PROFILE_UPDATE" "$M3_GPU_CHECK" "$STATE/m3-gpu-check" <<'M3_TRANSACTION_BEGIN'
import base64, json, os, stat, sys
from pathlib import Path
snapshot=Path(sys.argv[1])
rows=[]
for name in sys.argv[2:]:
    path=Path(name)
    if path.exists() or path.is_symlink():
        info=path.lstat()
        if not stat.S_ISREG(info.st_mode): raise SystemExit('transaction path must be a regular file: '+name)
        rows.append(dict(path=name, data=base64.b64encode(path.read_bytes()).decode(), mode=stat.S_IMODE(info.st_mode)))
    else: rows.append(dict(path=name, data=None))
with snapshot.open('x') as out:
    json.dump(rows,out); out.flush(); os.fsync(out.fileno())
os.chmod(snapshot,0o600)
M3_TRANSACTION_BEGIN
  M3_PERSISTENT_TRANSACTION_ACTIVE=1
}

m3_persistent_transaction_rollback() {
  $sudo python3 - "$STATE/m3-persistent-transaction.json" <<'M3_TRANSACTION_ROLLBACK'
import base64, json, os, stat, sys, tempfile
from pathlib import Path
snapshot=Path(sys.argv[1])
for row in json.loads(snapshot.read_text()):
    path=Path(row['path'])
    if path.exists() or path.is_symlink():
        if not stat.S_ISREG(path.lstat().st_mode): raise SystemExit('refusing nonregular rollback path: '+str(path))
    if row['data'] is None: path.unlink(missing_ok=True)
    else:
        fd,name=tempfile.mkstemp(prefix=path.name+'.',dir=path.parent)
        try:
            with os.fdopen(fd,'wb') as out:
                out.write(base64.b64decode(row['data'])); out.flush(); os.fsync(out.fileno())
            os.chmod(name,row['mode']); os.replace(name,path)
        finally:
            if os.path.exists(name): os.unlink(name)
snapshot.unlink()
M3_TRANSACTION_ROLLBACK
  local status=$?
  ((status == 0)) || return "$status"
  M3_PERSISTENT_TRANSACTION_ACTIVE=0
}

m3_persistent_transaction_commit() {
  $sudo rm -f "$STATE/m3-persistent-transaction.json"
  M3_PERSISTENT_TRANSACTION_ACTIVE=0
  M3_PERSISTENT_MAIN_ARMED=0
}

m3_install_packages() {
  # The generated main UKI remains unarmed until the matched transaction has
  # succeeded. The custom GPU-off entry was registered before this call.
  if ((M3_GPU_PERSISTENT)) && [[ $chain == limine ]]; then m3_persistent_cmdline "$chain" 0; fi
  if ((FROZEN_PACKAGES)); then
    $sudo test -f "$FROZEN_TRANSACTION_CONFIG" || die "missing admitted frozen package transaction"
    $sudo pacman -U --config "$FROZEN_TRANSACTION_CONFIG" --noconfirm --ask 4 "${FROZEN_TRANSACTION_FILES[@]}"
    if [[ -n $M3_PRO_MESA_PACKAGE && -f $work/$(m3_pro_mesa_file) ]]; then
      M3_PRO_MESA_RESULT=installed
    fi
  else
    $sudo pacman -U --noconfirm --ask 4 "$work"/*.pkg.tar.zst
  fi
  if ((M3_PRO_PERF_SELECTED)) && [[ -f $work/$(m3_pro_mesa_file) ]]; then M3_PRO_MESA_RESULT=installed; fi
  if ((M3_GPU_PERSISTENT)); then
    M3_PRO_MESA_RESULT=installed
    if [[ $M3_GPU_PROFILE == j613-25g83 ]]; then
      [[ $(cat "$M3_MESA_NATIVE_MARKER" 2>/dev/null) == j613-25g83-gl-only ]] ||
        die "matched Mesa native profile marker is missing; activation was not published"
      ! m3_25_j615 || m3_mesa_admits_j615 ||
        die "matched Mesa does not admit the J615 to the 25G83 profile ($M3_MESA_NATIVE25_BOARDS); activation was not published"
    fi
    m3_persistent_cmdline "$chain"
    if [[ $chain == grub ]]; then
      M3_PERSISTENT_MAIN_ARMED=1
      $sudo grub-mkconfig -o /boot/grub/grub.cfg
    fi
    m3_persistent_keep_entry publish
    M3_PERSISTENT_MAIN_ARMED=1
    m3_persistent_select
  fi
}

m3_persistent_cmdline() {
  local chain=$1 path armed=${2:-1}
  if [[ $chain == grub ]]; then path=$M3_GRUB_DEFAULTS
  else path=$M3_LIMINE_DEFAULTS; fi
  [[ -f $STATE/m3-defaults.saved ]] || $sudo cp -p "$path" "$STATE/m3-defaults.saved"
  $sudo python3 - "$path" "$chain" "$armed" "$M3_LIMINE_VENDOR_CONF" "$M3_PROCFS/cmdline" <<'M3_CMDLINE_PY'
import os, re, sys, tempfile
from pathlib import Path
path, chain = Path(sys.argv[1]), sys.argv[2]
armed = sys.argv[3]
if armed not in ('0', '1'): raise SystemExit('invalid M3 activation setting')
text = path.read_text()
begin, end = '# >>> aurora-sep: M3 GPU cmdline', '# <<< aurora-sep: M3 GPU cmdline'
if text.count(begin) != text.count(end) or text.count(begin) > 1:
    raise SystemExit('malformed owned M3 cmdline block')
text = re.sub(re.escape(begin) + r'\n.*?' + re.escape(end) + r'\n?', '', text, flags=re.S)
if chain == 'grub':
    setting = 'GRUB_CMDLINE_LINUX="${GRUB_CMDLINE_LINUX} asahi.t8122_start=' + armed + '"'
else:
    # Limine reads literal assignments; it does not expand shell variables.
    values = {}
    def read_config(data):
        for line in data.splitlines():
            line = line.strip()
            if not line or line.startswith('#'): continue
            assignment = re.match(r'^KERNEL_CMDLINE(?:\[[^\]]*\])?\s*(?:\+=|=)(.*)$', line)
            if assignment and '+=' in assignment[1]:
                raise SystemExit('Limine cannot preserve += inside command-line arguments; fix ' + str(path) + ' before retrying')
            if '+=' in line: key, value = line.split('+=', 1); append = True
            elif '=' in line: key, value = line.split('=', 1); append = False
            else: continue
            key, value = key.strip(), value.strip()
            if key == 'KERNEL_CMDLINE': target = 'default'
            elif key.startswith('KERNEL_CMDLINE[') and ']' in key:
                target = key[len('KERNEL_CMDLINE['):key.index(']')].strip()
            else: continue
            if len(target) >= 2 and target.startswith('"') and target.endswith('"'): target = target[1:-1]
            target = target or 'default'
            if len(value) >= 2 and value.startswith('"') and value.endswith('"'): value = value[1:-1]
            values[target] = value + ' ' + values[target] if append and target in values else value
    etc = path.parent.parent
    vendor = Path(sys.argv[4])
    sources = sorted(vendor.glob('*.conf')) if vendor.is_dir() else []
    sources += [etc / 'limine-entry-tool.conf']
    directory = etc / 'limine-entry-tool.d'
    if directory.is_dir(): sources += sorted(directory.glob('*.conf'))
    for source in sources:
        if source.is_file(): read_config(source.read_text())
    read_config(text)
    value = values.get('linux-aurora', '').strip() or values.get('default', '').strip()
    if not value:
        fallback = etc / 'kernel/cmdline'
        value = fallback.read_text().replace('\n', ' ').strip() if fallback.is_file() else ''
        if not value: value = Path(sys.argv[5]).read_text().strip()
    if any(c in value for c in ('"', "'", '\n', '\r', '`', '${', '$(', '+=')) or not any(w.startswith('root=') for w in value.split()):
        raise SystemExit('Limine command line must contain literal root= arguments without quotes, shell expressions or += inside arguments; fix ' + str(path) + ' before retrying')
    words = [w for w in value.split() if not re.match(r'asahi\.t8122[_-]start(?:=|$)', w)
             and not w.startswith(('mesa_m3=', 'air_gpu.oneshot=', 'BOOT_IMAGE='))]
    words += ['asahi.t8122_start=' + armed]
    if armed == '0': words += ['mesa_m3=off']
    setting = 'KERNEL_CMDLINE[linux-aurora]="' + ' '.join(words) + '"'
data = text.rstrip() + '\n\n' + begin + '\n' + setting + '\n' + end + '\n'
fd, name = tempfile.mkstemp(dir=path.parent, prefix=path.name+'.')
with os.fdopen(fd, 'w') as out:
    out.write(data); out.flush(); os.fsync(out.fileno())
os.chmod(name, 0o644)
os.replace(name, path)
M3_CMDLINE_PY
}

m3_persistent_package_check() {
  local package=$1 marker
  [[ -f $package ]] || die "matched Mesa package is missing"
  if [[ $M3_GPU_PROFILE == j613-25g83 ]]; then
    marker=$(bsdtar -xOf "$package" opt/mesa-m3/25g83/share/mesa-m3/profile 2>/dev/null) ||
      die "matched Mesa package has no native profile marker"
    [[ $marker == j613-25g83-gl-only ]] || die "matched Mesa package native profile marker differs"
    bsdtar -tf "$package" | grep -qx 'opt/mesa-m3/libexec/mesa-m3-abi-check' ||
      die "matched Mesa package has no runtime ABI checker"
  fi
}

m3_boot_profile_install() {
  $sudo install -d -m 0755 "${M3_BOOT_PROFILE_HELPER%/*}"
  $sudo tee "$M3_BOOT_PROFILE_HELPER" >/dev/null <<'M3_BOOT_PROFILE_PY'
#!/usr/bin/env python3
"""Retain the running Limine UKI and publish bounded experimental activation."""
import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import tempfile
import time

BEGIN = '# >>> aurora-sep: persistent experimental M3 GPU'
END = '# <<< aurora-sep: persistent experimental M3 GPU'
CONF_PATHS = ('EFI/BOOT/limine.conf', 'boot/limine/limine.conf', 'boot/limine.conf', 'limine/limine.conf', 'limine.conf')
FALLBACK = 'Aurora previous (GPU off)'

def regular(path):
    if not stat.S_ISREG(path.lstat().st_mode):
        raise ValueError(f'{path} must be a regular file')
    return path

def atomic(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() or path.is_symlink(): regular(path)
    fd, name = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as out:
            out.write(data); out.flush(); os.fsync(out.fileno())
        os.chmod(name, 0o644)
        os.replace(name, path)
        dfd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try: os.fsync(dfd)
        finally: os.close(dfd)
    finally:
        if os.path.exists(name): os.unlink(name)

@contextlib.contextmanager
def locks(paths, timeout=30.0):
    deadline = time.monotonic() + timeout
    fds = []
    try:
        for path in paths:
            path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
            fds.append(fd)
            if not stat.S_ISREG(os.fstat(fd).st_mode): raise ValueError('nonregular boot lock')
            while True:
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f'boot partition locks remained busy for {timeout:g} seconds; retry this installer') from None
                    time.sleep(min(0.05, remaining))
        yield
    finally:
        for fd in reversed(fds): os.close(fd)

def cmdline(line, armed):
    if '"' in line or "'" in line or not any(w.startswith('root=') for w in line.split()):
        raise ValueError('boot command line must contain root= and have no quotes')
    words = [w for w in line.split() if not re.match(r'asahi\.t8122[_-]start(?:=|$)', w)
             and not w.startswith('mesa_m3=') and not w.startswith('air_gpu.oneshot=')]
    return ' '.join(words + ['asahi.t8122_start=' + ('1' if armed else '0')] + ([] if armed else ['mesa_m3=off']))

def without_block(text):
    if text.count(BEGIN) != text.count(END) or text.count(BEGIN) > 1:
        raise ValueError('malformed persistent boot block')
    return re.sub(r'\n?' + re.escape(BEGIN) + r'\n.*?' + re.escape(END) + r'\n?', '\n', text, flags=re.S)

def entry(text, kernel, depth=2):
    lines = text.splitlines(keepends=True)
    matches = []
    for i, line in enumerate(lines):
        if line.strip() != '/' * depth + kernel: continue
        end = i + 1
        while end < len(lines) and not lines[end].lstrip().startswith('/'): end += 1
        fields = {}
        for j in range(i + 1, end):
            m = re.match(r'\s*(protocol|path|cmdline):\s*(.*?)\s*$', lines[j])
            if m:
                if m[1] in fields: raise ValueError('duplicate entry field')
                fields[m[1]] = (j, m[2])
        matches.append(fields)
    if len(matches) != 1 or any(k not in matches[0] for k in ('protocol', 'path', 'cmdline')):
        raise ValueError(f'exactly one complete {"/" * depth}{kernel} entry is required')
    if matches[0]['protocol'][1] != 'efi': raise ValueError('only EFI UKI entries are supported')
    return lines, matches[0]

def uki_path(esp, value):
    m = re.fullmatch(r'boot\(\):(/[^#]+)(?:#([0-9a-f]{128}))?', value)
    if not m or '..' in Path(m[1]).parts: raise ValueError('unsupported UKI path')
    path = esp / m[1].lstrip('/')
    if not path.resolve().is_relative_to(esp.resolve()): raise ValueError('UKI escapes ESP')
    for component in (path, *path.parents):
        if component == esp.parent: break
        if component.is_symlink(): raise ValueError('symlink in UKI path')
    data = regular(path).read_bytes()
    digest = hashlib.blake2b(data).hexdigest()
    if m[2] and m[2] != digest: raise ValueError('UKI hash pin differs')
    return data, digest

def same_esp_path(left, right):
    # VFAT names ignore ASCII case and trailing dots in each component.
    key = lambda path: tuple(part.rstrip('.').lower() for part in path.resolve().parts)
    if key(left) == key(right): return True
    try:
        return os.path.samestat(left.stat(), right.stat())
    except FileNotFoundError:
        return False

def limine(args):
    with locks(args.lock, args.lock_timeout):
        loader = regular(args.esp / 'EFI/BOOT/BOOTAA64.EFI').read_bytes()
        if b'limine.conf' not in loader: raise ValueError('EFI loader is not Limine')
        sig = b'++CONFIG_B2SUM_SIGNATURE++'
        if sig in loader:
            value = loader.split(sig, 1)[1][:128]
            if value != b'0' * 128: raise ValueError('Limine configuration hash is enrolled')
        defaults = regular(args.defaults).read_text()
        if re.search(r'^\s*ENABLE_ENROLL_LIMINE_CONFIG=[\"\x27]?yes', defaults, re.M):
            raise ValueError('Limine configuration enrollment is enabled')
        conf = next((args.esp / p for p in CONF_PATHS if (args.esp / p).is_file()), None)
        if conf is None: raise ValueError('Limine configuration missing')
        text = regular(conf).read_text()
        clean = without_block(text)
        state_path = args.state / 'm3-known-entry.json'
        if args.action in ('check-remove', 'remove'):
            output = clean
            retained = None
            has_entry = re.search(r'^\s*/' + re.escape(FALLBACK) + r'\s*$', clean, re.M)
            if state_path.exists() or state_path.is_symlink():
                saved = json.loads(regular(state_path).read_text())
                match = re.fullmatch(r'boot\(\):(/EFI/Linux/aurora-m3-previous-[0-9a-f]{16}\.efi)#([0-9a-f]{128})', saved['path'])
                if not match or not match[1].endswith(match[2][:16] + '.efi'):
                    raise ValueError('retained UKI name and hash pin differ')
                retained = args.esp / match[1].lstrip('/')
                # An edited or snapshot entry must not lose its kernel or modules.
                if has_entry:
                    lines, fields = entry(clean, FALLBACK, 1)
                    if fields['path'][1] != saved['path'] or fields['cmdline'][1] != saved['cmdline']:
                        raise ValueError('custom fallback differs from retained entry')
                    start = next(i for i, line in enumerate(lines) if line.strip() == '/' + FALLBACK)
                    end = start + 1
                    while end < len(lines) and not lines[end].lstrip().startswith('/'): end += 1
                    output = ''.join(lines[:start] + lines[end:]).rstrip() + '\n'
                for value in re.findall(r'^\s*path:\s*(\S+)', output, re.M | re.I):
                    reference = re.fullmatch(r'boot\(\):(/[^#]+)(?:#[0-9a-fA-F]{128})?', value)
                    if reference and same_esp_path(args.esp / reference[1].lstrip('/'), retained):
                        raise ValueError('another boot entry still uses the retained UKI; remove that entry first')
                if retained.exists() or retained.is_symlink():
                    uki_path(args.esp, saved['path'])
                elif has_entry:
                    raise ValueError('registered fallback UKI is missing')
                else:
                    retained = None
            else:
                owned_reference = any(re.fullmatch(r'aurora-m3-previous-[0-9a-f]{16}\.efi',
                    Path(value.split('#', 1)[0]).name.rstrip('.').lower())
                    for value in re.findall(r'^\s*path:\s*boot\(\):(/\S+)', clean, re.M | re.I))
                if has_entry or owned_reference:
                    raise ValueError('custom fallback has no saved ownership record')
            if regular(conf).read_text() != text: raise ValueError('Limine configuration changed during remove')
            if args.action == 'remove':
                atomic(conf, output.encode())
                if retained is not None: retained.unlink()
            return
        args.state.mkdir(parents=True, exist_ok=True)
        if args.action == 'retain':
            if state_path.exists() or state_path.is_symlink():
                saved = json.loads(regular(state_path).read_text())
                uki_path(args.esp, saved['path'])
                if not (args.state / ('modules-' + saved['release']) / 'modules.dep').is_file():
                    raise ValueError('retained modules missing')
            else:
                _, fields = entry(clean, args.kernel)
                data, digest = uki_path(args.esp, fields['path'][1])
                if not args.release or args.release.encode() + b'\0' not in data:
                    raise ValueError('UKI does not identify the running kernel')
                modules = args.modules / args.release
                if regular(modules / 'pkgbase').read_text().strip() != args.kernel:
                    raise ValueError('running modules do not match boot entry package')
                regular(modules / 'modules.dep')
                dest = args.state / ('modules-' + args.release)
                if dest.exists(): raise ValueError('unrecorded fallback modules exist')
                shutil.copytree(modules, dest, symlinks=True)
                shutil.rmtree(dest / 'dtbs', ignore_errors=True)
                rel = '/EFI/Linux/aurora-m3-previous-' + digest[:16] + '.efi'
                atomic(args.esp / rel.lstrip('/'), data)
                saved = dict(path='boot():' + rel + '#' + digest,
                             cmdline=cmdline(fields['cmdline'][1], False), release=args.release)
                atomic(state_path, (json.dumps(saved, sort_keys=True) + '\n').encode())
            # Publish a custom top-level EFI node under both locks. Kernel
            # regeneration owns only the OS's kernel children; the custom
            # node has no machine-id/kernel-id and survives that writer.
            # The 1.37.1 --add-efi CLI requires x86_64, so it cannot register
            # this entry on the Air. Publish the same custom menu format here.
            if re.search(r'^\s*/' + re.escape(FALLBACK) + r'\s*$', clean, re.M):
                _, fields = entry(clean, FALLBACK, 1)
                if fields['path'][1] != saved['path'] or fields['cmdline'][1] != saved['cmdline']:
                    raise ValueError('existing custom fallback differs from retained entry')
                output = clean
            else:
                output = (clean.rstrip() + '\n\n/' + FALLBACK + '\n'
                          '    # Aurora retained kernel; GPU disabled\n'
                          '    # order-priority=90\n'
                          '    protocol: efi\n    path: ' + saved['path'] + '\n'
                          '    cmdline: ' + saved['cmdline'] + '\n')
            if regular(conf).read_text() != text: raise ValueError('Limine configuration changed during retain')
            atomic(conf, output.encode())
            return
        saved = json.loads(regular(state_path).read_text())
        uki_path(args.esp, saved['path'])
        _, fallback = entry(clean, FALLBACK, 1)
        if fallback['path'][1] != saved['path'] or fallback['cmdline'][1] != saved['cmdline']:
            raise ValueError('registered GPU-off fallback differs')
        lines, fields = entry(clean, 'linux-aurora')
        # Validate the newly generated UKI before changing its activation line.
        uki_path(args.esp, fields['path'][1])
        j, old = fields['cmdline']
        lines[j] = re.match(r'\s*', lines[j])[0] + 'cmdline: ' + cmdline(old, args.action == 'publish') + '\n'
        output = ''.join(lines).rstrip() + '\n'
        if regular(conf).read_text() != text: raise ValueError('Limine configuration changed during update')
        atomic(conf, output.encode())

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=('retain', 'publish', 'disarm', 'check-remove', 'remove'))
    p.add_argument('--esp', type=Path, required=True)
    p.add_argument('--state', type=Path, required=True)
    p.add_argument('--defaults', type=Path, default=Path('/etc/default/limine'))
    p.add_argument('--kernel', default='linux-aurora')
    p.add_argument('--release', default=os.uname().release)
    p.add_argument('--modules', type=Path, default=Path('/usr/lib/modules'))
    p.add_argument('--lock', type=Path, action='append')
    p.add_argument('--lock-timeout', type=float, default=30.0, help='total boot-lock wait, in seconds (0..30)')
    args = p.parse_args()
    if not 0 <= args.lock_timeout <= 30:
        p.error('lock timeout must be between 0 and 30 seconds')
    args.lock = args.lock or [Path('/run/lock/boot-partition.lock'), Path('/tmp/limine-global.lock')]
    try: limine(args)
    except (OSError, ValueError, KeyError) as e: p.exit(1, f'M3 boot profile refused: {e}\n')

if __name__ == '__main__': main()
M3_BOOT_PROFILE_PY
  $sudo chmod 0755 "$M3_BOOT_PROFILE_HELPER"
}

m3_limine_cleanup_needed() {
  local target conf status
  [[ ! -e $STATE/m3-known-entry.json && ! -L $STATE/m3-known-entry.json ]] || return 0
  target=$(esp_bootbin) || return 1
  for conf in EFI/BOOT/limine.conf boot/limine/limine.conf boot/limine.conf limine/limine.conf limine.conf; do
    conf=${target%/m1n1/boot.bin}/$conf
    [[ -e $conf || -L $conf ]] || continue
    status=0
    $sudo grep -qiE '^[[:space:]]*/Aurora previous \(GPU off\)[[:space:]]*$|^[[:space:]]*path:[[:space:]]*boot\(\):/[^[:space:]]*aurora-m3-previous-[[:xdigit:]]{16}\.efi\.*([#][^[:space:]]*)?[[:space:]]*$' "$conf" || status=$?
    ((status <= 1)) || die "could not inspect the Limine recovery entry; nothing was uninstalled"
    return "$status"
  done
  return 1
}

m3_limine_remove_check() {
  local target
  target=$(esp_bootbin) || die "cannot check the retained Limine entry without its mounted FAT boot partition; recovery state was kept"
  m3_boot_profile_install
  $sudo python3 "$M3_BOOT_PROFILE_HELPER" "${1:-check-remove}" --esp "${target%/m1n1/boot.bin}" --state "$STATE" ||
    die "could not safely remove the retained Limine entry; its recovery state and modules were kept"
  if [[ ${1:-} == remove ]]; then $sudo rm -f "$M3_BOOT_PROFILE_HELPER"; fi
}

m3_keep_limine_entry() {
  local target esp release
  target=$(esp_bootbin) || die "persistent GPU requires the mounted FAT boot partition"
  esp=${target%/m1n1/boot.bin}
  m3_boot_profile_install
  $sudo python3 "$M3_BOOT_PROFILE_HELPER" "$1" --esp "$esp" --state "$STATE" --kernel "${kernel:-linux-aurora}" ||
    die "could not retain/publish the checked GPU-off Limine entry"
  if [[ $1 == retain ]]; then
    release=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["release"])' "$STATE/m3-known-entry.json")
    m3_fallback_modules_service "$release"
  fi
}

m3_fallback_modules_service() {
  local release=$1
  [[ $release =~ ^[A-Za-z0-9._+-]+$ ]] || die "invalid fallback kernel release"
  $sudo tee /etc/systemd/system/aurora-sep-fallback-modules.service >/dev/null <<EOF
[Unit]
Description=Restore modules of the retained Aurora GPU-off kernel
DefaultDependencies=no
ConditionKernelVersion=$release
ConditionPathExists=!/usr/lib/modules/$release/modules.dep
After=systemd-remount-fs.service
Before=systemd-modules-load.service systemd-udevd.service systemd-udev-trigger.service linux-modules-cleanup.service
[Service]
Type=oneshot
ExecStart=/usr/bin/cp -a $STATE/modules-$release /usr/lib/modules/$release
[Install]
WantedBy=sysinit.target
EOF
  $sudo systemctl daemon-reload
  $sudo systemctl enable aurora-sep-fallback-modules.service
}

m3_grub_fallback_off() {
  $sudo python3 - "$1" <<'M3_GRUB_OFF'
from pathlib import Path
import re, sys
path=Path(sys.argv[1]);text=path.read_text()
lines=text.splitlines(keepends=True)
indices=[i for i,l in enumerate(lines) if re.match(r'\s*linux\s+\S*vmlinuz-aurora-sep-previous\s',l)]
if len(indices)!=1: raise SystemExit('retained GRUB entry is missing its unique previous kernel line')
i=indices[0]
line=re.sub(r'(^| )asahi\.t8122[_-]start(?:=[^ ]*)?(?= |$)',r'\1',lines[i].rstrip())
line=re.sub(r'(^| )mesa_m3=[^ ]*',r'\1',line)
lines[i]=line+' asahi.t8122_start=0 mesa_m3=off\n'
path.write_text(''.join(lines))
M3_GRUB_OFF
}

m3_persistent_keep_entry() {
  if [[ $chain == grub ]]; then
    [[ -f /boot/vmlinuz-aurora-sep-previous && -f /boot/initramfs-aurora-sep-previous.img ]] ||
      die "persistent GPU requires the retained previous GRUB kernel entry"
    m3_grub_fallback_off /etc/grub.d/42_aurora_sep_previous
  else
    m3_keep_limine_entry "${1:-retain}"
  fi
}

m3_persistent_cmdline_remove() {
  local path owned=0
  for path in "$M3_GRUB_DEFAULTS" "$M3_LIMINE_DEFAULTS"; do
    if [[ -f $path ]] && grep -qF '# >>> aurora-sep: M3 GPU cmdline' "$path"; then owned=1; fi
  done
  ((owned)) || return 0
  $sudo python3 - "$M3_GRUB_DEFAULTS" "$M3_LIMINE_DEFAULTS" <<'M3_REMOVE_CMDLINE'
from pathlib import Path
import re, sys
begin, end = '# >>> aurora-sep: M3 GPU cmdline', '# <<< aurora-sep: M3 GPU cmdline'
for name in sys.argv[1:]:
    path=Path(name)
    if not path.is_file(): continue
    text=path.read_text()
    if begin not in text: continue
    if text.count(begin)!=1 or text.count(end)!=1: raise SystemExit('malformed owned M3 cmdline block')
    path.write_text(re.sub(re.escape(begin)+r'\n.*?'+re.escape(end)+r'\n?','',text,flags=re.S))
M3_REMOVE_CMDLINE
}

m3_persistent_remove() {
  if [[ -f $STATE/m3-persistent-transaction.json ]]; then m3_persistent_transaction_rollback; fi
  # An interrupted first install may own a defaults block without having
  # reached the final profile-state write. Remove that block independently.
  m3_persistent_cmdline_remove
  [[ -f $STATE/m3-gpu-persistent ]] || return 0
  $sudo rm -f "$M3_GPU_OPTIN" "$M3_GPU_PROFILE_FILE" "$M3_PROFILE_HOOK" "$M3_PROFILE_UPDATE" "$M3_BOOT_PROFILE_HELPER"
  $sudo rm -f "$STATE/m3-gpu-persistent"
}

m3_persistent_hook() {
  $sudo install -d -m 0755 "${M3_PROFILE_HOOK%/*}" "${M3_PROFILE_UPDATE%/*}"
  $sudo tee "$M3_PROFILE_HOOK" >/dev/null <<EOF
[Trigger]
Operation = Install
Operation = Upgrade
Type = Package
Target = linux-aurora
Target = limine-mkinitcpio-hook
[Action]
Description = Retain the Aurora GPU-off boot entry
When = PostTransaction
Exec = $M3_PROFILE_UPDATE
EOF
  $sudo tee "$M3_PROFILE_UPDATE" >/dev/null <<EOF
#!/bin/bash
set -eu
[[ -f $STATE/m3-gpu-persistent ]] || exit 0
for esp in /boot/efi /boot; do
  [[ -f \$esp/m1n1/boot.bin && \$(findmnt -no FSTYPE --target "\$esp") == vfat ]] || continue
  exec python3 $M3_BOOT_PROFILE_HELPER publish --esp "\$esp" --state $STATE
done
exit 1
EOF
  $sudo chmod 0755 "$M3_PROFILE_UPDATE"
}

m3_persistent_select() {
  local dir=${M3_GPU_OPTIN%/*}
  if [[ $M3_GPU_PROFILE == j613-25g83 ]]; then
    [[ $(cat "$M3_MESA_NATIVE_MARKER" 2>/dev/null) == j613-25g83-gl-only ]] ||
      die "matched Mesa native profile marker is missing; experimental intent remains unchanged"
    ! m3_25_j615 || m3_mesa_admits_j615 ||
      die "matched Mesa does not admit the J615 to the 25G83 profile ($M3_MESA_NATIVE25_BOARDS); experimental intent remains unchanged"
  fi
  $sudo install -d -m 0755 "$dir"
  if [[ $M3_GPU_PROFILE == j613-25g83 ]]; then
    printf '%s\n' "$M3_PROFILE_SELECTOR" | $sudo tee "$M3_GPU_PROFILE_FILE" >/dev/null
  else
    $sudo rm -f "$M3_GPU_PROFILE_FILE"
  fi
  printf '1\n' | $sudo tee "$M3_GPU_OPTIN" >/dev/null
  printf '%s\n' "$M3_GPU_PROFILE" | $sudo tee "$STATE/m3-gpu-persistent" >/dev/null
  if [[ ${chain:-} == limine ]]; then m3_persistent_hook; fi
  $sudo chmod 0644 "$M3_GPU_OPTIN" "$STATE/m3-gpu-persistent"
  [[ ! -f $M3_GPU_PROFILE_FILE ]] || $sudo chmod 0644 "$M3_GPU_PROFILE_FILE"
}

# The desktop GPU checker is owned by its recorded content hash.
m3_gpu_check_plan() {
  local recorded="" current=""
  if [[ -e $M3_GPU_CHECK || -L $M3_GPU_CHECK ]]; then
    [[ -f $M3_GPU_CHECK && ! -L $M3_GPU_CHECK ]] || die "$M3_GPU_CHECK is not an installer-owned regular file; it was left unchanged"
    [[ -f $STATE/m3-gpu-check && ! -L $STATE/m3-gpu-check ]] || die "$M3_GPU_CHECK already exists without an ownership record; it was left unchanged"
    # An explicit matching selection can restore an empty helper/record pair.
    if [[ ! -s $M3_GPU_CHECK && ! -s $STATE/m3-gpu-check && $M3_GPU_PERSISTENT == 1 &&
          -f $STATE/m3-gpu-persistent && ! -L $STATE/m3-gpu-persistent ]]; then
      case $M3_GPU_PROFILE in
        legacy|j613-25g83)
          if [[ $(cat "$STATE/m3-gpu-persistent") == "$M3_GPU_PROFILE" ]]; then
            say "Restoring the empty GPU checker and ownership record for this selected profile"
            return 0
          fi
          ;;
      esac
    fi
    recorded=$(cat "$STATE/m3-gpu-check")
    current=$(sha256sum "$M3_GPU_CHECK" | cut -d' ' -f1)
    [[ $recorded =~ ^[0-9a-f]{64}$ && $current == "$recorded" ]] || die "$M3_GPU_CHECK was changed outside this installer; it was left unchanged"
  elif [[ -e $STATE/m3-gpu-check || -L $STATE/m3-gpu-check ]]; then
    [[ -f $STATE/m3-gpu-check && ! -L $STATE/m3-gpu-check ]] || die "GPU check ownership record is not a regular file"
    [[ $(cat "$STATE/m3-gpu-check") =~ ^[0-9a-f]{64}$ ]] || die "GPU check ownership record is invalid"
  fi
}

m3_gpu_check_install() {
  local sha
  m3_gpu_check_plan
  m3_gpu_check_builtin >"$work/aurora-m3-gpu-check"
  sha=$(sha256sum "$work/aurora-m3-gpu-check" | cut -d' ' -f1)
  $sudo python3 - "$work/aurora-m3-gpu-check" "$M3_GPU_CHECK" "$STATE/m3-gpu-check" "$sha" <<'M3_CHECK_INSTALL_PY'
import hashlib, os, stat, sys, tempfile
from pathlib import Path
source, helper, record = map(Path, sys.argv[1:4])
data = source.read_bytes()
if not data or hashlib.sha256(data).hexdigest() != sys.argv[4]:
    raise SystemExit('GPU checker source is empty or changed')
compile(data, str(helper), 'exec')
staged = []
try:
    for path, content, mode in ((helper, data, 0o755),
                               (record, (sys.argv[4] + '\n').encode(), 0o644)):
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists() or path.is_symlink():
            if not stat.S_ISREG(path.lstat().st_mode):
                raise SystemExit('GPU checker destination is not a regular file: ' + str(path))
        fd, name = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
        staged.append((name, path))
        with os.fdopen(fd, 'wb') as output:
            os.fchmod(output.fileno(), mode)
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
    # Publish complete files; persist the helper before its ownership record.
    for name, path in staged:
        os.replace(name, path)
        fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
finally:
    for name, path in staged:
        if os.path.exists(name):
            os.unlink(name)
M3_CHECK_INSTALL_PY
}

m3_gpu_check_remove() {
  [[ -f $STATE/m3-gpu-check && ! -L $STATE/m3-gpu-check ]] || return 0
  if [[ -e $M3_GPU_CHECK || -L $M3_GPU_CHECK ]]; then
    if [[ ! -f $M3_GPU_CHECK || -L $M3_GPU_CHECK ||
          $(sha256sum "$M3_GPU_CHECK" | cut -d' ' -f1) != "$(cat "$STATE/m3-gpu-check")" ]]; then
      warn "Keeping $M3_GPU_CHECK: it was changed outside this installer"
      return 0
    fi
    $sudo rm -f "$M3_GPU_CHECK"
  fi
  $sudo rm -f "$STATE/m3-gpu-check"
}

m3_gpu_check_run() {
  ((EUID != 0)) || die "run --m3-gpu-check in your desktop terminal, without sudo"
  local code
  code=$(m3_gpu_check_builtin)
  if [[ -n ${MESA_M3_HOOK_RUN_ID:-} ]]; then
    python3 -c "$code" --session "$@"
  else
    systemd-run --user --pipe --wait --collect --quiet --slice=app-graphical.slice \
      python3 -c "$code" --session "$@"
  fi
}

m3_gpu_check_builtin() {
  cat <<'AURORA_M3_GPU_CHECK_PY'
#!/usr/bin/python3
import os
from pathlib import Path
import re
import stat
import subprocess
import sys


class CheckError(Exception):
    def __init__(self, message, remedy, details=''):
        super().__init__(message)
        self.remedy = remedy
        self.details = details


def reject(message, remedy='Run the matched installer with --m3-report and include this error.', details=''):
    raise CheckError(message, remedy, details)


def fields(text):
    values = {}
    for line in text.splitlines():
        if '=' not in line:
            reject('The GPU session record is incomplete.', 'Log out and log in again, then retry.')
        key, value = line.split('=', 1)
        if key in values:
            reject('The GPU session record has duplicate fields.', 'Log out and log in again, then retry.')
        values[key] = value
    return values


def verify(root, env, uid, run=subprocess.run):
    if uid == 0:
        reject('The GPU check needs your desktop session.', 'Run aurora-m3-gpu-check without sudo after login.')
    state_path = root / f'run/user/{uid}/mesa-m3-session.state'
    try:
        info = state_path.lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_uid != uid or info.st_size > 16384:
            raise ValueError('invalid session record')
        state = fields(state_path.read_text())
    except (OSError, ValueError):
        reject('No current GPU session was found.', 'Reboot and log into your desktop, then retry.')
    if state.get('schema') != 'aurora.mesa-m3-session/2':
        reject('The GPU session record is unsupported.', 'Install the matched Mesa package and log in again.')
    if state.get('decision') != 'active' or state.get('reason') != 'active':
        reason = state.get('reason', 'unknown')
        remedy = {
            'experimental': 'Run the matched installer with --m3-gpu, then reboot and log in.',
            'no-access': 'Log out and log in again so render-group membership takes effect.',
            'no-gpu': "Check this boot's GPU startup: journalctl -b -k -g 'asahi|M3 G15G'; report the error with --m3-report in issue #35.",
            'opt-out': 'Review your GPU opt-out setting; this check does not change it.',
            'previous-failed': 'Use the retained GPU-off entry and report the previous failed login.',
        }.get(reason, 'Run the matched installer with --m3-report and include this error.')
        reject(f'Apple GPU is inactive ({reason}).', remedy)
    boot = (root / 'proc/sys/kernel/random/boot_id').read_text().strip()
    if (state.get('uid') != str(uid) or state.get('boot_id') != boot or
            not state.get('hook_run_id') or state['hook_run_id'] != env.get('MESA_M3_HOOK_RUN_ID') or
            (env.get('XDG_SESSION_ID') and state.get('session_id') != env['XDG_SESSION_ID'])):
        reject('The GPU session does not match this login.', 'Log out and log in again, then retry.')
    selector_path = root / 'etc/mesa-m3/t8122-profile'
    selector = selector_path.read_text().rstrip('\n') if selector_path.exists() else ''
    native = selector == 'j613-25g83-hal200'
    if selector not in ('', 'j613-25g83-hal200'):
        reject('The installed GPU profile is unknown.')
    prefix = '/opt/mesa-m3/25g83' if native else '/opt/mesa-m3'
    profile = selector if native else 'legacy'
    if state.get('prefix') != prefix or env.get('MESA_M3_SESSION') != prefix:
        reject('The desktop and installed GPU profile differ.', 'Reboot and log in again after the matched installation.')
    if state.get('fallback') != 'none' or env.get('MESA_M3_FALLBACK') or env.get('LIBGL_ALWAYS_SOFTWARE', '0') not in ('', '0'):
        reject('This login is using software rendering.', 'Reboot and log in again; if unchanged, run --m3-report.')
    compatible = (root / 'proc/device-tree/compatible').read_bytes().split(b'\0')
    soc = 't8122' if b'apple,t8122' in compatible else 't6030' if b'apple,t6030' in compatible else ''
    if not soc:
        reject('This Mac has no supported M3 GPU profile.')
    if soc == 't8122' and not (root / 'etc/mesa-m3/t8122-gpu-experiment').is_file():
        reject('The Air GPU experimental opt-in is absent.', 'Run the matched installer with --m3-gpu, then reboot and log in.')
    nodes = list((root / 'sys/class/drm').glob('renderD*'))
    if len(nodes) != 1:
        reject('The kernel has no unique M3 render node.', "Check this boot's GPU startup: journalctl -b -k -g 'asahi|M3 G15G'; report the error with --m3-report in issue #35.")
    node = nodes[0]
    of_node = node / 'device/of_node'
    if ((node / 'device/driver').resolve().name != 'asahi' or
            f'apple,agx-{soc}'.encode() not in (of_node / 'compatible').read_bytes().split(b'\0')):
        reject('The render node is not this Mac\'s Asahi GPU.')
    device = root / f'dev/dri/{node.name}'
    if not os.access(device, os.R_OK | os.W_OK):
        reject('The GPU render node is not accessible.', 'Log out and log in again so render-group membership takes effect.')
    if native:
        # The J615 runs the same 25G83 firmware ABI only behind m1n1's experimental switch.
        try:
            j615_opt_in = (root / 'proc/device-tree/chosen/asahi,j615-25g83-experimental').read_bytes() == b'1\0'
        except OSError:
            j615_opt_in = False
        j613 = b'apple,j613' in compatible
        j615 = b'apple,j615' in compatible
        board = j613 != j615 and (j613 or j615_opt_in)
        if (soc != 't8122' or not board or
                (of_node / 'apple,firmware-compat').read_bytes() != bytes.fromhex('0000001a0000000600000002') or
                (of_node / 'apple,j613-25g83-gpu-handoff').read_bytes() != bytes.fromhex('00000001') or
                (root / prefix.lstrip('/') / 'share/mesa-m3/profile').read_text().rstrip('\n') != 'j613-25g83-gl-only'):
            reject('Native OpenGL requires the exact 25G83 handoff on a J613, or J615 with the experimental opt-in.')
    elif (of_node / 'apple,j613-25g83-gpu-handoff').exists():
        reject('The native HAL200 handoff cannot use legacy Mesa.')
    abi = run([str(root / 'opt/mesa-m3/libexec/mesa-m3-abi-check'), profile, str(device)],
              capture_output=True, text=True, timeout=10)
    expected_hal = 'HAL200' if native else 'HAL0'
    if (abi.returncode or 'match=1' not in abi.stdout.split() or
            expected_hal not in abi.stdout.split() or 'USC3' not in abi.stdout.split()):
        reject('The running GPU ABI does not match the installed profile.', details=abi.stdout + abi.stderr)
    probe = run([str(root / prefix.lstrip('/') / 'bin/mesa-m3-probe')],
                capture_output=True, text=True, timeout=95)
    text = probe.stdout
    def value(key):
        matches = [line[len(key) + 1:] for line in text.splitlines() if line.startswith(key + '=')]
        if len(matches) != 1:
            reject('The GPU readback report is incomplete.', details=text + probe.stderr)
        return matches[0]
    if probe.returncode or value('result') != 'pass' or value('exit') != '0':
        reject('The Apple GPU readback did not pass.', details=text + probe.stderr)
    identities = [line for line in text.splitlines() if line.startswith('pid=')]
    if (len(identities) != 1 or not re.fullmatch(r'pid=[1-9][0-9]* starttime=[1-9][0-9]* boot_id=' + re.escape(boot), identities[0]) or
            value('env.MESA_M3_HOOK_RUN_ID') != state['hook_run_id'] or value('env.MESA_M3_SESSION') != prefix):
        reject('The readback does not match this GPU session.', details=text)
    renderer = value('gl.renderer')
    if 'apple m3' not in renderer.lower() or any(name in renderer.lower() for name in ('llvmpipe', 'softpipe', 'lavapipe', 'swrast', 'software')):
        reject('OpenGL is not using the Apple M3 GPU.', details=text)
    if native and (not renderer.startswith('Apple M3') or 'zink' in renderer.lower()):
        reject('The native profile is not using native Apple OpenGL.', details=text)
    if value('gl.platform') != 'wayland' or value('gl.swap') != 'pass':
        reject('The GPU probe could not render through your desktop.', 'Run the check after normal desktop login.', text)
    render = value('gl.render')
    if not re.fullmatch(r'pass left=255,0,0,\d+ right=0,0,255,\d+ glerror=0x0', render):
        reject('OpenGL red/blue pixel readback failed.', details=text)
    if f'node=/dev/dri/{node.name}' not in [part for line in text.splitlines() if line.startswith('fd.dri ') for part in line.split()]:
        reject('The readback process did not open the admitted GPU node.', details=text)
    maps = [dict(part.split('=', 1) for part in line.split()[1:] if '=' in part)
            for line in text.splitlines() if line.startswith('map class=implementation ')]
    if not maps or any(m.get('same_file') != '1' or m.get('deleted') != '0' or
                       not m.get('path', '').startswith(prefix + '/') or
                       (not native and m['path'].startswith('/opt/mesa-m3/25g83/')) for m in maps):
        reject('The readback loaded a different or replaced Mesa profile.', 'Reboot and log in again after the matched installation.', text)
    if native:
        if value('vk.capability') != 'unavailable profile=j613-25g83':
            reject('The native profile reported an unsupported Vulkan capability.', details=text)
        vk = 'Vulkan unavailable in the experimental 25G83 profile.'
    else:
        if ('apple m3' not in value('vk.device').lower() or value('vk.device_type') != 'integrated-gpu' or
                value('vk.job') != 'pass words=65536 wrong=0'):
            reject('Apple GPU Vulkan compute readback failed.', details=text)
        vk = 'PASS Apple GPU Vulkan (compute readback).'
    return ['PASS Apple GPU OpenGL (red/blue readback).', vk], text


def main():
    args = sys.argv[1:]
    if any(arg not in ('--session', '--details') for arg in args):
        print('usage: aurora-m3-gpu-check [--details]', file=sys.stderr)
        return 2
    if '--session' not in args and not os.environ.get('MESA_M3_HOOK_RUN_ID') and os.geteuid() != 0:
        command = ['systemd-run', '--user', '--pipe', '--wait', '--collect', '--quiet',
                   '--slice=app-graphical.slice', sys.executable, __file__, '--session']
        if '--details' in args:
            command.append('--details')
        try:
            return subprocess.run(command).returncode
        except OSError:
            print('FAIL: No desktop session launcher was found. Run this check in your desktop terminal.')
            return 1
    try:
        messages, details = verify(Path('/'), os.environ, os.geteuid())
        print('\n'.join(messages))
        if '--details' in args:
            print(details, end='')
        return 0
    except CheckError as error:
        print('FAIL: ' + str(error))
        print('Next: ' + error.remedy)
        if '--details' in args and error.details:
            print(error.details, end='')
    except (OSError, ValueError, subprocess.TimeoutExpired):
        print('FAIL: GPU check could not complete. Run --m3-report and include this error.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
AURORA_M3_GPU_CHECK_PY
}

desktop_fixes_manifest() { printf '%s' "$DESKTOP_FIXES_DATA" | base64 --decode; }
desktop_fixes_run() {
  python3 <(desktop_fixes_builtin) "$1" <(desktop_fixes_manifest) "${@:2}"
}
desktop_fixes_plan() {
  DESKTOP_FIXES_PACKAGES=""
  ((DESKTOP_FIXES)) || return 0
  [[ -n $DESKTOP_FIXES_DATA ]] || die "this release has no matched desktop-fixes manifest; nothing was installed"
  command -v python3 >/dev/null || die "--desktop-fixes requires python3; nothing was installed"
  command -v bsdtar >/dev/null || die "--desktop-fixes requires bsdtar; nothing was installed"
  DESKTOP_FIXES_PACKAGES=$(desktop_fixes_run plan) || die "desktop preflight failed; nothing was installed"
}
desktop_fixes_prepare() {
  local entry file sha
  desktop_fixes_plan
  ((DESKTOP_FIXES)) || return 0
  work=$(mktemp -d)
  trap 'm3_install_cleanup' EXIT
  while read -r file sha; do
    [[ -n $file ]] || continue
    say "Downloading $file"
    curl -fL --retry 3 --progress-bar -o "$work/$file" "$RELEASE_URL/$file" || die "desktop package download failed; nothing was installed"
    [[ $(sha256sum "$work/$file" | cut -d' ' -f1) == "$sha" ]] || die "desktop artifact checksum differs; nothing was installed"
  done < <(desktop_fixes_files)
  desktop_fixes_verify "$work"
}
desktop_fixes_files() {
  [[ -z $DESKTOP_FIXES_PACKAGES ]] || printf '%s\n' "$DESKTOP_FIXES_PACKAGES"
}
desktop_fixes_verify() {
  local checked
  ((DESKTOP_FIXES)) || return 0
  checked=$(desktop_fixes_run verify --directory "$1") || die "desktop package preflight failed; nothing was installed"
  [[ $checked == "$DESKTOP_FIXES_PACKAGES" ]] || die "desktop package selection changed during download; nothing was installed"
}
desktop_fixes_notice() {
  ((DESKTOP_FIXES)) || return 0
  if [[ $DESKTOP_FIXES_PACKAGES == *omarchy-4.0.4-2* ]]; then
    say "Desktop fixes installed. Log out or reboot to enter the guarded session; the running session keeps its existing launcher."
    say "Kernel uninstall keeps the desktop guard and authenticated login recovery installed."
  fi
}
desktop_fixes_builtin() {
  cat <<'AURORA_DESKTOP_FIXES_PY'
#!/usr/bin/env python3
"""Check optional desktop packages without changing the installed system."""
import argparse
import configparser
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import sys
import tarfile

NAMES = {'omarchy': '4.0.4-2', 'omarchy-settings': '4.0.4-2',
         'aquamarine': '0.15.1-1.3'}
PAM_LINE = 'session required pam_exec.so seteuid /usr/bin/omarchy-session-guard --pam'
HEX = re.compile(r'[0-9a-f]{64}')


def command(*args, accepted=(0,)):
    result = subprocess.run(args, text=True, capture_output=True,
                            env={**os.environ, 'LC_ALL': 'C'})
    if result.returncode not in accepted:
        raise ValueError(f'{args[0]} failed: {result.stderr.strip()}')
    if result.returncode == 127 and not result.stdout.strip():
        raise ValueError(f'{args[0]} dependency query failed without a result')
    return result.stdout


def safe_path(name):
    path = PurePosixPath(name)
    if path.is_absolute() or '..' in path.parts or str(path) != name or not path.parts:
        raise ValueError('invalid desktop payload path')
    return name


def immutable(name):
    return name.startswith('usr/bin/') or name.startswith('usr/share/omarchy/')


def validate_manifest(data):
    if data.get('schema') != 'aurora.desktop-fixes/1' or set(data.get('packages', {})) != set(NAMES):
        raise ValueError('desktop manifest must name exactly the matched three packages')
    for name, version in NAMES.items():
        item = data['packages'][name]
        if item.get('version') != version or not HEX.fullmatch(item.get('sha256', '')):
            raise ValueError('desktop package pin differs')
        if not re.fullmatch(r'[A-Za-z0-9._+:-]+\.pkg\.tar\.zst', item.get('file', '')):
            raise ValueError('desktop artifact must be a local basename')
        for path, digest in item.get('payload', {}).items():
            safe_path(path)
            if not HEX.fullmatch(digest):
                raise ValueError('desktop payload hash differs')
    catalogs = data.get('omarchy_catalogs', {})
    if set(catalogs) != {'4.0.4-1', '4.0.4-2'}:
        raise ValueError('both stable runtime catalogs are required')
    for catalog in catalogs.values():
        if not catalog.get('files') or not re.fullmatch(r'[0-9a-f]{40}', catalog.get('source', '')):
            raise ValueError('exact runtime source catalog is required')
        for path, item in catalog['files'].items():
            safe_path(path)
            if not immutable(path):
                raise ValueError('runtime catalog contains another file scope')
            if set(item) == {'sha256', 'mode'}:
                if not HEX.fullmatch(item['sha256']) or item['mode'] not in (0o644, 0o755):
                    raise ValueError('invalid runtime digest')
            elif set(item) != {'symlink'} or not isinstance(item['symlink'], str):
                raise ValueError('invalid runtime catalog entry')
    required = {
        'omarchy': ('usr/bin/omarchy-session-guard', 'usr/share/omarchy/shell/session-guard.py',
                    'usr/share/omarchy/shell/session-guard-install.py',
                    'usr/share/omarchy/shell/plugins/lock/Service.qml',
                    'usr/share/libalpm/hooks/95-omarchy-session-guard.hook'),
        'omarchy-settings': ('usr/lib/systemd/user/wayland-wm@hyprland.desktop.service.d/99-session-lock-recovery.conf',
                             'usr/local/share/wayland-sessions/omarchy.desktop',
                             'usr/local/share/wayland-sessions/omarchy-guarded-hyprland.desktop',
                             'etc/sddm.conf.d/90-session-lock-recovery.conf')}
    for name, paths in required.items():
        if not set(paths) <= set(data['packages'][name].get('payload', {})):
            raise ValueError('desktop guard, service and recovery pins are required')
    activation = {p for name in required for p in data['packages'][name]['payload'] if not immutable(p)}
    for catalog in catalogs.values():
        if set(catalog.get('activation', {})) != activation:
            raise ValueError('exact installed launcher and recovery catalog is required')
        for path, digest in catalog['activation'].items():
            safe_path(path)
            if digest is not None and not HEX.fullmatch(digest):
                raise ValueError('invalid installed launcher digest')


def checked_file(root, name):
    path = root / safe_path(name)
    owner = 0 if root == Path('/') else os.getuid()
    for parent in reversed(path.parents):
        if parent == root.parent:
            continue
        if parent == root or root in parent.parents:
            info = parent.lstat()
            if not stat.S_ISDIR(info.st_mode) or info.st_uid != owner or info.st_mode & 0o022:
                raise ValueError('unsafe desktop parent: ' + str(parent))
    info = path.lstat()
    if info.st_uid != owner or (not stat.S_ISLNK(info.st_mode) and info.st_mode & 0o022):
        raise ValueError('unsafe desktop file: ' + name)
    return path, info


def verify_runtime(root, catalog):
    owned = command('pacman', '-Qql', 'omarchy', 'omarchy-settings').splitlines()
    listed = set()
    for path in owned:
        name = path.lstrip('/')
        if immutable(name) and not path.endswith('/'):
            listed.add(safe_path(name))
    if listed != set(catalog['files']):
        raise ValueError('installed immutable Omarchy file list differs from stable source')
    physical = set()
    tree = root / 'usr/share/omarchy'
    for parent, directories, files in os.walk(tree, followlinks=False):
        for leaf in files + [d for d in directories if (Path(parent) / d).is_symlink()]:
            physical.add(str((Path(parent) / leaf).relative_to(root)))
    if physical != {p for p in listed if p.startswith('usr/share/omarchy/')}:
        raise ValueError('unmapped files exist in the immutable Omarchy runtime tree')
    for name, item in catalog['files'].items():
        path, info = checked_file(root, name)
        if 'symlink' in item:
            if not stat.S_ISLNK(info.st_mode) or os.readlink(path) != item['symlink']:
                raise ValueError('Omarchy symlink differs: ' + name)
        elif (not stat.S_ISREG(info.st_mode) or stat.S_IMODE(info.st_mode) != item['mode'] or
              hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']):
            raise ValueError('Omarchy runtime source differs: ' + name)
    for name, digest in catalog['activation'].items():
        path = root / name
        owner = 0 if root == Path('/') else os.getuid()
        for parent in reversed(path.parents):
            if parent == root or root in parent.parents:
                if not parent.exists() and not parent.is_symlink():
                    continue
                info = parent.lstat()
                if not stat.S_ISDIR(info.st_mode) or info.st_uid != owner or info.st_mode & 0o022:
                    raise ValueError('unsafe desktop activation parent: ' + str(parent))
        if digest is None:
            if path.exists() or path.is_symlink():
                raise ValueError('unexpected installed launcher or recovery file: ' + name)
        else:
            file, info = checked_file(root, name)
            if not stat.S_ISREG(info.st_mode) or hashlib.sha256(file.read_bytes()).hexdigest() != digest:
                raise ValueError('installed launcher or recovery source differs: ' + name)


def verify_recovery(root, current):
    pam, info = checked_file(root, 'etc/pam.d/sddm')
    if not stat.S_ISREG(info.st_mode):
        raise ValueError('SDDM PAM must be a regular file')
    if current and PAM_LINE not in pam.read_text().splitlines():
        raise ValueError('installed authenticated recovery hook is missing')
    effective = None
    paths = sorted((root / 'usr/lib/sddm/sddm.conf.d').glob('*.conf'))
    paths += sorted(set((root / 'etc/sddm.conf.d').glob('*.conf')) |
                    {root / 'etc/sddm.conf.d/90-session-lock-recovery.conf'})
    paths += [root / 'etc/sddm.conf']
    for path in paths:
        if path == root / 'etc/sddm.conf.d/90-session-lock-recovery.conf' and not current:
            if path.exists() or path.is_symlink():
                _, mode = checked_file(root, str(path.relative_to(root)))
                if not stat.S_ISREG(mode.st_mode):
                    raise ValueError('SDDM recovery config must be a regular file')
            text = '[Autologin]\nRelogin=false\n'
        elif path.exists() or path.is_symlink():
            checked, mode = checked_file(root, str(path.relative_to(root)))
            if not stat.S_ISREG(mode.st_mode):
                raise ValueError('SDDM config must be a regular file')
            text = checked.read_text()
        else:
            continue
        config = configparser.ConfigParser(interpolation=None, strict=False)
        config.read_string(text)
        if config.has_option('Autologin', 'Relogin'):
            effective = config.get('Autologin', 'Relogin').strip().lower()
    if effective != 'false':
        raise ValueError('SDDM configuration overrides authenticated recovery Relogin=false')


def plan(data, root):
    installed = {}
    for line in command('pacman', '-Q').splitlines():
        name, version = line.split()
        installed[name] = version
    selected = []
    if 'omarchy-dev' in installed or 'omarchy-settings-dev' in installed:
        raise ValueError('development Omarchy packages are not a stable upgrade baseline')
    pair = [installed.get(name) for name in ('omarchy', 'omarchy-settings')]
    if any(pair):
        if pair[0] != pair[1] or pair[0] not in ('4.0.4-1', '4.0.4-2'):
            raise ValueError('Omarchy requires the exact matching stable 4.0.4 pair; newer or unknown sources are preserved')
        verify_runtime(root, data['omarchy_catalogs'][pair[0]])
        verify_recovery(root, pair[0] == '4.0.4-2')
        if pair[0] == '4.0.4-1':
            selected += ['omarchy', 'omarchy-settings']
        else:
            for name in ('omarchy', 'omarchy-settings'):
                for path, digest in data['packages'][name]['payload'].items():
                    file, info = checked_file(root, path)
                    if not stat.S_ISREG(info.st_mode) or hashlib.sha256(file.read_bytes()).hexdigest() != digest:
                        raise ValueError('installed guarded desktop payload differs: ' + path)
            guard, _ = checked_file(root, 'usr/bin/omarchy-session-guard')
            if not os.access(guard, os.X_OK):
                raise ValueError('installed session guard is not executable')
            print('Desktop fixes: exact guarded Omarchy pair verified; no reinstall.', file=sys.stderr)
    else:
        print('Desktop fixes: Omarchy absent; desktop settings are unchanged.', file=sys.stderr)
    version = installed.get('aquamarine')
    if version in ('0.15.1-1', '0.15.1-1.1', '0.15.1-1.2'):
        metadata = command('pacman', '-Qi', 'aquamarine')
        if re.search(r'(?m)^Provides\s*:\s*.*\blibaquamarine\.so=14-64\b', metadata):
            selected.append('aquamarine')
        else:
            print('Desktop fixes: Aquamarine ABI14 is not provided; keeping the installed library.', file=sys.stderr)
    else:
        print('Desktop fixes: Aquamarine absent, already fixed or outside the supported baseline; keeping it.', file=sys.stderr)
    return selected


def archive_metadata(path):
    values = {}
    for line in command('bsdtar', '-xOf', str(path), '.PKGINFO').splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1)
            values.setdefault(key, []).append(value)
    return values


def package_contents(path):
    entries = {}
    process = subprocess.Popen(['bsdtar', '-cf', '-', '@' + str(path)],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        with tarfile.open(fileobj=process.stdout, mode='r|') as archive:
            for member in archive:
                name = member.name.removeprefix('./')
                if member.isdir():
                    continue
                safe_path(name)
                if name in entries:
                    raise ValueError('duplicate desktop archive entry: ' + name)
                if member.isfile():
                    entries[name] = {'sha256': hashlib.file_digest(archive.extractfile(member), 'sha256').hexdigest(),
                                     'mode': member.mode & 0o777}
                elif member.issym():
                    entries[name] = {'symlink': member.linkname}
                else:
                    raise ValueError('unsupported desktop archive entry: ' + name)
        if process.wait() != 0:
            raise ValueError('desktop archive listing failed')
        return entries
    finally:
        process.stdout.close()
        process.stderr.close()
        if process.poll() is None:
            process.kill()
            process.wait()


def verify_archive(name, item, directory):
    path = directory / item['file']
    if path.is_symlink() or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
        raise ValueError('desktop artifact checksum differs: ' + name)
    metadata = archive_metadata(path)
    if metadata.get('pkgname') != [name] or metadata.get('pkgver') != [NAMES[name]] or metadata.get('arch') != ['aarch64']:
        raise ValueError('desktop artifact package identity differs: ' + name)
    if name == 'omarchy' and 'omarchy-settings=4.0.4-2' not in metadata.get('depend', []):
        raise ValueError('Omarchy runtime must require the exact settings version')
    if name == 'aquamarine' and 'libaquamarine.so=14-64' not in metadata.get('provides', []):
        raise ValueError('Aquamarine artifact must provide ABI14')
    contents = package_contents(path)
    for member, digest in item.get('payload', {}).items():
        if contents.get(member, {}).get('sha256') != digest:
            raise ValueError('desktop package payload differs: ' + member)
    return metadata.get('depend', [])


def verify(data, selected, directory):
    dependencies = []
    for name in selected:
        dependencies += verify_archive(name, data['packages'][name], directory)
    dependencies = sorted(set(dependencies) - ({'omarchy-settings=4.0.4-2'} if 'omarchy-settings' in selected else set()))
    if dependencies:
        unmet = command('pacman', '-T', *dependencies, accepted=(0, 127)).strip()
        if unmet:
            raise ValueError('desktop dependencies must already be installed: ' + unmet.replace('\n', ', '))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('plan', 'verify'))
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--directory', type=Path)
    parser.add_argument('--root', type=Path, default=Path('/'))
    args = parser.parse_args()
    try:
        if not args.root.is_absolute():
            raise ValueError('desktop inspection root must be absolute')
        data = json.loads(args.manifest.read_text())
        validate_manifest(data)
        selected = plan(data, args.root)
        if args.mode == 'verify':
            if args.directory is None:
                raise ValueError('desktop package directory is required')
            verify(data, selected, args.directory)
        for name in selected:
            item = data['packages'][name]
            print(item['file'] + ' ' + item['sha256'])
    except (OSError, ValueError, KeyError, TypeError, tarfile.TarError, subprocess.CalledProcessError, configparser.Error) as error:
        print('Desktop fixes refused: ' + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
AURORA_DESKTOP_FIXES_PY
}

# A separate T8140 profile keeps the G17 ABI and bootloader out of M3 sessions.
NEO_GPU=0
NEO_GPU_PREPARED=0
NEO_GPU_COMMITTED=0
NEO_GPU_CONFIG=/etc/mesa-neo
NEO_MESA_PREFIX=/opt/mesa-neo
NEO_GPU_PROFILE=j700-g17p-hal200
NEO_MESA_PACKAGE=""
NEO_M1N1_PACKAGE=""
NEO_M1N1_BIN_SHA=""

neo_gpu_plan() {
  if is_neo && [[ -f $STATE/neo-gpu && -e $NEO_GPU_CONFIG/gpu-experiment ]] &&
     [[ $(cat "$NEO_GPU_CONFIG/profile" 2>/dev/null) == "$NEO_GPU_PROFILE" ]]; then NEO_GPU=1; fi
  ((NEO_GPU)) || return 0
  is_neo && [[ $(this_board) == j700 ]] || die "--neo-gpu requires a MacBook Neo (J700/T8140); nothing was installed"
  ((M3_GPU_PERSISTENT == 0 && M3_GPU_EXPERIMENT == 0 && M3_TRY == 0)) || die "Neo and M3 GPU options cannot be combined"
  local pin
  for pin in "$NEO_MESA_PACKAGE" "$NEO_M1N1_PACKAGE"; do
    [[ $pin =~ ^[a-zA-Z0-9._+-]+\.pkg\.tar\.zst\ [0-9a-f]{64}$ ]] || die "this installer has no frozen Neo package pair; nothing was installed"
  done
  [[ $NEO_M1N1_BIN_SHA =~ ^[0-9a-f]{64}$ ]] || die "this installer has no frozen Neo bootloader; nothing was installed"
  update_m1n1_frozen && die "update-m1n1 is disabled in $UPDATE_M1N1_CONF; keep that owner setting or remove it before --neo-gpu"
  command -v update-m1n1 >/dev/null || die "update-m1n1 is required for the Neo profile"
  esp_bootbin >/dev/null || die "the Neo's existing boot.bin must be mounted before installation"
  M1N1_PACKAGE=$NEO_M1N1_PACKAGE
  M1N1_BIN_SHA=$NEO_M1N1_BIN_SHA
  M1N1_BIN=/usr/lib/m1n1-neo/m1n1.bin
  NEO_AURORA_M1N1=1
  PINNED="$PINNED mesa-neo m1n1-neo"
  say "Installing the matched Neo native OpenGL/Honeykrisp Vulkan profile; preserving this Neo's U-Boot and firmware"
}

neo_gpu_files() {
  ((NEO_GPU)) && printf '%s\n' "$NEO_MESA_PACKAGE"
  return 0
}

# Both ESP spellings select the same release boot= payload path.
neo_gpu_stage2_config_check() {
  local config=$1
  grep -Eq '^#define (J700_ESP_STAGE2|ESP_STAGE2)$' <<<"$config" &&
    grep -qx '#define CHAINLOADING' <<<"$config" &&
    grep -qx '#define RELEASE' <<<"$config" &&
    ! grep -Eq '^#define (T8140_KIS_PROXY|J700_CDC_PROXY|J613_ESP_STAGE1)$' <<<"$config"
}

neo_gpu_package_check() {
  ((NEO_GPU)) || return 0
  local mesa=$work/${NEO_MESA_PACKAGE%% *} boot=$work/${NEO_M1N1_PACKAGE%% *}
  [[ $(bsdtar -xOf "$mesa" opt/mesa-neo/share/mesa-neo/profile) == "$NEO_GPU_PROFILE" ]] || die "Neo Mesa profile marker differs; nothing was installed"
  [[ $(bsdtar -xOf "$boot" usr/share/m1n1-neo/profile) == "$NEO_GPU_PROFILE" ]] || die "Neo bootloader profile marker differs; nothing was installed"
  local config
  config=$(bsdtar -xOf "$boot" usr/share/m1n1-neo/build-config.h) || die "Neo bootloader build configuration is missing; nothing was installed"
  neo_gpu_stage2_config_check "$config" || die "Neo bootloader is not an ESP release stage 2; nothing was installed"
  local path
  for path in opt/mesa-neo/libexec/mesa-neo-abi-check opt/mesa-neo/libexec/mesa-neo-loadcheck \
    opt/mesa-neo/share/vulkan/icd.d/asahi_icd.aarch64.json opt/mesa-neo/bin/mesa-neo-probe; do
    bsdtar -tf "$mesa" | grep -qx "$path" || die "Neo Mesa lacks $path; nothing was installed"
  done
}

neo_gpu_transaction_begin() {
  ((NEO_GPU)) || return 0
  local target original_m1n1
  target=$(esp_bootbin) || die "the Neo's boot.bin disappeared"
  # Keep the original binary as well as boot.bin: its package may be replaced.
  original_m1n1=$($sudo bash -c 'M1N1=/usr/lib/asahi-boot/m1n1.bin; [[ ! -f $1 ]] || source "$1"; printf "%s" "$M1N1"' _ "$UPDATE_M1N1_CONF")
  [[ $original_m1n1 == /* && -f $original_m1n1 && ! -L $original_m1n1 ]] || die "cannot preserve this Neo's original M1N1 binary: $original_m1n1"

  $sudo install -d "$STATE"
  $sudo python3 - "$STATE/neo-transaction.json" "$UPDATE_M1N1_CONF" "$target" \
    "$NEO_GPU_CONFIG/profile" "$NEO_GPU_CONFIG/gpu-experiment" "$STATE/neo-gpu" "$original_m1n1" <<'NEO_SNAPSHOT'
import base64,json,stat,sys
from pathlib import Path
rows=[]
for name in sys.argv[2:]:
 p=Path(name)
 if p.exists() or p.is_symlink():
  info=p.lstat()
  if not stat.S_ISREG(info.st_mode): raise SystemExit('Neo transaction needs regular files: '+name)
  rows.append({'path':name,'mode':stat.S_IMODE(info.st_mode),'data':base64.b64encode(p.read_bytes()).decode()})
 else: rows.append({'path':name,'data':None})
p=Path(sys.argv[1]);p.write_text(json.dumps(rows));p.chmod(0o600)
NEO_SNAPSHOT
  [[ -f $STATE/neo-before.json ]] || $sudo cp "$STATE/neo-transaction.json" "$STATE/neo-before.json"
  NEO_GPU_PREPARED=1
  neo_gpu_boot_config
}

neo_gpu_boot_config() {
  local tmp
  tmp=$(mktemp)
  if [[ -f $UPDATE_M1N1_CONF ]]; then
    $sudo sed '/^# >>> aurora-sep: Neo stage2$/,/^# <<< aurora-sep: Neo stage2$/d' "$UPDATE_M1N1_CONF" > "$tmp"
  fi
  cat >>"$tmp" <<'NEO_BOOT_CONFIG'
# >>> aurora-sep: Neo stage2
M1N1=/usr/lib/m1n1-neo/m1n1.bin
DTBS=$(pacman -Qlq linux-aurora 2>/dev/null | grep '/dtbs/[^/]*\.dtb$' || true)
# <<< aurora-sep: Neo stage2
NEO_BOOT_CONFIG
  $sudo install -Dm644 "$tmp" "$UPDATE_M1N1_CONF"
  rm -f "$tmp"
}

neo_gpu_restore() {
  local snapshot=$1
  [[ -f $snapshot ]] || return 0
  $sudo python3 - "$snapshot" <<'NEO_RESTORE'
import base64,json,os,tempfile,sys
from pathlib import Path
for row in json.loads(Path(sys.argv[1]).read_text()):
 p=Path(row['path'])
 if row['data'] is None: p.unlink(missing_ok=True);continue
 p.parent.mkdir(parents=True,exist_ok=True)
 fd,name=tempfile.mkstemp(prefix='.'+p.name+'.neo-',dir=p.parent)
 try:
  with os.fdopen(fd,'wb') as f:
   f.write(base64.b64decode(row['data']));f.flush();os.fsync(f.fileno())
  os.chmod(name,row['mode']);os.replace(name,p)
 finally:
  if os.path.exists(name):os.unlink(name)
NEO_RESTORE
  sync
}

neo_gpu_activate() {
  ((NEO_GPU)) || return 0
  [[ $(cat "$NEO_MESA_PREFIX/share/mesa-neo/profile") == "$NEO_GPU_PROFILE" ]] || die "installed Neo Mesa marker differs"
  update_m1n1_frozen && die "Neo boot.bin was not rebuilt: update-m1n1 is disabled"
  m1n1_check_and_record
  local name uid gid home shell password
  getent group render >/dev/null || $sudo groupadd --system render
  while IFS=: read -r name password uid gid password home shell; do
    [[ $uid =~ ^[0-9]+$ ]] || continue
    ((uid >= 1000 && uid < 60000)) || continue
    [[ $shell != */nologin && $shell != */false ]] || continue
    $sudo usermod -aG render "$name"
  done < <(getent passwd)
  $sudo install -d "$NEO_GPU_CONFIG"
  printf '%s\n' "$NEO_GPU_PROFILE" | $sudo tee "$NEO_GPU_CONFIG/profile" >/dev/null
  printf '# Experimental Neo GPU selected by the matched installer. Remove this file to use software rendering.\n' | $sudo tee "$NEO_GPU_CONFIG/gpu-experiment" >/dev/null
  printf '%s\n' "$NEO_GPU_PROFILE" | $sudo tee "$STATE/neo-gpu" >/dev/null
  NEO_GPU_COMMITTED=1
  $sudo rm -f "$STATE/neo-transaction.json"
  say "Neo GPU profile installed. Reboot, then run: /opt/mesa-neo/bin/mesa-neo-probe"
}

# ---- the M3 Max boot loader variant's earlier boot.bin ---------------------------------------------
# The boot.bin an M3 Max booted before its boot loader variant first went on: the copy
# keep_bootbin_on_esp kept, recorded with its sha256 in $STATE/m3max-bootbin-backup ("path
# sha256"). A later install keeps the first record, so --uninstall and the test kit's --restore
# always put back the boot.bin from before the variant, whatever release ran since.
m3max_bootbin_record() {
  local target keep sha
  [[ -f $STATE/m3max-bootbin-backup ]] && return 0
  target=$(esp_bootbin) || return 0
  keep=$target.before-$VERSION
  sha=$($sudo sha256sum "$keep" 2>/dev/null | cut -d' ' -f1) || sha=""
  [[ $sha =~ ^[0-9a-f]{64}$ ]] || die "could not read the kept copy $keep; nothing was installed"
  $sudo install -d "$STATE"
  echo "$keep $sha" | $sudo tee "$STATE/m3max-bootbin-backup" >/dev/null
}

# update-m1n1's configuration as a kernel-only M3 has it: the one this Mac had before the first
# install (snapshot_boot_state), with the image's own DTBS line; without that copy, the current
# one without the DTBS block m1n1_update wrote for the variant's rebuilds. Then frozen by this
# script (m3_freeze), so no kernel or m1n1 update rebuilds the boot.bin that was put back.
m3max_conf_kernel_only() {
  local tmp
  if [[ -f $STATE/update-m1n1.default.saved ]]; then
    $sudo install -m 644 "$STATE/update-m1n1.default.saved" "$UPDATE_M1N1_CONF"
  elif [[ -e $STATE/update-m1n1.absent ]]; then
    $sudo rm -f "$UPDATE_M1N1_CONF"
  elif [[ -f $UPDATE_M1N1_CONF ]]; then
    tmp=$(mktemp)
    $sudo sed -e '/^# aurora-sep: build m1n1/,/^DTBS=/d' -e '/^DTBS=/d' "$UPDATE_M1N1_CONF" >"$tmp"
    if grep -q '[^[:space:]]' "$tmp"; then
      $sudo install -m 644 "$tmp" "$UPDATE_M1N1_CONF"
    else
      $sudo rm -f "$UPDATE_M1N1_CONF"
    fi
    rm -f "$tmp"
  fi
  m3_freeze
}

# True when the installed m1n1 ($M1N1_BIN) is a T6031 build, which runs the M3 Max handoffs
# unless a switch says =0 (M3_MAX_SWITCHES). The variant installs one, and putting the earlier
# boot.bin back leaves it installed: a boot.bin rebuilt from it without those switches would run
# the GPU handoff at every boot.
m3max_m1n1_installed() {
  local bin rc=1
  [[ -f $M1N1_BIN ]] || return 1
  bin=$(mktemp)
  # One string per line, in a file, as m1n1_pkg_has_handoff reads a package's.
  if tr '\0' '\n' <"$M1N1_BIN" >"$bin" 2>/dev/null && grep -qaxF asahi,t6031-gpu "$bin"; then rc=0; fi
  rm -f "$bin"
  return "$rc"
}

# Puts that boot.bin back (checked by its sha256, replaced through a compared copy), and drops
# the record and $STATE/m1n1-installed, so no later run takes the put-back boot.bin for a failed
# m1n1. With "check", only says whether it can. Returns 1, with a warning, when it can't.
m3max_bootbin_restore() { # [check]
  local keep sha target
  if [[ ! -f $STATE/m3max-bootbin-backup ]]; then
    warn "there is no record of the boot.bin this M3 Max booted before its boot loader variant ($STATE/m3max-bootbin-backup)"
    return 1
  fi
  read -r keep sha <"$STATE/m3max-bootbin-backup"
  if ! target=$(esp_bootbin); then
    warn "could not find m1n1's boot.bin on the EFI partition"
    return 1
  fi
  if [[ $($sudo sha256sum "$keep" 2>/dev/null | cut -d' ' -f1) != "$sha" ]]; then
    warn "$keep, the boot.bin this M3 Max booted before its boot loader variant, is missing or changed (it was sha256 $sha)"
    return 1
  fi
  [[ ${1:-} != check ]] || return 0
  if [[ $($sudo sha256sum "$target" | cut -d' ' -f1) != "$sha" ]]; then replace_on_esp "$keep" "$target"; fi
  [[ $($sudo sha256sum "$target" | cut -d' ' -f1) == "$sha" ]] ||
    die "$target is not the boot.bin it should be after putting it back; $keep is still there"
  $sudo rm -f "$STATE/m3max-bootbin-backup" "$STATE/m1n1-installed"
  say "Put back the boot.bin this M3 Max booted before the boot loader variant (sha256 $sha, from $keep)"
}

# ---- the M3 Max test kit (--m3max-kit) -----------------------------------------------------------
# One command runs the M3 Max test plan (T1 to T4 of the J516C ADT review) on a 16-core M3 Max
# (t6031: J514C, J516C), so the tester runs it once and uploads one file. --m3max-kit:
#   1. checks this Mac (chip and board, the Limine boot menu, the one-shot boot variables, disk
#      space) and downloads and checks the kit's files, before anything changes;
#   2. shows one consent screen (or takes --yes);
#   3. writes the baseline report (stage 00-baseline, the --m3-report collection);
#   4. installs this release's kernel and the M3 Max boot loader variant (--m3-handoff), keeping
#      the boot.bin the Mac booted with on the EFI partition (with the restore steps) and the
#      kernel it runs as a boot menu entry ("Aurora previous (GPU off)"), plus the kit's
#      separate-prefix Mesa when the release has one (M3MAX_KIT_MESA_PACKAGE);
#   5. installs the runner, aurora-m3max-kit (this script, run with --m3max-kit-runner), and two
#      systemd units, and restarts the Mac.
# From then on the runner works at every boot (aurora-m3max-kit.service): it collects what this
# boot shows into the stage's directory, then arms the plan's next boot as ONE Limine one-shot
# boot (air-gpu-oneshot.sh's LoaderEntryOneShot mechanism, with its locks and checks) and
# restarts the Mac after a short delay. A one-shot boot is used once: Limine deletes the
# variable before the kernel starts, so a hang, a panic (panic=10) or a power cycle always lands
# on the normal entry. A stage whose boot left no collection is recorded as hung at the next boot
# (hung when it reached userspace, aurora-m3max-kit-mark.service's marker; hung-early when it did
# not), with that boot's journal, and the kit goes on. The plan (M3MAX_KIT_PLAN, a release
# asset; m3max_kit_plan_check says what it holds) names every boot and its kernel parameters, so
# its knob sets change without a code change. The kit arms at most the plan's max-boots
# experimental boots (never more than M3MAX_KIT_BOOT_CEILING). It ends on a normal boot: it
# collects that boot too, packs every stage into one tgz in the desktop user's home, masked and
# checked as --m3-report's is, and says which file to upload. "sudo aurora-m3max-kit --status",
# "--stop" and "--restore" work at any point; --restore puts back the boot.bin the Mac booted with
# (and the kernel-only state around it) and keeps the previous kernel's boot menu entry.
M3MAX_KIT=0
# --yes after --m3max-kit: the consent is given on the command line.
M3MAX_KIT_YES=0
M3MAX_KIT_STATE=/var/lib/aurora-m3max-kit
M3MAX_KIT_LIBEXEC=/usr/local/libexec/aurora-m3max-kit
M3MAX_KIT_BIN=/usr/local/bin/aurora-m3max-kit
M3MAX_KIT_UNIT_DIR=/etc/systemd/system
M3MAX_KIT_UNIT=aurora-m3max-kit.service
M3MAX_KIT_MARK_UNIT=aurora-m3max-kit-mark.service
# The kit's plan, "file sha256": a release asset (tools/aurora-sep/m3max-kit/m3max-kit.plan),
# downloaded and checked like M3_GPU_SCRIPTS. The kit also takes air-gpu-oneshot.sh and
# air-gpu-job.sh from M3_GPU_SCRIPTS, and the ADT reader from M3_ADT_READER.
M3MAX_KIT_PLAN="m3max-kit.plan c8df9530af334437c6c6fdce711532b0c23cc9a68c4209129a23d990b1c2c530"
# A separate-prefix Mesa for the jobs stage, "file sha256", or empty when the release has none:
# mesa-m3-g15c (G15C behind ASAHI_M3_G15C=1, in /opt/mesa-m3-g15c, with bin/g15c-first-job and
# bin/mesa-m3-probe). It goes on only when it can be used and changes nothing else: every file
# under /opt, every dependency already satisfied (pacman -T; no partial upgrade), the test
# kernel has the GPU start experiment, and the image does not hold its packages; then in a
# pacman transaction of its own.
M3MAX_KIT_MESA_PACKAGE="mesa-m3-g15c-26.1.4.g15c1-2-aarch64.pkg.tar.zst 552c94de1a98a25cc5e0dbd9e3a4cf5fdd738fb5d95689b41428f9dee3aa0936"
# The most experimental boots one kit run arms, whatever its plan says.
M3MAX_KIT_BOOT_CEILING=12
# The tag on an armed boot's command line (its arming id), and the Limine entry's name.
M3MAX_KIT_TAG=m3max_kit.boot
M3MAX_KIT_ENTRY=m3max-kit
# Free space (MB) the kit needs in its state directory and in the desktop user's home.
M3MAX_KIT_MIN_MB=200
# Where the consent screen reads the answer (tests point it at a file).
M3MAX_KIT_TTY=/dev/tty
# Seconds to let a boot settle after it finished starting, before the runner collects it, and
# more for a display boot (the DCP waits up to 30 s for the PMP).
M3MAX_KIT_SETTLE=20
M3MAX_KIT_DISPLAY_SETTLE=40
# The device tree as the boot loader handed it over (before the kernel's display gate changed it),
# and the journal directory that keeps a hung boot's log.
M3MAX_KIT_FDT=/sys/firmware/fdt
M3MAX_KIT_JOURNAL_DIR=/var/log/journal
M3MAX_KIT_DEBUGFS=/sys/kernel/debug
# Where the one-shot arming finds the EFI variables, the command line, the Limine tools' locks
# and their settings (air-gpu-oneshot.sh's own defaults; tests point them at a fake Mac), and the
# arming script itself (the runner's copy; the setup's download before that).
M3MAX_KIT_EFIVARS=/sys/firmware/efi/efivars
M3MAX_KIT_CMDLINE=/proc/cmdline
M3MAX_KIT_LOCKS="/run/lock/boot-partition.lock /tmp/limine-global.lock"
M3MAX_KIT_LIMINE_DEFAULTS=/etc/default/limine
M3MAX_KIT_ONESHOT=$M3MAX_KIT_LIBEXEC/air-gpu-oneshot.sh
M3MAX_KIT_JOB=$M3MAX_KIT_LIBEXEC/air-gpu-job.sh
# The plan this run follows: the runner's copy, or the setup's checked download.
M3MAX_KIT_PLAN_FILE=$M3MAX_KIT_STATE/plan
# The device tree the one-shot subshell reads (air-gpu-oneshot.sh sets its own DT when sourced).
M3MAX_KIT_DT=""
# Set by m3max_kit_arm: the id of the boot it armed, or why it did not arm.
M3MAX_KIT_ARMED_ID="" M3MAX_KIT_WHY=""

# ---- the plan -------------------------------------------------------------------------------
# Checks a plan file and prints it normalized (no comments, single spaces), or prints why not on
# stderr and returns 1. One directive per line; # starts a comment:
#   format 1                      first, always
#   chip t6031                    the chip it is for
#   boards j514c j516c            the boards it is for (M3_MAX_BOARDS or fewer)
#   max-boots N                   experimental boots one run arms (1 to M3MAX_KIT_BOOT_CEILING; 6)
#   reboot-delay S                seconds between a boot's collection and the restart (30-900; 60)
#   safety PARAM...               added to every experimental boot's command line, unchecked in
#                                 the kernel image (hang detection, the systemd watchdog)
#   loader-delay S                seconds before the restart into the loader boot (30-900; 180)
#   boot NAME KIND [PARAM...]     one boot, in the order they run. KIND:
#                                   data     the normal boot after the install (no one-shot, no
#                                            parameters); exactly one, the first
#                                   display  one one-shot boot with PARAMs
#                                   loader   boot.bin rebuilt with m1n1's GPU handoff on
#                                            (M3_MAX_SWITCHES), then a normal boot: not a one-shot.
#                                            The interim results are packed first. No parameters;
#                                            at most one; the gpu and jobs boots come after it
#                                   gpu      one one-shot boot with PARAMs; the gpu boots after
#                                            the first that succeeds are skipped
#                                   jobs     one one-shot boot after a gpu boot succeeded, when
#                                            mesa-prefix has a Vulkan driver: "@gpu" stands for
#                                            that boot's PARAMs; then it runs the jobs below
#   success KIND REGEX            an extended regex over the boot's kernel log that marks a boot
#                                 of KIND (display or gpu) a success
#   mesa-prefix PATH              the jobs stage's Mesa (under /opt)
#   job-env NAME=VALUE...         the jobs' environment, on top of air-gpu-job.sh's
#   jobs JOB...                   first-job (the prefix's headless bin/g15c-first-job), compute
#                                 (air-gpu-job.sh) and render (the prefix's bin/mesa-m3-probe in
#                                 the desktop session), in order, up to 6; none after a hang
# A PARAM is name=value: name is module.param (not rd., systemd. but systemd.watchdog_sec,
# init., root.), or one of the plain names in BARE below; value is 1 to 64 of [A-Za-z0-9_.,:+-].
# The kit adds panic=10 and its tag itself.
m3max_kit_plan_check() { # FILE
  python3 -I - "$1" "$M3MAX_KIT_BOOT_CEILING" "$M3_MAX_BOARDS" <<'M3MAX_PLAN_PY'
import re, sys

path, ceiling, max_boards = sys.argv[1], int(sys.argv[2]), sys.argv[3].split()
KINDS = ("data", "display", "loader", "gpu", "jobs")
NAME = re.compile(r"[a-z0-9][a-z0-9-]{0,31}")
PNAME = re.compile(r"[a-z0-9_]+(\.[a-z0-9_]+)?")
VALUE = re.compile(r"[A-Za-z0-9_.,:+-]{1,64}")
BARE = {"softlockup_panic", "hung_task_panic", "hung_task_timeout_secs", "loglevel",
        "ignore_loglevel", "initcall_debug", "printk.devkmsg"}
DOTTED_OK = {"systemd.watchdog_sec", "printk.devkmsg"}
DENIED_MODULES = {"rd", "systemd", "init", "root", "rootflags", "luks", "udev", "fstab",
                  "m3max_kit", "mesa_m3", "air_gpu"}
ENV_NAME = re.compile(r"[A-Z_][A-Z0-9_]{0,63}")
ENV_VALUE = re.compile(r"[A-Za-z0-9_.,:/=+-]{0,128}")


def fail(line, why):
    where = f"line {line}: " if line else ""
    sys.stderr.write(f"the kit's plan, {where}{why}\n")
    sys.exit(1)


def param(word, line, allow_gpu=False):
    if word == "@gpu":
        if not allow_gpu:
            fail(line, "@gpu belongs to a jobs boot only")
        return word
    if "=" not in word:
        fail(line, f"{word!r} is not name=value")
    name, value = word.split("=", 1)
    if not PNAME.fullmatch(name):
        fail(line, f"{name!r} is not a parameter name")
    if "." in name:
        module = name.split(".", 1)[0]
        if name not in DOTTED_OK and module in DENIED_MODULES:
            fail(line, f"{name} is not a parameter the kit sets")
    elif name not in BARE:
        fail(line, f"{name} is not a parameter the kit sets (a plain name must be one of {', '.join(sorted(BARE))})")
    if not VALUE.fullmatch(value):
        fail(line, f"{name}={value}: the value must be 1 to 64 of A-Z a-z 0-9 _ . , : + -")
    return word


try:
    with open(path, encoding="ascii") as f:
        raw = f.read()
except (OSError, UnicodeError) as e:
    fail(0, f"could not be read ({e})")
seen = {}
boots = []
out = []
success = {}
jobs = []
for n, text in enumerate(raw.splitlines(), 1):
    text = text.split("#", 1)[0].strip() if not text.lstrip().startswith("success ") else text.strip()
    if not text:
        continue
    words = text.split()
    key = words[0]
    if not out and key != "format":
        fail(n, "the first directive must be: format 1")
    once = ("format", "chip", "boards", "max-boots", "reboot-delay", "loader-delay", "safety", "mesa-prefix",
            "job-env", "jobs")
    if key in once:
        if key in seen:
            fail(n, f"{key} is given twice")
        seen[key] = words[1:]
    if key == "format":
        if words[1:] != ["1"]:
            fail(n, "this kit reads plan format 1")
    elif key == "chip":
        if words[1:] != ["t6031"]:
            fail(n, "this kit is for chip t6031")
    elif key == "boards":
        if not words[1:] or any(b not in max_boards for b in words[1:]):
            fail(n, f"boards must be among {' '.join(max_boards)}")
    elif key == "max-boots":
        if len(words) != 2 or not words[1].isdigit() or not 1 <= int(words[1]) <= ceiling:
            fail(n, f"max-boots must be 1 to {ceiling}")
        words[1] = str(int(words[1]))
    elif key in ("reboot-delay", "loader-delay"):
        if len(words) != 2 or not words[1].isdigit() or not 30 <= int(words[1]) <= 900:
            fail(n, f"{key} must be 30 to 900 seconds")
        words[1] = str(int(words[1]))
    elif key == "safety":
        for w in words[1:]:
            param(w, n)
    elif key == "boot":
        if len(words) < 3:
            fail(n, "boot needs a name and a kind")
        name, kind, params = words[1], words[2], words[3:]
        if not NAME.fullmatch(name) or name in ("baseline", "final"):
            fail(n, f"{name!r} is not a boot name (a-z, 0-9 and -, up to 32; not baseline or final)")
        if any(b[0] == name for b in boots):
            fail(n, f"boot {name} is given twice")
        if kind not in KINDS:
            fail(n, f"{kind!r} is not a kind ({', '.join(KINDS)})")
        if params == ["-"]:
            params = []
            words = words[:3]
        if kind == "data" and (params or boots):
            fail(n, "the data boot is the first boot, with no parameters")
        if kind != "data" and not boots:
            fail(n, "the first boot must be the data boot")
        if kind == "data" and any(b[1] == "data" for b in boots):
            fail(n, "only one data boot")
        if kind != "jobs" and any(b[1] == "jobs" for b in boots):
            fail(n, "the jobs boot comes after every other boot")
        if kind == "jobs" and any(b[1] == "jobs" for b in boots):
            fail(n, "only one jobs boot")
        if kind in ("display", "gpu") and not params:
            fail(n, f"a {kind} boot needs parameters")
        if kind == "loader" and params:
            fail(n, "the loader boot takes no parameters")
        if kind == "loader" and any(b[1] == "loader" for b in boots):
            fail(n, "only one loader boot")
        if kind == "loader" and any(b[1] in ("gpu", "jobs") for b in boots):
            fail(n, "the gpu and jobs boots come after the loader boot")
        for w in params:
            param(w, n, allow_gpu=(kind == "jobs"))
        if params.count("@gpu") > 1:
            fail(n, "@gpu is given twice")
        boots.append((name, kind, params))
    elif key == "success":
        if len(words) < 3 or words[1] not in ("display", "gpu"):
            fail(n, "success needs a kind (display or gpu) and an extended regex")
        regex = text.split(None, 2)[2]
        try:
            re.compile(regex)
        except re.error as e:
            fail(n, f"the success regex does not compile ({e})")
        if words[1] in success:
            fail(n, f"success {words[1]} is given twice")
        success[words[1]] = regex
        out.append(f"success {words[1]} {regex}")
        continue
    elif key == "mesa-prefix":
        if len(words) != 2 or not re.fullmatch(r"/opt/[A-Za-z0-9._/-]+", words[1]) or ".." in words[1].split("/"):
            fail(n, "mesa-prefix must be one path under /opt")
    elif key == "job-env":
        for w in words[1:]:
            k, _, v = w.partition("=")
            if not _ or not ENV_NAME.fullmatch(k) or not ENV_VALUE.fullmatch(v):
                fail(n, f"{w!r} is not NAME=VALUE")
    elif key == "jobs":
        if not 1 <= len(words) - 1 <= 6 or any(j not in ("first-job", "compute", "render") for j in words[1:]):
            fail(n, "jobs takes 1 to 6 of first-job, compute and render")
    else:
        fail(n, f"unknown directive {key!r}")
    out.append(" ".join(words))
for need in ("chip", "boards"):
    if need not in seen:
        fail(0, f"{need} is missing")
if not boots:
    fail(0, "it names no boots")
if "max-boots" not in seen:
    out.append("max-boots 6")
if "reboot-delay" not in seen:
    out.append("reboot-delay 60")
if "loader-delay" not in seen:
    out.append("loader-delay 180")
if any(b[1] == "jobs" for b in boots) and "mesa-prefix" not in seen:
    fail(0, "a jobs boot needs mesa-prefix")
sys.stdout.write("\n".join(out) + "\n")
M3MAX_PLAN_PY
}

# The value of the plan's first KEY line (the words after the key), or nothing.
m3max_plan() { awk -v k="$1" '$1 == k { sub(/^[^ ]+ ?/, ""); print; exit }' "$M3MAX_KIT_PLAN_FILE" 2>/dev/null; }
# The plan's boots, one per line: "NAME KIND PARAM...".
m3max_plan_boots() { awk '$1 == "boot" { sub(/^boot /, ""); print }' "$M3MAX_KIT_PLAN_FILE" 2>/dev/null; }
# The INDEX-th boot (0 is the data boot), as m3max_plan_boots prints it.
m3max_plan_boot() { m3max_plan_boots | sed -n "$(($1 + 1))p"; }
m3max_plan_count() { m3max_plan_boots | wc -l; }
# The success regex of KIND, or nothing.
m3max_plan_success() { awk -v k="$1" '$1 == "success" && $2 == k { sub(/^success [^ ]+ /, ""); print; exit }' "$M3MAX_KIT_PLAN_FILE" 2>/dev/null; }

# ---- the kit's state ----------------------------------------------------------------------------
# $M3MAX_KIT_STATE (root's, 0700): status (key=value lines: phase, next, armed, boots, gpu_ok,
# kernel, bootbin, user, board, started, restart, tarball), plan, log, the one-shot's own records
# (oneshot/), reached-<id> markers, and stages/NN-NAME/ with each stage's collection.
#   phase: setup (until the install is done), running, finishing (the next normal boot packs),
#   done, stopped (--stop) or restored (--restore).
#   next: the index of the next plan boot to arm. armed: "<id> <index>" while a boot is armed.
m3max_kit_get() {
  [[ -f $M3MAX_KIT_STATE/status ]] || return 0
  sed -n "s/^$1=//p" "$M3MAX_KIT_STATE/status" | tail -1
}

# Sets KEY to VALUE (pairs of arguments), durably: a temp file, synced, renamed.
m3max_kit_set() {
  local tmp=$M3MAX_KIT_STATE/status.tmp
  if [[ -f $M3MAX_KIT_STATE/status ]]; then cp "$M3MAX_KIT_STATE/status" "$tmp"; else : >"$tmp"; fi
  while (($# >= 2)); do
    sed -i "/^$1=/d" "$tmp"
    printf '%s=%s\n' "$1" "$2" >>"$tmp"
    shift 2
  done
  sync "$tmp" 2>/dev/null || sync
  mv -f "$tmp" "$M3MAX_KIT_STATE/status"
}

m3max_kit_log() {
  local line
  line="$(date -Is) $*"
  printf '%s\n' "$line" >>"$M3MAX_KIT_STATE/log" 2>/dev/null || true
  printf 'aurora-m3max-kit: %s\n' "$*"
}

# The stage directory of plan boot INDEX (stages/01-data for 0), or of baseline / final.
m3max_kit_stage_dir() {
  local name
  case $1 in
    baseline) echo "$M3MAX_KIT_STATE/stages/00-baseline" ;;
    final) echo "$M3MAX_KIT_STATE/stages/99-final" ;;
    *)
      read -r name _ <<<"$(m3max_plan_boot "$1")"
      printf '%s/stages/%02d-%s\n' "$M3MAX_KIT_STATE" "$(($1 + 1))" "$name"
      ;;
  esac
}

# Records a stage's outcome: one line "OUTCOME DETAIL" in its outcome.txt.
m3max_kit_outcome() { # STAGE OUTCOME DETAIL
  local dir
  dir=$(m3max_kit_stage_dir "$1")
  mkdir -p "$dir"
  printf '%s %s\n' "$2" "${3:-}" >"$dir/outcome.txt"
  m3max_kit_log "stage ${dir##*/}: $2${3:+ ($3)}"
}

# This boot's arming id, from its command line, or nothing on a normal boot.
m3max_kit_boot_tag() {
  tr ' ' '\n' <"$M3MAX_KIT_CMDLINE" 2>/dev/null | sed -n "s/^${M3MAX_KIT_TAG//./\\.}=//p" | head -1
}

# The desktop user the kit writes its result for, and that user's home.
m3max_kit_user() { local u; u=$(m3max_kit_get user); echo "${u:-root}"; }
m3max_kit_home() { local h; h=$(getent passwd "$1" 2>/dev/null | cut -d: -f6); echo "${h:-/root}"; }

# Free space in MB where PATH is (its nearest existing directory).
m3max_kit_free_mb() {
  local p=$1
  while [[ ! -d $p && $p == /* && $p != / ]]; do p=${p%/*}; p=${p:-/}; done
  df -Pm "$p" 2>/dev/null | awk 'NR == 2 { print $4 }'
}

# ---- one-shot boots, through air-gpu-oneshot.sh ----------------------------------------------
# The kit sources air-gpu-oneshot.sh in a subshell and keeps its mechanism, its checks and its
# locks as they are: the same EFI variable, the same U-Boot variable file, the same Limine block
# handling. What it changes is what is armed: its own entry (m3max-kit), tag and record directory,
# its own board check (is_m3_max), the plan's parameters (checked already, and each one looked
# for in the kernel image by the script's own preflight), the plan's safety parameters, and a
# command line without any earlier t6031 parameter or tag.
# shellcheck disable=SC2034,SC2329 # its variables and functions are air-gpu-oneshot.sh's
m3max_kit_oneshot_setup() {
  # Its own DT default, back to this script's.
  DT=$M3MAX_KIT_DT
  PROG=m3max-kit-oneshot
  ENTRY=$M3MAX_KIT_ENTRY
  TAG=$M3MAX_KIT_TAG
  BEGIN_MARK="# >>> m3max-kit: one M3 Max test kit boot, chosen only through LoaderEntryOneShot (remove with: sudo aurora-m3max-kit --stop)"
  END_MARK="# <<< m3max-kit"
  STATE_DIR=$M3MAX_KIT_STATE/oneshot
  EFIVARS=$M3MAX_KIT_EFIVARS
  CMDLINE=$M3MAX_KIT_CMDLINE
  LOCKS=$M3MAX_KIT_LOCKS
  LIMINE_DEFAULTS=$M3MAX_KIT_LIMINE_DEFAULTS
  check_board() {
    is_m3_max || refuse "this Mac ($(this_board), $(this_soc)) is not a 16-core M3 Max (t6031: $M3_MAX_BOARDS)."
  }
  # The kit records the GPU node's state itself, in each stage.
  gpu_node_note() { :; }
  block() {
    printf '%s\n/%s\n    comment: M3 Max test kit, one boot (%s %s)\n    protocol: efi\n    path: %s\n    cmdline: %s\n%s\n' \
      "$BEGIN_MARK" "$ENTRY" "${M3MAX_ARM_NAME:-?}" "$1" "$2" "$3" "$END_MARK"
  }
  new_cmdline() {
    local w out=() p skip words
    set -f
    read -ra words <<<"$SRC_CMDLINE"
    set +f
    for w in "${words[@]}"; do
      skip=0
      [[ $w == asahi.t6031[_-]* || $w == apple_t6031_display.* || $w == "$TAG"=* || $w == panic=* ]] && skip=1
      for p in "${PARAMS[@]}" ${M3MAX_ARM_SAFETY[@]+"${M3MAX_ARM_SAFETY[@]}"}; do
        [[ $w == "${p%%=*}" || $w == "${p%%=*}"=* ]] && skip=1
      done
      ((skip)) || out+=("$w")
    done
    printf '%s\n' "${out[*]} ${PARAMS[*]}${M3MAX_ARM_SAFETY[*]:+ ${M3MAX_ARM_SAFETY[*]}} panic=10 $TAG=$ID"
  }
  # The script's own checks, then: the normal entry must not set a parameter this boot sets, or
  # every boot would run with it.
  eval "oneshot_preflight() $(declare -f preflight | tail -n +2)"
  preflight() {
    local p
    oneshot_preflight
    for p in "${PARAMS[@]}"; do
      [[ " $SRC_CMDLINE " != *" ${p%%=*}="* && " $SRC_CMDLINE " != *" ${p%%=*} "* ]] ||
        refuse "the normal //$SOURCE_ENTRY entry already sets ${p%%=*}, so every boot would run with it. Remove it from /etc/default/limine first."
    done
  }
  if declare -F m3max_kit_oneshot_hook >/dev/null; then m3max_kit_oneshot_hook; fi
}

# Runs air-gpu-oneshot.sh's ACTION (arm NAME PARAM..., disarm, check, armed) with the kit's
# setup, in a subshell; its refusals end the subshell with status 1 and one line on stderr.
m3max_kit_oneshot() { # ACTION [ARGS...]
  local action=$1
  shift
  M3MAX_KIT_DT=$DT
  # shellcheck disable=SC2034 # AIR_GPU_SOURCE_ONLY, QUIET_LOG and KEYS are air-gpu-oneshot.sh's
  (
    set -euo pipefail
    AIR_GPU_SOURCE_ONLY=1
    # shellcheck source=/dev/null
    source "$M3MAX_KIT_ONESHOT"
    m3max_kit_oneshot_setup
    case $action in
      arm)
        M3MAX_ARM_NAME=$1
        shift
        PARAMS=("$@")
        read -ra M3MAX_ARM_SAFETY <<<"$(m3max_plan safety)"
        do_arm
        ;;
      disarm) do_disarm ;;
      check) PARAMS=(); M3MAX_ARM_SAFETY=(); QUIET_LOG=1; do_check ;;
      armed) armed_name ;;
      # The parameter names (of NAME=VALUE...) the kernel image of the normal entry lacks, one
      # per line; status 3 when that image can't be found (the arming's own checks then decide).
      missing)
        find_esp && find_conf || exit 3
        KEYS=$(entry_keys)
        [[ $(key count) == 1 ]] || exit 3
        UKI=$(limine_file "$(key path)")
        [[ -f $UKI ]] || exit 3
        local p
        for p in "$@"; do
          [[ $p == @gpu ]] && continue
          grep -qaF "${p%%=*}" "$UKI" || echo "${p%%=*}"
        done
        ;;
      *) return 2 ;;
    esac
  )
}

# Arms plan boot NAME with PARAMs for the next boot. Sets M3MAX_KIT_ARMED_ID, or M3MAX_KIT_WHY
# and returns 1 (nothing armed).
m3max_kit_arm() { # NAME PARAM...
  local out rc=0
  M3MAX_KIT_ARMED_ID="" M3MAX_KIT_WHY=""
  rm -f "$M3MAX_KIT_STATE/oneshot/armed"
  out=$(m3max_kit_oneshot arm "$@" 2>&1) || rc=$?
  printf '%s\n' "$out" >>"$M3MAX_KIT_STATE/log"
  if ((rc)) || [[ ! -f $M3MAX_KIT_STATE/oneshot/armed ]]; then
    M3MAX_KIT_WHY=$(sed -n 's/.*refused: //p' <<<"$out" | head -1)
    M3MAX_KIT_WHY=${M3MAX_KIT_WHY:-the one-shot arming failed (exit $rc)}
    return 1
  fi
  read -r M3MAX_KIT_ARMED_ID _ <"$M3MAX_KIT_STATE/oneshot/armed"
}

# Clears the one-shot (if it is the kit's) and removes the kit's Limine entry. Returns 1, with
# the reason in the log, when air-gpu-oneshot.sh refused.
m3max_kit_disarm() {
  local out rc=0
  [[ -f $M3MAX_KIT_ONESHOT ]] || return 0
  out=$(m3max_kit_oneshot disarm 2>&1) || rc=$?
  printf '%s\n' "$out" >>"$M3MAX_KIT_STATE/log" 2>/dev/null || true
  return "$rc"
}

# ---- what each stage collects -----------------------------------------------------------------
# Every stage: the --m3-report collection (m3_report_collect: who this Mac is, /chosen, every
# device-tree node, the drivers, the whole kernel log, the ADT allowlist and m1n1's log through
# phram, display, power, ...) into report/, and checks.txt: what that stage looks at, and a
# "verdict:" line, which also goes to outcome.txt.
# Where the runner looks for desktop sessions (notifications, the render job, hyprctl).
M3MAX_KIT_RUNDIR=/run/user

# Waits until this boot has finished starting (at most 5 minutes), then lets it settle. The step's
# unit is Type=exec, so its own start job does not hold the boot up.
m3max_kit_wait_boot() {
  timeout 300 systemctl is-system-running --wait >/dev/null 2>&1 || true
  sleep "$M3MAX_KIT_SETTLE"
}

# The device-tree nodes whose properties the boot loader fills, copied whole (one file per
# property, as /proc/device-tree has them) into the stage's dt/: these hold the GPU and display
# facts whatever the kernel does with them. The /chosen entries of the M3 boot loader
# (asahi,t6031-*: gpu-powered-identity, fw-uuids, pmp, and the rest, plus m1n1's own asahi,m1n1-*
# and firmware versions), every reserved-memory node, and the nodes the aliases below point to
# (or, for the GPU, every /soc/gpu@* node). Properties whose names carry per-chip calibration or
# a serial number or an address are left out, as the ADT reader leaves them out.
M3MAX_KIT_DT_ALIASES="gpu dcp disp0 disp0-piodma dcpext0 dcpext1 dcpext2 dcpext3 pmp"
M3MAX_KIT_DT_DENIED='fuse|calibration|serial|mac-address|bd-address|ecid|nonce|unique'

# Copies device-tree node SRC (recursively) to DST, without the denied properties.
m3max_kit_dt_copy() { # SRC DST
  local f rel
  [[ -d $1 ]] || return 0
  while IFS= read -r -d '' f; do
    rel=${f#"$1"}
    [[ ${rel##*/} =~ $M3MAX_KIT_DT_DENIED ]] && continue
    mkdir -p "$2${rel%/*}"
    cp "$f" "$2$rel" 2>/dev/null || true
  done < <(find "$1" -type f -print0 2>/dev/null)
  mkdir -p "$2"
}

# Copies a flattened device tree with the values of properties named like
# M3MAX_KIT_DT_DENIED zeroed in place (same length): the MAC and Bluetooth addresses in it are raw
# bytes, which the text masking would not find. Their names in the strings block are blanked too
# (letters to x, same length), so the copy stays a valid tree and no name reads as a label.
m3max_kit_fdt_copy() { # SRC DST
  python3 -I - "$1" "$2" "$M3MAX_KIT_DT_DENIED" <<'M3MAX_FDT_PY'
import re, struct, sys
src, dst, denied = sys.argv[1], sys.argv[2], re.compile(sys.argv[3], re.I)
with open(src, "rb") as f:
    data = bytearray(f.read(16 * 1024 * 1024 + 1))
if not 40 <= len(data) <= 16 * 1024 * 1024:
    sys.exit("not an FDT of a usable size")
magic, total, off_struct, off_strings, _, version, _, _, size_strings, size_struct = struct.unpack_from(">10I", data, 0)
if magic != 0xD00DFEED or total > len(data) or version < 17:
    sys.exit("not a flattened device tree (version 17)")
end_struct = off_struct + size_struct
if end_struct > total or off_strings + size_strings > total:
    sys.exit("its blocks run past its end")
off, masked, names = off_struct, 0, set()
while off + 4 <= end_struct:
    (tok,) = struct.unpack_from(">I", data, off)
    off += 4
    if tok == 1:
        off = (data.index(b"\0", off, end_struct) + 4) & ~3
    elif tok == 3:
        length, nameoff = struct.unpack_from(">II", data, off)
        off += 8
        if off + length > end_struct or nameoff >= size_strings:
            sys.exit("a property runs past the structure block")
        start = off_strings + nameoff
        name = data[start:data.index(b"\0", start)].decode("ascii", "replace")
        if denied.search(name):
            data[off:off + length] = bytes(length)
            masked += 1
            names.add((start, len(name)))
        off = (off + length + 3) & ~3
    elif tok in (2, 4):
        continue
    elif tok == 9:
        break
    else:
        sys.exit(f"unknown token {tok:#x}")
for start, n in names:
    data[start:start + n] = bytes(0x78 if chr(b).isalpha() else b for b in data[start:start + n])
with open(dst, "wb") as f:
    f.write(bytes(data[:total]))
print(f"{masked} properties zeroed")
M3MAX_FDT_PY
}

m3max_kit_dt_facts() { # DIR
  local out=$1/dt a target f
  mkdir -p "$out/chosen" "$out/nodes"
  # The whole tree as the boot loader handed it over: every gate and DCP refusal can be
  # reproduced from it offline.
  if [[ -r $M3MAX_KIT_FDT ]]; then
    m3max_kit_fdt_copy "$M3MAX_KIT_FDT" "$out/fdt" >"$out/fdt.txt" 2>&1 || rm -f "$out/fdt"
  fi
  for f in "$DT"/chosen/asahi,t6031-* "$DT"/chosen/asahi,m1n1-* "$DT"/chosen/asahi,*-fw-version \
    "$DT"/chosen/asahi,iboot*-version; do
    [[ -e $f ]] || continue
    if [[ -d $f ]]; then m3max_kit_dt_copy "$f" "$out/chosen/${f##*/}"; else cp "$f" "$out/chosen/"; fi
  done
  m3max_kit_dt_copy "$DT/reserved-memory" "$out/reserved-memory"
  for a in $M3MAX_KIT_DT_ALIASES; do
    target=$({ tr -d '\0' <"$DT/aliases/$a"; } 2>/dev/null) || target=""
    [[ $target == /* && $target != *..* && -d $DT$target ]] || continue
    m3max_kit_dt_copy "$DT$target" "$out/nodes/$a"
    echo "$a $target" >>"$out/nodes/aliases.txt"
  done
  for f in "$DT"/soc/gpu@*; do
    [[ -d $f && ! -e $out/nodes/gpu ]] || continue
    m3max_kit_dt_copy "$f" "$out/nodes/${f##*/}"
  done
  # The GPU's OPP tables (opp-table-gpu, -cs, -afr) and the display subsystem.
  for f in "$DT"/opp-table-gpu* "$DT"/soc/opp-table-gpu* "$DT"/soc/display-subsystem*; do
    [[ -d $f ]] || continue
    m3max_kit_dt_copy "$f" "$out/nodes/${f##*/}"
  done
  {
    echo "# The boot loader's device-tree facts of this boot, one file per property (raw)."
    echo "# chosen/: /chosen asahi,t6031-*, asahi,m1n1-* and the firmware versions"
    echo "# reserved-memory/: every reserved-memory node"
    echo "# nodes/: the nodes of the aliases in nodes/aliases.txt (gpu, dcp, disp0, ...), or /soc/gpu@*"
    echo "# fdt: $M3MAX_KIT_FDT, the tree the boot loader handed over, those properties zeroed (fdt.txt says how many)"
    echo "# Left out: properties named like ${M3MAX_KIT_DT_DENIED//|/, }; listed below by name."
    (cd "$out" && find . -type f ! -name README.txt -printf '%P %s\n' | LC_ALL=C sort)
    find "$DT/chosen" "$DT/reserved-memory" "$DT/soc" -type f 2>/dev/null | grep -E "/[^/]*($M3MAX_KIT_DT_DENIED)[^/]*$" |
      grep -E 't6031|reserved-memory|gpu|dcp|disp|opp' | sed "s#^$DT#left out: #" | head -50 || true
  } >"$out/README.txt"
}

# This boot's report into DIR/report (unmasked; the pack masks), the boot loader's device-tree
# facts into DIR/dt (m3max_kit_dt_facts), and where its kernel log came from
# (DIR/kernel-log-source.txt: journal or dmesg).
m3max_kit_collect() { # DIR
  local dir=$1
  mkdir -p "$dir/report"
  m3max_kit_dt_facts "$dir" || m3max_kit_log "warning: the device-tree facts of this boot are incomplete"
  M3_WORK=$(mktemp -d)
  m3_report_collect "$dir/report" || m3max_kit_log "warning: the report of this boot is incomplete"
  echo "${M3_REPORT_KLOG_SRC:-dmesg}" >"$dir/kernel-log-source.txt"
  m3_adt_unload || true
  rm -rf "$M3_WORK"
  M3_WORK=""
}

# A reserved-memory node's reg as "BASE SIZE" in decimal (one entry), or nothing.
m3max_kit_reg() {
  local r
  r=$(m3_dt_reg "$1/reg" "$(m3_dt_u32 "$DT/reserved-memory/#address-cells")" "$(m3_dt_u32 "$DT/reserved-memory/#size-cells")")
  [[ $r =~ ^0x([0-9a-f]{1,15})\+0x([0-9a-f]{1,15})$ ]] || return 0
  echo "$((16#${BASH_REMATCH[1]})) $((16#${BASH_REMATCH[2]}))"
}

# The data stage (T1): the boot loader's T6031 facts, the display processors' log reservations
# against System RAM, the GPU's powered identity, the CPUs, and whether boot.bin is the kit's.
m3max_kit_checks_data() {
  local f d base size start end n=0 inside=0 facts=0 ident=absent cpus line
  local -a ram=()
  echo "== the boot loader's T6031 facts ($DT/chosen/asahi,t6031-*)"
  for f in "$DT"/chosen/asahi,t6031-*; do
    [[ -e $f ]] || continue
    facts=$((facts + 1))
    if [[ -d $f ]]; then
      echo "${f##*/}/ (a node): $(find "$f" -mindepth 1 -maxdepth 1 -printf '%f ' 2>/dev/null)"
    else
      echo "${f##*/}: $(m3_dt_text "$f")"
    fi
  done
  ((facts)) || echo "(none)"
  [[ -e $DT/chosen/asahi,t6031-gpu-powered-identity ]] && ident=present
  echo "== the display processors' log reservations (dcp-oslog), against System RAM"
  while read -r line; do
    [[ $line =~ ^([0-9a-f]+)-([0-9a-f]+)\ :\ System\ RAM$ ]] && ram+=("$((16#${BASH_REMATCH[1]})) $((16#${BASH_REMATCH[2]}))")
  done <"$M3_PROCFS/iomem"
  for d in "$DT"/reserved-memory/dcp-oslog@*; do
    [[ -d $d ]] || continue
    n=$((n + 1))
    read -r base size <<<"$(m3max_kit_reg "$d")"
    if [[ -z $base ]]; then echo "${d##*/}: no single reg entry"; continue; fi
    f="outside System RAM"
    for line in ${ram[@]+"${ram[@]}"}; do
      read -r start end <<<"$line"
      if ((base <= end && base + size - 1 >= start)); then f="INSIDE System RAM"; inside=$((inside + 1)); break; fi
    done
    printf '%s: 0x%x+0x%x, %s\n' "${d##*/}" "$base" "$size" "$f"
  done
  ((n)) || echo "(none)"
  echo "== the GPU's powered identity (asahi,t6031-gpu-powered-identity): $ident (absent while m1n1's GPU handoff is off)"
  echo "== what the display gate found handed over (apple-t6031-display: handoff:)"
  LC_ALL=C grep -a 'apple-t6031-display: ' "${1:-/dev/null}" 2>/dev/null | head -40 || true
  m3max_kit_m1n1_lines "${2:-/dev/null}"
  if [[ -d $M3MAX_KIT_JOURNAL_DIR ]]; then
    echo "== journal: persistent ($M3MAX_KIT_JOURNAL_DIR): a hung test boot's log is kept"
  else
    echo "== journal: NOT persistent (no $M3MAX_KIT_JOURNAL_DIR): a hung test boot leaves no log"
  fi
  cpus=$(grep -c '^processor' "$M3_PROCFS/cpuinfo" 2>/dev/null) || cpus=0
  echo "== CPUs: $cpus"
  echo "== m1n1 stage 2: $({ tr -d '\0' <"$DT/chosen/asahi,m1n1-stage2-version"; } 2>/dev/null || echo -)"
  if [[ -n $(m3max_kit_get bootbin) && $(m3_bootbin_sha) == "$(m3max_kit_get bootbin)" ]]; then
    echo "== boot.bin: the kit's ($(m3max_kit_get bootbin))"
  else
    echo "== boot.bin: NOT the kit's (sha256 $(m3_bootbin_sha); the kit installed $(m3max_kit_get bootbin))"
  fi
  echo "verdict: t6031-facts=$facts dcp-oslog=$n/5 in-system-ram=$inside gpu-identity=$ident cpus=$cpus"
}

# The lines of m1n1's log of this boot the boot loader workstream reads (its T6031 display, GPU,
# PMP, facts and firmware lines, power-manager timeouts and warnings), from the report's text copy.
m3max_kit_m1n1_lines() { # M1N1-STAGE2-LOG.TXT
  echo "== m1n1's T6031 lines of this boot (m1n1-stage2-log.txt; the whole log is m1n1-stage2-log.raw)"
  LC_ALL=C grep -aE 'm1n1 v|T6031|ADT: GPU:|FDT: GPU:|pmgr: timeout|WARNING|PMP values|firmware image|Max power table' "$1" 2>/dev/null |
    head -200 || true
}

# A big-endian u32 property as a number, or "-".
m3max_kit_u32() { m3_dt_u32 "$1"; }

# The loader boot (S3a): what m1n1's GPU handoff did on this boot. The powered identity (its
# probe-result: 1 pass, 2 power-on failed, 3 read faulted, 4 mismatch, 5 power not restored; the
# power-manager words before and after, which must be equal), the GPU image facts, the GPU node as
# m1n1 filled it, and m1n1's lines. The kernel starts nothing on this boot.
m3max_kit_checks_loader() { # DIR
  local id=$DT/chosen/asahi,t6031-gpu-powered-identity img=$DT/chosen/asahi,t6031-gpu-image gpu="" a result=- restored=-
  local status=- family=-
  echo "== the GPU's powered identity ($id)"
  if [[ -d $id ]]; then
    for a in "$id"/*; do [[ -f $a ]] && echo "${a##*/}: $(od -An -tx4 --endian=big -v "$a" 2>/dev/null | tr -s ' \n' ' ')"; done
    result=$(m3max_kit_u32 "$id/probe-result")
    if [[ -f $id/pmgr-before && -f $id/pmgr-after ]]; then
      if cmp -s "$id/pmgr-before" "$id/pmgr-after"; then restored=yes; else restored=no; fi
    fi
  else
    echo "(absent)"
  fi
  echo "== the GPU image facts ($img)"
  if [[ -d $img ]]; then
    for a in "$img"/*; do [[ -f $a ]] && echo "${a##*/}: $(m3_dt_text "$a")"; done
    family=$(m3_dt_words "$img/firmware-family")
  else
    echo "(absent)"
  fi
  a=$({ tr -d '\0' <"$DT/aliases/gpu"; } 2>/dev/null) || a=""
  [[ $a == /* && $a != *..* && -d $DT$a ]] && gpu=$DT$a
  if [[ -z $gpu ]]; then for a in "$DT"/soc/gpu@*; do [[ -d $a ]] && gpu=$a; done; fi
  echo "== the GPU node (${gpu#"$DT"})"
  if [[ -n $gpu ]]; then
    status=$(m3_dt_words "$gpu/status")
    [[ $status == - ]] && status=okay
    echo "status: $status, compatible: $(m3_dt_words "$gpu/compatible")"
    echo "apple,firmware-compat: $(od -An -tu4 --endian=big "$gpu/apple,firmware-compat" 2>/dev/null | tr -s ' ' ' ')"
    echo "asahi,t6031-gpu-standin: $(m3_dt_words "$gpu/asahi,t6031-gpu-standin")"
  fi
  m3max_kit_m1n1_lines "$1/report/m1n1-stage2-log.txt"
  echo "verdict: probe-result=$result power-restored=$restored family=$family gpu-node=$status"
}

# DRM devices and connectors, the backlight, and the desktop's own view (hyprctl) when it runs.
m3max_kit_display_state() {
  local c drv
  for c in "$M3_SYSFS"/class/drm/card*; do
    [[ -d $c ]] || continue
    if [[ ${c##*/} == card+([0-9]) ]]; then
      drv=$(m3_report_driver_of "$c/device")
      echo "${c##*/}: driver $drv"
    else
      echo "  ${c##*/}: status $(m3_attr "$c/status"), enabled $(m3_attr "$c/enabled"), modes $(tr '\n' ' ' <"$c/modes" 2>/dev/null)"
    fi
  done
  for c in "$M3_SYSFS"/class/backlight/*; do
    [[ -d $c ]] && echo "backlight ${c##*/}: driver $(m3_report_driver_of "$c/device"), brightness $(m3_attr "$c/brightness") of $(m3_attr "$c/max_brightness")"
  done
  m3max_kit_hyprctl
  return 0
}

# Whether each backlight follows a level: half of its maximum for a second, then back.
m3max_kit_backlight_check() {
  local b old max
  for b in "$M3_SYSFS"/class/backlight/*; do
    [[ -w $b/brightness ]] || continue
    old=$(m3_attr "$b/brightness") max=$(m3_attr "$b/max_brightness")
    [[ $old =~ ^[0-9]+$ && $max =~ ^[1-9][0-9]*$ ]] || continue
    echo $((max / 2)) >"$b/brightness" 2>/dev/null || continue
    sleep 1
    echo "== backlight ${b##*/}: set $((max / 2)) of $max, actual $(m3_attr "$b/actual_brightness"); put back $old"
    echo "$old" >"$b/brightness" 2>/dev/null || true
  done
  return 0
}

# hyprctl's monitors, as the desktop user, for each Hyprland that runs (with refresh rates).
m3max_kit_hyprctl() {
  local s user uid sig
  command -v hyprctl >/dev/null || return 0
  for s in "$M3MAX_KIT_RUNDIR"/*/hypr/*/.socket.sock; do
    [[ -S $s ]] || continue
    sig=${s%/.socket.sock}; sig=${sig##*/}
    uid=${s#"$M3MAX_KIT_RUNDIR"/}; uid=${uid%%/*}
    # shellcheck disable=SC2015 # either failing skips this socket
    [[ $uid =~ ^[0-9]+$ ]] && user=$(id -nu "$uid" 2>/dev/null) || continue
    echo "== hyprctl monitors (session of uid $uid)"
    timeout 5 runuser -u "$user" -- env XDG_RUNTIME_DIR="$M3MAX_KIT_RUNDIR/$uid" HYPRLAND_INSTANCE_SIGNATURE="$sig" \
      hyprctl monitors all 2>&1 | head -60 || true
  done
  return 0
}

# A display boot (T2): the DRM state, the display lines of this boot's kernel log, and the verdict:
# native when a card other than the boot framebuffer's has a connected connector with modes.
m3max_kit_checks_display() { # DIR
  local log=$1/report/kernel-log.txt state native=no re match=""
  state=$(m3max_kit_display_state)
  echo "== DRM and backlight"
  printf '%s\n' "$state"
  echo "== the display lines of this boot's kernel log"
  LC_ALL=C grep -aiE 'apple-t6031-display|apple_t6031_display|t6031.*(display|dcp|pmp)|j516c display|dcp|apple-drm|appledrm|piodma|pmp|panel|backlight' "$log" | head -400 || true
  if awk '/^card[0-9]+: driver / { drv = $3 }
    /^  card[0-9]+-.*status connected/ && drv != "" && drv != "-" && drv != "simple-framebuffer" &&
      drv != "simpledrm" && drv != "asahi" && $0 !~ /modes *$/ { found = 1 }
    END { exit !found }' <<<"$state"; then
    native=yes
  fi
  echo "== the DCP's and DRM's debugfs"
  for d in "$M3MAX_KIT_DEBUGFS"/dcp-*/* "$M3MAX_KIT_DEBUGFS"/dri/*/name "$M3MAX_KIT_DEBUGFS"/dri/*/state; do
    [[ -f $d ]] && { echo "-- ${d#"$M3MAX_KIT_DEBUGFS"/}"; head -c 65536 "$d" 2>/dev/null; echo; }
  done
  m3max_kit_backlight_check
  re=$(m3max_plan_success display)
  if [[ -n $re ]]; then match=$(LC_ALL=C grep -aE -m1 -- "$re" "$log" || true); fi
  echo "== the plan's display success line: ${match:-(none)}"
  echo "verdict: native-display=$native success=$([[ -n $match ]] && echo yes || echo no)"
}

# A GPU boot (T3): the GPU node and device, render nodes, devcoredumps, the GPU lines of this
# boot's kernel log, and the verdict: success when the plan's gpu success regex matches.
m3max_kit_checks_gpu() { # DIR
  local log=$1/report/kernel-log.txt d re match="" phase=""
  echo "== GPU nodes"
  for d in "$DT"/soc/gpu@*; do
    [[ -d $d ]] && echo "${d#"$DT"}: compatible $(m3_dt_words "$d/compatible"), status $(m3_dt_words "$d/status")"
  done
  echo "== GPU devices"
  for d in "$M3_SYSFS"/bus/platform/devices/*.gpu; do
    [[ -e $d ]] && echo "${d##*/}: driver $(m3_report_driver_of "$d")"
  done
  echo "== render nodes"
  for d in "$M3_SYSFS"/class/drm/renderD*; do
    [[ -e $d ]] && echo "${d##*/}: driver $(m3_report_driver_of "$d/device")"
  done
  echo "== devcoredump (copied, up to 8 MiB each)"
  for d in "$M3_SYSFS"/class/devcoredump/devcd*; do
    [[ -e $d/data ]] || continue
    echo "${d##*/}"
    head -c $((8 * 1024 * 1024)) "$d/data" >"$1/${d##*/}.bin" 2>/dev/null || true
  done
  echo "== asahi-m3 debugfs"
  for d in "$M3MAX_KIT_DEBUGFS"/asahi-m3/*; do
    [[ -f $d ]] && { echo "-- ${d##*/}"; head -c 65536 "$d" 2>/dev/null; echo; }
  done
  echo "== the GPU start's status (asahi-t6031/status: early copy, then now)"
  cat "$1/early/asahi-t6031-status.txt" 2>/dev/null || echo "(no early copy)"
  m3max_kit_debugfs
  cat "$M3MAX_KIT_DEBUGFS/asahi-t6031/status" 2>/dev/null || echo "(absent now)"
  phase=$(sed -n 's/.* phase=\([^ ]*\).*/\1/p' "$1/early/asahi-t6031-status.txt" "$M3MAX_KIT_DEBUGFS/asahi-t6031/status" 2>/dev/null | tail -1)
  ls -l "$1/early" 2>/dev/null || true
  echo "== the GPU start's lines (M3 G15C start:, M3 G15C verdict:, M3 G15C:, M3 G15S: firmware crashed)"
  LC_ALL=C grep -aE 'M3 G15C (start|verdict)?:|M3 G15S: firmware crashed' "$log" | head -400 || true
  echo "== the GPU lines of this boot's kernel log"
  LC_ALL=C grep -aE 'asahi|agx|G15|t6031|RTKit|gpu|uat|devcoredump' "$log" | head -800 || true
  re=$(m3max_plan_success gpu)
  if [[ -n $re ]]; then match=$(LC_ALL=C grep -aE -m1 -- "$re" "$log" || true); fi
  echo "== the plan's gpu success line: ${match:-(none)}"
  echo "verdict: gpu-start=$([[ -n $match ]] && echo success || echo no-success)${phase:+ phase=$phase}"
}

# debugfs, mounted when it is not (the GPU start's status file lives there).
m3max_kit_debugfs() {
  [[ $M3MAX_KIT_DEBUGFS == /sys/kernel/debug ]] || return 0
  mountpoint -q "$M3MAX_KIT_DEBUGFS" 2>/dev/null || mount -t debugfs debugfs "$M3MAX_KIT_DEBUGFS" 2>/dev/null || true
}

# The GPU start's own record of this boot, copied as early as possible (devcoredump drops its
# data a few minutes after it is offered): /sys/kernel/debug/asahi-t6031/status and every
# devcoredump's data, into the stage's early/ directory (an existing copy is kept).
m3max_kit_gpu_early() { # STAGE-DIR
  local dir=$1/early d
  mkdir -p "$dir"
  m3max_kit_debugfs
  if [[ -f $M3MAX_KIT_DEBUGFS/asahi-t6031/status && ! -f $dir/asahi-t6031-status.txt ]]; then
    cp "$M3MAX_KIT_DEBUGFS/asahi-t6031/status" "$dir/asahi-t6031-status.txt" 2>/dev/null || true
  fi
  for d in "$M3_SYSFS"/class/devcoredump/devcd*; do
    [[ -e $d/data && ! -e $dir/${d##*/}.bin ]] || continue
    head -c $((8 * 1024 * 1024)) "$d/data" >"$dir/${d##*/}.bin" 2>/dev/null || true
  done
  sync
}

# The stage directory of the boot armed with this boot's tag, or nothing.
m3max_kit_tagged_stage() {
  local tag armed id index
  tag=$(m3max_kit_boot_tag)
  armed=$(m3max_kit_get armed)
  [[ -n $tag && -n $armed ]] || return 0
  read -r id index <<<"$armed"
  [[ $id == "$tag" ]] && m3max_kit_stage_dir "$index"
  return 0
}

# The jobs boot (T4): the plan's jobs, each held to a time limit, with their records, and a
# "jobs:" summary line (passed, failed and skipped).
m3max_kit_jobs() { # DIR
  local dir=$1 prefix job n=0 user uid sock rec rc pass=0 fail=0 skip=0 hang=0 out
  local -a env=()
  prefix=$(m3max_plan mesa-prefix)
  read -ra env <<<"$(m3max_plan job-env)"
  user=$(m3max_kit_user)
  for job in $(m3max_plan jobs); do
    n=$((n + 1))
    if ((hang)); then
      echo "job $n $job: skipped after a hang"
      skip=$((skip + 1))
      continue
    fi
    case $job in
      first-job)
        # The headless first jobs (compute, Vulkan and GL render with every word and pixel
        # checked, wrong results again with the scheduling stalls, a short load), as the desktop
        # user, into a directory of theirs, then copied here. It stops after a hang itself.
        rc=0
        out=$(mktemp -d)
        chown "$user" "$out" 2>/dev/null || true
        if [[ -x $prefix/bin/g15c-first-job ]]; then
          timeout -k 10 900 runuser -u "$user" -- env ${env[@]+"${env[@]}"} "$prefix/bin/g15c-first-job" "$out/mesa" 5 \
            >"$dir/job-$n-first-job.txt" 2>&1 || rc=$?
          echo "exit=$rc" >>"$dir/job-$n-first-job.txt"
          cp -r "$out/mesa" "$dir/mesa" 2>/dev/null || true
          echo "job $n first-job: $(grep -c ' pass$' "$dir/mesa/summary.txt" 2>/dev/null || echo 0) passed; $(grep '^VERDICT' "$dir/mesa/summary.txt" 2>/dev/null | tr '\n' ';' | cut -c1-300) (exit $rc)"
          if ((rc == 0)); then pass=$((pass + 1)); else fail=$((fail + 1)); fi
          if ((rc == 124 || rc == 137)) || grep -q ' hang$' "$dir/mesa/summary.txt" 2>/dev/null; then hang=1; fi
        else
          echo "job $n first-job: skipped (no $prefix/bin/g15c-first-job)"
          skip=$((skip + 1))
        fi
        rm -rf "$out"
        ;;
      compute)
        rc=0
        timeout 60 env -u SUDO_USER ${env[@]+"${env[@]}"} "$M3MAX_KIT_JOB" --seconds 5 "$prefix" >"$dir/job-$n-compute.txt" 2>&1 || rc=$?
        echo "exit=$rc" >>"$dir/job-$n-compute.txt"
        rec=$(sed -n 's/.*Record: \(\/[^ ]*\.txt\)$/\1/p' "$dir/job-$n-compute.txt" | tail -1)
        if [[ -n $rec && -f $rec ]]; then cp "$rec" "$dir/job-$n-compute-record.txt"; fi
        echo "job $n compute: $(grep '^air-gpu-job: ' "$dir/job-$n-compute.txt" | tail -1 | cut -c1-200) (exit $rc)"
        if ((rc == 0)); then pass=$((pass + 1)); else fail=$((fail + 1)); fi
        if ((rc == 3 || rc == 5 || rc == 7 || rc == 8)); then hang=1; fi
        ;;
      render)
        uid=$(id -u "$user" 2>/dev/null) || uid=""
        sock=""
        if [[ -n $uid ]]; then
          sock=$(find "$M3MAX_KIT_RUNDIR/$uid" -maxdepth 1 -type s -name 'wayland-*' 2>/dev/null | head -1)
        fi
        if [[ -x $prefix/bin/mesa-m3-probe && -n $sock ]]; then
          rc=0
          timeout 100 runuser -u "$user" -- env XDG_RUNTIME_DIR="$M3MAX_KIT_RUNDIR/$uid" WAYLAND_DISPLAY="${sock##*/}" \
            ${env[@]+"${env[@]}"} "$prefix/bin/mesa-m3-probe" >"$dir/job-$n-render.txt" 2>&1 || rc=$?
          echo "exit=$rc" >>"$dir/job-$n-render.txt"
          echo "job $n render: $(grep -E '^(result|gl\.renderer|gl\.render)=' "$dir/job-$n-render.txt" | tr '\n' ' ' | cut -c1-300) (exit $rc)"
          if ((rc == 0)) && grep -qx 'result=pass' "$dir/job-$n-render.txt"; then pass=$((pass + 1)); else fail=$((fail + 1)); fi
        else
          echo "job $n render: skipped (no desktop session of $user, or no $prefix/bin/mesa-m3-probe)"
          skip=$((skip + 1))
        fi
        ;;
    esac
  done
  echo "== the GPU's packages and nodes"
  pacman -Q mesa-m3-g15c glibc vulkan-icd-loader libglvnd 2>&1 || true
  ls -l "$M3_DEVFS/dri" "$M3_DEVFS/dri/by-path" 2>&1 || true
  cat "$M3_SYSFS"/class/drm/renderD*/device/uevent 2>/dev/null || true
  echo "jobs: passed=$pass failed=$fail skipped=$skip"
}

# Collects plan boot INDEX in this boot: the report, the stage's checks, its outcome.
m3max_kit_collect_stage() { # INDEX
  local index=$1 name kind dir verdict params
  read -r name kind params <<<"$(m3max_plan_boot "$index")"
  dir=$(m3max_kit_stage_dir "$index")
  mkdir -p "$dir"
  {
    echo "stage: ${dir##*/} ($kind)"
    echo "collected: $(date -Is), kernel $(uname -r), boot $(tr -d '-' <"$M3_PROCFS/sys/kernel/random/boot_id" 2>/dev/null || echo -)"
    echo "command line: $(cat "$M3MAX_KIT_CMDLINE" 2>/dev/null)"
  } >>"$dir/meta.txt"
  m3max_kit_log "collecting ${dir##*/}"
  if [[ $kind == display ]]; then sleep "$M3MAX_KIT_DISPLAY_SETTLE"; fi
  # The jobs run first, so the report's kernel log has what they did.
  if [[ $kind == jobs ]]; then m3max_kit_jobs "$dir" >"$dir/jobs.txt" 2>&1 || true; fi
  m3max_kit_collect "$dir"
  {
    case $kind in
      data) m3max_kit_checks_data "$dir/report/kernel-log.txt" "$dir/report/m1n1-stage2-log.txt" ;;
      loader) m3max_kit_checks_loader "$dir" ;;
      display) m3max_kit_checks_display "$dir" ;;
      gpu) m3max_kit_checks_gpu "$dir" ;;
      jobs)
        cat "$dir/jobs.txt"
        m3max_kit_checks_gpu "$dir" | sed 's/^verdict: /gpu /'
        echo "verdict: $(sed -n 's/^jobs: //p' "$dir/jobs.txt" | tail -1)"
        ;;
    esac
  } >"$dir/checks.txt" 2>&1 || true
  verdict=$(sed -n 's/^verdict: //p' "$dir/checks.txt" | tail -1)
  m3max_kit_outcome "$index" collected "$verdict"
  if [[ $kind == gpu && $verdict == *gpu-start=success* && -z $(m3max_kit_get gpu_ok) ]]; then
    m3max_kit_set gpu_ok "$index"
  fi
  if [[ $kind == loader && $verdict == *probe-result=1*gpu-node=okay* ]]; then
    m3max_kit_set loader_ok 1
  fi
}

# The boot the journal knows by arming id ID (its kernel command line carries the tag), or
# nothing.
m3max_kit_journal_boot() { # ID SINCE-EPOCH
  journalctl -k -o json --no-pager --since "@$2" -g "${M3MAX_KIT_TAG//./\\.}=$1( |\$)" 2>/dev/null |
    python3 -I -c '
import json, sys
for line in sys.stdin:
    try:
        entry = json.loads(line)
    except ValueError:
        continue
    if "Kernel command line:" in str(entry.get("MESSAGE", "")) and entry.get("_BOOT_ID"):
        print(entry["_BOOT_ID"])
        break
' 2>/dev/null || true
}

# The armed boot of plan boot INDEX (arming id ID) was not collected: this is a normal boot.
# Records why (hung, hung-early, or not-booted when Limine never took the one-shot) with that
# boot's journal when there is one. Returns 1 when the kit can't go on (not-booted).
m3max_kit_missed() { # ID INDEX
  local id=$1 index=$2 dir since bid detail
  dir=$(m3max_kit_stage_dir "$index")
  mkdir -p "$dir"
  if [[ $(m3max_kit_oneshot armed 2>/dev/null) == "$M3MAX_KIT_ENTRY" ]]; then
    m3max_kit_outcome "$index" not-booted "the one-shot was still set at the next boot: Limine did not boot it, so the kit can't arm its test boots on this Mac"
    return 1
  fi
  since=$(stat -c %Y "$M3MAX_KIT_STATE/oneshot/armed" 2>/dev/null) || since=$(($(date +%s) - 86400))
  bid=$(m3max_kit_journal_boot "$id" "$since")
  if [[ -n $bid ]]; then
    journalctl -k -b "$bid" -o short-monotonic --no-pager >"$dir/previous-boot-kernel-log.txt" 2>&1 || true
    journalctl -b "$bid" -o short-monotonic --no-pager 2>/dev/null | tail -n 5000 >"$dir/previous-boot-journal.txt" || true
    detail="its journal is in previous-boot-kernel-log.txt and previous-boot-journal.txt"
  else
    detail="it left no journal"
  fi
  if [[ -f $M3MAX_KIT_STATE/reached-$id ]]; then
    cp "$M3MAX_KIT_STATE/reached-$id" "$dir/reached.txt"
    m3max_kit_outcome "$index" hung "the boot reached userspace ($(cut -c1-60 "$dir/reached.txt")) and ended before the kit collected it: a hang, a panic or a power cycle; $detail"
  else
    m3max_kit_outcome "$index" hung-early "no sign that the boot reached userspace: it hung or was switched off while the kernel started; $detail"
  fi
  return 0
}

# ---- the run, boot by boot --------------------------------------------------------------------
m3max_kit_lock() {
  exec {M3MAX_KIT_LOCK_FD}>>"$M3MAX_KIT_STATE/lock" || die "could not open $M3MAX_KIT_STATE/lock"
  flock -w "${1:-900}" "$M3MAX_KIT_LOCK_FD" || die "the kit is busy (another aurora-m3max-kit holds $M3MAX_KIT_STATE/lock); try again in a few minutes"
}

# At every boot (aurora-m3max-kit.service): collect this boot, then arm the next one.
m3max_kit_step() {
  local phase tag armed id index
  [[ -f $M3MAX_KIT_STATE/status ]] || return 0
  m3max_kit_lock
  phase=$(m3max_kit_get phase)
  case $phase in
    running | finishing) ;;
    *) m3max_kit_log "this boot: the kit is ${phase:-not set up}; nothing to do"; return 0 ;;
  esac
  trap 'm3_report_cleanup' EXIT
  # A GPU boot's status file and crash log first: devcoredump does not keep them for long.
  local early
  early=$(m3max_kit_tagged_stage)
  if [[ -n $early ]]; then m3max_kit_gpu_early "$early"; fi
  m3max_kit_wait_boot
  tag=$(m3max_kit_boot_tag)
  m3max_kit_set restart ""
  if [[ $phase == finishing ]]; then
    if [[ -n $tag ]]; then
      m3max_kit_log "this boot carries arming id $tag while the kit finishes; restarting into a normal boot"
      m3max_kit_restart "a normal boot to finish"
      return 0
    fi
    m3max_kit_finish
    return 0
  fi
  if [[ $(uname -r) != "$(m3max_kit_get kernel)" ]]; then
    m3max_kit_log "this boot runs $(uname -r), not the kit's kernel $(m3max_kit_get kernel) (another boot menu entry?): the kit waits for a boot of its kernel and arms nothing"
    m3max_kit_notify "This boot runs another kernel than the test kit's, so the kit waits. Restart and pick the default entry to go on, or: sudo aurora-m3max-kit --stop"
    return 0
  fi
  armed=$(m3max_kit_get armed)
  if [[ -n $armed && $armed == loader:* ]]; then
    # The loader stage is not a one-shot: this boot is its boot when boot.bin is still the one
    # with the GPU handoff on. The boot.bin from before the kit means it was put back from macOS.
    read -r id index <<<"$armed"
    m3max_kit_set armed ""
    case $(m3max_kit_bootbin_name) in
      "the kit's, GPU handoff on")
        m3max_kit_collect_stage "$index"
        ;;
      "the boot.bin from before the kit")
        m3max_kit_outcome "$index" hung "the Mac did not come back with the boot loader's GPU handoff on, and the boot.bin from before the kit was put back from macOS"
        m3max_kit_abandon "the boot loader's GPU handoff stage did not come back"
        return 0
        ;;
      *)
        m3max_kit_outcome "$index" failed "boot.bin (sha256 $(m3_bootbin_sha)) is neither the one the kit built with the GPU handoff on nor the one from before the kit"
        m3max_kit_abandon "boot.bin changed outside the kit during the boot loader's GPU handoff stage"
        return 0
        ;;
    esac
    m3max_kit_advance "$tag"
    return 0
  fi
  if [[ -z $armed && $(m3max_kit_get next) == 0 ]]; then
    # The first boot after the install: the variant's, unless it did not boot and the boot.bin
    # from before the kit was put back from macOS (its m1n1 is then recorded as failed here).
    case $(m3max_kit_bootbin_name) in
      "the boot.bin from before the kit")
        m3max_kit_outcome 0 hung "the boot loader variant did not boot, and the boot.bin from before the kit was put back from macOS"
        m1n1_failed_add "$M1N1_BIN_SHA" "${M1N1_PACKAGE%% *}: the M3 Max test kit's variant did not boot, found $(date +%F)"
        m3max_kit_abandon "the boot loader variant did not boot"
        return 0
        ;;
      "not one the kit knows")
        m3max_kit_outcome 0 failed "boot.bin (sha256 $(m3_bootbin_sha)) is not the variant the kit installed"
        m3max_kit_abandon "boot.bin changed outside the kit"
        return 0
        ;;
    esac
  fi
  if [[ -n $armed ]]; then
    read -r id index <<<"$armed"
    if [[ -n $tag && $tag == "$id" ]]; then
      m3max_kit_collect_stage "$index"
    elif ! m3max_kit_missed "$id" "$index"; then
      m3max_kit_set armed ""
      m3max_kit_disarm || true
      m3max_kit_skip_rest "the kit's one-shot boots do not work on this Mac"
      m3max_kit_begin_finish ""
      return 0
    fi
    m3max_kit_set armed ""
    m3max_kit_disarm || m3max_kit_log "warning: could not remove the kit's boot entry; the next arming replaces it"
  elif [[ $(m3max_kit_get next) == 0 ]]; then
    m3max_kit_collect_stage 0
    m3max_kit_set next 1
  elif [[ -n $tag ]]; then
    m3max_kit_log "this boot carries arming id $tag, which the kit has no record of; it collects nothing from it"
  fi
  m3max_kit_advance "$tag"
}

# Marks every plan boot from the next one on as skipped, for WHY.
m3max_kit_skip_rest() { # WHY
  local i count
  count=$(m3max_plan_count)
  for ((i = $(m3max_kit_get next); i < count; i++)); do
    [[ -f $(m3max_kit_stage_dir "$i")/outcome.txt ]] || m3max_kit_outcome "$i" skipped "$1"
  done
  m3max_kit_set next "$count"
}

# Arms the plan's next boot that applies and schedules the restart into it; or, when there is
# none, finishes (on a normal boot) or restarts into a normal boot to finish.
m3max_kit_advance() { # THIS-BOOT'S-TAG
  local index count max boots name kind params ok prefix w free
  local -a prm=()
  count=$(m3max_plan_count) max=$(m3max_plan max-boots)
  while :; do
    index=$(m3max_kit_get next)
    if ((index >= count)); then
      m3max_kit_begin_finish "$1"
      return 0
    fi
    read -r name kind params <<<"$(m3max_plan_boot "$index")"
    ok=$(m3max_kit_get gpu_ok)
    case $kind in
      data)
        m3max_kit_set next $((index + 1))
        continue
        ;;
      loader)
        boots=$(m3max_kit_get boots)
        if ((${boots:-0} >= max)); then
          m3max_kit_skip_rest "the plan's limit of $max experimental boots was reached"
          continue
        fi
        m3max_kit_arm_loader "$index" "$name" && return 0
        m3max_kit_set next $((index + 1))
        continue
        ;;
      gpu)
        # The GPU start needs the GPU node m1n1's GPU handoff fills: after a loader boot that
        # passed, when the plan has one.
        if m3max_plan_boots | awk '$2 == "loader" { f = 1 } END { exit !f }' && [[ -z $(m3max_kit_get loader_ok) ]]; then
          m3max_kit_outcome "$index" skipped "the boot loader's GPU handoff did not pass (see the loader stage)"
          m3max_kit_set next $((index + 1))
          continue
        fi
        if [[ -n $ok ]]; then
          m3max_kit_outcome "$index" skipped "GPU boot $(m3max_kit_stage_dir "$ok" | sed 's#.*/##') succeeded already"
          m3max_kit_set next $((index + 1))
          continue
        fi
        ;;
      jobs)
        prefix=$(m3max_plan mesa-prefix)
        if [[ -z $ok && -n $(m3max_kit_get nostart) ]]; then
          m3max_kit_outcome "$index" skipped "kernel has no start experiment, so there is no GPU to run jobs on"
          m3max_kit_set next $((index + 1))
          continue
        fi
        if [[ -z $ok ]]; then
          m3max_kit_outcome "$index" skipped "no GPU boot succeeded"
          m3max_kit_set next $((index + 1))
          continue
        fi
        if ! compgen -G "$prefix/share/vulkan/icd.d/*.json" >/dev/null; then
          m3max_kit_outcome "$index" skipped "no Mesa with a Vulkan driver in $prefix"
          m3max_kit_set next $((index + 1))
          continue
        fi
        read -r _ _ w <<<"$(m3max_plan_boot "$ok")"
        params=${params//@gpu/$w}
        ;;
    esac
    boots=$(m3max_kit_get boots)
    if ((${boots:-0} >= max)); then
      m3max_kit_skip_rest "the plan's limit of $max experimental boots was reached"
      continue
    fi
    free=$(m3max_kit_free_mb "$M3MAX_KIT_STATE")
    if [[ -n $free ]] && ((free < M3MAX_KIT_MIN_MB / 2)); then
      m3max_kit_skip_rest "only $free MB free for the kit's results"
      continue
    fi
    read -ra prm <<<"$params"
    # A boot whose parameters the test kernel does not have is skipped without spending a boot:
    # a kernel without the GPU start experiment (asahi.t6031_start) skips the GPU boots, and
    # with them the jobs, and the kit goes on to the final boot.
    w=$(m3max_kit_oneshot missing "${prm[@]}" 2>/dev/null) || w=""
    if [[ -n $w ]]; then
      if [[ $kind == gpu && $'\n'$w$'\n' == *$'\n'asahi.t6031_start$'\n'* ]]; then
        m3max_kit_outcome "$index" skipped "kernel has no start experiment (the test kernel has no asahi.t6031_start)"
        m3max_kit_set nostart 1
      else
        m3max_kit_outcome "$index" skipped "the test kernel has no ${w//$'\n'/, }"
      fi
      m3max_kit_set next $((index + 1))
      continue
    fi
    if ! m3max_kit_arm "$name" "${prm[@]}"; then
      m3max_kit_outcome "$index" not-armed "$M3MAX_KIT_WHY"
      m3max_kit_set next $((index + 1))
      continue
    fi
    m3max_kit_set armed "$M3MAX_KIT_ARMED_ID $index" next $((index + 1)) boots $((${boots:-0} + 1))
    mkdir -p "$(m3max_kit_stage_dir "$index")"
    printf 'armed: %s, id %s, parameters: %s\n' "$(date -Is)" "$M3MAX_KIT_ARMED_ID" "$params" >>"$(m3max_kit_stage_dir "$index")/meta.txt"
    m3max_kit_log "armed ${name} ($kind) for the next boot: id $M3MAX_KIT_ARMED_ID, $params"
    m3max_kit_restart "test boot $((index + 1)) of $count ($name)"
    return 0
  done
}

# Nothing more to arm. The end state of a run whose boots all came back: the variant with the
# GPU handoff off (proven to boot on this Mac), so a boot.bin the loader stage left with it on is
# rebuilt with it off first (or, if that fails, the boot.bin from before the kit goes back). Then
# finish on this boot when it is a normal one on that boot.bin, else restart into one.
m3max_kit_begin_finish() { # THIS-BOOT'S-TAG
  local changed=0
  m3max_kit_set phase finishing
  if [[ $(m3max_kit_get loader) == on ]]; then
    changed=1
    if ! m3max_kit_loader off; then
      m3max_kit_log "could not rebuild boot.bin with the GPU handoff off ($M3MAX_KIT_WHY); putting back the boot.bin from before the kit"
      m3max_kit_unvariant || m3max_kit_log "warning: the boot.bin from before the kit could not be put back either; sudo aurora-m3max-kit --restore"
    fi
  fi
  if [[ -n $1 ]] || ((changed)); then
    m3max_kit_restart "a normal boot, to finish"
  else
    m3max_kit_finish
  fi
}

# A boot-loader-level failure (the variant did not boot, or the loader stage did not come back,
# and the boot.bin from before the kit was put back from macOS): the rest is skipped, the
# kernel-only state around that boot.bin is restored, and the kit finishes on this boot.
m3max_kit_abandon() { # WHY
  m3max_kit_skip_rest "$1"
  m3max_kit_unvariant || m3max_kit_log "warning: the kernel-only state could not be restored; sudo aurora-m3max-kit --restore"
  m3max_kit_set armed "" phase finishing
  m3max_kit_finish
}

# The boot loader GPU stage (plan kind loader): the interim results are packed (in the home folder
# and on the EFI partition, where macOS reaches them), the restore steps refreshed (likewise), and
# boot.bin rebuilt with m1n1's GPU handoff on; then a normal boot after a longer, announced delay.
# Returns 1 when it was not armed (the stage then says why).
m3max_kit_arm_loader() { # INDEX NAME
  local index=$1 name=$2 boots sha
  boots=$(m3max_kit_get boots)
  m3max_kit_interim || m3max_kit_log "warning: the interim results could not be packed"
  m3max_kit_restore_files
  if ! m3max_kit_loader on; then
    m3max_kit_outcome "$index" not-armed "boot.bin could not be rebuilt with the GPU handoff on: $M3MAX_KIT_WHY"
    return 1
  fi
  sha=$(m3_bootbin_sha)
  m3max_kit_set armed "loader:$sha $index" next $((index + 1)) boots $((${boots:-0} + 1))
  mkdir -p "$(m3max_kit_stage_dir "$index")"
  printf 'armed: %s, boot.bin with %s, sha256 %s\n' "$(date -Is)" "$M3_MAX_SWITCHES" "$sha" >>"$(m3max_kit_stage_dir "$index")/meta.txt"
  m3max_kit_log "armed $name (loader): boot.bin rebuilt with the GPU handoff on, sha256 $sha"
  m3max_kit_restart "test boot $((index + 1)) of $(m3max_plan_count) ($name): the boot loader's GPU handoff.
    If the Mac does not come back to the desktop, put the boot loader back from macOS: the steps are
    in ~/aurora-m3max-kit-RESTORE.txt and in AURORA-M3MAX-KIT-RESTORE.txt on the EFI partition, and
    the results so far are in $(m3max_kit_get interim)" loader
  return 0
}

# Rebuilds boot.bin with m1n1's GPU handoff on (M3_MAX_SWITCHES) or off (M3_MAX_SAFE_SWITCHES) and
# checks it (update-m1n1, m3_verify_bootbin). A failed "on" goes back to "off". Sets loader and
# bootbin, or M3MAX_KIT_WHY and returns 1.
m3max_kit_loader() { # on|off
  local rc=0
  M3MAX_KIT_WHY=""
  if [[ $1 == on ]]; then M3_MAX_GPU_HANDOFF=1; else M3_MAX_GPU_HANDOFF=0; fi
  M3_MODE=handoff
  m3_switches_write
  if update_m1n1_frozen; then
    M3MAX_KIT_WHY="update-m1n1 is frozen in $UPDATE_M1N1_CONF"
    rc=1
  elif ! $sudo update-m1n1 >>"$M3MAX_KIT_STATE/log" 2>&1; then
    M3MAX_KIT_WHY="update-m1n1 failed (see the kit's log)"
    rc=1
  elif ! (m3_verify_bootbin) >>"$M3MAX_KIT_STATE/log" 2>&1; then
    M3MAX_KIT_WHY="the rebuilt boot.bin did not check out (see the kit's log)"
    rc=1
  fi
  if ((rc)) && [[ $1 == on ]]; then
    M3_MAX_GPU_HANDOFF=0
    m3_switches_write
    $sudo update-m1n1 >>"$M3MAX_KIT_STATE/log" 2>&1 || true
    m3max_kit_set loader off bootbin "$(m3_bootbin_sha)"
  fi
  M3_MAX_GPU_HANDOFF=0
  ((rc == 0)) || return 1
  m3max_kit_set loader "$1" bootbin "$(m3_bootbin_sha)"
  m3max_kit_log "boot.bin rebuilt with m1n1's GPU handoff $1 (sha256 $(m3max_kit_get bootbin))"
}

# Back to the kernel-only state this Mac had before the kit: the boot.bin from before the variant
# (when this one is not it), update-m1n1 frozen with no DTBS block, no switch block, m3-mode kernel.
m3max_kit_unvariant() {
  if [[ -f $STATE/m3max-bootbin-backup ]]; then
    m3max_bootbin_restore || return 1
    m3max_conf_kernel_only
    m3_switches_remove
    echo kernel | $sudo tee "$STATE/m3-mode" >/dev/null
  fi
  m3max_kit_set loader off
  m3max_kit_esp_files remove
}

# The interim results: everything collected so far, packed like the final file, before any GPU
# stage; a copy on the EFI partition, which macOS can read if Linux does not come back.
m3max_kit_interim() {
  local esp
  m3max_kit_pack interim || return 1
  if esp=$(m3max_kit_esp); then
    if ! { cp "$(m3max_kit_get interim_state)" "$esp/m1n1/aurora-m3max-kit-interim.tgz.new" &&
      sync && mv -f "$esp/m1n1/aurora-m3max-kit-interim.tgz.new" "$esp/m1n1/aurora-m3max-kit-interim.tgz" && sync; }; then
      rm -f "$esp/m1n1/aurora-m3max-kit-interim.tgz.new"
      m3max_kit_log "warning: could not copy the interim results to the EFI partition"
    fi
  fi
  m3max_kit_log "interim results: $(m3max_kit_get interim)"
  m3max_kit_notify "Interim results, enough to upload on their own: $(m3max_kit_get interim)"
}

# The EFI partition (where m1n1's boot.bin is), or 1.
m3max_kit_esp() {
  local target
  target=$(esp_bootbin) || return 1
  echo "${target%/m1n1/boot.bin}"
}

# The restore steps in the desktop user's home and on the EFI partition (refreshed before the
# loader stage, removed with the interim copy when the kit ends or is undone).
m3max_kit_restore_files() {
  local user home esp tmp
  tmp=$(mktemp)
  m3max_kit_restore_text >"$tmp"
  chmod 0644 "$tmp"
  user=$(m3max_kit_user)
  home=$(m3max_kit_home "$user")
  if [[ $user != root && -d $home ]]; then
    runuser -u "$user" -- cp -- "$tmp" "$home/aurora-m3max-kit-RESTORE.txt" ||
      m3max_kit_log "warning: could not save the restore steps in $home"
  fi
  if esp=$(m3max_kit_esp); then
    { cp "$tmp" "$esp/AURORA-M3MAX-KIT-RESTORE.txt.new" && sync &&
      mv -f "$esp/AURORA-M3MAX-KIT-RESTORE.txt.new" "$esp/AURORA-M3MAX-KIT-RESTORE.txt" && sync; } ||
      m3max_kit_log "warning: could not save the restore steps on the EFI partition"
  fi
  rm -f "$tmp"
}

# Removes the kit's files from the EFI partition (the restore steps and the interim copy).
m3max_kit_esp_files() { # remove
  local esp
  esp=$(m3max_kit_esp) || return 0
  rm -f "$esp/AURORA-M3MAX-KIT-RESTORE.txt" "$esp/m1n1/aurora-m3max-kit-interim.tgz"
}

# The last, normal boot: collect it, pack everything, and say which file to upload.
m3max_kit_finish() {
  local dir
  dir=$(m3max_kit_stage_dir final)
  mkdir -p "$dir"
  m3max_kit_collect "$dir"
  { m3max_kit_display_state; echo "verdict: normal boot, kernel $(uname -r)"; } >"$dir/checks.txt" 2>&1 || true
  m3max_kit_outcome final collected "the normal boot after the test boots, on boot.bin sha256 $(m3_bootbin_sha) ($(m3max_kit_bootbin_name))"
  m3max_kit_units off
  m3max_kit_esp_files remove
  m3max_kit_set phase "done"
  if m3max_kit_pack; then
    m3max_kit_log "done: upload $(m3max_kit_get tarball)"
    m3max_kit_notify "Done. Please upload this file: $(m3max_kit_get tarball)"
  else
    m3max_kit_log "done, but the results could not be packed (see above); sudo aurora-m3max-kit --status"
    m3max_kit_notify "The test kit finished, but could not pack its results. Run: sudo aurora-m3max-kit --status"
  fi
}

# Schedules the restart (shutdown -r, which warns the logged-in users) after the plan's delay (the
# loader delay for the loader stage).
m3max_kit_restart() { # WHAT [loader]
  local delay mins
  delay=$(m3max_plan reboot-delay)
  if [[ ${2:-} == loader ]]; then delay=$(m3max_plan loader-delay); fi
  mins=$(((${delay:-60} + 59) / 60))
  m3max_kit_set restart "in $mins min, for $1"
  m3max_kit_log "restarting in $mins min for $1"
  if ! shutdown -r "+$mins" "M3 Max test kit: this Mac restarts in $mins min for $1. To stop the kit: sudo aurora-m3max-kit --stop" >/dev/null 2>&1; then
    m3max_kit_log "warning: shutdown -r failed; restart the Mac by hand to go on"
  fi
  m3max_kit_notify "Restarting in $mins min for $1. To stop the kit: sudo aurora-m3max-kit --stop"
}

# What this boot.bin is: the kit's (GPU handoff on or off), the one from before the kit, or other.
m3max_kit_bootbin_name() {
  local now orig
  now=$(m3_bootbin_sha)
  orig=$(m3max_kit_get orig_bootbin)
  if [[ -z $orig && -f $STATE/m3max-bootbin-backup ]]; then read -r _ orig <"$STATE/m3max-bootbin-backup"; fi
  if [[ -n $orig && $now == "$orig" ]]; then echo "the boot.bin from before the kit"
  elif [[ $now == "$(m3max_kit_get bootbin)" ]]; then echo "the kit's, GPU handoff $(m3max_kit_get loader)"
  else echo "not one the kit knows"
  fi
}

# A desktop notification for every logged-in user with a session bus (best effort).
m3max_kit_notify() {
  local bus uid user
  for bus in "$M3MAX_KIT_RUNDIR"/*/bus; do
    [[ -S $bus ]] || continue
    uid=${bus#"$M3MAX_KIT_RUNDIR"/}
    uid=${uid%/bus}
    # shellcheck disable=SC2015 # any of them failing skips this bus
    [[ $uid =~ ^[0-9]+$ ]] && ((uid >= 1000)) && user=$(id -nu "$uid" 2>/dev/null) || continue
    timeout 5 runuser -u "$user" -- env DBUS_SESSION_BUS_ADDRESS="unix:path=$bus" \
      notify-send -u critical -a "M3 Max test kit" "M3 Max test kit" "$1" >/dev/null 2>&1 || true
  done
  return 0
}

# The kit's systemd units on (enabled for the next boots) or off.
m3max_kit_units() { # on|off
  if [[ $1 == on ]]; then
    $sudo systemctl daemon-reload
    $sudo systemctl enable "$M3MAX_KIT_UNIT" "$M3MAX_KIT_MARK_UNIT" >/dev/null 2>&1 ||
      die "could not enable the kit's systemd units"
  else
    $sudo systemctl disable "$M3MAX_KIT_UNIT" "$M3MAX_KIT_MARK_UNIT" >/dev/null 2>&1 || true
  fi
}

# --mark (aurora-m3max-kit-mark.service, early in a boot whose command line carries the tag):
# this armed boot reached userspace.
m3max_kit_mark() {
  local id
  id=$(m3max_kit_boot_tag)
  [[ $id =~ ^[0-9A-Za-z-]{1,32}$ && -d $M3MAX_KIT_STATE ]] || return 0
  printf 'reached userspace %s, uptime %s s\n' "$(date -Is)" "$(cut -d' ' -f1 "$M3_PROCFS/uptime" 2>/dev/null)" \
    >"$M3MAX_KIT_STATE/reached-$id"
  sync
  # A GPU boot's status file and crash log, before anything else can happen to them.
  local stage
  stage=$(m3max_kit_tagged_stage)
  if [[ -n $stage ]]; then m3max_kit_gpu_early "$stage"; fi
}

# ---- the result ---------------------------------------------------------------------------------
# What the kit has done, for --status and for kit-status.txt in the tgz.
m3max_kit_status_text() {
  local i count name kind phase o armed
  phase=$(m3max_kit_get phase)
  count=$(m3max_plan_count)
  echo "M3 Max test kit ($(m3max_kit_get board)): ${phase:-not set up}"
  echo "started: $(m3max_kit_get started); kernel: $(m3max_kit_get kernel); boot loader variant: $M3_MAX_VARIANT"
  echo "experimental boots armed: $(m3max_kit_get boots) of at most $(m3max_plan max-boots)"
  armed=$(m3max_kit_get armed)
  echo "stages:"
  o=$(cat "$(m3max_kit_stage_dir baseline)/outcome.txt" 2>/dev/null || echo -)
  printf '  00 %-12s %-8s %s\n' baseline report "$o"
  for ((i = 0; i < count; i++)); do
    read -r name kind _ <<<"$(m3max_plan_boot "$i")"
    o=$(cat "$(m3max_kit_stage_dir "$i")/outcome.txt" 2>/dev/null || echo -)
    if [[ $o == - && -n $armed && ${armed#* } == "$i" ]]; then o="armed for the next boot (id ${armed%% *})"; fi
    printf '  %02d %-12s %-8s %s\n' "$((i + 1))" "$name" "$kind" "$o"
  done
  o=$(cat "$(m3max_kit_stage_dir final)/outcome.txt" 2>/dev/null || echo -)
  printf '  99 %-12s %-8s %s\n' final normal "$o"
  [[ -z $(m3max_kit_get restart) ]] || echo "restart: $(m3max_kit_get restart)"
  [[ -z $(m3max_kit_get tarball) ]] || echo "result: $(m3max_kit_get tarball)"
  return 0
}

# Packs the plan, the status, the kit's log and every stage into one tgz, masked and checked as
# --m3-report's is (m3_privacy_pack), and puts a copy in the desktop user's home, written as that
# user. Sets tarball (with "interim": interim, and interim_state for the state directory's copy).
# Returns 1 when nothing could be kept.
m3max_kit_pack() { # [interim]
  local work dir name user home f pub key=tarball
  local -a journals=()
  user=$(m3max_kit_user)
  home=$(m3max_kit_home "$user")
  name=aurora-m3max-kit-$(m3max_kit_get board)-$(date +%Y%m%d-%H%M%S).tgz
  if [[ ${1:-} == interim ]]; then
    name=aurora-m3max-kit-$(m3max_kit_get board)-$(date +%Y%m%d-%H%M%S)-interim.tgz
    key=interim
  fi
  work=$(mktemp -d)
  dir=$work/kit
  mkdir -p "$dir"
  if [[ -d $M3MAX_KIT_STATE/stages ]]; then cp -a "$M3MAX_KIT_STATE/stages" "$dir/"; fi
  m3max_kit_status_text >"$dir/kit-status.txt" 2>&1 || true
  cp "$M3MAX_KIT_PLAN_FILE" "$dir/plan.txt" 2>/dev/null || true
  cp "$M3MAX_KIT_STATE/log" "$dir/kit-log.txt" 2>/dev/null || true
  cat >"$dir/README.txt" <<EOF
aurora-sep M3 Max test kit ($TAG), packed $(date -u +%Y-%m-%dT%H:%M:%SZ)
The host name is replaced by "host", user names by "user", serial numbers by SERIAL and MAC
addresses by xx:xx:xx:xx:xx:xx, as in --m3-report.
  kit-status.txt    every stage and how it ended (collected, hung, hung-early, not-booted,
                    not-armed, skipped, cancelled), with its verdict
  plan.txt          the plan the kit ran: each boot's kernel parameters
  kit-log.txt       the kit's own log
  stages/NN-NAME/   one directory per stage: 00-baseline (before the install), then each boot
                    of the plan, then 99-final (the normal boot after them)
    report/         the --m3-report collection of that boot (see its README.txt)
    checks.txt      what the stage looked at, and its verdict
    outcome.txt     how it ended
    meta.txt        when it was armed and collected, and the boot's command line
    previous-boot-*.txt  for a boot that was not collected: its kernel log and journal, when
                    the journal kept them
    job-*.txt       the jobs boot's GPU jobs and their records
EOF
  for f in "$dir"/stages/*/kernel-log-source.txt; do
    [[ -f $f && $(cat "$f") == journal && -f ${f%/kernel-log-source.txt}/report/kernel-log.txt ]] &&
      journals+=("${f%/kernel-log-source.txt}/report/kernel-log.txt")
  done
  for f in "$dir"/stages/*/previous-boot-*.txt; do [[ -f $f ]] && journals+=("$f"); done
  m3_privacy_secrets >"$work/secrets"
  if ! m3_privacy_pack "$dir" "$M3MAX_KIT_STATE/$name" "$work/secrets" ${journals[@]+"${journals[@]}"}; then
    rm -rf "$work"
    return 1
  fi
  m3max_kit_set "$key" "$M3MAX_KIT_STATE/$name" "${key}_state" "$M3MAX_KIT_STATE/$name"
  if [[ $user != root && -d $home ]]; then
    pub=$(mktemp --tmpdir aurora-m3max-kit.XXXXXX)
    cp "$M3MAX_KIT_STATE/$name" "$pub" && chmod 0644 "$pub"
    # As the user, so a link in the home can only point where the user may write.
    if runuser -u "$user" -- cp -- "$pub" "$home/$name" 2>/dev/null; then
      m3max_kit_set "$key" "$home/$name"
    else
      m3max_kit_log "warning: could not copy the result to $home; it is in $M3MAX_KIT_STATE/$name"
    fi
    rm -f "$pub"
  fi
  rm -rf "$work"
}

# ---- --status, --stop, --restore --------------------------------------------------------------
m3max_kit_status() {
  if [[ ! -f $M3MAX_KIT_STATE/status ]]; then
    say "the M3 Max test kit has not run on this Mac (no $M3MAX_KIT_STATE/status)"
    return 0
  fi
  m3max_kit_status_text
  echo
  case $(m3max_kit_get phase) in
    done | stopped | restored)
      if [[ -n $(m3max_kit_get tarball) ]]; then
        echo "Please upload this file (drag it into a comment on the issue): $(m3max_kit_get tarball)"
      fi
      ;;
    *)
      echo "The kit is running and restarts this Mac by itself. To stop it: sudo aurora-m3max-kit --stop"
      echo "To stop it and put back the boot loader this Mac had: sudo aurora-m3max-kit --restore"
      ;;
  esac
}

# Cancels the kit's scheduled restart and its armed boot (that stage: cancelled), and switches
# its units off. Needs the lock.
m3max_kit_halt() {
  local armed id index
  shutdown -c >/dev/null 2>&1 || true
  m3max_kit_set restart ""
  armed=$(m3max_kit_get armed)
  if [[ -n $armed ]]; then
    read -r id index <<<"$armed"
    if [[ $(m3max_kit_boot_tag) != "$id" ]]; then
      m3max_kit_outcome "$index" cancelled "the kit was stopped before this boot ran"
    fi
    m3max_kit_set armed ""
  fi
  m3max_kit_disarm || die "could not clear the kit's one-shot boot (see $M3MAX_KIT_STATE/log). Run this again."
  m3max_kit_units off
  # A boot.bin the loader stage left with m1n1's GPU handoff on goes back to the variant without it.
  if [[ $(m3max_kit_get loader) == on ]] && ! m3max_kit_loader off; then
    warn "could not rebuild boot.bin with the GPU handoff off ($M3MAX_KIT_WHY); sudo aurora-m3max-kit --restore puts back the one from before the kit"
  fi
}

# --stop: arm nothing more, and pack the results so far.
m3max_kit_stop() {
  local phase
  if [[ ! -f $M3MAX_KIT_STATE/status ]]; then
    say "the M3 Max test kit has not run on this Mac; nothing to stop"
    return 0
  fi
  m3max_kit_lock
  phase=$(m3max_kit_get phase)
  case $phase in
    done | stopped | restored)
      say "the kit is $phase already; nothing to stop"
      m3max_kit_status_text
      return 0
      ;;
  esac
  m3max_kit_halt
  m3max_kit_set phase stopped
  m3max_kit_log "stopped by --stop"
  if m3max_kit_pack; then
    say "Stopped: nothing more is armed, and no restart is scheduled. The results so far:
    $(m3max_kit_get tarball)
    The boot loader variant stays; sudo aurora-m3max-kit --restore puts back the one this Mac had."
  else
    warn "stopped, but the results could not be packed (see above); they stay in $M3MAX_KIT_STATE"
  fi
}

# --restore: --stop, then put back the boot.bin this Mac booted before the kit (and the
# kernel-only state around it: the freeze on update-m1n1, no switch block, m3-mode kernel), take
# the kit's Mesa off, and check that the previous kernel's boot menu entry is there. The test
# kernel stays: its new paths are off unless a test boot switches them on.
m3max_kit_restore() {
  local had=0 phase="" name result=""
  if [[ -f $M3MAX_KIT_STATE/status ]]; then
    m3max_kit_lock
    phase=$(m3max_kit_get phase)
    had=1
    [[ $phase == restored ]] || m3max_kit_halt
  elif [[ -f $M3MAX_KIT_ONESHOT ]]; then
    m3max_kit_disarm || die "could not clear the kit's one-shot boot. Run this again."
  fi
  if [[ -f $STATE/m3max-bootbin-backup ]]; then
    m3max_kit_unvariant || die "the boot.bin this Mac booted before the kit could not be put back (see above).
    Nothing more was changed. The kit's boot entry and restarts are off."
  else
    say "boot.bin: nothing to put back (the kit did not change it, or it was put back already)"
    m3max_kit_esp_files remove
  fi
  name=$(m3max_kit_get mesa)
  if [[ -n $name ]] && pacman -Q "$name" >/dev/null 2>&1; then
    $sudo pacman -Rns --noconfirm "$name" || warn "could not remove $name; remove it with: sudo pacman -Rns $name"
  fi
  if [[ -f $STATE/m3-known-entry.json && $(boot_chain) == limine ]]; then
    (m3_keep_limine_entry retain) || warn "could not check the boot menu entry \"Aurora previous (GPU off)\""
  fi
  if ((had)); then
    if [[ $phase != restored && $phase != "done" ]]; then m3max_kit_pack || warn "the results could not be packed"; fi
    m3max_kit_set phase restored
    m3max_kit_log "restored by --restore"
    if [[ -n $(m3max_kit_get tarball) ]]; then result="
    The kit's results: $(m3max_kit_get tarball)"; fi
  fi
  say "Restored: the kit is off, and boot.bin is the one this Mac booted before the kit (update-m1n1 is
    frozen again, as on a kernel-only M3). The test kernel stays the default, with its new M3 Max
    paths off; the kernel this Mac ran before the kit is in the boot menu as \"Aurora previous
    (GPU off)\". To go back to the release kernel as well:
      curl -fsSL $LATEST_URL | bash$result"
}

# The steps that put the boot loader back, for the file the kit leaves in the user's home.
m3max_kit_restore_text() {
  local target keep uuid
  target=$(esp_bootbin) || target=/boot/efi/m1n1/boot.bin
  keep=$target.before-$VERSION
  uuid=$(findmnt -no PARTUUID --target "${target%/m1n1/boot.bin}" 2>/dev/null) || uuid="?"
  cat <<EOF
M3 Max test kit: how to undo it ($(date -u +%Y-%m-%d), $TAG)

From Linux, at any time (stops the kit, puts back the boot loader this Mac had):
  sudo aurora-m3max-kit --restore

If the Mac stops in the boot loader (m1n1 text on screen, often "No valid payload found", and no
boot menu), put the boot loader back from macOS:
  1. Hold the power button until the Mac turns off, then press and hold it again for the
     startup options, and start macOS (or Options, then Utilities > Terminal).
  2. In Terminal, run: diskutil list
     Find the partition whose UUID is $uuid (any case) with:
       diskutil info diskNsM | grep -i 'partition uuid'
  3. sudo diskutil mount diskNsM   (it prints the /Volumes path)
  4. cp -X '<that path>/m1n1/boot.bin.before-$VERSION' '<that path>/m1n1/boot.bin'
  Then start Linux and run: sudo aurora-m3max-kit --restore

After the boot loader is put back, start Linux: the kit sees it, records that boot as hung,
puts this Mac back as it was before the kit, and finishes by itself; then run
sudo aurora-m3max-kit --status for the file to upload. If Linux does not come back at all, the
results up to the boot loader test are on the same partition as
m1n1/aurora-m3max-kit-interim.tgz (copy it off from macOS and upload it).

If a test boot freezes or the screen stays black for more than 5 minutes: hold the power
button until the Mac turns off, then press it again. The next boot is always the normal one,
and the kit records that test boot as hung and goes on by itself. This is true of every test
boot but one: the boot loader GPU test changes boot.bin, so if the Mac does not come back from
it, put the boot loader back from macOS as above.

If the normal boot itself fails, pick "Aurora previous (GPU off)" in the boot menu: the kernel
this Mac ran before the kit. The kept boot loader is $keep.
EOF
}

# ---- setting the kit up (--m3max-kit) ------------------------------------------------------------
# The work directory of the setup: the kit's downloaded files and this script's copy.
M3MAX_KIT_WORK=""

# Downloads a kit file ("file sha256") into the work directory and checks it. Dies otherwise.
m3max_kit_fetch() { # "FILE SHA256"
  local file sha rc=0
  read -r file sha <<<"$1"
  [[ -n $file && $sha =~ ^[0-9a-f]{64}$ ]] || die "this release names no checked copy of a kit file (${1:-empty}). Nothing was changed."
  if [[ -f $M3MAX_KIT_WORK/$file && $(sha256sum "$M3MAX_KIT_WORK/$file" | cut -d' ' -f1) == "$sha" ]]; then return 0; fi
  say "Downloading $file"
  fetch_release_file "$RELEASE_URL/$file" "$M3MAX_KIT_WORK/$file" || rc=$?
  ((rc == 0)) || die "could not download $file from $TAG (curl exit $rc). Nothing was changed."
  [[ $(sha256sum "$M3MAX_KIT_WORK/$file" | cut -d' ' -f1) == "$sha" ]] ||
    die "$file does not match its published checksum. Nothing was changed."
}

# The identity of an installer: what it installs and with which kit files.
m3max_kit_identity() {
  printf '%s|' "$VERSION" "$TAG" "$M1N1_PACKAGE" "$M1N1_BIN_SHA" "$M3MAX_KIT_PLAN" "$M3MAX_KIT_MESA_PACKAGE" \
    "$M3_MAX_SWITCHES" "$M3_MAX_VARIANT" "$M3_ADT_READER" "${PACKAGES[*]}" "${M3_GPU_SCRIPTS[*]}"
}

# This script, on disk, into the work directory: the runner the kit installs. From a pipe
# (curl ... | bash) it can't be read again, so this release's copy is downloaded, and it must
# install exactly what this one does (m3max_kit_identity).
m3max_kit_self() {
  local out=$M3MAX_KIT_WORK/install-aurora-sep.sh theirs rc=0
  if [[ $SELF_SOURCE == file && -f $SELF_PATH ]]; then
    cp "$SELF_PATH" "$out"
    [[ $(sha256sum "$out" | cut -d' ' -f1) == "$SELF_SHA256" ]] || die "this script changed while it ran. Nothing was changed."
    return 0
  fi
  say "Downloading this release's install-aurora-sep.sh, for the kit's runner"
  fetch_release_file "$RELEASE_URL/install-aurora-sep.sh" "$out" || rc=$?
  ((rc == 0)) || die "could not download install-aurora-sep.sh from $TAG (curl exit $rc). Nothing was changed."
  # shellcheck disable=SC2016 # expanded by that bash
  theirs=$(bash -c 'AURORA_SEP_SOURCE_ONLY=1 source "$1" >/dev/null 2>&1 && m3max_kit_identity' _ "$out" 2>/dev/null) || theirs=""
  [[ -n $theirs && $theirs == "$(m3max_kit_identity)" ]] ||
    die "the install-aurora-sep.sh of $TAG is not the script that runs now (another release?). Run the
    kit's command again. Nothing was changed."
  chmod 0755 "$out"
}

# How many restarts the plan takes at most, and roughly how long, for the consent screen.
m3max_kit_estimate() {
  local e max
  e=$(m3max_plan_boots | awk '$2 != "data"' | wc -l)
  max=$(m3max_plan max-boots)
  ((e <= max)) || e=$max
  # One restart into the install, one per test boot, one back into a normal boot; about five
  # minutes each (restart, settle, collect, the delay before the next).
  echo "$((e + 2)) $(((e + 2) * 5 + 10))"
}

m3max_kit_consent() {
  local restarts minutes answer=""
  read -r restarts minutes <<<"$(m3max_kit_estimate)"
  cat <<EOF

======================== M3 MAX TEST KIT ($(this_board)) ========================
This runs the M3 Max test plan on this Mac in one go. It:
  1. writes a report of this Mac as it is now (it only reads);
  2. installs the test kernel ($(m3_kernel_release)) and a new boot loader: m1n1-aurora
     $(m1n1_version) with the M3 Max boot loader variant ($M3_MAX_VARIANT). The boot loader this
     Mac has now is kept on the EFI partition, and the kernel it runs now stays in the boot
     menu as "Aurora previous (GPU off)";
  3. restarts the Mac by itself up to $restarts times, over about $minutes minutes. The test boots
     are used once: whatever happens in one, the next restart is your normal boot. Save your
     work, keep the Mac on power, and let it run; nothing needs to be typed. Each restart is
     announced a minute or so ahead.
     If a test boot freezes or the screen stays black for more than 5 minutes, hold the power
     button until the Mac turns off, then press it again: the kit notes that boot and goes on.
     One step is different: the boot loader GPU test changes boot.bin itself. Before it, the
     kit writes the results so far (a file you can upload on its own) and announces it 3
     minutes ahead. If the Mac does not come back from it, put the boot loader back from macOS
     (the steps below); the kit then finishes by itself at the next Linux start.
  4. at the end, writes one file in your home folder, aurora-m3max-kit-$(this_board)-<date>.tgz,
     with the host name, user names, serial numbers and MAC addresses masked. You upload it.
At any time:
  sudo aurora-m3max-kit --status    what it has done, and what comes next
  sudo aurora-m3max-kit --stop      arm nothing more (keeps the results so far)
  sudo aurora-m3max-kit --restore   stop, and put back the boot loader this Mac has now
The steps to put the boot loader back from macOS are printed below and saved in your home
folder as aurora-m3max-kit-RESTORE.txt.
==========================================================================================
EOF
  if ((M3MAX_KIT_YES)); then
    say "--yes: starting"
    return 0
  fi
  if ! { exec {M3MAX_KIT_TTY_FD}<"$M3MAX_KIT_TTY"; } 2>/dev/null; then
    die "the kit asks for your yes on this screen, and this run has no terminal to ask in. Run the
    command in a terminal, or add --yes after --m3max-kit to agree on the command line. Nothing was changed."
  fi
  printf 'Type yes and press Enter to start (anything else stops, with nothing changed): '
  read -r -u "$M3MAX_KIT_TTY_FD" answer || answer=""
  exec {M3MAX_KIT_TTY_FD}<&-
  [[ ${answer,,} == yes ]] || die "stopped: nothing was changed."
}

# Every check before anything changes: the Mac, the boot chain, a run in progress, the kit's
# files, the plan, the one-shot arming, the space, and the boot loader variant's own checks.
m3max_kit_preflight() {
  local out phase free home
  if ! is_m3_max; then
    if [[ $(this_soc) == t6034 ]]; then
      die "the M3 Max test kit is for the 16-core M3 Max (t6031: $M3_MAX_BOARDS). This 14-core M3 Max
    ($(this_board), t6034) needs its own ADT first: please run --m3-report and attach the file to an
    issue. Nothing was changed."
    fi
    die "the M3 Max test kit is for the 16-core M3 Max (t6031: $M3_MAX_BOARDS), and this Mac is
    $(this_board) ($(this_soc)). Nothing was changed."
  fi
  [[ $(boot_chain) == limine ]] ||
    die "the kit arms its test boots through the Limine boot menu, and this Mac boots with $(boot_chain).
    Nothing was changed."
  phase=$($sudo sed -n 's/^phase=//p' "$M3MAX_KIT_STATE/status" 2>/dev/null | tail -1) || phase=""
  case $phase in
    running | finishing)
      die "a test kit run is in progress on this Mac ($phase): sudo aurora-m3max-kit --status shows it,
    --stop stops it, and --restore stops it and puts back the boot loader. Nothing was changed."
      ;;
  esac
  m3max_kit_fetch "$M3MAX_KIT_PLAN"
  m3max_kit_fetch "$(printf '%s\n' "${M3_GPU_SCRIPTS[@]}" | grep '^air-gpu-oneshot\.sh ')"
  m3max_kit_fetch "$(printf '%s\n' "${M3_GPU_SCRIPTS[@]}" | grep '^air-gpu-job\.sh ')"
  m3max_kit_fetch "$M3_ADT_READER"
  if [[ -n $M3MAX_KIT_MESA_PACKAGE ]]; then m3max_kit_fetch "$M3MAX_KIT_MESA_PACKAGE"; fi
  out=$(m3max_kit_plan_check "$M3MAX_KIT_WORK/${M3MAX_KIT_PLAN%% *}" 2>&1 >"$M3MAX_KIT_WORK/plan") ||
    die "$out. Nothing was changed."
  M3MAX_KIT_PLAN_FILE=$M3MAX_KIT_WORK/plan
  [[ " $(m3max_plan boards) " == *" $(this_board) "* ]] ||
    die "the kit's plan is for $(m3max_plan boards), not this $(this_board). Nothing was changed."
  M3MAX_KIT_ONESHOT=$M3MAX_KIT_WORK/air-gpu-oneshot.sh
  M3MAX_KIT_JOB=$M3MAX_KIT_WORK/air-gpu-job.sh
  out=$(m3max_kit_oneshot check 2>&1) ||
    die "this Mac can't arm the kit's one-shot test boots: $(sed -n 's/.*refused: //p' <<<"$out" | head -1)
    Nothing was changed."
  m3_esp_space_check
  free=$(m3max_kit_free_mb "$M3MAX_KIT_STATE")
  [[ -z $free ]] || ((free >= M3MAX_KIT_MIN_MB)) ||
    die "the kit needs about $M3MAX_KIT_MIN_MB MB free for its results in ${M3MAX_KIT_STATE%/*}, and there are $free MB. Nothing was changed."
  home=$(m3max_kit_home "$M3MAX_KIT_USER_NAME")
  free=$(m3max_kit_free_mb "$home")
  [[ -z $free ]] || ((free >= M3MAX_KIT_MIN_MB / 2)) ||
    die "the kit needs about $((M3MAX_KIT_MIN_MB / 2)) MB free in $home for its result, and there are $free MB. Nothing was changed."
  # The boot loader variant's own checks (stub, stage 1, update-m1n1), which install_all repeats.
  m3_plan
  [[ $M3_MODE == handoff ]] || die "the kit needs the M3 Max boot loader variant, which this Mac can't take (see above). Nothing was changed."
}

# The desktop user the kit writes its result for (who ran the command).
M3MAX_KIT_USER_NAME=""

m3max_kit_setup() { # [--yes] [--kit-work DIR]
  local a home dir restarts minutes
  while (($#)); do
    case $1 in
      --yes) M3MAX_KIT_YES=1 ;;
      --kit-work) M3MAX_KIT_WORK=${2:-}; shift ;;
      *) die "unknown option $1 after --m3max-kit (only --yes)" ;;
    esac
    shift
  done
  M3MAX_KIT=1 M3_TRY=1
  release_source
  require_supported_soc
  is_m3_max || m3max_kit_preflight
  if [[ -z $M3MAX_KIT_WORK ]]; then M3MAX_KIT_WORK=$(mktemp -d); fi
  M3MAX_KIT_USER_NAME=${SUDO_USER:-$(id -un)}
  # The kit runs as root from here (its state, its checks of the EFI variables, its runner), from
  # this script's copy on disk.
  if [[ -n $sudo ]]; then
    m3max_kit_self
    say "The kit runs as root from here on (sudo may ask for your password)."
    a=()
    ((M3MAX_KIT_YES == 0)) || a=(--yes)
    exec $sudo --preserve-env=AURORA_RELEASE_URL,AURORA_RELEASES_API bash "$M3MAX_KIT_WORK/install-aurora-sep.sh" \
      --m3max-kit --kit-work "$M3MAX_KIT_WORK" "${a[@]}"
  fi
  [[ -f $M3MAX_KIT_WORK/install-aurora-sep.sh ]] || m3max_kit_self
  m3max_kit_preflight
  m3max_kit_consent
  # From here on, the Mac changes.
  if [[ -d $M3MAX_KIT_STATE ]]; then mv "$M3MAX_KIT_STATE" "$M3MAX_KIT_STATE.before-$(date +%Y%m%d-%H%M%S)"; fi
  install -d -m 0700 "$M3MAX_KIT_STATE" "$M3MAX_KIT_STATE/stages" "$M3MAX_KIT_STATE/oneshot"
  install -m 0600 "$M3MAX_KIT_WORK/plan" "$M3MAX_KIT_STATE/plan"
  M3MAX_KIT_PLAN_FILE=$M3MAX_KIT_STATE/plan
  m3max_kit_set phase setup user "$M3MAX_KIT_USER_NAME" board "$(this_board)" started "$(date -Is)" \
    next 0 boots 0 armed "" gpu_ok "" kernel "$(m3_kernel_release)" tag "$TAG"
  say "Writing the baseline report (stage 00-baseline)"
  dir=$(m3max_kit_stage_dir baseline)
  trap 'm3_report_cleanup' EXIT
  m3max_kit_collect "$dir"
  m3max_kit_checks_data >"$dir/checks.txt" 2>&1 || true
  m3max_kit_outcome baseline collected "$(sed -n 's/^verdict: //p' "$dir/checks.txt" | tail -1)"
  install_all
  [[ $M3_MODE == handoff ]] || die "the install did not put the M3 Max boot loader variant on (see above); the kit stops here.
    sudo bash $M3MAX_KIT_WORK/install-aurora-sep.sh --m3max-kit-restore undoes what was done."
  m3max_kit_mesa_install
  m3max_kit_set bootbin "$(m3_bootbin_sha)" loader off \
    orig_bootbin "$(cut -d' ' -f2 "$STATE/m3max-bootbin-backup" 2>/dev/null || true)"
  m3max_kit_install_runner
  m3max_kit_units on
  m3max_kit_restore_files
  m3max_kit_set phase running
  m3max_kit_log "set up: $(m3max_plan_count) boots in the plan, at most $(m3max_plan max-boots) of them experimental"
  read -r restarts minutes <<<"$(m3max_kit_estimate)"
  echo
  echo "======================== M3 MAX TEST KIT: STARTED ($(this_board)) ========================"
  echo "  The Mac restarts by itself in 2 minutes, then up to $((restarts - 1)) more times, over about"
  echo "  $minutes minutes in all. Leave it on power and let it run. Before the boot loader GPU test"
  echo "  it writes the results so far (enough to upload) and says where. When it stops restarting:"
  echo "    sudo aurora-m3max-kit --status"
  echo "  prints the one file to upload (aurora-m3max-kit-$(this_board)-<date>.tgz in your home folder)."
  echo "  Stop it at any time: sudo aurora-m3max-kit --stop   (and --restore to put back the boot loader)"
  echo "  The restore steps are in ~/aurora-m3max-kit-RESTORE.txt, and on the EFI partition (which"
  echo "  macOS can read) as AURORA-M3MAX-KIT-RESTORE.txt. They are also printed above."
  m3max_kit_restore_text | sed 's/^/  | /' 
  echo "=========================================================================================="
  m3max_kit_set restart "in 2 min, into the first boot of the test kernel and boot loader"
  shutdown -r +2 "M3 Max test kit: this Mac restarts in 2 minutes to start the tests. To stop the kit: sudo aurora-m3max-kit --stop" \
    >/dev/null 2>&1 || warn "could not schedule the restart; restart the Mac to start the tests"
  rm -rf "$M3MAX_KIT_WORK"
}

# The kit's Mesa (M3MAX_KIT_MESA_PACKAGE), when the release has one: only a package whose every
# file is under /opt, in a pacman transaction of its own. A failure leaves the jobs stage out.
m3max_kit_mesa_install() {
  local file name unmet=""
  [[ -n $M3MAX_KIT_MESA_PACKAGE ]] || return 0
  # An image that holds its packages (frozen_package_detection) gets no transaction beyond the
  # installer's own: the kit leaves its Mesa out there, and the jobs stage is skipped.
  if ((FROZEN_PACKAGES)); then
    warn "this image holds its packages, so the kit's Mesa was left out (the jobs stage will be skipped)"
    return 0
  fi
  file=$M3MAX_KIT_WORK/${M3MAX_KIT_MESA_PACKAGE%% *}
  # No GPU start, no jobs: a kernel without the start experiment gets no Mesa.
  if [[ -n $(m3max_kit_oneshot missing asahi.t6031_start=1 2>/dev/null) ]]; then
    say "The test kernel has no GPU start experiment, so the kit's Mesa (for GPU jobs) is not installed"
    return 0
  fi
  # The whole listing first, then one grep with no pipe: grep -q stops reading at its first
  # match, and a writer's SIGPIPE under pipefail would turn a package with files elsewhere into
  # a pass.
  local listing
  if ! listing=$(bsdtar -tf "$file" 2>/dev/null) || grep -qvE '^(\.|opt/)' <<<"$listing"; then
    warn "the kit's Mesa package has files outside /opt, or could not be listed, so it was left out (no GPU jobs)"
    return 0
  fi
  local -a deps=()
  mapfile -t deps < <(bsdtar -xOf "$file" .PKGINFO 2>/dev/null | sed -n 's/^depend = //p')
  if ((${#deps[@]})) && ! unmet=$(pacman -T "${deps[@]}" 2>/dev/null); then
    warn "the kit's Mesa needs $(tr '\n' ' ' <<<"$unmet")which this Mac does not have at those versions, so it was
    left out (no GPU jobs; nothing was upgraded)"
    return 0
  fi
  name=$(bsdtar -xOf "$file" .PKGINFO 2>/dev/null | sed -n 's/^pkgname = //p' | head -1)
  if [[ -n $name ]] && pacman -U --noconfirm "$file"; then
    m3max_kit_set mesa "$name"
    say "Installed the kit's Mesa ($name) in its own prefix"
  else
    warn "the kit's Mesa did not install, so the jobs stage will be skipped"
  fi
}

# The runner: this script's copy with the kit's files next to it (the ADT reader where
# m3_adt_reader looks for it), aurora-m3max-kit, and the two units.
m3max_kit_install_runner() {
  local unit=$M3MAX_KIT_UNIT_DIR
  install -d -m 0755 "$M3MAX_KIT_LIBEXEC" "${M3MAX_KIT_BIN%/*}" "$unit"
  install -m 0755 "$M3MAX_KIT_WORK/install-aurora-sep.sh" "$M3MAX_KIT_LIBEXEC/install-aurora-sep.sh"
  install -m 0755 "$M3MAX_KIT_WORK/air-gpu-oneshot.sh" "$M3MAX_KIT_WORK/air-gpu-job.sh" "$M3MAX_KIT_LIBEXEC/"
  install -m 0644 "$M3MAX_KIT_WORK/${M3_ADT_READER%% *}" "$M3MAX_KIT_LIBEXEC/aurora-adt-extract.py"
  cat >"$M3MAX_KIT_BIN.tmp" <<EOF
#!/bin/bash
# aurora-m3max-kit: the M3 Max test kit, set up by install-aurora-sep.sh --m3max-kit ($TAG).
#   sudo aurora-m3max-kit --status    what the kit has done, what comes next, the file to upload
#   sudo aurora-m3max-kit --stop      arm nothing more, cancel its restart, pack the results so far
#   sudo aurora-m3max-kit --restore   --stop, and put back the boot loader this Mac had before
if ((EUID != 0)); then exec sudo "\$0" "\$@"; fi
exec $M3MAX_KIT_LIBEXEC/install-aurora-sep.sh --m3max-kit-runner "\$@"
EOF
  chmod 0755 "$M3MAX_KIT_BIN.tmp"
  mv -f "$M3MAX_KIT_BIN.tmp" "$M3MAX_KIT_BIN"
  cat >"$unit/$M3MAX_KIT_UNIT" <<EOF
[Unit]
Description=M3 Max test kit: collect this boot and arm the next test boot
After=multi-user.target
ConditionPathExists=$M3MAX_KIT_STATE/status

[Service]
# exec, not oneshot: the step waits until the boot has finished starting
# (systemctl is-system-running --wait), which a oneshot start job would hold up.
Type=exec
ExecStart=$M3MAX_KIT_BIN --step

[Install]
WantedBy=multi-user.target
EOF
  cat >"$unit/$M3MAX_KIT_MARK_UNIT" <<EOF
[Unit]
Description=M3 Max test kit: note that a test boot reached userspace
DefaultDependencies=no
After=local-fs.target
Before=sysinit.target shutdown.target
Conflicts=shutdown.target
ConditionKernelCommandLine=$M3MAX_KIT_TAG
ConditionPathExists=$M3MAX_KIT_STATE/status

[Service]
Type=oneshot
ExecStart=$M3MAX_KIT_BIN --mark

[Install]
WantedBy=sysinit.target
EOF
}

# --uninstall: the kit stops for good. Its results stay in $M3MAX_KIT_STATE.
m3max_kit_uninstall() {
  local mesa
  [[ -e $M3MAX_KIT_BIN || -e $M3MAX_KIT_UNIT_DIR/$M3MAX_KIT_UNIT || -e $M3MAX_KIT_LIBEXEC ]] || return 0
  if $sudo test -f "$M3MAX_KIT_STATE/status"; then
    $sudo "$M3MAX_KIT_BIN" --stop || die "could not stop the M3 Max test kit (sudo aurora-m3max-kit --stop). Nothing was uninstalled."
  fi
  m3max_kit_units off
  mesa=$($sudo sed -n 's/^mesa=//p' "$M3MAX_KIT_STATE/status" 2>/dev/null | tail -1) || mesa=""
  if [[ -n $mesa ]] && pacman -Q "$mesa" >/dev/null 2>&1; then
    $sudo pacman -Rns --noconfirm "$mesa" || warn "could not remove the kit's $mesa; remove it with: sudo pacman -Rns $mesa"
  fi
  $sudo rm -rf "$M3MAX_KIT_LIBEXEC" "$M3MAX_KIT_BIN" "$M3MAX_KIT_UNIT_DIR/$M3MAX_KIT_UNIT" "$M3MAX_KIT_UNIT_DIR/$M3MAX_KIT_MARK_UNIT"
  $sudo systemctl daemon-reload || true
  say "Removed the M3 Max test kit's runner and units; its results stay in $M3MAX_KIT_STATE"
}

# --m3max-kit-status, --m3max-kit-stop and --m3max-kit-restore from a release's script: the
# installed runner's, when there is one; this script's own as root, when there is none.
m3max_kit_from_release() { # ACTION
  if [[ -n $sudo ]]; then
    if [[ -x $M3MAX_KIT_BIN ]]; then exec $sudo "$M3MAX_KIT_BIN" "$1"; fi
    die "there is no aurora-m3max-kit on this Mac; run this as root:
      curl -fsSL $LATEST_URL | sudo bash -s -- --m3max-kit-${1#--}"
  fi
  m3max_kit_runner "$1"
}

# The installed runner (aurora-m3max-kit): its actions.
m3max_kit_runner() { # ACTION
  case ${1:-} in
    --status | --stop | --restore | --step | --mark) ;;
    *) die "aurora-m3max-kit takes --status, --stop or --restore" ;;
  esac
  ((EUID == 0)) || [[ -z $sudo ]] || die "run it as root: sudo aurora-m3max-kit $1"
  case $1 in
    --status) m3max_kit_status ;;
    --stop) m3max_kit_stop ;;
    --restore) m3max_kit_restore ;;
    --step) m3max_kit_step ;;
    --mark) m3max_kit_mark ;;
  esac
}

# Tests source this file for its functions only.
if [[ ${AURORA_SEP_SOURCE_ONLY:-} == 1 ]]; then return 0; fi

# --m3-handoff, --m3-gpu-experiment and --no-m3-mesa go with an install, alone, together or
# with --read-only.
args=()
for a in "$@"; do
  case $a in
    --archive-esp-history) ESP_ARCHIVE_HISTORY=1 ;;
    --neo-gpu) NEO_GPU=1 ;;
    --m3-handoff) M3_TRY=1 ;;
    --m3-gpu-experiment) M3_GPU_EXPERIMENT=1 ;;
    --m3-gpu) M3_GPU_AUTO=1; M3_GPU_PERSISTENT=1; M3_TRY=1 ;;
    --m3-gpu-persistent) M3_GPU_EXPLICIT_PROFILE=1; M3_GPU_PERSISTENT=1; M3_TRY=1 ;;
    --m3-profile=j613-25g83) M3_GPU_EXPLICIT_PROFILE=1; M3_GPU_PROFILE=j613-25g83; M3_GPU_PERSISTENT=1; M3_TRY=1; M3_25_J615=0 ;;
    # The same 25G83 firmware ABI profile, admitted on a J615 as an experiment.
    --m3-profile=j615-25g83) M3_GPU_EXPLICIT_PROFILE=1; M3_GPU_PROFILE=j613-25g83; M3_GPU_PERSISTENT=1; M3_TRY=1; M3_25_J615=1 ;;
    --no-m3-mesa) M3_PRO_MESA=0 ;;
    --desktop-fixes) DESKTOP_FIXES=1 ;;
    *) args+=("$a") ;;
  esac
done
set -- "${args[@]}"
if ((ESP_ARCHIVE_HISTORY)) && [[ -n ${1:-} ]]; then
  die "--archive-esp-history goes with an install; use --esp-history for a read-only inventory"
fi
if ((NEO_GPU)) && [[ -n ${1:-} && $1 != --read-only ]]; then
  die "--neo-gpu goes with an install, not with $1"
fi
if ((M3_GPU_AUTO && M3_GPU_EXPLICIT_PROFILE)); then
  die "use --m3-gpu by itself to detect the profile, or choose one explicit GPU profile"
fi
if ((DESKTOP_FIXES)) && [[ -n ${1:-} && $1 != --read-only ]]; then
  die "--desktop-fixes goes with an install (alone or with --read-only), not with $1"
fi
if ((M3_TRY)) && [[ -n ${1:-} && $1 != --read-only ]]; then
  die "--m3-handoff goes with an install (alone or with --read-only), not with $1"
fi
if ((M3_GPU_EXPERIMENT || M3_GPU_PERSISTENT)) && [[ -n ${1:-} && $1 != --read-only ]]; then
  die "M3 GPU options go with an install (alone, with --m3-handoff or with --read-only), not with $1"
fi
if ((!M3_PRO_MESA)) && [[ -n ${1:-} && $1 != --read-only ]]; then
  die "--no-m3-mesa goes with an install (alone, with another install option or with --read-only), not with $1"
fi
# One option at a time; these commands take arguments of their own.
if (($# > 1)) && [[ $1 != --reset-touchid && $1 != --m3-gpu-check && $1 != --m3max-kit && $1 != --m3max-kit-runner ]]; then
  die "unexpected arguments after $1: ${*:2}"
fi

if preflight_needed "${1:-}"; then preflight; fi
case ${1:-} in
  "") install_all ;;
  --read-only) READ_ONLY=1; install_all ;;
  --uninstall) uninstall_all ;;
  --reset-touchid) shift; reset_touchid "$@" ;;
  --esp-history) esp_history_run report ;;
  --m3-report) m3_report ;;
  --m3-power-survey) m3_power_survey ;;
  --m3-gpu-check) shift; m3_gpu_check_run "$@" ;;
  --m3max-kit) shift; m3max_kit_setup "$@" ;;
  --m3max-kit-runner) shift; m3max_kit_runner "$@" ;;
  --m3max-kit-status) m3max_kit_from_release --status ;;
  --m3max-kit-stop) m3max_kit_from_release --stop ;;
  --m3max-kit-restore) m3max_kit_from_release --restore ;;
  --agent-prompt) release_source >&2; prompt_notice; agent_prompt ;;
  *) die "unknown option $1 (--read-only, --uninstall, --reset-touchid, --agent-prompt, --m3-report, --m3-power-survey, --neo-gpu, --m3-handoff, --m3-gpu, --m3-gpu-check, --m3-gpu-experiment, --m3-gpu-persistent, --m3-profile=j613-25g83, --m3-profile=j615-25g83, --no-m3-mesa, --archive-esp-history, --esp-history, --desktop-fixes, --m3max-kit or --m3max-kit-status, -stop, -restore)" ;;
esac
