#!/bin/sh
# Tests for mesa-m3-session-env: fake sysfs/procfs/prefix/home trees per case,
# fake journalctl, logger, loginctl and systemctl. Checks every reason of the
# gate, the state file and journal line, the exports, and that sourcing leaves
# nothing behind. Run with sh and bash. No root needed.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
ENVFILE=${ENVFILE:-"$HERE/../mesa-m3-session-env"}
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
fail=0
UID_=$(id -u)
# The shell this suite runs under, for the cases that must exercise it.
TSH=sh
[ -z "${BASH_VERSION:-}" ] || TSH=bash

mkdir -p "$T/bin" "$T/home"
# journalctl, as the hook calls it (the whole kernel log of this boot):
# FAKE_JOURNAL=none|match|bank|dcp|g15g|verdict|verdictok|noperm|error. "none" holds harmless lines
# that name failures of other devices or refused requests; "error" is how the
# real journalctl fails (exit 1, message on stderr).
cat > "$T/bin/journalctl" <<'J'
#!/bin/sh
normal() {
	echo "Booting Linux on physical CPU 0x0000000000 [0x611f0490]"
	echo "mmc0: tuning execution failed: -110"
	echo "asahi: M3 compute rejected: flags=0x4 samplers=1 heap=0x0 attachments=0 timestamps=0/0"
	echo "asahi 292400000.gpu: M3: firmware ready; common DRM GEM/VM frontend registered"
}
case "${FAKE_JOURNAL:-none}" in
none) normal ;;
match) normal; echo "asahi 292400000.gpu: M3 scheduler: execution failed ETIMEDOUT" ;;
bank) normal; echo "asahi 292400000.gpu: M3 bank 3 fault=0x1 mapping=0x0" ;;
dcp) normal; echo "apple-dcp 2d2c00000.dcp: DCP firmware crashed" ;;
verdict) normal; echo "asahi 292400000.gpu: M3 G15G verdict: job-timed-out-powered: the job did not retire with the GPU powered (pstate register 0x2, GPU start timestamp None); see the lines around this one" ;;
verdictok) normal; echo "asahi 292400000.gpu: M3 G15G verdict: firmware-running: the firmware accepted the InitData (version 0xc08e21e83800490) and the device controls; no job has run yet"
	echo "asahi 292400000.gpu: M3 G15G verdict: job-completed: the first compute job finished, GPU timestamps 0x1..0x2, 1 ns of GPU time"
	echo "asahi 292400000.gpu: M3 G15G verdict: job-retired-without-timestamps: the first render job retired, but its GPU timestamps are unreadable" ;;
g15g) normal; echo "asahi 292400000.gpu: M3 G15G: firmware crashed, retained crashlog bytes=4096" ;;
noperm) : ;;
error) echo "Failed to open journal: Permission denied" >&2; exit 1 ;;
stall) sleep 60; normal ;;
esac
exit 0
J
cat > "$T/bin/logger" <<'J'
#!/bin/sh
printf '%s\n' "$*" >> "$FAKE_LOG"
J
cat > "$T/bin/loginctl" <<'J'
#!/bin/sh
echo 7
J
cat > "$T/bin/systemctl" <<'J'
#!/bin/sh
case "$*" in *"user@"*".service"*) echo 0123456789abcdef0123456789abcdef ;; esac
J
chmod +x "$T/bin/"*

nul() { printf '%s\0' "$@"; }

# mkroot NAME DT_COMPAT "GPU_COMPATS" DCP(yes/no) NODES PREFIX(yes/no) [DRIVER]
# The display card's compatible is that of the SoC in DT_COMPAT (T8122 or else
# T6030); MKROOT_DCP=<compatible> overrides it.
mkroot() {
	r="$T/$1"; mkdir -p "$r/proc/device-tree" "$r/sys/class/drm" "$r/sys/bus/platform/drivers/${7:-asahi}" \
		"$r/sys/bus/platform/drivers/apple-drm" "$r/dev/dri" "$r/etc" "$r/usr/lib" "$r/run/user/$UID_" "$r$T/home"
	echo "root=UUID=x rw quiet" > "$r/proc/cmdline"
	mkdir -p "$r/proc/sys/kernel/random"; echo "boot-a" > "$r/proc/sys/kernel/random/boot_id"
	nul $2 apple,arm-platform > "$r/proc/device-tree/compatible"
	i=0
	while [ "$i" -lt "$5" ]; do
		n=$((128 + i)); d="$r/sys/devices/platform/gpu$i"
		mkdir -p "$d/of_node"; nul $3 > "$d/of_node/compatible"
		ln -s "$r/sys/bus/platform/drivers/${7:-asahi}" "$d/driver"
		mkdir -p "$r/sys/class/drm/renderD$n"; ln -s "$d" "$r/sys/class/drm/renderD$n/device"
		: > "$r/dev/dri/renderD$n"; chmod 600 "$r/dev/dri/renderD$n"
		i=$((i + 1))
	done
	if [ "$4" = yes ]; then
		d="$r/sys/devices/platform/display"; mkdir -p "$d/of_node"
		case " $2 " in
		*" apple,t8122 "*) _dcp=apple,t8122-display-subsystem ;;
		*) _dcp=apple,t6030-display-subsystem ;;
		esac
		nul "${MKROOT_DCP:-$_dcp}" > "$d/of_node/compatible"
		ln -s "$r/sys/bus/platform/drivers/apple-drm" "$d/driver"
		mkdir -p "$r/sys/class/drm/card2" "$r/sys/class/drm/card2-eDP-1"
		ln -s "$d" "$r/sys/class/drm/card2/device"
	fi
	if [ "$6" = yes ]; then
		p="$r/opt/mesa-m3"
		mkdir -p "$p/share/vulkan/icd.d" "$p/share/glvnd/egl_vendor.d" "$p/share/drirc.d" "$p/lib/gbm" \
			"$p/lib/dri" "$p/share/mesa-m3" "$p/libexec"
		for f in share/vulkan/icd.d/asahi_icd.aarch64.json share/glvnd/egl_vendor.d/50_mesa.json \
			share/drirc.d/10-m3-asahi-zink.conf lib/libvulkan_asahi.so lib/libEGL_mesa.so.0 lib/libgbm.so.1 \
			lib/gbm/dri_gbm.so lib/dri/libdril_dri.so lib/libgallium-26.1.4.so; do : > "$p/$f"; done
		ln -s libdril_dri.so "$p/lib/dri/zink_dri.so"
		printf '# sonames\nlibdrm.so.2\nlibexpat.so.1\nlibgallium-26.1.4.so\n' > "$p/share/mesa-m3/needed-sonames"
		: > "$r/usr/lib/libdrm.so.2"; : > "$r/usr/lib/libexpat.so.1"
		echo "26.1.4.m3.1-1" > "$p/share/mesa-m3/package-version"
		printf '#!/bin/sh\n[ "${FAKE_LOAD:-ok}" = ok ] && { echo "ok $1/lib/libgallium-26.1.4.so"; exit 0; }\necho "fail $1/lib/libvulkan_asahi.so: libdisplay-info.so.3: cannot open shared object file"; exit 1\n' \
			> "$p/libexec/mesa-m3-loadcheck"
		printf '#!/bin/sh\n[ "${FAKE_ABI:-ok}" = ok ]\n' > "$p/libexec/mesa-m3-abi-check"
		chmod +x "$p/libexec/mesa-m3-abi-check"
		chmod +x "$p/libexec/mesa-m3-loadcheck"
		cp "$HERE/../mesa-m3-user-setup" "$p/libexec/mesa-m3-user-setup"
		cp "$HERE/../user-setup.list" "$p/share/mesa-m3/user-setup.list"
		cp "$HERE/../mesa-m3-session-confirm" "$p/libexec/mesa-m3-session-confirm"
		: > "$p/libexec/mesa-m3-wlcheck"
	fi
	echo "$r"
}

# gate ROOT [FAKE_JOURNAL [VAR=value ...]]: the gate's line, in a clean environment.
gate() {
	_r=$1 _j=${2:-none}
	rm -f "$_r$T/home/.local/state/mesa-m3/attempt"
	shift 1; [ "$#" -gt 0 ] && shift
	env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_JOURNAL="$_j" FAKE_LOG="$T/log" XDG_CURRENT_DESKTOP=Hyprland "$@" \
		MESA_M3_SESSION_ENV_NO_RUN=1 sh -c '. "$1"; mesa_m3_gate "$2"' sh "$ENVFILE" "$_r"
}
# expect ROOT REASON "detail fragment" [FAKE_JOURNAL [VAR=value ...]]
expect() {
	_er=$1 _ew=$2 _ef=$3
	shift 3
	out=$(gate "$_er" "$@")
	case "$out" in
	"$_ew "*"$_ef"*) printf 'ok    %-14s %-18s %s\n' "${_er##*/}" "$_ew" "$out" ;;
	*) printf 'FAIL  %-14s got: %s (want %s ...%s...)\n' "${_er##*/}" "$out" "$_ew" "$_ef"; fail=1 ;;
	esac
}

G15S="apple,agx-t6030 apple,agx-g15x apple,agx-g15s"
J516S=$(mkroot j516s "apple,j516s apple,t6030" "$G15S" yes 1 yes)
expect "$J516S" active "T6030, render node /dev/dri/renderD128, display card2"
expect "$(mkroot j514s "apple,j514s apple,t6030" "$G15S" yes 1 yes)" active "T6030"

# opt-out
R=$(mkroot optout1 "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R/etc/mesa-m3"; : > "$R/etc/mesa-m3/disable"
expect "$R" opt-out "/etc/mesa-m3/disable"
R=$(mkroot optout2 "apple,j516s apple,t6030" "$G15S" yes 1 yes); echo "root=UUID=x rw mesa_m3=off quiet" > "$R/proc/cmdline"
expect "$R" opt-out "mesa_m3=off"
R=$(mkroot optout3 "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/.config/mesa-m3"; : > "$R$T/home/.config/mesa-m3/disable"
expect "$R" opt-out "mesa-m3/disable"
# not-supported: every board other than T6030 and T8122
expect "$(mkroot j514c "apple,j514c apple,t6031" "apple,agx-t6031 apple,agx-g15x apple,agx-g15c" yes 1 yes)" not-supported "neither apple,t6030 nor apple,t8122"
expect "$(mkroot j516m "apple,j516m apple,t6034" "apple,agx-t6034" yes 1 yes)" not-supported ""
expect "$(mkroot j293 "apple,j293 apple,t8103" "apple,agx-t8103 apple,agx-g13g" no 1 yes)" not-supported ""
expect "$(mkroot j314s "apple,j314s apple,t6000" "apple,agx-t6000 apple,agx-g13x apple,agx-g13s" no 1 yes)" not-supported ""
expect "$(mkroot j413 "apple,j413 apple,t8112" "apple,agx-t8112 apple,agx-g14g" no 1 yes)" not-supported ""
expect "$(mkroot j414s "apple,j414s apple,t6020" "apple,agx-t6020 apple,agx-g14x apple,agx-g14s" no 1 yes)" not-supported ""
expect "$(mkroot j700 "apple,j700 apple,t8140" "apple,agx-t8140 apple,agx-g17p" no 1 yes)" not-supported ""

# T8122 (M3 13" J613, 15" J615): the same gate with the T8122 compatibles.
# Our kernel's GPU node is compatible "apple,agx-t8122" only.
G15G="apple,agx-t8122"
# 12.4: the T8122 GPU is experimental. Its sessions use /opt/mesa-m3 only with
# the opt-in marker /etc/mesa-m3/t8122-gpu-experiment (optin ROOT writes it).
optin() { mkdir -p "$1/etc/mesa-m3"; : > "$1/etc/mesa-m3/t8122-gpu-experiment"; echo "$1"; }
J613=$(optin "$(mkroot j613 "apple,j613 apple,t8122" "$G15G" yes 1 yes)")
expect "$J613" active "T8122, render node /dev/dri/renderD128, display card2"
expect "$(optin "$(mkroot j615 "apple,j615 apple,t8122" "$G15G" yes 1 yes)")" active "T8122"
# Without the marker: experimental, whatever else holds.
J613X=$(mkroot j613x "apple,j613 apple,t8122" "$G15G" yes 1 yes)
expect "$J613X" experimental "only with /etc/mesa-m3/t8122-gpu-experiment"
expect "$(mkroot j615x "apple,j615 apple,t8122" "$G15G" yes 1 yes)" experimental "(the installer writes it for --m3-gpu-experiment)"
expect "$(mkroot j613xoff "apple,j613 apple,t8122" "x" yes 0 yes)" experimental "T8122"
mkdir -p "$J613X/etc/mesa-m3/t8122-gpu-experiment"
expect "$J613X" experimental "t8122-gpu-experiment"
rmdir "$J613X/etc/mesa-m3/t8122-gpu-experiment"
mkdir -p "$J613X$T/home/.config/mesa-m3"; : > "$J613X$T/home/.config/mesa-m3/t8122-gpu-experiment"
expect "$J613X" experimental "t8122-gpu-experiment"
rm -rf "$J613X$T/home/.config/mesa-m3"
# The opt-out still wins over the opt-in; the T6030 ignores the marker.
R=$(optin "$(mkroot j613oo "apple,j613 apple,t8122" "$G15G" yes 1 yes)"); : > "$R/etc/mesa-m3/disable"
expect "$R" opt-out "/etc/mesa-m3/disable"
expect "$(optin "$(mkroot j516in "apple,j516s apple,t6030" "$G15S" yes 1 yes)")" active "T6030"
# GPU not armed (no handoff or no asahi.t8122_start=1): no render node.
expect "$(optin "$(mkroot j613off "apple,j613 apple,t8122" "x" yes 0 yes)")" no-gpu "found 0"
# Never a mix: a T8122 with the T6030 GPU or display, and the reverse.
expect "$(optin "$(mkroot j613g15s "apple,j613 apple,t8122" "$G15S" yes 1 yes)")" no-gpu "not the T8122 GPU (apple,agx-t8122)"
R=$(optin "$(MKROOT_DCP=apple,t6030-display-subsystem mkroot j613dcps "apple,j613 apple,t8122" "$G15G" yes 1 yes)")
expect "$R" no-display "apple,t8122-display-subsystem"
R=$(MKROOT_DCP=apple,t8122-display-subsystem mkroot j516dcpg "apple,j516s apple,t6030" "$G15S" yes 1 yes)
expect "$R" no-display "apple,t6030-display-subsystem"
expect "$(optin "$(mkroot j613nodcp "apple,j613 apple,t8122" "$G15G" no 1 yes)")" no-display "apple,t8122-display-subsystem"
expect "$J613" gpu-fault "M3 G15G: firmware crashed" g15g
expect "$J613" gpu-fault "M3 G15G verdict: job-timed-out-powered" verdict
expect "$J613" active "T8122" verdictok
expect "$J516S" gpu-fault "M3 G15G: firmware crashed" g15g
# no-gpu: kernel-only M3 Pro (no render node), two nodes, another driver, another GPU
expect "$(mkroot kernelonly "apple,j514s apple,t6030" "x" no 0 yes)" no-gpu "found 0"
expect "$(mkroot twonodes "apple,j516s apple,t6030" "$G15S" yes 2 yes)" no-gpu "found 2"
expect "$(mkroot otherdrv "apple,j516s apple,t6030" "apple,agx-t6030" yes 1 yes simpledrm)" no-gpu "not asahi"
expect "$(mkroot wronggpu "apple,j516s apple,t6030" "apple,agx-t8122 apple,agx-g15g" yes 1 yes)" no-gpu "apple,agx-t6030"
# no-display, no-access
expect "$(mkroot nodcp "apple,j516s apple,t6030" "$G15S" no 1 yes)" no-display "apple,t6030-display-subsystem"
R=$(mkroot noaccess "apple,j516s apple,t6030" "$G15S" yes 1 yes); chmod 000 "$R/dev/dri/renderD128"
if [ -r "$R/dev/dri/renderD128" ]; then echo "skip  noaccess (running as root)"; else expect "$R" no-access "cannot open"; fi

# user-setup: the environment
expect "$J516S" user-setup "environment sets MESA_LOADER_DRIVER_OVERRIDE" none MESA_LOADER_DRIVER_OVERRIDE=zink
expect "$J516S" user-setup "sets LIBGL_ALWAYS_SOFTWARE" none LIBGL_ALWAYS_SOFTWARE=1
expect "$J516S" user-setup "sets VK_ADD_DRIVER_FILES" none VK_ADD_DRIVER_FILES=/x.json
expect "$J516S" user-setup "sets DRIRC_CONFIGDIR" none DRIRC_CONFIGDIR=/x
expect "$J516S" user-setup "sets VK_ICD_FILENAMES" none VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/asahi_icd.aarch64.json
expect "$J516S" user-setup "sets __EGL_VENDOR_LIBRARY_FILENAMES" none __EGL_VENDOR_LIBRARY_FILENAMES=/home/u/p/50_mesa.json
mkdir -p "$J516S/home/u/prefix/lib"; : > "$J516S/home/u/prefix/lib/libgallium-26.1.4.so"
expect "$J516S" user-setup "puts another Mesa in LD_LIBRARY_PATH (/home/u/prefix/lib)" none LD_LIBRARY_PATH=/opt/mesa-m3/lib:/home/u/prefix/lib
expect "$J516S" active "T6030" none LD_LIBRARY_PATH=/opt/mesa-m3/lib:/usr/local/lib VK_DRIVER_FILES=/opt/mesa-m3/share/vulkan/icd.d/asahi_icd.aarch64.json
# user-setup: files read after this one
R=$(mkroot uwsmfile "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/.config/uwsm"
printf '# VK_ICD_FILENAMES=/commented\nexport VK_ICD_FILENAMES=/home/u/icd.json\n' > "$R$T/home/.config/uwsm/env"
expect "$R" user-setup "/.config/uwsm/env:2 sets VK_ICD_FILENAMES (/home/u/icd.json)"
R=$(mkroot uwsmours "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/.config/uwsm"
printf 'export VK_ICD_FILENAMES=/opt/mesa-m3/share/vulkan/icd.d/asahi_icd.aarch64.json\n# LIBGL_DRIVERS_PATH=/x\n' > "$R$T/home/.config/uwsm/env"
expect "$R" active "T6030"
R=$(mkroot uwsmld "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/.config/uwsm" "$R$T/home/mesa/lib"
: > "$R$T/home/mesa/lib/libEGL_mesa.so.0"
printf 'export LD_LIBRARY_PATH="$HOME/mesa/lib:$LD_LIBRARY_PATH"\n' > "$R$T/home/.config/uwsm/env-hyprland"
expect "$R" user-setup "puts another Mesa in LD_LIBRARY_PATH ($T/home/mesa/lib)"
R=$(mkroot hypr "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/.config/hypr"
printf 'env = GBM_BACKENDS_PATH,/usr/lib/gbm\n' > "$R$T/home/.config/hypr/envs.conf"
expect "$R" user-setup "/.config/hypr/envs.conf:1 sets GBM_BACKENDS_PATH (/usr/lib/gbm)"
R=$(mkroot hyprlua "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/.config/hypr"
printf -- '-- hl.env("LIBGL_DRIVERS_PATH", "/x")\nhl.env("LIBGL_DRIVERS_PATH", "/usr/lib/dri")\n' > "$R$T/home/.config/hypr/envs.lua"
expect "$R" user-setup "envs.lua:2 sets LIBGL_DRIVERS_PATH (/usr/lib/dri)"
R=$(mkroot drirc "apple,j516s apple,t6030" "$G15S" yes 1 yes)
printf '<driconf><device><application name="all"><option name="dri_driver" value="zink"/></application></device></driconf>\n' > "$R$T/home/.drirc"
expect "$R" user-setup ".drirc chooses a driver"
R=$(mkroot drircplain "apple,j516s apple,t6030" "$G15S" yes 1 yes)
printf '<driconf><device><application name="x"><option name="vblank_mode" value="0"/></application></device></driconf>\n' > "$R$T/home/.drirc"
expect "$R" active "T6030"
# user-setup: chonkstep, as on the lab m3pro today, and after the switch-over
R=$(mkroot chonk "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/.config/chonkstep"
printf 'CHONKSTEP_M3_CLIENTS=gpu\n\n# Qualified private Mesa\nCHONKSTEP_M3_MESA_PREFIX=/home/u/src/m3-gpu-work/mesa-eryk-prefix-20261001\nCHONKSTEP_M3_XWAYLAND_GLAMOR=1\n' \
	> "$R$T/home/.config/chonkstep/m3gpu-session.env"
# Round 4: chonkstep's file counts only for a chonkstep session.
expect "$R" user-setup "sets CHONKSTEP_M3_MESA_PREFIX to /home/u/src/m3-gpu-work/mesa-eryk-prefix-20261001" none XDG_CURRENT_DESKTOP=chonkstep
expect "$R" active "session desktops Hyprland; 1 setting(s) not used by this session ignored, first: desktop:chonkstep sure" none XDG_CURRENT_DESKTOP=Hyprland
expect "$R" active "session desktops unknown; 1 setting(s) not used by this session ignored" none XDG_CURRENT_DESKTOP=
expect "$R" active "session desktops sway; 1 setting(s) not used" none XDG_CURRENT_DESKTOP=sway
sed -i 's|^CHONKSTEP_M3_MESA_PREFIX=.*|CHONKSTEP_M3_MESA_PREFIX=/opt/mesa-m3|' "$R$T/home/.config/chonkstep/m3gpu-session.env"
expect "$R" active "T6030" none XDG_CURRENT_DESKTOP=chonkstep
expect "$J516S" user-setup "the environment sets CHONKSTEP_M3_MESA_PREFIX" none CHONKSTEP_M3_MESA_PREFIX=/x XDG_CURRENT_DESKTOP=chonkstep
expect "$J516S" active "1 setting(s) not used by this session ignored" none CHONKSTEP_M3_MESA_PREFIX=/x
R=$(mkroot chonkdef "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/.config/chonkstep"
printf 'CHONKSTEP_M3_CLIENTS=gpu\nCHONKSTEP_M3_XWAYLAND_GLAMOR=1\n' > "$R$T/home/.config/chonkstep/m3gpu-session.env"
expect "$R" user-setup "sets no CHONKSTEP_M3_MESA_PREFIX: chonkstep's M3 launcher uses its own default prefix" none XDG_CURRENT_DESKTOP=chonkstep
expect "$R" active "T6030" none XDG_CURRENT_DESKTOP=Hyprland
expect "$R" active "T6030" none CHONKSTEP_M3_MESA_PREFIX=/opt/mesa-m3 XDG_CURRENT_DESKTOP=chonkstep
R=$(mkroot nochonk "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/src/m3-gpu-work/mesa-eryk-prefix/lib"
: > "$R$T/home/src/m3-gpu-work/mesa-eryk-prefix/lib/libgallium-26.1.4.so"
expect "$R" active "T6030"
R=$(mkroot envd "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/.config/environment.d"
printf 'LD_LIBRARY_PATH=/usr/local/lib:${HOME}/lib\nPATH=/x\n' > "$R$T/home/.config/environment.d/10-path.conf"
expect "$R" active "T6030"
printf 'GALLIUM_DRIVER=llvmpipe\n' > "$R$T/home/.config/environment.d/20-mesa.conf"
expect "$R" user-setup "environment.d/20-mesa.conf:1 sets GALLIUM_DRIVER"
# Round 4: a Hyprland session with a uwsm env-hyprland file that chooses a
# private Mesa: Hyprland uses it, so the hook yields (no fallback).
R=$(mkroot hyprenv "apple,j516s apple,t6030" "$G15S" yes 1 yes); P="$T/home/src/m3-gpu-work/mesa-eryk-prefix-20261001"
mkdir -p "$R$T/home/.config/uwsm" "$R$P/lib" "$R$T/home/.config/chonkstep"; : > "$R$P/lib/libgallium-26.1.4.so"
printf 'export LD_LIBRARY_PATH="%s/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\nexport LIBGL_DRIVERS_PATH=%s/lib/dri\n' "$P" "$P" \
	> "$R$T/home/.config/uwsm/env-hyprland"
printf 'CHONKSTEP_M3_MESA_PREFIX=/opt/mesa-m3\n' > "$R$T/home/.config/chonkstep/m3gpu-session.env"
expect "$R" user-setup "env-hyprland:1 puts another Mesa in LD_LIBRARY_PATH ($P/lib)" none XDG_CURRENT_DESKTOP=Hyprland
expect "$R" active "2 setting(s) not used by this session ignored" none XDG_CURRENT_DESKTOP=chonkstep
R=$(mkroot listgone "apple,j516s apple,t6030" "$G15S" yes 1 yes); chmod 000 "$R/opt/mesa-m3/share/mesa-m3/user-setup.list"
if [ -r "$R/opt/mesa-m3/share/mesa-m3/user-setup.list" ]; then echo "skip  listgone (running as root)"; else
	expect "$R" user-setup-unknown "the user-setup check failed (exit 2"; fi

# incomplete-prefix, missing-soname, load-failed
expect "$(mkroot noprefix "apple,j516s apple,t6030" "$G15S" yes 1 no)" incomplete-prefix "is missing"
R=$(mkroot nosoname "apple,j516s apple,t6030" "$G15S" yes 1 yes); rm "$R/usr/lib/libexpat.so.1"
expect "$R" missing-soname "libexpat.so.1"
R=$(mkroot noload "apple,j516s apple,t6030" "$G15S" yes 1 yes)
expect "$R" load-failed "libdisplay-info.so.3" none FAKE_LOAD=fail
# log-unreadable, gpu-fault
expect "$J516S" log-unreadable "cannot read this boot's kernel log" noperm
expect "$J516S" log-unreadable "cannot read this boot's kernel log" error
expect "$J516S" gpu-fault "M3 bank 3 fault" bank
expect "$J516S" gpu-fault "DCP firmware crashed" dcp
expect "$J516S" gpu-fault "M3 scheduler: execution failed ETIMEDOUT" match

# The state file and the journal line.
# state ROOT SESSION_ID [VAR=value...]: run the hook as uwsm does; print the state file.
state() {
	_sr=$1 _ss=$2
	shift 2
	rm -f "$_sr/run/user/$UID_/mesa-m3-session.state" "$_sr$T/home/.local/state/mesa-m3/attempt" "$T/log"
	env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" XDG_SESSION_ID="$_ss" \
		XDG_CURRENT_DESKTOP=Hyprland "$@" \
		sh -c 'MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; mesa_m3_session_env "$2"' sh "$ENVFILE" "$_sr"
	cat "$_sr/run/user/$UID_/mesa-m3-session.state" 2>/dev/null
}
check_state() { # NAME DECISION REASON FALLBACK TEXT [PACKAGE_VERSION]
	_n=$1 _d=$2 _rs=$3 _fb=$4 _t=$5 _pv=${6:-26.1.4.m3.1-1}
	_keys=$(printf '%s\n' "$_t" | sed -n 's/=.*//p' | sort | tr '\n' ' ')
	_want='boot_id decision detail fallback hook_run_id package_version prefix reason schema session_id uid user_manager_invocation_id written_at '
	if [ "$(printf '%s\n' "$_t" | head -n 1)" = "schema=aurora.mesa-m3-session/2" ] && [ "$_keys" = "$_want" ] &&
		printf '%s\n' "$_t" | grep -qx "decision=$_d" && printf '%s\n' "$_t" | grep -qx "reason=$_rs" &&
		printf '%s\n' "$_t" | grep -qx "fallback=$_fb" &&
		printf '%s\n' "$_t" | grep -qx "session_id=5" && printf '%s\n' "$_t" | grep -qx "uid=$UID_" &&
		printf '%s\n' "$_t" | grep -qx "prefix=/opt/mesa-m3" &&
		printf '%s\n' "$_t" | grep -qx "user_manager_invocation_id=0123456789abcdef0123456789abcdef" &&
		printf '%s\n' "$_t" | grep -qxE "hook_run_id=[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}" &&
		printf '%s\n' "$_t" | grep -qx "boot_id=boot-a" &&
		printf '%s\n' "$_t" | grep -qx "package_version=$_pv" &&
		printf '%s\n' "$_t" | grep -qxE "written_at=[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z" &&
		[ "$(wc -l < "$T/log")" -eq 1 ] &&
		grep -q -- "-t mesa-m3 decision=$_d reason=$_rs fallback=$_fb session_id=5 user_manager_invocation_id=0123456789abcdef0123456789abcdef " "$T/log"; then
		echo "ok    state $_n: decision=$_d reason=$_rs fallback=$_fb, every key once, one matching journal line"
	else
		echo "FAIL  state $_n:"; printf '%s\n' "$_t" | sed 's/^/      /'; sed 's/^/      log: /' "$T/log"; fail=1
	fi
}
check_state active active active none "$(state "$J516S" 5)"
check_state opt-out inactive opt-out software "$(state "$T/optout1" 5)"
check_state not-supported inactive not-supported none "$(state "$T/j514c" 5)"
check_state "active (T8122, opted in)" active active none "$(state "$J613" 5)"
check_state "experimental (T8122, node open)" inactive experimental software "$(state "$J613X" 5)"
check_state "experimental (T8122, not armed)" inactive experimental none "$(state "$T/j613xoff" 5)"
check_state "gpu-fault (T8122)" inactive gpu-fault software "$(state "$J613" 5 FAKE_JOURNAL=g15g)"
check_state "no-gpu (T8122, not armed)" inactive no-gpu none "$(state "$T/j613off" 5)"
check_state no-gpu inactive no-gpu none "$(state "$T/kernelonly" 5)"
check_state "no-gpu (two nodes, the M3 one open)" inactive no-gpu software "$(state "$T/twonodes" 5)"
check_state no-display inactive no-display software "$(state "$T/nodcp" 5)"
check_state user-setup inactive user-setup none "$(state "$T/chonkdef" 5 XDG_CURRENT_DESKTOP=chonkstep)"
check_state "user-setup (Hyprland, env-hyprland)" inactive user-setup none "$(state "$T/hyprenv" 5)"
check_state "active (Hyprland, chonkstep's file ignored)" active active none "$(state "$T/chonkdef" 5)"
check_state incomplete-prefix inactive incomplete-prefix software "$(state "$T/noprefix" 5)" unknown
check_state missing-soname inactive missing-soname software "$(state "$T/nosoname" 5)"
check_state load-failed inactive load-failed software "$(state "$T/noload" 5 FAKE_LOAD=fail)"
check_state log-unreadable inactive log-unreadable software "$(state "$J516S" 5 FAKE_JOURNAL=noperm)"
check_state gpu-fault inactive gpu-fault software "$(state "$J516S" 5 FAKE_JOURNAL=match)"
if [ ! -r "$T/noaccess/dev/dri/renderD128" ]; then
	check_state no-access inactive no-access none "$(state "$T/noaccess" 5)"
fi
R=$(mkroot unk "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$T/home/.config/uwsm"
printf 'export VK_ICD_FILENAMES=/x\n' > "$T/home/.config/uwsm/env"; chmod 000 "$T/home/.config/uwsm/env"
mkdir -p "$R$T/home/.config/uwsm"; cp -p "$T/home/.config/uwsm/env" "$R$T/home/.config/uwsm/env" 2>/dev/null ||
	{ printf 'x\n' > "$R$T/home/.config/uwsm/env"; chmod 000 "$R$T/home/.config/uwsm/env"; }
chmod 644 "$T/home/.config/uwsm/env"; rm -f "$T/home/.config/uwsm/env"
check_state user-setup-unknown inactive user-setup-unknown software "$(state "$R" 5)"
R=$(mkroot ldrep "apple,j516s apple,t6030" "$G15S" yes 1 yes); mkdir -p "$R$T/home/.config/uwsm"
printf 'export LD_LIBRARY_PATH=/usr/local/lib\n' > "$R$T/home/.config/uwsm/env"
check_state user-setup-ldpath inactive user-setup-ldpath software "$(state "$R" 5)"
printf 'export LD_LIBRARY_PATH=/usr/local/lib\nexport GALLIUM_DRIVER=zink\n' > "$R$T/home/.config/uwsm/env"
check_state "user-setup (ldpath and a Mesa choice)" inactive user-setup none "$(state "$R" 5)"
state "$J516S" 5 >/dev/null
st=$(ls -l "$J516S/run/user/$UID_/mesa-m3-session.state" | cut -c1-10)
[ "$st" = "-rw-r--r--" ] && echo "ok    state file mode 0644" || { echo "FAIL  state file mode $st"; fail=1; }
if ls -a "$J516S/run/user/$UID_" | grep -q '^\.mesa-m3'; then echo "FAIL  temporary file left"; fail=1; else echo "ok    no temporary file left"; fi
# A line break in a value: no state file (the previous one is not truncated), and the journal says why.
state "$J516S" 5 >/dev/null
prev=$(cat "$J516S/run/user/$UID_/mesa-m3-session.state")
rm -f "$T/log"
env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" XDG_SESSION_ID="5
reason=active" sh -c 'MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; mesa_m3_session_env "$2"' sh "$ENVFILE" "$J516S"
if [ "$(cat "$J516S/run/user/$UID_/mesa-m3-session.state")" = "$prev" ] &&
	grep -q "state not written, session_id holds a line break" "$T/log"; then
	echo "ok    a session id with a newline: state not written, the previous file untouched, journal says why"
else echo "FAIL  newline in session id: $(cat "$T/log")"; fail=1; fi
CR=$(printf '\r')
out=$(state "$J516S" "5${CR}")
if [ -z "$out" ] && grep -q "line break" "$T/log"; then echo "ok    a session id with a CR: no state file"; else echo "FAIL  CR in session id"; fail=1; fi
# No XDG_SESSION_ID: loginctl's display session is used.
out=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" \
	sh -c 'MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; mesa_m3_session_env "$2"; cat "$2/run/user/'"$UID_"'/mesa-m3-session.state"' sh "$ENVFILE" "$J516S" |
	grep '^session_id=')
[ "$out" = session_id=7 ] && echo "ok    session_id from loginctl when XDG_SESSION_ID is unset" || { echo "FAIL  $out"; fail=1; }

# Sourcing as uwsm does on this (non-T6030) machine: nothing exported, nothing left behind.
rm -f "$T/log"
diff_out=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" sh -c '
	set | sort > "$HOME/before"
	. "$1" > "$HOME/msg"
	set | sort > "$HOME/after"
	command -v mesa_m3_gate >/dev/null && echo "function left: mesa_m3_gate"
	diff "$HOME/before" "$HOME/after" | grep "^[<>]" | grep -v -E "^. (_=|LINENO=|PIPESTATUS=|BASH_)"
' sh "$ENVFILE")
if [ -n "$diff_out" ]; then echo "FAIL  sourcing left changes: $diff_out"; fail=1; else echo "ok    sourcing on a machine that is neither T6030 nor T8122 changes nothing"; fi
grep -q 'decision=inactive reason=not-supported' "$T/log" && echo "ok    journal: $(cat "$T/log")" || { echo "FAIL journal line"; fail=1; }

# The exports when active: exactly the declared variables, plus the run id.
rm -f "$J516S$T/home/.local/state/mesa-m3/attempt"
exp=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" LD_LIBRARY_PATH=/x sh -c '
	MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"
	unset MESA_M3_SESSION_ENV_NO_RUN
	env | sort > "$HOME/b2"
	mesa_m3_session_env "$2" >/dev/null
	env | sort > "$HOME/a2"
	diff "$HOME/b2" "$HOME/a2" | grep "^>" | grep -v -E "^> (_=)" | sed "s/^> //"
' sh "$ENVFILE" "$J516S")
want='AGX_MESA_DEBUG=nopromote
ASAHI_M3_EXPERIMENTAL=1
GBM_BACKENDS_PATH=/opt/mesa-m3/lib/gbm
HK_DEBUG_LOG=0
LD_LIBRARY_PATH=/opt/mesa-m3/lib:/x
LIBGL_DRIVERS_PATH=/opt/mesa-m3/lib/dri
MESA_M3_SESSION=/opt/mesa-m3
QSG_RHI_BACKEND=opengl
VK_DRIVER_FILES=/opt/mesa-m3/share/vulkan/icd.d/asahi_icd.aarch64.json
VK_ICD_FILENAMES=/opt/mesa-m3/share/vulkan/icd.d/asahi_icd.aarch64.json
VK_LOADER_LAYERS_DISABLE=~implicit~
__EGL_VENDOR_LIBRARY_FILENAMES=/opt/mesa-m3/share/glvnd/egl_vendor.d/50_mesa.json
MESA_M3_HOOK_RUN_ID=<uuid>'
got=$(printf '%s\n' "$exp" | sed "s/^\([A-Z_a-z0-9]*\)='\(.*\)'$/\1=\2/" |
	sed -E 's/^MESA_M3_HOOK_RUN_ID=[0-9a-f-]{36}$/MESA_M3_HOOK_RUN_ID=<uuid>/' | sort)
if [ "$got" = "$(printf '%s\n' "$want" | sort)" ]; then echo "ok    active: exactly the 12 declared variables and the run id exported"; else echo "FAIL  exports differ:"; printf '%s\n' "$got"; fail=1; fi
rm -f "$J516S$T/home/.local/state/mesa-m3/attempt"
dup=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" LD_LIBRARY_PATH=/opt/mesa-m3/lib:/x sh -c '
	MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; mesa_m3_session_env "$2" >/dev/null; echo "$LD_LIBRARY_PATH"' sh "$ENVFILE" "$J516S")
[ "$dup" = /opt/mesa-m3/lib:/x ] && echo "ok    LD_LIBRARY_PATH already holding the prefix is not extended" || { echo "FAIL  LD_LIBRARY_PATH $dup"; fail=1; }
ina=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" sh -c '
	MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; mesa_m3_session_env "$2" >/dev/null; env | grep -E "^(LD_LIBRARY_PATH|VK_|LIBGL_|GBM_|__EGL_|MESA_M3_(SESSION|FALLBACK))=" ' sh "$ENVFILE" "$T/kernelonly")
[ -z "$ina" ] && echo "ok    inactive without an M3 render node: nothing exported" || { echo "FAIL  inactive exported: $ina"; fail=1; }

# The detector on its own: tests/test-user-setup.sh.

# The fallback exports exactly LIBGL_ALWAYS_SOFTWARE=1, MESA_M3_FALLBACK=software and the run id.
rm -f "$J516S$T/home/.local/state/mesa-m3/attempt"
fb=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" FAKE_JOURNAL=match sh -c '
	MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; unset MESA_M3_SESSION_ENV_NO_RUN
	env | sort > "$HOME/b3"; mesa_m3_session_env "$2" >/dev/null; env | sort > "$HOME/a3"
	diff "$HOME/b3" "$HOME/a3" | grep "^>" | grep -v "^> _=" | sed "s/^> //"' sh "$ENVFILE" "$J516S" |
	sed -E 's/^MESA_M3_HOOK_RUN_ID=[0-9a-f-]{36}$/MESA_M3_HOOK_RUN_ID=<uuid>/')
[ "$fb" = "LIBGL_ALWAYS_SOFTWARE=1
MESA_M3_FALLBACK=software
MESA_M3_HOOK_RUN_ID=<uuid>" ] && echo "ok    gpu-fault with the node open: exactly the software fallback exported" ||
	{ echo "FAIL  fallback exports: $fb"; fail=1; }
fb=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" XDG_CURRENT_DESKTOP=chonkstep sh -c '
	MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; mesa_m3_session_env "$2" >/dev/null; env | grep -E "^(LIBGL_ALWAYS_SOFTWARE|MESA_M3_FALLBACK)="' sh "$ENVFILE" "$T/chonkdef")
[ -z "$fb" ] && echo "ok    user-setup: no fallback exported" || { echo "FAIL  user-setup exported $fb"; fail=1; }
fb=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" sh -c '
	MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; mesa_m3_session_env "$2" >/dev/null; env | grep -E "^(LIBGL_ALWAYS_SOFTWARE|MESA_M3_FALLBACK|MESA_M3_HOOK_RUN_ID)="' sh "$ENVFILE" "$T/j514c")
[ -z "$fb" ] && echo "ok    not-supported: no fallback and no run id exported" || { echo "FAIL  not-supported exported $fb"; fail=1; }
# T8122: the same software fallback on a fault, and the same exports when active.
rm -f "$J613$T/home/.local/state/mesa-m3/attempt"
fb=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" FAKE_JOURNAL=g15g sh -c '
	MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; unset MESA_M3_SESSION_ENV_NO_RUN
	env | sort > "$HOME/b4"; mesa_m3_session_env "$2" >/dev/null; env | sort > "$HOME/a4"
	diff "$HOME/b4" "$HOME/a4" | grep "^>" | grep -v "^> _=" | sed "s/^> //"' sh "$ENVFILE" "$J613" |
	sed -E 's/^MESA_M3_HOOK_RUN_ID=[0-9a-f-]{36}$/MESA_M3_HOOK_RUN_ID=<uuid>/')
[ "$fb" = "LIBGL_ALWAYS_SOFTWARE=1
MESA_M3_FALLBACK=software
MESA_M3_HOOK_RUN_ID=<uuid>" ] && echo "ok    T8122 gpu-fault with the node open: exactly the software fallback exported" ||
	{ echo "FAIL  T8122 fallback exports: $fb"; fail=1; }
# T8122 without the opt-in: exactly the software fallback, and no attempt.
fb=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" sh -c '
	MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; unset MESA_M3_SESSION_ENV_NO_RUN
	env | sort > "$HOME/b6"; mesa_m3_session_env "$2" >/dev/null; env | sort > "$HOME/a6"
	diff "$HOME/b6" "$HOME/a6" | grep "^>" | grep -v "^> _=" | sed "s/^> //"' sh "$ENVFILE" "$J613X" |
	sed -E 's/^MESA_M3_HOOK_RUN_ID=[0-9a-f-]{36}$/MESA_M3_HOOK_RUN_ID=<uuid>/')
[ "$fb" = "LIBGL_ALWAYS_SOFTWARE=1
MESA_M3_FALLBACK=software
MESA_M3_HOOK_RUN_ID=<uuid>" ] && [ ! -e "$J613X$T/home/.local/state/mesa-m3/attempt" ] &&
	echo "ok    T8122 without the opt-in: exactly the software fallback exported, no attempt" ||
	{ echo "FAIL  T8122 experimental exports: $fb"; fail=1; }
rm -f "$J613$T/home/.local/state/mesa-m3/attempt"
exp8=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" LD_LIBRARY_PATH=/x sh -c '
	MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; unset MESA_M3_SESSION_ENV_NO_RUN
	env | sort > "$HOME/b5"; mesa_m3_session_env "$2" >/dev/null; env | sort > "$HOME/a5"
	diff "$HOME/b5" "$HOME/a5" | grep "^>" | grep -v -E "^> (_=)" | sed "s/^> //"' sh "$ENVFILE" "$J613" |
	sed "s/^\([A-Z_a-z0-9]*\)='\(.*\)'$/\1=\2/" | sed -E 's/^MESA_M3_HOOK_RUN_ID=[0-9a-f-]{36}$/MESA_M3_HOOK_RUN_ID=<uuid>/' | sort)
if [ "$exp8" = "$(printf '%s\n' "$want" | sort)" ]; then echo "ok    T8122 active: the same 12 variables and the run id as on the T6030"; else echo "FAIL  T8122 exports differ:"; printf '%s\n' "$exp8"; fail=1; fi
rm -f "$J613$T/home/.local/state/mesa-m3/attempt"
# A login after a fallback login (uwsm cleanup missed): the leftover pair is the hook's own.
rm -f "$J516S$T/home/.local/state/mesa-m3/attempt"
lo=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" LIBGL_ALWAYS_SOFTWARE=1 MESA_M3_FALLBACK=software \
	MESA_M3_SESSION_ENV_NO_RUN=1 sh -c '. "$1"; mesa_m3_gate "$2"' sh "$ENVFILE" "$J516S")
case "$lo" in active*) echo "ok    a leftover software fallback is not a user setup: $lo" ;; *) echo "FAIL  leftover fallback: $lo"; fail=1 ;; esac

# No login loop (review F1, round 3): an active login leaves an attempt bound
# to it; until a success marker with the same boot_id, session_id and
# hook_run_id exists (mesa-m3-session-confirm writes it only for a live,
# answering compositor; see test-confirm.sh), the next login of the same boot
# stays off /opt/mesa-m3.
ATT="$J516S$T/home/.local/state/mesa-m3/attempt"
OKM="$J516S$T/home/.local/state/mesa-m3/session-ok"
run() { # [VAR=value...]: one hook run on J516S; prints its state's decision and reason
	env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" XDG_SESSION_ID=5 XDG_CURRENT_DESKTOP=Hyprland "$@" \
		sh -c 'MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"; mesa_m3_session_env "$2"; echo "run=$MESA_M3_HOOK_RUN_ID"' sh "$ENVFILE" "$J516S" > "$T/run.out"
	st="$J516S/run/user/$UID_/mesa-m3-session.state"
	echo "$(sed -n 's/^decision=//p' "$st") $(sed -n 's/^reason=//p' "$st") $(sed -n 's/^fallback=//p' "$st")"
}
marker() { # BOOT SESSION RUN: a success marker as mesa-m3-session-confirm writes it
	printf 'schema=aurora.mesa-m3-session-ok/1\nboot_id=%s\nsession_id=%s\nhook_run_id=%s\ncompositor_pid=4242\ncompositor_starttime=1\ncompositor_comm=Hyprland\nwayland_socket=/run/user/1000/wayland-1\nsettle_s=15\nchecked_at=2026-10-07T00:00:00Z\n' \
		"$1" "$2" "$3" > "$OKM"
}
rm -f "$ATT" "$OKM"
r1=$(run); run1=$(sed -n 's/^run=//p' "$T/run.out")
[ "$r1" = "active active none" ] && [ "$(head -n 1 "$ATT")" = schema=aurora.mesa-m3-attempt/2 ] &&
	grep -qx "hook_run_id=$run1" "$ATT" && grep -qx 'session_id=5' "$ATT" && grep -qx 'boot_id=boot-a' "$ATT" &&
	! grep -q '^status=' "$ATT" && [ "$(sed -n 's/^hook_run_id=//p' "$J516S/run/user/$UID_/mesa-m3-session.state")" = "$run1" ] &&
	echo "ok    loop: an active login records an attempt bound to boot_id, session_id and hook_run_id (= state = session)" ||
	{ echo "FAIL  loop 1: $r1 $(cat "$ATT")"; fail=1; }
r2=$(run)
[ "$r2" = "inactive previous-failed software" ] && grep -q 'reason=previous-failed fallback=software' "$T/log" &&
	grep -q "hook_run_id $run1) has no success marker" "$T/log" && grep -q "rm $ATT, then log out and in" "$T/log" &&
	echo "ok    loop: no success marker: the next login of the boot is previous-failed, in software, and says how to retry" ||
	{ echo "FAIL  loop 2: $r2 $(cat "$T/log")"; fail=1; }
# A previous-failed login writes no new attempt: the failed one stays.
grep -qx "hook_run_id=$run1" "$ATT" && echo "ok    loop: previous-failed keeps the failed attempt" || { echo "FAIL  loop: attempt replaced"; fail=1; }
# A marker bound to that login: the next login is active again.
marker boot-a 5 "$run1"
r3=$(run); run3=$(sed -n 's/^run=//p' "$T/run.out")
[ "$r3" = "active active none" ] && echo "ok    loop: a marker bound to the attempt: the next login is active" ||
	{ echo "FAIL  loop 3: $r3"; fail=1; }
# Stale markers: from the earlier login (run1, while the attempt is now run3),
# from another boot, from another session: previous-failed.
r4=$(run)
[ "$r4" = "inactive previous-failed software" ] &&
	echo "ok    loop: a stale marker from an earlier login (other hook_run_id) does not count" ||
	{ echo "FAIL  loop 4: $r4"; fail=1; }
marker boot-z 5 "$run3"; r5=$(run)
marker boot-a 6 "$run3"; r6=$(run)
printf 'schema=aurora.mesa-m3-session-ok/0\nboot_id=boot-a\nsession_id=5\nhook_run_id=%s\n' "$run3" > "$OKM"; r7=$(run)
[ "$r5 $r6 $r7" = "inactive previous-failed software inactive previous-failed software inactive previous-failed software" ] &&
	echo "ok    loop: a marker of another boot, of another session or with another schema does not count" ||
	{ echo "FAIL  loop 5-7: $r5 / $r6 / $r7"; fail=1; }
marker boot-a 5 "$run3"; r8=$(run)
[ "$r8" = "active active none" ] && echo "ok    loop: the matching marker counts" || { echo "FAIL  loop 8: $r8"; fail=1; }
# An attempt from another boot does not count.
rm -f "$ATT" "$OKM"; run >/dev/null; echo boot-b > "$J516S/proc/sys/kernel/random/boot_id"
r9=$(run)
[ "$r9" = "active active none" ] && echo "ok    loop: an attempt of another boot is ignored" || { echo "FAIL  loop 9: $r9"; fail=1; }
echo boot-a > "$J516S/proc/sys/kernel/random/boot_id"; rm -f "$ATT" "$OKM"
# Round 4 (review R1): systemd removes /run/user/<uid> after the last session
# ends. The guard lives in ~/.local/state/mesa-m3, so a slow re-login still
# sees the failed attempt.
rm -f "$ATT" "$OKM"; r=$(run); runa=$(sed -n 's/^run=//p' "$T/run.out")
rm -rf "$J516S/run/user/$UID_"; mkdir -p "$J516S/run/user/$UID_"
rb=$(run)
[ "$r" = "active active none" ] && [ "$rb" = "inactive previous-failed software" ] && grep -qx "hook_run_id=$runa" "$ATT" &&
	grep -q "rm $ATT, then log out and in" "$T/log" &&
	[ "$(stat -c %a "$(dirname "$ATT")")" = 700 ] &&
	echo "ok    loop: the runtime directory emptied between logins: still previous-failed (state dir 0700)" ||
	{ echo "FAIL  loop R1: $r / $rb $(cat "$T/log")"; fail=1; }
# Without the confirm script or mesa-m3-wlcheck no marker could ever come: incomplete-prefix.
mv "$J516S/opt/mesa-m3/libexec/mesa-m3-wlcheck" "$T/wlc"
expect "$J516S" incomplete-prefix "mesa-m3-wlcheck is missing"
mv "$T/wlc" "$J516S/opt/mesa-m3/libexec/mesa-m3-wlcheck"

# Review F4: a gate file with a syntax error must not stop the uwsm preloader.
mkdir -p "$T/broken"; sed 's/^\tif \[ -e "\$_mm3_root\/etc\/mesa-m3\/disable" \]; then$/\tif then/' "$ENVFILE" > "$T/broken/gate"
grep -q '^	if then$' "$T/broken/gate" || { echo "FAIL  could not break the gate"; fail=1; }
sed "s|/opt/mesa-m3/libexec/mesa-m3-session-env|$T/broken/gate|g" "$HERE/../50-mesa-m3" > "$T/broken/50-mesa-m3"
out=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" bash --posix -c '. "$1"; echo "preloader continues rc=$?"; env | grep -c "^MESA_M3" || true' sh "$T/broken/50-mesa-m3" 2>&1)
case "$out" in "preloader continues rc=2"*0) echo "ok    a broken gate is skipped (sh -n), the preloader continues, nothing exported" ;;
*) echo "FAIL  broken gate: $out"; fail=1 ;; esac
out=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" bash --posix -c '. "$1"; echo "after rc=$?"' sh "$T/broken/gate" 2>&1 | tail -n 1)
case "$out" in *"syntax error"*|*"after rc="*) echo "      (without the check, sourcing the broken gate: $out)" ;; esac

# Exact firmware/native profile checks, with a mocked params ioctl boundary.
NATIVE=$(mkroot native25 "apple,j613 apple,t8122" 'apple,agx-t8122' yes 1 yes)
mkdir -p "$NATIVE/etc/mesa-m3"
: > "$NATIVE/etc/mesa-m3/t8122-gpu-experiment"
echo j613-25g83-hal200 > "$NATIVE/etc/mesa-m3/t8122-profile"
gpu="$NATIVE/sys/devices/platform/gpu0/of_node"
printf '\000\000\000\032\000\000\000\006\000\000\000\002' > "$gpu/apple,firmware-compat"
printf '\000\000\000\001' > "$gpu/apple,j613-25g83-gpu-handoff"
cp -a "$NATIVE/opt/mesa-m3" "$NATIVE/opt/native25"
mv "$NATIVE/opt/native25" "$NATIVE/opt/mesa-m3/25g83"
p="$NATIVE/opt/mesa-m3/25g83"
rm "$p/lib/libvulkan_asahi.so" "$p/lib/dri/zink_dri.so"
ln -s libdril_dri.so "$p/lib/dri/asahi_dri.so"
mkdir -p "$p/share/vulkan/disabled"
echo j613-25g83-gl-only > "$p/share/mesa-m3/profile"
expect "$NATIVE" active T8122
echo j613-25g83 > "$NATIVE/etc/mesa-m3/t8122-profile"
expect "$NATIVE" profile-mismatch unknown
echo j613-25g83-hal200 > "$NATIVE/etc/mesa-m3/t8122-profile"
# Check selection in the caller shell as well as the gate subprocess.
out=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" sh -c '
 MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"
 mesa_m3_session_env "$2"
 printf "selected=%s\ndriver=%s\ngallium=%s\nicds=%s\nicdfiles=%s\n" "$MESA_M3_SESSION" "${MESA_LOADER_DRIVER_OVERRIDE-unset}" "$GALLIUM_DRIVER" "$VK_DRIVER_FILES" "$VK_ICD_FILENAMES"
' sh "$ENVFILE" "$NATIVE")
for expected in 'selected=/opt/mesa-m3/25g83' 'driver=unset' 'gallium=asahi' 'icds=/opt/mesa-m3/25g83/share/vulkan/disabled' 'icdfiles=/opt/mesa-m3/25g83/share/vulkan/disabled'; do
 if ! printf '%s\n' "$out" | grep -Fxq "$expected"; then echo "FAIL native export $expected: $out"; fail=1; fi
done
expect "$NATIVE" profile-mismatch 'runtime GPU ABI' none FAKE_ABI=bad
rm "$gpu/apple,j613-25g83-gpu-handoff"
expect "$NATIVE" profile-mismatch 'exact firmware'
printf '\000\000\000\001' > "$gpu/apple,j613-25g83-gpu-handoff"
printf '\000\000\000\032\000\000\000\006\000\000\000\003' > "$gpu/apple,firmware-compat"
expect "$NATIVE" profile-mismatch 'exact firmware'
printf '\000\000\000\032\000\000\000\006\000\000\000\002' > "$gpu/apple,firmware-compat"
rm "$NATIVE/etc/mesa-m3/t8122-profile"
expect "$NATIVE" profile-mismatch 'explicit native'
echo unknown > "$NATIVE/etc/mesa-m3/t8122-profile"
expect "$NATIVE" profile-mismatch unknown
echo j613-25g83-hal200 > "$NATIVE/etc/mesa-m3/t8122-profile"
rm "$p/share/mesa-m3/profile"
expect "$NATIVE" incomplete-prefix 'capability marker'
echo j613-25g83-gl-only > "$p/share/mesa-m3/profile"
: > "$NATIVE/etc/mesa-m3/disable"
expect "$NATIVE" opt-out '/etc/mesa-m3/disable'
rm "$NATIVE/etc/mesa-m3/disable"
# The J615 (15") shares the J613's 25G83 DCP, PMP and GPU images; it gets the
# same profile only with m1n1's experimental switch, which m1n1 copies to
# /chosen as the string "1" (m1n1's handoffs and the kernel check it too).
J615C="$NATIVE/proc/device-tree/chosen/asahi,j615-25g83-experimental"
nul apple,j615 apple,t8122 > "$NATIVE/proc/device-tree/compatible"
expect "$NATIVE" profile-mismatch 'J615 is experimental and requires chosen.asahi,j615-25g83-experimental=1'
mkdir -p "$NATIVE/proc/device-tree/chosen"
for v in '0\000' '11\000' '1' '1\000\000' ''; do
	printf "$v" > "$J615C"
	expect "$NATIVE" profile-mismatch 'J615 is experimental'
done
printf '1\000' > "$J615C"
expect "$NATIVE" active 'T8122 (J615 on the 25G83 profile, experimental)'
out=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" sh -c '
 MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"
 mesa_m3_session_env "$2"
 printf "selected=%s\ndriver=%s\nicds=%s\nprofile=%s\n" "$MESA_M3_SESSION" "${MESA_LOADER_DRIVER_OVERRIDE-unset}" "$VK_DRIVER_FILES" "$MESA_M3_PROFILE"
' sh "$ENVFILE" "$NATIVE")
for expected in 'selected=/opt/mesa-m3/25g83' 'driver=unset' 'icds=/opt/mesa-m3/25g83/share/vulkan/disabled' 'profile=j613-25g83-hal200'; do
 if printf '%s\n' "$out" | grep -Fxq "$expected"; then echo "ok    J615 native export $expected"; else echo "FAIL J615 native export $expected: $out"; fail=1; fi
done
# A root naming both Air boards cannot select either native route.
nul apple,j613 apple,j615 apple,t8122 > "$NATIVE/proc/device-tree/compatible"
for v in '0\000' '1\000'; do
 printf "$v" > "$J615C"
 expect "$NATIVE" profile-mismatch 'requires J613/T8122'
 out=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" sh -c '
  MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"
  mesa_m3_session_env "$2"
  printf "selected=%s\ndriver=%s\nicds=%s\nsoftware=%s\n" "${MESA_M3_SESSION:-}" "${MESA_LOADER_DRIVER_OVERRIDE:-}" "${VK_DRIVER_FILES:-}" "${LIBGL_ALWAYS_SOFTWARE:-}"
  grep -E "^(decision|reason)=" "$2/run/user/$(id -u)/mesa-m3-session.state"
 ' sh "$ENVFILE" "$NATIVE")
 for expected in 'selected=' 'driver=' 'icds=' 'software=1' 'decision=inactive' 'reason=profile-mismatch'; do
  if printf '%s\n' "$out" | grep -Fxq "$expected"; then echo "ok    ambiguous Air native refusal $v $expected"; else echo "FAIL ambiguous Air native refusal $v $expected: $out"; fail=1; fi
 done
done
nul apple,j615 apple,t8122 > "$NATIVE/proc/device-tree/compatible"
printf '1\000' > "$J615C"
expect "$NATIVE" active 'T8122 (J615 on the 25G83 profile, experimental)'
# Every other check of the profile still holds on the J615.
expect "$NATIVE" profile-mismatch 'runtime GPU ABI' none FAKE_ABI=bad
rm "$gpu/apple,j613-25g83-gpu-handoff"
expect "$NATIVE" profile-mismatch 'exact firmware'
printf '\000\000\000\001' > "$gpu/apple,j613-25g83-gpu-handoff"
printf '\000\000\000\032\000\000\000\006\000\000\000\003' > "$gpu/apple,firmware-compat"
expect "$NATIVE" profile-mismatch 'exact firmware'
printf '\000\000\000\032\000\000\000\006\000\000\000\002' > "$gpu/apple,firmware-compat"
rm "$p/share/mesa-m3/profile"
expect "$NATIVE" incomplete-prefix 'capability marker'
echo j613-25g83-gl-only > "$p/share/mesa-m3/profile"
rm "$NATIVE/etc/mesa-m3/t8122-profile"
expect "$NATIVE" profile-mismatch 'explicit native'
echo j613-25g83-hal200 > "$NATIVE/etc/mesa-m3/t8122-profile"
rm "$NATIVE/etc/mesa-m3/t8122-gpu-experiment"
expect "$NATIVE" experimental 'T8122'
: > "$NATIVE/etc/mesa-m3/t8122-gpu-experiment"
# The switch admits only the J615: any other T8122 board stays out.
nul apple,j504 apple,t8122 > "$NATIVE/proc/device-tree/compatible"
expect "$NATIVE" profile-mismatch 'requires J613/T8122'
rm -r "$NATIVE/proc/device-tree/chosen"
nul apple,j613 apple,t8122 > "$NATIVE/proc/device-tree/compatible"
expect "$NATIVE" active T8122
# The old Pro and 14.x fixtures continue to pass above; a stale native
# selector on Pro must fail without selecting the HAL200 library.
mkdir -p "$J516S/etc/mesa-m3"
echo j613-25g83-hal200 > "$J516S/etc/mesa-m3/t8122-profile"
expect "$J516S" profile-mismatch J613
rm "$J516S/etc/mesa-m3/t8122-profile"


# System Mesa handoff: opt-out must not strand a qualified native /usr
# installation in software rendering. Every failed admission retains recovery.
# The helper is a stub here; its real board/renderer/provenance tests are separate.
system_root() {
	case "$2" in
	air) _hd="apple,j613 apple,t8122"; _hg=$G15G ;;
	*) _hd="apple,j516s apple,t6030"; _hg=$G15S ;;
	esac
	_hr=$(mkroot "handoff-$1" "$_hd" "$_hg" yes 1 yes)
	mkdir -p "$_hr/etc/mesa-m3"
	: > "$_hr/etc/mesa-m3/disable"
	cat > "$_hr/opt/mesa-m3/libexec/mesa-m3-system-check" <<'H'
#!/bin/sh
here=${0%/*}
printf '%s\n' "$@" > "$here/system-check.args"
env | sort > "$here/system-check.env"
[ ! -e "$here/system-check.sleep" ] || sleep 8
[ ! -e "$here/system-check.fail" ]
H
	chmod +x "$_hr/opt/mesa-m3/libexec/mesa-m3-system-check"
	# The bounded admission sources the installed hook again in its own sh.
	cp "$ENVFILE" "$_hr/opt/mesa-m3/libexec/mesa-m3-session-env"
	echo "$_hr"
}
handoff() { # ROOT FALLBACK HELPER_CALLED [VAR=value...]
	_hr=$1 _hfb=$2 _hc=$3
	shift 3
	rm -f "$_hr/opt/mesa-m3/libexec/system-check.args"
	_ha="$_hr$T/home/.local/state/mesa-m3/attempt"
	_hbefore=missing
	[ ! -f "$_ha" ] || _hbefore=$(sha256sum "$_ha" | cut -d' ' -f1)
	_ho=$(env -i PATH="$T/bin:/usr/bin:/bin" HOME="$T/home" FAKE_LOG="$T/log" \
		XDG_CURRENT_DESKTOP=Hyprland "$@" "$TSH" -c '
		MESA_M3_SESSION_ENV_NO_RUN=1 . "$1"
		mesa_m3_session_env "$2"
		printf "software=%s\nmarker=%s\nlegacy=%s\n" "${LIBGL_ALWAYS_SOFTWARE:-}" "${MESA_M3_FALLBACK:-}" "${MESA_M3_SESSION:-}"
		cat "$2/run/user/$(id -u)/mesa-m3-session.state"
	' sh "$ENVFILE" "$_hr")
	_hafter=missing
	[ ! -f "$_ha" ] || _hafter=$(sha256sum "$_ha" | cut -d' ' -f1)
	_hcalled=no
	[ ! -e "$_hr/opt/mesa-m3/libexec/system-check.args" ] || _hcalled=yes
	if printf '%s\n' "$_ho" | grep -qx "fallback=$_hfb" &&
		printf '%s\n' "$_ho" | grep -qx 'decision=inactive' &&
		printf '%s\n' "$_ho" | grep -qx 'reason=opt-out' &&
		printf '%s\n' "$_ho" | grep -qx 'legacy=' &&
		[ "$_hc" = "$_hcalled" ] && [ "$_hbefore" = "$_hafter" ]; then
		echo "ok    ${_hr##*/}: fallback=$_hfb helper=$_hc, legacy attempt unchanged"
	else
		echo "FAIL  ${_hr##*/}: want fallback=$_hfb helper=$_hc; got helper=$_hcalled attempt=$_hbefore/$_hafter: $_ho"
		fail=1
	fi
	if [ "$_hfb" = none ]; then
		if ! printf '%s\n' "$_ho" | grep -qx 'software=' ||
			! printf '%s\n' "$_ho" | grep -qx 'marker=' ||
			! printf '%s\n' "$_ho" | grep -q 'detail=.*system Mesa on /usr admits'; then
			echo "FAIL  ${_hr##*/}: system admission exports or state detail: $_ho"; fail=1
		fi
	else
		if ! printf '%s\n' "$_ho" | grep -qx 'software=1' ||
			! printf '%s\n' "$_ho" | grep -qx 'marker=software' ||
			printf '%s\n' "$_ho" | grep -q 'detail=.*system Mesa on /usr admits'; then
			echo "FAIL  ${_hr##*/}: recovery exports or state detail: $_ho"; fail=1
		fi
	fi
}
HRA=$(system_root user-optout air)
rm "$HRA/etc/mesa-m3/disable"
mkdir -p "$HRA$T/home/.config/mesa-m3"
: > "$HRA$T/home/.config/mesa-m3/disable"
handoff "$HRA" none yes
if [ "$(cat "$HRA/opt/mesa-m3/libexec/system-check.args")" = "$(printf '/dev/dri/renderD128\nt8122')" ] &&
	grep -qx 'PATH=/usr/bin:/bin' "$HRA/opt/mesa-m3/libexec/system-check.env" &&
	[ -z "$(grep -vE '^(PATH=/usr/bin:/bin|PWD=.*|SHLVL=[0-9]+|_=.*)$' "$HRA/opt/mesa-m3/libexec/system-check.env")" ]; then
	echo 'ok    handoff: helper receives exact device/SoC and empty selector environment'
else echo 'FAIL  handoff: helper argv/environment'; fail=1; fi
HRP=$(system_root root-optout pro)
handoff "$HRP" none yes
handoff "$HRP" none yes LIBGL_ALWAYS_SOFTWARE=1 MESA_M3_FALLBACK=software
for mode in fail sleep; do
	HR=$(system_root "probe-$mode" pro)
	: > "$HR/opt/mesa-m3/libexec/system-check.$mode"
	handoff "$HR" software yes
done
for mode in missing not-executable; do
	HR=$(system_root "probe-$mode" pro)
	if [ "$mode" = missing ]; then rm "$HR/opt/mesa-m3/libexec/mesa-m3-system-check";
	else chmod 600 "$HR/opt/mesa-m3/libexec/mesa-m3-system-check"; fi
	handoff "$HR" software no
done
for mode in missing-cmdline missing-detector; do
	HR=$(system_root "$mode" pro)
	if [ "$mode" = missing-cmdline ]; then rm "$HR/proc/cmdline";
	else rm "$HR/opt/mesa-m3/libexec/mesa-m3-user-setup"; fi
	handoff "$HR" software no
done
for flag in mesa_m3=off nomodeset; do
	HR=$(system_root "kernel-$flag" pro)
	echo "root=UUID=x rw $flag quiet" > "$HR/proc/cmdline"
	handoff "$HR" software no
done
for journal in noperm error match bank dcp g15g verdict; do
	HR=$(system_root "journal-$journal" pro)
	handoff "$HR" software no FAKE_JOURNAL="$journal"
done
for dt in 'apple,j615 apple,t8122' 'apple,j514s apple,t6030' \
	'apple,j614s apple,t6030' 'apple,j613 apple,j615 apple,t8122' \
	'apple,j613 apple,t6030' 'apple,j516s apple,t6030 apple,t8122'; do
	HR=$(system_root "board-$(printf '%s' "$dt" | tr ' ,' '--')" pro)
	nul $dt > "$HR/proc/device-tree/compatible"
	# Match the GPU to the ambiguous board's first SoC so the old fallback
	# remains observable; the system handoff must still refuse it.
	case "$dt" in *t8122*) nul apple,agx-t8122 apple,agx-t6030 > "$HR/sys/devices/platform/gpu0/of_node/compatible" ;; esac
	handoff "$HR" software no
done
HR=$(system_root no-display pro)
rm "$HR/sys/class/drm/card2/device"
handoff "$HR" software no
HR=$(system_root wrong-display pro)
nul apple,t8122-display-subsystem > "$HR/sys/devices/platform/display/of_node/compatible"
handoff "$HR" software no
HR=$(system_root two-nodes pro)
mkdir "$HR/sys/class/drm/renderD129"
handoff "$HR" software no
for variable in LIBGL_ALWAYS_SOFTWARE=1 \
	LD_LIBRARY_PATH=/opt/mesa-m3/lib \
	VK_DRIVER_FILES=/opt/mesa-m3/share/vulkan/icd.d/asahi_icd.aarch64.json \
	__EGL_VENDOR_LIBRARY_FILENAMES=/opt/mesa-m3/share/glvnd/egl_vendor.d/50_mesa.json \
	ASAHI_M3_EXPERIMENTAL=1 XDG_CURRENT_DESKTOP=chonkstep; do
	HR=$(system_root "override-${variable%%=*}" pro)
	handoff "$HR" software no "$variable"
done
for value in 'LIBGL_DRIVERS_PATH=/opt/mesa-m3/lib/dri' 'VK_DRIVER_FILES=$UNKNOWN'; do
	HR=$(system_root "late-${value%%=*}" pro)
	mkdir -p "$HR$T/home/.config/uwsm"
	printf '%s\n' "$value" > "$HR$T/home/.config/uwsm/env"
	handoff "$HR" software no
done
for variable in GBM_BACKEND=nvidia-drm MESA_GL_VERSION_OVERRIDE=4.6; do
	HR=$(system_root "override-${variable%%=*}" pro)
	handoff "$HR" software no "$variable"
done
HR=$(system_root no-installed-hook pro)
rm "$HR/opt/mesa-m3/libexec/mesa-m3-session-env"
handoff "$HR" software no
# A stalled journal or scanner refuses within the bound (20 s, then 2 s to
# kill) and keeps the software fallback, instead of holding the login.
HR=$(system_root stalled-journal pro)
_hstart=$(date +%s)
handoff "$HR" software no FAKE_JOURNAL=stall
_hspent=$(( $(date +%s) - _hstart ))
if [ "$_hspent" -le 30 ]; then echo "ok    stalled-journal: refused after ${_hspent}s"
else echo "FAIL  stalled-journal: took ${_hspent}s"; fail=1; fi
HR=$(system_root stalled-detector pro)
printf '#!/bin/sh\nsleep 60\n' > "$HR/opt/mesa-m3/libexec/mesa-m3-user-setup"
_hstart=$(date +%s)
handoff "$HR" software no
_hspent=$(( $(date +%s) - _hstart ))
if [ "$_hspent" -le 30 ]; then echo "ok    stalled-detector: refused after ${_hspent}s"
else echo "FAIL  stalled-detector: took ${_hspent}s"; fail=1; fi
HR=$(system_root failed-legacy-login pro)
HATT="$HR$T/home/.local/state/mesa-m3/attempt"
HOK="$HR$T/home/.local/state/mesa-m3/session-ok"
mkdir -p "${HATT%/*}"
printf 'schema=aurora.mesa-m3-attempt/2\nboot_id=boot-a\nsession_id=5\nhook_run_id=old-run\n' > "$HATT"
handoff "$HR" software no
printf 'schema=aurora.mesa-m3-session-ok/1\nboot_id=boot-a\nsession_id=wrong\nhook_run_id=old-run\n' > "$HOK"
handoff "$HR" software no
sed -i 's/session_id=wrong/session_id=5/' "$HOK"
handoff "$HR" none yes
rm "$HOK"
sed -i 's/boot_id=boot-a/boot_id=old-boot/' "$HATT"
handoff "$HR" none yes

[ "$fail" = 0 ] && echo "ALL PASS" || echo "SOME FAILED"
exit "$fail"
