#!/bin/bash
# air-gpu-oneshot.sh: arm ONE boot of this M3 MacBook Air with the GPU start switch
# (asahi.t8122_start=1) and the given asahi knobs on the kernel command line.
#
#   sudo air-gpu-oneshot.sh start                    arm one boot with the kernel's defaults
#   sudo air-gpu-oneshot.sh <knob>=<value> ...       arm one boot with these knobs as well
#   sudo air-gpu-oneshot.sh --check [knobs]          say whether this Mac can arm; changes nothing
#   sudo air-gpu-oneshot.sh --status                 what is armed, and what this boot ran with
#   sudo air-gpu-oneshot.sh --disarm                 clear the one-shot and remove its boot entry
#   air-gpu-oneshot.sh --list-knobs                  the knobs this script accepts
#   --allow-test-on-m3pro                            also run on an M3 Pro (t6030): a test of the boot steps
#
# Knobs are written without the "asahi." prefix (t8122_pstate_cap=2); the prefix is accepted too.
#
# How it works. The asahi GPU driver is built into the kernel (CONFIG_DRM_ASAHI=y), and it
# starts the GPU at kernel init, before the initramfs runs. Module options in /etc/modprobe.d
# never reach it, and no userspace step runs before it. So the knobs go on the kernel command
# line of one extra Limine entry, "air-gpu-oneshot", which boots the same UKI as the normal
# linux-aurora entry, with that entry's command line plus the knobs. The entry is chosen for one
# boot only, through the Boot Loader Interface variable LoaderEntryOneShot. Limine deletes that
# variable as it reads it, before it starts the kernel, and U-Boot writes the deletion to its
# variable file on the EFI partition straight away. Whatever happens in the armed boot, the next
# boot (a reboot, a reset, a power cycle) is the normal default entry.
#
# Every change is made under the Limine tools' locks, checked after it is made, and undone if
# a check fails. Each run prints one line saying what it did or why it refused.
set -euo pipefail

# ---------------------------------------------------------------------------------------------
# The knobs this script accepts: name, the values it takes (an extended regex for the whole
# value), and what the knob sets. They are the kernel's (air/t8122-gpu-start: t8122_knobs.rs and
# m3_params.rs), with the values the kernel accepts; knob_ok below adds the checks a regex can't
# make. A knob must also be a parameter of the kernel the armed entry boots: the script looks for
# the string "asahi.<name>" in that kernel image (the UKI) and refuses one it does not find.
KNOBS='
t8122_start             1                                  the T8122 start experiment (always set by this script)
t8122_initdata_version  0x[0-9a-fA-F]{1,16}|[1-9][0-9]{0,19}  InitData version for the firmware (default 0x0c08e21e83800490)
t8122_fender            rule|adt|0x104000|0x12c000         Fender window: rule 0x104000 (default) or adt 0x12c000
t8122_clkgen            e1c|e5c|none                       clock-generator slot: SGX+0xe1c000 (default), SGX+0xe5c000, none
t8122_sgx_setup         none|t6030                         SGX write before the firmware starts: none (default) or T6030 0x70001
t8122_unit_mask_a       0x[0-9a-fA-F]{1,9}|[1-9][0-9]{0,10}  HwDataB unit mask A: nonzero, within 0x700000001 (default)
t8122_unit_mask_b       0x[1-7]|[1-7]                      HwDataB unit mask B: nonzero, within 0x7 (default 0x3)
t8122_pstate_cap        [1-5]                              ceiling 1..5; default thermal, fallback 2
m3_expose               -1|0|1                             render node: -1 auto (on for J613/J615, the default), 0 off, 1 on
m3_timeout_nohang       0|1                                on a job timeout, fail the job without the hang path
'

# Whether a decimal or 0x-hex string fits in 64 bits. The regexes bound hex to 16 digits
# (<= 2^64-1) and forbid a leading zero in a decimal (so no octal and no $(( )) parse error); this
# bounds a decimal to 2^64-1 before any arithmetic, which would otherwise wrap silently.
u64_ok() {
  local t=$1
  [[ $t == 0x* || $t == 0X* ]] && return 0
  (( ${#t} < 20 )) && return 0
  # A 20-digit decimal (both strings 20 chars, so a lexical compare is the numeric one): <= 2^64-1.
  # shellcheck disable=SC2071  # a string compare is wanted: equal-length digit strings
  (( ${#t} == 20 )) && [[ ! $t > 18446744073709551615 ]]
}

# The checks on a knob value that the regex can't make, as the kernel makes them.
knob_ok() { # name value
  local v=$2
  case $1 in
    t8122_initdata_version | t8122_unit_mask_a | t8122_unit_mask_b) u64_ok "$v" || return 1; v=$((v)) ;;
    *) return 0 ;;
  esac
  case $1 in
    # Nonzero, and not the kernel's two reserved values (2^64-1 "not given", 2^64-2 "invalid"),
    # which bash arithmetic reads as -1 and -2.
    t8122_initdata_version) ((v != 0 && v != -1 && v != -2)) ;;
    t8122_unit_mask_a) ((v != 0 && (v & ~0x700000001) == 0)) ;;
    t8122_unit_mask_b) ((v != 0 && (v & ~0x7) == 0)) ;;
  esac
}
# ---------------------------------------------------------------------------------------------

PROG=air-gpu-oneshot
ENTRY=air-gpu-oneshot
# The normal entry whose UKI and command line the armed entry copies (limine-entry-tool names
# it after the kernel package, linux-aurora, under the OS directory).
SOURCE_ENTRY=linux-aurora
BEGIN_MARK="# >>> air-gpu-oneshot: one experiment boot, chosen only through LoaderEntryOneShot (remove with: air-gpu-oneshot.sh --disarm)"
END_MARK="# <<< air-gpu-oneshot"
# The tag on the armed command line: the arming id. air-gpu-collect.sh reads it.
TAG=air_gpu.oneshot
AIR_BOARDS="j613 j615"
DT=/proc/device-tree
EFIVARS=/sys/firmware/efi/efivars
CMDLINE=/proc/cmdline
BLI_GUID=4a67b082-0a4c-41cf-b6c7-440b29bb8c4f
UBOOT_GUID=b2ac5fc9-92b7-4acd-aeac-11e818c3130c
GLOBAL_GUID=8be4df61-93ca-11d2-aa0d-00e098032b8c
# Omarchy's Limine tools take one of these before they touch the EFI partition (newer ones the
# first, older ones the second); maclab takes both in this order, and so does this script.
LOCKS="/run/lock/boot-partition.lock /tmp/limine-global.lock"
STATE_DIR=/var/lib/air-gpu
# limine-entry-tool's settings; ENABLE_ENROLL_LIMINE_CONFIG=yes there makes Limine check a hash
# of limine.conf enrolled into its EFI binary, so an edited limine.conf would not boot.
LIMINE_DEFAULTS=/etc/default/limine
KREL=$(uname -r)
ALLOW_M3PRO=0
# Where Limine looks for its configuration on the partition it was loaded from, in its order.
CONF_PATHS="EFI/BOOT/limine.conf boot/limine/limine.conf boot/limine.conf limine/limine.conf limine.conf"

is_root() { ((EUID == 0)); }
say() { printf '%s: %s\n' "$PROG" "$*"; }
# --check and --status write nothing, not even a journal line.
QUIET_LOG=0
log() { ((QUIET_LOG)) || logger -t "$PROG" -- "$*" 2>/dev/null || true; }
refuse() {
  printf '%s: refused: %s Nothing was changed.\n' "$PROG" "$*" >&2
  log "refused: $*"
  exit 1
}

# ---- what this Mac is ------------------------------------------------------------------------

compatibles() { tr '\0' '\n' <"$DT/compatible" 2>/dev/null || true; }
board() { compatibles | sed -n 's/^apple,\(j[0-9a-z]*\)$/\1/p' | head -1; }
is_air() {
  local b
  compatibles | grep -qx 'apple,t8122' || return 1
  b=$(board)
  [[ -n $b && " $AIR_BOARDS " == *" $b "* ]]
}
is_m3pro() { compatibles | grep -qx 'apple,t6030'; }

check_board() {
  if is_air; then return 0; fi
  if is_m3pro && ((ALLOW_M3PRO)); then
    say "test run on an M3 Pro ($(board)): the t8122 knobs do nothing on this GPU"
    return 0
  fi
  if is_m3pro; then
    refuse "this is an M3 Pro ($(board)), not an M3 MacBook Air (j613, j615); --allow-test-on-m3pro tests the boot steps there."
  fi
  refuse "this Mac ($(board), $(compatibles | sed -n 's/^apple,\(t[0-9]*\)$/\1/p' | head -1)) is not an M3 MacBook Air (j613, j615)."
}

# ---- knobs -----------------------------------------------------------------------------------

knob_line() { printf '%s\n' "$KNOBS" | awk -v n="$1" '$1 == n { print; exit }'; }

list_knobs() {
  echo "knob               values                         sets"
  printf '%s\n' "$KNOBS" | sed '/^$/d'
}

# Fills PARAMS ("asahi.name=value" words, t8122_start first) from the arguments, or refuses.
PARAMS=()
parse_knobs() {
  local arg name value line re seen=" "
  PARAMS=("asahi.t8122_start=1")
  for arg in "$@"; do
    [[ $arg == start ]] && continue
    [[ $arg == *=* ]] || refuse "\"$arg\" is not a knob=value (or start)."
    name=${arg%%=*} value=${arg#*=}
    name=${name#asahi.}
    line=$(knob_line "$name")
    [[ -n $line ]] || refuse "unknown knob \"$name\" (air-gpu-oneshot.sh --list-knobs lists them)."
    re=$(awk '{ print $2 }' <<<"$line")
    [[ $value =~ ^($re)$ ]] || refuse "$name=$value: $name takes $re."
    knob_ok "$name" "$value" ||
      refuse "$name=$value is outside what the kernel accepts ($(knob_line "$name" | awk '{ $1 = $2 = ""; sub(/^ +/, ""); print }'))."
    [[ $seen != *" $name "* ]] || refuse "$name is given twice."
    seen+="$name "
    [[ $name == t8122_start ]] && continue
    PARAMS+=("asahi.$name=$value")
  done
}

# ---- the EFI partition and Limine ------------------------------------------------------------

ESP=""
CONF=""
find_esp() {
  local d
  [[ -n $ESP ]] && return 0
  for d in /boot/efi /efi /boot; do
    if [[ $(findmnt -no TARGET "$d" 2>/dev/null) == "$d" && $(findmnt -no FSTYPE "$d" 2>/dev/null) == vfat ]]; then
      ESP=$d
      return 0
    fi
  done
  return 1
}

find_conf() {
  local p
  for p in $CONF_PATHS; do
    if [[ -f $ESP/$p ]]; then CONF=$ESP/$p; return 0; fi
  done
  return 1
}

# The source entry's keys, as "key value" lines: protocol, path, cmdline, kver, and count (how
# many entries matched).
entry_keys() {
  awk -v name="$SOURCE_ENTRY" '
    function trim(s) { sub(/^[ \t]+/, "", s); sub(/[ \t\r]+$/, "", s); return s }
    /^[ \t]*\// { inent = ($0 ~ "^[ \t]*//" name "[ \t\r]*$"); if (inent) n++; next }
    !inent || n != 1 { next }
    /^[ \t]*protocol:/ { v = $0; sub(/^[ \t]*protocol:/, "", v); print "protocol " trim(v) }
    /^[ \t]*path:/ { v = $0; sub(/^[ \t]*path:/, "", v); print "path " trim(v) }
    /^[ \t]*(kernel_)?cmdline:/ { v = $0; sub(/^[ \t]*(kernel_)?cmdline:/, "", v); print "cmdline " trim(v) }
    /^[ \t]*comment:[ \t]*Kernel version:/ { v = $0; sub(/^[ \t]*comment:[ \t]*Kernel version:/, "", v); print "kver " trim(v) }
    END { print "count " n + 0 }
  ' "$CONF"
}
key() { awk -v k="$1" '$1 == k { sub(/^[^ ]+ /, ""); print; exit }' <<<"$KEYS"; }

# A Limine path such as boot():/EFI/Linux/x.efi#<blake2b> as a file on the ESP.
limine_file() {
  local p=${1%%#*}
  [[ $p == *"):"* ]] && p=${p#*):}
  p=${p#/}
  printf '%s/%s\n' "$ESP" "$p"
}

# ---- EFI variables ---------------------------------------------------------------------------

var_path() { printf '%s/%s-%s\n' "$EFIVARS" "$1" "$2"; }
# A variable's data (without its 4 attribute bytes) as text: UTF-16LE when $3 is utf16.
var_text() {
  local f
  f=$(var_path "$1" "$2")
  [[ -e $f ]] || return 1
  if [[ ${3:-} == utf16 ]]; then
    tail -c +5 "$f" | iconv -f UTF-16LE -t UTF-8 2>/dev/null | tr -d '\0'
  else
    tail -c +5 "$f" | tr -d '\0'
  fi
}
utf16z() { printf '%s' "$1" | iconv -f UTF-8 -t UTF-16LE; printf '\0\0'; }
armed_name() { var_text LoaderEntryOneShot "$BLI_GUID" utf16 || true; }

# U-Boot keeps EFI variables in RAM while Linux runs and loads them from a file on the ESP at
# boot (RTStorageVolatile names it). VarToFile is the whole store in that file's format.
VAR_FILE=""
find_var_file() {
  local name
  VAR_FILE=""
  name=$(var_text RTStorageVolatile "$UBOOT_GUID") || return 0
  [[ -n $name && $name != *..* && $name != */* ]] || return 1
  VAR_FILE=$ESP/$name
}
uboot_store() { tail -c +5 "$(var_path VarToFile "$UBOOT_GUID")"; }

# Writes U-Boot's store to its file on the ESP, atomically, and syncs it.
persist() {
  local tmp
  [[ -n $VAR_FILE ]] || return 0
  tmp=$VAR_FILE.air-gpu.tmp
  uboot_store >"$tmp" || { rm -f "$tmp"; return 1; }
  if [[ $(head -c 15 "$tmp" | tail -c 7) != UbEfiVa ]]; then rm -f "$tmp"; return 1; fi
  sync "$tmp" && mv -f "$tmp" "$VAR_FILE" && sync "$(dirname "$VAR_FILE")"
}
# Whether U-Boot's variable file holds LoaderEntryOneShot naming the entry. U-Boot stores each
# variable's UTF-16LE name, its terminator, then its data; the bytes are compared as hex words so
# that LimineLastBootedEntry, which Limine sets to the armed entry during that boot, never counts.
hex_bytes() { od -An -v -tx1 | tr -s ' \n' ' '; }
file_has_name() {
  local want
  want=$({ printf 'LoaderEntryOneShot' | iconv -f UTF-8 -t UTF-16LE; printf '\0\0'; utf16z "$ENTRY"; } | hex_bytes)
  want=${want# } want=${want% }
  [[ " $(hex_bytes <"$1" 2>/dev/null) " == *" $want "* ]]
}

set_oneshot() {
  local f tmp
  f=$(var_path LoaderEntryOneShot "$BLI_GUID")
  tmp=$(mktemp)
  # Non-volatile, boot service and runtime access, then the UTF-16LE name: one write.
  { printf '\x07\x00\x00\x00'; utf16z "$ENTRY"; } >"$tmp"
  if [[ -e $f ]]; then chattr -i "$f" 2>/dev/null || true; rm -f "$f"; fi
  dd if="$tmp" of="$f" bs=4096 conv=notrunc status=none 2>/dev/null || { rm -f "$tmp"; return 1; }
  rm -f "$tmp"
}
clear_oneshot() {
  local f
  f=$(var_path LoaderEntryOneShot "$BLI_GUID")
  [[ -e $f ]] || return 0
  chattr -i "$f" 2>/dev/null || true
  rm -f "$f"
}

# ---- locks -----------------------------------------------------------------------------------

take_locks() {
  local l fd
  # Short waits, and never hold one lock while blocking on the next: the Limine tools wait only
  # ~10 s and then carry on without the lock, so holding one for a minute makes a race more
  # likely, not less. If a lock can't be taken quickly, back off and refuse; the operator
  # retries. The write itself is still guarded by a compare-before-rename.
  LOCK_FDS=()
  for l in $LOCKS; do
    mkdir -p "$(dirname "$l")"
    if [[ -L $l ]]; then release_locks; refuse "$l is a symbolic link; not taking the Limine lock through it."; fi
    exec {fd}<>"$l" || { release_locks; refuse "could not open the Limine lock $l."; }
    if ! flock -w 5 "$fd"; then
      release_locks
      refuse "$l is held by another tool (limine-snapper-sync?); try again in a moment."
    fi
    LOCK_FDS+=("$fd")
  done
}
LOCK_FDS=()
release_locks() {
  local fd
  # Unlock each held lock; the fds themselves close when the script exits. (Closing them here with
  # exec {fd}>&- is fragile, and keeping them open until exit is harmless.)
  for fd in ${LOCK_FDS[@]+"${LOCK_FDS[@]}"}; do flock -u "$fd" 2>/dev/null || true; done
  LOCK_FDS=()
  return 0
}

# ---- limine.conf -----------------------------------------------------------------------------

# limine.conf without this script's block, and without the one blank line written before it.
# Fails (and prints nothing) if a block has no end marker: then the file is not edited.
conf_without_block() {
  awk -v b="$BEGIN_MARK" -v e="$END_MARK" '
    { line[NR] = $0 }
    END {
      for (i = 1; i <= NR; i++) {
        if (line[i] == b) { inb = 1; if (out > 0 && kept[out] == "") out--; continue }
        if (inb) { if (line[i] == e) inb = 0; continue }
        kept[++out] = line[i]
      }
      if (inb) exit 3
      for (i = 1; i <= out; i++) print kept[i]
    }' "$CONF"
}

# limine.conf without our marked block AND without any stray top-level /air-gpu-oneshot entry
# (its lines, up to the next top-level entry or a blank line), for --disarm. Fails if a block
# has no end marker.
strip_all_entries() {
  awk -v b="$BEGIN_MARK" -v e="$END_MARK" -v entry="/$ENTRY" '
    { line[NR] = $0 }
    END {
      for (i = 1; i <= NR; i++) {
        if (line[i] == b) { inb = 1; if (out > 0 && kept[out] == "") out--; continue }
        if (inb) { if (line[i] == e) inb = 0; continue }
        if (line[i] == entry) { ins = 1; if (out > 0 && kept[out] == "") out--; continue }
        if (ins) { if (line[i] == "" || line[i] ~ /^\//) ins = 0; else continue }
        if (ins) continue
        kept[++out] = line[i]
      }
      if (inb) exit 3
      for (i = 1; i <= out; i++) print kept[i]
    }' "$CONF"
}

# Replaces limine.conf with $1 (plus a final newline), atomically, and syncs it. $2 is the bytes
# the edit was based on: just before the rename, the file on disk must still be those bytes, or a
# Limine tool rewrote it since and the edit is abandoned, so a stale UKI pin can't be written.
write_conf() {
  local tmp=$CONF.air-gpu.tmp
  if [[ -n ${2:-} && $(cat "$CONF") != "$2" ]]; then return 2; fi
  printf '%s\n' "$1" >"$tmp" || { rm -f "$tmp"; return 1; }
  sync "$tmp" && mv -f "$tmp" "$CONF" && sync "$(dirname "$CONF")"
}

block() {
  printf '%s\n/%s\n    comment: M3 Air GPU experiment, one boot (%s)\n    protocol: efi\n    path: %s\n    cmdline: %s\n%s\n' \
    "$BEGIN_MARK" "$ENTRY" "$1" "$2" "$3" "$END_MARK"
}
conf_has_block() { grep -qxF "$BEGIN_MARK" "$CONF" 2>/dev/null; }
# Any top-level /air-gpu-oneshot entry, inside a marked block or not.
conf_entry_count() { grep -cxF "/$ENTRY" "$CONF" 2>/dev/null || true; }
# A /air-gpu-oneshot entry left in the file with no marked block around it (a tool dropped the
# markers, or a hand edit): conf_without_block can't remove it, so neither --disarm nor a re-arm
# would be clean.
conf_has_stray_entry() {
  local n b
  n=$(conf_entry_count)
  b=0
  conf_has_block && b=1
  (( n > b ))
}

# Whether Limine's own EFI binary has a config hash enrolled: the 128 bytes after the
# ++CONFIG_B2SUM_SIGNATURE++ marker are all '0' in the unenrolled state. An enrolled binary
# rejects any edited limine.conf on every boot, so arming would brick the Mac until the ESP is
# fixed from outside. On any SoC this version of limine-mkinitcpio-hook enrolls only on x86_64,
# so the /etc/default/limine setting can be off while the binary is enrolled.
limine_config_enrolled() {
  local loader=$ESP/EFI/BOOT/BOOTAA64.EFI off sig
  [[ -f $loader ]] || return 2
  off=$({ grep -aob -- '++CONFIG_B2SUM_SIGNATURE++' "$loader" 2>/dev/null || true; } | head -1 | cut -d: -f1)
  [[ -n $off ]] || return 1   # no marker: an old Limine with no config-hash feature
  sig=$(dd if="$loader" bs=1 skip=$((off + 26)) count=128 status=none 2>/dev/null | tr -d '\0')
  [[ $sig != *[!0]* && ${#sig} -eq 128 ]] && return 1   # all zeros: unenrolled
  return 0   # a non-zero hash, or an unreadable one: treat as enrolled
}

# ---- the checks every arming makes -----------------------------------------------------------

KEYS="" UKI="" SRC_PATH="" SRC_CMDLINE="" NEW_CMDLINE="" ID=""
WARNINGS=()
preflight() {
  local protocol pin sum w p kver
  WARNINGS=()
  find_esp || refuse "no FAT EFI partition mounted at /boot/efi, /efi or /boot."
  grep -qaF limine.conf "$ESP/EFI/BOOT/BOOTAA64.EFI" 2>/dev/null ||
    refuse "the EFI loader ($ESP/EFI/BOOT/BOOTAA64.EFI) is not Limine; this script arms through Limine only."
  find_conf || refuse "Limine is the EFI loader but there is no limine.conf on $ESP."
  if grep -Eq '^[[:space:]]*ENABLE_ENROLL_LIMINE_CONFIG=["'"'"']?yes' "$LIMINE_DEFAULTS" 2>/dev/null; then
    refuse "$LIMINE_DEFAULTS has ENABLE_ENROLL_LIMINE_CONFIG=yes: Limine checks an enrolled hash of limine.conf, so adding an entry would stop this Mac booting."
  fi
  if limine_config_enrolled; then
    refuse "$ESP/EFI/BOOT/BOOTAA64.EFI has a config hash enrolled (or an unreadable one): Limine would reject an edited limine.conf and this Mac would not boot. Reset it (limine enroll-config --reset) before arming."
  fi
  if grep -Eiq '^[[:space:]]*remember_last_entry:[[:space:]]*yes' "$CONF"; then
    refuse "$CONF has remember_last_entry: yes, so Limine would boot the armed entry again after it."
  fi
  if conf_has_stray_entry; then
    refuse "$CONF has a /$ENTRY entry with no air-gpu-oneshot markers around it (a Limine tool or a hand edit dropped them). Remove it by hand, then arm again."
  fi
  KEYS=$(entry_keys)
  [[ $(key count) == 1 ]] ||
    refuse "$CONF has $(key count) //$SOURCE_ENTRY entries; this script needs exactly one to copy."
  protocol=$(key protocol) SRC_PATH=$(key path) SRC_CMDLINE=$(key cmdline) kver=$(key kver)
  [[ $protocol == efi && -n $SRC_PATH ]] ||
    refuse "the //$SOURCE_ENTRY entry is not an efi entry with a path (protocol: ${protocol:-none})."
  [[ -n $SRC_CMDLINE && $SRC_CMDLINE == *root=* ]] ||
    refuse "the //$SOURCE_ENTRY entry has no cmdline: with root= to copy."
  [[ $SRC_CMDLINE != *'"'* && $SRC_CMDLINE != *"'"* ]] ||
    refuse "the //$SOURCE_ENTRY command line has quotes, which this script does not rewrite."
  # The kernel reads - and _ alike in a parameter name, and a bare asahi.t8122_start means =1.
  [[ ! " $SRC_CMDLINE " =~ \ asahi\.t8122[_-]start([=\ ]) ]] ||
    refuse "the normal //$SOURCE_ENTRY entry already sets asahi.t8122_start, so every boot would start the GPU. Remove it from /etc/default/limine first."
  UKI=$(limine_file "$SRC_PATH")
  [[ -f $UKI ]] || refuse "the //$SOURCE_ENTRY entry's UKI $UKI does not exist."
  if [[ $SRC_PATH == *#* ]]; then
    pin=${SRC_PATH#*#}
    sum=$(b2sum "$UKI" | cut -d' ' -f1)
    [[ $sum == "$pin" ]] || refuse "$UKI measures BLAKE2b $sum, but its entry pins $pin."
  fi
  if [[ -n $kver && $kver != "$KREL" ]]; then
    refuse "the //$SOURCE_ENTRY entry boots $kver, but this boot runs $KREL. Reboot into the installed kernel first."
  fi
  grep -qaF "$KREL" "$UKI" || refuse "$UKI is not the kernel this boot runs ($KREL). Reboot into the installed kernel first."
  for p in "${PARAMS[@]}"; do
    p=${p%%=*}
    if ! grep -qaF "$p" "$UKI"; then
      if is_m3pro && ((ALLOW_M3PRO)); then
        WARNINGS+=("this kernel has no $p parameter; the test boot ignores it")
      else
        refuse "the kernel the armed entry boots ($KREL) has no $p parameter."
      fi
    fi
  done
  [[ -e $(var_path LoaderInfo "$BLI_GUID") ]] ||
    refuse "no Boot Loader Interface variables at $EFIVARS (LoaderInfo); the one-shot needs them."
  if [[ $(var_text LoaderInfo "$BLI_GUID" utf16 2>/dev/null) != Limine* ]]; then
    refuse "this boot's loader did not report itself as Limine (LoaderInfo), so LoaderEntryOneShot may not be honoured."
  fi
  if [[ $(tail -c 1 "$(var_path SecureBoot "$GLOBAL_GUID")" 2>/dev/null | od -An -tu1 | tr -d ' ') == 1 ]]; then
    refuse "Secure Boot is on, so the UKI would ignore the armed command line."
  fi
  find_var_file || refuse "U-Boot names an unusable variable file in RTStorageVolatile."
  if [[ -n $VAR_FILE ]]; then
    [[ -e $(var_path VarToFile "$UBOOT_GUID") ]] ||
      refuse "U-Boot keeps EFI variables in RAM, but VarToFile is missing, so the one-shot could not be saved."
    [[ $(uboot_store | head -c 15 | tail -c 7) == UbEfiVa ]] ||
      refuse "U-Boot's VarToFile does not hold a variable file."
    [[ -f $VAR_FILE ]] || refuse "U-Boot's variable file $VAR_FILE does not exist."
  fi
  w=$(armed_name)
  [[ -z $w || $w == "$ENTRY" ]] ||
    refuse "another one-shot boot is armed ($w): not replacing it."
  gpu_node_note
  esp_note
}

# The boot loader names the EFI partition it booted from; arming a different one changes nothing
# at the next boot (harmless, but the boot is then not armed).
esp_note() {
  local want have
  want=$(tr -d '\0' <"$DT/chosen/asahi,efi-system-partition" 2>/dev/null || true)
  [[ -n $want ]] || return 0
  have=$(findmnt -no PARTUUID "$ESP" 2>/dev/null || true)
  [[ -n $have ]] || return 0
  if [[ ${want,,} != "${have,,}" ]]; then
    WARNINGS+=("$ESP (PARTUUID $have) is not the EFI partition this boot came from ($want); arming it would not arm the next boot")
  fi
}

# The GPU only starts when the boot loader hands it over; say so when this boot's did not.
gpu_node_note() {
  local d status
  for d in "$DT"/soc/gpu@*; do
    [[ -e $d/compatible ]] || continue
    grep -qaE 'apple,agx-t(8122|6030)' "$d/compatible" || continue
    status=$(tr -d '\0' <"$d/status" 2>/dev/null || echo okay)
    if [[ $status != okay ]]; then
      WARNINGS+=("this boot's GPU node is $status (the boot loader did not hand the GPU over); unless the next boot's does, the armed boot ends ARMED-NOT-STARTED")
    fi
    return 0
  done
  WARNINGS+=("this boot's device tree has no M3 GPU node")
}

new_cmdline() {
  local w out=() p name skip words
  # No glob expansion against the current directory: a * on the normal command line stays literal.
  set -f
  read -ra words <<<"$SRC_CMDLINE"
  set +f
  for w in "${words[@]}"; do
    skip=0
    [[ $w == asahi.t8122[_-]* || $w == "$TAG"=* || $w == panic=* ]] && skip=1
    for p in "${PARAMS[@]}"; do
      name=${p%%=*}
      [[ $w == "$name" || $w == "$name"=* ]] && skip=1
    done
    ((skip)) || out+=("$w")
  done
  # panic=10 so a panic in the armed boot reboots itself into the normal entry (the one-shot is
  # already consumed), instead of waiting for a power-button hold.
  printf '%s\n' "${out[*]} ${PARAMS[*]} panic=10 $TAG=$ID"
}

# ---- actions ---------------------------------------------------------------------------------

do_arm() {
  local before w rest current rc
  is_root || refuse "run as root (sudo)."
  check_board
  # Every refusal comes from this first pass, before anything is touched. The second pass reads
  # limine.conf again under the Limine tools' locks, so none of them rewrites it in between.
  preflight
  take_locks
  preflight
  ID=$(date +%m%d-%H%M%S)
  NEW_CMDLINE=$(new_cmdline)
  mkdir -p "$STATE_DIR" || { release_locks; refuse "could not create $STATE_DIR."; }
  before=$STATE_DIR/limine.conf.before-$ID
  cp -p "$CONF" "$before" || { release_locks; refuse "could not keep a copy of $CONF in $STATE_DIR."; }
  current=$(cat "$CONF") || { release_locks; refuse "could not read $CONF."; }
  rest=$(conf_without_block) || { release_locks; refuse "$CONF has an air-gpu-oneshot block without its end marker; fix it by hand."; }
  # write_conf returns 2 if limine.conf changed under us since this read (nothing was written), 1
  # if the write failed. Its status is caught here, so set -e can't end the run without a line.
  rc=0
  write_conf "$rest"$'\n\n'"$(block "$ID" "$SRC_PATH" "$NEW_CMDLINE")" "$current" || rc=$?
  case $rc in
    0) ;;
    2) release_locks; refuse "$CONF changed while arming (a Limine tool rewrote it); nothing was armed. Try again." ;;
    *)
      # The rename may have happened before a failed directory sync: put the file back.
      if [[ $(cat "$CONF" 2>/dev/null) != "$current" ]]; then cp -p "$before" "$CONF" 2>/dev/null || true; sync; fi
      release_locks
      refuse "could not write $CONF; nothing was armed."
      ;;
  esac
  if ! grep -qxF "/$ENTRY" "$CONF" || ! grep -qxF "    cmdline: $NEW_CMDLINE" "$CONF"; then
    cp -p "$before" "$CONF"; sync; release_locks
    refuse "$CONF did not read back with the armed entry; it was put back."
  fi
  if ! set_oneshot || [[ $(armed_name) != "$ENTRY" ]] || ! persist ||
    { [[ -n $VAR_FILE ]] && ! file_has_name "$VAR_FILE"; }; then
    clear_oneshot; persist || true
    write_conf "$rest" || cp -p "$before" "$CONF" || true
    sync; release_locks
    refuse "the one-shot variable did not read back from the firmware store; it was cleared and the entry removed."
  fi
  # The arming record, which air-gpu-collect.sh judges by, is synced with everything else.
  if ! { printf '%s\n' "$ID $KREL $NEW_CMDLINE" >"$STATE_DIR/armed.tmp" && sync "$STATE_DIR/armed.tmp" &&
    mv -f "$STATE_DIR/armed.tmp" "$STATE_DIR/armed"; }; then
    say "warning: could not write the arming record $STATE_DIR/armed; the boot is armed, but air-gpu-collect.sh will need --boot to find it"
  fi
  sync
  release_locks
  for w in "${WARNINGS[@]}"; do say "warning: $w"; done
  say "armed one boot ($ID): the next boot only runs $KREL with ${PARAMS[*]}. Limine clears the one-shot before that boot starts, so the boots after it should be the normal entry; this has been tested on an M3 Pro only, so on a first arming check it with --status in the armed boot. Don't update packages before the reboot. Reboot when ready, then run: sudo air-gpu-collect.sh"
  log "armed one boot ($ID): $KREL ${PARAMS[*]}"
}

do_check() {
  local w
  is_root || refuse "run as root (sudo)."
  check_board
  preflight
  ID=check
  NEW_CMDLINE=$(new_cmdline)
  for w in "${WARNINGS[@]}"; do say "warning: $w"; done
  say "check passed: this Mac can arm one boot of $KREL from $CONF (entry //$SOURCE_ENTRY, variable file ${VAR_FILE:-none}) with: ${PARAMS[*]}. Nothing was changed."
}

# Whether a boot ran with the recorded arming's id on its command line: this boot, or one in the
# journal since the record was written.
arming_booted() {
  local id
  read -r id _ <"$STATE_DIR/armed" 2>/dev/null || return 1
  [[ -n $id ]] || return 1
  [[ " $(cat "$CMDLINE" 2>/dev/null) " == *" $TAG=$id "* ]] && return 0
  journalctl _TRANSPORT=kernel --since "@$(stat -c %Y "$STATE_DIR/armed")" -o cat --no-pager 2>/dev/null |
    grep -q "Kernel command line:.* $TAG=$id\( \|$\)"
}

do_disarm() {
  local armed rest current rc changed=""
  is_root || refuse "run as root (sudo)."
  find_esp || refuse "no FAT EFI partition mounted at /boot/efi, /efi or /boot."
  find_var_file || refuse "U-Boot names an unusable variable file in RTStorageVolatile."
  take_locks
  armed=$(armed_name)
  # Persist whenever the variable file still names the entry, even if the RAM variable is already
  # clear: a first --disarm may have cleared the variable but failed to save, and a second
  # run must finish the save rather than skip it because armed_name is now empty.
  if [[ $armed == "$ENTRY" ]] || { [[ -n $VAR_FILE ]] && file_has_name "$VAR_FILE"; }; then
    clear_oneshot
    if ! persist; then release_locks; refuse "could not save the cleared one-shot to $VAR_FILE; run --disarm again."; fi
    if [[ -n $VAR_FILE ]] && file_has_name "$VAR_FILE"; then
      release_locks; refuse "$VAR_FILE still names $ENTRY after saving; run --disarm again."
    fi
    changed+="cleared the one-shot; "
  elif [[ -n $armed ]]; then
    say "another one-shot is armed ($armed); leaving it alone"
  fi
  # Remove a marked block, and also a markerless /air-gpu-oneshot entry a tool left behind.
  if find_conf && { conf_has_block || conf_has_stray_entry; }; then
    current=$(cat "$CONF") || { release_locks; refuse "could not read $CONF."; }
    rest=$(strip_all_entries) || { release_locks; refuse "$CONF has an air-gpu-oneshot block without its end marker; fix it by hand."; }
    rc=0
    write_conf "$rest" "$current" || rc=$?
    case $rc in
      0) changed+="removed the air-gpu-oneshot entry from $CONF; " ;;
      2) release_locks; refuse "$CONF changed while disarming (a Limine tool rewrote it); run --disarm again." ;;
      *) release_locks; refuse "could not write $CONF; run --disarm again." ;;
    esac
  fi
  # The arming record stays once its boot ran, so air-gpu-collect.sh keeps judging that boot by its
  # id whenever it runs. Only an arming whose boot never ran (no boot's command line carries its
  # id) is marked cancelled. A one-shot still in the variable file after its boot ran (U-Boot
  # did not save Limine's deletion) does not count as never booted.
  if [[ $changed == *"cleared the one-shot"* && -f $STATE_DIR/armed ]] && ! arming_booted; then
    mv -f "$STATE_DIR/armed" "$STATE_DIR/armed.cancelled" || true
  fi
  sync
  release_locks
  say "disarmed: ${changed:-nothing was armed.}"
  log "disarmed: ${changed:-nothing was armed}"
}

do_status() {
  local cmd id a
  cmd=$(cat "$CMDLINE" 2>/dev/null || true)
  id=$(tr ' ' '\n' <<<"$cmd" | sed -n "s/^${TAG//./\\.}=//p" | head -1)
  if [[ -n $id ]]; then
    say "this boot is armed boot $id: $(tr ' ' '\n' <<<"$cmd" | grep '^asahi\.' | tr '\n' ' ')"
  else
    say "this boot is a normal boot (no $TAG= on the command line)"
  fi
  if [[ -d $EFIVARS ]]; then
    a=$(armed_name)
    say "one-shot variable: ${a:-not set}"
  fi
  if find_esp; then
    find_var_file || true
    if [[ -n $VAR_FILE && -f $VAR_FILE ]]; then
      if file_has_name "$VAR_FILE"; then say "$VAR_FILE: names $ENTRY (the next boot is armed)"
      else say "$VAR_FILE: does not name $ENTRY"; fi
    fi
    if find_conf; then
      if conf_has_block; then say "$CONF: has the $ENTRY entry ($(grep -m1 '    comment: M3 Air GPU experiment' "$CONF" | sed 's/.*(\(.*\))/\1/'))"
      else say "$CONF: no $ENTRY entry"; fi
    fi
  fi
}

# A safety net: any command that fails outside a checked path still ends the run with one line.
on_error() {
  printf '%s: error: unexpected failure at line %s (%s); run air-gpu-oneshot.sh --status to see what is armed.\n' \
    "$PROG" "$1" "$2" >&2
  log "error: unexpected failure at line $1 ($2)"
}

main() {
  local action=arm args=() a
  set -E
  trap 'on_error "$LINENO" "$BASH_COMMAND"' ERR
  for a in "$@"; do
    case $a in
      --allow-test-on-m3pro) ALLOW_M3PRO=1 ;;
      --check) action=check QUIET_LOG=1 ;;
      --status) action=status QUIET_LOG=1 ;;
      --disarm) action=disarm ;;
      --list-knobs) action=list ;;
      -h | --help) sed -n '2,11p' "$0"; exit 0 ;;
      -*) refuse "unknown option $a." ;;
      *) args+=("$a") ;;
    esac
  done
  case $action in
    list) list_knobs ;;
    status) do_status ;;
    disarm) ((${#args[@]} == 0)) || refuse "--disarm takes no knobs."; do_disarm ;;
    check) parse_knobs "${args[@]}"; do_check ;;
    arm)
      ((${#args[@]} > 0)) || refuse "say what to arm: start, or knob=value ... (--help)."
      parse_knobs "${args[@]}"
      do_arm
      ;;
  esac
}

if [[ ${AIR_GPU_SOURCE_ONLY:-} == 1 ]]; then return 0; fi
main "$@"
