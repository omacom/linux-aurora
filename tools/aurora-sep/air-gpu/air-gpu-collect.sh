#!/bin/bash
# air-gpu-collect.sh: after a boot, collect what the M3 Air GPU experiment needs into one tgz in
# your home directory, and print one verdict line.
#
#   sudo air-gpu-collect.sh             the armed boot: this one, or the one before it when this
#                                       boot is a normal one (the armed boot hung, say), read back
#                                       from the journal
#   sudo air-gpu-collect.sh --boot 0    this boot
#   sudo air-gpu-collect.sh --boot -1   the boot before this one
#
# Read-only: apart from the tgz (and a temporary directory) it writes nothing. The kernel log in
# the tgz has lines naming a serial number dropped and MAC addresses masked.
#
# The last line printed is the verdict, one of:
#   NOT-ARMED            the analysed boot did not have aurora.t8122_start=1 (or asahi.t8122_start=1) (no one-shot boot)
#   ARMED-NO-LOG         the last arming's boot left no kernel log (most likely a hang in the GPU
#                        start, before the journal was written); no older boot's result is shown
#   ARMED-NOT-STARTED    armed, but the driver stopped before it started the firmware
#   FW-BOOT-FAILED       the firmware was started but did not come up
#   DRIVER-REFUSED       the firmware came up, but the driver did not publish the InitData
#   PUBLISH-FAILED       the InitData publish message could not be sent to the firmware
#   DEVICE-CONTROL-FAILED a device-control message was not acknowledged after the publish (the
#                        firmware's control channel, not the InitData contents)
#   INITDATA-REJECTED    the firmware did not acknowledge the published InitData, or crashed on it
#   FW-RUNNING           the firmware accepted the InitData and runs; no GPU job ran
#   FW-CHECK-FAILED      the firmware accepted the InitData, then a post-boot control or check
#                        failed
#   FW-CRASHED           the firmware ran, then crashed or faulted
#   CAP-VIOLATED         after a job the firmware reported a performance state above the cap
#   JOB-NOT-DISPATCHED   a GPU job was never dispatched: the kernel read its GPU start timestamp as 0
#   JOB-FAILED           a GPU job failed: dispatched or not unknown, or dispatched and not completed
#                        (stuck, killed, timed out, faulted, wrong results)
#   JOB-NO-RESULT        a GPU job started but no result was recorded (the boot ended first, or
#                        it is still running)
#   JOB-COMPLETED        air-gpu-job.sh ran, and every submission completed with correct results
#   UNKNOWN              a kernel verdict this script does not recognise
#
# The line also names the source that decided it: the command line, the arming record, the
# kernel's verdict line, the kernel log's structure, or the job evidence. Job evidence comes from
# two sources read the same way in every pass: the air-gpu-job record files and the journal's
# air-gpu-job lines for that boot. A record left at result=running whose journal line has the
# result is judged by the journal, so the same boot gets the same verdict before and after a
# hard reset.
set -euo pipefail

# ---- the verdict table: the kernel log lines each step is judged by --------------------------
# Keep this in step with the kernel's T8122 start experiment (drivers/gpu/drm/asahi/t8122_start.rs).
#
# 1. The kernel's own verdict lines, on an armed T8122 ("M3 G15G verdict: <label>"), and the
#    verdict each label gives. The match is on the label up to the first ':' or '('. Each label
#    has a rank, and the highest-ranked label present decides; among equal ranks the later line
#    wins. So a cap violation outranks everything (it is the fanless safety signal), a job
#    outcome outranks a firmware one, and any failure outranks job-completed and
#    firmware-running.
#    JOB-NOT-DISPATCHED is kept for the two labels the kernel logs only when the job's GPU start
#    timestamp is 0 (an unreadable one, "timestamp None", gives JOB-FAILED); a job the kernel saw
#    dispatched (job-ran-completion-missed) is never JOB-NOT-DISPATCHED. A
#    firmware-running-check-failed line that says the firmware crashed gives FW-CRASHED.
#   label                           verdict                rank
KERNEL_VERDICTS='
firmware-running                FW-RUNNING             10
job-completed                   JOB-COMPLETED          20
firmware-boot-failed            FW-BOOT-FAILED         30
driver-refused                  DRIVER-REFUSED         30
publish-failed                  PUBLISH-FAILED         30
device-control-failed           DEVICE-CONTROL-FAILED  30
initdata-rejected               INITDATA-REJECTED      30
firmware-running-check-failed   FW-CHECK-FAILED        40
job-accepted-never-dispatched   JOB-NOT-DISPATCHED     50
job-timed-out-powered           JOB-NOT-DISPATCHED     50
job-timed-out                   JOB-FAILED             50
job-ran-completion-missed       JOB-FAILED             50
job-faulted                     JOB-FAILED             50
job-retired-without-timestamps  JOB-FAILED             50
job-failed-before-wait          JOB-FAILED             50
cap-violated                    CAP-VIOLATED           60
'
RE_KERNEL_VERDICT='M3 G15G verdict: [a-z-]+'
# The start experiment's own lines: the values it armed with, and its refusals.
RE_KERNEL_ARMED='M3 G15G start: armed'
#
# 2. Without those lines (a kernel before them, or a test run on an M3 Pro), these extended regexes
#    decide, in this order: armed? -> firmware started? -> InitData sent? -> accepted? -> job?
# The boot was armed: on its command line.
RE_ARMED='(^| )(asahi|aurora)\.t8122_start=1( |$)'
# The driver refused or stopped before it started the firmware (the last such line is the reason).
RE_REFUSED='M3 G15G start: (refused|not armed)|GPU startup disabled|identity or ABI rejected|startup refused|no GPU backend|invalid for parameter .(asahi|aurora)\.|InitData self-check failed'
# The probe ended in an error (the reason when nothing more specific was logged).
RE_PROBE_FAILED='probe with driver (asahi|aurora) failed'
# The driver probed the GPU at all (without a boot-loader handoff the GPU node is disabled).
RE_PROBED='asahi [0-9a-f]+\.gpu: Probing'
# The GPU coprocessor's firmware was started.
RE_FW_STARTED='asahi [0-9a-f]+\.gpu: RTKit: Initializing|M3: publishing owned initdata'
# The firmware crashed, reported an error, or the GPU faulted: any of these overrides a
# firmware-running or job-completed verdict, since the firmware did not run cleanly.
RE_FW_CRASH='\.gpu: RTKit: co-processor has crashed|M3 G15[A-Z]*: firmware crashed|firmware crashed while waiting for init|GPU firmware crashed|M3 firmware error event|M3 bank [0-9]+ fault=0x[0-9a-f]*[1-9a-f]|M3 fault render|M3 fault mapping'
# The InitData was handed to the firmware, and the firmware accepted it.
RE_INITDATA_SENT='M3: publishing owned initdata'
RE_INITDATA_OK='M3: firmware accepted owned initdata'
# The firmware's readiness report (the reason when the InitData was not accepted).
RE_READINESS='M3 firmware readiness:|M3 firmware error event'
# The kernel saw a GPU job fail. "GPU is powered down" at a failed job is the not-dispatched
# signature seen on an M3 Pro with the power words at 0; the other lines say the job failed without saying whether it was dispatched.
RE_JOB_NOT_DISPATCHED='GPU is powered down'
RE_JOB_FAILED='M3 scheduler: execution failed|M3 completion events=|GPU is powered down|M3 fault render|M3 bank [0-9]+ fault=0x[0-9a-f]*[1-9a-f]|M3 fault mapping'
#
# The GPU lines kept in gpu-log.txt, and the firmware's own log lines in firmware-log.txt.
RE_GPU_LINES='asahi|aurora|agx|G15|M3[ _:]|t8122|gpu|RTKit|devcoredump|macsmc'
RE_FW_LOG='\.gpu: RTKit: (syslog|crashlog|co-processor|Unknown)|firmware crashed|crashlog|M3 firmware|M3 G15G (verdict|start)'
# SMC power keys that read zero at idle on an M3 Air (J613) sample (2026-10-06): the GPU rail
# candidates. One of them should rise the first time the GPU draws power.
GPU_RAIL_CANDIDATES="PP1b PP2l PP3b PP9l PPdb PR6b PRab PRcb PReb PZC0 PZC1 PZCB PZCU PZD1"
# ----------------------------------------------------------------------------------------------

PROG=air-gpu-collect
TAG=air_gpu.oneshot
DT=/proc/device-tree
EFIVARS=/sys/firmware/efi/efivars
BLI_GUID=4a67b082-0a4c-41cf-b6c7-440b29bb8c4f
DEBUGFS=/sys/kernel/debug
SYS=/sys
CMDLINE=/proc/cmdline
BOOT_ID=/proc/sys/kernel/random/boot_id
STATE_DIR=/var/lib/air-gpu

is_root() { ((EUID == 0)); }
# Run as the invoking user when root under sudo, else directly: the tgz is written this way, so a
# symlink in the user's home can't redirect a root write or chown.
runu() {
  if [[ $(id -u) == 0 && -n ${SUDO_USER:-} && $SUDO_USER != root ]]; then
    runuser -u "$SUDO_USER" -- "$@"
  else
    "$@"
  fi
}
# The tgz goes to this user's home (the invoking user's under sudo).
user_home() {
  local h
  h=$(getent passwd "$1" | cut -d: -f6)
  echo "${h:-/root}"
}
say() { printf '%s: %s\n' "$PROG" "$*"; }

# Kernel log text without serial numbers and MAC addresses.
scrub() {
  { grep -aviE 'serialnumber|serial number|serial-number' || true; } |
    sed -E 's/([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}/xx:xx:xx:xx:xx:xx/g'
}

cmdline_of() { # boot: 0 or -1
  if [[ $1 == 0 ]]; then cat "$CMDLINE"; return; fi
  { journalctl -k -b "$1" -o cat --no-pager 2>/dev/null || true; } | sed -n 's/^.*Kernel command line: //p' | head -1
}
boot_id_of() {
  if [[ $1 == 0 ]]; then tr -d '-' <"$BOOT_ID"; return; fi
  { journalctl --list-boots --no-pager 2>/dev/null || true; } | awk -v i="$1" '$1 == i { print $2; exit }'
}
oneshot_of() { tr ' ' '\n' <<<"$1" | sed -n "s/^${TAG//./\\.}=//p" | head -1; }
knobs_of() { tr ' ' '\n' <<<"$1" | grep '^asahi\.' | tr '\n' ' ' | sed 's/ $//'; }

# The kernel log of a boot, oldest first: this boot's from the journal when the journal has its
# start, otherwise dmesg; an earlier boot's from the journal.
# The journal is preferred for every boot (the same source in every pass); dmesg is the fallback
# only for this boot when the journal lacks its start. KLOG_SRC names the source used.
KLOG_SRC=""
kernel_log() { # boot index, output file
  local j
  KLOG_SRC=journal
  if [[ $1 == 0 ]]; then
    j=$(journalctl -k -b 0 -o short-monotonic --no-pager 2>/dev/null || true)
    if grep -qaE 'Kernel command line:|Linux version' <<<"$j"; then printf '%s\n' "$j" >"$2"; return 0; fi
    KLOG_SRC=dmesg
    { dmesg 2>/dev/null || true; } >"$2"
    return 0
  fi
  { journalctl -k -b "$1" -o short-monotonic --no-pager 2>/dev/null || true; } >"$2"
}

# ---- job evidence ----------------------------------------------------------------------------
# Two sources, read the same way in every pass, reconciled per job (keyed by the record path):
#   - the record files whose boot_id is the analysed boot's. Under sudo only the invoking user's
#     home is read; from a root shell, /root and every home under EXTRA_HOMES. Each record is read
#     as the owner of the home it is in, into root-only space, so a symlink or a swapped file there
#     can only reach what that user could read anyway; every later check works on that copy.
#   - the journal's air-gpu-job lines for that boot, from those same users (and root) only:
#     "start: ... record <path>" and "result: <result> (...) stage <stage> record <path>". An
#     older result line without "record" is paired with the earliest start that has no result yet.
EXTRA_HOMES="/root /home/*"
JOB_HOME=""
JOB_COPIES=""
# The homes searched for records, as "owner<TAB>home" lines (the owner of the directory itself).
job_homes() {
  local h o
  if [[ -n ${SUDO_USER:-} && $SUDO_USER != root && -n $JOB_HOME ]]; then
    printf '%s\t%s\n' "$SUDO_USER" "$JOB_HOME"
    return 0
  fi
  # shellcheck disable=SC2086 # EXTRA_HOMES is a list of globs
  for h in "$JOB_HOME" $EXTRA_HOMES; do
    [[ -n $h && -d $h && ! -L $h ]] || continue
    o=$(stat -c %U -- "$h" 2>/dev/null) || continue
    printf '%s\t%s\n' "$o" "$h"
  done | awk -F'\t' '!seen[$2]++'
  return 0
}
# Reads one record as its home's owner (root's own home: as root), at most 64 KiB.
read_record_as() { # owner file
  if [[ $(id -u) == 0 && $1 != root ]]; then
    runuser -u "$1" -- head -c 65536 -- "$2" 2>/dev/null
  else
    head -c 65536 -- "$2" 2>/dev/null
  fi
}
# The boot's record files, as "path<TAB>copy" lines: each copied (read as its owner) into
# JOB_COPIES, and kept only when the copy carries the boot's id.
job_record_files() { # boot_id
  local bid=$1 owner h f c n=0
  [[ -n $bid && -n $JOB_COPIES ]] || return 0
  while IFS=$'\t' read -r owner h; do
    [[ -d $h/air-gpu-runs && ! -L $h/air-gpu-runs ]] || continue
    for f in "$h"/air-gpu-runs/job-*.txt; do
      [[ -e $f ]] || continue
      n=$((n + 1))
      c=$JOB_COPIES/$n.txt
      read_record_as "$owner" "$f" >"$c" || { rm -f "$c"; continue; }
      if grep -qx "boot_id=$bid" "$c"; then printf '%s\t%s\n' "$f" "$c"; else rm -f "$c"; fi
    done
  done < <(job_homes)
  return 0
}
# The users whose air-gpu-job journal lines are trusted: root and the owners of those homes.
job_uids() {
  local owner
  echo 0
  while IFS=$'\t' read -r owner _; do id -u -- "$owner" 2>/dev/null || true; done < <(job_homes)
}
journal_job_lines() { # boot id (or index)
  local -a match=()
  local u
  while read -r u; do [[ -n $u ]] && match+=("_UID=$u"); done < <(job_uids | sort -u)
  journalctl -b "$1" -t air-gpu-job ${match[@]+"${match[@]}"} -o cat --no-pager 2>/dev/null || true
}
rec_key() { [[ -f $1 ]] || return 0; sed -n "s/^$2=//p" "$1" | head -1; }

# Prints one line per job of the boot: "<result><TAB><source><TAB><record path><TAB><copy>" (the
# copy is the record as read, empty for a job known only from the journal).
#   result: the record's when it holds a final one (not "running"); else the journal's result for
#           that record; else "running" (the job started and no result was recorded). When both
#           hold a final result they must agree, or the result is "conflict".
#   source: record, journal, record+journal, or "journal (record left at running)".
job_evidence() { # boot_id boot_index [records list from job_record_files]
  local bid=$1 idx=$2 line path copy r rr jj res src p
  local -A rec_r=() rec_c=() jr_r=() known=()
  local -a order=() pending=() np=()
  while IFS= read -r line; do
    case $line in
      "start: "*" record "*)
        path=${line##* record }
        if [[ -z ${known[$path]:-} ]]; then known[$path]=1; order+=("$path"); fi
        pending+=("$path")
        ;;
      "result: "*)
        r=${line#result: }
        r=${r%% *}
        if [[ $line == *" record "* ]]; then path=${line##* record }
        elif ((${#pending[@]})); then path=${pending[0]}
        else continue; fi
        jr_r[$path]=$r
        if [[ -z ${known[$path]:-} ]]; then known[$path]=1; order+=("$path"); fi
        np=()
        for p in ${pending[@]+"${pending[@]}"}; do [[ $p == "$path" ]] || np+=("$p"); done
        pending=(${np[@]+"${np[@]}"})
        ;;
    esac
  done < <(journal_job_lines "${bid:-$idx}")
  while IFS=$'\t' read -r path copy; do
    rec_r[$path]=$(rec_key "$copy" result)
    rec_c[$path]=$copy
    if [[ -z ${known[$path]:-} ]]; then known[$path]=1; order+=("$path"); fi
  done < <(if [[ -n ${3:-} ]]; then cat -- "$3"; else job_record_files "$bid"; fi)
  for path in ${order[@]+"${order[@]}"}; do
    rr=${rec_r[$path]:-} jj=${jr_r[$path]:-}
    if [[ -n $rr && $rr != running ]]; then
      res=$rr src=record
      if [[ -n $jj ]]; then
        src=record+journal
        [[ $jj == "$rr" ]] || res=conflict
      fi
    elif [[ -n $jj ]]; then
      res=$jj src=journal
      [[ -z $rr ]] || src="journal (record left at running)"
    else
      res=running
      if [[ -n $rr ]]; then src=record; else src=journal; fi
    fi
    printf '%s\t%s\t%s\t%s\n' "$res" "$src" "$path" "${rec_c[$path]:-}"
  done
  return 0
}

# ---- the verdict -----------------------------------------------------------------------------

VERDICT="" REASON="" VERDICT_SRC=""
# The jobs of the analysed boot, folded into one state so the verdict never rests on one record
# Set by job_aggregate from job_evidence's lines, read by classify.
#   JOB_STATE: fail (>=1 failed) | no-result (>=1 started, no result) | pass (>=1 passed) |
#              not-reached | none, in that order of precedence
#   JOB_DETAIL: a short reason; JOB_SRC: the source of the deciding job(s)
JOB_STATE=none JOB_DETAIL="" JOB_SRC=""

# One job's result: the result alone decides. Only no-render-node and no-device mean the job
# never reached the GPU; running means no result was recorded; anything else (stuck, killed,
# device-lost, fence-timeout, stalled, wrong-result, error, conflict) is a failure.
job_outcome() { # result -> pass | fail | no-result | not-reached
  case $1 in
    pass) echo pass ;;
    no-render-node | no-device) echo not-reached ;;
    running) echo no-result ;;
    *) echo fail ;;
  esac
}

job_aggregate() { # evidence file ("result<TAB>source<TAB>path" lines)
  local r src path o n=0 npass=0 nfail=0 nnores=0 nreach=0 failp="" fails="" failr="" passp="" passs=""
  local noresp="" noress="" reachr="" first device stage err copy name
  JOB_STATE=none JOB_DETAIL="" JOB_SRC=""
  while IFS=$'\t' read -r r src path copy; do
    [[ -n $r ]] || continue
    # Details come from the copy read as the owner, never from the path again.
    name=${path##*/}
    path=${copy:-$path}
    n=$((n + 1))
    o=$(job_outcome "$r")
    case $o in
      pass) npass=$((npass + 1)); passp=$path passs=$src ;;
      fail) nfail=$((nfail + 1)); if [[ -z $failp ]]; then failp=$path fails=$src failr=$r; fi ;;
      no-result) nnores=$((nnores + 1)); if [[ -z $noresp ]]; then noresp=$name noress=$src; fi ;;
      not-reached) nreach=$((nreach + 1)); reachr=$r ;;
    esac
  done <"${1:-/dev/null}"
  if ((nfail > 0)); then
    JOB_STATE=fail JOB_SRC=$fails
    err=$(rec_key "$failp" error) stage=$(rec_key "$failp" stage)
    JOB_DETAIL="air-gpu-job: $failr${err:+: $err}${stage:+ (stage $stage)}; $npass of $n runs passed"
  elif ((nnores > 0)); then
    JOB_STATE=no-result JOB_SRC=$noress
    JOB_DETAIL="air-gpu-job started ($noresp) but no result was recorded: the boot ended first, or the job is still running; $npass of $n runs passed"
  elif ((npass > 0)); then
    JOB_STATE=pass JOB_SRC=$passs
    first=$(rec_key "$passp" first_submit_ms) device=$(rec_key "$passp" device)
    JOB_DETAIL="air-gpu-job: $npass of $n runs completed with correct results${first:+ (first in $first ms)}${device:+ on $device}"
  elif ((nreach > 0)); then
    JOB_STATE=not-reached JOB_SRC=record
    JOB_DETAIL="air-gpu-job: the job did not reach the GPU ($reachr)"
  fi
  return 0
}

# Sets VERDICT/REASON/VERDICT_SRC from the job state, for a boot whose firmware accepted the
# InitData. Returns 1 when there is no job evidence (the caller keeps its own verdict).
job_verdict() {
  case $JOB_STATE in
    fail) VERDICT=JOB-FAILED REASON=$JOB_DETAIL VERDICT_SRC="job $JOB_SRC" ;;
    no-result) VERDICT=JOB-NO-RESULT REASON=$JOB_DETAIL VERDICT_SRC="job $JOB_SRC" ;;
    pass) VERDICT=JOB-COMPLETED REASON=$JOB_DETAIL VERDICT_SRC="job $JOB_SRC" ;;
    *) return 1 ;;
  esac
}

# classify LOG CMDLINE  (JOB_STATE set by job_aggregate beforehand). It decides the verdict, then
# applies the crash override to both the kernel-line and the structural paths.
classify() { classify_core "$@"; crash_override "$1"; }
classify_core() {
  local log=$1 cmd=$2 accept_ln line
  first_ln() { { grep -naE "$1" "$log" || true; } | head -1 | cut -d: -f1; }
  last_line() { { grep -aE "$1" "$log" || true; } | tail -1 | sed -E 's/^\[[^]]*\] *//; s/^[^ ]+ kernel: //'; }
  if ! [[ " $cmd " =~ $RE_ARMED ]]; then
    VERDICT=NOT-ARMED VERDICT_SRC=cmdline
    REASON="neither aurora.t8122_start=1 nor asahi.t8122_start=1 is on the analysed boot's command line"
    return 0
  fi
  if grep -qaE "$RE_KERNEL_VERDICT" "$log"; then
    classify_kernel "$log"
    return 0
  fi
  VERDICT_SRC=kernel-log
  if ! grep -qaE "$RE_FW_STARTED" "$log"; then
    VERDICT=ARMED-NOT-STARTED
    if line=$(last_line "$RE_REFUSED") && [[ -n $line ]]; then REASON=$line
    elif line=$(last_line "$RE_PROBE_FAILED") && [[ -n $line ]]; then REASON=$line
    elif ! grep -qaE "$RE_PROBED" "$log"; then
      REASON="the GPU driver never probed: the GPU node was disabled (the boot loader did not hand the GPU over)"
    else REASON="the driver stopped before it started the GPU firmware"; fi
    return 0
  fi
  if ! grep -qaE "$RE_INITDATA_SENT" "$log"; then
    VERDICT=FW-BOOT-FAILED
    if line=$(last_line "$RE_FW_CRASH") && [[ -n $line ]]; then REASON=$line
    elif line=$(last_line "$RE_PROBE_FAILED") && [[ -n $line ]]; then REASON="the firmware did not come up: $line"
    else REASON="the firmware was started but never took the InitData"; fi
    return 0
  fi
  accept_ln=$(first_ln "$RE_INITDATA_OK")
  if [[ -z $accept_ln ]]; then
    VERDICT=INITDATA-REJECTED
    if line=$(last_line "$RE_FW_CRASH") && [[ -n $line ]]; then REASON="the firmware crashed after taking the InitData: $line"
    elif line=$(last_line "$RE_READINESS") && [[ -n $line ]]; then REASON=$line
    else REASON="the InitData was sent but the firmware never accepted it"; fi
    return 0
  fi
  # The firmware accepted the InitData. The kernel's not-dispatched signature, a failed job, a
  # kernel job-failure line, a job with no result, a passed job, in that order.
  if line=$(last_line "$RE_JOB_NOT_DISPATCHED") && [[ -n $line ]]; then
    VERDICT=JOB-NOT-DISPATCHED REASON="$line${JOB_DETAIL:+; $JOB_DETAIL}"
    return 0
  fi
  if [[ $JOB_STATE == fail ]]; then
    job_verdict
    if line=$(last_line "$RE_JOB_FAILED") && [[ -n $line ]]; then REASON="$line; $REASON"; fi
    return 0
  fi
  if line=$(last_line "$RE_JOB_FAILED") && [[ -n $line ]]; then
    VERDICT=JOB-FAILED REASON="$line${JOB_DETAIL:+; $JOB_DETAIL}"
    return 0
  fi
  job_verdict && return 0
  VERDICT=FW-RUNNING
  if [[ $JOB_STATE == not-reached ]]; then REASON="the firmware accepted the InitData and runs; $JOB_DETAIL"
  else REASON="the firmware accepted the InitData and runs; no GPU job ran in this boot"; fi
}

# The kernel's own verdict lines decide. The highest-ranked label present wins; among equal
# ranks the later line wins. The job evidence then gates job-completed and firmware-running: the
# kernel's line alone never gives JOB-COMPLETED. A job-failure line from the kernel log without a
# verdict label (another client's job, say) also keeps a run from FW-RUNNING or JOB-COMPLETED.
classify_kernel() { # LOG
  local log=$1 o v rank best="" bestrank=-1 bestv="" unknown="" line
  kernel_line() { { grep -aE "M3 G15G verdict: $1" "$log" || true; } | tail -1 | sed -E 's/^.*(M3 G15G verdict: )/\1/'; }
  plain_line() { { grep -aE "$1" "$log" || true; } | tail -1 | sed -E 's/^\[[^]]*\] *//; s/^[^ ]+ kernel: //'; }
  VERDICT_SRC=kernel-verdict
  while IFS= read -r o; do
    v="" rank=""
    # An unknown label gives no awk output, so read hits EOF; || true keeps set -e from aborting.
    read -r v rank < <(printf '%s\n' "$KERNEL_VERDICTS" | awk -v o="$o" '$1 == o { print $2, $3 }') || true
    if [[ -z $v ]]; then [[ " $unknown " == *" $o "* ]] || unknown+="${unknown:+ }$o"; continue; fi
    if ((rank >= bestrank)); then bestrank=$rank best=$o bestv=$v; fi
  done < <({ grep -aoE "$RE_KERNEL_VERDICT" "$log" || true; } | sed 's/^M3 G15G verdict: //')
  if [[ -z $bestv ]]; then
    VERDICT=UNKNOWN REASON="a kernel verdict this script does not recognise: $({ grep -aoE "$RE_KERNEL_VERDICT.*" "$log" || true; } | tail -1)"
    return 0
  fi
  case $bestv in
    JOB-COMPLETED | FW-RUNNING)
      if line=$(plain_line "$RE_JOB_NOT_DISPATCHED") && [[ -n $line ]]; then
        VERDICT=JOB-NOT-DISPATCHED REASON="$line${JOB_DETAIL:+; $JOB_DETAIL}" VERDICT_SRC=kernel-log
      elif line=$(plain_line "$RE_JOB_FAILED") && [[ -n $line ]]; then
        VERDICT=JOB-FAILED REASON="$line${JOB_DETAIL:+; $JOB_DETAIL}" VERDICT_SRC=kernel-log
      elif [[ $bestv == JOB-COMPLETED && $JOB_STATE == pass ]]; then
        job_verdict
        REASON="$(kernel_line job-completed); $JOB_DETAIL" VERDICT_SRC="kernel-verdict + job $JOB_SRC"
      elif job_verdict; then
        [[ $bestv != JOB-COMPLETED ]] || REASON="$JOB_DETAIL (the kernel logged: $(kernel_line job-completed))"
      elif [[ $bestv == JOB-COMPLETED ]]; then
        VERDICT=FW-RUNNING
        REASON="a job retired (kernel: $(kernel_line job-completed)), but no air-gpu-job record checked its results on the CPU${JOB_DETAIL:+; $JOB_DETAIL}"
      else
        VERDICT=FW-RUNNING REASON="$(kernel_line firmware-running)${JOB_DETAIL:+; $JOB_DETAIL}"
      fi
      ;;
    JOB-NOT-DISPATCHED)
      VERDICT=JOB-NOT-DISPATCHED REASON=$(kernel_line "$best")
      # The not-dispatched labels need a GPU start timestamp of 0; an unreadable one proves nothing.
      [[ $REASON != *"timestamp None"* ]] || VERDICT=JOB-FAILED
      REASON+="${JOB_DETAIL:+; $JOB_DETAIL}"
      ;;
    FW-CHECK-FAILED)
      VERDICT=FW-CHECK-FAILED REASON=$(kernel_line "$best")
      # Its crash variant says so: the firmware crashed during the post-boot control or check.
      [[ $REASON != *crashed* ]] || VERDICT=FW-CRASHED
      ;;
    *)
      VERDICT=$bestv REASON=$(kernel_line "$best")
      if [[ $bestv == JOB-FAILED && -n $JOB_DETAIL ]]; then REASON+="; $JOB_DETAIL"; fi
      ;;
  esac
  [[ -z $unknown ]] || REASON+="; also a kernel verdict this script does not recognise: $unknown"
  return 0
}

# A crash, fault or RTKit-crash line still overrides a firmware-running, job-completed or
# job-no-result verdict: the firmware did not run cleanly.
crash_override() { # LOG
  local log=$1 line
  case $VERDICT in FW-RUNNING | JOB-COMPLETED | JOB-NO-RESULT) ;; *) return 0 ;; esac
  grep -qaE "$RE_FW_CRASH" "$log" || return 0
  line=$({ grep -aE "$RE_FW_CRASH" "$log" || true; } | tail -1 | sed -E 's/^\[[^]]*\] *//; s/^[^ ]+ kernel: //')
  case $VERDICT in
    JOB-COMPLETED) REASON="a job completed, then the firmware crashed or faulted: $line (earlier: ${REASON})" ;;
    JOB-NO-RESULT) REASON="the firmware crashed or faulted while a job had no result: $line (${REASON})" ;;
    *) REASON="the firmware ran, then crashed or faulted: $line" ;;
  esac
  VERDICT=FW-CRASHED VERDICT_SRC="kernel-log (crash line)"
}

# ---- privacy ---------------------------------------------------------------------------------

# The host names to mask: this boot's (uname), the static and transient ones, and the FQDN, each
# with its short form; longest first. Generic names (localhost, the distribution's default) and
# names under 3 characters are left alone: masking them would garble unrelated text. Only names
# made of letters, digits, '.' and '-' are used, so none needs escaping beyond the '.'.
host_names() {
  local n
  {
    uname -n 2>/dev/null
    cat /etc/hostname 2>/dev/null
    hostnamectl hostname 2>/dev/null
    hostnamectl --static 2>/dev/null
    hostname -f 2>/dev/null
  } | tr -d '[:blank:]\r' | while IFS= read -r n; do
    [[ -n $n ]] || continue
    echo "$n"
    echo "${n%%.*}"
  done | grep -E '^[A-Za-z0-9][A-Za-z0-9.-]{2,}$' | grep -vixE 'localhost|localhost\.localdomain|archlinux|omarchy|localdomain' |
    awk '{ print length, $0 }' | sort -rn | cut -d' ' -f2- | awk '!seen[$0]++'
  return 0
}

# Masks host and user names in every file under DIR, as the last step before the tgz is built:
#   - every occurrence of a name from host_names becomes "host" (in a binary dump, the same
#     number of 'x' bytes, so its layout is kept);
#   - "Hostname set to <...>" (systemd's line) becomes "Hostname set to <host>", whatever the name
#     was in the boot the line comes from;
#   - in the journal-format files (the kernel log and the two files grep makes from it), the host
#     column after the timestamp becomes "host" on every line (an earlier boot may have had
#     another name);
#   - home directories become /home/USER/.
mask_tree() { # DIR [journal-format files in it...]
  local dir=$1 f n esc
  shift
  local -a text=() bin=()
  while IFS= read -r n; do
    esc=${n//./\\.}
    text+=(-e "s/$esc/host/g")
    bin+=(-e "s/$esc/$(printf '%*s' "${#n}" '' | tr ' ' x)/g")
  done < <(host_names)
  text+=(-e 's/Hostname set to <[^>]*>/Hostname set to <host>/g' -e 's#/home/[^/[:space:]]+/#/home/USER/#g')
  while IFS= read -r -d '' f; do
    if [[ $f == *.bin ]]; then
      ((${#bin[@]} == 0)) || sed -i -E "${bin[@]}" -- "$f"
    else
      sed -i -E "${text[@]}" -- "$f"
    fi
  done < <(find "$dir" -type f -print0)
  for f in "$@"; do
    [[ -f $f ]] && sed -i -E 's/^(\[ *[0-9.]+\]) [^ ]+ /\1 host /' -- "$f"
  done
  return 0
}

# ---- collection ------------------------------------------------------------------------------

smc_keys_file() { find "$DEBUGFS" -maxdepth 3 -name keys -path '*smc*' 2>/dev/null | head -1; }

# The LoaderEntryOneShot variable's value now, empty when unset (Limine clears it as it boots).
oneshot_var() {
  local f=$EFIVARS/LoaderEntryOneShot-$BLI_GUID
  [[ -e $f ]] || return 0
  tail -c +5 "$f" | iconv -f UTF-16LE -t UTF-8 2>/dev/null | tr -d '\0'
}
# The boot index (0, -1, -2 ...) whose command line carries air_gpu.oneshot=<id>, or empty.
boot_with_oneshot() {
  local want=$1 idx c
  for idx in 0 $({ journalctl --list-boots --no-pager 2>/dev/null || true; } | awk '$1 < 0 { print $1 }'); do
    c=$(cmdline_of "$idx")
    [[ " $c " == *" ${TAG}=$want "* ]] && { echo "$idx"; return 0; }
  done
  return 1
}

main() {
  local boot="" explicit=0 dumps=0 armed_id="" sel cmd0 b cmd id home user dir out log recs f board kf n live n_tg rails
  # Removed on any exit (a global, so the EXIT trap still sees it).
  WORK=""
  trap 'rm -rf "${WORK:-}"' EXIT
  while (($#)); do
    case $1 in
      --boot) boot=${2:-}; explicit=1; shift ;;
      --include-dumps) dumps=1 ;;
      -h | --help) sed -n '2,24p' "$0"; exit 0 ;;
      *) say "refused: unknown argument $1 (--boot N or --include-dumps)."; exit 2 ;;
    esac
    shift
  done
  [[ -z $boot || $boot =~ ^-?[0-9]+$ ]] || { say "refused: --boot takes a boot index (0, -1, ...)."; exit 2; }
  is_root || { say "refused: run as root (sudo): the SMC keys and the kernel log need it."; exit 2; }

  cmd0=$(cat "$CMDLINE")
  # Judge the last arming by its id, so a hung armed boot never shows an older boot's result.
  # An empty or unreadable record (a crash right after arming) counts as none.
  if [[ -f $STATE_DIR/armed ]]; then read -r armed_id _ <"$STATE_DIR/armed" || armed_id=""; fi
  if ((! explicit)); then
    if [[ -n $armed_id ]]; then
      sel=$(boot_with_oneshot "$armed_id") || sel=""
      if [[ -n $sel ]]; then
        boot=$sel
      elif [[ -z $(oneshot_var) ]]; then
        # The one-shot was consumed (Limine cleared it) but no boot carries the id: the armed
        # boot ran and left no journal, the signature of a hang in the GPU start.
        boot=0 VERDICT=ARMED-NO-LOG VERDICT_SRC=arming-record
        REASON="the armed boot (oneshot $armed_id) left no kernel log, most likely a hang in the GPU start before the journal was written; no older boot's result is shown"
      else
        # Still armed, not booted yet: the tester ran this before rebooting.
        boot=0
        say "the armed boot (oneshot $armed_id) has not run yet (LoaderEntryOneShot is still set). Reboot, then run this again."
      fi
    else
      boot=0
      if ! [[ " $cmd0 " =~ $RE_ARMED ]] && [[ " $(cmdline_of -1) " =~ $RE_ARMED ]]; then boot=-1; fi
    fi
  fi
  cmd=$(cmdline_of "$boot")
  b=$(boot_id_of "$boot")
  id=$(oneshot_of "$cmd")
  [[ -z $id && -n $armed_id && $VERDICT == ARMED-NO-LOG ]] && id=$armed_id
  user=${SUDO_USER:-root}
  home=$(user_home "$user")
  JOB_HOME=$home
  board=$(tr '\0' '\n' <"$DT/compatible" 2>/dev/null | sed -n 's/^apple,\(j[0-9a-z]*\)$/\1/p' | head -1)
  out=$home/air-gpu-collect-${board:-mac}-$(date +%Y%m%d-%H%M%S)
  dir=$(mktemp -d)
  WORK=$dir
  mkdir -p "$dir/c"
  local d=$dir/c

  # The analysed boot's kernel log, and this boot's dmesg.
  log=$d/kernel-log-boot$boot.txt
  kernel_log "$boot" "$dir/klog.raw"
  scrub <"$dir/klog.raw" >"$log"
  if [[ $boot != 0 ]]; then dmesg 2>/dev/null | scrub >"$d/dmesg-boot0.txt" || true; fi
  grep -aiE "$RE_GPU_LINES" "$log" >"$d/gpu-log.txt" || true
  grep -aE "$RE_FW_LOG" "$log" >"$d/firmware-log.txt" || true

  # The analysed boot's jobs, from the record files and the journal, folded into JOB_STATE.
  mkdir -p "$d/jobs"
  JOB_COPIES=$dir/records
  mkdir -m 0700 "$JOB_COPIES"
  job_record_files "$b" >"$dir/records.tsv"
  job_evidence "$b" "$boot" "$dir/records.tsv" >"$dir/evidence.tsv"
  journal_job_lines "${b:-$boot}" >"$d/jobs/journal.txt"
  # The tgz gets the copies (as read by their owner), under the records' own names.
  mapfile -t recs < <(cut -f1 "$dir/records.tsv")
  while IFS=$'\t' read -r f c; do cp -- "$c" "$d/jobs/${f##*/}"; done <"$dir/records.tsv"
  cut -f1-3 "$dir/evidence.tsv" >"$d/jobs/evidence.tsv"
  job_aggregate "$dir/evidence.tsv"

  # Judged without the command-line line, which names the knobs. ARMED-NO-LOG was already set.
  if [[ $VERDICT != ARMED-NO-LOG ]]; then
    grep -av 'Kernel command line:' "$log" >"$dir/judged.txt" || true
    classify "$dir/judged.txt" "$cmd"
  fi

  {
    echo "== analysed boot: $boot (boot id ${b:-unknown})"
    echo "cmdline: $cmd"
    echo "oneshot: ${id:-none}"
    echo "knobs: $(knobs_of "$cmd")"
    echo "last armed record: $(cat "$STATE_DIR/armed" 2>/dev/null || echo -)"
    echo "== this boot"
    # No host name (uname -a would include it): this tgz is meant for a public issue.
    uname -srvm
    echo "cmdline: $cmd0"
    echo "board: ${board:-?} compatible: $(tr '\0' ' ' <"$DT/compatible" 2>/dev/null)"
    pacman -Q linux-aurora m1n1-aurora 2>/dev/null || true
    pacman -Qq 2>/dev/null | grep -i 'mesa' | xargs -r pacman -Q 2>/dev/null || true
    printf 'm3-mode: '; cat /var/lib/aurora-sep/m3-mode 2>/dev/null || echo -
    for f in "$DT"/soc/gpu@*; do
      [[ -e $f/compatible ]] || continue
      echo "gpu node ${f##*/}: $(tr '\0' ' ' <"$f/compatible") status=$(tr -d '\0' <"$f/status" 2>/dev/null || echo okay)"
    done
    echo "chosen:"
    for f in "$DT"/chosen/asahi,t8122-* "$DT"/chosen/asahi,t6030-*; do [[ -e $f ]] && echo "  ${f##*/}"; done
  } >"$d/system.txt" 2>&1

  # The one-shot's state now: the variable, the armed record, the boot entry.
  {
    printf 'LoaderEntryOneShot: '
    if [[ -e $EFIVARS/LoaderEntryOneShot-$BLI_GUID ]]; then
      tail -c +5 "$EFIVARS/LoaderEntryOneShot-$BLI_GUID" | iconv -f UTF-16LE -t UTF-8 2>/dev/null | tr -d '\0'; echo
    else echo "not set"; fi
    printf 'LoaderEntrySelected (this boot): '
    tail -c +5 "$EFIVARS/LoaderEntrySelected-$BLI_GUID" 2>/dev/null | iconv -f UTF-16LE -t UTF-8 2>/dev/null | tr -d '\0'; echo
    printf 'last armed: '; cat "$STATE_DIR/armed" 2>/dev/null || echo -
    command -v air-gpu-oneshot.sh >/dev/null && air-gpu-oneshot.sh --status 2>&1 || true
  } >"$d/oneshot.txt" 2>&1

  # The asahi parameters: the ones sysfs shows, and the ones the analysed boot asked for.
  {
    echo "== /sys/module/asahi/parameters"
    for f in "$SYS"/module/asahi/parameters/*; do [[ -r $f ]] && echo "${f##*/}=$(cat "$f" 2>/dev/null)"; done
    echo "== asked for on the command line (boot $boot)"
    tr ' ' '\n' <<<"$cmd" | grep '^asahi\.' || echo -
  } >"$d/asahi-params.txt" 2>&1
  {
    for f in progress activity memory timing; do
      echo "== $DEBUGFS/asahi-m3/$f"
      head -c 65536 "$DEBUGFS/asahi-m3/$f" 2>&1 || true
    done
  } >"$d/debugfs-asahi-m3.txt" 2>&1
  # A devcoredump is a raw image of GPU-side memory. The default tgz is meant for a public
  # issue, so it records only each dump's presence and size, not its bytes. --include-dumps adds
  # the raw .bin for a private hand-over.
  for f in "$SYS"/class/devcoredump/devcd*; do
    [[ -e $f/data ]] || continue
    echo "${f##*/}: failing_device=$(readlink -f "$f/failing_device" 2>/dev/null) size=$(stat -c %s "$f/data" 2>/dev/null)" >>"$d/devcoredump.txt"
    if ((dumps)); then head -c 25165824 "$f/data" >"$d/${f##*/}.bin" 2>/dev/null || true; fi
  done
  if [[ -e $d/devcoredump.txt ]] && ((! dumps)); then
    echo "(raw dumps left out; run with --include-dumps for a private hand-over, not a public issue)" >>"$d/devcoredump.txt"
  fi

  # Every SMC T* and P* key, with its value (mC, mW), from this boot.
  kf=$(smc_keys_file)
  if [[ -n $kf ]]; then
    { head -1 "$kf"; awk '$2 ~ /^[TP]/' "$kf"; } >"$d/smc-keys.txt" 2>&1 || true
  else
    echo "no SMC key list in debugfs" >"$d/smc-keys.txt"
  fi

  {
    echo "== /dev/dri"; ls -l /dev/dri 2>&1 || true
    for f in "$SYS"/class/drm/renderD*; do
      [[ -e $f/device/driver ]] && echo "${f##*/}: driver $(basename "$(readlink -f "$f/device/driver")")"
    done
  } >"$d/render-node.txt" 2>&1
  if grep -q 'driver asahi' "$d/render-node.txt"; then n=present; else n=absent; fi

  n_tg=$(awk '$2 ~ /^Tg/' "$d/smc-keys.txt" | wc -l)
  live=$(awk '$2 ~ /^Tg/ && $6 + 0 >= 10000' "$d/smc-keys.txt" | wc -l)
  rails=$(for k in $GPU_RAIL_CANDIDATES; do awk -v k="$k" '$2 == k { printf "%s=%s ", k, $6 }' "$d/smc-keys.txt"; done)
  {
    echo "AIR-GPU VERDICT: $VERDICT | boot $boot | oneshot ${id:-none} | source: ${VERDICT_SRC:-?} | $REASON"
    echo "knobs: $(knobs_of "$cmd")"
    echo "kernel armed with: $({ grep -aoE "$RE_KERNEL_ARMED.*" "$log" || true; } | tail -1 | sed -n 's/^.*admitted): //p')"
    echo "render node: $n"
    echo "SMC (this boot): $live of $n_tg Tg* keys read 10 C or more; GPU rail candidates (mW): ${rails:-none found}"
    echo "jobs: $(awk 'NF' "$d/jobs/evidence.tsv" | wc -l) (record files ${#recs[@]}, state $JOB_STATE${JOB_SRC:+, from $JOB_SRC}); kernel log from the $KLOG_SRC"
  } >"$d/summary.txt"

  # No host name or user name in the tgz (it is meant for a public issue): a last pass over every
  # file staged for it.
  if [[ $KLOG_SRC == journal ]]; then
    mask_tree "$d" "$log" "$d/gpu-log.txt" "$d/firmware-log.txt"
  else
    mask_tree "$d"
  fi
  # Build the tgz in the work dir (root-owned temp), then copy it out as the user.
  local base summary
  base=$(basename "$out")
  mv "$d" "$dir/$base"
  summary=$dir/$base/summary.txt
  tar -C "$dir" -czf "$dir/out.tgz" "$base"
  # The user (runu) can't read the root-only mktemp dir, so hand the tgz over through its own
  # world-readable temp file (the tgz is meant for a public issue); the collection dir stays 0700.
  local pub
  pub=$(mktemp --tmpdir "air-gpu-collect.XXXXXX.tgz")
  cp -f "$dir/out.tgz" "$pub"
  chmod 0644 "$pub"
  # Durable before this returns (fsync the tgz and its directory): a hard reset right after must
  # not lose it.
  # shellcheck disable=SC2016 # $1..$3 are expanded by that sh
  if runu sh -c 'cp -f -- "$1" "$2" && sync -- "$2" && sync -- "$3"' _ "$pub" "$out.tgz" "$home" 2>/dev/null; then
    say "wrote $out.tgz"
  else
    # The user's home was not writable (a symlink, say): keep the temp copy, under a name
    # mktemp chose, where root can reach it.
    sync "$pub" 2>/dev/null || true
    say "wrote $pub (could not write $user's home)"
    pub=""
  fi
  [[ -z $pub ]] || rm -f "$pub"
  sed -n '2,6p' "$summary"
  head -1 "$summary"
}

if [[ ${AIR_GPU_SOURCE_ONLY:-} == 1 ]]; then return 0; fi
main "$@"
