#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only OR MIT
set -eu
# This checks compute work through file close, but does not observe GPU execution or test render or VM_BIND.

ksft_skip=4
if [ "${ASAHI_SIGKILL_RUN:-}" != 1 ]; then
    echo "SKIP: set ASAHI_SIGKILL_RUN=1 to run destructive GPU tests"
    exit "$ksft_skip"
fi

count=${ASAHI_SIGKILL_COUNT:-20}
case "$count" in
    ''|*[!0-9]*) echo "FAIL: ASAHI_SIGKILL_COUNT must be an integer" >&2; exit 1 ;;
esac
if [ "$count" -lt 20 ]; then
    echo "FAIL: the regression test requires at least 20 kills" >&2
    exit 1
fi

command -v cc >/dev/null || { echo "FAIL: cc is required" >&2; exit 1; }
command -v journalctl >/dev/null || { echo "FAIL: journalctl is required" >&2; exit 1; }
if command -v glslangValidator >/dev/null; then
    shader_compiler=glslangValidator
elif command -v glslc >/dev/null; then
    shader_compiler=glslc
else
    echo "FAIL: glslangValidator or glslc is required" >&2
    exit 1
fi

VK_DRIVER_FILES=${VK_DRIVER_FILES:-/usr/share/vulkan/icd.d/asahi_icd.json}
if [ ! -r "$VK_DRIVER_FILES" ]; then
    echo "FAIL: Vulkan driver manifest is not readable: $VK_DRIVER_FILES" >&2
    exit 1
fi
export VK_DRIVER_FILES
echo "Vulkan driver manifest: $VK_DRIVER_FILES"

root=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
tmp=$(mktemp -d "${TMPDIR:-/tmp}/asahi-sigkill.XXXXXX")
child=''
cleanup() {
    if [ -n "$child" ]; then
        kill -KILL "$child" 2>/dev/null || true
        wait "$child" 2>/dev/null || true
    fi
    rm -rf -- "$tmp"
}
trap cleanup EXIT
trap 'exit 1' HUP INT TERM

cc -O1 "$root/vkrepro.c" -lvulkan -o "$tmp/vkrepro"
shader="$tmp/comp.spv"
if [ "$shader_compiler" = glslangValidator ]; then
    glslangValidator -V -S comp "$root/comp.comp" -o "$shader"
else
    glslc -fshader-stage=compute "$root/comp.comp" -o "$shader"
fi

kernel_journal_snapshot() {
    journalctl -k -b --no-pager -o cat > "$tmp/kernel.log" || {
        echo "FAIL: cannot read current-boot kernel journal" >&2
        return 1
    }
    if [ ! -s "$tmp/kernel.log" ]; then
        echo "FAIL: current-boot kernel journal is empty" >&2
        return 1
    fi
}

gpu_fault_count() {
    kernel_journal_snapshot || return 1
    awk '/GPU fault nya|Unmapped address/ { count++ } END { print count + 0 }' "$tmp/kernel.log"
}

gpu_timeout_count() {
    kernel_journal_snapshot || return 1
    awk '/GPU timeout nya/ { count++ } END { print count + 0 }' "$tmp/kernel.log"
}

kernel_error_count() {
    kernel_journal_snapshot || return 1
    awk '
        /MMU: deferred VM close failed/ ||
        /deferred user unmap failed/ ||
        /deferred object unmap failed/ ||
        /WARNING:/ || /BUG:/ { count++ }
        END { print count + 0 }
    ' "$tmp/kernel.log"
}

before_faults=$(gpu_fault_count)
before_timeouts=$(gpu_timeout_count)
before_kernel_errors=$(kernel_error_count)
iteration=1
while [ "$iteration" -le "$count" ]; do
    log="$tmp/run.log"
    "$tmp/vkrepro" "$shader" > "$log" 2>&1 &
    child=$!
    deadline=$(( $(date +%s) + 30 ))
    while ! grep -q '^ready$' "$log" 2>/dev/null; do
        if ! kill -0 "$child" 2>/dev/null; then
            cat "$log" >&2
            echo "FAIL: repro exited before readiness at iteration $iteration" >&2
            exit 1
        fi
        if [ "$(date +%s)" -ge "$deadline" ]; then
            cat "$log" >&2
            echo "FAIL: readiness timed out at iteration $iteration" >&2
            exit 1
        fi
        sleep 0.1
    done
    kill -KILL "$child"
    wait "$child" 2>/dev/null || true
    child=
    sleep 1
    echo "SIGKILL $iteration/$count"
    iteration=$((iteration + 1))
done

sleep 3

after_faults=$(gpu_fault_count)
after_timeouts=$(gpu_timeout_count)
after_kernel_errors=$(kernel_error_count)
new_faults=$((after_faults - before_faults))
new_timeouts=$((after_timeouts - before_timeouts))
new_kernel_errors=$((after_kernel_errors - before_kernel_errors))
echo "GPU faults before=$before_faults after=$after_faults delta=$new_faults"
echo "GPU timeouts before=$before_timeouts after=$after_timeouts delta=$new_timeouts"
echo "Kernel error lines before=$before_kernel_errors after=$after_kernel_errors delta=$new_kernel_errors"
if [ "$new_faults" -ne 0 ] || [ "$new_timeouts" -ne 0 ] || [ "$new_kernel_errors" -ne 0 ]; then
    echo "FAIL: GPU faults, timeouts, or kernel errors increased" >&2
    exit 1
fi
echo "PASS: $count SIGKILL runs after two compute submissions had no GPU faults, timeouts, or kernel errors"
