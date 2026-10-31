#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only OR MIT
set -eu
avd_test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
avd_test_out=${1:?usage: run-host.sh OUTPUT_BINARY}
${CC:-cc} -std=gnu11 -Wall -Wextra -Werror -O2 \
	-I"$avd_test_dir/include" "$avd_test_dir/queue-host.c" -o "$avd_test_out"
"$avd_test_out"
