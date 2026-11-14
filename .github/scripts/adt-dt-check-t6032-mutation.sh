#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only OR MIT
# Mutation test for adt-dt-check-t6032.py section 6: switch one ANE engine and one external display
# controller on in a copy of the DTB; the checker must exit 1 and name both nodes.
set -u
dtb=$1
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cp "$dtb" "$tmp/mut.dtb"
fdtput -t s "$tmp/mut.dtb" /soc@200000000/ane@3c9400000 status okay || exit 2
fdtput -t s "$tmp/mut.dtb" /soc@200000000/dcp@306c00000 status okay || exit 2
out=$(python3 "$(dirname "$0")/adt-dt-check-t6032.py" --fetch-ipsw "$tmp/mut.dtb")
rc=$?
echo "$out" | grep -E "FAIL" | head
[ "$rc" -eq 1 ] || { echo "mutation not detected (exit $rc)"; exit 1; }
echo "$out" | grep -E "FAIL.*ane@3c9400000" >/dev/null || { echo "ane@3c9400000 not flagged"; exit 1; }
echo "$out" | grep -E "FAIL.*dcp@306c00000" >/dev/null || { echo "dcp@306c00000 not flagged"; exit 1; }
echo "mutation detected"
