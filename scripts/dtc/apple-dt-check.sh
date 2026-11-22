#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
# apple-dt-check.sh BASE HEAD WORKDIR: run CHECK_DTBS on every Apple device tree at BASE and at HEAD
# and fail when HEAD has a warning BASE does not. Also runs dt_binding_check on every binding HEAD
# changes. Phandle values in the messages are masked, so renumbering is not a new warning, and so
# is the pattern list of a "does not match any of the regexes" message, which grows with every
# vendor prefix the bindings add. A firmware-profile variant (a .dts that includes another board's
# .dts, such as t8122-j613-25g83) is reported under its base board's name, so the base's existing
# warnings are not new warnings of the variant; anything the variant adds still is.
# Needs dtschema (dt-validate), clang/lld (LLVM=1), flex, bison. Prints a summary.
set -u
BASE=$1 HEAD=$2 W=$3
export ARCH=arm64 LLVM=1
mkdir -p "$W"
warnings() { # rev tag
	local t=$W/src-$2 o=$W/out-$2
	git worktree add -q --detach "$t" "$1" || return 1
	make -s -C "$t" O="$o" defconfig >/dev/null || return 1
	local dtbs; dtbs=$(cd "$t/arch/arm64/boot/dts" && ls apple/*.dts | sed 's/dts$/dtb/')
	make -C "$t" O="$o" -j"$(nproc)" CHECK_DTBS=y $dtbs > "$W/$2.log" 2>&1
	local rc=$?
	local variants; variants=$(cd "$t/arch/arm64/boot/dts" &&
		grep -l '^#include "[^"]*\.dts"' apple/*.dts 2>/dev/null | while read -r v; do
			sed -nE 's#^\#include "([^"]*)\.dts".*#s|^'"${v%.dts}"'\\.dtb: |apple/\1.dtb: |;#p' "$v" | head -1
		done)
	grep -E '\.dtb: |[Ww]arning|rror' "$W/$2.log" | grep -v '^  ' |
		sed -E "s#$t/##g; s#$o/##g" |
		sed -E "s#arch/arm64/boot/dts/##; $variants" | sed -E 's#^apple/#arch/arm64/boot/dts/apple/#' |
		perl -pe 's{(\[\[[^:]*?\]\])}{(my $x = $1) =~ s/\d+/N/g; $x}ge; s{b\x27(?:\\.|[^\x27])*\x27}{b\x27N\x27}g; s{( not match any of the regexes): .*}{$1: ...}' |
		sort -u > "$W/$2.warn"
	echo "$2: make rc=$rc, $(wc -l < "$W/$2.warn") warning lines, $(echo $dtbs | wc -w) DTBs"
	return $rc
}
warnings "$BASE" base || echo "base build failed (its warnings still count)"
warnings "$HEAD" head; hrc=$?
comm -13 "$W/base.warn" "$W/head.warn" > "$W/new.warn"
comm -23 "$W/base.warn" "$W/head.warn" > "$W/gone.warn"
echo "new warnings: $(wc -l < "$W/new.warn"), removed: $(wc -l < "$W/gone.warn")"
fail=0
[ $hrc = 0 ] || { echo "HEAD device trees do not build"; fail=1; }
if [ -s "$W/new.warn" ]; then echo "--- new CHECK_DTBS warnings"; cat "$W/new.warn"; fail=1; fi
b=$(git diff --name-only --diff-filter=AM "$BASE" "$HEAD" -- 'Documentation/devicetree/bindings/*.yaml')
for f in $b; do
	if make -s -C "$W/src-head" O="$W/out-head" dt_binding_check DT_SCHEMA_FILES="${f#Documentation/devicetree/bindings/}" > "$W/binding.log" 2>&1 &&
	   ! grep -qE 'warning|error' "$W/binding.log"; then
		echo "binding ok: $f"
	else
		echo "--- binding check failed: $f"; cat "$W/binding.log"; fail=1
	fi
done
git worktree remove --force "$W/src-base" 2>/dev/null; true
exit $fail
