#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""The T6050 combo PHY adds DisplayPort pin D beside a running USB 3 port in place
(combo_dp2_join_usb()) because its USB 3 start is its pin D start: check that the
generated tables say so, for both orientations, and that the USB 3 shutdown is the
pin D power-off. A regenerated table that differs fails here."""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
USB = ROOT / 'drivers/phy/apple/usb-t6050/apple_t6050_usb_phy.h'
MF = ROOT / 'drivers/phy/apple/atc-t6050-typec-data.h'
COMBO = ROOT / 'drivers/phy/apple/atc-t6050-combo.h'
PAIRS = (('apple_t6050_usb3_init_normal', 'mf_normal_init'),
         ('apple_t6050_usb3_init_reverse', 'mf_reverse_init'),
         ('apple_t6050_usb3_shutdown', 'mf_poweroff'))
ROW = re.compile(r'\{\s*(T6050_\w+|ATC_\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)'
                 r'\s*,\s*(\w+)(?:\s*,\s*("[^"]*"|0))?\s*\}')


def table(text, name):
    m = re.search(r'\b' + re.escape(name) + r'\[\]\s*=\s*\{(.*?)\n\};', text, re.S)
    assert m, name
    return [(r[0].split('_', 1)[1], *(int(x, 0) for x in r[1:6]), (r[6] or '').strip('"'))
            for r in ROW.findall(m.group(1))]


def tunable_index():
    """combo_tunable()'s name -> (bank, index into the PHY's tunables) for the common banks."""
    body = re.search(r'static int combo_tunable\(.*?\n\}', COMBO.read_text(), re.S).group(0)
    common = re.search(r'common\[\] = \{(.*?)\};', body, re.S).group(1)
    return {n: (int(b), int(i)) for b, i, n in
            re.findall(r'\{\s*(\d+)\s*,\s*(\d+)\s*,\s*"([^"]+)"\s*\}', common)}


def step(s, names):
    """One step as the PHY runs it: the power op is the wrapper domain on both paths."""
    op, bank, off, mask, value, timeout, name = s
    if op == 'POWER':
        return ('POWER', value)
    if op == 'TUNABLE':
        return ('TUNABLE', *names[name]) if name else ('TUNABLE', bank, value)
    return (op, bank, off, mask, value, timeout)


def differences(usb, mf, names):
    if len(usb) != len(mf):
        return ['%d steps against %d' % (len(usb), len(mf))]
    return ['step %d: %s against %s' % (i, step(a, names), step(b, names))
            for i, (a, b) in enumerate(zip(usb, mf)) if step(a, names) != step(b, names)]


def main():
    names = tunable_index()
    usb_text, mf_text = USB.read_text(), MF.read_text()
    failures = 0
    for usb, mf in PAIRS:
        a, b = table(usb_text, usb), table(mf_text, mf)
        diff = differences(a, b, names)
        print('%s %s = %s (%d steps)' % ('FAIL' if diff else 'ok', usb, mf, len(a)))
        for d in diff[:5]:
            print('  ' + d)
        failures += bool(diff)
        # The check itself must see a one-step change.
        bad = list(a)
        i = next(n for n, s in enumerate(bad) if s[0] == 'MASK')
        bad[i] = bad[i][:4] + (bad[i][4] ^ 1,) + bad[i][5:]
        if not differences(bad, b, names):
            print('FAIL: a changed %s step was not seen' % usb)
            failures += 1
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
