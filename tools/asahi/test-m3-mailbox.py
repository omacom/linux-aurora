#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise the production T8122 mailbox predicate with captured DTB properties.

Usage: python3 tools/asahi/test-m3-mailbox.py [runtime.dtb daily.dtb]
Requires dtc's fdtget. No target access or MMIO.
"""
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
board = (ROOT / 'drivers/gpu/drm/asahi/m3/m3_board.rs').read_text()
soc = (ROOT / 'drivers/gpu/drm/asahi/m3/m3_soc.rs').read_text()
t8122 = soc[soc.index('pub(crate) static T8122:'):]
compatibles = re.search(r'mailbox_compatibles: &(\[.*?\]),', t8122, re.S)[1]
irqs = re.search(r'mailbox_interrupts: &(\[.*?\]),', t8122, re.S)[1]
base = re.search(r'mailbox: (0x[0-9a-fA-F_]+),', t8122)[1]
regfn = re.search(r'fn expected_mailbox_reg\(.*?\n}', board, re.S)[0]
predicate = board[board.index('    if !soc.mailbox_compatibles'):]
predicate = predicate[:predicate.index('\n    {')].strip()[3:]


def prop(dtb, node, name, kind):
    result = subprocess.check_output(['fdtget', '-t', kind, str(dtb), node, name], text=True)
    return [int(v, 16) for v in result.split()]


def vec(values):
    return 'vec![' + ','.join(map(str, values)) + ']'


fixtures = []
for argument in sys.argv[1:]:
    dtb = Path(argument)
    gpu = '/soc/gpu@292400000'
    # Both retained captures use this mailbox path; verify the phandle link,
    # not its capture-specific numeric value.
    mbox = '/soc/mbox@292408000'
    if prop(dtb, gpu, 'mboxes', 'x') != prop(dtb, mbox, 'phandle', 'x'):
        raise RuntimeError('GPU mailbox phandle mismatch')
    values = [prop(dtb, mbox, name, kind) for name, kind in
              [('compatible', 'bx'), ('reg', 'x'), ('interrupt-names', 'bx'),
               ('interrupts', 'x'), ('#mbox-cells', 'x')]]
    if len(values[4]) != 1:
        raise RuntimeError('bad #mbox-cells length')
    fixtures.append('check(' + ','.join(vec(v) for v in values[:4]) + ',' + str(values[4][0]) + ');')
    print(dtb.name, hashlib.sha256(dtb.read_bytes()).hexdigest())
if not sys.argv[1:]:
    # Public t8122-j613-dcp-25g83.dtsi and t8122-gpu.dtsi resource contract.
    names = list(b'send-empty\0send-not-empty\0recv-empty\0recv-not-empty\0')
    for compat in (b'apple,t8122-agx-asc-mailbox\0apple,asc-mailbox-v4\0',
                   b'apple,t8122-asc-mailbox\0apple,asc-mailbox-v4\0'):
        values = [list(compat), [2, 0x92408000, 0, 0x4000], names,
                  [0,723,4,0,724,4,0,725,4,0,726,4]]
        fixtures.append('check(' + ','.join(vec(v) for v in values) + ',0);')
if len(fixtures) != 2:
    raise SystemExit('supply runtime.dtb and daily.dtb')

source = '''
struct Windows { mailbox: u64 }
struct Soc { mailbox_compatibles: &'static [&'static [u8]], mailbox_interrupts: &'static [u32], windows: Windows }
''' + regfn + '''
fn accepts(soc: &Soc, compat: &Vec<u8>, mbox_reg: &Vec<u32>, irq_names: &Vec<u8>, irqs: &Vec<u32>, cells: u32) -> bool {
    !(''' + predicate + ''')
}
fn check(compat: Vec<u8>, reg: Vec<u32>, names: Vec<u8>, irqs: Vec<u32>, cells: u32) {
    let soc = Soc { mailbox_compatibles: &''' + compatibles + ''', mailbox_interrupts: &''' + irqs + ''', windows: Windows { mailbox: ''' + base + ''' } };
    let ok = |c: &Vec<u8>, r: &Vec<u32>, n: &Vec<u8>, i: &Vec<u32>, x| accepts(&soc,c,r,n,i,x);
    assert!(ok(&compat,&reg,&names,&irqs,cells));
    for i in 0..reg.len() { let mut v=reg.clone(); v[i]^=1; assert!(!ok(&compat,&v,&names,&irqs,cells)); }
    for i in 0..irqs.len() { let mut v=irqs.clone(); v[i]^=1; assert!(!ok(&compat,&reg,&names,&v,cells)); }
    assert!(!ok(&compat,&reg,&names,&irqs,1));
    let mut v=names.clone(); v[0]^=1; assert!(!ok(&compat,&reg,&v,&irqs,cells));
    for v in [b"apple,asc-mailbox-v4\\0".to_vec(), b"apple,t8122-agx-asc-mailbox\\0unknown\\0".to_vec(), b"apple,t6031-agx-asc-mailbox\\0".to_vec()] { assert!(!ok(&v,&reg,&names,&irqs,cells)); }
    // Compatible lists are ordered: reversing either supported pair must fail.
    for pair in soc.mailbox_compatibles {
        let parts: Vec<_> = pair.split(|byte| *byte == 0).filter(|part| !part.is_empty()).collect();
        if parts.len() == 2 {
            let mut reversed = parts[1].to_vec();
            reversed.push(0);
            reversed.extend_from_slice(parts[0]);
            reversed.push(0);
            assert!(!ok(&reversed,&reg,&names,&irqs,cells));
        }
    }
    for v in soc.mailbox_compatibles { assert!(ok(&v.to_vec(),&reg,&names,&irqs,cells)); }
}
fn main() {
''' + '\n'.join(fixtures) + '\nprintln!("mailbox: both captures, all exact supported lists and resource/identity mutation controls PASS");\n}\n'
with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp)
    (path / 'test.rs').write_text(source)
    subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', str(path / 'test.rs'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)
print('m3_board.rs', hashlib.sha256(board.encode()).hexdigest())
print('m3_soc.rs', hashlib.sha256(soc.encode()).hexdigest())
