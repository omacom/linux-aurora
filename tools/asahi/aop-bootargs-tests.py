#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise production bootarg patching and the per-SoC nCal choice with synthetic MMIO."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

HARNESS = r'''
#![allow(dead_code)]
use std::{cell::{Cell,RefCell},marker::PhantomData,mem,ops::{Deref,DerefMut}};
type Result<T=()> = std::result::Result<T,i32>;
const EINVAL:i32=-22; const EIO:i32=-5; const GFP_KERNEL:u32=0;
const AOP_MMIO_SIZE:usize=128; const BOOTARGS_OFFSET:usize=16; const BOOTARGS_SIZE:usize=20;
struct KVec<T>(Vec<T>);
impl<T:Clone> KVec<T>{fn from_elem(v:T,n:usize,_:u32)->Result<Self>{Ok(Self(vec![v;n]))}}
impl<T> Deref for KVec<T>{type Target=Vec<T>;fn deref(&self)->&Vec<T>{&self.0}}
impl<T> DerefMut for KVec<T>{fn deref_mut(&mut self)->&mut Vec<T>{&mut self.0}}
struct Region<const N:usize>;
struct Device{bytes:RefCell<Vec<u8>>,writes:RefCell<Vec<usize>>,fail:Cell<bool>}
struct Mmio<'a,T>{dev:&'a Device,_p:PhantomData<T>}
impl<'a,T> Mmio<'a,T>{
 fn size(&self)->usize{self.dev.bytes.borrow().len()}
 fn read32(&self,at:usize)->u32{self.try_read32(at).unwrap()}
 fn try_read32(&self,at:usize)->Result<u32>{assert_eq!(at%4,0);Ok(u32::from_le_bytes(self.dev.bytes.borrow()[at..at+4].try_into().unwrap()))}
 fn try_write32(&self,v:u32,at:usize)->Result{assert_eq!(at%4,0);self.dev.writes.borrow_mut().push(at);if !self.dev.fail.get(){self.dev.bytes.borrow_mut()[at..at+4].copy_from_slice(&v.to_le_bytes());}Ok(())}
 fn try_cast<U>(&self)->Result<Mmio<'a,U>>{Ok(Mmio{dev:self.dev,_p:PhantomData})}
}
struct IoSlice<'a>{dev:&'a Device,start:usize,end:usize}
impl IoSlice<'_>{
 fn copy_to_slice(&self,v:&mut[u8]){v.copy_from_slice(&self.dev.bytes.borrow()[self.start..self.end]);}
 fn copy_from_slice(&self,v:&[u8]){self.dev.bytes.borrow_mut()[self.start..self.end].copy_from_slice(v);}
}
macro_rules! io_project {($io:ident,[try: $start:ident .. $($end:tt)*])=>{IoSlice{dev:$io.dev,start:$start,end:$($end)*}};}
struct Aop;
impl Aop { METHOD }
fn fixture(offset:usize,table:&[u8])->Device{
 let mut bytes=vec![0xa5;AOP_MMIO_SIZE];bytes[16..20].copy_from_slice(&(offset as u32).to_le_bytes());bytes[20..24].copy_from_slice(&(table.len() as u32).to_le_bytes());
 if offset+table.len()<=bytes.len(){bytes[offset..offset+table.len()].copy_from_slice(table);}
 Device{bytes:RefCell::new(bytes),writes:RefCell::new(Vec::new()),fail:Cell::new(false)}
}
fn patch(dev:&Device,word:bool)->Result{Aop.patch_bootargs(&Mmio{dev,_p:PhantomData::<Region<AOP_MMIO_SIZE>>},&[(1,Some(0x12345678)),(2,Some(0x87654321))],word)}
fn patch_with(dev:&Device,patches:&[(u32,Option<u64>)])->Result{Aop.patch_bootargs(&Mmio{dev,_p:PhantomData::<Region<AOP_MMIO_SIZE>>},patches,true)}
fn from_fourcc(b:&[u8;4])->u32{u32::from_le_bytes(*b)}
CONFIGS
fn main(){
 let table:[u8;24]=[1,0,0,0,4,0,0,0,0,0,0,0,2,0,0,0,4,0,0,0,0,0,0,0];
 for offset in 32..36 {for word in [false,true] {
  let d=fixture(offset,&table);let mut expected=d.bytes.borrow().clone();expected[offset+8..offset+12].copy_from_slice(&0x12345678u32.to_le_bytes());expected[offset+20..offset+24].copy_from_slice(&0x87654321u32.to_le_bytes());
  assert_eq!(patch(&d,word),Ok(()));assert_eq!(*d.bytes.borrow(),expected,"adjacent bytes changed");
  assert!(d.writes.borrow().iter().all(|&x|x%4==0&&x>=offset&!3&&x+4<=(offset+table.len()+3)&!3));
  if word {assert!(!d.writes.borrow().is_empty());}else{assert!(d.writes.borrow().is_empty());}
  let writes=d.writes.borrow().len();assert_eq!(patch(&d,word),Ok(()));assert_eq!(d.writes.borrow().len(),writes,"unchanged words rewritten");
 }}
 let mut malformed=table;malformed[16..20].copy_from_slice(&9u32.to_le_bytes());
 for word in [false,true]{let d=fixture(35,&malformed);let before=d.bytes.borrow().clone();assert_eq!(patch(&d,word),Err(EINVAL));assert_eq!(*d.bytes.borrow(),before);assert!(d.writes.borrow().is_empty());}
 for tail in 1..8 {let d=fixture(35,&table[..12+tail]);assert_eq!(patch(&d,true),Err(EINVAL));assert!(d.writes.borrow().is_empty());}
 let d=fixture(35,&table);d.fail.set(true);assert_eq!(patch(&d,true),Err(EIO),"readback error ignored");
 assert_eq!(patch(&fixture(35,&[]),true),Err(EINVAL));assert_eq!(patch(&fixture(35,&[]),false),Ok(()));
 assert_eq!(patch(&fixture(120,&table),true),Err(EINVAL));
 // None keeps the firmware value of its key and still patches the others.
 let d=fixture(32,&table);let mut expected=d.bytes.borrow().clone();expected[32+8..32+12].copy_from_slice(&0x12345678u32.to_le_bytes());
 let fw=0x0badf00du32;d.bytes.borrow_mut()[32+20..32+24].copy_from_slice(&fw.to_le_bytes());expected[32+20..32+24].copy_from_slice(&fw.to_le_bytes());
 assert_eq!(patch_with(&d,&[(1,Some(0x12345678)),(2,None)]),Ok(()));assert_eq!(*d.bytes.borrow(),expected,"None overwrote the firmware value");
 // Per-SoC nCal: T8112 and T6000 keep the image value, T6040/T8132 write 1, the others write 0.
 let ncal=from_fourcc(b"nCal");
 for (name,cfg,want) in [("T8103",&HW_CFG_T8103,None),("T8112",&HW_CFG_T8112,Some(1u32)),("T6000",&HW_CFG_T6000,Some(1)),("T6020",&HW_CFG_T6020,None),("T6040",&HW_CFG_T6040,None),("T8132",&HW_CFG_T8132,None),("T8140",&HW_CFG_T8140,None)] {
  let expect=match name {"T8112"|"T6000"=>None,"T6040"|"T8132"=>Some(1u64),_=>Some(0u64)};
  assert_eq!(cfg.ncal,expect,"{} nCal choice",name);
  let mut t=[0u8;12];t[0..4].copy_from_slice(&ncal.to_le_bytes());t[4..8].copy_from_slice(&4u32.to_le_bytes());t[8..12].copy_from_slice(&1u32.to_le_bytes());
  let d=fixture(32,&t);assert_eq!(patch_with(&d,&[(ncal,cfg.ncal)]),Ok(()));
  let got=u32::from_le_bytes(d.bytes.borrow()[40..44].try_into().unwrap());
  let wanted=want.unwrap_or_else(||cfg.ncal.unwrap() as u32);assert_eq!(got,wanted,"{} nCal written",name);
 }
 // ED0p: writes 8-byte IOVA when an external data region is present, preserves unused ED0p when None,
 // and rejects missing, duplicate, or non-8-byte ED0p entries without writing MMIO.
 let ed0p=from_fourcc(b"ED0p");
 let ec0p=from_fourcc(b"EC0p");
 let ed0p_iova=0x1000_0020_000u64;
 let mut t27=[0u8;32];
 t27[0..4].copy_from_slice(&ec0p.to_le_bytes());t27[4..8].copy_from_slice(&8u32.to_le_bytes());
 t27[16..20].copy_from_slice(&ed0p.to_le_bytes());t27[20..24].copy_from_slice(&8u32.to_le_bytes());
 t27[24..32].copy_from_slice(&0xdead_beef_cafe_babeu64.to_le_bytes());
 let d=fixture(32,&t27);
 assert_eq!(patch_with(&d,&[(ec0p,Some(0x1000_0000_000)),(ed0p,Some(ed0p_iova))]),Ok(()));
 assert_eq!(u64::from_le_bytes(d.bytes.borrow()[32+24..32+32].try_into().unwrap()),ed0p_iova,"ED0p not written");
 let d=fixture(32,&t27);
 assert_eq!(patch_with(&d,&[(ec0p,Some(0x1000_0000_000)),(ed0p,None)]),Ok(()));
 assert_eq!(u64::from_le_bytes(d.bytes.borrow()[32+24..32+32].try_into().unwrap()),0xdead_beef_cafe_babeu64,"unused ED0p overwritten");
 let d=fixture(32,&t27[..16]);
 assert_eq!(patch_with(&d,&[(ec0p,Some(0x1000_0000_000)),(ed0p,Some(ed0p_iova))]),Err(EINVAL),"missing ED0p accepted");
 assert!(d.writes.borrow().is_empty());
 let mut dup=[0u8;32];
 dup[0..16].copy_from_slice(&t27[16..32]);dup[16..32].copy_from_slice(&t27[16..32]);
 let d=fixture(32,&dup);
 assert_eq!(patch_with(&d,&[(ed0p,Some(ed0p_iova))]),Err(EINVAL),"duplicate ED0p accepted");
 assert!(d.writes.borrow().is_empty());
 let mut short=t27;short[20..24].copy_from_slice(&4u32.to_le_bytes());
 let d=fixture(32,&short[..28]);
 assert_eq!(patch_with(&d,&[(ec0p,Some(0x1000_0000_000)),(ed0p,Some(ed0p_iova))]),Err(EINVAL),"4-byte ED0p accepted");
 assert!(d.writes.borrow().is_empty());
 println!("PASS aligned/unaligned byte and word access, unchanged writes, malformed tables, bounds, failed readback, None keeps the firmware value, per-SoC nCal, ED0p external data");
}
'''


def method(source):
    start = source.index('    fn patch_bootargs(')
    at = source.index('{', start)
    depth = 1
    end = at + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def configs(source):
    """struct AopHwConfig and the HW_CFG_* constants, each with its doc comment.

    Other items between them (the sleep's PM callbacks) use kernel bindings the harness
    doesn't have.
    """
    start = source.index('struct AopHwConfig {')
    end = source.index('kernel::of_device_table!', start)
    out, pending, item, keep, depth = [], [], [], None, 0
    for line in source[start:end].splitlines(keepends=True):
        if keep is None:
            if not line.strip() or line.startswith(('//', '#[')):
                pending.append(line)
                continue
            keep = line.startswith(('struct AopHwConfig', 'const HW_CFG_'))
            item, pending, depth = pending + [line], [], 0
        else:
            item.append(line)
        depth += line.count('{') - line.count('}')
        if depth == 0 and line.rstrip().endswith((';', '}')):
            if keep:
                out += item
            keep = None
    return ''.join(out + pending)


def compile_run(rustc, p, binary, code):
    p.write_text(code)
    subprocess.run([rustc, '--edition=2021', str(p), '-o', str(binary)], check=True)
    return subprocess.run([str(binary)], capture_output=True, text=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=Path(__file__).resolve().parents[2] / 'drivers/soc/apple/aop.rs')
    args = parser.parse_args()
    production = method(args.source.read_text())
    macros = ''.join('macro_rules! ' + name + ' {($($t:tt)*)=>{()};}\n' for name in ['dev_info', 'dev_err'] if name + '!' in production)
    code = HARNESS.replace('METHOD', production).replace('CONFIGS', configs(args.source.read_text())).replace('struct Aop;', macros + 'struct Aop;')
    with tempfile.TemporaryDirectory(prefix='aop-bootargs-') as tmp:
        p = Path(tmp) / 'test.rs'
        binary = p.with_suffix('')
        p.write_text(code)
        subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', str(p), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        bad = code.replace('if aop_mmio.try_read32(address)? != value {', 'if false {')
        assert bad != code
        p.write_text(bad)
        subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', str(p), '-o', str(binary)], check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True)
        assert result.returncode != 0 and 'readback error ignored' in result.stderr
        print('PASS negative control: missing write verification')
        keep = 'let Some(v) = v else {'
        assert code.count(keep) == 1
        rustc = os.environ.get('RUSTC', 'rustc')
        bad = code.replace(keep, 'let v = &v.unwrap_or(0); if false {')
        result = compile_run(rustc, p, binary, bad)
        assert result.returncode != 0 and 'None overwrote the firmware value' in result.stderr, result.stderr
        print('PASS negative control: None written as 0')
        bad = code.replace('if need_ed0p && !seen_ed0p {', 'if need_ed0p && false {')
        assert bad != code
        result = compile_run(rustc, p, binary, bad)
        assert result.returncode != 0 and 'missing ED0p accepted' in result.stderr, result.stderr
        print('PASS negative control: missing ED0p ignored')
        for soc, val in (('T8112', 'Some(0)'), ('T6040', 'Some(0)'), ('T6000', 'Some(1)')):
            start = code.index('const HW_CFG_%s: AopHwConfig' % soc)
            at = code.index('    ncal: ', start)
            line_end = code.index('\n', at)
            bad = code[:at] + '    ncal: %s,' % val + code[line_end:]
            result = compile_run(rustc, p, binary, bad)
            assert result.returncode != 0 and '%s nCal choice' % soc in result.stderr, (soc, result.stderr)
            print('PASS negative control: %s nCal %s' % (soc, val))


if __name__ == '__main__':
    main()
