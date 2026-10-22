#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Compile retained render-generation construction and address selection."""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import subprocess

ROOT = Path(__file__).resolve().parents[2]
PATH = 'drivers/gpu/drm/asahi/g17/queue/render/memory.rs'
PREFIX = r'''
#![allow(dead_code)]
use std::ops::{Deref,Range};
type Result<T=()> = std::result::Result<T,i32>;
const GFP_KERNEL:u32=0;
struct KBox<T>(Box<T>);
impl<T> KBox<T>{fn new(v:T,_g:u32)->Result<Self>{Ok(Self(Box::new(v)))}}
impl<T> Deref for KBox<T>{type Target=T;fn deref(&self)->&T{&self.0}}
mod mmu {
 pub const UAT_PGSZ:usize=0x4000;
 pub const PROT_GPU_FW_SHARED_RW:u32=3;
 pub const PROT_GPU_SHARED_RW:u32=2;
 pub struct Vm;
 pub struct KernelMapping{pub va:u64}
 impl KernelMapping{pub fn iova(&self)->u64{self.va}}
}
mod storage{CONSTANTS}
mod agx_memory_stats{pub enum KernelCategory{RenderGeneration}}
use agx_memory_stats::KernelCategory;
mod cfg{pub const AUXILIARY:std::ops::Range<u64>=0x100000..0x500000;}
enum CpuMap{WriteCombined}
struct KernelObject{va:u64,size:usize}
impl KernelObject{
 fn classify_memory(&self,_c:KernelCategory){}
 fn gpu_va(&self)->u64{self.va}
 fn map_range(&self,_v:&mmu::Vm,_r:Range<usize>,_a:Range<u64>,_align:u64,_p:u32,_g:u32)->Result<mmu::KernelMapping>{Err(-28)}
}
struct Allocator<'a>{_p:std::marker::PhantomData<&'a ()>}
impl Allocator<'_>{fn kernel(&self,size:usize,align:u64,p:u32,_c:CpuMap)->Result<KernelObject>{assert_eq!(size,mmu::UAT_PGSZ);assert_eq!(align,0x4000);assert_eq!(p,3);Ok(KernelObject{va:0x40000000,size})}}
mod buffer{
 use super::*;
 pub fn compact_alias(o:&KernelObject,_v:&mmu::Vm,r:Range<usize>,align:u64,p:u32,guard:usize)->Result<mmu::KernelMapping>{assert!(r.end<=o.size);assert_eq!(align,0x4000);assert_eq!(guard,0x4000);assert!(p==2||p==3);Ok(mmu::KernelMapping{va:o.va+r.start as u64})}
}
'''
MAIN = r'''
fn main(){
 let alloc=Allocator{_p:std::marker::PhantomData};let vm=mmu::Vm;
 assert!(storage::AUXILIARY+storage::AUXILIARY_SIZE<=storage::STATE_SIZE);
 for context in 0..128 {for generation in 0..16 {
  let va=0x200000000u64+(context*16+generation)*0x20000;
  let state=KernelObject{va,size:storage::STATE_SIZE};
  let g=Generation::new(&alloc,&vm,state).expect("construction needs no auxiliary alias");
  let addresses=g.addresses();
  assert_eq!(addresses[4],va+storage::AUXILIARY as u64);
  assert_eq!(g.state.gpu_va(),va);
  assert!(addresses[4]>=va&&addresses[4]+storage::AUXILIARY_SIZE as u64<=va+storage::STATE_SIZE as u64);
  assert_eq!(addresses[1],va+storage::TA_STATUS as u64);
 }}
 println!("PASS 2048 actual Generation constructions and retained auxiliary address controls");
}
'''


def item(source, start):
    pos = source.index(start)
    end = source.index('{', pos)+1
    depth = 1
    while depth:
        depth += (source[end]=='{')-(source[end]=='}')
        end += 1
    return source[pos:end]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--old', action='store_true')
    p.add_argument('--mutation', action='store_true')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    source = (subprocess.check_output(['git','show','990a45b609dd:'+PATH.replace('/asahi/', '/asahi-neo/')], cwd=ROOT,text=True)
              if args.old else (ROOT/PATH).read_text())
    structure = item(source, 'pub(super) struct Generation').replace('pub(super)', '')
    constructor = item(source[source.index('impl Generation'):], '    fn new(')
    addresses = item(source, '    pub(super) fn addresses').replace('pub(super)', '')
    storage=(ROOT/'drivers/gpu/drm/asahi/g17/fw/render/storage.rs').read_text()
    constants=[]
    for name in ['STATE_SIZE','DEFLAKE','TA_STATUS','AUXILIARY','AUXILIARY_SIZE']:
        line=next(x for x in storage.splitlines() if 'const '+name+':' in x)
        constants.append(line.replace('pub(crate)','pub'))
    code=PREFIX.replace('CONSTANTS','\n'.join(constants))+structure+'\nimpl Generation{'+constructor+addresses+'}\n'+MAIN
    if args.mutation:
        code=code.replace('self.state.gpu_va() + storage::AUXILIARY as u64','self.state.gpu_va()')
    path=args.output/'controls.rs';path.write_text(code)
    binary=args.output/'controls'
    subprocess.run(['rustc','--edition=2021',str(path),'-o',str(binary)],check=True)
    def no_core():resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    run=subprocess.run([str(binary)],capture_output=True,text=True,preexec_fn=no_core)
    receipt={'old':args.old,'mutation':args.mutation,'production_sha256':{n:hashlib.sha256(b.encode()).hexdigest() for n,b in [('structure',structure),('constructor',constructor),('addresses',addresses)]},'translation_sha256':hashlib.sha256(code.encode()).hexdigest(),'exit':run.returncode,'stdout':run.stdout,'stderr':run.stderr,'scope':'Actual generation constructor/addresses with typed allocation/mapping boundary services. Synthetic exhausted alias arena; not real GPU or whole Rust driver.'}
    (args.output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt,indent=2))
    if run.returncode:raise SystemExit(1)


if __name__=='__main__':main()
