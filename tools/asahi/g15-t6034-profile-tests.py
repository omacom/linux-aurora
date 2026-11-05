#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Compile actual G15C profiles and test T6034 shape and exact firmware admission.

The diagnostic profile has no firmware row. Once a measured row is present,
exercise its complete public identity and J514M binding without reading an image.
"""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
G15 = ROOT / 'drivers/gpu/drm/asahi/g15'

def item(text, marker):
    a=text.index(marker); at=text.index('{',a)+1; depth=1
    while depth:
        depth+=(text[at]=='{')-(text[at]=='}');at+=1
    return text[a:at]

hw=(G15/'hw.rs').read_text();record=(G15/'board/record.rs').read_text()
config=item((G15/'mod.rs').read_text(),'pub(crate) struct Config {')
reference=item(record,'pub(crate) struct Reference {')
value=item(record,'pub(crate) const J516C: Reference = Reference {')+';'
types='\n'.join(item(hw,name) for name in ['pub(crate) struct HwId {','pub(crate) struct FirmwareBuild {','pub(crate) struct Layout {','pub(crate) struct ComputeRegs {'])
types+='\n#[derive(Copy, Clone)]\n'+item(hw,'pub(crate) struct IoMap {')
types+='\n#[derive(Copy, Clone, PartialEq, Eq)]\n'+item(hw,'pub(crate) enum Platform {')
types+='\n#[derive(Copy, Clone, PartialEq, Eq)]\n'+item(hw,'pub(crate) enum Board {')
source=r'''
#![allow(dead_code)]
extern crate self as kernel;
pub mod str { pub type CStr = core::ffi::CStr; }
#[macro_export] macro_rules! c_str { ($s:literal) => { $crate::c_str_const(concat!($s,"\0").as_bytes()) }; }
const fn c_str_const(s: &[u8])->&core::ffi::CStr { unsafe { core::ffi::CStr::from_bytes_with_nul_unchecked(s) } }
mod hw { pub enum GpuVariant { C } }
#[path="@MATCHING@"] mod matching;
#[path="@LOOKUP@"] mod lookup;
mod g15 {
 use kernel::str::CStr;
 @CONFIG@
 pub mod board { pub mod record {
  const RELATIVE_STATES: usize = 7;
  #[derive(Copy,Clone,Debug,PartialEq,Eq)]
  @REFERENCE@
  @VALUE@
 } }
 pub mod hw {
   @TYPES@
  #[path="@T6031@"] pub(crate) mod t6031;
  #[path="@T6034@"] pub(crate) mod t6034;
 }
}
fn main() {
 use g15::hw::{t6031,t6034};
 let c=&t6034::CONFIG;
 assert_eq!(c.chip_id,0x6034);
 assert_eq!(c.machine.to_bytes(),b"apple,j514m");
 assert_eq!((c.num_clusters,c.num_cores_per_cluster),(4,10));
 assert_eq!(c.gpu_revision,0x11);
 assert_eq!((c.hw_ids[0].version,c.hw_ids[0].counts),(0x07041100,0x0021040a));
 // An empty table is the first diagnostic build. A completed profile may
 // contain only the measured row for this exact board.
 assert_eq!(c.firmware.len(),1);
 if !c.firmware.is_empty() {
  use lookup::{Measured,Verdict};
  assert_eq!(c.firmware.len(),1);
  let b=&c.firmware[0];
  assert_eq!(b.machine.map(|m|m.to_bytes()),Some(b"apple,j514m".as_slice()));
  let exact=Measured {text_size:b.text_size,data_size:b.data_size,marker:b.marker,sha256:b.text_sha256};
  let verdict=|measured:&Measured,board:&[u8]| {
   let qualified=c.firmware.iter().map(|b| {
    (Measured {text_size:b.text_size,data_size:b.data_size,marker:b.marker,sha256:b.text_sha256},
     b.machine.is_some_and(|m|m.to_bytes()==board))
   });
   lookup::verdict(measured,qualified,0)
  };
  assert_eq!(verdict(&exact,b"apple,j514m"),Verdict::Pinned(0));
  for board in [b"apple,j514c".as_slice(),b"apple,j516c",b"apple,j516m",b"apple,j514s",b"apple,j999"] {
   assert_eq!(verdict(&exact,board),Verdict::Refused);
  }
  let mut changed=exact;changed.sha256[31]^=1;
  assert_eq!(verdict(&changed,b"apple,j514m"),Verdict::Refused);
  let mut changed=exact;changed.text_size+=0x4000;
  assert_eq!(verdict(&changed,b"apple,j514m"),Verdict::Refused);
  let mut changed=exact;changed.data_size+=0x4000;
  assert_eq!(verdict(&changed,b"apple,j514m"),Verdict::Refused);
  let mut changed=exact;changed.marker+=8;
  assert_eq!(verdict(&changed,b"apple,j514m"),Verdict::Refused);
 }
 assert_eq!((c.compute.tail,c.compute.ptc,c.compute.dupm),(0x2980,0x6000400020,0x3c000000140));
 assert_eq!((c.compute.tail,c.compute.ptc,c.compute.dupm),(t6031::CONFIG.compute.tail,t6031::CONFIG.compute.ptc,t6031::CONFIG.compute.dupm));
 let mask=0x7b5fc7f5fdu64;
 let masks:Vec<_>=(0..4).map(|i|(mask>>(10*i))&1023).collect();
 assert_eq!(masks,[0x1fd,0x1fd,0x1fc,0x1ed]);
 let counts:Vec<_>=(0..4).map(|i|((mask>>(10*i))&1023).count_ones()).collect();
 assert_eq!(counts,[8,8,7,7]);
 let maps=c.layout.iomaps;let records=c.layout.record_slots;
 assert_eq!(maps.len(),17);assert_eq!(records.len(),17);
 for (i,map) in maps.iter().enumerate() {
  if map.slot==11 {
   assert_eq!((map.pa,map.map_bytes),(0x220000000,6*0xd8000));
   assert_eq!(records[i],(6*0xd8000,0xd8000,true));
  } else {
   assert_eq!((map.pa,map.map_bytes,map.delta),(t6031::IOMAPS[i].pa,t6031::IOMAPS[i].map_bytes,t6031::IOMAPS[i].delta));
  }
  if i>0 { assert_eq!(map.va,maps[i-1].va+maps[i-1].map_bytes+0x4000); }
 }
 assert_eq!(t6031::IOMAPS.iter().find(|m|m.slot==11).unwrap().map_bytes,8*0xd8000);
}
'''
for token,replacement in {'@CONFIG@':config,'@REFERENCE@':reference,'@VALUE@':value,'@TYPES@':types,'@T6031@':str(G15/'hw/t6031.rs'),'@T6034@':str(G15/'hw/t6034.rs'),'@MATCHING@':str(G15/'identity/matching.rs'),'@LOOKUP@':str(G15/'identity/lookup.rs')}.items():source=source.replace(token,replacement)
identity=(G15/'identity.rs').read_text()
# Run the production firmware admission against synthetic public identity bytes.
firmware=item(identity,'pub(crate) fn firmware(')
identity_source=r'''
#![allow(dead_code,unused_variables)]
use std::fmt;
use std::sync::atomic::{AtomicU32,Ordering};
#[derive(Debug)] struct Error;
impl From<std::num::TryFromIntError> for Error { fn from(_:std::num::TryFromIntError)->Self { Self } }
type Result<T=()> = std::result::Result<T,Error>;
type KVVec<T> = Vec<T>;
const EINVAL:Error=Error;
mod device { pub struct Device; }
struct FirmwareBuild { text_size:u64,data_size:u64,marker:u32,text_sha256:[u8;32],machine:Option<&'static str>,name:&'static str }
mod adt { pub struct GpuAdt { pub text:std::ops::Range<u64>,pub data:std::ops::Range<u64> } }
mod params { pub fn accept_text_prefix()->u64 { super::PREFIX_READS.fetch_add(1,super::Ordering::Relaxed); 0x1111111111111111 } }
static PREFIX_READS:AtomicU32=AtomicU32::new(0);
mod glue { pub fn sha256(_: &[u8])->[u8;32] {[0x11;32]} pub fn machine(_: &str)->bool {true} }
#[path="MATCHING"] mod matching;
#[path="LOOKUP"] mod lookup;
use lookup::{Measured,Verdict};
struct Hex<'a>(&'a [u8]);
impl fmt::Display for Hex<'_> {fn fmt(&self,f:&mut fmt::Formatter<'_>)->fmt::Result {for b in self.0 {write!(f,"{:02x}",b)?;}Ok(())}}
impl From<&FirmwareBuild> for Measured { fn from(b:&FirmwareBuild)->Self {Self {text_size:b.text_size,data_size:b.data_size,marker:b.marker,sha256:b.text_sha256}} }
macro_rules! dev_err { ($d:expr,$($x:tt)*) => {{ eprintln!($($x)*); }} }
macro_rules! dev_info { ($d:expr,$($x:tt)*) => {{}} }
macro_rules! dev_warn { ($d:expr,$($x:tt)*) => {{}} }
fn copy_physical(_:u64,size:usize)->Result<KVVec<u8>> {
 let mut b=vec![0u8;size];b[16..24].copy_from_slice(b"GKTS\x08\0\0\0");b[24..32].fill(0xa5);Ok(b)
}
mod identity {
 use super::*;
 FIRMWARE
}
static PINNED:[FirmwareBuild;1]=[FirmwareBuild {text_size:64,data_size:64,marker:16,text_sha256:[0x11;32],machine:Some("apple,j514m"),name:"synthetic pinned identity"}];
static OTHER:[FirmwareBuild;1]=[FirmwareBuild {text_size:64,data_size:64,marker:16,text_sha256:[0x22;32],machine:None,name:"other"}];
fn main() {
 let a=adt::GpuAdt{text:0x1000..0x1040,data:0x2000..0x2040};
 assert!(identity::firmware(&device::Device,&a,&[],true).is_err());
 assert_eq!(PREFIX_READS.load(Ordering::Relaxed),0,"empty profile must not consult prefix escape hatch");
 assert!(identity::firmware(&device::Device,&a,&PINNED,false).unwrap().is_some(),"pinned exact identity remains admitted with prefix disabled");
 assert!(identity::firmware(&device::Device,&a,&OTHER,false).is_err(),"nonempty strict profile must reject a matching override prefix");
 assert_eq!(PREFIX_READS.load(Ordering::Relaxed),0,"strict profiles never consult the prefix escape hatch");
 assert!(identity::firmware(&device::Device,&a,&OTHER,true).unwrap().is_none(),"existing explicitly requested prefix behavior remains");
 assert_eq!(PREFIX_READS.load(Ordering::Relaxed),1);
}
'''
for token,replacement in {'FIRMWARE':firmware,'MATCHING':str(G15/'identity/matching.rs'),'LOOKUP':str(G15/'identity/lookup.rs')}.items():identity_source=identity_source.replace(token,replacement)
with tempfile.TemporaryDirectory(prefix='t6034-profile-') as temp:
 for name,code in [('profile',source),('measurement-refusal',identity_source)]:
  p=Path(temp)/name;p.with_suffix('.rs').write_text(code)
  subprocess.run(['rustc','--edition=2021','--cfg','CONFIG_DRM_ASAHI_G15','-o',str(p),str(p.with_suffix('.rs'))],check=True)
  subprocess.run([str(p)],check=True)
  print(name+': PASS')
