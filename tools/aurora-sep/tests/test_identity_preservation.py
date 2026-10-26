# SPDX-License-Identifier: MIT
"""Compile production identity readers against in-memory, device-free seams."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3] / 'drivers/soc/apple'


def body(source, signature):
    start = source.index(signature)
    at = source.index('{', start)
    depth, end = 1, at + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


COMMON = r'''
#![allow(dead_code, unused_variables, unused_imports, unused_macros, non_upper_case_globals)]
use std::cell::{Cell, RefCell};
use std::ops::{Deref,DerefMut};
use std::ffi::CStr;
type Result<T> = std::result::Result<T,i32>;
const ENOENT:i32=2; const EIO:i32=5; const EINVAL:i32=22;
const ENOTSYNC:i32=52; const EEXIST:i32=17; const GFP_KERNEL:u32=0;
#[derive(Clone)] struct KVec<T>(Vec<T>);
impl<T> KVec<T> {
 fn new()->Self {Self(Vec::new())}
 fn with_capacity(n:usize,_:u32)->Result<Self> {Ok(Self(Vec::with_capacity(n)))}
 fn push(&mut self,v:T,_:u32)->Result<()> {self.0.push(v);Ok(())}
}
impl<T:Clone> KVec<T> {
 fn resize(&mut self,n:usize,v:T,_:u32)->Result<()> {self.0.resize(n,v);Ok(())}
 fn extend_from_slice(&mut self,v:&[T],_:u32)->Result<()> {self.0.extend_from_slice(v);Ok(())}
}
impl<T> Deref for KVec<T> {type Target=Vec<T>; fn deref(&self)->&Vec<T>{&self.0}}
impl<T> DerefMut for KVec<T> {fn deref_mut(&mut self)->&mut Vec<T>{&mut self.0}}
struct Secret(KVec<u8>);
impl Deref for Secret {type Target=[u8];fn deref(&self)->&[u8]{&self.0.0}}
macro_rules! dev_err {($($x:tt)*)=>{()};}
macro_rules! dev_warn {($($x:tt)*)=>{()};}
'''

REFKEY = r'''
thread_local! {static OPEN_ERROR:Cell<i32>=const{Cell::new(0)};}
mod sks {pub type KeyBagHandle=u32;}
mod module_parameters {
 pub struct Param; pub static refkey_v2:Param=Param;
 impl Param {pub fn value(&self)->&u8{&0}}
}
mod shim {
 use super::*;
 pub struct StoreFile;
 impl StoreFile {
  pub fn open_readonly(_: &CStr)->Result<Self>{OPEN_ERROR.with(|e|if e.get()==0{Ok(Self)}else{Err(e.get())})}
  pub fn open(_: &CStr)->Result<Self>{panic!("fixture never permits persistence")}
  pub fn size(&self)->Result<u64>{Ok(1)}
  pub fn read_exact(&self,_:u64,_:&mut[u8])->Result<()>{Ok(())}
  pub fn write_all(&self,_:u64,_:&[u8])->Result<()>{panic!("write")}
  pub fn sync(&self)->Result<()>{panic!("sync")}
 }
}
struct Cache;
impl Cache {fn lock(&self)->Option<()>{None}}
struct SepData {machine_refkey:Cache,creates:Cell<usize>}
impl SepData {
 fn sks_create_refkey(&self,_:u32,_:&[u8])->Option<KVec<u8>>{self.creates.set(self.creates.get()+1);None}
 fn cache_machine_refkey(&self,_:KVec<u8>)->bool{true}
 METHOD
}
fn main(){
 for error in [0,ENOENT,EIO,13,12,30,40] {
  OPEN_ERROR.with(|e|e.set(error));
  let s=SepData{machine_refkey:Cache,creates:Cell::new(0)};
  s.sks_machine_refkey(0,&[]);
  assert_eq!(s.creates.get(),usize::from(error==ENOENT),"only absence permits ref-key creation");
 }
 println!("refkey cases passed");
}
'''

INDEX = r'''
const UUID_LEN:usize=16; const LABEL_LEN:usize=128;const MAX_IDENTITIES:usize=32;
const INDEX_FORMAT:u32=2;
#[derive(Clone)] struct Identity{uuid:[u8;UUID_LEN],label:[u8;LABEL_LEN]}
impl Identity {const ZERO:Self=Self{uuid:[0;UUID_LEN],label:[0;LABEL_LEN]};}
struct Store(Option<Vec<u8>>);
impl Store {fn read(&mut self,_:&())->Result<Option<KVec<u8>>>{Ok(self.0.clone().map(KVec))}}
fn identity_key(){}
struct IdentityIndex{entries:KVec<Identity>}
impl IdentityIndex{
 fn new()->Self{Self{entries:KVec::new()}}
 METHOD
}
fn image(count:u32,n:usize)->Vec<u8>{
 let mut v=Vec::from(INDEX_FORMAT.to_le_bytes());v.extend(count.to_le_bytes());
 v.resize(8+n*(UUID_LEN+LABEL_LEN),0);v
}
fn main(){
 assert!(IdentityIndex::load(&mut Store(None)).unwrap().entries.is_empty());
 for n in [0,1,MAX_IDENTITIES]{assert_eq!(IdentityIndex::load(&mut Store(Some(image(n as u32,n)))).unwrap().entries.len(),n);}
 for len in 0..8 {assert!(IdentityIndex::load(&mut Store(Some(vec![0;len]))).is_err(),"short index admitted");}
 let valid=image(1,1);
 for len in 8..valid.len(){assert!(IdentityIndex::load(&mut Store(Some(valid[..len].to_vec()))).is_err(),"truncated index admitted");}
 for v in [image(2,1),image((MAX_IDENTITIES+1) as u32,MAX_IDENTITIES+1)] {
  assert!(IdentityIndex::load(&mut Store(Some(v))).is_err(),"count mismatch admitted");
 }
 let mut trailing=image(0,0);trailing.push(0);
 assert!(IdentityIndex::load(&mut Store(Some(trailing))).is_err(),"trailing index bytes admitted");
 println!("index cases passed");
}
'''

KEYBAG = r'''
const MAGIC:[u8;16]=*b"APPLE-SEP-KBAG01";
const VERSION:u32=2;const STATE_REFUSED:u32=2;const UUID_LEN:usize=16;
const OFF_VERSION:usize=16;const OFF_STATE:usize=20;const OFF_LEN:usize=24;
const OFF_CRC:usize=28;const OFF_UUID:usize=32;const OFF_SECRET_LEN:usize=48;
const HEADER:usize=52;const MAX_WRAPPED:usize=65536;
#[derive(Clone,Copy)] enum Slot{Identity}
impl Slot{fn path(self)->&'static CStr{c"unused"}}
enum UuidProvenance{Generated,Readback}
impl UuidProvenance{fn from_state(n:u32)->Option<Self>{match n{1=>Some(Self::Generated),3=>Some(Self::Readback),_=>None}}}
struct NoStoredKeyBag{slot:Slot}
struct StoredKeyBag{wrapped:KVec<u8>,uuid:[u8;16],secret:Secret,provenance:UuidProvenance}
enum State{Present(StoredKeyBag),Absent(NoStoredKeyBag)}
thread_local!{static IMAGE:RefCell<Vec<u8>>=const{RefCell::new(Vec::new())};}
mod shim{
 use super::*;pub struct StoreFile;
 impl StoreFile{
  pub fn open_readonly(_: &CStr)->Result<Self>{Ok(Self)}
  pub fn size(&self)->Result<u64>{Ok(IMAGE.with(|v|v.borrow().len() as u64))}
  pub fn read_exact(&self,at:u64,out:&mut[u8])->Result<()>{
   IMAGE.with(|v|{let v=v.borrow();let Some(s)=v.get(at as usize..at as usize+out.len())else{return Err(EIO)};out.copy_from_slice(s);Ok(())})
  }
 }
}
@@CRC@@
CHECKSUM
METHOD
fn refused()->Vec<u8>{
 let mut v=vec![0;HEADER];v[..16].copy_from_slice(&MAGIC);
 v[OFF_VERSION..OFF_VERSION+4].copy_from_slice(&VERSION.to_le_bytes());
 v[OFF_STATE..OFF_STATE+4].copy_from_slice(&STATE_REFUSED.to_le_bytes());
 let crc=crc16_ccitt_false(&checksum_input(STATE_REFUSED,&[],&[0;16],&[]).unwrap());
 v[OFF_CRC..OFF_CRC+2].copy_from_slice(&crc.to_le_bytes());v
}
fn main(){
 let good=refused();IMAGE.with(|v|*v.borrow_mut()=good.clone());
 assert!(matches!(read(Slot::Identity),Ok(State::Absent(_))));
 for at in OFF_LEN..HEADER {
  let mut bad=good.clone();bad[at]^=1;IMAGE.with(|v|*v.borrow_mut()=bad);
  assert!(read(Slot::Identity).is_err(),"noncanonical refusal admitted at {}",at);
 }
 let mut long=good;long.push(0);IMAGE.with(|v|*v.borrow_mut()=long);
 assert!(read(Slot::Identity).is_err(),"trailing refusal data admitted");
 let wrapped=[1,2,3];let secret=[4,5];let uuid=[6;16];
 let mut committed=refused();committed[OFF_STATE..OFF_STATE+4].copy_from_slice(&3u32.to_le_bytes());
 committed[OFF_LEN..OFF_LEN+4].copy_from_slice(&3u32.to_le_bytes());
 committed[OFF_SECRET_LEN..OFF_SECRET_LEN+4].copy_from_slice(&2u32.to_le_bytes());
 committed[OFF_UUID..OFF_UUID+16].copy_from_slice(&uuid);
 let crc=crc16_ccitt_false(&checksum_input(3,&wrapped,&uuid,&secret).unwrap());
 committed[OFF_CRC..OFF_CRC+2].copy_from_slice(&crc.to_le_bytes());
 committed.extend(wrapped);committed.extend(secret);IMAGE.with(|v|*v.borrow_mut()=committed);
 assert!(matches!(read(Slot::Identity),Ok(State::Present(_))),"valid committed bag refused");
 println!("keybag cases passed");
}
'''


@unittest.skipUnless(shutil.which('rustc'), 'rustc required')
class Preservation(unittest.TestCase):
    def run_rust(self, text, should_pass=True):
        with tempfile.TemporaryDirectory(prefix='sep-identity-readers-') as tmp:
            p=Path(tmp);(p/'test.rs').write_text(COMMON+text)
            subprocess.run(['rustc','--edition=2021',str(p/'test.rs'),'-o',str(p/'test')],check=True)
            result=subprocess.run([str(p/'test')],capture_output=True,text=True)
            if should_pass:
                self.assertEqual(result.returncode,0,result.stderr)
            else:
                self.assertNotEqual(result.returncode,0)
                self.assertIn('panicked',result.stderr)

    def test_refkey_requires_genuine_absence(self):
        text=(ROOT/'refkey.rs').read_text()
        method=body(text,'    pub(crate) fn sks_machine_refkey(')
        self.run_rust(REFKEY.replace('METHOD',method))
        old=re.sub(r'        let existing = match.*?        if let Some\(f\) = existing \{',
                   '        if let Ok(f) = shim::StoreFile::open_readonly(path) {',method,flags=re.S)
        self.assertNotEqual(old,method)
        self.run_rust(REFKEY.replace('METHOD',old),should_pass=False)

    def test_index_requires_complete_record(self):
        method=body((ROOT/'bio.rs').read_text(),'    pub(crate) fn load(store:')
        self.run_rust(INDEX.replace('METHOD',method))
        old=method.replace('if raw.len() < 8 {\n            return Err(EINVAL);','if raw.len() < 8 {\n            return Ok(index);',1)
        self.assertNotEqual(old,method)
        self.run_rust(INDEX.replace('METHOD',old),should_pass=False)

    def test_refused_keybag_requires_canonical_writer_record(self):
        text=(ROOT/'keybag.rs').read_text()
        method=body(text,'pub(crate) fn read(slot:')
        crc=body((ROOT/'store.rs').read_text(),'pub(crate) const fn crc16_ccitt_false(')
        checksum=body(text,'fn checksum_input(')
        make=lambda m:KEYBAG.replace('METHOD',m).replace('CHECKSUM',checksum).replace('@@CRC@@',crc)
        self.run_rust(make(method))
        start=method.index('    if state == STATE_REFUSED {')
        stop=start+len(body(method[start:],'    if state == STATE_REFUSED {'))
        old=method[:start]+'    if state == STATE_REFUSED { return Ok(State::Absent(NoStoredKeyBag { slot })); }'+method[stop:]
        self.run_rust(make(old),should_pass=False)


if __name__=='__main__':
    unittest.main()
