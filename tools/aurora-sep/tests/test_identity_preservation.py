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

CATACOMB = r'''
const MAGIC:[u8;8]=*b"SEPCTMB1";const HEADER:usize=16;const MAX_CATACOMB:usize=1<<20;
const PRIVATE_TYPE_CATACOMB_MASTER:u8=1;const PRIVATE_TYPE_CATACOMB_OWNER:u8=2;const PRIVATE_TYPE_CATACOMB_USER:u8=3;
thread_local!{static OPEN:Cell<i32>=const{Cell::new(0)};static IMAGE:RefCell<Vec<u8>>=const{RefCell::new(Vec::new())};}
mod shim{
 use super::*;pub struct StoreFile;
 impl StoreFile{
  pub fn open_readonly(_: &CStr)->Result<Self>{OPEN.with(|e|if e.get()==0{Ok(Self)}else{Err(e.get())})}
  pub fn size(&self)->Result<u64>{Ok(IMAGE.with(|v|v.borrow().len() as u64))}
  pub fn read_exact(&self,at:u64,out:&mut[u8])->Result<()>{
   IMAGE.with(|v|{let v=v.borrow();let Some(s)=v.get(at as usize..at as usize+out.len())else{return Err(EIO)};out.copy_from_slice(s);Ok(())})
  }
 }
}
@@CRC@@
PATH
METHOD
fn image(blob:&[u8])->Vec<u8>{
 let mut v=Vec::from(MAGIC);v.extend((blob.len() as u32).to_le_bytes());
 v.extend(crc16_ccitt_false(blob).to_le_bytes());v.extend([0,0]);v.extend(blob);v
}
fn main(){
 let k=PRIVATE_TYPE_CATACOMB_MASTER;
 OPEN.with(|e|e.set(ENOENT));assert!(matches!(read(k),Ok(None)),"missing file must be absence");
 for err in [EIO,13,12,30,40]{OPEN.with(|e|e.set(err));assert!(read(k).is_err(),"open error read as absence");}
 OPEN.with(|e|e.set(0));
 let good=image(&[1,2,3]);IMAGE.with(|v|*v.borrow_mut()=good.clone());
 assert_eq!(read(k).unwrap().unwrap().0,vec![1,2,3]);
 for len in 0..good.len(){IMAGE.with(|v|*v.borrow_mut()=good[..len].to_vec());assert!(read(k).is_err(),"truncated file read as absence or data");}
 let mut bad=good.clone();bad[HEADER]^=1;IMAGE.with(|v|*v.borrow_mut()=bad);assert!(read(k).is_err(),"bad checksum admitted");
 let mut bad=good;bad[0]^=1;IMAGE.with(|v|*v.borrow_mut()=bad);assert!(read(k).is_err(),"bad magic admitted");
 println!("catacomb cases passed");
}
'''


CALLERS = r'''
use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
macro_rules! dev_info {($($x:tt)*)=>{()};}
const PRIVATE_TYPE_CATACOMB_MASTER:u8=0xF2;const PRIVATE_TYPE_CATACOMB_OWNER:u8=0xF3;const PRIVATE_TYPE_CATACOMB_USER:u8=0xF4;
const SBIO_PROBE_USER_ID:i32=1;
#[derive(Debug,PartialEq)] enum RestoreOutcome{Restored,AlreadyActive,NoStoredFile,Failed,EmptyTolerated}
enum LoadAnswer{StatusZero,ColdTransition,AlreadyActive}
// 0 = valid file, ENOENT = missing, anything else = read error.
thread_local!{
 static FILE:RefCell<[i32;3]>=const{RefCell::new([0;3])};
 static HOST:Cell<i32>=const{Cell::new(-1)};
 static ASKED_ENCLAVE:Cell<bool>=const{Cell::new(false)};
 static TRANSFERRED:Cell<bool>=const{Cell::new(false)};
}
mod catacomb{
 use super::*;
 pub fn is_kind(_:u8)->bool{true}
 pub fn read(kind:u8)->Result<Option<KVec<u8>>>{
  match FILE.with(|f|f.borrow()[(kind-PRIVATE_TYPE_CATACOMB_MASTER) as usize]){
   0=>Ok(Some(KVec(vec![1,0,0,0]))),ENOENT=>Ok(None),e=>Err(e)}
 }
 @@IS_SAVED@@
 @@IS_ABSENT@@
}
mod store{pub struct Key;impl Key{pub fn root(_:u8)->Self{Key}}}
struct Store;
impl Store{fn read(&self,_:&store::Key)->Result<Option<KVec<u8>>>{match HOST.with(|h|h.get()){0=>Ok(Some(KVec(vec![1,0,0,0]))),ENOENT=>Ok(None),e=>Err(e)}}}
mod module_parameters{pub struct Param;pub static probe_owner_export:Param=Param;impl Param{pub fn value(&self)->&u8{&1}}}
mod sbio{
 pub enum ComponentAction{AlreadyActive,Unsupported,Load}
 pub fn component_action(_:u32)->ComponentAction{ComponentAction::Load}
 pub const SBIO_SAVED_USER_ID_AT:usize=0;pub const SBIO_STATUS_COLD_TRANSITION:u32=0x8002;pub const SBIO_STATUS_ALREADY_ACTIVE:u32=0x101;
 pub const COMPONENT_STATE_ACTIVE:u32=1;
 pub struct SbioLoadCatacomb;
 impl SbioLoadCatacomb{pub fn new(_:&[u8])->Option<Self>{Some(Self)}pub fn opcode(&self)->u8{0}pub fn name(&self)->u8{0}pub fn payload(&self)->&[u8]{&[]}}
 pub struct NoExistingCatacomb;impl NoExistingCatacomb{pub fn from_zero_identities(_:usize)->Option<Self>{None}}
 pub enum ContextScope{SYSTEM}pub fn sbio_select_context(_:ContextScope,_:&NoExistingCatacomb){}
 pub struct ComponentStates;impl ComponentStates{pub fn new(_:&[u8])->Self{Self}pub fn state_for(&self,_:i32)->Option<u32>{None}}
 pub struct CatacombUser;impl CatacombUser{pub const OWNER:Self=Self;pub fn value(&self)->i32{0}}
}
struct Status;impl Status{fn is_ok(&self)->bool{true}fn answered(&self)->Option<u32>{None}}
impl std::fmt::Display for Status{fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result{Ok(())}}
struct Done{status:Status}
struct Sbio{dev:(),templates_restored:AtomicBool}
impl Sbio{
 fn with_host_store<R>(&self,f:impl FnOnce(&Store)->R)->Option<R>{if HOST.with(|h|h.get())<0{None}else{Some(f(&Store))}}
 fn sbio_transfer_raw(&self,_:u8,_:u8,_:&[u8])->Result<Done>{TRANSFERRED.with(|t|t.set(true));Ok(Done{status:Status})}
 fn confirm_active(&self,_:i32,_:&CStr,_:LoadAnswer)->RestoreOutcome{RestoreOutcome::Restored}
 fn log_identity_count(&self,_:&CStr)->Option<usize>{Some(1)}
 fn enclave_identities_for(&self,_:i32)->Option<Vec<u8>>{ASKED_ENCLAVE.with(|a|a.set(true));None}
 fn sbio_expect_ok(&self,_:&())->Option<()>{None}
 fn read_component_states(&self)->Option<Vec<u8>>{None}
 fn save_catacomb(&self,_:sbio::CatacombUser,_:u8,_:&CStr)->bool{false}
 @@RESTORE@@
 @@READ_STORED@@
 @@PROBE@@
}
fn set(files:[i32;3],host:i32){FILE.with(|f|*f.borrow_mut()=files);HOST.with(|h|h.set(host));ASKED_ENCLAVE.with(|a|a.set(false));TRANSFERRED.with(|t|t.set(false));}
fn main(){
 let s=Sbio{dev:(),templates_restored:AtomicBool::new(false)};
 let user=PRIVATE_TYPE_CATACOMB_USER;
 set([0,0,EIO],-1);
 assert_eq!(s.restore_catacomb(Some(0),1,user,c"user"),RestoreOutcome::Failed,"unreadable catacomb restored as missing");
 assert!(!TRANSFERRED.with(|t|t.get()),"unreadable catacomb sent to the enclave");
 set([0,0,ENOENT],0);
 assert_eq!(s.restore_catacomb(Some(0),1,user,c"user"),RestoreOutcome::Restored,"host-store copy not used");
 set([0,0,ENOENT],EIO);
 assert_eq!(s.restore_catacomb(Some(0),1,user,c"user"),RestoreOutcome::Failed,"unreadable host store restored as missing");
 set([0,0,ENOENT],-1);
 assert_eq!(s.restore_catacomb(Some(0),1,user,c"user"),RestoreOutcome::NoStoredFile,"genuine absence not reported");
 // Owner probe: positive control reaches the enclave; an unreadable owner file must not.
 set([0,ENOENT,0],-1);s.probe_owner_export();
 assert!(ASKED_ENCLAVE.with(|a|a.get()),"owner probe never reached its enclave check");
 for e in [EIO,13]{set([0,e,0],-1);s.probe_owner_export();assert!(!ASKED_ENCLAVE.with(|a|a.get()),"unreadable owner catacomb treated as absent");}
 set([EIO,ENOENT,0],-1);s.probe_owner_export();
 assert!(!ASKED_ENCLAVE.with(|a|a.get()),"unreadable master catacomb treated as saved");
 println!("caller cases passed");
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

    def test_catacomb_read_error_is_not_absence(self):
        text=(ROOT/'catacomb.rs').read_text()
        method=body(text,'pub(crate) fn read(kind: u8)')
        path=body(text,'fn path_for(kind: u8)').replace('crate::','')
        crc=body((ROOT/'store.rs').read_text(),'pub(crate) const fn crc16_ccitt_false(')
        make=lambda m:CATACOMB.replace('METHOD',m).replace('PATH',path).replace('@@CRC@@',crc)
        self.run_rust(make(method))
        old=method.replace("Err(e) => return Err(e),","Err(e) => return Ok(None),",1)
        self.assertNotEqual(old,method)
        self.run_rust(make(old),should_pass=False)

    def test_callers_refuse_unreadable_catacombs(self):
        cat=(ROOT/'catacomb.rs').read_text();sb=(ROOT/'sbio.rs').read_text()
        parts={'IS_SAVED':body(cat,'pub(crate) fn is_saved('),'IS_ABSENT':body(cat,'pub(crate) fn is_absent('),
               'RESTORE':body(sb,'    fn restore_catacomb('),'READ_STORED':body(sb,'    fn read_stored('),
               'PROBE':body(sb,'    fn probe_owner_export(')}
        def make(**over):
            text=CALLERS
            for key,value in {**parts,**over}.items():
                text=text.replace('@@'+key+'@@',value.replace('crate::',''))
            return text
        self.run_rust(make())
        restore=parts['RESTORE'].replace('Err(_) => return RestoreOutcome::Failed,','Err(_) => return RestoreOutcome::NoStoredFile,',1)
        absent=parts['IS_ABSENT'].replace('matches!(read(kind), Ok(None))','!matches!(read(kind), Ok(Some(_)))',1)
        for name,old in (('RESTORE',restore),('IS_ABSENT',absent)):
            self.assertNotEqual(old,parts[name])
            self.run_rust(make(**{name:old}),should_pass=False)

if __name__=='__main__':
    unittest.main()
