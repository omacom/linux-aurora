#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise the production report decoder, per-channel cache and IIO callback."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def source():
    common = (ROOT / 'rust/kernel/iio/common/aop_sensors.rs').read_text()
    als = (ROOT / 'drivers/iio/common/aop_sensors/aop_als.rs').read_text()
    common = common[common.index('pub const MICRO:'):common.index('struct IIOSpec')]
    constants = '\n'.join(line for line in als.splitlines()
                          if line.startswith('const LUX_OFFSET_') or
                          line.startswith('const CCT_OFFSET_') or
                          line.startswith('const VD6286_COLOR_REPORT_LEN'))
    conversion = als[als.index('fn f32_to_micro('):als.index('enum AlsIio')]
    return STUBS + common + constants + conversion + TESTS


STUBS = r'''
#![allow(dead_code, non_camel_case_types)]
use std::{ops::Deref, sync::atomic::Ordering::{Acquire, Relaxed, Release}};
#[derive(Debug)] pub struct Error(i32);
impl Error { fn to_errno(&self)->i32 { self.0 } }
const EINVAL:Error=Error(-22); const ENODATA:Error=Error(-61);
type Result<T> = std::result::Result<T,Error>;
const GFP_KERNEL:u32=0;
mod device { pub struct Device; }
type ARef<T> = std::sync::Arc<T>;
pub struct Arc<T>(std::sync::Arc<T>);
impl<T> Arc<T> {
 fn new(value:T,_flags:u32)->Result<Self> { Ok(Self(std::sync::Arc::new(value))) }
 unsafe fn borrow<'a>(ptr:*const T)->&'a T { unsafe { &*ptr } }
}
impl<T> Deref for Arc<T> { type Target=T; fn deref(&self)->&T { &self.0 } }
struct Atomic<T>(std::sync::Mutex<T>);
impl<T:Copy> Atomic<T> {
 const fn new(value:T)->Self { Self(std::sync::Mutex::new(value)) }
 fn store(&self,value:T,_order:std::sync::atomic::Ordering) { *self.0.lock().unwrap()=value; }
 fn load(&self,_order:std::sync::atomic::Ordering)->T { *self.0.lock().unwrap() }
}
trait FakehidListener { fn process_fakehid_report(&self, data:&[u8])->Result<()>; }
mod bindings {
 pub const BINDINGS_IIO_CHAN_INFO_PROCESSED:u32=2;
 pub const BINDINGS_IIO_CHAN_INFO_RAW:u32=1;
 pub const BINDINGS_IIO_CHAN_INFO_SCALE:u32=3;
 pub const IIO_VAL_INT_PLUS_MICRO:u32=2;
 pub const IIO_VAL_INT_PLUS_NANO:u32=3;
 pub const IIO_VAL_INT:u32=1;
 pub struct iio_dev { pub priv_:*mut std::ffi::c_void }
 pub struct iio_chan_spec { pub type_:u32, pub address:u64 }
}
'''

TESTS = r'''
fn put(report:&mut [u8],offset:usize,value:f32) {
 report[offset..offset+4].copy_from_slice(&value.to_le_bytes());
}
fn read<T:MessageProcessor+'static,const N:usize>(data:&AopSensorData<T,N>,channel:u64,ty:u32)->(i32,i32,i32) {
 let mut dev=bindings::iio_dev{priv_:(data as *const AopSensorData<T,N>).cast_mut().cast()};
 let chan=bindings::iio_chan_spec{type_:ty,address:channel};
 let(mut a,mut b)=(-1,-1);
 let ret=unsafe { aop_read_raw::<T,N>(&mut dev,&chan,&mut a,&mut b,2) };
 (ret,a,b)
}
#[test] fn redbird_firmware_channels_and_abi() {
 let mut report=[0u8;122];
 // Distinct sentinels at the host event's offsets catch coordinate-system mistakes.
 put(&mut report,0x14,900.0); put(&mut report,0x34,1200.0);
 put(&mut report,0x28,321.25); put(&mut report,0x45,5000.5);
 let data=AopSensorData::new_channels(std::sync::Arc::new(device::Device),[5,6],MsgProc(LUX_OFFSET_VD6286)).unwrap();
 assert_eq!(read(&data,0,5).0,-61); assert_eq!(read(&data,1,6).0,-61);
 data.process_fakehid_report(&report).unwrap();
 assert_eq!(read(&data,0,5),(2,321,250000));
 assert_eq!(read(&data,1,6),(2,5000,500000));
 assert_eq!(read(&data,2,6).0,-22); assert_eq!(read(&data,1,5).0,-22);
 // Invalid lux does not prevent a valid CCT from advancing.
 put(&mut report,0x28,f32::NAN); put(&mut report,0x45,4100.75);
 data.process_fakehid_report(&report).unwrap();
 assert_eq!(read(&data,0,5),(2,321,250000)); assert_eq!(read(&data,1,6),(2,4100,750000));
 // Invalid CCT does not prevent lux from advancing; preserve last valid CCT.
 for bad in [0.0,-1.0,f32::NAN,f32::INFINITY,f32::NEG_INFINITY] {
  put(&mut report,0x45,bad); put(&mut report,0x28,0.125);
  data.process_fakehid_report(&report).unwrap();
  assert_eq!(read(&data,0,5),(2,0,125000)); assert_eq!(read(&data,1,6),(2,4100,750000));
 }
 // Storage saturation protects both signed IIO output parts.
 put(&mut report,0x45,f32::MAX); data.process_fakehid_report(&report).unwrap();
 assert_eq!(read(&data,1,6),(2,i32::MAX,999999));
}
#[test] fn unknown_layout_does_not_fabricate_cct() {
 let decoder=MsgProc(LUX_OFFSET_VD6286);
 let mut report=vec![0u8;140]; put(&mut report,0x28,2.0); put(&mut report,0x45,6000.0);
 for length in 0..140 {
  let sample=&report[..length];
  assert_eq!(decoder.process_channel(sample,1),if length==122 {Some(6000000000)} else {None});
  assert_eq!(decoder.process_channel(sample,0),if length>=44 {Some(2000000)} else {None});
 }
 assert_eq!(decoder.process_channel(&report,2),None);
 let data=AopSensorData::new_channels(std::sync::Arc::new(device::Device),[5,6],decoder).unwrap();
 data.process_fakehid_report(&report[..121]).unwrap();
 assert_eq!(read(&data,0,5),(2,2,0)); assert_eq!(read(&data,1,6).0,-61);
}
#[test] fn single_channel_clients_and_other_sensors() {
 struct Scalar;
 impl MessageProcessor for Scalar { fn process(&self,_m:&[u8])->Option<u64> { Some(99000000) } }
 let scalar=AopSensorData::new(std::sync::Arc::new(device::Device),7,Scalar).unwrap();
 scalar.process_fakehid_report(&[]).unwrap(); assert_eq!(read(&scalar,0,7),(2,99,0));
 assert_eq!(Scalar.process_channel(&[],1),None);
 for offset in [LUX_OFFSET_CT720,LUX_OFFSET_CT817] {
  let mut report=[0u8;122]; put(&mut report,offset,0.5); put(&mut report,0x45,6000.0);
  let decoder=MsgProc(offset);
  assert_eq!(decoder.process(&report),Some(500000));
  assert_eq!(decoder.process_channel(&report,1),None);
 }
 assert_eq!(f32_to_micro((-0.25f32).to_bits()),Some(0));
 assert_eq!(f32_to_micro(1u32),Some(0));
}
fn ct720(counts:[u32;4],flag:u8,cct:f32,integ:(u32,u32),lux:f32)->[u8;55] {
 let mut report=[0u8;55];
 for (i,count) in counts.iter().enumerate() { report[0x0d+4*i..0x11+4*i].copy_from_slice(&count.to_le_bytes()); }
 put(&mut report,0x1d,lux); report[0x2a]=flag; put(&mut report,0x2b,cct);
 report[0x2f..0x33].copy_from_slice(&integ.0.to_le_bytes()); report[0x33..0x37].copy_from_slice(&integ.1.to_le_bytes());
 report
}
#[test] fn ct720_color_channels() {
 const T:[u32;6]=[5,9,9,9,9,6];
 let data=AopSensorData::new_channels(std::sync::Arc::new(device::Device),T,MsgProc(LUX_OFFSET_CT720)).unwrap();
 let all=|data:&AopSensorData<MsgProc,6>| -> Vec<(i32,i32,i32)> { (0..6).map(|c| read(data,c as u64,T[c])).collect() };
 for channel in 0..6 { assert_eq!(read(&data,channel as u64,T[channel]).0,-61); }
 // Red, green, blue, clear in report order; lux and the color temperature as floats.
 data.process_fakehid_report(&ct720([100,200,300,400],1,5460.5,(197380,197380),12.5)).unwrap();
 assert_eq!(all(&data),[(2,12,500000),(2,100,0),(2,200,0),(2,300,0),(2,400,0),(2,5460,500000)]);
 // Too little light for a color temperature: the counts stay, the temperature reads ENODATA.
 data.process_fakehid_report(&ct720([1,2,3,11],1,788.0,(197380,197380),0.5)).unwrap();
 assert_eq!(all(&data),[(2,0,500000),(2,1,0),(2,2,0),(2,3,0),(2,11,0),(-61,-1,-1)]);
 data.process_fakehid_report(&ct720([1,2,3,12],1,2904.0,(197380,197380),0.5)).unwrap();
 assert_eq!(read(&data,5,6),(2,2904,0));
 // The firmware's validity byte, and a zero, negative or non-finite temperature.
 for (flag,cct) in [(0,5000.0),(1,0.0),(1,-5000.0),(1,f32::NAN),(1,f32::INFINITY)] {
  data.process_fakehid_report(&ct720([10,20,30,40],1,4000.0,(197380,197380),1.0)).unwrap();
  assert_eq!(read(&data,5,6),(2,4000,0));
  data.process_fakehid_report(&ct720([10,20,30,40],flag,cct,(197380,197380),1.0)).unwrap();
  assert_eq!(read(&data,5,6).0,-61); assert_eq!(read(&data,4,9),(2,40,0));
 }
 // Not the layout the color fields are read from: lux advances, the color sample is withdrawn.
 for integ in [(197380,197381),(0,0),(10_000_001,10_000_001)] {
  data.process_fakehid_report(&ct720([10,20,30,40],1,4000.0,(197380,197380),1.0)).unwrap();
  data.process_fakehid_report(&ct720([70,80,90,95],1,6000.0,integ,2.0)).unwrap();
  assert_eq!(all(&data),[(2,2,0),(-61,-1,-1),(-61,-1,-1),(-61,-1,-1),(-61,-1,-1),(-61,-1,-1)]);
 }
 data.process_fakehid_report(&ct720([10,20,30,40],1,4000.0,(10_000_000,10_000_000),1.0)).unwrap();
 assert_eq!(read(&data,1,9),(2,10,0));
 // A short report still carries lux.
 data.process_fakehid_report(&ct720([70,80,90,95],1,6000.0,(197380,197380),3.0)[..54]).unwrap();
 assert_eq!(read(&data,0,5),(2,3,0)); assert_eq!(read(&data,1,9).0,-61);
 // A report with no lux reading leaves every channel as it was.
 data.process_fakehid_report(&ct720([10,20,30,40],1,4000.0,(197380,197380),1.0)).unwrap();
 let before=all(&data);
 data.process_fakehid_report(&ct720([70,80,90,95],0,6000.0,(1,2),f32::NAN)).unwrap();
 assert_eq!(all(&data),before);
 // A count of u32::MAX saturates the stored reading without touching its neighbours.
 data.process_fakehid_report(&ct720([u32::MAX,20,30,40],1,4000.0,(197380,197380),1.0)).unwrap();
 assert_eq!(read(&data,1,9),(2,i32::MAX,999999)); assert_eq!(read(&data,2,9),(2,20,0));
 // The other parts keep their channels: no CT720 field is read at another lux offset.
 let report=ct720([100,200,300,400],1,5460.5,(197380,197380),12.5);
 for offset in [LUX_OFFSET_VD6286,LUX_OFFSET_CT817] {
  for channel in 1..6 { assert_eq!(MsgProc(offset).process_channel(&report,channel),None); assert!(!MsgProc(offset).channel_withdrawn(&report,channel)); }
 }
}
'''


def main():
    code = source()
    mutations = {
        'host-cct-offset': ('CCT_OFFSET_VD6286: usize = 0x45', 'CCT_OFFSET_VD6286: usize = 0x34'),
        'unknown-layout': ('message.len() == VD6286_COLOR_REPORT_LEN', 'message.len() >= VD6286_COLOR_REPORT_LEN - 1'),
        'cct-zero-valid': ('.filter(|value| *value != 0)', ''),
        'channel-alias': ('report_float(message, CCT_OFFSET_VD6286)', 'report_float(message, LUX_OFFSET_VD6286)'),
        'missing-saturation': ('value.min(MAX_MICRO)', 'value'),
        'ct720-clear-gate': ('CT720_CCT_MIN_CLEAR: u32 = 12', 'CT720_CCT_MIN_CLEAR: u32 = 1'),
        'ct720-integration-copies': ('integ != 0 && integ == again && ', 'integ != 0 && again != 0 && '),
        'ct720-integration-bound': (' && integ <= CT720_INTEG_MAX', ''),
        'ct720-count-order': ('[0x0d, 0x11, 0x15, 0x19]', '[0x11, 0x0d, 0x15, 0x19]'),
        'ct720-keeps-stale-color': ('self.0 == LUX_OFFSET_CT720 && channel != 0 && self.process(message).is_some()', 'self.0 == LUX_OFFSET_CT720 && channel != 0 && self.process(message).is_none()'),
        'ct720-validity-byte': (' || message.get(CT720_CCT_FLAG).copied() == Some(0)', ''),
    }
    with tempfile.TemporaryDirectory(prefix='aop-als-channels-') as directory:
        directory = Path(directory)
        for name, text in [('production', code)] + [
                (name, code.replace(old, new)) for name, (old, new) in mutations.items()]:
            assert text != code or name == 'production', name
            path = directory / 'channels.rs'
            path.write_text(text)
            binary = directory / 'channels'
            subprocess.run(['rustc', '--edition=2021', '-Dwarnings', '--test', str(path), '-o', str(binary)], check=True)
            result = subprocess.run([str(binary)], text=True, capture_output=True)
            if name == 'production':
                print(result.stdout)
                assert result.returncode == 0, result.stderr
            else:
                assert result.returncode == 101 and 'panicked at' in result.stdout + result.stderr, (name, result)
                print('killed:', name)


if __name__ == '__main__':
    main()
