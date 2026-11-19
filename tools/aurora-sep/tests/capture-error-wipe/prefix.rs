// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![allow(dead_code)]
use std::cell::{Cell, RefCell};
use std::ops::{Deref, DerefMut};
const GFP_KERNEL: u32 = 0;
const CRC_LEN: usize = 2;
const MAX_CAPTURE: usize = 0x10000;
const CMD_LEN: usize = 7;
const READ_CMD_LEN: usize = 11;
const CMD_READ_PREFIX: [u8; CMD_LEN] = [0x80, 0x13, 0x00, 0x0b, 0x00, 0x00, 0x00];
#[derive(Debug)]
struct Error(i32);
impl Error {
    fn from_errno(e: i32) -> Self {
        Self(e)
    }
}
thread_local! {
 static FREED:RefCell<Vec<Vec<u8>>>=const{RefCell::new(Vec::new())};
 static FRAME:RefCell<Vec<u8>>=const{RefCell::new(Vec::new())};
 static RC:Cell<i32>=const{Cell::new(0)};
 static PARTIAL:Cell<bool>=const{Cell::new(false)};
 static ALLOC_FAIL:Cell<bool>=const{Cell::new(false)};
 static RESIZE_FAIL:Cell<bool>=const{Cell::new(false)};
 static CALLS:Cell<usize>=const{Cell::new(0)};
}
struct KVec<T: Clone>(Vec<T>);
impl KVec<u8> {
    fn with_capacity(n: usize, _f: u32) -> Result<Self, Error> {
        if ALLOC_FAIL.with(|v| v.get()) {
            Err(Error(-12))
        } else {
            Ok(Self(Vec::with_capacity(n)))
        }
    }
    fn resize(&mut self, n: usize, b: u8, _f: u32) -> Result<(), Error> {
        if RESIZE_FAIL.with(|v| v.get()) {
            Err(Error(-12))
        } else {
            self.0.resize(n, b);
            Ok(())
        }
    }
}
impl<T: Clone> Deref for KVec<T> {
    type Target = [T];
    fn deref(&self) -> &[T] {
        &self.0
    }
}
impl<T: Clone> DerefMut for KVec<T> {
    fn deref_mut(&mut self) -> &mut [T] {
        &mut self.0
    }
}
// Only u8 is instantiated; the generic hook records the raw initialized bytes.
impl<T: Clone> Drop for KVec<T> {
    fn drop(&mut self) {
        assert_eq!(std::mem::size_of::<T>(), 1);
        let b = unsafe { std::slice::from_raw_parts(self.0.as_ptr().cast::<u8>(), self.0.len()) };
        FREED.with(|v| v.borrow_mut().push(b.to_vec()));
    }
}
unsafe fn sep_sensor_xfer2(
    tx: *const std::ffi::c_void,
    tn: usize,
    rx: *mut std::ffi::c_void,
    rn: usize,
) -> i32 {
    CALLS.with(|v| v.set(v.get() + 1));
    assert_eq!(tn, READ_CMD_LEN);
    let cmd = unsafe { std::slice::from_raw_parts(tx.cast::<u8>(), tn) };
    assert_eq!(&cmd[..CMD_LEN], &CMD_READ_PREFIX);
    assert_eq!(
        u32::from_le_bytes(cmd[CMD_LEN..].try_into().unwrap()),
        rn as u32
    );
    FRAME.with(|f| {
        let f = f.borrow();
        let n = if PARTIAL.with(|v| v.get()) {
            rn / 2
        } else {
            rn
        };
        assert!(f.len() >= n);
        unsafe { std::ptr::copy_nonoverlapping(f.as_ptr(), rx.cast::<u8>(), n) }
    });
    RC.with(|v| v.get())
}
