// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Miscellaneous utility functions

use core::ops::{
    Add,
    BitAnd,
    Div,
    Not,
    Sub, //
};
use kernel::prelude::*;

/// Aligns an integer type to a power of two.
pub(crate) fn align<T>(a: T, b: T) -> T
where
    T: Copy
        + Default
        + BitAnd<Output = T>
        + Not<Output = T>
        + Add<Output = T>
        + Sub<Output = T>
        + Div<Output = T>
        + core::cmp::PartialEq,
{
    let def: T = Default::default();
    #[allow(clippy::eq_op)]
    let one: T = !def / !def;

    assert!((b & (b - one)) == def);

    (a + b - one) & !(b - one)
}

/// Aligns an integer type down to a power of two.
pub(crate) fn align_down<T>(a: T, b: T) -> T
where
    T: Copy
        + Default
        + BitAnd<Output = T>
        + Not<Output = T>
        + Sub<Output = T>
        + Div<Output = T>
        + core::cmp::PartialEq,
{
    let def: T = Default::default();
    #[allow(clippy::eq_op)]
    let one: T = !def / !def;

    assert!((b & (b - one)) == def);

    a & !(b - one)
}

pub(crate) trait RangeExt<T> {
    fn overlaps(&self, other: Self) -> bool;
    fn is_superset(&self, other: Self) -> bool;
    // fn len(&self) -> usize;
    fn range(&self) -> T;
}

impl<T: PartialOrd<T> + Default + Copy + Sub<Output = T>> RangeExt<T> for core::ops::Range<T>
where
    usize: core::convert::TryFrom<T>,
    <usize as core::convert::TryFrom<T>>::Error: core::fmt::Debug,
{
    fn overlaps(&self, other: Self) -> bool {
        !(self.is_empty() || other.is_empty() || self.end <= other.start || other.end <= self.start)
    }
    fn is_superset(&self, other: Self) -> bool {
        !self.is_empty()
            && (other.is_empty() || (other.start >= self.start && other.end <= self.end))
    }
    fn range(&self) -> T {
        if self.is_empty() {
            Default::default()
        } else {
            self.end - self.start
        }
    }
    // fn len(&self) -> usize {
    //     self.range().try_into().unwrap()
    // }
}

pub(crate) fn gcd(in_n: u64, in_m: u64) -> u64 {
    let mut n = in_n;
    let mut m = in_m;

    while n != 0 {
        let remainder = m % n;
        m = n;
        n = remainder;
    }

    m
}

pub(crate) unsafe trait AnyBitPattern: Default + Sized + Copy + 'static {}

pub(crate) struct Reader<'a> {
    buffer: &'a [u8],
    offset: usize,
}

impl<'a> Reader<'a> {
    pub(crate) fn new(buffer: &'a [u8]) -> Self {
        Reader { buffer, offset: 0 }
    }

    pub(crate) fn read_up_to<T: AnyBitPattern>(&mut self, declared_size: usize) -> Result<T> {
        let mut obj: T = Default::default();
        let size: usize = core::mem::size_of::<T>().min(declared_size);
        let end = self.offset.checked_add(declared_size).ok_or(EINVAL)?;
        let range = self.offset..end;
        let src = self.buffer.get(range).ok_or(EINVAL)?;

        // SAFETY: The output pointer is valid, and the size does not exceed
        // the type size, and all bit patterns are valid.
        let dst = unsafe { core::slice::from_raw_parts_mut(&mut obj as *mut _ as *mut u8, size) };

        dst.copy_from_slice(&src[..size]);
        self.offset = end;
        Ok(obj)
    }

    /// Reads a record like [`Self::read_up_to`], for a path that implements only the first
    /// `supported` bytes of `T`. Any nonzero byte of the declared record from `supported` on,
    /// whether in a field this path does not implement or in a tail beyond `T`, fails with
    /// `EINVAL` instead of being ignored.
    pub(crate) fn read_supported<T: AnyBitPattern>(
        &mut self,
        declared_size: usize,
        supported: usize,
    ) -> Result<T> {
        let end = self.offset.checked_add(declared_size).ok_or(EINVAL)?;
        let record = self.buffer.get(self.offset..end).ok_or(EINVAL)?;
        if record.iter().skip(supported).any(|byte| *byte != 0) {
            return Err(EINVAL);
        }
        self.read_up_to(declared_size)
    }

    pub(crate) fn read<T: Default + AnyBitPattern>(&mut self) -> Result<T> {
        self.read_up_to(core::mem::size_of::<T>())
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.offset >= self.buffer.len()
    }

    pub(crate) fn skip(&mut self, size: usize) {
        self.offset += size
    }

    pub(crate) fn rewind(&mut self) {
        self.offset = 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[repr(C)]
    #[derive(Default, Copy, Clone)]
    struct Pair {
        first: u32,
        second: u32,
    }

    unsafe impl AnyBitPattern for Pair {}
    unsafe impl AnyBitPattern for u32 {}

    #[test]
    fn read_supported_accepts_zero_unsupported_fields_and_tail() {
        let raw = [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0];
        let mut reader = Reader::new(&raw);
        let pair: Pair = reader.read_supported(12, 4).unwrap();
        assert_eq!(pair.first, 5);
        assert_eq!(pair.second, 0);
        assert_eq!(reader.read::<u32>().unwrap(), 6);
        assert!(reader.is_empty());
    }

    #[test]
    fn read_supported_accepts_a_record_shorter_than_the_supported_part() {
        let raw = [7, 0];
        let mut reader = Reader::new(&raw);
        let pair: Pair = reader.read_supported(2, 4).unwrap();
        assert_eq!(pair.first, 7);
        assert_eq!(pair.second, 0);
        assert!(reader.is_empty());
    }

    #[test]
    fn read_supported_rejects_an_unsupported_field() {
        let raw = [1, 0, 0, 0, 0, 1, 0, 0];
        let mut reader = Reader::new(&raw);
        assert!(reader.read_supported::<Pair>(8, 4).is_err());
    }

    #[test]
    fn read_supported_rejects_a_nonzero_tail_beyond_the_type() {
        let raw = [1, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0x80];
        let mut reader = Reader::new(&raw);
        assert!(reader.read_supported::<Pair>(12, 8).is_err());
        let mut reader = Reader::new(&raw);
        let pair: Pair = reader.read_supported(8, 8).unwrap();
        assert_eq!(pair.second, 2);
    }

    #[test]
    fn read_supported_rejects_a_truncated_record() {
        let raw = [1, 0, 0, 0];
        let mut reader = Reader::new(&raw);
        assert!(reader.read_supported::<Pair>(8, 8).is_err());
    }
}

/// Fallibly initializes `slot` with `init` (`pin_init::raw_try_init` of later pin-init releases).
///
/// # Safety
///
/// - `slot` is a valid pointer to uninitialized memory.
/// - the caller does not touch `slot` when `Err` is returned, they are only permitted to
///   deallocate.
/// - `slot` will not move until it is dropped, i.e. it will be pinned.
#[inline(always)]
pub(crate) unsafe fn raw_try_init<T, E>(slot: *mut T, init: impl PinInit<T, E>) -> Result<(), E> {
    // SAFETY: Per safety requirement.
    unsafe { init.__pinned_init(slot) }
}
