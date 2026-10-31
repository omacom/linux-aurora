// SPDX-License-Identifier: GPL-2.0-only OR MIT
const PRIVATE_TYPE_CATACOMB_MASTER: u8 = 0xf2;
const PRIVATE_TYPE_CATACOMB_OWNER: u8 = 0xf3;
const PRIVATE_TYPE_CATACOMB_USER: u8 = 0xf4;
extern "C" {
    fn fixture_reset();
    fn fixture_done();
    fn fixture_fault(stage: i32, error: i32);
    fn fixture_write_fault(at: usize, error: i32, zero: i32);
    fn fixture_short(n: usize);
    fn fixture_count(n: i32) -> i32;
}
struct Fixture;
impl Fixture {
    fn new() -> Self {
        unsafe { fixture_reset() };
        Self
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        unsafe { fixture_done() };
    }
}
fn fault(stage: i32, error: i32) {
    unsafe { fixture_fault(stage, error) }
}
fn count(n: i32) -> i32 {
    unsafe { fixture_count(n) }
}
fn user_read() -> Result<Option<KVec<u8>>> {
    catacomb::read_checked(PRIVATE_TYPE_CATACOMB_USER)
}
fn current() -> Vec<u8> {
    user_read().unwrap().unwrap().to_vec()
}
fn original() {
    catacomb::write(PRIVATE_TYPE_CATACOMB_USER, b"original valid catacomb").unwrap();
}
struct Status;
impl Status {
    fn is_ok(&self) -> bool {
        true
    }
}
impl std::fmt::Display for Status {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ok")
    }
}
struct Done {
    status: Status,
    payload: KVec<u8>,
}
struct SepData {
    dev: (),
    confirms: Cell<usize>,
}
impl SepData {
    fn new() -> Self {
        Self {
            dev: (),
            confirms: Cell::new(0),
        }
    }
    fn sbio_transfer(&self, op: &sbio::Operation) -> Result<Done> {
        let mut blob = vec![0xab; sbio::SBIO_SAVED_MIN];
        blob[8..12].copy_from_slice(&501i32.to_le_bytes());
        if op.0 {
            self.confirms.set(self.confirms.get() + 1);
        }
        Ok(Done {
            status: Status,
            payload: KVec(blob),
        })
    }
}
mod sbio {
    #[derive(Clone, Copy)]
    pub struct CatacombUser;
    impl CatacombUser {
        pub const OWNER: Self = Self;
        pub fn value(&self) -> i32 {
            501
        }
    }
    pub struct SaveSelector;
    impl SaveSelector {
        pub fn new(_: CatacombUser) -> Self {
            Self
        }
    }
    pub struct Operation(pub bool);
    pub fn sbio_save_catacomb(_: &SaveSelector) -> Operation {
        Operation(false)
    }
    pub fn sbio_confirm_save(_: &SaveSelector) -> Operation {
        Operation(true)
    }
    pub const SBIO_SAVED_MIN: usize = 0x22;
    pub const SBIO_SAVED_MAX: usize = 0x4b000;
    pub const SBIO_SAVED_USER_ID_AT: usize = 8;
}
fn save(s: &SepData) -> bool {
    s.save_catacomb(
        sbio::CatacombUser::OWNER,
        PRIVATE_TYPE_CATACOMB_USER,
        c"test user",
    )
}
#[test]
fn success_serialization_and_crc_round_trip() {
    let _f = Fixture::new();
    original();
    let b = vec![0x55; 10001];
    catacomb::write(PRIVATE_TYPE_CATACOMB_USER, &b).unwrap();
    assert_eq!(current(), b);
}
#[test]
fn maximum_size_round_trip() {
    let _f = Fixture::new();
    let b = vec![0x89; 1 << 20];
    catacomb::write(PRIVATE_TYPE_CATACOMB_USER, &b).unwrap();
    assert_eq!(current(), b);
}
#[test]
fn oversized_blob_preserves_previous() {
    let _f = Fixture::new();
    original();
    let old = current();
    assert_eq!(
        catacomb::write(PRIVATE_TYPE_CATACOMB_USER, &vec![0; (1 << 20) + 1]),
        Err(ENOSPC)
    );
    assert_eq!(current(), old);
}
#[test]
fn invalid_kind_preserves_previous() {
    let _f = Fixture::new();
    original();
    assert_eq!(catacomb::write(0, &[1]), Err(EINVAL));
    assert_eq!(current(), b"original valid catacomb");
}
#[test]
fn header_error_preserves_valid_previous() {
    let _f = Fixture::new();
    original();
    unsafe { fixture_write_fault(0, 5, 0) };
    assert_eq!(
        catacomb::write(PRIVATE_TYPE_CATACOMB_USER, b"replacement"),
        Err(EIO)
    );
    assert_eq!(current(), b"original valid catacomb");
}
#[test]
fn partial_payload_error_preserves_valid_previous() {
    let _f = Fixture::new();
    original();
    unsafe { fixture_write_fault(18, 5, 0) };
    assert_eq!(
        catacomb::write(PRIVATE_TYPE_CATACOMB_USER, b"replacement"),
        Err(EIO)
    );
    assert_eq!(current(), b"original valid catacomb");
}
#[test]
fn temporary_fsync_error_preserves_valid_previous() {
    let _f = Fixture::new();
    original();
    fault(9, 5);
    assert_eq!(
        catacomb::write(PRIVATE_TYPE_CATACOMB_USER, b"replacement"),
        Err(EIO)
    );
    assert_eq!(current(), b"original valid catacomb");
}
#[test]
fn rename_error_preserves_valid_previous() {
    let _f = Fixture::new();
    original();
    fault(8, 5);
    assert_eq!(
        catacomb::write(PRIVATE_TYPE_CATACOMB_USER, b"replacement"),
        Err(EIO)
    );
    assert_eq!(current(), b"original valid catacomb");
}
#[test]
fn directory_fsync_error_keeps_complete_visible_replacement() {
    let _f = Fixture::new();
    original();
    fault(10, 5);
    assert_eq!(
        catacomb::write(PRIVATE_TYPE_CATACOMB_USER, b"replacement"),
        Err(EIO)
    );
    assert_eq!(current(), b"replacement");
}
#[test]
fn short_write_round_trip() {
    let _f = Fixture::new();
    unsafe { fixture_short(3) };
    catacomb::write(PRIVATE_TYPE_CATACOMB_USER, b"replacement").unwrap();
    assert_eq!(current(), b"replacement");
}
#[test]
fn save_confirms_only_after_durable_replacement() {
    let _f = Fixture::new();
    original();
    let s = SepData::new();
    assert!(save(&s));
    assert_eq!(s.confirms.get(), 1);
    assert_eq!(count(2), 2);
}
#[test]
fn save_with_payload_error_keeps_old_and_never_confirms() {
    let _f = Fixture::new();
    original();
    unsafe { fixture_write_fault(18, 5, 0) };
    let s = SepData::new();
    assert!(!save(&s));
    assert_eq!(s.confirms.get(), 0);
    assert_eq!(current(), b"original valid catacomb");
}
#[test]
fn save_with_file_fsync_error_keeps_old_and_never_confirms() {
    let _f = Fixture::new();
    original();
    fault(9, 5);
    let s = SepData::new();
    assert!(!save(&s));
    assert_eq!(s.confirms.get(), 0);
    assert_eq!(current(), b"original valid catacomb");
}
#[test]
fn save_with_rename_error_keeps_old_and_never_confirms() {
    let _f = Fixture::new();
    original();
    fault(8, 5);
    let s = SepData::new();
    assert!(!save(&s));
    assert_eq!(s.confirms.get(), 0);
    assert_eq!(current(), b"original valid catacomb");
}
#[test]
fn save_with_directory_fsync_error_never_confirms() {
    let _f = Fixture::new();
    original();
    fault(10, 5);
    let s = SepData::new();
    assert!(!save(&s));
    assert_eq!(s.confirms.get(), 0);
    assert_eq!(current().len(), sbio::SBIO_SAVED_MIN);
}
