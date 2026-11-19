// SPDX-License-Identifier: GPL-2.0-only OR MIT
fn init() {
    FREED.with(|v| v.borrow_mut().clear());
    FRAME.with(|v| *v.borrow_mut() = vec![7; 12]);
    RC.with(|v| v.set(0));
    PARTIAL.with(|v| v.set(false));
    ALLOC_FAIL.with(|v| v.set(false));
    RESIZE_FAIL.with(|v| v.set(false));
    CALLS.with(|v| v.set(0));
}
fn wiped() {
    FREED.with(|v| {
        let v = v.borrow();
        assert_eq!(v.len(), 1);
        assert!(
            v[0].iter().all(|b| *b == 0),
            "image-bearing allocation freed without wiping: {:?}",
            v[0]
        );
    });
}
#[test]
fn partial_bus_failure_wipes() {
    init();
    RC.with(|v| v.set(-5));
    PARTIAL.with(|v| v.set(true));
    assert!(matches!(
        read_capture(12),
        Err(CaptureError::Bus(Error(-5)))
    ));
    wiped();
}
#[test]
fn complete_bus_failure_wipes() {
    init();
    RC.with(|v| v.set(-110));
    assert!(matches!(
        read_capture(12),
        Err(CaptureError::Bus(Error(-110)))
    ));
    wiped();
}
#[test]
fn crc_rejection_wipes() {
    init();
    assert!(matches!(
        read_capture(12),
        Err(CaptureError::Checksum { .. })
    ));
    wiped();
}
#[test]
fn success_preserves_until_owner_dropped() {
    init();
    FRAME.with(|f| {
        let mut f = f.borrow_mut();
        let crc = crc16_ansi(&f[..10]);
        f[10..].copy_from_slice(&crc.to_le_bytes());
    });
    let c = match read_capture(12) {
        Ok(c) => c,
        Err(_) => panic!("unexpected error"),
    };
    assert_eq!(&c.bytes()[..10], &[7; 10]);
    FREED.with(|v| assert!(v.borrow().is_empty()));
    drop(c);
    wiped();
}
#[test]
fn zero_length_no_io() {
    init();
    assert!(matches!(read_capture(0), Err(CaptureError::Length(0))));
    CALLS.with(|v| assert_eq!(v.get(), 0));
}
#[test]
fn crc_only_no_io() {
    init();
    assert!(matches!(read_capture(2), Err(CaptureError::Length(2))));
    CALLS.with(|v| assert_eq!(v.get(), 0));
}
#[test]
fn over_max_no_io() {
    init();
    assert!(matches!(
        read_capture(MAX_CAPTURE as u32 + 1),
        Err(CaptureError::Length(_))
    ));
    CALLS.with(|v| assert_eq!(v.get(), 0));
}
#[test]
fn allocation_failure_no_io() {
    init();
    ALLOC_FAIL.with(|v| v.set(true));
    assert!(matches!(read_capture(12), Err(CaptureError::NoMemory)));
    CALLS.with(|v| assert_eq!(v.get(), 0));
    FREED.with(|v| assert!(v.borrow().is_empty()));
}
#[test]
fn resize_failure_no_image_written() {
    init();
    RESIZE_FAIL.with(|v| v.set(true));
    assert!(matches!(read_capture(12), Err(CaptureError::NoMemory)));
    CALLS.with(|v| assert_eq!(v.get(), 0));
    FREED.with(|v| assert_eq!(&*v.borrow(), &[Vec::<u8>::new()]));
}
