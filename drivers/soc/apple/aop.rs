// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple AOP driver
//!
//! Copyright (C) The Asahi Linux Contributors

use core::{
    arch::asm,
    cmp,
    marker::Unpin,
    mem,
    ptr::{self, NonNull},
    slice, //
};

use kernel::{
    bindings,
    c_str,
    device,
    device::{Bound, Core},
    dma::{
        Coherent,
        Device,
        DmaMask, //
    },
    error::{
        from_err_ptr,
        to_result, //
    },
    firmware::Firmware,
    fmt,
    io::{
        io_project,
        Io,
        IoBase,
        IoSysMap,
        Mmio,
        Region, //
    },
    module_platform_driver,
    new_condvar,
    new_mutex,
    of,
    platform,
    prelude::*,
    soc::apple::aop::{
        from_fourcc,
        EPICService,
        FakehidListener,
        ReportListener,
        SourceRing,
        AOP, //
    },
    soc::apple::mailbox,
    soc::apple::rtkit::{
        self,
        ASC_CPU_CONTROL, //
    },
    str::CString,
    sync::{
        aref::ARef,
        atomic::{
            Acquire,
            Atomic,
            Release, //
        },
        Arc,
        ArcBorrow,
        CondVar,
        CondVarTimeoutResult,
        Mutex,
        MutexGuard, //
    },
    time::msecs_to_jiffies,
    types::{
        ForeignOwnable,
        ScopeGuard, //
    },
    workqueue::{
        impl_has_work,
        new_work,
        OwnedQueue,
        Work,
        WorkItem, //
    }, //
};

const AOP_MAX_CALLS: usize = 8;
const AOP_MMIO_SIZE: usize = 0x1e0000;
const ASC_MMIO_SIZE: usize = 0x4000;
const BOOTARGS_OFFSET: usize = 0x22c;
const BOOTARGS_SIZE: usize = 0x230;
const AFK_ENDPOINT_START: u8 = 0x20;
const AFK_ENDPOINT_COUNT: u8 = 0xf;
const AFK_OPC_GET_BUF: u64 = 0x89;
const AFK_OPC_INIT: u64 = 0x80;
const AFK_OPC_INIT_RX: u64 = 0x8b;
const AFK_OPC_INIT_TX: u64 = 0x8a;
const AFK_OPC_INIT_UNK: u64 = 0x8c;
const AFK_OPC_SEND: u64 = 0xa2;
const AFK_OPC_START_ACK: u64 = 0x86;
const AFK_OPC_SHUTDOWN_ACK: u64 = 0xc1;
const AFK_OPC_RECV: u64 = 0x85;
const AFK_MSG_GET_BUF_ACK: u64 = 0xa1 << 48;
const AFK_MSG_INIT: u64 = AFK_OPC_INIT << 48;
const AFK_MSG_INIT_ACK: u64 = 0xa0 << 48;
const AFK_MSG_START: u64 = 0xa3 << 48;
const AFK_MSG_SHUTDOWN: u64 = 0xc0 << 48;
const AFK_RB_BLOCK_STEP: usize = 0x40;
const EPIC_TYPE_NOTIFY: u32 = 0;
const EPIC_CATEGORY_REPORT: u8 = 0x00;
const EPIC_CATEGORY_NOTIFY: u8 = 0x10;
const EPIC_CATEGORY_REPLY: u8 = 0x20;
const EPIC_SUBTYPE_STD_SERVICE: u16 = 0xc0;
const EPIC_SUBTYPE_FAKEHID_REPORT: u16 = 0xc4;
const EPIC_SUBTYPE_RETCODE: u16 = 0x84;
const EPIC_SUBTYPE_RETCODE_PAYLOAD: u16 = 0xa0;
const EPIC_SUBTYPE_STRING: u16 = 0x8a;
const QE_MAGIC1: u32 = from_fourcc(b" POI");
const QE_MAGIC2: u32 = from_fourcc(b" POA");
/// Bound on the wait for the reply to an EPIC call. The firmware can stop
/// answering, and a caller must not be left in D state forever.
const EPIC_CALL_TIMEOUT_MS: u32 = 5000;
/// Bound on the wait for an endpoint's shutdown acknowledgment.
const AFK_SHUTDOWN_TIMEOUT_MS: u32 = 5000;

// The T8140 AOP boots through a second mailbox, its "setup port", besides the
// RTKit one. Its management endpoint runs a HELLO / endpoint map / power
// handshake, after which five service endpoints each request a host-to-AOP
// message page and an AOP-to-host reply window, which the host hands out of
// an arena it maps through the setup mailbox's own DART stream. Messages are
// a 64-bit word plus the endpoint number.
const SETUP_PAGE: usize = 0x4000;
/// Five message pages plus the reply windows: the endpoints request either
/// 0x400 or 0x1000 16-byte reply entries, one or four pages, and the five of
/// them take eight pages in total.
const SETUP_ARENA_PAGES: usize = 13;
const SETUP_MGMT_EP: u8 = 0;
/// Management message types, in bits 52..60 of the word.
const SETUP_TYPE_HELLO: u64 = 1;
const SETUP_TYPE_HELLO_REPLY: u64 = 2;
const SETUP_TYPE_UNK3: u64 = 3;
const SETUP_TYPE_UNK3_REPLY: u64 = 4;
const SETUP_TYPE_PWR_ACK: u64 = 7;
const SETUP_TYPE_EPMAP: u64 = 8;
const SETUP_TYPE_AP_PWR: u64 = 0xb;
const SETUP_HELLO_VERSION: u64 = 0xc000c;
const SETUP_EPMAP_LAST: u64 = 1 << 51;
const SETUP_AP_PWR_INIT: u64 = 0x220;
const SETUP_AP_PWR_ON: u64 = 0x20;
/// A service endpoint's buffer request: the type in bits 56..64, the number
/// of reply entries in the low 32 bits.
const SETUP_BUFFER_REQUEST: u64 = 0x12;
const SETUP_BUFFER_REQUEST_ACK: u64 = SETUP_BUFFER_REQUEST << 56;
const SETUP_BUFFER_ENTRY_SIZE: usize = 16;
/// The service endpoints that request buffers; boot completes once all of
/// them have theirs.
const SETUP_ENDPOINTS: [u8; 5] = [0x20, 0x21, 0x23, 0x30, 0x32];
const SETUP_BOOT_TIMEOUT_MS: u32 = 15000;
// A request to a service endpoint: the payload goes into the endpoint's
// message page, (3 << 60) | length announces it, the firmware answers with a
// result word after writing a status byte at the start of the reply window,
// and 4 << 60 releases the request.
const SETUP_REQUEST_TYPE: u64 = 3 << 60;
const SETUP_REQUEST_DONE: u64 = 4 << 60;
/// The result word of a completed request.
const SETUP_REPLY_READY: u64 = 0x2000000000000001;
/// The result word the ALS calibration is answered with instead.
const SETUP_REPLY_READY_ALT: u64 = 0x2000000000000004;
const SETUP_REPLY_TIMEOUT_MS: u32 = 5000;
/// The ambient light sensor's setup-port endpoint. The firmware does not
/// start the sensor until its calibration has been sent here.
const SETUP_ALS_EP: u8 = 0x21;
/// The low-power microphone's setup-port endpoint, which takes the source
/// ring binding: a 24-byte request of this operation code, the ring's IOVA
/// and its size.
const SETUP_SOURCE_EP: u8 = 0x20;
const SETUP_SET_SOURCE_BUFFER: u64 = 0x6b80_3ce4_92bd_9547;
/// The calibration is an 80-byte message: a 64-bit operation code, a 64-bit
/// body length of 56, and 64 bytes of calibration data sent unchanged (the
/// last eight lie beyond the stated body length).
const ALS_CALIBRATION_LEN: usize = 80;
const ALS_CALIBRATION_OPERATION: u64 = 0x0746_b8d6_6515_2e31;
const ALS_CALIBRATION_BODY_LEN: u64 = 56;

mod aop_ring;

/// Index into the AFK endpoint tables for RTKit endpoint `ep`, if it is one
/// of the endpoints this driver drives.
fn afk_endpoint_index(ep: u8) -> Option<usize> {
    ep.checked_sub(AFK_ENDPOINT_START)
        .filter(|i| *i < AFK_ENDPOINT_COUNT)
        .map(usize::from)
}

/// Reads the little-endian `u16` at `off`; `b` must hold it.
fn le_u16(b: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([b[off], b[off + 1]])
}

/// Reads the little-endian `u32` at `off`; `b` must hold it.
fn le_u32(b: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([b[off], b[off + 1], b[off + 2], b[off + 3]])
}

/// Reads the little-endian `u64` at `off`; `b` must hold it.
fn le_u64(b: &[u8], off: usize) -> u64 {
    u64::from(le_u32(b, off)) | u64::from(le_u32(b, off + 4)) << 32
}

#[inline(always)]
fn mem_sync() {
    // SAFETY: a full-system barrier touches no memory or registers of ours.
    unsafe {
        asm!("dsb sy");
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct QEHeader {
    magic: u32,
    size: u32,
    channel: u32,
    ty: u32,
}

/// The size of a ring entry header.
const QEH_SIZE: usize = mem::size_of::<QEHeader>();

impl QEHeader {
    fn from_bytes(b: &[u8; QEH_SIZE]) -> QEHeader {
        QEHeader {
            magic: le_u32(b, 0),
            size: le_u32(b, 4),
            channel: le_u32(b, 8),
            ty: le_u32(b, 12),
        }
    }

    fn to_bytes(self) -> [u8; QEH_SIZE] {
        let mut b = [0; QEH_SIZE];
        b[0..4].copy_from_slice(&{ self.magic }.to_le_bytes());
        b[4..8].copy_from_slice(&{ self.size }.to_le_bytes());
        b[8..12].copy_from_slice(&{ self.channel }.to_le_bytes());
        b[12..16].copy_from_slice(&{ self.ty }.to_le_bytes());
        b
    }
}

#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct EPICHeader {
    version: u8,
    seq: u16,
    _pad0: u8,
    _unk0: u32,
    timestamp: u64,
    // Subheader
    length: u32,
    sub_version: u8,
    category: u8,
    subtype: u16,
    tag: u16,
    _unk1: u16,
    _pad1: u64,
    inline_len: u32,
}

/// The EPIC header of the T8140 firmware (sub-header version 4): the
/// sub-header carries a timestamp ahead of the tag, and the reply capacity
/// of a call goes into `inline_len`. Same size as the version 2 header.
#[repr(C, packed)]
#[derive(Clone, Copy, Default)]
struct EPICHeaderV4 {
    version: u8,
    seq: u16,
    _pad0: u8,
    _unk0: u32,
    timestamp: u64,
    // Subheader
    length: u32,
    sub_version: u8,
    category: u8,
    subtype: u16,
    sub_timestamp: u64,
    tag: u16,
    _unk1: u16,
    inline_len: u32,
}

const _: () = assert!(mem::size_of::<EPICHeaderV4>() == mem::size_of::<EPICHeader>());

/// The EPIC header fields that message dispatch needs.
struct EPICHeaderFields {
    category: u8,
    subtype: u16,
    tag: u16,
}

impl EPICHeaderFields {
    /// Splits `msg` into its decoded header and its payload, or returns `None`
    /// when `msg` is too short to hold a header. The sub-header version in the
    /// message selects the layout; both layouts share the leading fields.
    fn decode(msg: &[u8]) -> Option<(EPICHeaderFields, &[u8])> {
        if msg.len() < mem::size_of::<EPICHeader>() {
            return None;
        }
        let (hdr, data) = msg.split_at(mem::size_of::<EPICHeader>());
        let tag_offset = if hdr[mem::offset_of!(EPICHeader, sub_version)] >= 4 {
            mem::offset_of!(EPICHeaderV4, tag)
        } else {
            mem::offset_of!(EPICHeader, tag)
        };
        let fields = EPICHeaderFields {
            category: hdr[mem::offset_of!(EPICHeader, category)],
            subtype: le_u16(hdr, mem::offset_of!(EPICHeader, subtype)),
            tag: le_u16(hdr, tag_offset),
        };
        Some((fields, data))
    }
}

/// A service announcement (report subtype [`EPIC_SUBTYPE_STD_SERVICE`]):
/// a NUL-padded 32-byte service name followed by the channel the service
/// answers on. Its trailing 8 bytes are not used here.
const EPIC_ANNOUNCE_NAME_LEN: usize = 32;
const EPIC_ANNOUNCE_CHANNEL_OFFSET: usize = 32;
const EPIC_ANNOUNCE_LEN: usize = 44;

#[pin_data]
struct FutureValue<T> {
    #[pin]
    val: Mutex<Option<T>>,
    #[pin]
    completion: CondVar,
}

impl<T: Unpin> FutureValue<T> {
    fn pin_init() -> impl PinInit<FutureValue<T>> {
        pin_init!(
            FutureValue {
                val <- new_mutex!(None),
                completion <- new_condvar!()
            }
        )
    }
    fn complete(&self, val: T) {
        *self.val.lock() = Some(val);
        self.completion.notify_all();
    }
    /// Waits at most `timeout_ms` for the value; `None` if it did not arrive.
    ///
    /// The wait is uninterruptible: a firmware transaction must not be
    /// abandoned because the calling task has a signal pending, or its reply
    /// would arrive for a call that no longer exists.
    fn wait_timeout(&self, timeout_ms: u32) -> Option<T> {
        let mut ret_guard = self.val.lock();
        let mut left = msecs_to_jiffies(timeout_ms);
        while ret_guard.is_none() {
            match self.completion.wait_timeout(&mut ret_guard, left) {
                CondVarTimeoutResult::Timeout => break,
                CondVarTimeoutResult::Woken { jiffies }
                | CondVarTimeoutResult::Signal { jiffies } => left = jiffies,
            }
        }
        ret_guard.take()
    }
    fn reset(&self) {
        *self.val.lock() = None;
    }
}

struct AFKRingBuffer {
    offset: usize,
    block_size: usize,
    buf_size: usize,
}

struct CallResult {
    retcode: u32,
    extra_data: Option<KVec<u8>>,
}

/// An EPIC call in flight on an endpoint. The slot index plus one is the tag
/// the firmware echoes in the reply.
enum CallSlot {
    /// The caller waits for the reply; the buffer receives the reply payload.
    Pending(Arc<FutureValue<CallResult>>, Option<KVec<u8>>),
    /// The caller gave up waiting. The slot stays reserved, so that its tag is
    /// not handed to a later call that the late reply would then complete,
    /// until that reply arrives and is dropped.
    Abandoned,
}

struct AFKEndpoint {
    index: u8,
    /// The AFK handshake was started; only such endpoints are shut down.
    started: bool,
    iomem: Option<Coherent<[u8]>>,
    txbuf: Option<AFKRingBuffer>,
    rxbuf: Option<AFKRingBuffer>,
    seq: u16,
    calls: [Option<CallSlot>; AOP_MAX_CALLS],
    /// Messages this endpoint could not handle, for throttling their logging.
    dropped: u32,
}

impl AFKEndpoint {
    fn new(index: u8) -> AFKEndpoint {
        AFKEndpoint {
            index,
            started: false,
            iomem: None,
            txbuf: None,
            rxbuf: None,
            seq: 0,
            calls: [const { None }; AOP_MAX_CALLS],
            dropped: 0,
        }
    }

    fn start(&self, rtkit: Pin<&mut rtkit::RtKit<AopData>>) -> Result<()> {
        rtkit.send_message(self.index, AFK_MSG_INIT)
    }

    fn stop(&self, rtkit: Pin<&mut rtkit::RtKit<AopData>>) -> Result<()> {
        rtkit.send_message(self.index, AFK_MSG_SHUTDOWN)
    }

    fn recv_message(
        &mut self,
        client: ArcBorrow<'_, AopData>,
        rtkit: Pin<&mut rtkit::RtKit<AopData>>,
        msg: u64,
    ) -> Result<()> {
        let opc = msg >> 48;
        match opc {
            AFK_OPC_INIT => {
                rtkit.send_message(self.index, AFK_MSG_INIT_ACK)?;
            }
            AFK_OPC_GET_BUF => {
                self.recv_get_buf(client.dev.clone(), rtkit, msg)?;
            }
            AFK_OPC_INIT_UNK => {} // no-op
            AFK_OPC_START_ACK => {}
            AFK_OPC_INIT_RX => {
                if self.rxbuf.is_some() {
                    dev_err!(
                        client.dev,
                        "Got InitRX message with existing rxbuf at endpoint {}",
                        self.index
                    );
                    return Err(EIO);
                }
                self.rxbuf = Some(self.parse_ring_buf(msg)?);
                if self.txbuf.is_some() {
                    rtkit.send_message(self.index, AFK_MSG_START)?;
                }
            }
            AFK_OPC_INIT_TX => {
                if self.txbuf.is_some() {
                    dev_err!(
                        client.dev,
                        "Got InitTX message with existing txbuf at endpoint {}",
                        self.index
                    );
                    return Err(EIO);
                }
                self.txbuf = Some(self.parse_ring_buf(msg)?);
                if self.rxbuf.is_some() {
                    rtkit.send_message(self.index, AFK_MSG_START)?;
                }
            }
            AFK_OPC_RECV => {
                self.recv_rb(client)?;
            }
            AFK_OPC_SHUTDOWN_ACK => {
                client.shutdown_complete(self.index);
            }
            _ => dev_err!(
                client.dev,
                "AFK endpoint {} got unknown message {}",
                self.index,
                msg
            ),
        }
        Ok(())
    }

    /// Decodes an InitRX/InitTX message: a ring of `size` bytes at `offset`
    /// in the shared buffer, laid out as three header blocks (buffer size,
    /// read pointer, write pointer) followed by `buf_size` bytes of entries.
    fn parse_ring_buf(&self, msg: u64) -> Result<AFKRingBuffer> {
        let msg = msg as usize;
        let size = ((msg >> 16) & 0xFFFF) * AFK_RB_BLOCK_STEP;
        let offset = ((msg >> 32) & 0xFFFF) * AFK_RB_BLOCK_STEP;
        let iomem_size = self.iomem.as_ref().ok_or(ENXIO)?.size();
        let buf_size = self.iomem_read32(offset)? as usize;
        let block_size = size.checked_sub(buf_size).ok_or(EIO)? / 3;
        let end = offset.checked_add(size).ok_or(EIO)?;
        if end > iomem_size || buf_size == 0 || block_size == 0 || !block_size.is_power_of_two() {
            return Err(EIO);
        }
        Ok(AFKRingBuffer {
            offset,
            block_size,
            buf_size,
        })
    }

    /// Returns a pointer to `len` bytes at `off` in the shared buffer after
    /// checking that they lie inside it.
    fn iomem_ptr(&self, off: usize, len: usize) -> Result<*mut u8> {
        let iomem = self.iomem.as_ref().ok_or(ENXIO)?;
        let end = off.checked_add(len).ok_or(EIO)?;
        if end > iomem.size() {
            return Err(EIO);
        }
        // SAFETY: `off + len` does not exceed the size of the allocation, so
        // the offset pointer stays inside it.
        Ok(unsafe { iomem.as_mut_ptr().cast::<u8>().add(off) })
    }

    /// Writes one of the ring's pointer words. The firmware polls the word
    /// concurrently, so the write is volatile.
    fn iomem_write32(&mut self, off: usize, data: u32) -> Result<()> {
        if off % mem::align_of::<u32>() != 0 {
            return Err(EIO);
        }
        let ptr = self.iomem_ptr(off, mem::size_of::<u32>())?;
        // SAFETY: `ptr` is valid for four bytes and aligned for a `u32`. A
        // volatile write is the kernel's WRITE_ONCE() for a word the device
        // reads at any time.
        unsafe { ptr.cast::<u32>().write_volatile(data.to_le()) };
        Ok(())
    }

    /// Reads one of the ring's pointer words. The firmware updates the word
    /// concurrently, so the read is volatile.
    fn iomem_read32(&self, off: usize) -> Result<u32> {
        if off % mem::align_of::<u32>() != 0 {
            return Err(EIO);
        }
        let ptr = self.iomem_ptr(off, mem::size_of::<u32>())?;
        // SAFETY: `ptr` is valid for four bytes and aligned for a `u32`. A
        // volatile read is the kernel's READ_ONCE() for a word the device
        // writes at any time.
        Ok(u32::from_le(unsafe { ptr.cast::<u32>().read_volatile() }))
    }

    /// Copies a ring entry out of the shared buffer. Callers only read
    /// entries between the read and the write pointer, which the firmware
    /// has finished writing and does not touch again until the read pointer
    /// passes them.
    fn memcpy_from_iomem(&self, off: usize, target: &mut [u8]) -> Result<()> {
        let src = self.iomem_ptr(off, target.len())?;
        // SAFETY: `src` is valid for `target.len()` bytes, `target` is a
        // distinct allocation, and by the ring protocol (see above) the device
        // does not write the entry while it is copied.
        unsafe { ptr::copy_nonoverlapping(src, target.as_mut_ptr(), target.len()) };
        Ok(())
    }

    /// Copies a ring entry into the shared buffer. Callers only write between
    /// the write and the read pointer, which the firmware does not read until
    /// the write pointer is advanced past the entry.
    fn memcpy_to_iomem(&mut self, off: usize, src: &[u8]) -> Result<()> {
        let dst = self.iomem_ptr(off, src.len())?;
        // SAFETY: `dst` is valid for `src.len()` bytes, `src` is a distinct
        // allocation, and by the ring protocol (see above) the device does
        // not access the entry while it is written.
        unsafe { ptr::copy_nonoverlapping(src.as_ptr(), dst, src.len()) };
        Ok(())
    }

    fn recv_get_buf(
        &mut self,
        dev: ARef<device::Device>,
        rtkit: Pin<&mut rtkit::RtKit<AopData>>,
        msg: u64,
    ) -> Result<()> {
        let size = ((msg & 0xFFFF0000) >> 16) as usize * AFK_RB_BLOCK_STEP;
        if self.iomem.is_some() {
            dev_err!(
                dev,
                "Got GetBuf message with existing buffer on endpoint {}",
                self.index
            );
            return Err(EIO);
        }
        // SAFETY: This runs from the RTKit receive worker. The RTKit handle
        // is dropped by `AopData::remove()`, which runs from unbind or from a
        // failed probe while the device is still bound, and no callback runs
        // once that drop has returned. So the device is bound for as long as
        // this callback runs.
        let bound_dev = unsafe { dev.as_bound() };
        let iomem = Coherent::<u8>::zeroed_slice(bound_dev, size, GFP_KERNEL)?;
        let iova = iomem.dma_handle();
        // Own the buffer before its address leaves the host: should the
        // doorbell fail after the firmware has seen the message, the buffer
        // must not go back to the allocator while the firmware uses it.
        self.iomem = Some(iomem);
        rtkit.send_message(self.index, AFK_MSG_GET_BUF_ACK | iova)?;
        Ok(())
    }

    fn recv_rb(&mut self, client: ArcBorrow<'_, AopData>) -> Result<()> {
        let (buf_offset, block_size, buf_size) = match self.rxbuf.as_ref() {
            Some(b) => (b.offset, b.block_size, b.buf_size),
            None => {
                dev_err!(
                    client.dev,
                    "Got Recv message with no rxbuf at endpoint {}",
                    self.index
                );
                return Err(EIO);
            }
        };
        let mut rptr = self.iomem_read32(buf_offset + block_size)? as usize;
        let mut wptr = self.iomem_read32(buf_offset + block_size * 2)? as usize;
        mem_sync();
        let base = buf_offset + block_size * 3;
        let mut msg_buf = KVec::new();
        let ring = match aop_ring::Ring::new(buf_size, block_size) {
            Ok(ring) => ring,
            Err(e) => {
                dev_err!(
                    client.dev,
                    "Invalid ring on ep {}: size {:#x} block {:#x}",
                    self.index,
                    buf_size,
                    block_size
                );
                return Err(e);
            }
        };
        // Entries the producer adds while this pass runs beyond one ring's
        // worth are left for the doorbell that announces them.
        let mut budget = ring.capacity();
        loop {
            if ring.check_ptr(rptr).is_err() || ring.check_ptr(wptr).is_err() {
                dev_err!(
                    client.dev,
                    "Ring pointers out of range on ep {}: rptr {:#x} wptr {:#x}",
                    self.index,
                    rptr,
                    wptr
                );
                return Err(EIO);
            }
            if wptr == rptr || budget == 0 {
                break;
            }
            budget -= 1;
            let mut last = QEHeader::default();
            let read = ring.read_entry(rptr, |ptr| {
                let mut qeh_bytes = [0; QEH_SIZE];
                self.memcpy_from_iomem(base + ptr, &mut qeh_bytes)?;
                last = QEHeader::from_bytes(&qeh_bytes);
                Ok(last)
            });
            let Ok((qeh, entry, next)) = read else {
                let (magic, size) = (last.magic, last.size);
                dev_err!(
                    client.dev,
                    "Invalid entry on ep {} at {:#x}: magic {:x} size {:#x}",
                    self.index,
                    rptr,
                    magic,
                    size
                );
                return Err(EIO);
            };
            msg_buf.resize(qeh.size as usize, 0, GFP_KERNEL)?;
            self.memcpy_from_iomem(base + entry + QEH_SIZE, &mut msg_buf)?;
            // A message the endpoint cannot handle is skipped. The ring
            // position is still good, and not advancing past the entry would
            // read it again on every doorbell and stall the endpoint for good.
            match EPICHeaderFields::decode(&msg_buf) {
                None => {
                    self.drop_message(&client.dev, fmt!("short message ({} bytes)", msg_buf.len()))
                }
                Some((header, msg)) => {
                    if let Err(e) = self.handle_ipc(client, &qeh, &header, msg) {
                        let channel = qeh.channel;
                        self.drop_message(
                            &client.dev,
                            fmt!(
                                "category {:#x} subtype {:#x} tag {} on channel {}: {:?}",
                                header.category,
                                header.subtype,
                                header.tag,
                                channel,
                                e
                            ),
                        );
                    }
                }
            }
            rptr = next;
            mem_sync();
            self.iomem_write32(buf_offset + block_size, rptr as u32)?;
            wptr = self.iomem_read32(buf_offset + block_size * 2)? as usize;
            mem_sync();
        }
        Ok(())
    }
    /// Counts a message the endpoint could not handle and logs it. The log
    /// is throttled to the powers of two of the count, so that a stream of
    /// such messages cannot flood it.
    fn drop_message(&mut self, dev: &device::Device, why: fmt::Arguments<'_>) {
        self.dropped = self.dropped.saturating_add(1);
        if self.dropped.is_power_of_two() {
            dev_warn!(
                dev,
                "Endpoint {:#04x} dropped a message: {} ({} so far)",
                self.index,
                why,
                self.dropped
            );
        }
    }
    /// Dispatches one message. An error means the message was not handled;
    /// the caller logs it and skips the message.
    fn handle_ipc(
        &mut self,
        client: ArcBorrow<'_, AopData>,
        qhdr: &QEHeader,
        ehdr: &EPICHeaderFields,
        data: &[u8],
    ) -> Result<()> {
        let subtype = ehdr.subtype;
        if ehdr.category == EPIC_CATEGORY_REPORT {
            if subtype == EPIC_SUBTYPE_STD_SERVICE {
                if data.len() < EPIC_ANNOUNCE_LEN {
                    return Err(EMSGSIZE);
                }
                let name = &data[..EPIC_ANNOUNCE_NAME_LEN];
                let name = &name[..name.iter().position(|x| *x == 0).unwrap_or(name.len())];
                let chan = le_u32(data, EPIC_ANNOUNCE_CHANNEL_OFFSET);
                return Into::<Arc<_>>::into(client).register_service(self, chan, name);
            } else if subtype == EPIC_SUBTYPE_FAKEHID_REPORT {
                return client.process_fakehid_report(self, qhdr.channel, data);
            } else if client.process_report(self, qhdr.channel, subtype, data)? {
                return Ok(());
            }
            // A report nobody listens for.
            return Err(ENOENT);
        } else if ehdr.category == EPIC_CATEGORY_REPLY {
            if subtype == EPIC_SUBTYPE_RETCODE_PAYLOAD
                || subtype == EPIC_SUBTYPE_RETCODE
                || subtype == EPIC_SUBTYPE_STRING
            {
                if data.len() < mem::size_of::<u32>() {
                    return Err(EMSGSIZE);
                }
                let retcode = le_u32(data, 0);
                let tag = ehdr.tag as usize;
                let slot = match tag.checked_sub(1) {
                    Some(slot) if slot < self.calls.len() && self.calls[slot].is_some() => slot,
                    // The version 4 firmware does not echo the tag. Calls on
                    // such an endpoint are serialized, and none is started
                    // while an abandoned one is outstanding, so at most one
                    // slot is in use and the reply belongs to it.
                    _ if client.epic_v4 => {
                        self.calls.iter().position(|c| c.is_some()).ok_or(ENOENT)?
                    }
                    _ if tag == 0 || tag > self.calls.len() => return Err(EINVAL),
                    _ => return Err(ENOENT),
                };
                let tag = slot + 1;
                let (future, ret) = match self.calls[slot].take() {
                    Some(CallSlot::Pending(future, ret)) => (future, ret),
                    Some(CallSlot::Abandoned) => {
                        // The late reply to a call that timed out; its slot
                        // is free again.
                        dev_warn!(
                            client.dev,
                            "Late reply for tag {} on endpoint {}",
                            tag,
                            self.index
                        );
                        return Ok(());
                    }
                    None => return Err(ENOENT),
                };
                let extra_data = ret.map(|mut ret| {
                    let len = cmp::min(data.len() - 4, ret.len());
                    ret[..len].copy_from_slice(&data[4..(len + 4)]);
                    ret.truncate(len);
                    ret
                });
                future.complete(CallResult {
                    retcode,
                    extra_data,
                });

                return Ok(());
            }
            return Err(EINVAL);
        }
        Err(EINVAL)
    }
    /// Writes one entry into the transmit ring and returns the doorbell
    /// message that announces it. Nothing is visible to the firmware until
    /// that message is sent.
    fn write_entry(
        &mut self,
        client: &AopData,
        channel: u32,
        ty: u32,
        header: &[u8],
        data: &[u8],
    ) -> Result<u64> {
        let (buf_offset, block_size, buf_size) = match self.txbuf.as_ref() {
            Some(b) => (b.offset, b.block_size, b.buf_size),
            None => {
                dev_err!(
                    client.dev,
                    "Attempting to send message with no txbuf at endpoint {}",
                    self.index
                );
                return Err(EIO);
            }
        };
        let base = buf_offset + block_size * 3;
        mem_sync();
        let rptr = self.iomem_read32(buf_offset + block_size)? as usize;
        let wptr = self.iomem_read32(buf_offset + block_size * 2)? as usize;
        let ring = aop_ring::Ring::new(buf_size, block_size)?;
        if ring.check_ptr(rptr).is_err() || ring.check_ptr(wptr).is_err() {
            dev_err!(
                client.dev,
                "Tx ring pointers out of range at endpoint {}: rptr {:#x} wptr {:#x}",
                self.index,
                rptr,
                wptr
            );
            return Err(EIO);
        }
        let payload_len = header.len().checked_add(data.len()).ok_or(EIO)?;
        let Ok((entry, next)) = ring.write_entry(rptr, wptr, payload_len) else {
            dev_err!(client.dev, "Tx buffer full at endpoint {}", self.index);
            return Err(EIO);
        };
        let qeh = QEHeader {
            magic: QE_MAGIC1,
            size: u32::try_from(payload_len).map_err(|_| EIO)?,
            channel,
            ty,
        };
        let qeh_bytes = qeh.to_bytes();
        if entry != wptr {
            self.memcpy_to_iomem(base + wptr, &qeh_bytes)?;
        }
        self.memcpy_to_iomem(base + entry, &qeh_bytes)?;
        self.memcpy_to_iomem(base + entry + QEH_SIZE, header)?;
        self.memcpy_to_iomem(base + entry + QEH_SIZE + header.len(), data)?;
        mem_sync();
        self.iomem_write32(buf_offset + block_size * 2, next as u32)?;
        Ok(next as u64 | (AFK_OPC_SEND << 48))
    }
    fn epic_notify(
        &mut self,
        client: &AopData,
        rtkit: Pin<&mut rtkit::RtKit<AopData>>,
        channel: u32,
        subtype: u16,
        data: &[u8],
        ret: Option<KVec<u8>>,
    ) -> Result<Arc<FutureValue<CallResult>>> {
        if client.epic_v4
            && self
                .calls
                .iter()
                .any(|c| matches!(c, Some(CallSlot::Abandoned)))
        {
            // Replies carry no tag here: until the reply to the abandoned call
            // has arrived, a new call's reply could not be told from it.
            dev_dbg!(
                client.dev,
                "Endpoint {:#04x} waits for a late reply, call refused",
                self.index
            );
            return Err(EIO);
        }
        let Some(slot) = self.calls.iter().position(|c| c.is_none()) else {
            dev_err!(
                client.dev,
                "Too many inflight calls on endpoint {}",
                self.index
            );
            return Err(EIO);
        };
        let call = Arc::pin_init(FutureValue::pin_init(), GFP_KERNEL)?;
        let tag = (slot + 1) as u16;
        let hdr_v2 = EPICHeader {
            version: 2,
            seq: self.seq,
            length: data.len() as u32,
            sub_version: 2,
            category: EPIC_CATEGORY_NOTIFY,
            subtype,
            tag,
            ..EPICHeader::default()
        };
        // The version 4 firmware wants the reply capacity advertised; a
        // property read is answered even with zero, other calls may not be.
        let hdr_v4 = EPICHeaderV4 {
            version: 2,
            seq: self.seq,
            length: data.len() as u32,
            sub_version: 4,
            category: EPIC_CATEGORY_NOTIFY,
            subtype,
            tag,
            inline_len: ret.as_ref().map_or(0, |ret| ret.len() as u32),
            ..EPICHeaderV4::default()
        };
        // SAFETY: Both headers are packed plain-data structs that outlive the
        // slice, which covers exactly the bytes of the one selected.
        let hdr_bytes = unsafe {
            if client.epic_v4 {
                slice::from_raw_parts(
                    ptr::from_ref(&hdr_v4).cast::<u8>(),
                    mem::size_of::<EPICHeaderV4>(),
                )
            } else {
                slice::from_raw_parts(
                    ptr::from_ref(&hdr_v2).cast::<u8>(),
                    mem::size_of::<EPICHeader>(),
                )
            }
        };
        let doorbell = self.write_entry(client, channel, EPIC_TYPE_NOTIFY, hdr_bytes, data)?;
        self.seq = self.seq.wrapping_add(1);
        self.calls[slot] = Some(CallSlot::Pending(call.clone(), ret));
        if let Err(e) = rtkit.send_message(self.index, doorbell) {
            // The entry is in the ring and the next doorbell delivers it, so
            // its reply still arrives; keep the tag reserved until it does.
            self.calls[slot] = Some(CallSlot::Abandoned);
            return Err(e);
        }
        Ok(call)
    }
    /// Gives up on `call`: its slot stays reserved until the reply arrives.
    fn abandon_call(&mut self, call: &Arc<FutureValue<CallResult>>) {
        for slot in self.calls.iter_mut() {
            if let Some(CallSlot::Pending(pending, _)) = slot {
                if Arc::ptr_eq(pending, call) {
                    *slot = Some(CallSlot::Abandoned);
                    return;
                }
            }
        }
    }
}

struct ListenerEntry {
    svc: EPICService,
    listener: Arc<dyn FakehidListener>,
}

struct ReportListenerEntry {
    svc: EPICService,
    subtype: u16,
    listener: Arc<dyn ReportListener>,
}

/// The source ring and whether the firmware is known to have taken it.
struct SourceRingBinding {
    ring: Arc<SourceRing>,
    /// The bind request completed. Until then the firmware may or may not
    /// hold the ring, so it is neither freed nor bound again.
    confirmed: bool,
}

/// One of the setup port's service endpoints, once it has its buffers: a
/// host-to-AOP message page and an AOP-to-host reply window in the arena.
#[derive(Clone, Copy)]
struct SetupEndpoint {
    ep: u8,
    tx_iova: u64,
    rx_iova: u64,
    /// A request is outstanding; the endpoint takes one at a time.
    busy: bool,
    /// The outstanding request timed out. The endpoint stays busy until the
    /// reply arrives, so that it cannot be taken for a later request's.
    abandoned: bool,
    /// The last word received on the endpoint that was not a buffer request.
    reply: Option<u64>,
}

struct SetupState {
    /// `None` until the port is opened and again once it is closed.
    mbox: Option<mailbox::Mailbox<SetupPortCallback>>,
    arena: Option<Coherent<[u8]>>,
    /// The next unassigned page of the arena.
    next_page: usize,
    endpoints: [Option<SetupEndpoint>; SETUP_ENDPOINTS.len()],
    map_done: bool,
    ap_ready: bool,
    power_sent: bool,
    /// The protocol broke down; waiters give up instead of timing out.
    failed: bool,
}

impl SetupState {
    fn new() -> SetupState {
        SetupState {
            mbox: None,
            arena: None,
            next_page: 0,
            endpoints: [None; SETUP_ENDPOINTS.len()],
            map_done: false,
            ap_ready: false,
            power_sent: false,
            failed: false,
        }
    }

    fn endpoint_mut(&mut self, ep: u8) -> Option<&mut SetupEndpoint> {
        self.endpoints.iter_mut().flatten().find(|e| e.ep == ep)
    }

    fn endpoint_count(&self) -> usize {
        self.endpoints.iter().flatten().count()
    }

    fn arena_iova(&self) -> Result<u64> {
        Ok(self.arena.as_ref().ok_or(ENXIO)?.dma_handle())
    }

    /// Returns a pointer to `len` bytes at `off` in the arena after checking
    /// that they lie inside it.
    fn arena_ptr(&self, off: usize, len: usize) -> Result<*mut u8> {
        let arena = self.arena.as_ref().ok_or(ENXIO)?;
        let end = off.checked_add(len).ok_or(EINVAL)?;
        if end > arena.size() {
            return Err(EINVAL);
        }
        // SAFETY: `off + len` does not exceed the size of the allocation, so
        // the offset pointer stays inside it.
        Ok(unsafe { arena.as_mut_ptr().cast::<u8>().add(off) })
    }

    /// The offset of an endpoint buffer in the arena.
    fn arena_offset(&self, iova: u64) -> Result<usize> {
        let off = iova.checked_sub(self.arena_iova()?).ok_or(EINVAL)?;
        usize::try_from(off).map_err(|_| EINVAL)
    }

    /// Boot is complete once the AP power state is on and every service
    /// endpoint has its buffers.
    fn ready(&self) -> bool {
        self.ap_ready && self.endpoint_count() == SETUP_ENDPOINTS.len()
    }

    /// Sends one word to a setup-port endpoint. Callers run in process
    /// context, so the send may sleep for room in the mailbox FIFO.
    fn send(&self, ep: u8, word: u64) -> Result<()> {
        let mbox = self.mbox.as_ref().ok_or(ENXIO)?;
        mbox.send(
            mailbox::Message {
                msg0: word,
                msg1: u32::from(ep),
            },
            false,
        )
    }

    /// The word that hands a buffer to the firmware:
    /// (5 << 60) | (host_to_aop << 54) | (pages << 48) | (iova >> 4).
    fn shared_descriptor(iova: u64, pages: usize, host_to_aop: bool) -> u64 {
        (5u64 << 60) | (u64::from(host_to_aop) << 54) | ((pages as u64) << 48) | (iova >> 4)
    }
}

struct SetupPortCallback;

impl mailbox::MailCallback for SetupPortCallback {
    type Data = Arc<AopData>;

    /// Runs in hard IRQ context: hands the message to the ordered setup
    /// queue, which is all that may be done here. A message that cannot be
    /// queued is lost, and the protocol state with it.
    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, msg: mailbox::Message) {
        let queued = SetupRxWork::new(data.into(), msg).map(|work| {
            if let Some(queue) = data.setup_queue.as_ref() {
                queue.enqueue(work);
            }
        });
        if queued.is_err() {
            data.setup_lost.store(true, Release);
            data.setup_cv.notify_all();
        }
    }
}

/// One received setup-port message on its way to the setup queue.
#[pin_data]
struct SetupRxWork {
    data: Arc<AopData>,
    msg: mailbox::Message,
    #[pin]
    work: Work<SetupRxWork>,
}

impl_has_work! {
    impl HasWork<Self, 0> for SetupRxWork { self.work }
}

impl SetupRxWork {
    /// Allocates atomically: the caller is the mailbox interrupt handler.
    fn new(data: Arc<AopData>, msg: mailbox::Message) -> Result<Pin<KBox<Self>>> {
        KBox::pin_init(
            pin_init!(SetupRxWork {
                data,
                msg,
                work <- new_work!("SetupRxWork::work"),
            }),
            GFP_ATOMIC,
        )
    }
}

impl WorkItem for SetupRxWork {
    type Pointer = Pin<KBox<SetupRxWork>>;

    fn run(this: Pin<KBox<SetupRxWork>>) {
        this.data.setup_receive(this.msg);
    }
}

/// A service platform device registered by this driver. It is unregistered
/// explicitly by [`AopData::remove`]; nothing else touches the pointer.
struct ChildDevice(NonNull<bindings::platform_device>);

// SAFETY: The only operation on the pointer is `platform_device_unregister()`,
// which may be called from any thread.
unsafe impl Send for ChildDevice {}

/// A driver override that no driver matches: keeps a service device from
/// binding again once its transport is gone but it has to stay registered.
const RETIRED_DRIVER_OVERRIDE: &CStr = c_str!("apple-aop-retired");

/// Set once a shutdown could not be confirmed. The firmware may still be
/// running on the retained buffers, so the device is not brought up again
/// before a reboot.
static RETIRED: Atomic<bool> = Atomic::new(false);

impl ChildDevice {
    /// Keeps the child from binding to any driver again; it stays registered.
    fn retire(&self) {
        // SAFETY: The device is registered, so its embedded `struct device` is
        // valid, and the override string is NUL-terminated.
        let ret = unsafe {
            bindings::__device_set_driver_override(
                ptr::addr_of_mut!((*self.0.as_ptr()).dev),
                RETIRED_DRIVER_OVERRIDE.as_char_ptr(),
                RETIRED_DRIVER_OVERRIDE.to_bytes().len(),
            )
        };
        // Only an allocation failure; the device is unbound either way and
        // the retired flag keeps the parent from coming back.
        let _ = ret;
    }

    /// Unbinds the child's driver while the child stays registered.
    fn release_driver(&self) {
        // SAFETY: The pointer came from a successful
        // `platform_device_register_full()` and the device is still
        // registered, so its embedded `struct device` is valid.
        unsafe { bindings::device_release_driver(ptr::addr_of_mut!((*self.0.as_ptr()).dev)) };
    }

    fn unregister(self) {
        // SAFETY: The pointer came from a successful
        // `platform_device_register_full()` and is unregistered exactly once,
        // here, because this consumes `self`.
        unsafe { bindings::platform_device_unregister(self.0.as_ptr()) };
    }
}

#[pin_data]
struct AopData {
    dev: ARef<device::Device>,
    /// The firmware speaks EPIC with version 4 sub-headers.
    epic_v4: bool,
    /// Runs the service registrations one at a time; drained on removal.
    registration_queue: OwnedQueue,
    /// Serializes queueing a registration with the start of removal, so that
    /// nothing is queued once the queue drains. Never held across a drain.
    #[pin]
    registration_gate: Mutex<()>,
    /// Set once by the first teardown; later teardowns return at once.
    removing: Atomic<bool>,
    /// Set when the transport starts closing; no call is started after it.
    transport_closing: Atomic<bool>,
    /// The co-processor was started; only then is there anything to shut
    /// down and to retain.
    cpu_started: Atomic<bool>,
    /// Shut the co-processor down on removal, and if that cannot be
    /// confirmed, retain everything it may still DMA to.
    quiesce_on_unbind: bool,
    /// The endpoints that have to start; empty means all advertised ones.
    required_endpoints: &'static [u8],
    /// Runs the setup-port messages in order; only on firmware with a setup
    /// port.
    setup_queue: Option<OwnedQueue>,
    /// A setup-port message could not be queued from the interrupt handler.
    setup_lost: Atomic<bool>,
    #[pin]
    setup: Mutex<SetupState>,
    /// Signalled after every setup-port message.
    #[pin]
    setup_cv: CondVar,
    #[pin]
    rtkit: Mutex<Option<rtkit::RtKit<AopData>>>,
    #[pin]
    endpoints: [Mutex<AFKEndpoint>; AFK_ENDPOINT_COUNT as usize],
    /// Version 4 firmware answers without the request tag, so an endpoint
    /// carries one call at a time: callers queue here for their turn.
    #[pin]
    call_turn: [Mutex<()>; AFK_ENDPOINT_COUNT as usize],
    #[pin]
    ep_shutdown: [FutureValue<()>; AFK_ENDPOINT_COUNT as usize],
    #[pin]
    hid_listeners: Mutex<KVec<ListenerEntry>>,
    #[pin]
    report_listeners: Mutex<KVec<ReportListenerEntry>>,
    #[pin]
    source_ring: Mutex<Option<SourceRingBinding>>,
    #[pin]
    subdevices: Mutex<KVec<ChildDevice>>,
}

#[pin_data]
struct AopServiceRegisterWork {
    name: &'static CStr,
    data: Arc<AopData>,
    service: EPICService,
    #[pin]
    work: Work<AopServiceRegisterWork>,
}

impl_has_work! {
    impl HasWork<Self, 0> for AopServiceRegisterWork { self.work }
}

impl AopServiceRegisterWork {
    fn new(
        name: &'static CStr,
        data: Arc<AopData>,
        service: EPICService,
    ) -> Result<Pin<KBox<Self>>> {
        KBox::pin_init(
            pin_init!(AopServiceRegisterWork {
                name, data, service,
                work <- new_work!("AopServiceRegisterWork::work"),
            }),
            GFP_KERNEL,
        )
    }
}

impl WorkItem for AopServiceRegisterWork {
    type Pointer = Pin<KBox<AopServiceRegisterWork>>;

    fn run(this: Pin<KBox<AopServiceRegisterWork>>) {
        // Held until the device has been registered: the registration takes
        // its own reference on the node only then.
        let fwnode = this
            .data
            .dev
            .fwnode()
            .and_then(|x| x.get_child_by_name(this.name));
        let info = bindings::platform_device_info {
            parent: this.data.dev.as_raw(),
            name: this.name.as_char_ptr(),
            id: bindings::PLATFORM_DEVID_AUTO,
            res: ptr::null_mut(),
            num_res: 0,
            data: ptr::from_ref(&this.service).cast(),
            size_data: mem::size_of::<EPICService>(),
            dma_mask: 0,
            fwnode: fwnode
                .as_ref()
                .map(|x| x.as_raw())
                .unwrap_or(ptr::null_mut()),
            swnode: ptr::null_mut(),
            properties: ptr::null_mut(),
            of_node_reused: false,
        };
        // The slot is reserved before the device exists, so that a device
        // that was registered is always tracked and gets unregistered. The
        // child's probe runs inside the registration and calls back into the
        // transport, but nothing on that path takes this lock.
        let mut subdevices = this.data.subdevices.lock();
        if subdevices.reserve(1, GFP_KERNEL).is_err() {
            dev_err!(
                this.data.dev,
                "Failed to allocate the device slot for service {:?}",
                this.name
            );
            return;
        }
        // SAFETY: `info` is a valid, fully initialized `platform_device_info`
        // whose pointers outlive the call.
        let pdev = unsafe { from_err_ptr(bindings::platform_device_register_full(&info)) };
        drop(fwnode);
        match pdev.and_then(|pdev| NonNull::new(pdev).ok_or(EINVAL)) {
            Err(e) => {
                dev_err!(
                    this.data.dev,
                    "Failed to create device for service {:?}: {:?}",
                    this.name,
                    e
                );
            }
            Ok(pdev) => {
                if let Err(child) = subdevices.push_within_capacity(ChildDevice(pdev)) {
                    // Cannot happen after the reservation above; never leave
                    // a registered device untracked.
                    child.0.unregister();
                }
            }
        }
    }
}

impl AopData {
    fn new(dev: &platform::Device<Core<'_>>, cfg: &AopHwConfig) -> Result<Arc<AopData>> {
        let registration_queue = OwnedQueue::new_ordered(c_str!("apple-aop"))?;
        let setup_queue = if cfg.setup_port {
            Some(OwnedQueue::new_ordered(c_str!("apple-aop-setup"))?)
        } else {
            None
        };
        Arc::pin_init(
            pin_init!(
                AopData {
                    dev: dev.as_ref().into(),
                    epic_v4: cfg.epic_v4,
                    registration_queue,
                    registration_gate <- new_mutex!(()),
                    removing: Atomic::new(false),
                    transport_closing: Atomic::new(false),
                    cpu_started: Atomic::new(false),
                    quiesce_on_unbind: cfg.quiesce_on_unbind,
                    required_endpoints: cfg.required_endpoints,
                    setup_queue,
                    setup_lost: Atomic::new(false),
                    setup <- new_mutex!(SetupState::new()),
                    setup_cv <- new_condvar!(),
                    rtkit <- new_mutex!(None),
                    endpoints <- pin_init::pin_init_array_from_fn(|i| {
                        new_mutex!(AFKEndpoint::new(AFK_ENDPOINT_START + i as u8))
                    }),
                    call_turn <- pin_init::pin_init_array_from_fn(|_| new_mutex!(())),
                    ep_shutdown <- pin_init::pin_init_array_from_fn(|_| FutureValue::pin_init()),
                    hid_listeners <- new_mutex!(KVec::new()),
                    report_listeners <- new_mutex!(KVec::new()),
                    source_ring <- new_mutex!(None),
                    subdevices <- new_mutex!(KVec::new()),
                }
            ),
            GFP_KERNEL,
        )
    }
    fn start(&self) -> Result<()> {
        self.wake()?;
        self.start_afk()
    }
    /// Runs the RTKit handshake up to AP power on.
    fn wake(&self) -> Result<()> {
        let mut guard = self.rtkit.lock();
        let mut rtk = guard.as_mut().as_pin_mut().ok_or(ENODEV)?;
        rtk.as_mut().wake()
    }
    /// Starts the AFK handshake on every advertised endpoint. An endpoint
    /// the match data does not require may fail to start; it is skipped.
    fn start_afk(&self) -> Result<()> {
        for ep in 0..AFK_ENDPOINT_COUNT as usize {
            let rtk_ep_num = AFK_ENDPOINT_START + ep as u8;
            let mut guard = self.rtkit.lock();
            let mut rtk = guard.as_mut().as_pin_mut().ok_or(ENODEV)?;
            if !rtk.as_mut().has_endpoint(rtk_ep_num) {
                continue;
            }
            let required =
                self.required_endpoints.is_empty() || self.required_endpoints.contains(&rtk_ep_num);
            let started = rtk.as_mut().start_endpoint(rtk_ep_num).and_then(|()| {
                let mut ep_guard = self.endpoints[ep].lock();
                ep_guard.start(rtk.as_mut())?;
                ep_guard.started = true;
                Ok(())
            });
            match started {
                Ok(()) => {}
                Err(e) if !required => {
                    dev_warn!(
                        self.dev,
                        "Endpoint {:#04x} did not start ({:?}); skipping it",
                        rtk_ep_num,
                        e
                    );
                }
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }
    fn register_service(
        self: Arc<Self>,
        ep: &mut AFKEndpoint,
        channel: u32,
        name: &[u8],
    ) -> Result<()> {
        let svc = EPICService {
            channel,
            endpoint: ep.index,
        };
        let dev_name = match name {
            b"aop-audio" => c_str!("audio"),
            b"las" => c_str!("las"),
            b"als" => c_str!("als"),
            _ => {
                return Ok(());
            }
        };
        // The child's probe calls back into the transport, so it runs from a
        // work item with the endpoint lock dropped. The gate orders queueing
        // it against removal, which drains the queue after setting the flag.
        let gate = self.registration_gate.lock();
        if self.removing.load(Acquire) {
            return Ok(());
        }
        let work = AopServiceRegisterWork::new(dev_name, self.clone(), svc)?;
        self.registration_queue.enqueue(work);
        drop(gate);
        Ok(())
    }

    /// Hands a report to its listener; `Ok(false)` when there is none.
    fn process_report(&self, ep: &AFKEndpoint, ch: u32, subtype: u16, data: &[u8]) -> Result<bool> {
        let guard = self.report_listeners.lock();
        for entry in &*guard {
            if entry.svc.endpoint == ep.index && entry.svc.channel == ch && entry.subtype == subtype
            {
                entry.listener.process_report(subtype, data)?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn process_fakehid_report(&self, ep: &AFKEndpoint, ch: u32, data: &[u8]) -> Result<()> {
        let guard = self.hid_listeners.lock();
        for entry in &*guard {
            if entry.svc.endpoint == ep.index && entry.svc.channel == ch {
                return entry.listener.process_fakehid_report(data);
            }
        }
        Ok(())
    }

    fn shutdown_complete(&self, endpoint: u8) {
        if let Some(index) = afk_endpoint_index(endpoint) {
            self.ep_shutdown[index].complete(());
        }
    }

    /// Shuts down every started endpoint, waiting a bounded time for each
    /// acknowledgment. Returns the first error but still tries the rest.
    fn stop(&self) -> Result<()> {
        let mut ret = Ok(());
        for ep in 0..AFK_ENDPOINT_COUNT as usize {
            {
                let mut guard = self.rtkit.lock();
                let mut rtk = guard.as_mut().as_pin_mut().ok_or(ENODEV)?;
                let mut ep_guard = self.endpoints[ep].lock();
                if !ep_guard.started {
                    continue;
                }
                // Whatever happens below, the endpoint is not started again.
                ep_guard.started = false;
                self.ep_shutdown[ep].reset();
                if let Err(e) = ep_guard.stop(rtk.as_mut()) {
                    ret = ret.and(Err(e));
                    continue;
                }
            }
            if self.ep_shutdown[ep]
                .wait_timeout(AFK_SHUTDOWN_TIMEOUT_MS)
                .is_none()
            {
                dev_warn!(
                    self.dev,
                    "Endpoint {:#04x} did not acknowledge its shutdown",
                    AFK_ENDPOINT_START + ep as u8
                );
                ret = ret.and(Err(ETIMEDOUT));
            }
        }
        ret
    }

    fn patch_bootargs(
        &self,
        aop_mmio: &Mmio<'_, Region<AOP_MMIO_SIZE>>,
        patches: &[(u32, u64)],
    ) -> Result<()> {
        let offset = aop_mmio.read32(BOOTARGS_OFFSET) as usize;
        let size = aop_mmio.read32(BOOTARGS_SIZE) as usize;
        let mut arg_bytes = KVec::<u8>::from_elem(0, size, GFP_KERNEL)?;
        let aop_io_array: Mmio<'_, [u8; AOP_MMIO_SIZE]> = aop_mmio.try_cast()?;
        let bootargs = io_project!(aop_io_array, [try: offset .. offset+size]);
        bootargs.copy_to_slice(&mut arg_bytes);
        let mut idx = 0;
        while idx < size {
            let key = u32::from_le_bytes(arg_bytes[idx..idx + 4].try_into().unwrap());
            let size = u32::from_le_bytes(arg_bytes[idx + 4..idx + 8].try_into().unwrap()) as usize;
            idx += 8;
            for (k, v) in patches.iter() {
                if *k != key {
                    continue;
                }
                arg_bytes[idx..idx + size].copy_from_slice(&v.to_le_bytes()[..size]);
                break;
            }
            idx += size;
        }
        bootargs.copy_from_slice(&arg_bytes);
        Ok(())
    }

    /// Leaks every allocation the firmware may still access. Called when the
    /// shutdown could not be confirmed; the memory is lost until reboot.
    fn retain_dma_buffers(&self) {
        for endpoint in &self.endpoints {
            if let Some(buffer) = endpoint.lock().iomem.take() {
                mem::forget(buffer);
            }
        }
        if let Some(arena) = self.setup.lock().arena.take() {
            mem::forget(arena);
        }
        if let Some(binding) = self.source_ring.lock().take() {
            mem::forget(binding);
        }
    }
}

impl AopData {
    /// On firmware whose replies carry no tag, takes the endpoint's turn: the
    /// guard is held until the reply or the timeout, so that a reply can only
    /// belong to the one call in flight.
    fn take_call_turn(&self, ep_idx: usize) -> Option<MutexGuard<'_, ()>> {
        self.epic_v4.then(|| self.call_turn[ep_idx].lock())
    }

    /// Waits a bounded time for the reply to `call` on endpoint `ep_idx`.
    fn wait_call(
        &self,
        ep_idx: usize,
        svc: &EPICService,
        subtype: u16,
        call: Arc<FutureValue<CallResult>>,
    ) -> Result<CallResult> {
        if let Some(res) = call.wait_timeout(EPIC_CALL_TIMEOUT_MS) {
            return Ok(res);
        }
        self.endpoints[ep_idx].lock().abandon_call(&call);
        dev_err!(
            self.dev,
            "EPIC call {:#x} on channel {} timed out after {} ms{}",
            subtype,
            svc.channel,
            EPIC_CALL_TIMEOUT_MS,
            if self.epic_v4 {
                "; the endpoint takes no call until the reply arrives"
            } else {
                ""
            }
        );
        Err(ETIMEDOUT)
    }
}

impl AOP for AopData {
    fn epic_call(&self, svc: &EPICService, subtype: u16, msg_bytes: &[u8]) -> Result<u32> {
        if self.transport_closing.load(Acquire) {
            return Err(ENODEV);
        }
        let ep_idx = afk_endpoint_index(svc.endpoint).ok_or(EINVAL)?;
        let _turn = self.take_call_turn(ep_idx);
        let call = {
            let mut rtk_guard = self.rtkit.lock();
            let mut rtk = rtk_guard.as_mut().as_pin_mut().ok_or(ENODEV)?;
            let mut ep_guard = self.endpoints[ep_idx].lock();
            ep_guard.epic_notify(self, rtk.as_mut(), svc.channel, subtype, msg_bytes, None)?
        };
        Ok(self.wait_call(ep_idx, svc, subtype, call)?.retcode)
    }
    fn epic_call_ret(
        &self,
        svc: &EPICService,
        subtype: u16,
        msg_bytes: &[u8],
        ret_len: usize,
    ) -> Result<(u32, KVec<u8>)> {
        if self.transport_closing.load(Acquire) {
            return Err(ENODEV);
        }
        let ep_idx = afk_endpoint_index(svc.endpoint).ok_or(EINVAL)?;
        let _turn = self.take_call_turn(ep_idx);
        let call = {
            let mut rtk_guard = self.rtkit.lock();
            let mut rtk = rtk_guard.as_mut().as_pin_mut().ok_or(ENODEV)?;
            let mut ep_guard = self.endpoints[ep_idx].lock();
            let mut ret_buf = KVec::new();
            ret_buf.resize(ret_len, 0, GFP_KERNEL)?;
            ep_guard.epic_notify(
                self,
                rtk.as_mut(),
                svc.channel,
                subtype,
                msg_bytes,
                Some(ret_buf),
            )?
        };
        let res = self.wait_call(ep_idx, svc, subtype, call)?;
        Ok((res.retcode, res.extra_data.ok_or(EIO)?))
    }
    fn add_fakehid_listener(
        &self,
        svc: EPICService,
        listener: Arc<dyn FakehidListener>,
    ) -> Result<()> {
        let mut guard = self.hid_listeners.lock();
        if self.removing.load(Acquire) {
            return Err(ENODEV);
        }
        if guard.iter().any(|entry| entry.svc == svc) {
            return Err(EBUSY);
        }
        Ok(guard.push(ListenerEntry { svc, listener }, GFP_KERNEL)?)
    }
    fn remove_fakehid_listener(&self, svc: &EPICService) -> bool {
        let mut guard = self.hid_listeners.lock();
        for i in 0..guard.len() {
            if guard[i].svc == *svc {
                guard.swap_remove(i);
                return true;
            }
        }
        false
    }
    fn add_report_listener(
        &self,
        svc: EPICService,
        subtype: u16,
        listener: Arc<dyn ReportListener>,
    ) -> Result<()> {
        let mut guard = self.report_listeners.lock();
        if self.removing.load(Acquire) {
            return Err(ENODEV);
        }
        if guard
            .iter()
            .any(|entry| entry.svc == svc && entry.subtype == subtype)
        {
            return Err(EBUSY);
        }
        Ok(guard.push(
            ReportListenerEntry {
                svc,
                subtype,
                listener,
            },
            GFP_KERNEL,
        )?)
    }
    fn remove_report_listener(&self, svc: &EPICService, subtype: u16) -> bool {
        let mut guard = self.report_listeners.lock();
        for i in 0..guard.len() {
            if guard[i].svc == *svc && guard[i].subtype == subtype {
                guard.swap_remove(i);
                return true;
            }
        }
        false
    }
    fn source_ring(&self, dev: &device::Device<Bound>, size: usize) -> Result<Arc<SourceRing>> {
        if self.setup_queue.is_none() || self.transport_closing.load(Acquire) {
            return Err(ENODEV);
        }
        let mut guard = self.source_ring.lock();
        if let Some(binding) = guard.as_ref() {
            if !binding.confirmed {
                return Err(EIO);
            }
            if binding.ring.size() < size {
                return Err(EBUSY);
            }
            return Ok(binding.ring.clone());
        }
        let buf = Coherent::<u8>::zeroed_slice(dev, size, GFP_KERNEL)?;
        let ring = Arc::new(SourceRing::new(buf), GFP_KERNEL)?;
        let iova = ring.iova;
        // The binding is recorded before the firmware learns the address:
        // whatever the request's outcome, the ring is kept and not bound
        // again, since even a timeout may mean the firmware took it.
        *guard = Some(SourceRingBinding {
            ring: ring.clone(),
            confirmed: false,
        });
        self.setup_bind_source(iova, size as u64)?;
        if let Some(binding) = &mut *guard {
            binding.confirmed = true;
        }
        Ok(ring)
    }
    /// Takes the AOP down: from unbind, from a failed probe, or as a fallback
    /// from Drop. Only the first call does anything.
    fn remove(&self) {
        {
            let _gate = self.registration_gate.lock();
            if self.removing.xchg(true, Acquire) {
                return;
            }
        }
        // No registration is queued after this point, so once the queue is
        // empty the list of children is complete.
        self.registration_queue.drain();
        // Unbind the children while the transport still works, so that their
        // unbind can talk to their services. Then drop any listener a child
        // left behind: no report may reach a driver that is going away.
        let children = mem::take(&mut *self.subdevices.lock());
        for child in &children {
            child.release_driver();
        }
        self.hid_listeners.lock().clear();
        self.report_listeners.lock().clear();
        self.transport_closing.store(true, Release);
        if let Err(e) = self.stop() {
            dev_err!(self.dev, "Failed to stop AOP {:?}", e);
        }
        // Take the handle out of the shared state before dropping it: the
        // drop waits for the RTKit receive worker, which takes the same lock.
        // After it, no callback runs and the device may be unbound.
        let rtkit = self.rtkit.lock().take();
        let mut quiesced = true;
        if let Some(mut rtkit) = rtkit {
            if self.quiesce_on_unbind && self.cpu_started.load(Acquire) {
                // The co-processor DMAs into the shared buffers until it has
                // acknowledged the shutdown. If it does not, nothing it may
                // still write to can be freed.
                if let Err(e) = Pin::new(&mut rtkit).shutdown() {
                    dev_err!(
                        self.dev,
                        "AOP shutdown unconfirmed ({:?}); retaining its buffers until reboot",
                        e
                    );
                    rtkit.retain_shared_buffers_on_drop();
                    quiesced = false;
                }
            }
            drop(rtkit);
        }
        // Close the setup port: dropping the mailbox stops its interrupt, and
        // draining the queue finishes the messages that were already taken.
        // Neither may happen under the setup lock, which the queue's work
        // takes. The arena is freed while the device is still bound.
        let setup_mbox = self.setup.lock().mbox.take();
        drop(setup_mbox);
        if let Some(queue) = self.setup_queue.as_ref() {
            queue.drain();
        }
        if !quiesced {
            // The children stay registered: their IOMMU domains hold the
            // mappings the firmware may still use. Keep them from binding
            // again, and this device from probing again, until a reboot.
            self.retain_dma_buffers();
            for child in &children {
                child.retire();
            }
            RETIRED.store(true, Release);
            dev_err!(
                self.dev,
                "keeping {} unbound service devices and their DMA mappings",
                children.len()
            );
            return;
        }
        let arena = self.setup.lock().arena.take();
        drop(arena);
        // The ring is DMA of the audio child; free it while the child is
        // still registered and keeps its IOMMU domain.
        let source = self.source_ring.lock().take();
        drop(source);
        for child in children {
            child.unregister();
        }
    }
}

impl AopData {
    /// Opens the setup port: the mailbox and the arena its endpoints get
    /// their buffers from. Done before the co-processor runs, since its
    /// first message has to be answered.
    fn setup_open(this: &Arc<AopData>, dev: &device::Device) -> Result<()> {
        let mbox =
            mailbox::Mailbox::<SetupPortCallback>::new_byname(dev, c_str!("setup"), this.clone())?;
        // The arena is DMA of the mailbox provider's device: its node carries
        // the DART stream the firmware reaches the setup buffers through.
        let mbox_dev = mbox.device();
        // SAFETY: `mbox_dev` is a valid device and nothing of ours has DMA in
        // flight on it yet.
        unsafe {
            to_result(bindings::dma_set_mask_and_coherent(
                mbox_dev.as_raw(),
                DmaMask::new::<42>().value(),
            ))?;
        }
        // SAFETY: Getting the mailbox added a device link from this device to
        // the provider, so the provider stays bound for as long as this device
        // is, and the arena is freed by `remove()` while this device is bound.
        let bound = unsafe { mbox_dev.as_bound() };
        let arena =
            Coherent::<u8>::zeroed_slice(bound, SETUP_ARENA_PAGES * SETUP_PAGE, GFP_KERNEL)?;
        let mut st = this.setup.lock();
        st.arena = Some(arena);
        st.mbox = Some(mbox);
        Ok(())
    }

    /// Handles one received setup-port message, in order, on the setup queue.
    fn setup_receive(&self, msg: mailbox::Message) {
        let mut st = self.setup.lock();
        if st.mbox.is_none() {
            return;
        }
        if self.setup_lost.load(Acquire) {
            st.failed = true;
        } else {
            let ep = (msg.msg1 & 0xff) as u8;
            if let Err(e) = self.setup_handle(&mut st, ep, msg.msg0) {
                dev_err!(
                    self.dev,
                    "setup port: protocol error on endpoint {:#x}, message {:#x}: {:?}",
                    ep,
                    msg.msg0,
                    e
                );
                st.failed = true;
            }
        }
        drop(st);
        self.setup_cv.notify_all();
    }

    fn setup_handle(&self, st: &mut SetupState, ep: u8, word: u64) -> Result<()> {
        if ep == SETUP_MGMT_EP {
            return match (word >> 52) & 0xff {
                SETUP_TYPE_HELLO => {
                    if word & 0xffff_ffff != SETUP_HELLO_VERSION {
                        dev_warn!(
                            self.dev,
                            "setup port: HELLO version {:#x}",
                            word & 0xffff_ffff
                        );
                    }
                    st.send(
                        SETUP_MGMT_EP,
                        (SETUP_TYPE_HELLO_REPLY << 52) | SETUP_HELLO_VERSION,
                    )
                }
                SETUP_TYPE_EPMAP => {
                    // Each fragment of the endpoint map is acknowledged by
                    // echoing it.
                    st.send(SETUP_MGMT_EP, word)?;
                    if word & SETUP_EPMAP_LAST != 0 {
                        st.map_done = true;
                    }
                    Ok(())
                }
                SETUP_TYPE_AP_PWR => {
                    st.ap_ready = (word & 0xffff) == SETUP_AP_PWR_ON;
                    Ok(())
                }
                SETUP_TYPE_UNK3 => st.send(SETUP_MGMT_EP, SETUP_TYPE_UNK3_REPLY << 52),
                SETUP_TYPE_PWR_ACK => Ok(()),
                ty => {
                    dev_warn!(
                        self.dev,
                        "setup port: ignoring management message type {:#x} ({:#x})",
                        ty,
                        word
                    );
                    Ok(())
                }
            };
        }
        if word >> 56 == SETUP_BUFFER_REQUEST {
            return self.setup_endpoint_request(st, ep, word);
        }
        if let Some(endpoint) = st.endpoint_mut(ep) {
            if endpoint.abandoned {
                // The late reply to a request that timed out; the endpoint
                // takes requests again.
                dev_warn!(
                    self.dev,
                    "setup port: late reply {:#x} on endpoint {:#x}",
                    word,
                    ep
                );
                endpoint.abandoned = false;
                endpoint.busy = false;
                return Ok(());
            }
            endpoint.reply = Some(word);
            return Ok(());
        }
        dev_warn!(
            self.dev,
            "setup port: ignoring message {:#x} on endpoint {:#x}",
            word,
            ep
        );
        Ok(())
    }

    /// Answers a service endpoint's buffer request with one message page
    /// and a reply window of the requested number of entries, both taken
    /// from the arena.
    fn setup_endpoint_request(&self, st: &mut SetupState, ep: u8, word: u64) -> Result<()> {
        let requested = (word & 0xffff_ffff) as usize;
        let slot = SETUP_ENDPOINTS
            .iter()
            .position(|e| *e == ep)
            .ok_or(EINVAL)?;
        if st.endpoints[slot].is_some() || (requested != 0x400 && requested != 0x1000) {
            return Err(EINVAL);
        }
        let rx_pages = (requested * SETUP_BUFFER_ENTRY_SIZE).div_ceil(SETUP_PAGE);
        if st.next_page + 1 + rx_pages > SETUP_ARENA_PAGES {
            return Err(ENOMEM);
        }
        let base = st.arena_iova()?;
        let tx_iova = base + (st.next_page * SETUP_PAGE) as u64;
        let rx_iova = tx_iova + SETUP_PAGE as u64;
        st.next_page += 1 + rx_pages;
        st.send(ep, SETUP_BUFFER_REQUEST_ACK)?;
        st.send(ep, SetupState::shared_descriptor(tx_iova, 1, true))?;
        st.send(ep, SetupState::shared_descriptor(rx_iova, rx_pages, false))?;
        st.endpoints[slot] = Some(SetupEndpoint {
            ep,
            tx_iova,
            rx_iova,
            busy: false,
            abandoned: false,
            reply: None,
        });
        dev_dbg!(
            self.dev,
            "setup port: endpoint {:#x} tx {:#x} rx {:#x} ({} pages)",
            ep,
            tx_iova,
            rx_iova,
            rx_pages
        );
        Ok(())
    }

    /// Waits, in TASK_UNINTERRUPTIBLE and for at most `timeout_ms`, until
    /// `done` holds for the setup state. Fails at once when the setup port
    /// has failed. Spurious wakeups continue the wait with the time left.
    fn setup_wait(
        &self,
        st: &mut MutexGuard<'_, SetupState>,
        timeout_ms: u32,
        done: impl Fn(&SetupState) -> bool,
    ) -> Result<()> {
        let mut left = msecs_to_jiffies(timeout_ms);
        loop {
            if done(st) {
                return Ok(());
            }
            if st.failed || self.setup_lost.load(Acquire) {
                return Err(EIO);
            }
            match self.setup_cv.wait_timeout(st, left) {
                CondVarTimeoutResult::Timeout => {
                    return if done(st) { Ok(()) } else { Err(ETIMEDOUT) };
                }
                CondVarTimeoutResult::Woken { jiffies }
                | CondVarTimeoutResult::Signal { jiffies } => left = jiffies,
            }
        }
    }

    /// Sends `payload` as a request on setup-port endpoint `ep`, waits for
    /// the firmware's result word, and returns the status byte the firmware
    /// wrote to the reply window if the result is one of `accepted`. An
    /// endpoint takes one request at a time (EBUSY otherwise). After a
    /// timeout the endpoint stays busy until the late reply arrives.
    fn setup_request(&self, ep: u8, payload: &[u8], accepted: &[u64]) -> Result<u8> {
        if payload.len() > SETUP_PAGE {
            return Err(EMSGSIZE);
        }
        let mut st = self.setup.lock();
        let endpoint = st.endpoint_mut(ep).ok_or(ENXIO)?;
        if endpoint.busy {
            return Err(EBUSY);
        }
        endpoint.busy = true;
        endpoint.reply = None;
        let (tx_iova, rx_iova) = (endpoint.tx_iova, endpoint.rx_iova);
        let ret = self.setup_request_locked(&mut st, ep, tx_iova, rx_iova, payload, accepted);
        if let Some(endpoint) = st.endpoint_mut(ep) {
            if ret == Err(ETIMEDOUT) {
                endpoint.abandoned = true;
            } else {
                endpoint.busy = false;
            }
        }
        ret
    }

    fn setup_request_locked(
        &self,
        st: &mut MutexGuard<'_, SetupState>,
        ep: u8,
        tx_iova: u64,
        rx_iova: u64,
        payload: &[u8],
        accepted: &[u64],
    ) -> Result<u8> {
        let tx_off = st.arena_offset(tx_iova)?;
        let rx_off = st.arena_offset(rx_iova)?;
        let tx = st.arena_ptr(tx_off, payload.len())?;
        // SAFETY: `tx` is valid for `payload.len()` bytes of the arena, and
        // the message page is the host's until the request word below is
        // sent; the endpoint is busy, so nothing else writes it.
        unsafe { ptr::copy_nonoverlapping(payload.as_ptr(), tx, payload.len()) };
        mem_sync();
        st.send(ep, SETUP_REQUEST_TYPE | payload.len() as u64)?;
        let replied = |st: &SetupState| {
            st.endpoints
                .iter()
                .flatten()
                .any(|e| e.ep == ep && e.reply.is_some())
        };
        if let Err(e) = self.setup_wait(st, SETUP_REPLY_TIMEOUT_MS, replied) {
            dev_err!(
                self.dev,
                "setup port: no reply to the request on endpoint {:#x}: {:?}",
                ep,
                e
            );
            return Err(e);
        }
        let reply = st
            .endpoint_mut(ep)
            .and_then(|e| e.reply.take())
            .ok_or(EIO)?;
        if !accepted.contains(&reply) {
            dev_err!(
                self.dev,
                "setup port: request on endpoint {:#x} answered with {:#x}",
                ep,
                reply
            );
            return Err(EIO);
        }
        mem_sync();
        let rx = st.arena_ptr(rx_off, 1)?;
        // SAFETY: `rx` is valid for one byte of the arena. The firmware wrote
        // the status before it sent the reply and does not touch the window
        // again until the next request.
        let status = unsafe { rx.read_volatile() };
        st.send(ep, SETUP_REQUEST_DONE)?;
        Ok(status)
    }

    /// Binds the source ring to the firmware: one bind per boot.
    fn setup_bind_source(&self, iova: u64, size: u64) -> Result<()> {
        let mut req = [0u8; 24];
        req[..8].copy_from_slice(&SETUP_SET_SOURCE_BUFFER.to_le_bytes());
        req[8..16].copy_from_slice(&iova.to_le_bytes());
        req[16..].copy_from_slice(&size.to_le_bytes());
        let status = self.setup_request(SETUP_SOURCE_EP, &req, &[SETUP_REPLY_READY])?;
        if status != 0 {
            dev_err!(self.dev, "source ring bind rejected (status {:#x})", status);
            return Err(EIO);
        }
        Ok(())
    }

    /// Sends the ambient light sensor its calibration, named by the ALS
    /// node's firmware-name, before the AFK endpoints start. Without it the
    /// sensor answers property reads and accepts its reporting interval but
    /// never reports; the trusted side does not start it. A missing file is
    /// not an error: the AOP's other services work without it.
    fn setup_als_calibration(&self) -> Result<()> {
        let Some(name) = self
            .dev
            .fwnode()
            .and_then(|node| node.get_child_by_name(c_str!("als")))
            .and_then(|als| {
                als.property_read::<CString>(c_str!("firmware-name"))
                    .optional()
            })
        else {
            dev_info!(
                self.dev,
                "no ALS calibration named in the device tree; the sensor stays off"
            );
            return Ok(());
        };
        let calibration = match Firmware::request_nowarn(&name, &self.dev) {
            Ok(calibration) => calibration,
            Err(e) => {
                dev_warn!(
                    self.dev,
                    "ALS calibration {:?} not available ({:?}); the sensor stays off",
                    &*name,
                    e
                );
                return Ok(());
            }
        };
        let data = calibration.data();
        if data.len() != ALS_CALIBRATION_LEN
            || le_u64(data, 0) != ALS_CALIBRATION_OPERATION
            || le_u64(data, 8) != ALS_CALIBRATION_BODY_LEN
        {
            dev_err!(
                self.dev,
                "ALS calibration {:?} is not an {}-byte calibration message",
                &*name,
                ALS_CALIBRATION_LEN
            );
            return Err(EINVAL);
        }
        let status = self.setup_request(
            SETUP_ALS_EP,
            data,
            &[SETUP_REPLY_READY, SETUP_REPLY_READY_ALT],
        )?;
        if status != 0 {
            dev_err!(self.dev, "ALS calibration rejected (status {:#x})", status);
            return Err(EIO);
        }
        Ok(())
    }

    /// After the RTKit side has reached AP power on: requests the setup
    /// port's AP power state and waits for it and for the five endpoint
    /// buffers.
    fn setup_finish_boot(&self) -> Result<()> {
        let mut st = self.setup.lock();
        if !st.map_done {
            dev_warn!(
                self.dev,
                "setup port: endpoint map incomplete before the AP power request"
            );
        }
        if !st.power_sent {
            st.send(SETUP_MGMT_EP, (SETUP_TYPE_AP_PWR << 52) | SETUP_AP_PWR_INIT)?;
            st.power_sent = true;
        }
        if let Err(e) = self.setup_wait(&mut st, SETUP_BOOT_TIMEOUT_MS, SetupState::ready) {
            dev_err!(
                self.dev,
                "setup port: boot incomplete (AP power on: {}, endpoints: {}/{}): {:?}",
                st.ap_ready,
                st.endpoint_count(),
                SETUP_ENDPOINTS.len(),
                e
            );
            return Err(e);
        }
        Ok(())
    }
}

struct NoBuffer;
impl rtkit::Buffer for NoBuffer {
    fn iova(&self) -> Result<usize> {
        unreachable!()
    }
    fn buf(&mut self) -> Result<IoSysMap<'_, Region>> {
        unreachable!()
    }
}

#[vtable]
impl rtkit::Operations for AopData {
    type Data = Arc<AopData>;
    type Buffer = NoBuffer;

    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, ep: u8, msg: u64) {
        let Some(index) = afk_endpoint_index(ep) else {
            dev_warn!(
                data.dev,
                "Message {:#x} on unexpected endpoint {:#04x}",
                msg,
                ep
            );
            return;
        };
        let mut guard = data.rtkit.lock();
        let Some(mut rtk) = guard.as_mut().as_pin_mut() else {
            return;
        };
        let mut ep_guard = data.endpoints[index].lock();
        let ret = ep_guard.recv_message(data, rtk.as_mut(), msg);
        if let Err(e) = ret {
            dev_err!(data.dev, "Failed to handle rtkit message, error: {:?}", e);
        }
    }

    fn crashed(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, _crashlog: Option<&[u8]>) {
        dev_err!(data.dev, "AOP firmware crashed");
    }
}

/// The driver data of the AOP. Children reach the `Arc` through
/// `AOP::from_child()`, which relies on this being `repr(transparent)` over
/// it.
#[repr(transparent)]
struct AopDriver(Arc<dyn AOP>);

struct AopHwConfig {
    ec0p: u64,
    alig: u64,
    aopt: u64,
    /// Complete the firmware's boot arguments before starting it.
    patch_bootargs: bool,
    /// The firmware speaks EPIC with version 4 sub-headers.
    epic_v4: bool,
    /// The firmware boots through a second, "setup", mailbox as well.
    setup_port: bool,
    /// Shut the co-processor down on unbind and retain its buffers if the
    /// shutdown cannot be confirmed.
    quiesce_on_unbind: bool,
    /// The endpoints that have to start; empty means all advertised ones.
    required_endpoints: &'static [u8],
}

const HW_CFG_T8103: AopHwConfig = AopHwConfig {
    ec0p: 0x020000,
    aopt: 1,
    alig: 128,
    patch_bootargs: true,
    epic_v4: false,
    setup_port: false,
    quiesce_on_unbind: false,
    required_endpoints: &[],
};
const HW_CFG_T8112: AopHwConfig = AopHwConfig {
    ec0p: 0x020000,
    aopt: 0,
    alig: 128,
    patch_bootargs: true,
    epic_v4: false,
    setup_port: false,
    quiesce_on_unbind: false,
    required_endpoints: &[],
};
const HW_CFG_T6000: AopHwConfig = AopHwConfig {
    ec0p: 0x020000,
    aopt: 0,
    alig: 64,
    patch_bootargs: true,
    epic_v4: false,
    setup_port: false,
    quiesce_on_unbind: false,
    required_endpoints: &[],
};
const HW_CFG_T6020: AopHwConfig = AopHwConfig {
    ec0p: 0x0100_00000000,
    aopt: 0,
    alig: 64,
    patch_bootargs: true,
    epic_v4: false,
    setup_port: false,
    quiesce_on_unbind: false,
    required_endpoints: &[],
};
/// T8140: the firmware is started with the boot arguments the bootloader
/// left, boots through the setup port and speaks EPIC version 4. Of the
/// advertised AFK endpoints, the application map is 0x20 misc, 0x21
/// aop-audio, 0x22 aop-voicetrigger, 0x23 als and 0x2b aop-audprov; the rest
/// are started if they will.
const HW_CFG_T8140: AopHwConfig = AopHwConfig {
    ec0p: 0,
    aopt: 0,
    alig: 0,
    patch_bootargs: false,
    epic_v4: true,
    setup_port: true,
    quiesce_on_unbind: true,
    required_endpoints: &[0x20, 0x21, 0x22, 0x23, 0x2b],
};

/// T8152 (Mac mini): the firmware is preloaded by the bootloader and the setup
/// mailbox belongs to the secure world, so there is no setup port: the CPU is
/// started through the wrapper and RTKit boots on the primary mailbox. It
/// advertises three AFK endpoints, 0x20 SPUApp, 0x21 wakehint and 0x22
/// aop-audio, all announced with EPIC sub-header version 2. The firmware only
/// brings its AFK endpoints up once all of them are started, which start_afk
/// does back to back. Calls with the version 2 header on the aop-audio
/// endpoint were all answered, with the tag echoed at the version 2 offset.
const HW_CFG_T8152: AopHwConfig = AopHwConfig {
    ec0p: 0,
    aopt: 0,
    alig: 0,
    patch_bootargs: false,
    epic_v4: false,
    setup_port: false,
    quiesce_on_unbind: true,
    required_endpoints: &[0x20, 0x21, 0x22],
};

kernel::of_device_table!(
    OF_TABLE,
    MODULE_OF_TABLE,
    <AopDriver as platform::Driver>::IdInfo,
    [
        (of::DeviceId::new(c_str!("apple,t8103-aop")), &HW_CFG_T8103),
        (of::DeviceId::new(c_str!("apple,t8112-aop")), &HW_CFG_T8112),
        (of::DeviceId::new(c_str!("apple,t6000-aop")), &HW_CFG_T6000),
        (of::DeviceId::new(c_str!("apple,t6020-aop")), &HW_CFG_T6020),
        (of::DeviceId::new(c_str!("apple,t8140-aop")), &HW_CFG_T8140),
        (of::DeviceId::new(c_str!("apple,t8152-aop")), &HW_CFG_T8152),
    ]
);

impl platform::Driver for AopDriver {
    type IdInfo = &'static AopHwConfig;
    type Data<'bound> = AopDriver;

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let cfg = info.ok_or(ENODEV)?;
        if RETIRED.load(Acquire) {
            dev_err!(
                pdev.as_ref(),
                "an earlier instance could not be shut down; reboot before probing again"
            );
            return Err(ENODEV);
        }
        // SAFETY: nothing of this device has DMA in flight before probe.
        unsafe { pdev.dma_set_mask_and_coherent(DmaMask::new::<42>())? };
        let aop_req = pdev.io_request_by_index(0).ok_or(EINVAL)?;
        let aop_mmio = aop_req.iomap_sized::<AOP_MMIO_SIZE>()?;
        let asc_req = pdev.io_request_by_index(1).ok_or(EINVAL)?;
        let asc_mmio = asc_req.iomap_sized::<ASC_MMIO_SIZE>()?;
        let asc_mmio = asc_mmio.as_view().relaxed();
        let data = AopData::new(pdev, cfg)?;
        // Whatever fails below leaves the AOP running and its children
        // registering; the same teardown as unbind's cleans that up.
        let probe_guard = ScopeGuard::new_with_data(data.clone(), |data| data.remove());
        let aop_mmio = aop_mmio.as_view();
        if cfg.patch_bootargs {
            data.patch_bootargs(
                &aop_mmio,
                &[
                    (from_fourcc(b"EC0p"), cfg.ec0p),
                    (from_fourcc(b"nCal"), 0x0),
                    (from_fourcc(b"alig"), cfg.alig),
                    (from_fourcc(b"AOPt"), cfg.aopt),
                ],
            )?;
        }
        let rtkit = rtkit::RtKit::<AopData>::new(pdev.as_ref(), None, 0, data.clone())?;
        data.rtkit.lock().as_mut().set(Some(rtkit));
        if cfg.setup_port {
            AopData::setup_open(&data, pdev.as_ref())?;
        }
        asc_mmio.update(ASC_CPU_CONTROL, |r| r.with_const_cpu_run::<1>());
        data.cpu_started.store(true, Release);
        if cfg.setup_port {
            // The RTKit handshake up to AP power on comes first, then the
            // setup port's own power state and endpoint buffers, and only
            // then the AFK endpoints.
            data.wake()?;
            data.setup_finish_boot()?;
            // Before any client reaches the ALS service: a failure here costs
            // the sensor, not the AOP.
            if let Err(e) = data.setup_als_calibration() {
                dev_warn!(pdev.as_ref(), "ALS calibration failed ({:?})", e);
            }
            data.start_afk()?;
        } else {
            data.start()?;
        }
        probe_guard.dismiss();
        let data = data as Arc<dyn AOP>;
        Ok(Self(data))
    }

    fn unbind(_dev: &platform::Device<Core<'_>>, this: Pin<&Self>) {
        // The device is still bound here, which the DMA allocations the
        // teardown frees require. Drop only repeats it as a no-op.
        this.0.remove();
    }
}

impl Drop for AopDriver {
    fn drop(&mut self) {
        self.0.remove();
    }
}

// SAFETY: the wrapped `Arc<dyn AOP>` is `Send + Sync` (the trait requires
// both), so the handle may move to the thread that unbinds the device.
unsafe impl Send for AopDriver {}

module_platform_driver! {
    type: AopDriver,
    name: "apple_aop",
    description: "AOP driver",
    license: "Dual MIT/GPL",
}
