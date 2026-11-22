// SPDX-License-Identifier: GPL-2.0-only OR MIT
#![recursion_limit = "2048"]

//! Apple PMP driver
//!
//! The Rust owner of the PMP RTKit session. It is linked into the pmp module
//! together with the retained J713 power-vote bridge and read-only telemetry.
//!
//! Copyright (C) The Asahi Linux Contributors

use core::mem;
use core::sync::atomic::{fence, AtomicI32, AtomicPtr, Ordering};

mod pmp_bootargs;
mod pmp_power;
use pmp_power::Command;

use kernel::{
    bindings,
    device::{
        self,
        Core, //
    },
    dma::{
        Coherent,
        CoherentBox,
        Device as _,
        DmaMask, //
    },
    error::to_result,
    io::{
        mem::DevresIoMem,
        Io,
        IoBackend,
        IoBase,
        IoSysMap,
        Mmio,
        MmioBackend,
        Region, //
    },
    kvec,
    module_platform_driver,
    new_condvar,
    new_mutex,
    of,
    platform,
    prelude::*,
    soc::apple::rtkit::{
        self,
        ASC_CPU_CONTROL, //
    },
    str::CString,
    sync::{
        aref::ARef,
        Arc,
        CondVar,
        CondVarTimeoutResult,
        Mutex, //
    },
    transmute::{
        AsBytes,
        FromBytes, //
    },
    types::ForeignOwnable, //
};

const PMP_MIN_MMIO_SIZE: usize = 0x80000;
const ASC_MMIO_SIZE: usize = 0x4000;
const BOOTARGS_UUID: usize = 0x204;
const BOOTARGS_UUID_MAGIC: u32 = 0x6469_7575; // "uuid"
const BOOTARGS_UUID_VERSION: u32 = 5;
const BOOTARGS_OFFSET: usize = 0x22c;
const BOOTARGS_SIZE: usize = 0x230;
const ASC_CPU_CONTROL_OFFSET: usize = 0x44;
const PMP_ENDPOINT: u8 = 0x20;
const PM_COMMAND_MARKER: u64 = pmp_power::MARKER;
const PM_COMMAND_SHIFT: u32 = pmp_power::SHIFT;
const OPC_GET_IOVA_TABLE: u64 = 0x10;
const OPC_MALLOC: u64 = 0x12;
const OPC_FREE: u64 = 0x14;
const OPC_SET_BUF: u64 = 0x30;
const OPC_REGISTER_IOREG: u64 = 0x32;
const OPC_SET_IOREG: u64 = 0x34;
const OPC_UPDATE_IOREG: u64 = 0x36;
const OPC_CONFIRM_IOREG: u64 = 0x37;
const OPC_ACK_MASK: u64 = 0x1;
const OPC_SHIFT: u32 = 48;
const MALLOC_SIZE_MASK: u64 = 0xFFFFFF;
const MSG_IOVA_MASK: u64 = 0xFFFFFFFFFFFF;
const SET_IOREG_INDEX_MASK: u64 = 0xFFFF;
const PIO_VM_BASE: u64 = 0xc0000000;
const PIO_GRANULARITY: u64 = 0x1000000;

const fn from_fourcc(b: &[u8; 4]) -> u32 {
    b[3] as u32 | (b[2] as u32) << 8 | (b[1] as u32) << 16 | (b[0] as u32) << 24
}

/// Boot arguments set from an optional property when it is present; the
/// firmware's default for each is 0.
const OPTIONAL_BOOTARGS: [(&[u8; 4], &CStr); 5] = [
    (b"DCAP", c"apple,dram-capacity"),
    (b"DCHD", c"apple,dram-channel-disable"),
    (b"PMC_", c"apple,pmc"),
    (b"PMCX", c"apple,pmc-msg-disabled"),
    (b"CVAR", c"apple,soc-chip-variant"),
];

struct PmpAllocation {
    addr: u64,
    alloc: Coherent<[u8]>,
}

struct IovaTableEntry {
    _host_addr: u64,
    _pio_base: u64,
    _size: u64,
}

// SAFETY: TODO:
unsafe impl AsBytes for IovaTableEntry {}
// SAFETY: TODO:
unsafe impl FromBytes for IovaTableEntry {}

/// A range of the firmware's SRAM as it was when the firmware stopped.
struct SramCopy {
    offset: usize,
    words: KVVec<u32>,
}

struct PmpState {
    iova_table: Option<Coherent<[IovaTableEntry]>>,
    allocs: KVec<PmpAllocation>,
    value_buf: Option<u64>,
    ioreg_entries: KVec<u32>,
    /// Held from a system sleep's start until its end.
    sram: Option<SramCopy>,
}

impl PmpState {
    fn new() -> Result<PmpState> {
        Ok(PmpState {
            iova_table: None,
            allocs: KVec::with_capacity(10, GFP_KERNEL)?,
            value_buf: None,
            ioreg_entries: KVec::with_capacity(340, GFP_KERNEL)?,
            sram: None,
        })
    }
    fn find_alloc(&self, addr: u64) -> Option<usize> {
        // Due to how pmp manages memory, iterating in reverse will
        // usually result in us getting the right one on the first try
        for (i, e) in self.allocs.iter().enumerate().rev() {
            if e.addr == addr {
                return Some(i);
            }
        }
        None
    }
    fn get_buf(&mut self, addr: u64) -> Option<&mut Coherent<[u8]>> {
        let idx = self.find_alloc(addr)?;
        Some(&mut self.allocs[idx].alloc)
    }
}

#[pin_data]
struct PmpData {
    dev: ARef<device::Device>,
    pmp_mmio: DevresIoMem<PMP_MIN_MMIO_SIZE>,
    asc_mmio: DevresIoMem<ASC_MMIO_SIZE>,
    #[pin]
    rtkit: Mutex<Option<rtkit::RtKit<PmpData>>>,
    #[pin]
    state: Mutex<PmpState>,
    #[pin]
    power_command: Mutex<Command>,
    #[pin]
    power_reply: CondVar,
    /// The C power-vote bridge, published once and retained until reboot.
    display: AtomicPtr<bindings::apple_pmp_display>,
    /// The first failure seen before or after the bridge was published.
    display_error: AtomicI32,
    /// See [`PmpHwConfig::sram_backup`].
    sram_backup: Option<(usize, usize)>,
}

impl PmpData {
    fn new(dev: &platform::Device<Core<'_>>, cfg: &PmpHwConfig) -> Result<Arc<PmpData>> {
        let pmp_req = dev.io_request_by_name(c"pmp").ok_or(EINVAL)?;
        let pmp_mmio = pmp_req.iomap_sized::<PMP_MIN_MMIO_SIZE>()?.into_devres()?;
        let asc_req = dev.io_request_by_name(c"asc").ok_or(EINVAL)?;
        let asc_mmio = asc_req.iomap_sized::<ASC_MMIO_SIZE>()?.into_devres()?;
        // The SRAM range kept over a sleep, if the SoC's config has none: from the DT, where
        // the boot loader puts it.
        let sram_backup = match cfg.sram_backup {
            Some(range) => Some(range),
            None => {
                let node = dev.as_ref().fwnode().ok_or(EIO)?;
                let prop_name = c"apple,sram-keep";
                if node.property_present(prop_name) {
                    let v = node
                        .property_read_array_vec::<u32>(prop_name, 2)?
                        .required_by(dev.as_ref())?;
                    Some((v[0] as usize, v[1] as usize))
                } else {
                    None
                }
            }
        };
        Arc::pin_init(
            try_pin_init!(
                PmpData {
                    dev: dev.as_ref().into(),
                    rtkit <- new_mutex!(None),
                    state <- new_mutex!(PmpState::new()?),
                    power_command <- new_mutex!(Command::Idle),
                    power_reply <- new_condvar!(),
                    pmp_mmio,
                    asc_mmio,
                    display: AtomicPtr::new(core::ptr::null_mut()),
                    display_error: AtomicI32::new(0),
                    sram_backup,
                }
            ),
            GFP_KERNEL,
        )
    }
    fn start(&self) -> Result<()> {
        let mut guard = self.rtkit.lock();
        let mut rtk = guard.as_mut().as_pin_mut().unwrap();
        rtk.as_mut().wake()?;
        rtk.start_endpoint(PMP_ENDPOINT)
    }
    fn patch_bootargs(
        &self,
        pmp_mmio: &Mmio<'_, Region<PMP_MIN_MMIO_SIZE>>,
        patches: &[(u32, u32)],
        check_uuid: bool,
    ) -> Result<()> {
        let io = pmp_mmio.as_view().relaxed();
        if check_uuid {
            let magic = io.read32(BOOTARGS_UUID);
            let version = io.read32(BOOTARGS_UUID + 4);
            if magic != BOOTARGS_UUID_MAGIC || version != BOOTARGS_UUID_VERSION {
                dev_err!(
                    self.dev,
                    "no boot argument header (uuid {:#x}, version {})\n",
                    magic,
                    version
                );
                return Err(ENODEV);
            }
        }
        let offset = io.read32(BOOTARGS_OFFSET) as usize;
        let size = io.read32(BOOTARGS_SIZE) as usize;
        if !pmp_bootargs::region_valid(offset, size, io.size()) {
            return Err(EINVAL);
        }
        let mut arg_bytes = kvec![0u8; size]?;
        for (idx, byte) in arg_bytes.iter_mut().enumerate() {
            *byte = io.try_read8(offset + idx)?;
        }
        if !pmp_bootargs::patch(&mut arg_bytes, patches) {
            return Err(EINVAL);
        }
        for (idx, byte) in arg_bytes.iter().enumerate() {
            io.try_write8(*byte, offset + idx)?;
        }
        Ok(())
    }
    fn get_iova_table(&self) -> Result<u64> {
        let mut state = self.state.lock();
        if state.iova_table.is_some() {
            dev_err!(self.dev, "Asked for iova table with existing buffer");
            return Err(EIO);
        }
        let node = self.dev.fwnode().ok_or(EIO)?;
        let mut pio_base = PIO_VM_BASE;
        let prop_name = c"apple,pio-ranges";
        if !node.property_present(prop_name) {
            return Ok((OPC_GET_IOVA_TABLE | OPC_ACK_MASK) << OPC_SHIFT);
        }
        let n_entries = node.property_count_elem::<u64>(prop_name)? / 2;
        let ranges = node
            .property_read_array_vec::<u64>(prop_name, n_entries * 2)?
            .required_by(&self.dev)?;
        // SAFETY: TODO: ensure self.dev is bound
        let bound_dev = unsafe { self.dev.as_bound() };
        let mut table = CoherentBox::<[IovaTableEntry]>::zeroed_slice(bound_dev, 170, GFP_KERNEL)?;

        let domain = unsafe { bindings::iommu_get_domain_for_dev(self.dev.as_raw()) };
        for i in 0..n_entries {
            let host_addr = ranges[i * 2];
            let size = ranges[i * 2 + 1];
            unsafe {
                let err = bindings::iommu_map(
                    domain,
                    pio_base as usize,
                    host_addr,
                    size as usize,
                    (bindings::IOMMU_READ | bindings::IOMMU_WRITE | bindings::IOMMU_MMIO) as i32,
                    bindings::GFP_KERNEL,
                );
                if err != 0 {
                    return Err(Error::from_errno(err));
                }
            }
            table[i] = IovaTableEntry {
                _host_addr: host_addr,
                _pio_base: pio_base,
                _size: size,
            };
            pio_base += PIO_GRANULARITY;
        }
        let table: Coherent<[IovaTableEntry]> = table.into();
        let msg = (OPC_GET_IOVA_TABLE | OPC_ACK_MASK) << OPC_SHIFT | table.dma_address();
        state.iova_table = Some(table);
        Ok(msg)
    }
    fn malloc(&self, size: u64) -> Result<u64> {
        // SAFETY: TODO: ensure self.dev is bound
        let bound_dev = unsafe { self.dev.as_bound() };
        let iomem = Coherent::<u8>::zeroed_slice(bound_dev, size as usize, GFP_KERNEL)?;
        let mut state = self.state.lock();
        let addr = iomem.dma_address();
        let msg = (OPC_MALLOC | OPC_ACK_MASK) << OPC_SHIFT | addr;
        state.allocs.push(
            PmpAllocation {
                addr: addr,
                alloc: iomem,
            },
            GFP_KERNEL,
        )?;
        Ok(msg)
    }
    fn free(&self, addr: u64) -> Result<u64> {
        let mut state = self.state.lock();
        if let Some(idx) = state.find_alloc(addr) {
            state.allocs.swap_remove(idx);
        } else {
            dev_err!(
                self.dev,
                "Attempted to free memory that was not allocated {}",
                addr
            );
            return Err(EIO);
        }
        let msg = (OPC_FREE | OPC_ACK_MASK) << OPC_SHIFT;
        Ok(msg)
    }
    fn set_buf(&self, addr: u64) -> Result<u64> {
        let mut state = self.state.lock();
        if state.value_buf.is_some() {
            dev_err!(self.dev, "Setting a buffer when one exists");
            return Err(EIO);
        }
        let ptr_buf = if let Some(s) = state.get_buf(addr) {
            s
        } else {
            dev_err!(self.dev, "Unable to find buffer");
            return Err(EIO);
        };
        if ptr_buf.size() < mem::size_of::<u64>() {
            dev_err!(self.dev, "Buffer too small");
            return Err(EIO);
        }
        let ptr = unsafe { *(ptr_buf.as_ptr() as *const u64) };
        state.value_buf = Some(ptr);
        let msg = (OPC_SET_BUF | OPC_ACK_MASK) << OPC_SHIFT;
        Ok(msg)
    }
    fn register_ioreg(&self, addr: u64) -> Result<u64> {
        let mut state = self.state.lock();
        let msg_buf = if let Some(s) = state.get_buf(addr) {
            s
        } else {
            dev_err!(self.dev, "Unable to find buffer");
            return Err(EIO);
        };
        if msg_buf.size() < 0x44 {
            dev_err!(self.dev, "Buffer too small");
            return Err(EIO);
        }
        // SAFETY: TODO
        let mut size = u32::from_le_bytes(unsafe {
            <&[u8] as TryInto<[u8; 4]>>::try_into(&msg_buf.as_ref()[0x40..0x44]).unwrap()
        });
        if size == 0 {
            let mut name_vec = KVec::with_capacity(0x31, GFP_KERNEL)?;
            name_vec
                .extend_from_slice(unsafe { &msg_buf.as_ref()[0..0x30] }, GFP_KERNEL)
                .unwrap();
            name_vec.push(0, GFP_KERNEL).unwrap();
            let name_str = CStr::from_bytes_until_nul(&name_vec).unwrap();
            let name_str = CString::try_from_fmt(fmt!("apple,tunable-{name_str}"))?;
            let node = self.dev.fwnode().ok_or(EIO)?;
            if state.value_buf.is_none() {
                dev_err!(self.dev, "Value buf not set");
                return Err(EIO);
            }
            let val_buf_addr = state.value_buf.unwrap();
            let val_buf = if let Some(s) = state.get_buf(val_buf_addr) {
                s
            } else {
                dev_err!(self.dev, "Unable to find value buffer");
                return Err(EIO);
            };
            if node.property_present(&name_str) {
                let len = node.property_count_elem::<u8>(&name_str)?;
                if len > val_buf.len() {
                    // Answered as missing, which the firmware tolerates.
                    dev_warn!(
                        self.dev,
                        "property {:?} ({} bytes) does not fit the firmware buffer ({} bytes)\n",
                        name_str,
                        len,
                        val_buf.len()
                    );
                } else {
                    let data = node
                        .property_read_array_vec::<u8>(&name_str, len)?
                        .required_by(&self.dev)?;
                    unsafe {
                        val_buf.as_mut()[0..len].copy_from_slice(&data);
                    }
                    size = len as u32;
                }
            } else {
                // Not every boot loader provides every tunable the firmware
                // asks for, and the firmware starts without them.
                dev_dbg!(self.dev, "unknown property {:?}", name_str);
            }
        }
        state.ioreg_entries.push(size, GFP_KERNEL)?;
        let index = state.ioreg_entries.len() as u64;
        let msg = (OPC_REGISTER_IOREG | OPC_ACK_MASK) << OPC_SHIFT | (index << 32) | size as u64;
        Ok(msg)
    }
    fn set_ioreg(&self, index: u64) -> Result<u64> {
        let len = *self
            .state
            .lock()
            .ioreg_entries
            .get(index as usize)
            .ok_or(EIO)? as u64;
        let msg = (OPC_SET_IOREG | OPC_ACK_MASK) << OPC_SHIFT | len;
        Ok(msg)
    }
    fn fail_display(&self, error: Error) {
        // Publish failure before looking up either lazily registered consumer.
        // Pair with the post-registration fence below: registration cannot miss
        // the error while this path also misses the newly published provider.
        let _ = self.display_error.compare_exchange(
            0,
            error.to_errno(),
            Ordering::SeqCst,
            Ordering::SeqCst,
        );
        fence(Ordering::SeqCst);
        // SAFETY: a published telemetry provider is retained by this PMP device.
        unsafe { bindings::apple_pmp_telemetry_stop(self.dev.as_raw()) };
        *self.power_command.lock() = Command::Failed;
        self.power_reply.notify_all();
        // SAFETY: NULL is accepted, and a published bridge is never freed in this epoch.
        unsafe {
            bindings::apple_pmp_display_fail(
                self.display.load(Ordering::SeqCst),
                self.display_error.load(Ordering::SeqCst),
            )
        };
    }
    /// The handle, to wait on the firmware with the lock free: its messages take that lock.
    fn rtkit_handle(&self) -> Result<*mut bindings::apple_rtkit> {
        let mut guard = self.rtkit.lock();
        let rtk = guard.as_mut().as_pin_mut().ok_or(ENODEV)?;
        Ok(rtk.as_raw())
    }
    /// The ASC's CPU control register.
    fn asc_cpu_control(&self) -> Result<*mut kernel::ffi::c_void> {
        // SAFETY: the PM core only calls a bound device.
        let bound_dev = unsafe { self.dev.as_bound() };
        let asc_mmio = self.asc_mmio.access(bound_dev)?.as_view();
        Ok(MmioBackend::as_ptr(asc_mmio)
            .cast::<u8>()
            .wrapping_add(ASC_CPU_CONTROL_OFFSET)
            .cast())
    }
    /// Room for what SRAM needs kept, taken first so that running out refuses a sleep cleanly.
    fn sram_alloc(&self, sram_size: usize) -> Result<Option<SramCopy>> {
        let Some((offset, size)) = self.sram_backup else {
            return Ok(None);
        };
        if !pmp_bootargs::word_range_valid(offset, size, sram_size) {
            return Err(EINVAL);
        }
        let mut words = KVVec::new();
        words.resize(size / 4, 0u32, GFP_KERNEL)?;
        Ok(Some(SramCopy { offset, words }))
    }
    /// Stops the firmware for a system sleep, then keeps what its restart needs from SRAM.
    fn sleep(&self) -> Result {
        // SAFETY: the PM core only calls a bound device.
        let bound_dev = unsafe { self.dev.as_bound() };
        let pmp_mmio = self.pmp_mmio.access(bound_dev)?.as_view();
        let io = pmp_mmio.as_view().relaxed();
        let mut copy = self.sram_alloc(io.size())?;
        let rtk = self.rtkit_handle()?;
        let control = self.asc_cpu_control()?;
        // SAFETY: the handle stays valid until unbind, which can't run during a system sleep,
        // and `control` is inside the ASC mapping held by `self`.
        if let Err(error) = to_result(unsafe { bindings::apple_rtkit_sleep_stop(rtk, control) }) {
            // SAFETY: as above.
            if to_result(unsafe { bindings::apple_rtkit_sleep_return(rtk, control) }).is_err() {
                self.fail_display(error);
            }
            return Err(error);
        }
        if let Some(copy) = copy.as_mut() {
            for (idx, word) in copy.words.iter_mut().enumerate() {
                // The range was checked against the mapping when it was allocated.
                *word = io.try_read32(copy.offset + idx * 4).unwrap_or(0);
            }
        }
        self.state.lock().sram = copy;
        Ok(())
    }
    /// For a stop at the commit: the RTKit handle, the CPU control and the SRAM copy's room,
    /// whose buffer stays where it is once moved into the state.
    fn sleep_prepare(&self, stop: &mut bindings::apple_pmp_commit_stop) -> Result {
        // SAFETY: the PM core only calls a bound device.
        let bound_dev = unsafe { self.dev.as_bound() };
        let pmp_mmio = self.pmp_mmio.access(bound_dev)?.as_view();
        let size = pmp_mmio.as_view().relaxed().size();
        let mut copy = self.sram_alloc(size)?;
        stop.rtk = self.rtkit_handle()?;
        stop.control = self.asc_cpu_control()?;
        stop.sram = core::ptr::null_mut();
        stop.words = core::ptr::null_mut();
        stop.count = 0;
        if let Some(copy) = copy.as_mut() {
            stop.sram = MmioBackend::as_ptr(pmp_mmio)
                .cast::<u8>()
                .wrapping_add(copy.offset)
                .cast();
            stop.words = copy.words.as_mut_ptr();
            stop.count = copy.words.len();
        }
        self.state.lock().sram = copy;
        Ok(())
    }
    /// Writes SRAM back after a sleep that took its power, while the firmware is stopped.
    fn sram_restore(&self) -> Result {
        let Some(copy) = self.state.lock().sram.take() else {
            return Ok(());
        };
        // SAFETY: the PM core only calls a bound device.
        let bound_dev = unsafe { self.dev.as_bound() };
        let pmp_mmio = self.pmp_mmio.access(bound_dev)?.as_view();
        let io = pmp_mmio.as_view().relaxed();
        for (idx, word) in copy.words.iter().enumerate() {
            io.try_write32(*word, copy.offset + idx * 4)?;
        }
        // The firmware mustn't start ahead of writes still on their way.
        let _ = io.try_read32(copy.offset);
        dev_dbg!(self.dev, "SRAM restored: {} bytes\n", copy.words.len() * 4);
        Ok(())
    }
    /// Brings the firmware back after a system sleep. It keeps its endpoints and everything the
    /// host gave it, so nothing here repeats what it asked for at probe.
    fn wake(&self) -> Result {
        self.sram_restore()?;
        let rtk = self.rtkit_handle()?;
        let control = self.asc_cpu_control()?;
        // SAFETY: the handle stays valid until unbind, which can't run during a system sleep,
        // and `control` is inside the ASC mapping held by `self`.
        to_result(unsafe { bindings::apple_rtkit_sleep_return(rtk, control) })
    }
    fn recv_message(&self, msg: u64) -> Result<()> {
        let pm_command = (msg >> PM_COMMAND_SHIFT) & 0xff;
        if msg & PM_COMMAND_MARKER != 0 && (pm_command == 0x0f || pm_command == 0x10) {
            let mut state = self.power_command.lock();
            state.reply(msg);
            if *state == Command::Failed {
                dev_err!(
                    self.dev,
                    "Unexpected PMP device-state reply {:#018x}; power commands disabled",
                    msg
                );
                drop(state);
                self.fail_display(EPROTO);
            } else {
                self.power_reply.notify_all();
            }
            return Ok(());
        }
        let opc = (msg >> OPC_SHIFT) & 0xFF;
        let reply = match opc {
            OPC_GET_IOVA_TABLE => self.get_iova_table()?,
            OPC_MALLOC => self.malloc(msg & MALLOC_SIZE_MASK)?,
            OPC_FREE => self.free(msg & MSG_IOVA_MASK)?,
            OPC_SET_BUF => self.set_buf(msg & MSG_IOVA_MASK)?,
            OPC_REGISTER_IOREG => self.register_ioreg(msg & MSG_IOVA_MASK)?,
            OPC_SET_IOREG => self.set_ioreg(msg & SET_IOREG_INDEX_MASK)?,
            // Firmware waits for this confirmation before publishing reports.
            // Do not echo the registry index/payload or add a generic ACK bit.
            OPC_UPDATE_IOREG => OPC_CONFIRM_IOREG << OPC_SHIFT,
            _ => {
                dev_err!(self.dev, "Got unknown message {}", msg);
                return Err(EIO);
            }
        };
        let mut rtk_guard = self.rtkit.lock();
        let rtk = rtk_guard.as_mut().as_pin_mut().unwrap();
        rtk.send_message(PMP_ENDPOINT, reply)?;
        Ok(())
    }
}

unsafe extern "C" fn display_send(cookie: *const kernel::ffi::c_void, word: u64) -> i32 {
    // SAFETY: the C bridge holds this Arc's foreign reference until reboot.
    let _data = unsafe { Arc::<PmpData>::borrow(cookie.cast_mut()) };
    // Use the shared, serialized PMP command path for all power consumers.
    // It validates supplier readiness and consumes the exact untagged reply.
    match word {
        0x0020f50100000000 => unsafe { bindings::apple_pmp_set_device_power(0x0f, 5, 1) },
        0x0020f80100000000 => unsafe { bindings::apple_pmp_set_device_power(0x0f, 8, 1) },
        // DISP released while its DCP has powered the display domains down.
        0x0020f80000000000 => unsafe { bindings::apple_pmp_set_device_power(0x0f, 8, 0) },
        _ => EINVAL.to_errno(),
    }
}

/// Private bridge entry: resolve typed data only after the PMP provider has bound.
///
/// # Safety
/// `raw` is a live, fully bound device with its device lock held by the caller.
#[kernel::macros::export]
unsafe extern "C" fn apple_pmp_display_bound_owner(
    raw: *mut bindings::device,
) -> *mut bindings::apple_pmp_display {
    // SAFETY: guaranteed by the C caller under the supplier device lock.
    let dev = unsafe { device::Device::<device::Normal>::from_raw(raw) };
    // SAFETY: the C caller checked the bound PMP driver and holds its device lock.
    let ptr = dev.get_drvdata::<PmpDriver>();
    if ptr.is_null() {
        return core::ptr::null_mut();
    }
    // SAFETY: the checked PMP driver stored this pinned PmpDriver, and the
    // device lock excludes unbind for the entire read-only borrow.
    let driver = unsafe { &*ptr };
    let data = &driver.0;
    let existing = data.display.load(Ordering::Acquire);
    if !existing.is_null() {
        return existing;
    }
    let cookie = data.clone().into_foreign();
    // SAFETY: callback borrows this Arc; registration validates identity and sends nothing.
    let display = unsafe { bindings::apple_pmp_display_register(raw, cookie, Some(display_send)) };
    if display.is_null() {
        // SAFETY: registration did not take ownership of the foreign reference.
        drop(unsafe { Arc::<PmpData>::from_foreign(cookie) });
    } else {
        data.display.store(display, Ordering::SeqCst);
        let error = data.display_error.load(Ordering::SeqCst);
        if error != 0 {
            // SAFETY: the bridge is now retained and must inherit earlier failures.
            unsafe { bindings::apple_pmp_display_fail(display, error) };
        }
    }
    display
}

unsafe impl Send for PmpData {}
unsafe impl Sync for PmpData {}

/// Send a power request while the C bridge serializes the untagged reply stream.
/// A lost reply disables further requests until the PMP session is recreated.
///
/// # Safety
/// `data` must point to the live PmpData registered by PmpDriver.
#[export]
pub unsafe extern "C" fn apple_pmp_send_power_command(
    data: *const core::ffi::c_void,
    command: u8,
    device_id: u16,
    enabled: u32,
) -> i32 {
    // SAFETY: The C bridge serializes calls against unregister in Drop.
    let data = unsafe { &*data.cast::<PmpData>() };
    let message = PM_COMMAND_MARKER
        | (u64::from(command) << PM_COMMAND_SHIFT)
        | (u64::from(device_id) << 40)
        | (u64::from(enabled) << 32);
    {
        let mut state = data.power_command.lock();
        if !state.begin(command) {
            return EIO.to_errno();
        }
    }
    let sent = {
        let mut guard = data.rtkit.lock();
        match guard.as_mut().as_pin_mut() {
            Some(rtk) => rtk.send_message(PMP_ENDPOINT, message),
            None => Err(ENODEV),
        }
    };
    let mut state = data.power_command.lock();
    if let Err(error) = sent {
        *state = Command::Failed;
        drop(state);
        data.fail_display(error);
        return error.to_errno();
    }
    let mut remaining = kernel::time::msecs_to_jiffies(1000);
    while matches!(*state, Command::Pending(_)) {
        match data.power_reply.wait_timeout(&mut state, remaining) {
            CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
            CondVarTimeoutResult::Timeout | CondVarTimeoutResult::Signal { .. } => {
                // A reply may have arrived while the waiter reacquired its
                // mutex. Consume it if complete; otherwise never reuse the
                // untagged stream after an ambiguous wait.
                if *state != Command::Complete {
                    *state = Command::Failed;
                    dev_err!(
                        data.dev,
                        "PMP device-power request {:#018x} did not complete",
                        message
                    );
                    drop(state);
                    data.fail_display(ETIMEDOUT);
                    return ETIMEDOUT.to_errno();
                }
            }
        }
    }
    if *state != Command::Complete {
        drop(state);
        data.fail_display(EIO);
        return EIO.to_errno();
    }
    *state = Command::Idle;
    dev_dbg!(
        data.dev,
        "PMP device={} enabled={} acknowledged",
        device_id,
        enabled
    );
    0
}

/// Stops the firmware for a system sleep.
///
/// # Safety
/// `data` must point to the live PmpData registered by PmpDriver.
#[export]
pub unsafe extern "C" fn apple_pmp_sleep(data: *const core::ffi::c_void) -> i32 {
    // SAFETY: The C bridge serializes calls against unregister in Drop.
    let data = unsafe { &*data.cast::<PmpData>() };
    match data.sleep() {
        Ok(()) => 0,
        Err(error) => error.to_errno(),
    }
}

/// Takes what the sleep's last step needs to stop the firmware with interrupts off, and the room
/// for the SRAM it keeps. Nothing is stopped here.
///
/// # Safety
/// `data` must point to the live PmpData registered by PmpDriver, and `stop` to writable memory.
#[export]
pub unsafe extern "C" fn apple_pmp_sleep_prepare(
    data: *const core::ffi::c_void,
    stop: *mut bindings::apple_pmp_commit_stop,
) -> i32 {
    // SAFETY: The C bridge serializes calls against unregister in Drop.
    let data = unsafe { &*data.cast::<PmpData>() };
    // SAFETY: The caller passes a valid pointer.
    let stop = unsafe { &mut *stop };
    match data.sleep_prepare(stop) {
        Ok(()) => 0,
        Err(error) => error.to_errno(),
    }
}

/// Brings the firmware back after a system sleep. A firmware that doesn't come back disables
/// further power commands.
///
/// # Safety
/// `data` must point to the live PmpData registered by PmpDriver.
#[export]
pub unsafe extern "C" fn apple_pmp_wake(data: *const core::ffi::c_void) -> i32 {
    // SAFETY: The C bridge serializes calls against unregister in Drop.
    let data = unsafe { &*data.cast::<PmpData>() };
    match data.wake() {
        Ok(()) => 0,
        Err(error) => {
            data.fail_display(error);
            error.to_errno()
        }
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
impl rtkit::Operations for PmpData {
    type Data = Arc<PmpData>;
    type Buffer = NoBuffer;

    fn recv_message(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, _ep: u8, msg: u64) {
        let ret = data.recv_message(msg);
        if let Err(e) = ret {
            // Only the retained J713 bridge latches a message it cannot
            // handle as fatal. Other boards log it and keep power commands.
            // SAFETY: read-only board identity check.
            if unsafe { bindings::apple_pmp_display_retained_board() } {
                data.fail_display(e);
            }
            dev_err!(data.dev, "Failed to handle rtkit message, error: {:?}", e);
        }
    }

    fn crashed(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, _crashlog: Option<&[u8]>) {
        data.fail_display(EIO);
        dev_err!(data.dev, "PMP firmware crashed");
    }
}

#[allow(dead_code)]
struct PmpDriver(Arc<PmpData>);

/// Per-SoC configuration.
struct PmpHwConfig {
    /// DMA mask to set before the firmware allocates anything, or `None` to
    /// keep the default.
    dma_mask: Option<DmaMask>,
    /// Check the firmware's `uuid` header before trusting the boot argument
    /// block it points to.
    check_bootargs_uuid: bool,
    /// When the firmware needs system endpoint 7 (CoreAnalytics) started
    /// and given its buffer before it acknowledges the IOP power state.
    coreanalytics: Ep7,
    /// The range of the firmware's SRAM, as (offset, size) in bytes, that a system sleep keeps
    /// and writes back. `None` where no sleep has been traced.
    sram_backup: Option<(usize, usize)>,
}

/// Whether a PMP firmware needs endpoint 7 to finish its RTKit boot.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Ep7 {
    /// Never started.
    No,
    /// Only on the retained J713 board.
    RetainedBoard,
    /// On every board with this SoC.
    Always,
}

const HW_CFG_DEFAULT: PmpHwConfig = PmpHwConfig {
    dma_mask: None,
    check_bootargs_uuid: false,
    coreanalytics: Ep7::No,
    sram_backup: None,
};

/// T8140: the DART translates 42 bits and its DMA window starts at 1 TiB.
const HW_CFG_T8140: PmpHwConfig = PmpHwConfig {
    dma_mask: Some(DmaMask::new::<42>()),
    check_bootargs_uuid: false,
    coreanalytics: Ep7::No,
    sram_backup: Some((0x3c000, 0x5c000)),
};

/// T6041: the DART window starts at 1 TiB; the boot argument header is
/// version 5. The firmware advertises endpoint 7 and does not acknowledge
/// the IOP power state (-ETIME) until it is started and has its buffer.
const HW_CFG_T6041: PmpHwConfig = PmpHwConfig {
    dma_mask: Some(DmaMask::new::<42>()),
    check_bootargs_uuid: true,
    coreanalytics: Ep7::Always,
    sram_backup: None,
};

/// T8142: the DART window starts at 1 TiB as on T8140. Its registry-v2
/// firmware starts endpoint 7 and takes its 8 KiB buffer before it
/// acknowledges the IOP power state.
const HW_CFG_T8142: PmpHwConfig = PmpHwConfig {
    dma_mask: Some(DmaMask::new::<42>()),
    check_bootargs_uuid: false,
    coreanalytics: Ep7::Always,
    sram_backup: None,
};

/// T6050: the DART window starts at 1 TiB (ADT dart-pmp0 vm-base) and spans
/// 64 GiB, as on T8142. Its firmware never readies endpoint 7 (-ETIME). The
/// SRAM it keeps over a sleep comes from the DT (`apple,sram-keep`).
const HW_CFG_T6050: PmpHwConfig = PmpHwConfig {
    dma_mask: Some(DmaMask::new::<42>()),
    check_bootargs_uuid: false,
    coreanalytics: Ep7::No,
    sram_backup: None,
};

/// T8132: the sleep engine reloads `__TEXT` from DRAM at the wake, and only `__DATA`
/// (the ADT's second segment) has to be kept.
const HW_CFG_T8132: PmpHwConfig = PmpHwConfig {
    dma_mask: None,
    check_bootargs_uuid: false,
    coreanalytics: Ep7::RetainedBoard,
    sram_backup: Some((0x3c000, 0x68000)),
};

impl Drop for PmpDriver {
    fn drop(&mut self) {
        // SAFETY: The Arc remains live until unregister has joined any caller.
        unsafe { bindings::apple_pmp_unregister(Arc::as_ptr(&self.0).cast()) };
    }
}

kernel::of_device_table!(
    OF_TABLE,
    <PmpDriver as platform::Driver>::IdInfo,
    [
        (of::DeviceId::new(c"apple,t8132-pmp-v2"), &HW_CFG_T8132),
        (of::DeviceId::new(c"apple,t8140-pmp-v2"), &HW_CFG_T8140),
        (of::DeviceId::new(c"apple,t8142-pmp-v2"), &HW_CFG_T8142),
        (of::DeviceId::new(c"apple,t6041-pmp-v2"), &HW_CFG_T6041),
        (of::DeviceId::new(c"apple,t6050-pmp-v2"), &HW_CFG_T6050),
        (of::DeviceId::new(c"apple,t6000-pmp-v2"), &HW_CFG_DEFAULT),
    ]
);

impl platform::Driver for PmpDriver {
    type IdInfo = &'static PmpHwConfig;
    type Data<'bound> = PmpDriver;

    fn suppress_bind_attrs() -> bool {
        // Retained J713 power votes have no teardown path, and an unbound PMP that registered
        // with the sleep driver still runs.
        // SAFETY: read-only machine identity and device tree checks.
        unsafe { bindings::apple_pmp_keep_bound() }
    }

    fn pm_ops() -> Option<&'static bindings::dev_pm_ops> {
        // SAFETY: a static in `pmp-export.c`, never written.
        Some(unsafe { &*core::ptr::addr_of!(bindings::apple_pmp_pm_ops) })
    }

    const OF_ID_TABLE: Option<of::IdTable<Self::IdInfo>> = Some(&OF_TABLE);

    fn probe<'bound>(
        pdev: &'bound platform::Device<Core<'_>>,
        info: Option<&'bound Self::IdInfo>,
    ) -> impl PinInit<Self::Data<'bound>, Error> + 'bound {
        let dev: ARef<device::Device> = pdev.as_ref().into();
        let cfg = info.ok_or(ENODEV)?;
        if let Some(mask) = cfg.dma_mask {
            // SAFETY: No DMA allocations or mappings exist for this device
            // yet.
            unsafe { pdev.dma_set_mask_and_coherent(mask)? };
        }
        let data = PmpData::new(pdev, cfg)?;
        let pmp_mmio = data.pmp_mmio.access(pdev.as_ref())?.as_view();
        let asc_mmio = data.asc_mmio.access(pdev.as_ref())?.as_view();
        let node = dev.fwnode().ok_or(EIO)?;
        let dvid = node
            .property_read(c"apple,dram-vendor-id")
            .required_by(&dev)?;
        let bdid = node.property_read(c"apple,board-id").required_by(&dev)?;
        let mut patches = KVec::with_capacity(9, GFP_KERNEL)?;
        patches.push((from_fourcc(b"BDID"), bdid), GFP_KERNEL)?;
        patches.push((from_fourcc(b"DVID"), dvid), GFP_KERNEL)?;
        for (tag, prop) in OPTIONAL_BOOTARGS.iter() {
            if let Some(v) = node.property_read::<u32>(prop).optional() {
                patches.push((from_fourcc(tag), v), GFP_KERNEL)?;
            }
        }
        // PMCV and PMCB are bits 0 and 3 of the PMGR's pmc-pmgr word.
        if let Some(v) = node.property_read::<u32>(c"apple,pmc-pmgr").optional() {
            patches.push((from_fourcc(b"PMCV"), v & 1), GFP_KERNEL)?;
            patches.push((from_fourcc(b"PMCB"), (v >> 3) & 1), GFP_KERNEL)?;
        }
        data.patch_bootargs(&pmp_mmio, &patches, cfg.check_bootargs_uuid)?;
        // EP7 is a startup dependency on every T6041 PMP, and on the
        // retained J713 PMP among the T8132 boards.
        let needs_coreanalytics = match cfg.coreanalytics {
            Ep7::No => false,
            Ep7::Always => true,
            // SAFETY: the predicate only reads machine compatibility.
            Ep7::RetainedBoard => unsafe { bindings::apple_pmp_telemetry_retained_board() },
        };
        let rtkit = if needs_coreanalytics {
            rtkit::RtKit::<PmpData>::new_with_coreanalytics(&dev, None, 0, data.clone())?
        } else {
            rtkit::RtKit::<PmpData>::new(&dev, None, 0, data.clone())?
        };
        data.rtkit.lock().as_mut().set(Some(rtkit));
        asc_mmio
            .relaxed()
            .update(ASC_CPU_CONTROL, |r| r.with_const_cpu_run::<1>());
        data.start()?;
        // SAFETY: the PMP owner retains both Devres mappings; managed child
        // teardown and provider cleanup precede device mapping revocation.
        to_result(unsafe {
            bindings::apple_pmp_telemetry_register(
                dev.as_raw(),
                MmioBackend::as_ptr(pmp_mmio).cast(),
                pmp_mmio.size(),
                MmioBackend::as_ptr(asc_mmio).cast(),
            )
        })?;
        fence(Ordering::SeqCst);
        if data.display_error.load(Ordering::SeqCst) != 0 {
            // SAFETY: registration returned the live managed provider.
            unsafe { bindings::apple_pmp_telemetry_stop(dev.as_raw()) };
        }
        dev_info!(dev, "PMP firmware and endpoint ready");
        // SAFETY: PmpDriver retains this Arc and unregisters it before dropping it.
        let ret = unsafe { bindings::apple_pmp_register(dev.as_raw(), Arc::as_ptr(&data).cast()) };
        if ret != 0 {
            return Err(Error::from_errno(ret));
        }
        Ok(PmpDriver(data))
    }

    fn unbind<'bound>(_dev: &'bound platform::Device<Core<'_>>, this: Pin<&Self::Data<'bound>>) {
        this.0.fail_display(ENODEV);
    }
}

module_platform_driver! {
    type: PmpDriver,
    name: "apple_pmp",
    description: "Apple Power Management Processor",
    license: "Dual MIT/GPL",
}
