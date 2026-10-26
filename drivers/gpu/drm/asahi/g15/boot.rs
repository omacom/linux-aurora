// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Admission and firmware start to READY.
//!
//! Admission (no firmware-visible writes): machine check (the T6030 board check
//! `apple_pmp_t6030_board`, or the SoC's machine compatible), the PMP supplier on a SoC with
//! the PMP AGX vote (`apple_pmp_j514s_gpu_prepare`, may return EPROBE_DEFER), the GPU regions and
//! segments ([`super::adt`]: the ADT export, or the GPU device tree node), exact firmware TEXT
//! identity ([`super::identity`]), MMIO windows, runtime-PM hold of the GFX domain.
//!
//! Start: PMP AGX vote (`apple_pmp_j514s_gpu_get`) where the SoC takes one, hardware identity
//! and core masks (one word, or two above 32 core slots), `sgx_open`
//! 0x70001, cold-only check (ASC RUN clear, TTBAT all zero), UAT adoption and the PTE self-check
//! (nothing firmware-visible yet), RTKit init (the node's mailbox), wake, endpoints 0x20/0x21,
//! PPL/UAT bootstrap ([`super::uat`]), initdata graph ([`super::initdata`]), EP 0x20
//! `0x0081 << 48 | ROOT`, DEVCTL 0x13 then 9, readiness (`Runtime+0x314 == 0xabcdabcd` and
//! `Flags+4 == 1`, 2 s, draining rings), DEVCTL 0x13 again.
//!
//! RTKit receive: EP 0x20 `0x0042 << 48` in interrupt context only kicks the poll work
//! ([`super::Shared::kick`]); a crash latches fail-stop. The callbacks are registered through
//! the C RTKit API directly: the only RTKit buffer accepted is the inherited one inside firmware
//! DATA, mapped as I/O memory, which the Rust RTKit abstraction (buffers in system memory only)
//! cannot express.
//!
//! Once the firmware has been touched nothing it can reach is ever freed: a failed start, or a
//! failed backend start, keeps the session as an [`Orphan`] whose rings are still drained.

use core::ffi::c_void;
use core::sync::atomic::Ordering;

use kernel::{
    bindings, device,
    device::Core,
    error::{from_err_ptr, to_result},
    platform,
    prelude::*,
    sync::{aref::ARef, Arc, ArcBorrow},
    time::{delay::fsleep, Delta, Instant, Monotonic},
};

use super::adt::{self, GpuAdt};
use super::channel::{self, Channel, Channels};
use super::event::Events;
use super::fw::channels::{devctl, DevCtl};
use super::fw::initdata::runtime;
use super::hw;
use super::initdata::{self, InitData};
use super::object::Objects;
use super::{identity, regs, uat, Config, Shared};
use crate::driver::AsahiDrmDevice as AsahiDevice;
use crate::mmu;

/// The C shim (`g15/glue.c`): helpers without Rust bindings.
pub(crate) mod glue {
    use core::sync::atomic::{AtomicBool, Ordering};
    use kernel::{device, prelude::*};

    extern "C" {
        fn asahi_g15_sha256(data: *const u8, len: usize, out: *mut u8);
        fn asahi_g15_board() -> bool;
        fn asahi_g15_machine(compatible: *const core::ffi::c_char) -> bool;
        fn asahi_g15_hold_power(dev: *mut core::ffi::c_void);
        fn asahi_g15_release_power(dev: *mut core::ffi::c_void);
        fn asahi_g15_module_try_get() -> bool;
        fn asahi_g15_module_put();
        pub(crate) fn asahi_g15_reprobe();
        pub(crate) fn asahi_g15_report_loader_gpu();
        fn asahi_g15_adt_phram(start: *mut u64, size: *mut u64) -> core::ffi::c_int;
        fn asahi_g15_ram_disjoint(start: u64, size: u64) -> bool;
        fn asahi_g15_ram_within(start: u64, size: u64) -> bool;
        fn asahi_g15_memory_mode(start: u64, size: u64) -> u32;
        fn asahi_g15_ioremap_wc(start: u64, size: usize) -> *mut core::ffi::c_void;
        fn asahi_g15_dma_coherent(dev: *mut core::ffi::c_void) -> bool;
        pub(crate) fn asahi_g15_pmp_prepare(dev: *mut core::ffi::c_void) -> core::ffi::c_int;
        pub(crate) fn asahi_g15_pmp_get(dev: *mut core::ffi::c_void) -> core::ffi::c_int;
        pub(crate) fn asahi_g15_pmp_ready() -> core::ffi::c_int;
        pub(crate) fn asahi_g15_pmp_put(dev: *mut core::ffi::c_void);
        fn asahi_g15_reserved_uat(index: u32, start: u64, size: u64) -> bool;
    }

    /// SHA-256 (`sha256()`).
    pub(crate) fn sha256(data: &[u8]) -> [u8; 32] {
        let mut out = [0u8; 32];
        // SAFETY: `data` is valid for its length and `out` holds the 32-byte digest.
        unsafe { asahi_g15_sha256(data.as_ptr(), data.len(), out.as_mut_ptr()) };
        out
    }

    /// A J514S/J516S T6030 board (`apple_pmp_t6030_board()`).
    pub(crate) fn board() -> bool {
        // SAFETY: Reads the machine compatible strings only.
        unsafe { asahi_g15_board() }
    }

    /// Exact enabled UAT reservation from the maintained boot-loader handoff.
    pub(crate) fn reserved_uat(index: u32, start: u64, size: u64) -> bool {
        // SAFETY: Queries and releases OF nodes; no physical memory is accessed.
        unsafe { asahi_g15_reserved_uat(index, start, size) }
    }

    /// Whether the machine carries `compatible` (`of_machine_is_compatible()`).
    pub(crate) fn machine(compatible: &CStr) -> bool {
        // SAFETY: Reads the machine compatible strings only; the string is NUL-terminated.
        unsafe { asahi_g15_machine(compatible.as_char_ptr()) }
    }

    /// Holds the GFX power domain for the retained session.
    pub(crate) fn hold_power(dev: &device::Device) {
        // SAFETY: `dev` is a valid device; runtime-PM accounting on it has no other requirement.
        unsafe { asahi_g15_hold_power(dev.as_raw().cast()) };
    }

    /// Gives the GFX power domain hold back after a start that touched nothing firmware-visible.
    pub(crate) fn release_power(dev: &device::Device) {
        // SAFETY: `dev` is the device `hold_power` was called with.
        unsafe { asahi_g15_release_power(dev.as_raw().cast()) };
    }

    static RETAINED: AtomicBool = AtomicBool::new(false);

    /// Whether the module is pinned: the firmware has been touched.
    pub(crate) fn retained() -> bool {
        RETAINED.load(Ordering::Acquire)
    }

    /// Pins this module until reboot, once (`try_module_get(THIS_MODULE)`, see `retain.rs`).
    /// ENODEV, with nothing pinned or marked, once the module's removal has begun.
    pub(crate) fn retain_module() -> Result {
        crate::g15::retain::retain(
            &RETAINED,
            // SAFETY: Takes a reference to this module unless its removal has begun.
            || unsafe { asahi_g15_module_try_get() },
            // SAFETY: Drops only the reference this call took; the winner's is never put.
            || unsafe { asahi_g15_module_put() },
        )
        .map_err(|_| ENODEV)
    }

    /// The phram "adt" export: start and size.
    pub(crate) fn adt_phram() -> Result<(u64, u64)> {
        let (mut start, mut size) = (0u64, 0u64);
        // SAFETY: Both pointers are valid for a u64 write.
        kernel::error::to_result(unsafe { asahi_g15_adt_phram(&mut start, &mut size) })?;
        Ok((start, size))
    }

    /// The physical range is disjoint from System RAM.
    pub(crate) fn ram_disjoint(start: u64, size: u64) -> bool {
        // SAFETY: Only queries the resource tree.
        unsafe { asahi_g15_ram_disjoint(start, size) }
    }

    /// Every page is valid System RAM and remains reserved from the allocator.
    pub(crate) fn ram_within(start: u64, size: u64) -> bool {
        // SAFETY: Only queries the resource tree.
        unsafe { asahi_g15_ram_within(start, size) }
    }

    /// WB for complete reserved RAM, WC for aligned non-RAM carveouts; reject everything else.
    pub(crate) fn memory_mode(start: u64, size: u64) -> Result<u32> {
        // SAFETY: Only queries resources and page metadata; it does not create a mapping.
        match unsafe { asahi_g15_memory_mode(start, size) } {
            0 => Err(EINVAL),
            mode => Ok(mode),
        }
    }

    /// Whether the device's DMA mappings skip cache maintenance.
    pub(crate) fn dma_coherent(dev: &device::Device) -> bool {
        // SAFETY: Reads a flag of a valid device.
        unsafe { asahi_g15_dma_coherent(dev.as_raw().cast()) }
    }

    /// `ioremap_wc`, for the RTKit crash buffer.
    pub(crate) fn ioremap_wc(start: u64, size: usize) -> *mut core::ffi::c_void {
        // SAFETY: Creates a new mapping of a device range; no other effect.
        unsafe { asahi_g15_ioremap_wc(start, size) }
    }
}

/// The PMP AGX vote of a start, released on drop until the session is retained ([`Vote::keep`]).
struct Vote(Option<ARef<device::Device>>);

impl Vote {
    /// The session is retained: the vote stays with it until reboot.
    fn keep(mut self) {
        self.0 = None;
    }
}

impl Drop for Vote {
    fn drop(&mut self) {
        // The vote is a level, not a count: never give it back under a retained session.
        if let Some(d) = self.0.take().filter(|_| !glue::retained()) {
            // SAFETY: `d` is the device `asahi_g15_pmp_get` voted for.
            unsafe { glue::asahi_g15_pmp_put(d.as_raw().cast()) };
        }
    }
}

/// `EPROTO` (not among the Rust kernel's named error codes).
pub(crate) fn eproto() -> Error {
    Error::from_errno(-(bindings::EPROTO as i32))
}

/// `Shared` from an RTKit cookie.
///
/// # Safety
///
/// `cookie` must be the pointer [`Rtkit::new`] registered.
unsafe fn shared<'a>(cookie: *mut c_void) -> ArcBorrow<'a, Shared> {
    // SAFETY: The cookie is a leaked `Arc<Shared>` (see `Rtkit::new`), live until reboot.
    unsafe { ArcBorrow::from_raw(cookie as *const Shared) }
}

/// RTKit crash callback: latches fail-stop.
unsafe extern "C" fn rtkit_crashed(cookie: *mut c_void, _log: *const c_void, _size: usize) {
    // SAFETY: RTKit passes back the registered cookie.
    let shared = unsafe { shared(cookie) };
    dev_err!(
        shared.dev,
        "GPU firmware crashed; retaining session until reboot\n"
    );
    shared.fail_stop(EIO);
}

/// RTKit message callback.
unsafe extern "C" fn rtkit_recv(cookie: *mut c_void, endpoint: u8, message: u64) {
    // SAFETY: RTKit passes back the registered cookie.
    let shared = unsafe { shared(cookie) };
    // EP 0x20 0x0042... asks for receive-ring draining; nothing else expected.
    if endpoint == hw::ep::FIRMWARE && message == hw::ep::RING_SERVICE {
        if shared.started.load(Ordering::Acquire) {
            Arc::<Shared>::from(shared).kick();
        }
        return;
    }
    dev_info!(
        shared.dev,
        "firmware EP{:#x} message {:#x}\n",
        endpoint,
        message
    );
}

/// EP 0x20 ring-service request, in the mailbox interrupt: wake only. The drain and the
/// backend's completion check run in the poll work; the shared state outlives the mailbox.
unsafe extern "C" fn rtkit_recv_early(cookie: *mut c_void, endpoint: u8, message: u64) -> bool {
    // SAFETY: RTKit passes back the registered cookie.
    let shared = unsafe { shared(cookie) };
    if !hw::tuning::EARLY_WAKE
        || endpoint != hw::ep::FIRMWARE
        || message != hw::ep::RING_SERVICE
        || !shared.started.load(Ordering::Acquire)
    {
        return false;
    }
    Arc::<Shared>::from(shared).kick();
    true
}

/// Only the inherited crash buffer inside this session's firmware DATA, once.
unsafe extern "C" fn rtkit_shmem_setup(
    cookie: *mut c_void,
    bfr: *mut bindings::apple_rtkit_shmem,
) -> core::ffi::c_int {
    // SAFETY: RTKit passes back the registered cookie.
    let shared = unsafe { shared(cookie) };
    // SAFETY: RTKit passes a valid buffer descriptor it owns for the duration of the call.
    let bfr = unsafe { &mut *bfr };
    let (iova, size) = (bfr.iova as u64, bfr.size as u64);
    let data = &shared.data;
    let data_size = data.end - data.start;
    if shared.rtkit_buffer.load(Ordering::Acquire) != 0
        || iova == 0
        || size == 0
        || (iova | size) & 0xfff != 0
        || iova < data.start
        || size > data_size
        || iova - data.start > data_size - size
    {
        dev_err!(
            shared.dev,
            "refusing RTKit buffer {:#x}/{:#x}\n",
            iova,
            size
        );
        shared.fail_stop(EINVAL);
        return EINVAL.to_errno();
    }
    let iomem = glue::ioremap_wc(iova, bfr.size);
    if iomem.is_null() {
        return ENOMEM.to_errno();
    }
    bfr.iomem = iomem;
    bfr.is_mapped = true;
    shared.rtkit_buffer.store(iova, Ordering::Release);
    dev_info!(
        shared.dev,
        "mapped firmware crash storage {:#x}/{:#x}\n",
        iova,
        size
    );
    0
}

/// Retained until reboot: no qualified stop path.
unsafe extern "C" fn rtkit_shmem_destroy(
    _cookie: *mut c_void,
    _bfr: *mut bindings::apple_rtkit_shmem,
) {
}

/// The GPU's RTKit callbacks.
static RTKIT_OPS: bindings::apple_rtkit_ops = bindings::apple_rtkit_ops {
    flags: 0,
    crashed: Some(rtkit_crashed),
    recv_message: Some(rtkit_recv),
    recv_message_early: Some(rtkit_recv_early),
    shmem_setup: Some(rtkit_shmem_setup),
    shmem_destroy: Some(rtkit_shmem_destroy),
};

/// The GPU's RTKit client. Never freed: the firmware session lasts until reboot.
///
/// # Invariants
///
/// `rtk` is a live `struct apple_rtkit` whose cookie is a leaked `Arc<Shared>`.
pub(crate) struct Rtkit {
    rtk: *mut bindings::apple_rtkit,
}

// SAFETY: The C RTKit API is callable from any thread; mutating calls take `&mut self`.
unsafe impl Send for Rtkit {}
// SAFETY: Shared calls (`has_endpoint`) only read RTKit state.
unsafe impl Sync for Rtkit {}

impl Rtkit {
    /// Initializes RTKit on the maintained GPU node's first, unnamed mailbox.
    /// The session is not device-managed because it is retained until reboot.
    fn new(dev: &device::Device, shared: &Arc<Shared>) -> Result<Self> {
        let cookie = Arc::into_raw(shared.clone()) as *mut c_void;
        // SAFETY: `dev` is valid, the name is a NUL-terminated string or null and the ops are
        // static.
        let rtk = from_err_ptr(unsafe {
            bindings::apple_rtkit_init(dev.as_raw(), cookie, core::ptr::null(), 0, &RTKIT_OPS)
        });
        match rtk {
            // INVARIANT: The cookie stays leaked from here on.
            Ok(rtk) => Ok(Self { rtk }),
            Err(e) => {
                // SAFETY: Init failed, so RTKit holds no copy of the cookie from `into_raw`.
                drop(unsafe { Arc::from_raw(cookie as *const Shared) });
                Err(e)
            }
        }
    }

    fn wake(&mut self) -> Result {
        // SAFETY: `rtk` is live by the type invariant.
        to_result(unsafe { bindings::apple_rtkit_wake(self.rtk) })
    }

    fn has_endpoint(&self, endpoint: u8) -> bool {
        // SAFETY: `rtk` is live by the type invariant.
        unsafe { bindings::apple_rtkit_has_endpoint(self.rtk, endpoint) }
    }

    fn start_endpoint(&mut self, endpoint: u8) -> Result {
        // SAFETY: `rtk` is live by the type invariant.
        to_result(unsafe { bindings::apple_rtkit_start_ep(self.rtk, endpoint) })
    }

    /// Sends a message without waiting.
    pub(crate) fn send(&mut self, endpoint: u8, message: u64) -> Result {
        // SAFETY: `rtk` is live by the type invariant.
        to_result(unsafe {
            bindings::apple_rtkit_send_message(
                self.rtk,
                endpoint,
                message,
                core::ptr::null_mut(),
                false,
            )
        })
    }
}

/// The result of admission: everything probe-time checks established, nothing written.
pub(crate) struct Admitted {
    pub(crate) regs: regs::Regs,
    pub(crate) adt: GpuAdt,
    pub(crate) board_images: initdata::BoardImages,
}

/// The running firmware session: RTKit, MMIO and the adopted UAT. (The
/// host window lives in [`Objects`], which maps through it.)
pub(crate) struct Session {
    pub(crate) dev: ARef<device::Device>,
    pub(crate) shared: Arc<Shared>,
    pub(crate) rtkit: Rtkit,
    pub(crate) regs: regs::Regs,
    pub(crate) adt: GpuAdt,
    uat: Arc<mmu::Uat>,
}

impl Session {
    /// The adopted UAT (user VMs are created and bound through it).
    pub(crate) fn uat(&self) -> Arc<mmu::Uat> {
        self.uat.clone()
    }

    /// The latched error, if any.
    pub(crate) fn check(&self) -> Result {
        self.shared.check()
    }

    /// Sends a mailbox message.
    pub(crate) fn send(&mut self, endpoint: u8, message: u64) -> Result {
        self.rtkit.send(endpoint, message)
    }

    /// Rings a submit channel's doorbell: EP 0x21 `0x0083 << 48 | channel`.
    pub(crate) fn ring(&mut self, channel: Channel) -> Result {
        self.send(hw::ep::DOORBELL, hw::engine::DOORBELL | channel as u64)
    }
}

/// The firmware at READY.
pub(crate) struct Booted {
    pub(crate) session: Session,
    pub(crate) init: InitData,
    pub(crate) objects: Objects,
    pub(crate) channels: Channels,
    pub(crate) events: Events,
    pub(crate) id: regs::Identity,
    pub(crate) max_frequency_khz: u32,
}

/// A firmware session that did not complete: retained until reboot, its receive rings still
/// drained so the firmware never waits on the host.
pub(crate) struct Orphan {
    pub(crate) session: Session,
    /// Retained only: the firmware may still use them.
    pub(crate) _objects: Objects,
    pub(crate) channels: Option<Channels>,
    pub(crate) _init: Option<InitData>,
    pub(crate) events: Events,
}

impl Orphan {
    /// Drains the receive rings, if they exist.
    pub(crate) fn drain(&mut self) -> Result {
        if let Some(channels) = &mut self.channels {
            channels.drain_rings(&self.session.dev, &mut self.events)?;
        }
        Ok(())
    }
}

/// Probe-time admission.
pub(crate) fn admit(
    pdev: &platform::Device<Core<'_>>,
    _dev: &AsahiDevice,
    cfg: &'static Config,
) -> Result<Admitted> {
    let dev = pdev.as_ref();
    let pmp = cfg.platform == hw::Platform::AdtWithPmp;
    if pmp {
        if !glue::board() {
            return Err(ENODEV);
        }
        // Retain the PMP supplier (and its AGX bridge) before any vote.
        // SAFETY: `dev` is the bound GPU device.
        let ret = unsafe { glue::asahi_g15_pmp_prepare(dev.as_raw().cast()) };
        if ret != 0 {
            let err = Error::from_errno(ret);
            if err != EPROBE_DEFER {
                dev_err!(dev, "PMP supplier not ready: {:?}\n", err);
            }
            return Err(err);
        }
    } else if !glue::machine(cfg.machine) {
        dev_err!(dev, "G15: not a {} machine\n", cfg.name);
        return Err(ENODEV);
    }
    let adt = adt::read(dev, cfg)
        .inspect_err(|e| dev_err!(dev, "GPU regions not admitted: {:?}\n", e))?;
    identity::firmware(dev, &adt, cfg.firmware)
        .inspect_err(|e| dev_err!(dev, "loaded GFX TEXT identity mismatch: {:?}\n", e))?;
    dev_info!(dev, "M3: firmware identity accepted\n");
    if cfg.board == hw::Board::None {
        dev_err!(
            dev,
            "G15: no board data for the {}; the firmware is left as the boot loader left it\n",
            cfg.name
        );
        return Err(ENODEV);
    }
    // Firmware objects are cleaned to the point of coherency by their DMA mapping: that needs
    // a device that is not DMA-coherent, as the synthesized node is.
    if glue::dma_coherent(dev) {
        dev_err!(
            dev,
            "G15: the GPU device is DMA-coherent; firmware objects would not be cleaned\n"
        );
        return Err(EINVAL);
    }
    let board_images = initdata::prepare_board(dev, cfg, &adt)?;
    let regs = regs::Regs::new(pdev)?;
    // Keep the GFX domain powered for the whole (retained) session.
    glue::hold_power(dev);
    dev_info!(dev, "{} admitted, ASC={:#x}\n", cfg.name, regs.asc_control()?);
    Ok(Admitted { regs, adt, board_images })
}

/// Posts one device control with only the opcode and waits for it.
fn devctl_op(
    session: &mut Session,
    channels: &mut Channels,
    events: &mut Events,
    init: &InitData,
    opcode: u32,
) -> Result {
    channel::devctl_submit(session, channels, events, init, &DevCtl::op(opcode), true)
}

/// From the wake to READY. On failure the caller keeps everything as an [`Orphan`].
fn run(
    session: &mut Session,
    cfg: &Config,
    objects: &mut Objects,
    events: &mut Events,
    built: &mut Option<(InitData, Channels, u32)>,
    board_images: initdata::BoardImages,
) -> Result {
    let dev = session.dev.clone();
    session.rtkit.wake()?;
    if !session.rtkit.has_endpoint(hw::ep::FIRMWARE)
        || !session.rtkit.has_endpoint(hw::ep::DOORBELL)
    {
        return Err(ENODEV);
    }
    session.rtkit.start_endpoint(hw::ep::FIRMWARE)?;
    session.rtkit.start_endpoint(hw::ep::DOORBELL)?;
    let uat = session.uat.clone();
    objects.window.bootstrap(&dev, &uat)?;
    let (init, channels, _) = built.insert(initdata::build(&dev, cfg, &session.adt, objects, board_images)?);
    uat::tlbi_all();
    dev_info!(
        dev,
        "M3: publishing owned initdata root {:#x} ({} pages, {} L3)\n",
        hw::va::ROOT,
        objects.pages(),
        objects.window.l3_count()
    );
    session.send(
        hw::ep::FIRMWARE,
        hw::ep::INITDATA | (hw::va::ROOT & hw::ep::INITDATA_ADDR_MASK),
    )?;
    devctl_op(session, channels, events, init, devctl::IMPORT_GLOBALS)?;
    devctl_op(session, channels, events, init, devctl::START)?;
    let start = Instant::<Monotonic>::now();
    loop {
        let (magic, ready) = init.readiness()?;
        if magic == runtime::READY_MAGIC && ready == 1 {
            break;
        }
        channels.drain_rings(&dev, events)?;
        session.check()?;
        if start.elapsed() > Delta::from_millis(2000) {
            let (magic, ready) = init.readiness()?;
            dev_err!(
                dev,
                "readiness: runtime+0x314={:#x} flags+4={:#x}\n",
                magic,
                ready
            );
            return Err(ETIMEDOUT);
        }
        fsleep(Delta::from_micros(1000));
    }
    let (magic, ready) = init.readiness()?;
    dev_info!(
        dev,
        "M3 firmware readiness: magic={:#x} flags={}\n",
        magic,
        ready
    );
    devctl_op(session, channels, events, init, devctl::IMPORT_GLOBALS)?;
    channels.drain_rings(&dev, events)?;
    dev_info!(
        dev,
        "M3: firmware accepted owned initdata and device controls\n"
    );
    Ok(())
}

/// Brings the firmware to READY.
pub(crate) fn start(
    admitted: Admitted,
    dev: &AsahiDevice,
    cfg: &'static Config,
    shared: &Arc<Shared>,
) -> Result<Booted> {
    let Admitted { regs, adt, board_images } = admitted;
    let d: ARef<device::Device> = shared.dev.clone();
    // Released by an early failure below, until the session is retained.
    let mut vote = Vote(None);
    if cfg.platform == hw::Platform::AdtWithPmp {
        // SAFETY: `d` is the bound GPU device.
        let ret = unsafe { glue::asahi_g15_pmp_get(d.as_raw().cast()) };
        if ret != 0 {
            dev_err!(d, "PMP AGX vote failed: {}\n", ret);
            return Err(Error::from_errno(ret));
        }
        vote = Vote(Some(d.clone()));
    }
    let id = regs.identity(cfg)?;
    dev_info!(
        d,
        "{}: power acknowledged, version={:#x} counts={:#x} core-mask={:#x}\n",
        cfg.name,
        id.version,
        id.counts,
        id.cores
    );
    if !cfg
        .hw_ids
        .iter()
        .any(|h| h.version == id.version && h.counts == id.counts)
    {
        dev_err!(
            d,
            "unsupported {} hardware: version {:#x} counts {:#x}\n",
            cfg.name,
            id.version,
            id.counts
        );
        return Err(eproto());
    }
    // Binning: any non-empty enabled set in each cluster, nothing above the core slots.
    let clusters = cfg.num_clusters as usize;
    let slots = cfg.core_slots();
    if clusters > regs::MAX_CLUSTERS
        || id.core_masks[..clusters].iter().any(|m| *m == 0)
        || (slots < 64 && id.cores >> slots != 0)
    {
        dev_err!(d, "implausible {} core mask {:#x}\n", cfg.name, id.cores);
        return Err(eproto());
    }
    let mut enabled = [0u32; regs::MAX_CLUSTERS];
    for (count, mask) in enabled.iter_mut().zip(id.core_masks.iter()) {
        *count = mask.count_ones();
    }
    dev_info!(
        d,
        "{}: cores enabled per cluster: {:?}\n",
        cfg.name,
        &enabled[..clusters]
    );
    regs.open()?;
    // Cold only: ASC RUN clear and an all-zero TTBAT before ownership.
    let control = regs.asc_control()?;
    if control & regs::ASC_RUN != 0 {
        return Err(EBUSY);
    }
    let window = uat::Window::new(&adt)?;
    if !window.ttbat_is_clear()? {
        return Err(EBUSY);
    }
    // Adopt the UAT and prove the leaf words before anything firmware-visible is written: no
    // context names the shadow window or the lower root yet.
    let uat = uat::adopt(dev, cfg, &adt)?;
    let mut objects = Objects::new(dev, uat.clone(), window);
    if let Err(e) = objects.self_check() {
        // Mapped with cached firmware classes: never unmapped.
        core::mem::forget(objects);
        return Err(e);
    }

    // Pin the module before anything firmware-visible. Once its removal has begun this fails
    // and the firmware is left untouched, cleaned up as for a failed self-check: the vote is
    // released and the session stays unretained.
    if let Err(e) = glue::retain_module() {
        dev_err!(d, "G15: the module is being removed; the firmware is not started\n");
        // Mapped with cached firmware classes: never unmapped.
        core::mem::forget(objects);
        return Err(e);
    }
    // From here on the firmware is touched: nothing it can reach is freed, the vote included.
    vote.keep();
    let rtkit = match Rtkit::new(&d, shared) {
        Ok(rtkit) => rtkit,
        Err(e) => {
            dev_err!(d, "G15: RTKit mailbox initialization failed: {:?}\n", e);
            core::mem::forget(objects);
            return Err(e);
        }
    };
    shared.started.store(true, Ordering::Release);
    let mut session = Session {
        dev: d.clone(),
        shared: shared.clone(),
        rtkit,
        regs,
        adt,
        uat,
    };
    let mut events = Events::new();
    let mut built = None;
    let result = events
        .reserve()
        .and_then(|_| session.regs.set_asc_control(control | regs::ASC_RUN))
        .and_then(|_| run(&mut session, cfg, &mut objects, &mut events, &mut built, board_images));
    if let Err(e) = result {
        dev_err!(
            d,
            "G15 start failed: {:?}; session retained until reboot\n",
            e
        );
        let (init, channels) = match built {
            Some((init, channels, _)) => (Some(init), Some(channels)),
            None => (None, None),
        };
        shared.orphan(Orphan {
            session,
            _objects: objects,
            channels,
            _init: init,
            events,
        });
        return Err(e);
    }
    let (init, channels, max_frequency_khz) = built.ok_or(EINVAL)?;
    Ok(Booted {
        session,
        init,
        objects,
        channels,
        events,
        id,
        max_frequency_khz,
    })
}
