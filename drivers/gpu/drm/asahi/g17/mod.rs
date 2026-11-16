// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! G17 GPU support.
//!
//! G17 GPUs run their firmware on two coprocessor instances, the primary and the secondary
//! [`Role`]. Each has its own CPU control registers, RTKit mailbox, crash log buffer and kernel
//! page table root, and both share the UAT, the TTBAT and the firmware data structures that the
//! driver allocates at boot (see [`initdata`]).
//!
//! The firmware notifies the host of new records in its event and trace rings with a message on
//! either coprocessor; the event worker drains them.
//!
//! # Locking
//!
//! `Shared::state` is the device lock. It serializes binding, recovery, the event worker and all
//! firmware-shared state that has no narrower owner. Steady-state compute publication into a
//! bound queue takes only that queue's cell lock and the port's locks ([`port`]), reached under
//! a preparation lease. Completions are signalled after dropping the device lock. Lock order:
//! `Shared::state` -> compute queue cell (one at a time) -> port locks -> availability domains;
//! `Shared::state` -> UAT shared state -> VM exec lock -> GEM reservation. `RoleState::ack`
//! nests inside nothing. Capacity waits block on an availability domain signalled by the
//! release that provides the capacity, never on a sleep.

mod availability;
mod buffer;
mod channel;
mod command;
pub(crate) use command::Attachments;
mod completion;
pub(crate) mod context;
mod data_segment;
mod dependency;
#[cfg(CONFIG_DRM_ASAHI_G15)]
pub(crate) use dependency::inherited_error;
mod sync_input;
pub(crate) mod sync_output;
mod trace;
mod event;
mod feed;
mod fence;
mod freelist;
pub(crate) mod fw;
mod ids;
pub(crate) mod hazard;
mod initdata;
mod job;
mod capacity;
mod kick;
mod object;
mod port;
pub(crate) mod power;
mod preparation;
mod qos;
mod queue;
mod recovery;
mod replay;
mod runtime;
pub(crate) mod status;
mod teardown;
mod timeout;
mod validation;

use core::any::Any;
use core::ops::Range;
use core::sync::atomic::{
    AtomicBool,
    AtomicU32,
    AtomicU64,
    Ordering, //
};

use kernel::{
    c_str,
    device::{self, Core},
    drm::gem::shmem,
    io::{
        IoBase,
        IoSysMap,
        Region, //
    },
    impl_has_delayed_work, impl_has_work,
    new_condvar, new_delayed_work, new_mutex, new_work, platform,
    prelude::*,
    soc::apple::rtkit,
    sync::{
        aref::ARef,
        Arc,
        CondVar,
        Mutex, //
    },
    time::{
        delay::fsleep,
        msecs_to_jiffies,
        Delta, //
    },
    types::{ForeignOwnable, Opaque},
    uapi,
    workqueue::{
        self,
        DelayedWork,
        Work,
        WorkItem, //
    },
};

use crate::driver::{
    AsahiDevRef,
    AsahiDrmDevice, //
};
use crate::gem::AsahiObject;
use crate::{
    alloc,
    gem,
    gpu,
    hw,
    mmu,
    regs, //
};

use object::{
    CpuMap,
    KernelObject,
    Placement, //
};

/// Expected identification of a G17 GPU.
/// One monotonic domain for recovery grace, idle effort, pool retention and
/// control-ring retirement.
pub(crate) fn now_ns() -> u64 {
    <kernel::time::Monotonic as kernel::time::ClockSource>::ktime_get() as u64
}

pub(crate) struct Identity {
    /// GPU family field of the ID register.
    pub(crate) family: u8,
    /// GPU variant field of the ID register.
    pub(crate) variant: u8,
    /// Number of GPU dies.
    pub(crate) num_dies: u32,
    /// GPU variant reported to userspace.
    pub(crate) gpu_variant: hw::GpuVariant,
    /// Shader core generation reported to userspace.
    pub(crate) usc_generation: u32,
    /// Hardware abstraction generation reported to userspace.
    pub(crate) hal_generation: u32,
}

/// A buffer that the firmware expects at a fixed address of the kernel lower root.
pub(crate) struct FixedBuffer {
    /// GPU address.
    pub(crate) va: u64,
    /// Size in bytes.
    pub(crate) size: usize,
}

/// Static configuration of a SoC with a G17 GPU.
pub(crate) struct Config {
    /// SoC identifier.
    pub(crate) chip_id: u32,
    /// Expected GPU identification.
    pub(crate) identity: Identity,
    /// Input address bits translated by each UAT root.
    pub(crate) uat_ias: u32,
    /// Output (physical) address bits of the UAT.
    pub(crate) uat_oas: u32,
    /// Offset of the secondary coprocessor's kernel root in the `pagetables` region; `None` for
    /// a single-instance firmware.
    pub(crate) secondary_root_offset: Option<u64>,
    /// Where the coprocessors' crash log buffers live.
    pub(crate) crash_buffers: CrashBuffers,
    /// Parameter buffer page metrics shared with the firmware.
    pub(crate) pm_metrics: FixedBuffer,
    /// Register windows the firmware maps through the hardware data.
    pub(crate) windows: &'static [fw::initdata::RegisterWindow],
    /// Register-map slots that are reserved but have no window.
    pub(crate) reserved_slots: &'static [usize],
    /// Per-SoC parts of the performance tables.
    pub(crate) perf: &'static fw::initdata::PerfConfig,
    /// Number of operating points the device tree lists.
    pub(crate) opps: usize,
    /// UAT policy where it differs from the A18 Pro's.
    pub(crate) uat_quirks: mmu::UatQuirks,
    /// Snapshot the firmware root's own top-level entries after the firmware booted, clear the
    /// others and confirm the snapshot after building (A18 Pro). Otherwise the root is used as
    /// the firmware left it, which is the only way the M4 Pro has run.
    pub(crate) adopt_root_entries: bool,
    /// Enable the GPU-to-fabric bridge and dynamic core power gating before the firmware
    /// starts (A18 Pro). The M4 Pro has only run with neither written.
    pub(crate) fabric_bridge: bool,
    /// The SoC's values of region C.
    pub(crate) region_c: &'static fw::initdata::RegionCConfig,
    /// Physical address of the firmware's per-slot recovery keys, read while the firmware is
    /// halted to attribute a fault; `None` where it is unknown (recovery then relies on the
    /// firmware's own blame).
    pub(crate) recovery_slot_keys: Option<u64>,
    /// GPU generation reported to userspace.
    pub(crate) gpu_generation: hw::GpuGen,
    /// The firmware builds this SoC runs.
    pub(crate) firmware: fw::variant::Firmware,
    /// Layouts of the command buffers that follow the GPU's core layout.
    pub(crate) core_layout: CoreLayout,
}

/// Layouts of the firmware command buffers that follow a SoC's GPU core layout.
pub(crate) struct CoreLayout {
    /// The cluster and enabled core counts the layouts were measured on, which the GPU must
    /// report at probe; `None` where the layouts hold for every configuration of the SoC.
    pub(crate) cores: Option<(u32, u32)>,
    /// A compute queue's operand-state buffer.
    pub(crate) operand_state: fw::compute::OperandState,
    /// A render's deflake buffer.
    pub(crate) deflake: fw::render::Deflake,
}

/// Where a G17 target's crash log buffers live.
pub(crate) enum CrashBuffers {
    /// One buffer per [`Role`] that the firmware requests at a fixed address of the kernel
    /// lower root, mapped by the driver before the coprocessors start.
    Fixed([FixedBuffer; 2]),
    /// The firmware places its crash log in its own data segment, the `fw-data` reserved
    /// region, and asks for it there by physical address.
    FirmwareData,
}

/// A firmware coprocessor instance.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(super) enum Role {
    /// The coprocessor that owns the device-control ring.
    Primary = 0,
    /// The coprocessor that only runs work the primary one hands to it.
    Secondary = 1,
}

/// RTKit endpoint of firmware messages: initialization, its acknowledgement and event
/// notifications.
const EP_FIRMWARE: u8 = 0x20;
/// RTKit endpoint of host notifications to the firmware.
const EP_DOORBELL: u8 = 0x21;

/// Bit position of the type field of firmware messages.
const MSG_TYPE_SHIFT: u32 = 48;
/// Host message handing a coprocessor the address of its initialization root.
const MSG_INITDATA: u64 = 0x81 << MSG_TYPE_SHIFT;
/// Address bits of an initialization root that [`MSG_INITDATA`] carries.
const MSG_INITDATA_ADDR_MASK: u64 = (1 << 44) - 1;
/// Firmware message acknowledging [`MSG_INITDATA`].
const MSG_INITDATA_ACK: u64 = 0x09;
/// Firmware message announcing new records in the event and trace rings.
const MSG_EVENTS: u64 = 0x42 << MSG_TYPE_SHIFT;
/// Primary processor notification for newly published device-control records.
const MSG_CONTROL_NOTIFY: u64 = (0x84 << MSG_TYPE_SHIFT) | 0x11;
/// Host message, sent to both coprocessors once both are initialized, ending the power-on
/// transition. The firmware then retires the opening device-control records.
const MSG_POWER_ON_DONE: u64 = 0x89 << MSG_TYPE_SHIFT;

/// Time the firmware takes at most to acknowledge its initialization.
const INITDATA_ACK_TIMEOUT_MS: u32 = 2000;
/// Bounded mailbox-provider readiness retries, before either coprocessor starts.
const RECEIVER_ATTEMPTS: usize = 3;
const RECEIVER_RETRY_MS: i64 = 20;
/// The firmware does not notify the host when it retires device-control records, so their
/// counters are sampled initially and after at most [`OPENING_RETIRE_POLLS`] waits.
const OPENING_RETIRE_POLL_MS: i64 = 1;
const OPENING_RETIRE_POLLS: usize = 100;
/// One-millisecond polls for a firmware that acknowledges its initialization only in status A
/// ([`fw::variant::InitAck::Status`]).
const INIT_ACCEPT_POLLS: usize = 3000;

/// Address bits of a buffer the coprocessor places itself; the coprocessor sets higher bits in
/// its buffer requests.
const RTKIT_BUFFER_ADDR_MASK: u64 = (1 << 40) - 1;
/// Initial contents of crash log buffers, so that a crash log shows what the firmware wrote.
const CRASH_BUFFER_FILL: [u8; 2] = [0xa5, 0x5a];

/// A crash log buffer handed to RTKit.
pub(crate) struct CrashLog {
    iova: usize,
    map: CrashLogMap,
}

/// The CPU mapping of a crash log buffer.
enum CrashLogMap {
    /// The driver's buffer ([`CrashBuffers::Fixed`]).
    Object(shmem::VMapOwned<AsahiObject>),
    /// A buffer in the firmware's data segment ([`CrashBuffers::FirmwareData`]).
    FirmwareData(data_segment::DataSegmentLog),
}

impl rtkit::Buffer for CrashLog {
    fn iova(&self) -> Result<usize> {
        Ok(self.iova)
    }

    fn buf(&mut self) -> Result<IoSysMap<'_, Region>> {
        match &mut self.map {
            CrashLogMap::Object(map) => Ok(IoSysMap::from((&*map).as_view())),
            CrashLogMap::FirmwareData(log) => log.buf(),
        }
    }
}

/// Callback state; initialization ACKs can arrive in hard interrupt context.
#[pin_data]
struct RoleState {
    role: Role,
    shared: Arc<Shared>,
    /// The fixed crash log buffer and its mapping; `None` where the firmware places its own.
    crash_log: Option<(FixedBuffer, shmem::VMapOwned<AsahiObject>)>,
    crashed: AtomicBool,
    #[pin]
    ack: Opaque<kernel::bindings::completion>,
}

// SAFETY: Completion access uses the kernel completion API; the remaining fields are immutable
// shared references or atomics. The completion is pinned before callbacks can observe it.
unsafe impl Send for RoleState {}
// SAFETY: As above; complete_all is safe concurrently with the single boot waiter.
unsafe impl Sync for RoleState {}

impl RoleState {
    fn signal(&self, crashed: bool) {
        if crashed {
            self.crashed.store(true, Ordering::Release);
        }
        // SAFETY: ack is initialized and pinned for the lifetime of the callback state.
        unsafe { kernel::bindings::complete_all(self.ack.get()) };
    }

    fn wait_for_ack(&self) -> Result {
        // SAFETY: ack is initialized and remains pinned while this reference exists.
        let completed = unsafe {
            kernel::bindings::wait_for_completion_timeout(
                self.ack.get(),
                msecs_to_jiffies(INITDATA_ACK_TIMEOUT_MS),
            )
        };
        if self.crashed.load(Ordering::Acquire) {
            return Err(EIO);
        }
        if completed == 0 {
            return Err(ETIMEDOUT);
        }
        Ok(())
    }
}

fn is_events_message(endpoint: u8, message: u64) -> bool {
    endpoint == EP_FIRMWARE && message == MSG_EVENTS
}

/// RTKit callbacks of a coprocessor.
struct RoleOps;

#[vtable]
impl rtkit::Operations for RoleOps {
    type Data = Arc<RoleState>;
    type Buffer = CrashLog;

    const COPROC_PLACES_BUFFERS: bool = true;

    fn recv_message_early(
        data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        ep: u8,
        msg: u64,
    ) -> bool {
        if ep == EP_FIRMWARE && msg >> MSG_TYPE_SHIFT == MSG_INITDATA_ACK {
            data.signal(false);
            return true;
        }
        if !is_events_message(ep, msg) {
            return false;
        }
        data.shared.queue_event_records();
        true
    }

    fn crashed(data: <Self::Data as ForeignOwnable>::Borrowed<'_>, crashlog: Option<&[u8]>) {
        data.signal(true);
        data.shared.lose_device(data.role, crashlog);
    }

    fn shmem_map(
        data: <Self::Data as ForeignOwnable>::Borrowed<'_>,
        iova: usize,
        size: usize,
    ) -> Result<CrashLog> {
        if let Some((buffer, map)) = data.crash_log.as_ref() {
            if (iova as u64) & RTKIT_BUFFER_ADDR_MASK != buffer.va || size > buffer.size {
                return Err(EINVAL);
            }
            return Ok(CrashLog {
                iova,
                map: CrashLogMap::Object(map.owner().owned_vmap()?),
            });
        }
        // The firmware places its buffers in its data segment, at their physical address.
        Ok(CrashLog {
            iova,
            map: CrashLogMap::FirmwareData(data_segment::DataSegmentLog::map(
                &data.shared.dev,
                iova,
                size,
            )?),
        })
    }
}

/// The RTKit of one coprocessor. The mailbox driver serializes its own register writes; this
/// lock only serializes the exclusive borrows the RTKit API takes, so that publishers holding
/// different locks can ring the same doorbell.
#[pin_data]
struct Mailbox {
    #[pin]
    lock: Mutex<()>,
    rtkit: core::cell::UnsafeCell<rtkit::RtKit<RoleOps>>,
}

// SAFETY: Every access to the RTKit goes through `with`, which holds the lock.
unsafe impl Sync for Mailbox {}

impl Mailbox {
    fn new(rtkit: rtkit::RtKit<RoleOps>) -> Result<Pin<KBox<Self>>> {
        KBox::pin_init(
            pin_init!(Self {
                lock <- new_mutex!((), "g17::Mailbox::lock"),
                rtkit: core::cell::UnsafeCell::new(rtkit),
            }),
            GFP_KERNEL,
        )
    }

    fn with<R>(&self, f: impl FnOnce(Pin<&mut rtkit::RtKit<RoleOps>>) -> R) -> R {
        let _guard = self.lock.lock();
        // SAFETY: The lock excludes every other borrow of the RTKit.
        f(Pin::new(unsafe { &mut *self.rtkit.get() }))
    }

    fn send(&self, endpoint: u8, message: u64) -> Result {
        self.with(|rtkit| rtkit.send_message(endpoint, message))
    }
}

/// A running firmware coprocessor.
///
/// Dropping it stops the CPU before RTKit and the crash log buffer go away.
struct Coprocessor {
    cpu: regs::CpuControl,
    running: bool,
    rtkit: Pin<KBox<Mailbox>>,
    state: Arc<RoleState>,
    _crash_buffer: Option<KernelObject>,
}

impl Coprocessor {
    /// Sets up a coprocessor's RTKit and crash log buffer, without starting it.
    fn new(
        pdev: &platform::Device<Core<'_>>,
        dev: &AsahiDrmDevice,
        cfg: &Config,
        uat: &mmu::Uat,
        shared: &Arc<Shared>,
        role: Role,
    ) -> Result<Coprocessor> {
        let index = role as usize;
        let (crash_buffer, crash_log) = match &cfg.crash_buffers {
            CrashBuffers::Fixed(buffers) => {
                let buffer = &buffers[index];
                let mut crash_buffer = KernelObject::new(
                    dev,
                    uat.kernel_lower_vm(),
                    Placement::At(buffer.va),
                    buffer.size,
                    mmu::PROT_FW_SHARED_RW,
                    CpuMap::WriteCombined,
                )?;
                crash_buffer.bytes_mut().fill(CRASH_BUFFER_FILL[index]);
                let crash_map = crash_buffer.kernel_map()?;
                let fixed = FixedBuffer {
                    va: buffer.va,
                    size: buffer.size,
                };
                (Some(crash_buffer), Some((fixed, crash_map)))
            }
            CrashBuffers::FirmwareData => (None, None),
        };
        let state = Arc::pin_init(
            pin_init!(RoleState {
                role,
                shared: shared.clone(),
                crash_log,
                crashed: AtomicBool::new(false),
                ack <- Opaque::ffi_init(|ptr| {
                    // SAFETY: ptr is valid uninitialized completion storage that is pinned
                    // before any RTKit callback can use it.
                    unsafe { kernel::bindings::init_completion(ptr) };
                }),
            }),
            GFP_KERNEL,
        )?;

        Ok(Coprocessor {
            cpu: regs::CpuControl::new(pdev, index)?,
            running: false,
            rtkit: Mailbox::new(Self::receiver(dev, index, &state)?)?,
            state,
            _crash_buffer: crash_buffer,
        })
    }

    fn receiver(
        dev: &AsahiDrmDevice,
        index: usize,
        state: &Arc<RoleState>,
    ) -> Result<rtkit::RtKit<RoleOps>> {
        for attempt in 0..RECEIVER_ATTEMPTS {
            match rtkit::RtKit::<RoleOps>::new(dev.as_ref().as_ref(), None, index, state.clone()) {
                Ok(receiver) => return Ok(receiver),
                Err(error) if error == EINVAL || error == ETIMEDOUT => {
                    if attempt + 1 < RECEIVER_ATTEMPTS {
                        fsleep(Delta::from_millis(RECEIVER_RETRY_MS));
                    }
                }
                Err(error) => return Err(error),
            }
        }
        Err(EPROBE_DEFER)
    }

    /// Starts the CPU and waits for RTKit to come up.
    fn boot(&mut self) -> Result {
        self.cpu.start()?;
        self.running = true;
        self.rtkit.with(|rtkit| rtkit.boot())
    }

    /// Starts the firmware endpoints and hands the firmware its initialization root. An
    /// acknowledgment message is awaited here; one in status A is the caller's to poll.
    fn init(&mut self, root_va: u64, ack: fw::variant::InitAck) -> Result {
        self.rtkit.with(|mut rtkit| {
            rtkit.as_mut().start_endpoint(EP_FIRMWARE)?;
            rtkit.as_mut().start_endpoint(EP_DOORBELL)?;
            rtkit.send_message(
                EP_FIRMWARE,
                MSG_INITDATA | (root_va & MSG_INITDATA_ADDR_MASK),
            )
        })?;
        match ack {
            fw::variant::InitAck::Message => self.state.wait_for_ack(),
            fw::variant::InitAck::Status => Ok(()),
        }
    }

    /// Sends a host notification to the firmware.
    fn notify(&self, msg: u64) -> Result {
        trace::message(trace::MAILBOX, msg);
        self.rtkit.send(EP_DOORBELL, msg)
    }

    /// Stops the CPU. The firmware keeps its state, but no longer runs.
    fn stop(&mut self) {
        if self.running {
            self.cpu.stop();
            self.running = false;
        }
    }
}

impl Drop for Coprocessor {
    fn drop(&mut self) {
        self.stop();
    }
}

/// The GPU-to-fabric bridge, enabled for as long as this exists.
struct Bridge {
    regs: regs::G17Resources,
    enabled: bool,
}

impl Bridge {
    fn enable(regs: regs::G17Resources) -> Result<Bridge> {
        let bridge = Bridge {
            regs,
            enabled: true,
        };
        bridge.regs.enable_g17_bridge()?;
        Ok(bridge)
    }

    /// Keeps the register block for a target that runs without the bridge.
    fn unused(regs: regs::G17Resources) -> Bridge {
        Bridge {
            regs,
            enabled: false,
        }
    }

    fn disable(&mut self) {
        if self.enabled {
            self.regs.disable_g17_bridge();
            self.enabled = false;
        }
    }
}

impl Drop for Bridge {
    fn drop(&mut self) {
        self.disable();
    }
}

/// Everything that exists while the firmware runs.
///
/// Field order is teardown order: both coprocessors stop (in [`Drop`]) before their RTKit is
/// dropped, the bridge is disabled once they are stopped, and the firmware objects and their
/// mappings go before the UAT. Page tables the firmware or the bootloader own are never freed.
struct Firmware {
    /// The secondary instance, on GPUs whose firmware runs on two (A18 Pro). A single-instance
    /// firmware (M4 Pro) carries both roles on the primary.
    secondary: Option<Coprocessor>,
    primary: Coprocessor,
    bridge: Bridge,
    /// Publication state shared with publishers that do not hold the device mutex.
    port: Arc<port::Port>,
    /// Fast-path submissions already accounted by the idle check.
    seen_submissions: u64,
    rings: channel::Rings,
    recovery: recovery::State,
    effort: power::Effort,
    hold: power::Hold,
    registration_power_release_pending: bool,
    variant: &'static fw::Variant,
    queues: runtime::Registry,
    pool: Arc<object::Pool>,
    /// Render kick ring storage, kept and reused for the life of the device.
    render_rings: Arc<kick::RenderRings>,
    render_ids: Arc<freelist::RenderIds>,
    init: initdata::InitData,
    _render_global: mmu::Vm,
    _pm_metrics: Arc<KernelObject>,
    uat: Arc<mmu::Uat>,
}

impl Drop for Firmware {
    fn drop(&mut self) {
        if let Some(secondary) = self.secondary.as_mut() {
            secondary.stop();
        }
        self.primary.stop();
        self.pool.stop_after_firmware();
        // RTKit destruction releases the mailbox power references. Clear the bridge while
        // those references still keep the register block powered.
        self.bridge.disable();
    }
}

impl Firmware {
    /// The firmware instances this GPU runs.
    fn roles(&self) -> &'static [Role] {
        if self.secondary.is_some() {
            &[Role::Primary, Role::Secondary]
        } else {
            &[Role::Primary]
        }
    }

    /// Publishes controls only while the restart handshake is closed. The device mutex remains
    /// held through publication and the following mailbox notification.
    fn publish_control(&mut self, record: &fw::channels::ControlRecord) -> Result<u32> {
        use recovery::Memory;
        if self.recovery.pending() || self.init.recovery_state()? != 0 {
            return Err(EAGAIN);
        }
        let target = channel::Rings::publish_control(&self.init, record)?;
        core::sync::atomic::fence(Ordering::SeqCst);
        self.primary.notify(MSG_CONTROL_NOTIFY)?;
        Ok(target)
    }

    /// Holds idle descent while a fresh physical render pair is registered.
    fn acquire_registration_power(&mut self) -> Result {
        power::acquire(self)
    }

    fn release_registration_power(&mut self) -> Result {
        // An idle hold keeps the cores powered past registration; it releases them itself.
        if self.hold.active() {
            self.registration_power_release_pending = false;
            return Ok(());
        }
        power::release(self)
    }

    /// Called at actual submission publication, not while a queued job is merely being built.
    fn note_submission(&mut self) -> Result {
        self.effort.submit();
        self.apply_effort(power::FULL_EFFORT)
    }

    fn apply_effort(&mut self, effort: u32) -> Result {
        Self::apply_shared_effort(&self.init, &mut self.effort, &self.port, effort)
    }

    fn apply_shared_effort(
        init: &initdata::InitData,
        state: &mut power::Effort,
        port: &port::Port,
        effort: u32,
    ) -> Result {
        if state.applied() == effort && init.effort()? == effort {
            return Ok(());
        }
        init.publish_effort(effort)?;
        trace::message(trace::EFFORT, u64::from(effort));
        state.did_apply(effort);
        port.set_effort_full(effort == power::FULL_EFFORT);
        Ok(())
    }

    /// Returns the delay in ns for a follow-up idle check. Queue owners report only submitted,
    /// unretired work; quarantined work retaining mappings is excluded. Submissions published
    /// without the device mutex count as activity, and a clear that races one is abandoned.
    fn idle_check(&mut self, now_ns: u64, in_flight: bool) -> Result<Option<u64>> {
        let submissions = self.port.submissions();
        let submitted = submissions != self.seen_submissions;
        self.seen_submissions = submissions;
        if submitted {
            self.effort.submit();
        }
        let effort = match self.effort.idle(now_ns, in_flight || submitted) {
            power::IdleAction::None => None,
            power::IdleAction::Clear => {
                self.port.set_effort_full(false);
                if self.port.submissions() != submissions {
                    self.port.set_effort_full(true);
                    self.effort.submit();
                    return Ok(Some(power::IDLE_HOLD_NS));
                }
                self.apply_effort(0)?;
                None
            }
            power::IdleAction::Recheck(delay) => Some(delay),
        };
        Ok(match (effort, self.idle_hold(now_ns, in_flight || submitted)) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        })
    }

    /// Drives the idle hold (see power::Hold); returns the delay in ns for its next check. A
    /// refused control (recovery in progress) is retried on a later check.
    fn idle_hold(&mut self, now_ns: u64, in_flight: bool) -> Option<u64> {
        const RETRY_NS: u64 = 1_000_000;
        let hold_ms = power::IDLE_HOLD_MS.load(Ordering::Relaxed);
        for _ in 0..2 {
            match self.hold.step(now_ns, in_flight, self.effort.submissions(), hold_ms) {
                power::HoldAction::None => return None,
                power::HoldAction::Recheck(delay) => return Some(delay),
                power::HoldAction::Assert => {
                    if self.set_idle_control(false).is_err() {
                        return None;
                    }
                    self.hold.asserted();
                }
                power::HoldAction::Release => {
                    if self.set_idle_control(true).is_err() {
                        return Some(RETRY_NS);
                    }
                    self.hold.released();
                    return None;
                }
            }
        }
        None
    }

    fn set_idle_control(&mut self, enabled: bool) -> Result<u32> {
        power::Control::set_idle(self, enabled)
    }
}

impl power::Control for Firmware {
    fn crashed(&self) -> bool {
        self.primary.state.shared.crashed.load(Ordering::Acquire)
    }
    fn powered(&self) -> bool {
        self.bridge.regs.g17_configuration_powered()
    }
    fn counters(&self) -> Result<[u32; 3]> {
        self.init.control_counters()
    }
    fn set_idle(&mut self, enabled: bool) -> Result<u32> {
        use fw::channels::{ControlRecord, IdlePolicy, IdlePowerOff};
        self.publish_control(&ControlRecord::IdlePowerOff(IdlePowerOff::new(
            if enabled {
                IdlePolicy::AllowPowerOff
            } else {
                IdlePolicy::KeepPowered
            },
        )))
    }
    fn wait_tick(&self) {
        fsleep(Delta::from_millis(1));
    }
    fn registration_release_failed(&mut self) {
        self.registration_power_release_pending = true;
    }
    fn registration_release_succeeded(&mut self) {
        self.registration_power_release_pending = false;
    }
}

/// Event work item ID.
const EVENT_WORK: u64 = 0;
/// Short control/retirement polls do not wait behind the longer idle deadline.
const POLL_WORK: u64 = 1;
const IDLE_WORK: u64 = 2;
const GROW_WORK: u64 = 3;
const RECLAIM_WORK: u64 = 4;
const FEED_WORK: u64 = 5;

/// Device state shared with the coprocessor callbacks and the event worker.
#[pin_data]
struct Shared {
    /// Diagnostics may outlive shutdown without retaining the DRM data graph.
    dev: ARef<device::Device>,
    /// Allocation users take independent references before reserving resources.
    /// Shutdown removes this owning backedge after joining device work.
    #[pin]
    drm: Mutex<Option<AsahiDevRef>>,
    clusters: u32,
    active_cores: u32,
    variant: &'static fw::Variant,
    /// Layout of compute queues' operand-state buffers for this GPU's cores.
    operand_state: &'static fw::compute::OperandState,
    /// Layout of renders' deflake buffers for this GPU's cores.
    deflake: fw::render::Deflake,
    descriptor_flags: [u32; 2],
    crashed: AtomicBool,
    feed: Arc<feed::Feed>,
    #[pin]
    feed_work: Work<Shared, FEED_WORK>,
    preparations: Arc<preparation::Gate>,
    /// Compute admission waiting for a physical channel: signalled when a channel becomes
    /// reusable or shareable, a QID or slot is returned, or a pool vacancy is witnessed.
    channels: Arc<availability::Availability>,
    /// Compute admission waiting for retirement-bound capacity (work states, ring and
    /// in-flight room, drain witnesses, pool releases) or for publication to reopen.
    progress: Arc<availability::Availability>,
    /// Logical-queue creation waiting for a returned queue slot or execution context.
    queues: Arc<availability::Availability>,
    /// Render publication waiting for render capacity, a pair on loan, a work
    /// state or firmware room: signalled when a pair, lease or work state
    /// returns, an owner closes, a teardown parks, and on every event pass
    /// while a render publisher waits (firmware consumption has no event).
    renders: Arc<availability::Availability>,
    /// Unbound compute publishers which can reuse a retired channel.
    compute_demand: AtomicU32,
    #[pin]
    work: Work<Shared, EVENT_WORK>,
    /// The next event pass runs every housekeeping step: set by every [`Self::queue_events`]
    /// request, not by the firmware's event notification ([`Self::queue_event_records`]).
    housekeeping: AtomicBool,
    /// When an event pass last ran every housekeeping step, in ns.
    housekeeping_ns: AtomicU64,
    #[pin]
    poll: DelayedWork<Shared, POLL_WORK>,
    #[pin]
    idle: DelayedWork<Shared, IDLE_WORK>,
    #[pin]
    grow: Work<Shared, GROW_WORK>,
    #[pin]
    reclaim_work: Work<Shared, RECLAIM_WORK>,
    /// Nests inside the device mutex; detached mappings are dropped after both unlock.
    #[pin]
    reclaim: Mutex<runtime::teardown::ReclaimBatch>,
    /// Serializes detached firmware destruction with unbind. Stop paths only;
    /// normal publication and RTKit/fence handlers never acquire this lock.
    #[pin]
    stop: Mutex<()>,
    /// The device lock. `None` before boot completes and after detachment.
    #[pin]
    state: Mutex<Option<KBox<Firmware>>>,
    #[pin]
    changed: CondVar,
    /// The port of the session in `state`; null before boot and once detachment begins.
    port: core::sync::atomic::AtomicPtr<port::Port>,
}

impl_has_work! {
    impl HasWork<Shared, EVENT_WORK> for Shared { self.work }
}

impl_has_delayed_work! {
    impl HasDelayedWork<Shared, POLL_WORK> for Shared { self.poll }
}
impl_has_delayed_work! {
    impl HasDelayedWork<Shared, IDLE_WORK> for Shared { self.idle }
}
impl_has_work! {
    impl HasWork<Shared, GROW_WORK> for Shared { self.grow }
}
impl_has_work! {
    impl HasWork<Shared, RECLAIM_WORK> for Shared { self.reclaim_work }
}
impl WorkItem<GROW_WORK> for Shared {
    type Pointer = Arc<Shared>;
    fn run(this: Arc<Shared>) {
        if let Err(error) = this.grow_render() {
            if this.state.lock().is_some() {
                dev_err!(
                    this.dev.as_ref(),
                    "Render parameter-buffer growth failed: {:?}\n",
                    error
                );
                this.lose_device_quietly();
            }
        }
        this.queue_events();
    }
}
impl WorkItem<RECLAIM_WORK> for Shared {
    type Pointer = Arc<Shared>;
    fn run(this: Arc<Shared>) {
        this.drain_reclaims();
        this.queue_events();
    }
}
impl WorkItem<POLL_WORK> for Shared {
    type Pointer = Arc<Shared>;
    fn run(this: Arc<Shared>) {
        let result = {
            let mut state = this.state.lock();
            match (*state).as_deref_mut() {
                Some(firmware) if firmware.render_control_backpressured() => {
                    firmware.note_render_control_backpressure()
                }
                _ => Ok(()),
            }
        };
        // Only an elapsed one-millisecond timer consumes the retry budget.
        // Concurrent event IRQs can request a pass without aging the control.
        if result.is_err() {
            this.lose_device_quietly();
        } else {
            this.queue_events();
        }
    }
}
impl WorkItem<IDLE_WORK> for Shared {
    type Pointer = Arc<Shared>;
    fn run(this: Arc<Shared>) {
        this.queue_events();
    }
}

impl WorkItem<EVENT_WORK> for Shared {
    type Pointer = Arc<Shared>;

    fn run(this: Arc<Shared>) {
        this.run_runtime_events();
    }
}

impl Shared {
    fn drm(&self) -> Result<AsahiDevRef> {
        self.drm.lock().as_ref().cloned().ok_or(ENODEV)
    }

    fn release_drm(&self) {
        // The final put may call device-release callbacks. Never run those with
        // the reference-slot mutex held.
        let retired = self.drm.lock().take();
        drop(retired);
    }

    /// Wakes the waiters for a returned pair, binding or capacity change.
    fn note_changed(&self) {
        self.changed.notify_all();
        self.renders.notify();
    }


    /// The session's port, valid while `lease` is held: shutdown and device loss join every
    /// lease before they detach the session.
    fn port<'a>(&self, lease: &'a preparation::Lease) -> Option<&'a port::Port> {
        let _ = lease;
        // SAFETY: The pointer is published before the session becomes reachable and cleared
        // only after every lease has been joined, so it names a live port for 'a.
        unsafe { self.port.load(Ordering::Acquire).as_ref() }
    }

    /// Called after joining every preparation lease, before the session is detached.
    fn forget_port(&self) {
        self.port.store(core::ptr::null_mut(), Ordering::Release);
    }

    /// Queues the event worker for a pass that runs every housekeeping step. Callable from any
    /// context.
    fn queue_events(self: &Arc<Self>) {
        self.housekeeping.store(true, Ordering::Release);
        self.queue_event_records();
    }

    /// Queues the event worker for firmware records alone: completions, or records that arrived
    /// during a pass. The pass skips the housekeeping that nothing needs (see
    /// `runtime_event_turn`). Callable from any context.
    fn queue_event_records(self: &Arc<Self>) {
        // Already queued if this fails, which is just as good.
        let _ = workqueue::system_highpri().enqueue::<Arc<Shared>, EVENT_WORK>(self.clone());
    }

    fn queue_grow(self: &Arc<Self>) {
        let _ = workqueue::system_dfl().enqueue::<Arc<Self>, GROW_WORK>(self.clone());
    }

    fn queue_reclaims(self: &Arc<Self>) {
        let _ = workqueue::system_dfl().enqueue::<Arc<Self>, RECLAIM_WORK>(self.clone());
    }

    fn drain_reclaims(self: &Arc<Self>) {
        let mut reclaimed = false;
        loop {
            let next = self.reclaim.lock().take_one();
            let Some(next) = next else {
                break;
            };
            next.finish();
            reclaimed = true;
        }
        if reclaimed {
            // Context release returns its execution root off-lock. The next
            // device pass can now detach idle compute aliases and their QoS
            // leases, without depending on a new client's submission/event.
            self.queues.notify();
            self.queue_events();
        }
    }

    fn disable_work<const ID: u64>(self: &Arc<Self>, work: &Work<Self, ID>) {
        // SAFETY: Callers pass this Shared's initialized pinned work field.
        // Disabling joins a running callback and prevents subsequent enqueue.
        let cancelled = unsafe { kernel::bindings::disable_work_sync(Work::raw_get(work)) };
        if cancelled {
            // SAFETY: Cancelling the pending callback leaves its one enqueue-
            // transferred Arc to reclaim; the callback can no longer consume it.
            drop(unsafe { Arc::from_raw(Arc::as_ptr(self)) });
        }
    }

    fn queue_poll(self: &Arc<Self>) {
        let _ = workqueue::system_highpri()
            .enqueue_delayed::<Arc<Self>, POLL_WORK>(self.clone(), msecs_to_jiffies(1));
    }

    fn queue_idle(self: &Arc<Self>, delay_ns: u64) {
        // Effort::idle bounds this by the 16 ms idle interval. Round up so a
        // timer never changes the firmware effort word before that deadline.
        let delay_ms = delay_ns.div_ceil(1_000_000).min(u64::from(u32::MAX)) as u32;
        let _ = workqueue::system_highpri()
            .enqueue_delayed::<Arc<Self>, IDLE_WORK>(self.clone(), msecs_to_jiffies(delay_ms));
    }

    fn disable_timer<const ID: u64>(self: &Arc<Self>, timer: &DelayedWork<Self, ID>) {
        // SAFETY: Every caller passes one of this Shared's initialized pinned
        // timer fields. raw_as_work points into its delayed_work; the C call
        // disables future enqueues, cancels pending work and joins execution.
        let cancelled = unsafe {
            let work = Work::raw_get(DelayedWork::raw_as_work(timer));
            let delayed = kernel::container_of!(work, kernel::bindings::delayed_work, work);
            kernel::bindings::disable_delayed_work_sync(delayed)
        };
        if cancelled {
            // SAFETY: The cancelled pending callback owned exactly one Arc
            // transferred by enqueue_delayed. It cannot run to reclaim it.
            drop(unsafe { Arc::from_raw(Arc::as_ptr(self)) });
        }
    }

    /// Marks the device lost without a crash log.
    fn lose_device_quietly(self: &Arc<Self>) {
        self.preparations.fail();
        if !self.crashed.swap(true, Ordering::AcqRel) {
            dev_err!(self.dev.as_ref(), "GPU runtime stopped, device lost\n");
        }
        self.queue_events();
    }

    /// Marks the device lost after a coprocessor crash and publishes a crash dump.
    ///
    /// The firmware cannot be restarted: outstanding and future work fails.
    fn lose_device(self: &Arc<Self>, role: Role, crashlog: Option<&[u8]>) {
        self.preparations.fail();
        self.crashed.store(true, Ordering::Release);
        dev_err!(
            self.dev.as_ref(),
            "GPU firmware ({:?}) crashed, device lost\n",
            role
        );

        // The GPU dev coredump support this used is gone; the crash log is not published.
        let _ = crashlog;
        self.queue_events();
    }
}

/// A G17 GPU.
pub(crate) struct Gpu {
    cfg: &'static Config,
    id: regs::G17Id,
    max_frequency_khz: u32,
    geometry: mmu::UatGeometry,
    ids: gpu::SequenceIDs,
    shared: Arc<Shared>,
    _capacity: Pin<KBox<kernel::debugfs::File<capacity::View>>>,
}

impl Gpu {
    /// Boots the firmware of a G17 GPU.
    pub(crate) fn new(
        pdev: &platform::Device<Core<'_>>,
        dev: &AsahiDrmDevice,
        cfg: &'static Config,
        regs: regs::G17Resources,
    ) -> Result<Arc<Gpu>> {
        let variant = Self::firmware_variant(dev, &cfg.firmware)?;
        let id = regs.get_g17_id()?;
        if id.family != cfg.identity.family
            || id.variant != cfg.identity.variant
            || id.num_dies != cfg.identity.num_dies
        {
            dev_err!(
                dev.as_ref(),
                "Unexpected GPU family {:#x} variant {:#x} with {} dies\n",
                id.family,
                id.variant,
                id.num_dies
            );
            return Err(ENODEV);
        }

        let geometry = mmu::UatGeometry {
            ias: cfg.uat_ias,
            oas: cfg.uat_oas,
        };
        let max_frequency_khz = Self::max_frequency_khz(dev)?;
        let channels = availability::Availability::new()?;
        let progress = availability::Availability::new()?;
        let queues = availability::Availability::new()?;
        let preparations = preparation::Gate::with_waiters(Some(progress.clone()))?;
        let renders = availability::Availability::new()?;
        let reclaim = runtime::teardown::ReclaimBatch::new()?;
        let feed = feed::Feed::new()?;
        let active_cores = id.core_masks.iter().map(|mask| mask.count_ones()).sum();
        if cfg
            .core_layout
            .cores
            .is_some_and(|cores| cores != (id.num_clusters, active_cores))
        {
            dev_err!(
                dev.as_ref(),
                "Unsupported GPU core layout: {} clusters with {} cores\n",
                id.num_clusters,
                active_cores
            );
            return Err(ENODEV);
        }
        let shared = Arc::pin_init(
            pin_init!(Shared {
                drm <- new_mutex!(Some(dev.into()), "g17::Shared::drm"),
                dev: dev.as_ref().as_ref().into(),
                clusters: id.num_clusters,
                active_cores,
                variant,
                operand_state: &cfg.core_layout.operand_state,
                deflake: cfg.core_layout.deflake,
                descriptor_flags: [id.perf_control, id.perf_map[0]],
                crashed: AtomicBool::new(false),
                feed,
                feed_work <- new_work!("g17::Shared::feed_work"),
                preparations,
                channels,
                progress,
                queues,
                renders,
                compute_demand: AtomicU32::new(0),
                work <- new_work!("g17::Shared::work"),
                housekeeping: AtomicBool::new(true),
                housekeeping_ns: AtomicU64::new(0),
                poll <- new_delayed_work!("g17::Shared::poll"),
                idle <- new_delayed_work!("g17::Shared::idle"),
                grow <- new_work!("g17::Shared::grow"),
                reclaim_work <- new_work!("g17::Shared::reclaim_work"),
                reclaim <- new_mutex!(reclaim, "g17::Shared::reclaim"),
                stop <- new_mutex!((), "g17::Shared::stop"),
                state <- new_mutex!(None, "g17::Shared::state"),
                changed <- new_condvar!("g17::Shared::changed"),
                port: core::sync::atomic::AtomicPtr::new(core::ptr::null_mut()),
            }),
            GFP_KERNEL,
        )?;

        let directory = kernel::debugfs::Dir::new(c"asahi-g17");
        let capacity = KBox::pin_init(
            directory.read_only_file(c"capacity", capacity::View(shared.clone())),
            GFP_KERNEL,
        )?;
        let gpu = Arc::new(
            Gpu {
                cfg,
                id,
                max_frequency_khz,
                geometry,
                ids: Default::default(),
                shared,
                _capacity: capacity,
            },
            GFP_KERNEL,
        )?;
        let firmware = Self::boot(pdev, dev, cfg, regs, geometry, &gpu.shared)?;
        gpu.shared
            .port
            .store(Arc::as_ptr(&firmware.port).cast_mut(), Ordering::Release);
        *gpu.shared.state.lock() = Some(firmware);
        gpu.shared.queue_events();
        Ok(gpu)
    }

    /// Returns the variant of the loaded firmware build. An identified SoC refuses a build it
    /// does not know, before anything is written to the GPU.
    fn firmware_variant(
        dev: &AsahiDrmDevice,
        firmware: &fw::variant::Firmware,
    ) -> Result<&'static fw::Variant> {
        let builds = match firmware {
            fw::variant::Firmware::Fixed(variant) => return Ok(variant),
            fw::variant::Firmware::Identified(builds) => builds,
        };
        let version = Self::os_firmware_version(dev);
        if let Some(build) = builds.iter().find(|build| Some(build.os_version) == version) {
            dev_info!(dev.as_ref(), "GPU firmware: {}\n", build.name);
            return Ok(build.variant);
        }
        dev_err!(
            dev.as_ref(),
            "Unsupported GPU firmware: macOS version {:?} (apple,firmware-version)\n",
            version
        );
        for build in builds.iter() {
            dev_err!(dev.as_ref(), "Supported GPU firmware: {}\n", build.name);
        }
        Err(ENODEV)
    }

    /// The macOS version the boot loader reports for the boot firmware that loaded the GPU
    /// firmware: `apple,firmware-version` of the GPU node, which m1n1 writes from the ADT.
    fn os_firmware_version(dev: &AsahiDrmDevice) -> Option<[u32; 3]> {
        let node = dev.as_ref().as_ref().of_node()?;
        let cells = node
            .get_property::<KVec<u32>>(c_str!("apple,firmware-version"))
            .ok()?;
        cells.as_slice().try_into().ok()
    }

    /// Returns the highest advertised operating-point frequency without changing table order.
    fn max_frequency_khz(dev: &AsahiDrmDevice) -> Result<u32> {
        let node = dev.as_ref().as_ref().of_node().ok_or(EIO)?;
        let opps = node
            .parse_phandle(c_str!("operating-points-v2"), 0)
            .ok_or(EIO)?;
        let mut freq_hz: Option<u64> = None;
        for opp in opps.children() {
            let frequency: u64 = opp.get_property(c_str!("opp-hz"))?;
            freq_hz = Some(freq_hz.map_or(frequency, |highest| highest.max(frequency)));
        }
        Ok((freq_hz.ok_or(EINVAL)? / 1000).try_into()?)
    }

    /// Starts both coprocessors and initializes their firmware.
    fn boot(
        pdev: &platform::Device<Core<'_>>,
        dev: &AsahiDrmDevice,
        cfg: &'static Config,
        regs: regs::G17Resources,
        geometry: mmu::UatGeometry,
        shared: &Arc<Shared>,
    ) -> Result<KBox<Firmware>> {
        let uat = Arc::new(
            mmu::Uat::new(
                dev.into(),
                geometry,
                true,
                mmu::UatFirmware::PublishedRoots {
                    secondary_root_offset: cfg.secondary_root_offset,
                },
                cfg.uat_quirks,
            )?,
            GFP_KERNEL,
        )?;

        // The firmware only samples the kernel lower root's top-level entries when it starts, so
        // everything it uses there must be mapped first.
        let pm_metrics = KernelObject::new(
            dev,
            uat.kernel_lower_vm(),
            Placement::At(cfg.pm_metrics.va),
            cfg.pm_metrics.size,
            mmu::PROT_GPU_FW_SHARED_RW,
            CpuMap::WriteCombined,
        )?;
        let primary = Coprocessor::new(pdev, dev, cfg, &uat, shared, Role::Primary)
            .inspect_err(|e| dev_err!(dev.as_ref(), "Primary coprocessor setup failed: {:?}\n", e))?;
        let secondary = match cfg.secondary_root_offset {
            Some(_) => Some(
                Coprocessor::new(pdev, dev, cfg, &uat, shared, Role::Secondary).inspect_err(|e| {
                    dev_err!(dev.as_ref(), "Secondary coprocessor setup failed: {:?}\n", e)
                })?,
            ),
            None => None,
        };
        let render_global = uat.new_vm(0, geometry.user_range())?;

        let bridge = match cfg.fabric_bridge {
            true => Bridge::enable(regs)?,
            false => Bridge::unused(regs),
        };

        // Install all session ownership before either CPU starts. Error unwinding then runs
        // Firmware::drop, stopping both CPUs before releasing shared objects or translations.
        let mut firmware = KBox::try_init(
            kernel::try_init!(Firmware {
                secondary,
                primary,
                bridge,
                port: port::Port::new()?,
                seen_submissions: 0,
                rings: channel::Rings::new(),
                recovery: recovery::State::new(),
                effort: power::Effort::new(),
                hold: power::Hold::new(),
                registration_power_release_pending: false,
                variant: shared.variant,
                queues <- runtime::Registry::with_waiters(
                    shared.clusters,
                    shared.deflake,
                    shared.descriptor_flags,
                    shared.variant,
                    Some(shared.queues.clone()),
                    Some(shared.renders.clone()),
                ),
                pool: object::Pool::new()?,
                render_rings: kick::RenderRings::new()?,
                render_ids: Arc::new(freelist::RenderIds::new(), GFP_KERNEL)?,
                init: initdata::InitData::new(),
                _render_global: render_global,
                _pm_metrics: Arc::new(pm_metrics, GFP_KERNEL)?,
                uat,
            }),
            GFP_KERNEL,
        )?;
        let session = &mut *firmware;
        session.primary.boot()?;
        if let Some(secondary) = session.secondary.as_mut() {
            secondary.boot()?;
        }
        // The firmware built its root and kernel-window tables while booting.
        session.uat.invalidate_reserved_tables();
        let root_entries = match cfg.adopt_root_entries {
            true => Some(session.uat.adopt_firmware_root_entries()?),
            false => None,
        };
        session.init.build(
            dev,
            cfg,
            &session.uat,
            &session._render_global,
            &session._pm_metrics,
            shared.clusters,
            shared.active_cores,
            shared.variant,
        )?;
        if let Some(root_entries) = root_entries {
            session.uat.confirm_firmware_root_entries(root_entries)?;
        }
        let mailbox: *const Mailbox = &*session.primary.rtkit;
        session.port.attach(session.init.port_views(mailbox)?, session.bridge.regs.clone())?;
        session.uat.mirror_secondary_root()?;
        if cfg.fabric_bridge {
            session.bridge.regs.enable_g17_gating()?;
        }
        let ack = shared.variant.init_ack;
        session.primary.init(session.init.root_va(Role::Primary)?, ack)?;
        if ack == fw::variant::InitAck::Status {
            Self::wait_for_init_accepted(dev, &session.init, &session.primary)?;
        }
        if let Some(secondary) = session.secondary.as_mut() {
            secondary.init(session.init.root_va(Role::Secondary)?, ack)?;
        }
        if shared.variant.power_on_done {
            session.primary.notify(MSG_POWER_ON_DONE)?;
            if let Some(secondary) = session.secondary.as_mut() {
                secondary.notify(MSG_POWER_ON_DONE)?;
            }
        }
        Self::wait_for_opening(dev, &session.init, &session.primary, session.secondary.as_ref())?;
        session
            .uat
            .activate_render_context(&session._render_global)?;
        Ok(firmware)
    }

    /// Waits for a firmware that sends no acknowledgment message to accept its
    /// initialization in status A.
    fn wait_for_init_accepted(
        dev: &AsahiDrmDevice,
        init: &initdata::InitData,
        primary: &Coprocessor,
    ) -> Result {
        for poll in 0..=INIT_ACCEPT_POLLS {
            if primary.state.crashed.load(Ordering::Acquire) {
                return Err(EIO);
            }
            if init.init_accepted()? {
                return Ok(());
            }
            if poll < INIT_ACCEPT_POLLS {
                fsleep(Delta::from_millis(1));
            }
        }
        dev_err!(
            dev.as_ref(),
            "Firmware did not accept its initialization: status A {:#x?}\n",
            init.status_read_words::<4>(Role::Primary, 0x10)?
        );
        Err(ETIMEDOUT)
    }

    /// Waits for both coprocessors to retire their opening device-control records.
    fn wait_for_opening(
        dev: &AsahiDrmDevice,
        init: &initdata::InitData,
        primary: &Coprocessor,
        secondary: Option<&Coprocessor>,
    ) -> Result {
        for poll in 0..=OPENING_RETIRE_POLLS {
            if primary.state.crashed.load(Ordering::Acquire)
                || secondary.is_some_and(|s| s.state.crashed.load(Ordering::Acquire))
            {
                return Err(EIO);
            }
            if init.opening_retired(Role::Primary)?
                && (secondary.is_none() || init.opening_retired(Role::Secondary)?)
            {
                return Ok(());
            }
            if poll < OPENING_RETIRE_POLLS {
                fsleep(Delta::from_millis(OPENING_RETIRE_POLL_MS));
            }
        }
        dev_err!(
            dev.as_ref(),
            "Firmware did not retire its opening records\n"
        );
        Err(ETIMEDOUT)
    }

    /// Stops the firmware and releases everything it used.
    pub(crate) fn shutdown(&self) {
        self.shared.feed.begin_shutdown();
        self.shared.preparations.remove();
        self.shared.stop_queues();
        self.shared.disable_timer(&self.shared.poll);
        self.shared.disable_timer(&self.shared.idle);
        self.shared.disable_work(&self.shared.grow);
        self.shared.disable_work(&self.shared.reclaim_work);
        self.shared.disable_work(&self.shared.work);
        self.shared.disable_work(&self.shared.feed_work);
        self.shared.feed.shutdown();
        self.shared.drain_reclaims();
        self.shared.release_drm();
    }
}

impl gpu::Gpu for Gpu {
    fn syncobj_wait_hint(&self) -> Option<(u32, u32)> {
        self.shared.feed.wait_hint()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn arc_as_any(self: Arc<Self>) -> Arc<dyn Any + Sync + Send> {
        self as Arc<dyn Any + Sync + Send>
    }

    fn stats_snapshot(&self) -> Option<Arc<crate::stats::StatsSnapshot>> {
        None
    }

    fn manager(&self) -> Option<&dyn gpu::GpuManager> {
        None
    }

    fn init(&self) -> Result {
        Ok(())
    }

    fn ids(&self) -> &gpu::SequenceIDs {
        &self.ids
    }

    fn is_crashed(&self) -> bool {
        self.shared.crashed.load(Ordering::Acquire)
    }

    fn get_params(&self, params: &mut uapi::drm_asahi_params_global) -> Result {
        params.features |=
            u64::from(uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_PER_COMMAND_INPUT_SYNCS)
                | u64::from(uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_EXACT_PRIOR_BARRIERS)
                | u64::from(uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_FRAGMENT_BARRIERS)
                | u64::from(uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_BOUNDED_AS_BUILD)
                | sync_output::FEATURE_CUMULATIVE_SIGNALS;
        let variant = self.shared.variant;
        if variant.few_primitives {
            params.features |=
                u64::from(uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_FEW_PRIMITIVES);
        }
        if variant.ray_traversal {
            params.features |=
                u64::from(uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_RENDER_RAY_QUERY);
        }
        params.gpu_generation = self.cfg.gpu_generation as u32;
        params.gpu_variant = self.cfg.identity.gpu_variant as u32;
        params.gpu_revision = self.id.gpu_rev as u32;
        params.chip_id = self.cfg.chip_id;
        params.num_dies = self.id.num_dies;
        params.num_clusters_total = self.id.num_clusters;
        params.num_cores_per_cluster = self.id.num_cores;
        params.max_frequency_khz = self.max_frequency_khz;
        params.usc_generation = self.cfg.identity.usc_generation;
        params.gpu_hal_generation = self.cfg.identity.hal_generation;

        for (i, mask) in self.id.core_masks.iter().enumerate() {
            *(params.core_masks.get_mut(i).ok_or(EIO)?) = (*mask).into();
        }

        Ok(())
    }

    fn base_clock_hz(&self) -> u32 {
        let frequency: u64;
        // SAFETY: CNTFRQ_EL0 is a read-only register containing the architectural counter rate.
        unsafe { core::arch::asm!("mrs {frequency}, CNTFRQ_EL0", frequency = out(reg) frequency) };
        frequency as u32
    }

    fn supports_vm_status(&self) -> bool {
        true
    }

    /// The G17 runtime never programs fault_control into the firmware, so unmapped reads are not
    /// guaranteed to return zero. Advertising soft faults made Mesa point out-of-bounds vec3/vec4
    /// vertex fetches at address 0 and skip bounds checks on bounded loads; on T6040 those reads
    /// returned non-zero data (2026-10-19: every robustness2 rgba32 vertex_attribute_fetch case
    /// failed; all pass with soft faults off).
    fn supports_soft_faults(&self) -> bool {
        false
    }

    fn queue_limits(&self) -> Option<uapi::drm_asahi_queue_limits> {
        Some(uapi::drm_asahi_queue_limits {
            max_queues: kick::queue_limit(self.shared.variant.qid_floor),
            max_in_flight_per_queue: hw::t8140::queues::RENDER_PAIRS_PER_OWNER as u32,
            ..Default::default()
        })
    }

    fn uat_geometry(&self) -> mmu::UatGeometry {
        self.geometry
    }

    fn new_vm(&self, kernel_range: Range<u64>) -> Result<mmu::Vm> {
        let mut state = self.shared.state.lock();
        let firmware = (*state).as_mut().ok_or(ENODEV)?;
        let firmware = &mut **firmware;
        let vm = firmware
            .uat
            .new_vm(self.ids.vm.next(), kernel_range)?
            .with_status()?;
        let mappings = firmware.init.user_aliases(&vm, &firmware._pm_metrics)?;
        let ranges =
            hw::t8140::resources::client_reserved_ranges(self.cfg.pm_metrics.va).ok_or(EINVAL)?;
        let mut reserved = KVec::with_capacity(ranges.len(), GFP_KERNEL)?;
        for range in ranges {
            reserved.push(range, GFP_KERNEL)?;
        }
        vm.install_driver_mappings(mappings, reserved)?;
        Ok(vm)
    }

    fn new_queue(
        &self,
        vm: mmu::Vm,
        _ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        _ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        usc_exec_base: u64,
    ) -> Result<KBox<dyn crate::queue::Queue>> {
        let dev = self.shared.drm()?;
        let owner = self.ids.queue.next();
        let frontend = queue::Frontend::new(
            priority,
            usc_exec_base,
            self.geometry.user_usable_range(),
            self.shared.variant,
        )?;
        let policy = scheduling_policy(priority)?;
        let admission_started = now_ns();
        let (backend, slot) = loop {
            let observed = self.shared.queues.observe();
            match runtime::Backend::new(&self.shared, &vm, owner, policy) {
                Err(error) if error == EAGAIN => continue,
                Err(error) if error == ENOSPC => {
                    // Closed logical queues can still own execution/QoS IDs
                    // until the scheduler release is consumed and reclaimed.
                    // Backend construction has rolled back all temporary leases.
                    let waited = now_ns().saturating_sub(admission_started);
                    let limit = job::TIMEOUT_MS as u64 * 1_000_000;
                    if waited >= limit { return Err(EBUSY); }
                    self.shared.queue_events();
                    // A returned queue slot or reclaimed context wakes this.
                    self.shared
                        .queues
                        .block(observed, (limit - waited).div_ceil(1_000_000) as u32, true)?;
                }
                result => break result?,
            }
        };
        let frontend = match queue::Queue::new(
            &dev,
            backend.context.clone(),
            backend.clone(),
            frontend,
            slot,
        ) {
            Ok(frontend) => frontend,
            Err(error) => {
                job::Backend::release_owner(&*backend);
                return Err(error);
            }
        };
        Ok(KBox::new(frontend, GFP_KERNEL)?)
    }

    fn map_timestamp_buffer(
        &self,
        mut bo: gem::ObjectRef,
        range: Range<usize>,
    ) -> Result<crate::timestamp::Buffer> {
        let mapping = {
            let state = self.shared.state.lock();
            let firmware = (*state).as_ref().ok_or(ENODEV)?;
            let mut addresses = firmware.uat.geometry().kernel_range();
            addresses.start += hw::t8140::dynamic::KERNEL_OFFSET;
            bo.map_range_into_range(
                firmware.uat.kernel_vm(),
                range,
                addresses,
                mmu::UAT_PGSZ as u64,
                mmu::PROT_FW_SHARED_RW,
                false,
            )?
        };
        // The buffer's CPU mapping takes the reservation lock: not under the device mutex.
        crate::timestamp::Buffer::firmware(mapping)
    }
}

/// Scheduling profile of a new logical queue. `file.rs` passes
/// `DRM_ASAHI_PRIORITY_REALTIME - priority` and refuses REALTIME, so HIGH
/// arrives as 1, MEDIUM as 2 and LOW as 3.
fn scheduling_policy(priority: u32) -> Result<fw::queue::Policy> {
    match priority {
        1 => Ok(hw::t8140::scheduling::HIGH),
        2 => Ok(hw::t8140::scheduling::MEDIUM),
        3 => Ok(hw::t8140::scheduling::LOW),
        _ => Err(EINVAL),
    }
}

impl Drop for Gpu {
    fn drop(&mut self) {
        self.shutdown();
    }
}
