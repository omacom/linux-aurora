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
//! `Shared::state` is the device lock. It serializes all firmware-visible publication and all
//! firmware-shared state, including the bounded waits for the firmware that publication needs.
//! Completions are signalled after dropping it. Lock order: `Shared::state` -> UAT shared state
//! -> VM exec lock -> GEM reservation. `RoleState::ack` nests inside nothing.

/// A finite device resource refused an allocation: evaluates to `ENOSPC`.
/// The first refusal at each call site is reported once per module load,
/// naming the resource and site, so a client's ENOSPC can be attributed.
macro_rules! resource_exhausted {
    ($what:literal) => {{
        static REPORTED: ::core::sync::atomic::AtomicBool =
            ::core::sync::atomic::AtomicBool::new(false);
        if !REPORTED.swap(true, ::core::sync::atomic::Ordering::Relaxed) {
            let file = ::core::file!();
            ::kernel::pr_warn!(
                "asahi: G17 {} exhausted ({}:{})\n",
                $what,
                file.rsplit("asahi/").next().unwrap_or(file),
                ::core::line!()
            );
        }
        ::kernel::error::code::ENOSPC
    }};
}

mod buffer;
mod channel;
mod command;
pub(crate) use command::Attachments;
mod completion;
pub(crate) mod context;
mod dependency;
mod event;
mod feed;
mod fence;
mod freelist;
pub(crate) mod fw;
mod ids;
pub(crate) mod hazard;
mod initdata;
mod job;
mod kick;
mod object;
mod power;
mod preparation;
mod qos;
mod queue;
mod recovery;
mod runtime;
pub(crate) mod status;
mod statistics;
mod load;
mod telemetry;
mod teardown;
mod timeout;
mod validation;

use core::any::Any;
use core::ops::Range;
use core::sync::atomic::{
    AtomicBool,
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
    /// Offset of the secondary coprocessor's kernel root in the `pagetables` region.
    pub(crate) secondary_root_offset: u64,
    /// Crash log buffer of each [`Role`].
    pub(crate) crash_buffers: [FixedBuffer; 2],
    /// Parameter buffer page metrics shared with the firmware.
    pub(crate) pm_metrics: FixedBuffer,
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

/// Address bits of a buffer the coprocessor places itself; the coprocessor sets higher bits in
/// its buffer requests.
const RTKIT_BUFFER_ADDR_MASK: u64 = (1 << 40) - 1;
/// Initial contents of crash log buffers, so that a crash log shows what the firmware wrote.
const CRASH_BUFFER_FILL: [u8; 2] = [0xa5, 0x5a];

/// A crash log buffer handed to RTKit.
pub(crate) struct CrashLog {
    iova: usize,
    map: shmem::VMapOwned<AsahiObject>,
}

impl rtkit::Buffer for CrashLog {
    fn iova(&self) -> Result<usize> {
        Ok(self.iova)
    }

    fn buf(&mut self) -> Result<IoSysMap<'_, Region>> {
        Ok(IoSysMap::from((&self.map).as_view()))
    }
}

/// Callback state; initialization ACKs can arrive in hard interrupt context.
#[pin_data]
struct RoleState {
    role: Role,
    shared: Arc<Shared>,
    crash_log: FixedBuffer,
    crash_map: shmem::VMapOwned<AsahiObject>,
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
        data.shared.queue_events();
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
        if (iova as u64) & RTKIT_BUFFER_ADDR_MASK != data.crash_log.va || size > data.crash_log.size
        {
            return Err(EINVAL);
        }
        Ok(CrashLog {
            iova,
            map: data.crash_map.owner().owned_vmap()?,
        })
    }
}

/// A running firmware coprocessor.
///
/// Dropping it stops the CPU before RTKit and the crash log buffer go away.
struct Coprocessor {
    cpu: regs::CpuControl,
    running: bool,
    rtkit: rtkit::RtKit<RoleOps>,
    state: Arc<RoleState>,
    _crash_buffer: KernelObject,
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
        let buffer = &cfg.crash_buffers[index];
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
        let state = Arc::pin_init(
            pin_init!(RoleState {
                role,
                shared: shared.clone(),
                crash_log: FixedBuffer {
                    va: buffer.va,
                    size: buffer.size,
                },
                crash_map,
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
            rtkit: Self::receiver(dev, index, &state)?,
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
        Pin::new(&mut self.rtkit).boot()
    }

    /// Starts the firmware endpoints and hands the firmware its initialization root.
    fn init(&mut self, root_va: u64) -> Result {
        let mut rtkit = Pin::new(&mut self.rtkit);
        rtkit.as_mut().start_endpoint(EP_FIRMWARE)?;
        rtkit.as_mut().start_endpoint(EP_DOORBELL)?;
        rtkit.send_message(
            EP_FIRMWARE,
            MSG_INITDATA | (root_va & MSG_INITDATA_ADDR_MASK),
        )?;
        self.state.wait_for_ack()
    }

    /// Sends a host notification to the firmware.
    fn notify(&mut self, msg: u64) -> Result {
        Pin::new(&mut self.rtkit).send_message(EP_DOORBELL, msg)
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
    secondary: Coprocessor,
    primary: Coprocessor,
    bridge: Bridge,
    rings: channel::Rings,
    recovery: recovery::State,
    effort: power::Effort,
    registration_power_release_pending: bool,
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
        self.secondary.stop();
        self.primary.stop();
        self.pool.stop_after_firmware();
        // RTKit destruction releases the mailbox power references. Clear the bridge while
        // those references still keep the register block powered.
        self.bridge.disable();
    }
}

impl Firmware {
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
        power::release(self)
    }

    /// Called at actual submission publication, not while a queued job is merely being built.
    fn note_submission(&mut self) -> Result {
        self.effort.submit();
        self.apply_effort(power::FULL_EFFORT)
    }

    fn apply_effort(&mut self, effort: u32) -> Result {
        Self::apply_shared_effort(&self.init, &mut self.effort, effort)
    }

    fn apply_shared_effort(
        init: &initdata::InitData,
        state: &mut power::Effort,
        effort: u32,
    ) -> Result {
        if state.applied() == effort && init.effort()? == effort {
            return Ok(());
        }
        init.publish_effort(effort)?;
        state.did_apply(effort);
        Ok(())
    }

    /// Returns the delay in ns for a follow-up idle check. Queue owners report only submitted,
    /// unretired work; quarantined work retaining mappings is excluded.
    fn idle_check(&mut self, now_ns: u64, in_flight: bool) -> Result<Option<u64>> {
        match self.effort.idle(now_ns, in_flight) {
            power::IdleAction::None => Ok(None),
            power::IdleAction::Clear => {
                self.apply_effort(0)?;
                Ok(None)
            }
            power::IdleAction::Recheck(delay) => Ok(Some(delay)),
        }
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
    descriptor_flags: [u32; 2],
    crashed: AtomicBool,
    feed: Arc<feed::Feed>,
    statistics: Arc<statistics::Statistics>,
    #[pin]
    feed_work: Work<Shared, FEED_WORK>,
    preparations: Arc<preparation::Gate>,
    #[pin]
    work: Work<Shared, EVENT_WORK>,
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

    /// Queues the event worker. Callable from any context.
    fn queue_events(self: &Arc<Self>) {
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
        loop {
            let next = self.reclaim.lock().take_one();
            let Some(next) = next else {
                break;
            };
            next.finish();
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
        self.statistics.load.invalidate();
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
        self.statistics.load.invalidate();
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
    _memory: Pin<KBox<kernel::debugfs::File<crate::agx_memory_stats::View>>>,
    _statistics: Pin<KBox<kernel::debugfs::File<StatisticsView>>>,
    _capacity: Pin<KBox<kernel::debugfs::File<CapacityView>>>,
    _telemetry: Option<telemetry::Telemetry>,
}

/// One line of queue resource counts, taken with the device mutex. `contexts`
/// counts leased application contexts against the TTBAT limit; `closing` are
/// those still held by closed queues whose scheduler release has not completed.
struct CapacityView(Arc<Shared>);

impl kernel::debugfs::Writer for CapacityView {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        let state = self.0.state.lock();
        match (*state).as_deref() {
            Some(firmware) => writeln!(f, "version=1 {}", firmware.capacity()),
            None => writeln!(f, "version=1 stopped"),
        }
    }
}

struct StatisticsView(Arc<statistics::Statistics>);

impl kernel::debugfs::Writer for StatisticsView {
    fn write(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        self.0.request_capture(now_ns());
        writeln!(f, "version=2 payload=raw_firmware_words received_clock=monotonic_ns capture_window_ns={}", statistics::CAPTURE_NS)?;
        for role in 0..2 {
            for tag in 16..=30 {
                let Some(report) = self.0.snapshot(role, tag) else {
                    writeln!(f, "role={} tag={:#x} busy=1", role, tag)?;
                    continue;
                };
                if report.count == 0 {
                    continue;
                }
                write!(f, "role={} tag={:#x} count={} received_ns={} words=",
                       role, tag, report.count, report.received_ns)?;
                for word in report.words {
                    write!(f, "{:08x} ", word)?;
                }
                writeln!(f)?;
            }
        }
        Ok(())
    }
}

impl Gpu {
    /// Boots the firmware of a G17 GPU.
    pub(crate) fn new(
        pdev: &platform::Device<Core<'_>>,
        dev: &AsahiDrmDevice,
        cfg: &'static Config,
        regs: regs::G17Resources,
    ) -> Result<Arc<Gpu>> {
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
        let preparations = preparation::Gate::new()?;
        let reclaim = runtime::teardown::ReclaimBatch::new()?;
        let feed = feed::Feed::new()?;
        let statistics = Arc::new(statistics::Statistics::new(), GFP_KERNEL)?;
        let report_statistics = statistics.clone();
        let telemetry = if *crate::module_parameters::stats_export.value() == 0 {
            None
        } else {
            match telemetry::Telemetry::new(pdev.as_ref(), statistics.clone()) {
                Ok(telemetry) => Some(telemetry),
                Err(error) => {
                    dev_warn!(pdev.as_ref(), "GPU telemetry registration failed: {:?}\n", error);
                    None
                }
            }
        };
        let active_cores = id.core_masks.iter().map(|mask| mask.count_ones()).sum();
        let shared = Arc::pin_init(
            pin_init!(Shared {
                drm <- new_mutex!(Some(dev.into()), "g17::Shared::drm"),
                dev: dev.as_ref().as_ref().into(),
                clusters: id.num_clusters,
                active_cores,
                descriptor_flags: [id.perf_control, id.perf_map[0]],
                crashed: AtomicBool::new(false),
                feed,
                statistics,
                feed_work <- new_work!("g17::Shared::feed_work"),
                preparations,
                work <- new_work!("g17::Shared::work"),
                poll <- new_delayed_work!("g17::Shared::poll"),
                idle <- new_delayed_work!("g17::Shared::idle"),
                grow <- new_work!("g17::Shared::grow"),
                reclaim_work <- new_work!("g17::Shared::reclaim_work"),
                reclaim <- new_mutex!(reclaim, "g17::Shared::reclaim"),
                stop <- new_mutex!((), "g17::Shared::stop"),
                state <- new_mutex!(None, "g17::Shared::state"),
                changed <- new_condvar!("g17::Shared::changed"),
            }),
            GFP_KERNEL,
        )?;

        let directory = kernel::debugfs::Dir::new(c"asahi-g17");
        let memory = KBox::pin_init(
            directory.read_only_file(c"memory", crate::agx_memory_stats::View),
            GFP_KERNEL,
        )?;
        let statistics = KBox::pin_init(
            directory.read_only_file(c"statistics", StatisticsView(report_statistics)),
            GFP_KERNEL,
        )?;
        let capacity = KBox::pin_init(
            directory.read_only_file(c"capacity", CapacityView(shared.clone())),
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
                _memory: memory,
                _statistics: statistics,
                _capacity: capacity,
                _telemetry: telemetry,
            },
            GFP_KERNEL,
        )?;
        let firmware = Self::boot(pdev, dev, cfg, regs, geometry, &gpu.shared)?;
        *gpu.shared.state.lock() = Some(firmware);
        gpu.shared.queue_events();
        Ok(gpu)
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
        let secondary = Coprocessor::new(pdev, dev, cfg, &uat, shared, Role::Secondary)
            .inspect_err(|e| dev_err!(dev.as_ref(), "Secondary coprocessor setup failed: {:?}\n", e))?;
        let render_global = uat.new_vm(0, geometry.user_range())?;

        let bridge = Bridge::enable(regs)?;

        // Install all session ownership before either CPU starts. Error unwinding then runs
        // Firmware::drop, stopping both CPUs before releasing shared objects or translations.
        let mut firmware = KBox::try_init(
            kernel::try_init!(Firmware {
                secondary,
                primary,
                bridge,
                rings: channel::Rings::new(),
                recovery: recovery::State::new(),
                effort: power::Effort::new(),
                registration_power_release_pending: false,
                queues <- runtime::Registry::new(shared.clusters, shared.descriptor_flags),
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
        session.secondary.boot()?;
        let root_entries = session.uat.adopt_firmware_root_entries()?;
        session.init.build(
            dev,
            cfg,
            &session.uat,
            &session._render_global,
            &session._pm_metrics,
        )?;
        session.uat.confirm_firmware_root_entries(root_entries)?;
        session.uat.mirror_secondary_root()?;
        session.bridge.regs.enable_g17_gating()?;
        session.primary.init(session.init.root_va(Role::Primary)?)?;
        session
            .secondary
            .init(session.init.root_va(Role::Secondary)?)?;
        session.primary.notify(MSG_POWER_ON_DONE)?;
        session.secondary.notify(MSG_POWER_ON_DONE)?;
        Self::wait_for_opening(dev, &session.init, &session.primary, &session.secondary)?;
        session
            .uat
            .activate_render_context(&session._render_global)?;
        Ok(firmware)
    }

    /// Waits for both coprocessors to retire their opening device-control records.
    fn wait_for_opening(
        dev: &AsahiDrmDevice,
        init: &initdata::InitData,
        primary: &Coprocessor,
        secondary: &Coprocessor,
    ) -> Result {
        for poll in 0..=OPENING_RETIRE_POLLS {
            if primary.state.crashed.load(Ordering::Acquire)
                || secondary.state.crashed.load(Ordering::Acquire)
            {
                return Err(EIO);
            }
            if init.opening_retired(Role::Primary)? && init.opening_retired(Role::Secondary)? {
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
        self.shared.statistics.load.invalidate();
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
            uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_PER_COMMAND_INPUT_SYNCS as u64
                | uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_EXACT_PRIOR_BARRIERS as u64
                | uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_FEW_PRIMITIVES as u64
                | uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_FRAGMENT_BARRIERS as u64
                | uapi::drm_asahi_feature_DRM_ASAHI_FEATURE_RENDER_RAY_QUERY as u64;
        params.gpu_generation = hw::GpuGen::G17 as u32;
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

    fn queue_limits(&self) -> Option<uapi::drm_asahi_queue_limits> {
        Some(uapi::drm_asahi_queue_limits {
            max_queues: mmu::MAX_EXECUTION_CONTEXTS,
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
        let frontend =
            queue::Frontend::new(priority, usc_exec_base, self.geometry.user_usable_range())?;
        let policy = scheduling_policy(priority)?;
        let (backend, slot) = loop {
            match runtime::Backend::new(&self.shared, &vm, owner, policy) {
                Err(error) if error == EAGAIN => continue,
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
    ) -> Result<mmu::KernelMapping> {
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
        )
    }
}

/// Scheduling profile of a new logical queue. `file.rs` passes
/// `DRM_ASAHI_PRIORITY_REALTIME - priority` and refuses HIGH and REALTIME, so
/// MEDIUM arrives as 2 and LOW as 3.
fn scheduling_policy(priority: u32) -> Result<fw::queue::Policy> {
    match priority {
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
