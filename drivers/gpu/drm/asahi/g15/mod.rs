// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! G15 GPU support (T6030, M3 Pro: J514S, J516S; firmware RTKit 2419, the macOS 14.7-14.8 stub).
//!
//! G15 firmware is *classic*: one ring per engine (TA, FS, compute) and ordering expressed by
//! in-ring barrier records that wait on another queue's stamp, unlike the G17 kick-ring/QID
//! model. The boot loader leaves the firmware loaded; this driver identifies the exact build,
//! adopts the running PPL handoff, builds the 49-object initdata graph from this machine's ADT
//! and brings the firmware to READY. All logical queues and VMs share one persistent compute
//! graph and one render graph; the commands of a submission are encoded into compute and pass
//! slots and published as *prefixes* on the firmware's queues, then retired on their stamps.
//! Everything firmware-visible that a failed command may still reference is retained until
//! reboot.
//!
//! # Plugging into the shared driver
//!
//! - The shared platform driver binds this generation to `apple,agx-t6030` and
//!   `apple,agx-t6031` ([`crate::driver::ProbeConfig::G15`]) and probes it with
//!   [`crate::driver::AsahiDriver::probe_g15`]. A probe that fails once the firmware has been
//!   touched keeps the device bound and the session retained ([`retained`]).
//! - [`Gpu`] implements [`crate::gpu::Gpu`]: GET_PARAMS identity, VM creation (`new_vm`),
//!   queues (`new_queue`, see [`queue`]) and timestamp buffers.
//! - User VMs are shared-layer [`mmu::Vm`]s on a [`mmu::Uat::new_adopted`] UAT with tracked
//!   status ([`mmu::Vm::with_tracked_status`]); `file.rs` handles VM_BIND, GEM, syncobjs,
//!   VM_STATUS(_MIRROR) and MADVISE.
//!
//! # Locking
//!
//! [`Shared::state`] is the device lock. It serializes all firmware-visible publication, the
//! device-control ring and every firmware-shared object, including the bounded waits for the
//! firmware that publication, recovery and the device controls need (up to 2 s). Fences are
//! signalled after dropping it ([`job::Completions`]). Lock order: `Shared::state` -> UAT shared
//! state -> VM exec lock -> GEM reservation. The RTKit receive callbacks run in interrupt
//! context and only set atomics and queue the poll work.
//!
//! # Module structure
//!
//! - Platform, firmware boot and UAT handoff: `boot`, `hw`, `adt`, `identity`, `regs`, `uat`,
//!   `object`, `board`, `initdata`, `channel`, `event`, `fw/{mod,initdata,channels}`.
//! - Submission engine: `engine`, `encode`, `slots`, `graph`, `poll`, `fw/{work,graph}`.
//! - Recovery, parameter buffer and power: `recovery`, `pools`, `power`, `fw/buffer`.
//! - UAPI glue and VM integration: `queue`, `command`, `gate`, `job`, `fence`, `vm`,
//!   `timestamp`, `params`, and the hooks in the shared driver (`file.rs`, `gem.rs`, `gpu.rs`,
//!   `mmu.rs`, `pgtable.rs`).

mod adt;
mod board;
mod boot;
mod channel;
mod command;
mod encode;
mod engine;
mod event;
mod fence;
pub(crate) mod fw;
mod gate;
mod graph;
pub(crate) mod hw;
mod identity;
mod initdata;
mod job;
mod object;
mod params;
mod poll;
mod pools;
mod power;
mod queue;
mod recovery;
mod regs;
mod slots;
mod timestamp;
mod uat;
pub(crate) mod vm;

use core::any::Any;
use core::ops::Range;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU64, Ordering};

use kernel::{
    device::{self, Core},
    impl_has_delayed_work, new_delayed_work, new_mutex, platform,
    prelude::*,
    sync::{aref::ARef, Arc, Mutex},
    time::hrtimer::{ArcHrTimerHandle, HrTimer},
    uapi,
    workqueue::{DelayedWork, WorkItem},
};

use crate::driver::AsahiDevice;
use crate::{alloc, gem, gpu, mmu, regs as shared_regs};

/// GPU generation reported to userspace.
pub(crate) const GPU_GENERATION: u32 = 15;

/// Static configuration of a SoC with a G15 GPU.
pub(crate) struct Config {
    /// SoC and GPU name, for the log.
    pub(crate) name: &'static str,
    /// SoC identifier reported to userspace.
    pub(crate) chip_id: u32,
    /// GPU variant reported to userspace.
    pub(crate) gpu_variant: crate::hw::GpuVariant,
    /// GPU revision reported to userspace.
    pub(crate) gpu_revision: u32,
    /// Input address bits translated by each UAT root.
    pub(crate) uat_ias: u32,
    /// Output (physical) address bits of the UAT.
    pub(crate) uat_oas: u32,
    /// Clusters and cores per cluster.
    pub(crate) num_clusters: u32,
    pub(crate) num_cores_per_cluster: u32,
    /// Qualified identification register values.
    pub(crate) hw_ids: &'static [hw::HwId],
    /// Qualified firmware builds.
    pub(crate) firmware: &'static [hw::FirmwareBuild],
    /// Machine compatible this SoC's boards carry.
    pub(crate) machine: &'static CStr,
    /// Where regions, segments and performance states come from, and how the GPU is powered.
    pub(crate) platform: hw::Platform,
    /// Initdata placement and sizes that differ between firmware builds.
    pub(crate) layout: &'static hw::Layout,
    /// Board images.
    pub(crate) board: hw::Board,
    /// Compute register values that scale with the cluster count.
    pub(crate) compute: hw::ComputeRegs,
}

/// Core slots of a SoC: the width of its enabled-core mask.
impl Config {
    pub(crate) const fn core_slots(&self) -> u32 {
        self.num_clusters * self.num_cores_per_cluster
    }
}

/// Whether the firmware has been touched: the module is pinned and the session is retained
/// until reboot, whatever the probe's outcome.
pub(crate) fn retained() -> bool {
    boot::glue::retained()
}

/// Work item ID of the poll work.
const POLL_WORK: u64 = 0;

/// Device state shared with the RTKit callbacks and the poll work.
#[pin_data]
pub(crate) struct Shared {
    pub(crate) dev: ARef<device::Device>,
    /// Latched fail-stop: every VM fails with EIO until reboot.
    pub(crate) crashed: AtomicBool,
    /// Set by the firmware's EP 0x20 event message; consumed by the poll work.
    pub(crate) evented: AtomicBool,
    // --- RTKit callback state and a retained failed session. ---
    /// The first latched error, as a negative errno.
    pub(crate) error: AtomicI32,
    /// The RTKit receive callbacks may queue the poll work.
    pub(crate) started: AtomicBool,
    /// Firmware DATA, where the only RTKit buffer accepted lies.
    pub(crate) data: Range<u64>,
    /// Physical address of the accepted RTKit buffer, 0 before.
    pub(crate) rtkit_buffer: AtomicU64,
    /// A firmware session whose start or backend start failed: retained, rings still drained.
    pub(crate) orphaned: AtomicBool,
    #[pin]
    pub(crate) orphan: Mutex<Option<KBox<boot::Orphan>>>,
    #[pin]
    pub(crate) poll: DelayedWork<Shared, POLL_WORK>,
    // --- The completion/repoll hrtimer and the handle of its current expiry (dropping it
    // cancels the expiry; see `Shared::rearm`). ---
    #[pin]
    pub(crate) resnap: HrTimer<Shared>,
    #[pin]
    pub(crate) resnap_handle: Mutex<Option<ArcHrTimerHandle<Shared>>>,
    /// The device lock. `None` before boot completes.
    #[pin]
    pub(crate) state: Mutex<Option<KBox<Firmware>>>,
}

impl_has_delayed_work! {
    impl HasDelayedWork<Shared, POLL_WORK> for Shared { self.poll }
}

impl WorkItem<POLL_WORK> for Shared {
    type Pointer = Arc<Shared>;
    fn run(this: Arc<Shared>) {
        if this.orphaned.load(Ordering::Acquire) {
            this.drain_orphan();
            return;
        }
        this.run_poll();
    }
}

impl Shared {
    /// Queues the poll work immediately. Callable from any context.
    pub(crate) fn kick(self: &Arc<Self>) {
        self.evented.store(true, Ordering::Release);
        // `mod_delayed_work(.., 0)` so that a kick also runs a poll whose 1-jiffy backup is
        // pending (a plain enqueue is refused then).
        self.poll_now();
    }

    /// Latches the fail-stop state.
    pub(crate) fn fail_stop(&self, error: Error) {
        let _ =
            self.error
                .compare_exchange(0, error.to_errno(), Ordering::AcqRel, Ordering::Acquire);
        if !self.crashed.swap(true, Ordering::AcqRel) {
            dev_err!(
                self.dev.as_ref(),
                "G15: fail-stop ({:?}); GPU unusable until reboot\n",
                error
            );
        }
    }

    /// The latched error, if any.
    pub(crate) fn check(&self) -> Result {
        if !self.crashed.load(Ordering::Acquire) {
            return Ok(());
        }
        match self.error.load(Ordering::Acquire) {
            0 => Err(EIO),
            e => Err(Error::from_errno(e)),
        }
    }

    /// Keeps a firmware session that did not complete until reboot; its receive rings stay
    /// drained by the poll work.
    pub(crate) fn orphan(&self, orphan: boot::Orphan) {
        let mut slot = self.orphan.lock();
        match KBox::new(orphan, GFP_KERNEL) {
            Ok(orphan) => {
                *slot = Some(orphan);
                self.orphaned.store(true, Ordering::Release);
            }
            // Without memory for the box, leak the parts as they are.
            Err(_) => dev_err!(
                self.dev.as_ref(),
                "G15: cannot keep the failed session; leaked\n"
            ),
        }
    }

    /// Drains the receive rings of an orphaned session.
    fn drain_orphan(&self) {
        if let Some(orphan) = self.orphan.lock().as_deref_mut() {
            let _ = orphan.drain();
        }
    }
}

/// Everything that exists while the firmware runs, under the device lock.
///
/// Nothing in here is ever freed while the firmware can reach it: a G15 firmware session cannot
/// be stopped, so dropping this structure leaks the firmware-visible parts on purpose
/// ([`Gpu::retain`]).
pub(crate) struct Firmware {
    /// The SoC configuration.
    pub(crate) cfg: &'static Config,
    /// RTKit, MMIO, regions, the adopted UAT and the host window.
    pub(crate) session: boot::Session,
    /// The initdata graph and its live words (Globals, Control, HardwareData, ...).
    pub(crate) init: initdata::InitData,
    /// Firmware objects: the firmware VA allocator and the retained objects.
    pub(crate) objects: object::Objects,
    /// Submit channels, the device-control ring and the receive rings.
    pub(crate) channels: channel::Channels,
    /// Decoded firmware events (stage counters, last error, growth requests).
    pub(crate) events: event::Events,
    /// The compute and render graphs.
    pub(crate) graph: graph::Graphs,
    /// Compute slots, pass slots and clustering buffers.
    pub(crate) slots: slots::Slots,
    /// Pending jobs, prefixes, horizons and flights.
    pub(crate) engine: engine::Engine,
    /// Halt/resume, kills, deadlines and fail-stop.
    pub(crate) recovery: recovery::Recovery,
    /// Render UMA/PB allocators, their spare set and parameter-buffer growth.
    pub(crate) pools: pools::Pools,
    /// Live VMs and their exclusive TTBAT slots, and the client aliases of every user VM.
    pub(crate) vms: Arc<vm::Registry>,
}

/// A G15 GPU.
pub(crate) struct Gpu {
    cfg: &'static Config,
    /// Identification read at boot.
    pub(crate) id: regs::Identity,
    /// Highest firmware performance state, kHz.
    pub(crate) max_frequency_khz: u32,
    ids: gpu::SequenceIDs,
    pub(crate) shared: Arc<Shared>,
    /// The adopted UAT, for creating and binding user VMs without the device lock.
    pub(crate) uat: Arc<mmu::Uat>,
}

impl Gpu {
    /// Admits and boots a G15 GPU: identity checks, PMP AGX vote, RTKit, UAT handoff, initdata,
    /// READY, then the compute and render graphs. On success the module is pinned: the
    /// firmware session lasts until reboot.
    pub(crate) fn new(
        pdev: &platform::Device<Core>,
        dev: &AsahiDevice,
        cfg: &'static Config,
        res: shared_regs::Resources,
    ) -> Result<Arc<Gpu>> {
        // Admission (probe-time checks), then the firmware start to READY.
        let admitted = boot::admit(pdev, dev, cfg, res)?;
        // Admission holds the GFX domain. A failure before the firmware is touched gives it
        // back (a start that touched the firmware keeps it, retained until reboot).
        let unretained = |error: Error| -> Error {
            if !boot::glue::retained() {
                boot::glue::release_power(dev.as_ref());
            }
            error
        };
        let data = admitted.adt.data.clone();
        let shared = Arc::pin_init(
            pin_init!(Shared {
                dev: dev.as_ref().into(),
                crashed: AtomicBool::new(false),
                evented: AtomicBool::new(false),
                error: AtomicI32::new(0),
                started: AtomicBool::new(false),
                data,
                rtkit_buffer: AtomicU64::new(0),
                orphaned: AtomicBool::new(false),
                orphan <- new_mutex!(None, "g15::Shared::orphan"),
                poll <- new_delayed_work!("g15::Shared::poll"),
                resnap <- HrTimer::new(),
                resnap_handle <- new_mutex!(None, "g15::Shared::resnap_handle"),
                state <- new_mutex!(None, "g15::Shared::state"),
            }),
            GFP_KERNEL,
        )
        .map_err(unretained)?;
        let booted = boot::start(admitted, dev, cfg, &shared).map_err(unretained)?;
        let uat = booted.session.uat()?;
        let (id, max_frequency_khz) = (booted.id, booted.max_frequency_khz);
        // The backend: graphs, slots, pools, engine, recovery and VM registry.
        let firmware = Firmware::start_backend(dev, cfg, booted, &shared)?;
        *shared.state.lock() = Some(firmware);
        // The engine's statistics file (a debugfs failure does not fail the probe).
        if let Err(error) = shared.start_engine() {
            dev_warn!(dev.as_ref(), "G15: no asahi-g15 debugfs ({:?})\n", error);
        }
        let gpu = Arc::new(
            Gpu {
                cfg,
                id,
                max_frequency_khz,
                ids: Default::default(),
                shared,
                uat,
            },
            GFP_KERNEL,
        )?;
        gpu.retain();
        Ok(gpu)
    }

    /// Keeps the firmware session and this module alive until reboot.
    ///
    /// Takes the module reference once. Nothing firmware-reachable is freed on any path: the
    /// RTKit callbacks hold [`Shared`], which holds the [`Firmware`] (or the [`boot::Orphan`] of a
    /// failed start) until reboot.
    pub(crate) fn retain(&self) {
        boot::glue::retain_module();
    }
}

impl Firmware {
    /// Builds the graphs and the engine after READY. On failure the firmware stays at READY,
    /// retained as an orphan of `shared`.
    fn start_backend(
        dev: &AsahiDevice,
        cfg: &'static Config,
        booted: boot::Booted,
        shared: &Arc<Shared>,
    ) -> Result<KBox<Firmware>> {
        let boot::Booted {
            session,
            init,
            mut objects,
            channels,
            events,
            ..
        } = booted;
        dev_info!(
            dev.as_ref(),
            "G15: firmware at READY, starting the backend\n"
        );
        let orphan = |session: boot::Session,
                      objects: object::Objects,
                      channels: channel::Channels,
                      init: initdata::InitData,
                      events: event::Events,
                      error: Error|
         -> Error {
            dev_err!(
                dev.as_ref(),
                "G15: backend start failed ({:?}); firmware retained at READY until reboot\n",
                error
            );
            shared.orphan(boot::Orphan {
                session,
                _objects: objects,
                channels: Some(channels),
                _init: Some(init),
                events,
            });
            error
        };
        let backend = (|| -> Result<_> {
            let graph = graph::Graphs::new(dev, &mut objects)?;
            let slots = slots::Slots::new(&mut objects)?;
            let pools = pools::Pools::new(dev, &mut objects)?;
            let slot = KBox::<Firmware>::new_uninit(GFP_KERNEL)?;
            Ok((graph, slots, pools, slot))
        })();
        let (graph, slots, pools, slot) = match backend {
            Ok(backend) => backend,
            Err(e) => return Err(orphan(session, objects, channels, init, events, e)),
        };
        let mut firmware = KBox::write(
            slot,
            Firmware {
                cfg,
                session,
                init,
                objects,
                channels,
                events,
                graph,
                slots,
                engine: engine::Engine::new(),
                recovery: recovery::Recovery::new(),
                pools,
                vms: vm::Registry::new()?,
            },
        );
        if let Err(e) = firmware.init_graphs() {
            let Firmware {
                session,
                init,
                objects,
                channels,
                events,
                ..
            } = KBox::into_inner(firmware);
            return Err(orphan(session, objects, channels, init, events, e));
        }
        Ok(firmware)
    }
}

impl gpu::Gpu for Gpu {
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn arc_as_any(self: Arc<Self>) -> Arc<dyn Any + Sync + Send> {
        self as Arc<dyn Any + Sync + Send>
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
        params::fill(self, params)
    }

    fn supports_vm_status(&self) -> bool {
        true
    }

    fn queue_limits(&self) -> Option<uapi::drm_asahi_queue_limits> {
        Some(params::queue_limits())
    }

    fn base_clock_hz(&self) -> u32 {
        let frequency: u64;
        // SAFETY: CNTFRQ_EL0 is a read-only register containing the architectural counter rate.
        unsafe { core::arch::asm!("mrs {frequency}, CNTFRQ_EL0", frequency = out(reg) frequency) };
        frequency as u32
    }

    fn uat_geometry(&self) -> mmu::UatGeometry {
        mmu::UatGeometry {
            ias: self.cfg.uat_ias,
            oas: self.cfg.uat_oas,
        }
    }

    fn user_range(&self) -> Range<u64> {
        hw::USER_RANGE
    }

    fn tracked_submission(&self) -> bool {
        true
    }

    fn soft_faults(&self) -> bool {
        false
    }

    fn prepare_new_objects(&self) -> bool {
        true
    }

    // VM_BIND operations per call at most.
    fn bind_batch_limit(&self) -> Option<u32> {
        Some(params::MAX_BINDS)
    }

    fn new_vm(&self, kernel_range: Range<u64>) -> Result<mmu::Vm> {
        vm::new_vm(self, kernel_range)
    }

    fn new_queue(
        &self,
        vm: mmu::Vm,
        _ualloc: Arc<Mutex<alloc::DefaultAllocator>>,
        _ualloc_priv: Arc<Mutex<alloc::DefaultAllocator>>,
        priority: u32,
        usc_exec_base: u64,
    ) -> Result<KBox<dyn crate::queue::Queue>> {
        Ok(KBox::new(
            queue::Queue::new(self, vm, priority, usc_exec_base)?,
            GFP_KERNEL,
        )?)
    }

    fn map_timestamp_buffer(
        &self,
        bo: gem::ObjectRef,
        range: Range<usize>,
    ) -> Result<mmu::KernelMapping> {
        timestamp::map(self, bo, range)
    }
}
