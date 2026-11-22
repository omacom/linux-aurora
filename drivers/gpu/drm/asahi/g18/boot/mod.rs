// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! J873 / T8152 / G18G (26A428, RTKit 3514.0.15) staged live boot.
//!
//! Plan milestone M1 step 1.8 plus the M0 probes. Derived from
//! `m5_live_boot.rs`; the M5 (T8142) path is not touched by anything here.
//!
//! The stage comes from `aurora.g18_initdata_test`. The driver's T8152 probe
//! gate ([`crate::driver`]) has already sampled the three GPU PSD services,
//! mapped SGX and, only if the sgx PSD proved the block powered, read the ID
//! registers. It hands everything over in a [`T8152Probe`].
//!
//! | stage | effect | approval |
//! |-------|--------|----------|
//! | 0     | (driver) refuse with ENODEV before any MMIO | none |
//! | 1     | read-only: PSD state, SGX ID, both wrappers' CPU control/status, GFX/GFX1 post-office status words. Sends nothing. ENODEV | none |
//! | 2     | GPU must ALREADY be powered (iBoot or stage 90). Start GFX1 then GFX ASC CPU, RTKit HELLO/EPMAP with syslog and crashlog attached, 2 s syslog window, stop. ENODEV | REQUIRES USER APPROVAL |
//! | 3     | stage 2 + init-data graph, publish GFX then GFX1 root on EP 0x20 (0x81 doorbell), wait for each 0x0009 ACK, stop. ENODEV | REQUIRES USER APPROVAL |
//! | 4     | stage 3 + 0x89 power-transition-end to both roles, EP 0x21 idle-policy wake, passive 60 s event/trace drain, stop. Returns the retained owner | REQUIRES USER APPROVAL |
//! | 5     | stage 4 plus one owned empty queue registration before the passive observation. No GPU job. Returns the retained owner | REQUIRES USER APPROVAL |
//! | 6     | stage 5 plus a second tag15 binding the owned scheduler/control object. No GPU job | REQUIRES USER APPROVAL |
//! | 7     | stage 6 plus the owned initial UMA pool binding. No GPU job or grow request | REQUIRES USER APPROVAL |
//! | 90    | PMGR2 PSD request 0 for sgx, service 3, ONLY. Poll ACK before reading STATE. No automatic restore. ENODEV | REQUIRES USER APPROVAL |
//! | 91    | request 2 for gfx-asc, service 81 (sgx must already read state 0) | REQUIRES USER APPROVAL |
//! | 92    | request 2 for gfx1-asc, service 82 (sgx must already read state 0) | REQUIRES USER APPROVAL |
//! | 93    | restore: idle request (6) on every GPU service not already idle, in reverse native order 81, 82, 3. ENODEV | REQUIRES USER APPROVAL |
//!
//! Stages 2..7 start the ASC CPUs, which the mailbox provider additionally
//! refuses unless `apple_mailbox.ascwrap_v8_cpu_control=1` (also REQUIRES
//! USER APPROVAL). Stages 90..93 are the only code in the driver that writes
//! a PSD request register; each writes one service (93: only the services
//! it restores), and none is ever implied by another stage. Native order is
//! 90, 92, 91. J873 hardware confirms STATE reads consume ACK: transition
//! waits poll ACK before reading STATE once. Later access gates require
//! each service's active state without requiring an already-consumed ACK.
//!
//! The GFX/GFX1 mailbox providers exist only when the loader enabled their
//! nodes, which it does together with the firmware receipt. A missing
//! provider is reported and turned into ENODEV, never EPROBE_DEFER, so the
//! driver core cannot re-run a stage in a deferral loop. The providers
//! receive by polling unless `apple_mailbox.ascwrap_v8_rx_irq=1` (REQUIRES
//! USER APPROVAL), so no GPU mailbox interrupt is enabled by default.
//!
//! Safety rules this module follows:
//! - Every SGX or ASC wrapper MMIO read follows a PSD read of that block that
//!   proves it powered. Wrapper reads go through the provider's C helpers,
//!   which read the role's PSD block first; SGX reads use
//!   [`regs::t8152_prove_sgx_powered`]. The post-office status word is read
//!   only after this module read the role's PSD block as powered.
//! - Only the provider's own 0x80-byte post-office window is touched (the
//!   status word at +0x20); never the rest of the shared bank.
//! - Once an ASC CPU was asked to start, no firmware-visible owner (UAT,
//!   init-data graph, RTKit transport) is ever freed: the session is stopped
//!   and then leaked, and the module is pinned, until reboot. There is one
//!   firmware init per boot; a second attempt is refused.
//! - On any firmware fault the crashlog is dumped: the RTKit crash callback
//!   dumps a host buffer when there is one, and otherwise (and after any
//!   failed stage) scans the role's `__DATA` reservation for the `CLHE`
//!   crashlog header and dumps what it finds.

use crate::{
    driver::AsahiDrmDevice,
    g18::probe::{G18Runtime, T8152Probe},
    g18::initdata::{G18FirmwareMemory, G18TvbGrowRequest},
    g18::runtime::compute::G18PreparedComputeCommand,
    g18::buffer::pool::G18UmaPoolBinding,
    g18::queue::probe::{G18QueueProbe, QUEUE_ID},
    g18::fw::rtkit as g17_rtkit, mmu, g18::regs as regs,
};
use core::{
    ffi::c_void,
    mem::MaybeUninit,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use kernel::{
    bindings, c_str,
    device::Core,
    error::{from_err_ptr, to_result},
    io::{mem::DevresIoMem, Io, IoSysMap, Mmio, MmioRaw, Region},
    macros::vtable,
    new_mutex, platform,
    prelude::*,
    soc::apple::rtkit,
    sync::{aref::ARef, Arc, Mutex},
    time::{delay::fsleep, Delta, Instant, Monotonic},
};

// ---------------------------------------------------------------------------
// Stages
// ---------------------------------------------------------------------------

/// Read-only probe.
pub(crate) const STAGE_READ_ONLY: u32 = 1;
/// REQUIRES USER APPROVAL: ASC CPU start + RTKit HELLO/EPMAP.
pub(crate) const STAGE_RTKIT: u32 = 2;
/// REQUIRES USER APPROVAL: stage 2 + init-data roots and 0x0009 ACKs.
pub(crate) const STAGE_INITDATA: u32 = 3;
/// REQUIRES USER APPROVAL: stage 3 + openings, idle wake, 60 s heartbeat.
pub(crate) const STAGE_HEARTBEAT: u32 = 4;
/// Owned empty queue registration, then the same 60 s stability observation.
pub(crate) const STAGE_QUEUE_REGISTER: u32 = 5;
/// Register an owned queue and bind its scheduler/control, without a GPU job.
pub(crate) const STAGE_SCHEDULER_BIND: u32 = 6;
/// Bind the owned empty compute UMA pool, without submitting a GPU job.
pub(crate) const STAGE_POOL_BIND: u32 = 7;
/// One captured addition with retained resources and independent completion/output gates.
pub(crate) const STAGE_COMPUTE_ADD: u32 = 8;
pub(crate) const STAGE_RENDER: u32 = crate::g18::boot::render_stage::STAGE_RENDER;
pub(crate) const STAGE_DRM: u32 = 10;
pub(crate) mod render_stage;

#[path = "session.rs"]
mod application;
pub(crate) use application::start_drm;
/// REQUIRES USER APPROVAL: PMGR2 PSD power-on request for sgx (service 3)
/// only. Standalone; no other stage requests PSD power.
pub(crate) const STAGE_PSD_POWER_ON: u32 = 90;
/// REQUIRES USER APPROVAL: as [`STAGE_PSD_POWER_ON`] for gfx-asc (81).
pub(crate) const STAGE_PSD_POWER_ON_GFX: u32 = 91;
/// REQUIRES USER APPROVAL: as [`STAGE_PSD_POWER_ON`] for gfx1-asc (82).
pub(crate) const STAGE_PSD_POWER_ON_GFX1: u32 = 92;
/// REQUIRES USER APPROVAL: idle request on every non-idle GPU service.
pub(crate) const STAGE_PSD_RESTORE_IDLE: u32 = 93;

/// Stage values this module implements. The driver refuses anything else
/// with EINVAL before any MMIO.
pub(crate) const fn stage_is_known(stage: u32) -> bool {
    matches!(
        stage,
        STAGE_READ_ONLY
            | STAGE_RTKIT
            | STAGE_INITDATA
            | STAGE_HEARTBEAT
            | STAGE_QUEUE_REGISTER
            | STAGE_SCHEDULER_BIND
            | STAGE_POOL_BIND
            | STAGE_COMPUTE_ADD
            | STAGE_RENDER
            | STAGE_DRM
            | STAGE_PSD_POWER_ON
            | STAGE_PSD_POWER_ON_GFX
            | STAGE_PSD_POWER_ON_GFX1
            | STAGE_PSD_RESTORE_IDLE
    )
}

/// The single PSD service a power-on stage requests, if `stage` is one.
const fn psd_power_on_service(stage: u32) -> Option<regs::T8152PsdService> {
    match stage {
        STAGE_PSD_POWER_ON => Some(regs::T8152PsdService::Sgx),
        STAGE_PSD_POWER_ON_GFX => Some(regs::T8152PsdService::GfxAsc),
        STAGE_PSD_POWER_ON_GFX1 => Some(regs::T8152PsdService::Gfx1Asc),
        _ => None,
    }
}

/// Stages that write a PSD request register.
const fn stage_writes_psd(stage: u32) -> bool {
    psd_power_on_service(stage).is_some() || stage == STAGE_PSD_RESTORE_IDLE
}

/// Stages that run the loaded firmware and therefore need the loader's
/// G18G firmware receipt and the 42-bit DMA mask.
pub(crate) const fn stage_runs_firmware(stage: u32) -> bool {
    matches!(stage, STAGE_RTKIT | STAGE_INITDATA | STAGE_HEARTBEAT
        | STAGE_QUEUE_REGISTER | STAGE_SCHEDULER_BIND | STAGE_POOL_BIND | STAGE_COMPUTE_ADD | STAGE_RENDER | STAGE_DRM)
}

/// Set once any firmware stage reached its first ASC CPU start request.
/// The firmware initializes once per boot; the module is pinned from then on,
/// so this survives unbind/rebind and refuses a second attempt.
static G18_FIRMWARE_ATTEMPTED: AtomicBool = AtomicBool::new(false);
/// Set once the module is pinned for the retained session.
static G18_MODULE_RETAINED: AtomicBool = AtomicBool::new(false);
/// Bit per PSD request kind written by this module instance: bit 0 sgx,
/// bit 1 gfx-asc, bit 2 gfx1-asc (power-on), bit 3 restore. Each kind is
/// written at most once per module load; a re-bind does not repeat it.
static G18_PSD_REQUESTS: AtomicU32 = AtomicU32::new(0);
const PSD_REQUEST_RESTORE_BIT: u32 = 1 << 3;

const fn psd_request_bit(service: regs::T8152PsdService) -> u32 {
    match service {
        regs::T8152PsdService::Sgx => 1 << 0,
        regs::T8152PsdService::GfxAsc => 1 << 1,
        regs::T8152PsdService::Gfx1Asc => 1 << 2,
    }
}

// ---------------------------------------------------------------------------
// Fixed 26A428 protocol facts (plan step 1.7, FW_PROVEN unless noted)
// ---------------------------------------------------------------------------

const EP_GFX_MESSAGES: u8 = g17_rtkit::ENDPOINT_GFX_MESSAGES; // 0x20
const EP_GFX_INTERRUPTS: u8 = g17_rtkit::ENDPOINT_GFX_INTERRUPTS; // 0x21
/// Init-data ACK on EP 0x20 (b000 0x24c78), identical to M5.
const INITDATA_ACK: u64 = 0x0009_0000_0000_0000;
/// EP 0x21 device-control doorbell for the idle-policy wake (M5 value,
/// unchanged in the 26A428 EP 0x21 handler 0x254cc-0x25610).
const IDLE_POLICY_WAKE: u64 = 0x0084_0000_0000_0011;
/// The same doorbell after any other primary control record (g17
/// MSG_CONTROL_NOTIFY).
const CONTROL_NOTIFY: u64 = IDLE_POLICY_WAKE;

const ACK_POLLS: u32 = 200; // x 10 ms
const CONTROL_POLLS: u32 = 1000; // x 1 ms
const SYSLOG_WINDOW_MS: u32 = 2000;
const HEARTBEAT_MS: u32 = 60_000;
const HEARTBEAT_STEP_MS: u32 = 100;
const MESSAGE_LOG_BUDGET: u32 = 32;

/// PSD service block layout (the GPU power tool's contract).
const PSD_STATE: usize = 0x80;
const PSD_REQUEST: usize = 0x90;
const PSD_ACK: usize = 0xbc;
const PSD_BLOCK_SIZE: u64 = 0x100;
/// Cold-boot idle state of all three GPU services on J873.
const PSD_STATE_IDLE: u32 = 6;
const PSD_POLLS: u32 = 1000; // x 1 ms, as gpu_power.py (1 s)

/// ASCWrap v8 CPU control RUN bit (m1n1 asc.c; INFERRED for the GFX
/// wrappers, see the mailbox provider).
const ASCWRAP_V8_CPU_RUN: u32 = 1 << 4;
/// Counted-FIFO status word inside the role's 0x80-byte post-office window.
const POSTOFFICE_STATUS: usize = 0x20;
const POSTOFFICE_STATUS_WINDOW: usize = POSTOFFICE_STATUS + core::mem::size_of::<u32>();

/// RTKit crashlog container (`CLHE` header, sections, `CLHE` trailer).
const fn fourcc(a: u8, b: u8, c: u8, d: u8) -> u32 {
    ((a as u32) << 24) | ((b as u32) << 16) | ((c as u32) << 8) | d as u32
}
const CRASHLOG_HEADER: u32 = fourcc(b'C', b'L', b'H', b'E');
const CRASHLOG_STRING: u32 = fourcc(b'C', b's', b't', b'r');
const CRASHLOG_REGISTERS: u32 = fourcc(b'C', b'r', b'g', b'8');
/// Largest crashlog this module copies and hex-dumps (the ADT
/// coredump-copy-buffer is 0x8000).
const CRASHLOG_MAX_BYTES: usize = 0x8000;
/// Granule of the `__DATA` crashlog scan.
const CRASHLOG_SCAN_STEP: usize = 0x1000;

// ---------------------------------------------------------------------------
// Roles
// ---------------------------------------------------------------------------

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum G18Role {
    /// Mailbox index 0, ASC `gfx-asc`, post-office mailbox 1.
    Gfx,
    /// Mailbox index 1, ASC `gfx1-asc`, post-office mailbox 2.
    Gfx1,
}

impl G18Role {
    const ALL: [Self; 2] = [Self::Gfx, Self::Gfx1];
    /// M5 order, carried to G18G (INFERRED; the 26A428 bootFirmware publish
    /// order is not re-derived): GFX1 starts and publishes first.
    const START_ORDER: [Self; 2] = [Self::Gfx1, Self::Gfx];
    /// Reverse of the start order.
    const STOP_ORDER: [Self; 2] = [Self::Gfx, Self::Gfx1];

    const fn index(self) -> usize {
        match self {
            Self::Gfx => 0,
            Self::Gfx1 => 1,
        }
    }

    const fn name(self) -> &'static str {
        match self {
            Self::Gfx => "GFX",
            Self::Gfx1 => "GFX1",
        }
    }

    /// `enum apple_mbox_ascwrap_v6_role` value the provider reports.
    const fn lifecycle_role(self) -> u32 {
        match self {
            Self::Gfx => 1,
            Self::Gfx1 => 2,
        }
    }

    /// ADT gfx-asc/gfx1-asc reg[0] (iop,ascwrap-v8).
    const fn wrapper(self) -> (u64, u64) {
        match self {
            Self::Gfx => (0x5_8260_0000, 0x7_4000),
            Self::Gfx1 => (0x5_82e0_0000, 0x7_4000),
        }
    }

    /// ADT gfx-asc/gfx1-asc reg[1].
    const fn iop_vbar(self) -> (u64, u64) {
        match self {
            Self::Gfx => (0x5_8205_0000, 0x8),
            Self::Gfx1 => (0x5_8285_0000, 0x8),
        }
    }

    /// The role's 0x80-byte post-office window (informational).
    const fn postoffice_window(self) -> u64 {
        match self {
            Self::Gfx => 0x3_1602_0100,
            Self::Gfx1 => 0x3_1602_0200,
        }
    }

    const fn psd(self) -> regs::T8152PsdService {
        match self {
            Self::Gfx => regs::T8152PsdService::GfxAsc,
            Self::Gfx1 => regs::T8152PsdService::Gfx1Asc,
        }
    }

    const fn data_region_name(self) -> &'static CStr {
        match self {
            Self::Gfx => c_str!("gfx-data"),
            Self::Gfx1 => c_str!("gfx1-data"),
        }
    }

    /// The `__DATA_SHARED_RO` reservation right after `__DATA`. On J873 the
    /// RTKit crash callback reports the firmware-owned crash buffer here
    /// (e.g. GFX1: 0x10003900000, 0x8000 bytes).
    const fn shared_ro_region_name(self) -> &'static CStr {
        match self {
            Self::Gfx => c_str!("gfx-data-shared-ro"),
            Self::Gfx1 => c_str!("gfx1-data-shared-ro"),
        }
    }

    /// 26A428 `__DATA` sizes (b000 / b010), as `Uat::new_t8152` checks.
    const fn data_size(self) -> u64 {
        match self {
            Self::Gfx => 0xb_c000,
            Self::Gfx1 => 0xb_4000,
        }
    }
}

// Compile-time guards on the stage gates and the fixed J873 facts.
const _: () = {
    assert!(!stage_is_known(0) && !stage_is_known(11) && !stage_is_known(89) && !stage_is_known(94));
    assert!(stage_is_known(STAGE_RENDER) && stage_runs_firmware(STAGE_RENDER));
    assert!(!stage_writes_psd(STAGE_RENDER));
    assert!(stage_is_known(STAGE_READ_ONLY) && stage_is_known(STAGE_PSD_POWER_ON));
    assert!(stage_is_known(STAGE_PSD_POWER_ON_GFX) && stage_is_known(STAGE_PSD_POWER_ON_GFX1));
    assert!(stage_is_known(STAGE_PSD_RESTORE_IDLE));
    // Neither the read-only probe nor a power stage runs firmware, and no
    // firmware stage writes PSD.
    assert!(!stage_runs_firmware(STAGE_READ_ONLY) && !stage_writes_psd(STAGE_READ_ONLY));
    assert!(!stage_runs_firmware(STAGE_PSD_POWER_ON) && !stage_runs_firmware(STAGE_PSD_POWER_ON_GFX));
    assert!(!stage_runs_firmware(STAGE_PSD_POWER_ON_GFX1) && !stage_runs_firmware(STAGE_PSD_RESTORE_IDLE));
    assert!(stage_runs_firmware(STAGE_RTKIT) && stage_runs_firmware(STAGE_INITDATA));
    assert!(stage_runs_firmware(STAGE_HEARTBEAT));
    assert!(stage_is_known(STAGE_POOL_BIND) && stage_runs_firmware(STAGE_POOL_BIND));
    assert!(stage_is_known(STAGE_COMPUTE_ADD) && stage_runs_firmware(STAGE_COMPUTE_ADD));
    assert!(!stage_writes_psd(STAGE_RTKIT) && !stage_writes_psd(STAGE_INITDATA));
    assert!(!stage_writes_psd(STAGE_HEARTBEAT));
    assert!(!stage_writes_psd(STAGE_POOL_BIND));
    assert!(!stage_writes_psd(STAGE_COMPUTE_ADD));
    // Exactly one service per power-on stage, sgx first.
    assert!(matches!(psd_power_on_service(STAGE_PSD_POWER_ON), Some(regs::T8152PsdService::Sgx)));
    assert!(matches!(psd_power_on_service(STAGE_PSD_POWER_ON_GFX), Some(regs::T8152PsdService::GfxAsc)));
    assert!(matches!(psd_power_on_service(STAGE_PSD_POWER_ON_GFX1), Some(regs::T8152PsdService::Gfx1Asc)));
    assert!(psd_power_on_service(STAGE_PSD_RESTORE_IDLE).is_none() && stage_writes_psd(STAGE_PSD_RESTORE_IDLE));
    assert!(psd_request_bit(regs::T8152PsdService::Sgx) | psd_request_bit(regs::T8152PsdService::GfxAsc)
        | psd_request_bit(regs::T8152PsdService::Gfx1Asc) | PSD_REQUEST_RESTORE_BIT == 0xf);
    // GFX1 starts and publishes first; stop is the reverse.
    assert!(G18Role::START_ORDER[0].index() == 1 && G18Role::START_ORDER[1].index() == 0);
    assert!(G18Role::STOP_ORDER[0].index() == 0 && G18Role::STOP_ORDER[1].index() == 1);
    // PSD services 81/82 and the post-office windows of mailboxes 1/2
    // ((2 * mb + !po) * 0x80 into 0x316020000).
    assert!(G18Role::Gfx.psd().physical_base() == 0x3_03a1_5100);
    assert!(G18Role::Gfx1.psd().physical_base() == 0x3_03a1_5200);
    assert!(G18Role::Gfx.postoffice_window() == 0x3_1602_0000 + 2 * 0x80);
    assert!(G18Role::Gfx1.postoffice_window() == 0x3_1602_0000 + 4 * 0x80);
    assert!(POSTOFFICE_STATUS_WINDOW <= 0x80);
    // __DATA sizes are 16 KiB multiples and hold at least one crash view.
    assert!(G18Role::Gfx.data_size() % 0x4000 == 0 && G18Role::Gfx1.data_size() % 0x4000 == 0);
    assert!(CRASHLOG_MAX_BYTES as u64 <= G18Role::Gfx1.data_size());
    assert!(CRASHLOG_HEADER == 0x434c_4845);
};

// ---------------------------------------------------------------------------
// PSD service blocks
// ---------------------------------------------------------------------------

const fn psd_resource_name(service: regs::T8152PsdService) -> &'static CStr {
    match service {
        regs::T8152PsdService::Sgx => c_str!("psd-sgx"),
        regs::T8152PsdService::GfxAsc => c_str!("psd-gfx-asc"),
        regs::T8152PsdService::Gfx1Asc => c_str!("psd-gfx1-asc"),
    }
}

/// One GPU PSD service block, mapped (not claimed) from the GPU node for the
/// duration of a stage. Only [`PsdBlock::request`] writes, and only stages
/// 90..93 call it.
struct PsdBlock {
    service: regs::T8152PsdService,
    io: DevresIoMem,
}

impl PsdBlock {
    fn map(pdev: &platform::Device<Core<'_>>, service: regs::T8152PsdService) -> Result<Self> {
        let name = psd_resource_name(service);
        let resource = pdev.resource_by_name(name).ok_or(ENODEV)?;
        if resource.start() != service.physical_base() || resource.size() != PSD_BLOCK_SIZE {
            dev_err!(
                pdev.as_ref(),
                "G18_PSD service {} resource {:#x}+{:#x} is not {:#x}+{:#x}\n",
                service.service_id(),
                resource.start(),
                resource.size(),
                service.physical_base(),
                PSD_BLOCK_SIZE
            );
            return Err(EINVAL);
        }
        let request = pdev.io_request_by_name(name).ok_or(ENODEV)?;
        let io = request.iomap()?.into_devres()?;
        Ok(Self { service, io })
    }

    fn sample(&self) -> Result<regs::T8152PsdSample> {
        let io = self.io.try_access().ok_or(ENODEV)?;
        // Reading STATE consumes ACK; capture ACK first for telemetry.
        let ack = io.try_read32(PSD_ACK)?;
        let state = io.try_read32(PSD_STATE)?;
        Ok(regs::T8152PsdSample {
            service: self.service,
            state,
            ack,
        })
    }

    /// Poll ACK without touching STATE until the transition completes.
    fn acknowledged_sample(&self) -> Result<Option<regs::T8152PsdSample>> {
        let io = self.io.try_access().ok_or(ENODEV)?;
        let ack = io.try_read32(PSD_ACK)?;
        if ack & 1 == 0 {
            return Ok(None);
        }
        let state = io.try_read32(PSD_STATE)?;
        Ok(Some(regs::T8152PsdSample { service: self.service, state, ack }))
    }

    /// REQUIRES USER APPROVAL. Accept only this service's native active
    /// state (SGX 0, ASCs 2) or idle (6). Called by stages 90..93 only.
    fn request(&self, value: u32) -> Result {
        if value != self.service.active_state() && value != PSD_STATE_IDLE {
            return Err(EINVAL);
        }
        let io = self.io.try_access().ok_or(ENODEV)?;
        io.try_write32(value, PSD_REQUEST)
    }
}

fn psd_line(sample: &regs::T8152PsdSample) -> (u32, u32, u32, u32, &'static str) {
    (
        sample.service.service_id(),
        sample.state,
        sample.power_state(),
        sample.ack,
        sample.verdict(),
    )
}

fn log_psd(pdev: &platform::Device<Core<'_>>, label: &str, sample: &regs::T8152PsdSample) {
    let (id, state, power, ack, verdict) = psd_line(sample);
    dev_info!(
        pdev.as_ref(),
        "G18_PSD {} service={} state={:#x} power-state={} ack={:#x} -> {}\n",
        label,
        id,
        state,
        power,
        ack,
        verdict
    );
}

/// Fresh read-only samples of all three GPU services.
fn read_all_psd(pdev: &platform::Device<Core<'_>>) -> Result<[regs::T8152PsdSample; 3]> {
    let mut samples = [regs::T8152PsdSample {
        service: regs::T8152PsdService::Sgx,
        state: 0,
        ack: 0,
    }; 3];
    for (slot, service) in samples.iter_mut().zip(regs::T8152PsdService::ALL) {
        *slot = regs::t8152_read_psd(pdev, service)?;
    }
    Ok(samples)
}

// ---------------------------------------------------------------------------
// Mailbox providers (apple,t8152-gfx-ascwrap-v8)
// ---------------------------------------------------------------------------

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
struct G18Lifecycle {
    role: u32,
    wrapper: (u64, u64),
    iop_vbar: (u64, u64),
    missing: u32,
}

/// Borrowed view of one ASCWrap v8 provider. The C provider owns every
/// mapping and every lifecycle transition, and reads the role's PSD block
/// before each wrapper access. This view never touches the wrapper itself;
/// the one direct read it makes is the post-office status word, gated here
/// on the role's PSD block.
struct G18Provider {
    mbox: *mut bindings::apple_mbox,
    role: G18Role,
}

// SAFETY: the provider is kept alive by the device link apple_mbox_get()
// establishes; every mutation is serialized inside the C provider.
unsafe impl Send for G18Provider {}
// SAFETY: as above; the only Rust-side MMIO is one relaxed status read.
unsafe impl Sync for G18Provider {}

impl G18Provider {
    /// Look up the role's provider. A provider that is not bound makes
    /// apple_mbox_get() return EPROBE_DEFER; that is reported and turned into
    /// ENODEV here so that no stage can put the GPU device into a deferral
    /// loop (every retry would repeat the PSD and SGX reads). The loader
    /// enables both mailbox nodes only together with the firmware receipt.
    fn get(dev: &kernel::device::Device, role: G18Role) -> Result<Self> {
        // SAFETY: `dev` is live; apple_mbox_get() links the provider to it.
        let mbox = unsafe {
            from_err_ptr(bindings::apple_mbox_get(dev.as_raw(), role.index().try_into()?))
        };
        match mbox {
            Ok(mbox) => Ok(Self { mbox, role }),
            Err(error) => Err(no_probe_defer(dev, role.name(), "mailbox provider", error)),
        }
    }

    fn lifecycle(&self) -> Result<G18Lifecycle> {
        let mut raw = MaybeUninit::<bindings::apple_mbox_ascwrap_v6_lifecycle>::uninit();
        // SAFETY: `self.mbox` is a live provider and `raw` is writable.
        to_result(unsafe {
            bindings::apple_mbox_ascwrap_v6_get_lifecycle(self.mbox, raw.as_mut_ptr())
        })?;
        // SAFETY: the successful call initialized every field.
        let raw = unsafe { raw.assume_init() };
        Ok(G18Lifecycle {
            role: raw.role as u32,
            wrapper: (raw.wrapper_start, raw.wrapper_size),
            iop_vbar: (raw.iop_vbar_start, raw.iop_vbar_size),
            missing: raw.missing,
        })
    }

    /// Exact role and resources of the J873 v8 provider. The 0x74000
    /// wrapper size identifies the v8 provider, whose `regs` mapping is the
    /// role's 0x80-byte post-office window.
    fn validate(&self, dev: &kernel::device::Device) -> Result<G18Lifecycle> {
        let lifecycle = self.lifecycle()?;
        if lifecycle.role != self.role.lifecycle_role()
            || lifecycle.wrapper != self.role.wrapper()
            || lifecycle.iop_vbar != self.role.iop_vbar()
        {
            dev_err!(
                dev,
                "G18 {} mailbox provider mismatch: role={} wrapper={:x?} iop-vbar={:x?}; expected role={} wrapper={:x?} iop-vbar={:x?}\n",
                self.role.name(),
                lifecycle.role,
                lifecycle.wrapper,
                lifecycle.iop_vbar,
                self.role.lifecycle_role(),
                self.role.wrapper(),
                self.role.iop_vbar()
            );
            return Err(EINVAL);
        }
        Ok(lifecycle)
    }

    fn require_safe_cpu_lifecycle(&self) -> Result {
        // SAFETY: capability check on the live provider.
        to_result(unsafe { bindings::apple_mbox_ascwrap_v6_require_safe_cpu_lifecycle(self.mbox) })
    }

    /// REQUIRES USER APPROVAL (the provider refuses with EPERM unless
    /// `apple_mailbox.ascwrap_v8_cpu_control=1`; it also refuses an unpowered
    /// block).
    fn start_cpu(&self) -> Result {
        // SAFETY: the provider serializes the lifecycle and owns the MMIO.
        to_result(unsafe { bindings::apple_mbox_ascwrap_v6_start_cpu(self.mbox) })
    }

    /// As [`Self::start_cpu`]; a no-op unless this provider started the CPU.
    fn stop_cpu(&self) -> Result {
        // SAFETY: as above.
        to_result(unsafe { bindings::apple_mbox_ascwrap_v6_stop_cpu(self.mbox) })
    }

    /// CPU_CONTROL (+0x44) and CPU_STATUS (+0x48). The provider reads the
    /// role's PSD first and returns EHOSTDOWN, without touching the wrapper,
    /// when the block is not powered.
    fn cpu_state(&self) -> Result<(u32, u32)> {
        let mut control = 0;
        let mut status = 0;
        // SAFETY: live provider; both outputs are writable stack storage.
        to_result(unsafe {
            bindings::apple_mbox_ascwrap_v6_get_cpu_state(self.mbox, &mut control, &mut status)
        })?;
        Ok((control, status))
    }

    /// Read the counted-FIFO status word (+0x20) of this role's post-office
    /// window, after reading the role's PSD block. Returns `None` for the
    /// word when the block is not powered. Call only after [`Self::validate`].
    fn postoffice_status(
        &self,
        pdev: &platform::Device<Core<'_>>,
    ) -> Result<(regs::T8152PsdSample, Option<u32>)> {
        let psd = regs::t8152_read_psd(pdev, self.role.psd())?;
        if !psd.powered() {
            return Ok((psd, None));
        }
        // SAFETY: the provider (device-link protected) owns this mapping.
        let (window, removing) = unsafe {
            (
                (*self.mbox).regs as usize,
                core::ptr::read_volatile(core::ptr::addr_of!((*self.mbox).removing)),
            )
        };
        if window == 0 || removing {
            return Err(ENODEV);
        }
        let raw = MmioRaw::<Region<POSTOFFICE_STATUS_WINDOW>>::new_region(window, POSTOFFICE_STATUS_WINDOW)?;
        // SAFETY: `raw` lies inside the provider's 0x80-byte window and
        // covers exactly the single status read below.
        let status = unsafe { Mmio::from_raw(raw) }.relaxed().read32(POSTOFFICE_STATUS);
        Ok((psd, Some(status)))
    }
}

/// Map EPROBE_DEFER to ENODEV with an explanation; pass anything else on.
fn no_probe_defer(dev: &kernel::device::Device, role: &str, what: &str, error: Error) -> Error {
    if error != EPROBE_DEFER {
        return error;
    }
    dev_err!(
        dev,
        "G18 {} {} not bound: the GFX/GFX1 mailbox nodes are disabled (the loader enables them only together with the firmware receipt) or the provider failed to probe; ENODEV, not deferring\n",
        role,
        what
    );
    ENODEV
}

fn log_cpu_state(dev: &kernel::device::Device, role: G18Role, label: &str, provider: &G18Provider) {
    match provider.cpu_state() {
        Ok((control, status)) => dev_info!(
            dev,
            "G18_ASC {} {} CPU_CONTROL={:#x} (RUN={}) CPU_STATUS={:#x}\n",
            role.name(),
            label,
            control,
            (control & ASCWRAP_V8_CPU_RUN != 0) as u32,
            status
        ),
        Err(error) if error == regs::t8152_not_powered() => dev_info!(
            dev,
            "G18_ASC {} {} wrapper not read: role PSD not powered\n",
            role.name(),
            label
        ),
        Err(error) => dev_err!(
            dev,
            "G18_ASC {} {} CPU state read failed ({:?})\n",
            role.name(),
            label,
            error
        ),
    }
}

// ---------------------------------------------------------------------------
// Crashlog dump
// ---------------------------------------------------------------------------

fn le_u32(raw: &[u8], offset: usize) -> Option<u32> {
    Some(u32::from_le_bytes(raw.get(offset..offset.checked_add(4)?)?.try_into().ok()?))
}

fn le_u64(raw: &[u8], offset: usize) -> Option<u64> {
    Some(u64::from_le_bytes(raw.get(offset..offset.checked_add(8)?)?.try_into().ok()?))
}

fn fourcc_text(kind: u32) -> [u8; 4] {
    let mut text = kind.to_be_bytes();
    for byte in text.iter_mut() {
        if !byte.is_ascii_graphic() {
            *byte = b'.';
        }
    }
    text
}

/// Log a crashlog: parsed sections (strings, the exception register frame
/// with the G17P `Crg8` layout, INFERRED for G18G) and a hex dump of every
/// non-zero 32-byte line, bounded to [`CRASHLOG_MAX_BYTES`].
fn dump_crashlog(dev: &kernel::device::Device, role: &str, source: &str, raw: &[u8]) {
    let header = le_u32(raw, 0).unwrap_or(0);
    let declared = le_u32(raw, 8).unwrap_or(0) as usize;
    let valid = header == CRASHLOG_HEADER && declared >= 0x20 && declared <= raw.len();
    let total = if valid { declared } else { raw.len() }.min(CRASHLOG_MAX_BYTES);
    dev_err!(
        dev,
        "G18_CRASHLOG {} source={} bytes={} header={} declared={:#x} dumped={:#x}\n",
        role,
        source,
        raw.len(),
        core::str::from_utf8(&fourcc_text(header)).unwrap_or("????"),
        declared,
        total
    );

    if valid {
        let mut offset = 0x20usize;
        for _ in 0..64 {
            let (Some(kind), Some(size)) = (le_u32(raw, offset), le_u32(raw, offset + 12)) else {
                break;
            };
            let size = size as usize;
            if kind == CRASHLOG_HEADER {
                break;
            }
            let Some(end) = offset.checked_add(size) else {
                break;
            };
            if size < 16 || end > declared {
                dev_err!(dev, "G18_CRASHLOG {} section at {:#x} has bad size {:#x}\n", role, offset, size);
                break;
            }
            dev_err!(
                dev,
                "G18_CRASHLOG {} section {} at {:#x} size {:#x}\n",
                role,
                core::str::from_utf8(&fourcc_text(kind)).unwrap_or("????"),
                offset,
                size
            );
            let body = offset + 16;
            if kind == CRASHLOG_STRING && size >= 20 {
                let id = le_u32(raw, body).unwrap_or(0);
                let bytes = &raw[body + 4..end];
                let length = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len()).min(256);
                match core::str::from_utf8(&bytes[..length]) {
                    Ok(message) => dev_err!(dev, "G18_CRASHLOG {} message {}: {}\n", role, id, message),
                    Err(_) => dev_err!(dev, "G18_CRASHLOG {} message {}: (not UTF-8)\n", role, id),
                }
            } else if kind == CRASHLOG_REGISTERS && size >= 16 + 0x350 {
                // Crg8 begins with an eight-byte frame descriptor, then X0.
                // Verified against the 0x401 frame in the stage6 CLHE dump.
                let reg = |index: usize| le_u64(raw, body + 8 + index * 8).unwrap_or(0);
                for first in (0..31).step_by(4) {
                    let last = (first + 4).min(31);
                    let mut words = [0u64; 4];
                    for (slot, index) in words.iter_mut().zip(first..last) {
                        *slot = reg(index);
                    }
                    dev_err!(dev, "G18_CRASHLOG {} x{}..x{}: {:x?}\n", role, first, last - 1,
                        &words[..last - first]);
                }
                dev_err!(
                    dev,
                    "G18_CRASHLOG {} exception pc={:#x} far={:#x} esr={:#x} sp={:#x} psr={:#x}\n",
                    role,
                    le_u64(raw, body + 0x108).unwrap_or(0),
                    le_u64(raw, body + 0x330).unwrap_or(0),
                    le_u32(raw, body + 0x340).unwrap_or(0),
                    le_u64(raw, body + 0x100).unwrap_or(0),
                    le_u64(raw, body + 0x110).unwrap_or(0)
                );
            }
            offset = end;
        }
    }

    let mut zero_lines = 0u32;
    let mut offset = 0usize;
    while offset + 32 <= total {
        let mut words = [0u32; 8];
        for (index, word) in words.iter_mut().enumerate() {
            *word = le_u32(raw, offset + index * 4).unwrap_or(0);
        }
        if words.iter().all(|word| *word == 0) {
            zero_lines += 1;
        } else {
            dev_err!(
                dev,
                "G18_CRASHLOG {} +{:05x}: {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x} {:08x}\n",
                role, offset, words[0], words[1], words[2], words[3], words[4], words[5],
                words[6], words[7]
            );
        }
        offset += 32;
    }
    dev_err!(dev, "G18_CRASHLOG {} end; {} all-zero lines omitted\n", role, zero_lines);
}

/// Invalidate the AP's view of a range before reading memory the GPU
/// firmware wrote (clean+invalidate: nothing here is ever written by the AP).
fn invalidate_dcache_range(pointer: *const u8, length: usize) {
    let ctr: u64;
    // SAFETY: CTR_EL0 is readable at EL1.
    unsafe {
        core::arch::asm!("mrs {ctr}, ctr_el0", ctr = out(reg) ctr,
            options(nomem, nostack, preserves_flags));
    }
    let line = 4usize << ((ctr >> 16) & 0xf);
    let mut address = pointer as usize & !(line - 1);
    let end = pointer as usize + length;
    while address < end {
        // SAFETY: `address` lies in a live cacheable mapping owned by the
        // caller; clean+invalidate by VA has no architectural side effect on
        // the data.
        unsafe {
            core::arch::asm!("dc civac, {address}", address = in(reg) address,
                options(nostack, preserves_flags));
        }
        address += line;
    }
    // SAFETY: barrier only.
    unsafe { core::arch::asm!("dsb sy", options(nostack, preserves_flags)) };
}

/// Read-only write-back mapping of a firmware carveout range (not Linux
/// RAM), used only to find and copy a crashlog.
struct PhysMap {
    pointer: NonNull<c_void>,
    size: usize,
}

impl PhysMap {
    fn new(physical: u64, size: usize) -> Result<Self> {
        if size == 0 || physical & (crate::pgtable::UAT_PGMSK as u64) != 0 {
            return Err(EINVAL);
        }
        // SAFETY: page-aligned firmware reservation validated by the caller
        // against the GPU node; released by Drop and only read.
        let pointer = unsafe { bindings::memremap(physical, size, bindings::MEMREMAP_WB as _) };
        Ok(Self { pointer: NonNull::new(pointer).ok_or(ENOMEM)?, size })
    }

    fn copy(&self, offset: usize, out: &mut [u8]) -> Result {
        let end = offset.checked_add(out.len()).ok_or(EOVERFLOW)?;
        if (offset | out.len()) & 3 != 0 || end > self.size {
            return Err(EINVAL);
        }
        // SAFETY: bounds checked above against the mapping size.
        let base = unsafe { (self.pointer.as_ptr() as *const u8).add(offset) };
        invalidate_dcache_range(base, out.len());
        for (index, chunk) in out.chunks_exact_mut(4).enumerate() {
            // SAFETY: 4-byte aligned, inside the mapping.
            let word = unsafe { core::ptr::read_volatile(base.add(index * 4) as *const u32) };
            chunk.copy_from_slice(&word.to_le_bytes());
        }
        Ok(())
    }
}

impl Drop for PhysMap {
    fn drop(&mut self) {
        // SAFETY: returned by memremap and not yet unmapped.
        unsafe { bindings::memunmap(self.pointer.as_ptr()) };
    }
}

// ---------------------------------------------------------------------------
// RTKit
// ---------------------------------------------------------------------------

/// Per-role callback state. Callbacks only publish atomics and log.
struct G18RoleState {
    dev: ARef<AsahiDrmDevice>,
    role: G18Role,
    /// The role's `__DATA` reservation (physical base, size).
    data: (u64, u64),
    /// The role's `__DATA_SHARED_RO` reservation, (0, 0) when absent.
    shared_ro: (u64, u64),
    acked: AtomicBool,
    crashed: AtomicBool,
    crash_dumped: AtomicBool,
    messages: AtomicU32,
}

impl G18RoleState {
    fn crashed(&self) -> bool {
        self.crashed.load(Ordering::Acquire)
    }

    /// Scan the role's `__DATA` for a `CLHE` crashlog and dump it. Firmware
    /// with a firmware-owned crash buffer (the G17P RTKit mode) hands the
    /// host no bytes; on M5 that buffer is `__DATA + 0xa0000`. The G18G
    /// offset is unknown, so scan every 4 KiB. Runs at most once per role.
    fn dump_data_crashlog(&self, reason: &str) {
        if self.crash_dumped.swap(true, Ordering::AcqRel) {
            return;
        }
        let name = self.role.name();
        let mut found = self.scan_crashlog("__DATA", self.data, reason);
        if self.shared_ro.1 != 0 {
            found += self.scan_crashlog("__DATA_SHARED_RO", self.shared_ro, reason);
        }
        if found == 0 {
            dev_err!(self.dev.as_ref(), "G18_CRASHLOG {} no CLHE header in __DATA or __DATA_SHARED_RO\n", name);
        }
    }

    /// Scan one reservation every CRASHLOG_SCAN_STEP for a `CLHE` crashlog
    /// and dump what it finds. Read-only mapping. Returns the count found.
    fn scan_crashlog(&self, label: &str, region: (u64, u64), reason: &str) -> u32 {
        let dev: &kernel::device::Device = self.dev.as_ref().as_ref();
        let name = self.role.name();
        let (base, size) = region;
        let Ok(size) = usize::try_from(size) else {
            return 0;
        };
        dev_err!(dev, "G18_CRASHLOG {} scanning {} {:#x}+{:#x} ({})\n", name, label, base, size, reason);
        let map = match PhysMap::new(base, size) {
            Ok(map) => map,
            Err(error) => {
                dev_err!(dev, "G18_CRASHLOG {} {} map failed ({:?})\n", name, label, error);
                return 0;
            }
        };
        let mut found = 0u32;
        let mut offset = 0usize;
        while offset + 16 <= size {
            let mut head = [0u8; 16];
            if map.copy(offset, &mut head).is_ok() && le_u32(&head, 0) == Some(CRASHLOG_HEADER) {
                let declared = le_u32(&head, 8).unwrap_or(0) as usize;
                if declared < 0x20 || declared > size - offset {
                    dev_err!(dev, "G18_CRASHLOG {} CLHE at {}+{:#x} declares {:#x} bytes; skipped\n",
                        name, label, offset, declared);
                } else {
                    let length = (declared.min(CRASHLOG_MAX_BYTES) + 3) & !3;
                    let length = length.min(size - offset);
                    match KVVec::from_elem(0u8, length, GFP_KERNEL) {
                        Ok(mut bytes) => {
                            if map.copy(offset, &mut bytes).is_ok() {
                                dev_err!(dev, "G18_CRASHLOG {} found at {}+{:#x} (PA {:#x})\n",
                                    name, label, offset, base + offset as u64);
                                dump_crashlog(dev, name, label, &bytes);
                                found += 1;
                            }
                        }
                        Err(_) => dev_err!(dev, "G18_CRASHLOG {} no memory for {:#x} bytes\n", name, length),
                    }
                }
            }
            offset += CRASHLOG_SCAN_STEP;
        }
        found
    }
}

/// No host-backed RTKit buffers: exactly the M5 transport configuration
/// (`G17PNoRtkitBuffer`). Host-allocated requests use the RTKit core's
/// coherent-DMA fallback; the firmware-owned crash buffer is echoed.
struct G18NoRtkitBuffer;

impl rtkit::Buffer for G18NoRtkitBuffer {
    fn iova(&self) -> Result<usize> {
        Err(ENOTSUPP)
    }

    fn buf(&mut self) -> Result<IoSysMap<'_, Region>> {
        Err(ENOTSUPP)
    }
}

struct G18RtkitOps;

#[vtable]
impl rtkit::Operations for G18RtkitOps {
    type Data = Arc<G18RoleState>;
    type Buffer = G18NoRtkitBuffer;

    fn recv_message_early(
        data: <Self::Data as kernel::types::ForeignOwnable>::Borrowed<'_>,
        endpoint: u8,
        message: u64,
    ) -> bool {
        if endpoint == EP_GFX_MESSAGES && message == INITDATA_ACK {
            data.acked.store(true, Ordering::Release);
            true
        } else {
            false
        }
    }

    fn recv_message(
        data: <Self::Data as kernel::types::ForeignOwnable>::Borrowed<'_>,
        endpoint: u8,
        message: u64,
    ) {
        let ordinal = data.messages.fetch_add(1, Ordering::Relaxed);
        if ordinal < MESSAGE_LOG_BUDGET || ordinal.is_power_of_two() {
            dev_info!(
                data.dev.as_ref(),
                "G18_MSG role={} ordinal={} ep={:#x} type={:#06x} payload={:#018x}\n",
                data.role.name(),
                ordinal,
                endpoint,
                message >> 48,
                message
            );
        }
    }

    fn crashed(
        data: <Self::Data as kernel::types::ForeignOwnable>::Borrowed<'_>,
        crashlog: Option<&[u8]>,
    ) {
        data.crashed.store(true, Ordering::Release);
        dev_err!(
            data.dev.as_ref(),
            "G18 firmware crashed role={} host-crashlog={}\n",
            data.role.name(),
            crashlog.map_or(0, |bytes| bytes.len())
        );
        match crashlog {
            Some(bytes) if !bytes.is_empty() => {
                data.crash_dumped.store(true, Ordering::Release);
                dump_crashlog(data.dev.as_ref().as_ref(), data.role.name(), "rtkit", bytes);
            }
            // Firmware-owned buffer: the RTKit core logged its DVA.
            _ => data.dump_data_crashlog("rtkit crash callback, firmware-owned buffer"),
        }
    }
}

// ---------------------------------------------------------------------------
// Session
// ---------------------------------------------------------------------------

struct G18RoleSlot {
    role: G18Role,
    provider: G18Provider,
    rtkit: rtkit::RtKit<G18RtkitOps>,
    started: bool,
}

struct G18Session {
    roles: [G18RoleSlot; 2],
    states: [Arc<G18RoleState>; 2],
    memory: Option<KBox<G18FirmwareMemory>>,
    queue_probe: Option<G18QueueProbe>,
    render_job: Option<KBox<crate::g18::runtime::graphics_job::G18GraphicsJob>>,
    application_job: Option<KBox<crate::g18::runtime::graphics_job::application::ApplicationJob>>,
    uat: Arc<mmu::Uat>,
    _registers: regs::Resources,
    dev: ARef<AsahiDrmDevice>,
    stage: u32,
    /// InitData profile of the firmware build the loader admitted.
    profile: crate::g18::fw::initdata::RootProfile,
    /// A TVB grow request of the running job was answered "not grown".
    /// The tiler then stays stalled, so wait_job() ends the render with a
    /// recovery at once instead of waiting for its watchdog.
    growth_refused: bool,
}

fn cntvct() -> u64 {
    let value: u64;
    // SAFETY: read-only architectural counter.
    unsafe {
        core::arch::asm!("mrs {value}, cntvct_el0", value = out(reg) value,
            options(nomem, nostack, preserves_flags));
    }
    value
}

impl G18Session {
    fn device(&self) -> &kernel::device::Device {
        self.dev.as_ref().as_ref()
    }

    fn any_crashed(&self) -> bool {
        self.states.iter().any(|state| state.crashed())
    }

    /// Drain the report rings once the graph exists. Returns whether a
    /// recovery (type 4) event was seen. Answers TVB grow requests.
    fn drain(&mut self) -> Result<bool> {
        let Some(memory) = self.memory.as_mut() else { return Ok(false) };
        let recovery = memory.drain_reports(&self.dev)?;
        while let Some(request) = self.memory.as_mut().ok_or(EIO)?.take_tvb_grow() {
            self.answer_tvb_grow(request)?;
        }
        Ok(recovery)
    }

    /// Grow the PB that a firmware TVB grow request names and reply, so the
    /// waiting tiler continues; "not grown" when that fails or the PB is at
    /// its limit. A request from before the current recovery epoch is
    /// stale and gets no reply, as on M5.
    fn answer_tvb_grow(&mut self, request: G18TvbGrowRequest) -> Result {
        let epoch = self.memory.as_mut().ok_or(EIO)?.recovery_word()?.epoch;
        if request.halt_count != epoch {
            dev_warn!(self.device(), "G18_TVB_GROW stale {:?} epoch={}\n", request, epoch);
            return Ok(());
        }
        let grown = match self.application_job.as_mut() {
            Some(job) => job.grow_parameter_buffer(&self.dev, request.buffer_slot, request.vm_slot),
            None => Err(ENOENT),
        };
        let grew = match grown {
            Ok(0) => {
                dev_warn!(self.device(), "G18_TVB_GROW {:?} not grown: PB at its limit\n", request);
                false
            }
            Ok(_) => true,
            Err(error) => {
                dev_warn!(self.device(), "G18_TVB_GROW {:?} not grown: {:?}\n", request, error);
                false
            }
        };
        self.growth_refused |= !grew;
        self.memory.as_mut().ok_or(EIO)?.reply_tvb_grow(request, grew)?;
        Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit)
            .send_message(EP_GFX_INTERRUPTS, CONTROL_NOTIFY)?;
        Ok(())
    }

    /// Fault check used by every wait loop.
    fn check_faults(&mut self, label: &str) -> Result {
        // No host SGX MMIO here: firmware power-gates SGX at will, and a
        // gated read is a synchronous external abort (stage34 oops at
        // SGX+0xd8c0 during root publication). A GMMU fault reaches us as a
        // firmware recovery event. Read retained DRAM for diagnosis; a
        // userspace read of power-gated SGX registers is unsafe as well.
        if self.drain()? {
            dev_err!(self.device(), "G18 {}: firmware posted a recovery event\n", label);
            return Err(EIO);
        }
        if self.any_crashed() {
            dev_err!(self.device(), "G18 {}: firmware crashed\n", label);
            return Err(EIO);
        }
        Ok(())
    }

    fn stop_cpus(&mut self) -> Result {
        let mut failure = None;
        for role in G18Role::STOP_ORDER {
            let slot = &mut self.roles[role.index()];
            if slot.started {
                match slot.provider.stop_cpu() {
                    Ok(()) => slot.started = false,
                    Err(error) => failure = Some(error),
                }
            }
            log_cpu_state(self.dev.as_ref().as_ref(), role, "after-stop", &self.roles[role.index()].provider);
        }
        failure.map_or(Ok(()), Err)
    }

    /// Dump whatever crash evidence exists after a failed stage.
    fn dump_fault_evidence(&mut self, reason: &str) {
        for _ in 0..4 {
            if self.drain().is_err() {
                break;
            }
        }
        if let Some(memory) = self.memory.as_mut() {
            match memory.control_counters() {
                Ok(counters) => dev_err!(self.dev.as_ref(), "G18 fault: control counters {:?}\n", counters),
                Err(error) => dev_err!(self.dev.as_ref(), "G18 fault: counters unreadable ({:?})\n", error),
            }
            memory.log_summary(&self.dev);
            dev_err!(self.dev.as_ref(), "G18 fault: report counts {:?}\n", memory.report_counts());
        }
        for state in self.states.iter() {
            state.dump_data_crashlog(reason);
        }
    }

    fn run(&mut self) -> Result {
        // Defense in depth for any future caller that bypasses driver probe.
        if self.stage == STAGE_RENDER {
            crate::g18::boot::render_stage::admit(self.device(),
                crate::g18::params::g18_render_variant.value(),
                crate::g18::params::g18_render_profile.value())?;
        }
        self.boot_rtkit()?;
        if self.stage == STAGE_RTKIT {
            self.syslog_window()?;
            return Ok(());
        }
        self.build_initdata()?;
        self.publish_roots()?;
        if self.stage == STAGE_INITDATA {
            return Ok(());
        }
        self.retire_openings()?;
        self.wake_idle_policy()?;
        if matches!(self.stage, STAGE_QUEUE_REGISTER | STAGE_SCHEDULER_BIND | STAGE_POOL_BIND | STAGE_COMPUTE_ADD) {
            self.register_queue()?;
        }
        if matches!(self.stage, STAGE_SCHEDULER_BIND | STAGE_POOL_BIND | STAGE_COMPUTE_ADD) {
            self.bind_scheduler()?;
        }
        if self.stage == STAGE_POOL_BIND {
            let binding = self.memory.as_mut().ok_or(EIO)?.prepare_uma_pool_binding()?;
            self.bind_pool(binding)?;
        }
        if self.stage == STAGE_COMPUTE_ADD {
            let (command, pool) = self.memory.as_mut().ok_or(EIO)?.prepare_addition(&self.dev)?;
            self.bind_pool(pool)?;
            self.submit_addition(command, pool)?;
        }
        if self.stage == STAGE_RENDER { self.submit_render()?; }
        if self.stage == STAGE_DRM {return Ok(());}
        self.heartbeat()
    }

    /// Stage 2: start each ASC CPU (GFX1, then GFX) and run RTKit
    /// HELLO/EPMAP. The RTKit core starts the crashlog endpoint before
    /// acknowledging the EPMAP that advertises it (firmware-owned crash
    /// buffer, as G17P/M5), then syslog and the other system endpoints.
    fn boot_rtkit(&mut self) -> Result {
        for role in G18Role::START_ORDER {
            let dev = self.dev.clone();
            let slot = &mut self.roles[role.index()];
            log_cpu_state(dev.as_ref().as_ref(), role, "before-start", &slot.provider);
            // The provider re-reads the role's PSD block and refuses an
            // unpowered block; it refuses without the CPU-control approval.
            let (control, _) = slot.provider.cpu_state()?;
            if control & ASCWRAP_V8_CPU_RUN == 0 {
                slot.provider.start_cpu()?;
            } else {
                dev_info!(dev.as_ref(), "G18_ASC {} adopted: RUN already set, start skipped\n", role.name());
            }
            slot.started = true;
            log_cpu_state(dev.as_ref().as_ref(), role, "after-start", &slot.provider);
            let started = Instant::<Monotonic>::now();
            let booted = Pin::new(&mut slot.rtkit).boot();
            let elapsed = started.elapsed().as_micros_ceil();
            let mut endpoints = 0u64;
            for endpoint in 0..64u8 {
                if Pin::new(&mut slot.rtkit).has_endpoint(endpoint) {
                    endpoints |= 1 << endpoint;
                }
            }
            dev_info!(
                dev.as_ref(),
                "G18_RTKIT role={} boot={:?} after {} us; EPMAP[0..64]={:#018x} (crashlog={} syslog={} ep20={} ep21={} ep23={})\n",
                role.name(),
                booted,
                elapsed,
                endpoints,
                (endpoints >> 1) & 1,
                (endpoints >> 2) & 1,
                (endpoints >> EP_GFX_MESSAGES) & 1,
                (endpoints >> EP_GFX_INTERRUPTS) & 1,
                (endpoints >> 0x23) & 1
            );
            booted?;
            if self.states[role.index()].crashed() {
                return Err(EIO);
            }
        }
        Ok(())
    }

    /// Stage 2 only: give the syslog endpoint time to print its banner
    /// (expected 'G18 Firmware initializing', 'FW Build version: b0'; the
    /// RTKit core logs each entry as "RTKit: syslog message").
    fn syslog_window(&mut self) -> Result {
        dev_info!(self.device(), "G18_STAGE2 syslog window open for {} ms\n", SYSLOG_WINDOW_MS);
        for _ in 0..SYSLOG_WINDOW_MS / 10 {
            if self.any_crashed() {
                return Err(EIO);
            }
            fsleep(Delta::from_millis(10));
        }
        for role in G18Role::ALL {
            log_cpu_state(self.dev.as_ref().as_ref(), role, "after-syslog-window", &self.roles[role.index()].provider);
            dev_info!(
                self.device(),
                "G18_STAGE2 role={} non-system messages={}\n",
                role.name(),
                self.states[role.index()].messages.load(Ordering::Relaxed)
            );
        }
        dev_info!(self.device(), "G18_STAGE2 syslog window closed\n");
        Ok(())
    }

    /// Stage 3: map the init-data graph under the firmware-published roots
    /// (same order as M5; no SGX pre-init pokes are issued, none are derived
    /// for G18G yet).
    fn build_initdata(&mut self) -> Result {
        let roots = self.uat.prepare_t8140_firmware_high_roots()?;
        let memory = G18FirmwareMemory::new_with_uat(&self.dev, self.uat.clone(), cntvct(),
            matches!(self.stage, STAGE_RENDER | STAGE_DRM), self.profile)?;
        self.memory = Some(memory);
        if matches!(self.stage, STAGE_RENDER | STAGE_DRM) {
            self.render_job = Some(crate::g18::boot::render_stage::prepare(&self.dev, &self.uat)?);
            self.memory.as_mut().ok_or(EIO)?.prepare_render(&self.dev,
                self.render_job.as_mut().ok_or(EIO)?)?;
        }
        if matches!(self.stage, STAGE_QUEUE_REGISTER | STAGE_SCHEDULER_BIND | STAGE_POOL_BIND | STAGE_COMPUTE_ADD)
            || (self.stage == STAGE_DRM && crate::g18::params::g18_admit_compute.value() == 1) {
            self.queue_probe = Some(G18QueueProbe::new(&self.dev, &self.uat,
                matches!(self.stage, STAGE_SCHEDULER_BIND | STAGE_POOL_BIND | STAGE_COMPUTE_ADD | STAGE_DRM))?);
        }
        self.uat.confirm_t8140_primary_high_root(roots)?;
        self.uat.mirror_t8140_secondary_high_root()?;
        let memory = self.memory.as_mut().ok_or(EIO)?;
        if matches!(self.stage, STAGE_COMPUTE_ADD | STAGE_RENDER | STAGE_DRM) {
            memory.use_firmware_queue_programming(&self.dev)?;
        }
        memory.log_summary(&self.dev);
        dev_info!(self.dev.as_ref(), "G18_INIT status links before publish {:x?}; counters {:?}\n",
            memory.status_links()?, memory.control_counters()?);
        Ok(())
    }

    /// Stage 3: publish GFX's root, then GFX1's, each on EP 0x20 with the
    /// 0x81 doorbell, and wait for that role's 0x0009 ACK. Native 26A428
    /// captures 051/053/073 all publish in this order. The ASC CPU/RTKit
    /// startup order is separate and remains GFX1 first.
    fn publish_roots(&mut self) -> Result {
        for role in [G18Role::Gfx, G18Role::Gfx1] {
            let index = role.index();
            let address = self.memory.as_ref().ok_or(EIO)?.root(role == G18Role::Gfx1);
            let message = g17_rtkit::encode_initdata_doorbell(address).map_err(|_| EINVAL)?;
            {
                let slot = &mut self.roles[index];
                for endpoint in [EP_GFX_MESSAGES, EP_GFX_INTERRUPTS] {
                    if !Pin::new(&mut slot.rtkit).has_endpoint(endpoint) {
                        dev_err!(self.dev.as_ref(), "G18_INIT role={} did not advertise EP {:#x}\n",
                            role.name(), endpoint);
                        return Err(ENODEV);
                    }
                }
                Pin::new(&mut slot.rtkit).start_endpoint(EP_GFX_MESSAGES)?;
                Pin::new(&mut slot.rtkit).start_endpoint(EP_GFX_INTERRUPTS)?;
                Pin::new(&mut slot.rtkit).send_message(EP_GFX_MESSAGES, message)?;
            }
            dev_info!(self.device(), "G18_INIT role={} published root={:#x} doorbell={:#018x}\n",
                role.name(), address, message);
            let started = Instant::<Monotonic>::now();
            let mut acked = false;
            for _ in 0..ACK_POLLS {
                self.check_faults("initdata ACK wait")?;
                if self.states[index].acked.load(Ordering::Acquire) {
                    acked = true;
                    break;
                }
                fsleep(Delta::from_millis(10));
            }
            if !acked {
                dev_err!(self.device(), "G18_INIT role={} no 0x0009 ACK within {} ms\n",
                    role.name(), ACK_POLLS * 10);
                return Err(ETIMEDOUT);
            }
            dev_info!(self.device(), "G18_INIT role={} initdata-ack after {} us\n",
                role.name(), started.elapsed().as_micros_ceil());
        }
        let memory = self.memory.as_mut().ok_or(EIO)?;
        dev_info!(self.dev.as_ref(), "G18_INIT both roles ACKed; status links {:x?}; counters {:?}; completion cursors {:?}\n",
            memory.status_links()?, memory.control_counters()?, memory.completion_cursors()?);
        self.check_faults("after both ACKs")
    }

    /// Stage 4: 0x89 power-transition-end to GFX, then GFX1, and wait for
    /// both opening records (0x15 / 0x2a) to retire.
    fn retire_openings(&mut self) -> Result {
        let before = self.memory.as_mut().ok_or(EIO)?.control_counters()?;
        dev_info!(self.device(), "G18_OPEN before power-transition-end {:?}\n", before);
        for role in G18Role::STOP_ORDER {
            Pin::new(&mut self.roles[role.index()].rtkit)
                .send_message(EP_GFX_INTERRUPTS, g17_rtkit::encode_power_transition_end())?;
        }
        let mut counters = before;
        for _ in 0..CONTROL_POLLS {
            self.check_faults("opening retire")?;
            counters = self.memory.as_mut().ok_or(EIO)?.control_counters()?;
            if counters == [[1; 3]; 2] {
                dev_info!(self.device(), "G18_OPEN both control openings retired {:?}\n", counters);
                return Ok(());
            }
            fsleep(Delta::from_millis(1));
        }
        dev_err!(self.device(), "G18_OPEN openings did not retire {:?}\n", counters);
        Err(ETIMEDOUT)
    }

    /// Stage 4: publish IdlePowerOff(0) (opcode 0x0a) and ring EP 0x21 with
    /// the idle-policy wake, then wait for GFX's control ring to retire it.
    fn wake_idle_policy(&mut self) -> Result {
        self.memory.as_mut().ok_or(EIO)?.publish_idle_wake()?;
        Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit)
            .send_message(EP_GFX_INTERRUPTS, IDLE_POLICY_WAKE)?;
        let mut counters = [[0; 3]; 2];
        for _ in 0..CONTROL_POLLS {
            self.check_faults("idle-policy wake")?;
            counters = self.memory.as_mut().ok_or(EIO)?.control_counters()?;
            if counters == [[2; 3], [1; 3]] {
                dev_info!(self.device(), "G18_POWER idle policy retired {:?}\n", counters);
                return Ok(());
            }
            fsleep(Delta::from_millis(1));
        }
        dev_err!(self.device(), "G18_POWER wake did not retire {:?}\n", counters);
        Err(ETIMEDOUT)
    }

    fn queue_probe_status(&mut self) -> Result<([u32; 3], [u32; 3], [u64; 5])> {
        let outer = self.memory.as_mut().ok_or(EIO)?.compute_outer_counters()?;
        let probe = self.queue_probe.as_mut().ok_or(EIO)?;
        Ok((outer, probe.counters()?, probe.slot()))
    }

    /// Register one empty queue. No scheduler binding, event or tag14 kick.
    fn register_queue(&mut self) -> Result {
        let before = self.queue_probe_status()?;
        dev_info!(self.device(), "G18_QUEUE before tag15-only {:?}\n", before);
        if before != ([0; 3], [0, 0, 1], [0; 5]) { return Err(EBUSY); }
        let address = self.queue_probe.as_ref().ok_or(EIO)?.queue_address();
        self.memory.as_mut().ok_or(EIO)?.publish_queue_probe(address, QUEUE_ID, 1)?;
        Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit)
            .send_message(EP_GFX_INTERRUPTS, 0x0083_0000_0000_000a)?;
        for _ in 0..CONTROL_POLLS {
            self.check_faults("queue registration")?;
            let (outer, inner, slot) = self.queue_probe_status()?;
            if outer == [1; 3] && inner == [1; 3] && G18QueueProbe::installed(slot) {
                dev_info!(self.device(),
                    "G18_QUEUE installed qid11 outer={:?} inner={:?} slot={:x?}; no kicks\n",
                    outer, inner, slot);
                return Ok(());
            }
            fsleep(Delta::from_millis(1));
        }
        let status = self.queue_probe_status()?;
        dev_err!(self.device(), "G18_QUEUE registration timeout {:?}\n", status);
        Err(ETIMEDOUT)
    }

    fn bind_scheduler(&mut self) -> Result {
        let probe = self.queue_probe.as_mut().ok_or(EIO)?;
        let before = probe.scheduler_status()?;
        dev_info!(self.dev.as_ref(), "G18_SCHED before bind {:x?}\n", before);
        probe.prepare_scheduler_bind()?;
        let address = probe.queue_address();
        self.memory.as_mut().ok_or(EIO)?.publish_queue_probe(address, QUEUE_ID, 2)?;
        Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit)
            .send_message(EP_GFX_INTERRUPTS, 0x0083_0000_0000_000a)?;
        for _ in 0..CONTROL_POLLS {
            self.check_faults("scheduler bind")?;
            let (outer, inner, slot) = self.queue_probe_status()?;
            let status = self.queue_probe.as_mut().ok_or(EIO)?.scheduler_status()?;
            if outer == [2; 3] && inner == [2; 3] && G18QueueProbe::installed(slot)
                && G18QueueProbe::scheduler_bound(status) {
                dev_info!(self.device(),
                    "G18_SCHED bound qid11 outer={:?} inner={:?} status={:x?}; no kicks\n",
                    outer, inner, status);
                return Ok(());
            }
            fsleep(Delta::from_millis(1));
        }
        let queues = self.queue_probe_status()?;
        let status = self.queue_probe.as_mut().ok_or(EIO)?.scheduler_status()?;
        dev_err!(self.device(), "G18_SCHED bind timeout queues={:?} status={:x?}\n", queues, status);
        Err(ETIMEDOUT)
    }

    fn bind_pool(&mut self, binding: G18UmaPoolBinding) -> Result {
        let probe = self.queue_probe.as_mut().ok_or(EIO)?;
        let before = probe.pool_status();
        dev_info!(self.dev.as_ref(), "G18_POOL before bind {:x?}; state={:#x} generation={} slot={}\n",
            before, binding.state, binding.generation, binding.slot);
        probe.prepare_pool_bind(binding)?;
        let address = probe.queue_address();
        self.memory.as_mut().ok_or(EIO)?.publish_queue_probe(address, QUEUE_ID, 3)?;
        Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit)
            .send_message(EP_GFX_INTERRUPTS, 0x0083_0000_0000_000a)?;
        for _ in 0..CONTROL_POLLS {
            self.check_faults("UMA pool bind")?;
            let (outer, inner, slot) = self.queue_probe_status()?;
            let status = self.queue_probe.as_ref().ok_or(EIO)?.pool_status();
            if outer == [3; 3] && inner == [3; 3] && G18QueueProbe::installed(slot)
                && G18QueueProbe::pool_bound(status, binding) {
                dev_info!(self.device(),
                    "G18_POOL bound qid11 outer={:?} inner={:?} status={:x?}; no kicks or growth\n",
                    outer, inner, status);
                return Ok(());
            }
            fsleep(Delta::from_millis(1));
        }
        let queues = self.queue_probe_status()?;
        let status = self.queue_probe.as_ref().ok_or(EIO)?.pool_status();
        dev_err!(self.device(), "G18_POOL bind timeout queues={:?} status={:x?}\n", queues, status);
        Err(ETIMEDOUT)
    }

    /// Submit exactly one addition; a retired setup or changed stamp alone is
    /// insufficient. Preserve the whole firmware owner after every outcome.
    fn submit_addition(&mut self, command: G18PreparedComputeCommand,
                       pool: G18UmaPoolBinding) -> Result {
        let memory = self.memory.as_mut().ok_or(EIO)?;
        let output = memory.addition_output()?;
        let completion = memory.compute().completion_snapshot()?;
        let scheduler = memory.scheduler().snapshot(0)?;
        if output.checked != 4096 || output.untouched != 4096 || output.correct != 0
            || completion.stamps != [0; 2] || memory.completion_masks() != [0; 2]
            || completion.timestamps != [0; 2] || scheduler.submitted != 0
            || scheduler.completed != 0 || scheduler.linked != 0 {
            return Err(EBUSY);
        }
        let roots = memory.compute().publish_context()?;
        let probe = self.queue_probe.as_mut().ok_or(EIO)?;
        probe.prepare_compute_job(command, pool)?;
        let address = probe.queue_address();
        // From this counter onward, even a failed publication retains all
        // memory. Firmware may already observe the host-owned counter.
        let submitted = self.memory.as_mut().ok_or(EIO)?.scheduler().publish_kick(0)?;
        self.queue_probe.as_mut().ok_or(EIO)?.publish_compute_job()?;
        self.memory.as_mut().ok_or(EIO)?.publish_compute_queue(address, QUEUE_ID)?;
        Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit)
            .send_message(EP_GFX_INTERRUPTS, 0x0083_0000_0000_000a)?;
        dev_info!(self.device(),
            "G18_COMPUTE submitted qid11 context={} roots={:x?} count={} expected_stamp={:#x}\n",
            command.context_id, roots, submitted, command.expected_stamp);
        // Bounded first-use run. Polling observes completion; it sends no
        // additional commands and never retries a GPU job in this boot.
        for step in 0..10_000 {
            self.check_faults("compute addition")?;
            let (outer, inner, _) = self.queue_probe_status()?;
            let memory = self.memory.as_mut().ok_or(EIO)?;
            let completion = memory.compute().completion_snapshot()?;
            let scheduler = memory.scheduler().snapshot(0)?;
            let masks = memory.completion_masks();
            if outer == [4; 3] && inner == [6; 3]
                // Original b000 19234/19268 writes through the first qword.
                // The second qword is carried into trace 0xe as an identity,
                // not written by this completion path (stage29 hardware).
                && completion.stamps[0] == command.expected_stamp
                && masks[0] & (1u64 << QUEUE_ID) != 0
                && completion.timestamps[0] != 0
                && completion.timestamps[1] >= completion.timestamps[0]
                && scheduler.submitted == 1 && scheduler.completed == 1 && scheduler.linked == 0 {
                let output = memory.addition_output()?;
                dev_info!(self.dev.as_ref(),
                    "G18_COMPUTE completion outer={:?} inner={:?} stamps={:x?} times={:x?} event_masks={:x?} scheduler={:?} output={:?}\n",
                    outer, inner, completion.stamps, completion.timestamps, masks, scheduler, output);
                if crate::g18::params::g18_compute_variant.value() == 2 {
                    let pixels = memory.triangle_output()?;
                    dev_info!(self.dev.as_ref(), "G18_COMPUTE_TRIANGLE output={:?}\n", pixels);
                    if !pixels.passed() { return Err(EIO); }
                    self.check_faults("compute triangle output")?;
                    dev_info!(self.device(), "G18_COMPUTE_TRIANGLE_PASS: 4096 GPU pixels match after independent completion; compute rasterization, no TA/3D or Mesa\n");
                    return Ok(());
                }
                // g18_compute_variant=1 dispatches nothing: every output must stay poisoned.
                let empty = crate::g18::params::g18_compute_variant.value() == 1;
                let expected = if empty { (0, 4096) } else { (4096, 0) };
                if output.checked != 4096 || (output.correct, output.untouched) != expected {
                    if empty {
                        dev_err!(self.device(), "G18_COMPUTE_EMPTY_STREAM_FAIL: completed without a dispatch, but output changed {:?}\n", output);
                    }
                    return Err(EIO);
                }
                self.check_faults("compute output")?;
                if empty {
                    dev_info!(self.device(), "G18_COMPUTE_EMPTY_STREAM_PASS: completion observed with no dispatch; all 4096 outputs still poisoned\n");
                } else {
                    dev_info!(self.device(), "G18_COMPUTE_ADD_PASS: all 4096 outputs match after independent completion\n");
                }
                return Ok(());
            }
            if step % 1000 == 999 {
                dev_info!(self.dev.as_ref(),
                    "G18_COMPUTE wait={}ms outer={:?} inner={:?} stamps={:x?} scheduler={:?}\n",
                    step + 1, outer, inner, completion.stamps, scheduler);
            }
            fsleep(Delta::from_millis(1));
        }
        dev_err!(self.device(), "G18_COMPUTE timeout; owners retained, no resubmission\n");
        Err(ETIMEDOUT)
    }

    /// A single bounded H1 pass; all owners already live in the retained session.
    fn submit_render(&mut self) -> Result {
        let pass = self.render_job.as_ref().ok_or(EIO)?.pass()?;
        if self.memory.as_ref().ok_or(EIO)?.completion_masks()!=[0;2] {return Err(EBUSY);}
        for phase in 1..=3 {
            self.check_faults("render binding")?;
            let queues = self.render_job.as_mut().ok_or(EIO)?.queues();
            let records = match phase {
                1 => queues.install_records(&pass)?,
                2 => queues.scheduler_records(&pass)?,
                _ => queues.pool_records(&pass)?,
            };
            self.memory.as_mut().ok_or(EIO)?.publish_outer_records(&records)?;
            Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit)
                .send_message(EP_GFX_INTERRUPTS, 0x0083_0000_0000_000a)?;
            let mut passed = false;
            for _ in 0..CONTROL_POLLS {
                self.check_faults("render binding")?;
                let outer=self.memory.as_mut().ok_or(EIO)?.compute_outer_counters()?;
                if outer==[phase*2;3] && self.render_job.as_mut().ok_or(EIO)?.queues().stage_gate(phase,&pass)? {
                    dev_info!(self.device(), "G18_RENDER_BIND_PASS phase={} outer={:?} stages=0,1\n",phase,outer);
                    passed=true;break;
                }
                fsleep(Delta::from_millis(1));
            }
            if !passed {
                let outer=self.memory.as_mut().ok_or(EIO)?.compute_outer_counters()?;
                let job=self.render_job.as_mut().ok_or(EIO)?;
                dev_err!(self.dev.as_ref(), "G18_RENDER_BIND_FAIL phase={} outer={:?} inner={:?} firmware={:x?} pool_pb={:x?}\n",
                    phase,outer,job.queues().counters()?,job.queues().firmware_status()?,job.queues().pool_and_pb_status(0,0)?);
                return Err(ETIMEDOUT);
            }
        }
        let job=self.render_job.as_mut().ok_or(EIO)?;
        if job.parameter_buffer().count_submitted_render()?!=1 {return Err(EIO);}
        self.memory.as_mut().ok_or(EIO)?.audit_render(&self.dev,job)?;
        job.queues().prepare_pass(&pass)?;
        job.queues().audit_prepared_pass(&self.dev,&pass)?;
        self.check_faults("render pre-kick")?;
        let job=self.render_job.as_mut().ok_or(EIO)?;
        let roots=job.publish_h1()?;
        let records=job.queues().publish_pass()?;
        self.memory.as_mut().ok_or(EIO)?.publish_outer_records(&records)?;
        Pin::new(&mut self.roles[G18Role::Gfx.index()].rtkit)
            .send_message(EP_GFX_INTERRUPTS, 0x0083_0000_0000_000a)?;
        dev_info!(self.device(), "G18_RENDER_KICK context={} roots={:x?} order=3D,TA submitted=2 barrier=1 PB-generation=1\n",pass.context_id,roots);
        for step in 0..10_000 {
            self.check_faults("render completion")?;
            let memory=self.memory.as_mut().ok_or(EIO)?;
            let outer=memory.compute_outer_counters()?;let masks=memory.completion_masks();
            let job=self.render_job.as_mut().ok_or(EIO)?;
            let inner=job.queues().counters()?;let stamps=job.stamps()?;
            let scheduler=job.scheduler_snapshot()?;let list=job.queues().completion_list()?;
            if outer==[8;3] && inner==[[6;3];2] && stamps==[0x100;2]
                && scheduler.submitted==2 && scheduler.completed==2 && scheduler.linked==0
                && list==[0,job.queues().job_list()] && masks[0]&0x600==0x600 {
                let pixels=job.pixels()?;
                dev_info!(self.dev.as_ref(), "G18_RENDER_COMPLETION outer={:?} inner={:?} stamps={:x?} masks={:x?} scheduler={:?} list={:x?}\n",outer,inner,stamps,masks,scheduler,list);
                dev_info!(self.dev.as_ref(), "G18_RENDER_PIXELS sha256={:08x}{:08x}{:08x}{:08x}{:08x}{:08x}{:08x}{:08x} background={} poison={}\n",
                    pixels.0[0],pixels.0[1],pixels.0[2],pixels.0[3],pixels.0[4],pixels.0[5],pixels.0[6],pixels.0[7],pixels.1,pixels.2);
                self.check_faults("render pixels")?;
                dev_info!(self.device(), "G18_RENDER_H1_PASS variant={} completion=independent pixels=exact owners=retained\n",crate::g18::params::g18_render_variant.value());
                return Ok(());
            }
            if step%1000==999 {
                dev_info!(self.dev.as_ref(), "G18_RENDER_WAIT ms={} outer={:?} inner={:?} stamps={:x?} masks={:x?} scheduler={:?} list={:x?}\n",step+1,outer,inner,stamps,masks,scheduler,list);
            }
            fsleep(Delta::from_millis(1));
        }
        Err(ETIMEDOUT)
    }

    /// Stage 4..9: passive drain of both roles' event and trace rings. Sends
    /// nothing. This is the milestone-1 heartbeat.
    fn heartbeat(&mut self) -> Result {
        dev_info!(self.device(), "G18_HEARTBEAT passive drain for {} ms, no host messages\n", HEARTBEAT_MS);
        let steps = HEARTBEAT_MS / HEARTBEAT_STEP_MS;
        for step in 0..steps {
            self.check_faults("heartbeat")?;
            if step % 100 == 99 {
                let memory = self.memory.as_mut().ok_or(EIO)?;
                dev_info!(self.dev.as_ref(),
                    "G18_HEARTBEAT t={} ms reports [role][event,trace]={:?} counters={:?} cursors={:?}\n",
                    (step + 1) * HEARTBEAT_STEP_MS, memory.report_counts(),
                    memory.control_counters()?, memory.completion_cursors()?);
            }
            fsleep(Delta::from_millis(HEARTBEAT_STEP_MS as i64));
        }
        let counts = self.memory.as_ref().ok_or(EIO)?.report_counts();
        let alive = counts.iter().all(|role| role[1] != 0);
        dev_info!(self.device(), "G18_HEARTBEAT done: reports={:?}; trace advanced on both roles={}; no recovery record\n",
            counts, alive);
        Ok(())
    }
}

/// Holds a session that may have firmware running. Dropping it stops both
/// CPUs and then LEAKS every owner: the halt is not qualified, and the
/// firmware may still reference the UAT, the graph and the transports.
struct Retained(Option<KBox<G18Session>>);

impl Retained {
    fn stop(&mut self, label: &str) -> Result {
        let Some(session) = self.0.as_mut() else {
            return Ok(());
        };
        let stopped = session.stop_cpus();
        dev_info!(session.device(), "G18 {}: stop requests={:?}; owners retained until reboot\n",
            label, stopped);
        stopped
    }
}

impl Drop for Retained {
    fn drop(&mut self) {
        let _ = self.stop("session release");
        if let Some(session) = self.0.take() {
            core::mem::forget(session);
        }
    }
}

/// Stage-4 owner kept by the driver (`AsahiRuntime::G18`).
#[pin_data]
pub(crate) struct G18Owner {
    #[pin]
    session: Mutex<Retained>,
}

impl G18Runtime for G18Owner {
    fn shutdown(&self) {
        let _ = self.session.lock().stop("unbind");
    }
}

// ---------------------------------------------------------------------------
// Stage 1: read-only probe
// ---------------------------------------------------------------------------

fn stage_read_only(pdev: &platform::Device<Core<'_>>, probe: &T8152Probe) {
    let dev: &kernel::device::Device<Core<'_>> = pdev.as_ref();
    for sample in probe.psd.iter() {
        log_psd(pdev, "stage1 (probe sample)", sample);
    }
    match &probe.observed {
        Some(id) => dev_info!(dev, "G18_STAGE1 SGX identity (sgx PSD proven powered): {:?}\n", id),
        None => dev_info!(dev, "G18_STAGE1 SGX identity not read: sgx PSD not powered\n"),
    }
    dev_info!(dev, "G18_STAGE1 firmware receipt {}\n",
        if probe.firmware_profile_admitted { "admitted" } else { "absent" });

    for role in G18Role::ALL {
        let provider = match G18Provider::get(dev, role) {
            Ok(provider) => provider,
            Err(error) => {
                // G18Provider::get already explained an unbound provider.
                dev_info!(dev, "G18_STAGE1 {} mailbox provider unavailable ({:?}); wrapper and post-office reads skipped\n",
                    role.name(), error);
                continue;
            }
        };
        let lifecycle = match provider.validate(dev) {
            Ok(lifecycle) => lifecycle,
            Err(error) => {
                dev_err!(dev, "G18_STAGE1 {} provider rejected ({:?}); no reads\n", role.name(), error);
                continue;
            }
        };
        dev_info!(dev, "G18_STAGE1 {} provider wrapper={:x?} iop-vbar={:x?} post-office={:#x} missing={:#x} (bit0: CPU control not approved)\n",
            role.name(), lifecycle.wrapper, lifecycle.iop_vbar, role.postoffice_window(), lifecycle.missing);
        log_cpu_state(dev, role, "stage1", &provider);
        match provider.postoffice_status(pdev) {
            Ok((psd, Some(status))) => {
                log_psd(pdev, "stage1 post-office gate", &psd);
                dev_info!(dev, "G18_STAGE1 {} post-office status={:#x} (rx-occupancy={} tx-free={})\n",
                    role.name(), status, status & 0xf, (status >> 8) & 0xf);
            }
            Ok((psd, None)) => {
                log_psd(pdev, "stage1 post-office gate", &psd);
                dev_info!(dev, "G18_STAGE1 {} post-office status not read: role PSD not powered\n", role.name());
            }
            Err(error) => dev_err!(dev, "G18_STAGE1 {} post-office read failed ({:?})\n", role.name(), error),
        }
    }
    dev_info!(dev, "G18_STAGE1 done; nothing sent, no write issued; ENODEV\n");
}

// ---------------------------------------------------------------------------
// Stages 90..93: PSD requests (each REQUIRES USER APPROVAL)
// ---------------------------------------------------------------------------

/// Native PSD::getPsdAck order: poll ACK, then read STATE once. STATE
/// clears ACK on J873; reading it while waiting loses the acknowledgement.
fn wait_psd_state(pdev: &platform::Device<Core<'_>>, block: &PsdBlock, target: u32) -> Result<regs::T8152PsdSample> {
    let started = Instant::<Monotonic>::now();
    for poll in 0..PSD_POLLS {
        if let Some(sample) = block.acknowledged_sample()? {
            dev_info!(pdev.as_ref(), "G18_PSD service={} poll={} t={} us state={:#x} ack={:#x} (ACK read before STATE)\n",
                sample.service.service_id(), poll, started.elapsed().as_micros_ceil(),
                sample.state, sample.ack);
            if sample.power_state() != target {
                dev_err!(pdev.as_ref(), "G18_PSD service={} acknowledged unexpected state {}, wanted {}\n",
                    sample.service.service_id(), sample.power_state(), target);
                return Err(EIO);
            }
            dev_info!(pdev.as_ref(), "G18_PSD service={} reached state {} after {} us; ack={:#x}\n",
                sample.service.service_id(), target, started.elapsed().as_micros_ceil(), sample.ack);
            return Ok(sample);
        }
        fsleep(Delta::from_millis(1));
    }
    dev_err!(pdev.as_ref(), "G18_PSD service={} did not acknowledge state {} within {} ms\n",
        block.service.service_id(), target, PSD_POLLS);
    Err(ETIMEDOUT)
}

/// Claim one PSD request kind for this module instance.
fn claim_psd_request(dev: &kernel::device::Device, bit: u32) -> Result {
    if G18_FIRMWARE_ATTEMPTED.load(Ordering::Acquire) {
        dev_err!(dev, "G18_PSD refused: a firmware stage already ran this boot\n");
        return Err(EBUSY);
    }
    if G18_PSD_REQUESTS.fetch_or(bit, Ordering::AcqRel) & bit != 0 {
        dev_err!(dev, "G18_PSD refused: this request was already written by this module instance\n");
        return Err(EBUSY);
    }
    Ok(())
}

fn map_psd_blocks(pdev: &platform::Device<Core<'_>>) -> Result<KVec<PsdBlock>> {
    let mut blocks = KVec::new();
    for service in regs::T8152PsdService::ALL {
        blocks.push(PsdBlock::map(pdev, service)?, GFP_KERNEL)?;
    }
    Ok(blocks)
}

/// REQUIRES USER APPROVAL. Stages 90 (sgx), 91 (gfx-asc), 92 (gfx1-asc):
/// request the native active state (SGX 0, ASCs 2) on exactly one service.
///
/// Refuses unless that service reads cold idle (state 6); gfx-asc and
/// gfx1-asc additionally need sgx to read state 0 first. Native order is
/// 3, 82, 81. Polls ACK before STATE. On a timeout nothing is restored: the service is
/// left as requested and the power state is reported UNKNOWN, to be
/// restored by stage 93 under its own approval. The SGX ID is read only if
/// sgx then proves powered.
fn stage_psd_power_on(pdev: &platform::Device<Core<'_>>, probe: &T8152Probe, service: regs::T8152PsdService) -> Result {
    let dev: &kernel::device::Device<Core<'_>> = pdev.as_ref();
    dev_warn!(dev, "G18_PSD stage {}: PMGR2 PSD power-on of service {} only (REQUIRES USER APPROVAL)\n",
        probe.stage, service.service_id());
    let blocks = map_psd_blocks(pdev)?;
    let mut before = [regs::T8152PsdSample { service: regs::T8152PsdService::Sgx, state: 0, ack: 0 }; 3];
    for (slot, block) in before.iter_mut().zip(blocks.iter()) {
        *slot = block.sample()?;
        log_psd(pdev, "power-on before", slot);
    }
    let index = regs::T8152PsdService::ALL.iter().position(|s| *s == service).ok_or(EINVAL)?;
    let target = &before[index];
    let active_state = service.active_state();
    if target.powered() {
        dev_info!(dev, "G18_PSD service {} already reads state {} ({}); no request written\n",
            service.service_id(), active_state, target.verdict());
        return Ok(());
    }
    if target.power_state() != PSD_STATE_IDLE {
        dev_err!(dev, "G18_PSD refused: service {} must read cold idle state {} first\n",
            service.service_id(), PSD_STATE_IDLE);
        return Err(EBUSY);
    }
    if service != regs::T8152PsdService::Sgx && !before[0].powered() {
        dev_err!(dev, "G18_PSD refused: sgx (service 3) must read state 0 before service {} (run stage 90 first)\n",
            service.service_id());
        return Err(EBUSY);
    }
    claim_psd_request(dev, psd_request_bit(service))?;
    let block = &blocks[index];
    dev_warn!(dev, "G18_PSD write service={} request={} (REQUIRES USER APPROVAL)\n",
        service.service_id(), active_state);
    let result = block.request(active_state).and_then(|()| wait_psd_state(pdev, block, active_state));
    for block in blocks.iter() {
        if let Ok(sample) = block.sample() {
            log_psd(pdev, "power-on after", &sample);
        }
    }
    if let Err(error) = result {
        dev_err!(dev, "G18_PSD service {} power-on failed ({:?}); left as requested, GPU power state UNKNOWN; no automatic restore (stage 93 restores, REQUIRES USER APPROVAL)\n",
            service.service_id(), error);
        return Err(error);
    }
    if service == regs::T8152PsdService::Sgx {
        match regs::t8152_prove_sgx_powered(pdev) {
            Ok(powered) => {
                let id = probe.registers.get_gpu_id_t8152(&powered, probe.soc)?;
                dev_info!(dev, "G18_PSD sgx powered after acknowledged transition; SGX identity {:?}\n", id);
            }
            Err(error) => dev_err!(dev, "G18_PSD sgx reached state 0 but is not provably powered ({:?}); SGX not read\n", error),
        }
    }
    dev_info!(dev, "G18_PSD stage {} done; service {} left in state {}, no firmware touched; ENODEV\n",
        probe.stage, service.service_id(), active_state);
    Ok(())
}

/// REQUIRES USER APPROVAL. Stage 93: request idle (6) on every GPU service
/// that reads its active state, in reverse native order (81, 82, 3),
/// polling ACK before STATE. Refuses if a service is neither active nor
/// idle, or if a firmware stage ran this boot. Stops at the first
/// failure and reports the power state UNKNOWN.
fn stage_psd_restore_idle(pdev: &platform::Device<Core<'_>>) -> Result {
    let dev: &kernel::device::Device<Core<'_>> = pdev.as_ref();
    dev_warn!(dev, "G18_PSD stage 93: PMGR2 PSD idle restore (REQUIRES USER APPROVAL)\n");
    let blocks = map_psd_blocks(pdev)?;
    let mut samples = [regs::T8152PsdSample { service: regs::T8152PsdService::Sgx, state: 0, ack: 0 }; 3];
    for (slot, block) in samples.iter_mut().zip(blocks.iter()) {
        *slot = block.sample()?;
        log_psd(pdev, "restore before", slot);
    }
    if samples.iter().any(|sample| !sample.powered() && sample.power_state() != PSD_STATE_IDLE) {
        dev_err!(dev, "G18_PSD restore refused: a service is neither active nor idle\n");
        return Err(EBUSY);
    }
    if samples.iter().all(|sample| sample.power_state() == PSD_STATE_IDLE) {
        dev_info!(dev, "G18_PSD every GPU service already reads idle; no request written\n");
        return Ok(());
    }
    claim_psd_request(dev, PSD_REQUEST_RESTORE_BIT)?;
    let mut result = Ok(());
    for index in [1, 2, 0] {
        let block = &blocks[index];
        let sample = &samples[index];
        if sample.power_state() == PSD_STATE_IDLE {
            continue;
        }
        dev_warn!(dev, "G18_PSD write service={} request={} (REQUIRES USER APPROVAL)\n",
            block.service.service_id(), PSD_STATE_IDLE);
        if let Err(error) = block.request(PSD_STATE_IDLE).and_then(|()| wait_psd_state(pdev, block, PSD_STATE_IDLE)) {
            dev_err!(dev, "G18_PSD restore of service {} failed ({:?}); GPU power state is UNKNOWN; stopping\n",
                block.service.service_id(), error);
            result = Err(error);
            break;
        }
    }
    for block in blocks.iter() {
        if let Ok(sample) = block.sample() {
            log_psd(pdev, "restore after", &sample);
        }
    }
    result
}

// ---------------------------------------------------------------------------
// Stages 2..6
// ---------------------------------------------------------------------------

fn prepare_firmware_session(pdev: &platform::Device<Core<'_>>, probe: T8152Probe) -> Result<Retained> {
    let dev: &kernel::device::Device<Core<'_>> = pdev.as_ref();
    let stage = probe.stage;
    dev_warn!(dev, "G18 stage {}: starts the GPU ASC CPUs (REQUIRES USER APPROVAL)\n", stage);
    let profile = match probe.firmware_build {
        Some(profile) if probe.firmware_profile_admitted => profile,
        _ => return Err(ENODEV),
    };
    if G18_FIRMWARE_ATTEMPTED.load(Ordering::Acquire) {
        dev_err!(dev, "G18 refused: the firmware was already started this boot; reboot first\n");
        return Err(EBUSY);
    }

    // The GPU must already be powered (by iBoot, or by approved stages
    // 90..92 on this boot). This path never requests PSD power. STATE reads
    // consume transition ACK, so this steady-state gate checks the native
    // active states (SGX 0, ASCs 2), not an already-consumed acknowledgement.
    let psd = read_all_psd(pdev)?;
    for sample in psd.iter() {
        log_psd(pdev, "firmware-stage gate", sample);
    }
    if !psd.iter().all(|sample| sample.powered()) {
        dev_err!(dev, "G18 stage {} refused: the GPU PSD services are not all powered; nothing started\n", stage);
        return Err(ENODEV);
    }
    // Fresh SGX proof plus identity read.
    let powered = regs::t8152_prove_sgx_powered(pdev)?;
    let id = probe.registers.get_gpu_id_t8152(&powered, probe.soc)?;
    dev_info!(dev, "G18 SGX identity {:?}\n", id);

    // J873 hardware, 2026-09-25: with SGX+0xe90000 left at 0x30000, the GFX1
    // firmware accepts the init data but takes an async bus error on
    // SGX+0xbc08 right after the 0x89 power-transition-end. The register
    // reads 0x30000 here too, the state M5 native sets to 0x30001 before its
    // firmware runs; apply the same checked bit-0 step before any init data.
    if stage >= STAGE_INITDATA {
        probe.registers.m5_match_pmgr_boot_control()?;
    }

    // Providers: exact v8 resources, CPU control approved, CPUs stopped.
    let mut providers = KVec::new();
    for role in G18Role::ALL {
        let provider = G18Provider::get(dev, role)?;
        let lifecycle = provider.validate(dev)?;
        if lifecycle.missing != 0 || provider.require_safe_cpu_lifecycle().is_err() {
            dev_err!(dev, "G18 {} provider lifecycle missing={:#x}: boot with apple_mailbox.ascwrap_v8_cpu_control=1 (REQUIRES USER APPROVAL)\n",
                role.name(), lifecycle.missing);
            return Err(EPERM);
        }
        let (control, status) = provider.cpu_state()?;
        dev_info!(dev, "G18_ASC {} before start CPU_CONTROL={:#x} CPU_STATUS={:#x}\n",
            role.name(), control, status);
        // J873 hardware, 2026-09-25: after the PSD power-on both ASCs read
        // RUN=1 with one message waiting in the post-office RX FIFO (the
        // firmware booted on power-up and posted its HELLO). Adopt them:
        // boot_rtkit skips the CPU start and the RTKit core reads that HELLO.
        if control & ASCWRAP_V8_CPU_RUN != 0 {
            dev_info!(dev, "G18_ASC {} already running (RUN set); adopting the live CPU\n",
                role.name());
        }
        providers.push(provider, GFP_KERNEL)?;
    }

    // Firmware __DATA reservations (crashlog scan source), as new_t8152
    // checks them.
    let node = dev.of_node().ok_or(EINVAL)?;
    let mut data = [(0u64, 0u64); 2];
    for role in G18Role::ALL {
        let region = node.reserved_mem_region_to_resource_byname(role.data_region_name())?;
        if region.size() != role.data_size() || region.start() & (crate::pgtable::UAT_PGMSK as u64) != 0 {
            return Err(EINVAL);
        }
        data[role.index()] = (region.start(), region.size());
    }
    let mut shared_ro = [(0u64, 0u64); 2];
    for role in G18Role::ALL {
        if let Ok(region) = node.reserved_mem_region_to_resource_byname(role.shared_ro_region_name()) {
            shared_ro[role.index()] = (region.start(), region.size());
        }
    }

    let drm = probe.drm.clone();
    let uat = Arc::new(mmu::Uat::new_t8152(&drm, None)?, GFP_KERNEL)?;
    let mut states = KVec::new();
    let mut slots = KVec::new();
    let mut providers = providers.into_iter();
    for role in G18Role::ALL {
        let state = Arc::new(G18RoleState {
            dev: drm.clone(),
            role,
            data: data[role.index()],
            shared_ro: shared_ro[role.index()],
            acked: AtomicBool::new(false),
            crashed: AtomicBool::new(false),
            crash_dumped: AtomicBool::new(false),
            messages: AtomicU32::new(0),
        }, GFP_KERNEL)?;
        let rtkit = rtkit::RtKit::<G18RtkitOps>::new_g17p(drm.as_ref().as_ref(), None, role.index(), state.clone())
            .map_err(|error| no_probe_defer(dev, role.name(), "RTKit mailbox", error))?;
        let provider = providers.next().ok_or(EINVAL)?;
        slots.push(G18RoleSlot { role, provider, rtkit, started: false }, GFP_KERNEL)?;
        states.push(state, GFP_KERNEL)?;
    }
    let mut slots = slots.into_iter();
    let mut state_iter = states.into_iter();
    let roles = [slots.next().ok_or(EINVAL)?, slots.next().ok_or(EINVAL)?];
    let states = [state_iter.next().ok_or(EINVAL)?, state_iter.next().ok_or(EINVAL)?];
    if roles[0].role != G18Role::Gfx || roles[1].role != G18Role::Gfx1 {
        return Err(EINVAL);
    }
    let session = KBox::new(G18Session {
        roles,
        states,
        memory: None,
        queue_probe: None,
        render_job: None,
        application_job: None,
        uat,
        _registers: probe.registers,
        dev: drm,
        stage,
        profile,
        growth_refused: false,
    }, GFP_KERNEL)?;

    // Point of no return: from here every path retains the session, and the
    // module stays loaded so RTKit callbacks keep valid code until reboot.
    // No ASC CPU has started yet: once module removal has begun, refuse with
    // nothing started and the session freed.
    crate::retain::retain_module(&G18_MODULE_RETAINED)?;
    G18_FIRMWARE_ATTEMPTED.store(true, Ordering::Release);
    Ok(Retained(Some(session)))
}

fn stage_firmware(pdev: &platform::Device<Core<'_>>, probe: T8152Probe) -> Result<Arc<dyn G18Runtime>> {
    let dev: &kernel::device::Device<Core<'_>> = pdev.as_ref();
    let stage = probe.stage;
    let mut retained = prepare_firmware_session(pdev, probe)?;
    if stage == STAGE_RENDER {
        // A shallow workqueue stack avoids nesting render builders beneath
        // platform probe/module loading. The closure owns the retained session
        // before any ASC start, including on spawn failure and every exit.
        kernel::workqueue::system_long().try_spawn(GFP_KERNEL, move || {
            let result = match retained.0.as_mut() {
                Some(session) => session.run(), None => Err(EIO),
            };
            if let Some(session) = retained.0.as_mut() {
                dev_info!(session.device(), "G18_RENDER_STAGE_DONE result={:?} owners=retained\n", result);
                if result.is_err() {
                    dev_err!(session.device(), "G18_RENDER_H1_FAIL result={:?} owners=retained no-retry\n", result);
                    session.dump_fault_evidence("render worker");
                }
            }
            let _ = retained.stop("render worker end");
            if let Some(session) = retained.0.as_mut() {
                for _ in 0..4 { if session.drain().is_err() { break; } }
                if let Some(memory) = session.memory.as_ref() { memory.log_trace_journal(&session.dev); }
            }
            drop(retained);
        })?;
        dev_info!(dev, "G18_RENDER_WORKER queued; ENODEV is diagnostic, watch G18_RENDER_H1_PASS/FAIL and G18_RENDER_STAGE_DONE\n");
        return Err(ENODEV);
    }
    let session = retained.0.as_mut().ok_or(EIO)?;
    let result = session.run();
    dev_info!(dev, "G18 stage {} transaction result={:?}\n", stage, result);
    if result.is_err() {
        if stage == STAGE_RENDER { dev_err!(dev, "G18_RENDER_H1_FAIL result={:?} owners=retained no-retry\n", result); }
        session.dump_fault_evidence("failed stage");
    }
    let _ = retained.stop("stage end");

    // Bulk logging must not delay active job polling. Stop is not qualified,
    // so drain only a bounded tail and retain all owners as before. The export
    // accounts for consumed records; it does not assert the producer is halted.
    if let Some(session) = retained.0.as_mut() {
        for _ in 0..4 {
            if session.drain().is_err() {
                break;
            }
        }
        if let Some(memory) = session.memory.as_ref() {
            memory.log_trace_journal(&session.dev);
        }
    }

    match result {
        Ok(()) if matches!(stage, STAGE_HEARTBEAT | STAGE_QUEUE_REGISTER | STAGE_SCHEDULER_BIND | STAGE_POOL_BIND) => {
            let owner: Arc<G18Owner> = Arc::pin_init(
                try_pin_init!(G18Owner { session <- new_mutex!(retained, "G18Owner::session") }),
                GFP_KERNEL,
            )?;
            dev_info!(dev, "G18 stage {} complete; owner retained by the driver\n", stage);
            let owner: Arc<dyn G18Runtime> = owner;
            Ok(owner)
        }
        Ok(()) => {
            dev_info!(dev, "G18 stage {} complete; exiting ENODEV with every owner retained\n", stage);
            drop(retained);
            Err(ENODEV)
        }
        Err(error) => {
            drop(retained);
            Err(error)
        }
    }
}

/// Entry point called by the driver's T8152 probe gate for
/// `g18_initdata_test >= 1`.
pub(crate) fn start(pdev: &platform::Device<Core<'_>>, probe: T8152Probe) -> Result<Arc<dyn G18Runtime>> {
    let result = match probe.stage {
        STAGE_READ_ONLY => {
            stage_read_only(pdev, &probe);
            Err(ENODEV)
        }
        stage if stage_writes_psd(stage) => {
            let result = match psd_power_on_service(stage) {
                Some(service) => stage_psd_power_on(pdev, &probe, service),
                None => stage_psd_restore_idle(pdev),
            };
            dev_info!(pdev.as_ref(), "G18 stage {} result={:?}\n", stage, result);
            result.and(Err(ENODEV))
        }
        stage if stage_runs_firmware(stage) => stage_firmware(pdev, probe),
        _ => Err(EINVAL),
    };
    // Belt and braces: no stage may ever defer the GPU probe.
    let dev: &kernel::device::Device<Core<'_>> = pdev.as_ref();
    result.map_err(|error| no_probe_defer(dev, "GPU", "stage", error))
}
