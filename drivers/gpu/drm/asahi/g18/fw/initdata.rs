// SPDX-License-Identifier: GPL-2.0-only OR MIT

#![cfg_attr(not(test), allow(dead_code))]

//! Evidence-backed T8140/G17P firmware init-data layout.
//!
//! HISTORY / PROVENANCE. The previous revision of this module was derived from
//! the **M5 / G17S** on-disk firmware `GFX.a010.macho` (parser at VA
//! 0xfffffc000002127c) — a different chip and a different firmware build from
//! the target, **A18 Pro / T8140 / G17P**. G17S and G17P share a generation
//! name but not an ABI: every structure size in that model was wrong, the five
//! `+0x254` pointers were modeled as independent allocations instead of views
//! into one bundle, and the M5 magic 0x0c8d_e339_0728_04c0 is rejected by the
//! T8140 firmware with "Host and FW Interface is mismatched". This revision is
//! re-derived from:
//!   - live T8140 hardware runs (Mac17,5 / J700 / T8140), which booted both
//!     firmware instances with a source-constructed descriptor;
//!   - our own J700 firmware image: the version-word gate is the MOVZ/MOVK
//!     chain at `__TEXT+0x1e8a8` (`mov x10,#0x4c0; movk #0x396,lsl 16;
//!     movk #0xa322,lsl 32; movk #0xc8a,lsl 48`), and the same routine
//!     confirms the endpoint-0x20 doorbell contract
//!     (`(msg & 0xbf<<48) == 0x81<<48`, address rebuilt as
//!     `msg | 0xfffff0_0000_0000_0000`);
//!   - the executable serializers in the m1n1 tree
//!     (`proxyclient/m1n1/agx/g17p.py`, `g17p_initdata.py`) and
//!     `docs/t8140-g17p-firmware-abi-spec.md`, both validated by byte-exact
//!     rebuilds of pre-handoff captures.
//!
//! The object graph: each firmware instance is handed one root (`0xb8` bytes
//! for the primary, `0xc8` for the secondary). The root carries the version
//! tuple, the full UAT geometry (page size, level count, and three 0x20-byte
//! level descriptors), and points at a per-instance main configuration
//! (`0x600`), the shared region A (16 KiB), the shared region C (`0x1000`),
//! and per-instance status blocks. The main configuration points at the shared
//! hardware-data object (`0x3db4`, at the base of a contiguous bundle
//! allocation of at least `0x10000`) and carries a 17-entry channel table.
//! The five bare pointers at main `+0x254` are views into that one bundle;
//! allocating them independently breaks the overlap and offset relations
//! firmware relies on.
//!
//! Byte slices are used instead of packed Rust structures because several
//! proven fields are unaligned (main `+0x254`, main `+0x471`, and the region
//! record address at `+0x0c`).

use core::ops::Range;
use core::result::Result;

/// The recovered fields are sufficient for offline validation and encoding.
pub(crate) const GROUNDED_LAYOUT_AVAILABLE: bool = true;
/// The init-data object graph the T8140 firmware *accepts and boots with* —
/// per-role root sizes, the version tuple, the UAT geometry block, the main
/// configuration, the bundle-view relations, the 17-entry channel table, the
/// hardware-data acceptance block, region C, and the status blocks — is
/// grounded by live T8140 boots of a source-constructed descriptor. This is
/// what `INITDATA_GRAPH` gates.
pub(crate) const PARSE_ACCEPTED_GRAPH_AVAILABLE: bool = true;
/// Whether every hardware-data scalar the firmware reads is grounded to a
/// concrete *derivation* (rather than a recorded value). It is not: the
/// performance ladders/voltage columns and the recorded opaque platform
/// constants ([`HW_DATA_OPAQUE_RUNS`], [`REGION_C_RUNS`]) still lack an ADT
/// derivation, and this module does not encode the performance tables at all
/// (see [`PERF_TABLE_VALUES_ENCODED`]).
pub(crate) const ALL_HW_GLOBALS_VALUES_GROUNDED: bool = false;
/// The complete hardware-validated T8140 performance tables are encoded.
/// Their eventual ADT derivation remains future work, so they are retained as
/// target-scoped measured values rather than inferred from an AGX2 config.
pub(crate) const PERF_TABLE_VALUES_ENCODED: bool = true;
/// The sparse *content* of the five bundle views and of the bundle's third
/// page (`MAIN_ADDR_OBJECTS` / `HWDATA_BUNDLE_STATIC_RUNS` in `g17p.py`,
/// several KiB of recorded runs) is not yet carried by this module; only the
/// view topology is. A live boot needs those runs staged into the bundle.
pub(crate) const BUNDLE_VIEW_CONTENT_CARRIED: bool = true;
/// The primary status-B pointer names a large (`0xeb00`) status/configuration
/// object, not merely the 0x80-byte block modeled here: it holds FW-control
/// state/ring pointers at `+0x48e0`/`+0x48e8`, a lifecycle header at
/// `+0xe434`, and sparse configuration from `+0xe440`
/// (`NATIVE_PRIMARY_STATUS_B_CONFIG_RUNS` in `g17p.py`). Not yet carried.
pub(crate) const PRIMARY_STATUS_B_FULL_OBJECT_CARRIED: bool = true;

// --- Object sizes ------------------------------------------------------------

/// Root size for the primary (`gfx-asc`) instance.
pub(crate) const ROOT_SIZE_PRIMARY: usize = 0x0b8;
/// Root size for the secondary (`gfx1-asc`) instance; the extra 0x10 bytes are
/// the two secondary-only pointers at `+0xb8`/`+0xc0`.
pub(crate) const ROOT_SIZE_SECONDARY: usize = 0x0c8;
pub(crate) const MAIN_CONFIG_SIZE: usize = 0x600;
pub(crate) const HW_DATA_SIZE: usize = 0x3db4;
/// Native QoS manager block published through hardware data.
pub(crate) const G17P_QOS_RESOURCE_OFFSET: usize = 0x27a4;
/// The QoS table pointer as the LIVE 25G83 build reads it.
///
/// KEXT_PROVEN (25G83 decompile): the live initdata object is 0x2708 bytes and
/// no instruction in that build references 0x27a4 or 0x27ac at all -- both of
/// our writes land past its end, in space the firmware never reads. The real
/// field is +0x26ac, written from `AGXQOSManager + 0x28`. The neighbouring
/// live fields are +0x2614 (the 4 x 0x20 KSM completion table, which we do
/// populate), +0x2694 (a 0x40-entry table fwva), +0x2604 (deferred-kick-mode
/// selector) and the flags at +0x2600/+0x2608/+0x2610.
///
/// We keep writing 0x27a4 as well: if that offset really is dead space the
/// extra store is inert, and leaving it in place keeps this a single-variable
/// change.
pub(crate) const G17P_QOS_RESOURCE_LIVE_OFFSET: usize = 0x26ac;
pub(crate) const G17P_QOS_RESOURCE_SIZE: usize = 0xe40;
/// Native global SKSM/QID configuration block in hardware data.
pub(crate) const G17P_SKSM_QID_CONFIG_2700_OFFSET: usize = 0x2700;
pub(crate) const G17P_SKSM_QID_CONFIG_2708_OFFSET: usize = 0x2708;
pub(crate) const G17P_SKSM_QID_RESOURCE_OFFSET: usize = 0x27ac;
/// Host->firmware transport selector, `initdata[0x26f8] = accel[0x4f8] & 1`
/// (`AGXFirmware::initFirmwareData` @0xfffffe0008a20940). 1 = the host
/// publishes channel commands; 0 = the host drives the MMIO kick bridge.
const HW_DATA_TRANSPORT_SELECTOR_OFFSET: usize = 0x26f8;
/// MEASURED, live 25G83 hardware-data block: native reads 0 here.
/// initFirmwareData clears the u32 at +0x26fe, which covers +0x2700..+0x2701,
/// and never sets it. Our 1 was a non-native write inside the live object --
/// the last non-perf difference in the whole 0x2708-byte structure.
pub(crate) const G17P_SKSM_QID_CONFIG_2700: u64 = 0;
pub(crate) const G17P_SKSM_QID_CONFIG_2708: u32 = 0x100;
/// Firmware log enable word inside main config.
pub(crate) const MAIN_FIRMWARE_LOG_ENABLE: usize = 0x250;

pub(crate) const G17P_SKSM_QID_RESOURCE_SIZE: usize = 0x2000;
pub(crate) const REGION_A_SIZE: usize = 0x4000;
pub(crate) const REGION_C_SIZE: usize = 0x1000;
pub(crate) const STATUS_BLOCK_SIZE: usize = 0x80;

/// The hardware-data bundle: one contiguous allocation with the hardware-data
/// object at its base. Its principal content spans `0xc000`, but the repeated
/// writable region at `+0xc500` (named by main `+0x08`/`+0x10`) lies past
/// that, so the allocation must reach at least `0x10000`.
pub(crate) const HW_DATA_BUNDLE_SIZE: usize = 0xc000;
pub(crate) const HW_DATA_BUNDLE_MIN_ALLOC: usize = 0x10000;
/// The native placement additionally hosts the twelve work rings and both main
/// configuration objects inside the same cluster, which then spans this much
/// (see [`WORK_RING_BUNDLE_OFFSETS`], [`NATIVE_PRIMARY_MAIN_BUNDLE_OFFSET`]).
pub(crate) const NATIVE_SHARED_CLUSTER_SIZE: usize = 0x28000;
/// Firmware-high base of the complete hardware-data/main-config/work-ring
/// cluster in every output-positive G17P capture.
pub(crate) const NATIVE_SHARED_CLUSTER_GPU_VA: u64 = 0xffff_fc20_c078_8000;

// --- Version tuple ------------------------------------------------------------

/// Root `+0x00`: four u16 version words, `{0x04c0, 0x0396, 0xa322, 0x0c8a}`.
/// Grounded twice: read out of every live T8140 root, and independently
/// re-derived from the MOVZ/MOVK chain at `__TEXT+0x1e8a8` of our own J700
/// firmware image. A mismatch is rejected with "Host and FW Interface is
/// mismatched" (the M5/G17S image wants 0x0c8d_e339_0728_04c0 instead — that
/// value must never be sent to T8140 firmware).
pub(crate) const INITDATA_VERSION: [u16; 4] = [0x04c0, 0x0396, 0xa322, 0x0c8a];
/// The same tuple as the little-endian u64 the firmware compares in one go.
pub(crate) const INITDATA_VERSION_WORD: u64 = 0x0c8a_a322_0396_04c0;

/// Target/build-specific root encoding. This is a serialization choice, not
/// permission to start firmware or reuse the other target's hardware tables.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum RootProfile {
    J700G17P,
    /// J813 Mac17,3 / 25G76, RTKit-3255.160.4.release. Both roots were captured
    /// before INBOX1 publication. Independently confirmed by the loaded GFX
    /// version comparison at +0x1fa5c and GFX1 at +0x21140.
    J813G17G25G76,
    /// J873 Mac18,5 / T8152 / G18G / 26A428, RTKit_release-3514.0.15
    /// (armfw_g18g-b000 on GFX, armfw1_g18g-b010 on GFX1). No capture of a
    /// native root exists; the encoding is source-constructed from the 26A428
    /// kext and firmware (see the "J873 / G18G" section below). Selecting it
    /// changes nothing for the other two profiles.
    J873G18G26A428,
    /// J873 / G18G / 26B123 (macOS 27.2), RTKit 3514.41.1. The 26A428 graph
    /// with the 26B123 version word, a 0x7860-byte perf/policy block and a
    /// 0x6d8-byte status config (see [`G18G_26B123_LAYOUTS`]).
    J873G18G26B123,
}

impl RootProfile {
    pub(crate) const fn version_word(self) -> u64 {
        match self {
            Self::J700G17P => INITDATA_VERSION_WORD,
            Self::J813G17G25G76 => 0x0c8a_a322_039e_04c0,
            Self::J873G18G26A428 => G18G_26A428_INITDATA_VERSION_WORD,
            Self::J873G18G26B123 => G18G_26B123_INITDATA_VERSION_WORD,
        }
    }

    /// Either G18G (J873) build.
    pub(crate) const fn is_g18g(self) -> bool {
        matches!(self, Self::J873G18G26A428 | Self::J873G18G26B123)
    }

    /// The G18G InitData layouts of this build; None for J700/J813.
    pub(crate) const fn g18g_layouts(self) -> Option<&'static G18GLayouts> {
        match self {
            Self::J873G18G26A428 => Some(&G18G_26A428_LAYOUTS),
            Self::J873G18G26B123 => Some(&G18G_26B123_LAYOUTS),
            Self::J700G17P | Self::J813G17G25G76 => None,
        }
    }

    /// Bytes of the root object for `role`. J700/J813 keep the historical
    /// 0xb8/0xc8 split. G18G sizes both roots to 0xc8 because both firmware
    /// images copy 0xc8 bytes from each root (FW_PROVEN b000 0x21664-0x216a8).
    pub(crate) const fn root_size(self, role: InstanceRole) -> usize {
        match self {
            Self::J873G18G26A428 | Self::J873G18G26B123 => G18G_26A428_ROOT_SIZE,
            Self::J700G17P | Self::J813G17G25G76 => root_size(role),
        }
    }

    /// Opcode of the one opening device-control record staged at main
    /// `+0x4c0`. G18G renumbered the opening handler: 0x16 is now an
    /// unrelated handler and 0x15 reaches the opening (FW_PROVEN jump table
    /// 0xfffffc0000006c60, handler b000 0x3ed0). The secondary keeps 0x2a.
    pub(crate) const fn control_opening_opcode(self, role: InstanceRole) -> u32 {
        match (self, role) {
            (Self::J873G18G26A428 | Self::J873G18G26B123, InstanceRole::Primary) => {
                G18G_26A428_CONTROL_OPENING_OPCODE_PRIMARY
            }
            (Self::J873G18G26A428 | Self::J873G18G26B123, InstanceRole::Secondary) => {
                G18G_26A428_CONTROL_OPENING_OPCODE_SECONDARY
            }
            (_, InstanceRole::Primary) => CONTROL_OPENING_OPCODE_PRIMARY,
            (_, InstanceRole::Secondary) => CONTROL_OPENING_OPCODE_SECONDARY,
        }
    }

    /// The "primary register" device-control opcode (0x20 on J700/M5).
    /// FW_PROVEN: G18G opcode 0x20 is in the invalid set of the 56-entry
    /// jump table, so the profile has none and callers must not emit one.
    pub(crate) const fn control_register_opcode(self) -> Option<u32> {
        match self {
            Self::J873G18G26A428 | Self::J873G18G26B123 => None,
            Self::J700G17P | Self::J813G17G25G76 => Some(CONTROL_OPENING_PRIMARY_REGISTER_OPCODE),
        }
    }

    /// Bundle offsets of the five views published at main `+0x254`.
    pub(crate) const fn bundle_view_offsets(self) -> [usize; 5] {
        match self {
            Self::J873G18G26A428 => G18G_26A428_BUNDLE_VIEW_OFFSETS,
            Self::J873G18G26B123 => G18G_26B123_BUNDLE_VIEW_OFFSETS,
            Self::J700G17P | Self::J813G17G25G76 => BUNDLE_VIEW_OFFSETS,
        }
    }

    /// Extents each view must fit inside the bundle allocation.
    pub(crate) const fn bundle_view_extents(self) -> [usize; 5] {
        match self {
            // 26B123 grows only the policy; the five views keep their sizes.
            Self::J873G18G26A428 | Self::J873G18G26B123 => G18G_26A428_BUNDLE_VIEW_EXTENTS,
            Self::J700G17P | Self::J813G17G25G76 => BUNDLE_VIEW_EXTENTS,
        }
    }

    /// Bundle offset named twice by main `+0x08`/`+0x10` (the empty BRN
    /// table and the 0x88 block, which the allocator places together).
    pub(crate) const fn repeated_region_bundle_offset(self) -> usize {
        match self {
            Self::J873G18G26A428 => G18G_26A428_SHARED_LAYOUT.block_88,
            Self::J873G18G26B123 => G18G_26B123_SHARED_LAYOUT.block_88,
            Self::J700G17P | Self::J813G17G25G76 => MAIN_REPEATED_REGION_BUNDLE_OFFSET,
        }
    }

    /// Bundle offsets of the twelve work rings in channel-table order.
    pub(crate) const fn work_ring_bundle_offsets(self) -> [usize; WORK_CHANNEL_COUNT] {
        match self {
            Self::J873G18G26A428 => G18G_26A428_WORK_RING_BUNDLE_OFFSETS,
            Self::J873G18G26B123 => G18G_26B123_WORK_RING_BUNDLE_OFFSETS,
            Self::J700G17P | Self::J813G17G25G76 => WORK_RING_BUNDLE_OFFSETS,
        }
    }
}

// --- Address representability --------------------------------------------------

// The endpoint-0x20 doorbell carries the root address in a 44-bit field; the
// J700 firmware rebuilds it as `msg | 0xfffff0_0000_0000_0000`. Every pointer
// placed in the graph must therefore be either bare or carry exactly that
// kernel-half prefix, or it silently truncates.
const INITDATA_ADDR_MASK: u64 = 0x0000_0fff_ffff_ffff;
const INITDATA_ADDR_PREFIX: u64 = 0xffff_f000_0000_0000;

// --- Root layout ---------------------------------------------------------------

const ROOT_VERSION: usize = 0x00;
const ROOT_REGION_A: usize = 0x08;
/// Root `+0x10` is zero at handoff on both instances; semantics unknown.
const ROOT_ZERO_10: usize = 0x10;
const ROOT_MAIN_CONFIG: usize = 0x18;
const ROOT_REGION_C: usize = 0x20;
const ROOT_INSTANCE_KIND: usize = 0x28;
const ROOT_CONSTANT_2C: usize = 0x2c;
const ROOT_PAGE_SIZE: usize = 0x30;
const ROOT_PAGE_BITS: usize = 0x32;
const ROOT_LEVEL_COUNT: usize = 0x33;
const ROOT_LEVELS: usize = 0x34;
/// Root `+0xa8`: Status A pointer. (The M5-derived model misused this slot as
/// a role pointer.)
const ROOT_STATUS_A: usize = 0xa8;
/// Root `+0xb0`: Status B pointer. Real for the primary, zero for the
/// secondary. (The M5-derived model misused it as a cross-role pointer.)
const ROOT_STATUS_B: usize = 0xb0;
const ROOT_SECONDARY_EXTRA_0: usize = 0xb8;
const ROOT_SECONDARY_EXTRA_1: usize = 0xc0;

// --- UAT geometry (root +0x30..+0x94) -----------------------------------------

pub(crate) const UAT_PAGE_SIZE: u16 = 0x4000;
pub(crate) const UAT_PAGE_BITS: u8 = 14;
pub(crate) const UAT_LEVEL_COUNT: usize = 3;
/// `(index_shift, entry_count)` for the three levels below the root,
/// confirmed against a live table walk.
pub(crate) const UAT_LEVELS: [(u8, u16); UAT_LEVEL_COUNT] = [(36, 64), (25, 2048), (14, 2048)];
pub(crate) const UAT_LEVEL_DESC_SIZE: usize = 0x20;
/// Level descriptor `+0x10`: physical mask, identical in all three.
pub(crate) const UAT_LEVEL_PHYS_MASK: u64 = 0x0000_03ff_ffff_c000;

// --- Main configuration layout --------------------------------------------------

const MAIN_HW_DATA: usize = 0x00;
/// The same repeated-region address appears at `+0x08` and `+0x10`; it is the
/// bundle base plus [`MAIN_REPEATED_REGION_BUNDLE_OFFSET`], not the bundle
/// base itself. It reads zero at handoff (firmware writes into it), so the
/// host only has to provide it mapped.
const MAIN_REPEATED_0: usize = 0x08;
const MAIN_REPEATED_1: usize = 0x10;
pub(crate) const MAIN_REPEATED_REGION_BUNDLE_OFFSET: usize = 0xc500;
const MAIN_CHANNEL_TABLE: usize = 0x20;
/// Main `+0x254`: five unaligned u64 views into the hardware-data bundle.
const MAIN_BUNDLE_VIEWS: usize = 0x254;
/// Main `+0x2d0`: six qwords of region views.
const MAIN_REGION_VIEWS: usize = 0x2d0;
/// Main `+0x300`: u32 4, secondary only.
const MAIN_SECONDARY_KIND: usize = 0x300;
const MAIN_SECONDARY_KIND_VALUE: u32 = 4;
/// Main `+0x3e0`: u32 0xff, primary only.
const MAIN_PRIMARY_MASK: usize = 0x3e0;
const MAIN_PRIMARY_MASK_VALUE: u32 = 0xff;
const MAIN_PRIMARY_SCALAR_32C: usize = 0x32c;
const MAIN_PRIMARY_SCALAR_344: usize = 0x344;
/// Main `+0x471`: unaligned u64, secondary only, pointing *into* bundle view 2
/// at [`SECONDARY_EXTRA_VIEW_OFFSET`] (not a fresh allocation — the M5-derived
/// model had this at `+0x481` backed by an invented 0x12688-byte buffer; both
/// were wrong).
const MAIN_SECONDARY_EXTRA_PTR: usize = 0x471;
pub(crate) const SECONDARY_EXTRA_VIEW_INDEX: usize = 2;
pub(crate) const SECONDARY_EXTRA_VIEW_OFFSET: usize = 0xe40;
/// Main `+0x4c0` is both a scalar and a place: the leading u32 of the
/// device-control ring, which is embedded in the main configuration object at
/// exactly this offset on both instances. Writing the opening opcode here *is*
/// staging the opening device-control record (records are 0x40 bytes; fields
/// past the opcode are zero). The control channel's producer counter must then
/// be set to the role-specific value returned by
/// [`control_opening_producer`] before the descriptor is handed over.
const MAIN_CONTROL_RING: usize = 0x4c0;
pub(crate) const CONTROL_OPENING_OPCODE_PRIMARY: u32 = 0x16;
pub(crate) const CONTROL_OPENING_OPCODE_SECONDARY: u32 = 0x2a;
pub(crate) const CONTROL_RECORD_SIZE: usize = 0x40;
pub(crate) const CONTROL_OPENING_PRIMARY_RECORDS: usize = 1;
pub(crate) const CONTROL_OPENING_SECONDARY_RECORDS: usize = 1;
pub(crate) const CONTROL_OPENING_PRIMARY_PRODUCER: u32 = 1;
pub(crate) const CONTROL_OPENING_SECONDARY_PRODUCER: u32 = 1;
pub(crate) const CONTROL_OPENING_PRIMARY_REGISTER_OPCODE: u32 = 0x20;
pub(crate) const CONTROL_OPENING_SECONDARY_CONTINUE_OPCODE: u32 = 0x22;
/// Hardware-buffer release emitted after the opening UMA grow request retires.
pub(crate) const CONTROL_BOOTSTRAP_UMA_RELEASE_OPCODE: u32 = 0x2e;
/// Scheduler-state (channel control) release.
///
/// Decoded from AGXG17P 353.14 (25G83) `AGXArmFirmware::submitReleaseResource`
/// at `0xfffffe00088a722c`: the record's first word is `0x14` on this image
/// (the G17X image audited for the G17 native detach plan
/// carries `0x13`; the G17P opcode route table at `0xfffffe0007136180` sends
/// `0x14` to ring 0, the same primary control ring as `0x2e`, and `0x13` to
/// ring 1). `AGXCommandQueue::freeReclaimableResources` (`0xfffffe0008865354`)
/// submits it with the four identity bytes read back from the live scheduler
/// state page and then waits for the ring consumer to pass the record before
/// it frees the work queues and their parameter management.
pub(crate) const CONTROL_SCHEDULER_STATE_RELEASE_OPCODE: u32 = 0x14;
/// `AGXCommandQueue::processExitTimeoutHandler` (@fffffe000886a494): kill
/// the work of one scheduler-state page; acknowledged by firmware event 12
/// (`kAGFIFirmwareEventTypeProcessExitComplete`) carrying the cookie.
pub(crate) const CONTROL_CONTEXT_KILL_OPCODE: u32 = 0x29;
/// Device-control ring capacity. FW_PROVEN (b1
/// `AGFAcceleratorDrainDeviceControlRing` @ `0xfffffc0000003a74`): the drain
/// walks `ring + index * 0x40` and advances its consumer with
/// `(consumer + 1) & 0xff`, so the ring is exactly 256 records. The placement
/// agrees byte for byte: the primary ring starts at
/// [`NATIVE_PRIMARY_MAIN_BUNDLE_OFFSET`] `+ 0x4c0` = bundle `+0x1ea80` and the
/// secondary main configuration begins at bundle `+0x22a80`, exactly
/// `0x100 * 0x40` bytes later.
pub(crate) const CONTROL_RING_ENTRIES: usize = 0x100;
pub(crate) const CONTROL_RING_INDEX_MASK: u32 = CONTROL_RING_ENTRIES as u32 - 1;
/// Device control `IdlePowerOff(arg)`. FW_PROVEN: the opcode jump table at
/// `0xfffffc0000005380` (indexed by `opcode - 4`) sends `0x0a` to
/// `0xfffffc0000003fd8`, which stores the record's `arg` into the idle-power-off
/// enable word `[0xfffffc0000177838]` and, **only when that arg is zero**, does
/// an unconditional `bl 0xfffffc0000021960`. That routine is the one thing in
/// the image that releases KSM pause reason bit 1 (mask `0x2`) in steady state:
/// it latches `[0xfffffc00001771f8] |= 2` and tail-calls
/// `gpu_core_power(0, 1, 0)` @ `0xfffffc000000e004`, whose power-on arm clears
/// the reason bit at `0xe488` and emits KTrace `0x46 (0, 0x2, mask)` at
/// `0xe4cc`. It is gated by nothing -- unlike the EP-0x21 work doorbell, which
/// handler `0x261a4` drops silently while `[0x1771f8] & 0x18` (RTKit
/// SLEEP/NAP) -- and it is idempotent, because `0x21960` returns immediately
/// when the powered latch is already set.
pub(crate) const DEVICE_CONTROL_OPCODE_IDLE_POWER_OFF: u32 = 0x0a;
/// Disable ordinary idle power-off and request that the GPU cores wake now.
///
/// FW_PROVEN (25F80 B100, SHA256 83af9ae0...): opcode 0x0a stores this value
/// in `[0xfffffc0000177838]` and calls the wake path only for zero. The
/// scheduler's 0x20000000 idle event skips its descent while that policy word
/// is zero. A cold wake may reload the word from Region-C +0x30, so callers
/// which need a scoped hold must retire a second zero after the core is up.
pub(crate) const DEVICE_CONTROL_IDLE_POWER_OFF_ARG_DISABLE_IDLE_AND_WAKE: u32 = 0;
/// Restore the firmware's ordinary idle power-off policy. A non-zero value
/// permits the scheduler idle event to descend; it is not a power hold.
pub(crate) const DEVICE_CONTROL_IDLE_POWER_OFF_ARG_ENABLE_IDLE: u32 = 1;
/// Separate retained compute resources from the render pool. Native
/// populateInitialFirmwareState (8aa258c) publishes resource-derived +0x14
/// page-list and +0x30 run-list addresses; neither is firmware-fixed.
pub(crate) const COMPUTE_FLIST_RELOCATION: u64 = 0x0200_0000;
pub(crate) const COMPUTE_FLIST_PAGE_LIST_ADDRESS: u64 = 0x70_0000_0000 + COMPUTE_FLIST_RELOCATION;
pub(crate) const COMPUTE_FLIST_RUN_TABLE_ADDRESS: u64 = CONTROL_OPERAND_TABLE_ADDRESS + COMPUTE_FLIST_RELOCATION;
pub(crate) const COMPUTE_FLIST_BUFFER_BASE: u64 = CONTROL_OPERAND_BUFFER_BASE + COMPUTE_FLIST_RELOCATION;

pub(crate) const COMPUTE_RUNTIME_CONTROL_RECORDS: usize = 65;
pub(crate) const COMPUTE_RUNTIME_CONTROL_FINAL_COUNTER: u32 =
    CONTROL_OPENING_PRIMARY_PRODUCER + COMPUTE_RUNTIME_CONTROL_RECORDS as u32;
pub(crate) const COMPUTE_RUNTIME_CLASS1_RECORD_INDEX: usize = 54;
pub(crate) const COMPUTE_RUNTIME_CLASS2_RECORD_INDEX: usize = 57;
pub(crate) const COMPUTE_RUNTIME_SUPPORT_ADDRESS: u64 = 0xffff_fc20_c086_8000;
pub(crate) const COMPUTE_RUNTIME_STATE_ADDRESS: u64 = 0xffff_fc20_0163_0000;
pub(crate) const COMPUTE_RUNTIME_CLASS1_TABLE_ADDRESS: u64 = 0x70_01ad_8000 + COMPUTE_FLIST_RELOCATION;
pub(crate) const COMPUTE_RUNTIME_PAGE_LIST_ADDRESS: u64 = 0x70_018d_0000 + COMPUTE_FLIST_RELOCATION;
pub(crate) const COMPUTE_RUNTIME_ZERO_BUFFER_0_ADDRESS: u64 = 0x70_0000_0000 + COMPUTE_FLIST_RELOCATION;
pub(crate) const COMPUTE_RUNTIME_ZERO_BUFFER_1_ADDRESS: u64 = 0x70_0010_8000 + COMPUTE_FLIST_RELOCATION;
pub(crate) const COMPUTE_RUNTIME_SUPPORT_SIZE: usize = 0x4000;
pub(crate) const COMPUTE_RUNTIME_CLASS1_TABLE_SIZE: usize = 0x4000;
pub(crate) const COMPUTE_RUNTIME_PAGE_LIST_SIZE: usize = 0xa800;
pub(crate) const COMPUTE_DISPATCH_RECORD_SIZE: usize = 0x20;

/// Output-positive first-CL readiness suffix. These six records follow the
/// clean three-record QoS opening. Native publishes and retires each slot
/// before exposing the next.
pub(crate) const COMPUTE_READINESS_RECORDS: usize = 6;
/// Output-positive final compute graph maps and names exactly operand entries
/// 0..20. The retained opening still owns 28 backing objects.
pub(crate) const COMPUTE_READINESS_OPERAND_ENTRY_COUNT: usize = 21;
pub(crate) const COMPUTE_READINESS_FINAL_COUNTER: u32 =
    CONTROL_OPENING_PRIMARY_PRODUCER + COMPUTE_READINESS_RECORDS as u32;
pub(crate) const COMPUTE_READINESS_CLASS1_SUPPORT_ADDRESS: u64 = 0xffff_fc20_c087_8000;
pub(crate) const COMPUTE_READINESS_CLASS1_STATE_ADDRESS: u64 = 0xffff_fc20_0164_8000;
pub(crate) const COMPUTE_READINESS_CLASS3_SUPPORT_ADDRESS: u64 = 0xffff_fc20_c085_0000;
pub(crate) const COMPUTE_READINESS_CLASS3_STATE_ADDRESS: u64 = 0xffff_fc20_0165_0000;
pub(crate) const COMPUTE_READINESS_PAGE_SIZE: usize = 0x4000;
pub(crate) const COMPUTE_READINESS_ACTIVATION_RECORD_INDEX: usize = 2;
pub(crate) const COMPUTE_READINESS_ACTIVATION_RECORD_OFFSET: usize =
    COMPUTE_READINESS_ACTIVATION_RECORD_INDEX * CONTROL_RECORD_SIZE;

/// One native first-CL device-control publication boundary.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct ComputeReadinessRecordBoundary {
    pub(crate) index: u8,
    pub(crate) producer_before: u32,
    pub(crate) producer_after: u32,
    pub(crate) opcode: u32,
    pub(crate) sequence: u32,
    pub(crate) context: u32,
}

pub(crate) const COMPUTE_READINESS_BOUNDARIES:
    [ComputeReadinessRecordBoundary; COMPUTE_READINESS_RECORDS] = [
    ComputeReadinessRecordBoundary {
        index: 0,
        producer_before: CONTROL_OPENING_PRIMARY_PRODUCER,
        producer_after: CONTROL_OPENING_PRIMARY_PRODUCER + 1,
        opcode: 0x20,
        sequence: 35,
        context: 1,
    },
    ComputeReadinessRecordBoundary {
        index: 1,
        producer_before: CONTROL_OPENING_PRIMARY_PRODUCER + 1,
        producer_after: CONTROL_OPENING_PRIMARY_PRODUCER + 2,
        opcode: 0x2e,
        sequence: 35,
        context: 1,
    },
    ComputeReadinessRecordBoundary {
        index: 2,
        producer_before: CONTROL_OPENING_PRIMARY_PRODUCER + 2,
        producer_after: CONTROL_OPENING_PRIMARY_PRODUCER + 3,
        opcode: 0x2e,
        sequence: 36,
        context: 1,
    },
    ComputeReadinessRecordBoundary {
        index: 3,
        producer_before: CONTROL_OPENING_PRIMARY_PRODUCER + 3,
        producer_after: CONTROL_OPENING_PRIMARY_PRODUCER + 4,
        opcode: 0x20,
        sequence: 37,
        context: 2,
    },
    ComputeReadinessRecordBoundary {
        index: 4,
        producer_before: CONTROL_OPENING_PRIMARY_PRODUCER + 4,
        producer_after: CONTROL_OPENING_PRIMARY_PRODUCER + 5,
        opcode: 0x2e,
        sequence: 37,
        context: 2,
    },
    ComputeReadinessRecordBoundary {
        index: 5,
        producer_before: CONTROL_OPENING_PRIMARY_PRODUCER + 5,
        producer_after: CONTROL_OPENING_PRIMARY_PRODUCER + 6,
        opcode: 0x2e,
        sequence: 38,
        context: 2,
    },
];

pub(crate) const fn compute_readiness_boundary(
    index: usize,
) -> Option<ComputeReadinessRecordBoundary> {
    if index < COMPUTE_READINESS_RECORDS {
        Some(COMPUTE_READINESS_BOUNDARIES[index])
    } else {
        None
    }
}

/// Host progress for the native per-record publication handshake.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct ComputeReadinessProgress {
    retired_records: u8,
    pending_target: Option<u32>,
}

impl ComputeReadinessProgress {
    pub(crate) const fn new() -> Self {
        Self {
            retired_records: 0,
            pending_target: None,
        }
    }

    pub(crate) const fn retired_records(self) -> usize {
        self.retired_records as usize
    }

    pub(crate) const fn pending_target(self) -> Option<u32> {
        self.pending_target
    }

    pub(crate) const fn next_boundary(self) -> Option<ComputeReadinessRecordBoundary> {
        if self.pending_target.is_some() {
            None
        } else {
            compute_readiness_boundary(self.retired_records as usize)
        }
    }

    pub(crate) const fn begin_publication(
        self,
        boundary: ComputeReadinessRecordBoundary,
    ) -> Option<Self> {
        if self.pending_target.is_some() || boundary.index != self.retired_records {
            return None;
        }
        Some(Self {
            retired_records: self.retired_records,
            pending_target: Some(boundary.producer_after),
        })
    }

    pub(crate) const fn retire(self, target: u32) -> Option<Self> {
        match self.pending_target {
            Some(pending) if pending == target => Some(Self {
                retired_records: self.retired_records + 1,
                pending_target: None,
            }),
            _ => None,
        }
    }

    pub(crate) const fn complete(self) -> bool {
        self.retired_records as usize == COMPUTE_READINESS_RECORDS
            && self.pending_target.is_none()
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum ComputeReadinessPublicationStep {
    ControlRecord(u8),
    InitializeContext2Activation,
}

pub(crate) const COMPUTE_READINESS_PUBLICATION_ORDER: [ComputeReadinessPublicationStep; 7] = [
    ComputeReadinessPublicationStep::ControlRecord(0),
    ComputeReadinessPublicationStep::ControlRecord(1),
    ComputeReadinessPublicationStep::ControlRecord(2),
    ComputeReadinessPublicationStep::InitializeContext2Activation,
    ComputeReadinessPublicationStep::ControlRecord(3),
    ComputeReadinessPublicationStep::ControlRecord(4),
    ComputeReadinessPublicationStep::ControlRecord(5),
];

pub(crate) const CONTROL_SHARED_ADDRESS: u64 = 0xffff_fc20_c083_0000;
pub(crate) const CONTROL_SHARED_INNER_ADDRESS: u64 = 0xffff_fc20_0160_8000;
pub(crate) const CONTROL_SHARED_OBJECT_SIZE: usize = 0x4000;
pub(crate) const CONTROL_SHARED_CURSOR_OFFSET: usize = 0x48;
pub(crate) const CONTROL_SHARED_CURSOR_BEFORE: u32 = 0x88;
pub(crate) const CONTROL_SHARED_INNER_PRESENTED: u64 = 2;
/// Separate compact class-2 object consumed by the first-partial opening.
/// Descriptor tails and later compute control continue to use
/// [`CONTROL_SHARED_ADDRESS`].
pub(crate) const PARTIAL_OPENING_CONTROL_SHARED_ADDRESS: u64 = 0xffff_fc20_c082_8000;
pub(crate) const PARTIAL_OPENING_CONTROL_SHARED_INNER_ADDRESS: u64 =
    0xffff_fc20_0160_0000;
/// Hardware-proven compact compute cursor. Twenty-one consecutive run qwords
/// occupy slots 0..20 and the next eight-qword boundary is 0x18.
pub(crate) const COMPUTE_PARTIAL_OPENING_CONTROL_SHARED_CURSOR_BEFORE: u32 = 0x18;
/// Initial compact FList cursor in qwords.
///
/// `AGXUSCPrivMemFList::prepareBufferResources` encodes one 1-MiB run for
/// each of the 17 initial pool blocks, rounding to an eight-qword slot after
/// each block. The first grow starts at qword 17 * 8 = 0x88 (byte +0x440),
/// exactly as the complete 25G83 first-render opcode-0x20 record publishes.
pub(crate) const PARTIAL_OPENING_CONTROL_SHARED_CURSOR_BEFORE: u32 = 0x88;
pub(crate) const PARTIAL_OPENING_CONTROL_SHARED_INNER_PRESENTED: u64 = 0;
/// Linux owns one USC private-memory FList. HardwareBufferIDManager permits
/// slot zero (native's generic allocator pops 0 first); macOS observed ID 1
/// only because another live FList already owned slot zero in that session.
pub(crate) const PARTIAL_OPENING_FREELIST_HARDWARE_BUFFER_ID: u32 = 0;
pub(crate) const CONTROL_OPERAND_TABLE_ADDRESS: u64 = 0x70_0020_8000;
pub(crate) const CONTROL_OPERAND_TABLE_SIZE: usize = 0x4000;
pub(crate) const CONTROL_OPERAND_SLOT_OFFSET: u64 = 0x440;
pub(crate) const PARTIAL_OPENING_CONTROL_OPERAND_SLOT_OFFSET: u64 = 0x640;
pub(crate) const PARTIAL_OPENING_CONTROL_REGISTER_COUNT: u32 = 0x18;
pub(crate) const PARTIAL_OPENING_CONTROL_OPERAND_ENTRY_COUNT: usize = 28;
pub(crate) const CONTROL_OPERAND_BUFFER_BASE: u64 = 0x70_0022_0000;
pub(crate) const CONTROL_OPERAND_BUFFER_STRIDE: u64 = 0x10_8000;
pub(crate) const CONTROL_OPERAND_BUFFER_SIZE: u64 = 0x10_0000;
pub(crate) const CONTROL_OPERAND_ENTRY_COUNT: usize = 22;
pub(crate) const CONTROL_OPERAND_ENTRY_STRIDE: usize = 0x40;
pub(crate) const CONTROL_OPERAND_ENTRY_FLAG: u64 = 1 << 60;
pub(crate) const COMPUTE_RUNTIME_BUFFER_ADDRESSES: [u64; 21] = [
    0x70_0022_0000 + COMPUTE_FLIST_RELOCATION,
    0x70_0032_8000 + COMPUTE_FLIST_RELOCATION,
    0x70_0043_0000 + COMPUTE_FLIST_RELOCATION,
    0x70_0053_8000 + COMPUTE_FLIST_RELOCATION,
    0x70_0064_0000 + COMPUTE_FLIST_RELOCATION,
    0x70_0074_8000 + COMPUTE_FLIST_RELOCATION,
    0x70_0085_0000 + COMPUTE_FLIST_RELOCATION,
    0x70_0095_8000 + COMPUTE_FLIST_RELOCATION,
    0x70_00a6_0000 + COMPUTE_FLIST_RELOCATION,
    0x70_00b6_8000 + COMPUTE_FLIST_RELOCATION,
    0x70_00c7_0000 + COMPUTE_FLIST_RELOCATION,
    0x70_00d7_8000 + COMPUTE_FLIST_RELOCATION,
    0x70_00e8_0000 + COMPUTE_FLIST_RELOCATION,
    0x70_00f8_8000 + COMPUTE_FLIST_RELOCATION,
    0x70_0109_0000 + COMPUTE_FLIST_RELOCATION,
    0x70_0119_8000 + COMPUTE_FLIST_RELOCATION,
    0x70_012a_0000 + COMPUTE_FLIST_RELOCATION,
    0x70_013a_8000 + COMPUTE_FLIST_RELOCATION,
    COMPUTE_RUNTIME_ZERO_BUFFER_0_ADDRESS,
    COMPUTE_RUNTIME_ZERO_BUFFER_1_ADDRESS,
    0x70_014b_0000 + COMPUTE_FLIST_RELOCATION,
];

pub(crate) const fn control_opening_producer(role: InstanceRole) -> u32 {
    match role {
        InstanceRole::Primary => CONTROL_OPENING_PRIMARY_PRODUCER,
        InstanceRole::Secondary => CONTROL_OPENING_SECONDARY_PRODUCER,
    }
}

pub(crate) const fn control_opening_size(role: InstanceRole) -> usize {
    match role {
        InstanceRole::Primary => CONTROL_OPENING_PRIMARY_RECORDS * CONTROL_RECORD_SIZE,
        InstanceRole::Secondary => CONTROL_OPENING_SECONDARY_RECORDS * CONTROL_RECORD_SIZE,
    }
}

fn encode_control_registration(
    out: &mut [u8],
    slot: usize,
    control_class: u32,
    sequence: u32,
    object: u64,
    operand: u64,
    slot_offset: u64,
    count: u32,
    context_word: u32,
) {
    let base = slot * CONTROL_RECORD_SIZE;
    put_u32(out, base, CONTROL_OPENING_PRIMARY_REGISTER_OPCODE);
    put_u32(out, base + 0x04, control_class);
    put_u32(out, base + 0x08, 0x3f);
    put_u32(out, base + 0x0c, sequence);
    put_u64(out, base + 0x14, object);
    put_u64(out, base + 0x1c, operand);
    put_u64(out, base + 0x24, operand + slot_offset);
    put_u32(out, base + 0x2c, count);
    put_u32(out, base + 0x30, context_word);
    put_u32(out, base + 0x34, 1);
}

fn encode_control_tick(out: &mut [u8], slot: usize, sequence: u32) {
    let base = slot * CONTROL_RECORD_SIZE;
    put_u32(out, base, CONTROL_BOOTSTRAP_UMA_RELEASE_OPCODE);
    put_u32(out, base + 0x04, sequence);
}

/// Encode the exact release record produced when an Async FList grow request
/// drops its last HardwareBuffer ID reference. The captured opening uses
/// sequence zero and ID zero; accepting them as operands keeps the encoder
/// tied to the retained host ownership rather than to a raw type-13 tuple.
pub(crate) fn encode_bootstrap_uma_release_record(
    sequence: u64,
    hardware_buffer_id: u32,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_RECORD_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0, CONTROL_BOOTSTRAP_UMA_RELEASE_OPCODE);
    put_u64(out, 0x04, sequence);
    put_u32(out, 0x0c, hardware_buffer_id);
    Ok(())
}

/// Encode the scheduler-state release record exactly as
/// `AGXArmFirmware::submitReleaseResource(va, a, b, c, d)` writes it into its
/// 64-byte device-control entry: opcode at `+0`, zero at `+4`, byte `d` at
/// `+8`, `a` at `+9`, `b` at `+0xa`, `c` at `+0xb`, and the resource's
/// firmware VA as an unaligned u64 at `+0xc`. `identity` is `[a, b, c, d]`,
/// i.e. bytes `[0]`, `[1]`, `[4]` and `[0x26]` of the live scheduler-state
/// page, as `freeReclaimableResources` loads them at `0xfffffe0008865398`.
pub(crate) fn encode_scheduler_state_release_record(
    resource_va: u64,
    identity: [u8; 4],
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_RECORD_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0, CONTROL_SCHEDULER_STATE_RELEASE_OPCODE);
    out[0x08] = identity[3];
    out[0x09] = identity[0];
    out[0x0a] = identity[1];
    out[0x0b] = identity[2];
    put_u64(out, 0x0c, resource_va);
    Ok(())
}

/// 64-byte process-exit kill record exactly as `processExitTimeoutHandler`
/// builds it on its stack (record base sp+8): opcode at `+0`, the
/// scheduler-state page VA (queue+0x8b8) as an unaligned u64 at `+4`, and
/// the cookie as an unaligned u64 at `+0xc`; everything else zero. Firmware
/// 0x29 (fw 0x49cc) reads exactly `[x24,#4]` and `[x24,#0xc]`.
pub(crate) fn encode_context_kill_record(
    scheduler_state_va: u64,
    cookie: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_RECORD_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0, CONTROL_CONTEXT_KILL_OPCODE);
    put_u64(out, 0x04, scheduler_state_va);
    put_u64(out, 0x0c, cookie);
    Ok(())
}

fn encode_control_tick_with_context(
    out: &mut [u8],
    slot: usize,
    sequence: u32,
    context_word: u32,
) {
    encode_control_tick(out, slot, sequence);
    let base = slot * CONTROL_RECORD_SIZE;
    put_u32(out, base + 0x0c, context_word);
}

/// Encode one of the six exact records immediately preceding the first CL.
pub(crate) fn encode_compute_readiness_record(
    index: usize,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_RECORD_SIZE || index >= COMPUTE_READINESS_RECORDS {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    match index {
        0 => encode_control_registration(
            out,
            0,
            1,
            35,
            COMPUTE_READINESS_CLASS1_SUPPORT_ADDRESS,
            COMPUTE_FLIST_RUN_TABLE_ADDRESS,
            0x440,
            0x20,
            1,
        ),
        1 => encode_control_tick_with_context(out, 0, 35, 1),
        2 => encode_control_tick_with_context(out, 0, 36, 1),
        3 => encode_control_registration(
            out,
            0,
            3,
            37,
            COMPUTE_READINESS_CLASS3_SUPPORT_ADDRESS,
            COMPUTE_FLIST_RUN_TABLE_ADDRESS,
            0x5c0,
            0x28,
            2,
        ),
        4 => encode_control_tick_with_context(out, 0, 37, 2),
        5 => encode_control_tick_with_context(out, 0, 38, 2),
        _ => return Err(InitdataError::BufferSize),
    }
    Ok(())
}

fn encode_compute_readiness_support(
    out: &mut [u8],
    class: u32,
    active: u32,
    resource_class: u64,
    cursor: u32,
    state: u64,
    final_kind: u32,
) -> Result<(), InitdataError> {
    if out.len() != COMPUTE_READINESS_PAGE_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0x00, class);
    put_u32(out, 0x08, active);
    put_u32(out, 0x10, class);
    put_u64(out, 0x14, COMPUTE_FLIST_PAGE_LIST_ADDRESS);
    put_u32(out, 0x1c, 0x4_0000);
    put_u64(out, 0x20, resource_class << 40);
    put_u64(out, 0x28, resource_class << 40);
    put_u64(out, 0x30, COMPUTE_FLIST_RUN_TABLE_ADDRESS);
    put_u32(out, 0x40, 4);
    put_u32(out, 0x48, cursor);
    put_u64(out, 0x4c, state);
    put_u32(out, 0x54, 0);
    put_u32(out, 0x5c, 0);
    put_u32(out, 0x60, final_kind);
    Ok(())
}

pub(crate) fn encode_compute_readiness_class1_support(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    encode_compute_readiness_support(
        out,
        1,
        1,
        0x11,
        0x88,
        COMPUTE_READINESS_CLASS1_STATE_ADDRESS,
        2,
    )
}

pub(crate) fn encode_compute_readiness_class3_support(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    encode_compute_readiness_support(
        out,
        3,
        2,
        0x17,
        0xb8,
        COMPUTE_READINESS_CLASS3_STATE_ADDRESS,
        3,
    )
}

pub(crate) fn encode_compute_readiness_state(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != COMPUTE_READINESS_PAGE_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0, 1);
    Ok(())
}

pub(crate) fn encode_compute_readiness_activation_record(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_RECORD_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u64(out, 0x00, 0x0000_0100_0000_ffff);
    put_u64(out, 0x20, 0x0002_0000_0000_0000);
    put_u64(out, 0x30, 0x0000_0000_ff00_0000);
    Ok(())
}

pub(crate) fn encode_compute_readiness_operand_table(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_OPERAND_TABLE_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    for index in 0..COMPUTE_READINESS_OPERAND_ENTRY_COUNT {
        let address = COMPUTE_FLIST_BUFFER_BASE
            .checked_add(index as u64 * CONTROL_OPERAND_BUFFER_STRIDE)
            .ok_or(InitdataError::AddressOutOfRange)?;
        put_u64(
            out,
            index * CONTROL_OPERAND_ENTRY_STRIDE,
            address | CONTROL_OPERAND_ENTRY_FLAG,
        );
    }
    Ok(())
}

/// Encode the complete control ring prefix consumed during the compute opening.
pub(crate) fn encode_control_opening(
    role: InstanceRole,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != control_opening_size(role) {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);

    match role {
        InstanceRole::Primary => {
            for slot in 0..CONTROL_OPENING_PRIMARY_RECORDS {
                put_u32(
                    out,
                    slot * CONTROL_RECORD_SIZE,
                    CONTROL_OPENING_OPCODE_PRIMARY,
                );
            }
        }
        InstanceRole::Secondary => {
            let initial_records = core::cmp::min(3, CONTROL_OPENING_SECONDARY_RECORDS);
            for slot in 0..initial_records {
                put_u32(
                    out,
                    slot * CONTROL_RECORD_SIZE,
                    CONTROL_OPENING_OPCODE_SECONDARY,
                );
            }
            for slot in initial_records..CONTROL_OPENING_SECONDARY_RECORDS {
                put_u32(
                    out,
                    slot * CONTROL_RECORD_SIZE,
                    CONTROL_OPENING_SECONDARY_CONTINUE_OPCODE,
                );
            }
        }
    }
    Ok(())
}

/// [`encode_control_opening`] for an explicit firmware build. J700/J813 are
/// byte-identical to [`encode_control_opening`]. G18G stages exactly one
/// record per role, 0x15 (primary) or 0x2a (secondary), and never a
/// continuation or register opcode: G18G 0x20 is invalid and 0x22 has not
/// been re-derived.
pub(crate) fn encode_control_opening_for_profile(
    profile: RootProfile,
    role: InstanceRole,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if !profile.is_g18g() {
        return encode_control_opening(role, out);
    }
    if out.len() != CONTROL_RECORD_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0, profile.control_opening_opcode(role));
    Ok(())
}

/// Encode one bare device-control record: `u32 opcode @ +0x00`,
/// `u32 arg @ +0x04`, and 0x38 zero bytes.
///
/// FW_PROVEN (b1 `0x3a74` drain): a record is 0x40 bytes at
/// `ring + index * 0x40`; the drain loads the opcode with `ldr w?, [x24]` and
/// the argument with `ldr w8, [x24, #4]`. Every opcode that carries no further
/// operands leaves the remaining 0x38 bytes untouched, exactly as the opening
/// records this module already stages do.
pub(crate) fn encode_device_control_record(
    opcode: u32,
    arg: u32,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_RECORD_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0x00, opcode);
    put_u32(out, 0x04, arg);
    Ok(())
}

/// Encode one record in the output-positive 65-record runtime control tail.
/// Record zero follows the clean three-record QoS opening.
pub(crate) fn encode_compute_runtime_control_record(
    index: usize,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_RECORD_SIZE || index >= COMPUTE_RUNTIME_CONTROL_RECORDS {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);

    match index {
        0..=53 => encode_control_tick(out, 0, index as u32),
        COMPUTE_RUNTIME_CLASS1_RECORD_INDEX => encode_control_registration(
            out,
            0,
            1,
            54,
            COMPUTE_RUNTIME_SUPPORT_ADDRESS,
            COMPUTE_RUNTIME_CLASS1_TABLE_ADDRESS,
            0x480,
            0x18,
            0,
        ),
        55..=56 => encode_control_tick(out, 0, index as u32 - 1),
        COMPUTE_RUNTIME_CLASS2_RECORD_INDEX => encode_control_registration(
            out,
            0,
            2,
            56,
            COMPUTE_RUNTIME_SUPPORT_ADDRESS,
            COMPUTE_FLIST_RUN_TABLE_ADDRESS,
            0x5c0,
            0x28,
            0,
        ),
        58..=64 => encode_control_tick(out, 0, index as u32 - 2),
        _ => return Err(InitdataError::BufferSize),
    }
    Ok(())
}

fn encode_compute_runtime_support(
    out: &mut [u8],
    control_class: u32,
    low_buffer: u64,
    operand_table: u64,
    resource_class: u64,
    cursor: u32,
    final_kind: u32,
) -> Result<(), InitdataError> {
    if out.len() != COMPUTE_RUNTIME_SUPPORT_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0x00, control_class);
    put_u32(out, 0x10, control_class);
    put_u64(out, 0x14, low_buffer);
    put_u32(out, 0x1c, 0x4_0000);
    let resource = resource_class << 40;
    put_u64(out, 0x20, resource);
    put_u64(out, 0x28, resource);
    put_u64(out, 0x30, operand_table);
    put_u32(out, 0x40, 4);
    put_u32(out, 0x48, cursor);
    put_u64(out, 0x4c, COMPUTE_RUNTIME_STATE_ADDRESS);
    put_u32(out, 0x60, final_kind);
    Ok(())
}

pub(crate) fn encode_compute_runtime_class1_support(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    encode_compute_runtime_support(
        out,
        1,
        COMPUTE_RUNTIME_PAGE_LIST_ADDRESS,
        COMPUTE_RUNTIME_CLASS1_TABLE_ADDRESS,
        0x12,
        0x90,
        2,
    )
}

pub(crate) fn encode_compute_runtime_class2_support(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    encode_compute_runtime_support(out, 2, COMPUTE_FLIST_PAGE_LIST_ADDRESS, COMPUTE_FLIST_RUN_TABLE_ADDRESS, 0x17, 0xb8, 3)
}

pub(crate) fn encode_compute_runtime_class1_table(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != COMPUTE_RUNTIME_CLASS1_TABLE_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    for (index, address) in COMPUTE_RUNTIME_BUFFER_ADDRESSES.into_iter().enumerate() {
        put_u64(
            out,
            index * CONTROL_OPERAND_ENTRY_STRIDE,
            address | CONTROL_OPERAND_ENTRY_FLAG,
        );
    }
    Ok(())
}

pub(crate) fn encode_compute_runtime_page_inventory(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != COMPUTE_RUNTIME_PAGE_LIST_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    let mut index = 0usize;
    for base in COMPUTE_RUNTIME_BUFFER_ADDRESSES {
        for offset in (0..CONTROL_OPERAND_BUFFER_SIZE).step_by(0x1000) {
            put_u64(out, index * 8, base + offset);
            index += 1;
        }
    }
    if index * 8 != COMPUTE_RUNTIME_PAGE_LIST_SIZE {
        return Err(InitdataError::BufferSize);
    }
    Ok(())
}

/// Populate the 21-block pool advertised by the compute UMA descriptor.
/// Like render's initial page list, this contains 4-KiB addresses, not compact
/// run descriptors. Publishing nonzero UMA counters over a zero-filled list
/// makes the allocator return address zero on its first consumption.
pub(crate) fn encode_compute_flist_page_list(out: &mut [u8]) -> Result<(), InitdataError> {
    let pages_per_buffer = (CONTROL_OPERAND_BUFFER_SIZE / 0x1000) as usize;
    let count = COMPUTE_READINESS_OPERAND_ENTRY_COUNT * pages_per_buffer;
    if out.len() < count * 8 {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    for buffer in 0..COMPUTE_READINESS_OPERAND_ENTRY_COUNT {
        let base = COMPUTE_FLIST_BUFFER_BASE + buffer as u64 * CONTROL_OPERAND_BUFFER_STRIDE;
        for page in 0..pages_per_buffer {
            put_u64(out, (buffer * pages_per_buffer + page) * 8, base + page as u64 * 0x1000);
        }
    }
    Ok(())
}

pub(crate) fn encode_compute_dispatch_record(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != COMPUTE_DISPATCH_RECORD_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    // halLoadUMADesc packs the page-list VA in units of 128 bytes below
    // unit_pages << 41. The native literal names render's unrelocated pool;
    // slot 1 must instead name the independently backed compute page list.
    put_u64(out, 0x00, (0x4_0000u64 << 41) | (COMPUTE_FLIST_PAGE_LIST_ADDRESS >> 7));
    put_u64(out, 0x08, 0x1500u64 << 33);
    put_u64(out, 0x10, 0x1500);
    Ok(())
}

/// Encode the generic shared object named by descriptor tails and later
/// compute-control records.
pub(crate) fn encode_control_shared(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != CONTROL_SHARED_OBJECT_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    for (offset, value) in [
        (0x00, 0x0000_0000_0000_0001),
        (0x10, 0x0000_0000_0000_0001),
        (0x18, 0x0004_0000_0000_0070),
        (0x20, 0x0000_1100_0000_0000),
        (0x28, 0x0000_1100_0000_0000),
        (0x30, CONTROL_OPERAND_TABLE_ADDRESS),
        (0x40, 0x0000_0000_0000_0004),
        (0x60, 0x0000_0000_0000_0002),
    ] {
        put_u64(out, offset, value);
    }
    put_u32(out, CONTROL_SHARED_CURSOR_OFFSET, CONTROL_SHARED_CURSOR_BEFORE);
    put_u64(out, 0x4c, CONTROL_SHARED_INNER_ADDRESS);
    Ok(())
}

/// Encode the distinct first-partial support object before opcode 0x20.
pub(crate) fn encode_partial_opening_control_shared(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_SHARED_OBJECT_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    // AGXUSCPrivMemFList::populateInitialFirmwareState seeds this hardware
    // buffer id with -1, but AGXUSCPrivMemPool::prepareLocked replaces it with
    // the allocated id before publishing the first tag-15 record. Linux owns
    // the cold-opening slot zero statically, so encode the post-allocation
    // state that the firmware actually receives.
    put_u32(
        out,
        0x08,
        PARTIAL_OPENING_FREELIST_HARDWARE_BUFFER_ID,
    );
    for (offset, value) in [
        (0x00, 0x0000_0000_0000_0001),
        (0x10, 0x0000_0000_0000_0002),
        (0x18, 0x0004_0000_0000_0070),
        // The complete first-render trace has 17 initial one-MiB blocks:
        // 17 MiB / 4 KiB = 0x1100 pages. Opcode 0x20 appends five blocks.
        (0x20, 0x0000_1100_0000_0000),
        (0x28, 0x0000_1100_0000_0000),
        (0x30, CONTROL_OPERAND_TABLE_ADDRESS),
        (0x40, 0x0000_0000_0000_0004),
        (0x60, 0x0000_0000_0000_0003),
    ] {
        put_u64(out, offset, value);
    }
    put_u32(
        out,
        CONTROL_SHARED_CURSOR_OFFSET,
        PARTIAL_OPENING_CONTROL_SHARED_CURSOR_BEFORE,
    );
    put_u64(
        out,
        0x4c,
        PARTIAL_OPENING_CONTROL_SHARED_INNER_ADDRESS,
    );
    Ok(())
}

/// Encode the live render FList state at the native c0830000 owner address.
///
/// The page-pool fields are the same prepared 17-block plus first-grow state
/// used by the first-partial encoder, but +0 is a pointer to the distinct
/// c01608000 inner object. Valid cold opcode-0x20, both render tag-15 records,
/// and descriptor +0x21ce all name this one object; c0828000 remains the
/// separate compact compute/first-partial state.
pub(crate) fn encode_render_control_shared(out: &mut [u8]) -> Result<(), InitdataError> {
    encode_partial_opening_control_shared(out)?;
    put_u64(out, 0x00, CONTROL_SHARED_INNER_ADDRESS);
    put_u64(out, 0x4c, CONTROL_SHARED_INNER_ADDRESS);
    Ok(())
}

/// Encode a per-pair render FList whose whole pool is populated up front.
///
/// Same object as `encode_render_control_shared`, but the page counts at
/// +0x24/+0x2c and the run cursor at +0x48 cover every block, as
/// AGXUSCPrivMemFList::populateInitialFirmwareState writes them for Apple's
/// 3D pool (0x1600 pages, cursor 0xb0 for 22 blocks). Nothing is left for a
/// first opcode-0x20 grow to append.
pub(crate) fn encode_presized_render_control_shared(
    out: &mut [u8],
    blocks: u32,
) -> Result<(), InitdataError> {
    let pages = blocks.checked_mul(0x100).ok_or(InitdataError::BufferSize)?;
    encode_render_control_shared(out)?;
    put_u64(out, 0x20, u64::from(pages) << 32);
    put_u64(out, 0x28, u64::from(pages) << 32);
    put_u32(out, CONTROL_SHARED_CURSOR_OFFSET, blocks * 8);
    Ok(())
}

/// Encode the hardware-proven compute form of the compact class-2 object.
///
/// This is deliberately separate from the render UMA form above. Compute's
/// successful opening uses 21 compact consecutive run descriptors, a cursor of
/// 0x18 and the original unallocated HardwareBufferID sentinel. Substituting
/// render's 22-block plus asynchronous-grow state makes compute retire without
/// executing.
pub(crate) fn encode_compute_partial_opening_control_shared(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_SHARED_OBJECT_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0x08, u32::MAX);
    for (offset, value) in [
        (0x00, 0x0000_0000_0000_0001),
        (0x10, 0x0000_0000_0000_0002),
        (0x18, 0x0004_0000_0000_0070),
        (0x20, 0x0000_1900_0000_0000),
        (0x28, 0x0000_1900_0000_0000),
        (0x30, COMPUTE_FLIST_RUN_TABLE_ADDRESS),
        (0x40, 0x0000_0000_0000_0004),
        (0x60, 0x0000_0000_0000_0003),
    ] {
        put_u64(out, offset, value);
    }
    put_u64(out, 0x14, COMPUTE_FLIST_PAGE_LIST_ADDRESS);
    put_u32(
        out,
        CONTROL_SHARED_CURSOR_OFFSET,
        COMPUTE_PARTIAL_OPENING_CONTROL_SHARED_CURSOR_BEFORE,
    );
    put_u64(
        out,
        0x4c,
        PARTIAL_OPENING_CONTROL_SHARED_INNER_ADDRESS,
    );
    Ok(())
}

/// Keep the first-partial operand page blank through its opening and first
/// control-done notification. The host publishes the 28-entry form before
/// making either first-work producer visible.
pub(crate) fn encode_partial_opening_operand_table_pre_control(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_OPERAND_TABLE_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    Ok(())
}

pub(crate) fn encode_partial_opening_operand_table_post_control(
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != CONTROL_OPERAND_TABLE_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    for index in 0..PARTIAL_OPENING_CONTROL_OPERAND_ENTRY_COUNT {
        let address = CONTROL_OPERAND_BUFFER_BASE
            .checked_add(index as u64 * CONTROL_OPERAND_BUFFER_STRIDE)
            .ok_or(InitdataError::AddressOutOfRange)?;
        put_u64(
            out,
            index * CONTROL_OPERAND_ENTRY_STRIDE,
            address | CONTROL_OPERAND_ENTRY_FLAG,
        );
    }
    Ok(())
}

/// Encode the 22 flagged buffer pointers used by the later compute tail.
pub(crate) fn encode_control_operand_table(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != CONTROL_OPERAND_TABLE_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    for index in 0..CONTROL_OPERAND_ENTRY_COUNT {
        let address = COMPUTE_FLIST_BUFFER_BASE
            .checked_add(index as u64 * CONTROL_OPERAND_BUFFER_STRIDE)
            .ok_or(InitdataError::AddressOutOfRange)?;
        put_u64(
            out,
            index * CONTROL_OPERAND_ENTRY_STRIDE,
            address | CONTROL_OPERAND_ENTRY_FLAG,
        );
    }
    Ok(())
}

/// Bundle offsets of the five views named at main `+0x254`. They are views of
/// the common bundle, not independent objects: view 0 overlaps the tail of
/// the hardware-data object, and views 3 and 4 overlap each other by all but
/// 0x80 bytes.
pub(crate) const BUNDLE_VIEW_OFFSETS: [usize; 5] = [0x2740, 0x3380, 0x4400, 0xbc80, 0xbd00];
/// Logical extents of the five views.
pub(crate) const BUNDLE_VIEW_EXTENTS: [usize; 5] = [0x18c0, 0x0c80, 0x3c00, 0x3380, 0x3300];

/// The context-0 mappings of the PB descriptor and UMA page-pool descriptor
/// tables published at main `+0x2d8` and `+0x2e8`, respectively. The matching
/// firmware-high mappings are published at `+0x2e0` and `+0x2f0`.
pub(crate) const REGION_VIEW_LOW_ADDRS: [u64; 2] = [0x70_0183_8000, 0x70_0184_0000];

// --- Channel table ---------------------------------------------------------------

pub(crate) const CHANNEL_TABLE_ENTRIES: usize = 17;
pub(crate) const CHANNEL_ENTRY_SIZE: usize = 0x20;
/// Channel entry layout: three *separate* 32-bit state-counter addresses at
/// `+0x00`/`+0x08`/`+0x10` (the counters themselves sit at 0x10 spacing in
/// their state block — do not collapse them into one state object), then the
/// ring base at `+0x18`.
pub(crate) const CHANNEL_ENTRY_STATE_COUNT: usize = 3;
pub(crate) const CHANNEL_STATE_SPACING: usize = 0x10;
pub(crate) const CHANNEL_STATE_STRIDE: usize = 0x40;
/// For a work channel, state 0 is the consumer and state 2 the producer.
pub(crate) const CHANNEL_STATE_CONSUMER: usize = 0;
pub(crate) const CHANNEL_STATE_PRODUCER: usize = 2;
/// Entries 0..11 are ordered TA_0, 3D_0, CL_0, TA_1, ... CL_3.
pub(crate) const WORK_CHANNEL_COUNT: usize = 12;
/// Entry 12 is device control. It continues the compact control-state grid
/// ([`WORK_STATE_GRID_OFFSETS`] has 13 entries, one more than there are work
/// channels), and its ring is embedded at main `+0x4c0`. The M5-derived model
/// refused this index, claiming a "cursor pair at +0x1a0/+0x1a8"; that was a
/// misread — those offsets are exactly this entry's state-counter addresses 0
/// and 1.
pub(crate) const CONTROL_CHANNEL_INDEX: usize = 12;
/// Entries 13 and 14 are report channels with split state objects: their
/// state addresses alias off status A rather than the compact grid, and the
/// firmware peer of each host-owned counter is at `+0x20` within its object.
pub(crate) const REPORT_CHANNEL_INDICES: [usize; 2] = [13, 14];
/// Entry 15 is partial: only its first state address, no ring. Entry 16 is
/// empty. Filling them like the rest declares channels that do not exist.
pub(crate) const PARTIAL_CHANNEL_INDEX: usize = 15;

/// Offsets of the channel state blocks (channels 0..=12) inside the compact
/// state grid. Native state is *not* in channel-table order: within each
/// workload group the work queues sit at descending offsets, and the control
/// channel's block follows at `+0x300`. Preserving these address identities is
/// part of the descriptor ABI.
pub(crate) const WORK_STATE_GRID_OFFSETS: [usize; 13] = [
    0x0c0, 0x1c0, 0x2c0, // TA_0, 3D_0, CL_0
    0x080, 0x180, 0x280, // TA_1, 3D_1, CL_1
    0x040, 0x140, 0x240, // TA_2, 3D_2, CL_2
    0x000, 0x100, 0x200, // TA_3, 3D_3, CL_3
    0x300, // device control
];
/// The primary root's status-B pointer is the word immediately after that
/// compact grid: `status_b == state_grid + 0x340` in the native layout.
pub(crate) const PRIMARY_STATUS_B_STATE_GRID_OFFSET: usize = 0x340;
/// The primary status-B object runs from there to status A: `0xeb00` bytes.
pub(crate) const PRIMARY_STATUS_B_OBJECT_SIZE: usize = 0xeb00;

/// State-counter addresses of report channels 13/14, as offsets from status A.
pub(crate) const REPORT_STATE_STATUS_A_OFFSETS: [[usize; 3]; 2] =
    [[0x00040, 0x002c0, 0x00080], [0x00240, 0xa6ac0, 0x00280]];
/// Ring bases of report channels 13/14, as offsets from status A.
pub(crate) const REPORT_RING_STATUS_A_OFFSETS: [usize; 2] = [0x04ac0, 0xafac0];
/// Entry 15's sole state address, as an offset from status A.
pub(crate) const PARTIAL_STATE_STATUS_A_OFFSET: usize = 0x2d2c0;

/// Native bundle offsets of the twelve work rings (0x1800 each), in
/// channel-table order. These extend past [`HW_DATA_BUNDLE_MIN_ALLOC`]; a host
/// using this placement sizes the cluster [`NATIVE_SHARED_CLUSTER_SIZE`].
pub(crate) const WORK_RING_BUNDLE_OFFSETS: [usize; 12] = [
    0x10dc0, 0x16dc0, 0x1cdc0, 0x0f5c0, 0x155c0, 0x1b5c0, 0x0ddc0, 0x13dc0, 0x19dc0, 0x0c5c0,
    0x125c0, 0x185c0,
];
/// Native bundle offsets of the two main configuration objects.
pub(crate) const NATIVE_PRIMARY_MAIN_BUNDLE_OFFSET: usize = 0x1e5c0;
pub(crate) const NATIVE_SECONDARY_MAIN_BUNDLE_OFFSET: usize = 0x22a80;

// --- Hardware-data object ---------------------------------------------------------

/// Register-mapping array: 53 slots of 0x28 bytes at `+0x640`. Slots may also
/// carry only the flag at `+0x20` (declared but unmapped). Register windows
/// come from the ADT; do not replace ADT-defined sizes or addresses.
/// The five qwords `AGXFirmware::initFirmwareData` writes at the head of the
/// hardware-data object: `{range-3 base, range-3 size, GART window base, GART
/// window base, reserved top-of-range-5 page}`.
pub(crate) const HW_FIRMWARE_RANGE_HEAD: &[(usize, u64)] = &[
    (0x00, 0x0000_006f_0000_0000),
    (0x08, 0x0000_0000_ffc0_0000),
    (0x10, 0x0000_0010_0000_0000),
    (0x18, 0x0000_0010_0000_0000),
    (0x20, 0x0000_02ff_ffff_8000),
];

pub(crate) const HW_REGISTER_MAP_OFFSET: usize = 0x640;
pub(crate) const HW_REGISTER_MAP_ENTRY_SIZE: usize = 0x28;
pub(crate) const HW_REGISTER_MAP_SLOTS: usize = 53;
/// FW device-VA aperture the register windows live in.
pub(crate) const HW_REGISTER_APERTURE: Range<u64> = 0xffff_fc21_8000_0000..0xffff_fc21_9000_0000;

/// `+0xe90`: u32 chip identifier.
pub(crate) const HW_CHIP_ID_OFFSET: usize = 0xe90;
pub(crate) const HW_CHIP_ID_T8140: u32 = 0x8140;

// The acceptance block at +0x258e. Bisecting by zeroing showed every *other*
// unexplained hardware-data byte can be zero, so this is the part that has to
// be right for the descriptor to be accepted. It decodes as the work-channel
// count, two {0,1,1,1} flag groups, one 0xffffffff "unassigned" word per work
// channel, and three trailing ones.
pub(crate) const HW_ACCEPT_CHANNEL_COUNT: usize = 0x258e; // u16 = 12
pub(crate) const HW_ACCEPT_FLAG_GROUPS: [usize; 2] = [0x2590, 0x25a0];
pub(crate) const HW_ACCEPT_FLAG_PATTERN: [u32; 4] = [0, 1, 1, 1];
pub(crate) const HW_ACCEPT_UNASSIGNED_TABLE: usize = 0x25b4; // 12 x u32 0xffffffff
pub(crate) const HW_ACCEPT_TRAILING_ONES: [usize; 3] = [0x25f4, 0x2600, 0x2608];
/// `+0x2630` is firmware state, not host configuration (the one byte that
/// differed between a pre-start and a running capture). It must start zero.
pub(crate) const HW_FIRMWARE_STATE_OFFSET: usize = 0x2630;

/// Four inline G17P KSM completion descriptors at hardware-data `+0x2614`.
/// Only ordinals 0 and 2 are present. Each names low/high mappings of one
/// retained `0x800 * 0x40 == 0x20000` backing; ordinals 1 and 3 stay zero.
pub(crate) const G17P_KSM_COMPLETION_TABLE_OFFSET: usize = 0x2614;
pub(crate) const G17P_KSM_COMPLETION_DESCRIPTOR_STRIDE: usize = 0x20;
pub(crate) const G17P_KSM_COMPLETION_PRESENT_ORDINALS: [u32; 2] = [0, 2];
pub(crate) const G17P_KSM_COMPLETION_CAPACITY: u32 = 0x800;
pub(crate) const G17P_KSM_COMPLETION_ENTRY_SIZE: u32 = 0x40;
pub(crate) const G17P_KSM_COMPLETION_BACKING_SIZE: usize =
    G17P_KSM_COMPLETION_CAPACITY as usize * G17P_KSM_COMPLETION_ENTRY_SIZE as usize;
pub(crate) const G17P_KSM_COMPLETION_LOW_VAS: [u64; 2] =
    [0x70_03f0_0000, 0x70_03f2_8000];
pub(crate) const G17P_KSM_COMPLETION_HIGH_BUNDLE_OFFSETS: [usize; 2] = [0x48000, 0x70000];
const G17P_KSM_COMPLETION_TABLE_PREFIX_OFFSET: usize = 0x2610;
const G17P_KSM_COMPLETION_TABLE_PREFIX: u32 = 0x100;

/// Two GPU mappings of one present completion-queue backing.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct G17PKsmCompletionAliases {
    pub(crate) low: u64,
    pub(crate) high: u64,
}

pub(crate) const M5_25G76_COMPLETION_QUEUE_COUNT: usize = 4;
pub(crate) const M5_25G76_COMPLETION_BACKING_SIZE: usize = 0x800 * 0x40;

/// M5 publishes all four descriptors at hardware-data +0x2614, with ordinals
/// 0,1,2,3. Each names low/high mappings of its own retained 128-KiB backing.
/// The two native pre-handoff captures agree on every descriptor byte.
/// Construction only: this does not allocate, map, publish, or poll a queue.
pub(crate) fn encode_m5_25g76_completion_table(
    aliases: [G17PKsmCompletionAliases; M5_25G76_COMPLETION_QUEUE_COUNT],
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != M5_25G76_COMPLETION_QUEUE_COUNT * 0x20 {
        return Err(InitdataError::BufferSize);
    }
    let extent = M5_25G76_COMPLETION_BACKING_SIZE as u64;
    // Validate every range before mutating the output. Different queues must
    // not alias each other in either address space.
    for (i, alias) in aliases.iter().enumerate() {
        for address in [alias.low, alias.high] {
            required_address(address)?;
            let end = address.checked_add(extent).ok_or(InitdataError::AddressOutOfRange)?;
            required_address(end - 1)?;
        }
        for prior in &aliases[..i] {
            if (alias.low < prior.low + extent && prior.low < alias.low + extent)
                || (alias.high < prior.high + extent && prior.high < alias.high + extent)
            {
                return Err(InitdataError::PairAliasedObject);
            }
        }
    }
    out.fill(0);
    for (ordinal, alias) in aliases.iter().enumerate() {
        let offset = ordinal * 0x20;
        put_u64(out, offset, alias.low);
        put_u64(out, offset + 8, alias.high);
        put_u32(out, offset + 0x10, 0x800);
        put_u32(out, offset + 0x14, 0x40);
        put_u32(out, offset + 0x18, ordinal as u32);
    }
    Ok(())
}

// Performance-table geometry (values deliberately not encoded; see
// [`PERF_TABLE_VALUES_ENCODED`]). Two groups with identical internal layout,
// one per 11-entry ladder: frequency ladder, then a core-voltage column 0x40
// on, then a memory-voltage column 0x440 on, stored as 0x40-stride per-state
// blocks. Group 0 repeats each per-state value across 16 words; group 1
// stores it once.
pub(crate) const HW_PERF_GROUP_BASES: [usize; 2] = [0xfc8, 0x1cdc];
pub(crate) const HW_PERF_GROUP_REPEATS: [usize; 2] = [16, 1];
pub(crate) const HW_PERF_LADDER_ENTRIES: usize = 11;
pub(crate) const HW_PERF_VOLTAGE_DELTA: usize = 0x40;
pub(crate) const HW_PERF_MEMORY_VOLTAGE_DELTA: usize = 0x440;
pub(crate) const HW_PERF_STATE_BLOCK_STRIDE: usize = 0x40;
pub(crate) const HW_PERF_FREQ_LADDER_B: usize = 0x1808;
pub(crate) const HW_PERF_SCALE_LADDER_B: usize = 0x1848;
pub(crate) const HW_PERF_RELATIVE_LADDER_A: usize = 0x18c8;
pub(crate) const HW_PERF_RELATIVE_LADDER_B: usize = 0x1908;
pub(crate) const HW_PERF_INDEX_MAP_A: usize = 0x19c8;
pub(crate) const HW_PERF_INDEX_MAP_B: usize = 0x1a08;
const HW_PERF_FREQ_A: [u32; HW_PERF_LADDER_ENTRIES] =
    [0, 338, 492, 618, 796, 928, 1056, 1170, 1278, 1338, 1470];
const HW_PERF_FREQ_B: [u32; HW_PERF_LADDER_ENTRIES] =
    [0, 338, 492, 618, 796, 928, 952, 1053, 1152, 1204, 1326];
const HW_PERF_SCALE_B: [u32; HW_PERF_LADDER_ENTRIES] = [1_065_520_988; 11];
/// Relative-performance ladder A. Native's live block reads
/// 0f 16 1d 2a 38 4a 56 at +0x18cc (states 1..7) = 15 22 29 42 56 74 86,
/// against our 16 23 30 43 57 74 85 -- off by one nearly everywhere. States
/// 8..10 are not covered by that read and keep our values.
const HW_PERF_RELATIVE_A_VALUES: [u32; HW_PERF_LADDER_ENTRIES] =
    [0, 15, 22, 29, 42, 56, 74, 86, 90, 92, 100];
const HW_PERF_RELATIVE_B_VALUES: [u32; HW_PERF_LADDER_ENTRIES] =
    [0, 0, 13, 24, 40, 52, 63, 73, 83, 88, 100];
const HW_PERF_INDEX_A_VALUES: [u32; HW_PERF_LADDER_ENTRIES] =
    [0, 1, 2, 3, 4, 5, 7, 9, 11, 13, 15];
const HW_PERF_INDEX_B_VALUES: [u32; HW_PERF_LADDER_ENTRIES] =
    [0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 14];
/// Per-perf-state GPU core voltage, in mV.
///
/// MEASURED against native's live hardware-data block (25G83, hwdata.d reading
/// *(AGXArmFirmware+0x2b8)): native carries
///     605 630 650 695 745 800 845 885 905 975
/// for states 1..10, and this driver derived
///     595 615 635 685 730 780 815 865 885 955
/// -- systematically LOW at every state, by 10 to 30 mV. Both machines run the
/// same ADT, so this is our derivation and not chip binning.
///
/// This is not cosmetic. The register this tree calls "GPC core power",
/// sgx+0xe01000 & 0xf, is G16::AGXFenderCommon::getGPCPerformanceState -- the
/// GPU PERFORMANCE state, where 0 means off. The firmware selects a state from
/// exactly these tables and raises the GPC to it, so a table it judges invalid
/// or below its floor is a direct mechanism for "firmware runs, tables loaded,
/// GPC stays 0, nothing ever executes".
const HW_PERF_CORE_VOLTAGE: [u32; HW_PERF_LADDER_ENTRIES] =
    [125, 605, 630, 650, 695, 745, 800, 845, 885, 905, 975];
/// Per-perf-state SRAM/memory-rail voltage, in mV.
///
/// Same defect as the core table: our tail tracked our own (wrong) core values.
/// Native's live block carries 765 x6 then 800 845 885 905 975, which is the
/// ADT perf-states-sram column mapped through HW_PERF_INDEX_A. The correct
/// derivation is memory[i] = ADT perf-states-sram[INDEX_A[i]].volt, not "the
/// memory tail tracks core".
const HW_PERF_MEMORY_VOLTAGE: [u32; HW_PERF_LADDER_ENTRIES] =
    [765, 765, 765, 765, 765, 765, 800, 845, 885, 905, 975];

// --- Region C ---------------------------------------------------------------------

/// Byte immediately preceding the unaligned host power-assert counter.
pub(crate) const REGION_C_REQUIRED_OFFSET: usize = 0xe50;
pub(crate) const REGION_C_REQUIRED_VALUE: u32 = 0;
/// Unaligned host-owned GPU-core power-assert counter.
///
/// FW_PROVEN (b1 `0x3aa8`/`0x3b20`): the firmware reads this **unaligned** u32
/// and compares it with its own tally `status_b[0xe438]`, which it increments
/// once per drained record whose opcode flag byte (`__const 0x45430 + opcode`)
/// is non-zero. When the GPU is unpowered and the two disagree, `0x3b2c` calls
/// `0x21960` and powers the cores on before draining. Equal values mean "no
/// unaccounted power-asserting work", so a value pinned forever at 1 — the old
/// static initializer — can never trigger that arm after the first flagged
/// record.
///
/// Was `0xe55`, which is off by four: that offset is read nowhere in b1.
/// The flag byte is `0` (i.e. no power assert) for opcodes `0x00`, `0x0e`,
/// `0x15`, `0x16`, `0x1b`, `0x22`, `0x2a`, `0x2e` and `0x3a..=0x3f`; note that
/// includes `0x2e`, contradicting the older comment here that named it as a
/// core-power claimant.
pub(crate) const REGION_C_GPU_CORE_PWR_ASSERT_COUNTER_OFFSET: usize = 0xe51;
/// Native cold value before the host publishes its first flag-1 control.
pub(crate) const REGION_C_GPU_CORE_PWR_ASSERT_COUNT: u32 = 0;

/// Whether a device-control opcode advances the host GPU-core power-assert
/// counter. The 64-entry table is exhaustive in B1; values outside it are not
/// valid device-control opcodes.
pub(crate) const fn device_control_asserts_gpu_power(opcode: u32) -> Option<bool> {
    if opcode > 0x3f {
        return None;
    }
    Some(!matches!(
        opcode,
        0x00 | 0x0e | 0x15 | 0x16 | 0x1b | 0x22 | 0x2a | 0x2e | 0x3a..=0x3f
    ))
}

// --- Status blocks -----------------------------------------------------------------

/// Status block `+0x04` holds u32 1 pre-init. `+0x10`/`+0x14` are *post-ack
/// firmware state* and must be zero at handoff.
pub(crate) const STATUS_BLOCK_ONE_OFFSET: usize = 0x04;
pub(crate) const STATUS_BLOCK_POST_ACK_OFFSETS: [usize; 2] = [0x10, 0x14];
pub(crate) const PRIMARY_STATUS_B_FWCTL_STATE: usize = 0x48e0;
pub(crate) const PRIMARY_STATUS_B_FWCTL_RING: usize = 0x48e8;
pub(crate) const PRIMARY_STATUS_B_CONFIG_HEADER: usize = 0xe434;
pub(crate) const PRIMARY_STATUS_B_CONFIG_OFFSET: usize = 0xe440;

// --- Instance separation -----------------------------------------------------------

/// The secondary root sits exactly this far above the primary root; the
/// secondary's initialisation message carries the primary's address plus this
/// delta in the same field, so the pair is not placed freely.
pub(crate) const SECONDARY_ROOT_DELTA: u64 = 0x8000;
/// Each instance maps the ADT shared region from its own half: the secondary
/// maps it from base plus this delta, and walks a top table at the same
/// offset into its half. Mappings installed only in the primary's tables
/// leave the secondary unable to translate anything it is handed.
pub(crate) const SECONDARY_SHARED_REGION_DELTA: u64 = 0x40000;

/// Physical firmware instance encoded at root `+0x28`.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum InstanceRole {
    /// `gfx-asc`: scheduler, device control, and all work channels.
    Primary = 0,
    /// `gfx1-asc`: control-only peer. Linux starts it after primary RTKit is ready.
    Secondary = 1,
}

pub(crate) const fn root_size(role: InstanceRole) -> usize {
    match role {
        InstanceRole::Primary => ROOT_SIZE_PRIMARY,
        InstanceRole::Secondary => ROOT_SIZE_SECONDARY,
    }
}

/// GPU virtual addresses of the objects both instances share.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct SharedAddresses {
    /// Shared 16 KiB region A (root `+0x08`).
    pub(crate) region_a: u64,
    /// Shared region C (root `+0x20`), 0x1000 bytes.
    pub(crate) region_c: u64,
    /// Base of the contiguous hardware-data bundle. The hardware-data object
    /// sits at the base; the five main-config views, the repeated region, and
    /// the secondary's `+0x471` pointer are all derived offsets into it.
    pub(crate) hw_data_bundle: u64,
    /// Size of that one allocation; must be at least
    /// [`HW_DATA_BUNDLE_MIN_ALLOC`].
    pub(crate) hw_data_bundle_alloc: usize,
}

/// GPU virtual addresses of one instance's private objects.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct InstanceAddresses {
    /// This instance's root; its address rides the endpoint-0x20 doorbell.
    pub(crate) root: u64,
    pub(crate) main_config: u64,
    pub(crate) status_a: u64,
    /// Required for the primary; must be zero for the secondary (whose root
    /// still has the field, written zero).
    pub(crate) status_b: u64,
    /// Root `+0xb8`/`+0xc0`. Required for the secondary; the primary root is
    /// only 0xb8 bytes and cannot even hold them, so they must be zero there.
    pub(crate) secondary_extras: [u64; 2],
}

/// One channel-table entry: three state-counter addresses, then the ring base.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct ChannelEntry {
    pub(crate) states: [u64; 3],
    pub(crate) ring: u64,
}

impl ChannelEntry {
    pub(crate) const EMPTY: ChannelEntry = ChannelEntry {
        states: [0; 3],
        ring: 0,
    };
}

/// The primary's global region views at main `+0x2d0`: a globally retained,
/// initially zero HWDSID accounting table (256 pairs of u32), followed by
/// the high halves of two low/high alias pairs (the low halves
/// are the fixed context-0 addresses in
/// [`REGION_VIEW_LOW_ADDRS`]). Each pair must map one physical page from both
/// address spaces — an invariant of the UAT mapping layer that cannot be
/// checked from the addresses alone.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct PrimaryRegionViews {
    pub(crate) sentinel_high: u64,
    pub(crate) alias_high: [u64; 2],
}

/// One register-window declaration for the hardware-data mapping array.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct RegisterWindow {
    pub(crate) slot: usize,
    pub(crate) phys: u64,
    pub(crate) device_va: u64,
    /// Byte count; written at both `+0x10` and `+0x14`. Not always a granule
    /// multiple.
    pub(crate) size: u32,
    /// Low 24 bits of the physical address for accelerator-internal windows,
    /// else zero.
    pub(crate) unk_18: u64,
    pub(crate) flag: u32,
}

/// M5 25G76 descriptor fields +0x10 and +0x14 are independent. In slot 39
/// native publishes 0x10000 and 0x8000 respectively. The older RegisterWindow
/// encoder duplicates one size and must not serialize this target's table.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct M5RegisterWindow {
    pub(crate) slot: usize,
    pub(crate) phys: u64,
    pub(crate) device_va: u64,
    pub(crate) size_10: u32,
    pub(crate) size_14: u32,
    pub(crate) unk_18: u64,
    pub(crate) flag: u32,
}

impl M5RegisterWindow {
    /// Actual M5 PTE walks cover size_10 rounded to 16 KiB, including the
    /// four pages of slot 39. size_14 is independently serialized firmware
    /// metadata and must not truncate that mapping to two pages.
    pub(crate) fn mapped_pages(&self) -> Result<(u64, u64, u64), InitdataError> {
        let mask = UAT_PAGE_SIZE as u64 - 1;
        if self.size_10 == 0 || self.phys & mask != self.device_va & mask {
            return Err(InitdataError::RegisterWindow);
        }
        let physical = self.phys & !mask;
        let virtual_address = self.device_va & !mask;
        let end = self.device_va.checked_add(self.size_10 as u64)
            .and_then(|value| value.checked_add(mask))
            .ok_or(InitdataError::AddressOutOfRange)? & !mask;
        let span = end.checked_sub(virtual_address).ok_or(InitdataError::AddressOutOfRange)?;
        physical.checked_add(span).ok_or(InitdataError::AddressOutOfRange)?;
        Ok((physical, virtual_address, span))
    }
}

/// Source-constructed register descriptors from both identical pre-handoff
/// J813/25G76 captures. These are firmware virtual mappings, not a request to
/// access or power the devices. No runtime currently consumes this profile.
pub(crate) const M5_25G76_REGISTER_WINDOWS: [M5RegisterWindow; 22] = {
    const fn w(slot: usize, phys: u64, va: u64, size_10: u32, size_14: u32,
               unk_18: u64, flag: u32) -> M5RegisterWindow {
        M5RegisterWindow { slot, phys, device_va: 0xffff_fc21_8000_0000 + va,
                           size_10, size_14, unk_18, flag }
    }
    [
        w(0,  0x381014000, 0x000000, 0x04000, 0x04000, 0, 2),
        w(3,  0x220104000, 0x008000, 0x18000, 0x18000, 0, 2),
        w(8,  0x3a1350000, 0x028000, 0x04000, 0x04000, 0, 2),
        w(9,  0x3803d0000, 0x030000, 0x04000, 0x04000, 0, 2),
        w(10, 0x3803c0000, 0x038000, 0x04000, 0x04000, 0, 0),
        w(12, 0x42165c000, 0x040000, 0x04000, 0x04000, 0, 2),
        w(14, 0x380280000, 0x048000, 0x08000, 0x08000, 0, 0),
        w(15, 0x38c840000, 0x058000, 0x24000, 0x24000, 0, 2),
        w(17, 0x580000000, 0x080000, 0x21400, 0x21400, 0, 2),
        w(22, 0x581000000, 0x0a8000, 0x08000, 0x08000, 0, 2),
        w(24, 0x580ec0000, 0x0b8000, 0x10000, 0x10000, 0, 2),
        w(26, 0x580d04000, 0x0d0000, 0x08000, 0x08000, 0xd04000, 2),
        w(27, 0x580d0d000, 0x0e1000, 0x01000, 0x01000, 0xd0d000, 2),
        w(28, 0x580d50000, 0x0e8000, 0x10000, 0x10000, 0xd50000, 2),
        w(29, 0x580d10000, 0x100000, 0x04000, 0x04000, 0xd10000, 2),
        w(31, 0x580d40000, 0x108000, 0x04000, 0x04000, 0xd40000, 2),
        w(32, 0x580d60000, 0x110000, 0x08000, 0x08000, 0xd60000, 2),
        w(35, 0x580e00000, 0x120000, 0x04000, 0x04000, 0xe00000, 2),
        w(39, 0x580e08000, 0x128000, 0x10000, 0x08000, 0, 2),
        w(41, 0x580e33800, 0x143800, 0x04000, 0x04000, 0, 2),
        w(42, 0x580e35000, 0x151000, 0x04000, 0x04000, 0, 2),
        w(50, 0x580014400, 0x160400, 0x00400, 0x00400, 0x14400, 0),
    ]
};

pub(crate) fn encode_m5_25g76_register_map(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != HW_REGISTER_MAP_SLOTS * HW_REGISTER_MAP_ENTRY_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    for window in M5_25G76_REGISTER_WINDOWS {
        let base = window.slot * HW_REGISTER_MAP_ENTRY_SIZE;
        put_u64(out, base, window.phys);
        put_u64(out, base + 8, window.device_va);
        put_u32(out, base + 0x10, window.size_10);
        put_u32(out, base + 0x14, window.size_14);
        put_u64(out, base + 0x18, window.unk_18);
        put_u32(out, base + 0x20, window.flag);
    }
    for slot in [2, 5, 6, 7, 30, 33, 34, 37, 38, 40, 43, 46, 47, 48, 49, 52] {
        put_u32(out, slot * HW_REGISTER_MAP_ENTRY_SIZE + 0x20, 2);
    }
    Ok(())
}

pub(crate) const M5_25G76_HW_DATA_SIZE: usize = 0x2708;

/// Native sharedDataDescriptor sizes loaded at AGXFirmware::allocFirmwareData
/// +0x8b8 onwards from 25G76 __const 0xfffffe0007139820. The allocator rounds
/// each next object to 64 bytes. These are independent allocations, not the
/// overlapping views used while investigating T8140.
pub(crate) const M5_25G76_SHARED_ALLOCATIONS: [(usize, usize); 7] = [
    (0x0000, 0x2708), (0x2740, 0x0c18), (0x3380, 0x1048),
    (0x4400, 0x0e10), (0x5240, 0x6a30), (0xbc80, 0x0068),
    (0xbd00, 0x0800),
];
pub(crate) const M5_25G76_SHARED_PREFIX_SIZE: usize = 0xc500;
pub(crate) const M5_25G76_AON_PTD_SIZE: usize = 0xe4;

/// J813/25G76 AON PTD backing, captured twice before each role's handoff and
/// unchanged after a passing Metal workload. The three recorded words are
/// scoped to this platform's telemetry description; they are not generic
/// M5 constants. The allocation and GPU binding must remain caller-owned.
pub(crate) fn encode_m5_25g76_aon_ptd(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != M5_25G76_AON_PTD_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, 0, 0x6730_5452);
    put_u32(out, 4, 0x001d_0001);
    put_u32(out, 8, 0x0000_1401);
    Ok(())
}

/// Native independent primary status/config allocation sizes, from the
/// 25G76 descriptors at 0xfffffe0007139880 / 0xfffffe0007139888.
pub(crate) const M5_25G76_STATUS_CONFIG_SIZE: usize = 0x6a8;
const M5_25G76_STATUS_CONFIG_WORDS: &[(usize, u32)] = &[
    (0x010, 0x0000001f), (0x014, 0x3e0a3d71), (0x018, 0x401ae148),
    (0x01c, 0x00000028), (0x020, 0x00000001), (0x0c8, 0x00002af8),
    (0x0cc, 0x00001f40), (0x0d8, 0x41980000), (0x0dc, 0x3d0f5c28),
    (0x0f0, 0x000026ac), (0x0f4, 0x000000c8), (0x108, 0x42c80000),
    (0x10c, 0x43c80000), (0x110, 0x000000c8), (0x128, 0x00000001),
    (0x12c, 0x00005eb0), (0x130, 0x3e800000), (0x134, 0x3f99999a),
    (0x158, 0x0000000e), (0x1c0, 0x00000001), (0x1c4, 0x00000001),
    (0x1c8, 0x00000002), (0x1cc, 0x00000001), (0x1d0, 0x00000001),
    (0x1d4, 0x00000001), (0x1d8, 0x00000001), (0x588, 0x00000001),
    (0x58c, 0x0000000a), (0x590, 0x00000dac), (0x594, 0x000003e8),
    (0x598, 0x00000bb8), (0x59c, 0x000002bc), (0x5cc, 0x00000001),
    (0x5d0, 0x00000004), (0x5dc, 0x00001f40), (0x5e0, 0x000000c8),
    (0x5e4, 0x00000fa0), (0x5e8, 0x0000012c), (0x5ec, 0x000007d0),
    (0x5f0, 0x0000012c), (0x5f4, 0x000003e8), (0x5f8, 0x0000012c),
    (0x600, 0x00000001), (0x604, 0x41700000), (0x608, 0x40a00000),
    (0x60c, 0x00000020), (0x610, 0x000004b0), (0x620, 0x0000044c),
];

/// Build the primary status object before either firmware role is started.
/// Secondary root+0xb8 aliases status+0xe440. GFX1 copies that pointer to
/// firmware global 0x176248 at text+0x211d4 and later writes config+0x670,
/// +0x678 and +0x680 at text+0x1af94..0x1afc4. Those are firmware-owned
/// runtime state, not platform constants, and must start at zero even when
/// a primary pre-handoff capture (taken after secondary startup) has values.
pub(crate) fn encode_m5_25g76_primary_status_b(
    fwctl_state: u64,
    fwctl_ring: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != PRIMARY_STATUS_B_OBJECT_SIZE {
        return Err(InitdataError::BufferSize);
    }
    required_address(fwctl_state)?;
    required_address(fwctl_ring)?;
    out.fill(0);
    encode_status_block(&mut out[..STATUS_BLOCK_SIZE])?;
    put_u64(out, PRIMARY_STATUS_B_FWCTL_STATE, fwctl_state);
    put_u64(out, PRIMARY_STATUS_B_FWCTL_RING, fwctl_ring);
    put_u32(out, PRIMARY_STATUS_B_CONFIG_HEADER, 1);
    for &(offset, value) in M5_25G76_STATUS_CONFIG_WORDS {
        put_u32(out, PRIMARY_STATUS_B_CONFIG_OFFSET + offset, value);
    }
    Ok(())
}


/// Shared private arena observed before primary initialization on J813.
/// The status and secondary-grid pointers name overlapping views of this
/// one backing. This extent is a measured mapped arena, not a claim about
/// the sizes of the native host's individual suballocations.
pub(crate) const M5_25G76_PRIVATE_CLUSTER_SIZE: usize = 0x178000;

pub(crate) fn encode_m5_25g76_private_cluster(
    fwctl_state: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != M5_25G76_PRIVATE_CLUSTER_SIZE {
        return Err(InitdataError::BufferSize);
    }
    required_address(fwctl_state)?;
    let fwctl_ring = fwctl_state.checked_add(0x40).ok_or(InitdataError::AddressOutOfRange)?;
    required_address(fwctl_ring)?;
    out.fill(0);
    encode_m5_25g76_primary_status_b(fwctl_state, fwctl_ring, &mut out[0x340..0xee40])?;
    for offset in [0xee40, 0xc3100] {
        encode_status_block(&mut out[offset..offset + STATUS_BLOCK_SIZE])?;
    }
    encode_secondary_root_extra_1_for_profile(RootProfile::J813G17G25G76,
        &mut out[0x177400..0x177480])?;
    // The main-config encoders carry one measured opening record (0x16 /
    // 0x2a). Publish exactly one producer entry for each control channel.
    // All work-channel and report counters remain zero.
    put_u32(out, 0x320, 1);
    put_u32(out, 0x1773e0, 1);
    Ok(())
}

/// Caller-owned bindings in the power/performance policy allocation.
/// The host stores mach_absolute_time() at policy+0x651c. Supply the live
/// platform counter in its native timebase, never a captured timestamp.
#[derive(Clone, Copy)]
pub(crate) struct M5PolicyBindings {
    pub state: u64,
    pub aon_ptd: u64,
    pub boot_timestamp: u64,
}

/// Pre-handoff J813/25G76 scalars, identical across two cold boots after
/// removing owned pointers and the native boot timestamp. The repeated
/// 512-entry u16 table is encoded separately. Calibration fields whose
/// meanings remain unknown are scoped to this measured ten-core target.
const M5_25G76_POLICY_WORDS: &[(usize, u32)] = &[
    (0x3d98, 0xffffffff), (0x3db0, 0xffffffff), (0x5244, 0x00030d40),
    (0x5248, 0x00030d40), (0x5250, 0x00000004), (0x5254, 0x3f800000),
    (0x526c, 0x00000001), (0x5270, 0x00000001), (0x5280, 0x00000064),
    (0x5284, 0x00000001), (0x5288, 0x000004b0), (0x5290, 0x00000064),
    (0x52a0, 0x00000004), (0x52a4, 0x3f800000), (0x52ac, 0x00000001),
    (0x52b0, 0x00000001), (0x52c0, 0x00000064), (0x52c4, 0x00000001),
    (0x52c8, 0x000004b0), (0x52d0, 0x00000064), (0x5300, 0x3f828f5c),
    (0x5304, 0x3f828f5c), (0x5308, 0x3f828f5c), (0x530c, 0x3f828f5c),
    (0x5310, 0x3f828f5c), (0x5314, 0x3f828f5c), (0x5318, 0x3f828f5c),
    (0x531c, 0x3f828f5c), (0x5320, 0x3f828f5c), (0x5324, 0x3f828f5c),
    (0x5328, 0x3f828f5c), (0x532c, 0x3f828f5c), (0x5330, 0x3f828f5c),
    (0x5bf8, 0x00000271), (0x5c04, 0x3f7f2e9f), (0x5c0c, 0x3b516154),
    (0x5c14, 0x3ca59586), (0x5c20, 0x46bd6000), (0x5c24, 0x40a90fe4),
    (0x5c2c, 0xbd4ac5b4), (0x5c30, 0x00000028), (0x5c34, 0x000004b0),
    (0x5c38, 0x000004b0), (0x5c44, 0x00005eb0), (0x5c78, 0x000004b0),
    (0x5cb0, 0x00000008), (0x5cbc, 0x3f000000), (0x5cc4, 0x3f000000),
    (0x5ccc, 0x3f99999a), (0x5cd8, 0x47800000), (0x5cdc, 0x3e800000),
    (0x5ce8, 0x00000028), (0x5cec, 0x000004b0), (0x5cfc, 0x00005eb0),
    (0x5d08, 0x00000010), (0x5d10, 0x0005dc00), (0x5d68, 0x0000004a),
    (0x5d70, 0x0000005a), (0x5d74, 0x00000032), (0x5d78, 0x00000006),
    (0x5d80, 0x00000006), (0x5d84, 0x00000001), (0x5d8c, 0x3f4ccccd),
    (0x5d90, 0x3f7eb852), (0x5d94, 0x3e4ccccd), (0x5d98, 0x3ba3d70a),
    (0x5d9c, 0x3f144b3c), (0x5da0, 0x3f144b3c), (0x5da8, 0x42be0000),
    (0x5dac, 0x4029eb85), (0x5db0, 0x4029eb85), (0x5db4, 0x4139435e),
    (0x5db8, 0x00000064), (0x5dbc, 0x000004b0), (0x5dc0, 0x00000064),
    (0x5dcc, 0x0000004a), (0x5dd4, 0x00000001), (0x5e00, 0x00000064),
    (0x5e20, 0x00000401), (0x5e24, 0x00006400), (0x5e2c, 0x001f4000),
    (0x5e30, 0x0000c800), (0x5e34, 0x000fa000), (0x5e38, 0x00012c00),
    (0x5e3c, 0x0007d000), (0x5e40, 0x00012c00), (0x5e44, 0x0003e800),
    (0x5e48, 0x00012c00), (0x5e4c, 0x00000100), (0x5ea4, 0x00000001),
    (0x5ed0, 0x00000001), (0x74d8, 0x47800000), (0x74dc, 0x42200000),
    (0x74e0, 0x44960000), (0x74e4, 0x0000062a), (0x74f8, 0x00000028),
    (0x7500, 0x42c80000), (0x7504, 0x000004b0), (0x750c, 0x3f4ccccd),
    (0x7510, 0x3e4ccccd), (0x7550, 0xdcdcfbfa), (0x7558, 0x00001f40),
    (0x7560, 0x00000004), (0x7574, 0x3f800000), (0x757c, 0x3d0f5c28),
    (0x7588, 0x47800000), (0x758c, 0x41980000), (0x7598, 0x00000028),
    (0x759c, 0x000004b0), (0x75a0, 0x000004b0), (0x75ac, 0x00002af8),
    (0x75e0, 0x000004b0), (0x77f8, 0x00000002), (0x77fc, 0x42000006),
    (0x7a10, 0xb8520000), (0x7a14, 0x00003f7e), (0x7a18, 0xd70a0000),
    (0x7a1c, 0x00003ba3), (0x7a24, 0x000043c8), (0x7a30, 0x00004780),
    (0x7a34, 0x000042c8), (0x7a3c, 0x0384bac8), (0x7a40, 0x03e80000),
    (0x7a44, 0x03e80000), (0x7a4c, 0x80000000), (0x7a50, 0x26ac453b),
    (0x7a84, 0x03e80000), (0x7aa0, 0x00c80000), (0x7b58, 0x000004b0),
    (0x7b6c, 0x00000001), (0x7ba0, 0x00000101), (0x7ba4, 0x00000200),
    (0x7bbc, 0x00023f00), (0x7bc0, 0x00000100), (0x7bc4, 0x00000100),
    (0x7bdc, 0x00024000), (0x7be0, 0x00000100), (0x7be4, 0x00000100),
    (0x7bfc, 0x00024100), (0x7c48, 0x00000001), (0x7c50, 0x00005eb0),
    (0x7c54, 0x00005eb0), (0x7c58, 0x00005eb0), (0x7c68, 0x3f800000),
    (0x7c6c, 0x00000004), (0x7c70, 0x00000008), (0x7c74, 0x0002ee00),
    (0x7d44, 0x0000007c), (0x7d50, 0x3f77bdef), (0x7d58, 0x3d042108),
    (0x7d60, 0x3e0a3d71), (0x7d6c, 0x47800000), (0x7d70, 0x401ae148),
    (0x7d7c, 0x00000028), (0x7d80, 0x000004b0), (0x7d84, 0x000004b0),
    (0x7d8c, 0x46bd6000), (0x7d90, 0x00005eb0), (0x7d9c, 0x000000f8),
    (0x7da4, 0x005ad200), (0x7dc4, 0x000004b0), (0x7e04, 0x00000001),
    (0x7ed4, 0x00000032), (0x7ed8, 0x00000001), (0x7ee0, 0x3f638e39),
    (0x7ee4, 0x3f2aaaab), (0x7ee8, 0x3de38e39), (0x7eec, 0x3eaaaaab),
    (0x7ef0, 0xbecccccd), (0x7ef4, 0xbf4ccccd), (0x7efc, 0x47800000),
    (0x7f00, 0xc0a00000), (0x7f04, 0xc0a00000), (0x7f0c, 0x00000064),
    (0x7f10, 0x000004b0), (0x7f14, 0x00000064), (0x7f1c, 0x45fa0000),
    (0x7f20, 0x00000578), (0x7f28, 0x0000000e), (0x7f2c, 0x00000048),
    (0x7f30, 0x00000018), (0x7f34, 0x001a5e00), (0x7f3c, 0x0008ca00),
    (0x7f48, 0x47800000), (0x7f70, 0x00001f40), (0x7f74, 0x00000004),
    (0x7f78, 0x000009c4), (0x7f7c, 0x00000258), (0x7f80, 0x00000002),
    (0x7f84, 0x00000004), (0x7f88, 0x00000032), (0x7fb8, 0x47800000),
    (0x7fc8, 0x00000028), (0x7fcc, 0x000004b0), (0x8098, 0x00000001),
    (0x80ac, 0x0000000a), (0x80b8, 0x00000001), (0x8174, 0xac000000),
    (0x8178, 0xe800000d), (0x817c, 0xb8000003), (0x8180, 0xb000000b),
    (0x8184, 0xb0000004), (0x8188, 0xbc000004), (0x818c, 0x20000002),
    (0x8198, 0x00000001), (0x81b4, 0x00000004), (0x81c0, 0x3f780000),
    (0x81c8, 0x3d000000), (0x81d0, 0x40a00000), (0x81dc, 0x47800000),
    (0x81e0, 0x41700000), (0x81ec, 0x0000044c), (0x81f0, 0x000004b0),
    (0x81f4, 0x000004b0), (0x8200, 0x000004b0), (0x8234, 0x000004b0),
    (0x827c, 0x00000064), (0x8290, 0xdcdcfbfa), (0x8298, 0x0000007d),
    (0x829c, 0x00000001), (0x8ee0, 0xdcdcfbfb), (0x8ee8, 0x00000001),
    (0xaeec, 0x41c80000), (0xaef0, 0x41c80000), (0xb6a8, 0x67305452),
    (0xb6e8, 0x00000001), (0xb728, 0x5243464c), (0xb72c, 0x4758464c),
    (0xb738, 0x47584c50), (0xb748, 0x00000001), (0xb7a8, 0x00000001),
    (0xb7b4, 0x42b73333), (0xb7f4, 0x449b8000), (0xb834, 0x42bc0000),
    (0xb8e0, 0x0000186a),
];

/// Initialize the six allocations following M5 hardware-data. Preserve the
/// hardware block and subsequent channel resources. No firmware is started.
pub(crate) fn encode_m5_25g76_bundle_policy(
    bindings: M5PolicyBindings,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() < M5_25G76_SHARED_PREFIX_SIZE {
        return Err(InitdataError::BufferSize);
    }
    required_address(bindings.state)?;
    required_address(bindings.aon_ptd)?;
    let state_end = bindings.state.checked_add(STATUS_BLOCK_SIZE as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    let aon_end = bindings.aon_ptd.checked_add(M5_25G76_AON_PTD_SIZE as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    required_address(state_end - 1)?;
    required_address(aon_end - 1)?;
    if bindings.state < aon_end && bindings.aon_ptd < state_end {
        return Err(InitdataError::PairAliasedObject);
    }
    out[0x2740..M5_25G76_SHARED_PREFIX_SIZE].fill(0);
    for &(offset, value) in M5_25G76_POLICY_WORDS {
        put_u32(out, offset, value);
    }
    for index in 0..512 {
        put_u16(out, 0x6ed7 + index * 2, 0x7fff);
    }
    put_u64(out, 0x8ed8, bindings.state);
    put_u64(out, 0xb6a0, bindings.aon_ptd);
    put_u64(out, 0xb75c, bindings.boot_timestamp);
    Ok(())
}


/// Recorded platform words outside the explicit register, performance and
/// owned-pointer fields. Meanings not yet derived are deliberately kept as
/// target-scoped measured values. Never insert runtime completion counters
/// or captured allocation addresses here. Reference hardware-data SHA-256:
/// 6f9ac0c4664f898a0ddb4e5df9ad160a23c1b52d3fd750750779cbdebe446be9.
const M5_25G76_HW_PLATFORM_WORDS: &[(usize, u32)] = &[
    (0x0004, 0x0000006f), (0x0008, 0xffc00000), (0x0014, 0x00000010),
    (0x001c, 0x00000010), (0x0020, 0xffff8000), (0x0024, 0x000002ff),
    (0x0028, 0x81400000), (0x002c, 0xfffffc21), (0x00e0, 0x00002008),
    (0x00e8, 0x20080000), (0x00f4, 0x00002008), (0x00f8, 0x000024cb),
    (0x00fc, 0xc98a2cfa), (0x0100, 0xf4f624cb), (0x0104, 0x1877e917),
    (0x0108, 0x38d924cb), (0x010c, 0xbdab0000), (0x02d8, 0xfff82000),
    (0x02dc, 0xd32d2cdb), (0x02e0, 0xf5002000), (0x02e4, 0x21dae926),
    (0x02e8, 0x38b62000), (0x02ec, 0xc7420008), (0x03e0, 0x00007fe0),
    (0x03e8, 0x7fe00000), (0x03f4, 0x00007fe0), (0x03f8, 0x00008000),
    (0x0400, 0x80000000), (0x040c, 0x00008000), (0x05d8, 0x4b232645),
    (0x05dc, 0x00000e98), (0x05e0, 0xd5a2ea5f), (0x05e4, 0x40003fff),
    (0x05e8, 0xca5e4000), (0x05ec, 0x4000f5a2), (0x0e90, 0x00008142),
    (0x0e94, 0x00000002), (0x0ea0, 0x00000001), (0x0ea4, 0x00000001),
    (0x0eb8, 0x00000001), (0x0ec0, 0x00000001), (0x0ec8, 0x00000001),
    (0x0ed0, 0x00005dc0), (0x0ed4, 0x00000001), (0x0ed8, 0x00000008),
    (0x0ee0, 0x00000001), (0x0ee4, 0x00000001), (0x0ee8, 0x00000001),
    (0x0f04, 0x0000001f), (0x0f24, 0x07800004), (0x0f28, 0x00000006),
    (0x0f34, 0x00000001), (0x0f38, 0x00000001), (0x0f4c, 0x00000031),
    (0x0f6c, 0x00000001), (0x0f88, 0x0000000a), (0x0f8c, 0x00000001),
    (0x0fac, 0x00000001), (0x0fb8, 0x00000020), (0x0fbc, 0x00000005),
    (0x0fc0, 0x0000000a), (0x0fc4, 0x0000000c), (0x2544, 0x00000004),
    (0x2548, 0x00000006), (0x254c, 0x00000003), (0x2550, 0x00000007),
    (0x2554, 0x00000007), (0x2568, 0x0000000a), (0x2570, 0x00000001),
    (0x258c, 0x00180000), (0x2594, 0x00000001), (0x2598, 0x00000001),
    (0x259c, 0x00000001), (0x25a4, 0x00000001), (0x25a8, 0x00000001),
    (0x25ac, 0x00000001), (0x25b4, 0xffffffff), (0x25b8, 0xffffffff),
    (0x25bc, 0xffffffff), (0x25c0, 0xffffffff), (0x25c4, 0xffffffff),
    (0x25c8, 0xffffffff), (0x25cc, 0xffffffff), (0x25d0, 0xffffffff),
    (0x25d4, 0xffffffff), (0x25d8, 0xffffffff), (0x25dc, 0xffffffff),
    (0x25e0, 0xffffffff), (0x25f4, 0x00000001), (0x2600, 0x00000001),
    (0x2608, 0x00000001), (0x2610, 0x00000100), (0x26b4, 0x00000001),
    (0x26b8, 0x00000001), (0x26e0, 0x000003e8), (0x26e8, 0x00000001),
];

/// Reconstruct the M5 reference hardware-data block with caller-owned queue
/// and QoS bindings. Performance values are measured on the ten-core J813
/// reference, not derived from ADT and not a generic M5 power configuration.
/// Both pre-handoff firmware roles supplied identical reference bytes.
/// This encoder grants no runtime admission and performs no MMIO accesses.
pub(crate) fn encode_m5_25g76_hw_data(
    aliases: [G17PKsmCompletionAliases; M5_25G76_COMPLETION_QUEUE_COUNT],
    qos_high: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != M5_25G76_HW_DATA_SIZE {
        return Err(InitdataError::BufferSize);
    }
    required_address(qos_high)?;
    let qos_end = qos_high.checked_add(0xe40).ok_or(InitdataError::AddressOutOfRange)?;
    required_address(qos_end - 1)?;
    let mut completion = [0; 0x80];
    encode_m5_25g76_completion_table(aliases, &mut completion)?;
    for alias in aliases {
        if qos_high < alias.high + M5_25G76_COMPLETION_BACKING_SIZE as u64
            && alias.high < qos_end {
            return Err(InitdataError::PairAliasedObject);
        }
    }

    out.fill(0);
    for &(offset, value) in M5_25G76_HW_PLATFORM_WORDS {
        put_u32(out, offset, value);
    }
    encode_m5_25g76_register_map(&mut out[0x640..0xe88])?;
    out[0x2614..0x2694].copy_from_slice(&completion);
    put_u64(out, 0x26ac, qos_high);

    // Native includes index zero plus twelve operating points. Each voltage
    // occupies all sixteen lanes of its 0x40-byte state row.
    const FREQUENCY: [u32; 13] = [0, 338, 486, 636, 796, 888, 988, 1084, 1182, 1278, 1374, 1470, 1578];
    const CORE_VOLTAGE: [u32; 13] = [125, 615, 630, 660, 705, 735, 755, 800, 830, 865, 905, 945, 1010];
    const MEMORY_VOLTAGE: [u32; 13] = [770, 770, 770, 770, 770, 770, 770, 800, 830, 865, 905, 945, 1010];
    const RELATIVE_A: [u32; 13] = [0, 17, 23, 30, 42, 50, 58, 71, 82, 85, 89, 93, 100];
    const RELATIVE_B: [u32; 13] = [0, 0, 11, 24, 36, 44, 52, 60, 68, 75, 83, 91, 100];
    for index in 0..13 {
        put_u32(out, 0xfc8 + index * 4, FREQUENCY[index]);
        put_u32(out, 0x1848 + index * 4, 0x3f828f5c);
        put_u32(out, 0x18c8 + index * 4, RELATIVE_A[index]);
        put_u32(out, 0x1908 + index * 4, RELATIVE_B[index]);
        for lane in 0..16 {
            put_u32(out, 0x1008 + index * 0x40 + lane * 4, CORE_VOLTAGE[index]);
            put_u32(out, 0x1408 + index * 0x40 + lane * 4, MEMORY_VOLTAGE[index]);
        }
    }
    for index in 0..16 {
        put_u32(out, 0x19c8 + index * 4, index as u32);
    }
    Ok(())
}

/// T8140 register windows mapped into the firmware MMIO aperture.
pub(crate) const T8140_REGISTER_WINDOWS: [RegisterWindow; 18] = [
    RegisterWindow {
        slot: 0,
        phys: 0x0030_1014_000,
        device_va: 0xffff_fc21_8000_0000,
        size: 0x004000,
        unk_18: 0,
        flag: 2,
    },
    RegisterWindow {
        slot: 3,
        phys: 0x0022_0104_000,
        device_va: 0xffff_fc21_8000_8000,
        size: 0x018000,
        unk_18: 0,
        flag: 2,
    },
    RegisterWindow {
        slot: 9,
        phys: 0x0030_03d0_000,
        device_va: 0xffff_fc21_8002_8000,
        size: 0x001000,
        unk_18: 0,
        flag: 2,
    },
    RegisterWindow {
        slot: 10,
        phys: 0x0030_03c0_000,
        device_va: 0xffff_fc21_8003_0000,
        size: 0x002000,
        unk_18: 0,
        flag: 0,
    },
    RegisterWindow {
        slot: 12,
        phys: 0x0040_165c_000,
        device_va: 0xffff_fc21_8003_8000,
        size: 0x004000,
        unk_18: 0,
        flag: 2,
    },
    RegisterWindow {
        slot: 14,
        phys: 0x0030_0280_000,
        device_va: 0xffff_fc21_8004_0000,
        size: 0x008000,
        unk_18: 0,
        flag: 0,
    },
    RegisterWindow {
        slot: 17,
        phys: 0x0048_0000_000,
        device_va: 0xffff_fc21_8005_0000,
        size: 0x021400,
        unk_18: 0,
        flag: 2,
    },
    RegisterWindow {
        slot: 22,
        phys: 0x0048_1000_000,
        device_va: 0xffff_fc21_8007_8000,
        size: 0x008000,
        unk_18: 0,
        flag: 2,
    },
    RegisterWindow {
        slot: 26,
        phys: 0x0048_0d04_000,
        device_va: 0xffff_fc21_8008_8000,
        size: 0x008000,
        unk_18: 0xd04000,
        flag: 2,
    },
    RegisterWindow {
        slot: 27,
        phys: 0x0048_0d0d_000,
        device_va: 0xffff_fc21_8009_9000,
        size: 0x001000,
        unk_18: 0xd0d000,
        flag: 2,
    },
    RegisterWindow {
        slot: 28,
        phys: 0x0048_0d58_000,
        device_va: 0xffff_fc21_800a_0000,
        size: 0x008000,
        unk_18: 0xd58000,
        flag: 2,
    },
    RegisterWindow {
        slot: 29,
        phys: 0x0048_0d10_000,
        device_va: 0xffff_fc21_800b_0000,
        size: 0x004000,
        unk_18: 0xd10000,
        flag: 2,
    },
    RegisterWindow {
        slot: 31,
        phys: 0x0048_0d40_000,
        device_va: 0xffff_fc21_800b_8000,
        size: 0x004000,
        unk_18: 0xd40000,
        flag: 2,
    },
    RegisterWindow {
        slot: 32,
        phys: 0x0048_0d60_000,
        device_va: 0xffff_fc21_800c_0000,
        size: 0x004000,
        unk_18: 0xd60000,
        flag: 2,
    },
    RegisterWindow {
        slot: 35,
        phys: 0x0048_0e00_000,
        device_va: 0xffff_fc21_800c_8000,
        size: 0x004000,
        unk_18: 0xe00000,
        flag: 2,
    },
    RegisterWindow {
        slot: 39,
        phys: 0x0048_0e08_000,
        device_va: 0xffff_fc21_800d_0000,
        size: 0x008000,
        unk_18: 0,
        flag: 2,
    },
    RegisterWindow {
        slot: 40,
        phys: 0x0048_0e1c_000,
        device_va: 0xffff_fc21_800e_0000,
        size: 0x004000,
        unk_18: 0xe1c000,
        flag: 2,
    },
    RegisterWindow {
        slot: 41,
        phys: 0x0048_0e1f_800,
        device_va: 0xffff_fc21_800e_b800,
        size: 0x004000,
        unk_18: 0,
        flag: 2,
    },
];

pub(crate) const T8140_REGISTER_FLAG_ONLY_SLOTS: [(usize, u32); 17] = [
    (2, 2),
    (5, 2),
    (6, 2),
    (7, 2),
    (8, 2),
    (30, 2),
    (33, 2),
    (34, 2),
    (37, 2),
    (38, 2),
    (42, 2),
    (43, 2),
    (46, 2),
    (47, 2),
    (48, 2),
    (49, 2),
    (52, 2),
];

/// Fail-closed validation errors for the grounded layout.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum InitdataError {
    BufferSize,
    AddressOutOfRange,
    NullRequiredAddress,
    RoleSpecificAddress,
    BundleAllocationTooSmall,
    ChannelTableShape,
    RegisterWindow,
    PairRootSeparation,
    PairAliasedObject,
    SharedRegionSeparation,
    InterfaceMismatch,
    HostMappedAllocationsDisabled,
}

fn address_representable(address: u64) -> bool {
    let high = address & !INITDATA_ADDR_MASK;
    high == 0 || high == INITDATA_ADDR_PREFIX
}

fn required_address(address: u64) -> Result<(), InitdataError> {
    if address == 0 {
        Err(InitdataError::NullRequiredAddress)
    } else if !address_representable(address) {
        Err(InitdataError::AddressOutOfRange)
    } else {
        Ok(())
    }
}

fn put_u16(bytes: &mut [u8], offset: usize, value: u16) {
    bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_u32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_u64(bytes: &mut [u8], offset: usize, value: u64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

fn get_u32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

fn get_u64(bytes: &[u8], offset: usize) -> u64 {
    u64::from_le_bytes(bytes[offset..offset + 8].try_into().unwrap())
}

fn write_runs(bytes: &mut [u8], runs: &[(usize, &[u8])]) {
    for (offset, data) in runs {
        bytes[*offset..*offset + data.len()].copy_from_slice(data);
    }
}

fn validate_shared(shared: &SharedAddresses) -> Result<(), InitdataError> {
    required_address(shared.region_a)?;
    required_address(shared.region_c)?;
    required_address(shared.hw_data_bundle)?;
    if shared.hw_data_bundle_alloc < HW_DATA_BUNDLE_MIN_ALLOC {
        return Err(InitdataError::BundleAllocationTooSmall);
    }
    Ok(())
}

fn validate_instance(role: InstanceRole, inst: &InstanceAddresses) -> Result<(), InitdataError> {
    required_address(inst.root)?;
    required_address(inst.main_config)?;
    required_address(inst.status_a)?;
    match role {
        InstanceRole::Primary => {
            // The primary root has no +0xb8/+0xc0 fields at all; carrying
            // secondary extras there is a role-confusion bug at the caller.
            if inst.secondary_extras != [0, 0] {
                return Err(InitdataError::RoleSpecificAddress);
            }
            required_address(inst.status_b)?;
        }
        InstanceRole::Secondary => {
            // Every hardware-booting host hands the secondary a zero status-B;
            // a nonzero value there is ungrounded, so fail closed on it.
            if inst.status_b != 0 {
                return Err(InitdataError::RoleSpecificAddress);
            }
            for extra in inst.secondary_extras {
                required_address(extra)?;
            }
        }
    }
    Ok(())
}

/// Encode one 0x20-byte UAT level descriptor. Every field is derivable from
/// the level's index shift and entry count; the index mask
/// `(entries - 1) << shift` reproduces all three captured masks exactly.
fn encode_uat_level_descriptor(bytes: &mut [u8], offset: usize, shift: u8, entries: u16) {
    bytes[offset] = 8;
    bytes[offset + 1] = UAT_PAGE_BITS;
    bytes[offset + 2] = UAT_PAGE_BITS;
    bytes[offset + 3] = shift;
    put_u16(bytes, offset + 0x04, entries);
    put_u16(bytes, offset + 0x06, UAT_PAGE_SIZE);
    put_u64(bytes, offset + 0x08, 1);
    put_u64(bytes, offset + 0x10, UAT_LEVEL_PHYS_MASK);
    put_u64(bytes, offset + 0x18, (entries as u64 - 1) << shift);
}

/// Encode one instance's root.
///
/// The output is zeroed first, which makes unknown bytes deterministic and,
/// critically, prevents stale host memory from becoming invented firmware
/// ABI. Root `+0x10` stays zero: it is zero at handoff on both instances and
/// its semantics are unknown.
pub(crate) fn encode_root(
    role: InstanceRole,
    shared: &SharedAddresses,
    inst: &InstanceAddresses,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    encode_root_for_profile(RootProfile::J700G17P, role, shared, inst, out)
}

/// Encode the common root layout with an explicitly selected firmware build.
/// The existing runtime continues to call the J700-only wrapper above.
pub(crate) fn encode_root_for_profile(
    profile: RootProfile,
    role: InstanceRole,
    shared: &SharedAddresses,
    inst: &InstanceAddresses,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    // Profile-sized: identical to root_size(role) for J700/J813.
    if out.len() != profile.root_size(role) {
        return Err(InitdataError::BufferSize);
    }
    validate_shared(shared)?;
    validate_instance(role, inst)?;

    out.fill(0);
    put_u64(out, ROOT_VERSION, profile.version_word());
    put_u64(out, ROOT_REGION_A, shared.region_a);
    // ROOT_ZERO_10 deliberately left zero.
    put_u64(out, ROOT_MAIN_CONFIG, inst.main_config);
    put_u64(out, ROOT_REGION_C, shared.region_c);
    put_u32(out, ROOT_INSTANCE_KIND, role as u32);
    put_u32(out, ROOT_CONSTANT_2C, 1);
    put_u16(out, ROOT_PAGE_SIZE, UAT_PAGE_SIZE);
    out[ROOT_PAGE_BITS] = UAT_PAGE_BITS;
    out[ROOT_LEVEL_COUNT] = UAT_LEVEL_COUNT as u8;
    for (index, (shift, entries)) in UAT_LEVELS.iter().enumerate() {
        encode_uat_level_descriptor(
            out,
            ROOT_LEVELS + index * UAT_LEVEL_DESC_SIZE,
            *shift,
            *entries,
        );
    }
    put_u64(out, ROOT_STATUS_A, inst.status_a);
    put_u64(out, ROOT_STATUS_B, inst.status_b);
    if role == InstanceRole::Secondary {
        put_u64(out, ROOT_SECONDARY_EXTRA_0, inst.secondary_extras[0]);
        put_u64(out, ROOT_SECONDARY_EXTRA_1, inst.secondary_extras[1]);
    }
    Ok(())
}

/// Byte range of one channel-table entry inside the main configuration.
pub(crate) fn channel_table_entry_range(index: usize) -> Option<Range<usize>> {
    if index >= CHANNEL_TABLE_ENTRIES {
        return None;
    }
    let start = MAIN_CHANNEL_TABLE + index * CHANNEL_ENTRY_SIZE;
    Some(start..start + CHANNEL_ENTRY_SIZE)
}

/// Derive the native channel table for one instance.
///
/// `state_grid` is this instance's compact channel-state block (0x40-stride,
/// permuted per [`WORK_STATE_GRID_OFFSETS`]); `status_a` anchors the trailing
/// report/partial channels; the ring for entry 12 is `main_config + 0x4c0`;
/// work rings for the primary sit in the bundle at
/// [`WORK_RING_BUNDLE_OFFSETS`] (which requires the
/// [`NATIVE_SHARED_CLUSTER_SIZE`] cluster). The secondary leaves entries
/// 0..11 empty: it is a control-only peer with no work channels.
pub(crate) fn derive_channel_table(
    role: InstanceRole,
    state_grid: u64,
    status_a: u64,
    main_config: u64,
    bundle: u64,
) -> [ChannelEntry; CHANNEL_TABLE_ENTRIES] {
    let mut table = [ChannelEntry::EMPTY; CHANNEL_TABLE_ENTRIES];
    let grid_states = |index: usize| {
        let base = state_grid + WORK_STATE_GRID_OFFSETS[index] as u64;
        [
            base,
            base + CHANNEL_STATE_SPACING as u64,
            base + 2 * CHANNEL_STATE_SPACING as u64,
        ]
    };
    if role == InstanceRole::Primary {
        for index in 0..WORK_CHANNEL_COUNT {
            table[index] = ChannelEntry {
                states: grid_states(index),
                ring: bundle + WORK_RING_BUNDLE_OFFSETS[index] as u64,
            };
        }
    }
    table[CONTROL_CHANNEL_INDEX] = ChannelEntry {
        states: grid_states(CONTROL_CHANNEL_INDEX),
        ring: main_config + MAIN_CONTROL_RING as u64,
    };
    for (slot, index) in REPORT_CHANNEL_INDICES.iter().enumerate() {
        let mut states = [0u64; 3];
        for (word, offset) in states.iter_mut().zip(REPORT_STATE_STATUS_A_OFFSETS[slot]) {
            *word = status_a + offset as u64;
        }
        table[*index] = ChannelEntry {
            states,
            ring: status_a + REPORT_RING_STATUS_A_OFFSETS[slot] as u64,
        };
    }
    table[PARTIAL_CHANNEL_INDEX] = ChannelEntry {
        states: [status_a + PARTIAL_STATE_STATUS_A_OFFSET as u64, 0, 0],
        ring: 0,
    };
    table
}

/// [`derive_channel_table`] with the profile's work-ring placement. For J700
/// and J813 the result is identical to [`derive_channel_table`]. G18G keeps
/// the same channel/ring permutation and status-A-relative report layout
/// (FW_PROVEN consumers b000 0x84f0/0x27048/0x7f14/0x19984 match G17); only
/// the ring bundle offsets move (+0xf00, from the 26A428 size table).
pub(crate) fn derive_channel_table_for_profile(
    profile: RootProfile,
    role: InstanceRole,
    state_grid: u64,
    status_a: u64,
    main_config: u64,
    bundle: u64,
) -> [ChannelEntry; CHANNEL_TABLE_ENTRIES] {
    let mut table = derive_channel_table(role, state_grid, status_a, main_config, bundle);
    if role == InstanceRole::Primary {
        let rings = profile.work_ring_bundle_offsets();
        for (index, entry) in table.iter_mut().take(WORK_CHANNEL_COUNT).enumerate() {
            entry.ring = bundle + rings[index] as u64;
        }
    }
    table
}

fn validate_populated_entry(entry: &ChannelEntry) -> Result<(), InitdataError> {
    // A populated channel needs all three state-counter addresses — a null
    // producer (state 2) on a populated entry makes firmware fault on address
    // zero — and a real ring.
    for state in entry.states {
        if state == 0 {
            return Err(InitdataError::ChannelTableShape);
        }
        if !address_representable(state) {
            return Err(InitdataError::AddressOutOfRange);
        }
    }
    required_address(entry.ring)
}

fn validate_channel_table(
    role: InstanceRole,
    main_config: u64,
    channels: &[ChannelEntry; CHANNEL_TABLE_ENTRIES],
) -> Result<(), InitdataError> {
    for (index, entry) in channels.iter().enumerate() {
        match index {
            0..=11 => match role {
                InstanceRole::Primary => validate_populated_entry(entry)?,
                // The secondary has no work channels at all.
                InstanceRole::Secondary => {
                    if *entry != ChannelEntry::EMPTY {
                        return Err(InitdataError::ChannelTableShape);
                    }
                }
            },
            12 => {
                validate_populated_entry(entry)?;
                // Device control is embedded in the main object, at exactly
                // main + 0x4c0 on both instances.
                if entry.ring != main_config + MAIN_CONTROL_RING as u64 {
                    return Err(InitdataError::ChannelTableShape);
                }
            }
            13 | 14 => validate_populated_entry(entry)?,
            15 => {
                // Partial: a first state address and nothing else.
                required_address(entry.states[0])?;
                if entry.states[1] != 0 || entry.states[2] != 0 || entry.ring != 0 {
                    return Err(InitdataError::ChannelTableShape);
                }
            }
            _ => {
                if *entry != ChannelEntry::EMPTY {
                    return Err(InitdataError::ChannelTableShape);
                }
            }
        }
    }
    Ok(())
}

/// Encode one instance's 0x600-byte main configuration object.
///
/// `region_views` must be `Some` for the primary and `None` for the
/// secondary: the secondary's region views carry the two low context-0
/// values with no addresses at all.
pub(crate) fn encode_main_config(
    role: InstanceRole,
    shared: &SharedAddresses,
    inst: &InstanceAddresses,
    channels: &[ChannelEntry; CHANNEL_TABLE_ENTRIES],
    region_views: Option<&PrimaryRegionViews>,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    encode_main_config_for_profile(RootProfile::J700G17P, role, shared, inst,
                                   channels, region_views, out)
}

/// The two complete M5/25G76 pre-handoff main configurations match the common
/// layout except primary +0x32c/+0x344 and secondary +0x300. Keep these
/// measured scalars build-specific; their meanings are not inferred here.
pub(crate) fn encode_main_config_for_profile(
    profile: RootProfile,
    role: InstanceRole,
    shared: &SharedAddresses,
    inst: &InstanceAddresses,
    channels: &[ChannelEntry; CHANNEL_TABLE_ENTRIES],
    region_views: Option<&PrimaryRegionViews>,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != MAIN_CONFIG_SIZE {
        return Err(InitdataError::BufferSize);
    }
    validate_shared(shared)?;
    validate_instance(role, inst)?;
    validate_channel_table(role, inst.main_config, channels)?;
    match (role, region_views) {
        (InstanceRole::Primary, Some(views)) => {
            required_address(views.sentinel_high)?;
            for high in views.alias_high {
                required_address(high)?;
            }
        }
        (InstanceRole::Secondary, None) => {}
        _ => return Err(InitdataError::RoleSpecificAddress),
    }
    // Every view must fit inside the one bundle allocation. (The last two end
    // exactly at 0xf000, inside the 0x10000 minimum.) The view table is the
    // profile's: J700/J813 keep BUNDLE_VIEW_OFFSETS/EXTENTS unchanged.
    let view_offsets = profile.bundle_view_offsets();
    for (offset, extent) in view_offsets.iter().zip(profile.bundle_view_extents()) {
        if offset + extent > shared.hw_data_bundle_alloc {
            return Err(InitdataError::BundleAllocationTooSmall);
        }
    }

    out.fill(0);
    // Firmware log enable. The emitter at A000 0xfffffc0000021fa0 reads this
    // word and returns immediately when it is zero; every buffer the facility
    // needs is already allocated and pointed at by the addresses below. The
    // records are plain ASCII. Shipping macOS provisions the whole facility
    // and then throws the messages away (drainFirmwareLogRing is `bti c; ret`),
    // which is why the captured value is zero.
    if crate::g18::params::g17p_fw_log.value() != 0 {
        put_u32(out, MAIN_FIRMWARE_LOG_ENABLE, 1);
    }
    let bundle = shared.hw_data_bundle;
    put_u64(out, MAIN_HW_DATA, bundle);
    let repeated = bundle + profile.repeated_region_bundle_offset() as u64;
    put_u64(out, MAIN_REPEATED_0, repeated);
    put_u64(out, MAIN_REPEATED_1, repeated);

    for (index, entry) in channels.iter().enumerate() {
        let base = MAIN_CHANNEL_TABLE + index * CHANNEL_ENTRY_SIZE;
        for (word, state) in entry.states.iter().enumerate() {
            put_u64(out, base + word * 8, *state);
        }
        put_u64(out, base + CHANNEL_ENTRY_STATE_COUNT * 8, entry.ring);
    }

    match (role, region_views) {
        (InstanceRole::Primary, Some(views)) => {
            // The five bare pointers are views into the one bundle
            // allocation, at fixed offsets. Unaligned stores at +0x254.
            for (index, offset) in view_offsets.iter().enumerate() {
                put_u64(out, MAIN_BUNDLE_VIEWS + index * 8, bundle + *offset as u64);
            }
            // High-only sentinel followed by two low/high region-view pairs.
            put_u64(out, MAIN_REGION_VIEWS, views.sentinel_high);
            put_u64(out, MAIN_REGION_VIEWS + 0x08, REGION_VIEW_LOW_ADDRS[0]);
            put_u64(out, MAIN_REGION_VIEWS + 0x10, views.alias_high[0]);
            put_u64(out, MAIN_REGION_VIEWS + 0x18, REGION_VIEW_LOW_ADDRS[1]);
            put_u64(out, MAIN_REGION_VIEWS + 0x20, views.alias_high[1]);
            if profile == RootProfile::J700G17P {
                put_u32(out, MAIN_PRIMARY_SCALAR_32C, 1);
                put_u32(out, MAIN_PRIMARY_SCALAR_344, 0xabcd_abcd);
            }
            // KEXT_PROVEN for G18G too (0xfffffe0008c31780: main+0x3e0=0xff).
            put_u32(out, MAIN_PRIMARY_MASK, MAIN_PRIMARY_MASK_VALUE);
            put_u32(out, MAIN_CONTROL_RING, profile.control_opening_opcode(role));
        }
        (InstanceRole::Secondary, _) => {
            // The secondary keeps the two low context-0 values but carries no
            // view or region addresses; the values are therefore not
            // properties of any region the host allocates.
            put_u64(out, MAIN_REGION_VIEWS + 0x08, REGION_VIEW_LOW_ADDRS[0]);
            put_u64(out, MAIN_REGION_VIEWS + 0x18, REGION_VIEW_LOW_ADDRS[1]);
            put_u32(out, MAIN_SECONDARY_KIND, match profile {
                RootProfile::J700G17P => MAIN_SECONDARY_KIND_VALUE,
                RootProfile::J813G17G25G76 => 0xc,
                // INFERRED: 26A428 stores accel+0x10a74 here (25G76:
                // accel+0xed30, measured 0xc on J813). Value not read on J873.
                RootProfile::J873G18G26A428 | RootProfile::J873G18G26B123 => {
                    G18G_26A428_MAIN_SECONDARY_KIND
                }
            });
            if profile.is_g18g() {
                // KEXT_PROVEN (26A428 initFirmwareSharedData, g18g_key.c:4464):
                // role 1 main+0x481 = FW VA of the perf/policy allocation.
                // 25G76 stored it at +0x471 and also wrote an accelerator
                // pair at +0x479/+0x481, which 26A428 no longer writes.
                put_u64(out, G18G_26A428_MAIN_SECONDARY_POLICY_PTR,
                        bundle + G18G_26A428_SHARED_LAYOUT.policy as u64);
            } else {
                // One additional unaligned pointer *into* primary view 2, landing
                // five bytes before that view's first populated run.
                let extra = bundle
                    + BUNDLE_VIEW_OFFSETS[SECONDARY_EXTRA_VIEW_INDEX] as u64
                    + SECONDARY_EXTRA_VIEW_OFFSET as u64;
                put_u64(out, MAIN_SECONDARY_EXTRA_PTR, extra);
            }
            put_u32(out, MAIN_CONTROL_RING, profile.control_opening_opcode(role));
        }
        _ => unreachable!(),
    }
    Ok(())
}

/// Encode the shared 0x3db4-byte hardware-data object (the base of the
/// bundle). Both instances reference this one object; handing the secondary a
/// private copy makes it fault.
///
/// `completion_aliases` are the low/high GPU mappings of the two retained
/// completion-queue backings named by descriptor ordinals 0 and 2.
pub(crate) fn encode_hw_data(
    windows: &[RegisterWindow],
    flag_only_slots: &[(usize, u32)],
    completion_aliases: [G17PKsmCompletionAliases; 2],
    qos_resource_high: u64,
    sksm_qid_resource_high: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != HW_DATA_SIZE {
        return Err(InitdataError::BufferSize);
    }
    let mut used = [false; HW_REGISTER_MAP_SLOTS];
    let mut claim = |slot: usize| -> Result<(), InitdataError> {
        if slot >= HW_REGISTER_MAP_SLOTS || used[slot] {
            return Err(InitdataError::RegisterWindow);
        }
        used[slot] = true;
        Ok(())
    };
    for window in windows {
        claim(window.slot)?;
        if window.phys == 0 || window.size == 0 {
            return Err(InitdataError::RegisterWindow);
        }
        // Firmware reaches its registers only through this table; the device
        // VAs live in one fixed aperture.
        if !HW_REGISTER_APERTURE.contains(&window.device_va) {
            return Err(InitdataError::RegisterWindow);
        }
    }
    for (slot, _flag) in flag_only_slots {
        claim(*slot)?;
    }
    for aliases in completion_aliases {
        required_address(aliases.low)?;
        required_address(aliases.high)?;
    }
    required_address(qos_resource_high)?;
    required_address(sksm_qid_resource_high)?;

    out.fill(0);
    // Native's `AGXFirmware::initFirmwareData` opens this object with five
    // qwords that Linux has always left zero: the range-3 base and size, the
    // architectural GART window base twice (Asahi's HwDataB calls the pair
    // "USC start"), and the reserved page at the top of range 5. They are the
    // only place the host tells the firmware side about 0x10_0000_0000, which
    // is the base hardware folds into every compact TA/VDM pointer, so a zero
    // here leaves USC-class units resolving bare offsets.
    for (offset, value) in HW_FIRMWARE_RANGE_HEAD {
        put_u64(out, *offset, *value);
    }
    for window in windows {
        let base = HW_REGISTER_MAP_OFFSET + window.slot * HW_REGISTER_MAP_ENTRY_SIZE;
        put_u64(out, base, window.phys);
        put_u64(out, base + 0x08, window.device_va);
        put_u32(out, base + 0x10, window.size);
        put_u32(out, base + 0x14, window.size);
        put_u64(out, base + 0x18, window.unk_18);
        put_u32(out, base + 0x20, window.flag);
    }
    for (slot, flag) in flag_only_slots {
        let base = HW_REGISTER_MAP_OFFSET + slot * HW_REGISTER_MAP_ENTRY_SIZE;
        put_u32(out, base + 0x20, *flag);
    }

    put_u32(out, HW_CHIP_ID_OFFSET, HW_CHIP_ID_T8140);

    for index in 0..HW_PERF_LADDER_ENTRIES {
        put_u32(out, HW_PERF_GROUP_BASES[0] + index * 4, HW_PERF_FREQ_A[index]);
        put_u32(out, HW_PERF_FREQ_LADDER_B + index * 4, HW_PERF_FREQ_B[index]);
        put_u32(out, HW_PERF_SCALE_LADDER_B + index * 4, HW_PERF_SCALE_B[index]);
        put_u32(
            out,
            HW_PERF_RELATIVE_LADDER_A + index * 4,
            HW_PERF_RELATIVE_A_VALUES[index],
        );
        put_u32(
            out,
            HW_PERF_RELATIVE_LADDER_B + index * 4,
            HW_PERF_RELATIVE_B_VALUES[index],
        );
        put_u32(out, HW_PERF_INDEX_MAP_A + index * 4, HW_PERF_INDEX_A_VALUES[index]);
        put_u32(out, HW_PERF_INDEX_MAP_B + index * 4, HW_PERF_INDEX_B_VALUES[index]);
        put_u32(out, HW_PERF_GROUP_BASES[1] + index * 4, HW_PERF_FREQ_B[index]);

        for repeat in 0..HW_PERF_GROUP_REPEATS[0] {
            put_u32(
                out,
                HW_PERF_GROUP_BASES[0]
                    + HW_PERF_VOLTAGE_DELTA
                    + index * HW_PERF_STATE_BLOCK_STRIDE
                    + repeat * 4,
                HW_PERF_CORE_VOLTAGE[index],
            );
            put_u32(
                out,
                HW_PERF_GROUP_BASES[0]
                    + HW_PERF_MEMORY_VOLTAGE_DELTA
                    + index * HW_PERF_STATE_BLOCK_STRIDE
                    + repeat * 4,
                HW_PERF_MEMORY_VOLTAGE[index],
            );
        }
        put_u32(
            out,
            HW_PERF_GROUP_BASES[1]
                + HW_PERF_VOLTAGE_DELTA
                + index * HW_PERF_STATE_BLOCK_STRIDE,
            HW_PERF_CORE_VOLTAGE[index],
        );
        put_u32(
            out,
            HW_PERF_GROUP_BASES[1]
                + HW_PERF_MEMORY_VOLTAGE_DELTA
                + index * HW_PERF_STATE_BLOCK_STRIDE,
            HW_PERF_MEMORY_VOLTAGE[index],
        );
    }

    // The acceptance block: derived from the work-channel count rather than
    // copied, so a constructed descriptor needs nothing from a capture here.
    put_u16(out, HW_ACCEPT_CHANNEL_COUNT, WORK_CHANNEL_COUNT as u16);
    for group in HW_ACCEPT_FLAG_GROUPS {
        for (index, value) in HW_ACCEPT_FLAG_PATTERN.iter().enumerate() {
            put_u32(out, group + index * 4, *value);
        }
    }
    for channel in 0..WORK_CHANNEL_COUNT {
        put_u32(out, HW_ACCEPT_UNASSIGNED_TABLE + channel * 4, 0xffff_ffff);
    }
    for offset in HW_ACCEPT_TRAILING_ONES {
        // NB +0x2600 is also DAT_178648, a firmware GATE: FUN_e004's per-DM
        // KSM completion-block arm runs only when (a == 0 || DAT_178648 == 0).
        // Writing 0 here to let that arm run was TESTED and the driver failed
        // probe with -22. I first wrote that our own initdata validation
        // rejected it; that was an invention -- nothing in the runtime path
        // validates this field, and the cause of the -22 was never
        // established. What settles it instead is the live 25G83 capture:
        // native's hardware-data block reads 1 at +0x2600 and 1 at +0x2608,
        // the same as ours, so this is not a divergence.
        put_u32(out, offset, 1);
    }

    put_u32(
        out,
        G17P_KSM_COMPLETION_TABLE_PREFIX_OFFSET,
        G17P_KSM_COMPLETION_TABLE_PREFIX,
    );
    for (aliases, ordinal) in completion_aliases
        .iter()
        .zip(G17P_KSM_COMPLETION_PRESENT_ORDINALS)
    {
        let base = G17P_KSM_COMPLETION_TABLE_OFFSET
            + ordinal as usize * G17P_KSM_COMPLETION_DESCRIPTOR_STRIDE;
        put_u64(out, base, aliases.low);
        put_u64(out, base + 0x08, aliases.high);
        put_u32(out, base + 0x10, G17P_KSM_COMPLETION_CAPACITY);
        put_u32(out, base + 0x14, G17P_KSM_COMPLETION_ENTRY_SIZE);
        put_u32(out, base + 0x18, ordinal);
        // +0x1c is the firmware-owned processed cursor and starts at zero.
    }

    put_u64(out, G17P_QOS_RESOURCE_LIVE_OFFSET, qos_resource_high);
    put_u64(
        out,
        G17P_SKSM_QID_CONFIG_2700_OFFSET,
        G17P_SKSM_QID_CONFIG_2700,
    );
    // The valid live G17P HWData object leaves +0x2708, +0x27a4 and +0x27ac
    // zero. The kext writes those only in a staging/version tail and B1 has no
    // consumer; the live QoS pointer is the independent +0x26ac field above.

    // Recorded platform constants. Descriptor *acceptance* survives without
    // them, but the device-control phase that follows does not. +0x2630 is
    // deliberately absent from the runs: it is firmware state and must start
    // zero (the fill above guarantees it).
    write_runs(out, HW_DATA_OPAQUE_RUNS);
    // The host->firmware transport selector.
    //
    // `AGXFirmware::initFirmwareData` @0xfffffe0008a20940 exports
    // `initdata[0x26f8] = accel[0x4f8] & 1`. That same bit decides, in
    // `AGX{TA,3D,CL}ChannelSKSM::submitBuffer`, whether the host publishes the
    // command record and the tag-14 AddKicks as CHANNEL COMMANDS through
    // `writeChannelCommandPointer` (bit set, `accel[0x10769] & 1 == 0`) or
    // instead pushes onto the host kick-trace ring and drives
    // `configureHardware`/`addKicks` over MMIO (bit clear).
    //
    // A live SKSM capture on this target shows tag-15 and tag-14 published as
    // channel commands for both render queues, so native runs with the bit
    // SET and this byte at 1. This driver uses the same channel-command
    // transport but left the byte at 0 -- it is outside every
    // `HW_DATA_OPAQUE_RUNS` extent and nothing else wrote it -- so the
    // firmware was told the host drives the MMIO bridge while the host in
    // fact speaks channel commands. Declare the transport we actually use.
    // MEASURED 2026-09-02, and it contradicts the reasoning above. Native's
    // complete hardware-data block (hwdata-gfx.txt, captured from
    // accel+0x5b0 -> +0x2b8 at submit3DChannel ENTRY, i.e. long after init)
    // carries ZERO at 0x26f8. The "native runs with the bit SET" conclusion was
    // inferred from seeing channel commands on the wire, not read off the
    // block, and the block disagrees. Publish what native publishes.
    if let Some(byte) = out.get_mut(HW_DATA_TRANSPORT_SELECTOR_OFFSET) {
        *byte = 0;
    }
    // The measured performance tables were encoded above. Their values are
    // target-scoped observations rather than a complete ADT derivation; see
    // [`PERF_TABLE_VALUES_ENCODED`].
    Ok(())
}

/// Encode the shared 0x1000-byte region C. Both roots point at the same
/// object; giving the secondary a private copy is a difference from what a
/// working host does.
pub(crate) fn encode_region_c(out: &mut [u8]) -> Result<(), InitdataError> {
    encode_region_c_for_profile(RootProfile::J700G17P, out)
}

/// Encode target-specific pre-handoff platform policy. M5's native policy is
/// kept separate from the experimental T8140 watchdog and idle overrides.
pub(crate) fn encode_region_c_for_profile(
    profile: RootProfile,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != REGION_C_SIZE {
        return Err(InitdataError::BufferSize);
    }
    if profile.is_g18g() {
        // Derived from the J813 bytes below through the proven +4 shift.
        return encode_g18g_26a428_region_c(out);
    }
    out.fill(0);
    write_runs(out, REGION_C_RUNS);
    if profile == RootProfile::J813G17G25G76 {
        // Scalars measured before each ASC received its root on two cold
        // J813 / 25G76 boots. These are a scoped reference configuration,
        // not generic M5 calibration or a power-management qualification.
        // The final pair moved by 0x20; leaving the T8140 pair in place would
        // initialize unrelated M5 fields.
        for (offset, value) in [
            (0x084, 0x0000_5eb0),
            (0x088, 0x0000_04b0),
            (0x08c, 0x0000_04b0),
            (0x098, 0x0000_04b0),
            (0x7c4, 0x0000_04e2),
            (0x7e0, 0x3f33_3333),
            (0x9a8, 0x0000_0333),
            (0xe38, 0),
            (0xe3c, 0),
            (0xe58, 0x0000_0300),
            (0xe5c, 0x0000_0100),
        ] {
            put_u32(out, offset, value);
        }
        return Ok(());
    }
    // regionC+0x9cc is the KSM progress-watchdog threshold for data master 2
    // (compute); the firmware copies it to 0x1778d0 and its handler starts
    // `cbz w20 -> ret`, so 0 disables the watchdog. We ship 0x32 (50) here and
    // leave the other data master's slot at 0 -- i.e. we arm the watchdog on
    // exactly the queue we submit to. When work cannot progress for any other
    // reason this turns a stall into a self-sustaining recovery loop.
    if crate::g18::fw::config::PROGRESS_WATCHDOG == 0 {
        put_u32(out, 0x9cc, 0);
    }
    // Native initializes the idle-descent selector to zero. Keep the old
    // alternate predicate only as an explicit diagnostic override.
    if crate::g18::fw::config::IDLE_SELECTOR != 0 {
        out[0xe21] = 1;
    }
    for (offset, value) in [
        (0x7c, 1),
        (0x80, 1),
        (0x9c, 0x3000_001d),
        (0xa0, 0x3100_0002),
    ] {
        put_u32(out, offset, value);
    }
    Ok(())
}

/// Overlay the target-scoped static third-page bytes and its one live state
/// pointer without disturbing hardware-data or overlapping view content.
pub(crate) fn encode_bundle_static(
    state_ptr: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() < HW_DATA_BUNDLE_MIN_ALLOC {
        return Err(InitdataError::BufferSize);
    }
    required_address(state_ptr)?;
    write_runs(out, BUNDLE_STATIC_RUNS);
    for (index, runs) in BUNDLE_VIEW_RUNS.iter().enumerate() {
        let base = BUNDLE_VIEW_OFFSETS[index];
        let valid = BUNDLE_VIEW_EXTENTS[index];
        for (offset, data) in *runs {
            if offset + data.len() > valid {
                continue;
            }
            let start = base + offset;
            out[start..start + data.len()].copy_from_slice(data);
        }
    }
    for (view, offset, count, trailing_ff) in BUNDLE_VIEW_U16_REPEATS {
        let base = BUNDLE_VIEW_OFFSETS[*view] + offset;
        let end = base + *count * 2 + usize::from(*trailing_ff);
        if end > BUNDLE_VIEW_OFFSETS[*view] + BUNDLE_VIEW_EXTENTS[*view] {
            return Err(InitdataError::BundleAllocationTooSmall);
        }
        for index in 0..*count {
            out[base + index * 2..base + index * 2 + 2].copy_from_slice(&[0xff, 0x7f]);
        }
        if *trailing_ff {
            out[base + *count * 2] = 0xff;
        }
    }
    put_u64(out, 0x8ed8, state_ptr);
    Ok(())
}

/// Encode one 0x80-byte status block in its pre-init state: u32 1 at `+0x04`,
/// everything else zero. `+0x10`/`+0x14` are post-acknowledgement firmware
/// state and must be zero at handoff.
pub(crate) fn encode_status_block(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != STATUS_BLOCK_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    put_u32(out, STATUS_BLOCK_ONE_OFFSET, 1);
    Ok(())
}

/// Encode the complete pre-init primary status/config object.
pub(crate) fn encode_primary_status_b(
    fwctl_state: u64,
    fwctl_ring: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != PRIMARY_STATUS_B_OBJECT_SIZE {
        return Err(InitdataError::BufferSize);
    }
    required_address(fwctl_state)?;
    required_address(fwctl_ring)?;
    out.fill(0);
    encode_status_block(&mut out[..STATUS_BLOCK_SIZE])?;
    put_u64(out, PRIMARY_STATUS_B_FWCTL_STATE, fwctl_state);
    put_u64(out, PRIMARY_STATUS_B_FWCTL_RING, fwctl_ring);
    put_u32(out, PRIMARY_STATUS_B_CONFIG_HEADER, 1);
    for (offset, data) in PRIMARY_STATUS_B_CONFIG_RUNS {
        let start = PRIMARY_STATUS_B_CONFIG_OFFSET + offset;
        out[start..start + data.len()].copy_from_slice(data);
    }
    Ok(())
}

/// Encode the secondary root's independent `+0xc0` target.
pub(crate) fn encode_secondary_root_extra_1(out: &mut [u8]) -> Result<(), InitdataError> {
    encode_secondary_root_extra_1_for_profile(RootProfile::J700G17P, out)
}

pub(crate) fn encode_secondary_root_extra_1_for_profile(
    profile: RootProfile,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != STATUS_BLOCK_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    write_runs(out, SECONDARY_ROOT_EXTRA_1_RUNS);
    // G18G: KEXT_PROVEN identical store geometry for this 0x60 object
    // (26A428 vs 25G76 R_60 stores, all at the same offsets); the J813 values
    // are carried unchanged (INFERRED, listed in G18G_26A428_UNPROVEN).
    if matches!(profile, RootProfile::J813G17G25G76) || profile.is_g18g() {
        put_u32(out, 0x10, 2);
    }
    Ok(())
}

// --- GPU firmware fast-die controller (GFX1) --------------------------------
//
// KEXT_PROVEN (25G83 AGXG17P):
// - AGXAcceleratorG17P::populateFastDieDeviceConfigData (0x88c7af0) sets target
//   110.0 C and sensor mask 0xaa0 when accel[0x4a0] == 0x1f, else 0x82a.  It
//   zeroes the per-sensor array for each mask bit.  It panics ("Not all MTR
//   Sensors required for FastDie are enabled") only when the host MTR-enable word
//   accel[0xe978] is non-zero and lacks a required bit.
// - The base populate (0x883d41c) takes optional ADT gpu-fast-die0-sensor-mask64/
//   -sensor-mask/-target (default 101.0 C, overridden above).  setupConfig reads
//   the optional gpu-fast-die0-release-temp/-prop-tgt-delta/-proportional-gain/
//   -integral-gain and gpu-fastdie-mrc-period.  The J700 ADT carries none of
//   them.
// - initPowerAndPerformanceData copies the staged values into
//   - the GFX1 power object: `[[fw+0x2d8]+0x6678]`; GFX1 reaches it through
//     secondary main +0x471 = bundle view 2 + 0xe40;
//   - the 0x6a8-byte secondary-root +0xb8 object (`fw+0xcd8`), which Linux
//     places at primary status B + 0xe440.
// FW_PROVEN (fw1 25G83 b100, 0x174d0):
// - When the +0xb8 object's +0xc4 flag is set, GFX1 copies its +0xc8..+0xe4
//   fields into the power object.  Native updateLifetimeServoTempTarget and
//   updateFastDieController set that flag after writing the fields.
// - GFX1 takes the lower of that target and an external target, then programs
//   each masked MTR sensor's threshold registers (stride 0x200, +0x1000/+0x1004/
//   +0x1014) at (target - delta) / 100 C.
// The recorded Linux runs already carry every non-zero value native publishes
// here (see the test `fast_die_config_matches_native`).

/// Offset of the GFX1 power object inside the hardware-data bundle.
pub(crate) const FASTDIE_OBJECT_BUNDLE_OFFSET: usize =
    BUNDLE_VIEW_OFFSETS[SECONDARY_EXTRA_VIEW_INDEX] + SECONDARY_EXTRA_VIEW_OFFSET;
pub(crate) const FASTDIE_OBJ_MASK: usize = 0x2310;
pub(crate) const FASTDIE_OBJ_RELEASE_CC: usize = 0x2318;
pub(crate) const FASTDIE_OBJ_KI_DT: usize = 0x233c;
pub(crate) const FASTDIE_OBJ_KP: usize = 0x234c;
pub(crate) const FASTDIE_OBJ_DELTA_CC: usize = 0x2364;
pub(crate) const FASTDIE_OBJ_TARGET_CC: usize = 0x236c;
pub(crate) const FASTDIE_OBJ_STATE: usize = 0x25bc;
/// Offsets inside the secondary-root +0xb8 object (status B + 0xe440 here).
pub(crate) const FASTDIE_LIVE_UPDATE: usize = 0xc4;
pub(crate) const FASTDIE_LIVE_TARGET_CC: usize = 0xc8;
pub(crate) const FASTDIE_LIVE_RELEASE_CC: usize = 0xcc;
pub(crate) const FASTDIE_LIVE_KP: usize = 0xd8;
pub(crate) const FASTDIE_LIVE_KI_DT: usize = 0xdc;
/// Native G17P target, 0.01 C.
pub(crate) const FASTDIE_NATIVE_TARGET_CC: u32 = 11_000;
/// Diagnostic overrides may only lower the target, never below this, so the
/// recorded 80.00 C release stays below it.
pub(crate) const FASTDIE_MIN_TARGET_CC: u32 = 8_500;

/// Effective fast-die configuration as encoded (raw words; floats as bits).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct FastDieConfig {
    pub(crate) mask: u64,
    pub(crate) target_cc: u32,
    pub(crate) release_cc: u32,
    pub(crate) delta_cc: u32,
    pub(crate) kp_bits: u32,
    pub(crate) ki_dt_bits: u32,
    pub(crate) state: u32,
    pub(crate) live_target_cc: u32,
    pub(crate) live_release_cc: u32,
    pub(crate) live_kp_bits: u32,
    pub(crate) live_ki_dt_bits: u32,
    pub(crate) live_update: u32,
}

/// Read the fast-die fields from an encoded bundle and status-B object.
pub(crate) fn fast_die_config(bundle: &[u8], status_b: &[u8]) -> Result<FastDieConfig, InitdataError> {
    let o = FASTDIE_OBJECT_BUNDLE_OFFSET;
    let l = PRIMARY_STATUS_B_CONFIG_OFFSET;
    if bundle.len() < o + FASTDIE_OBJ_STATE + 4 || status_b.len() < l + FASTDIE_LIVE_KI_DT + 4 {
        return Err(InitdataError::BufferSize);
    }
    Ok(FastDieConfig {
        mask: get_u64(bundle, o + FASTDIE_OBJ_MASK),
        target_cc: get_u32(bundle, o + FASTDIE_OBJ_TARGET_CC),
        release_cc: get_u32(bundle, o + FASTDIE_OBJ_RELEASE_CC),
        delta_cc: get_u32(bundle, o + FASTDIE_OBJ_DELTA_CC),
        kp_bits: get_u32(bundle, o + FASTDIE_OBJ_KP),
        ki_dt_bits: get_u32(bundle, o + FASTDIE_OBJ_KI_DT),
        state: get_u32(bundle, o + FASTDIE_OBJ_STATE),
        live_target_cc: get_u32(status_b, l + FASTDIE_LIVE_TARGET_CC),
        live_release_cc: get_u32(status_b, l + FASTDIE_LIVE_RELEASE_CC),
        live_kp_bits: get_u32(status_b, l + FASTDIE_LIVE_KP),
        live_ki_dt_bits: get_u32(status_b, l + FASTDIE_LIVE_KI_DT),
        live_update: get_u32(status_b, l + FASTDIE_LIVE_UPDATE),
    })
}

/// Validate a diagnostic target: 0 keeps native; otherwise lower-only within
/// [`FASTDIE_MIN_TARGET_CC`]..=[`FASTDIE_NATIVE_TARGET_CC`].
pub(crate) fn fast_die_target_override(requested_cc: u32) -> Result<Option<u32>, InitdataError> {
    match requested_cc {
        0 => Ok(None),
        t if (FASTDIE_MIN_TARGET_CC..=FASTDIE_NATIVE_TARGET_CC).contains(&t) => Ok(Some(t)),
        _ => Err(InitdataError::BufferSize),
    }
}

/// Apply a validated target to the power object copy (pre-boot encode only).
pub(crate) fn apply_fast_die_target_bundle(bundle: &mut [u8], target_cc: u32) -> Result<(), InitdataError> {
    let at = FASTDIE_OBJECT_BUNDLE_OFFSET + FASTDIE_OBJ_TARGET_CC;
    if bundle.len() < at + 4 {
        return Err(InitdataError::BufferSize);
    }
    put_u32(bundle, at, target_cc);
    Ok(())
}

/// Apply a validated target to the +0xb8 live object copy (pre-boot encode
/// only; no update flag: firmware starts from these words).
pub(crate) fn apply_fast_die_target_status_b(status_b: &mut [u8], target_cc: u32) -> Result<(), InitdataError> {
    let at = PRIMARY_STATUS_B_CONFIG_OFFSET + FASTDIE_LIVE_TARGET_CC;
    if status_b.len() < at + 4 {
        return Err(InitdataError::BufferSize);
    }
    put_u32(status_b, at, target_cc);
    Ok(())
}

/// Validate the cross-instance invariants established before either ASC is
/// kicked. The two roots are not placed freely: the secondary's sits exactly
/// [`SECONDARY_ROOT_DELTA`] above the primary's.
pub(crate) fn validate_pair(
    primary: &InstanceAddresses,
    secondary: &InstanceAddresses,
) -> Result<(), InitdataError> {
    if secondary.root != primary.root + SECONDARY_ROOT_DELTA {
        return Err(InitdataError::PairRootSeparation);
    }
    if primary.main_config == secondary.main_config || primary.status_a == secondary.status_a {
        return Err(InitdataError::PairAliasedObject);
    }
    Ok(())
}

/// Validate the per-instance halves of the ADT shared region: the secondary
/// maps it from the primary's base plus [`SECONDARY_SHARED_REGION_DELTA`].
pub(crate) fn validate_shared_region_bases(
    primary_base: u64,
    secondary_base: u64,
) -> Result<(), InitdataError> {
    if secondary_base != primary_base + SECONDARY_SHARED_REGION_DELTA {
        return Err(InitdataError::SharedRegionSeparation);
    }
    Ok(())
}

/// Reproduce the firmware's root acceptance checks.
///
/// The version-word gate is re-derived from our own J700 image
/// (`__TEXT+0x1e8a8`); a mismatch is answered with "Host and FW Interface is
/// mismatched" on the crashlog endpoint. The `+0x2c` nonzero gate was proven
/// on the M5 parser and the field is constant 1 in every T8140 capture; its
/// T8140 parse-gate status has not been independently re-derived, but a host
/// must write 1 there regardless. Passing this does not mean the rest of the
/// descriptor is complete.
pub(crate) fn check_firmware_root_gates(root: &[u8]) -> Result<(), InitdataError> {
    check_firmware_root_gates_for_profile(RootProfile::J700G17P, root)
}

pub(crate) fn check_firmware_root_gates_for_profile(
    profile: RootProfile,
    root: &[u8],
) -> Result<(), InitdataError> {
    if root.len() != ROOT_SIZE_PRIMARY && root.len() != ROOT_SIZE_SECONDARY {
        return Err(InitdataError::BufferSize);
    }
    // G18G: both roles are copied as 0xc8 bytes (FW_PROVEN b000 0x21664).
    if profile.is_g18g() && root.len() != G18G_26A428_ROOT_SIZE {
        return Err(InitdataError::BufferSize);
    }
    if get_u64(root, ROOT_VERSION) != profile.version_word() {
        return Err(InitdataError::InterfaceMismatch);
    }
    if get_u32(root, ROOT_CONSTANT_2C) == 0 {
        return Err(InitdataError::HostMappedAllocationsDisabled);
    }
    Ok(())
}

/// Recorded T8140 hardware-data platform constants (values known, meanings
/// not; derived from the hardware-validated `g17p_initdata.py`
/// `HWDATA_CONSTANTS`). 29 sparse runs, 262 bytes total. `+0x2630` is
/// deliberately absent: it is firmware state, not host configuration.
///
/// Native fixture pointer bytes are not platform constants. The original
/// +0x26ad run included seven bytes of the captured QoS allocation address;
/// replaying it after encode_hw_data's live pointer store silently replaced
/// the owned allocation with that unrelated address. Keep only its adjacent
/// state words here and encode the complete pointer from its retained owner.
///
/// +0x0f08 is zero in native too. That field is the firmware's ISR mask
/// selector -- FUN_1e858 copies it to DAT_1785f4, and FUN_215cc picks the
/// interrupt-enable word from it as
///     DAT_1785f4 == 2 ? 0x0e00620038 : 0x0a00620038
/// storing it to DAT_179328, which the ISR ANDs against MMIO[0x10a10]. Writing
/// 2 here was tested and does exactly that (0x10a08 measured 0x0e00620038,
/// bit 34 enabled) and changes nothing else, so leaving it unset is right and
/// this part is not the variant-2 configuration.
///
/// A caution for anyone diffing this table again: it has TWENTY-NINE runs,
/// several spanning many lines. A naive `\(0x..,\s*&\[(.*?)\]\)` regex
/// silently matches only 19 of them and mis-associates offsets, which
/// manufactures differences that are not there and, if used to rewrite the
/// table, corrupts it into 17 runs / 292 bytes and makes probe fail with -22.
/// Parse by matching brackets, and check the 29/262 totals before trusting a
/// comparison.
pub(crate) const HW_DATA_OPAQUE_RUNS: &[(usize, &[u8])] = &[
    (0x0004, &[0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff]),
    (
        0x0014,
        &[
            0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x80,
            0xff, 0xff, 0xff, 0x02, 0x00, 0x00, 0x00, 0x00, 0x40, 0x81, 0x21, 0xfc, 0xff, 0xff,
        ],
    ),
    (0x00e0, &[0x08, 0x20]),
    (0x00ea, &[0x08, 0x20]),
    (
        0x00f4,
        &[
            0x08, 0x20, 0x00, 0x00, 0xcb, 0x24, 0x00, 0x00, 0xfa, 0x2c, 0x8a, 0xc9, 0xcb, 0x24,
            0xf6, 0xf4, 0x17, 0xe9, 0x77, 0x18, 0xcb, 0x24, 0xd9, 0x38, 0x00, 0x00, 0xab, 0xbd,
        ],
    ),
    (
        0x02d9,
        &[
            0x20, 0xf8, 0xff, 0xdb, 0x2c, 0x2d, 0xd3, 0x00, 0x20, 0x00, 0xf5, 0x26, 0xe9, 0xda,
            0x21, 0x00, 0x20, 0xb6, 0x38, 0x08, 0x00, 0x42, 0xc7,
        ],
    ),
    (0x03e0, &[0xe0, 0x7f]),
    (0x03ea, &[0xe0, 0x7f]),
    (0x03f4, &[0xe0, 0x7f, 0x00, 0x00, 0x00, 0x80]),
    (0x0403, &[0x80]),
    (0x040d, &[0x80]),
    (
        0x05d8,
        &[
            0x45, 0x26, 0x23, 0x4b, 0x98, 0x0e, 0x00, 0x00, 0x5f, 0xea, 0xa2, 0xd5, 0xff, 0x3f,
            0x00, 0x40, 0x00, 0x40, 0x5e, 0xca, 0xa2, 0xf5, 0x00, 0x40,
        ],
    ),
    (
        0x0e94,
        &[
            0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
            0x00, 0x00, 0x01,
        ],
    ),
    (
        0x0eb8,
        &[
            0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x5d, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
            0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
        ],
    ),
    (0x0f04, &[0x1f]),
    // NOTE, do not "fix" this by adding 0x0f08 without evidence. That u32 is
    // the firmware's ISR mask selector: FUN_1e858 copies hw-data +0xf08 to
    // DAT_1785f4 (store at b1 0x1e9e8), and FUN_215cc picks the interrupt
    // enable word from it with
    //     csel: DAT_1785f4 == 2 ? 0x0e00620038 : 0x0a00620038
    // storing the result to DAT_179328, which the ISR at 0x216f0 ANDs against
    // MMIO[0x10a10] to decide which interrupts to service. The field is absent
    // here, so it reads zero and 0x10a08 measures 0x0a00620038 -- the variant
    // without bit 34.
    //
    // TESTED 2026-09-02: writing 2 here does exactly what the disassembly
    // says -- 0x10a08 measured 0x0e00620038, bit 34 enabled -- and changes
    // nothing else. The render still stalls with bit 4 clear, no KTrace 2,
    // fragment status 4. Reverted because there is no oracle saying this part
    // is variant 2 and it alters firmware interrupt behaviour on a guess.
    // Both variants already enable bits 3, 4 and 5, so neither withholds
    // new-work.
    (0x0f24, &[0x04]),
    (0x0f34, &[0x01, 0x00, 0x00, 0x00, 0x01]),
    (0x0f4c, &[0x31]),
    (0x0f6c, &[0x01]),
    (0x0f88, &[0x06, 0x00, 0x00, 0x00, 0x01]),
    (0x0fac, &[0x01]),
    (
        0x0fb8,
        &[
            0x1e, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x0a,
        ],
    ),
    (0x1cd8, &[0x0a]),
    (
        0x2544,
        &[
            0x04, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x07, 0x00,
            0x00, 0x00, 0x07,
        ],
    ),
    (
        0x2568,
        &[0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01],
    ),
    // +0x26ac..+0x26b3 is the live QoS pointer, never captured allocation bytes.
    (0x26b4, &[0x01, 0x00, 0x00, 0x00, 0x01]),
    (0x26e8, &[0x01]),
    (0x3d98, &[0xff, 0xff, 0xff, 0xff]),
    (0x3db0, &[0xff, 0xff, 0xff, 0xff]),
];

/// Recorded T8140 region-C platform tuning constants (values known, meanings
/// not; generated from the hardware-validated `g17p_initdata.py`
/// `REGION_C_CONSTANTS`). 12 sparse runs, 180 bytes total. The final run
/// initializes the byte preceding the host power-assert counter and the
/// counter itself to their native cold value of zero.
pub(crate) const REGION_C_RUNS: &[(usize, &[u8])] = &[
    (0x0024, &[0xb8, 0x0b, 0x00, 0x00]),
    (
        0x0030,
        &[
            0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0x0054,
        &[
            0xff, 0xff, 0x28, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00,
        ],
    ),
    (0x0078, &[0x01, 0x00, 0x00, 0x00]),
    (
        0x0084,
        &[
            0x4e, 0x25, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00,
        ],
    ),
    (0x0098, &[0xe8, 0x03, 0x00, 0x00]),
    (
        0x07c4,
        &[
            0xd0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0xcd, 0xcc, 0x4c, 0x3f, 0xcd, 0xcc,
            0x4c, 0x3e, 0x66, 0x66, 0x66, 0x3f, 0xcd, 0xcc, 0xcc, 0x3d, 0x00, 0x00, 0x80, 0x3e,
            0x9a, 0x99, 0x19, 0x3f, 0x66, 0x66, 0x66, 0x3f, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00,
            0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
        ],
    ),
    (
        0x0998,
        &[
            0x28, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0xfa, 0x00, 0x00, 0x00, 0x01, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0x09b8,
        &[
            0x02, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
        ],
    ),
    (0x0e1c, &[0x00, 0x01, 0x00, 0x00]),
    (0x0e38, &[0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00]),
    (0x0e50, &[0x00, 0x00, 0x00, 0x00]),
];

pub(crate) const BUNDLE_VIEW_RUNS: [&[(usize, &[u8])]; 5] = [
    &[
        (0x1658, &[0xff, 0xff, 0xff, 0xff]),
        (0x1670, &[0xff, 0xff, 0xff, 0xff]),
    ],
    &[
        (0xa18, &[0xff, 0xff, 0xff, 0xff]),
        (0xa30, &[0xff, 0xff, 0xff, 0xff]),
    ],
    &[
        (0xe45, &[0xdc, 0x05, 0x00, 0x00, 0xdc, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f]),
        (0xe6c, &[0x01, 0x00, 0x00, 0x00, 0x01]),
        (0xe80, &[0x64, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64]),
        (0xea0, &[0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01]),
        (0xec0, &[0x64, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64]),
        (0xf00, &[0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f]),
        (0x17f8, &[0x71, 0x02]),
        (0x1804, &[0x9f, 0x2e, 0x7f, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x54, 0x61, 0x51, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x86, 0x95, 0xa5, 0x3c]),
        (0x1821, &[0x38, 0x15, 0x46, 0xdb, 0x0f, 0xa9, 0x40, 0x00, 0x00, 0x00, 0x00, 0x44, 0x73, 0xd6, 0xbd, 0x28, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xe8, 0x03]),
        (0x1844, &[0x4e, 0x25]),
        (0x1878, &[0xe8, 0x03]),
        (0x18b0, &[0x04]),
        (0x18c6, &[0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc, 0xcc, 0x40]),
        (0x18da, &[0x80, 0x47, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0xe8, 0x03]),
        (0x18fc, &[0x4e, 0x25]),
        (0x1908, &[0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05]),
        (0x1968, &[0x5c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc, 0x4c, 0x3f, 0xb6, 0xf3, 0x7d, 0x3f, 0xcd, 0xcc, 0x4c, 0x3e, 0x6f, 0x12, 0x03, 0x3c, 0xd8, 0xd4, 0x69, 0x3f, 0xd8, 0xd4, 0x69, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbe, 0x42, 0xc3, 0xf5, 0x64, 0x40, 0xc3, 0xf5, 0x64, 0x40, 0x36, 0x94, 0x17, 0x41, 0x64, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x64]),
        (0x19cc, &[0x5c]),
        (0x1a00, &[0x64]),
        (0x1a20, &[0x01, 0x04, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x1f, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xa0, 0x0f, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xd0, 0x07, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x01]),
        (0x1aa4, &[0x01]),
        (0x1ad0, &[0x01]),
        (0x30da, &[0x80, 0x47, 0x00, 0x00, 0x20, 0x42, 0x00, 0x00, 0x7a, 0x44, 0xbe, 0x05]),
        (0x30f8, &[0x28]),
        (0x3102, &[0xc8, 0x42, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc, 0x4c, 0x3f, 0xcd, 0xcc, 0x4c, 0x3e]),
        (0x3150, &[0x2a, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04]),
        (0x3176, &[0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x19, 0x04, 0x16, 0x41]),
        (0x318a, &[0x80, 0x47, 0x52, 0xb8, 0xa2, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xe8, 0x03]),
        (0x31ac, &[0xf8, 0x2a]),
        (0x31e0, &[0xe8, 0x03]),
        (0x33fc, &[0x06, 0x00, 0x00, 0x42]),
        (0x3612, &[0x52, 0xb8, 0x7e, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x0a, 0xd7, 0xa3, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x43]),
        (0x3630, &[0x80, 0x47, 0x00, 0x00, 0xc8, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc8, 0xba, 0x84, 0x03, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3b, 0x45, 0xac, 0x26]),
        (0x3686, &[0xe8, 0x03]),
        (0x36a2, &[0xc8]),
        (0x3758, &[0xe8, 0x03]),
        (0x376c, &[0x01]),
        (0x37a0, &[0x01, 0x01, 0x00, 0x00, 0x00, 0x04]),
        (0x37bd, &[0x3f, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01]),
        (0x37dd, &[0x40, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01]),
        (0x37fd, &[0x41, 0x02]),
        (0x3848, &[0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4e, 0x25, 0x00, 0x00, 0x4e, 0x25, 0x00, 0x00, 0x4e, 0x25]),
        (0x386a, &[0x80, 0x3f, 0x04, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05]),
        (0x3944, &[0x3c]),
        (0x3950, &[0xef, 0xee, 0x6e, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x89, 0x88, 0x88, 0x3d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f]),
        (0x396e, &[0x80, 0x47, 0x00, 0x00, 0x88, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x15, 0x46, 0x4e, 0x25]),
        (0x399c, &[0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0x57]),
        (0x39c4, &[0xe8, 0x03]),
        (0x3a04, &[0x01]),
        (0x3ad4, &[0x32, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x8e, 0x63, 0x3f, 0xab, 0xaa, 0x2a, 0x3f, 0x39, 0x8e, 0xe3, 0x3d, 0xab, 0xaa, 0xaa, 0x3e, 0xcd, 0xcc, 0x4c, 0xbf, 0xcd, 0xcc, 0x4c, 0xbf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x47, 0x00, 0x00, 0xa0, 0xc0, 0x00, 0x00, 0xa0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x64]),
        (0x3b1e, &[0x7a, 0x46, 0x78, 0x05]),
        (0x3b2c, &[0x90, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0xbc, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0x11]),
        (0x3b4a, &[0x80, 0x47]),
        (0x3b70, &[0x80, 0x3e, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xc4, 0x09, 0x00, 0x00, 0x0d, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x32]),
        (0x3bba, &[0x80, 0x47]),
        (0x3bc8, &[0x28, 0x00, 0x00, 0x00, 0xe8, 0x03]),
    ],
    &[
        (0x15c5, &[0xdc, 0x05, 0x00, 0x00, 0xdc, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f]),
        (0x15ec, &[0x01, 0x00, 0x00, 0x00, 0x01]),
        (0x1600, &[0x64, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64]),
        (0x1620, &[0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01]),
        (0x1640, &[0x64, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64]),
        (0x1680, &[0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f]),
        (0x1f78, &[0x71, 0x02]),
        (0x1f84, &[0x9f, 0x2e, 0x7f, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x54, 0x61, 0x51, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x86, 0x95, 0xa5, 0x3c]),
        (0x1fa1, &[0x38, 0x15, 0x46, 0xdb, 0x0f, 0xa9, 0x40, 0x00, 0x00, 0x00, 0x00, 0x44, 0x73, 0xd6, 0xbd, 0x28, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xe8, 0x03]),
        (0x1fc4, &[0x4e, 0x25]),
        (0x1ff8, &[0xe8, 0x03]),
        (0x2030, &[0x04]),
        (0x2046, &[0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc, 0xcc, 0x40]),
        (0x205a, &[0x80, 0x47, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0xe8, 0x03]),
        (0x207c, &[0x4e, 0x25]),
        (0x2088, &[0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05]),
        (0x20e8, &[0x5c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc, 0x4c, 0x3f, 0xb6, 0xf3, 0x7d, 0x3f, 0xcd, 0xcc, 0x4c, 0x3e, 0x6f, 0x12, 0x03, 0x3c, 0xd8, 0xd4, 0x69, 0x3f, 0xd8, 0xd4, 0x69, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbe, 0x42, 0xc3, 0xf5, 0x64, 0x40, 0xc3, 0xf5, 0x64, 0x40, 0x36, 0x94, 0x17, 0x41, 0x64, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x64]),
        (0x214c, &[0x5c]),
        (0x2180, &[0x64]),
        (0x21a0, &[0x01, 0x04, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x1f, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xa0, 0x0f, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xd0, 0x07, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x01]),
        (0x2224, &[0x01]),
        (0x2250, &[0x01]),
    ],
    &[
        (0x1545, &[0xdc, 0x05, 0x00, 0x00, 0xdc, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f]),
        (0x156c, &[0x01, 0x00, 0x00, 0x00, 0x01]),
        (0x1580, &[0x64, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64]),
        (0x15a0, &[0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01]),
        (0x15c0, &[0x64, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64]),
        (0x1600, &[0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f, 0x5c, 0x8f, 0x82, 0x3f]),
        (0x1ef8, &[0x71, 0x02]),
        (0x1f04, &[0x9f, 0x2e, 0x7f, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x54, 0x61, 0x51, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x86, 0x95, 0xa5, 0x3c]),
        (0x1f21, &[0x38, 0x15, 0x46, 0xdb, 0x0f, 0xa9, 0x40, 0x00, 0x00, 0x00, 0x00, 0x44, 0x73, 0xd6, 0xbd, 0x28, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xe8, 0x03]),
        (0x1f44, &[0x4e, 0x25]),
        (0x1f78, &[0xe8, 0x03]),
        (0x1fb0, &[0x04]),
        (0x1fc6, &[0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc, 0xcc, 0x40]),
        (0x1fda, &[0x80, 0x47, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0xe8, 0x03]),
        (0x1ffc, &[0x4e, 0x25]),
        (0x2008, &[0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05]),
        (0x2068, &[0x5c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcd, 0xcc, 0x4c, 0x3f, 0xb6, 0xf3, 0x7d, 0x3f, 0xcd, 0xcc, 0x4c, 0x3e, 0x6f, 0x12, 0x03, 0x3c, 0xd8, 0xd4, 0x69, 0x3f, 0xd8, 0xd4, 0x69, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbe, 0x42, 0xc3, 0xf5, 0x64, 0x40, 0xc3, 0xf5, 0x64, 0x40, 0x36, 0x94, 0x17, 0x41, 0x64, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0x64]),
        (0x20cc, &[0x5c]),
        (0x2100, &[0x64]),
        (0x2120, &[0x01, 0x04, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x1f, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xa0, 0x0f, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xd0, 0x07, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x01]),
        (0x21a4, &[0x01]),
        (0x21d0, &[0x01]),
    ],
];
pub(crate) const BUNDLE_VIEW_U16_REPEATS: &[(usize, usize, usize, bool)] = &[
    (2, 0x2ad7, 512, false),
    (3, 0x3257, 148, true),
    (4, 0x31d7, 148, true),
];

pub(crate) const BUNDLE_STATIC_RUNS: &[(usize, &[u8])] = &[
    (0x80ac, &[0x06]),
    (0x80b8, &[0x01]),
    (0x8177, &[0xac, 0x0d]),
    (0x817b, &[0xe8, 0x03]),
    (0x817f, &[0xb8, 0x0b]),
    (0x8183, &[0xe8, 0x03]),
    (0x8187, &[0xe8, 0x03]),
    (0x818b, &[0x64]),
    (0x818f, &[0x20]),
    (0x81b4, &[0x04]),
    (0x81c2, &[0x78, 0x3f]),
    (0x81cb, &[0x3d]),
    (0x81d2, &[0xa0, 0x40]),
    (0x81de, &[0x80, 0x47]),
    (0x81e2, &[0x70, 0x41]),
    (0x81ec, &[0x84, 0x03]),
    (0x81f0, &[0xe8, 0x03]),
    (0x81f4, &[0xe8, 0x03]),
    (0x8200, &[0xe8, 0x03]),
    (0x8234, &[0xe8, 0x03]),
    (0x8274, &[0x64]),
    (0x8290, &[0x2a, 0x08]),
    (0x8298, &[0x7d]),
    (0x829c, &[0x01]),
    (0x8ee0, &[0xae, 0x18]),
    (0x8ee8, &[0x01]),
    (0xaeec, &[0xcd, 0xcc, 0xc8, 0x41, 0xcd, 0xcc, 0xc8, 0x41]),
    (0xb5e4, &[0x01]),
    (0xb5e8, &[0xf4, 0x01]),
    (0xb5ec, &[0x06]),
    (0xb5f0, &[0xd4, 0x30]),
    (0xb5f4, &[0xd4, 0x30]),
    (0xb5f8, &[0xd4, 0x30]),
    (0xb5fc, &[0xd4, 0x30]),
    (0xb600, &[0xd4, 0x30]),
    (0xb604, &[0xd4, 0x30]),
    (0xb608, &[0xa0, 0x0f]),
    (0xb61c, &[0xd4, 0x30]),
    (0xb620, &[0xd4, 0x30]),
    (0xb624, &[0xd4, 0x30]),
    (0xb628, &[0xd4, 0x30]),
    (0xb62c, &[0xd4, 0x30]),
    (0xb630, &[0x01]),
    (0xb7a8, &[0x01]),
    (0xb7b4, &[0xcd, 0xcc, 0x4c, 0x42]),
    (0xb7f5, &[0x80, 0x06, 0x44]),
    (0xb834, &[0x9a, 0x99, 0xc9, 0x41]),
    (0xb8e0, &[0x6a, 0x18]),
];

pub(crate) const PRIMARY_STATUS_B_CONFIG_RUNS: &[(usize, &[u8])] = &[
    (0x010, &[0x0f]),
    (0x017, &[0x3f]),
    (0x01a, &[0x88, 0x40, 0x28]),
    (0x020, &[0x01]),
    (0x0c8, &[0xf8, 0x2a]),
    (0x0cc, &[0x40, 0x1f]),
    (0x0d8, &[0x52, 0xb8, 0xa2, 0x41, 0x19, 0x04, 0x16, 0x41]),
    (0x0f0, &[0xac, 0x26]),
    (0x0f4, &[0xc8]),
    (0x10a, &[0xc8, 0x42]),
    (0x10e, &[0xc8, 0x43, 0xc8]),
    (0x128, &[0x01]),
    (0x12c, &[0x16, 0x26]),
    (0x133, &[0x3f, 0xcd, 0xcc, 0xcc, 0x40]),
    (0x14c, &[0x01]),
    (0x1c0, &[0x01]),
    (0x1c4, &[0x01]),
    (0x1c8, &[0x04]),
    (0x1cc, &[0x01]),
    (0x1d0, &[0x01]),
    (0x1d4, &[0x01]),
    (0x1d8, &[0x01]),
    (0x4e4, &[0x01]),
    (0x4e8, &[0xf4, 0x01]),
    (0x4ec, &[0x06]),
    (0x4f0, &[0xd4, 0x30]),
    (0x4f4, &[0xd4, 0x30]),
    (0x4f8, &[0xd4, 0x30]),
    (0x4fc, &[0xd4, 0x30]),
    (0x500, &[0xd4, 0x30]),
    (0x504, &[0xd4, 0x30]),
    (0x51c, &[0xd4, 0x30]),
    (0x520, &[0xd4, 0x30]),
    (0x524, &[0xd4, 0x30]),
    (0x528, &[0xd4, 0x30]),
    (0x52c, &[0xd4, 0x30]),
    (0x530, &[0x01]),
    (0x58c, &[0x06]),
    (0x590, &[0xac, 0x0d]),
    (0x594, &[0xe8, 0x03]),
    (0x598, &[0xb8, 0x0b]),
    (0x59c, &[0x64]),
    (0x5cc, &[0x01]),
    (0x5d0, &[0x04]),
    (0x5dc, &[0x40, 0x1f]),
    (0x5e0, &[0xc8]),
    (0x5e4, &[0xa0, 0x0f]),
    (0x5e8, &[0xc8]),
    (0x5ec, &[0xd0, 0x07]),
    (0x5f0, &[0xc8]),
    (0x5f4, &[0xe8, 0x03]),
    (0x5f8, &[0xc8]),
    (0x606, &[0x70, 0x41]),
    (0x60a, &[0xa0, 0x40, 0x20]),
    (0x610, &[0xe8, 0x03]),
    (0x620, &[0x84, 0x03]),
];

pub(crate) const SECONDARY_ROOT_EXTRA_1_RUNS: &[(usize, &[u8])] = &[
    (0x14, &[0x01]),
    (0x2c, &[0x01]),
    (0x30, &[0x01]),
    (0x40, &[0x01]),
    (0x4c, &[0x01]),
    (0x50, &[0x6a, 0x18]),
];

// =====================================================================================
// J873 / T8152 / G18G / 26A428 (RTKit_release-3514.0.15)
// =====================================================================================
//
// Plan M1 steps 1.1-1.7. Everything below is selected by
// `RootProfile::J873G18G26A428` or carries a `G18G_26A428`/`g18g_26a428` name.
// Nothing above changes its output for J700 or J813: the M5 constants stay
// literal, and `g18_initdata_test.rs` proves that the layout functions below,
// run over the 25G76 kext tables, reproduce those constants bit for bit, and
// that every J700/J813 encoder output hash is unchanged.
//
// Evidence tags:
//   FW_PROVEN   read off armfw_g18g-b000 / armfw1_g18g-b010 (J873 loads the B
//               images; ADT iop nub UUIDs match them)
//   KEXT_PROVEN read off the 26A428 AGXG18G kext in the kernelcache
//   ADT_PROVEN  read off the J873 ADT (full38-live-adt.bin)
//   INFERRED    carried from the measured J813/25G76 reference through a
//               proven relocation, or matched by analogy; the value itself is
//               not proven for 26A428
//   GUESSED     no evidence for the value; a placeholder
// A relocation band's tag qualifies the offset mapping, not the carried value.
// Every INFERRED/GUESSED value is also a row of [`G18G_26A428_UNPROVEN`].

/// Root `+0x00`: u16 {0x04c0, 0x83a0, 0x4339, 0x0c8b}. FW_PROVEN: b000
/// 0x2164c-0x21660 and b010 0x294f0-0x29504 compare root+0 with this word and
/// abort with "Host and FW Interface is mismatched" otherwise. KEXT_PROVEN:
/// AGXArmFirmware::initFirmwareData (0xfffffe0008c31f6c) writes it to both
/// roots.
pub(crate) const G18G_26A428_INITDATA_VERSION_WORD: u64 = 0x0c8b_4339_83a0_04c0;
/// Both roots are 0xc8 bytes. FW_PROVEN: each image copies 0xc8 bytes of its
/// root (b000 0x21664-0x216a8) and requires root+0x2c != 0 (b000 0x216ac).
/// Root field offsets are unchanged from 25G76 (KEXT_PROVEN g18g_key.c:3748).
pub(crate) const G18G_26A428_ROOT_SIZE: usize = 0xc8;

/// Provenance of one G18G relocation band or unproven field.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) enum Evidence {
    FirmwareProven,
    KextProven,
    AdtProven,
    Inferred,
    Guessed,
}

// --- Kext allocation tables and the packing rule -----------------------------------

/// `AGXFirmware::allocFirmwareData` allocation-size table, 24 u64.
/// KEXT_PROVEN: 25G76 __const 0xfffffe0007139820. Index names are the
/// `KX_*` constants below.
pub(crate) const KEXT_ALLOCATION_SIZES_25G76: [usize; 24] = [
    0x2708, 0x0c18, 0x1048, 0x0e10, 0x6a30, 0x0068, 0x0800, 0x0088, 0x1800, 0x04c0, 0x4000, 0x0030,
    0xe440, 0x06a8, 0x0020, 0x01b0, 0x4800, 0x28800, 0x79800, 0x9000, 0x0060, 0x0008, 0x0e78, 0x00e4,
];
/// KEXT_PROVEN: 26A428 __const 0xfffffe000714f0d8. Only entries 0 (hw_data),
/// 4 (perf/policy), 12 (status B), 13 (status config) and 22 (region C)
/// differ from 25G76.
pub(crate) const KEXT_ALLOCATION_SIZES_26A428: [usize; 24] = [
    0x2848, 0x0c18, 0x1048, 0x0e10, 0x7800, 0x0068, 0x0800, 0x0088, 0x1800, 0x04c0, 0x4000, 0x0030,
    0xe730, 0x06d0, 0x0020, 0x01b0, 0x4800, 0x28800, 0x79800, 0x9000, 0x0060, 0x0008, 0x0e80, 0x00e4,
];
pub(crate) const KX_HW_DATA: usize = 0;
pub(crate) const KX_BLOCK_C18: usize = 1;
pub(crate) const KX_BLOCK_1048: usize = 2;
pub(crate) const KX_BLOCK_E10: usize = 3;
pub(crate) const KX_POLICY: usize = 4;
pub(crate) const KX_BLOCK_68: usize = 5;
pub(crate) const KX_BLOCK_800: usize = 6;
pub(crate) const KX_BLOCK_88: usize = 7;
pub(crate) const KX_DM_RING: usize = 8;
pub(crate) const KX_MAIN_CONFIG: usize = 9;
pub(crate) const KX_CONTROL_RING: usize = 10;
pub(crate) const KX_RING_STATE: usize = 11;
pub(crate) const KX_STATUS_B: usize = 12;
pub(crate) const KX_STATUS_CONFIG: usize = 13;
pub(crate) const KX_STATUS_A: usize = 14;
pub(crate) const KX_LOG_STATE: usize = 15;
pub(crate) const KX_RING_4800: usize = 16;
pub(crate) const KX_BUFFER_28800: usize = 17;
pub(crate) const KX_BUFFER_79800: usize = 18;
pub(crate) const KX_RING_9000: usize = 19;
pub(crate) const KX_SECONDARY_EXTRA_1: usize = 20;
pub(crate) const KX_POLICY_STATE: usize = 21;
pub(crate) const KX_REGION_C: usize = 22;
pub(crate) const KX_AON_PTD: usize = 23;
/// `getSizeOfFWBRNTable` returns 0 in both kexts (KEXT_PROVEN
/// 0xfffffe00089ee74c / 0xfffffe0008c507e4).
pub(crate) const KEXT_BRN_TABLE_SIZE: usize = 0;
/// Both shared allocations are rounded to one 16 KiB page (allocateSharedData).
pub(crate) const KEXT_ALLOCATION_PAGE: usize = 0x4000;
pub(crate) const SHARED_ALLOCATION_ENTRIES: usize = 25;
pub(crate) const PRIVATE_ALLOCATION_ENTRIES: usize = 38;
const ROLE_SET_ENTRIES: usize = 10;

const fn round_up_usize(value: usize, alignment: usize) -> usize {
    (value + alignment - 1) & !(alignment - 1)
}

/// The allocateSharedData packing rule (KEXT_PROVEN 0xfffffe0008c038d8):
/// each next entry starts at `(previous offset + previous size + 0x3f) & !0x3f`.
pub(crate) const fn next_packed_allocation(offset: usize, size: usize) -> usize {
    round_up_usize(offset + size, 0x40)
}

/// Offsets of `sizes` packed by [`next_packed_allocation`], the first at 0.
pub(crate) const fn packed_allocation_offsets<const N: usize>(sizes: &[usize; N]) -> [usize; N] {
    let mut out = [0usize; N];
    let mut index = 1;
    while index < N {
        out[index] = next_packed_allocation(out[index - 1], sizes[index - 1]);
        index += 1;
    }
    out
}

/// The 25 entries of shared allocation #1 in allocation order: hw_data, the
/// 0xc18/0x1048/0xe10 blocks, perf/policy, the 0x68/0x800 blocks, the BRN
/// table, the 0x88 block, twelve data-master rings, then main config and
/// device-control ring for role 0 and for role 1 (KEXT_PROVEN descriptor
/// build, 25G76 g17g_key.c:1224-1310 / 26A428 g18g_key.c:1236-1320).
pub(crate) const fn shared_allocation_sizes(table: &[usize; 24]) -> [usize; SHARED_ALLOCATION_ENTRIES] {
    let mut sizes = [0usize; SHARED_ALLOCATION_ENTRIES];
    sizes[0] = table[KX_HW_DATA];
    sizes[1] = table[KX_BLOCK_C18];
    sizes[2] = table[KX_BLOCK_1048];
    sizes[3] = table[KX_BLOCK_E10];
    sizes[4] = table[KX_POLICY];
    sizes[5] = table[KX_BLOCK_68];
    sizes[6] = table[KX_BLOCK_800];
    sizes[7] = KEXT_BRN_TABLE_SIZE;
    sizes[8] = table[KX_BLOCK_88];
    let mut ring = 0;
    while ring < WORK_CHANNEL_COUNT {
        sizes[9 + ring] = table[KX_DM_RING];
        ring += 1;
    }
    sizes[21] = table[KX_MAIN_CONFIG];
    sizes[22] = table[KX_CONTROL_RING];
    sizes[23] = table[KX_MAIN_CONFIG];
    sizes[24] = table[KX_CONTROL_RING];
    sizes
}

/// The 38 entries of shared allocation #2 (the private cluster): twelve
/// data-master ring states and the role-0 device-control state (0x30 each),
/// status B, status config, the ten-entry per-role set for role 0 and role 1,
/// the role-1 device-control state, the 0x60 secondary-root object and the
/// 8-byte policy state (KEXT_PROVEN 25G76 g17g_key.c:1318-1450 / 26A428
/// g18g_key.c:1330-1460).
pub(crate) const fn private_allocation_sizes(table: &[usize; 24]) -> [usize; PRIVATE_ALLOCATION_ENTRIES] {
    let mut sizes = [0usize; PRIVATE_ALLOCATION_ENTRIES];
    let mut state = 0;
    while state <= WORK_CHANNEL_COUNT {
        sizes[state] = table[KX_RING_STATE];
        state += 1;
    }
    sizes[13] = table[KX_STATUS_B];
    sizes[14] = table[KX_STATUS_CONFIG];
    let role_set = [
        table[KX_STATUS_A], table[KX_RING_STATE], table[KX_LOG_STATE], table[KX_RING_STATE],
        table[KX_RING_STATE], table[KX_RING_4800], table[KX_BUFFER_28800],
        table[KX_BUFFER_79800], table[KX_RING_9000], table[KX_RING_4800],
    ];
    let mut role = 0;
    while role < 2 {
        let mut entry = 0;
        while entry < ROLE_SET_ENTRIES {
            sizes[15 + role * ROLE_SET_ENTRIES + entry] = role_set[entry];
            entry += 1;
        }
        role += 1;
    }
    sizes[35] = table[KX_RING_STATE];
    sizes[36] = table[KX_SECONDARY_EXTRA_1];
    sizes[37] = table[KX_POLICY_STATE];
    sizes
}

/// Offsets (from the start of the shared allocation #1 backing) of every
/// object in it.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct SharedClusterLayout {
    pub(crate) hw_data: usize,
    pub(crate) block_c18: usize,
    pub(crate) block_1048: usize,
    pub(crate) block_e10: usize,
    pub(crate) policy: usize,
    pub(crate) block_68: usize,
    pub(crate) block_800: usize,
    pub(crate) brn_table: usize,
    pub(crate) block_88: usize,
    /// Data-master rings in allocation order (see [`WORK_CHANNEL_DM_INDEX`]).
    pub(crate) dm_rings: [usize; WORK_CHANNEL_COUNT],
    pub(crate) main_config: [usize; 2],
    pub(crate) control_ring: [usize; 2],
    pub(crate) end: usize,
    pub(crate) total: usize,
}

/// One role's ten-entry set in the private cluster.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct RoleSetLayout {
    /// Root `+0xa8` (0x20 bytes).
    pub(crate) status_a: usize,
    /// Event report state (report channel 13 state 0).
    pub(crate) event_state: usize,
    /// 0x1b0 firmware-log control block (report channel 13 state 2).
    pub(crate) log_state: usize,
    /// Trace report state (report channel 14 state 0).
    pub(crate) trace_state: usize,
    pub(crate) state_30: usize,
    /// 256 x 0x48 event ring (report channel 13 state 1).
    pub(crate) event_ring: usize,
    pub(crate) buffer_28800: usize,
    /// 0x79800 buffer (main +0x200 / channel 15 state 0).
    pub(crate) buffer_79800: usize,
    /// 512 x 0x48 trace ring (report channel 14 state 1).
    pub(crate) trace_ring: usize,
    pub(crate) buffer_4800: usize,
}

/// Offsets (from the start of shared allocation #2) of every object in it.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct PrivateClusterLayout {
    pub(crate) dm_ring_states: [usize; WORK_CHANNEL_COUNT],
    /// Role-0 device-control state (0x30).
    pub(crate) control_state: usize,
    /// Primary root `+0xb0`.
    pub(crate) status_b: usize,
    /// Secondary root `+0xb8`.
    pub(crate) status_config: usize,
    pub(crate) roles: [RoleSetLayout; 2],
    /// Role-1 device-control state (0x30).
    pub(crate) secondary_control_state: usize,
    /// Secondary root `+0xc0` (0x60 bytes).
    pub(crate) secondary_extra_1: usize,
    /// 8-byte state named by the policy block.
    pub(crate) policy_state: usize,
    pub(crate) end: usize,
    pub(crate) total: usize,
}

pub(crate) const fn shared_cluster_layout(table: &[usize; 24]) -> SharedClusterLayout {
    let sizes = shared_allocation_sizes(table);
    let offsets = packed_allocation_offsets(&sizes);
    let mut dm_rings = [0usize; WORK_CHANNEL_COUNT];
    let mut ring = 0;
    while ring < WORK_CHANNEL_COUNT {
        dm_rings[ring] = offsets[9 + ring];
        ring += 1;
    }
    let end = offsets[24] + sizes[24];
    SharedClusterLayout {
        hw_data: offsets[0],
        block_c18: offsets[1],
        block_1048: offsets[2],
        block_e10: offsets[3],
        policy: offsets[4],
        block_68: offsets[5],
        block_800: offsets[6],
        brn_table: offsets[7],
        block_88: offsets[8],
        dm_rings,
        main_config: [offsets[21], offsets[23]],
        control_ring: [offsets[22], offsets[24]],
        end,
        total: round_up_usize(end, KEXT_ALLOCATION_PAGE),
    }
}

const fn role_set_layout(offsets: &[usize; PRIVATE_ALLOCATION_ENTRIES], first: usize) -> RoleSetLayout {
    RoleSetLayout {
        status_a: offsets[first],
        event_state: offsets[first + 1],
        log_state: offsets[first + 2],
        trace_state: offsets[first + 3],
        state_30: offsets[first + 4],
        event_ring: offsets[first + 5],
        buffer_28800: offsets[first + 6],
        buffer_79800: offsets[first + 7],
        trace_ring: offsets[first + 8],
        buffer_4800: offsets[first + 9],
    }
}

pub(crate) const fn private_cluster_layout(table: &[usize; 24]) -> PrivateClusterLayout {
    let sizes = private_allocation_sizes(table);
    let offsets = packed_allocation_offsets(&sizes);
    let mut dm_ring_states = [0usize; WORK_CHANNEL_COUNT];
    let mut state = 0;
    while state < WORK_CHANNEL_COUNT {
        dm_ring_states[state] = offsets[state];
        state += 1;
    }
    let end = offsets[37] + sizes[37];
    PrivateClusterLayout {
        dm_ring_states,
        control_state: offsets[12],
        status_b: offsets[13],
        status_config: offsets[14],
        roles: [role_set_layout(&offsets, 15), role_set_layout(&offsets, 25)],
        secondary_control_state: offsets[35],
        secondary_extra_1: offsets[36],
        policy_state: offsets[37],
        end,
        total: round_up_usize(end, KEXT_ALLOCATION_PAGE),
    }
}

/// Data-master allocation index of each channel-table work entry (TA_0, 3D_0,
/// CL_0, TA_1, ...). A ring and its 0x30-byte state share the index, which is
/// why the native state grid [`WORK_STATE_GRID_OFFSETS`] is `index * 0x40`.
/// The host object order of both allocations is unchanged in 26A428 (the ring
/// descriptors are this+0x448.. in 25G76 and this+0x438.. in 26A428, same
/// sequence), so the permutation is INFERRED unchanged for G18G.
pub(crate) const WORK_CHANNEL_DM_INDEX: [usize; WORK_CHANNEL_COUNT] =
    [3, 7, 11, 2, 6, 10, 1, 5, 9, 0, 4, 8];

/// Work-ring bundle offsets in channel-table order for a shared layout.
pub(crate) const fn work_ring_offsets(layout: &SharedClusterLayout) -> [usize; WORK_CHANNEL_COUNT] {
    let mut rings = [0usize; WORK_CHANNEL_COUNT];
    let mut channel = 0;
    while channel < WORK_CHANNEL_COUNT {
        rings[channel] = layout.dm_rings[WORK_CHANNEL_DM_INDEX[channel]];
        channel += 1;
    }
    rings
}

/// The 25G76 layouts. Not used by the M5 encoders above (which keep their
/// literal constants); the regression tests prove they agree.
pub(crate) const M5_25G76_SHARED_LAYOUT: SharedClusterLayout =
    shared_cluster_layout(&KEXT_ALLOCATION_SIZES_25G76);
pub(crate) const M5_25G76_PRIVATE_LAYOUT: PrivateClusterLayout =
    private_cluster_layout(&KEXT_ALLOCATION_SIZES_25G76);
/// The 26A428 layouts: hw_data 0x2848 at 0, blocks at 0x2880/0x34c0/0x4540,
/// perf/policy 0x7800 at 0x5380, 0x68/0x800 at 0xcb80/0xcc00, BRN and 0x88 at
/// 0xd400, rings from 0xd4c0, main r0 0x1f4c0, control r0 0x1f980, main r1
/// 0x23980, control r1 0x23e40; status B 0x340, status config 0xea80, status
/// A 0xf180/0xc3440, tail 0x177700/0x177740/0x1777c0. Totals 0x28000 and
/// 0x178000, as in 25G76.
pub(crate) const G18G_26A428_SHARED_LAYOUT: SharedClusterLayout =
    shared_cluster_layout(&KEXT_ALLOCATION_SIZES_26A428);
pub(crate) const G18G_26A428_PRIVATE_LAYOUT: PrivateClusterLayout =
    private_cluster_layout(&KEXT_ALLOCATION_SIZES_26A428);

pub(crate) const G18G_26A428_SHARED_CLUSTER_SIZE: usize = G18G_26A428_SHARED_LAYOUT.total;
pub(crate) const G18G_26A428_PRIVATE_CLUSTER_SIZE: usize = G18G_26A428_PRIVATE_LAYOUT.total;
pub(crate) const G18G_26A428_HW_DATA_SIZE: usize = KEXT_ALLOCATION_SIZES_26A428[KX_HW_DATA];
pub(crate) const G18G_26A428_POLICY_SIZE: usize = KEXT_ALLOCATION_SIZES_26A428[KX_POLICY];
pub(crate) const G18G_26A428_STATUS_B_SIZE: usize = KEXT_ALLOCATION_SIZES_26A428[KX_STATUS_B];
pub(crate) const G18G_26A428_STATUS_CONFIG_SIZE: usize = KEXT_ALLOCATION_SIZES_26A428[KX_STATUS_CONFIG];
/// The kext allocates 0xe80 bytes of region C (25G76: 0xe78). The host maps
/// a whole page ([`REGION_C_SIZE`]) for both.
pub(crate) const G18G_26A428_REGION_C_ALLOCATION: usize = KEXT_ALLOCATION_SIZES_26A428[KX_REGION_C];
pub(crate) const G18G_26A428_AON_PTD_SIZE: usize = KEXT_ALLOCATION_SIZES_26A428[KX_AON_PTD];
/// Bytes of shared allocation #1 before the BRN table: hw_data plus the six
/// blocks the policy encoder owns (25G76 analog: [`M5_25G76_SHARED_PREFIX_SIZE`]).
pub(crate) const G18G_26A428_SHARED_PREFIX_SIZE: usize = G18G_26A428_SHARED_LAYOUT.brn_table;
/// Main `+0x254` views (KEXT_PROVEN 26A428 initFirmwareSharedData: the FW VAs
/// of the 0xc18, 0x1048, 0xe10, 0x68 and 0x800 blocks, this+0x310..0x338).
pub(crate) const G18G_26A428_BUNDLE_VIEW_OFFSETS: [usize; 5] = [
    G18G_26A428_SHARED_LAYOUT.block_c18,
    G18G_26A428_SHARED_LAYOUT.block_1048,
    G18G_26A428_SHARED_LAYOUT.block_e10,
    G18G_26A428_SHARED_LAYOUT.block_68,
    G18G_26A428_SHARED_LAYOUT.block_800,
];
pub(crate) const G18G_26A428_BUNDLE_VIEW_EXTENTS: [usize; 5] = [
    KEXT_ALLOCATION_SIZES_26A428[KX_BLOCK_C18],
    KEXT_ALLOCATION_SIZES_26A428[KX_BLOCK_1048],
    KEXT_ALLOCATION_SIZES_26A428[KX_BLOCK_E10],
    KEXT_ALLOCATION_SIZES_26A428[KX_BLOCK_68],
    KEXT_ALLOCATION_SIZES_26A428[KX_BLOCK_800],
];
pub(crate) const G18G_26A428_WORK_RING_BUNDLE_OFFSETS: [usize; WORK_CHANNEL_COUNT] =
    work_ring_offsets(&G18G_26A428_SHARED_LAYOUT);
pub(crate) const G18G_26A428_MAIN_BUNDLE_OFFSETS: [usize; 2] = G18G_26A428_SHARED_LAYOUT.main_config;
pub(crate) const G18G_26A428_CONTROL_RING_BUNDLE_OFFSETS: [usize; 2] =
    G18G_26A428_SHARED_LAYOUT.control_ring;
/// Role-1 main `+0x481`: FW VA of the perf/policy allocation (KEXT_PROVEN).
pub(crate) const G18G_26A428_MAIN_SECONDARY_POLICY_PTR: usize = 0x481;
/// Role-1 main `+0x300`. INFERRED: the J813 value (26A428 source accel+0x10a74).
pub(crate) const G18G_26A428_MAIN_SECONDARY_KIND: u32 = 0xc;

/// Private-cluster offsets the resources module and live boot need.
pub(crate) const G18G_26A428_PRIVATE_STATUS_A: [usize; 2] = [
    G18G_26A428_PRIVATE_LAYOUT.roles[0].status_a,
    G18G_26A428_PRIVATE_LAYOUT.roles[1].status_a,
];
/// Compact channel-state grid of each role: role 0 starts at the first ring
/// state; role 1's device-control state is the tail 0x30 object, so its
/// (control-only) grid base is that object minus the grid's control offset.
pub(crate) const G18G_26A428_PRIVATE_STATE_GRIDS: [usize; 2] = [
    G18G_26A428_PRIVATE_LAYOUT.dm_ring_states[0],
    G18G_26A428_PRIVATE_LAYOUT.secondary_control_state - WORK_STATE_GRID_OFFSETS[CONTROL_CHANNEL_INDEX],
];
/// Status config inside the status-B span (primary status B .. status A).
pub(crate) const G18G_26A428_STATUS_CONFIG_IN_STATUS_B: usize =
    G18G_26A428_PRIVATE_LAYOUT.status_config - G18G_26A428_PRIVATE_LAYOUT.status_b;
/// Bytes from primary status B to primary status A (the object M5 calls
/// [`PRIMARY_STATUS_B_OBJECT_SIZE`], 0xeb00 there).
pub(crate) const G18G_26A428_STATUS_B_SPAN: usize =
    G18G_26A428_PRIVATE_LAYOUT.roles[0].status_a - G18G_26A428_PRIVATE_LAYOUT.status_b;
/// The fwctl ring follows its 0x20-byte state in the ArmFirmware fwctl
/// allocation (KEXT_PROVEN {0x20, 0x1800} entries, 64-byte packing).
pub(crate) const G18G_26A428_FWCTL_RING_OFFSET: usize = 0x40;

// --- Status B and status config (plan step 1.3) -----------------------------------

/// KEXT_PROVEN (25G76 0x48e0/0x48e8 stores moved +0x1e0): FW-control state
/// and ring pointers.
pub(crate) const G18G_26A428_STATUS_B_FWCTL_STATE: usize = 0x4ac0;
pub(crate) const G18G_26A428_STATUS_B_FWCTL_RING: usize = 0x4ac8;
/// FW_PROVEN u64 recovery epoch (25G76 +0x48f0). The recovery routine
/// (b000 0x6ef0) increments it as it halts (0x7fb8-0x7fc4), and posts the
/// value it had before as the type-4 event's first payload word
/// (0x79ac/0x7df0). EP 0x21 type 6 (b000 0x255b0-0x255dc) requests a host
/// recovery only when its low 44 bits equal it. Starts zero.
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_EPOCH: usize = 0x4ad0;
/// FW_PROVEN u32 recovery handshake state (25G76 +0x4900). Firmware writes 1
/// and spins until the host writes 2 (b000 0x7fb0-0x7fe0), recovers, then
/// writes 3 and spins until the host writes 0 (0x80fc-0x8114). Starts zero.
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_STATE: usize = 0x4ae0;
/// FW_PROVEN u32: nonzero when the host requested the recovery (b000 0x7fa8
/// stores w25 = block 0x68 +4 != 0; 25G76 +0x4910). A host recovery posts
/// no type-4 event.
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_HOST: usize = 0x4af0;
/// FW_PROVEN: 256 x 8-byte recovery-info entries (bit 0 = valid) and the
/// cause words after them. The routine clears 0x828 bytes from +0x4298 before
/// each firmware-detected recovery (b000 0x7df8-0x7e08) and writes the cause
/// at +0x4a9c..+0x4abf (0x7eb0-0x7ee8); 25G76 +0x40b8..+0x48df.
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_INFO: usize = 0x4298;
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_INFO_ENTRIES: usize = 0x100;
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_EMITTED: usize = 0x4a98;
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_SOURCE_COUNT: usize = 0x4aa0;
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_COMMAND: usize = 0x4aa8;
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_REASON: usize = 0x4aac;
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_DETAIL: usize = 0x4ab0;
pub(crate) const G18G_26A428_STATUS_B_RECOVERY_SLOT_MASK: usize = 0x4ab8;
/// FW_PROVEN: the 0x68-byte shared block published at main +0x26c is the
/// recovery request (b000 0x6f18-0x6f44, 0x255c8-0x255dc): +0 dispatcher
/// state, +4 host request, +0xc firmware request, +0x20..+0x4b cause.
pub(crate) const G18G_26A428_RECOVERY_REQUEST_WORDS: usize = 0x68 / 4;
/// The 0x800-byte shared block published at main +0x274: one
/// `{u32 valid, u32 pad, u64 stamp}` last-submitted entry per QID. Native
/// `restartWorkQueue` writes it before resuming a halted firmware, which
/// rebuilds queue progress from it (G17 analysis, g17_resources.rs).
pub(crate) const G18G_26A428_QUEUE_PROGRESS_ENTRIES: usize = 128;
pub(crate) const G18G_26A428_QUEUE_PROGRESS_STRIDE: usize = 0x10;
/// FW_PROVEN: the region-C loader reads it (b000 0x36a0/0x377c); KEXT_PROVEN
/// value 1 (25G76 +0xe434).
pub(crate) const G18G_26A428_STATUS_B_CONFIG_HEADER: usize = 0xe714;
/// KEXT_PROVEN new cross-link pair (g18g_key.c:813-816): u32 0 at +0xe71c and
/// the FW VA of status config +0x6c4 at +0xe720.
pub(crate) const G18G_26A428_STATUS_B_LINK_WORD: usize = 0xe71c;
pub(crate) const G18G_26A428_STATUS_B_LINK_PTR: usize = 0xe720;
/// KEXT_PROVEN: status config +0x6c4 = 1 and +0x6c8 = FW VA of status B +0xe71c.
pub(crate) const G18G_26A428_STATUS_CONFIG_LINK_WORD: usize = 0x6c4;
pub(crate) const G18G_26A428_STATUS_CONFIG_LINK_PTR: usize = 0x6c8;
/// KEXT_PROVEN submit*Channel polls (25G76 +0x670/+0x678/+0x680). Written by
/// firmware at run time; they must start zero.
pub(crate) const G18G_26A428_STATUS_CONFIG_SUBMIT_POLLS: [usize; 3] = [0x68c, 0x694, 0x69c];

// --- J873 / G18G / 26B123 (macOS 27.2, RTKit 3514.41.1) ---------------------------
//
// 26B123 runs the 26A428 graph with three differences:
//
// - the root version word gains bit 45, which both roles compare with root +0;
// - the perf/policy block grows from 0x7800 to 0x7860 and the status config
//   from 0x6d0 to 0x6d8; every other allocation keeps its size;
// - inside those two objects, fields move as described by
//   [`G18G_26B123_POLICY_BANDS`] and [`G18G_26B123_STATUS_CONFIG_BANDS`].
//
// hw_data, the roots, the main configs, status B and region C keep their field
// offsets.

/// FW_PROVEN 26B123: u16 {0x04c0, 0x83a0, 0x6339, 0x0c8b}.
pub(crate) const G18G_26B123_INITDATA_VERSION_WORD: u64 = 0x0c8b_6339_83a0_04c0;
/// KEXT_PROVEN 26B123 allocation sizes: entries 4 (perf/policy) and 13
/// (status config) differ from 26A428.
pub(crate) const KEXT_ALLOCATION_SIZES_26B123: [usize; 24] = [
    0x2848, 0x0c18, 0x1048, 0x0e10, 0x7860, 0x0068, 0x0800, 0x0088, 0x1800, 0x04c0, 0x4000, 0x0030,
    0xe730, 0x06d8, 0x0020, 0x01b0, 0x4800, 0x28800, 0x79800, 0x9000, 0x0060, 0x0008, 0x0e80, 0x00e4,
];
/// The policy grows by 0x60, so everything after it in shared allocation #1
/// moves by 0x80 (64-byte packing); the page total stays 0x28000.
pub(crate) const G18G_26B123_SHARED_LAYOUT: SharedClusterLayout =
    shared_cluster_layout(&KEXT_ALLOCATION_SIZES_26B123);
/// 0x6d8 still packs to the 0x700 that 0x6d0 took: the private cluster is
/// the 26A428 one.
pub(crate) const G18G_26B123_PRIVATE_LAYOUT: PrivateClusterLayout =
    private_cluster_layout(&KEXT_ALLOCATION_SIZES_26B123);
const _: () = assert!(G18G_26B123_SHARED_LAYOUT.total == G18G_26A428_SHARED_LAYOUT.total);
const _: () =
    assert!(G18G_26B123_SHARED_LAYOUT.block_68 == G18G_26A428_SHARED_LAYOUT.block_68 + 0x80);
const _: () = assert!(G18G_26B123_PRIVATE_LAYOUT.total == G18G_26A428_PRIVATE_LAYOUT.total);
const _: () = assert!(G18G_26B123_PRIVATE_LAYOUT.status_b == G18G_26A428_PRIVATE_LAYOUT.status_b);
const _: () = assert!(G18G_26B123_PRIVATE_LAYOUT.roles[0].status_a
    == G18G_26A428_PRIVATE_LAYOUT.roles[0].status_a);
const _: () = assert!(G18G_26B123_PRIVATE_LAYOUT.secondary_control_state
    == G18G_26A428_PRIVATE_LAYOUT.secondary_control_state);
const _: () = assert!(G18G_26B123_PRIVATE_LAYOUT.policy_state
    == G18G_26A428_PRIVATE_LAYOUT.policy_state);
pub(crate) const G18G_26B123_BUNDLE_VIEW_OFFSETS: [usize; 5] = [
    G18G_26B123_SHARED_LAYOUT.block_c18,
    G18G_26B123_SHARED_LAYOUT.block_1048,
    G18G_26B123_SHARED_LAYOUT.block_e10,
    G18G_26B123_SHARED_LAYOUT.block_68,
    G18G_26B123_SHARED_LAYOUT.block_800,
];
pub(crate) const G18G_26B123_WORK_RING_BUNDLE_OFFSETS: [usize; WORK_CHANNEL_COUNT] =
    work_ring_offsets(&G18G_26B123_SHARED_LAYOUT);

/// Perf/policy 26A428 -> 26B123 (KEXT_PROVEN).
///
/// - `[0x2f08, 0x2f2c)`: the GPU power controller's parameter words gain
///   one. +0x2f08 stays; +0x2f0c..+0x2f17 move by 0x10; the new u32 at
///   +0x2f28 stays zero. No machine source carries it, and with it zero
///   the 26B123 firmware boots, renders and keeps its throughput and
///   latency (J873g), as the 26A428 layout runs without the word.
/// - `[0x2f18, 0x2fb4)` holds nothing the host writes in either build.
/// - From +0x2fb4 to the last field the host writes in 26A428 (+0x6d98, u64)
///   everything moves by 0x18, among them the policy state pointer (+0x3d48),
///   the AON PTD pointer (+0x64f0) and the boot timestamp (+0x6c2c).
/// - Past +0x6da0 the host writes nothing; the firmware-written CSW history
///   and trace snapshots there moved by 0x60 (+0x716d -> +0x71cd, +0x73dd ->
///   +0x743d), so the remaining 0x48 bytes are inserted in that
///   host-unwritten range. The 26A428 encoding must be zero there.
pub(crate) const G18G_26B123_POLICY_BANDS: &[LayoutBand] = &[
    band(0x0000, 0x2f0c, 0, Evidence::KextProven),
    band(0x2f0c, 0x2f18, 0x10, Evidence::KextProven),
    band(0x2fb4, 0x6da0, 0x18, Evidence::KextProven),
];
/// Status config 26A428 -> 26B123 (KEXT_PROVEN). +0x59c is a
/// new word the host leaves zero (covered only by the q-zeroing at +0x590);
/// the 26A428 words from +0x59c move by 4; +0x5b8 is the power controller's
/// new parameter again, also left zero; the 26A428 words from +0x5b4 to the
/// end move by 8, among them the link word/pointer (+0x6cc/+0x6d0) and +0x644.
pub(crate) const G18G_26B123_STATUS_CONFIG_BANDS: &[LayoutBand] = &[
    band(0x000, 0x59c, 0, Evidence::KextProven),
    band(0x59c, 0x5b4, 4, Evidence::KextProven),
    band(0x5b4, 0x6d0, 8, Evidence::KextProven),
];

/// The G18G InitData layout values that differ between firmware builds.
#[derive(Debug)]
pub(crate) struct G18GLayouts {
    pub(crate) profile: RootProfile,
    pub(crate) version_word: u64,
    pub(crate) shared: SharedClusterLayout,
    pub(crate) private: PrivateClusterLayout,
    pub(crate) work_rings: [usize; WORK_CHANNEL_COUNT],
    pub(crate) policy_size: usize,
    pub(crate) status_config_size: usize,
    /// Bands from the 26A428 encoding; None for 26A428 itself.
    pub(crate) policy_bands: Option<&'static [LayoutBand]>,
    pub(crate) status_config_bands: Option<&'static [LayoutBand]>,
    pub(crate) status_config_link_word: usize,
    pub(crate) status_config_link_ptr: usize,
}

pub(crate) const G18G_26A428_LAYOUTS: G18GLayouts = G18GLayouts {
    profile: RootProfile::J873G18G26A428,
    version_word: G18G_26A428_INITDATA_VERSION_WORD,
    shared: G18G_26A428_SHARED_LAYOUT,
    private: G18G_26A428_PRIVATE_LAYOUT,
    work_rings: G18G_26A428_WORK_RING_BUNDLE_OFFSETS,
    policy_size: KEXT_ALLOCATION_SIZES_26A428[KX_POLICY],
    status_config_size: KEXT_ALLOCATION_SIZES_26A428[KX_STATUS_CONFIG],
    policy_bands: None,
    status_config_bands: None,
    status_config_link_word: G18G_26A428_STATUS_CONFIG_LINK_WORD,
    status_config_link_ptr: G18G_26A428_STATUS_CONFIG_LINK_PTR,
};

pub(crate) const G18G_26B123_LAYOUTS: G18GLayouts = G18GLayouts {
    profile: RootProfile::J873G18G26B123,
    version_word: G18G_26B123_INITDATA_VERSION_WORD,
    shared: G18G_26B123_SHARED_LAYOUT,
    private: G18G_26B123_PRIVATE_LAYOUT,
    work_rings: G18G_26B123_WORK_RING_BUNDLE_OFFSETS,
    policy_size: KEXT_ALLOCATION_SIZES_26B123[KX_POLICY],
    status_config_size: KEXT_ALLOCATION_SIZES_26B123[KX_STATUS_CONFIG],
    policy_bands: Some(G18G_26B123_POLICY_BANDS),
    status_config_bands: Some(G18G_26B123_STATUS_CONFIG_BANDS),
    status_config_link_word: G18G_26A428_STATUS_CONFIG_LINK_WORD + 8,
    status_config_link_ptr: G18G_26A428_STATUS_CONFIG_LINK_PTR + 8,
};

/// Move a block encoded in the 26A428 layout (its first `old_size` bytes) to
/// the layout `bands` describe, in place. Bytes no band carries must be zero;
/// the bytes a band leaves behind are zeroed.
pub(crate) fn relayout_by_bands(
    bands: &[LayoutBand],
    old_size: usize,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() < old_size {
        return Err(InitdataError::BufferSize);
    }
    for offset in 0..old_size {
        if out[offset] != 0 && relocate_by_bands(bands, offset).is_none() {
            return Err(InitdataError::BufferSize);
        }
    }
    // Highest band first: every band moves up, so a lower band's target
    // never overlaps a higher band's source that has not moved yet.
    let mut index = bands.len();
    while index > 0 {
        index -= 1;
        let band = bands[index];
        if band.delta == 0 {
            continue;
        }
        if band.delta < 0 {
            return Err(InitdataError::BufferSize);
        }
        let target = band.start + band.delta as usize;
        if target + (band.end - band.start) > out.len() {
            return Err(InitdataError::BufferSize);
        }
        out.copy_within(band.start..band.end, target);
        out[band.start..target.min(band.end)].fill(0);
    }
    Ok(())
}

// --- Relocation bands -------------------------------------------------------------

/// Bytes `[start, end)` of a 25G76 object move to `[start + delta, end + delta)`
/// of the 26A428 object.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct LayoutBand {
    pub(crate) start: usize,
    pub(crate) end: usize,
    pub(crate) delta: isize,
    pub(crate) evidence: Evidence,
}

const fn band(start: usize, end: usize, delta: isize, evidence: Evidence) -> LayoutBand {
    LayoutBand { start, end, delta, evidence }
}

/// The 26A428 offset of 25G76 byte `offset`, or None when no band carries it
/// (the byte was removed or its new place is unknown).
pub(crate) const fn relocate_by_bands(bands: &[LayoutBand], offset: usize) -> Option<usize> {
    let mut index = 0;
    while index < bands.len() {
        let band = bands[index];
        if band.start <= offset && offset < band.end {
            return Some((offset as isize + band.delta) as usize);
        }
        index += 1;
    }
    None
}

/// hw_data 25G76 -> 26A428. The offsets are FW_PROVEN by the init copier
/// (b000 0x21708-0x21d94 vs G17 0x1fb18-0x20124: every G17 read relocates onto
/// a b000 read, see `g18_initdata_test.rs`) and KEXT_PROVEN by the
/// initFirmwareData store alignment. The carried J813 values are INFERRED.
/// 0x640..0xe88 (the 53-slot 25G76 register map) is re-encoded per G18G slot,
/// not relocated.
pub(crate) const G18G_26A428_HW_DATA_BANDS: &[LayoutBand] = &[
    // Host prefix: range head (KEXT_PROVEN same five qwords), +0x28/+0x30/+0x638
    // qwords and the two 32 x 0x18 accelerator tables at +0x38/+0x338 (same
    // geometry in both kexts). Not read by the copier; J813 values carried.
    band(0x000, 0x640, 0, Evidence::Inferred),
    // u64 FW VA of accel+0x1d40 (26A428 writes it at +0xfa0 = "4000").
    band(0xe88, 0xe90, 0x118, Evidence::KextProven),
    // Chip block: the copier's 16-byte read moved 0xe90 -> 0xfb0.
    band(0xe90, 0xebc, 0x120, Evidence::FirmwareProven),
    // 0xebc: 25G76 u32 0 with no 26A428 counterpart (4 bytes removed).
    band(0xec0, 0xf5c, 0x11c, Evidence::FirmwareProven),
    band(0xf5c, 0xf60, 0x124, Evidence::KextProven),
    // 0xf60..0xf68: 25G76 u64 0 removed; 26A428 adds its own u64 0 at +0x1078.
    band(0xf68, 0xf98, 0x11c, Evidence::FirmwareProven),
    band(0xf98, 0xfac, 0x120, Evidence::FirmwareProven),
    // 0xfac: 25G76 flag (G17 copier read 0x1ff60, J813 value 1) has neither a
    // 26A428 store nor a b000 read: dropped.
    band(0xfb0, 0x25f8, 0x11c, Evidence::FirmwareProven),
    band(0x25f8, 0x26b4, 0x120, Evidence::FirmwareProven),
    band(0x26b4, 0x26f0, 0x128, Evidence::FirmwareProven),
    band(0x26f0, M5_25G76_HW_DATA_SIZE, 0x12c, Evidence::FirmwareProven),
];

/// Perf/policy block (relative to the block) 25G76 -> 26A428, from the
/// AGXFirmware::initPowerAndPerformanceData store alignment (KEXT_PROVEN at
/// the band ends; interior bytes without a matched store are INFERRED).
/// 0x6468..0x651c is deliberately absent: 26A428 inserts 0x680 bytes
/// somewhere between the AON PTD pointer (+0x6460 -> +0x64d8) and the boot
/// timestamp (+0x651c -> +0x6c14), and the J813 words there are dropped.
pub(crate) const G18G_26A428_POLICY_BANDS: &[LayoutBand] = &[
    band(0x0000, 0x009c, 0, Evidence::KextProven),
    // The 0x160-byte host-config memmove moved +0xa0 -> +0xb0.
    band(0x00a0, 0x0200, 0x10, Evidence::KextProven),
    band(0x09b8, 0x295c, 0x10, Evidence::KextProven),
    band(0x2960, 0x29c1, 0x29, Evidence::KextProven),
    band(0x2a01, 0x2a05, 0x8d, Evidence::KextProven),
    band(0x2a08, 0x2e5c, 0x90, Evidence::KextProven),
    band(0x2e6c, 0x2e70, 0x9c, Evidence::KextProven),
    // this+0x216c -> this+0x211c (the host object moved by -0x50).
    band(0x2e78, 0x2e7c, 0x74, Evidence::Inferred),
    band(0x2f37, 0x2f43, -0x2b, Evidence::KextProven),
    band(0x2f43, 0x2f53, 0x71, Evidence::KextProven),
    band(0x2f58, 0x3260, 0x70, Evidence::KextProven),
    // Policy state pointer and its neighbours (+0x3c98 -> +0x3d30).
    band(0x3c98, 0x3cac, 0x98, Evidence::KextProven),
    band(0x5cac, 0x5cb4, 0x98, Evidence::KextProven),
    band(0x63a4, 0x63f4, 0xa0, Evidence::KextProven),
    band(0x6458, 0x645c, 0x78, Evidence::KextProven),
    // AON PTD FW VA (this+0xf78 in 26A428).
    band(0x6460, 0x6468, 0x78, Evidence::KextProven),
    // Boot mach_absolute_time (+0x651c -> +0x6c14) and what follows it.
    band(0x651c, 0x66a8, 0x6f8, Evidence::KextProven),
    band(0x6a18, 0x6a1c, 0x710, Evidence::KextProven),
];

/// Region C 25G76 -> 26A428. FW_PROVEN: every read of the G17 region-C loader
/// (0x3068-0x330c) relocates onto a b000 loader read (0x3508-0x37c0) under
/// this map, and the only extra b000 read is the new field at +0x38. The kext
/// agrees: 26A428 stores 0x78 at +0x40 (25G76 +0x3c) and a feature bit at +0x38.
pub(crate) const G18G_26A428_REGION_C_BANDS: &[LayoutBand] = &[
    band(0x000, 0x038, 0, Evidence::FirmwareProven),
    band(0x038, 0xe78, 4, Evidence::FirmwareProven),
];

/// Status config 25G76 -> 26A428 (KEXT_PROVEN initFirmwareSharedData store
/// alignment). The J813 words live in the first, sixth and seventh bands.
pub(crate) const G18G_26A428_STATUS_CONFIG_BANDS: &[LayoutBand] = &[
    band(0x000, 0x1dc, 0, Evidence::KextProven),
    band(0x4e0, 0x518, 0xc, Evidence::KextProven),
    band(0x518, 0x528, 0x14, Evidence::KextProven),
    band(0x530, 0x534, 0xc, Evidence::KextProven),
    band(0x554, 0x584, 0xc, Evidence::KextProven),
    band(0x584, 0x5a8, 0x1c, Evidence::KextProven),
    band(0x5cc, 0x628, 0x18, Evidence::KextProven),
];

/// J873 secondary power-control inputs from eight native 26A428 capture views.
/// The original initPowerAndPerformanceData stores at 8c30740..8c30760
/// copy these u32 fields from accelerator+10c10+92c..948/950. GFX1 then
/// consumes them at b010 1071c onward. Do not inherit the J813 controller
/// settings here. Values and store geometry are qualified; semantic names
/// and any relationship to the outstanding compute stall remain unqualified.
const G18G_26A428_STATUS_CONFIG_POWER_WORDS: &[(usize, u32)] = &[
    (0x1c0, 0), (0x1c4, 1), (0x1c8, 1), (0x1cc, 4), (0x1d0, 1),
    (0x1d4, 1), (0x1d8, 1), (0x1dc, 1), (0x1e0, 29000),
];

/// Remaining fixed J873/26A428 status/config defaults. These typed scalar
/// corrections and the generated tables below make the complete 0x6d0-byte
/// allocation match all eight pre-publication native capture views. Preserve
/// the exact float bit patterns; individual calibration meanings are not
/// qualified. Never use post-run firmware counters as encoder defaults.
const G18G_26A428_STATUS_CONFIG_NATIVE_WORDS: &[(usize, u32)] = &[
    (0x0d8, 0x41a770a4), (0x0dc, 0x3d20045e), (0x12c, 0x8f16),
    (0x4e8, 0x00010001), (0x59c, 1), (0x5a0, 700), (0x5a4, 12),
    (0x5a8, 5000), (0x5ac, 2000), (0x5b0, 5000), (0x5b4, 0),
    (0x5b8, 0), (0x618, 0), (0x628, 1300), (0x638, 1200),
];

fn encode_g18g_26a428_status_config_defaults(out: &mut [u8]) {
    for words in [G18G_26A428_STATUS_CONFIG_POWER_WORDS, G18G_26A428_STATUS_CONFIG_NATIVE_WORDS] {
        for &(offset, value) in words {
            put_u32(out, offset, value);
        }
    }
    // Original host stores at 8c30790..8c30804 copy two 16x16 tables
    // from accelerator+111c4/+113c4, then two u16 enables at +4e8/+4ea.
    // Native J873 tables have zero first rows/columns and linear remaining
    // entries. Generate the values; do not carry the different J813 table.
    for row in 0..16 {
        for col in 0..16 {
            let active = row != 0 && col != 0;
            let wide = if active { (col * 0x1000 - 1) as u16 } else { 0 };
            let narrow = if active { (col * 0x10 - 1) as u8 } else { 0 };
            put_u16(out, 0x1e8 + 2 * (row * 16 + col), wide);
            out[0x3e8 + row * 16 + col] = narrow;
        }
    }
}

/// Copy every non-zero byte of `words` (u32 at `offset - source_base`) to its
/// relocated place in `out`. Bytes without a band are dropped.
fn write_relocated_words(
    words: &[(usize, u32)],
    source_base: usize,
    bands: &[LayoutBand],
    out: &mut [u8],
) -> Result<(), InitdataError> {
    for &(offset, value) in words {
        for (index, byte) in value.to_le_bytes().into_iter().enumerate() {
            if byte == 0 {
                continue;
            }
            let source = offset
                .checked_sub(source_base)
                .ok_or(InitdataError::BufferSize)?
                + index;
            if let Some(target) = relocate_by_bands(bands, source) {
                *out.get_mut(target).ok_or(InitdataError::BufferSize)? = byte;
            }
        }
    }
    Ok(())
}

// --- hw_data (plan step 1.4) --------------------------------------------------------

/// 60 register slots at `+0x640` (FW_PROVEN b000 0x236d8 walks 53 named slot
/// IDs from DATA 0xfffffc0000078bb8; KEXT_PROVEN the 26A428 copy loop runs
/// over 0x3c slots, 0x648..0xfa8).
pub(crate) const G18G_26A428_HW_REGISTER_MAP_SLOTS: usize = 60;
pub(crate) const G18G_26A428_HW_REGISTER_MAP_END: usize =
    HW_REGISTER_MAP_OFFSET + G18G_26A428_HW_REGISTER_MAP_SLOTS * HW_REGISTER_MAP_ENTRY_SIZE;
/// KEXT_PROVEN: initFirmwareData forces this slot's device VA (+0xcb0) to 0
/// (25G76: slot 36, +0xbe8; FenderRegsDFenderPowerTransG16G in both).
pub(crate) const G18G_26A428_ZEROED_REGISTER_SLOT: usize = 41;
/// FW_PROVEN: the copier's 16-byte read of the chip block moved 0xe90 -> 0xfb0.
pub(crate) const G18G_26A428_HW_CHIP_ID_OFFSET: usize = 0xfb0;
pub(crate) const G18G_26A428_HW_CHIP_ID: u32 = 0x8152;
/// KEXT_PROVEN: initPowerAndPerformanceData stores this+0xf10 (the number of
/// performance states; J813 +0xfc4 = 12 = its gpu-num-perf-states) here.
pub(crate) const G18G_26A428_HW_PERF_STATE_COUNT: usize = 0x10e0;
/// KEXT_PROVEN: 16 frequencies in MHz (accel+0x1b8f8.. / 1e6).
pub(crate) const G18G_26A428_HW_PERF_FREQUENCY: usize = 0x10e4;
/// FW_PROVEN: 16 rows of 0x40 bytes; the b000 loop reads lanes 0 and 1 of
/// each row (0x1124/0x1128 core, 0x1524/0x1528 SRAM; b000 0x218d0-0x219f0).
pub(crate) const G18G_26A428_HW_PERF_CORE_VOLTAGE: usize = 0x1124;
pub(crate) const G18G_26A428_HW_PERF_SRAM_VOLTAGE: usize = 0x1524;
pub(crate) const G18G_26A428_HW_PERF_ROWS: usize = 16;
pub(crate) const G18G_26A428_HW_PERF_ROW_STRIDE: usize = 0x40;
pub(crate) const G18G_26A428_HW_PERF_LANES: usize = 16;
pub(crate) const G18G_26A428_HW_PERF_SCALE: usize = 0x1964;
pub(crate) const G18G_26A428_HW_PERF_RELATIVE_A: usize = 0x19e4;
pub(crate) const G18G_26A428_HW_PERF_RELATIVE_B: usize = 0x1a24;
pub(crate) const G18G_26A428_HW_PERF_INDEX_MAP: usize = 0x1ae4;
/// FW_PROVEN: four 0x20-byte KSM completion descriptors (b000 0xf57c `mov
/// w9,#0x2734`), firmware cursor at +0x1c of each (b000 0x1cbc0..0x1cc48).
pub(crate) const G18G_26A428_KSM_COMPLETION_TABLE_OFFSET: usize = 0x2734;
pub(crate) const G18G_26A428_KSM_COMPLETION_CURSOR: usize = 0x1c;
/// FW_PROVEN: QoS pointer (b000 0xa0f0; firmware dereferences QoS+0xe00).
pub(crate) const G18G_26A428_QOS_POINTER_OFFSET: usize = 0x27cc;
/// KEXT_PROVEN new constant (26A428 initFirmwareData `hw+0x2714 = 0x96`).
/// FW_PROVEN numeric use: b000 0x23220 compares it with -1 and scales it by 24;
/// 0xbb90 logs it. (It is not the 25G76 +0x25f4 flag, which moves to +0x2710.)
pub(crate) const G18G_26A428_HW_2714_OFFSET: usize = 0x2714;
pub(crate) const G18G_26A428_HW_2714_VALUE: u32 = 0x96;
/// Fields that exist only in 26A428 (KEXT_PROVEN stores), with their size.
/// +0x2714 is KEXT_PROVEN. Captured scalars below override the zero defaults;
/// the native-owned pointer +0x27d4 remains unresolved and is not copied.
pub(crate) const G18G_26A428_HW_NEW_FIELDS: [(usize, usize); 11] = [
    (0x0fa8, 8), (0x1078, 8), (0x10b4, 4), (0x2714, 4), (0x27d4, 8), (0x2818, 4),
    (0x281e, 4), (0x2822, 8), (0x2832, 4), (0x283a, 4), (0x283e, 4),
];

/// Stable scalar values from both roles in macOS 26A428 captures 051/053/073/075.
/// Replace the J813 defaults and unknown zero enables only at these offsets.
/// Their values are observed; their complete semantics remain unqualified.
/// Failure boundaries: no pointers, timing values, voltage tables or register
/// writes are copied. audit-g18-hwdata-scalars.py checks all eight snapshots,
/// the float32 ladders, and the full opening records before a hardware trial.
pub(crate) const G18G_26A428_HW_CAPTURED_WORDS: [(usize, u32); 17] = [
    (0x0fb4, 1), (0x10d4, 0x1b), (0x10d8, 4), (0x10dc, 12),
    (0x283a, 1), (0x283e, 1),
    (0x0fa8, 0x8000), (0x0fac, 0), (0x0fb8, 1),
    (0x1044, 12), (0x10a4, 12), (0x2660, 0), (0x2684, 12),
    (0x2710, 0), (0x2720, 0), (0x2724, 1), (0x281e, 1),
];
/// The adjacent +0x281e word is unaligned; +0x281c is only a halfword.
pub(crate) const G18G_26A428_HW_CAPTURED_HALFWORDS: [(usize, u16); 1] = [(0x281c, 1)];

/// J873 ADT /arm-io/sgx perf-states: (MHz, core mV), 14 entries including
/// state 0. ADT_PROVEN.
pub(crate) const J873_GPU_PERF_STATES: [(u32, u32); 14] = [
    (0, 125), (502, 675), (669, 700), (834, 720), (1002, 755), (1090, 770), (1178, 795),
    (1266, 810), (1354, 840), (1442, 875), (1530, 915), (1620, 960), (1680, 975), (1735, 990),
];
/// J873 ADT perf-states-sram voltages (mV). ADT_PROVEN.
pub(crate) const J873_GPU_PERF_STATES_SRAM_MV: [u32; 14] =
    [775, 775, 775, 775, 775, 775, 795, 810, 840, 875, 915, 960, 975, 990];
/// J873 ADT gpu-num-perf-states. ADT_PROVEN.
pub(crate) const J873_GPU_NUM_PERF_STATES: u32 = 13;
/// INFERRED: the J813 per-state scale word (1.02f), one per state.
const G18G_26A428_PERF_SCALE_VALUE: u32 = 0x3f82_8f5c;
/// Native 26A428 relative-performance ladders, identical in both roles of
/// captures 051/053/073/075. These are IEEE754 float32 bit patterns, not u32
/// percentages. Store the bits without floating-point operations in kernel
/// context. All values are finite, monotonic, and bounded by 0..100; the
/// physical calibration formula is not yet derived. Voltage/frequency tables
/// above remain the independently verified J873 ADT values.
const G18G_26A428_PERF_RELATIVE_A: [u32; 14] =
    [0, 1100735714, 1103989076, 1107351648, 1109719030, 1110992894, 1112721914,
     1114199739, 1116076971, 1117492892, 1119211535, 1120006268, 1120204862, 1120403456];
const G18G_26A428_PERF_RELATIVE_B: [u32; 14] =
    [0, 0, 1096332557, 1104636123, 1109537981, 1111408920, 1113279858,
     1115150797, 1116353300, 1117288769, 1118224238, 1119180968, 1119818789, 1120403456];

/// The 26A428 slot of each 25G76 register slot (FW_PROVEN name tables: G17
/// DATA 0xfffffc0000073488, 47 rows; b000 DATA 0xfffffc0000078bb8, 53 rows).
/// None: 25G76 slot 8 (StreamingCodecRegisters) has no 26A428 slot. Unnamed
/// slots 17 and 21 keep their numbers; 25 -> 28 and 44/45/46 -> 50/51/52 are
/// INFERRED by position (44 -> 50 is confirmed by the identical die-1 PIO
/// record 0xd24000/0x100 in both kexts).
pub(crate) const G18G_26A428_SLOT_FROM_25G76: [Option<u8>; HW_REGISTER_MAP_SLOTS] = [
    Some(0), Some(1), Some(2), Some(3), Some(4), Some(5), Some(6), Some(7), None, Some(9),
    Some(10), Some(11), Some(12), Some(13), Some(14), Some(15), Some(16), Some(17), Some(18),
    Some(19), Some(20), Some(21), Some(22), Some(23), Some(24), Some(28), Some(31), Some(32),
    Some(33), Some(34), Some(35), Some(36), Some(37), Some(38), Some(39), Some(40), Some(41),
    Some(42), Some(43), Some(44), Some(45), Some(46), Some(47), Some(48), Some(50), Some(51),
    Some(52), Some(53), Some(54), Some(55), Some(56), Some(57), Some(58),
];

/// J873 moved the always-on blocks the 25G76 map names (AIC, MCC telemetry,
/// PMGR event log, AON PTD) down by this much (ADT_PROVEN); the SGX and GPU
/// PMGR banks did not move.
pub(crate) const J873_AON_REBASE: u64 = 0x8000_0000;

/// A register window before its device VA is assigned.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct RegisterWindowSpec {
    pub(crate) slot: usize,
    pub(crate) phys: u64,
    /// Instance count x instance size (hw_data slot +0x10).
    pub(crate) size_10: u32,
    /// Instance size (slot +0x14).
    pub(crate) size_14: u32,
    /// Offset from the GPU base for PIO-table windows (slot +0x18), else 0.
    pub(crate) unk_18: u64,
    /// accelerator entry flags & 2 (slot +0x20).
    pub(crate) flag: u32,
}

const fn spec(slot: usize, phys: u64, size: u32, unk_18: u64, flag: u32) -> RegisterWindowSpec {
    RegisterWindowSpec { slot, phys, size_10: size, size_14: size, unk_18, flag }
}

/// Assign firmware device VAs in slot order, from the register aperture base:
/// each window starts at the next 32 KiB boundary at least one 16 KiB guard
/// page past the previous window, plus its physical page offset. This rule
/// reproduces all 22 native J813/25G76 device VAs exactly (regression test);
/// for G18G the VAs are host-chosen (INFERRED: same rule), since firmware
/// reaches its registers only through this table.
pub(crate) const fn allocate_register_window_vas<const N: usize>(
    specs: &[RegisterWindowSpec; N],
) -> [M5RegisterWindow; N] {
    let mask = UAT_PAGE_SIZE as u64 - 1;
    let mut out = [M5RegisterWindow {
        slot: 0, phys: 0, device_va: 0, size_10: 0, size_14: 0, unk_18: 0, flag: 0,
    }; N];
    let mut next: u64 = 0;
    let mut index = 0;
    while index < N {
        let window = specs[index];
        let va = next + (window.phys & mask);
        out[index] = M5RegisterWindow {
            slot: window.slot,
            phys: window.phys,
            device_va: HW_REGISTER_APERTURE.start + va,
            size_10: window.size_10,
            size_14: window.size_14,
            unk_18: window.unk_18,
            flag: window.flag,
        };
        let end = (va + window.size_10 as u64 + mask) & !mask;
        next = (end + UAT_PAGE_SIZE as u64 + 0x7fff) & !0x7fff;
        index += 1;
    }
    out
}

/// The 22 mapped G18G windows, in slot order. Sources (26A428 kext):
/// - AcceleratorG::configureDevice (0xfffffe0008c490ac): slot 3 MCache
///   0x220104000 (0x140fb rounded, count = popcount(active-mcc-bit-vector),
///   absent on J873 -> 1), 9/10 telemetry at AON+0x3d0000/+0x3c0000, 14 PMS at
///   AON+0x280000 size 0x14f54 (25G76: 0x8000), 15 AONPTDSpace AON+0xc840000
///   0x24000, 29 DPEP 0x580001800/0x27f8 and 30 ADCLK 0x580ec2200/0xfc (both
///   new), 46 ThrtlStatCntrs GPU+0xe33800. Slots 10/14/29/30 get flags 0.
/// - PI_400 Accelerator::configureDevice (0xfffffe0008c45428): slot 22
///   GPU+0x1000000/0x8000 and slot 44 MTR GPU+0xde0000, one 0x15000 instance
///   (25G76: GPU+0xe08000, two 0x8000 instances).
/// - AGFPIOGFXHALRelativeOffsetTableG18G (0xfffffe0007152228, 12 records):
///   slot 17 GPU+0 size 0x36000 (25G76 0x21400), Fender slots 31..37/40 at the
///   25G76 offsets, new 35 (KSMFE, 0xd28000) and 49 (FenderRegsHWL, 0xe18000);
///   +0x18 is the table offset. 25G76 slot 50 (DpwrMXU, 0x14400) is gone.
/// - AGXAccelerator::configureDevice (base): flags 2 for the named slots,
///   0 for 49; slot 0 {flags 2, count 1} and the AIC SW-interrupt page (the
///   ADT meta-sw-interrupt, 0x301014048 on J873: ADT_PROVEN).
/// - Slot 24 (DVFMMgpu) is absent. Both roles in native 26A428 initialization
///   captures 051/053/073 leave all 0x28 bytes zero. Do not carry the J813
///   0x580ec0000 window into this profile or allocate firmware VA space for it.
/// The GPU base is the SGX reg 0x580000000 (unchanged, ADT_PROVEN); the AON
/// base moved 0x380000000 -> 0x300000000 ([`J873_AON_REBASE`]).
pub(crate) const G18G_26A428_REGISTER_WINDOW_SPECS: [RegisterWindowSpec; 21] = [
    spec(0, 0x3_0101_4000, 0x4000, 0, 2),
    spec(3, 0x2_2010_4000, 0x18000, 0, 2),
    spec(9, 0x3_003d_0000, 0x4000, 0, 2),
    spec(10, 0x3_003c_0000, 0x4000, 0, 0),
    spec(14, 0x3_0028_0000, 0x14f54, 0, 0),
    spec(15, 0x3_0c84_0000, 0x24000, 0, 2),
    spec(17, 0x5_8000_0000, 0x36000, 0, 2),
    spec(22, 0x5_8100_0000, 0x8000, 0, 2),
    spec(29, 0x5_8000_1800, 0x27f8, 0, 0),
    spec(30, 0x5_80ec_2200, 0xfc, 0, 0),
    spec(31, 0x5_80d0_4000, 0x8000, 0xd0_4000, 2),
    spec(32, 0x5_80d0_d000, 0x1000, 0xd0_d000, 2),
    spec(33, 0x5_80d5_0000, 0x10000, 0xd5_0000, 2),
    spec(34, 0x5_80d1_0000, 0x4000, 0xd1_0000, 2),
    spec(35, 0x5_80d2_8000, 0x4000, 0xd2_8000, 2),
    spec(36, 0x5_80d4_0000, 0x4000, 0xd4_0000, 2),
    spec(37, 0x5_80d6_0000, 0x8000, 0xd6_0000, 2),
    spec(40, 0x5_80e0_0000, 0x4000, 0xe0_0000, 2),
    spec(44, 0x5_80de_0000, 0x15000, 0, 2),
    spec(46, 0x5_80e3_3800, 0x4000, 0, 2),
    spec(49, 0x5_80e1_8000, 0x4000, 0xe1_8000, 0),
];
pub(crate) const G18G_26A428_REGISTER_WINDOWS: [M5RegisterWindow; 21] =
    allocate_register_window_vas(&G18G_26A428_REGISTER_WINDOW_SPECS);
/// Slots whose base-configured flags are 2 but which receive no physical
/// window (KEXT_PROVEN by decoding AGXAccelerator::configureDevice; the same
/// decode of the 25G76 function reproduces the J813 list exactly). Slot 47
/// (AVMR) lost its G-variant window in 26A428; slot 8 keeps a flag although
/// firmware no longer walks it.
pub(crate) const G18G_26A428_REGISTER_FLAG_ONLY_SLOTS: [usize; 20] =
    [2, 5, 6, 7, 8, 25, 26, 27, 38, 39, 42, 43, 45, 47, 48, 52, 53, 54, 55, 58];

/// Encode the 60-slot, 0x960-byte G18G register map. Construction only:
/// page tables for these windows are the caller's (see g18_resources.rs).
pub(crate) fn encode_g18g_26a428_register_map(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != G18G_26A428_HW_REGISTER_MAP_SLOTS * HW_REGISTER_MAP_ENTRY_SIZE {
        return Err(InitdataError::BufferSize);
    }
    out.fill(0);
    for window in G18G_26A428_REGISTER_WINDOWS {
        if window.slot >= G18G_26A428_HW_REGISTER_MAP_SLOTS
            || window.slot == G18G_26A428_ZEROED_REGISTER_SLOT
            || !HW_REGISTER_APERTURE.contains(&window.device_va)
        {
            return Err(InitdataError::RegisterWindow);
        }
        let base = window.slot * HW_REGISTER_MAP_ENTRY_SIZE;
        put_u64(out, base, window.phys);
        put_u64(out, base + 0x08, window.device_va);
        put_u32(out, base + 0x10, window.size_10);
        put_u32(out, base + 0x14, window.size_14);
        put_u64(out, base + 0x18, window.unk_18);
        put_u32(out, base + 0x20, window.flag);
    }
    for slot in G18G_26A428_REGISTER_FLAG_ONLY_SLOTS {
        put_u32(out, slot * HW_REGISTER_MAP_ENTRY_SIZE + 0x20, 2);
    }
    Ok(())
}

/// Reconstruct the G18G hw_data object (0x2848 bytes) with caller-owned
/// completion-queue and QoS bindings. The J813 platform words are relocated by
/// [`G18G_26A428_HW_DATA_BANDS`]; the register map, chip id, perf tables,
/// completion table and QoS pointer are G18G encodings. No MMIO access.
pub(crate) fn encode_g18g_26a428_hw_data(
    aliases: [G17PKsmCompletionAliases; M5_25G76_COMPLETION_QUEUE_COUNT],
    qos_high: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != G18G_26A428_HW_DATA_SIZE {
        return Err(InitdataError::BufferSize);
    }
    required_address(qos_high)?;
    let qos_end = qos_high
        .checked_add(G17P_QOS_RESOURCE_SIZE as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    required_address(qos_end - 1)?;
    let mut completion = [0; M5_25G76_COMPLETION_QUEUE_COUNT * G17P_KSM_COMPLETION_DESCRIPTOR_STRIDE];
    // INFERRED descriptor format: the J813 one (26A428 fills it through an
    // accelerator virtual that was not decoded; the location is FW_PROVEN).
    encode_m5_25g76_completion_table(aliases, &mut completion)?;
    for alias in aliases {
        if qos_high < alias.high + M5_25G76_COMPLETION_BACKING_SIZE as u64 && alias.high < qos_end {
            return Err(InitdataError::PairAliasedObject);
        }
    }

    out.fill(0);
    write_relocated_words(M5_25G76_HW_PLATFORM_WORDS, 0, G18G_26A428_HW_DATA_BANDS, out)?;
    encode_g18g_26a428_register_map(&mut out[HW_REGISTER_MAP_OFFSET..G18G_26A428_HW_REGISTER_MAP_END])?;
    put_u32(out, G18G_26A428_HW_CHIP_ID_OFFSET, G18G_26A428_HW_CHIP_ID);
    // INFERRED: this+0xf10 is the number of performance states (J813 12 =
    // its gpu-num-perf-states); J873 ADT gpu-num-perf-states is 13.
    put_u32(out, G18G_26A428_HW_PERF_STATE_COUNT, J873_GPU_NUM_PERF_STATES);
    put_u32(out, G18G_26A428_HW_2714_OFFSET, G18G_26A428_HW_2714_VALUE);
    for (offset, value) in G18G_26A428_HW_CAPTURED_WORDS {
        put_u32(out, offset, value);
    }
    for (offset, value) in G18G_26A428_HW_CAPTURED_HALFWORDS {
        put_u16(out, offset, value);
    }

    // Performance ladder from the J873 ADT (values ADT_PROVEN, locations
    // FW_PROVEN/KEXT_PROVEN). Each voltage fills all 16 lanes of its row as on
    // J813 (INFERRED); b000 reads lanes 0 and 1. Rows 14 and 15 stay zero.
    for (state, &(frequency, core)) in J873_GPU_PERF_STATES.iter().enumerate() {
        put_u32(out, G18G_26A428_HW_PERF_FREQUENCY + state * 4, frequency);
        put_u32(out, G18G_26A428_HW_PERF_SCALE + state * 4, G18G_26A428_PERF_SCALE_VALUE);
        put_u32(out, G18G_26A428_HW_PERF_RELATIVE_A + state * 4, G18G_26A428_PERF_RELATIVE_A[state]);
        put_u32(out, G18G_26A428_HW_PERF_RELATIVE_B + state * 4, G18G_26A428_PERF_RELATIVE_B[state]);
        for lane in 0..G18G_26A428_HW_PERF_LANES {
            let row = state * G18G_26A428_HW_PERF_ROW_STRIDE + lane * 4;
            put_u32(out, G18G_26A428_HW_PERF_CORE_VOLTAGE + row, core);
            put_u32(out, G18G_26A428_HW_PERF_SRAM_VOLTAGE + row, J873_GPU_PERF_STATES_SRAM_MV[state]);
        }
    }
    // INFERRED: the J813 identity index map over all 16 rows.
    for index in 0..G18G_26A428_HW_PERF_ROWS {
        put_u32(out, G18G_26A428_HW_PERF_INDEX_MAP + index * 4, index as u32);
    }
    let table = G18G_26A428_KSM_COMPLETION_TABLE_OFFSET;
    out[table..table + completion.len()].copy_from_slice(&completion);
    put_u64(out, G18G_26A428_QOS_POINTER_OFFSET, qos_high);
    Ok(())
}

// --- Perf/policy block and the blocks around it (plan step 1.6) -------------------

/// KEXT_PROVEN (26A428 initPowerAndPerformanceData 0xfffffe0008c0807c): the
/// host bzeros 0x7800 bytes, memmoves a 0x160-byte host config to +0xb0 and
/// stores mach_absolute_time() at +0x6c14 (25G76: +0x651c).
pub(crate) const G18G_26A428_POLICY_BOOT_TIMESTAMP: usize = 0x6c14;
/// KEXT_PROVEN: FW VA of the 8-byte private-cluster tail (this+0x3a0) at
/// +0x3d30 (25G76 +0x3c98).
pub(crate) const G18G_26A428_POLICY_STATE_PTR: usize = 0x3d30;
/// KEXT_PROVEN: FW VA of the AON PTD (this+0xf78) at +0x64d8 (25G76 +0x6460).
pub(crate) const G18G_26A428_POLICY_AON_PTD_PTR: usize = 0x64d8;
/// Policy-relative place of the J813 512-entry u16 0x7fff table that
/// [`encode_m5_25g76_bundle_policy`] writes at bundle +0x6ed7.
const M5_25G76_POLICY_U16_TABLE: usize = 0x1c97;
const M5_25G76_POLICY_U16_TABLE_ENTRIES: usize = 512;
/// ADT tunables at the places the 26A428 kext stores them. KEXT_PROVEN
/// locations: +0xb38 = accel+0x11080 (gpu-perf-tgt-utilization), +0xb6c/+0xb70
/// = getPerformanceControllerIntegralGain(0/1) = accel+0x110a8/0x110ac
/// (gpu-perf-integral-gain/2), +0xb7c/+0xb80 = ProportionalGain(0/1) =
/// accel+0x110b8/0x110bc; +0x22d4/+0x2928/+0x2e1c = this+0xf10 * 100.
/// Values ADT_PROVEN (J873 sgx).
pub(crate) const G18G_26A428_POLICY_ADT_WORDS: [(usize, u32); 8] = [
    (0x0b38, 75),
    (0x0b6c, 0x3f92_e148),
    (0x0b70, 0x3f92_e148),
    (0x0b7c, 0x4092_e148),
    (0x0b80, 0x4092_e148),
    (0x22d4, J873_GPU_NUM_PERF_STATES * 100),
    (0x2928, J873_GPU_NUM_PERF_STATES * 100),
    (0x2e1c, J873_GPU_NUM_PERF_STATES * 100),
];
/// Region-C ADT tunables. KEXT_PROVEN: +0x7c8 = accel+0x106b0
/// (gpu-idleoff-standby-timer; J813 +0x7c4 carried its ADT 1250) and +0x7e4 =
/// accel+0x106cc (gpu-idleoff-min_conf); ADT_PROVEN values 1500 and 0.8f.
/// +0x9c = this+0xf14 * 100 (25G76 +0x98 = this+0xf40 * 100 = 1200 on J813):
/// INFERRED to be the perf-state count again.
pub(crate) const G18G_26A428_REGION_C_ADT_WORDS: [(usize, u32); 3] = [
    (0x7c8, 1500),
    (0x7e4, 0x3f4c_cccd),
    (0x09c, J873_GPU_NUM_PERF_STATES * 100),
];

/// J873/26A428 pre-handoff policy corrections relative to the relocated
/// J813 baseline. Both roles in four cold captures (051/053/073/075) agree
/// byte-for-byte except the separately encoded boot timestamp. Values are
/// captured calibration/configuration, not inferred field types or MHz.
/// No owned address or timestamp is part of this table. The complete
/// production encoder is compared by a host startup
/// comparison; allocation and execution require live proof.
pub(crate) const G18G_26A428_POLICY_CAPTURED_WORDS: [(usize, u32); 65] = [
    (0x0048, 0x00000514), (0x0088, 0x00000514),
    (0x009c, 1), (0x00a0, 1), (0x00a4, 1), (0x00a8, 1),
    (0x0104, 0x3f828f5c), (0x09f0, 0x470f1600),
    (0x09fc, 0xbd115e00), (0x0a04, 0x00000514),
    (0x0a08, 0x00000514), (0x0a14, 0x00008f16),
    (0x0a48, 0x00000514), (0x0abc, 0x00000514),
    (0x0acc, 0x00008f16), (0x0b4c, 1),
    (0x0b84, 0x414a1af3), (0x0b8c, 0x00000514), (0x0ca0, 0),
    (0x22ac, 0x42c80000), (0x22b0, 0x44a28000),
    (0x22b4, 0x000006c7), (0x22c8, 0x00000064),
    (0x2320, 0xff003f80), (0x234c, 0x3d20045e),
    (0x235c, 0x41a770a4), (0x236c, 0x00000514),
    (0x2370, 0x00000514), (0x23b0, 0x00000514),
    (0x25c8, 0x00000020), (0x2810, 0x05140000),
    (0x2854, 0x05140000), (0x298c, 0x00040000),
    (0x2a88, 0x71480000), (0x2a90, 0x00640000),
    (0x2aa0, 0x00008f16), (0x2aa4, 0x00008f16),
    (0x2aa8, 0x00008f16), (0x2bd0, 0x00000514),
    (0x2bd4, 0x00000514), (0x2bdc, 0x470f1600),
    (0x2be0, 0x00008f16), (0x2c14, 0x00000514),
    (0x2d60, 0x00000514), (0x2f08, 0x0000000c),
    (0x2f0c, 0x00001388), (0x2f10, 0x000007d0),
    (0x2f14, 0x00001388), (0x2fb4, 0x00000514),
    (0x2fb8, 0x00000514), (0x2fc8, 0),
    (0x301c, 0x000004b0), (0x3020, 0x00000514),
    (0x3024, 0x00000514), (0x3030, 0x00000514),
    (0x3064, 0x00000514), (0x30ac, 0),
    (0x30b0, 0x42c80000), (0x30c0, 0xff003f80),
    (0x3d38, 0xff003f80), (0x64e0, 0x67305452),
    (0x6520, 1), (0x6c6c, 0x43366667),
    (0x6cac, 0x44e5c000), (0x6cec, 0x42f00000),
];

/// Stable region-C inputs from the same eight initialization snapshots.
/// +0x38 is the new 26A428 feature bit, no longer a guessed zero.
pub(crate) const G18G_26A428_REGION_C_CAPTURED_WORDS: [(usize, u32); 7] = [
    (0x0038, 1), (0x0088, 0x00008f16), (0x008c, 0x00000514),
    (0x0090, 0x00000514), (0x09b0, 0x00000200),
    (0x09dc, 0x00000032), (0x0e0c, 0x000000fa),
];

/// Initialize the six allocations following G18G hw_data (0x2880..0xd400)
/// from the J813 policy words relocated by [`G18G_26A428_POLICY_BANDS`], plus
/// the owned pointers, the boot timestamp and the J873 ADT tunables. Preserves
/// hw_data and everything from the BRN table on. No firmware is started.
pub(crate) fn encode_g18g_26a428_bundle_policy(
    layouts: &G18GLayouts,
    bindings: M5PolicyBindings,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    let layout = layouts.shared;
    if out.len() < layout.brn_table {
        return Err(InitdataError::BufferSize);
    }
    required_address(bindings.state)?;
    required_address(bindings.aon_ptd)?;
    let state_end = bindings.state.checked_add(STATUS_BLOCK_SIZE as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    let aon_end = bindings.aon_ptd.checked_add(G18G_26A428_AON_PTD_SIZE as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    required_address(state_end - 1)?;
    required_address(aon_end - 1)?;
    if bindings.state < aon_end && bindings.aon_ptd < state_end {
        return Err(InitdataError::PairAliasedObject);
    }
    out[layout.block_c18..layout.brn_table].fill(0);
    let m5 = M5_25G76_SHARED_LAYOUT;
    let m5_policy_end = m5.policy + KEXT_ALLOCATION_SIZES_25G76[KX_POLICY];
    for &(offset, value) in M5_25G76_POLICY_WORDS {
        if (m5.block_1048..m5.block_e10).contains(&offset) {
            // KEXT_PROVEN (R_1048 stores identical in both kexts): the two
            // 0xffffffff words at +0xa18/+0xa30 of the 0x1048 block.
            put_u32(out, layout.block_1048 + (offset - m5.block_1048), value);
        } else if !(m5.policy..m5_policy_end).contains(&offset) {
            return Err(InitdataError::BufferSize);
        }
    }
    let policy = &mut out[layout.policy..layout.policy + layouts.policy_size];
    for &(offset, value) in M5_25G76_POLICY_WORDS {
        if !(m5.policy..m5_policy_end).contains(&offset) {
            continue;
        }
        write_relocated_words(&[(offset, value)], m5.policy, G18G_26A428_POLICY_BANDS, policy)?;
    }
    // The 512 x 0x7fff u16 table (J813 bundle +0x6ed7 = policy +0x1c97)
    // moves with its band (INFERRED: no store is matched inside it).
    for index in 0..M5_25G76_POLICY_U16_TABLE_ENTRIES {
        for (byte_index, byte) in 0x7fffu16.to_le_bytes().into_iter().enumerate() {
            let source = M5_25G76_POLICY_U16_TABLE + index * 2 + byte_index;
            let target = relocate_by_bands(G18G_26A428_POLICY_BANDS, source)
                .ok_or(InitdataError::BufferSize)?;
            *policy.get_mut(target).ok_or(InitdataError::BufferSize)? = byte;
        }
    }
    put_u64(policy, G18G_26A428_POLICY_STATE_PTR, bindings.state);
    put_u64(policy, G18G_26A428_POLICY_AON_PTD_PTR, bindings.aon_ptd);
    put_u64(policy, G18G_26A428_POLICY_BOOT_TIMESTAMP, bindings.boot_timestamp);
    for (offset, value) in G18G_26A428_POLICY_ADT_WORDS {
        put_u32(policy, offset, value);
    }
    for (offset, value) in G18G_26A428_POLICY_CAPTURED_WORDS {
        put_u32(policy, offset, value);
    }
    if let Some(bands) = layouts.policy_bands {
        relayout_by_bands(bands, G18G_26A428_POLICY_SIZE, policy)?;
    }
    Ok(())
}

// --- Region C (plan step 1.5) -------------------------------------------------------

/// Encode the G18G region C: the J813 bytes, the 25G76 fields from +0x38 on
/// moved up by 4 (FW_PROVEN), the J873 ADT tunables and stable native J873
/// inputs. The new +0x38 feature bit (accel+0x6b9 bit 2) is captured as one.
pub(crate) fn encode_g18g_26a428_region_c(out: &mut [u8]) -> Result<(), InitdataError> {
    if out.len() != REGION_C_SIZE {
        return Err(InitdataError::BufferSize);
    }
    encode_region_c_for_profile(RootProfile::J813G17G25G76, out)?;
    // The J813 object never reaches its 0xe78 end; shifting in place keeps the
    // tail zero and needs no second 4 KiB buffer on the kernel stack.
    let shifted = KEXT_ALLOCATION_SIZES_25G76[KX_REGION_C];
    if out[shifted..].iter().any(|byte| *byte != 0) {
        return Err(InitdataError::BufferSize);
    }
    out.copy_within(0x38..shifted, 0x3c);
    out[0x38..0x3c].fill(0);
    for (offset, value) in G18G_26A428_REGION_C_ADT_WORDS {
        put_u32(out, offset, value);
    }
    for (offset, value) in G18G_26A428_REGION_C_CAPTURED_WORDS {
        put_u32(out, offset, value);
    }
    // Idle power-off policy word (Region C +0x30). The firmware loads
    // it on every cold wake, which undoes the IdlePowerOff(0) sent once at boot.
    // 0 keeps the scheduler idle event from powering the cores down between
    // jobs. Measured on the M6: 8.6 percent less time per op.
    put_u32(out, 0x30, 0);
    Ok(())
}

// --- Private cluster: status B, status config, status A (plan step 1.3) ----------

/// Encode the G18G primary status-B span (status B, status config and the gap
/// to status A; [`G18G_26A428_STATUS_B_SPAN`] bytes) before either role is
/// started. `status_b` is the FW VA of the span, for the new cross-links.
pub(crate) fn encode_g18g_26a428_primary_status_b(
    layouts: &G18GLayouts,
    fwctl_state: u64,
    fwctl_ring: u64,
    status_b: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    if out.len() != G18G_26A428_STATUS_B_SPAN {
        return Err(InitdataError::BufferSize);
    }
    required_address(fwctl_state)?;
    required_address(fwctl_ring)?;
    required_address(status_b)?;
    let status_b_end = status_b
        .checked_add(G18G_26A428_STATUS_B_SPAN as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    required_address(status_b_end - 1)?;
    let config = G18G_26A428_STATUS_CONFIG_IN_STATUS_B;
    out.fill(0);
    // +0x04 = 1: INFERRED from J813 (26A428 stores bit 2 of accel+0x6b0 there).
    encode_status_block(&mut out[..STATUS_BLOCK_SIZE])?;
    put_u64(out, G18G_26A428_STATUS_B_FWCTL_STATE, fwctl_state);
    put_u64(out, G18G_26A428_STATUS_B_FWCTL_RING, fwctl_ring);
    put_u32(out, G18G_26A428_STATUS_B_CONFIG_HEADER, 1);
    put_u32(out, G18G_26A428_STATUS_B_LINK_WORD, 0);
    put_u64(out, G18G_26A428_STATUS_B_LINK_PTR,
            status_b + (config + layouts.status_config_link_word) as u64);
    let status_config = &mut out[config..config + layouts.status_config_size];
    write_relocated_words(M5_25G76_STATUS_CONFIG_WORDS, 0, G18G_26A428_STATUS_CONFIG_BANDS,
                          &mut status_config[..G18G_26A428_STATUS_CONFIG_SIZE])?;
    encode_g18g_26a428_status_config_defaults(&mut status_config[..G18G_26A428_STATUS_CONFIG_SIZE]);
    if let Some(bands) = layouts.status_config_bands {
        relayout_by_bands(bands, G18G_26A428_STATUS_CONFIG_SIZE, status_config)?;
    }
    put_u32(out, config + layouts.status_config_link_word, 1);
    put_u64(out, config + layouts.status_config_link_ptr,
            status_b + G18G_26A428_STATUS_B_LINK_WORD as u64);
    Ok(())
}

/// Encode the whole 0x178000-byte G18G private cluster mapped at
/// `private_base`: status-B span, both status A blocks, the secondary root
/// `+0xc0` object and exactly one opening-producer entry per control channel
/// (plan 1.3: role 0 +0x320, role 1 +0x177720 = the J813 +0x1773e0 moved with
/// the tail). Work and report counters stay zero.
pub(crate) fn encode_g18g_26a428_private_cluster(
    layouts: &G18GLayouts,
    private_base: u64,
    fwctl_state: u64,
    out: &mut [u8],
) -> Result<(), InitdataError> {
    let layout = layouts.private;
    if out.len() != layout.total {
        return Err(InitdataError::BufferSize);
    }
    required_address(private_base)?;
    let private_end = private_base
        .checked_add(layout.total as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    required_address(private_end - 1)?;
    required_address(fwctl_state)?;
    let fwctl_ring = fwctl_state
        .checked_add(G18G_26A428_FWCTL_RING_OFFSET as u64)
        .ok_or(InitdataError::AddressOutOfRange)?;
    required_address(fwctl_ring)?;
    if fwctl_state < private_end && private_base < fwctl_ring + CONTROL_RECORD_SIZE as u64 {
        return Err(InitdataError::PairAliasedObject);
    }
    out.fill(0);
    let status_b = layout.status_b;
    let status_a = layout.roles[0].status_a;
    encode_g18g_26a428_primary_status_b(layouts, fwctl_state, fwctl_ring,
                                        private_base + status_b as u64,
                                        &mut out[status_b..status_a])?;
    for role in layout.roles {
        encode_status_block(&mut out[role.status_a..role.status_a + STATUS_BLOCK_SIZE])?;
    }
    let extra = layout.secondary_extra_1;
    encode_secondary_root_extra_1_for_profile(layouts.profile,
        &mut out[extra..extra + STATUS_BLOCK_SIZE])?;
    put_u32(out, layout.control_state + CHANNEL_STATE_PRODUCER * CHANNEL_STATE_SPACING,
            CONTROL_OPENING_PRIMARY_PRODUCER);
    put_u32(out, layout.secondary_control_state + CHANNEL_STATE_PRODUCER * CHANNEL_STATE_SPACING,
            CONTROL_OPENING_SECONDARY_PRODUCER);
    Ok(())
}

/// AON PTD (0xe4, KEXT_PROVEN size). INFERRED: the J813 header words.
pub(crate) fn encode_g18g_26a428_aon_ptd(out: &mut [u8]) -> Result<(), InitdataError> {
    encode_m5_25g76_aon_ptd(out)
}

// --- Device-control and endpoint vocabulary (plan step 1.7) -----------------------

/// FW_PROVEN (jump table 0xfffffc0000006c60, 56 entries from opcode 4): the
/// opening/region-C reload handler (b000 0x3ed0) is reached by 0x15, 0x2a
/// and 0x30.
pub(crate) const G18G_26A428_CONTROL_OPENING_OPCODE_PRIMARY: u32 = 0x15;
pub(crate) const G18G_26A428_CONTROL_OPENING_OPCODE_SECONDARY: u32 = 0x2a;
/// FW_PROVEN: IdlePowerOff keeps its opcode and behaviour (b000 0x4ef0).
pub(crate) const G18G_26A428_DEVICE_CONTROL_OPCODE_IDLE_POWER_OFF: u32 = 0x0a;
pub(crate) const G18G_26A428_DEVICE_CONTROL_OPCODE_MIN: u32 = 4;
pub(crate) const G18G_26A428_DEVICE_CONTROL_OPCODE_MAX: u32 = 0x3b;
/// FW_PROVEN: jump-table entries routed to the default (invalid) handler.
pub(crate) const G18G_26A428_DEVICE_CONTROL_INVALID_OPCODES: [u32; 9] =
    [0x19, 0x1c, 0x20, 0x23, 0x26, 0x28, 0x31, 0x39, 0x3a];

pub(crate) const fn g18g_26a428_device_control_opcode_valid(opcode: u32) -> bool {
    if opcode < G18G_26A428_DEVICE_CONTROL_OPCODE_MIN || opcode > G18G_26A428_DEVICE_CONTROL_OPCODE_MAX {
        return false;
    }
    let mut index = 0;
    while index < G18G_26A428_DEVICE_CONTROL_INVALID_OPCODES.len() {
        if G18G_26A428_DEVICE_CONTROL_INVALID_OPCODES[index] == opcode {
            return false;
        }
        index += 1;
    }
    true
}

// --- Unproven values ------------------------------------------------------------------

/// One G18G value that is not proven for 26A428.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct UnprovenField {
    /// "hw_data", "register_map" (offset = slot), "policy", "region_c",
    /// "status_b", "status_config", "main_config", "aon_ptd",
    /// "secondary_extra_1".
    pub(crate) object: &'static str,
    pub(crate) offset: usize,
    pub(crate) len: usize,
    pub(crate) evidence: Evidence,
    pub(crate) note: &'static str,
}

const fn unproven(object: &'static str, offset: usize, len: usize, evidence: Evidence,
                  note: &'static str) -> UnprovenField {
    UnprovenField { object, offset, len, evidence, note }
}

/// Every INFERRED or GUESSED G18G value, for review and for the hardware
/// bring-up log. A crashlog assertion should be matched against this list
/// first.
pub(crate) const G18G_26A428_UNPROVEN: &[UnprovenField] = &[
    unproven("hw_data", 0x0000, 0x640, Evidence::Inferred,
             "host prefix (+0x28 FW VA, two 32x0x18 accelerator tables) carried from J813"),
    unproven("hw_data", 0x0fa0, 8, Evidence::Inferred, "u64 FW VA of accel+0x1d40; J813 0"),
    unproven("hw_data", 0x0fa8, 8, Evidence::Inferred, "captured 0x8000; complete semantics unqualified"),
    unproven("hw_data", 0x0fb4, 4, Evidence::Inferred, "captured J873 value 1; meaning unknown"),
    unproven("hw_data", 0x0fc0, 0x110, Evidence::Inferred,
             "J813 relocated words with captured J873 scalar corrections; complete semantics unqualified"),
    unproven("hw_data", 0x10b4, 4, Evidence::Guessed, "new u32 (accel byte & 1); left 0"),
    unproven("hw_data", 0x10d4, 0xc, Evidence::Inferred,
             "captured J873 0x1b/4/12; complete field semantics unqualified"),
    unproven("hw_data", 0x10e0, 4, Evidence::Inferred, "13 = gpu-num-perf-states (J813 12 matched by value)"),
    unproven("hw_data", 0x1124, 0x800, Evidence::Inferred,
             "ADT voltages filled into all 16 lanes of each row; b000 reads lanes 0 and 1"),
    unproven("hw_data", 0x1964, 0x38, Evidence::Inferred, "1.02f scale word per state (J813 value)"),
    unproven("hw_data", 0x19e4, 0x38, Evidence::Inferred, "native float32 ladder A; calibration formula unqualified"),
    unproven("hw_data", 0x1a24, 0x38, Evidence::Inferred, "native float32 ladder B; calibration formula unqualified"),
    unproven("hw_data", 0x1ae4, 0x40, Evidence::Inferred, "identity index map 0..15 (J813)"),
    unproven("hw_data", 0x2660, 0xd4, Evidence::Inferred,
             "J813 relocated words with captured J873 scalar corrections; complete semantics unqualified"),
    unproven("hw_data", 0x2734, 0x80, Evidence::Inferred,
             "KSM completion descriptor format (capacity 0x800, entry 0x40) carried from J813"),
    unproven("hw_data", 0x27d4, 8, Evidence::Guessed, "new u64 (accel+0x12338); left 0"),
    unproven("hw_data", 0x27dc, 0x3c, Evidence::Inferred, "J813 +0x26b4..+0x26ec words relocated"),
    unproven("hw_data", 0x2818, 4, Evidence::Guessed, "new u32 (accel+0x6c8); read as part of a u64 by b000"),
    unproven("hw_data", 0x281e, 0xc, Evidence::Inferred, "captured u32=1/u64=0; complete semantics unqualified"),
    unproven("hw_data", 0x2832, 4, Evidence::Guessed, "new mode word, compared with 3 at ~10 b000 sites; left 0"),
    unproven("hw_data", 0x283a, 8, Evidence::Inferred, "new enables (accel+0x1165c/+0x1166c bit 0); captured as 1/1"),
    unproven("register_map", 0, 1, Evidence::Inferred, "AIC SW-int page from ADT meta-sw-interrupt"),
    unproven("register_map", 12, 1, Evidence::Inferred, "ANE0 doorbell left unmapped: 26A428 G variant no longer sets it"),
    unproven("register_map", 29, 2, Evidence::Inferred, "DPEP/ADCLK +0x18 left 0; flags 0 mapped read-only like J813 slots 10/14"),
    unproven("register_map", 49, 1, Evidence::Inferred, "FenderRegsHWL flags 0 (base sets none) mapped read-only"),
    unproven("register_map", 0, 60, Evidence::Inferred, "device VAs assigned by the J813 allocation rule"),
    unproven("policy", 0x0000, 0x7800, Evidence::Inferred,
             "matches eight native J873 snapshots with owned pointers/timestamp normalized; calibration semantics unqualified"),
    unproven("policy", 0x00b0, 0x160, Evidence::Inferred, "captured host config matches; individual field semantics unqualified"),
    unproven("policy", 0x1ca7, 0x400, Evidence::Inferred, "512 x 0x7fff u16 table moved with its +0x10 band"),
    unproven("policy", 0x0b5c, 0x10, Evidence::Inferred,
             "filter-time-constant terms: J813 defaults (J873 ADT has none)"),
    unproven("policy", 0x0b9c, 0xc, Evidence::Inferred,
             "tgt - dz and dz with dz = 1 carried from J813 (J873 ADT has no gpu-perf-dz)"),
    unproven("policy", 0x64e0, 0x9c, Evidence::Inferred,
             "native RT0g at +0x64e0 and one at +0x6520; other words zero, complete semantics unqualified"),
    unproven("region_c", 0x0000, 0xe80, Evidence::Inferred,
             "matches eight native J873 snapshots; individual field semantics unqualified"),
    unproven("region_c", 0x0038, 4, Evidence::Inferred, "new feature bit captured as one; runtime effect unqualified"),
    unproven("region_c", 0x008c, 8, Evidence::Inferred, "native 1300/1300; field units unqualified"),
    unproven("region_c", 0x009c, 4, Evidence::Inferred, "1300 = this+0xf14 x 100 assumed equal to the state count"),
    unproven("status_b", 0x0004, 4, Evidence::Inferred, "1 carried (26A428 stores accel+0x6b0 bit 2)"),
    unproven("status_config", 0x0000, 0x640, Evidence::Inferred,
             "complete 0x6d0 allocation matches eight native J873 pre-boot views; individual calibration semantics unqualified"),
    unproven("status_config", 0x01dc, 8, Evidence::Inferred,
             "native J873 1/29000 and original host stores verified; semantic names unqualified"),
    unproven("status_config", 0x03e8, 0x104, Evidence::Inferred,
             "native generated 16x16 u8 table and two u16 enables; original host copy geometry verified"),
    unproven("main_config", 0x0300, 4, Evidence::Inferred, "secondary kind 0xc carried (accel+0x10a74)"),
    unproven("main_config", 0x02f8, 8, Evidence::Guessed, "third descriptor table (accel[0x235e]) absent: 0"),
    unproven("aon_ptd", 0x0000, 0xc, Evidence::Inferred, "J813 PTD header words"),
    unproven("secondary_extra_1", 0x0000, 0x60, Evidence::Inferred, "T8140/J813 content (same store geometry)"),
];

#[cfg(test)]
#[path = "g18_initdata_test.rs"]
mod g18_tests;

#[cfg(test)]
mod tests {
    #[test]
    fn m5_register_extent_uses_size_10_and_rejects_invalid_pages() {
        let native = super::M5_25G76_REGISTER_WINDOWS.iter().find(|w| w.slot == 39).unwrap();
        assert_eq!(native.mapped_pages(), Ok((0x580e08000, 0xfffffc2180128000, 0x10000)));
        let mut invalid = *native;
        invalid.phys += 1;
        assert_eq!(invalid.mapped_pages(), Err(super::InitdataError::RegisterWindow));
        invalid = *native;
        invalid.size_10 = 0;
        assert_eq!(invalid.mapped_pages(), Err(super::InitdataError::RegisterWindow));
        invalid = *native;
        invalid.device_va = u64::MAX & !0x3fff;
        assert_eq!(invalid.mapped_pages(), Err(super::InitdataError::AddressOutOfRange));
        invalid = *native;
        invalid.phys = u64::MAX & !0x3fff;
        assert_eq!(invalid.mapped_pages(), Err(super::InitdataError::AddressOutOfRange));
    }

    #[test]
    fn m5_private_arena_rejects_invalid_bindings_before_writing() {
        let mut arena = vec![0xa5; super::M5_25G76_PRIVATE_CLUSTER_SIZE];
        for pointer in [0, u64::MAX - 8, 1 << 48] {
            assert!(super::encode_m5_25g76_private_cluster(pointer, &mut arena).is_err());
            assert!(arena.iter().all(|&byte| byte == 0xa5));
        }
    }

    use super::*;

    const KERN: u64 = 0xffff_fc20_0000_0000;

    fn encoded_fast_die() -> FastDieConfig {
        let mut bundle = vec![0u8; HW_DATA_BUNDLE_MIN_ALLOC];
        encode_bundle_static(KERN + 0x1000, &mut bundle).unwrap();
        let mut status_b = vec![0u8; PRIMARY_STATUS_B_OBJECT_SIZE];
        encode_primary_status_b(KERN + 0x2000, KERN + 0x2040, &mut status_b).unwrap();
        fast_die_config(&bundle, &status_b).unwrap()
    }

    #[test]
    fn fast_die_config_matches_native() {
        let c = encoded_fast_die();
        assert_eq!(FASTDIE_OBJECT_BUNDLE_OFFSET, 0x5240);
        assert_eq!(c.mask, 0x82a); // G17P mask for accel[0x4a0] != 0x1f
        assert_eq!(c.target_cc, FASTDIE_NATIVE_TARGET_CC); // 110.00 C
        assert_eq!(c.release_cc, 8000);
        assert_eq!(c.delta_cc, 0);
        assert_eq!(c.kp_bits, 0x41a2_b852);
        assert_eq!(c.ki_dt_bits, 0x4116_0419);
        assert_eq!(c.state, 0x4200_0006); // fw1 0x174fc constant
        assert_eq!(c.live_target_cc, c.target_cc);
        assert_eq!(c.live_release_cc, c.release_cc);
        assert_eq!(c.live_kp_bits, c.kp_bits);
        assert_eq!(c.live_ki_dt_bits, c.ki_dt_bits);
        assert_eq!(c.live_update, 0);
        // Same object g17_dvfs_policy::GFX1_STATS_OFFSET (primary state + 0xe780) reads.
        assert_eq!(PRIMARY_STATUS_B_STATE_GRID_OFFSET + PRIMARY_STATUS_B_CONFIG_OFFSET, 0xe780);
    }

    #[test]
    fn fast_die_target_override_is_lower_only() {
        assert_eq!(fast_die_target_override(0).unwrap(), None);
        assert_eq!(fast_die_target_override(10_000).unwrap(), Some(10_000));
        assert_eq!(fast_die_target_override(8_500).unwrap(), Some(8_500));
        assert!(fast_die_target_override(11_001).is_err());
        assert!(fast_die_target_override(8_499).is_err());
        let mut bundle = vec![0u8; HW_DATA_BUNDLE_MIN_ALLOC];
        encode_bundle_static(KERN + 0x1000, &mut bundle).unwrap();
        let mut status_b = vec![0u8; PRIMARY_STATUS_B_OBJECT_SIZE];
        encode_primary_status_b(KERN + 0x2000, KERN + 0x2040, &mut status_b).unwrap();
        apply_fast_die_target_bundle(&mut bundle, 10_000).unwrap();
        apply_fast_die_target_status_b(&mut status_b, 10_000).unwrap();
        let c = fast_die_config(&bundle, &status_b).unwrap();
        assert_eq!((c.target_cc, c.live_target_cc, c.release_cc, c.mask), (10_000, 10_000, 8000, 0x82a));
    }


    fn native_compute_bytes(bytes: &[u8]) -> Vec<u8> {
        let mut native = bytes.to_vec();
        for offset in 0..bytes.len().saturating_sub(7) {
            let value = get_u64(bytes, offset);
            let address = value & !CONTROL_OPERAND_ENTRY_FLAG;
            if (0x70_0200_0000..0x70_03ef_8000).contains(&address) {
                put_u64(&mut native, offset, value - COMPUTE_FLIST_RELOCATION);
            }
        }
        native
    }
    #[test]
    fn compute_aperture_ranges_do_not_overlap_each_other_or_render() {
        let mut ranges = vec![
            (COMPUTE_FLIST_PAGE_LIST_ADDRESS, 0x20_0000),
            (COMPUTE_FLIST_RUN_TABLE_ADDRESS, 0x1_0000),
            (COMPUTE_RUNTIME_PAGE_LIST_ADDRESS, 0xc000),
            (COMPUTE_RUNTIME_CLASS1_TABLE_ADDRESS, 0x4000),
        ];
        for i in 0..CONTROL_OPERAND_ENTRY_COUNT {
            ranges.push((COMPUTE_FLIST_BUFFER_BASE + i as u64 * CONTROL_OPERAND_BUFFER_STRIDE,
                         CONTROL_OPERAND_BUFFER_SIZE));
        }
        for base in G17P_KSM_COMPLETION_LOW_VAS {
            ranges.push((base, G17P_KSM_COMPLETION_BACKING_SIZE as u64));
        }
        // Native mature kernel-low ZERO0/ZERO1 intentionally alias that
        // mode's main backing, whereas ordinary client mappings retain its
        // separate page-list. They are not additional overlapping mappings.
        assert_eq!(COMPUTE_RUNTIME_ZERO_BUFFER_0_ADDRESS, COMPUTE_FLIST_PAGE_LIST_ADDRESS);
        assert_eq!(COMPUTE_RUNTIME_ZERO_BUFFER_1_ADDRESS, COMPUTE_FLIST_PAGE_LIST_ADDRESS + 0x10_8000);
        for (i, &(base, size)) in ranges.iter().enumerate() {
            assert_eq!(base & 0x3fff, 0);
            assert!(base >= 0x70_0200_0000 && base + size <= 0x70_0400_0000);
            for &(other, other_size) in ranges.iter().skip(i + 1) {
                assert!(base + size <= other || other + other_size <= base,
                        "overlap: {base:#x}+{size:#x} with {other:#x}+{other_size:#x}");
            }
        }
        assert_eq!(CONTROL_OPERAND_TABLE_ADDRESS, 0x70_0020_8000);
        assert_eq!(CONTROL_OPERAND_BUFFER_BASE, 0x70_0022_0000);
    }

    #[test]
    fn compute_support_uses_full_unaligned_page_pointer() {
        for encode in [encode_compute_readiness_class1_support,
                       encode_compute_readiness_class3_support,
                       encode_compute_runtime_class2_support,
                       encode_compute_partial_opening_control_shared] {
            let mut bytes = vec![0xa5; 0x4000];
            encode(&mut bytes).unwrap();
            assert_eq!(get_u64(&bytes, 0x14), 0x70_0200_0000);
            assert_eq!(get_u32(&bytes, 0x1c), 0x4_0000);
            assert_eq!(get_u64(&bytes, 0x30), 0x70_0220_8000);
        }
    }

    fn shared() -> SharedAddresses {
        SharedAddresses {
            region_a: KERN + 0x1b_8000,
            region_c: KERN + 0x1a_0000,
            hw_data_bundle: KERN + 0xc078_8000,
            hw_data_bundle_alloc: NATIVE_SHARED_CLUSTER_SIZE,
        }
    }

    fn primary() -> InstanceAddresses {
        InstanceAddresses {
            root: KERN + 0x1a_8000,
            main_config: KERN + 0xc078_8000 + NATIVE_PRIMARY_MAIN_BUNDLE_OFFSET as u64,
            status_a: KERN + 0x2_ee40,
            status_b: KERN + 0x2_0000 + PRIMARY_STATUS_B_STATE_GRID_OFFSET as u64,
            secondary_extras: [0, 0],
        }
    }

    fn secondary() -> InstanceAddresses {
        InstanceAddresses {
            root: KERN + 0x1a_8000 + SECONDARY_ROOT_DELTA,
            main_config: KERN + 0xc078_8000 + NATIVE_SECONDARY_MAIN_BUNDLE_OFFSET as u64,
            status_a: KERN + 0xe_3100,
            status_b: 0,
            secondary_extras: [KERN + 0x2_e780, KERN + 0x19_7400],
        }
    }

    fn state_grid(role: InstanceRole) -> u64 {
        match role {
            InstanceRole::Primary => KERN + 0x2_0000,
            InstanceRole::Secondary => KERN + 0x19_70c0,
        }
    }

    fn channels(role: InstanceRole) -> [ChannelEntry; CHANNEL_TABLE_ENTRIES] {
        let inst = match role {
            InstanceRole::Primary => primary(),
            InstanceRole::Secondary => secondary(),
        };
        derive_channel_table(
            role,
            state_grid(role),
            inst.status_a,
            inst.main_config,
            shared().hw_data_bundle,
        )
    }

    fn region_views() -> PrimaryRegionViews {
        PrimaryRegionViews {
            sentinel_high: KERN + 0xc07c_8000,
            alias_high: [KERN + 0x15e_0000, KERN + 0x15e_8000],
        }
    }

    fn encode_root_bytes(role: InstanceRole) -> Vec<u8> {
        let inst = match role {
            InstanceRole::Primary => primary(),
            InstanceRole::Secondary => secondary(),
        };
        let mut out = vec![0xa5; root_size(role)];
        encode_root(role, &shared(), &inst, &mut out).unwrap();
        out
    }

    fn encode_main_bytes(role: InstanceRole) -> Vec<u8> {
        let inst = match role {
            InstanceRole::Primary => primary(),
            InstanceRole::Secondary => secondary(),
        };
        let views = region_views();
        let views = match role {
            InstanceRole::Primary => Some(&views),
            InstanceRole::Secondary => None,
        };
        let mut out = vec![0xa5; MAIN_CONFIG_SIZE];
        encode_main_config(role, &shared(), &inst, &channels(role), views, &mut out).unwrap();
        out
    }

    #[test]
    fn descriptor_table_aliases_are_published_in_both_main_configs() {
        let views = region_views();
        let primary = encode_main_bytes(InstanceRole::Primary);
        assert_eq!(get_u64(&primary, 0x2d8), REGION_VIEW_LOW_ADDRS[0]);
        assert_eq!(get_u64(&primary, 0x2e0), views.alias_high[0]);
        assert_eq!(get_u64(&primary, 0x2e8), REGION_VIEW_LOW_ADDRS[1]);
        assert_eq!(get_u64(&primary, 0x2f0), views.alias_high[1]);

        let secondary = encode_main_bytes(InstanceRole::Secondary);
        assert_eq!(get_u64(&secondary, 0x2d8), REGION_VIEW_LOW_ADDRS[0]);
        assert_eq!(get_u64(&secondary, 0x2e0), 0);
        assert_eq!(get_u64(&secondary, 0x2e8), REGION_VIEW_LOW_ADDRS[1]);
        assert_eq!(get_u64(&secondary, 0x2f0), 0);
    }

    #[test]
    fn per_role_root_sizes_version_tuple_and_handoff_zeroes() {
        assert_eq!(ROOT_SIZE_PRIMARY, 0xb8);
        assert_eq!(ROOT_SIZE_SECONDARY, 0xc8);
        assert_eq!(root_size(InstanceRole::Primary), 0xb8);
        assert_eq!(root_size(InstanceRole::Secondary), 0xc8);
        // The exact version tuple, both as words and as the u64 the firmware
        // compares. The M5 value 0x0c8d_e339_0728_04c0 must never reappear.
        assert_eq!(INITDATA_VERSION, [0x04c0, 0x0396, 0xa322, 0x0c8a]);
        assert_eq!(INITDATA_VERSION_WORD, 0x0c8a_a322_0396_04c0);

        let root = encode_root_bytes(InstanceRole::Primary);
        assert_eq!(root.len(), 0xb8);
        assert_eq!(get_u64(&root, 0x00), INITDATA_VERSION_WORD);
        assert_eq!(get_u64(&root, 0x08), shared().region_a);
        // +0x10 is zero at handoff even though the buffer started dirty.
        assert_eq!(get_u64(&root, 0x10), 0);
        assert_eq!(get_u64(&root, 0x18), primary().main_config);
        assert_eq!(get_u64(&root, 0x20), shared().region_c);
        assert_eq!(get_u32(&root, 0x28), 0);
        assert_eq!(get_u32(&root, 0x2c), 1);
        assert_eq!(get_u64(&root, 0xa8), primary().status_a);
        assert_eq!(get_u64(&root, 0xb0), primary().status_b);
        assert!(check_firmware_root_gates(&root).is_ok());

        let root = encode_root_bytes(InstanceRole::Secondary);
        assert_eq!(root.len(), 0xc8);
        assert_eq!(get_u32(&root, 0x28), 1);
        assert_eq!(get_u64(&root, 0xa8), secondary().status_a);
        // Status B is real only on the primary.
        assert_eq!(get_u64(&root, 0xb0), 0);
        assert_eq!(get_u64(&root, 0xb8), secondary().secondary_extras[0]);
        assert_eq!(get_u64(&root, 0xc0), secondary().secondary_extras[1]);
        assert!(check_firmware_root_gates(&root).is_ok());
    }

    #[test]
    fn uat_geometry_block_and_level_descriptors() {
        let root = encode_root_bytes(InstanceRole::Primary);
        // Page size 0x4000 (u16), page bits 14, level count 3.
        assert_eq!(
            u16::from_le_bytes(root[0x30..0x32].try_into().unwrap()),
            0x4000
        );
        assert_eq!(root[0x32], 14);
        assert_eq!(root[0x33], 3);
        // Three 0x20-byte descriptors: shifts 36/25/14, entries 64/2048/2048.
        let expected = [(36u8, 64u16), (25, 2048), (14, 2048)];
        for (index, (shift, entries)) in expected.iter().enumerate() {
            let base = 0x34 + index * 0x20;
            assert_eq!(root[base], 8);
            assert_eq!(root[base + 1], 14);
            assert_eq!(root[base + 2], 14);
            assert_eq!(root[base + 3], *shift);
            assert_eq!(
                u16::from_le_bytes(root[base + 4..base + 6].try_into().unwrap()),
                *entries
            );
            assert_eq!(
                u16::from_le_bytes(root[base + 6..base + 8].try_into().unwrap()),
                0x4000
            );
            assert_eq!(get_u64(&root, base + 0x08), 1);
            assert_eq!(get_u64(&root, base + 0x10), 0x0000_03ff_ffff_c000);
            assert_eq!(get_u64(&root, base + 0x18), (*entries as u64 - 1) << *shift);
        }
        // Spot-check the derived index masks against their absolute values.
        assert_eq!(get_u64(&root, 0x34 + 0x18), 0x3f_0000_0000_0 >> 4 << 4); // 63 << 36
        assert_eq!(get_u64(&root, 0x34 + 0x18), 63u64 << 36);
        assert_eq!(get_u64(&root, 0x54 + 0x18), 2047u64 << 25);
        assert_eq!(get_u64(&root, 0x74 + 0x18), 2047u64 << 14);
        // The descriptor block ends at 0x94; the gap to status A stays zero.
        assert!(root[0x94..0xa8].iter().all(|byte| *byte == 0));
    }

    #[test]
    fn main_config_bundle_views_are_views_into_one_allocation() {
        assert_eq!(MAIN_CONFIG_SIZE, 0x600);
        let main = encode_main_bytes(InstanceRole::Primary);
        assert_eq!(main.len(), 0x600);
        let bundle = shared().hw_data_bundle;
        assert_eq!(get_u64(&main, 0x00), bundle);
        // The repeated writable region is bundle + 0xc500, at +0x08 and
        // +0x10 — past the 0xc000 principal bundle, which is why the
        // allocation must reach 0x10000.
        assert_eq!(get_u64(&main, 0x08), bundle + 0xc500);
        assert_eq!(get_u64(&main, 0x10), bundle + 0xc500);
        assert!(HW_DATA_BUNDLE_MIN_ALLOC >= 0xc500);
        // Five unaligned qwords at +0x254: views into the ONE bundle
        // allocation at the fixed offsets, not independent objects.
        let offsets = [0x2740u64, 0x3380, 0x4400, 0xbc80, 0xbd00];
        let extents = [0x18c0usize, 0x0c80, 0x3c00, 0x3380, 0x3300];
        for (index, (offset, extent)) in offsets.iter().zip(extents).enumerate() {
            let view = get_u64(&main, 0x254 + index * 8);
            assert_eq!(view, bundle + offset);
            // Each view lies wholly inside the single allocation.
            assert!((*offset as usize) + extent <= shared().hw_data_bundle_alloc);
        }
        assert_eq!(BUNDLE_VIEW_OFFSETS.map(|o| o as u64), offsets);
        assert_eq!(BUNDLE_VIEW_EXTENTS, extents);
        // Views 3 and 4 overlap by all but 0x80 bytes, and view 0 overlaps
        // the tail of the hardware-data object — the relations independent
        // allocations cannot reproduce.
        assert_eq!(0xbc80 + 0x3380, 0xbd00 + 0x3300);
        assert!(BUNDLE_VIEW_OFFSETS[0] < HW_DATA_SIZE);
    }

    #[test]
    fn bundle_allocation_below_minimum_fails_closed() {
        let mut small = shared();
        small.hw_data_bundle_alloc = 0xc000;
        let mut out = vec![0; ROOT_SIZE_PRIMARY];
        assert_eq!(
            encode_root(InstanceRole::Primary, &small, &primary(), &mut out),
            Err(InitdataError::BundleAllocationTooSmall)
        );
    }

    #[test]
    fn channel_table_has_seventeen_entries_with_the_control_grid_at_12() {
        assert_eq!(CHANNEL_TABLE_ENTRIES, 17);
        assert_eq!(channel_table_entry_range(0), Some(0x20..0x40));
        assert_eq!(channel_table_entry_range(11), Some(0x180..0x1a0));
        // Entry 12 exists. The M5-derived model refused it, misreading its
        // state-counter addresses at +0x1a0/+0x1a8 as an unrelated cursor.
        assert_eq!(channel_table_entry_range(12), Some(0x1a0..0x1c0));
        assert_eq!(channel_table_entry_range(16), Some(0x220..0x240));
        assert_eq!(channel_table_entry_range(17), None);

        let main = encode_main_bytes(InstanceRole::Primary);
        let grid = state_grid(InstanceRole::Primary);
        let bundle = shared().hw_data_bundle;
        // Work entry 0 (TA_0): three separate state-counter addresses at 0x10
        // spacing; consumer = state 0, producer = state 2. Its ring is a work
        // ring in the bundle.
        assert_eq!(get_u64(&main, 0x20), grid + 0x0c0); // consumer
        assert_eq!(get_u64(&main, 0x28), grid + 0x0d0);
        assert_eq!(get_u64(&main, 0x30), grid + 0x0e0); // producer
        assert_eq!(
            get_u64(&main, 0x38),
            bundle + WORK_RING_BUNDLE_OFFSETS[0] as u64
        );

        // Entry 12 (device control): three counters in the compact grid, and
        // the ring embedded at main + 0x4c0, where the opening opcode is staged.
        let e12 = channel_table_entry_range(12).unwrap().start;
        assert_eq!(
            get_u64(&main, e12 + CHANNEL_STATE_CONSUMER * 8),
            grid + 0x300
        );
        assert_eq!(
            get_u64(&main, e12 + CHANNEL_STATE_PRODUCER * 8),
            grid + 0x320
        );
        assert_eq!(get_u64(&main, e12 + 0x18), primary().main_config + 0x4c0);
        assert_eq!(get_u32(&main, 0x4c0), CONTROL_OPENING_OPCODE_PRIMARY);

        // Entry 15 is partial: a first state address, no producer, no ring.
        let e15 = channel_table_entry_range(15).unwrap().start;
        assert_ne!(get_u64(&main, e15), 0);
        assert_eq!(get_u64(&main, e15 + 0x08), 0);
        assert_eq!(get_u64(&main, e15 + 0x10), 0);
        assert_eq!(get_u64(&main, e15 + 0x18), 0);
        // Entry 16 is entirely empty.
        let e16 = channel_table_entry_range(16).unwrap().start;
        assert!(main[e16..e16 + CHANNEL_ENTRY_SIZE]
            .iter()
            .all(|byte| *byte == 0));
        // The primary carries the +0x3e0 mask; +0x300 (secondary kind) stays 0.
        assert_eq!(get_u32(&main, 0x3e0), MAIN_PRIMARY_MASK_VALUE);
        assert_eq!(get_u32(&main, 0x32c), 1);
        assert_eq!(get_u32(&main, 0x344), 0xabcd_abcd);
        assert_eq!(get_u32(&main, 0x300), 0);
    }

    #[test]
    fn secondary_leaves_work_channels_empty_but_keeps_control() {
        let main = encode_main_bytes(InstanceRole::Secondary);
        // Entries 0..11 are empty on the control-only peer.
        for index in 0..WORK_CHANNEL_COUNT {
            let range = channel_table_entry_range(index).unwrap();
            assert!(main[range].iter().all(|byte| *byte == 0));
        }
        // Entry 12 control ring is still main + 0x4c0, with the secondary opcode.
        let e12 = channel_table_entry_range(12).unwrap().start;
        assert_eq!(get_u64(&main, e12 + 0x18), secondary().main_config + 0x4c0);
        assert_eq!(get_u32(&main, 0x4c0), CONTROL_OPENING_OPCODE_SECONDARY);
        // Secondary-only fields: +0x300 = 4, and the unaligned +0x471 pointer
        // into primary view 2. The primary's +0x3e0 mask is absent here.
        assert_eq!(get_u32(&main, 0x300), MAIN_SECONDARY_KIND_VALUE);
        let extra = shared().hw_data_bundle
            + BUNDLE_VIEW_OFFSETS[SECONDARY_EXTRA_VIEW_INDEX] as u64
            + SECONDARY_EXTRA_VIEW_OFFSET as u64;
        assert_eq!(get_u64(&main, 0x471), extra);
        assert_eq!(get_u32(&main, 0x3e0), 0);
    }

    #[test]
    fn primary_control_opening_is_the_single_native_record() {
        let mut opening = vec![0; control_opening_size(InstanceRole::Primary)];
        encode_control_opening(InstanceRole::Primary, &mut opening).unwrap();

        assert_eq!(control_opening_producer(InstanceRole::Primary), 1);
        for slot in 0..1 {
            let base = slot * CONTROL_RECORD_SIZE;
            assert_eq!(get_u32(&opening, base), CONTROL_OPENING_OPCODE_PRIMARY);
            assert!(opening[base + 4..base + CONTROL_RECORD_SIZE]
                .iter()
                .all(|byte| *byte == 0));
        }
        assert_eq!(opening.len(), CONTROL_RECORD_SIZE);
    }

    #[test]
    fn live_compute_control_tail_matches_the_output_positive_sequence() {
        fn fnv1a64(bytes: &[u8]) -> u64 {
            bytes.iter().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
                (hash ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3)
            })
        }

        let mut tail = vec![0u8; COMPUTE_RUNTIME_CONTROL_RECORDS * CONTROL_RECORD_SIZE];
        for index in 0..COMPUTE_RUNTIME_CONTROL_RECORDS {
            let start = index * CONTROL_RECORD_SIZE;
            encode_compute_runtime_control_record(
                index,
                &mut tail[start..start + CONTROL_RECORD_SIZE],
            )
            .unwrap();
        }
        assert_eq!(tail.len(), 65 * 0x40);
        assert_eq!(fnv1a64(&native_compute_bytes(&tail)), 0x544f_a4b7_bc23_e65a);
        assert_eq!(COMPUTE_RUNTIME_CONTROL_FINAL_COUNTER, 66);

        for (index, sequence) in [(0, 0), (53, 53), (55, 54), (56, 55), (58, 56), (64, 62)] {
            let base = index * CONTROL_RECORD_SIZE;
            assert_eq!(get_u32(&tail, base), 0x2e);
            assert_eq!(get_u32(&tail, base + 0x04), sequence);
            assert_eq!(get_u32(&tail, base + 0x0c), 0);
        }
        for (index, class, sequence, operand, slot_offset, count) in [
            (
                COMPUTE_RUNTIME_CLASS1_RECORD_INDEX,
                1,
                54,
                COMPUTE_RUNTIME_CLASS1_TABLE_ADDRESS,
                0x480,
                0x18,
            ),
            (
                COMPUTE_RUNTIME_CLASS2_RECORD_INDEX,
                2,
                56,
                COMPUTE_FLIST_RUN_TABLE_ADDRESS,
                0x5c0,
                0x28,
            ),
        ] {
            let base = index * CONTROL_RECORD_SIZE;
            assert_eq!(get_u32(&tail, base), 0x20);
            assert_eq!(get_u32(&tail, base + 0x04), class);
            assert_eq!(get_u32(&tail, base + 0x08), 0x3f);
            assert_eq!(get_u32(&tail, base + 0x0c), sequence);
            assert_eq!(get_u64(&tail, base + 0x14), COMPUTE_RUNTIME_SUPPORT_ADDRESS);
            assert_eq!(get_u64(&tail, base + 0x1c), operand);
            assert_eq!(get_u64(&tail, base + 0x24), operand + slot_offset);
            assert_eq!(get_u32(&tail, base + 0x2c), count);
            assert_eq!(get_u32(&tail, base + 0x30), 0);
            assert_eq!(get_u32(&tail, base + 0x34), 1);
        }
    }

    #[test]
    fn scheduler_state_release_record_is_exact() {
        let mut record = [0xa5; CONTROL_RECORD_SIZE];
        encode_scheduler_state_release_record(
            0xffff_fc20_0123_4000,
            [0x11, 0x22, 0x33, 0x44],
            &mut record,
        )
        .unwrap();
        assert_eq!(get_u32(&record, 0), 0x14);
        assert_eq!(get_u32(&record, 4), 0);
        assert_eq!(record[0x08], 0x44);
        assert_eq!(record[0x09], 0x11);
        assert_eq!(record[0x0a], 0x22);
        assert_eq!(record[0x0b], 0x33);
        assert_eq!(get_u64(&record, 0x0c), 0xffff_fc20_0123_4000);
        assert!(record[0x14..].iter().all(|byte| *byte == 0));
        assert!(device_control_asserts_gpu_power(0x14) == Some(true));
    }

    #[test]
    fn bootstrap_uma_release_record_is_exact() {
        let mut record = [0xa5; CONTROL_RECORD_SIZE];
        encode_bootstrap_uma_release_record(0, 0, &mut record).unwrap();
        assert_eq!(get_u32(&record, 0), CONTROL_BOOTSTRAP_UMA_RELEASE_OPCODE);
        assert!(record[4..].iter().all(|byte| *byte == 0));

        encode_bootstrap_uma_release_record(0x1122_3344_5566_7788, 0xaabb_ccdd, &mut record)
            .unwrap();
        assert_eq!(get_u64(&record, 0x04), 0x1122_3344_5566_7788);
        assert_eq!(get_u32(&record, 0x0c), 0xaabb_ccdd);
        assert!(record[0x10..].iter().all(|byte| *byte == 0));
    }

    #[test]
    fn first_cl_readiness_suffix_has_exact_sequences_contexts_and_bytes() {
        assert_eq!(
            COMPUTE_READINESS_PUBLICATION_ORDER,
            [
                ComputeReadinessPublicationStep::ControlRecord(0),
                ComputeReadinessPublicationStep::ControlRecord(1),
                ComputeReadinessPublicationStep::ControlRecord(2),
                ComputeReadinessPublicationStep::InitializeContext2Activation,
                ComputeReadinessPublicationStep::ControlRecord(3),
                ComputeReadinessPublicationStep::ControlRecord(4),
                ComputeReadinessPublicationStep::ControlRecord(5),
            ]
        );
        let mut records = [[0u8; CONTROL_RECORD_SIZE]; COMPUTE_READINESS_RECORDS];
        for (index, record) in records.iter_mut().enumerate() {
            encode_compute_readiness_record(index, record).unwrap();
        }

        for (index, boundary) in COMPUTE_READINESS_BOUNDARIES.iter().enumerate() {
            assert_eq!(boundary.index as usize, index);
            assert_eq!(
                boundary.producer_before,
                CONTROL_OPENING_PRIMARY_PRODUCER + index as u32
            );
            assert_eq!(
                boundary.producer_after,
                CONTROL_OPENING_PRIMARY_PRODUCER + 1 + index as u32
            );
            assert_eq!(get_u32(&records[index], 0x00), boundary.opcode);
            let (sequence_offset, context_offset) = if boundary.opcode == 0x20 {
                (0x0c, 0x30)
            } else {
                (0x04, 0x0c)
            };
            assert_eq!(get_u32(&records[index], sequence_offset), boundary.sequence);
            assert_eq!(get_u32(&records[index], context_offset), boundary.context);
            assert_eq!(compute_readiness_boundary(index), Some(*boundary));
        }
        assert_eq!(compute_readiness_boundary(COMPUTE_READINESS_RECORDS), None);

        let mut progress = ComputeReadinessProgress::new();
        assert_eq!(progress.retire(5), None);
        for boundary in COMPUTE_READINESS_BOUNDARIES {
            assert_eq!(progress.next_boundary(), Some(boundary));
            let pending = progress.begin_publication(boundary).unwrap();
            assert_eq!(pending.pending_target(), Some(boundary.producer_after));
            assert_eq!(pending.next_boundary(), None);
            assert_eq!(pending.begin_publication(boundary), None);
            assert_eq!(pending.retire(boundary.producer_before), None);
            progress = pending.retire(boundary.producer_after).unwrap();
            assert_eq!(progress.retired_records(), boundary.index as usize + 1);
        }
        assert!(progress.complete());
        assert_eq!(progress.next_boundary(), None);

        for (index, class, sequence, support, slot, count, context) in [
            (
                0,
                1,
                35,
                COMPUTE_READINESS_CLASS1_SUPPORT_ADDRESS,
                0x440,
                0x20,
                1,
            ),
            (
                3,
                3,
                37,
                COMPUTE_READINESS_CLASS3_SUPPORT_ADDRESS,
                0x5c0,
                0x28,
                2,
            ),
        ] {
            let record = &records[index];
            assert_eq!(get_u32(record, 0x00), 0x20);
            assert_eq!(get_u32(record, 0x04), class);
            assert_eq!(get_u32(record, 0x08), 0x3f);
            assert_eq!(get_u32(record, 0x0c), sequence);
            assert_eq!(get_u64(record, 0x14), support);
            assert_eq!(get_u64(record, 0x1c), COMPUTE_FLIST_RUN_TABLE_ADDRESS);
            assert_eq!(get_u64(record, 0x24), COMPUTE_FLIST_RUN_TABLE_ADDRESS + slot);
            assert_eq!(get_u32(record, 0x2c), count);
            assert_eq!(get_u32(record, 0x30), context);
            assert_eq!(get_u32(record, 0x34), 1);
        }

        for (index, sequence, context) in [(1, 35, 1), (2, 36, 1), (4, 37, 2), (5, 38, 2)] {
            let record = &records[index];
            assert_eq!(get_u32(record, 0x00), 0x2e);
            assert_eq!(get_u32(record, 0x04), sequence);
            assert_eq!(get_u32(record, 0x0c), context);
            assert!(record[0x10..].iter().all(|byte| *byte == 0));
        }
        assert_eq!(
            COMPUTE_READINESS_FINAL_COUNTER,
            CONTROL_OPENING_PRIMARY_PRODUCER + COMPUTE_READINESS_RECORDS as u32
        );
        assert!(encode_compute_readiness_record(COMPUTE_READINESS_RECORDS, &mut records[0])
            .is_err());
    }

    #[test]
    fn first_cl_readiness_support_state_and_activation_pages_are_exact() {
        fn expected_support(
            class: u32,
            active: u32,
            resource_class: u64,
            cursor: u32,
            state: u64,
            final_kind: u32,
        ) -> [u8; COMPUTE_READINESS_PAGE_SIZE] {
            let mut expected = [0u8; COMPUTE_READINESS_PAGE_SIZE];
            put_u32(&mut expected, 0x00, class);
            put_u32(&mut expected, 0x08, active);
            put_u32(&mut expected, 0x10, class);
            put_u64(&mut expected, 0x14, COMPUTE_FLIST_PAGE_LIST_ADDRESS);
            put_u32(&mut expected, 0x1c, 0x4_0000);
            put_u64(&mut expected, 0x20, resource_class << 40);
            put_u64(&mut expected, 0x28, resource_class << 40);
            put_u64(&mut expected, 0x30, COMPUTE_FLIST_RUN_TABLE_ADDRESS);
            put_u32(&mut expected, 0x40, 4);
            put_u32(&mut expected, 0x48, cursor);
            put_u64(&mut expected, 0x4c, state);
            put_u32(&mut expected, 0x60, final_kind);
            expected
        }

        let mut class1 = [0u8; COMPUTE_READINESS_PAGE_SIZE];
        let mut class3 = [0u8; COMPUTE_READINESS_PAGE_SIZE];
        let mut state = [0xffu8; COMPUTE_READINESS_PAGE_SIZE];
        encode_compute_readiness_class1_support(&mut class1).unwrap();
        encode_compute_readiness_class3_support(&mut class3).unwrap();
        encode_compute_readiness_state(&mut state).unwrap();
        assert_eq!(
            class1,
            expected_support(
                1,
                1,
                0x11,
                0x88,
                COMPUTE_READINESS_CLASS1_STATE_ADDRESS,
                2,
            )
        );
        assert_eq!(
            class3,
            expected_support(
                3,
                2,
                0x17,
                0xb8,
                COMPUTE_READINESS_CLASS3_STATE_ADDRESS,
                3,
            )
        );
        assert_eq!(get_u32(&state, 0), 1);
        assert!(state[4..].iter().all(|byte| *byte == 0));

        let mut activation = [0xffu8; CONTROL_RECORD_SIZE];
        encode_compute_readiness_activation_record(&mut activation).unwrap();
        let mut expected = [0u8; CONTROL_RECORD_SIZE];
        put_u64(&mut expected, 0x00, 0x0000_0100_0000_ffff);
        put_u64(&mut expected, 0x20, 0x0002_0000_0000_0000);
        put_u64(&mut expected, 0x30, 0x0000_0000_ff00_0000);
        assert_eq!(activation, expected);
        assert_eq!(COMPUTE_READINESS_ACTIVATION_RECORD_OFFSET, 0x80);

        let mut operand_table = [0xffu8; CONTROL_OPERAND_TABLE_SIZE];
        encode_compute_readiness_operand_table(&mut operand_table).unwrap();
        for index in 0..COMPUTE_READINESS_OPERAND_ENTRY_COUNT {
            assert_eq!(
                get_u64(&operand_table, index * CONTROL_OPERAND_ENTRY_STRIDE),
                (COMPUTE_FLIST_BUFFER_BASE
                    + index as u64 * CONTROL_OPERAND_BUFFER_STRIDE)
                    | CONTROL_OPERAND_ENTRY_FLAG,
            );
        }
        assert!(operand_table
            [COMPUTE_READINESS_OPERAND_ENTRY_COUNT * CONTROL_OPERAND_ENTRY_STRIDE..]
            .iter()
            .all(|byte| *byte == 0));
    }

    #[test]
    fn live_compute_control_closure_matches_source_builders() {
        fn fnv1a64(bytes: &[u8]) -> u64 {
            bytes.iter().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
                (hash ^ u64::from(*byte)).wrapping_mul(0x0000_0100_0000_01b3)
            })
        }

        let mut class1 = vec![0; COMPUTE_RUNTIME_SUPPORT_SIZE];
        encode_compute_runtime_class1_support(&mut class1).unwrap();
        assert_eq!(fnv1a64(&native_compute_bytes(&class1)), 0xb40b_a63a_16c1_937d);
        assert_eq!(get_u32(&class1, 0x00), 1);
        assert_eq!(get_u64(&class1, 0x14), COMPUTE_RUNTIME_PAGE_LIST_ADDRESS);
        assert_eq!(get_u64(&class1, 0x30), COMPUTE_RUNTIME_CLASS1_TABLE_ADDRESS);
        assert_eq!(get_u64(&class1, 0x4c), COMPUTE_RUNTIME_STATE_ADDRESS);

        let mut class2 = vec![0; COMPUTE_RUNTIME_SUPPORT_SIZE];
        encode_compute_runtime_class2_support(&mut class2).unwrap();
        assert_eq!(fnv1a64(&native_compute_bytes(&class2)), 0xdd12_cbe2_2d7c_3d50);
        assert_eq!(get_u32(&class2, 0x00), 2);
        assert_eq!(get_u64(&class2, 0x30), COMPUTE_FLIST_RUN_TABLE_ADDRESS);

        let mut table = vec![0; COMPUTE_RUNTIME_CLASS1_TABLE_SIZE];
        encode_compute_runtime_class1_table(&mut table).unwrap();
        assert_eq!(fnv1a64(&native_compute_bytes(&table)), 0xc820_66ed_9e50_6c41);
        for (index, address) in COMPUTE_RUNTIME_BUFFER_ADDRESSES.into_iter().enumerate() {
            assert_eq!(get_u64(&table, index * 0x40), address | (1 << 60));
        }

        let mut inventory = vec![0; COMPUTE_RUNTIME_PAGE_LIST_SIZE];
        encode_compute_runtime_page_inventory(&mut inventory).unwrap();
        assert_eq!(fnv1a64(&native_compute_bytes(&inventory)), 0x9f78_e915_163a_cea5);
        assert_eq!(get_u64(&inventory, 0), COMPUTE_RUNTIME_BUFFER_ADDRESSES[0]);
        assert_eq!(
            get_u64(&inventory, inventory.len() - 8),
            COMPUTE_RUNTIME_BUFFER_ADDRESSES[20] + 0xff000
        );

        let mut dispatch = [0u8; COMPUTE_DISPATCH_RECORD_SIZE];
        encode_compute_dispatch_record(&mut dispatch).unwrap();
        let packed = get_u64(&dispatch, 0);
        assert_eq!((packed & ((1 << 41) - 1)) << 7, COMPUTE_FLIST_PAGE_LIST_ADDRESS);
        // Compare the capture only after undoing the packed VA relocation.
        put_u64(&mut dispatch, 0, packed - (COMPUTE_FLIST_RELOCATION >> 7));
        assert_eq!(fnv1a64(&dispatch), 0x12a0_cf17_a88d_a788);
    }

    #[test]
    fn secondary_control_opening_is_the_single_native_record() {
        let mut opening = vec![0; control_opening_size(InstanceRole::Secondary)];
        encode_control_opening(InstanceRole::Secondary, &mut opening).unwrap();

        assert_eq!(control_opening_producer(InstanceRole::Secondary), 1);
        for slot in 0..CONTROL_OPENING_SECONDARY_RECORDS {
            let base = slot * CONTROL_RECORD_SIZE;
            let expected = if slot < core::cmp::min(3, CONTROL_OPENING_SECONDARY_RECORDS) {
                CONTROL_OPENING_OPCODE_SECONDARY
            } else {
                CONTROL_OPENING_SECONDARY_CONTINUE_OPCODE
            };
            assert_eq!(get_u32(&opening, base), expected);
            assert!(opening[base + 4..base + CONTROL_RECORD_SIZE]
                .iter()
                .all(|byte| *byte == 0));
        }
    }

    #[test]
    fn generic_control_support_keeps_descriptor_and_runtime_identity() {
        let mut shared = vec![0; CONTROL_SHARED_OBJECT_SIZE];
        encode_control_shared(&mut shared).unwrap();

        assert_eq!(get_u64(&shared, 0x00), 1);
        assert_eq!(get_u64(&shared, 0x10), 1);
        assert_eq!(get_u64(&shared, 0x18), 0x0004_0000_0000_0070);
        assert_eq!(get_u64(&shared, 0x20), 0x0000_1100_0000_0000);
        assert_eq!(get_u64(&shared, 0x28), 0x0000_1100_0000_0000);
        assert_eq!(get_u64(&shared, 0x30), CONTROL_OPERAND_TABLE_ADDRESS);
        assert_eq!(
            get_u32(&shared, CONTROL_SHARED_CURSOR_OFFSET),
            CONTROL_SHARED_CURSOR_BEFORE
        );
        assert_eq!(get_u64(&shared, 0x4c), CONTROL_SHARED_INNER_ADDRESS);
        assert_eq!(get_u64(&shared, 0x60), 2);
        assert_eq!(CONTROL_SHARED_INNER_PRESENTED, 2);
    }

    #[test]
    fn compute_partial_support_preserves_hardware_proven_compact_state() {
        let mut shared = vec![0; CONTROL_SHARED_OBJECT_SIZE];
        encode_compute_partial_opening_control_shared(&mut shared).unwrap();

        assert_eq!(get_u64(&shared, 0x00), 1);
        assert_eq!(get_u32(&shared, 0x08), u32::MAX);
        assert_eq!(get_u32(&shared, 0x10), 2);
        assert_eq!(get_u64(&shared, 0x20), 0x0000_1900_0000_0000);
        assert_eq!(get_u64(&shared, 0x28), 0x0000_1900_0000_0000);
        assert_eq!(get_u64(&shared, 0x30), COMPUTE_FLIST_RUN_TABLE_ADDRESS);
        assert_eq!(
            get_u32(&shared, CONTROL_SHARED_CURSOR_OFFSET),
            COMPUTE_PARTIAL_OPENING_CONTROL_SHARED_CURSOR_BEFORE
        );
        assert_eq!(COMPUTE_PARTIAL_OPENING_CONTROL_SHARED_CURSOR_BEFORE, 0x18);
        assert_eq!(
            get_u64(&shared, 0x4c),
            PARTIAL_OPENING_CONTROL_SHARED_INNER_ADDRESS
        );
        assert_eq!(get_u64(&shared, 0x60), 3);
    }

    #[test]
    fn first_partial_support_has_distinct_address_cursor_and_inner_page() {
        let mut shared = vec![0; CONTROL_SHARED_OBJECT_SIZE];
        encode_partial_opening_control_shared(&mut shared).unwrap();

        assert_eq!(PARTIAL_OPENING_CONTROL_SHARED_ADDRESS, 0xffff_fc20_c082_8000);
        assert_eq!(get_u64(&shared, 0x00), 1);
        assert_eq!(get_u32(&shared, 0x08), 0);
        assert_eq!(get_u32(&shared, 0x0c), 0);
        assert_eq!(get_u64(&shared, 0x10), 2);
        assert_eq!(get_u64(&shared, 0x20), 0x0000_1100_0000_0000);
        assert_eq!(get_u64(&shared, 0x28), 0x0000_1100_0000_0000);
        assert_eq!(get_u64(&shared, 0x30), CONTROL_OPERAND_TABLE_ADDRESS);
        assert_eq!(
            get_u32(&shared, CONTROL_SHARED_CURSOR_OFFSET),
            PARTIAL_OPENING_CONTROL_SHARED_CURSOR_BEFORE
        );
        assert_eq!(
            get_u64(&shared, 0x4c),
            PARTIAL_OPENING_CONTROL_SHARED_INNER_ADDRESS
        );
        assert_eq!(get_u64(&shared, 0x60), 3);
        assert_eq!(PARTIAL_OPENING_CONTROL_SHARED_INNER_PRESENTED, 0);

        encode_render_control_shared(&mut shared).unwrap();
        assert_eq!(get_u64(&shared, 0x00), CONTROL_SHARED_INNER_ADDRESS);
        assert_eq!(get_u32(&shared, 0x08), 0);
        assert_eq!(get_u64(&shared, 0x10), 2);
        assert_eq!(get_u64(&shared, 0x14), 0x70_0000_0000);
        assert_eq!(get_u64(&shared, 0x30), CONTROL_OPERAND_TABLE_ADDRESS);
        assert_eq!(get_u64(&shared, 0x4c), CONTROL_SHARED_INNER_ADDRESS);
        assert_eq!(get_u64(&shared, 0x60), 3);
    }

    #[test]
    fn presized_render_control_shared_matches_apple_initial_3d_pool() {
        let mut shared = [0u8; CONTROL_SHARED_OBJECT_SIZE];
        encode_presized_render_control_shared(&mut shared, 22).unwrap();
        // 25G83 populateInitialFirmwareState for the 3D pool: 0x1600 pages
        // at +0x24 and +0x2c, run cursor 0xb0.
        assert_eq!(get_u64(&shared, 0x20), 0x0000_1600_0000_0000);
        assert_eq!(get_u64(&shared, 0x28), 0x0000_1600_0000_0000);
        assert_eq!(get_u32(&shared, CONTROL_SHARED_CURSOR_OFFSET), 0xb0);
        assert_eq!(get_u64(&shared, 0x00), CONTROL_SHARED_INNER_ADDRESS);
        assert_eq!(get_u64(&shared, 0x4c), CONTROL_SHARED_INNER_ADDRESS);
        assert_eq!(get_u64(&shared, 0x60), 3);
    }

    #[test]
    fn first_partial_operand_table_is_published_between_control_and_work() {
        let mut table = vec![0xff; CONTROL_OPERAND_TABLE_SIZE];
        encode_partial_opening_operand_table_pre_control(&mut table).unwrap();
        assert!(table.iter().all(|byte| *byte == 0));

        encode_partial_opening_operand_table_post_control(&mut table).unwrap();
        for index in 0..PARTIAL_OPENING_CONTROL_OPERAND_ENTRY_COUNT {
            assert_eq!(
                get_u64(&table, index * CONTROL_OPERAND_ENTRY_STRIDE),
                CONTROL_OPERAND_ENTRY_FLAG
                    | (CONTROL_OPERAND_BUFFER_BASE
                        + index as u64 * CONTROL_OPERAND_BUFFER_STRIDE)
            );
        }
        assert!(table[PARTIAL_OPENING_CONTROL_OPERAND_ENTRY_COUNT
            * CONTROL_OPERAND_ENTRY_STRIDE..]
            .iter()
            .all(|byte| *byte == 0));

        encode_control_operand_table(&mut table).unwrap();

        for index in 0..CONTROL_OPERAND_ENTRY_COUNT {
            assert_eq!(
                get_u64(&table, index * CONTROL_OPERAND_ENTRY_STRIDE),
                CONTROL_OPERAND_ENTRY_FLAG
                    | (COMPUTE_FLIST_BUFFER_BASE
                        + index as u64 * CONTROL_OPERAND_BUFFER_STRIDE)
            );
        }
        assert!(table[CONTROL_OPERAND_ENTRY_COUNT * CONTROL_OPERAND_ENTRY_STRIDE..]
            .iter()
            .all(|byte| *byte == 0));
    }

    #[test]
    fn a_null_producer_on_a_populated_channel_fails_closed() {
        // A populated work channel with a null producer (state 2) makes the
        // firmware fault on address zero; encode must refuse it.
        let mut table = channels(InstanceRole::Primary);
        table[0].states[CHANNEL_STATE_PRODUCER] = 0;
        let views = region_views();
        let mut out = vec![0; MAIN_CONFIG_SIZE];
        assert_eq!(
            encode_main_config(
                InstanceRole::Primary,
                &shared(),
                &primary(),
                &table,
                Some(&views),
                &mut out,
            ),
            Err(InitdataError::ChannelTableShape)
        );
    }

    #[test]
    fn role_specific_root_fields_fail_closed() {
        // A primary must not carry the secondary-only extras (its root is only
        // 0xb8 bytes and cannot even hold them).
        let mut inst = primary();
        inst.secondary_extras = [KERN + 0x1000, 0];
        let mut out = vec![0; ROOT_SIZE_PRIMARY];
        assert_eq!(
            encode_root(InstanceRole::Primary, &shared(), &inst, &mut out),
            Err(InitdataError::RoleSpecificAddress)
        );
        // A secondary must not carry a status-B pointer.
        let mut inst = secondary();
        inst.status_b = KERN + 0x1000;
        let mut out = vec![0; ROOT_SIZE_SECONDARY];
        assert_eq!(
            encode_root(InstanceRole::Secondary, &shared(), &inst, &mut out),
            Err(InitdataError::RoleSpecificAddress)
        );
    }

    #[test]
    fn wrong_root_buffer_size_fails_closed() {
        // The primary root is 0xb8, not 0xc8; a secondary-sized buffer is wrong.
        let mut out = vec![0; ROOT_SIZE_SECONDARY];
        assert_eq!(
            encode_root(InstanceRole::Primary, &shared(), &primary(), &mut out),
            Err(InitdataError::BufferSize)
        );
    }

    fn register_windows() -> Vec<RegisterWindow> {
        vec![
            RegisterWindow {
                slot: 0,
                phys: 0x2_0000_0000,
                device_va: HW_REGISTER_APERTURE.start,
                size: 0x4000,
                unk_18: 0,
                flag: 2,
            },
            RegisterWindow {
                slot: 5,
                phys: 0x2_1000_0000,
                device_va: HW_REGISTER_APERTURE.start + 0x1_0000,
                size: 0x8000,
                unk_18: 0,
                flag: 2,
            },
        ]
    }

    fn completion_aliases() -> [G17PKsmCompletionAliases; 2] {
        [
            G17PKsmCompletionAliases {
                low: G17P_KSM_COMPLETION_LOW_VAS[0],
                high: shared().hw_data_bundle
                    + G17P_KSM_COMPLETION_HIGH_BUNDLE_OFFSETS[0] as u64,
            },
            G17PKsmCompletionAliases {
                low: G17P_KSM_COMPLETION_LOW_VAS[1],
                high: shared().hw_data_bundle
                    + G17P_KSM_COMPLETION_HIGH_BUNDLE_OFFSETS[1] as u64,
            },
        ]
    }

    fn sksm_qid_resource_high() -> u64 {
        shared().hw_data_bundle + NATIVE_SHARED_CLUSTER_SIZE as u64 + 0x4000
    }

    fn qos_resource_high() -> u64 {
        shared().hw_data_bundle + NATIVE_SHARED_CLUSTER_SIZE as u64 + 0x8000
    }

    #[test]
    fn hw_data_live_pointers_survive_opaque_constants() {
        for qos_resource in [
            0xffff_fc20_001f_8000,
            0xffff_fc20_013a_c000,
            0xffff_fc20_cabc_4000,
        ] {
            let aliases = [
                G17PKsmCompletionAliases {
                    low: 0x70_0240_0000,
                    high: 0xffff_fc20_c240_0000,
                },
                G17PKsmCompletionAliases {
                    low: 0x70_0280_0000,
                    high: 0xffff_fc20_c280_0000,
                },
            ];
            let windows: Vec<_> = (0..HW_REGISTER_MAP_SLOTS)
                .map(|slot| RegisterWindow {
                    slot,
                    phys: 0x4_0000_0000 + (slot as u64) * 0x4000,
                    device_va: HW_REGISTER_APERTURE.start + (slot as u64) * 0x4000,
                    size: 0x4000,
                    unk_18: 0,
                    flag: 2,
                })
                .collect();
            let mut hw = vec![0xa5; HW_DATA_SIZE];
            encode_hw_data(&windows, &[], aliases, qos_resource,
                sksm_qid_resource_high(), &mut hw).unwrap();
            assert_eq!(get_u64(&hw, G17P_QOS_RESOURCE_LIVE_OFFSET), qos_resource);
            assert_eq!(get_u32(&hw, 0x26b4), 1);
            assert_eq!(get_u32(&hw, 0x26b8), 1);
            for (aliases, ordinal) in aliases.iter().zip(G17P_KSM_COMPLETION_PRESENT_ORDINALS) {
                let base = G17P_KSM_COMPLETION_TABLE_OFFSET
                    + ordinal as usize * G17P_KSM_COMPLETION_DESCRIPTOR_STRIDE;
                assert_eq!(get_u64(&hw, base), aliases.low);
                assert_eq!(get_u64(&hw, base + 8), aliases.high);
            }
            for window in windows {
                let base = HW_REGISTER_MAP_OFFSET + window.slot * HW_REGISTER_MAP_ENTRY_SIZE;
                assert_eq!(get_u64(&hw, base), window.phys);
                assert_eq!(get_u64(&hw, base + 8), window.device_va);
            }
        }
    }

    #[test]
    fn hw_data_opaque_runs_do_not_overlap_live_fields() {
        let live = [
            HW_REGISTER_MAP_OFFSET
                ..HW_REGISTER_MAP_OFFSET + HW_REGISTER_MAP_SLOTS * HW_REGISTER_MAP_ENTRY_SIZE,
            G17P_KSM_COMPLETION_TABLE_OFFSET
                ..G17P_KSM_COMPLETION_TABLE_OFFSET + 4 * G17P_KSM_COMPLETION_DESCRIPTOR_STRIDE,
            G17P_QOS_RESOURCE_LIVE_OFFSET..G17P_QOS_RESOURCE_LIVE_OFFSET + 8,
        ];
        for (offset, data) in HW_DATA_OPAQUE_RUNS {
            for range in &live {
                assert!(*offset + data.len() <= range.start || *offset >= range.end,
                    "opaque range {offset:#x}..{:#x} overlaps live {range:#x?}",
                    offset + data.len());
            }
        }
    }

    #[test]
    fn hw_data_size_acceptance_completion_table_and_firmware_state() {
        assert_eq!(HW_DATA_SIZE, 0x3db4);
        let windows = register_windows();
        let flag_only = [(3usize, 2u32), (7, 2)];
        let aliases = completion_aliases();
        let mut hw = vec![0xa5; HW_DATA_SIZE];
        let qos_resource = qos_resource_high();
        let sksm_qid_resource = sksm_qid_resource_high();
        encode_hw_data(
            &windows,
            &flag_only,
            aliases,
            qos_resource,
            sksm_qid_resource,
            &mut hw,
        )
        .unwrap();
        assert_eq!(hw.len(), 0x3db4);

        // Chip id at +0xe90.
        assert_eq!(get_u32(&hw, 0xe90), 0x8140);

        // Register-mapping array: 0x28 stride at +0x640.
        let base0 = HW_REGISTER_MAP_OFFSET;
        assert_eq!(get_u64(&hw, base0), 0x2_0000_0000);
        assert_eq!(get_u64(&hw, base0 + 0x08), HW_REGISTER_APERTURE.start);
        assert_eq!(get_u32(&hw, base0 + 0x10), 0x4000);
        assert_eq!(get_u32(&hw, base0 + 0x14), 0x4000);
        assert_eq!(get_u32(&hw, base0 + 0x20), 2);
        // A declared-but-unmapped slot carries only its flag.
        let base3 = HW_REGISTER_MAP_OFFSET + 3 * HW_REGISTER_MAP_ENTRY_SIZE;
        assert_eq!(get_u64(&hw, base3), 0);
        assert_eq!(get_u32(&hw, base3 + 0x20), 2);

        // The +0x258e acceptance block — the one block firmware requires.
        assert_eq!(
            u16::from_le_bytes(hw[0x258e..0x2590].try_into().unwrap()),
            12
        );
        for group in [0x2590usize, 0x25a0] {
            for (index, value) in [0u32, 1, 1, 1].iter().enumerate() {
                assert_eq!(get_u32(&hw, group + index * 4), *value);
            }
        }
        for channel in 0..12 {
            assert_eq!(get_u32(&hw, 0x25b4 + channel * 4), 0xffff_ffff);
        }
        for offset in [0x25f4usize, 0x2600, 0x2608] {
            assert_eq!(get_u32(&hw, offset), 1);
        }
        // +0x2630 is firmware state and must start zero even though the buffer
        // began dirty (0xa5).
        assert_eq!(hw[0x2630], 0);

        // Four inline descriptors begin at +0x2614; only ordinals 0/2 exist.
        assert_eq!(get_u32(&hw, G17P_KSM_COMPLETION_TABLE_PREFIX_OFFSET), 0x100);
        for ordinal in [1usize, 3] {
            let base = G17P_KSM_COMPLETION_TABLE_OFFSET
                + ordinal * G17P_KSM_COMPLETION_DESCRIPTOR_STRIDE;
            assert_eq!(&hw[base..base + 0x20], &[0; 0x20]);
        }
        for (aliases, ordinal) in aliases.iter().zip(G17P_KSM_COMPLETION_PRESENT_ORDINALS) {
            let base = G17P_KSM_COMPLETION_TABLE_OFFSET
                + ordinal as usize * G17P_KSM_COMPLETION_DESCRIPTOR_STRIDE;
            assert_eq!(get_u64(&hw, base), aliases.low);
            assert_eq!(get_u64(&hw, base + 0x08), aliases.high);
            assert_eq!(get_u32(&hw, base + 0x10), 0x800);
            assert_eq!(get_u32(&hw, base + 0x14), 0x40);
            assert_eq!(get_u32(&hw, base + 0x18), ordinal);
            assert_eq!(get_u32(&hw, base + 0x1c), 0);
        }
        assert_eq!(get_u64(&hw, G17P_SKSM_QID_CONFIG_2700_OFFSET), 0);
        assert_eq!(get_u64(&hw, G17P_QOS_RESOURCE_LIVE_OFFSET), qos_resource);
        assert_eq!(get_u32(&hw, G17P_SKSM_QID_CONFIG_2708_OFFSET), 0);
        assert_eq!(get_u64(&hw, G17P_QOS_RESOURCE_OFFSET), 0);
        assert_eq!(get_u64(&hw, G17P_SKSM_QID_RESOURCE_OFFSET), 0);

        // The recorded opaque runs landed (spot check the two 0xffffffff tails).
        assert_eq!(get_u32(&hw, 0x3d98), 0xffff_ffff);
        assert_eq!(get_u32(&hw, 0x3db0), 0xffff_ffff);

        // Both measured ladders and their voltage columns are carried.
        assert!(PERF_TABLE_VALUES_ENCODED);
        assert_eq!(get_u32(&hw, HW_PERF_GROUP_BASES[0]), 0);
        assert_eq!(get_u32(&hw, HW_PERF_GROUP_BASES[0] + 10 * 4), 1470);
        assert_eq!(get_u32(&hw, HW_PERF_GROUP_BASES[1] + 10 * 4), 1326);
        assert_eq!(get_u32(&hw, HW_PERF_GROUP_BASES[0] + 0x40), 125);
        assert_eq!(get_u32(&hw, HW_PERF_GROUP_BASES[1] + 0x40), 125);
    }

    #[test]
    fn hw_data_rejects_out_of_aperture_and_duplicate_slots() {
        let aliases = completion_aliases();
        let mut hw = vec![0; HW_DATA_SIZE];

        // A device VA outside the fixed register aperture fails closed.
        let mut window = register_windows();
        window[0].device_va = 0x1000;
        assert_eq!(
            encode_hw_data(
                &window,
                &[],
                aliases,
                qos_resource_high(),
                sksm_qid_resource_high(),
                &mut hw,
            ),
            Err(InitdataError::RegisterWindow)
        );

        // Two declarations for the same slot fail closed.
        assert_eq!(
            encode_hw_data(
                &register_windows(),
                &[(0usize, 2u32)],
                aliases,
                qos_resource_high(),
                sksm_qid_resource_high(),
                &mut hw,
            ),
            Err(InitdataError::RegisterWindow)
        );
    }

    #[test]
    fn region_c_size_and_required_word() {
        assert_eq!(REGION_C_SIZE, 0x1000);
        let mut region_c = vec![0xa5; REGION_C_SIZE];
        encode_region_c(&mut region_c).unwrap();
        assert_eq!(region_c.len(), 0x1000);
        assert_eq!(REGION_C_REQUIRED_VALUE, 0);
        assert_eq!(get_u32(&region_c, 0xe50), REGION_C_REQUIRED_VALUE);
        assert_eq!(&region_c[0xe50..0xe55], &[0; 5]);
        // Those same four bytes, framed the way b1 actually reads them: the
        // byte at +0xe50 is zero and the GPU-core power-assert counter is the
        // UNALIGNED u32 at +0xe51. Nothing in the image reads +0xe50 or +0xe55.
        assert_eq!(region_c[REGION_C_REQUIRED_OFFSET], 0);
        assert_eq!(
            get_u32(&region_c, REGION_C_GPU_CORE_PWR_ASSERT_COUNTER_OFFSET),
            REGION_C_GPU_CORE_PWR_ASSERT_COUNT
        );
        assert_eq!(REGION_C_GPU_CORE_PWR_ASSERT_COUNTER_OFFSET, 0xe51);
        assert_eq!(get_u32(&region_c, 0x9b8), 2);
        assert_eq!(get_u32(&region_c, 0x9bc), 40);
        assert_eq!(region_c[0xe21], 0);
        // A recorded run near the front landed; the tail past the last run is 0.
        assert_eq!(get_u32(&region_c, 0x24), 0x0000_0bb8);
        assert_eq!(get_u32(&region_c, 0x7c), 1);
        assert_eq!(get_u32(&region_c, 0x80), 1);
        assert_eq!(get_u32(&region_c, 0x9c), 0x3000_001d);
        assert_eq!(get_u32(&region_c, 0xa0), 0x3100_0002);
        assert_eq!(region_c[REGION_C_SIZE - 1], 0);
    }

    #[test]
    fn device_control_power_assert_flags_match_b1() {
        const CLEAR: [u32; 15] = [
            0x00, 0x0e, 0x15, 0x16, 0x1b, 0x22, 0x2a, 0x2e, 0x3a, 0x3b, 0x3c, 0x3d,
            0x3e, 0x3f, 0x40,
        ];
        for opcode in 0..=0x3f {
            assert_eq!(
                device_control_asserts_gpu_power(opcode),
                Some(!CLEAR[..CLEAR.len() - 1].contains(&opcode)),
                "opcode {opcode:#x}",
            );
        }
        assert_eq!(device_control_asserts_gpu_power(CLEAR[CLEAR.len() - 1]), None);

        // Native host-tally lifecycle for two updateSubmitInfo records, one
        // HardwareBufferID release, and one IdlePowerOff record.
        let mut tally = REGION_C_GPU_CORE_PWR_ASSERT_COUNT;
        for (opcode, expected) in [(0x20, 1), (0x20, 2), (0x2e, 2), (0x0a, 3)] {
            if device_control_asserts_gpu_power(opcode).unwrap() {
                tally = tally.wrapping_add(1);
            }
            assert_eq!(tally, expected);
        }
    }

    #[test]
    fn status_block_pre_init_shape() {
        assert_eq!(STATUS_BLOCK_SIZE, 0x80);
        let mut status = vec![0xa5; STATUS_BLOCK_SIZE];
        encode_status_block(&mut status).unwrap();
        assert_eq!(get_u32(&status, 0x00), 0);
        assert_eq!(get_u32(&status, 0x04), 1);
        // +0x10 / +0x14 are post-ack firmware state and must be zero at handoff.
        assert_eq!(get_u32(&status, 0x10), 0);
        assert_eq!(get_u32(&status, 0x14), 0);
    }

    #[test]
    fn full_status_objects_and_bundle_views_are_carried() {
        let mut primary_status_b = vec![0xa5; PRIMARY_STATUS_B_OBJECT_SIZE];
        let fwctl = KERN + 0x5000_0000;
        encode_primary_status_b(fwctl, fwctl + CONTROL_RECORD_SIZE as u64, &mut primary_status_b)
            .unwrap();
        assert_eq!(get_u32(&primary_status_b, 0x04), 1);
        assert_eq!(get_u64(&primary_status_b, PRIMARY_STATUS_B_FWCTL_STATE), fwctl);
        assert_eq!(
            get_u64(&primary_status_b, PRIMARY_STATUS_B_FWCTL_RING),
            fwctl + CONTROL_RECORD_SIZE as u64
        );
        assert_eq!(get_u32(&primary_status_b, PRIMARY_STATUS_B_CONFIG_HEADER), 1);
        assert_eq!(primary_status_b[PRIMARY_STATUS_B_CONFIG_OFFSET + 0x10], 0x0f);

        let mut secondary_extra = [0xa5; STATUS_BLOCK_SIZE];
        encode_secondary_root_extra_1(&mut secondary_extra).unwrap();
        assert_eq!(secondary_extra[0x14], 1);
        assert_eq!(&secondary_extra[0x50..0x52], &[0x6a, 0x18]);

        let mut bundle = vec![0; NATIVE_SHARED_CLUSTER_SIZE];
        let state = KERN + 0x6000_0000;
        encode_bundle_static(state, &mut bundle).unwrap();
        assert_eq!(get_u64(&bundle, 0x8ed8), state);
        assert_eq!(bundle[0x80ac], 0x06);
        assert_eq!(bundle[BUNDLE_VIEW_OFFSETS[2] + 0xe45], 0xdc);
        assert_eq!(
            &bundle[BUNDLE_VIEW_OFFSETS[2] + 0x2ad7..BUNDLE_VIEW_OFFSETS[2] + 0x2adb],
            &[0xff, 0x7f, 0xff, 0x7f]
        );
    }

    #[test]
    fn instance_separation_invariants() {
        assert!(validate_pair(&primary(), &secondary()).is_ok());
        // The two roots are exactly 0x8000 apart.
        assert_eq!(SECONDARY_ROOT_DELTA, 0x8000);
        let mut bad = secondary();
        bad.root += 0x4000;
        assert_eq!(
            validate_pair(&primary(), &bad),
            Err(InitdataError::PairRootSeparation)
        );
        // The two instances' shared-region halves are 0x40000 apart.
        assert_eq!(SECONDARY_SHARED_REGION_DELTA, 0x40000);
        assert!(validate_shared_region_bases(KERN, KERN + 0x40000).is_ok());
        assert_eq!(
            validate_shared_region_bases(KERN, KERN + 0x8000),
            Err(InitdataError::SharedRegionSeparation)
        );
    }

    #[test]
    fn root_gate_rejects_the_m5_version_word() {
        let mut root = encode_root_bytes(InstanceRole::Primary);
        assert!(check_firmware_root_gates(&root).is_ok());
        // The M5/G17S word is answered with "Host and FW Interface is mismatched".
        put_u64(&mut root, 0x00, 0x0c8d_e339_0728_04c0);
        assert_eq!(
            check_firmware_root_gates(&root),
            Err(InitdataError::InterfaceMismatch)
        );
        put_u64(&mut root, 0x00, INITDATA_VERSION_WORD);
        put_u32(&mut root, 0x2c, 0);
        assert_eq!(
            check_firmware_root_gates(&root),
            Err(InitdataError::HostMappedAllocationsDisabled)
        );
    }

    #[test]
    fn evidence_flags_are_honest() {
        // The graph the firmware accepts and boots with is grounded.
        assert!(GROUNDED_LAYOUT_AVAILABLE);
        assert!(PARSE_ACCEPTED_GRAPH_AVAILABLE);
        // The measured performance values are carried, while ADT derivation,
        // The target-scoped bundle and status-B bytes are carried too.
        assert!(!ALL_HW_GLOBALS_VALUES_GROUNDED);
        assert!(PERF_TABLE_VALUES_ENCODED);
        assert!(BUNDLE_VIEW_CONTENT_CARRIED);
        assert!(PRIMARY_STATUS_B_FULL_OBJECT_CARRIED);
    }

    #[test]
    fn recorded_run_tables_match_the_serializer_totals() {
        // HWData excludes the seven captured QoS pointer bytes; the adjacent
        // state words and the remaining platform runs are preserved.
        let hw: usize = HW_DATA_OPAQUE_RUNS.iter().map(|(_, data)| data.len()).sum();
        assert_eq!(HW_DATA_OPAQUE_RUNS.len(), 29);
        assert_eq!(hw, 262);
        let region_c: usize = REGION_C_RUNS.iter().map(|(_, data)| data.len()).sum();
        assert_eq!(region_c, 180);
        // Every run fits its object.
        assert!(HW_DATA_OPAQUE_RUNS
            .iter()
            .all(|(off, data)| off + data.len() <= HW_DATA_SIZE));
        assert!(REGION_C_RUNS
            .iter()
            .all(|(off, data)| off + data.len() <= REGION_C_SIZE));
        // No opaque run touches firmware-state +0x2630.
        assert!(HW_DATA_OPAQUE_RUNS.iter().all(|(off, data)| {
            !(*off <= HW_FIRMWARE_STATE_OFFSET && HW_FIRMWARE_STATE_OFFSET < *off + data.len())
        }));
    }

    /// Pins the remaining documented ABI/geometry constants to their grounded
    /// values. They describe parts of the graph this offline module records but
    /// does not yet encode (the control-record framing and the full primary
    /// status-B object); asserting them keeps the
    /// recorded facts under test even before the encoders reach them.
    #[test]
    fn documented_constants_have_their_grounded_values() {
        assert_eq!(ROOT_ZERO_10, 0x10);
        assert_eq!(REGION_A_SIZE, 0x4000);
        assert_eq!(HW_DATA_BUNDLE_SIZE, 0xc000);
        assert_eq!(REGION_C_REQUIRED_OFFSET, 0xe50);
        assert_eq!(STATUS_BLOCK_POST_ACK_OFFSETS, [0x10, 0x14]);
        assert_eq!(PRIMARY_STATUS_B_OBJECT_SIZE, 0xeb00);
        // Device-control framing and the ownerless-UMA-free opening prefixes.
        assert_eq!(CONTROL_RECORD_SIZE, 0x40);
        assert_eq!(CONTROL_OPENING_PRIMARY_PRODUCER, 1);
        assert_eq!(COMPUTE_RUNTIME_CONTROL_RECORDS, 65);
        assert_eq!(COMPUTE_RUNTIME_CONTROL_FINAL_COUNTER, 66);
        assert_eq!(CONTROL_OPENING_SECONDARY_PRODUCER, 1);
        // Channel state-block stride within the compact grid.
        assert_eq!(CHANNEL_STATE_STRIDE, 0x40);
        // Performance-table geometry and measured values.
        assert_eq!(HW_PERF_LADDER_ENTRIES, 11);
        assert_eq!(HW_PERF_GROUP_REPEATS, [16, 1]);
        assert_eq!(HW_PERF_VOLTAGE_DELTA, 0x40);
        assert_eq!(HW_PERF_MEMORY_VOLTAGE_DELTA, 0x440);
        assert_eq!(HW_PERF_STATE_BLOCK_STRIDE, 0x40);
        assert_eq!(HW_PERF_FREQ_LADDER_B, 0x1808);
        assert_eq!(HW_PERF_SCALE_LADDER_B, 0x1848);
        assert_eq!(HW_PERF_RELATIVE_LADDER_A, 0x18c8);
        assert_eq!(HW_PERF_RELATIVE_LADDER_B, 0x1908);
        assert_eq!(HW_PERF_INDEX_MAP_A, 0x19c8);
        assert_eq!(HW_PERF_INDEX_MAP_B, 0x1a08);
    }
}
