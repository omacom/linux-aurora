// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! G17 firmware builds.
//!
//! Firmware builds that share the interface version ([`super::initdata::INTERFACE_VERSION`])
//! still differ in a few shared layouts and queue rules. A [`Variant`] names those differences
//! for one build; each SoC's [`crate::g17::Config`] points at the variant of the firmware it
//! runs.

/// Where a firmware build keeps the host words of a queue's pointer block
/// ([`super::queue::PointerBlock`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) struct PointerLayout {
    /// Offset of the item-ring producer.
    pub(crate) producer: usize,
    /// Offset of the word the host copies the producer to before it acknowledges a recovery,
    /// if the build has one.
    pub(crate) recovery_target: Option<usize>,
}

/// How a firmware build acknowledges the initialization root the host hands it.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum InitAck {
    /// With an `INITDATA_ACK` message on the firmware endpoint.
    Message,
    /// By setting word `+0x14` of the primary status A to 1; it sends no message.
    Status,
}

/// The initialization graph of a firmware build.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum InitGraph {
    /// Two instances, as the A18 Pro firmware (26A428) runs.
    G17P,
    /// One instance carrying both roles, with the 25G83 layouts ([`super::initdata::v25g83`]).
    V25G83,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum QueueRegistration {
    Firmware,
    PoweredRenderPairs,
    Mailbox,
}

/// The layouts and rules of one G17 firmware build.
#[derive(Copy, Clone)]
pub(crate) struct Variant {
    /// The initialization graph.
    pub(crate) graph: InitGraph,
    /// How the firmware acknowledges its initialization.
    pub(crate) init_ack: InitAck,
    /// The host ends the power-on transition with `POWER_ON_DONE` once every coprocessor
    /// is initialized.
    pub(crate) power_on_done: bool,
    /// First QID the host may use; the firmware keeps the lower ones for itself.
    pub(crate) qid_floor: usize,
    /// Host words of a queue's pointer block.
    pub(crate) pointers: PointerLayout,
    pub(crate) registration: QueueRegistration,
    /// Render kick entries may name an MCache table.
    pub(crate) mcache: bool,
    /// Renders may set the few-primitives auxiliary framebuffer flag.
    pub(crate) few_primitives: bool,
    /// Renders may carry DRM_ASAHI_RENDER_STAGE_PREFIXES: the tiling kick waits on its own,
    /// narrower render and compute prefixes, the fragment kick on the whole ones. The M5
    /// command set accepts them in its own validation.
    pub(crate) render_stage_prefixes: bool,
    /// The firmware runs ray-traversal work (ray queries and intersection programs).
    pub(crate) ray_traversal: bool,
    /// Ray-traversal commands run for one client at a time, with other clients' compute
    /// commands kept out meanwhile ([`crate::g17::traversal`]).
    pub(crate) traversal_exclusion: bool,
    /// Renders may carry ray-query stages. The M5 command set has none: it encodes no render
    /// ray-query state.
    pub(crate) render_ray_query: bool,
    /// Tiling descriptors report whether the fragment queue still had earlier kicks
    /// outstanding (`fragment_busy`).
    pub(crate) fragment_busy: bool,
    /// Render item rings carry lead commands: InitBufferManager before the first tiling pass
    /// of a buffer manager.
    pub(crate) lead_commands: bool,
    /// Register programs of the render descriptors.
    pub(crate) programs: super::render::RegisterPrograms,
    /// Use 0x1c890 = 1, 0x1c918 = 0 in V25G83-format tiling arrays (device/build policy).
    pub(crate) ta_mode_1: bool,
    pub(crate) compute: super::compute::ComputeProgram,
    pub(crate) opening_record: bool,
    pub(crate) split_firmware_area: bool,
    /// Uses the 26B123 initialization layouts and scheduler-release opcode.
    pub(crate) v26b123: bool,
    /// Command validation applies the M5 command set: resource commands, the M5 render and
    /// compute flags, 8x MSAA and the M5 tile-buffer limit.
    pub(crate) m5_commands: bool,
    /// The host holds the cores powered around a render pair's registration and its first
    /// work. The bare-metal M5 firmware powers the cores for work by its own policy: under a
    /// hold, the M5 Ultra's core-power witness reads unpowered until the hold is released.
    pub(crate) registration_power_hold: bool,
    /// A context takes its render pool identity when it is created, not at its first render,
    /// so that identities held by render graphs kept until the firmware stops refuse new
    /// contexts with an error instead of failing a context's first render. Set for the A18
    /// Pro only: the M4 Pro and Max keep the first-render claim until the claim at creation has
    /// been tested on that hardware.
    pub(crate) render_id_at_create: bool,
    /// Cached descriptor storage, queue records and queue configuration that come back
    /// after their owner's release was consumed are reused once the firmware has consumed
    /// every control record published before they came back, not only after a later one.
    /// Set for the A18 Pro. Elsewhere they keep the stricter fence of 34b438de, which fixed
    /// an M4 Pro crash on a recycled work state; extending this needs a create/destroy-heavy
    /// queue test on that hardware.
    pub(crate) objects_after_release: bool,
}

/// A firmware build a SoC supports, identified by its loaded TEXT.
pub(crate) struct Build {
    /// macOS version (major, minor, patch), used to order image candidates.
    pub(crate) os_version: [u32; 3],
    /// Name, for the log.
    pub(crate) name: &'static str,
    /// The admitted loaded images, measured from the `fw-text` region.
    pub(crate) images: &'static [crate::fwid::FirmwareBuild],
    /// Layouts and rules of the build.
    pub(crate) variant: &'static Variant,
    /// The SoC configuration this build runs with, which replaces the probed one once the
    /// build is identified; `None` keeps the probed configuration.
    pub(crate) config: Option<&'static crate::g17::Config>,
}

/// How a SoC's firmware build is determined.
pub(crate) enum Firmware {
    /// The SoC runs a single build, which is not identified (A18 Pro).
    Fixed(&'static Variant),
    /// One of these exact loaded images. The loader's `apple,firmware-version` is only
    /// a candidate-ordering hint because multiple OS releases can share an iBoot string.
    Identified(&'static [Build]),
}

/// The A18 Pro firmware (26A428).
pub(crate) const V26A428: Variant = Variant {
    graph: InitGraph::G17P,
    init_ack: InitAck::Message,
    power_on_done: true,
    qid_floor: 0,
    pointers: PointerLayout {
        producer: 0x40,
        recovery_target: Some(0x50),
    },
    registration: QueueRegistration::PoweredRenderPairs,
    mcache: true,
    few_primitives: true,
    render_stage_prefixes: true,
    ray_traversal: true,
    traversal_exclusion: true,
    render_ray_query: true,
    fragment_busy: true,
    lead_commands: false,
    programs: super::render::RegisterPrograms::G17P,
    ta_mode_1: false,
    compute: super::compute::ComputeProgram::G17P,
    opening_record: true,
    split_firmware_area: false,
    v26b123: false,
    m5_commands: false,
    registration_power_hold: true,
    render_id_at_create: true,
    objects_after_release: true,
};

/// The M4 Pro firmware (25G83, `mBoot-18000.161.10`).
pub(crate) const V25G83: Variant = Variant {
    graph: InitGraph::V25G83,
    init_ack: InitAck::Status,
    // Never sent on the M4 Pro: the firmware has retired its opening record when it accepts
    // its initialization.
    power_on_done: false,
    // QIDs below 32 belong to the firmware's bootstrap protocol.
    qid_floor: 32,
    pointers: PointerLayout {
        producer: 0x50,
        recovery_target: None,
    },
    // Whether this build tolerates the MMIO registration is unknown; the M4 Pro has only run
    // without it.
    registration: QueueRegistration::Firmware,
    mcache: false,
    // A fragment stage with the few-primitives flag hangs.
    few_primitives: false,
    // Not measured on this firmware.
    render_stage_prefixes: false,
    ray_traversal: true,
    // Not measured on this firmware.
    traversal_exclusion: false,
    render_ray_query: true,
    // When a tiling pass runs out of memory with this set, the firmware resumes it without
    // adding memory and the tiler stops silently; macOS always writes 0.
    fragment_busy: false,
    lead_commands: true,
    programs: super::render::RegisterPrograms::V25G83,
    ta_mode_1: false,
    compute: super::compute::ComputeProgram::G17P,
    opening_record: true,
    split_firmware_area: false,
    v26b123: false,
    m5_commands: false,
    registration_power_hold: true,
    render_id_at_create: false,
    objects_after_release: false,
};

/// The 26A428 firmware of the M5 GPUs, which runs the M5 protocol (the M5 graph, queues and
/// command set).
pub(crate) const V26A428_M5: Variant = Variant {
    m5_commands: true,
    // Not measured on the M5 firmware.
    traversal_exclusion: false,
    render_ray_query: false,
    registration_power_hold: false,
    render_id_at_create: false,
    objects_after_release: false,
    ..V26A428
};

pub(crate) const V25F84: Variant = Variant {
    qid_floor: 0,
    registration: QueueRegistration::Mailbox,
    compute: super::compute::ComputeProgram::V25F84,
    opening_record: false,
    split_firmware_area: true,
    ..V25G83
};

/// The T6041 27.2 firmware (26B123, `mBoot-20457.41.3`).
pub(crate) const V26B123: Variant = Variant {
    split_firmware_area: false,
    v26b123: true,
    ..V25F84
};

const _: () = {
    assert!(super::queue::PointerBlock::valid_layout(&V26A428.pointers));
    assert!(super::queue::PointerBlock::valid_layout(&V25G83.pointers));
    assert!(super::queue::PointerBlock::valid_layout(&V25F84.pointers));
    assert!(super::queue::PointerBlock::valid_layout(&V26B123.pointers));
};
