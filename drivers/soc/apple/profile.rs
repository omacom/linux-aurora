// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj
// Copyright 2026 Ryan Murray

//! Typed platform profiles for the SEP driver.
//!
//! Every SoC-specific fact the driver needs — the shared-memory capacity, the
//! boot handshake, the OS-identity source and the sensor transport — is named
//! here once. The driver reads addresses from the device tree, never scans for
//! them, and never patches properties at runtime.
//!
//! Five bring-up targets are modelled:
//!
//! * `T8103` / J313 (MacBook Air, M1): the host boots the SEP with the boot
//!   endpoint handshake over a 0x30000 shared-memory window.
//! * `T6000` / J316s (MacBook Pro 16", M1 Pro): the T8103 boot handshake and
//!   key store, with the sensor on the T6020 SPI2 address.
//! * `T6020` / J414s (MacBook Pro 14", M2 Pro): the driver does the warm
//!   single-message registration over a 0x40000 window. Before system
//!   firmware 26.3 the enclave speaks the T8103 key store.
//! * `T8140` / J700 (MacBook Neo, A18 Pro): iBoot boots sepOS before the AP
//!   OS, so the driver uses the warm registration path.
//! * `T8112` / J413, J415 (MacBook Air, M2): iBoot hands over a running SEP and
//!   a powered sensor, so the driver uses the warm registration path.
//! * `T8122` / J613, J615 (MacBook Air, M3): the T8112 warm registration with
//!   the T8140 SEP DART aperture above 4 GiB.

// The boot handshake, identity source, sensor and DART fields are consumed by
// the boot-endpoint, identity and sensor-transport paths.
#![allow(dead_code)]

use kernel::of;
use kernel::pr_info_once;
use kernel::prelude::*;

/// How the driver brings the shared-memory table to the SEP.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Bootstrap {
    /// Cold boot: send TZ0, map the firmware reserved region and answer the
    /// second boot acknowledgement with the firmware address and `SET_SHMEM`.
    Boot,
    /// Warm attach: send the single shared-memory registration message with the
    /// table address and size.
    WarmRegister,
}

/// Where the 16-byte OS identity comes from.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum IdentitySource {
    /// `/chosen/apfs-preboot-uuid`, the value the firmware handed to this boot
    /// and the boot chain forwards. A malformed or absent property is refused;
    /// the identity is never invented.
    Chosen,
    /// A UUID provisioned in the Linux host-state store on first bring-up.
    HostPersisted,
}

/// The root-key input and loading point verified for each nonce-session board.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum SensorRootKey {
    None,
    /// Preserve J700's signed container upload before SESSION_SHARE.
    J700,
    /// J715's device-specific FDR MSRk body, uploaded only after status 0x16.
    J715,
}

impl SensorRootKey {
    pub(crate) const fn firmware(self) -> Option<&'static CStr> {
        match self {
            Self::None => None,
            Self::J700 => Some(c"apple/msrk-j700.bin"),
            Self::J715 => Some(c"apple/msrk-j715.bin"),
        }
    }

    /// Only the observed J715 FDR body size is qualified. The enclave checks
    /// its authenticity; a matching length alone does not authenticate it.
    pub(crate) const fn accepts_length(self, length: usize) -> bool {
        match self {
            Self::None => false,
            Self::J700 => length != 0,
            Self::J715 => length == 10_848,
        }
    }

    pub(crate) fn recover_share(self, status: Option<u32>) -> bool {
        self == Self::J715 && status == Some(0x16)
    }
}

/// SPI mode as the device tree spells it. Mode 1 is CPOL=0/CPHA=1 (`spi-cpha`);
/// mode 2 is CPOL=1/CPHA=0 (`spi-cpol`).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum SpiMode {
    Mode1,
    Mode2,
}

impl SpiMode {
    /// The value the SPI shim programs into `spi_setup()`, using the kernel's
    /// `SPI_CPHA`/`SPI_CPOL` bits.
    pub(crate) const fn wire(self) -> u32 {
        match self {
            SpiMode::Mode1 => 1, // SPI_CPHA
            SpiMode::Mode2 => 2, // SPI_CPOL
        }
    }

    pub(crate) const fn name(self) -> &'static CStr {
        match self {
            SpiMode::Mode1 => c"mode 1 (CPOL=0 CPHA=1)",
            SpiMode::Mode2 => c"mode 2 (CPOL=1 CPHA=0)",
        }
    }
}

pub(crate) struct SensorProfile {
    /// SPI controller register base. Diagnostics only: the device tree is
    /// authoritative and the driver never enables or creates the bus.
    pub(crate) controller_base: u64,
    pub(crate) chip_select: u32,
    pub(crate) max_hz: u32,
    /// Chip-select setup and hold in nanoseconds.
    pub(crate) cs_setup_ns: u32,
    pub(crate) cs_hold_ns: u32,
    pub(crate) mode: SpiMode,
    pub(crate) expected_id: u16,
    /// Capture traffic stays disabled until a qualified DMA transport exists;
    /// the PIO path is status-only.
    pub(crate) capture_qualified: bool,
}

/// Identity keybag `CREATE_KEYBAG` field encoding. The first word is the
/// codec's struct version: the 13.5 enclave (T8103) takes versions 0-2 only
/// and an identity is created with version 2, while the T6020 enclave on
/// newer system firmware takes variant 5. Kept per profile so the T8103 path
/// is correct without disturbing the proven T6020 encoding (hardware-verified
/// for enrol, match, and reboot).
pub(crate) struct KeybagCreate {
    /// First word: struct version (request variant), echoed back in the reply.
    pub(crate) variant: u32,
    /// Third word: bag type. Identity is `0x400000` on 13.5; `0` on variant
    /// 5, which distinguishes the bag by the variant word instead.
    pub(crate) bag_type: u32,
    /// Fourth word: create argument / parent handle.
    pub(crate) arg: i32,
}

/// Which key-store protocol the enclave speaks. It follows the sepOS the
/// firmware hands the SEP, not the SoC alone: the T8103 stub boots the 13.5
/// (22G74) sepOS, whose request shapes are the 13.5 ones; the T6020 sepOS
/// speaks them too before system firmware 26.3 (see [`t6020_profile`]).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum KeyStore {
    /// macOS 13.5's key store. Before any other request the endpoint is
    /// initialised with `0x4d`, then
    /// `set_env` carrying the ADT's `/defaults` `cpx-encryption-mode`.
    Sepos13 { cpx_encryption_mode: u32 },
    /// The hardware-verified T6020 encoding (variant-5 create).
    Variant5,
}

/// The SEP DART DMA window a warm-attach profile programs when the device
/// tree does not describe it, as `apple,dma-offset` and `apple,dma-range`
/// cells.
pub(crate) struct DartWindow {
    pub(crate) name: &'static str,
    pub(crate) offset: [u32; 2],
    pub(crate) range: [u32; 4],
}

/// IOVAs 0..4 GiB, offset by BIT(40): `[0x100, 0]`. `[0x1, 0]` would be
/// BIT(32) and miss the page tables.
const DART_HIGH_ALIAS: DartWindow = DartWindow {
    name: "offset BIT(40), range 0..4 GiB",
    offset: [0x100, 0x0],
    range: [0, 0, 1, 0],
};

/// The T8112 SEP DART has a 32-bit input aperture, 0x4000 + 0xffff0000, and
/// no high alias.
const DART_T8112: DartWindow = DartWindow {
    name: "no offset, range 0x4000..4 GiB",
    offset: [0x0, 0x0],
    range: [0x0, 0x4000, 0x0, 0xffff_0000],
};

/// The T8132 SEP DART input aperture is the BIT(40) alias itself,
/// 0x100_0000_4000 + 0xffff0000 (J715 ADT `dart-sep` vm-base-0 and
/// vm-size-0), given as a range with no offset. Unlike [`DART_HIGH_ALIAS`]
/// it reaches the SEP whether or not the DART driver applies
/// `apple,dma-offset`.
const DART_T8132: DartWindow = DartWindow {
    name: "no offset, range 0x10000004000..+0xffff0000",
    offset: [0x0, 0x0],
    range: [0x100, 0x4000, 0x0, 0xffff_0000],
};

pub(crate) struct PlatformProfile {
    pub(crate) name: &'static str,
    /// Capacity of the boot shared-memory window. The layout is still checked
    /// against the manifests; this only bounds the allocation.
    pub(crate) shmem_capacity: usize,
    /// Fourcc of the first shared-memory item. The two paths spell it
    /// differently, so each profile carries its own spelling.
    pub(crate) shmem_first_item: &'static [u8; 4],
    pub(crate) bootstrap: Bootstrap,
    pub(crate) identity: IdentitySource,
    pub(crate) sensor: SensorProfile,
    /// Require a static `apple,dma-range` on the SEP DART. The T6020 SEP only
    /// accepts IOVAs below 4 GiB; T8103 works with the stock DART aperture.
    pub(crate) dart_range_required: bool,
    /// Whether the SEP DART IOVA window needs a DMA mask wider than 32 bits.
    pub(crate) wide_dma_mask: bool,
    /// The SEP DART DMA window programmed on warm attach (see [`DartWindow`]).
    pub(crate) dart_window: &'static DartWindow,
    /// Reserved-memory region holding the SEP firmware image (cold-boot path).
    pub(crate) firmware_region: &'static CStr,
    /// Identity keybag CREATE_KEYBAG field encoding (per-SoC; see [`KeybagCreate`]).
    pub(crate) keybag_create: KeybagCreate,
    /// Require nonce authentication and signed sensor material before capture.
    pub(crate) nonce_session: bool,
    pub(crate) sensor_root_key: SensorRootKey,
    /// Key-store protocol generation (see [`KeyStore`]).
    pub(crate) key_store: KeyStore,
    /// Platforms with reboot-tested owner export and user-before-master saves.
    pub(crate) persistent_enrol: bool,
    /// The `xart_guard` level when the parameter is left at its default (see
    /// [`crate::xart_guard`]).
    pub(crate) xart_guard: u8,
}

pub(crate) const T8103: PlatformProfile = PlatformProfile {
    name: "T8103/J313",
    shmem_capacity: 0x3_0000,
    shmem_first_item: b"CNIP",
    bootstrap: Bootstrap::Boot,
    identity: IdentitySource::Chosen,
    sensor: SensorProfile {
        controller_base: 0x2_3510_8000,
        chip_select: 0,
        max_hz: 8_000_000,
        cs_setup_ns: 20,
        cs_hold_ns: 20,
        mode: SpiMode::Mode1,
        expected_id: 0x3352,
        capture_qualified: false,
    },
    dart_range_required: false,
    wide_dma_mask: false,
    dart_window: &DART_HIGH_ALIAS,
    firmware_region: c"sepfw",
    // The 13.5 key store creates an identity bag with
    // version 2, type 0x400000, parent -1 (it accepts only -1 or <= -10).
    keybag_create: KeybagCreate {
        variant: 2,
        bag_type: 0x40_0000,
        arg: -1,
    },
    // The j293 and j313 ADTs both carry `/defaults` `cpx-encryption-mode = 2`.
    key_store: KeyStore::Sepos13 {
        cpx_encryption_mode: 2,
    },
    persistent_enrol: true,
    nonce_session: false,
    sensor_root_key: SensorRootKey::None,
    // macOS shares this enclave: keep Linux to its own xART records.
    xart_guard: 1,
};

/// M1 Pro (J316s). The M1 family cold-boots its SEP like T8103, so the boot
/// path, shared-memory geometry and key store follow T8103. The sensor sits on
/// SPI2 at the T6020 address, in the mode T6020 verified: the J313, J316s and
/// J414s platform device trees describe the same sensor (id 0x3352)
/// identically. Enrolment persistence is reboot-tested on J314s.
pub(crate) const T6000: PlatformProfile = PlatformProfile {
    name: "T6000/J316s",
    shmem_capacity: 0x3_0000,
    shmem_first_item: b"CNIP",
    bootstrap: Bootstrap::Boot,
    identity: IdentitySource::Chosen,
    sensor: SensorProfile {
        controller_base: 0x3_9b10_8000,
        chip_select: 0,
        max_hz: 8_000_000,
        cs_setup_ns: 20,
        cs_hold_ns: 20,
        mode: SpiMode::Mode2,
        expected_id: 0x3352,
        capture_qualified: false,
    },
    dart_range_required: false,
    wide_dma_mask: false,
    dart_window: &DART_HIGH_ALIAS,
    firmware_region: c"sepfw",
    keybag_create: KeybagCreate {
        variant: T8103.keybag_create.variant,
        bag_type: T8103.keybag_create.bag_type,
        arg: T8103.keybag_create.arg,
    },
    // The J316s ADT carries `/defaults` `cpx-encryption-mode = 2`.
    key_store: KeyStore::Sepos13 {
        cpx_encryption_mode: 2,
    },
    persistent_enrol: true,
    nonce_session: false,
    sensor_root_key: SensorRootKey::None,
    // As T8103. On J316s, creating the identity keybag also rewrites a
    // session record that predates Linux. The guard refuses that, so the
    // keybag can only be created there with xart_guard=0.
    xart_guard: 1,
};

pub(crate) const T6020: PlatformProfile = PlatformProfile {
    name: "T6020/J414s",
    shmem_capacity: 0x4_0000,
    shmem_first_item: b"CINP",
    bootstrap: Bootstrap::WarmRegister,
    identity: IdentitySource::HostPersisted,
    sensor: SensorProfile {
        controller_base: 0x3_9b10_8000,
        chip_select: 0,
        max_hz: 8_000_000,
        cs_setup_ns: 20,
        cs_hold_ns: 20,
        mode: SpiMode::Mode2,
        expected_id: 0x3352,
        capture_qualified: false,
    },
    dart_range_required: true,
    wide_dma_mask: false,
    dart_window: &DART_HIGH_ALIAS,
    firmware_region: c"sepfw",
    // Proven encoding: the lenient enclave takes the variant in the first word.
    keybag_create: KeybagCreate {
        variant: 5,
        bag_type: 0,
        arg: -1,
    },
    key_store: KeyStore::Variant5,
    persistent_enrol: true,
    nonce_session: false,
    sensor_root_key: SensorRootKey::None,
    // A warm-attach enclave shared with macOS, as on T8112.
    xart_guard: 1,
};

/// M2 Pro/Max on system firmware before 26.3. Their enclave refuses the
/// variant-5 create (status -13, empty reply: J414s on 26.0, J416s on 26.2)
/// and takes the 13.5 key store, as T8103 does; J414s on 26.0 enrols,
/// matches, signs with the reference key and restores after a reboot this way.
/// Everything else is [`T6020`].
pub(crate) const T6020_SEPOS13: PlatformProfile = PlatformProfile {
    name: "T6020/J414s (13.5 key store)",
    keybag_create: KeybagCreate {
        variant: T8103.keybag_create.variant,
        bag_type: T8103.keybag_create.bag_type,
        arg: T8103.keybag_create.arg,
    },
    key_store: KeyStore::Sepos13 {
        cpx_encryption_mode: 2,
    },
    ..T6020
};

/// MacBook Neo. The J700 ADT records a pre-booted SEP, a spi2 Mesa sensor
/// (0x3356), and a SEP DART aperture above 4 GiB. These values describe that
/// board; device-tree nodes remain authoritative for addresses and resources.
pub(crate) const T8140: PlatformProfile = PlatformProfile {
    key_store: KeyStore::Variant5,
    persistent_enrol: true,
    name: "T8140/J700",
    shmem_capacity: 0x4_0000,
    shmem_first_item: b"CINP",
    bootstrap: Bootstrap::WarmRegister,
    identity: IdentitySource::Chosen,
    sensor: SensorProfile {
        controller_base: 0x3_8510_8000,
        chip_select: 0,
        max_hz: 8_000_000,
        cs_setup_ns: 20,
        cs_hold_ns: 20,
        mode: SpiMode::Mode2,
        expected_id: 0x3356,
        capture_qualified: false,
    },
    dart_range_required: true,
    wide_dma_mask: true,
    dart_window: &DART_HIGH_ALIAS,
    firmware_region: c"sepfw",
    // J700 hardware refused the strict encoding and accepted this T6020 form
    // (first verified on hardware 2026-09-19).
    keybag_create: KeybagCreate {
        variant: 5,
        bag_type: 0,
        arg: -1,
    },
    nonce_session: true,
    sensor_root_key: SensorRootKey::J700,
    // Linux records from before the guard's list exist here, and Linux has
    // left the macOS records unchanged on this platform.
    xart_guard: 0,
};

/// MacBook Air M2 (J413, J415). iBoot leaves the SEP running and the Mesa
/// sensor powered (J415: AP GPIO 178 is a driven-high output at handoff), so
/// this is a warm-registration platform like T6020. The J415 ADT describes the
/// sensor exactly as J413's does: spi2 at the T8103 address, power on AP GPIO
/// 178, data-ready on 177, 8 MHz, chip select 0. The SEP DART aperture is
/// 0x4000 + 0xffff0000, below 4 GiB, so no wide DMA mask. `/defaults`
/// carries `cpx-encryption-mode = 2`.
///
/// The shared-memory geometry and key-store dialect follow the other
/// warm-registration platforms (T6020 on current firmware, T8140).
pub(crate) const T8112: PlatformProfile = PlatformProfile {
    name: "T8112/J415",
    shmem_capacity: 0x4_0000,
    shmem_first_item: b"CINP",
    bootstrap: Bootstrap::WarmRegister,
    identity: IdentitySource::Chosen,
    sensor: SensorProfile {
        controller_base: 0x2_3510_8000,
        chip_select: 0,
        max_hz: 8_000_000,
        cs_setup_ns: 20,
        cs_hold_ns: 20,
        mode: SpiMode::Mode1,
        expected_id: 0x3352,
        capture_qualified: false,
    },
    dart_range_required: true,
    wide_dma_mask: false,
    dart_window: &DART_T8112,
    firmware_region: c"sepfw",
    // J415 accepted the 13.5 GET_CAPABILITIES/SET_ENV init but refused the
    // strict CREATE_KEYBAG (mailbox status -1, empty reply), as J700 did;
    // use the variant-5 form like the other warm-registration platforms.
    keybag_create: KeybagCreate {
        variant: 5,
        bag_type: 0,
        arg: -1,
    },
    key_store: KeyStore::Variant5,
    persistent_enrol: true,
    nonce_session: false,
    sensor_root_key: SensorRootKey::None,
    // Guarded from its first Linux boot (2026-10-16): no record refused.
    xart_guard: 1,
};

/// MacBook Air M3 (J613, J615). The J613 and J615 ADTs (26.6.2, 25G83) are
/// wired identically: the Mesa sensor (`R3`) sits on spi2 at `0x2a1108000`,
/// 8 MHz, chip select 0, with the same `reg` bytes as J415's, so this keeps
/// J415's SPI mode. Its power is on the NUB GPIO controller (pin 29), not an
/// AP GPIO, and its data-ready line is AP GPIO 199. The SEP is
/// `iop-sep,ascwrap-v6` and its DART's input aperture is
/// `0x100_0000_4000 + 0xffff0000`, the BIT(40) alias J700 uses, so this needs
/// the wide DMA mask. `/defaults` carries `cpx-encryption-mode = 2`.
///
/// Otherwise a warm-registration platform like T8112: the shared-memory
/// geometry and key-store dialect follow it. J613 on system firmware 27.0
/// (OS firmware 26.6.2) attaches, provisions the keybag, enrols, matches and
/// restores the enrolment after a reboot; J615 has not been run.
pub(crate) const T8122: PlatformProfile = PlatformProfile {
    name: "T8122/J613",
    sensor: SensorProfile {
        controller_base: 0x2_a110_8000,
        ..T8112.sensor
    },
    wide_dma_mask: true,
    // Not T8112's 32-bit window: this DART needs the wide mask.
    dart_window: &DART_HIGH_ALIAS,
    // A warm-attach enclave shared with macOS, as on T8112.
    xart_guard: 1,
    ..T8112
};

/// MacBook Air 15-inch M4 (J715). The J715 ADT has `sepfw-booted`: iBoot
/// hands over a running SEP, so this is T8122's warm registration,
/// shared-memory geometry and variant-5 key store. The SEP DART window is the
/// ADT's own (see [`DART_T8132`]). The Mesa sensor sits on spi2 at
/// `0x3ad108000`; its identity and wiring are unchecked, so capture stays
/// unqualified. Like the M3, its enclave is shared with macOS.
/// The successful J715 session uses nonce authentication, FDR MSRk recovery
/// after SESSION_SHARE status 0x16, and source-6 FSC2 calibration.
pub(crate) const T8132: PlatformProfile = PlatformProfile {
    name: "T8132/J715",
    nonce_session: true,
    sensor_root_key: SensorRootKey::J715,
    sensor: SensorProfile {
        controller_base: 0x3_ad10_8000,
        capture_qualified: false,
        ..T8122.sensor
    },
    dart_window: &DART_T8132,
    ..T8122
};

static_assert!(T8103.shmem_capacity == 0x30000);
static_assert!(T6020.shmem_capacity == 0x40000);
static_assert!(T8140.shmem_capacity == 0x40000);
static_assert!(T8103.sensor.controller_base == 0x235108000);
static_assert!(T6020.sensor.controller_base == 0x39b108000);
static_assert!(T8140.sensor.controller_base == 0x385108000);
static_assert!(T8103.sensor.expected_id == T6020.sensor.expected_id);
static_assert!(T6000.shmem_capacity == T8103.shmem_capacity);
static_assert!(T6000.sensor.controller_base == T6020.sensor.controller_base);
static_assert!(T6000.sensor.expected_id == T6020.sensor.expected_id);
static_assert!(T8112.shmem_capacity == T6020.shmem_capacity);
static_assert!(T8112.sensor.controller_base == T8103.sensor.controller_base);
static_assert!(T8122.shmem_capacity == T8112.shmem_capacity);
static_assert!(T8122.sensor.controller_base == 0x2a1108000);
static_assert!(T8122.wide_dma_mask);
static_assert!(T8132.wide_dma_mask);
static_assert!(T8132.shmem_capacity == T8122.shmem_capacity);

/// Whether the machine root declares `compatible`.
///
/// The root property is a NUL-separated list, so whole entries are compared
/// instead of substrings: `apple,t8103` cannot match a longer unrelated value.
pub(crate) fn machine_has(compatible: &[u8]) -> bool {
    let Some(root) = of::root() else {
        return false;
    };
    let Ok(list) = root.get_property::<KVec<u8>>(c"compatible") else {
        return false;
    };

    let mut at = 0usize;
    while at < list.len() {
        let end = match list[at..].iter().position(|&byte| byte == 0) {
            Some(offset) => at + offset,
            None => list.len(),
        };
        if &list[at..end] == compatible {
            return true;
        }
        at = end + 1;
    }

    false
}

/// iBoot builds before 26.3's (betas 13822.80, release 13822.81) get the 13.5
/// key store. The 26.0 and 26.2 enclaves refuse the variant-5 create, while
/// 26.6.2 and 27.0 take it; 26.3 to 26.5 are untested and keep variant 5, as
/// before. Firmware from 26.4 on has an `mBoot-` build and never gets here.
const T6020_SEPOS13_BEFORE: (u32, u32) = (13822, 80);

/// The leading `major.minor` of an `iBoot-` build string, as m1n1 copies the
/// machine's own iBoot into `asahi,iboot1-version` ("iBoot-13822.61.10").
/// Anything else, including an `mBoot-` build, yields `None`. Unlike
/// `asahi,system-fw-version`, this does not depend on m1n1's version table,
/// so every m1n1 gives the same answer.
fn parse_iboot_build(raw: &[u8]) -> Option<(u32, u32)> {
    fn number(text: &[u8]) -> Option<(u32, &[u8])> {
        let digits = text.iter().take_while(|byte| byte.is_ascii_digit()).count();
        if digits == 0 || digits > 6 {
            return None;
        }
        let value = text[..digits]
            .iter()
            .fold(0u32, |value, &byte| value * 10 + u32::from(byte - b'0'));
        Some((value, &text[digits..]))
    }

    let text = match raw.iter().position(|&byte| byte == 0) {
        Some(end) => &raw[..end],
        None => raw,
    };
    let (major, rest) = number(text.strip_prefix(b"iBoot-")?)?;
    let (minor, rest) = number(rest.strip_prefix(b".")?)?;
    match rest.first() {
        None | Some(b'.') => Some((major, minor)),
        Some(_) => None,
    }
}

fn t6020_uses_sepos13(iboot: Option<&[u8]>) -> bool {
    iboot
        .and_then(parse_iboot_build)
        .is_some_and(|build| build < T6020_SEPOS13_BEFORE)
}

fn chosen_string(name: &CStr) -> Option<KVec<u8>> {
    of::chosen()?.get_property::<KVec<u8>>(name).ok()
}

/// A `/chosen` string for the log, without its NUL.
fn shown(raw: &Option<KVec<u8>>) -> &kernel::str::BStr {
    let raw = raw.as_deref().unwrap_or(&b"none"[..]);
    let end = raw.iter().position(|&byte| byte == 0).unwrap_or(raw.len());
    kernel::str::BStr::from_bytes(&raw[..end])
}

/// The M2 Pro/Max profile for the machine's system firmware. Only an iBoot
/// build before 26.3's selects the 13.5 key store; anything else, including
/// a missing property, keeps [`T6020`], so a machine that works today keeps
/// the encoding it works with.
fn t6020_profile() -> &'static PlatformProfile {
    let iboot = chosen_string(c"asahi,iboot1-version");
    let firmware = chosen_string(c"asahi,system-fw-version");
    let os_firmware = chosen_string(c"asahi,os-fw-version");
    let sepos13 = t6020_uses_sepos13(iboot.as_deref());
    let profile = if sepos13 { &T6020_SEPOS13 } else { &T6020 };
    pr_info_once!(
        "M2 Pro/Max on system firmware {} ({}), OS firmware {}: using {}\n",
        shown(&firmware),
        shown(&iboot),
        shown(&os_firmware),
        if sepos13 {
            "the 13.5 key store"
        } else {
            "the variant-5 key store"
        }
    );
    profile
}

/// Select the profile for the running machine. An unsupported SoC is refused
/// rather than guessed at, so the driver cannot run a handshake with the wrong
/// geometry.
pub(crate) fn detect() -> Result<&'static PlatformProfile> {
    if machine_has(b"apple,t8103") {
        return Ok(&T8103);
    }
    if machine_has(b"apple,t6020") {
        return Ok(t6020_profile());
    }
    // t6020.dtsi is defined as a cut-down t6021: it includes t6021.dtsi and
    // disables the parts the smaller die lacks. Both pull in t602x-die0.dtsi,
    // so the SEP, its DART and the SPI controller sit at the same addresses
    // with the same interrupts on either part, and the T6020 constants apply
    // to the M2 Max unchanged.
    if machine_has(b"apple,t6021") {
        return Ok(t6020_profile());
    }
    if machine_has(b"apple,t6000") {
        return Ok(&T6000);
    }
    // t6000.dtsi is defined as a cut-down t6001: it includes t6001.dtsi and
    // deletes the parts the smaller die lacks. Both pull in t600x-die0.dtsi,
    // so the SEP, its mailbox, its DART and the SPI controller sit at the same
    // addresses with the same interrupts on either part, and the T6000
    // constants apply to the M1 Max unchanged. The Mac Studio (j375c) is also
    // a t6001 but leaves the SEP disabled, so nothing binds there; apple,t6002
    // (M1 Ultra, no built-in sensor) stays unmapped.
    if machine_has(b"apple,t6001") {
        return Ok(&T6000);
    }
    if machine_has(b"apple,t8140") {
        return Ok(&T8140);
    }
    if machine_has(b"apple,t8112") {
        return Ok(&T8112);
    }
    // Only the boards whose ADTs were read: the other T8122 Macs (J433, J434,
    // J504) share the SoC but their sensor wiring is unchecked.
    if machine_has(b"apple,t8122") && (machine_has(b"apple,j613") || machine_has(b"apple,j615")) {
        return Ok(&T8122);
    }
    // Only the board whose ADT was read.
    if machine_has(b"apple,t8132") && machine_has(b"apple,j715") {
        return Ok(&T8132);
    }
    Err(ENODEV)
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[::kernel::macros::kunit_tests(apple_sep_profile)]
mod tests {
    use super::*;

    #[test]
    fn old_firmware_uses_the_legacy_key_store() {
        assert!(t6020_uses_sepos13(Some(b"iBoot-13822.79.1\0")));
        assert!(t6020_uses_sepos13(Some(b"iBoot-13821.999\0")));
        assert!(!t6020_uses_sepos13(Some(b"iBoot-13822.80\0")));
        assert!(!t6020_uses_sepos13(Some(b"iBoot-13822.81.2\0")));
    }

    #[test]
    fn unknown_firmware_preserves_the_current_dialect() {
        assert!(!t6020_uses_sepos13(None));
        for raw in [
            &b""[..],
            &b"mBoot-1.2"[..],
            &b"iBoot-.2"[..],
            &b"iBoot-1."[..],
            &b"iBoot-1234567.2"[..],
            &b"iBoot-1.1234567"[..],
            &b"iBoot-1.2x"[..],
        ] {
            assert!(!t6020_uses_sepos13(Some(raw)));
        }
    }

    #[test]
    fn profile_extensions_preserve_authenticated_neo_geometry() {
        assert!(T8140.nonce_session);
        assert!(T8140.wide_dma_mask);
        assert!(!T6020_SEPOS13.nonce_session);
        assert!(matches!(T6020_SEPOS13.key_store, KeyStore::Sepos13 { .. }));
        assert!(T6020_SEPOS13.shmem_capacity == T6020.shmem_capacity);
        assert!(T6020_SEPOS13.sensor.controller_base == T6020.sensor.controller_base);
        assert!(!T8122.nonce_session);
        assert!(T8122.wide_dma_mask);
        assert!(T8122.shmem_capacity == T8112.shmem_capacity);
        assert!(T8122.sensor.controller_base == 0x2_a110_8000);
        assert!(T8132.nonce_session);
        assert!(T8132.wide_dma_mask);
        assert!(T8132.xart_guard == T8122.xart_guard);
        assert!(!T8132.sensor.capture_qualified);
    }
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[kernel::macros::kunit_tests(apple_sep_sensor_root_key)]
mod root_key_tests {
    use super::*;

    #[test]
    fn non_nonce_boards_never_load_or_recover_a_root_key() {
        // Given the existing non-nonce profiles, including inherited ones.
        for profile in [&T8103, &T6000, &T6020, &T6020_SEPOS13, &T8112, &T8122] {
            // When selecting their root-key policy.
            let policy = profile.sensor_root_key;
            // Then no file, upload or recovery is selected.
            assert!(!profile.nonce_session);
            assert!(policy.firmware().is_none());
            assert!(!policy.accepts_length(10_848));
            assert!(!policy.recover_share(Some(0x16)));
        }
    }

    #[test]
    fn j700_preserves_its_nonempty_container_without_share_recovery() {
        // Given J700's existing signed-container policy.
        let policy = T8140.sensor_root_key;
        // When selecting firmware and handling the observed refusal.
        let name = policy.firmware();
        // Then the existing name and nonempty input rule remain in force.
        assert!(T8140.nonce_session);
        assert!(policy == SensorRootKey::J700);
        assert!(name == Some(c"apple/msrk-j700.bin"));
        assert!(!policy.accepts_length(0));
        assert!(policy.accepts_length(1));
        assert!(policy.accepts_length(1726));
        assert!(!policy.recover_share(Some(0x16)));
    }

    #[test]
    fn j715_accepts_only_the_qualified_fdr_body_length() {
        // Given J715's nonce-session profile.
        let policy = T8132.sensor_root_key;
        // When selecting firmware and checking input sizes.
        let name = policy.firmware();
        // Then the public board name selects only the qualified FDR size.
        assert!(T8132.nonce_session);
        assert!(policy == SensorRootKey::J715);
        assert!(name == Some(c"apple/msrk-j715.bin"));
        assert!(policy.accepts_length(10_848));
        for length in [0, 1, 1726, 10_847, 10_849, usize::MAX] {
            assert!(!policy.accepts_length(length));
        }
    }

    #[test]
    fn j715_recovers_only_an_explicit_share_status_16() {
        // Given J715's recovery policy.
        let policy = T8132.sensor_root_key;
        // When classifying answered and missing statuses.
        let recover = policy.recover_share(Some(0x16));
        // Then only the observed refusal selects the single retry path.
        assert!(recover);
        for status in [
            None,
            Some(0),
            Some(1),
            Some(0x15),
            Some(0x17),
            Some(u32::MAX),
        ] {
            assert!(!policy.recover_share(status));
        }
    }
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[kernel::macros::kunit_tests(apple_sep_dart_window)]
mod dart_window_tests {
    use super::*;

    /// The 64-bit value of `apple,dma-offset`.
    fn offset(window: &DartWindow) -> u64 {
        (u64::from(window.offset[0]) << 32) | u64::from(window.offset[1])
    }

    /// `apple,dma-range` as the IOVAs it starts and ends at.
    fn range(window: &DartWindow) -> (u64, u64) {
        let start = (u64::from(window.range[0]) << 32) | u64::from(window.range[1]);
        let size = (u64::from(window.range[2]) << 32) | u64::from(window.range[3]);
        (start, start + size)
    }

    #[test]
    fn t8112_has_no_offset_and_stays_below_4_gib() {
        assert_eq!(offset(T8112.dart_window), 0);
        assert_eq!(range(T8112.dart_window), (0x4000, 0x4000 + 0xffff_0000));
        // The whole window fits the 32-bit DART input aperture.
        assert!(range(T8112.dart_window).1 <= 1 << 32);
    }

    #[test]
    fn the_other_profiles_keep_the_bit40_window() {
        for profile in [&T8103, &T6000, &T6020, &T8140, &T6020_SEPOS13, &T8122] {
            assert_eq!(offset(profile.dart_window), 1 << 40);
            assert_eq!(range(profile.dart_window), (0, 1 << 32));
        }
    }

    #[test]
    fn t8132_uses_the_adt_window_without_an_offset() {
        assert_eq!(offset(T8132.dart_window), 0);
        assert_eq!(
            range(T8132.dart_window),
            (0x100_0000_4000, 0x100_0000_4000 + 0xffff_0000)
        );
        // The window sits at the BIT(40) alias, inside a 42-bit DMA mask.
        assert!(range(T8132.dart_window).0 >= 1 << 40);
        assert!(range(T8132.dart_window).1 <= 1 << 42);
    }
}
