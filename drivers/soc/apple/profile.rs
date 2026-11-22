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
//! These bring-up targets are modelled:
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
//! * `T6040` / J616s (MacBook Pro 16", M4 Pro): iBoot boots sepOS, so the
//!   driver uses the warm registration path; the sensor is the J700 part.
//! * `T6050` / J714s, J716c (M5 Pro/Max): T6040's SEP and sensor, with
//!   the sensor's bus at its own address.

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
    /// Reserved declaration; the current selector does not implement this policy.
    /// Wiring it up on an enrolled install would change its anti-replay lane.
    HostPersisted,
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
    // Off: Touch ID setup on a cold-boot SEP rewrites a session record that
    // predates Linux, so the guard would only block it.
    xart_guard: 0,
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
    // As T8103. On J316s, creating the identity keybag also rewrites a
    // session record that predates Linux.
    xart_guard: 0,
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

/// MacBook Pro 16" M4 Pro (J616s). The J616s ADT (26.6.2, 25G83) records a
/// pre-booted SEP (`sepfw-load-at-boot`), an `iop-sep,ascwrap-v6` SEP whose
/// DART aperture starts at the BIT(40) alias (`vm-base` 0x10000004000), and
/// the Mesa sensor J700 carries (`sensor-id` 0x3356) on spi2 at
/// `0x429108000`, 8 MHz, chip select 0. Its power is NUB GPIO 53 and its
/// data-ready line AP GPIO 116. `/defaults` carries `cpx-encryption-mode = 2`.
///
/// A warm-registration platform like T8140, with T8140's authenticated
/// sensor session: the enclave serves this sensor only after the nonce and
/// the signed root key. Device-tree nodes remain authoritative for addresses.
pub(crate) const T6040: PlatformProfile = PlatformProfile {
    name: "T6040/J616s",
    sensor: SensorProfile {
        controller_base: 0x4_2910_8000,
        ..T8140.sensor
    },
    // macOS shares this enclave and Linux has no records here yet: keep
    // Linux to its own xART records from the first boot.
    xart_guard: 1,
    ..T8140
};

/// MacBook Pro 14" and 16" M4 Max (J614c, J616c). Their ADTs describe the SEP
/// and the sensor as J616s's does: on the J614c (26.6.2) an `iop-sep,ascwrap-v6`
/// SEP with `sepfw-booted`, and the Mesa sensor (`sensor-id` 0x3356) on spi2 at
/// `0x429108000`, 8 MHz, powered by NUB GPIO 53 with data-ready on AP GPIO 116.
pub(crate) const T6041: PlatformProfile = PlatformProfile {
    name: "T6041/J614c, J616c",
    ..T6040
};

/// MacBook Pro 14" M5 Pro (J714s). The J714sAP ADT (26A428) records a
/// pre-booted SEP (`sepfw-booted`), an `iop-sep,ascwrap-v6` SEP whose DART
/// aperture starts at the BIT(40) alias (`vm-base-0` 0x10000004000,
/// `vm-size-0` 0xffff0000), and the Mesa sensor J700 carries (`sensor-id`
/// 0x3356) on spi2 at `0x505108000`, 8 MHz, chip select 0. That is J616s's
/// SEP, so T6040's profile with this sensor bus. J716c has the same pre-booted
/// ASC, DART aperture, sensor revision, bus address and SPI frequency.
pub(crate) const T6050: PlatformProfile = PlatformProfile {
    name: "T6050/J714s, K116s, J716c",
    sensor: SensorProfile {
        controller_base: 0x5_0510_8000,
        ..T6040.sensor
    },
    ..T6040
};

/// MacBook Air M4 (J713). The J713 ADT (26.6.2, 25G83) puts the Mesa sensor
/// on spi2 at `0x3ad108000`, 8 MHz, chip select 0, with power on the NUB GPIO
/// controller (pin 28) and data-ready on AP GPIO 199. The SEP is
/// `iop-sep,ascwrap-v6` behind a DART with the BIT(40) input alias, so the SoC
/// side is J613's.
///
/// The sensor is J700's generation (`V3`, 0x3356), not J613's (`R3`): the
/// enclave refuses the R3 SESSION_SHARE (status 0x16) and its factory
/// calibration is an FSC2 image. So it takes J700's SPI mode 2 and nonce
/// session; its device tree names its signed root key. J713 on OS firmware
/// 26.6.2 provisions its keybag, enrols, matches and restores the enrolment
/// after a reboot. The J715 ADT describes the same sensor, wiring and SEP.
pub(crate) const T8132: PlatformProfile = PlatformProfile {
    name: "T8132/J713/J715",
    sensor: SensorProfile {
        controller_base: 0x3_ad10_8000,
        mode: SpiMode::Mode2,
        expected_id: 0x3356,
        ..T8122.sensor
    },
    nonce_session: true,
    ..T8122
};

/// MacBook Air M5 (J813 and J815). Their ADTs (macOS 27.0 stub firmware) put
/// the Mesa sensor (`V3`, the J700/J713/J714s generation) on spi2 at
/// `0x3a5108000`, 8 MHz, chip select 0, with power on the NUB GPIO controller
/// (pin 36) and data-ready on AP GPIO 200. The SEP is a pre-booted
/// `iop-sep,ascwrap-v6` (`sepfw-booted`) behind a t8110 DART whose input
/// aperture is the BIT(40) alias (`vm-base-0` 0x10000004000, `vm-size-0`
/// 0xffff0000): the same SoC side as T8132 and T6050 (J714s, the M5 Pro).
/// So this is T8132's profile with the sensor bus moved. J813 was tested
/// with explicit xart_guard=1; the inherited guard policy is unchanged.
pub(crate) const T8142: PlatformProfile = PlatformProfile {
    name: "T8142/J813, J815",
    sensor: SensorProfile {
        controller_base: 0x3_a510_8000,
        ..T8132.sensor
    },
    ..T8132
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
static_assert!(T6040.sensor.controller_base == 0x429108000);
static_assert!(T6040.sensor.expected_id == T8140.sensor.expected_id);
static_assert!(T8132.sensor.controller_base == 0x3ad108000);
static_assert!(T8132.sensor.expected_id == T8140.sensor.expected_id);
static_assert!(T8132.nonce_session && T8132.wide_dma_mask);
static_assert!(T8142.sensor.controller_base == 0x3a5108000);
static_assert!(T8142.sensor.expected_id == T8132.sensor.expected_id);
static_assert!(T8142.nonce_session && T8142.wide_dma_mask);

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
    select(&machine_has, &t6020_profile)
}

/// The profile for a machine whose root `compatible` list `has` answers for,
/// with `t6020` choosing the M2 Pro and Max key-store dialect.
fn select(
    has: &dyn Fn(&[u8]) -> bool,
    t6020: &dyn Fn() -> &'static PlatformProfile,
) -> Result<&'static PlatformProfile> {
    if has(b"apple,t8103") {
        return Ok(&T8103);
    }
    if has(b"apple,t6020") {
        return Ok(t6020());
    }
    // t6020.dtsi is defined as a cut-down t6021: it includes t6021.dtsi and
    // disables the parts the smaller die lacks. Both pull in t602x-die0.dtsi,
    // so the SEP, its DART and the SPI controller sit at the same addresses
    // with the same interrupts on either part, and the T6020 constants apply
    // to the M2 Max unchanged.
    if has(b"apple,t6021") {
        return Ok(t6020());
    }
    if has(b"apple,t6000") {
        return Ok(&T6000);
    }
    // t6000.dtsi is defined as a cut-down t6001: it includes t6001.dtsi and
    // deletes the parts the smaller die lacks. Both pull in t600x-die0.dtsi,
    // so the SEP, its mailbox, its DART and the SPI controller sit at the same
    // addresses with the same interrupts on either part, and the T6000
    // constants apply to the M1 Max unchanged. The Mac Studio (j375c) is also
    // a t6001 but leaves the SEP disabled, so nothing binds there; apple,t6002
    // (M1 Ultra, no built-in sensor) stays unmapped.
    if has(b"apple,t6001") {
        return Ok(&T6000);
    }
    if has(b"apple,t8140") {
        return Ok(&T8140);
    }
    if has(b"apple,t8112") {
        return Ok(&T8112);
    }
    // Only the boards whose ADTs were read: the other T8122 Macs (J433, J434,
    // J504) share the SoC but their sensor wiring is unchecked.
    if has(b"apple,t8122") && (has(b"apple,j613") || has(b"apple,j615")) {
        return Ok(&T8122);
    }
    // The T6040 profile is tested on J614s and J616s.
    if has(b"apple,t6040") && (has(b"apple,j616s") || has(b"apple,j614s")) {
        return Ok(&T6040);
    }
    if has(b"apple,t6041") && (has(b"apple,j616c") || has(b"apple,j614c")) {
        return Ok(&T6041);
    }
    if has(b"apple,t6050")
        && (has(b"apple,j714s") || has(b"apple,k116s") || has(b"apple,j716c"))
    {
        return Ok(&T6050);
    }
    // Only the M4 MacBook Airs: the other T8132 Macs are unread.
    if has(b"apple,t8132") && (has(b"apple,j713") || has(b"apple,j715")) {
        return Ok(&T8132);
    }
    // The J815 and the J813, whose ADTs were read: both have the sensor (mesa) on spi2
    // at 0x3a5108000.
    if has(b"apple,t8142") && (has(b"apple,j815") || has(b"apple,j813")) {
        return Ok(&T8142);
    }
    Err(ENODEV)
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[::kernel::macros::kunit_tests(apple_sep_profile)]
mod tests {
    use super::*;

    #[test]
    fn t8142_sensor_boards_require_the_matching_soc() {
        assert_eq!(T8142.xart_guard, 1);
        let cases: [(&[u8], &[u8], bool); 8] = [
            (b"apple,t8142", b"apple,j813", true),
            (b"apple,t8142", b"apple,j815", true),
            (b"apple,t8142", b"apple,j814", false),
            (b"apple,t8142", b"apple,j704", false),
            (b"apple,t8142", b"", false),
            (b"", b"apple,j813", false),
            (b"apple,t8132", b"apple,j813", false),
            (b"apple,t6050", b"apple,j815", false),
        ];
        for (soc, board, admitted) in cases {
            let has = |c: &[u8]| c == soc || c == board;
            let selected = select(&has, &|| &T6020);
            if admitted {
                assert!(matches!(selected, Ok(profile) if profile.name == T8142.name));
            } else {
                assert!(selected.is_err());
            }
        }
    }

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
    fn neo_keeps_its_existing_chosen_identity_policy() {
        // A host-persisted default would strand existing chosen-lane enrolments.
        assert!(T8140.identity == IdentitySource::Chosen);
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
        assert!(T6040.nonce_session);
        assert!(T6040.wide_dma_mask);
        assert!(T6040.bootstrap == Bootstrap::WarmRegister);
        assert!(T6040.identity == IdentitySource::Chosen);
        assert!(T6040.xart_guard == 1);
    }

    /// The name of the profile `select` picks for a root `compatible` list.
    fn picked(compatibles: &[&[u8]]) -> Option<&'static str> {
        let has = |c: &[u8]| compatibles.iter().any(|x| *x == c);
        select(&has, &|| &T6020).ok().map(|p| p.name)
    }

    #[test]
    fn t8132_selection_is_limited_to_the_m4_airs() {
        assert_eq!(picked(&[b"apple,j713", b"apple,t8132"]), Some(T8132.name));
        assert_eq!(picked(&[b"apple,j715", b"apple,t8132"]), Some(T8132.name));
        assert_eq!(picked(&[b"apple,j604", b"apple,t8132"]), None);
        assert_eq!(picked(&[b"apple,t8132"]), None);
        assert_eq!(picked(&[b"apple,j613", b"apple,t8132"]), None);
        assert_eq!(picked(&[b"apple,j713", b"apple,t8122"]), None);
        assert_eq!(picked(&[b"apple,j613", b"apple,t8122"]), Some(T8122.name));
        assert_eq!(picked(&[b"apple,j700", b"apple,t8140"]), Some(T8140.name));
        assert_eq!(picked(&[b"apple,j616s", b"apple,t6040"]), Some(T6040.name));
        assert_eq!(picked(&[b"apple,t6040"]), None);
        assert_eq!(picked(&[b"apple,j614s", b"apple,t6040"]), Some(T6040.name));
        assert_eq!(picked(&[b"apple,j616c", b"apple,t6041"]), Some(T6041.name));
        assert_eq!(picked(&[b"apple,j614c", b"apple,t6041"]), Some(T6041.name));
        assert_eq!(picked(&[b"apple,t6041"]), None);
        assert_eq!(picked(&[b"apple,j713", b"apple,t6041"]), None);
        assert_eq!(picked(&[b"apple,j714s", b"apple,t6050"]), Some(T6050.name));
        assert_eq!(picked(&[b"apple,k116s", b"apple,t6050"]), Some(T6050.name));
        assert_eq!(picked(&[b"apple,j716c", b"apple,t6050"]), Some(T6050.name));
        assert_eq!(picked(&[b"apple,j716c", b"apple,t6041"]), None);
        assert_eq!(picked(&[b"apple,t6050"]), None);
    }

    /// Touch ID has one path for every Mac. The retry limits, the lock
    /// states, the password re-arm and the lift-gated capture read nothing
    /// from the profile; only the transport underneath (bus, nonce session,
    /// key-store dialect) does. What the shared path needs is a sensor whose
    /// status it decodes, so every Mac with a built-in sensor must select a
    /// profile naming one of the known Mesa revisions.
    #[test]
    fn every_touch_id_mac_gets_a_sensor_the_shared_path_decodes() {
        let macs: [&[&[u8]]; 18] = [
            &[b"apple,j313", b"apple,t8103"],
            &[b"apple,j316s", b"apple,t6000"],
            &[b"apple,j316c", b"apple,t6001"],
            &[b"apple,j414s", b"apple,t6020"],
            &[b"apple,j414c", b"apple,t6021"],
            &[b"apple,j413", b"apple,t8112"],
            &[b"apple,j415", b"apple,t8112"],
            &[b"apple,j613", b"apple,t8122"],
            &[b"apple,j615", b"apple,t8122"],
            &[b"apple,j614s", b"apple,t6040"],
            &[b"apple,j616s", b"apple,t6040"],
            &[b"apple,j616c", b"apple,t6041"],
            &[b"apple,j713", b"apple,t8132"],
            &[b"apple,j700", b"apple,t8140"],
            &[b"apple,j714s", b"apple,t6050"],
            &[b"apple,j716c", b"apple,t6050"],
            &[b"apple,j813", b"apple,t8142"],
            &[b"apple,j815", b"apple,t8142"],
        ];
        for compatibles in macs {
            let has = |c: &[u8]| compatibles.iter().any(|x| *x == c);
            let selected = select(&has, &|| &T6020);
            assert!(selected.is_ok());
            if let Ok(profile) = selected {
                assert!(crate::sensor::KNOWN_IDENTIFIERS.contains(&profile.sensor.expected_id));
            }
        }
        assert!(crate::sensor::KNOWN_IDENTIFIERS.contains(&T6020_SEPOS13.sensor.expected_id));
    }

    #[test]
    fn j713_takes_the_v3_sensor_on_the_j613_soc_side() {
        assert!(T8132.sensor.expected_id == T8140.sensor.expected_id);
        assert!(T8132.sensor.mode == SpiMode::Mode2);
        assert!(T8132.nonce_session && T8132.wide_dma_mask && T8132.persistent_enrol);
        assert!(T8132.shmem_capacity == T8122.shmem_capacity);
        assert!(T8132.xart_guard == T8122.xart_guard);
        assert!(core::ptr::eq(T8132.dart_window, T8122.dart_window));
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
        for profile in [
            &T8103,
            &T6000,
            &T6020,
            &T8140,
            &T6020_SEPOS13,
            &T8122,
            &T6040,
            &T6041,
            &T6050,
            &T8132,
            &T8142,
        ] {
            assert_eq!(offset(profile.dart_window), 1 << 40);
            assert_eq!(range(profile.dart_window), (0, 1 << 32));
        }
    }
}

/// The owner-qualified J613 OS-firmware lane; system firmware may differ.
pub(crate) fn j613_26_6_2() -> bool {
    machine_has(b"apple,j613")
        && chosen_string(c"asahi,os-fw-version")
            .as_deref()
            .is_some_and(os_firmware_26_6_2)
}

fn os_firmware_26_6_2(raw: &[u8]) -> bool {
    raw == b"26.6.2" || raw == b"26.6.2\0"
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[kernel::macros::kunit_tests(apple_sep_j613_firmware)]
mod j613_firmware_tests {
    use super::*;

    #[test]
    fn exact_os_firmware_only() {
        assert!(os_firmware_26_6_2(b"26.6.2"));
        assert!(os_firmware_26_6_2(b"26.6.2\0"));
        for raw in [&b"14.8.3"[..], &b"26.6.20"[..], &b"27.0"[..], &b""[..]] {
            assert!(!os_firmware_26_6_2(raw));
        }
    }
}
