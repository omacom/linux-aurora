// SPDX-License-Identifier: GPL-2.0-only OR MIT


#[cfg(not(test))]
use crate::identity::{GpuGen, GpuVariant};
#[cfg(test)]
#[path = "../identity.rs"]
mod identity;
#[cfg(test)]
use identity::{GpuGen, GpuVariant};

/// Identification-only configuration for one AGX3 SoC.
#[derive(Debug)]
pub(crate) struct SocConfig {
    /// Chip ID in hex format (e.g. 0x6050 for t6050).
    pub(crate) chip_id: u32,
    /// Decoded design generation for this chip.
    pub(crate) gpu_gen: GpuGen,
    /// Decoded variant for this chip.
    pub(crate) gpu_variant: GpuVariant,
    /// Hardware family byte expected at `0xd04000[31:24]`.
    pub(crate) hw_family: u8,
    /// Hardware variant byte expected at `0xd04000[23:16]`.
    pub(crate) hw_variant: u8,
    /// Die count expected at `0xd04010[19:16]`.
    pub(crate) num_dies: u32,
    /// Input-address bits per UAT root (42 on every AGX3 part).
    pub(crate) uat_ias: u8,
    /// Output address space (42-bit on every AGX3 part).
    pub(crate) uat_oas: u32,
}

const fn agx3(
    chip_id: u32,
    gpu_gen: GpuGen,
    gpu_variant: GpuVariant,
    hw_family: u8,
    hw_variant: u8,
    num_dies: u32,
) -> SocConfig {
    SocConfig {
        chip_id,
        gpu_gen,
        gpu_variant,
        hw_family,
        hw_variant,
        num_dies,
        uat_ias: 42,
        uat_oas: 42,
    }
}

pub(crate) const T8122: SocConfig = agx3(0x8122, GpuGen::G15, GpuVariant::G, 0x7, 2, 1);
/// Apple M3 Ultra (G15D, two dies).
pub(crate) const T6032: SocConfig = agx3(0x6032, GpuGen::G15, GpuVariant::D, 0x7, 4, 2);
