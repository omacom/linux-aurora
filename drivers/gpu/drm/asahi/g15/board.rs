// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Board-data templates of the supported firmware build.
//!
//! Sparse 16-byte-line images (HardwareData 35,332 B, Globals 15,868 B, Power 2,292 B), each
//! with its SHA-256: [`expand`] expands and verifies them, [`from_adt`] rebuilds DVFS tables A/B
//! from the ADT performance states (`DVFS_A` {0,1,2,3,5,7,9,11,13}, `DVFS_B`
//! {0,1,2,3,4,6,8,10,12}; MHz at +0xb5c, +0xf9c, +0x1350, core mV +0xb9c, SRAM mV +0xd9c, top
//! MHz +0x4dc4). The tables in [`data`] are generated, and every expanded image is checked
//! against its hash at boot.

pub(crate) mod blob;
pub(crate) mod data;

use kernel::{device, prelude::*};

use super::adt::GpuAdt;
use super::boot::glue;

/// One template line: 16 bytes at `offset`.
pub(crate) struct Line {
    pub(crate) offset: u32,
    pub(crate) data: [u8; 16],
}

/// A sparse template.
pub(crate) struct Template {
    pub(crate) size: usize,
    pub(crate) lines: &'static [Line],
    pub(crate) sha256: [u8; 32],
}

/// Expands a template into `out` and verifies its hash.
pub(crate) fn expand(template: &Template, out: &mut [u8]) -> Result {
    let out = out.get_mut(..template.size).ok_or(EINVAL)?;
    out.fill(0);
    for line in template.lines {
        let offset = line.offset as usize;
        if offset >= template.size {
            return Err(EINVAL);
        }
        let n = 16.min(template.size - offset);
        out[offset..offset + n].copy_from_slice(&line.data[..n]);
    }
    if glue::sha256(out) != template.sha256 {
        return Err(EINVAL);
    }
    Ok(())
}

/// DVFS states of the firmware tables.
pub(crate) const DVFS_STATES: usize = 9;
const DVFS_A: [usize; DVFS_STATES] = [0, 1, 2, 3, 5, 7, 9, 11, 13];
const DVFS_B: [usize; DVFS_STATES] = [0, 1, 2, 3, 4, 6, 8, 10, 12];

fn put32(hw: &mut [u8], offset: usize, value: u32) -> Result {
    hw.get_mut(offset..offset + 4)
        .ok_or(EINVAL)?
        .copy_from_slice(&value.to_le_bytes());
    Ok(())
}

/// The machine-specific part of HardwareData, rebuilt from this machine's ADT performance
/// states over the template: the firmware's two DVFS tables take the three shared low states,
/// then per voltage level the higher (table A) and the lower (table B) frequency of the ADT
/// pair. Table A carries the core and SRAM voltages (eight identical lanes per state) and its top
/// frequency. With the reference machine's ADT this reproduces the template exactly. Returns the
/// top frequency in kHz.
pub(crate) fn from_adt(dev: &device::Device, adt: &GpuAdt, hw: &mut [u8]) -> Result<u32> {
    for s in 1..DVFS_STATES {
        let (a, pa) = (DVFS_A[s], DVFS_A[s - 1]);
        let (b, pb) = (DVFS_B[s], DVFS_B[s - 1]);
        if adt.perf_mhz[a] <= adt.perf_mhz[pa]
            || adt.perf_mhz[b] <= adt.perf_mhz[pb]
            || adt.perf_mv[a] < adt.perf_mv[pa]
            || adt.sram_mv[a] < adt.sram_mv[pa]
        {
            dev_err!(dev, "ADT perf-states do not form ordered DVFS tables\n");
            return Err(EINVAL);
        }
    }
    for s in 0..DVFS_STATES {
        let (a, b) = (DVFS_A[s], DVFS_B[s]);
        put32(hw, 0xb5c + 4 * s, adt.perf_mhz[a])?;
        put32(hw, 0xf9c + 4 * s, adt.perf_mhz[b])?;
        put32(hw, 0x1350 + 4 * s, adt.perf_mhz[b])?;
        for k in 0..8 {
            put32(hw, 0xb9c + 0x20 * s + 4 * k, adt.perf_mv[a])?;
            put32(hw, 0xd9c + 0x20 * s + 4 * k, adt.sram_mv[a])?;
        }
    }
    let top = adt.perf_mhz[DVFS_A[DVFS_STATES - 1]];
    put32(hw, 0x4dc4, top)?;
    Ok(top * 1000)
}
