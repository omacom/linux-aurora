// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Per-chip T6040 power data supplied by the boot loader.

use super::fw::initdata::{Opp, PerfConfig, PERF_STATES};
use crate::driver::AsahiDrmDevice;
use kernel::{c_str, of, prelude::*, str::CStr};

/// Validated calibration of the running chip. There is no built-in fallback.
pub(super) struct Calibration {
    pub(super) perf: PerfConfig,
    pub(super) leakage: [u32; 56],
    pub(super) max_power_mw: u32,
    core_mask: u32,
    base: usize,
}

fn words<const N: usize>(node: &of::Node, name: &CStr) -> Result<[u32; N]> {
    let values: KVec<u32> = node.get_property(name)?;
    values.as_slice().try_into().map_err(|_| EINVAL)
}

// Raw ADT OPP slots and SGX register slots number 16. The supported firmware consumes
// the 11 populated selectors; the remaining handoff entries must all be zero.
const RAW_STATES: usize = 16;

fn active(values: [u32; RAW_STATES]) -> Result<[u32; PERF_STATES]> {
    if values[PERF_STATES..].iter().any(|&v| v != 0) {
        return Err(EINVAL);
    }
    values[..PERF_STATES].try_into().map_err(|_| EINVAL)
}

fn map(values: [u32; RAW_STATES]) -> Result<[u8; PERF_STATES]> {
    let values = active(values)?;
    if values[0] != 0
        || values[PERF_STATES - 1] != RAW_STATES as u32 - 1
        || values.iter().any(|&v| v >= RAW_STATES as u32)
        || values.windows(2).any(|p| p[0] >= p[1])
    {
        return Err(EINVAL);
    }
    Ok(values.map(|v| v as u8))
}

fn relative(values: &[u32; PERF_STATES]) -> bool {
    values[0] == 0
        && values[PERF_STATES - 1] == 100
        && values.windows(2).all(|p| p[0] <= p[1] && p[1] <= 100)
}

impl Calibration {
    pub(super) fn read(dev: &AsahiDrmDevice) -> Result<Self> {
        let node = dev.as_ref().as_ref().of_node().ok_or(EIO)?;
        if node.get_property::<u32>(c_str!("apple,t6040-handoff-version"))? != 2 {
            return Err(EINVAL);
        }
        let core_mask = node.get_property(c_str!("apple,core-mask"))?;
        let max_power_mw = node.get_property(c_str!("apple,max-power-mw"))?;
        let base = node.get_property::<u32>(c_str!("apple,perf-base-pstate"))? as usize;
        if core_mask == 0
            || core_mask & !0xfffff != 0
            || !(1..=200_000).contains(&max_power_mw)
            || base != 1
        {
            return Err(EINVAL);
        }
        let perf = PerfConfig {
            states_a: map(words(&node, c_str!("apple,perf-map-a"))?)?,
            states_b: map(words(&node, c_str!("apple,perf-map-b"))?)?,
            relative_a: active(words(&node, c_str!("apple,perf-relative-a"))?)?,
            relative_b: active(words(&node, c_str!("apple,perf-relative-b"))?)?,
            sram_min_mv: 810,
        };
        if !relative(&perf.relative_a)
            || !relative(&perf.relative_b)
            || perf.relative_b[..=base].iter().any(|&v| v != 0)
        {
            return Err(EINVAL);
        }
        let core = words::<2>(&node, c_str!("apple,core-leak-coef"))?;
        let sram = words::<2>(&node, c_str!("apple,sram-leak-coef"))?;
        let cs = words::<1>(&node, c_str!("apple,cs-leak-coef"))?;
        let afr = words::<1>(&node, c_str!("apple,afr-leak-coef"))?;
        // Positive finite binary32 values. Integer comparisons preserve their ordering.
        if Iterator::chain(
            Iterator::chain(Iterator::chain(core.iter(), &sram), &cs),
            &afr,
        )
        .any(|&v| !(0x3f80_0000..0x4780_0000).contains(&v))
        {
            return Err(EINVAL);
        }
        let mut leakage = [0; 56];
        leakage[..2].copy_from_slice(&core);
        leakage[32..34].copy_from_slice(&sram);
        leakage[52] = cs[0];
        leakage[54] = afr[0];
        Ok(Self {
            perf,
            leakage,
            max_power_mw,
            core_mask,
            base,
        })
    }

    /// Check the loader's mask against the GPU before starting its firmware.
    pub(super) fn check_cores(&self, masks: &[u32]) -> Result {
        if masks != [self.core_mask & 0x3ff, self.core_mask >> 10] {
            return Err(EINVAL);
        }
        Ok(())
    }

    /// The performance maps index OPPs in ADT order, rather than sorted frequency order.
    pub(super) fn check_opps(&self, opps: &[Opp]) -> Result {
        if opps.len() != 16 || opps[0].freq_hz != 0 || opps.iter().any(|opp| opp.cells != 2) {
            return Err(EINVAL);
        }
        for map in [&self.perf.states_a, &self.perf.states_b] {
            let mut previous = 0;
            for &index in &map[1..] {
                let opp = &opps[usize::from(index)];
                let mhz = opp.freq_hz / 1_000_000;
                if mhz <= previous
                    || opp.volt_uv[..2]
                        .iter()
                        .any(|v| !(400_000..=1_300_000).contains(v))
                {
                    return Err(EINVAL);
                }
                previous = mhz;
            }
        }
        let base = opps[usize::from(self.perf.states_a[self.base])].freq_hz / 1_000_000;
        let top = opps[usize::from(self.perf.states_a[PERF_STATES - 1])].freq_hz / 1_000_000;
        for (i, &index) in self.perf.states_a.iter().enumerate() {
            let freq = opps[usize::from(index)].freq_hz / 1_000_000;
            let expected = if i < self.base {
                0
            } else {
                100 * (freq - base) / (top - base)
            };
            if u64::from(self.perf.relative_b[i]) != expected {
                return Err(EINVAL);
            }
        }
        Ok(())
    }
}

#[kunit_tests(asahi_t6040_calibration)]
mod tests {
    use super::*;

    fn chip() -> Calibration {
        Calibration {
            perf: PerfConfig {
                states_a: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
                states_b: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
                relative_a: [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100],
                relative_b: [0, 0, 11, 22, 33, 44, 55, 66, 77, 88, 100],
                sram_min_mv: 810,
            },
            leakage: [0; 56],
            max_power_mw: 70_000,
            core_mask: 0xfffff,
            base: 1,
        }
    }

    #[test]
    fn operating_points_and_mask_must_match_the_handoff() {
        let chip = chip();
        let mut opps = [Opp {
            freq_hz: 0,
            volt_uv: [800_000; super::super::fw::initdata::OPP_CELLS],
            cells: 2,
        }; 16];
        for (i, opp) in opps.iter_mut().enumerate() {
            opp.freq_hz = i as u64 * 100_000_000;
        }
        assert!(chip.check_opps(&opps).is_ok());
        opps[0].freq_hz = 100_000_000;
        assert!(chip.check_opps(&opps).is_err());
        opps[0].freq_hz = 0;
        assert!(chip.check_cores(&[0x3ff, 0x3ff]).is_ok());
        assert!(chip.check_cores(&[0x3ff, 0x1ff]).is_err());
        assert!(chip.check_cores(&[0xfffff]).is_err());
        opps[5].volt_uv[1] = 0;
        assert!(chip.check_opps(&opps).is_err());
        opps[5].volt_uv[1] = 800_000;
        opps[5].freq_hz = opps[4].freq_hz;
        assert!(chip.check_opps(&opps).is_err());
        opps[5].freq_hz = 501_000_000;
        assert!(chip.check_opps(&opps).is_ok());
        opps[5].freq_hz = 511_000_000;
        assert!(chip.check_opps(&opps).is_err());
    }

    #[test]
    fn maps_refuse_off_repeats_and_out_of_range() {
        let valid = [0, 1, 2, 3, 4, 6, 8, 10, 12, 14, 15, 0, 0, 0, 0, 0];
        assert!(map(valid).is_ok());
        for (index, value) in [
            (0, 1),
            (1, 0),
            (5, 4),
            (10, 16),
            (10, u32::MAX),
            (10, 14),
            (11, 1),
            (15, 15),
        ] {
            let mut bad = valid;
            bad[index] = value;
            assert!(map(bad).is_err());
        }
    }

    #[test]
    fn relative_tables_are_bounded_and_monotonic() {
        let mut values = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100];
        assert!(relative(&values));
        values[5] = 101;
        assert!(!relative(&values));
        values[5] = 39;
        assert!(!relative(&values));
    }
}
