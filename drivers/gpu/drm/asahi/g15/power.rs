// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The firmware idle policy.
//!
//! The words and their rationale are in [`idle`]. Initdata writes them into the Globals image
//! before the firmware first sees it:
//!
//! - +0x7a8 standby = `g15_idle_standby_us` when set (clamped 100..100000), else the GPU node's
//!   `apple,idleoff-standby-timer`, else the tuned 2500 ([`idle::standby`]);
//! - +0x7c4 minimum confidence = `g15_idle_min_conf` (500 permille as f32);
//! - +0x30 predictive = 0 ([`super::hw::tuning::IDLE_PREDICTIVE`]): the finite delay, the GPU
//!   powers down after the +0x9a0 delay (template 2, about 2 ms);
//! - +0x2c idle decision enabled (template 1, unchanged).
//!
//! DVFS is firmware-run on the ADT tables. The state it runs at is read back by
//! `aurora-g15/pstate` ([`pstate`], [`super::Firmware::write_pstate`]): the newest power-state
//! record of the stats ring, and HardwareDataA's state words.

mod idle;
pub(crate) mod pstate;
pub(crate) use idle::{permille_f32, standby};

use kernel::{fmt, prelude::*};

use super::board;
use super::fw::initdata::{globals, hwdata};
use super::hw::rings;
use super::Firmware;

static_assert!(idle::ENABLE == globals::IDLE_ENABLE);
static_assert!(idle::PREDICTIVE == globals::IDLE_PREDICTIVE);
static_assert!(idle::STANDBY_US == globals::IDLE_STANDBY_US);
static_assert!(idle::MIN_CONF == globals::IDLE_MIN_CONF);
static_assert!(idle::DELAY == globals::IDLE_DELAY);

static_assert!(pstate::STATES == board::DVFS_STATES);
static_assert!(pstate::STATS_RECORD * pstate::STATS_SLOTS as usize == rings::STATS_RING_SIZE);
static_assert!(pstate::STATS_SLOTS == super::fw::channels::RING_SLOTS);

impl Firmware {
    /// HardwareDataA's state words, as the firmware last wrote them.
    pub(crate) fn pstate_words(&self) -> Result<pstate::HwWords> {
        let hw = self.init.hwdata.as_ref().ok_or(ENODEV)?;
        let word = |offset: usize| hw.r32(hwdata::HWDATA_A + offset);
        Ok(pstate::HwWords {
            actual: word(pstate::HWDATA_A_ACTUAL)?,
            target: word(pstate::HWDATA_A_TARGET)?,
            current: word(pstate::HWDATA_A_CURRENT)?,
            freq_mhz: word(pstate::HWDATA_A_FREQ_MHZ)?,
        })
    }

    /// The `aurora-g15/pstate` file ([`pstate::write`]): the stats ring's records so far, those
    /// it holds unconsumed (read, not consumed), HardwareDataA's words and table A of the
    /// device tree's operating points. Read-only: nothing firmware-visible is written.
    pub(crate) fn write_pstate(&self, f: &mut impl fmt::Write) -> fmt::Result {
        let Ok(hw) = self.pstate_words() else {
            return Ok(());
        };
        let seen = self
            .channels
            .peek_stats()
            .unwrap_or(self.channels.stats_seen);
        let table = pstate::Table::from_mhz(board::table_a_mhz(&self.session.adt));
        pstate::write(f, &seen, &hw, &table)
    }
}
