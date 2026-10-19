// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The firmware idle policy.
//!
//! The words and their rationale are in [`idle`]. Initdata writes them into the Globals image
//! before the firmware first sees it:
//!
//! - +0x7a8 standby = `g15_idle_standby_us` (2500, clamped 100..100000);
//! - +0x7c4 minimum confidence = `g15_idle_min_conf` (500 permille as f32);
//! - +0x30 predictive = 0 ([`super::hw::tuning::IDLE_PREDICTIVE`]): the finite delay, the GPU
//!   powers down after the +0x9a0 delay (template 2, about 2 ms);
//! - +0x2c idle decision enabled (template 1, unchanged).
//!
//! DVFS is firmware-run on the ADT tables.

mod idle;
pub(crate) use idle::permille_f32;

use kernel::prelude::*;

use super::fw::initdata::globals;

static_assert!(idle::ENABLE == globals::IDLE_ENABLE);
static_assert!(idle::PREDICTIVE == globals::IDLE_PREDICTIVE);
static_assert!(idle::STANDBY_US == globals::IDLE_STANDBY_US);
static_assert!(idle::MIN_CONF == globals::IDLE_MIN_CONF);
static_assert!(idle::DELAY == globals::IDLE_DELAY);
