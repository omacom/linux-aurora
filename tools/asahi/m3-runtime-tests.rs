// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Host tests for the production M3 admission, IO ownership, thermal policy, the T8122 start
//! parameters and the T8122 HwDataA +0x418c block.
//! Run: rustc --edition=2021 --test tools/asahi/m3-runtime-tests.rs -o /tmp/m3-runtime-tests
#![allow(dead_code)]

#[path = "../../drivers/gpu/drm/asahi/m3/t8122_admission.rs"]
mod t8122_admission;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_init_layout.rs"]
mod m3_init_layout;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_init_storage.rs"]
mod m3_init_storage;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_thermal_policy.rs"]
mod m3_thermal_policy;
#[path = "../../drivers/gpu/drm/asahi/m3/t8122_hwdata.rs"]
mod t8122_hwdata;

// `t8122_knobs` takes its default InitData version from `m3_firmware`, which needs the kernel.
// Only that constant is stood in for (`defaults_are_the_plan_baseline` checks its value); the
// parameter parsing and validation are the production module.
mod m3 {
    pub mod m3_firmware {
        pub const G15_V14_8_3_INITDATA: u64 = 0x0c08_e21e_8380_0490;
    }
}
#[path = "../../drivers/gpu/drm/asahi/m3/t8122_knobs.rs"]
mod t8122_knobs;
