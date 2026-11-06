// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Host tests for the production M3 admission, IO ownership, thermal policy and the T8122
//! HwDataA +0x418c block.
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

#[path = "../../drivers/gpu/drm/asahi/m3/m3_init_26.rs"]
mod m3_init_26;

#[path = "../../drivers/gpu/drm/asahi/m3/m3_profile.rs"]
mod m3_profile;

#[path = "../../drivers/gpu/drm/asahi/m3/m3_mmio_26.rs"]
mod m3_mmio_26;

#[path = "../../drivers/gpu/drm/asahi/m3/m3_queue_layout.rs"]
mod m3_queue_layout;

#[path = "../../drivers/gpu/drm/asahi/m3/m3_pool_layout.rs"]
mod m3_pool_layout;

#[path = "../../drivers/gpu/drm/asahi/m3/m3_compute_layout.rs"]
mod m3_compute_layout;

#[path = "../../drivers/gpu/drm/asahi/m3/m3_channel_format.rs"]
mod m3_channel_format;
