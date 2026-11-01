// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Host tests of production firmware identity and normalization.
#![allow(dead_code)]
#[path = "../../drivers/gpu/drm/asahi/m3/m3_profile.rs"]
pub(crate) mod m3_profile;
#[path = "../../drivers/gpu/drm/asahi/m3/agx_resources.rs"]
pub(crate) mod resources;
mod m3 {
    pub(crate) use crate::firmware as m3_firmware;
    pub(crate) use crate::m3_profile;
    pub(crate) use crate::resources as m3_resources;
}
#[path = "../../drivers/gpu/drm/asahi/m3/m3_firmware.rs"]
pub(crate) mod firmware;
