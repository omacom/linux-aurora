// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Read-only T8122 identity and firmware-interface admission.

use super::m3_profile::Profile;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(crate) struct Identity {
    pub(crate) active_cores: u32,
    pub(crate) core_slots: u32,
}

/// Admit only the bounded stage-2 result and G15G revision observed on M3 Air.
/// The ten-bit field describes core *slots*, not ten active cores: J613 has eight.
pub(crate) fn validate_identity(
    firmware_compat: &[u32],
    schema: u32,
    result: u32,
    id_version: u32,
    masks: [u32; 3],
) -> Option<Identity> {
    validate_identity_for(Profile::Legacy, firmware_compat, schema, result, id_version, masks)
}

pub(crate) fn validate_identity_for(
    profile: Profile,
    firmware_compat: &[u32],
    schema: u32,
    result: u32,
    id_version: u32,
    masks: [u32; 3],
) -> Option<Identity> {
    let family = id_version >> 24;
    let variant = (id_version >> 16) & 0xff;
    let revision = (id_version >> 8) & 0xff;
    let active_cores = masks[0].count_ones();
    if firmware_compat != profile.firmware_compat()
        || schema != 1
        || result != 1
        || family != 7
        || variant != 2
        || revision != 0x20
        || masks[0] & !0x3ff != 0
        || masks[1] != 0
        || masks[2] != 0
        || (active_cores != 8 && active_cores != 10)
    {
        return None;
    }
    Some(Identity {
        active_cores,
        core_slots: 10,
    })
}

/// Firmware-interface selection is exact and does not change physical GPU encodings.
pub(crate) fn profile_for(compat:&[u32],j613:bool)->Option<Profile> {
    match compat {
        [14,8,3]=>Some(Profile::Legacy),
        [26,6,2] if j613=>Some(Profile::Extended),
        _=>None,
    }
}

#[cfg(not(test))]
pub(crate) fn loaded_profile(pdev:&kernel::platform::Device<kernel::device::Core<'_>>)
    ->kernel::error::Result<Profile> {
    use kernel::{c_str,prelude::*};
    let node=pdev.as_ref().of_node().ok_or(ENODEV)?;
    let compat:KVec<u32>=node.get_property(c_str!("apple,firmware-compat"))?;
    #[cfg(CONFIG_DRM_ASAHI_M3_T8122)]
    let j613 = super::m3_board::board_is(b"apple,j613");
    #[cfg(not(CONFIG_DRM_ASAHI_M3_T8122))]
    let j613 = false;
    profile_for(&compat, j613).ok_or(ENODEV)
}

/// Shape of the voltage-sorted running-state tables: one entry per distinct millivolt,
/// and the highest primary frequency in MHz. Voltages round up, matching `PwrConfig`.
/// The first point is the off state. Both primary and secondary frequencies must increase
/// with voltage, and at most fifteen running entries fit alongside the off entry.
pub(crate) fn opp_table_shape(opps: impl IntoIterator<Item = (u64, u32)>) -> Option<(u32, u32)> {
    let mut opps = opps.into_iter();
    if opps.next()?.0 != 0 {
        return None;
    }
    let mut volts = [0u32; 15];
    let mut highs = [0u64; 15];
    let mut lows = [u64::MAX; 15];
    let mut count = 0;
    let mut points = 1;
    for (hz, microvolt) in opps {
        points += 1;
        let mv = microvolt.div_ceil(1000);
        if points > u8::MAX as usize || hz < 1_000_000 || hz > u64::from(u32::MAX) || mv == 0 {
            return None;
        }
        let i = match volts[..count].iter().position(|v| *v == mv) {
            Some(i) => i,
            None => {
                *volts.get_mut(count)? = mv;
                count += 1;
                count - 1
            }
        };
        highs[i] = highs[i].max(hz);
        lows[i] = lows[i].min(hz);
    }
    if count == 0 {
        return None;
    }
    let mut last_mv = 0;
    let mut last_high = 0;
    let mut last_low = 0;
    for _ in 0..count {
        let i = (0..count).filter(|i| volts[*i] > last_mv).min_by_key(|i| volts[*i])?;
        if highs[i] <= last_high || lows[i] <= last_low {
            return None;
        }
        last_mv = volts[i];
        last_high = highs[i];
        last_low = lows[i];
    }
    Some((count as u32, (last_high / 1_000_000) as u32))
}

/// Read the m1n1 observation and firmware ABI before any GPU MMIO or DMA setup.
/// This is intentionally only an identity gate; it does not admit a full HwConfig.
#[cfg(not(test))]
pub(crate) fn read_identity(
    pdev: &kernel::platform::Device<kernel::device::Core<'_>>,
) -> kernel::error::Result<Identity> {
    read_identity_for(pdev, loaded_profile(pdev)?)
}

#[cfg(not(test))]
pub(crate) fn read_identity_for(
    pdev: &kernel::platform::Device<kernel::device::Core<'_>>,
    profile: Profile,
) -> kernel::error::Result<Identity> {
    use kernel::{bindings, c_str, prelude::*};

    struct NodeRef(*mut bindings::device_node);
    impl Drop for NodeRef {
        fn drop(&mut self) {
            // Without CONFIG_OF_DYNAMIC, of_node_put() is an empty inline, as in
            // kernel::of::Node.
            #[cfg(CONFIG_OF_DYNAMIC)]
            // SAFETY: `of_find_node_opts_by_path` returned this owned reference.
            unsafe {
                bindings::of_node_put(self.0)
            };
        }
    }
    impl NodeRef {
        fn get_u32(&self, name: &CStr) -> Result<u32> {
            let mut length = 0;
            // SAFETY: `self` holds the node reference; the property pointer is
            // read only while that reference remains live.
            let value = unsafe {
                bindings::of_get_property(self.0, name.as_char_ptr(), &mut length)
            };
            if value.is_null() || length != 4 {
                return Err(ENODEV);
            }
            // SAFETY: the property is exactly four bytes, checked above.
            let bytes = unsafe { core::slice::from_raw_parts(value as *const u8, 4) };
            Ok(u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
        }
    }

    let gpu = pdev.as_ref().of_node().ok_or(ENODEV)?;
    let firmware_compat: KVec<u32> = gpu.get_property(c_str!("apple,firmware-compat"))?;
    let path = c_str!("/chosen/asahi,t8122-gpu-powered-identity");
    // SAFETY: `path` is a static NUL-terminated string. A non-null result is
    // an owned OF reference released by `NodeRef`.
    let raw = unsafe {
        bindings::of_find_node_opts_by_path(path.as_char_ptr(), core::ptr::null_mut())
    };
    if raw.is_null() {
        return Err(ENODEV);
    }
    let observation = NodeRef(raw);
    let schema = observation.get_u32(c_str!("schema-version"))?;
    let result = observation.get_u32(c_str!("probe-result"))?;
    let id_version = observation.get_u32(c_str!("id-version"))?;
    let masks = [
        observation.get_u32(c_str!("core-mask-0"))?,
        observation.get_u32(c_str!("core-mask-1"))?,
        observation.get_u32(c_str!("core-mask-2"))?,
    ];
    validate_identity_for(profile, firmware_compat.as_slice(), schema, result, id_version, masks)
        .ok_or(ENODEV)
}

#[cfg(test)]
mod tests {
    use super::*;

    const G15G_C0: u32 = 0x0702_2000;

    #[test]
    fn accepts_eight_and_ten_active_cores_without_conflating_slots() {
        for (mask, count) in [(0x0ff, 8), (0x3ff, 10)] {
            let id = validate_identity(&[14, 8, 3], 1, 1, G15G_C0, [mask, 0, 0]).unwrap();
            assert_eq!(id.active_cores, count);
            assert_eq!(id.core_slots, 10);
        }
    }

    #[test]
    fn extended_identity_requires_its_actual_version_and_the_same_g15g_bounds() {
        for mask in [0xff, 0x3ff] {
            let good = || validate_identity_for(Profile::Extended, &[26, 6, 2], 1, 1, G15G_C0, [mask, 0, 0]);
            assert_eq!(good().unwrap().active_cores, mask.count_ones());
            assert!(validate_identity(&[26, 6, 2], 1, 1, G15G_C0, [mask, 0, 0]).is_none());
            assert!(validate_identity_for(Profile::Extended, &[14, 8, 3], 1, 1, G15G_C0, [mask, 0, 0]).is_none());
        }
        for (schema, result, id, masks) in [
            (0, 1, G15G_C0, [0xff, 0, 0]),
            (1, 0, G15G_C0, [0xff, 0, 0]),
            (1, 1, G15G_C0 ^ (1 << 24), [0xff, 0, 0]),
            (1, 1, G15G_C0 ^ (1 << 16), [0xff, 0, 0]),
            (1, 1, G15G_C0 ^ (1 << 8), [0xff, 0, 0]),
            (1, 1, G15G_C0, [0x1ff, 0, 0]),
            (1, 1, G15G_C0, [0x7ff, 0, 0]),
            (1, 1, G15G_C0, [0xff, 1, 0]),
            (1, 1, G15G_C0, [0xff, 0, 1]),
        ] {
            assert!(validate_identity_for(Profile::Extended, &[26, 6, 2], schema, result, id, masks).is_none());
        }
    }

    /// The booted J613 ladder (`perf-states`, frequency then millivolts), in ADT order: fourteen
    /// states, eight voltages above the idle one, 1338 MHz at the top.
    const J613_PERF_STATES: [(u64, u32); 14] = [
        (0, 125),
        (338_000_000, 630),
        (618_000_000, 690),
        (796_000_000, 720),
        (836_000_000, 760),
        (928_000_000, 760),
        (952_000_000, 810),
        (1_056_000_000, 810),
        (1_053_000_000, 870),
        (1_170_000_000, 870),
        (1_152_000_000, 910),
        (1_278_000_000, 910),
        (1_204_000_000, 935),
        (1_338_000_000, 935),
    ];

    #[test]
    fn j613_operating_points_give_eight_states_up_to_1338_mhz() {
        // The boot loader turns millivolts into opp-microvolt by multiplying by 1000.
        let opps = J613_PERF_STATES.map(|(hz, mv)| (hz, mv * 1000));
        assert_eq!(opp_table_shape(opps), Some((8, 1338)));
    }

    #[test]
    fn operating_point_shapes_round_like_the_driver_and_bound_the_table() {
        // Voltages are rounded up to millivolts: 810000 and 810001 uV are two states.
        assert_eq!(
            opp_table_shape([(0, 0), (1_000_000, 810_000), (2_000_000, 810_001)]),
            Some((2, 2))
        );
        assert_eq!(
            opp_table_shape([(0, 0), (1_000_000, 810_000), (2_000_000, 809_001)]),
            Some((1, 2))
        );
        // No running state.
        assert_eq!(opp_table_shape([(0, 125_000)]), None);
        // Fifteen voltages fit the firmware table, sixteen do not.
        let ladder = |n: u32| core::iter::once((0, 0))
            .chain((1..=n).map(|i| (u64::from(i) * 1_000_000, 500_000 + i * 10_000)));
        assert_eq!(opp_table_shape(ladder(15)), Some((15, 15)));
        assert_eq!(opp_table_shape(ladder(16)), None);
    }

    #[test]
    fn operating_point_shapes_reject_invalid_order_voltage_and_frequency() {
        for opps in [
            vec![],
            vec![(1_000_000, 500_000)],
            vec![(0, 0), (0, 500_000)],
            vec![(0, 0), (1_000_000, 0)],
            vec![(0, 0), (999_999, 500_000)],
            vec![(0, 0), (u64::from(u32::MAX) + 1, 500_000)],
            vec![(0, 0), (2_000_000, 500_000), (1_000_000, 600_000)],
            vec![(0, 0), (2_000_000, 500_000), (3_000_000, 500_000),
                (1_000_000, 600_000), (4_000_000, 600_000)],
        ] {
            assert_eq!(opp_table_shape(opps), None);
        }
    }

    #[test]
    fn operating_point_shapes_sort_voltages_and_preserve_voltage_extremes() {
        assert_eq!(opp_table_shape([
            (0, 0), (4_000_000, 600_000), (1_000_000, 500_000),
            (3_000_000, 600_000), (2_000_000, 500_000),
        ]), Some((2, 4)));
        assert_eq!(opp_table_shape([
            (0, 0), (u64::from(u32::MAX), u32::MAX),
        ]), Some((1, 4294)));
        let repeated = core::iter::once((0, 0))
            .chain(core::iter::repeat_n((1_000_000, 500_000), 255));
        assert_eq!(opp_table_shape(repeated), None);
    }

    #[test]
    fn rejects_failed_probes_wrong_revision_and_out_of_range_masks() {
        let valid = [0x0ff, 0, 0];
        assert!(validate_identity(&[14, 7, 0], 1, 1, G15G_C0, valid).is_none());
        assert!(validate_identity(&[14, 8, 3], 2, 1, G15G_C0, valid).is_none());
        assert!(validate_identity(&[14, 8, 3], 1, 5, G15G_C0, valid).is_none());
        assert!(validate_identity(&[14, 8, 3], 1, 1, 0x0703_2000, valid).is_none());
        assert!(validate_identity(&[14, 8, 3], 1, 1, 0x0702_1000, valid).is_none());
        assert!(validate_identity(&[14, 8, 3], 1, 1, G15G_C0, [0x4ff, 0, 0]).is_none());
        assert!(validate_identity(&[14, 8, 3], 1, 1, G15G_C0, [0xff, 1, 0]).is_none());
        assert!(validate_identity(&[14, 8, 3], 1, 1, G15G_C0, [0xff, 0, 1]).is_none());
        assert!(validate_identity(&[14, 8, 3], 1, 1, G15G_C0, [0x7f, 0, 0]).is_none());
    }
}

#[cfg(test)]
mod selection_tests {
    use super::*;
    #[test] fn extended_admission_requires_exact_firmware_and_j613() {
        for board in [false,true] {
            assert_eq!(profile_for(&[14,8,3],board),Some(Profile::Legacy));
            assert_eq!(profile_for(&[26,6,2],board),if board{Some(Profile::Extended)}else{None});
            for wrong in [&[][..],&[26,6][..],&[26,6,1][..],&[26,6,2,0][..],&[14,8,4][..]] {
                assert_eq!(profile_for(wrong,board),None);
            }
        }
    }
}
