// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Firmware-interface properties, independent of the physical G15 GPU record format.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Profile {
    Legacy,
    Extended,
}

pub(crate) const PARAMS_REQUIRED: u64 = 600;

impl Profile {
    pub(crate) const fn firmware_compat(self) -> [u32; 3] {
        match self {
            Self::Legacy => [14, 8, 3],
            Self::Extended => [26, 6, 2],
        }
    }

    pub(crate) const fn hal(self) -> u32 {
        match self {
            Self::Legacy => 0,
            Self::Extended => 200,
        }
    }

    pub(crate) const fn user_end(self) -> u64 {
        match self {
            Self::Legacy => 0x2ff_ffff_8000,
            Self::Extended => (1 << 42) - 2 * 0x4000,
        }
    }

    pub(crate) fn gpu_duration_ns(self, elapsed: u64) -> Option<u64> {
        match self {
            Self::Legacy => elapsed.checked_mul(1000).map(|value| value / 24),
            Self::Extended => Some(elapsed),
        }
    }

    pub(crate) const fn vm_acknowledged(self, params_bytes: u64, flags: u32) -> bool {
        // Both profiles consume G15 records; the existing VM padding stays zero.
        params_bytes >= PARAMS_REQUIRED && flags == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profile_acknowledgement_is_exact_and_file_query_covers_hal() {
        for profile in [Profile::Legacy, Profile::Extended] {
            let expected = 0;
            for size in [0, 599, 600, 616, u64::MAX] {
                for flags in [0, 1, 2, 3, u32::MAX] {
                    assert_eq!(
                        profile.vm_acknowledged(size, flags),
                        size >= 600 && flags == expected
                    );
                }
            }
        }
    }

    #[test]
    fn firmware_interface_version_does_not_change_physical_identity() {
        assert_eq!(Profile::Legacy.hal(), 0);
        assert_eq!(Profile::Extended.hal(), 200);
        assert_eq!(Profile::Legacy.firmware_compat(), [14, 8, 3]);
        assert_eq!(Profile::Extended.firmware_compat(), [26, 6, 2]);
        assert_eq!(Profile::Extended.user_end(), 0x3ff_ffff_8000);
        assert_eq!(Profile::Legacy.user_end(), 0x2ff_ffff_8000);
    }
    #[test]
    fn native_nanoseconds_are_never_rescaled_as_legacy_ticks() {
        assert_eq!(
            Profile::Extended.gpu_duration_ns(24_000_000),
            Some(24_000_000)
        );
        assert_eq!(
            Profile::Legacy.gpu_duration_ns(24_000_000),
            Some(1_000_000_000)
        );
        assert_eq!(Profile::Extended.gpu_duration_ns(u64::MAX), Some(u64::MAX));
        assert_eq!(Profile::Legacy.gpu_duration_ns(u64::MAX), None);
    }
}
