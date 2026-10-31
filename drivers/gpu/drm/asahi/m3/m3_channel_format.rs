// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Bounded firmware channel records. Publication and retirement belong to Config.

use super::m3_init_layout::Error;

pub(crate) const CONTROL_BYTES: usize = 64;
pub(crate) const RECEIVE_BYTES: usize = 72;
pub(crate) const SLOTS: u32 = 256;

#[derive(Clone, Copy, Debug)]
pub(crate) enum Action {
    Initialize,
    Register {
        pipe: u32,
        engine: u32,
    },
    Idle {
        enabled: bool,
    },
    Release {
        group: u8,
        index: u8,
        weight: u8,
        address: u64,
    },
}

impl Action {
    /// The first two actions precede the first generation-bearing publication.
    pub(crate) fn generation(self, published: u32, consumed: u32) -> Result<u32, Error> {
        match self {
            Self::Initialize | Self::Register { .. } => {
                if published != 0 || consumed != 0 {
                    return Err(Error::Address);
                }
                Ok(0)
            }
            _ => published.max(consumed).checked_add(1).ok_or(Error::Address),
        }
    }

    pub(crate) fn encode(self, output: &mut [u8]) -> Result<(), Error> {
        let mut words = [0u32; 3];
        let mut release = None;
        match self {
            Self::Initialize => words[0] = 0x16,
            Self::Register { pipe, engine } => {
                if pipe >= 3 || !(1..=2).contains(&engine) {
                    return Err(Error::Address);
                }
                words = [0x1b, pipe, engine];
            }
            Self::Idle { enabled } => words = [0x0a, enabled.into(), 0],
            Self::Release {
                group,
                index,
                weight,
                address,
            } => {
                if index == 255
                    || weight == 255
                    || address < 0xffff_fc00_0000_0000
                    || address & 7 != 0
                {
                    return Err(Error::Address);
                }
                words[0] = 0x14;
                words[2] = u32::from_le_bytes([group, index, weight, 4]);
                release = Some(address);
            }
        }
        let bytes = output.get_mut(..CONTROL_BYTES).ok_or(Error::Size)?;
        bytes.fill(0);
        for (index, word) in words.iter().enumerate() {
            bytes[index * 4..index * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
        if let Some(address) = release {
            bytes[12..20].copy_from_slice(&address.to_le_bytes());
        }
        Ok(())
    }
}

/// An empty ring is required before publishing a synchronous control transaction.
/// Both consumers must retire it before the next control action may be published.
pub(crate) fn idle_cursor(cursors: [u32; 3]) -> Result<u32, Error> {
    if cursors.iter().any(|value| *value >= SLOTS) {
        return Err(Error::Bounds);
    }
    if cursors[0] != cursors[2] || cursors[1] != cursors[2] {
        return Err(Error::Address);
    }
    Ok(cursors[2])
}

pub(crate) fn payload_offset(thread: usize, slot: u32) -> Result<usize, Error> {
    if thread >= 9 {
        return Err(Error::Bounds);
    }
    Ok((thread * 256 + (slot & 255) as usize) * 0xd8)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initialization_records_have_only_their_defined_words() {
        for action in [Action::Initialize, Action::Register { pipe: 2, engine: 1 }] {
            let mut output = [0xa5; CONTROL_BYTES + 8];
            action.encode(&mut output).unwrap();
            assert_eq!(&output[12..CONTROL_BYTES], &[0; CONTROL_BYTES - 12]);
            assert_eq!(&output[CONTROL_BYTES..], &[0xa5; 8]);
            assert_eq!(action.generation(0, 0), Ok(0));
            assert!(action.generation(1, 0).is_err());
        }
    }

    #[test]
    fn invalid_actions_preserve_the_entire_output() {
        for action in [
            Action::Register { pipe: 3, engine: 1 },
            Action::Register { pipe: 0, engine: 0 },
            Action::Release {
                group: 0,
                index: 255,
                weight: 1,
                address: 0xffff_fc21_0000_0000,
            },
            Action::Release {
                group: 0,
                index: 0,
                weight: 1,
                address: 8,
            },
        ] {
            let mut output = [0xa5; CONTROL_BYTES];
            assert!(action.encode(&mut output).is_err());
            assert_eq!(output, [0xa5; CONTROL_BYTES]);
        }
    }

    #[test]
    fn generations_follow_consumption_and_never_wrap() {
        let action = Action::Idle { enabled: true };
        assert_eq!(action.generation(3, 5), Ok(6));
        assert_eq!(action.generation(5, 3), Ok(6));
        assert!(action.generation(u32::MAX, 0).is_err());
        assert!(action.generation(0, u32::MAX).is_err());
        let mut output = [0; CONTROL_BYTES];
        action.encode(&mut output).unwrap();
        assert_eq!(&output[..8], &[10, 0, 0, 0, 1, 0, 0, 0]);
    }

    #[test]
    fn both_consumers_and_all_indices_are_checked() {
        assert_eq!(idle_cursor([255, 255, 255]), Ok(255));
        for cursors in [[0, 1, 1], [1, 0, 1], [256, 256, 256]] {
            assert!(idle_cursor(cursors).is_err());
        }
        assert_eq!(payload_offset(8, 0x101), Ok((8 * 256 + 1) * 0xd8));
        assert!(payload_offset(9, 0).is_err());
    }

    #[test]
    fn release_uses_the_packed_owned_pointer() {
        let action = Action::Release {
            group: 1,
            index: 2,
            weight: 3,
            address: 0xffff_fc21_0400_0000,
        };
        let mut output = [0; CONTROL_BYTES];
        action.encode(&mut output).unwrap();
        assert_eq!(&output[8..12], &[1, 2, 3, 4]);
        assert_eq!(
            u64::from_le_bytes(output[12..20].try_into().unwrap()),
            0xffff_fc21_0400_0000
        );
    }
}
