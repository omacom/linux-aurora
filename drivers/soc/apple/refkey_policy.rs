// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Access control for application-owned biometric signing keys.

use crate::der::{encode_refkey_set, RefKeyValue as V};
use kernel::prelude::*;

/// Bind private-key use to this biometric credential set. Enrollment changes
/// within the set are permitted; this is not a current-enrollment-set policy.
pub(crate) fn biometric_signing_policy(binding: &[u8; 16]) -> Result<KVec<u8>> {
    let biometric = biometric_constraint(binding)?;
    let acl = encode_refkey_set(&[
        (b"oa", V::Boolean(true)),
        (b"ock", V::Der(&biometric)),
        (b"okd", V::Der(&biometric)),
        (b"odel", V::Boolean(true)),
        (b"osgn", V::Der(&biometric)),
    ])?;
    encode_refkey_set(&[(b"acl", V::Der(&acl))])
}

pub(crate) fn biometric_combined_policy(binding: &[u8; 16]) -> Result<KVec<u8>> {
    let biometric = biometric_constraint(binding)?;
    let acl = encode_refkey_set(&[
        (b"oa", V::Boolean(true)),
        (b"ock", V::Der(&biometric)),
        (b"okd", V::Der(&biometric)),
        (b"osgn", V::Der(&biometric)),
        (b"od", V::Der(&biometric)),
        (b"oecd", V::Der(&biometric)),
        (b"oe", V::Boolean(true)),
        (b"odel", V::Boolean(true)),
    ])?;
    encode_refkey_set(&[(b"acl", V::Der(&acl))])
}

fn biometric_constraint(binding: &[u8; 16]) -> Result<KVec<u8>> {
    if binding.iter().all(|v| *v == 0) { return Err(EINVAL); }
    let credential = encode_refkey_set(&[(b"pbioc", V::Octets(binding))])?;
    encode_refkey_set(&[(b"cbio", V::Der(&credential))])
}

#[kernel::macros::kunit_tests(apple_sep_refkey_policy)]
mod tests {
    use super::*;

    #[test]
    fn member_order_uses_encoded_length_before_key() -> Result {
        let encoded = encode_refkey_set(&[
            (b"a", V::Octets(&[0; 16])),
            (b"z", V::Boolean(true)),
        ])?;
        assert_eq!(&encoded[..10], &[0x31, 31, 0x30, 6, 0x0c, 1, b'z', 1, 1, 1]);
        Ok(())
    }

    #[test]
    fn policy_matches_native_encoding() -> Result {
        let binding = core::array::from_fn(|i| i as u8);
        let expected: &[u8] = &[
            0x31, 0x81, 0xaa, 0x30, 0x81, 0xa7, 0x0c, 0x03, 0x61, 0x63, 0x6c, 0x31,
            0x81, 0x9f, 0x30, 0x07, 0x0c, 0x02, 0x6f, 0x61, 0x01, 0x01, 0x01, 0x30,
            0x09, 0x0c, 0x04, 0x6f, 0x64, 0x65, 0x6c, 0x01, 0x01, 0x01, 0x30, 0x2c,
            0x0c, 0x03, 0x6f, 0x63, 0x6b, 0x31, 0x25, 0x30, 0x23, 0x0c, 0x04, 0x63,
            0x62, 0x69, 0x6f, 0x31, 0x1b, 0x30, 0x19, 0x0c, 0x05, 0x70, 0x62, 0x69,
            0x6f, 0x63, 0x04, 0x10, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
            0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x30, 0x2c, 0x0c, 0x03,
            0x6f, 0x6b, 0x64, 0x31, 0x25, 0x30, 0x23, 0x0c, 0x04, 0x63, 0x62, 0x69,
            0x6f, 0x31, 0x1b, 0x30, 0x19, 0x0c, 0x05, 0x70, 0x62, 0x69, 0x6f, 0x63,
            0x04, 0x10, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
            0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x30, 0x2d, 0x0c, 0x04, 0x6f, 0x73,
            0x67, 0x6e, 0x31, 0x25, 0x30, 0x23, 0x0c, 0x04, 0x63, 0x62, 0x69, 0x6f,
            0x31, 0x1b, 0x30, 0x19, 0x0c, 0x05, 0x70, 0x62, 0x69, 0x6f, 0x63, 0x04,
            0x10, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,
            0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
        ];
        assert_eq!(biometric_signing_policy(&binding)?.as_slice(), expected);
        Ok(())
    }

    #[test]
    fn policy_requires_a_credential_set() {
        assert!(biometric_signing_policy(&[0; 16]).is_err());
    }
}
