// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj
//! Machine ref-key: Secure-Enclave key sealing, signing, and attestation.
//!
//! A ref-key (op `0x22`) is an EC keypair the enclave keeps; the private half
//! never leaves the SEP. The host-side ECIES encrypt lives in [`refkey_seal`];
//! every half that needs the private key runs in the enclave.

use crate::{image, keybag, profile, proto, refkey_seal, shim};
use crate::{LockState, MachineRefKey, SepData, SksRequest};
use kernel::prelude::*;
use kernel::sync::atomic::Relaxed;

pub(crate) const REFKEY_ENVELOPE_VERSION: u32 = 2;

impl SepData {
    fn sks_req_refkey(&self, keybag: i32, der_set: &[u8]) -> Result<SksRequest> {
        let (keybag, envelope) = match self.profile.key_store {
            // The 13.5 operation envelope is 3. Use the designated identity
            // session rather than the source handle; class F with -1 expired
            // at the next boot on J313.
            profile::KeyStore::Sepos13 { .. } => {
                let user = crate::sks::DesignateUser::new(crate::SBIO_PROBE_USER_ID)
                    .ok_or(EINVAL)?;
                (user.special_handle().value(), 3)
            }
            profile::KeyStore::Variant5 => (keybag, REFKEY_ENVELOPE_VERSION),
        };
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(keybag)?;
        body.put_u32(envelope)?;
        body.put_blob(der_set)?;
        let img = image::build_request(image::Version::V1, self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg =
            crate::sks::encode_sks_perform_operation(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_PERFORM_OP_NAME,
            msg,
            img,
        })
    }

    fn sks_prepare_identity_refkey(
        &self,
        handle: crate::sks::KeyBagHandle,
        secret: &[u8],
    ) -> bool {
        if !self.keybag_designated.load(Relaxed) {
            self.sks_designate_user_keybag(handle, secret);
        }
        if !self.keybag_designated.load(Relaxed) {
            dev_warn!(self.dev, "sks: ref-key identity designation failed\n");
            return false;
        }
        let Some(user) = crate::sks::DesignateUser::new(crate::SBIO_PROBE_USER_ID) else {
            return false;
        };
        let Some(healthy) = self.sks_health_check(c"ref-key identity unlock") else {
            return false;
        };
        let Some(out) = self.sks_send(self.sks_req_unlock_special(
            user.special_handle(),
            secret,
            healthy,
        )) else {
            dev_warn!(self.dev, "sks: ref-key identity unlock got no reply\n");
            return false;
        };
        dev_info!(
            self.dev,
            "sks: ref-key identity unlock mailbox status {}\n",
            out.reply.status
        );
        if out.reply.status != 0 {
            return false;
        }
        let Some(body) = self.sks_report_response(c"DEVICE_STATE_TRANSITION", &out) else {
            return false;
        };
        body.len() == 20 && body[..4] == [0; 4]
    }

    fn sks_create_refkey(
        &self,
        handle: crate::sks::KeyBagHandle,
        secret: &[u8],
    ) -> Option<KVec<u8>> {
        use crate::der::RefKeyValue as RV;

        let (protection_class, key_type) = match self.profile.key_store {
            // J313 accepted this non-class-F class and 13.5 EC key type for
            // creation, same-boot unseal, and post-reboot unseal. The exact
            // Apple policy name for class 9 is not established here.
            profile::KeyStore::Sepos13 { .. } => (9, 4),
            profile::KeyStore::Variant5 => (9, 5),
        };

        if self.profile.key_store == profile::KeyStore::Variant5 {
            // On variant 5 the ref-key is bound to the unlocked identity bag.
            if let Some(healthy) = self.sks_health_check(c"ref-key create") {
                let _ = self.sks_send(self.sks_req_change_lock_state(
                    handle,
                    LockState::Unlocked,
                    secret,
                    healthy,
                ));
            }
        } else if !self.sks_prepare_identity_refkey(handle, secret) {
            return None;
        }

        let create_der = {
            let mut items: KVec<(&[u8], RV<'_>)> = KVec::new();
            if items.push((b"o", RV::Utf8(b"oc")), GFP_KERNEL).is_err()
                || items
                    .push((b"bc", RV::Integer(protection_class)), GFP_KERNEL)
                    .is_err()
                || items
                    .push((b"kt", RV::Integer(key_type)), GFP_KERNEL)
                    .is_err()
            {
                return None;
            }
            crate::der::encode_refkey_set(&items).ok()?
        };
        let cout = match self.sks_send(self.sks_req_refkey(handle.value(), &create_der)) {
            Some(out) => out,
            None => {
                dev_warn!(self.dev, "sks: ref-key create got no usable reply\n");
                return None;
            }
        };
        dev_info!(
            self.dev,
            "sks: ref-key create mailbox status {}, response {} bytes\n",
            cout.reply.status,
            cout.response.len()
        );
        if cout.reply.status != 0 {
            return None;
        }
        let cbody = self.sks_report_response(crate::sks::SKS_PERFORM_OP_NAME, &cout)?;
        let mut cf = proto::FieldCursor::new(cbody);
        let (Some(version), Some(cblob)) = (cf.i32(), cf.blob()) else {
            dev_warn!(self.dev, "sks: ref-key create reply was malformed\n");
            return None;
        };
        if version != 0 {
            dev_warn!(self.dev, "sks: ref-key create reply version {}\n", version);
            return None;
        }
        let mut refkey_blob: KVec<u8> = KVec::new();
        refkey_blob.extend_from_slice(cblob, GFP_KERNEL).ok()?;
        Some(refkey_blob)
    }

    fn refkey_pub(blob: &[u8]) -> Option<&[u8]> {
        let rk_tlv = crate::der::refkey_find(blob, b"rk")?;
        let pub_raw =
            crate::der::refkey_find(rk_tlv, b"pub").and_then(crate::der::octet_string_body)?;
        (pub_raw.len() == refkey_seal::POINT_LEN).then_some(pub_raw)
    }

    pub(crate) fn sks_machine_refkey(&self, handle: crate::sks::KeyBagHandle, secret: &[u8]) {
        const MACHINE_REFKEY_PATH: &CStr = c"/var/lib/aurora-sep-refkey.bin";
        const MACHINE_REFKEY_V2_PATH: &CStr = c"/var/lib/aurora-sep-refkey-v2.bin";
        let path = if *crate::module_parameters::refkey_v2.value() != 0 {
            MACHINE_REFKEY_V2_PATH
        } else {
            MACHINE_REFKEY_PATH
        };

        if self.machine_refkey.lock().is_some() {
            return;
        }

        let existing = match shim::StoreFile::open_readonly(path) {
            Ok(file) => Some(file),
            Err(ENOENT) => None,
            Err(error) => {
                dev_err!(
                    self.dev,
                    "sks: cannot read existing machine ref-key: {:?}; refusing replacement\n",
                    error
                );
                return;
            }
        };
        if let Some(f) = existing {
            let sz = f.size().unwrap_or(0);
            if (1..=8192).contains(&sz) {
                let mut blob: KVec<u8> = KVec::new();
                if blob.resize(sz as usize, 0u8, GFP_KERNEL).is_ok()
                    && f.read_exact(0, &mut blob).is_ok()
                    && self.cache_machine_refkey(blob)
                {
                    return;
                }
            }
            dev_err!(
                self.dev,
                "sks: existing machine ref-key is unreadable or invalid; refusing replacement\n"
            );
            return;
        }

        let Some(blob) = self.sks_create_refkey(handle, secret) else {
            dev_warn!(
                self.dev,
                "sks: could not create the machine ref-key; trusted-key sealing is unavailable this boot.\n"
            );
            return;
        };
        let Ok(f) = shim::StoreFile::open(path) else {
            dev_err!(self.dev, "sks: could not persist machine ref-key\n");
            return;
        };
        if f.write_all(0, &blob).is_err() || f.sync().is_err() {
            dev_err!(self.dev, "sks: machine ref-key persistence failed\n");
            return;
        }
        let _ = self.cache_machine_refkey(blob);
    }

    fn cache_machine_refkey(&self, blob: KVec<u8>) -> bool {
        let Some(pubv) = Self::refkey_pub(&blob) else {
            return false;
        };
        let mut pub_raw: KVec<u8> = KVec::new();
        if pub_raw.extend_from_slice(pubv, GFP_KERNEL).is_err() {
            return false;
        }
        *self.machine_refkey.lock() = Some(MachineRefKey { blob, pub_raw });
        true
    }

    fn ensure_machine_refkey(&self) -> Result<()> {
        if self.machine_refkey.lock().is_some() {
            return Ok(());
        }
        if !self.sks_ready() {
            return Err(ENODEV);
        }
        let keybag::State::Present(stored) = keybag::read(keybag::Slot::Identity)? else {
            return Err(ENODEV);
        };
        let (handle, _uuid) = self.sks_recover(&stored).ok_or(EIO)?;
        self.sks_machine_refkey(handle, stored.secret());
        let _ = self.sks_send(self.sks_req_unload_keybag(handle));
        if self.machine_refkey.lock().is_some() {
            Ok(())
        } else {
            Err(EIO)
        }
    }

    pub(crate) fn refkey_seal_trusted(&self, key: &[u8]) -> Result<KVec<u8>> {
        self.wait_for_trusted_ready()?;
        self.ensure_machine_refkey()?;
        let sealed = {
            let guard = self.machine_refkey.lock();
            let mk = guard.as_ref().ok_or(ENODEV)?;
            refkey_seal::ecies_seal(&mk.pub_raw, key)?
        };

        // The cached public key can still encrypt after the identity bag
        // stops admitting its private ref-key. Do not return a new trusted
        // blob unless this bag can recover it. Release the cache lock before
        // unsealing, which acquires that lock again.
        let recovered = crate::Secret(self.refkey_unseal_trusted(&sealed)?);
        if &*recovered != key {
            dev_err!(self.dev, "trusted-keys: refusing a new blob whose private-key round trip failed\n");
            return Err(EIO);
        }
        Ok(sealed)
    }

    pub(crate) fn refkey_unseal_trusted(&self, sealed: &[u8]) -> Result<KVec<u8>> {
        self.wait_for_trusted_ready()?;
        self.ensure_machine_refkey()?;
        let blob = {
            let guard = self.machine_refkey.lock();
            let mk = guard.as_ref().ok_or(ENODEV)?;
            let mut b: KVec<u8> = KVec::new();
            b.extend_from_slice(&mk.blob, GFP_KERNEL)?;
            b
        };
        let keybag::State::Present(stored) = keybag::read(keybag::Slot::Identity)? else {
            return Err(ENODEV);
        };
        let (handle, _uuid) = self.sks_recover(&stored).ok_or(EIO)?;
        if self.profile.key_store == profile::KeyStore::Variant5 {
            if let Some(healthy) = self.sks_health_check(c"trusted-key unseal") {
                let _ = self.sks_send(self.sks_req_change_lock_state(
                    handle,
                    LockState::Unlocked,
                    stored.secret(),
                    healthy,
                ));
            }
        } else if !self.sks_prepare_identity_refkey(handle, stored.secret()) {
            let _ = self.sks_send(self.sks_req_unload_keybag(handle));
            return Err(EIO);
        }
        let recovered = self.sks_refkey_unseal(handle, &blob, sealed);
        let _ = self.sks_send(self.sks_req_unload_keybag(handle));
        recovered.ok_or(EIO)
    }

    /// Op `osgn`: ECDSA-P256 over `challenge` as the pre-computed digest.
    fn sks_refkey_sign(
        &self,
        handle: crate::sks::KeyBagHandle,
        refkey_blob: &[u8],
        challenge: &[u8],
    ) -> Option<KVec<u8>> {
        use crate::der::RefKeyValue as RV;
        let mut items: KVec<(&[u8], RV<'_>)> = KVec::new();
        items.push((b"o", RV::Utf8(b"osgn")), GFP_KERNEL).ok()?;
        items.push((b"d", RV::Octets(challenge)), GFP_KERNEL).ok()?;
        items.push((b"rk", RV::Der(refkey_blob)), GFP_KERNEL).ok()?;
        let der = crate::der::encode_refkey_set(&items).ok()?;
        let out = self.sks_send(self.sks_req_refkey(handle.value(), &der))?;
        if out.reply.status != 0 {
            dev_warn!(
                self.dev,
                "sks: ref-key attest sign failed (status {})\n",
                out.reply.status
            );
            return None;
        }
        let body = self.sks_report_response(crate::sks::SKS_PERFORM_OP_NAME, &out)?;
        let mut f = proto::FieldCursor::new(body);
        let (Some(0), Some(sig)) = (f.i32(), f.blob()) else {
            return None;
        };
        // Enclave wraps the sig in OCTET STRING { SEQUENCE { r, s } }; strip to inner DER.
        let der_sig = crate::der::octet_string_body(sig).unwrap_or(sig);
        let mut owned: KVec<u8> = KVec::new();
        owned.extend_from_slice(der_sig, GFP_KERNEL).ok()?;
        Some(owned)
    }

    /// Attestation of key possession by signing proof; op `oa` (attestation-chained)
    /// needs the SEP device attestation key, absent on a Linux-attached SEP.
    pub(crate) fn refkey_attest_sign(&self, challenge: &[u8]) -> Result<(KVec<u8>, KVec<u8>)> {
        self.ensure_machine_refkey()?;
        let (blob, pubk) = {
            let guard = self.machine_refkey.lock();
            let mk = guard.as_ref().ok_or(ENODEV)?;
            let mut b: KVec<u8> = KVec::new();
            b.extend_from_slice(&mk.blob, GFP_KERNEL)?;
            let mut p: KVec<u8> = KVec::new();
            p.extend_from_slice(&mk.pub_raw, GFP_KERNEL)?;
            (b, p)
        };
        let keybag::State::Present(stored) = keybag::read(keybag::Slot::Identity)? else {
            return Err(ENODEV);
        };
        let (handle, _uuid) = self.sks_recover(&stored).ok_or(EIO)?;
        if let Some(healthy) = self.sks_health_check(c"ref-key attest") {
            let _ = self.sks_send(self.sks_req_change_lock_state(
                handle,
                LockState::Unlocked,
                stored.secret(),
                healthy,
            ));
        }
        let sig = self.sks_refkey_sign(handle, &blob, challenge);
        let _ = self.sks_send(self.sks_req_unload_keybag(handle));
        Ok((sig.ok_or(EIO)?, pubk))
    }

    /// Enclave ECIES-decrypt (o = "oecd"); takes the ephemeral from the front of `d`.
    fn sks_refkey_unseal(
        &self,
        handle: crate::sks::KeyBagHandle,
        refkey_blob: &[u8],
        sealed: &[u8],
    ) -> Option<KVec<u8>> {
        use crate::der::RefKeyValue as RV;
        let mut items: KVec<(&[u8], RV<'_>)> = KVec::new();
        items.push((b"o", RV::Utf8(b"oecd")), GFP_KERNEL).ok()?;
        items.push((b"d", RV::Octets(sealed)), GFP_KERNEL).ok()?;
        items.push((b"rk", RV::Der(refkey_blob)), GFP_KERNEL).ok()?;
        let der = crate::der::encode_refkey_set(&items).ok()?;
        let out = self.sks_send(self.sks_req_refkey(handle.value(), &der))?;
        if out.reply.status != 0 {
            dev_warn!(
                self.dev,
                "trusted-keys: ref-key unseal failed (status {})\n",
                out.reply.status
            );
            return None;
        }
        let body = self.sks_report_response(crate::sks::SKS_PERFORM_OP_NAME, &out)?;
        let mut f = proto::FieldCursor::new(body);
        match (f.i32(), f.blob()) {
            (Some(0), Some(rec)) => {
                let mut owned = KVec::new();
                owned.extend_from_slice(rec, GFP_KERNEL).ok()?;
                Some(owned)
            }
            _ => None,
        }
    }
}
