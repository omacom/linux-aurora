// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Application-owned biometric P-256 keys. Authentication contexts stay in kernel.

use crate::{bio, der, keybag, profile, proto, sbio, scrd, SepData};
use crate::der::RefKeyValue as V;
use kernel::prelude::*;
use kernel::sync::atomic::Relaxed;

pub(crate) const MAX_BLOB: usize = 4096;

/// A request owns a fresh credential. Discard the grant even on cancellation or
/// transport failure; a failed deletion disables application keys until reboot.
pub(crate) struct Credential<'a> {
    sep: &'a SepData,
    handle: [u8; scrd::SCRD_ACM_HANDLE_LEN],
}

impl<'a> Credential<'a> {
    pub(crate) fn new(sep: &'a SepData, handle: [u8; scrd::SCRD_ACM_HANDLE_LEN]) -> Self {
        Self { sep, handle }
    }

    pub(crate) fn handle(&self) -> &[u8; scrd::SCRD_ACM_HANDLE_LEN] {
        &self.handle
    }

    pub(crate) fn into_handle(self) -> [u8; scrd::SCRD_ACM_HANDLE_LEN] {
        let handle = self.handle;
        core::mem::forget(self);
        handle
    }
}

impl Drop for Credential<'_> {
    fn drop(&mut self) {
        if !matches!(self.sep.scrd_command(&scrd::scrd_context_delete(&self.handle)), Some((0, _))) {
            self.sep.key_api_failed.store(true, Relaxed);
            dev_err!(self.sep.dev, "keys: credential disposal failed; application key operations disabled until reboot\n");
        }
        self.handle.fill(0);
    }
}

impl SepData {
    pub(crate) fn biometric_keys_supported(&self) -> bool {
        // The credential-match layout and policy are validated on this protocol
        // generation only. Other profiles retain their existing capabilities.
        matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. })
            && !self.key_api_failed.load(Relaxed)
    }

    pub(crate) fn key_credential(&self) -> Result<Credential<'_>> {
        if !self.biometric_keys_supported() {
            return Err(Error::from_errno(-(kernel::bindings::EOPNOTSUPP as i32)));
        }
        let keybag::State::Present(stored) = keybag::read(keybag::Slot::Identity)? else {
            return Err(ENOENT);
        };
        let (handle, _) = self.sks_recover(&stored).ok_or(EIO)?;
        let result = (|| {
            if !self.sks_prepare_identity_refkey(handle, stored.secret()) {
                return Err(EIO);
            }
            let user = sbio::UserId::new(crate::SBIO_PROBE_USER_ID).ok_or(EINVAL)?;
            let context = self.establish_passcode_validated_context(user).ok_or_else(|| {
                self.key_api_failed.store(true, Relaxed);
                EIO
            })?;
            Ok(Credential::new(self, context))
        })();
        let unloaded = self.sks_send(self.sks_req_unload_keybag(handle));
        if !matches!(unloaded, Some(ref out) if out.reply.status == 0) {
            self.key_api_failed.store(true, Relaxed);
            return Err(EIO);
        }
        result
    }

    fn application_refkey_op(&self, items: &[(&[u8], V<'_>)]) -> Result<KVec<u8>> {
        let encoded = der::encode_refkey_set(items)?;
        let out = self.sks_send(self.sks_req_refkey(0, &encoded)).ok_or(EIO)?;
        if out.reply.status == -8 {
            return Err(EACCES);
        }
        if out.reply.status != 0 {
            return Err(EIO);
        }
        let body = self.sks_report_response(crate::sks::SKS_PERFORM_OP_NAME, &out).ok_or(EIO)?;
        let mut fields = proto::FieldCursor::new(body);
        match fields.i32().ok_or(EIO)? {
            0 => {},
            -8 => return Err(EACCES),
            _ => return Err(EIO),
        }
        let blob = fields.blob().ok_or(EIO)?;
        if blob.len() > MAX_BLOB {
            return Err(EIO);
        }
        let mut owned = KVec::new();
        owned.extend_from_slice(blob, GFP_KERNEL)?;
        Ok(owned)
    }

    pub(crate) fn create_biometric_key(&self, policy_kind: u32) -> Result<(KVec<u8>, [u8; bio::ATTEST_PUB_LEN])> {
        let credential = self.key_credential()?;
        let result = (|| {
            let user = sbio::UserId::new(crate::SBIO_PROBE_USER_ID).ok_or(EINVAL)?;
            let crate::SbioOutcome::Ok(binding) = self.sbio_call(&sbio::sbio_catacomb_id(user)) else {
                return Err(EIO);
            };
            let binding: &[u8; 16] = binding.as_slice().try_into().map_err(|_| EIO)?;
            let policy = if policy_kind == 1 {
                crate::refkey_policy::biometric_combined_policy(binding)?
            } else {
                crate::refkey_policy::biometric_signing_policy(binding)?
            };
            let items = [
                (b"o".as_slice(), V::Utf8(b"oc")),
                (b"bc".as_slice(), V::Integer(9)),
                (b"kt".as_slice(), V::Integer(4)),
                (b"ed".as_slice(), V::Der(&policy)),
                (b"acmh".as_slice(), V::Octets(credential.handle())),
            ];
            let blob = self.application_refkey_op(&items)?;
            let mut public = [0; bio::ATTEST_PUB_LEN];
            public.copy_from_slice(Self::refkey_pub(&blob).ok_or(EIO)?);
            Ok((blob, public))
        })();
        drop(credential);
        if self.key_api_failed.load(Relaxed) { return Err(EIO); }
        result
    }

    pub(crate) fn sign_biometric_key(
        &self,
        blob: &[u8],
        digest: &[u8; 32],
        credential: &Credential<'_>,
    ) -> Result<Signature> {
        let encoded = self.application_refkey_op(&[
            (b"o", V::Utf8(b"osgn")),
            (b"d", V::Octets(digest)),
            (b"rk", V::Der(blob)),
            (b"acmh", V::Octets(credential.handle())),
        ])?;
        let bytes = der::octet_string_body(&encoded).unwrap_or(&encoded);
        if bytes.len() > bio::ATTEST_SIG_MAX || bytes.first() != Some(&0x30) {
            return Err(EIO);
        }
        let mut signature = Signature { len: bytes.len() as u32, bytes: [0; bio::ATTEST_SIG_MAX] };
        signature.bytes[..bytes.len()].copy_from_slice(bytes);
        Ok(signature)
    }

    fn decrypt_biometric_key(
        &self,
        blob: &[u8],
        sealed: &[u8],
        credential: &Credential<'_>,
    ) -> Result<crate::Secret> {
        let mut items = KVec::new();
        items.push((b"o".as_slice(), V::Utf8(b"oecd")), GFP_KERNEL)?;
        items.push((b"d".as_slice(), V::Octets(sealed)), GFP_KERNEL)?;
        items.push((b"rk".as_slice(), V::Der(blob)), GFP_KERNEL)?;
        items.push((b"acmh".as_slice(), V::Octets(credential.handle())), GFP_KERNEL)?;
        self.application_refkey_op(&items).map(crate::Secret)
    }
}

pub(crate) struct Signature {
    pub(crate) len: u32,
    pub(crate) bytes: [u8; bio::ATTEST_SIG_MAX],
}

pub(crate) struct KeyRequest {
    pub(crate) blob: KVec<u8>,
    pub(crate) input: Input,
    pub(crate) deadline_ns: u64,
}

pub(crate) enum Input {
    Create(u32),
    Sign([u8; 32]),
    Decrypt(KVec<u8>),
}

pub(crate) enum Output {
    Created(KVec<u8>, [u8; bio::ATTEST_PUB_LEN]),
    Signature(Signature),
    Plaintext(crate::Secret),
}

impl SepData {
    pub(crate) fn run_key_operation(&self, request: &crate::key_request::Request) {
        let result = (|| {
            if !request.is_live() { return Err(ECANCELED); }
            if let Input::Create(policy) = &request.data.input {
                return self.create_biometric_key(*policy).map(|(blob, public)| Output::Created(blob, public));
            }
            let prepared = self.prepare_verify_capture().map_err(|_| EIO)?;
            if !request.is_live() { return Err(ECANCELED); }
            let credential = self.key_credential()?;
            if !request.is_live() { return Err(ECANCELED); }
            let result = match prepared.verify(Some(credential.handle())) {
                bio::VerifyOutcome::Matched(_) => {
                    if !request.is_live() { Err(ECANCELED) }
                    else {
                        match &request.data.input {
                            Input::Create(_) => Err(EINVAL),
                            Input::Sign(digest) => self.sign_biometric_key(&request.data.blob, digest, &credential)
                                .map(Output::Signature),
                            Input::Decrypt(sealed) => self.decrypt_biometric_key(&request.data.blob, sealed, &credential)
                                .map(Output::Plaintext),
                        }
                    }
                }
                bio::VerifyOutcome::NoMatch => Err(EACCES),
                bio::VerifyOutcome::Failed(crate::ENROL_STATUS_TIMEOUT) => Err(ETIMEDOUT),
                bio::VerifyOutcome::Failed(_) => Err(EIO),
            };
            drop(credential);
            if self.key_api_failed.load(Relaxed) { return Err(EIO); }
            result
        })();
        request.complete(result);
    }
}
