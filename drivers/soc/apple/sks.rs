// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj
//! SEP key store (SKS, endpoint `0x12`): key-bag and key-management request/reply
//! framing, DER-imaged request builders, and lock-state control.

use super::*;
use crate::proto::*;
use kernel::prelude::*;
use kernel::soc::apple::mailbox::Message;

struct KeybagCreateIntent<'a> {
    dev: &'a device::Device,
    slot: keybag::Slot,
    sent: bool,
}

impl<'a> KeybagCreateIntent<'a> {
    fn new(dev: &'a device::Device, slot: keybag::Slot) -> Option<Self> {
        if let Err(e) = keybag::write_intent(slot) {
            dev_err!(dev, "sks: cannot persist keybag-create intent: {:?}\n", e);
            return None;
        }
        Some(Self {
            dev,
            slot,
            sent: false,
        })
    }

    fn sending(&mut self) {
        self.sent = true;
    }
}

impl Drop for KeybagCreateIntent<'_> {
    fn drop(&mut self) {
        if !self.sent {
            if let Err(e) = keybag::discard_uncommitted(self.slot) {
                dev_err!(
                    self.dev,
                    "sks: cannot remove unsent keybag create record: {:?}\n",
                    e
                );
            }
        }
    }
}

impl SepData {
    pub(crate) fn on_sks(&self, msg: Message) {
        let r = crate::sks::decode_sks(&msg);
        let mut probe = self.sks_probe.lock();
        if probe.active {
            if probe.captured.len() < SKS_MAX_CAPTURE {
                let _ = probe.captured.push(msg, GFP_KERNEL);
            }
            drop(probe);
            self.sks_wq.notify_all();
        } else {
            let matched = probe
                .abandoned
                .iter()
                .flatten()
                .find(|a| a.selector == r.selector && a.seq == r.seq)
                .copied();
            drop(probe);
            if let Some(a) = matched {
                let _ = self.sks_zero_buffers();
                self.sks_wedged.store(0, Relaxed);
                dev_warn!(
                    self.dev,
                    "sks: late answer to {} after {} ms; wedge lifted\n",
                    a.label,
                    a.waited_ms
                );
            }
        }
    }

    fn sks_note_abandoned(&self, selector: u8, seq: u8, label: &'static CStr, waited_ms: u64) {
        let mut probe = self.sks_probe.lock();
        let slot = probe.abandoned_next;
        probe.abandoned[slot] = Some(Abandoned {
            selector,
            seq,
            label,
            waited_ms,
        });
        probe.abandoned_next = (slot + 1) % SKS_MAX_ABANDONED;
    }

    fn sks_read_buffer(&self, inbound: bool, len: usize) -> Result<KVec<u8>> {
        let guard = self.ool_sks.lock();
        let pair = guard.as_ref().ok_or(ENODEV)?;
        let src = if inbound {
            &pair.inbound
        } else {
            &pair.outbound
        };
        if len > pair.allocated {
            return Err(EINVAL);
        }
        let mut out = KVec::new();
        // SAFETY: the enclave writes this buffer then signals the mailbox, so it is
        // quiescent when we read it; the driver is the only other accessor.
        out.extend_from_slice(unsafe { &src.as_ref()[..len] }, GFP_KERNEL)?;
        Ok(out)
    }

    fn sks_zero_buffers(&self) -> Result<()> {
        let mut guard = self.ool_sks.lock();
        let pair: &mut Option<OolPair> = &mut guard;
        let buffers = pair.as_mut().ok_or(ENODEV)?;
        // SAFETY: the enclave touches these only between a request word and its
        // reply, and none is outstanding here; the driver is the only other accessor.
        unsafe {
            buffers.inbound.as_mut()[..buffers.allocated].fill(0);
            buffers.outbound.as_mut()[..buffers.allocated].fill(0);
        }
        Ok(())
    }

    pub(crate) fn ool_registered(&self, slot: &Mutex<Option<OolPair>>) -> bool {
        slot.lock().as_ref().is_some_and(|b| b.registered)
    }

    pub(crate) fn sks_next_seq(&self) -> crate::sks::Sequence {
        let n = self.sks_seq.load(Relaxed);
        self.sks_seq.store(n.wrapping_add(1), Relaxed);
        crate::sks::Sequence::from_counter(n as u8)
    }

    fn sks_exchange(
        &self,
        label: &'static CStr,
        msg: Message,
        img: &image::RequestImage,
    ) -> Option<SksOutcome> {
        self.sks_exchange_patient(label, msg, img, 0, true)
    }

    fn sks_exchange_patient(
        &self,
        label: &'static CStr,
        msg: Message,
        img: &image::RequestImage,
        floor_ms: time::Msecs,
        condemn: bool,
    ) -> Option<SksOutcome> {
        let _exchange = self.sks_exchange_lock.lock();
        if !self.ool_registered(&self.ool_sks) {
            return None;
        }

        if self.sks_wedged.load(Relaxed) != 0 {
            return None;
        }

        if self.sks_zero_buffers().is_err() {
            return None;
        }
        if self.ool_write(&self.ool_sks, 0, img.as_slice()).is_err() {
            return None;
        }

        let sized_ms = sks_timeout_for(img.len());
        // The 13.5 key store is given a 0x1770 (6000 ms) wait per delivered
        // request.
        // On a j313 a CREATE took
        // 1.6 s and a COPY 1.66 s, close enough to the 2 s base that a slow
        // one would be abandoned mid-way through its xART writes.
        let floor_ms = match self.profile.key_store {
            profile::KeyStore::Sepos13 { .. } if floor_ms < SKS_TIMEOUT_13_MS => SKS_TIMEOUT_13_MS,
            _ => floor_ms,
        };
        let timeout_ms = if sized_ms < floor_ms {
            floor_ms
        } else {
            sized_ms
        };
        self.sks_arm();
        if self.send(msg).is_err() {
            self.sks_disarm();
            let _ = self.sks_zero_buffers();
            return None;
        }
        let started_ns = crate::shim::boottime_ns();

        let sent = crate::sks::decode_sks(&msg);
        let mut examined = 0usize;
        let mut set_aside = 0u32;
        let correlating = loop {
            self.sks_wait(timeout_ms, examined + 1);

            let next = {
                let probe = self.sks_probe.lock();
                let m = probe.captured.get(examined).copied();
                if m.is_some() {
                    examined += 1;
                }
                m
            };

            let Some(candidate) = next else { break None };

            let decoded = crate::sks::decode_sks(&candidate);
            if decoded.seq == sent.seq && decoded.selector == sent.selector {
                break Some(candidate);
            }

            set_aside = set_aside.saturating_add(1);
            if set_aside >= SKS_MAX_SET_ASIDE {
                break None;
            }
        };

        let leftover = self.sks_disarm();
        let correlating = match correlating {
            Some(m) => Some(m),
            None => leftover.iter().skip(examined).copied().find(|m| {
                let d = crate::sks::decode_sks(m);
                d.seq == sent.seq && d.selector == sent.selector
            }),
        };

        let Some(raw) = correlating else {
            let waited_ms = crate::shim::boottime_ns().saturating_sub(started_ns) / 1_000_000;
            self.sks_note_abandoned(sent.selector, sent.seq, label, waited_ms);
            if condemn {
                self.sks_wedged.store(1, Relaxed);
            }
            dev_err!(
                self.dev,
                "sks: {} got no reply after {} ms\n",
                label,
                waited_ms
            );
            return None;
        };

        let reply = crate::sks::decode_sks(&raw);
        // reply.status is signed here on (e.g. -13, not 0xf3).

        let mut response = Secret::empty();
        if reply.response_size > 0 {
            if let Ok(bytes) = self.sks_read_buffer(false, reply.response_size as usize) {
                response = Secret(bytes);
            }
        }

        // The correlated reply means the enclave has finished with both OOL buffers.
        let _ = self.sks_zero_buffers();

        Some(SksOutcome { reply, response })
    }

    pub(crate) fn sks_report_response<'a>(
        &self,
        _label: &CStr,
        out: &'a SksOutcome,
    ) -> Option<&'a [u8]> {
        if out.response.is_empty() {
            return None;
        }

        let parsed = match image::parse_response(&out.response) {
            Ok(p) => p,
            Err(_) => {
                return None;
            }
        };
        Some(parsed.body)
    }

    /// The IPC header version: 1 unless the 13.5 endpoint init negotiated 2.
    pub(crate) fn sks_ipc_version(&self) -> image::Version {
        if self.sks_header_version.load(Relaxed) == 2 {
            image::Version::V2
        } else {
            image::Version::V1
        }
    }

    pub(crate) fn sks_timestamp_us(&self) -> u64 {
        crate::shim::boottime_ns() / 1000
    }

    pub(crate) fn sks_image_len(&self, img: &image::RequestImage) -> Result<crate::sks::ImageLen> {
        let declared_in = sks_declared_sizes().0;
        if img.len() > declared_in {
            return Err(ENOSPC);
        }
        crate::sks::ImageLen::of(img.len()).ok_or(EINVAL)
    }

    fn sks_seal(&self, op: &crate::sks::SksOp, body: &image::Body) -> Result<SksRequest> {
        let img = image::build_request(self.sks_ipc_version(), self.sks_timestamp_us(), body)?;
        let len = self.sks_image_len(&img)?;
        Ok(SksRequest {
            name: op.name(),
            msg: crate::sks::encode_sks_read(op, self.sks_next_seq(), len),
            img,
        })
    }

    /// `0x4d` get capabilities.
    fn sks_req_get_capabilities(&self) -> Result<SksRequest> {
        let op = crate::sks::sks_get_capabilities();
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(1)?;
        body.put_blob(&[])?;
        self.sks_seal(&op, &body)
    }

    /// `0x2a` set_env in its "false" form, the environment the key store is
    /// given at endpoint init: a 0x40c-byte environment
    /// `{1, mode, 4, 0x400 zero bytes}`, sent as struct version 0, u64 1 and
    /// the environment as a blob.
    fn sks_req_set_env(&self, mode: u32) -> Result<SksRequest> {
        let op = crate::sks::sks_set_env();
        let mut env = KVec::new();
        env.resize(crate::sks::SKS_SET_ENV_LEN, 0u8, GFP_KERNEL)?;
        env[0..4].copy_from_slice(&1u32.to_le_bytes());
        env[4..8].copy_from_slice(&mode.to_le_bytes());
        env[8..12].copy_from_slice(&crate::sks::SKS_SET_ENV_FALSE.to_le_bytes());
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(1)?;
        body.put_blob(&env)?;
        self.sks_seal(&op, &body)
    }

    /// Endpoint init on the 13.5 key store, before any other request: `0x4d`,
    /// whose capability word sets the IPC header version to `min(word, 2)`,
    /// then `set_env`. Returns false if the key store must stay closed this
    /// boot.
    /// The variant-5 key store sends nothing here.
    pub(crate) fn sks_init_endpoint(&self) -> bool {
        let profile::KeyStore::Sepos13 {
            cpx_encryption_mode,
        } = self.profile.key_store
        else {
            self.sks_initialized.store(true, Relaxed);
            return true;
        };

        let Some(out) = self.sks_send(self.sks_req_get_capabilities()) else {
            dev_err!(self.dev, "sks: GET_CAPABILITIES got no reply\n");
            return false;
        };
        // Reply: struct version 0, u64 capability word, blob.
        let word = self
            .sks_report_response(c"GET_CAPABILITIES", &out)
            .filter(|body| out.reply.status == 0 && body.len() >= 12)
            .map(|body| u64::from_le_bytes(body[4..12].try_into().unwrap()));
        match word {
            // Version 0 leaves out the header fields the payload hash covers.
            Some(0) | None => {
                dev_err!(
                    self.dev,
                    "sks: GET_CAPABILITIES status {}, no usable capability word\n",
                    out.reply.status
                );
                return false;
            }
            Some(word) => {
                let version = if word < 2 { 1 } else { 2 };
                self.sks_header_version.store(version, Relaxed);
                dev_info!(
                    self.dev,
                    "sks: capability word {}, IPC header version {}\n",
                    word,
                    version
                );
            }
        }

        let Some(out) = self.sks_send(self.sks_req_set_env(cpx_encryption_mode)) else {
            dev_err!(self.dev, "sks: SET_ENV got no reply\n");
            return false;
        };
        // The reply is the struct-version word 0 alone (0x…4c44-4c5c).
        let accepted = self
            .sks_report_response(c"SET_ENV", &out)
            .is_some_and(|body| body.len() == 4 && image::operation_status(body) == Some(0));
        if out.reply.status != 0 || !accepted {
            dev_err!(
                self.dev,
                "sks: SET_ENV (cpx-encryption-mode {}) refused: status {}\n",
                cpx_encryption_mode,
                out.reply.status
            );
            return false;
        }
        self.sks_initialized.store(true, Relaxed);
        true
    }

    /// The designate struct version, which the reply echoes. On 13.5,
    /// designate uses struct version 0; the other profile uses its own
    /// variant.
    fn sks_designate_variant(&self) -> u32 {
        match self.profile.key_store {
            profile::KeyStore::Sepos13 { .. } => crate::sks::SKS_DESIGNATE_VERSION_13,
            profile::KeyStore::Variant5 => crate::sks::SKS_DESIGNATE_VARIANT,
        }
    }

    /// `0x0d` designate, which makes the system key bag. On 13.5 the body is:
    /// source handle, session, an empty blob and
    /// nothing after it.
    fn sks_req_designate(&self, d: &crate::sks::Designation, secret: &[u8]) -> Result<SksRequest> {
        let mut body = image::Body::new();
        body.put_u32(self.sks_designate_variant())?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(d.source().value())?;
        body.put_i32(d.user().special_handle().value())?;
        if self.profile.key_store == profile::KeyStore::Variant5 {
            body.put_blob(secret)?;
            // Flags is a u64, not u32; a u32 leaves the body short -> enclave answers -13.
            body.put_u64(crate::sks::SKS_DESIGNATE_FLAGS)?;
        } else {
            body.put_blob(&[])?;
        }

        let img = image::build_request(self.sks_ipc_version(), self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_designate(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_DESIGNATE_NAME,
            msg,
            img,
        })
    }

    /// `0x05` unload the source handle.
    pub(crate) fn sks_req_unload_keybag(
        &self,
        handle: crate::sks::KeyBagHandle,
    ) -> Result<SksRequest> {
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(handle.value())?;
        let img = image::build_request(self.sks_ipc_version(), self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_unload_keybag(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_UNLOAD_NAME,
            msg,
            img,
        })
    }

    /// `0x06` UUID read, against the special handle.
    pub(crate) fn sks_req_copy_uuid_special(
        &self,
        special: crate::sks::SpecialHandle,
    ) -> Result<SksRequest> {
        let op = crate::sks::sks_copy_keybag_uuid();
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(special.value())?;
        self.sks_seal(&op, &body)
    }

    pub(crate) fn sks_req_unlock_special(
        &self,
        special: crate::sks::SpecialHandle,
        secret: &[u8],
        healthy: Healthy,
    ) -> Result<SksRequest> {
        let Healthy(()) = healthy;
        if self.profile.key_store != profile::KeyStore::Variant5 {
            return self.sks_req_device_state_unlock(special, secret);
        }
        let mut body = image::Body::new();
        body.put_u32(SKS_LOCK_STATE_VARIANT)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(special.value())?;
        body.put_i32(LockState::Unlocked.wire())?;
        body.put_blob(secret)?;
        body.put_u64(SKS_LOCK_STATE_FLAGS)?;
        let img = image::build_request(self.sks_ipc_version(), self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg =
            crate::sks::encode_sks_change_lock_state(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_LOCK_STATE_NAME,
            msg,
            img,
        })
    }

    /// `0x18` device-state transition to unlocked, the 13.5 identity-session
    /// unlock: `device_state_transition` for the session's handle and secret,
    /// with state 0 and flags 0.
    ///
    /// It uses struct version 0. The body is the version, u64 client,
    /// u32 handle, u32 state, u64 flags and the secret as a blob.
    /// The reply is the version word
    /// and two u64.
    fn sks_req_device_state_unlock(
        &self,
        special: crate::sks::SpecialHandle,
        secret: &[u8],
    ) -> Result<SksRequest> {
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(special.value())?;
        body.put_i32(LockState::Unlocked.wire())?;
        body.put_u64(0)?;
        body.put_blob(secret)?;
        let img = image::build_request(self.sks_ipc_version(), self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_device_state_transition(self.sks_next_seq(), len);
        Ok(SksRequest {
            name: crate::sks::SKS_DEVICE_STATE_NAME,
            msg,
            img,
        })
    }

    /// `0x06` copy key-bag UUID.
    fn sks_req_copy_keybag_uuid(&self, handle: crate::sks::KeyBagHandle) -> Result<SksRequest> {
        let op = crate::sks::sks_copy_keybag_uuid();
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(handle.value())?;
        self.sks_seal(&op, &body)
    }

    fn sks_req_copy_keybag(&self, handle: crate::sks::KeyBagHandle) -> Result<SksRequest> {
        let op = crate::sks::sks_copy_keybag();
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(handle.value())?;
        self.sks_seal(&op, &body)
    }

    /// `0x02` copy the designated biometric identity bag.
    fn sks_req_copy_keybag_special(&self, handle: crate::sks::SpecialHandle) -> Result<SksRequest> {
        let op = crate::sks::sks_copy_keybag();
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(handle.value())?;
        self.sks_seal(&op, &body)
    }

    /// `0x03` load key bag.
    fn sks_req_load_keybag(&self, wrapped: &[u8]) -> Result<SksRequest> {
        let mut body = image::Body::new();
        body.put_u32(0)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_blob(wrapped)?;

        let img = image::build_request(self.sks_ipc_version(), self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_load(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_LOAD_NAME,
            msg,
            img,
        })
    }

    fn sks_req_create_identity_keybag(
        &self,
        secret: &[u8],
        uuid: &[u8; crate::sks::SKS_IDENTITY_UUID_LEN],
        proof: keybag::NoStoredKeyBag,
    ) -> Result<SksRequest> {
        if proof.slot() != keybag::Slot::Identity {
            return Err(EINVAL);
        }
        // The 13.5 and variant-5 key stores encode the identity bag
        // differently; the profile carries the field values (see
        // profile::KeybagCreate). On 13.5 the request is: secret, an empty
        // second blob and the 16-byte identity UUID, nothing after it, decoded
        // as an exact-length message.
        let enc = &self.profile.keybag_create;
        let mut body = image::Body::new();
        body.put_u32(enc.variant)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_u32(enc.bag_type)?;
        body.put_i32(enc.arg)?;
        body.put_blob(secret)?;
        body.put_blob(&[])?;
        body.put_blob(uuid)?;
        if self.profile.key_store == profile::KeyStore::Variant5 {
            body.put_blob(&[])?;
            body.put_u64(0)?;
            body.put_u64(0)?;
            body.put_blob(&[])?;
        }
        let img = image::build_request(self.sks_ipc_version(), self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_create(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_CREATE_NAME,
            msg,
            img,
        })
    }

    // A completed CREATE_KEYBAG that the enclave answered with a non-zero
    // status created nothing, so the header-only intent must not block the
    // next attempt and must not stay behind as a 52-byte file. A timeout or
    // no answer after sending() keeps the intent and stays fail-closed: the
    // enclave may hold a bag this host never named.
    fn sks_mark_create_refused(&self, slot: keybag::Slot, status: i8) {
        if status == 0 {
            return;
        }
        if let Err(e) = keybag::discard_uncommitted(slot) {
            dev_err!(
                self.dev,
                "sks: cannot remove refused keybag create record: {:?}\n",
                e
            );
        }
    }

    pub(crate) fn sks_provision_identity_keybag(&self) -> bool {
        let proof = match keybag::read(keybag::Slot::Identity) {
            Ok(keybag::State::Present(_)) => return true,
            Ok(keybag::State::Absent(proof)) => proof,
            Err(e) => {
                dev_err!(
                    self.dev,
                    "sks: identity keybag state is ambiguous: {:?}\n",
                    e
                );
                return false;
            }
        };
        let slot = proof.slot();
        let mut intent = match KeybagCreateIntent::new(&self.dev, slot) {
            Some(intent) => intent,
            None => return false,
        };
        let mut secret_bytes = KVec::new();
        if secret_bytes.resize(SKS_SECRET_LEN, 0, GFP_KERNEL).is_err()
            || shim::random_bytes(&mut secret_bytes).is_err()
        {
            return false;
        }
        let secret = Secret(secret_bytes);
        let mut uuid = [0u8; crate::sks::SKS_IDENTITY_UUID_LEN];
        if shim::random_bytes(&mut uuid).is_err() {
            return false;
        }
        uuid[6] = (uuid[6] & 0x0f) | 0x40;
        uuid[8] = (uuid[8] & 0x3f) | 0x80;

        let request = match self.sks_req_create_identity_keybag(&secret, &uuid, proof) {
            Ok(request) => request,
            Err(e) => {
                dev_err!(
                    self.dev,
                    "sks: cannot build identity keybag request: {:?}\n",
                    e
                );
                return false;
            }
        };
        // A cold-boot Mac's create rewrites a pre-Linux record for its own
        // reasons (XART-SAFETY.md), so only warm attach gets the window.
        let first_setup = if self.profile.bootstrap == profile::Bootstrap::WarmRegister {
            match self.with_store(|store| store.open_first_setup()) {
                Some(Ok(open)) => open,
                None => false,
                // Unsent, so the intent is dropped as retryable.
                Some(Err(e)) => {
                    dev_err!(
                        self.dev,
                        "sks: cannot record the xART guard's first-setup window ({:?}); not creating the keybag this boot\n",
                        e
                    );
                    return false;
                }
            }
        } else {
            false
        };
        intent.sending();
        let exchanged = self.sks_exchange(request.name, request.msg, &request.img);
        if first_setup {
            self.with_store(|store| store.close_first_setup());
        }
        let Some(out) = exchanged else {
            dev_err!(
                self.dev,
                "sks: CREATE_KEYBAG exchange did not complete (buffers unregistered, wedged, or OOL write failed)\n"
            );
            // sent is true, so the intent file stays. A timeout can mean the
            // enclave created a bag this host never named.
            return false;
        };
        let Some(body) = self.sks_report_response(crate::sks::SKS_CREATE_NAME, &out) else {
            dev_err!(
                self.dev,
                "sks: CREATE_KEYBAG reply carried no parsable body: mailbox status {}, response {} bytes, response_size {}\n",
                out.reply.status,
                out.response.len(),
                out.reply.response_size
            );
            self.sks_mark_create_refused(slot, out.reply.status);
            return false;
        };
        // 13.5 replies with the struct version and the handle alone; the
        // variant-5 reply adds a blob.
        let reply_min = match self.profile.key_store {
            profile::KeyStore::Sepos13 { .. } => 8,
            profile::KeyStore::Variant5 => 12,
        };
        if out.reply.status != 0 || body.len() < reply_min {
            dev_err!(
                self.dev,
                "sks: CREATE_KEYBAG failed: mailbox {}, body {} bytes\n",
                out.reply.status,
                body.len()
            );
            self.sks_mark_create_refused(slot, out.reply.status);
            return false;
        }
        let variant = u32::from_le_bytes(body[0..4].try_into().unwrap());
        let raw_handle = i32::from_le_bytes(body[4..8].try_into().unwrap());
        let end = match self.profile.key_store {
            profile::KeyStore::Sepos13 { .. } => 8,
            profile::KeyStore::Variant5 => {
                let Some((_fv_data, end)) = image::read_blob(body, 8) else {
                    dev_err!(
                        self.dev,
                        "sks: CREATE_KEYBAG reply has no fv_data blob at offset 8 (body {} bytes)\n",
                        body.len()
                    );
                    return false;
                };
                end
            }
        };
        if variant != self.profile.keybag_create.variant || raw_handle < 0 || end != body.len()
        {
            dev_err!(
                self.dev,
                "sks: invalid CREATE_KEYBAG reply: variant {}, handle {}\n",
                variant,
                raw_handle
            );
            return false;
        }
        let handle = crate::sks::KeyBagHandle::from_create_reply(raw_handle);
        // An identity create reads no UUID back: 13.5 names the identity by the
        // UUID it was created with, and saving the identity is
        // just the copy below, then the unload.
        let bag_uuid = match self.profile.key_store {
            profile::KeyStore::Sepos13 { .. } => None,
            profile::KeyStore::Variant5 => {
                let Some(bag_uuid) = self.sks_read_uuid(handle) else {
                    dev_err!(self.dev, "sks: could not read the new keybag's UUID back\n");
                    return false;
                };
                Some(bag_uuid)
            }
        };
        let Some(out) = self.sks_send(self.sks_req_copy_keybag(handle)) else {
            dev_err!(self.dev, "sks: COPY_KEYBAG of the new identity keybag got no reply\n");
            return false;
        };
        let Some(wrapped) = self.wrapped_from_copy_reply(&out, c"new identity keybag") else {
            dev_err!(
                self.dev,
                "sks: COPY_KEYBAG reply unusable: mailbox status {}, response {} bytes\n",
                out.reply.status,
                out.response.len()
            );
            return false;
        };
        let committed = match bag_uuid {
            Some(bag_uuid) => keybag::write_bag_uuid(slot, &wrapped, &bag_uuid, &secret),
            None => keybag::write_generated_uuid(slot, &wrapped, &uuid, &secret),
        };
        if let Err(e) = committed {
            dev_err!(self.dev, "sks: cannot commit identity keybag: {:?}\n", e);
            return false;
        }
        let _ = self.sks_send(self.sks_req_unload_keybag(handle));
        dev_info!(self.dev, "sks: identity keybag provisioned\n");
        true
    }

    pub(crate) fn sks_send(&self, req: Result<SksRequest>) -> Option<SksOutcome> {
        match req {
            Ok(r) => self.sks_exchange(r.name, r.msg, &r.img),
            Err(_) => None,
        }
    }

    pub(crate) fn sks_designate_user_keybag(
        &self,
        handle: crate::sks::KeyBagHandle,
        secret: &[u8],
    ) {
        let Some(user) = crate::sks::DesignateUser::new(SBIO_PROBE_USER_ID) else {
            return;
        };

        let designation = crate::sks::Designation::new(handle, user);

        let Some(out) = self.sks_send(self.sks_req_designate(&designation, secret)) else {
            dev_warn!(
                self.dev,
                "sks: DESIGNATE_KEYBAG did not complete; enrolment will refuse\n"
            );
            return;
        };
        let Some(body) = self.sks_report_response(crate::sks::SKS_DESIGNATE_NAME, &out) else {
            return;
        };

        if body.len() != crate::sks::SKS_DESIGNATE_REPLY_LEN {
            return;
        }
        let variant = u32::from_le_bytes([body[0], body[1], body[2], body[3]]);
        if variant != self.sks_designate_variant() {
            return;
        }

        self.keybag_designated.store(true, Relaxed);

        self.sks_remember_enrolment_material(designation.user().special_handle(), secret);
    }

    fn sks_remember_enrolment_material(&self, special: crate::sks::SpecialHandle, secret: &[u8]) {
        let mut copy = KVec::new();
        if copy.extend_from_slice(secret, GFP_KERNEL).is_err() {
            return;
        }
        *self.enrol_material.lock() = Some(EnrolMaterial {
            special,
            secret: Secret(copy),
            snapshot_identity: None,
        });
    }

    /// `0x04` change lock state (v1).
    pub(crate) fn sks_req_change_lock_state(
        &self,
        handle: crate::sks::KeyBagHandle,
        state: LockState,
        secret: &[u8],
        healthy: Healthy,
    ) -> Result<SksRequest> {
        let Healthy(()) = healthy;
        let mut body = image::Body::new();
        body.put_u32(SKS_LOCK_STATE_VARIANT)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(handle.value())?;
        body.put_i32(state.wire())?;
        body.put_blob(secret)?;
        // Trailing u64 goes after the blob (`0x18` puts it before).
        body.put_u64(SKS_LOCK_STATE_FLAGS)?;
        let img = image::build_request(self.sks_ipc_version(), self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg =
            crate::sks::encode_sks_change_lock_state(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_LOCK_STATE_NAME,
            msg,
            img,
        })
    }

    /// `0x21` verify-secret (variant 1).
    pub(crate) fn sks_req_verify_secret(
        &self,
        special: crate::sks::SpecialHandle,
        secret: &[u8],
        acm_context: &[u8; crate::scrd::SCRD_ACM_HANDLE_LEN],
    ) -> Result<SksRequest> {
        let mut body = image::Body::new();
        body.put_u32(crate::sks::SKS_VERIFY_SECRET_VARIANT)?;
        body.put_u64(crate::sks::SKS_CLIENT_ID)?;
        body.put_i32(special.value())?;
        body.put_blob(secret)?;
        body.put_blob(acm_context)?;
        body.put_u64(0)?;
        let img = image::build_request(self.sks_ipc_version(), self.sks_timestamp_us(), &body)?;
        let len = self.sks_image_len(&img)?;
        let msg = crate::sks::encode_sks_verify_secret(self.sks_next_seq(), len).ok_or(EINVAL)?;
        Ok(SksRequest {
            name: crate::sks::SKS_VERIFY_SECRET_NAME,
            msg,
            img,
        })
    }

    pub(crate) fn sks_step(
        &self,
        label: &CStr,
        build: impl FnOnce(Healthy) -> Result<SksRequest>,
    ) -> Option<SksOutcome> {
        let healthy = self.sks_health_check(label)?;
        let out = self.sks_send(build(healthy))?;
        if out.reply.status != 0 {
            return None;
        }
        Some(out)
    }

    pub(crate) fn sks_health_check(&self, why: &CStr) -> Option<Healthy> {
        // macOS 13.5 sends 0x4d once, at endpoint init, never again.
        if self.profile.key_store != profile::KeyStore::Variant5 {
            if !self.sks_initialized.load(Relaxed) {
                dev_err!(
                    self.dev,
                    "sks: health check ({}): endpoint not initialised\n",
                    why
                );
                return None;
            }
            return Some(Healthy(()));
        }
        let Some(out) = self.sks_send(self.sks_req_get_capabilities()) else {
            dev_err!(
                self.dev,
                "sks: health check ({}) got no reply; endpoint gone for this boot\n",
                why
            );
            return None;
        };
        if out.reply.status != 0 {
            dev_err!(
                self.dev,
                "sks: health check ({}) returned status {}\n",
                why,
                out.reply.status
            );
            return None;
        }
        Some(Healthy(()))
    }

    pub(crate) fn sks_recover(
        &self,
        stored: &keybag::StoredKeyBag,
    ) -> Option<(crate::sks::KeyBagHandle, [u8; keybag::UUID_LEN])> {
        // Opening a 13.5 identity addresses an enclave-resident identity bag by
        // its generated UUID. Older J313 records instead store the bag's
        // read-back UUID and a wrapped bag, which must be loaded as a blob.
        let uuid_load = matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. })
            && stored.uuid_provenance() == keybag::UuidProvenance::AsGenerated;
        let key = if uuid_load {
            &stored.uuid()[..]
        } else {
            stored.wrapped()
        };
        let request = match self.sks_req_load_keybag(key) {
            Ok(request) => request,
            Err(e) => {
                dev_err!(
                    self.dev,
                    "sks: could not build LOAD_KEYBAG request: {:?}\n",
                    e
                );
                return None;
            }
        };
        let out = self.sks_exchange(request.name, request.msg, &request.img)?;

        dev_info!(
            self.dev,
            "sks: LOAD_KEYBAG reply mailbox status {}, response size {}, copied {} bytes\n",
            out.reply.status,
            out.reply.response_size,
            out.response.len()
        );

        let body = match self.sks_report_response(crate::sks::SKS_LOAD_NAME, &out) {
            Some(body) => body,
            None => {
                dev_warn!(
                    self.dev,
                    "sks: LOAD_KEYBAG reply image was empty or malformed\n"
                );
                return None;
            }
        };

        if out.reply.status != 0 {
            dev_warn!(
                self.dev,
                "sks: LOAD_KEYBAG mailbox status {}; no handle\n",
                out.reply.status
            );
            return None;
        }
        if body.len() != SKS_LOAD_REPLY_LEN {
            dev_warn!(
                self.dev,
                "sks: LOAD_KEYBAG returned {} bytes, expected {}\n",
                body.len(),
                SKS_LOAD_REPLY_LEN
            );
            return None;
        }
        let status = i32::from_le_bytes([body[0], body[1], body[2], body[3]]);
        let handle = i32::from_le_bytes([body[4], body[5], body[6], body[7]]);
        if status != 0 || handle < 0 {
            dev_warn!(
                self.dev,
                "sks: LOAD_KEYBAG operation status {}, handle {}\n",
                status,
                handle
            );
            return None;
        }
        let handle = crate::sks::KeyBagHandle::from_load_reply(handle);

        if uuid_load {
            return Some((handle, *stored.uuid()));
        }

        let Some(uuid) = self.sks_read_uuid(handle) else {
            dev_warn!(self.dev, "sks: loaded keybag has no readable UUID\n");
            let _ = self.sks_send(self.sks_req_unload_keybag(handle));
            return None;
        };

        if uuid == *stored.uuid() {
            Some((handle, uuid))
        } else {
            dev_err!(
                self.dev,
                "sks: load succeeded but bag identity mismatch: stored UUID {}, loaded UUID {}\n",
                Hex(stored.uuid()),
                Hex(&uuid)
            );
            let _ = self.sks_send(self.sks_req_unload_keybag(handle));
            None
        }
    }

    pub(crate) fn resnapshot_identity_keybag(&self) -> bool {
        let (special, identity) = {
            let material = self.enrol_material.lock();
            let Some(material) = material.as_ref() else {
                return false;
            };
            let Some(identity) = material.snapshot_identity else {
                dev_err!(self.dev, "sks: refusing snapshot without a recovered-bag identity binding\n");
                return false;
            };
            (material.special, identity)
        };

        let Some(out) = self.sks_send(self.sks_req_copy_keybag_special(special)) else {
            return false;
        };
        let Some(wrapped) = self.wrapped_from_copy_reply(&out, c"the designated identity bag")
        else {
            return false;
        };

        let Some(uuid) = self
            .sks_send(self.sks_req_copy_uuid_special(special))
            .and_then(|out| self.sks_uuid_from_reply(&out))
        else {
            return false;
        };

        match keybag::replace_wrapped(keybag::Slot::Identity, &wrapped, &uuid, &identity) {
            Ok(()) => true,
            Err(e) => {
                dev_err!(
                    self.dev,
                    "enrol: could not commit the identity-bag snapshot ({:?}); may not survive reboot\n",
                    e
                );
                false
            }
        }
    }

    pub(crate) fn sks_read_uuid(&self, handle: crate::sks::KeyBagHandle) -> Option<[u8; keybag::UUID_LEN]> {
        let out = self.sks_send(self.sks_req_copy_keybag_uuid(handle))?;
        self.sks_uuid_from_reply(&out)
    }

    pub(crate) fn sks_uuid_from_reply(&self, out: &SksOutcome) -> Option<[u8; keybag::UUID_LEN]> {
        let body = self.sks_report_response(c"COPY_KEYBAG_UUID", out)?;

        if out.reply.status != 0 || image::operation_status(body).unwrap_or(-1) != 0 {
            return None;
        }
        let (blob, _) = image::read_blob(body, 4)?;
        if blob.len() != keybag::UUID_LEN {
            return None;
        }
        let mut uuid = [0u8; keybag::UUID_LEN];
        uuid.copy_from_slice(blob);
        Some(uuid)
    }

    pub(crate) fn enable_sks(&self) -> Result<()> {
        self.register_ool(&self.ool_sks)
    }

    pub(crate) fn sks_ready(&self) -> bool {
        if self.ool_registered(&self.ool_sks) {
            return true;
        }
        if !self.endpoint_present(proto::EP_SKS) {
            dev_err!(
                self.dev,
                "sks: endpoint 0x{:02x} (key store) not advertised; {} EPs up (control={} xarm={} sbio={} scrd={}); keybag/ref-key cannot run\n",
                proto::EP_SKS,
                self.endpoint_count(),
                self.endpoint_present(proto::EP_CONTROL) as u8,
                self.endpoint_present(proto::EP_XARM) as u8,
                self.endpoint_present(proto::EP_SBIO) as u8,
                self.endpoint_present(proto::EP_SCRD) as u8,
            );
            return false;
        }
        if let Err(e) = self.enable_sks() {
            dev_err!(
                self.dev,
                "sks: could not register out-of-line buffers ({:?})\n",
                e
            );
            return false;
        }
        dev_info!(
            self.dev,
            "sks: endpoint 0x{:02x} (key store) up; {} EPs; out-of-line buffers registered\n",
            proto::EP_SKS,
            self.endpoint_count()
        );
        true
    }

    fn sks_arm(&self) {
        let mut probe = self.sks_probe.lock();
        probe.captured.clear();
        probe.active = true;
    }

    fn sks_disarm(&self) -> KVec<Message> {
        let mut probe = self.sks_probe.lock();
        probe.active = false;
        core::mem::take(&mut probe.captured)
    }

    fn sks_wait(&self, ms: time::Msecs, until: usize) {
        let mut remaining = time::msecs_to_jiffies(ms);
        let mut guard = self.sks_probe.lock();
        loop {
            if self.shutting_down.load(Relaxed)
                || guard.captured.len() >= until || remaining == 0 {
                return;
            }
            match self
                .sks_wq
                .wait_interruptible_timeout(&mut guard, remaining)
            {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        }
    }
}

pub(crate) const SKS_REPLY_BIT: u8 = 0x80;

pub(crate) struct SksOp {
    opcode: u8,
    name: &'static CStr,
}

impl SksOp {
    pub(crate) fn opcode(&self) -> u8 {
        self.opcode
    }
    pub(crate) fn name(&self) -> &'static CStr {
        self.name
    }
}

const OP_SKS_UNWRAP_PFK: u8 = 0x09;
pub(crate) const SKS_UNWRAP_PFK_NAME: &CStr = c"UNWRAP_PFK";

const OP_SKS_NEW_PFK: u8 = 0x10;
pub(crate) const SKS_NEW_PFK_NAME: &CStr = c"NEW_PFK";

const OP_SKS_UNWRAP_MEDIA_KEY: u8 = 0x32;
pub(crate) const SKS_UNWRAP_MEDIA_KEY_NAME: &CStr = c"UNWRAP_MEDIA_KEY";

const OP_SKS_UNWRAP_VEK: u8 = 0x41;
pub(crate) const SKS_UNWRAP_VEK_NAME: &CStr = c"FV_UNWRAP_VEK";

const OP_SKS_SET_PROTECTION: u8 = 0x47;
pub(crate) const SKS_SET_PROTECTION_NAME: &CStr = c"FV_SET_PROTECTION";

const OP_SKS_GET_BLOB_STATE: u8 = 0x48;
pub(crate) const SKS_GET_BLOB_STATE_NAME: &CStr = c"FV_GET_BLOB_STATE";

const OP_SKS_PERFORM_OPERATION: u8 = 0x22;
pub(crate) const SKS_PERFORM_OP_NAME: &CStr = c"PERFORM_OPERATION";

pub(crate) fn encode_sks_perform_operation(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_PERFORM_OPERATION, seq.value(), len.value());
    Some(msg)
}

pub(crate) fn encode_sks_unwrap_media_key(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_UNWRAP_MEDIA_KEY, seq.value(), len.value());
    Some(msg)
}

pub(crate) fn encode_sks_unwrap_vek(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_UNWRAP_VEK, seq.value(), len.value());
    Some(msg)
}

pub(crate) fn encode_sks_unwrap_pfk(seq: Sequence, len: ImageLen) -> Message {
    encode_sks_raw(OP_SKS_UNWRAP_PFK, seq.value(), len.value())
}

pub(crate) fn encode_sks_new_pfk(seq: Sequence, len: ImageLen) -> Message {
    encode_sks_raw(OP_SKS_NEW_PFK, seq.value(), len.value())
}

pub(crate) fn encode_sks_set_protection(seq: Sequence, len: ImageLen) -> Message {
    encode_sks_raw(OP_SKS_SET_PROTECTION, seq.value(), len.value())
}

pub(crate) fn encode_sks_get_blob_state(seq: Sequence, len: ImageLen) -> Message {
    encode_sks_raw(OP_SKS_GET_BLOB_STATE, seq.value(), len.value())
}
const OP_SKS_CAPABILITIES: u8 = 0x4d;
pub(crate) fn sks_get_capabilities() -> SksOp {
    SksOp {
        opcode: OP_SKS_CAPABILITIES,
        name: c"GET_CAPABILITIES",
    }
}

const OP_SKS_SET_ENV: u8 = 0x2a;
pub(crate) fn sks_set_env() -> SksOp {
    SksOp {
        opcode: OP_SKS_SET_ENV,
        name: c"SET_ENV",
    }
}
pub(crate) const SKS_SET_ENV_LEN: usize = 12 + 0x400;
/// Third environment word for the "false" `set_env` sent at endpoint init.
pub(crate) const SKS_SET_ENV_FALSE: u32 = 4;

const OP_SKS_COPY_UUID: u8 = 0x06;
pub(crate) fn sks_copy_keybag_uuid() -> SksOp {
    SksOp {
        opcode: OP_SKS_COPY_UUID,
        name: c"COPY_KEYBAG_UUID",
    }
}

const OP_SKS_COPY_KEYBAG: u8 = 0x02;
pub(crate) fn sks_copy_keybag() -> SksOp {
    SksOp {
        opcode: OP_SKS_COPY_KEYBAG,
        name: c"COPY_KEYBAG",
    }
}

const OP_SKS_CREATE_KEYBAG: u8 = 0x01;
pub(crate) const SKS_CREATE_NAME: &CStr = c"CREATE_KEYBAG";
pub(crate) const SKS_IDENTITY_UUID_LEN: usize = 16;

const OP_SKS_LOAD_KEYBAG: u8 = 0x03;

pub(crate) const SKS_LOAD_NAME: &CStr = c"LOAD_KEYBAG";

const OP_SKS_CHANGE_LOCK_STATE: u8 = 0x04;

pub(crate) const SKS_LOCK_STATE_NAME: &CStr = c"CHANGE_LOCK_STATE";

const OP_SKS_DEVICE_STATE_TRANSITION: u8 = 0x18;
pub(crate) const SKS_DEVICE_STATE_NAME: &CStr = c"DEVICE_STATE_TRANSITION";

pub(crate) fn encode_sks_device_state_transition(seq: Sequence, len: ImageLen) -> Message {
    encode_sks_raw(OP_SKS_DEVICE_STATE_TRANSITION, seq.value(), len.value())
}

const OP_SKS_VERIFY_SECRET: u8 = 0x21;
pub(crate) const SKS_VERIFY_SECRET_NAME: &CStr = c"VERIFY_SECRET";
pub(crate) const SKS_VERIFY_SECRET_VARIANT: u32 = 1;

pub(crate) fn encode_sks_verify_secret(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_VERIFY_SECRET, seq.value(), len.value());
    Some(msg)
}

pub(crate) const SKS_CLIENT_ID: u64 = u64::from_be_bytes(*b"LINUXSKS");
static_assert!(SKS_CLIENT_ID == 0x4c49_4e55_5853_4b53);

#[derive(Clone, Copy)]
pub(crate) struct KeyBagHandle(i32);

impl KeyBagHandle {
    pub(crate) const fn from_create_reply(v: i32) -> KeyBagHandle {
        KeyBagHandle(v)
    }

    pub(crate) const fn from_load_reply(v: i32) -> KeyBagHandle {
        KeyBagHandle(v)
    }

    pub(crate) const fn value(&self) -> i32 {
        self.0
    }
}

const OP_SKS_DESIGNATE_KEYBAG: u8 = 0x0d;

pub(crate) const SKS_DESIGNATE_VARIANT: u32 = 1;
static_assert!(SKS_DESIGNATE_VARIANT == 1);
pub(crate) const SKS_DESIGNATE_VERSION_13: u32 = 0;

pub(crate) const SKS_DESIGNATE_USER_MIN: i32 = 10;
static_assert!(SKS_DESIGNATE_USER_MIN > 0);

#[derive(Clone, Copy)]
pub(crate) struct DesignateUser(i32);

impl DesignateUser {
    pub(crate) fn new(value: i32) -> Option<DesignateUser> {
        if value >= SKS_DESIGNATE_USER_MIN {
            Some(DesignateUser(value))
        } else {
            None
        }
    }

    pub(crate) const fn special_handle(&self) -> SpecialHandle {
        SpecialHandle(-self.0)
    }
}

#[derive(Clone, Copy)]
pub(crate) struct SpecialHandle(i32);

impl SpecialHandle {
    pub(crate) const fn value(&self) -> i32 {
        self.0
    }
}

pub(crate) const SKS_AUTH_TOKEN_LEN: usize = 16;

pub(crate) struct Designation {
    source: KeyBagHandle,
    user: DesignateUser,
}

impl Designation {
    pub(crate) fn new(source: KeyBagHandle, user: DesignateUser) -> Designation {
        Designation { source, user }
    }

    pub(crate) const fn source(&self) -> KeyBagHandle {
        self.source
    }

    pub(crate) const fn user(&self) -> DesignateUser {
        self.user
    }
}

pub(crate) const SKS_DESIGNATE_REPLY_LEN: usize = 4;

pub(crate) const SKS_DESIGNATE_FLAGS: u64 = 0;
static_assert!(SKS_DESIGNATE_FLAGS == 0);

pub(crate) fn encode_sks_designate(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_DESIGNATE_KEYBAG, seq.value(), len.value());
    Some(msg)
}

pub(crate) const SKS_DESIGNATE_NAME: &CStr = c"DESIGNATE_KEYBAG";

const OP_SKS_UNLOAD_KEYBAG: u8 = 0x05;
pub(crate) const SKS_UNLOAD_NAME: &CStr = c"UNLOAD_KEYBAG";

pub(crate) fn encode_sks_unload_keybag(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_UNLOAD_KEYBAG, seq.value(), len.value());
    Some(msg)
}

#[derive(Clone, Copy)]
pub(crate) struct Sequence(u8);

const SEQ_MIN: u8 = 0x60;

impl Sequence {
    pub(crate) const fn from_counter(n: u8) -> Sequence {
        Sequence(SEQ_MIN | (n & !SEQ_MIN))
    }

    pub(crate) const fn value(&self) -> u8 {
        self.0
    }
}

static_assert!(Sequence::from_counter(0).value() != Sequence::from_counter(1).value());
static_assert!(Sequence::from_counter(0).value() == Sequence::from_counter(32).value());

#[derive(Clone, Copy)]
pub(crate) struct ImageLen(u16);

impl ImageLen {
    pub(crate) fn of(len: usize) -> Option<ImageLen> {
        if len < crate::image::HEADER_WIRE {
            return None;
        }
        match u16::try_from(len) {
            Ok(v) => Some(ImageLen(v)),
            Err(_) => None,
        }
    }

    pub(crate) const fn value(&self) -> u16 {
        self.0
    }
}

const fn encode_sks_raw(selector: u8, seq: u8, len: u16) -> Message {
    Message {
        msg0: (EP_SKS as u64)
            | ((selector as u64) << MSG_TAG_SHIFT)
            | ((seq as u64) << MSG_TYPE_SHIFT)
            | ((len as u64) << 48),
        msg1: 0,
    }
}

pub(crate) fn encode_sks_read(op: &SksOp, seq: Sequence, len: ImageLen) -> Message {
    encode_sks_raw(op.opcode(), seq.value(), len.value())
}

pub(crate) fn encode_sks_load(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_LOAD_KEYBAG, seq.value(), len.value());
    Some(msg)
}

pub(crate) fn encode_sks_create(seq: Sequence, len: ImageLen) -> Option<Message> {
    Some(encode_sks_raw(
        OP_SKS_CREATE_KEYBAG,
        seq.value(),
        len.value(),
    ))
}

pub(crate) fn encode_sks_change_lock_state(seq: Sequence, len: ImageLen) -> Option<Message> {
    let msg = encode_sks_raw(OP_SKS_CHANGE_LOCK_STATE, seq.value(), len.value());
    Some(msg)
}

pub(crate) struct SksReply {
    pub(crate) selector: u8,
    pub(crate) seq: u8,
    pub(crate) status: i8,
    pub(crate) response_size: u16,
}

pub(crate) fn decode_sks(msg: &Message) -> SksReply {
    let b = msg.msg0.to_le_bytes();
    SksReply {
        selector: b[1] & !SKS_REPLY_BIT,
        seq: b[2],
        status: b[3] as i8,
        response_size: u16::from_le_bytes([b[6], b[7]]),
    }
}
