// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj
// Copyright 2026 Ryan Murray
//! Touch ID: the biometric endpoint (SBIO) and its `/dev/sep-bio` interface.
//! The enclave matches; no biometric image ever crosses to userspace.

use super::*;
use crate::proto::*;
use crate::sks::SKS_AUTH_TOKEN_LEN;
use kernel::prelude::*;
use kernel::soc::apple::mailbox::Message;

impl SepData {
    fn sbio_enrol_step(&self, op: &crate::sbio::SbioOp) -> Option<KVec<u8>> {
        match self.sbio_call(op) {
            SbioOutcome::Ok(payload) => Some(payload),
            SbioOutcome::PrerequisiteMissing => {
                dev_err!(self.dev, "enrol: {} prerequisite missing (status 0x01)\n", op.name());
                None
            }
            SbioOutcome::Status16 => {
                dev_err!(self.dev, "enrol: {} refused (status 0x16)\n", op.name());
                None
            }
            SbioOutcome::Other => {
                dev_err!(self.dev, "enrol: {} got no usable answer\n", op.name());
                None
            }
        }
    }

    fn sbio_expect_ok(&self, op: &crate::sbio::SbioOp) -> Option<KVec<u8>> {
        match self.sbio_call(op) {
            SbioOutcome::Ok(payload) => Some(payload),
            _ => None,
        }
    }

    // A match step whose refusal ends the verify. libfprint reports every
    // failed verify as the same unspecified error, so the enclave's exact
    // answer has to be recorded here or nowhere. Success is decided as in
    // `sbio_call`.
    fn sbio_verify_step(&self, op: &crate::sbio::SbioOp) -> Option<KVec<u8>> {
        let done = match self.sbio_transfer(op) {
            Ok(done) => done,
            Err(e) => {
                dev_err!(
                    self.dev,
                    "verify: {} transfer failed ({:?})\n",
                    op.name(),
                    e
                );
                return None;
            }
        };
        match done.status.answered() {
            Some(err) if err as u16 == crate::sbio::SBIO_STATUS_OK => Some(done.payload),
            _ => {
                dev_err!(
                    self.dev,
                    "verify: {} answered status {}\n",
                    op.name(),
                    done.status
                );
                None
            }
        }
    }

    pub(crate) fn sbio_call(&self, op: &crate::sbio::SbioOp) -> SbioOutcome {
        let done = match self.sbio_transfer(op) {
            Ok(done) => done,
            Err(e) => {
                if op.opcode() == OP_SBIO_BEGIN_ENROL {
                    dev_err!(self.dev, "enrol: BEGIN_ENROL transfer failed ({:?})\n", e);
                }
                return SbioOutcome::Other;
            }
        };

        let Some(err) = done.status.answered() else {
            if op.opcode() == OP_SBIO_BEGIN_ENROL {
                dev_err!(self.dev, "enrol: BEGIN_ENROL reply had no status: {}\n", done.status);
            }
            return SbioOutcome::Other;
        };

        match err as u16 {
            crate::sbio::SBIO_STATUS_OK => SbioOutcome::Ok(done.payload),
            crate::sbio::SBIO_STATUS_PREREQUISITE => SbioOutcome::PrerequisiteMissing,
            crate::sbio::SBIO_STATUS_16 => SbioOutcome::Status16,
            _ => {
                if op.opcode() == OP_SBIO_BEGIN_ENROL {
                    dev_err!(self.dev, "enrol: BEGIN_ENROL replied with status 0x{:x}\n", err);
                }
                SbioOutcome::Other
            }
        }
    }

    fn sbio_relay(&self, relay: &crate::sbio::SbioRelay<'_>) -> Option<KVec<u8>> {
        match self.sbio_transfer_raw(relay.opcode(), relay.name(), relay.payload()) {
            Ok(done) if done.status.is_ok() => Some(done.payload),
            Ok(done) => {
                dev_err!(self.dev, "enrol: {} relay ({} bytes) refused with status {}\n", relay.name(), relay.payload().len(), done.status);
                None
            }
            Err(e) => {
                dev_err!(self.dev, "enrol: {} relay ({} bytes) failed ({:?})\n", relay.name(), relay.payload().len(), e);
                None
            }
        }
    }

    fn sync_device_view(&self) -> bool {
        let synced = matches!(
            self.sbio_call(&crate::sbio::sbio_update_device_list()),
            SbioOutcome::Ok(_)
        );

        let policy_ok = match self.sbio_call(&crate::sbio::sbio_match_policy()) {
            SbioOutcome::Ok(policy) if policy.len() == crate::sbio::SBIO_MATCH_POLICY_LEN => true,
            SbioOutcome::Ok(_) => false,
            _ => false,
        };

        synced && policy_ok
    }

    fn note_capture_end(&self) {
        self.last_capture_end_ns.store(shim::boottime_ns(), Relaxed);
    }

    fn settle_before_capture(&self) {
        let last = self.last_capture_end_ns.load(Relaxed);
        if last == 0 {
            return;
        }
        let elapsed_ms = shim::boottime_ns().saturating_sub(last) / 1_000_000;
        if elapsed_ms >= u64::from(MATCH_SETTLE_MS) {
            return;
        }
        let remaining = MATCH_SETTLE_MS - elapsed_ms as u32;
        let _ = sensor::idle();
        kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(remaining)));
    }

    fn pace_between_captures(&self) {
        let _ = sensor::idle();

        if !self.await_sensor_state(sensor::STATE_IDLE, c"idle, between captures") {
            dev_warn!(
                self.dev,
                "enrol: sensor did not idle within {} ms; continuing the reposition wait anyway (pause is for the person)\n",
                ENROL_IDLE_TIMEOUT_MS
            );
        }

        kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(
            ENROL_REPOSITION_MS,
        )));
    }

    fn stash_enrol_identity(&self, record: &[u8]) {
        let mut found: KVec<[u8; bio::UUID_LEN]> = KVec::new();
        crate::sbio::enrol_identity_candidates(record, SBIO_PROBE_USER_ID, |_at, uuid| {
            let _ = found.push(uuid, GFP_KERNEL);
        });
        *self.enrol_identity_candidates.lock() = found;
    }

    fn identity_just_enrolled(&self) -> Option<[u8; bio::UUID_LEN]> {
        let candidates = core::mem::take(&mut *self.enrol_identity_candidates.lock());
        let listed = self.enclave_identities_for(SBIO_PROBE_USER_ID)?;

        let mut confirmed: KVec<[u8; bio::UUID_LEN]> = KVec::new();
        for uuid in candidates.iter() {
            if listed.iter().any(|u| u == uuid) && !confirmed.iter().any(|u| u == uuid) {
                if confirmed.push(*uuid, GFP_KERNEL).is_err() {
                    break;
                }
            }
        }

        if confirmed.len() == 1 {
            return Some(confirmed[0]);
        }

        let index = self.bio_index.lock();
        let mut new_identity = None;
        for uuid in listed.iter() {
            if index.contains_uuid(uuid) {
                continue;
            }
            if new_identity.is_some() {
                return None;
            }
            new_identity = Some(*uuid);
        }
        new_identity
    }

    fn reconcile_identities(&self) {
        let Some(listed) = self.enclave_identities_for(SBIO_PROBE_USER_ID) else {
            return;
        };

        if listed.is_empty() {
            return;
        }

        let dropped = {
            let mut index = self.bio_index.lock();
            match index.reconcile_to(&listed) {
                Ok(dropped) => dropped,
                Err(_) => {
                    return;
                }
            }
        };

        let held = {
            let index = self.bio_index.lock();
            index.total()
        };
        if dropped.is_empty() && held == listed.len() {
            return;
        }
        let persisted = {
            let index = self.bio_index.lock();
            self.with_host_store(|store| index.persist(store))
        };
        match persisted {
            Some(Ok(())) => {},
            _ => dev_warn!(
                self.dev,
                "sbio: the reconciled index could not be written; it is correct in memory for this boot and will be rebuilt at the next attach\n"
            ),
        }
    }

    fn enclave_identities_for(&self, user_id: i32) -> Option<KVec<[u8; bio::UUID_LEN]>> {
        let op = crate::sbio::sbio_list_identities();
        let SbioOutcome::Ok(reply) = self.sbio_call(&op) else {
            return None;
        };

        let records = crate::sbio::IdentityRecords::new(&reply)?;

        let mut out = KVec::new();
        let mut failed = false;
        records.each_built_in_for(user_id, |identity| {
            if out.push(*identity.uuid(), GFP_KERNEL).is_err() {
                failed = true;
            }
        });
        if failed {
            return None;
        }
        Some(out)
    }

    fn finish_enrolment(&self, outcome: core::result::Result<[u8; bio::UUID_LEN], u32>) {
        let filed = {
            let mut session = self.bio_session.lock();
            let mut index = self.bio_index.lock();
            bio::enrol_finish(
                &mut session, self.capture_epoch.load(Relaxed), &mut index, outcome,
            )
        };
        if !filed {
            return;
        }
        // lock ordering: store lock outside the session lock (UAF otherwise)
        if outcome.is_ok() {
            let index = self.bio_index.lock();
            let saved = self.with_host_store(|store| index.persist(store));
            match saved {
                Some(Ok(())) => {},
                Some(Err(e)) => dev_err!(
                    self.dev,
                    "enrol: enclave holds a new template but the host index could not be written ({:?})\n",
                    e
                ),
                None => {}
            }
        }
        self.bio_wake();
    }

    fn bio_wake(&self) {
        if let Some(dev) = self.bio_dev.lock().as_ref() {
            dev.wake();
        }
    }

    pub(crate) fn attach_sensor(&self) {
        if let Err(e) = sensor::register_driver() {
            dev_err!(
                self.dev,
                "sensor: could not register the SPI driver: {:?}\n",
                e
            );
            return;
        }

        match dt::enable_spi_sensor(sensor::CONTROLLER_BASE, sensor::CHIP_SELECT) {
            Ok(true) => {},
            Ok(false) => {
                dev_info!(
                    self.dev,
                    "sensor: SPI controller withheld by the J700 firmware handoff; Touch ID unavailable\n"
                );
                return;
            }
            Err(e) => {
                dev_warn!(
                    self.dev,
                    "sensor: could not enable the SPI bus at 0x{:x} or create the sensor node ({:?}); no sensor\n",
                    sensor::CONTROLLER_BASE,
                    e
                );
                return;
            }
        }

        let bound = sensor::is_bound();
        if bound && sensor::power_source() == sensor::PowerSource::None {
            dev_warn!(
                self.dev,
                "sensor: no power line ({}); an unpowered sensor answers sixteen zero bytes like a dead bus\n",
                sensor::power_source().name()
            );
        } else if bound {
            dev_info!(self.dev, "sensor: power via {}\n", sensor::power_source().name());
        }
    }

    fn open_enrolment_context(&self, user: crate::sbio::UserId) -> bool {
        let listed = match self.enclave_identities_for(SBIO_PROBE_USER_ID) {
            Some(list) => list.len(),
            None => {
                dev_warn!(self.dev, "enrol: identity enumeration failed; trying the existing context\n");
                return self.enrol_into_existing_context(user);
            }
        };

        let Some(proof) = crate::sbio::NoExistingCatacomb::from_zero_identities(listed) else {
            return self.enrol_into_existing_context(user);
        };

        self.open_fresh_context(user, &proof)
    }

    fn enrol_into_existing_context(&self, user: crate::sbio::UserId) -> bool {
        if !self.activate_protected_config(user) {
            return false;
        }
        true
    }

    fn open_fresh_context(
        &self,
        user: crate::sbio::UserId,
        proof: &crate::sbio::NoExistingCatacomb,
    ) -> bool {
        let system = crate::sbio::sbio_select_context(crate::sbio::ContextScope::SYSTEM, proof);
        if self.sbio_enrol_step(&system).is_none() {
            return false;
        }

        let Some(reply) = self.read_component_states() else {
            dev_warn!(self.dev, "enrol: could not read component state after system SELECT_CONTEXT\n");
            return false;
        };
        let states = crate::sbio::ComponentStates::new(&reply);
        let owner_initial_state = states.state_for(crate::sbio::CatacombUser::OWNER.value());

        for (who, kind, what) in [
            (
                crate::sbio::CatacombUser::MASTER,
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"master catacomb",
            ),
            (
                crate::sbio::CatacombUser::OWNER,
                PRIVATE_TYPE_CATACOMB_OWNER,
                c"owner catacomb",
            ),
        ] {
            let Some(state) = states.state_for(who.value()) else {
                dev_err!(self.dev, "enrol: {} is absent after system SELECT_CONTEXT\n", what);
                return false;
            };
            dev_info!(self.dev, "enrol: after system SELECT_CONTEXT {} state 0x{:x}\n", what, state);
            if state & crate::sbio::COMPONENT_STATE_ACTIVE == 0 {
                dev_err!(self.dev, "enrol: {} is not active after system SELECT_CONTEXT\n", what);
                return false;
            }
            // On m2, state 0x3 refused SAVE_CATACOMB with status 0x6. The
            // 0x04 save-pending bit is also used by the T2 Catacomb protocol;
            // only persist a fresh component when SEP reports that bit.
            if state & crate::sbio::COMPONENT_STATE_SAVE_PENDING == 0 {
                continue;
            }
            if !self.save_catacomb(who, kind, what) {
                dev_err!(self.dev, "enrol: could not save {} after selecting the system context\n", what);
                return false;
            }
        }

        // The owner Catacomb must be exported even though state 0x3 has no
        // save-pending bit. J313, J414s and J314s each restored a fresh
        // enrollment after this export and user-before-master completion.
        // Every profile now does this.
        if self.profile.persistent_enrol || *module_parameters::j414s_persistent_enrol.value() != 0 {
            if owner_initial_state == Some(0x3)
                && !self.save_catacomb(
                    crate::sbio::CatacombUser::OWNER,
                    PRIVATE_TYPE_CATACOMB_OWNER,
                    c"fresh-context owner catacomb (user 501)",
                )
            {
                dev_err!(self.dev, "enrol: opt-in fresh-context owner export failed\n");
                return false;
            }
            if crate::catacomb::read(PRIVATE_TYPE_CATACOMB_OWNER).is_none() {
                dev_err!(self.dev, "enrol: opt-in fresh context has no owner file\n");
                return false;
            }
        }

        let per_user =
            crate::sbio::sbio_select_context(crate::sbio::ContextScope::user(user), proof);
        if self.sbio_enrol_step(&per_user).is_none() {
            return false;
        }

        if !self.activate_protected_config(user) {
            return false;
        }

        true
    }

    fn activate_protected_config(&self, user: crate::sbio::UserId) -> bool {
        let op = crate::sbio::sbio_protected_config(user);
        let Some(config) = self.sbio_enrol_step(&op) else {
            return false;
        };
        if config.len() != crate::sbio::SBIO_PROTECTED_CONFIG_LEN {
            dev_err!(self.dev, "enrol: protected configuration has {} bytes, expected {}\n", config.len(), crate::sbio::SBIO_PROTECTED_CONFIG_LEN);
            return false;
        }
        true
    }

    fn begin_enrolment_on_enclave(&self) -> Option<OpenEnrolment<'_>> {
        if self.enrol_open.load(Relaxed) {
            self.enrol_open.store(false, Relaxed);
            let _ = self.sbio_call(&crate::sbio::sbio_cancel_operation());
        }

        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            dev_err!(self.dev, "enrol: invalid probe user id\n");
            return None;
        };

        // 0x03 answers -3 until a user key bag is designated
        if !self.keybag_designated.load(Relaxed) {
            dev_err!(self.dev, "enrol: no user keybag was designated\n");
            return None;
        }

        if self.enrol_material.lock().is_none() {
            dev_err!(self.dev, "enrol: no enrolment material is held\n");
            return None;
        }

        // 0x03 answers 0x1 until an enrolment context exists
        if !self.open_enrolment_context(user) {
            dev_err!(self.dev, "enrol: could not open the enclave enrolment context\n");
            return None;
        }

        let Some(acm_handle) = self.establish_passcode_validated_context(user) else {
            dev_err!(
                self.dev,
                "enrol: could not establish the PasscodeValidated SCRD credential; not falling back to a transient SKS token (a catacomb sealed under one does not survive a reboot)\n"
            );
            return None;
        };
        let op =
            crate::sbio::sbio_begin_enrol(user, crate::sbio::BE_AUTH_TYPE_ACM_CONTEXT, &acm_handle);

        match self.sbio_enrol_step(&op) {
            Some(_payload) => {
                self.enrol_open.store(true, Relaxed);
                Some(OpenEnrolment {
                    sep: self,
                    armed: true,
                })
            }
            _ => None,
        }
    }

    fn probe_owner_export(&self) {
        if *module_parameters::probe_owner_export.value() == 0 {
            return;
        }
        if self.templates_restored.load(Relaxed)
            || crate::catacomb::read(PRIVATE_TYPE_CATACOMB_OWNER).is_some()
            || crate::catacomb::read(PRIVATE_TYPE_CATACOMB_MASTER).is_none()
            || crate::catacomb::read(PRIVATE_TYPE_CATACOMB_USER).is_none()
        {
            dev_warn!(self.dev, "owner-probe: preconditions not met; no context selected\n");
            return;
        }
        let Some(identities) = self.enclave_identities_for(SBIO_PROBE_USER_ID) else {
            dev_warn!(self.dev, "owner-probe: identity count unavailable; no context selected\n");
            return;
        };
        let Some(proof) = crate::sbio::NoExistingCatacomb::from_zero_identities(identities.len()) else {
            dev_warn!(self.dev, "owner-probe: enclave identity exists; no context selected\n");
            return;
        };

        // This is destructive to the enclave's *volatile* current context.
        // It is opt-in and only reached after proving zero live identities.
        let system = crate::sbio::sbio_select_context(crate::sbio::ContextScope::SYSTEM, &proof);
        if self.sbio_expect_ok(&system).is_none() {
            dev_warn!(self.dev, "owner-probe: system context selection refused\n");
            return;
        }
        let state = self.read_component_states().and_then(|reply| {
            crate::sbio::ComponentStates::new(&reply)
                .state_for(crate::sbio::CatacombUser::OWNER.value())
        });
        dev_info!(self.dev, "owner-probe: selected owner component state {:?}\n", state);
        if !matches!(state, Some(value) if value & crate::sbio::COMPONENT_STATE_ACTIVE != 0) {
            return;
        }
        let saved = self.save_catacomb(
            crate::sbio::CatacombUser::OWNER,
            PRIVATE_TYPE_CATACOMB_OWNER,
            c"owner catacomb (user 501) diagnostic",
        );
        dev_warn!(self.dev, "owner-probe: owner SAVE_CATACOMB and CONFIRM_SAVE completed {}\n", saved);
    }

    fn restore_all_components(&self) -> bool {
        // 0x6b is re-read before every component, not once up front.
        let user_id = SBIO_PROBE_USER_ID;
        let mut any = false;
        let mut missing_files = false;
        let mut user_outcome = RestoreOutcome::NoStoredFile;

        // A 0x8002 reply is not sufficient evidence of restoration. Re-read
        // the component state for every load, including the user component.
        for (id, kind, what) in [
            (
                crate::sbio::CatacombUser::MASTER.value(),
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"master catacomb",
            ),
            (
                crate::sbio::CatacombUser::OWNER.value(),
                PRIVATE_TYPE_CATACOMB_OWNER,
                c"owner catacomb",
            ),
            (user_id, PRIVATE_TYPE_CATACOMB_USER, c"user catacomb"),
        ] {
            let Some(reply) = self.read_component_states() else {
                return false;
            };
            let states = crate::sbio::ComponentStates::new(&reply);
            let state = states.state_for(id);
            let outcome = self.restore_catacomb(state, id, kind, what);
            dev_info!(
                self.dev,
                "sbio: restore {} state {:?} outcome {:?}\n",
                what,
                state,
                outcome
            );
            match outcome {
                RestoreOutcome::Restored | RestoreOutcome::AlreadyActive => any = true,
                RestoreOutcome::NoStoredFile => missing_files = true,
                RestoreOutcome::Failed | RestoreOutcome::EmptyTolerated => {}
            }
            if id == user_id {
                user_outcome = outcome;
            }
        }

        let user_ok = matches!(
            user_outcome,
            RestoreOutcome::Restored | RestoreOutcome::AlreadyActive
        );
        let lockout_outcome = self.restore_lockout();
        dev_info!(self.dev, "sbio: restore lockout outcome {:?}\n", lockout_outcome);
        let lockout = match lockout_outcome {
            RestoreOutcome::Restored | RestoreOutcome::AlreadyActive => true,
            RestoreOutcome::EmptyTolerated => true,
            RestoreOutcome::NoStoredFile => {
                missing_files = true;
                false
            }
            RestoreOutcome::Failed => false,
        };

        if missing_files {
            dev_warn!(
                self.dev,
                "sbio: at least one catacomb/lockout file is absent; match readiness depends on the user restore and device-view proof\n"
            );
        }

        let _ = any;
        user_ok && lockout
    }

    fn read_component_states(&self) -> Option<KVec<u8>> {
        let op = crate::sbio::sbio_context_state();
        let SbioOutcome::Ok(reply) = self.sbio_call(&op) else {
            return None;
        };

        let states = crate::sbio::ComponentStates::new(&reply);
        if states.count() == 0 {
            return None;
        }
        Some(reply)
    }

    fn restore_catacomb(
        &self,
        state: Option<u32>,
        id: i32,
        kind: u8,
        what: &CStr,
    ) -> RestoreOutcome {
        let Some(state) = state else {
            return RestoreOutcome::Failed;
        };
        match crate::sbio::component_action(state) {
            crate::sbio::ComponentAction::AlreadyActive => RestoreOutcome::AlreadyActive,
            crate::sbio::ComponentAction::Unsupported => RestoreOutcome::Failed,
            crate::sbio::ComponentAction::Load => {
                let Some(blob) = self.read_stored(kind, what) else {
                    return RestoreOutcome::NoStoredFile;
                };

                let at = crate::sbio::SBIO_SAVED_USER_ID_AT;
                if blob.len() < at + 4 {
                    return RestoreOutcome::Failed;
                }
                let carried =
                    i32::from_le_bytes([blob[at], blob[at + 1], blob[at + 2], blob[at + 3]]);
                if carried != id {
                    return RestoreOutcome::Failed;
                }

                let Some(request) = crate::sbio::SbioLoadCatacomb::new(&blob) else {
                    return RestoreOutcome::Failed;
                };

                match self.sbio_transfer_raw(request.opcode(), request.name(), request.payload()) {
                    Ok(done) if done.status.is_ok() => {
                        self.confirm_active(id, what, LoadAnswer::StatusZero)
                    }
                    Ok(done)
                        if done.status.answered()
                            == Some(crate::sbio::SBIO_STATUS_COLD_TRANSITION) =>
                    {
                        let active = self.confirm_active(id, what, LoadAnswer::ColdTransition);
                        if id != SBIO_PROBE_USER_ID
                            || !matches!(&active, RestoreOutcome::Restored)
                        {
                            return active;
                        }
                        match self.log_identity_count(c"after first user cold-transition load") {
                            Some(n) if n > 0 => return active,
                            Some(0) => {}
                            _ => return RestoreOutcome::Failed,
                        }

                        // Diagnostic only: one idempotent retry of the same
                        // host blob, with no context reset or save. An active
                        // state by itself did not prove a live identity on J414s.
                        let second = self.sbio_transfer_raw(
                            request.opcode(), request.name(), request.payload()
                        );
                        let answer = match second {
                            Ok(done) if done.status.is_ok() => LoadAnswer::StatusZero,
                            Ok(done)
                                if done.status.answered()
                                    == Some(crate::sbio::SBIO_STATUS_COLD_TRANSITION) =>
                            {
                                LoadAnswer::ColdTransition
                            }
                            Ok(done)
                                if done.status.answered()
                                    == Some(crate::sbio::SBIO_STATUS_ALREADY_ACTIVE) =>
                            {
                                LoadAnswer::AlreadyActive
                            }
                            Ok(done) => {
                                dev_warn!(self.dev, "sbio: second user LOAD_CATACOMB refused with status {}\n", done.status);
                                return RestoreOutcome::Failed;
                            }
                            Err(e) => {
                                dev_warn!(self.dev, "sbio: second user LOAD_CATACOMB transfer failed ({:?})\n", e);
                                return RestoreOutcome::Failed;
                            }
                        };
                        let active = self.confirm_active(id, what, answer);
                        match (active, self.log_identity_count(c"after second user cold-transition load")) {
                            (RestoreOutcome::Restored, Some(n)) if n > 0 => RestoreOutcome::Restored,
                            _ => RestoreOutcome::Failed,
                        }
                    }
                    // 0x101 ALREADY_ACTIVE is a success
                    Ok(done)
                        if done.status.answered()
                            == Some(crate::sbio::SBIO_STATUS_ALREADY_ACTIVE) =>
                    {
                        self.confirm_active(id, what, LoadAnswer::AlreadyActive)
                    }
                    Ok(done) => {
                        dev_warn!(self.dev, "sbio: {} LOAD_CATACOMB refused with status {}\n", what, done.status);
                        RestoreOutcome::Failed
                    }
                    Err(e) => {
                        dev_warn!(self.dev, "sbio: {} LOAD_CATACOMB transfer failed ({:?})\n", what, e);
                        RestoreOutcome::Failed
                    }
                }
            }
        }
    }

    fn confirm_active(&self, id: i32, what: &CStr, answer: LoadAnswer) -> RestoreOutcome {
        let Some(reply) = self.read_component_states() else {
            dev_err!(
                self.dev,
                "sbio: the {} answered {} and 0x6b could not be re-read; treating unknown as failure\n",
                what,
                answer.describe()
            );
            return RestoreOutcome::Failed;
        };
        let states = crate::sbio::ComponentStates::new(&reply);

        let state = states.state_for(id);
        dev_info!(self.dev, "sbio: {} LOAD_CATACOMB answered {} with post-load state {:?}\n", what, answer.describe(), state);
        match state {
            Some(state) if state & crate::sbio::COMPONENT_STATE_ACTIVE != 0 => {
                RestoreOutcome::Restored
            }
            Some(state) if state & crate::sbio::COMPONENT_STATE_COLD != 0 => RestoreOutcome::Failed,
            Some(_) => RestoreOutcome::Failed,
            None => RestoreOutcome::Failed,
        }
    }

    fn restore_lockout(&self) -> RestoreOutcome {
        let Some(blob) = self.read_stored(PRIVATE_TYPE_LOCKOUT, c"bio lockout record") else {
            return RestoreOutcome::NoStoredFile;
        };
        let Some(request) = crate::sbio::SbioLoadLockout::new(&blob) else {
            return RestoreOutcome::Failed;
        };
        match self.sbio_transfer_raw(request.opcode(), request.name(), request.payload()) {
            Ok(done) if done.status.is_ok() => RestoreOutcome::Restored,
            Ok(done)
                if matches!(
                    done.status,
                    transfer::DeviceStatus::Answered(crate::sbio::SBIO_STATUS_COLD_TRANSITION)
                ) =>
            {
                RestoreOutcome::EmptyTolerated
            }
            Ok(_) => RestoreOutcome::Failed,
            Err(_) => RestoreOutcome::Failed,
        }
    }

    fn read_stored(&self, kind: u8, _what: &CStr) -> Option<KVec<u8>> {
        if crate::catacomb::is_kind(kind) {
            if let Some(blob) = crate::catacomb::read(kind) {
                return Some(blob);
            }
        }
        match self.with_host_store(|store| store.read(&store::Key::root(kind))) {
            Some(Ok(Some(blob))) => Some(blob),
            Some(Ok(None)) => None,
            Some(Err(_)) => None,
            None => None,
        }
    }

    fn save_all_components(&self, user: crate::sbio::UserId) -> bool {
        if self.profile.persistent_enrol || *module_parameters::j414s_persistent_enrol.value() != 0 {
            return self.save_all_components_user_first(user);
        }
        // On m2fix11, a completed enrollment marked both master and user 0x7,
        // but saving and confirming the user first was followed by master
        // SAVE_CATACOMB status 0x6. Try the master before the user at this
        // completion gate; post-match persistence keeps its separate order.
        // Re-read state before each save and never send an unconditional one.
        for (who, kind, what) in [
            (
                crate::sbio::CatacombUser::MASTER,
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"master catacomb (user -1)",
            ),
            (
                crate::sbio::CatacombUser::OWNER,
                PRIVATE_TYPE_CATACOMB_OWNER,
                c"owner catacomb (user 501)",
            ),
            (
                crate::sbio::CatacombUser::enrolling(user),
                PRIVATE_TYPE_CATACOMB_USER,
                c"user catacomb",
            ),
        ] {
            let Some(reply) = self.read_component_states() else {
                dev_err!(self.dev, "enrol: cannot read component state before saving {}\n", what);
                return false;
            };
            let states = crate::sbio::ComponentStates::new(&reply);
            let Some(state) = states.state_for(who.value()) else {
                dev_err!(self.dev, "enrol: {} is absent at enrollment completion\n", what);
                return false;
            };
            dev_info!(self.dev, "enrol: completed {} state 0x{:x}\n", what, state);
            if state & crate::sbio::COMPONENT_STATE_ACTIVE == 0 {
                dev_err!(self.dev, "enrol: {} is not active at enrollment completion\n", what);
                return false;
            }
            if state & crate::sbio::COMPONENT_STATE_SAVE_PENDING != 0 {
                if !self.save_catacomb(who, kind, what) {
                    return false;
                }
            } else if who.value() == user.value() {
                dev_err!(self.dev, "enrol: completed user catacomb is not marked for saving\n");
                return false;
            }
            if (who.value() == user.value()
                || who.value() == crate::sbio::CatacombUser::MASTER.value())
                && crate::catacomb::read(kind).is_none()
            {
                dev_err!(self.dev, "enrol: {} has no durable host file\n", what);
                return false;
            }
        }
        if !self.save_lockout() {
            dev_err!(self.dev, "enrol: lockout persistence failed\n");
            return false;
        }
        true
    }

    fn save_all_components_user_first(&self, user: crate::sbio::UserId) -> bool {
        if crate::catacomb::read(PRIVATE_TYPE_CATACOMB_OWNER).is_none() {
            dev_err!(self.dev, "enrol: opt-in completion has no owner Catacomb file\n");
            return false;
        }
        // Saving the user can change system material. Persist it before the
        // master, as in the reboot-tested J313/J414s enrollment paths.
        for (who, kind, what) in [
            (
                crate::sbio::CatacombUser::enrolling(user),
                PRIVATE_TYPE_CATACOMB_USER,
                c"opt-in completed user catacomb",
            ),
            (
                crate::sbio::CatacombUser::MASTER,
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"opt-in post-user master catacomb",
            ),
        ] {
            let Some(reply) = self.read_component_states() else {
                dev_err!(self.dev, "enrol: opt-in cannot read state before saving {}\n", what);
                return false;
            };
            let state = crate::sbio::ComponentStates::new(&reply).state_for(who.value());
            dev_info!(self.dev, "enrol: opt-in {} state {:?}\n", what, state);
            let Some(state) = state else {
                return false;
            };
            if state & crate::sbio::COMPONENT_STATE_ACTIVE == 0 {
                return false;
            }
            if who.value() == user.value()
                && state & crate::sbio::COMPONENT_STATE_SAVE_PENDING == 0
            {
                dev_err!(self.dev, "enrol: opt-in user catacomb is not save-pending\n");
                return false;
            }
            if !self.save_catacomb(who, kind, what) {
                return false;
            }
            if crate::catacomb::read(kind).is_none() {
                dev_err!(self.dev, "enrol: opt-in {} has no durable host file\n", what);
                return false;
            }
        }
        if !self.save_lockout() {
            dev_err!(self.dev, "enrol: opt-in lockout persistence failed\n");
            return false;
        }
        true
    }

    fn save_after_match(&self, user: crate::sbio::UserId) -> bool {
        if !self.save_lockout() {
            dev_err!(
                self.dev,
                "verify: post-match lockout persistence failed; refusing to claim the AP and SEP anti-replay state are synchronized\n"
            );
            return false;
        }

        // Matching can update a template without making every component
        // saveable. The user must be considered before the master, and 0x6b
        // must be re-read after each save because saving the user can change
        // the master's state. Never claim persistence without host copies.
        for (who, kind, what) in [
            (
                crate::sbio::CatacombUser::enrolling(user),
                PRIVATE_TYPE_CATACOMB_USER,
                c"post-match user catacomb",
            ),
            (
                crate::sbio::CatacombUser::MASTER,
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"post-match master catacomb",
            ),
        ] {
            let Some(reply) = self.read_component_states() else {
                dev_err!(self.dev, "verify: cannot read state before saving {}\n", what);
                return false;
            };
            let states = crate::sbio::ComponentStates::new(&reply);
            let Some(state) = states.state_for(who.value()) else {
                dev_err!(self.dev, "verify: {} is absent after matching\n", what);
                return false;
            };
            if state & crate::sbio::COMPONENT_STATE_ACTIVE == 0 {
                dev_err!(self.dev, "verify: {} is not active after matching (state 0x{:x})\n", what, state);
                return false;
            }
            if state & crate::sbio::COMPONENT_STATE_SAVE_PENDING != 0
                && !self.save_catacomb(who, kind, what)
            {
                dev_err!(self.dev, "verify: {} persistence failed\n", what);
                return false;
            }
            if crate::catacomb::read(kind).is_none() {
                dev_err!(self.dev, "verify: {} has no durable host file\n", what);
                return false;
            }
        }

        if !self.resnapshot_identity_keybag() {
            dev_err!(
                self.dev,
                "verify: post-match identity-bag snapshot failed; the newly confirmed catacomb has no matching durable bag image\n"
            );
            return false;
        }

        true
    }

    fn save_catacomb(&self, who: crate::sbio::CatacombUser, kind: u8, what: &CStr) -> bool {
        let selector = crate::sbio::SaveSelector::new(who);

        let save = match self.sbio_transfer(&crate::sbio::sbio_save_catacomb(&selector)) {
            Ok(done) if done.status.is_ok() => done,
            Ok(done) => {
                dev_err!(self.dev, "enrol: {} SAVE_CATACOMB refused with status {}\n", what, done.status);
                return false;
            }
            Err(e) => {
                dev_err!(self.dev, "enrol: {} SAVE_CATACOMB transfer failed ({:?})\n", what, e);
                return false;
            }
        };
        let blob = save.payload;
        if blob.len() < crate::sbio::SBIO_SAVED_MIN || blob.len() > crate::sbio::SBIO_SAVED_MAX {
            dev_err!(self.dev, "enrol: {} SAVE_CATACOMB returned invalid length {}\n", what, blob.len());
            return false;
        }

        let at = crate::sbio::SBIO_SAVED_USER_ID_AT;
        let carried = i32::from_le_bytes([blob[at], blob[at + 1], blob[at + 2], blob[at + 3]]);
        if carried != who.value() {
            dev_err!(self.dev, "enrol: {} SAVE_CATACOMB returned user id {}, expected {}\n", what, carried, who.value());
            return false;
        }

        if crate::catacomb::write(kind, &blob).is_err() {
            dev_err!(self.dev, "enrol: {} SAVE_CATACOMB host persistence failed\n", what);
            return false;
        }

        match self.sbio_transfer(&crate::sbio::sbio_confirm_save(&selector)) {
            Ok(done) if done.status.is_ok() => true,
            Ok(done) => {
                dev_err!(self.dev, "enrol: {} CONFIRM_SAVE refused with status {}\n", what, done.status);
                false
            }
            Err(e) => {
                dev_err!(self.dev, "enrol: {} CONFIRM_SAVE transfer failed ({:?})\n", what, e);
                false
            }
        }
    }

    fn save_lockout(&self) -> bool {
        let Some(blob) = self.sbio_expect_ok(&crate::sbio::sbio_save_lockout()) else {
            return false;
        };
        if blob.is_empty() {
            return false;
        }
        match self
            .with_host_store(|store| store.write(&store::Key::root(PRIVATE_TYPE_LOCKOUT), &blob))
        {
            Some(Ok(())) => true,
            Some(Err(_)) => false,
            None => false,
        }
    }

    fn enable_sbio(&self) -> Result<()> {
        if self.sbio_ready.load(Relaxed) {
            return Ok(());
        }

        self.register_ool(&self.ool_sbio)?;

        self.sbio_ready.store(true, Relaxed);
        let init_word = if self.profile.nonce_session {
            Some(6u32)
        } else if matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. }) {
            Some(1u32)
        } else {
            None
        };
        if let Some(word) = init_word {
            let result = self.sbio_transfer_raw(
                0x73, c"INIT_SBIO_COMMUNICATION", &word.to_le_bytes(),
            );
            match result {
                Ok(done) if done.status.is_ok()
                    && (self.profile.nonce_session || done.payload.is_empty()) => {
                    dev_info!(self.dev, "sbio: communication initialization accepted\n");
                }
                Ok(done) => {
                    self.sbio_ready.store(false, Relaxed);
                    dev_err!(self.dev, "sbio: communication init refused: status {}, {} bytes\n",
                        done.status, done.payload.len());
                    return Err(EIO);
                }
                Err(e) => {
                    self.sbio_ready.store(false, Relaxed);
                    dev_err!(self.dev, "sbio: communication init transport failed {:?}; aborting Touch ID setup\n", e);
                    return Err(e);
                }
            }
        }
        Ok(())
    }

    /// Brings the sensor up, starting again after a failed attempt as
    /// `retry` allows and for as long as `wanted` says the result is still
    /// needed.
    ///
    /// A bring-up that fails on the bus leaves nothing a new attempt cannot
    /// undo: each attempt power-cycles the sensor and, after a failed one,
    /// starts the registration again as [`registration::retry_stage`] says,
    /// clearing the enclave's registration and session as every verify does
    /// and, on a profile without a nonce session, every second time from a
    /// fresh registration instead.
    fn bring_sensor_online_retrying(
        &self,
        purpose: &CStr,
        kind: Bringup,
        retry: &registration::Retry,
        wanted: impl Fn() -> bool,
    ) -> registration::Outcome {
        let mut attempts = BringupAttempts {
            sep: self,
            purpose,
            wanted,
            began_ns: shim::boottime_ns(),
            attach: kind == Bringup::Attach,
            // The late round follows the failed attach-time one.
            retry: u32::from(kind == Bringup::Late),
        };
        registration::bring_up(retry, &mut attempts)
    }

    /// One attach-time bring-up. A profile without a nonce session only
    /// wakes and patches the sensor here; its first verify calibrates it.
    fn attach_bring_sensor_online(&self) -> bool {
        if self.profile.nonce_session {
            return self.bring_sensor_online();
        }
        let bind = diag::sensor_bind();
        let Some(patch) = self.wake_sensor() else {
            dev_warn!(self.dev, "sbio: the sensor did not wake at attach\n");
            diag::sensor_brought_up(bind, false);
            return false;
        };
        if !self.bringup_may_continue(c"the sensor's completion") {
            diag::sensor_brought_up(bind, false);
            return false;
        }
        let online = self.complete_bringup(patch);
        diag::sensor_brought_up(bind, online);
        online
    }

    /// Checked between the protocol stages of one bring-up attempt: once the
    /// system has begun going to sleep or the driver is going away, the
    /// attempt stops before `next`. The attempt then counts as failed, and a
    /// bring-up whose result is no longer wanted makes no other.
    fn bringup_may_continue(&self, next: &CStr) -> bool {
        if registration::may_continue(
            self.shutting_down.load(Relaxed),
            self.suspending.load(Relaxed),
        ) {
            return true;
        }
        dev_info!(
            self.dev,
            "sensor: bring-up stopped before {}: the system is going to sleep or the driver is going away\n",
            next
        );
        false
    }

    /// Whether a bring-up that came online leaves the sensor attached: on a
    /// profile with a nonce session only once the enclave's identity view
    /// is proven too.
    fn attach_complete(&self, online: bool) -> bool {
        online && (!self.profile.nonce_session || self.device_view_synced.load(Relaxed))
    }

    /// Brings the sensor up as Touch ID is activated. Returns whether the
    /// attach fell short and a late retry should follow.
    fn attach_bringup(&self) -> bool {
        if !self.profile.nonce_session && !sensor::is_bound() {
            dev_warn!(
                self.dev,
                "sbio: no sensor bound at attach; catacomb restore ran but capture is unavailable\n"
            );
            diag::sensor_attach(registration::AttachState::NotReady);
            return false;
        }
        let outcome = self.bring_sensor_online_retrying(
            c"attach-time",
            Bringup::Attach,
            &registration::ATTACH,
            || {
                registration::may_continue(
                    self.shutting_down.load(Relaxed),
                    self.suspending.load(Relaxed),
                )
            },
        );
        let complete = self.attach_complete(outcome.online());
        diag::sensor_attach(registration::after_round(complete, true));
        if !complete {
            if self.profile.nonce_session {
                dev_warn!(
                    self.dev,
                    "sensor: authenticated attach incomplete; capture unavailable; retrying in {} s\n",
                    LATE_ATTACH_DELAY_MS / 1000
                );
            } else {
                dev_warn!(
                    self.dev,
                    "sbio: attach-time sensor bring-up did not complete; matching unavailable; retrying in {} s\n",
                    LATE_ATTACH_DELAY_MS / 1000
                );
            }
        }
        !complete
    }

    /// One more round of sensor bring-up, some seconds after an attach-time
    /// one fell short and once the boot-time bring-up is over. Without it
    /// the sensor stayed offline until the first verify, which the login
    /// screen does not make. It runs at most once a boot, waits for neither
    /// a capture nor a suspend, and ends within the attach's attempts.
    pub(crate) fn run_late_attach(this: &Arc<SepData>) {
        if this.shutting_down.load(Relaxed) {
            return;
        }
        if this.phase.load(Relaxed) != PHASE_READY || this.suspending.load(Relaxed) {
            Self::defer_late_attach(this);
            return;
        }
        // Enrol, verify and deletion drive the sensor and the enclave under
        // the same lock, and `capture_begin` lets a suspend wait for us.
        let _writer = this.catacomb_writer.lock();
        if !this.capture_begin() {
            Self::defer_late_attach(this);
            return;
        }
        if this.attach_complete(this.device_view_synced.load(Relaxed)) {
            // An enrol or a verify got there first.
            diag::sensor_attach(registration::AttachState::Attached);
            this.capture_end();
            return;
        }
        dev_info!(
            this.dev,
            "sensor: retrying the attach-time bring-up after the ready phase\n"
        );
        let outcome = this.bring_sensor_online_retrying(
            c"late attach",
            Bringup::Late,
            &registration::ATTACH,
            || !this.shutting_down.load(Relaxed) && !this.suspending.load(Relaxed),
        );
        if let registration::Outcome::Abandoned(_) = outcome {
            this.capture_end();
            Self::defer_late_attach(this);
            return;
        }
        if outcome.online() {
            this.refresh_match_credential();
        }
        let complete = this.attach_complete(outcome.online());
        diag::sensor_attach(registration::after_round(complete, false));
        if complete {
            dev_info!(this.dev, "sensor: attached after the ready phase\n");
        } else {
            dev_warn!(
                this.dev,
                "sensor: authenticated attach incomplete after the late retry; capture unavailable until an enrol or verify brings the sensor up\n"
            );
        }
        this.capture_end();
    }

    /// The late attach could not run yet; try again later, a few times.
    fn defer_late_attach(this: &Arc<SepData>) {
        if this.shutting_down.load(Relaxed) {
            return;
        }
        if this.late_attach_defers.fetch_add(1, Relaxed) < LATE_ATTACH_DEFERS {
            Self::queue_late_attach(this.clone());
            return;
        }
        dev_warn!(
            this.dev,
            "sensor: the late attach could not run; capture unavailable until an enrol or verify brings the sensor up\n"
        );
        diag::sensor_attach(registration::AttachState::NotReady);
    }

    pub(crate) fn queue_late_attach(this: Arc<SepData>) {
        let delay = time::msecs_to_jiffies(LATE_ATTACH_DELAY_MS);
        let _ = workqueue::system_dfl()
            .enqueue_delayed::<Arc<SepData>, LATE_ATTACH_WORK_ID>(this, delay);
    }

    /// Whether the activation that just ended wants a late attach. True at
    /// most once.
    pub(crate) fn take_late_attach(&self) -> bool {
        self.late_attach_wanted.xchg(false, Relaxed)
    }

    fn bring_sensor_online(&self) -> bool {
        let bind = diag::sensor_bind();
        let Some(mut patch) = self.wake_sensor() else {
            diag::sensor_brought_up(bind, false);
            return false;
        };

        // Newer sensors require the encrypted parameters and transparent
        // channel before the calibration challenge can be serviced.
        let parameters = if self.profile.nonce_session {
            match self.apply_sensor_parameters() {
                Some(parameters) => Some(parameters),
                None => return false,
            }
        } else {
            None
        };

        if !self.sensor_calibrated.load(Relaxed) {
            if !self.calibrate_sensor() {
                diag::sensor_brought_up(bind, false);
                return false;
            }
            self.sensor_calibrated.store(true, Relaxed);
            if parameters.is_none() {
                let Some(reloaded_patch) = self.wake_sensor() else {
                    diag::sensor_brought_up(bind, false);
                    return false;
                };
                patch = reloaded_patch;
            }
        }

        if !self.bringup_may_continue(c"the sensor's completion") {
            diag::sensor_brought_up(bind, false);
            return false;
        }
        let ok = match parameters {
            Some(parameters) => self.finish_sensor_setup(patch, parameters),
            None => self.complete_bringup(patch),
        };
        diag::sensor_brought_up(bind, ok);
        // The sensor is patched and idle here -- the only safe moment to
        // configure the data-ready interrupt, which is then left alone for the
        // driver's life. Interrupt capture is the default; a machine that does
        // not describe a data-ready line falls back to SPI polling.
        if ok && !sensor::irq_available() {
            if sensor::irq_setup() {
                dev_info!(self.dev, "sensor: interrupt-driven capture\n");
            } else {
                dev_warn!(
                    self.dev,
                    "sensor: no data-ready interrupt (describe it in DT); capture falls back to SPI polling\n"
                );
            }
        }
        ok
    }

    pub(crate) fn run_bringup(&self) {
        if self.bringup_started.xchg(true, Relaxed) {
            return;
        }
        if let Err(e) = self.enable_sks() {
            dev_warn!(self.dev, "bringup: could not enable the key store ({:?}); keybag and ref-key operations are unavailable\n", e);
            return;
        }
        if !self.sks_init_endpoint() {
            dev_err!(
                self.dev,
                "bringup: key-store endpoint init failed; the key store stays closed\n"
            );
            return;
        }
        diag::keystore_open();
        if *module_parameters::provision_keybag.value() != 0 {
            if *module_parameters::xart_writes.value() == 0 {
                dev_err!(
                    self.dev,
                    "bringup: provision_keybag=1 requires xart_writes=1\n"
                );
                return;
            }
            if !self.sks_provision_identity_keybag() {
                dev_err!(
                    self.dev,
                    "bringup: identity-keybag provisioning failed; no retry this boot\n"
                );
                diag::keybag(diag::Keybag::Failed);
                return;
            }
            // Either one was already persisted or one has just been.
            diag::keybag(diag::Keybag::Present);
        }
        if let Err(e) = self.register_bio() {
            dev_err!(
                self.dev,
                "bringup: could not publish /dev/{} after SKS became ready: {:?}\n",
                bio::DEVICE_NAME,
                e
            );
        }
    }

    fn activate_touchid(&self) -> Result<()> {
        if self.touchid_started.load(Relaxed) {
            return Ok(());
        }
        if self.touchid_failed.load(Relaxed) {
            return Err(EIO);
        }

        let mut store = store::Store::open().inspect_err(|e| {
            dev_err!(
                self.dev,
                "Touch ID: opening the host state failed: {:?}\n",
                e
            );
        })?;
        let index = bio::IdentityIndex::load(&mut store).inspect_err(|e| {
            dev_err!(
                self.dev,
                "Touch ID: loading the host identity index failed: {:?}\n",
                e
            );
        })?;
        *self.host_store.lock() = Some(store);
        *self.bio_index.lock() = index;

        self.attach_sensor();
        self.enable_sbio().inspect_err(|e| {
            dev_err!(
                self.dev,
                "Touch ID: registering the biometric buffers failed: {:?}\n",
                e
            );
        })?;
        let stored = match keybag::read(keybag::Slot::Identity) {
            Ok(keybag::State::Present(stored)) => stored,
            Ok(keybag::State::Absent(_)) => {
                dev_err!(self.dev, "Touch ID: no persisted identity keybag exists\n");
                diag::keybag(diag::Keybag::Missing);
                return Err(ENOENT);
            }
            Err(e) => {
                dev_err!(
                    self.dev,
                    "Touch ID: reading the persisted identity keybag failed: {:?}\n",
                    e
                );
                diag::keybag(diag::Keybag::Failed);
                return Err(e);
            }
        };
        if !self.sks_ready() {
            dev_err!(
                self.dev,
                "Touch ID: the key-store endpoint is unavailable\n"
            );
            return Err(ENODEV);
        }
        let (handle, uuid) = self.sks_recover(&stored).ok_or_else(|| {
            dev_err!(
                self.dev,
                "Touch ID: the persisted identity keybag did not recover\n"
            );
            // Only the 13.5 key store records the UUID it generated. An M2
            // Pro/Max uses it before system firmware 26.3 and variant 5 after.
            if self.profile.key_store == profile::KeyStore::Variant5
                && stored.uuid_provenance() == crate::keybag::UuidProvenance::AsGenerated
            {
                dev_err!(
                    self.dev,
                    "Touch ID: the keybag was made by the 13.5 key store, but this enclave speaks variant 5; if macOS was updated past 26.2, start Touch ID over (install-aurora-sep.sh --reset-touchid) and enrol again\n"
                );
            }
            // Only the variant-5 key store records the UUID read back from
            // the bag on a warm-registration Mac. Firmware that selects the
            // 13.5 key store for a bag variant 5 made is worth a report.
            if matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. })
                && self.profile.bootstrap == profile::Bootstrap::WarmRegister
                && stored.uuid_provenance() == crate::keybag::UuidProvenance::ReadBackFromBag
            {
                dev_err!(
                    self.dev,
                    "Touch ID: the keybag was made by the variant-5 key store, but this firmware selects the 13.5 one; please report this with the \"M2 Pro/Max on system firmware\" line\n"
                );
            }
            diag::keybag(diag::Keybag::Failed);
            EIO
        })?;
        diag::keybag(diag::Keybag::Present);
        self.sks_designate_user_keybag(handle, stored.secret());
        self.sks_machine_refkey(handle, stored.secret());
        let prepared = self.cold_match_continue(handle, uuid, stored.uuid_provenance());
        self.ensure_restored_after(prepared);
        if self.attach_bringup() {
            self.late_attach_wanted.store(true, Relaxed);
        }
        self.probe_owner_export();
        self.refresh_match_credential();
        self.touchid_started.store(true, Relaxed);
        diag::touchid(true);
        Ok(())
    }

    // The SCRD credential set up before the Catacomb restore does not survive
    // the rest of activation: J414c (variant-5 key store) and J314s (13.5 key
    // store) answered every cold-boot MATCH_RESULT with 0x1 on a usable image,
    // and a J313 showed the same symptom. Setting it up once more here was
    // enough for every match that boot, including verifies that re-register
    // the sensor with CLEAR_STATE. It runs on every profile.
    fn refresh_match_credential(&self) {
        if !self.templates_restored.load(Relaxed) {
            return;
        }
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            return;
        };
        if !self.establish_scrd_match_context(user) {
            dev_warn!(
                self.dev,
                "Touch ID: the match credential did not re-establish after the restore; the enclave may refuse matches this boot\n"
            );
        }
    }

    fn prepare_bio_open(&self) -> Result<()> {
        if let Err(e) = self.activate_touchid() {
            self.touchid_failed.store(true, Relaxed);
            diag::touchid(false);
            dev_err!(
                self.dev,
                "Touch ID activation failed: {:?}; refusing retries this boot because a partial keybag load is not safely repeatable\n",
                e
            );
            return Err(e);
        }
        Ok(())
    }

    // The attach-time bring-up can fail after the Catacomb restore (the
    // sensor patch step has no retry), which leaves the device-view proof
    // undone and every verify refused until reboot. The restore itself is
    // still a valid candidate, so bring the sensor up once more and let
    // complete_bringup() finish the proof before refusing.
    fn retry_restore_proof(&self) {
        if self.templates_restored.load(Relaxed)
            || !self.cold_restore_candidate.load(Relaxed)
            || self.device_view_synced.load(Relaxed)
        {
            return;
        }
        dev_warn!(
            self.dev,
            "verify: attach-time bring-up left the device-view proof undone; retrying the sensor bring-up\n"
        );
        if !self.bring_sensor_online() {
            // After a failed first patch, the enclave can refuse the serial
            // re-registration that follows CLEAR_STATE, while a reboot's
            // fresh registration works. Retry once from the fresh stage.
            dev_warn!(
                self.dev,
                "verify: retrying the sensor bring-up from a fresh registration\n"
            );
            self.bringup.store(BRINGUP_FRESH, Relaxed);
            if !self.bring_sensor_online() {
                dev_err!(self.dev, "verify: the retried sensor bring-up failed\n");
            }
        }
        let _ = sensor::idle();
        self.refresh_match_credential();
    }

    /// Whether the verify this work item runs is still wanted: not cancelled,
    /// not ended by a suspend and not closed.
    fn verify_wanted(&self) -> bool {
        !self.shutting_down.load(Relaxed)
            && bio::verify_is_live(&self.bio_session.lock(), self.capture_epoch.load(Relaxed))
    }

    // Every verify registers the sensor again. On a profile with a nonce
    // session that registration can fail as the attach-time one does, most
    // often the first one after a system suspend, and a new attempt recovers
    // it in the same way. Without one, the failure ends the user's
    // authentication even though the sensor works.
    fn bring_sensor_online_for_verify(&self) -> bool {
        if !self.profile.nonce_session {
            return self.bring_sensor_online();
        }
        self.bring_sensor_online_retrying(c"verify", Bringup::Verify, &registration::VERIFY, || {
            self.verify_wanted()
        })
        .online()
    }

    pub(crate) fn run_verify(&self) {
        self.retry_restore_proof();
        if !self.templates_restored.load(Relaxed) {
            dev_err!(
                self.dev,
                "verify: refusing before touching the sensor; cold-match prep, restore, or device-view identity proof did not complete. This is not a non-matching finger\n"
            );
            self.finish_verify(
                bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE),
                [0u8; bio::TOKEN_LEN],
            );
            return;
        }

        let Some(token_bytes) = self.mint_token_bytes() else {
            dev_err!(self.dev, "verify: could not draw a result token\n");
            self.finish_verify(
                bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE),
                [0u8; bio::TOKEN_LEN],
            );
            return;
        };

        self.settle_before_capture();

        // The enclave refuses a capture from an uncalibrated sensor (0x65 answers 1).
        if !self.bring_sensor_online_for_verify() {
            dev_err!(self.dev, "verify: the sensor did not come back online\n");
            self.finish_verify(bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR), token_bytes);
            let _ = sensor::idle();
            return;
        }

        // The match uses the plain 0x23->0x06->0x65 pipeline, with no SCRD step.

        let outcome = self.verify_one_image();
        let _ = sensor::idle();
        sensor::power(false);
        self.note_capture_end();

        let definitive = matches!(
            &outcome,
            bio::VerifyOutcome::Matched(_) | bio::VerifyOutcome::NoMatch
        );
        if definitive {
            let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
                self.finish_verify(
                    bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE),
                    token_bytes,
                );
                return;
            };
            if !self.save_after_match(user) {
                self.finish_verify(
                    bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE),
                    token_bytes,
                );
                return;
            }
        }
        self.finish_verify(outcome, token_bytes);
    }

    // Every exit that fails a live verify logs why: userspace sees only a bare
    // "unspecified error", which cannot tell a sensor fault from a refused
    // match step.
    fn verify_one_image(&self) -> bio::VerifyOutcome {
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            dev_err!(self.dev, "verify: invalid probe user id\n");
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };

        if let Err(e) = sensor::start_capture() {
            dev_err!(self.dev, "verify: could not start the capture ({:?})\n", e);
            return bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR);
        }
        let advertised = match self.await_capture() {
            CaptureWait::Ready(n) => n,
            CaptureWait::Timeout => {
                dev_warn!(self.dev, "verify: the capture ended without a frame\n");
                return bio::VerifyOutcome::Failed(ENROL_STATUS_TIMEOUT);
            }
            CaptureWait::Fault(state) => {
                dev_err!(
                    self.dev,
                    "verify: the sensor fell to state {} during the capture\n",
                    state
                );
                return bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR);
            }
            CaptureWait::Abandon => {
                // A cancelled verify also abandons; only a live one is a fault.
                if bio::verify_is_live(
                    &self.bio_session.lock(), self.capture_epoch.load(Relaxed),
                ) {
                    dev_err!(
                        self.dev,
                        "verify: the sensor status became unreadable during the capture\n"
                    );
                }
                return bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR);
            }
        };

        let capture = match sensor::read_capture(advertised) {
            Ok(capture) => capture,
            Err(sensor::CaptureError::Checksum {
                advertised: sent,
                computed,
            }) => {
                dev_warn!(
                    self.dev,
                    "verify: frame checksum mismatch (sensor sent {:#06x}, computed {:#06x})\n",
                    sent,
                    computed
                );
                return bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY);
            }
            Err(sensor::CaptureError::Length(n)) => {
                dev_warn!(
                    self.dev,
                    "verify: the sensor advertised an unusable frame length {}\n",
                    n
                );
                return bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY);
            }
            Err(sensor::CaptureError::Bus(e)) => {
                dev_err!(self.dev, "verify: reading the frame failed ({:?})\n", e);
                return bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR);
            }
            Err(sensor::CaptureError::NoMemory) => {
                dev_err!(self.dev, "verify: no memory for the frame\n");
                return bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR);
            }
        };

        if self
            .sbio_verify_step(&crate::sbio::sbio_prepare_image_processing())
            .is_none()
        {
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }
        let _image = ImageContext { sep: self };

        // MATCHING: byte 0x12 set, 0x13 clear; last-image flag at byte 9 (enrol byte 8)
        let init = crate::sbio::sbio_image_processing_init(
            crate::sbio::ImagePurpose::Matching,
            true,
            true,
            0,
            user,
            crate::shim::monotonic_ns(),
        );
        if self.sbio_verify_step(&init).is_none() {
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }

        let relay = crate::sbio::SbioRelay::new(&capture);
        if self.sbio_relay(&relay).is_none() {
            dev_err!(
                self.dev,
                "verify: enclave refused the capture (status above); no comparison performed, not a non-match. Reported as a failure so userspace does not tell the user their finger was rejected\n"
            );
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }
        drop(capture);

        let Some(assessment) = self.sbio_verify_step(&crate::sbio::sbio_image_assessment()) else {
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        if assessment.len() <= crate::sbio::ASSESS_USABLE_MATCH {
            dev_err!(
                self.dev,
                "verify: image assessment is {} bytes, too short to read\n",
                assessment.len()
            );
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }

        if assessment[crate::sbio::ASSESS_USABLE_MATCH] == 0 {
            dev_warn!(
                self.dev,
                "verify: the image is not usable for matching (partial contact)\n"
            );
            return bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY);
        }

        let Some(result) = self.sbio_verify_step(&crate::sbio::sbio_match_result()) else {
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        let Some(parsed) = crate::sbio::MatchResult::parse(&result) else {
            dev_err!(
                self.dev,
                "verify: the {}-byte match result did not parse\n",
                result.len()
            );
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };

        if !parsed.matches(user) {
            return bio::VerifyOutcome::NoMatch;
        }

        // match result: user id groups templates, UUID at offset 0x04 identifies one
        let identity = *parsed.identity_uuid();
        let known = self.bio_index.lock().contains_uuid(&identity);

        if !known {
            dev_err!(
                self.dev,
                "verify: the enclave matched identity {}, which the host index does not hold\n",
                Hex(&identity)
            );
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }

        bio::VerifyOutcome::Matched(bio::MatchEvidence::from_enclave_reply(identity))
    }

    pub(crate) fn run_enrolment(&self) {
        self.enrol_frames_accepted.store(0, Relaxed);

        if !self.bring_sensor_online() {
            dev_err!(self.dev, "enrol: the sensor did not come back online\n");
            self.finish_enrolment(Err(ENROL_STATUS_SENSOR));
            let _ = sensor::idle();
            return;
        }

        let Some(open) = self.begin_enrolment_on_enclave() else {
            dev_err!(self.dev, "enrol: the enclave would not start an enrolment; see the preceding step\n");
            self.finish_enrolment(Err(ENROL_STATUS_ENCLAVE));
            let _ = sensor::idle();
            return;
        };

        let mut counter: u32 = 0;
        let mut enrolment_completed = false;
        let mut enrolment_persisted = false;
        let mut identity: Option<[u8; bio::UUID_LEN]> = None;
        let mut last_percent: u32 = 0;

        let outcome = loop {
            if !bio::enrol_is_live(&self.bio_session.lock(), self.capture_epoch.load(Relaxed)) {
                break None;
            }
            if counter >= ENROL_MAX_CAPTURES {
                let accepted = self.enrol_frames_accepted.load(Relaxed);
                dev_warn!(
                    self.dev,
                    "enrol: capture budget exhausted ({} captures: {} accepted, reaching {}%, {} retried); aborting\n",
                    counter,
                    accepted,
                    last_percent,
                    counter.saturating_sub(accepted)
                );
                break Some(Err(ENROL_STATUS_TOO_MANY));
            }

            match self.enrol_one_image(counter) {
                ImageOutcome::Progress {
                    stage,
                    percent,
                    complete,
                    has_template,
                } => {
                    counter = counter.saturating_add(1);
                    last_percent = last_percent.max(percent);
                    let woke = bio::enrol_advance(
                        &mut self.bio_session.lock(),
                        self.capture_epoch.load(Relaxed),
                        stage,
                        percent,
                        bio::Guidance::LiftAndMove,
                    );
                    if woke {
                        self.bio_wake();
                    }
                    // completion is the flag at offset 0xbfe, not a derived stage count
                    if complete {
                        if !has_template {
                            dev_err!(
                                self.dev,
                                "enrol: enclave flags completion after {} capture(s) (stage {}, {}%) but reports no template\n",
                                counter,
                                stage,
                                percent
                            );
                            break Some(Err(ENROL_STATUS_ENCLAVE));
                        }
                        enrolment_completed = true;
                        break Some(Ok(()));
                    }
                }
                ImageOutcome::Retry => {
                    counter = counter.saturating_add(1);
                    // The capture did not take (partial/unusable contact). Nudge
                    // the person to hold still and press again rather than
                    // silently re-capturing with no feedback.
                    if bio::capture_guide(
                        &mut self.bio_session.lock(), self.capture_epoch.load(Relaxed),
                        bio::Guidance::HoldStill,
                    ) {
                        self.bio_wake();
                    }
                }
                ImageOutcome::NoFinger => {
                    break Some(Err(ENROL_STATUS_TIMEOUT));
                }
                ImageOutcome::Failed(status) => break Some(Err(status)),
            }

            // A spent budget ends the run at the top of the loop; don't make
            // the person wait out a reposition pause first.
            if counter < ENROL_MAX_CAPTURES {
                self.pace_between_captures();
            }
        };

        if enrolment_completed {
            open.completed();

            if let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) {
                if !self.save_all_components(user) {
                    dev_err!(
                        self.dev,
                        "enrol: enclave completed but Catacomb persistence failed; not reporting enrollment success\n"
                    );
                } else if !self.resnapshot_identity_keybag() {
                    dev_err!(
                        self.dev,
                        "enrol: Catacombs saved but identity-bag snapshot failed; not reporting enrollment success\n"
                    );
                } else {
                    self.templates_restored.store(true, Relaxed);
                    enrolment_persisted = true;
                }
            }

            identity = self.identity_just_enrolled();

            self.reconcile_identities();
        }

        let _ = sensor::idle();
        sensor::power(false);
        self.note_capture_end();

        if let Some(result) = outcome {
            let filed = match (result, identity, enrolment_persisted) {
                (Ok(()), Some(uuid), true) => Ok(uuid),
                (Ok(()), _, _) => Err(ENROL_STATUS_UNFILED),
                (Err(status), _, _) => Err(status),
            };
            self.finish_enrolment(filed);
        }
    }

    fn sensor_ms(&self) -> u64 {
        shim::boottime_ns().saturating_sub(self.sensor_power_ns.load(Relaxed)) / 1_000_000
    }

    fn sensor_trace(&self, step: &CStr) {
        if *module_parameters::sensor_trace.value() != 0 {
            dev_info!(self.dev, "sensor: {} done at {} ms\n", step, self.sensor_ms());
        }
    }

    /// Reports frames the sensor exchange had to read again or discard.
    fn note_sensor_recoveries(&self) {
        let (reread, drained) = sensor::take_recoveries();
        if reread != 0 || drained != 0 {
            dev_warn!(
                self.dev,
                "sensor: re-read {} frame(s) the sensor still offered after a garbled read, discarded {} stale frame(s)\n",
                reread,
                drained
            );
        }
    }

    /// Reports a failed registration step with what the sensor shows
    /// afterwards: its state, patch acknowledgment, whether it presents an
    /// identifier and the length of any frame it holds. Never the
    /// identifier, the serial or any key material.
    fn sensor_step_failed(&self, step: &CStr) {
        self.note_sensor_recoveries();
        let ms = self.sensor_ms();
        match sensor::status() {
            Ok(st) => dev_warn!(
                self.dev,
                "sensor: {} failed {} ms after power-on; sensor now state {} ack {:#04x} identifier {} holding {} bytes{}\n",
                step,
                ms,
                st.state,
                st.patch_ack(),
                st.identifier_to_register().is_some(),
                st.available().unwrap_or(0),
                if st.is_silent() { c" (silent)" } else { c"" }
            ),
            Err(e) => dev_warn!(
                self.dev,
                "sensor: {} failed {} ms after power-on; its status read failed too ({:?})\n",
                step,
                ms,
                e
            ),
        }
    }

    /// One enclave request of the registration. A refusal is named.
    fn sensor_enclave_step(&self, op: &crate::sbio::SbioOp) -> Option<KVec<u8>> {
        match self.sbio_transfer(op) {
            Ok(done) if done.status.is_ok() => {
                self.sensor_trace(op.name());
                Some(done.payload)
            }
            Ok(done) => {
                dev_warn!(self.dev, "sensor: {} refused with status {}\n", op.name(), done.status);
                self.sensor_step_failed(op.name());
                None
            }
            Err(e) => {
                dev_warn!(self.dev, "sensor: {} transfer failed: {:?}\n", op.name(), e);
                self.sensor_step_failed(op.name());
                None
            }
        }
    }

    fn register_sensor(&self, id: &sensor::Identifier) -> bool {
        let stage = self.bringup.load(Relaxed);
        let plan = registration::plan(
            stage,
            self.sensor_held.load(Relaxed),
            self.profile.nonce_session,
            matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. }),
        );
        dev_info!(
            self.dev,
            "sbio: registering sensor at bring-up stage {}{}\n",
            stage,
            if plan.clear_state && stage < BRINGUP_ESTABLISHED {
                c", clearing the enclave's partial registration first"
            } else {
                c""
            }
        );

        if plan.identify_first {
            // J700 needs identification before CLEAR_STATE on every reset.
            let platform = match Self::sensor_platform_identity() {
                Ok(platform) => platform,
                Err(error) => {
                    dev_err!(self.dev, "sensor: platform identity unavailable: {:?}\n", error);
                    return false;
                }
            };
            if !self.sensor_session_request(0x48, c"PLATFORM_IDENTITY", &platform) {
                self.sensor_step_failed(c"PLATFORM_IDENTITY");
                return false;
            }
            self.sensor_trace(c"PLATFORM_IDENTITY");
            // Counted as held from submission on: a reply that is lost or
            // late does not mean the enclave did not act on the request.
            self.sensor_held.store(true, Relaxed);
            if self.sensor_enclave_step(&crate::sbio::sbio_register_sensor(id)).is_none() {
                return false;
            }
        }

        if plan.clear_state {
            let _ = self.log_identity_count(c"before CLEAR_STATE for sensor re-registration");
            if self
                .sensor_enclave_step(&crate::sbio::sbio_clear_state())
                .is_none()
            {
                return false;
            }
            let _ = self.log_identity_count(c"after CLEAR_STATE for sensor re-registration");
        }

        if stage == BRINGUP_FRESH
            && matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. })
        {
            // The 13.5 J313 flow begins with REGISTER_SENSOR_SERIAL (0x17).
            // The newer REGISTER_SENSOR (0x80) returned 0x16 on this firmware.
            self.bringup.store(BRINGUP_IDENTIFIED, Relaxed);
        } else if plan.register_sensor {
            let op = crate::sbio::sbio_register_sensor(id);
            match self.sbio_call(&op) {
                SbioOutcome::Ok(_payload) => {
                    self.bringup.store(BRINGUP_IDENTIFIED, Relaxed);
                }
                _ => {
                    return false;
                }
            }
        }

        if self.profile.nonce_session {
            if !self.authenticate_sensor() {
                return false;
            }
        } else {
            let Some(serial) = self.read_serial_logged() else {
                return false;
            };
            let op = crate::sbio::sbio_register_sensor_serial(&serial);
            if self.sensor_enclave_step(&op).is_none() {
                return false;
            }
        }
        self.bringup.store(BRINGUP_ESTABLISHED, Relaxed);
        self.note_sensor_recoveries();
        let _ = self.log_identity_count(c"after sensor serial registration");
        true
    }

    fn sensor_platform_identity() -> Result<[u8; 12]> {
        let node = kernel::of::root().ok_or(ENODEV)?;
        let property: KVec<u8> = node.get_property(c"serial-number")?;
        let length = property.iter().position(|byte| *byte == 0).ok_or(EINVAL)?;
        let mut identity = [0; 12];
        if length == 0 || length > identity.len() || property.len() != length + 1 {
            return Err(EINVAL);
        }
        identity[..length].copy_from_slice(&property[..length]);
        Ok(identity)
    }

    fn sensor_session_request(&self, opcode: u16, name: &'static CStr, bytes: &[u8]) -> bool {
        match self.sbio_transfer_raw(opcode, name, bytes) {
            Ok(reply) if reply.status.is_ok() => true,
            Ok(reply) => {
                dev_err!(self.dev, "sensor: {} refused with status {}\n", name, reply.status);
                false
            }
            Err(error) => {
                dev_err!(self.dev, "sensor: {} transfer failed: {:?}\n", name, error);
                false
            }
        }
    }

    fn read_serial_logged(&self) -> Option<sensor::SensorSerial> {
        match sensor::read_sensor_serial() {
            Ok(serial) => {
                self.sensor_trace(c"serial read");
                Some(serial)
            }
            Err(e) => {
                dev_warn!(self.dev, "sensor: serial read failed: {:?}\n", e);
                self.sensor_step_failed(c"serial read");
                None
            }
        }
    }

    /// Authenticates the sensor on a profile with a nonce session, in the
    /// order [`registration::authenticate`] gives.
    fn authenticate_sensor(&self) -> bool {
        let mut steps = SensorAuthentication {
            sep: self,
            refused16: false,
        };
        let mut unfinished = self.serial_unfinished.load(Relaxed);
        let result = registration::authenticate(&mut steps, &mut unfinished);
        self.serial_unfinished.store(unfinished, Relaxed);
        match result {
            Ok(registration::Authenticated::Registered) => true,
            Ok(registration::Authenticated::Resumed) => {
                dev_info!(
                    self.dev,
                    "sensor: REGISTER_SERIAL answered 0x16 while the enclave held the unfinished registration of an earlier attempt; the session opened on that one\n"
                );
                true
            }
            Err(registration::Stopped::RegisterSerial) if steps.refused16 => {
                dev_warn!(self.dev, "sensor: REGISTER_SERIAL refused with status 0x16\n");
                self.sensor_step_failed(c"REGISTER_SERIAL");
                false
            }
            // Each step has said why it failed.
            Err(_) => false,
        }
    }

    fn load_sensor_root_key(&self) -> bool {
        let policy = self.profile.sensor_root_key;
        let Some(name) = policy.firmware() else {
            return false;
        };
        let firmware = match kernel::firmware::Firmware::request(name, &self.dev) {
            Ok(firmware) => firmware,
            Err(error) => {
                dev_err!(
                    self.dev,
                    "sensor: root-key firmware {} unavailable: {:?}\n",
                    name,
                    error
                );
                return false;
            }
        };
        if !policy.accepts_length(firmware.data().len()) {
            dev_err!(
                self.dev,
                "sensor: root-key firmware {} has unsupported length {}\n",
                name,
                firmware.data().len()
            );
            self.sensor_step_failed(c"SENSOR_ROOT_KEY");
            return false;
        }
        if !self.sensor_session_request(0x86, c"SENSOR_ROOT_KEY", firmware.data()) {
            self.sensor_step_failed(c"SENSOR_ROOT_KEY");
            return false;
        }
        self.sensor_key_loaded.store(true, Relaxed);
        self.sensor_trace(c"SENSOR_ROOT_KEY");
        true
    }

    /// SENSOR_NONCE, the board's root-key policy, and the session share.
    fn sensor_nonce_session(&self, nonce: &[u8; 32]) -> bool {
        if *module_parameters::sensor_test_fault.value() != 0
            && !self.sensor_test_fault_done.xchg(true, Relaxed)
        {
            dev_warn!(
                self.dev,
                "sensor: sensor_test_fault: stopping before SENSOR_NONCE, so that the enclave holds an unfinished registration\n"
            );
            return false;
        }
        if !self.sensor_session_request(0x81, c"SENSOR_NONCE", nonce) {
            self.sensor_step_failed(c"SENSOR_NONCE");
            return false;
        }
        self.sensor_trace(c"SENSOR_NONCE");

        if self.profile.sensor_root_key == profile::SensorRootKey::J700
            && !self.sensor_key_loaded.load(Relaxed)
            && !self.load_sensor_root_key()
        {
            return false;
        }
        // Each identification starts a new sensor-side exchange, even though
        // the signed key remains loaded for the lifetime of the enclave.
        self.establish_session()
    }

    fn wake_sensor(&self) -> Option<PatchLoaded> {
        let _source = sensor::power_source();
        self.sensor_power_ns.store(shim::boottime_ns(), Relaxed);
        let cycled = sensor::power_cycle();
        if !cycled && self.profile.nonce_session {
            // A sensor that was not reset may still hold the session of an
            // earlier attempt; registering it again cannot be trusted.
            dev_err!(self.dev, "sensor: the power cycle failed; not registering the sensor\n");
            return None;
        }
        if !sensor::cs_timing().is_hardware() {
            dev_warn!(
                self.dev,
                "sensor: chip-select timing not reaching hardware; software emulation cannot time this sensor, silent status expected (mode 2 is the other half)\n"
            );
        }

        let mut answered_without_identifier = false;

        for delay in sensor::POWER_ON_READ_DELAYS_MS {
            if delay > 0 {
                kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(delay)));
            }
            let st = match sensor::status() {
                Ok(st) => st,
                Err(_) => {
                    dev_warn!(self.dev, "sensor: status read failed on wake\n");
                    return None;
                }
            };
            if st.is_silent() {
                continue;
            }

            if let sensor::Offset12::Identifier(id) = st.offset12() {
                if id != 0 && !sensor::KNOWN_IDENTIFIERS.contains(&id) {
                    dev_err!(
                        self.dev,
                        "sensor: unknown Mesa identifier {:#06x} at status offset 12; not registering it with the enclave\n",
                        id
                    );
                    return None;
                }
            }

            let Some(id) = st.identifier_to_register() else {
                answered_without_identifier = true;
                continue;
            };
            if *module_parameters::sensor_trace.value() != 0 {
                dev_info!(
                    self.dev,
                    "sensor: identifier present {} ms after power-on\n",
                    self.sensor_ms()
                );
            }
            if !self.register_sensor(&id) {
                dev_warn!(self.dev, "sensor: the enclave did not register the sensor\n");
                return None;
            }
            if !self.bringup_may_continue(c"the post-registration status read and the patch") {
                return None;
            }

            // Read the status again before anything else goes to the sensor,
            // on every profile. Registration can reset an authenticated
            // session, so the acknowledgement sampled before it is stale; and
            // on a T6000 the session share that followed a registration
            // without this read was dropped or answered wrongly on 4 boots in
            // 15, while the kernels that made it never failed. See
            // registration::patch_loaded for what decides.
            let st = match sensor::status() {
                Ok(st) if !st.is_silent() => st,
                Ok(_) => {
                    dev_warn!(
                        self.dev,
                        "sensor: status unavailable after registration: silent {} ms after power-on\n",
                        self.sensor_ms()
                    );
                    return None;
                }
                Err(e) => {
                    dev_warn!(
                        self.dev,
                        "sensor: status unavailable after registration: read failed ({:?}) {} ms after power-on\n",
                        e,
                        self.sensor_ms()
                    );
                    return None;
                }
            };
            if registration::patch_loaded(
                self.profile.nonce_session,
                st.patch_ack() == sensor::PATCH_ACCEPTED,
                st.state == sensor::STATE_NEEDS_PATCH,
            ) {
                return Some(PatchLoaded(()));
            }
            return self.apply_sensor_patch(st.patch_ack());
        }

        dev_err!(
            self.dev,
            "sensor gated: all {} wake reads returned sixteen zero bytes. Power {}. CS timing: {}. Mode 2 (CPOL=1 CPHA=0)\n",
            sensor::POWER_ON_READ_DELAYS_MS.len(),
            if cycled { c"was cycled" } else { c"was NOT cycled" },
            sensor::cs_timing().name()
        );
        if answered_without_identifier {
            return None;
        }
        None
    }

    fn calibrate_sensor(&self) -> bool {
        // The blob is per-device factory data, so its name is board data: take
        // it from the sensor node's firmware-name, falling back to the default.
        let name = sensor::firmware_name().unwrap_or(CALIBRATION_FIRMWARE);
        let blob = match kernel::firmware::Firmware::request(name, &self.dev) {
            Ok(fw) => CalibrationBlob::new(fw),
            Err(e) => {
                dev_err!(
                    self.dev,
                    "sensor: calibration blob {} could not be loaded ({:?}); per-device factory data, cannot be synthesised, stopping\n",
                    name,
                    e
                );
                return false;
            }
        };

        let Some(challenge) = self.sbio_expect_ok(&crate::sbio::sbio_module_challenge()) else {
            return false;
        };
        if challenge.len() != crate::sbio::SBIO_MODULE_CHALLENGE_LEN {
            return false;
        }

        let challenge = sensor::Params::new(challenge);
        if sensor::send_module_challenge(&challenge).is_err() {
            return false;
        }

        let reply = match sensor::read_module_reply() {
            Ok(reply) => reply,
            Err(_) => {
                return false;
            }
        };

        // commit the sensor's reply, not the challenge; both are 64 bytes at count 0x4b
        let Some(commit) = crate::sbio::sbio_module_commit(&reply) else {
            return false;
        };
        let Some(_serial) = self.sbio_expect_ok(&commit) else {
            return false;
        };

        let source = if self.profile.nonce_session { 6 } else { SBIO_CALIBRATION_SOURCE };
        let request = match crate::sbio::SbioCalibration::new(&blob, source) {
            Ok(request) => request,
            Err(_) => {
                return false;
            }
        };
        if !self.sensor_session_request(request.opcode(), request.name(), request.payload()) {
            dev_err!(self.dev, "sensor: calibration refused (source {}, {} bytes)\n",
                source, request.payload().len());
            return false;
        }

        let _ = self.sbio_call(&crate::sbio::sbio_diagnostics());

        true
    }

    fn complete_bringup(&self, patch: PatchLoaded) -> bool {
        let Some(params) = self.apply_sensor_parameters() else {
            return false;
        };

        self.finish_sensor_setup(patch, params)
    }

    fn finish_sensor_setup(&self, patch: PatchLoaded, params: ParametersApplied) -> bool {
        let op = crate::sbio::sbio_complete_init(patch, params);
        if self.sbio_expect_ok(&op).is_none() {
            dev_err!(
                self.dev,
                "sensor: 0x01 COMPLETE_INIT failed — status above\n"
            );
            return false;
        }

        let listed = self.sync_device_view();

        if listed && !self.device_view_synced.xchg(true, Relaxed) {
            let count = self.log_identity_count(c"after device-view synchronisation");
            dev_info!(self.dev, "sbio: device-view identity count {:?}, restore-ready before proof {}\n", count, self.templates_restored.load(Relaxed));
            match count {
                Some(0) => {
                    self.templates_restored.store(false, Relaxed);
                    diag::restore(diag::Restore::Empty);
                }
                Some(n) if (self.templates_restored.load(Relaxed)
                    || self.cold_restore_candidate.load(Relaxed)) && self.prove_restore() => {
                    self.templates_restored.store(true, Relaxed);
                    diag::restore(diag::Restore::Restored);
                    dev_info!(
                        self.dev,
                        "Touch ID: restored {} enrolled identity/identities; enrolment survived reboot\n",
                        n
                    );
                }
                Some(_) => {
                    self.templates_restored.store(false, Relaxed);
                    diag::restore(diag::Restore::Unproven);
                }
                None => {
                    self.templates_restored.store(false, Relaxed);
                    diag::restore(diag::Restore::Unproven);
                }
            }
            dev_info!(self.dev, "sbio: restore-ready after device-view proof {}\n", self.templates_restored.load(Relaxed));
            if self.templates_restored.load(Relaxed) {
                self.reconcile_identities();
            }
        } else if !listed {
            dev_warn!(self.dev, "sbio: device-view synchronisation or match policy unavailable\n");
        }

        true
    }

    fn apply_sensor_parameters(&self) -> Option<ParametersApplied> {
        // 0x5d and 0x5c: an empty blob is a failure.
        self.relay_parameters(
            &crate::sbio::sbio_coverage_params(),
            &sensor::Geometry::COVERAGE,
            false,
        )?;
        self.relay_parameters(
            &crate::sbio::sbio_operation_params(),
            &sensor::Geometry::OPERATION,
            false,
        )?;
        // 0x6a: an empty blob is success, and relaying nothing is correct.
        self.relay_parameters(
            &crate::sbio::sbio_transparent_channel(),
            &sensor::Geometry::TRANSPARENT,
            true,
        )?;

        Some(ParametersApplied(()))
    }

    fn relay_parameters(
        &self,
        op: &crate::sbio::SbioOp,
        geom: &sensor::Geometry,
        empty_is_success: bool,
    ) -> Option<usize> {
        let blob = match self.sbio_call(op) {
            SbioOutcome::Ok(b) => sensor::Params::new(b),
            _ => {
                return None;
            }
        };

        if blob.is_empty() {
            if empty_is_success {
                return Some(0);
            }
            return None;
        }

        let relayed = blob.len();
        match sensor::send_encrypted_parameters(&blob, geom) {
            Ok(()) => Some(relayed),
            Err(sensor::ParamsError::Empty) => None,
            Err(sensor::ParamsError::TooLong(n, capacity)) => {
                dev_err!(
                    self.dev,
                    "sensor: {}'s blob is {} bytes and this frame holds {} (declared 0x{:x} less the 9 bytes of header and CRC)\n",
                    op.name(),
                    n,
                    capacity,
                    geom.declared()
                );
                None
            }
            Err(sensor::ParamsError::Transfer(_e)) => None,
        }
    }

    fn establish_session(&self) -> bool {
        let op = crate::sbio::sbio_request_session_share();
        let done = match self.sbio_transfer(&op) {
            Ok(done) => done,
            Err(error) => {
                dev_warn!(self.dev, "sensor: SESSION_SHARE transfer failed: {:?}\n", error);
                self.sensor_step_failed(c"SESSION_SHARE");
                return false;
            }
        };
        let share = if done.status.is_ok() {
            self.sensor_trace(c"SESSION_SHARE");
            done.payload
        } else if self
            .profile
            .sensor_root_key
            .recover_share(done.status.answered())
        {
            dev_info!(self.dev, "sensor: SESSION_SHARE status 0x16; loading FDR MSRk\n");
            if !self.load_sensor_root_key() {
                return false;
            }
            // One retry only: this step does not enter the recovery path again.
            let Some(share) = self.sensor_enclave_step(&op) else {
                return false;
            };
            share
        } else {
            dev_warn!(self.dev, "sensor: SESSION_SHARE refused with status {}\n", done.status);
            self.sensor_step_failed(c"SESSION_SHARE");
            return false;
        };

        // 0x30 reserved for this reply; length is a floor, take the first 40
        if share.len() < sensor::SESSION_SHARE_LEN {
            dev_warn!(self.dev, "sensor: short session share ({} bytes)\n", share.len());
            return false;
        }
        let mut out = [0u8; sensor::SESSION_SHARE_LEN];
        out.copy_from_slice(&share[..sensor::SESSION_SHARE_LEN]);

        // relay it: class 0x72, no CRC, no padding
        if let Err(e) = sensor::send_session_share(&out) {
            dev_warn!(self.dev, "sensor: sending the session share failed: {:?}\n", e);
            self.sensor_step_failed(c"session share send");
            return false;
        }

        let reply = match sensor::read_session_reply() {
            Ok(r) => r,
            Err(e) => {
                dev_warn!(self.dev, "sensor: session reply read failed: {:?}\n", e);
                self.sensor_step_failed(c"session reply read");
                return false;
            }
        };

        self.sensor_enclave_step(&crate::sbio::sbio_commit_session_share(&reply))
            .is_some()
    }

    // Each failure names its step: a report that only says "sequence counter
    // init failed" cannot tell the enclave side from the sensor side.
    fn init_sequence_counter(&self) -> bool {
        let challenge = match self.sbio_call(&crate::sbio::sbio_request_challenge()) {
            SbioOutcome::Ok(c) => c,
            SbioOutcome::PrerequisiteMissing => {
                if !self.establish_session() {
                    dev_warn!(self.dev, "sequence counter: session setup failed\n");
                    return false;
                }
                match self.sbio_call(&crate::sbio::sbio_request_challenge()) {
                    SbioOutcome::Ok(c) => c,
                    _ => {
                        dev_warn!(self.dev, "sequence counter: challenge request failed after session setup\n");
                        return false;
                    }
                }
            }
            SbioOutcome::Status16 => {
                dev_warn!(self.dev, "sequence counter: challenge request answered status 16\n");
                return false;
            }
            SbioOutcome::Other => {
                dev_warn!(self.dev, "sequence counter: challenge request failed\n");
                return false;
            }
        };
        if challenge.len() < sensor::CHALLENGE_LEN {
            dev_warn!(self.dev, "sequence counter: short challenge ({} bytes)\n", challenge.len());
            return false;
        }
        let mut out = [0u8; sensor::CHALLENGE_LEN];
        out.copy_from_slice(&challenge[..sensor::CHALLENGE_LEN]);

        if sensor::send_challenge(&out).is_err() {
            dev_warn!(self.dev, "sequence counter: sending the challenge to the sensor failed\n");
            return false;
        }

        let reply = match sensor::read_challenge_reply() {
            Ok(r) => r,
            Err(_) => {
                dev_warn!(self.dev, "sequence counter: the sensor did not answer the challenge\n");
                return false;
            }
        };

        match self.sbio_transfer(&crate::sbio::sbio_commit_challenge(&reply)) {
            Ok(done) if done.status.is_ok() => true,
            Ok(done) => {
                dev_warn!(self.dev, "sequence counter: the enclave refused the sensor's reply, status {}\n", done.status);
                self.sensor_step_failed(c"challenge commit");
                false
            }
            Err(e) => {
                dev_warn!(self.dev, "sequence counter: committing the reply failed {:?}\n", e);
                false
            }
        }
    }

    fn apply_sensor_patch(&self, _ack_before: u8) -> Option<PatchLoaded> {
        let blob = match self.sbio_call(&crate::sbio::sbio_fetch_patch()) {
            SbioOutcome::Ok(b) => b,
            SbioOutcome::PrerequisiteMissing => {
                if !self.init_sequence_counter() {
                    dev_warn!(
                        self.dev,
                        "sensor patch: sequence counter init failed before the patch fetch\n"
                    );
                    return None;
                }
                match self.sbio_call(&crate::sbio::sbio_fetch_patch()) {
                    SbioOutcome::Ok(b) => b,
                    _ => {
                        dev_warn!(
                            self.dev,
                            "sensor patch: patch fetch failed after the sequence counter init\n"
                        );
                        return None;
                    }
                }
            }
            SbioOutcome::Status16 => {
                dev_warn!(self.dev, "sensor patch: patch fetch answered status 16\n");
                return None;
            }
            SbioOutcome::Other => {
                dev_warn!(self.dev, "sensor patch: patch fetch failed\n");
                return None;
            }
        };
        if blob.is_empty() {
            dev_warn!(self.dev, "sensor patch: the enclave returned an empty patch\n");
            return None;
        }

        // enable command is mandatory; skipping it leaves the sensor in state 9
        if sensor::setup_patch_enable().is_err() {
            dev_warn!(self.dev, "sensor patch: the patch enable command failed on the bus\n");
            return None;
        }

        if !self.await_sensor_state(sensor::STATE_IDLE, c"idle, before sending the patch") {
            dev_warn!(self.dev, "sensor patch: the sensor did not reach idle before the patch\n");
            return None;
        }

        if sensor::send_patch(&blob).is_err() {
            dev_warn!(self.dev, "sensor patch: sending the patch failed on the bus\n");
            return None;
        }

        // acceptance is at status[8], not at the state
        for _attempt in 0..PATCH_POLL_ATTEMPTS {
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(PATCH_POLL_MS)));
            let st = match sensor::status() {
                Ok(st) => st,
                Err(_) => {
                    dev_warn!(
                        self.dev,
                        "sensor patch: status read failed while waiting for the acknowledgment\n"
                    );
                    return None;
                }
            };
            if st.patch_ack() == sensor::PATCH_ACCEPTED {
                if let Ok(after) = sensor::status() {
                    if after.state == sensor::STATE_NEEDS_PATCH {
                        dev_warn!(
                            self.dev,
                            "sensor patch: acknowledged, but the sensor still asks for a patch\n"
                        );
                        return None;
                    }
                }
                return Some(PatchLoaded(()));
            }
        }

        let _ = sensor::status();
        dev_warn!(
            self.dev,
            "sensor patch: no acknowledgment after {} polls\n",
            PATCH_POLL_ATTEMPTS
        );
        None
    }

    fn await_sensor_state(&self, want: u8, _why: &CStr) -> bool {
        for _attempt in 0..PATCH_POLL_ATTEMPTS {
            match sensor::status() {
                Ok(st) if st.state == want => {
                    return true;
                }
                Ok(_) => {}
                Err(_) => {
                    return false;
                }
            }
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(PATCH_POLL_MS)));
        }
        false
    }

    // Like verify, every exit that ends an enrolment says why: userspace sees
    // only "driver status 0x1", which cannot tell these apart.
    fn enrol_one_image(&self, counter: u32) -> ImageOutcome {
        if let Err(e) = sensor::start_capture() {
            dev_err!(self.dev, "enrol: could not start capture {} ({:?})\n", counter, e);
            return ImageOutcome::Failed(ENROL_STATUS_SENSOR);
        }

        let available = match self.await_capture() {
            CaptureWait::Ready(n) => n,
            CaptureWait::Timeout => {
                dev_warn!(self.dev, "enrol: capture {} ended without a frame\n", counter);
                return ImageOutcome::NoFinger;
            }
            CaptureWait::Fault(state) => {
                dev_err!(
                    self.dev,
                    "enrol: the sensor fell to state {} during capture {}\n",
                    state,
                    counter
                );
                return ImageOutcome::Failed(ENROL_STATUS_SENSOR);
            }
            CaptureWait::Abandon => {
                // A cancelled enrolment also abandons; only a live one is a fault.
                if bio::enrol_is_live(&self.bio_session.lock(), self.capture_epoch.load(Relaxed)) {
                    dev_err!(
                        self.dev,
                        "enrol: the sensor status became unreadable during capture {}\n",
                        counter
                    );
                }
                return ImageOutcome::Failed(ENROL_STATUS_SENSOR);
            }
        };

        let capture = match sensor::read_capture(available) {
            Ok(c) => c,
            Err(sensor::CaptureError::Checksum {
                advertised: sent,
                computed,
            }) => {
                dev_warn!(
                    self.dev,
                    "enrol: capture {} frame checksum mismatch (sensor sent {:#06x}, computed {:#06x}); retrying\n",
                    counter,
                    sent,
                    computed
                );
                return ImageOutcome::Retry;
            }
            Err(sensor::CaptureError::Length(n)) => {
                dev_warn!(
                    self.dev,
                    "enrol: capture {} advertised an unusable frame length {}; retrying\n",
                    counter,
                    n
                );
                return ImageOutcome::Retry;
            }
            Err(sensor::CaptureError::Bus(e)) => {
                dev_err!(self.dev, "enrol: reading the frame of capture {} failed ({:?})\n", counter, e);
                return ImageOutcome::Failed(ENROL_STATUS_SENSOR);
            }
            Err(sensor::CaptureError::NoMemory) => {
                dev_err!(self.dev, "enrol: no memory for the frame of capture {}\n", counter);
                return ImageOutcome::Failed(ENROL_STATUS_SENSOR);
            }
        };

        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            dev_err!(self.dev, "enrol: invalid user id before image processing\n");
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        if self
            .sbio_enrol_step(&crate::sbio::sbio_prepare_image_processing())
            .is_none()
        {
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }
        let _image = ImageContext { sep: self };

        let init = crate::sbio::sbio_image_processing_init(
            crate::sbio::ImagePurpose::Enrolment,
            counter == 0,
            false,
            counter,
            user,
            crate::shim::monotonic_ns(),
        );
        if self.sbio_enrol_step(&init).is_none() {
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }

        let relay = crate::sbio::SbioRelay::new(&capture);
        if self.sbio_relay(&relay).is_none() {
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }
        drop(capture);

        let Some(assessment) = self.sbio_enrol_step(&crate::sbio::sbio_image_assessment()) else {
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        if assessment.len() < crate::sbio::ASSESS_MIN_LEN {
            dev_err!(self.dev, "enrol: image assessment short ({} bytes)\n", assessment.len());
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }
        let usable = assessment[crate::sbio::ASSESS_USABLE_ENROL] != 0;
        if !usable {
            return ImageOutcome::Retry;
        }

        let Some(result) = self.sbio_enrol_step(&crate::sbio::sbio_enrolment_result()) else {
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        let Some(parsed) = crate::sbio::EnrolmentResult::parse(&result) else {
            dev_err!(self.dev, "enrol: malformed enrolment result ({} bytes)\n", result.len());
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };

        if parsed.complete() {
            self.stash_enrol_identity(&result);
        }

        let percent = parsed.progress_percent();
        // Coverage can stop changing while SEP continues accepting frames.
        // Count accepted captures so each one remains visible to libfprint.
        let accepted = self.enrol_frames_accepted.fetch_add(1, Relaxed) + 1;
        let stage = if parsed.complete() {
            bio::ENROL_STAGES
        } else {
            accepted.min(bio::ENROL_STAGES - 1)
        };

        ImageOutcome::Progress {
            stage,
            percent,
            complete: parsed.complete(),
            has_template: parsed.has_template(),
        }
    }

    fn await_capture(&self) -> CaptureWait {
        let mut previous: Option<[u8; sensor::STATUS_LEN]> = None;
        let mut armed_reported = false;

        // Interrupt-driven capture blocks on the data-ready line instead of
        // polling; the SPI status read below still gates every frame, so a
        // missed or spurious interrupt costs at most one backstop interval. A
        // machine with no data-ready line falls back to SPI polling.
        let use_irq = sensor::irq_available();
        if use_irq {
            sensor::irq_arm();
        }

        // Bound the wait by wall-clock time, not a fixed iteration count. The
        // data-ready interrupt can wake early on a stray edge (the sensor
        // toggles the line as the finger moves between enrol frames); a fixed
        // attempt budget would then be spent in milliseconds and time the
        // capture out before the frame lands. Time-bounding degrades, at worst,
        // to the same duration as the poll path.
        let deadline_ns = crate::shim::monotonic_ns().saturating_add(
            u64::from(ENROL_POLL_ATTEMPTS) * u64::from(ENROL_POLL_MS) * 1_000_000,
        );
        let mut attempt: u32 = 0;
        loop {
            if !bio::capture_is_live(&self.bio_session.lock(), self.capture_epoch.load(Relaxed)) {
                return CaptureWait::Abandon;
            }

            let st = match sensor::status() {
                Ok(st) => st,
                Err(_) => {
                    return CaptureWait::Abandon;
                }
            };

            if attempt == 0 || previous != Some(st.raw) {
                previous = Some(st.raw);
            }

            if st.state == sensor::STATE_NEEDS_PATCH {
                return CaptureWait::Fault(st.state);
            }

            let guide = match st.state {
                sensor::STATE_ARMED => Some(bio::Guidance::Place),
                sensor::STATE_READING => Some(bio::Guidance::HoldStill),
                _ => None,
            };
            if let Some(guide) = guide {
                if bio::capture_guide(
                    &mut self.bio_session.lock(), self.capture_epoch.load(Relaxed), guide,
                ) {
                    self.bio_wake();
                }
            }

            if st.state == sensor::STATE_ARMED && !armed_reported {
                armed_reported = true;
            }

            if let sensor::Offset12::Available(count) = st.offset12() {
                if count == 0 {
                    return CaptureWait::Timeout;
                }
                return CaptureWait::Ready(count);
            }

            if crate::shim::monotonic_ns() >= deadline_ns {
                break;
            }

            if use_irq {
                // Block on the data-ready line; the interrupt wakes this the
                // instant a frame is ready, and the backstop bounds a missed
                // edge. Re-arm for the next frame.
                let _ = sensor::irq_wait(ENROL_IRQ_WAIT_MS);
                sensor::irq_arm();
            } else {
                kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(ENROL_POLL_MS)));
            }
            attempt = attempt.saturating_add(1);
        }
        let _ = sensor::status();
        CaptureWait::Timeout
    }

    pub(crate) fn on_sbio(&self, msg: Message) {
        let f = proto::decode(&msg);
        let marker = f.tag;

        if !self.sbio_ready.load(Relaxed) {
            return;
        }
        // Before reading the reply: a requester that times out meanwhile and
        // begins a new exchange must not receive it.
        let generation = self.sbio_rx.lock().generation();

        // notification and the enclave's DMA are not ordered; buffer may still be poison
        let packet = match transfer::take_header(
            &SbioHeader(self),
            marker,
            OOL_POISON_OUTBOUND,
            OOL_WRITE_POLL_ATTEMPTS,
        ) {
            Ok(p) => p,
            Err(status) => {
                self.sbio_fail_with(status);
                return;
            }
        };

        let chunk = if transfer::carries_payload(marker) {
            packet.chunk as usize
        } else {
            0
        };
        let payload = if chunk == 0 {
            KVec::new()
        } else if chunk > OOL_SIZE_SBIO - transfer::HEADER_LEN {
            self.sbio_fail_with(transfer::DeviceStatus::Malformed);
            return;
        } else if !self.ool_await_written(&self.ool_sbio, transfer::HEADER_LEN, chunk) {
            self.sbio_fail_with(transfer::DeviceStatus::BufferNeverWritten);
            return;
        } else {
            match self.ool_read(&self.ool_sbio, transfer::HEADER_LEN, chunk) {
                Ok(p) => p,
                Err(_) => {
                    self.sbio_fail();
                    return;
                }
            }
        };

        let progress = self.sbio_rx.lock().on_chunk(generation, marker, &packet, &payload);

        match progress {
            transfer::Progress::NeedMore(cont) => {
                self.sbio_continue(&cont);
            }
            transfer::Progress::Complete => {
                // 0xFE requests the peer's next packet and acks the final one; nothing to send
                self.sbio_wq.notify_all();
            }
            transfer::Progress::Ignored => {}
            transfer::Progress::Grant => {
                self.sbio_wq.notify_all();
            }
            transfer::Progress::Notification => {}
            transfer::Progress::Failed => {
                self.sbio_wq.notify_all();
            }
        }
    }

    fn sbio_continue(&self, cont: &transfer::Continuation) {
        let mut header = [0u8; transfer::HEADER_LEN];
        if cont.packet().encode(&mut header).is_err() {
            self.sbio_fail();
            return;
        }
        if self.ool_write(&self.ool_sbio, 0, &header).is_err() {
            self.sbio_fail();
            return;
        }
        if self.send(crate::sbio::encode_sbio_continue(cont)).is_err() {
            self.sbio_fail();
        }
    }

    fn sbio_fail(&self) {
        self.sbio_rx.lock().abort();
        self.sbio_wq.notify_all();
    }

    fn sbio_fail_with(&self, status: transfer::DeviceStatus) {
        self.sbio_rx.lock().abort_with(status);
        self.sbio_wq.notify_all();
    }

    fn sbio_transfer(&self, op: &crate::sbio::SbioOp) -> Result<transfer::Completed> {
        self.sbio_transfer_raw(op.opcode(), op.name(), op.payload())
    }

    fn sbio_transfer_raw(
        &self,
        opcode: u16,
        name: &'static CStr,
        payload: &[u8],
    ) -> Result<transfer::Completed> {
        // The capture work item and the ioctl path both reach this endpoint.
        // Without this, the second caller is refused with EBUSY, or clears a
        // reply the first has not collected yet and takes the next one. Taken
        // before the shutdown check, so a waiter never starts after remove().
        let _exchange = self.sbio_exchange_lock.lock();
        if self.shutting_down.load(Relaxed) || !self.sbio_ready.load(Relaxed) {
            return Err(ENODEV);
        }

        self.sbio_rx.lock().begin(u32::from(opcode))?;

        if let Err(e) = self.sbio_start(opcode, name, payload) {
            let mut guard = self.sbio_rx.lock();
            guard.abort();
            let _ = guard.take_done();
            return Err(e);
        }

        let mut remaining = time::msecs_to_jiffies(SBIO_TIMEOUT_MS);
        let mut guard = self.sbio_rx.lock();
        loop {
            if self.shutting_down.load(Relaxed) {
                guard.abort();
                let _ = guard.take_done();
                return Err(ENODEV);
            }
            if let Some(done) = guard.take_done() {
                return Ok(done);
            }
            if remaining == 0 {
                guard.abort();
                let _ = guard.take_done();
                drop(guard);
                return Err(ETIMEDOUT);
            }
            match self
                .sbio_wq
                .wait_interruptible_timeout(&mut guard, remaining)
            {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        }
    }

    fn sbio_start(&self, opcode: u16, name: &'static CStr, payload: &[u8]) -> Result<()> {
        if crate::sbio::encode_sbio_raw(opcode, 0).is_none() {
            return Err(EPERM);
        }

        let total = payload.len();
        if total > transfer::MAX_TRANSACTION as usize {
            return Err(EINVAL);
        }

        let capacity = OOL_SIZE_SBIO - transfer::HEADER_LEN;

        self.sbio_rx.lock().begin_send(u32::from(opcode));
        let result = self.sbio_send_chunks(opcode, name, payload, capacity);
        self.sbio_rx.lock().end_send();
        result
    }

    fn sbio_send_chunks(
        &self,
        opcode: u16,
        name: &'static CStr,
        payload: &[u8],
        capacity: usize,
    ) -> Result<()> {
        let total = payload.len();
        let mut offset = 0usize;
        let mut first = true;
        let mut seq: u16 = 0;
        let mut request = KVec::new();

        loop {
            let chunk = core::cmp::min(total - offset, capacity);

            let packet = transfer::Packet {
                version: 1,
                total: total as u32,
                offset: offset as u32,
                flags: 0,
                err: 0,
                opcode: u32::from(opcode),
                chunk: chunk as u32,
            };

            request.clear();
            request.resize(transfer::HEADER_LEN + chunk, 0u8, GFP_KERNEL)?;
            packet.encode(&mut request)?;
            request[transfer::HEADER_LEN..].copy_from_slice(&payload[offset..offset + chunk]);

            self.ool_write(&self.ool_sbio, 0, &request)?;

            let msg = if first {
                crate::sbio::encode_sbio_raw(opcode, seq)
            } else {
                crate::sbio::encode_sbio_next(opcode, seq)
            }
            .ok_or(EPERM)?;
            // The last grant's header may sit in the outbound buffer; the
            // reply to this chunk must not take it.
            transfer::send_for_reply(&SbioHeader(self), || self.send(msg))?;

            offset += chunk;
            first = false;
            seq = seq.wrapping_add(1);
            if offset >= total {
                return Ok(());
            }

            self.sbio_await_grant(name, offset, total)?;
        }
    }

    fn sbio_await_grant(&self, _name: &'static CStr, _sent: usize, _total: usize) -> Result<()> {
        let mut remaining = time::msecs_to_jiffies(SBIO_TIMEOUT_MS);
        let mut guard = self.sbio_rx.lock();
        loop {
            if self.shutting_down.load(Relaxed) {
                return Err(ENODEV);
            }
            if guard.take_grant() {
                return Ok(());
            }
            if remaining == 0 {
                drop(guard);
                return Err(ETIMEDOUT);
            }
            match self
                .sbio_wq
                .wait_interruptible_timeout(&mut guard, remaining)
            {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        }
    }

    pub(crate) fn register_bio(&self) -> Result<()> {
        let mut guard = self.bio_dev.lock();
        if guard.is_some() {
            return Ok(());
        }

        let ctx = core::ptr::from_ref(self).cast_mut().cast::<c_void>();

        // SAFETY: `ctx` is this `SepData`, kept alive by the `Arc` in the
        // driver's private data. `remove()` drops the registration before that
        // `Arc` can go, and the shim's unregister clears its stored context
        // under the same lock every callback takes, so once it returns, no
        // callback can observe this pointer again.
        let dev = unsafe {
            shim::BioChardev::register(
                bio::DEVICE_NAME,
                bio::DEVICE_MODE,
                ctx,
                bio_open_trampoline,
                bio_release_trampoline,
                bio_ioctl_trampoline,
                bio_ready_trampoline,
            )
        }?;

        *guard = Some(dev);
        diag::bio_published();
        Ok(())
    }

    pub(crate) fn bio_open(&self) -> Result<()> {
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }
        let mut session = self.bio_session.lock();
        self.wait_for_worker_exit(&mut session, true)?;
        bio::open(&mut session)?;
        drop(session);
        if let Err(e) = self.prepare_bio_open() {
            bio::release(&mut self.bio_session.lock());
            return Err(e);
        }
        Ok(())
    }

    pub(crate) fn bio_release(&self) {
        bio::release(&mut self.bio_session.lock());
        if let Some(dev) = self.bio_dev.lock().as_ref() {
            dev.wake();
        }
    }

    /// Called by the enrol and verify work items before they touch the sensor,
    /// and by a deletion before it touches the enclave. Returns false if the
    /// system is going to sleep; the session has then been ended and the
    /// capture or deletion must not start. A true return must be paired with
    /// [`Self::capture_end`].
    pub(crate) fn capture_begin(&self) -> bool {
        let mut session = self.bio_session.lock();
        if self.suspending.load(Relaxed) {
            let woke = bio::abort_capture(&mut session);
            drop(session);
            if woke {
                self.bio_wake();
            }
            return false;
        }
        self.captures_running.fetch_add(1, Relaxed);
        true
    }

    pub(crate) fn capture_end(&self) {
        self.captures_running.fetch_sub(1, Relaxed);
    }

    /// The enrol or verify work item `epoch` has stopped.
    pub(crate) fn capture_worker_stopped(&self, epoch: u64) {
        bio::worker_stopped(&mut self.bio_session.lock(), epoch);
        self.worker_gone.notify_all();
    }

    /// Holds an open or a capture start while the last enrol or verify work
    /// item is still on its way out. It reports its result before it stops,
    /// and after a cancel or a close it stops only once the capture notices,
    /// so an open or a start straight after either would be refused with
    /// EBUSY, which the caller cannot tell from a device in use. If it has
    /// not stopped in time, the caller refuses as before.
    fn wait_for_worker_exit(
        &self,
        session: &mut MutexGuard<'_, bio::Session>,
        opening: bool,
    ) -> Result<()> {
        let mut remaining = time::msecs_to_jiffies(WORKER_EXIT_WAIT_MS);
        while bio::waiting_on_worker(session, opening) {
            if self.shutting_down.load(Relaxed) {
                return Err(ENODEV);
            }
            if remaining == 0 {
                dev_warn!(
                    self.dev,
                    "Touch ID: the last capture had not stopped after {} ms\n",
                    WORKER_EXIT_WAIT_MS
                );
                return Ok(());
            }
            match self.worker_gone.wait_interruptible_timeout(session, remaining) {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Timeout => remaining = 0,
                CondVarTimeoutResult::Signal { .. } => return Err(ERESTARTSYS),
            }
        }
        Ok(())
    }

    /// The system is about to suspend. End any capture in progress, hold new
    /// ones, and wait for the enrol or verify work to stop touching the sensor
    /// before devices go down.
    pub(crate) fn sleep_prepare(&self) {
        let woke = {
            let mut session = self.bio_session.lock();
            self.suspending.store(true, Relaxed);
            bio::abort_capture(&mut session)
        };
        if woke {
            dev_info!(
                self.dev,
                "Touch ID: ending the capture in progress before sleep\n"
            );
            self.bio_wake();
        }

        let mut waited: u32 = 0;
        while self.captures_running.load(Relaxed) != 0 {
            if waited >= SLEEP_DRAIN_MS {
                // Suspend anyway, as before: failing here would leave a
                // laptop awake with its lid shut. The work item has been told
                // to stop, and the next capture power-cycles and registers
                // the sensor again.
                dev_warn!(
                    self.dev,
                    "Touch ID: a capture was still running {} ms after the system started to sleep; continuing\n",
                    SLEEP_DRAIN_MS
                );
                return;
            }
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(10));
            waited += 10;
        }
    }

    /// The system has resumed, or the suspend was aborted.
    pub(crate) fn sleep_finished(&self) {
        let _session = self.bio_session.lock();
        self.suspending.store(false, Relaxed);
        self.sleep_over.notify_all();
    }

    /// Holds a capture start until the system is awake. Userspace is thawed
    /// before the driver hears of the resume, so a lock screen that retries at
    /// once would otherwise be refused. A start made just before the system
    /// sleeps is interrupted by the freezer and restarted after resume.
    fn wait_until_awake(&self) -> Result<()> {
        let mut session = self.bio_session.lock();
        while self.suspending.load(Relaxed) {
            if self.sleep_over.wait_interruptible(&mut session) {
                return Err(ERESTARTSYS);
            }
        }
        Ok(())
    }

    pub(crate) fn queue_enrolment(this: Arc<SepData>) {
        let Some(epoch) = bio::queued_epoch(&this.bio_session.lock()) else {
            return;
        };
        this.capture_epoch.store(epoch, Relaxed);
        if workqueue::system_dfl()
            .enqueue::<Arc<SepData>, ENROL_WORK_ID>(this.clone())
            .is_err()
        {
            this.finish_enrolment(Err(ENROL_STATUS_SENSOR));
            this.capture_worker_stopped(epoch);
        }
    }

    pub(crate) fn queue_verify(this: Arc<SepData>) {
        let Some(epoch) = bio::queued_epoch(&this.bio_session.lock()) else {
            return;
        };
        this.capture_epoch.store(epoch, Relaxed);
        if workqueue::system_dfl()
            .enqueue::<Arc<SepData>, VERIFY_WORK_ID>(this.clone())
            .is_err()
        {
            this.finish_verify(
                bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR),
                [0u8; bio::TOKEN_LEN],
            );
            this.capture_worker_stopped(epoch);
        }
    }

    fn finish_verify(&self, outcome: bio::VerifyOutcome, token_bytes: [u8; bio::TOKEN_LEN]) {
        let filed = bio::verify_finish(
            &mut self.bio_session.lock(), self.capture_epoch.load(Relaxed), outcome, token_bytes,
        );
        if filed {
            self.bio_wake();
        }
    }

    fn mint_token_bytes(&self) -> Option<[u8; bio::TOKEN_LEN]> {
        let mut bytes = [0u8; bio::TOKEN_LEN];
        // This is a host-issued, per-operation capability, not SEP match
        // evidence. Use the initialized kernel CSPRNG, as for other host
        // secrets, rather than trusting firmware control replies as entropy.
        // J313/13.5 answered those draws with zero and caused valid matches
        // to carry an invalid all-zero token to userspace.
        if shim::random_bytes(&mut bytes).is_err() || bytes.iter().all(|b| *b == 0) {
            return None;
        }
        Some(bytes)
    }

    pub(crate) fn bio_ioctl(&self, cmd: u32, arg: usize) -> Result<bio::Handled> {
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }
        if cmd == bio::IOC_ATTEST {
            return self.bio_attest(arg);
        }
        // With xART writes off the enclave cannot persist what an enrolment or
        // a match changes. It saves anti-replay state after every match, and a
        // refused save has left it unresponsive until reboot. Refuse the
        // capture instead; attach and the read-only ioctls still work.
        if bio::starts_capture(cmd) && !self.xart_writable() {
            if !self.capture_refusal_logged().xchg(true, Relaxed) {
                dev_warn!(
                    self.dev,
                    "Touch ID: enrol and verify need xart_writes=1; the enclave saves anti-replay state after every match and stops responding when that save is refused\n"
                );
            }
            return Err(EROFS);
        }
        // A capture started while the system goes to sleep would run into the
        // suspend; `capture_begin` repeats this check under the session lock.
        if bio::starts_capture(cmd) {
            self.wait_until_awake()?;
        }
        // Query SEP outside the bio session/index locks. A host index can
        // outlive its SEP identity after a failed cold restore; userspace
        // must not mistake that index for proof of a live template.
        let live_identity_count = if (cmd == bio::IOC_GET_INFO || cmd == bio::IOC_ENROL_START)
            && self.touchid_started.load(Relaxed)
        {
            self.enclave_identity_count()
        } else {
            None
        };
        // The SPI probe may have deferred until its SMC power supplier bound.
        // A saved attach-time snapshot would incorrectly reject later ioctls.
        let sensor_present = sensor::is_bound();
        let handled = {
            let mut session = self.bio_session.lock();
            if bio::starts_capture(cmd) {
                self.wait_for_worker_exit(&mut session, false)?;
            }
            let index = self.bio_index.lock();
            let mut ctx = bio::Context {
                session: &mut session,
                index: &index,
                sensor_present,
                live_identity_count,
            };
            bio::ioctl(&mut ctx, cmd, arg)?
        };

        if handled.delete_identity.is_some() || !handled.delete_identities.is_empty() {
            // A deletion is made durable by saving the Catacombs, and with
            // xART writes off that save would be refused. Refuse before 0x57
            // rather than report a deletion that the next boot undoes.
            if !self.xart_writable() {
                dev_warn!(
                    self.dev,
                    "Touch ID: deleting a fingerprint needs xart_writes=1; with writes off the deletion would not survive a reboot\n"
                );
                return Err(EROFS);
            }
            // 0x57 and the saves are sent with the session lock released
            // (they wait on mailbox replies). The system must not sleep
            // partway through the saves, and a verify that finishes after a
            // cancel saves too.
            let deleted = loop {
                self.wait_until_awake()?;
                let _writer = self.catacomb_writer.lock();
                // A suspend began while this waited for the lock: wait it out.
                if !self.capture_begin() {
                    continue;
                }
                let deleted = self.delete_and_persist(&handled);
                self.capture_end();
                break deleted;
            };
            deleted?;
        }

        if handled.wake {
            self.bio_wake();
        }
        Ok(handled)
    }

    fn bio_attest(&self, arg: usize) -> Result<bio::Handled> {
        let user = kernel::uaccess::UserPtr::from_addr(arg);
        let req: bio::Attest =
            kernel::uaccess::UserSlice::new(user, core::mem::size_of::<bio::Attest>())
                .reader()
                .read()?;
        let (sig, pubk) = self.refkey_attest_sign(&req.challenge)?;
        if sig.len() > bio::ATTEST_SIG_MAX || pubk.len() != bio::ATTEST_PUB_LEN {
            return Err(EIO);
        }
        let mut out = bio::Attest {
            sig_len: sig.len() as u32,
            challenge: req.challenge,
            public_key: [0u8; bio::ATTEST_PUB_LEN],
            signature: [0u8; bio::ATTEST_SIG_MAX],
            reserved: [0u8; 3],
        };
        out.public_key.copy_from_slice(&pubk);
        out.signature[..sig.len()].copy_from_slice(&sig);
        kernel::uaccess::UserSlice::new(user, core::mem::size_of::<bio::Attest>())
            .writer()
            .write(&out)?;
        Ok(bio::Handled {
            ret: 0,
            wake: false,
            start_enrol: false,
            start_verify: false,
            delete_identity: None,
            delete_identities: KVec::new(),
        })
    }

    fn ensure_restored_after(&self, prepared: bool) {
        // Probe credential ordering without assuming it fixes cold restore.
        // J414s still answered 0x8002 and listed zero identities with this
        // credential established, so the identity proof below remains required.
        let mut unlocked = false;
        if self.keybag_designated.load(Relaxed) {
            if let Some(user) = crate::sks::DesignateUser::new(SBIO_PROBE_USER_ID) {
                let special = user.special_handle();
                if let Ok(keybag::State::Present(stored)) = keybag::read(keybag::Slot::Identity) {
                    unlocked = match self.profile.key_store {
                        profile::KeyStore::Sepos13 { .. } => self
                            .sks_step(c"DEVICE_STATE_TRANSITION", |healthy| {
                                self.sks_req_unlock_special(special, stored.secret(), healthy)
                            })
                            .is_some(),
                        profile::KeyStore::Variant5 => self
                            .sks_step(crate::sks::SKS_LOCK_STATE_NAME, |healthy| {
                                self.sks_req_unlock_special(special, stored.secret(), healthy)
                            })
                            .is_some(),
                    };
                }
            }
        }
        let match_context = crate::sbio::UserId::new(SBIO_PROBE_USER_ID)
            .map(|user| self.establish_scrd_match_context(user))
            .unwrap_or(false);
        dev_info!(self.dev, "sbio: pre-restore special unlock {}, SCRD match context {}\n", unlocked, match_context);

        let restored = self.restore_all_components();
        dev_info!(self.dev, "sbio: cold-match preparation {}, component restore {}\n", prepared, restored);
        // Restore can replace the credential and the device's identity view.
        // Refresh both after loading J700's persistent components, before
        // admitting a verify operation.
        let match_context = if self.profile.nonce_session {
            let synced = restored && self.sync_device_view();
            self.device_view_synced.store(synced, Relaxed);
            synced && crate::sbio::UserId::new(SBIO_PROBE_USER_ID)
                .is_some_and(|user| self.establish_scrd_match_context(user))
        } else {
            match_context
        };
        let identities = self.log_identity_count(c"after Catacomb restore with sensor registered");
        let identity_ready = matches!(identities, Some(n) if n > 0);
        let candidate = prepared && match_context && restored && identity_ready;
        self.cold_restore_candidate.store(candidate, Relaxed);
        // COMPLETE_INIT has not synchronized the device view yet. Never make
        // verification available on the preliminary Catacomb proof alone;
        // complete_bringup finishes the proof and sets templates_restored.
        self.templates_restored.store(false, Relaxed);
        // The nonce path synced the view before COMPLETE_INIT to refresh SCRD.
        // Re-arm the final proof and its bounded retry on every cold restore.
        if self.profile.nonce_session {
            self.device_view_synced.store(false, Relaxed);
        }
        if !candidate {
            dev_err!(
                self.dev,
                "matching unavailable this boot: cold-match preparation, SCRD credential, Catacomb restore, or identity proof did not complete; template state is unproven\n"
            );
            return;
        }
        dev_info!(self.dev, "sbio: cold restore candidate ready; waiting for device-view proof\n");

    }

    fn cold_match_continue(
        &self,
        source: crate::sks::KeyBagHandle,
        uuid: [u8; keybag::UUID_LEN],
        provenance: keybag::UuidProvenance,
    ) -> bool {
        if !self.keybag_designated.load(Relaxed) {
            return false;
        }

        let Some(user) = crate::sks::DesignateUser::new(SBIO_PROBE_USER_ID) else {
            return false;
        };
        let special = user.special_handle();
        self.log_identity_count(c"before the cold-match preparation");

        let Some(source_uuid) = self.sks_read_uuid(source) else {
            return false;
        };
        let generated_lookup = matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. })
            && provenance == keybag::UuidProvenance::AsGenerated;
        let Some(identity) = keybag::SnapshotIdentity::from_loaded(
            uuid, provenance, source_uuid, generated_lookup,
        ) else {
            dev_err!(self.dev, "sks: recovered source bag does not match its stored identity\n");
            return false;
        };
        let uuid_ok = self
            .sks_send(self.sks_req_copy_uuid_special(special))
            .and_then(|out| self.sks_uuid_from_reply(&out));
        if !uuid_ok.is_some_and(|got| identity.accepts(&uuid, provenance, &got)) {
            dev_err!(self.dev, "sks: designated bag does not match the recovered source bag\n");
            return false;
        }

        if self.sks_send(self.sks_req_unload_keybag(source)).is_none() {
            return false;
        }

        let mut material = self.enrol_material.lock();
        let Some(material) = &mut *material else {
            return false;
        };
        material.snapshot_identity = Some(identity);
        dev_info!(self.dev, "sks: snapshot identity bound to recovered and designated bag; generated lookup {}\n", generated_lookup);

        true
    }

    pub(crate) fn sep_random(&self, buf: &mut [u8]) -> Result<()> {
        // The trusted-key framework asks for a fresh plaintext key before
        // passing it to SEP for sealing. The host CSPRNG is appropriate for
        // that key; J313/13.5's unverified SEP control entropy replies have
        // returned all zeros and must not be used as key material.
        shim::random_bytes(buf)?;
        if !buf.is_empty() && buf.iter().all(|b| *b == 0) {
            return Err(EIO);
        }
        Ok(())
    }

    pub(crate) fn wrapped_from_copy_reply(
        &self,
        out: &SksOutcome,
        _from: &CStr,
    ) -> Option<KVec<u8>> {
        let body = self.sks_report_response(c"COPY_KEYBAG", out)?;
        if out.reply.status != 0 || image::operation_status(body).unwrap_or(-1) != 0 {
            return None;
        }
        let (blob, _) = image::read_blob(body, 4)?;
        if blob.is_empty() {
            return None;
        }
        let mut copy = KVec::new();
        if copy.extend_from_slice(blob, GFP_KERNEL).is_err() {
            return None;
        }
        Some(copy)
    }

    fn prove_restore(&self) -> bool {
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            return false;
        };
        let op = crate::sbio::sbio_free_identity_count(user);
        let Some(reply) = self.sbio_expect_ok(&op) else {
            return false;
        };
        if reply.len() != crate::sbio::SBIO_FREE_COUNT_REPLY_LEN {
            return false;
        }
        let count = u32::from_le_bytes([reply[0], reply[1], reply[2], reply[3]]);
        if count > crate::sbio::SBIO_FREE_COUNT_MAX {
            return false;
        }
        if count == crate::sbio::SBIO_FREE_COUNT_MAX {
            return true;
        }
        true
    }

    fn enclave_identity_count(&self) -> Option<usize> {
        let op = crate::sbio::sbio_list_identities();
        let SbioOutcome::Ok(reply) = self.sbio_call(&op) else {
            return None;
        };
        let records = crate::sbio::IdentityRecords::new(&reply)?;
        Some(records.count())
    }

    fn log_identity_count(&self, when: &CStr) -> Option<usize> {
        let Some(count) = self.enclave_identity_count() else {
            dev_warn!(self.dev, "sbio: identity list unavailable {}\n", when);
            return None;
        };
        dev_info!(self.dev, "sbio: identity count {} {}\n", count, when);
        Some(count)
    }

    fn enclave_lists(&self, uuid: &[u8; bio::UUID_LEN]) -> Option<bool> {
        let op = crate::sbio::sbio_list_identities();
        let SbioOutcome::Ok(reply) = self.sbio_call(&op) else {
            return None;
        };
        let records = crate::sbio::IdentityRecords::new(&reply)?;
        Some(records.lists_uuid(uuid))
    }

    /// Called with `catacomb_writer` held. A host index entry goes only once
    /// its deletion is saved: if the save fails, the entry stays, userspace
    /// still lists the fingerprint, and a retried deletion saves again.
    fn delete_and_persist(&self, handled: &bio::Handled) -> Result<()> {
        let retry = self.deletion_unsaved.load(Relaxed);
        let wanted =
            Iterator::chain(handled.delete_identity.iter(), handled.delete_identities.iter());
        // Allocated before any 0x57, so recording a removal cannot fail.
        let mut gone: KVec<[u8; bio::UUID_LEN]> =
            KVec::with_capacity(wanted.clone().count(), GFP_KERNEL)?;
        let mut delete_err: Option<Error> = None;
        for identity in wanted {
            match self.delete_identity(identity) {
                Ok(true) => {
                    let _ = gone.push_within_capacity(*identity.uuid());
                }
                Ok(false) => {}
                Err(e) => delete_err = delete_err.or(Some(e)),
            }
        }
        // Save what was removed even if another deletion failed.
        if !gone.is_empty() {
            match self.persist_deletion(retry) {
                Ok(()) => {
                    self.deletion_unsaved.store(false, Relaxed);
                    for uuid in gone.iter() {
                        self.forget_identity(uuid, c"removed from the enclave");
                    }
                }
                Err(e) => {
                    dev_err!(
                        self.dev,
                        "bio: removed from the enclave but not saved; the fingerprint stays listed, returns at the next boot, and deleting it again retries the save\n"
                    );
                    self.deletion_unsaved.store(true, Relaxed);
                    delete_err = delete_err.or(Some(e));
                }
            }
        }
        match delete_err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// Returns whether the enclave no longer holds the identity and the host
    /// entry must wait for [`Self::persist_deletion`].
    fn delete_identity(&self, identity: &crate::sbio::IdentityV1) -> Result<bool> {
        if self.enclave_lists(identity.uuid()) == Some(false) {
            // Unless an earlier deletion's save failed, which leaves the
            // enclave without an identity the saved Catacomb still holds.
            if self.deletion_unsaved.load(Relaxed) {
                return Ok(true);
            }
            self.forget_identity(identity.uuid(), c"a host entry the enclave never had");
            return Ok(false);
        }

        // 0x57 cannot be undone. Check first that its deletion can be saved.
        if crate::catacomb::read(PRIVATE_TYPE_CATACOMB_OWNER).is_none()
            || !self
                .enrol_material
                .lock()
                .as_ref()
                .is_some_and(|material| material.snapshot_identity.is_some())
        {
            dev_err!(
                self.dev,
                "bio: not deleting: there is no owner Catacomb file or identity-bag binding to save the deletion with; nothing was changed\n"
            );
            return Err(EIO);
        }

        let op = crate::sbio::sbio_delete_identity(identity);

        if self.sbio_expect_ok(&op).is_none() {
            dev_err!(
                self.dev,
                "bio: 0x57 failed, status above; host index left unchanged (forgetting a template the enclave still holds would report a deletion that did not happen)\n"
            );
            return Err(EIO);
        }

        Ok(true)
    }

    /// 0x57 removes an identity from the enclave's memory only. Until the user
    /// Catacomb is saved, the next attach restores the saved one and the
    /// deleted fingerprint with it. Saved like a match: only the components
    /// the enclave marks save-pending (a SAVE_CATACOMB of one that is not has
    /// been refused), the user before the master because saving the user can
    /// change the master, state re-read before each, then the lockout and the
    /// identity bag, which must match the Catacombs. Safe to repeat after a
    /// failure partway: a `retry` accepts a user Catacomb an earlier attempt
    /// already saved.
    fn persist_deletion(&self, retry: bool) -> Result<()> {
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            return Err(EIO);
        };
        for (who, kind, what) in [
            (
                crate::sbio::CatacombUser::enrolling(user),
                PRIVATE_TYPE_CATACOMB_USER,
                c"user Catacomb after deletion",
            ),
            (
                crate::sbio::CatacombUser::MASTER,
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"master Catacomb after deletion",
            ),
        ] {
            let Some(reply) = self.read_component_states() else {
                dev_err!(self.dev, "bio: cannot read component state before saving the {}\n", what);
                return Err(EIO);
            };
            let state = crate::sbio::ComponentStates::new(&reply).state_for(who.value());
            dev_info!(self.dev, "bio: {} state {:?}\n", what, state);
            let Some(state) = state else {
                dev_err!(self.dev, "bio: the {} is absent; the deletion cannot be saved\n", what);
                return Err(EIO);
            };
            if state & crate::sbio::COMPONENT_STATE_ACTIVE == 0 {
                dev_err!(self.dev, "bio: the {} is not active; the deletion cannot be saved\n", what);
                return Err(EIO);
            }
            if state & crate::sbio::COMPONENT_STATE_SAVE_PENDING != 0 {
                if !self.save_catacomb(who, kind, what) {
                    dev_err!(self.dev, "bio: saving the {} failed\n", what);
                    return Err(EIO);
                }
            } else if who.value() == user.value() && !retry {
                dev_err!(
                    self.dev,
                    "bio: the enclave did not mark the user Catacomb for saving after 0x57; the deletion cannot be saved\n"
                );
                return Err(EIO);
            }
            if crate::catacomb::read(kind).is_none() {
                dev_err!(self.dev, "bio: the {} has no durable host file\n", what);
                return Err(EIO);
            }
        }
        if !self.save_lockout() {
            dev_err!(self.dev, "bio: lockout persistence failed after the deletion\n");
            return Err(EIO);
        }
        if !self.resnapshot_identity_keybag() {
            dev_err!(
                self.dev,
                "bio: Catacombs saved after the deletion but the identity-bag snapshot failed\n"
            );
            return Err(EIO);
        }
        Ok(())
    }

    fn forget_identity(&self, uuid: &[u8; bio::UUID_LEN], why: &CStr) {
        let persisted = {
            let mut index = self.bio_index.lock();
            index.remove(uuid);
            self.with_host_store(|store| index.persist(store))
        };
        match persisted {
            Some(Ok(())) => {},
            _ => dev_warn!(
                self.dev,
                "bio: {} ({}) but the host index could not be written; correct in memory this boot, rebuilt at next attach\n",
                Hex(uuid),
                why
            ),
        }
    }

    pub(crate) fn bio_ready(&self) -> bool {
        self.bio_session.lock().ready()
    }
}

/// Which bring-up a round of attempts makes.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Bringup {
    /// As Touch ID is activated.
    Attach,
    /// The late round after an attach that fell short.
    Late,
    /// Before a verify.
    Verify,
}

/// A sensor bring-up with retries, as [`registration::bring_up`] drives it.
struct BringupAttempts<'a, W: Fn() -> bool> {
    sep: &'a SepData,
    purpose: &'a CStr,
    wanted: W,
    began_ns: u64,
    /// Each attempt is the attach-time one rather than a verify's.
    attach: bool,
    /// How many failed attempts the next one follows, in this round or the
    /// one before it; [`registration::retry_stage`] restarts from it.
    retry: u32,
}

impl<W: Fn() -> bool> registration::Attempts for BringupAttempts<'_, W> {
    fn attempt(&mut self) -> bool {
        if self.retry > 0 {
            let stage = registration::retry_stage(
                self.sep.bringup.load(Relaxed),
                self.sep.profile.nonce_session,
                self.retry,
            );
            self.sep.bringup.store(stage, Relaxed);
        }
        self.retry += 1;
        if self.attach {
            self.sep.attach_bring_sensor_online()
        } else {
            self.sep.bring_sensor_online()
        }
    }

    fn wanted(&self) -> bool {
        (self.wanted)()
    }

    fn elapsed_ms(&self) -> u64 {
        shim::boottime_ns().saturating_sub(self.began_ns) / 1_000_000
    }

    fn failed(&mut self, attempt: u32, of: u32, pause_ms: u32) {
        if pause_ms == 0 {
            dev_warn!(
                self.sep.dev,
                "sensor: {} bring-up attempt {} of {} failed; starting again\n",
                self.purpose,
                attempt,
                of
            );
        } else {
            dev_warn!(
                self.sep.dev,
                "sensor: {} bring-up attempt {} of {} failed; starting again in {} ms\n",
                self.purpose,
                attempt,
                of,
                pause_ms
            );
        }
    }

    // In short steps, so that a suspend or an unbind is not kept waiting.
    fn pause(&mut self, ms: u32) -> bool {
        let mut left = ms;
        while left > 0 {
            if !(self.wanted)() {
                return false;
            }
            let step = left.min(BRINGUP_PAUSE_STEP_MS);
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(step)));
            left -= step;
        }
        (self.wanted)()
    }
}

/// The requests of a nonce-session authentication, as
/// [`registration::authenticate`] orders them. Every failure is logged where
/// it happens, except a 0x16 refusal of REGISTER_SERIAL, which may be the
/// enclave pointing at a registration it already holds.
struct SensorAuthentication<'a> {
    sep: &'a SepData,
    refused16: bool,
}

impl registration::Authentication for SensorAuthentication<'_> {
    type Serial = sensor::SensorSerial;
    type Nonce = [u8; 32];

    fn read_serial(&mut self) -> Option<sensor::SensorSerial> {
        self.sep.read_serial_logged()
    }

    fn read_nonce(&mut self) -> Option<[u8; 32]> {
        match sensor::session_nonce() {
            Ok(nonce) => {
                self.sep.sensor_trace(c"nonce read");
                Some(nonce)
            }
            Err(error) => {
                dev_err!(self.sep.dev, "sensor: nonce read failed: {:?}\n", error);
                self.sep.sensor_step_failed(c"nonce read");
                None
            }
        }
    }

    fn register_serial(&mut self, serial: &sensor::SensorSerial) -> registration::SerialAnswer {
        let op = crate::sbio::sbio_register_sensor_serial(serial);
        match self.sep.sbio_transfer(&op) {
            Ok(done) if done.status.is_ok() => {
                self.sep.sensor_trace(op.name());
                registration::SerialAnswer::Accepted
            }
            Ok(done)
                if matches!(
                    done.status.answered(),
                    Some(err) if err as u16 == crate::sbio::SBIO_STATUS_16
                ) =>
            {
                self.refused16 = true;
                registration::SerialAnswer::Refused16
            }
            Ok(done) => {
                dev_warn!(
                    self.sep.dev,
                    "sensor: {} refused with status {}\n",
                    op.name(),
                    done.status
                );
                self.sep.sensor_step_failed(op.name());
                registration::SerialAnswer::Failed
            }
            Err(e) => {
                dev_warn!(self.sep.dev, "sensor: {} transfer failed: {:?}\n", op.name(), e);
                self.sep.sensor_step_failed(op.name());
                registration::SerialAnswer::Failed
            }
        }
    }

    fn open_session(&mut self, nonce: &[u8; 32]) -> bool {
        self.sep.sensor_nonce_session(nonce)
    }
}

/// The SBIO reply header, as [`transfer::take_header`] samples it.
struct SbioHeader<'a>(&'a SepData);

impl transfer::HeaderSlot for SbioHeader<'_> {
    fn sample(&self, header: &mut [u8; transfer::HEADER_LEN]) -> bool {
        self.0.ool_peek(&self.0.ool_sbio, 0, header)
    }

    fn repoison(&self) {
        self.0
            .ool_repoison(&self.0.ool_sbio, 0, transfer::HEADER_LEN);
    }

    fn pause(&self) {
        let poll = kernel::time::Delta::from_millis(i64::from(OOL_WRITE_POLL_MS));
        kernel::time::delay::fsleep(poll);
    }
}

pub(crate) struct SbioOp {
    opcode: u16,
    payload: [u8; SBIO_MAX_PAYLOAD],
    payload_len: usize,
    name: &'static CStr,
}

const SBIO_MAX_PAYLOAD: usize = 0x97;

pub(crate) const SBIO_IMAGE_INIT_LEN: usize = 0x97;
static_assert!(SBIO_IMAGE_INIT_LEN <= SBIO_MAX_PAYLOAD);

const fn pad_payload(src: &[u8]) -> [u8; SBIO_MAX_PAYLOAD] {
    let mut out = [0u8; SBIO_MAX_PAYLOAD];
    let mut i = 0;
    while i < src.len() {
        out[i] = src[i];
        i += 1;
    }
    out
}

impl SbioOp {
    pub(crate) fn opcode(&self) -> u16 {
        self.opcode
    }
    pub(crate) fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_len]
    }
    pub(crate) fn name(&self) -> &'static CStr {
        self.name
    }
}

pub(crate) const SBIO_PROTOCOL_GENERATION: u32 = 1;

const OP_SBIO_REGISTER_SENSOR: u16 = 0x80;
pub(crate) fn sbio_register_sensor(id: &crate::sensor::Identifier) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_REGISTER_SENSOR,
        payload: pad_payload(&id.value().to_le_bytes()),
        payload_len: 2,
        name: c"REGISTER_SENSOR",
    }
}

pub(crate) const SBIO_SIGNAL_QUALITY: u32 = 0;

const OP_SBIO_COVERAGE_PARAMS: u16 = 0x5d;
pub(crate) fn sbio_coverage_params() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_COVERAGE_PARAMS,
        payload: pad_payload(&SBIO_SIGNAL_QUALITY.to_le_bytes()),
        payload_len: 4,
        name: c"COVERAGE_PARAMS",
    }
}

const OP_SBIO_OPERATION_PARAMS: u16 = 0x5c;

pub(crate) const SBIO_OPERATION_PARAMS_LEN: usize = match SBIO_PROTOCOL_GENERATION {
    1 => 4,
    6 => 8,
    _ => 0,
};
static_assert!(SBIO_OPERATION_PARAMS_LEN != 0);
static_assert!(SBIO_OPERATION_PARAMS_LEN <= SBIO_MAX_PAYLOAD);

pub(crate) fn sbio_operation_params() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_OPERATION_PARAMS,
        payload: pad_payload(&[]),
        payload_len: SBIO_OPERATION_PARAMS_LEN,
        name: c"OPERATION_PARAMS",
    }
}

const OP_SBIO_TRANSPARENT_CHANNEL: u16 = 0x6a;
pub(crate) fn sbio_transparent_channel() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_TRANSPARENT_CHANNEL,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"TRANSPARENT_CHANNEL",
    }
}

const OP_SBIO_COMPLETE_INIT: u16 = 0x01;
pub(crate) fn sbio_complete_init(
    patch: crate::PatchLoaded,
    params: crate::ParametersApplied,
) -> SbioOp {
    let (_, _) = (patch, params);
    let mut body = [0u8; 8];
    body[0..4].copy_from_slice(&1u32.to_le_bytes());
    body[4..8].copy_from_slice(&1u32.to_le_bytes());
    SbioOp {
        opcode: OP_SBIO_COMPLETE_INIT,
        payload: pad_payload(&body),
        payload_len: 8,
        name: c"COMPLETE_INIT",
    }
}

const CONTEXT_SCOPE_SYSTEM: i32 = -1;
static_assert!(CONTEXT_SCOPE_SYSTEM != 0);
static_assert!(CONTEXT_SCOPE_SYSTEM < 0);

#[derive(Clone, Copy)]
pub(crate) struct ContextScope(i32);

impl ContextScope {
    pub(crate) const SYSTEM: ContextScope = ContextScope(CONTEXT_SCOPE_SYSTEM);

    pub(crate) fn user(id: UserId) -> ContextScope {
        ContextScope(id.value())
    }

    pub(crate) fn value(&self) -> i32 {
        self.0
    }
}

const OP_SBIO_CONTEXT_STATE: u16 = 0x6b;
pub(crate) fn sbio_context_state() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CONTEXT_STATE,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"CONTEXT_STATE",
    }
}

// 0x2d destroys the catacomb; only send with zero existing identities
pub(crate) struct NoExistingCatacomb {
    _seal: (),
}

impl NoExistingCatacomb {
    pub(crate) fn from_zero_identities(count: usize) -> Option<NoExistingCatacomb> {
        match count {
            0 => Some(NoExistingCatacomb { _seal: () }),
            _ => None,
        }
    }
}

const OP_SBIO_SELECT_CONTEXT: u16 = 0x2d;
pub(crate) fn sbio_select_context(scope: ContextScope, proof: &NoExistingCatacomb) -> SbioOp {
    let NoExistingCatacomb { _seal: () } = proof;
    SbioOp {
        opcode: OP_SBIO_SELECT_CONTEXT,
        payload: pad_payload(&scope.value().to_le_bytes()),
        payload_len: 4,
        name: c"SELECT_CONTEXT",
    }
}

pub(crate) const SBIO_PROTECTED_CONFIG_LEN: usize = 32;

const OP_SBIO_PROTECTED_CONFIG: u16 = 0x2b;
pub(crate) fn sbio_protected_config(id: UserId) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_PROTECTED_CONFIG,
        payload: pad_payload(&id.value().to_le_bytes()),
        payload_len: 4,
        name: c"PROTECTED_CONFIG",
    }
}

const OP_SBIO_BEGIN_ENROL: u16 = 0x03;

pub(crate) const BE_AUTH_TYPE_ACM_CONTEXT: u32 = 0;

pub(crate) const SBIO_BEGIN_ENROL_LEN: usize = 0x44;
static_assert!(SBIO_BEGIN_ENROL_LEN <= SBIO_MAX_PAYLOAD);

const BE_FLAGS: usize = 0;
const BE_USER_ID: usize = 4;
const BE_AUTH_TYPE: usize = 8;
const BE_TOKEN_LEN: usize = 12;
const BE_TOKEN: usize = 16;
static_assert!(BE_TOKEN + SKS_AUTH_TOKEN_LEN <= SBIO_BEGIN_ENROL_LEN);

pub(crate) fn sbio_begin_enrol(
    user: UserId,
    auth_type: u32,
    token: &[u8; SKS_AUTH_TOKEN_LEN],
) -> SbioOp {
    let mut body = [0u8; SBIO_BEGIN_ENROL_LEN];
    body[BE_FLAGS..BE_FLAGS + 4].copy_from_slice(&0u32.to_le_bytes());
    body[BE_USER_ID..BE_USER_ID + 4].copy_from_slice(&user.value().to_le_bytes());
    body[BE_AUTH_TYPE..BE_AUTH_TYPE + 4].copy_from_slice(&auth_type.to_le_bytes());
    body[BE_TOKEN_LEN..BE_TOKEN_LEN + 4]
        .copy_from_slice(&(SKS_AUTH_TOKEN_LEN as u32).to_le_bytes());
    body[BE_TOKEN..BE_TOKEN + SKS_AUTH_TOKEN_LEN].copy_from_slice(token);
    SbioOp {
        opcode: OP_SBIO_BEGIN_ENROL,
        payload: pad_payload(&body),
        payload_len: SBIO_BEGIN_ENROL_LEN,
        name: c"BEGIN_ENROL",
    }
}

const OP_SBIO_MODULE_CHALLENGE: u16 = 0x31;
pub(crate) fn sbio_module_challenge() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_MODULE_CHALLENGE,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"MODULE_CHALLENGE",
    }
}

pub(crate) const SBIO_MODULE_CHALLENGE_LEN: usize = 0x40;

const OP_SBIO_MODULE_COMMIT: u16 = 0x32;
pub(crate) fn sbio_module_commit(reply: &[u8]) -> Option<SbioOp> {
    if reply.is_empty() || reply.len() > SBIO_MAX_PAYLOAD {
        return None;
    }
    Some(SbioOp {
        opcode: OP_SBIO_MODULE_COMMIT,
        payload: pad_payload(reply),
        payload_len: reply.len(),
        name: c"MODULE_COMMIT",
    })
}

const OP_SBIO_LOAD_CALIBRATION: u16 = 0x5b;

pub(crate) const SBIO_CALIBRATION_SOURCE: u32 = 3;

pub(crate) struct SbioCalibration(KVec<u8>);

impl SbioCalibration {
    pub(crate) fn new(blob: &crate::CalibrationBlob, source: u32) -> Result<SbioCalibration> {
        let bytes = blob.bytes();
        let mut body = KVec::new();
        body.extend_from_slice(&source.to_le_bytes(), GFP_KERNEL)?;
        body.extend_from_slice(bytes, GFP_KERNEL)?;
        Ok(SbioCalibration(body))
    }

    pub(crate) fn opcode(&self) -> u16 {
        OP_SBIO_LOAD_CALIBRATION
    }

    pub(crate) fn payload(&self) -> &[u8] {
        &self.0
    }

    pub(crate) fn name(&self) -> &'static CStr {
        c"LOAD_CALIBRATION"
    }
}

const OP_SBIO_UPDATE_DEVICE_LIST: u16 = 0x7b;
pub(crate) fn sbio_update_device_list() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_UPDATE_DEVICE_LIST,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"UPDATE_DEVICE_LIST",
    }
}

pub(crate) const SBIO_MATCH_POLICY_LEN: usize = 2;

const OP_SBIO_MATCH_POLICY: u16 = 0x1f;
pub(crate) fn sbio_match_policy() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_MATCH_POLICY,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"MATCH_POLICY",
    }
}

pub(crate) const SBIO_ENROL_RESULT_LEN: usize = 0xc98;

const ER_PROGRESS: usize = 0x004;
const ER_HAS_TEMPLATE: usize = 0x006;
const ER_COMPLETE: usize = 0xbfe;

static_assert!(ER_PROGRESS < SBIO_ENROL_RESULT_LEN);
static_assert!(ER_HAS_TEMPLATE + 4 <= SBIO_ENROL_RESULT_LEN);
static_assert!(ER_COMPLETE + 4 <= SBIO_ENROL_RESULT_LEN);
static_assert!(ER_COMPLETE > ER_HAS_TEMPLATE + 4);

pub(crate) const ER_IDENTITY_LEN: usize = 4 + IDENTITY_UUID_LEN;
static_assert!(ER_IDENTITY_LEN == 20);
static_assert!(ER_IDENTITY_LEN <= SBIO_ENROL_RESULT_LEN);

pub(crate) fn enrol_identity_candidates(
    bytes: &[u8],
    user_id: i32,
    mut visit: impl FnMut(usize, [u8; IDENTITY_UUID_LEN]),
) {
    if bytes.len() < ER_IDENTITY_LEN {
        return;
    }
    let wanted = user_id.to_le_bytes();
    for at in 0..=bytes.len() - ER_IDENTITY_LEN {
        if bytes[at..at + 4] != wanted {
            continue;
        }
        let mut uuid = [0u8; IDENTITY_UUID_LEN];
        uuid.copy_from_slice(&bytes[at + 4..at + ER_IDENTITY_LEN]);
        if uuid.iter().all(|&b| b == 0) {
            continue;
        }
        visit(at, uuid);
    }
}

pub(crate) struct EnrolmentResult {
    progress_raw: u8,
    has_template: u32,
    complete: u32,
}

impl EnrolmentResult {
    pub(crate) fn parse(bytes: &[u8]) -> Option<EnrolmentResult> {
        if bytes.len() < SBIO_ENROL_RESULT_LEN {
            return None;
        }
        Some(EnrolmentResult {
            progress_raw: bytes[ER_PROGRESS],
            has_template: u32::from_le_bytes([
                bytes[ER_HAS_TEMPLATE],
                bytes[ER_HAS_TEMPLATE + 1],
                bytes[ER_HAS_TEMPLATE + 2],
                bytes[ER_HAS_TEMPLATE + 3],
            ]),
            complete: u32::from_le_bytes([
                bytes[ER_COMPLETE],
                bytes[ER_COMPLETE + 1],
                bytes[ER_COMPLETE + 2],
                bytes[ER_COMPLETE + 3],
            ]),
        })
    }

    pub(crate) fn progress_percent(&self) -> u32 {
        (u32::from(self.progress_raw) * 100 + 127) / 255
    }

    pub(crate) fn has_template(&self) -> bool {
        self.has_template != 0
    }

    pub(crate) fn complete(&self) -> bool {
        self.complete != 0
    }
}

#[derive(Clone, Copy)]
pub(crate) struct CatacombUser(i32);

impl CatacombUser {
    pub(crate) const MASTER: CatacombUser = CatacombUser(-1);
    pub(crate) const OWNER: CatacombUser = CatacombUser(501);

    pub(crate) fn enrolling(user: UserId) -> CatacombUser {
        CatacombUser(user.value())
    }

    pub(crate) const fn value(&self) -> i32 {
        self.0
    }
}
static_assert!(CatacombUser::MASTER.value() == -1);
static_assert!(CatacombUser::OWNER.value() == 501);
static_assert!(CatacombUser::MASTER.value() != CatacombUser::OWNER.value());
static_assert!(CatacombUser::MASTER.value() < 0);

pub(crate) const SBIO_SAVED_USER_ID_AT: usize = 8;
static_assert!(SBIO_SAVED_USER_ID_AT + 4 <= SBIO_SAVED_MIN);

pub(crate) const SBIO_SAVE_SELECTOR_LEN: usize = 24;
const SS_USER_ID: usize = 0;
static_assert!(SBIO_SAVE_SELECTOR_LEN <= SBIO_MAX_PAYLOAD);

pub(crate) struct SaveSelector([u8; SBIO_SAVE_SELECTOR_LEN]);

impl SaveSelector {
    pub(crate) fn new(user: CatacombUser) -> SaveSelector {
        let mut out = [0u8; SBIO_SAVE_SELECTOR_LEN];
        out[SS_USER_ID..SS_USER_ID + 4].copy_from_slice(&user.value().to_le_bytes());
        SaveSelector(out)
    }

    pub(crate) fn bytes(&self) -> &[u8; SBIO_SAVE_SELECTOR_LEN] {
        &self.0
    }
}

pub(crate) const SBIO_STATUS_COLD_TRANSITION: u32 = 0x8002;
static_assert!(SBIO_STATUS_COLD_TRANSITION != 0);

// 0x101 from a 0x6d load = already active; a success answer, not an error
pub(crate) const SBIO_STATUS_ALREADY_ACTIVE: u32 = 0x101;
static_assert!(SBIO_STATUS_ALREADY_ACTIVE != 0);
static_assert!(SBIO_STATUS_ALREADY_ACTIVE != SBIO_STATUS_COLD_TRANSITION);

pub(crate) const COMPONENT_STATE_COLD: u32 = 0x1;
pub(crate) const COMPONENT_STATE_ACTIVE: u32 = 0x2;
pub(crate) const COMPONENT_STATE_SAVE_PENDING: u32 = 0x4;
static_assert!(COMPONENT_STATE_COLD != COMPONENT_STATE_ACTIVE);
static_assert!(COMPONENT_STATE_SAVE_PENDING & (COMPONENT_STATE_COLD | COMPONENT_STATE_ACTIVE) == 0);

pub(crate) const COMPONENT_PAIR_LEN: usize = 8;

pub(crate) struct ComponentStates<'a>(&'a [u8]);

impl<'a> ComponentStates<'a> {
    pub(crate) fn new(reply: &'a [u8]) -> ComponentStates<'a> {
        ComponentStates(reply)
    }

    pub(crate) fn count(&self) -> usize {
        self.0.len() / COMPONENT_PAIR_LEN
    }

    pub(crate) fn pair(&self, i: usize) -> Option<(i32, u32)> {
        let at = i.checked_mul(COMPONENT_PAIR_LEN)?;
        let bytes = self.0.get(at..at + COMPONENT_PAIR_LEN)?;
        Some((
            i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
        ))
    }

    pub(crate) fn state_for(&self, id: i32) -> Option<u32> {
        (0..self.count())
            .filter_map(|i| self.pair(i))
            .find(|(pair_id, _)| *pair_id == id)
            .map(|(_, state)| state)
    }
}

pub(crate) enum ComponentAction {
    AlreadyActive,
    Load,
    Unsupported,
}

pub(crate) fn component_action(state: u32) -> ComponentAction {
    if state & COMPONENT_STATE_ACTIVE != 0 {
        ComponentAction::AlreadyActive
    } else if state & COMPONENT_STATE_COLD != 0 {
        ComponentAction::Load
    } else {
        ComponentAction::Unsupported
    }
}

const OP_SBIO_SAVE_LOCKOUT: u16 = 0x70;
pub(crate) fn sbio_save_lockout() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_SAVE_LOCKOUT,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"SAVE_LOCKOUT",
    }
}

pub(crate) struct SbioLoadLockout<'a>(&'a [u8]);

impl<'a> SbioLoadLockout<'a> {
    pub(crate) fn new(blob: &'a [u8]) -> Option<SbioLoadLockout<'a>> {
        if blob.is_empty() || blob.len() > SBIO_SAVED_MAX {
            return None;
        }
        Some(SbioLoadLockout(blob))
    }

    pub(crate) fn opcode(&self) -> u16 {
        OP_SBIO_LOAD_LOCKOUT
    }

    pub(crate) fn payload(&self) -> &[u8] {
        self.0
    }

    pub(crate) fn name(&self) -> &'static CStr {
        c"LOAD_LOCKOUT"
    }
}
const OP_SBIO_LOAD_LOCKOUT: u16 = 0x71;

const OP_SBIO_SAVE_CATACOMB: u16 = 0x6c;
pub(crate) fn sbio_save_catacomb(selector: &SaveSelector) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_SAVE_CATACOMB,
        payload: pad_payload(selector.bytes()),
        payload_len: SBIO_SAVE_SELECTOR_LEN,
        name: c"SAVE_CATACOMB",
    }
}

const OP_SBIO_CONFIRM_SAVE: u16 = 0x37;
pub(crate) fn sbio_confirm_save(selector: &SaveSelector) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CONFIRM_SAVE,
        payload: pad_payload(selector.bytes()),
        payload_len: SBIO_SAVE_SELECTOR_LEN,
        name: c"CONFIRM_SAVE",
    }
}

pub(crate) const SBIO_SAVED_MIN: usize = 0x22;
pub(crate) const SBIO_SAVED_MAX: usize = 0x4b000;
static_assert!(SBIO_SAVED_MIN < SBIO_SAVED_MAX);
static_assert!(SBIO_SAVED_MAX as u32 <= transfer::MAX_TRANSACTION);

pub(crate) struct SbioLoadCatacomb<'a>(&'a [u8]);

impl<'a> SbioLoadCatacomb<'a> {
    pub(crate) fn new(blob: &'a [u8]) -> Option<SbioLoadCatacomb<'a>> {
        if blob.len() < SBIO_SAVED_MIN || blob.len() > SBIO_SAVED_MAX {
            return None;
        }
        Some(SbioLoadCatacomb(blob))
    }

    pub(crate) fn opcode(&self) -> u16 {
        OP_SBIO_LOAD_CATACOMB
    }

    pub(crate) fn payload(&self) -> &[u8] {
        self.0
    }

    pub(crate) fn name(&self) -> &'static CStr {
        c"LOAD_CATACOMB"
    }
}
const OP_SBIO_LOAD_CATACOMB: u16 = 0x6d;

pub(crate) const IDENTITY_RECORD_LEN: usize = 40;
const IR_SELECTOR: usize = 20;
const IR_SELECTOR_BUILTIN: u32 = 1;
static_assert!(IR_SELECTOR == IDENTITY_V1_LEN);
static_assert!(IR_SELECTOR + 4 <= IDENTITY_RECORD_LEN);

const OP_SBIO_LIST_IDENTITIES: u16 = 0x7a;
pub(crate) fn sbio_list_identities() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_LIST_IDENTITIES,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"LIST_IDENTITIES",
    }
}

pub(crate) struct IdentityRecords<'a>(&'a [u8]);

impl<'a> IdentityRecords<'a> {
    pub(crate) fn new(reply: &'a [u8]) -> Option<IdentityRecords<'a>> {
        if reply.len() % IDENTITY_RECORD_LEN != 0 {
            return None;
        }
        Some(IdentityRecords(reply))
    }

    pub(crate) fn count(&self) -> usize {
        self.0.len() / IDENTITY_RECORD_LEN
    }

    pub(crate) fn record(&self, i: usize) -> Option<(IdentityV1, bool)> {
        let at = i.checked_mul(IDENTITY_RECORD_LEN)?;
        let bytes = self.0.get(at..at + IDENTITY_RECORD_LEN)?;
        let user_id = i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let mut uuid = [0u8; IDENTITY_UUID_LEN];
        uuid.copy_from_slice(&bytes[4..IDENTITY_V1_LEN]);
        let selector = u32::from_le_bytes([
            bytes[IR_SELECTOR],
            bytes[IR_SELECTOR + 1],
            bytes[IR_SELECTOR + 2],
            bytes[IR_SELECTOR + 3],
        ]);
        Some((
            IdentityV1 { user_id, uuid },
            selector == IR_SELECTOR_BUILTIN,
        ))
    }

    pub(crate) fn lists_uuid(&self, uuid: &[u8; IDENTITY_UUID_LEN]) -> bool {
        (0..self.count())
            .filter_map(|i| self.record(i))
            .any(|(identity, _)| identity.uuid == *uuid)
    }

    pub(crate) fn each_built_in_for(&self, user_id: i32, mut visit: impl FnMut(IdentityV1)) {
        for i in 0..self.count() {
            if let Some((identity, built_in)) = self.record(i) {
                if built_in && identity.user_id == user_id {
                    visit(identity);
                }
            }
        }
    }
}

const FIC_DEVICE_BUILTIN: u32 = 1;
pub(crate) const SBIO_FREE_COUNT_LEN: usize = 4 + 20;
static_assert!(SBIO_FREE_COUNT_LEN == SBIO_SAVE_SELECTOR_LEN);
pub(crate) const SBIO_FREE_COUNT_REPLY_LEN: usize = 4;
pub(crate) const SBIO_FREE_COUNT_MAX: u32 = 3;

const OP_SBIO_FREE_IDENTITY_COUNT: u16 = 0x38;
pub(crate) fn sbio_free_identity_count(user: UserId) -> SbioOp {
    let mut body = [0u8; SBIO_FREE_COUNT_LEN];
    body[0..4].copy_from_slice(&user.value().to_le_bytes());
    body[4..8].copy_from_slice(&FIC_DEVICE_BUILTIN.to_le_bytes());
    SbioOp {
        opcode: OP_SBIO_FREE_IDENTITY_COUNT,
        payload: pad_payload(&body),
        payload_len: SBIO_FREE_COUNT_LEN,
        name: c"FREE_IDENTITY_COUNT",
    }
}

const OP_SBIO_DELETE_IDENTITY: u16 = 0x57;
pub(crate) fn sbio_delete_identity(identity: &IdentityV1) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_DELETE_IDENTITY,
        payload: pad_payload(&identity.wire()),
        payload_len: IDENTITY_V1_LEN,
        name: c"DELETE_IDENTITY",
    }
}

const OP_SBIO_MATCH_RESULT: u16 = 0x09;
pub(crate) fn sbio_match_result() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_MATCH_RESULT,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"MATCH_RESULT",
    }
}

pub(crate) const SBIO_MATCH_RESULT_LEN: usize = 0xca2;

const MR_USER_ID: usize = 0x000;
const MR_IDENTITY: usize = 0x004;
static_assert!(MR_USER_ID + 4 <= SBIO_MATCH_RESULT_LEN);
static_assert!(MR_IDENTITY + IDENTITY_UUID_LEN <= SBIO_MATCH_RESULT_LEN);
static_assert!(MR_IDENTITY == MR_USER_ID + 4);
static_assert!(SBIO_MATCH_RESULT_LEN != SBIO_ENROL_RESULT_LEN);

pub(crate) const IDENTITY_UUID_LEN: usize = 16;
pub(crate) const IDENTITY_V1_LEN: usize = 4 + IDENTITY_UUID_LEN;
static_assert!(IDENTITY_V1_LEN == 0x14);

#[derive(Clone, Copy)]
pub(crate) struct IdentityV1 {
    user_id: i32,
    uuid: [u8; IDENTITY_UUID_LEN],
}

impl IdentityV1 {
    pub(crate) fn from_index(user_id: i32, uuid: [u8; IDENTITY_UUID_LEN]) -> IdentityV1 {
        IdentityV1 { user_id, uuid }
    }

    pub(crate) fn uuid(&self) -> &[u8; IDENTITY_UUID_LEN] {
        &self.uuid
    }

    pub(crate) fn wire(&self) -> [u8; IDENTITY_V1_LEN] {
        let mut out = [0u8; IDENTITY_V1_LEN];
        out[0..4].copy_from_slice(&self.user_id.to_le_bytes());
        out[4..IDENTITY_V1_LEN].copy_from_slice(&self.uuid);
        out
    }
}

pub(crate) struct MatchResult {
    user_id: i32,
    identity: [u8; IDENTITY_UUID_LEN],
}

impl MatchResult {
    pub(crate) fn parse(bytes: &[u8]) -> Option<MatchResult> {
        if bytes.len() < SBIO_MATCH_RESULT_LEN {
            return None;
        }
        let mut identity = [0u8; IDENTITY_UUID_LEN];
        identity.copy_from_slice(&bytes[MR_IDENTITY..MR_IDENTITY + IDENTITY_UUID_LEN]);
        Some(MatchResult {
            user_id: i32::from_le_bytes([
                bytes[MR_USER_ID],
                bytes[MR_USER_ID + 1],
                bytes[MR_USER_ID + 2],
                bytes[MR_USER_ID + 3],
            ]),
            identity,
        })
    }

    pub(crate) fn identity_uuid(&self) -> &[u8; IDENTITY_UUID_LEN] {
        &self.identity
    }

    pub(crate) fn matches(&self, user: UserId) -> bool {
        self.user_id == user.value()
    }
}

const OP_SBIO_IMAGE_CLEANUP: u16 = 0x22;
pub(crate) fn sbio_image_cleanup() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_IMAGE_CLEANUP,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"IMAGE_CLEANUP",
    }
}

const OP_SBIO_CANCEL: u16 = 0x05;
pub(crate) fn sbio_cancel_operation() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CANCEL,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"CANCEL_OPERATION",
    }
}

const OP_SBIO_CLEAR_STATE: u16 = 0x19;
pub(crate) fn sbio_clear_state() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CLEAR_STATE,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"CLEAR_STATE",
    }
}

const OP_SBIO_REGISTER_SERIAL: u16 = 0x17;
pub(crate) fn sbio_register_sensor_serial(serial: &crate::sensor::SensorSerial) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_REGISTER_SERIAL,
        payload: pad_payload(serial.bytes()),
        payload_len: crate::sensor::SENSOR_SERIAL_LEN,
        name: c"REGISTER_SERIAL",
    }
}

const OP_SBIO_DIAGNOSTICS: u16 = 0x63;
pub(crate) const fn sbio_diagnostics() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_DIAGNOSTICS,
        payload: [0; SBIO_MAX_PAYLOAD],
        payload_len: 0,
        name: c"DIAGNOSTICS",
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum ImagePurpose {
    Enrolment,
    Matching,
}

impl ImagePurpose {
    const fn offset(self) -> usize {
        match self {
            ImagePurpose::Matching => 0x12,
            ImagePurpose::Enrolment => 0x13,
        }
    }

    const fn last_image_offset(self) -> usize {
        match self {
            ImagePurpose::Enrolment => 0x08,
            ImagePurpose::Matching => 0x09,
        }
    }
}

static_assert!(ImagePurpose::Enrolment.offset() != ImagePurpose::Matching.offset());
static_assert!(ImagePurpose::Matching.offset() == 0x12);
static_assert!(ImagePurpose::Enrolment.offset() == 0x13);
static_assert!(ImagePurpose::Enrolment.last_image_offset() == 0x08);
static_assert!(ImagePurpose::Matching.last_image_offset() == 0x09);
static_assert!(
    ImagePurpose::Enrolment.last_image_offset() != ImagePurpose::Matching.last_image_offset()
);
static_assert!(ImagePurpose::Matching.last_image_offset() < ImagePurpose::Matching.offset());

#[derive(Clone, Copy)]
pub(crate) struct UserId(i32);

impl UserId {
    pub(crate) fn new(value: i32) -> Option<UserId> {
        if value > 0 {
            Some(UserId(value))
        } else {
            None
        }
    }

    pub(crate) const fn value(&self) -> i32 {
        self.0
    }
}

const IPI_FIRST_IMAGE: usize = 0x07;
const IPI_TIMESTAMP: usize = 0x40;
const IPI_CAPTURE_COUNTER: usize = 0x48;
const IPI_DEVICE_KIND: usize = 0x4f;
const IPI_USER_ID: usize = 0x67;

const IPI_DEVICE_BUILTIN: u32 = 1;

static_assert!(IPI_USER_ID + 4 <= SBIO_MAX_PAYLOAD);
static_assert!(IPI_DEVICE_KIND + 4 <= SBIO_MAX_PAYLOAD);
static_assert!(IPI_TIMESTAMP + 8 <= SBIO_MAX_PAYLOAD);
static_assert!(IPI_CAPTURE_COUNTER + 4 <= SBIO_MAX_PAYLOAD);
static_assert!(IPI_FIRST_IMAGE < ImagePurpose::Matching.offset());
static_assert!(ImagePurpose::Enrolment.offset() < IPI_TIMESTAMP);

const OP_SBIO_PREPARE_IMAGE: u16 = 0x23;
pub(crate) fn sbio_prepare_image_processing() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_PREPARE_IMAGE,
        payload: [0; SBIO_MAX_PAYLOAD],
        payload_len: 0,
        name: c"PREPARE_IMAGE_PROCESSING",
    }
}

const OP_SBIO_IMAGE_INIT: u16 = 0x06;
pub(crate) fn sbio_image_processing_init(
    purpose: ImagePurpose,
    first_image: bool,
    last_image: bool,
    capture_counter: u32,
    user: UserId,
    monotonic_ns: u64,
) -> SbioOp {
    let mut payload = [0u8; SBIO_MAX_PAYLOAD];

    if first_image {
        payload[IPI_FIRST_IMAGE] = 1;
    }
    if last_image {
        payload[purpose.last_image_offset()] = 1;
    }
    payload[purpose.offset()] = 1;

    // ns*24/1000 split to dodge u64 overflow and the kernel's absent __udivti3
    let scaled = (monotonic_ns / 1000) * 24 + ((monotonic_ns % 1000) * 24) / 1000;
    payload[IPI_TIMESTAMP..IPI_TIMESTAMP + 8].copy_from_slice(&scaled.to_le_bytes());
    payload[IPI_CAPTURE_COUNTER..IPI_CAPTURE_COUNTER + 4]
        .copy_from_slice(&capture_counter.to_le_bytes());
    payload[IPI_DEVICE_KIND..IPI_DEVICE_KIND + 4]
        .copy_from_slice(&IPI_DEVICE_BUILTIN.to_le_bytes());
    payload[IPI_USER_ID..IPI_USER_ID + 4].copy_from_slice(&user.value().to_le_bytes());

    SbioOp {
        opcode: OP_SBIO_IMAGE_INIT,
        payload,
        payload_len: SBIO_IMAGE_INIT_LEN,
        name: c"IMAGE_PROCESSING_INIT",
    }
}

const OP_SBIO_RELAY_CAPTURE: u16 = 0x65;

pub(crate) struct SbioRelay<'a> {
    capture: &'a crate::sensor::Capture,
}

impl<'a> SbioRelay<'a> {
    pub(crate) fn new(capture: &'a crate::sensor::Capture) -> SbioRelay<'a> {
        SbioRelay { capture }
    }

    pub(crate) fn opcode(&self) -> u16 {
        OP_SBIO_RELAY_CAPTURE
    }

    pub(crate) fn payload(&self) -> &[u8] {
        self.capture.bytes()
    }

    pub(crate) fn name(&self) -> &'static CStr {
        c"RELAY_CAPTURE"
    }
}

const OP_SBIO_ASSESSMENT: u16 = 0x07;
pub(crate) fn sbio_image_assessment() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_ASSESSMENT,
        payload: [0; SBIO_MAX_PAYLOAD],
        payload_len: 0,
        name: c"IMAGE_ASSESSMENT",
    }
}

const OP_SBIO_ENROL_RESULT: u16 = 0x08;
pub(crate) fn sbio_enrolment_result() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_ENROL_RESULT,
        payload: [0; SBIO_MAX_PAYLOAD],
        payload_len: 0,
        name: c"ENROLMENT_RESULT",
    }
}

pub(crate) const ASSESS_MIN_LEN: usize = 0x89;
pub(crate) const ASSESS_USABLE_MATCH: usize = 0x06;
pub(crate) const ASSESS_USABLE_ENROL: usize = 0x07;

static_assert!(ASSESS_USABLE_MATCH < ASSESS_USABLE_ENROL);

pub(crate) const SBIO_SESSION_SHARE_LEN: usize = 40;

pub(crate) const SBIO_STATUS_OK: u16 = 0x00;
pub(crate) const SBIO_STATUS_PREREQUISITE: u16 = 0x01;
pub(crate) const SBIO_STATUS_16: u16 = 0x16;
static_assert!(SBIO_STATUS_PREREQUISITE != SBIO_STATUS_16);
pub(crate) const SBIO_SESSION_MODE: u32 = 1;
static_assert!(SBIO_SESSION_SHARE_LEN <= SBIO_MAX_PAYLOAD);

const OP_SBIO_SESSION_SHARE: u16 = 0x15;
pub(crate) fn sbio_request_session_share() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_SESSION_SHARE,
        payload: pad_payload(&SBIO_SESSION_MODE.to_le_bytes()),
        payload_len: 4,
        name: c"SESSION_SHARE",
    }
}

const OP_SBIO_COMMIT_SESSION: u16 = 0x16;
pub(crate) fn sbio_commit_session_share(share: &[u8; SBIO_SESSION_SHARE_LEN]) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_COMMIT_SESSION,
        payload: pad_payload(share),
        payload_len: SBIO_SESSION_SHARE_LEN,
        name: c"SESSION_COMMIT",
    }
}
pub(crate) const SBIO_CHALLENGE_LEN: usize = 64;
static_assert!(SBIO_CHALLENGE_LEN <= SBIO_MAX_PAYLOAD);

const OP_SBIO_CHALLENGE: u16 = 0x42;
pub(crate) fn sbio_request_challenge() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CHALLENGE,
        payload: [0; SBIO_MAX_PAYLOAD],
        payload_len: 0,
        name: c"SEQUENCE_CHALLENGE",
    }
}

const OP_SBIO_COMMIT_CHALLENGE: u16 = 0x18;
pub(crate) fn sbio_commit_challenge(reply: &[u8; SBIO_CHALLENGE_LEN]) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_COMMIT_CHALLENGE,
        payload: pad_payload(reply),
        payload_len: SBIO_CHALLENGE_LEN,
        name: c"SEQUENCE_COMMIT",
    }
}
const OP_SBIO_FETCH_PATCH: u16 = 0x5f;

const SBIO_PATCH_SUBTYPE: u16 = 2;

pub(crate) fn sbio_fetch_patch() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_FETCH_PATCH,
        payload: pad_payload(&SBIO_PATCH_SUBTYPE.to_le_bytes()),
        payload_len: 2,
        name: c"FETCH_SENSOR_PATCH",
    }
}

const fn encode_sbio(opcode: u16, marker: u8, seq: u16) -> Message {
    Message {
        msg0: (EP_SBIO as u64)
            | ((marker as u64) << MSG_TAG_SHIFT)
            | ((opcode as u64) << MSG_TYPE_SHIFT)
            | ((seq as u64) << 48),
        msg1: 0,
    }
}

static_assert!(encode_sbio(0x73, transfer::MARKER_FIRST, 0).msg0 == 0x0000_0073_fc08);

pub(crate) fn encode_sbio_raw(opcode: u16, seq: u16) -> Option<Message> {
    Some(encode_sbio(opcode, transfer::MARKER_FIRST, seq))
}

pub(crate) fn encode_sbio_next(opcode: u16, seq: u16) -> Option<Message> {
    Some(encode_sbio(opcode, transfer::MARKER_NEXT, seq))
}

pub(crate) fn encode_sbio_continue(cont: &transfer::Continuation) -> Message {
    encode_sbio(cont.opcode() as u16, transfer::MARKER_REQUEST, cont.seq())
}
