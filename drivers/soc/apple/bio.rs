// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

use crate::store::{Key, Store};
use kernel::ioctl::{_IO, _IOR, _IOW, _IOWR};
use kernel::prelude::*;
use kernel::uaccess::{UserPtr, UserSlice};

pub(crate) const IFACE_VERSION: u32 = 4;

pub(crate) const ENROL_USER_ID: i32 = 1000;

pub(crate) const UUID_LEN: usize = 16;
const LABEL_LEN: usize = 128;
const NONCE_LEN: usize = 32;
pub(crate) const TOKEN_LEN: usize = 32;
const MAX_IDENTITIES: usize = 32;
// The host index ABI has 32 entries, but this driver uses one SEP user context.
// Mac Touch ID permits only three fingerprints in a single user context.
const SEP_USER_CAPACITY: usize = 3;

pub(crate) const DEVICE_NAME: &CStr = c"sep-bio";
pub(crate) const DEVICE_MODE: u16 = 0o600;

#[derive(Clone, Copy, Default)]
#[repr(u32)]
enum State {
    #[default]
    Idle = 0,
    Pending = 1,
    Progress = 2,
    Done = 3,
    Failed = 4,
}

#[derive(Clone, Copy, Default, PartialEq)]
#[repr(u32)]
pub(crate) enum Guidance {
    #[default]
    None = 0,
    Place = 1,
    LiftAndMove = 2,
    HoldStill = 3,
}

#[derive(Clone, Copy, Default)]
#[repr(u32)]
enum MatchResult {
    #[default]
    NoMatch = 0,
    Match = 1,
    NotCompared = 2,
}

// A stage is an accepted capture, not a bucket of SEP's coverage percentage.
// Leave the final stage for the completed template. libfprint reports a touch
// only when the stage rises, so there is a stage for each capture the budget
// allows: every accepted capture short of the last raises it, and an enrolment
// that completes sooner jumps to the total.
pub(crate) const ENROL_STAGES: u32 = crate::ENROL_MAX_CAPTURES;

const TOKEN_LIFETIME_NS: u64 = 10 * 1_000_000_000;

const SUSPEND_SLACK_NS: u64 = 1_000_000;

const STATUS_LOCAL: u32 = 0;

#[repr(C)]
#[derive(Clone, Copy)]
pub(crate) struct Identity {
    uuid: [u8; UUID_LEN],
    label: [u8; LABEL_LEN],
}

impl Identity {
    const ZERO: Identity = Identity {
        uuid: [0; UUID_LEN],
        label: [0; LABEL_LEN],
    };
}

#[repr(C)]
#[derive(Default)]
struct Info {
    version: u32,
    sensor_present: u32,
    enrolled: u32,
    capacity: u32,
    enroll_stages: u32,
    reserved: [u32; 3],
}

#[repr(C)]
struct List {
    count: u32,
    reserved: u32,
    id: [Identity; MAX_IDENTITIES],
}

#[repr(C)]
struct EnrolStart {
    flags: u32,
    reserved: u32,
    label: [u8; LABEL_LEN],
}

#[repr(C)]
#[derive(Default)]
struct EnrolPoll {
    state: State,
    stage: u32,
    stages_total: u32,
    status: u32,
    uuid: [u8; UUID_LEN],
    guidance: Guidance,
    progress_percent: u32,
}

#[repr(C)]
struct VerifyStart {
    flags: u32,
    reserved: u32,
    nonce: [u8; NONCE_LEN],
}

#[repr(C)]
#[derive(Default)]
struct VerifyPoll {
    state: State,
    result: MatchResult,
    status: u32,
    guidance: Guidance,
    uuid: [u8; UUID_LEN],
    token: [u8; TOKEN_LEN],
    deadline_ns: u64,
}

#[repr(C)]
struct Delete {
    uuid: [u8; UUID_LEN],
}

pub(crate) const ATTEST_CHALLENGE_LEN: usize = 32;
/// Uncompressed P-256 public point (`04‖X‖Y`).
pub(crate) const ATTEST_PUB_LEN: usize = 65;
/// Max DER `SEQUENCE { INTEGER r, INTEGER s }` for P-256.
pub(crate) const ATTEST_SIG_MAX: usize = 72;

#[repr(C)]
pub(crate) struct Attest {
    pub(crate) sig_len: u32,
    /// in: challenge to sign.
    pub(crate) challenge: [u8; ATTEST_CHALLENGE_LEN],
    /// out: ref-key public point.
    pub(crate) public_key: [u8; ATTEST_PUB_LEN],
    /// out: DER signature, zero-padded to `ATTEST_SIG_MAX`.
    pub(crate) signature: [u8; ATTEST_SIG_MAX],
    pub(crate) reserved: [u8; 3],
}

// Sizes are baked into the ioctl numbers.
static_assert!(core::mem::size_of::<Identity>() == 144);
static_assert!(core::mem::size_of::<Info>() == 32);
static_assert!(core::mem::size_of::<List>() == 4616);
static_assert!(core::mem::size_of::<EnrolStart>() == 136);
static_assert!(core::mem::size_of::<EnrolPoll>() == 40);
static_assert!(core::mem::size_of::<VerifyStart>() == 40);
static_assert!(core::mem::size_of::<VerifyPoll>() == 72);
static_assert!(core::mem::size_of::<Delete>() == 16);
static_assert!(core::mem::size_of::<Attest>() == 176);

macro_rules! ioctl_pod {
    (to_user: $($t:ty),*; from_user: $($u:ty),*) => {
        $(
            // SAFETY: no padding, so the value is a faithful byte image.
            unsafe impl kernel::transmute::AsBytes for $t {}
        )*
        $(
            // SAFETY: every bit pattern of the members is a valid value.
            unsafe impl kernel::transmute::FromBytes for $u {}
        )*
    };
}
ioctl_pod! {
    to_user: Info, List, EnrolPoll, VerifyPoll, Attest;
    from_user: EnrolStart, VerifyStart, Delete, Attest
}

const MAGIC: u32 = 0xB1;

pub(crate) const IOC_GET_INFO: u32 = _IOR::<Info>(MAGIC, 0x01);
const IOC_LIST: u32 = _IOR::<List>(MAGIC, 0x02);
pub(crate) const IOC_ENROL_START: u32 = _IOW::<EnrolStart>(MAGIC, 0x03);
const IOC_ENROL_POLL: u32 = _IOR::<EnrolPoll>(MAGIC, 0x04);
const IOC_VERIFY_START: u32 = _IOW::<VerifyStart>(MAGIC, 0x05);
const IOC_VERIFY_POLL: u32 = _IOR::<VerifyPoll>(MAGIC, 0x06);
const IOC_CANCEL: u32 = _IO(MAGIC, 0x07);
const IOC_DELETE: u32 = _IOW::<Delete>(MAGIC, 0x08);
const IOC_DELETE_ALL: u32 = _IO(MAGIC, 0x09);
pub(crate) const IOC_ATTEST: u32 = _IOWR::<Attest>(MAGIC, 0x0a);

pub(crate) struct MatchEvidence {
    identity: [u8; UUID_LEN],
}

impl MatchEvidence {
    pub(crate) fn from_enclave_reply(identity: [u8; UUID_LEN]) -> MatchEvidence {
        MatchEvidence { identity }
    }
}

pub(crate) enum VerifyOutcome {
    Failed(u32),
    NoMatch,
    Matched(MatchEvidence),
}

struct ResultToken {
    bytes: [u8; TOKEN_LEN],
    nonce: [u8; NONCE_LEN],
    identity: [u8; UUID_LEN],
    deadline_ns: u64,
    minted_mono_ns: u64,
    minted_boot_ns: u64,
}

impl ResultToken {
    fn mint(
        evidence: &MatchEvidence,
        nonce: &[u8; NONCE_LEN],
        bytes: [u8; TOKEN_LEN],
    ) -> ResultToken {
        let mono = crate::shim::monotonic_ns();
        ResultToken {
            bytes,
            nonce: *nonce,
            identity: evidence.identity,
            deadline_ns: mono.saturating_add(TOKEN_LIFETIME_NS),
            minted_mono_ns: mono,
            minted_boot_ns: crate::shim::boottime_ns(),
        }
    }

    fn usable(&self, nonce: &[u8; NONCE_LEN], identity: &[u8; UUID_LEN]) -> bool {
        let mono = crate::shim::monotonic_ns();
        if mono > self.deadline_ns {
            return false;
        }
        let suspended = crate::shim::boottime_ns().saturating_sub(self.minted_boot_ns);
        let awake = mono.saturating_sub(self.minted_mono_ns);
        if suspended.saturating_sub(awake) > SUSPEND_SLACK_NS {
            return false;
        }
        self.nonce == *nonce && self.identity == *identity
    }
}

const PRIVATE_TYPE_IDENTITIES: u8 = 0xF1;

fn identity_key() -> Key {
    Key::root(PRIVATE_TYPE_IDENTITIES)
}

const INDEX_FORMAT: u32 = 2;

pub(crate) struct IdentityIndex {
    entries: KVec<Identity>,
}

impl IdentityIndex {
    pub(crate) fn new() -> IdentityIndex {
        IdentityIndex {
            entries: KVec::new(),
        }
    }

    pub(crate) fn load(store: &mut Store) -> Result<IdentityIndex> {
        let mut index = IdentityIndex::new();
        let Some(raw) = store.read(&identity_key())? else {
            return Ok(index);
        };
        if raw.len() < 8 {
            return Err(EINVAL);
        }
        if u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]) != INDEX_FORMAT {
            return Err(EINVAL);
        }
        let count = u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]) as usize;
        let stride = UUID_LEN + LABEL_LEN;
        if count > MAX_IDENTITIES || raw.len() != 8 + count * stride {
            return Err(EINVAL);
        }
        for i in 0..count {
            let at = 8 + i * stride;
            let mut entry = Identity::ZERO;
            entry.uuid.copy_from_slice(&raw[at..at + UUID_LEN]);
            entry
                .label
                .copy_from_slice(&raw[at + UUID_LEN..at + stride]);
            index.entries.push(entry, GFP_KERNEL)?;
        }
        Ok(index)
    }

    fn save(&self, store: &mut Store) -> Result<()> {
        let mut raw = KVec::new();
        raw.extend_from_slice(&INDEX_FORMAT.to_le_bytes(), GFP_KERNEL)?;
        raw.extend_from_slice(&(self.entries.len() as u32).to_le_bytes(), GFP_KERNEL)?;
        for entry in self.entries.iter() {
            raw.extend_from_slice(&entry.uuid, GFP_KERNEL)?;
            raw.extend_from_slice(&entry.label, GFP_KERNEL)?;
        }
        store.write(&identity_key(), &raw)
    }

    pub(crate) fn total(&self) -> usize {
        self.entries.len()
    }

    pub(crate) fn insert(&mut self, id: Identity) -> Result<()> {
        for existing in self.entries.iter_mut() {
            if existing.uuid == id.uuid {
                *existing = id;
                return Ok(());
            }
        }
        if self.entries.len() >= MAX_IDENTITIES {
            return Err(ENOSPC);
        }
        self.entries.push(id, GFP_KERNEL)?;
        Ok(())
    }

    pub(crate) fn persist(&self, store: &mut Store) -> Result<()> {
        self.save(store)
    }

    pub(crate) fn identity_v1_for(
        &self,
        uuid: &[u8; UUID_LEN],
        user_id: i32,
    ) -> Option<crate::sbio::IdentityV1> {
        if !self.contains(uuid) {
            return None;
        }
        Some(crate::sbio::IdentityV1::from_index(user_id, *uuid))
    }

    pub(crate) fn reconcile_to(
        &mut self,
        listed: &[[u8; UUID_LEN]],
    ) -> Result<KVec<[u8; UUID_LEN]>> {
        let mut dropped = KVec::new();
        for entry in self.entries.iter() {
            if !listed.contains(&entry.uuid) {
                dropped.push(entry.uuid, GFP_KERNEL)?;
            }
        }
        self.entries.retain(|e| listed.contains(&e.uuid));
        for uuid in listed.iter() {
            if !self.contains(uuid) {
                self.insert(Identity {
                    uuid: *uuid,
                    label: [0u8; LABEL_LEN],
                })?;
            }
        }
        Ok(dropped)
    }

    pub(crate) fn remove(&mut self, uuid: &[u8; UUID_LEN]) {
        self.entries.retain(|e| e.uuid != *uuid);
    }

    pub(crate) fn contains_uuid(&self, uuid: &[u8; UUID_LEN]) -> bool {
        self.contains(uuid)
    }

    fn contains(&self, uuid: &[u8; UUID_LEN]) -> bool {
        self.entries.iter().any(|e| e.uuid == *uuid)
    }
}

enum Op {
    Idle,
    Enrol {
        label: [u8; LABEL_LEN],
        stage: u32,
        terminal: Option<EnrolOutcome>,
        guidance: Guidance,
        percent: u32,
    },
    Verify {
        nonce: [u8; NONCE_LEN],
        terminal: Option<VerifyOutcome>,
        guidance: Guidance,
    },
}

enum EnrolOutcome {
    Failed(u32),
    Done([u8; UUID_LEN]),
}

impl Op {
    fn is_terminal(&self) -> bool {
        match self {
            Op::Idle => false,
            Op::Enrol { terminal, .. } => terminal.is_some(),
            Op::Verify { terminal, .. } => terminal.is_some(),
        }
    }

    fn may_start(&self) -> bool {
        matches!(self, Op::Idle) || self.is_terminal()
    }
}

pub(crate) struct Session {
    generation: u64,
    worker_epoch: Option<u64>,
    open: bool,
    op: Op,
    unseen: bool,
    enrol_stage_unseen: bool,
    token: Option<ResultToken>,
}

impl Session {
    pub(crate) fn new() -> Session {
        Session {
            generation: 0,
            worker_epoch: None,
            open: false,
            op: Op::Idle,
            unseen: false,
            enrol_stage_unseen: false,
            token: None,
        }
    }

    fn reset(&mut self) {
        self.generation = self.generation.saturating_add(1);
        self.open = false;
        self.op = Op::Idle;
        self.unseen = false;
        self.enrol_stage_unseen = false;
        self.token = None;
    }

    fn start_worker(&mut self) -> Result<()> {
        if self.worker_epoch.is_some() {
            return Err(EBUSY);
        }
        self.generation = self.generation.checked_add(1).ok_or(EOVERFLOW)?;
        self.worker_epoch = Some(self.generation);
        Ok(())
    }

    pub(crate) fn ready(&self) -> bool {
        self.unseen
    }
}

pub(crate) struct Context<'a> {
    pub(crate) session: &'a mut Session,
    pub(crate) index: &'a IdentityIndex,
    pub(crate) sensor_present: bool,
    pub(crate) live_identity_count: Option<usize>,
}

pub(crate) struct Handled {
    pub(crate) ret: isize,
    pub(crate) wake: bool,
    pub(crate) start_enrol: bool,
    pub(crate) start_verify: bool,
    pub(crate) delete_identity: Option<crate::sbio::IdentityV1>,
    pub(crate) delete_identities: KVec<crate::sbio::IdentityV1>,
}

fn ok() -> Result<Handled> {
    Ok(Handled {
        ret: 0,
        wake: false,
        start_enrol: false,
        start_verify: false,
        delete_identity: None,
        delete_identities: KVec::new(),
    })
}

fn require_admin() -> Result<()> {
    if crate::shim::capable_admin() {
        Ok(())
    } else {
        Err(EPERM)
    }
}

fn require_sensor(present: bool) -> Result<()> {
    if present {
        Ok(())
    } else {
        Err(ENODEV)
    }
}

pub(crate) fn ioctl(ctx: &mut Context<'_>, cmd: u32, arg: usize) -> Result<Handled> {
    let user = UserPtr::from_addr(arg);

    match cmd {
        IOC_GET_INFO => get_info(ctx, user),
        IOC_LIST => list(ctx, user),
        IOC_ENROL_START => enrol_start(ctx, user),
        IOC_ENROL_POLL => enrol_poll(ctx, user),
        IOC_VERIFY_START => verify_start(ctx, user),
        IOC_VERIFY_POLL => verify_poll(ctx, user),
        IOC_CANCEL => cancel(ctx),
        IOC_DELETE => delete(ctx, user),
        IOC_DELETE_ALL => delete_all(ctx),
        _ => Err(ENOTTY),
    }
}

fn get_info(ctx: &mut Context<'_>, user: UserPtr) -> Result<Handled> {
    let info = Info {
        version: IFACE_VERSION,
        sensor_present: u32::from(ctx.sensor_present),
        enrolled: ctx.index.total() as u32,
        capacity: SEP_USER_CAPACITY as u32,
        enroll_stages: ENROL_STAGES,
        // reserved[0] is a validity flag. A zero count is only meaningful
        // when SEP itself successfully answered LIST_IDENTITIES; older
        // kernels and unavailable SEP contexts leave this flag clear.
        reserved: [
            u32::from(ctx.live_identity_count.is_some()),
            ctx.live_identity_count.unwrap_or(0) as u32,
            0,
        ],
    };
    UserSlice::new(user, core::mem::size_of::<Info>())
        .writer()
        .write(&info)?;
    ok()
}

fn list(ctx: &mut Context<'_>, user: UserPtr) -> Result<Handled> {
    let mut out = List {
        count: 0,
        reserved: 0,
        id: [Identity::ZERO; MAX_IDENTITIES],
    };
    for entry in ctx.index.entries.iter() {
        if out.count as usize >= MAX_IDENTITIES {
            break;
        }
        out.id[out.count as usize] = *entry;
        out.count += 1;
    }

    UserSlice::new(user, core::mem::size_of::<List>())
        .writer()
        .write(&out)?;
    ok()
}

fn enrol_start(ctx: &mut Context<'_>, user: UserPtr) -> Result<Handled> {
    require_admin()?;
    let request: EnrolStart = UserSlice::new(user, core::mem::size_of::<EnrolStart>())
        .reader()
        .read()?;
    if request.flags != 0 || request.reserved != 0 {
        return Err(EINVAL);
    }
    require_sensor(ctx.sensor_present)?;

    if ctx.session.worker_epoch.is_some() || !ctx.session.op.may_start() {
        return Err(EBUSY);
    }
    if matches!(ctx.live_identity_count, Some(count) if count >= SEP_USER_CAPACITY)
        || ctx.index.total() >= MAX_IDENTITIES
    {
        return Err(ENOSPC);
    }

    ctx.session.start_worker()?;
    ctx.session.op = Op::Enrol {
        label: request.label,
        stage: 0,
        terminal: None,
        guidance: Guidance::Place,
        percent: 0,
    };
    ctx.session.unseen = false;
    ctx.session.enrol_stage_unseen = false;
    Ok(Handled {
        ret: 0,
        wake: false,
        start_enrol: true,
        start_verify: false,
        delete_identity: None,
        delete_identities: KVec::new(),
    })
}

pub(crate) fn enrol_advance(
    session: &mut Session,
    epoch: u64,
    completed: u32,
    percent_now: u32,
    guide: Guidance,
) -> bool {
    if !worker_is_current(session, epoch) {
        return false;
    }
    if let Op::Enrol {
        stage,
        terminal,
        guidance,
        percent,
        ..
    } = &mut session.op
    {
        if terminal.is_none() {
            *stage = completed;
            *percent = percent_now;
            *guidance = guide;
            session.unseen = true;
            session.enrol_stage_unseen = true;
            return true;
        }
    }
    false
}

pub(crate) fn capture_guide(session: &mut Session, epoch: u64, guide: Guidance) -> bool {
    if !worker_is_current(session, epoch) {
        return false;
    }
    let changed = match &mut session.op {
        Op::Enrol { terminal, guidance, .. } => {
            if terminal.is_none() && *guidance != guide {
                *guidance = guide;
                true
            } else {
                false
            }
        }
        Op::Verify { terminal, guidance, .. } => {
            if terminal.is_none() && *guidance != guide {
                *guidance = guide;
                true
            } else {
                false
            }
        }
        Op::Idle => false,
    };
    if changed {
        session.unseen = true;
    }
    changed
}

pub(crate) fn enrol_finish(
    session: &mut Session,
    epoch: u64,
    index: &mut IdentityIndex,
    outcome: core::result::Result<[u8; UUID_LEN], u32>,
) -> bool {
    if !worker_is_current(session, epoch) {
        return false;
    }
    let Op::Enrol {
        label, terminal, ..
    } = &mut session.op
    else {
        return false;
    };
    if terminal.is_some() {
        return false;
    }
    match outcome {
        Ok(uuid) => {
            match index.insert(Identity {
                uuid,
                label: *label,
            }) {
                Ok(()) => *terminal = Some(EnrolOutcome::Done(uuid)),
                Err(_) => *terminal = Some(EnrolOutcome::Failed(STATUS_LOCAL)),
            }
        }
        Err(status) => *terminal = Some(EnrolOutcome::Failed(status)),
    }
    session.unseen = true;
    true
}

pub(crate) fn enrol_is_live(session: &Session, epoch: u64) -> bool {
    worker_is_current(session, epoch) && matches!(&session.op, Op::Enrol { terminal: None, .. })
}

fn enrol_poll(ctx: &mut Context<'_>, user: UserPtr) -> Result<Handled> {
    let mut out = EnrolPoll {
        stages_total: ENROL_STAGES,
        ..EnrolPoll::default()
    };

    match &ctx.session.op {
        Op::Idle => out.state = State::Idle,
        Op::Verify { .. } => return Err(EBUSY),
        Op::Enrol {
            stage,
            terminal,
            guidance,
            percent,
            ..
        } => {
            out.stage = *stage;
            out.guidance = *guidance;
            out.progress_percent = *percent;
            match terminal {
                None => {
                    // Guidance also wakes the poller, but it is not an image
                    // capture or a completed enrollment stage.
                    out.state = if ctx.session.enrol_stage_unseen {
                        State::Progress
                    } else {
                        State::Pending
                    }
                }
                Some(EnrolOutcome::Failed(status)) => {
                    out.state = State::Failed;
                    out.status = *status;
                }
                Some(EnrolOutcome::Done(uuid)) => {
                    out.state = State::Done;
                    out.uuid = *uuid;
                }
            }
        }
    }

    UserSlice::new(user, core::mem::size_of::<EnrolPoll>())
        .writer()
        .write(&out)?;
    ctx.session.unseen = false;
    ctx.session.enrol_stage_unseen = false;
    ok()
}

fn verify_start(ctx: &mut Context<'_>, user: UserPtr) -> Result<Handled> {
    let request: VerifyStart = UserSlice::new(user, core::mem::size_of::<VerifyStart>())
        .reader()
        .read()?;
    if request.flags != 0 || request.reserved != 0 {
        return Err(EINVAL);
    }
    require_sensor(ctx.sensor_present)?;

    if ctx.session.worker_epoch.is_some() || !ctx.session.op.may_start() {
        return Err(EBUSY);
    }

    if ctx.index.total() == 0 {
        ctx.session.token = None;
        ctx.session.op = Op::Verify {
            nonce: request.nonce,
            terminal: Some(VerifyOutcome::NoMatch),
            guidance: Guidance::None,
        };
        ctx.session.unseen = true;
        return Ok(Handled {
            ret: 0,
            wake: true,
            start_enrol: false,
            start_verify: false,
            delete_identity: None,
            delete_identities: KVec::new(),
        });
    }

    ctx.session.start_worker()?;
    ctx.session.token = None;
    ctx.session.op = Op::Verify {
        nonce: request.nonce,
        terminal: None,
        guidance: Guidance::Place,
    };
    Ok(Handled {
        ret: 0,
        wake: false,
        start_enrol: false,
        start_verify: true,
        delete_identity: None,
        delete_identities: KVec::new(),
    })
}

pub(crate) fn verify_finish(
    session: &mut Session,
    epoch: u64,
    outcome: VerifyOutcome,
    token_bytes: [u8; TOKEN_LEN],
) -> bool {
    if !worker_is_current(session, epoch) {
        return false;
    }
    let Op::Verify { nonce, terminal, .. } = &mut session.op else {
        return false;
    };
    if terminal.is_some() {
        return false;
    }

    let minted = match &outcome {
        VerifyOutcome::Matched(evidence) => Some(ResultToken::mint(evidence, nonce, token_bytes)),
        VerifyOutcome::NoMatch | VerifyOutcome::Failed(_) => None,
    };

    *terminal = Some(outcome);
    session.token = minted;
    session.unseen = true;
    true
}

pub(crate) fn verify_is_live(session: &Session, epoch: u64) -> bool {
    worker_is_current(session, epoch) && matches!(&session.op, Op::Verify { terminal: None, .. })
}

pub(crate) fn capture_is_live(session: &Session, epoch: u64) -> bool {
    enrol_is_live(session, epoch) || verify_is_live(session, epoch)
}

pub(crate) fn queued_epoch(session: &Session) -> Option<u64> {
    session.worker_epoch
}

fn worker_is_current(session: &Session, epoch: u64) -> bool {
    session.open && session.generation == epoch && session.worker_epoch == Some(epoch)
}

pub(crate) fn worker_stopped(session: &mut Session, epoch: u64) {
    if session.worker_epoch == Some(epoch) {
        session.worker_epoch = None;
    }
}

/// Whether an open (`opening`) or a capture start by the current opener would
/// be refused only because the last enrol or verify work item has not stopped
/// yet, although its operation has ended or its opener has gone.
pub(crate) fn waiting_on_worker(session: &Session, opening: bool) -> bool {
    session.worker_epoch.is_some() && session.op.may_start() && session.open != opening
}

fn verify_poll(ctx: &mut Context<'_>, user: UserPtr) -> Result<Handled> {
    let mut out = VerifyPoll::default();
    // Spend the token only after the copy to user succeeds; a fault must not lose a match.
    let mut consume_token = false;

    match &ctx.session.op {
        Op::Idle => out.state = State::Idle,
        Op::Enrol { .. } => return Err(EBUSY),
        Op::Verify { nonce, terminal, guidance } => {
            out.guidance = *guidance;
            match terminal {
                None => {
                    out.state = if ctx.session.unseen {
                        State::Progress
                    } else {
                        State::Pending
                    }
                }
                Some(VerifyOutcome::Failed(status)) => {
                    out.state = State::Failed;
                    out.result = MatchResult::NotCompared;
                    out.status = *status;
                }
                Some(VerifyOutcome::NoMatch) => {
                    out.state = State::Done;
                    out.result = MatchResult::NoMatch;
                }
                Some(VerifyOutcome::Matched(evidence)) => {
                    out.state = State::Done;
                    match ctx.session.token.as_ref() {
                        Some(token) if token.usable(nonce, &evidence.identity) => {
                            out.result = MatchResult::Match;
                            out.uuid = evidence.identity;
                            out.token = token.bytes;
                            out.deadline_ns = token.deadline_ns;
                            consume_token = true;
                        }
                        _ => {
                            out.result = MatchResult::NotCompared;
                        }
                    }
                }
            }
        }
    }

    UserSlice::new(user, core::mem::size_of::<VerifyPoll>())
        .writer()
        .write(&out)?;
    if consume_token {
        ctx.session.token = None;
    }
    ctx.session.unseen = false;
    ok()
}

fn cancel(ctx: &mut Context<'_>) -> Result<Handled> {
    if ctx.session.op.is_terminal() {
        return Err(ENOENT);
    }

    match &mut ctx.session.op {
        Op::Idle => return Err(ENOENT),
        Op::Enrol { terminal, .. } => *terminal = Some(EnrolOutcome::Failed(STATUS_LOCAL)),
        Op::Verify { terminal, .. } => *terminal = Some(VerifyOutcome::Failed(STATUS_LOCAL)),
    }

    ctx.session.generation = ctx.session.generation.saturating_add(1);
    ctx.session.token = None;
    ctx.session.unseen = true;

    Ok(Handled {
        ret: 0,
        wake: true,
        start_enrol: false,
        start_verify: false,
        delete_identity: None,
        delete_identities: KVec::new(),
    })
}

/// Ends a live enrolment or verify the way a CANCEL does, for the system going
/// to sleep: userspace sees a failure it can retry after resume. Returns
/// whether a waiter needs waking.
pub(crate) fn abort_capture(session: &mut Session) -> bool {
    match &mut session.op {
        Op::Enrol { terminal, .. } if terminal.is_none() => {
            *terminal = Some(EnrolOutcome::Failed(STATUS_LOCAL))
        }
        Op::Verify { terminal, .. } if terminal.is_none() => {
            *terminal = Some(VerifyOutcome::Failed(STATUS_LOCAL))
        }
        _ => return false,
    }
    session.token = None;
    session.unseen = true;
    true
}

/// The ioctls that start a sensor capture.
pub(crate) fn starts_capture(cmd: u32) -> bool {
    cmd == IOC_ENROL_START || cmd == IOC_VERIFY_START
}

fn delete(ctx: &mut Context<'_>, user: UserPtr) -> Result<Handled> {
    require_admin()?;
    if !ctx.session.op.may_start() {
        return Err(EBUSY);
    }
    let request: Delete = UserSlice::new(user, core::mem::size_of::<Delete>())
        .reader()
        .read()?;

    if !ctx.index.contains(&request.uuid) {
        pr_info!(
            "sep_bio: DELETE of an identity not held; reporting success (nothing to delete)\n"
        );
        return ok();
    }

    // `0x57` takes one `identity_v1_t` (signed user id + 16-byte UUID).
    let Some(identity) = ctx.index.identity_v1_for(&request.uuid, ENROL_USER_ID) else {
        pr_warn!("sep_bio: DELETE of a held identity could not be expressed as an identity_v1_t\n");
        return Err(ENOENT);
    };

    Ok(Handled {
        ret: 0,
        wake: false,
        start_enrol: false,
        start_verify: false,
        delete_identity: Some(identity),
        delete_identities: KVec::new(),
    })
}

fn delete_all(ctx: &mut Context<'_>) -> Result<Handled> {
    require_admin()?;
    if !ctx.session.op.may_start() {
        return Err(EBUSY);
    }

    if ctx.index.total() == 0 {
        pr_info!("sep_bio: DELETE_ALL on empty index; reporting success (nothing to delete)\n");
        return ok();
    }

    let mut ids = KVec::new();
    for entry in ctx.index.entries.iter() {
        let Some(identity) = ctx.index.identity_v1_for(&entry.uuid, ENROL_USER_ID) else {
            pr_warn!(
                "sep_bio: DELETE_ALL of a held identity could not be expressed as an identity_v1_t\n"
            );
            return Err(ENOENT);
        };
        ids.push(identity, GFP_KERNEL)?;
    }

    Ok(Handled {
        ret: 0,
        wake: false,
        start_enrol: false,
        start_verify: false,
        delete_identity: None,
        delete_identities: ids,
    })
}

pub(crate) fn open(session: &mut Session) -> Result<()> {
    if session.open || session.worker_epoch.is_some() {
        return Err(EBUSY);
    }
    session.reset();
    session.open = true;
    Ok(())
}

pub(crate) fn release(session: &mut Session) {
    session.reset();
}
