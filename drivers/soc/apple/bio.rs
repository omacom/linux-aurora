// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

use crate::store::{Key, Store};
use kernel::ioctl::{_IO, _IOR, _IOW, _IOWR};
use kernel::prelude::*;
use kernel::uaccess::{UserPtr, UserSlice};

pub(crate) const IFACE_VERSION: u32 = 5;

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

/// The Touch ID lockout's own node. fprintd holds `/dev/sep-bio` open, so
/// a password authentication cannot reach the lockout through that node.
pub(crate) const REARM_DEVICE_NAME: &CStr = c"sep-bio-rearm";
pub(crate) const REARM_DEVICE_MODE: u16 = 0o600;

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
    // The last touch's finger is still down; the next capture waits for it.
    Lift = 4,
    // A cause reported with a retry (an unusable capture): partial contact,
    // rest more of the finger on the middle of the sensor.
    Center = 5,
    // The finger moved during the capture: hold it still.
    TooFast = 6,
    // Dirt on the sensor or a wet finger: clean or dry it.
    Clean = 7,
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
/// `SEP_BIO_STATUS_RETRY` in a running enrolment's poll: the latest capture
/// did not count; `guidance` says why.
pub(crate) const ENROL_POLL_RETRY: u32 = 5;

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
    account_uid: u32,
    finger: u32,
    reserved: u32,
    label: [u8; LABEL_LEN],
}

#[repr(C)]
pub(crate) struct EnclaveList {
    pub(crate) count: u32,
    pub(crate) reserved: u32,
    pub(crate) uuid: [[u8; UUID_LEN]; MAX_IDENTITIES],
}

impl EnclaveList {
    pub(crate) fn new() -> Self {
        Self { count: 0, reserved: 0, uuid: [[0; UUID_LEN]; MAX_IDENTITIES] }
    }

    pub(crate) fn push(&mut self, uuid: [u8; UUID_LEN]) -> Result<()> {
        let at = self.count as usize;
        if at >= MAX_IDENTITIES {
            return Err(EOVERFLOW);
        }
        self.uuid[at] = uuid;
        self.count += 1;
        Ok(())
    }
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
pub(crate) struct Delete {
    pub(crate) uuid: [u8; UUID_LEN],
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

/// The Touch ID lockout, as `/dev/sep-bio-rearm` reports it.
#[repr(C)]
#[derive(Default)]
pub(crate) struct Lockout {
    /// 1 while every verify is refused until a password authentication.
    pub(crate) locked: u32,
    /// Failed matches since the last match or re-arm.
    pub(crate) failures: u32,
    pub(crate) reserved: u32,
    /// `LOCKOUT_FLAG_*`: what this driver's re-arm does.
    pub(crate) flags: u32,
}

/// The re-arm never waits for a capture while Touch ID is not locked, and
/// it clears pending failed attempts: call it after every verified password.
pub(crate) const LOCKOUT_FLAG_EVERY_PASSWORD: u32 = 1;
/// Touch ID waits for the first password since this boot.
pub(crate) const LOCKOUT_FLAG_AFTER_BOOT: u32 = 2;
/// The lockout follows too many failed matches.
pub(crate) const LOCKOUT_FLAG_FAILED_ATTEMPTS: u32 = 4;

impl Lockout {
    /// `locked` is a lockout (or one recorded before the restart);
    /// `after_boot` is the wait for the first password since boot. Either
    /// one refuses every verify. `failed_attempts` says that the lockout
    /// follows too many failed matches; without it, a lockout needs the
    /// password for another reason, such as a lapsed match credential.
    pub(crate) fn new(
        locked: bool,
        after_boot: bool,
        failed_attempts: bool,
        failures: u32,
    ) -> Lockout {
        let mut flags = LOCKOUT_FLAG_EVERY_PASSWORD;
        if after_boot {
            flags |= LOCKOUT_FLAG_AFTER_BOOT;
        }
        if locked && failed_attempts {
            flags |= LOCKOUT_FLAG_FAILED_ATTEMPTS;
        }
        Lockout {
            locked: u32::from(locked || after_boot),
            failures,
            reserved: 0,
            flags,
        }
    }
}

pub(crate) const REARM_SECRET_MAX: usize = 1024;

/// A password which userspace has already verified for the requesting user.
#[repr(C)]
pub(crate) struct Rearm {
    pub(crate) secret_len: u32,
    pub(crate) reserved: u32,
    pub(crate) secret: [u8; REARM_SECRET_MAX],
}

impl Drop for Rearm {
    fn drop(&mut self) {
        crate::image::wipe(&mut self.secret);
    }
}

/// Copies in and checks a `REARM` request: the password it carries, in a
/// buffer that wipes itself, and nothing left behind on the stack or in the
/// request, whatever is refused.
pub(crate) fn read_rearm(arg: usize) -> Result<crate::Secret> {
    let request = crate::key::read_user_boxed::<Rearm>(UserPtr::from_addr(arg))?;
    let len = request.secret_len as usize;
    if request.reserved != 0 || len == 0 || len > request.secret.len() {
        return Err(EINVAL);
    }
    let mut bytes = KVec::with_capacity(len, GFP_KERNEL)?;
    bytes.extend_from_slice(&request.secret[..len], GFP_KERNEL)?;
    Ok(crate::Secret(bytes))
}

// Sizes are baked into the ioctl numbers.
static_assert!(core::mem::size_of::<Identity>() == 144);
static_assert!(core::mem::size_of::<Info>() == 32);
static_assert!(core::mem::size_of::<List>() == 4616);
static_assert!(core::mem::size_of::<EnrolStart>() == 144);
static_assert!(core::mem::size_of::<EnclaveList>() == 520);
static_assert!(core::mem::size_of::<EnrolPoll>() == 40);
static_assert!(core::mem::size_of::<VerifyStart>() == 40);
static_assert!(core::mem::size_of::<VerifyPoll>() == 72);
static_assert!(core::mem::size_of::<Delete>() == 16);
static_assert!(core::mem::size_of::<Attest>() == 176);
static_assert!(core::mem::size_of::<Lockout>() == 16);
static_assert!(core::mem::size_of::<Rearm>() == 1032);

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
    to_user: Info, List, EnclaveList, EnrolPoll, VerifyPoll, Attest, Lockout;
    from_user: EnrolStart, VerifyStart, Delete, Attest, Rearm
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
pub(crate) const IOC_LIST_ENCLAVE: u32 = _IOR::<EnclaveList>(MAGIC, 0x0b);
pub(crate) const IOC_DELETE_ORPHAN: u32 = _IOW::<Delete>(MAGIC, 0x0c);
pub(crate) const IOC_PREPARE_ENROL: u32 = _IOW::<EnrolStart>(MAGIC, 0x0d);
// On /dev/sep-bio-rearm only.
pub(crate) const IOC_LOCKOUT_STATE: u32 = _IOR::<Lockout>(MAGIC, 0x0e);
pub(crate) const IOC_REARM: u32 = _IOW::<Rearm>(MAGIC, 0x0f);
pub(crate) const IOC_REQUIRE_PASSWORD: u32 = _IO(MAGIC, 0x10);

#[derive(Clone)]
pub(crate) struct MatchEvidence {
    identity: [u8; UUID_LEN],
}

impl MatchEvidence {
    pub(crate) fn from_enclave_reply(identity: [u8; UUID_LEN]) -> MatchEvidence {
        MatchEvidence { identity }
    }
}

#[derive(Clone)]
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

    pub(crate) fn can_persist_next(&self, store: &Store) -> bool {
        store.can_write(&identity_key(), 8 + (self.entries.len() + 1) * (UUID_LEN + LABEL_LEN))
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
    // An enrolment capture was not usable; the next poll reports it once.
    enrol_retry_unseen: bool,
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
            enrol_retry_unseen: false,
            token: None,
        }
    }

    fn reset(&mut self) {
        self.generation = self.generation.saturating_add(1);
        self.open = false;
        self.op = Op::Idle;
        self.unseen = false;
        self.enrol_stage_unseen = false;
        self.enrol_retry_unseen = false;
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

pub(crate) fn ok() -> Result<Handled> {
    Ok(Handled {
        ret: 0,
        wake: false,
        start_enrol: false,
        start_verify: false,
        delete_identity: None,
        delete_identities: KVec::new(),
    })
}

pub(crate) fn require_admin() -> Result<()> {
    if crate::shim::capable_admin() {
        Ok(())
    } else {
        Err(EPERM)
    }
}

pub(crate) fn orphan_delete(identity: crate::sbio::IdentityV1) -> Result<Handled> {
    let mut handled = ok()?;
    handled.delete_identity = Some(identity);
    Ok(handled)
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

fn validate_enrol_request(request: &EnrolStart) -> Result<()> {
    // NSS account resolution occurs in libfprint. This ABI requires its
    // resolved uid, the named finger and the generated user label.
    if request.flags != 0 || request.reserved != 0
        || request.account_uid == u32::MAX || !(1..=10).contains(&request.finger)
        || !request.label.starts_with(b"FP1-")
        || request.label[12] != b'-' || request.label[14] != b'-'
        || request.label[23] != b'-' || request.label[24] == 0
        || request.label[13] != b"123456789A"[request.finger as usize - 1]
        || !request.label.contains(&0)
    {
        return Err(EINVAL);
    }
    Ok(())
}

pub(crate) fn validate_enrol_start_arg(arg: usize) -> Result<()> {
    let user = UserPtr::from_addr(arg);
    let request: EnrolStart = UserSlice::new(user, core::mem::size_of::<EnrolStart>())
        .reader().read()?;
    validate_enrol_request(&request)
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[::kernel::macros::kunit_tests(apple_sep_enrol_request)]
mod enrol_request_tests {
    use super::*;

    fn valid() -> EnrolStart {
        let mut request = EnrolStart {
            flags: 0, account_uid: 1000, finger: 7, reserved: 0,
            label: [0; LABEL_LEN],
        };
        request.label[..28].copy_from_slice(b"FP1-20261027-7-12345678-user");
        request
    }

    fn take_enrol_poll(session: &mut Session) -> Result<EnrolPoll> {
        let out = enrol_poll_report(session)?;
        mark_enrol_seen(session);
        Ok(out)
    }

    fn enrolling(session: &mut Session) -> u64 {
        session.open = true;
        session.start_worker().unwrap();
        session.op = Op::Enrol {
            label: [0; LABEL_LEN],
            stage: 0,
            terminal: None,
            guidance: Guidance::Place,
            percent: 0,
        };
        session.worker_epoch.unwrap()
    }

    // An enrolment capture that did not count is reported once, with its
    // cause, and the enrolment goes on.
    #[test]
    fn uncounted_enrol_capture_is_reported_once() {
        let mut session = Session::new();
        let epoch = enrolling(&mut session);
        assert!(enrol_retry(&mut session, epoch, Guidance::Center));
        let first = take_enrol_poll(&mut session).unwrap();
        assert_eq!(first.status, ENROL_POLL_RETRY);
        assert!(first.guidance == Guidance::Center);
        assert!(matches!(first.state, State::Pending));
        let second = take_enrol_poll(&mut session).unwrap();
        assert_eq!(second.status, 0);
        // A stage after a retry: the next poll carries the stage, no retry.
        assert!(enrol_advance(&mut session, epoch, 3, 40, Guidance::LiftAndMove));
        let third = take_enrol_poll(&mut session).unwrap();
        assert!(matches!(third.state, State::Progress));
        assert_eq!((third.stage, third.status), (3, 0));
        // A stale worker cannot report into a newer enrolment.
        assert!(!enrol_retry(&mut session, epoch + 1, Guidance::Center));
    }

    #[test]
    fn resolved_account_and_known_finger() {
        assert!(validate_enrol_request(&valid()).is_ok());
    }

    #[test]
    fn missing_uid_is_refused() {
        let mut request = valid();
        request.account_uid = u32::MAX;
        assert!(validate_enrol_request(&request).is_err());
    }

    #[test]
    fn unknown_or_mismatched_finger_is_refused() {
        let mut request = valid();
        request.finger = 0;
        assert!(validate_enrol_request(&request).is_err());
        request.finger = 6;
        assert!(validate_enrol_request(&request).is_err());
    }

    #[test]
    fn missing_username_is_refused() {
        let mut request = valid();
        request.label[24] = 0;
        assert!(validate_enrol_request(&request).is_err());
    }
}

fn enrol_start(ctx: &mut Context<'_>, user: UserPtr) -> Result<Handled> {
    require_admin()?;
    let request: EnrolStart = UserSlice::new(user, core::mem::size_of::<EnrolStart>())
        .reader()
        .read()?;
    validate_enrol_request(&request)?;
    require_sensor(ctx.sensor_present)?;

    if ctx.session.worker_epoch.is_some() || !ctx.session.op.may_start() {
        return Err(EBUSY);
    }
    let count = ctx.live_identity_count.ok_or(EIO)?;
    if count >= SEP_USER_CAPACITY || ctx.index.total() >= MAX_IDENTITIES {
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
    ctx.session.enrol_retry_unseen = false;
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

/// An enrolment capture was not usable: the enrolment goes on, and the next
/// poll reports `ENROL_POLL_RETRY` once with the cause in `guidance`, so
/// userspace can say why the touch did not count.
pub(crate) fn enrol_retry(session: &mut Session, epoch: u64, cause: Guidance) -> bool {
    if !worker_is_current(session, epoch) {
        return false;
    }
    if let Op::Enrol { terminal, guidance, .. } = &mut session.op {
        if terminal.is_none() {
            *guidance = cause;
            session.unseen = true;
            session.enrol_retry_unseen = true;
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

/// What an enrolment poll reports now.
fn enrol_poll_report(session: &Session) -> Result<EnrolPoll> {
    let mut out = EnrolPoll {
        stages_total: ENROL_STAGES,
        ..EnrolPoll::default()
    };

    match &session.op {
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
                    out.state = if session.enrol_stage_unseen {
                        State::Progress
                    } else {
                        State::Pending
                    };
                    if session.enrol_retry_unseen {
                        out.status = ENROL_POLL_RETRY;
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
    Ok(out)
}

/// The poll was delivered: its one-shot reports are seen.
fn mark_enrol_seen(session: &mut Session) {
    session.unseen = false;
    session.enrol_stage_unseen = false;
    session.enrol_retry_unseen = false;
}

fn enrol_poll(ctx: &mut Context<'_>, user: UserPtr) -> Result<Handled> {
    let out = enrol_poll_report(ctx.session)?;
    UserSlice::new(user, core::mem::size_of::<EnrolPoll>())
        .writer()
        .write(&out)?;
    mark_enrol_seen(ctx.session);
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

pub(crate) fn idle_for_mutation(session: &Session) -> bool {
    session.worker_epoch.is_none() && session.op.may_start()
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
        pr_info_ratelimited!(
            "sep_bio: DELETE of an identity not held; reporting success (nothing to delete)\n"
        );
        return ok();
    }

    // `0x57` takes one `identity_v1_t` (signed user id + 16-byte UUID).
    let Some(identity) = ctx.index.identity_v1_for(&request.uuid, ENROL_USER_ID) else {
        pr_warn_ratelimited!("sep_bio: DELETE of a held identity could not be expressed as an identity_v1_t\n");
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
        pr_info_ratelimited!("sep_bio: DELETE_ALL on empty index; reporting success (nothing to delete)\n");
        return ok();
    }

    let mut ids = KVec::new();
    for entry in ctx.index.entries.iter() {
        let Some(identity) = ctx.index.identity_v1_for(&entry.uuid, ENROL_USER_ID) else {
            pr_warn_ratelimited!(
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

/// KUnit hooks for `sep_ioctl_test.c`: a `/dev/sep-bio` session driven
/// through [`ioctl`] from real user memory, with no sensor or enclave.
#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
pub(crate) struct KunitBio {
    session: Session,
    index: IdentityIndex,
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[no_mangle]
pub extern "C" fn sep_kunit_bio_new() -> *mut c_void {
    match KBox::new(KunitBio { session: Session::new(), index: IdentityIndex::new() }, GFP_KERNEL) {
        Ok(b) => KBox::into_raw(b).cast(),
        Err(_) => core::ptr::null_mut(),
    }
}

/// # Safety
/// `bio` comes from [`sep_kunit_bio_new`] and is not yet freed.
#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[no_mangle]
pub unsafe extern "C" fn sep_kunit_bio_ioctl(bio: *mut c_void, cmd: c_uint, arg: c_ulong) -> c_long {
    // SAFETY: per the contract above.
    let bio = unsafe { &mut *bio.cast::<KunitBio>() };
    let mut ctx = Context {
        session: &mut bio.session,
        index: &bio.index,
        sensor_present: true,
        live_identity_count: Some(0),
    };
    match ioctl(&mut ctx, cmd, arg as usize) {
        Ok(h) => h.ret as c_long,
        Err(e) => e.to_errno() as c_long,
    }
}

/// The capture worker a started enrolment or verify would run has stopped.
///
/// # Safety
/// As for [`sep_kunit_bio_ioctl`].
#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[no_mangle]
pub unsafe extern "C" fn sep_kunit_bio_worker_stopped(bio: *mut c_void) {
    // SAFETY: per the contract above.
    let bio = unsafe { &mut *bio.cast::<KunitBio>() };
    if let Some(epoch) = bio.session.worker_epoch {
        worker_stopped(&mut bio.session, epoch);
    }
}

/// # Safety
/// As for [`sep_kunit_bio_ioctl`]; `bio` is not used again.
#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[no_mangle]
pub unsafe extern "C" fn sep_kunit_bio_free(bio: *mut c_void) {
    if !bio.is_null() {
        // SAFETY: per the contract above, `bio` came from `KBox::into_raw`.
        drop(unsafe { KBox::from_raw(bio.cast::<KunitBio>()) });
    }
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[no_mangle]
pub extern "C" fn sep_kunit_rearm_read(arg: c_ulong) -> c_int {
    match read_rearm(arg as usize) {
        Ok(_) => 0,
        Err(e) => e.to_errno(),
    }
}
