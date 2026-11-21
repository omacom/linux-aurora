// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Ryan Murray

//! Keeps the enclave's xART writes to the records this Linux install created.
//!
//! The gigalocker is shared by every OS installed on the Mac. Each OS keeps
//! its anti-replay state in session records keyed by a random session id.
//! Nothing in a record says which OS wrote it, so the driver cannot tell
//! another OS's record from its own by looking at it.
//!
//! With `xart_guard` set, the driver keeps a list of the session records the
//! enclave created while Linux was running, in a host file. It then refuses
//! to rewrite or delete any session record that was in the gigalocker when the
//! driver opened it and is not on that list. With `xart_guard=2` it also
//! refuses writes to the two shared root records. A refused write fails the
//! enclave's request, the same way a read-only gigalocker does: Touch ID may
//! then be unavailable, but no other OS's record changes.
//!
//! The parameter's default, -1, takes the platform profile's level: 1 on the
//! M2 and M3 Macs, whose enclave Linux shares with a macOS install, and 0 on
//! the M1 Macs, where setting up Touch ID has to rewrite a record from before
//! Linux, and on the MacBook Neo, which already holds Linux records from
//! before the list existed and would otherwise lose write access to them.
//!
//! The effective level, whether the list could be loaded and every refused
//! write are reported in the driver's `diag` attributes, with what they make
//! of a fingerprint match ([`Matching`]), so that a readiness check can tell
//! a guard that blocks matching from one that has refused nothing yet.
//!
//! The list entry is made durable before the enclave's write reaches the
//! gigalocker, and the list's own directory entry is flushed before anything
//! is adopted; the key is listed in memory only once its entry is written and
//! flushed. A crash between the two durable points leaves an entry for a
//! record that was never written, which is harmless, and a failed adoption is
//! retried in the same slot rather than skipped. It can never leave a Linux
//! record that the next boot treats as another OS's.
//!
//! A boot policy change (a new Stage 1, a different boot object) makes the
//! enclave rewrite one existing session record the first time Linux's
//! identity keybag is used under the new policy: its first unlock, or
//! Touch ID's `DESIGNATE_KEYBAG` when that comes first. Linux did not create
//! that record. On a boot under an unchanged policy neither rewrites an
//! existing session record, so a rewrite inside them is the sign of a policy
//! change. The guard lets exactly one kind-4 rewrite through in that window
//! ([`policy_window`]): the first unlock or designate of an identity keybag
//! that already existed, at most once per boot, never a delete, and only
//! once the record's previous value is saved in a host file
//! ([`PREVIOUS_PATH`]). The record is not added to Linux's list.
//!
//! For a policy change the window did not cover, `aurora-sep-policy-change`
//! backs up the gigalocker and leaves a one-time allowance in a host file
//! ([`ALLOW_PATH`]): one record, named or the first of its kind, that the
//! enclave may rewrite during the next boot only, on the same terms. The
//! driver empties the file before it uses the allowance.
//!
//! Creating Linux's identity keybag also rewrites one existing kind-4
//! record, so the create runs in the same window ([`keybag_create_window`]).
//! An unfinished create is sent again only if Linux's list shows it left no
//! record behind ([`may_retry_create`]).
//!
//! A Mac that cold-boots its enclave (the M1 family) cannot stay inside those
//! windows. Its first Touch ID setup rewrites up to three SEP-xART lockers
//! (kind 4) from before Linux that the enclave reads at every attach: on a
//! MacBook Air M1 (J313), two at the `COPY_KEYBAG` after the create and one
//! during the first activation, after its `LOAD_KEYBAG`. The guard refuses
//! them, which is why the M1 profiles default to no guard. With
//! `xart_guard=1 xart_guard_pins=1` (run on a J313 only) the owner accepts
//! those lockers instead ([`Setup::Pin`]). The install's first setup then gets
//! a window of its own, recorded in the list as used before it opens, from
//! before the create until the first activation returns, and shut by any
//! failed step before that. Each existing locker the enclave rewrites in it,
//! provided the enclave read it since the store opened, is pinned in the list,
//! up to [`MAX_PINS`], once its previous value is saved ([`PREVIOUS_PATH`]).
//! On a later boot with the parameter set, the enclave may rewrite a pinned
//! locker once it has read it. A pinned locker is never deleted and never
//! listed as Linux's. While the window is open, a read locker is pinned even
//! where a per-boot window would have let it through once; every other
//! rewrite the per-boot windows or the one-time allowance cover still goes
//! through, unless recording a pin fails. Without the parameter a cold-boot
//! Mac's first setup gets no window of its own and its pins are not
//! honoured.

use crate::shim::StoreFile;
use crate::xart_store::{crc32_ieee, Key};
use kernel::prelude::*;
use kernel::sync::atomic::{Atomic, Relaxed};

/// Host file listing the session records Linux created.
pub(crate) const OWNED_PATH: &CStr = c"/var/lib/aurora-sep-xart-owned.bin";
/// Host file holding the one-time allowance for a boot policy change.
pub(crate) const ALLOW_PATH: &CStr = c"/var/lib/aurora-sep-xart-allow-once.bin";
/// Host file keeping the previous value of every existing record a window
/// or allowance let the enclave rewrite.
pub(crate) const PREVIOUS_PATH: &CStr = c"/var/lib/aurora-sep-xart-policy-previous.bin";
const PREVIOUS_MAGIC: &[u8; 4] = b"AXGP";
/// The kind of session record a window or allowance may rewrite.
const POLICY_KIND: u8 = 4;

/// Why the guard let the enclave rewrite an existing record Linux did not
/// create.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Rewrite {
    /// The first unlock of an existing keybag, or a one-time allowance.
    PolicyChange = 1,
    /// The creation of Linux's identity keybag.
    KeybagCreate = 2,
    /// A cold-boot Mac's first Touch ID setup under `xart_guard_pins`, which
    /// pins the record.
    FirstSetupPin = 3,
}

impl Rewrite {
    fn from_raw(raw: u32) -> Option<Rewrite> {
        match raw {
            1 => Some(Rewrite::PolicyChange),
            2 => Some(Rewrite::KeybagCreate),
            _ => None,
        }
    }

    /// The reason as the kernel log gives it.
    pub(crate) fn describe(self) -> &'static str {
        match self {
            Rewrite::PolicyChange => "for a boot policy change",
            Rewrite::KeybagCreate => "while Linux's identity keybag was created",
            Rewrite::FirstSetupPin => "and pinned it, for this install's first Touch ID setup",
        }
    }
}

/// Whether this boot's one window has started, and the [`Rewrite`] it is
/// open for now (0 when closed).
static WINDOW_STARTED: Atomic<bool> = Atomic::new(false);
static WINDOW_OPEN: Atomic<u32> = Atomic::new(0);

/// The window open now.
fn open_window() -> Option<Rewrite> {
    Rewrite::from_raw(WINDOW_OPEN.load(Relaxed))
}

/// Runs `f` with the window open for `why`, unless this boot had one already.
fn with_window<T>(why: Rewrite, f: impl FnOnce() -> T) -> T {
    if WINDOW_STARTED.xchg(true, Relaxed) {
        return f();
    }
    WINDOW_OPEN.store(why as u32, Relaxed);
    let result = f();
    WINDOW_OPEN.store(0, Relaxed);
    result
}

/// Runs `unlock`, the unlock or Touch ID designate of an identity keybag
/// that already existed, with the boot policy window open if it is this
/// boot's first. Later ones run with it closed.
pub(crate) fn policy_window<T>(unlock: impl FnOnce() -> T) -> T {
    with_window(Rewrite::PolicyChange, unlock)
}

/// Runs `create`, the identity keybag's CREATE_KEYBAG exchange, with the
/// window open: the enclave rewrites one existing kind-4 record to create it.
pub(crate) fn keybag_create_window<T>(create: impl FnOnce() -> T) -> T {
    with_window(Rewrite::KeybagCreate, create)
}

/// Linux's list as an unfinished create's intent recorded it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Unfinished {
    /// Written by a driver that did not record it.
    Unmarked,
    /// No guard ran with a list.
    Unguarded,
    /// The list had this many entries.
    Guarded(u64),
}

/// Whether an unfinished create may be sent again, given the entries in
/// Linux's list at load (`None` without a list). Every new session record is
/// listed before it is written, so an unchanged list means the create left
/// nothing; an unmarked intent needs an empty list.
pub(crate) fn may_retry_create(unfinished: Unfinished, listed: Option<u64>) -> bool {
    match (unfinished, listed) {
        (_, None) => false,
        (Unfinished::Guarded(then), Some(now)) => now == then,
        (Unfinished::Unmarked, Some(now)) => now == 0,
        (Unfinished::Unguarded, Some(_)) => false,
    }
}

/// Keeps the window shut for the rest of the boot: a keybag or ref-key
/// created this boot is not unlocked under an older policy.
pub(crate) fn close_policy_window() {
    WINDOW_STARTED.store(true, Relaxed);
}

/// Saves `value`, the current value of `key`, before the enclave rewrites
/// it. The rewrite goes ahead only once this is durable.
pub(crate) fn save_previous(key: &Key, value: &[u8]) -> Result<()> {
    let (file, _created) = StoreFile::open_host(PREVIOUS_PATH)?;
    file.sync_parent()?;
    let end = file.size()?;
    let mut head = [0u8; 32];
    head[..4].copy_from_slice(PREVIOUS_MAGIC);
    head[4] = key.kind;
    head[8..24].copy_from_slice(&key.uuid);
    let len = u32::try_from(value.len()).map_err(|_| EINVAL)?;
    head[24..28].copy_from_slice(&len.to_le_bytes());
    head[28..32].copy_from_slice(&crc32_ieee(value).to_le_bytes());
    file.write_all(end, &head)?;
    file.write_all(end + head.len() as u64, value)?;
    file.sync()
}

const ENTRY_SIZE: usize = 32;
const ENTRY_MAGIC: &[u8; 4] = b"AXGO";
const ALLOW_MAGIC: &[u8; 4] = b"AXGA";
/// Bounds the file the driver reads at probe, counted in entries the file
/// holds, torn and repeated ones included. The gigalocker has at most a few
/// hundred slots, so a longer list is damaged. Appending stops at the same
/// bound, so the driver never writes a list it would refuse to read.
const MAX_ENTRIES: usize = 4096;
/// Refusals are logged individually up to this count, then only counted.
const LOGGED_REFUSALS: usize = 8;
/// Marks a list entry as a pinned locker's. It sits in the first of the three
/// bytes an owned entry keeps zero, so a kernel without pinning reads the
/// entry as unreadable and lists nothing by it.
const ENTRY_PINNED: u8 = 1;
/// SEP-xART lockers the enclave read since the store opened that a pinning
/// guard remembers. The gigalocker has at most a few hundred slots; a locker
/// past the bound counts as unread.
const MAX_READ: usize = 512;
/// Pre-Linux lockers one install may pin. On a MacBook Air M1 (J313, 13.5
/// sepOS) the first setup rewrites three, all read since probe: two at
/// COPY_KEYBAG, one during the first activation, after its LOAD_KEYBAG.
const MAX_PINS: usize = 3;

/// What a platform's first Touch ID setup may do to SEP-xART lockers from
/// before Linux, beyond the per-boot windows.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Setup {
    /// Nothing more: a warm-attach Mac, or a cold-boot Mac without
    /// `xart_guard_pins`.
    Windows,
    /// A cold-boot Mac with `xart_guard_pins=1`: the install's first setup pins
    /// each SEP-xART locker it rewrites, up to [`MAX_PINS`], and the enclave
    /// may rewrite those again.
    Pin,
}

/// The rule for a platform that cold-boots its enclave (`cold`) or attaches
/// to a running one, with `xart_guard_pins=pins`. The parameter changes
/// nothing on a warm-attach Mac.
pub(crate) fn setup(cold: bool, pins: u8) -> Setup {
    if cold && pins != 0 {
        Setup::Pin
    } else {
        Setup::Windows
    }
}

/// Whether the install's first-setup window under [`Setup::Pin`] is open.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FirstSetup {
    /// Not open this boot, or shut.
    Closed,
    /// Open: the enclave may rewrite and pin a locker it read.
    Open,
    /// Every pin is used; further refusals stand.
    Used,
}

/// What the guard does with one enclave write or delete.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Verdict {
    /// The record is Linux's own, or a shared root at level 1.
    Allow,
    /// A session record that did not exist when the gigalocker was opened:
    /// list it as Linux's before writing it.
    Adopt,
    /// A record that predates this boot and that Linux did not create.
    Refuse,
}

/// Decides one request. `level` is the `xart_guard` parameter, `existed`
/// whether the record was in the gigalocker when the driver opened it, and
/// `owned` whether it is on Linux's list.
pub(crate) fn decide(level: u8, key: &Key, existed: bool, owned: bool) -> Verdict {
    if key.kind <= 2 {
        return if level >= 2 { Verdict::Refuse } else { Verdict::Allow };
    }
    if owned {
        Verdict::Allow
    } else if existed {
        Verdict::Refuse
    } else {
        Verdict::Adopt
    }
}

/// The guard level for `xart_guard=param` on a platform whose profile
/// defaults to `platform`, or `None` for no guard. A negative value takes the
/// platform's level; a level above 2 is treated as 2, the strictest.
pub(crate) fn level(param: i8, platform: u8) -> Option<u8> {
    let level = u8::try_from(param).unwrap_or(platform);
    match level {
        0 => None,
        level => Some(level.min(2)),
    }
}

/// Whether a fingerprint match can save the state it writes to xART, as far
/// as the driver can tell. The values are `SEP_DIAG_MATCHING_*` in `shim.h`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Matching {
    /// xART writes are enabled and no guard runs.
    Usable = 1,
    /// A guard runs and nothing says it refuses a match, but nothing says it
    /// allows one either.
    Unverified = 2,
    /// A match's write is refused, or known to be.
    Blocked = 3,
}

/// What [`Matching`] is derived from.
#[derive(Clone, Copy, Debug)]
pub(crate) struct MatchFacts {
    /// The store opened with writes enabled.
    pub(crate) xart_writable: bool,
    /// The guard's effective level and whether its list loaded, or `None`
    /// when no guard runs.
    pub(crate) guard: Option<(u8, bool)>,
    /// The guard's refusals since probe.
    pub(crate) refusals: u32,
}

impl MatchFacts {
    /// A match rewrites existing session records, so it is blocked when
    /// xART is read-only, after a refusal, or without a list. Otherwise a
    /// guard leaves it unverified: not even an empty list proves the setup
    /// ran unguarded.
    pub(crate) fn matching(&self) -> Matching {
        match self.guard {
            _ if !self.xart_writable || self.refusals != 0 => Matching::Blocked,
            None => Matching::Usable,
            Some((_, false)) => Matching::Blocked,
            Some(_) => Matching::Unverified,
        }
    }
}

/// Encodes one list entry.
pub(crate) fn encode(key: &Key) -> [u8; ENTRY_SIZE] {
    encode_with(ENTRY_MAGIC, key)
}

fn encode_with(magic: &[u8; 4], key: &Key) -> [u8; ENTRY_SIZE] {
    let mut entry = [0u8; ENTRY_SIZE];
    entry[..4].copy_from_slice(magic);
    entry[4] = key.kind;
    entry[8..24].copy_from_slice(&key.uuid);
    let crc = crc32_ieee(&entry[..24]);
    entry[24..28].copy_from_slice(&crc.to_le_bytes());
    entry
}

/// Decodes one list entry; `None` for a torn or foreign one.
pub(crate) fn decode(entry: &[u8]) -> Option<Key> {
    decode_with(ENTRY_MAGIC, entry)
}

fn decode_with(magic: &[u8; 4], entry: &[u8]) -> Option<Key> {
    match parse(magic, entry)? {
        (kind @ 3..=4, [0, 0, 0], uuid) => Some(Key::new(kind, uuid)),
        _ => None,
    }
}

/// The kind, the three bytes after it and the session id of a whole entry
/// with `magic` and a good CRC; `None` for a torn or foreign one.
fn parse(magic: &[u8; 4], entry: &[u8]) -> Option<(u8, [u8; 3], [u8; 16])> {
    if entry.len() != ENTRY_SIZE || &entry[..4] != magic {
        return None;
    }
    let crc = u32::from_le_bytes([entry[24], entry[25], entry[26], entry[27]]);
    if crc != crc32_ieee(&entry[..24]) {
        return None;
    }
    let mut uuid = [0u8; 16];
    uuid.copy_from_slice(&entry[8..24]);
    Some((entry[4], [entry[5], entry[6], entry[7]], uuid))
}

fn with_flag(mut entry: [u8; ENTRY_SIZE], flag: u8) -> [u8; ENTRY_SIZE] {
    entry[5] = flag;
    let crc = crc32_ieee(&entry[..24]);
    entry[24..28].copy_from_slice(&crc.to_le_bytes());
    entry
}

/// The entry that records this install's first-setup window as used. Kind 0
/// is no session record's, so it never lists a record as Linux's, and a
/// kernel without pinning reads it as one unreadable entry.
pub(crate) fn setup_consumed() -> [u8; ENTRY_SIZE] {
    let mut entry = [0u8; ENTRY_SIZE];
    entry[..4].copy_from_slice(ENTRY_MAGIC);
    entry[8..24].copy_from_slice(b"first setup used");
    let crc = crc32_ieee(&entry[..24]);
    entry[24..28].copy_from_slice(&crc.to_le_bytes());
    entry
}

/// Encodes the entry that pins `key`, a SEP-xART locker from before Linux.
/// [`decode`] reads it as no record, so it lists nothing as Linux's.
pub(crate) fn encode_pin(key: &Key) -> [u8; ENTRY_SIZE] {
    with_flag(encode(key), ENTRY_PINNED)
}

/// Decodes a pin entry; `None` for anything else, a pin of a record that is
/// not a SEP-xART locker included.
pub(crate) fn decode_pin(entry: &[u8]) -> Option<Key> {
    match parse(ENTRY_MAGIC, entry)? {
        (POLICY_KIND, [ENTRY_PINNED, 0, 0], uuid) => Some(Key::new(POLICY_KIND, uuid)),
        _ => None,
    }
}

/// A session id, printed the way `aurora-sep-policy-change` takes it.
pub(crate) struct Id<'a>(pub(crate) &'a [u8; 16]);

impl kernel::fmt::Display for Id<'_> {
    fn fmt(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        for byte in self.0 {
            write!(f, "{:02x}", byte)?;
        }
        Ok(())
    }
}

/// The one existing session record the enclave may rewrite once this boot,
/// after a boot policy change.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Allowance {
    /// This record, named by an earlier boot's refusal.
    Record(Key),
    /// The first existing record of this kind the enclave asks to rewrite.
    /// An installer that changes the boot policy cannot know the record's
    /// session id beforehand; the keybag's first use under the new policy
    /// (an unlock or Touch ID's designate) comes before anything else can
    /// rewrite a session record.
    FirstOfKind(u8),
}

impl Allowance {
    /// Decodes the allowance file; `None` for anything but one valid entry.
    /// An all-zero session id stands for [`Allowance::FirstOfKind`].
    pub(crate) fn decode(file: &[u8]) -> Option<Allowance> {
        let key = decode_with(ALLOW_MAGIC, file)?;
        Some(if key.uuid == [0; 16] {
            Allowance::FirstOfKind(key.kind)
        } else {
            Allowance::Record(key)
        })
    }

    fn covers(&self, key: &Key) -> bool {
        match self {
            Allowance::Record(allowed) => allowed == key,
            Allowance::FirstOfKind(kind) => *kind == key.kind,
        }
    }
}

/// Reads and empties the allowance file. The allowance is for this boot
/// only: when the file cannot be emptied, it is not used at all.
fn take_allowance() -> Option<Allowance> {
    let file = StoreFile::open_readonly(ALLOW_PATH).ok()?;
    let size = file.size().ok()?;
    if size == 0 {
        return None;
    }
    let mut entry = [0u8; ENTRY_SIZE];
    let read = if size == ENTRY_SIZE as u64 {
        file.read_exact(0, &mut entry).is_ok()
    } else {
        false
    };
    drop(file);
    if let Err(e) = StoreFile::open_trunc(ALLOW_PATH).and_then(|file| file.sync()) {
        pr_warn_ratelimited!(
            "xART guard: could not empty the one-time allowance ({:?}); not using it\n",
            e
        );
        return None;
    }
    let allowance = if read { Allowance::decode(&entry) } else { None };
    match allowance {
        Some(Allowance::Record(key)) => pr_info_ratelimited!(
            "xART guard: this boot may rewrite kind-{} record {} once, for a boot policy change\n",
            key.kind,
            Id(&key.uuid)
        ),
        Some(Allowance::FirstOfKind(kind)) => pr_info_ratelimited!(
            "xART guard: this boot may rewrite the first existing kind-{} record the enclave asks for, once, for a boot policy change\n",
            kind
        ),
        None => pr_warn_ratelimited!("xART guard: ignored a damaged one-time allowance\n"),
    }
    allowance
}

/// The file that holds Linux's list: the host file, or memory in the tests.
pub(crate) trait ListFile {
    fn size(&self) -> Result<u64>;
    fn read_exact(&self, off: u64, buf: &mut [u8]) -> Result<()>;
    fn write_all(&self, off: u64, buf: &[u8]) -> Result<()>;
    fn sync(&self) -> Result<()>;
    /// Makes the file's directory entry durable.
    fn sync_parent(&self) -> Result<()>;
}

/// Linux's list at [`OWNED_PATH`], opened for each access and closed again.
///
/// The guard lives as long as the xART store, which the driver keeps until it
/// is unbound, and nothing unbinds it before a reboot. Holding the list open
/// for writing that long keeps a writer on the root file system, so the
/// read-only remount of `/` at shutdown fails with EBUSY. Writes never create
/// the file: only [`Guard::open`] does that.
pub(crate) struct HostListFile;

impl ListFile for HostListFile {
    fn size(&self) -> Result<u64> {
        StoreFile::open_readonly(OWNED_PATH)?.size()
    }

    fn read_exact(&self, off: u64, buf: &mut [u8]) -> Result<()> {
        StoreFile::open_readonly(OWNED_PATH)?.read_exact(off, buf)
    }

    fn write_all(&self, off: u64, buf: &[u8]) -> Result<()> {
        StoreFile::open_writable_existing(OWNED_PATH)?.write_all(off, buf)
    }

    // fsync on a new descriptor still flushes the writes made through earlier
    // ones, and reports a writeback error that no descriptor has seen yet.
    fn sync(&self) -> Result<()> {
        StoreFile::open_readonly(OWNED_PATH)?.sync()
    }

    fn sync_parent(&self) -> Result<()> {
        StoreFile::open_readonly(OWNED_PATH)?.sync_parent()
    }
}

pub(crate) struct Guard<F: ListFile = HostListFile> {
    level: u8,
    /// `None` when the list could not be opened: nothing can be adopted, so
    /// every session write that is not already listed is refused.
    file: Option<F>,
    /// Byte offset of the next entry.
    end: u64,
    /// Entries in the list at load, torn ones included.
    loaded_entries: u64,
    owned: KVec<Key>,
    existed: KVec<Key>,
    refused: usize,
    /// The one-time allowance for a boot policy change, until it is used.
    allowance: Option<Allowance>,
    /// This boot's window rewrite is used.
    window_used: bool,
    /// Why the write just allowed rewrites an existing record; its previous
    /// value is saved first.
    save_previous: Option<Rewrite>,
    /// The window open now; the tests set their own.
    window: fn() -> Option<Rewrite>,
    /// What the install's first Touch ID setup may do on this platform.
    setup: Setup,
    first_setup: FirstSetup,
    /// The list holds the first-setup window's record, from this boot or an
    /// earlier one.
    setup_recorded: bool,
    /// The SEP-xART lockers from before Linux that the list pins, up to
    /// [`MAX_PINS`]. They are honoured only under [`Setup::Pin`] at level 1.
    pinned: [Option<Key>; MAX_PINS],
    /// Rewrites of pinned lockers let through since probe.
    pinned_writes: usize,
    /// The locker the write just allowed pins, once its previous value is
    /// saved ([`Guard::commit_pin`]).
    pending_pin: Option<Key>,
    /// The SEP-xART lockers the enclave read since the store opened, kept
    /// only under [`Setup::Pin`].
    read: KVec<Key>,
}

impl Guard {
    /// Opens Linux's list in the host file and records which keys `existing`
    /// holds now, and takes this boot's one-time allowance if there is one.
    /// `setup` is the platform's first-setup rule. Never fails: a list that
    /// cannot be read protects every existing record.
    pub(crate) fn open(level: u8, existing: KVec<Key>, setup: Setup) -> Guard {
        let mut guard = Guard::with_file(
            level,
            existing,
            // Only to create the list if absent; the handle closes here.
            StoreFile::open_host(OWNED_PATH).map(|_| HostListFile),
        )
        .with_setup(setup);
        guard.allowance = take_allowance();
        guard
    }
}

impl<F: ListFile> Guard<F> {
    /// As [`Guard::open`], with the list in `file`.
    pub(crate) fn with_file(level: u8, existing: KVec<Key>, file: Result<F>) -> Guard<F> {
        let mut guard = Guard {
            level,
            file: None,
            end: 0,
            loaded_entries: 0,
            owned: KVec::new(),
            existed: existing,
            refused: 0,
            allowance: None,
            window_used: false,
            save_previous: None,
            window: open_window,
            setup: Setup::Windows,
            first_setup: FirstSetup::Closed,
            setup_recorded: false,
            pinned: [None; MAX_PINS],
            pinned_writes: 0,
            pending_pin: None,
            read: KVec::new(),
        };
        // A list created this boot (or by a crashed one) is only safe to adopt
        // into once its name is durable: fsync() of the file alone does not
        // guarantee that. If the directory cannot be flushed, the file is
        // dropped and nothing is adopted, as without a list.
        let file = file.and_then(|file| file.sync_parent().map(|()| file));
        match file {
            Ok(file) => {
                if let Err(e) = guard.load(&file) {
                    // Nothing a partial read parsed counts: no owned record,
                    // no pin and no used window from an unreadable list.
                    guard.owned = KVec::new();
                    guard.pinned = [None; MAX_PINS];
                    guard.setup_recorded = false;
                    guard.end = 0;
                    guard.loaded_entries = 0;
                    pr_warn_ratelimited!(
                        "xART guard: the list of Linux's records is unreadable ({:?}); every existing session record is protected and none can be adopted\n",
                        e
                    );
                } else {
                    guard.file = Some(file);
                }
            }
            Err(e) => pr_warn_ratelimited!(
                "xART guard: the list of Linux's records could not be opened ({:?}); every existing session record is protected and none can be adopted\n",
                e
            ),
        }
        let sessions = guard.existed.iter().filter(|k| k.kind > 2).count();
        let own = guard
            .existed
            .iter()
            .filter(|k| k.kind > 2 && guard.owned.contains(k))
            .count();
        let roots = if level >= 2 {
            ", and so are the shared roots"
        } else {
            ""
        };
        pr_info_ratelimited!(
            "xART guard level {}: {} session records exist, {} created by Linux; the other {} are protected{}\n",
            level,
            sessions,
            own,
            sessions - own,
            roots
        );
        guard
    }

    fn load(&mut self, file: &F) -> Result<()> {
        let size = file.size()?;
        let whole = (size as usize / ENTRY_SIZE).min(MAX_ENTRIES + 1);
        if whole > MAX_ENTRIES {
            return Err(EFBIG);
        }
        let mut torn = 0usize;
        let mut extra_pins = 0usize;
        let mut entry = [0u8; ENTRY_SIZE];
        for i in 0..whole {
            file.read_exact((i * ENTRY_SIZE) as u64, &mut entry)?;
            match decode(&entry) {
                Some(key) if !self.owned.contains(&key) => self.owned.push(key, GFP_KERNEL)?,
                Some(_) => {}
                None if entry == setup_consumed() => self.setup_recorded = true,
                None => match decode_pin(&entry) {
                    // At most MAX_PINS lockers per install: the first-setup
                    // window pins them, and that window opens once.
                    Some(key) if !self.is_pinned(&key) && !self.pins_full() => self.add_pin(key),
                    Some(_) => extra_pins += 1,
                    None => torn += 1,
                },
            }
        }
        if torn != 0 {
            pr_warn_ratelimited!("xART guard: ignored {} unreadable list entries\n", torn);
        }
        if extra_pins != 0 {
            pr_warn_ratelimited!(
                "xART guard: ignored {} pin entries past the first {} or repeated\n",
                extra_pins,
                MAX_PINS
            );
        }
        // A torn tail entry is overwritten by the next adoption.
        self.end = (whole * ENTRY_SIZE) as u64;
        self.loaded_entries = whole as u64;
        Ok(())
    }

    /// The level this guard applies.
    pub(crate) fn effective_level(&self) -> u8 {
        self.level
    }

    /// Whether Linux's list was loaded. Without it nothing can be adopted.
    pub(crate) fn list_loaded(&self) -> bool {
        self.file.is_some()
    }

    /// Entries in Linux's list now, or `None` without a list.
    pub(crate) fn listed_now(&self) -> Option<u64> {
        self.file.as_ref().map(|_| self.end / ENTRY_SIZE as u64)
    }

    /// Entries in Linux's list at load, or `None` without a list.
    pub(crate) fn listed_at_load(&self) -> Option<u64> {
        self.file.as_ref().map(|_| self.loaded_entries)
    }

    /// The same guard with the first-setup rule `setup`, which it reports.
    fn with_setup(mut self, setup: Setup) -> Guard<F> {
        self.setup = setup;
        self.announce_pin();
        self
    }

    /// Whether the list loaded and holds no entry, torn ones included: what
    /// the first-setup window needs.
    fn list_empty(&self) -> bool {
        self.file
            .as_ref()
            .is_some_and(|file| file.size().is_ok_and(|size| size == 0))
    }

    /// Says what `xart_guard_pins` and the pinned lockers come to on this boot.
    fn announce_pin(&self) {
        let honoured = self.setup == Setup::Pin && self.level == 1;
        match (honoured, self.pinned[0]) {
            (true, Some(_)) => {
                for key in self.pinned.iter().flatten() {
                    pr_info_ratelimited!(
                        "xART guard: xart_guard_pins is set: the enclave may rewrite this install's pinned SEP-xART locker {} once it has read it\n",
                        Id(&key.uuid)
                    );
                }
            }
            (true, None) if self.list_empty() => pr_info_ratelimited!(
                "xART guard: xart_guard_pins is set and no locker is pinned: this install's first Touch ID setup may pin up to three\n"
            ),
            // The window opens only on an empty list, so an earlier guarded
            // attempt on this install shuts it for good.
            (true, None) => pr_warn_ratelimited!(
                "xART guard: xart_guard_pins is set, but no locker is pinned and the list of Linux's records is not empty or did not load: the first-setup window stays shut. Reset Touch ID with the list removed to set up under xart_guard_pins\n"
            ),
            (false, Some(_)) => {
                for key in self.pinned.iter().flatten() {
                    pr_info_ratelimited!(
                        "xART guard: this install pinned SEP-xART locker {}, which counts only with xart_guard=1 xart_guard_pins=1: rewrites of it are refused\n",
                        Id(&key.uuid)
                    );
                }
            }
            (false, None) if self.setup == Setup::Pin => pr_info_ratelimited!(
                "xART guard: xart_guard_pins has no effect at level {}\n",
                self.level
            ),
            (false, None) => {}
        }
    }

    /// Notes that the enclave was handed the record under `key`. Only
    /// SEP-xART lockers are kept, and only under [`Setup::Pin`], whose
    /// allowances ask for it. A locker that cannot be kept counts as unread,
    /// which they refuse.
    pub(crate) fn note_read(&mut self, key: &Key) {
        if self.setup != Setup::Pin
            || key.kind != POLICY_KIND
            || self.read.contains(key)
            || self.read.len() >= MAX_READ
        {
            return;
        }
        let _ = self.read.push(*key, GFP_KERNEL);
    }

    /// Opens the first-setup window, only under [`Setup::Pin`] at level 1 on
    /// a fresh install: the list loaded and holding no entry, torn ones
    /// included. The window is recorded as used in the list, durably, before
    /// it opens, so it opens at most once per install whatever becomes of the
    /// keybag. `Ok(false)` when the install is not eligible; an error, with
    /// the window shut, when the record could not be made durable.
    pub(crate) fn open_first_setup(&mut self) -> Result<bool> {
        if self.level != 1 || self.setup != Setup::Pin {
            return Ok(false);
        }
        let Some(file) = self.file.as_ref() else {
            return Ok(false);
        };
        // The file's length, not `end`: a torn first entry rounds `end` down
        // to 0 but is still an earlier setup.
        if !file.size().is_ok_and(|size| size == 0) {
            return Ok(false);
        }
        // The list's directory entry was flushed when it was opened. A failed
        // write or flush leaves `end` at 0, so an adoption overwrites the
        // slot, and any bytes that survive keep the window shut.
        file.write_all(0, &setup_consumed())?;
        file.sync()?;
        self.end = ENTRY_SIZE as u64;
        self.setup_recorded = true;
        self.first_setup = FirstSetup::Open;
        pr_info_ratelimited!(
            "xART guard: this install's first Touch ID setup window is open until the first activation returns\n"
        );
        Ok(true)
    }

    /// Closes the first-setup window, used or not.
    pub(crate) fn close_first_setup(&mut self) {
        self.first_setup = FirstSetup::Closed;
    }

    fn is_pinned(&self, key: &Key) -> bool {
        self.pinned.iter().any(|pin| *pin == Some(*key))
    }

    fn pins_full(&self) -> bool {
        self.pinned.iter().all(Option::is_some)
    }

    fn add_pin(&mut self, key: Key) {
        if let Some(slot) = self.pinned.iter_mut().find(|pin| pin.is_none()) {
            *slot = Some(key);
        }
    }

    /// Whether a request the guard would refuse passes as the enclave's
    /// rewrite of one of this install's pinned lockers: under
    /// `xart_guard_pins` at level 1, a rewrite (not a delete) of a locker the
    /// list pins, once the enclave has read it since the store opened.
    fn pinned_allows(&self, key: &Key, write: bool) -> bool {
        self.setup == Setup::Pin
            && self.level == 1
            && write
            && self.is_pinned(key)
            && self.read.contains(key)
    }

    /// Whether the first-setup window pins a request the guard would refuse:
    /// a rewrite of a SEP-xART locker the enclave read since the store
    /// opened, at level 1, with a pin free.
    fn first_setup_pins(&self, key: &Key, write: bool) -> bool {
        self.setup == Setup::Pin
            && self.level == 1
            && self.first_setup == FirstSetup::Open
            && write
            && key.kind == POLICY_KIND
            && self.read.contains(key)
            && !self.is_pinned(key)
            && !self.pins_full()
    }

    /// Records `key` as one of this install's pinned lockers, durably.
    fn pin(&mut self, key: &Key) -> Result<()> {
        if self.pins_full() {
            return Err(ENOSPC);
        }
        let file = self.file.as_ref().ok_or(ENOENT)?;
        // As in adopt(): the file's length is what load() accepts.
        if usize::try_from(self.end).map_or(true, |end| end / ENTRY_SIZE >= MAX_ENTRIES) {
            return Err(ENOSPC);
        }
        file.write_all(self.end, &encode_pin(key))?;
        file.sync()?;
        // Only now that the entry is written and flushed. A failed write or
        // flush leaves the end unchanged, so the next entry overwrites the
        // slot, and a pin that survives there names a locker whose previous
        // value is saved and which the owner's parameter was about to allow.
        self.add_pin(*key);
        self.end += ENTRY_SIZE as u64;
        Ok(())
    }

    /// The write [`Guard::check`] just allowed has its previous value saved:
    /// pins the locker if the first-setup window allowed it, before the
    /// write lands. An error refuses the write.
    pub(crate) fn commit_pin(&mut self, key: &Key) -> Result<()> {
        if self.pending_pin.take() != Some(*key) {
            return Ok(());
        }
        if let Err(e) = self.pin(key) {
            crate::diag::xart_guard_refused();
            self.refused += 1;
            pr_warn_ratelimited!(
                "xART guard: could not record SEP-xART locker {} as pinned ({:?}); refusing to rewrite it\n",
                Id(&key.uuid),
                e
            );
            return Err(EPERM);
        }
        if self.pins_full() {
            self.first_setup = FirstSetup::Used;
        }
        pr_info_ratelimited!(
            "xART guard: pinned the existing SEP-xART locker {} the enclave rewrote during this install's first Touch ID setup ({} of {})\n",
            Id(&key.uuid),
            self.pinned.iter().flatten().count(),
            MAX_PINS
        );
        Ok(())
    }

    fn adopt(&mut self, key: &Key) -> Result<()> {
        let file = self.file.as_ref().ok_or(ENOENT)?;
        // The file's length, not the number of distinct keys: a torn or
        // repeated entry still counts against what load() accepts.
        if usize::try_from(self.end).map_or(true, |end| end / ENTRY_SIZE >= MAX_ENTRIES) {
            return Err(ENOSPC);
        }
        // Reserve room for the in-memory key before writing, so an entry made
        // durable is always listed in memory as well: no allocation can fail
        // after a record is written.
        self.owned.reserve(1, GFP_KERNEL)?;
        file.write_all(self.end, &encode(key))?;
        file.sync()?;
        // Only now that the entry is written and flushed: list it (into the
        // reserved slot, which cannot fail) and then advance the end. A failed
        // write or flush above leaves the end unchanged, so the next adoption
        // overwrites this slot, which holds no record the enclave kept.
        self.owned.push_within_capacity(*key).map_err(|_| ENOMEM)?;
        self.end += ENTRY_SIZE as u64;
        Ok(())
    }

    /// Checks one enclave write (`write`) or delete before it reaches the
    /// gigalocker. An error refuses the request.
    pub(crate) fn check(&mut self, key: &Key, write: bool) -> Result<()> {
        let existed = self.existed.contains(key);
        let owned = self.owned.contains(key);
        self.pending_pin = None;
        // A pinned locker is never adopted as Linux's, even one absent at
        // probe.
        let verdict = if self.is_pinned(key) && !owned {
            Verdict::Refuse
        } else {
            decide(self.level, key, existed, owned)
        };
        match verdict {
            Verdict::Allow => Ok(()),
            Verdict::Adopt if !write => Ok(()),
            Verdict::Adopt => self.adopt(key).inspect_err(|e| {
                crate::diag::xart_guard_refused();
                pr_warn_ratelimited!(
                    "xART guard: could not list a new kind-{} record as Linux's ({:?}); refusing to write it\n",
                    key.kind,
                    e
                )
            }),
            Verdict::Refuse if self.pinned_allows(key, write) => {
                self.pinned_writes += 1;
                if self.pinned_writes == 1 {
                    pr_info_ratelimited!(
                        "xART guard: allowed the enclave to rewrite this install's pinned SEP-xART locker {}\n",
                        Id(&key.uuid)
                    );
                }
                Ok(())
            }
            // Under xart_guard_pins the window passes only a locker the
            // enclave read since the store opened. The store saves its
            // previous value, then commit_pin() pins it, before the write.
            Verdict::Refuse if self.first_setup_pins(key, write) => {
                self.save_previous = Some(Rewrite::FirstSetupPin);
                self.pending_pin = Some(*key);
                Ok(())
            }
            Verdict::Refuse if write => match self.take_allowance_for(key) {
                Some(why) => {
                    self.save_previous = Some(why);
                    Ok(())
                }
                None => Err(self.refuse(key, write)),
            },
            Verdict::Refuse => Err(self.refuse(key, write)),
        }
    }

    fn refuse(&mut self, key: &Key, write: bool) -> Error {
        crate::diag::xart_guard_refused();
        self.refused += 1;
        if self.refused <= LOGGED_REFUSALS {
            let action = if write { "rewrite" } else { "delete" };
            pr_warn_ratelimited!(
                "xART guard: refused to {} kind-{} record {}, which this Linux install did not create\n",
                action,
                key.kind,
                Id(&key.uuid)
            );
            self.explain_pin_refusal(key, write);
        }
        EPERM
    }

    /// Under `xart_guard_pins`, says whether a refused locker was read, and
    /// what to do when the first-setup window is spent with pins free.
    fn explain_pin_refusal(&self, key: &Key, write: bool) {
        if self.setup != Setup::Pin || key.kind != POLICY_KIND {
            return;
        }
        let read = self.read.contains(key);
        pr_info_ratelimited!(
            "xART guard: the enclave {} locker {} since the store opened\n",
            if read { "read" } else { "had not read" },
            Id(&key.uuid)
        );
        if write
            && self.level == 1
            && read
            && self.setup_recorded
            && self.first_setup == FirstSetup::Closed
            && !self.pins_full()
        {
            pr_warn_ratelimited!(
                "xART guard: this install's first-setup window is spent with {} of {} lockers pinned, so this rewrite stays refused and Touch ID may fail. If the first setup did not reach Touch ID's first activation, set up again: install-aurora-sep.sh --reset-touchid --force, then remove {}, which the reset keeps (Linux's earlier records are then treated as another OS's)\n",
                self.pinned.iter().flatten().count(),
                MAX_PINS,
                OWNED_PATH
            );
        }
    }

    /// Uses up the allowance if it covers `key`, else this boot's window
    /// rewrite.
    fn take_allowance_for(&mut self, key: &Key) -> Option<Rewrite> {
        if self.allowance.is_some_and(|allowance| allowance.covers(key)) {
            self.allowance = None;
            return Some(Rewrite::PolicyChange);
        }
        if key.kind != POLICY_KIND || self.window_used {
            return None;
        }
        let why = (self.window)()?;
        self.window_used = true;
        Some(why)
    }

    /// Why the write just allowed rewrites an existing record, if it does.
    pub(crate) fn take_save_previous(&mut self) -> Option<Rewrite> {
        self.save_previous.take()
    }

    /// The previous value could not be saved: the rewrite is refused after
    /// all.
    pub(crate) fn refuse_unsaved(&mut self, key: &Key, why: Rewrite, e: Error) {
        self.pending_pin = None;
        crate::diag::xart_guard_refused();
        self.refused += 1;
        pr_warn_ratelimited!(
            "xART guard: could not save kind-{} record {} before rewriting it {} ({:?}); refusing it\n",
            key.kind,
            Id(&key.uuid),
            why.describe(),
            e
        );
    }
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[kernel::macros::kunit_tests(apple_sep_xart_guard)]
mod tests {
    use super::*;
    use core::cell::{Cell, RefCell};

    fn session(byte: u8) -> Key {
        Key::new(4, [byte; 16])
    }

    fn facts(xart_writable: bool, guard: Option<(u8, bool)>, refusals: u32) -> Matching {
        MatchFacts {
            xart_writable,
            guard,
            refusals,
        }
        .matching()
    }

    #[test]
    fn matching_without_a_guard_is_usable() {
        assert_eq!(facts(true, None, 0), Matching::Usable);
    }

    #[test]
    fn matching_under_a_listed_guard_is_unverified() {
        // An empty list is still "loaded", and does not make the setup
        // unguarded.
        assert_eq!(facts(true, Some((1, true)), 0), Matching::Unverified);
        assert_eq!(facts(true, Some((2, true)), 0), Matching::Unverified);
    }

    #[test]
    fn matching_is_blocked_where_it_is_known_to_be() {
        // No list: nothing that existed at probe is Linux's.
        assert_eq!(facts(true, Some((1, false)), 0), Matching::Blocked);
        // A refusal seen.
        assert_eq!(facts(true, Some((1, true)), 1), Matching::Blocked);
        // xART read-only: no guard runs, and no write can land.
        assert_eq!(facts(false, None, 0), Matching::Blocked);
    }

    #[test]
    fn another_os_record_is_refused() {
        assert_eq!(decide(1, &session(1), true, false), Verdict::Refuse);
        assert_eq!(
            decide(2, &Key::new(3, [1; 16]), true, false),
            Verdict::Refuse
        );
    }

    #[test]
    fn linux_records_stay_writable() {
        assert_eq!(decide(1, &session(2), true, true), Verdict::Allow);
        assert_eq!(decide(1, &session(2), false, true), Verdict::Allow);
    }

    #[test]
    fn a_new_record_is_adopted() {
        assert_eq!(decide(1, &session(3), false, false), Verdict::Adopt);
    }

    #[test]
    fn roots_follow_the_level() {
        assert_eq!(decide(1, &Key::root(1), true, false), Verdict::Allow);
        assert_eq!(decide(1, &Key::root(2), true, false), Verdict::Allow);
        assert_eq!(decide(2, &Key::root(1), true, false), Verdict::Refuse);
        assert_eq!(decide(2, &Key::root(2), false, false), Verdict::Refuse);
    }

    #[test]
    fn entries_round_trip() {
        let key = Key::new(3, [0x5a; 16]);
        assert_eq!(decode(&encode(&key)), Some(key));
    }

    #[test]
    fn damaged_entries_are_ignored() {
        let key = session(7);
        let mut entry = encode(&key);
        entry[9] ^= 1;
        assert_eq!(decode(&entry), None);
        let mut root = encode(&Key::root(1));
        let crc = crc32_ieee(&root[..24]);
        root[24..28].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode(&root), None);
        assert_eq!(decode(&encode(&key)[..31]), None);
    }

    /// Failures a [`MemFile`] reports, switchable while a guard holds it.
    #[derive(Default)]
    struct Faults {
        /// The write fails and changes nothing.
        write: Cell<bool>,
        /// The write lands, but the flush after it fails.
        sync: Cell<bool>,
        /// The directory entry cannot be flushed.
        parent: Cell<bool>,
        /// Reads at or past this offset fail.
        read_from: Cell<Option<u64>>,
    }

    /// A list file in memory, kept across the guards of successive boots, with
    /// optional injected faults.
    struct MemFile<'a>(&'a RefCell<KVec<u8>>, Option<&'a Faults>);

    fn mem(bytes: &RefCell<KVec<u8>>) -> MemFile<'_> {
        MemFile(bytes, None)
    }

    fn faulty<'a>(bytes: &'a RefCell<KVec<u8>>, faults: &'a Faults) -> MemFile<'a> {
        MemFile(bytes, Some(faults))
    }

    impl MemFile<'_> {
        fn fails(&self, fault: impl Fn(&Faults) -> &Cell<bool>) -> bool {
            self.1.is_some_and(|faults| fault(faults).get())
        }
    }

    impl ListFile for MemFile<'_> {
        fn size(&self) -> Result<u64> {
            Ok(self.0.borrow().len() as u64)
        }

        fn read_exact(&self, off: u64, buf: &mut [u8]) -> Result<()> {
            if self
                .1
                .is_some_and(|faults| faults.read_from.get().is_some_and(|from| off >= from))
            {
                return Err(EIO);
            }
            let bytes = self.0.borrow();
            let off = usize::try_from(off).map_err(|_| EINVAL)?;
            let end = off.checked_add(buf.len()).ok_or(EINVAL)?;
            buf.copy_from_slice(bytes.get(off..end).ok_or(EIO)?);
            Ok(())
        }

        fn write_all(&self, off: u64, buf: &[u8]) -> Result<()> {
            if self.fails(|f| &f.write) {
                return Err(EIO);
            }
            let mut bytes = self.0.borrow_mut();
            let off = usize::try_from(off).map_err(|_| EINVAL)?;
            let end = off.checked_add(buf.len()).ok_or(EINVAL)?;
            while bytes.len() < end {
                bytes.push(0, GFP_KERNEL)?;
            }
            bytes[off..end].copy_from_slice(buf);
            Ok(())
        }

        fn sync(&self) -> Result<()> {
            if self.fails(|f| &f.sync) {
                return Err(EIO);
            }
            Ok(())
        }

        fn sync_parent(&self) -> Result<()> {
            if self.fails(|f| &f.parent) {
                return Err(EIO);
            }
            Ok(())
        }
    }

    fn keys(list: &[Key]) -> Result<KVec<Key>> {
        let mut keys = KVec::new();
        for key in list {
            keys.push(*key, GFP_KERNEL)?;
        }
        Ok(keys)
    }

    fn listed(bytes: &RefCell<KVec<u8>>) -> KVec<Option<Key>> {
        let mut entries = KVec::new();
        for entry in bytes.borrow().chunks(ENTRY_SIZE) {
            if entries.push(decode(entry), GFP_KERNEL).is_err() {
                break;
            }
        }
        entries
    }

    #[test]
    fn a_guard_refuses_another_os_record() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let foreign = session(1);
        let mut guard = Guard::with_file(1, keys(&[foreign])?, Ok(mem(&bytes)));
        assert_eq!(guard.effective_level(), 1);
        assert!(guard.list_loaded());
        assert_eq!(guard.check(&foreign, true), Err(EPERM));
        assert_eq!(guard.check(&foreign, false), Err(EPERM));
        // A refusal lists nothing.
        assert_eq!(bytes.borrow().len(), 0);
        Ok(())
    }

    #[test]
    fn a_guard_allows_a_listed_record() -> Result<()> {
        let own = session(2);
        let mut list = KVec::new();
        list.extend_from_slice(&encode(&own), GFP_KERNEL)?;
        let bytes = RefCell::new(list);
        let mut guard = Guard::with_file(1, keys(&[own])?, Ok(mem(&bytes)));
        assert_eq!(guard.check(&own, true), Ok(()));
        assert_eq!(guard.check(&own, false), Ok(()));
        // An entry that is already listed is not written again.
        assert_eq!(bytes.borrow().len(), ENTRY_SIZE);
        Ok(())
    }

    #[test]
    fn the_owned_list_survives_a_reboot() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let foreign = session(1);
        let created = session(3);

        // First boot: the enclave creates a record, which is listed first.
        let mut guard = Guard::with_file(1, keys(&[foreign])?, Ok(mem(&bytes)));
        assert_eq!(guard.check(&created, true), Ok(()));
        assert_eq!(listed(&bytes).as_slice(), &[Some(created)]);
        drop(guard);

        // Next boot: both records predate the guard; only Linux's is writable.
        let mut guard = Guard::with_file(1, keys(&[foreign, created])?, Ok(mem(&bytes)));
        assert_eq!(guard.check(&created, true), Ok(()));
        assert_eq!(guard.check(&created, false), Ok(()));
        assert_eq!(guard.check(&foreign, true), Err(EPERM));
        assert_eq!(listed(&bytes).as_slice(), &[Some(created)]);
        Ok(())
    }

    #[test]
    fn a_torn_tail_entry_is_overwritten() -> Result<()> {
        let first = session(4);
        let second = session(5);
        let mut list = KVec::new();
        list.extend_from_slice(&encode(&first), GFP_KERNEL)?;
        // A crash during the next append left part of an entry behind.
        list.extend_from_slice(&encode(&second)[..10], GFP_KERNEL)?;
        let bytes = RefCell::new(list);

        let mut guard = Guard::with_file(1, keys(&[first])?, Ok(mem(&bytes)));
        assert_eq!(guard.check(&first, true), Ok(()));
        assert_eq!(guard.check(&second, true), Ok(()));
        assert_eq!(listed(&bytes).as_slice(), &[Some(first), Some(second)]);
        Ok(())
    }

    /// A list file of `entries`, each written as given.
    fn list_of(entries: &[Key]) -> Result<KVec<u8>> {
        let mut bytes = KVec::with_capacity(entries.len() * ENTRY_SIZE, GFP_KERNEL)?;
        for key in entries {
            bytes.extend_from_slice(&encode(key), GFP_KERNEL)?;
        }
        Ok(bytes)
    }

    fn numbered(i: usize) -> Key {
        let mut uuid = [0u8; 16];
        uuid[..8].copy_from_slice(&(i as u64 + 1).to_le_bytes());
        Key::new(4, uuid)
    }

    #[test]
    fn a_full_list_stops_adopting_and_still_opens() -> Result<()> {
        let mut all = KVec::with_capacity(MAX_ENTRIES, GFP_KERNEL)?;
        for i in 0..MAX_ENTRIES {
            all.push(numbered(i), GFP_KERNEL)?;
        }
        let bytes = RefCell::new(list_of(&all)?);
        let mut guard = Guard::with_file(1, KVec::new(), Ok(mem(&bytes)));
        assert!(guard.file.is_some());
        assert_eq!(guard.check(&numbered(MAX_ENTRIES), true), Err(ENOSPC));
        assert_eq!(bytes.borrow().len(), MAX_ENTRIES * ENTRY_SIZE);
        drop(guard);
        // Reopened: still within what load() accepts, and Linux's records
        // are still Linux's.
        let mut guard = Guard::with_file(1, keys(&[numbered(7)])?, Ok(mem(&bytes)));
        assert!(guard.file.is_some());
        assert_eq!(guard.check(&numbered(7), true), Ok(()));
        Ok(())
    }

    #[test]
    fn repeated_entries_count_against_the_bound() -> Result<()> {
        // Fewer distinct keys than the bound, but as many entries: a list
        // that reached its length through repeats.
        let distinct = MAX_ENTRIES - 96;
        let mut all = KVec::with_capacity(MAX_ENTRIES, GFP_KERNEL)?;
        for i in 0..MAX_ENTRIES {
            all.push(numbered(i % distinct), GFP_KERNEL)?;
        }
        let bytes = RefCell::new(list_of(&all)?);
        let mut guard = Guard::with_file(1, KVec::new(), Ok(mem(&bytes)));
        assert_eq!(guard.owned.len(), distinct);
        // Appending would make a list the next boot refuses.
        assert_eq!(guard.check(&numbered(MAX_ENTRIES), true), Err(ENOSPC));
        assert_eq!(bytes.borrow().len(), MAX_ENTRIES * ENTRY_SIZE);
        drop(guard);
        let guard = Guard::with_file(1, KVec::new(), Ok(mem(&bytes)));
        assert!(guard.file.is_some());
        Ok(())
    }

    #[test]
    fn a_torn_tail_at_the_bound_takes_the_last_entry() -> Result<()> {
        let mut all = KVec::with_capacity(MAX_ENTRIES - 1, GFP_KERNEL)?;
        for i in 0..MAX_ENTRIES - 1 {
            all.push(numbered(i), GFP_KERNEL)?;
        }
        let mut list = list_of(&all)?;
        list.extend_from_slice(&encode(&numbered(MAX_ENTRIES))[..10], GFP_KERNEL)?;
        let bytes = RefCell::new(list);

        let mut guard = Guard::with_file(1, KVec::new(), Ok(mem(&bytes)));
        // The torn tail is overwritten by the one entry that still fits.
        assert_eq!(guard.check(&numbered(MAX_ENTRIES), true), Ok(()));
        assert_eq!(bytes.borrow().len(), MAX_ENTRIES * ENTRY_SIZE);
        assert_eq!(guard.check(&numbered(MAX_ENTRIES + 1), true), Err(ENOSPC));
        drop(guard);
        let mut guard = Guard::with_file(1, KVec::new(), Ok(mem(&bytes)));
        assert!(guard.file.is_some());
        assert_eq!(guard.check(&numbered(MAX_ENTRIES), true), Ok(()));
        Ok(())
    }

    #[test]
    fn without_the_list_nothing_is_adopted() -> Result<()> {
        let foreign = session(1);
        let mut guard = Guard::<MemFile<'_>>::with_file(1, keys(&[foreign])?, Err(ENOENT));
        assert!(!guard.list_loaded());
        assert_eq!(guard.check(&foreign, true), Err(EPERM));
        assert!(guard.check(&session(6), true).is_err());
        // Deleting a record that does not exist changes nothing.
        assert_eq!(guard.check(&session(6), false), Ok(()));
        assert_eq!(guard.check(&Key::root(1), true), Ok(()));
        Ok(())
    }

    #[test]
    fn a_failed_write_lists_nothing() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let faults = Faults::default();
        let created = session(3);
        let mut guard = Guard::with_file(1, KVec::new(), Ok(faulty(&bytes, &faults)));
        faults.write.set(true);
        // The write fails: nothing is listed and the end did not advance.
        assert_eq!(guard.check(&created, true), Err(EIO));
        assert_eq!(bytes.borrow().len(), 0);
        faults.write.set(false);
        // The same slot takes the record once the write lands.
        assert_eq!(guard.check(&created, true), Ok(()));
        assert_eq!(listed(&bytes).as_slice(), &[Some(created)]);
        Ok(())
    }

    #[test]
    fn a_failed_flush_is_retried_in_the_same_slot() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let faults = Faults::default();
        let refused = session(3);
        let created = session(4);
        let mut guard = Guard::with_file(1, KVec::new(), Ok(faulty(&bytes, &faults)));
        faults.sync.set(true);
        // The write lands but the flush fails: the record is not listed, so it
        // is not writable on a retry either.
        assert_eq!(guard.check(&refused, true), Err(EIO));
        assert_eq!(guard.check(&refused, true), Err(EIO));
        faults.sync.set(false);
        // The next adoption reuses the same slot the unflushed write took, so
        // the list holds no duplicate.
        assert_eq!(guard.check(&created, true), Ok(()));
        assert_eq!(guard.check(&refused, true), Ok(()));
        assert_eq!(listed(&bytes).as_slice(), &[Some(created), Some(refused)]);
        drop(guard);

        // The next boot reads exactly those two, in order.
        let mut guard = Guard::with_file(1, keys(&[created, refused])?, Ok(mem(&bytes)));
        assert_eq!(guard.check(&created, true), Ok(()));
        assert_eq!(guard.check(&refused, true), Ok(()));
        Ok(())
    }

    #[test]
    fn without_a_durable_directory_entry_nothing_is_adopted() -> Result<()> {
        let own = session(2);
        let bytes = RefCell::new(list_of(&[own])?);
        let faults = Faults::default();
        faults.parent.set(true);
        let mut guard = Guard::with_file(1, keys(&[own])?, Ok(faulty(&bytes, &faults)));
        // The directory flush fails, so the file is dropped: as without a
        // list, every existing record is protected and nothing is listed.
        assert!(!guard.list_loaded());
        assert_eq!(guard.check(&own, true), Err(EPERM));
        assert!(guard.check(&session(3), true).is_err());
        assert_eq!(bytes.borrow().len(), ENTRY_SIZE);
        Ok(())
    }

    #[test]
    fn the_default_follows_the_platform() {
        use crate::profile;

        // Off on M1, whose Touch ID setup rewrites a pre-Linux record.
        assert_eq!(level(-1, profile::T8103.xart_guard), None);
        assert_eq!(level(-1, profile::T6000.xart_guard), None);
        // On where Linux shares the enclave with macOS.
        assert_eq!(level(-1, profile::T6020.xart_guard), Some(1));
        assert_eq!(level(-1, profile::T6020_SEPOS13.xart_guard), Some(1));
        assert_eq!(level(-1, profile::T8112.xart_guard), Some(1));
        assert_eq!(level(-1, profile::T8122.xart_guard), Some(1));
        // Off on the Neo, whose Linux records predate the list.
        assert_eq!(level(-1, profile::T8140.xart_guard), None);
        assert_eq!(level(i8::MIN, 1), Some(1));
    }

    #[test]
    fn an_explicit_level_overrides_the_platform() {
        assert_eq!(level(0, 1), None);
        assert_eq!(level(1, 0), Some(1));
        assert_eq!(level(2, 1), Some(2));
        assert_eq!(level(i8::MAX, 0), Some(2));
    }

    /// What `aurora-sep-policy-change` writes.
    fn allowance_entry(allowance: &Allowance) -> [u8; ENTRY_SIZE] {
        match allowance {
            Allowance::Record(key) => encode_with(ALLOW_MAGIC, key),
            Allowance::FirstOfKind(kind) => encode_with(ALLOW_MAGIC, &Key::new(*kind, [0; 16])),
        }
    }

    #[test]
    fn allowances_round_trip() {
        let named = Allowance::Record(session(9));
        assert_eq!(Allowance::decode(&allowance_entry(&named)), Some(named));
        let first = Allowance::FirstOfKind(4);
        assert_eq!(Allowance::decode(&allowance_entry(&first)), Some(first));
        // A list entry is not an allowance, and roots cannot be allowed.
        assert_eq!(Allowance::decode(&encode(&session(9))), None);
        assert_eq!(Allowance::decode(&encode_with(ALLOW_MAGIC, &Key::root(1))), None);
        let mut torn = allowance_entry(&named);
        torn[12] ^= 1;
        assert_eq!(Allowance::decode(&torn), None);
    }

    #[test]
    fn a_named_allowance_rewrites_that_record_once() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let (policy, other) = (session(1), session(2));
        let mut guard = Guard::with_file(1, keys(&[policy, other])?, Ok(mem(&bytes)));
        guard.allowance = Some(Allowance::Record(policy));
        // Never a delete, and never another record.
        assert_eq!(guard.check(&policy, false), Err(EPERM));
        assert_eq!(guard.check(&other, true), Err(EPERM));
        assert_eq!(guard.check(&policy, true), Ok(()));
        assert_eq!(guard.take_save_previous(), Some(Rewrite::PolicyChange));
        assert_eq!(guard.check(&policy, true), Err(EPERM));
        // The record is not adopted.
        assert!(bytes.borrow().is_empty());
        Ok(())
    }

    #[test]
    fn a_first_of_kind_allowance_covers_one_record() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let (a, b) = (session(1), session(2));
        let mut guard = Guard::with_file(1, keys(&[a, b, Key::new(3, [3; 16])])?, Ok(mem(&bytes)));
        guard.allowance = Some(Allowance::FirstOfKind(4));
        assert_eq!(guard.check(&Key::new(3, [3; 16]), true), Err(EPERM));
        assert_eq!(guard.check(&b, true), Ok(()));
        assert_eq!(guard.check(&a, true), Err(EPERM));
        assert_eq!(guard.check(&b, true), Err(EPERM));
        Ok(())
    }

    #[test]
    fn an_allowance_leaves_new_and_listed_records_alone() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let (old, new) = (session(1), session(2));
        let mut guard = Guard::with_file(1, keys(&[old])?, Ok(mem(&bytes)));
        guard.allowance = Some(Allowance::FirstOfKind(4));
        // A new record is adopted as usual and does not use the allowance.
        assert_eq!(guard.check(&new, true), Ok(()));
        assert_eq!(guard.check(&old, true), Ok(()));
        assert_eq!(guard.allowance, None);
        Ok(())
    }

    #[test]
    fn the_policy_window_lets_one_kind_4_rewrite_through() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let (a, b, c) = (session(1), session(2), Key::new(3, [3; 16]));
        let mut guard = Guard::with_file(1, keys(&[a, b, c])?, Ok(mem(&bytes)));
        // Outside the window nothing changes.
        guard.window = || None;
        assert_eq!(guard.check(&a, true), Err(EPERM));
        guard.window = || Some(Rewrite::PolicyChange);
        assert_eq!(guard.check(&c, true), Err(EPERM));
        assert_eq!(guard.check(&a, false), Err(EPERM));
        assert_eq!(guard.check(&a, true), Ok(()));
        assert_eq!(guard.take_save_previous(), Some(Rewrite::PolicyChange));
        assert_eq!(guard.take_save_previous(), None);
        assert_eq!(guard.check(&b, true), Err(EPERM));
        assert_eq!(guard.check(&a, true), Err(EPERM));
        // Not adopted.
        assert!(bytes.borrow().is_empty());
        Ok(())
    }

    #[test]
    fn the_create_window_lets_one_kind_4_rewrite_through() -> Result<()> {
        // A create: one existing kind-4 record rewritten, a new record and a
        // root written.
        let bytes = RefCell::new(KVec::new());
        let attach = session(1);
        let other = session(2);
        let kind3 = Key::new(3, [3; 16]);
        let new = session(9);
        let mut guard = Guard::with_file(1, keys(&[attach, other, kind3])?, Ok(mem(&bytes)));
        guard.window = || Some(Rewrite::KeybagCreate);
        // Never another kind, never a delete.
        assert_eq!(guard.check(&kind3, true), Err(EPERM));
        assert_eq!(guard.check(&attach, false), Err(EPERM));
        // A new record is adopted as before and does not use the window.
        assert_eq!(guard.check(&new, true), Ok(()));
        assert_eq!(guard.take_save_previous(), None);
        assert_eq!(guard.check(&Key::root(2), true), Ok(()));
        assert_eq!(guard.take_save_previous(), None);
        // The one rewrite, which saves the previous value first.
        assert_eq!(guard.check(&attach, true), Ok(()));
        assert_eq!(guard.take_save_previous(), Some(Rewrite::KeybagCreate));
        // Only one: neither the same record again nor another one.
        assert_eq!(guard.check(&attach, true), Err(EPERM));
        assert_eq!(guard.check(&other, true), Err(EPERM));
        // Only the new record is Linux's.
        assert_eq!(listed(&bytes).as_slice(), &[Some(new)]);
        Ok(())
    }

    #[test]
    fn level_2_keeps_the_roots_in_the_create_window() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let mut guard = Guard::with_file(2, keys(&[session(1)])?, Ok(mem(&bytes)));
        guard.window = || Some(Rewrite::KeybagCreate);
        assert_eq!(guard.check(&Key::root(1), true), Err(EPERM));
        assert_eq!(guard.check(&Key::root(2), true), Err(EPERM));
        assert_eq!(guard.take_save_previous(), None);
        Ok(())
    }

    #[test]
    fn an_allowance_is_used_before_the_create_window() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let (named, other) = (session(1), session(2));
        let mut guard = Guard::with_file(1, keys(&[named, other])?, Ok(mem(&bytes)));
        guard.allowance = Some(Allowance::Record(named));
        guard.window = || Some(Rewrite::KeybagCreate);
        assert_eq!(guard.check(&named, true), Ok(()));
        assert_eq!(guard.take_save_previous(), Some(Rewrite::PolicyChange));
        // The window still has its one rewrite.
        assert_eq!(guard.check(&other, true), Ok(()));
        assert_eq!(guard.take_save_previous(), Some(Rewrite::KeybagCreate));
        Ok(())
    }

    #[test]
    fn the_list_length_is_kept_from_load() -> Result<()> {
        let first = session(4);
        let mut list = list_of(&[first])?;
        // A torn tail is not an entry yet.
        list.extend_from_slice(&encode(&session(5))[..10], GFP_KERNEL)?;
        let bytes = RefCell::new(list);
        let mut guard = Guard::with_file(1, keys(&[first])?, Ok(mem(&bytes)));
        assert_eq!(guard.listed_at_load(), Some(1));
        assert_eq!(guard.listed_now(), Some(1));
        assert_eq!(guard.check(&session(6), true), Ok(()));
        assert_eq!(guard.listed_at_load(), Some(1));
        assert_eq!(guard.listed_now(), Some(2));
        let unlisted = Guard::<MemFile<'_>>::with_file(1, KVec::new(), Err(ENOENT));
        assert_eq!(unlisted.listed_at_load(), None);
        assert_eq!(unlisted.listed_now(), None);
        Ok(())
    }

    #[test]
    fn a_create_that_listed_nothing_may_be_sent_again() {
        // The list did not grow since the intent: the enclave kept nothing.
        assert!(may_retry_create(Unfinished::Guarded(0), Some(0)));
        assert!(may_retry_create(Unfinished::Guarded(3), Some(3)));
        // It did: a record of that create may exist.
        assert!(!may_retry_create(Unfinished::Guarded(3), Some(4)));
        // A list shorter than at the intent was rolled back or replaced.
        assert!(!may_retry_create(Unfinished::Guarded(3), Some(2)));
        // No guard, or no list, now: nothing proves it.
        assert!(!may_retry_create(Unfinished::Guarded(3), None));
        assert!(!may_retry_create(Unfinished::Unmarked, None));
        // Sent without a guard: the enclave's records were never listed.
        assert!(!may_retry_create(Unfinished::Unguarded, Some(0)));
        assert!(!may_retry_create(Unfinished::Unguarded, None));
    }

    #[test]
    fn an_older_intent_may_be_sent_again_only_with_an_empty_list() {
        // A create the guard refused before intents recorded the list.
        assert!(may_retry_create(Unfinished::Unmarked, Some(0)));
        assert!(!may_retry_create(Unfinished::Unmarked, Some(1)));
    }

    /// A cold-boot guard under `xart_guard_pins`, with the per-boot windows
    /// shut so that only the pins decide.
    fn pinning<'a>(level: u8, existing: &[Key], bytes: &'a RefCell<KVec<u8>>) -> Result<Guard<MemFile<'a>>> {
        let mut guard = Guard::with_file(level, keys(existing)?, Ok(mem(bytes))).with_setup(Setup::Pin);
        guard.window = || None;
        Ok(guard)
    }

    /// One enclave write as the store makes it: the guard's check, the
    /// previous value saved (here: assumed saved), then the pin.
    fn write<F: ListFile>(guard: &mut Guard<F>, key: &Key) -> Result<()> {
        guard.check(key, true)?;
        let _ = guard.take_save_previous();
        guard.commit_pin(key)
    }

    /// The lockers the list in `bytes` pins, entry by entry.
    fn pins(bytes: &RefCell<KVec<u8>>) -> KVec<Option<Key>> {
        let mut entries = KVec::new();
        for entry in bytes.borrow().chunks(ENTRY_SIZE) {
            if entries.push(decode_pin(entry), GFP_KERNEL).is_err() {
                break;
            }
        }
        entries
    }

    #[test]
    fn the_setup_rule_follows_the_boot_and_the_parameter() {
        assert_eq!(setup(false, 0), Setup::Windows);
        // The parameter changes nothing on a warm-attach Mac.
        assert_eq!(setup(false, 1), Setup::Windows);
        assert_eq!(setup(true, 0), Setup::Windows);
        assert_eq!(setup(true, 1), Setup::Pin);
        assert_eq!(setup(true, u8::MAX), Setup::Pin);
    }

    #[test]
    fn pin_entries_round_trip_and_list_no_record() {
        let locker = session(7);
        let pin = encode_pin(&locker);
        assert_eq!(decode_pin(&pin), Some(locker));
        // To the list of Linux's records, and to a kernel without pinning,
        // a pin is no entry at all, and neither is the used-window marker.
        assert_eq!(decode(&pin), None);
        assert_eq!(decode(&setup_consumed()), None);
        // Nor is either an allowance.
        assert_eq!(Allowance::decode(&pin), None);
        // An owned entry and the marker pin nothing.
        assert_eq!(decode_pin(&encode(&locker)), None);
        assert_eq!(decode_pin(&setup_consumed()), None);
        // Nor does a damaged pin, a short one, or one of a USER-xART locker.
        let mut damaged = pin;
        damaged[9] ^= 1;
        assert_eq!(decode_pin(&damaged), None);
        assert_eq!(decode_pin(&pin[..31]), None);
        assert_eq!(decode_pin(&encode_pin(&Key::new(3, [7; 16]))), None);
    }

    #[test]
    fn a_cold_boot_mac_gets_no_first_setup_window_by_default() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let locker = session(1);
        let mut guard =
            Guard::with_file(1, keys(&[locker])?, Ok(mem(&bytes))).with_setup(Setup::Windows);
        guard.window = || None;
        // Reads are not kept, the window stays shut and nothing is recorded.
        guard.note_read(&locker);
        assert!(guard.read.is_empty());
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(bytes.borrow().len(), 0);
        assert_eq!(write(&mut guard, &locker), Err(EPERM));
        Ok(())
    }

    #[test]
    fn a_pinning_first_setup_pins_up_to_three_lockers_the_enclave_rewrites() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let shared = session(1);
        let unread = session(2);
        let other = session(4);
        let third = session(5);
        let fourth = session(6);
        let user = Key::new(3, [3; 16]);
        let mut guard = pinning(1, &[shared, unread, other, third, fourth, user], &bytes)?;
        guard.note_read(&shared);
        guard.note_read(&other);
        guard.note_read(&third);
        guard.note_read(&fourth);
        guard.note_read(&user);
        // Reading a locker allows nothing by itself.
        assert_eq!(write(&mut guard, &shared), Err(EPERM));
        assert_eq!(guard.open_first_setup(), Ok(true));
        // A locker the enclave never read, a delete and a USER-xART locker
        // stay refused and do not use the window.
        assert_eq!(write(&mut guard, &unread), Err(EPERM));
        assert_eq!(guard.check(&shared, false), Err(EPERM));
        assert_eq!(write(&mut guard, &user), Err(EPERM));
        assert_eq!(pins(&bytes)[..], [None]);
        // Each read locker the setup rewrites is pinned, after the marker,
        // and is not listed as Linux's: up to three (J313: two at COPY_KEYBAG,
        // one during the first activation).
        assert_eq!(write(&mut guard, &shared), Ok(()));
        assert_eq!(write(&mut guard, &other), Ok(()));
        assert_eq!(write(&mut guard, &third), Ok(()));
        assert_eq!(pins(&bytes)[..], [None, Some(shared), Some(other), Some(third)]);
        assert_eq!(listed(&bytes)[..], [None, None, None, None]);
        // A fourth read locker stays refused.
        assert_eq!(write(&mut guard, &fourth), Err(EPERM));
        // A record the setup adds is listed, as on any Mac.
        let created = session(9);
        assert_eq!(write(&mut guard, &created), Ok(()));
        assert_eq!(listed(&bytes)[..], [None, None, None, None, Some(created)]);
        guard.close_first_setup();
        // After the setup the pinned lockers stay writable, for a match's
        // rewrite, and are still never deleted.
        assert_eq!(write(&mut guard, &shared), Ok(()));
        assert_eq!(write(&mut guard, &shared), Ok(()));
        assert_eq!(guard.pinned_writes, 2);
        assert_eq!(guard.check(&shared, false), Err(EPERM));
        assert_eq!(write(&mut guard, &other), Ok(()));
        assert_eq!(write(&mut guard, &third), Ok(()));
        assert_eq!(write(&mut guard, &fourth), Err(EPERM));
        assert_eq!(write(&mut guard, &created), Ok(()));
        assert_eq!(guard.refused, 7);
        // None of that added to the list.
        assert_eq!(pins(&bytes).len(), 5);
        Ok(())
    }

    #[test]
    fn a_pin_is_written_only_after_the_previous_value_is_saved() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let locker = session(1);
        let mut guard = pinning(1, &[locker], &bytes)?;
        guard.note_read(&locker);
        assert_eq!(guard.open_first_setup(), Ok(true));
        assert_eq!(guard.check(&locker, true), Ok(()));
        // The store is told to save the previous value; nothing is pinned yet.
        assert_eq!(guard.take_save_previous(), Some(Rewrite::FirstSetupPin));
        assert_eq!(pins(&bytes)[..], [None]);
        // Saving it failed: the rewrite is refused and nothing is pinned.
        guard.refuse_unsaved(&locker, Rewrite::FirstSetupPin, EIO);
        assert_eq!(guard.commit_pin(&locker), Ok(()));
        assert_eq!(pins(&bytes)[..], [None]);
        // Next time it is saved, then pinned.
        assert_eq!(write(&mut guard, &locker), Ok(()));
        assert_eq!(pins(&bytes)[..], [None, Some(locker)]);
        // A pinned locker's later rewrites save nothing.
        assert_eq!(guard.check(&locker, true), Ok(()));
        assert_eq!(guard.take_save_previous(), None);
        Ok(())
    }

    #[test]
    fn a_pinning_window_passes_a_locker_only_once_it_is_read() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let locker = session(1);
        let mut guard = pinning(1, &[locker], &bytes)?;
        assert_eq!(guard.open_first_setup(), Ok(true));
        assert_eq!(write(&mut guard, &locker), Err(EPERM));
        // Read during the create, it passes: the refusal left the window open.
        guard.note_read(&locker);
        assert_eq!(write(&mut guard, &locker), Ok(()));
        assert_eq!(guard.pinned[0], Some(locker));
        Ok(())
    }

    #[test]
    fn the_pin_survives_a_reboot_and_needs_the_parameter() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let shared = session(1);
        let other = session(2);
        let mut guard = pinning(1, &[shared, other], &bytes)?;
        guard.note_read(&shared);
        assert_eq!(guard.open_first_setup(), Ok(true));
        assert_eq!(write(&mut guard, &shared), Ok(()));
        guard.close_first_setup();
        drop(guard);

        // The next boot, with the parameter: the pin holds once the enclave
        // has read the locker, and the window does not open again.
        let mut guard = pinning(1, &[shared, other], &bytes)?;
        assert_eq!(guard.pinned[0], Some(shared));
        assert!(guard.setup_recorded);
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(write(&mut guard, &shared), Err(EPERM));
        guard.note_read(&shared);
        guard.note_read(&other);
        assert_eq!(write(&mut guard, &shared), Ok(()));
        assert_eq!(write(&mut guard, &other), Err(EPERM));
        drop(guard);

        // Without the parameter the pin is not honoured.
        let mut guard =
            Guard::with_file(1, keys(&[shared, other])?, Ok(mem(&bytes))).with_setup(Setup::Windows);
        guard.window = || None;
        guard.note_read(&shared);
        assert_eq!(write(&mut guard, &shared), Err(EPERM));
        // Even with the read on record, which note_read() does not keep here.
        guard.read.push(shared, GFP_KERNEL)?;
        assert_eq!(write(&mut guard, &shared), Err(EPERM));
        drop(guard);

        // Nor at level 2, which refuses the shared roots as well.
        let mut guard = pinning(2, &[shared, other], &bytes)?;
        guard.note_read(&shared);
        assert_eq!(write(&mut guard, &shared), Err(EPERM));
        Ok(())
    }

    #[test]
    fn a_pin_that_cannot_be_recorded_refuses_the_rewrite() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let faults = Faults::default();
        let locker = session(1);
        let mut guard = Guard::with_file(1, keys(&[locker])?, Ok(faulty(&bytes, &faults)))
            .with_setup(Setup::Pin);
        guard.window = || None;
        guard.note_read(&locker);
        assert_eq!(guard.open_first_setup(), Ok(true));
        faults.sync.set(true);
        // The flush fails: the rewrite is refused and nothing is pinned.
        assert_eq!(write(&mut guard, &locker), Err(EPERM));
        assert_eq!(guard.pinned[0], None);
        faults.sync.set(false);
        // The window is still open, and the same slot takes the pin.
        assert_eq!(write(&mut guard, &locker), Ok(()));
        assert_eq!(pins(&bytes)[..], [None, Some(locker)]);
        Ok(())
    }

    #[test]
    fn a_shut_first_setup_window_pins_nothing() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let (locker, other) = (session(1), session(2));
        let mut guard = pinning(1, &[locker, other], &bytes)?;
        guard.note_read(&locker);
        guard.note_read(&other);
        assert_eq!(guard.open_first_setup(), Ok(true));
        assert_eq!(write(&mut guard, &locker), Ok(()));
        // A failed provisioning step, or the first activation's return.
        guard.close_first_setup();
        assert_eq!(write(&mut guard, &other), Err(EPERM));
        assert_eq!(pins(&bytes)[..], [None, Some(locker)]);
        // The window opens once per install, not once per boot.
        assert_eq!(guard.open_first_setup(), Ok(false));
        Ok(())
    }

    #[test]
    fn the_first_setup_window_needs_level_1_and_an_empty_loaded_list() -> Result<()> {
        // Level 2.
        let bytes = RefCell::new(KVec::new());
        let mut guard = pinning(2, &[], &bytes)?;
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert!(bytes.borrow().is_empty());
        // A list holding anything, a torn entry included.
        let torn = RefCell::new(KVec::new());
        torn.borrow_mut().extend_from_slice(&encode(&session(3))[..10], GFP_KERNEL)?;
        let mut guard = pinning(1, &[], &torn)?;
        assert_eq!(guard.open_first_setup(), Ok(false));
        let owned = RefCell::new(list_of(&[session(3)])?);
        let mut guard = pinning(1, &[session(3)], &owned)?;
        assert_eq!(guard.open_first_setup(), Ok(false));
        // No list.
        let mut guard =
            Guard::<MemFile<'_>>::with_file(1, KVec::new(), Err(ENOENT)).with_setup(Setup::Pin);
        assert_eq!(guard.open_first_setup(), Ok(false));
        // A marker that cannot be made durable keeps it shut.
        let unsynced = RefCell::new(KVec::new());
        let faults = Faults::default();
        let mut guard =
            Guard::with_file(1, KVec::new(), Ok(faulty(&unsynced, &faults))).with_setup(Setup::Pin);
        faults.sync.set(true);
        assert_eq!(guard.open_first_setup(), Err(EIO));
        assert_eq!(guard.first_setup, FirstSetup::Closed);
        Ok(())
    }

    #[test]
    fn the_pins_only_add_to_the_per_boot_windows() -> Result<()> {
        // Under the pins, a locker the enclave did not read still gets the
        // create window's one rewrite, saved and not pinned, as without them.
        let bytes = RefCell::new(KVec::new());
        let (read, unread) = (session(1), session(2));
        let mut guard = pinning(1, &[read, unread], &bytes)?;
        guard.window = || Some(Rewrite::KeybagCreate);
        guard.note_read(&read);
        assert_eq!(guard.open_first_setup(), Ok(true));
        assert_eq!(guard.check(&unread, true), Ok(()));
        assert_eq!(guard.take_save_previous(), Some(Rewrite::KeybagCreate));
        assert_eq!(guard.commit_pin(&unread), Ok(()));
        // A read locker is pinned, and does not use the window.
        assert_eq!(guard.check(&read, true), Ok(()));
        assert_eq!(guard.take_save_previous(), Some(Rewrite::FirstSetupPin));
        assert_eq!(guard.commit_pin(&read), Ok(()));
        assert_eq!(pins(&bytes)[..], [None, Some(read)]);
        // The window's one rewrite is spent.
        assert_eq!(write(&mut guard, &unread), Err(EPERM));
        Ok(())
    }

    #[test]
    fn a_list_that_fails_partway_counts_for_nothing() -> Result<()> {
        let pinned = session(1);
        let owned = session(2);
        let other = session(3);
        let mut list = KVec::new();
        list.extend_from_slice(&setup_consumed(), GFP_KERNEL)?;
        list.extend_from_slice(&encode_pin(&pinned), GFP_KERNEL)?;
        list.extend_from_slice(&encode(&owned), GFP_KERNEL)?;
        list.extend_from_slice(&encode(&other), GFP_KERNEL)?;
        let bytes = RefCell::new(list);
        let faults = Faults::default();
        faults.read_from.set(Some((3 * ENTRY_SIZE) as u64));
        let mut guard = Guard::with_file(
            1,
            keys(&[pinned, owned, other])?,
            Ok(faulty(&bytes, &faults)),
        )
        .with_setup(Setup::Pin);
        guard.window = || None;
        guard.note_read(&pinned);
        assert!(!guard.list_loaded());
        // Neither the pin, the record listed before the failed read nor the
        // used window counts.
        assert_eq!(write(&mut guard, &pinned), Err(EPERM));
        assert_eq!(write(&mut guard, &owned), Err(EPERM));
        assert!(!guard.setup_recorded);
        assert_eq!(guard.listed_at_load(), None);
        assert_eq!(guard.open_first_setup(), Ok(false));
        Ok(())
    }

    #[test]
    fn a_saved_pin_absent_at_probe_is_never_adopted() -> Result<()> {
        let pinned = session(1);
        let mut list = KVec::new();
        list.extend_from_slice(&setup_consumed(), GFP_KERNEL)?;
        list.extend_from_slice(&encode_pin(&pinned), GFP_KERNEL)?;
        let bytes = RefCell::new(list);
        let mut guard = pinning(1, &[], &bytes)?;
        assert_eq!(write(&mut guard, &pinned), Err(EPERM));
        assert_eq!(guard.check(&pinned, false), Err(EPERM));
        assert!(guard.owned.is_empty());
        assert_eq!(listed(&bytes)[..], [None, None]);
        // Recreated and read, it may be rewritten as the pinned locker, but it
        // is still never listed as Linux's or deleted.
        guard.note_read(&pinned);
        assert_eq!(write(&mut guard, &pinned), Ok(()));
        assert_eq!(guard.check(&pinned, false), Err(EPERM));
        assert!(guard.owned.is_empty());
        assert_eq!(listed(&bytes)[..], [None, None]);
        // A new record that is not pinned is still adopted.
        let created = session(9);
        assert_eq!(write(&mut guard, &created), Ok(()));
        assert_eq!(listed(&bytes)[..], [None, None, Some(created)]);
        Ok(())
    }

    #[test]
    fn only_the_first_three_pins_in_the_list_count() -> Result<()> {
        let pinned = [session(1), session(2), session(3)];
        let fourth = session(4);
        let mut list = KVec::new();
        list.extend_from_slice(&setup_consumed(), GFP_KERNEL)?;
        let all = [pinned[0], pinned[1], pinned[2], fourth];
        for key in &all {
            list.extend_from_slice(&encode_pin(key), GFP_KERNEL)?;
        }
        let bytes = RefCell::new(list);
        let mut guard = pinning(1, &all, &bytes)?;
        for key in &all {
            guard.note_read(key);
        }
        for key in &pinned {
            assert_eq!(write(&mut guard, key), Ok(()));
        }
        assert_eq!(write(&mut guard, &fourth), Err(EPERM));
        // No pin made its locker Linux's.
        assert!(guard.owned.is_empty());
        Ok(())
    }

    #[test]
    fn only_a_pinning_guard_remembers_reads() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let locker = session(1);
        let mut guard = Guard::with_file(1, keys(&[locker])?, Ok(mem(&bytes)));
        guard.note_read(&locker);
        assert!(guard.read.is_empty());
        let mut guard = guard.with_setup(Setup::Pin);
        // Not a USER-xART locker or a root.
        guard.note_read(&Key::new(3, [3; 16]));
        guard.note_read(&Key::root(1));
        assert!(guard.read.is_empty());
        // Once each.
        guard.note_read(&locker);
        guard.note_read(&locker);
        assert_eq!(guard.read.len(), 1);
        // Past the bound a locker counts as unread.
        for i in 0..MAX_READ {
            guard.note_read(&numbered(i));
        }
        assert_eq!(guard.read.len(), MAX_READ);
        assert!(!guard.read.contains(&numbered(MAX_READ - 1)));
        Ok(())
    }
}
