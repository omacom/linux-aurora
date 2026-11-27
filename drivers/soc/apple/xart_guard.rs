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
//! M1, M2 and M3 Macs, whose enclave Linux shares with a macOS install, and
//! 0 on the MacBook Neo, which already holds Linux records from before the
//! list existed and would otherwise lose write access to them.
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
//! One exception lets a fresh install set up Touch ID on a warm-attach Mac.
//! Blessing a newly installed OS in macOS leaves a SEP-xART locker (kind 4)
//! that every OS's enclave reads at attach in a pending state, and the next
//! enclave to save its state commits it. macOS's enclave does that at boot;
//! under Linux the first save is the first `CREATE_KEYBAG`. While that create
//! runs on an install whose list is loaded and empty ([`FirstSetup`]), the
//! guard lets the enclave rewrite the first existing kind-4 locker it writes;
//! nothing identifies that locker as the pending one. It does not list the
//! locker as Linux's, and the list records the window as consumed before it
//! opens, so no later create on this install gets it again.
//!
//! Cold-boot Macs may also need to rewrite existing SEP-xART lockers. With
//! `xart_guard_pins=1`, the board's policy limits how many kind-4 lockers the
//! enclave may pin during provisioning and first activation. Each must have
//! been read since probe. Neither the interval between those operations nor a
//! later boot can admit a new pin. A stored pin permits rewriting the same
//! locker after a read, never deleting or adopting it. See XART-SAFETY.md.

use crate::shim::StoreFile;
use crate::xart_store::{crc32_ieee, Key};
use kernel::prelude::*;

/// Host file listing the session records Linux created.
pub(crate) const OWNED_PATH: &CStr = c"/var/lib/aurora-sep-xart-owned.bin";

const ENTRY_SIZE: usize = 32;
const ENTRY_MAGIC: &[u8; 4] = b"AXGO";
/// Bounds the file the driver reads at probe, counted in entries the file
/// holds, torn and repeated ones included. The gigalocker has at most a few
/// hundred slots, so a longer list is damaged. Appending stops at the same
/// bound, so the driver never writes a list it would refuse to read.
const MAX_ENTRIES: usize = 4096;
/// Refusals are logged individually up to this count, then only counted.
const LOGGED_REFUSALS: usize = 8;
/// Marks a list entry as a pinned locker's. It sits in the first of the
/// three bytes an owned entry keeps zero, so a kernel without pinning reads
/// the entry as unreadable and lists nothing by it.
const ENTRY_PINNED: u8 = 1;
/// A completion entry identifying the committed guarded keybag by UUID.
const ENTRY_SETUP_UUID: u8 = 2;
/// SEP-xART lockers the enclave read since probe that a pinning guard
/// remembers. The gigalocker has at most a few hundred slots; a locker past
/// the bound counts as unread.
const MAX_READ: usize = 512;
/// Maximum pins supported by the list format. Admission is board- and
/// operation-specific, and can be lower or zero.
const MAX_PINS: usize = 3;

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

/// The only states that admit new shared lockers are Open and Activating.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FirstSetup {
    Closed,
    /// The creation/provisioning operation is running.
    Open,
    /// Provisioning has already used its allowance.
    Used,
    /// COPY_KEYBAG returned. Only a successful host keybag commit can arm activation.
    Copied,
    /// J313 completed provisioning this boot. No new locker may be pinned
    /// while waiting for someone to open the biometric device.
    AwaitingActivation,
    /// The driver is recovering and activating the provisioned J313 keybag.
    Activating,
}

/// Whether a request the guard would refuse passes as the enclave committing a
/// pending SEP-xART locker during the first keybag creation: a rewrite (not a
/// delete) of a kind-4 record at level 1, while the window is open.
pub(crate) fn first_setup_allows(level: u8, window: FirstSetup, key: &Key, write: bool) -> bool {
    level == 1
        && matches!(window, FirstSetup::Open | FirstSetup::Activating)
        && write
        && key.kind == 4
}

/// What an install's first keybag creation may do to a locker from before
/// Linux, which depends on how the platform's enclave boots.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Setup {
    /// Warm attach: commit one pending SEP-xART locker.
    Commit,
    /// Cold boot: nothing. The create's rewrite is refused.
    Refuse,
    /// J314s, J316s and J316c: one observed provisioning locker.
    PinOne,
    /// J313: two observed provisioning lockers and one in the first activation.
    PinJ313,
    /// J293 has a successful run but no measured count. Preserve existing
    /// pins under opt-in without enrolling any more.
    ExistingPins,
}

impl Setup {
    fn pin_limit(self) -> usize {
        match self {
            Self::PinOne => 1,
            Self::PinJ313 | Self::ExistingPins => MAX_PINS,
            Self::Commit | Self::Refuse => 0,
        }
    }

    fn provisioning_pins(self) -> usize {
        match self {
            Self::PinOne => 1,
            Self::PinJ313 => 2,
            _ => 0,
        }
    }
}

/// Select by the board in the root compatible list, not its shared SoC
/// profile. Unknown boards and other parameter values get no exception.
pub(crate) fn setup(cold: bool, pin: u8, machine_has: impl Fn(&[u8]) -> bool) -> Setup {
    if !cold {
        Setup::Commit
    } else if pin != 1 {
        Setup::Refuse
    } else if machine_has(b"apple,j313") {
        Setup::PinJ313
    } else if machine_has(b"apple,j314s")
        || machine_has(b"apple,j316s")
        || machine_has(b"apple,j316c")
    {
        Setup::PinOne
    } else if machine_has(b"apple,j293") {
        Setup::ExistingPins
    } else {
        Setup::Refuse
    }
}

/// A short tag for one locker in the log, so that a refusal can be compared
/// with the lockers this install pinned. It is not the session id, which the
/// log never carries.
fn tag(key: &Key) -> u32 {
    crc32_ieee(&key.uuid)
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
    /// Unguarded writes are enabled, or a legacy guarded match succeeded this boot.
    Usable = 1,
    /// A guard runs and nothing says it refuses a match, but nothing says it
    /// allows one either.
    Unverified = 2,
    /// A match's write is refused, or known to be.
    Blocked = 3,
    /// A validated keybag and a prior driver's pins exist. A live match
    /// must establish this boot's ability to save the result.
    Legacy = 4,
}

/// Evidence about the identity bag committed on the host.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum KeybagEvidence {
    #[default]
    Unknown,
    /// The prior driver recorded pins without identifying the created bag.
    Legacy,
    /// The guarded commit and the persisted bag have the same UUID.
    Committed,
}

/// What [`Matching`] is derived from.
#[derive(Clone, Copy, Debug)]
pub(crate) struct MatchFacts {
    /// The store opened with writes enabled.
    pub(crate) xart_writable: bool,
    /// The platform cold-boots its enclave.
    pub(crate) cold: bool,
    /// What the list establishes about the validated, persisted identity bag.
    pub(crate) keybag: KeybagEvidence,
    /// The guard's effective level and whether its list loaded, or `None`
    /// when no guard runs.
    pub(crate) guard: Option<(u8, bool)>,
}

impl MatchFacts {
    /// A match may rewrite existing session records. On a cold-boot Mac the
    /// initial diagnostic needs evidence for the persisted identity. A
    /// successful guarded commit establishes its origin. A legacy pin list
    /// can instead qualify for an actual verification of restored fingers.
    /// The sysfs read gives any later refusal precedence over this base state.
    pub(crate) fn matching(&self) -> Matching {
        match self.guard {
            _ if !self.xart_writable => Matching::Blocked,
            None => Matching::Usable,
            Some((_, false)) => Matching::Blocked,
            Some(_) if self.cold => match self.keybag {
                KeybagEvidence::Committed => Matching::Unverified,
                KeybagEvidence::Legacy => Matching::Legacy,
                KeybagEvidence::Unknown => Matching::Blocked,
            },
            Some(_) => Matching::Unverified,
        }
    }
}

fn entry_bytes(kind: u8, uuid: &[u8; 16]) -> [u8; ENTRY_SIZE] {
    let mut entry = [0u8; ENTRY_SIZE];
    entry[..4].copy_from_slice(ENTRY_MAGIC);
    entry[4] = kind;
    entry[8..24].copy_from_slice(uuid);
    let crc = crc32_ieee(&entry[..24]);
    entry[24..28].copy_from_slice(&crc.to_le_bytes());
    entry
}

/// Encodes one list entry.
pub(crate) fn encode(key: &Key) -> [u8; ENTRY_SIZE] {
    entry_bytes(key.kind, &key.uuid)
}

/// The entry that records this install's first-setup window as consumed.
/// Kind 0 is no session record's, so it never lists a record as Linux's.
pub(crate) fn setup_consumed() -> [u8; ENTRY_SIZE] {
    entry_bytes(0, b"first setup used")
}

/// Distinguish a new cold provisioning attempt from the previous driver's
/// spent marker. An interrupted new attempt must never be mistaken for a
/// legacy installation, even if provisioning had already pinned a locker.
fn cold_setup_consumed() -> [u8; ENTRY_SIZE] {
    entry_bytes(0, b"cold setup spent")
}

/// A tagged entry is never decoded as Linux ownership, including by older drivers.
fn tagged_entry(kind: u8, role: u8, uuid: &[u8; 16]) -> [u8; ENTRY_SIZE] {
    let mut entry = entry_bytes(kind, uuid);
    entry[5] = role;
    let crc = crc32_ieee(&entry[..24]);
    entry[24..28].copy_from_slice(&crc.to_le_bytes());
    entry
}

pub(crate) fn encode_pin(key: &Key) -> [u8; ENTRY_SIZE] {
    tagged_entry(key.kind, ENTRY_PINNED, &key.uuid)
}

/// Appended after the guarded keybag is committed on disk. Older drivers
/// still see a nonempty list and skip this entry.
fn committed_keybag(uuid: &[u8; 16]) -> [u8; ENTRY_SIZE] {
    tagged_entry(0, ENTRY_SETUP_UUID, uuid)
}

/// The kind, the three bytes after it and the session id of a whole entry
/// with the list's magic and a good CRC; `None` for a torn or foreign one.
fn parse(entry: &[u8]) -> Option<(u8, [u8; 3], [u8; 16])> {
    if entry.len() != ENTRY_SIZE || &entry[..4] != ENTRY_MAGIC {
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

/// Decodes a pin entry; `None` for anything else, a pin of a record that is
/// not a SEP-xART locker included.
#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
fn decode_pin(entry: &[u8]) -> Option<Key> {
    match parse(entry)? {
        (4, [ENTRY_PINNED, 0, 0], uuid) => Some(Key::new(4, uuid)),
        _ => None,
    }
}

/// Decodes one list entry; `None` for a torn or foreign one.
pub(crate) fn decode(entry: &[u8]) -> Option<Key> {
    match parse(entry)? {
        (kind @ 3..=4, [0, 0, 0], uuid) => Some(Key::new(kind, uuid)),
        _ => None,
    }
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

impl ListFile for StoreFile {
    fn size(&self) -> Result<u64> {
        StoreFile::size(self)
    }

    fn read_exact(&self, off: u64, buf: &mut [u8]) -> Result<()> {
        StoreFile::read_exact(self, off, buf)
    }

    fn write_all(&self, off: u64, buf: &[u8]) -> Result<()> {
        StoreFile::write_all(self, off, buf)
    }

    fn sync(&self) -> Result<()> {
        StoreFile::sync(self)
    }

    fn sync_parent(&self) -> Result<()> {
        StoreFile::sync_parent(self)
    }
}

pub(crate) struct Guard<F: ListFile = StoreFile> {
    level: u8,
    /// `None` when the list could not be opened: nothing can be adopted, so
    /// every session write that is not already listed is refused.
    file: Option<F>,
    /// Byte offset of the next entry.
    end: u64,
    owned: KVec<Key>,
    existed: KVec<Key>,
    refused: usize,
    first_setup: FirstSetup,
    /// What the first-setup window may do on this platform.
    setup: Setup,
    /// UUID saved after the guarded keybag commit. A spent marker alone
    /// establishes no identity.
    setup_uuid: Option<[u8; 16]>,
    /// A complete older list starts with the original spent marker.
    legacy_setup: bool,
    /// Remember old pins even when their use is disabled, so an absent
    /// pinned locker cannot be adopted as a new Linux record.
    pinned: [Option<Key>; MAX_PINS],
    /// Rewrites of pinned lockers let through since probe.
    pinned_writes: usize,
    /// The SEP-xART lockers the enclave read since probe, kept only under
    /// a board's effective pin policy.
    read: KVec<Key>,
}

impl Guard {
    /// Opens Linux's list in the host file and records which keys `existing`
    /// holds now. `setup` is the platform's first-setup rule. Never fails: a
    /// list that cannot be read protects every existing record.
    pub(crate) fn open(level: u8, existing: KVec<Key>, setup: Setup) -> Guard {
        Guard::new(
            level,
            existing,
            StoreFile::open_host(OWNED_PATH).map(|(file, _created)| file),
            setup,
        )
    }
}

impl<F: ListFile> Guard<F> {
    /// As [`Guard::open`], with the list in `file`. There is no default
    /// first-setup rule: every guard is told its platform's.
    fn new(level: u8, existing: KVec<Key>, file: Result<F>, setup: Setup) -> Guard<F> {
        let mut guard = Guard {
            level,
            file: None,
            end: 0,
            owned: KVec::new(),
            existed: existing,
            refused: 0,
            first_setup: FirstSetup::Closed,
            setup,
            setup_uuid: None,
            legacy_setup: false,
            pinned: [None; MAX_PINS],
            pinned_writes: 0,
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
                    // no pin and no spent window from an unreadable list.
                    guard.owned = KVec::new();
                    guard.pinned = [None; MAX_PINS];
                    guard.setup_uuid = None;
                    guard.legacy_setup = false;
                    guard.end = 0;
                    pr_warn!(
                        "xART guard: the list of Linux's records is unreadable ({:?}); every existing session record is protected and none can be adopted\n",
                        e
                    );
                } else {
                    guard.file = Some(file);
                }
            }
            Err(e) => pr_warn!(
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
        pr_info!(
            "xART guard level {}: {} session records exist, {} created by Linux; the other {} are protected{}\n",
            level,
            sessions,
            own,
            sessions - own,
            roots
        );
        guard.announce_pin();
        guard
    }

    /// Whether the list loaded and holds no entry, torn ones included: what a
    /// first-setup window needs.
    fn list_empty(&self) -> bool {
        self.file
            .as_ref()
            .is_some_and(|file| file.size().is_ok_and(|size| size == 0))
    }

    /// Says what `xart_guard_pins` and a pinned locker come to on this boot.
    fn announce_pin(&self) {
        if self.pins_enabled() {
            for key in self.pinned.iter().flatten() {
                pr_info!(
                    "xART guard: existing SEP-xART pin (tag {:08x}) permits rewrites after a read this boot\n",
                    tag(key)
                );
            }
            if self.list_empty() {
                pr_info!(
                    "xART guard: first provisioning may pin up to {} observed SEP-xART lockers on this board\n",
                    self.setup.provisioning_pins()
                );
            }
        } else if self.pinned[0].is_some() {
            pr_info!("xART guard: stored SEP-xART pins are protected on this boot\n");
        }
    }

    /// What a loader can rely on from the running guard. On J293 the opt-in
    /// only preserves a previously populated pin list.
    pub(crate) fn pins_enabled(&self) -> bool {
        self.level == 1
            && self.file.is_some()
            && self.setup.pin_limit() != 0
            && (self.setup != Setup::ExistingPins || self.pinned[0].is_some())
            && self.pin_count() <= self.setup.pin_limit()
    }

    fn load(&mut self, file: &F) -> Result<()> {
        let size = file.size()?;
        let whole = (size as usize / ENTRY_SIZE).min(MAX_ENTRIES + 1);
        if whole > MAX_ENTRIES {
            return Err(EFBIG);
        }
        let mut torn = 0usize;
        let mut consumed = false;
        let mut entry = [0u8; ENTRY_SIZE];
        for i in 0..whole {
            file.read_exact((i * ENTRY_SIZE) as u64, &mut entry)?;
            match decode(&entry) {
                Some(key) if !self.owned.contains(&key) => self.owned.push(key, GFP_KERNEL)?,
                Some(_) => {}
                None if i == 0 && entry == setup_consumed() => self.legacy_setup = true,
                None if i == 0 && entry == cold_setup_consumed() => consumed = true,
                None => match parse(&entry) {
                    Some((0, [ENTRY_SETUP_UUID, 0, 0], uuid)) if consumed => {
                        if self.setup_uuid.replace(uuid).is_some() {
                            return Err(EINVAL);
                        }
                    }
                    Some((4, [ENTRY_PINNED, 0, 0], uuid)) => {
                        let key = Key::new(4, uuid);
                        if !self.is_pinned(&key) {
                            // An omitted pin could be adopted if absent at probe.
                            if self.pin_count() == MAX_PINS {
                                return Err(EINVAL);
                            }
                            self.add_pin(key);
                        }
                    }
                    _ => torn += 1,
                },
            }
        }
        if torn != 0 || size % ENTRY_SIZE as u64 != 0 {
            self.legacy_setup = false;
        }
        if torn != 0 {
            pr_warn!("xART guard: ignored {} unreadable list entries\n", torn);
        }
        // A torn tail entry is overwritten by the next adoption.
        self.end = (whole * ENTRY_SIZE) as u64;
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

    /// The warm-attach exception ends with CREATE_KEYBAG. Cold boards can
    /// also need the following COPY_KEYBAG.
    pub(crate) fn end_create(&mut self) {
        if self.setup == Setup::Commit {
            self.close_first_setup();
        }
    }

    /// Stop admitting new pins before the host validates or writes the bag.
    pub(crate) fn end_copy(&mut self) {
        if matches!(self.first_setup, FirstSetup::Open | FirstSetup::Used) {
            self.first_setup = FirstSetup::Copied;
        }
    }

    /// Called only after the host committed the keybag. A crash before this
    /// entry leaves the window spent and the diagnostic pessimistic.
    pub(crate) fn commit_keybag(&mut self, uuid: &[u8; 16]) -> Result<bool> {
        if self.first_setup != FirstSetup::Copied || !self.pins_enabled() {
            return Ok(false);
        }
        self.close_first_setup();
        self.append(&committed_keybag(uuid))?;
        self.setup_uuid = Some(*uuid);
        if self.setup == Setup::PinJ313 && self.pin_count() == self.setup.provisioning_pins() {
            self.first_setup = FirstSetup::AwaitingActivation;
        }
        Ok(true)
    }

    pub(crate) fn begin_activation(&mut self, uuid: &[u8; 16]) {
        self.first_setup = if self.first_setup == FirstSetup::AwaitingActivation
            && self.keybag_evidence(uuid) == KeybagEvidence::Committed
        {
            FirstSetup::Activating
        } else {
            FirstSetup::Closed
        };
    }

    /// A caller must supply the UUID of a validated, committed host keybag.
    pub(crate) fn keybag_evidence(&self, uuid: &[u8; 16]) -> KeybagEvidence {
        if !self.pins_enabled() {
            KeybagEvidence::Unknown
        } else if self.setup_uuid.as_ref() == Some(uuid) {
            KeybagEvidence::Committed
        } else if self.legacy_setup && self.pinned[0].is_some() {
            KeybagEvidence::Legacy
        } else {
            KeybagEvidence::Unknown
        }
    }

    /// Notes that the enclave was handed the record under `key` since probe.
    /// Only SEP-xART lockers are kept, and only for pin policies, whose
    /// allowances ask for it. A locker that cannot be kept counts as unread,
    /// which they refuse.
    pub(crate) fn note_read(&mut self, key: &Key) {
        if !self.pins_enabled()
            || key.kind != 4
            || self.read.contains(key)
            || self.read.len() >= MAX_READ
        {
            return;
        }
        let _ = self.read.push(*key, GFP_KERNEL);
    }

    /// Opens the first-setup window for one keybag creation, only at level 1
    /// on a fresh install: the list loaded and holding no entry, torn ones
    /// included. A cold-boot Mac has no window unless the owner set
    /// `xart_guard_pins`. The window is recorded as consumed in the list,
    /// durably, before it opens, so it opens at most once for this list
    /// whatever becomes of the keybag. `Ok(false)` when the install is not eligible; an
    /// error, with the window shut, when the record could not be made durable.
    pub(crate) fn open_first_setup(&mut self) -> Result<bool> {
        if self.level != 1 || (self.setup != Setup::Commit && self.setup.provisioning_pins() == 0) {
            return Ok(false);
        }
        let Some(file) = self.file.as_ref() else {
            return Ok(false);
        };
        // The file's length, not `end`: a torn first entry rounds `end` down
        // to 0 but is still an earlier setup.
        if self.end != 0 || !file.size().is_ok_and(|size| size == 0) {
            return Ok(false);
        }
        // The list's directory entry was flushed when it was opened. A failed
        // write or flush leaves `end` at 0, so an adoption overwrites the
        // slot, and any bytes that survive keep the window shut.
        let marker = if self.setup == Setup::Commit {
            setup_consumed()
        } else {
            cold_setup_consumed()
        };
        self.append(&marker)?;
        self.first_setup = FirstSetup::Open;
        Ok(true)
    }

    /// Closes the first-setup window, used or not.
    pub(crate) fn close_first_setup(&mut self) {
        self.first_setup = FirstSetup::Closed;
    }

    /// Advance the cursor only after the entry is durable. A failed write or
    /// sync is retried in the same slot; the file never exceeds the read limit.
    fn append(&mut self, entry: &[u8; ENTRY_SIZE]) -> Result<()> {
        let file = self.file.as_ref().ok_or(ENOENT)?;
        if self.end / ENTRY_SIZE as u64 >= MAX_ENTRIES as u64 {
            return Err(ENOSPC);
        }
        file.write_all(self.end, entry)?;
        file.sync()?;
        self.end += ENTRY_SIZE as u64;
        Ok(())
    }

    fn adopt(&mut self, key: &Key) -> Result<()> {
        // Reserve before making the entry durable.
        self.owned.reserve(1, GFP_KERNEL)?;
        self.append(&encode(key))?;
        self.owned.push_within_capacity(*key).map_err(|_| ENOMEM)
    }

    fn pin(&mut self, key: &Key) -> Result<()> {
        if self.pin_count() >= self.setup.pin_limit() {
            return Err(ENOSPC);
        }
        self.append(&encode_pin(key))?;
        self.add_pin(*key);
        Ok(())
    }

    fn is_pinned(&self, key: &Key) -> bool {
        self.pinned.iter().any(|pin| *pin == Some(*key))
    }

    fn pin_count(&self) -> usize {
        self.pinned.iter().flatten().count()
    }

    fn add_pin(&mut self, key: Key) {
        if let Some(slot) = self.pinned.iter_mut().find(|pin| pin.is_none()) {
            *slot = Some(key);
        }
    }

    /// Whether a request the guard would refuse passes as the enclave's
    /// rewrite of one of this install's pinned lockers: under
    /// `xart_guard_pins` at level 1, a rewrite (not a delete) of a locker the
    /// list pins, once the enclave has read it since probe.
    fn pinned_allows(&self, key: &Key, write: bool) -> bool {
        self.pins_enabled() && write && self.is_pinned(key) && self.read.contains(key)
    }

    /// Checks one enclave write (`write`) or delete before it reaches the
    /// gigalocker. An error refuses the request.
    pub(crate) fn check(&mut self, key: &Key, write: bool) -> Result<()> {
        let existed = self.existed.contains(key);
        let owned = self.owned.contains(key);
        // A pinned locker is never adopted as Linux's, even one absent at probe.
        let verdict = if self.is_pinned(key) {
            Verdict::Refuse
        } else {
            decide(self.level, key, existed, owned)
        };
        match verdict {
            Verdict::Allow => Ok(()),
            Verdict::Adopt if !write => Ok(()),
            Verdict::Adopt => self.adopt(key).inspect_err(|e| {
                crate::diag::xart_guard_refused();
                pr_warn!(
                    "xART guard: could not list a new kind-{} record as Linux's ({:?}); refusing to write it\n",
                    key.kind,
                    e
                )
            }),
            Verdict::Refuse if self.pinned_allows(key, write) => {
                self.pinned_writes += 1;
                if self.pinned_writes == 1 {
                    pr_info!(
                        "xART guard: allowed the enclave to rewrite this install's pinned SEP-xART locker (tag {:08x})\n",
                        tag(key)
                    );
                }
                Ok(())
            }
            // Under xart_guard_pins the window passes only a locker the
            // enclave read since probe, and pins it before the write lands.
            Verdict::Refuse
                if self.pins_enabled()
                    && first_setup_allows(self.level, self.first_setup, key, write)
                    && self.read.contains(key) =>
            {
                self.pin(key).inspect_err(|e| {
                    crate::diag::xart_guard_refused();
                    pr_warn!(
                        "xART guard: could not record the SEP-xART locker to pin ({:?}); refusing to rewrite it\n",
                        e
                    )
                })?;
                if self.first_setup == FirstSetup::Activating {
                    self.close_first_setup();
                } else if self.pin_count() >= self.setup.provisioning_pins() {
                    self.first_setup = FirstSetup::Used;
                }
                pr_info!(
                    "xART guard: pinned the existing SEP-xART locker the enclave rewrote during this install's first Touch ID setup (tag {:08x})\n",
                    tag(key)
                );
                Ok(())
            }
            Verdict::Refuse
                if self.setup == Setup::Commit
                    && first_setup_allows(self.level, self.first_setup, key, write) =>
            {
                self.first_setup = FirstSetup::Used;
                pr_info!(
                    "xART guard: allowed the enclave to commit an existing kind-{} locker during this install's first keybag creation\n",
                    key.kind
                );
                Ok(())
            }
            Verdict::Refuse => {
                crate::diag::xart_guard_refused();
                self.refused += 1;
                if self.refused <= LOGGED_REFUSALS {
                    let action = if write { "rewrite" } else { "delete" };
                    pr_warn!(
                        "xART guard: refused to {} a kind-{} record this Linux install did not create\n",
                        action,
                        key.kind
                    );
                    // Under xart_guard_pins, which locker and whether the
                    // enclave had read it tell the refusals apart: the
                    // pinned lockers before a read, or another one.
                    if self.pins_enabled() && key.kind == 4 {
                        let read = if self.read.contains(key) {
                            "read"
                        } else {
                            "had not read"
                        };
                        pr_info!(
                            "xART guard: the refused locker has tag {:08x}; the enclave {} it this boot\n",
                            tag(key),
                            read
                        );
                    }
                }
                Err(EPERM)
            }
        }
    }
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[kernel::macros::kunit_tests(apple_sep_xart_guard)]
mod tests {
    use super::*;
    use core::cell::{Cell, RefCell};

    /// The tests' shorthand: a warm-attach Mac's guard, as every guard was
    /// before a cold-boot Mac had a rule of its own. The driver itself has no
    /// default and always names the rule.
    impl<F: ListFile> Guard<F> {
        fn with_file(level: u8, existing: KVec<Key>, file: Result<F>) -> Guard<F> {
            Guard::new(level, existing, file, Setup::Commit)
        }

        /// The same guard on a platform with the rule `setup`.
        fn with_setup(mut self, setup: Setup) -> Guard<F> {
            self.setup = setup;
            self.announce_pin();
            self
        }
    }

    const BAG: [u8; 16] = [0x42; 16];

    fn session(byte: u8) -> Key {
        Key::new(4, [byte; 16])
    }

    fn facts(xart_writable: bool, cold: bool, guard: Option<(u8, bool)>) -> Matching {
        MatchFacts {
            xart_writable,
            cold,
            keybag: KeybagEvidence::Unknown,
            guard,
        }
        .matching()
    }

    /// As [`facts`], on a Mac whose first keybag creation ran under the guard.
    fn facts_after_setup(xart_writable: bool, cold: bool, guard: Option<(u8, bool)>) -> Matching {
        MatchFacts {
            xart_writable,
            cold,
            keybag: KeybagEvidence::Committed,
            guard,
        }
        .matching()
    }

    #[test]
    fn matching_without_a_guard_is_usable() {
        // The Neo (warm, level 0) and an M1 Pro pinned to xart_guard=0 (cold).
        assert_eq!(facts(true, false, None), Matching::Usable);
        assert_eq!(facts(true, true, None), Matching::Usable);
    }

    #[test]
    fn matching_under_a_listed_warm_guard_is_unverified() {
        // An M2 set up under the guard. An empty list is still "loaded", and
        // does not make the setup unguarded.
        assert_eq!(facts(true, false, Some((1, true))), Matching::Unverified);
        assert_eq!(facts(true, false, Some((2, true))), Matching::Unverified);
    }

    #[test]
    fn matching_on_a_cold_boot_mac_follows_its_setup() {
        // Set up under the guard with xart_guard_pins: unverified, as a
        // warm-attach Mac is.
        assert_eq!(
            facts_after_setup(true, true, Some((1, true))),
            Matching::Unverified
        );
        // The fact changes nothing on a warm-attach Mac, or without a guard.
        assert_eq!(
            facts_after_setup(true, false, Some((1, true))),
            Matching::Unverified
        );
        assert_eq!(facts_after_setup(true, true, None), Matching::Usable);
        // What blocks any Mac still blocks this one.
        assert_eq!(
            facts_after_setup(true, true, Some((1, false))),
            Matching::Blocked
        );
        assert_eq!(facts_after_setup(false, true, None), Matching::Blocked);
    }

    #[test]
    fn matching_is_blocked_where_it_is_known_to_be() {
        // A cold-boot M1-family Mac under the default level 1, not set up
        // under the guard: its keybag was created with the guard off.
        assert_eq!(facts(true, true, Some((1, true))), Matching::Blocked);
        // No list: nothing that existed at probe is Linux's.
        assert_eq!(facts(true, false, Some((1, false))), Matching::Blocked);
        // xART read-only: no guard runs, and no write can land.
        assert_eq!(facts(false, false, None), Matching::Blocked);
        assert_eq!(facts(false, true, None), Matching::Blocked);
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
            if self.1.and_then(|faults| faults.read_from.get()).is_some_and(|from| off >= from) {
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
    fn first_setup_allows_only_a_kind_4_rewrite_at_level_1() {
        let open = FirstSetup::Open;
        assert!(first_setup_allows(1, open, &session(1), true));
        assert!(!first_setup_allows(1, open, &session(1), false));
        assert!(!first_setup_allows(1, open, &Key::new(3, [1; 16]), true));
        assert!(!first_setup_allows(2, open, &session(1), true));
        assert!(!first_setup_allows(1, FirstSetup::Closed, &session(1), true));
        assert!(!first_setup_allows(1, FirstSetup::Used, &session(1), true));
    }

    #[test]
    fn a_fresh_install_commits_one_locker_during_its_first_create() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let pending = session(1);
        let other = session(2);
        let user = Key::new(3, [3; 16]);
        let mut guard = Guard::with_file(1, keys(&[pending, other, user])?, Ok(mem(&bytes)));
        // Outside the window the locker is protected as before.
        assert_eq!(guard.check(&pending, true), Err(EPERM));
        assert_eq!(guard.open_first_setup(), Ok(true));
        // The window is consumed in the list before it opens.
        assert_eq!(&bytes.borrow()[..], &setup_consumed()[..]);
        // A delete, or a USER-xART locker, does not use the window.
        assert_eq!(guard.check(&other, false), Err(EPERM));
        assert_eq!(guard.check(&user, true), Err(EPERM));
        assert_eq!(guard.check(&pending, true), Ok(()));
        // One rewrite only, and the locker does not become Linux's.
        assert_eq!(guard.check(&other, true), Err(EPERM));
        assert_eq!(listed(&bytes)[..], [None]);
        guard.close_first_setup();
        assert_eq!(guard.check(&pending, true), Err(EPERM));
        assert_eq!(guard.refused, 5);
        Ok(())
    }

    #[test]
    fn first_setup_opens_once_per_install() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let pending = session(1);
        let mut guard = Guard::with_file(1, keys(&[pending])?, Ok(mem(&bytes)));
        assert_eq!(guard.open_first_setup(), Ok(true));
        guard.close_first_setup();
        // A second create this boot, unused window or not.
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(guard.check(&pending, true), Err(EPERM));

        // After a reboot, or with the keybag file removed by a Touch ID reset,
        // which keeps the list.
        let mut guard = Guard::with_file(1, keys(&[pending])?, Ok(mem(&bytes)));
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(guard.check(&pending, true), Err(EPERM));
        assert_eq!(bytes.borrow().len(), ENTRY_SIZE);
        Ok(())
    }

    #[test]
    fn the_consumed_marker_lists_no_record() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        bytes.borrow_mut().extend_from_slice(&setup_consumed(), GFP_KERNEL)?;
        let fresh = session(5);
        let mut guard = Guard::with_file(1, KVec::new(), Ok(mem(&bytes)));
        assert!(guard.list_loaded());
        assert_eq!(guard.owned.len(), 0);
        // An adoption appends after the marker and keeps it.
        assert_eq!(guard.check(&fresh, true), Ok(()));
        assert_eq!(listed(&bytes)[..], [None, Some(fresh)]);
        assert_eq!(&bytes.borrow()[..ENTRY_SIZE], &setup_consumed()[..]);
        assert_eq!(guard.open_first_setup(), Ok(false));

        // Adopting in the same boot as the window also appends after it.
        let bytes = RefCell::new(KVec::new());
        let mut guard = Guard::with_file(1, KVec::new(), Ok(mem(&bytes)));
        assert_eq!(guard.open_first_setup(), Ok(true));
        assert_eq!(guard.check(&fresh, true), Ok(()));
        assert_eq!(listed(&bytes)[..], [None, Some(fresh)]);
        Ok(())
    }

    #[test]
    fn first_setup_stays_shut_when_it_cannot_be_recorded() -> Result<()> {
        let pending = session(1);
        let fresh = session(5);

        // The write fails: nothing lands, and the window stays shut.
        let bytes = RefCell::new(KVec::new());
        let faults = Faults::default();
        faults.write.set(true);
        let mut guard = Guard::with_file(1, keys(&[pending])?, Ok(faulty(&bytes, &faults)));
        assert_eq!(guard.open_first_setup(), Err(EIO));
        assert_eq!(guard.check(&pending, true), Err(EPERM));
        faults.write.set(false);
        assert_eq!(guard.open_first_setup(), Ok(true));

        // The marker lands but its flush fails: the window stays shut, and
        // the bytes that landed keep it shut on every later attempt.
        let bytes = RefCell::new(KVec::new());
        let faults = Faults::default();
        faults.sync.set(true);
        let mut guard = Guard::with_file(1, keys(&[pending])?, Ok(faulty(&bytes, &faults)));
        assert_eq!(guard.open_first_setup(), Err(EIO));
        assert_eq!(guard.check(&pending, true), Err(EPERM));
        faults.sync.set(false);
        assert_eq!(guard.open_first_setup(), Ok(false));
        // The unflushed slot is the next adoption's.
        assert_eq!(guard.check(&fresh, true), Ok(()));
        assert_eq!(listed(&bytes)[..], [Some(fresh)]);
        let mut guard = Guard::with_file(1, keys(&[pending])?, Ok(mem(&bytes)));
        assert_eq!(guard.open_first_setup(), Ok(false));
        Ok(())
    }

    #[test]
    fn first_setup_needs_level_1_and_an_empty_loaded_list() -> Result<()> {
        let pending = session(1);
        let own = session(2);
        let bytes = RefCell::new(list_of(&[own])?);
        let mut guard = Guard::with_file(1, keys(&[pending, own])?, Ok(mem(&bytes)));
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(guard.check(&pending, true), Err(EPERM));

        // A torn entry is still an earlier setup.
        let torn = RefCell::new(KVec::new());
        torn.borrow_mut().extend_from_slice(&encode(&own)[..10], GFP_KERNEL)?;
        let mut guard = Guard::with_file(1, keys(&[pending])?, Ok(mem(&torn)));
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(torn.borrow().len(), 10);

        let mut guard = Guard::<MemFile<'_>>::with_file(1, keys(&[pending])?, Err(ENOENT));
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(guard.check(&pending, true), Err(EPERM));

        // Level 2 gets no window and consumes nothing.
        let empty = RefCell::new(KVec::new());
        let mut guard = Guard::with_file(2, keys(&[pending])?, Ok(mem(&empty)));
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(empty.borrow().len(), 0);
        Ok(())
    }

    #[test]
    fn the_setup_rule_requires_a_qualified_board_and_explicit_opt_in() {
        let select = |cold, pin, board: &[u8]| setup(cold, pin, |compatible| compatible == board);
        assert_eq!(select(false, 1, b"apple,j313"), Setup::Commit);
        assert_eq!(select(true, 0, b"apple,j313"), Setup::Refuse);
        assert_eq!(select(true, 2, b"apple,j313"), Setup::Refuse);
        assert_eq!(select(true, 1, b"apple,j313"), Setup::PinJ313);
        for board in [b"apple,j314s", b"apple,j316s", b"apple,j316c"] {
            assert_eq!(select(true, 1, board), Setup::PinOne);
        }
        assert_eq!(select(true, 1, b"apple,j293"), Setup::ExistingPins);
        assert_eq!(select(true, 1, b"apple,t6001"), Setup::Refuse);
    }

    #[test]
    fn pin_entries_round_trip_and_list_no_record() {
        let locker = session(7);
        let pin = encode_pin(&locker);
        assert_eq!(decode_pin(&pin), Some(locker));
        // To the list of Linux's records, and to a kernel without pinning,
        // a pin is no entry at all.
        assert_eq!(decode(&pin), None);
        // An owned entry and the consumed marker pin nothing.
        assert_eq!(decode_pin(&encode(&locker)), None);
        assert_eq!(decode_pin(&setup_consumed()), None);
        // Nor does a damaged pin, a short one, or one of a USER-xART locker.
        let mut damaged = pin;
        damaged[9] ^= 1;
        assert_eq!(decode_pin(&damaged), None);
        assert_eq!(decode_pin(&pin[..31]), None);
        assert_eq!(decode_pin(&encode_pin(&Key::new(3, [7; 16]))), None);
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
    fn a_cold_boot_mac_gets_no_window_by_default() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let locker = session(1);
        let mut guard =
            Guard::with_file(1, keys(&[locker])?, Ok(mem(&bytes))).with_setup(Setup::Refuse);
        // Reads are not kept, the window stays shut and nothing is recorded.
        guard.note_read(&locker);
        assert!(guard.read.is_empty());
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(bytes.borrow().len(), 0);
        assert_eq!(guard.check(&locker, true), Err(EPERM));
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Unknown);
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
        let mut guard = Guard::with_file(
            1,
            keys(&[shared, unread, other, third, fourth, user])?,
            Ok(mem(&bytes)),
        )
        .with_setup(Setup::PinJ313);
        guard.note_read(&shared);
        guard.note_read(&other);
        guard.note_read(&third);
        guard.note_read(&fourth);
        guard.note_read(&user);
        assert!(guard.pins_enabled());
        // Reading a locker allows nothing by itself.
        assert_eq!(guard.check(&shared, true), Err(EPERM));
        assert_eq!(guard.open_first_setup(), Ok(true));
        // A locker the enclave never read, a delete and a USER-xART locker
        // stay refused and do not use the window.
        assert_eq!(guard.check(&unread, true), Err(EPERM));
        assert_eq!(guard.check(&shared, false), Err(EPERM));
        assert_eq!(guard.check(&user, true), Err(EPERM));
        assert_eq!(pins(&bytes)[..], [None]);
        // Each read locker the setup rewrites is pinned, after the marker,
        // and is not listed as Linux's: up to three (J313: two at COPY_KEYBAG,
        // one during the first activation).
        assert_eq!(guard.check(&shared, true), Ok(()));
        assert_eq!(guard.check(&other, true), Ok(()));
        assert_eq!(guard.check(&third, true), Err(EPERM));
        guard.end_copy();
        assert_eq!(guard.commit_keybag(&BAG), Ok(true));
        // No userspace open yet. Other requests still cannot pin anything.
        assert_eq!(guard.check(&third, true), Err(EPERM));
        guard.begin_activation(&BAG);
        assert_eq!(guard.check(&third, true), Ok(()));
        assert_eq!(
            pins(&bytes)[..],
            [None, Some(shared), Some(other), None, Some(third)]
        );
        assert_eq!(listed(&bytes)[..], [None, None, None, None, None]);
        // A fourth read locker stays refused.
        assert_eq!(guard.check(&fourth, true), Err(EPERM));
        // A record the setup adds is listed, as on any Mac.
        let created = session(9);
        assert_eq!(guard.check(&created, true), Ok(()));
        assert_eq!(
            listed(&bytes)[..],
            [None, None, None, None, None, Some(created)]
        );
        guard.close_first_setup();
        // After the setup the pinned lockers stay writable, for a match's
        // rewrite, and are still never deleted.
        assert_eq!(guard.check(&shared, true), Ok(()));
        assert_eq!(guard.check(&shared, true), Ok(()));
        assert_eq!(guard.pinned_writes, 2);
        assert_eq!(guard.check(&shared, false), Err(EPERM));
        assert_eq!(guard.check(&other, true), Ok(()));
        assert_eq!(guard.check(&third, true), Ok(()));
        assert_eq!(guard.check(&fourth, true), Err(EPERM));
        assert_eq!(guard.check(&created, true), Ok(()));
        assert_eq!(guard.refused, 9);
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Committed);
        Ok(())
    }

    #[test]
    fn a_pinning_window_passes_a_locker_only_once_it_is_read() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let locker = session(1);
        let mut guard =
            Guard::with_file(1, keys(&[locker])?, Ok(mem(&bytes))).with_setup(Setup::PinOne);
        assert_eq!(guard.open_first_setup(), Ok(true));
        assert_eq!(guard.check(&locker, true), Err(EPERM));
        // Read during the create, it passes: the refusal left the window open.
        guard.note_read(&locker);
        assert_eq!(guard.check(&locker, true), Ok(()));
        assert_eq!(guard.pinned[0], Some(locker));
        Ok(())
    }

    #[test]
    fn the_pin_survives_a_reboot_and_needs_the_parameter() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let shared = session(1);
        let other = session(2);
        let mut guard =
            Guard::with_file(1, keys(&[shared, other])?, Ok(mem(&bytes))).with_setup(Setup::PinOne);
        guard.note_read(&shared);
        assert_eq!(guard.open_first_setup(), Ok(true));
        assert_eq!(guard.check(&shared, true), Ok(()));
        guard.end_copy();
        assert_eq!(guard.commit_keybag(&BAG), Ok(true));
        guard.close_first_setup();
        drop(guard);

        // The next boot, with the parameter: the pin holds once the enclave
        // has read the locker, and the window does not open again.
        let mut guard =
            Guard::with_file(1, keys(&[shared, other])?, Ok(mem(&bytes))).with_setup(Setup::PinOne);
        assert_eq!(guard.pinned[0], Some(shared));
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Committed);
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(guard.check(&shared, true), Err(EPERM));
        guard.note_read(&shared);
        guard.note_read(&other);
        assert_eq!(guard.check(&shared, true), Ok(()));
        assert_eq!(guard.check(&other, true), Err(EPERM));
        drop(guard);

        // Without the parameter the pin is not honoured, and matching is
        // blocked as on any cold-boot Mac.
        let mut guard =
            Guard::with_file(1, keys(&[shared, other])?, Ok(mem(&bytes))).with_setup(Setup::Refuse);
        guard.note_read(&shared);
        assert_eq!(guard.check(&shared, true), Err(EPERM));
        // Even with the read on record, which note_read() does not keep here.
        guard.read.push(shared, GFP_KERNEL)?;
        assert_eq!(guard.check(&shared, true), Err(EPERM));
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Unknown);
        drop(guard);

        // Nor at level 2, which refuses the shared roots as well.
        let mut guard =
            Guard::with_file(2, keys(&[shared, other])?, Ok(mem(&bytes))).with_setup(Setup::PinOne);
        guard.note_read(&shared);
        assert_eq!(guard.check(&shared, true), Err(EPERM));
        Ok(())
    }

    #[test]
    fn a_pin_that_cannot_be_recorded_refuses_the_rewrite() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let faults = Faults::default();
        let locker = session(1);
        let mut guard = Guard::with_file(1, keys(&[locker])?, Ok(faulty(&bytes, &faults)))
            .with_setup(Setup::PinOne);
        guard.note_read(&locker);
        assert_eq!(guard.open_first_setup(), Ok(true));
        faults.sync.set(true);
        // The flush fails: the rewrite is refused and nothing is pinned.
        assert_eq!(guard.check(&locker, true), Err(EIO));
        assert_eq!(guard.pinned[0], None);
        faults.sync.set(false);
        // The window is still open, and the same slot takes the pin.
        assert_eq!(guard.check(&locker, true), Ok(()));
        assert_eq!(pins(&bytes)[..], [None, Some(locker)]);
        Ok(())
    }

    #[test]
    fn only_a_pinning_window_outlasts_the_create() -> Result<()> {
        let locker = session(1);
        // Warm attach: the window passes a locker rewrite during the create,
        // and nothing once the create is over.
        let bytes = RefCell::new(KVec::new());
        let mut guard = Guard::with_file(1, keys(&[locker])?, Ok(mem(&bytes)));
        assert_eq!(guard.open_first_setup(), Ok(true));
        guard.end_create();
        assert_eq!(guard.check(&locker, true), Err(EPERM));
        let bytes = RefCell::new(KVec::new());
        let mut guard = Guard::with_file(1, keys(&[locker])?, Ok(mem(&bytes)));
        assert_eq!(guard.open_first_setup(), Ok(true));
        assert_eq!(guard.check(&locker, true), Ok(()));
        // Pin: the window still pins a read locker after the create.
        let bytes = RefCell::new(KVec::new());
        let other = session(2);
        let mut guard =
            Guard::with_file(1, keys(&[locker, other])?, Ok(mem(&bytes))).with_setup(Setup::PinOne);
        guard.note_read(&locker);
        assert_eq!(guard.open_first_setup(), Ok(true));
        guard.end_create();
        assert_eq!(guard.check(&locker, true), Ok(()));
        assert_eq!(pins(&bytes)[..], [None, Some(locker)]);
        guard.close_first_setup();
        guard.note_read(&other);
        assert_eq!(guard.check(&other, true), Err(EPERM));
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
        .with_setup(Setup::PinOne);
        guard.note_read(&pinned);
        assert!(!guard.list_loaded());
        // Neither the pin, the record listed before the failed read nor the
        // spent window counts.
        assert_eq!(guard.check(&pinned, true), Err(EPERM));
        assert_eq!(guard.check(&owned, true), Err(EPERM));
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Unknown);
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
        let none: [Key; 0] = [];
        let mut guard =
            Guard::with_file(1, keys(&none)?, Ok(mem(&bytes))).with_setup(Setup::PinOne);
        assert_eq!(guard.check(&pinned, true), Err(EPERM));
        assert_eq!(guard.check(&pinned, false), Err(EPERM));
        assert!(guard.owned.is_empty());
        assert_eq!(listed(&bytes)[..], [None, None]);
        // Recreated and read, it may be rewritten as the pinned locker, but it
        // is still never listed as Linux's or deleted.
        guard.note_read(&pinned);
        assert_eq!(guard.check(&pinned, true), Ok(()));
        assert_eq!(guard.check(&pinned, false), Err(EPERM));
        assert!(guard.owned.is_empty());
        assert_eq!(listed(&bytes)[..], [None, None]);
        // A new record that is not pinned is still adopted.
        let created = session(9);
        assert_eq!(guard.check(&created, true), Ok(()));
        assert_eq!(listed(&bytes)[..], [None, None, Some(created)]);
        Ok(())
    }

    #[test]
    fn an_unrepresentable_pin_list_fails_closed_even_for_absent_records() -> Result<()> {
        let mut list = KVec::new();
        list.extend_from_slice(&setup_consumed(), GFP_KERNEL)?;
        for byte in 1..=4 {
            list.extend_from_slice(&encode_pin(&session(byte)), GFP_KERNEL)?;
        }
        let bytes = RefCell::new(list);
        let mut guard = Guard::new(1, KVec::new(), Ok(mem(&bytes)), Setup::PinJ313);
        assert!(!guard.list_loaded());
        assert!(!guard.pins_enabled());
        assert!(guard.check(&session(4), true).is_err());
        Ok(())
    }

    #[test]
    fn legacy_j293_pins_survive_without_a_new_admission_window() -> Result<()> {
        let empty = RefCell::new(KVec::new());
        let mut fresh = Guard::new(1, KVec::new(), Ok(mem(&empty)), Setup::ExistingPins);
        assert_eq!(fresh.open_first_setup(), Ok(false));
        assert!(!fresh.pins_enabled());
        let pinned = session(1);
        let other = session(2);
        let mut list = KVec::new();
        list.extend_from_slice(&setup_consumed(), GFP_KERNEL)?;
        list.extend_from_slice(&encode_pin(&pinned), GFP_KERNEL)?;
        let bytes = RefCell::new(list);
        let mut guard = Guard::new(
            1,
            keys(&[pinned, other])?,
            Ok(mem(&bytes)),
            Setup::ExistingPins,
        );
        assert!(guard.pins_enabled());
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Legacy);
        assert_eq!(
            MatchFacts {
                xart_writable: true,
                cold: true,
                keybag: guard.keybag_evidence(&BAG),
                guard: Some((1, true)),
            }
            .matching(),
            Matching::Legacy
        );
        guard.note_read(&pinned);
        guard.note_read(&other);
        assert_eq!(guard.check(&pinned, true), Ok(()));
        assert_eq!(guard.check(&pinned, false), Err(EPERM));
        assert_eq!(guard.open_first_setup(), Ok(false));
        assert_eq!(guard.check(&other, true), Err(EPERM));
        Ok(())
    }

    #[test]
    fn a_spent_legacy_marker_alone_or_a_damaged_list_is_not_legacy_evidence() -> Result<()> {
        let mut list = KVec::new();
        list.extend_from_slice(&setup_consumed(), GFP_KERNEL)?;
        let bytes = RefCell::new(list);
        let guard = Guard::new(1, KVec::new(), Ok(mem(&bytes)), Setup::PinOne);
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Unknown);
        drop(guard);
        bytes
            .borrow_mut()
            .extend_from_slice(&encode_pin(&session(1)), GFP_KERNEL)?;
        bytes.borrow_mut().push(0xab, GFP_KERNEL)?;
        let guard = Guard::new(1, KVec::new(), Ok(mem(&bytes)), Setup::PinOne);
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Unknown);
        Ok(())
    }

    #[test]
    fn a_one_locker_board_never_honours_an_over_budget_legacy_set() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        {
            let mut old = Guard::new(
                1,
                keys(&[session(1), session(2)])?,
                Ok(mem(&bytes)),
                Setup::PinJ313,
            );
            assert_eq!(old.open_first_setup(), Ok(true));
            for i in 1..=2 {
                old.note_read(&session(i));
                assert_eq!(old.check(&session(i), true), Ok(()));
            }
        }
        let mut guard = Guard::new(1, KVec::new(), Ok(mem(&bytes)), Setup::PinOne);
        assert!(!guard.pins_enabled());
        for i in 1..=2 {
            guard.note_read(&session(i));
            assert_eq!(guard.check(&session(i), true), Err(EPERM));
            assert_eq!(guard.check(&session(i), false), Err(EPERM));
        }
        Ok(())
    }

    #[test]
    fn only_a_pinning_guard_remembers_reads() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let locker = session(1);
        // A warm-attach Mac's window never asks, so nothing is kept.
        let mut guard = Guard::with_file(1, keys(&[locker])?, Ok(mem(&bytes)));
        guard.note_read(&locker);
        assert!(guard.read.is_empty());
        let mut guard = guard.with_setup(Setup::PinOne);
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

    #[test]
    fn matching_needs_a_committed_bag_with_the_recorded_uuid() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        {
            let mut guard = Guard::new(1, KVec::new(), Ok(mem(&bytes)), Setup::PinOne);
            assert_eq!(guard.open_first_setup(), Ok(true));
            // A failed or interrupted create leaves only a spent window.
            assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Unknown);
            assert_eq!(guard.commit_keybag(&BAG), Ok(false));
            guard.end_copy();
            assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Unknown);
            assert_eq!(guard.commit_keybag(&BAG), Ok(true));
        }
        let guard = Guard::new(1, KVec::new(), Ok(mem(&bytes)), Setup::PinOne);
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Committed);
        // A later unguarded bag does not inherit the earlier bag's evidence.
        assert_eq!(guard.keybag_evidence(&[0x99; 16]), KeybagEvidence::Unknown);
        Ok(())
    }

    #[test]
    fn a_failed_keybag_completion_cannot_arm_activation() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let faults = Faults::default();
        let mut guard = Guard::new(
            1,
            keys(&[session(1), session(2), session(3)])?,
            Ok(faulty(&bytes, &faults)),
            Setup::PinJ313,
        );
        assert_eq!(guard.open_first_setup(), Ok(true));
        for i in 1..=2 {
            guard.note_read(&session(i));
            assert_eq!(guard.check(&session(i), true), Ok(()));
        }
        guard.end_copy();
        faults.write.set(true);
        assert_eq!(guard.commit_keybag(&BAG), Err(EIO));
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Unknown);
        guard.begin_activation(&BAG);
        guard.note_read(&session(3));
        assert_eq!(guard.check(&session(3), true), Err(EPERM));
        drop(guard);
        let guard = Guard::new(1, KVec::new(), Ok(mem(&bytes)), Setup::PinJ313);
        assert_eq!(guard.keybag_evidence(&BAG), KeybagEvidence::Unknown);
        Ok(())
    }

    #[test]
    fn activation_only_accepts_the_same_bag_in_the_provisioning_boot() -> Result<()> {
        let bytes = RefCell::new(KVec::new());
        let mut guard = Guard::new(
            1,
            keys(&[session(1), session(2), session(3)])?,
            Ok(mem(&bytes)),
            Setup::PinJ313,
        );
        assert_eq!(guard.open_first_setup(), Ok(true));
        for i in 1..=2 {
            guard.note_read(&session(i));
            assert_eq!(guard.check(&session(i), true), Ok(()));
        }
        guard.end_copy();
        assert_eq!(guard.commit_keybag(&BAG), Ok(true));
        guard.note_read(&session(3));
        guard.begin_activation(&[0x99; 16]);
        assert_eq!(guard.check(&session(3), true), Err(EPERM));
        guard.begin_activation(&BAG);
        assert_eq!(guard.check(&session(3), true), Err(EPERM));
        drop(guard);
        let mut guard = Guard::new(1, keys(&[session(3)])?, Ok(mem(&bytes)), Setup::PinJ313);
        guard.note_read(&session(3));
        guard.begin_activation(&BAG);
        assert_eq!(guard.check(&session(3), true), Err(EPERM));
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

        // On where Linux shares the enclave with macOS.
        assert_eq!(level(-1, profile::T8103.xart_guard), Some(1));
        assert_eq!(level(-1, profile::T6000.xart_guard), Some(1));
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
}
