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

/// Whether the first keybag creation of a fresh install is in progress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FirstSetup {
    /// No first keybag creation is in progress.
    Closed,
    /// The enclave may rewrite one existing SEP-xART locker.
    Open,
    /// It has; further refusals stand.
    Used,
}

/// Whether a request the guard would refuse passes as the enclave committing a
/// pending SEP-xART locker during the first keybag creation: a rewrite (not a
/// delete) of a kind-4 record at level 1, while the window is open.
pub(crate) fn first_setup_allows(level: u8, window: FirstSetup, key: &Key, write: bool) -> bool {
    level == 1 && window == FirstSetup::Open && write && key.kind == 4
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
    /// The platform cold-boots its enclave.
    pub(crate) cold: bool,
    /// The guard's effective level and whether its list loaded, or `None`
    /// when no guard runs.
    pub(crate) guard: Option<(u8, bool)>,
    /// The guard's refusals since probe.
    pub(crate) refusals: u32,
}

impl MatchFacts {
    /// A match rewrites session records that existed before it (seen on a
    /// T6000). It is blocked when xART is read-only, once the guard has
    /// refused anything, when the guard has no list to allow Linux's records
    /// by, and on a cold-boot Mac: creating the keybag there needs the guard
    /// off, so the records a match rewrites were never listed.
    ///
    /// Nothing else proves an unguarded setup. An empty list in particular
    /// does not: a guarded keybag creation on an M2 writes no record, and a
    /// rollback can empty the list. So a guarded warm-attach Mac is
    /// unverified until a refusal says otherwise.
    pub(crate) fn matching(&self) -> Matching {
        match self.guard {
            _ if !self.xart_writable || self.refusals != 0 => Matching::Blocked,
            None => Matching::Usable,
            Some((_, false)) => Matching::Blocked,
            Some(_) if self.cold => Matching::Blocked,
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

/// Decodes one list entry; `None` for a torn or foreign one.
pub(crate) fn decode(entry: &[u8]) -> Option<Key> {
    if entry.len() != ENTRY_SIZE || &entry[..4] != ENTRY_MAGIC || entry[5..8] != [0; 3] {
        return None;
    }
    let crc = u32::from_le_bytes([entry[24], entry[25], entry[26], entry[27]]);
    if crc != crc32_ieee(&entry[..24]) || !(3..=4).contains(&entry[4]) {
        return None;
    }
    let mut uuid = [0u8; 16];
    uuid.copy_from_slice(&entry[8..24]);
    Some(Key::new(entry[4], uuid))
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
}

impl Guard {
    /// Opens Linux's list in the host file and records which keys `existing`
    /// holds now. Never fails: a list that cannot be read protects every
    /// existing record.
    pub(crate) fn open(level: u8, existing: KVec<Key>) -> Guard {
        Guard::with_file(
            level,
            existing,
            StoreFile::open_host(OWNED_PATH).map(|(file, _created)| file),
        )
    }
}

impl<F: ListFile> Guard<F> {
    /// As [`Guard::open`], with the list in `file`.
    pub(crate) fn with_file(level: u8, existing: KVec<Key>, file: Result<F>) -> Guard<F> {
        let mut guard = Guard {
            level,
            file: None,
            end: 0,
            owned: KVec::new(),
            existed: existing,
            refused: 0,
            first_setup: FirstSetup::Closed,
        };
        // A list created this boot (or by a crashed one) is only safe to adopt
        // into once its name is durable: fsync() of the file alone does not
        // guarantee that. If the directory cannot be flushed, the file is
        // dropped and nothing is adopted, as without a list.
        let file = file.and_then(|file| file.sync_parent().map(|()| file));
        match file {
            Ok(file) => {
                if let Err(e) = guard.load(&file) {
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
        guard
    }

    fn load(&mut self, file: &F) -> Result<()> {
        let size = file.size()?;
        let whole = (size as usize / ENTRY_SIZE).min(MAX_ENTRIES + 1);
        if whole > MAX_ENTRIES {
            return Err(EFBIG);
        }
        let mut torn = 0usize;
        let mut entry = [0u8; ENTRY_SIZE];
        for i in 0..whole {
            file.read_exact((i * ENTRY_SIZE) as u64, &mut entry)?;
            match decode(&entry) {
                Some(key) if !self.owned.contains(&key) => self.owned.push(key, GFP_KERNEL)?,
                Some(_) => {}
                None if entry == setup_consumed() => {}
                None => torn += 1,
            }
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

    /// Opens the first-setup window for one keybag creation, only at level 1
    /// on a fresh install: the list loaded and holding no entry, torn ones
    /// included. The window is recorded as consumed in the list, durably,
    /// before it opens, so it opens at most once per install whatever becomes
    /// of the keybag. `Ok(false)` when the install is not eligible; an error,
    /// with the window shut, when the record could not be made durable.
    pub(crate) fn open_first_setup(&mut self) -> Result<bool> {
        if self.level != 1 {
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
        self.first_setup = FirstSetup::Open;
        Ok(true)
    }

    /// Closes the first-setup window, used or not.
    pub(crate) fn close_first_setup(&mut self) {
        self.first_setup = FirstSetup::Closed;
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
        match decide(self.level, key, existed, owned) {
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
            Verdict::Refuse if first_setup_allows(self.level, self.first_setup, key, write) => {
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

    fn session(byte: u8) -> Key {
        Key::new(4, [byte; 16])
    }

    fn facts(
        xart_writable: bool,
        cold: bool,
        guard: Option<(u8, bool)>,
        refusals: u32,
    ) -> Matching {
        MatchFacts {
            xart_writable,
            cold,
            guard,
            refusals,
        }
        .matching()
    }

    #[test]
    fn matching_without_a_guard_is_usable() {
        // The Neo (warm, level 0) and an M1 Pro pinned to xart_guard=0 (cold).
        assert_eq!(facts(true, false, None, 0), Matching::Usable);
        assert_eq!(facts(true, true, None, 0), Matching::Usable);
    }

    #[test]
    fn matching_under_a_listed_warm_guard_is_unverified() {
        // An M2 set up under the guard. An empty list is still "loaded", and
        // does not make the setup unguarded.
        assert_eq!(facts(true, false, Some((1, true)), 0), Matching::Unverified);
        assert_eq!(facts(true, false, Some((2, true)), 0), Matching::Unverified);
    }

    #[test]
    fn matching_is_blocked_where_it_is_known_to_be() {
        // A cold-boot M1-family Mac under the default level 1.
        assert_eq!(facts(true, true, Some((1, true)), 0), Matching::Blocked);
        // No list: nothing that existed at probe is Linux's.
        assert_eq!(facts(true, false, Some((1, false)), 0), Matching::Blocked);
        // A refusal seen.
        assert_eq!(facts(true, false, Some((1, true)), 1), Matching::Blocked);
        // xART read-only: no guard runs, and no write can land.
        assert_eq!(facts(false, false, None, 0), Matching::Blocked);
        assert_eq!(facts(false, true, None, 0), Matching::Blocked);
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
        assert_eq!(level(-1, profile::T6030.xart_guard), Some(1));
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
