// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

//! Linux-only state. SEP anti-replay records live in `xart_store`.

use crate::shim;
use kernel::prelude::*;

pub(crate) const STORE_PATH: &CStr = c"/var/lib/aurora-sep-host-state.bin";

const BLOCK_SIZE: usize = 0x8000;
const BLOCK_COUNT: usize = 72;
pub(crate) const STORE_SIZE: usize = BLOCK_SIZE * BLOCK_COUNT;
const SLOT_COUNT: usize = BLOCK_COUNT - 1;
pub(crate) const MAX_VALUE: usize = BLOCK_SIZE;

const MAGIC: [u8; 16] = *b"AURORA-SEP-STOR\x01";
const VERSION: u32 = 1;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct Key {
    pub(crate) kind: u8,
    pub(crate) uuid: [u8; 16],
}

impl Key {
    pub(crate) const fn root(kind: u8) -> Key {
        Key {
            kind,
            uuid: [0u8; 16],
        }
    }
}

#[derive(Clone, Copy)]
struct Slot {
    used: bool,
    kind: u8,
    uuid: [u8; 16],
    len: u32,
}

impl Slot {
    const FREE: Slot = Slot {
        used: false,
        kind: 0,
        uuid: [0; 16],
        len: 0,
    };

    fn matches(&self, key: &Key) -> bool {
        self.used && self.kind == key.kind && self.uuid == key.uuid
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Intent {
    None,
    Write { slot: u16, kind: u8 },
}

const SB_MAGIC: usize = 0;
const SB_VERSION: usize = 16;
const SB_SLOT_COUNT: usize = 20;
const SB_BLOCK_SIZE: usize = 24;
const SB_GENERATION: usize = 28;
const SB_INTENT_KIND: usize = 36;
const SB_INTENT_SLOT: usize = 37;
const SB_INTENT_TYPE: usize = 39;
const SB_RESERVED: usize = 40;
const SB_SLOT_TABLE: usize = 64;
const SLOT_ENTRY_SIZE: usize = 24;
const SB_SLOT_TABLE_END: usize = SB_SLOT_TABLE + SLOT_COUNT * SLOT_ENTRY_SIZE;

/// The superblock is rewritten in place, so the format relies on the device
/// writing at least this much of it atomically: a torn write of the slot table
/// would mix entries of two tables. Every field a reader uses lies within it.
/// NVMe storage on Apple silicon has 4 KiB logical blocks.
const ATOMIC_WRITE: usize = 4096;
static_assert!(SB_SLOT_TABLE_END <= ATOMIC_WRITE);

const INTENT_NONE: u8 = 0;
const INTENT_WRITE: u8 = 1;

fn valid_kind(kind: u8) -> bool {
    (0xf0..=0xf5).contains(&kind)
}

/// Why an existing store is refused. It is logged, so it never carries record
/// contents.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Damage {
    /// The file is not exactly [`STORE_SIZE`] bytes long.
    Size,
    /// The magic, version or geometry is not this format's.
    Header,
    /// A byte outside the header fields and the slot table is not zero.
    Reserved,
    /// The slot entry is neither all zero (free) nor a valid record.
    Slot(usize),
    /// The slot repeats the key of an earlier slot.
    Duplicate(usize),
    /// The pending intent is not a write into a free slot.
    Intent,
}

/// Backing I/O of a [`Store`], separate so that the format handling can be
/// tested without a filesystem.
pub(crate) trait StoreIo {
    fn size(&self) -> Result<u64>;
    fn read_exact(&self, off: u64, buf: &mut [u8]) -> Result;
    fn write_all(&self, off: u64, buf: &[u8]) -> Result;
    fn sync(&self) -> Result;
}

impl StoreIo for shim::StoreFile {
    fn size(&self) -> Result<u64> {
        shim::StoreFile::size(self)
    }

    fn read_exact(&self, off: u64, buf: &mut [u8]) -> Result {
        shim::StoreFile::read_exact(self, off, buf)
    }

    fn write_all(&self, off: u64, buf: &[u8]) -> Result {
        shim::StoreFile::write_all(self, off, buf)
    }

    fn sync(&self) -> Result {
        shim::StoreFile::sync(self)
    }
}

// Not internally locked; the caller holds a mutex around all access.
pub(crate) struct Store<F = shim::StoreFile> {
    file: F,
    slots: [Slot; SLOT_COUNT],
    generation: u64,
}

fn le32(buf: &[u8], off: usize) -> u32 {
    u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

fn le64(buf: &[u8], off: usize) -> u64 {
    let mut v = [0u8; 8];
    v.copy_from_slice(&buf[off..off + 8]);
    u64::from_le_bytes(v)
}

/// Describes an existing file that holds no store as "empty" or "all zero".
/// That is all an interrupted creation leaves, and such a file holds nothing
/// to keep; anything else may be what is left of a store. Reads at most a
/// store's worth of the file.
fn describe<F: StoreIo>(file: &F, size: u64) -> Option<&'static str> {
    if size == 0 {
        return Some("empty");
    }
    if size > STORE_SIZE as u64 {
        return None;
    }
    let mut buf = KVec::new();
    buf.resize(BLOCK_SIZE, 0, GFP_KERNEL).ok()?;
    let mut off = 0;
    while off < size {
        let len = (size - off).min(BLOCK_SIZE as u64) as usize;
        file.read_exact(off, &mut buf[..len]).ok()?;
        if buf[..len].iter().any(|&b| b != 0) {
            return None;
        }
        off += len as u64;
    }
    Some("all zero")
}

/// Decodes the slot table of the superblock `sb` into `slots`.
///
/// Accepts exactly what this format's writer produces: the version 1 header,
/// zero in every byte it does not use, free slots that are all zero, used slots
/// with a known kind, a length that fits the slot's block and a key no other
/// slot has, and either no intent or a write intent naming a free slot.
fn decode(sb: &[u8], slots: &mut [Slot; SLOT_COUNT]) -> Result<(), Damage> {
    if sb[SB_MAGIC..SB_MAGIC + 16] != MAGIC
        || le32(sb, SB_VERSION) != VERSION
        || le32(sb, SB_SLOT_COUNT) as usize != SLOT_COUNT
        || le32(sb, SB_BLOCK_SIZE) as usize != BLOCK_SIZE
    {
        return Err(Damage::Header);
    }
    if sb[SB_RESERVED..SB_SLOT_TABLE].iter().any(|&b| b != 0)
        || sb[SB_SLOT_TABLE_END..BLOCK_SIZE].iter().any(|&b| b != 0)
    {
        return Err(Damage::Reserved);
    }

    for i in 0..SLOT_COUNT {
        let off = SB_SLOT_TABLE + i * SLOT_ENTRY_SIZE;
        let entry = &sb[off..off + SLOT_ENTRY_SIZE];
        if entry.iter().all(|&b| b == 0) {
            slots[i] = Slot::FREE;
            continue;
        }
        let len = le32(entry, 18);
        if entry[0] != 1
            || !valid_kind(entry[1])
            || len as usize > MAX_VALUE
            || entry[22..].iter().any(|&b| b != 0)
        {
            return Err(Damage::Slot(i));
        }
        let mut uuid = [0u8; 16];
        uuid.copy_from_slice(&entry[2..18]);
        let key = Key {
            kind: entry[1],
            uuid,
        };
        if slots[..i].iter().any(|slot| slot.matches(&key)) {
            return Err(Damage::Duplicate(i));
        }
        slots[i] = Slot {
            used: true,
            kind: key.kind,
            uuid,
            len,
        };
    }

    let slot = u16::from_le_bytes([sb[SB_INTENT_SLOT], sb[SB_INTENT_SLOT + 1]]) as usize;
    let intent_valid = match sb[SB_INTENT_KIND] {
        INTENT_NONE => slot == 0 && sb[SB_INTENT_TYPE] == 0,
        // The intent is written along with the committed table, in which the
        // block it names is still free.
        INTENT_WRITE => slot < SLOT_COUNT && !slots[slot].used && valid_kind(sb[SB_INTENT_TYPE]),
        _ => false,
    };
    if !intent_valid {
        return Err(Damage::Intent);
    }
    Ok(())
}

impl Store {
    pub(crate) fn open() -> Result<Self> {
        let (file, created) = shim::StoreFile::open_host(STORE_PATH)?;
        Self::from_file(file, created)
    }
}

impl<F: StoreIo> Store<F> {
    fn from_file(file: F, created: bool) -> Result<Self> {
        let mut store = Store {
            file,
            slots: [Slot::FREE; SLOT_COUNT],
            generation: 0,
        };

        // New files and interrupted empty or all-zero creations hold no state.
        // Any other existing file may hold the only copy of the host state,
        // so one that does not validate is refused and left as it is.
        let size = store.file.size()?;
        if created && size == 0 {
            store.create()?;
            pr_info!(
                "created a new host-state store at /var/lib/aurora-sep-host-state.bin ({} bytes)\n",
                STORE_SIZE
            );
            return Ok(store);
        }

        let mut sb = KVec::with_capacity(BLOCK_SIZE, GFP_KERNEL)?;
        sb.resize(BLOCK_SIZE, 0, GFP_KERNEL)?;
        let decoded = if size == STORE_SIZE as u64 {
            store.file.read_exact(0, &mut sb)?;
            decode(&sb, &mut store.slots)
        } else {
            Err(Damage::Size)
        };
        if let Err(damage) = decoded {
            if matches!(damage, Damage::Size | Damage::Header) {
                if let Some(what) = describe(&store.file, size) {
                    pr_warn!(
                        "host-state store file is {} and holds no state; initialising it\n",
                        what
                    );
                    store.create()?;
                    return Ok(store);
                }
            }
            return Err(store.refuse(damage));
        }

        // Writes are copy-on-write, so an interrupted one is discarded, never
        // completed. Its intent names a free block and the next write replaces
        // it, so opening writes nothing.
        store.generation = le64(&sb, SB_GENERATION);
        Ok(store)
    }

    /// Logs why the existing file is refused, and returns the error. Nothing
    /// is written to the file.
    fn refuse(&self, damage: Damage) -> Error {
        pr_err!(
            "host-state store failed validation ({:?}), refusing it\n",
            damage
        );
        EIO
    }

    fn create(&mut self) -> Result<()> {
        self.slots = [Slot::FREE; SLOT_COUNT];
        self.generation = 1;

        let mut zero = KVec::with_capacity(BLOCK_SIZE, GFP_KERNEL)?;
        zero.resize(BLOCK_SIZE, 0, GFP_KERNEL)?;
        for b in 0..BLOCK_COUNT {
            self.file.write_all((b * BLOCK_SIZE) as u64, &zero)?;
        }

        self.commit()?;
        Ok(())
    }

    fn commit(&mut self) -> Result<()> {
        self.write_superblock(Intent::None)?;
        self.file.sync()
    }

    fn write_superblock(&mut self, intent: Intent) -> Result<()> {
        let mut sb = KVec::with_capacity(BLOCK_SIZE, GFP_KERNEL)?;
        sb.resize(BLOCK_SIZE, 0, GFP_KERNEL)?;

        sb[SB_MAGIC..SB_MAGIC + 16].copy_from_slice(&MAGIC);
        sb[SB_VERSION..SB_VERSION + 4].copy_from_slice(&VERSION.to_le_bytes());
        sb[SB_SLOT_COUNT..SB_SLOT_COUNT + 4].copy_from_slice(&(SLOT_COUNT as u32).to_le_bytes());
        sb[SB_BLOCK_SIZE..SB_BLOCK_SIZE + 4].copy_from_slice(&(BLOCK_SIZE as u32).to_le_bytes());
        sb[SB_GENERATION..SB_GENERATION + 8].copy_from_slice(&self.generation.to_le_bytes());

        let (ikind, islot, itype) = match intent {
            Intent::None => (INTENT_NONE, 0u16, 0u8),
            Intent::Write { slot, kind } => (INTENT_WRITE, slot, kind),
        };
        sb[SB_INTENT_KIND] = ikind;
        sb[SB_INTENT_SLOT..SB_INTENT_SLOT + 2].copy_from_slice(&islot.to_le_bytes());
        sb[SB_INTENT_TYPE] = itype;

        for (i, slot) in self.slots.iter().enumerate() {
            let off = SB_SLOT_TABLE + i * SLOT_ENTRY_SIZE;
            sb[off] = u8::from(slot.used);
            sb[off + 1] = slot.kind;
            sb[off + 2..off + 18].copy_from_slice(&slot.uuid);
            sb[off + 18..off + 22].copy_from_slice(&slot.len.to_le_bytes());
        }

        self.file.write_all(0, &sb)
    }

    fn find(&self, key: &Key) -> Option<usize> {
        self.slots.iter().position(|s| s.matches(key))
    }

    fn find_free(&self) -> Option<usize> {
        self.slots.iter().position(|s| !s.used)
    }

    fn block_offset(slot: usize) -> u64 {
        ((slot + 1) * BLOCK_SIZE) as u64
    }

    pub(crate) fn read(&mut self, key: &Key) -> Result<Option<KVec<u8>>> {
        if !valid_kind(key.kind) {
            return Err(EINVAL);
        }
        let Some(idx) = self.find(key) else {
            return Ok(None);
        };
        let len = self.slots[idx].len as usize;
        let mut value = KVec::with_capacity(len, GFP_KERNEL)?;
        value.resize(len, 0, GFP_KERNEL)?;
        if len > 0 {
            self.file.read_exact(Self::block_offset(idx), &mut value)?;
        }
        Ok(Some(value))
    }

    pub(crate) fn write(&mut self, key: &Key, value: &[u8]) -> Result<()> {
        if !valid_kind(key.kind) {
            return Err(EINVAL);
        }
        if value.len() > MAX_VALUE {
            return Err(ENOSPC);
        }

        let old = self.find(key);
        let fresh = self.find_free().ok_or(ENOSPC)?;

        // 1. Record the intent, durably, before touching any data block.
        self.write_superblock(Intent::Write {
            slot: fresh as u16,
            kind: key.kind,
        })?;
        self.file.sync()?;

        // 2. Fill the new block and make it durable.
        if !value.is_empty() {
            self.file.write_all(Self::block_offset(fresh), value)?;
        }
        self.file.sync()?;

        // 3. Switch the slot table, release the old block, clear the intent.
        self.slots[fresh] = Slot {
            used: true,
            kind: key.kind,
            uuid: key.uuid,
            len: value.len() as u32,
        };
        if let Some(old) = old {
            self.slots[old] = Slot::FREE;
        }
        self.generation = self.generation.wrapping_add(1);
        self.commit()
    }
}

pub(crate) const fn crc16_ccitt_false(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    let mut i = 0;
    while i < data.len() {
        crc ^= (data[i] as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
            bit += 1;
        }
        i += 1;
    }
    crc
}

// Canonical CCITT-FALSE check value; guards against a reflected/XOR variant.
static_assert!(crc16_ccitt_false(b"123456789") == 0x29B1);

// The driver as it shipped before this validation, changed only to reach its
// file through `StoreIo`. The tests use it as the reference for compatibility
// in both directions.
#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[allow(dead_code)]
#[path = "store_shipped.rs"]
mod shipped;

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[::kernel::macros::kunit_tests(apple_sep_host_store)]
mod tests {
    use super::*;
    use core::cell::{Cell, RefCell};

    /// A backing file in memory. A power cut can be armed on a write: that
    /// write keeps only a prefix of its data, and every later call fails as if
    /// the machine had stopped.
    struct MemoryFile {
        bytes: RefCell<VVec<u8>>,
        writes: Cell<usize>,
        cut: Cell<Option<(usize, usize)>>,
        dead: Cell<bool>,
    }

    impl MemoryFile {
        fn new() -> Self {
            Self {
                bytes: RefCell::new(VVec::new()),
                writes: Cell::new(0),
                cut: Cell::new(None),
                dead: Cell::new(false),
            }
        }

        fn copy(&self) -> Result<Self> {
            let file = Self::new();
            file.bytes
                .borrow_mut()
                .extend_from_slice(&self.bytes.borrow(), GFP_KERNEL)?;
            Ok(file)
        }

        /// Cuts the power during the `nth` write from now, after `keep` bytes.
        fn cut(&self, nth: usize, keep: usize) {
            self.cut.set(Some((self.writes.get() + nth, keep)));
        }

        fn restore_power(&self) {
            self.cut.set(None);
            self.dead.set(false);
        }

        fn poke(&self, off: usize, data: &[u8]) {
            self.bytes.borrow_mut()[off..off + data.len()].copy_from_slice(data);
        }

        fn same(&self, other: &Self) -> bool {
            self.bytes.borrow()[..] == other.bytes.borrow()[..]
        }
    }

    impl StoreIo for &MemoryFile {
        fn size(&self) -> Result<u64> {
            if self.dead.get() {
                return Err(EIO);
            }
            Ok(self.bytes.borrow().len() as u64)
        }

        fn read_exact(&self, off: u64, buf: &mut [u8]) -> Result {
            if self.dead.get() {
                return Err(EIO);
            }
            let bytes = self.bytes.borrow();
            let start = off as usize;
            buf.copy_from_slice(bytes.get(start..start + buf.len()).ok_or(EIO)?);
            Ok(())
        }

        fn write_all(&self, off: u64, buf: &[u8]) -> Result {
            if self.dead.get() {
                return Err(EIO);
            }
            let nth = self.writes.get();
            self.writes.set(nth + 1);
            let keep = match self.cut.get() {
                Some((at, keep)) if at == nth => {
                    self.dead.set(true);
                    keep.min(buf.len())
                }
                _ => buf.len(),
            };
            let start = off as usize;
            let mut bytes = self.bytes.borrow_mut();
            if bytes.len() < start + keep {
                bytes.resize(start + keep, 0, GFP_KERNEL)?;
            }
            bytes[start..start + keep].copy_from_slice(&buf[..keep]);
            if self.dead.get() {
                Err(EIO)
            } else {
                Ok(())
            }
        }

        fn sync(&self) -> Result {
            if self.dead.get() {
                Err(EIO)
            } else {
                Ok(())
            }
        }
    }

    /// Offset of `byte` within the table entry of `slot`.
    const fn entry(slot: usize, byte: usize) -> usize {
        SB_SLOT_TABLE + slot * SLOT_ENTRY_SIZE + byte
    }

    /// A store this driver created and then wrote `records` to.
    fn written(records: &[(Key, &[u8])]) -> Result<MemoryFile> {
        let file = MemoryFile::new();
        let mut store = Store::from_file(&file, true)?;
        for (key, value) in records {
            store.write(key, value)?;
        }
        Ok(file)
    }

    /// Opens an existing store, checking that opening wrote nothing.
    fn reopen(file: &MemoryFile) -> Result<Store<&MemoryFile>> {
        let writes = file.writes.get();
        let store = Store::from_file(file, false)?;
        if file.writes.get() != writes {
            return Err(EINVAL);
        }
        Ok(store)
    }

    /// Whether opening `file` fails and leaves it exactly as it was.
    fn refused(file: &MemoryFile) -> Result<bool> {
        let before = file.copy()?;
        let writes = file.writes.get();
        Ok(Store::from_file(file, false).is_err()
            && file.writes.get() == writes
            && file.same(&before))
    }

    fn damage(file: &MemoryFile) -> Option<Damage> {
        let mut slots = [Slot::FREE; SLOT_COUNT];
        decode(&file.bytes.borrow()[..BLOCK_SIZE], &mut slots).err()
    }

    /// Whether a store holding two records, patched with `data` at `off`, is
    /// refused because of `slot`.
    fn refuses_slot(off: usize, data: &[u8], slot: usize) -> Result<bool> {
        let file = written(&[(Key::root(0xf1), b"index"), (Key::root(0xf5), b"")])?;
        file.poke(off, data);
        Ok(damage(&file) == Some(Damage::Slot(slot)) && refused(&file)?)
    }

    /// Whether a store holding one record is refused with the given intent.
    fn refuses_intent(kind: u8, slot: u16, record_kind: u8) -> Result<bool> {
        let file = written(&[(Key::root(0xf1), b"index")])?;
        file.poke(SB_INTENT_KIND, &[kind]);
        file.poke(SB_INTENT_SLOT, &slot.to_le_bytes());
        file.poke(SB_INTENT_TYPE, &[record_kind]);
        Ok(damage(&file) == Some(Damage::Intent) && refused(&file)?)
    }

    #[test]
    fn new_file_is_initialised() -> Result {
        let file = written(&[])?;
        assert_eq!(file.bytes.borrow().len(), STORE_SIZE);
        assert!(damage(&file).is_none());
        assert!(reopen(&file)?.read(&Key::root(0xf1))?.is_none());
        Ok(())
    }

    #[test]
    fn existing_empty_and_zero_files_are_initialised() -> Result {
        for len in [0, 1, BLOCK_SIZE, STORE_SIZE - 1, STORE_SIZE] {
            let file = MemoryFile::new();
            file.bytes.borrow_mut().resize(len, 0, GFP_KERNEL)?;
            let store = Store::from_file(&file, false)?;
            assert!(store.slots.iter().all(|slot| !slot.used));
            assert_eq!((&file).size()?, STORE_SIZE as u64);
            assert!(damage(&file).is_none());
            assert!(reopen(&file)?.read(&Key::root(0xf1))?.is_none());
        }
        Ok(())
    }

    #[test]
    fn a_created_flag_keeps_valid_records_and_retries_blank_files() -> Result {
        // Should a file that was there be reported as created, a store in it
        // is still opened in place, and anything else refused.
        let key = Key::root(0xf1);
        let file = written(&[(key, b"index")])?;
        let writes = file.writes.get();
        let mut store = Store::from_file(&file, true)?;
        assert!(store.read(&key)?.as_deref() == Some(&b"index"[..]));
        assert_eq!(file.writes.get(), writes);

        let file = MemoryFile::new();
        file.bytes.borrow_mut().resize(BLOCK_SIZE, 0, GFP_KERNEL)?;
        assert!(Store::from_file(&file, true).is_ok());
        assert!(file.writes.get() > 0);
        assert_eq!(file.bytes.borrow().len(), STORE_SIZE);
        Ok(())
    }

    /// How a refusal of `file` describes it.
    fn described(file: &MemoryFile) -> Result<Option<&'static str>> {
        Ok(describe(&file, file.size()?))
    }

    #[test]
    fn interrupted_creation_is_retried() -> Result {
        // Creation writes every block and then the superblock. Until the
        // superblock is on disk the file is not a store, and holds nothing.
        for nth in [0, 1, BLOCK_COUNT - 1, BLOCK_COUNT] {
            for keep in [0, BLOCK_SIZE / 2] {
                let file = MemoryFile::new();
                file.cut(nth, keep);
                assert!(Store::from_file(&file, true).is_err());
                file.restore_power();
                if nth == BLOCK_COUNT && keep > 0 {
                    assert!(reopen(&file).is_ok());
                } else {
                    assert!(described(&file)?.is_some());
                    assert!(Store::from_file(&file, false).is_ok());
                    assert!(damage(&file).is_none());
                }
            }
        }
        Ok(())
    }

    #[test]
    fn nonblank_short_files_and_bad_headers_are_refused_untouched() -> Result {
        for len in [100, STORE_SIZE] {
            let file = MemoryFile::new();
            file.bytes.borrow_mut().resize(len, 0x5a, GFP_KERNEL)?;
            assert!(refused(&file)?);
        }
        Ok(())
    }

    #[test]
    fn blank_creation_is_distinguished_from_damaged_state() -> Result {
        for (len, what) in [
            (0, Some("empty")),
            (1, Some("all zero")),
            (BLOCK_SIZE + 1, Some("all zero")),
            (STORE_SIZE, Some("all zero")),
            // Creation never writes past the store.
            (STORE_SIZE + 1, None),
        ] {
            let file = MemoryFile::new();
            file.bytes.borrow_mut().resize(len, 0, GFP_KERNEL)?;
            assert!(described(&file)? == what);
            if what.is_some() {
                assert!(Store::from_file(&file, false).is_ok());
            } else {
                assert!(refused(&file)?);
            }
        }

        // A truncated store, and a single byte of data, may be what is left of
        // the host state.
        let file = written(&[(Key::root(0xf1), b"index")])?;
        file.bytes.borrow_mut().truncate(2 * BLOCK_SIZE);
        assert!(refused(&file)?);
        assert!(described(&file)?.is_none());
        let file = MemoryFile::new();
        file.bytes.borrow_mut().resize(STORE_SIZE, 0, GFP_KERNEL)?;
        file.poke(STORE_SIZE - 1, &[1]);
        assert!(refused(&file)?);
        assert!(described(&file)?.is_none());
        Ok(())
    }

    #[test]
    fn foreign_headers_are_refused() -> Result {
        let file = written(&[])?;
        file.poke(SB_MAGIC, b"X");
        assert_eq!(damage(&file), Some(Damage::Header));
        assert!(refused(&file)?);
        // Other versions, including 2, and other geometries.
        for (off, value) in [
            (SB_VERSION, 0),
            (SB_VERSION, 2),
            (SB_SLOT_COUNT, SLOT_COUNT as u32 - 1),
            (SB_BLOCK_SIZE, BLOCK_SIZE as u32 / 2),
        ] {
            let file = written(&[])?;
            file.poke(off, &value.to_le_bytes());
            assert_eq!(damage(&file), Some(Damage::Header));
            assert!(refused(&file)?);
        }
        Ok(())
    }

    #[test]
    fn unused_bytes_must_be_zero() -> Result {
        for off in [
            SB_RESERVED,
            SB_SLOT_TABLE - 1,
            SB_SLOT_TABLE_END,
            BLOCK_SIZE - 1,
        ] {
            let file = written(&[])?;
            file.poke(off, &[1]);
            assert_eq!(damage(&file), Some(Damage::Reserved));
            assert!(refused(&file)?);
        }
        Ok(())
    }

    #[test]
    fn slots_must_be_records_or_zero() -> Result {
        // Slot 0 holds 0xf1, slot 1 holds an empty 0xf5 and slot 2 is free.
        assert!(refuses_slot(entry(2, 1), &[0xf0], 2)?);
        assert!(refuses_slot(entry(2, 18), &[1], 2)?);
        assert!(refuses_slot(entry(0, 0), &[2], 0)?);
        assert!(refuses_slot(entry(0, 1), &[0xef], 0)?);
        assert!(refuses_slot(entry(0, 1), &[0xf6], 0)?);
        assert!(refuses_slot(entry(1, 22), &[1], 1)?);
        // A length beyond the slot's block would be read from the next ones.
        let over = MAX_VALUE as u32 + 1;
        assert!(refuses_slot(entry(1, 18), &over.to_le_bytes(), 1)?);
        assert!(refuses_slot(entry(1, 18), &u32::MAX.to_le_bytes(), 1)?);
        Ok(())
    }

    #[test]
    fn keys_must_be_unique() -> Result {
        let other = Key {
            kind: 0xf1,
            uuid: [7; 16],
        };
        let file = written(&[(Key::root(0xf1), b"a"), (other, b"b")])?;
        // The same kind under another UUID is another key.
        assert!(damage(&file).is_none());
        file.poke(entry(1, 2), &[0; 16]);
        assert_eq!(damage(&file), Some(Damage::Duplicate(1)));
        assert!(refused(&file)?);
        Ok(())
    }

    #[test]
    fn intents_must_name_a_free_block() -> Result {
        // Slot 0 holds the record and slot 1 is free.
        assert!(refuses_intent(INTENT_WRITE, 0, 0xf2)?);
        assert!(refuses_intent(INTENT_WRITE, SLOT_COUNT as u16, 0xf2)?);
        assert!(refuses_intent(INTENT_WRITE, 1, 0xef)?);
        assert!(refuses_intent(2, 1, 0xf2)?);
        assert!(refuses_intent(INTENT_NONE, 1, 0)?);
        assert!(refuses_intent(INTENT_NONE, 0, 0xf2)?);
        assert!(!refuses_intent(INTENT_WRITE, 1, 0xf2)?);
        Ok(())
    }

    #[test]
    fn records_are_never_discarded_or_overread() -> Result {
        // An intent naming a committed record's block would discard the
        // record, and a length beyond its block would be read from the next.
        let file = written(&[(Key::root(0xf1), b"index")])?;
        file.poke(SB_INTENT_KIND, &[INTENT_WRITE]);
        file.poke(SB_INTENT_TYPE, &[0xf1]);
        assert!(refused(&file)?);
        let file = written(&[(Key::root(0xf1), b"index")])?;
        file.poke(entry(0, 18), &(MAX_VALUE as u32 + 1).to_le_bytes());
        assert!(refused(&file)?);
        Ok(())
    }

    #[test]
    fn interrupted_write_is_discarded_without_writing() -> Result {
        let key = Key::root(0xf1);
        let file = written(&[(key, b"old")])?;
        // The intent and the new block reach the disk, the new table does not.
        file.cut(2, 0);
        assert!(reopen(&file)?.write(&key, b"new").is_err());
        file.restore_power();

        // The intent is still pending, and opening leaves it there.
        assert_eq!(file.bytes.borrow()[SB_INTENT_KIND], INTENT_WRITE);
        let mut store = reopen(&file)?;
        assert!(store.read(&key)?.as_deref() == Some(&b"old"[..]));
        store.write(&key, b"newer")?;
        assert_eq!(file.bytes.borrow()[SB_INTENT_KIND], INTENT_NONE);
        assert!(reopen(&file)?.read(&key)?.as_deref() == Some(&b"newer"[..]));
        Ok(())
    }

    #[test]
    fn a_full_block_is_the_largest_record() -> Result {
        let mut full = KVec::new();
        full.resize(MAX_VALUE, 0x5a, GFP_KERNEL)?;
        let key = Key::root(0xf2);
        let file = written(&[(key, &full)])?;
        let mut store = reopen(&file)?;
        assert!(store.read(&key)?.as_deref() == Some(&full[..]));
        full.push(0, GFP_KERNEL)?;
        assert!(store.write(&key, &full).is_err());

        // The last slot's block ends where the file does.
        file.poke(entry(SLOT_COUNT - 1, 0), &[1, 0xf3]);
        file.poke(entry(SLOT_COUNT - 1, 18), &(MAX_VALUE as u32).to_le_bytes());
        let value = reopen(&file)?.read(&Key::root(0xf3))?;
        assert_eq!(value.map(|v| v.len()), Some(MAX_VALUE));
        Ok(())
    }

    /// Opens `file` with the shipped driver.
    fn open_shipped(file: &MemoryFile) -> Result<shipped::Store<&MemoryFile>> {
        shipped::Store::open(file)
    }

    /// `key` as the shipped driver spells it.
    fn as_shipped(key: &Key) -> shipped::Key {
        shipped::Key {
            kind: key.kind,
            uuid: key.uuid,
        }
    }

    /// A record value whose every byte differs from those of other seeds.
    fn value(len: usize, seed: u8) -> Result<KVec<u8>> {
        let mut v = KVec::with_capacity(len, GFP_KERNEL)?;
        for i in 0..len {
            v.push(seed.wrapping_add(i as u8).wrapping_mul(31), GFP_KERNEL)?;
        }
        Ok(v)
    }

    /// Every kind, a non-root key, an empty value, full blocks and overwrites.
    fn history() -> Result<KVec<(Key, KVec<u8>)>> {
        let mut ops = KVec::new();
        let other = Key {
            kind: 0xf0,
            uuid: [0x5c; 16],
        };
        for (key, len, seed) in [
            (Key::root(0xf1), 300, 1),
            (Key::root(0xf5), 0, 2),
            (Key::root(0xf2), MAX_VALUE, 3),
            (other, 16, 4),
            (Key::root(0xf1), 600, 5),
            (Key::root(0xf3), 1, 6),
            (Key::root(0xf4), MAX_VALUE - 1, 7),
            (Key::root(0xf0), 16, 8),
            (Key::root(0xf5), 12, 9),
        ] {
            ops.push((key, value(len, seed)?), GFP_KERNEL)?;
        }
        Ok(ops)
    }

    /// Whether this driver and the shipped one read the same value for every
    /// key in `ops` from `file`.
    fn both_read(file: &MemoryFile, ops: &[(Key, KVec<u8>)]) -> Result<bool> {
        let mut new = reopen(file)?;
        let mut shipped = open_shipped(file)?;
        if shipped.fresh {
            return Ok(false);
        }
        for (key, _) in ops {
            if new.read(key)?.as_deref() != shipped.read(&as_shipped(key))?.as_deref() {
                return Ok(false);
            }
        }
        Ok(true)
    }

    #[test]
    fn writes_match_the_shipped_driver_byte_for_byte() -> Result {
        let ours = MemoryFile::new();
        let theirs = MemoryFile::new();
        let mut new = Store::from_file(&ours, true)?;
        let mut shipped = open_shipped(&theirs)?;
        assert!(ours.same(&theirs));
        for (key, value) in history()?.iter() {
            new.write(key, value)?;
            shipped.write(&as_shipped(key), value)?;
            assert!(ours.same(&theirs));
        }
        Ok(())
    }

    #[test]
    fn both_drivers_reject_the_same_writes() -> Result {
        let mut long = KVec::new();
        long.resize(MAX_VALUE + 1, 0x5a, GFP_KERNEL)?;
        let ours = written(&[(Key::root(0xf1), b"index")])?;
        let theirs = ours.copy()?;
        let mut new = reopen(&ours)?;
        let mut shipped = open_shipped(&theirs)?;
        for (key, value) in [
            (Key::root(0xef), &b"x"[..]),
            (Key::root(0xf6), &b"x"[..]),
            (Key::root(0xf2), &long[..]),
        ] {
            let ours_failed = new.write(&key, value).err();
            assert!(ours_failed.is_some());
            assert!(ours_failed == shipped.write(&as_shipped(&key), value).err());
        }
        assert!(new.read(&Key::root(0xef)).is_err());
        assert!(shipped.read(&as_shipped(&Key::root(0xef))).is_err());
        assert!(ours.same(&theirs));
        Ok(())
    }

    #[test]
    fn each_driver_opens_the_others_store_in_place() -> Result {
        let ops = history()?;
        let (first, second) = ops.split_at(4);

        // Shipped writer, new reader, then the other way round.
        let file = MemoryFile::new();
        let mut shipped = open_shipped(&file)?;
        for (key, value) in first {
            shipped.write(&as_shipped(key), value)?;
        }
        assert!(both_read(&file, &ops)?);
        let mut new = reopen(&file)?;
        for (key, value) in second {
            new.write(key, value)?;
        }
        assert!(both_read(&file, &ops)?);
        for (key, _) in ops.iter() {
            let stored = open_shipped(&file)?.read(&as_shipped(key))?;
            let last = ops
                .iter()
                .rev()
                .find(|(k, _)| k == key)
                .map(|(_, v)| &v[..]);
            assert!(stored.as_deref() == last);
        }
        Ok(())
    }

    #[test]
    fn torn_writes_open_in_either_driver() -> Result {
        let key = Key::root(0xf2);
        let old = value(MAX_VALUE, 0x11)?;
        let new = value(MAX_VALUE, 0x22)?;
        // A write is an intent superblock, the new block, then the new table.
        // Each is cut off after every multiple of the atomic write size.
        for writer_is_new in [false, true] {
            for nth in 0..3 {
                for keep in (0..=BLOCK_SIZE).step_by(ATOMIC_WRITE) {
                    let file = MemoryFile::new();
                    open_shipped(&file)?.write(&as_shipped(&key), &old)?;
                    open_shipped(&file)?.write(&as_shipped(&Key::root(0xf5)), b"lockout")?;
                    file.cut(nth, keep);
                    let failed = if writer_is_new {
                        reopen(&file)?.write(&key, &new).is_err()
                    } else {
                        open_shipped(&file)?.write(&as_shipped(&key), &new).is_err()
                    };
                    assert!(failed);
                    file.restore_power();

                    // Only a table that reached the disk publishes the value.
                    let expect = if nth == 2 && keep > 0 { &new } else { &old };
                    let crashed = file.copy()?;
                    for new_first in [false, true] {
                        let file = crashed.copy()?;
                        if new_first {
                            assert!(reopen(&file)?.read(&key)?.as_deref() == Some(&expect[..]));
                        }
                        let mut shipped = open_shipped(&file)?;
                        assert!(!shipped.fresh);
                        assert!(shipped.read(&as_shipped(&key))?.as_deref() == Some(&expect[..]));
                        let lockout = reopen(&file)?.read(&Key::root(0xf5))?;
                        assert!(lockout.as_deref() == Some(&b"lockout"[..]));
                        assert!(reopen(&file)?.read(&key)?.as_deref() == Some(&expect[..]));
                    }
                }
            }
        }
        Ok(())
    }

    #[test]
    fn the_shipped_driver_erased_what_this_one_refuses() -> Result {
        let key = Key::root(0xf1);
        let truncated = written(&[(key, b"index")])?;
        truncated
            .bytes
            .borrow_mut()
            .truncate(STORE_SIZE - BLOCK_SIZE);
        let unknown = written(&[(key, b"index")])?;
        unknown.poke(SB_VERSION, &2u32.to_le_bytes());
        for file in [truncated, unknown] {
            assert!(refused(&file)?);
            let mut shipped = open_shipped(&file)?;
            assert!(shipped.fresh);
            assert!(shipped.read(&as_shipped(&key))?.is_none());
        }
        Ok(())
    }
}
