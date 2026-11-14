// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

//! Host-side persistence for the key bag.
//!
//! `0x02` copies the bag out enclave-wrapped, `0x03` reloads it. NEVER create a
//! second bag while a stored blob exists — it orphans the previous one in the
//! enclave with no recovery; [`NoStoredKeyBag`] enforces this by type.

use crate::shim;
use crate::store::crc16_ccitt_false;
use kernel::prelude::*;

pub(crate) const KEYBAG_PATH: &CStr = c"/var/lib/aurora-sep-keybag.bin";

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Slot {
    Identity,
}

impl Slot {
    pub(crate) fn path(self) -> &'static CStr {
        match self {
            Slot::Identity => KEYBAG_PATH,
        }
    }
}

const MAGIC: [u8; 16] = *b"APPLE-SEP-KBAG01";
/// v2 stores the bag secret; a v1 record is refused, not reinterpreted.
const VERSION: u32 = 2;

const STATE_COMMITTED: u32 = 1;
/// Create refused by the enclave: nothing exists, retry allowed (unlike an
/// intent record, where a bag may exist unnamed — block retry).
const STATE_REFUSED: u32 = 2;
/// UUID field is the bag's own, read back via `0x06`; state 1 holds the
/// host-generated one, and for an identity bag these differ.
const STATE_COMMITTED_BAG_UUID: u32 = 3;

/// Only an `AsGenerated` UUID may be repaired on a mismatch; adopting one for a
/// read-back record would point it at a different bag and strand the real one.
pub(crate) use crate::keybag_identity::{SnapshotIdentity, UuidProvenance};

impl UuidProvenance {
    fn state(self) -> u32 {
        match self {
            Self::AsGenerated => STATE_COMMITTED,
            Self::ReadBackFromBag => STATE_COMMITTED_BAG_UUID,
        }
    }

    fn from_state(state: u32) -> Option<Self> {
        match state {
            STATE_COMMITTED => Some(Self::AsGenerated),
            STATE_COMMITTED_BAG_UUID => Some(Self::ReadBackFromBag),
            _ => None,
        }
    }
}

const OFF_VERSION: usize = 0x10;
const OFF_STATE: usize = 0x14;
const OFF_LEN: usize = 0x18;
const OFF_CRC: usize = 0x1c;
const OFF_UUID: usize = 0x20;
const OFF_SECRET_LEN: usize = 0x30;
const HEADER: usize = 0x34;

static_assert!(OFF_VERSION == MAGIC.len());
static_assert!(OFF_STATE == OFF_VERSION + 4);
static_assert!(OFF_LEN == OFF_STATE + 4);
static_assert!(OFF_CRC == OFF_LEN + 4);
static_assert!(OFF_UUID == OFF_CRC + 4);
static_assert!(OFF_SECRET_LEN == OFF_UUID + UUID_LEN);
static_assert!(HEADER == OFF_SECRET_LEN + 4);

pub(crate) const UUID_LEN: usize = 16;

const MAX_WRAPPED: usize = crate::store::MAX_VALUE;

/// Proof the host holds no bag; create takes one by value, so it is unreachable
/// without it and unrepeatable with it.
pub(crate) struct NoStoredKeyBag {
    slot: Slot,
}

impl NoStoredKeyBag {
    pub(crate) fn slot(&self) -> Slot {
        self.slot
    }
}

pub(crate) struct StoredKeyBag {
    wrapped: KVec<u8>,
    uuid: [u8; UUID_LEN],
    secret: crate::Secret,
    provenance: UuidProvenance,
}

impl StoredKeyBag {
    pub(crate) fn wrapped(&self) -> &[u8] {
        &self.wrapped
    }

    pub(crate) fn uuid(&self) -> &[u8; UUID_LEN] {
        &self.uuid
    }

    pub(crate) fn uuid_provenance(&self) -> UuidProvenance {
        self.provenance
    }

    pub(crate) fn secret(&self) -> &[u8] {
        &self.secret
    }
}

pub(crate) enum State {
    Present(StoredKeyBag),
    Absent(NoStoredKeyBag),
}

/// CRC preimage; this byte order is the on-disk contract — changing it breaks
/// records written by an earlier build.
fn checksum_input(
    state: u32,
    wrapped: &[u8],
    uuid: &[u8; UUID_LEN],
    secret: &[u8],
) -> Result<crate::Secret> {
    let mut v = KVec::new();
    v.extend_from_slice(&VERSION.to_le_bytes(), GFP_KERNEL)?;
    v.extend_from_slice(&state.to_le_bytes(), GFP_KERNEL)?;
    v.extend_from_slice(&(wrapped.len() as u32).to_le_bytes(), GFP_KERNEL)?;
    v.extend_from_slice(&(secret.len() as u32).to_le_bytes(), GFP_KERNEL)?;
    v.extend_from_slice(uuid, GFP_KERNEL)?;
    v.extend_from_slice(wrapped, GFP_KERNEL)?;
    v.extend_from_slice(secret, GFP_KERNEL)?;
    Ok(crate::Secret(v))
}

/// Absent only when the file is missing; every other unreadable state errors —
/// "cannot tell" must never read as "nothing here", which would orphan a bag.
pub(crate) fn read(slot: Slot) -> Result<State> {
    let file = match shim::StoreFile::open_readonly(slot.path()) {
        Ok(f) => f,
        Err(e) if e == ENOENT => {
            return Ok(State::Absent(NoStoredKeyBag { slot }));
        }
        Err(e) => return Err(e),
    };

    let size = file.size()?;
    if size < HEADER as u64 {
        return Err(EINVAL);
    }

    let mut head = [0u8; HEADER];
    file.read_exact(0, &mut head)?;
    if head[..MAGIC.len()] != MAGIC {
        return Err(EINVAL);
    }
    let version = u32::from_le_bytes([
        head[OFF_VERSION],
        head[OFF_VERSION + 1],
        head[OFF_VERSION + 2],
        head[OFF_VERSION + 3],
    ]);
    if version != VERSION {
        return Err(ENOTSYNC);
    }
    let state = u32::from_le_bytes([
        head[OFF_STATE],
        head[OFF_STATE + 1],
        head[OFF_STATE + 2],
        head[OFF_STATE + 3],
    ]);
    if state == STATE_REFUSED {
        return Ok(State::Absent(NoStoredKeyBag { slot }));
    }
    let Some(provenance) = UuidProvenance::from_state(state) else {
        // Intent or unknown state: a bag may exist unnamed; creating again would
        // orphan it permanently.
        return Err(EEXIST);
    };

    let len = u32::from_le_bytes([
        head[OFF_LEN],
        head[OFF_LEN + 1],
        head[OFF_LEN + 2],
        head[OFF_LEN + 3],
    ]) as usize;
    if len == 0 || len > MAX_WRAPPED || size < (HEADER + len) as u64 {
        return Err(EINVAL);
    }
    let crc = u16::from_le_bytes([head[OFF_CRC], head[OFF_CRC + 1]]);

    let mut uuid = [0u8; UUID_LEN];
    uuid.copy_from_slice(&head[OFF_UUID..OFF_UUID + UUID_LEN]);

    let secret_len = u32::from_le_bytes([
        head[OFF_SECRET_LEN],
        head[OFF_SECRET_LEN + 1],
        head[OFF_SECRET_LEN + 2],
        head[OFF_SECRET_LEN + 3],
    ]) as usize;
    if secret_len > MAX_WRAPPED || size < (HEADER + len + secret_len) as u64 {
        return Err(EINVAL);
    }

    let mut wrapped = KVec::with_capacity(len, GFP_KERNEL)?;
    wrapped.resize(len, 0, GFP_KERNEL)?;
    file.read_exact(HEADER as u64, &mut wrapped)?;

    let mut secret_bytes = KVec::with_capacity(secret_len, GFP_KERNEL)?;
    secret_bytes.resize(secret_len, 0, GFP_KERNEL)?;
    if secret_len > 0 {
        file.read_exact((HEADER + len) as u64, &mut secret_bytes)?;
    }

    if size != (HEADER + len + secret_len) as u64 {
        return Err(EINVAL);
    }
    let secret = crate::Secret(secret_bytes);

    if crc16_ccitt_false(&checksum_input(state, &wrapped, &uuid, &secret)?) != crc {
        return Err(EINVAL);
    }

    Ok(State::Present(StoredKeyBag {
        wrapped,
        uuid,
        secret,
        provenance,
    }))
}

pub(crate) fn write_intent(slot: Slot) -> Result<()> {
    write_record(slot, 0, &[], &[0; UUID_LEN], &[])
}

/// Removes a header-only intent or refused record. A committed bag, or any
/// file that is not exactly that 52-byte header, is left in place: unlinking
/// it would hide a bag the enclave may still hold.
pub(crate) fn discard_uncommitted(slot: Slot) -> Result<()> {
    let file = match shim::StoreFile::open_readonly(slot.path()) {
        Ok(file) => file,
        Err(e) if e == ENOENT => return Ok(()),
        Err(e) => return Err(e),
    };
    let size = file.size()?;
    if size != HEADER as u64 {
        return Err(EEXIST);
    }
    let mut head = [0u8; HEADER];
    file.read_exact(0, &mut head)?;
    if head[..MAGIC.len()] != MAGIC {
        return Err(EINVAL);
    }
    let state = u32::from_le_bytes([
        head[OFF_STATE],
        head[OFF_STATE + 1],
        head[OFF_STATE + 2],
        head[OFF_STATE + 3],
    ]);
    let len = u32::from_le_bytes([
        head[OFF_LEN],
        head[OFF_LEN + 1],
        head[OFF_LEN + 2],
        head[OFF_LEN + 3],
    ]);
    let secret_len = u32::from_le_bytes([
        head[OFF_SECRET_LEN],
        head[OFF_SECRET_LEN + 1],
        head[OFF_SECRET_LEN + 2],
        head[OFF_SECRET_LEN + 3],
    ]);
    if (state != 0 && state != STATE_REFUSED) || len != 0 || secret_len != 0 {
        return Err(EEXIST);
    }
    drop(file);
    match shim::unlink(slot.path()) {
        Ok(()) => Ok(()),
        Err(e) if e == ENOENT => Ok(()),
        Err(e) => Err(e),
    }
}

pub(crate) fn write_bag_uuid(
    slot: Slot,
    wrapped: &[u8],
    uuid: &[u8; UUID_LEN],
    secret: &[u8],
) -> Result<()> {
    if wrapped.is_empty() || wrapped.len() > MAX_WRAPPED || secret.len() > MAX_WRAPPED {
        return Err(EINVAL);
    }
    write_record(
        slot,
        UuidProvenance::ReadBackFromBag.state(),
        wrapped,
        uuid,
        secret,
    )
}

/// Commits a bag named by the host-generated UUID it was created with: the
/// 13.5 enclave keeps an identity bag itself and loads it by that UUID.
pub(crate) fn write_generated_uuid(
    slot: Slot,
    wrapped: &[u8],
    uuid: &[u8; UUID_LEN],
    secret: &[u8],
) -> Result<()> {
    if wrapped.is_empty() || wrapped.len() > MAX_WRAPPED || secret.len() > MAX_WRAPPED {
        return Err(EINVAL);
    }
    write_record(
        slot,
        UuidProvenance::AsGenerated.state(),
        wrapped,
        uuid,
        secret,
    )
}

/// Replaces only the wrapped blob. The enclave ratchets bag material while a bag
/// is active, so this snapshot must be re-taken at the catacomb-save commit or
/// the enclave answers a later restore empty.
pub(crate) fn replace_wrapped(
    slot: Slot,
    fresh: &[u8],
    snapshot_uuid: &[u8; UUID_LEN],
    identity: &SnapshotIdentity,
) -> Result<()> {
    if fresh.is_empty() || fresh.len() > MAX_WRAPPED {
        return Err(EINVAL);
    }
    let stored = match read(slot)? {
        State::Present(stored) => stored,
        State::Absent(_) => return Err(ENOENT),
    };
    if !identity.accepts(stored.uuid(), stored.uuid_provenance(), snapshot_uuid) {
        // Reject a different bag or changed host identity. The generated
        // Sepos13 lookup UUID is not the UUID returned by COPY_KEYBAG_UUID.
        return Err(EPERM);
    }
    write_record(
        slot,
        stored.uuid_provenance().state(),
        fresh,
        stored.uuid(),
        stored.secret(),
    )
}

fn write_record(
    slot: Slot,
    state: u32,
    wrapped: &[u8],
    uuid: &[u8; UUID_LEN],
    secret: &[u8],
) -> Result<()> {
    let crc = crc16_ccitt_false(&checksum_input(state, wrapped, uuid, secret)?);

    let mut head = [0u8; HEADER];
    head[..MAGIC.len()].copy_from_slice(&MAGIC);
    head[OFF_VERSION..OFF_VERSION + 4].copy_from_slice(&VERSION.to_le_bytes());
    head[OFF_STATE..OFF_STATE + 4].copy_from_slice(&state.to_le_bytes());
    head[OFF_LEN..OFF_LEN + 4].copy_from_slice(&(wrapped.len() as u32).to_le_bytes());
    head[OFF_CRC..OFF_CRC + 2].copy_from_slice(&crc.to_le_bytes());
    head[OFF_UUID..OFF_UUID + UUID_LEN].copy_from_slice(uuid);
    head[OFF_SECRET_LEN..OFF_SECRET_LEN + 4].copy_from_slice(&(secret.len() as u32).to_le_bytes());

    let file = shim::StoreFile::open_trunc(slot.path())?;
    // Body first, then the header that vouches for it: a crash between leaves a
    // bad checksum, which `read` refuses — the safe side.
    if !wrapped.is_empty() {
        file.write_all(HEADER as u64, wrapped)?;
    }
    if !secret.is_empty() {
        file.write_all((HEADER + wrapped.len()) as u64, secret)?;
    }
    file.write_all(0, &head)?;
    file.sync()
}
