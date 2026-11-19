# Shared xART gigalocker safety gate

## Current default

`xart_writes=1` is the driver default. The APFS extent-ownership checks,
record validation and write-ordering checks remain mandatory; unsupported
layouts fail closed. `xart_writes=0 provision_keybag=0` explicitly selects
read-only operation. `provision_keybag=1` is the default: create the Linux
identity if missing and recover an existing identity without replacing it.
The historical evidence below records the earlier opt-in deployments and
does not establish qualification on every supported Mac or APFS layout.

## Other OSes' records: `xart_guard`

The gigalocker is shared by every OS on the Mac, and nothing in a session
record says which OS wrote it. With `xart_guard=1` the driver lists every
session record the enclave creates while Linux runs, in
`/var/lib/aurora-sep-xart-owned.bin`, before the record is written. It then
refuses to rewrite or delete any session record that was already in the
gigalocker and is not on that list. `xart_guard=2` also refuses writes to the
two shared root records. A refused write fails the enclave's request the way
a read-only gigalocker does: Touch ID may then be unavailable, but no other
OS's record changes. The one exception is a fresh install's first keybag
creation on a warm-attach Mac at level 1, which may rewrite one existing
SEP-xART locker (see "Fresh installs" below). On a cold-boot Mac the owner
can accept up to three more with `xart_guard_pins=1` (run on a J313 only): rewrites of the shared
lockers its first Touch ID setup reaches (see "Cold-boot Macs: pinning the shared
locker" below; three on the MacBook Air M1).

The default, `xart_guard=-1`, takes the platform's level:

| Platform | Default |
| --- | --- |
| M1 (T8103), M1 Pro/Max (T6000/T6001) | 1 |
| M2 (T8112), M2 Pro/Max (T6020/T6021) | 1 |
| M3 (T8122) | 1 |
| MacBook Neo (T8140) | 0 |

The Neo stays at 0 because it already holds Linux records from before the
list existed, and a guarded boot would treat those as another OS's.

What the guard allows depends on what the enclave writes, and that differs
between restoring Touch ID at boot, using it, and setting it up. The driver
does not choose these writes: the enclave asks for them through the xART
service, and every write and delete it asks for passes the guard
(`xarm.rs`, `OP_SESSION_WRITE` and `OP_SESSION_DELETE`, into
`XartStore::write()` and `XartStore::delete()` in `xart_store.rs`, which call
`Guard::check()`). Reads are never refused (`XartStore::read()`).

### Restore at boot

Loading the identity keybag (`sks_recover()`, LOAD_KEYBAG, in `sks.rs`) and
restoring the enrolled fingerprints only read xART. On an M1 Pro (T6000,
J316s) a boot that restored its fingerprint changed no record (ledger diff
of 2026-10-17, R6a kernel, guard off). So "Touch ID: restored N" can appear
under level 1 on every platform.

### Using Touch ID: a fingerprint match

A match does write. On the same M1 Pro, one successful verify rewrote two
session records that existed before it and both shared roots (same ledger,
across the verify). The roots are allowed at level 1. The two session
records are allowed only if they are on Linux's list: records the enclave
created while the guard was on. Which records they are has not been
established, so:

- On a Mac whose Touch ID was set up with `xart_guard=0` (a cold-boot Mac
  set up without `xart_guard_pins`, see below, and any Mac that used Touch ID
  before the guard existed), none of Linux's records are on the list. A guarded boot restores the
  fingerprints and then refuses the match's write ("xART guard: refused to
  rewrite a kind-4 record this Linux install did not create"), and the
  verify fails. Such a Mac needs `xart_guard=0` to use Touch ID, not only to
  set it up.
- On a warm-attach Mac set up under the guard, whether every record a match
  rewrites is on the list has not been tested.
- On a cold-boot Mac set up under the guard with `xart_guard_pins=1`, the
  record the create added is on the list and the shared lockers the first
  setup rewrote are pinned (three on a J313). On that J313, matches before
  and after a reboot had no refusal: every pre-Linux record they rewrote
  was pinned or listed (writes to the shared roots are allowed at level 1
  and not logged). Other boards are untested.

### Setting up Touch ID: creating the keybag

Creating Linux's identity keybag (CREATE_KEYBAG, `sks.rs`) happens on a
first setup and after `install-aurora-sep.sh --reset-touchid`. Enrolling
another finger into an existing keybag does not create one.

| Platform | Bootstrap | Keybag creation under level 1 |
| --- | --- | --- |
| M1 (T8103), M1 Pro/Max (T6000/T6001) | cold boot | Refused: it rewrites pre-Linux session records (seen on T6000; on a T8103 J313, three SEP-xART lockers; T6001 untested). Needs `xart_guard=0`, or `xart_guard_pins=1`, run on a J313 only. |
| M2 (T8112) | warm attach | No xART writes (J413), so the guard does not affect it. |
| M2 Max (T6021, J416c) | warm attach | On a fresh install, commits one pending SEP-xART locker; allowed once (see "Fresh installs"). Otherwise no xART writes. |
| M2 Pro (T6020), M3 (T8122) | warm attach | Untested. |
| MacBook Neo (T8140) | warm attach | Guard off by default. |

Whether the enclave writes xART records while enrolling a finger has not
been measured on a guarded Mac; a refused write fails the enrolment closed.

### Recovery when the guard refuses

A refusal fails closed: the enclave's request fails, no record changes, and
the kernel log names the refused kind. What to do depends on the case:

- A cold-boot Mac, or any Mac set up with the guard off: set
  `options apple_sep xart_guard=0` in `/etc/modprobe.d/` and reboot. Do it
  only once the rewrite of the shared, pre-Linux record has been accepted for
  that Mac (see the next section), and treat macOS Touch ID as unverified
  until it has been checked there.
- A cold-boot Mac that has never tried to create its keybag under the guard:
  `options apple_sep xart_guard_pins=1` keeps the guard on and accepts the
  rewrites of the lockers its first setup reaches, up to three, only (run on
  a J313 only) (see
  "Cold-boot Macs: pinning the shared locker"). The same caution about macOS applies. A Mac that has tried, or
  that already has a keybag, must reset Touch ID and remove
  `/var/lib/aurora-sep-xart-owned.bin` first: the window that pins the
  lockers opens only on an empty list, and a reset alone keeps the list. The
  driver says at probe which of the two it found.
- A warm-attach Mac set up under the guard whose verify is refused: report
  the log line. `xart_guard=0` restores use at the cost of the protection.
- A list that could not be read: every record that existed at boot is
  protected and nothing can be adopted, so new records cannot be written
  either.
- A full list (see "The list of Linux's records"): new records can't be
  adopted, so they cannot be written; existing ownership still applies, and
  the records already on the list stay writable.

Turning the guard back on later does not list the records written while it
was off.

### What `aurora-sep-ready` reports

"Touch ID: restored N" says the fingerprints came back; it does not say a
match can save its state. The driver's `diag` attributes (see
`Documentation/ABI/testing/sysfs-driver-apple-sep`) report the guard's
effective level (`xart_guard`), whether its list loaded
(`xart_guard_list`), how many writes it refused (`xart_guard_refusals`),
and from those `matching`:

| `matching` | When | `aurora-sep-ready` |
| --- | --- | --- |
| `usable` | xART writes enabled, no guard | ready |
| `unverified` | guard with its list loaded and nothing refused yet, on a warm-attach Mac or on a cold-boot Mac whose keybag was created under it with `xart_guard_pins=1` | restored, matching UNVERIFIED (not called ready; exit 0) |
| `blocked` | xART writes disabled, a refusal seen, no list, or a guard on a cold-boot Mac whose keybag was not created under it | NOT ready (exit 1), with the reason and the `xart_guard=0` opt-out |
| absent | a kernel that does not report it | restored, matching UNVERIFIED (exit 0) |

The diag ABI stays at version 2: these attributes were added to it, and a
reader tests for them rather than for a version.

An empty list next to an existing keybag is not taken as a sign that Touch
ID was set up without the guard: a guarded keybag creation on an M2 wrote no
record, and a rollback can empty the list. Such a Mac stays `unverified`
until the guard refuses a write.

### The list of Linux's records

`/var/lib/aurora-sep-xart-owned.bin` holds 32-byte entries, one per session
record the enclave created under the guard. It is read at probe and
appended before each new record is written. The driver refuses to open a
list longer than 4096 entries, counting torn and repeated ones, and stops
appending at the same length, so it never writes a list it would refuse.
With a full list, new records can't be adopted; existing ownership still
applies, so the records already on it stay writable.

The list is a host file, so what happens to the root file system happens to
it:

- A btrfs/snapper rollback, or restoring a backup, brings back an older list.
  Records the enclave created after that snapshot are no longer listed, so
  the guard treats them as another OS's and refuses to rewrite them. Nothing
  is overwritten, but Touch ID may fail until the newer list is put back.
  Keep the newest copy, or leave `/var/lib/aurora-sep-xart-owned.bin` out of
  rollback snapshots.
- A reinstall, or deleting the file, empties the list. Every Linux record
  then looks like another OS's. Copy the file back from the old install if
  you have it. Without it, those records stay protected; resetting Touch ID
  (`install-aurora-sep.sh --reset-touchid`) creates a new keybag, which on a
  cold-boot Mac needs `xart_guard=0` or `xart_guard_pins=1` as above.
- A torn last entry, from a crash while appending, is ignored at the next
  probe and overwritten by the next new record.
- The first entry may be a marker instead of a record: it says this
  install's first-setup window (see "Fresh installs") has been used. It
  lists no record. Resetting Touch ID keeps the list, so the new keybag gets
  no window; a reinstall or deleting the file gives the next create one.
- On a cold-boot Mac set up with `xart_guard_pins=1`, up to three entries
  are pins instead of records: each names a shared SEP-xART locker the
  enclave may rewrite, and lists no record as Linux's. Only the first three
  pins in the list count. A kernel without pinning reports each as an
  unreadable entry and ignores it, so that kernel refuses the lockers as
  before.

The file holds only record kinds and session ids, no key material, and it
can be backed up and copied like any other file.

## Cold-boot SEP Macs rewrite a pre-Linux record (2026-10-16)

On Macs where the driver cold-boots the SEP, creating Linux's identity
keybag reaches beyond Linux's own records. This was observed on an M1 Pro
(T6000, J316s), with the macOS 13.5 sepOS loaded by the driver.

Creating the keybag makes the enclave do three things:

- write one new session record;
- rewrite a shared root record;
- rewrite one session record that existed before Linux ever ran.

The rewritten record is one of the four session records the enclave reads at
every attach, and it grows by 71 bytes per keybag created. It is not tied to
the OS identity, so giving Linux its own identity does not keep this write
away from records macOS also reads. A driver that refuses to rewrite records
Linux did not create cannot create a keybag on these Macs.

Warm-attach Macs behave differently. On an M2 (T8112, J413), keybag creation
made no xART writes at all. On the J700, the macOS-era records stayed
byte-identical through Linux enrolment.

On a T600x Mac where Linux has created its keybag, treat macOS Touch ID as
unverified until macOS Touch ID unlock and keychain access have been checked
there. The other cold-boot SoCs differ in detail: on a T8103 (J313) the
create rewrites no pre-Linux record, the COPY_KEYBAG after it and the first
LOAD_KEYBAG rewrite three (see "MacBook Air M1 (J313)" below). T6001 and
T6002 are untested.

## Cold-boot Macs: pinning the shared locker (`xart_guard_pins`)

### MacBook Air M1 (J313), 2026-11-19: three lockers, after the create

Run on a J313 with macOS 13.5 firmware (sepOS 13.5), `xart_guard_pins=1`, guard level 1, macOS booted after the Linux install, each attempt with no keybag and no list:

- The create does not rewrite a pre-Linux locker on this Mac. The `COPY_KEYBAG` of the new bag rewrites two (tags `da70de89`, `140a890d`), and the first activation's `LOAD_KEYBAG` a third (`5700cc64`). The enclave had read all three since probe.
- All three were in a raw copy of the iBoot System Container taken before Linux's first xART write. An earlier setup on the same Mac with `xart_guard=0` changed five 4 KiB blocks of the gigalocker, which are where those three lockers sat or moved.
- With the window closed after the create, the first was refused and provisioning failed. With it held through provisioning, the second was refused. With two pins, the third was refused at `LOAD_KEYBAG`; after that refusal the enclave answered nothing more that boot (`DEVICE_STATE_TRANSITION` and the ref-key unlock got no reply, sensor registration timed out).
- So the window now runs from the create to the first `activate_touchid()`, and pins up to three lockers. With that: `identity keybag provisioned`, three pins, `xart_guard_refusals` 0, enrol, 3 of 3 matches, then after a reboot `16 session records exist, 1 created by Linux; the other 15 are protected`, the enrolment restored, 2 of 2 matches, refusals 0. The owner reports macOS Touch ID still works afterwards; macOS unlock and keychain were not checked one by one, so treat macOS Touch ID as unverified as below.

Run on that J313 only. The rule was first built from the J316s measurements above, which saw one locker at the create.

Until now a cold-boot Mac had two choices: no Touch ID, or `xart_guard=0`,
which lets the enclave rewrite or delete any session record on the Mac.
The J316s measurement says the create needs far less: one new record, a
shared root, and one existing session record that the enclave reads at
every attach; the J313 needs three such lockers. `xart_guard_pins=1`
(default 0) lets the owner accept exactly those records, if they are
SEP-xART lockers (kind 4), up to three, and keep the guard on for everything
else. It has an effect only on a cold-boot Mac at level 1. Its name,
`xart_guard_pins`, is what `modinfo -p apple_sep` shows for this rule;
the first draft of it (`xart_guard_pin`) pinned one locker at the create
only and did not work on a J313.

- The first-setup window, which cold-boot Macs otherwise never get, opens
  for the first `CREATE_KEYBAG` of an install whose list is loaded and
  empty, and stays open until the first activation's `LOAD_KEYBAG`
  (`activate_touchid`). Any failed step of provisioning or of that
  activation before the load closes it. The marker is written first, as on a
  warm-attach Mac. Without the parameter a cold-boot Mac gets no window.
- In the window the guard passes rewrites of existing kind-4 lockers, up to
  three, and only of lockers the enclave read since probe (the driver notes
  each `OP_SESSION_READ` it answers with a record). Before each write reaches
  the gigalocker, the guard appends a pin entry naming that locker to the
  list and flushes it. If the pin cannot be made durable, the rewrite is
  refused.
- If the first activation does not reach `LOAD_KEYBAG` in the boot that
  created the keybag, a locker it would have pinned cannot be pinned later.
  If nothing opens `/dev/sep-bio` that boot, the window stays open until
  the boot ends; if `/dev/sep-bio` cannot be published, or an early step of
  the activation fails, the window closes then. Either way:
  the window opens once. The guard then logs a reset hint when it refuses a
  read locker on an install with free pins. Recovery is a Touch ID reset with
  `/var/lib/aurora-sep-xart-owned.bin` removed.
- A pin counts only from a list that loaded whole, and a pinned locker is
  never adopted as Linux's, even if it was absent at probe.
- From then on the enclave may rewrite a pinned locker, on any boot where
  the parameter is set and the enclave has read the locker. That covers a
  match, if the locker is one of the records a match rewrites, and should
  cover a later `CREATE_KEYBAG` after a Touch ID reset (untested).
- A pinned locker is never deleted and never listed as Linux's. Every
  other record from before Linux, kind 3 or 4, stays refused. Level 2 gets
  none of this.
- Without the parameter the pin stays in the list and is not honoured, so
  the Mac is back to refusing those lockers.
- The record the create adds is listed as Linux's, as on any guarded Mac,
  so a match's rewrite of it passes. `matching` then reads `unverified`
  rather than `blocked`.

Before the first setup under the pin:

- Boot macOS once after installing, so that macOS's enclave commits any
  SEP-xART locker the install left pending (see "Fresh installs" below).
  The window pins the first existing lockers the setup rewrites. On a Mac
  where one of those is the pending locker, a shared one is refused
  afterwards and the setup fails.
- Start with no list. An earlier guarded attempt leaves entries in it, and
  the window does not open on a list that is not empty.

What the pin does not establish:

- On boards other than the J313, that the records the setup rewrites are
  SEP-xART lockers. The refusals logged on J316s and J314s name kind 4, but
  the ledger above does not give the kind. A kind-3 record is refused in the
  window as anywhere else, and the setup then fails as it does today.
- That macOS is unaffected by Linux's rewrites of that locker. The locker
  is not tied to an OS identity and macOS reads it too. This is the same
  open question as with `xart_guard=0` on these Macs; the pin narrows which
  records Linux can change, not what a change to this one does. Treat macOS
  Touch ID as unverified until unlock and keychain access have been checked
  after a Linux keybag creation and after Linux matches.
- That a match rewrites only listed records and pinned lockers. If it
  rewrites another record from before Linux, the guard refuses it and logs
  the refused locker's tag and whether the enclave had read it, next to the
  pinned lockers' tags in the earlier log lines, so the two can be compared.
- That each pinned locker keeps its session id. If another OS replaces one
  under a new id, its pin names a record that no longer exists and the new
  locker is refused; the tags in the log show the difference. The window
  does not open twice, so recovering means resetting Touch ID with the list
  removed.
- That the pin survives a driver reload. The allowance needs the enclave's
  reads of the locker since probe. If the driver is bound again while the
  enclave keeps running and does not repeat its attach reads, the pinned
  locker is refused until a restart.
- That T6001 and T6002 behave as T6000 or T8103 does.

The tags in those log lines are a CRC-32 of the session id. They identify a
locker across boots in the log without printing its id.

## Fresh installs: the pending SEP-xART locker (2026-10-25)

Measured on an M2 Max (T6021, J416c, system firmware 26.6.2) installed with
the Omarchy Mac installer:

- The installer blesses the new stub from macOS. While it does, the
  enclave saves SEP-xART root/locker/root triples, which leave one SEP-xART
  locker (kind 4) larger than its usual 1137 bytes (2321 bytes right after
  the bless on one run). By Linux's first boot, after macOS shut down and
  recoveryOS finished the install, it held 2715 bytes on both runs.
- Every OS's enclave reads that locker at attach, alongside three others:
  macOS's own xART log shows the same four, under its own OS identity.
- The next enclave to save its state commits the locker back to 1137 bytes.
  macOS's enclave does that at boot. Under Linux the first save comes inside
  the first `CREATE_KEYBAG`: a SEP-xART root write, then the locker rewrite.
  The guard refused the rewrite, `CREATE_KEYBAG` got no reply, and the
  enclave stopped answering until a restart.
- After one macOS boot, keybag creation under the guard made no xART writes.
  Enrolment, match, restore after a restart and a match after it wrote only
  the USER-xART root. macOS Touch ID unlock and the login keychain were
  unaffected.
- Changing the startup disk between macOS and Linux in System Settings did
  not leave the locker pending.
- With the window below, in its first version (before the marker), a
  fresh install (no macOS boot after it) created
  its keybag on Linux's first boot: the enclave committed the locker to
  1137 bytes, as macOS's does, and no record was listed. Enrolment, match,
  `keyctl` seal and load, restore and match after a restart, macOS Touch ID
  unlock and keychain, then restore, `keyctl` load and match back in Linux
  all passed, with no refusal.

So during the first `CREATE_KEYBAG` of an install whose list is loaded and
empty, on a warm-attach Mac at level 1, the guard lets the enclave rewrite
the first existing kind-4 locker it writes (`FirstSetup` in
`xart_guard.rs`). Nothing identifies that locker as the pending one: no
record says which OS or bless it belongs to, and a size test would not prove
it. Deletes, kind-3 lockers and a second rewrite are refused as before.
Level 2 gets no window, and neither do cold-boot Macs unless their owner
set `xart_guard_pins=1`: their first setup rewrites pre-Linux lockers for a
different reason (at the create on a J316s, after it on a J313), and the
window then pins those lockers instead (above).

The window is used once while the install keeps its list. Before the
create is sent, the driver writes a marker as the list's first entry and
flushes it, so the list is no longer empty whatever becomes of the create,
and no later create (another boot, or after a Touch ID reset) gets the
window. Restoring or deleting the list along with the keybag can open it
again. If the marker cannot be written, the create is not sent that boot,
and the driver marks the attempt retryable (if that fails too, later creates
fail with `EEXIST`). The next boot gets the window only if none of the
marker survived.

A create that gets no reply, as one with a refused write does, keeps its
intent record, so later creates fail with `EEXIST` until Touch ID is reset
(`install-aurora-sep.sh --reset-touchid`): the enclave may have kept part of
what it saved before the refusal. A create the enclave answers with an error
stays retryable, as before.

## Earlier deployment policy

The iBoot APFS container and its XART `.gl` file are shared with macOS. The
deployed `m2fix4` kernel remains read-only. The `m2fix5` source has an explicit
`xart_writes=1` opt-in; it accepts writes only when the APFS-resolved extent is
single-owner, unsnapshotted, un-cloned, and all existing records validate.
`xart_start_sector` remains only an assertion against the APFS result. The
opt-in is for the controlled J414s hardware test, not general APFS write
support or a promise that restoring an old raw image reverses SEP anti-replay
state. Do not enable it on other installations without equivalent checks.

## Evidence collected on 2026-09-21

- A read-only inspection of `m2`'s iBoot partition found a complete, valid
  checkpoint at transaction 57, one XART volume (role `0x100`), and one
  unencrypted `.gl` extent: 6 MiB beginning at APFS block 1236. All 11 live
  gigalocker records in that extent passed their payload CRC checks. The raw
  signature heuristic happened to select the same block on this snapshot.
- A synthetic image with the first root moved to slot 10 reproduces a legacy
  locator failure: the highest-scoring window begins 90 APFS blocks after the
  true file start and extends beyond `.gl`. This proves ambiguity in that
  locator, **not** the cause of the previously damaged installation.
- Static inspection of the macOS 27.2 AppleSEPManager kernel collection found
  `gl_rec_write` allocating a fresh 0x9000-byte slot, writing it, then clearing
  0x1000 bytes at the former slot. The low-level write path also calls a disk
  cache-synchronization ioctl. `gl_fixup_rev_init` validates records and
  handles duplicates during open. These observations do not prove that our
  APFS ownership and failure ordering match macOS.

## Before authorizing writes

1. Confirm the exact macOS vnode/raw-device and extent-locking contract,
   including whether any APFS copy-on-write, snapshot, encryption, or remap
   state can invalidate the physical mapping during Linux's lifetime.
2. Verify record ordering, duplicate selection, failed-write recovery,
   barriers/cache synchronization, and revision behavior against both macOS
   code and controlled non-production tests.
3. Compare a pre/post macOS-and-Linux power-cycle trace, with APFS checksums,
   `.gl` mapping and records checked before and after. Keep a recoverable image
   of the iBoot partition before the first write experiment.

The current APFS locator supports only a complete checkpoint, single-node
object maps and file-system tree, and a single unencrypted 6 MiB `.gl` extent.
Unsupported layouts fail closed; this is not a general APFS implementation.

## Read-only J414s recovery image (2026-09-23)

After the minimal stage2 made the SEP mailbox and RNG functional, a fresh
read-only inspection still found one unencrypted 6 MiB `.gl` extent at APFS
block 1236, 11 live records with valid payload CRCs, and no malformed records.
The selected container checkpoint was xid 136. No xART writes or keybag
provisioning were attempted.

The 524,288,000-byte `iBootSystemContainer` partition was copied through a
read-only block handle and compressed to
`work/m2-tb-sep-deploy/iboot-system-container-2026-09-22.img.zst` (7,857,330
bytes, mode 0600). The local archive passed `zstd -t`; its decompressed
SHA-256 matched a separate read of the partition:
`09206fcf0500f5646ea04ec37cf1fc8a0493f1c5ee3271c2af562ed9b39a6d37`.
The compressed archive SHA-256 is
`8a885a2397c26404cf8395ca64b279efc793f9118eae408f2a6d84fe9af1ded8`.
The temporary m2 copy was removed after verification. This is a recovery
artifact, **not** evidence that Linux may safely write APFS-owned `.gl` blocks.

## Narrow write-ownership evidence (2026-09-23)

The live m2 partition SHA-256 still matches the independent backup hash above.
The updated read-only inspector selected checkpoint xid 136 and checked the
current xART volume superblock and checksum-validated extent-reference tree:
volume flags `0x1` (unencrypted only), zero snapshots, zero pending revert
fields, and an inode with no clone flag. The sole `.gl` extent is APFS blocks
1236–2771 (1536 blocks, 6 MiB); its only physical extent-reference record is
kind `APFS_KIND_NEW`, owner 16 (the inode's private ID), reference count 1.
The inspector now rejects missing, overlapping, shared, cloned/snapshotted,
or mismatched references, and four synthetic tests pass. The `m2fix5` kernel
independently checks the same narrow ownership conditions on its block handle
before arming writes and refuses a writable scan with malformed or duplicate
xART records. These checks establish the current physical mapping and lack of
APFS sharing; they do not by themselves establish macOS's full record-update
or SEP anti-replay recovery semantics. The APFS field meanings come from the
[Apple File System Reference](https://developer.apple.com/support/apple-file-system/Apple-File-System-Reference.pdf).
