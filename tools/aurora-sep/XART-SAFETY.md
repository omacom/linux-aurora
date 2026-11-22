# Shared xART gigalocker safety gate

## SEP writes are always on

xART writes and identity-keybag provisioning are always on. The `apple_sep`
driver has no parameter that turns them off: `xart_writes` and
`provision_keybag` were removed (aurora-silicon/linux#2591), and a stale value
on the kernel command line or in modprobe.d is ignored. The APFS
extent-ownership checks, record validation and write-ordering checks remain
mandatory, and unsupported layouts fail closed. Provisioning creates the Linux
identity keybag if it is missing and keeps an existing one without replacing
it.

The historical evidence below records the earlier opt-in deployments and
does not establish qualification on every supported Mac or APFS layout.

Kernels from before linux#2591 still have both parameters. They default to 1
there, but a `0` is still honoured. So remove any `xart_writes=0` or
`provision_keybag=0` that an earlier read-only install or an old version of
these docs left in `/etc/modprobe.d/` or on the kernel command line.
`aurora-xart status` names each one.

## Other OSes' records: `xart_guard`

The gigalocker is shared by every OS on the Mac, and nothing in a session
record says which OS wrote it. With `xart_guard=1` the driver lists every
session record the enclave creates while Linux runs, in
`/var/lib/aurora-sep-xart-owned.bin`, before the record is written. It then
refuses to rewrite or delete any session record that was already in the
gigalocker and is not on that list. `xart_guard=2` also refuses writes to the
two shared root records. A refused write fails the enclave's request the way
a read-only gigalocker does: Touch ID may then be unavailable, but no other
OS's record changes.

The default, `xart_guard=-1`, takes the platform's level:

| Platform | Default |
| --- | --- |
| M1 (T8103), M1 Pro/Max (T6000/T6001) | 0 |
| M2 (T8112), M2 Pro/Max (T6020/T6021) | 1 |
| M3 (T8122) | 1 |
| MacBook Neo (T8140) | 0 |

M1 Macs are at 0 because they cold-boot the SEP, and creating Linux's
identity keybag there rewrites a session record that predates Linux (see
"Cold-boot SEP Macs rewrite a pre-Linux record"). A guard could only block
Touch ID setup on them. The Neo stays at 0 because it already holds Linux
records from before the list existed, and a guarded boot would treat those
as another OS's.

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

- On a Mac whose Touch ID was set up with `xart_guard=0`, or before the
  guard existed, none of Linux's records are on the list. A guarded boot restores the
  fingerprints and then refuses the match's write ("xART guard: refused to
  rewrite a kind-4 record this Linux install did not create"), and the
  verify fails. Such a Mac needs `xart_guard=0` to use Touch ID, not only to
  set it up.
- On a warm-attach Mac set up under the guard, whether every record a match
  rewrites is on the list has not been tested.

### Setting up Touch ID: creating the keybag

Creating Linux's identity keybag (CREATE_KEYBAG, `sks.rs`) happens on a
first setup and after `install-aurora-sep.sh --reset-touchid`. Enrolling
another finger into an existing keybag does not create one.

| Platform | Bootstrap | Keybag creation under level 1 |
| --- | --- | --- |
| M1 (T8103), M1 Pro/Max (T6000/T6001) | cold boot | Guard off by default. Rewrites one pre-Linux session record (T6000). |
| M2 (T8112) | warm attach | No xART writes (J413), so the guard does not affect it. |
| M2 Pro/Max (T6020/T6021) | warm attach | Rewrites one existing kind-4 record (J414s, 2026-11-02); the keybag creation window lets it through. |
| M3 (T8122) | warm attach | Untested. |
| MacBook Neo (T8140) | warm attach | Guard off by default. |

Whether the enclave writes xART records while enrolling a finger has not
been measured on a guarded Mac; a refused write fails the enrolment closed.

### Recovery when the guard refuses

A refusal fails closed: the enclave's request fails, no record changes, and
the kernel log names the refused kind. What to do depends on the case:

- A Mac set up with the guard off: set
  `options apple_sep xart_guard=0` in `/etc/modprobe.d/` and reboot. Do it
  only once the rewrite of the shared, pre-Linux record has been accepted for
  that Mac (see the next section), and treat macOS Touch ID as unverified
  until it has been checked there.
- A warm-attach Mac set up under the guard whose verify is refused: report
  the log line. `xart_guard=0` restores use at the cost of the protection.
- After a boot policy change (a new Stage 1, another boot object), see
  "Boot policy changes" below: one record, one boot, with a backup first.
- A keybag create that never answered ("identity keybag state is
  ambiguous: EEXIST" on every later boot): see "An unfinished keybag
  create" below. A kernel with that rule sends the create again by itself
  when the list proves nothing was created.
- A list that could not be read: every record that existed at boot is
  protected and nothing can be adopted, so new records cannot be written
  either.
- A full list (see "The list of Linux's records"): new records can't be
  adopted, so they cannot be written; existing ownership still applies, and
  the records already on the list stay writable.

Turning the guard back on later does not list the records written while it
was off.

### Boot policy changes

Installing a new Stage 1, changing the boot object, or converting the boot
mode changes the boot policy the Mac keeps for Linux. The first time
Linux's identity keybag is unlocked under the new policy (`CHANGE_LOCK_STATE`
in `refkey.rs`, at the loader's first trusted-key seal), the enclave
rewrites one session record that already existed. Linux did not create it,
so the guard refuses it. The enclave then never answers the unlock, the
machine ref-key stays down, and every trusted-key seal fails with EIO. The
keybag, the ref-key and the enrolled fingerprints are untouched, and since
the write was refused, so is the record.

This was seen on the MacBook Pro M4 Pro (T6040, J616s) on 2026-10-28, after
a Stage 1 reinstall that changed only the kernel collection hash (`coih`).
The same kernel, under the old policy, unlocked the keybag without any
session record write. Linux had never created a record there: the guard
ran from Linux's first boot and its list stayed empty.

Touch ID can use the keybag first instead. When fprintd opens the sensor
before any trusted key is unsealed (a login screen that offers Touch ID,
for example), the first use of the keybag under the new policy is the
`DESIGNATE_KEYBAG` in `activate_touchid` (`sbio.rs`), and the enclave asks
for the rewrite there, after LOAD_KEYBAG has answered. The guard refuses
it, and the designate is never answered:

    xART guard: refused to rewrite kind-4 record <id>, which this Linux install did not create
    sks: DESIGNATE_KEYBAG got no reply after 2037 ms
    sks: DESIGNATE_KEYBAG did not complete; enrolment will refuse
    hwrng: read produced nothing (ETIMEDOUT)
    matching unavailable this boot: ...

Touch ID and the enclave's RNG stay down for that boot, and the refusal
repeats on every boot until the record is allowed. This was seen on the
MacBook Air M3 (T8122, J613) on 2026-11-07, after a change from a raw
m1n1 Stage 1 to a Mach-O one, and on a J616s after every Recovery
enrolment of a Stage 1 that day (https://github.com/aurora-silicon/linux/issues/1190). Kernels from before that
designate ran in the window below need `aurora-sep-policy-change` for this
case.

Under an unchanged policy neither that first unlock nor that designate
rewrites an existing session record (every J616s boot before the change,
and every guarded Touch ID boot on the J613 and J616s). So the driver
treats a rewrite inside them as the policy change and lets exactly one
through, by itself, on the first boot after the change:

- only during the first unlock or Touch ID designate of an identity keybag
  that already existed, whichever comes first, once per boot; a keybag or
  ref-key created this boot closes the window;
- only one kind-4 session record, only a rewrite, never a delete or a
  shared root, and the record is not added to Linux's list;
- only once the record's previous value is durable in
  `/var/lib/aurora-sep-xart-policy-previous.bin` (kind, session id, length,
  CRC, then the value); if it cannot be saved, the write is refused.

The kernel log names the record ("allowed one rewrite of kind-4 record
... for a boot policy change"), and `aurora-sep-load` logs the m1n1, iBoot
and OS firmware versions the boot came up with.

When the driver did not let it through (a kernel from before this, or a
second record), `aurora-sep-policy-change` (root) allows it on the next
boot:

- After the change, with Touch ID down and the refusal in this boot's log,
  run it with no arguments. It takes the session id from the refusal.
- Before a planned change, run it with `--expect`. The record cannot be
  named in advance, so the allowance covers the first existing kind-4
  record the enclave asks to rewrite. The keybag's first use, Touch ID's
  designate or the first unlock, is the first thing that asks. See
  "Changing Stage 1 on purpose" below.
- On a kernel whose refusal line names no session id (one from before the
  allowance code), the tool with no arguments finds nothing to allow. Use
  `--expect` there, and boot a kernel with the allowance code next.

Either way it first copies the whole 6 MiB gigalocker extent, the driver's
host files and `/var/lib/fprint` to
`/var/lib/aurora-rollback/sep-policy-change-<time>/` with `SHA256SUMS`, then
writes `/var/lib/aurora-sep-xart-allow-once.bin`. At the next probe the
driver empties that file before it uses it, so the allowance lasts one boot
whether or not it is used. It covers one rewrite of one existing session
record on the same terms as above, previous value saved first. The kernel
log names the record either way. `--status`
shows what is armed and what this boot refused; `--cancel` removes an
allowance that was not used yet.

#### Changing Stage 1 on purpose

Arm the allowance before the change, so the first boot under the new
policy keeps Touch ID:

1. On the Linux install, run `aurora-sep-policy-change --expect` as root.
   The running kernel only has to load the driver, which logs where the
   gigalocker is; it need not have the allowance code.
2. Change Stage 1 (for example, enrol it from 1TR).
3. Make the first boot afterwards a kernel with the allowance code, the
   one that reads `/var/lib/aurora-sep-xart-allow-once.bin`. It empties
   the file at probe and logs "this boot may rewrite the first existing
   kind-4 record the enclave asks for, once", then "allowed one rewrite of
   kind-4 record ... for a boot policy change" when the enclave asks.
   A kernel without that code does not read the file. With the guard on,
   it refuses the rewrite as above and leaves the allowance for a later
   boot; with the guard off, it lets the rewrite through without saving
   the previous value.
4. Later boots under the same policy need nothing, on either kernel.

If the first boot was the wrong kernel, Touch ID is down for that boot
only: boot one with the allowance code next, and it uses the allowance
still in the file.

`aurora-sep-load` recognises a refusal and stops at once with a message
that points here, instead of retrying for two minutes.

On the J616s the first such update (2026-10-28, Stage 1 window change)
rewrote one kind-4 record and one shared root. The other six kind-4
records and the kind-3 record kept their revisions, Ryan's enrolled
finger matched afterwards, and it kept matching across later reboots.

On the J613 (2026-11-07) the tool with no arguments armed the record the
designate refusal named. The next boot logged "allowed one rewrite of
kind-4 record ... for a boot policy change", restored the Catacomb and the
enrolment ("Touch ID: restored 1 enrolled identity/identities; enrolment
survived reboot"), and a kernel without the allowance code brought Touch
ID up on later boots. A second Stage 1 change on that Mac (2026-11-08),
armed beforehand as in "Changing Stage 1 on purpose", kept Touch ID on
the first boot with no refusal.

The copy of the gigalocker is evidence and a last resort, not an undo:
writing an old copy back does not roll back the enclave's anti-replay
counters, and the enclave may refuse it.

### The keybag creation window

Creating Linux's identity keybag makes the enclave rewrite one existing
kind-4 session record. The record is shared enclave state that the enclave
reads at every attach, under every OS, and it predates every Linux install,
so it is never on Linux's list. Refused, the enclave stops answering the
create, and the key store is unusable for the rest of the boot.

So the driver runs the identity CREATE_KEYBAG exchange in a window
(`keybag_create_window` in `xart_guard.rs`), on the boot policy window's
terms:

- only during that exchange, which runs only when the host holds no keybag;
- one rewrite per boot, shared with the boot policy window, of one kind-4
  record that existed at probe; never a delete or another kind, and the
  roots still follow the level;
- only once the record's previous value is durable in
  `/var/lib/aurora-sep-xart-policy-previous.bin`;
- the record is not added to Linux's list.

The kernel log names the record ("allowed one rewrite of kind-4 record ...
while Linux's identity keybag was created"). Every other existing session
record stays refused, including any record another Linux install made.
Another install's keybag, ref-key and fingerprints are files on its own
root file system.

### An unfinished keybag create

The driver writes an intent to `/var/lib/aurora-sep-keybag.bin` before it
sends CREATE_KEYBAG and replaces it once the enclave answers. If a later
boot finds the intent, a bag may exist that nothing names, so the driver
does not create another ("identity keybag state is ambiguous: EEXIST").

The intent records how many entries Linux's list had. Every new session
record is listed before it is written, so if the list has not grown, the
create left nothing and a later boot sends it again ("creating the identity
keybag again"). An intent from an older kernel records no length, and is
sent again only while the list is empty. That is wrong only for a create
sent with the guard off that did keep a record. It then leaves one unnamed
bag, as `install-aurora-sep.sh --reset-touchid` does.

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
| `unverified` | guard on, list loaded, nothing refused yet | restored, matching UNVERIFIED (not called ready; exit 0) |
| `blocked` | xART writes disabled, a refusal seen, or no list | NOT ready (exit 1), with the reason and the `xart_guard=0` opt-out |
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
  is overwritten and those records stay protected. If Touch ID fails
  afterwards, set it up again with `aurora-xart reset-touchid` (or
  `install-aurora-sep.sh --reset-touchid`).
- A reinstall, or deleting the file, empties the list. Every Linux record
  then looks like another OS's, and those records stay protected. Set Touch
  ID up again with `aurora-xart reset-touchid` (or
  `install-aurora-sep.sh --reset-touchid`), which creates a new keybag.
- A torn last entry, from a crash while appending, is ignored at the next
  probe and overwritten by the next new record.

The file holds only record kinds and session ids, no key material.

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
away from records macOS also reads. The keybag creation window lets that one
rewrite through if the record is kind-4.

Warm-attach Macs differ. On an M2 (T8112, J413), keybag creation made no xART
writes at all, and on the J700 the macOS-era records stayed byte-identical
through Linux enrolment. An M2 Pro (T6020, J414s) rewrites one kind-4 record,
as here.

On a T600x Mac where Linux has created its keybag, treat macOS Touch ID as
unverified until macOS Touch ID unlock and keychain access have been checked
there. The other cold-boot SoCs (T8103, T6001, T6002) probably behave the
same way; that is untested.

## Earlier deployment policy

The iBoot APFS container and its XART `.gl` file are shared with macOS. At
the time, the deployed `m2fix4` kernel was read-only, and the `m2fix5` source
had an explicit `xart_writes=1` opt-in, a parameter since removed
(aurora-silicon/linux#2591). It accepted writes only when the APFS-resolved
extent was single-owner, unsnapshotted, un-cloned, and all existing records
validated, as the driver still requires. `xart_start_sector` remains only an
assertion against the APFS result. That opt-in was for the controlled J414s
hardware test; it was not general APFS write support or a promise that
restoring an old raw image reverses SEP anti-replay state.

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
