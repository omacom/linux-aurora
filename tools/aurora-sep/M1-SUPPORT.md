# M1 SEP firmware, enrollment and recovery

The T8103 profile implements the SEP request shapes exercised on a J313
MacBook Air booting the macOS 13.5 (22G74) firmware. This firmware needs the
key-store capability/environment initialization, v2 identity creation, UUID
lookup, v0 designation, and `DEVICE_STATE_TRANSITION` unlock. The profile is
selected by hardware today; other firmware combinations need validation.

## Required fixes

* Initialize SBIO communication with opcode `0x73` and `u32(1)` before ordinary
  SBIO commands. Sensor setup and SCRD authentication could succeed without
  this, while `BEGIN_ENROL` still failed.
* Use the sensor's DT SPI mode and `enable-gpios`. Keep the hardware CS timing
  callback confined to the Mesa bus: assigning it to every SPI controller
  broke M1 keyboard/trackpad setup.
* Begin 13.5 sensor registration with the serial operation. The newer `0x80`
  registration was rejected by the tested firmware.
* Keep the generated identity lookup UUID distinct from the recovered bag's
  UUID. Bind snapshots to the recovered and designated bag, preserving the
  stored lookup UUID and rejecting changes to that binding.
* Export the owner Catacomb in a fresh context, then save the completed user
  before the master. The tested T8103/T6020 profiles select this automatically;
  other profiles retain the existing opt-in control.
* Treat restored Catacombs as provisional until COMPLETE_INIT and the device
  view checks finish. Requiring the final ready bit before setting that same
  bit prevented cold-boot matching.
* Generate host match tokens and trusted-key plaintext with the initialized
  Linux CSPRNG. The tested J313 SEP control entropy source returned zeros,
  which made libfprint reject kernel matches. The hardware RNG survey now
  rejects an all-zero sample; this is not a complete firmware RNG fix.
* Use the 13.5 reference-key envelope, EC type and designated identity session
  for sealing/unsealing. Existing reference-key files are retained on failure.

The libfprint patch also checks SEP's live count before IDENTIFY. An explicitly
valid zero count can answer a stale host gallery as no-match and permit fresh
enrollment. An unavailable count is not zero, and VERIFY keeps its normal
match/token checks. Capture guidance does not count as enrollment progress.

## Existing installations

Keep the existing keybag, ref-key, Catacombs and fprintd records backed up
root-only. Never copy them between machines, restore old raw xART snapshots,
or delete a keybag to repair a fingerprint error. A host-listed finger is not
proof that SEP restored its template; inspect the live count and restore logs.

One test installation had replaced its identity bag during earlier experiments
while retaining a reference key from the former bag. The old reference key
could still encrypt using its public half, but SEP rejected unseal with `-7`.
A separate `refkey_v2=1` opt-in selects
`/var/lib/aurora-sep-refkey-v2.bin`, leaving the original ref-key file intact.
It is a recovery slot for an explicitly selected new context, not an automatic
migration or a way to recover old sealed data. Ordinary installations should
keep the default slot. Always verify new/load and cold-boot reload before
using sealed keys for persistent data. The per-seal round-trip diagnostic is
not included in the production path.

`j414s_persistent_enrol=1` remains available for explicitly testing the owner
export/save order on other profiles. T8103 and T6020 no longer need it.
Raw xART writes are always on (aurora-silicon/linux#2591) and remain subject
to the extent ownership checks in [XART-SAFETY.md](XART-SAFETY.md). The kernel
includes no raw private tracing or experimental dummy-keybag/enrollment
protocol switches from the investigation.

## Validation boundaries

The deployed J313 source completed enrollment and same-boot matching reported
by the tester. After reboot, all three Catacombs loaded, SEP listed one live
identity, and the final device-view proof enabled matching. New reference-key
seal/unseal and a saved blob's cold-boot unseal passed. SPI keyboard/trackpad
drivers bound after the CS timing fix. A physical fingerprint unlock through
the configured desktop PAM service has not been observed remotely.

The consolidated branch preserves the later Neo deferred SMC supplier and
capture-guidance fixes that the older J313 test checkout did not contain.
Build/test results for this consolidation are not a new hardware regression
test on Neo, M2, or every M1 board. The consolidated SEP module compiled against
the prepared J313 kernel build. Its four identity-binding tests, ten extracted
entropy-method tests, and eight calibration/xART tests passed. The libfprint
patch (now maintained in aurora-silicon/aurora-sep-userspace) applied to clean libfprint 1.94.100 and built with `drivers=aurora`;
the three core unit-test suites passed. In that minimal build, 33 unrelated
driver/hwdb tests were skipped and AppStream validation rejected the generated
empty `<provides>` device list. No full-kernel rebuild or new hardware run of
the exact consolidated tree is claimed. Reference-key attestation/signing is
not covered by the sealing/unsealing results.

Source-only identity tests can be run with:

```sh
rustc --edition=2021 --test drivers/soc/apple/keybag_identity.rs -o /tmp/sep-identity-tests
/tmp/sep-identity-tests
python3 -m unittest discover -s tools/aurora-sep/tests -v
```

Desktop authentication setup is installation-specific. The tested Air runs
Chonkstep with Omarchy Quickshell. That shell uses separate password and
fingerprint PAM services; configuring an unused hyprlock file has no effect.
The machine's GRUB entries, automatic SEP loader and PAM edits are deployment
configuration and are not installed implicitly by this kernel branch.
