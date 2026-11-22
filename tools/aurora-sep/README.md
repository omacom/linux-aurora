# Aurora SEP userspace integration

These files connect the Apple SEP kernel driver to the shared APFS xART
gigalocker and the desktop fingerprint stack.

The current release installer and matched packages are published in
[omacom/linux-aurora](https://github.com/omacom/linux-aurora/releases/latest):

```sh
curl -fsSL https://github.com/omacom/linux-aurora/releases/latest/download/install-aurora-sep.sh | bash
```

The installer in this source tree is an assembly template. Use the release
asset above, whose package checksums and GPU profiles are pinned together.

Each release installer downloads its own exact package versions and verifies
their checksums. Experimental hardware profiles require their explicit opt-ins.
Normal stable delivery uses Omacom's package channels and installer images.

For experimental acceleration on the 13-inch or 15-inch M3 MacBook Air, see
the [M3 GPU quick start](M3-GPU.md) for the matched activation command and
desktop GPU check.

For the J413 M2 Air hibernation request, see [HIBERNATION.md](HIBERNATION.md)
for the current platform blockers and required resume qualification.

See [M1-SUPPORT.md](M1-SUPPORT.md) for the T8103 firmware requirements,
reboot-persistent enrollment fixes, reference-key recovery caveats and current
validation boundaries. The historical bring-up results below describe the
individual builds at the time they were tested.

For Neo OS-identity continuity, read
[OS-IDENTITY.md](OS-IDENTITY.md) before changing boot parameters or identity
sources. Existing chosen-lane enrolments require no migration.

Install the driver service:

```sh
install -Dm755 load-driver /usr/local/sbin/aurora-sep-load
install -Dm644 aurora-sep.service /etc/systemd/system/aurora-sep.service
systemctl daemon-reload
systemctl enable aurora-sep.service
```

The loader waits up to 120 s for the enclave to seal and unseal a trusted key,
then up to 60 s for `/dev/sep-bio`, and logs how long each took. On J700 the
enclave's key store arrives around 43 s after boot. To change a wait, set
`AURORA_SEP_READY_TIMEOUT` or `AURORA_SEP_DEVICE_TIMEOUT` in whole seconds,
for example with `systemctl edit aurora-sep.service`:

```ini
[Service]
Environment=AURORA_SEP_READY_TIMEOUT=180
```

SEP writes are always on: xART writes and identity-keybag provisioning have
no switch. The driver's `xart_writes` and `provision_keybag` parameters were
removed (aurora-silicon/linux#2591), and a stale value on the kernel command
line or in modprobe.d is ignored. Kernels from before that change still honour a
`0`, so remove any `xart_writes=0` or `provision_keybag=0` left from an earlier
read-only install. The driver exports no module alias, so udev
never loads it on its own: the service above loads it, once the iBoot system
container is reachable. Writes are accepted only when the APFS `.gl`
extent passes the driver's narrow ownership checks: one physical extent, no snapshots or revert
state, and a single extent reference owned by the `.gl` inode. The kernel
resolves that extent from checksum-validated APFS metadata; unsupported layouts
fail closed. The first boot creates the Linux identity needed for trusted
keys, and later boots recover that identity and its machine reference key.
An existing unreadable or invalid identity is not silently replaced.
Writes were first tested on m2 with identity-keybag provisioning,
trusted-key seal/unseal, and reboot reload. The raw iBoot container hash did not
change in those tests, so an actual xART write was **not** validated. Do not
infer that all SEP or Touch ID operations work with a read-only GigaLocker.
Read `XART-SAFETY.md` before installing on a machine that also runs macOS.

For read-only structural diagnostics on Linux, run
`sudo python3 apfs-xart-inspect.py /dev/disk/by-partlabel/iBootSystemContainer`.
The inspector checks APFS metadata checksums, resolves the latest complete
checkpoint's xART `.gl` file extent, and compares it with the legacy root-record
heuristic. It prints physical addresses and record metadata, not payloads.
It currently supports only a single-node object map and file-system tree and
cannot authorize raw writes. Apple's APFS reference is the format authority:
https://developer.apple.com/support/apple-file-system/Apple-File-System-Reference.pdf.
`python3 -m unittest discover -s tests -v` runs a synthetic regression case:
when the first live root moves to slot 10, the legacy heuristic chooses a
window shifted 90 APFS blocks beyond the actual `.gl` extent. This proves the
locator can misidentify the extent; it does not identify the historical cause
of any particular damaged container.

For fingerprint support, install libfprint with the aurora driver, then the
fprintd device policy below. The userspace driver is maintained in
https://github.com/aurora-silicon/aurora-sep-userspace (libfprint with the
aurora driver, as source and as distribution packages); this tree carries only
the kernel driver, its loader and its service. The driver behaviour fprintd
sees:

The Aurora driver now reports stage progress only when the kernel's stage
number increases; guidance and stage-zero events are not captured samples.
fprintd 1.94.5 itself emits an initial `enroll-stage-passed` after its
duplicate check, **before** starting enrollment. That first line does not
prove a finger was captured.
A capture the enclave cannot use is reported as a retry, both while enrolling
and while matching: fprintd shows `enroll-retry-scan` or `verify-retry-scan`,
and starts a match again by itself. When the kernel ends an enrolment because
too many captures in a row were unusable (`SEP_BIO_STATUS_REJECTED`), the
enrolment fails with that reason in fprintd's log, and fprintd reports
`enroll-unknown-error`. Kernels from before these statuses never send them.

Interface version 5 requires a resolved Unix uid, a known finger, the selected
Linux SEP OS identity, a live identity count below three, a bound identity bag,
and readable, writable owner/user/master Catacomb files before ENROL_START can
schedule enclave capture. On an empty Linux lane, the driver prepares the owner
context after libfprint validates the account; the user/master Catacombs are
saved at the first completed enrolment. The matching aurora libfprint build
rejects a missing passwd entry before either ioctl. A missing persistence file
for an existing identity is an admission error; repair the binding rather than
retrying a capture against unbound state.

`sep-bio-identities` is the recovery interface for a print with no fprintd
record. As root, run `sep-bio-identities` to list only the selected Linux OS
identity's built-in SEP user context. Use `sep-bio-identities --delete-orphan
UUID --keep GOOD_UUID` when another identity must remain. The kernel index may
still hold a UUID whose fprintd record was lost, so compare the IDs carefully.
The utility does not expose DELETE_ALL. Record the list and
backup of any known-good print before deletion, and confirm the deleted UUID
stays absent after a reboot. This path must be tested on the affected hardware
before treating a deletion as a successful recovery.
The device-specific `apple/mesa_calibration.bin` on m2 was recovered using
the bundled [`extract-mesa-calibration.py`](extract-mesa-calibration.py), copied
from [Gist revision `eb079a8007985d04e75182f20994f1a1c496f12e`](https://gist.github.com/DjDeveloperr/867a1961b861c570442724f48f770158/eb079a8007985d04e75182f20994f1a1c496f12e),
which scans the local iBoot System Container read-only for an FSCl/CALB `comb`
record or a standalone FSC2 IMG4 with an IM4M manifest. It checks the container
structure and markers, **not**
the manifest's cryptographic signature. Keep this blob private to the machine;
it is not a generic firmware package. On m2, a premature master
`SAVE_CATACOMB` returned SEP status `0x6` while its component state was `0x3`
(no save-pending bit). The
`m2fix10` fresh-context gate skips that unavailable save, and bounded
enrollment attempts reached the physical-finger wait without an immediate
enclave error. With the later UUID-forwarding stage2, a physical enrollment
completed on m2fix15 and master/user Catacombs were saved, but on each tested
reboot SEP listed zero live identities despite the host index still listing
the finger. m2fix16–m2fix20 narrowed this to the user Catacomb load: it
returns `0x8002` and becomes active, but reveals zero identities even with
the SCRD credential and sensor/device view prepared first. The saved files
remain backed up root-only on m2. A bounded repeated load still listed zero
identities. An opt-in owner export then produced a durable owner file, but
loading it did not recover the old user identity. `m2fix23` is a separate,
boot-tested candidate that saves owner in the fresh context and user before
master at completion; it has not been finger- or reboot-persistence-tested.
Login must stay password-based until a live post-reboot identity and
successful match are independently verified.

On each Linux machine, extract its own calibration from its local iBoot System
Container (the input partition is discovered automatically):

```sh
sudo install -d -m 0755 /usr/lib/firmware/apple
sudo python3 tools/aurora-sep/extract-mesa-calibration.py \
  -o /usr/lib/firmware/apple/mesa_calibration.bin
```

The extractor reads the iBoot partition but never writes to it. It creates the
output mode `0600`, rejects input/output aliases, and refuses ambiguous or
malformed calibration candidates. Do not commit or copy the resulting blob to
another machine.

Once the hardware path is verified, install libfprint and then the fprintd
device policy:

```sh
install -Dm644 fprintd-aurora.conf \
    /etc/systemd/system/fprintd.service.d/aurora.conf
systemctl daemon-reload
systemctl restart fprintd.service
```

Where SELinux is enforcing, as on Fedora, fprintd may not open `/dev/sep-bio`
until it is labelled. Build and load the policy module (Fedora's
`selinux-policy-devel` provides the Makefile), then label the device:

```sh
make -f /usr/share/selinux/devel/Makefile aurora_sep.pp
semodule -i aurora_sep.pp
restorecon /dev/sep-bio
```

The module also lets fprintd read input event devices. libfprint follows the
lid through their SW_LID switch to hide the reader while the lid is closed;
without it fprintd logs "Permission denied" for `/dev/input/event*` and keeps
offering Touch ID in clamshell. The device type, `aurora_sep_device_t`, is the
one the distribution packages use for the same module, so moving between
them keeps `/dev/sep-bio` labelled.

`semodule -i` installs at priority 400, above the packages' 200, so a module
loaded by hand keeps winning after a package is installed. Remove it then with
`semodule -X 400 -r aurora_sep`.

On Omarchy, run `omarchy-apply-lock` once after enrollment. The Quickshell
lock screen then selects `omarchy-lock-fingerprint` automatically and accepts
Touch ID through fprintd. Hyprlock's separate fingerprint switch is not used
by the Quickshell lock screen.

## Checking the SEP state

`aurora-sep status` prints every state the driver reports, one line each
with PASS, WARN, FAIL or INFO: the attach, the twelve endpoints by name
(macOS lists the same twelve as `sep-endpoint,<name>` services), the
bring-up time, the key store and identity keybag, xART, the OS identity
(and the os_uuid pin on the MacBook Neo), the hwrng, Touch ID, the sensor,
the fingerprint restore and matching, the xART guard, failed xART
requests, unhandled messages, sleep, `aurora-sep-ready.service` and the
apple_sep error and warning lines of this boot. It exits 1 if anything
FAILed. `--summary` prints only the last line, `--quiet` hides PASS and
INFO lines and `--json` prints the report as JSON.

It only reads: it never opens `/dev/sep-bio`, since that activates Touch
ID. Run as root and with the enclave as the current hwrng, it reads 32
bytes from `/dev/hwrng`, which shows that the enclave still answers, also
after a resume; `--no-live` skips that. The kernel log check needs root.
Older kernels without some of the diag attributes are checked on what
they have (the endpoint count instead of the names, for example).

## Packages

`packaging/` builds `aurora-sep`, with the loader, its service and the fprintd
policy, as a distribution package. libfprint with the aurora driver comes from
https://github.com/aurora-silicon/aurora-sep-userspace; install both. Installing
`aurora-sep` lets fprintd load the driver with xART writes on, as above.

On Fedora, `packaging/fedora/build.sh` builds `aurora-sep`. It installs build
dependencies, so run it in a throwaway container from the top of this tree:

```sh
mkdir rpms
podman run --rm -v "$PWD":/src:ro,z -v "$PWD/rpms":/out:z \
    registry.fedoraproject.org/fedora:45 \
    /src/tools/aurora-sep/packaging/fedora/build.sh /out
sudo dnf install rpms/aurora-sep-1-*.noarch.rpm rpms/aurora-sep-selinux-1-*.noarch.rpm
```

`aurora-sep-selinux` holds the policy module above. `aurora-sep` requires it
wherever the targeted policy is installed. Where SELinux is disabled, the
module is installed but not loaded, and nothing is relabelled.

**On Fedora, turn on fingerprint authentication yourself.** PAM decides what
asks for a fingerprint, and the packages leave it alone. Fedora's authselect
profiles keep fingerprints off until their `with-fingerprint` feature is
selected. Until then sudo, polkit and the lock screen never ask, and GNOME
Settings shows no Fingerprint Login row, even with a finger enrolled. Once a
finger is enrolled:

```sh
sudo authselect enable-feature with-fingerprint
```

This adds `pam_fprintd` to the system and fingerprint PAM stacks. It also sets
`org.gnome.login-screen enable-fingerprint-authentication`, which authselect
otherwise locks to false. `sudo authselect disable-feature with-fingerprint`
undoes both. While the feature is off, `aurora-sep-ready` logs this command
once per boot. On Omarchy, `aurora-touchid-setup` makes the PAM changes after
an enrolment.

On Arch Linux and Arch Linux ARM, build `aurora-sep`:

```sh
cd tools/aurora-sep/packaging/arch/aurora-sep && makepkg -si
```

fprintd starts the loader the first time it runs. Loading the driver at boot
instead has the enclave ready before the login screen asks for a fingerprint.
The Fedora package enables that with a systemd preset when it is first
installed; on Arch, enable it yourself:

```sh
sudo systemctl enable aurora-sep.service
```

The driver attaches to the enclave once per boot, so reboot rather than
restarting the service or reloading `apple_sep`. To move from the manual
install above, remove `/usr/local/sbin/aurora-sep-load`,
`/etc/systemd/system/aurora-sep.service`,
`/etc/systemd/system/fprintd.service.d/aurora.conf` and any drop-in that points
fprintd at another libfprint, then run `systemctl daemon-reload` and
`systemctl reenable aurora-sep.service`. To go back to the distribution's
libfprint, remove `aurora-sep` and run `dnf distro-sync libfprint` or
`pacman -S libfprint`.

## Desktop integration

The first open of `/dev/sep-bio` activates Touch ID: the driver restores the
enrolled fingerprints from the enclave before the open returns, which takes
a few seconds. `aurora-sep-ready.service`, which `aurora-sep.service` pulls
in, makes that open at boot, before fprintd starts. The lock screen, sudo
and polkit then find the reader and the fingerprints ready, and the first
fingerprint request does not wait for the restore.

An open that succeeds is not enough: the sensor can still fall short of
attaching. The script reports Touch ID ready only once the driver's `diag`
attributes show the sensor attached and the fingerprints restored (or none
enrolled), the state in which the kernel logs `Touch ID: restored N`. The
driver tries an attach that fell short once more about 10 s later, so the
script waits for that, up to 50 s from its start. Otherwise it logs
`Touch ID is NOT ready` with the reason and the unit fails; anything that
merely wants it still starts.

Restored is not the same as able to match: a match rewrites xART session
records, which the xART guard can refuse. The script reads the driver's
`matching` attribute: `usable` is ready, `blocked` is NOT ready with the
reason and the `xart_guard=0` opt-out, and `unverified` (or a kernel that
does not say) logs `Touch ID is restored, matching UNVERIFIED` and does not
fail the unit. See `XART-SAFETY.md`.

The packages install it; by hand:

```sh
install -Dm755 aurora-sep-ready /usr/local/sbin/aurora-sep-ready
install -Dm644 aurora-sep-ready.service /etc/systemd/system/aurora-sep-ready.service
systemctl daemon-reload
```

fprintd exits after 30 s without a client, and each new start probes the
reader again, while the login and lock screens ask for the reader as they
come up. `fprintd-no-timeout.conf.in` keeps it running so that it answers
them at once. The packages fill in where the distribution keeps fprintd; by
hand, with `/usr/lib/fprintd` on Arch:

```sh
sed 's|@FPRINTD@|/usr/libexec/fprintd|' fprintd-no-timeout.conf.in \
    >/etc/systemd/system/fprintd.service.d/aurora-sep-no-timeout.conf
```

The Touch ID sensor is the power button. A touch to authenticate goes to the
enclave and never reaches logind as a key press, but a press does, and logind
cannot tell a press meant for the sensor from one meant for the button. Its
default is to power off. `logind-aurora-sep.conf` makes a short press
suspend instead, so pressing while authenticating loses nothing and the next
press wakes the machine, and holding the button for 5 s powers off cleanly.
GNOME sessions and the GDM login screen handle a short press themselves
(`power-button-action`, `suspend` by default); the setting covers consoles
and other desktops. It takes effect at the next boot. By hand:

```sh
install -Dm644 logind-aurora-sep.conf /etc/systemd/logind.conf.d/aurora-sep.conf
```

GNOME Shell 51 offers fingerprint only on the lock screen, never on the
login screen, and `org.gnome.login-screen enable-fingerprint-authentication`
(true in GNOME's defaults, false under authselect until `with-fingerprint` is
on) cannot change that: the greeter connects its fingerprint reader only
when it re-authenticates a running session. That is GNOME's
choice, because a fingerprint cannot unlock the login keyring: after a
fingerprint login, the first application that wants a stored secret asks
for the password. `aurora-sep-greeter-fingerprint.service` lifts the
restriction for those who want it. At boot it copies the one greeter source
file that holds the check out of the installed GNOME Shell, drops the check,
and serves the copy to the login screen's shell alone through a GResource
overlay. When the check is not exactly as expected, for example after a
GNOME update, it changes nothing. The enable key above still applies.

The login screen asks fprintd for the reader once, as it comes up, and the
enclave can take most of a minute after boot. The login screen does not wait
for it: `aurora-sep-greeter-refresh.service` runs once both are up, and when
Touch ID came up later and nobody has logged in, it shows the login screen
again so it offers fingerprint. With automatic login, or once someone has
logged in, it changes nothing. Both are one switch, off by default. It needs
`gresource` from glib2-devel. With the packages, or after installing it by
hand, turn it on with `systemctl enable`:

```sh
install -Dm755 aurora-sep-greeter-fingerprint /usr/local/sbin/aurora-sep-greeter-fingerprint
install -Dm644 aurora-sep-greeter-fingerprint.service aurora-sep-greeter-refresh.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable aurora-sep-greeter-fingerprint.service
```

Disabling it and rebooting returns the login screen to GNOME's behaviour.

## Biometric userspace header

The canonical ioctl ABI is `include/uapi/linux/apple-sep.h`. Export this
checkout's headers with `make headers_install INSTALL_HDR_PATH=<directory>`
and add that directory's `include` to the libfprint build include path. The
aurora driver in aurora-sep-userspace uses `<linux/apple-sep.h>` directly; it
has no copied ABI header. Interface version, ioctl numbers and fixed-width layouts are unchanged.
