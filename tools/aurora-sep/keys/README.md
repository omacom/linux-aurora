# Biometric application keys

`/dev/sep-keys` creates device-bound P-256 keys, signs SHA-256 digests and
performs biometric ECIES decryption. Private-key operations require a fresh
fingerprint match into a kernel-owned credential; the credential is disposed
before success is delivered. The canonical ABI is `<linux/apple-sep.h>`, exported
by `make headers_install`. Its comments define layouts, limits and ownership.

This is a separate device from `/dev/sep-bio`. Query its interface version and
capabilities before use. Unsupported profiles or a failed credential-cleanup
latch report unavailable capabilities; operations return `EOPNOTSUPP`. This does
not add enrollment or deletion commands to the key device.

There is **no password/passphrase fallback**. The driver's automatically supplied
identity-bag secret is machine-held setup material, not fresh user authentication.
Current keys accept any enrolled finger; adding a finger can give it access to
existing keys. Current-enrollment-set binding and private-key import are not
implemented. Adding a future creation policy will not upgrade existing blobs.

## Reference client

`sep_key.py` is an importable Python 3.11+ client requiring `cryptography`.
It needs an existing biometric installation with enrolled fingers and enabled
persistence. It does not provision keybags, enroll fingers or change services.
Run these examples from this directory:

```sh
sudo python3 sep_key.py create signing-key.json
printf 'disposable message' > message
sudo python3 sep_key.py sign signing-key.json message signature.der
sudo python3 sep_key.py create --decrypt item-key.json
sudo python3 sep_key.py selftest-decrypt item-key.json
```

Creation requires no fingerprint scan. `sign` hashes once with SHA-256, then
independently verifies the returned signature before writing it. `sign-digest`
accepts exactly 32 digest bytes. The decryption self-test uses disposable data.

Persist the opaque blob and public point atomically in authenticated application
storage. The example JSON is a container, not an authorization record. Blobs do
not restore keys on another device or after erasing secure state. Key population
limits have not been measured.

Python callers use `from sep_key import SepClient`, then `sign_digest(blob, digest)`
or `decrypt_into(blob, sealed, output)`. Decryption fills a caller-owned 4096-byte
bytearray and returns its used length. The client wipes it on failure; the caller
must wipe successful output after use. Python/backend temporary copies cannot all
be erased; production memory handling belongs in the application's bounded,
zeroizing secret-memory implementation.

## Requests and cancellation

Opening a key file does not reserve the sensor. Idle BIO and key files can
coexist; conflicting hardware operations return `EBUSY` before submission. Each
key file accepts one operation at a time, with no kernel request queue. A worker
retains the hardware lease until cleanup, even if its waiting caller has returned.
BIO queries that need firmware may also receive `EBUSY` during a capture.

For an interruptible UI, keep the device context open and:

1. Allocate `request_id = client.new_request_id()` before starting a worker.
2. Pass `request_id=` to `sign_digest` or `decrypt_into` on that worker.
3. Revoke application delivery immediately on UI cancellation and call
   `client.cancel(request_id)` from another thread.
4. Join the caller before closing the device context. Discard any response for
   a revoked application request, even if completion raced cancellation.

IDs are nonzero and strictly increasing per open file. Preacceptance cancellation
retires unaccepted IDs through the canceled ID, covering input-copy races. It
never cancels a different active request. `EALREADY` means a result was claimed;
`ENOENT` means no cancellable accepted request with that ID remains. Neither is
authorization to use a revoked result or repeat a key operation.

CANCEL revokes delivery; it cannot undo firmware work already submitted. Closing
an fd in another thread does not cancel its active syscall. Signals return final
`EINTR` after submission. The client calls libc directly because some runtimes
retry interrupted ioctls automatically; private-key requests must never be
implicitly resubmitted.

## Results and errors

The blocking call returns one signature or plaintext result. It claims ownership
before any user copy, outside file/request state locks. An `EFAULT`, including a
response-descriptor fault after a successful data copy, consumes the result.
`ENOSPC` reports required capacity but also consumes it. Wipe any partial output;
neither error permits redelivery or automatic resubmission.

Cancellation/deadlines discard undelivered results. An in-flight copy cannot be
recalled. Request deadlines do not bound a stalled user copy or erase application
memory. Completed failures retain their diagnosis.

`EACCES` reports mismatch/authorization refusal, `ECANCELED` cancellation,
`ETIMEDOUT` expiry and `EROFS` disabled persistence. `EIO` includes uncertain
transport, protocol, decrypt or cleanup failures. Worker busy failures also use
`EIO`, preserving `EBUSY` as pre-submission contention. Credential-disposal failure
disables the application-key API until reboot. No distinct biometric-lockout or
password-required reason is currently exposed; do not infer unlimited attempts.

Never automatically retry an uncertain private-key operation. Bounded waiting
before a new submission after a known `EBUSY` refusal is different from replaying
an operation that may have reached firmware. Keep explicit user-request and
revocation state in the broker.

## Application security boundary

A trusted broker enforces caller/session identity, item ownership, allowed use
and WebAuthn relying-party/origin rules. Device access is privileged and does not
isolate Linux users sharing one SEP identity. A touch does not bind a trusted
display to the user's intended transaction. Comprehensive protection against a
compromised host/root/kernel has not been established.

Native signing returns a signature while the private scalar stays in SEP.
Decryption returns application plaintext to the host. A synced/software passkey
stored as an encrypted item therefore signs on the host after unsealing. Native
SEP passkeys need their public key registered with the relying party; WebAuthn
signs SHA256(authenticatorData || clientDataHash). Two different signing keys do
not provide alternative unlock routes for the same credential.

The combined creation policy deliberately enables signing and decryption. Its
signing-only counterpart has not received a complete forbidden-operation matrix.
The broker must enforce its narrower usage rules. Ordinary BIO tokens, machine
attestation and trusted-key sealing do not grant biometric application-key use.

`sep_envelope.seal_item(public, context, plaintext)` encrypts without a scan.
After decryption, `item_view(output, length, expected_context)` validates context
and returns a borrowed view. Derive context from trusted selected-record state,
binding owner, application, record, purpose and algorithm. Checking it does not
prevent rollback or identify the sender: anyone with the public point can encrypt.
The application remains responsible for authenticated storage and rollback policy.
