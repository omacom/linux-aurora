J715 Touch ID session inputs
===========================

The T8132/J715 profile uses nonce authentication and source-6 calibration.
It does not change the other boards' session policies, xART guards or
persistence rules. It still requires a successful sensor power cycle before
registration.

Root-key firmware
-----------------

``apple/msrk-j715.bin`` must contain this machine's own FDR MSRk data: the
complete, unmodified body supplied to SBIO SET_MSRK_DATA (opcode ``0x86``).
The qualified input is 10,848 bytes, as in the successful J715 candidate.
Do not substitute J700's signed-container file, strip or add a wrapper, pad
another input to this size, or use another machine's data.

The driver checks the length, not a private identifier or hash. It does not
parse or authenticate the FDR container; a matching length is not evidence
of validity. The enclave must accept the unmodified data. Acquisition and
container verification belong to provisioning, not the kernel driver.
No automatic extraction or alternate-file fallback is provided.

After SENSOR_NONCE, the driver first requests SESSION_SHARE normally.
Only an explicit status ``0x16`` on J715 triggers the firmware request and
upload. A successful upload permits exactly one further SESSION_SHARE
request within that session attempt. A missing file, wrong length, failed
upload, failed transfer or refused retry stops the attempt. A successful
initial share does not read the firmware file.

J700 retains ``apple/msrk-j700.bin``, the nonempty signed-container rule,
and its upload before SESSION_SHARE once per enclave lifetime. It does not
use J715's refusal recovery.

Calibration and validation
--------------------------

The existing sensor ``firmware-name`` property, or its existing default
``apple/mesa_calibration.bin``, supplies calibration. J715 requires its own
source-6 FSC2 data from the successful setup, not the older extracted FDR3
input. The existing calibration encoder prefixes the unchanged body with a
four-byte source word. This patch does not add a calibration fallback.

With ``CONFIG_APPLE_SEP_KUNIT_TEST=y``, the
``apple_sep_sensor_root_key`` suite tests board selection, input-length
boundaries and refusal classification. Also run ``apple_sep_profile``,
``apple_sep_dart_window`` and ``apple_sep_registration`` to cover the
existing profile and registration behavior.

The old candidate's successful enrollment and same-boot verification do not
qualify this upstream kernel. Hardware validation must establish sensor
readiness, enrollment, matching and persistence across a cold boot, with
the existing power-cycle and registration guards enabled. Login integration
is userspace work; disk unlock is outside this support.
