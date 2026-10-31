J713 Touch ID session inputs
===========================

The T8132/J713 profile (MacBook Air 13-inch, M4) is the J715 profile with
J713's own root-key file. Read j715-touchid.rst first: the nonce session,
the single SESSION_SHARE retry after status ``0x16``, the length rule and
the calibration rules are the same, and so are the provisioning limits.

Root-key firmware
-----------------

``apple/msrk-j713.bin`` must contain this machine's own FDR MSRk data, the
complete, unmodified body supplied to SBIO SET_MSRK_DATA (opcode ``0x86``).
The observed J713 body is 10,848 bytes, the same length as J715's. The
signed IMG4 payload alone (1,726 bytes on the tested machine) is refused by
the enclave with status ``0x1c``; do not substitute it or J700's container.
J713 never reads ``apple/msrk-j715.bin``.

Device tree
-----------

The sensor is on spi2 (``0x3ad108000``), chip select 0, powered by NUB GPIO
28 (active high). It answers in SPI mode 2, so the board node sets
``spi-cpol`` and not ``spi-cpha``; the sensor ID reads back as ``0x3356``.
The data-ready line (AP GPIO 199 in the ADT) is not described, so the
driver polls. Calibration is the machine's own source-6 FSC2 data under
``apple/mesacal-j713.bin`` (130,427 bytes on the tested machine).

Validation
----------

On one J713, with this profile ported to the Omarchy M4 kit kernel 7.1.9
and the default ``xart_guard`` of 1:

- the first SESSION_SHARE returned ``0x16``, the J713 body was loaded and
  the retry was accepted;
- a fingerprint was enrolled and matched, and another finger was rejected;
- after a cold boot the driver restored the enrolment from the Catacomb
  files and the same finger matched again;
- the guard refused no write. The only change in the gigalocker was the
  enclave rewriting a shared root record, which guard 1 permits.

The identity keybag used in that run had been created earlier by a
development build with ``xart_guard=0``. Under guard 1 that creation was
refused: after rewriting the shared root, the enclave rewrote a session
record that predates Linux. A J713 that has never had a keybag has not been
tested with the guard on. Login integration and suspend were not tested.
