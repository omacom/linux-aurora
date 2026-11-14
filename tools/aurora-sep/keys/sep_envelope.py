# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Reference item envelope for the local biometric ECIES decrypt extension.

Requires Python cryptography. This is an integration example, not a production
secret-memory implementation: Python/backend temporaries cannot all be wiped.
"""

import hashlib
import hmac

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.ciphers.aead import AESGCM


MAX_SEALED = 4096
MAGIC = b'SEPITEM\x01'


def seal_item(public, context, plaintext):
    """Encrypt to a recipient public point; no biometric scan is needed.

    context is the application's canonical binding of owner, record ID, purpose
    and algorithm. It is encrypted/authenticated with the item, not sent as AAD.
    """
    if len(public) != 65 or public[0] != 4:
        raise ValueError('expected uncompressed P-256 public point')
    if not context or len(context) > 1024:
        raise ValueError('expected nonempty bounded item context')
    envelope = MAGIC + len(context).to_bytes(4, 'big') + context + plaintext
    if len(envelope) + 81 > MAX_SEALED:
        raise ValueError('item envelope exceeds driver input limit')
    peer = ec.EllipticCurvePublicKey.from_encoded_point(ec.SECP256R1(), public)
    sender = ec.generate_private_key(ec.SECP256R1())
    ephemeral = sender.public_key().public_bytes(
        serialization.Encoding.X962, serialization.PublicFormat.UncompressedPoint)
    shared = sender.exchange(ec.ECDH(), peer)
    material = b''.join(hashlib.sha256(shared + counter.to_bytes(4, 'big') + ephemeral).digest()
                        for counter in (1, 2))
    # Firmware profile: AES-256, derived 16-byte IV, no AAD, 16-byte tag.
    return ephemeral + AESGCM(material[:32]).encrypt(material[32:48], envelope, None)


def item_view(output, length, expected_context):
    """Validate trusted expected context, then borrow plaintext without copying.

    The caller owns output and must wipe it on success AND on exceptions. Do not
    derive expected_context from an untrusted field beside the ciphertext.
    """
    if not isinstance(output, bytearray) or not 0 <= length <= len(output):
        raise ValueError('invalid plaintext buffer')
    if not expected_context or len(expected_context) > 1024:
        raise ValueError('invalid expected context')
    view = memoryview(output)[:length]
    header = len(MAGIC) + 4
    if length < header or view[:len(MAGIC)] != MAGIC:
        raise ValueError('unsupported item envelope')
    context_len = int.from_bytes(view[len(MAGIC):header], 'big')
    if context_len != len(expected_context) or header + context_len > length:
        raise ValueError('item context mismatch')
    if not hmac.compare_digest(view[header:header + context_len], expected_context):
        raise ValueError('item context mismatch')
    return view[header + context_len:]
