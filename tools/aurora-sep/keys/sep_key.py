#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Reference client for blocking, biometric application keys on /dev/sep-keys.

Allocate a request ID before dispatching a worker, pass it as request_id=, and
cancel that ID from the UI thread. Never automatically retry an interrupted or
uncertain key operation. Join the caller before closing the device context.
"""

import argparse
import base64
import ctypes
import errno
import hashlib
import json
import os
from pathlib import Path
import struct
import threading


MAX_BLOB = 4096


def ioctl_number(direction, number, size):
    return (direction << 30) | (size << 16) | (0xB2 << 8) | number


INFO = ioctl_number(2, 0, 16)
CREATE = ioctl_number(3, 1, 88)
SIGN = ioctl_number(3, 2, 136)
DECRYPT = ioctl_number(3, 3, 56)
CANCEL = ioctl_number(1, 4, 8)
CAP_SIGN, CAP_DECRYPT = 1, 2
libc = ctypes.CDLL(None, use_errno=True)
libc.ioctl.argtypes = (ctypes.c_int, ctypes.c_ulong, ctypes.c_void_p)
libc.ioctl.restype = ctypes.c_int


class SepClient:
    def __init__(self):
        self._next_id = 0
        self._id_lock = threading.Lock()

    def __enter__(self):
        self.fd = os.open('/dev/sep-keys', os.O_RDWR | os.O_CLOEXEC)
        try:
            info = bytearray(16)
            self._call(INFO, info)
            version, self.capabilities, r1, r2 = struct.unpack('=4I', info)
            if version != 1 or r1 or r2:
                raise RuntimeError('unsupported key interface')
            if not self.capabilities & CAP_SIGN:
                raise OSError(errno.EOPNOTSUPP, 'application-key API unavailable')
        except BaseException:
            os.close(self.fd)
            raise
        return self

    def __exit__(self, *unused):
        os.close(self.fd)

    def _call(self, command, request):
        # fcntl.ioctl may automatically retry EINTR. Call libc exactly once.
        storage = (ctypes.c_ubyte * len(request)).from_buffer(request)
        if libc.ioctl(self.fd, command, ctypes.addressof(storage)) == -1:
            error = ctypes.get_errno()
            raise OSError(error, os.strerror(error))

    def new_request_id(self):
        with self._id_lock:
            if self._next_id == (1 << 64) - 1:
                raise OverflowError('open a new key device file')
            self._next_id += 1
            return self._next_id

    def cancel(self, request_id):
        request_id = self._request_id(request_id)
        self._call(CANCEL, bytearray(struct.pack('=Q', request_id)))

    def _request_id(self, request_id):
        if request_id is None:
            return self.new_request_id()
        if not 0 < request_id < 1 << 64:
            raise ValueError('invalid request ID')
        with self._id_lock:
            self._next_id = max(self._next_id, request_id)
        return request_id

    def create(self, decrypt=False):
        if decrypt and not self.capabilities & CAP_DECRYPT:
            raise OSError(errno.EOPNOTSUPP, 'biometric decryption unavailable')
        blob = ctypes.create_string_buffer(MAX_BLOB)
        request = bytearray(struct.pack('=IIQ65s7x', int(decrypt), MAX_BLOB,
                                        ctypes.addressof(blob), b''))
        self._call(CREATE, request)
        _, size, _, public = struct.unpack('=IIQ65s7x', request)
        if not 0 < size <= MAX_BLOB or public[0] != 4:
            raise RuntimeError('invalid key response')
        return bytes(blob[:size]), public

    def sign_digest(self, blob, digest, timeout_ms=60000, *, request_id=None):
        if not 0 < len(blob) <= MAX_BLOB or len(digest) != 32:
            raise ValueError('expected wrapped key and SHA-256 digest')
        if not 1 <= timeout_ms <= 120000:
            raise ValueError('timeout must be 1..120000 ms')
        request_id = self._request_id(request_id)
        key = ctypes.create_string_buffer(blob)
        request = bytearray(struct.pack('=QIIIIQ32s72x', request_id, 0, timeout_ms, len(blob), 0,
                                        ctypes.addressof(key), digest))
        self._call(SIGN, request)
        size = struct.unpack_from('=I', request, 20)[0]
        if not 0 < size <= 72:
            raise RuntimeError('invalid signature length')
        return bytes(request[64:64 + size])

    def decrypt_into(self, blob, sealed, output, timeout_ms=60000, *, request_id=None):
        if not isinstance(output, bytearray) or len(output) != MAX_BLOB:
            raise ValueError('output must be a 4096-byte bytearray')
        destination = (ctypes.c_ubyte * len(output)).from_buffer(output)
        ctypes.memset(ctypes.addressof(destination), 0, len(output))
        success = False
        try:
            if not self.capabilities & CAP_DECRYPT:
                raise OSError(errno.EOPNOTSUPP, 'biometric decryption unavailable')
            if not 0 < len(blob) <= MAX_BLOB or not 81 <= len(sealed) <= MAX_BLOB:
                raise ValueError('invalid wrapped key or sealed length')
            if sealed[0] != 4 or not 1 <= timeout_ms <= 120000:
                raise ValueError('expected P-256 point and valid timeout')
            request_id = self._request_id(request_id)
            key, ciphertext = ctypes.create_string_buffer(blob), ctypes.create_string_buffer(sealed)
            request = bytearray(struct.pack('=QIIIIQQIIQ', request_id, 0, timeout_ms, len(blob), len(sealed),
                ctypes.addressof(key), ctypes.addressof(ciphertext), len(output), 0,
                ctypes.addressof(destination)))
            self._call(DECRYPT, request)
            size = struct.unpack_from('=I', request, 40)[0]
            if size > len(output):
                raise RuntimeError('invalid plaintext length')
            success = True
            return size
        finally:
            if not success:
                ctypes.memset(ctypes.addressof(destination), 0, len(output))



def verify(public, digest, signature):
    from cryptography.hazmat.primitives import hashes
    from cryptography.hazmat.primitives.asymmetric import ec, utils

    key = ec.EllipticCurvePublicKey.from_encoded_point(ec.SECP256R1(), public)
    key.verify(signature, digest, ec.ECDSA(utils.Prehashed(hashes.SHA256())))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    create = commands.add_parser('create')
    create.add_argument('key', type=Path)
    create.add_argument('--decrypt', action='store_true',
                        help='create a biometric signing/decryption key')
    for name in ('sign', 'sign-digest'):
        sign = commands.add_parser(name)
        sign.add_argument('key', type=Path)
        sign.add_argument('message', type=Path)
        sign.add_argument('signature', type=Path)
    check = commands.add_parser('selftest-decrypt', help='decrypt disposable data with an existing key')
    check.add_argument('key', type=Path)
    args = parser.parse_args()
    if args.command == 'create':
        if args.key.exists():
            raise FileExistsError(args.key)
        with SepClient() as client:
            blob, public = client.create(decrypt=args.decrypt)
        data = {'version': 1, 'policy': 'biometry-any',
                'blob': base64.b64encode(blob).decode(),
                'public': base64.b64encode(public).decode()}
        if args.decrypt:
            data['operations'] = ['sign', 'decrypt']
        fd = os.open(args.key, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o600)
        with os.fdopen(fd, 'w') as output:
            json.dump(data, output)
            output.flush()
            os.fsync(output.fileno())
        print('Biometric P-256 key created')
    else:
        if args.key.stat().st_size > 8192:
            raise ValueError('key file too large')
        data = json.loads(args.key.read_text())
        if data['version'] != 1 or data['policy'] != 'biometry-any':
            raise ValueError('unsupported key file')
        blob = base64.b64decode(data['blob'], validate=True)
        public = base64.b64decode(data['public'], validate=True)
        if args.command == 'selftest-decrypt':
            from sep_envelope import seal_item, item_view
            context = b'sep-example:selftest-decrypt:v1'
            expected = os.urandom(32)
            sealed = seal_item(public, context, expected)
            output = bytearray(MAX_BLOB)
            try:
                print('Place an enrolled finger on Touch ID', flush=True)
                with SepClient() as client:
                    length = client.decrypt_into(blob, sealed, output)
                if item_view(output, length, context) != expected:
                    raise RuntimeError('disposable plaintext mismatch')
            finally:
                destination = (ctypes.c_ubyte * MAX_BLOB).from_buffer(output)
                ctypes.memset(ctypes.addressof(destination), 0, MAX_BLOB)
            print('Disposable decryption verified')
            return
        with args.message.open('rb') as message:
            if args.command == 'sign':
                digest = hashlib.file_digest(message, 'sha256').digest()
            else:
                digest = message.read(33)
                if len(digest) != 32:
                    raise ValueError('sign-digest requires exactly 32 raw SHA-256 bytes')
        print('Place an enrolled finger on Touch ID', flush=True)
        with SepClient() as client:
            signature = client.sign_digest(blob, digest)
        verify(public, digest, signature)
        args.signature.write_bytes(signature)
        print('Biometric signature verified')


if __name__ == '__main__':
    main()
