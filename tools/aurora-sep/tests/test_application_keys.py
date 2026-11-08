# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Check the exported ABI and reference-client failure cleanup without SEP."""
from pathlib import Path
import errno
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
KEYS = ROOT / 'tools/aurora-sep/keys'
sys.path.insert(0, str(KEYS))

import sep_key


class ApplicationKeys(unittest.TestCase):
    def test_c_and_cpp_uapi_matches_client(self):
        constants = ('INFO', 'CANCEL', 'CREATE', 'SIGN', 'DECRYPT')
        header = ROOT / 'include/uapi/linux/apple-sep.h'
        source = f'#include \"{header}\"\n#include <stddef.h>\n'
        source += '#ifdef __cplusplus\n#define CHECK static_assert\n#else\n#define CHECK _Static_assert\n#endif\n'
        for name in constants:
            symbol = 'SEP_KEY_' + name
            client = sep_key
            source += f'CHECK({symbol} == {getattr(client, name)}, "{name}");\n'
        for name, size in [('info', 16), ('create', 88), ('sign', 136), ('decrypt', 56), ('cancel', 8)]:
            source += f'CHECK(sizeof(struct sep_key_{name}) == {size}, "{name}");\n'
        source += 'CHECK(offsetof(struct sep_bio_attest, public_key) == 36, "attest");\n'
        source += 'CHECK(offsetof(struct sep_key_create, public_key) == 16, "create");\n'
        source += f'CHECK(SEP_KEY_CAP_SIGN == {sep_key.CAP_SIGN}, "sign capability");\n'
        source += f'CHECK(SEP_KEY_CAP_DECRYPT == {sep_key.CAP_DECRYPT}, "decrypt capability");\n'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'abi.c'
            path.write_text(source)
            for compiler, language in [('cc', 'c'), ('c++', 'c++')]:
                if not shutil.which(compiler):
                    self.skipTest(f'requires {compiler}')
                subprocess.run([compiler, '-x', language, '-fsyntax-only',
                                str(path)], check=True)

    def test_interrupted_and_failed_syscalls_are_not_retried(self):
        for decrypt in (False, True):
            for failure in (errno.EINTR, errno.EIO, errno.EFAULT):
                with self.subTest(decrypt=decrypt, failure=failure):
                    client = sep_key.SepClient()
                    client.fd, client.capabilities = 42, 3
                    output = bytearray(b'x' * sep_key.MAX_BLOB)
                    with patch.object(client, '_call', side_effect=OSError(failure, 'injected')) as call:
                        with self.assertRaises(OSError) as error:
                            if decrypt:
                                client.decrypt_into(b'blob', b'\x04' + bytes(80), output)
                            else:
                                client.sign_digest(b'blob', bytes(32))
                    self.assertEqual(error.exception.errno, failure)
                    self.assertEqual(call.call_count, 1)
                    if decrypt:
                        self.assertFalse(any(output))

    def test_preallocated_cancellation_id_is_not_replaced_or_retried(self):
        client = sep_key.SepClient()
        client.fd, client.capabilities = 42, 3
        request_id = client.new_request_id()
        calls = []

        def call(command, request):
            calls.append((command, struct.unpack_from('=Q', request)[0]))
            if command == sep_key.SIGN:
                raise OSError(errno.ECANCELED, 'retired before acceptance')

        with patch.object(client, '_call', call):
            client.cancel(request_id)
            with self.assertRaises(OSError) as error:
                client.sign_digest(b'blob', bytes(32), request_id=request_id)
        self.assertEqual(error.exception.errno, errno.ECANCELED)
        self.assertEqual(calls, [(sep_key.CANCEL, request_id), (sep_key.SIGN, request_id)])
        self.assertGreater(client.new_request_id(), request_id)


if __name__ == '__main__':
    unittest.main()
