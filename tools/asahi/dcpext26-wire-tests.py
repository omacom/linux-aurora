#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Execute the production native26 encoder against independent byte fixtures."""
import ctypes as C
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SIZE = 0x1BD8


class Primary(C.Structure):
    _fields_ = [("address", C.c_uint64), ("backing", C.c_uint64),
                ("width", C.c_uint32), ("height", C.c_uint32), ("pitch", C.c_uint32)]


def fixture(layer, address, width, height, pitch, swap_id=17):
    """Byte schema, including every zero byte, independent of C field writes."""
    fields = {0x98: struct.pack('<I', swap_id),
              0x14C: struct.pack('<II', 0x80000007, 0x80000007),
              0x154: struct.pack('<I', 0xFF000000),
              0x1BCB: bytes([1] * 10), 0x1BD6: b'\1\1'}
    if address:
        base = 0x588 + layer * 0x22C
        groups = {2: b'\1', 3: struct.pack('<II', 1, 1), 11: b'ARGB',
                  0x13: b'\15\14', 0x19: b'\1\0\1\1',
                  0x15: struct.pack('<I', pitch),
                  0x21: struct.pack('<III', width, height, pitch * height),
                  0x35: struct.pack('<I', 1), 0x51: struct.pack('<QII', 1, width, height),
                  0x69: struct.pack('<IIHBB', pitch, pitch * height, 4, 1, 1),
                  0x149: struct.pack('<Q', 1)}
        fields.update({base + off: value for off, value in groups.items()})
        fields[0x9C + 4 * layer] = struct.pack('<I', 1)
        fields[0xE38 + 8 * layer] = struct.pack('<Q', address)
        for base in (0xAC, 0x10C):
            fields[base + 16 * layer] = struct.pack('<IIII', 0, 0, width, height)
    wire = bytearray(SIZE)
    for off, value in fields.items():
        wire[off:off + len(value)] = value
    if address:
        wire[0x1BCB + layer] = 0
    return wire


class WireTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        p = Path(cls.temp.name)
        (p / 'linux').mkdir()
        for name, body in {
            'types.h': '#include <stdint.h>\n#include <stddef.h>\ntypedef uint8_t u8; typedef uint32_t u32; typedef uint64_t u64;\n',
            'errno.h': '#ifdef __linux__\n#include <asm-generic/errno.h>\n#else\n#include <errno.h>\n#endif\n', 'string.h': '#include <string.h>\n',
            'unaligned.h': '''#include <stdint.h>
#include <string.h>
static inline void put_unaligned_le32(uint32_t v, void *p) {
 unsigned char *b=p; for (unsigned i=0;i<4;i++) b[i]=v>>(8*i);
}
static inline void put_unaligned_le64(uint64_t v, void *p) {
 unsigned char *b=p; for (unsigned i=0;i<8;i++) b[i]=v>>(8*i);
}
static inline uint32_t get_unaligned_le32(const void *p) {
 const unsigned char *b=p; uint32_t v=0;
 for (unsigned i=0;i<4;i++) { v|=(uint32_t)b[i]<<(8*i); }
 return v;
}
''',
        }.items():
            (p / 'linux' / name).write_text(body)
        (p / 'test.c').write_text('''#include "dcpext26-wire.h"
int encode(void *b, size_t n, u32 id, unsigned layer, const struct dcpext26_primary *p) {
 return dcpext26_primary_encode(b,n,id,layer,p);
}
int start(const void *r,size_t n,u32 *id) { return dcpext26_start_reply(r,n,id); }
int submit(const void *r,size_t n) { return dcpext26_submit_reply(r,n); }
''')
        exe = p / 'wire.so'
        subprocess.run([os.environ.get('CC', 'cc'), '-shared', '-fPIC', '-O2',
                        '-Wall', '-Wextra', '-Werror', '-I' + str(p),
                        '-I' + str(ROOT / 'drivers/gpu/drm/apple'), str(p / 'test.c'),
                        '-o', str(exe)], check=True)
        cls.lib = C.CDLL(str(exe))
        cls.lib.encode.argtypes = [C.c_void_p, C.c_size_t, C.c_uint32,
                                   C.c_uint, C.POINTER(Primary)]
        cls.lib.start.argtypes = [C.c_void_p, C.c_size_t, C.POINTER(C.c_uint32)]
        cls.lib.submit.argtypes = [C.c_void_p, C.c_size_t]

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_complete_surface_records(self):
        for layer in range(4):
            for w, h, pitch in [(3840, 2160, 15360), (1920, 1080, 8192), (1, 1, 4)]:
                p = Primary(0x12345678000, pitch * h + 4096, w, h, pitch)
                b = C.create_string_buffer(SIZE + 2)
                C.memset(b, 0xCD, SIZE + 2)
                self.assertEqual(self.lib.encode(C.byref(b, 1), SIZE, 17, layer, C.byref(p)), 0)
                self.assertEqual(b.raw[1:-1], fixture(layer, p.address, w, h, pitch))
                self.assertEqual(b.raw[0], 0xCD)
                self.assertEqual(b.raw[-1], 0xCD)

    def test_null_surface(self):
        b = C.create_string_buffer(SIZE)
        self.assertEqual(self.lib.encode(b, SIZE, 17, 0, None), 0)
        self.assertEqual(b.raw, fixture(0, 0, 0, 0, 0))

    def test_invalid_inputs_preserve_buffer(self):
        good = (0x100000, 33177600, 3840, 2160, 15360)
        cases = [(0, *good[1:]), (good[0], 4, *good[2:]),
                 (*good[:2], 0, *good[3:]), (*good[:3], 0, good[4]),
                 (*good[:4], 15359), (*good[:4], 16),
                 (2**64-4, *good[1:]), (good[0], 2**64-1, 1, 2**32-1, 8)]
        for values in cases:
            b = C.create_string_buffer(b'\xcd' * SIZE, SIZE)
            p = Primary(*values)
            self.assertLess(self.lib.encode(b, SIZE, 17, 0, C.byref(p)), 0)
            self.assertEqual(b.raw, b'\xcd' * SIZE)
        p = Primary(*good)
        for length, swap_id, layer in [(SIZE-1, 17, 0), (SIZE+1, 17, 0),
                                      (SIZE, 0, 0), (SIZE, 17, 4), (SIZE, 17, 2**32-1)]:
            b = C.create_string_buffer(b'\xcd' * SIZE, SIZE)
            self.assertLess(self.lib.encode(b, length, swap_id, layer, C.byref(p)), 0)
            self.assertEqual(b.raw, b'\xcd' * SIZE)

    def test_reply_lengths_and_status_offsets(self):
        ident = C.c_uint32(0xFFFFFFFF)
        for n in range(20):
            start = C.create_string_buffer(struct.pack('<II', 23, 0) + bytes(16))
            self.assertEqual(self.lib.start(start, n, C.byref(ident)) == 0, n == 8)
            submit = C.create_string_buffer(bytes(24))
            self.assertEqual(self.lib.submit(submit, n) == 0, n == 12)
        for off in range(4, 8):
            b = bytearray(struct.pack('<II', 23, 0)); b[off] = 1
            ident.value = 0xFFFFFFFF
            self.assertLess(self.lib.start(bytes(b), 8, C.byref(ident)), 0)
            self.assertEqual(ident.value, 0xFFFFFFFF)
        self.assertLess(self.lib.start(bytes(8), 8, C.byref(ident)), 0)
        for off in range(12):
            b = bytearray(12); b[off] = 1
            self.assertEqual(self.lib.submit(bytes(b), 12) < 0, 5 <= off < 9)


if __name__ == '__main__':
    unittest.main()
