#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""VP9 production programs: VP(N+1) starts after VP(N) DONE, during PP(N).

Usage: unicorn-vp9-test.py FIRMWARE_TREE v3-t1_IMAGE VP9_HOST_OUTPUT
The symbol counts are a host fixture. CM3 code, descriptors, program words,
PP commands and configure_stream writes are production. No pixels are decoded.
"""
import importlib.util
from pathlib import Path
import struct
import sys
from unicorn.arm_const import UC_ARM_REG_SP

fw, image, programs = map(lambda p: Path(p).resolve(), sys.argv[1:4])
spec = importlib.util.spec_from_file_location('transport', fw / 'tests/transport-test.py')
transport = importlib.util.module_from_spec(spec)
spec.loader.exec_module(transport)


class VP9(transport.Transport):
    def __init__(self):
        self.booting = False
        self.expected = []
        self.events = []
        self.live = self.pp_owner = None
        self.feed = self.word = 0
        super().__init__(image)
        assert self.version == 3 and self.tier == 1
        self.booting = True
        self.uc.reg_write(UC_ARM_REG_SP, self.u32(0))
        self.uc.emu_start(self.u32(4), 0, count=500000)
        assert not self.booting

    def route(self, irq):
        if irq < 14:
            router = self.u32(0x50010010) & (1 << irq)
        else:
            off = irq - 14
            router = self.u32(0x50010014 + off // 32 * 4) & (1 << (off % 32))
        nvic = self.u32(0xe000e100 + irq // 32 * 4) & (1 << (irq % 32))
        assert router and nvic, f'IRQ {irq} masked'
        self.call(f'irq{irq}')

    def write(self, uc, access, addr, size, value, user):
        if self.booting and addr == 0x50010090 and value == 1:
            self.booting = False
            uc.emu_stop()
        if addr == self.decode + self.fifo + 8 * 4:
            idx, words = self.expected[self.feed]
            assert value == words[self.word], (idx, self.word, value)
            if not self.word:
                assert self.live is None, 'successor issued before VP DONE'
                self.live = idx
                self.events.append(f'VP9 VP START {idx}; PP owner={self.pp_owner}')
            self.word += 1
            if self.word == len(words):
                self.feed += 1
                self.word = 0
        if addr == self.decode + self.pp:
            idx = (value >> 4) & 15
            assert self.pp_owner is None, 'successor PP issued before predecessor PP DONE'
            assert value == 0x2b00020f | idx << 4
            self.pp_owner = idx
            self.events.append(f'PP START {idx}')
        if addr == 0x50010060:
            self.events.append(f'notification {value:#x}')
        super().write(uc, access, addr, size, value, user)

    def submit_host(self, idx, start):
        stem = f'vp9-adaptive-{idx}'
        data = (programs / f'{stem}.bin').read_bytes()
        words = list(struct.unpack('<' + 'I' * (len(data) // 4), data))
        desc = list(struct.unpack('<16I', (programs / f'{stem}-desc.bin').read_bytes()))
        assert desc[:4] == [0x41565131, 0x10 | idx, 2, 8]
        assert desc[4:6] == [0, 0] and desc[6] == len(words)
        assert desc[8:16] == [1, 0, 0, 3, 0, 0, 0, 0]
        assert words[0] == 0x2b000200 | idx << 4
        assert words[-1] == 0x2b000400 | idx << 4
        mmio = (programs / f'{stem}-mmio.bin').read_bytes()
        for offset in (0x150, 0x18c, 0x1c8, 0x204):
            at = offset + idx * 4
            self.put(self.decode + at, struct.unpack_from('<I', mmio, at)[0])
        for at in (0x11c, 0x120):  # VP8 / PP local IRQ masks
            self.put(self.decode + at, struct.unpack_from('<I', mmio, at)[0])
        assert self.u32(self.decode + 0x150 + idx * 4)
        desc[4] = start
        self.uc.mem_write(0x10008000 + idx * 64, struct.pack('<16I', *desc))
        self.expected.append((idx, words))
        transport.ring_write(self, start, words)
        self.put(0x10007ffc, start + len(words))
        self.put(0x50010058, self.ring + idx * 64)
        self.events.append(f'host submit {idx}')
        self.route(1)
        return start + len(words)

    def finish_vp(self, idx):
        assert self.live == idx and not self.word
        self.live = None
        self.events.append(f'VP9 VP DONE {idx}')
        self.route(120)  # VP8 unknown/error/DONE: 118/119/120

    def finish_pp(self, idx):
        assert self.pp_owner == idx
        self.pp_owner = None
        self.events.append(f'PP DONE {idx}')
        self.route(64)


for delay in (False, True):
    t = VP9()
    wp = t.submit_host(0, 0)
    t.poll(20)
    assert t.live == 0 and t.pp_owner is None and not t.notifications
    if delay:
        t.busy = True  # keep VP notification blocked while PP can run
    t.finish_vp(0)
    t.poll(20)
    assert t.pp_owner == 0 and t.live is None and t.result(0) == [1, 0, 0]
    if delay:
        assert not t.notifications
        t.busy = False
        t.poll(2)
    assert t.notifications == [0x54540010]
    assert t.u32(0x10008038) == 0x10 and t.u32(0x1000803c) == 1
    # Host validates the full VP tag, adapts the fixture counts and prepares
    # the production bank-1 program (the C host test checks the arithmetic).
    t.busy = False
    wp = t.submit_host(1, wp)
    t.poll(20)
    assert t.live == 1 and t.pp_owner == 0, 'no VP/PP overlap'
    t.finish_vp(1)
    t.poll(10)
    assert t.pp_owner == 0 and t.notifications[-1] == 0x54540011
    # Hold the unread VP notification across PP DONE: no token may be lost.
    t.finish_pp(0)
    t.poll(10)
    assert t.result(0) == [1, 0, 0]
    t.busy = False
    t.poll(10)
    assert t.result(0) == [2, 0, 0x10] and t.notifications[-1] == 0x54530010
    assert t.pp_owner == 1
    t.finish_pp(1)
    t.poll(10)
    assert t.result(1) == [1, 0, 0]
    t.busy = False
    t.poll(10)
    assert t.result(1) == [2, 0, 0x11]
    assert t.notifications == [0x54540010, 0x54540011, 0x54530010, 0x54530011]
    assert t.u32(0x10007ff8) == t.u32(0x10007ffc) == wp
    assert not t.fetches and not t.ring_writes
    print('\n'.join('TRACE ' + e for e in t.events))
    print(f'PASS: VP9 adaptive picture overlap, delayed VP mailbox={delay}; VP(1) issued before PP(0) DONE')
