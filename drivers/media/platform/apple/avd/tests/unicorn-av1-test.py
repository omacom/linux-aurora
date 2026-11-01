#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""AV1 production programs: VP(N+1) starts after VP(N) DONE, during PP(N).

Usage: unicorn-av1-test.py FIRMWARE_TREE v5-t2_IMAGE AV1_HOST_OUTPUT
CDF/map input snapshots are host fixtures. CM3 code, descriptors, program words,
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


class AV1(transport.Transport):
    def __init__(self):
        self.booting = False
        self.expected = []
        self.events = []
        self.live = self.pp_owner = None
        self.feed = self.word = 0
        super().__init__(image)
        assert self.version == 5 and self.tier == 2
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
        if addr == self.decode + self.fifo + 9 * 4:
            idx, words = self.expected[self.feed]
            assert value == words[self.word], (idx, self.word, value)
            if not self.word:
                assert self.live is None, 'successor issued before VP DONE'
                self.live = idx
                self.events.append(f'AV1 VP START {idx}; PP owner={self.pp_owner}')
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
        stem = f'av1-overlap-{idx}'
        data = (programs / f'{stem}.bin').read_bytes()
        words = list(struct.unpack('<' + 'I' * (len(data) // 4), data))
        desc = list(struct.unpack('<16I', (programs / f'{stem}-desc.bin').read_bytes()))
        assert desc[:4] == [0x41565131, 0x10 | idx, 3, 9]
        assert desc[4:6] == [0, 0] and desc[6] == len(words)
        assert desc[8:16] == [1, 0, 0, 1, 0, 0, 0, 0]
        assert words[0] == 0x2b000200 | idx << 4
        assert words[-1] == 0x2b000400 | idx << 4
        mmio = (programs / f'{stem}-mmio.bin').read_bytes()
        for offset in (0x1d0, 0x20c, 0x248, 0x284, 0x2c0):
            at = offset + idx * 4
            self.put(self.decode + at, struct.unpack_from('<I', mmio, at)[0])
        for at in (0x16c + 9 * 4, 0x190):  # VP8 / PP local IRQ masks
            self.put(self.decode + at, struct.unpack_from('<I', mmio, at)[0])
        assert self.u32(self.decode + 0x1d0 + idx * 4)
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
        self.events.append(f'AV1 VP DONE {idx}')
        self.route(125)  # VP9 (AV1) unknown/error/DONE: 123/124/125

    def finish_pp(self, idx):
        assert self.pp_owner == idx
        self.pp_owner = None
        self.events.append(f'PP DONE {idx}')
        self.route(64)


for blocked in (False, True):
    t = AV1()
    wp = t.submit_host(0, 0)
    t.poll(20)
    assert t.live == 0 and t.pp_owner is None and not t.notifications
    t.finish_vp(0)
    t.poll(20)
    assert t.pp_owner == 0 and t.live is None and not t.notifications
    # The host fixture proved that the second picture reads only immutable
    # preparation snapshots. No VP notification is needed for this subset.
    wp = t.submit_host(1, wp)
    t.poll(20)
    assert t.live == 1 and t.pp_owner == 0, 'no AV1 VP/PP overlap'
    t.finish_vp(1)
    t.poll(20)
    assert t.pp_owner == 0 and not t.notifications
    t.busy = blocked
    t.finish_pp(0)
    t.poll(20)
    if blocked:
        assert not t.notifications and t.result(0) == [1, 0, 0]
        t.busy = False
        t.poll(20)
    assert t.result(0) == [2, 0, 0x10] and t.notifications == [0x54530010]
    assert t.pp_owner == 1
    t.busy = False
    t.finish_pp(1)
    t.poll(20)
    assert t.result(1) == [2, 0, 0x11]
    assert t.notifications == [0x54530010, 0x54530011]
    assert t.u32(0x10007ff8) == t.u32(0x10007ffc) == wp
    assert not t.fetches and not t.ring_writes
    assert not t.u32(0x1000803c) and not t.u32(0x1000807c)
    print('\n'.join('TRACE ' + e for e in t.events))
    print(f'PASS: AV1 snapshot picture overlap, blocked PP mailbox={blocked}; VP(1) issued before PP(0) DONE')
