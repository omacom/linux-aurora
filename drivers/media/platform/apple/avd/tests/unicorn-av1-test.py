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
    def __init__(self, multi=False):
        self.multi = multi
        self.engines = {}
        self.live_engines = {}
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

    def drain(self, n):
        # Simulate a host consuming each available token; self.busy separately
        # models a deliberately blocked outbox in the poll-only sections.
        for _ in range(n):
            self.busy = False
            self.poll()

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
        if self.decode + self.fifo <= addr < self.decode + self.fifo + self.vps * 4:
            idx, words = self.expected[self.feed]
            assert value == words[self.word], (idx, self.word, value)
            vp = self.engines[idx]
            assert addr == self.decode + self.fifo + vp * 4
            if not self.word and self.multi:
                assert vp not in self.live_engines, "VP reused before its DONE"
                self.live_engines[vp] = idx
            if not self.word and not self.multi:
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
        stem = f"av1-{'multi' if self.multi else 'overlap'}-{idx}"
        data = (programs / f'{stem}.bin').read_bytes()
        words = list(struct.unpack('<' + 'I' * (len(data) // 4), data))
        desc = list(struct.unpack('<16I', (programs / f'{stem}-desc.bin').read_bytes()))
        vp = 9 + idx % 3 if self.multi else 9
        assert self.pool[3] & (1 << vp)
        assert desc[:4] == [0x41565131, 0x10 | idx, 3, vp]
        self.engines[idx] = vp
        assert desc[4:6] == [0, 0] and desc[6] == len(words)
        assert desc[8:16] == [1, 0, 0, 3, 0, 0, 0, 0]
        assert words[0] == 0x2b000200 | idx << 4
        assert words[-1] == 0x2b000400 | idx << 4
        mmio = (programs / f'{stem}-mmio.bin').read_bytes()
        for offset in (0x1d0, 0x20c, 0x248, 0x284, 0x2c0):
            at = offset + idx * 4
            self.put(self.decode + at, struct.unpack_from('<I', mmio, at)[0])
        for at in (0x15c + vp * 4, 0x190):  # VP8 / PP local IRQ masks
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
        if self.multi:
            vp = self.engines[idx]
            assert self.live_engines.pop(vp) == idx and not self.word
            self.route(80 + vp * 5)
            return
        assert self.live == idx and not self.word
        self.live = None
        self.events.append(f'AV1 VP DONE {idx}')
        self.route(125)  # VP9 (AV1) unknown/error/DONE: 123/124/125

    def finish_pp(self, idx):
        assert self.pp_owner == idx
        self.pp_owner = None
        self.events.append(f'PP DONE {idx}')
        self.route(64)


def pp_tokens(t):
    return [word for word in t.notifications if word >> 16 == 0x5453]


for blocked in (False, True):
    t = AV1()
    wp = t.submit_host(0, 0)
    t.drain(20)
    assert t.live == 0 and t.pp_owner is None and not pp_tokens(t)
    t.finish_vp(0)
    t.drain(20)
    assert t.pp_owner == 0 and t.live is None and not pp_tokens(t)
    # The host consumes the tagged VP barrier before dispatching this named
    # reference. Its CDF/map inputs come from immutable preparation snapshots.
    wp = t.submit_host(1, wp)
    t.drain(20)
    assert t.live == 1 and t.pp_owner == 0, 'no AV1 VP/PP overlap'
    t.finish_vp(1)
    t.drain(20)
    assert t.pp_owner == 0 and not pp_tokens(t)
    t.busy = blocked
    t.finish_pp(0)
    t.poll(20)
    if blocked:
        assert not pp_tokens(t) and t.result(0) == [1, 0, 0]
        t.busy = False
        t.drain(20)
    assert t.result(0) == [2, 0, 0x10] and pp_tokens(t) == [0x54530010]
    assert t.pp_owner == 1
    t.busy = False
    t.finish_pp(1)
    t.drain(20)
    assert t.result(1) == [2, 0, 0x11]
    assert t.notifications == [0x54540010, 0x54540011, 0x54530010, 0x54530011]
    assert t.u32(0x10007ff8) == t.u32(0x10007ffc) == wp
    assert not t.fetches and not t.ring_writes
    assert t.u32(0x1000803c) == t.u32(0x1000807c) == 1
    print('\n'.join('TRACE ' + e for e in t.events))
    print(f'PASS: AV1 snapshot picture overlap, blocked PP mailbox={blocked}; VP(1) issued before PP(0) DONE')


for blocked in (False, True):
    t = AV1(multi=True)
    wp = 0
    for idx in range(4):
        wp = t.submit_host(idx, wp)
    t.drain(30)
    assert t.live_engines == {9: 0, 10: 1, 11: 2}
    assert t.feed == 3 and t.pp_owner is None
    # DONE on a later engine never starts its PP before predecessor VP DONE.
    for idx in (2, 1):
        t.finish_vp(idx)
        t.drain(10)
        assert t.pp_owner is None and not pp_tokens(t)
    t.finish_vp(0)
    t.drain(20)
    assert t.pp_owner == 0 and t.live_engines == {9: 3}
    t.finish_vp(3)
    t.drain(10)
    assert not t.live_engines and t.pp_owner == 0
    for idx in range(4):
        assert t.pp_owner == idx
        t.busy = blocked
        t.finish_pp(idx)
        t.poll(10)
        if blocked:
            assert t.result(idx) == [1, 0, 0]
            t.busy = False
            t.drain(10)
        assert t.result(idx) == [2, 0, 0x10 | idx]
    assert pp_tokens(t) == [0x54530010 | idx for idx in range(4)]
    assert [word for word in t.notifications if word >> 16 == 0x5454] == [0x54540012, 0x54540011, 0x54540010, 0x54540013]
    assert t.u32(0x10007ff8) == t.u32(0x10007ffc) == wp
    assert not t.fetches and not t.ring_writes
    print(f'PASS: AV1 three-engine dispatch, fourth waits for VP9, reverse VP DONE, ordered PP, blocked mailbox={blocked}')

# A routed error on each advertised AV1 engine must fault every owner; restart
# from reset then proves the next set of production descriptors can run again.
for failed_vp in (9, 10, 11):
    t = AV1(multi=True)
    wp = 0
    for idx in range(3):
        wp = t.submit_host(idx, wp)
    t.drain(20)
    assert len(t.live_engines) == 3
    t.live_engines.clear()
    t.route(79 + failed_vp * 5)  # engine-specific error, not DONE
    t.drain(30)
    assert t.pp_owner is None
    assert all(t.result(idx) == [2, 2, 0x10 | idx] for idx in range(3))
    assert t.notifications == [0x54530010 | idx for idx in range(3)]
    # A fresh CM3 reset boots all three engines after the halt.
    restarted = AV1(multi=True)
    wp = 0
    for idx in range(3):
        wp = restarted.submit_host(idx, wp)
    restarted.drain(20)
    assert restarted.live_engines == {9: 0, 10: 1, 11: 2}
    print(f'PASS: AV1 VP{failed_vp} routed error drains all owners; reset restores all engines')
