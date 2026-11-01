#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Routed CM3 queue test using production host programs, DMA sites and MMIO.

usage: unicorn-inflight-test.py FIRMWARE_TREE IMAGE PROGRAM_DIRECTORY [--before]
The decoder model holds each VP busy until an explicit DONE, checks emitted
addresses, and delays DONE/PP/outbox to exercise overlap. It does not decode
pixels or simulate the decoder's undocumented internal register execution.
"""
import importlib.util
from pathlib import Path
import struct
import sys
from unicorn.arm_const import UC_ARM_REG_SP

firmware, image, programs = map(lambda p: Path(p).resolve(), sys.argv[1:4])
spec = importlib.util.spec_from_file_location('transport', firmware / 'tests/transport-test.py')
transport = importlib.util.module_from_spec(spec)
spec.loader.exec_module(transport)


class LiveVPError(AssertionError):
    pass


class Inflight(transport.Transport):
    def __init__(self):
        self.booting = False
        self.live = None
        self.feed_idx = 0
        self.feed_word = 0
        self.expected = []
        self.events = []
        self.address_checks = 0
        self.pp_owner = None
        self.vp_events = self.pp_events = 0
        super().__init__(image)
        self.booting = True
        self.uc.reg_write(UC_ARM_REG_SP, self.u32(0))
        self.uc.emu_start(self.u32(4), 0, count=500000)
        assert not self.booting, 'reset did not publish boot-ready'
        assert self.u32(0x50010010) & 2
        self.lo = 0x150 if self.version == 3 else 0x1d0
        self.hi = None if self.version == 3 else 0x20c
        self.mask = 0x18c if self.version == 3 else 0x248
        self.cach = 0x1c8 if self.version == 3 else 0x284
        self.xfer = 0x204 if self.version == 3 else 0x2c0

    def route(self, irq):
        if irq < 14:
            router = self.u32(0x50010010) & (1 << irq)
        else:
            off = irq - 14
            router = self.u32(0x50010014 + (off // 32) * 4) & (1 << (off % 32))
        nvic = self.u32(0xe000e100 + (irq // 32) * 4) & (1 << (irq % 32))
        assert router and nvic, f'masked IRQ{irq}'
        self.call(f'irq{irq}')

    def fifo_addr(self, idx):
        low = self.u32(self.decode + self.lo + idx * 4)
        addr = low << 8 if self.hi is None else low | self.u32(self.decode + self.hi + idx * 4) << 32
        assert addr, f'zero FIFO DMA address at slot {idx}'
        self.address_checks += 1
        return addr

    def write(self, uc, access, addr, size, value, user):
        if self.booting and addr == 0x50010090 and value == 1:
            self.booting = False
            uc.emu_stop()
        if self.decode + self.fifo <= addr < self.decode + self.fifo + self.vps * 4:
            idx, words = self.expected[self.feed_idx]
            assert addr == self.decode + self.fifo + 4 * 4, 'wrong H.264 VP'
            assert value == words[self.feed_word], (idx, self.feed_word, hex(value))
            if self.feed_word == 0:
                if self.live is not None:
                    raise LiveVPError(f'VP4 receives slot {idx} START while slot {self.live} has no DONE')
                self.live = idx
                assert (value >> 4) & 15 == idx
                fifo = self.fifo_addr(idx)
                self.events.append(f'VP4 START slot={idx} fifo={fifo:#x}')
            self.feed_word += 1
            if self.feed_word == len(words):
                self.feed_idx += 1
                self.feed_word = 0
        if addr == self.decode + self.pp:
            idx = (value >> 4) & 15
            assert self.pp_owner is None, 'PP overwritten before DONE'
            assert value == 0x2b000200 | idx << 4 | self.info[10]
            self.pp_owner = idx
            fifo = self.fifo_addr(idx)
            self.events.append(f'PP START slot={idx} fifo={fifo:#x}')
        if addr == 0x50010060:
            self.events.append(f'completion token={value:#x}')
        super().write(uc, access, addr, size, value, user)

    def submit_host(self, idx, start, tag):
        variant = image.stem.removeprefix('avd-fw-')
        source = programs / variant
        data = (source / f'descriptor-{idx}.bin').read_bytes()
        desc = list(struct.unpack('<16I', data))
        words = list(struct.unpack('<' + 'I' * desc[6], (source / f'h264-{idx}.bin').read_bytes()))
        assert desc[:4] == [0x41565131, 0x10 | idx, 1, 4]
        assert desc[4:6] == [0, 0] and desc[8:12] == [1, 0, 0, 1]
        assert desc[12:] == [0, 0, 0, 0]
        assert desc[7] == 0x2b000200 | idx << 4 | self.info[10]
        # program_lo is a free-running SRAM word count, not a DMA pointer.
        desc[4] = start
        desc[1] = tag
        assert words[0] == 0x2b000200 | idx << 4
        assert words[-1] == 0x2b000400 | idx << 4
        for line in (source / f'dma-sites-{idx}.txt').read_text().splitlines():
            offset, width, shift, expected = line.split()
            offset, width, shift, expected = int(offset), int(width), int(shift), int(expected, 16)
            encoded = sum(words[offset + n] << (32 * n) for n in range(width))
            assert encoded and encoded == expected >> shift, f'zero/truncated DMA operand slot {idx} word {offset}'
            self.address_checks += 1
        for n, word in enumerate(words[:-1]):
            if word >> 20 == 0x2d8:  # coded-slice address: high 13 bits + next low word
                assert ((word & 0x1fff) << 32) | words[n + 1], f'zero coded-data DMA slot {idx}'
                self.address_checks += 1
        snapshot = (source / f'mmio-{idx}.bin').read_bytes()
        # Replay only this host submission's writes, preserving live slots.
        offsets = [self.lo, self.mask, self.cach, self.xfer]
        if self.hi is not None:
            offsets.append(self.hi)
        for off in offsets:
            at = off + idx * 4
            self.put(self.decode + at, struct.unpack_from('<I', snapshot, at)[0])
        for at in ([0x10c, 0x120] if self.version == 3 else [0x16c, 0x190]):
            self.put(self.decode + at, struct.unpack_from('<I', snapshot, at)[0])
        self.fifo_addr(idx)
        self.expected.append((idx, words))
        self.uc.mem_write(0x10008000 + idx * 64, struct.pack('<16I', *desc))
        transport.ring_write(self, start, words)
        self.put(0x10007ffc, start + len(words))
        self.put(0x50010058, self.ring + idx * 64)
        self.events.append(f'doorbell slot={idx} tag={tag:#x} rp_start={start} words={len(words)}')
        self.route(1)
        return start + len(words)

    def finish_vp(self):
        assert self.live is not None and not self.feed_word
        self.events.append(f'VP4 DONE slot={self.live}')
        self.live = None
        self.vp_events += 1
        self.route(100)

    def finish_pp(self):
        assert self.pp_owner is not None
        self.events.append(f'PP DONE slot={self.pp_owner}')
        self.pp_owner = None
        self.pp_events += 1
        self.route(64)


def check(depth):
    t = Inflight()
    wp = submitted = retired = peak = 0
    tags = []
    slots = t.info[10]
    # T8140 advertises seven physical FIFOs. Workloads 8..15 exercise a
    # seven-slot window and reuse; do not fabricate absent hardware FIFOs.
    while retired < depth:
        while submitted < depth and submitted - retired < slots:
            tag = ((1 + submitted // slots) << 4) | (submitted % slots)
            tags.append(tag)
            wp = t.submit_host(submitted % slots, wp, tag)
            submitted += 1
            peak = max(peak, submitted - retired)
        t.poll(30)
        if t.pp_owner is not None:
            # Hold PP and mailbox busy while the next VP runs to completion.
            if t.live is not None:
                t.finish_vp()
                t.poll(10)
            t.finish_pp()
            t.busy = True
            t.poll(10)
            t.busy = False
            t.poll(10)
        elif t.live is not None:
            t.finish_vp()
            t.poll(10)
        if len(t.notifications) > retired:
            idx = retired % slots
            assert t.result(idx) == [2, 0, tags[retired]]
            assert t.notifications[retired] == 0x54530000 | tags[retired]
            retired += 1
            t.busy = False
    assert t.vp_events == t.pp_events == depth
    assert t.feed_idx == depth and not t.feed_word and t.live is t.pp_owner is None
    assert t.u32(0x10007ff8) == t.u32(0x10007ffc) == wp
    assert not t.fetches and not t.ring_writes
    if depth in (2, 15):
        print('\n'.join('TRACE ' + event for event in t.events), flush=True)
    print(f'PASS {image.stem}: workload {depth}, peak {peak}, {t.address_checks} nonzero DMA checks, '
          f'{depth} routed VP/PP completions, RP=WP={wp}', flush=True)


if '--before' in sys.argv[4:]:
    try:
        check(2)
    except LiveVPError as error:
        print(f'REPRODUCED {image.stem}: {error}', flush=True)
    else:
        raise AssertionError('baseline unexpectedly respected VP ownership')
else:
    for depth in range(2, 16):
        check(depth)
