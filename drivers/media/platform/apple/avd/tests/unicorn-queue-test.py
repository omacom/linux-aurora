#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run codec-generated Linux descriptors/programs through actual CM3 code.

Usage: unicorn-queue-test.py FIRMWARE_TREE IMAGE PROGRAM_DIRECTORY [--broken]
The firmware tree provides its existing tests/transport-test.py MMIO harness.
Extend that harness with reset-vector boot, real FIFO status bits, IRQ router
and NVIC checks, and VP/PP events following a complete generated program.
Decoder arithmetic/DMA/pixels remain synthetic; this is a transport regression.
--broken proves the old image stalls both at IRQ routing and at publication.
"""
import importlib.util
from pathlib import Path
import struct
import sys

from unicorn.arm_const import UC_ARM_REG_SP

firmware = Path(sys.argv[1]).resolve()
image = Path(sys.argv[2]).resolve()
programs = Path(sys.argv[3]).resolve()
broken = '--broken' in sys.argv[4:]
spec = importlib.util.spec_from_file_location('transport', firmware / 'tests/transport-test.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RoutedTransport(module.Transport):
    def __init__(self, image):
        self.booting = False
        self.suppressed = set()
        super().__init__(image)
        self.booting = True
        self.uc.reg_write(UC_ARM_REG_SP, self.u32(0))
        self.uc.emu_start(self.u32(4), 0, count=500000)
        assert not self.booting, 'reset vector did not publish boot-ready'
        assert self.u32(0x50010010) & 2, 'CM3 mailbox-0 not-empty disabled'
        # Linux avd_boot() AP mailbox-1 notification routing, separate from CM3.
        self.ap_irq_enable = 8

    def read(self, uc, access, addr, size, value, user):
        super().read(uc, access, addr, size, value, user)
        if addr == 0x5001005c:
            # BIT(2) belongs to IRQ status, not this FIFO status register.
            # Empty FIFO has bit 17 set, ENABLE remains bit 0.
            self.put(addr, 1 | (0x10100 if self.busy else 1 << 17))

    def write(self, uc, access, addr, size, value, user):
        super().write(uc, access, addr, size, value, user)
        if self.booting and addr == 0x50010090 and value == 1:
            self.booting = False
            uc.emu_stop()
        if addr == 0x50010060:
            assert self.ap_irq_enable == 8, 'AP mailbox-1 IRQ disabled'

    def route(self, irq):
        if irq < 14:
            router = self.u32(0x50010010) & (1 << irq)
        else:
            offset = irq - 14
            router = self.u32(0x50010014 + (offset // 32) * 4) & (1 << (offset % 32))
        nvic = self.u32(0xe000e100 + (irq // 32) * 4) & (1 << (irq % 32))
        if not (router and nvic):
            self.suppressed.add(irq)
            return False
        self.call('irq' + str(irq))
        return True

    def descriptor(self, idx):
        data = (programs / f'descriptor-{idx}.bin').read_bytes()
        d = struct.unpack('<16I', data)
        assert d[0:4] == (0x41565131, 0x10 | idx, 1, 4)
        assert d[4:6] == (0, 0) and d[8:12] == (1, 0, 0, 1)
        assert d[7] == 0x2b00020f | (idx << 4)
        assert d[12:] == (0, 0, 0, 0)
        self.uc.mem_write(0x10008000 + idx * 64, data)
        # Actual Linux t8112_configure_stream() results are asserted by program-test.
        self.put(self.decode + 0x10c, 7)   # VP4 decoder-local mask
        self.put(self.decode + 0x120, 5)   # PP decoder-local mask
        self.put(0x50010058, self.ring + idx * 64)
        assert self.route(1), 'AP mailbox-0 doorbell was not delivered'
        words = list(struct.unpack('<' + 'I' * d[6], (programs / f'h264-{idx}.bin').read_bytes()))
        assert words[0] == 0x2b000200 | (idx << 4)
        assert words[-1] == 0x2b000400 | (idx << 4)
        module.ring_write(self, 0, words)
        self.put(0x10007ffc, len(words))
        return words

    def execute(self, words):
        vp_done = pp_done = False
        for _ in range(40):
            self.poll()
            if not vp_done and self.stores == words:
                # Generate DONE only after the complete real codec program was fed.
                assert self.u32(self.decode + 0x10c) & 4
                vp_done = self.route(100)  # v3 H.264 VP4 DONE
            if self.kicks and not pp_done:
                assert self.u32(self.decode + 0x120) & 4
                pp_done = self.route(64)
        assert self.stores == words
        assert self.u32(0x10007ff8) == self.u32(0x10007ffc) == len(words)
        assert not self.fetches and not self.ring_writes
        return vp_done, pp_done


def force_decoder_routes(t):
    # Isolate the independent status-register bug in the unfixed image.
    for irq in (100, 64):
        offset = irq - 14
        addr = 0x50010014 + (offset // 32) * 4
        t.put(addr, t.u32(addr) | (1 << (offset % 32)))


assert image.stem == 'avd-fw-v3-t1', 'this first-light regression targets T8112'
if broken:
    t = RoutedTransport(image)
    words = t.descriptor(0)
    assert t.execute(words) == (False, False)
    assert 100 in t.suppressed and t.result(0) == [1, 0, 0] and not t.notifications
    print('REPRODUCED: v3 VP4 IRQ100 masked; program consumed but descriptor remains SUBMITTED')
    t = RoutedTransport(image)
    words = t.descriptor(0)
    force_decoder_routes(t)
    assert t.execute(words) == (True, True)
    assert t.kicks == [0x2b00020f]
    assert t.result(0) == [1, 0, 0] and not t.notifications
    print('REPRODUCED: VP/PP done with routes repaired; EMPTY bit17 never passes firmware BIT(2) test')
else:
    for idx in (0, 1, 14):
        t = RoutedTransport(image)
        words = t.descriptor(idx)
        assert t.execute(words) == (True, True)
        assert t.u32(0x50010018) == 0x50000  # PP unknown/DONE
        assert t.u32(0x5001001c) == 0x700000  # VP4 unknown/error/DONE only
        assert all(t.u32(a) == 0 for a in (0x50010014, 0x50010020, 0x50010024, 0x50010028))
        assert t.kicks == [0x2b00020f | (idx << 4)]
        assert t.result(idx) == [2, 0, 0x10 | idx]
        assert t.notifications == [0x54530010 | idx]
        print(f'PASS: v3-t1 slot {idx}, {len(words)} production H.264 words, '
              f'IRQ1 -> VP4 IRQ100 -> PP IRQ64 -> tagged completion {t.notifications[0]:#x}')
