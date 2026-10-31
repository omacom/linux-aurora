#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""ADT-vs-DT checker for the Apple T6032 (M3 Ultra) j575d device tree.

Compares the built t6032-j575d DTB(s) against the j575dap ADT of the pinned
macOS 27.0.1 (26A434) firmware so the fixes that were made against that ADT
cannot silently regress:

  - every ADT node the board needs (six USB-C controllers, their DARTs and
    phys, the GPU and its ASC mailbox, the eight external display pipes with
    their DARTs and mailboxes, both ANE engines with their DARTs, and the
    console devices AOP/ANS/SMC/SART/UART) must be covered by a DT node whose
    register windows overlap it, after die-1 bus translation;
  - nodes inherited from the laptop descriptions that the ADT does not have
    (the MTP stack, the AOP audio DMA, i2c3, the SPI-NOR, and the phantom
    0x2f_ die-1 mirrors) must not exist;
  - every DT SoC node with registers must overlap an ADT node, except the
    die-1 cpufreq and pmgr mirrors (hardware mirrors at the die stride that
    the ADT expresses inside the pmgr register-window list, not as nodes);
  - the power states the new nodes reference must exist in the DT at the
    bank + offset listed in REQUIRED_PS (those offsets were read from the ADT
    pmgr by hand; the script does not read them from the ADT), in both dies;
  - the GPU, ANE and display nodes must stay disabled: a DT node is such a
    gated node when its first register window starts where a window of a gated
    ADT node starts (a DART: any window), so the test needs no DT labels, and
    fewer matching DT nodes than gated ADT nodes is a failure.

Expected, not a checker failure: dtc warns simple_bus_reg for the die-1 ANE
node ane@23c9400000 (its reg is 0x3c9400000, the die-local address).

Usage:
    adt-dt-check-t6032.py ADT DTB [DTB...]
    adt-dt-check-t6032.py --fetch-ipsw DTB [DTB...]
        Fetch the pinned DeviceTree.j575dap.im4p by HTTP range requests from
        the pinned 27.0.1 IPSW (no full download), verify both pinned SHA256
        hashes, LZFSE-decompress it to a temp file and check against it.

Needs dtc on PATH for DTB decompilation; --fetch-ipsw additionally needs
pyliblzfse. Prints one line per check; the exit status is 1 if any check
fails. Nothing is written (except the temp ADT with --fetch-ipsw).
"""

import hashlib
import re
import struct
import subprocess
import sys
import tempfile
import urllib.request
import zlib

# macOS 27.0.1 (26A434) for Mac15,14; metadata from api.ipsw.me 2026-09-28.
IPSW_URL = ("https://updates.cdn-apple.com/2026FallFCS/59241290-5d51-4ca8-9df4-31624b9a4eac/"
            "UniversalMac_27.0.1_26A434_Restore.ipsw")
IPSW_SHA256 = "2f016638293c3e641b8b25391a76fbc16563b3711915a5551cf8aa0f5598a5c1"
IM4P_SHA256 = "6c3ca4ac7272ecf9c05f6b2b413ef923ff2d3ff252469493fc700260b6938c60"
ADT_SHA256 = "4a818ecc7f3d71195c1def12c6eed0c90b287f82cad2a6a4ffd6f0849fe13c12"
IM4P_MEMBER = "Firmware/all_flash/DeviceTree.j575dap.im4p"

# m1n1 src/pmgr.h: PMGR_DIE_OFFSET.
PMGR_DIE_OFFSET = 0x2000000000

failures = []


def check(ok, line):
    print(("ok   " if ok else "FAIL ") + line)
    if not ok:
        failures.append(line)


# ---------------------------------------------------------------- ADT side

def adt_parse(b, off=0):
    nprops, nchild = struct.unpack_from("<II", b, off)
    off += 8
    props = {}
    for _ in range(nprops):
        name = b[off:off + 32].split(b"\0")[0].decode("latin1")
        size = struct.unpack_from("<I", b, off + 32)[0] & 0x7FFFFFFF
        off += 36
        props[name] = b[off:off + size]
        off += (size + 3) & ~3
    kids = []
    for _ in range(nchild):
        kid, off = adt_parse(b, off)
        kids.append(kid)
    return {"props": props, "kids": kids}, off


def adt_name(n):
    return n["props"].get("name", b"").split(b"\0")[0].decode()


def adt_find(node, *path):
    for want in path:
        node = next((k for k in node["kids"] if adt_name(k) == want), None)
        if node is None:
            return None
    return node


def adt_reg(node):
    r = node["props"].get("reg")
    if r is None:
        return []
    return [struct.unpack_from("<QQ", r, 16 * i) for i in range(len(r) // 16)]


def adt_ranges(node):
    r = node["props"].get("ranges")
    if r is None:
        return []
    return [struct.unpack_from("<QQQ", r, 24 * i) for i in range(len(r) // 24)]


def adt_translate(ranges, addr):
    for child, parent, size in ranges:
        if child <= addr < child + size:
            return parent + (addr - child)
    return addr


def adt_windows(root):
    """All physical register windows the ADT describes: every /arm-io
    descendant node window and every /arm-io/pmgr reg window."""
    io = adt_find(root, "arm-io")
    if io is None:
        sys.exit("no /arm-io in the ADT")
    rng = adt_ranges(io)
    windows = {}

    def walk(n):
        for addr, size in adt_reg(n):
            if size:
                windows[adt_translate(rng, addr)] = size
        for k in n["kids"]:
            walk(k)

    walk(io)
    pmgr = adt_find(io, "pmgr")
    if pmgr is not None:
        for addr, size in adt_reg(pmgr):
            if size:
                windows[adt_translate(rng, addr)] = size
    return windows


def adt_gated_nodes(root, names):
    io = adt_find(root, "arm-io")
    rng = adt_ranges(io)
    out = {}
    for n in io["kids"]:
        if adt_name(n) in names:
            reg = adt_reg(n)
            out[adt_name(n)] = [(adt_translate(rng, a), s) for a, s in reg if s]
    return out


# ------------------------------------------------------------------ DT side

def dtb_decompile(path):
    txt = subprocess.run(["dtc", "-I", "dtb", "-O", "dts", path],
                         capture_output=True, text=True, check=True).stdout
    # strip comments so braces in comments cannot confuse the brace walker
    txt = re.sub(r"/\*.*?\*/", "", txt, flags=re.S)
    txt = re.sub(r"(?m)//.*$", "", txt)
    return txt


class DNode:
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.props = {}
        self.kids = []


def dt_parse_tree(txt):
    """Parse the decompiled DTS into a tree. Node openers are lines ending
    with '{' (their first token is the node name), property lines end with
    ';'."""
    root = DNode("/", None)
    stack = [root]
    for raw in txt.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.endswith("{"):
            name = line[:-1].strip().split(":")[-1].strip()
            if name == "/" and stack[-1] is root:
                continue  # the root opener; 'root' already is '/'
            n = DNode(name, stack[-1])
            stack[-1].kids.append(n)
            stack.append(n)
        elif line in ("}", "};"):
            stack.pop()
        elif line.endswith(";"):
            body = line[:-1].strip()
            m = re.match(r"([A-Za-z0-9,-]+)[ \t]*(?:=(.*))?$", body, re.S)
            if m:
                stack[-1].props[m.group(1)] = (m.group(2) or "").strip()
    return root


def dt_cells(val):
    out = []
    for grp in re.findall(r"<([0-9a-fxA-FX \t]+)>", val or ""):
        for x in grp.split():
            out.append(int(x, 16) if x.lower().startswith("0x") else int(x))
    return out


def adt_irqs(node):
    """(all_irqs, valid_irqs) of an ADT node, as (die, irq) pairs. The ADT
    encodes die 1 by adding 0x1000 to the AIC number. 'interrupts-valid' is
    a bitmask over the 'interrupts' entries; entries whose bit is clear are
    marked invalid by the firmware (sgx entry 5, IRQ 1249). Without the
    property every entry counts as valid."""
    pr = node["props"]
    raw = pr.get("interrupts")
    if raw is None:
        return [], []
    all_irqs = [(1 if n >= 0x1000 else 0, n - 0x1000 if n >= 0x1000 else n)
                for n in (int.from_bytes(raw[i:i + 4], "little")
                          for i in range(0, len(raw), 4))]
    mask = pr.get("interrupts-valid")
    if mask is None:
        return all_irqs, list(all_irqs)
    m = int.from_bytes(mask, "little")
    valid = [irq for i, irq in enumerate(all_irqs) if m & (1 << i)]
    return all_irqs, valid


def adt_irq_tables(root, names):
    io = adt_find(root, "arm-io")
    out = {}
    for n in io["kids"]:
        if adt_name(n) in names:
            out[adt_name(n)] = adt_irqs(n)
    return out


def dt_irqs(node):
    """(die, irq) pairs of a DT node: 4-cell `interrupts` specs
    (<AIC_IRQ die irq type>) and 5-cell `interrupts-extended` specs
    (<&aic AIC_IRQ die irq type>). Every interrupt-parent here is the AIC,
    so the fixed grouping is safe for this board; nodes whose
    interrupts-extended has irregular (<0> filler) specs are skipped, as
    the spec length is then not derivable (e.g. the ADMAC channel list)."""
    out = []
    c = dt_cells(node.props.get("interrupts", ""))
    if len(c) % 4 == 0:
        out += [(c[i + 1], c[i + 2]) for i in range(0, len(c), 4)]
    c = dt_cells(node.props.get("interrupts-extended", ""))
    if c and len(c) % 5 == 0:
        out += [(c[i + 2], c[i + 3]) for i in range(0, len(c), 5)]
    return out


def dt_strings(val):
    return re.findall(r'"([^"]*)"', val or "")


def dt_soc_nodes(root):
    """Yield (path, node, windows) for every DIRECT child of either SoC bus
    that has a reg property, windows translated to physical addresses.
    Deeper buses (pmgr states, DCP children, CIO children) use other
    address-cell widths and are deliberately not covered here; the ADT-vs-DT
    register comparison is on SoC-level devices."""
    for soc in root.kids:
        if not soc.name.startswith("soc@"):
            continue
        rc = dt_cells(soc.props.get("ranges", ""))
        rng = [((rc[i] << 32) | rc[i + 1],
                (rc[i + 2] << 32) | rc[i + 3],
                (rc[i + 4] << 32) | rc[i + 5]) for i in range(0, len(rc) - 5, 6)]

        def tr(a):
            for child, parent, size in rng:
                if child <= a < child + size:
                    return parent + (a - child)
            return a

        for k in soc.kids:
            if "reg" not in k.props:
                continue
            c = dt_cells(k.props["reg"])
            wins = []
            if len(c) % 4 == 0 and c:
                for i in range(0, len(c), 4):
                    wins.append((tr((c[i] << 32) | c[i + 1]),
                                 (c[i + 2] << 32) | c[i + 3]))
            yield "/" + soc.name + "/" + k.name, k, wins


def overlap(w1, w2):
    return w1[0] < w2[0] + w2[1] and w2[0] < w1[0] + w1[1]


# ------------------------------------------------------------------- fetch

def http_range(url, start, end):
    req = urllib.request.Request(url, headers={"Range": f"bytes={start}-{end}"})
    with urllib.request.urlopen(req, timeout=120) as r:
        return r.read()


def fetch_adt():
    """Pull DeviceTree.j575dap.im4p out of the pinned IPSW with two range
    requests (central directory, then the member) and LZFSE-decompress it.
    Both pinned SHA256 hashes must match."""
    import lzfse  # pyliblzfse
    req = urllib.request.Request(IPSW_URL, headers={"Range": "bytes=0-0"})
    with urllib.request.urlopen(req, timeout=120) as r:
        cr = r.headers.get("Content-Range", "")
        total = int(cr.rsplit("/", 1)[-1])
    cd = http_range(IPSW_URL, max(0, total - (1 << 20)), total - 1)
    eocd = cd.rfind(b"PK\x05\x06")
    if eocd < 0:
        sys.exit("no zip End Of Central Directory in the IPSW tail")
    count, cdoff = struct.unpack_from("<H", cd, eocd + 10)[0], struct.unpack_from("<I", cd, eocd + 16)[0]
    # ZIP64: the 26.6 GB image keeps the real values in the ZIP64 EOCD and
    # its locator, which sit immediately before the classic EOCD.
    if cdoff == 0xFFFFFFFF or count == 0xFFFF:
        loc = eocd - 20
        if cd[loc:loc + 4] != b"PK\x06\x07":
            sys.exit("no zip64 central-directory locator in the IPSW tail")
        z64off = struct.unpack_from("<Q", cd, loc + 8)[0]
        z64 = http_range(IPSW_URL, z64off, z64off + 56 - 1)
        if z64[:4] != b"PK\x06\x06":
            sys.exit("no zip64 EOCD record at the locator target")
        count = struct.unpack_from("<Q", z64, 32)[0]
        cdoff = struct.unpack_from("<Q", z64, 48)[0]
    cdsize = 1 << 20
    cdbytes = http_range(IPSW_URL, cdoff, cdoff + cdsize - 1)
    pos, entry = 0, None
    for _ in range(count):
        if cdbytes[pos:pos + 4] != b"PK\x01\x02":
            break
        method = struct.unpack_from("<H", cdbytes, pos + 10)[0]
        csize = struct.unpack_from("<I", cdbytes, pos + 20)[0]
        nlen, elen, clen = struct.unpack_from("<HHH", cdbytes, pos + 28)
        lhoff = struct.unpack_from("<I", cdbytes, pos + 42)[0]
        fname = cdbytes[pos + 46:pos + 46 + nlen]
        if fname.decode() == IM4P_MEMBER:
            # zip64: the real offset lives in the 0x0001 extra field
            extra = cdbytes[pos + 46 + nlen:pos + 46 + nlen + elen]
            epos = 0
            while epos + 4 <= len(extra):
                eid, esz = struct.unpack_from("<HH", extra, epos)
                if eid == 1:
                    fields = extra[epos + 4:epos + 4 + esz]
                    fpos = 0
                    if lhoff == 0xFFFFFFFF and fpos + 8 <= len(fields):
                        lhoff = struct.unpack_from("<Q", fields, fpos)[0]
                        fpos += 8
                    break
                epos += 4 + esz
            entry = (method, csize, lhoff, nlen, elen)
            break
        pos += 46 + nlen + elen + clen
    if entry is None:
        sys.exit(f"{IM4P_MEMBER} not in the IPSW central directory")
    method, csize, lhoff, nlen, elen = entry
    head = http_range(IPSW_URL, lhoff, lhoff + 30 - 1)
    if head[:4] != b"PK\x03\x04":
        sys.exit("bad local header for the DeviceTree member")
    nlen2, elen2 = struct.unpack_from("<HH", head, 26)
    # Sizes come from the central directory: the local header may carry a
    # data descriptor (streamed entries) with a zero size.
    data = http_range(IPSW_URL, lhoff + 30 + nlen2 + elen2,
                      lhoff + 30 + nlen2 + elen2 + csize - 1)
    if method == 8:
        data = zlib.decompress(data, -15)
    elif method != 0:
        sys.exit(f"unsupported zip method {method} for the DeviceTree member")
    if hashlib.sha256(data).hexdigest() != IM4P_SHA256:
        sys.exit("DeviceTree.j575dap.im4p sha256 mismatch against the pin")
    # minimal IM4P walk: SEQUENCE { IA5 'IM4P', IA5 type, IA5 version, OCTET payload }
    def der_tlv(b, p):
        t = b[p]
        l = b[p + 1]
        p += 2
        if l & 0x80:
            n = l & 0x7F
            l = int.from_bytes(b[p:p + n], "big")
            p += n
        return t, l, p
    t, l, p = der_tlv(data, 0)
    end = p + l
    fields = []
    while p < end:
        t, l, q = der_tlv(data, p)
        fields.append(data[q:q + l])
        p = q + l
    payload = fields[3]
    if payload[:4] == b"bvx2":
        payload = lzfse.decompress(payload)
    elif payload[:4] != b"bvx$":
        pass  # stored ADT
    if hashlib.sha256(payload).hexdigest() != ADT_SHA256:
        sys.exit("decompressed j575dap ADT sha256 mismatch against the pin")
    tmp = tempfile.NamedTemporaryFile(suffix=".adt", delete=False)
    tmp.write(payload)
    tmp.close()
    print(f"ok   fetched and verified the 27.0.1 (26A434) j575dap ADT "
          f"({len(payload)} bytes, sha256 {ADT_SHA256[:12]}...)")
    return tmp.name


# ------------------------------------------------------------------ checks

REQUIRED = (
    ["usb-drd%d" % i for i in range(6)]
    + ["atc-phy%d" % i for i in range(6)]
    + ["dart-usb%d" % i for i in range(6)]
    + ["ane0", "ane1", "dart-ane0", "dart-ane1"]
    + ["sgx", "gfx-asc"]
    + ["dcpext%d" % i for i in range(8)]
    + ["dart-dcpext%d" % i for i in range(8)]
    + ["dart-dispext%d" % i for i in range(8)]
    + ["dispext%d" % i for i in range(8)]
    + ["aop", "ans", "sart-ans", "smc", "uart0"]
)

# Physical windows the tree must NOT touch: the phantom die-1 mirrors of
# die-0 port 2, and the laptop-only devices absent from the j575dap ADT.
# (base, size, what)
DENIED = [
    (0x2F01108000, 0x4000, "usb4_2 mailbox mirror (die 1)"),
    (0x2F01AC0000, 0xC000, "usb4_2 acio mirror (die 1)"),
    (0x2F02280000, 0x10000, "usb controller mirror (die 1)"),
    (0x2F02F00000, 0x4000, "usb dart mirror (die 1)"),
    (0x2F02F80000, 0x4000, "usb dart mirror (die 1)"),
    (0x2F03000000, 0x4C000, "atc-phy mirror (die 1)"),
    (0x2AC400000, 0x4000, "MTP"),
    (0x2AC408000, 0x4000, "MTP mailbox"),
    (0x2AC808000, 0x4000, "MTP DART"),
    (0x2ACB14000, 0x4000, "MTP dockchannel fifo"),
    (0x2A8980000, 0x34000, "AOP audio ADMAC"),
    (0x39101C000, 0x4000, "i2c3"),
    (0x391104000, 0x4000, "SPI-NOR (spi1)"),
]

# power states the new nodes reference: (soc, pmgr node, offset, what for)
REQUIRED_PS = []
for soc in ("soc@200000000", "soc@2200000000"):
    sfx = "_die1" if soc.endswith("2200000000") else ""
    for i in range(4):
        off = {0: 0x4A8, 1: 0x4F8, 2: 0x508, 3: 0x488}[i]
        REQUIRED_PS.append((soc, "power-management@292800000", off,
                            f"ps_dispext{i}_cpu{sfx}"))
    for i in range(3):
        off = {0: 0x140, 1: 0x148, 2: 0x150}[i]
        REQUIRED_PS.append((soc, "power-management@2a0280000", off,
                            f"ps_atc{i}_usb{sfx}"))
    for off, nm in ((0x520, "sys"), (0x5A0, "mpm"), (0x5A8, "cpu"),
                    (0x5B8, "td"), (0x5C0, "base")):
        REQUIRED_PS.append((soc, "power-management@292280000", off,
                            f"ps_ane_{nm}{sfx}"))
REQUIRED_PS.append(("soc@2200000000", "power-management@2a0280000", 0x158, "ps_atc3_usb"))
REQUIRED_PS.append(("soc@200000000", "power-management@408e80000", 0x110, "ps_gfx"))
REQUIRED_PS.append(("soc@2200000000", "power-management@408e80000", 0x110, "ps_gfx_die1"))

# ADT nodes whose interrupts the DT deliberately does not carry on the node
# that covers their registers. Each display pipe (dispextN) shares its
# register windows with dcpextN, which carries no interrupts of its own:
# the ADT pipe interrupts are carried by the mailbox and DART nodes, and
# those are checked against their own ADT nodes (dart-dcpextN, dart-
# dispextN, dcpextN).
IRQ_COVERAGE_EXEMPT = {f"dispext{i}" for i in range(8)}

# Individual (die, irq) values the DT carries deliberately although no ADT
# device lists them, each with its reason:
#   (0, 1321): the DT models the T6031 four-cluster audio block in one mca
#     node; the 27.0.1 j575dap ADT lists three MCA instances (mca0-2, irqs
#     1318-1320) and has no fourth instance.
DT_IRQ_ALLOW = {
    (0, 1321),
}

# ADT nodes the DT must keep disabled: GPU and its ASC, both ANE engines and
# their DARTs, the eight external display pipes with their DARTs. The DT
# nodes are found by register window, not by name or label.
GATED_ADT = (["sgx", "gfx-asc", "ane0", "ane1", "dart-ane0", "dart-ane1"]
             + [f"{p}{i}" for i in range(8) for p in ("dcpext", "dart-dcpext", "dart-dispext")])


def main():
    args = sys.argv[1:]
    fetch = False
    if args and args[0] == "--fetch-ipsw":
        fetch = True
        args = args[1:]
    if not args:
        sys.exit(__doc__)
    dtbs = args[1:] if not fetch else args
    if fetch:
        adt_path = fetch_adt()
    else:
        adt_path, dtbs = args[0], args[1:]
    if not dtbs:
        sys.exit("no DTB given")

    root, _ = adt_parse(open(adt_path, "rb").read())
    awindows = adt_windows(root)
    gated = adt_gated_nodes(root, set(REQUIRED))
    airqs = adt_irq_tables(root, set(REQUIRED))
    # every /arm-io node with registers: windows + full irq set, for the
    # per-DT-node subset rule
    io = adt_find(root, "arm-io")
    io_rng = adt_ranges(io)
    adt_all = []
    for n in io["kids"]:
        wins = [(adt_translate(io_rng, a), s) for a, s in adt_reg(n) if s]
        if wins:
            adt_all.append((adt_name(n), wins, adt_irqs(n)[0]))

    for dtb in dtbs:
        print(f"== {dtb} ==")
        tree = dt_parse_tree(dtb_decompile(dtb))
        dtnodes = list(dt_soc_nodes(tree))
        bybase = {}
        for path, node, wins in dtnodes:
            bybase.setdefault(node.name.split("@")[0] + "@", []).append((path, node, wins))

        # 1. required ADT nodes covered
        for name in REQUIRED:
            wins = gated.get(name)
            if not wins:
                check(False, f"{name}: absent from the ADT")
                continue
            hit = any(overlap(w, dw) for _, _, dwins in dtnodes for dw in dwins
                      for w in wins)
            check(hit, f"{name} ({', '.join(hex(a) for a, _ in wins[:1])}...): "
                       f"covered by the DT" if hit else
                  f"{name} ({hex(wins[0][0])}): no DT node overlaps it")

        # 2. denied windows untouched
        for base, size, what in DENIED:
            bad = [path for path, _, wins in dtnodes
                   if any(overlap(w, (base, size)) for w in wins)]
            check(not bad, f"no node at {base:x} ({what})"
                  if not bad else f"{what}: present at {bad}")

        # 3. every DT node with registers maps into the ADT
        for path, node, wins in dtnodes:
            if not wins:
                continue
            if any(overlap(w, (a, s)) for w in wins for a, s in awindows.items()):
                continue
            name = node.name
            if name.startswith(("cpufreq@", "power-management@")):
                print(f"ok   {path}: die mirror of a register file (no ADT node, allowed)")
                continue
            check(False, f"{path} ({hex(wins[0][0])}): no ADT node overlaps it")

        # 4. power states exist in the DT at the REQUIRED_PS offsets
        psidx = {}
        stack = [("", tree)]
        while stack:
            path, n = stack.pop()
            for k in n.kids:
                kp = path + "/" + k.name
                parts = kp.split("/")
                if len(parts) == 4 and parts[2].startswith("power-management@") \
                        and parts[3].startswith("power-controller@"):
                    psidx.setdefault((parts[1], parts[2]), {})[parts[3]] = k
                stack.append((kp, k))
        for soc, pmgr, off, label in REQUIRED_PS:
            kids = psidx.get((soc, pmgr), {})
            node = kids.get(f"power-controller@{off:x}")
            if node is None:
                check(False, f"{label}: no {pmgr}/power-controller@{off:x} in {soc}")
                continue
            c = dt_cells(node.props.get("reg", ""))
            got = c[0] if c else None
            check(got == off, f"{label}: at {pmgr}+{off:#x}"
                  if got == off else
                  f"{label}: {pmgr}/power-controller@{off:x} has reg offset {got:#x}, ADT says {off:#x}")

        # 5. interrupts: every DT interrupt must appear in the ADT irq set
        # of a node its windows overlap (order differences like the
        # gfx-asc mailbox reorder are handled by set comparison), and
        # every required ADT node with valid interrupts must see at least
        # one of them carried by a DT node (IRQ_COVERAGE_EXEMPT excepted).
        matched_irqs = {name: [] for name in REQUIRED}
        for path, node, wins in dtnodes:
            if not wins:
                continue
            dirs = dt_irqs(node)
            if not dirs:
                continue
            union = set()
            names = set()
            for name, aw, airq in adt_all:
                if any(overlap(w, aw2) for w in wins for aw2 in aw):
                    union |= set(airq)
                    names.add(name)
            bad = [i for i in dirs if i not in union and i not in DT_IRQ_ALLOW]
            check(not bad, f"{path}: irqs {[f'{d}:{n}' for d, n in dirs]} within the ADT ({', '.join(sorted(names))})"
                  if not bad else
                  f"{path}: irq(s) {bad} not in the ADT interrupt set of the overlapped node(s)")
            for name in names:
                if name in matched_irqs:
                    matched_irqs[name] += dirs
        for name in REQUIRED:
            if name in IRQ_COVERAGE_EXEMPT:
                continue
            valid = airqs.get(name, ([], []))[1]
            if not valid:
                continue
            carried = [i for i in matched_irqs[name] if i in valid]
            check(carried, f"{name}: ADT interrupts {valid} carried by the DT"
                  if carried else
                  f"{name}: ADT interrupts {valid} carried by no DT node")

        # 6. disabled gates: a DT node whose first window starts where a
        # window of a gated ADT node starts must be disabled (DARTs: any window)
        gw = adt_gated_nodes(root, set(GATED_ADT))
        starts = {ws[0][0] for ws in gw.values() if ws}
        starts |= {a for n, ws in gw.items() if n.startswith("dart-") for a, _ in ws}
        hit = 0
        for path, node, wins in dtnodes:
            if wins and wins[0][0] in starts:
                hit += 1
                st = dt_strings(node.props.get("status", ""))
                check(st == ["disabled"], f"{path}: disabled"
                      if st == ["disabled"] else f"{path}: status is {st or 'okay'}, must be disabled")
        check(hit >= len(gw), f"{hit} DT nodes match the {len(gw)} gated ADT nodes"
              if hit >= len(gw) else
              f"only {hit} DT nodes match the {len(gw)} gated ADT nodes: a gated node is missing or moved")

    if failures:
        print(f"{len(failures)} check(s) failed")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
