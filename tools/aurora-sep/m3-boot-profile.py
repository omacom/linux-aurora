#!/usr/bin/env python3
"""Retain the running Limine UKI and publish bounded experimental activation."""
import argparse
import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import tempfile
import time

BEGIN = '# >>> aurora-sep: persistent experimental M3 GPU'
END = '# <<< aurora-sep: persistent experimental M3 GPU'
CONF_PATHS = ('EFI/BOOT/limine.conf', 'boot/limine/limine.conf', 'boot/limine.conf', 'limine/limine.conf', 'limine.conf')
FALLBACK = 'Aurora previous (GPU off)'

def regular(path):
    if not stat.S_ISREG(path.lstat().st_mode):
        raise ValueError(f'{path} must be a regular file')
    return path

def atomic(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() or path.is_symlink(): regular(path)
    fd, name = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as out:
            out.write(data); out.flush(); os.fsync(out.fileno())
        os.chmod(name, 0o644)
        os.replace(name, path)
        dfd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try: os.fsync(dfd)
        finally: os.close(dfd)
    finally:
        if os.path.exists(name): os.unlink(name)

@contextlib.contextmanager
def locks(paths, timeout=30.0):
    deadline = time.monotonic() + timeout
    fds = []
    try:
        for path in paths:
            path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK, 0o600)
            fds.append(fd)
            if not stat.S_ISREG(os.fstat(fd).st_mode): raise ValueError('nonregular boot lock')
            while True:
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f'boot partition locks remained busy for {timeout:g} seconds; retry this installer') from None
                    time.sleep(min(0.05, remaining))
        yield
    finally:
        for fd in reversed(fds): os.close(fd)

def cmdline(line, armed):
    if '"' in line or "'" in line or not any(w.startswith('root=') for w in line.split()):
        raise ValueError('boot command line must contain root= and have no quotes')
    words = [w for w in line.split() if not re.match(r'(?:asahi|aurora)\.t8122[_-]start(?:=|$)', w)
             and not w.startswith('mesa_m3=') and not w.startswith('air_gpu.oneshot=')]
    # aurora.ko reads aurora.*, the older asahi.ko asahi.*; each module ignores the other's key.
    value = '1' if armed else '0'
    return ' '.join(words + ['asahi.t8122_start=' + value, 'aurora.t8122_start=' + value] + ([] if armed else ['mesa_m3=off']))

def without_block(text):
    if text.count(BEGIN) != text.count(END) or text.count(BEGIN) > 1:
        raise ValueError('malformed persistent boot block')
    return re.sub(r'\n?' + re.escape(BEGIN) + r'\n.*?' + re.escape(END) + r'\n?', '\n', text, flags=re.S)

def entry(text, kernel, depth=2):
    lines = text.splitlines(keepends=True)
    matches = []
    for i, line in enumerate(lines):
        if line.strip() != '/' * depth + kernel: continue
        end = i + 1
        while end < len(lines) and not lines[end].lstrip().startswith('/'): end += 1
        fields = {}
        for j in range(i + 1, end):
            m = re.match(r'\s*(protocol|path|cmdline):\s*(.*?)\s*$', lines[j])
            if m:
                if m[1] in fields: raise ValueError('duplicate entry field')
                fields[m[1]] = (j, m[2])
        matches.append(fields)
    if len(matches) != 1 or any(k not in matches[0] for k in ('protocol', 'path', 'cmdline')):
        raise ValueError(f'exactly one complete {"/" * depth}{kernel} entry is required')
    if matches[0]['protocol'][1] != 'efi': raise ValueError('only EFI UKI entries are supported')
    return lines, matches[0]

def uki_path(esp, value):
    m = re.fullmatch(r'boot\(\):(/[^#]+)(?:#([0-9a-f]{128}))?', value)
    if not m or '..' in Path(m[1]).parts: raise ValueError('unsupported UKI path')
    path = esp / m[1].lstrip('/')
    if not path.resolve().is_relative_to(esp.resolve()): raise ValueError('UKI escapes ESP')
    for component in (path, *path.parents):
        if component == esp.parent: break
        if component.is_symlink(): raise ValueError('symlink in UKI path')
    data = regular(path).read_bytes()
    digest = hashlib.blake2b(data).hexdigest()
    if m[2] and m[2] != digest: raise ValueError('UKI hash pin differs')
    return data, digest

def same_esp_path(left, right):
    # VFAT names ignore ASCII case and trailing dots in each component.
    key = lambda path: tuple(part.rstrip('.').lower() for part in path.resolve().parts)
    if key(left) == key(right): return True
    try:
        return os.path.samestat(left.stat(), right.stat())
    except FileNotFoundError:
        return False

def limine(args):
    with locks(args.lock, args.lock_timeout):
        loader = regular(args.esp / 'EFI/BOOT/BOOTAA64.EFI').read_bytes()
        if b'limine.conf' not in loader: raise ValueError('EFI loader is not Limine')
        sig = b'++CONFIG_B2SUM_SIGNATURE++'
        if sig in loader:
            value = loader.split(sig, 1)[1][:128]
            if value != b'0' * 128: raise ValueError('Limine configuration hash is enrolled')
        defaults = regular(args.defaults).read_text()
        if re.search(r'^\s*ENABLE_ENROLL_LIMINE_CONFIG=[\"\x27]?yes', defaults, re.M):
            raise ValueError('Limine configuration enrollment is enabled')
        conf = next((args.esp / p for p in CONF_PATHS if (args.esp / p).is_file()), None)
        if conf is None: raise ValueError('Limine configuration missing')
        text = regular(conf).read_text()
        clean = without_block(text)
        state_path = args.state / 'm3-known-entry.json'
        if args.action in ('check-remove', 'remove'):
            output = clean
            retained = None
            has_entry = re.search(r'^\s*/' + re.escape(FALLBACK) + r'\s*$', clean, re.M)
            if state_path.exists() or state_path.is_symlink():
                saved = json.loads(regular(state_path).read_text())
                match = re.fullmatch(r'boot\(\):(/EFI/Linux/aurora-m3-previous-[0-9a-f]{16}\.efi)#([0-9a-f]{128})', saved['path'])
                if not match or not match[1].endswith(match[2][:16] + '.efi'):
                    raise ValueError('retained UKI name and hash pin differ')
                retained = args.esp / match[1].lstrip('/')
                # An edited or snapshot entry must not lose its kernel or modules.
                if has_entry:
                    lines, fields = entry(clean, FALLBACK, 1)
                    if fields['path'][1] != saved['path'] or fields['cmdline'][1] != saved['cmdline']:
                        raise ValueError('custom fallback differs from retained entry')
                    start = next(i for i, line in enumerate(lines) if line.strip() == '/' + FALLBACK)
                    end = start + 1
                    while end < len(lines) and not lines[end].lstrip().startswith('/'): end += 1
                    output = ''.join(lines[:start] + lines[end:]).rstrip() + '\n'
                for value in re.findall(r'^\s*path:\s*(\S+)', output, re.M | re.I):
                    reference = re.fullmatch(r'boot\(\):(/[^#]+)(?:#[0-9a-fA-F]{128})?', value)
                    if reference and same_esp_path(args.esp / reference[1].lstrip('/'), retained):
                        raise ValueError('another boot entry still uses the retained UKI; remove that entry first')
                if retained.exists() or retained.is_symlink():
                    uki_path(args.esp, saved['path'])
                elif has_entry:
                    raise ValueError('registered fallback UKI is missing')
                else:
                    retained = None
            else:
                owned_reference = any(re.fullmatch(r'aurora-m3-previous-[0-9a-f]{16}\.efi',
                    Path(value.split('#', 1)[0]).name.rstrip('.').lower())
                    for value in re.findall(r'^\s*path:\s*boot\(\):(/\S+)', clean, re.M | re.I))
                if has_entry or owned_reference:
                    raise ValueError('custom fallback has no saved ownership record')
            if regular(conf).read_text() != text: raise ValueError('Limine configuration changed during remove')
            if args.action == 'remove':
                atomic(conf, output.encode())
                if retained is not None: retained.unlink()
            return
        args.state.mkdir(parents=True, exist_ok=True)
        if args.action == 'retain':
            if state_path.exists() or state_path.is_symlink():
                saved = json.loads(regular(state_path).read_text())
                uki_path(args.esp, saved['path'])
                if not (args.state / ('modules-' + saved['release']) / 'modules.dep').is_file():
                    raise ValueError('retained modules missing')
            else:
                _, fields = entry(clean, args.kernel)
                data, digest = uki_path(args.esp, fields['path'][1])
                if not args.release or args.release.encode() + b'\0' not in data:
                    raise ValueError('UKI does not identify the running kernel')
                modules = args.modules / args.release
                if regular(modules / 'pkgbase').read_text().strip() != args.kernel:
                    raise ValueError('running modules do not match boot entry package')
                regular(modules / 'modules.dep')
                dest = args.state / ('modules-' + args.release)
                if dest.exists(): raise ValueError('unrecorded fallback modules exist')
                shutil.copytree(modules, dest, symlinks=True)
                shutil.rmtree(dest / 'dtbs', ignore_errors=True)
                rel = '/EFI/Linux/aurora-m3-previous-' + digest[:16] + '.efi'
                atomic(args.esp / rel.lstrip('/'), data)
                saved = dict(path='boot():' + rel + '#' + digest,
                             cmdline=cmdline(fields['cmdline'][1], False), release=args.release)
                atomic(state_path, (json.dumps(saved, sort_keys=True) + '\n').encode())
            # Publish a custom top-level EFI node under both locks. Kernel
            # regeneration owns only the OS's kernel children; the custom
            # node has no machine-id/kernel-id and survives that writer.
            # The 1.37.1 --add-efi CLI requires x86_64, so it cannot register
            # this entry on the Air. Publish the same custom menu format here.
            if re.search(r'^\s*/' + re.escape(FALLBACK) + r'\s*$', clean, re.M):
                _, fields = entry(clean, FALLBACK, 1)
                if fields['path'][1] != saved['path'] or fields['cmdline'][1] != saved['cmdline']:
                    raise ValueError('existing custom fallback differs from retained entry')
                output = clean
            else:
                output = (clean.rstrip() + '\n\n/' + FALLBACK + '\n'
                          '    # Aurora retained kernel; GPU disabled\n'
                          '    # order-priority=90\n'
                          '    protocol: efi\n    path: ' + saved['path'] + '\n'
                          '    cmdline: ' + saved['cmdline'] + '\n')
            if regular(conf).read_text() != text: raise ValueError('Limine configuration changed during retain')
            atomic(conf, output.encode())
            return
        saved = json.loads(regular(state_path).read_text())
        uki_path(args.esp, saved['path'])
        _, fallback = entry(clean, FALLBACK, 1)
        if fallback['path'][1] != saved['path'] or fallback['cmdline'][1] != saved['cmdline']:
            raise ValueError('registered GPU-off fallback differs')
        lines, fields = entry(clean, 'linux-aurora')
        # Validate the newly generated UKI before changing its activation line.
        uki_path(args.esp, fields['path'][1])
        j, old = fields['cmdline']
        lines[j] = re.match(r'\s*', lines[j])[0] + 'cmdline: ' + cmdline(old, args.action == 'publish') + '\n'
        output = ''.join(lines).rstrip() + '\n'
        if regular(conf).read_text() != text: raise ValueError('Limine configuration changed during update')
        atomic(conf, output.encode())

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=('retain', 'publish', 'disarm', 'check-remove', 'remove'))
    p.add_argument('--esp', type=Path, required=True)
    p.add_argument('--state', type=Path, required=True)
    p.add_argument('--defaults', type=Path, default=Path('/etc/default/limine'))
    p.add_argument('--kernel', default='linux-aurora')
    p.add_argument('--release', default=os.uname().release)
    p.add_argument('--modules', type=Path, default=Path('/usr/lib/modules'))
    p.add_argument('--lock', type=Path, action='append')
    p.add_argument('--lock-timeout', type=float, default=30.0, help='total boot-lock wait, in seconds (0..30)')
    args = p.parse_args()
    if not 0 <= args.lock_timeout <= 30:
        p.error('lock timeout must be between 0 and 30 seconds')
    args.lock = args.lock or [Path('/run/lock/boot-partition.lock'), Path('/tmp/limine-global.lock')]
    try: limine(args)
    except (OSError, ValueError, KeyError) as e: p.exit(1, f'M3 boot profile refused: {e}\n')

if __name__ == '__main__': main()
