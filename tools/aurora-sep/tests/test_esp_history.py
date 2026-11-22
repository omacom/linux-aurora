"""EFI history custody, boot-reference protection and archive failure controls."""
from pathlib import Path
import importlib.util
import json
import hashlib
import os
import tempfile
import subprocess
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'esp-history.py'
SPEC = importlib.util.spec_from_file_location('esp_history', SCRIPT)
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


class HistoryTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.esp = self.root / 'esp'; self.esp.mkdir()
        self.state = self.root / 'state'; self.state.mkdir()
        self.destination = self.state / 'esp-history'
        self.conf = self.esp / 'limine.conf'
        self.conf.write_text('/Active\n path: boot():/EFI/Linux/main.efi\n')
        (self.esp / 'EFI/Linux').mkdir(parents=True)
        (self.esp / 'EFI/Linux/main.efi').write_bytes(b'active')
        self.history = self.esp / 'limine_history'; self.history.mkdir()
        self.boot = self.esp / 'm1n1'; self.boot.mkdir()
        (self.boot / 'boot.bin').write_bytes(b'current')
        for index in range(4):
            for p in (self.history / f'old{index}.efi', self.boot / f'boot.bin.before-v{index}'):
                p.write_bytes(bytes([index]) * 100)
                os.utime(p, ns=(index+1, index+1))
        self.locks = [self.root / 'boot.lock', self.root / 'limine.lock']

    def archive(self):
        actual = os.stat
        # The fixture's archive and FAT stand-in share a test filesystem.
        def different(path, *args, **kwargs):
            value = actual(path, *args, **kwargs)
            if Path(path) == self.destination:
                values = list(value); values[2] += 1
                return os.stat_result(values)
            return value
        with patch.object(M.os, 'stat', side_effect=different):
            return M.archive(self.esp, self.state, self.destination, self.locks)

    def test_report_retains_newest_and_never_changes_files(self):
        before = {p:p.read_bytes() for p in self.esp.rglob('*') if p.is_file()}
        rows = M.inventory(self.esp, self.state)
        self.assertEqual(sum(not row['protected'] for row in rows), 4)
        self.assertEqual(sum(row['protected']=='recent' for row in rows), 4)
        self.assertEqual(before, {p:p.read_bytes() for p in self.esp.rglob('*') if p.is_file()})

    def test_archive_verified_copies_and_restore_receipts(self):
        rows = self.archive()
        self.assertEqual(len(rows), 4)
        for row in rows:
            self.assertFalse((self.esp / row['original']).exists())
            self.assertEqual(M.digest(self.destination / row['archive']), row['sha256'])
            path_key = hashlib.sha256(row['original'].encode()).hexdigest()[:16]
            self.assertEqual(json.loads((self.destination / (row['sha256']+'-'+path_key+'.receipt')).read_text()), row)
        self.assertTrue((self.esp / 'EFI/Linux/main.efi').is_file())
        self.assertTrue((self.boot / 'boot.bin').is_file())
        self.assertEqual(self.archive(), [])

    def test_config_state_and_utf16_references_protect_history(self):
        self.conf.write_text('path: boot():/limine_history/OLD0.EFI...#1234\n')
        (self.state / 'recovery.json').write_text('{"file":"m1n1/boot.bin.before-v0"}')
        (self.esp / 'ubootefi.var').write_bytes('old1.efi'.encode('utf-16-le'))
        archived = {row['original'] for row in self.archive()}
        self.assertEqual(archived, {'m1n1/boot.bin.before-v1'})

    def test_saved_headers_are_payload_not_boot_metadata(self):
        headers = self.state / 'modules-7.1.12-sep-ARCH/build/include'
        headers.mkdir(parents=True)
        (headers / 'linked.h').symlink_to('/missing/kernel/header.h')
        (headers / 'fake.conf').write_text('old0.efi boot.bin.before-v0')
        (self.state / 'esp-history').mkdir()
        (self.state / 'esp-history/unrelated.json').write_text('old0.efi')
        (self.state / 'recovery.json').write_text('boot.bin.before-v0')
        rows = self.archive()
        self.assertEqual({row['original'] for row in rows},
                         {'limine_history/old0.efi', 'limine_history/old1.efi',
                          'm1n1/boot.bin.before-v1'})
        self.assertTrue((self.boot / 'boot.bin.before-v0').exists())
        self.assertTrue((headers / 'linked.h').is_symlink())
        for row in rows:
            saved = self.destination / row['archive']
            original = self.esp / row['original']
            original.write_bytes(saved.read_bytes())
            self.assertEqual(M.digest(original), row['sha256'])

    def test_skipped_module_directory_symlink_is_never_followed(self):
        outside = self.root / 'outside'; outside.mkdir()
        (outside / 'recovery.json').write_text('old0.efi')
        (self.state / 'modules-old').symlink_to(outside, target_is_directory=True)
        rows = M.inventory(self.esp, self.state)
        self.assertFalse(next(row['protected'] for row in rows
                              if row['file']=='limine_history/old0.efi'))

    def test_metadata_file_and_directory_symlinks_still_refuse_archive(self):
        outside = self.root / 'outside'; outside.mkdir()
        (outside / 'recovery.json').write_text('old0.efi')
        for name, target in [('recovery.json', outside / 'recovery.json'),
                             ('recovery', outside)]:
            with self.subTest(name=name):
                link = self.state / name; link.symlink_to(target)
                try:
                    with self.assertRaisesRegex(ValueError, 'symlink'):
                        self.archive()
                    self.assertTrue((self.history / 'old0.efi').exists())
                finally:
                    link.unlink()

    def test_state_root_symlink_is_refused_even_if_dangling(self):
        self.state.rmdir(); self.state.symlink_to(self.root / 'missing-state')
        with self.assertRaisesRegex(ValueError, 'symlink'):
            M.inventory(self.esp, self.state)

    def test_missing_config_keeps_all_files(self):
        self.conf.unlink()
        with self.assertRaisesRegex(ValueError, 'configuration missing'):
            self.archive()
        self.assertTrue((self.history / 'old0.efi').exists())

    def test_same_filesystem_is_refused(self):
        with self.assertRaisesRegex(ValueError, 'different filesystem'):
            M.archive(self.esp, self.state, self.destination, self.locks)
        self.assertTrue((self.history / 'old0.efi').exists())

    def test_symlink_tree_is_refused(self):
        (self.esp / 'linked.conf').symlink_to(self.conf)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            self.archive()
        self.assertTrue((self.history / 'old0.efi').exists())

    def test_damaged_archive_never_removes_original(self):
        self.destination.mkdir()
        old = self.history / 'old0.efi'
        (self.destination / (M.digest(old)+'.bin')).write_bytes(b'damaged')
        with self.assertRaisesRegex(ValueError, 'damaged archive'):
            self.archive()
        self.assertTrue(old.exists())

    def test_changed_boot_reference_retains_original(self):
        original = M.inventory; calls = 0
        def change(esp, state):
            nonlocal calls
            calls += 1
            if calls == 2:
                self.conf.write_text('path: boot():/m1n1/boot.bin.before-v0\n')
            return original(esp, state)
        with patch.object(M, 'inventory', side_effect=change):
            with self.assertRaisesRegex(ValueError, 'boot state changed'):
                self.archive()
        self.assertTrue((self.boot / 'boot.bin.before-v0').exists())

    def test_copy_write_failure_retains_original(self):
        with patch.object(M.os, 'fsync', side_effect=OSError('write failure')):
            with self.assertRaisesRegex(OSError, 'write failure'):
                self.archive()
        self.assertTrue((self.boot / 'boot.bin.before-v0').exists())

    def test_existing_correct_archive_sync_failure_retains_original(self):
        self.destination.mkdir()
        original = self.boot / 'boot.bin.before-v0'
        saved = self.destination / (M.digest(original) + '.bin')
        saved.write_bytes(original.read_bytes())
        with patch.object(M.os, 'fsync', side_effect=OSError('archive sync failed')):
            with self.assertRaisesRegex(OSError, 'archive sync failed'):
                self.archive()
        self.assertTrue(original.exists())
        self.assertFalse(list(self.destination.glob('*.receipt')))

    def shell(self, body):
        installer = SCRIPT.with_name('install-aurora-sep.sh')
        return subprocess.run(['bash', '-c',
            'export AURORA_SEP_SOURCE_ONLY=1; source "$1"; ' + body,
            'test', str(installer)], capture_output=True, text=True)

    def test_installer_embeds_exact_helper(self):
        run = self.shell('esp_history_builtin')
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(run.stdout, SCRIPT.read_text())

    def test_install_archives_only_when_requested_before_planning(self):
        stubs = ('release_source(){ :; }; require_supported_soc(){ :; }; '
                 'neo_gpu_plan(){ echo PLAN; exit 77; }; '
                 'esp_history_run(){ echo "HISTORY:$1"; }; ')
        for enabled in (0, 1):
            run = self.shell(stubs + f'ESP_ARCHIVE_HISTORY={enabled}; install_all')
            self.assertEqual(run.returncode, 77, run.stderr)
            self.assertEqual(run.stdout.splitlines(),
                             (['HISTORY:archive'] if enabled else []) + ['PLAN'])

    def test_removed_read_only_option_does_not_archive(self):
        # --read-only was removed (SEP writes are always on): with --archive-esp-history the
        # option parser refuses it before anything runs.
        installer = SCRIPT.with_name('install-aurora-sep.sh')
        text = installer.read_text()
        tail = text[text.index('args=()\n', text.index('# Tests source this file')):]
        stubs = ('preflight(){ :; }; esp_history_run(){ echo ARCHIVE; }; '
                 'install_all(){ echo INSTALL; }; ')
        run = subprocess.run(['bash', '-c', 'export AURORA_SEP_SOURCE_ONLY=1; source "$1"; shift; '
                              + stubs + tail, 'test', str(installer),
                              '--archive-esp-history', '--read-only'],
                             capture_output=True, text=True)
        self.assertNotEqual(run.returncode, 0)
        self.assertEqual(run.stdout, '')
        self.assertIn('--read-only was removed: SEP writes are always on', run.stderr)


if __name__ == '__main__':
    unittest.main()
