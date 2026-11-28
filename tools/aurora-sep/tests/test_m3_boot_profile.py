from pathlib import Path
import fcntl
import hashlib
import importlib.util
import json
import subprocess
import tempfile
import unittest
HELPER=Path(__file__).resolve().parent.parent/'m3-boot-profile.py'
class BootProfile(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.esp=self.root/'esp';self.state=self.root/'state'
        (self.esp/'EFI/BOOT').mkdir(parents=True);(self.esp/'EFI/Linux').mkdir()
        (self.esp/'EFI/BOOT/BOOTAA64.EFI').write_bytes(b'limine.conf++CONFIG_B2SUM_SIGNATURE++'+b'0'*128)
        self.defaults=self.root/'defaults';self.defaults.write_text('ENABLE_ENROLL_LIMINE_CONFIG=no\n')
        self.uki=self.esp/'EFI/Linux/omarchy_linux-aurora.efi';self.uki.write_bytes(b'fixture UKI6.12-test\0')
        self.conf=self.esp/'EFI/BOOT/limine.conf'
        digest=hashlib.blake2b(self.uki.read_bytes()).hexdigest()
        self.original='/+Omarchy\n  //linux-aurora\n    protocol: efi\n    path: boot():/EFI/Linux/omarchy_linux-aurora.efi#'+digest+'\n    cmdline: root=UUID=abc rw\n'
        self.conf.write_text(self.original)
        modules=self.root/'modules/6.12-test';modules.mkdir(parents=True)
        (modules/'pkgbase').write_text('linux-aurora\n');(modules/'modules.dep').write_text('fixture\n')
        (modules/'dtbs').mkdir();(modules/'dtbs/wrong.dtb').touch()
        self.lock1=self.root/'lock1';self.lock2=self.root/'lock2'
    def run_helper(self,action,ok=True,timeout=None):
        import os, shlex
        runner=shlex.split(os.environ.get("M3_TEST_PYTHON", "python3"))
        extra=[] if timeout is None else ['--lock-timeout',str(timeout)]
        result=subprocess.run(runner+[str(HELPER),action,'--esp',str(self.esp),'--state',str(self.state),'--defaults',str(self.defaults),'--modules',str(self.root/'modules'),'--release','6.12-test','--lock',str(self.lock1),'--lock',str(self.lock2)]+extra,capture_output=True,text=True)
        if ok:self.assertEqual(result.returncode,0,result.stderr)
        else:self.assertNotEqual(result.returncode,0)
        return result
    def test_retains_bytes_modules_and_gpu_off_entry(self):
        self.run_helper('retain')
        saved=json.loads((self.state/'m3-known-entry.json').read_text())
        self.assertIn('asahi.t8122_start=0 aurora.t8122_start=0 mesa_m3=off',saved['cmdline'])
        path=self.esp/saved['path'].split('#')[0].removeprefix('boot():/')
        self.assertEqual(path.read_bytes(),self.uki.read_bytes())
        self.assertFalse((self.state/'modules-6.12-test/dtbs').exists())
        self.assertIn('/Aurora previous (GPU off)', self.conf.read_text())
        self.assertIn('asahi.t8122_start=0 aurora.t8122_start=0 mesa_m3=off', self.conf.read_text())
        self.assertNotIn('asahi.t8122_start=1', self.conf.read_text())
        self.run_helper('publish');self.run_helper('publish')
        text=self.conf.read_text()
        self.assertEqual(text.count('/Aurora previous (GPU off)'),1)
        self.assertIn('root=UUID=abc rw asahi.t8122_start=1',text)
        self.assertIn('asahi.t8122_start=0 aurora.t8122_start=0 mesa_m3=off',text)
    def test_hash_mutation_refuses_before_configuration_change(self):
        self.uki.write_bytes(b'mutated')
        self.run_helper('retain',False)
        self.assertEqual(self.conf.read_text(),self.original)
        self.assertFalse((self.state/'m3-known-entry.json').exists())
    def test_enrolled_limine_refuses(self):
        (self.esp/'EFI/BOOT/BOOTAA64.EFI').write_bytes(b'limine.conf++CONFIG_B2SUM_SIGNATURE++'+b'a'*128)
        self.run_helper('retain',False)
        self.assertEqual(self.conf.read_text(),self.original)
    def test_locked_or_symlink_lock_refuses(self):
        self.lock1.touch()
        with self.lock1.open('r+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            self.run_helper('retain',False,timeout=0.15)
        self.lock1.unlink();self.lock1.symlink_to(self.defaults)
        self.run_helper('retain',False)
    def test_transient_real_boot_lock_waits_then_retains(self):
        import threading, time
        self.lock1.touch()
        with self.lock1.open('r+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            release=threading.Timer(0.6,fcntl.flock,args=(lock,fcntl.LOCK_UN))
            release.start()
            started=time.monotonic()
            try:self.run_helper('retain')
            finally:release.join()
        self.assertGreaterEqual(time.monotonic()-started,0.5)
        self.assertIn('/Aurora previous (GPU off)',self.conf.read_text())

    def test_timeout_publishes_nothing_and_releases_first_lock(self):
        before=self.conf.read_bytes(); uki=self.uki.read_bytes()
        self.lock2.touch()
        with self.lock2.open('r+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            result=self.run_helper('retain',False,timeout=0.15)
        self.assertIn('boot partition locks remained busy',result.stderr)
        self.assertEqual(self.conf.read_bytes(),before);self.assertEqual(self.uki.read_bytes(),uki)
        self.assertFalse(self.state.exists())
        with self.lock1.open('r+') as lock:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)

    def test_two_locks_share_one_deadline(self):
        from unittest import mock

        spec = importlib.util.spec_from_file_location('m3_boot_profile', HELPER)
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        now = [0.0]
        descriptors = []
        first_acquired = []

        def advance(seconds):
            now[0] += seconds

        def contend(fd, flags):
            self.assertEqual(flags, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if fd not in descriptors:
                descriptors.append(fd)
            if fd == descriptors[0] and now[0] >= 0.12:
                first_acquired.append(fd)
                return
            raise BlockingIOError

        # Only clock and contention are synthetic; production opens, validates,
        # acquires and closes both lock files. A deadline reset at the second
        # lock would advance the clock beyond the original 0.22-second budget.
        with mock.patch.object(helper.time, 'monotonic', side_effect=lambda: now[0]), \
             mock.patch.object(helper.time, 'sleep', side_effect=advance), \
             mock.patch.object(helper.fcntl, 'flock', side_effect=contend):
            with self.assertRaises(TimeoutError):
                with helper.locks([self.lock1, self.lock2], timeout=0.22):
                    self.fail('the second lock must remain busy')
        self.assertTrue(first_acquired)
        self.assertEqual(len(descriptors), 2)
        self.assertAlmostEqual(now[0], 0.22)
        for fd in descriptors:
            with self.assertRaises(OSError):
                helper.os.fstat(fd)

    def test_installed_copy_matches_standalone_helper(self):
        installer=HELPER.with_name('install-aurora-sep.sh').read_text()
        embedded=installer.split("<<'M3_BOOT_PROFILE_PY'\n",1)[1].split('\nM3_BOOT_PROFILE_PY',1)[0]+'\n'
        self.assertEqual(embedded,HELPER.read_text())

    def test_wrong_running_kernel_refuses(self):
        self.uki.write_bytes(b'wrong6.13\0')
        digest=hashlib.blake2b(self.uki.read_bytes()).hexdigest()
        self.conf.write_text(self.original.split('#')[0]+'#'+digest+'\n    cmdline: root=UUID=abc rw\n')
        self.run_helper('retain',False)
    def test_new_kernel_hook_restores_retained_entry(self):
        self.run_helper('retain');self.run_helper('publish')
        self.regenerate()
        self.run_helper('publish')
        self.assertIn('/Aurora previous (GPU off)',self.conf.read_text())
        self.assertIn('asahi.t8122_start=1',self.conf.read_text())

    def regenerate(self):
        fallback=self.conf.read_text().split('/Aurora previous (GPU off)',1)[1]
        self.uki.write_bytes(b'new UKI6.13\0');digest=hashlib.blake2b(self.uki.read_bytes()).hexdigest()
        self.conf.write_text(self.original.split('#')[0]+'#'+digest+'\n    cmdline: root=UUID=abc rw\n\n/Aurora previous (GPU off)'+fallback)

    def test_post_generation_second_lock_failure_keeps_reachable_fallback(self):
        self.run_helper('retain')
        self.regenerate()
        before=self.conf.read_bytes()
        self.lock2.touch()
        with self.lock2.open('r+') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            self.run_helper('publish',False,timeout=0.15)
        self.assertEqual(self.conf.read_bytes(),before)
        self.assertIn('/Aurora previous (GPU off)',self.conf.read_text())
        self.assertIn('asahi.t8122_start=0 aurora.t8122_start=0 mesa_m3=off',self.conf.read_text())
        self.assertNotIn('asahi.t8122_start=1',self.conf.read_text())

    def test_missing_custom_entry_refuses_activation(self):
        self.run_helper('retain')
        self.conf.write_text(self.original)
        self.run_helper('publish',False)
        self.assertNotIn('asahi.t8122_start=1',self.conf.read_text())

    def test_disarm_keeps_both_main_and_fallback_gpu_off(self):
        self.run_helper('retain');self.run_helper('publish')
        self.run_helper('disarm')
        self.assertNotIn('asahi.t8122_start=1',self.conf.read_text())
        self.assertEqual(self.conf.read_text().count('asahi.t8122_start=0 aurora.t8122_start=0 mesa_m3=off'),2)

    def test_remove_drops_gpu_off_entry_and_retained_uki(self):
        self.run_helper('retain')
        saved=json.loads((self.state/'m3-known-entry.json').read_text())
        retained=self.esp/saved['path'].split('#')[0].removeprefix('boot():/')
        self.assertTrue(retained.exists())
        self.run_helper('remove')
        self.assertEqual(self.conf.read_text(),self.original)
        self.assertFalse(retained.exists())
        self.assertTrue(self.uki.exists())
        self.run_helper('remove')
        self.assertEqual(self.conf.read_text(),self.original)

    def test_remove_after_publish_keeps_the_main_entry(self):
        self.run_helper('retain');self.run_helper('publish')
        self.run_helper('remove')
        text=self.conf.read_text()
        self.assertNotIn('/Aurora previous (GPU off)',text)
        self.assertNotIn('mesa_m3=off',text)
        self.assertIn('  //linux-aurora\n',text)
        self.assertTrue(self.uki.exists())

    def test_remove_without_retained_entry_changes_nothing(self):
        self.run_helper('remove')
        self.assertEqual(self.conf.read_text(),self.original)
        self.assertEqual(list((self.esp/'EFI/Linux').glob('aurora-m3-previous-*')),[])

    def retained_path(self):
        saved=json.loads((self.state/'m3-known-entry.json').read_text())
        return self.esp/saved['path'].split('#')[0].removeprefix('boot():/'),saved

    def test_remove_refuses_corrupt_state_before_publication(self):
        self.run_helper('retain')
        retained,_=self.retained_path();before=self.conf.read_bytes()
        (self.state/'m3-known-entry.json').write_text('{broken')
        self.run_helper('remove',False)
        self.assertEqual(self.conf.read_bytes(),before)
        self.assertTrue(retained.exists())

    def test_remove_refuses_edited_fallback_before_publication(self):
        self.run_helper('retain')
        retained,_=self.retained_path()
        self.conf.write_text(self.conf.read_text().replace('asahi.t8122_start=0 aurora.t8122_start=0 mesa_m3=off','asahi.t8122_start=1 aurora.t8122_start=1'))
        before=self.conf.read_bytes()
        self.run_helper('remove',False)
        self.assertEqual(self.conf.read_bytes(),before)
        self.assertTrue(retained.exists())

    def test_remove_preserves_snapshot_membership(self):
        self.run_helper('retain')
        retained,saved=self.retained_path()
        original=self.conf.read_text()
        digest=hashlib.blake2b(retained.read_bytes()).hexdigest()
        for key in ('path', 'PATH'):
            for suffix in ('', '#'+digest, '#'+digest.upper()):
                with self.subTest(key=key,pin=suffix):
                    self.conf.write_text(original+'\n/Snapshot\n protocol: efi\n '+key+': '+saved['path'].split('#')[0]+suffix+'\n cmdline: root=UUID=abc ro\n')
                    before=self.conf.read_bytes()
                    result=self.run_helper('remove',False)
                    self.assertIn('another boot entry',result.stderr)
                    self.assertEqual(self.conf.read_bytes(),before)
                    self.assertTrue(retained.exists())

    def test_remove_preserves_vfat_case_and_dot_alias_membership(self):
        self.run_helper('retain')
        retained,saved=self.retained_path()
        original=self.conf.read_text()
        path=saved['path'].split('#')[0]
        for alias in (path.replace('/EFI/Linux/','/efi/linux/').replace('.efi','.EFI'),path+'.'):
            with self.subTest(alias=alias):
                self.conf.write_text(original+'\n/Snapshot\n protocol: efi\n path: '+alias+'\n cmdline: root=UUID=abc ro\n')
                before=self.conf.read_bytes()
                result=self.run_helper('remove',False)
                self.assertIn('another boot entry',result.stderr)
                self.assertEqual(self.conf.read_bytes(),before)
                self.assertTrue(retained.exists())

    def test_remove_refuses_changed_or_symlinked_uki(self):
        self.run_helper('retain')
        retained,_=self.retained_path();before=self.conf.read_bytes()
        original=retained.read_bytes()
        retained.write_bytes(b'changed')
        self.run_helper('remove',False)
        self.assertEqual(self.conf.read_bytes(),before)
        retained.unlink();retained.symlink_to(self.uki)
        self.run_helper('remove',False)
        self.assertEqual(self.conf.read_bytes(),before)
        self.assertEqual(self.uki.read_bytes(),original)

    def test_check_remove_validates_without_removing_anything(self):
        self.run_helper('retain')
        retained,_=self.retained_path();before=self.conf.read_bytes()
        self.run_helper('check-remove')
        self.assertEqual(self.conf.read_bytes(),before)
        self.assertTrue(retained.exists())
        self.run_helper('remove')
        self.assertFalse(retained.exists())

    def test_remove_refuses_a_malformed_gpu_off_entry(self):
        self.run_helper('retain')
        broken=self.conf.read_text().replace('    protocol: efi\n    path: boot():/EFI/Linux/aurora-m3-previous','    path: boot():/EFI/Linux/aurora-m3-previous')
        self.conf.write_text(broken)
        self.run_helper('remove',False)
        self.assertEqual(self.conf.read_text(),broken)
