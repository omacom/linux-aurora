"""Literal Limine boot arguments and package-hook activation ordering."""
from pathlib import Path
import unittest
import test_m3_persistent


class Cmdline(unittest.TestCase):
    def setUp(self):
        self.fixture = test_m3_persistent.Persistent()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.defaults = self.fixture.etc / 'default/limine'
        self.defaults.write_text('KERNEL_CMDLINE[default]="root=UUID=root rd.luks.name=crypt=root rw"\n')
        self.vendor = self.fixture.tmp / 'vendor'
        self.vendor.mkdir()
        self.proc = self.fixture.tmp / 'proc'
        self.proc.mkdir()
        (self.proc / 'cmdline').write_text('BOOT_IMAGE=/vmlinuz root=UUID=running rd.luks.name=running=root rw\n')
        self.setup = (f'M3_LIMINE_DEFAULTS="{self.defaults}"\n'
                      f'M3_LIMINE_VENDOR_CONF="{self.vendor}"\n'
                      f'M3_PROCFS="{self.proc}"\n')

    def run_cmdline(self, armed=1, check=True):
        return self.fixture.run_sh(self.setup + f'm3_persistent_cmdline limine {armed}', check=check)

    def setting(self):
        return self.defaults.read_text().split('# >>> aurora-sep: M3 GPU cmdline\n')[1].splitlines()[0]

    def test_default_encryption_preserved_without_shell_expression(self):
        self.run_cmdline()
        self.assertEqual(self.setting(), 'KERNEL_CMDLINE[linux-aurora]="root=UUID=root rd.luks.name=crypt=root rw asahi.t8122_start=1 aurora.t8122_start=1"')
        original = self.defaults.read_bytes()
        self.run_cmdline()
        self.assertEqual(self.defaults.read_bytes(), original)

    def test_specific_kernel_overrides_default_without_changing_other_keys(self):
        self.defaults.write_text('KERNEL_CMDLINE[default]="root=UUID=default rw"\n'
                                 'KERNEL_CMDLINE[linux-aurora]="root=UUID=selected rd.luks.uuid=selected rootflags=subvol=@"\n'
                                 'KERNEL_CMDLINE[linux-lts]="root=UUID=other ro"\n')
        self.run_cmdline()
        self.assertIn('root=UUID=selected rd.luks.uuid=selected rootflags=subvol=@', self.setting())
        self.assertIn('KERNEL_CMDLINE[linux-lts]="root=UUID=other ro"', self.defaults.read_text())

    def test_config_precedence_and_prepend_assignments(self):
        (self.vendor/'10-default.conf').write_text('KERNEL_CMDLINE[default]="root=UUID=vendor rw"\n')
        (self.fixture.etc/'limine-entry-tool.conf').write_text('KERNEL_CMDLINE[default]+="rd.luks.name=crypt=root"\n')
        directory=self.fixture.etc/'limine-entry-tool.d'
        directory.mkdir()
        (directory/'10-options.conf').write_text('KERNEL_CMDLINE[default]+="quiet"\n')
        (directory/'20-options.conf').write_text('KERNEL_CMDLINE[default]+="initramfs_async=0"\n')
        self.defaults.write_text('KERNEL_CMDLINE[default]+="rootflags=subvol=@"\n')
        self.run_cmdline()
        self.assertEqual(self.setting(), 'KERNEL_CMDLINE[linux-aurora]="rootflags=subvol=@ initramfs_async=0 quiet rd.luks.name=crypt=root root=UUID=vendor rw asahi.t8122_start=1 aurora.t8122_start=1"')

    def test_old_owned_expression_is_repaired_from_literal_defaults(self):
        self.defaults.write_text(self.defaults.read_text() + '\n# >>> aurora-sep: M3 GPU cmdline\n'
                                 'KERNEL_CMDLINE[linux-aurora]="${KERNEL_CMDLINE[linux-aurora]:-${KERNEL_CMDLINE[default]}} asahi.t8122_start=1"\n'
                                 '# <<< aurora-sep: M3 GPU cmdline\n')
        self.run_cmdline(0)
        self.assertNotIn('${', self.defaults.read_text())
        self.assertIn('rd.luks.name=crypt=root', self.setting())
        self.assertIn('asahi.t8122_start=0 aurora.t8122_start=0 mesa_m3=off', self.setting())

    def test_only_gpu_and_boot_image_arguments_are_removed(self):
        self.defaults.write_text('KERNEL_CMDLINE="root=UUID=root asahi.t8122-start=1 aurora.t8122_start=0 mesa_m3=off air_gpu.oneshot=1 BOOT_IMAGE=/old loglevel=4 usbcore.autosuspend=-1"\n')
        self.run_cmdline()
        self.assertEqual(self.setting(), 'KERNEL_CMDLINE[linux-aurora]="root=UUID=root loglevel=4 usbcore.autosuspend=-1 asahi.t8122_start=1 aurora.t8122_start=1"')

    def test_empty_config_uses_etc_kernel_then_running_cmdline(self):
        self.defaults.write_text('# no configured command line\n')
        directory=self.fixture.etc/'kernel';directory.mkdir()
        (directory/'cmdline').write_text('root=UUID=etc\nrd.luks.name=etc=root rw\n')
        self.run_cmdline()
        self.assertIn('root=UUID=etc rd.luks.name=etc=root rw', self.setting())
        (directory/'cmdline').unlink()
        self.run_cmdline()
        self.assertIn('root=UUID=running rd.luks.name=running=root rw', self.setting())
        self.assertNotIn('BOOT_IMAGE=', self.setting())

    def test_invalid_user_arguments_refuse_before_writing_or_evaluating(self):
        marker=self.fixture.tmp/'executed'
        for value in ('rw', 'root=UUID=root ${MISSING}', f'root=UUID=root $(touch {marker})',
                      f'root=UUID=root `touch {marker}`', 'root=UUID=root "quoted"'):
            with self.subTest(value=value):
                self.defaults.write_text('KERNEL_CMDLINE[default]="'+value+'"\n')
                before=self.defaults.read_bytes()
                result=self.run_cmdline(check=False)
                self.assertNotEqual(result.returncode,0)
                self.assertEqual(self.defaults.read_bytes(),before)
                self.assertFalse(marker.exists())

    def test_package_hook_receives_gpu_off_literal_before_transaction(self):
        work=self.fixture.tmp/'work';work.mkdir();(work/'matched.pkg.tar.zst').touch()
        result=self.fixture.run_sh(self.setup+f'''
M3_GPU_PERSISTENT=1
chain=limine
work="{work}"
pacman() {{
  grep -F 'root=UUID=root rd.luks.name=crypt=root rw asahi.t8122_start=0 aurora.t8122_start=0 mesa_m3=off' "$M3_LIMINE_DEFAULTS"
  return 23
}}
m3_install_packages
''',check=False)
        self.assertEqual(result.returncode,23,result.stderr)
        self.assertIn('rd.luks.name=crypt=root',result.stdout)
        self.assertNotIn('asahi.t8122_start=1',self.setting())

    def test_invalid_config_prevents_package_transaction(self):
        self.defaults.write_text('KERNEL_CMDLINE[default]="rw"\n')
        called=self.fixture.tmp/'pacman-called'
        result=self.fixture.run_sh(self.setup+f'''
M3_GPU_PERSISTENT=1
chain=limine
pacman() {{ touch "{called}"; }}
m3_install_packages
''',check=False)
        self.assertNotEqual(result.returncode,0)
        self.assertFalse(called.exists())

    def test_assignment_like_argument_refuses_without_dropping_user_options(self):
        for key in ('KERNEL_CMDLINE', 'KERNEL_CMDLINE[default]', 'KERNEL_CMDLINE[linux-aurora]'):
            with self.subTest(key=key):
                self.defaults.write_text(key+'="root=UUID=root rd.luks.name=configured=root systemd.setenv=TEST+=value"\n')
                before=self.defaults.read_bytes()
                self.assertNotEqual(self.run_cmdline(check=False).returncode,0)
                self.assertEqual(self.defaults.read_bytes(),before)
        self.defaults.write_text('# use running command line\n')
        (self.proc/'cmdline').write_text('root=UUID=running systemd.setenv=TEST+=value\n')
        before=self.defaults.read_bytes()
        self.assertNotEqual(self.run_cmdline(check=False).returncode,0)
        self.assertEqual(self.defaults.read_bytes(),before)


if __name__ == '__main__':
    unittest.main()
