The installer leaves desktop packages unchanged unless `--desktop-fixes` is
selected. This option requires a release containing a matched desktop manifest.
It accompanies the normal install.

The supported Omarchy upgrade is the matching stable `omarchy` and
`omarchy-settings` pair, `4.0.4-1` to `4.0.4-2`. The installer checks the
package-owned runtime file list, bytes, modes, symlink targets, launchers and
recovery configuration. Modified, development, mixed or newer packages are
refused. An exact guarded `4.0.4-2` pair is verified without reinstalling it.
An absent Omarchy pair leaves desktop settings unchanged.

Aquamarine `0.15.1-1`, `1.1` or `1.2`, providing ABI14, can upgrade to
`0.15.1-1.3`. Absent, newer, already fixed or different-ABI libraries are kept.
All selected archives must match their checksums, package identities,
architecture and required payloads. Their dependencies must already be
installed, apart from the settings package supplied in the matching pair.
These checks complete before installation or boot-setting changes. The
desktop packages join the kernel's existing package transaction.

Log out or reboot after the Omarchy upgrade to enter the guarded compositor
session. Installing it does not replace an already running launcher. Lock
intent is retained, and kernel uninstall or failed-install cleanup does not
remove the desktop guard or its authenticated login recovery. Suspend,
compositor restart, logout and the next login still require verification on
the installed hardware and desktop stack.

`assemble-m3-stack.py` accepts a separate `desktop_fixes` object with schema
`aurora.desktop-fixes/1`. It names exactly the three optional archives and
both stable Omarchy runtime catalogs. It checks the final runtime catalog
against the paired archives before embedding it. The six required kernel,
bootloader and supporting package roles remain unchanged. Without this
object, the assembled installer refuses `--desktop-fixes`.
