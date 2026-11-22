# SPDX-License-Identifier: GPL-2.0-only
#
# Built from tools/aurora-sep: rpmbuild --define "_sourcedir <that directory>"
# (build.sh does this).

%global selinuxtype targeted
%global modulename aurora_sep

Name:           aurora-sep
Version:        1
Release:        13%{?dist}
Summary:        Loader and desktop integration for the Apple SEP fingerprint driver

License:        GPL-2.0-only
URL:            https://github.com/aurora-silicon/linux
Source0:        load-driver
Source1:        aurora-sep.service
Source2:        fprintd-aurora.conf
Source3:        README.md
Source4:        XART-SAFETY.md
Source5:        aurora-sep-ready
Source6:        aurora-sep-ready.service
Source7:        fprintd-no-timeout.conf.in
Source8:        logind-aurora-sep.conf
Source9:        aurora-sep-greeter-fingerprint
Source10:       aurora-sep-greeter-fingerprint.service
Source14:       aurora-sep-greeter-refresh.service
Source15:       aurora-sep-policy-change
Source11:       aurora_sep.te
Source12:       aurora_sep.fc
Source13:       OS-IDENTITY.md
Source16:       sep-bio-identities
Source17:       aurora-sep.timer
Source18:       aurora-sep-load.conf
Source19:       90-aurora-sep.conf
Source20:       sensor-present
Source21:       touchid-setup-needed
Source22:       aurora-touchid-setup
Source23:       aurora-touchid-setup.service
Source24:       extract-mesa-calibration.py
Source25:       powerdevilrc
Source26:       aurora-sep

BuildArch:      noarch
BuildRequires:  systemd-rpm-macros
# Only to check where fprintd's unit runs it from.
BuildRequires:  fprintd
Requires:       coreutils
Requires:       fprintd
Requires:       keyutils
Requires:       kmod
Requires:       python3
Requires:       (%{name}-selinux = %{version}-%{release} if selinux-policy-%{selinuxtype})
# Only for aurora-sep-greeter-fingerprint.service, which is off by default.
Suggests:       /usr/bin/gresource
%{?systemd_requires}

%description
Loads the Apple SEP driver (apple_sep) with xART writes enabled, waits until
the secure enclave seals and unseals a trusted key and /dev/sep-bio appears,
and lets fprintd reach /dev/sep-bio. The loader runs at boot, and fprintd
also pulls it in when it starts. Touch ID is then activated, before fprintd
starts, and pressing the Touch ID button suspends rather than powers off.
libfprint needs the aurora driver, which this package does not provide; it is
packaged in https://github.com/aurora-silicon/aurora-sep-userspace. Read XART-SAFETY.md before installing it on
a machine that also runs macOS.

%package selinux
Summary:        SELinux policy module for the Apple SEP fingerprint device
BuildRequires:  selinux-policy-devel
Requires:       selinux-policy-%{selinuxtype}
Requires(post): selinux-policy-%{selinuxtype}
%{?selinux_requires}

%description selinux
Labels /dev/sep-bio, the Apple SEP driver's Touch ID device, and lets fprintd
open it. The targeted policy otherwise leaves the device as device_t, which
fprintd may not open, so fprintd lists no fingerprint reader. It also lets
fprintd read input event devices, for libfprint's lid switch follower.

%prep
%setup -q -c -T
cp -p %{SOURCE3} %{SOURCE4} %{SOURCE11} %{SOURCE12} %{SOURCE13} .

%build
make -f %{_datadir}/selinux/devel/Makefile %{modulename}.pp
bzip2 -9 %{modulename}.pp

%install
# A unit and the helper it runs ($1, installed as $3), pointed at the
# packaged helper.
unit() {
    install -Dpm 0755 "$1" %{buildroot}%{_libexecdir}/"$3"
    install -Dpm 0644 "$2" %{buildroot}%{_unitdir}/"${2##*/}"
    sed -i "s|^\(Exec[A-Za-z]*=\)/usr/local/sbin/|\1%{_libexecdir}/|" \
        %{buildroot}%{_unitdir}/"${2##*/}"
    if grep -q /usr/local %{buildroot}%{_unitdir}/"${2##*/}"; then exit 1; fi
}
unit %{SOURCE0} %{SOURCE1} aurora-sep-load
unit %{SOURCE5} %{SOURCE6} aurora-sep-ready
unit %{SOURCE9} %{SOURCE10} aurora-sep-greeter-fingerprint
install -Dpm 0644 %{SOURCE14} %{buildroot}%{_unitdir}/aurora-sep-greeter-refresh.service
install -Dpm 0755 %{SOURCE15} %{buildroot}%{_sbindir}/aurora-sep-policy-change
sed -i "s|^\(Exec[A-Za-z]*=\)/usr/local/sbin/aurora-sep-greeter-fingerprint|\1%{_libexecdir}/aurora-sep-greeter-fingerprint|" %{buildroot}%{_unitdir}/aurora-sep-greeter-refresh.service
install -Dpm 0644 %{SOURCE2} \
    %{buildroot}%{_unitdir}/fprintd.service.d/aurora-sep.conf
grep -qx 'ExecStart=%{_libexecdir}/fprintd' %{_unitdir}/fprintd.service
sed 's|@FPRINTD@|%{_libexecdir}/fprintd|' %{SOURCE7} \
    >%{buildroot}%{_unitdir}/fprintd.service.d/50-aurora-sep-no-timeout.conf
install -Dpm 0644 %{SOURCE8} \
    %{buildroot}%{_prefix}/lib/systemd/logind.conf.d/50-aurora-sep.conf
install -Dpm 0755 %{SOURCE16} %{buildroot}%{_bindir}/sep-bio-identities
install -Dpm 0755 %{SOURCE26} %{buildroot}%{_bindir}/aurora-sep
sed -i "s|^\(Exec[A-Za-z]*=\)/usr/local/sbin/|\1%{_libexecdir}/|" \
    %{buildroot}%{_unitdir}/fprintd.service.d/aurora-sep.conf
install -Dpm 0755 %{SOURCE20} %{buildroot}%{_libexecdir}/aurora-sep-sensor-present
# The Touch ID calibration setup, run when a file is missing or refused.
install -Dpm 0755 %{SOURCE21} %{buildroot}%{_libexecdir}/aurora-touchid-setup-needed
install -Dpm 0755 %{SOURCE22} %{buildroot}%{_sbindir}/aurora-touchid-setup
install -Dpm 0755 %{SOURCE24} %{buildroot}%{_libexecdir}/aurora/extract-mesa-calibration
install -Dpm 0644 %{SOURCE23} %{buildroot}%{_unitdir}/aurora-touchid-setup.service
sed -i -e "s|^\(Exec[A-Za-z]*=\)/usr/local/sbin/aurora-touchid-setup |\1%{_sbindir}/aurora-touchid-setup |" \
    -e "s|^\(Exec[A-Za-z]*=\)/usr/local/sbin/|\1%{_libexecdir}/|" \
    %{buildroot}%{_unitdir}/aurora-touchid-setup.service
# Load the driver from aurora-sep.service, once /var/lib is there, and keep
# it out of the initramfs.
install -Dpm 0644 %{SOURCE18} %{buildroot}%{_prefix}/lib/modprobe.d/aurora-sep-load.conf
install -Dpm 0644 %{SOURCE19} %{buildroot}%{_prefix}/lib/dracut/dracut.conf.d/90-aurora-sep.conf
install -Dpm 0644 %{SOURCE17} %{buildroot}%{_unitdir}/aurora-sep.timer
install -Dpm 0644 %{SOURCE25} %{buildroot}%{_sysconfdir}/xdg/powerdevilrc
if grep -rq /usr/local %{buildroot}%{_unitdir}; then exit 1; fi
# Load the driver at boot, so the enclave is up before the login screen asks.
install -d %{buildroot}%{_presetdir}
printf 'enable aurora-sep.service\nenable aurora-sep.timer\n' >%{buildroot}%{_presetdir}/80-aurora-sep.preset
install -Dpm 0644 %{modulename}.pp.bz2 \
    %{buildroot}%{_datadir}/selinux/packages/%{selinuxtype}/%{modulename}.pp.bz2

%post
%systemd_post aurora-sep.timer aurora-sep.service aurora-sep-ready.service aurora-touchid-setup.service aurora-sep-greeter-fingerprint.service aurora-sep-greeter-refresh.service

%preun
%systemd_preun aurora-sep.timer aurora-sep.service aurora-sep-ready.service aurora-touchid-setup.service aurora-sep-greeter-fingerprint.service aurora-sep-greeter-refresh.service

%postun
# Never restart them: the driver attaches to the enclave once per boot.
%systemd_postun aurora-sep.timer aurora-sep.service aurora-sep-ready.service aurora-touchid-setup.service aurora-sep-greeter-fingerprint.service aurora-sep-greeter-refresh.service

%pre selinux
%selinux_relabel_pre -s %{selinuxtype}

%post selinux
%selinux_modules_install -s %{selinuxtype} %{_datadir}/selinux/packages/%{selinuxtype}/%{modulename}.pp.bz2

%postun selinux
%selinux_modules_uninstall -s %{selinuxtype} %{modulename}
if [ $1 -eq 0 ] && selinuxenabled 2>/dev/null && [ -c /dev/sep-bio ]; then
    restorecon /dev/sep-bio || :
fi

%posttrans selinux
%selinux_relabel_post -s %{selinuxtype}
# udev labels the device when the driver creates it; label one that exists.
if selinuxenabled 2>/dev/null && [ -c /dev/sep-bio ]; then
    restorecon /dev/sep-bio || :
fi

%files
%doc README.md XART-SAFETY.md OS-IDENTITY.md
%{_libexecdir}/aurora-sep-load
%{_libexecdir}/aurora-sep-ready
%{_libexecdir}/aurora-sep-greeter-fingerprint
%{_bindir}/sep-bio-identities
%{_bindir}/aurora-sep
%{_sbindir}/aurora-sep-policy-change
%{_sbindir}/aurora-touchid-setup
%{_libexecdir}/aurora-sep-sensor-present
%{_libexecdir}/aurora-touchid-setup-needed
%dir %{_libexecdir}/aurora
%{_libexecdir}/aurora/extract-mesa-calibration
%{_prefix}/lib/modprobe.d/aurora-sep-load.conf
%{_prefix}/lib/dracut/dracut.conf.d/90-aurora-sep.conf
%config(noreplace) %{_sysconfdir}/xdg/powerdevilrc
%{_unitdir}/aurora-sep.timer
%{_unitdir}/aurora-touchid-setup.service
%{_unitdir}/aurora-sep.service
%{_unitdir}/aurora-sep-ready.service
%{_unitdir}/aurora-sep-greeter-fingerprint.service
%{_unitdir}/aurora-sep-greeter-refresh.service
%dir %{_unitdir}/fprintd.service.d
%{_unitdir}/fprintd.service.d/aurora-sep.conf
%{_unitdir}/fprintd.service.d/50-aurora-sep-no-timeout.conf
%{_prefix}/lib/systemd/logind.conf.d/50-aurora-sep.conf
%{_presetdir}/80-aurora-sep.preset

%files selinux
%{_datadir}/selinux/packages/%{selinuxtype}/%{modulename}.pp.bz2
%ghost %verify(not md5 size mode mtime) %{_sharedstatedir}/selinux/%{selinuxtype}/active/modules/200/%{modulename}

%changelog
* Fri Nov 20 2026 Diljit Singh <hey@djdev.me> - 1-13
- SEP writes are always on: aurora-sep-load runs modprobe apple_sep with no
  options, since the driver no longer has xart_writes or provision_keybag
  (aurora-silicon/linux#2591); the docs no longer offer a read-only setting
- aurora-sep and aurora-sep-ready name only the fail-closed causes of
  disabled xART writes

* Sat Nov 07 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-12
- Take the distribution packages' changes: wait on the driver's key-store
  attribute before the trusted-key probe, leave /dev/sep-bio to
  aurora-sep-ready (a Mac without a reader still gets SEP trusted keys),
  run the readiness unit and fprintd only with a reader present, load the
  driver from the service after /var/lib (modprobe blacklist, timer, not in
  the initramfs), and set up a missing or refused Touch ID calibration with
  aurora-touchid-setup
- The SELinux module gives the driver's state files their own type

* Sun Nov 01 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-11
- aurora-sep-selinux: name the device type aurora_sep_device_t, as the
  distribution packages do, let fprintd read input devices for the lid
  follower and let the readiness helpers open the device as init_t

* Fri Oct 30 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-10
- Add the Linux-lane orphan identity list and delete tool

* Wed Oct 28 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-9
- Start the login screen without waiting for Touch ID, and show it again
  once Touch ID is ready if nobody has logged in yet
- Stop holding multi-user.target back for the enclave
- aurora-sep-load: stop at once, and say why, when the xART guard refuses
  the record a boot policy change makes the enclave rewrite
- aurora-sep-policy-change: back up the gigalocker and allow that one
  rewrite on the next boot, so Touch ID survives a boot policy change

* Mon Oct 19 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-7
- XART-SAFETY.md: what the xART guard allows for restore, use, setup and
  recovery, per platform, and what a rollback does to its list

* Mon Oct 19 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-6
- Log the authselect command when PAM does not use fingerprints

* Mon Oct 19 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-5
- Add aurora-sep-selinux, which labels /dev/sep-bio for fprintd

* Sun Oct 18 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-4
- Report Touch ID ready only once the sensor attached and the
  fingerprints were restored, and the reason when it is not

* Sun Oct 11 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-3
- libfprint with the aurora driver is now built in aurora-sep-userspace

* Sun Oct 11 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-2
- Activate Touch ID at boot, before fprintd starts
- Keep fprintd running
- Suspend rather than power off on the Touch ID button
- Offer fingerprint at the GNOME 51 login screen, off by default

* Sun Oct 11 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-1
- Package the loader, its service and the fprintd policy
