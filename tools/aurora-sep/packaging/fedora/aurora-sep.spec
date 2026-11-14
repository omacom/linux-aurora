# SPDX-License-Identifier: GPL-2.0-only
#
# Built from tools/aurora-sep: rpmbuild --define "_sourcedir <that directory>"
# (build.sh does this).

%global selinuxtype targeted
%global modulename aurora_sep

Name:           aurora-sep
Version:        1
Release:        8%{?dist}
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

BuildArch:      noarch
BuildRequires:  systemd-rpm-macros
# Only to check where fprintd's unit runs it from.
BuildRequires:  fprintd
Requires:       coreutils
Requires:       fprintd
Requires:       keyutils
Requires:       kmod
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
fprintd may not open, so fprintd lists no fingerprint reader.

%prep
%setup -q -c -T
cp -p %{SOURCE3} %{SOURCE4} %{SOURCE11} %{SOURCE12} .

%build
make -f %{_datadir}/selinux/devel/Makefile %{modulename}.pp
bzip2 -9 %{modulename}.pp

%install
# A unit and the helper it runs ($1, installed as $3), pointed at the
# packaged helper.
unit() {
    install -Dpm 0755 "$1" %{buildroot}%{_libexecdir}/"$3"
    install -Dpm 0644 "$2" %{buildroot}%{_unitdir}/"${2##*/}"
    sed -i "s|^\(Exec[A-Za-z]*=\)/usr/local/sbin/$3|\1%{_libexecdir}/$3|" \
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
# Load the driver at boot, so the enclave is up before the login screen asks.
install -d %{buildroot}%{_presetdir}
echo 'enable aurora-sep.service' >%{buildroot}%{_presetdir}/80-aurora-sep.preset
install -Dpm 0644 %{modulename}.pp.bz2 \
    %{buildroot}%{_datadir}/selinux/packages/%{selinuxtype}/%{modulename}.pp.bz2

%post
%systemd_post aurora-sep.service aurora-sep-ready.service aurora-sep-greeter-fingerprint.service aurora-sep-greeter-refresh.service

%preun
%systemd_preun aurora-sep.service aurora-sep-ready.service aurora-sep-greeter-fingerprint.service aurora-sep-greeter-refresh.service

%postun
# Never restart them: the driver attaches to the enclave once per boot.
%systemd_postun aurora-sep.service aurora-sep-ready.service aurora-sep-greeter-fingerprint.service aurora-sep-greeter-refresh.service

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
%doc README.md XART-SAFETY.md
%{_libexecdir}/aurora-sep-load
%{_libexecdir}/aurora-sep-ready
%{_libexecdir}/aurora-sep-greeter-fingerprint
%{_sbindir}/aurora-sep-policy-change
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
* Wed Oct 28 2026 Ryan Murray <ryan@aurorasilicon.org> - 1-8
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
