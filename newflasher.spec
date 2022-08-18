#
# spec file for newflasher
#
# Copyright (c) 2022 Mikel Pérez (io@mikelpr.com)
#
# All modifications and additions to the file contributed by third parties
# remain the property of their copyright owners, unless otherwise agreed
# upon. The license for this file, and modifications and additions to the
# file, is the same license as for the pristine package itself (unless the
# license for the pristine package is not an Open Source License, in which
# case the license is the MIT License). An "Open Source License" is a
# license that conforms to the Open Source Definition (Version 1.9)
# published by the Open Source Initiative.


%define pkg_version 52
Name:           newflasher
Version:        %{pkg_version}
Release:        1
Summary:        Flash Sony Xperia firmwares for XZ Premium phones and later.
License:        MIT
URL:            https://github.com/munjeni/newflasher
Source0:        https://github.com/munjeni/newflasher/archive/refs/tags/%{pkg_version}.tar.gz
BuildRequires:  gcc
BuildRequires:  pkgconfig(expat)
BuildRequires:  pkgconfig(zlib)
Requires:       (libexpat.so.1()(64bit) or libexpat.so.1)
Requires:       zlib

%description
This _experimental_ software allows you to flash firmwares acquired
through XperiFirm to Sony phones from the XZ Premium and newer.

%prep
%setup -q

%build
make CFLAGS="%{optflags}" %{?_smp_mflags}
make newflasher.1.gz

%install
# the make install target wants to use real root privileges
# so can't use the %%make_install macro
mkdir -p %{buildroot}/usr/bin/
install -m 755 newflasher %{buildroot}/usr/bin/newflasher
mkdir -p %{buildroot}/usr/share/man/man1
install -m 644 newflasher.1.gz %{buildroot}/usr/share/man/man1

%files
%defattr(-,root,root)
%doc readme.md
%{_bindir}/%{name}
%{_mandir}/*/*

%changelog
