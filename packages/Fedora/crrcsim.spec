Summary: A Model-Airplane Flight Simulation Program
Name: crrcsim
Version: 0.9.10
Release: 1.fc10
License: GPL
Group: Amusements/Games
URL: http://crrcsim.sf.net/
Source0: %{name}-%{version}.tar.gz
Source1: CRRCsim.desktop

Requires: plib
Requires: portaudio
Requires: SDL

BuildPrereq: SDL-devel
BuildPrereq: portaudio-devel
BuildPrereq: libjpeg-devel
BuildPrereq: plib-devel
BuildRequires: desktop-file-utils

%description
Crrcsim is a model-airplane flight simulation program. Using it, you can learn
how to fly model aircraft, test new aircraft designs, and improve your
skills by practicing on your computer. 	

It rules! The flight model is very realistic. The flight model parameters are
calculated based on a 3D representation of the aircraft. Stalls are properly
modelled as well. Model control is possible with your own rc transmitter, or
any input device such as joystick, mouse, keyboard ...

%prep
%setup -q

%build
./configure
make

%install
rm -rf %{buildroot}

make DESTDIR=%{buildroot} install
#make  install
desktop-file-install --vendor=""                        \
        --dir=${RPM_BUILD_ROOT}%{_datadir}/applications \
	--add-category X-Red-Hat-Extra                  \
        ${RPM_SOURCE_DIR}/CRRCsim.desktop

%clean
rm -rf %{buildroot}

%files
%defattr(-,root,root)
/usr/local/bin/crrcsim
/usr/local/share/crrcsim
/usr/local/share/doc/crrcsim
/usr/local/share/man/man1/crrcsim.1
/usr/share/applications/CRRCsim.desktop


%changelog
* Thu Mar 05 2009 Jerry Williams <jwilliam@xmission.com> 0.9.10
- Changed for Fedora 10

* Mon May 27 2008 Jerry Williams <jwilliam@xmission.com> 0.9.9-1
- initial build

