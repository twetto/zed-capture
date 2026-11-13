#!/bin/bash
# Installs zed-capture's uvcvideo (root): the kernel's own uvcvideo with 32 transfers queued for a
# stream instead of 5, built by dkms for this kernel and for each new one, and loaded now if the
# camera is not open. Installs dkms if it is missing (without its recommended packages: on a
# Jetson those would be Ubuntu's kernel headers). Building needs the kernel's headers (JetPack:
# nvidia-l4t-kernel-headers) and the internet (prepare.sh downloads the source). Running it again
# reinstalls; uninstall.sh puts the kernel's back.
#   sudo dkms/install.sh
set -eu
[ "$(id -u)" = 0 ] || { echo "Run it as root (sudo)" >&2; exit 1; }
here=$(cd "$(dirname "$0")" && pwd)
name=uvcvideo-zed
version=$(sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' "$here/dkms.conf")
[ -n "$version" ] || { echo "No PACKAGE_VERSION in $here/dkms.conf" >&2; exit 1; }
[ -d "/lib/modules/$(uname -r)/build" ] || { echo "No kernel headers at /lib/modules/$(uname -r)/build" >&2; exit 1; }

command -v dkms > /dev/null || apt-get install -y --no-install-recommends dkms
dir=/usr/src/$name-$version
if [ -e "/var/lib/dkms/$name/$version" ]; then dkms remove -m "$name" -v "$version" --all; fi
rm -rf "$dir"
mkdir -p "$dir"
install -m 644 "$here/dkms.conf" "$dir/"
install -m 755 "$here/prepare.sh" "$here/ensure.sh" "$dir/"
dkms add -m "$name" -v "$version"
sed "s|@DIR@|$dir|" "$here/uvcvideo-zed.service" > /etc/systemd/system/uvcvideo-zed.service
systemctl daemon-reload
systemctl enable uvcvideo-zed.service
"$dir/ensure.sh"
