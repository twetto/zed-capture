#!/bin/bash
# Removes zed-capture's uvcvideo (root): out of dkms and /usr/src, its boot service gone, and the
# kernel's own uvcvideo loaded again if the camera is not open (else at the next boot).
#   sudo dkms/uninstall.sh
set -eu
[ "$(id -u)" = 0 ] || { echo "Run it as root (sudo)" >&2; exit 1; }
name=uvcvideo-zed
systemctl disable --now uvcvideo-zed.service 2> /dev/null || true
rm -f /etc/systemd/system/uvcvideo-zed.service
systemctl daemon-reload
for dir in /usr/src/"$name"-*; do
  [ -d "$dir" ] || continue
  version=${dir##*/"$name"-}
  if [ -e "/var/lib/dkms/$name/$version" ]; then dkms remove -m "$name" -v "$version" --all; fi
  rm -rf "$dir"
done
if [ -d /sys/module/uvcvideo ] && [ "$(cat /sys/module/uvcvideo/refcnt)" != 0 ]; then
  echo "A program has the camera open: the kernel's uvcvideo loads at the next boot" >&2
  exit 0
fi
modprobe -r uvcvideo 2> /dev/null || true
modprobe uvcvideo
echo "uvcvideo: $(cat /sys/module/uvcvideo/version) ($(modinfo -n uvcvideo))"
