#!/bin/bash
# zed-capture's uvcvideo for the running kernel (root): built and installed by dkms if it is not
# yet, then loaded in place of the kernel's if the camera is not open. From install.sh, and at
# boot from uvcvideo-zed.service: after a kernel update whose dkms hook ran before the new
# kernel's headers were there, the first boot builds it (with the internet: it waits up to 5
# minutes for it) and loads it if no program has the camera open yet.
#   sudo /usr/src/uvcvideo-zed-<version>/ensure.sh
set -u
here=$(cd "$(dirname "$0")" && pwd)
name=uvcvideo-zed
version=$(sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' "$here/dkms.conf")
kver=$(uname -r)
ours() { grep -q -- '-urbs' /sys/module/uvcvideo/version 2> /dev/null; }

if ours; then
  echo "uvcvideo-zed: loaded ($(cat /sys/module/uvcvideo/version))"
  exit 0
fi
if ! dkms status -m "$name" -v "$version" -k "$kver" 2> /dev/null | grep -q ': installed'; then
  for _ in $(seq 15); do
    curl -fsS -o /dev/null --max-time 10 https://git.kernel.org/ && break
    sleep 10
  done
  if ! dkms install -m "$name" -v "$version" -k "$kver"; then
    echo "uvcvideo-zed: not built for $kver; the kernel's uvcvideo stays" >&2
    exit 1
  fi
fi
if [ -d /sys/module/uvcvideo ]; then
  if [ "$(cat /sys/module/uvcvideo/refcnt)" != 0 ]; then
    echo "uvcvideo-zed: installed for $kver, but a program has the camera open: it loads at the next" \
      "boot, or close the program and run $here/ensure.sh again" >&2
    exit 0
  fi
  modprobe -r uvcvideo || exit 1
fi
modprobe uvcvideo || exit 1
if ours; then
  echo "uvcvideo-zed: loaded ($(cat /sys/module/uvcvideo/version))"
else
  echo "uvcvideo-zed: installed, but the loaded uvcvideo is $(cat /sys/module/uvcvideo/version 2> /dev/null)" \
    "($(modinfo -n uvcvideo))" >&2
  exit 1
fi
