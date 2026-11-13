#!/bin/bash
# dkms's PRE_BUILD for uvcvideo-zed: the kernel's own uvcvideo, with URBS transfers queued for a
# stream instead of 5. Into the current directory (dkms's build directory), for kernel
# KERNEL_VERSION:
#   - uvcvideo's source from the kernel's stable release of the same version (5.15.199-tegra:
#     v5.15.199), from kernel.org's git or its GitHub mirror;
#   - taken only if, built unchanged, it is this kernel's uvcvideo: the same srcversion (modpost's
#     checksum of a module's source) as the kernel's own module. JetPack 6.2's kernel (L4T 36.5,
#     5.15.199-tegra) passes; a kernel whose uvcvideo is patched does not, and nothing is built;
#   - then uvcvideo.h's UVC_URBS set to URBS, and the module's version "<the kernel's>-urbsURBS"
#     (/sys/module/uvcvideo/version: which one is loaded; dkms also replaces a kernel's module
#     only with a newer version).
# The directory gets the source only when all of that worked, so otherwise dkms's make fails
# (dkms 2.8 does not look at this script's exit status).
#   prepare.sh KERNEL_VERSION URBS
set -euo pipefail
kver=$1
urbs=$2
dest=$PWD
fail() { echo "uvcvideo-zed: $*" >&2; exit 1; }
[[ $urbs =~ ^[0-9]+$ ]] && ((urbs >= 5)) || fail "URBS: $urbs"

stock=$(find "/lib/modules/$kver/kernel" -name 'uvcvideo.ko*' | head -1)
[ -n "$stock" ] || fail "no uvcvideo in /lib/modules/$kver/kernel"
want=$(modinfo -F srcversion "$stock")
[ -n "$want" ] || fail "$stock has no srcversion to check a source against"
base=${kver%%-*}
tag=v${base%.0} # 5.15.0 is v5.15

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
mkdir "$work/src"
fetch() { # FILE: drivers/media/usb/uvc/FILE at the tag
  local url
  for url in "https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/media/usb/uvc/$1?h=$tag" \
             "https://raw.githubusercontent.com/gregkh/linux/$tag/drivers/media/usb/uvc/$1"; do
    curl -fsSL --retry 2 --max-time 60 -o "$work/src/$1" "$url" && return 0
  done
  fail "could not download $1 of $tag"
}
echo "uvcvideo-zed: uvcvideo's source of $tag"
fetch Makefile
# The objects its Makefile names, then the local headers their sources include
for f in $(grep -o 'uvc_[a-z0-9_]*\.o' "$work/src/Makefile" | sort -u | sed 's/\.o$/.c/'); do fetch "$f"; done
for f in $(sed -n 's/^#include "\([^"/]*\)"$/\1/p' "$work"/src/*.c | sort -u); do fetch "$f"; done

cp -r "$work/src" "$work/stock"
make -s -j"$(nproc)" -C "/lib/modules/$kver/build" M="$work/stock" modules > "$work/stock.log" 2>&1 \
  || { cat "$work/stock.log" >&2; fail "$tag's uvcvideo did not build unchanged"; }
got=$(modinfo -F srcversion "$work/stock/uvcvideo.ko")
[ "$got" = "$want" ] || fail "$tag's uvcvideo is not this kernel's (srcversion $got, the kernel's $want)"
echo "uvcvideo-zed: $tag's uvcvideo is this kernel's (srcversion $got)"

cd "$work/src"
grep -qE '^#define[[:space:]]+UVC_URBS[[:space:]]+5$' uvcvideo.h || fail "uvcvideo.h has no UVC_URBS of 5"
sed -i -E "s/^(#define[[:space:]]+UVC_URBS[[:space:]]+)5$/\1$urbs/" uvcvideo.h
if grep -qE '^#define[[:space:]]+DRIVER_VERSION[[:space:]]+"[^"]*"' uvcvideo.h; then
  sed -i -E "s/^(#define[[:space:]]+DRIVER_VERSION[[:space:]]+\"[^\"]*)\"/\1-urbs$urbs\"/" uvcvideo.h
else
  printf '\nMODULE_VERSION("urbs%s");\n' "$urbs" >> uvc_driver.c
fi
cp ./* "$dest/"
echo "uvcvideo-zed: UVC_URBS $urbs, version $(sed -n 's/^#define[[:space:]]*DRIVER_VERSION[[:space:]]*"\(.*\)"/\1/p' uvcvideo.h)"
