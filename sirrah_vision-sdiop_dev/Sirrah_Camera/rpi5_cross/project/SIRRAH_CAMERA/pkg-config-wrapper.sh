#!/bin/bash
# pkg-config wrapper for RPi5 cross build. Resolves the sysroot relative to this script
# so it works regardless of where the repo is cloned (e.g. /mnt/c/... in WSL).

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SYSROOT="$(cd "$SCRIPT_DIR/../.." && pwd)/sysroot"

PKG_CONFIG_PATH="$SYSROOT/usr/lib/aarch64-linux-gnu/pkgconfig:$SYSROOT/usr/share/pkgconfig"
PKG_CONFIG_LIBDIR="$PKG_CONFIG_PATH"
PKG_CONFIG_SYSROOT_DIR="$SYSROOT"
export PKG_CONFIG_PATH PKG_CONFIG_LIBDIR PKG_CONFIG_SYSROOT_DIR

/usr/bin/pkg-config "$@"
