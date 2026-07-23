#!/usr/bin/env bash
# build.sh
#/* ----------------------------------------------------------------------------
#* Confidential file
#* Copyright (C) ARCK Sensor - All rights reserved
#* ----------------------------------------------------------------------------
#* Description:
#* Meson build for the full Sirrah Camera project.
#* ----------------------------------------------------------------------------
#* 02/12/2025 S.Diop: Initial creation
#* ----------------------------------------------------------------------------
#*/

set -euo pipefail

here="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$here"

# Create the build directory if missing, otherwise reconfigure.
enable_tests_flag="-Denable_tests=false"
if [ "${ENABLE_TESTS:-0}" = "1" ]; then
  enable_tests_flag="-Denable_tests=true"
fi

enable_cam_flag="-Denable_camera_acquisition=false"
if [ "${ENABLE_CAMERA_ACQUISITION:-0}" = "1" ]; then
  enable_cam_flag="-Denable_camera_acquisition=true"
fi

if [ ! -d "build" ]; then
  meson setup build "$enable_tests_flag" "$enable_cam_flag"
else
  meson setup build --reconfigure "$enable_tests_flag" "$enable_cam_flag"
fi

# Compile everything.
meson compile -C build
