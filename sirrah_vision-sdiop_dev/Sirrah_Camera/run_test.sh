#!/usr/bin/env bash
# run_test.sh
# -----------------------------------------------------------------------------
# run file for sirrah_angle_tests
#/* ----------------------------------------------------------------------------
#* Confidential file
#* Copyright (C) ARCK Sensor - All rights reserved
#* ----------------------------------------------------------------------------
#* Description:
#* Run the GoogleTest suite. Assumes the project is already built with ENABLE_TESTS=1.
#* ----------------------------------------------------------------------------
#* 02/12/2025 S.Diop: Initial creation
#* ----------------------------------------------------------------------------
#*/

set -euo pipefail

here="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$here"

if [ ! -d "build" ]; then
  echo "Build directory not found. Run ./build.sh first." >&2
  exit 1
fi

meson test -C build --verbose --print-errorlogs
