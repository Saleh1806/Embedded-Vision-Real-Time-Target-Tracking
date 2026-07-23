#!/usr/bin/env bash
# clean.sh
#/* ----------------------------------------------------------------------------
#* Confidential file
#* Copyright (C) ARCK Sensor - All rights reserved
#* ----------------------------------------------------------------------------
#* Description:
#* Remove the Meson build directory for the full project.
#* ----------------------------------------------------------------------------
#* 02/12/2025 S.Diop: Initial creation
#* ----------------------------------------------------------------------------
#*/
set -euo pipefail

here="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$here"

if [ -d "build" ]; then
  rm -rf build
  echo "Removed build directory."
else
  echo "Nothing to clean."
fi
