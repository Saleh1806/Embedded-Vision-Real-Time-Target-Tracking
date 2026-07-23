#!/usr/bin/env bash
# run_sirrah_demo.sh
# -----------------------------------------------------------------------------
# run file for sirrah_demo
#/* ----------------------------------------------------------------------------
#* Confidential file
#* Copyright (C) ARCK Sensor - All rights reserved
#* ----------------------------------------------------------------------------
#* Description:
#* Run the sirrah_demo executable. Assumes the project is already built.
#* ----------------------------------------------------------------------------
#* 02/12/2025 S.Diop: Initial creation
#* ----------------------------------------------------------------------------
#*/
set -euo pipefail

here="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$here"

raw_capture_bin="${RAW_CAPTURE_BIN:-$here/build/raw_capture}"
if [ ! -x "$raw_capture_bin" ] && [ -x "/usr/bin/raw_capture" ]; then
  raw_capture_bin="/usr/bin/raw_capture"
fi

if [ ! -x "$raw_capture_bin" ]; then
  echo "raw_capture not found. Run ./build.sh first." >&2
  exit 1
fi

exec "$raw_capture_bin" "$@"
