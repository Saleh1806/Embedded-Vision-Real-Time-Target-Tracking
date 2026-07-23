#!/bin/bash
# build_and_send.sh
# -----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# -----------------------------------------------------------------------------
# Cross-compile Sirrah_Camera (sirrah_demo + raw_capture) for Raspberry Pi,
# package as .deb, push to the Pi, and rename the installed binary with a timestamp.
# -----------------------------------------------------------------------------
# History
# 12/11/2025 S. Diop : creation (raw capture)
# 12/12/2025 Updated to build Computing_Angle/sirrah_demo for RPi
# 07/01/2026 Updated to build full Sirrah_Camera project for RPi
# -----------------------------------------------------------------------------
# https://mesonbuild.com/Cross-compilation.html
set -e  # Stop the script immediately if any command fails

# Configuration
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
BUILD_DIR="$SCRIPT_DIR/build_sirrah_camera"
CROSS_FILE="$SCRIPT_DIR/cross_rpi5.txt"
EXECUTABLE_ANGLE="sirrah_demo"
EXECUTABLE_CAPTURE="raw_capture"
ENTRYPOINT_SCRIPT="sirrah_run_v2"
CONFIG_INI="$(cd "$PROJECT_DIR/.." && pwd)/configParam.ini"

PKG_DIR="${HOME}/.cache/sirrah_camera_pkg"

# Raspberry Pi deployment target
PI_USER="raspberrypi"
PI_IP="192.168.3.13"
PI_DEST="/home/raspberrypi"

# Timestamp (YYYYMMDD_HHMMSS)
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# Meson Setup
if [ ! -d "$BUILD_DIR" ]; then
    meson setup "$BUILD_DIR" "$PROJECT_DIR" --cross-file "$CROSS_FILE" -Denable_tests=false -Denable_camera_acquisition=true
else
    echo "Build directory already exists"
    if ! meson setup --reconfigure "$BUILD_DIR" "$PROJECT_DIR" --cross-file "$CROSS_FILE" -Denable_tests=false -Denable_camera_acquisition=true; then
        echo "Reconfigure failed, wiping build directory and re-running setup"
        meson setup --wipe "$BUILD_DIR" "$PROJECT_DIR" --cross-file "$CROSS_FILE" -Denable_tests=false -Denable_camera_acquisition=true
    fi
fi

# Compilation
meson compile -C "$BUILD_DIR"

# Debian Package Creation
echo "Creating Debian package"
rm -rf "$PKG_DIR"
mkdir -p "$PKG_DIR/DEBIAN"
mkdir -p "$PKG_DIR/usr/bin"
mkdir -p "$PKG_DIR/usr/share/sirrah_camera"
mkdir -p "$PKG_DIR/usr/lib"

# Copy required files
cp "$BUILD_DIR/$EXECUTABLE_ANGLE" "$PKG_DIR/usr/bin/"
if [ -f "$BUILD_DIR/$EXECUTABLE_CAPTURE" ]; then
    cp "$BUILD_DIR/$EXECUTABLE_CAPTURE" "$PKG_DIR/usr/bin/"
fi
if [ -f "$BUILD_DIR/libsirrah_angles.so" ]; then
    cp "$BUILD_DIR/libsirrah_angles.so" "$PKG_DIR/usr/lib/"
fi
if [ -f "$PROJECT_DIR/Extraction_LED_coordinates/Image_Processing/led_centroid.py" ]; then
    cp "$PROJECT_DIR/Extraction_LED_coordinates/Image_Processing/led_centroid.py" "$PKG_DIR/usr/share/sirrah_camera/"
fi
if [ -f "$CONFIG_INI" ]; then
    cp "$CONFIG_INI" "$PKG_DIR/usr/share/sirrah_camera/"
fi
if [ -f "$PROJECT_DIR/run_sirrah_demo.sh" ]; then
    cp "$PROJECT_DIR/run_sirrah_demo.sh" "$PKG_DIR/usr/bin/$ENTRYPOINT_SCRIPT"
    # Ensure Unix line endings to avoid "/bin/bash^M" issues on Raspberry Pi.
    sed -i 's/\r$//' "$PKG_DIR/usr/bin/$ENTRYPOINT_SCRIPT"
    chmod 755 "$PKG_DIR/usr/bin/$ENTRYPOINT_SCRIPT"
fi

# Create Debian control file
cat > "$PKG_DIR/DEBIAN/control" <<EOF
Package: sirrah-camera
Version: 1.0.0
Architecture: arm64
Maintainer: Saliou Diop <serignekhadiba@gmail.com>
Conflicts: computing-angle
Replaces: computing-angle
Description: Sirrah Camera full pipeline (acq + LED + angle + TCP) for Raspberry Pi 5
EOF

# Fix permissions for Debian packaging (control dir 0755, control file 0644)
chmod 755 "$PKG_DIR/DEBIAN"
chmod 644 "$PKG_DIR/DEBIAN/control"

# Build the .deb file with timestamp
DEB_FILE="$SCRIPT_DIR/sirrah_camera_1.0.0_${TIMESTAMP}_arm64.deb"
dpkg-deb --build "$PKG_DIR" "$DEB_FILE"

if [ ! -f "$DEB_FILE" ]; then
    echo "ERROR: Debian package was not generated."
    exit 1
fi

# Transfer to Raspberry Pi
echo "Transferring .deb to Raspberry Pi ($PI_IP)"
if ! scp "$DEB_FILE" "$PI_USER@$PI_IP:$PI_DEST"; then
    echo "Transfer failed. Problem with the network connection ip"
    exit 1
fi
echo

# Automatic installation on Raspberry Pi
echo "Installing package on Raspberry Pi"

ssh "$PI_USER@$PI_IP" "
    sudo dpkg -i $PI_DEST/$(basename "$DEB_FILE")
    sudo ldconfig
"
