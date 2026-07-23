#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# Project   : SIRRAH VISION
# ----------------------------------------------------------------------------
# Title     : RAW 10-bit Image Loader and Viewer (SRGGB10)
# Author    : Serigne Saliou Mbacké Diop
# Date      : 30/09/2025
# ----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# ----------------------------------------------------------------------------
# Description :
#   This script reads and visualizes a single 10-bit RAW image (SRGGB10 format)
#   captured from the Global Shutter camera.
#   It performs the following steps:
#     1. Reads a RAW file as 16-bit unsigned integers,
#     2. Detects potential stride (padding per line),
#     3. Extracts the 10 meaningful bits,
#     4. Converts the result to 8-bit for visualization.
#
#   Refer to BERT_296 for implementation and test references.
# ----------------------------------------------------------------------------
# History :
#   30/09/2025  S.Diop : creation
# ============================================================================

# -----------------------------------------------------------------------------
# Import
# -----------------------------------------------------------------------------

import numpy as np
import cv2

# -----------------------------------------------------------------------------
# Configuration parameters
# -----------------------------------------------------------------------------

width = 1456
height = 1088
pixel_dtype = np.uint16          # 16-bit storage, 10-bit data
filename = "frame_0.raw"         # input RAW file path

# -----------------------------------------------------------------------------
# Main script
# -----------------------------------------------------------------------------
def main():
    """Read and display a 10-bit RAW (SRGGB10) grayscale image."""

    # Read RAW file
    with open(filename, "rb") as f:
        raw = np.fromfile(f, dtype=pixel_dtype)

    # Compute stride (pixels per line)
    stride = raw.size // height
    if stride < width:
        raise ValueError(f"RAW file too small: {raw.size} pixels for {height} lines")
    elif stride > width:
        print(f"Detected stride: {stride} pixels per line (padding detected)")

    # Reshape considering stride
    raw_strided = raw[:stride * height].reshape((height, stride))

    # Keep only the useful pixels (remove line padding)
    raw_image = raw_strided[:, :width]

    # Extract 10 meaningful bits (ignore unused bits)
    raw_10bit = raw_image & 0xFFC0

    # Convert to 8-bit for display
    raw_8bit = (raw_10bit / 1023.0 * 255).astype(np.uint8)

    # Display grayscale image
    cv2.imshow("SRGGB10 RAW 10-bit", raw_8bit)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    main()

# end of file
