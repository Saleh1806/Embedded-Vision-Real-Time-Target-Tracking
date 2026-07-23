#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# Project   : SIRRAH VISION
# ----------------------------------------------------------------------------
# Title     : Global Binary Maximum Image with Local Threshold
# Author    : Serigne Saliou Mbacke Diop
# Date      : 30/09/2025
# ----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# ----------------------------------------------------------------------------
# Description :
#   This script computes a global binary maximum image from a RAW frame
#   sequence. For each frame, thresholding is adaptive and based on local
#   intensity maximum (max - 1), then all binary masks are merged by
#   pixel-wise maximum.
#
#   The output highlights bright regions that appeared at least once over
#   the full sequence.
# ----------------------------------------------------------------------------
# History :
#   30/09/2025  S.Diop : creation
# ============================================================================

# -----------------------------------------------------------------------------
# Import
# -----------------------------------------------------------------------------

import os

import cv2
import matplotlib.pyplot as plt
import numpy as np

# -----------------------------------------------------------------------------
# Configuration parameters
# -----------------------------------------------------------------------------

WIDTH = 2064
HEIGHT = 1552
NB_IMAGES = 555
BASE_NAME = r"C:\Users\Arck\Documents\OpencvApp\Test2_Tis36_ledfull\frame_ref_"
START_INDEX = 0

# -----------------------------------------------------------------------------
# Function : read_raw_frame
# Description :
#   Read one RAW frame in uint16 format, infer stride from file length, and
#   crop to active image width.
# Arguments :
#   path   -> input RAW path.
#   width  -> expected active width.
#   height -> expected active height.
# Returns :
#   np.ndarray (uint16): frame with shape (height, width).
# Raises :
#   ValueError if inferred stride is lower than width.
# -----------------------------------------------------------------------------
def read_raw_frame(path: str, width: int, height: int) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.uint16)
    stride = raw.size // height
    if stride < width:
        raise ValueError(f"{path}: stride ({stride}) < width ({width})")

    frame16 = raw[: stride * height].reshape(height, stride)[:, :width]
    return frame16


# -----------------------------------------------------------------------------
# Function : build_global_binary_max
# Description :
#   Build cumulative binary image over the full sequence.
#   Each frame uses local threshold (max_val - 1), then masks are merged by
#   taking pixel-wise maximum.
# Arguments :
#   none (uses global configuration constants).
# Returns :
#   np.ndarray (uint8): global binary maximum image.
# -----------------------------------------------------------------------------
def build_global_binary_max() -> np.ndarray:
    max_binary = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)

    for idx in range(NB_IMAGES):
        filename = f"{BASE_NAME}{START_INDEX + idx}.raw"
        print(f"Reading and thresholding {filename} ...")

        frame16 = read_raw_frame(filename, WIDTH, HEIGHT)

        _, max_val, _, _ = cv2.minMaxLoc(frame16)
        threshold_value = max(0, int(max_val) - 1)
        _, binary = cv2.threshold(frame16, threshold_value, 255, cv2.THRESH_BINARY)

        max_binary = np.maximum(max_binary, binary.astype(np.uint8))

    print(f"\nGlobal binary maximum image computed from {NB_IMAGES} frames.")
    return max_binary


# -----------------------------------------------------------------------------
# Function : display_result
# Description :
#   Display the global binary maximum image with axes and grid.
# Arguments :
#   max_binary -> np.ndarray (uint8), cumulative binary mask.
# Returns :
#   none
# -----------------------------------------------------------------------------
def display_result(max_binary: np.ndarray) -> None:
    plt.figure(figsize=(10, 8))
    plt.imshow(max_binary, cmap="gray")
    plt.title(f"Global binary maximum image (local threshold = max - 1)\n{NB_IMAGES} frames")
    plt.xlabel("X (pixels)")
    plt.ylabel("Y (pixels)")
    plt.colorbar(label="Pixel value (0 or 255)")
    plt.grid(color="red", linestyle="--", linewidth=0.4)
    plt.tight_layout()
    plt.show()


# -----------------------------------------------------------------------------
# Function : save_result
# Description :
#   Save the global binary maximum image in the same folder as input files.
# Arguments :
#   max_binary -> np.ndarray (uint8), cumulative binary mask.
#   base_name  -> input base file path/prefix used to derive output folder.
# Returns :
#   str: full output path of saved image.
# -----------------------------------------------------------------------------
def save_result(max_binary: np.ndarray, base_name: str) -> str:
    output_folder = os.path.dirname(base_name)
    if output_folder == "":
        output_folder = "."

    output_path = os.path.join(output_folder, "max_binary_local_threshold.png")
    cv2.imwrite(output_path, max_binary)
    print(f"Binary max image saved as '{output_path}'")
    return output_path


# -----------------------------------------------------------------------------
# Function : main
# Description :
#   Execute complete workflow:
#     1) build global binary image,
#     2) display result,
#     3) save result image on disk.
# Arguments :
#   none
# Returns :
#   none
# -----------------------------------------------------------------------------
def main() -> None:
    max_binary = build_global_binary_max()
    display_result(max_binary)
    save_result(max_binary, BASE_NAME)


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    main()

# end of file
