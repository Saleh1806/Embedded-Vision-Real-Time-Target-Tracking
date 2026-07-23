#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# Project   : SIRRAH VISION
# ----------------------------------------------------------------------------
# Title     : RAW Image Display and LED Blob Selection
# Author    : Serigne Saliou Mbacke Diop
# Date      : 30/09/2025
# ----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# ----------------------------------------------------------------------------
# Description :
#   This script analyzes one RAW frame to visualize LED-like bright blobs.
#   Processing steps:
#     1. Read RAW data with stride handling,
#     2. Build a local threshold near image maximum,
#     3. Clean the binary mask with morphology,
#     4. Keep connected components within area limits,
#     5. Generate and save debug images (mask + overlay),
#     6. Display final visualization panels.
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

# Expected RAW frame geometry from the acquisition pipeline.
WIDTH = 1456
HEIGHT = 1088

# Input RAW file (single-image mode).
RAW_FILE = (
    r"C:\Users\Arck\Documents\OpencvApp\Donnees_Images_Test_SIRRAH_VISION"
    r"\Bonnes_images_Decalage_angualire_test\Test_exterieur\Test_exterieur_40m"
    r"\ET_50\capture_ET50_#1.raw"
)

# Threshold strategy: keep pixels close to frame maximum.
# Effective threshold = max(frame) - DELTA_FROM_MAX.
DELTA_FROM_MAX = 3

# Area filtering for connected components.
MIN_AREA = 1
MAX_AREA = 500

# -----------------------------------------------------------------------------
# RAW loading and visualization helpers
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Function : read_raw_frame
# Description :
#   Read a RAW frame in uint16 format, infer stride, and crop to active width.
# Arguments :
#   path   -> input RAW file path.
#   width  -> expected active width in pixels.
#   height -> expected active height in pixels.
# Returns :
#   np.ndarray (uint16): frame of shape (height, width).
# Raises :
#   ValueError if file is empty or inferred stride is invalid.
# -----------------------------------------------------------------------------
def read_raw_frame(path: str, width: int, height: int) -> np.ndarray:
    """Read RAW frame as uint16 and crop active region using inferred stride."""
    raw = np.fromfile(path, dtype=np.uint16)
    if raw.size == 0:
        raise ValueError(f"{path}: empty file")

    stride = raw.size // height
    if stride < width:
        raise ValueError(f"{path}: stride ({stride}) < width ({width})")

    frame16 = raw[: stride * height].reshape((height, stride))[:, :width]
    return frame16


# -----------------------------------------------------------------------------
# Function : display_stretch_u8
# Description :
#   Convert uint16 image to a readable uint8 visualization using percentile
#   clipping and gamma correction.
# Arguments :
#   img16  -> input uint16 image.
#   p_low  -> lower percentile for clipping.
#   p_high -> upper percentile for clipping.
#   gamma  -> gamma correction factor.
# Returns :
#   np.ndarray (uint8): display-ready image.
# -----------------------------------------------------------------------------
def display_stretch_u8(
    img16: np.ndarray,
    p_low: float = 0.2,
    p_high: float = 99.8,
    gamma: float = 0.45,
) -> np.ndarray:
    """Convert uint16 image to readable uint8 using percentile stretch + gamma.

    This is only for display/debug. Detection still runs on the RAW-domain data.
    """
    p1, p99 = np.percentile(img16, [p_low, p_high])
    if p99 <= p1:
        vis16 = cv2.normalize(img16, None, 0, 65535, cv2.NORM_MINMAX)
    else:
        clipped = np.clip(img16.astype(np.float32), p1, p99)
        vis16 = ((clipped - p1) * (65535.0 / (p99 - p1))).astype(np.uint16)

    vis16 = ((vis16.astype(np.float32) / 65535.0) ** gamma * 65535.0).astype(np.uint16)
    vis8 = cv2.convertScaleAbs(vis16, alpha=255.0 / 65535.0)
    return vis8

# -----------------------------------------------------------------------------
# Main script
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Function : main
# Description :
#   Execute complete single-image workflow:
#     1) read RAW frame,
#     2) threshold and clean mask,
#     3) filter connected components,
#     4) build overlay and save debug outputs,
#     5) display final panels.
# Arguments :
#   none
# Returns :
#   none
# -----------------------------------------------------------------------------
def main() -> None:
    """Run single-frame LED blob detection and visualization."""
    if not os.path.exists(RAW_FILE):
        raise FileNotFoundError(f"Missing RAW file: {RAW_FILE}")

    # 1) Load RAW frame.
    frame16 = read_raw_frame(RAW_FILE, WIDTH, HEIGHT)

    # 2) Build binary mask around the brightest levels in the frame.
    _, max_val, _, _ = cv2.minMaxLoc(frame16)
    threshold_value = max(0, int(max_val) - DELTA_FROM_MAX)
    _, binary16 = cv2.threshold(frame16, threshold_value, 65535, cv2.THRESH_BINARY)
    max_binary = (binary16 > 0).astype(np.uint8) * 255

    # 3) Clean mask with opening and light dilation to stabilize blobs.
    kernel = np.ones((3, 3), np.uint8)
    mask_clean = cv2.morphologyEx(max_binary, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.dilate(mask_clean, kernel, iterations=1)

    # 4) Keep components within area limits.
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_clean)
    selected_mask = np.zeros_like(mask_clean)
    centers = []

    for label in range(1, num_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if MIN_AREA <= area <= MAX_AREA:
            selected_mask[labels == label] = 255
            cx, cy = centroids[label]
            centers.append((int(round(cx)), int(round(cy)), area))

    print(f"Detected blobs after filtering: {len(centers)}")

    # 5) Build debug overlay on display-stretched background.
    background_u8 = display_stretch_u8(frame16)
    overlay = cv2.cvtColor(background_u8, cv2.COLOR_GRAY2BGR)

    contours, _ = cv2.findContours(selected_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 0, 255), 1)
    for (cx, cy, area) in centers:
        cv2.circle(overlay, (cx, cy), 4, (0, 255, 255), -1)
        cv2.putText(
            overlay,
            f"{area}",
            (cx + 5, cy - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    # 6) Save debug outputs next to source RAW file.
    out_dir = os.path.dirname(RAW_FILE) or "."
    path_binary = os.path.join(out_dir, "single_binary_local_threshold.png")
    path_selected = os.path.join(out_dir, "leds_selected_mask.png")
    path_overlay = os.path.join(out_dir, "leds_selected_overlay.png")

    cv2.imwrite(path_binary, max_binary)
    cv2.imwrite(path_selected, selected_mask)
    cv2.imwrite(path_overlay, overlay)

    print(f"Saved: {path_binary}")
    print(f"Saved: {path_selected}")
    print(f"Saved: {path_overlay}")

    # 7) Display consolidated visualization.
    plt.figure(figsize=(14, 5))

    plt.subplot(1, 3, 1)
    plt.imshow(background_u8, cmap="gray")
    plt.title("Frame (stretched)")
    plt.axis("off")

    plt.subplot(1, 3, 2)
    plt.imshow(selected_mask, cmap="gray")
    plt.title("LED mask")
    plt.axis("off")

    plt.subplot(1, 3, 3)
    plt.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
    plt.title("LED overlay")
    plt.axis("off")

    plt.tight_layout()
    plt.show()

# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    main()

# end of file
