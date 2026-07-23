#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# Project   : SIRRAH VISION
# ----------------------------------------------------------------------------
# Title     : RAW Image Sequence Analysis and Spot Jitter Evaluation
# Author    : Serigne Saliou Mbacké Diop
# Date      : 30/09/2025
# ----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# ----------------------------------------------------------------------------
# Description :
#   This script processes a sequence of RAW images to:
#     1. Detect the bright spot in each frame,
#     2. Compute the centroid position across all images,
#     3. Evaluate the stability and jitter over time,
#     4. Visualize the average image, centroid trajectory,
#        and jitter distribution.
#
#   Refer to BERT_296 for additional technical details.
# ----------------------------------------------------------------------------
# History :
#   30/09/2025  S.Diop : creation
# ============================================================================

# -----------------------------------------------------------------------------
# Import
# -----------------------------------------------------------------------------

import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------------
# Configuration parameters
# -----------------------------------------------------------------------------

width, height = 1456, 1088
nb_images = 198             # number of images to process
base_name = "led_centre_G1_ET50000/frame_centre_G1_ET50000"  # common prefix
start_index = 0             # starting index
threshold_value = 200       # binary threshold to detect the LED spot

# -----------------------------------------------------------------------------
# Function definitions
# -----------------------------------------------------------------------------
def load_raw(filename, width, height):
    """Load a single RAW image and convert it to 8-bit grayscale."""
    raw = np.fromfile(filename, dtype=np.uint16)
    stride = raw.size // height
    if stride < width:
        raise ValueError(f"{filename}: stride ({stride}) < width ({width})")

    raw_image = raw[:stride * height].reshape(height, stride)[:, :width]
    raw_8bit = cv2.convertScaleAbs(raw_image, alpha=255.0 / raw_image.max())
    return raw_8bit


# -----------------------------------------------------------------------------
# Main script
# -----------------------------------------------------------------------------
def main():
    """Main routine for jitter computation from a RAW image sequence."""

    centres = []
    binaries = []

    # -------------------------------
    # Compute centroid for each image
    # -------------------------------
    for i in range(nb_images):
        fname = f"{base_name}{start_index + i}.raw"
        img = load_raw(fname, width, height)

        # Threshold to isolate the LED spot
        _, binary = cv2.threshold(img, threshold_value, 255, cv2.THRESH_BINARY)
        binaries.append(binary)

        # Compute centroid of the bright region
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)
        if num_labels <= 1:
            print(f"{fname}: no spot detected")
            continue

        # Assume LED is the largest region (excluding background)
        idx = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        cx, cy = centroids[idx]
        centres.append((cx, cy))

        print(f"{fname}: centroid = (x={cx:.2f}, y={cy:.2f})")

    centres = np.array(centres)
    print(f"\n{len(centres)} centroids detected over {nb_images} images")

    # -------------------------------
    # Compute jitter statistics
    # -------------------------------
    cx0, cy0 = centres[0]
    dx = centres[:, 0] - cx0
    dy = centres[:, 1] - cy0
    r = np.sqrt(dx**2 + dy**2)

    stats = {
        "mean_dx": np.mean(dx),
        "mean_dy": np.mean(dy),
        "std_dx": np.std(dx),
        "std_dy": np.std(dy),
        "rms_jitter": np.sqrt(np.mean(dx**2 + dy**2)),
        "max_jitter": np.max(r),
        "p95_jitter": np.percentile(r, 95)
    }

    print("\n=== JITTER STATISTICS (in pixels) ===")
    for k, v in stats.items():
        print(f"{k:>12}: {v:.4f}")

    # -------------------------------
    # Visualization
    # -------------------------------
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # Average image
    mean_image = np.mean(binaries, axis=0).astype(np.uint8)
    axes[0].imshow(mean_image, cmap='gray')
    axes[0].set_title(f"Average Image ({nb_images} frames)")
    axes[0].grid(color='red', linestyle='--', linewidth=0.4)
    axes[0].set_xlabel("X (pixels)")
    axes[0].set_ylabel("Y (pixels)")

    # Centroid trajectory
    axes[1].plot(dx, dy, '-o')
    axes[1].axhline(0, color='k', lw=0.5)
    axes[1].axvline(0, color='k', lw=0.5)
    axes[1].set_title("Centroid Trajectory (ΔX, ΔY)")
    axes[1].set_xlabel("ΔX [px]")
    axes[1].set_ylabel("ΔY [px]")
    axes[1].set_aspect('equal', 'box')
    axes[1].grid(True, linestyle='--')

    # Radial jitter histogram
    axes[2].hist(r, bins=20, color='skyblue', edgecolor='k')
    axes[2].set_title(
        f"Radial Jitter Distribution\nRMS={stats['rms_jitter']:.3f}px, 95%={stats['p95_jitter']:.3f}px"
    )
    axes[2].set_xlabel("Radial displacement [pixels]")
    axes[2].set_ylabel("Frame count")

    plt.tight_layout()
    plt.show()


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    main()

# end of file
