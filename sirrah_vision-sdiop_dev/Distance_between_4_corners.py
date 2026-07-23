#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# Project   : SIRRAH VISION
# ----------------------------------------------------------------------------
# Title     : Pixel Size Estimation from Extreme LED Pairs
# Author    : Serigne Saliou Mbacke Diop
# Date      : 30/09/2025
# ----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# ----------------------------------------------------------------------------
# Description :
#   This script estimates pixel size (mm/pixel) using distances between
#   extreme LED pairs on a detected LED grid.
#
#   Workflow:
#     1. Build a cumulative binary image from RAW sequence,
#     2. Detect LED centroids with connected components,
#     3. Evaluate predefined extreme pairs (top/right/bottom/left),
#     4. Convert pixel distances to mm/pixel,
#     5. Compute statistics and display plots.
# ----------------------------------------------------------------------------
# History :
#   30/09/2025  S.Diop : creation
# ============================================================================

# -----------------------------------------------------------------------------
# Import
# -----------------------------------------------------------------------------

import cv2
import matplotlib.pyplot as plt
import numpy as np

# -----------------------------------------------------------------------------
# Configuration parameters
# -----------------------------------------------------------------------------

# RAW frame geometry.
WIDTH = 2064
HEIGHT = 1552

# Sequence definition.
NB_IMAGES = 555
BASE_NAME = r"C:\Users\Arck\Documents\OpencvApp\Test2_Tis36_ledfull\frame_ref_"
START_INDEX = 1

# Minimal connected component area (noise rejection).
MIN_BLOB_AREA_PX = 2

# Physical setup.
REAL_LED_SPACING_MM = 60.0
NUM_INTERVALS = 8
TOTAL_REAL_DISTANCE_MM = NUM_INTERVALS * REAL_LED_SPACING_MM

# Extreme pair definitions using 1-based LED indexing.
PAIRS_TO_CHECK = [
    (1, 9, "Top (1<->9)"),
    (9, 81, "Right (9<->81)"),
    (73, 81, "Bottom (73<->81)"),
    (1, 73, "Left (1<->73)"),
]

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def read_raw_frame(path: str, width: int, height: int) -> np.ndarray:
    """Read one RAW frame with stride handling and active-area cropping."""
    raw = np.fromfile(path, dtype=np.uint16)
    stride = raw.size // height
    if stride < width:
        raise ValueError(f"{path}: stride ({stride}) < width ({width})")

    frame16 = raw[: stride * height].reshape(height, stride)[:, :width]
    return frame16


# -----------------------------------------------------------------------------
# Function : build_global_binary_max
# Description :
#   Build a cumulative binary image from the full RAW sequence.
#   Each frame is thresholded near its local maximum, then merged with a
#   pixel-wise maximum operation.
# Arguments :
#   none (uses configured globals: NB_IMAGES, BASE_NAME, START_INDEX, etc.)
# Returns :
#   np.ndarray (uint8): cumulative binary mask of detected bright pixels.
# -----------------------------------------------------------------------------
def build_global_binary_max() -> np.ndarray:
    """Build cumulative binary image using local max threshold per frame."""
    max_binary = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)

    for idx in range(NB_IMAGES):
        filename = f"{BASE_NAME}{START_INDEX + idx}.raw"
        print(f"Reading {filename} ...")

        frame16 = read_raw_frame(filename, WIDTH, HEIGHT)

        # Threshold close to local maximum to isolate bright LED pixels.
        _, max_val, _, _ = cv2.minMaxLoc(frame16)
        threshold_value = max(0, int(max_val) - 1)
        _, binary = cv2.threshold(frame16, threshold_value, 255, cv2.THRESH_BINARY)

        max_binary = np.maximum(max_binary, binary.astype(np.uint8))

    print(f"\nGlobal binary maximum image computed from {NB_IMAGES} frames.")
    return max_binary


# -----------------------------------------------------------------------------
# Function : detect_centroids
# Description :
#   Detect connected components on the cumulative binary mask and return LED
#   centroids after area-based filtering.
# Arguments :
#   max_binary -> np.ndarray (uint8), cumulative binary image.
# Returns :
#   np.ndarray: centroids array (N x 2) in pixel coordinates.
# Raises :
#   RuntimeError if no valid LED is detected.
# -----------------------------------------------------------------------------
def detect_centroids(max_binary: np.ndarray) -> np.ndarray:
    """Detect LED centroids from cumulative binary image with area filtering."""
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(max_binary)
    if num_labels <= 1:
        raise RuntimeError("No LED detected.")

    # Remove background component (label 0).
    areas = stats[1:, cv2.CC_STAT_AREA]
    centroids_fg = centroids[1:]

    valid_mask = areas >= MIN_BLOB_AREA_PX
    filtered_out = int(np.sum(~valid_mask))
    centroids_valid = centroids_fg[valid_mask]

    if len(centroids_valid) == 0:
        raise RuntimeError("No LED detected after area filtering.")

    if filtered_out > 0:
        print(f"\nFiltered out {filtered_out} tiny blob(s) with area < {MIN_BLOB_AREA_PX} px.")

    print(f"\nDetected {len(centroids_valid)} LEDs:")
    for i, (cx, cy) in enumerate(centroids_valid, start=1):
        print(f"LED {i}: centroid = ({cx:.2f}, {cy:.2f})")

    return centroids_valid


# -----------------------------------------------------------------------------
# Function : compute_extreme_pair_results
# Description :
#   Evaluate predefined extreme LED pairs and compute:
#     - distance in pixels,
#     - equivalent distance in millimeters,
#     - estimated pixel size (mm/pixel).
# Arguments :
#   centroids -> np.ndarray (N x 2), detected LED centroids.
# Returns :
#   list of tuples (i, j, label, distance_px, distance_mm, pixel_size_mm_per_px).
# -----------------------------------------------------------------------------
def compute_extreme_pair_results(
    centroids: np.ndarray,
) -> list[tuple[int, int, str, float, float, float]]:
    """Compute pixel/mm distances and mm/pixel for predefined extreme pairs."""
    print("\n=== DISTANCES BETWEEN EXTREME LED PAIRS ===")
    results: list[tuple[int, int, str, float, float, float]] = []

    for (i, j, label) in PAIRS_TO_CHECK:
        if i <= len(centroids) and j <= len(centroids):
            cx_i, cy_i = centroids[i - 1]
            cx_j, cy_j = centroids[j - 1]

            distance_px = float(np.hypot(cx_i - cx_j, cy_i - cy_j))
            pixel_size_mm = TOTAL_REAL_DISTANCE_MM / distance_px
            distance_mm = distance_px * pixel_size_mm

            results.append((i, j, label, distance_px, distance_mm, pixel_size_mm))
            print(
                f"{label}: {distance_px:.2f} px -> {distance_mm:.2f} mm "
                f"-> {pixel_size_mm:.6f} mm/px"
            )
        else:
            print(f"{label}: indices out of range.")

    return results


# -----------------------------------------------------------------------------
# Function : display_results
# Description :
#   Print global statistics on estimated pixel sizes and display two plots:
#     - LED map with annotated extreme links,
#     - histogram of mm/pixel values.
# Arguments :
#   max_binary -> np.ndarray, cumulative binary image.
#   centroids -> np.ndarray, detected LED centroids.
#   results -> list of computed pair metrics.
# Returns :
#   none
# Raises :
#   RuntimeError if no valid result is available.
# -----------------------------------------------------------------------------
def display_results(
    max_binary: np.ndarray,
    centroids: np.ndarray,
    results: list[tuple[int, int, str, float, float, float]],
) -> None:
    """Display geometric overlay and histogram of estimated pixel sizes."""
    if not results:
        raise RuntimeError("No valid pair available for statistics.")

    pixel_sizes = np.array([row[5] for row in results], dtype=np.float64)
    avg_pixel_size = float(np.mean(pixel_sizes))
    std_pixel_size = float(np.std(pixel_sizes))
    cv_pixel_size = (std_pixel_size / avg_pixel_size) * 100.0

    print("\n=== PIXEL SIZE STATISTICS ===")
    print(f"Mean pixel size : {avg_pixel_size:.6f} mm/px")
    print(f"Standard dev.   : {std_pixel_size:.6f} mm/px")
    print(f"Coeff. variation: {cv_pixel_size:.3f} %")

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    plt.suptitle("Extreme-LED distances and pixel-size statistics", fontsize=14, weight="bold")

    # Left panel: LED map and annotated extreme links.
    ax1 = axes[0]
    ax1.imshow(max_binary, cmap="gray")
    ax1.set_title("Extreme LED distances (px and mm)")
    ax1.set_xlabel("X (pixels)")
    ax1.set_ylabel("Y (pixels)")

    for i, (cx, cy) in enumerate(centroids, start=1):
        ax1.plot(cx, cy, "ro")
        ax1.text(cx + 8, cy, f"{i}", color="red", fontsize=8)

    for (i, j, label, distance_px, distance_mm, _) in results:
        x1, y1 = centroids[i - 1]
        x2, y2 = centroids[j - 1]
        ax1.plot([x1, x2], [y1, y2], "y-", linewidth=1.2)

        mid_x, mid_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        ax1.text(
            mid_x,
            mid_y,
            f"{label}\n{distance_px:.1f} px\n{distance_mm:.1f} mm",
            color="yellow",
            fontsize=9,
            ha="center",
            va="center",
            bbox=dict(facecolor="black", alpha=0.5, edgecolor="none", pad=3),
        )

    ax1.grid(color="red", linestyle="--", linewidth=0.4)

    # Right panel: histogram of mm/pixel estimates.
    ax2 = axes[1]
    ax2.hist(pixel_sizes, bins=10, color="skyblue", edgecolor="black", alpha=0.8)
    ax2.axvline(
        avg_pixel_size,
        color="red",
        linestyle="--",
        linewidth=1.5,
        label=f"Mean = {avg_pixel_size:.5f} mm/px",
    )
    ax2.axvline(
        avg_pixel_size + std_pixel_size,
        color="green",
        linestyle=":",
        label=f"+1sigma = {(avg_pixel_size + std_pixel_size):.5f} mm/px",
    )
    ax2.axvline(
        avg_pixel_size - std_pixel_size,
        color="green",
        linestyle=":",
        label=f"-1sigma = {(avg_pixel_size - std_pixel_size):.5f} mm/px",
    )

    ax2.set_xlabel("Pixel size (mm/pixel)")
    ax2.set_ylabel("Frequency")
    ax2.set_title("Pixel-size distribution")
    ax2.legend(fontsize=9)
    ax2.grid(alpha=0.3)

    summary_text = (
        f"Mean: {avg_pixel_size:.6f} mm/px\n"
        f"Std: {std_pixel_size:.6f} mm/px\n"
        f"CV: {cv_pixel_size:.3f} %"
    )
    ax2.text(
        0.98,
        0.95,
        summary_text,
        transform=ax2.transAxes,
        fontsize=10,
        va="top",
        ha="right",
        bbox=dict(facecolor="white", alpha=0.8, edgecolor="gray"),
    )

    plt.tight_layout(rect=[0, 0, 1, 0.94])
    plt.show()

# -----------------------------------------------------------------------------
# Function : main
# Description :
#   Execute full workflow:
#     1) cumulative binary generation,
#     2) centroid detection,
#     3) extreme-pair distance computation,
#     4) statistics and visualization.
# Arguments :
#   none
# Returns :
#   none
# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

def main() -> None:
    """Run complete extreme-pair pixel-size estimation workflow."""
    max_binary = build_global_binary_max()
    centroids = detect_centroids(max_binary.astype(np.uint8))
    results = compute_extreme_pair_results(centroids)
    display_results(max_binary, centroids, results)


if __name__ == "__main__":
    main()

# end of file
