#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# Project   : SIRRAH VISION
# ----------------------------------------------------------------------------
# Title     : LED Grid Detection and Pixel Size Estimation
# Author    : Serigne Saliou Mbacke Diop
# Date      : 30/09/2025
# ----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# ----------------------------------------------------------------------------
# Description :
#   This script automatically detects LED positions from a sequence of RAW
#   images, determines their centroids, and computes average spacing between
#   neighboring LEDs to estimate pixel size (mm/pixel).
#
#   Main steps:
#     1. Generate a maximum binary map over all frames,
#     2. Detect LED centroids from connected components,
#     3. Compute nearest-neighbor distances with consistency filtering,
#     4. Estimate pixel size from known physical spacing,
#     5. Display geometric and statistical visualizations.
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

WIDTH = 2064
HEIGHT = 1552
NB_IMAGES = 555
BASE_NAME = r"C:\Users\Arck\Documents\OpencvApp\Test2_Tis36_ledfull\frame_ref_"
START_INDEX = 0
TOLERANCE_PX = 5.0
MIN_BLOB_AREA_PX = 2
KNOWN_DISTANCE_MM = 60.0

# -----------------------------------------------------------------------------
# Function : read_raw_frame
# Description :
#   Read one RAW frame as uint16, infer stride, and crop to active image width.
# Arguments :
#   path   -> input RAW file path.
#   width  -> expected active width in pixels.
#   height -> expected active height in pixels.
# Returns :
#   np.ndarray (uint16): frame of shape (height, width).
# Raises :
#   ValueError if inferred stride is smaller than width.
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
#   Build a cumulative binary image from the RAW sequence.
#   Each frame is thresholded near its local maximum, then merged using
#   pixel-wise maximum.
# Arguments :
#   none (uses global configuration constants).
# Returns :
#   np.ndarray (uint8): maximum binary image over all frames.
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
    print("Local threshold used: (max_val - 1) per frame.")
    return max_binary


# -----------------------------------------------------------------------------
# Function : detect_centroids
# Description :
#   Detect connected components from cumulative binary image and keep valid LED
#   centroids after area filtering.
# Arguments :
#   max_binary -> np.ndarray (uint8), cumulative binary image.
# Returns :
#   np.ndarray: centroids array (N x 2).
# Raises :
#   RuntimeError if no valid LED remains after filtering.
# -----------------------------------------------------------------------------
def detect_centroids(max_binary: np.ndarray) -> np.ndarray:
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(max_binary)
    if num_labels <= 1:
        raise RuntimeError("No LED detected.")

    areas = stats[1:, cv2.CC_STAT_AREA]
    centroids_fg = centroids[1:]

    valid_mask = areas >= MIN_BLOB_AREA_PX
    filtered_out = int(np.sum(~valid_mask))
    centroids_valid = centroids_fg[valid_mask]

    if len(centroids_valid) == 0:
        raise RuntimeError("No LED detected after area filtering.")

    if filtered_out > 0:
        print(f"\nFiltered out {filtered_out} tiny blob(s) with area < {MIN_BLOB_AREA_PX} px.")

    print(f"\nDetected {len(centroids_valid)} LED(s):")
    for i, (cx, cy) in enumerate(centroids_valid, start=1):
        print(f"LED {i}: centroid = ({cx:.2f}, {cy:.2f})")

    return centroids_valid


# -----------------------------------------------------------------------------
# Function : compute_neighbor_distances
# Description :
#   For each centroid, compute the 4 nearest neighbors and keep links that are
#   distance-consistent under a tolerance threshold.
# Arguments :
#   centroids -> np.ndarray (N x 2), detected LED centroids.
# Returns :
#   list[tuple[int, int, float]]: (source_idx_1based, target_idx_1based, dist_px).
# -----------------------------------------------------------------------------
def compute_neighbor_distances(centroids: np.ndarray) -> list[tuple[int, int, float]]:
    distances_neighbors: list[tuple[int, int, float]] = []

    for i in range(len(centroids)):
        cx_i, cy_i = centroids[i]

        dists = []
        for j in range(len(centroids)):
            if i == j:
                continue
            cx_j, cy_j = centroids[j]
            d = float(np.hypot(cx_i - cx_j, cy_i - cy_j))
            dists.append((j, d))

        dists.sort(key=lambda item: item[1])
        nearest_4 = dists[:4]
        if not nearest_4:
            continue

        distances_only = [d for (_, d) in nearest_4]
        if max(distances_only) - min(distances_only) <= TOLERANCE_PX:
            valid_neighbors = nearest_4
        else:
            median_d = float(np.median(distances_only))
            valid_neighbors = [
                (j, d) for (j, d) in nearest_4 if abs(d - median_d) <= TOLERANCE_PX
            ]

        for (j, d) in valid_neighbors:
            distances_neighbors.append((i + 1, j + 1, d))

    return distances_neighbors


# -----------------------------------------------------------------------------
# Function : compute_statistics
# Description :
#   Compute summary statistics from neighbor distances and derive pixel size.
# Arguments :
#   distances_neighbors -> list of validated neighbor links with pixel distances.
#   known_distance_mm   -> real physical spacing between neighboring LEDs.
# Returns :
#   tuple (dists_px, avg_dist_px, std_dist_px, cv_percent,
#          pixel_size_mm, std_pixel_size_mm)
# Raises :
#   RuntimeError if no consistent neighbors are available.
# -----------------------------------------------------------------------------
def compute_statistics(
    distances_neighbors: list[tuple[int, int, float]],
    known_distance_mm: float,
) -> tuple[np.ndarray, float, float, float, float, float]:
    if not distances_neighbors:
        raise RuntimeError("No consistent neighbors found.")

    dists_px = np.array([d for (_, _, d) in distances_neighbors], dtype=np.float64)

    avg_dist_px = float(np.mean(dists_px))
    std_dist_px = float(np.std(dists_px))
    cv_percent = (std_dist_px / avg_dist_px) * 100.0

    pixel_size_mm = known_distance_mm / avg_dist_px
    std_pixel_size_mm = (known_distance_mm / (avg_dist_px ** 2)) * std_dist_px

    return dists_px, avg_dist_px, std_dist_px, cv_percent, pixel_size_mm, std_pixel_size_mm


# -----------------------------------------------------------------------------
# Function : print_statistics
# Description :
#   Print formatted statistics block for neighbor distances and pixel size.
# Arguments :
#   dists_px          -> np.ndarray of distances in pixels.
#   avg_dist_px       -> mean distance in pixels.
#   std_dist_px       -> standard deviation in pixels.
#   cv_percent        -> coefficient of variation in percent.
#   pixel_size_mm     -> estimated pixel size in mm/pixel.
#   std_pixel_size_mm -> uncertainty-like spread in mm/pixel.
# Returns :
#   none
# -----------------------------------------------------------------------------
def print_statistics(
    dists_px: np.ndarray,
    avg_dist_px: float,
    std_dist_px: float,
    cv_percent: float,
    pixel_size_mm: float,
    std_pixel_size_mm: float,
) -> None:
    print("\n================= STATISTICS =================")
    print(f"Number of pairs          : {len(dists_px)}")
    print(f"Mean distance            : {avg_dist_px:.2f} px")
    print(f"Std distance             : {std_dist_px:.2f} px")
    print(f"Coefficient of variation : {cv_percent:.3f} %")
    print(f"Mean pixel size          : {pixel_size_mm:.6f} +/- {std_pixel_size_mm:.6f} mm/pixel")
    print("=============================================")


# -----------------------------------------------------------------------------
# Function : display_results
# Description :
#   Display two subplots:
#     - LED positions and validated neighbor links,
#     - histogram of neighbor distances with key statistics.
# Arguments :
#   max_binary         -> cumulative binary image.
#   centroids          -> detected LED centroids.
#   distances_neighbors-> validated neighbor links.
#   dists_px           -> distances array.
#   avg_dist_px        -> mean distance.
#   std_dist_px        -> standard deviation.
#   cv_percent         -> coefficient of variation.
#   pixel_size_mm      -> estimated mm/pixel.
# Returns :
#   none
# -----------------------------------------------------------------------------
def display_results(
    max_binary: np.ndarray,
    centroids: np.ndarray,
    distances_neighbors: list[tuple[int, int, float]],
    dists_px: np.ndarray,
    avg_dist_px: float,
    std_dist_px: float,
    cv_percent: float,
    pixel_size_mm: float,
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    plt.suptitle("Analysis of neighboring LED distances", fontsize=14, weight="bold")

    ax1 = axes[0]
    ax1.imshow(max_binary, cmap="gray")
    ax1.set_title("Detected positions and consistent links", fontsize=12)
    ax1.set_xlabel("X (pixels)")
    ax1.set_ylabel("Y (pixels)")

    for i, (cx, cy) in enumerate(centroids, start=1):
        ax1.plot(cx, cy, "ro")
        ax1.text(cx + 10, cy, f"{i}", color="red", fontsize=8)

    for (i, j, _) in distances_neighbors:
        x1, y1 = centroids[i - 1]
        x2, y2 = centroids[j - 1]
        ax1.plot([x1, x2], [y1, y2], "y--", linewidth=0.8)

    ax1.grid(color="red", linestyle="--", linewidth=0.3)

    ax2 = axes[1]
    ax2.hist(dists_px, bins=20, color="skyblue", edgecolor="black", alpha=0.8)
    ax2.axvline(avg_dist_px, color="red", linestyle="--", linewidth=1.5, label=f"Mean = {avg_dist_px:.2f}px")
    ax2.axvline(
        avg_dist_px + std_dist_px,
        color="green",
        linestyle=":",
        linewidth=1.2,
        label=f"+1sigma = {avg_dist_px + std_dist_px:.2f}px",
    )
    ax2.axvline(
        avg_dist_px - std_dist_px,
        color="green",
        linestyle=":",
        linewidth=1.2,
        label=f"-1sigma = {avg_dist_px - std_dist_px:.2f}px",
    )

    ax2.set_title("Distance distribution (pixels)", fontsize=12)
    ax2.set_xlabel("Distance (px)")
    ax2.set_ylabel("Frequency")
    ax2.legend(fontsize=9)
    ax2.grid(alpha=0.3)

    text = (
        f"Mean: {avg_dist_px:.2f} px\n"
        f"Std: {std_dist_px:.2f} px\n"
        f"CV: {cv_percent:.3f} %\n"
        f"Pixel size: {pixel_size_mm:.6f} mm/px"
    )
    ax2.text(
        0.98,
        0.95,
        text,
        transform=ax2.transAxes,
        fontsize=10,
        va="top",
        ha="right",
        bbox=dict(facecolor="white", alpha=0.8, edgecolor="gray"),
    )

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


# -----------------------------------------------------------------------------
# Function : main
# Description :
#   Execute the complete centroid-distance workflow:
#     1) cumulative binary generation,
#     2) centroid extraction,
#     3) neighbor distance filtering,
#     4) statistics and visualization.
# Arguments :
#   none
# Returns :
#   none
# -----------------------------------------------------------------------------
def main() -> None:
    max_binary = build_global_binary_max()
    centroids = detect_centroids(max_binary.astype(np.uint8))
    distances_neighbors = compute_neighbor_distances(centroids)

    dists_px, avg_dist_px, std_dist_px, cv_percent, pixel_size_mm, std_pixel_size_mm = compute_statistics(
        distances_neighbors,
        KNOWN_DISTANCE_MM,
    )

    print_statistics(dists_px, avg_dist_px, std_dist_px, cv_percent, pixel_size_mm, std_pixel_size_mm)
    display_results(
        max_binary,
        centroids,
        distances_neighbors,
        dists_px,
        avg_dist_px,
        std_dist_px,
        cv_percent,
        pixel_size_mm,
    )


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    main()

# end of file
