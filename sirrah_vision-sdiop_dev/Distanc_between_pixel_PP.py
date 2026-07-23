#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# Project   : SIRRAH VISION
# ----------------------------------------------------------------------------
# Title     : Pixel Size Estimation from Neighbor LED Distances
# Author    : Serigne Saliou Mbacke Diop
# Date      : 30/09/2025
# ----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# ----------------------------------------------------------------------------
# Description :
#   This script processes a sequence of RAW images to estimate pixel size
#   (mm/pixel) from distances between neighboring LED centroids.
#
#   Global workflow:
#     1. Build a cumulative binary image from all frames,
#     2. Detect LED blobs and compute their centroids,
#     3. Keep nearest-neighbor links with distance consistency filtering,
#     4. Convert distances (px) to pixel size (mm/pixel),
#     5. Compute statistics and display analysis plots.
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
START_INDEX = 0

# Neighbor distance consistency threshold (in pixels).
TOLERANCE_PX = 5

# Real spacing between neighboring LEDs in millimeters.
KNOWN_DISTANCE_MM = 60.0

# Minimal connected component area to remove tiny noise blobs.
MIN_BLOB_AREA_PX = 2

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Function : read_raw_frame
# Description :
#   Read one RAW frame as uint16, infer stride from file size, and crop active
#   image region.
# Arguments :
#   path   -> input RAW file path.
#   width  -> expected active width in pixels.
#   height -> expected active height in pixels.
# Returns :
#   np.ndarray (uint16) if valid, otherwise None.
# -----------------------------------------------------------------------------
def read_raw_frame(path: str, width: int, height: int) -> np.ndarray | None:
    """Read one RAW frame with stride handling.

    Returns:
      uint16 image (height x width) if valid, otherwise None.
    """
    raw = np.fromfile(path, dtype=np.uint16)
    if raw.size == 0:
        print(f"Ignored (empty file): {path}")
        return None

    stride = raw.size // height
    if stride < width:
        print(f"Ignored: {path} (stride {stride} < width {width})")
        return None

    frame16 = raw[: stride * height].reshape(height, stride)[:, :width]
    return frame16


# -----------------------------------------------------------------------------
# Function : build_global_binary_max
# Description :
#   Build cumulative binary mask over all frames using local threshold
#   (max_val - 1) per frame.
# Arguments :
#   none (uses global constants).
# Returns :
#   np.ndarray (uint8): cumulative binary image.
# -----------------------------------------------------------------------------
def build_global_binary_max() -> np.ndarray:
    """Build cumulative binary maximum image from all configured frames."""
    max_binary = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)

    for idx in range(NB_IMAGES):
        filename = f"{BASE_NAME}{START_INDEX + idx}.raw"
        print(f"Reading and thresholding {filename} ...")

        frame16 = read_raw_frame(filename, WIDTH, HEIGHT)
        if frame16 is None:
            continue

        # Local threshold near per-frame maximum to isolate brightest regions.
        _, max_val, _, _ = cv2.minMaxLoc(frame16)
        threshold_value = max(0, int(max_val) - 1)

        _, binary = cv2.threshold(frame16, threshold_value, 255, cv2.THRESH_BINARY)
        binary_u8 = binary.astype(np.uint8)

        # Pixel-wise union over all frames.
        max_binary = np.maximum(max_binary, binary_u8)

    print(f"\nGlobal binary maximum image computed from {NB_IMAGES} frames.")
    print("Local threshold used: (max_val - 1) per frame.")
    return max_binary


# -----------------------------------------------------------------------------
# Function : detect_led_centroids
# Description :
#   Detect connected components in the cumulative binary image and return valid
#   LED centroids after area filtering.
# Arguments :
#   binary_image -> np.ndarray (uint8), cumulative binary mask.
# Returns :
#   np.ndarray: centroid array (N x 2).
# Raises :
#   RuntimeError if no valid LED is detected.
# -----------------------------------------------------------------------------
def detect_led_centroids(binary_image: np.ndarray) -> np.ndarray:
    """Return centroid array after area filtering of connected components."""
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_image)
    if num_labels <= 1:
        raise RuntimeError("No LED detected.")

    # Remove background label (index 0).
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
# Function : compute_neighbor_links
# Description :
#   Build neighbor links for each LED using nearest-4 candidates and consistency
#   filtering around median distance when needed.
# Arguments :
#   centroids -> np.ndarray (N x 2), detected LED centroids.
# Returns :
#   list[tuple[int, int, float]] with 1-based LED indices and distance in px.
# -----------------------------------------------------------------------------
def compute_neighbor_links(centroids: np.ndarray) -> list[tuple[int, int, float]]:
    """Build neighbor links using nearest-4 strategy with consistency filter.

    For each LED:
      - Find 4 nearest neighbors,
      - Keep all 4 if their spread is consistent,
      - Otherwise keep only distances close to the median.
    """
    distances_neighbors: list[tuple[int, int, float]] = []

    for i in range(len(centroids)):
        cx_i, cy_i = centroids[i]
        dists: list[tuple[int, float]] = []

        for j in range(len(centroids)):
            if i == j:
                continue
            cx_j, cy_j = centroids[j]
            dist = float(np.hypot(cx_i - cx_j, cy_i - cy_j))
            dists.append((j, dist))

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

        for (j, dist) in valid_neighbors:
            # Store 1-based indices for reporting consistency with previous script style.
            distances_neighbors.append((i + 1, j + 1, dist))

    return distances_neighbors


# -----------------------------------------------------------------------------
# Function : compute_pixel_sizes
# Description :
#   Convert neighbor distances from pixels to mm/pixel using known physical
#   LED spacing.
# Arguments :
#   distances_neighbors -> validated neighbor links and distances in px.
# Returns :
#   np.ndarray of pixel sizes (mm/pixel).
# Raises :
#   RuntimeError if no valid distance is available.
# -----------------------------------------------------------------------------
def compute_pixel_sizes(distances_neighbors: list[tuple[int, int, float]]) -> np.ndarray:
    """Convert neighbor distances from pixels to mm/pixel."""
    pixel_sizes = []

    print("\n=== Individual Pixel Sizes per Neighbor Pair ===")
    for (i, j, dist) in distances_neighbors:
        if dist <= 0:
            continue
        pixel_size = KNOWN_DISTANCE_MM / dist
        pixel_sizes.append(pixel_size)
        print(f"LED {i} -> LED {j}: {dist:.2f} px -> {pixel_size:.6f} mm/pixel")

    if not pixel_sizes:
        raise RuntimeError("No valid distances found to compute pixel size.")

    return np.array(pixel_sizes, dtype=np.float64)


# -----------------------------------------------------------------------------
# Function : display_results
# Description :
#   Print summary statistics and display two panels:
#     - LED map with neighbor links,
#     - histogram of pixel-size values.
# Arguments :
#   max_binary          -> cumulative binary mask.
#   centroids           -> detected LED centroids.
#   distances_neighbors -> validated links with distances.
#   pixel_sizes         -> array of mm/pixel estimates.
# Returns :
#   none
# -----------------------------------------------------------------------------
def display_results(
    max_binary: np.ndarray,
    centroids: np.ndarray,
    distances_neighbors: list[tuple[int, int, float]],
    pixel_sizes: np.ndarray,
) -> None:
    """Display geometric links and pixel size histogram with statistics."""
    avg_pixel_size = float(np.mean(pixel_sizes))
    std_pixel_size = float(np.std(pixel_sizes))
    cv_pixel_size = (std_pixel_size / avg_pixel_size) * 100.0

    print("\n=== STATISTICAL ANALYSIS OF PIXEL SIZE ===")
    print(f"Number of pairs: {len(pixel_sizes)}")
    print(f"Average pixel size = {avg_pixel_size:.6f} mm/pixel")
    print(f"Standard deviation = {std_pixel_size:.6f} mm/pixel")
    print(f"Coefficient of variation (CV) = {cv_pixel_size:.3f} %")

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    plt.suptitle("Statistical analysis of pixel size", fontsize=14, weight="bold")

    # Left panel: detected LEDs and neighbor links.
    ax1 = axes[0]
    ax1.imshow(max_binary, cmap="gray")
    ax1.set_title("LEDs and nearest-neighbor links")
    ax1.set_xlabel("X (pixels)")
    ax1.set_ylabel("Y (pixels)")

    for i, (cx, cy) in enumerate(centroids, start=1):
        ax1.plot(cx, cy, "ro")
        ax1.text(cx + 10, cy, f"{i}", color="red", fontsize=9)

    for (i, j, _) in distances_neighbors:
        x1, y1 = centroids[i - 1]
        x2, y2 = centroids[j - 1]
        ax1.plot([x1, x2], [y1, y2], "y--", linewidth=0.8)

    ax1.grid(color="red", linestyle="--", linewidth=0.4)

    # Right panel: histogram of mm/pixel values.
    ax2 = axes[1]
    ax2.hist(pixel_sizes, bins=20, color="skyblue", edgecolor="black", alpha=0.8)
    ax2.axvline(
        avg_pixel_size,
        color="red",
        linestyle="--",
        linewidth=1.5,
        label=f"Mean = {avg_pixel_size:.6f}",
    )
    ax2.axvline(
        avg_pixel_size + std_pixel_size,
        color="green",
        linestyle=":",
        linewidth=1.2,
        label=f"+1sigma = {(avg_pixel_size + std_pixel_size):.6f}",
    )
    ax2.axvline(
        avg_pixel_size - std_pixel_size,
        color="green",
        linestyle=":",
        linewidth=1.2,
        label=f"-1sigma = {(avg_pixel_size - std_pixel_size):.6f}",
    )

    ax2.set_xlabel("Pixel size (mm/pixel)")
    ax2.set_ylabel("Frequency")
    ax2.set_title("Pixel size distribution (nearest neighbors)")
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
# Entry point
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Function : main
# Description :
#   Run complete pipeline:
#     1) build cumulative binary mask,
#     2) detect LED centroids,
#     3) compute neighbor links,
#     4) derive mm/pixel values and display results.
# Arguments :
#   none
# Returns :
#   none
# -----------------------------------------------------------------------------
def main() -> None:
    """Run full pixel-size estimation workflow."""
    max_binary = build_global_binary_max()
    centroids = detect_led_centroids(max_binary.astype(np.uint8))
    distances_neighbors = compute_neighbor_links(centroids)
    pixel_sizes = compute_pixel_sizes(distances_neighbors)
    display_results(max_binary, centroids, distances_neighbors, pixel_sizes)


if __name__ == "__main__":
    main()

# end of file
