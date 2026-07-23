#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# Project   : SIRRAH VISION
# ----------------------------------------------------------------------------
# Title     : Centroid Motion and Positional Deviation Analysis (RAW SRGGB10)
# Author    : Serigne Saliou Mbacke Diop
# Date      : 30/09/2025
# ----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# ----------------------------------------------------------------------------
# Description :
#   This script analyzes a sequence of RAW 10-bit images to estimate beacon
#   centroid displacement with respect to one reference image. It performs:
#     1. RAW loading with stride handling,
#     2. LED/blob detection from thresholded image,
#     3. Marker centroid computation from selected blobs,
#     4. Per-frame shift computation (Dx, Dy) versus reference,
#     5. Statistical summary and plots of variations.
#
#   Detection follows the same geometric blob-selection logic used in the
#   project pipeline (square-like constraint on candidate blobs).
# ----------------------------------------------------------------------------
# History :
#   30/09/2025  S.Diop : creation
# ============================================================================

# -----------------------------------------------------------------------------
# Import
# -----------------------------------------------------------------------------

import itertools
import math
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np

# -----------------------------------------------------------------------------
# Configuration parameters
# -----------------------------------------------------------------------------

WIDTH = 2064
HEIGHT = 1552

REF_PATH = r"Test2_Tis36_Decalage_angulaire/led_ref/frame_ref_1.raw"
FOLDER = r"Test2_Tis36_Decalage_angulaire/led_decalee_2B/"

FILE_PATTERN = "frame_ref_"
START_IDX = 1
NB_IMAGES = 10

# LED thresholding and blob selection parameters.
# These constants replicate the project detection logic so this analysis script
# is comparable to the online pipeline behavior.
AUTO_THRESH_LED = True
THRESH_PERCENTILE = 99.9
THRESH_MIN_10 = 400
THRESH_LED_10 = 1020  # used only if AUTO_THRESH_LED is False
TARGET_BLOBS = 4
MIN_BLOBS = 1
MAX_CANDIDATES = 10
DIAG_EQUALITY_TOL = 0.20
DIAG_RATIO_TOL = 0.25

# -----------------------------------------------------------------------------
# RAW reading and blob detection
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Function : read_raw_10bit
# Description :
#   Read a RAW image file, handle potential line stride, crop active region,
#   and normalize data into 10-bit domain [0..1023].
# Arguments :
#   path   -> input RAW file path.
#   width  -> expected active width in pixels.
#   height -> expected active height in pixels.
# Returns :
#   np.ndarray (uint16): normalized 10-bit image.
# Raises :
#   ValueError if file is too small or image is empty/zero.
# -----------------------------------------------------------------------------
def read_raw_10bit(path: str, width: int, height: int) -> np.ndarray:
    """Read RAW file and return normalized 10-bit image with stride handling.

    Steps:
      1) Load file as uint16 (sensor container format),
      2) Infer stride from total samples / image height,
      3) Remove line padding to keep only active pixels,
      4) Normalize to [0..1023] to work in 10-bit domain.
    """
    raw = np.fromfile(path, dtype=np.uint16)
    if raw.size < width * height:
        raise ValueError(f"Ignored file (insufficient size): {path}")

    stride = raw.size // height
    raw_strided = raw[: stride * height].reshape((height, stride))
    raw_image = raw_strided[:, :width]

    max_val = float(raw_image.max()) if raw_image.size else 0.0
    if max_val <= 0.0:
        raise ValueError(f"Ignored file (empty/zero image): {path}")

    raw_10bit = np.round(raw_image.astype(np.float32) * (1023.0 / max_val)).astype(np.uint16)
    return raw_10bit


# -----------------------------------------------------------------------------
# Function : detect_blobs_like_pipeline
# Description :
#   Detect bright blobs from a 10-bit image and select the subset that best
#   matches the expected square-like LED geometry.
# Arguments :
#   raw_10bit -> np.ndarray, normalized 10-bit image.
# Returns :
#   dict with threshold/masks/centroids/selected subset, or None if no valid
#   candidate exists.
# -----------------------------------------------------------------------------
def detect_blobs_like_pipeline(raw_10bit: np.ndarray) -> dict | None:
    """Detect and select LED blobs using the same geometry constraints as pipeline.

    The detector:
      - Thresholds bright pixels,
      - Extracts connected components,
      - Keeps the best subset using square-like geometry scoring.
    """
    # Auto threshold follows a high percentile to keep only the brightest zones.
    # This is robust to global illumination changes across frames.
    if AUTO_THRESH_LED:
        percentile_value = float(np.percentile(raw_10bit, THRESH_PERCENTILE))
        threshold_led_10 = int(max(THRESH_MIN_10, min(1023, percentile_value)))
    else:
        threshold_led_10 = THRESH_LED_10

    # Binary mask of potential LEDs.
    leds_mask = (raw_10bit >= threshold_led_10).astype(np.uint8) * 255

    # Connected components: index 0 is background and is discarded.
    _, _, stats, centroids = cv2.connectedComponentsWithStats(leds_mask)
    centroids = centroids[1:]
    stats = stats[1:]

    # Keep only the strongest candidates when too many blobs are present.
    # This reduces combinatorial cost and filters tiny noise components.
    candidate_idxs = list(range(len(centroids)))
    if len(candidate_idxs) > MAX_CANDIDATES:
        areas = stats[:, cv2.CC_STAT_AREA]
        top_idxs = np.argsort(areas)[-MAX_CANDIDATES:]
        candidate_idxs = list(top_idxs)

    if len(candidate_idxs) < MIN_BLOBS:
        return None

    # We try to recover up to 4 LEDs (or fewer if fewer candidates exist).
    target = min(TARGET_BLOBS, len(candidate_idxs))

    # -------------------------------------------------------------------------
    # Function : square_geometry_score (local helper)
    # Description :
    #   Score one candidate subset according to square geometry consistency:
    #   side uniformity, diagonal uniformity, and diagonal-to-side ratio.
    # Arguments :
    #   idxs_subset -> indices of candidate centroids.
    # Returns :
    #   float score (lower is better), inf if subset is rejected.
    # -------------------------------------------------------------------------
    def square_geometry_score(idxs_subset):
        # For a perfect square with 4 points:
        # - 4 shortest pairwise distances are the sides,
        # - 2 largest distances are the diagonals,
        # - diagonals are equal and diagonal/side ~= sqrt(2).
        pts_subset = [centroids[i] for i in idxs_subset]
        dists = [
            np.linalg.norm(np.array(p1) - np.array(p2))
            for p1, p2 in itertools.combinations(pts_subset, 2)
        ]
        if not dists:
            return float("inf")

        if len(dists) < 6:
            return float(np.std(np.array(dists, dtype=np.float64)))

        d_sorted = np.sort(np.array(dists, dtype=np.float64))
        sides = d_sorted[:4]
        diags = d_sorted[4:]

        side_mean = float(np.mean(sides))
        if side_mean <= 1e-9:
            return float("inf")

        side_uniformity = float(np.std(sides)) / side_mean
        diag_uniformity = float(np.std(diags)) / side_mean
        expected_diag = side_mean * math.sqrt(2.0)
        diag_ratio_error = float(np.mean(np.abs(diags - expected_diag))) / side_mean
        diag_diff_rel = float(abs(diags[0] - diags[1])) / side_mean
        diag_ratio_rel = float(abs((float(np.mean(diags)) / side_mean) - math.sqrt(2.0)))

        if diag_diff_rel > DIAG_EQUALITY_TOL or diag_ratio_rel > DIAG_RATIO_TOL:
            return float("inf")

        # Lower score means geometry is closer to the expected square pattern.
        return side_uniformity + diag_uniformity + diag_ratio_error

    if target == 1:
        selected_idxs = [candidate_idxs[0]]
    else:
        # Evaluate all candidate subsets and keep the one with best geometry.
        best_idxs = None
        best_score = float("inf")
        for idxs_subset in itertools.combinations(candidate_idxs, target):
            score = square_geometry_score(idxs_subset)
            if score < best_score:
                best_score = score
                best_idxs = idxs_subset
        selected_idxs = list(best_idxs) if best_idxs is not None else list(candidate_idxs[:target])

    selected_pts = np.array([centroids[i] for i in selected_idxs], dtype=np.float64)
    return {
        "threshold_led_10": threshold_led_10,
        "leds_mask": leds_mask,
        "centroids": centroids,
        "stats": stats,
        "selected_idxs": selected_idxs,
        "selected_pts": selected_pts,
    }


# -----------------------------------------------------------------------------
# Function : find_marker_centroid
# Description :
#   Compute marker centroid for one image from the selected LED blobs and
#   optionally display debugging overlays.
# Arguments :
#   path      -> input RAW file path.
#   visualize -> if True, display diagnostic plots.
# Returns :
#   tuple (cx, cy) if detection succeeds, else None.
# -----------------------------------------------------------------------------
def find_marker_centroid(path: str, visualize: bool = False) -> tuple[float, float] | None:
    """Compute marker centroid from selected blobs in one RAW image.

    Returns:
      (cx, cy) if detection succeeds, else None.
    """
    try:
        raw_10bit = read_raw_10bit(path, WIDTH, HEIGHT)
    except ValueError as exc:
        print(str(exc))
        return None

    detection = detect_blobs_like_pipeline(raw_10bit)
    if detection is None:
        return None

    selected_pts = detection["selected_pts"]
    # Final marker position is the mean of selected LED centroids.
    cx = float(np.mean(selected_pts[:, 0]))
    cy = float(np.mean(selected_pts[:, 1]))

    if visualize:
        # Build debug overlays:
        # - all detected blobs,
        # - selected blobs and final centroid.
        raw_8bit = cv2.convertScaleAbs(raw_10bit, alpha=255.0 / 1023.0)
        overlay = cv2.cvtColor(raw_8bit, cv2.COLOR_GRAY2BGR)
        overlay_all = overlay.copy()

        for (px, py) in detection["centroids"]:
            cv2.circle(overlay_all, (int(px), int(py)), 4, (0, 255, 255), -1)

        selected_areas = [int(detection["stats"][i, cv2.CC_STAT_AREA]) for i in detection["selected_idxs"]]
        for (px, py), area in zip(selected_pts, selected_areas):
            cv2.circle(overlay, (int(px), int(py)), 6, (0, 0, 255), -1)
            cv2.putText(
                overlay,
                f"{area}px",
                (int(px) + 8, int(py) - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        for p1, p2 in itertools.combinations(selected_pts, 2):
            cv2.line(overlay, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0, 255, 0), 2)

        cv2.circle(overlay, (int(round(cx)), int(round(cy))), 6, (255, 0, 0), 2)
        cv2.putText(
            overlay,
            "Centroid",
            (int(round(cx)) + 10, int(round(cy)) + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        plt.figure(figsize=(12, 4))
        plt.subplot(1, 3, 1)
        plt.imshow(raw_8bit, cmap="gray")
        plt.title("Raw image")
        plt.axis("off")

        plt.subplot(1, 3, 2)
        plt.imshow(overlay_all[..., ::-1])
        plt.title("All blobs")
        plt.axis("off")

        plt.subplot(1, 3, 3)
        plt.imshow(overlay[..., ::-1])
        plt.title("Selected blobs + centroid")
        plt.axis("off")

        plt.tight_layout()
        plt.show()

    return cx, cy

# -----------------------------------------------------------------------------
# Sequence analysis
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Function : main
# Description :
#   Run complete displacement analysis relative to one reference frame:
#     1) detect reference centroid,
#     2) process sequence and compute Dx/Dy per frame,
#     3) print statistics and display variation plots.
# Arguments :
#   none
# Returns :
#   none
# -----------------------------------------------------------------------------
def main():
    """Run centroid displacement analysis against a reference RAW frame.

    Workflow:
      1) Detect reference centroid,
      2) Detect centroid in each test frame,
      3) Compute Dx, Dy per frame,
      4) Aggregate statistics,
      5) Plot temporal variations.
    """

    if not os.path.exists(REF_PATH):
        raise FileNotFoundError(f"Reference image not found: {REF_PATH}")

    ref_centroid = find_marker_centroid(REF_PATH, visualize=True)
    if ref_centroid is None:
        raise RuntimeError("Unable to detect valid blobs in reference image.")

    ref_cx, ref_cy = ref_centroid
    print("\n=== REFERENCE IMAGE ===")
    print(f"{os.path.basename(REF_PATH)}: ({ref_cx:.2f}, {ref_cy:.2f})")

    deltas = []
    print("\n=== MEASUREMENTS ===")

    for idx in range(NB_IMAGES):
        filename = f"{FILE_PATTERN}{START_IDX + idx}.raw"
        filepath = os.path.join(FOLDER, filename)
        if not os.path.exists(filepath):
            print(f"Missing file: {filepath}")
            continue

        centroid = find_marker_centroid(filepath, visualize=False)
        if centroid is None:
            print(f"No valid blob detected for {filename}")
            continue

        cx, cy = centroid
        dx = cx - ref_cx
        dy = cy - ref_cy
        deltas.append((dx, dy))
        print(f"{filename}: Dx = {dx:.3f} px, Dy = {dy:.3f} px")

    # Safety stop if all frames failed detection.
    if not deltas:
        raise RuntimeError("No valid displacement computed.")

    # Statistical summary in pixel domain.
    deltas_np = np.array(deltas, dtype=np.float64)
    dx_mean, dy_mean = np.mean(deltas_np, axis=0)
    dx_std, dy_std = np.std(deltas_np, axis=0)
    global_shift = float(np.sqrt(np.mean(np.sum(deltas_np ** 2, axis=1))))

    print("\n=== STATISTICS ===")
    print(f"Analyzed images: {len(deltas)}")
    print(f"Mean Dx = {dx_mean:.4f} px, Mean Dy = {dy_mean:.4f} px")
    print(f"Std Dx  = {dx_std:.4f} px, Std Dy  = {dy_std:.4f} px")
    print(f"Global mean shift = {global_shift:.4f} px")

    # Plot Dx/Dy drift across sequence index.
    plt.figure(figsize=(9, 5))
    plt.plot(deltas_np[:, 0], "o-", label="Dx (px)")
    plt.plot(deltas_np[:, 1], "s-", label="Dy (px)")
    plt.axhline(dx_mean, color="r", linestyle="--", label="Mean Dx")
    plt.axhline(dy_mean, color="g", linestyle="--", label="Mean Dy")
    plt.title("Centroid variation (selected blobs)")
    plt.xlabel("Image index")
    plt.ylabel("Displacement (px)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    main()

# end of file
