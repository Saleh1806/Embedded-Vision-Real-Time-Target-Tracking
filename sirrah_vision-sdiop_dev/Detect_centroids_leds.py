#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# Project   : SIRRAH VISION
# ----------------------------------------------------------------------------
# Title     : Multi-LED Centroid Detection and Selection (RAW Image)
# Author    : Serigne Saliou Mbacke Diop
# Date      : 07/01/2026
# ----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# ----------------------------------------------------------------------------
# Description :
#   This script loads one RAW image, detects bright LED blobs, selects the best
#   subset using a square-geometry criterion, and displays/saves debug outputs.
# ----------------------------------------------------------------------------
# History :
#   07/01/2026  S.Diop : creation
# ============================================================================

# -----------------------------------------------------------------------------
# Import
# -----------------------------------------------------------------------------

import itertools
import math
import os
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np

# -----------------------------------------------------------------------------
# Configuration parameters
# -----------------------------------------------------------------------------

WIDTH = 2064
HEIGHT = 1552
FILENAME = (
    r"C:\Users\Arck\Documents\OpencvApp\Test_exterieur_TIS36\Test_2_J05m"
    r"\ET_25\capture_ET25_#3.raw"
)

AUTO_THRESH_LED = True
THRESH_PERCENTILE = 99.9
THRESH_MIN_10 = 400
THRESH_LED_10 = 1020

TARGET_BLOBS = 4
MIN_BLOBS = 1
MAX_CANDIDATES = 10
DIAG_EQUALITY_TOL = 0.20
DIAG_RATIO_TOL = 0.25

# -----------------------------------------------------------------------------
# Function : read_raw_10bit
# Description :
#   Read RAW image, infer stride, crop active image area, and normalize to
#   10-bit domain [0..1023].
# Arguments :
#   path   -> RAW file path.
#   width  -> expected active width.
#   height -> expected active height.
# Returns :
#   np.ndarray (uint16): normalized 10-bit image.
# Raises :
#   FileNotFoundError if file is missing.
#   ValueError if frame content is invalid.
# -----------------------------------------------------------------------------
def read_raw_10bit(path: str, width: int, height: int) -> np.ndarray:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Fichier introuvable : {path}")

    raw = np.fromfile(path, dtype=np.uint16)
    if raw.size < width * height:
        raise ValueError(f"Image invalide: taille insuffisante ({raw.size})")

    stride = raw.size // height
    if stride < width:
        raise ValueError(f"Image invalide: stride ({stride}) < width ({width})")

    raw_strided = raw[: stride * height].reshape((height, stride))
    raw_image = raw_strided[:, :width]

    max_val = float(raw_image.max()) if raw_image.size else 0.0
    if max_val <= 0.0:
        raise ValueError("Image vide ou intensite nulle")

    raw_10bit = np.round(raw_image.astype(np.float32) * (1023.0 / max_val)).astype(np.uint16)
    return raw_10bit


# -----------------------------------------------------------------------------
# Function : compute_led_threshold
# Description :
#   Compute LED threshold in 10-bit domain either from percentile or fixed value.
# Arguments :
#   img_10bit -> normalized 10-bit image.
# Returns :
#   int: threshold in [0..1023].
# -----------------------------------------------------------------------------
def compute_led_threshold(img_10bit: np.ndarray) -> int:
    if AUTO_THRESH_LED:
        percentile_value = float(np.percentile(img_10bit, THRESH_PERCENTILE))
        threshold_led_10 = int(max(THRESH_MIN_10, min(1023, percentile_value)))
        print(f"THRESH_LED_10 auto = {threshold_led_10} (p{THRESH_PERCENTILE})")
    else:
        threshold_led_10 = THRESH_LED_10
        print(f"THRESH_LED_10 fixe = {threshold_led_10}")

    return threshold_led_10


# -----------------------------------------------------------------------------
# Function : detect_blobs
# Description :
#   Build binary LED mask and run connected components.
# Arguments :
#   img_10bit       -> normalized image.
#   threshold_led_10-> threshold value.
# Returns :
#   tuple (mask, stats, centroids) without background label.
# -----------------------------------------------------------------------------
def detect_blobs(img_10bit: np.ndarray, threshold_led_10: int):
    leds_mask = (img_10bit >= threshold_led_10).astype(np.uint8) * 255
    leds_mask_clean = leds_mask.copy()

    _, _, stats, centroids = cv2.connectedComponentsWithStats(leds_mask_clean)
    centroids = centroids[1:]
    stats = stats[1:]

    print(f"{len(centroids)} blobs lumineux detectes apres nettoyage")
    return leds_mask, leds_mask_clean, stats, centroids


# -----------------------------------------------------------------------------
# Function : square_geometry_score
# Description :
#   Score candidate subset based on square consistency (side/diagonal geometry).
# Arguments :
#   pts_subset -> list/array of candidate points.
# Returns :
#   float: lower is better, inf means rejected subset.
# -----------------------------------------------------------------------------
def square_geometry_score(pts_subset: list[np.ndarray]) -> float:
    dists = [
        float(np.linalg.norm(np.array(p1) - np.array(p2)))
        for p1, p2 in itertools.combinations(pts_subset, 2)
    ]
    if len(dists) == 0:
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

    return side_uniformity + diag_uniformity + diag_ratio_error


# -----------------------------------------------------------------------------
# Function : select_best_blobs
# Description :
#   Select up to TARGET_BLOBS candidates that best satisfy square geometry.
# Arguments :
#   centroids -> detected centroids.
#   stats     -> connected-component stats aligned with centroids.
# Returns :
#   tuple (selected_indices, selected_points, selected_areas).
# Raises :
#   RuntimeError if not enough candidates exist.
# -----------------------------------------------------------------------------
def select_best_blobs(centroids: np.ndarray, stats: np.ndarray):
    candidate_idxs = list(range(len(centroids)))
    if len(candidate_idxs) > MAX_CANDIDATES:
        areas = stats[:, cv2.CC_STAT_AREA]
        top_idxs = np.argsort(areas)[-MAX_CANDIDATES:]
        candidate_idxs = list(top_idxs)

    if len(candidate_idxs) < MIN_BLOBS:
        raise RuntimeError("Pas assez de blobs candidats")

    target = min(TARGET_BLOBS, len(candidate_idxs))

    if target == 1:
        selected_idxs = [candidate_idxs[0]]
    else:
        best_idxs = None
        best_score = float("inf")
        for idxs_subset in itertools.combinations(candidate_idxs, target):
            pts_subset = [centroids[i] for i in idxs_subset]
            score = square_geometry_score(pts_subset)
            if score < best_score:
                best_score = score
                best_idxs = idxs_subset

        selected_idxs = list(best_idxs) if best_idxs is not None else list(candidate_idxs[:target])

    selected_pts = [centroids[i] for i in selected_idxs]
    selected_areas = [int(stats[i, cv2.CC_STAT_AREA]) for i in selected_idxs]

    for k, (idx, area) in enumerate(zip(selected_idxs, selected_areas), start=1):
        print(f"Blob selectionne {k}: idx={idx}, aire={area} px")

    return selected_idxs, selected_pts, selected_areas


# -----------------------------------------------------------------------------
# Function : summarize_selection
# Description :
#   Compute and print summary metrics for selected blobs.
# Arguments :
#   selected_pts -> selected centroid points.
# Returns :
#   float: square score for selected set (nan if not applicable).
# -----------------------------------------------------------------------------
def summarize_selection(selected_pts: list[np.ndarray]) -> float:
    dists = [
        float(np.linalg.norm(np.array(p1) - np.array(p2)))
        for p1, p2 in itertools.combinations(selected_pts, 2)
    ]
    dists.sort()

    selected_count = len(selected_pts)
    square_score_selected = float("nan")

    if len(dists) >= 6:
        mean_side = float(np.mean(dists[0:4]))
        side_uniformity = float(np.std(dists[0:4])) / max(1e-9, mean_side)
        diag_uniformity = float(np.std(dists[4:6])) / max(1e-9, mean_side)
        expected_diag = mean_side * math.sqrt(2.0)
        diag_ratio_error = float(np.mean(np.abs(np.array(dists[4:6]) - expected_diag))) / max(1e-9, mean_side)
        square_score_selected = side_uniformity + diag_uniformity + diag_ratio_error
        msg = f"{selected_count} blobs selectionnes. Moyenne cote = {mean_side:.1f}px"
    elif len(dists) >= 3:
        mean_side = float(np.mean(dists))
        msg = f"{selected_count} blobs selectionnes. Moyenne distances = {mean_side:.1f}px"
    else:
        msg = f"{selected_count} blob selectionne." if selected_count == 1 else f"{selected_count} blobs selectionnes."

    print(msg)
    if np.isfinite(square_score_selected):
        print(f"Score critere carre = {square_score_selected:.4f}")

    return square_score_selected


# -----------------------------------------------------------------------------
# Function : build_visualizations
# Description :
#   Build overlays and masks for final display and export.
# Arguments :
#   raw_10bit      -> input image.
#   leds_mask      -> threshold mask.
#   leds_mask_clean-> cleaned mask.
#   centroids      -> all detected centroids.
#   selected_pts   -> selected centroids.
#   selected_areas -> areas of selected blobs.
# Returns :
#   tuple (raw_8bit, overlay, overlay_all, mask_leds_only, leds_mask_clean).
# -----------------------------------------------------------------------------
def build_visualizations(
    raw_10bit: np.ndarray,
    leds_mask: np.ndarray,
    leds_mask_clean: np.ndarray,
    centroids: np.ndarray,
    selected_pts: list[np.ndarray],
    selected_areas: list[int],
):
    mask_leds_only = np.zeros_like(leds_mask_clean)
    for (cx, cy) in selected_pts:
        cv2.circle(mask_leds_only, (int(cx), int(cy)), 4, 255, -1)

    raw_8bit = cv2.convertScaleAbs(raw_10bit, alpha=255.0 / 1023.0)
    overlay = cv2.cvtColor(raw_8bit, cv2.COLOR_GRAY2BGR)
    overlay_all = overlay.copy()

    for (cx, cy) in centroids:
        cv2.circle(overlay_all, (int(cx), int(cy)), 4, (0, 255, 255), -1)

    for (cx, cy), area in zip(selected_pts, selected_areas):
        cv2.circle(overlay, (int(cx), int(cy)), 6, (0, 0, 255), -1)
        cv2.putText(
            overlay,
            f"{area}px",
            (int(cx) + 8, int(cy) - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    for p1, p2 in itertools.combinations(selected_pts, 2):
        cv2.line(overlay, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0, 255, 0), 2)

    cv2.putText(overlay, "4 LEDs selectionnees", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    return raw_8bit, overlay, overlay_all, mask_leds_only, leds_mask_clean


# -----------------------------------------------------------------------------
# Function : display_results
# Description :
#   Display final debug panels.
# Arguments :
#   raw_8bit, leds_mask, overlay_all, overlay, mask_leds_only -> debug images.
# Returns :
#   none
# -----------------------------------------------------------------------------
def display_results(
    raw_8bit: np.ndarray,
    leds_mask: np.ndarray,
    overlay_all: np.ndarray,
    overlay: np.ndarray,
    mask_leds_only: np.ndarray,
) -> None:
    plt.figure(figsize=(14, 8))

    plt.subplot(2, 3, 1)
    plt.imshow(raw_8bit, cmap="gray")
    plt.title("Image brute")
    plt.axis("off")

    plt.subplot(2, 3, 2)
    plt.imshow(leds_mask, cmap="gray")
    plt.title("Masque binaire LEDs")
    plt.axis("off")

    plt.subplot(2, 3, 3)
    plt.imshow(overlay_all[..., ::-1])
    plt.title("Image avec les blobs")
    plt.axis("off")

    plt.subplot(2, 3, 4)
    plt.imshow(overlay[..., ::-1])
    plt.title("Image avec blobs selectionnes")
    plt.axis("off")

    plt.subplot(2, 3, 5)
    plt.imshow(mask_leds_only, cmap="gray")
    plt.title("LEDs isolees")
    plt.axis("off")

    plt.tight_layout()
    plt.show()


# -----------------------------------------------------------------------------
# Function : save_outputs
# Description :
#   Save generated debug images to disk.
# Arguments :
#   overlay, overlay_all, leds_mask_clean, mask_leds_only -> images to export.
# Returns :
#   none
# -----------------------------------------------------------------------------
def save_outputs(
    overlay: np.ndarray,
    overlay_all: np.ndarray,
    leds_mask_clean: np.ndarray,
    mask_leds_only: np.ndarray,
) -> None:
    cv2.imwrite("leds_detectees_overlay.png", overlay)
    cv2.imwrite("leds_all_blobs.png", overlay_all)
    cv2.imwrite("leds_mask_clean.png", leds_mask_clean)
    cv2.imwrite("leds_binarisees_full.png", mask_leds_only)
    print(
        "Images enregistrees : leds_detectees_overlay.png, leds_all_blobs.png, "
        "leds_mask_clean.png, leds_binarisees_full.png"
    )


# -----------------------------------------------------------------------------
# Function : main
# Description :
#   Run complete LED centroid detection workflow on one RAW image.
# Arguments :
#   none
# Returns :
#   none
# -----------------------------------------------------------------------------
def main() -> None:
    t0 = time.perf_counter()

    raw_10bit = read_raw_10bit(FILENAME, WIDTH, HEIGHT)
    threshold_led_10 = compute_led_threshold(raw_10bit)
    leds_mask, leds_mask_clean, stats, centroids = detect_blobs(raw_10bit, threshold_led_10)

    selected_idxs, selected_pts, selected_areas = select_best_blobs(centroids, stats)
    _ = selected_idxs
    summarize_selection(selected_pts)

    raw_8bit, overlay, overlay_all, mask_leds_only, leds_mask_clean = build_visualizations(
        raw_10bit,
        leds_mask,
        leds_mask_clean,
        centroids,
        selected_pts,
        selected_areas,
    )

    display_results(raw_8bit, leds_mask, overlay_all, overlay, mask_leds_only)
    save_outputs(overlay, overlay_all, leds_mask_clean, mask_leds_only)

    t_total_ms = (time.perf_counter() - t0) * 1000.0
    print(f"Temps de calcul total : {t_total_ms:.2f} ms")


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    main()

# end of file
