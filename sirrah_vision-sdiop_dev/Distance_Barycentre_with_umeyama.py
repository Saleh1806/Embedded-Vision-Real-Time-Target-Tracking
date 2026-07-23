#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# Project   : SIRRAH VISION
# ----------------------------------------------------------------------------
# Title     : Barycenter and Distance Estimation with Umeyama Alignment
# Author    : Serigne Saliou Mbacke Diop
# Date      : 07/01/2026
# ----------------------------------------------------------------------------
# Confidential file
# Copyright (C) ARCK Sensor - All rights reserved
# ----------------------------------------------------------------------------
# Description :
#   This script detects up to 4 LED centroids in one image and compares several
#   barycenter estimation strategies:
#     1. Geometric barycenter from detected LEDs,
#     2. Barycenter propagated through Umeyama similarity transform,
#     3. Reduced-visibility fallbacks (3-LED diagonal, 2-LED midpoint).
#
#   It also compares distance estimates based on:
#     - scale from Umeyama,
#     - scale derived from polygon image area.
# ----------------------------------------------------------------------------
# History :
#   30/09/2025  S.Diop : creation
# ============================================================================

# -----------------------------------------------------------------------------
# Import
# -----------------------------------------------------------------------------

import itertools
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

# -----------------------------------------------------------------------------
# Configuration parameters
# -----------------------------------------------------------------------------

BASE_DIR = Path(__file__).resolve().parent
FILENAME = (
    BASE_DIR
    / "Donnees_Images_Test_SIRRAH_VISION"
    / "Bonnes_images_Decalage_angualire_test"
    / "Test_exterieur"
    / "Test_exterieur_10m"
    / "ET_1000"
    / "capture_ET1000_#1_16bits.png"
)

WIDTH = 1456
HEIGHT = 1088
THRESH_LED = 200

# Camera and model parameters.
# L is the real beacon side length in meters.
L = 0.07
FX = 2717.14
FY = 2602.63
F = np.sqrt(FX * FY)

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Function : sort_by_angle
# Description :
#   Sort 2D points counterclockwise around their geometric center.
# Arguments :
#   points -> np.ndarray (N x 2), unordered 2D points.
# Returns :
#   np.ndarray: same points sorted by polar angle.
# -----------------------------------------------------------------------------
def sort_by_angle(points: np.ndarray) -> np.ndarray:
    """Sort 2D points counterclockwise around their centroid."""
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    return points[np.argsort(angles)]


# -----------------------------------------------------------------------------
# Function : umeyama
# Description :
#   Estimate similarity transform between source and destination points:
#   destination ~= scale * (R @ source) + t.
# Arguments :
#   src -> np.ndarray (N x 2), source/model points.
#   dst -> np.ndarray (N x 2), destination/image points.
# Returns :
#   tuple (scale, R, t): scalar scale, 2x2 rotation matrix, translation vector.
# -----------------------------------------------------------------------------
def umeyama(src: np.ndarray, dst: np.ndarray):
    """Estimate similarity transform (scale, rotation, translation).

    The transform maps src to dst as:
      dst ~= scale * (R @ src) + t
    """
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)

    src_d = src - src_mean
    dst_d = dst - dst_mean

    cov = dst_d.T @ src_d / src.shape[0]
    U, S, Vt = np.linalg.svd(cov)

    R = U @ Vt
    if np.linalg.det(R) < 0:
        # Reflection case: flip last singular vector to enforce proper rotation.
        Vt[-1] *= -1
        R = U @ Vt

    scale = S.sum() / np.sum(src_d ** 2)
    t = dst_mean - scale * (R @ src_mean)
    return scale, R, t


# -----------------------------------------------------------------------------
# Function : bary_3_leds_diagonal
# Description :
#   Estimate marker center from 3 visible LEDs using midpoint of the farthest
#   pair (assumed to be opposite corners / diagonal).
# Arguments :
#   pts -> np.ndarray (3 x 2), detected LED points.
# Returns :
#   np.ndarray (2,): estimated center (x, y).
# -----------------------------------------------------------------------------
def bary_3_leds_diagonal(pts: np.ndarray) -> np.ndarray:
    """3-LED fallback: center estimated as midpoint of farthest pair (diagonal)."""
    max_d = -1.0
    pair = None
    for i in range(3):
        for j in range(i + 1, 3):
            d = float(np.linalg.norm(pts[i] - pts[j]))
            if d > max_d:
                max_d = d
                pair = (pts[i], pts[j])
    return 0.5 * (pair[0] + pair[1])


# -----------------------------------------------------------------------------
# Function : bary_2_leds_midpoint
# Description :
#   Estimate marker center from 2 visible LEDs using segment midpoint.
# Arguments :
#   pts -> np.ndarray (2 x 2), detected LED points.
# Returns :
#   np.ndarray (2,): estimated center (x, y).
# -----------------------------------------------------------------------------
def bary_2_leds_midpoint(pts: np.ndarray) -> np.ndarray:
    """2-LED fallback: center estimated as midpoint of the segment."""
    return 0.5 * (pts[0] + pts[1])


# -----------------------------------------------------------------------------
# Function : polygon_area_px
# Description :
#   Compute polygon area in pixel units from ordered points using shoelace
#   formula.
# Arguments :
#   pts -> np.ndarray (N x 2), ordered polygon points.
# Returns :
#   float: polygon area in square pixels.
# -----------------------------------------------------------------------------
def polygon_area_px(pts: np.ndarray) -> float:
    """Polygon area in pixel units for ordered points (shoelace formula)."""
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


# -----------------------------------------------------------------------------
# Function : fmt_pt
# Description :
#   Format one 2D point for readable console logging.
# Arguments :
#   pt -> np.ndarray (2,), point coordinates.
# Returns :
#   str: formatted "(x, y)" text.
# -----------------------------------------------------------------------------
def fmt_pt(pt: np.ndarray) -> str:
    """Format 2D point for readable logs."""
    return f"({pt[0]:.2f}, {pt[1]:.2f})"

# -----------------------------------------------------------------------------
# Main script
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# Function : main
# Description :
#   Execute full analysis workflow:
#     1) load image and detect LEDs,
#     2) compare barycenter estimators (4/3/2 LEDs),
#     3) compare distance estimates (Umeyama vs area),
#     4) display final overlay visualization.
# Arguments :
#   none
# Returns :
#   none
# -----------------------------------------------------------------------------
def main() -> None:
    """Detect LEDs, compare barycenter estimators, and display results."""

    # -------------------------------------------------------------------------
    # 1) Load image
    # -------------------------------------------------------------------------
    raw = cv2.imread(str(FILENAME), cv2.IMREAD_UNCHANGED)
    if raw is None or raw.dtype != np.uint16:
        raise RuntimeError("Invalid image: expected 16-bit input")

    raw = raw[:HEIGHT, :WIDTH]
    max_val = float(raw.max()) if raw.size else 0.0
    if max_val <= 0.0:
        raise RuntimeError("Invalid image: empty or zero data")

    raw_8bit = cv2.convertScaleAbs(raw, alpha=255.0 / max_val)

    # -------------------------------------------------------------------------
    # 2) LED detection
    # -------------------------------------------------------------------------
    _, mask = cv2.threshold(raw_8bit, THRESH_LED, 255, cv2.THRESH_BINARY)
    _, _, stats, centroids = cv2.connectedComponentsWithStats(mask)

    centroids = centroids[1:]
    areas = stats[1:, cv2.CC_STAT_AREA]

    if len(centroids) == 0:
        raise RuntimeError("No LED detected")

    # Keep the 4 largest components, then order points angularly.
    idx = np.argsort(areas)[::-1][:4]
    pts = np.array(centroids[idx], dtype=np.float64)
    pts = sort_by_angle(pts)

    # Geometric barycenter from detected image points.
    bary_img = pts.mean(axis=0)

    print("\n=== Detected LEDs ===")
    for i, p in enumerate(pts):
        print(f"LED {i}: {fmt_pt(p)}")
    print(f"Image barycenter (4 LEDs): {fmt_pt(bary_img)}")

    # -------------------------------------------------------------------------
    # 3) Square model definition
    # -------------------------------------------------------------------------
    ref_square = np.array(
        [
            [-0.035, -0.035],
            [0.035, -0.035],
            [0.035, 0.035],
            [-0.035, 0.035],
        ],
        dtype=np.float64,
    )

    ref_model = sort_by_angle(ref_square)
    bary_model = ref_model.mean(axis=0)  # expected model center near (0, 0)

    # -------------------------------------------------------------------------
    # 4) Barycenter and distance comparisons
    # -------------------------------------------------------------------------
    print("\n===== COMPARISONS =====")

    # --- 4 LEDs: full-visibility case ---
    scale_4, rot_4, trans_4 = umeyama(ref_model, pts)
    bary_um_4 = scale_4 * (rot_4 @ bary_model) + trans_4

    # Cross-check scale and distance from polygon area.
    area_img = polygon_area_px(pts)
    scale_from_area = np.sqrt(area_img / (L * L))
    z_from_area = F / scale_from_area
    z_umeyama = F / scale_4

    print("\n[4 LEDs]")
    print(f"Umeyama barycenter   : {fmt_pt(bary_um_4)}")
    print(f"Geometric barycenter : {fmt_pt(bary_img)}")
    print(f"Delta (Umeyama - geo): {np.linalg.norm(bary_um_4 - bary_img):.4f} px")

    print(f"s from area          : {scale_from_area:.3f} px/unit")
    print(f"Delta s              : {abs(scale_4 - scale_from_area):.3f}")

    print(f"Distance Z (area)    : {z_from_area:.2f} m")
    print(f"Distance Z (Umeyama) : {z_umeyama:.2f} m")

    # --- 3 LEDs: degraded-visibility analysis ---
    print("\n[3 LEDs]")
    for combo in itertools.combinations(range(4), 3):
        pts3 = pts[list(combo)]
        model3 = ref_model[list(combo)]

        scale_3, rot_3, trans_3 = umeyama(model3, pts3)
        bary_um_3 = scale_3 * (rot_3 @ bary_model) + trans_3

        bary_geo_3 = bary_3_leds_diagonal(pts3)

        print(
            f"LEDs {combo} | "
            f"Umeyama {fmt_pt(bary_um_3)} | "
            f"Diagonal {fmt_pt(bary_geo_3)} | "
            f"Delta Umeyama {np.linalg.norm(bary_um_3 - bary_img):.4f} px | "
            f"Delta Diagonal {np.linalg.norm(bary_geo_3 - bary_img):.4f} px"
        )

    # --- 2 LEDs: highly degraded-visibility analysis ---
    print("\n[2 LEDs]")
    for combo in itertools.combinations(range(4), 2):
        pts2 = pts[list(combo)]
        model2 = ref_model[list(combo)]

        scale_2, rot_2, trans_2 = umeyama(model2, pts2)
        bary_um_2 = scale_2 * (rot_2 @ bary_model) + trans_2

        bary_geo_2 = bary_2_leds_midpoint(pts2)

        print(
            f"LEDs {combo} | "
            f"Umeyama {fmt_pt(bary_um_2)} | "
            f"Midpoint {fmt_pt(bary_geo_2)} | "
            f"Delta Umeyama {np.linalg.norm(bary_um_2 - bary_img):.4f} px | "
            f"Delta Midpoint {np.linalg.norm(bary_geo_2 - bary_img):.4f} px"
        )

    # -------------------------------------------------------------------------
    # 5) Visualization
    # -------------------------------------------------------------------------
    overlay = cv2.cvtColor(raw_8bit, cv2.COLOR_GRAY2BGR)

    for x, y in pts:
        cv2.circle(overlay, (int(x), int(y)), 6, (0, 0, 255), -1)

    cv2.circle(overlay, (int(bary_img[0]), int(bary_img[1])), 8, (255, 255, 0), -1)

    plt.figure(figsize=(10, 5))
    plt.imshow(overlay[..., ::-1])
    plt.title("LEDs (red) and image barycenter (cyan)")
    plt.axis("off")
    plt.show()

# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    main()

# end of file
