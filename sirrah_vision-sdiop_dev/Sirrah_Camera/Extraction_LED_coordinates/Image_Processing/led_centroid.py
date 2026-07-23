import argparse
import glob
import itertools
import math
import os
import time

import cv2
import numpy as np
import subprocess

AUTO_THRESH_PERCENTILE = 99.9
AUTO_THRESH_MIN_10 = 400
TARGET_BLOBS = 4
MIN_BLOBS = 1
MAX_CANDIDATES = 10
DIAG_EQUALITY_TOL = 0.20
DIAG_RATIO_TOL = 0.25
REQUIRE_EXACT_FOUR_LEDS = True
SQUARE_SCORE_MAX = 0.25
ALLOW_ISOSCELES_TRIANGLE_3LEDS = True
ISOSCELES_REL_TOL = 0.12


def load_raw_image(path, width, height):
    raw = np.fromfile(path, dtype=np.uint16)
    if raw.size < width * height:
        return None

    stride = raw.size // height
    raw_image = raw[:stride * height].reshape((height, stride))[:, :width]
    return raw_image


def sort_by_angle(points):
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    return points[np.argsort(angles)]


def is_isosceles_triangle(points, rel_tol):
    if len(points) != 3:
        return False
    d = [
        float(np.linalg.norm(np.array(p1, dtype=np.float64) - np.array(p2, dtype=np.float64)))
        for p1, p2 in itertools.combinations(points, 2)
    ]
    d.sort()
    if d[1] <= 1e-9:
        return False
    # Isosceles if at least two sides are close enough.
    return (abs(d[0] - d[1]) / d[1] <= rel_tol) or (abs(d[1] - d[2]) / d[2] <= rel_tol)


def find_led_centroid(path, width, height, threshold):
    raw_image = load_raw_image(path, width, height)
    if raw_image is None:
        return None

    max_val = float(raw_image.max()) if raw_image.size else 0.0
    if max_val <= 0.0:
        return None

    raw_10bit = np.round(raw_image.astype(np.float32) * (1023.0 / max_val)).astype(np.uint16)

    if threshold <= 1023:
        threshold_led_10 = int(max(0, min(1023, threshold)))
    else:
        p = float(np.percentile(raw_10bit, AUTO_THRESH_PERCENTILE))
        threshold_led_10 = int(max(AUTO_THRESH_MIN_10, min(1023, p)))

    leds_mask = (raw_10bit >= threshold_led_10).astype(np.uint8) * 255
    _, _, stats, centroids = cv2.connectedComponentsWithStats(leds_mask)
    centroids = centroids[1:]
    stats = stats[1:]
    if len(centroids) < MIN_BLOBS:
        return None

    candidate_idxs = list(range(len(centroids)))
    if len(candidate_idxs) > MAX_CANDIDATES:
        areas = stats[:, cv2.CC_STAT_AREA]
        top_idxs = np.argsort(areas)[-MAX_CANDIDATES:]
        candidate_idxs = list(top_idxs)

    if REQUIRE_EXACT_FOUR_LEDS and len(candidate_idxs) < 3:
        return None

    if REQUIRE_EXACT_FOUR_LEDS:
        target = TARGET_BLOBS if len(candidate_idxs) >= TARGET_BLOBS else 3
    else:
        target = min(TARGET_BLOBS, len(candidate_idxs))

    def square_geometry_score(idxs_subset):
        pts_subset = [centroids[i] for i in idxs_subset]
        dists = [
            np.linalg.norm(np.array(p1, dtype=np.float64) - np.array(p2, dtype=np.float64))
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

        diag_diff_rel = float(abs(diags[0] - diags[1])) / side_mean
        diag_ratio_rel = float(abs((float(np.mean(diags)) / side_mean) - math.sqrt(2.0)))
        if diag_diff_rel > DIAG_EQUALITY_TOL or diag_ratio_rel > DIAG_RATIO_TOL:
            return float("inf")

        side_uniformity = float(np.std(sides)) / side_mean
        diag_uniformity = float(np.std(diags)) / side_mean
        expected_diag = side_mean * math.sqrt(2.0)
        diag_ratio_error = float(np.mean(np.abs(diags - expected_diag))) / side_mean
        return side_uniformity + diag_uniformity + diag_ratio_error

    if target == 1:
        selected_idxs = [candidate_idxs[0]]
        best_score = float("inf")
    else:
        best_idxs = None
        best_score = float("inf")
        for idxs_subset in itertools.combinations(candidate_idxs, target):
            score = square_geometry_score(idxs_subset)
            if score < best_score:
                best_score = score
                best_idxs = idxs_subset
        if best_idxs is None:
            return None
        selected_idxs = list(best_idxs)

    if REQUIRE_EXACT_FOUR_LEDS:
        if target == TARGET_BLOBS:
            if not np.isfinite(best_score) or best_score > SQUARE_SCORE_MAX:
                return None
        elif target == 3:
            if not ALLOW_ISOSCELES_TRIANGLE_3LEDS:
                return None
            selected_pts_tmp = np.array([centroids[i] for i in selected_idxs], dtype=np.float64)
            if not is_isosceles_triangle(selected_pts_tmp, ISOSCELES_REL_TOL):
                return None
        else:
            return None

    selected_pts = np.array([centroids[i] for i in selected_idxs], dtype=np.float64)
    selected_pts = sort_by_angle(selected_pts)
    cx = float(np.mean(selected_pts[:, 0]))
    cy = float(np.mean(selected_pts[:, 1]))

    left = int(np.min(stats[selected_idxs, cv2.CC_STAT_LEFT]))
    top = int(np.min(stats[selected_idxs, cv2.CC_STAT_TOP]))
    rights = stats[selected_idxs, cv2.CC_STAT_LEFT] + stats[selected_idxs, cv2.CC_STAT_WIDTH]
    bottoms = stats[selected_idxs, cv2.CC_STAT_TOP] + stats[selected_idxs, cv2.CC_STAT_HEIGHT]
    right = int(np.max(rights))
    bottom = int(np.max(bottoms))
    bbox = (left, top, max(1, right - left), max(1, bottom - top))

    raw_16bit = (raw_10bit.astype(np.uint32) * 65535 // 1023).astype(np.uint16)
    return cx, cy, bbox, raw_16bit, selected_pts


def collect_files(input_path, dir_path, pattern):
    if input_path:
        return [input_path]

    search_pattern = os.path.join(dir_path, pattern)
    files = sorted(glob.glob(search_pattern))
    return files


def main():
    parser = argparse.ArgumentParser(
        description="Extract LED centroid from RAW images generated by RawCapture.cpp"
    )
    parser.add_argument("--input", help="single raw file path")
    parser.add_argument("--dir", default="/home/raspberrypi/captures", help="folder containing raw files")
    parser.add_argument("--pattern", default="frame_Pipeline*.raw", help="glob pattern to match files")
    parser.add_argument("--width", type=int, default=2064)
    parser.add_argument("--height", type=int, default=1552)
    parser.add_argument("--threshold", type=int, default=60000, help="16-bit threshold (0-65535)")
    parser.add_argument("--limit", type=int, default=0, help="limit number of files (0 = all)")
    parser.add_argument("--coords-stdout", action="store_true", help="emit ts,u1,v1,...,uN,vN to stdout")
    parser.add_argument("--quiet", action="store_true", help="suppress non-essential logs")
    parser.add_argument("--watch", action="store_true", help="watch directory for new files")
    parser.add_argument("--poll-ms", type=int, default=200, help="poll interval in ms when watching")
    parser.add_argument("--save-debug", action="store_true", help="save annotated image with LED bbox")
    parser.add_argument("--debug-dir", default="/home/raspberrypi/captures/led_debug",
                        help="directory for annotated images")
    parser.add_argument(
        "--angle-exe",
        help="path to C++ angle calculator executable (sirrah_demo)"
    )
    args = parser.parse_args()

    coords_file = None

    processed = set()
    processed_count = 0
    detected_count = 0
    while True:
        files = collect_files(args.input, args.dir, args.pattern)
        if not files:
            if args.watch:
                time.sleep(args.poll_ms / 1000.0)
                continue
            if not args.quiet:
                print("No files found.")
            return 1

        for path in files:
            if path in processed:
                continue
            processed.add(path)

            result = find_led_centroid(path, args.width, args.height, args.threshold)
            name = os.path.basename(path)
            if result is None:
                if not args.quiet:
                    print(f"{name}: no LED detected")
                continue
            cx, cy, bbox, raw_16bit, selected_pts = result
            detected_count += 1
            if not args.quiet:
                print(f"{name}: cx={cx:.3f}, cy={cy:.3f}")
            if args.save_debug:
                os.makedirs(args.debug_dir, exist_ok=True)
                x, y, w, h = bbox
                raw_8bit = cv2.convertScaleAbs(raw_16bit, alpha=255.0 / 65535.0)
                annotated = cv2.cvtColor(raw_8bit, cv2.COLOR_GRAY2BGR)
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(annotated, (int(round(cx)), int(round(cy))), 3, (0, 0, 255), -1)
                base = os.path.splitext(name)[0]
                out_path = os.path.join(args.debug_dir, f"{base}_led.png")
                cv2.imwrite(out_path, annotated)
            if args.coords_stdout:
                ts_ms = int(time.time() * 1000)
                payload = [str(ts_ms)]
                for pt in selected_pts:
                    payload.append(f"{float(pt[0]):.6f}")
                    payload.append(f"{float(pt[1]):.6f}")
                print(",".join(payload), flush=True)
            if args.angle_exe:
                proc = subprocess.run(
                    [args.angle_exe, "--cx", f"{cx:.6f}", "--cy", f"{cy:.6f}"],
                    capture_output=True,
                    text=True,
                )
                if proc.returncode != 0:
                    err = proc.stderr.strip()
                    output = proc.stdout.strip()
                    if output:
                        if not args.quiet:
                            print(f"{name}: {output}")
                    if err:
                        if not args.quiet:
                            print(f"{name}: angle calc failed: {err}")
                    else:
                        if not args.quiet:
                            print(f"{name}: angle calc failed (exit {proc.returncode})")
                else:
                    output = proc.stdout.strip()
                    if output and not args.quiet:
                        print(f"{name}: {output}")

            processed_count += 1
            if args.limit > 0 and processed_count >= args.limit:
                return 0

        if not args.watch:
            if processed_count > 0 and detected_count == 0:
                print("No LED detected in processed files.")
            break
        time.sleep(args.poll_ms / 1000.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
