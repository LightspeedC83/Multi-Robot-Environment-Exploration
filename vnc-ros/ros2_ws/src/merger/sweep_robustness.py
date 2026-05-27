#!/usr/bin/env python3
"""
sweep_robustness.py
-------------------
Sweep over a range of ground-truth rotations to characterize how the map
coordinator's accuracy and confidence degrade with increasing rotation
mismatch. Output is a small CSV plus a summary printout suitable for the
intermediate-update slides.

Credit: This file was created with AI. I can explain what's happening in here. 
"""

import csv
import json
import math
import os
import numpy as np

import make_test_maps as mt
import coordinator as mc

RESULTS_DIR = "results"
os.makedirs(RESULTS_DIR, exist_ok=True)

ROTATIONS_DEG = [0, 3, 7, 10, 15, 20, 30, 45, 60, 90]
TRANSLATION_CELLS = (25.0, -15.0)
RESOLUTION = 0.05


def build_maps(theta_deg, tx_cells, ty_cells):
    """Reproduce make_test_maps's pipeline without the file I/O so we can
    iterate quickly over many configurations.
    """
    full = mt.load_maze("maze.png")
    h, w = full.shape

    def lawnmower(x_min, x_max, y_min, y_max, step=3):
        pts = []
        y, direction = y_min, 1
        while y <= y_max:
            xs = range(x_min, x_max + 1, step) if direction > 0 \
                else range(x_max, x_min - 1, -step)
            for x in xs:
                pts.append((x, y))
            y += step
            direction *= -1
        return pts

    traj_a = lawnmower(int(w * 0.05), int(w * 0.85),
                       int(h * 0.05), int(h * 0.85))
    traj_b = lawnmower(int(w * 0.15), int(w * 0.95),
                       int(h * 0.15), int(h * 0.95))
    explore_radius = max(15, min(h, w) // 14)

    grid_a = mt.carve_explored_region(full, traj_a, explore_radius)
    grid_b_native = mt.carve_explored_region(full, traj_b, explore_radius)

    out_h, out_w = h + 80, w + 80
    grid_b_padded = np.full((out_h, out_w), -1, dtype=np.int16)
    grid_b_padded[40:40 + h, 40:40 + w] = grid_b_native
    grid_b = mt.apply_rigid_transform(
        grid_b_padded, math.radians(theta_deg), tx_cells, ty_cells,
        out_shape=(out_h, out_w),
    )
    return grid_a, grid_b


def main():
    rows = []
    print(f"{'theta_gt':>10} {'theta_rec':>10} {'err_deg':>8} "
          f"{'inliers':>8} {'wall_agree':>11} {'confidence':>11} "
          f"{'success':>8}")
    print("-" * 80)

    for theta_deg in ROTATIONS_DEG:
        grid_a, grid_b = build_maps(theta_deg, *TRANSLATION_CELLS)
        res = mc.merge_maps(grid_a, grid_b,
                            resolution_m_per_cell=RESOLUTION,
                            confidence_threshold=0.5)

        if res["transform_meters"] is not None:
            theta_rec = math.degrees(res["transform_meters"]["theta"])
            err = abs(theta_rec - (-theta_deg))
            err = min(err, abs(err - 360))
        else:
            theta_rec = float("nan")
            err = float("nan")

        d = res["diagnostics"]
        wall_agree = d.get("wall_agreement", 0.0)
        print(f"{theta_deg:>10.1f} {theta_rec:>10.2f} {err:>8.2f} "
              f"{d.get('inliers', 0):>8d} {wall_agree:>11.4f} "
              f"{res['confidence']:>11.4f} {str(res['success']):>8}")

        rows.append({
            "theta_gt_deg": theta_deg,
            "theta_recovered_deg": theta_rec,
            "angle_error_deg": err,
            "inliers": d.get("inliers", 0),
            "wall_agreement": wall_agree,
            "overlap_ratio": d.get("overlap_ratio", 0.0),
            "confidence": res["confidence"],
            "success": res["success"],
        })

    out_csv = os.path.join(RESULTS_DIR, "robustness_sweep.csv")
    with open(out_csv, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        for r in rows:
            writer.writerow(r)
    print(f"\nwrote {out_csv}")

    out_json = os.path.join(RESULTS_DIR, "robustness_sweep.json")
    with open(out_json, "w") as fh:
        json.dump(rows, fh, indent=2)
    print(f"wrote {out_json}")


if __name__ == "__main__":
    main()
