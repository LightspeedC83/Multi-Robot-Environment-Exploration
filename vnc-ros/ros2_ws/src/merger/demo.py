#!/usr/bin/env python3
"""
demo.py
-----------
End-to-end demo of the map coordinator:

  1. Load the two synthetic robot maps (grid_a, grid_b) -- B has been rotated
     and translated by a known ground-truth amount, simulating an unknown
     inter-robot odom offset.
  2. Run the coordinator to recover the transform and fuse the maps.
  3. Compare recovered transform against ground truth.
  4. Render a set of PNGs that can be dropped into the intermediate-update
     slides.

All output images go in ./results/.
"""

import json
import os
import math
import numpy as np
import cv2
from PIL import Image

import map_coordinator as mc

RESULTS_DIR = "results"
os.makedirs(RESULTS_DIR, exist_ok=True)

# Map resolution from maze.yml; lets us report meter-space transforms.
RESOLUTION_M_PER_CELL = 0.05


# -----------------------------------------------------------------------------
# Rendering helpers (kept here so map_coordinator.py stays free of viz code).
# -----------------------------------------------------------------------------

def render_grid(grid):
    """Render an occupancy grid as an RGB image.
       free=white, occupied=black, unknown=red-tinted gray.
    """
    h, w = grid.shape
    img = np.zeros((h, w, 3), dtype=np.uint8)

    known = grid >= 0
    intensity = np.clip(grid.astype(np.float32), 0, 100) / 100.0
    gray = (255 - 255 * intensity).astype(np.uint8)
    img[known] = np.stack([gray[known]] * 3, axis=-1)

    # Unknown cells: muted red so they're easy to read in the figures.
    img[~known] = [80, 40, 40]
    return img


def save_png(arr, path):
    Image.fromarray(arr).save(os.path.join(RESULTS_DIR, path))
    print(f"  wrote {os.path.join(RESULTS_DIR, path)}")


def render_matches(result, max_matches=80):
    """Draw the inlier matches on top of the two grayscale grids."""
    img_a = cv2.cvtColor(result["_img_a"], cv2.COLOR_GRAY2BGR)
    img_b = cv2.cvtColor(result["_img_b"], cv2.COLOR_GRAY2BGR)

    matches = result["_matches"]
    mask = result["_inlier_mask"]

    if mask is not None:
        inlier_matches = [m for m, ok in zip(matches, mask.ravel()) if ok]
    else:
        inlier_matches = []

    # cv2.drawMatches expects DMatch list; pick at most max_matches to keep
    # the figure readable.
    show = inlier_matches[:max_matches]
    out = cv2.drawMatches(
        img_a, result["_kp_a"],
        img_b, result["_kp_b"],
        show, None,
        matchColor=(0, 255, 0),
        singlePointColor=(255, 0, 0),
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )
    # cv2 returns BGR; convert to RGB before saving.
    return cv2.cvtColor(out, cv2.COLOR_BGR2RGB)


def render_overlay(grid_a, grid_b, transform_pixels):
    """Visualize alignment quality: warp B into A's frame on a canvas big
    enough to hold both, then draw A in green and B in magenta. Overlapping
    cells show as white.
    """
    canvas_w, canvas_h, M_a, M_b = mc._expand_canvas(
        grid_a, None, transform_pixels, grid_b.shape
    )

    # Warp the occupancy grids into the canvas.
    prob_a, known_a = mc._warp_occupancy(grid_a, M_a, canvas_w, canvas_h)
    prob_b, known_b = mc._warp_occupancy(grid_b, M_b, canvas_w, canvas_h)

    # Treat anything with prob >= 0.5 (and known) as "wall" for the visualization.
    wall_a = known_a & (prob_a >= 0.5)
    wall_b = known_b & (prob_b >= 0.5)

    img = np.full((canvas_h, canvas_w, 3), 255, dtype=np.uint8)

    # Background tint: any cell either robot has *observed* (known) gets a
    # very light gray, so the explored region is visible against unknown.
    seen = known_a | known_b
    img[seen] = [240, 240, 240]

    # A's walls -> green channel only.
    img[wall_a] = [0, 180, 0]
    # B's walls -> magenta only.
    img[wall_b] = [200, 0, 200]
    # Overlap -> dark gray (means alignment is correct).
    overlap = wall_a & wall_b
    img[overlap] = [40, 40, 40]
    return img


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------

def main():
    grid_a = np.load("grid_a.npy")
    grid_b = np.load("grid_b.npy")

    with open("ground_truth.json") as fh:
        gt = json.load(fh)

    print("=" * 60)
    print("Map Coordinator Demo")
    print("=" * 60)
    print(f"  grid A: {grid_a.shape}")
    print(f"  grid B: {grid_b.shape}")
    print(f"  ground truth applied to B:")
    print(f"    rotation    : {gt['theta_deg']:.2f} deg")
    print(f"    translation : ({gt['tx_cells_applied_to_b']:.1f}, "
          f"{gt['ty_cells_applied_to_b']:.1f}) cells")
    print()

    # Save inputs as figures.
    save_png(render_grid(grid_a), "01_input_grid_a.png")
    save_png(render_grid(grid_b), "02_input_grid_b.png")

    # Run alignment + fusion.
    result = mc.merge_maps(
        grid_a, grid_b,
        resolution_m_per_cell=RESOLUTION_M_PER_CELL,
        confidence_threshold=0.5,
    )

    print("Alignment diagnostics:")
    for k, v in result["diagnostics"].items():
        if isinstance(v, float):
            print(f"  {k:20s}: {v:.4f}")
        else:
            print(f"  {k:20s}: {v}")
    print(f"  confidence          : {result['confidence']:.4f}")
    print(f"  success             : {result['success']}")

    if result["transform_meters"] is not None:
        tm = result["transform_meters"]
        recovered_deg = math.degrees(tm["theta"])
        print()
        print("Recovered B -> A transform:")
        print(f"    rotation    : {recovered_deg:+.2f} deg")
        print(f"    translation : ({tm['tx']:.3f}, {tm['ty']:.3f}) m  "
              f"= ({tm['tx']/RESOLUTION_M_PER_CELL:.1f}, "
              f"{tm['ty']/RESOLUTION_M_PER_CELL:.1f}) cells")

        # The ground-truth transform was applied *to* B, so to bring B back to
        # A the coordinator should recover the inverse: rotation -theta_gt
        # and translation that undoes (tx_gt, ty_gt) under that rotation.
        expected_theta = -gt["theta_rad"]
        c, s = math.cos(expected_theta), math.sin(expected_theta)
        # Inverse of "rotate by theta_gt, then translate by (tx,ty)" is
        # "translate by -(tx,ty), then rotate by -theta_gt", which gives
        # translation:
        expected_tx = -(c * gt["tx_cells_applied_to_b"]
                        - s * gt["ty_cells_applied_to_b"])
        expected_ty = -(s * gt["tx_cells_applied_to_b"]
                        + c * gt["ty_cells_applied_to_b"])
        print()
        print("Expected B -> A transform (inverse of ground truth):")
        print(f"    rotation    : {math.degrees(expected_theta):+.2f} deg")
        print(f"    translation : ({expected_tx:.1f}, {expected_ty:.1f}) cells")

        # Errors. We also have to fold in the +40 padding embedding of B.
        # The padded B was placed at canvas offset (40, 40), so the
        # coordinator sees a B image whose 'true' origin in A's frame is
        # shifted by an additional (-c*40 + s*40, -s*40 - c*40) relative
        # to the unpadded ground truth. We report the angle error (which is
        # padding-invariant) as the primary metric.
        angle_err_deg = abs(recovered_deg - math.degrees(expected_theta))
        # Normalize to [0, 180].
        angle_err_deg = min(angle_err_deg, abs(angle_err_deg - 360))
        print()
        print(f"Angle error: {angle_err_deg:.2f} deg")

    # Figures: matches and overlay.
    if result["_inlier_mask"] is not None:
        save_png(render_matches(result), "03_inlier_matches.png")

    if result["transform_pixels"] is not None:
        overlay = render_overlay(grid_a, grid_b, result["transform_pixels"])
        save_png(overlay, "04_alignment_overlay.png")

    if result["merged_grid"] is not None:
        save_png(render_grid(result["merged_grid"]), "05_merged_map.png")
        np.save(os.path.join(RESULTS_DIR, "merged_grid.npy"),
                result["merged_grid"])
        print(f"  wrote {RESULTS_DIR}/merged_grid.npy")

    # Save the report.
    report_path = os.path.join(RESULTS_DIR, "report.json")
    serializable = {
        "ground_truth": gt,
        "diagnostics": result["diagnostics"],
        "confidence": result["confidence"],
        "success": result["success"],
        "transform_meters": result["transform_meters"],
        "transform_pixels": (None if result["transform_pixels"] is None
                             else result["transform_pixels"].tolist()),
    }
    with open(report_path, "w") as fh:
        json.dump(serializable, fh, indent=2)
    print(f"  wrote {report_path}")
    print()
    print("Done.")


if __name__ == "__main__":
    main()
