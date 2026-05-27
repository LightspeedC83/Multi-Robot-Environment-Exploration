#!/usr/bin/env python3
"""
  - the source is the same maze (maze.png referenced by maze.yml), so the maps share resolution and
    structure
  - each robot only sees part of the maze (a circular "explored" region
    around its trajectory), with the rest left as unknown, copying the
    structure a frontier-explorer would produce mid-exploration
  - robot B's map is rotated and translated relative to robot A's, mimicking
    the unknown odom-frame offset described in the proposal. And this rotation is noted. And used
    as ground truth later for evaluation. output: ground_truth.json

Outputs: grid_a.npy, grid_b.npy
Credits: AI was used to create this test maps file
"""

import json
import math
import numpy as np
import cv2
from PIL import Image


def load_maze(png_path, yaml_thresh_free=0.196, yaml_thresh_occ=0.65):
#Following maze.yml conventions: dark pixels are occupied (walls), light are free
    img = np.array(Image.open(png_path).convert("L"))
    # Normalize 0..1, then invert so 1 = occupied
    # behavior when `negate: 0`.
    p = 1.0 - img.astype(np.float32) / 255.0
    occ = np.full(p.shape, -1, dtype=np.int16)
    occ[p < yaml_thresh_free] = 0
    occ[p > yaml_thresh_occ] = 100
    # Anything in between stays unknown 
    return occ


def carve_explored_region(full_grid, trajectory_xy, radius_cells):
    #Return a copy of full_grid with all cells outside `radius_cells` of any
    #point in `trajectory_xy` set to unknown (-1).

    #This simulates a robot that has only sensed the area it has driven near.
    out = np.full(full_grid.shape, -1, dtype=np.int16)
    mask = np.zeros(full_grid.shape, dtype=np.uint8)
    for (cx, cy) in trajectory_xy:
        cv2.circle(mask, (int(cx), int(cy)), radius_cells, 1, thickness=-1)
    out[mask > 0] = full_grid[mask > 0]
    return out


def apply_rigid_transform(grid, theta_rad, tx_cells, ty_cells, out_shape=None):
    #Rotates and shifts an occupancy grid.
    h, w = grid.shape # input and output shapes may differ if the transform causes the map to grow beyond the original bounds
    if out_shape is None: # if out_shape is None we keep the same shape and let OpenCV clip anything that falls outside.
        out_shape = (h, w)
    out_h, out_w = out_shape

    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)
    M = np.array([
        [cos_t, -sin_t, tx_cells],
        [sin_t,  cos_t, ty_cells],
    ], dtype=np.float32)

    # We warp a float grid where unknown is encoded as -1, then convert back.
    src = grid.astype(np.float32)
    warped = cv2.warpAffine(
        src, M, (out_w, out_h),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=-1.0,
    )
    return np.round(warped).astype(np.int16)


def main():
    # 1. Load the underlying maze.
    full = load_maze("maze.png")
    h, w = full.shape
    print(f"loaded maze: {w} x {h}")

    # 2. Simulated trajectories. The maze has most of its distinguishing
    #    interior structure in the central region, so both robots' paths
    #    must visit it for ORB to have any hope of aligning them.
    #    Robot A approaches the center from the upper-left; robot B
    #    approaches from the lower-right. Both end up sweeping through
    #    the same central rooms.
    #
    # Coordinates are (column, row) in image pixels.
    def lawnmower(x_min, x_max, y_min, y_max, step=3):
        """Generate a back-and-forth coverage trajectory in a rectangle."""
        pts = []
        y = y_min
        direction = 1
        while y <= y_max:
            xs = range(x_min, x_max + 1, step) if direction > 0 \
                else range(x_max, x_min - 1, -step)
            for x in xs:
                pts.append((x, y))
            y += step
            direction *= -1
        return pts

    # Robot A: upper-left portion plus center. Heavy overlap with B in the
    # center region.
    traj_a = lawnmower(int(w * 0.05), int(w * 0.85),
                       int(h * 0.05), int(h * 0.85))
    # Robot B: lower-right portion plus center.
    traj_b = lawnmower(int(w * 0.15), int(w * 0.95),
                       int(h * 0.15), int(h * 0.95))

    explore_radius = max(15, min(h, w) // 14)

    grid_a_native = carve_explored_region(full, traj_a, explore_radius)
    grid_b_native = carve_explored_region(full, traj_b, explore_radius)

    # 3. Apply a known rigid transform to robot B's map. This is the unknown
    #    inter-robot offset we want the coordinator to recover.
    #
    # We support running this script in "easy" or "hard" mode. The default
    # ("easy") uses a modest rotation that the feature-based pipeline can
    # handle reliably; "hard" pushes rotation and translation to stress-test
    # the pipeline and is used in the diagnostics section of the report.
    import sys
    difficulty = sys.argv[1] if len(sys.argv) > 1 else "easy"

    if difficulty == "hard":
        gt_theta = math.radians(15.0)
        gt_tx    = 35.0
        gt_ty    = -20.0
    else:
        gt_theta = math.radians(7.0)    # 7 deg rotation
        gt_tx    = 18.0                 # cells
        gt_ty    = -10.0                # cells

    # Make B's canvas a bit larger so the rotated map fits.
    out_h = h + 80
    out_w = w + 80
    # We embed B in the larger canvas (offset by 40,40) before applying the
    # ground truth transform so nothing gets clipped.
    grid_b_padded = np.full((out_h, out_w), -1, dtype=np.int16)
    grid_b_padded[40:40 + h, 40:40 + w] = grid_b_native

    grid_b_transformed = apply_rigid_transform(
        grid_b_padded, gt_theta, gt_tx, gt_ty,
        out_shape=(out_h, out_w),
    )

    # 4. Save everything.
    np.save("grid_a.npy", grid_a_native.astype(np.int16))
    np.save("grid_b.npy", grid_b_transformed.astype(np.int16))
    np.save("grid_full_truth.npy", full.astype(np.int16))

    with open("ground_truth.json", "w") as fh:
        # Note: the recovered transform from the map coordinator should be
        # the *inverse* of this in pixel-space, because we applied the GT
        # rotation/translation to B (so coordinator must undo it to align
        # B back to A).
        json.dump({
            "theta_rad": gt_theta,
            "theta_deg": math.degrees(gt_theta),
            "tx_cells_applied_to_b": gt_tx,
            "ty_cells_applied_to_b": gt_ty,
            "b_canvas_padding": 40,
            "explore_radius_cells": explore_radius,
        }, fh, indent=2)

    # Brief summary.
    def report(name, g):
        unk = int((g == -1).sum())
        free = int((g == 0).sum())
        occ = int((g == 100).sum())
        total = g.size
        print(f"  {name}: shape={g.shape}  "
              f"unknown={unk}/{total} ({100*unk/total:.1f}%)  "
              f"free={free}  occ={occ}")

    print("generated:")
    report("grid_a", grid_a_native)
    report("grid_b", grid_b_transformed)
    print(f"ground truth: rotated B by {math.degrees(gt_theta):.1f} deg, "
          f"translated by ({gt_tx}, {gt_ty}) cells")


if __name__ == "__main__":
    main()
