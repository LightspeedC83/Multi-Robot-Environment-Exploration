#!/usr/bin/env python3

#Aligns two occupancy grids from two robots with unknown relative odom frames,
#and fuses them into a single global occupancy grid.

#Pipeline:
#  1. Convert each occupancy grid to a grayscale image suitable for feature
#     detection. Free/occupied/unknown cells are mapped to distinct values.
#  2. Detect ORB features on both grids.
#  3. Brute-force match the descriptors and filter with Lowe's ratio test.
#  4. Estimate a rigid transform (translation + rotation, no scale) with RANSAC.
#  5. Score the alignment with an occupancy-cell agreement metric to act as a
#     confidence value. The coordinating node only commits when this exceeds a
#     threshold.
#  6. Warp robot B's grid into robot A's frame and fuse the two grids by
#     combining log-odds for each cell.

#this module is intentionally only numpy and OpenCV so it can be unit tested
#without ROS2. Integration is a thin wrapper that hands occupancy-grid messages
#to `merge_maps`.

#Occupancy-grid convention (matching nav_msgs/OccupancyGrid):
#    -1   = unknown
#     0   = free
#   100   = occupied
# (intermediate values allowed when probabilistic mapping is on)

#Work by: Ajwa Shahid
#Credit: AI was used to help create this file. 

import math
import numpy as np
import cv2

# Constants

# Grayscale values used when rasterizing (just means converting grid to image) an occupancy grid for ORB
GRAY_FREE      = 255   # bright = free
GRAY_OCCUPIED  = 0     # dark   = wall
GRAY_UNKNOWN   = 128   # middle value

# ORB feature detector budget. Occupancy grids tend to be sparse-feature
# images (long flat walls, few corners), so we ask for a lot of keypoints
# and rely on RANSAC for outlier rejection.
ORB_N_FEATURES = 5000

# Lowe's ratio for filtering ambiguous matches. A slightly looser ratio
# helps on low-feature maps where descriptors are less discriminative.
LOWE_RATIO     = 0.85

# RANSAC reprojection threshold (in pixels of the upscaled grid image).
RANSAC_REPROJ_THRESH_PX = 5.0

# Below this many inliers we treat the alignment as failed regardless of the
# overlap score.
MIN_INLIERS_FOR_ALIGNMENT = 10

# Upscale factor applied to grids before feature detection. Occupancy grids
# are small (~200 cells per side), and ORB's pyramid struggles with sparse
# features at low resolution; upscaling lets it find more keypoints on wall
# corners. We undo this factor when reporting the final transform.
FEATURE_UPSCALE = 3


# Grid to image

def occupancy_to_gray(grid):
    """Rasterize an occupancy grid (int values in {-1, 0..100}) to uint8.
    -1   -> GRAY_UNKNOWN
     0   -> GRAY_FREE
    100  -> GRAY_OCCUPIED
    intermediate probabilities are linearly interpolated.
    """
    grid = np.asarray(grid)
    img = np.full(grid.shape, GRAY_UNKNOWN, dtype=np.uint8)

    known = grid >= 0
    # Linear mapping: occ_prob 0 -> GRAY_FREE, 100 -> GRAY_OCCUPIED.
    prob = np.clip(grid[known].astype(np.float32), 0, 100) / 100.0
    img[known] = (GRAY_FREE + prob * (GRAY_OCCUPIED - GRAY_FREE)).astype(np.uint8)
    return img


# Feature-based alignment

def detect_and_match(img_a, img_b, upscale=FEATURE_UPSCALE,
                     mask_a=None, mask_b=None):
    #Detect ORB features in two images and brute-force match them.

    #The grids are upscaled before feature detection
    #this code finds matching landmarks/features between the two occupancy maps and keeps only the matches that are distinctive enough to trust.
    """
    AI suggestion:
    mask_a / mask_b: optional uint8 masks (same shape as img_a / img_b before
    upscaling) that restrict ORB to keypoints inside the known region.
    Important for occupancy-grid matching because the unknown / free boundary
    is a strong but spurious feature -- it reflects where the robot has
    driven, not where the walls are.
    """

    #Returns (kp_a, kp_b, good_matches, upscale). keypoints in map a and b
    
    if mask_a is not None:
        mask_a = mask_a.astype(np.uint8)
    if mask_b is not None:
        mask_b = mask_b.astype(np.uint8)

    if upscale != 1:
        h_a, w_a = img_a.shape
        h_b, w_b = img_b.shape
        img_a = cv2.resize(img_a, (w_a * upscale, h_a * upscale),
                           interpolation=cv2.INTER_NEAREST)
        img_b = cv2.resize(img_b, (w_b * upscale, h_b * upscale),
                           interpolation=cv2.INTER_NEAREST)
        if mask_a is not None:
            mask_a = cv2.resize(mask_a, (w_a * upscale, h_a * upscale),
                                interpolation=cv2.INTER_NEAREST)
        if mask_b is not None:
            mask_b = cv2.resize(mask_b, (w_b * upscale, h_b * upscale),
                                interpolation=cv2.INTER_NEAREST)

    orb = cv2.ORB_create(
        nfeatures=ORB_N_FEATURES,
        scaleFactor=1.2,
        nlevels=8,
        edgeThreshold=10,
        fastThreshold=5,  # lower than default 20 -> more, weaker corners
    )
    kp_a, des_a = orb.detectAndCompute(img_a, mask_a)
    kp_b, des_b = orb.detectAndCompute(img_b, mask_b)

    if des_a is None or des_b is None or len(kp_a) < 2 or len(kp_b) < 2:
        return kp_a, kp_b, [], upscale

    # KNN brute force with Hamming distance (ORB descriptors are binary).
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False) #create brute force matcher
    knn = bf.knnMatch(des_a, des_b, k=2)#for every descriptor find 2 closest matches in other image

    good = []
    for pair in knn:
        if len(pair) < 2: #if less than 2 matches, skip it
            continue
        m, n = pair
        if m.distance < LOWE_RATIO * n.distance:
            good.append(m) #keep a list of good matches that pass Lowe's ratio test
    return kp_a, kp_b, good, upscale


def estimate_rigid_transform(kp_a, kp_b, matches, n_attempts=5):
    #Use RANSAC to estimate a rigid 2x3 transform mapping image B onto A.

    #We use cv2.estimateAffinePartial2D, which constrains the transform to
    #rotation + uniform scale + translation. We then normalize the scale to 1
    #because two LiDAR-built grids share the same resolution, only rotation
    #and translation should differ between them.

   # Because feature matching on sparse occupancy grids is unreliable, we run
    #RANSAC several times and return all candidate transforms (sorted by
    #inlier count). The caller can then re-rank them by a downstream metric
    #like wall-agreement.

    # Returns a list of (M, inlier_mask, n_inliers), best-inlier-first.
    # Empty list on failure.
    
    if len(matches) < 4:
        return []

    pts_a = np.float32([kp_a[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    pts_b = np.float32([kp_b[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    candidates = []
    seen_signatures = set()

    for attempt in range(n_attempts):
        # estimateAffinePartial2D finds [s*R | t] mapping pts_b -> pts_a.
        M, inliers = cv2.estimateAffinePartial2D(
            pts_b, pts_a,
            method=cv2.RANSAC,
            ransacReprojThreshold=RANSAC_REPROJ_THRESH_PX,
            maxIters=2000,
            confidence=0.99,
            refineIters=10,
        )
        if M is None or inliers is None:
            continue

        # Normalize scale to exactly 1. the two grids share resolution.
        a, b = M[0, 0], M[0, 1]
        s = math.sqrt(a * a + b * b)
        if s > 1e-6:
            M[0, 0] /= s
            M[0, 1] /= s
            M[1, 0] /= s
            M[1, 1] /= s

        n_inliers = int(inliers.sum())

        # De-duplicate near-identical candidates. We keep the signature
        # coarse-grained (5-degree, 20-pixel buckets) so RANSAC's random
        # restarts give us genuinely different transforms, not just rounding
        # variations.
        theta = math.atan2(M[1, 0], M[0, 0])
        sig = (round(math.degrees(theta) / 5.0),
               round(float(M[0, 2]) / 20.0),
               round(float(M[1, 2]) / 20.0))
        if sig in seen_signatures:
            continue
        seen_signatures.add(sig)

        candidates.append((M, inliers, n_inliers))

    candidates.sort(key=lambda c: c[2], reverse=True)
    return candidates


# Fusion

def _expand_canvas(grid_a, grid_b_warped_into_a_frame, transform_2x3, shape_b):
    """Compute the bounding box of (grid_a U warped(grid_b)) and the affine
    transform that places A and B into that canvas.

    Returns (canvas_w, canvas_h, M_a, M_b) where M_a / M_b are 2x3 affines
    mapping each grid into the canvas.
    """
    h_a, w_a = grid_a.shape
    h_b, w_b = shape_b

    # Corners of B in B's own frame.
    corners_b = np.float32([[0, 0], [w_b, 0], [w_b, h_b], [0, h_b]]).reshape(-1, 1, 2)
    # Warp them into A's frame using transform_2x3.
    corners_b_in_a = cv2.transform(corners_b, transform_2x3).reshape(-1, 2)

    # All corners involved in the merged canvas, expressed in A's frame.
    all_corners = np.vstack([
        np.float32([[0, 0], [w_a, 0], [w_a, h_a], [0, h_a]]),
        corners_b_in_a,
    ])
    min_x, min_y = np.floor(all_corners.min(axis=0)).astype(int)
    max_x, max_y = np.ceil(all_corners.max(axis=0)).astype(int)

    canvas_w = int(max_x - min_x)
    canvas_h = int(max_y - min_y)

    # Translation that shifts A's origin to the canvas origin.
    shift = np.array([[1.0, 0.0, -min_x],
                      [0.0, 1.0, -min_y]], dtype=np.float32)

    # M_a: identity + shift
    M_a = shift.copy()

    # M_b: compose transform_2x3 (B->A) with the shift (A->canvas).
    T_b_to_a = np.vstack([transform_2x3, [0, 0, 1]])
    T_shift  = np.vstack([shift,         [0, 0, 1]])
    M_b = (T_shift @ T_b_to_a)[:2, :]

    return canvas_w, canvas_h, M_a, M_b


def _warp_occupancy(grid, M, out_w, out_h):
    """Warp an occupancy grid into a new canvas. Returns the warped occupancy
    plus a mask of cells that are 'known' after warping.
    """
    # Probability channel: occupied prob in [0, 1]; unknown cells get -1.
    prob = np.where(grid >= 0, grid.astype(np.float32) / 100.0, -1.0)

    # Warp prob; cells that fall outside source get the border value -1.
    warped_prob = cv2.warpAffine(
        prob, M, (out_w, out_h),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=-1.0,
    )
    known_mask = warped_prob >= 0
    return warped_prob, known_mask


def fuse_grids(grid_a, grid_b, transform_2x3):
    """Fuse two occupancy grids using a known rigid transform from B to A.

    Returns the merged occupancy grid in standard {-1, 0..100} form, with shape
    large enough to contain both inputs.

    Fusion rule (per cell):
      - if neither robot saw it -> unknown (-1)
      - if only one robot saw it -> that robot's value
      - if both saw it          -> average in log-odds space (Bayesian combine)
    """
    canvas_w, canvas_h, M_a, M_b = _expand_canvas(
        grid_a, None, transform_2x3, grid_b.shape
    )

    prob_a, known_a = _warp_occupancy(grid_a, M_a, canvas_w, canvas_h)
    prob_b, known_b = _warp_occupancy(grid_b, M_b, canvas_w, canvas_h)

    # Convert probabilities to log-odds for known cells, then sum (recursive
    # Bayesian update with each grid treated as one independent observation).
    # Clamp to keep log-odds finite.
    def to_logodds(p):
        p = np.clip(p, 0.01, 0.99)
        return np.log(p / (1.0 - p))

    log_a = np.where(known_a, to_logodds(prob_a), 0.0)
    log_b = np.where(known_b, to_logodds(prob_b), 0.0)
    log_sum = log_a + log_b

    # Back to probability.
    p_merged = 1.0 - 1.0 / (1.0 + np.exp(log_sum))

    # Build the final {-1, 0..100} grid.
    out = np.full((canvas_h, canvas_w), -1, dtype=np.int16)
    both_known = known_a | known_b
    out[both_known] = np.round(p_merged[both_known] * 100).astype(np.int16)

    return out


# Rotation disambiguation

def _compose_rotation_about_point(M, extra_theta_rad, pivot_xy):
    """Return a new 2x3 transform equal to (rotate by extra_theta around pivot)
    composed with M.

    pivot is given in the destination frame (i.e. the frame M maps *into*).
    """
    c, s = math.cos(extra_theta_rad), math.sin(extra_theta_rad)
    px, py = pivot_xy
    # R about pivot: T(p) * R * T(-p)
    R_about_p = np.array([
        [c, -s, px - c * px + s * py],
        [s,  c, py - s * px - c * py],
    ], dtype=np.float32)

    M_h = np.vstack([M, [0, 0, 1]])
    R_h = np.vstack([R_about_p, [0, 0, 1]])
    return (R_h @ M_h)[:2, :].astype(np.float32)


def _try_rotation_variants(grid_a, grid_b, M_grid, n_inliers,
                           swap_margin=0.10):
    """
    Suggested addition by AI
    Given a candidate transform (in grid coords), test the four 90-degree
    rotational variants of it and return the (transform, confidence,
    diagnostics, extra_deg) tuple chosen as best.

    Why this exists: on near-symmetric environments (e.g., a roughly square
    maze with strong outer walls), ORB+RANSAC sometimes locks onto a
    transform that is rotated 90 or 180 degrees from the truth. The outer
    walls still match, so the inlier count is high, but the interior is
    misaligned. We can detect this cheaply by rotating the candidate about
    the centroid of B's known region and re-scoring against wall agreement.

    The swap_margin parameter prevents the disambiguator from flipping the
    transform based on tiny score differences: a 90- or 180-deg variant
    only wins if its confidence beats the 0-deg variant by at least this
    margin. Otherwise we trust the feature-based estimate and keep extra=0.
    """
    # Pivot: centroid of B's known cells, mapped into A's frame via M_grid.
    h_b, w_b = grid_b.shape
    ys, xs = np.where(grid_b >= 0)
    if len(xs) == 0:
        cx_b, cy_b = w_b / 2.0, h_b / 2.0
    else:
        cx_b, cy_b = float(xs.mean()), float(ys.mean())
    pivot = (M_grid[0, 0] * cx_b + M_grid[0, 1] * cy_b + M_grid[0, 2],
             M_grid[1, 0] * cx_b + M_grid[1, 1] * cy_b + M_grid[1, 2])

    # Always evaluate the 0-deg variant first so we can compare against it.
    base_conf, base_info = alignment_confidence(grid_a, grid_b, M_grid, n_inliers)
    best_transform = M_grid
    best_conf = base_conf
    best_info = base_info
    best_extra = 0.0

    for extra_deg in (90.0, 180.0, 270.0):
        M_var = _compose_rotation_about_point(
            M_grid, math.radians(extra_deg), pivot,
        )
        c, info = alignment_confidence(grid_a, grid_b, M_var, n_inliers)
        if c > best_conf + swap_margin:
            best_conf = c
            best_info = info
            best_transform = M_var
            best_extra = extra_deg

    return best_transform, best_conf, best_info, best_extra


# Confidence scoring

def alignment_confidence(grid_a, grid_b, transform_2x3, n_inliers):
    """Compute a confidence in [0, 1] for the proposed transform.

    Occupancy-grid alignment is tricky to score because most cells are free
    space, and any reasonable-looking transform will get high free-vs-free
    agreement even if walls are misaligned. We therefore compute the
    confidence from three orthogonal signals:

      * wall agreement: of all cells where *either* robot claims a wall,
        what fraction do both agree on? Free-vs-free agreement is ignored.
      * overlap size: small overlaps are unreliable, so we down-weight
        confidence when the joint observed region is small.
      * inlier strength: scaled RANSAC inlier count, saturating at 40.

    Returns (confidence, diagnostics_dict).
    """
    canvas_w, canvas_h, M_a, M_b = _expand_canvas(
        grid_a, None, transform_2x3, grid_b.shape
    )
    prob_a, known_a = _warp_occupancy(grid_a, M_a, canvas_w, canvas_h)
    prob_b, known_b = _warp_occupancy(grid_b, M_b, canvas_w, canvas_h)

    both_known = known_a & known_b
    overlap_cells = int(both_known.sum())

    occ_a = (prob_a >= 0.5) & known_a
    occ_b = (prob_b >= 0.5) & known_b

    # Allow a 1-cell tolerance in the agreement check. ORB+RANSAC routinely
    # produces transforms that are correct to within ~1 pixel of the upscaled
    # image (so ~1/3 cell), but pixel-exact wall comparison would still
    # penalize these as misaligned. Dilating one mask by a small kernel gives
    # the metric a tolerance band consistent with the underlying registration
    # precision.
    kernel = np.ones((3, 3), np.uint8)
    occ_b_dilated = cv2.dilate(occ_b.astype(np.uint8), kernel, iterations=1) > 0
    occ_a_dilated = cv2.dilate(occ_a.astype(np.uint8), kernel, iterations=1) > 0

    # Cells where at least one robot sees a wall (inside the joint observed
    # region).
    wall_union = (occ_a | occ_b) & both_known
    wall_union_count = int(wall_union.sum())
    # Cells where A's wall matches a B wall within tolerance, OR vice versa.
    wall_agree = (occ_a & occ_b_dilated) | (occ_b & occ_a_dilated)
    wall_agree_count = int(wall_agree.sum())

    if wall_union_count < 5:
        wall_agreement = 0.0
    else:
        wall_agreement = min(wall_agree_count / wall_union_count, 1.0)

    # Down-weight when joint overlap is tiny relative to either input grid.
    a_known_total = int(known_a.sum())
    b_known_total = int(known_b.sum())
    smaller = max(min(a_known_total, b_known_total), 1)
    overlap_ratio = min(overlap_cells / smaller, 1.0)
    # Smooth ramp: little credit until at least ~10% of the smaller map
    # overlaps with the other.
    overlap_score = min(overlap_ratio / 0.10, 1.0)

    inlier_score = min(n_inliers / 40.0, 1.0)

    # Geometric mean: every signal must be reasonable.
    conf = (max(wall_agreement, 0.0)
            * max(overlap_score, 0.0)
            * max(inlier_score, 0.0)) ** (1.0 / 3.0)

    return conf, {
        "overlap_cells": overlap_cells,
        "wall_union_cells": wall_union_count,
        "wall_agreement": wall_agreement,
        "overlap_ratio": overlap_ratio,
        "inliers": n_inliers,
    }


# Public entry point

def merge_maps(grid_a, grid_b, resolution_m_per_cell=None,
               confidence_threshold=0.5):
    """Top-level: align grid B to grid A and fuse them.

    Args:
        grid_a, grid_b: 2D numpy arrays in nav_msgs/OccupancyGrid convention.
        resolution_m_per_cell: optional; used to also report the translation
            of the recovered transform in meters.
        confidence_threshold: alignment is rejected (merged grid returned as
            None) below this confidence.

    Returns a dict with keys:
        success           : bool
        confidence        : float
        transform_pixels  : 2x3 affine, B -> A in pixel coords (or None)
        transform_meters  : dict with tx, ty (meters) and theta (rad), or None
        merged_grid       : merged occupancy grid (or None on failure)
        diagnostics       : dict with intermediate counts for debugging, just like extra info 
    """
    img_a = occupancy_to_gray(grid_a)
    img_b = occupancy_to_gray(grid_b)

    # Mask: cells that are known. We erode it slightly so ORB doesn't latch
    # onto features at the explored / unknown boundary, which encode the
    # robot's trajectory, not the environment.
    erode_kernel = np.ones((3, 3), np.uint8)
    mask_a = cv2.erode((grid_a >= 0).astype(np.uint8) * 255, erode_kernel,
                       iterations=2)
    mask_b = cv2.erode((grid_b >= 0).astype(np.uint8) * 255, erode_kernel,
                       iterations=2)

    kp_a, kp_b, matches, upscale = detect_and_match(
        img_a, img_b, mask_a=mask_a, mask_b=mask_b,
    )

    result = {
        "success": False,
        "confidence": 0.0,
        "transform_pixels": None,
        "transform_meters": None,
        "merged_grid": None,
        "diagnostics": {
            "kp_a": len(kp_a) if kp_a else 0,
            "kp_b": len(kp_b) if kp_b else 0,
            "good_matches": len(matches),
            "inliers": 0,
        },
        # Kept around for visualization purposes:
        "_img_a": img_a,
        "_img_b": img_b,
        "_kp_a": kp_a,
        "_kp_b": kp_b,
        "_matches": matches,
        "_inlier_mask": None,
        "_upscale": upscale,
    }

    if len(matches) < 4:
        return result

    candidates = estimate_rigid_transform(kp_a, kp_b, matches, n_attempts=6)

    if not candidates:
        return result

    # Rank candidates by the structural-confidence metric, not by raw inlier
    # count. This is the key insight that gets us past spurious matches on
    # repetitive corridor structures.
    #
    # For each candidate we also test the four 90-degree rotational variants
    # of it, which handles the failure mode where ORB locks onto a transform
    # that aligns the outer walls of a near-symmetric environment but flips
    # the interior.
    best_conf = -1.0
    best = None
    best_extra_rot_deg = 0.0
    for M_cand, inliers_cand, n_inliers_cand in candidates:
        if n_inliers_cand < MIN_INLIERS_FOR_ALIGNMENT:
            continue
        # Rescale to grid coordinates for the confidence check.
        if upscale != 1:
            M_grid = M_cand.copy()
            M_grid[0, 2] /= upscale
            M_grid[1, 2] /= upscale
        else:
            M_grid = M_cand.copy()

        variant = _try_rotation_variants(grid_a, grid_b, M_grid, n_inliers_cand)
        if variant is None:
            continue
        M_best_var, c, info, extra_deg = variant
        if c > best_conf:
            best_conf = c
            best = (M_cand, inliers_cand, n_inliers_cand, M_best_var, info)
            best_extra_rot_deg = extra_deg

    if best is None:
        return result

    M_upscaled, inlier_mask, n_inliers, M, score_info = best
    result["diagnostics"]["inliers"] = n_inliers
    result["diagnostics"]["n_candidates"] = len(candidates)
    result["diagnostics"]["disambig_rotation_deg"] = best_extra_rot_deg
    result["_inlier_mask"] = inlier_mask
    result["confidence"] = best_conf
    result["diagnostics"].update(score_info)
    result["transform_pixels"] = M

    confidence = best_conf

    if resolution_m_per_cell is not None:
        theta = math.atan2(M[1, 0], M[0, 0])
        tx_m = float(M[0, 2]) * resolution_m_per_cell
        ty_m = float(M[1, 2]) * resolution_m_per_cell
        result["transform_meters"] = {"tx": tx_m, "ty": ty_m, "theta": theta}

    if confidence < confidence_threshold:
        return result

    result["merged_grid"] = fuse_grids(grid_a, grid_b, M)
    result["success"] = True
    return result
