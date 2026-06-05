#!/usr/bin/env python3
"""Create report-ready visual evidence from the final path artifacts."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    import cv2
    import numpy as np
except ImportError as exc:  # pragma: no cover - this is a runtime help path.
    raise SystemExit(
        "generate_report_visuals.py needs OpenCV and NumPy. "
        "Run it inside the ROS Docker container after building the workspace."
    ) from exc


Point = Tuple[float, float]
Color = Tuple[int, int, int]
MapSnapshot = Dict[str, Any]

BG: Color = (246, 245, 241)
CARD: Color = (255, 255, 255)
INK: Color = (41, 45, 50)
MUTED: Color = (105, 112, 122)
LINE: Color = (210, 214, 220)
BLUE: Color = (221, 112, 36)
GREEN: Color = (70, 156, 85)
ORANGE: Color = (39, 135, 238)
RED: Color = (58, 75, 230)
DARK: Color = (47, 62, 77)


def default_results_dir() -> Path:
    container_results = Path("/root/ros2_ws/src/final_path_results")
    if container_results.exists():
        return container_results
    return Path(__file__).resolve().parents[2] / "final_path_results"


def read_summary(path: Path) -> Dict[str, str]:
    summary: Dict[str, str] = {}
    if not path.exists():
        return summary
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in raw_line:
            continue
        key, value = raw_line.split(":", 1)
        summary[key.strip()] = value.strip()
    return summary


def read_waypoints(path: Path) -> List[Point]:
    points: List[Point] = []
    with path.open(newline="", encoding="utf-8") as csv_file:
        for row in csv.DictReader(csv_file):
            points.append((float(row["x_m"]), float(row["y_m"])))
    return points


def cumulative_length(points: Sequence[Point]) -> float:
    return sum(
        math.hypot(bx - ax, by - ay)
        for (ax, ay), (bx, by) in zip(points, points[1:])
    )


def make_canvas(width: int, height: int, color: Color = BG) -> np.ndarray:
    return np.full((height, width, 3), color, dtype=np.uint8)


def write_text(
    image: np.ndarray,
    text: str,
    x: int,
    y: int,
    scale: float = 0.62,
    color: Color = INK,
    thickness: int = 1,
) -> int:
    (_width, text_height), baseline = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        thickness,
    )
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA,
    )
    return y + text_height + baseline + 8


def text_width(text: str, scale: float, thickness: int = 1) -> int:
    return cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0][0]


def text_step(scale: float, thickness: int = 1, line_gap: int = 10) -> int:
    (_width, text_height), baseline = cv2.getTextSize(
        "Ag",
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        thickness,
    )
    return text_height + baseline + line_gap


def split_long_token(token: str, max_width_px: int, scale: float, thickness: int = 1) -> List[str]:
    if text_width(token, scale, thickness) <= max_width_px:
        return [token]

    pieces: List[str] = []
    current = ""
    for char in token:
        candidate = current + char
        if current and text_width(candidate, scale, thickness) > max_width_px:
            pieces.append(current)
            current = char
        else:
            current = candidate
    if current:
        pieces.append(current)
    return pieces


def wrap_pixels(text: str, max_width_px: int, scale: float, thickness: int = 1) -> List[str]:
    lines: List[str] = []
    for paragraph in text.splitlines() or [""]:
        if not paragraph.strip():
            lines.append("")
            continue

        current = ""
        tokens: List[str] = []
        for token in paragraph.split(" "):
            tokens.extend(split_long_token(token, max_width_px, scale, thickness))

        for token in tokens:
            candidate = token if not current else f"{current} {token}"
            if current and text_width(candidate, scale, thickness) > max_width_px:
                lines.append(current)
                current = token
            else:
                current = candidate
        if current:
            lines.append(current)
    return lines


def wrapped_text(
    image: np.ndarray,
    text: str,
    x: int,
    y: int,
    width_chars: int,
    scale: float = 0.58,
    color: Color = INK,
    line_gap: int = 10,
) -> int:
    max_width_px = max(80, int(width_chars * 10))
    for line in wrap_pixels(text, max_width_px, scale):
        if not line:
            y += text_step(scale, line_gap=line_gap)
            continue
        y = write_text(image, line, x, y, scale=scale, color=color)
        y += line_gap
    return y


def draw_text_box(
    image: np.ndarray,
    text: str,
    x: int,
    y: int,
    w: int,
    h: int,
    scale: float = 0.56,
    color: Color = INK,
    thickness: int = 1,
    min_scale: float = 0.38,
) -> int:
    """Draw wrapped text and shrink slightly if the box is too full."""
    scale_now = scale
    while scale_now >= min_scale:
        lines = wrap_pixels(text, w, scale_now, thickness)
        step = text_step(scale_now, thickness=thickness, line_gap=9)
        if max(1, len(lines)) * step <= h:
            break
        scale_now -= 0.04

    lines = wrap_pixels(text, w, scale_now, thickness)
    step = text_step(scale_now, thickness=thickness, line_gap=9)
    max_lines = max(1, h // step)
    if len(lines) > max_lines:
        lines = lines[:max_lines]
        if lines:
            while lines[-1] and text_width(lines[-1] + "...", scale_now, thickness) > w:
                lines[-1] = lines[-1][:-1]
            lines[-1] = (lines[-1].rstrip() + "...").strip()

    cursor_y = y
    for line in lines:
        if line:
            cv2.putText(
                image,
                line,
                (x, cursor_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                scale_now,
                color,
                thickness,
                cv2.LINE_AA,
            )
        cursor_y += step
    return cursor_y


def card(image: np.ndarray, x: int, y: int, w: int, h: int, title: str = "") -> None:
    cv2.rectangle(image, (x, y), (x + w, y + h), CARD, -1, cv2.LINE_AA)
    cv2.rectangle(image, (x, y), (x + w, y + h), LINE, 2, cv2.LINE_AA)
    if title:
        write_text(image, title, x + 22, y + 40, scale=0.72, color=DARK, thickness=2)


def resize_to_fit(image: np.ndarray, max_w: int, max_h: int) -> np.ndarray:
    h, w = image.shape[:2]
    ratio = min(max_w / w, max_h / h)
    size = (max(1, int(w * ratio)), max(1, int(h * ratio)))
    return cv2.resize(image, size, interpolation=cv2.INTER_AREA)


def paste(base: np.ndarray, image: np.ndarray, x: int, y: int) -> None:
    h, w = image.shape[:2]
    base[y : y + h, x : x + w] = image


def draw_swatch_legend(
    image: np.ndarray,
    items: Sequence[Tuple[str, Color]],
    x: int,
    y: int,
    max_width: int,
    scale: float = 0.40,
) -> int:
    cursor_x = x
    cursor_y = y
    row_height = 28
    swatch = 16

    for label, color in items:
        item_width = swatch + 10 + text_width(label, scale) + 18
        if cursor_x > x and cursor_x + item_width > x + max_width:
            cursor_x = x
            cursor_y += row_height

        cv2.rectangle(image, (cursor_x, cursor_y - 14), (cursor_x + swatch, cursor_y + 2), color, -1)
        cv2.rectangle(image, (cursor_x, cursor_y - 14), (cursor_x + swatch, cursor_y + 2), (118, 122, 128), 1)
        write_text(image, label, cursor_x + swatch + 9, cursor_y + 2, scale=scale, color=INK)
        cursor_x += item_width

    return cursor_y + row_height - y


def find_optional_image(results_dir: Path, filename: str) -> Optional[np.ndarray]:
    return find_first_existing_image(results_dir, [filename])


def find_first_existing_image(results_dir: Path, filenames: Sequence[str]) -> Optional[np.ndarray]:
    candidates = [
        base / filename
        for filename in filenames
        for base in (results_dir / "snapshots", results_dir / "report_inputs", results_dir)
    ]
    for path in candidates:
        if path.exists():
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is not None:
                return image
    return None


def tile_image(
    image: np.ndarray,
    source: Optional[np.ndarray],
    x: int,
    y: int,
    w: int,
    h: int,
    title: str,
    caption: str,
    accent: Color,
    missing_note: str,
    crop_source: bool = False,
    legend_items: Optional[Sequence[Tuple[str, Color]]] = None,
) -> None:
    card(image, x, y, w, h)
    cv2.rectangle(image, (x, y), (x + w, y + 10), accent, -1)
    write_text(image, title, x + 22, y + 42, scale=0.62, color=INK, thickness=2)

    image_top = y + 58 if crop_source else y + 64
    caption_h = 54 if crop_source else 72
    legend_h = 38 if legend_items else 0
    image_h = h - (76 if crop_source else 88) - caption_h - legend_h
    image_w = w - 44
    cv2.rectangle(image, (x + 22, image_top), (x + 22 + image_w, image_top + image_h), (238, 239, 237), -1)
    cv2.rectangle(image, (x + 22, image_top), (x + 22 + image_w, image_top + image_h), LINE, 1)

    if source is not None:
        if crop_source:
            source = crop_map_content(source)
        fitted = resize_to_fit(source, image_w - 8, image_h - 8)
        fx = x + 22 + (image_w - fitted.shape[1]) // 2
        fy = image_top + (image_h - fitted.shape[0]) // 2
        paste(image, fitted, fx, fy)
    else:
        cv2.line(image, (x + 44, image_top + 30), (x + image_w, image_top + image_h - 30), (195, 198, 199), 2)
        cv2.line(image, (x + image_w, image_top + 30), (x + 44, image_top + image_h - 30), (195, 198, 199), 2)
        draw_text_box(
            image,
            missing_note,
            x + 46,
            image_top + image_h // 2 - 26,
            image_w - 48,
            80,
            scale=0.50,
            color=MUTED,
        )

    if legend_items:
        legend_y = image_top + image_h + 29
        write_text(
            image,
            "Legend:",
            x + 28,
            legend_y + 1,
            scale=0.34,
            color=INK,
            thickness=1,
        )
        draw_swatch_legend(
            image,
            legend_items,
            x + 92,
            legend_y,
            image_w - 76,
            scale=0.34,
        )

    draw_text_box(
        image,
        caption,
        x + 22,
        y + h - caption_h + 16,
        w - 44,
        caption_h - 18,
        scale=0.48,
        color=MUTED,
    )


def placeholder_note(topic: str) -> str:
    return f"Run snapshot capture while {topic} is publishing."


def crop_map_content(source: np.ndarray, margin: int = 28) -> np.ndarray:
    """Crop large unknown-map borders while keeping enough context for reports."""
    if source is None or source.size == 0:
        return source

    gray_165 = np.array([165, 165, 165], dtype=np.int16)
    gray_170 = np.array([170, 170, 170], dtype=np.int16)
    source_i16 = source.astype(np.int16)
    diff_165 = np.max(np.abs(source_i16 - gray_165), axis=2)
    diff_170 = np.max(np.abs(source_i16 - gray_170), axis=2)
    mask = np.minimum(diff_165, diff_170) > 18
    if not np.any(mask):
        return source

    mask_u8 = mask.astype(np.uint8)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
    if num_labels > 1:
        areas = stats[1:, cv2.CC_STAT_AREA]
        largest_label = int(np.argmax(areas)) + 1
        largest_area = int(stats[largest_label, cv2.CC_STAT_AREA])
        total_area = int(np.count_nonzero(mask))
        if largest_area >= max(40, int(0.25 * total_area)):
            mask = labels == largest_label

    ys, xs = np.where(mask)
    y0 = max(0, int(ys.min()) - margin)
    y1 = min(source.shape[0], int(ys.max()) + margin + 1)
    x0 = max(0, int(xs.min()) - margin)
    x1 = min(source.shape[1], int(xs.max()) + margin + 1)
    if (y1 - y0) < 20 or (x1 - x0) < 20:
        return source
    return source[y0:y1, x0:x1]


def first_goal_image_name(robot_label: str, suffix: str) -> str:
    return f"first_goal_{robot_label}_{suffix}.png"


def find_first_goal_or_snapshot(results_dir: Path, robot_label: str, first_suffix: str, fallback_filename: str) -> Optional[np.ndarray]:
    """Prefer the saved first-goal frame, then fall back to the finalizer snapshot."""
    return find_first_existing_image(
        results_dir,
        [
            first_goal_image_name(robot_label, first_suffix),
            fallback_filename,
        ],
    )


def load_map_snapshot(results_dir: Path, filename: str) -> Optional[MapSnapshot]:
    raw_name = filename.replace(".png", ".npz")
    for base in (results_dir / "snapshots", results_dir / "report_inputs", results_dir):
        path = base / raw_name
        if not path.exists():
            continue
        try:
            data = np.load(str(path), allow_pickle=False)
            return {
                "grid": data["grid"].astype(np.int16),
                "resolution": float(data["resolution"]),
                "origin_x": float(data["origin_x"]),
                "origin_y": float(data["origin_y"]),
                "frame_id": str(data["frame_id"].item()),
                "topic": str(data["topic"].item()) if "topic" in data.files else filename,
            }
        except Exception:
            continue
    return None


def crop_grid_to_known_extent(grid: np.ndarray, padding: int = 10) -> np.ndarray:
    known = grid >= 0
    if not np.any(known):
        return grid

    #  Cropping choosing: use the full known envelope, not just walls, so free space stays visible.
    ys, xs = np.where(known)
    y0 = max(0, int(ys.min()) - padding)
    y1 = min(grid.shape[0], int(ys.max()) + padding + 1)
    x0 = max(0, int(xs.min()) - padding)
    x1 = min(grid.shape[1], int(xs.max()) + padding + 1)
    if (y1 - y0) < 8 or (x1 - x0) < 8:
        return grid
    return grid[y0:y1, x0:x1]


def render_occupancy_array(grid: np.ndarray, crop: bool = True, max_side_px: int = 900) -> np.ndarray:
    if grid is None or grid.size == 0:
        return grid
    if crop:
        grid = crop_grid_to_known_extent(grid)

    height, width = grid.shape
    display_grid = np.flipud(grid)
    image = np.full((height, width, 3), (170, 170, 170), dtype=np.uint8)
    unknown = display_grid < 0
    occupied = display_grid >= 80
    known_free = (~unknown) & (~occupied)

    image[unknown] = (165, 165, 165)
    image[occupied] = (18, 28, 28)
    if np.any(known_free):
        values = np.clip(display_grid[known_free], 0, 79).astype(np.float32)
        shade = (252 - values * 1.55).clip(130, 252).astype(np.uint8)
        image[known_free] = np.stack([shade, shade, shade], axis=1)

    scale = max(1, min(12, int(max_side_px / max(width, height, 1))))
    return cv2.resize(image, (width * scale, height * scale), interpolation=cv2.INTER_NEAREST)


def map_snapshot_or_image(results_dir: Path, filename: str) -> Optional[np.ndarray]:
    snapshot = load_map_snapshot(results_dir, filename)
    if snapshot is not None:
        return render_occupancy_array(snapshot["grid"], crop=True)
    return find_optional_image(results_dir, filename)


def paste_mask(canvas: np.ndarray, mask: np.ndarray, x0: int, y0: int) -> None:
    h, w = mask.shape
    y_start = max(0, y0)
    x_start = max(0, x0)
    y_end = min(canvas.shape[0], y0 + h)
    x_end = min(canvas.shape[1], x0 + w)
    if y_end <= y_start or x_end <= x_start:
        return
    src_y0 = y_start - y0
    src_x0 = x_start - x0
    canvas[y_start:y_end, x_start:x_end] |= mask[src_y0:src_y0 + (y_end - y_start), src_x0:src_x0 + (x_end - x_start)]


def contribution_legend_items(compact: bool = False) -> List[Tuple[str, Color]]:
    if compact:
        return [
            ("R1 known", (209, 234, 212)),
            ("R2 known", (238, 224, 198)),
            ("overlap", (230, 216, 235)),
            ("walls", (16, 25, 25)),
            ("A* path", (235, 102, 45)),
        ]

    return [
        ("robot 1 known", (209, 234, 212)),
        ("robot 2 known", (238, 224, 198)),
        ("overlap", (230, 216, 235)),
        ("occupied", (16, 25, 25)),
        ("A* path", (235, 102, 45)),
    ]


def draw_contribution_legend(image: np.ndarray, x: int, y: int, scale: float = 0.52) -> None:
    cursor_x = x
    for label, color in contribution_legend_items():
        cv2.rectangle(image, (cursor_x, y - 16), (cursor_x + 22, y + 6), color, -1)
        cv2.rectangle(image, (cursor_x, y - 16), (cursor_x + 22, y + 6), (120, 124, 130), 1)
        write_text(image, label, cursor_x + 32, y + 4, scale=scale, color=INK)
        cursor_x += max(168, text_width(label, scale) + 76)


def render_merged_contribution_image(
    results_dir: Path,
    waypoints: Optional[Sequence[Point]] = None,
    max_side_px: int = 1000,
    include_legend: bool = False,
) -> Optional[np.ndarray]:
    r1 = load_map_snapshot(results_dir, "map_robot1_slam.png")
    r2 = load_map_snapshot(results_dir, "map_robot2_slam.png")
    if r1 is None or r2 is None:
        return None

    res = float(r1["resolution"])
    if res <= 0 or abs(res - float(r2["resolution"])) > 1e-4:
        return None

    snapshots = [r1, r2]
    min_x = min(float(s["origin_x"]) for s in snapshots)
    min_y = min(float(s["origin_y"]) for s in snapshots)
    max_x = max(float(s["origin_x"]) + s["grid"].shape[1] * res for s in snapshots)
    max_y = max(float(s["origin_y"]) + s["grid"].shape[0] * res for s in snapshots)
    width = max(1, int(math.ceil((max_x - min_x) / res)) + 1)
    height = max(1, int(math.ceil((max_y - min_y) / res)) + 1)

    known_1 = np.zeros((height, width), dtype=bool)
    known_2 = np.zeros((height, width), dtype=bool)
    occ_1 = np.zeros((height, width), dtype=bool)
    occ_2 = np.zeros((height, width), dtype=bool)

    for target_known, target_occ, snapshot in (
        (known_1, occ_1, r1),
        (known_2, occ_2, r2),
    ):
        grid = snapshot["grid"]
        x0 = int(round((float(snapshot["origin_x"]) - min_x) / res))
        y0 = int(round((float(snapshot["origin_y"]) - min_y) / res))
        paste_mask(target_known, grid >= 0, x0, y0)
        paste_mask(target_occ, grid >= 80, x0, y0)

    union = known_1 | known_2 | occ_1 | occ_2
    if not np.any(union):
        return None

    ys, xs = np.where(union)
    pad = 8
    y0 = max(0, int(ys.min()) - pad)
    y1 = min(height, int(ys.max()) + pad + 1)
    x0 = max(0, int(xs.min()) - pad)
    x1 = min(width, int(xs.max()) + pad + 1)
    known_1 = known_1[y0:y1, x0:x1]
    known_2 = known_2[y0:y1, x0:x1]
    occ = (occ_1 | occ_2)[y0:y1, x0:x1]
    cropped_origin_x = min_x + x0 * res
    cropped_origin_y = min_y + y0 * res

    h, w = known_1.shape
    image = np.full((h, w, 3), (168, 168, 168), dtype=np.uint8)
    only_1 = known_1 & ~known_2
    only_2 = known_2 & ~known_1
    overlap = known_1 & known_2
    image[only_1] = (209, 234, 212)
    image[only_2] = (238, 224, 198)
    image[overlap] = (230, 216, 235)
    image[occ] = (16, 25, 25)

    display = np.flipud(image)
    scale = max(1, min(12, int(max_side_px / max(w, h, 1))))
    display = cv2.resize(display, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

    if waypoints:
        def to_pixel(point: Point) -> Tuple[int, int]:
            px = int(round((point[0] - cropped_origin_x) / res * scale))
            py_grid = int(round((point[1] - cropped_origin_y) / res))
            py = int((h - 1 - py_grid) * scale)
            return px, py

        pts = [to_pixel(pt) for pt in waypoints]
        for start, end in zip(pts, pts[1:]):
            cv2.line(display, start, end, (235, 102, 45), max(2, scale), cv2.LINE_AA)
        if pts:
            cv2.circle(display, pts[0], max(6, scale * 2), (78, 181, 88), -1, cv2.LINE_AA)
            cv2.circle(display, pts[-1], max(7, scale * 2), (37, 136, 245), -1, cv2.LINE_AA)

    if not include_legend:
        return display

    legend_h = 120
    out = np.full((display.shape[0] + legend_h, display.shape[1], 3), (248, 248, 246), dtype=np.uint8)
    paste(out, display, 0, legend_h)
    draw_contribution_legend(out, 18, 52, scale=0.62)
    return out


def build_merged_contribution_figure(
    results_dir: Path,
    summary: Dict[str, str],
    waypoints: Sequence[Point],
) -> Optional[Path]:
    merged = render_merged_contribution_image(results_dir, waypoints, max_side_px=1100)
    if merged is None:
        return None

    output = make_canvas(1400, 980)
    write_text(output, "Merged Map Contribution View", 44, 58, scale=1.05, thickness=2)
    draw_text_box(
        output,
        "This redraws the local occupancy grids in one coordinate canvas so the map merge is visible: robot 1 contribution, robot 2 contribution, overlap, walls, and the final A* path.",
        46,
        96,
        1180,
        68,
        scale=0.56,
        color=MUTED,
    )

    card(output, 70, 175, 1260, 720, "Merged occupancy evidence")
    draw_contribution_legend(output, 105, 255, scale=0.58)
    fitted = resize_to_fit(merged, 1120, 470)
    paste(output, fitted, 105 + (1120 - fitted.shape[1]) // 2, 315 + (470 - fitted.shape[0]) // 2)
    draw_text_box(
        output,
        f"Path source: {summary.get('map_source', '?')} | length: {summary.get('path_length_m', '?')} m | frame: {summary.get('path_frame', '?')}",
        105,
        850,
        1190,
        42,
        scale=0.52,
        color=MUTED,
    )

    path = results_dir / "report_visuals" / "report_merged_contribution_map.png"
    cv2.imwrite(str(path), output)
    return path


def parse_robot_id(summary: Dict[str, str]) -> str:
    value = summary.get("robot_id", "?")
    return f"robot {value}" if value != "?" else "robot ?"


def draw_metric(
    image: np.ndarray,
    x: int,
    y: int,
    label: str,
    value: str,
    accent: Color,
) -> None:
    cv2.circle(image, (x + 18, y + 18), 8, accent, -1, cv2.LINE_AA)
    write_text(image, label, x + 40, y + 22, scale=0.48, color=MUTED)
    write_text(image, value, x + 40, y + 56, scale=0.78, color=INK, thickness=2)


def build_evidence_panel(
    results_dir: Path,
    summary: Dict[str, str],
    waypoints: Sequence[Point],
    map_image: np.ndarray,
) -> Path:
    output = make_canvas(1500, 920)
    write_text(
        output,
        "Multi-Robot Exploration: Final A* Evidence",
        42,
        58,
        scale=1.12,
        color=INK,
        thickness=2,
    )
    wrapped_text(
        output,
        "Goal detection freezes exploration, stores the goal location, and publishes the shortest available start-to-goal path.",
        44,
        95,
        width_chars=105,
        scale=0.58,
        color=MUTED,
    )

    card(output, 40, 140, 920, 720, "Final map and path")
    fitted_map = resize_to_fit(map_image, 860, 620)
    paste(output, fitted_map, 70, 205)
    write_text(
        output,
        "Blue path = A* route, green = selected start, orange = detected goal",
        74,
        835,
        scale=0.56,
        color=MUTED,
    )

    card(output, 990, 140, 470, 250, "Key result")
    length = summary.get("path_length_m") or f"{cumulative_length(waypoints):.2f}"
    draw_metric(output, 1022, 208, "Path length", f"{length} m", BLUE)
    draw_metric(output, 1022, 304, "Waypoints", summary.get("waypoints", str(len(waypoints))), GREEN)
    draw_metric(output, 1232, 208, "Chosen start", parse_robot_id(summary), ORANGE)
    draw_metric(output, 1232, 304, "Source map", summary.get("map_source", "?"), RED)

    card(output, 990, 420, 470, 210, "What this proves")
    draw_text_box(
        output,
        "The robots explore with occupancy-grid mapping. The first reliable goal observation becomes the stored target, so both robots stop searching and the coordinator returns the shortest A* path from the best start.",
        1020,
        480,
        415,
        120,
        scale=0.56,
        color=INK,
    )

    card(output, 990, 660, 470, 210, "Report caption")
    wrapped_text(
        output,
        f"Generated after /mission_complete from coordinator files. Frame: {summary.get('path_frame', '?')}; path: {summary.get('path_kind', '?')}.",
        1020,
        720,
        width_chars=38,
        scale=0.52,
        color=INK,
    )

    path = results_dir / "report_visuals" / "report_demo_evidence_panel.png"
    cv2.imwrite(str(path), output)
    return path


def plot_bounds(points: Sequence[Point]) -> Tuple[float, float, float, float]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span = max(max_x - min_x, max_y - min_y, 1.0)
    pad = max(0.45, span * 0.20)
    return min_x - pad, max_x + pad, min_y - pad, max_y + pad


def build_waypoint_trace(
    results_dir: Path,
    summary: Dict[str, str],
    waypoints: Sequence[Point],
) -> Path:
    output = make_canvas(1200, 840)
    write_text(output, "Shortest Start-to-Goal Path Trace", 44, 58, scale=1.05, thickness=2)
    wrapped_text(
        output,
        "This plot uses the same CSV waypoints published in RViz, shown in metric odom coordinates for report readability.",
        46,
        94,
        width_chars=100,
        scale=0.56,
        color=MUTED,
    )

    plot_x, plot_y, plot_w, plot_h = 90, 150, 1020, 600
    cv2.rectangle(output, (plot_x, plot_y), (plot_x + plot_w, plot_y + plot_h), CARD, -1)
    cv2.rectangle(output, (plot_x, plot_y), (plot_x + plot_w, plot_y + plot_h), LINE, 2)

    min_x, max_x, min_y, max_y = plot_bounds(waypoints)

    def to_pixel(point: Point) -> Tuple[int, int]:
        x, y = point
        px = plot_x + int((x - min_x) / (max_x - min_x) * plot_w)
        py = plot_y + plot_h - int((y - min_y) / (max_y - min_y) * plot_h)
        return px, py

    for i in range(6):
        gx = plot_x + int(i / 5 * plot_w)
        gy = plot_y + int(i / 5 * plot_h)
        cv2.line(output, (gx, plot_y), (gx, plot_y + plot_h), (232, 234, 237), 1)
        cv2.line(output, (plot_x, gy), (plot_x + plot_w, gy), (232, 234, 237), 1)

    pixels = [to_pixel(point) for point in waypoints]
    for start, end in zip(pixels, pixels[1:]):
        cv2.line(output, start, end, BLUE, 5, cv2.LINE_AA)
    for index, pixel in enumerate(pixels):
        radius = 5 if index % 4 == 0 else 3
        cv2.circle(output, pixel, radius, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(output, pixel, radius, BLUE, 1, cv2.LINE_AA)

    start_px = pixels[0]
    goal_px = pixels[-1]
    cv2.circle(output, start_px, 15, GREEN, -1, cv2.LINE_AA)
    cv2.circle(output, goal_px, 17, ORANGE, -1, cv2.LINE_AA)
    write_text(output, "start", start_px[0] + 16, start_px[1] - 12, scale=0.54, color=GREEN, thickness=2)
    write_text(output, "goal", goal_px[0] + 16, goal_px[1] - 12, scale=0.54, color=ORANGE, thickness=2)

    y = 790
    write_text(
        output,
        f"{summary.get('path_length_m', f'{cumulative_length(waypoints):.2f}')} m | {len(waypoints)} waypoints | {summary.get('path_frame', '?')}",
        92,
        y,
        scale=0.65,
        color=INK,
        thickness=2,
    )

    path = results_dir / "report_visuals" / "report_waypoint_trace.png"
    cv2.imwrite(str(path), output)
    return path


def build_cv_detection_evidence(results_dir: Path) -> Path:
    output = make_canvas(1700, 1060)
    write_text(output, "Computer Vision Detection Evidence", 44, 58, scale=1.05, thickness=2)
    draw_text_box(
        output,
        "These panels come from the live debug-image topics. They show the camera view and segmentation overlays used to publish goal and heuristic PointStamped detections.",
        46,
        96,
        1280,
        70,
        scale=0.56,
        color=MUTED,
    )
    draw_swatch_legend(
        output,
        [
            ("blue box = selected target", (255, 0, 0)),
            ("yellow box = other visible detections", (0, 210, 255)),
            ("red dot = centroid", (0, 0, 255)),
            ("green mask = selected segment", (0, 180, 80)),
        ],
        46,
        158,
        1500,
        scale=0.46,
    )

    missing = "Run capture_report_snapshots.py while the integrated demo is active."
    tiles = [
        (
            "Robot 1 raw camera",
            find_first_goal_or_snapshot(results_dir, "robot1", "camera_raw", "cv_robot1_camera_raw.png"),
            "/robot1/camera/image_raw",
            "Raw camera frame from the first stored goal sighting when available.",
            GREEN,
            "Run the integrated demo until the goal detector saves first-goal evidence.",
        ),
        (
            "Robot 1 goal segmentation",
            find_first_goal_or_snapshot(results_dir, "robot1", "goal_debug", "cv_robot1_goal_debug.png"),
            "/robot1/goal_debug_image",
            "Goal detector overlay from the first stored goal sighting when available.",
            ORANGE,
            placeholder_note("/robot1/goal_debug_image"),
        ),
        (
            "Robot 1 heuristic segmentation",
            find_optional_image(results_dir, "cv_robot1_heuristic_debug.png"),
            "/robot1/heuristic_debug_image",
            "Heuristic clue overlay for bottle detections before the goal is locked.",
            BLUE,
            placeholder_note("/robot1/heuristic_debug_image"),
        ),
        (
            "Robot 2 raw camera",
            find_first_goal_or_snapshot(results_dir, "robot2", "camera_raw", "cv_robot2_camera_raw.png"),
            "/robot2/camera/image_raw",
            "Second robot camera stream, preferring first-goal evidence when robot 2 saw it.",
            GREEN,
            "Run the integrated demo until the goal detector saves first-goal evidence.",
        ),
        (
            "Robot 2 goal segmentation",
            find_first_goal_or_snapshot(results_dir, "robot2", "goal_debug", "cv_robot2_goal_debug.png"),
            "/robot2/goal_debug_image",
            "Second robot goal detector overlay, preferring first-goal evidence.",
            ORANGE,
            placeholder_note("/robot2/goal_debug_image"),
        ),
        (
            "Robot 2 heuristic segmentation",
            find_optional_image(results_dir, "cv_robot2_heuristic_debug.png"),
            "/robot2/heuristic_debug_image",
            "Second robot heuristic detector overlay.",
            BLUE,
            placeholder_note("/robot2/heuristic_debug_image"),
        ),
    ]
    x_positions = [44, 600, 1156]
    y_positions = [190, 620]
    for index, (title, source_image, topic, caption, accent, missing_note) in enumerate(tiles):
        x = x_positions[index % 3]
        y = y_positions[index // 3]
        tile_image(
            output,
            source_image,
            x,
            y,
            500,
            370,
            title,
            f"{topic} | {caption}",
            accent,
            missing_note or missing,
        )

    path = results_dir / "report_visuals" / "report_cv_detection_evidence.png"
    cv2.imwrite(str(path), output)
    return path


def build_map_progression(
    results_dir: Path,
    summary: Dict[str, str],
    map_image: np.ndarray,
    waypoints: Sequence[Point],
) -> Path:
    output = make_canvas(1700, 1210)
    write_text(output, "Mapping And Final Path Progression", 44, 58, scale=1.05, thickness=2)
    draw_text_box(
        output,
        "This figure shows the evidence chain from individual robot occupancy grids, to the merged/global map when available, to the final A* path returned for the closest start.",
        46,
        96,
        1280,
        70,
        scale=0.56,
        color=MUTED,
    )

    missing = "Run snapshot capture while this map topic is publishing."
    merged_contribution = render_merged_contribution_image(results_dir, waypoints)
    if merged_contribution is None:
        merged_contribution = map_snapshot_or_image(results_dir, "map_merged.png")
    tile_image(
        output,
        map_snapshot_or_image(results_dir, "map_robot1_slam.png"),
        44,
        185,
        780,
        450,
        "Robot 1 local map",
        "/SLAM_map_1 | Occupancy belief grid created from robot 1 LiDAR.",
        GREEN,
        placeholder_note("/SLAM_map_1"),
        crop_source=True,
    )
    tile_image(
        output,
        map_snapshot_or_image(results_dir, "map_robot2_slam.png"),
        876,
        185,
        780,
        450,
        "Robot 2 local map",
        "/SLAM_map_2 | Independent occupancy belief grid from robot 2.",
        GREEN,
        placeholder_note("/SLAM_map_2"),
        crop_source=True,
    )
    tile_image(
        output,
        merged_contribution,
        44,
        695,
        780,
        450,
        "Merged map",
        "/merged_map | colored contribution view: robot 1, robot 2, overlap, walls, and A* path.",
        RED,
        missing,
        crop_source=False,
        legend_items=contribution_legend_items(compact=True),
    )
    tile_image(
        output,
        map_image,
        876,
        695,
        780,
        450,
        "Final A* answer",
        (
            f"{summary.get('map_source', '?')} | shortest path from "
            f"{parse_robot_id(summary)} start to the detected goal."
        ),
        BLUE,
        "Final path map was not saved.",
        crop_source=True,
    )

    path = results_dir / "report_visuals" / "report_map_progression.png"
    cv2.imwrite(str(path), output)
    return path


def arrow(image: np.ndarray, start: Tuple[int, int], end: Tuple[int, int]) -> None:
    cv2.arrowedLine(image, start, end, DARK, 3, cv2.LINE_AA, tipLength=0.04)


def flow_box(
    image: np.ndarray,
    x: int,
    y: int,
    w: int,
    h: int,
    title: str,
    body: str,
    accent: Color,
) -> None:
    card(image, x, y, w, h)
    cv2.rectangle(image, (x, y), (x + w, y + 10), accent, -1)
    write_text(image, title, x + 22, y + 42, scale=0.66, color=INK, thickness=2)
    draw_text_box(image, body, x + 22, y + 82, w - 44, h - 100, scale=0.48, color=MUTED)


def build_system_flow(results_dir: Path) -> Path:
    output = make_canvas(1500, 900)
    write_text(output, "Integrated System Flow", 44, 58, scale=1.08, thickness=2)
    wrapped_text(
        output,
        "The demo is intentionally simple after the goal is found: store the goal, stop exploration, compute A*, publish evidence.",
        46,
        96,
        width_chars=110,
        scale=0.56,
        color=MUTED,
    )

    y1, y2 = 160, 500
    flow_box(output, 70, y1, 260, 190, "Robot 1", "camera, LiDAR, odom, local controller", GREEN)
    flow_box(output, 70, y2, 260, 190, "Robot 2", "camera, LiDAR, odom, local controller", GREEN)
    flow_box(output, 405, 190, 280, 220, "Per-Robot Mapping", "occupancy belief grid and frontier exploration", BLUE)
    flow_box(output, 405, 470, 280, 220, "Vision Pipeline", "heuristic bottle clue and goal sphere detection", ORANGE)
    flow_box(output, 770, 300, 290, 220, "Map Merger", "accepts /SLAM_map_<id>, publishes /merged_map when alignment is confident", RED)
    flow_box(output, 1130, 300, 300, 220, "Coordinator", "prioritizes goal over heuristic, computes shortest A* path, publishes mission complete", DARK)

    arrow(output, (330, y1 + 95), (405, 250))
    arrow(output, (330, y2 + 95), (405, 580))
    arrow(output, (685, 300), (770, 365))
    arrow(output, (685, 580), (770, 455))
    arrow(output, (1060, 410), (1130, 410))

    cv2.line(output, (1280, 520), (1280, 625), DARK, 3, cv2.LINE_AA)
    arrow(output, (1280, 625), (1280, 675))
    flow_box(output, 1080, 660, 400, 150, "Final Evidence", "RViz markers, final path topics, PNG/SVG/CSV/summary files", BLUE)

    path = results_dir / "report_visuals" / "report_system_flow.png"
    cv2.imwrite(str(path), output)
    return path


def build_topic_flow(results_dir: Path) -> Path:
    output = make_canvas(1500, 900)
    write_text(output, "ROS Topic Evidence Map", 44, 58, scale=1.08, thickness=2)
    wrapped_text(
        output,
        "These are the topics to show during the demo or cite in the report when explaining how information moves through the system.",
        46,
        96,
        width_chars=110,
        scale=0.56,
        color=MUTED,
    )

    flow_box(output, 70, 180, 330, 180, "Robot State", "/pose_<id>\n/robot<id>/odom\n/robot<id>/scan", GREEN)
    flow_box(output, 70, 460, 330, 180, "Computer Vision", "/robot<id>/goal_point_odom\n/robot<id>/heuristic_point_odom\n/robot<id>/goal_debug_image", ORANGE)
    flow_box(output, 500, 180, 330, 180, "Mapping", "/SLAM_map_1\n/SLAM_map_2\n/merged_map", BLUE)
    flow_box(output, 500, 460, 330, 180, "Coordination", "/merge_status\n/nav_path_<id>\n/robot<id>/cmd_vel", RED)
    flow_box(output, 930, 300, 420, 220, "Final Answer", "/final_start_to_goal_path\n/final_start_to_goal_nav_path\n/final_result_markers\n/mission_complete", DARK)

    arrow(output, (400, 270), (500, 270))
    arrow(output, (400, 550), (500, 550))
    arrow(output, (830, 270), (930, 365))
    arrow(output, (830, 550), (930, 455))

    card(output, 930, 570, 420, 190, "Convention")
    wrapped_text(
        output,
        "New map publishers use /SLAM_map_<id>. The merger keeps /robot<id>/SLAM_map only as an alias for older branches.",
        960,
        630,
        width_chars=35,
        scale=0.50,
        color=INK,
    )

    path = results_dir / "report_visuals" / "report_topic_flow.png"
    cv2.imwrite(str(path), output)
    return path


def build_behavior_timeline(results_dir: Path) -> Path:
    output = make_canvas(1800, 650)
    write_text(output, "Demo Behavior Timeline", 44, 58, scale=1.08, thickness=2)
    draw_text_box(
        output,
        "Mission sequence after exploration begins, visual evidence is accepted, and the final shortest path is returned.",
        46,
        96,
        1300,
        70,
        scale=0.56,
        color=MUTED,
    )

    steps = [
        ("1", "Explore", "frontier goals fill local occupancy grids", GREEN),
        ("2", "Use Clues", "bottle detections bias search while the goal is unknown", ORANGE),
        ("3", "Goal Seen", "goal sphere position is stored from CV + LiDAR", BLUE),
        ("4", "Freeze Motion", "heuristics are ignored and cmd_vel is zeroed", RED),
        ("5", "Return Answer", "A* chooses the shortest start-to-goal path", DARK),
    ]
    start_x, y = 70, 250
    step_w, gap = 300, 35
    for index, (num, title, body, accent) in enumerate(steps):
        x = start_x + index * (step_w + gap)
        card(output, x, y, step_w, 235)
        cv2.circle(output, (x + 45, y + 55), 28, accent, -1, cv2.LINE_AA)
        write_text(output, num, x + 35, y + 65, scale=0.68, color=(255, 255, 255), thickness=2)
        draw_text_box(
            output,
            title,
            x + 82,
            y + 62,
            step_w - 110,
            44,
            scale=0.58,
            color=INK,
            thickness=2,
            min_scale=0.40,
        )
        draw_text_box(
            output,
            body,
            x + 26,
            y + 122,
            step_w - 52,
            92,
            scale=0.46,
            color=MUTED,
            min_scale=0.34,
        )
        if index < len(steps) - 1:
            arrow(output, (x + step_w, y + 110), (x + step_w + gap, y + 110))

    path = results_dir / "report_visuals" / "report_behavior_timeline.png"
    cv2.imwrite(str(path), output)
    return path


def build_visuals(results_dir: Path) -> List[Path]:
    summary_path = results_dir / "final_start_to_goal_summary.txt"
    csv_path = results_dir / "final_start_to_goal_path.csv"
    map_path = results_dir / "final_start_to_goal_map.png"
    missing = [path for path in (summary_path, csv_path, map_path) if not path.exists()]
    if missing:
        joined = ", ".join(str(path) for path in missing)
        raise SystemExit(f"Missing final path artifact(s): {joined}")

    output_dir = results_dir / "report_visuals"
    output_dir.mkdir(parents=True, exist_ok=True)

    summary = read_summary(summary_path)
    waypoints = read_waypoints(csv_path)
    if len(waypoints) < 2:
        raise SystemExit("The final path CSV must contain at least two waypoints.")

    map_image = cv2.imread(str(map_path), cv2.IMREAD_COLOR)
    if map_image is None:
        raise SystemExit(f"Could not read map image: {map_path}")

    outputs = [
        build_evidence_panel(results_dir, summary, waypoints, map_image),
        build_map_progression(results_dir, summary, map_image, waypoints),
        build_cv_detection_evidence(results_dir),
        build_waypoint_trace(results_dir, summary, waypoints),
        build_system_flow(results_dir),
        build_topic_flow(results_dir),
        build_behavior_timeline(results_dir),
    ]
    merged_contribution = build_merged_contribution_figure(results_dir, summary, waypoints)
    if merged_contribution is not None:
        outputs.insert(2, merged_contribution)
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate report-ready PNG diagrams from final_start_to_goal artifacts."
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=default_results_dir(),
        help="Directory containing final_start_to_goal_map.png/csv/summary.",
    )
    args = parser.parse_args()

    outputs = build_visuals(args.results_dir.resolve())
    print("Report visuals generated:")
    for output in outputs:
        print(f"  {output}")


if __name__ == "__main__":
    main()
