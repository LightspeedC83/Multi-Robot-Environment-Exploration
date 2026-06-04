#!/usr/bin/env python3
"""Create report-ready visual evidence from the final path artifacts."""

from __future__ import annotations

import argparse
import csv
import math
import textwrap
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

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
    return y + int(28 * scale) + 8


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
    for paragraph in text.splitlines() or [""]:
        if not paragraph.strip():
            y += int(24 * scale)
            continue
        for line in textwrap.wrap(paragraph, width=width_chars) or [""]:
            y = write_text(image, line, x, y, scale=scale, color=color)
            y += line_gap
    return y


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
    wrapped_text(
        output,
        "The robots explore with occupancy-grid mapping. The first reliable goal observation becomes the stored target, so both robots stop searching and the coordinator returns the shortest A* path from the best start.",
        1020,
        480,
        width_chars=45,
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
    wrapped_text(image, body, x + 22, y + 82, width_chars=max(24, w // 10), scale=0.48, color=MUTED)


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
    output = make_canvas(1500, 620)
    write_text(output, "Demo Behavior Timeline", 44, 58, scale=1.08, thickness=2)
    wrapped_text(
        output,
        "Use this as the one-slide explanation of why the robots may stop before both physically touch the goal.",
        46,
        96,
        width_chars=105,
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
    step_w, gap = 250, 35
    for index, (num, title, body, accent) in enumerate(steps):
        x = start_x + index * (step_w + gap)
        card(output, x, y, step_w, 220)
        cv2.circle(output, (x + 45, y + 55), 28, accent, -1, cv2.LINE_AA)
        write_text(output, num, x + 35, y + 65, scale=0.68, color=(255, 255, 255), thickness=2)
        write_text(output, title, x + 82, y + 62, scale=0.66, color=INK, thickness=2)
        wrapped_text(output, body, x + 26, y + 112, width_chars=25, scale=0.50, color=MUTED)
        if index < len(steps) - 1:
            arrow(output, (x + step_w, y + 110), (x + step_w + gap, y + 110))

    path = results_dir / "report_visuals" / "report_behavior_timeline.png"
    cv2.imwrite(str(path), output)
    return path


def write_index(results_dir: Path, outputs: Sequence[Path], summary: Dict[str, str]) -> Path:
    index_path = results_dir / "report_visuals" / "report_visual_index.md"
    lines = [
        "# Report Visuals",
        "",
        "Generated from the final path artifacts produced by the coordinator.",
        "",
        "## Final Result",
        "",
        f"- Path length: {summary.get('path_length_m', '?')} m",
        f"- Waypoints: {summary.get('waypoints', '?')}",
        f"- Map source: {summary.get('map_source', '?')}",
        f"- Path frame: {summary.get('path_frame', '?')}",
        f"- Path kind: {summary.get('path_kind', '?')}",
        "",
        "## Files",
        "",
    ]
    for output in outputs:
        lines.append(f"- `{output.name}`")
    lines.extend(
        [
            "",
            "Suggested report usage:",
            "",
            "- `report_demo_evidence_panel.png`: main result figure.",
            "- `report_waypoint_trace.png`: clean A* waypoint/path figure.",
            "- `report_system_flow.png`: architecture diagram.",
            "- `report_topic_flow.png`: ROS evidence/topic diagram.",
            "- `report_behavior_timeline.png`: short demo behavior explanation.",
            "",
        ]
    )
    index_path.write_text("\n".join(lines), encoding="utf-8")
    return index_path


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
        build_waypoint_trace(results_dir, summary, waypoints),
        build_system_flow(results_dir),
        build_topic_flow(results_dir),
        build_behavior_timeline(results_dir),
    ]
    outputs.append(write_index(results_dir, outputs, summary))
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
