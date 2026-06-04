#!/usr/bin/env python3
"""Capture live ROS topic snapshots for the report visual pack."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


IMAGE_TOPICS: Dict[str, Tuple[str, str]] = {
    "cv_robot1_goal_debug.png": ("/robot1/goal_debug_image", "Robot 1 goal segmentation"),
    "cv_robot1_heuristic_debug.png": ("/robot1/heuristic_debug_image", "Robot 1 heuristic segmentation"),
    "cv_robot2_goal_debug.png": ("/robot2/goal_debug_image", "Robot 2 goal segmentation"),
    "cv_robot2_heuristic_debug.png": ("/robot2/heuristic_debug_image", "Robot 2 heuristic segmentation"),
    "cv_robot1_camera_raw.png": ("/robot1/camera/image_raw", "Robot 1 raw camera"),
    "cv_robot2_camera_raw.png": ("/robot2/camera/image_raw", "Robot 2 raw camera"),
}

MAP_TOPICS: Dict[str, Tuple[str, str]] = {
    "map_robot1_slam.png": ("/SLAM_map_1", "Robot 1 local occupancy grid"),
    "map_robot2_slam.png": ("/SLAM_map_2", "Robot 2 local occupancy grid"),
    "map_merged.png": ("/merged_map", "Merged occupancy grid"),
}


def default_results_dir() -> Path:
    container_results = Path("/root/ros2_ws/src/final_path_results")
    if container_results.exists():
        return container_results
    return Path(__file__).resolve().parents[2] / "final_path_results"


def render_occupancy_grid(msg: OccupancyGrid) -> np.ndarray:
    """Convert nav_msgs/OccupancyGrid into a report-readable BGR image."""
    width = msg.info.width
    height = msg.info.height
    grid = np.array(msg.data, dtype=np.int16).reshape((height, width))
    display_grid = np.flipud(grid)
    image = np.full((height, width, 3), (170, 170, 170), dtype=np.uint8)

    unknown = display_grid < 0
    occupied = display_grid >= 80
    known_free = (~unknown) & (~occupied)

    image[unknown] = (165, 165, 165)
    image[occupied] = (20, 30, 30)
    if np.any(known_free):
        values = np.clip(display_grid[known_free], 0, 79).astype(np.float32)
        shade = (252 - values * 1.55).clip(130, 252).astype(np.uint8)
        image[known_free, 0] = shade
        image[known_free, 1] = shade
        image[known_free, 2] = shade

    scale = max(2, min(10, int(1000 / max(width, height, 1))))
    return cv2.resize(image, (width * scale, height * scale), interpolation=cv2.INTER_NEAREST)


class SnapshotNode(Node):
    def __init__(self, output_dir: Path):
        super().__init__("report_snapshot_capture")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.saved: Dict[str, Dict[str, str]] = {}

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscriptions_holder = []
        for filename, (topic, label) in IMAGE_TOPICS.items():
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, filename=filename, topic=topic, label=label: self.image_callback(msg, filename, topic, label),
                image_qos,
            )
            self.subscriptions_holder.append(sub)

        for filename, (topic, label) in MAP_TOPICS.items():
            sub = self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, filename=filename, topic=topic, label=label: self.map_callback(msg, filename, topic, label),
                map_qos,
            )
            self.subscriptions_holder.append(sub)

    def already_saved(self, filename: str) -> bool:
        return filename in self.saved

    def image_callback(self, msg: Image, filename: str, topic: str, label: str) -> None:
        if self.already_saved(filename):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            frame = self.bridge.imgmsg_to_cv2(msg)
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        path = self.output_dir / filename
        cv2.imwrite(str(path), frame)
        self.saved[filename] = {"topic": topic, "label": label, "path": str(path)}
        self.get_logger().info(f"saved {label}: {path}")

    def map_callback(self, msg: OccupancyGrid, filename: str, topic: str, label: str) -> None:
        if self.already_saved(filename):
            return
        image = render_occupancy_grid(msg)
        path = self.output_dir / filename
        cv2.imwrite(str(path), image)
        self.saved[filename] = {
            "topic": topic,
            "label": label,
            "path": str(path),
            "frame_id": msg.header.frame_id,
            "resolution": f"{msg.info.resolution:.4f}",
            "width": str(msg.info.width),
            "height": str(msg.info.height),
        }
        self.get_logger().info(f"saved {label}: {path}")

    def write_manifest(self) -> None:
        manifest_path = self.output_dir / "snapshot_manifest.json"
        manifest_path.write_text(json.dumps(self.saved, indent=2, sort_keys=True), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture report PNG snapshots from live ROS topics.")
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=default_results_dir(),
        help="Directory containing final path artifacts; snapshots are written below it.",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=12.0,
        help="How long to wait for image and map messages.",
    )
    args = parser.parse_args()

    snapshot_dir = args.results_dir.resolve() / "snapshots"
    rclpy.init()
    node = SnapshotNode(snapshot_dir)
    deadline = time.monotonic() + max(1.0, args.seconds)
    expected_count = len(IMAGE_TOPICS) + len(MAP_TOPICS)

    try:
        while rclpy.ok() and time.monotonic() < deadline and len(node.saved) < expected_count:
            rclpy.spin_once(node, timeout_sec=0.15)
    finally:
        node.write_manifest()
        saved_count = len(node.saved)
        print(f"Saved {saved_count}/{expected_count} snapshots in {snapshot_dir}")
        if saved_count < expected_count:
            expected = set(IMAGE_TOPICS.keys()) | set(MAP_TOPICS.keys())
            missing = sorted(expected - set(node.saved.keys()))
            print("Missing snapshots:")
            for filename in missing:
                topic = IMAGE_TOPICS.get(filename, MAP_TOPICS.get(filename))[0]
                print(f"  {filename} from {topic}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
