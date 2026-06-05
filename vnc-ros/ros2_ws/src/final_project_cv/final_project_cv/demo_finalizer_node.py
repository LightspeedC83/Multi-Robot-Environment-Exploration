#!/usr/bin/env python3
"""Finalize the integrated demo after the coordinator publishes its answer."""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import Bool


DEFAULT_RESULTS_DIR = "/root/ros2_ws/src/final_path_results"
DEFAULT_TOOLS_DIR = "/root/ros2_ws/src/final_project_cv/tools"


class DemoFinalizer(Node):
    def __init__(self) -> None:
        super().__init__("demo_finalizer")

        self.declare_parameter("results_dir", DEFAULT_RESULTS_DIR)
        self.declare_parameter("tools_dir", DEFAULT_TOOLS_DIR)
        self.declare_parameter("snapshot_seconds", 5.0)
        self.declare_parameter("shutdown_on_complete", True)
        self.declare_parameter("shutdown_delay_sec", 1.5)

        self.results_dir = Path(str(self.get_parameter("results_dir").value))
        self.tools_dir = Path(str(self.get_parameter("tools_dir").value))
        self.snapshot_seconds = float(self.get_parameter("snapshot_seconds").value)
        self.shutdown_on_complete = bool(self.get_parameter("shutdown_on_complete").value)
        self.shutdown_delay_sec = float(self.get_parameter("shutdown_delay_sec").value)
        self.finalized = False

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription_holder = [
            self.create_subscription(
                Bool,
                "/mission_complete",
                self.mission_complete_callback,
                latched_qos,
            ),
            self.create_subscription(
                PoseArray,
                "/final_start_to_goal_path",
                self.final_path_callback,
                latched_qos,
            ),
            self.create_subscription(
                NavPath,
                "/final_start_to_goal_nav_path",
                self.final_nav_path_callback,
                latched_qos,
            ),
        ]
        self.get_logger().info(
            "demo finalizer ready; waiting for /mission_complete or final start-to-goal path"
        )

    def mission_complete_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.start_finalization_once("/mission_complete")

    def final_path_callback(self, msg: PoseArray) -> None:
        if msg.poses:
            self.start_finalization_once("/final_start_to_goal_path")

    def final_nav_path_callback(self, msg: NavPath) -> None:
        if msg.poses:
            self.start_finalization_once("/final_start_to_goal_nav_path")

    def start_finalization_once(self, reason: str) -> None:
        if self.finalized:
            return
        self.finalized = True
        self.get_logger().info(f"demo finalizer triggered by {reason}")
        worker = threading.Thread(target=self.finalize_demo, daemon=True)
        worker.start()

    def run_helper(self, script_name: str, args: list[str], timeout_sec: float) -> bool:
        script_path = self.tools_dir / script_name
        if not script_path.exists():
            self.get_logger().warn(f"finalizer helper missing: {script_path}")
            return False

        command = [sys.executable, str(script_path), *args]
        self.get_logger().info(f"running {' '.join(command)}")
        result = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_sec,
            check=False,
        )
        for line in result.stdout.splitlines():
            self.get_logger().info(line)
        if result.returncode != 0:
            self.get_logger().warn(f"{script_name} exited with code {result.returncode}")
            return False
        return True

    def finalize_demo(self) -> None:
        #  result gathering: leave ROS alive briefly so maps and camera panels can still be sampled.
        time.sleep(0.8)
        self.results_dir.mkdir(parents=True, exist_ok=True)

        capture_ok = self.run_helper(
            "capture_report_snapshots.py",
            [
                "--results-dir",
                str(self.results_dir),
                "--seconds",
                f"{self.snapshot_seconds:.1f}",
            ],
            timeout_sec=max(8.0, self.snapshot_seconds + 8.0),
        )
        visuals_ok = self.run_helper(
            "generate_report_visuals.py",
            ["--results-dir", str(self.results_dir)],
            timeout_sec=20.0,
        )

        self.get_logger().info("RESULTS READY")
        self.get_logger().info(f"final summary: {self.results_dir / 'final_start_to_goal_summary.txt'}")
        self.get_logger().info(f"final map: {self.results_dir / 'final_start_to_goal_map.png'}")
        self.get_logger().info(f"report visuals: {self.results_dir / 'report_visuals'}")
        self.get_logger().info(f"snapshots: {self.results_dir / 'snapshots'}")
        if not capture_ok or not visuals_ok:
            self.get_logger().warn("results were finalized with at least one missing helper output")

        if self.shutdown_on_complete:
            time.sleep(max(0.0, self.shutdown_delay_sec))
            self.get_logger().info("mission complete; requesting integrated demo launch shutdown")
            self.schedule_delayed_cleanup()
            os._exit(0)

    def schedule_delayed_cleanup(self) -> None:
        """Leave a small cleanup pass behind in case Gazebo ignores launch shutdown."""
        cleanup_script = self.tools_dir / "cleanup_demo_processes.py"
        if cleanup_script.exists():
            subprocess.Popen(
                [sys.executable, str(cleanup_script), "--delay", "4.0", "--quiet"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        else:
            command = "sleep 4; pkill -9 -f '[g]azebo' || true; pkill -9 -f '[g]zserver' || true; pkill -9 -f '[g]zclient' || true"
            subprocess.Popen(["bash", "-lc", command], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DemoFinalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
