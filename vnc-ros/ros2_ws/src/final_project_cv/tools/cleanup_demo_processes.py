#!/usr/bin/env python3
"""Clean up stale integrated-demo processes without matching the cleanup shell."""

from __future__ import annotations

import argparse
import os
import signal
import time
from pathlib import Path


PATTERNS = [
    "/opt/ros/humble/bin/ros2 launch final_project_cv integrated_two_robot_demo.launch.py",
    "gazebo --verbose",
    "gzserver --verbose",
    "gzclient --verbose",
    "/root/ros2_ws/src/mapper/mapper.py",
    "/root/ros2_ws/src/mapper/coordinator.py",
    "/root/ros2_ws/install/merger/lib/merger/map_merger_node",
    "/root/ros2_ws/install/final_project_cv/lib/final_project_cv/vision_target_detector",
    "/root/ros2_ws/install/final_project_cv/lib/final_project_cv/target_localizer",
    "/opt/ros/humble/lib/tf2_ros/static_transform_publisher",
]


def process_cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return ""
    return raw.replace(b"\x00", b" ").decode("utf-8", errors="ignore")


def parent_pid(pid: int) -> int | None:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except OSError:
        return None
    parts = stat.split()
    if len(parts) < 4:
        return None
    try:
        return int(parts[3])
    except ValueError:
        return None


def ancestor_pids(pid: int) -> set[int]:
    ancestors = set()
    current = parent_pid(pid)
    while current and current > 1 and current not in ancestors:
        ancestors.add(current)
        current = parent_pid(current)
    return ancestors


def main() -> None:
    parser = argparse.ArgumentParser(description="Kill stale integrated demo processes.")
    parser.add_argument("--delay", type=float, default=0.0)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    if args.delay > 0:
        time.sleep(args.delay)

    own_pid = os.getpid()
    protected_pids = {own_pid} | ancestor_pids(own_pid)
    killed = []

    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid in protected_pids:
            continue
        cmdline = process_cmdline(pid)
        if not cmdline:
            continue
        if any(pattern in cmdline for pattern in PATTERNS):
            try:
                os.kill(pid, signal.SIGKILL)
                killed.append((pid, cmdline))
            except OSError:
                pass

    if not args.quiet:
        for pid, cmdline in killed:
            print(f"killed stale demo process {pid}: {cmdline}")
        print(f"cleanup complete: {len(killed)} process(es) killed")


if __name__ == "__main__":
    main()
