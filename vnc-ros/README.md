# ROS 2 Docker Workspace

This folder contains the Docker Compose environment used for the multi-robot exploration demo. It runs ROS 2 Humble, Gazebo, RViz, and a browser-accessible NoVNC desktop so the project can be developed from macOS or Windows without a native Linux install.

## Prerequisites

- Docker Desktop
- A browser for NoVNC
- This repository checked out locally

## Start The Container

From the host machine:

```bash
cd /Users/emiliodaza/Dartmouth/Robotics/integrated-multi-robot/vnc-ros
docker compose build
docker compose up -d
docker compose exec ros bash
```

Open the desktop:

```text
http://localhost:8080/vnc.html
```

## Build The Workspace

Inside the ROS container:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The host folder `vnc-ros/ros2_ws/src` is mounted into the container at `/root/ros2_ws/src`, so local edits are visible inside Docker.

## Topic Convention

Coordination topics use flat IDs:

```text
/new_robot_id
/SLAM_map_<id>
/pose_<id>
/id_active_<id>
/nav_path_<id>
```

Robot hardware, sensor, and CV topics use robot namespaces:

```text
/robot<id>/cmd_vel
/robot<id>/scan
/robot<id>/odom
/robot<id>/goal_point_odom
/robot<id>/heuristic_point_odom
```

Use `/SLAM_map_<id>` for mapper output. `/robot<id>/SLAM_map` is accepted only as a merger compatibility alias for older code.

## Run The Integrated Demo

Terminal 1, simulation and autonomy:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py
```

The launch starts Gazebo, both robot mappers/controllers, the coordinator, map merger, and both CV pipelines. It defaults to `fresh_start:=true`, which clears stale Gazebo/demo processes before loading the world. To reuse an existing Gazebo session:

```bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py fresh_start:=false
```

The default run uses `min_exploration_before_goal_sec:=24.0`, so a goal seen immediately by the camera is held briefly while both robots begin frontier exploration. Lower it for a faster final answer:

```bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py min_exploration_before_goal_sec:=12.0
```

Lower-load detection option:

```bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py process_every_n:=3 use_fastsam:=false
```

## Visual Evidence

Terminal 2, RViz:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d /root/ros2_ws/src/final_project_cv/rviz/integrated_demo.rviz
```

RViz shows:

- `/SLAM_map_1` for robot 1's live occupancy grid.
- `/merged_map` when map alignment has enough confidence.
- `/final_start_to_goal_path` and `/final_start_to_goal_nav_path` after the final answer is available.
- `/final_result_markers` for the chosen start, detected goal, and path overlay.
- Goal segmentation image panels for both robot cameras.
- `/SLAM_map_2` disabled by default; enable it after map alignment or switch fixed frame to `robot2/odom`.

Terminal 3, optional larger image viewer:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Useful image topics:

```text
/robot1/camera/image_raw
/robot1/heuristic_debug_image
/robot1/goal_debug_image
/robot2/camera/image_raw
/robot2/heuristic_debug_image
/robot2/goal_debug_image
```

## Useful Topics

```bash
ros2 topic echo --qos-durability transient_local --once /SLAM_map_1
ros2 topic echo --qos-durability transient_local --once /merged_map
ros2 topic echo --qos-durability transient_local --once /merge_status
ros2 topic echo /robot1/goal_point_odom
ros2 topic echo /robot2/goal_point_odom
ros2 topic echo --qos-durability transient_local --once /final_start_to_goal_path
ros2 topic echo --qos-durability transient_local --once /final_start_to_goal_nav_path
ros2 topic echo --qos-durability transient_local --once /final_result_markers
ros2 topic echo --once /mission_complete
```

Important terminal lines:

```text
Goal point for robot ... using ...
cmd_vel publishing: linear=...
merged_map_published anchor=robot1 follower=robot2 confidence=...
GOAL FOUND: final A* path starts at robot_..., path_length=... m, map_source=...
FINAL PATH ACQUIRED: closest_start_robot=robot_..., path_kind=..., map_source=..., path_frame=..., path_length_m=..., markers_topic=/final_result_markers
final path artifacts saved: svg=/root/ros2_ws/src/final_path_results/final_start_to_goal_path.svg, csv=/root/ros2_ws/src/final_path_results/final_start_to_goal_path.csv, map_png=/root/ros2_ws/src/final_path_results/final_start_to_goal_map.png
Mission complete received; stopping exploration
```

## Report Visuals

While the demo is running, capture live CV and map snapshots:

```bash
python3 /root/ros2_ws/src/final_project_cv/tools/capture_report_snapshots.py \
  --results-dir /root/ros2_ws/src/final_path_results \
  --seconds 12
```

After the final path is saved, generate report-ready diagrams from the saved artifacts and snapshots:

```bash
python3 /root/ros2_ws/src/final_project_cv/tools/generate_report_visuals.py \
  --results-dir /root/ros2_ws/src/final_path_results
```

Generated files:

```text
/root/ros2_ws/src/final_path_results/report_visuals/report_visual_results_pack.png
/root/ros2_ws/src/final_path_results/report_visuals/report_demo_evidence_panel.png
/root/ros2_ws/src/final_path_results/report_visuals/report_map_progression.png
/root/ros2_ws/src/final_path_results/report_visuals/report_cv_detection_evidence.png
/root/ros2_ws/src/final_path_results/report_visuals/report_waypoint_trace.png
/root/ros2_ws/src/final_path_results/report_visuals/report_system_flow.png
/root/ros2_ws/src/final_path_results/report_visuals/report_topic_flow.png
/root/ros2_ws/src/final_path_results/report_visuals/report_behavior_timeline.png
/root/ros2_ws/src/final_path_results/report_visuals/report_visual_index.md
```

## Behavior Summary

- Robots explore frontiers while the goal is unknown.
- Robot motion uses LiDAR clearance shaping while moving, so it slows and turns away before emergency recovery is needed.
- Heuristic bottle detections bias frontier choice before goal detection.
- As soon as a goal sphere is seen by either robot, heuristic bias is ignored and motion freezes while the final A* answer is published.
- As soon as the final A* answer is available, motion stops.
- The final returned path is the shortest A* route from a robot start to the detected goal, chosen by path length.

## Reset

If Gazebo or ROS nodes look stale after several launches:

```bash
pkill -f '[m]apper.py' || true
pkill -f '[c]oordinator.py' || true
pkill -f '[m]ap_merger_node' || true
pkill -f '[v]ision_target_detector' || true
pkill -f '[t]arget_localizer' || true
pkill -9 gzserver || true
pkill -9 gzclient || true
pkill -9 gazebo || true
```
