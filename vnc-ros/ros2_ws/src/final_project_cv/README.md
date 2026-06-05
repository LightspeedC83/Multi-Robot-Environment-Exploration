# final_project_cv

Computer-vision and simulation package for the integrated two-robot demo. It provides the Gazebo world, target detectors, target localizers, RViz config, and launch files used by the mapper/coordinator stack.

## Targets

- `sports ball`: goal target.
- `bottle`: heuristic clue used only before the goal is found.

Once a goal observation exists, the coordinator ignores heuristic detections, holds robot motion, and publishes the final A* answer.

## Detection And Localization

Each robot runs two detector/localizer pairs:

```text
camera image
-> detector bounding box / centroid
-> target localizer
-> odom-frame PointStamped target
```

The localizer prefers the LiDAR beam at the camera-centroid bearing when that range is consistent with the visual size estimate. If the beam is likely an obstacle or self-hit, it falls back to the visual estimate and logs the rejected LiDAR range.

The integrated demo uses YOLO for both detector classes (`sports ball` and `bottle`), then FastSAM lazy-loads and refines the measurement with a learned mask when a target is selected. False final goals are handled downstream by stable goal clustering and rejection of goal clusters that sit on top of heuristic bottle clues.

Example log:

```text
goal found sphere detected: robot1/odom=(x=..., y=..., z=...), range=..., lidar=... m near ... deg, image_centroid=(...), vision_planar_range=...
```

## Main Topics

Robot 1:

```text
/robot1/camera/image_raw
/robot1/camera/camera_info
/robot1/scan
/robot1/odom
/robot1/cmd_vel
/robot1/heuristic_debug_image
/robot1/goal_debug_image
/robot1/heuristic_point_odom
/robot1/goal_point_odom
```

Robot 2:

```text
/robot2/camera/image_raw
/robot2/camera/camera_info
/robot2/scan
/robot2/odom
/robot2/cmd_vel
/robot2/heuristic_debug_image
/robot2/goal_debug_image
/robot2/heuristic_point_odom
/robot2/goal_point_odom
```

Target messages are `geometry_msgs/PointStamped` in the corresponding robot odom frame, such as `robot1/odom` or `robot2/odom`.

## Run

Integrated project demo:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py
```

By default, early goal detections are held while both local maps mature. The final answer waits for `min_exploration_before_goal_sec:=45.0`, `min_local_map_known_ratio_before_goal:=0.70` inside the arena region, `min_goal_observations_before_acceptance:=5` stable clustered goal observations, and `goal_heuristic_rejection_radius_m:=0.38` so cap-like clusters on bottle clues are ignored. `max_exploration_before_goal_sec:=105.0` is the safety cap. After that gate passes, the launch waits until the final A* answer exists, captures a few seconds of evidence, prints `RESULTS READY`, and exits. For a faster debug result:

```bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py min_exploration_before_goal_sec:=12.0 min_local_map_known_ratio_before_goal:=0.35 min_goal_observations_before_acceptance:=2
```

The integrated launch also defaults to `auto_finalize:=true`: after `/mission_complete` or a non-empty final path on `/final_start_to_goal_path`, it captures final snapshots, regenerates report visuals, prints `RESULTS READY`, and shuts down. Use `auto_finalize:=false` for manual visualization sessions.

The detector also saves first-goal CV evidence under `/root/ros2_ws/src/final_path_results/snapshots/`, so the report can show the raw camera and goal overlay from the moment the goal was first observed.

Gazebo world only:

```bash
ros2 launch final_project_cv lightweight_targets_gazebo.launch.py
```

CV pipeline for one robot:

```bash
ros2 launch final_project_cv dual_target_tracking_pipeline.launch.py robot_namespace:=robot1
```

Lower-load integrated run:

```bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py process_every_n:=4 process_width:=320 imgsz:=320
```

## Visualization

RViz:

```bash
rviz2 -d /root/ros2_ws/src/final_project_cv/rviz/integrated_demo.rviz
```

The RViz config includes the final shortest A* start-to-goal path as both a PoseArray and a continuous Path line:

```text
/final_start_to_goal_path
/final_start_to_goal_nav_path
/final_result_markers
```

Map/planner evidence topics:

```text
/SLAM_map_1
/SLAM_map_2
/merged_map
/merge_status
```

Camera overlays:

```bash
ros2 run rqt_image_view rqt_image_view
```

Useful debug topics:

```text
/robot1/goal_debug_image
/robot2/goal_debug_image
/robot1/heuristic_debug_image
/robot2/heuristic_debug_image
```

## Report Visuals

While the integrated demo is running, capture live CV/map snapshots:

```bash
python3 /root/ros2_ws/src/final_project_cv/tools/capture_report_snapshots.py \
  --results-dir /root/ros2_ws/src/final_path_results \
  --seconds 12
```

After the final path is saved, generate clean report figures:

```bash
python3 /root/ros2_ws/src/final_project_cv/tools/generate_report_visuals.py \
  --results-dir /root/ros2_ws/src/final_path_results
```

The snapshot capture writes map PNGs plus raw `.npz` occupancy grids. The `.npz` files let the visual generator redraw the merged contribution map with robot 1 cells, robot 2 cells, overlap, occupied cells, and the A* path separated by color.

The output folder is `/root/ros2_ws/src/final_path_results/report_visuals/` and includes the final result panel, map-progression panel, merged-contribution map, CV evidence panel, waypoint trace, system-flow diagram, topic-flow diagram, and behavior timeline.
