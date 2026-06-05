# mapper

Mapping, local control, and planning/coordinator package for the multi-robot demo.

## Nodes

`mapper.py` runs once per robot. It:

- builds a local occupancy grid from LiDAR scans,
- publishes `/SLAM_map_<id>` and `/pose_<id>`,
- follows coordinator paths from `/nav_path_<id>`,
- performs LiDAR clearance shaping plus local obstacle/stall recovery,
- stops on `/mission_complete`.

`coordinator.py` runs once globally. It:

- assigns robot IDs through `/get_unique_id`,
- subscribes to each robot map, pose, heuristic point, and goal point,
- chooses frontier paths while the goal is unknown,
- publishes the final shortest A* start-to-goal path on `/final_start_to_goal_path` and `/final_start_to_goal_nav_path`,
- publishes `/final_result_markers` for the chosen start, detected goal, and final path,
- saves final path PNG/SVG/CSV artifacts under `/root/ros2_ws/src/final_path_results/`,
- publishes `/mission_complete` and zero velocity commands when the answer is ready.

## Behavior

- Unknown goal: robots explore frontier cells ranked by distance plus simulated raycast information gain, with heuristic bottle detections as soft search bias.
- Goal candidate seen: the coordinator waits for repeated stable goal observations, rejects clusters sitting directly on bottle clues, disables heuristic bias, and publishes the final A* answer.
- Obstacle handling: mappers maintain LiDAR clearance while moving, then back up, turn, and replan if motion still stalls near obstacles.
- Path following: each mapper clips new paths to the nearest remaining waypoint before driving.
- Final answer: the coordinator chooses the shortest available A* path from a robot starting pose to the detected goal.

## Mapping

`PROBABILISTIC_MAPPING = True` enables log-odds occupancy updates. Published occupancy-grid values are:

- `-1`: unknown,
- `0`: likely free,
- `1-99`: probabilistic occupancy belief,
- `100`: occupied.

This is an obstacle/map belief. It is not a target-belief map.

## Topics

The mapper/coordinator interface uses flat ID-suffixed coordination topics. Robot hardware and CV topics use `/robot<id>/...` namespaces.

Mapper output:

```text
/SLAM_map_<id>
/pose_<id>
/id_active_<id>
```

Coordinator output:

```text
/nav_path_<id>
/final_start_to_goal_path
/final_start_to_goal_nav_path
/final_result_markers
/mission_complete
```

Merged-map and registration topics:

```text
/new_robot_id
/merged_map
/merge_status
```

CV inputs consumed by the coordinator:

```text
/robot1/heuristic_point_odom
/robot1/goal_point_odom
/robot2/heuristic_point_odom
/robot2/goal_point_odom
```

## Services

```text
/get_unique_id        mapper_interfaces/srv/GetUniqueID
/get_path             mapper_interfaces/srv/GetNewFrontierPath
```

## Run In Integrated Demo

Use the project launch rather than starting mapper/coordinator manually:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py
```

The integrated launch holds early goal observations while local maps mature. During that gate the coordinator still serves frontier paths, so map growth is visible before the final shortest A* path is published. Defaults are `min_exploration_before_goal_sec:=45.0`, `min_local_map_known_ratio_before_goal:=0.70`, `min_goal_observations_before_acceptance:=5`, `goal_heuristic_rejection_radius_m:=0.38`, and `max_exploration_before_goal_sec:=105.0`. Once the final answer exists, `/mission_complete` stops the robots and the finalizer writes report evidence; the finalizer also watches `/final_start_to_goal_path` directly so the launch does not hang if that completion flag is missed.

For map evidence:

```bash
rviz2 -d /root/ros2_ws/src/final_project_cv/rviz/integrated_demo.rviz
```

Useful checks:

```bash
ros2 topic echo --qos-durability transient_local --once /SLAM_map_1
ros2 topic echo --qos-durability transient_local --once /merge_status
ros2 topic echo --qos-durability transient_local --once /final_start_to_goal_path
ros2 topic echo --qos-durability transient_local --once /final_start_to_goal_nav_path
ros2 topic echo --qos-durability transient_local --once /final_result_markers
ros2 topic echo --once /mission_complete
```

Final path files:

```text
/root/ros2_ws/src/final_path_results/final_start_to_goal_path.svg
/root/ros2_ws/src/final_path_results/final_start_to_goal_map.png
/root/ros2_ws/src/final_path_results/final_start_to_goal_path.csv
/root/ros2_ws/src/final_path_results/final_start_to_goal_summary.txt
```
