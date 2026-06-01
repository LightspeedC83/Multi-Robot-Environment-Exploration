# mapper

Mapping, local control, and planning/coordinator package for the multi-robot demo.

## Nodes

`mapper.py` runs once per robot. It:

- builds a local occupancy grid from LiDAR scans,
- publishes `/SLAM_map_<id>` and `/pose_<id>`,
- follows coordinator paths from `/nav_path_<id>`,
- performs local obstacle/stall recovery,
- stops on `/mission_complete`.

`coordinator.py` runs once globally. It:

- assigns robot IDs through `/get_unique_id`,
- subscribes to each robot map, pose, heuristic point, and goal point,
- chooses frontier paths while the goal is unknown,
- switches both robots to goal-directed paths once the goal is seen,
- publishes the final goal-to-start path on `/final_goal_to_start_path`,
- publishes `/mission_complete` and zero velocity commands when the answer is ready.

## Behavior

- Unknown goal: robots explore frontier cells ranked by distance plus simulated raycast information gain, with heuristic bottle detections as soft search bias.
- Goal seen: heuristic bias is disabled, and both robots receive goal-directed plans.
- Obstacle/stall recovery: mappers back up, turn, and replan when motion stalls near obstacles.
- Path following: each mapper clips new paths to the nearest remaining waypoint before driving.
- Final answer: the coordinator chooses the shortest available path from the detected goal to a robot starting pose.

## Mapping

`PROBABILISTIC_MAPPING = True` enables log-odds occupancy updates. Published occupancy-grid values are:

- `-1`: unknown,
- `0`: likely free,
- `1-99`: probabilistic occupancy belief,
- `100`: occupied.

This is an obstacle/map belief. It is not a target-belief map.

## Topics

Mapper output:

```text
/SLAM_map_<id>
/pose_<id>
/id_active_<id>
```

Coordinator output:

```text
/nav_path_<id>
/final_goal_to_start_path
/mission_complete
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

For map evidence:

```bash
rviz2 -d /root/ros2_ws/src/final_project_cv/rviz/integrated_demo.rviz
```

Useful checks:

```bash
ros2 topic echo --qos-durability transient_local --once /SLAM_map_1
ros2 topic echo --qos-durability transient_local --once /final_goal_to_start_path
ros2 topic echo --once /mission_complete
```
