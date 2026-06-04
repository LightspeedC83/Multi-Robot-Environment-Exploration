# Multi-Robot Environment Exploration

Two robots explore an unknown Gazebo environment, build local occupancy-grid maps, use camera detections to find a visual goal, and return the shortest available path between the closest robot start position and the goal.

## Behavior

- Each robot runs local mapping, obstacle avoidance, and frontier exploration.
- Bottle detections are heuristic clues only before the goal is seen.
- Once either robot sees the goal sphere, both robots switch to goal-directed motion and ignore heuristic clues.
- The coordinator publishes the final shortest A* start-to-goal path on `/final_start_to_goal_path` and `/final_start_to_goal_nav_path`.
- The coordinator saves final path evidence under `/root/ros2_ws/src/final_path_results/`.
- When the final answer is available, `/mission_complete` stops robot motion.

## Topic Convention

Coordination topics use the flat `<name>_<id>` convention:

```text
/new_robot_id
/SLAM_map_<id>
/pose_<id>
/id_active_<id>
/nav_path_<id>
```

Robot hardware, sensor, and CV topics stay namespaced:

```text
/robot<id>/cmd_vel
/robot<id>/scan
/robot<id>/odom
/robot<id>/goal_point_odom
/robot<id>/heuristic_point_odom
```

New code should publish maps as `/SLAM_map_<id>`. The merger also listens to `/robot<id>/SLAM_map` as a compatibility alias for older branches, but that is not the project convention.

## Quickstart

Start Docker from the host:

```bash
cd /Users/emiliodaza/Dartmouth/Robotics/integrated-multi-robot/vnc-ros
docker compose up -d
docker compose exec ros bash
```

Open the visual desktop:

```text
http://localhost:8080/vnc.html
```

Run the integrated demo inside the ROS container:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py
```

The launch defaults to `fresh_start:=true`, so stale Gazebo and ROS demo processes are cleared and the robots reload at the SDF start poses. To reuse an already-running Gazebo session:

```bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py fresh_start:=false
```

## Evidence Windows

Open RViz for occupancy grids, TF, final path, and goal segmentation panels:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d /root/ros2_ws/src/final_project_cv/rviz/integrated_demo.rviz
```

The RViz config shows `/SLAM_map_1`, `/merged_map`, `/final_start_to_goal_path`, `/final_start_to_goal_nav_path`, and goal debug images for both robots. `/SLAM_map_2` is included but disabled by default; enable it after map alignment or switch the fixed frame to `robot2/odom` to inspect robot 2 locally.

Open larger camera/debug views:

```bash
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

Useful terminal checks:

```bash
ros2 topic echo --qos-durability transient_local --once /SLAM_map_1
ros2 topic echo --qos-durability transient_local --once /merged_map
ros2 topic echo --qos-durability transient_local --once /merge_status
ros2 topic echo --qos-durability transient_local --once /final_start_to_goal_path
ros2 topic echo --qos-durability transient_local --once /final_start_to_goal_nav_path
ros2 topic echo --once /mission_complete
```

Successful demo logs include:

```text
FINAL PATH ACQUIRED: closest_start_robot=robot_..., path_kind=..., path_length_m=..., waypoints=...
merged_map_published anchor=robot1 follower=robot2 confidence=...
final path artifacts saved: svg=/root/ros2_ws/src/final_path_results/final_start_to_goal_path.svg, csv=/root/ros2_ws/src/final_path_results/final_start_to_goal_path.csv
Mission complete received; stopping exploration
```

## Packages

- `final_project_cv`: Gazebo world, camera target detection, target localization, RViz config, integrated launch.
- `mapper`: local occupancy mapping, robot control, coordinator, frontier/goal planning, final path publication.
- `merger`: map alignment and merged occupancy-grid publication.
- `mapper_interfaces`: ROS 2 services used by mappers and coordinator.

## Reset

If Gazebo or old nodes survive repeated launches, run inside the ROS container:

```bash
pkill -f '[m]apper.py' || true
pkill -f '[c]oordinator.py' || true
pkill -f '[m]ap_merger_node' || true
pkill -9 gzserver || true
pkill -9 gzclient || true
pkill -9 gazebo || true
```
