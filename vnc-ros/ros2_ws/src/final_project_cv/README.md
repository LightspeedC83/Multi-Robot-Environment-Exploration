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
ros2 launch final_project_cv integrated_two_robot_demo.launch.py process_every_n:=3 use_fastsam:=false
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
