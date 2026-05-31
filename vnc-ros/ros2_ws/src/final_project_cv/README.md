# Computer Vision Target Localization

This package is the CV interface for the multi-robot exploration project. It runs in the `vnc-ros` Docker workspace and provides a lightweight Gazebo world with two robots, each with:

- RGB camera
- 2D LiDAR
- odometry
- differential-drive `cmd_vel`
- YOLO/FastSAM target detection
- odom-frame target localization

The detector recognizes:

- `bottle` as a heuristic clue
- `sports ball` as the goal target

The output is intended for the planning and map-merging side of the project: each robot publishes semantic target observations in its own odom frame, plus a LiDAR sanity check at the same target bearing.

## CV Pipeline

```text
camera image
-> YOLO bounding box
-> FastSAM mask refinement
-> centroid and observed pixel diameter
-> monocular depth estimate
-> target point in robot odom frame
-> matching LiDAR beam check
```

Depth is estimated from the apparent target size:

```text
Z = f * D / d_px
```

where `D` is the assumed real object diameter and `d_px` is the observed diameter in pixels. The centroid is back-projected with:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

The localizer then transforms the point into the robot's odom frame and logs the LiDAR range near the same bearing.

## Planning Interface

Subscribe to these topics for semantic search and goal handling:

```text
/robot1/heuristic_point_odom
/robot1/goal_point_odom
/robot2/heuristic_point_odom
/robot2/goal_point_odom
```

Each message is a `geometry_msgs/PointStamped`.

```text
header.frame_id = robot1/odom or robot2/odom
point.x         = estimated target x in that odom frame
point.y         = estimated target y in that odom frame
point.z         = estimated target height/depth component from projection
```

Use these topics as semantic observations:

```text
bottle detected      -> boost belief near that odom point
sports ball detected -> goal found / terminate search or switch to approach behavior
no detection         -> planning may down-weight currently visible cells
```

Important frame note: `/robot1/heuristic_point_odom` is in `robot1/odom`, and `/robot2/heuristic_point_odom` is in `robot2/odom`. The map-merging layer must align those odom/map frames before combining target observations into one global belief grid.

## Map-Merging Interface

The CV package does not merge occupancy maps. It provides semantic target observations that can be fused after the mapping layer aligns robot frames.

Expected integration pattern:

```text
/robot1/scan -> robot1 mapper -> /robot1/SLAM_map
/robot2/scan -> robot2 mapper -> /robot2/SLAM_map

/robot1/SLAM_map + /robot2/SLAM_map
-> map merger estimates robot2 map transform into robot1/global frame
-> /merged_map

/robot1/heuristic_point_odom + /robot2/heuristic_point_odom
-> transform both semantic observations into merged-map frame
-> update heuristic belief grid
```

If the map merger chooses `robot1/odom` as the global reference, then robot 1 target points can be used directly and robot 2 target points must be transformed through the estimated map alignment.

## Topics

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

## Run The Two-Robot Demo

Start Docker from the host machine:

```bash
cd ~/Dartmouth/Robotics/vnc-ros
docker compose up -d
docker compose exec ros bash
```

Build:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select final_project_cv
source install/setup.bash
```

Terminal 1, Gazebo:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch final_project_cv lightweight_targets_gazebo.launch.py
```

Terminal 2, robot 1 CV:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch final_project_cv dual_target_tracking_pipeline.launch.py \
  robot_namespace:=robot1 \
  process_every_n:=2 \
  use_fastsam:=true
```

Terminal 3, robot 2 CV:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch final_project_cv dual_target_tracking_pipeline.launch.py \
  robot_namespace:=robot2 \
  process_every_n:=2 \
  use_fastsam:=true
```

Running both CV pipelines is heavier because it starts two YOLO/FastSAM stacks. If the laptop slows down, set `process_every_n:=3` or run CV for one robot at a time.

Terminal 4, image viewer:

```bash
ros2 run rqt_image_view rqt_image_view
```

Useful debug images:

```text
/robot1/heuristic_debug_image
/robot1/goal_debug_image
/robot2/heuristic_debug_image
/robot2/goal_debug_image
```

Terminal 5, robot 1 teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot1/cmd_vel
```

Terminal 6, robot 2 teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot2/cmd_vel
```

Teleop controls:

```text
i   forward
,   backward
j   turn left
l   turn right
k   stop
q   increase speed
z   decrease speed
```

## Inspect Outputs

Target points:

```bash
ros2 topic echo /robot1/heuristic_point_odom
ros2 topic echo /robot1/goal_point_odom
ros2 topic echo /robot2/heuristic_point_odom
ros2 topic echo /robot2/goal_point_odom
```

LiDAR:

```bash
ros2 topic echo /robot1/scan
ros2 topic echo /robot2/scan
```

Topic list:

```bash
ros2 topic list | grep -E "robot1|robot2|camera|scan|heuristic|goal|odom|cmd_vel"
```

Expected CV/localizer logs:

```text
heuristics detected: robot1/odom=(x=..., y=..., z=...), range=..., image_centroid=(...), lidar=... m near ... deg, vision_planar_range=... m

goal found sphere detected: robot2/odom=(x=..., y=..., z=...), range=..., image_centroid=(...), lidar=... m near ... deg, vision_planar_range=... m
```

The LiDAR value is not another classifier. It is the LaserScan range at the bearing where the CV-estimated target point should be, useful for checking whether the monocular depth estimate is plausible.

## Clean Restart

If Gazebo, TF, or FastDDS gets noisy after several restarts:

```bash
pkill -f gazebo || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f rqt_image_view || true
rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_*
rm -f ~/.gazebo/gui.ini
```

Then rebuild and relaunch.
