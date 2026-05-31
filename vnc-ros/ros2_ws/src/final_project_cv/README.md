# Computer Vision Target Localization

This ROS 2 package contains the computer vision part of the multi-robot exploration project. It runs inside the `vnc-ros` Docker workspace and provides a lightweight Gazebo world with two small robots, sparse obstacles, heuristic bottles, and an orange sports ball goal.

Each robot has:

- RGB camera
- 2D LiDAR
- differential drive teleop through `/robotX/cmd_vel`
- odometry through `/robotX/odom`
- namespaced camera, scan, debug, and target topics

The CV pipeline uses YOLO for object detection, optionally refines the selected object with FastSAM, estimates a 3D target point using a monocular size assumption, and logs the LiDAR beam that corresponds to the target bearing.

## Pipeline

```text
Gazebo camera image
-> YOLO detection
-> FastSAM centroid refinement
-> monocular depth estimate from known object size
-> transform camera point into robot odom
-> compare with LiDAR beam at the same bearing
-> publish heuristic and goal target points
```

YOLO gives the object class and bounding box. FastSAM is applied inside the selected box to improve the centroid estimate. The localizer estimates depth from the apparent pixel diameter:

```text
Z = f * D / d_px
```

where `D` is the assumed real-world object diameter and `d_px` is the observed diameter in pixels. The point is then back-projected:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

The resulting point is transformed into the robot odom frame. The localizer also transforms that target point into the LiDAR frame, finds the nearest scan beam by bearing, and prints the matching LiDAR range in the terminal log.

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

`heuristic_point_odom` is the estimated bottle location. `goal_point_odom` is the estimated sports ball location.

## Run The Demo

Start Docker from the host machine:

```bash
cd ~/Dartmouth/Robotics/vnc-ros
docker compose up -d
```

Open a Docker terminal:

```bash
docker compose exec ros bash
```

Build the package:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select final_project_cv
source install/setup.bash
```

### Terminal 1: Gazebo

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch final_project_cv lightweight_targets_gazebo.launch.py
```

This starts the two-robot world. Gazebo opens from above and shows each robot's blue camera-view footprint.

### Terminal 2: Robot 1 CV Pipeline

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch final_project_cv dual_target_tracking_pipeline.launch.py process_every_n:=2 use_fastsam:=true
```

This defaults to `robot1`. It starts the bottle detector/localizer and the sports-ball detector/localizer:

```text
bottle       -> /robot1/heuristic_point_odom
sports ball  -> /robot1/goal_point_odom
```

The terminal logs include the odom point, the monocular vision range, and the LiDAR range near the corresponding target bearing.

### Terminal 3: Robot 1 Debug Images

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
```

### Terminal 4: Robot 1 Teleop

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot1/cmd_vel
```

Controls:

```text
i   forward
,   backward
j   turn left
l   turn right
k   stop
q   increase speed
z   decrease speed
```

Keep the teleop terminal focused while driving.

## Running Robot 2 Instead

Gazebo already starts both robots. To run the same CV pipeline on robot 2, use:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch final_project_cv dual_target_tracking_pipeline.launch.py \
  robot_namespace:=robot2 \
  process_every_n:=2 \
  use_fastsam:=true
```

Robot 2 teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot2/cmd_vel
```

Running CV on both robots at once is possible, but it is heavier because it launches two YOLO/FastSAM pipelines. For smoother demos, start with one robot.

## Inspect Outputs

Echo target points:

```bash
ros2 topic echo /robot1/heuristic_point_odom
ros2 topic echo /robot1/goal_point_odom
```

Echo LiDAR:

```bash
ros2 topic echo /robot1/scan
```

List useful topics:

```bash
ros2 topic list | grep -E "robot1|robot2|camera|scan|heuristic|goal|odom|cmd_vel"
```

Expected terminal logs:

```text
heuristics detected: robot1/odom=(x=..., y=..., z=...), range=..., image_centroid=(...), lidar=... m near ... deg, vision_planar_range=... m

goal found sphere detected: robot1/odom=(x=..., y=..., z=...), range=..., image_centroid=(...), lidar=... m near ... deg, vision_planar_range=... m
```

The LiDAR value is not another object detector. It is the LaserScan range at the bearing where the CV-estimated target point should be. It is useful as a sanity check against the monocular depth estimate.

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

Then rebuild and launch again.

## Semantic Search Heuristic

The planning idea is to maintain a belief grid over possible goal locations:

- All cells start with a uniform belief.
- If the robot observes an area and sees no goal or heuristic, those cells are down-weighted.
- If the robot sees a heuristic bottle, nearby cells are up-weighted.
- If the sports ball is found, search ends and the robot can switch to goal-directed behavior.

In compact form:

```text
no clue observed:  B(cell) <- B(cell) / alpha
clue observed:     B(cell) <- beta * B(cell)
target observed:   finish search
```

The CV package provides the semantic observations, odom-frame target coordinates, and a matching LiDAR range check for each visible target.
