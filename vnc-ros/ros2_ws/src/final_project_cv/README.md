# Computer Vision Target Localization

This ROS 2 package contains the computer vision portion of the multi-robot exploration project. It runs inside the `vnc-ros` Docker workspace and provides a lightweight Gazebo test world where the robot can detect:

- `bottle` objects as semantic heuristic clues
- `sports ball` as the final goal target

The pipeline uses YOLO for object detection, optionally refines the selected object with FastSAM, estimates a 3D target point using a monocular size assumption, and publishes that point in the `odom` frame.

## Pipeline

```text
Gazebo camera image
-> YOLO detection
-> FastSAM centroid refinement
-> monocular depth estimate from known object size
-> transform from camera frame to odom
-> publish heuristic and goal target points
```

YOLO gives the object class and bounding box. FastSAM is applied inside the selected box to improve the centroid estimate. If the object is detected, the localizer estimates depth from its apparent pixel diameter:

```text
Z = f * D / d_px
```

where `D` is the assumed real-world object diameter and `d_px` is the observed diameter in pixels. The point is then back-projected:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

The final point is transformed into `odom`, so planning code can subscribe to a stable map-frame estimate.

## Main Topics

Inputs:

```text
/camera/image_raw
/camera/camera_info
/tf
/odom
/cmd_vel
```

Detection outputs:

```text
/heuristic_centroid
/goal_centroid
/heuristic_debug_image
/goal_debug_image
```

Odom-frame target outputs:

```text
/heuristic_point_odom
/goal_point_odom
/heuristic_pose_odom
/goal_pose_odom
```

`/heuristic_point_odom` is the estimated bottle location. `/goal_point_odom` is the estimated sports ball location.

## Run The Demo

Start the Docker environment from the host machine:

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

This opens the lightweight world with a robot, camera, odometry, sparse obstacles, heuristic bottles, and an orange sports ball goal. The world also includes a top-down default view and a blue camera field-of-view visualization.

### Terminal 2: CV Pipeline

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch final_project_cv dual_target_tracking_pipeline.launch.py process_every_n:=2 use_fastsam:=true
```

This starts two detector/localizer pairs:

```text
bottle       -> heuristic detector -> /heuristic_point_odom
sports ball  -> goal detector      -> /goal_point_odom
```

The terminal logs print centroid, FastSAM usage, estimated range, and odom coordinates when a target is visible.

### Terminal 3: Debug Images

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Useful image topics:

```text
/camera/image_raw
/heuristic_debug_image
/goal_debug_image
```

The debug image shows the selected box, segmentation mask when available, and the centroid used for localization.

### Terminal 4: Teleop

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Common controls:

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

## Inspect Outputs

Echo the odom-frame detections:

```bash
ros2 topic echo /heuristic_point_odom
ros2 topic echo /goal_point_odom
```

List the relevant topics:

```bash
ros2 topic list | grep -E "camera|heuristic|goal|odom|cmd_vel"
```

Expected log examples:

```text
heuristics detected: centroid=(...), diameter_px=..., source=fastsam
heuristics detected: odom=(x=..., y=..., z=...), range=...

goal found sphere detected: centroid=(...), diameter_px=..., source=fastsam
goal found sphere detected: odom=(x=..., y=..., z=...), range=...
```

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

The CV package provides the semantic observations and odom-frame coordinates needed for that belief update.
