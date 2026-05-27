# Physical Robot Pipeline

This package expects a real ROS camera stream and camera calibration. It detects one target class, estimates depth from the apparent target size, and publishes the target position in `odom`.

The detector uses YOLO to find the target box, then FastSAM refines the centroid from the target mask. FastSAM is enabled by default in `target_tracking_pipeline.launch.py`.

## Check The Robot

Install the Python detector dependencies inside the ROS container with NumPy pinned below 2. ROS Humble binary packages such as `cv_bridge` are built against NumPy 1.x.

```bash
cd ~/ros2_ws/src/final_project_cv
python3 -m pip uninstall -y numpy opencv-python matplotlib ultralytics
python3 -m pip install --force-reinstall -r requirements_ros.txt
```

If your `pip` supports `--break-system-packages` and asks for it, add that flag. Older ROS Humble containers do not support that option.

Confirm the imports are healthy:

```bash
python3 -c "import numpy; print(numpy.__version__); from cv_bridge import CvBridge; from ultralytics import FastSAM, YOLO; print('imports ok')"
```

```bash
ros2 topic list | grep camera
ros2 topic echo /camera/camera_info --once
ros2 run tf2_ros tf2_echo odom camera_link
```

Use the camera topics and frame names that actually exist on the robot. Common alternatives are:

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `camera_color_optical_frame`
- `base_link`

## Terminal-Only Video Test

Use this when you are not connected to the physical robot. It publishes frames from a video file as `/camera/image_raw`, publishes matching `/camera/camera_info`, runs YOLO + FastSAM, and prints the localizer output in the terminal.

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch final_project_cv terminal_video_pipeline.launch.py
```

To use a different video:

```bash
ros2 launch final_project_cv terminal_video_pipeline.launch.py \
  source:=/absolute/path/to/video.mp4 \
  target:="sports ball" \
  object_diameter_m:=0.22
```

This launch sets the camera frame to `odom` so it does not need robot TF. It is for terminal verification of the message flow and depth math, not a physical robot pose test.

## Terminal-Only Logic Test

Use this when you want no robot, no video, no YOLO, no FastSAM, and no PyTorch warnings. It publishes changing centroid measurements directly, then runs the real `target_localizer`.

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch final_project_cv terminal_logic_pipeline.launch.py
```

This is the cleanest command for explaining the localization logic:

```text
synthetic centroid + pixel diameter -> /target_centroid
camera intrinsics -> /camera/camera_info
target_localizer -> /target_point_odom
```

It also records the trace:

```text
/root/ros2_ws/src/final_project_cv/output/target_trace.csv
/root/ros2_ws/src/final_project_cv/output/target_trace.png
```

## Run The Pipeline

Bottle example:

```bash
ros2 launch final_project_cv target_tracking_pipeline.launch.py \
  target:=bottle \
  object_diameter_m:=0.07 \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  target_frame:=odom \
  use_fastsam:=true
```

Sports ball example:

```bash
ros2 launch final_project_cv target_tracking_pipeline.launch.py \
  target:="sports ball" \
  object_diameter_m:=0.22 \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  target_frame:=odom \
  use_fastsam:=true
```

If TF uses a camera frame name that is not present in the image header, pass it explicitly:

```bash
ros2 launch final_project_cv target_tracking_pipeline.launch.py \
  target:="sports ball" \
  object_diameter_m:=0.22 \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  target_frame:=odom \
  camera_frame:=camera_color_optical_frame \
  use_fastsam:=true
```

## Inspect Outputs

```bash
ros2 topic echo /target_centroid
ros2 topic echo /target_point_odom
ros2 run rqt_image_view rqt_image_view
```

In `rqt_image_view`, select `/target_debug_image` to see the selected detection.

The debug image shows the YOLO box in blue, the FastSAM mask in green, and the published centroid as a red dot.

## What The Localizer Assumes

Without stereo or depth, the target depth is estimated from object size:

```text
Z = f * real_object_diameter_m / observed_diameter_px
```

This works best when the target is roughly spherical or when its visible size is close to the assumed diameter.
