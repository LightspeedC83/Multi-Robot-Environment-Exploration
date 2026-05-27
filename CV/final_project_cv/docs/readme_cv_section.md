# Computer Vision Target Localization

This folder contains the computer vision and target-localization portion of the project. The goal is to detect a target object in the camera image, estimate where it is relative to the robot, and publish that estimate in the `odom` frame so it can later be used by navigation or path planning.

## Pipeline Overview

The intended physical-robot pipeline is:

```text
camera image
-> YOLO target detection
-> FastSAM mask refinement
-> centroid + apparent object size
-> monocular depth estimate
-> target point in odom
```

YOLO is used to identify the target class, such as `sports ball` or `bottle`. FastSAM is then used inside the YOLO bounding box to get a better centroid estimate from the object mask rather than relying only on the rectangular box center.

The detector publishes:

```text
/target_centroid
```

where the message is a `geometry_msgs/PointStamped`:

```text
point.x = centroid u coordinate in pixels
point.y = centroid v coordinate in pixels
point.z = observed target diameter in pixels
```

The localization node combines this with camera intrinsics from:

```text
/camera/camera_info
```

and publishes:

```text
/target_point_odom
/target_pose_odom
```

## Depth Assumption

Because we are not using stereo vision or a depth camera, the target depth is estimated from an assumed real-world object diameter.

The main equation is:

```text
Z = f * D / d_px
```

where:

- `Z` is the estimated depth
- `f` is the camera focal length in pixels
- `D` is the real target diameter in meters
- `d_px` is the observed target diameter in pixels

Then the centroid is back-projected into 3D:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

This works best when the target has a known size and is roughly spherical, which is why a ball is a useful test object.

## Terminal Logic Test

Before connecting to the physical robot, we can test the localization math without loading YOLO, FastSAM, or PyTorch:

```bash
ros2 launch final_project_cv terminal_logic_pipeline.launch.py
```

This publishes synthetic centroid measurements and camera intrinsics, then runs the real localization node. It is useful for verifying the message flow:

```text
synthetic centroid
-> camera intrinsics
-> target_localizer
-> odom target point
```

The test also records a CSV and plot:

```text
output/target_trace.csv
output/target_trace.png
```

These plots show how changing image centroids and apparent object size produce different estimated target points.

## Physical Robot Test

Once the robot camera is available, the full pipeline can be launched with:

```bash
ros2 launch final_project_cv target_tracking_pipeline.launch.py \
  target:="sports ball" \
  object_diameter_m:=0.22 \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  target_frame:=odom \
  use_fastsam:=true
```

The exact camera topics and camera frame may need to be adjusted depending on the robot.

Useful checks:

```bash
ros2 topic list | grep camera
ros2 topic echo /camera/camera_info --once
ros2 topic echo /target_centroid
ros2 topic echo /target_point_odom
```

## Semantic Search Heuristic

For path planning, the team is also considering a semantic search heuristic. The idea is to maintain a belief grid over possible target locations.

- All cells start with a uniform belief.
- If the robot observes an area and does not see the target or a useful clue, the belief of those cells is reduced.
- If the robot sees a heuristic clue, cells around that clue are boosted.
- If the target is found, search ends and the robot switches to direct target localization.

In simple form:

```text
no clue observed:  B(cell) <- B(cell) / alpha
clue observed:     B(cell) <- beta * B(cell)
target observed:   switch to target localization
```

This lets the planner prefer regions that are more likely to contain the target while still avoiding areas that have already been checked.
