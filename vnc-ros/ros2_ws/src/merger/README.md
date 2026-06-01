# Multi-Robot Map Coordination

A ROS 2 wrapper around a pure-Python map-fusion library. Any number of robots publish their SLAM maps, and this node produces a single globally-merged map plus the static transforms needed to work in a shared odom frame.

## Files

| File | Purpose |
| --- | --- |
| `map_coordinator.py` | Pure NumPy/OpenCV library that aligns two `nav_msgs/OccupancyGrid` maps, scores the alignment, and fuses them. No ROS dependencies, so it can be unit-tested offline. |
| `controller.py` | ROS 2 node that subscribes to each robot's map, calls `map_coordinator.merge_maps()`, publishes the fused map on `/merged_map`, and broadcasts static TFs between robots' odom frames. |

## How it works

When the node starts, it knows about zero robots. Robots are added dynamically as the mapper coordinator hands out IDs:

controller starts
└── subscribes to /new_robot_id
└── robot_states = {}  (empty)
Robot 1 boots, calls get_unique_id service
└── mapper coordinator assigns ID=1, publishes Int32(1) on /new_robot_id
└── controller receives it
├── creates RobotMapState('robot1')
└── subscribes to /robot1/SLAM_map
Robot 2 boots, same flow → controller subscribes to /robot2/SLAM_map
Both robots now publish maps
└── controller merges, publishes /merged_map, broadcasts inter-odom TF

Alignment is attempted in two situations:
- a robot has discovered 500 new known cells since the last attempt, or
- every 5 seconds as a periodic fallback.

Robot 1 is always the anchor when present, so the merged map lives in `robot1/odom`.

## ROS 2 Interface

**Subscriptions:**

| Topic | Type | Notes |
| --- | --- | --- |
| `/new_robot_id` | `std_msgs/Int32` | Triggers dynamic registration of a new robot. |
| `/<robot_id>/SLAM_map` | `nav_msgs/OccupancyGrid` | One subscription per registered robot. |

**Publications:**

| Topic | Type | Notes |
| --- | --- | --- |
| `/merged_map` | `nav_msgs/OccupancyGrid` | Fused global map. Frame: `robot1/odom`. |

**Static TFs:**

`robot<anchor>/odom → robot<follower>/odom`, broadcast via `tf2_ros/StaticTransformBroadcaster` once an alignment is accepted. Lets planners and visualizers transform poses between robot odom frames.

## Parameters

| Name | Default | Description |
| --- | --- | --- |
| `confidence_threshold` | `0.5` | Minimum confidence (0–1) for a merge to be accepted. Raise for more conservative fusing. |
| `TRANSFORM_RETRY_CELLS` | `500` | Newly-known cells a robot must discover before triggering a new alignment attempt. Hard-coded. |
| `COORDINATION_PERIOD_SEC` | `5.0` | Seconds between periodic coordination passes when no robot fires a trigger. Hard-coded. |

Change the threshold at runtime:

```bash
ros2 param set /map_coordinator confidence_threshold 0.7
```

## Running the node

### 1. Start the container

From the `vnc-ros` folder on your host machine:

```bash
cd path/to/vnc-ros
docker compose up -d
docker compose exec ros bash
```

### 2. Source ROS 2 and build

Inside the container:

```bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws
colcon build --packages-select merger
source install/setup.bash
```

### 3. Launch

```bash
ros2 run merger map_merger_node
```

Expected log output on a clean start:
[INFO] [map_coordinator]: map_coordinator ready, waiting for robots to register, confidence_threshold=0.5

Once robots register and start publishing maps, you'll see logs like:
[INFO] [map_coordinator]: New robot registered: robot1, subscribing to /robot1/SLAM_map
[INFO] [map_coordinator]: align robot1<-robot2: inliers=19, wall_agree=0.81, conf=0.78, success=True
[INFO] [map_coordinator]: broadcasted static TF robot1/odom <- robot2/odom (tx=-0.143m, ty=0.482m, theta=14.88deg)

## Viewing the merged map in RViz

To watch the coordinator working live, open RViz alongside the simulation and display both robots' maps plus the merged result.

### Launch RViz

In a new terminal inside the container:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2
```

### Configure displays

In the left-hand panel:

- **Fixed Frame** (under "Global Options"): `robot1/odom` — this is the frame the merged map is published in.

Click **Add** at the bottom of the displays panel and add three `Map` displays:

| Display name | Topic | Color scheme |
| --- | --- | --- |
| Robot 1 map | `/robot1/SLAM_map` | `map` (default) |
| Robot 2 map | `/robot2/SLAM_map` | `costmap` (for contrast) |
| Merged map | `/merged_map` | `raw` |

Drop each map's alpha to ~0.6 to see them all at once.

### What you should see

- **Before alignment succeeds:** robot 1's map appears centered. Robot 2's map appears wherever its odom frame places it. `/merged_map` is empty.
- **After alignment succeeds:** the merged map appears, covering both explored regions in robot 1's frame. The coordinator log shows `success=True` and the inter-odom TF.

### Troubleshooting

- **Merged map never appears.** Check the coordinator's logs for the `conf=...` value. If it stays below 0.5, the robots haven't observed enough common structure yet. Drive both robots into an overlapping region.
- **Robot 2's map appears at an odd offset before alignment.** Expected — the two robots' odom frames are independent until the coordinator publishes the static TF.
- **"Fixed Frame doesn't exist" error in RViz.** Robot 1's mapper isn't publishing yet, or the TF tree hasn't reached that frame. Verify with `ros2 topic echo /robot1/SLAM_map` and `ros2 run tf2_tools view_frames`.