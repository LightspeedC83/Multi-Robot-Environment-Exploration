# Multi‑Robot Map Coordination  

This package provides a **lightweight ROS 2 wrapper** around a pure‑Python map‑fusion library.  
It lets any number of robots publish their SLAM maps and receives a single global map plus the static transforms needed to work in a common odom frame.

t=0s  robot1 starts mapping
t=?   robot1 hits 1000 cells  → first alignment attempt (likely fails, robot2 not ready)
t=5s  timer fires             → alignment attempt
t=10s timer fires             → alignment attempt
...
t=?   robot2 also has data, alignment succeeds
         → /tf_static updated
         → /merged_map published
t=?   robot1 maps 500 more cells → re-alignment attempt → update if successful
t=?   robot2 maps 500 more cells → re-alignment attempt → update if successful

## FULL FLOW

Mapper Coordinator starts
     │
     ▼
controller starts
  └── subscribes to /new_robot_id
  └── robot_states = {}  (empty)

Robot 1 boots, calls get_unique_id service
     │
     ▼
mapper coordinator: assigns ID=1, publishes Int32(1) on /new_robot_id
     │
     ▼
controller receives Int32(1)
  └── robot_id = 'robot1'
  └── creates RobotMapState('robot1')
  └── subscribes to /robot1/map

Robot 2 boots, calls get_unique_id service
     │
     ▼
mapper coordinator: assigns ID=2, publishes Int32(2) on /new_robot_id
     │
     ▼
controller receives Int32(2)
  └── robot_id = 'robot2'
  └── creates RobotMapState('robot2')
  └── subscribes to /robot2/map

Both robots now publishing maps
     │
     ▼
Your node merges maps, publishes /merged_map, broadcasts TF

## Files

`controller.py`  a self‑contained NumPy/OpenCV library that aligns two `nav_msgs/OccupancyGrid` maps, scores the alignment, and fuses them into one larger grid. No ROS dependencies, so it can be unit‑tested offline
`map_coordinator.py` ROS 2 node that (1) subscribes to each robot’s map topic, (2) calls `coordinator.merge_maps()`, (3) publishes the fused map on `/merged_map`, and (4) broadcasts static TFs `odom_<anchor> → odom_<follower>`

## ROS 2 Interface  

Subscriptions 
`/<robot_id>/map` (one per robot)  `nav_msgs/msg/OccupancyGrid` Local SLAM map produced by each robot.
Publications 
`/merged_map` `nav_msgs/msg/OccupancyGrid` Latest globally fused map (frame = `odom_1`).
Static TFs
`odom_<anchor> → odom_<follower>`, `tf2_ros/StaticTransformBroadcaster`  Allows planners/visualisers to transform poses between robot odom frames.

> The node creates a separate subscriber for every robot ID listened to.


## Parameters (ROS 2 parameters)

`confidence_threshold` `0.5` Minimum confidence (0‑1) required for a merge to be accepted. Raise for more conservative fusing. 
(hard‑coded) `TRANSFORM_RETRY_CELLS` `500` Number of newly‑known cells a robot must discover before it triggers a new alignment attempt. 
(hard‑coded) `COORDINATION_PERIOD_SEC` `5.0` Seconds between periodic coordination passes when no robot fires a trigger. 

Change the threshold at runtime, e.g.:

```bash
ros2 param set /map_coordinator confidence_threshold 0.7

## Run the Code

### Step 1: Start the container
Navigate to the `vnc-ros` folder on your Windows machine and run:
cmd
cd path\to\vnc-ros
docker compose up -d
docker compose exec ros bash


### Step 2: Source ROS 2
Inside the container:
bash
source /opt/ros/humble/setup.bash

### Step 3: Build the package
bash
cd /root/ros2_ws
colcon build --packages-select merger
source install/setup.bash


### Step 4: Run the node
bash
ros2 run merger map_merger_node


You should see:
[INFO] [map_coordinator]: map_coordinator ready, waiting for robots to register, confidence_threshold=0.5
