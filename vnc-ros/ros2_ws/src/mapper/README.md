# Author: Charles Lowney 
# Date: 5/10/26

# Final project mapper(CS81)

## Description
This code has a robot map its environment, generating and publishing occupancyGrid data that it creates


## How to execute 

In one terminal from ros2_ws\src:
```bash
docker compose up
```

if docker had completely reset previously, and you need to install dependancies, run the following:
```bash
docker compose exec ros bash
bash install_stage.sh && source ../install/setup.bash
apt update
apt install -y python3-pip
pip3 install anytree
```

you may also need to build the interface packages with:
```bash
docker compose exec ros bash
cd /root/ros2_ws
colcon build
```

in another terminal:
```bash
docker compose exec ros bash
ros2 launch stage_ros2 stage.launch.py world:=/root/ros2_ws/src/mapper/maze enforce_prefixes:=false one_tf_tree:=true
```

in another terminal:
```bash
docker compose exec ros bash
python3 mapper/coordinator.py
```

in another terminal:
```bash
docker compose exec ros bash
python3 mapper/mapper.py
```

go to http://localhost:8080/vnc.html to see simulation


if you don't have the following line in the bash.rc file, you need to run this line every time you open a new ros2 terminal:
```bash
source /opt/ros/humble/setup.bash
```

## Visualizing the Map in rviz2
ensure that the PA4.py code is running (see above section)

in a separate terminal:
```bash
docker compose exec ros bash
rviz2
```

Go to http://localhost:8080/vnc.html and you should see the rviz2 window. 

Click "Add" in the bottom left of the rviz2 screen
Select "By topic" in the display that pops up
Under "/SLAM_map" select "Map"
Hit "OK"

You may now need to restart the PA4 code or set fixed frame to "rosbot/odom"

## implementation of probabilistic mapping
Normal operation has following values in the occupancy grid:
- Unknown (-1): Unexplored regions.
- Freespace (0): Cells along the ray that are clear of obstacles.
- Occupied (100): Cells where a laser "hit" was recorded.

When `PROBABILISTIC_MAPPING` is False, the robot operates in normal mode, as outlined above. But when `PROBABILISTIC_MAPPING` is True, the robot implements a recursive Bayesian update using log-odds to make the map resilient to sensor noise. Where the value in the grid falls in a range from 0 to 100, and reprensents the probability (percent) that the cell is occupied. A value of -1 still means that the cell has not been explored.  

# Coordinator node

## topic list
- nav_path_`id`
- pose_`id`
- SLAM_map_`id`
- id_active_`id`

## services
the coordinator runs 2 services:
- Type: GetUniqueID, Name: `'get_unique_id'`
- Type: GetNewFrontierPath, Name: `'get_path'`