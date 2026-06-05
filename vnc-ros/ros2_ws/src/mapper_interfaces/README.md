# mapper_interfaces

Custom ROS 2 service interfaces shared by the mapper and coordinator.

## Services

`GetUniqueID.srv`

- Called by each mapper at startup.
- Returns a stable integer robot ID for topic naming.
- The coordinator also announces new IDs on `/new_robot_id` so the merger can subscribe to `/SLAM_map_<id>`.

`GetNewFrontierPath.srv`

- Called by a mapper when it needs a new path.
- The coordinator publishes the actual path on `/nav_path_<id>`.
- The service response reports whether a path was published.

## Topic Convention

Mapper/coordinator topics use flat ID suffixes: `/SLAM_map_<id>`, `/pose_<id>`, `/id_active_<id>`, and `/nav_path_<id>`. Robot hardware and CV topics use `/robot<id>/...` namespaces.

## Build

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mapper_interfaces
source install/setup.bash
```

Normally this package is built as part of the full workspace:

```bash
colcon build --symlink-install
```
