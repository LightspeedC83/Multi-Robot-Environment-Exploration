# merger

Map coordination package for the multi-robot demo. It listens for robot registrations, subscribes to each robot's local occupancy grid, estimates the transform between robot odom frames, and publishes a merged map when confidence is high enough.

## Node

```bash
ros2 run merger map_merger_node
```

The integrated launch starts this node automatically:

```bash
ros2 launch final_project_cv integrated_two_robot_demo.launch.py
```

## Inputs

```text
/new_robot_id          std_msgs/Int32
/SLAM_map_<id>         nav_msgs/OccupancyGrid
```

The node creates one map subscription per registered robot.

## Outputs

```text
/merged_map            nav_msgs/OccupancyGrid
/tf_static             transform between robot odom frames after alignment
```

`/merged_map` uses the anchor robot odom frame, typically `robot1/odom`.

## Parameters

```text
confidence_threshold   default: 0.5
map_topic_template     default: /SLAM_map_{id}
```

Example:

```bash
ros2 run merger map_merger_node --ros-args -p confidence_threshold:=0.7
```

## Notes

- Alignment is attempted periodically and after maps gain enough new known cells.
- If confidence is too low, the node keeps publishing local maps separately and waits for more structure.
- In RViz, `/merged_map` appears once a successful alignment has been accepted.
