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

The project convention is `/SLAM_map_<id>`, for example `/SLAM_map_1`. The node also accepts `/robot<id>/SLAM_map` as a compatibility alias so older branches do not starve the merger, but new code should not publish maps there.

The node creates map subscriptions after receiving robot IDs. `/new_robot_id` uses transient-local QoS with enough history for both demo robots, so the merger can recover registrations even if it starts slightly late.

## Outputs

```text
/merged_map            nav_msgs/OccupancyGrid
/merge_status          std_msgs/String
/tf_static             transform between robot odom frames after alignment
```

`/merged_map` uses the anchor robot odom frame, typically `robot1/odom`.

## Parameters

```text
confidence_threshold      default in integrated launch: 0.68
use_metadata_origin_prior default: true
map_topic_template        default: /SLAM_map_{id}
map_topic_alias_templates default: [/{robot_id}/SLAM_map]
```

Example:

```bash
ros2 run merger map_merger_node --ros-args -p confidence_threshold:=0.7
```

## Notes

- Alignment is attempted as soon as both maps have enough known cells, then periodically as the maps grow.
- The integrated Gazebo demo uses OccupancyGrid origin metadata as a trusted prior. This keeps the merge stable when both odom frames are already world-aligned and prevents ORB from rotating a partial map into a false match.
- Feature-based ORB/RANSAC alignment is still scored, but it must pass wall agreement, overlap, and occupancy-conflict gates before publication.
- Once a transform is accepted, the node reuses it to keep publishing a fuller `/merged_map` even when a later feature retry is inconclusive.
- If confidence is too low and no cached transform exists, the node keeps publishing local maps separately and waits for more structure.
- `/merge_status` reports whether the node is waiting for maps, rejected an alignment, or published a merged map.
- In RViz, `/merged_map` appears once a successful alignment has been accepted.
- For report evidence, `report_merged_contribution_map.png` redraws the saved local map grids with robot 1 cells, robot 2 cells, overlap, occupied cells, and the final A* path in separate colors.
