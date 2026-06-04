#!/usr/bin/env python3
"""
ROS2 node that wraps the coordinator module into the coordinating
process described in the project proposal.

Responsibilities:
- subscribes to each robot's /SLAM_map (nav_msgs/OccupancyGrid) topic
- tracks how many cells have changed from unknown -> known per robot
- once a robot has converted N cells (TRANSFORM_RETRY_CELLS), attempts a
    new alignment between its grid and every other robot's grid
- publishes the merged occupancy grid on /merged_map every time a new map
    is successfully fused
- broadcasts a static TF from each robot's odom frame to the merged-map
    frame (the "global" frame) so the rest of the system can transform
    poses, target detections, and paths into the shared coordinate space
- pose subscriptions to /robotN/odom are stubbed in but not used by the
    coordinator itself -- the planner node will read those
- confidence threshold defaults to 0.5 (the value used in the demo). It
    can be raised in launch parameters once we tune against real robot
    runs

"""

# NOTE: rclpy etc. are intentionally not imported at module load time so this
# file can live alongside the demo on a machine without ROS2 installed. The
# imports happen inside main().

import math
from typing import Dict, Tuple, Optional

import numpy as np

from merger import map_coordinator as mc


# Constants

DEFAULT_MERGED_MAP_TOPIC   = 'merged_map'
DEFAULT_CONFIDENCE_THRESH  = 0.5
TRANSFORM_RETRY_CELLS      = 500   # try to realign after this many new known cells
MIN_CELLS_BEFORE_FIRST_TRY = 1000  # don't bother before either robot has seen this much

# Time in seconds between coordination passes when no per-robot trigger fires.
COORDINATION_PERIOD_SEC = 5.0
MAP_SUBTOPIC = 'SLAM_map'
ANCHOR_ROBOT_ID = 'robot1'

# Helpers

def occupancy_grid_msg_to_numpy(msg) -> Tuple[np.ndarray, float, float, float]:
    """Convert a nav_msgs/OccupancyGrid message into a 2D int16 numpy array
    plus its (resolution, origin_x, origin_y) metadata.

    Note: OccupancyGrid stores data row-major with y axis going up from the
    origin, matching our mapper.py convention.
    """
    w = msg.info.width
    h = msg.info.height
    arr = np.array(msg.data, dtype=np.int16).reshape(h, w)
    return (
        arr,
        float(msg.info.resolution),
        float(msg.info.origin.position.x),
        float(msg.info.origin.position.y),
    )


def numpy_to_occupancy_grid_msg(grid, resolution, origin_x, origin_y,
                                frame_id, stamp):
    """Build a nav_msgs/OccupancyGrid from a numpy array."""
    # Local imports so this file works without ROS2 installed.
    from nav_msgs.msg import OccupancyGrid, MapMetaData
    from geometry_msgs.msg import Pose, Point, Quaternion

    h, w = grid.shape
    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    info = MapMetaData()
    info.resolution = float(resolution)
    info.width = int(w)
    info.height = int(h)
    info.origin = Pose()
    info.origin.position = Point(x=float(origin_x), y=float(origin_y), z=0.0)
    info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    msg.info = info

    # nav_msgs/OccupancyGrid expects int8 in [-1, 100].
    flat = np.clip(grid, -1, 100).astype(np.int8).flatten().tolist()
    msg.data = flat
    return msg

# Per-robot state held by the coordinator

class RobotMapState:
    #Tracks the latest map and bookkeeping for one robot

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self.latest_grid: Optional[np.ndarray] = None
        self.resolution: Optional[float] = None
        self.origin_x: Optional[float] = None
        self.origin_y: Optional[float] = None
        # Count of cells we've ever seen converted from -1 to known. Used to
        # decide when to retry alignment.
        self.cells_known: int = 0
        self.cells_known_at_last_alignment: int = 0

    def update_from_grid(self, grid, resolution, origin_x, origin_y):
        self.latest_grid = grid
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.cells_known = int((grid >= 0).sum())

    def should_attempt_alignment(self) -> bool:
        if self.latest_grid is None:
            return False
        if self.cells_known < MIN_CELLS_BEFORE_FIRST_TRY:
            return False
        return (self.cells_known
                - self.cells_known_at_last_alignment) >= TRANSFORM_RETRY_CELLS

# The node

def build_node():
    #Construct the MapCoordinatorNode class. Done inside a function so the
    #rclpy import is deferred until main() runs.

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from std_msgs.msg import Int32

    class MapCoordinatorNode(Node):
        def __init__(self):
            super().__init__('map_coordinator')

            self.declare_parameter('confidence_threshold', DEFAULT_CONFIDENCE_THRESH)
            self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)

            self.robot_states: Dict[str, RobotMapState] = {}
            # Currently-known transforms between robot odom frames. Key:
            # (anchor_id, follower_id). Value: 2x3 affine in grid coords plus
            # the resolution it was solved at.
            self.known_transforms: Dict[Tuple[str, str], dict] = {}
            self._map_subs = {} # robot_id -> subscription object

            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._qos = qos

            # Listen for new robot registrations from controller node in mapper
            self._new_robot_sub = self.create_subscription(
                Int32,
                '/new_robot_id',
                self._on_new_robot_registered,
                10,
            )

            # Publishers / broadcasters.
            self._merged_pub = self.create_publisher(
                OccupancyGrid, DEFAULT_MERGED_MAP_TOPIC, qos,
            )
            self._tf_broadcaster = StaticTransformBroadcaster(self)

            # Periodic timer for the case where no single robot trigger fires
            # for a while (e.g. they are both stationary).
            self._timer = self.create_timer(
                COORDINATION_PERIOD_SEC, self._coordination_pass,
            )

            self.get_logger().info(f'map_coordinator ready, waiting for robots to register, 'f'confidence_threshold={self.confidence_threshold}')

        # Subscribers
        def _on_new_robot_registered(self, msg: Int32):
            #Called when mapper coordinator assigns a new integer ID to a robot
            integer_id = msg.data
            robot_id = f'robot{integer_id}' # convert int 1 -> string 'robot1'

            if robot_id in self.robot_states:
                self.get_logger().warn(f'{robot_id} already registered, ignoring.')
                return

            # Add state tracker for this robot
            self.robot_states[robot_id] = RobotMapState(robot_id)

            # Subscribe to this robot's map topic  e.g. /robot1/map
            topic = f'/{robot_id}/{MAP_SUBTOPIC}'
            self.get_logger().info(f'New robot registered: {robot_id}, subscribing to {topic}')

            sub = self.create_subscription(
                OccupancyGrid,
                topic,
                self._make_map_callback(robot_id),
                self._qos,
            )
            self._map_subs[robot_id] = sub      # hold reference to prevent GC

            self.get_logger().info(
                f'Now tracking {len(self.robot_states)} robot(s): '
                f'{list(self.robot_states.keys())}'
            )


        def _make_map_callback(self, robot_id):
            def cb(msg):
                grid, res, ox, oy = occupancy_grid_msg_to_numpy(msg)
                state = self.robot_states[robot_id]
                state.update_from_grid(grid, res, ox, oy)
                if state.should_attempt_alignment():
                    state.cells_known_at_last_alignment = state.cells_known
                    self._coordination_pass(triggering_robot=robot_id)
            return cb

        # Coordination

        def _coordination_pass(self, triggering_robot=None):

            ready = [rid for rid, s in self.robot_states.items()
                    if s.latest_grid is not None]
            if len(ready) < 2:
                return

            # Robot 1 is always the anchor when it participates, so the merged
            # map ends up in the odom_1 frame
            pairs = []
            if ANCHOR_ROBOT_ID in ready:
                for other in ready:
                    if other != ANCHOR_ROBOT_ID:
                        pairs.append((ANCHOR_ROBOT_ID, other))
            else:
                for i, a in enumerate(ready):
                    for b in ready[i + 1:]:
                        pairs.append((a, b))

            for anchor_id, follower_id in pairs:
                self._try_align_pair(anchor_id, follower_id)

        def _try_align_pair(self, anchor_id, follower_id):
            anchor = self.robot_states[anchor_id]
            follower = self.robot_states[follower_id]

            # Both grids must share resolution, if they don't, we can't merge
            # without resampling, which our pipeline does not currently do.
            if abs(anchor.resolution - follower.resolution) > 1e-4:
                self.get_logger().warn(
                    f'resolution mismatch between {anchor_id} '
                    f'({anchor.resolution}) and {follower_id} '
                    f'({follower.resolution}); skipping alignment'
                )
                return

            result = mc.merge_maps(
                anchor.latest_grid, follower.latest_grid,
                resolution_m_per_cell=anchor.resolution,
                confidence_threshold=self.confidence_threshold,
            )

            d = result['diagnostics']
            self.get_logger().info(
                f'align {anchor_id}<-{follower_id}: '
                f'inliers={d.get("inliers", 0)}, '
                f'wall_agree={d.get("wall_agreement", 0):.2f}, '
                f'conf={result["confidence"]:.2f}, '
                f'success={result["success"]}'
            )

            if not result['success']:
                return

            # Record the transform for downstream nodes.
            self.known_transforms[(anchor_id, follower_id)] = {
                'transform_pixels': result['transform_pixels'],
                'transform_meters': result['transform_meters'],
                'resolution': anchor.resolution,
            }

            # Broadcast TF: follower_odom -> anchor_odom (a.k.a. the "global"
            # frame from anchor's perspective). The planner can then chain
            # through this to express the target position in either robot's
            # frame.
            self._broadcast_tf(anchor_id, follower_id, result)

            # Publish merged map in anchor's frame. The merged grid extends
            # past anchor's original bounds, so we adjust the origin
            # accordingly. _expand_canvas was called inside fuse_grids; we
            # recompute the offset here for the message origin.
            merged_grid = result['merged_grid']
            # Recover the canvas offset that fuse_grids applied to A:
            #   A's (0,0) cell is at canvas pixel (-min_x, -min_y) where
            #   min_x, min_y were computed inside _expand_canvas. We get the
            #   same numbers from the public transform.
            M = result['transform_pixels']
            canvas_w, canvas_h, M_a, _ = mc._expand_canvas(
                anchor.latest_grid, None, M, follower.latest_grid.shape,
            )
            # M_a is the affine that puts A into the canvas; its translation
            # column is (-min_x, -min_y).
            offset_x_cells = float(M_a[0, 2])
            offset_y_cells = float(M_a[1, 2])

            new_origin_x = anchor.origin_x - offset_x_cells * anchor.resolution
            new_origin_y = anchor.origin_y - offset_y_cells * anchor.resolution

            msg = numpy_to_occupancy_grid_msg(
                merged_grid,
                resolution=anchor.resolution,
                origin_x=new_origin_x,
                origin_y=new_origin_y,
                frame_id=f'{anchor_id}/odom',
                stamp=self.get_clock().now().to_msg(),
            )
            self._merged_pub.publish(msg)

        def _broadcast_tf(self, anchor_id, follower_id, result):
            #Publish a static TF putting follower's odom frame inside anchor's odom frame.

            # The transform we have is in grid (cell) coordinates and includes
            # the canvas offsets used during fusion. To recover the pure odom
            # -> odom transform we need only the rotation + the metric
            # translation between the two map origins. We already computed
            # transform_meters for the metric (tx, ty, theta) that maps cells
            # in B to cells in A; combined with the resolution and the two
            # grids' origin offsets it becomes the inter-odom transform.
            
            tm = result['transform_meters']
            if tm is None:
                return

            # We rotate first, then translate when mapping a point from
            # follower's odom frame to anchor's odom frame. Build a 3D
            # transform_stamped accordingly.
            tx = tm['tx']
            ty = tm['ty']
            theta = tm['theta']

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f'{anchor_id}/odom'
            t.child_frame_id = f'{follower_id}/odom'
            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(theta/2.0)
            t.transform.rotation.w = math.cos(theta/2.0)

            self._tf_broadcaster.sendTransform(t)
            self.get_logger().info(
                f'broadcasted static TF {anchor_id}/odom <- {follower_id}/odom '
                f'(tx={tx:.3f}m, ty={ty:.3f}m, theta={math.degrees(theta):.2f}deg)'
            )

    return MapCoordinatorNode


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    NodeCls = build_node()
    node = NodeCls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('shutting down')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
